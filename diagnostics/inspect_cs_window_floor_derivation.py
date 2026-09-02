# WORK PACKAGE: CS validity repair, part A, Phase 1 -- re-derive the
# window-growth floors (cs_min_window_samples, cs_min_slip_angle_span_rad)
# and the linear-region end (cs_linear_slip_threshold_rad) against THIS
# car's own ekf_auto_pacejka alpha/Fy data, replacing values that were
# never re-checked after the estimator changed from kinematic. Diagnostics
# only -- no config/estimator change made here (Phase 2 writes to config).
#
# METHOD (Tier B, standard technique -- bootstrap resampling of a
# windowed OLS slope estimator to find where its own sampling variance
# stabilises as a function of window size):
#
# (a) N-sweep: for a grid of FIXED window lengths N, take a large sample
#     of real (alpha, Fy) windows ending at real, moving, non-kerb sample
#     indices (both axles, whole session -- no corner pre-selection, since
#     that is exactly the population the production estimator scans).
#     For each window, bootstrap-resample its own N (alpha, Fy) pairs with
#     replacement B times and refit the OLS slope each time; the spread of
#     those B slopes estimates the sampling variance of a single fit at
#     that window length. Report the median relative bootstrap std
#     (std/|slope|, scale-free) across all sampled windows, per N.
#
# (b) span-sweep: for a grid of TARGET alpha spans, grow a window
#     backward from the same real end-indices (mirroring the production
#     growth loop) until the achieved span first reaches the target (or
#     the session start is hit), then bootstrap that window the same way.
#     Reports the median relative bootstrap std AND the median resulting
#     N, per target span -- this is what lets a chosen span floor be
#     translated back into an expected sample count.
#
# (c) Linear-region end: bins |alpha| (both axles folded to the same
#     sign convention as the tyre curve -- Fy and alpha negated together
#     for alpha<0 samples, exploiting the tyre curve's expected odd
#     symmetry) into fixed-width bins and fits a local OLS slope per bin.
#     Reports the bin index where the local slope first departs from the
#     near-zero-alpha reference slope (first two bins) by more than a
#     stated relative tolerance.
#
# All three curves are printed as tables; NO value is written to config
# by this script (Phase 2 does that, using this script's own output).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

N_GRID = [8, 10, 12, 15, 20, 25, 30, 40, 50, 65, 80, 100, 130, 160, 200]
SPAN_GRID_RAD = [0.010, 0.015, 0.020, 0.030, 0.045, 0.065, 0.090, 0.120, 0.160]
N_SAMPLE_WINDOWS = 500  # end-indices sampled per grid point, per axle
BOOTSTRAP_B = 200
RNG_SEED = 42

LINEAR_MAX_ALPHA_RAD = 0.15   # ~8.6 deg, comfortably past any expected onset
LINEAR_DEPARTURE_TOL = 0.10   # 10% relative slope departure from the near-zero reference
LINEAR_WINDOW_HALF_WIDTH_RAD = 0.02  # sliding window is 2x this wide -- matches the
    # derived min_span floor (0.04 rad) rather than an arbitrary narrow bin, since a
    # bin much narrower than the estimator's own min_span floor would just reproduce
    # the same small-span sampling noise Phase 1a/1b already diagnosed.
LINEAR_STEP_RAD = 0.005


def _ols_slope(alpha, Fy):
    a_mean = np.mean(alpha)
    f_mean = np.mean(Fy)
    denom = np.sum((alpha - a_mean) ** 2)
    if denom < 1e-12:
        return np.nan
    return float(np.sum((alpha - a_mean) * (Fy - f_mean)) / denom)


def _bootstrap_rel_std(alpha, Fy, rng):
    n = len(alpha)
    idx = rng.integers(0, n, size=(BOOTSTRAP_B, n))
    a_boot = alpha[idx]
    f_boot = Fy[idx]
    a_mean = a_boot.mean(axis=1, keepdims=True)
    f_mean = f_boot.mean(axis=1, keepdims=True)
    denom = np.sum((a_boot - a_mean) ** 2, axis=1)
    valid = denom > 1e-12
    slopes = np.full(BOOTSTRAP_B, np.nan)
    slopes[valid] = (
        np.sum((a_boot - a_mean) * (f_boot - f_mean), axis=1)[valid] / denom[valid]
    )
    slopes = slopes[np.isfinite(slopes)]
    if len(slopes) < BOOTSTRAP_B // 2:
        return np.nan, np.nan
    point_slope = _ols_slope(alpha, Fy)
    if not np.isfinite(point_slope) or abs(point_slope) < 1e-6:
        return np.nan, point_slope
    return float(np.std(slopes) / abs(point_slope)), point_slope


def n_sweep(alpha, Fy, end_indices, rng):
    rows = []
    for N in N_GRID:
        rel_stds = []
        spans = []
        for i in end_indices:
            if i < N:
                continue
            window_alpha = alpha[i - N:i]
            window_Fy = Fy[i - N:i]
            if not (np.all(np.isfinite(window_alpha)) and np.all(np.isfinite(window_Fy))):
                continue
            rel_std, _slope = _bootstrap_rel_std(window_alpha, window_Fy, rng)
            if np.isfinite(rel_std):
                rel_stds.append(rel_std)
                spans.append(float(window_alpha.max() - window_alpha.min()))
        if rel_stds:
            rows.append({
                "N": N, "n_windows": len(rel_stds),
                "median_rel_std": float(np.median(rel_stds)),
                "p75_rel_std": float(np.percentile(rel_stds, 75)),
                "median_span_rad": float(np.median(spans)),
            })
    return rows


def span_sweep(alpha, Fy, end_indices, rng):
    rows = []
    for target_span in SPAN_GRID_RAD:
        rel_stds = []
        ns = []
        for i in end_indices:
            start = i - 1
            if start < 1:
                continue
            while start > 0:
                span = np.max(alpha[start:i]) - np.min(alpha[start:i])
                if span >= target_span:
                    break
                start -= 1
            window_alpha = alpha[start:i]
            window_Fy = Fy[start:i]
            n = len(window_alpha)
            if n < 5 or not (np.all(np.isfinite(window_alpha)) and np.all(np.isfinite(window_Fy))):
                continue
            achieved_span = float(window_alpha.max() - window_alpha.min())
            if achieved_span < target_span * 0.9:
                continue  # session-start truncation, did not actually reach the target
            rel_std, _slope = _bootstrap_rel_std(window_alpha, window_Fy, rng)
            if np.isfinite(rel_std):
                rel_stds.append(rel_std)
                ns.append(n)
        if rel_stds:
            rows.append({
                "target_span_rad": target_span, "n_windows": len(rel_stds),
                "median_rel_std": float(np.median(rel_stds)),
                "p75_rel_std": float(np.percentile(rel_stds, 75)),
                "median_N": float(np.median(ns)),
            })
    return rows


def linear_region_end(alpha, Fy, moving):
    a = alpha[moving]
    f = Fy[moving]
    finite = np.isfinite(a) & np.isfinite(f)
    a, f = a[finite], f[finite]
    # Fold to a single-sign convention exploiting the tyre curve's expected
    # odd symmetry: Fy(-alpha) = -Fy(alpha). Sign of alpha decides the fold.
    sign = np.sign(a)
    sign[sign == 0] = 1.0
    a_folded = a * sign
    f_folded = f * sign

    centres = np.arange(LINEAR_WINDOW_HALF_WIDTH_RAD, LINEAR_MAX_ALPHA_RAD, LINEAR_STEP_RAD)
    rows = []
    for c in centres:
        lo, hi = c - LINEAR_WINDOW_HALF_WIDTH_RAD, c + LINEAR_WINDOW_HALF_WIDTH_RAD
        mask = (a_folded >= lo) & (a_folded < hi)
        n = int(mask.sum())
        slope = _ols_slope(a_folded[mask], f_folded[mask]) if n >= 30 else np.nan
        rows.append({"centre_rad": float(c), "n": n, "slope_N_per_rad": slope})

    # Reference: the FIRST sliding window (centred at the floor's own half-width,
    # i.e. [0, 2*half_width] rad) -- wide enough to average out the near-zero-alpha
    # sign-fold noise a narrower reference bin would carry.
    ref_slope = rows[0]["slope_N_per_rad"] if rows and np.isfinite(rows[0]["slope_N_per_rad"]) else None
    if ref_slope is None:
        return rows, None, None
    onset_centre = None
    for r in rows:
        s = r["slope_N_per_rad"]
        if not np.isfinite(s):
            continue
        rel_dep = abs(s - ref_slope) / abs(ref_slope) if abs(ref_slope) > 1e-9 else np.inf
        r["rel_departure_from_ref"] = rel_dep
        if onset_centre is None and rel_dep > LINEAR_DEPARTURE_TOL:
            onset_centre = r["centre_rad"] - LINEAR_WINDOW_HALF_WIDTH_RAD  # window's own leading edge
    return rows, ref_slope, onset_centre


def print_table(title, rows, cols):
    print(f"\n-- {title} --")
    header = "  ".join(f"{c:>16}" for c in cols)
    print(f"  {header}")
    for r in rows:
        line = "  ".join(
            f"{r[c]:16.5f}" if isinstance(r[c], float) else f"{r[c]:16d}"
            for c in cols
        )
        print(f"  {line}")


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to derive floors on it")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)

    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask

    rng = np.random.default_rng(RNG_SEED)
    moving_idx = np.where(moving)[0]
    moving_idx = moving_idx[moving_idx >= max(N_GRID)]  # skip session-start indices too short for the largest N

    for axle_label, alpha_key, Fy_key in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        alpha = slip[alpha_key]
        Fy = forces[Fy_key]

        sample_idx = rng.choice(moving_idx, size=min(N_SAMPLE_WINDOWS, len(moving_idx)), replace=False)

        print(f"\n{'=' * 78}\nAXLE: {axle_label}\n{'=' * 78}")

        n_rows = n_sweep(alpha, Fy, sample_idx, rng)
        print_table(
            f"{axle_label}: bootstrap relative slope std vs FIXED window length N",
            n_rows, ["N", "n_windows", "median_rel_std", "p75_rel_std", "median_span_rad"],
        )

        span_rows = span_sweep(alpha, Fy, sample_idx, rng)
        print_table(
            f"{axle_label}: bootstrap relative slope std vs TARGET alpha span",
            span_rows, ["target_span_rad", "n_windows", "median_rel_std", "p75_rel_std", "median_N"],
        )

        lin_rows, ref_slope, onset_centre = linear_region_end(alpha, Fy, moving)
        half_w = LINEAR_WINDOW_HALF_WIDTH_RAD
        print(f"\n-- {axle_label}: linear-region local-slope-vs-|alpha| sliding windows "
              f"(width={2 * half_w:.3f} rad, step={LINEAR_STEP_RAD:.3f} rad, reference slope "
              f"(first window, [0,{2 * half_w:.3f}) rad) = "
              f"{ref_slope:.1f} N/rad, departure tolerance {LINEAR_DEPARTURE_TOL:.0%}) --")
        for r in lin_rows:
            if r["n"] == 0 or not np.isfinite(r["slope_N_per_rad"]):
                continue
            dep = r.get("rel_departure_from_ref")
            lo, hi = r["centre_rad"] - half_w, r["centre_rad"] + half_w
            print(f"  [{lo:.4f},{hi:.4f}) rad  n={r['n']:5d}  "
                  f"slope={r['slope_N_per_rad']:10.1f} N/rad  "
                  f"rel_departure={dep:.3f}")
        if onset_centre is not None:
            print(f"  ONSET (first window's leading edge exceeding {LINEAR_DEPARTURE_TOL:.0%} departure): "
                  f"{onset_centre:.4f} rad")
        else:
            print("  ONSET: none found within scanned range")


if __name__ == "__main__":
    main()
