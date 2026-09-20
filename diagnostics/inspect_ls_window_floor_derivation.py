# Metrology extension Phase 2a: LS_ratio window-floor derivation, applying
# the CS_ratio playbook (thesis_notes.md "CS validity repair, part A,
# Phase 1 REVISION: floors re-derived against the phase-level MEDIAN").
# SAME METHOD, on kappa/Fx instead of alpha/Fy, on BOTH real sessions (the
# work order's own explicit instruction -- CS's original derivation was
# Dubai-only): bootstrap-resampled (B=150) phase-level MEDIAN stability
# (what summarise_corners's own _stats() actually feeds classify_fn/the
# frame, not a single window's own sampling variance) as a function of
# candidate (min_window_samples, min_slip_span) pairs, at representative
# corner-phase lengths L drawn from THIS data's own real corner-phase-
# duration spectrum -- measured fresh here, not copied from CS's own L
# set (a different, unrelated corner-phase-duration measurement would
# only coincidentally match; measuring it directly is cheap and honest).
#
# Fast path: no EKF/beta fit needed (kappa/Fx do not depend on sideslip),
# confirmed by reading modules/longitudinal_forces.py directly.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import _filtered, reconstruct_ls_window_start

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

CANDIDATES = [
    (5, 0.002), (8, 0.003), (10, 0.004), (12, 0.005), (15, 0.006),
    (20, 0.008), (25, 0.010), (30, 0.012), (40, 0.016), (50, 0.020),
]
N_STRETCHES = 150
BOOTSTRAP_B = 150
RNG_SEED = 42
TARGET_REL_STD = 0.15
COMPUTATIONAL_MAX_WINDOW_M = 1.0e6  # generous, computational-only -- never binds a genuine stretch


def _ols_slope(x, y):
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    denom = np.sum((x - x_mean) ** 2)
    if denom < 1e-12:
        return np.nan
    return float(np.sum((x - x_mean) * (y - y_mean)) / denom)


def _per_sample_slopes(kappa, fx, stretch_start, L, n_win, span, s_m):
    slopes = np.full(L, np.nan)
    for k in range(L):
        i = stretch_start + k
        start = reconstruct_ls_window_start(kappa, i, n_win, span, s_m=s_m, max_window_m=COMPUTATIONAL_MAX_WINDOW_M)
        window_kappa = kappa[start:i]
        window_fx = fx[start:i]
        if len(window_kappa) < n_win:
            continue
        achieved_span = window_kappa.max() - window_kappa.min()
        if achieved_span < span:
            continue
        slopes[k] = _ols_slope(window_kappa, window_fx)
    return slopes


def _eligible_starts(demand_mask, L, lo_bound):
    """All start indices >= lo_bound where demand_mask[start:start+L] is
    entirely True, via a prefix-sum rolling-window count (no rejection
    sampling -- a sparse demand_mask would otherwise make uniform random
    sampling attempt-starved, confirmed the slow way first: an earlier
    version of this script used random rejection sampling and produced
    zero output after 5+ minutes on the real data)."""
    n = len(demand_mask)
    if n - L <= lo_bound:
        return np.array([], dtype=int)
    prefix = np.concatenate(([0], np.cumsum(demand_mask.astype(np.int32))))
    starts = np.arange(lo_bound, n - L)
    counts = prefix[starts + L] - prefix[starts]
    return starts[counts == L]


def phase_median_bootstrap(kappa, fx, eligible_starts, n_win, span, L, rng, s_m):
    rel_stds = []
    no_signal_stretches = 0
    attempts = 0
    if len(eligible_starts) == 0:
        return rel_stds, no_signal_stretches, 0
    max_attempts = N_STRETCHES * 3
    pool = rng.choice(eligible_starts, size=min(max_attempts, len(eligible_starts) * 5), replace=True)
    for stretch_start in pool:
        if len(rel_stds) + no_signal_stretches >= N_STRETCHES:
            break
        attempts += 1
        slopes = _per_sample_slopes(kappa, fx, int(stretch_start), L, n_win, span, s_m)
        finite = slopes[np.isfinite(slopes)]
        if len(finite) < 3:
            no_signal_stretches += 1
            continue
        point_median = float(np.median(finite))
        if abs(point_median) < 1e-6:
            no_signal_stretches += 1
            continue
        idx = rng.integers(0, L, size=(BOOTSTRAP_B, L))
        resampled = slopes[idx]
        boot_medians = np.nanmedian(resampled, axis=1)
        boot_medians = boot_medians[np.isfinite(boot_medians)]
        if len(boot_medians) < BOOTSTRAP_B // 2:
            no_signal_stretches += 1
            continue
        rel_stds.append(float(np.std(boot_medians) / abs(point_median)))
    return rel_stds, no_signal_stretches, attempts


def _measure_representative_L(data, state, sample_rate_hz):
    """Real corner-phase-duration spectrum for THIS file, measured fresh
    (not copied from CS's own set) -- 5 representative lengths spanning
    the real distribution, same "5 values across the spectrum" shape CS
    used, picked at percentiles 10/30/50/70/90 of the real per-phase
    sample-count population (all 5 phase types pooled, entry_1_brake
    included this time -- unlike CS's own exclusion, entry_1_brake is
    exactly where LS's own braking signal lives, not a degenerate case
    here).
    """
    t = state["time"]
    durations = []
    for c in data.get("corners", []):
        for phase, (start_t, end_t) in c["segments"].items():
            if end_t <= start_t:
                continue
            lo = int(np.searchsorted(t, start_t, side="left"))
            hi = int(np.searchsorted(t, end_t, side="right"))
            if hi > lo:
                durations.append(hi - lo)
    durations = np.array(durations)
    pcts = [10, 30, 50, 70, 90]
    L = sorted(set(int(round(np.percentile(durations, p))) for p in pcts))
    L = [max(l, 20) for l in L]
    return L, durations


def run_session(label, raw_file):
    print(f"\n{'#'*100}\n{label}\n{'#'*100}")
    params = load_parameters()
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    sr = state["sample_rate_hz"]
    s_m = state.get("s_m")
    print(f"sample_rate_hz={sr}, s_m available={s_m is not None}")

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip = estimate_slip_ratio(state, data["channels"], params)
    ls_cfg = params["longitudinal_stiffness"]

    kappa_f_filt = _filtered(slip["kappa_f"], sr, ls_cfg["cutoff_hz"])
    kappa_r_filt = _filtered(slip["kappa_r"], sr, ls_cfg["cutoff_hz"])
    fx_f_filt = _filtered(long_forces["fx_f_N"], sr, ls_cfg["cutoff_hz"])
    fx_r_filt = _filtered(long_forces["fx_r_N"], sr, ls_cfg["cutoff_hz"])

    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    ax = state["ax_mps2"]
    # Longitudinal-DEMAND population: real braking/traction activity, not
    # the whole moving session (mirrors CS's own restriction to genuine
    # cornering stretches, same rationale -- a flat/no-demand stretch
    # dilutes the noise level with data no real corner ever produces).
    demand_mask = moving & (np.abs(ax) > 1.0)
    print(f"demand_mask (|ax|>1.0, moving, non-kerb) fraction of moving samples: "
          f"{demand_mask[moving].mean():.1%}")

    L_values, durations = _measure_representative_L(data, state, sr)
    print(f"real corner-phase duration population: n={len(durations)}, "
          f"p10/30/50/70/90={np.percentile(durations, [10,30,50,70,90])}")
    print(f"representative L values (samples): {L_values}")

    print("precomputing eligible stretch-start indices per L (no rejection sampling)...", flush=True)
    eligible_by_L = {}
    for L in L_values:
        eligible = _eligible_starts(demand_mask, L, lo_bound=200)
        eligible_by_L[L] = eligible
        print(f"  L={L}: {len(eligible)} eligible stretch starts", flush=True)

    rng = np.random.default_rng(RNG_SEED)
    results = {}
    for axle_label, kappa, fx in (("front", kappa_f_filt, fx_f_filt), ("rear", kappa_r_filt, fx_r_filt)):
        print(f"\n{'='*96}\nAXLE: {axle_label}\n{'='*96}", flush=True)
        axle_results = {}
        for L in L_values:
            print(f"\n-- phase length L={L} samples ({L/sr:.2f} s) --", flush=True)
            print(f"  {'(n,span)':>16} {'n_stretches':>12} {'no_signal':>10} {'attempts':>9} "
                  f"{'median_rel_std':>15} {'p75_rel_std':>12}", flush=True)
            for n_win, span in CANDIDATES:
                rel_stds, no_signal, attempts = phase_median_bootstrap(
                    kappa, fx, eligible_by_L[L], n_win, span, L, rng, s_m)
                if not rel_stds:
                    print(f"  {(n_win, span)!s:>16} {0:>12} {no_signal:>10} {attempts:>9} {'--':>15} {'--':>12}",
                          flush=True)
                    continue
                med = float(np.median(rel_stds))
                p75 = float(np.percentile(rel_stds, 75))
                flag = "  <== clears 15% (median)" if med <= TARGET_REL_STD else ""
                flag2 = "  <== clears 15% (P75)" if p75 <= TARGET_REL_STD else ""
                print(f"  {(n_win, span)!s:>16} {len(rel_stds):>12} {no_signal:>10} {attempts:>9} "
                      f"{med:>15.3f} {p75:>12.3f}{flag}{flag2}", flush=True)
                axle_results[(L, n_win, span)] = (med, p75)
        results[axle_label] = axle_results
    return results, L_values


def main():
    all_results = {}
    for label, raw_file in (("DUBAI", DUBAI_FILE), ("V3", V3_FILE)):
        results, L_values = run_session(label, raw_file)
        all_results[label] = (results, L_values)

    print(f"\n{'#'*100}\nSHORTEST-L SUMMARY, both sessions, both axles "
          f"(governing case = smallest (n,span) clearing P75<=15% at shortest L)\n{'#'*100}")
    for label, (results, L_values) in all_results.items():
        shortest_L = L_values[0]
        for axle_label, axle_results in results.items():
            print(f"\n{label} {axle_label}, shortest L={shortest_L}:")
            for n_win, span in CANDIDATES:
                key = (shortest_L, n_win, span)
                if key in axle_results:
                    med, p75 = axle_results[key]
                    print(f"  (n={n_win}, span={span}): median={med:.3f}, P75={p75:.3f}"
                          + ("  <== CLEARS" if p75 <= TARGET_REL_STD else ""))


if __name__ == "__main__":
    main()
