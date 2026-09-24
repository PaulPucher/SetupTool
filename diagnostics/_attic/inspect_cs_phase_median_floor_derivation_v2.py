# WORK PACKAGE: 100 Hz time base + corrected floor derivation, Phase 1
# (2026-09-02). Third pass at the CS window floors -- same phase-median
# bootstrap method as the (falsified) second pass, but the sampled
# population is now restricted to CORNERING stretches only: inside a
# real corner bracket (entry_1_brake start to exit_5 end) OR |ay| above
# gps_course_anchor_max_ay_g (0.05g -- already data-derived on this same
# car as the boundary where the |ay| distribution starts rising steeply
# toward cornering values, config's own provenance note). This directly
# targets the root cause the Phase 4 re-run diagnosed: the second pass's
# bootstrap sampled stretches UNIFORMLY from the whole moving population,
# diluting the noise level with slower-varying non-cornering data, while
# the REAL evaluation population (actual corner apex/turn-in dynamics)
# hits any small span floor almost immediately.
#
# Runs at whatever grid rate prepare_vehicle_state resolves for this file
# (100 Hz on Dubai, PHASE 0 of the same work order) -- sample counts
# below are native to THAT grid; convert to seconds via state[
# 'sample_rate_hz'] for the physical-duration config values.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

# Candidates now span a WIDER range than the second pass (which only
# tested up to n=40) since the corrected population is expected to be
# noisier (concentrated in genuinely fast-changing regions) and may need
# a larger floor than the falsified n=10/span=0.01 -- counts are in
# samples at whatever grid rate this run resolves (100 Hz on Dubai).
CANDIDATES = [
    (10, 0.010), (15, 0.015), (20, 0.020), (25, 0.025), (30, 0.030),
    (40, 0.040), (50, 0.050), (60, 0.060), (80, 0.080), (100, 0.100),
]
REPRESENTATIVE_L_SAMPLES_AT_100HZ = [80, 112, 152, 240, 324]  # 2x the prior pass's own L set, matching the 2x grid rate
N_STRETCHES = 150
BOOTSTRAP_B = 150
RNG_SEED = 42
TARGET_REL_STD = 0.15
COMPUTATIONAL_MAX_WINDOW = 4000  # generous, computational-only bound -- see prior pass's own note


def _ols_slope(alpha, Fy):
    a_mean = np.mean(alpha)
    f_mean = np.mean(Fy)
    denom = np.sum((alpha - a_mean) ** 2)
    if denom < 1e-12:
        return np.nan
    return float(np.sum((alpha - a_mean) * (Fy - f_mean)) / denom)


_INDEX_AS_METRES = None  # set once in main() -- np.arange(n_total), reused as a synthetic
                          # 1-unit-per-sample "distance" array so the existing s_m/max_window_m
                          # cap mechanism can enforce a computational-only sample bound during
                          # the search itself (truncating the RESULT afterward would not save
                          # the search's own cost -- the search must be capped, not the answer).


def _per_sample_slopes(alpha, Fy, stretch_start, L, n, span):
    slopes = np.full(L, np.nan)
    for k in range(L):
        i = stretch_start + k
        start = reconstruct_cs_window_start(alpha, i, n, span, s_m=_INDEX_AS_METRES,
                                             max_window_m=COMPUTATIONAL_MAX_WINDOW)
        window_alpha = alpha[start:i]
        window_Fy = Fy[start:i]
        if len(window_alpha) < n:
            continue
        achieved_span = window_alpha.max() - window_alpha.min()
        if achieved_span < span:
            continue
        slopes[k] = _ols_slope(window_alpha, window_Fy)
    return slopes


def phase_median_bootstrap(alpha, Fy, cornering_mask, n, span, L, rng):
    n_total = len(alpha)
    rel_stds = []
    no_signal_stretches = 0
    achieved_lengths = []
    attempts = 0
    max_attempts = N_STRETCHES * 60
    while len(rel_stds) + no_signal_stretches < N_STRETCHES and attempts < max_attempts:
        attempts += 1
        stretch_start = int(rng.integers(max(n, 200), n_total - L))
        if not np.all(cornering_mask[stretch_start:stretch_start + L]):
            continue
        slopes = _per_sample_slopes(alpha, Fy, stretch_start, L, n, span)
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
        for i in range(stretch_start, stretch_start + L):
            start = reconstruct_cs_window_start(alpha, i, n, span, s_m=_INDEX_AS_METRES,
                                                 max_window_m=COMPUTATIONAL_MAX_WINDOW)
            achieved_lengths.append(i - start)
    return rel_stds, no_signal_stretches, achieved_lengths, attempts


def main():
    global _INDEX_AS_METRES
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    sample_rate_hz = state["sample_rate_hz"]
    _INDEX_AS_METRES = np.arange(len(state["time"]), dtype=float)
    print(f"grid_rate_status: {state['grid_rate_status']}, sample_rate_hz: {sample_rate_hz}")

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

    t = state["time"]
    ay_g = np.abs(state["ay_mps2"]) / 9.81
    ay_threshold_g = params["stability_estimation"]["gps_course_anchor_max_ay_g"]

    in_corner = np.zeros(len(t), dtype=bool)
    for c in data.get("corners", []):
        start_t, _ = c["segments"]["entry_1_brake"]
        _, end_t = c["segments"]["exit_5"]
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi > lo:
            in_corner[lo:hi] = True

    cornering_mask = moving & (in_corner | (ay_g > ay_threshold_g))
    print(f"cornering_mask fraction of moving samples: {cornering_mask[moving].mean():.1%} "
          f"(in_corner OR'd with |ay|>{ay_threshold_g}g)")

    rng = np.random.default_rng(RNG_SEED)

    for axle_label, alpha_key, Fy_key in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        alpha = slip[alpha_key]
        Fy = forces[Fy_key]
        print(f"\n{'=' * 96}\nAXLE: {axle_label}\n{'=' * 96}")

        for L in REPRESENTATIVE_L_SAMPLES_AT_100HZ:
            print(f"\n-- phase length L={L} samples ({L / sample_rate_hz:.2f} s @ {sample_rate_hz:.0f} Hz) --")
            print(f"  {'(n,span)':>16} {'n_s':>8} {'n_stretches':>12} {'no_signal':>10} {'attempts':>9} "
                  f"{'median_rel_std':>15} {'p75_rel_std':>12} {'mean_achieved_n':>16}")
            for n, span in CANDIDATES:
                rel_stds, no_signal, achieved_lengths, attempts = phase_median_bootstrap(
                    alpha, Fy, cornering_mask, n, span, L, rng
                )
                n_s = n / sample_rate_hz
                if not rel_stds:
                    print(f"  {(n, span)!s:>16} {n_s:>8.3f} {0:>12} {no_signal:>10} {attempts:>9} "
                          f"{'--':>15} {'--':>12} {'--':>16}")
                    continue
                med = float(np.median(rel_stds))
                p75 = float(np.percentile(rel_stds, 75))
                mean_len = float(np.mean(achieved_lengths)) if achieved_lengths else float("nan")
                flag = "  <== clears 15%" if med <= TARGET_REL_STD else ""
                print(f"  {(n, span)!s:>16} {n_s:>8.3f} {len(rel_stds):>12} {no_signal:>10} {attempts:>9} "
                      f"{med:>15.3f} {p75:>12.3f} {mean_len:>16.1f}{flag}")


if __name__ == "__main__":
    main()
