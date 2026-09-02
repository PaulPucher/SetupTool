# WORK PACKAGE: CS validity repair, part A, Phase 1 REVISION (2026-09-02).
# Supersedes diagnostics/inspect_cs_window_floor_derivation.py's own
# criterion: that script bootstrapped a SINGLE WINDOW's own slope
# variance and found no natural knee (monotonic improvement with size,
# per textbook OLS variance scaling) -- correct on its own terms, but the
# wrong target. What actually feeds classify_fn is a PHASE-LEVEL MEDIAN
# over many per-sample windowed estimates, not any single window alone --
# the median already averages out per-window noise across O(30-200)
# samples in a realistic phase, so a much smaller per-window floor can
# still yield a stable phase median. This script re-derives the floors
# against THAT target directly.
#
# METHOD: for each candidate (n, span) floor pair and each representative
# phase length L (drawn from the real measured phase-duration spectrum,
# diagnostics/inspect_cs_window_cap_sizing.py's own CORNER-PHASE
# DURATIONS block), draw many real contiguous L-sample stretches from the
# session's moving population. For each stretch, compute the per-sample
# windowed-OLS slope at every one of its L samples using the REAL
# production growth mechanism (reconstruct_cs_window_start(alpha, i, n,
# span), no cap -- the cap is Phase 2's separate locality concern, not a
# precision concern, and is re-derived independently in this same
# revision). Bootstrap-resample (with replacement) the L per-sample
# slopes B times, take nanmedian of each resample -- this reproduces
# summarise_corners's own _stats() median under exactly the same
# finite/NaN mix a real phase of that length would show. Relative std of
# the bootstrap-median distribution vs the stretch's own point-estimate
# median is the target statistic; report its MEDIAN across many stretches,
# per (n, span, L). Smallest (n, span) clearing <=15% relative std at the
# SHORTEST physically-meaningful, consistently-populated phase length
# governs the final floor choice (conservative: longer phases only get
# more stable from the same floor, via more averaging).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

# Paired (n, span) candidates -- n and span scale together (a window that
# clears a bigger span at ~50 Hz racing alpha rates naturally also
# accumulates more samples), so a diagonal sweep is the relevant one here,
# not a full independent 2-D grid (the user asked for "the smallest n AND
# span" as a joint answer, not two separate optima).
CANDIDATES = [
    (5, 0.005), (6, 0.006), (8, 0.008), (10, 0.010), (12, 0.012),
    (15, 0.015), (20, 0.020), (25, 0.025), (30, 0.030), (40, 0.040),
]
# Representative phase lengths, from the real measured corner-phase-
# duration spectrum (inspect_cs_window_cap_sizing.py CORNER-PHASE
# DURATIONS: exit_4 p25~40-50/p50=56, exit_5 p50=76, entry_2_turnin
# p50=162). entry_1_brake excluded here (item 4 -- its own p50=1 sample
# is physically degenerate, not a floor-choice question) and apex_3
# excluded (superseded by the distance-based apex_region, Phase 3).
REPRESENTATIVE_L = [40, 56, 76, 120, 162]
N_STRETCHES = 150
BOOTSTRAP_B = 150
RNG_SEED = 42
TARGET_REL_STD = 0.15


def _ols_slope(alpha, Fy):
    a_mean = np.mean(alpha)
    f_mean = np.mean(Fy)
    denom = np.sum((alpha - a_mean) ** 2)
    if denom < 1e-12:
        return np.nan
    return float(np.sum((alpha - a_mean) * (Fy - f_mean)) / denom)


def _per_sample_slopes(alpha, Fy, stretch_start, L, n, span):
    slopes = np.full(L, np.nan)
    for k in range(L):
        i = stretch_start + k
        # COMPUTATIONAL bound only (not a design decision): without it, a
        # near-flat-alpha stretch forces an unbounded backward search
        # (exactly the runaway pathology Phase 2's own cap exists to
        # prevent in production) -- 2000 samples is generous relative to
        # every candidate here (max 40) and never binds for a genuine
        # cornering stretch, only for pathological flat regions this
        # analysis would have discarded as no-signal anyway.
        start = reconstruct_cs_window_start(alpha, i, n, span, max_window=2000)
        window_alpha = alpha[start:i]
        window_Fy = Fy[start:i]
        if len(window_alpha) < n:
            continue
        achieved_span = window_alpha.max() - window_alpha.min()
        if achieved_span < span:
            continue
        slopes[k] = _ols_slope(window_alpha, window_Fy)
    return slopes


def phase_median_bootstrap(alpha, Fy, moving, n, span, L, rng):
    n_total = len(alpha)
    rel_stds = []
    no_signal_stretches = 0
    achieved_lengths = []
    attempts = 0
    max_attempts = N_STRETCHES * 30
    while len(rel_stds) + no_signal_stretches < N_STRETCHES and attempts < max_attempts:
        attempts += 1
        stretch_start = int(rng.integers(max(n, 200), n_total - L))
        if not np.all(moving[stretch_start:stretch_start + L]):
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
            achieved_lengths.append(i - reconstruct_cs_window_start(alpha, i, n, span))
    return rel_stds, no_signal_stretches, achieved_lengths


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
    sample_rate_hz = state["sample_rate_hz"]
    print(f"log sample_rate_hz (this file, this pipeline's own common grid) = {sample_rate_hz}")
    print("REFERENCE: the chair's own default (10 samples) at ITS 100 Hz rate = 0.100 s physical window.")

    for axle_label, alpha_key, Fy_key in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        alpha = slip[alpha_key]
        Fy = forces[Fy_key]
        print(f"\n{'=' * 96}\nAXLE: {axle_label}\n{'=' * 96}")

        for L in REPRESENTATIVE_L:
            print(f"\n-- phase length L={L} samples ({L / sample_rate_hz:.2f} s @ {sample_rate_hz} Hz) --")
            print(f"  {'(n,span)':>16} {'n_s':>15} {'n_stretches':>12} {'no_signal':>10} "
                  f"{'median_rel_std':>15} {'p75_rel_std':>12} {'mean_achieved_n':>16}")
            for n, span in CANDIDATES:
                rel_stds, no_signal, achieved_lengths = phase_median_bootstrap(alpha, Fy, moving, n, span, L, rng)
                n_s = n / sample_rate_hz
                if not rel_stds:
                    print(f"  {(n, span)!s:>16} {n_s:>15.3f} {0:>12} {no_signal:>10} {'--':>15} {'--':>12} {'--':>16}")
                    continue
                med = float(np.median(rel_stds))
                p75 = float(np.percentile(rel_stds, 75))
                mean_len = float(np.mean(achieved_lengths)) if achieved_lengths else float("nan")
                flag = "  <== clears 15%" if med <= TARGET_REL_STD else ""
                print(f"  {(n, span)!s:>16} {n_s:>15.3f} {len(rel_stds):>12} {no_signal:>10} "
                      f"{med:>15.3f} {p75:>12.3f} {mean_len:>16.1f}{flag}")


if __name__ == "__main__":
    main()
