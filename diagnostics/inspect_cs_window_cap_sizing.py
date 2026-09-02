# WORK PACKAGE: CS validity repair, part A, Phase 1 (continued) -- given
# candidate re-derived floors (cs_min_window_samples, cs_min_slip_angle_
# span_rad), measure the window-LENGTH distribution the existing growth
# loop (reconstruct_cs_window_start) would actually produce across the
# whole real session under those floors, with NO cap -- sizing input for
# Phase 2's cs_max_window_samples. Also reports typical corner-phase
# durations (samples) as an independent locality yardstick: the cap
# should not typically exceed what a single corner phase spans, or the
# window stops being a per-corner statistic. Diagnostics only, no
# config/estimator change.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
CANDIDATE_FLOORS = [(10, 0.02), (65, 0.03), (100, 0.04)]
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
N_SAMPLE = 1500  # random moving indices per (axle, floor) -- census-equivalent for percentiles, tractable runtime
RNG_SEED = 42


def _percentiles(arr):
    arr = np.asarray(arr, dtype=float)
    return {p: float(np.percentile(arr, p)) for p in (50, 75, 90, 95, 99, 100)}


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    t = state["time"]

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to size the cap on it")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    moving_idx = np.where(moving)[0]
    rng = np.random.default_rng(RNG_SEED)

    for axle_label, alpha_key in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
        alpha = slip[alpha_key]
        print(f"\n{'=' * 78}\nAXLE: {axle_label}\n{'=' * 78}")
        for min_window, min_span in CANDIDATE_FLOORS:
            idx = moving_idx[moving_idx >= min_window]
            idx = rng.choice(idx, size=min(N_SAMPLE, len(idx)), replace=False)
            lengths = np.array([
                i - reconstruct_cs_window_start(alpha, int(i), min_window, min_span)
                for i in idx
            ], dtype=float)
            p = _percentiles(lengths)
            print(f"  floors (N={min_window}, span={min_span:.3f} rad): "
                  f"n_samples_checked={len(lengths)}  "
                  f"p50={p[50]:.0f} p75={p[75]:.0f} p90={p[90]:.0f} p95={p[95]:.0f} "
                  f"p99={p[99]:.0f} max={p[100]:.0f}")

    print(f"\n{'=' * 78}\nCORNER-PHASE DURATIONS (samples), for locality comparison\n{'=' * 78}")
    for phase in PHASE_KEYS:
        durs = []
        for c in data.get("corners", []):
            start_t, end_t = c["segments"][phase]
            if end_t < start_t:
                continue
            lo = int(np.searchsorted(t, start_t, side="left"))
            hi = int(np.searchsorted(t, end_t, side="right"))
            if hi > lo:
                durs.append(hi - lo)
        if durs:
            p = _percentiles(durs)
            print(f"  {phase:>16}: n={len(durs):3d}  p50={p[50]:.0f} p75={p[75]:.0f} "
                  f"p90={p[90]:.0f} max={p[100]:.0f}")


if __name__ == "__main__":
    main()
