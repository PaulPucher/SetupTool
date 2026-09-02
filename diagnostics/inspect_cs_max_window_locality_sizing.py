# WORK PACKAGE: CS validity repair, part A, Phase 1 REVISION, item 3
# (2026-09-02). Re-sizes cs_max_window_m as a LOCALITY bound, not from
# the observed whole-session window-length distribution (the original
# Phase 1 approach, which was dominated by irrelevant straight-line
# samples that need no cap at all since they carry no cornering signal
# anyway). METHOD: under the FINAL (small) re-derived floor, measure the
# natural (uncapped) window's own METRE extent at real, moving,
# cornering-relevant samples only (restricted to real corner brackets,
# so straights/kerbs never enter this specific measurement). Report the
# median footprint in metres; cs_max_window_m is set at 1.5x that median
# -- a window needing much more than 1.5x its own typical local extent to
# satisfy the floor is no longer describing local cornering behaviour.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

# FINAL floor, third pass (100 Hz time base + corrected floor
# derivation): the chair's own original physical window (0.2 s, 0.02 rad)
# re-derived at samples at THIS file's own 100 Hz grid (20 samples) --
# validated directly against real Phase 4 numbers (diagnostics/inspect_
# cs_floor_candidate_validation.py), not chosen from the phase-median
# bootstrap alone (that criterion cannot distinguish a reproducible-but-
# biased median from a genuinely accurate one -- see thesis_notes.md).
FINAL_MIN_WINDOW_SAMPLES = 20
FINAL_MIN_SPAN_RAD = 0.02

LOCALITY_MULTIPLIER = 1.5
N_SAMPLE = 2000
RNG_SEED = 42


def _percentiles(arr):
    arr = np.asarray(arr, dtype=float)
    return {p: float(np.percentile(arr, p)) for p in (10, 25, 50, 75, 90, 95, 99, 100)}


def main():
    if FINAL_MIN_WINDOW_SAMPLES is None or FINAL_MIN_SPAN_RAD is None:
        raise SystemExit("set FINAL_MIN_WINDOW_SAMPLES / FINAL_MIN_SPAN_RAD from the phase-median "
                          "bootstrap run's own output before running this script")

    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    s_m = state.get("s_m")

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

    # Restrict to real corner brackets only -- the population this cap
    # actually needs to stay local WITHIN, not the whole session (which
    # includes long straights that need no cap at all since CS_ratio
    # there is not meaningfully "local to a corner" in the first place).
    t = state["time"]
    in_corner = np.zeros(len(t), dtype=bool)
    for c in data.get("corners", []):
        bs, be = c.get("bracket_start_m"), c.get("bracket_end_m")
        if bs is None or be is None or s_m is None:
            continue
        start_t, _ = c["segments"]["entry_1_brake"]
        _, end_t = c["segments"]["exit_5"]
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi > lo:
            in_corner[lo:hi] = True

    rng = np.random.default_rng(RNG_SEED)
    moving_idx = np.where(moving & in_corner)[0]
    moving_idx = moving_idx[moving_idx >= FINAL_MIN_WINDOW_SAMPLES]

    for axle_label, alpha_key in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
        alpha = slip[alpha_key]
        idx = rng.choice(moving_idx, size=min(N_SAMPLE, len(moving_idx)), replace=False)
        footprints_m = []
        footprints_n = []
        for i in idx:
            start = reconstruct_cs_window_start(alpha, int(i), FINAL_MIN_WINDOW_SAMPLES, FINAL_MIN_SPAN_RAD)
            achieved_span = alpha[start:i].max() - alpha[start:i].min() if i > start else 0.0
            if achieved_span < FINAL_MIN_SPAN_RAD:
                continue  # no-signal at these floors -- excluded, not a real footprint
            footprints_n.append(i - start)
            if s_m is not None and np.isfinite(s_m[i - 1]) and np.isfinite(s_m[start]) and s_m[i - 1] >= s_m[start]:
                footprints_m.append(s_m[i - 1] - s_m[start])

        p_n = _percentiles(footprints_n)
        print(f"\n-- {axle_label}: natural (uncapped) window length, real corner-bracket samples only, "
              f"n_measured={len(footprints_n)} --")
        print(f"  samples: p10={p_n[10]:.0f} p25={p_n[25]:.0f} p50={p_n[50]:.0f} p75={p_n[75]:.0f} "
              f"p90={p_n[90]:.0f} p95={p_n[95]:.0f} p99={p_n[99]:.0f} max={p_n[100]:.0f}")
        if footprints_m:
            p_m = _percentiles(footprints_m)
            print(f"  metres:  p10={p_m[10]:.1f} p25={p_m[25]:.1f} p50={p_m[50]:.1f} p75={p_m[75]:.1f} "
                  f"p90={p_m[90]:.1f} p95={p_m[95]:.1f} p99={p_m[99]:.1f} max={p_m[100]:.1f}")
            print(f"  proposed cs_max_window_m ({LOCALITY_MULTIPLIER}x median footprint): "
                  f"{LOCALITY_MULTIPLIER * p_m[50]:.1f} m")
        else:
            print("  metres: no s_m-valid footprints measured")


if __name__ == "__main__":
    main()
