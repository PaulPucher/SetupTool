# Investigation, diagnostics only, PROPOSAL Part 2 input: for the 12
# corner/axle cases already investigated (C1/C2/C3/C4/C8/C9, front+rear,
# thesis_notes.md "Mechanism investigation: wholesale-negative CS_ratio
# under ekf_auto_pacejka"), trace Stage 2 (min-across-5-phases, per lap)
# then Stage 3 (min-across-laps, "worst phase of worst lap", explicitly
# signed off) to find which SPECIFIC phase produced each case's Stage-2
# minimum, and reports that phase's own valid-sample count against the
# local CS_ratio regression window's own footprint (samples) -- the
# input a within-phase validity gate (phase reports NO SIGNAL if its
# sample count is below k times the local window footprint) would be
# sized from. No config or estimator change; no gate value proposed
# here, numbers only.
#
# Follow-up section (same run): recomputes Stage 2 EXCLUDING any phase
# whose cs-valid sample count is below 1.5x its own local window
# footprint, across all 5 phases of every lap (not just the previously
# -identified worst one) -- answers whether the pre-registered C4-vs-
# artifact separation holds once apex_3's structurally-fixed 11-sample
# budget is gated out everywhere it would fail on its own terms.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners, reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
TARGET_IDS = (1, 2, 3, 4, 8, 9)
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
CANDIDATE_K = (1.5, 2.0, 3.0)


def _phase_slice(t, start_t, end_t, is_apex, apex_half_window_samples):
    # Mirrors summarise_corners's own private _phase_slice exactly
    # (modules/stability_analysis.py) -- not a reimplementation of new
    # logic, just re-exposed here since that helper is nested/private.
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if is_apex and hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window_samples)
        hi = min(len(t), centre + apex_half_window_samples + 1)
    return slice(lo, hi)


def _stage2_worst_phase(phases_dict, axle_key):
    # Exact reproduction of ui/views/outing_form.py::_classify_corner's
    # running-min loop (lines ~475-487) for one axle: min across the 5
    # phase medians, NaN-skipped via the same "val == val" idiom.
    worst_val = 1.0
    worst_phase = None
    for phase in PHASE_KEYS:
        val = phases_dict[phase][axle_key]["median"]
        if val == val and val < worst_val:
            worst_val = val
            worst_phase = phase
    return worst_val, worst_phase


def _phase_footprint(c, phase, alpha_arr, cs_ratio_arr, t, moving, min_window, min_span,
                      apex_half_window_samples):
    start_t, end_t = c["segments"][phase]
    sl = _phase_slice(t, start_t, end_t, phase == "apex_3", apex_half_window_samples)
    phase_moving = moving[sl]
    idx = np.where(phase_moving)[0] + sl.start
    cs_vals = cs_ratio_arr[idx]
    valid_idx = idx[np.isfinite(cs_vals)]
    footprints = np.array(
        [int(i) - reconstruct_cs_window_start(alpha_arr, int(i), min_window, min_span) for i in valid_idx],
        dtype=float,
    )
    return valid_idx.size, footprints


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    t = state["time"]
    moving = state["moving_mask"]

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to run")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    summaries = summarise_corners(data["corners"], cs, stab, state)

    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]
    apex_half_window_samples = se["apex_half_window_samples"]

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}

    # (stable_corner_id, lap_number) -> raw corner dict, for c["segments"]
    corners_by_id_lap = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None:
            corners_by_id_lap[(sid, c["lap_number"])] = c

    summaries_by_id = {}
    for s in summaries:
        lap = laps_by_number.get(s["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        summaries_by_id.setdefault(s.get("stable_corner_id"), []).append(s)

    alpha_arrs = {"f": slip["alpha_f_filt"], "r": slip["alpha_r_filt"]}
    cs_ratio_arrs = {"f": cs["CS_ratio_f"], "r": cs["CS_ratio_r"]}

    for cid in TARGET_IDS:
        instances = summaries_by_id.get(cid, [])
        if not instances:
            print(f"\n-- C{cid}: no valid lap instances --")
            continue

        for axle_label, axle_key, alpha_key in (("front", "cs_ratio_f", "f"), ("rear", "cs_ratio_r", "r")):
            per_lap = []
            for s in instances:
                worst_val, worst_phase = _stage2_worst_phase(s["phases"], axle_key)
                per_lap.append((worst_val, worst_phase, s["lap_number"]))

            finite = [row for row in per_lap if row[1] is not None]
            if not finite:
                print(f"\n-- C{cid} {axle_label}: no finite Stage-2 value in any lap --")
                continue
            worst_val, worst_phase, worst_lap = min(finite, key=lambda row: row[0])

            phase_stat = None
            for s in instances:
                if s["lap_number"] == worst_lap:
                    phase_stat = s["phases"][worst_phase][axle_key]
                    n_samples_raw = s["phases"][worst_phase]["n_samples"]
                    break

            c = corners_by_id_lap.get((cid, worst_lap))
            start_t, end_t = c["segments"][worst_phase]
            sl = _phase_slice(t, start_t, end_t, worst_phase == "apex_3", apex_half_window_samples)

            alpha_arr = alpha_arrs[alpha_key]
            cs_ratio_arr = cs_ratio_arrs[alpha_key]
            phase_moving = moving[sl]
            idx = np.where(phase_moving)[0] + sl.start
            cs_vals = cs_ratio_arr[idx]
            valid_idx = idx[np.isfinite(cs_vals)]

            footprints = []
            for i in valid_idx:
                start = reconstruct_cs_window_start(alpha_arr, int(i), min_window, min_span)
                footprints.append(int(i) - start)
            footprints = np.array(footprints, dtype=float)

            print(f"\n-- C{cid} {axle_label} ({MODE}) --")
            print(f"  per-lap Stage-2 (worst phase, worst value): "
                  f"{[(l, p, round(v, 3)) for v, p, l in per_lap]}")
            print(f"  Stage-3 (min-then-min): worst_lap={worst_lap}, worst_phase={worst_phase}, "
                  f"worst_value={worst_val:.3f}")
            print(f"  worst phase's own stat block: median={phase_stat['median']:.3f} "
                  f"p25={phase_stat['p25']:.3f} p75={phase_stat['p75']:.3f} "
                  f"n(cs_ratio-valid)={phase_stat['n']}  n_samples(raw moving)={n_samples_raw}")
            if footprints.size:
                print(f"  local window footprint (samples), over the phase's {footprints.size} "
                      f"cs_ratio-valid samples: mean={footprints.mean():.1f} "
                      f"min={footprints.min():.0f} max={footprints.max():.0f}")
                for k in CANDIDATE_K:
                    threshold = k * footprints.mean()
                    survives = phase_stat["n"] >= threshold
                    print(f"    k={k}: gate={threshold:.1f} samples -> "
                          f"phase n={phase_stat['n']} {'SURVIVES' if survives else 'DROPPED (NaN)'}")
            else:
                print("  local window footprint: no cs_ratio-valid samples in this phase (n=0)")

            # --- follow-up: Stage 2 recomputed with the k=1.5 within-phase
            # gate applied across ALL 5 phases of every lap, not just the
            # previously-identified worst one. A phase is excluded (NaN)
            # from this lap's min if its own cs-valid n < 1.5 * its own
            # mean local footprint.
            GATE_K = 1.5
            per_lap_gated = []
            for s in instances:
                c = corners_by_id_lap.get((cid, s["lap_number"]))
                best_val, best_phase, best_n = None, None, None
                for phase in PHASE_KEYS:
                    median = s["phases"][phase][axle_key]["median"]
                    if median != median:
                        continue
                    n_valid, footprints_p = _phase_footprint(
                        c, phase, alpha_arrs[alpha_key], cs_ratio_arrs[alpha_key],
                        t, moving, min_window, min_span, apex_half_window_samples,
                    )
                    if n_valid == 0 or footprints_p.size == 0:
                        continue
                    if n_valid < GATE_K * footprints_p.mean():
                        continue  # gated out -- reports NO SIGNAL for this phase
                    if best_val is None or median < best_val:
                        best_val, best_phase, best_n = median, phase, n_valid
                per_lap_gated.append((best_val, best_phase, best_n, s["lap_number"]))

            finite_gated = [row for row in per_lap_gated if row[0] is not None]
            print(f"  GATED (k=1.5, all phases) per-lap survivors: "
                  f"{[(l, p, round(v, 3), n) for v, p, n, l in per_lap_gated]}")
            if not finite_gated:
                print("  GATED Stage-3: NO SIGNAL in any lap -- every phase of every lap gated out")
            else:
                gv, gp, gn, gl = min(finite_gated, key=lambda row: row[0])
                delta = gv - worst_val
                print(f"  GATED Stage-3 (min-then-min): worst_lap={gl}, worst_phase={gp}, "
                      f"worst_value={gv:.3f}, n={gn}  |  delta vs original apex-driven value: "
                      f"{delta:+.3f} (original {worst_val:.3f} @ {worst_phase})")


if __name__ == "__main__":
    main()
