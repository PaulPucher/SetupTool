# Part B2 (NIS gate band decision + v3 diagnostics work order, 2026-09-03):
# C7/C15 "unstable yaw at brake" investigation. Read-only -- no config/
# production changes. Runs the exact production pipeline (same call
# sequence as ui/views/outing_form.py's StabilityAnalysisThread.run(),
# live config sideslip_source) on v3, then for every per-lap instance of
# stable_corner_id 7 and 15 reports: entry_1_brake phase sample count,
# actual brake pressure in that phase (log_pbrake_f via state["brake_f_
# bar"], same channel Module 5 already resolves), the stability
# statistic's own n/median for that phase, and whether cs_phase_min_
# valid_samples-style gating would already suppress it if applied.
# Disposable per CLAUDE.md's diagnostics/ rule.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "GT3_PRC_MLA-v3.txt"
TARGET_CORNERS = (7, 15)


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    sideslip_source = params["stability_estimation"].get("sideslip_source", "kinematic")
    print(f"sideslip_source (live config) = {sideslip_source}")

    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, sideslip_source, csv_path=RAW_FILE
    )
    print(f"fallback_used={fallback_used} fallback_reason={fallback_reason} "
          f"gate_verdict={gate_verdict['verdict'] if gate_verdict else None}")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state)

    t = state["time"]
    brake_f_bar = state["brake_f_bar"]
    cs_phase_min_valid_samples = params["stability_estimation"]["cs_phase_min_valid_samples"]

    corner_by_id_and_lap = {}
    for c in corners:
        cid = c.get("stable_corner_id")
        if cid in TARGET_CORNERS:
            corner_by_id_and_lap[(cid, c["lap_number"])] = c

    for s in summaries:
        cid = s.get("stable_corner_id")
        if cid not in TARGET_CORNERS:
            continue
        lap = s["lap_number"]
        c = corner_by_id_and_lap.get((cid, lap))
        phase = s["phases"]["entry_1_brake"]
        n = phase["n_samples"]
        stab_stat = phase["stability_observed_Nm_per_deg"]

        # Reproduce entry_1_brake's own time slice (no apex-window special
        # case for this phase) to pull brake_f_bar over the SAME samples
        # summarise_corners used for the stability statistic.
        start_t, end_t = c["segments"]["entry_1_brake"]
        if end_t < start_t:
            brake_vals = np.array([])
        else:
            lo = int(np.searchsorted(t, start_t, side="left"))
            hi = int(np.searchsorted(t, end_t, side="right"))
            moving = state["moving_mask"][lo:hi]
            brake_vals = brake_f_bar[lo:hi][moving]

        brake_max = float(np.nanmax(brake_vals)) if len(brake_vals) else float("nan")
        brake_mean = float(np.nanmean(brake_vals)) if len(brake_vals) else float("nan")
        would_gate_on_n = n < cs_phase_min_valid_samples

        print(f"\nC{cid} lap {lap}: entry_1_brake n_samples={n}, "
              f"stability_observed median={stab_stat['median']:.1f} Nm/deg (n={stab_stat['n']}), "
              f"brake_f_bar in phase: max={brake_max:.2f} bar, mean={brake_mean:.2f} bar, "
              f"n_brake_samples={len(brake_vals)}, "
              f"would_gate_on_min_valid_samples({cs_phase_min_valid_samples})={would_gate_on_n}")


if __name__ == "__main__":
    main()
