# Follow-up, Item 2: re-check the earlier INCONCLUSIVE fuel-drift finding
# (thesis_notes.md "Damper package... Phase 2b": mean straight-line total
# per lap looked confounded by speed differences between laps' own
# straight-line samples, not cleanly attributable to fuel burn). Now that
# modules.wheel_loads.estimate_session_corrected_axle_totals supplies a
# session-fit aero coefficient (c_session), this script additionally
# reports an AERO-NORMALISED total per lap (subtracting the speed-
# dependent aero term relative to the session's own mean straight-line
# v^2, so laps with different average straight-line speeds become
# comparable) alongside the raw total -- report only, no config/
# production change, no conclusion asserted beyond what the numbers show.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, load_car_data, prepare_vehicle_state
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, estimate_session_corrected_axle_totals,
    combine_with_reconstruction_and_fallback, CORNERS,
)
from modules.stability_analysis import estimate_lateral_forces, estimate_vertical_loads

RAW_FILE = "GT3_PRC_MLA-v3.txt"


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    if state is None or car_data is None:
        print("cannot run -- missing state or car_data.json")
        return

    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    session_corrected = estimate_session_corrected_axle_totals(state, damper_result, params)
    forces = estimate_lateral_forces(state, params)
    fz_static = estimate_vertical_loads(state, forces, params)
    static_fallback_fz = {c: fz_static[f"fz_{c}_N"] for c in CORNERS}
    fz_axle_totals = {"fz_f_N": session_corrected["fz_f_N"], "fz_r_N": session_corrected["fz_r_N"]}
    combined = combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz)
    total_fz = sum(combined[c]["fz_N"] for c in CORNERS)

    v = state["v_mps"]
    ax = state["ax_mps2"]
    ay = state["ay_mps2"]
    t = state["time"]
    moving = v >= params["stability_estimation"]["moving_speed_min_mps"]
    straight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)

    c_session = session_corrected["c_session_N_per_mps2"]
    v_ref_sq = float(np.mean(v[straight] ** 2))
    print(f"session c_session={c_session:.4f} N/(m/s)^2, straight-line mean v^2={v_ref_sq:.1f} (m/s)^2 "
          f"(v_ref={np.sqrt(v_ref_sq)*3.6:.0f} km/h)")

    print(f"\n{'lap':>4} {'n':>6} {'mean_v_kmh':>11} {'raw_total_N':>12} {'aero_adj_total_N':>17}")
    for lap in data["laps"]:
        if not lap["is_valid_for_analysis"] and lap["lap_number"] != 0:
            continue
        lap_mask = straight & (t >= lap["start_time"]) & (t <= lap["end_time"])
        if lap_mask.sum() < 5:
            continue
        mean_v = float(np.mean(v[lap_mask]))
        mean_v_sq = float(np.mean(v[lap_mask] ** 2))
        raw_total = float(np.mean(total_fz[lap_mask]))
        # Remove the speed-dependent aero term's excess over the session's
        # own reference v^2 -- what the total would read at the session's
        # typical straight-line speed, isolating non-speed (fuel-mass)
        # drift from the already-known speed-driven aero variation.
        aero_adj_total = raw_total - c_session * (mean_v_sq - v_ref_sq)
        print(f"{lap['lap_number']:>4} {int(lap_mask.sum()):>6} {mean_v*3.6:>11.0f} {raw_total:>12.1f} {aero_adj_total:>17.1f}")


if __name__ == "__main__":
    main()
