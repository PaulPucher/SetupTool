# Follow-up to the FINAL Phase 4 report: per-LAP breakdown (not just the
# worst-lap-per-corner aggregate) for the flagged corners, apex_region-
# substituted, under the final config. Answers "which laps" C4 (and
# C1/C2/C3/C6) actually read negative/positive on.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
FLAGGED = (1, 2, 3, 4, 6)


def worst_of_5_apex_region(s):
    apex_region = s.get("apex_region")
    csfs, csrs = [], []
    for phase, p in s["phases"].items():
        if phase == "apex_3" and apex_region is not None:
            cs_f = apex_region["cs_ratio_f"]["median"]
            cs_r = apex_region["cs_ratio_r"]["median"]
        else:
            cs_f = p["cs_ratio_f"]["median"]
            cs_r = p["cs_ratio_r"]["median"]
        if cs_f == cs_f:
            csfs.append((cs_f, phase))
        if cs_r == cs_r:
            csrs.append((cs_r, phase))
    worst_f = min(csfs, key=lambda x: x[0]) if csfs else (float("nan"), None)
    worst_r = min(csrs, key=lambda x: x[0]) if csrs else (float("nan"), None)
    return worst_f, worst_r


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason})")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    summaries = summarise_corners(data["corners"], cs, stab, state)

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    by_id = {}
    for s in summaries:
        lap = laps_by_number.get(s["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        by_id.setdefault(s.get("stable_corner_id"), []).append(s)

    for cid in FLAGGED:
        print(f"\n-- C{cid} -- per-lap apex_region-substituted worst-of-5-phases --")
        for s in sorted(by_id.get(cid, []), key=lambda x: x["lap_number"]):
            (wf, pf), (wr, pr) = worst_of_5_apex_region(s)
            print(f"   lap {s['lap_number']}: CSf={wf:+.3f} @ {pf}   CSr={wr:+.3f} @ {pr}")


if __name__ == "__main__":
    main()
