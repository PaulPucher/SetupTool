# Attribution-only follow-up to the circularity check's Section 4
# flagged-instance counts (pass_1, calibrated setting): which corners,
# which phase, which speed class -- no judgement of physical
# correctness, that needs the driver report and is a separate turn.
# Read-only, no config/production change.
#
# CAVEAT carried from earlier this session: entry_1_brake's start
# (modules/corner_analysis.py _build_corner, brake_start_t) is found by
# an off-throttle lookback across the entire prior time history and can
# extend up the preceding straight, overlapping neighbouring corners'
# own brackets -- phase attribution below is indicative, not exact.

import sys

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PASS_ID = sys.argv[1] if len(sys.argv) > 1 else "pass_1"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
laps = data.get("laps", [])
corners = data.get("corners", [])

beta_a = estimate_sideslip(state, params)
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id=PASS_ID)
beta_c = ekf_result["beta"]
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)

stab = estimate_yaw_moment_stability(state, beta_a, params, laps)
summaries = summarise_corners(corners, cs_c, stab, state)

STRONG_CSF = params["classification"]["STRONG_CSF"]["value"]
STRONG_CSR = params["classification"]["STRONG_CSR"]["value"]
MODERATE_CSF = params["classification"]["MODERATE_CSF"]["value"]
MODERATE_CSR = params["classification"]["MODERATE_CSR"]["value"]


def _severity(val, strong, moderate):
    if val < strong:
        return "strong"
    if val < moderate:
        return "moderate"
    return None


flagged_f, flagged_r = [], []
for s in summaries:
    csfs = [(phase, p["cs_ratio_f"]["median"]) for phase, p in s["phases"].items()
            if p["cs_ratio_f"]["median"] == p["cs_ratio_f"]["median"]]
    csrs = [(phase, p["cs_ratio_r"]["median"]) for phase, p in s["phases"].items()
            if p["cs_ratio_r"]["median"] == p["cs_ratio_r"]["median"]]

    if csfs:
        worst_phase_f, worst_val_f = min(csfs, key=lambda pv: pv[1])
        sev_f = _severity(worst_val_f, STRONG_CSF, MODERATE_CSF)
        if sev_f:
            flagged_f.append({
                "stable_corner_id": s.get("stable_corner_id"), "lap_number": s["lap_number"],
                "severity": sev_f, "value": worst_val_f, "phase": worst_phase_f,
                "speed_class": s.get("speed_class"),
            })

    if csrs:
        worst_phase_r, worst_val_r = min(csrs, key=lambda pv: pv[1])
        sev_r = _severity(worst_val_r, STRONG_CSR, MODERATE_CSR)
        if sev_r:
            flagged_r.append({
                "stable_corner_id": s.get("stable_corner_id"), "lap_number": s["lap_number"],
                "severity": sev_r, "value": worst_val_r, "phase": worst_phase_r,
                "speed_class": s.get("speed_class"),
            })

print("=" * 100)
print(f"Flagged worst-phase-per-corner-instance attribution, {PASS_ID} (calibrated setting)")
print("=" * 100)
print("CAVEAT: entry_1_brake can extend up the preceding straight and overlap neighbouring")
print("corners' own brackets (off-throttle lookback, modules/corner_analysis.py) -- phase")
print("attribution below is indicative, not exact.")
print()

for axle_name, flagged in (("front", flagged_f), ("rear", flagged_r)):
    print(f"--- {axle_name}: {len(flagged)} flagged instances ---")
    for item in sorted(flagged, key=lambda d: (d["stable_corner_id"], d["lap_number"])):
        print(f"  C{item['stable_corner_id']} lap{item['lap_number']}  severity={item['severity']:8s}  "
              f"value={item['value']:+.3f}  worst_phase={item['phase']:15s}  speed_class={item['speed_class']}")

    by_corner = {}
    for item in flagged:
        by_corner.setdefault(item["stable_corner_id"], []).append(item)
    print(f"  distinct stable_corner_ids flagged: {len(by_corner)}  "
          f"(instances per corner: {sorted((len(v) for v in by_corner.values()), reverse=True)})")
    print(f"  cluster/scatter: {'CLUSTERS' if len(by_corner) <= len(flagged) / 2 else 'SCATTERS'} "
          f"-- {len(by_corner)} distinct corners carrying {len(flagged)} flagged instances")
    print()
