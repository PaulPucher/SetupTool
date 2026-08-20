# Threshold-comparability check: the classification thresholds
# (STRONG_CSF/CSR, MODERATE_CSF/CSR) were derived against the KINEMATIC
# CS_ratio distribution. This reports the worst-phase-per-instance
# CS_ratio distribution for both kinematic and pass_1 side by side, and
# an instance-level overlap between the two flagged sets, so the
# flagged-count jump can be read as "shift" vs "new" without deriving
# or applying any new threshold. Read-only, no config change.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
laps = data.get("laps", [])
corners = data.get("corners", [])

beta_a = estimate_sideslip(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
beta_c = ekf_result["beta"]
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)
cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)

stab = estimate_yaw_moment_stability(state, beta_a, params, laps)

STRONG_CSF = params["classification"]["STRONG_CSF"]["value"]
STRONG_CSR = params["classification"]["STRONG_CSR"]["value"]
MODERATE_CSF = params["classification"]["MODERATE_CSF"]["value"]
MODERATE_CSR = params["classification"]["MODERATE_CSR"]["value"]


def _worst_per_instance(cs):
    summaries = summarise_corners(corners, cs, stab, state)
    out_f, out_r = [], []
    for s in summaries:
        csfs = [p["cs_ratio_f"]["median"] for p in s["phases"].values() if p["cs_ratio_f"]["median"] == p["cs_ratio_f"]["median"]]
        csrs = [p["cs_ratio_r"]["median"] for p in s["phases"].values() if p["cs_ratio_r"]["median"] == p["cs_ratio_r"]["median"]]
        key = (s.get("stable_corner_id"), s["lap_number"])
        if csfs:
            out_f.append((key, min(csfs)))
        if csrs:
            out_r.append((key, min(csrs)))
    return out_f, out_r


worst_f_a, worst_r_a = _worst_per_instance(cs_a)
worst_f_c, worst_r_c = _worst_per_instance(cs_c)

print("=" * 100)
print("Worst-phase-per-instance CS_ratio percentiles: kinematic vs pass_1")
print("=" * 100)
for axle_name, kin, ekf, strong_t, mod_t in (
    ("front", worst_f_a, worst_f_c, STRONG_CSF, MODERATE_CSF),
    ("rear", worst_r_a, worst_r_c, STRONG_CSR, MODERATE_CSR),
):
    kin_vals = np.array([v for _, v in kin])
    ekf_vals = np.array([v for _, v in ekf])
    pcts = [5, 10, 25, 50, 75, 90, 95]
    kin_p = np.percentile(kin_vals, pcts)
    ekf_p = np.percentile(ekf_vals, pcts)
    print(f"--- {axle_name} (n={len(kin_vals)} kinematic, n={len(ekf_vals)} pass_1; "
          f"thresholds STRONG<{strong_t} MODERATE<{mod_t}) ---")
    print(f"  {'pct':>6}" + "".join(f"{p:>8}" for p in pcts))
    print(f"  {'kinematic':>6}" + "".join(f"{v:8.3f}" for v in kin_p))
    print(f"  {'pass_1':>6}" + "".join(f"{v:8.3f}" for v in ekf_p))
    print()

    # instance-level overlap of flagged sets (strong or moderate)
    def _flagged_set(items, strong_t, mod_t):
        return {key for key, v in items if v < mod_t}

    kin_flagged = _flagged_set(kin, strong_t, mod_t)
    ekf_flagged = _flagged_set(ekf, strong_t, mod_t)
    both = kin_flagged & ekf_flagged
    kin_only = kin_flagged - ekf_flagged
    ekf_only = ekf_flagged - kin_flagged

    print(f"  flagged instance sets ({axle_name}): kinematic n={len(kin_flagged)}  pass_1 n={len(ekf_flagged)}")
    print(f"  in BOTH: {len(both)}   kinematic-only (lost at pass_1): {len(kin_only)}   "
          f"pass_1-only (newly flagged): {len(ekf_only)}")
    print(f"  kinematic flagged set is a subset of pass_1 flagged set: {kin_flagged <= ekf_flagged}")
    if kin_only:
        print(f"    kinematic-only instances: {sorted(kin_only)}")
    print()
