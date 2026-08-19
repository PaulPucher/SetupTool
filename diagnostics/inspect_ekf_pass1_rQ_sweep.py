# 2-D Q/R sweep for the pass-1 Dugoff EKF, following up on pass_1's
# opposite-direction NIS gate failure (ay over-corrected at 0.04%
# exceedance, gain collapsed ~180x; yaw rate under-corrected at 30.1%).
# A single scalar R_scale cannot fix this -- the two channels' relative
# weighting has to move, not just R's overall size. Read-only, no
# config/production change.
#
# Parameterisation: (R_ay_scale, R_yaw_rate_scale), each multiplying
# pass_1's own derived R_ay_var/R_yaw_rate_var independently -- pass_1
# is the anchor, not a restart from scratch. Q is FIXED at pass_1's
# value (unchanged from pass_0) for this sweep: nothing in the pass_1
# evaluation implicated Q specifically (the 93%->20% combined NIS
# improvement and the C2 fix both came from the R change alone), so
# adding a third free dimension here would widen the grid without a
# diagnosed reason to. This keeps the search a genuine, interpretable
# 2-D grid; Q's own scale stays an explicitly open follow-up dimension
# if this R-focused sweep doesn't resolve the gate.
#
# Only NIS exceedance per channel is computed per grid point (the cheap
# part of a full run) -- the full acceptance-criteria set is reported
# separately, once, at the recommended (or closest) point only.

import copy

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

cfg1 = params["tyre_model_ekf"]["pass_1"]
R_ay_base = cfg1["R_ay_var"]
R_yaw_base = cfg1["R_yaw_rate_var"]

R_AY_SCALES = [1.0, 0.3, 0.1, 0.03, 0.01]
R_YAW_SCALES = [1.0, 2.0, 4.0, 8.0, 16.0]

chi2_df1 = float(chi2.ppf(0.95, df=1))

print("=" * 100)
print("2-D sweep: R_ay_scale x R_yaw_rate_scale, anchored at pass_1 "
      f"(R_ay_var={R_ay_base:.4f}, R_yaw_rate_var={R_yaw_base:.6e}), Q fixed at pass_1")
print("=" * 100)
print(f"chi-square df=1 95% bound = {chi2_df1:.4f}   target band (gate, both channels): 3%-15%")
print()

results = []
for r_ay_scale in R_AY_SCALES:
    for r_yaw_scale in R_YAW_SCALES:
        params_sweep = copy.deepcopy(params)
        cfg_sweep = dict(cfg1)
        cfg_sweep["R_ay_var"] = R_ay_base * r_ay_scale
        cfg_sweep["R_yaw_rate_var"] = R_yaw_base * r_yaw_scale
        params_sweep["tyre_model_ekf"]["sweep_temp"] = cfg_sweep

        result = estimate_sideslip_ekf_dugoff(state, params_sweep, pass_id="sweep_temp")
        innovation = result["innovation"][base_mask]
        S_diag = result["S_diag"][base_mask]
        nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
        nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]
        f_yaw = float((nis_yaw > chi2_df1).mean())
        f_ay = float((nis_ay > chi2_df1).mean())

        both_in_band = (0.03 <= f_yaw <= 0.15) and (0.03 <= f_ay <= 0.15)
        results.append({
            "r_ay_scale": r_ay_scale, "r_yaw_scale": r_yaw_scale,
            "R_ay_var": cfg_sweep["R_ay_var"], "R_yaw_rate_var": cfg_sweep["R_yaw_rate_var"],
            "f_yaw": f_yaw, "f_ay": f_ay, "both_in_band": both_in_band,
        })
        tag = " <-- BOTH IN BAND" if both_in_band else ""
        print(f"  R_ay_scale={r_ay_scale:5.2f} (var={cfg_sweep['R_ay_var']:8.4f})  "
              f"R_yaw_scale={r_yaw_scale:5.2f} (var={cfg_sweep['R_yaw_rate_var']:.6e}):  "
              f"yaw_exceed={f_yaw:.4f}  ay_exceed={f_ay:.4f}{tag}")
print()

in_band = [r for r in results if r["both_in_band"]]
print(f"grid points with BOTH channels in [3%,15%]: {len(in_band)} / {len(results)}")
if in_band:
    for r in in_band:
        print(f"  R_ay_scale={r['r_ay_scale']}  R_yaw_scale={r['r_yaw_scale']}  "
              f"yaw_exceed={r['f_yaw']:.4f}  ay_exceed={r['f_ay']:.4f}")
else:
    print("NO grid point satisfies both bands simultaneously in the searched range.")
    # report the closest candidate by summed absolute distance from band centre (9%)
    def _dist(r):
        centre = 0.09
        return abs(r["f_yaw"] - centre) + abs(r["f_ay"] - centre)
    closest = min(results, key=_dist)
    print(f"closest candidate (by distance from 9% band centre, both channels): "
          f"R_ay_scale={closest['r_ay_scale']}  R_yaw_scale={closest['r_yaw_scale']}  "
          f"yaw_exceed={closest['f_yaw']:.4f}  ay_exceed={closest['f_ay']:.4f}")
