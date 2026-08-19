# h2-vs-ay correlation, dual population, for a direct apples-to-apples
# comparison against the n=471 apex-phase, kinematic-alpha reference
# (0.887) established two turns ago. Read-only, no config change.
# Reports, for a given pass_id's OWN converged alpha: (a) the full
# masked population (n=24183, already reported at 0.973 for pass_1's
# first-derivation R values); (b) the SAME n=471 apex-phase subset the
# 0.887 reference used, so population size is no longer a confound.

import sys

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff, slip_angles
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PASS_ID = sys.argv[1] if len(sys.argv) > 1 else "pass_1"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
v = state["v_mps"]
yaw_rate = state["yaw_rate_radps"]
delta_f = state["delta_f_rad"]
ay_meas = state["ay_mps2"]
moving = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving_clean = moving & ~kerb_mask if kerb_mask is not None else moving
v_min = params["stability_estimation"]["moving_speed_min_mps"]

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving_clean & racing_mask

apex_half_window = params["stability_estimation"]["apex_half_window_samples"]
apex_mask = np.zeros_like(t, dtype=bool)
for c in data.get("corners", []):
    start_t, end_t = c["segments"]["apex_3"]
    if end_t < start_t:
        continue
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        lo = max(0, lo - apex_half_window)
        hi = min(len(t), hi + apex_half_window + 1)
    apex_mask[lo:hi] = True

apex_pop_mask = base_mask & apex_mask

vp = params["vehicle"]
a = vp["cog_to_front_axle_m"]
b = vp["cog_to_rear_axle_m"]
m = vp["mass_kg"]
cfg = params["tyre_model_ekf"][PASS_ID]
c_alpha_f, c_alpha_r = cfg["c_alpha_front_n_per_rad"], cfg["c_alpha_rear_n_per_rad"]
mu_fz_f, mu_fz_r = cfg["mu_fz_front_N"], cfg["mu_fz_rear_N"]

result = estimate_sideslip_ekf_dugoff(state, params, pass_id=PASS_ID)
beta = result["beta"]


def _h2(idx):
    out = np.full(len(idx), np.nan)
    for k, i in enumerate(idx):
        Vx = max(float(v[i]), v_min)
        alpha_f, alpha_r = slip_angles(beta[i], yaw_rate[i], delta_f[i], Vx, a, b)
        Fy_f = dugoff_lateral_force(alpha_f, c_alpha_f, mu_fz_f)
        Fy_r = dugoff_lateral_force(alpha_r, c_alpha_r, mu_fz_r)
        out[k] = (Fy_f + Fy_r) / m
    return out


print("=" * 100)
print(f"h2-vs-ay correlation, {PASS_ID}'s own converged alpha, dual population")
print("=" * 100)

for label, mask in (("full masked population", base_mask), ("n=471-equivalent apex-phase subset", apex_pop_mask)):
    idx = np.where(mask)[0]
    h2 = _h2(idx)
    corr = float(np.corrcoef(h2, ay_meas[idx])[0, 1])
    print(f"  {label}: n={len(idx)}  corr(h2_pred, ay_meas)={corr:.4f}")

print()
print("reference: kinematic-alpha, n=471 apex-phase population (2 turns ago): corr=0.887")
