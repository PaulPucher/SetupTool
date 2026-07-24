# Vehicle-model-upgrade WP verification: Fy yaw-moment term (Module 4a).
# Steady-state reproduction check, per-phase CS_ratio median shift (old
# static split vs new 2-DOF split), and an RMS plausibility figure for the
# added yaw term. Report-only, no product-path changes.

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces_new = estimate_lateral_forces(state, params)
cs_new = estimate_cornering_stiffness(slip, forces_new, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
summaries_new = summarise_corners(data["corners"], cs_new, stab, state)

# --- reconstruct the OLD static-split-only Fy (one-line formula, no need
# for a git-history fetch -- unlike Module 5, this one is trivial to redo) ---
vp = params["vehicle"]
m = vp["mass_kg"]
cw = vp["corner_weights"]
W_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / W_total
rear_fraction = (cw["RL_kg"] + cw["RR_kg"]) / W_total
moving = state["moving_mask"]
Fy_total = m * state["ay_mps2"]
Fy_f_old = np.where(moving, Fy_total * front_fraction, 0.0)
Fy_r_old = np.where(moving, Fy_total * rear_fraction, 0.0)

sr = state["sample_rate_hz"]
se = params["stability_estimation"]
cutoff = se["cs_filter_cutoff_hz"]
from modules.stability_analysis import _butterworth_lowpass
Fy_f_old_filt = _butterworth_lowpass(Fy_f_old, cutoff, sr)
Fy_r_old_filt = _butterworth_lowpass(Fy_r_old, cutoff, sr)
forces_old = {"Fy_f_filt": Fy_f_old_filt, "Fy_r_filt": Fy_r_old_filt,
              "Fy_f_raw": Fy_f_old, "Fy_r_raw": Fy_r_old,
              "front_fraction": front_fraction, "rear_fraction": rear_fraction,
              "accuracy_level": 1}
cs_old = estimate_cornering_stiffness(slip, forces_old, state, params)
summaries_old = summarise_corners(data["corners"], cs_old, stab, state)

print("=== Steady-state reproduction check ===")
Iz = vp["yaw_inertia_kgm2"]
wheelbase = vp["wheelbase_m"]
psidd_raw = np.gradient(state["yaw_rate_radps"], state["time"])
yaw_term = Iz * psidd_raw / wheelbase
small_psidd = np.abs(psidd_raw) < np.percentile(np.abs(psidd_raw[moving]), 10)
mask = moving & small_psidd
new_f = forces_new["Fy_f_raw"][mask]
old_f = Fy_f_old[mask]
rel_diff = np.abs(new_f - old_f) / np.maximum(np.abs(old_f), 1.0)
print(f"  Samples in smallest-10%-|psidd| bucket: {mask.sum()}")
print(f"  Median relative |Fy_f_new - Fy_f_old| / |Fy_f_old|: {np.median(rel_diff)*100:.2f}%")
print(f"  Max relative diff in this bucket: {np.max(rel_diff)*100:.2f}%")

print("\n=== RMS(yaw term) / RMS(Fy_total) plausibility figure ===")
rms_yaw_term = np.sqrt(np.mean(yaw_term[moving] ** 2))
rms_fy_total = np.sqrt(np.mean(Fy_total[moving] ** 2))
print(f"  RMS(Iz*psidd/wheelbase) = {rms_yaw_term:.1f} N")
print(f"  RMS(m*ay)               = {rms_fy_total:.1f} N")
print(f"  Ratio: {rms_yaw_term/rms_fy_total*100:.1f}%")

print("\n=== Per-phase CS_ratio median shift (median across 51 corner instances' phase medians) ===")
phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
for phase in phase_keys:
    old_vals_f = [s["phases"][phase]["cs_ratio_f"]["median"] for s in summaries_old]
    new_vals_f = [s["phases"][phase]["cs_ratio_f"]["median"] for s in summaries_new]
    old_vals_r = [s["phases"][phase]["cs_ratio_r"]["median"] for s in summaries_old]
    new_vals_r = [s["phases"][phase]["cs_ratio_r"]["median"] for s in summaries_new]
    old_f = np.nanmedian([v for v in old_vals_f if v == v])
    new_f = np.nanmedian([v for v in new_vals_f if v == v])
    old_r = np.nanmedian([v for v in old_vals_r if v == v])
    new_r = np.nanmedian([v for v in new_vals_r if v == v])
    print(f"  {phase:>16}  CSf: {old_f:.3f} -> {new_f:.3f} (d={new_f-old_f:+.3f})   "
          f"CSr: {old_r:.3f} -> {new_r:.3f} (d={new_r-old_r:+.3f})")

print("\n=== Sample-level Fy_f/Fy_r range check ===")
print(f"  Fy_f_raw new: min={forces_new['Fy_f_raw'][moving].min():.0f}  max={forces_new['Fy_f_raw'][moving].max():.0f}")
print(f"  Fy_r_raw new: min={forces_new['Fy_r_raw'][moving].min():.0f}  max={forces_new['Fy_r_raw'][moving].max():.0f}")
print(f"  Fy_f_raw old: min={Fy_f_old[moving].min():.0f}  max={Fy_f_old[moving].max():.0f}")
print(f"  Fy_r_raw old: min={Fy_r_old[moving].min():.0f}  max={Fy_r_old[moving].max():.0f}")
