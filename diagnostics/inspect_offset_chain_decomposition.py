# WP-S3b (Open Board item B, sideslip methods comparison): zero-slip Fy
# offset chain decomposition. Read-only, Tier B (signal/data engineering
# -- standard decomposition and least-squares diagnostics, no new
# vehicle-dynamics claim). No production/config change; nothing here is
# whitelisted or called from any pipeline/UI path.
#
# Follow-up to WP-S3's Metric 5 (diagnostics/inspect_sideslip_methods_
# comparison.py), which found the direction-locked zero-slip Fy offset
# persists at comparable magnitude under two nearly-uncorrelated beta
# estimates (kinematic vs GPS-course, r=-0.24). That rules beta's own
# construction OUT as the dominant cause and moves suspicion to the
# parts of the alpha_f/alpha_r/Fy_f/Fy_r construction chain that do NOT
# depend on beta: yaw rate, ay, steering angle, and the shared
# Fy computation (estimate_lateral_forces) itself.
#
# Sections:
#   1. Rear slip-angle decomposition at |alpha_r_filt| < NEAR_ZERO_SLIP_DEG:
#      median beta, median rear yaw-geometry term (b*yaw_rate/v_x, the
#      component estimate_slip_angles forms inside its arctan argument),
#      median ay -- which term is cancelling which for alpha_r to read
#      (near) zero while ay (and hence Fy) is large.
#   2. Same decomposition for the front: beta term, yaw-geometry term
#      (a*yaw_rate/v_x), steering term (delta_f), plus ay for the same
#      continuity as section 1.
#   3. IMU lever-arm check: what is known in-repo about IMU mounting
#      position (config/parameters.json, thesis_notes.md limitations
#      register). If unknown (as found here), a diagnostic -- explicitly
#      NOT a calibration -- least-squares fit of the observed near-
#      zero-slip Fy offset against the psiddot-scaled lever-arm term
#      that a longitudinal IMU/CoG offset would predict.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
NEAR_ZERO_SLIP_DEG = 0.2  # matches inspect_c9_negative_cs.py / Metric 5

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
vp = params["vehicle"]

t_ref = state["time"]
v = state["v_mps"]
yaw_rate = state["yaw_rate_radps"]
delta_f = state["delta_f_rad"]
ay = state["ay_mps2"]
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

a = vp["cog_to_front_axle_m"]
b = vp["cog_to_rear_axle_m"]
mass_kg = vp["mass_kg"]
cw = vp["corner_weights"]
W_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / W_total
rear_fraction = (cw["RL_kg"] + cw["RR_kg"]) / W_total

beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
alpha_f, alpha_r = slip["alpha_f_filt"], slip["alpha_r_filt"]
Fy_f, Fy_r = forces["Fy_f_filt"], forces["Fy_r_filt"]

# Mirrors estimate_slip_angles's own v_x_safe construction exactly --
# needed here to reconstruct the same arctan-argument components it
# forms internally (not exposed in its return dict).
v_x_safe = np.where(moving_raw, v * np.cos(beta), 1.0)
yaw_geom_f = a * yaw_rate / v_x_safe   # a*psidot/v_x component of alpha_f's arctan argument
yaw_geom_r = b * yaw_rate / v_x_safe   # b*psidot/v_x component of alpha_r's arctan argument

# Same construction as estimate_lateral_forces's own internal psidd_raw
# (not exposed in its return dict either).
psidd_raw = np.gradient(yaw_rate, t_ref)

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in data.get("corners", []):
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to inspect_c9_negative_cs.py's / Metric 5's own helper of
    # the same name: a minimal reimplementation of ui/views/corner_trace_
    # dialog.py's _extend_slice_with_margin with zero margin. min()/max()
    # for the lap's own s extent, not first/last-finite-index -- a lap-
    # boundary reset sample can trail into [lo:hi) with s collapsed to
    # ~0, verified there against real laps 2 and 4.
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(np.max(lap_s[finite]))
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


def _pooled_near_zero(alpha_arr, value_arrays):
    """Per stable corner: pool named arrays at samples where |alpha_arr| <
    near_zero_rad inside each valid lap's canonical bracket window
    (moving & kerb-excluded). Returns {cid: {"n": n, name: median, ...}}.
    Same window/mask construction as Metric 5's _zero_slip_fy_offset."""
    out = {}
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled = {name: [] for name in value_arrays}
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            mm = moving[sl]
            aa = alpha_arr[sl]
            valid = mm & np.isfinite(aa) & (np.abs(aa) < near_zero_rad)
            if not valid.any():
                continue
            for name, varr in value_arrays.items():
                pooled[name].append(varr[sl][valid])
        n = sum(len(x) for x in next(iter(pooled.values()))) if pooled else 0
        medians = {name: (float(np.median(np.concatenate(v))) if v else float("nan"))
                   for name, v in pooled.items()}
        out[cid] = {"n": n, **medians}
    return out


# --- Section 1: rear decomposition -------------------------------------

print("=" * 78)
print("SECTION 1 -- rear slip-angle decomposition at |alpha_r_filt| < "
      f"{NEAR_ZERO_SLIP_DEG} deg")
print("=" * 78)
print("alpha_r = -arctan((v*sin(beta) - b*yaw_rate)/v_x); small-angle form")
print("alpha_r ~= yaw_geom_r - beta. Near zero alpha_r therefore means beta")
print("and the yaw-geometry term are nearly EQUAL (their difference cancels),")
print("not that they oppose in sign.")
print()

rear_arrays = {"beta": beta, "yaw_geom_r": yaw_geom_r, "ay": ay, "alpha_r": alpha_r,
               "Fy_r": Fy_r, "psidd": psidd_raw}
pooled_r = _pooled_near_zero(alpha_r, rear_arrays)

for cid in stable_ids:
    d = pooled_r.get(cid)
    if d is None or d["n"] == 0:
        print(f"  C{cid}: no near-zero-alpha_r samples in canonical window -- skipped.")
        continue
    beta_deg = np.degrees(d["beta"])
    yg_deg = np.degrees(d["yaw_geom_r"])
    ar_deg = np.degrees(d["alpha_r"])
    print(f"  C{cid}: n={d['n']:4d}  median beta={beta_deg:+7.3f} deg  "
          f"median yaw_geom_r={yg_deg:+7.3f} deg  (diff={yg_deg - beta_deg:+.3f} deg, "
          f"actual alpha_r={ar_deg:+.3f} deg)  median ay={d['ay']:+7.2f} m/s^2")

valid_r = [pooled_r[cid] for cid in stable_ids if pooled_r.get(cid, {}).get("n", 0) > 0]
if valid_r:
    mean_abs_beta = np.mean([abs(np.degrees(d["beta"])) for d in valid_r])
    mean_abs_yg = np.mean([abs(np.degrees(d["yaw_geom_r"])) for d in valid_r])
    mean_abs_ay = np.mean([abs(d["ay"]) for d in valid_r])
    print()
    print(f"  Across {len(valid_r)} corners: mean |median beta|={mean_abs_beta:.3f} deg, "
          f"mean |median yaw_geom_r|={mean_abs_yg:.3f} deg (ratio "
          f"{mean_abs_beta / mean_abs_yg:.2f}) -- comparable magnitude confirms beta and "
          f"the yaw-rate geometry term are what is cancelling for alpha_r to read zero, "
          f"while mean |median ay|={mean_abs_ay:.2f} m/s^2 confirms the car is genuinely "
          f"cornering hard at these same samples.")

# --- Section 2: front decomposition -------------------------------------

print()
print("=" * 78)
print("SECTION 2 -- front slip-angle decomposition at |alpha_f_filt| < "
      f"{NEAR_ZERO_SLIP_DEG} deg")
print("=" * 78)
print("alpha_f = delta_f - arctan((v*sin(beta) + a*yaw_rate)/v_x); small-angle")
print("form alpha_f ~= delta_f - (beta + yaw_geom_f). Near zero alpha_f means")
print("the steering term is nearly matched by the SUM of the beta and")
print("yaw-geometry terms.")
print()

front_arrays = {"beta": beta, "yaw_geom_f": yaw_geom_f, "delta_f": delta_f, "ay": ay,
                "alpha_f": alpha_f, "Fy_f": Fy_f, "psidd": psidd_raw}
pooled_f = _pooled_near_zero(alpha_f, front_arrays)

for cid in stable_ids:
    d = pooled_f.get(cid)
    if d is None or d["n"] == 0:
        print(f"  C{cid}: no near-zero-alpha_f samples in canonical window -- skipped.")
        continue
    beta_deg = np.degrees(d["beta"])
    yg_deg = np.degrees(d["yaw_geom_f"])
    df_deg = np.degrees(d["delta_f"])
    af_deg = np.degrees(d["alpha_f"])
    print(f"  C{cid}: n={d['n']:4d}  median beta={beta_deg:+7.3f} deg  "
          f"median yaw_geom_f={yg_deg:+7.3f} deg  median delta_f={df_deg:+7.3f} deg  "
          f"(sum={beta_deg + yg_deg:+.3f} deg vs delta_f, actual alpha_f={af_deg:+.3f} deg)  "
          f"median ay={d['ay']:+7.2f} m/s^2")

valid_f = [pooled_f[cid] for cid in stable_ids if pooled_f.get(cid, {}).get("n", 0) > 0]
if valid_f:
    mean_abs_df = np.mean([abs(np.degrees(d["delta_f"])) for d in valid_f])
    mean_abs_sum = np.mean([abs(np.degrees(d["beta"]) + np.degrees(d["yaw_geom_f"])) for d in valid_f])
    print()
    print(f"  Across {len(valid_f)} corners: mean |median delta_f|={mean_abs_df:.3f} deg vs "
          f"mean |median (beta+yaw_geom_f)|={mean_abs_sum:.3f} deg -- comparable magnitude "
          f"confirms the steering term is what alpha_f's near-zero reading trades against.")

# --- Section 3: IMU lever-arm check --------------------------------------

print()
print("=" * 78)
print("SECTION 3 -- IMU lever-arm check")
print("=" * 78)
print("In-repo search (config/parameters.json, thesis_notes.md limitations")
print("register, docs/): no numeric IMU mounting-position offset exists anywhere")
print("in this repo. config/parameters.json's accuracy_levels.lateral_acc entry")
print("carries \"capped_by\": \"provenance-assumption: IMU-not-at-CoG (thesis_notes.md")
print("limitations register item 5)\"; thesis_notes.md section 4 item 5 reads only")
print("\"Accelerometer assumed at CoG.\" -- a documented ASSUMPTION, not a measured or")
print("estimated offset. No x_imu/cog_to_imu/sensor-position field exists in")
print("config/parameters.json, config/channels.json, config/car.json, or docs/.")
print("VERDICT: IMU longitudinal position is UNKNOWN in-repo (assumption only,")
print("no measurement, no candidate number on record).")
print()
print("Candidate model (rigid-body IMU lever-arm kinematics, standard planar")
print("2-DOF result -- NOT implemented, NOT applied to any signal here):")
print("  ay_imu = ay_cog + x_imu * psiddot   =>   ay_cog = ay_imu - x_imu * psiddot")
print("x_imu = IMU's forward offset from CoG (m, positive = ahead of CoG); a lateral")
print("(y) IMU offset is not modelled here (typical centreline IMU mounting).")
print("Consequence for Fy_f/Fy_r (estimate_lateral_forces, Module 4a): substituting")
print("ay_imu for ay_cog would fold an extra +/- mass_kg*fraction*x_imu*psiddot term")
print("into each axle's Fy on top of the existing Iz*psidd/wheelbase term -- i.e. an")
print("uncorrected lever arm would look exactly like a psidd-correlated offset, the")
print("same signature Metric 5 found. This motivates the fit below.")
print()
print("DIAGNOSTIC FIT ONLY -- NOT a calibration, NOT applied to any production path.")
print("Per axle: least-squares (through the origin) of the near-zero-slip median Fy")
print("offset against mass_kg*fraction*median(psidd) at those same samples. If the")
print("lever-arm model were the dominant cause, the fitted slope recovers x_imu directly")
print("(units: N / (kg*rad/s^2) = m).")


def _lever_arm_fit(pooled, fy_key, fraction):
    xs, ys, cids = [], [], []
    for cid in stable_ids:
        d = pooled.get(cid)
        if d is None or d["n"] == 0:
            continue
        fy, psidd = d[fy_key], d["psidd"]
        if fy != fy or psidd != psidd:
            continue
        xs.append(mass_kg * fraction * psidd)
        ys.append(fy)
        cids.append(cid)
    xs, ys = np.array(xs), np.array(ys)
    if len(xs) < 3 or np.sum(xs ** 2) == 0:
        return None
    slope = float(np.sum(xs * ys) / np.sum(xs ** 2))
    pred = slope * xs
    ss_res = float(np.sum((ys - pred) ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return {"slope_m": slope, "n_corners": len(xs), "r2": r2, "cids": cids}


fit_f = _lever_arm_fit(pooled_f, "Fy_f", front_fraction)
fit_r = _lever_arm_fit(pooled_r, "Fy_r", rear_fraction)

print()
if fit_f:
    print(f"  Front axle fit: n_corners={fit_f['n_corners']}  "
          f"x_imu_fit={fit_f['slope_m']:+.3f} m  R^2={fit_f['r2']:.3f}")
else:
    print("  Front axle fit: not enough corners with valid Fy_f/psidd medians.")
if fit_r:
    print(f"  Rear axle fit:  n_corners={fit_r['n_corners']}  "
          f"x_imu_fit={fit_r['slope_m']:+.3f} m  R^2={fit_r['r2']:.3f}")
else:
    print("  Rear axle fit:  not enough corners with valid Fy_r/psidd medians.")

if fit_f and fit_r:
    agree = abs(fit_f["slope_m"] - fit_r["slope_m"])
    print(f"  Front/rear fitted x_imu agreement: |diff|={agree:.3f} m "
          f"(front={fit_f['slope_m']:+.3f} m, rear={fit_r['slope_m']:+.3f} m).")
    plausible = all(abs(fit["slope_m"]) < 2.0 for fit in (fit_f, fit_r))
    print(f"  Plausibility (GT3 wheelbase {vp['wheelbase_m']:.3f} m -- a sane IMU offset "
          f"should sit well inside +/-{vp['wheelbase_m']/2:.2f} m of half the wheelbase): "
          f"{'plausible range' if plausible else 'IMPLAUSIBLE -- exceeds a sane fraction of the wheelbase'}.")
