# Evaluate the pass-1 Dugoff EKF (noise-model-only recalibration, see
# config/parameters.json tyre_model_ekf.pass_1) against the acceptance
# criteria pre-registered before any pass-1 numbers existed. Read-only,
# no config/production change. Sections:
#   1. NIS exceedance fraction per channel, pass_0 vs pass_1.
#   2. C2 excursion recheck (t=883.0-885.5s): does the single-sample
#      sign discontinuity survive recalibration.
#   3. Sign check: median-per-corner (the pass/fail gate) AND the more
#      sensitive per-sample-within-corner fraction (reported, not
#      gated), both for pass_0 (baseline) and pass_1.
#   4. Over-inflation checks: Kalman gain's ay column (K_ay) summary,
#      and h2-vs-ay correlation using pass_1's own converged alpha.

import json

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff, slip_angles
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
v_kmh = state["v_mps"] * 3.6
ay = state["ay_mps2"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

result_0 = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_0")
result_1 = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")

# --- Section 1: NIS exceedance per channel, pass_0 vs pass_1 ---------------

chi2_df1 = float(chi2.ppf(0.95, df=1))
chi2_df2 = float(chi2.ppf(0.95, df=2))

print("=" * 100)
print("SECTION 1 -- NIS exceedance fraction per channel, pass_0 vs pass_1")
print("=" * 100)
print(f"chi-square 95% bounds: df=1 (per-channel)={chi2_df1:.4f}  df=2 (combined)={chi2_df2:.4f}")
print("acceptance band (both ends gate): 3%-15%")
print()

for label, result in (("pass_0", result_0), ("pass_1", result_1)):
    innovation = result["innovation"][base_mask]
    nis_combined = result["nis"][base_mask]
    nis_yaw = innovation[:, 0] ** 2 / result["S_diag"][base_mask][:, 0]
    nis_ay = innovation[:, 1] ** 2 / result["S_diag"][base_mask][:, 1]
    f_yaw = float((nis_yaw > chi2_df1).mean())
    f_ay = float((nis_ay > chi2_df1).mean())
    f_comb = float((nis_combined > chi2_df2).mean())
    g_yaw = "PASS" if 0.03 <= f_yaw <= 0.15 else "FAIL"
    g_ay = "PASS" if 0.03 <= f_ay <= 0.15 else "FAIL"
    print(f"  {label}: yaw_rate exceedance={f_yaw:.4f} [{g_yaw}]   ay exceedance={f_ay:.4f} [{g_ay}]   "
          f"combined exceedance={f_comb:.4f}   combined mean NIS={np.mean(nis_combined):.3f} (expect ~2 if calibrated)")
print()

# --- Section 2: C2 excursion recheck ----------------------------------------

print("=" * 100)
print("SECTION 2 -- C2 excursion recheck, t=883.0-885.5s")
print("=" * 100)

idx = np.where((t >= 883.0) & (t <= 885.5))[0]
for label, result in (("pass_0", result_0), ("pass_1", result_1)):
    beta_deg = np.degrees(result["beta"][idx])
    max_abs = float(np.max(np.abs(beta_deg)))
    step_diff = np.diff(beta_deg)
    max_step = float(np.max(np.abs(step_diff)))
    i_worst_step = int(np.argmax(np.abs(step_diff)))
    print(f"  {label}: max|beta| in window={max_abs:.3f} deg   "
          f"max single-sample step={max_step:.3f} deg (at t={t[idx[i_worst_step]]:.3f}s: "
          f"{beta_deg[i_worst_step]:+.3f} -> {beta_deg[i_worst_step+1]:+.3f})")
print()

# --- Section 3: sign check -- median (gate) + per-sample fraction (reported)

print("=" * 100)
print("SECTION 3 -- sign check: median-per-corner (gate) + per-sample fraction (reported)")
print("=" * 100)

with open("config/channels.json", "r", encoding="utf-8") as f:
    channels_json = json.load(f)
LOW_SPEED_MAX_KMH = channels_json["corner_speed_thresholds"]["low_max"]

corners = data.get("corners", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
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


s_m = state.get("s_m")

for label, result in (("pass_0", result_0), ("pass_1", result_1)):
    beta_deg_full = np.degrees(result["beta"])
    n_match_median = n_total = 0
    n_match_median_racing = n_racing = 0
    per_sample_frac_pooled_num = 0
    per_sample_frac_pooled_den = 0
    print(f"--- {label} ---")
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        speed_class = instances[0].get("speed_class")
        if bracket_start is None or bracket_end is None:
            continue
        pooled_ay, pooled_v, pooled_beta = [], [], []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = moving[sl]
            if not m.any():
                continue
            pooled_ay.append(ay[sl][m])
            pooled_v.append(v_kmh[sl][m])
            pooled_beta.append(beta_deg_full[sl][m])
        if not pooled_ay:
            continue
        ay_cat = np.concatenate(pooled_ay)
        v_cat = np.concatenate(pooled_v)
        beta_cat = np.concatenate(pooled_beta)
        med_ay = float(np.median(ay_cat))
        med_v = float(np.median(v_cat))
        med_beta = float(np.median(beta_cat))
        dir_sign = np.sign(med_ay)
        low_speed = med_v < LOW_SPEED_MAX_KMH
        median_match = (np.sign(med_beta) == -dir_sign) if dir_sign != 0 else None

        per_sample_match = (np.sign(beta_cat) == -dir_sign) if dir_sign != 0 else np.zeros_like(beta_cat, dtype=bool)
        per_sample_frac = float(np.mean(per_sample_match)) if dir_sign != 0 else float("nan")

        n_total += 1
        n_match_median += int(bool(median_match))
        if not low_speed:
            n_racing += 1
            n_match_median_racing += int(bool(median_match))
            per_sample_frac_pooled_num += int(np.sum(per_sample_match))
            per_sample_frac_pooled_den += len(per_sample_match)

        print(f"  C{cid}: median_match={median_match}  low_speed={low_speed}  "
              f"per_sample_fraction_matching={per_sample_frac:.4f}  n={len(beta_cat)}")

    pooled_frac = per_sample_frac_pooled_num / per_sample_frac_pooled_den if per_sample_frac_pooled_den else float("nan")
    print(f"  MEDIAN GATE: {n_match_median}/{n_total} all corners, {n_match_median_racing}/{n_racing} racing-speed")
    print(f"  PER-SAMPLE (racing-speed pooled, reported not gated): {pooled_frac:.4f} "
          f"({per_sample_frac_pooled_num}/{per_sample_frac_pooled_den})")
    print()

# --- Section 4: over-inflation checks ---------------------------------------

print("=" * 100)
print("SECTION 4 -- over-inflation checks: K_ay column, h2-vs-ay correlation (pass_1's own alpha)")
print("=" * 100)

for label, result in (("pass_0", result_0), ("pass_1", result_1)):
    K_ay = result["K_ay"][base_mask]
    finite = np.isfinite(K_ay[:, 0]) & np.isfinite(K_ay[:, 1])
    K_ay = K_ay[finite]
    print(f"  {label}: K_ay column (gain applied to ay innovation) -- "
          f"K_beta_ay: median={np.median(K_ay[:,0]):.3e}  p90={np.percentile(np.abs(K_ay[:,0]),90):.3e}   "
          f"K_r_ay: median={np.median(K_ay[:,1]):.3e}  p90={np.percentile(np.abs(K_ay[:,1]),90):.3e}")
print()

vp = params["vehicle"]
a = vp["cog_to_front_axle_m"]
b = vp["cog_to_rear_axle_m"]
v_min = params["stability_estimation"]["moving_speed_min_mps"]
cfg1 = params["tyre_model_ekf"]["pass_1"]
c_alpha_f, c_alpha_r = cfg1["c_alpha_front_n_per_rad"], cfg1["c_alpha_rear_n_per_rad"]
mu_fz_f, mu_fz_r = cfg1["mu_fz_front_N"], cfg1["mu_fz_rear_N"]

beta_1 = result_1["beta"]
yaw_rate = state["yaw_rate_radps"]
delta_f = state["delta_f_rad"]
v = state["v_mps"]
m = vp["mass_kg"]

idx_pop = np.where(base_mask)[0]
h2_pred = np.full(len(idx_pop), np.nan)
for k, i in enumerate(idx_pop):
    Vx = max(float(v[i]), v_min)
    alpha_f, alpha_r = slip_angles(beta_1[i], yaw_rate[i], delta_f[i], Vx, a, b)
    Fy_f = dugoff_lateral_force(alpha_f, c_alpha_f, mu_fz_f)
    Fy_r = dugoff_lateral_force(alpha_r, c_alpha_r, mu_fz_r)
    h2_pred[k] = (Fy_f + Fy_r) / m

corr_pass1 = float(np.corrcoef(h2_pred, ay[idx_pop])[0, 1])
print(f"h2-vs-ay correlation, pass_1's OWN converged alpha, full masked population (n={len(idx_pop)}): "
      f"{corr_pass1:.4f}   (cf. kinematic-alpha reference from the prior turn: 0.887, n=471 apex-only population)")
