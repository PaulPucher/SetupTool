# WP-N2 pass 2 evaluation: NIS exceedance, sign check and the h2-vs-ay
# apex-population check, run against the pass_2 EKF configuration (the
# refit curve from diagnostics/fit_dugoff_pass2_refit.py, Q/R/P0 held at
# pass_1). Mirrors inspect_ekf_pass1_evaluation.py's Sections 1/3 and the
# h2 check's methodology (diagnostics/inspect_ekf_dugoff_sanity_checks.py
# check_h2_vs_ay_consistency), single-pass rather than a two-pass compare
# since there is only one pass_2. Read-only, no config/production change.
#
# Circularity slope ratios, CS_ratio distribution/flagged counts, the
# self-consistency R^2 (frozen curve at its OWN pass's alpha) and onset
# coverage are NOT duplicated here -- diagnostics/inspect_ekf_dugoff_
# circularity.py already computes exactly these, parameterised by
# PASS_ID via sys.argv; run it directly with "pass_2" for that half of
# the report.

import json

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_slip_angles
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PASS_ID = "pass_2"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
ay = state["ay_mps2"]
v = state["v_mps"]
v_min = params["stability_estimation"]["moving_speed_min_mps"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

result_2 = estimate_sideslip_ekf_dugoff(state, params, pass_id=PASS_ID)
beta_2 = result_2["beta"]
slip_2 = estimate_slip_angles(state, beta_2, params)

cfg2 = params["tyre_model_ekf"][PASS_ID]
c_alpha_f, c_alpha_r = cfg2["c_alpha_front_n_per_rad"], cfg2["c_alpha_rear_n_per_rad"]
mu_fz_f, mu_fz_r = cfg2["mu_fz_front_N"], cfg2["mu_fz_rear_N"]

# Reference: pass_0/pass_1's frozen curve, for relative-change reporting.
cfg_prior = params["tyre_model_ekf"]["pass_1"]
prior = {
    "c_alpha_front_n_per_rad": cfg_prior["c_alpha_front_n_per_rad"],
    "c_alpha_rear_n_per_rad": cfg_prior["c_alpha_rear_n_per_rad"],
    "mu_fz_front_N": cfg_prior["mu_fz_front_N"],
    "mu_fz_rear_N": cfg_prior["mu_fz_rear_N"],
}
current = {
    "c_alpha_front_n_per_rad": c_alpha_f,
    "c_alpha_rear_n_per_rad": c_alpha_r,
    "mu_fz_front_N": mu_fz_f,
    "mu_fz_rear_N": mu_fz_r,
}

print("=" * 100)
print("SECTION 0 -- refitted parameters vs pass_0/pass_1 frozen prior, relative change")
print("=" * 100)
for key in current:
    rel = (current[key] - prior[key]) / prior[key]
    print(f"  {key}: pass_0/1={prior[key]:12.2f}   pass_2={current[key]:12.2f}   "
          f"relative change={rel*100:+.2f}%")
print()

# --- Section 1: NIS exceedance ----------------------------------------------

chi2_df1 = float(chi2.ppf(0.95, df=1))
chi2_df2 = float(chi2.ppf(0.95, df=2))

print("=" * 100)
print(f"SECTION 1 -- NIS exceedance, {PASS_ID} (R KNOWINGLY MIS-SPECIFIED for this curve -- "
      f"held at pass_1's value, see config note)")
print("=" * 100)
print(f"chi-square 95% bounds: df=1={chi2_df1:.4f}  df=2={chi2_df2:.4f}")
print()

innovation = result_2["innovation"][base_mask]
nis_combined = result_2["nis"][base_mask]
nis_yaw = innovation[:, 0] ** 2 / result_2["S_diag"][base_mask][:, 0]
nis_ay = innovation[:, 1] ** 2 / result_2["S_diag"][base_mask][:, 1]
f_yaw = float((nis_yaw > chi2_df1).mean())
f_ay = float((nis_ay > chi2_df1).mean())
f_comb = float((nis_combined > chi2_df2).mean())
print(f"  yaw_rate exceedance={f_yaw:.4f}   ay exceedance={f_ay:.4f}   "
      f"combined exceedance={f_comb:.4f}   combined mean NIS={np.mean(nis_combined):.3f} (expect ~2 if calibrated)")
print()

# --- Section 2: sign check ---------------------------------------------------

print("=" * 100)
print("SECTION 2 -- sign check: median-per-corner (gate) + per-sample fraction (reported)")
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
v_kmh = v * 3.6
beta_deg_full = np.degrees(beta_2)

n_match_median = n_total = 0
n_match_median_racing = n_racing = 0
per_sample_frac_pooled_num = 0
per_sample_frac_pooled_den = 0
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
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

    n_total += 1
    n_match_median += int(bool(median_match))
    if not low_speed:
        n_racing += 1
        n_match_median_racing += int(bool(median_match))
        per_sample_frac_pooled_num += int(np.sum(per_sample_match))
        per_sample_frac_pooled_den += len(per_sample_match)

pooled_frac = per_sample_frac_pooled_num / per_sample_frac_pooled_den if per_sample_frac_pooled_den else float("nan")
print(f"  MEDIAN GATE: {n_match_median}/{n_total} all corners, {n_match_median_racing}/{n_racing} racing-speed")
print(f"  PER-SAMPLE (racing-speed pooled, reported not gated): {pooled_frac:.4f} "
      f"({per_sample_frac_pooled_num}/{per_sample_frac_pooled_den})")
print()

# --- Section 3: h2-vs-ay, apex_3 population, pass_2's OWN alpha/curve ------

print("=" * 100)
print("SECTION 3 -- h2-vs-ay, apex_3 population (n~471), pass_2's OWN alpha and OWN curve")
print("=" * 100)
print("NOTE: the pre-registration's reference figure (0.9682) was the CORRELATION r at this")
print("population (thesis_notes.md WP-N2 pass 1 entry), not a regression slope -- both are")
print("reported below to avoid repeating that imprecision.")
print()

apex_mask = np.zeros_like(t, dtype=bool)
for c in corners:
    start_t, end_t = c["segments"]["apex_3"]
    if end_t < start_t:
        continue
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        apex_half = params["stability_estimation"]["apex_half_window_samples"]
        centre = lo
        lo = max(0, centre - apex_half)
        hi = min(len(t), centre + apex_half + 1)
    apex_mask[lo:hi] = True

apex_pop_mask = base_mask & apex_mask
idx = np.where(apex_pop_mask)[0]

alpha_f_2 = slip_2["alpha_f_filt"]
alpha_r_2 = slip_2["alpha_r_filt"]
m_kg = params["vehicle"]["mass_kg"]

h2_pred_apex = np.full(len(idx), np.nan)
for k, i in enumerate(idx):
    Fy_f = dugoff_lateral_force(alpha_f_2[i], c_alpha_f, mu_fz_f)
    Fy_r = dugoff_lateral_force(alpha_r_2[i], c_alpha_r, mu_fz_r)
    h2_pred_apex[k] = (Fy_f + Fy_r) / m_kg

ay_apex = ay[idx]
r_apex = float(np.corrcoef(h2_pred_apex, ay_apex)[0, 1])
slope_apex, intercept_apex = np.polyfit(ay_apex, h2_pred_apex, 1)
print(f"  n={len(idx)}  corr(h2_pred, ay_meas)={r_apex:+.4f}  "
      f"regression slope (h2_pred vs ay_meas)={float(slope_apex):.4f}  intercept={float(intercept_apex):+.4f}")

# Full masked population, for context against pass_1's 0.9808 figure.
idx_full = np.where(base_mask)[0]
h2_pred_full = np.full(len(idx_full), np.nan)
for k, i in enumerate(idx_full):
    Fy_f = dugoff_lateral_force(alpha_f_2[i], c_alpha_f, mu_fz_f)
    Fy_r = dugoff_lateral_force(alpha_r_2[i], c_alpha_r, mu_fz_r)
    h2_pred_full[k] = (Fy_f + Fy_r) / m_kg
r_full = float(np.corrcoef(h2_pred_full, ay[idx_full])[0, 1])
print(f"  full masked population (n={len(idx_full)}): corr(h2_pred, ay_meas)={r_full:+.4f}  "
      f"(cf. pass_1's 0.9808 at this population)")
print()
