# WP-N2 pass 1: final validation baseline for the carried-forward
# estimator (thesis_notes.md "WP-N2 carry-forward decision: pass 1").
# Consolidates checks already established at various points this
# session into ONE run, ONE timestamp, ONE manifest -- so combined-slip
# work has a single citable reference point instead of reassembling it
# from several dated entries. Introduces no new methodology; every
# section names the earlier script/entry its logic is drawn from.
# Read-only, no config/production change.

import json
import subprocess
from datetime import datetime, timezone

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PASS_ID = "pass_1"
MANIFEST_PATH = "diagnostics/pass1_final_validation_manifest.json"
NEAR_ZERO_SLIP_DEG = 0.2  # matches WP-S4b's own near-zero-alpha_r(A) comparison population

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
ay = state["ay_mps2"]
v = state["v_mps"]
v_kmh = v * 3.6
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

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


beta_a = estimate_sideslip(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
alpha_r_a = slip_a["alpha_r_filt"]

result_1 = estimate_sideslip_ekf_dugoff(state, params, pass_id=PASS_ID)
beta_1 = result_1["beta"]
slip_1 = estimate_slip_angles(state, beta_1, params)

forces = estimate_lateral_forces(state, params)
cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
cs_1 = estimate_cornering_stiffness(slip_1, forces, state, params)

cfg = params["tyre_model_ekf"][PASS_ID]
c_alpha_f, c_alpha_r = cfg["c_alpha_front_n_per_rad"], cfg["c_alpha_rear_n_per_rad"]
mu_fz_f, mu_fz_r = cfg["mu_fz_front_N"], cfg["mu_fz_rear_N"]

manifest = {"pass_id": PASS_ID, "data_file": RAW_FILE}

# --- Section 0: config summary, read live -----------------------------------

print("=" * 100)
print("SECTION 0 -- pass_1 configuration, read live from config/parameters.json")
print("=" * 100)
print(f"  c_alpha_front={c_alpha_f}  c_alpha_rear={c_alpha_r}  mu_fz_front={mu_fz_f}  mu_fz_rear={mu_fz_r}")
print(f"  Q_beta_var={cfg['Q_beta_var']}  Q_yaw_rate_var={cfg['Q_yaw_rate_var']}")
print(f"  R_ay_var={cfg['R_ay_var']}  R_yaw_rate_var={cfg['R_yaw_rate_var']}")
print()
manifest["config"] = {
    "c_alpha_front_n_per_rad": c_alpha_f, "c_alpha_rear_n_per_rad": c_alpha_r,
    "mu_fz_front_N": mu_fz_f, "mu_fz_rear_N": mu_fz_r,
    "Q_beta_var": cfg["Q_beta_var"], "Q_yaw_rate_var": cfg["Q_yaw_rate_var"],
    "R_ay_var": cfg["R_ay_var"], "R_yaw_rate_var": cfg["R_yaw_rate_var"],
}

# --- Section 1: NIS per channel ---------------------------------------------

chi2_df1 = float(chi2.ppf(0.95, df=1))
chi2_df2 = float(chi2.ppf(0.95, df=2))

innovation = result_1["innovation"][base_mask]
nis_combined = result_1["nis"][base_mask]
nis_yaw = innovation[:, 0] ** 2 / result_1["S_diag"][base_mask][:, 0]
nis_ay = innovation[:, 1] ** 2 / result_1["S_diag"][base_mask][:, 1]
f_yaw = float((nis_yaw > chi2_df1).mean())
f_ay = float((nis_ay > chi2_df1).mean())
f_comb = float((nis_combined > chi2_df2).mean())
mean_nis_comb = float(np.mean(nis_combined))

print("=" * 100)
print("SECTION 1 -- NIS per channel (methodology: inspect_ekf_pass1_evaluation.py Section 1)")
print("=" * 100)
print(f"  chi-square 95% bounds: df=1={chi2_df1:.4f}  df=2={chi2_df2:.4f}   "
      f"acceptance band (both ends gate): 3%-15%")
print(f"  yaw_rate exceedance={f_yaw:.4f}   ay exceedance={f_ay:.4f}   "
      f"combined exceedance={f_comb:.4f}   combined mean NIS={mean_nis_comb:.3f} (expect ~2 if calibrated)")
print()
manifest["nis"] = {"yaw_rate_exceedance": f_yaw, "ay_exceedance": f_ay,
                    "combined_exceedance": f_comb, "combined_mean_nis": mean_nis_comb,
                    "chi2_df1_95": chi2_df1, "chi2_df2_95": chi2_df2}

# --- Section 2: sign check ---------------------------------------------------

with open("config/channels.json", "r", encoding="utf-8") as f:
    channels_json = json.load(f)
LOW_SPEED_MAX_KMH = channels_json["corner_speed_thresholds"]["low_max"]

beta_deg_full = np.degrees(beta_1)
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

print("=" * 100)
print("SECTION 2 -- sign check (methodology: inspect_ekf_pass1_evaluation.py Section 3)")
print("=" * 100)
print(f"  MEDIAN GATE: {n_match_median}/{n_total} all corners, {n_match_median_racing}/{n_racing} racing-speed")
print(f"  PER-SAMPLE (racing-speed pooled, reported not gated): {pooled_frac:.4f} "
      f"({per_sample_frac_pooled_num}/{per_sample_frac_pooled_den})")
print()
manifest["sign_check"] = {
    "median_gate_all_corners": f"{n_match_median}/{n_total}",
    "median_gate_racing_speed": f"{n_match_median_racing}/{n_racing}",
    "per_sample_fraction_racing_speed": pooled_frac,
    "per_sample_num": per_sample_frac_pooled_num, "per_sample_den": per_sample_frac_pooled_den,
}

# --- Section 3: self-consistency R^2, simplified conjunction framing -------

in_corner_mask = np.zeros_like(t, dtype=bool)
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop > sl.start:
            in_corner_mask[sl] = True
corner_valid_mask = moving & racing_mask & in_corner_mask

print("=" * 100)
print("SECTION 3 -- self-consistency R^2 (methodology: inspect_ekf_dugoff_circularity.py Section 5)")
print("=" * 100)
print("SIMPLIFIED CONJUNCTION FRAMING: pass 1 never refit c_alpha (the curve is pass_0's,")
print("unchanged), so the two-part conjunction signature used in the pass 2-4 refit-loop")
print("entries (R^2 near ~0.997 AND c_alpha snapping back to a prior) does not apply here --")
print("there is no prior/posterior distinction to snap back to. The single relevant")
print("comparison is R^2 against the linear observer's ~0.997 level: pass_1's R^2 sits well")
print("below that, which is the original load-bearing evidence that calibration moved the")
print("filter's slip angles AWAY from restating its own assumed curve, not toward it.")
print()

r2_section3 = {}
for axle_name, alpha_c, Fy, c_a, mu_z in (
    ("front", slip_1["alpha_f_filt"], forces["Fy_f_filt"], c_alpha_f, mu_fz_f),
    ("rear", slip_1["alpha_r_filt"], forces["Fy_r_filt"], c_alpha_r, mu_fz_r),
):
    alpha_rad = alpha_c[corner_valid_mask]
    fy_meas = Fy[corner_valid_mask]
    finite = np.isfinite(alpha_rad) & np.isfinite(fy_meas)
    alpha_rad, fy_meas = alpha_rad[finite], fy_meas[finite]
    fy_pred = dugoff_lateral_force(alpha_rad, c_a, mu_z)
    resid = fy_meas - fy_pred
    ss_res = np.sum(resid ** 2)
    ss_tot = np.sum((fy_meas - np.mean(fy_meas)) ** 2)
    r2 = float(1.0 - ss_res / ss_tot)
    rms = float(np.sqrt(np.mean(resid ** 2)))
    print(f"  {axle_name}: n={len(alpha_rad)}  R^2={r2:.4f}  RMS residual={rms:.0f} N   "
          f"(linear-observer reference: ~0.997 at both axles)")
    r2_section3[axle_name] = {"r2": r2, "rms_residual_N": rms, "n": int(len(alpha_rad))}
print()
manifest["self_consistency_r2"] = r2_section3

# --- Section 4: onset and coverage per axle ---------------------------------

print("=" * 100)
print("SECTION 4 -- onset and coverage per axle (methodology: inspect_ekf_dugoff_circularity.py Section 6)")
print("=" * 100)

onset_section4 = {}
for axle_name, alpha_c, c_a, mu_z in (
    ("front", slip_1["alpha_f_filt"], c_alpha_f, mu_fz_f),
    ("rear", slip_1["alpha_r_filt"], c_alpha_r, mu_fz_r),
):
    tan_onset = mu_z / (2.0 * c_a)
    onset_deg = float(np.degrees(np.arctan(tan_onset)))
    alpha_deg = np.degrees(alpha_c)[base_mask]
    frac_beyond = float((np.abs(alpha_deg) > onset_deg).mean())
    p50a, p90a, p99a = np.percentile(np.abs(alpha_deg), [50, 90, 99])
    print(f"  {axle_name}: onset={onset_deg:.3f} deg   coverage={frac_beyond:.4f}   "
          f"|alpha| p50={p50a:.3f}  p90={p90a:.3f}  p99={p99a:.3f} deg")
    onset_section4[axle_name] = {"onset_deg": onset_deg, "coverage_fraction": frac_beyond,
                                  "alpha_abs_p50_deg": float(p50a), "alpha_abs_p90_deg": float(p90a),
                                  "alpha_abs_p99_deg": float(p99a)}
print("  (kinematic-population reference: front 34.0%, rear 6.95%)")
print()
manifest["onset_coverage"] = onset_section4

# --- Section 5: h2-vs-ay, apex_3 population, vs kinematic 0.887 ------------

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

alpha_f_1 = slip_1["alpha_f_filt"]
alpha_r_1 = slip_1["alpha_r_filt"]
m_kg = params["vehicle"]["mass_kg"]

h2_pred_apex = np.full(len(idx), np.nan)
for k, i in enumerate(idx):
    Fy_f = dugoff_lateral_force(alpha_f_1[i], c_alpha_f, mu_fz_f)
    Fy_r = dugoff_lateral_force(alpha_r_1[i], c_alpha_r, mu_fz_r)
    h2_pred_apex[k] = (Fy_f + Fy_r) / m_kg

ay_apex = ay[idx]
r_apex = float(np.corrcoef(h2_pred_apex, ay_apex)[0, 1])

print("=" * 100)
print("SECTION 5 -- h2-vs-ay, apex_3 population (methodology: inspect_ekf_dugoff_sanity_checks.py "
      "check_h2_vs_ay_consistency, reproduced with pass_1's own alpha/curve)")
print("=" * 100)
print(f"  n={len(idx)}  corr(h2_pred, ay_meas)={r_apex:+.4f}   (kinematic reference: 0.887)")
print()
manifest["h2_vs_ay_apex"] = {"n": int(len(idx)), "correlation": r_apex, "kinematic_reference": 0.887}

# --- Section 6: WP-S4b reference-spread comparison --------------------------

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)
Cr_a_arr = cs_a["C_linear_ref_r"]
Cr_1_arr = cs_1["C_linear_ref_r"]

print("=" * 100)
print("SECTION 6 -- WP-S4b reference-spread comparison (methodology: inspect_cs_linear_ref_"
      "staleness.py Section 3c, exact near-zero-alpha_r(A) sample selection)")
print("=" * 100)

spread_section6 = {}
for label, Clr_arr in (("kinematic", Cr_a_arr), ("pass_1", Cr_1_arr)):
    medians = []
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled = []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = base_mask[sl] & np.isfinite(alpha_r_a[sl]) & (np.abs(alpha_r_a[sl]) < near_zero_rad)
            if not m.any():
                continue
            pooled.append(Clr_arr[sl][m])
        if not pooled:
            continue
        vals = np.concatenate(pooled)
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            continue
        medians.append((cid, float(np.median(vals))))
    if medians:
        lo_cid, lo_v = min(medians, key=lambda x: x[1])
        hi_cid, hi_v = max(medians, key=lambda x: x[1])
        ratio = hi_v / lo_v if lo_v else float("nan")
        print(f"  {label:9s}: {len(medians)}/{len(stable_ids)} corners; range {lo_v:.0f} (C{lo_cid}) - "
              f"{hi_v:.0f} (C{hi_cid}) N/rad   ratio max/min={ratio:.2f}")
        spread_section6[label] = {"n_corners": len(medians), "min_N_per_rad": lo_v, "max_N_per_rad": hi_v,
                                   "ratio_max_min": ratio}
print()
manifest["wp_s4b_reference_spread"] = spread_section6

# --- Section 7: filter stability ---------------------------------------------

print("=" * 100)
print("SECTION 7 -- filter stability")
print("=" * 100)

idx_c2 = np.where((t >= 883.0) & (t <= 885.5))[0]
beta_deg_c2 = np.degrees(beta_1[idx_c2])
max_abs_c2 = float(np.max(np.abs(beta_deg_c2)))
step_diff_c2 = np.diff(beta_deg_c2)
max_step_c2 = float(np.max(np.abs(step_diff_c2)))
i_worst = int(np.argmax(np.abs(step_diff_c2)))
sign_flip_mask = (np.sign(beta_deg_c2[:-1]) != np.sign(beta_deg_c2[1:])) & \
                  (np.sign(beta_deg_c2[:-1]) != 0) & (np.sign(beta_deg_c2[1:]) != 0)
any_sign_flip = bool(sign_flip_mask.any())
n_sign_flips = int(sign_flip_mask.sum())

print("  C2 excursion window, t=883.0-885.5s:")
print(f"    pass_0 reference: max|beta|=14.119 deg, max single-step=10.826 deg (SIGN-FLIPPING)")
print(f"    pass_1 (this run): max|beta|={max_abs_c2:.3f} deg, max single-step={max_step_c2:.3f} deg "
      f"(at t={t[idx_c2[i_worst]]:.3f}s: {beta_deg_c2[i_worst]:+.3f} -> {beta_deg_c2[i_worst+1]:+.3f})")
print(f"    any single-step sign flip in this window: {any_sign_flip} ({n_sign_flips} flip(s) found)")
print()

diverged_frac = float(result_1["diverged_mask"][base_mask].mean())
print(f"  diverged_mask fraction over full masked population (n={int(base_mask.sum())}): {diverged_frac:.4f}")
print()

print("  divergence thresholds, READ LIVE from config/parameters.json tyre_model_ekf.pass_1:")
print(f"    nis_window_samples={cfg['nis_window_samples']}   nis_chi2_bound={cfg['nis_chi2_bound']}   "
      f"nis_flag_fraction={cfg['nis_flag_fraction']}")
print("  CAVEAT, stated explicitly: these remain the ORIGINAL PLACEHOLDER defaults and were")
print("  never validated against a real run. The short-run blind-spot quantification already")
print("  on record (thesis_notes.md, 'blind-spot quantification') was measured against a")
print("  DIFFERENT, never-implemented threshold pair (nis_window_samples=25, nis_flag_")
print("  fraction=1.0) -- it does NOT describe the monitor actually running here.")
print()

manifest["filter_stability"] = {
    "c2_excursion": {
        "pass_0_reference_max_beta_deg": 14.119, "pass_0_reference_max_step_deg": 10.826,
        "pass_0_reference_sign_flipping": True,
        "pass_1_max_beta_deg": max_abs_c2, "pass_1_max_step_deg": max_step_c2,
        "pass_1_any_sign_flip": any_sign_flip, "pass_1_n_sign_flips": n_sign_flips,
    },
    "diverged_mask_fraction": diverged_frac,
    "thresholds_live_from_config": {
        "nis_window_samples": cfg["nis_window_samples"], "nis_chi2_bound": cfg["nis_chi2_bound"],
        "nis_flag_fraction": cfg["nis_flag_fraction"],
    },
    "thresholds_caveat": "Original placeholder defaults, never validated. The recorded "
                          "blind-spot quantification used a different, never-implemented "
                          "pair (25 / 1.0) and does not describe this monitor.",
}

# --- Section 8: descriptives and provenance ----------------------------------

beta_masked_deg = np.degrees(beta_1[base_mask])
p1b, p25b, p50b, p75b, p99b = np.percentile(beta_masked_deg, [1, 25, 50, 75, 99])
max_abs_beta = float(np.max(np.abs(beta_masked_deg)))

try:
    git_hash = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
except Exception as e:
    git_hash = f"UNAVAILABLE ({e})"

run_timestamp = datetime.now(timezone.utc).isoformat()

print("=" * 100)
print("SECTION 8 -- descriptives and provenance")
print("=" * 100)
print(f"  beta (deg, masked population): p1={p1b:.3f}  p25={p25b:.3f}  p50={p50b:.3f}  "
      f"p75={p75b:.3f}  p99={p99b:.3f}  max|beta|={max_abs_beta:.3f}")
print(f"  masked population n={int(base_mask.sum())}  (mask = moving & ~kerb & valid-lap racing time)")
print(f"  git commit hash: {git_hash}")
print(f"  run timestamp (UTC): {run_timestamp}")
print()

manifest["descriptives"] = {
    "beta_deg_p1": float(p1b), "beta_deg_p25": float(p25b), "beta_deg_p50": float(p50b),
    "beta_deg_p75": float(p75b), "beta_deg_p99": float(p99b), "beta_deg_max_abs": max_abs_beta,
    "masked_population_n": int(base_mask.sum()),
    "mask_definition": "moving & ~kerb & valid-lap racing time",
}
manifest["provenance"] = {"git_commit_hash": git_hash, "run_timestamp_utc": run_timestamp}

with open(MANIFEST_PATH, "w") as fh:
    json.dump(manifest, fh, indent=2)

print(f"manifest written: {MANIFEST_PATH}")
