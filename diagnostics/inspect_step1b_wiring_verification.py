# WP-N2 Step 1b: wiring verification. Three parts, run in one script so
# they share one data load: (0) empirical check of the "effective_params
# equals raw params for this dataset" claim the Step 1b proposal's
# verification plan rested on -- diffed, not reasoned about; (1) the
# yaw-stability min_beta_std_rad gate-population shift, measured directly
# via modules.yaw_stability.calculate_observed_stability's own diagnostics
# dict rather than inferred; (2) full reproduction check against the frozen
# pass-1 baseline (diagnostics/pass1_final_validation_manifest.json),
# reusing inspect_pass1_final_validation.py's own section methodology,
# run under whatever config/parameters.json stability_estimation.
# sideslip_source is set to on disk when this script starts (the calling
# session sets it to "ekf_pass_1" before this run, "kinematic" after --
# see thesis_notes.md for the record of which). Read-only itself: writes
# no config, only reads whatever the caller already set.

import json

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.yaw_stability import calculate_filtered_yaw_acceleration, calculate_observed_stability
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MANIFEST_PATH = "diagnostics/pass1_final_validation_manifest.json"

data = parse_csv(RAW_FILE)
raw_params = load_parameters()
live_sideslip_source = raw_params["stability_estimation"].get("sideslip_source", "kinematic")

# =============================================================================
print("=" * 100)
print("SECTION 0 -- effective_params (production, cap='Best available'/None, "
      "setup_data=None) vs raw params, key by key")
print("=" * 100)

import os
_CAP = int(os.environ["STEP1B_VERIFY_CAP"]) if os.environ.get("STEP1B_VERIFY_CAP") else None
resolved = resolve_accuracy(raw_params, setup_data=None, cap=_CAP)
effective_params = apply_resolved_vehicle(raw_params, resolved)
print(f"  cap used this run: {_CAP!r} ({'Best available' if _CAP is None else f'Level<={_CAP}'})")


def _diff_dict(a, b, path=""):
    diffs = []
    for k in sorted(set(a.keys()) | set(b.keys()), key=str):
        pa = f"{path}.{k}" if path else str(k)
        if k not in a:
            diffs.append((pa, "<absent in raw>", b[k]))
        elif k not in b:
            diffs.append((pa, a[k], "<absent in effective>"))
        elif isinstance(a[k], dict) and isinstance(b[k], dict):
            diffs.extend(_diff_dict(a[k], b[k], pa))
        else:
            va, vb = a[k], b[k]
            try:
                eq = bool(np.all(va == vb))
            except Exception:
                eq = (va == vb)
            if not eq:
                diffs.append((pa, va, vb))
    return diffs


for block in ["vehicle", "stability_estimation", "tyre_model_ekf", "classification"]:
    d = _diff_dict(raw_params[block], effective_params[block], block)
    if not d:
        print(f"  {block}: IDENTICAL, key for key.")
    else:
        print(f"  {block}: {len(d)} differing key(s):")
        for path, va, vb in d:
            print(f"    {path}:")
            print(f"      raw       = {va}")
            print(f"      effective = {vb}")
print()
print(f"  live sideslip_source (both raw and effective, unaffected by apply_resolved_vehicle): "
      f"{live_sideslip_source!r}")
print()

# =============================================================================
print("=" * 100)
print("SECTION 1 -- yaw-stability min_beta_std_rad gate, kinematic vs ekf_pass_1, "
      "under effective_params")
print("=" * 100)

state = prepare_vehicle_state(data["channels"], effective_params)
laps = data.get("laps", [])
se = effective_params["stability_estimation"]
vp = effective_params["vehicle"]

beta_kin = estimate_sideslip(state, effective_params)
ekf_result = estimate_sideslip_ekf_dugoff(state, effective_params, pass_id="pass_1")
beta_ekf = ekf_result["beta_with_fallback"]


def _run_stability_with_diagnostics(beta, label):
    from modules.stability_analysis import _build_inout_lap_mask
    t = state["time"]
    sr = state["sample_rate_hz"]
    v = state["v_mps"]
    yaw_rate = state["yaw_rate_radps"]
    delta_f = state["delta_f_rad"]
    ax = state["ax_mps2"]
    az_g = state.get("az_g")
    s_m = state.get("s_m")
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    moving = moving & ~_build_inout_lap_mask(t, laps)
    Iz = vp["yaw_inertia_kgm2"]
    yaw_accel_filt = calculate_filtered_yaw_acceleration(yaw_rate, t, sr, se["yaw_stability_accel_window_s"])
    Mz_inertial = Iz * yaw_accel_filt
    az_mps2 = az_g * 9.81 if az_g is not None else None
    stability_observed, stability_valid, diag = calculate_observed_stability(
        s_m=s_m, beta_rad=beta, delta_f_rad=delta_f, v_mps=v, ax_mps2=ax, az_mps2=az_mps2,
        mz_inertial_Nm=Mz_inertial, valid_mask=moving,
        grid_step_m=se["yaw_stability_grid_step_m"], window_m=se["yaw_stability_window_m"],
        min_samples=se["yaw_stability_min_samples"], ridge=se["yaw_stability_ridge"],
        min_beta_std_rad=se["yaw_stability_min_beta_std_rad"],
    )
    print(f"  {label}: n_grid_points={diag['n_grid_points']}  n_grid_valid={diag['n_grid_valid']}  "
          f"skip[min_samples]={diag['skip_counts']['min_samples']}  "
          f"skip[beta_std]={diag['skip_counts']['beta_std']}  "
          f"skip[linalg_error]={diag['skip_counts']['linalg_error']}")
    print(f"  {label}: stability_valid population (per-sample, moving&racing not separately masked here) "
          f"= {int(stability_valid.sum())} / {len(stability_valid)}")
    return diag, stability_valid


diag_kin, valid_kin = _run_stability_with_diagnostics(beta_kin, "kinematic")
diag_ekf, valid_ekf = _run_stability_with_diagnostics(beta_ekf, "ekf_pass_1")
print()
print(f"  DELTA: n_grid_valid {diag_kin['n_grid_valid']} -> {diag_ekf['n_grid_valid']} "
      f"({diag_ekf['n_grid_valid'] - diag_kin['n_grid_valid']:+d})   "
      f"skip[beta_std] {diag_kin['skip_counts']['beta_std']} -> {diag_ekf['skip_counts']['beta_std']} "
      f"({diag_ekf['skip_counts']['beta_std'] - diag_kin['skip_counts']['beta_std']:+d})")
print(f"  DELTA: stability_valid samples {int(valid_kin.sum())} -> {int(valid_ekf.sum())} "
      f"({int(valid_ekf.sum()) - int(valid_kin.sum()):+d})")
print()

# =============================================================================
print("=" * 100)
print(f"SECTION 2 -- full comparison against frozen manifest, "
      f"live sideslip_source={live_sideslip_source!r}")
print("=" * 100)

with open(MANIFEST_PATH, "r") as f:
    manifest = json.load(f)

if live_sideslip_source != "ekf_pass_1":
    print("  SKIPPED: config/parameters.json stability_estimation.sideslip_source is "
          f"{live_sideslip_source!r}, not 'ekf_pass_1'. This section only runs meaningfully "
          "with the switch flipped.")
else:
    beta_1 = beta_ekf  # already computed above under effective_params, pass_1
    slip_a = estimate_slip_angles(state, beta_kin, effective_params)
    slip_1 = estimate_slip_angles(state, beta_1, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs_1 = estimate_cornering_stiffness(slip_1, forces, state, effective_params)

    cfg = effective_params["tyre_model_ekf"]["pass_1"]
    c_alpha_f, c_alpha_r = cfg["c_alpha_front_n_per_rad"], cfg["c_alpha_rear_n_per_rad"]
    mu_fz_f, mu_fz_r = cfg["mu_fz_front_N"], cfg["mu_fz_rear_N"]

    t = state["time"]
    ay = state["ay_mps2"]
    v_kmh = state["v_mps"] * 3.6
    s_m = state.get("s_m")
    moving_raw = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
    laps_by_number = {l["lap_number"]: l for l in laps}
    valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
    racing_mask = np.zeros_like(t, dtype=bool)
    for s, e in valid_windows:
        racing_mask |= (t >= s) & (t <= e)
    base_mask = moving & racing_mask

    corners = data.get("corners", [])
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

    def _report(label, got, expected, tol=None):
        if tol is None:
            match = (got == expected)
        else:
            match = abs(got - expected) <= tol
        flag = "MATCH" if match else "DIFFERS"
        print(f"  [{flag}] {label}: got={got}  manifest={expected}")
        return match

    all_match = True

    # Section 1 of the manifest: NIS
    innovation = ekf_result["innovation"][base_mask]
    nis_combined = ekf_result["nis"][base_mask]
    S = ekf_result["S_diag"][base_mask]
    nis_yaw = innovation[:, 0] ** 2 / S[:, 0]
    nis_ay = innovation[:, 1] ** 2 / S[:, 1]
    chi2_df1 = float(chi2.ppf(0.95, df=1))
    chi2_df2 = float(chi2.ppf(0.95, df=2))
    f_yaw = float((nis_yaw > chi2_df1).mean())
    f_ay = float((nis_ay > chi2_df1).mean())
    f_comb = float((nis_combined > chi2_df2).mean())
    mean_nis_comb = float(np.mean(nis_combined))
    print("  -- NIS --")
    all_match &= _report("yaw_rate_exceedance", round(f_yaw, 4), manifest["nis"]["yaw_rate_exceedance"], tol=1e-6)
    all_match &= _report("ay_exceedance", round(f_ay, 4), manifest["nis"]["ay_exceedance"], tol=1e-6)
    all_match &= _report("combined_exceedance", round(f_comb, 4), manifest["nis"]["combined_exceedance"], tol=1e-6)
    all_match &= _report("combined_mean_nis", round(mean_nis_comb, 3), manifest["nis"]["combined_mean_nis"], tol=1e-3)

    # Section 2 of the manifest: sign check (EXTERNAL: beta sign vs measured ay direction)
    with open("config/channels.json", "r", encoding="utf-8") as f:
        channels_json = json.load(f)
    LOW_SPEED_MAX_KMH = channels_json["corner_speed_thresholds"]["low_max"]
    beta_deg_full = np.degrees(beta_1)
    n_match_median = n_total = n_match_median_racing = n_racing = 0
    per_sample_frac_pooled_num = per_sample_frac_pooled_den = 0
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
    print("  -- SIGN CHECK (external: beta sign vs measured ay direction) --")
    all_match &= _report("median_gate_all_corners", f"{n_match_median}/{n_total}",
                          manifest["sign_check"]["median_gate_all_corners"])
    all_match &= _report("median_gate_racing_speed", f"{n_match_median_racing}/{n_racing}",
                          manifest["sign_check"]["median_gate_racing_speed"])
    all_match &= _report("per_sample_fraction_racing_speed", round(pooled_frac, 4),
                          manifest["sign_check"]["per_sample_fraction_racing_speed"], tol=1e-6)

    # Section 5 of the manifest: h2-vs-ay, apex_3 population (EXTERNAL: predicted ay vs measured ay)
    apex_mask = np.zeros_like(t, dtype=bool)
    for c in corners:
        start_t, end_t = c["segments"]["apex_3"]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi <= lo:
            apex_half = effective_params["stability_estimation"]["apex_half_window_samples"]
            centre = lo
            lo = max(0, centre - apex_half)
            hi = min(len(t), centre + apex_half + 1)
        apex_mask[lo:hi] = True
    apex_pop_mask = base_mask & apex_mask
    idx = np.where(apex_pop_mask)[0]
    alpha_f_1 = slip_1["alpha_f_filt"]
    alpha_r_1 = slip_1["alpha_r_filt"]
    m_kg = effective_params["vehicle"]["mass_kg"]
    h2_pred_apex = np.full(len(idx), np.nan)
    for k, i in enumerate(idx):
        Fy_f = dugoff_lateral_force(alpha_f_1[i], c_alpha_f, mu_fz_f)
        Fy_r = dugoff_lateral_force(alpha_r_1[i], c_alpha_r, mu_fz_r)
        h2_pred_apex[k] = (Fy_f + Fy_r) / m_kg
    ay_apex = ay[idx]
    r_apex = float(np.corrcoef(h2_pred_apex, ay_apex)[0, 1])
    print("  -- h2-vs-ay, apex_3 (external: Dugoff-predicted lateral accel vs measured ay) --")
    all_match &= _report("n", int(len(idx)), manifest["h2_vs_ay_apex"]["n"])
    all_match &= _report("correlation", round(r_apex, 4), manifest["h2_vs_ay_apex"]["correlation"], tol=1e-4)

    # Section 3 of the manifest: self-consistency R^2
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
    print("  -- self-consistency R^2 --")
    for axle_name, alpha_c, Fy, c_a, mu_z, manifest_key in (
        ("front", slip_1["alpha_f_filt"], forces["Fy_f_filt"], c_alpha_f, mu_fz_f, "front"),
        ("rear", slip_1["alpha_r_filt"], forces["Fy_r_filt"], c_alpha_r, mu_fz_r, "rear"),
    ):
        alpha_rad = alpha_c[corner_valid_mask]
        fy_meas = Fy[corner_valid_mask]
        finite = np.isfinite(alpha_rad) & np.isfinite(fy_meas)
        alpha_rad, fy_meas = alpha_rad[finite], fy_meas[finite]
        fy_pred = dugoff_lateral_force(alpha_rad, c_a, mu_z)
        resid = fy_meas - fy_pred
        r2 = float(1.0 - np.sum(resid ** 2) / np.sum((fy_meas - np.mean(fy_meas)) ** 2))
        all_match &= _report(f"{axle_name} R^2", round(r2, 4),
                              manifest["self_consistency_r2"][manifest_key]["r2"], tol=1e-4)

    print()
    print(f"  OVERALL: {'ALL SECTIONS MATCH' if all_match else 'AT LEAST ONE SECTION DIFFERS'}")

print()
print("=" * 100)
print("DONE")
print("=" * 100)
