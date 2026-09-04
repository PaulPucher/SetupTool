# One-shot per-session Dugoff tyre-curve fit + EKF validation chain.
# EXPERIMENTAL (PLAN.md unsupervised package, Phase 2) -- automates the
# procedure that was previously run by hand, one script per step, for
# the carried-forward pass-0/pass-1 estimator (thesis_notes.md "WP-N2
# carry-forward decision: pass 1"). NOT wired into the UI or the
# production analysis thread (ui/views/outing_form.py's
# StabilityAnalysisThread never imports this module). Pure Python/
# numpy/scipy, no Qt -- same modules/ contract as every other file in
# this package.
#
# Method lineage, pointer lines only (CLAUDE.md citation-location
# rule; full anchors in thesis_notes.md):
#   step (a) c_alpha       -- diagnostics/fit_dugoff_first_pass.py, WP-N1b entry
#   step (b) mu_fz         -- same script, bounded-search widening loop
#   step (c) R derivation  -- config/parameters.json tyre_model_ekf.pass_1's
#                              R_ay_derivation/R_yaw_rate_derivation/r_q_sweep_note,
#                              and diagnostics/inspect_ekf_pass1_rQ_sweep.py's 2-D grid
#   step (d) EKF run       -- diagnostics/sideslip_ekf_dugoff.py (imported directly,
#                              not duplicated -- see the note on that dependency below)
#   step (e) validation    -- diagnostics/inspect_pass1_final_validation.py's
#                              five sections (NIS, sign check, self-consistency
#                              R^2 is NOT reproduced here, onset/coverage, h2-vs-ay)
#
# DEPENDENCY NOTE (neutral engineering, not a science decision): this
# file lives in modules/ but imports diagnostics/sideslip_ekf_dugoff.py
# for the actual EKF recursion, inverting the project's usual one-way
# diagnostics-depends-on-modules direction. Deliberate: the EKF loop is
# ~150 lines of numerically sensitive Jacobian/update code; duplicating
# it here would let the two copies silently diverge. Every other
# tyre_model_ekf.pass_N config comment's "no modules/ consumer" note
# describes historical fact as of when it was written and is not
# retroactively edited by this file's existence (CLAUDE.md: config
# changes stay additive, existing keys/comments untouched).
#
# WHAT THIS DOES NOT REPRODUCE: fit_dugoff_first_pass.py's superseded
# raw-OLS c_alpha comparison (explicitly "not used" in the frozen
# procedure) and Module-4b's C_linear_ref self-consistency R^2 (pass-1
# validation Section 3 -- a comparison against the REJECTED linear
# observer's own R^2, meaningless without that observer's output on
# hand, which no per-session caller of this module is assumed to have).

import subprocess
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import minimize_scalar, minimize
from scipy.stats import chi2

from modules.stability_analysis import (
    prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_vertical_loads, estimate_cornering_stiffness,
    load_car_data,
)
from modules.tyre_model import dugoff_lateral_force
from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness
from modules.nis_gate import evaluate_gate
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from diagnostics.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka

PACEJKA_START_GUESS = (12.0, 1.9, 8000.0, 0.97)  # chair's own starting guess, PLAN.md Phase 3 work order

CHI2_DF1_95 = float(chi2.ppf(0.95, df=1))
CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))
NEAR_ZERO_SLIP_DEG = 0.2  # matches WP-S3b/S3c/S4b/pass-1-validation's own near-zero-alpha_r population


def _base_mask(state, laps):
    t = state["time"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
    racing_mask = np.zeros_like(t, dtype=bool)
    for s, e in valid_windows:
        racing_mask |= (t >= s) & (t <= e)
    return moving & racing_mask


def _fit_axle(alpha, Fy, Fz, C_alpha, CS_ratio, base_mask, cfg):
    """Steps (a)+(b): c_alpha from Module 4b's own linear-regime
    indicator (CS_ratio==1.0), then a bounded mu_fz least-squares fit,
    c_alpha held fixed. Bracket widening and the >=0.95 non-interior-
    optimum check reproduce fit_dugoff_first_pass.py's loop exactly --
    this is the safeguard that must catch the pass-4 rear degeneracy
    (mu_fz drifting to the bracket ceiling, curve collapsing to pure-
    linear) rather than silently accepting it.
    """
    m_c4b = base_mask & np.isfinite(C_alpha) & (CS_ratio == 1.0)
    c_alpha_pop = C_alpha[m_c4b]
    c_alpha_used = float(np.median(c_alpha_pop)) if len(c_alpha_pop) else float("nan")
    sign_ok = c_alpha_used > 0 if c_alpha_pop.size else False

    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz)
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]

    if not sign_ok or len(a2) < 10:
        return {
            "c_alpha_n_per_rad": c_alpha_used, "c_alpha_source_mask_n": int(len(c_alpha_pop)),
            "c_alpha_sign_check_ok": sign_ok, "mu_fz_N": float("nan"), "mu_fz_bound_fraction": float("nan"),
            "mu_fz_fit_n_samples": int(len(a2)), "mu_fz_fit_rms_resid_N": float("nan"),
            "degenerate": True, "degenerate_reason": "c_alpha sign check failed or insufficient fit samples",
            "residuals": None, "mean_axle_fz_N": float("nan"),
        }

    def _sse(mu_fz, alpha_arr=a2, fy_arr=f2, c_alpha_fixed=c_alpha_used):
        pred = dugoff_lateral_force(alpha_arr, c_alpha_fixed, mu_fz)
        return float(np.sum((pred - fy_arr) ** 2))

    hi_bound = cfg["mu_fz_hi_bound_initial_multiplier"] * float(np.max(np.abs(f2)))
    widen_mult = cfg["mu_fz_hi_bound_widen_multiplier"]
    max_attempts = int(cfg["mu_fz_max_widen_attempts"])
    bound_thresh = cfg["mu_fz_bound_fraction_degenerate_threshold"]

    mu_fz_bound_fraction = 1.0
    attempts = 0
    mu_fz = float("nan")
    while mu_fz_bound_fraction > bound_thresh and attempts < max_attempts:
        opt = minimize_scalar(_sse, bounds=(1.0, hi_bound), method="bounded")
        mu_fz = float(opt.x)
        mu_fz_bound_fraction = mu_fz / hi_bound
        attempts += 1
        if mu_fz_bound_fraction > bound_thresh and attempts < max_attempts:
            hi_bound *= widen_mult

    degenerate = mu_fz_bound_fraction > bound_thresh
    pred2 = dugoff_lateral_force(a2, c_alpha_used, mu_fz)
    resid = f2 - pred2
    rms2 = float(np.sqrt(np.mean(resid ** 2)))
    mean_fz = float(np.mean(z2))

    return {
        "c_alpha_n_per_rad": c_alpha_used, "c_alpha_source_mask_n": int(len(c_alpha_pop)),
        "c_alpha_sign_check_ok": sign_ok,
        "mu_fz_N": mu_fz, "mu_fz_search_bracket_N": [1.0, hi_bound],
        "mu_fz_bracket_widen_attempts": attempts, "mu_fz_bound_fraction": mu_fz_bound_fraction,
        "mu_fz_fit_n_samples": int(len(a2)), "mu_fz_fit_rms_resid_N": rms2,
        "mean_axle_fz_N": mean_fz, "effective_mu": (mu_fz / mean_fz) if mean_fz else float("nan"),
        "degenerate": degenerate,
        "degenerate_reason": ("mu_fz fit did not converge to an interior optimum -- hit the "
                               f"widened search bracket ceiling after {attempts} attempt(s) "
                               f"(bound_fraction={mu_fz_bound_fraction:.6f})") if degenerate else None,
        "residuals": resid, "residual_mask": m2,
    }


def _r_from_residuals(resid_front_full, resid_rear_full, common_mask, mass_kg):
    """Step (c), Method A: fit-residual-based R_ay, inter-axle
    correlation MEASURED (not assumed) on this session's own fit
    residuals -- reproduces tyre_model_ekf.pass_1.R_ay_derivation's
    formula (combined variance of two correlated error sources, each
    converted from a force residual to an acceleration residual by
    dividing by vehicle mass, since ay's measurement model is
    (Fy_f+Fy_r)/m). resid_front_full/resid_rear_full are full-length
    (zero outside each axle's own fit mask); common_mask selects the
    population both axles' residuals are actually defined over.
    """
    rf = resid_front_full[common_mask]
    rr = resid_rear_full[common_mask]
    std_f = float(np.sqrt(np.mean(rf ** 2))) / mass_kg
    std_r = float(np.sqrt(np.mean(rr ** 2))) / mass_kg
    rho = float(np.corrcoef(rf, rr)[0, 1]) if len(rf) > 2 else 0.0
    r_ay_var = std_f ** 2 + std_r ** 2 + 2.0 * rho * std_f * std_r
    return r_ay_var, std_f, std_r, rho


def _sign_check(t, s_m, beta, ay, moving, corners_by_stable_id, laps_by_number, racing_ids):
    beta_deg = np.degrees(beta)
    n_match_median = n_total = 0
    n_match_median_racing = n_racing = 0
    per_num = per_den = 0
    for cid, instances in corners_by_stable_id.items():
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled_ay, pooled_beta = [], []
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
            pooled_beta.append(beta_deg[sl][m])
        if not pooled_ay:
            continue
        ay_cat = np.concatenate(pooled_ay)
        beta_cat = np.concatenate(pooled_beta)
        med_ay = float(np.median(ay_cat))
        med_beta = float(np.median(beta_cat))
        dir_sign = np.sign(med_ay)
        median_match = (np.sign(med_beta) == -dir_sign) if dir_sign != 0 else None
        per_sample_match = (np.sign(beta_cat) == -dir_sign) if dir_sign != 0 else np.zeros_like(beta_cat, dtype=bool)
        n_total += 1
        n_match_median += int(bool(median_match))
        if cid in racing_ids:
            n_racing += 1
            n_match_median_racing += int(bool(median_match))
            per_num += int(np.sum(per_sample_match))
            per_den += len(per_sample_match)
    return {
        "median_gate_all": f"{n_match_median}/{n_total}",
        "median_gate_racing": f"{n_match_median_racing}/{n_racing}",
        "median_gate_racing_fraction": (n_match_median_racing / n_racing) if n_racing else float("nan"),
        "per_sample_fraction_racing": (per_num / per_den) if per_den else float("nan"),
    }


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo or s_m is None:
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


def fit_session(data, params, data_file_path=None):
    """Run the one-shot per-session Dugoff fit + EKF validation chain
    (steps a-e) and return a fit manifest dict. `data` is modules.
    csv_parser.parse_csv's output; `params` is load_parameters's raw
    dict (or an accuracy-resolved variant -- this function reads
    params["tyre_fit_auto"] plus whatever prepare_vehicle_state/
    estimate_* already need, nothing more).

    STATUS FIELD ("ok"/"marginal"/"degenerate"), proposed boundaries
    (config tyre_fit_auto.sign_check_degenerate_fraction/marginal_
    fraction, nis_gross_miscalibration_fraction, mu_fz_bound_fraction_
    degenerate_threshold -- all a PROPOSAL, not reviewed):
      DEGENERATE if any of: either axle's c_alpha sign check fails or
        has <10 fit samples; either axle's mu_fz fit hits its widened
        bracket ceiling (the pass-4 rear failure mode -- a curve that
        has collapsed to pure-linear must never be reported as ok);
        the racing-speed sign-check median-gate fraction is below
        sign_check_degenerate_fraction (0.5 -- beta's sign would be no
        better than a coin flip, the estimator conveys no directional
        information); OR the best available R grid point still leaves
        either channel's NIS exceedance above nis_gross_miscalibration_
        fraction (0.5 -- half of all samples statistically inconsistent
        with the filter's own uncertainty model, an order of magnitude
        beyond the 3-15% target band).
      MARGINAL if not degenerate but either: no grid point landed both
        NIS channels inside [nis_band_low, nis_band_high] (the 2-D
        sweep found only a nearest-candidate, not a genuine in-band
        point); or the racing-speed sign-check median-gate fraction is
        below sign_check_marginal_fraction (0.7) -- directionally
        mostly right but not to the standard the frozen pass-1
        baseline met (8/11 = 0.727 on Dubai).
      OK otherwise: an in-band R grid point exists, sign check clears
        0.7, and neither axle degenerated.
    """
    cfg = params["tyre_fit_auto"]
    vp = params["vehicle"]
    mass_kg = vp["mass_kg"]

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        return {"status": "degenerate", "degenerate_reason": "prepare_vehicle_state returned None "
                "-- required channels missing from this session", "data_file": data_file_path}

    laps = data.get("laps", [])
    base_mask = _base_mask(state, laps)

    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)
    fz = estimate_vertical_loads(state, forces, params)
    cs_kin = estimate_cornering_stiffness(slip_kin, forces, state, params)

    # --- steps (a)+(b): per-axle fit ------------------------------------
    front_fit = _fit_axle(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], fz["fz_f_N"],
                           cs_kin["C_alpha_f"], cs_kin["CS_ratio_f"], base_mask, cfg)
    rear_fit = _fit_axle(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], fz["fz_r_N"],
                          cs_kin["C_alpha_r"], cs_kin["CS_ratio_r"], base_mask, cfg)

    manifest = {
        "data_file": data_file_path,
        "laps_used": sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis")),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "axles": {"front": {k: v for k, v in front_fit.items() if k not in ("residuals", "residual_mask")},
                  "rear": {k: v for k, v in rear_fit.items() if k not in ("residuals", "residual_mask")}},
    }
    try:
        git_hash = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    except Exception as e:
        git_hash = f"UNAVAILABLE ({e})"
    manifest["git_commit_hash"] = git_hash

    if front_fit["degenerate"] or rear_fit["degenerate"]:
        manifest["status"] = "degenerate"
        manifest["degenerate_reason"] = "; ".join(
            f"{axle}: {fit['degenerate_reason']}" for axle, fit in (("front", front_fit), ("rear", rear_fit))
            if fit["degenerate_reason"]
        )
        return manifest

    # --- step (c): R derivation ------------------------------------------
    common_mask = front_fit["residual_mask"] & rear_fit["residual_mask"]
    resid_front_full = np.zeros_like(base_mask, dtype=float)
    resid_rear_full = np.zeros_like(base_mask, dtype=float)
    resid_front_full[front_fit["residual_mask"]] = front_fit["residuals"]
    resid_rear_full[rear_fit["residual_mask"]] = rear_fit["residuals"]
    r_ay_var_derived, std_f, std_r, rho = _r_from_residuals(resid_front_full, resid_rear_full, common_mask, mass_kg)

    interim_cfg = {
        "c_alpha_front_n_per_rad": front_fit["c_alpha_n_per_rad"], "c_alpha_rear_n_per_rad": rear_fit["c_alpha_n_per_rad"],
        "mu_fz_front_N": front_fit["mu_fz_N"], "mu_fz_rear_N": rear_fit["mu_fz_N"],
        "Q_beta_var": cfg["q_beta_var"], "Q_yaw_rate_var": cfg["q_yaw_rate_var"],
        "P0_beta_var": cfg["p0_beta_var"], "P0_yaw_rate_var": cfg["p0_yaw_rate_var"],
        "R_ay_var": r_ay_var_derived, "R_yaw_rate_var": cfg["r_yaw_rate_interim_seed_var"],
        "beta_hard_bound_deg": cfg["beta_hard_bound_deg"],
        "nis_window_samples": cfg["nis_window_samples"], "nis_chi2_bound": cfg["nis_chi2_bound"],
        "nis_flag_fraction": cfg["nis_flag_fraction"],
    }
    params_interim = dict(params)
    params_interim["tyre_model_ekf"] = dict(params.get("tyre_model_ekf", {}))
    params_interim["tyre_model_ekf"]["_auto_interim"] = interim_cfg
    interim_result = estimate_sideslip_ekf_dugoff(state, params_interim, pass_id="_auto_interim")
    yaw_innov_interim = interim_result["innovation"][base_mask][:, 0]
    r_yaw_rate_var_derived = float(np.mean(yaw_innov_interim ** 2))

    manifest["r_derivation"] = {
        "std_front_mps2": std_f, "std_rear_mps2": std_r, "inter_axle_correlation": rho,
        "r_ay_var_derived": r_ay_var_derived, "r_yaw_rate_var_derived": r_yaw_rate_var_derived,
    }

    # --- 2-D NIS-gated sweep, anchored at the just-derived baseline ------
    sweep_results = []
    for r_ay_scale in cfg["r_sweep_ay_scales"]:
        for r_yaw_scale in cfg["r_sweep_yaw_scales"]:
            sweep_cfg = dict(interim_cfg)
            sweep_cfg["R_ay_var"] = r_ay_var_derived * r_ay_scale
            sweep_cfg["R_yaw_rate_var"] = r_yaw_rate_var_derived * r_yaw_scale
            params_sweep = dict(params)
            params_sweep["tyre_model_ekf"] = dict(params.get("tyre_model_ekf", {}))
            params_sweep["tyre_model_ekf"]["_auto_sweep"] = sweep_cfg
            result = estimate_sideslip_ekf_dugoff(state, params_sweep, pass_id="_auto_sweep")
            innovation = result["innovation"][base_mask]
            S_diag = result["S_diag"][base_mask]
            f_yaw = float((innovation[:, 0] ** 2 / S_diag[:, 0] > CHI2_DF1_95).mean())
            f_ay = float((innovation[:, 1] ** 2 / S_diag[:, 1] > CHI2_DF1_95).mean())
            both_in_band = (cfg["nis_band_low"] <= f_yaw <= cfg["nis_band_high"]) and \
                            (cfg["nis_band_low"] <= f_ay <= cfg["nis_band_high"])
            sweep_results.append({"r_ay_scale": r_ay_scale, "r_yaw_scale": r_yaw_scale,
                                   "R_ay_var": sweep_cfg["R_ay_var"], "R_yaw_rate_var": sweep_cfg["R_yaw_rate_var"],
                                   "f_yaw": f_yaw, "f_ay": f_ay, "both_in_band": both_in_band})

    in_band = [r for r in sweep_results if r["both_in_band"]]
    centre = (cfg["nis_band_low"] + cfg["nis_band_high"]) / 2.0
    if in_band:
        chosen = min(in_band, key=lambda r: abs(r["f_yaw"] - centre) + abs(r["f_ay"] - centre))
        sweep_found_in_band = True
    else:
        chosen = min(sweep_results, key=lambda r: abs(r["f_yaw"] - centre) + abs(r["f_ay"] - centre))
        sweep_found_in_band = False
    manifest["r_sweep"] = {"grid_points": sweep_results, "chosen": chosen, "found_in_band": sweep_found_in_band}

    # --- step (d): final EKF run with the chosen R ------------------------
    final_cfg = dict(interim_cfg)
    final_cfg["R_ay_var"] = chosen["R_ay_var"]
    final_cfg["R_yaw_rate_var"] = chosen["R_yaw_rate_var"]
    params_final = dict(params)
    params_final["tyre_model_ekf"] = dict(params.get("tyre_model_ekf", {}))
    params_final["tyre_model_ekf"]["_auto_final"] = final_cfg
    final_result = estimate_sideslip_ekf_dugoff(state, params_final, pass_id="_auto_final")
    beta_ekf = final_result["beta"]
    manifest["final_config"] = final_cfg

    # --- step (e): validation summary -------------------------------------
    innovation = final_result["innovation"][base_mask]
    nis_combined = final_result["nis"][base_mask]
    S_diag = final_result["S_diag"][base_mask]
    # Production consumer note (fresh-session work package, Phase 1): the
    # raw "beta"/"nis" arrays above are for THIS module's own validation
    # figures only. Production callers must use beta_ekf_with_fallback,
    # never beta_ekf -- the raw series keeps diverged-window artifacts by
    # design (see diagnostics/sideslip_ekf_dugoff.py's own header), the
    # same "never feed a silently-diverged state downstream" rule the
    # existing ekf_pass_1 production path already follows. base_mask and
    # the full-length nis array are exposed so a caller can run modules.
    # nis_gate.evaluate_gate without re-deriving the masking logic.
    nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
    nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]
    f_yaw = float((nis_yaw > CHI2_DF1_95).mean())
    f_ay = float((nis_ay > CHI2_DF1_95).mean())
    f_comb = float((nis_combined > CHI2_DF2_95).mean())
    manifest["nis"] = {"yaw_rate_exceedance": f_yaw, "ay_exceedance": f_ay,
                        "combined_exceedance": f_comb, "combined_mean_nis": float(np.mean(nis_combined))}

    corners = data.get("corners", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    corners_by_stable_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    racing_ids = [cid for cid, insts in corners_by_stable_id.items() if insts[0].get("speed_class") != "low"]

    t = state["time"]
    s_m = state.get("s_m")
    ay = state["ay_mps2"]
    manifest["sign_check"] = _sign_check(t, s_m, beta_ekf, ay, base_mask, corners_by_stable_id, laps_by_number, racing_ids)

    slip_ekf = estimate_slip_angles(state, beta_ekf, params)
    onset_coverage = {}
    for axle_name, alpha_c, c_a, mu_z in (
        ("front", slip_ekf["alpha_f_filt"], front_fit["c_alpha_n_per_rad"], front_fit["mu_fz_N"]),
        ("rear", slip_ekf["alpha_r_filt"], rear_fit["c_alpha_n_per_rad"], rear_fit["mu_fz_N"]),
    ):
        tan_onset = mu_z / (2.0 * c_a)
        onset_deg = float(np.degrees(np.arctan(tan_onset)))
        alpha_deg = np.degrees(alpha_c)[base_mask]
        frac_beyond = float((np.abs(alpha_deg) > onset_deg).mean())
        onset_coverage[axle_name] = {"onset_deg": onset_deg, "coverage_fraction": frac_beyond}
    manifest["onset_coverage"] = onset_coverage

    apex_half = params["stability_estimation"]["apex_half_window_samples"]
    apex_mask = np.zeros_like(t, dtype=bool)
    for c in corners:
        start_t, end_t = c["segments"]["apex_3"]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi <= lo:
            lo = max(0, lo - apex_half)
            hi = min(len(t), lo + 2 * apex_half + 1)
        apex_mask[lo:hi] = True
    apex_pop = base_mask & apex_mask
    idx = np.where(apex_pop)[0]
    h2_pred = (dugoff_lateral_force(slip_ekf["alpha_f_filt"][idx], front_fit["c_alpha_n_per_rad"], front_fit["mu_fz_N"]) +
               dugoff_lateral_force(slip_ekf["alpha_r_filt"][idx], rear_fit["c_alpha_n_per_rad"], rear_fit["mu_fz_N"])) / mass_kg
    r_h2 = float(np.corrcoef(h2_pred, ay[idx])[0, 1]) if len(idx) > 2 else float("nan")
    manifest["h2_vs_ay_apex"] = {"n": int(len(idx)), "correlation": r_h2}

    # --- status classification ---------------------------------------------
    gate_frac = manifest["sign_check"]["median_gate_racing_fraction"]
    worst_channel_exceedance = max(f_yaw, f_ay)
    if gate_frac < cfg["sign_check_degenerate_fraction"] or worst_channel_exceedance > cfg["nis_gross_miscalibration_fraction"]:
        manifest["status"] = "degenerate"
    elif (not sweep_found_in_band) or gate_frac < cfg["sign_check_marginal_fraction"]:
        manifest["status"] = "marginal"
    else:
        manifest["status"] = "ok"

    manifest["beta_ekf"] = beta_ekf  # not JSON-serialisable directly -- caller's responsibility to strip/summarise
    manifest["beta_ekf_with_fallback"] = final_result["beta_with_fallback"]  # production must use this, not beta_ekf
    manifest["nis_full"] = final_result["nis"]  # full-length, for modules.nis_gate.evaluate_gate
    manifest["base_mask"] = base_mask
    return manifest


def _fit_axle_pacejka(alpha, Fy, base_mask):
    """Phase 3 variant of step (a)+(b): joint 4-parameter (B,C,D,E) fit
    via Powell (chair's own starting guess), same base_mask population
    as the Dugoff chain. No explicit search bracket to hit -- Powell's
    own convergence flag is the closest analogue to Dugoff's bound-
    fraction check, but per the Phase 3 pre-registration this is NOT
    expected to be a reliable degeneracy signal for this axle-fit
    identifiability question (see thesis_notes.md).
    """
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]

    if len(a2) == 0:
        # Empty population -- e.g. every lap failed is_valid_for_analysis
        # (a real v3 case, thesis_notes.md "v3 IndexError: empty fit
        # population"). np.percentile below has no defined behaviour on an
        # empty array and raises IndexError deep inside numpy's quantile
        # internals rather than a clean error -- must never reach it.
        # Matches _fit_axle's own established no-signal convention
        # (early return, sign_ok=False) instead of feeding Powell/
        # percentile an empty array; fit_session_pacejka's own degeneracy
        # check (sign_ok and powell_converged on both axles) already turns
        # this into a clean status="degenerate" manifest, so no separate
        # check is needed at that caller.
        return {
            "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
            "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
            "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
            "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }

    def _sse(p):
        B, C, D, E = p
        pred = pacejka_lateral_force(a2, B, C, D, E)
        return float(np.sum((pred - f2) ** 2))

    res = minimize(_sse, PACEJKA_START_GUESS, method="Powell")
    B, C, D, E = (float(x) for x in res.x)
    pred = pacejka_lateral_force(a2, B, C, D, E)
    resid = f2 - pred
    rms = float(np.sqrt(np.mean(resid ** 2)))

    # Peak location: dense grid search over the visited alpha range
    # extended to +/-90 deg (the model's own domain), refined by
    # bisection on the analytic stiffness's sign change nearest the
    # data. A peak outside the visited range is reported as such, not
    # silently clamped -- exactly the extrapolation risk the Phase 3
    # pre-registration is testing for.
    grid_deg = np.linspace(0.01, 89.9, 4000)
    grid_rad = np.radians(grid_deg)
    stiffness_grid = pacejka_lateral_stiffness(grid_rad, B, C, D, E)
    sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
    if len(sign_changes) > 0:
        i = sign_changes[0]
        peak_deg = float(grid_deg[i])
        peak_in_visited_range = peak_deg <= float(np.percentile(np.abs(np.degrees(a2)), 99))
    else:
        peak_deg = float("nan")
        peak_in_visited_range = False

    return {
        "B": B, "C": C, "D": D, "E": E,
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": rms,
        "peak_alpha_deg": peak_deg, "peak_in_visited_range": peak_in_visited_range,
        "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a2)), 99)),
        "sign_ok": D > 0,
        "residuals": resid, "residual_mask": m2,
    }


def _fit_axle_pacejka_mu(alpha, Fy, Fz, base_mask):
    """Fz-integration Phase 2: load-normalised variant of _fit_axle_
    pacejka -- fits (B, C, mu, E) instead of (B, C, D, E), with the peak
    term evaluated per-sample as D = mu * Fz inside the objective (Fz the
    measured per-axle load, same base_mask population as the free-D fit).
    # method: thesis_notes.md, "Pacejka load-normalised (mu) tyre fit"

    Returns the same dict shape as _fit_axle_pacejka (D holds a
    REPRESENTATIVE value, mu * mean(Fz) over the fit population, so every
    downstream consumer that already treats D as one axle-wide constant --
    the EKF Jacobian config, onset_coverage, h2_vs_ay_apex -- needs no
    change), plus two extra keys: "mu" (the fitted peak friction
    coefficient itself) and "mean_axle_fz_N" (the Fz this D was evaluated
    at, so a caller can reconstruct D at any other Fz if needed).
    """
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz)
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]

    if len(a2) == 0:
        # Same empty-population guard as _fit_axle_pacejka -- never let an
        # empty array reach Powell/percentile.
        return {
            "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
            "mu": float("nan"), "mean_axle_fz_N": float("nan"),
            "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
            "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
            "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }

    mean_fz = float(np.mean(z2))
    # Data-derived starting mu: the chair's own free-D starting guess
    # (PACEJKA_START_GUESS[2] = 8000N) divided by this axle's own mean Fz
    # -- keeps the same starting PEAK FORCE the free-D fit starts from,
    # expressed as a friction coefficient, rather than an arbitrarily
    # chosen mu constant.
    mu_start = PACEJKA_START_GUESS[2] / mean_fz if mean_fz else 1.5

    def _sse(p):
        B, C, mu, E = p
        pred = pacejka_lateral_force(a2, B, C, mu * z2, E)
        return float(np.sum((pred - f2) ** 2))

    start = (PACEJKA_START_GUESS[0], PACEJKA_START_GUESS[1], mu_start, PACEJKA_START_GUESS[3])
    res = minimize(_sse, start, method="Powell")
    B, C, mu, E = (float(x) for x in res.x)
    pred = pacejka_lateral_force(a2, B, C, mu * z2, E)
    resid = f2 - pred
    rms = float(np.sqrt(np.mean(resid ** 2)))

    D_repr = mu * mean_fz
    grid_deg = np.linspace(0.01, 89.9, 4000)
    grid_rad = np.radians(grid_deg)
    stiffness_grid = pacejka_lateral_stiffness(grid_rad, B, C, D_repr, E)
    sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
    if len(sign_changes) > 0:
        i = sign_changes[0]
        peak_deg = float(grid_deg[i])
        peak_in_visited_range = peak_deg <= float(np.percentile(np.abs(np.degrees(a2)), 99))
    else:
        peak_deg = float("nan")
        peak_in_visited_range = False

    return {
        "B": B, "C": C, "D": D_repr, "E": E, "mu": mu, "mean_axle_fz_N": mean_fz,
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": rms,
        "peak_alpha_deg": peak_deg, "peak_in_visited_range": peak_in_visited_range,
        "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a2)), 99)),
        "sign_ok": mu > 0,
        "residuals": resid, "residual_mask": m2,
    }


def fit_session_pacejka(data, params, data_file_path=None, load_normalised=False):
    """Phase 3: same one-shot chain as fit_session, fitting the reduced
    4-parameter Magic Formula (modules/tyre_model_pacejka.py) instead
    of Dugoff, and running the EKF with Pacejka's analytic stiffness in
    the Jacobians (diagnostics/sideslip_ekf_pacejka.py -- a separate
    code path, Dugoff's own EKF file is untouched). Structure mirrors
    fit_session's steps (c)-(e) exactly (R derivation, 2-D sweep,
    validation); only the per-axle fit (step a/b) and the EKF call
    differ. See fit_session's own docstring for the shared status-
    threshold design (identical thresholds, config tyre_fit_auto.*).

    Fz-integration Phase 2 (2026-09-03): load_normalised=True switches
    the per-axle fit from a free peak FORCE D to D = mu * Fz (Fz the
    measured per-axle load, mu the fitted peak friction coefficient --
    method: thesis_notes.md, "Pacejka load-normalised (mu) tyre fit").
    Requires measured Fz (modules.wheel_loads via stability_estimation.
    vertical_load_source="measured"); returns a degenerate manifest if
    unavailable (no damper channels this session, or car_data.json
    missing) rather than silently falling back to free-D. Default False
    reproduces the exact free-D behaviour, byte-identical -- steps
    (c)-(e) below are UNCHANGED either way, since _fit_axle_pacejka_mu
    returns the same dict shape (D holds a representative mu*mean(Fz)
    value for those steps' own constant-D usage).
    """
    cfg = params["tyre_fit_auto"]
    vp = params["vehicle"]
    mass_kg = vp["mass_kg"]

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        return {"status": "degenerate", "degenerate_reason": "prepare_vehicle_state returned None "
                "-- required channels missing from this session", "data_file": data_file_path}

    laps = data.get("laps", [])
    base_mask = _base_mask(state, laps)

    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)

    if load_normalised:
        car_data = load_car_data()
        if car_data is None:
            return {"status": "degenerate", "degenerate_reason": "load_normalised=True requires "
                    "car_data.json, not available", "data_file": data_file_path}
        # Force "measured" for THIS call regardless of the live config's own
        # stability_estimation.vertical_load_source -- load_normalised=True
        # is an explicit request for measured Fz, not a reflection of the
        # global flag (which stays "static" by default everywhere else in
        # production; without this override the mu fit would silently
        # degenerate to "static resolved" and never run, exactly the bug
        # this comment replaced after finding it empirically).
        params_measured_fz = dict(params)
        params_measured_fz["stability_estimation"] = dict(params["stability_estimation"])
        params_measured_fz["stability_estimation"]["vertical_load_source"] = "measured"
        fz = estimate_vertical_loads(state, forces, params_measured_fz,
                                      channels=data["channels"], car_data=car_data)
        if fz.get("vertical_load_source_used") != "measured":
            return {"status": "degenerate", "degenerate_reason": "load_normalised=True requires measured "
                    "Fz -- no damper-valid samples this session (vertical_load_source resolved to 'static')",
                    "data_file": data_file_path}
        front_fit = _fit_axle_pacejka_mu(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], fz["fz_f_N"], base_mask)
        rear_fit = _fit_axle_pacejka_mu(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], fz["fz_r_N"], base_mask)
    else:
        front_fit = _fit_axle_pacejka(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], base_mask)
        rear_fit = _fit_axle_pacejka(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], base_mask)

    manifest = {
        "data_file": data_file_path,
        "load_normalised": load_normalised,
        "laps_used": sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis")),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "axles": {"front": {k: v for k, v in front_fit.items() if k not in ("residuals", "residual_mask")},
                  "rear": {k: v for k, v in rear_fit.items() if k not in ("residuals", "residual_mask")}},
    }
    try:
        git_hash = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    except Exception as e:
        git_hash = f"UNAVAILABLE ({e})"
    manifest["git_commit_hash"] = git_hash

    if not (front_fit["sign_ok"] and front_fit["powell_converged"] and
            rear_fit["sign_ok"] and rear_fit["powell_converged"]):
        manifest["status"] = "degenerate"
        manifest["degenerate_reason"] = "Powell did not converge or D<=0 (sign check failed) on at least one axle"
        return manifest

    common_mask = front_fit["residual_mask"] & rear_fit["residual_mask"]
    resid_front_full = np.zeros_like(base_mask, dtype=float)
    resid_rear_full = np.zeros_like(base_mask, dtype=float)
    resid_front_full[front_fit["residual_mask"]] = front_fit["residuals"]
    resid_rear_full[rear_fit["residual_mask"]] = rear_fit["residuals"]
    r_ay_var_derived, std_f, std_r, rho = _r_from_residuals(resid_front_full, resid_rear_full, common_mask, mass_kg)

    interim_cfg = {
        "b_front": front_fit["B"], "c_front": front_fit["C"], "d_front": front_fit["D"], "e_front": front_fit["E"],
        "b_rear": rear_fit["B"], "c_rear": rear_fit["C"], "d_rear": rear_fit["D"], "e_rear": rear_fit["E"],
        "Q_beta_var": cfg["q_beta_var"], "Q_yaw_rate_var": cfg["q_yaw_rate_var"],
        "P0_beta_var": cfg["p0_beta_var"], "P0_yaw_rate_var": cfg["p0_yaw_rate_var"],
        "R_ay_var": r_ay_var_derived, "R_yaw_rate_var": cfg["r_yaw_rate_interim_seed_var"],
        "beta_hard_bound_deg": cfg["beta_hard_bound_deg"],
        "nis_window_samples": cfg["nis_window_samples"], "nis_chi2_bound": cfg["nis_chi2_bound"],
        "nis_flag_fraction": cfg["nis_flag_fraction"],
    }
    params_interim = dict(params)
    params_interim["tyre_model_ekf_pacejka"] = {"_auto_interim": interim_cfg}
    interim_result = estimate_sideslip_ekf_pacejka(state, params_interim, pass_id="_auto_interim")
    yaw_innov_interim = interim_result["innovation"][base_mask][:, 0]
    r_yaw_rate_var_derived = float(np.mean(yaw_innov_interim ** 2))

    manifest["r_derivation"] = {
        "std_front_mps2": std_f, "std_rear_mps2": std_r, "inter_axle_correlation": rho,
        "r_ay_var_derived": r_ay_var_derived, "r_yaw_rate_var_derived": r_yaw_rate_var_derived,
    }

    sweep_results = []
    for r_ay_scale in cfg["r_sweep_ay_scales"]:
        for r_yaw_scale in cfg["r_sweep_yaw_scales"]:
            sweep_cfg = dict(interim_cfg)
            sweep_cfg["R_ay_var"] = r_ay_var_derived * r_ay_scale
            sweep_cfg["R_yaw_rate_var"] = r_yaw_rate_var_derived * r_yaw_scale
            params_sweep = dict(params)
            params_sweep["tyre_model_ekf_pacejka"] = {"_auto_sweep": sweep_cfg}
            result = estimate_sideslip_ekf_pacejka(state, params_sweep, pass_id="_auto_sweep")
            innovation = result["innovation"][base_mask]
            S_diag = result["S_diag"][base_mask]
            f_yaw = float((innovation[:, 0] ** 2 / S_diag[:, 0] > CHI2_DF1_95).mean())
            f_ay = float((innovation[:, 1] ** 2 / S_diag[:, 1] > CHI2_DF1_95).mean())
            both_in_band = (cfg["nis_band_low"] <= f_yaw <= cfg["nis_band_high"]) and \
                            (cfg["nis_band_low"] <= f_ay <= cfg["nis_band_high"])
            sweep_results.append({"r_ay_scale": r_ay_scale, "r_yaw_scale": r_yaw_scale,
                                   "R_ay_var": sweep_cfg["R_ay_var"], "R_yaw_rate_var": sweep_cfg["R_yaw_rate_var"],
                                   "f_yaw": f_yaw, "f_ay": f_ay, "both_in_band": both_in_band})

    in_band = [r for r in sweep_results if r["both_in_band"]]
    centre = (cfg["nis_band_low"] + cfg["nis_band_high"]) / 2.0
    if in_band:
        chosen = min(in_band, key=lambda r: abs(r["f_yaw"] - centre) + abs(r["f_ay"] - centre))
        sweep_found_in_band = True
    else:
        chosen = min(sweep_results, key=lambda r: abs(r["f_yaw"] - centre) + abs(r["f_ay"] - centre))
        sweep_found_in_band = False
    manifest["r_sweep"] = {"grid_points": sweep_results, "chosen": chosen, "found_in_band": sweep_found_in_band}

    final_cfg = dict(interim_cfg)
    final_cfg["R_ay_var"] = chosen["R_ay_var"]
    final_cfg["R_yaw_rate_var"] = chosen["R_yaw_rate_var"]
    params_final = dict(params)
    params_final["tyre_model_ekf_pacejka"] = {"_auto_final": final_cfg}
    final_result = estimate_sideslip_ekf_pacejka(state, params_final, pass_id="_auto_final")
    beta_ekf = final_result["beta"]
    manifest["final_config"] = final_cfg

    innovation = final_result["innovation"][base_mask]
    nis_combined = final_result["nis"][base_mask]
    S_diag = final_result["S_diag"][base_mask]
    nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
    nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]
    f_yaw = float((nis_yaw > CHI2_DF1_95).mean())
    f_ay = float((nis_ay > CHI2_DF1_95).mean())
    f_comb = float((nis_combined > CHI2_DF2_95).mean())
    manifest["nis"] = {"yaw_rate_exceedance": f_yaw, "ay_exceedance": f_ay,
                        "combined_exceedance": f_comb, "combined_mean_nis": float(np.mean(nis_combined))}

    corners = data.get("corners", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    corners_by_stable_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    racing_ids = [cid for cid, insts in corners_by_stable_id.items() if insts[0].get("speed_class") != "low"]

    t = state["time"]
    s_m = state.get("s_m")
    ay = state["ay_mps2"]
    manifest["sign_check"] = _sign_check(t, s_m, beta_ekf, ay, base_mask, corners_by_stable_id, laps_by_number, racing_ids)

    slip_ekf = estimate_slip_angles(state, beta_ekf, params)
    coverage = {}
    for axle_name, alpha_c, fit in (("front", slip_ekf["alpha_f_filt"], front_fit), ("rear", slip_ekf["alpha_r_filt"], rear_fit)):
        alpha_deg = np.degrees(alpha_c)[base_mask]
        if np.isfinite(fit["peak_alpha_deg"]):
            frac_beyond = float((np.abs(alpha_deg) > fit["peak_alpha_deg"]).mean())
        else:
            frac_beyond = float("nan")
        coverage[axle_name] = {"peak_alpha_deg": fit["peak_alpha_deg"], "coverage_fraction": frac_beyond}
    manifest["onset_coverage"] = coverage  # field name kept consistent with fit_session's for direct comparison

    apex_half = params["stability_estimation"]["apex_half_window_samples"]
    apex_mask = np.zeros_like(t, dtype=bool)
    for c in corners:
        start_t, end_t = c["segments"]["apex_3"]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi <= lo:
            lo = max(0, lo - apex_half)
            hi = min(len(t), lo + 2 * apex_half + 1)
        apex_mask[lo:hi] = True
    apex_pop = base_mask & apex_mask
    idx = np.where(apex_pop)[0]
    h2_pred = (pacejka_lateral_force(slip_ekf["alpha_f_filt"][idx], front_fit["B"], front_fit["C"], front_fit["D"], front_fit["E"]) +
               pacejka_lateral_force(slip_ekf["alpha_r_filt"][idx], rear_fit["B"], rear_fit["C"], rear_fit["D"], rear_fit["E"])) / mass_kg
    r_h2 = float(np.corrcoef(h2_pred, ay[idx])[0, 1]) if len(idx) > 2 else float("nan")
    manifest["h2_vs_ay_apex"] = {"n": int(len(idx)), "correlation": r_h2}

    gate_frac = manifest["sign_check"]["median_gate_racing_fraction"]
    worst_channel_exceedance = max(f_yaw, f_ay)
    if gate_frac < cfg["sign_check_degenerate_fraction"] or worst_channel_exceedance > cfg["nis_gross_miscalibration_fraction"]:
        manifest["status"] = "degenerate"
    elif (not sweep_found_in_band) or gate_frac < cfg["sign_check_marginal_fraction"]:
        manifest["status"] = "marginal"
    else:
        manifest["status"] = "ok"

    if load_normalised:
        # mu plausibility (config tyre_fit_auto.mu_plausibility_band_low/
        # high) -- reported, never silently accepted or discarded (this
        # project's standing rule on an implausible Tier-A numeric result).
        mu_lo, mu_hi = cfg["mu_plausibility_band_low"], cfg["mu_plausibility_band_high"]
        manifest["mu_plausibility"] = {
            "mu_front": front_fit["mu"], "mu_rear": rear_fit["mu"],
            "band_low": mu_lo, "band_high": mu_hi,
            "front_plausible": bool(mu_lo <= front_fit["mu"] <= mu_hi),
            "rear_plausible": bool(mu_lo <= rear_fit["mu"] <= mu_hi),
        }

    manifest["beta_ekf"] = beta_ekf
    manifest["beta_ekf_with_fallback"] = final_result["beta_with_fallback"]  # production must use this, not beta_ekf
    manifest["nis_full"] = final_result["nis"]  # full-length, for modules.nis_gate.evaluate_gate
    manifest["base_mask"] = base_mask
    return manifest


def resolve_sideslip_beta(state, params, data, sideslip_source, csv_path=None):
    """Fresh-session work package, Phase 1: single source of truth for
    "which beta does this sideslip_source actually produce", used by
    ui/views/outing_form.py's StabilityAnalysisThread. Extracted into
    modules/ (not left inline in the QThread) so it is directly
    testable without any Qt dependency -- tests/test_auto_fit_wiring.py
    calls this exact function, not a reimplementation, matching this
    project's "no business logic in ui/" rule slightly more strictly
    than the pre-existing ekf_pass_1 branch did.

    Returns (beta, fit_manifest, gate_verdict, fallback_used,
    fallback_reason). fit_manifest is the JSON-safe subset of fit_
    session's/fit_session_pacejka's manifest (numpy-array keys
    stripped) -- None for every sideslip_source except the two auto
    modes. gate_verdict is modules.nis_gate.evaluate_gate's return
    dict, None outside the auto modes or when the fit itself degenerated
    (the gate never runs against a curve already known unusable).
    fallback_used/fallback_reason are False/None unless an auto mode's
    fit degenerated or its gate verdicted 'fail' -- in either case beta
    falls back to kinematic (estimate_sideslip), the reason is recorded
    as text, never silent. Never mutates params.
    """
    if sideslip_source == "ekf_pass_1":
        # beta_with_fallback, never the raw pre-fallback series: raw keeps
        # diverged-window artifacts for diagnostics (see diagnostics/
        # sideslip_ekf_dugoff.py's own docstring) -- production must never
        # feed a silently-diverged state into the rest of the pipeline.
        ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
        return ekf_result["beta_with_fallback"], None, None, False, None

    if sideslip_source in ("ekf_auto_dugoff", "ekf_auto_pacejka"):
        if sideslip_source == "ekf_auto_dugoff":
            raw_fit_manifest = fit_session(data, params, data_file_path=csv_path)
        else:
            # Fz-integration Phase 2: config-gated, default False -- see
            # fit_session_pacejka's own docstring. fit_session (Dugoff) has
            # no load_normalised mode; this phase only touches the Pacejka
            # path, per the work order.
            load_normalised = params.get("tyre_fit_auto", {}).get("load_normalised_fit_enabled", False)
            raw_fit_manifest = fit_session_pacejka(data, params, data_file_path=csv_path,
                                                    load_normalised=load_normalised)
        fit_status = raw_fit_manifest.get("status")
        fallback_used = False
        fallback_reason = None
        gate_verdict = None
        if fit_status == "degenerate":
            fallback_used = True
            fallback_reason = f"fit status 'degenerate': {raw_fit_manifest.get('degenerate_reason')}"
            beta = estimate_sideslip(state, params)
        else:
            gate_verdict = evaluate_gate(
                raw_fit_manifest["nis_full"], raw_fit_manifest["base_mask"], params, state["sample_rate_hz"])
            if gate_verdict["verdict"] == "fail":
                fallback_used = True
                fallback_reason = (
                    f"NIS gate verdict 'fail' (health_score={gate_verdict['health_score']!r}, "
                    f"threshold_warn={gate_verdict['threshold_warn']})"
                )
                beta = estimate_sideslip(state, params)
            else:
                beta = raw_fit_manifest["beta_ekf_with_fallback"]
        fit_manifest = {
            k: v for k, v in raw_fit_manifest.items()
            if k not in ("beta_ekf", "beta_ekf_with_fallback", "nis_full", "base_mask")
        }
        return beta, fit_manifest, gate_verdict, fallback_used, fallback_reason

    return estimate_sideslip(state, params), None, None, False, None
