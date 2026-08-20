# PLAN.md unsupervised package, Phase 4: NIS tyre-mismatch health-score
# prototype. Read-only, nothing wired -- diagnostic only. Design and
# pre-registered prediction: thesis_notes.md "Phase 4: NIS tyre-
# mismatch gate -- pre-registration". Reuses the EKF's own windowed-NIS
# machinery (config nis_window_samples, the same 3-15% acceptance band
# Phase 2's R sweep is gated on) as a continuous session-level score
# rather than inventing a new statistic.

import copy

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.tyre_fit_auto import fit_session, _base_mask
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
NIS_BAND_LOW, NIS_BAND_HIGH = 0.03, 0.15
CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
laps = data.get("laps", [])
base_mask = _base_mask(state, laps)
window = params["tyre_fit_auto"]["nis_window_samples"]


def health_score(nis_combined_full, mask):
    """Rolling trailing-window (width `window`) combined-NIS exceedance
    fraction, evaluated at every masked sample; health_score = fraction
    of masked samples whose LOCAL window sits inside [NIS_BAND_LOW,
    NIS_BAND_HIGH]. Full-length nis array in, so the window can look
    back across mask boundaries exactly like the EKF's own monitor
    does (it runs over the full session, mask applied only when
    reporting) -- avoids an artificial reset at every masked-out gap.
    """
    exceed = (nis_combined_full > CHI2_DF2_95).astype(float)
    n = len(exceed)
    cw = np.cumsum(np.insert(exceed, 0, 0.0))
    win_frac = np.full(n, np.nan)
    for i in range(window - 1, n):
        win_frac[i] = (cw[i + 1] - cw[i + 1 - window]) / window
    in_band = (win_frac >= NIS_BAND_LOW) & (win_frac <= NIS_BAND_HIGH)
    valid = mask & np.isfinite(win_frac)
    return float(in_band[valid].mean()), win_frac


print("=" * 100)
print("Phase 4 -- NIS tyre-mismatch health-score prototype")
print("=" * 100)
print(f"window={window} samples   band=[{NIS_BAND_LOW}, {NIS_BAND_HIGH}]   "
      f"chi2_df2_95={CHI2_DF2_95:.4f}   masked population n={int(base_mask.sum())}")
print()

healthy_fit = fit_session(data, params, data_file_path=RAW_FILE)
healthy_fit.pop("beta_ekf", None)
healthy_cfg = healthy_fit["final_config"]
print(f"healthy baseline status: {healthy_fit['status']}   "
      f"c_alpha_f/r={healthy_cfg['c_alpha_front_n_per_rad']:.0f}/{healthy_cfg['c_alpha_rear_n_per_rad']:.0f}  "
      f"mu_fz_f/r={healthy_cfg['mu_fz_front_N']:.0f}/{healthy_cfg['mu_fz_rear_N']:.0f}")
print()


def run_and_score(cfg, label):
    params_run = dict(params)
    params_run["tyre_model_ekf"] = dict(params.get("tyre_model_ekf", {}))
    params_run["tyre_model_ekf"]["_mismatch_probe"] = cfg
    result = estimate_sideslip_ekf_dugoff(state, params_run, pass_id="_mismatch_probe")
    score, _ = health_score(result["nis"], base_mask)
    print(f"  {label:20s} health_score={score:.4f}")
    return score


scenarios = {
    "healthy": healthy_cfg,
}
for param_name, config_keys in (
    ("c_alpha", ("c_alpha_front_n_per_rad", "c_alpha_rear_n_per_rad")),
    ("mu_fz", ("mu_fz_front_N", "mu_fz_rear_N")),
):
    for scale in (0.5, 2.0):
        cfg = copy.deepcopy(healthy_cfg)
        for k in config_keys:
            cfg[k] = cfg[k] * scale
        scenarios[f"{param_name}_x{scale}"] = cfg

scores = {}
for label, cfg in scenarios.items():
    scores[label] = run_and_score(cfg, label)
print()

healthy_score = scores["healthy"]
mismatch_scores = {k: v for k, v in scores.items() if k != "healthy"}
worst_mismatch = min(mismatch_scores.values())
best_mismatch = max(mismatch_scores.values())

print("=" * 100)
print("SEPARATION CHECK")
print("=" * 100)
print(f"  healthy={healthy_score:.4f}   mismatch range=[{worst_mismatch:.4f}, {best_mismatch:.4f}]")
all_lower = all(s < healthy_score for s in mismatch_scores.values())
print(f"  ALL mismatch scenarios score lower than healthy: {all_lower}")

c_alpha_scores = [scores[f"c_alpha_x{s}"] for s in (0.5, 2.0)]
mu_fz_scores = [scores[f"mu_fz_x{s}"] for s in (0.5, 2.0)]
c_alpha_worse = max(c_alpha_scores) < min(mu_fz_scores) if c_alpha_scores and mu_fz_scores else None
print(f"  c_alpha mismatches score worse than mu_fz mismatches (pre-registered): "
      f"c_alpha={c_alpha_scores}  mu_fz={mu_fz_scores}  -> {'CONFIRMED' if c_alpha_worse else 'NOT CONFIRMED'}")
print()

print("=" * 100)
print("PROPOSED THRESHOLDS (gap-selected between healthy and worst mismatch -- NOT applied anywhere)")
print("=" * 100)
gap_lo, gap_hi = worst_mismatch, healthy_score
if gap_hi > gap_lo:
    t_use = gap_lo + 0.75 * (gap_hi - gap_lo)
    t_warn = gap_lo + 0.35 * (gap_hi - gap_lo)
    print(f"  gap: [{gap_lo:.4f}, {gap_hi:.4f}]")
    print(f"  USE_EKF if health_score >= {t_use:.4f}")
    print(f"  WARN if {t_warn:.4f} <= health_score < {t_use:.4f}")
    print(f"  FALL_BACK_TO_KINEMATIC if health_score < {t_warn:.4f}")
else:
    print("  NO GAP -- healthy and mismatch scores overlap, gap-selection not applicable on this evidence. "
          "Record as a limitation, not force a threshold.")
