# WP-N1 Part B (WP-N1b revision): Dugoff first-pass parameter fit.
# Read-only w.r.t. production (modules/tyre_model.py is the only new
# modules/ file; the one config addition is tyre_model_fit.
# ay_linear_threshold_g). Tier B calibration/fit exercise -- this is a
# fitting procedure, not a validated tyre model for this car; see the
# manifest's fz_accuracy_level note.
#
# WP-N1b change (thesis_notes.md "WP-N1" entry, superseded-paragraph
# note): the original c_alpha source (a raw OLS slope of Fy vs alpha on
# the low-|ay| population, through the origin) was found to be an
# errors-in-variables attenuation artifact -- near zero slip, both alpha
# and Fy are small and noise-dominated (alpha/Fy correlation r=0.37
# front, r=0.13 rear at that population), and a single unweighted OLS
# pass has none of Module 4b's own safeguards against exactly this
# (minimum window span, R^2-weighted section blending). c_alpha now
# comes from Module 4b's own per-sample effective stiffness
# (estimate_cornering_stiffness) instead -- the same estimator already
# trusted throughout the rest of this pipeline. The raw OLS value is
# still computed and printed for comparison only, explicitly labelled
# not used.
#
# Two-step fit per axle:
#   1. c_alpha: median of Module 4b's C_alpha_f/C_alpha_r over this
#      script's base mask (valid-lap, moving, kerb-excluded) intersected
#      with Module 4b's OWN linear-regime indicator, CS_ratio == 1.0
#      (CS_ratio = min(C_alpha/C_linear_ref, 1.0) -- exactly 1.0 marks a
#      window whose stiffness is at or above the currently-known linear
#      reference, i.e. not detected as reduced from the linear regime;
#      this is the module's own operational definition, not a
#      new criterion invented for this script).
#   2. mu_fz: 1-D nonlinear least squares on the full Dugoff form
#      (modules/tyre_model.dugoff_lateral_force), c_alpha fixed from step
#      1, using every masked sample -- this is where the curve's
#      saturation shape actually gets visited, not just the linear subset.
#
# Slip source: kinematic (estimate_sideslip/estimate_slip_angles), the
# production sideslip estimate -- deliberately NOT the linear Kalman
# observer candidate, which is rejected for exactly this kind of use
# (PLAN.md STATUS, thesis_notes.md "Linear observer saturation-detection
# failure"): an estimator built on a linear tyre prior cannot supply slip
# angles to identify a saturating tyre curve without circularity.

import json
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import minimize_scalar

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_vertical_loads,
    estimate_cornering_stiffness,
)
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MANIFEST_PATH = "diagnostics/fit_dugoff_first_pass_manifest.json"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
fz = estimate_vertical_loads(state, forces, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)

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

ay_threshold_g = params["tyre_model_fit"]["ay_linear_threshold_g"]
ay_g = state["ay_mps2"] / 9.81
ols_linear_mask = base_mask & (np.abs(ay_g) < ay_threshold_g)  # old (superseded) c_alpha source, comparison only

axles = {
    "front": {
        "alpha": slip["alpha_f_filt"], "Fy": forces["Fy_f_filt"], "Fz": fz["fz_f_N"],
        "C_alpha": cs["C_alpha_f"], "CS_ratio": cs["CS_ratio_f"],
    },
    "rear": {
        "alpha": slip["alpha_r_filt"], "Fy": forces["Fy_r_filt"], "Fz": fz["fz_r_N"],
        "C_alpha": cs["C_alpha_r"], "CS_ratio": cs["CS_ratio_r"],
    },
}

valid_lap_numbers = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis"))

print("=" * 100)
print("WP-N1b -- Dugoff first-pass fit, c_alpha sourced from Module 4b")
print("=" * 100)
print(f"file: {RAW_FILE}")
print(f"laps used: {valid_lap_numbers}")
print(f"total masked samples: {int(base_mask.sum())}   "
      f"old OLS linear-regime samples (|ay| < {ay_threshold_g} g, comparison only): "
      f"{int(ols_linear_mask.sum())}")
print()
print("UNIT/SIGN CHECK: Module 4b's C_alpha (modules/stability_analysis.py "
      "estimate_cornering_stiffness) is itself the windowed regression slope of "
      "Fy_filt vs alpha_filt -- the exact same arrays already read elsewhere in "
      "this script (forces['Fy_*_filt'], slip['alpha_*_filt']), same units (N/rad). "
      "modules/tyre_model.py's Fy=c_alpha*tan(alpha)*f(lambda) reduces to "
      "Fy=c_alpha*tan(alpha) (~c_alpha*alpha for small alpha) in the unsaturated "
      "(lambda>=1) limit -- same definition, same positive sign convention "
      "(confirmed empirically in the WP-N1 entry, thesis_notes.md).")
print()

manifest = {
    "model_form": "Dugoff pure-lateral (Rajamani Ch. 13.10; pipeline Fy-vs-alpha sign "
                   "convention, see modules/tyre_model.py header)",
    "data_file": RAW_FILE,
    "laps_used": valid_lap_numbers,
    "ay_linear_threshold_g": ay_threshold_g,
    "slip_source": "kinematic",
    "c_alpha_source": "module_4b median over CS_ratio==1.0 linear-regime samples "
                       "(WP-N1b; supersedes WP-N1's raw low-ay OLS slope)",
    "fz_accuracy_level": "Level 1, Cl=0 -- effective mu conditional on this",
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "sample_counts": {
        "total_masked": int(base_mask.sum()),
        "ols_linear_regime_comparison_only": int(ols_linear_mask.sum()),
    },
    "axles": {},
}

for axle_name, d in axles.items():
    alpha_full = d["alpha"]
    Fy_full = d["Fy"]
    Fz_full = d["Fz"]
    C_alpha_full = d["C_alpha"]
    CS_ratio_full = d["CS_ratio"]

    # --- superseded OLS source, comparison only ---
    m_ols = ols_linear_mask & np.isfinite(alpha_full) & np.isfinite(Fy_full)
    a_ols, f_ols = alpha_full[m_ols], Fy_full[m_ols]
    c_alpha_ols = float(np.sum(a_ols * f_ols) / np.sum(a_ols ** 2))

    # --- WP-N1b source: Module 4b's own linear-regime samples ---
    m_c4b = base_mask & np.isfinite(C_alpha_full) & (CS_ratio_full == 1.0)
    c_alpha_pop = C_alpha_full[m_c4b]
    c_alpha_used = float(np.median(c_alpha_pop))
    sign_ok = c_alpha_used > 0

    m2 = base_mask & np.isfinite(alpha_full) & np.isfinite(Fy_full) & np.isfinite(Fz_full)
    a2, f2, z2 = alpha_full[m2], Fy_full[m2], Fz_full[m2]

    def _sse(mu_fz, a=a2, f=f2, c=c_alpha_used):
        pred = dugoff_lateral_force(a, c, mu_fz)
        return float(np.sum((pred - f) ** 2))

    # Bracket widens automatically if the optimum lands on it -- a boundary
    # solution means the true optimum (if any, within reach of this data)
    # sits further out, not that this bracket was simply drawn too small.
    hi_bound = 5.0 * float(np.max(np.abs(f2)))
    mu_fz_bound_fraction = 1.0
    attempts = 0
    while mu_fz_bound_fraction > 0.95 and attempts < 4:
        opt = minimize_scalar(_sse, bounds=(1.0, hi_bound), method="bounded")
        mu_fz = float(opt.x)
        mu_fz_bound_fraction = mu_fz / hi_bound
        attempts += 1
        if mu_fz_bound_fraction > 0.95 and attempts < 4:
            hi_bound *= 5.0

    pred2 = dugoff_lateral_force(a2, c_alpha_used, mu_fz)
    rms2 = float(np.sqrt(np.mean((f2 - pred2) ** 2)))

    mean_fz = float(np.mean(z2))
    effective_mu = mu_fz / mean_fz

    alpha_deg = np.degrees(a2)
    p1, p99 = np.percentile(alpha_deg, [1, 99])
    slip_range = {
        "min_deg": float(np.min(alpha_deg)),
        "p1_deg": float(p1),
        "p99_deg": float(p99),
        "max_deg": float(np.max(alpha_deg)),
    }

    print(f"--- {axle_name} ---")
    print(f"  c_alpha used = {c_alpha_used:.1f} N/rad  (Module 4b median, "
          f"mask=base_mask & isfinite(C_alpha) & CS_ratio==1.0, n={len(c_alpha_pop)})  "
          f"sign check: {'OK (positive)' if sign_ok else 'FAILED -- negative, convention mismatch'}")
    print(f"  c_alpha raw OLS (attenuated, not used) = {c_alpha_ols:.1f} N/rad  "
          f"(|ay|<{ay_threshold_g}g population, n={len(a_ols)})")
    print(f"  mu_fz = {mu_fz:.1f} N  (ceiling fit, n={len(a2)}, RMS resid={rms2:.1f} N, "
          f"final search bracket [1, {hi_bound:.1f}] N after {attempts} attempt(s), "
          f"at {mu_fz_bound_fraction*100:.1f}% of upper bound"
          f"{' -- STILL HIT BOUND, not an interior optimum' if mu_fz_bound_fraction > 0.95 else ' -- interior optimum'})")
    print(f"  mean axle Fz over fit samples = {mean_fz:.1f} N  ->  effective mu = {effective_mu:.4f}")
    print(f"  visited kinematic slip range: min={slip_range['min_deg']:.3f} deg  "
          f"p1={slip_range['p1_deg']:.3f} deg  p99={slip_range['p99_deg']:.3f} deg  "
          f"max={slip_range['max_deg']:.3f} deg")
    print()

    manifest["axles"][axle_name] = {
        "c_alpha_n_per_rad": c_alpha_used,
        "c_alpha_source_mask_n": int(len(c_alpha_pop)),
        "c_alpha_sign_check_ok": sign_ok,
        "c_alpha_raw_ols_n_per_rad_not_used": c_alpha_ols,
        "mu_fz_N": mu_fz,
        "mu_fz_search_bracket_N": [1.0, hi_bound],
        "mu_fz_bracket_widen_attempts": attempts,
        "mu_fz_bound_fraction": mu_fz_bound_fraction,
        "mu_fz_fit_n_samples": int(len(a2)),
        "mu_fz_fit_rms_resid_N": rms2,
        "mean_axle_fz_N": mean_fz,
        "effective_mu": effective_mu,
        "visited_slip_range_deg": slip_range,
    }

with open(MANIFEST_PATH, "w") as fh:
    json.dump(manifest, fh, indent=2)

print(f"manifest written: {MANIFEST_PATH}")
