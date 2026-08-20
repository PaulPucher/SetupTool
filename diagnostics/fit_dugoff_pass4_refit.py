# WP-N2 pass 4: Dugoff curve refit from the EKF's OWN slip angles
# (pass_3), continuing the kinematic-seeded-fit -> observer -> refit
# loop. Structurally identical to fit_dugoff_pass3_refit.py -- only
# EKF_SOURCE_PASS_ID and the manifest path differ -- same reproducibility
# reasoning as every prior pass in this sequence.
#
# This is the discriminating pass for the front axle's oscillation
# (thesis_notes.md WP-N2 pass 4 pre-registration): front c_alpha/mu_fz
# flipped sign at pass 3 with sharply shrinking magnitude; pass 4 tests
# damping (flip again, magnitude shrinks further) vs failure mode 1
# (flip again, magnitude does not shrink) vs an ambiguous middle.

import json
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import minimize_scalar

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_vertical_loads, estimate_cornering_stiffness,
)
from modules.tyre_model import dugoff_lateral_force
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MANIFEST_PATH = "diagnostics/fit_dugoff_pass4_refit_manifest.json"
EKF_SOURCE_PASS_ID = "pass_3"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id=EKF_SOURCE_PASS_ID)
beta_ekf = ekf_result["beta"]
slip = estimate_slip_angles(state, beta_ekf, params)
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
print(f"WP-N2 pass 4 -- Dugoff refit, slip angles from EKF {EKF_SOURCE_PASS_ID}")
print("=" * 100)
print(f"file: {RAW_FILE}")
print(f"laps used: {valid_lap_numbers}")
print(f"total masked samples: {int(base_mask.sum())}")
print()

manifest = {
    "model_form": "Dugoff pure-lateral (Rajamani Ch. 13.10; pipeline Fy-vs-alpha sign "
                   "convention, see modules/tyre_model.py header)",
    "data_file": RAW_FILE,
    "laps_used": valid_lap_numbers,
    "slip_source": f"ekf_{EKF_SOURCE_PASS_ID}",
    "ekf_source_pass_id": EKF_SOURCE_PASS_ID,
    "c_alpha_source": "module_4b median over CS_ratio==1.0 linear-regime samples, "
                       "estimate_cornering_stiffness RE-RUN on the EKF's OWN alpha/Fy "
                       "(not the kinematic path's CS_ratio flag) -- WP-N2 pass 4",
    "fz_accuracy_level": "Level 1, Cl=0 -- effective mu conditional on this",
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "sample_counts": {
        "total_masked": int(base_mask.sum()),
    },
    "axles": {},
}

for axle_name, d in axles.items():
    alpha_full = d["alpha"]
    Fy_full = d["Fy"]
    Fz_full = d["Fz"]
    C_alpha_full = d["C_alpha"]
    CS_ratio_full = d["CS_ratio"]

    m_c4b = base_mask & np.isfinite(C_alpha_full) & (CS_ratio_full == 1.0)
    c_alpha_pop = C_alpha_full[m_c4b]
    c_alpha_used = float(np.median(c_alpha_pop))
    sign_ok = c_alpha_used > 0

    m2 = base_mask & np.isfinite(alpha_full) & np.isfinite(Fy_full) & np.isfinite(Fz_full)
    a2, f2, z2 = alpha_full[m2], Fy_full[m2], Fz_full[m2]

    def _sse(mu_fz, a=a2, f=f2, c=c_alpha_used):
        pred = dugoff_lateral_force(a, c, mu_fz)
        return float(np.sum((pred - f) ** 2))

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
    print(f"  c_alpha used = {c_alpha_used:.1f} N/rad  (Module 4b median on EKF-alpha CS_ratio, "
          f"mask=base_mask & isfinite(C_alpha) & CS_ratio(ekf)==1.0, n={len(c_alpha_pop)})  "
          f"sign check: {'OK (positive)' if sign_ok else 'FAILED -- negative, convention mismatch'}")
    print(f"  mu_fz = {mu_fz:.1f} N  (ceiling fit, n={len(a2)}, RMS resid={rms2:.1f} N, "
          f"final search bracket [1, {hi_bound:.1f}] N after {attempts} attempt(s), "
          f"at {mu_fz_bound_fraction*100:.1f}% of upper bound"
          f"{' -- STILL HIT BOUND, not an interior optimum' if mu_fz_bound_fraction > 0.95 else ' -- interior optimum'})")
    print(f"  mean axle Fz over fit samples = {mean_fz:.1f} N  ->  effective mu = {effective_mu:.4f}")
    print(f"  visited EKF slip range: min={slip_range['min_deg']:.3f} deg  "
          f"p1={slip_range['p1_deg']:.3f} deg  p99={slip_range['p99_deg']:.3f} deg  "
          f"max={slip_range['max_deg']:.3f} deg")
    print()

    manifest["axles"][axle_name] = {
        "c_alpha_n_per_rad": c_alpha_used,
        "c_alpha_source_mask_n": int(len(c_alpha_pop)),
        "c_alpha_sign_check_ok": sign_ok,
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
