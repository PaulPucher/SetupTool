# PLAN.md unsupervised package, Phase 2: acceptance check for
# modules/tyre_fit_auto.py. Runs the new one-shot fit+validation chain
# on Dubai and compares against the recorded pass-0 fit (diagnostics/
# fit_dugoff_first_pass_manifest.json, WP-N1b) and pass-1's validation
# baseline (diagnostics/pass1_final_validation_manifest.json). Read-
# only, no config/production change.
#
# TOLERANCE, justified: c_alpha/mu_fz go through the identical
# deterministic formula/optimizer (median over an identical mask,
# scipy.optimize.minimize_scalar with an identical bracket-widening
# loop) against the identical Dubai data and config-derived inputs --
# an exact reproduction is expected, so the tolerance here (relative
# 1e-6) only needs to absorb dict-ordering/copy differences in how the
# two scripts build their masks, not any real numerical divergence.
# The R derivation and NIS/sign-check validation figures depend on the
# EKF recursion (also identical code, diagnostics/sideslip_ekf_
# dugoff.py, imported not duplicated) and on the exact same 2-D sweep
# grid -- a looser but still tight relative tolerance (1e-3) covers
# floating-point accumulation differences without masking a real
# methodology divergence.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters
from modules.tyre_fit_auto import fit_session

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REL_TOL_FIT = 1e-6
REL_TOL_VALIDATION = 1e-3

# Recorded reference figures (read directly from the manifests on disk,
# not retyped from memory).
import json
with open("diagnostics/fit_dugoff_first_pass_manifest.json") as f:
    pass0_manifest = json.load(f)
with open("diagnostics/pass1_final_validation_manifest.json") as f:
    pass1_manifest = json.load(f)

data = parse_csv(RAW_FILE)
params = load_parameters()

manifest = fit_session(data, params, data_file_path=RAW_FILE)
manifest.pop("beta_ekf", None)  # not part of the acceptance comparison

print("=" * 100)
print("Phase 2 acceptance check -- modules/tyre_fit_auto.py vs recorded pass-0/pass-1")
print("=" * 100)
print(f"status: {manifest.get('status')}")
print()


def _relcheck(label, got, expect, tol):
    if expect == 0:
        ok = abs(got - expect) < 1e-9
    else:
        ok = abs(got - expect) / abs(expect) < tol
    print(f"  {label:45s} got={got:16.6f}  expect={expect:16.6f}  rel_tol={tol}  {'OK' if ok else 'MISMATCH'}")
    return ok

all_ok = True
print("--- c_alpha / mu_fz vs pass-0 (WP-N1b) ---")
for axle in ("front", "rear"):
    got_c = manifest["axles"][axle]["c_alpha_n_per_rad"]
    exp_c = pass0_manifest["axles"][axle]["c_alpha_n_per_rad"]
    all_ok &= _relcheck(f"{axle} c_alpha_n_per_rad", got_c, exp_c, REL_TOL_FIT)
    got_m = manifest["axles"][axle]["mu_fz_N"]
    exp_m = pass0_manifest["axles"][axle]["mu_fz_N"]
    all_ok &= _relcheck(f"{axle} mu_fz_N", got_m, exp_m, REL_TOL_FIT)
print()

print("--- R derivation vs pass-1's recorded chosen grid point ---")
r_ay_expect = pass1_manifest["config"]["R_ay_var"]
r_yaw_expect = pass1_manifest["config"]["R_yaw_rate_var"]
all_ok &= _relcheck("chosen R_ay_var", manifest["r_sweep"]["chosen"]["R_ay_var"], r_ay_expect, REL_TOL_VALIDATION)
all_ok &= _relcheck("chosen R_yaw_rate_var", manifest["r_sweep"]["chosen"]["R_yaw_rate_var"], r_yaw_expect, REL_TOL_VALIDATION)
print(f"  chosen grid point: r_ay_scale={manifest['r_sweep']['chosen']['r_ay_scale']}  "
      f"r_yaw_scale={manifest['r_sweep']['chosen']['r_yaw_scale']}  found_in_band={manifest['r_sweep']['found_in_band']}")
print()

print("--- validation figures vs pass-1 baseline ---")
all_ok &= _relcheck("NIS yaw_rate_exceedance", manifest["nis"]["yaw_rate_exceedance"], pass1_manifest["nis"]["yaw_rate_exceedance"], REL_TOL_VALIDATION)
all_ok &= _relcheck("NIS ay_exceedance", manifest["nis"]["ay_exceedance"], pass1_manifest["nis"]["ay_exceedance"], REL_TOL_VALIDATION)
all_ok &= _relcheck("NIS combined_exceedance", manifest["nis"]["combined_exceedance"], pass1_manifest["nis"]["combined_exceedance"], REL_TOL_VALIDATION)
all_ok &= _relcheck("NIS combined_mean_nis", manifest["nis"]["combined_mean_nis"], pass1_manifest["nis"]["combined_mean_nis"], REL_TOL_VALIDATION)
print()
print(f"  sign check median gate (all): got={manifest['sign_check']['median_gate_all']}  "
      f"expect={pass1_manifest['sign_check']['median_gate_all_corners']}")
print(f"  sign check median gate (racing): got={manifest['sign_check']['median_gate_racing']}  "
      f"expect={pass1_manifest['sign_check']['median_gate_racing_speed']}")
print("  NOTE: pass1_final_validation.py's 'racing' population (13 corners) uses an ad-hoc window-median")
print("  speed classification; this module uses corner_analysis.py's canonical speed_class field (11")
print("  corners) -- a pre-existing convention inconsistency, not a bug (see thesis_notes.md Phase 1")
print("  correction entry). Per-sample fraction is therefore NOT numerically comparable and is reported,")
print("  not gated:")
print(f"    got (11 racing corners)={manifest['sign_check']['per_sample_fraction_racing']:.4f}  "
      f"pass1_final_validation.py (13 racing corners)={pass1_manifest['sign_check']['per_sample_fraction_racing_speed']:.4f}")
print()
for axle in ("front", "rear"):
    all_ok &= _relcheck(f"{axle} onset_deg", manifest["onset_coverage"][axle]["onset_deg"],
                         pass1_manifest["onset_coverage"][axle]["onset_deg"], REL_TOL_VALIDATION)
    all_ok &= _relcheck(f"{axle} coverage_fraction", manifest["onset_coverage"][axle]["coverage_fraction"],
                         pass1_manifest["onset_coverage"][axle]["coverage_fraction"], REL_TOL_VALIDATION)
print()
all_ok &= _relcheck("h2_vs_ay_apex correlation", manifest["h2_vs_ay_apex"]["correlation"],
                     pass1_manifest["h2_vs_ay_apex"]["correlation"], REL_TOL_VALIDATION)

print()
print("=" * 100)
print(f"OVERALL: {'PASS -- automation reproduces the recorded procedure' if all_ok else 'MISMATCH -- investigate before trusting the automation'}")
print("=" * 100)
