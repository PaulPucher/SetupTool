# Circularity check: the same method and quantities that condemned
# the linear-tyre observer (diagnostics/inspect_observer_slip_angle_
# circularity.py), run here against the pass-0 nonlinear Dugoff EKF's
# RAW slip angles, so the two are directly comparable. Read-only,
# Tier B -- no production/config change, no refit.
#
# This is a separate file rather than a parameterised edit of the
# original script, so the original (linear-observer) run stays
# reproducible byte-for-byte and untouched -- but every computation in
# sections 1-4 below is a literal copy of that script's own logic (same
# masking, same formulas, same print format), so the two scripts'
# numbers are directly comparable. Only the beta/slip-angle SOURCE
# differs (estimate_sideslip_ekf_dugoff instead of
# estimate_sideslip_kalman) and, in sections 1+2, the "prior" printed
# alongside the best-fit slope is now the EKF's own frozen c_alpha
# (config tyre_model_ekf.pass_0), the analogous fixed low-slip
# reference for this filter.
#
# Sections 5+6 are NEW, not present in the linear-observer version,
# because the reference curve is now nonlinear:
#   5. Evaluate the FROZEN pass-0 Dugoff curve at the EKF's own slip
#      angles and compare against measured Fy. High R^2 here would mean
#      the EKF's alpha is close to a deterministic inverse of the
#      assumed curve (the plot's shape restates the model, not measured
#      tyre behaviour); a materially lower R^2 means real information.
#   6. Onset coverage (fraction of EKF alpha beyond each axle's own
#      lambda=1 boundary), for direct comparison against the kinematic
#      coverage figures already reported (34.0% front, 6.95% rear).

import sys

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.tyre_model import dugoff_lateral_force

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PASS_ID = sys.argv[1] if len(sys.argv) > 1 else "pass_0"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
se = params["stability_estimation"]
cfg = params["tyre_model_ekf"][PASS_ID]

t_ref = state["time"]
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)
base_mask = moving & racing_mask  # section 6 population, matches the earlier onset-coverage report

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


laps_by_number = {l["lap_number"]: l for l in laps}
in_corner_mask = np.zeros_like(t_ref, dtype=bool)
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
        sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop > sl.start:
            in_corner_mask[sl] = True

corner_valid_mask = moving & racing_mask & in_corner_mask

beta_a = estimate_sideslip(state, params)
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id=PASS_ID)
beta_c = ekf_result["beta"]  # RAW, pre-fallback
slip_a = estimate_slip_angles(state, beta_a, params)
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)

c_alpha_prior_f = cfg["c_alpha_front_n_per_rad"]
c_alpha_prior_r = cfg["c_alpha_rear_n_per_rad"]
mu_fz_f = cfg["mu_fz_front_N"]
mu_fz_r = cfg["mu_fz_rear_N"]

# --- Sections 1 & 2: correlation/slope of EKF alpha vs Fy -------------------

print("=" * 100)
print(f"SECTIONS 1+2 -- {PASS_ID} EKF alpha vs Fy, corner samples only (linear fit, radians)")
print("=" * 100)
print(f"Rear prior (c_alpha_rear, frozen {PASS_ID}): {c_alpha_prior_r} N/rad.  "
      f"Front prior (c_alpha_front, frozen {PASS_ID}): {c_alpha_prior_f} N/rad.")
print()

fit_results = {}
for axle_name, alpha_c_deg, Fy in (
    ("front", slip_c["alpha_f_filt"], forces["Fy_f_filt"]),
    ("rear", slip_c["alpha_r_filt"], forces["Fy_r_filt"]),
):
    alpha_rad = alpha_c_deg[corner_valid_mask]
    fy_vals = Fy[corner_valid_mask]
    finite = np.isfinite(alpha_rad) & np.isfinite(fy_vals)
    alpha_rad, fy_vals = alpha_rad[finite], fy_vals[finite]

    r = float(np.corrcoef(alpha_rad, fy_vals)[0, 1])
    slope, intercept = np.polyfit(alpha_rad, fy_vals, 1)
    slope, intercept = float(slope), float(intercept)
    pred = slope * alpha_rad + intercept
    resid = fy_vals - pred
    resid_std = float(np.std(resid))
    fy_std = float(np.std(fy_vals))
    prior = c_alpha_prior_f if axle_name == "front" else c_alpha_prior_r

    fit_results[axle_name] = {"r": r, "r2": r ** 2, "slope": slope, "intercept": intercept,
                               "resid_std": resid_std, "fy_std": fy_std, "n": int(len(alpha_rad))}

    print(f"--- {axle_name} ---")
    print(f"  n={len(alpha_rad)}  r={r:+.4f}  R^2={r**2:.4f}")
    print(f"  best-fit slope={slope:.0f} N/rad  (prior={prior:.0f} N/rad, ratio slope/prior={slope/prior:.3f})  "
          f"intercept={intercept:+.0f} N")
    print(f"  residual std={resid_std:.0f} N  ({resid_std/fy_std*100:.1f}% of Fy's own std={fy_std:.0f} N)")
    print()

# --- Section 3: CS ratio computed from EKF alpha vs kinematic alpha --------

print("=" * 100)
print("SECTION 3 -- CS_ratio distribution, per axle: EKF-derived vs production (kinematic)")
print("=" * 100)

cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)

for axle_name, key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
    for label, cs in (("production (kinematic)", cs_a), ("EKF-derived", cs_c)):
        vals = cs[key][corner_valid_mask]
        vals = vals[np.isfinite(vals)]
        p5, p25, p50, p75, p95 = np.percentile(vals, [5, 25, 50, 75, 95])
        print(f"  {axle_name:6s} {label:24s} n={len(vals):6d}  "
              f"p5={p5:.3f}  p25={p25:.3f}  median={p50:.3f}  p75={p75:.3f}  p95={p95:.3f}")
    print()

# --- Section 4: worst-phase-per-corner-instance threshold counts -----------

print("=" * 100)
print("SECTION 4 -- corner instances flagged under CURRENT CS thresholds "
      "(indication only, not a re-derivation)")
print("=" * 100)

STRONG_CSF = params["classification"]["STRONG_CSF"]["value"]
STRONG_CSR = params["classification"]["STRONG_CSR"]["value"]
MODERATE_CSF = params["classification"]["MODERATE_CSF"]["value"]
MODERATE_CSR = params["classification"]["MODERATE_CSR"]["value"]
print(f"Thresholds (config/parameters.json classification block): STRONG_CSF={STRONG_CSF}  "
      f"STRONG_CSR={STRONG_CSR}  MODERATE_CSF={MODERATE_CSF}  MODERATE_CSR={MODERATE_CSR}")
print()

stab = estimate_yaw_moment_stability(state, beta_a, params, laps)  # kinematic; unused fields only, see header note


def _worst_per_instance(cs):
    summaries = summarise_corners(corners, cs, stab, state)
    worst_csf, worst_csr = [], []
    for s in summaries:
        csfs = [p["cs_ratio_f"]["median"] for p in s["phases"].values() if p["cs_ratio_f"]["median"] == p["cs_ratio_f"]["median"]]
        csrs = [p["cs_ratio_r"]["median"] for p in s["phases"].values() if p["cs_ratio_r"]["median"] == p["cs_ratio_r"]["median"]]
        if csfs:
            worst_csf.append(min(csfs))
        if csrs:
            worst_csr.append(min(csrs))
    return np.array(worst_csf), np.array(worst_csr)


worst_csf_a, worst_csr_a = _worst_per_instance(cs_a)
worst_csf_c, worst_csr_c = _worst_per_instance(cs_c)

print(f"Total corner instances: kinematic n={len(worst_csf_a)} (front) / {len(worst_csr_a)} (rear)  "
      f"EKF n={len(worst_csf_c)} (front) / {len(worst_csr_c)} (rear)")
print()
for label, wf, wr in (("production (kinematic)", worst_csf_a, worst_csr_a),
                      ("EKF-derived", worst_csf_c, worst_csr_c)):
    n_strong_f = int((wf < STRONG_CSF).sum())
    n_moderate_f = int(((wf >= STRONG_CSF) & (wf < MODERATE_CSF)).sum())
    n_strong_r = int((wr < STRONG_CSR).sum())
    n_moderate_r = int(((wr >= STRONG_CSR) & (wr < MODERATE_CSR)).sum())
    print(f"  {label}:")
    print(f"    front: strong (CSf<{STRONG_CSF})={n_strong_f}/{len(wf)}   "
          f"moderate ({STRONG_CSF}<=CSf<{MODERATE_CSF})={n_moderate_f}/{len(wf)}")
    print(f"    rear:  strong (CSr<{STRONG_CSR})={n_strong_r}/{len(wr)}   "
          f"moderate ({STRONG_CSR}<=CSr<{MODERATE_CSR})={n_moderate_r}/{len(wr)}")
    print()

# --- Section 5 (NEW): frozen Dugoff curve at EKF's own alpha vs measured Fy

print("=" * 100)
print("SECTION 5 (new, no linear-observer analogue) -- frozen pass-0 Dugoff curve "
      "evaluated at the EKF's own alpha, vs measured Fy")
print("=" * 100)
print("Interpretation to apply, not asserted here: R^2 near 1 means the EKF's alpha is "
      "close to a deterministic inverse of the assumed curve (restates the model, not "
      "measured tyre behaviour); a materially lower R^2 means the cloud departs from the "
      "assumed curve and that departure carries real information for the refit.")
print()

for axle_name, alpha_c, Fy, c_alpha_p, mu_fz_p in (
    ("front", slip_c["alpha_f_filt"], forces["Fy_f_filt"], c_alpha_prior_f, mu_fz_f),
    ("rear", slip_c["alpha_r_filt"], forces["Fy_r_filt"], c_alpha_prior_r, mu_fz_r),
):
    alpha_rad = alpha_c[corner_valid_mask]
    fy_meas = Fy[corner_valid_mask]
    finite = np.isfinite(alpha_rad) & np.isfinite(fy_meas)
    alpha_rad, fy_meas = alpha_rad[finite], fy_meas[finite]

    fy_pred = dugoff_lateral_force(alpha_rad, c_alpha_p, mu_fz_p)
    resid = fy_meas - fy_pred
    ss_res = np.sum(resid ** 2)
    ss_tot = np.sum((fy_meas - np.mean(fy_meas)) ** 2)
    r2 = 1.0 - ss_res / ss_tot
    rms = float(np.sqrt(np.mean(resid ** 2)))
    p10, p50, p90 = np.percentile(resid, [10, 50, 90])

    print(f"  {axle_name}: n={len(alpha_rad)}  R^2={r2:.4f}  RMS residual={rms:.0f} N  "
          f"residual percentiles: p10={p10:.0f}  p50={p50:.0f}  p90={p90:.0f} N")
print()

# --- Section 6 (NEW): onset coverage, EKF alpha, base_mask population ------

print("=" * 100)
print("SECTION 6 (new) -- fraction of EKF alpha beyond each axle's own onset "
      "(base_mask population, matching the earlier kinematic-coverage report)")
print("=" * 100)

tan_onset_f = mu_fz_f / (2.0 * c_alpha_prior_f)
tan_onset_r = mu_fz_r / (2.0 * c_alpha_prior_r)
onset_f_deg = np.degrees(np.arctan(tan_onset_f))
onset_r_deg = np.degrees(np.arctan(tan_onset_r))
print(f"onset: front {onset_f_deg:.3f} deg  rear {onset_r_deg:.3f} deg "
      f"(cf. kinematic-population reference: front 34.0%, rear 6.95%)")

for axle_name, alpha_c, onset_deg in (
    ("front", slip_c["alpha_f_filt"], onset_f_deg),
    ("rear", slip_c["alpha_r_filt"], onset_r_deg),
):
    alpha_deg = np.degrees(alpha_c)[base_mask]
    frac_beyond = float((np.abs(alpha_deg) > onset_deg).mean())
    p50a, p90a, p99a = np.percentile(np.abs(alpha_deg), [50, 90, 99])
    print(f"  {axle_name}: fraction beyond onset = {frac_beyond:.4f}   "
          f"|alpha| p50={p50a:.3f}  p90={p90a:.3f}  p99={p99a:.3f} deg")
