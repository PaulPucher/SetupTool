# WP-S6 critical check (Open Board item B, sideslip methods comparison):
# is the Kalman observer's rear slip angle independent information, or
# substantially a restatement of Fy_r through the fixed rear stiffness
# prior? Read-only, Tier B -- no production/config change.
#
# Concern under test (raised against the rear force-vs-slip plot,
# diagnostics/plot_slip_angle_comparison.py): the observer's own
# measurement model ties ay directly to beta via an equation that
# embeds Caf/Car (ay = -(Caf+Car)/m*beta - (Caf*lf-Car*lr)/(m*Vx)*
# yaw_rate + Caf/m*delta_f, thesis_notes.md WP-S4 entry) -- and ay is
# one of the observer's two measurement channels. If the Kalman gain
# pulls beta (and hence alpha_r = yaw_geom_r - beta) strongly toward
# whatever makes this equation hold, alpha_r_C could end up close to a
# deterministic function of Fy_r / Car_prior rather than carrying
# independent content -- which would make any CS_ratio_r computed from
# it collapse toward 1 (since C_alpha would recover ~Car_prior
# regardless of true tyre behaviour) and destroy rear saturation
# detection. The front should retain more independent content: delta_f
# (measured steering angle) enters alpha_f = delta_f - beta -
# yaw_geom_f as a genuine external input, not run through the KF's own
# Caf/Car-based reconstruction at all (it is the observer's control
# input u, not a measurement).
#
# Scope note for item 4 below: counts use the CS-ratio thresholds only
# (STRONG_CSF/CSR, MODERATE_CSF/CSR, config/parameters.json
# "classification" block), NOT the full severity classifier's AND-logic
# with destabilising yaw (ui/views/outing_form.py _classify_corner) --
# that AND-logic is a separate signal, irrelevant to the specific
# question here (does CS_ratio itself stay informative). summarise_
# corners is called with the KINEMATIC yaw-stability object for both
# CS variants (recomputing yaw stability from the observer's beta is
# outside this check's scope and its own separate, expensive
# computation) -- only cs_ratio_f/cs_ratio_r are read from its output,
# stability_observed is never used here.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from diagnostics.sideslip_kalman_observer import estimate_sideslip_kalman

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
se = params["stability_estimation"]

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
beta_c = estimate_sideslip_kalman(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)

Caf_prior = se["cs_front_fallback_reference_n_per_rad"]
Car_prior = se["cs_rear_fallback_reference_n_per_rad"]

# --- Sections 1 & 2: correlation/slope of observer alpha vs Fy --------------

print("=" * 100)
print("SECTIONS 1+2 -- observer alpha vs Fy, corner samples only (linear fit, radians)")
print("=" * 100)
print(f"Rear prior (Car_prior): {Car_prior} N/rad.  Front prior (Caf_prior): {Caf_prior} N/rad.")
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
    prior = Caf_prior if axle_name == "front" else Car_prior

    fit_results[axle_name] = {"r": r, "r2": r ** 2, "slope": slope, "intercept": intercept,
                               "resid_std": resid_std, "fy_std": fy_std, "n": int(len(alpha_rad))}

    print(f"--- {axle_name} ---")
    print(f"  n={len(alpha_rad)}  r={r:+.4f}  R^2={r**2:.4f}")
    print(f"  best-fit slope={slope:.0f} N/rad  (prior={prior} N/rad, ratio slope/prior={slope/prior:.3f})  "
          f"intercept={intercept:+.0f} N")
    print(f"  residual std={resid_std:.0f} N  ({resid_std/fy_std*100:.1f}% of Fy's own std={fy_std:.0f} N)")
    print()

# --- Section 3: CS ratio computed from observer alpha vs kinematic alpha ---

print("=" * 100)
print("SECTION 3 -- CS_ratio distribution, per axle: observer-derived vs production (kinematic)")
print("=" * 100)

cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)

for axle_name, key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
    for label, cs in (("production (kinematic)", cs_a), ("observer-derived", cs_c)):
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
      f"observer n={len(worst_csf_c)} (front) / {len(worst_csr_c)} (rear)")
print()
for label, wf, wr in (("production (kinematic)", worst_csf_a, worst_csr_a),
                      ("observer-derived", worst_csf_c, worst_csr_c)):
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
