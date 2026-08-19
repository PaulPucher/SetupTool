# WP-S4b (Open Board item B, sideslip methods comparison): observer
# self-consistency check. Read-only, Tier B -- no production/config
# change. Sibling script rather than an inspect_sideslip_methods_
# comparison.py extension: this is a narrow, one-off cross-check specific
# to the Kalman observer's own internal consistency, not a metric future
# CANDIDATES entries would automatically want (unlike the harness's five
# standard metrics) -- same reasoning already applied to WP-S3b/S3c.
#
# Context: WP-S4 found the observer's own alpha_r (alpha_r_C) overshoots
# WP-S3c's force-balance steady-state expectation (alpha_r_ss) by 2-3x
# at C3/C11/C13. Hypothesis under test: alpha_r_ss was biased LOW because
# its Cr (C_linear_ref_r) came from a windowed OLS regression of Fy_r
# against the KINEMATIC (washout-suppressed, under-reading) alpha_r --
# a too-small regressor range inflates the fitted slope, which then
# deflates alpha_r_ss = Fy_r_needed / Cr.
#
# Sections:
#   1. Cr as currently computed (from A's alpha) vs Cr recomputed with
#      the observer's own alpha fed through the identical estimate_
#      cornering_stiffness linear-reference logic -- per corner, at A's
#      own near-zero-alpha_r sample set (same samples WP-S3c Section 2
#      and WP-S4's own beta_C/alpha_r_C table used, for direct
#      comparability). Ratio Cr_C/Cr_A.
#   2. alpha_r_ss recomputed with Cr_C vs the observer's own alpha_r_C
#      at the same samples -- does the 2-3x overshoot shrink, persist,
#      or invert.
#   3. Self-consistency statement (Cr_C is itself alpha_r_C-derived, so
#      this is a coherence check, not an accuracy check).
#   4. Cr_C vs the RCVD fallback reference constants (Caf/Car prior the
#      observer's own Kalman recursion is built on) -- order-of-
#      magnitude plausibility only.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
)
from diagnostics.sideslip_kalman_observer import estimate_sideslip_kalman

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
NEAR_ZERO_SLIP_DEG = 0.2  # matches inspect_c9_negative_cs.py / Metric 5 / WP-S3c

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
vp = params["vehicle"]
se = params["stability_estimation"]

t_ref = state["time"]
ay = state["ay_mps2"]
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving_no_kerb = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

a = vp["cog_to_front_axle_m"]   # lf
wheelbase = vp["wheelbase_m"]
mass_kg = vp["mass_kg"]

Caf_prior = se["cs_front_fallback_reference_n_per_rad"]
Car_prior = se["cs_rear_fallback_reference_n_per_rad"]

# --- A (production kinematic) and C (Kalman observer), same Fy (beta-
# independent) shared across both. ---
beta_a = estimate_sideslip(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
forces = estimate_lateral_forces(state, params)
cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
alpha_r_a = slip_a["alpha_r_filt"]
Fy_r = forces["Fy_r_filt"]
Cr_a_arr = cs_a["C_linear_ref_r"]

beta_c = estimate_sideslip_kalman(state, params)
slip_c = estimate_slip_angles(state, beta_c, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)
alpha_r_c = slip_c["alpha_r_filt"]
Cr_c_arr = cs_c["C_linear_ref_r"]

Fy_r_needed_full = mass_kg * ay * a / wheelbase  # same steady-state 2-DOF moment balance, WP-S3c

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in data.get("corners", []):
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to inspect_c9_negative_cs.py's / Metric 5's / WP-S3b/c's
    # own helper of the same name.
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


# --- Pool, per corner, at A's own near-zero-alpha_r samples ---------------

per_corner = {}
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    pooled_ay, pooled_Cr_a, pooled_Cr_c, pooled_FyRn, pooled_ar_c = [], [], [], [], []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop <= sl.start:
            continue
        mm = moving_no_kerb[sl]
        ar_a = alpha_r_a[sl]
        valid = mm & np.isfinite(ar_a) & (np.abs(ar_a) < near_zero_rad)
        if not valid.any():
            continue
        pooled_ay.append(ay[sl][valid])
        pooled_Cr_a.append(Cr_a_arr[sl][valid])
        pooled_Cr_c.append(Cr_c_arr[sl][valid])
        pooled_FyRn.append(Fy_r_needed_full[sl][valid])
        pooled_ar_c.append(alpha_r_c[sl][valid])
    if not pooled_ay:
        continue
    per_corner[cid] = {
        "ay": np.concatenate(pooled_ay),
        "Cr_a": np.concatenate(pooled_Cr_a),
        "Cr_c": np.concatenate(pooled_Cr_c),
        "FyRn": np.concatenate(pooled_FyRn),
        "ar_c": np.concatenate(pooled_ar_c),
    }

# --- Section 1: Cr_A vs Cr_C -----------------------------------------------

print("=" * 78)
print("SECTION 1 -- Cr (C_linear_ref_r) from A's alpha vs from the observer's alpha")
print("=" * 78)
print("Same near-zero-alpha_r(A) samples per corner as WP-S3c Section 2 / WP-S4's")
print("own beta_C/alpha_r_C table, for direct comparability.")
print()

for cid in stable_ids:
    d = per_corner.get(cid)
    if d is None:
        print(f"  C{cid}: no near-zero-alpha_r(A) samples -- skipped.")
        continue
    valid_a = np.isfinite(d["Cr_a"]) & (d["Cr_a"] > 0)
    valid_c = np.isfinite(d["Cr_c"]) & (d["Cr_c"] > 0)
    both = valid_a & valid_c
    if not both.any():
        print(f"  C{cid}: n={len(d['ay']):4d}  no sample with both Cr_A and Cr_C valid -- skipped.")
        continue
    med_Cr_a = float(np.median(d["Cr_a"][both]))
    med_Cr_c = float(np.median(d["Cr_c"][both]))
    ratio = med_Cr_c / med_Cr_a if med_Cr_a != 0 else float("nan")
    print(f"  C{cid}: n={int(both.sum()):4d}  Cr_A={med_Cr_a:8.0f} N/rad  Cr_C={med_Cr_c:8.0f} N/rad  "
          f"ratio Cr_C/Cr_A={ratio:.3f}")

# --- Section 2: alpha_r_ss recomputed with Cr_C vs observer's own alpha_r_C

print()
print("=" * 78)
print("SECTION 2 -- alpha_r_ss (via Cr_C) vs the observer's own alpha_r_C")
print("=" * 78)
print("alpha_r_ss_new = Fy_r_needed / Cr_C (same force-balance formula as WP-S3c,")
print("new denominator only). Compare against alpha_r_C (the observer's own estimate)")
print("at the same samples -- does the previously-reported 2-3x overshoot (using Cr_A)")
print("shrink, persist, or invert.")
print()

FOCUS_CORNERS = {3, 11, 13}
for cid in stable_ids:
    d = per_corner.get(cid)
    if d is None:
        continue
    valid_c = np.isfinite(d["Cr_c"]) & (d["Cr_c"] > 0)
    if not valid_c.any():
        print(f"  C{cid}: no valid Cr_C -- alpha_r_ss_new not computable.")
        continue
    alpha_r_ss_new = d["FyRn"][valid_c] / d["Cr_c"][valid_c]
    med_ss_new_deg = np.degrees(float(np.median(alpha_r_ss_new)))
    med_ar_c_deg = np.degrees(float(np.median(d["ar_c"])))
    ratio = med_ar_c_deg / med_ss_new_deg if med_ss_new_deg != 0 else float("nan")
    flag = "  <-- focus corner (2-3x overshoot reported, WP-S4)" if cid in FOCUS_CORNERS else ""
    print(f"  C{cid}: alpha_r_ss_new={med_ss_new_deg:+7.3f} deg  alpha_r_C={med_ar_c_deg:+7.3f} deg  "
          f"ratio alpha_r_C/alpha_r_ss_new={ratio:+.3f}{flag}")

# --- Section 4 data gathered here, printed after Section 3's text below ---

all_Cr_c_valid = np.concatenate([per_corner[cid]["Cr_c"][np.isfinite(per_corner[cid]["Cr_c"]) &
                                                          (per_corner[cid]["Cr_c"] > 0)]
                                  for cid in per_corner])

print()
print("=" * 78)
print("SECTION 3 -- self-consistency statement")
print("=" * 78)
print("Cr_C is itself derived FROM alpha_r_C (the same windowed-OLS linear-reference")
print("logic, just fed the observer's own alpha instead of A's) -- so a small ratio in")
print("Section 2 (alpha_r_C close to alpha_r_ss_new) shows the observer's alpha and the")
print("observer's own regressed stiffness are MUTUALLY SELF-CONSISTENT: they satisfy the")
print("same force-balance relation the observer nominally targets. SELF-CONSISTENCY IS")
print("NOT ACCURACY -- Cr_C is computed FROM alpha_r_C, so agreement is partly by")
print("construction (regressing Fy_r against alpha_r_C, then dividing Fy_r_needed by")
print("that same regression's slope, is close to circular for any alpha whose OLS fit is")
print("well-conditioned). A wrong-but-internally-coherent estimator -- e.g. one with a")
print("systematically miscalibrated prior stiffness that nonetheless produces a smoothly")
print("varying alpha -- would pass this same test. Section 4 is the independent check:")
print("comparing Cr_C against the FIXED prior stiffness (Caf_prior/Car_prior) the")
print("observer's own Kalman recursion assumed BEFORE seeing any data, which Cr_C did")
print("not have access to during its own regression.")

print()
print("=" * 78)
print("SECTION 4 -- Cr_C vs the RCVD fallback reference constants")
print("=" * 78)
print(f"Rear fallback reference in config (Car_prior, the observer's own fixed Kalman")
print(f"prior): {Car_prior} N/rad.")
if len(all_Cr_c_valid) > 0:
    p25, p50, p75 = np.percentile(all_Cr_c_valid, [25, 50, 75])
    print(f"Cr_C distribution across all pooled near-zero-alpha_r(A) samples, all corners "
          f"(n={len(all_Cr_c_valid)}): p25={p25:.0f}  median={p50:.0f}  p75={p75:.0f} N/rad")
    print(f"Ratio median(Cr_C)/Car_prior = {p50/Car_prior:.3f}")
else:
    print("No valid Cr_C samples pooled -- comparison not computable.")
