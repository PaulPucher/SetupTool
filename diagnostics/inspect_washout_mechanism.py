# WP-S3c (Open Board item B, sideslip methods comparison): washout-
# mechanism check. Read-only, Tier B (signal/data engineering diagnostic
# -- standard windowed-integration/regression techniques, no new
# vehicle-dynamics claim). No production/config change; nothing here is
# whitelisted or called from any pipeline/UI path. estimate_sideslip,
# estimate_slip_angles, estimate_lateral_forces, estimate_cornering_
# stiffness are called exactly as production does; _butterworth_lowpass/
# _highpass_filter (simple filter wrappers, not closures) are imported
# directly rather than reimplemented, so the ablation's filtering is
# bit-identical to production's own.
#
# Hypothesis under test (follow-up to WP-S3/WP-S3b): the near-zero-alpha-
# at-high-ay samples Metric 5 found are an ARTEFACT of the 0.05 Hz
# washout high-pass stripping genuine steady-state sideslip out of the
# kinematically-integrated beta -- not a property of the underlying
# vehicle motion.
#
# Sections:
#   1. Phase location of the washout candidate's near-zero-|alpha_r|
#      samples: per corner, fraction in entry/apex/exit phases, and
#      fraction mid-corner (apex phase OR middle 50% of the bracket by
#      s-position) vs at the corner's edges. Washout predicts mid-
#      corner, steady-state-dominated.
#   2. Steady-state force-balance expectation: alpha_r_ss = Fy_r_needed
#      / Cr, Fy_r_needed = m*ay*lf/L (standard steady-state 2-DOF
#      moment balance, Milliken & Milliken RCVD -- diagnostic use only,
#      not implemented), Cr = C_linear_ref_r at the same samples (its
#      own provenance is alpha-derived, so this is informative for
#      order-of-magnitude/sign, not an independent magnitude check).
#   3. Washout ablation: per corner, per-lap-instance beta re-anchored
#      to zero at the last straight-line sample before corner entry,
#      integrating raw beta_dot with NO high-pass -- median beta
#      (washout vs re-anchored) over the canonical window, plus the
#      near-zero-slip sample count and Fy medians recomputed from the
#      re-anchored beta's own alpha. A data-driven residual-drift
#      estimate uses the first straight-line sample after corner exit
#      (where true beta should read ~0) as an empirical drift check.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    _butterworth_lowpass, _highpass_filter,
)
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
NEAR_ZERO_SLIP_DEG = 0.2  # matches inspect_c9_negative_cs.py / Metric 5
MID_CORNER_S_FRACTION = (0.25, 0.75)  # middle 50% of the bracket by s-position
MIN_FILT_LEN = 30  # conservative floor above filtfilt's own default padlen (~15 for order-4)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
vp = params["vehicle"]
se = params["stability_estimation"]

t_ref = state["time"]
sr = state["sample_rate_hz"]
dt = 1.0 / sr
v = state["v_mps"]
yaw_rate = state["yaw_rate_radps"]
delta_f = state["delta_f_rad"]
ay = state["ay_mps2"]
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving_no_kerb = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

a = vp["cog_to_front_axle_m"]           # lf
b = vp["cog_to_rear_axle_m"]
wheelbase = vp["wheelbase_m"]
mass_kg = vp["mass_kg"]
cs_cutoff_hz = se["cs_filter_cutoff_hz"]
washout_cutoff_hz = se["beta_washout_cutoff_hz"]

print(f"Washout candidate mechanism: production beta_washout_cutoff_hz={washout_cutoff_hz} Hz "
      f"high-pass on the raw kinematic integral; alpha's own low-pass stays at "
      f"cs_filter_cutoff_hz={cs_cutoff_hz} Hz throughout (unablated).")

# --- Production (washout) candidate, exactly as estimate_sideslip builds it ---
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
alpha_f, alpha_r = slip["alpha_f_filt"], slip["alpha_r_filt"]
Fy_f, Fy_r = forces["Fy_f_filt"], forces["Fy_r_filt"]
C_linear_ref_r = cs["C_linear_ref_r"]

# Same construction as estimate_sideslip's own beta_dot (mirrors lines
# 333-337 of modules/stability_analysis.py up to the cumsum/high-pass
# step, which section 3 replaces per-corner below).
v_safe = np.where(moving_raw, v, 1.0)
beta_dot = np.where(moving_raw, ay / v_safe - yaw_rate, 0.0)

Fy_r_needed_full = mass_kg * ay * a / wheelbase  # steady-state 2-DOF moment balance, diagnostic only

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in data.get("corners", []):
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)

phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
apex_half_window = se["apex_half_window_samples"]

# straight-line anchor mask, reused from inspect_wheel_speed_sources.py's
# established thresholds (already the standard "is the car going straight"
# gate across this WP thread) -- NOT restricted to racing_mask, since an
# anchor may legitimately sit just before/after a corner regardless of
# whether that specific lap is flagged valid-for-analysis elsewhere.
ay_g = ay / 9.81
yaw_rate_degps = np.degrees(yaw_rate)
straight_mask = moving_raw & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to inspect_c9_negative_cs.py's / Metric 5's own helper.
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


def _phase_slice(start_t, end_t, is_apex=False):
    # Identical to Metric 2/4's own helper (diagnostics/inspect_sideslip_
    # methods_comparison.py) -- apex-instant expansion included.
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t_ref, start_t, side="left"))
    hi = int(np.searchsorted(t_ref, end_t, side="right"))
    if is_apex and hi <= lo:
        lo = max(0, lo - apex_half_window)
        hi = min(len(t_ref), lo + 2 * apex_half_window + 1)
    return slice(lo, hi)


def _classify_phase(global_idx, phase_slices):
    for phase, sl in phase_slices.items():
        if sl.start <= global_idx < sl.stop:
            return phase
    return "unclassified"


def _find_anchor_before(lap_lo, window_start):
    idx = window_start - 1
    while idx >= lap_lo:
        if straight_mask[idx]:
            return idx
        idx -= 1
    return None


def _find_anchor_after(window_end, lap_hi):
    idx = window_end
    while idx < lap_hi:
        if straight_mask[idx]:
            return idx
        idx += 1
    return None


def _local_slip_angles(beta_local, v_local, yaw_rate_local, delta_f_local, moving_local):
    # Mirrors estimate_slip_angles's own arctan construction exactly
    # (modules/stability_analysis.py lines 507-519), scoped to a local
    # per-corner segment instead of the full session.
    if len(beta_local) < MIN_FILT_LEN:
        return None, None
    v_x_safe_l = np.where(moving_local, v_local * np.cos(beta_local), 1.0)
    af = delta_f_local - np.arctan((v_local * np.sin(beta_local) + a * yaw_rate_local) / v_x_safe_l)
    ar = -np.arctan((v_local * np.sin(beta_local) - b * yaw_rate_local) / v_x_safe_l)
    af = np.where(moving_local, af, 0.0)
    ar = np.where(moving_local, ar, 0.0)
    try:
        af = _butterworth_lowpass(af, cs_cutoff_hz, sr)
        ar = _butterworth_lowpass(ar, cs_cutoff_hz, sr)
    except ValueError:
        return None, None
    return af, ar


# --- Per-corner accumulators ---------------------------------------------

per_corner = {cid: {
    "phase_counts": {}, "mid_count": 0, "n_selected": 0,
    "ay_pool": [], "alpha_r_pool": [], "Cr_pool": [], "FyR_needed_pool": [],
    "FyR_wash_nearzero_pool": [],
    "beta_wash_win_pool": [], "beta_reanchor_win_pool": [],
    "FyR_reanchor_nearzero_pool": [], "n_r_reanchor": 0,
    "FyF_reanchor_nearzero_pool": [], "n_f_reanchor": 0,
    "n_instances": 0, "n_instances_with_anchor": 0,
} for cid in stable_ids}

drift_records = []
n_skipped_no_anchor = 0
n_skipped_too_short = 0

for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    acc = per_corner[cid]

    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop <= sl.start:
            continue
        acc["n_instances"] += 1

        lap_lo = int(np.searchsorted(t_ref, lap["start_time"], side="left"))
        lap_hi = int(np.searchsorted(t_ref, lap["end_time"], side="right"))

        mm = moving_no_kerb[sl]
        ar_wash = alpha_r[sl]
        af_wash = alpha_f[sl]
        beta_wash_win = beta[sl]
        Fy_r_win = Fy_r[sl]
        Fy_f_win = Fy_f[sl]
        ay_win = ay[sl]
        Cr_win = C_linear_ref_r[sl]
        FyR_needed_win = Fy_r_needed_full[sl]

        # --- Sections 1 + 2 input: near-zero-alpha_r (washout) selection ---
        valid_r_wash = mm & np.isfinite(ar_wash) & (np.abs(ar_wash) < near_zero_rad)
        if valid_r_wash.any():
            phase_slices = {ph: _phase_slice(*c["segments"][ph], is_apex=(ph == "apex_3")) for ph in phase_keys}
            for li in np.where(valid_r_wash)[0]:
                gi = sl.start + int(li)
                phase = _classify_phase(gi, phase_slices)
                acc["phase_counts"][phase] = acc["phase_counts"].get(phase, 0) + 1
                frac_pos = (s_m[gi] - bracket_start) / (bracket_end - bracket_start) if bracket_end > bracket_start else float("nan")
                mid = (phase == "apex_3") or (MID_CORNER_S_FRACTION[0] <= frac_pos <= MID_CORNER_S_FRACTION[1])
                if mid:
                    acc["mid_count"] += 1
                acc["n_selected"] += 1
            acc["ay_pool"].append(ay_win[valid_r_wash])
            acc["alpha_r_pool"].append(ar_wash[valid_r_wash])
            acc["Cr_pool"].append(Cr_win[valid_r_wash])
            acc["FyR_needed_pool"].append(FyR_needed_win[valid_r_wash])
            acc["FyR_wash_nearzero_pool"].append(Fy_r_win[valid_r_wash])

        # --- Section 3: washout ablation ---
        i_anchor = _find_anchor_before(lap_lo, sl.start)
        if i_anchor is None:
            n_skipped_no_anchor += 1
            continue
        acc["n_instances_with_anchor"] += 1
        i_exit = _find_anchor_after(sl.stop, lap_hi)
        seg_end = (i_exit + 1) if i_exit is not None else sl.stop

        beta_local = np.cumsum(beta_dot[i_anchor:seg_end]) * dt
        window_local = slice(sl.start - i_anchor, sl.stop - i_anchor)

        if i_exit is not None:
            drift_records.append(abs(float(beta_local[i_exit - i_anchor])))

        af_local, ar_local = _local_slip_angles(
            beta_local, v[i_anchor:seg_end], yaw_rate[i_anchor:seg_end],
            delta_f[i_anchor:seg_end], moving_raw[i_anchor:seg_end],
        )
        if af_local is None:
            n_skipped_too_short += 1
            continue

        beta_reanchor_win = beta_local[window_local]
        af_reanchor_win = af_local[window_local]
        ar_reanchor_win = ar_local[window_local]

        acc["beta_wash_win_pool"].append(beta_wash_win[mm])
        acc["beta_reanchor_win_pool"].append(beta_reanchor_win[mm])

        valid_r_re = mm & np.isfinite(ar_reanchor_win) & (np.abs(ar_reanchor_win) < near_zero_rad)
        acc["n_r_reanchor"] += int(valid_r_re.sum())
        if valid_r_re.any():
            acc["FyR_reanchor_nearzero_pool"].append(Fy_r_win[valid_r_re])

        valid_f_re = mm & np.isfinite(af_reanchor_win) & (np.abs(af_reanchor_win) < near_zero_rad)
        acc["n_f_reanchor"] += int(valid_f_re.sum())
        if valid_f_re.any():
            acc["FyF_reanchor_nearzero_pool"].append(Fy_f_win[valid_f_re])


# --- Section 1: phase location -------------------------------------------

print()
print("=" * 78)
print("SECTION 1 -- phase location of washout candidate's near-zero-|alpha_r| samples")
print("=" * 78)
print(f"Mid-corner = apex_3 phase OR within the middle {int((MID_CORNER_S_FRACTION[1]-MID_CORNER_S_FRACTION[0])*100)}% "
      "of the bracket by s-position; washout predicts mid-corner-dominated.")
print()

tot_entry = tot_apex = tot_exit = tot_unclass = tot_mid = tot_n = 0
for cid in stable_ids:
    acc = per_corner[cid]
    n = acc["n_selected"]
    if n == 0:
        print(f"  C{cid}: no near-zero-alpha_r samples -- skipped.")
        continue
    pc = acc["phase_counts"]
    n_entry = pc.get("entry_1_brake", 0) + pc.get("entry_2_turnin", 0)
    n_apex = pc.get("apex_3", 0)
    n_exit = pc.get("exit_4", 0) + pc.get("exit_5", 0)
    n_unclass = pc.get("unclassified", 0)
    n_mid = acc["mid_count"]
    print(f"  C{cid}: n={n:4d}  entry={n_entry/n*100:5.1f}%  apex={n_apex/n*100:5.1f}%  "
          f"exit={n_exit/n*100:5.1f}%  unclassified={n_unclass/n*100:5.1f}%  "
          f"mid-corner={n_mid/n*100:5.1f}%  edge={100-n_mid/n*100:5.1f}%")
    tot_entry += n_entry
    tot_apex += n_apex
    tot_exit += n_exit
    tot_unclass += n_unclass
    tot_mid += n_mid
    tot_n += n

if tot_n:
    print()
    print(f"  AGGREGATE (n={tot_n}): entry={tot_entry/tot_n*100:.1f}%  apex={tot_apex/tot_n*100:.1f}%  "
          f"exit={tot_exit/tot_n*100:.1f}%  unclassified={tot_unclass/tot_n*100:.1f}%  "
          f"mid-corner={tot_mid/tot_n*100:.1f}%  edge={(tot_n-tot_mid)/tot_n*100:.1f}%")

# --- Section 2: steady-state force-balance expectation --------------------

print()
print("=" * 78)
print("SECTION 2 -- steady-state alpha_r expectation vs estimated alpha_r")
print("=" * 78)
print("alpha_r_ss = Fy_r_needed / Cr, Fy_r_needed = m*ay*lf/L (steady-state 2-DOF")
print("moment balance, diagnostic only, NOT implemented). Cr = C_linear_ref_r at the")
print("same near-zero-alpha_r samples -- Cr's own numerator/denominator are alpha_r/")
print("Fy_r-derived (the same construction under test), so this is CIRCULAR for an")
print("independent magnitude claim; the OLS slope naturally demeans a roughly-constant")
print("per-window Fy offset, which limits (but does not remove) that circularity, so")
print("Cr is treated here as informative for order-of-magnitude and sign only.")
print()

for cid in stable_ids:
    acc = per_corner[cid]
    if not acc["ay_pool"]:
        print(f"  C{cid}: no near-zero-alpha_r samples -- skipped.")
        continue
    ay_pool = np.concatenate(acc["ay_pool"])
    ar_pool = np.concatenate(acc["alpha_r_pool"])
    Cr_pool = np.concatenate(acc["Cr_pool"])
    FyRn_pool = np.concatenate(acc["FyR_needed_pool"])

    valid_cr = np.isfinite(Cr_pool) & (Cr_pool > 0)
    if not valid_cr.any():
        print(f"  C{cid}: n={len(ay_pool):4d}  no valid (finite, positive) C_linear_ref_r "
              f"in this window -- alpha_r_ss not computable.  median ay={np.median(ay_pool):+.2f} m/s^2")
        continue
    alpha_r_ss = FyRn_pool[valid_cr] / Cr_pool[valid_cr]
    med_ss_deg = np.degrees(float(np.median(alpha_r_ss)))
    med_actual_deg = np.degrees(float(np.median(ar_pool)))
    med_cr = float(np.median(Cr_pool[valid_cr]))
    med_ay = float(np.median(ay_pool))
    print(f"  C{cid}: n={len(ay_pool):4d} (n_Cr_valid={int(valid_cr.sum())})  "
          f"median ay={med_ay:+7.2f} m/s^2  median Cr={med_cr:8.0f} N/rad  "
          f"alpha_r_ss={med_ss_deg:+7.3f} deg  actual alpha_r={med_actual_deg:+7.3f} deg  "
          f"(gap={med_ss_deg - med_actual_deg:+.3f} deg)")

# --- Section 3: washout ablation -------------------------------------------

print()
print("=" * 78)
print("SECTION 3 -- washout ablation (per-corner re-anchored, no high-pass)")
print("=" * 78)
print(f"n_instances_no_straight_anchor_before={n_skipped_no_anchor}  "
      f"n_instances_segment_too_short_for_filter={n_skipped_too_short}")
print()

for cid in stable_ids:
    acc = per_corner[cid]
    if not acc["beta_wash_win_pool"]:
        print(f"  C{cid}: no re-anchored instances -- skipped "
              f"({acc['n_instances_with_anchor']}/{acc['n_instances']} had a straight-line anchor).")
        continue
    beta_wash_deg = np.degrees(np.median(np.concatenate(acc["beta_wash_win_pool"])))
    beta_re_deg = np.degrees(np.median(np.concatenate(acc["beta_reanchor_win_pool"])))

    print(f"  C{cid}: n_instances_with_anchor={acc['n_instances_with_anchor']}/{acc['n_instances']}  "
          f"median beta washout={beta_wash_deg:+7.3f} deg  median beta re-anchored={beta_re_deg:+7.3f} deg  "
          f"(diff={beta_re_deg - beta_wash_deg:+.3f} deg)")

    n_r_re = acc["n_r_reanchor"]
    fy_r_wash_med = float(np.median(np.concatenate(acc["FyR_wash_nearzero_pool"]))) if acc["FyR_wash_nearzero_pool"] else float("nan")
    fy_r_re_med = float(np.median(np.concatenate(acc["FyR_reanchor_nearzero_pool"]))) if acc["FyR_reanchor_nearzero_pool"] else float("nan")
    n_f_re = acc["n_f_reanchor"]
    fy_f_re_med = float(np.median(np.concatenate(acc["FyF_reanchor_nearzero_pool"]))) if acc["FyF_reanchor_nearzero_pool"] else float("nan")
    print(f"        washout near-zero-alpha_r n={acc['n_selected']:4d}  median Fy_r(washout)={fy_r_wash_med:8.0f} N  ->  "
          f"re-anchored near-zero-alpha_r n={n_r_re:4d}  median Fy_r(re-anchored)={fy_r_re_med:8.0f} N")
    print(f"        re-anchored near-zero-alpha_f n={n_f_re:4d}  median Fy_f(re-anchored)={fy_f_re_med:8.0f} N")

print()
if drift_records:
    drift_arr = np.array(drift_records)
    print(f"Residual-drift estimate (|beta_reanchored| at the first straight-line sample after")
    print(f"corner exit, where true beta should read ~0 -- empirical, not a formal error bound):")
    print(f"  n={len(drift_arr)}  median={np.degrees(np.median(drift_arr)):.3f} deg  "
          f"mean={np.degrees(np.mean(drift_arr)):.3f} deg  p90={np.degrees(np.percentile(drift_arr, 90)):.3f} deg  "
          f"max={np.degrees(np.max(drift_arr)):.3f} deg")
else:
    print("Residual-drift estimate: no instance had both a before- and after-corner straight-line anchor.")

# --- Aggregate before/after summary ---------------------------------------

print()
print("=" * 78)
print("AGGREGATE -- washout vs re-anchored, all corners pooled")
print("=" * 78)
all_wash = [x for cid in stable_ids for x in per_corner[cid]["beta_wash_win_pool"]]
all_re = [x for cid in stable_ids for x in per_corner[cid]["beta_reanchor_win_pool"]]
if all_wash and all_re:
    print(f"  median |beta| washout={np.degrees(np.median(np.abs(np.concatenate(all_wash)))):.3f} deg  "
          f"median |beta| re-anchored={np.degrees(np.median(np.abs(np.concatenate(all_re)))):.3f} deg")

n_r_wash_total = sum(per_corner[cid]["n_selected"] for cid in stable_ids)
n_r_re_total = sum(per_corner[cid]["n_r_reanchor"] for cid in stable_ids)
fy_r_wash_all = [x for cid in stable_ids for x in per_corner[cid]["FyR_wash_nearzero_pool"]]
fy_r_re_all = [x for cid in stable_ids for x in per_corner[cid]["FyR_reanchor_nearzero_pool"]]
print(f"  near-zero-alpha_r sample count: washout n={n_r_wash_total}  -> re-anchored n={n_r_re_total}  "
      f"({'thins' if n_r_re_total < n_r_wash_total else 'does not thin'})")
if fy_r_wash_all:
    fy_r_wash_global_med = float(np.median(np.concatenate(fy_r_wash_all)))
    print(f"  washout GLOBAL median Fy_r at near-zero-alpha_r={fy_r_wash_global_med:.0f} N  "
          f"(WP-S3 Metric 5 on-record global median Fy_r=6197 N -- should match, this diagnostic's own")
    print(f"  independent recomputation of the same production quantity)")
if fy_r_re_all:
    fy_r_re_global_med = float(np.median(np.concatenate(fy_r_re_all)))
    print(f"  re-anchored GLOBAL median Fy_r at near-zero-alpha_r={fy_r_re_global_med:.0f} N")
