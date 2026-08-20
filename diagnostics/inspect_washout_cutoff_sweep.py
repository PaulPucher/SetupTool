# PLAN.md unsupervised package, Phase 1: washout cutoff sweep.
# Read-only, Tier B (signal/data engineering diagnostic -- standard
# high-pass parameter sweep, no new vehicle-dynamics claim). No
# production/config change; nothing here is whitelisted or called from
# any pipeline/UI path. Pre-registration, design rationale and the
# disqualifying drift bound: thesis_notes.md "Phase 1: washout cutoff
# sweep -- pre-registration".
#
# Two distinct beta constructions per cutoff (see pre-registration for
# the full justification):
#   1. GLOBAL -- production's own estimate_sideslip formula
#      (cumsum(beta_dot)*dt, then _highpass_filter at the swept
#      cutoff), run over the whole session. cutoff=0 skips the filter
#      step (scipy.signal.butter rejects a literal 0 Hz critical
#      frequency) and returns the raw accumulated integral. Feeds
#      metrics 1 (mid-corner recovery), 3 (sign check), 4 (EKF
#      correlation/RMS).
#   2. LOCAL RE-ANCHORED -- diagnostics/inspect_washout_mechanism.py's
#      WP-S3c Section 3 construction, reused verbatim (same anchor-
#      finding helpers, same straight-line mask), generalised to apply
#      _highpass_filter at the swept cutoff to the local re-anchored
#      segment instead of always skipping it. Feeds metric 2 (drift)
#      only. At cutoff=0 this is bit-identical to WP-S3c's own
#      construction -- the reproduction check below depends on that.

import numpy as np
from scipy.stats import pearsonr

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, _highpass_filter,
    _butterworth_lowpass,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
CUTOFFS = [0.05, 0.03, 0.02, 0.01, 0.005, 0.0]
NEAR_ZERO_SLIP_DEG = 0.2  # matches WP-S3b/S3c/S4b's own near-zero-alpha_r population
MIN_FILT_LEN = 30  # WP-S3c's own conservative floor above filtfilt's default padlen

# Pre-registered disqualifying bound (thesis_notes.md): median drift >=
# DRIFT_MEDIAN_BOUND_DEG or p90 drift >= DRIFT_P90_BOUND_DEG disqualifies.
DRIFT_MEDIAN_BOUND_DEG = 0.9
DRIFT_P90_BOUND_DEG = 5.8

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

with open("config/channels.json", "r", encoding="utf-8") as f:
    import json
    channels_json = json.load(f)
LOW_SPEED_MAX_KMH = channels_json["corner_speed_thresholds"]["low_max"]

t = state["time"]
sr = state["sample_rate_hz"]
dt = 1.0 / sr
v = state["v_mps"]
v_kmh = v * 3.6
yaw_rate = state["yaw_rate_radps"]
delta_f = state["delta_f_rad"]
ay = state["ay_mps2"]
s_m = state.get("s_m")
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
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

apex_half_window = params["stability_estimation"]["apex_half_window_samples"]
vp = params["vehicle"]
a_cog, b_cog = vp["cog_to_front_axle_m"], vp["cog_to_rear_axle_m"]
wheelbase = vp["wheelbase_m"]
mass_kg = vp["mass_kg"]


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


# --- Classify racing-speed vs low-speed corners: corner_analysis.py's
# own canonical speed_class field (median apex speed vs low_max/
# medium_max, assigned once per stable_corner_id) -- the same field
# feeding corner_summary["speed_class"], not a re-derivation from the
# bracket-window median (which would include the braking zone and is
# NOT what "racing-speed corner" has meant elsewhere in this arc).

racing_ids, low_speed_ids = [], []
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    canon_class = instances[0].get("speed_class")
    (low_speed_ids if canon_class == "low" else racing_ids).append(cid)

print("=" * 100)
print("Phase 1 -- washout cutoff sweep")
print("=" * 100)
print(f"racing-speed corners (median v >= {LOW_SPEED_MAX_KMH} km/h): {racing_ids} (n={len(racing_ids)})")
print(f"low-speed corners: {low_speed_ids} (n={len(low_speed_ids)})")
print()

# --- apex-phase mask restricted to racing-speed corners ---------------

apex_mask_racing = np.zeros_like(t, dtype=bool)
for c in corners:
    if c.get("stable_corner_id") not in racing_ids:
        continue
    start_t, end_t = c["segments"]["apex_3"]
    if end_t < start_t:
        continue
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window)
        hi = min(len(t), centre + apex_half_window + 1)
    apex_mask_racing[lo:hi] = True
apex_pop_mask = base_mask & apex_mask_racing
print(f"apex-phase population, racing-speed corners only: n={int(apex_pop_mask.sum())}")
print()

# --- force-balance demand, restricted to racing-speed corners (WP-S3b/
# S3c methodology, kinematic candidate, informative reference only -- Cr
# is alpha-derived, so this is not an independent magnitude check) ----

beta_kinematic_raw = np.where(moving_raw, ay / np.where(moving_raw, v, 1.0) - yaw_rate, 0.0)
beta_kinematic = _highpass_filter(np.cumsum(beta_kinematic_raw) * dt,
                                   params["stability_estimation"]["beta_washout_cutoff_hz"], sr)
beta_kinematic = np.where(moving_raw, beta_kinematic, 0.0)
slip_kin = estimate_slip_angles(state, beta_kinematic, params)
forces = estimate_lateral_forces(state, params)  # beta-independent, reused across all cutoffs
cs_kin = estimate_cornering_stiffness(slip_kin, forces, state, params)
Fy_r_needed_full = mass_kg * ay * a_cog / wheelbase
near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)

fb_demands = []
for cid in racing_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    pooled_ss = []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop <= sl.start:
            continue
        m = moving[sl] & np.isfinite(slip_kin["alpha_r_filt"][sl]) & (np.abs(slip_kin["alpha_r_filt"][sl]) < near_zero_rad)
        Cr = cs_kin["C_linear_ref_r"][sl]
        valid = m & np.isfinite(Cr) & (Cr > 0)
        if valid.any():
            pooled_ss.append(Fy_r_needed_full[sl][valid] / Cr[valid])
    if pooled_ss:
        fb_demands.append(float(np.degrees(np.median(np.concatenate(pooled_ss)))))

if fb_demands:
    print(f"force-balance demand alpha_r_ss, racing-speed corners only, recomputed here: "
          f"min={min(fb_demands):.2f}  max={max(fb_demands):.2f} deg  "
          f"(WP-S3b established figure across all 14 corners: 0.9-5.8 deg)")
print()


def global_beta(cutoff):
    if cutoff <= 0.0:
        beta_raw = np.cumsum(beta_kinematic_raw) * dt
        return np.where(moving_raw, beta_raw, 0.0)
    filt = _highpass_filter(np.cumsum(beta_kinematic_raw) * dt, cutoff, sr)
    return np.where(moving_raw, filt, 0.0)


# --- pass-1 EKF beta, fixed reference across the sweep ----------------

ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
beta_ekf = ekf_result["beta"]

# --- straight-line anchor mask + local re-anchor helpers (WP-S3c, reused verbatim) --

ay_g = ay / 9.81
yaw_rate_degps = np.degrees(yaw_rate)
straight_mask = moving_raw & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)


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


def drift_for_cutoff(cutoff):
    """WP-S3c Section 3 construction, generalised: local re-anchored raw
    integral, then _highpass_filter at `cutoff` (skipped at cutoff=0).
    Returns |beta| in degrees at the first straight-line sample after
    corner exit, one value per (corner, valid-lap instance) that has
    both a before- and after-corner straight-line anchor.
    """
    drift_records = []
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
            if sl.stop <= sl.start:
                continue
            lap_lo = int(np.searchsorted(t, lap["start_time"], side="left"))
            lap_hi = int(np.searchsorted(t, lap["end_time"], side="right"))
            i_anchor = _find_anchor_before(lap_lo, sl.start)
            if i_anchor is None:
                continue
            i_exit = _find_anchor_after(sl.stop, lap_hi)
            if i_exit is None:
                continue
            seg_end = i_exit + 1
            seg_len = seg_end - i_anchor
            local_raw = np.cumsum(beta_kinematic_raw[i_anchor:seg_end]) * dt
            if cutoff <= 0.0 or seg_len < MIN_FILT_LEN:
                beta_local = local_raw
            else:
                try:
                    beta_local = _highpass_filter(local_raw, cutoff, sr)
                except ValueError:
                    beta_local = local_raw
            drift_records.append(abs(float(beta_local[i_exit - i_anchor])))
    return np.array(drift_records)


# --- sweep --------------------------------------------------------------

results = {}
for cutoff in CUTOFFS:
    beta_g = global_beta(cutoff)

    # metric 1: mid-corner recovery
    beta_apex_deg = np.abs(np.degrees(beta_g[apex_pop_mask]))
    ekf_apex_deg = np.abs(np.degrees(beta_ekf[apex_pop_mask]))
    m1 = {
        "median_beta": float(np.median(beta_apex_deg)),
        "p90_beta": float(np.percentile(beta_apex_deg, 90)),
        "median_ekf": float(np.median(ekf_apex_deg)),
        "p90_ekf": float(np.percentile(ekf_apex_deg, 90)),
    }

    # metric 2: drift (local re-anchored construction)
    drift_arr = drift_for_cutoff(cutoff)
    drift_deg = np.degrees(drift_arr)
    m2 = {
        "n": len(drift_arr),
        "median": float(np.median(drift_deg)) if len(drift_deg) else float("nan"),
        "mean": float(np.mean(drift_deg)) if len(drift_deg) else float("nan"),
        "p90": float(np.percentile(drift_deg, 90)) if len(drift_deg) else float("nan"),
        "max": float(np.max(drift_deg)) if len(drift_deg) else float("nan"),
    }

    # metric 3: sign check (median gate + per-sample pooled fraction, racing-speed)
    n_match_median = n_total = 0
    n_match_median_racing = n_racing = 0
    per_num = per_den = 0
    beta_deg_full = np.degrees(beta_g)
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
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
            pooled_beta.append(beta_deg_full[sl][m])
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
    m3 = {
        "median_gate_all": f"{n_match_median}/{n_total}",
        "median_gate_racing": f"{n_match_median_racing}/{n_racing}",
        "per_sample_fraction_racing": (per_num / per_den) if per_den else float("nan"),
    }

    # metric 4: correlation and RMS vs pass-1 EKF beta, base_mask population
    beta_pop = beta_g[base_mask]
    ekf_pop = beta_ekf[base_mask]
    corr = float(pearsonr(beta_pop, ekf_pop)[0])
    rms_deg = float(np.sqrt(np.mean((np.degrees(beta_pop) - np.degrees(ekf_pop)) ** 2)))
    m4 = {"correlation": corr, "rms_diff_deg": rms_deg}

    disqualified = (m2["median"] >= DRIFT_MEDIAN_BOUND_DEG) or (m2["p90"] >= DRIFT_P90_BOUND_DEG)

    results[cutoff] = {"m1": m1, "m2": m2, "m3": m3, "m4": m4, "disqualified": disqualified}

    print(f"--- cutoff = {cutoff} Hz {'(no filter)' if cutoff <= 0 else ''} ---")
    print(f"  metric 1 (apex |beta|, racing corners): median={m1['median_beta']:.3f} deg  "
          f"p90={m1['p90_beta']:.3f} deg   |  EKF median={m1['median_ekf']:.3f} deg  p90={m1['p90_ekf']:.3f} deg")
    print(f"  metric 2 (post-corner drift, all corners): n={m2['n']}  median={m2['median']:.3f} deg  "
          f"mean={m2['mean']:.3f} deg  p90={m2['p90']:.3f} deg  max={m2['max']:.3f} deg")
    print(f"  metric 3 (sign check): median gate all={m3['median_gate_all']}  "
          f"racing={m3['median_gate_racing']}  per-sample pooled fraction (racing)={m3['per_sample_fraction_racing']:.4f}")
    print(f"  metric 4 (vs pass-1 EKF beta, base_mask n={int(base_mask.sum())}): "
          f"corr={m4['correlation']:.4f}  RMS diff={m4['rms_diff_deg']:.3f} deg")
    print(f"  DISQUALIFIED (median>={DRIFT_MEDIAN_BOUND_DEG} or p90>={DRIFT_P90_BOUND_DEG} deg drift): {disqualified}")
    print()

# --- reproduction check ---------------------------------------------------

print("=" * 100)
print("REPRODUCTION CHECK -- cutoff=0 vs WP-S3c recorded figures (median=5.70, p90=10.80 deg)")
print("=" * 100)
m2_zero = results[0.0]["m2"]
med_ok = abs(m2_zero["median"] - 5.7) < 0.05
p90_ok = abs(m2_zero["p90"] - 10.8) < 0.1
print(f"  this run: median={m2_zero['median']:.3f} deg  p90={m2_zero['p90']:.3f} deg")
print(f"  median match (tol 0.05 deg): {med_ok}   p90 match (tol 0.1 deg): {p90_ok}")
print(f"  VERDICT: {'PASS -- sweep trusted' if (med_ok and p90_ok) else 'FAIL -- STOP, investigate before trusting the sweep'}")
print()

# --- verdict: does any cutoff dominate 0.05? -------------------------------

print("=" * 100)
print("VERDICT -- does any non-disqualified cutoff dominate 0.05 on mid-corner recovery?")
print("=" * 100)
baseline = results[0.05]
print(f"  0.05 Hz (current production): apex median |beta|={baseline['m1']['median_beta']:.3f} deg  "
      f"drift median={baseline['m2']['median']:.3f}  p90={baseline['m2']['p90']:.3f} deg  "
      f"disqualified={baseline['disqualified']}")
dominant = []
for cutoff in CUTOFFS:
    if cutoff == 0.05:
        continue
    r = results[cutoff]
    if r["disqualified"]:
        continue
    better_recovery = r["m1"]["median_beta"] > baseline["m1"]["median_beta"]
    if better_recovery:
        dominant.append(cutoff)
        print(f"  {cutoff} Hz: NOT disqualified, apex median |beta|={r['m1']['median_beta']:.3f} deg "
              f"(> baseline) -- candidate dominant cutoff")
if not dominant:
    print("  NONE -- no swept cutoff both survives the disqualifying drift bound and improves mid-corner "
          "recovery over 0.05 Hz. Legitimate finding per pre-registration.")
