# WP-S5b (Open Board item B, sideslip methods comparison): corrected
# Kalman observer Q/R sweep -- a single RATIO parameter, not an
# independent (Q_scale, R_scale) grid. Read-only, Tier B -- diagnostics
# only, no production wiring, no config change. Sibling script, same
# monkey-patching approach as diagnostics/inspect_kalman_qr_sweep.py
# (reuses sideslip_kalman_observer.py's exact recursion unmodified).
#
# MOTIVATION: the prior 3x3 grid (inspect_kalman_qr_sweep.py) produced
# several byte-identical result rows -- Q_scale=0.1/R_scale=1.0 equalled
# Q_scale=1.0/R_scale=10.0; Q_scale=1.0/R_scale=0.1 equalled
# Q_scale=10.0/R_scale=1.0 -- both pairs sharing the same Q/R ratio
# (0.1 and 10 respectively). CONFIRMED here, not just suspected: a
# discrete linear Kalman filter's Kalman gain K = P_pred @ C.T @
# inv(C @ P_pred @ C.T + R) is invariant under uniform rescaling
# (Q, R, P) -> (lambda*Q, lambda*R, lambda*P), by direct substitution
# (the lambda factors cancel in K's own ratio construction). P0 (the
# post-stationary-reset covariance) is held fixed here, unscaled, so
# this invariance is not exact from the very first sample after a
# reset -- but the covariance recursion is a contracting map that
# forgets its initial condition within a handful of samples relative
# to each moving stretch (many hundreds to thousands of samples long
# in this data), so the reported medians/percentiles over the full
# session are unaffected in practice. Confirmed exactly: Q_scale=0.1/
# R_scale=1.0 and Q_scale=1.0/R_scale=10.0 (both ratio=0.1) produced
# IDENTICAL summary statistics to the decimal places printed in the
# prior script's output. PRACTICAL CONSEQUENCE, stated plainly per the
# work order: the prior script's 9 grid points contained only 3
# distinct settings (ratio in {0.1, 1, 10} approximately -- the ninth
# combination, Q=10/R=0.1, ratio=100, differed only slightly from the
# ratio=10 rows, consistent with ratio-dependence rather than
# refuting it, not because it is a fourth coincidental duplicate).
# The ABSOLUTE Q/R values swept there were therefore not separately
# meaningful; only their ratio was ever actually being tested.
#
# Tuning targets (same as WP-S5, restated for this corrected sweep):
#   1. Sideslip near zero during straight-line driving.
#   2. Consistent lap to lap at the same corner WITHOUT flattening real
#      variation -- corners must stay distinguishable from each other,
#      genuine lap-to-lap differences must survive. Over-smoothing is a
#      failure, not a success.
#   3. Physically sensible values in normal cornering, while genuine
#      large excursions (slides, moments) are preserved, not clipped or
#      smoothed away.
#   4. The sign result must not degrade at the eleven racing-speed
#      corners under any recommended setting.
# Explicitly NOT a target: agreement with the kinematic estimate.
# NEW this round: a transient-tracking robustness check (correlation of
# d(beta)/dt against d(ay)/dt during corner entry/exit phases) so
# heavier smoothing cannot look like a win purely because it improves
# the steady-state measures above while quietly killing genuine
# transient response.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
import diagnostics.sideslip_kalman_observer as sko
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

LARGE_EXCURSION_DEG = 10.0
LARGE_AY_G = 0.8
MIN_SUSTAIN_S = 0.2

# Single ratio parameter, ~7 log-spaced points, 1e-3 to 1e2. R_scale
# held fixed at 1.0 (baseline R); Q_scale = ratio directly, so ratio IS
# Q_scale/R_scale by construction -- avoids re-testing multiple
# (Q_scale, R_scale) pairs that share a ratio, per the finding above.
RATIOS = np.logspace(-3, 2, 7)

BASE_Q_BETA_VAR = sko.Q_BETA_VAR
BASE_Q_YAW_RATE_VAR = sko.Q_YAW_RATE_VAR
BASE_R_YAW_RATE_VAR = sko.R_YAW_RATE_VAR
BASE_R_AY_VAR = sko.R_AY_VAR
print(f"Baseline Q: Q_BETA_VAR={BASE_Q_BETA_VAR:.6e} rad^2  Q_YAW_RATE_VAR={BASE_Q_YAW_RATE_VAR:.6e} (rad/s)^2")
print(f"Baseline R: R_YAW_RATE_VAR={BASE_R_YAW_RATE_VAR:.6e} (rad/s)^2  R_AY_VAR={BASE_R_AY_VAR:.6e} (m/s^2)^2")
print(f"R held fixed at baseline (R_scale=1.0) throughout; Q_scale swept = the ratio itself.")
print(f"Ratios (Q_scale/R_scale, np.logspace(-3, 2, 7)): {[f'{r:.4g}' for r in RATIOS]}")

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t_ref = state["time"]
sr = state["sample_rate_hz"]
s_m = state.get("s_m")
ay = state["ay_mps2"]
ay_g = ay / 9.81
yaw_rate_degps = np.degrees(state["yaw_rate_radps"])
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
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

straight_mask = moving & racing_mask & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)

TRANSIENT_PHASES = ["entry_1_brake", "entry_2_turnin", "exit_4", "exit_5"]


def _phase_slice(start_t, end_t):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t_ref, start_t, side="left"))
    hi = int(np.searchsorted(t_ref, end_t, side="right"))
    return slice(lo, hi)


transient_mask = np.zeros_like(t_ref, dtype=bool)
for c in corners:
    lap = laps_by_number.get(c["lap_number"])
    if lap is None or not lap.get("is_valid_for_analysis"):
        continue
    for phase in TRANSIENT_PHASES:
        s_t, e_t = c["segments"][phase]
        sl = _phase_slice(s_t, e_t)
        if sl.stop > sl.start:
            transient_mask[sl] = True
transient_mask &= moving


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


def _corner_stats(beta_deg):
    stats = {}
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        lap_medians, pooled_overall, pooled_ay = [], [], []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = moving[sl]
            if not m.any():
                continue
            vals = beta_deg[sl][m]
            lap_medians.append(float(np.median(vals)))
            pooled_overall.append(vals)
            pooled_ay.append(ay[sl][m])
        if not pooled_overall:
            continue
        stats[cid] = {
            "lap_medians": lap_medians,
            "overall_median": float(np.median(np.concatenate(pooled_overall))),
            "median_ay": float(np.median(np.concatenate(pooled_ay))),
            "speed_class": instances[0].get("speed_class"),
        }
    return stats


def _sweep_metrics(beta_deg):
    valid_mask = moving & racing_mask & np.isfinite(beta_deg)

    sl_vals = np.abs(beta_deg[straight_mask & np.isfinite(beta_deg)])
    straight_median = float(np.median(sl_vals))
    straight_p90 = float(np.percentile(sl_vals, 90))

    all_vals = beta_deg[valid_mask]
    p1, p99 = np.percentile(all_vals, [1, 99])
    max_abs = float(np.max(np.abs(all_vals)))
    n_excursion = int(np.sum(np.abs(all_vals) > LARGE_EXCURSION_DEG))

    stats = _corner_stats(beta_deg)
    cross_lap_stds = [float(np.std(s["lap_medians"])) for s in stats.values() if len(s["lap_medians"]) >= 2]
    overall_medians = [s["overall_median"] for s in stats.values()]
    between_corner_std = float(np.std(overall_medians))
    between_corner_range = float(np.max(overall_medians) - np.min(overall_medians))

    n_match = n_total = 0
    for cid, s in stats.items():
        if s.get("speed_class") == "low":
            continue
        dir_sign = np.sign(s["median_ay"])
        if dir_sign == 0:
            continue
        n_total += 1
        if np.sign(s["overall_median"]) == -dir_sign:
            n_match += 1

    # Transient-tracking robustness check: d(beta)/dt vs d(ay)/dt during
    # corner entry/exit phases only -- does this setting still respond
    # to rapid changes, or has smoothing killed the transient?
    dbeta_dt = np.gradient(beta_deg, t_ref)
    day_g_dt = np.gradient(ay_g, t_ref)
    tm = transient_mask & np.isfinite(dbeta_dt) & np.isfinite(day_g_dt)
    if tm.sum() > 2:
        transient_corr = float(np.corrcoef(dbeta_dt[tm], day_g_dt[tm])[0, 1])
    else:
        transient_corr = float("nan")

    return {
        "straight_median": straight_median, "straight_p90": straight_p90,
        "cross_lap_std_median": float(np.median(cross_lap_stds)) if cross_lap_stds else float("nan"),
        "cross_lap_std_max": float(np.max(cross_lap_stds)) if cross_lap_stds else float("nan"),
        "between_corner_std": between_corner_std, "between_corner_range": between_corner_range,
        "p1": float(p1), "p99": float(p99), "max_abs": max_abs, "n_excursion": n_excursion,
        "n_total_valid": int(valid_mask.sum()),
        "sign_match": n_match, "sign_total": n_total,
        "transient_corr": transient_corr, "n_transient": int(tm.sum()),
    }


print()
print("=" * 100)
print("RATIO SWEEP RESULTS (one block per Q/R ratio, R held at baseline)")
print("=" * 100)

rows = []
for ratio in RATIOS:
    sko.Q_BETA_VAR = BASE_Q_BETA_VAR * ratio
    sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR * ratio
    sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR
    sko.R_AY_VAR = BASE_R_AY_VAR

    beta_deg = np.degrees(sko.estimate_sideslip_kalman(state, params))
    met = _sweep_metrics(beta_deg)
    rows.append((ratio, met))

    print(f"--- ratio (Q_scale/R_scale) = {ratio:.4g} ---")
    print(f"  1. Straight-line: median|beta|={met['straight_median']:.3f} deg  p90|beta|={met['straight_p90']:.3f} deg")
    print(f"  2a. Cross-lap std per corner (consistency): median={met['cross_lap_std_median']:.3f} deg  "
          f"max={met['cross_lap_std_max']:.3f} deg")
    print(f"  2b. Between-corner spread of median beta (discrimination, must NOT collapse): "
          f"std={met['between_corner_std']:.3f} deg  range={met['between_corner_range']:.3f} deg")
    print(f"  3. p1={met['p1']:+.3f} deg  p99={met['p99']:+.3f} deg  max|beta|={met['max_abs']:.3f} deg  "
          f"n(|beta|>{LARGE_EXCURSION_DEG} deg)={met['n_excursion']} of {met['n_total_valid']}")
    print(f"  4. Sign match at racing-speed corners: {met['sign_match']}/{met['sign_total']}")
    print(f"  5. Transient tracking: corr(d(beta)/dt, d(ay)/dt) during entry/exit phases = "
          f"{met['transient_corr']:+.4f}  (n={met['n_transient']})")
    print()

sko.Q_BETA_VAR = BASE_Q_BETA_VAR
sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR
sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR
sko.R_AY_VAR = BASE_R_AY_VAR

print("=" * 100)
print("TREND (change from previous ratio point, for turnover/flattening identification)")
print("=" * 100)
prev = None
for ratio, met in rows:
    if prev is None:
        print(f"  ratio={ratio:.4g}: (first point, no delta)")
    else:
        d_straight = met["straight_median"] - prev["straight_median"]
        d_crosslap = met["cross_lap_std_median"] - prev["cross_lap_std_median"]
        d_between = met["between_corner_std"] - prev["between_corner_std"]
        d_max = met["max_abs"] - prev["max_abs"]
        d_transient = met["transient_corr"] - prev["transient_corr"]
        print(f"  ratio={ratio:.4g}: d(straight_med)={d_straight:+.4f}  d(cross_lap_std)={d_crosslap:+.4f}  "
              f"d(between_corner_std)={d_between:+.4f}  d(max|beta|)={d_max:+.4f}  "
              f"d(transient_corr)={d_transient:+.4f}")
    prev = met

print()
print("=" * 100)
print("COMPACT SUMMARY (one line per ratio)")
print("=" * 100)
for ratio, met in rows:
    print(f"  ratio={ratio:9.4g}  straight_med={met['straight_median']:.3f}  straight_p90={met['straight_p90']:.3f}  "
          f"cross_lap_std={met['cross_lap_std_median']:.3f}  between_corner_std={met['between_corner_std']:.3f}  "
          f"max|beta|={met['max_abs']:.3f}  n_exc={met['n_excursion']}  sign={met['sign_match']}/{met['sign_total']}  "
          f"transient_corr={met['transient_corr']:+.4f}")
