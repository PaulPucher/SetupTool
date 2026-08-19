# WP-S5 (Open Board item B, sideslip methods comparison): Kalman
# observer Q/R tuning sweep. Read-only, Tier B -- diagnostics only, no
# production wiring, no config change. Sibling script rather than a
# sideslip_kalman_observer.py extension: sweeping Q/R by monkey-
# patching that module's own named constants per grid point reuses its
# exact recursion unmodified, without cluttering the observer file
# (kept a single clean hand-tuned candidate) with sweep-only plumbing.
#
# SUPERSEDED METHODOLOGY, kept for the record (WP-S5b, thesis_notes.md
# dated entry): this script's 3x3 (Q_scale, R_scale) grid contains only
# 3 DISTINCT settings, not 9 -- a linear Kalman filter's steady-state
# gain depends only on the Q/R RATIO (confirmed exactly: several grid
# points here are byte-identical, e.g. Q=0.1/R=1.0 == Q=1.0/R=10.0,
# both ratio=0.1). The corrected ratio sweep is diagnostics/inspect_
# kalman_qr_ratio_sweep.py. Left unmodified rather than deleted; its
# own numbers are still individually correct, just redundant across
# the grid.
#
# Context: WP-S5's sign check (thesis_notes.md, same-dated entry)
# already validated the observer's SIGN at all 14 corners (11 of them
# racing-speed, matching the physical bicycle-model expectation there;
# the other 3 are this dataset's only low-speed-class corners, where a
# sign reversal is separately expected and not itself a defect). This
# sweep is refinement of a working method, not repair.
#
# Tuning targets (agreed in advance, recorded here so the result cannot
# be judged against a preferred answer):
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
# Explicitly NOT a target: agreement with the kinematic estimate (the
# method shown to under-read and carry wrong signs mid-corner).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
import diagnostics.sideslip_kalman_observer as sko
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

# Diagnostic-local thresholds (report-only, gate nothing downstream),
# same convention as every other diagnostics/ script in this WP.
LARGE_EXCURSION_DEG = 10.0   # "genuine slide/moment" level -- roughly half BETA_SANE_BOUND_DEG (20 deg, the harness's own generous spin bound)
LARGE_AY_G = 0.8             # near-tyre-limit lateral g for a GT3 car, for the sustained-event check
MIN_SUSTAIN_S = 0.2          # minimum contiguous duration to count as a genuine event, not a noise spike

Q_SCALES = [0.1, 1.0, 10.0]
R_SCALES = [0.1, 1.0, 10.0]

# Baseline (current hand-tuned) values, captured before any monkey-patching.
BASE_Q_BETA_VAR = sko.Q_BETA_VAR
BASE_Q_YAW_RATE_VAR = sko.Q_YAW_RATE_VAR
BASE_R_YAW_RATE_VAR = sko.R_YAW_RATE_VAR
BASE_R_AY_VAR = sko.R_AY_VAR
print(f"Baseline Q: Q_BETA_VAR={BASE_Q_BETA_VAR:.6e} rad^2  Q_YAW_RATE_VAR={BASE_Q_YAW_RATE_VAR:.6e} (rad/s)^2")
print(f"Baseline R: R_YAW_RATE_VAR={BASE_R_YAW_RATE_VAR:.6e} (rad/s)^2  R_AY_VAR={BASE_R_AY_VAR:.6e} (m/s^2)^2")
print(f"P0 (initial post-reset covariance) held fixed throughout -- not a sweep target, only affects the")
print(f"brief transient right after each stationary reset, not the steady-state behaviour the targets test.")
print(f"Grid: Q_scale in {Q_SCALES} x R_scale in {R_SCALES} (9 combinations), Q_BETA_VAR/Q_YAW_RATE_VAR")
print(f"scaled together (Q_scale), R_YAW_RATE_VAR/R_AY_VAR scaled together (R_scale) -- preserves each")
print(f"pair's relative weighting, sweeps the model-trust-vs-sensor-trust balance the two knobs represent.")

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

    return {
        "straight_median": straight_median, "straight_p90": straight_p90,
        "cross_lap_std_median": float(np.median(cross_lap_stds)) if cross_lap_stds else float("nan"),
        "cross_lap_std_max": float(np.max(cross_lap_stds)) if cross_lap_stds else float("nan"),
        "between_corner_std": between_corner_std, "between_corner_range": between_corner_range,
        "p1": float(p1), "p99": float(p99), "max_abs": max_abs, "n_excursion": n_excursion,
        "n_total_valid": int(valid_mask.sum()),
        "sign_match": n_match, "sign_total": n_total,
    }


print()
print("=" * 100)
print("SWEEP RESULTS (one block per Q_scale x R_scale combination)")
print("=" * 100)

summary_rows = []
for q_scale in Q_SCALES:
    for r_scale in R_SCALES:
        sko.Q_BETA_VAR = BASE_Q_BETA_VAR * q_scale
        sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR * q_scale
        sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR * r_scale
        sko.R_AY_VAR = BASE_R_AY_VAR * r_scale

        beta_deg = np.degrees(sko.estimate_sideslip_kalman(state, params))
        met = _sweep_metrics(beta_deg)
        summary_rows.append((q_scale, r_scale, met))

        print(f"--- Q_scale={q_scale:5.2f}  R_scale={r_scale:5.2f} ---")
        print(f"  1. Straight-line: median|beta|={met['straight_median']:.3f} deg  p90|beta|={met['straight_p90']:.3f} deg")
        print(f"  2a. Cross-lap std per corner (consistency): median={met['cross_lap_std_median']:.3f} deg  "
              f"max={met['cross_lap_std_max']:.3f} deg")
        print(f"  2b. Between-corner spread of median beta (discrimination, must NOT collapse): "
              f"std={met['between_corner_std']:.3f} deg  range={met['between_corner_range']:.3f} deg")
        print(f"  3. p1={met['p1']:+.3f} deg  p99={met['p99']:+.3f} deg  max|beta|={met['max_abs']:.3f} deg  "
              f"n(|beta|>{LARGE_EXCURSION_DEG} deg)={met['n_excursion']} of {met['n_total_valid']}")
        print(f"  4. Sign match at racing-speed corners: {met['sign_match']}/{met['sign_total']}")
        print()

# Restore baseline before the sustained-event check and script exit.
sko.Q_BETA_VAR = BASE_Q_BETA_VAR
sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR
sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR
sko.R_AY_VAR = BASE_R_AY_VAR

print("=" * 100)
print("COMPACT SUMMARY (one line per combination)")
print("=" * 100)
for q_scale, r_scale, met in summary_rows:
    print(f"  Q={q_scale:5.2f} R={r_scale:5.2f}  straight_med={met['straight_median']:.3f}  "
          f"straight_p90={met['straight_p90']:.3f}  cross_lap_std_med={met['cross_lap_std_median']:.3f}  "
          f"between_corner_std={met['between_corner_std']:.3f}  max|beta|={met['max_abs']:.3f}  "
          f"n_excursion={met['n_excursion']}  sign={met['sign_match']}/{met['sign_total']}")

# --- Task 3: does Dubai contain a genuine sustained large-excursion event? ---

print()
print("=" * 100)
print(f"SUSTAINED LARGE-EXCURSION EVENT CHECK (baseline Q/R): |beta|>{LARGE_EXCURSION_DEG} deg AND "
      f"|ay|>{LARGE_AY_G} g, sustained >= {MIN_SUSTAIN_S} s (not a single-sample noise spike)")
print("=" * 100)

beta_baseline_deg = np.degrees(sko.estimate_sideslip_kalman(state, params))
valid_mask = moving & racing_mask & np.isfinite(beta_baseline_deg)
event_mask = valid_mask & (np.abs(beta_baseline_deg) > LARGE_EXCURSION_DEG) & (np.abs(ay_g) > LARGE_AY_G)
min_sustain_samples = int(round(MIN_SUSTAIN_S * sr))

runs = []
in_run = False
run_start = 0
for i in range(len(event_mask)):
    if event_mask[i] and not in_run:
        in_run = True
        run_start = i
    elif not event_mask[i] and in_run:
        in_run = False
        if i - run_start >= min_sustain_samples:
            runs.append((run_start, i))
if in_run and len(event_mask) - run_start >= min_sustain_samples:
    runs.append((run_start, len(event_mask)))

if runs:
    print(f"  FOUND {len(runs)} sustained event(s):")
    for a, b in runs:
        dur_s = (b - a) / sr
        peak_beta = float(np.max(np.abs(beta_baseline_deg[a:b])))
        peak_ay = float(np.max(np.abs(ay_g[a:b])))
        print(f"    t=[{t_ref[a]:.2f}, {t_ref[b-1]:.2f}] s  duration={dur_s:.2f} s  "
              f"peak|beta|={peak_beta:.2f} deg  peak|ay|={peak_ay:.2f} g")
else:
    print("  NONE FOUND. The Dubai sample contains no sustained (>=0.2 s) window where the observer's own")
    print("  |beta| exceeds 10 deg simultaneously with |ay| exceeding 0.8 g. Target 3's large-excursion-")
    print("  preservation criterion therefore CANNOT be verified against a genuine event on this data --")
    print("  documented as a limitation, to re-check when a session with a real slide/moment arrives.")
    n_beta_only = int(np.sum(valid_mask & (np.abs(beta_baseline_deg) > LARGE_EXCURSION_DEG)))
    n_ay_only = int(np.sum(valid_mask & (np.abs(ay_g) > LARGE_AY_G)))
    print(f"  For context: {n_beta_only} samples exceed |beta|>{LARGE_EXCURSION_DEG} deg alone (isolated, not")
    print(f"  sustained-with-high-ay); {n_ay_only} samples exceed |ay|>{LARGE_AY_G} g alone (normal hard")
    print(f"  cornering, this GT3 car's typical peak lateral g on this session).")
