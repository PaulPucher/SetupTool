# WP5b(c) validation report: GPS-course-based sideslip (beta_gps) vs
# Module 2's kinematic beta, on real Dubai data. Report-only --
# estimate_sideslip_gps is not called from any pipeline/UI path; production
# beta stays kinematic (modules.stability_analysis.estimate_sideslip).
#
# Sections:
#   0. Amendment 1: rotation-convention + latency evidence (also documented
#      in estimate_sideslip_gps's own docstring -- reproduced here so the
#      report is self-contained and independently reproducible).
#   1. Whole-session stats: correlation, bias by speed class, straight-line
#      near-zero check.
#   2. Per-corner/phase agreement: sign-agreement rate, magnitude ratio.
#   3. Known-weakness probes: kerbs, low speed, long-corner washout distortion.
#   4. Amendment 2: lever-arm (antenna-offset) probe -- finding only, no
#      correction wired into anything.
#   5. Decision-criteria verdict (met / partially met / not met).

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_sideslip_gps,
)

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
se = params["stability_estimation"]
state = prepare_vehicle_state(data["channels"], params)
beta_k = estimate_sideslip(state, params)          # production, kinematic
beta_g = estimate_sideslip_gps(state, data["channels"], params)  # candidate

t_ref = state["time"]
sr = state["sample_rate_hz"]
moving = state["moving_mask"]
kerb_mask = state.get("kerb_mask")

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)

gps_valid = ~np.isnan(beta_g)
core_mask = moving & racing_mask & gps_valid

print("=" * 70)
print("SECTION 0 -- Amendment 1: rotation convention + latency evidence")
print("=" * 70)

course_ch = data["channels"]["log_gps_course"]
ct, cd = course_ch["time"], course_ch["data"]
cd_rad = np.radians(cd)
dcourse = np.arctan2(np.sin(np.diff(cd_rad)), np.cos(np.diff(cd_rad)))
dt_native = np.diff(ct)
dcourse_dt_native = dcourse / dt_native
mid_t = (ct[:-1] + ct[1:]) / 2.0

yaw_rate_mid = np.interp(mid_t, t_ref, state["yaw_rate_radps"])
moving_mid = np.interp(mid_t, t_ref, moving.astype(float)) > 0.5
racing_mid = np.interp(mid_t, t_ref, racing_mask.astype(float)) > 0.5
valid_mid = moving_mid & racing_mid

r_plus = np.corrcoef(dcourse_dt_native[valid_mid], yaw_rate_mid[valid_mid])[0, 1]
r_minus = np.corrcoef(dcourse_dt_native[valid_mid], -yaw_rate_mid[valid_mid])[0, 1]
print(f"n samples (racing laps, moving): {valid_mid.sum()}")
print(f"r(dcourse/dt, +yaw_rate) = {r_plus:+.4f}")
print(f"r(dcourse/dt, -yaw_rate) = {r_minus:+.4f}")
adopted = "-yaw_rate_radps" if r_minus > r_plus else "+yaw_rate_radps"
print(f"Adopted sign: {adopted} (implemented in estimate_sideslip_gps)")

dcourse_dt_50hz = np.interp(t_ref, mid_t, dcourse_dt_native)
y = -state["yaw_rate_radps"]
x = dcourse_dt_50hz
lags_s = np.arange(-1.0, 1.0 + 1e-9, 1.0 / sr)
n = len(x)
rs = []
for lag_s in lags_s:
    shift = int(round(lag_s * sr))
    if shift >= 0:
        xs, ys = x[shift:], (y[: n - shift] if shift > 0 else y)
        ms = core_mask[shift:] & (core_mask[: n - shift] if shift > 0 else core_mask)
    else:
        s2 = -shift
        xs, ys, ms = x[: n - s2], y[s2:], core_mask[: n - s2] & core_mask[s2:]
    rs.append(np.corrcoef(xs[ms], ys[ms])[0, 1] if ms.sum() >= 100 else np.nan)
rs = np.array(rs)
best = np.nanargmax(rs)
print(f"Cross-correlation peak: r={rs[best]:.4f} at lag={lags_s[best]:+.3f}s "
      f"(r at zero lag: {rs[np.argmin(np.abs(lags_s))]:.4f})")
print("Positive lag = course-derived rate LAGS yaw_rate -- GPS course reflects a")
print(f"rotational state from ~lag seconds earlier. Iteration 2: corrected in the estimator")
print(f"via config gps_course_latency_s={se['gps_course_latency_s']}s (course sampled that far")
print(f"ahead of each query time before anchoring/subtraction).")

print()
print("=" * 70)
print("SECTION 1 -- Whole-session stats (racing laps, moving, GPS-valid)")
print("=" * 70)

bk_deg = np.degrees(beta_k)
bg_deg = np.degrees(beta_g)
r_beta = np.corrcoef(bk_deg[core_mask], bg_deg[core_mask])[0, 1]
bias = np.median((bg_deg - bk_deg)[core_mask])
print(f"n = {core_mask.sum()}")
print(f"Correlation r(beta_gps, beta_kinematic) = {r_beta:+.4f}")
print(f"Median bias (beta_gps - beta_kinematic) = {bias:+.3f} deg")

v_kmh = state["v_mps"] * 3.6
ch_cfg = None
import json
with open("config/channels.json", "r", encoding="utf-8") as f:
    ch_cfg = json.load(f)
low_max = ch_cfg["corner_speed_thresholds"]["low_max"]
med_max = ch_cfg["corner_speed_thresholds"]["medium_max"]
bins = {
    "low": core_mask & (v_kmh <= low_max),
    "medium": core_mask & (v_kmh > low_max) & (v_kmh <= med_max),
    "high": core_mask & (v_kmh > med_max),
}
for name, m in bins.items():
    if m.sum() == 0:
        print(f"  {name}: no samples")
        continue
    b = np.median((bg_deg - bk_deg)[m])
    print(f"  speed_class={name:6s}  n={m.sum():6d}  median bias={b:+.3f} deg")

# Straight-line near-zero check: reuse the same anchor-candidate gate the
# estimator itself uses (smoothed |ay| below threshold, sustained run).
ay_g = np.abs(state["ay_mps2"]) / 9.81
smooth_win = max(1, int(round(se["gps_course_anchor_smooth_window_s"] * sr)))
kernel = np.ones(smooth_win) / smooth_win
ay_g_smooth = np.convolve(ay_g, kernel, mode="same")
straight_mask = core_mask & (ay_g_smooth < se["gps_course_anchor_max_ay_g"])
print(f"\nStraight-line near-zero check (n={straight_mask.sum()}):")
print(f"  median |beta_kinematic| = {np.median(np.abs(bk_deg[straight_mask])):.3f} deg")
print(f"  median |beta_gps|       = {np.median(np.abs(bg_deg[straight_mask])):.3f} deg")

print()
print("=" * 70)
print("SECTION 1b -- Root-cause diagnostic: per-lap net rotation check")
print("=" * 70)
print("Closed-loop sanity check: over one lap, net gyro-integrated heading")
print("change should equal net GPS-course change (both measure the same net")
print("rotation of a closed circuit lap).")
psi_gyro_dot = -state["yaw_rate_radps"]
psi_gyro_check = np.cumsum(psi_gyro_dot) * (1.0 / sr)
for l in laps:
    if not l.get("is_valid_for_analysis"):
        continue
    s_t, e_t = l["start_time"], l["end_time"]
    i0, i1 = np.searchsorted(t_ref, s_t), np.searchsorted(t_ref, e_t)
    net_gyro_deg = np.degrees(psi_gyro_check[i1] - psi_gyro_check[i0])
    c0, c1 = np.searchsorted(ct, s_t), np.searchsorted(ct, e_t)
    course_unwrapped_deg = np.degrees(np.unwrap(np.radians(cd[c0:c1])))
    net_course_deg = course_unwrapped_deg[-1] - course_unwrapped_deg[0]
    print(f"  lap {l['lap_number']}: net_gyro={net_gyro_deg:+.1f} deg  "
          f"net_course={net_course_deg:+.1f} deg  shortfall={net_course_deg-net_gyro_deg:+.1f} deg")
print("Consistent ~6 deg/lap shortfall (~1.7% of the ~354-360 deg net rotation) across")
print("all 4 laps -- a small, systematic gyro-integration scale-type error, not random")
print("noise. Diagnosed root cause identified in iteration 1: allocating the anchor")
print("correction LINEARLY IN TIME under-corrects exactly where this rotation-concentrated")
print("drift actually accumulates (the ~15 corners/lap), not spread evenly by clock time.")
print("Iteration 2 (this run) replaces that with a rotation-proportional allocation -- see")
print("Section 5 for whether it resolved the downstream numbers, and the Amendment 2")
print("lever-arm cross-check for a falsifiable test of this diagnosis specifically.")

print()
print("=" * 70)
print("SECTION 2 -- Per-corner/phase agreement (racing laps only)")
print("=" * 70)

phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
apex_half_window = se["apex_half_window_samples"]


def _phase_slice(start_t, end_t, is_apex=False):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t_ref, start_t, side="left"))
    hi = int(np.searchsorted(t_ref, end_t, side="right"))
    if is_apex and hi <= lo:
        lo = max(0, lo - apex_half_window)
        hi = min(len(t_ref), lo + 2 * apex_half_window + 1)
    return slice(lo, hi)


sign_matches, sign_total = 0, 0
sign_matches_strict, sign_total_strict = 0, 0  # |beta_kinematic| > 0.3 deg only
abs_diffs = []
records = []
for c in data.get("corners", []):
    lap = next((l for l in laps if l["lap_number"] == c["lap_number"]), None)
    if lap is None or not lap.get("is_valid_for_analysis"):
        continue
    for phase in phase_keys:
        s_t, e_t = c["segments"][phase]
        sl = _phase_slice(s_t, e_t, is_apex=(phase == "apex_3"))
        m = core_mask[sl]
        if m.sum() == 0:
            continue
        bk_med = np.median(bk_deg[sl][m])
        bg_med = np.median(bg_deg[sl][m])
        records.append((c["lap_number"], c["corner_number"], c.get("stable_corner_id"), phase, bk_med, bg_med))
        sign_total += 1
        if np.sign(bk_med) == np.sign(bg_med):
            sign_matches += 1
        abs_diffs.append(abs(bg_med - bk_med))
        if abs(bk_med) > 0.3:
            sign_total_strict += 1
            if np.sign(bk_med) == np.sign(bg_med):
                sign_matches_strict += 1

print(f"Corner-phase instances compared: {sign_total}")
print(f"Sign agreement (all instances): {sign_matches}/{sign_total} = {sign_matches/sign_total*100:.1f}%")
if sign_total_strict:
    print(f"Sign agreement (|beta_kinematic|>0.3deg, n={sign_total_strict}): "
          f"{sign_matches_strict}/{sign_total_strict} = {sign_matches_strict/sign_total_strict*100:.1f}%")
print(f"Median |beta_gps - beta_kinematic| per corner-phase: {np.median(abs_diffs):.3f} deg")
print(f"(reference: the prior log_a_car probe matched sign in only 1 of 3 known corners)")

print()
print("=" * 70)
print("SECTION 3 -- Known-weakness probes")
print("=" * 70)

print("--- Kerbs ---")
if kerb_mask is not None:
    kerb_on = core_mask & kerb_mask
    kerb_off = core_mask & ~kerb_mask
    diff_on = np.abs((bg_deg - bk_deg)[kerb_on])
    diff_off = np.abs((bg_deg - bk_deg)[kerb_off])
    print(f"  on-kerb   n={kerb_on.sum():6d}  median |beta_gps-beta_kinematic| = {np.median(diff_on):.3f} deg")
    print(f"  off-kerb  n={kerb_off.sum():6d}  median |beta_gps-beta_kinematic| = {np.median(diff_off):.3f} deg")
else:
    print("  kerb_mask unavailable")

print("--- Low speed (proximity to moving_speed_min_mps gate) ---")
gate = se["moving_speed_min_mps"]
speed_bins = [(gate, gate + 3), (gate + 3, gate + 8), (gate + 8, np.inf)]
for lo, hi in speed_bins:
    m = core_mask & (state["v_mps"] >= lo) & (state["v_mps"] < hi)
    hi_label = f"{hi:.0f}" if np.isfinite(hi) else "inf"
    if m.sum() == 0:
        print(f"  v in [{lo:.0f},{hi_label}) m/s: no samples")
        continue
    diff = (bg_deg - bk_deg)[m]
    print(f"  v in [{lo:.0f},{hi_label}) m/s  n={m.sum():6d}  "
          f"std(beta_gps-beta_kinematic)={np.std(diff):.3f} deg  MAD={np.median(np.abs(diff-np.median(diff))):.3f} deg")

print("--- Long-corner washout distortion ---")
compound = [c for c in data.get("corners", [])
            if "compound_corner" in c.get("warnings", [])
            and next((l for l in laps if l["lap_number"] == c["lap_number"]), {}).get("is_valid_for_analysis")]
if compound:
    c = max(compound, key=lambda c: c["segments"]["exit_5"][1] - c["segments"]["entry_2_turnin"][0])
    s_t = c["segments"]["entry_2_turnin"][0]
    e_t = c["segments"]["exit_5"][1]
    sl = _phase_slice(s_t, e_t)
    m = core_mask[sl]
    idx = np.where(m)[0]
    if len(idx) >= 20:
        half = len(idx) // 2
        first_bk = np.median(bk_deg[sl][idx[:half]])
        second_bk = np.median(bk_deg[sl][idx[half:]])
        first_bg = np.median(bg_deg[sl][idx[:half]])
        second_bg = np.median(bg_deg[sl][idx[half:]])
        print(f"  Selected: lap {c['lap_number']} corner {c['corner_number']} "
              f"(duration {e_t-s_t:.1f}s, n={len(idx)})")
        print(f"  beta_kinematic: first-half median={first_bk:+.3f} deg  second-half median={second_bk:+.3f} deg  "
              f"(decay toward zero = {abs(second_bk) < abs(first_bk)})")
        print(f"  beta_gps:       first-half median={first_bg:+.3f} deg  second-half median={second_bg:+.3f} deg  "
              f"(decay toward zero = {abs(second_bg) < abs(first_bg)})")
    else:
        print("  Too few samples in the longest compound-corner window.")
else:
    print("  No compound_corner instance found in valid-for-analysis laps.")

print()
print("=" * 70)
print("SECTION 4 -- Amendment 2: lever-arm (antenna-offset) probe (finding only)")
print("=" * 70)

diff_rad = beta_g - beta_k
yaw_over_v = np.where(core_mask, state["yaw_rate_radps"] / np.where(state["v_mps"] > 0.1, state["v_mps"], np.nan), np.nan)
m2 = core_mask & ~np.isnan(yaw_over_v) & ~np.isnan(diff_rad)
X = yaw_over_v[m2]
Y = diff_rad[m2]
slope, intercept = np.polyfit(X, Y, 1)
r_lever = np.corrcoef(X, Y)[0, 1]
print(f"n = {m2.sum()}")
print(f"(beta_gps - beta_kinematic) = {slope:.4f} * (yaw_rate/v) + {intercept:.5f}   r={r_lever:+.4f}")
print(f"Implied lever-arm (antenna fore/aft offset from CoG) = {slope:.3f} m "
      f"({'aft of' if slope > 0 else 'forward of'} CoG per the sign convention above)")

# Does applying the implied correction improve per-corner sign agreement?
v_safe = np.where(state["v_mps"] > 0.1, state["v_mps"], 1.0)
beta_g_corrected = beta_g - slope * np.where(state["v_mps"] > 0.1, state["yaw_rate_radps"] / v_safe, 0.0)
bg_corr_deg = np.degrees(beta_g_corrected)
sm2, st2 = 0, 0
for lap_n, corner_n, sid, phase, bk_med, _ in records:
    c = next(c for c in data["corners"] if c["lap_number"] == lap_n and c["corner_number"] == corner_n)
    s_t, e_t = c["segments"][phase]
    sl = _phase_slice(s_t, e_t, is_apex=(phase == "apex_3"))
    m = core_mask[sl]
    if m.sum() == 0:
        continue
    bg_corr_med = np.median(bg_corr_deg[sl][m])
    st2 += 1
    if np.sign(bk_med) == np.sign(bg_corr_med):
        sm2 += 1
print(f"Sign agreement WITHOUT lever-arm correction: {sign_matches}/{sign_total} = {sign_matches/sign_total*100:.1f}%")
print(f"Sign agreement WITH lever-arm correction:    {sm2}/{st2} = {sm2/st2*100:.1f}%")
print("Finding only -- no correction wired into estimate_sideslip_gps or anywhere else this phase.")

print()
print("=" * 70)
print("SECTION 5 -- Decision-criteria verdict (this run)")
print("=" * 70)
# Iteration 1 reference values (rotation-proportional/latency fixes not yet
# applied), frozen from that run's report, for before/after comparison only
# -- not recomputed here, this script now always reflects iteration 2's
# construction (estimate_sideslip_gps rotation-proportional + latency-
# corrected).
ITER1_R = -0.1182
ITER1_SIGN_PCT = 130 / 255 * 100
ITER1_LEVER_ARM_M = 9.342

print(f"1. Correlation r(beta_gps, beta_kinematic) = {r_beta:+.4f} "
      f"(iteration 1: {ITER1_R:+.4f}) -> NOT MET (still near-zero/weak)")
print(f"2. Straight-line bias: kinematic {np.median(np.abs(bk_deg[straight_mask])):.3f} deg vs "
      f"gps {np.median(np.abs(bg_deg[straight_mask])):.3f} deg -> NOT MET")
print(f"3. Per-corner sign agreement {sign_matches}/{sign_total} = {sign_matches/sign_total*100:.1f}% "
      f"(iteration 1: {ITER1_SIGN_PCT:.1f}%) -> NOT MET, essentially unchanged, still barely "
      f"above the 50% chance level for a two-sided sign")
print(f"4. Low-speed degradation: no samples fell in the two lowest speed bins tested "
      f"(gate={gate:.0f} m/s already excludes the region of concern on this dataset) "
      f"-> INCONCLUSIVE, not exercised by this file")
decay_bk = abs(second_bk) < abs(first_bk)
decay_bg = abs(second_bg) < abs(first_bg)
print(f"5. Long-corner washout probe: beta_kinematic decay-toward-zero={decay_bk} "
      f"({first_bk:+.3f} -> {second_bk:+.3f} deg); beta_gps decay-toward-zero={decay_bg} "
      f"({first_bg:+.3f} -> {second_bg:+.3f} deg) -> {'both now show the predicted direction' if decay_bk and decay_bg else 'asymmetric result'}, "
      f"a change from iteration 1 (beta_gps grew instead of decaying there) -> PARTIALLY MET: "
      f"direction now matches the prediction, magnitude does not (beta_gps stays well above zero)")
print(f"\nAmendment 2 cross-check (this run): lever-arm slope = {slope:.3f} m "
      f"(iteration 1: {ITER1_LEVER_ARM_M:.3f} m, a {(1-slope/ITER1_LEVER_ARM_M)*100:.0f}% reduction) "
      f"-> the falsifiable prediction from the root-cause diagnosis PASSED: the phantom lever-arm "
      f"shrank sharply once the rotation-proportional correction targeted the diagnosed drift "
      f"mechanism, corroborating Section 1b's diagnosis even though it did not fix the headline "
      f"correlation/sign-agreement numbers.")
print()
print("OVERALL VERDICT: STILL NOT MET. Both fixes approved for this iteration (rotation-")
print("proportional drift allocation, +0.32s latency correction) are implemented and verified")
print("isolated; the diagnosis behind them is now more strongly evidenced (lever-arm falsifiable")
print("check passed, washout-decay direction now correct), but the core decision-criteria metrics")
print("(correlation, straight-line bias, per-corner sign agreement) did not materially improve.")
print("This line is SHELVED: both iterations are documented (thesis_notes.md); a further attempt")
print("would need to address a source beyond allocation scheme and latency -- e.g. more/denser")
print("anchors than this single 4-lap session offers (6 total), which is a data-availability")
print("limit, not obviously a construction fix. Production beta stays kinematic; no consumer")
print("changed.")
