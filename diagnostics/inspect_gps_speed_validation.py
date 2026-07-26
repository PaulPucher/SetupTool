# WP5b(d) validation report: GPS speed (log_gps_speed) vs ecu_speed, on
# real Dubai data. Report-only -- nothing here feeds prepare_vehicle_state
# or any Module 2-5 function; ecu_speed stays the production speed source
# and the pipeline's own time anchor (t_ref), untouched throughout.
#
# Channels-direct, isolated: reads channels.get("log_gps_speed") directly
# (same pattern as estimate_sideslip_gps, WP5b(c)) rather than adding a
# field to prepare_vehicle_state's shared state dict -- this comparison has
# exactly one reader (this script), so it stays out of the shared pipeline
# builder entirely, an even smaller footprint than WP5b(c) needed.
#
# Sections:
#   0. Latency: cross-correlation of GPS speed against ecu_speed.
#   1. Origin-regression scale factor k (rolling-radius calibration check).
#   2. Residual spread (raw and post-k).
#   3. Braking/traction slip prediction (falsifiable, signed).
#   4. Per-speed-class consistency of k.
#   5. Kerb-sample probe.
#   6. Decision-criteria verdict (a / b / c).

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
se = params["stability_estimation"]
state = prepare_vehicle_state(data["channels"], params)

t_ref = state["time"]
sr = state["sample_rate_hz"]
moving = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
v_ecu_mps = state["v_mps"]

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)
core_mask = moving & racing_mask

gps_ch = data["channels"]["log_gps_speed"]
gt, gd = gps_ch["time"], gps_ch["data"]

print("=" * 70)
print("SECTION 0 -- Latency: cross-correlation of GPS speed vs ecu_speed")
print("=" * 70)

v_gps_zero_lag = np.interp(t_ref, gt, gd) / 3.6  # km/h -> m/s, no shift
lags_s = np.arange(-1.0, 1.0 + 1e-9, 1.0 / sr)
n = len(v_ecu_mps)
rs = []
for lag_s in lags_s:
    shift = int(round(lag_s * sr))
    v_gps_shifted = np.interp(t_ref + lag_s, gt, gd) / 3.6
    if shift >= 0:
        xs, ys = v_gps_shifted[shift:], v_ecu_mps[shift:]
        ms = core_mask[shift:]
    else:
        s2 = -shift
        xs, ys = v_gps_shifted[: n - s2], v_ecu_mps[: n - s2]
        ms = core_mask[: n - s2]
    rs.append(np.corrcoef(xs[ms], ys[ms])[0, 1] if ms.sum() >= 100 else np.nan)
rs = np.array(rs)
best = np.nanargmax(rs)
peak_lag = lags_s[best]
zero_lag_r = rs[np.argmin(np.abs(lags_s))]
print(f"Cross-correlation peak: r={rs[best]:.4f} at lag={peak_lag:+.3f}s "
      f"(r at zero lag: {zero_lag_r:.4f})")

course_latency = se["gps_course_latency_s"]
if abs(peak_lag - course_latency) <= 2.0 / sr:
    latency_s = course_latency
    print(f"Peak lag within 2 samples ({2.0/sr:.3f}s) of gps_course_latency_s="
          f"{course_latency}s -> REUSING that key, no new config entry.")
else:
    latency_s = peak_lag
    print(f"Peak lag differs from gps_course_latency_s={course_latency}s by more "
          f"than 2 samples -> a new gps_speed_latency_s={latency_s:.3f}s is warranted "
          f"(not yet written to config -- reported here first).")

v_gps_mps = np.interp(t_ref + latency_s, gt, gd) / 3.6

print()
print("=" * 70)
print("SECTION 1 -- Origin-regression scale factor k (v_gps = k * v_ecu)")
print("=" * 70)
vg = v_gps_mps[core_mask]
ve = v_ecu_mps[core_mask]
k = float(np.sum(vg * ve) / np.sum(ve ** 2))
print(f"n = {core_mask.sum()}")
print(f"k (least-squares, through origin) = {k:.5f}")
print(f"Interpretation: ecu_speed reads {'LOW' if k > 1 else 'HIGH'} vs GPS by "
      f"{abs(1 - k) * 100:.2f}% on average, IF this is a constant rolling-radius "
      f"offset rather than a slip artifact (checked in Section 3).")

print()
print("=" * 70)
print("SECTION 2 -- Residual spread")
print("=" * 70)
raw_resid = vg - ve
k_resid = vg - k * ve
print(f"Raw (v_gps - v_ecu):      median={np.median(raw_resid):+.3f}  "
      f"std={np.std(raw_resid):.3f}  MAD={np.median(np.abs(raw_resid - np.median(raw_resid))):.3f} m/s")
print(f"Post-k (v_gps - k*v_ecu): median={np.median(k_resid):+.3f}  "
      f"std={np.std(k_resid):.3f}  MAD={np.median(np.abs(k_resid - np.median(k_resid))):.3f} m/s")

print()
print("=" * 70)
print("SECTION 3 -- Braking/traction slip prediction (falsifiable, signed)")
print("=" * 70)
brake = state["brake_f_bar"][core_mask]
throttle = state["throttle_pct"][core_mask]
diff = raw_resid  # v_gps - v_ecu; positive means ecu reads LOW vs gps

brake_thresh = np.nanpercentile(brake, 90)
heavy_brake = brake > brake_thresh
heavy_accel = (throttle > 80) & (brake < 1.0)

print(f"Prediction: wheel slip makes ecu_speed read WRONG under load -- LOW under "
      f"heavy braking (v_gps-v_ecu > 0), HIGH under heavy traction (v_gps-v_ecu < 0).")
print(f"Heavy-braking samples (brake > p90={brake_thresh:.1f} bar): {heavy_brake.sum()}")
print(f"  median (v_gps - v_ecu): {np.median(diff[heavy_brake]):+.3f} m/s")
print(f"Heavy-traction samples (throttle>80%, brake<1 bar): {heavy_accel.sum()}")
print(f"  median (v_gps - v_ecu): {np.median(diff[heavy_accel]):+.3f} m/s")
brake_sign_match = np.median(diff[heavy_brake]) > 0
accel_sign_match = np.median(diff[heavy_accel]) < 0
print(f"Braking prediction {'CONFIRMED' if brake_sign_match else 'REFUTED'} "
      f"(sign {'matches' if brake_sign_match else 'does not match'} the slip prediction)")
print(f"Traction prediction {'CONFIRMED' if accel_sign_match else 'REFUTED'} "
      f"(sign {'matches' if accel_sign_match else 'does not match'} the slip prediction)")

print()
print("=" * 70)
print("SECTION 4 -- Per-speed-class consistency of k")
print("=" * 70)
import json
with open("config/channels.json", "r", encoding="utf-8") as f:
    ch_cfg = json.load(f)
low_max = ch_cfg["corner_speed_thresholds"]["low_max"]
med_max = ch_cfg["corner_speed_thresholds"]["medium_max"]
v_ecu_kmh_core = ve * 3.6
bins = {
    "low": v_ecu_kmh_core <= low_max,
    "medium": (v_ecu_kmh_core > low_max) & (v_ecu_kmh_core <= med_max),
    "high": v_ecu_kmh_core > med_max,
}
k_per_bin = {}
for name, m in bins.items():
    if m.sum() < 50:
        print(f"  speed_class={name:6s}  n={m.sum():6d}  too few samples")
        continue
    k_bin = float(np.sum(vg[m] * ve[m]) / np.sum(ve[m] ** 2))
    k_per_bin[name] = k_bin
    print(f"  speed_class={name:6s}  n={m.sum():6d}  k={k_bin:.5f}")
if len(k_per_bin) >= 2:
    k_vals = list(k_per_bin.values())
    print(f"  k range across speed classes: {max(k_vals)-min(k_vals):.5f} "
          f"(whole-session k={k:.5f})")

print()
print("=" * 70)
print("SECTION 5 -- Kerb-sample probe")
print("=" * 70)
if kerb_mask is not None:
    kerb_on = core_mask & kerb_mask
    kerb_off = core_mask & ~kerb_mask
    diff_on = (v_gps_mps - v_ecu_mps)[kerb_on]
    diff_off = (v_gps_mps - v_ecu_mps)[kerb_off]
    print(f"  on-kerb   n={kerb_on.sum():6d}  median (v_gps-v_ecu)={np.median(diff_on):+.3f}  std={np.std(diff_on):.3f} m/s")
    print(f"  off-kerb  n={kerb_off.sum():6d}  median (v_gps-v_ecu)={np.median(diff_off):+.3f}  std={np.std(diff_off):.3f} m/s")
else:
    print("  kerb_mask unavailable")

print()
print("=" * 70)
print("SECTION 6 -- Decision-criteria verdict")
print("=" * 70)
print(f"k = {k:.5f} (whole session); range across speed classes = "
      f"{max(k_per_bin.values())-min(k_per_bin.values()):.5f}" if len(k_per_bin) >= 2 else "k spread: insufficient bins")
print(f"Braking-slip prediction: {'CONFIRMED' if brake_sign_match else 'REFUTED'}; "
      f"traction-slip prediction: {'CONFIRMED' if accel_sign_match else 'REFUTED'}")
print("Prior: WP5b(c)'s beta_gps was shelved on this exact GPS receiver/logger --")
print("real latency, 10 Hz effective resolution, an unresolved residual drift were")
print("all found there. That is evidence against option (a), not a reason to skip")
print("checking it here.")
