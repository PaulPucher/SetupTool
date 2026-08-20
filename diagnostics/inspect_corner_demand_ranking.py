# Corner tyre-demand ranking, independent of any CS_ratio/EKF output --
# built purely from measured speed/ay/curvature, to cross-reference
# against the pass_1 flagged set. Read-only, no config change.
#
# Ranking metric: median |ay| through each corner's canonical window.
# ay is the most direct physical proxy for tyre lateral force demand
# (Fy ~ m*ay) among the candidate metrics -- more direct than speed
# alone (doesn't account for radius) or radius alone (doesn't account
# for speed). p95 |ay| and duration above 0.8g are reported alongside
# as corroborating (peak vs sustained demand), not used as the primary
# rank key.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
AY_DEMAND_THRESHOLD_G = 0.8

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
s_m = state.get("s_m")
v_mps = state["v_mps"]
ay = state["ay_mps2"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
sr = state["sample_rate_hz"]

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners = data.get("corners", [])
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)

corner_radius_channel = data["channels"].get("corner_radius_filtered")
has_radius_channel = (corner_radius_channel is not None
                       and corner_radius_channel.get("quality") not in ("missing", "failed")
                       and corner_radius_channel.get("time") is not None)
radius_vals_full = None
if has_radius_channel:
    radius_vals_full = np.interp(t, corner_radius_channel["time"], corner_radius_channel["data"])
print(f"corner_radius_filtered channel available: {has_radius_channel}")
print()


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


rows = []
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    speed_class = instances[0].get("speed_class")
    if bracket_start is None or bracket_end is None:
        continue
    pooled_v, pooled_ay, pooled_radius = [], [], []
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
        pooled_v.append(v_mps[sl][m])
        pooled_ay.append(ay[sl][m])
        if has_radius_channel:
            pooled_radius.append(radius_vals_full[sl][m])
    if not pooled_v:
        continue

    v_cat = np.concatenate(pooled_v)
    ay_cat = np.abs(np.concatenate(pooled_ay))
    min_speed_kmh = float(np.min(v_cat)) * 3.6
    median_ay_g = float(np.median(ay_cat)) / 9.81
    p95_ay_g = float(np.percentile(ay_cat, 95)) / 9.81
    duration_above_s = float(np.sum(ay_cat > AY_DEMAND_THRESHOLD_G * 9.81)) / sr / len(pooled_v)  # per-lap-average seconds

    if has_radius_channel and pooled_radius:
        radius_cat = np.concatenate(pooled_radius)
        radius_finite = radius_cat[np.isfinite(radius_cat) & (radius_cat > 0)]
        median_radius_m = float(np.median(radius_finite)) if len(radius_finite) else float("nan")
        radius_source = "corner_radius_filtered"
    else:
        # kinematic fallback: R = v^2/ay, sample-wise then median (avoids
        # dividing by a pooled/averaged near-zero ay)
        ay_safe = np.concatenate(pooled_ay)
        v_safe = v_cat
        finite = np.abs(ay_safe) > 0.5  # guard near-zero ay -> huge/undefined radius
        radius_samples = (v_safe[finite] ** 2) / np.abs(ay_safe[finite])
        median_radius_m = float(np.median(radius_samples)) if finite.any() else float("nan")
        radius_source = "kinematic v^2/ay"

    rows.append({
        "cid": cid, "min_speed_kmh": min_speed_kmh, "median_ay_g": median_ay_g,
        "p95_ay_g": p95_ay_g, "duration_above_0p8g_s": duration_above_s,
        "median_radius_m": median_radius_m, "radius_source": radius_source,
        "speed_class": speed_class,
    })

print("=" * 100)
print("Per-corner demand metrics (valid laps, moving, kerb-excluded, canonical window)")
print("=" * 100)
for r in rows:
    print(f"  C{r['cid']:2d}  min_speed={r['min_speed_kmh']:6.1f} km/h  "
          f"median|ay|={r['median_ay_g']:.3f} g  p95|ay|={r['p95_ay_g']:.3f} g  "
          f"dur>0.8g={r['duration_above_0p8g_s']:.3f} s/lap  "
          f"radius={r['median_radius_m']:6.1f} m ({r['radius_source']})  speed_class={r['speed_class']}")
print()

ranked = sorted(rows, key=lambda r: r["median_ay_g"], reverse=True)
print("=" * 100)
print("RANKED by median |ay| (highest tyre demand first)")
print("=" * 100)
for rank, r in enumerate(ranked, 1):
    tag = "  <-- C4/C14" if r["cid"] in (4, 14) else ""
    print(f"  #{rank:2d}: C{r['cid']:2d}  median|ay|={r['median_ay_g']:.3f} g  "
          f"p95={r['p95_ay_g']:.3f} g  dur>0.8g={r['duration_above_0p8g_s']:.3f} s/lap{tag}")
