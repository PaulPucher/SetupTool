# WP-N0 (nonlinear observer work package, gating diagnostic): read-only,
# Tier B preprocessing check. No production/config/ui change.
#
# Two numbers this script produces feed two open design decisions for the
# next work package (nonlinear single-track Kalman filter with a
# data-identified tyre curve):
#   (1) how close |Fy|/Fz gets to a plausible friction ceiling on this car's
#       own data -- whether a saturating tyre model's peak-force parameter
#       is identifiable at all from what this session actually visited;
#   (2) how many samples a "low-slip regime" first curve fit (the planned
#       fit-refit iteration's starting point) would have to work with.
#
# Values only, no interpretation -- that happens in review.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_lateral_forces,
    estimate_vertical_loads,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
forces = estimate_lateral_forces(state, params)
fz = estimate_vertical_loads(state, forces, params)

t = state["time"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)

base_mask = moving & racing_mask

# --- Section C metadata ------------------------------------------------

valid_lap_numbers = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis"))

print("=" * 100)
print("SECTION C -- metadata")
print("=" * 100)
print(f"file: {RAW_FILE}")
print(f"laps used (is_valid_for_analysis): {valid_lap_numbers}")
print(f"total sample count after masking: {int(base_mask.sum())}")
print("masking applied: moving_mask & ~kerb_mask & valid-lap time windows "
      "(same mask summarise_corners/production pipeline uses)")
print()

# --- Section A: |Fy|/Fz distribution, per axle and per stable corner ---

corners = data.get("corners", [])
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _corner_span_mask(c, t, base_mask):
    # entry_2_turnin start / exit_5 end matches corner_analysis.py's own
    # bracket_start_t/bracket_end_t (assign_stable_corner_ids) -- NOT
    # entry_1_brake, whose off-throttle lookback can land far earlier in
    # the lap and would make corner windows overlap wildly.
    start_t, _ = c["segments"]["entry_2_turnin"]
    _, end_t = c["segments"]["exit_5"]
    if end_t < start_t:
        return np.zeros_like(base_mask)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    m = np.zeros_like(base_mask)
    m[lo:hi] = True
    return m & base_mask


ratio_f = np.abs(fz["fy_f_norm_N"])
ratio_r = np.abs(fz["fy_r_norm_N"])

print("=" * 100)
print("SECTION A -- |Fy_axle| / Fz_axle distribution (base mask: valid-lap, moving, kerb-excluded)")
print("=" * 100)

for axle_name, ratio in (("front", ratio_f), ("rear", ratio_r)):
    vals = ratio[base_mask]
    vals = vals[np.isfinite(vals)]
    p50, p90, p95, p99 = np.percentile(vals, [50, 90, 95, 99])
    print(f"  {axle_name:6s} n={len(vals):6d}  p50={p50:.3f}  p90={p90:.3f}  "
          f"p95={p95:.3f}  p99={p99:.3f}  max={float(np.max(vals)):.3f}")
print()

print("Per stable corner id (own p95 / max):")
for sid in stable_ids:
    corner_mask = np.zeros_like(base_mask)
    for c in corners_by_stable_id[sid]:
        corner_mask |= _corner_span_mask(c, t, base_mask)
    n = int(corner_mask.sum())
    if n == 0:
        print(f"  C{sid}: n=0")
        continue
    vf = ratio_f[corner_mask]
    vf = vf[np.isfinite(vf)]
    vr = ratio_r[corner_mask]
    vr = vr[np.isfinite(vr)]
    p95_f = float(np.percentile(vf, 95)) if len(vf) else float("nan")
    max_f = float(np.max(vf)) if len(vf) else float("nan")
    p95_r = float(np.percentile(vr, 95)) if len(vr) else float("nan")
    max_r = float(np.max(vr)) if len(vr) else float("nan")
    print(f"  C{sid}: n={n:5d}  front p95={p95_f:.3f} max={max_f:.3f}  "
          f"rear p95={p95_r:.3f} max={max_r:.3f}")
print()

# --- Section B: |ay| distribution and low-slip sample budget -----------

ay_g = np.abs(state["ay_mps2"])[base_mask] / 9.81
ay_g = ay_g[np.isfinite(ay_g)]
ay_mps2 = ay_g * 9.81

print("=" * 100)
print("SECTION B -- |ay| distribution and low-slip candidate-threshold sample budget")
print("=" * 100)

p50, p90 = np.percentile(ay_g, [50, 90])
print(f"n={len(ay_g)}  p50={p50:.3f} g ({p50*9.81:.3f} m/s^2)  "
      f"p90={p90:.3f} g ({p90*9.81:.3f} m/s^2)  max={float(np.max(ay_g)):.3f} g "
      f"({float(np.max(ay_mps2)):.3f} m/s^2)")
print()

for thresh_g in (0.3, 0.4, 0.5):
    below = ay_g < thresh_g
    count = int(below.sum())
    frac = float(below.mean())
    print(f"  |ay| < {thresh_g} g: count={count}  fraction={frac:.4f}")
