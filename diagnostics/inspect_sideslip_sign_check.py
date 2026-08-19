# WP-S5 preliminary (Open Board item B, sideslip methods comparison):
# sideslip sign check. Read-only, Tier B -- no production/config change.
# Sibling script, not a plot_sideslip_comparison.py extension: this is a
# numeric physics cross-check, not a visualization, and narrow/one-off
# like WP-S3b/S3c/S4b before it.
#
# Context: diagnostics/plot_sideslip_comparison.py's per-corner summary
# showed A_kinematic and C_kalman_observer disagreeing in SIGN (not just
# magnitude) at C6/C7/C9/C10/C12. Before any tuning, check which method's
# sign matches the physical expectation.
#
# Physical expectation (standard bicycle-model steady-state result,
# Rajamani sec. 2.3/2.6 -- same anchor as the observer's own vehicle
# model, thesis_notes.md WP-S4 entry): at racing speed, sideslip beta
# signs OPPOSITE the turn direction (rear points to the outside of the
# corner); at low speed this reverses (beta signs WITH the turn
# direction) as the lr/R kinematic term dominates over the speed-scaled
# term. Turn direction = sign(median ay) (this codebase's own established
# convention, thesis_notes.md "GPS-course sideslip..." / WP-S3c). Low-
# speed flag reuses this codebase's own canonical, data-derived
# corner_speed_thresholds.low_max (config/channels.json, already the
# production speed_class boundary -- not a new threshold invented here.

import json

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_sideslip
from diagnostics.sideslip_kalman_observer import estimate_sideslip_kalman

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
FLAGGED_CORNERS = {6, 7, 9, 10, 12}

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t_ref = state["time"]
s_m = state.get("s_m")
v_kmh = state["v_mps"] * 3.6
ay = state["ay_mps2"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

beta_a_deg = np.degrees(estimate_sideslip(state, params))
beta_c_deg = np.degrees(estimate_sideslip_kalman(state, params))

with open("config/channels.json", "r", encoding="utf-8") as f:
    channels_json = json.load(f)
LOW_SPEED_MAX_KMH = channels_json["corner_speed_thresholds"]["low_max"]

laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners = data.get("corners", [])
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to the helper of the same name used throughout this WP.
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


results = {}
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    speed_class = instances[0].get("speed_class")
    if bracket_start is None or bracket_end is None:
        continue
    pooled_ay, pooled_v, pooled_a, pooled_c = [], [], [], []
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
        pooled_ay.append(ay[sl][m])
        pooled_v.append(v_kmh[sl][m])
        pooled_a.append(beta_a_deg[sl][m])
        pooled_c.append(beta_c_deg[sl][m])
    if not pooled_ay:
        continue
    med_ay = float(np.median(np.concatenate(pooled_ay)))
    med_v = float(np.median(np.concatenate(pooled_v)))
    med_a = float(np.median(np.concatenate(pooled_a)))
    med_c = float(np.median(np.concatenate(pooled_c)))
    results[cid] = {
        "med_ay": med_ay, "med_v": med_v, "med_a": med_a, "med_c": med_c,
        "speed_class": speed_class,
    }

print("=" * 100)
print("Sideslip sign check vs physical expectation (racing speed: beta opposite turn direction)")
print("=" * 100)
print(f"Low-speed threshold reused from config/channels.json corner_speed_thresholds.low_max = "
      f"{LOW_SPEED_MAX_KMH} km/h (this codebase's own canonical speed_class boundary, not invented here).")
print()

n_match_a = n_match_c = n_total = 0
for cid in sorted(results):
    r = results[cid]
    dir_sign = np.sign(r["med_ay"])
    dir_tag = "ay+" if dir_sign > 0 else "ay-" if dir_sign < 0 else "0"
    low_speed = r["med_v"] < LOW_SPEED_MAX_KMH
    a_sign = np.sign(r["med_a"])
    c_sign = np.sign(r["med_c"])
    a_match = (a_sign == -dir_sign) if dir_sign != 0 else None
    c_match = (c_sign == -dir_sign) if dir_sign != 0 else None
    n_total += 1
    n_match_a += int(bool(a_match))
    n_match_c += int(bool(c_match))
    flag = "  <-- FLAGGED (plot script found sign disagreement A vs C)" if cid in FLAGGED_CORNERS else ""
    print(f"C{cid}: turn={dir_tag} (median ay={r['med_ay']:+.2f} m/s^2)  "
          f"median v={r['med_v']:6.1f} km/h  speed_class={r['speed_class']:<6}  "
          f"low_speed_reversal_expected={low_speed}")
    print(f"      kinematic beta={r['med_a']:+7.3f} deg  matches racing-speed expectation: {a_match}")
    print(f"      observer  beta={r['med_c']:+7.3f} deg  matches racing-speed expectation: {c_match}{flag}")
    print()

print("=" * 100)
print(f"SUMMARY: kinematic matches racing-speed physical expectation {n_match_a}/{n_total} corners; "
      f"observer matches {n_match_c}/{n_total} corners.")
print()
print("Flagged corners (C6, C7, C9, C10, C12) detail:")
for cid in sorted(FLAGGED_CORNERS):
    r = results.get(cid)
    if r is None:
        print(f"  C{cid}: not found in results.")
        continue
    dir_sign = np.sign(r["med_ay"])
    dir_tag = "ay+" if dir_sign > 0 else "ay-"
    low_speed = r["med_v"] < LOW_SPEED_MAX_KMH
    print(f"  C{cid}: turn={dir_tag}  v={r['med_v']:.1f} km/h  speed_class={r['speed_class']}  "
          f"low_speed_reversal_expected={low_speed}  "
          f"kinematic={r['med_a']:+.3f} deg  observer={r['med_c']:+.3f} deg")
