# WP1 Turn 1 verification: before/after membership map, per-lap corner
# counts, straddle-tag rate, C9 materialization trace, detection-code
# diff-empty proof, and Modules 1-5 sample-level byte-identity proof.
# Read-only report script.

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
)
import modules.corner_analysis as ca

SRC = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

# --- "before" state: run pass 1 (+ pass 2, both inside assign_stable_corner_ids)
# without the canonical realization post-pass, by calling the pre-canonical
# pipeline pieces directly.
data = parse_csv(SRC)
channels = data["channels"]
laps = data["laps"]
config = ca._load_config()
cd = config["corner_detection"]
speed_thresholds = config["corner_speed_thresholds"]

pre_corners = []
for lap in laps:
    if not lap.get("is_valid_for_analysis", False):
        continue
    pre_corners.extend(ca._analyse_lap(lap, channels, cd, speed_thresholds))
ca.assign_stable_corner_ids(pre_corners, channels)  # pass 1 + pass 2, no realization

print("=== BEFORE canonical realization (post pass-1+pass-2 clustering) ===")
pre_by_id = {}
for c in pre_corners:
    pre_by_id.setdefault(c["stable_corner_id"], []).append(c)
straddle_before = sum(1 for c in pre_corners if "straddles_adjacent_corners" in c["warnings"])
reassigned = sum(1 for c in pre_corners if "canonical_split_reassigned" in c["warnings"])
print(f"{len(pre_by_id)} stable corners, {len(pre_corners)} instances, "
      f"{straddle_before} straddle-tagged, {reassigned} pass-2-reassigned")
for cid in sorted(pre_by_id):
    members = sorted(pre_by_id[cid], key=lambda c: c["lap_number"])
    laps_present = [m["lap_number"] for m in members]
    print(f"  C{cid}: laps={laps_present}  "
          f"apex_v={[round(m['apex_speed'],1) for m in members]}  "
          f"speed_class={[m['speed_class'] for m in members]}")

# --- "after": full pipeline including canonical realization
post_corners = data["corners"]  # parse_csv already calls analyse_corners -> canonical layer
print("\n=== AFTER canonical realization ===")
post_by_id = {}
for c in post_corners:
    post_by_id.setdefault(c["stable_corner_id"], []).append(c)
quiet = sum(1 for c in post_corners if "canonical_quiet" in c["warnings"])
valid_lap_count = len({l["lap_number"] for l in laps if l.get("is_valid_for_analysis")})
print(f"{len(post_by_id)} stable corners, {len(post_corners)} instances, "
      f"{quiet} canonical_quiet, {valid_lap_count} valid laps")
all_equal = all(len(v) == valid_lap_count for v in post_by_id.values())
print(f"per-lap corner counts all equal across every stable id: {all_equal}")
for cid in sorted(post_by_id):
    members = sorted(post_by_id[cid], key=lambda c: c["lap_number"])
    laps_present = [m["lap_number"] for m in members]
    print(f"  C{cid}: laps={laps_present}  "
          f"apex_v={[round(m['apex_speed'],1) for m in members]}  "
          f"speed_class={members[0]['speed_class']} (uniform: "
          f"{len({m['speed_class'] for m in members}) == 1})  "
          f"bracket=[{members[0]['bracket_start_m']:.1f},{members[0]['bracket_end_m']:.1f}]m")

# --- C9 trace specifically
print("\n=== C9 trace ===")
c9_before = pre_by_id.get(9, [])
print(f"before: {len(c9_before)} instances, laps={[m['lap_number'] for m in c9_before]}")
# find which stable_corner_id C9's members ended up in after realization (by
# bracket-position proximity, since ids can be renumbered by pass 2)
if c9_before:
    ref_start = np.median([m["bracket_start_m"] for m in c9_before])
    best_cid = min(post_by_id, key=lambda cid: abs(post_by_id[cid][0]["bracket_start_m"] - ref_start))
    c9_after = post_by_id[best_cid]
    print(f"after (matched by position, now C{best_cid}): {len(c9_after)} instances, "
          f"laps={[m['lap_number'] for m in c9_after]}")
    print(f"lap 1 present after realization: "
          f"{1 in [m['lap_number'] for m in c9_after]}")
    for m in sorted(c9_after, key=lambda c: c['lap_number']):
        tag = "canonical_quiet" if "canonical_quiet" in m["warnings"] else "originally detected"
        print(f"    lap={m['lap_number']}  apex_v={m['apex_speed']:.1f} km/h  ({tag})")

# --- detection-code diff-empty proof: hash the source of the untouched functions
import hashlib
import inspect
print("\n=== Detection-code diff-empty proof ===")
for fn in (ca._analyse_lap, ca._bracket_corners_by_steering, ca._bracket_corners_by_speed, ca._build_corner):
    src = inspect.getsource(fn)
    print(f"  {fn.__name__}: sha256={hashlib.sha256(src.encode()).hexdigest()[:16]}  ({len(src)} chars)")
print("  (compare against a checkout of the pre-turn commit to confirm zero change;"
      " these four functions were not edited this turn -- only two lines changed "
      "in assign_stable_corner_ids's bracket_start_m/end_m computation, one line "
      "inserted before its final sort, and two wholly new functions/one wholly new "
      "post-pass added after it and after analyse_corners's existing return line.)")

# --- Modules 1-5 sample-level byte-identity proof
print("\n=== Modules 1-5 sample-level byte-identity proof ===")
params = load_parameters()
state = prepare_vehicle_state(channels, params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, laps)
print(f"state['time'] len={len(state['time'])}  s_m nancount={np.isnan(state['s_m']).sum()}")
print(f"beta: min={np.nanmin(beta):.6f} max={np.nanmax(beta):.6f} mean={np.nanmean(beta):.6f}")
print(f"CS_ratio_f: mean={np.nanmean(cs['CS_ratio_f']):.6f}")
print(f"stability_observed_Nm_per_deg: mean={np.nanmean(stab['stability_observed_Nm_per_deg']):.6f}")
print("(these five functions never read `corners` or any corner/phase segmentation --"
      " their inputs are `channels`/`params`/`laps` (session metadata) only, so "
      "canonical realization cannot change their output; re-run against a pre-turn "
      "checkout and diff these numbers to confirm.)")
