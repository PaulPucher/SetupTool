# WP2b-2 task 5(a): where does the Dubai corner population actually sit
# relative to the speed-class thresholds (config/channels.json
# corner_speed_thresholds low_max/medium_max) that gate every matrix rule
# in config/recommendations.json? A rule-gating corner sitting right on a
# boundary is a real risk -- a few km/h of lap-to-lap noise could flip
# which matrix cell (low/medium/high) fires for it. Read-only; no analysis
# thresholds changed here.

import json
from modules.csv_parser import parse_csv

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
corners = data.get("corners", [])

with open("config/channels.json", encoding="utf-8") as f:
    thresholds = json.load(f)["corner_speed_thresholds"]
low_max = thresholds["low_max"]
medium_max = thresholds["medium_max"]
band_km_h = 5.0

print(f"Thresholds: low_max={low_max} km/h, medium_max={medium_max} km/h "
      f"(config/channels.json corner_speed_thresholds)")
print(f"{len(corners)} corner-lap instances total.\n")

by_class = {"low": [], "medium": [], "high": []}
for c in corners:
    by_class[c["speed_class"]].append(c["apex_speed"])

for cls in ("low", "medium", "high"):
    vals = sorted(by_class[cls])
    if not vals:
        print(f"{cls:>6}: 0 instances")
        continue
    n = len(vals)
    print(f"{cls:>6}: {n:3d} instances, "
          f"range [{vals[0]:.1f}, {vals[-1]:.1f}] km/h, "
          f"median {vals[n // 2]:.1f} km/h")

print(f"\nCorner-lap instances within +-{band_km_h:.0f} km/h of a boundary "
      f"(low_max={low_max}, medium_max={medium_max}):")
near_boundary = []
for c in corners:
    v = c["apex_speed"]
    if abs(v - low_max) <= band_km_h:
        near_boundary.append((c, "low_max", v - low_max))
    elif abs(v - medium_max) <= band_km_h:
        near_boundary.append((c, "medium_max", v - medium_max))

if not near_boundary:
    print("  none -- every corner-lap instance sits clear of both boundaries.")
else:
    # Group by stable_corner_id: a corner that straddles a boundary across
    # its own laps is the actual risk (its speed_class could differ lap to
    # lap even before the engine's own modal-aggregation tiebreak kicks in).
    by_stable = {}
    for c, boundary, delta in near_boundary:
        by_stable.setdefault(c.get("stable_corner_id"), []).append((c, boundary, delta))
    for cid, entries in sorted(by_stable.items(), key=lambda kv: (kv[0] is None, kv[0])):
        print(f"  C{cid}:")
        for c, boundary, delta in sorted(entries, key=lambda e: e[0]["lap_number"]):
            sign = "+" if delta >= 0 else ""
            print(f"    lap={c['lap_number']}  apex_v={c['apex_speed']:.1f} km/h  "
                  f"({sign}{delta:.1f} km/h from {boundary})  speed_class={c['speed_class']}")
        classes_seen = {c["speed_class"] for c, _, _ in entries}
        all_classes_this_corner = {
            cc["speed_class"] for cc in corners if cc.get("stable_corner_id") == cid
        }
        if len(all_classes_this_corner) > 1:
            print(f"    -> straddles speed_class across its own laps: {sorted(all_classes_this_corner)}")

print("\nNote: modules/recommendation.py's _aggregate_speed_class takes the "
      "modal class across a corner's laps (median-adjacent tiebreak) before "
      "gating any matrix rule, so a single near-boundary lap does not by "
      "itself flip which rule fires -- but a corner whose laps split roughly "
      "evenly across a boundary (see 'straddles' above, if any) is exactly "
      "the case that tiebreak logic was written for.")
