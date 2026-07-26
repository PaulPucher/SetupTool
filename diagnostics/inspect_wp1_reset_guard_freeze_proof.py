# WP1-freeze before/after proof for the corner_analysis.py:359 reset-guard
# fix (small-decisions sweep, item 3). Dumps a compact, diffable summary --
# per-lap corner counts, every corner's apex_lap_distance_m, and every
# corner's stable_corner_id -- so a before/after run (old plain-np.interp
# vs the new reset-guarded interpolation) can be diffed directly.

from modules.csv_parser import parse_csv

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
corners = data.get("corners", [])
laps = data.get("laps", [])

per_lap_count = {}
for c in corners:
    per_lap_count[c["lap_number"]] = per_lap_count.get(c["lap_number"], 0) + 1

print("=== Per-lap corner counts ===")
for lap_number in sorted(per_lap_count):
    print(f"lap {lap_number}: {per_lap_count[lap_number]}")

print("\n=== Per-corner apex_lap_distance_m + stable_corner_id ===")
for c in sorted(corners, key=lambda c: (c["lap_number"], c["corner_number"])):
    print(f"lap={c['lap_number']:2d} corner={c['corner_number']:2d} "
          f"apex_lap_distance_m={c['apex_lap_distance_m']:.10f} "
          f"stable_corner_id={c['stable_corner_id']}")
