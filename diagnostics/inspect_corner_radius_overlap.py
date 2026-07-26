# WP2b-2 task 5(b): cross-validate our ay/threshold-based corner-bracket
# detection (modules/corner_analysis.py) against corner_radius_filtered, a
# raw logged channel (not in config/channels.json's whitelist -- read
# directly here, same technique as diagnostics/scan_channels.py) that the
# logger itself gates OFF entirely on straights (zero samples, not just
# smoothed/large) and keeps physically sane through corners (thesis_notes.md,
# "corner_radius_filtered ... gated OFF entirely on straights"). If our
# brackets and the channel's presence-windows disagree, that's evidence our
# detection is mis-drawing a bracket's start/end -- or that the logger's own
# corner classification differs from ours (compound corners, kerbs). Purely
# a comparison; nothing here feeds analysis or bracket detection.

import numpy as np
from modules.csv_parser import parse_csv

SRC = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"


def read_raw_channel(file_path, channel_name):
    # Same {ChannelBlock} tab-parsing as modules/csv_parser.py's parse_csv,
    # minus the config/channels.json whitelist filter -- this channel is
    # deliberately not in that whitelist (no consumer yet, see thesis_notes.md
    # "SCOPE DISCIPLINE"), so it has to be read directly for a one-off check.
    with open(file_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    i, n = 0, len(lines)
    while i < n:
        if lines[i].strip() == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    raw_name = header_parts[1].strip()
                    name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
                    i += 1
                    if name == channel_name:
                        times, values = [], []
                        while i < n and not lines[i].strip().startswith("{"):
                            parts = lines[i].strip().split("\t")
                            if len(parts) == 2:
                                try:
                                    t = float(parts[0].replace(",", "."))
                                    v = float(parts[1].replace(",", "."))
                                    times.append(t)
                                    values.append(v)
                                except ValueError:
                                    pass
                            i += 1
                        return np.array(times), np.array(values)
                    while i < n and not lines[i].strip().startswith("{"):
                        i += 1
                    continue
        i += 1
    return None, None


radius_t, radius_v = read_raw_channel(SRC, "corner_radius_filtered")
if radius_t is None:
    print("corner_radius_filtered not found in file -- nothing to compare.")
    raise SystemExit(0)
print(f"corner_radius_filtered: {len(radius_t)} present samples "
      f"(logger gates this channel off entirely on straights).")

# Presence-windows: contiguous runs of samples, allowing a small logger-rate
# gap (median dt-based) before treating a break as a real straight, not
# logger jitter.
dt_median = float(np.median(np.diff(radius_t)))
gap_thresh = dt_median * 3
windows = []
start = radius_t[0]
prev = radius_t[0]
for t in radius_t[1:]:
    if t - prev > gap_thresh:
        windows.append((start, prev))
        start = t
    prev = t
windows.append((start, prev))
print(f"{len(windows)} presence-windows (gap threshold {gap_thresh * 1000:.0f} ms).\n")

data = parse_csv(SRC)
corners = data.get("corners", [])
lap_lookup = {l["lap_number"]: l for l in data.get("laps", [])}
ld_ch = data["channels"].get("lap_distance")

overlaps = []
for c in corners:
    lap = lap_lookup.get(c["lap_number"])
    if lap is None:
        continue
    # Convert this corner's bracket (lap_distance, feet, per corner_analysis.py
    # conversion factor 0.3048) back to absolute time via the same channel's
    # own time base, then to lap-relative-independent absolute time using the
    # lap's start_time -- corner_radius_filtered is indexed on absolute time
    # like every other channel here.
    seg = c["segments"]
    bracket_start_t = seg["entry_2_turnin"][0]
    bracket_end_t = seg["exit_5"][1]

    best_overlap = 0.0
    bracket_len = bracket_end_t - bracket_start_t
    for w_start, w_end in windows:
        ov = min(bracket_end_t, w_end) - max(bracket_start_t, w_start)
        if ov > best_overlap:
            best_overlap = ov
    frac = best_overlap / bracket_len if bracket_len > 0 else 0.0
    overlaps.append((c, frac, bracket_len))

overlaps.sort(key=lambda x: x[1])
fracs = [f for _, f, _ in overlaps]
print(f"Overlap fraction (best-matching presence-window vs our bracket), "
      f"{len(overlaps)} corner-lap instances:")
print(f"  mean={np.mean(fracs):.2f}  median={np.median(fracs):.2f}  "
      f"min={np.min(fracs):.2f}  max={np.max(fracs):.2f}")

print("\nDisagreements (overlap fraction < 0.7):")
disagreements = [(c, f, blen) for c, f, blen in overlaps if f < 0.7]
if not disagreements:
    print("  none -- every bracket overlaps its best-matching presence-window >= 70%.")
else:
    for c, f, blen in disagreements:
        tags = ",".join(w for w in ("compound_corner", "straddles_adjacent_corners")
                         if w in c.get("warnings", []))
        print(f"  C{c.get('stable_corner_id')} lap={c['lap_number']} corner={c['corner_number']}: "
              f"overlap={f:.2f}  bracket_len={blen:.2f}s"
              f"{'  [' + tags + ']' if tags else ''}")

# WP1 Turn 2: full table (was disagreements-only before canonical
# realization existed), grouped per stable_corner_id, C10 highlighted.
print("\nFull table, per stable_corner_id (WP1 Turn 2):")
by_cid = {}
for c, f, blen in overlaps:
    by_cid.setdefault(c.get("stable_corner_id"), []).append((c, f, blen))
for cid in sorted(by_cid, key=lambda x: (x is None, x)):
    entries = sorted(by_cid[cid], key=lambda e: e[0]["lap_number"])
    marker = "  <=== C10, must not regress vs 0.67 baseline" if cid == 10 else ""
    fracs_c = [f for _, f, _ in entries]
    print(f"  C{cid}: mean={np.mean(fracs_c):.2f} min={np.min(fracs_c):.2f}{marker}")
    for c, f, blen in entries:
        tags = ",".join(w for w in ("compound_corner", "straddles_adjacent_corners", "canonical_quiet")
                         if w in c.get("warnings", []))
        print(f"      lap={c['lap_number']}: overlap={f:.3f}  bracket_len={blen:.2f}s"
              f"{'  [' + tags + ']' if tags else ''}")
