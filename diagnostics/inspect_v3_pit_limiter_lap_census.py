# Fz-integration Phase 4, pre-implementation census (2026-09-03).
# Read-only. Censuses v3's real lap_number/ecu_B_speedlimit_en/lap_
# distance/beacon channels around the session start and end, and Dubai's
# for comparison -- per CLAUDE.md's own standing rule (channel presence
# and content are censused from the file, never recalled) before writing
# any classification logic.

import numpy as np

from modules.csv_parser import parse_csv

FILES = [
    ("dubai", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("v3", "GT3_PRC_MLA-v3.txt"),
]


def _describe_channel(channels, name):
    ch = channels.get(name)
    if ch is None:
        return f"{name}: MISSING from channel dict"
    if ch.get("quality") in ("missing", "failed"):
        return f"{name}: quality={ch['quality']!r}"
    t, d = ch["time"], ch["data"]
    return f"{name}: quality={ch['quality']!r} n={len(d)} range=[{np.nanmin(d):.3f},{np.nanmax(d):.3f}] t=[{t[0]:.2f},{t[-1]:.2f}]s"


def main():
    for label, raw_file in FILES:
        print(f"\n{'='*90}\n{label}: {raw_file}\n{'='*90}")
        data = parse_csv(raw_file)
        channels = data["channels"]
        laps = data["laps"]

        for ch_name in ("lap_number", "ecu_B_speedlimit_en", "lap_distance", "lap_time"):
            print(" ", _describe_channel(channels, ch_name))

        print(f"\n  {len(laps)} laps detected:")
        for lap in laps:
            print(f"    lap {lap['lap_number']:>3}: t=[{lap['start_time']:.2f},{lap['end_time']:.2f}]s "
                  f"dur={lap['lap_time']:.2f}s is_outlap={lap['is_outlap']} is_inlap={lap['is_inlap']} "
                  f"is_valid_for_analysis={lap['is_valid_for_analysis']} is_fastest={lap['is_fastest']} "
                  f"warnings={lap['warnings']}")

        limiter_ch = channels.get("ecu_B_speedlimit_en")
        if limiter_ch is not None and limiter_ch.get("quality") not in ("missing", "failed"):
            t, d = limiter_ch["time"], limiter_ch["data"]
            active = d >= 0.5
            # Find contiguous active runs.
            edges = np.diff(active.astype(int))
            starts = np.where(edges == 1)[0] + 1
            ends = np.where(edges == -1)[0] + 1
            if active[0]:
                starts = np.concatenate(([0], starts))
            if active[-1]:
                ends = np.concatenate((ends, [len(active)]))
            print(f"\n  ecu_B_speedlimit_en active runs ({len(starts)} total):")
            for s, e in zip(starts, ends):
                dur = t[e - 1] - t[s]
                print(f"    t=[{t[s]:.2f},{t[e-1]:.2f}]s dur={dur:.2f}s")
                # which lap(s) does this run fall inside/span?
                for lap in laps:
                    if t[e - 1] >= lap["start_time"] and t[s] <= lap["end_time"]:
                        print(f"      overlaps lap {lap['lap_number']} "
                              f"[{lap['start_time']:.2f},{lap['end_time']:.2f}]s")
        else:
            print("\n  ecu_B_speedlimit_en not usable this session.")

        # First/last lap_distance behaviour (does distance reset cleanly at
        # lap boundaries, or does the FIRST lap start mid-distance -- a
        # signature of a pit-box-before-line layout, where the car is
        # already part-way down the lap distance measure when lap_number
        # first increments)?
        ld = channels.get("lap_distance")
        if ld is not None and ld.get("quality") not in ("missing", "failed"):
            t, d = ld["time"], ld["data"]
            for lap in (laps[0], laps[-1]):
                mask = (t >= lap["start_time"]) & (t <= lap["end_time"])
                if mask.any():
                    dm = d[mask]
                    print(f"\n  lap_distance during lap {lap['lap_number']} "
                          f"(is_outlap={lap['is_outlap']}, is_inlap={lap['is_inlap']}): "
                          f"first={dm[0]:.1f} last={dm[-1]:.1f} min={dm.min():.1f} max={dm.max():.1f}")


if __name__ == "__main__":
    main()
