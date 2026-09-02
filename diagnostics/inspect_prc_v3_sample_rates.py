# Side-task census (2026-09-02): GT3_PRC_MLA-v3.txt (repo root, gitignored,
# untracked -- confirmed via git check-ignore, matches the existing blanket
# /*.txt rule, no gitignore change needed). Read-only, single streaming
# pass -- no analysis pipeline run, no config touched, no parser changes.
# Disposable per CLAUDE.md's diagnostics/ rule.
#
# Layout: per-channel BLOCK format (each channel gets its own
# "{ChannelBlock}" section with a "Time\t<channel>" header and its own
# two-column Time/value rows), confirmed by direct inspection -- NOT the
# wide-table layout the v1 Paul Ricard file used. Blocks are separated by
# exactly one blank line. This means there is no single shared row-grid to
# sample 3 fixed byte-offset windows from (the v1 census's own method) --
# instead, for each channel of interest, this script reads that channel's
# own full block (bounded, one channel's worth of rows) and reports rate
# from the start/middle/end thirds of ITS OWN row sequence. Channels not of
# interest are skipped (lines still consumed to reach the next block, but
# never parsed) to keep one single full-file pass fast.

RAW_FILE = "GT3_PRC_MLA-v3.txt"

NAMED_INTEREST = {
    "ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z",
    "lap_distance", "log_speed_fl", "log_speed_fr", "log_speed_rl", "log_speed_rr",
}
WILDCARD_PREFIXES = ("log_dms_dam_", "log_susp_travel_")
PATTERN_SUBSTRINGS = ("tc_lat", "tc_lon", "abs", "brake_bias")


def _rate_from_times(times):
    if len(times) < 2:
        return float("nan")
    return (len(times) - 1) / (times[-1] - times[0])


def _window_rates(times):
    n = len(times)
    if n < 6:
        return _rate_from_times(times), _rate_from_times(times), _rate_from_times(times)
    third = max(2, n // 20)  # a real window, not the whole block, at start/mid/end
    start = times[:third]
    mid_lo = n // 2 - third // 2
    mid = times[mid_lo: mid_lo + third]
    end = times[-third:]
    return _rate_from_times(start), _rate_from_times(mid), _rate_from_times(end)


def main():
    total_blocks = 0
    all_channel_names = []
    pattern_matches = []
    detailed = {}  # channel -> {"n": int, "n_nan": int, "duration_s": float, "start_hz", "mid_hz", "end_hz"}

    with open(RAW_FILE, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.strip() == "{ChannelBlock}":
                total_blocks += 1
                header = f.readline()
                # header shape: "Time\t<channel>[unit]" -- strip any [unit] suffix
                parts = header.rstrip("\n").split("\t")
                raw_name = parts[1] if len(parts) > 1 else ""
                channel = raw_name.split("[")[0]
                all_channel_names.append(channel)

                lname = channel.lower()
                is_wildcard = channel.startswith(WILDCARD_PREFIXES)
                is_named = channel in NAMED_INTEREST
                is_pattern = any(p in lname for p in PATTERN_SUBSTRINGS)
                if is_pattern:
                    pattern_matches.append(channel)

                if is_wildcard or is_named:
                    times = []
                    n_nan = 0
                    row = f.readline()
                    while row and row != "\n":
                        cols = row.rstrip("\n").split("\t")
                        try:
                            t = float(cols[0])
                        except (ValueError, IndexError):
                            row = f.readline()
                            continue
                        times.append(t)
                        val = cols[1] if len(cols) > 1 else ""
                        if val == "" or val.strip().lower() in ("nan", "n/a"):
                            n_nan += 1
                        row = f.readline()
                    start_hz, mid_hz, end_hz = _window_rates(times)
                    duration = (times[-1] - times[0]) if len(times) >= 2 else float("nan")
                    detailed[channel] = {
                        "n": len(times), "n_nan": n_nan, "duration_s": duration,
                        "start_hz": start_hz, "mid_hz": mid_hz, "end_hz": end_hz,
                        "overall_hz": _rate_from_times(times),
                    }
                else:
                    # Skip -- consume rows to reach the next block, no parsing.
                    row = f.readline()
                    while row and row != "\n":
                        row = f.readline()
            line = f.readline()
            if total_blocks % 200 == 0 and total_blocks > 0 and line == "":
                pass  # EOF reached naturally

    print(f"total ChannelBlock sections: {total_blocks}")
    print(f"total distinct channel names: {len(set(all_channel_names))}")

    print(f"\n=== named/wildcard channels of interest (n={len(detailed)}) ===")
    for name in sorted(detailed):
        d = detailed[name]
        nan_frac = d["n_nan"] / d["n"] if d["n"] else float("nan")
        print(f"  {name}: n={d['n']} duration={d['duration_s']:.1f}s overall={d['overall_hz']:.2f}Hz "
              f"start={d['start_hz']:.2f}Hz mid={d['mid_hz']:.2f}Hz end={d['end_hz']:.2f}Hz "
              f"nan_fraction={nan_frac:.4f}")

    print(f"\n=== tc_lat/tc_lon/abs/brake_bias pattern matches (n={len(pattern_matches)}) ===")
    for name in pattern_matches:
        print(f"  {name}")


if __name__ == "__main__":
    main()
