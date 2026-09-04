# Fz-integration Phase 5, pre-implementation census (2026-09-03).
# Read-only. Two parts: (1) a raw-channel-name scan of GT3_PRC_MLA-v3.txt
# (streamed, not loaded whole -- the file is 1.29GB) for any ABS-domain
# wheel-SPEED channel (distinct from the already-known ABS-domain
# POSITION/setting channels, damper package Phase 4 census); (2) a
# per-channel plausibility diagnosis of log_speed_rr on v3 (dropouts,
# stuck/frozen segments, spikes vs its own mates and ecu_speed), same
# std/near-zero-variance-window discipline as modules.wheel_loads's own
# dead-channel guard (Fz-integration Phase 1).

import re

import numpy as np

from modules.csv_parser import parse_csv

RAW_FILE = "GT3_PRC_MLA-v3.txt"

WHEEL_SPEED_PATTERN = re.compile(r"(abs|ws|wheel).*speed|speed.*(abs|ws|wheel)|abs.*ws|ws.*abs", re.IGNORECASE)


def _scan_raw_channel_names(raw_file):
    names = []
    with open(raw_file, "r", encoding="latin-1") as f:
        prev = None
        for line in f:
            s = line.strip()
            if prev == "{ChannelBlock}":
                parts = s.split("\t")
                if len(parts) >= 2 and parts[0] == "Time":
                    # narrow: exactly one more column; wide: many more --
                    # either way, every subsequent column IS a channel name.
                    for raw in parts[1:]:
                        name = raw.split("[")[0].strip()
                        if name:
                            names.append(name)
            prev = s
    return names


def _channel_stats(channels, name, window_samples=25):
    ch = channels.get(name)
    if ch is None:
        return f"{name}: not in the whitelisted channel set"
    if ch.get("quality") in ("missing", "failed"):
        return f"{name}: quality={ch['quality']!r}"
    t, d = ch["time"], ch["data"]
    n = len(d)
    nan_frac = float(np.isnan(d).mean())
    # Rolling-window std (same style as the dead-channel guard) to find
    # STUCK/frozen segments, not just a whole-session std that a brief
    # stuck period wouldn't show up in.
    finite = d[np.isfinite(d)]
    stuck_windows = 0
    total_windows = 0
    for i in range(0, n - window_samples, window_samples):
        w = d[i:i + window_samples]
        if np.all(np.isfinite(w)):
            total_windows += 1
            if np.std(w) < 0.05:  # near-zero variance for a speed channel (kph), a generous floor
                stuck_windows += 1
    stuck_frac = stuck_windows / total_windows if total_windows else float("nan")
    return (f"{name}: n={n} nan_frac={nan_frac*100:.2f}% "
            f"range=[{np.nanmin(d):.2f},{np.nanmax(d):.2f}] mean={np.nanmean(finite):.2f} "
            f"stuck_window_frac={stuck_frac*100:.2f}% (window={window_samples} samples, std<0.05kph)")


def main():
    print("=== (1) raw channel name scan for ABS-domain wheel SPEED candidates ===")
    names = _scan_raw_channel_names(RAW_FILE)
    print(f"total raw channels in file: {len(names)}")
    hits = sorted(set(n for n in names if WHEEL_SPEED_PATTERN.search(n)))
    print(f"pattern matches ({{abs,ws,wheel}} x speed, either order): {len(hits)}")
    for h in hits:
        print(f"  {h}")

    print("\n=== (2) log_speed_rr plausibility diagnosis, v3 ===")
    data = parse_csv(RAW_FILE)
    channels = data["channels"]
    for name in ("log_speed_fl", "log_speed_fr", "log_speed_rl", "log_speed_rr", "ecu_speed"):
        print(" ", _channel_stats(channels, name))

    rr = channels.get("log_speed_rr")
    rl = channels.get("log_speed_rl")
    ecu = channels.get("ecu_speed")
    if rr and rl and ecu and rr.get("quality") not in ("missing", "failed"):
        t_ref = rr["time"]
        rr_v = rr["data"]
        rl_v = np.interp(t_ref, rl["time"], rl["data"])
        ecu_v = np.interp(t_ref, ecu["time"], ecu["data"])
        moving = ecu_v > 20.0  # kph, comfortably above any pit/stationary noise
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio_to_mate = np.where(moving, rr_v / np.where(rl_v != 0, rl_v, np.nan), np.nan)
            ratio_to_ecu = np.where(moving, rr_v / np.where(ecu_v != 0, ecu_v, np.nan), np.nan)
        print(f"\n  rr/rl ratio (moving only): mean={np.nanmean(ratio_to_mate):.4f} "
              f"std={np.nanstd(ratio_to_mate):.4f} "
              f"p1={np.nanpercentile(ratio_to_mate,1):.4f} p99={np.nanpercentile(ratio_to_mate,99):.4f}")
        print(f"  rr/ecu_speed ratio (moving only): mean={np.nanmean(ratio_to_ecu):.4f} "
              f"std={np.nanstd(ratio_to_ecu):.4f} "
              f"p1={np.nanpercentile(ratio_to_ecu,1):.4f} p99={np.nanpercentile(ratio_to_ecu,99):.4f}")
        implausible = moving & (np.abs(ratio_to_mate - 1.0) > 0.15)
        print(f"  fraction of moving samples with |rr/rl - 1| > 15%: {float(np.mean(implausible[moving]))*100:.2f}%")


if __name__ == "__main__":
    main()
