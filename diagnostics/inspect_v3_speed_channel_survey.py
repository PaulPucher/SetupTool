# Frame-Stage-2 work package, Phase 1(a): speed-channel census on
# GT3_PRC_MLA-v3.txt, read-only. The work order's own stated premise
# ("census said gps_speed absent") is checked directly per CLAUDE.md's
# channel-census rule, not trusted -- a broad header grep (this phase)
# found several GPS/radar/corrected-speed candidates the prior census
# did not name. This script streams the file once, collecting every
# candidate speed channel's own full time series for later analysis.

import io
import numpy as np

RAW_FILE = "GT3_PRC_MLA-v3.txt"
ENCODING = "latin-1"

TARGETS = [
    "ecu_speed[kph]",
    "log_speed_fl[kph]", "log_speed_fr[kph]", "log_speed_rl[kph]", "log_speed_rr[kph]",
    "abs_speed_fl[kph]", "abs_speed_fr[kph]", "abs_speed_rl[kph]", "abs_speed_rr[kph]",
    "ecu_speed_fl[kph]", "ecu_speed_fr[kph]", "ecu_speed_rl[kph]", "ecu_speed_rr[kph]",
    "log_gps_speed[kph]", "gpsa_speed[kph]",
    "corr_speed[kph]", "corr_speed_x[km/h]", "corr_speed_y[km/h]",
    "Corrected Speed[kph]", "DTM_speed_TTL[kph]", "MRR_EgoVehSpeed[km/h]",
    "Math_Speed_T1[kph]",
    "NMEA RX RMC Speed Over Ground[kph]", "NMEA RX VTG Speed Ground[kph]",
]


def read_blocks():
    blocks = {name: ([], []) for name in TARGETS}
    current = None
    with io.open(RAW_FILE, encoding=ENCODING) as f:
        for line in f:
            if line.startswith("Time\t"):
                name = line.strip().split("\t", 1)[1]
                current = name if name in TARGETS else None
                continue
            if current is None:
                continue
            parts = line.strip().split("\t")
            if len(parts) != 2:
                continue
            try:
                t = float(parts[0])
                v = float(parts[1])
            except ValueError:
                continue
            blocks[current][0].append(t)
            blocks[current][1].append(v)
    return {name: (np.array(t), np.array(v)) for name, (t, v) in blocks.items()}


def main():
    print(f"streaming {RAW_FILE} once for {len(TARGETS)} candidate speed channels...")
    blocks = read_blocks()
    for name in TARGETS:
        t, v = blocks[name]
        if len(v) == 0:
            print(f"{name}: ABSENT (no header found in file)")
            continue
        n_distinct = len(np.unique(v))
        print(f"{name}: n={len(v)}, rate~{len(v)/(t[-1]-t[0]):.1f}Hz, span=[{t[0]:.1f},{t[-1]:.1f}]s, "
              f"n_distinct={n_distinct}, mean={v.mean():.3f}, std={v.std():.3f}, "
              f"min={v.min():.3f}, max={v.max():.3f}")


if __name__ == "__main__":
    main()
