# Rolling-radius follow-up, QUEUED ITEM 1: is WP-S1's measured wheel-
# speed offset (log_speed_* vs ecu_speed) FLAT or SPEED-DEPENDENT?
# Read-only, Tier B (signal/data engineering -- standard binned
# descriptive statistics, no vehicle-dynamics claim, no attribution to
# a physical cause attempted or required). Nothing whitelisted, no
# config written.
#
# Reuses WP-S1's own straight-line population (moving & valid-lap &
# |ay|<=0.15g & |yaw rate|<=3.0 deg/s, diagnostics/inspect_wheel_
# speed_sources.py) so this measures the same thing WP-S1 measured,
# just resolved by speed instead of collapsed to one summary number.
# Front is further restricted to log_pbrake_f>5bar (WP-S1's braking
# population) since WP-S1 diagnosed a braking-specific front effect,
# not an all-speed one.
#
# A flat offset across bins means a single scale factor explains it
# (consistent with WP-S1's own rolling-radius/slip diagnoses). A
# trending offset means something speed-dependent sits on top and no
# single constant would be sufficient. Reported as observed; no
# interpretation forced either way.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
AY_STRAIGHT_MAX_G = 0.15
YAW_STRAIGHT_MAX_DEGPS = 3.0
BRAKING_MIN_BAR = 5.0
N_BINS = 8
MIN_BIN_N = 20  # below this, report but flag as low-n rather than suppress


def read_raw_channels(file_path, wanted_names):
    wanted = set(wanted_names)
    out = {}
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
                    channel_name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
                    i += 1
                    if channel_name in wanted:
                        times, values = [], []
                        while i < n and not lines[i].strip().startswith("{"):
                            raw_line = lines[i].strip()
                            if raw_line:
                                parts = raw_line.split("\t")
                                if len(parts) == 2:
                                    try:
                                        t = float(parts[0].replace(",", "."))
                                        v = float(parts[1].replace(",", "."))
                                        times.append(t)
                                        values.append(v)
                                    except ValueError:
                                        pass
                            i += 1
                        out[channel_name] = {"time": np.array(times), "data": np.array(values)}
                        continue
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                        continue
        i += 1
    return out


REAR_NAMES = ["log_speed_rl", "log_speed_rr"]
FRONT_NAMES = ["log_speed_fl", "log_speed_fr"]
raw = read_raw_channels(RAW_FILE, REAR_NAMES + FRONT_NAMES)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
v_ecu_kmh = state["v_mps"] * 3.6
ay_g = state["ay_mps2"] / 9.81
yaw_rate_degps = state["yaw_rate_radps"] * 180.0 / np.pi
moving = state["moving_mask"]
brake_f_bar = state["brake_f_bar"]

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)

straight_mask = moving & racing_mask & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)


def axle_speed_kmh(names):
    vals = [np.interp(t, raw[n]["time"], raw[n]["data"]) for n in names]
    return np.mean(vals, axis=0)


v_rear_kmh = axle_speed_kmh(REAR_NAMES)
v_front_kmh = axle_speed_kmh(FRONT_NAMES)

rear_offset = (v_rear_kmh - v_ecu_kmh) / v_ecu_kmh
front_offset = (v_front_kmh - v_ecu_kmh) / v_ecu_kmh


def report_binned(label, mask, offset, speed):
    m = mask & np.isfinite(offset) & np.isfinite(speed)
    n_total = int(m.sum())
    print(f"--- {label}: n={n_total} ---")
    if n_total < N_BINS * MIN_BIN_N // 2:
        print(f"  too few samples for {N_BINS} bins, reporting anyway with wider bins")
    v = speed[m]
    o = offset[m]
    edges = np.linspace(v.min(), v.max(), N_BINS + 1)
    for i in range(N_BINS):
        lo, hi = edges[i], edges[i + 1]
        in_bin = (v >= lo) & (v <= hi if i == N_BINS - 1 else v < hi)
        n_bin = int(in_bin.sum())
        if n_bin == 0:
            print(f"  [{lo:6.1f}-{hi:6.1f}) km/h  n=0")
            continue
        vals = o[in_bin]
        flag = "" if n_bin >= MIN_BIN_N else "  (LOW N)"
        print(f"  [{lo:6.1f}-{hi:6.1f}) km/h  n={n_bin:5d}  "
              f"mean={np.mean(vals)*100:+.3f}%  median={np.median(vals)*100:+.3f}%  "
              f"std={np.std(vals)*100:.3f}%{flag}")
    # simple linear trend check: slope of offset vs speed, least squares
    if n_total >= 10:
        slope, intercept = np.polyfit(v, o, 1)
        span = v.max() - v.min()
        print(f"  linear fit: offset = {intercept*100:+.4f}% + {slope*100:+.5f}%/(km/h) * v  "
              f"-- predicted swing across observed speed range: {slope*span*100:+.3f} pct pts")
    print()


print("=" * 78)
print("QUEUED ITEM 1 -- speed-dependence of the measured wheel-speed offset")
print("=" * 78)
print(f"Straight-line population: moving & valid-lap & |ay|<={AY_STRAIGHT_MAX_G}g "
      f"& |yaw rate|<={YAW_STRAIGHT_MAX_DEGPS} deg/s (WP-S1's own definition)")
print()

report_binned("REAR (log_speed_r vs ecu_speed), straight-line, all drive states",
              straight_mask, rear_offset, v_ecu_kmh)
report_binned("FRONT (log_speed_f vs ecu_speed), straight-line & braking (log_pbrake_f>5bar)",
              straight_mask & (brake_f_bar > BRAKING_MIN_BAR), front_offset, v_ecu_kmh)
