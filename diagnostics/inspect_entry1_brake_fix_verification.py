# Verification of the entry_1_brake phase-boundary fix, SECOND PASS
# (modules/corner_analysis.py line ~293-300, corrected construction:
# on_throttle = where(thr_d >= brake_throttle_max_pct), brake_start_t
# = thr_t[on_throttle[-1]] -- the last FULL-throttle sample before
# turn-in, i.e. the lift-off transition, replacing the first attempt's
# off_throttle[-1] which found the wrong end of the window). Read-only.
#
#   (a) per-corner durations post-fix: bounded, non-monotonic, no
#       longer collapsing to single samples for corners 1/5/7/13
#   (b) MANDATORY: corrected brake_start_t vs log_pbrake_f/log_pbrake_r
#       rise, distribution across all corners/laps
#   (c) total entry_1_brake population share
#   (d) hand spot-check: corner 1 lap 1 (the exposing case) + a late
#       corner, against the raw throttle trace
#   plus: empty-case frequency (item 2) and inherited-lookback risk
#   against the PRECEDING corner's own bracket (item 3)

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask
print(f"base_mask n={int(base_mask.sum())} (must still be 24183)")
print()

corners = data.get("corners", [])
valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
corners_valid = [c for c in corners if c["lap_number"] in valid_lap_numbers]

print("=" * 78)
print("(a) per-corner entry_1_brake durations post-fix (corrected construction)")
print("=" * 78)
durs = []
for c in corners_valid:
    st, et = c["segments"]["entry_1_brake"]
    durs.append((c["lap_number"], c["corner_number"], c.get("stable_corner_id"), et - st))
for d in durs:
    flag = "  <-- was collapsed (~0.01s) under the first fix attempt" \
        if d[2] in (1, 5, 7, 14) else ""  # stable_ids for corner_number 1/5/7/13 per earlier mapping
    print(f"  lap={d[0]}  corner_number={d[1]}  stable_id={d[2]}  duration={d[3]:.3f}s{flag}")
vals = np.array([d[3] for d in durs])
print(f"  n={len(vals)}  mean={vals.mean():.3f}s  median={np.median(vals):.3f}s  "
      f"max={vals.max():.3f}s  min={vals.min():.3f}s")
print()

print("=" * 78)
print("(c) total entry_1_brake population share")
print("=" * 78)
phase_mask = np.zeros_like(t, dtype=bool)
for c in corners_valid:
    st, et = c["segments"]["entry_1_brake"]
    if et < st:
        continue
    lo = int(np.searchsorted(t, st, side="left"))
    hi = int(np.searchsorted(t, et, side="right"))
    if hi > lo:
        phase_mask[lo:hi] = True
m = base_mask & phase_mask
print(f"  entry_1_brake covers {int(m.sum())}/{int(base_mask.sum())} masked samples "
      f"= {100*m.sum()/base_mask.sum():.2f}%  (pre-fix 85.35%; first-attempt-fix 7.62%)")
print()

print("=" * 78)
print("EMPTY-CASE frequency (on_throttle empty -- no full-throttle sample in lookback)")
print("=" * 78)
throttle_ch = data["channels"]["ecu_aps"]
laps_by_number = {l["lap_number"]: l for l in laps}
n_empty = 0
n_total_checked = 0
for c in corners_valid:
    lap = laps_by_number.get(c["lap_number"])
    if lap is None:
        continue
    s_t_start_abs = c["segments"]["entry_2_turnin"][0]
    window_mask = (throttle_ch["time"] >= lap["start_time"]) & (throttle_ch["time"] < s_t_start_abs)
    if not window_mask.any():
        continue
    n_total_checked += 1
    wd = throttle_ch["data"][window_mask]
    if not np.any(wd >= 95.0):
        n_empty += 1
        print(f"  EMPTY CASE: lap={c['lap_number']} corner_number={c['corner_number']} "
              f"stable_id={c.get('stable_corner_id')} -- no full-throttle sample found, "
              f"brake_start_t defaults to turn-in (zero-length phase)")
print(f"  {n_empty} / {n_total_checked} corner instances hit the empty case")
print()

print("=" * 78)
print("(b) MANDATORY -- corrected brake_start_t vs log_pbrake_f/log_pbrake_r rise")
print("=" * 78)


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
                                        tt = float(parts[0].replace(",", "."))
                                        vv = float(parts[1].replace(",", "."))
                                        times.append(tt)
                                        values.append(vv)
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


raw = read_raw_channels(RAW_FILE, ["log_pbrake_f", "log_pbrake_r"])
BRAKE_RISE_BAR = 5.0

offsets_f, offsets_r = [], []
for c in corners_valid:
    brake_start_t, s_t_start = c["segments"]["entry_1_brake"]
    for label, offsets in [("log_pbrake_f", offsets_f), ("log_pbrake_r", offsets_r)]:
        ch = raw[label]
        window_mask = (ch["time"] >= brake_start_t) & (ch["time"] <= s_t_start + 5.0)
        if not window_mask.any():
            continue
        wt = ch["time"][window_mask]
        wd = ch["data"][window_mask]
        rise_idx = np.where(wd > BRAKE_RISE_BAR)[0]
        if len(rise_idx) == 0:
            continue
        rise_t = wt[rise_idx[0]]
        offsets.append((c["lap_number"], c["corner_number"], rise_t - brake_start_t))

for label, offsets in [("log_pbrake_f", offsets_f), ("log_pbrake_r", offsets_r)]:
    if not offsets:
        print(f"  {label}: no corners with a brake-pressure rise found in window")
        continue
    o = np.array([x[2] for x in offsets])
    print(f"  {label}: n={len(o)}")
    print(f"    p10={np.percentile(o,10):.3f}  p50={np.median(o):.3f}  p90={np.percentile(o,90):.3f}  "
          f"mean={o.mean():.3f}  max={o.max():.3f}")
    print(f"    fraction negative (brake before lift): {np.mean(o<0)*100:.1f}%")
print()
print("  Expected signature: consistently small POSITIVE gap (lift, brief coast, then brake).")
print("  A median near zero or negative values would indicate the construction is still wrong.")
print()

print("=" * 78)
print("(d) hand spot-check: corner 1 lap 1 (the exposing case) + a late corner")
print("=" * 78)
for target_corner_number in [1, 12, 13]:
    match = [c for c in corners_valid if c["lap_number"] == 1 and c["corner_number"] == target_corner_number]
    if not match:
        print(f"  corner_number={target_corner_number}, lap 1: not found")
        continue
    c = match[0]
    brake_start_t, s_t_start = c["segments"]["entry_1_brake"]
    print(f"  corner_number={target_corner_number}  stable_id={c.get('stable_corner_id')}  "
          f"brake_start_t={brake_start_t:.3f}s  s_t_start(turn-in)={s_t_start:.3f}s  "
          f"duration={s_t_start-brake_start_t:.3f}s")
    window_mask = (throttle_ch["time"] >= brake_start_t - 2.0) & (throttle_ch["time"] <= brake_start_t + 1.0)
    wt = throttle_ch["time"][window_mask]
    wd = throttle_ch["data"][window_mask]
    idx_at = np.argmin(np.abs(wt - brake_start_t))
    print(f"    throttle at brake_start_t (nearest sample): {wd[idx_at]:.1f}%  "
          f"(config brake_throttle_max_pct=95)")
    before = wd[wt < brake_start_t]
    after = wd[wt >= brake_start_t]
    print(f"    throttle 2s before: min={before.min() if len(before) else float('nan'):.1f}%  "
          f"max={before.max() if len(before) else float('nan'):.1f}%")
    print(f"    throttle 1s after:  min={after.min() if len(after) else float('nan'):.1f}%  "
          f"max={after.max() if len(after) else float('nan'):.1f}%")
print()

print("=" * 78)
print("(3) INHERITED-LOOKBACK RISK -- did any corner's brake_start_t cross the")
print("    PRECEDING corner's own bracket end (same lap)?")
print("=" * 78)
by_lap = {}
for c in corners_valid:
    by_lap.setdefault(c["lap_number"], []).append(c)
n_crossed = 0
for lap_num, lap_corners in by_lap.items():
    lap_corners_sorted = sorted(lap_corners, key=lambda c: c["apex_time"])
    for i in range(1, len(lap_corners_sorted)):
        prev_c = lap_corners_sorted[i - 1]
        cur_c = lap_corners_sorted[i]
        prev_end = prev_c["segments"]["exit_5"][1]
        cur_brake_start = cur_c["segments"]["entry_1_brake"][0]
        if cur_brake_start < prev_end:
            n_crossed += 1
            print(f"  CROSSED: lap={lap_num} corner_number={cur_c['corner_number']} "
                  f"(stable_id={cur_c.get('stable_corner_id')}) brake_start_t={cur_brake_start:.3f}s "
                  f"reaches back before previous corner_number={prev_c['corner_number']}'s own bracket "
                  f"end ({prev_end:.3f}s), overlap={prev_end - cur_brake_start:.3f}s")
if n_crossed == 0:
    print("  none found -- no corner's corrected brake_start_t reaches back into the preceding")
    print("  corner's own bracket in this session's data.")
else:
    print(f"  {n_crossed} instance(s) found -- the inherited unbounded-lookback risk DOES occur.")
