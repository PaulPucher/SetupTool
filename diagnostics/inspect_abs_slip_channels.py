# Rolling-radius follow-up, QUEUED ITEM 2: abs_Slip_FL/FR/RL/RR[%] --
# found but not examined in the earlier combined-slip premise turn.
# Read-only, Tier B (signal/data engineering -- descriptive statistics
# and a direct reconstruction check, no vehicle-dynamics claim).
# Nothing whitelisted in config/channels.json, no config written.
#
# Question: if the ABS computes its own per-wheel slip using its own
# constants CONSISTENTLY within its own domain, that internal
# consistency is what a slip ratio needs -- and unlike abs_circ_f/r
# (a configured, field-retuned control parameter, see thesis_notes.md
# supersede note) a channel the ABS unit LOGS as its own live slip
# output is itself a Level-3 logged quantity, not a constant assumed
# to still hold. Three checks: (1) distribution of abs_Slip_* over the
# combined-slip premise turn's base_mask population; (2) internal
# reconstruction -- does abs_Slip_FL match (abs_speed_fl -
# abs_vVeh_absRef) / abs_vVeh_absRef, i.e. is it built from channels
# already in hand, or does it depend on something not visible here
# (e.g. per-wheel load, a different reference); (3) comparison against
# the log_speed_*-derived provisional kappa from diagnostics/inspect_
# combined_slip_premise.py.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REAR_ROLLING_RADIUS_OFFSET = 0.0141  # WP-S1, same correction used in the premise-test script

SLIP_NAMES = ["abs_Slip_FL", "abs_Slip_FR", "abs_Slip_RL", "abs_Slip_RR"]
SPEED_NAMES = ["abs_speed_fl", "abs_speed_fr", "abs_speed_rl", "abs_speed_rr"]
REF_NAME = "abs_vVeh_absRef"
LOG_SPEED_FRONT = ["log_speed_fl", "log_speed_fr"]
LOG_SPEED_REAR = ["log_speed_rl", "log_speed_rr"]

MPH_TO_KMH = 1.609344


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


all_names = SLIP_NAMES + SPEED_NAMES + [REF_NAME] + LOG_SPEED_FRONT + LOG_SPEED_REAR
raw = read_raw_channels(RAW_FILE, all_names)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
v_ecu_kmh = state["v_mps"] * 3.6
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask
print(f"base_mask n={int(base_mask.sum())} (must match 24183)")
print()


def interp_onto_t(name):
    return np.interp(t, raw[name]["time"], raw[name]["data"])


print("=" * 78)
print("CHECK 1 -- abs_Slip_* distribution over base_mask")
print("=" * 78)
slip_on_t = {}
for name in SLIP_NAMES:
    vals = interp_onto_t(name)
    slip_on_t[name] = vals
    m = base_mask & np.isfinite(vals)
    v = vals[m]
    print(f"  {name:14s} n={len(v):6d}  p50={np.median(v):+.4f}%  "
          f"p90={np.percentile(np.abs(v),90):.4f}%  p99={np.percentile(np.abs(v),99):.4f}%  "
          f"max|.|={np.max(np.abs(v)):.4f}%  (signed p1={np.percentile(v,1):+.4f}%, "
          f"p99={np.percentile(v,99):+.4f}%)")
print()

print("=" * 78)
print("CHECK 2 -- internal reconstruction: abs_Slip_FL vs (abs_speed_fl - abs_vVeh_absRef)/abs_vVeh_absRef")
print("=" * 78)
ref = interp_onto_t(REF_NAME)
for slip_name, speed_name in zip(SLIP_NAMES, SPEED_NAMES):
    spd = interp_onto_t(speed_name)
    predicted_pct = (spd - ref) / ref * 100.0
    actual_pct = slip_on_t[slip_name]
    m = base_mask & np.isfinite(predicted_pct) & np.isfinite(actual_pct) & (ref > 5.0)
    diff = predicted_pct[m] - actual_pct[m]
    corr = np.corrcoef(predicted_pct[m], actual_pct[m])[0, 1] if m.sum() > 1 else float("nan")
    print(f"  {slip_name:14s} n={int(m.sum()):6d}  corr(predicted,actual)={corr:+.4f}  "
          f"diff mean={np.mean(diff):+.4f} pct-pts  diff std={np.std(diff):.4f}  "
          f"diff median={np.median(diff):+.4f}")
print()

print("=" * 78)
print("CHECK 3 -- abs_Slip_* (axle-averaged) vs the log_speed_*-derived provisional kappa")
print("=" * 78)


def axle_speed_kmh(names, source):
    vals = [np.interp(t, raw[n]["time"], source(n)) for n in names]
    return np.mean(vals, axis=0)


v_front_raw = np.mean([interp_onto_t(n) for n in LOG_SPEED_FRONT], axis=0)
v_rear_raw = np.mean([interp_onto_t(n) for n in LOG_SPEED_REAR], axis=0)
v_rear_corrected = v_rear_raw / (1.0 + REAR_ROLLING_RADIUS_OFFSET)
kappa_front_pct = (v_front_raw - v_ecu_kmh) / v_ecu_kmh * 100.0
kappa_rear_pct = (v_rear_corrected - v_ecu_kmh) / v_ecu_kmh * 100.0

abs_slip_front_pct = np.mean([slip_on_t["abs_Slip_FL"], slip_on_t["abs_Slip_FR"]], axis=0)
abs_slip_rear_pct = np.mean([slip_on_t["abs_Slip_RL"], slip_on_t["abs_Slip_RR"]], axis=0)

for label, kappa, abs_slip in [("front", kappa_front_pct, abs_slip_front_pct),
                                 ("rear", kappa_rear_pct, abs_slip_rear_pct)]:
    m = base_mask & np.isfinite(kappa) & np.isfinite(abs_slip)
    diff = kappa[m] - abs_slip[m]
    corr = np.corrcoef(kappa[m], abs_slip[m])[0, 1] if m.sum() > 1 else float("nan")
    print(f"  {label:6s} n={int(m.sum())}  corr(provisional kappa, abs_Slip)={corr:+.4f}  "
          f"provisional p50={np.median(kappa[m]):+.4f}%  abs_Slip p50={np.median(abs_slip[m]):+.4f}%  "
          f"diff (provisional-abs_Slip) mean={np.mean(diff):+.4f}  median={np.median(diff):+.4f}  std={np.std(diff):.4f}")
