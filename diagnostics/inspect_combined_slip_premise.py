# Combined-slip arc, FIRST TASK (PLAN.md PARKED item, gated on the EKF
# arc closing): measurement-only test of whether the rear axle reaches
# meaningful longitudinal utilisation on exit, and the front under
# braking. Read-only, Tier B (signal/data engineering -- standard slip-
# ratio construction, no vehicle-dynamics claim). Nothing whitelisted in
# config/channels.json, no config written, no production path touched.
#
# Wheel-speed channels read directly from the raw Pi Toolbox file with
# the same bypass-the-whitelist block reader WP-S1 used
# (diagnostics/inspect_wheel_speed_sources.py) -- log_speed_fl/fr/rl/rr,
# the WP-S1-designated candidate family, byte-identical to
# ecu_speed_wheels_*.
#
# SLIP-RATIO DEFINITION: kappa_axle = (v_axle_corrected - v_ref) / v_ref,
# v_ref = ecu_speed (production channel). Chosen because WP-S1's own
# offsets were measured relative to ecu_speed already (thesis_notes.md
# "Wheel-speed source characterization (WP-S1)"), so this continues the
# same measurement chain rather than introducing GPS speed as a second,
# independently-caveated reference not otherwise part of this arc.
# ecu_speed's own provenance is opaque (config/parameters.json
# accuracy_levels.speed.capped_by) -- this slip ratio is PROVISIONAL for
# exactly that reason, as instructed.
#
# OFFSET CORRECTION (from WP-S1): rear axle reads a CONSTANT +1.41% high
# vs ecu_speed, diagnosed as a rolling-radius difference, not slip
# (throttle-independent: +1.44% on-throttle vs +1.41% overall) -- this
# is removed before computing rear slip: v_rear_corrected = v_rear_raw /
# 1.0141. Front axle reads ~0% off-braking (-0.03%) and needs no
# correction there; its -1.38% braking-specific deviation is WP-S1's own
# diagnosed front-wheel-slip-under-braking signature, i.e. exactly the
# signal this script is trying to measure -- it must NOT be subtracted
# out.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REAR_ROLLING_RADIUS_OFFSET = 0.0141  # WP-S1, constant, throttle-independent

WHEEL_NAMES = {
    "front": ["log_speed_fl", "log_speed_fr"],
    "rear": ["log_speed_rl", "log_speed_rr"],
}

# Proposed "meaningful longitudinal utilisation" threshold: kappa=0.05
# (5%). Justification (Tier B, literature-informed, not fitted to this
# data): peak longitudinal mu for a racing slick typically occurs near
# kappa=0.08-0.15 (Pacejka/Dugoff-family tyre models, e.g. Rajamani Ch.
# 2 Fig. 2.9 characteristic shape); half that value is comfortably past
# the small-slip linear region where longitudinal force is still
# growing close to linearly, so kappa>=0.05 indicates the tyre is doing
# real longitudinal work, not measurement noise on an unloaded wheel.
UTILISATION_THRESHOLD = 0.05


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


all_names = [n for names in WHEEL_NAMES.values() for n in names]
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

print("=" * 78)
print("BASE MASK CHECK (must match pass1_final_validation's n=24,183)")
print("=" * 78)
print(f"  base_mask = moving & ~kerb & valid-lap racing time -> n={int(base_mask.sum())}")
print()


def axle_speed_kmh(names):
    vals = [np.interp(t, raw[n]["time"], raw[n]["data"]) for n in names]
    return np.mean(vals, axis=0)


v_front_raw = axle_speed_kmh(WHEEL_NAMES["front"])
v_rear_raw = axle_speed_kmh(WHEEL_NAMES["rear"])
v_rear_corrected = v_rear_raw / (1.0 + REAR_ROLLING_RADIUS_OFFSET)
v_front_corrected = v_front_raw  # no correction, see header

kappa_front = (v_front_corrected - v_ecu_kmh) / v_ecu_kmh
kappa_rear = (v_rear_corrected - v_ecu_kmh) / v_ecu_kmh

# Corner-phase mask: entry_1_brake, exit_4, exit_5, from each corner's
# own c["segments"][phase] (start_t, end_t), same construction
# summarise_corners uses (modules/stability_analysis.py _phase_slice),
# restricted here to corners in the same valid-lap population as
# base_mask (corner dicts don't carry an is_valid_for_analysis flag of
# their own -- filtered via the corner's lap_number against valid laps).
corners = data.get("corners", [])
valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}

phase_mask = {p: np.zeros_like(t, dtype=bool) for p in ["entry_1_brake", "exit_4", "exit_5"]}
for c in corners:
    if c["lap_number"] not in valid_lap_numbers:
        continue
    for phase in phase_mask:
        start_t, end_t = c["segments"][phase]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi > lo:
            phase_mask[phase][lo:hi] = True

exit_mask = phase_mask["exit_4"] | phase_mask["exit_5"]
entry_brake_mask = phase_mask["entry_1_brake"]

print("=" * 78)
print("ITEM 3 -- |slip ratio| distribution per axle, base_mask population")
print("=" * 78)
for label, kappa in [("front", kappa_front), ("rear", kappa_rear)]:
    vals = np.abs(kappa[base_mask])
    vals = vals[np.isfinite(vals)]
    print(f"  {label:6s} n={len(vals):6d}  p50={np.percentile(vals,50)*100:.3f}%  "
          f"p90={np.percentile(vals,90)*100:.3f}%  p99={np.percentile(vals,99)*100:.3f}%  "
          f"max={np.max(vals)*100:.3f}%")
print()
print("  -- by corner phase (base_mask & phase, front and rear both reported per phase) --")
for phase_label, pmask in [("entry_1_brake", entry_brake_mask), ("exit_4", phase_mask["exit_4"]),
                            ("exit_5", phase_mask["exit_5"]), ("exit_4+5 combined", exit_mask)]:
    m = base_mask & pmask
    for label, kappa in [("front", kappa_front), ("rear", kappa_rear)]:
        vals = np.abs(kappa[m])
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            print(f"  {phase_label:18s} {label:6s} n=0")
            continue
        print(f"  {phase_label:18s} {label:6s} n={len(vals):6d}  p50={np.percentile(vals,50)*100:.3f}%  "
              f"p90={np.percentile(vals,90)*100:.3f}%  p99={np.percentile(vals,99)*100:.3f}%  "
              f"max={np.max(vals)*100:.3f}%")
print()

print("=" * 78)
print(f"ITEM 4 -- fraction above proposed utilisation threshold (kappa>={UTILISATION_THRESHOLD*100:.0f}%)")
print("=" * 78)
m_exit = base_mask & exit_mask
rear_exit_vals = np.abs(kappa_rear[m_exit])
rear_exit_vals = rear_exit_vals[np.isfinite(rear_exit_vals)]
frac_rear_exit = float(np.mean(rear_exit_vals >= UTILISATION_THRESHOLD)) if len(rear_exit_vals) else float("nan")
print(f"  REAR, exit phase (4+5): n={len(rear_exit_vals)}  "
      f"fraction |kappa|>={UTILISATION_THRESHOLD*100:.0f}% = {frac_rear_exit*100:.2f}%")

m_brake = base_mask & entry_brake_mask
front_brake_vals = np.abs(kappa_front[m_brake])
front_brake_vals = front_brake_vals[np.isfinite(front_brake_vals)]
frac_front_brake = float(np.mean(front_brake_vals >= UTILISATION_THRESHOLD)) if len(front_brake_vals) else float("nan")
print(f"  FRONT, entry_1_brake phase (SEE CAVEAT BELOW): n={len(front_brake_vals)}  "
      f"fraction |kappa|>={UTILISATION_THRESHOLD*100:.0f}% = {frac_front_brake*100:.2f}%")

# entry_1_brake's brake_start_t (corner_analysis.py _build_corner) takes
# off_throttle[0] -- the FIRST off-throttle sample anywhere earlier in
# the lap, not the last one before turn-in -- so the phase balloons for
# later corners in a lap (observed durations up to ~107s, see chat).
# Cross-check with a raw brake-pressure mask instead, same threshold
# WP-S1 already used (log_pbrake_f > 5 bar, diagnostics/inspect_wheel_
# speed_sources.py DRIVE_STATES).
brake_f_bar = state["brake_f_bar"]
m_brake_raw = base_mask & (brake_f_bar > 5.0)
front_brake_raw_vals = np.abs(kappa_front[m_brake_raw])
front_brake_raw_vals = front_brake_raw_vals[np.isfinite(front_brake_raw_vals)]
frac_front_brake_raw = float(np.mean(front_brake_raw_vals >= UTILISATION_THRESHOLD)) if len(front_brake_raw_vals) else float("nan")
print(f"  FRONT, log_pbrake_f>5bar (cross-check, WP-S1 convention): n={len(front_brake_raw_vals)}  "
      f"fraction |kappa|>={UTILISATION_THRESHOLD*100:.0f}% = {frac_front_brake_raw*100:.2f}%")
print(f"    distribution: p50={np.percentile(front_brake_raw_vals,50)*100:.3f}%  "
      f"p90={np.percentile(front_brake_raw_vals,90)*100:.3f}%  "
      f"p99={np.percentile(front_brake_raw_vals,99)*100:.3f}%  max={np.max(front_brake_raw_vals)*100:.3f}%")
