# Rolling-radius follow-up, QUEUED ITEMS 1 + 3 + TC-active extension of
# ITEM 4. Read-only, Tier B, nothing whitelisted, no config written.
#
# ITEM 1: test kappa ~= -100 * abs_Slip (sign-inversion + percent/
# fraction labelling-error hypothesis raised after last turn's -0.90/
# -0.55 correlations and ~50-75x magnitude ratio).
#
# ITEM 3: driven-vs-undriven slip, kappa_driven = (v_rear-v_front)/
# v_front, the RWD traction-control definition -- rear against front,
# not against a vehicle reference. Raw and WP-S1-corrected.
#
# TC-ACTIVE EXTENSION (item 4 continued): ecu_B_tc_act and ecu_slip_act
# /ecu_slip_nom, found in the full channel sweep (diagnostics/inspect_
# slip_channel_sweep.py) but not examined there -- these look like a
# genuine ECU-domain TC intervention flag and actual/target slip pair,
# in a plausible physical range (0-34.7%), unlike abs_Slip_*.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REAR_ROLLING_RADIUS_OFFSET = 0.0141

WHEEL_NAMES = {"fl": "log_speed_fl", "fr": "log_speed_fr", "rl": "log_speed_rl", "rr": "log_speed_rr"}
SLIP_NAMES = {"fl": "abs_Slip_FL", "fr": "abs_Slip_FR", "rl": "abs_Slip_RL", "rr": "abs_Slip_RR"}
TC_NAMES = ["ecu_B_tc_act", "ecu_slip_act", "ecu_slip_nom"]


def read_raw_channels(file_path, wanted_names):
    wanted = set(wanted_names)
    out = {}
    with open(file_path, "r", encoding="cp1252", errors="replace") as f:
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


all_names = list(WHEEL_NAMES.values()) + list(SLIP_NAMES.values()) + TC_NAMES
raw = read_raw_channels(RAW_FILE, all_names)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
v_ecu_kmh = state["v_mps"] * 3.6
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
throttle_pct = state["throttle_pct"]

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


wheel_speed = {w: interp_onto_t(name) for w, name in WHEEL_NAMES.items()}
abs_slip = {w: interp_onto_t(name) for w, name in SLIP_NAMES.items()}
kappa_wheel_pct = {w: (wheel_speed[w] - v_ecu_kmh) / v_ecu_kmh * 100.0 for w in WHEEL_NAMES}

print("=" * 78)
print("ITEM 1 -- regress kappa on abs_Slip_*, per wheel: kappa ~= a + b*abs_Slip?")
print("=" * 78)
for w in ["fl", "fr", "rl", "rr"]:
    m = base_mask & np.isfinite(kappa_wheel_pct[w]) & np.isfinite(abs_slip[w])
    x = abs_slip[w][m]
    y = kappa_wheel_pct[w][m]
    slope, intercept = np.polyfit(x, y, 1)
    pred = slope * x + intercept
    ss_res = np.sum((y - pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    print(f"  {w.upper():3s} n={int(m.sum())}  slope={slope:+.3f}  intercept={intercept:+.4f}  R^2={r2:.4f}  "
          f"(hypothesis: slope~=-100 -> {'CLOSE' if abs(slope + 100) < 20 else 'NOT CLOSE'})")
print()
print("  For reference, kappa's own percentile range (pct, |.|): "
      f"p50={np.percentile(np.abs(np.concatenate([kappa_wheel_pct[w][base_mask] for w in kappa_wheel_pct])),50):.3f}  "
      f"p99={np.percentile(np.abs(np.concatenate([kappa_wheel_pct[w][base_mask] for w in kappa_wheel_pct])),99):.3f}")
print()

print("=" * 78)
print("ITEM 3 -- driven-vs-undriven slip: kappa_driven = (v_rear - v_front)/v_front")
print("=" * 78)
v_front_raw = np.mean([wheel_speed["fl"], wheel_speed["fr"]], axis=0)
v_rear_raw = np.mean([wheel_speed["rl"], wheel_speed["rr"]], axis=0)
v_rear_corrected = v_rear_raw / (1.0 + REAR_ROLLING_RADIUS_OFFSET)

kappa_driven_raw = (v_rear_raw - v_front_raw) / v_front_raw
kappa_driven_corrected = (v_rear_corrected - v_front_raw) / v_front_raw

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

for label, kappa in [("RAW", kappa_driven_raw), ("WP-S1-CORRECTED", kappa_driven_corrected)]:
    print(f"--- {label} ---")
    vals = kappa[base_mask]
    vals = vals[np.isfinite(vals)]
    print(f"  base population n={len(vals)}  p50={np.percentile(vals,50)*100:+.3f}%  "
          f"p90={np.percentile(vals,90)*100:.3f}%  p99={np.percentile(vals,99)*100:.3f}%  "
          f"max={np.max(vals)*100:.3f}%  min={np.min(vals)*100:+.3f}%")
    for phase_label, pmask in [("exit_4+5", exit_mask), ("entry_1_brake (unreliable mask)", phase_mask["entry_1_brake"])]:
        m = base_mask & pmask
        v = kappa[m]
        v = v[np.isfinite(v)]
        if len(v) == 0:
            continue
        print(f"    {phase_label:32s} n={len(v):6d}  p50={np.percentile(v,50)*100:+.3f}%  "
              f"p90={np.percentile(v,90)*100:.3f}%  p99={np.percentile(v,99)*100:.3f}%  max={np.max(v)*100:.3f}%")
    m_throttle = base_mask & (throttle_pct > 20.0)
    v = kappa[m_throttle]
    v = v[np.isfinite(v)]
    print(f"    {'throttle>20%':32s} n={len(v):6d}  p50={np.percentile(v,50)*100:+.3f}%  "
          f"p90={np.percentile(v,90)*100:.3f}%  p99={np.percentile(v,99)*100:.3f}%  max={np.max(v)*100:.3f}%")
    print()

print("  -- comparison against the log_speed-derived provisional kappa (rear, WP-S1-corrected) --")
kappa_rear_provisional_pct = (v_rear_corrected - v_ecu_kmh) / v_ecu_kmh * 100.0
m = base_mask & np.isfinite(kappa_driven_corrected) & np.isfinite(kappa_rear_provisional_pct)
corr = np.corrcoef(kappa_driven_corrected[m] * 100.0, kappa_rear_provisional_pct[m])[0, 1]
print(f"  corr(kappa_driven_corrected, provisional kappa_rear) = {corr:+.4f}  n={int(m.sum())}")

abs_slip_rear_pct = np.mean([abs_slip["rl"], abs_slip["rr"]], axis=0)
m2 = base_mask & np.isfinite(kappa_driven_corrected) & np.isfinite(abs_slip_rear_pct)
corr2 = np.corrcoef(kappa_driven_corrected[m2] * 100.0, abs_slip_rear_pct[m2])[0, 1]
print(f"  corr(kappa_driven_corrected, abs_Slip rear avg) = {corr2:+.4f}  n={int(m2.sum())}")
print()

print("=" * 78)
print("TC-ACTIVE EXTENSION -- ecu_B_tc_act / ecu_slip_act / ecu_slip_nom")
print("=" * 78)
tc_act = interp_onto_t("ecu_B_tc_act")
tc_act_bin = tc_act > 0.5
slip_act = interp_onto_t("ecu_slip_act")
slip_nom = interp_onto_t("ecu_slip_nom")

print(f"ecu_B_tc_act fraction TRUE, base population: "
      f"{float(np.mean(tc_act_bin[base_mask]))*100:.2f}%  (n={int(base_mask.sum())})")
for phase_label, pmask in [("exit_4+5", exit_mask), ("entry_1_brake (unreliable mask)", phase_mask["entry_1_brake"])]:
    m = base_mask & pmask
    if m.sum() == 0:
        continue
    print(f"  {phase_label:32s} n={int(m.sum()):6d}  TC active fraction={float(np.mean(tc_act_bin[m]))*100:.2f}%")
m_throttle = base_mask & (throttle_pct > 20.0)
print(f"  {'throttle>20%':32s} n={int(m_throttle.sum()):6d}  TC active fraction={float(np.mean(tc_act_bin[m_throttle]))*100:.2f}%")
print()

print("ecu_slip_act[%] distribution (base population, all samples incl. TC-inactive):")
v = slip_act[base_mask]
print(f"  n={len(v)}  p50={np.percentile(v,50):.3f}  p90={np.percentile(v,90):.3f}  "
      f"p99={np.percentile(v,99):.3f}  max={np.max(v):.3f}")
print("ecu_slip_act[%] distribution, TC-active samples only:")
m_act = base_mask & tc_act_bin
if m_act.sum() > 0:
    v = slip_act[m_act]
    print(f"  n={len(v)}  p50={np.percentile(v,50):.3f}  p90={np.percentile(v,90):.3f}  "
          f"p99={np.percentile(v,99):.3f}  max={np.max(v):.3f}")
else:
    print("  n=0")
print()
print("ecu_slip_nom[%] (target slip setpoint) distribution, TC-active samples only:")
if m_act.sum() > 0:
    v = slip_nom[m_act]
    print(f"  n={len(v)}  p50={np.percentile(v,50):.3f}  p90={np.percentile(v,90):.3f}  "
          f"p99={np.percentile(v,99):.3f}  max={np.max(v):.3f}")
else:
    print("  n=0")

m3 = base_mask & np.isfinite(slip_act) & np.isfinite(kappa_driven_corrected)
corr3 = np.corrcoef(slip_act[m3], kappa_driven_corrected[m3] * 100.0)[0, 1]
print()
print(f"corr(ecu_slip_act, kappa_driven_corrected) = {corr3:+.4f}  n={int(m3.sum())}")
