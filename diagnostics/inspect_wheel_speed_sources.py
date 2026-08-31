# WP-S1 (Open Board item B, sideslip methods comparison), engineer's half,
# TASK 1: wheel-speed channel quality diagnostic. Read-only, Tier B
# (signal/data engineering -- standard QA techniques, no vehicle-dynamics
# claim). Report-only: nothing here feeds prepare_vehicle_state or any
# Module 2-5 function; ecu_speed stays the production speed source
# throughout. Nothing is whitelisted in config/channels.json by this
# script and no config is written.
#
# The raw log carries four redundant wheel-speed channel families, none
# of them in the channels.json whitelist: log_speed_*, ecu_speed_*,
# abs_speed_*, Team_nWheel* (each FL/FR/RL/RR). They are read directly
# from the raw Pi Toolbox file with a local block reader (same
# {ChannelBlock} format csv_parser.py uses, but without the whitelist
# filter), so nothing here touches config/channels.json.
#
# Sections:
#   0. Presence / units / native sample rate.
#   1. Per-wheel NaN, frozen, and dropout fractions.
#   2. Agreement of the 4-wheel average vs ecu_speed on straight-line
#      segments.
#   3. Straight-line left/right consistency (FL-FR, RL-RR spread).
#   4. Ranking summary.

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

# Straight-line selection thresholds (diagnostic-local, not config --
# this script is report-only and these gate nothing downstream). Picked
# to isolate low-lateral-load, low-yaw-rate driving where all four wheels
# should be turning at very nearly the same road speed, so that any
# spread between channels is measurement noise, not genuine slip/turning
# geometry.
AY_STRAIGHT_MAX_G = 0.15
YAW_STRAIGHT_MAX_DEGPS = 3.0

# Standard-technique QA thresholds (Tier B, diagnostic-local).
FREEZE_MIN_DURATION_S = 0.5   # signal "stuck" if identical for >= this long
DROPOUT_GAP_FACTOR = 3.0      # a time gap > this many median dt's = dropout

MPH_TO_KMH = 1.609344

FAMILIES = {
    "log_speed": {"wheels": ["fl", "fr", "rl", "rr"], "name_fmt": "log_speed_{w}",
                  "unit": "kph", "to_kmh": lambda x: x},
    "ecu_speed_wheels": {"wheels": ["fl", "fr", "rl", "rr"], "name_fmt": "ecu_speed_{w}",
                         "unit": "kph", "to_kmh": lambda x: x},
    "abs_speed": {"wheels": ["fl", "fr", "rl", "rr"], "name_fmt": "abs_speed_{w}",
                  "unit": "mph", "to_kmh": lambda x: x * MPH_TO_KMH},
    "Team_nWheel": {"wheels": ["FL", "FR", "RL", "RR"], "name_fmt": "Team_nWheel{w}",
                    "unit": "rpm", "to_kmh": None},  # no rolling radius in config -- see Section 2
}


def read_raw_channels(file_path, wanted_names):
    """Direct {ChannelBlock} reader, bypassing channels.json's whitelist
    filter entirely (csv_parser.parse_csv only keeps whitelisted names)."""
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
                        n_lines, n_fail = 0, 0
                        while i < n and not lines[i].strip().startswith("{"):
                            raw_line = lines[i].strip()
                            if raw_line:
                                n_lines += 1
                                parts = raw_line.split("\t")
                                ok = False
                                if len(parts) == 2:
                                    try:
                                        t = float(parts[0].replace(",", "."))
                                        v = float(parts[1].replace(",", "."))
                                        times.append(t)
                                        values.append(v)
                                        ok = True
                                    except ValueError:
                                        pass
                                if not ok:
                                    n_fail += 1
                            i += 1
                        out[channel_name] = {
                            "time": np.array(times), "data": np.array(values),
                            "n_lines": n_lines, "n_parse_fail": n_fail,
                        }
                        continue
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                        continue
        i += 1
    return out


def sample_rate_hz(t):
    if len(t) < 2:
        return float("nan")
    return 1.0 / float(np.median(np.diff(t)))


def frozen_fraction(v, rate_hz):
    if len(v) < 2 or not np.isfinite(rate_hz):
        return float("nan")
    min_run = max(3, int(round(FREEZE_MIN_DURATION_S * rate_hz)))
    same_as_prev = np.concatenate([[False], v[1:] == v[:-1]])
    # run length ending at each sample
    run_len = np.zeros(len(v), dtype=int)
    for idx in range(1, len(v)):
        if same_as_prev[idx]:
            run_len[idx] = run_len[idx - 1] + 1
    frozen = run_len >= (min_run - 1)
    # extend the flag backward to cover the whole run, not just its tail
    flagged = frozen.copy()
    for idx in range(len(v) - 1, 0, -1):
        if flagged[idx] and same_as_prev[idx]:
            flagged[idx - 1] = True
    return float(flagged.sum()) / len(v)


def dropout_fraction(t):
    if len(t) < 2:
        return float("nan")
    dt = np.diff(t)
    dt_med = np.median(dt)
    if dt_med <= 0:
        return float("nan")
    gaps = dt[dt > DROPOUT_GAP_FACTOR * dt_med]
    missing_est = np.sum(np.round(gaps / dt_med) - 1)
    return float(missing_est) / (len(t) + missing_est)


def nan_fraction(ch):
    if ch["n_lines"] == 0:
        return float("nan")
    return ch["n_parse_fail"] / ch["n_lines"]


all_names = [fam["name_fmt"].format(w=w) for fam in FAMILIES.values() for w in fam["wheels"]]
raw = read_raw_channels(RAW_FILE, all_names)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
t_ref = state["time"]
v_ecu_kmh = state["v_mps"] * 3.6
ay_g = state["ay_mps2"] / 9.81
yaw_rate_degps = state["yaw_rate_radps"] * 180.0 / np.pi
moving = state["moving_mask"]

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)

straight_mask = moving & racing_mask & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)

print("=" * 78)
print("SECTION 0 -- Presence / units / native sample rate")
print("=" * 78)
print(f"Straight-line selection: moving & valid-lap & |ay|<={AY_STRAIGHT_MAX_G}g "
      f"& |yaw rate|<={YAW_STRAIGHT_MAX_DEGPS} deg/s -> {straight_mask.sum()} / {len(straight_mask)} samples")
print(f"Freeze threshold: identical for >= {FREEZE_MIN_DURATION_S}s (per channel's own native rate)")
print(f"Dropout threshold: time gap > {DROPOUT_GAP_FACTOR}x that channel's median dt")
print()

per_channel = {}
for fam_key, fam in FAMILIES.items():
    print(f"--- {fam_key} ({fam['unit']}) ---")
    for w in fam["wheels"]:
        name = fam["name_fmt"].format(w=w)
        ch = raw.get(name)
        if ch is None or len(ch["data"]) == 0:
            print(f"  {name:<18} NOT PRESENT")
            continue
        rate = sample_rate_hz(ch["time"])
        per_channel[name] = {"fam": fam_key, "rate": rate, "ch": ch}
        print(f"  {name:<18} present, n={len(ch['data']):6d}, rate~{rate:5.1f} Hz")

print()
print("=" * 78)
print("SECTION 1 -- Per-wheel NaN / frozen / dropout fractions")
print("=" * 78)
for fam_key, fam in FAMILIES.items():
    print(f"--- {fam_key} ---")
    for w in fam["wheels"]:
        name = fam["name_fmt"].format(w=w)
        info = per_channel.get(name)
        if info is None:
            continue
        ch, rate = info["ch"], info["rate"]
        nan_f = nan_fraction(ch)
        frozen_f = frozen_fraction(ch["data"], rate)
        drop_f = dropout_fraction(ch["time"])
        per_channel[name]["nan_frac"] = nan_f
        per_channel[name]["frozen_frac"] = frozen_f
        per_channel[name]["dropout_frac"] = drop_f
        print(f"  {name:<18} NaN={nan_f*100:5.2f}%  frozen={frozen_f*100:5.2f}%  dropout={drop_f*100:5.2f}%")

print()
print("=" * 78)
print("SECTION 2 -- Agreement with ecu_speed on straight-line segments")
print("=" * 78)
ve = v_ecu_kmh[straight_mask]
family_deviation = {}
for fam_key, fam in FAMILIES.items():
    wheel_kmh = []
    ok = True
    for w in fam["wheels"]:
        name = fam["name_fmt"].format(w=w)
        ch = raw.get(name)
        if ch is None or len(ch["data"]) < 2:
            ok = False
            break
        interp = np.interp(t_ref, ch["time"], ch["data"])[straight_mask]
        wheel_kmh.append(interp)
    if not ok:
        print(f"--- {fam_key}: incomplete, skipped ---")
        continue
    avg_native = np.mean(wheel_kmh, axis=0)

    if fam["to_kmh"] is not None:
        avg_kmh = fam["to_kmh"](avg_native)
        rel_dev = (avg_kmh - ve) / ve
        print(f"--- {fam_key} (direct unit conversion, {fam['unit']} -> kph) ---")
    else:
        # Team_nWheel: rpm, no rolling-radius constant in config to convert
        # to km/h directly. Least-squares scale factor k (rpm_avg = k *
        # v_ecu, fit through the origin on straight-line samples only) --
        # same regression technique already used in
        # inspect_gps_speed_validation.py Section 1 (removed 2026-08-30,
        # see git history) -- lets us report a
        # relative deviation without inventing a radius. k itself is NOT a
        # rolling radius (units rpm per km/h, not m); reported for
        # completeness only, no config consequence.
        k = float(np.sum(avg_native * ve) / np.sum(ve ** 2))
        proxy_kmh = avg_native / k
        rel_dev = (proxy_kmh - ve) / ve
        print(f"--- {fam_key} (rpm, no radius in config -- least-squares scale k={k:.4f} rpm per kph) ---")
    family_deviation[fam_key] = rel_dev
    print(f"  n={len(rel_dev)}  mean rel. dev.={np.mean(rel_dev)*100:+.3f}%  "
          f"median rel. dev.={np.median(rel_dev)*100:+.3f}%  std={np.std(rel_dev)*100:.3f}%")

print()
print("=" * 78)
print("SECTION 3 -- Straight-line left/right consistency (noise measure)")
print("=" * 78)
family_lr_spread = {}
for fam_key, fam in FAMILIES.items():
    pairs = [("FL", "FR", fam["wheels"][0], fam["wheels"][1]),
             ("RL", "RR", fam["wheels"][2], fam["wheels"][3])]
    spreads = []
    print(f"--- {fam_key} ---")
    for label_l, label_r, w_l, w_r in pairs:
        name_l = fam["name_fmt"].format(w=w_l)
        name_r = fam["name_fmt"].format(w=w_r)
        ch_l, ch_r = raw.get(name_l), raw.get(name_r)
        if ch_l is None or ch_r is None or len(ch_l["data"]) < 2 or len(ch_r["data"]) < 2:
            print(f"  {label_l} vs {label_r}: incomplete, skipped")
            continue
        vl = np.interp(t_ref, ch_l["time"], ch_l["data"])[straight_mask]
        vr = np.interp(t_ref, ch_r["time"], ch_r["data"])[straight_mask]
        mean_lr = (vl + vr) / 2.0
        safe = mean_lr != 0
        rel_spread = (vl[safe] - vr[safe]) / mean_lr[safe]
        spreads.append(rel_spread)
        print(f"  {label_l} vs {label_r}: n={safe.sum()}  mean={np.mean(rel_spread)*100:+.3f}%  "
              f"median={np.median(rel_spread)*100:+.3f}%  std={np.std(rel_spread)*100:.3f}%")
    if spreads:
        all_spread = np.concatenate(spreads)
        family_lr_spread[fam_key] = all_spread
        print(f"  combined std (both axles): {np.std(all_spread)*100:.3f}%")

print()
print("=" * 78)
print("SECTION 4 -- Ranking summary (computed, not asserted)")
print("=" * 78)
for fam_key, fam_info in FAMILIES.items():
    dev = family_deviation.get(fam_key)
    spr = family_lr_spread.get(fam_key)
    dev_str = f"|median dev|={abs(np.median(dev))*100:.3f}%" if dev is not None else "dev n/a"
    spr_str = f"L/R std={np.std(spr)*100:.3f}%" if spr is not None else "L/R n/a"
    fam_channel_names = [fam_info["name_fmt"].format(w=w) for w in fam_info["wheels"]]
    worst_nan = max((per_channel[nm]["nan_frac"] for nm in fam_channel_names if nm in per_channel), default=float("nan"))
    worst_frozen = max((per_channel[nm]["frozen_frac"] for nm in fam_channel_names if nm in per_channel), default=float("nan"))
    worst_drop = max((per_channel[nm]["dropout_frac"] for nm in fam_channel_names if nm in per_channel), default=float("nan"))
    print(f"  {fam_key:<18} {dev_str:<22} {spr_str:<20} "
          f"worst NaN={worst_nan*100:.2f}%  worst frozen={worst_frozen*100:.2f}%  worst dropout={worst_drop*100:.2f}%")

print()
print("=" * 78)
print("SECTION 5 -- log_speed vs ecu_speed: axle x drive-state split")
print("=" * 78)
print("Scope: log_speed_* only. Section 2 already established log_speed_* and")
print("ecu_speed_* are byte-identical duplicates (same time base, max abs diff=0.0),")
print("so splitting one covers both; abs_speed_*/Team_nWheel* not repeated here.")

# Drive-state thresholds (diagnostic-local, not config -- report-only, same
# convention as the straight-line thresholds above). ecu_aps/log_pbrake_f
# are both whitelisted, production channels (state["throttle_pct"]/
# state["brake_f_bar"], already interpolated onto t_ref by
# prepare_vehicle_state).
THROTTLE_ON_MIN_PCT = 20.0  # ecu_aps -- meaningful pedal application, excludes coast/engine-braking noise near 0%
BRAKING_MIN_BAR = 5.0       # log_pbrake_f -- meaningful pedal application, excludes sensor noise floor

throttle_pct = state["throttle_pct"]
brake_f_bar = state["brake_f_bar"]

AXLES = {
    "front (FL/FR)": ("log_speed_fl", "log_speed_fr"),
    "rear (RL/RR)": ("log_speed_rl", "log_speed_rr"),
}


def axle_rel_dev(names, mask):
    wheel_vals = [np.interp(t_ref, raw[n]["time"], raw[n]["data"])[mask] for n in names]
    avg = np.mean(wheel_vals, axis=0)
    ve_local = v_ecu_kmh[mask]
    return (avg - ve_local) / ve_local


print()
print("--- 1. Front vs rear, straight-line (all drive states) ---")
axle_summary = {}
for axle_label, names in AXLES.items():
    dev = axle_rel_dev(names, straight_mask)
    axle_summary[("all", axle_label)] = dev
    print(f"  {axle_label:16s} n={len(dev):6d}  mean={np.mean(dev)*100:+.3f}%  "
          f"median={np.median(dev)*100:+.3f}%  std={np.std(dev)*100:.3f}%")

print()
print(f"--- 2. Front vs rear, split by drive state (straight-line samples only) ---")
DRIVE_STATES = {
    f"throttle-on (ecu_aps>{THROTTLE_ON_MIN_PCT:.0f}%)": straight_mask & (throttle_pct > THROTTLE_ON_MIN_PCT),
    f"braking (log_pbrake_f>{BRAKING_MIN_BAR:.0f}bar)": straight_mask & (brake_f_bar > BRAKING_MIN_BAR),
}
for state_label, state_mask in DRIVE_STATES.items():
    print(f"  {state_label}: n={state_mask.sum()}")
    if state_mask.sum() < 20:
        print(f"    too few samples, skipped")
        continue
    for axle_label, names in AXLES.items():
        dev = axle_rel_dev(names, state_mask)
        axle_summary[(state_label, axle_label)] = dev
        print(f"    {axle_label:16s} n={len(dev):6d}  mean={np.mean(dev)*100:+.3f}%  "
              f"median={np.median(dev)*100:+.3f}%  std={np.std(dev)*100:.3f}%")

print()
print("--- 3. Conclusion (facts only, from the numbers above) ---")


def _med_pct(key):
    dev = axle_summary.get(key)
    return None if dev is None else float(np.median(dev)) * 100


front_all, rear_all = _med_pct(("all", "front (FL/FR)")), _med_pct(("all", "rear (RL/RR)"))
throttle_key = next((k for k in DRIVE_STATES if k.startswith("throttle-on")), None)
brake_key = next((k for k in DRIVE_STATES if k.startswith("braking")), None)
front_thr, rear_thr = _med_pct((throttle_key, "front (FL/FR)")), _med_pct((throttle_key, "rear (RL/RR)"))
front_brk, rear_brk = _med_pct((brake_key, "front (FL/FR)")), _med_pct((brake_key, "rear (RL/RR)"))

if front_all is not None and rear_all is not None:
    axle_split_pts = abs(front_all - rear_all)
    print(f"Front/rear split, all straight-line samples: front={front_all:+.3f}%, rear={rear_all:+.3f}% "
          f"(split={axle_split_pts:.3f} pct pts) -- the Section 2 whole-axle-average figure (~0.5-0.7%) "
          f"is a blend of these two, not a per-wheel-uniform number.")
if front_thr is not None and rear_thr is not None:
    rear_shift_on_throttle = rear_thr - rear_all if rear_all is not None else float("nan")
    print(f"Under throttle: front={front_thr:+.3f}%, rear={rear_thr:+.3f}% "
          f"(rear shifts {rear_shift_on_throttle:+.3f} pct pts from its all-straight-line value).")
if front_brk is not None and rear_brk is not None:
    print(f"Under braking: front={front_brk:+.3f}%, rear={rear_brk:+.3f}%.")

if front_all is not None and rear_all is not None:
    if axle_split_pts < 0.3:
        print("Pattern: front and rear track each other closely -> consistent with a constant scale "
              "offset applied uniformly across the car, not an axle-specific effect.")
    else:
        rear_stable_under_throttle = (front_thr is not None and rear_thr is not None
                                       and abs(rear_thr - rear_all) < 0.3)
        if rear_stable_under_throttle:
            print("Pattern: NOT a uniform scale error -- front and rear disagree by "
                  f"{axle_split_pts:.2f} pct pts on the straight line, and the rear figure barely moves "
                  "between throttle-on and the overall straight-line set (a driven-axle traction-slip "
                  "signature would appear specifically under power and shrink off-throttle; it does not "
                  "here). More consistent with a REAR-axle-specific baseline offset present regardless of "
                  "drive state -- e.g. a front/rear difference in the ECU's per-wheel rolling-radius "
                  "term -- than with traction slip.")
        else:
            print("Pattern: rear deviation shifts materially under throttle relative to its overall "
                  "straight-line value -> consistent with a driven-axle traction-slip contribution, on "
                  "top of any constant front/rear offset.")
    if front_brk is not None and front_all is not None and abs(front_brk - front_all) > 0.5:
        print(f"Separately, front deviation swings under braking (from {front_all:+.3f}% to "
              f"{front_brk:+.3f}%) while rear stays comparatively flat -- consistent with front-wheel "
              "slip under heavy braking (front carries more brake bias on this car), a distinct effect "
              "from the throttle-state pattern above, layered on the same channel.")
