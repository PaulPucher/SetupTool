# Input corrections of ORDER SPEED-1 (author rulings R1-R3, 2026-11-06),
# each behind its own switch in parameters.json signal_corrections:
#   R1 speed_reference        ecu_speed spikes from the faulty ECU rear-right
#                             wheel input replaced by the front ABS speed
#   R2 engine_braking         drag of the closed-throttle engine assigned to
#                             the driven (rear) axle in the fit-L axle split
#   R3 sensor_offset_removal  zero offsets of yaw rate and a_y (standstill) and
#                             of the steering-wheel angle (near-zero-yaw straights)
# R1 and R3 act on the parsed channels (csv_parser.parse_csv), so the lap
# split, the corner detection and every estimator see the same corrected
# signal; R2 acts where the axle split is formed (tyre_fit_auto). Every
# record is JSON-safe and goes into the fit manifest (signal_corrections).

import warnings

import numpy as np
from scipy.ndimage import binary_dilation, maximum_filter1d

# Replacement source of R1: the larger of the two front ABS wheel speeds.
# The front axle is not driven, so its wheels carry no drive slip (Rajamani
# 2012, ch. 14: non-driven wheels as the speed reference); under braking they
# read low, which is why R1 uses them only where ecu_speed is known faulty.
# Detection reference: the maximum of all four ABS wheel speeds (criterion A
# of the 2026-11-05 counterfactual and SPEED-0b). The root-cause check (every
# flagged sample carries the faulty-wheel signature) holds for this reference
# at 1.0 m/s; against the front maximum alone it does not (SPEED-1 check:
# 5 flagged samples without the signature in 10 sessions), since a driven
# rear wheel can legitimately run faster than both fronts.
# Method-defining (which sensor detects and which replaces), not tunables.
FRONT_ABS_CHANNELS = ("abs_speed_fl", "abs_speed_fr")
ABS_CHANNELS = ("abs_speed_fl", "abs_speed_fr", "abs_speed_rl", "abs_speed_rr")

# R3 channels (ruling amended 2026-11-06, SPEED-1c).
OFFSET_CHANNELS = ("sclu_yaw_rate", "log_acc_y", "log_asteer")
WHEEL_CORNERS = ("fl", "fr", "rl", "rr")


def config(params, name):
    """The switch block of one correction; a params dict without the
    signal_corrections block (older configs, hand-built test dicts) runs the
    uncorrected chain."""
    block = (params.get("signal_corrections") or {}).get(name) or {}
    return block if block.get("enabled") else None


def _usable(ch):
    return ch is not None and ch.get("quality") not in ("missing", "failed") and ch.get("time") is not None \
        and len(ch["time"]) > 1


def _to_kmh(ch, canonical_unit, conversions):
    # the ABS channels are logged in mph on some exports and kph on others
    # (channel census 2026-11-06); the parser keeps the file's values, so the
    # declared unit is read here. No declared unit = unknown, not assumed.
    unit = ch.get("unit_raw")
    if unit is None:
        return None
    src = canonical_unit(unit)
    if src == "km/h":
        return np.asarray(ch["data"], dtype=float)
    factor = conversions.get((src, "km/h"))
    return None if factor is None else np.asarray(ch["data"], dtype=float) * factor


def correct_speed_reference(channels, cfg, canonical_unit, conversions, sample_rate_of):
    """R1 on ecu_speed's own time base (km/h, checked by the parser before).
    A sample is flagged where ecu_speed exceeds the running maximum of the
    four ABS wheel speeds within +-comparison_half_window_s by more than
    spike_margin_mps; the flagged runs, widened by widen_s each side, take the
    larger front ABS speed. The margin sits above the clean disagreement
    (p99.9 0.21 m/s pooled) and every flagged sample carried the faulty-wheel
    signature in the SPEED-0b root-cause check, so the rule touches only the
    known fault. Mutates channels["ecu_speed"]["data"] (a new array); returns
    the record."""
    rec = {"enabled": True, "spike_margin_mps": cfg["spike_margin_mps"],
           "comparison_half_window_s": cfg["comparison_half_window_s"], "widen_s": cfg["widen_s"],
           "detection": "max(" + ", ".join(ABS_CHANNELS) + ")",
           "replacement": "max(" + ", ".join(FRONT_ABS_CHANNELS) + ")"}
    sp = channels.get("ecu_speed")
    if not _usable(sp):
        rec.update(status="no_ecu_speed", applied=False, flagged_samples=0, replaced_samples=0)
        return rec
    wheels = {}
    for name in ABS_CHANNELS:
        ch = channels.get(name)
        v = _to_kmh(ch, canonical_unit, conversions) if _usable(ch) else None
        if v is None:
            why = "missing or failed" if not _usable(ch) else f"unit {ch.get('unit_raw')!r} not convertible"
            warnings.warn(f"speed reference: {name} {why} -- ecu_speed used uncorrected")
            rec.update(status="no_abs_channels", applied=False, flagged_samples=0, replaced_samples=0,
                       reason=f"{name} {why}")
            return rec
        wheels[name] = np.interp(sp["time"], ch["time"], v)
    v_ref = np.maximum(*(wheels[n] for n in FRONT_ABS_CHANNELS))
    v_det = np.max(np.vstack([wheels[n] for n in ABS_CHANNELS]), axis=0)
    v = np.asarray(sp["data"], dtype=float)
    sr = sample_rate_of(sp["time"])
    half = int(round(cfg["comparison_half_window_s"] * sr))
    widen = int(round(cfg["widen_s"] * sr))
    flagged = v > maximum_filter1d(v_det, size=2 * half + 1, mode="nearest") + cfg["spike_margin_mps"] * 3.6
    wide = binary_dilation(flagged, structure=np.ones(2 * widen + 1, bool)) if flagged.any() else flagged
    sp["data"] = np.where(wide, v_ref, v)
    d = np.diff(np.r_[0, flagged.astype(np.int8), 0])
    rec.update(status="applied", applied=True, sample_rate_hz=float(sr), flagged_samples=int(flagged.sum()),
               flagged_events=int((d == 1).sum()), replaced_samples=int(wide.sum()),
               max_excess_mps=float(np.max((v - v_det)[flagged]) / 3.6) if flagged.any() else 0.0)
    return rec


def _standstill_mask(state, channels, cfg):
    """Standstill on wheels, the rule of the event zero entries
    (diagnostics/inspect_prc24996_zero.py): speed below standstill_speed_max_kmh
    and every suspension travel within on_wheels_travel_mm of its pit-lane
    median (limiter on, speed inside pit_speed_range_kmh), so a car on jacks is
    not taken. None if the travel or limiter channels are unusable or there is
    no pit-lane sample."""
    from modules.loads import _normalize_travel_to_mm
    t = state["time"]
    v_kmh = state["v_mps"] * 3.6
    lim_ch = channels.get("ecu_B_speedlimit_en")
    trav = {c: channels.get(f"log_susp_travel_{c}") for c in WHEEL_CORNERS}
    if not _usable(lim_ch) or not all(_usable(ch) for ch in trav.values()):
        return None
    lim = np.interp(t, lim_ch["time"], lim_ch["data"])
    lo, hi = cfg["pit_speed_range_kmh"]
    pit = (lim > 0.5) & (v_kmh > lo) & (v_kmh < hi)
    if not pit.any():
        return None
    still = v_kmh < cfg["standstill_speed_max_kmh"]
    for ch in trav.values():
        x = _normalize_travel_to_mm(np.interp(t, ch["time"], ch["data"]), ch.get("unit_raw"))
        still &= np.abs(x - np.median(x[pit])) < cfg["on_wheels_travel_mm"]
    return still


def heading_excess_offset(yaw_deg_s, t, sr, laps):
    """Yaw-rate offset from closed laps: a closed lap turns the car by exactly
    one full turn, so the heading change beyond a whole number of turns divided
    by the lap time is the mean offset over that lap (a gain error is left
    aside; it is small against the excess, SPEED-1b). Median over the analysis
    laps. Returns (offset deg/s or None, number of laps)."""
    vals = []
    for l in laps:
        if not l.get("is_valid_for_analysis"):
            continue
        s = (t >= l["start_time"]) & (t <= l["end_time"])
        if s.sum() < 2:
            continue
        dpsi = float(np.sum(yaw_deg_s[s]) / sr)
        vals.append((dpsi - 360.0 * round(dpsi / 360.0)) / (s.sum() / sr))
    return (float(np.median(vals)), len(vals)) if vals else (None, 0)


def steering_offset(sw_deg, yaw_corr_deg_s, straight_mask, yaw_max_deg_s, n_min):
    """Steering-wheel centre: median over the straight samples whose
    bias-corrected yaw rate is below yaw_max_deg_s, i.e. where the car really
    runs straight. Returns (offset or None, sample count)."""
    sel = straight_mask & (np.abs(yaw_corr_deg_s) < yaw_max_deg_s) & np.isfinite(sw_deg)
    n = int(sel.sum())
    return (float(np.median(sw_deg[sel])) if n >= n_min else None), n


def remove_sensor_offsets(channels, laps, params, cfg):
    """R3 (ruling amended 2026-11-06, SPEED-1c): zero offsets of the yaw rate,
    the lateral acceleration and the steering-wheel angle, subtracted from the
    channels before any use (Tier B, zero-input calibration).
    - yaw rate: median at standstill on wheels; without a valid standstill the
      per-lap heading excess beyond one full turn over the lap time (median
      over the analysis laps). The source is recorded.
    - a_y: median at standstill only; without a valid standstill no correction
      (flagged). No straight-line median: the accelerometer measures specific
      force, and on straights that includes crossfall and roll, which are real
      lateral tyre load and stay in the signal.
    - steering-wheel angle: median over the chain's steering-defined straights
      (observer.steering_straights) restricted to samples whose bias-corrected
      yaw rate is below steering_straight_yaw_max_deg_s; too few -> no
      correction (flagged). The standstill is no reference here: the wheel is
      parked at any angle.
    Mutates the channels (new arrays); returns the record, offsets in channel
    units and in physical units."""
    from modules.observer import steering_straights
    from modules.stability_analysis import prepare_vehicle_state
    rec = {"enabled": True, "channels": {},
           "min_standstill_s": cfg["min_standstill_s"], "min_straight_s": cfg["min_straight_s"],
           "steering_straight_yaw_max_deg_s": cfg["steering_straight_yaw_max_deg_s"]}
    # the parser must not refuse a file the chain refuses later (grid rate
    # below the floor): no state -> no correction, flagged
    try:
        state = prepare_vehicle_state(channels, params)
    except ValueError as e:
        state, why = None, str(e)
    else:
        why = "required channels missing or failed"
    if state is None:
        rec.update(status="no_vehicle_state", applied=False, not_corrected=list(OFFSET_CHANNELS), reason=why)
        warnings.warn("sensor offsets: no vehicle state -- channels used uncorrected")
        return rec
    t, sr = state["time"], state["sample_rate_hz"]
    yaw_deg = np.degrees(state["yaw_rate_radps"])
    still = _standstill_mask(state, channels, cfg)
    still_s = float(still.sum() / sr) if still is not None else 0.0
    valid_still = still is not None and still_s >= cfg["min_standstill_s"]
    rec["standstill_s"] = still_s
    offsets = {}
    if valid_still:
        offsets["sclu_yaw_rate"] = (float(np.median(yaw_deg[still])), "standstill", int(still.sum()))
        offsets["log_acc_y"] = (float(np.median(state["ay_mps2"][still])), "standstill", int(still.sum()))
    else:
        off, n_laps = heading_excess_offset(yaw_deg, t, sr, laps)
        if off is not None:
            offsets["sclu_yaw_rate"] = (off, "heading_excess_per_lap", n_laps)
        else:
            rec["channels"]["sclu_yaw_rate"] = {"applied": False, "reason": "no valid standstill and no closed lap"}
        rec["channels"]["log_acc_y"] = {"applied": False, "reason": "no valid standstill"}
    if any(l.get("is_valid_for_analysis") for l in laps):
        mask = steering_straights(state, laps, params["tyre_fit_auto"])["mask"]
        yaw_corr = yaw_deg - offsets.get("sclu_yaw_rate", (0.0,))[0]
        off, n = steering_offset(np.degrees(state["steer_sw_rad"]), yaw_corr, mask,
                                 cfg["steering_straight_yaw_max_deg_s"], int(round(cfg["min_straight_s"] * sr)))
        rec["straight_samples"] = int(mask.sum())
        if off is not None:
            offsets["log_asteer"] = (off, "straights_near_zero_yaw", n)
        else:
            rec["channels"]["log_asteer"] = {"applied": False, "n": n,
                                             "reason": "too few straight samples with near-zero yaw rate"}
    else:
        rec["channels"]["log_asteer"] = {"applied": False, "reason": "no analysis laps"}
    # physical value -> channel unit: yaw rate as logged (yaw_rate_unit), a_y in g, steering in deg
    yaw_factor = params["stability_estimation"]["yaw_rate_to_radps"]
    to_ch = {"sclu_yaw_rate": lambda x: np.radians(x) / yaw_factor, "log_acc_y": lambda x: x / 9.81,
             "log_asteer": lambda x: x}
    unit_phys = {"sclu_yaw_rate": "deg/s", "log_acc_y": "m/s^2", "log_asteer": "deg"}
    for name, (phys, source, n) in offsets.items():
        ch = channels[name]
        off_ch = float(to_ch[name](phys))
        ch["data"] = np.asarray(ch["data"], dtype=float) - off_ch
        rec["channels"][name] = {"applied": True, "source": source, "n": n, "offset": off_ch,
                                 "unit": ch.get("unit_raw") or ch.get("unit"),
                                 "offset_physical": phys, "unit_physical": unit_phys[name]}
    missing = [n for n in OFFSET_CHANNELS if not rec["channels"].get(n, {}).get("applied")]
    rec.update(applied=bool(offsets), not_corrected=missing,
               status="applied" if not missing else ("partial" if offsets else "not_applied"))
    if missing:
        warnings.warn(f"sensor offsets: no correction for {', '.join(missing)} -- used uncorrected")
    return rec


def engine_braking_front_fx(fx_cog, share_f, throttle_pct, cfg):
    """R2: axle split of the CoG longitudinal force with engine braking.
    With the throttle closed and the car decelerating (fx_cog < 0) the
    engine's drag torque acts on the driven wheels only (Mitschke &
    Wallentowitz sec. 33.4.2; Rajamani 2012 sec. 4.2), so a constant
    drag_force_N goes to the rear first and only the remainder is split by
    axle load. Where the car decelerates less than that, the whole force
    stays on the rear: the rear never takes more than the total, the front
    gets nothing. Everywhere else the plain load split. The axle sum stays
    fx_cog (the caller forms rear = fx_cog - front). Returns (front, mask
    of the engine-braking samples)."""
    plain = np.where(fx_cog < 0.0, fx_cog * share_f, 0.0)
    eb = (fx_cog < 0.0) & (throttle_pct < cfg["throttle_closed_max_pct"])
    rem = fx_cog + cfg["drag_force_N"]
    front = np.where(eb, np.where(rem < 0.0, rem * share_f, 0.0), plain)
    return front, eb
