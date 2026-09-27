# Axle Fx and per-axle slip ratio kappa -- inputs to longitudinal_stiffness.
# SI units, kappa dimensionless.

import numpy as np

WHEEL_NAMES = {
    "front": ("log_speed_fl", "log_speed_fr"),
    "rear": ("log_speed_rl", "log_speed_rr"),
}
CORNERS = ("fl", "fr", "rl", "rr")
CORNER_AXLE_MATE = {"fl": "fr", "fr": "fl", "rl": "rr", "rr": "rl"}
ABS_SPEED_CHANNEL = {c: f"abs_speed_{c}" for c in CORNERS}


def _interp_channel(channels, ch_name, t_ref):
    ch = channels.get(ch_name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def _normalize_wheel_speed_to_kmh(data, unit_raw):
    """abs_speed_* unit differs per export (v3: kph, Dubai: mph) -- trust the
    file's unit_raw. Brings the fallback source to log_speed_*'s km/h.
    """
    if unit_raw in ("kph", "km/h"):
        return data
    if unit_raw == "mph":
        return data * 1.609344
    raise ValueError(
        f"abs_speed unit {unit_raw!r} not recognised (expected 'kph'/'km/h' or 'mph') -- "
        "add explicit handling before trusting this export's speed values"
    )


def _rolling_plausibility_mask(corner_kmh, mate_kmh, ecu_kmh, moving_mask, window_samples,
                                std_min_kmh, ratio_max_deviation):
    """Per-window validity of one wheel speed (non-overlapping windows, so a
    transient fault doesn't kill the whole session). Moving samples only.

    (1) stuck: std < std_min_kmh -> flag this corner.
    (2) mate disagreement: mean |ratio to axle mate - 1| > ratio_max_deviation.
        A disagreeing pair isn't two bad sensors -- ecu_speed decides which
        side is off. ecu_speed is only a tie-breaker, never a check on its
        own: real slip deviates from it legitimately.
    """
    n = len(corner_kmh)
    valid = np.ones(n, dtype=bool)
    for start in range(0, n, window_samples):
        end = min(start + window_samples, n)
        w_moving = moving_mask[start:end]
        if not w_moving.any():
            continue
        w_corner = corner_kmh[start:end][w_moving]
        w_mate = mate_kmh[start:end][w_moving]
        if len(w_corner) < 3 or not np.all(np.isfinite(w_corner)):
            continue
        if np.std(w_corner) < std_min_kmh:
            valid[start:end][w_moving] = False
            continue
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(w_mate != 0, w_corner / w_mate, np.nan)
        finite_ratio = ratio[np.isfinite(ratio)]
        mate_disagrees = (np.mean(np.abs(finite_ratio - 1.0)) > ratio_max_deviation) if len(finite_ratio) else False
        if not mate_disagrees:
            continue
        w_ecu = ecu_kmh[start:end][w_moving]
        with np.errstate(invalid="ignore", divide="ignore"):
            dev_corner = np.abs(np.where(w_ecu != 0, w_corner / w_ecu, np.nan) - 1.0)
            dev_mate = np.abs(np.where(w_ecu != 0, w_mate / w_ecu, np.nan) - 1.0)
        m_corner = np.nanmean(dev_corner) if np.isfinite(dev_corner).any() else np.nan
        m_mate = np.nanmean(dev_mate) if np.isfinite(dev_mate).any() else np.nan
        # worse side, or ecu_speed unusable here -> flag (conservative)
        if not (np.isfinite(m_corner) and np.isfinite(m_mate)) or m_corner >= m_mate:
            valid[start:end][w_moving] = False
    return valid


def _guarded_wheel_speed_kmh(channels, corner, t_ref, moving_mask, sample_rate_hz, params):
    """log_speed_{corner} with invalid windows replaced by abs_speed_{corner},
    NaN if no fallback. Returns (kmh or None, per-sample source label).
    """
    wg = params["wheel_speed_guard"]
    mate = CORNER_AXLE_MATE[corner]
    corner_kmh = _interp_channel(channels, f"log_speed_{corner}", t_ref)
    if corner_kmh is None:
        return None, None
    mate_kmh = _interp_channel(channels, f"log_speed_{mate}", t_ref)
    ecu_kmh = _interp_channel(channels, "ecu_speed", t_ref)

    n = len(t_ref)
    if mate_kmh is None or ecu_kmh is None:
        # no mate or no ecu_speed -> can't attribute, use as-is
        return corner_kmh, np.full(n, "log_speed", dtype=object)

    window_samples = max(1, round(wg["window_s"] * sample_rate_hz))
    valid = _rolling_plausibility_mask(corner_kmh, mate_kmh, ecu_kmh, moving_mask, window_samples,
                                        wg["std_min_kmh"], wg["ratio_max_deviation"])
    if valid.all():
        return corner_kmh, np.full(n, "log_speed", dtype=object)

    abs_ch = channels.get(ABS_SPEED_CHANNEL[corner])
    if abs_ch is not None and abs_ch.get("quality") not in ("missing", "failed") and abs_ch.get("time") is not None:
        abs_kmh_native = np.interp(t_ref, abs_ch["time"], abs_ch["data"])
        abs_kmh = _normalize_wheel_speed_to_kmh(abs_kmh_native, abs_ch.get("unit_raw"))
        out_kmh = np.where(valid, corner_kmh, abs_kmh)
        source = np.where(valid, "log_speed", "abs_speed_fallback")
    else:
        out_kmh = np.where(valid, corner_kmh, np.nan)
        source = np.where(valid, "log_speed", "nan_no_fallback")
    return out_kmh, source


def estimate_longitudinal_forces(state, channels, params):
    """Axle Fx_f/Fx_r. No Fx channel in the log -> fallback tier only.
    fx_total = m*ax + drag + rolling (Rajamani ch. 2); chair performance_analysis
    tooling (internal), fallback tier, as-is.
    Braking split by log_pbrake_f/r; drive all rear (RWD).
    ax < 0 under braking.
    """
    ls = params["longitudinal_stiffness"]
    vp = params["vehicle"]
    aero = vp["aero"]

    t_ref = state["time"]
    ax = state["ax_mps2"]
    v_mps = state["v_mps"]
    mass = vp["mass_kg"]

    v_forward = np.maximum(v_mps, 0.0)
    drag_N = 0.5 * aero["air_density_kgm3"] * ls["drag_coeff"] * aero["cross_track_area_m2"] * v_forward ** 2
    rolling_N = ls["rolling_resistance_coeff"] * mass * 9.81 * np.sign(v_forward)
    fx_total_N = mass * ax + drag_N + rolling_N

    brake_f = _interp_channel(channels, "log_pbrake_f", t_ref)
    brake_r = _interp_channel(channels, "log_pbrake_r", t_ref)

    fallback_fraction = ls["brake_front_fraction_fallback"]
    if brake_f is not None and brake_r is not None:
        brake_total = brake_f + brake_r
        brake_front_fraction = np.full(len(t_ref), fallback_fraction, dtype=float)
        np.divide(brake_f, brake_total, out=brake_front_fraction, where=brake_total > 1e-9)
        brake_front_fraction = np.clip(brake_front_fraction, 0.0, 1.0)
        brake_split_source = "measured log_pbrake_f/log_pbrake_r"
    else:
        brake_front_fraction = np.full(len(t_ref), fallback_fraction, dtype=float)
        brake_split_source = "fallback constant (brake pressure channel unavailable)"

    fx_brake_N = np.minimum(fx_total_N, 0.0)
    fx_drive_N = np.maximum(fx_total_N, 0.0)
    drive_front_fraction = 0.0  # RWD

    fx_f_N = fx_brake_N * brake_front_fraction + fx_drive_N * drive_front_fraction
    fx_r_N = fx_brake_N * (1.0 - brake_front_fraction) + fx_drive_N * (1.0 - drive_front_fraction)

    return {
        "fx_f_N": fx_f_N,
        "fx_r_N": fx_r_N,
        "fx_N": fx_total_N,
        "brake_front_fraction": brake_front_fraction,
        "brake_split_source": brake_split_source,
        "accuracy_level": params["accuracy_levels"]["longitudinal_force_split"]["level"],
    }


def estimate_slip_ratio(state, channels, params):
    """Per-axle slip ratio kappa = (v_axle - v_ref) / v_ref, v_ref = ecu_speed
    (Rajamani ch. 2.2). Axle-level proxy, no per-corner kinematics.
    Same formula as diagnostics/inspect_combined_slip_premise.py.

    Rear divided by (1 + rear_rolling_radius_offset) -- measured +1.41%
    rolling-radius difference, not slip. Front uncorrected: ~0% offset off
    the brakes; the braking deviation is the signal.
    Axle speed = mean of the two guarded corner speeds (_guarded_wheel_speed_kmh;
    trigger case: v3 log_speed_rr, diagnostics/inspect_v3_wheel_speed_census.py).
    """
    ls = params["longitudinal_stiffness"]
    t_ref = state["time"]
    v_ecu_kmh = state["v_mps"] * 3.6
    moving_mask = state["moving_mask"]
    sample_rate_hz = state["sample_rate_hz"]

    wheel_speed_source = {}

    def axle_speed_kmh(corners):
        vals = []
        for c in corners:
            v, source = _guarded_wheel_speed_kmh(channels, c, t_ref, moving_mask, sample_rate_hz, params)
            if v is None:
                return None
            vals.append(v)
            wheel_speed_source[c] = source
        return np.mean(vals, axis=0)

    v_front_kmh = axle_speed_kmh(("fl", "fr"))
    v_rear_kmh = axle_speed_kmh(("rl", "rr"))

    # below min_speed -> NaN, not inf: inf would spread through the whole
    # filtfilt output. kappa undefined at v_ref = 0 anyway.
    v_floor_kmh = ls["min_speed_mps"] * 3.6
    speed_ok = v_ecu_kmh >= v_floor_kmh

    with np.errstate(divide="ignore", invalid="ignore"):
        kappa_f = np.where(speed_ok, (v_front_kmh - v_ecu_kmh) / v_ecu_kmh, np.nan) if v_front_kmh is not None else None
        if v_rear_kmh is not None:
            v_rear_corrected_kmh = v_rear_kmh / (1.0 + ls["rear_rolling_radius_offset"])
            kappa_r = np.where(speed_ok, (v_rear_corrected_kmh - v_ecu_kmh) / v_ecu_kmh, np.nan)
        else:
            kappa_r = None

    n = len(t_ref)
    return {
        "kappa_f": kappa_f if kappa_f is not None else np.full(n, np.nan),
        "kappa_r": kappa_r if kappa_r is not None else np.full(n, np.nan),
        "source_available": v_front_kmh is not None and v_rear_kmh is not None,
        "wheel_speed_source": wheel_speed_source,  # {"fl": array[str], ...}
    }
