# Longitudinal axle forces and per-axle slip ratio for SetupTool.
# Pure Python/numpy. No Qt imports.
# Units: SI throughout (m, s, N, kg), except kappa (dimensionless fraction).
#
# PLAN.md STEP 3 (LS_ratio), Phase 1. Inputs for modules/longitudinal_
# stiffness.py's Module-4b-equivalent estimator, mirroring the chair's
# own two-stage construction: axle Fx first (this module), then the
# windowed dFx/dkappa regression (modules/longitudinal_stiffness.py).

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
    """abs_speed_* varies by export -- GT3_PRC_MLA-v3.txt logs kph
    already, Sample_Dubai.txt logs mph (both confirmed directly against
    their own raw files, Fz-integration Phase 5, 2026-09-03) -- the same
    per-file-unit hazard as lap_distance's ft/m fix and log_susp_travel's
    mm/m fix, same remedy: check the file's own claimed unit, never
    assume. log_speed_* (this module's own primary wheel-speed source)
    is always km/h, so this normalises the FALLBACK source to match it.
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
    """Fz-integration Phase 5: per-window (non-overlapping, same discrete-
    window convention as modules.wheel_loads._channel_is_dead's whole-
    session std guard, generalised here to a window so a TRANSIENT fault
    is caught without demoting the channel for the whole session).

    Two conditions, NOT symmetric between corner/mate:
    (1) STUCK -- this corner's own speed shows near-zero variance while
        the car is moving (evaluated on moving samples only, so genuine
        standstill is never mistaken for a stuck sensor). Single-channel,
        no ambiguity -- flags this corner directly.
    (2) MATE DISAGREEMENT -- mean ratio to the axle-mate deviates from 1.0
        beyond ratio_max_deviation (dropout/spike). Real per-corner
        cornering speed differential (outer wheel travels a longer arc)
        is a genuine, expected effect, not a fault -- ratio_max_deviation
        is gap-selected to sit above that population and below the fault
        population (config's own derived_from comment). A DISAGREEING
        PAIR DOES NOT MEAN BOTH ARE WRONG -- found empirically (Fz-
        integration Phase 5, thesis_notes.md): a first version of this
        check flagged v3's own healthy log_speed_rl almost as often as
        its genuinely faulty mate log_speed_rr, purely because a mate-
        only ratio check cannot tell which side of a disagreement is at
        fault. ecu_speed (ax le-independent, not affected by either
        wheel's own fault) is the tie-breaker: only the corner whose OWN
        mean deviation from ecu_speed exceeds its mate's is flagged.
        (Not used as a primary check on its own -- real slip/braking
        events are LEGITIMATE large deviations from ecu_speed, the same
        reason modules.longitudinal_stiffness's kerb guard never
        excludes on kappa magnitude alone; ecu_speed here only breaks a
        tie the mate-disagreement check already raised.)
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
        # Flag this corner if it is the worse-attributed side, OR if
        # attribution itself is impossible (ecu_speed unusable this
        # window) -- conservative default when the tie-breaker has no
        # evidence to offer, same "never silently trust" posture as the
        # rest of this guard.
        if not (np.isfinite(m_corner) and np.isfinite(m_mate)) or m_corner >= m_mate:
            valid[start:end][w_moving] = False
    return valid


def _guarded_wheel_speed_kmh(channels, corner, t_ref, moving_mask, sample_rate_hz, params):
    """Fz-integration Phase 5: guarded log_speed_{corner} -- invalid
    windows (see _rolling_plausibility_mask) are replaced by the
    corresponding abs_speed_{corner} channel (unit-normalised), where
    available; NaN where no fallback exists. Returns (kmh array or None
    if log_speed_{corner} itself is unavailable, source array of str).
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
        # No mate to ratio-check against, or no ecu_speed to attribute a
        # disagreement with -- log_speed_{corner} is used as-is (the
        # stuck check alone, without a mate, would be too weak/one-sided
        # a guard to act on; same "cannot attribute, do not guess" stance
        # as the mate-disagreement branch above).
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
    """Compute axle longitudinal force Fx_f/Fx_r -- fallback tier only (no direct
    per-wheel or per-axle Fx channel exists in the Cosworth log).

    Method anchor recorded in thesis_notes.md, "Citation cross-
    reference, modules/longitudinal_forces.py" entry. Same construction
    as the chair performance_analysis tooling's own
    calculate_longitudinal_axle_forces() third fallback tier
    (docs/literature/longitudinal_stiffness_estimator.py, internal) --
    adopted as-is, no deviation: fx_total = m*ax + drag + rolling
    recovers the tyre-contact-patch force from the measured net
    acceleration by adding back the resistive forces the tyres had to
    overcome. Braking split by measured front/rear brake pressure
    (log_pbrake_f/log_pbrake_r, both whitelisted); driving force
    assigned entirely to the rear axle (rear-wheel-drive GT3R, same
    drive_front_fraction=0.0 the chair's own fallback tier hardcodes).

    ax_mps2 sign convention (state["ax_mps2"], stability_analysis.py
    prepare_vehicle_state): negative under braking -- confirmed against
    this session's own data in the WP5b(b) Fz sign-convention check
    (thesis_notes.md), reused here rather than re-verified, since it is
    the same channel/convention.
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
    drive_front_fraction = 0.0  # rear-wheel-drive GT3R, chair-identical fallback assumption

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
    """Compute per-axle kinematic slip ratio kappa = (v_axle_corrected - v_ref) /
    v_ref, v_ref = ecu_speed. Tier B (signal/data engineering -- standard
    slip-ratio construction, method anchor recorded in thesis_notes.md,
    "Citation cross-reference, modules/longitudinal_forces.py" entry;
    no per-corner kinematic correction, a wheel-speed-vs-vehicle-speed
    proxy). Reproduces diagnostics/inspect_combined_slip_premise.py's
    formula exactly (same v_ref, same rear rolling-radius correction) so
    this module's output is externally checkable against that entry's
    already-recorded percentile figures (thesis_notes.md).

    log_speed_fl/fr/rl/rr are the WP-S1-designated wheel-speed family
    (byte-identical to ecu_speed_wheels_*), whitelisted by this phase.
    Rear axle speed is divided by (1 + rear_rolling_radius_offset) --
    WP-S1's measured, throttle-independent +1.41% rolling-radius
    difference, not a slip artifact (thesis_notes.md "Wheel-speed source
    characterization (WP-S1)"). Front axle is left uncorrected: its own
    off-braking offset is ~0%, and its braking-specific deviation IS the
    front-slip-under-braking signal this ratio exists to measure.

    Fz-integration Phase 5 (2026-09-03): each corner's own log_speed_*
    passes through _guarded_wheel_speed_kmh first (per-window stuck/
    implausible-ratio-to-mate guard, config wheel_speed_guard.*, falling
    back to abs_speed_* where the guard trips and that channel exists --
    real bug motivating this, GT3_PRC_MLA-v3.txt's own log_speed_rr:
    diagnostics/inspect_v3_wheel_speed_census.py, thesis_notes.md "Fz-
    integration Phase 5..."). Axle speed is the mean of the two GUARDED
    corner speeds, not the two raw ones -- kappa_f/kappa_r therefore
    already reflect the guarded/fallback channel with no separate wiring
    needed at either of this function's own two consumers (this module's
    slip ratio and modules.longitudinal_stiffness's kerb-adjacent
    plausibility guard, which reads kappa_r/kappa_f from here).
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

    # v_ecu_kmh sits at/near zero for every stationary (pit/grid) sample --
    # dividing there gives +/-inf, not NaN, which poisons Phase 2's
    # Butterworth filtfilt across the WHOLE array (a linear filter, unlike
    # pandas .interpolate(), does not stop at a single bad sample). Floored
    # at the same min_speed_mps gate Phase 2's estimator applies anyway
    # (longitudinal_stiffness.min_speed_mps), so kappa is NaN, never inf,
    # below it -- physically correct too: slip ratio is undefined at zero
    # reference speed.
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
        "wheel_speed_source": wheel_speed_source,  # {"fl": array[str], ...} -- see _guarded_wheel_speed_kmh
    }
