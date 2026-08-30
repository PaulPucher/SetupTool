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


def _interp_channel(channels, ch_name, t_ref):
    ch = channels.get(ch_name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def estimate_longitudinal_forces(state, channels, params):
    """Compute axle longitudinal force Fx_f/Fx_r -- fallback tier only (no direct
    per-wheel or per-axle Fx channel exists in the Cosworth log).

    Tier A: Rajamani, Vehicle Dynamics and Control, 2nd ed., Ch. 2
    (longitudinal equation of motion, F_x = m*a_x + resistive forces).
    Same construction as the chair performance_analysis tooling's own
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
    slip-ratio construction, Rajamani Ch. 2 sec. 2.2 kappa definition;
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
    """
    ls = params["longitudinal_stiffness"]
    t_ref = state["time"]
    v_ecu_kmh = state["v_mps"] * 3.6

    def axle_speed_kmh(names):
        vals = [_interp_channel(channels, n, t_ref) for n in names]
        if any(v is None for v in vals):
            return None
        return np.mean(vals, axis=0)

    v_front_kmh = axle_speed_kmh(WHEEL_NAMES["front"])
    v_rear_kmh = axle_speed_kmh(WHEEL_NAMES["rear"])

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
    }
