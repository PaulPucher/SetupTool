# Axle Fx from the CoG longitudinal balance (drag included) -- input to the
# fit-L axle split and the floor-event attributes (tyre_fit_auto). SI units.
# The slip-ratio / longitudinal-stiffness chain was removed in the
# 2026-10-22 pruning (tag pre-prune-2026-10-22).

import numpy as np


def _interp_channel(channels, ch_name, t_ref):
    ch = channels.get(ch_name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def estimate_longitudinal_forces(state, channels, params):
    """Axle Fx_f/Fx_r. No Fx channel in the log -> fallback tier only.
    fx_total = m*ax + drag + rolling (Rajamani ch. 2); chair performance_analysis
    tooling (internal), fallback tier, as-is.
    Braking split by log_pbrake_f/r; drive all rear (RWD).
    ax < 0 under braking.
    """
    lf = params["longitudinal_stiffness"]
    vp = params["vehicle"]
    aero = vp["aero"]

    t_ref = state["time"]
    ax = state["ax_mps2"]
    v_mps = state["v_mps"]
    mass = vp["mass_kg"]

    v_forward = np.maximum(v_mps, 0.0)
    drag_N = 0.5 * aero["air_density_kgm3"] * lf["drag_coeff"] * aero["cross_track_area_m2"] * v_forward ** 2
    rolling_N = lf["rolling_resistance_coeff"] * mass * 9.81 * np.sign(v_forward)
    fx_total_N = mass * ax + drag_N + rolling_N

    brake_f = _interp_channel(channels, "log_pbrake_f", t_ref)
    brake_r = _interp_channel(channels, "log_pbrake_r", t_ref)

    fallback_fraction = lf["brake_front_fraction_fallback"]
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
