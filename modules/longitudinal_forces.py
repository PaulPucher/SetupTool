# CoG longitudinal force (drag included) -- input to the fit-L axle split
# and the floor-event attributes (tyre_fit_auto). SI units.

import numpy as np


def estimate_longitudinal_forces(state, channels, params):
    """CoG Fx from the longitudinal balance; no Fx channel in the log.
    fx_total = m*ax + drag + rolling (Rajamani ch. 2); chair performance_analysis
    tooling (internal), fallback tier, as-is. ax < 0 under braking. The axle
    split is done by the caller (tyre_fit_auto, by measured axle load).
    """
    lf = params["longitudinal_stiffness"]
    vp = params["vehicle"]
    aero = vp["aero"]

    ax = state["ax_mps2"]
    v_mps = state["v_mps"]
    mass = vp["mass_kg"]

    v_forward = np.maximum(v_mps, 0.0)
    drag_N = 0.5 * aero["air_density_kgm3"] * lf["drag_coeff"] * aero["cross_track_area_m2"] * v_forward ** 2
    rolling_N = lf["rolling_resistance_coeff"] * mass * 9.81 * np.sign(v_forward)
    fx_total_N = mass * ax + drag_N + rolling_N

    return {
        "fx_N": fx_total_N,
        "accuracy_level": params["accuracy_levels"]["longitudinal_force_split"]["level"],
    }
