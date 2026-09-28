# Per-wheel Fz from pushrod force + suspension travel.
# Segers ch. 9 (pushrod force -> wheel via motion ratio), ch. 10 (geometric
# vs elastic load transfer).
#
# Fz = four additive terms per wheel:
#   1. sprung: pushrod force x motion ratio -- static weight, aero, pitch
#      and the elastic share of lateral transfer (all through the spring)
#   2. ARB: parallel load path, gauge doesn't see it
#   3. unsprung lateral transfer: straight to the contact patch, bypasses gauge
#   4. geometric (roll-centre) lateral transfer -- elastic share already in 1
#
# Bump rubbers not modelled -> underestimate at extreme compression.

import numpy as np

CORNERS = ("fl", "fr", "rl", "rr")
CORNER_AXLE = {"fl": "front", "fr": "front", "rl": "rear", "rr": "rear"}
CORNER_SIDE = {"fl": "left", "fr": "right", "rl": "left", "rr": "right"}
DAMPER_CHANNEL = {c: f"log_dms_dam_{c}" for c in CORNERS}
TRAVEL_CHANNEL = {c: f"log_susp_travel_{c}" for c in CORNERS}
AXLE_CORNERS = {"front": ("fl", "fr"), "rear": ("rl", "rr")}

# ay > 0 loads the right wheels -- same convention as estimate_vertical_loads
SIDE_SIGN = {"left": -1.0, "right": +1.0}


def _interp_channel(channels, ch_name, t_ref):
    ch = channels.get(ch_name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def _normalize_travel_to_mm(data, unit_raw):
    """log_susp_travel_* unit differs per export (Dubai: m, v3: mm) -- trust
    unit_raw. Everything downstream (motion-ratio table, ARB) is in mm.
    """
    if unit_raw == "mm":
        return data
    if unit_raw == "m":
        return data * 1000.0
    raise ValueError(
        f"log_susp_travel unit {unit_raw!r} not recognised (expected 'mm' or 'm') -- "
        "add explicit handling before trusting this export's travel values"
    )


def _check_damper_force_unit(unit_raw, ch_name):
    # non-N force needs a gauge calibration, not a factor -> refuse
    if unit_raw != "N":
        raise ValueError(
            f"{ch_name} unit {unit_raw!r} not recognised (expected 'N') -- "
            "add explicit handling before trusting this export's force values"
        )


def _channel_is_dead(data, std_max):
    """Frozen-at-a-plausible-value sensor (Dubai log_susp_travel_rr) -- the
    range gate can't see it. std, not range: one spike would fool range.
    """
    return bool(np.nanstd(data) < std_max)


def _motion_ratio(travel_mm, axle, car_data):
    """MR = damper travel / wheel travel (car_data table, all < 1).
    Virtual work: F_wheel = F_damper * MR. Clipped at table ends, no
    extrapolation.
    """
    table = car_data["motion_ratio_vs_wheel_travel"][axle]
    xs = np.array([p[0] for p in table["points"]], dtype=float)
    ys = np.array([p[1] for p in table["points"]], dtype=float)
    order = np.argsort(xs)
    return np.interp(travel_mm, xs[order], ys[order])


def _arb_rate_n_per_mm(axle, position, car_data):
    """ARB rate at the wheel (car_data arb[axle] x ratio_to_wheel).
    position clamped to 1-7 -- fallback position must always resolve.
    """
    arb = car_data["arb"][axle]
    position = int(min(max(position, 1), 7))
    rate_drop_link = arb["positions"][str(position)]
    return rate_drop_link * arb["ratio_to_wheel"]


def estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=None):
    """Per-corner damper-derived Fz where both channels are usable.

    arb_position: int for all corners, or {"front"/"fl"/...: int};
    missing -> wheel_loads.arb_position_fallback.
    Corner valid = both channels "valid", known units (else ValueError),
    and not dead (_channel_is_dead).

    Returns per corner: fz_N, valid, dead_channel, arb_valid (False where
    the axle has no ARB term -> ARB contributes 0), and the four component
    arrays.
    """
    wl = params["wheel_loads"]
    vp = params["vehicle"]
    t_ref = state["time"]
    ay = state["ay_mps2"]
    n = len(t_ref)

    if arb_position is None:
        arb_position = {}
    elif isinstance(arb_position, (int, float)):
        arb_position = {c: int(arb_position) for c in CORNERS}

    track = {"front": vp["track_width_front_m"], "rear": vp["track_width_rear_m"]}
    h_u = {"front": wl["tyre_dynamic_radius_front_m"], "rear": wl["tyre_dynamic_radius_rear_m"]}
    z_rc = {"front": wl["roll_centre_front_m"], "rear": wl["roll_centre_rear_m"]}
    m_unsprung = {"front": wl["unsprung_mass_front_kg"], "rear": wl["unsprung_mass_rear_kg"]}
    corner_weight_kg = {
        "fl": vp["corner_weights"]["FL_kg"], "fr": vp["corner_weights"]["FR_kg"],
        "rl": vp["corner_weights"]["RL_kg"], "rr": vp["corner_weights"]["RR_kg"],
    }
    g = 9.81

    # pushrod force - offset, x MR -> sprung wheel force
    pushrod_N = {}
    travel_mm = {}
    mr = {}
    valid = {}
    dead_channel = {}
    for c in CORNERS:
        axle = CORNER_AXLE[c]
        raw_force = _interp_channel(channels, DAMPER_CHANNEL[c], t_ref)
        raw_travel_native = _interp_channel(channels, TRAVEL_CHANNEL[c], t_ref)
        ch_force = channels.get(DAMPER_CHANNEL[c])
        ch_travel = channels.get(TRAVEL_CHANNEL[c])
        quality_ok = (raw_force is not None and raw_travel_native is not None
                      and ch_force.get("quality") == "valid" and ch_travel.get("quality") == "valid")

        if quality_ok:
            _check_damper_force_unit(ch_force.get("unit_raw"), DAMPER_CHANNEL[c])
            raw_travel = _normalize_travel_to_mm(raw_travel_native, ch_travel.get("unit_raw"))
            dead_channel[c] = (_channel_is_dead(raw_force, wl["dead_channel_std_max_force_N"])
                                or _channel_is_dead(raw_travel, wl["dead_channel_std_max_travel_mm"]))
            ok = not dead_channel[c]
        else:
            dead_channel[c] = False
            ok = False

        valid[c] = np.full(n, ok, dtype=bool)
        if ok:
            offset = wl[f"pushrod_offset_{c}_N"]
            travel_mm[c] = raw_travel
            mr[c] = _motion_ratio(raw_travel, axle, car_data)
            pushrod_N[c] = (raw_force - offset) * mr[c]
        else:
            travel_mm[c] = np.full(n, np.nan)
            mr[c] = np.full(n, np.nan)
            pushrod_N[c] = np.full(n, np.nan)

    # ARB = left-right travel delta x rate / MR (damper MR as proxy, no ARB
    # linkage table). Larger travel = outside wheel = positive; sign
    # checked against ay on real data.
    # One dead travel channel kills the whole axle's ARB term -> arb_valid,
    # since fz_N below zeroes a NaN arb_N.
    arb_N = {c: np.full(n, np.nan) for c in CORNERS}
    arb_valid = {c: np.zeros(n, dtype=bool) for c in CORNERS}
    for axle, (left_c, right_c) in (("front", ("fl", "fr")), ("rear", ("rl", "rr"))):
        both_ok = valid[left_c] & valid[right_c]
        arb_valid[left_c] = both_ok
        arb_valid[right_c] = both_ok
        if not both_ok.any():
            continue
        position = arb_position.get(axle, arb_position.get(left_c, wl["arb_position_fallback"]))
        rate = _arb_rate_n_per_mm(axle, position, car_data)
        delta_mm = travel_mm[left_c] - travel_mm[right_c]
        avg_mr = 0.5 * (mr[left_c] + mr[right_c])
        with np.errstate(invalid="ignore"):
            force_half = 0.5 * delta_mm * rate / avg_mr
        arb_N[left_c] = np.where(both_ok, -force_half, arb_N[left_c])
        arb_N[right_c] = np.where(both_ok, force_half, arb_N[right_c])

    # terms 3 and 4
    unsprung_N = {}
    geometric_N = {}
    for c in CORNERS:
        axle = CORNER_AXLE[c]
        side_sign = SIDE_SIGN[CORNER_SIDE[c]]
        m_sprung_axle = sum(corner_weight_kg[cc] for cc in AXLE_CORNERS[axle]) - 2.0 * m_unsprung[axle]
        unsprung_N[c] = np.where(
            valid[c], side_sign * ay * m_unsprung[axle] * h_u[axle] / track[axle], np.nan)
        geometric_N[c] = np.where(
            valid[c], side_sign * ay * m_sprung_axle * z_rc[axle] / track[axle], np.nan)

    result = {}
    for c in CORNERS:
        # missing ARB -> 0, flagged via arb_valid
        fz_damper = pushrod_N[c] + np.nan_to_num(arb_N[c], nan=0.0) + unsprung_N[c] + geometric_N[c]
        result[c] = {
            "fz_N": fz_damper,
            "valid": valid[c],
            "dead_channel": dead_channel[c],
            "arb_valid": arb_valid[c],
            "sprung_N": pushrod_N[c],
            "arb_N": arb_N[c],
            "unsprung_transfer_N": unsprung_N[c],
            "geometric_transfer_N": geometric_N[c],
            "motion_ratio": mr[c],
        }
    return result


def combine_with_static_fallback(damper_result, static_fallback_fz):
    """Damper Fz where valid, static split elsewhere -- per corner and
    sample, never a whole-session switch.
    """
    combined = {}
    for c in CORNERS:
        dr = damper_result[c]
        fz = np.where(dr["valid"], dr["fz_N"], static_fallback_fz[c])
        source = np.where(dr["valid"], "damper", "static_fallback")
        combined[c] = {"fz_N": fz, "source": source}
    return combined


AXLE_MATE = {"fl": "fr", "fr": "fl", "rl": "rr", "rr": "rl"}
AXLE_TOTAL_KEY = {"fl": "fz_f_N", "fr": "fz_f_N", "rl": "fz_r_N", "rr": "fz_r_N"}


def _axle_total_with_proxy(damper_result, corner_weight_kg, left_c, right_c):
    """Axle total for the straight-line fits: real corner values where valid;
    one invalid -> mate x static corner-weight ratio. Rear ratio != 1, so
    that proxy puts a static L/R split on a roll-dependent quantity -- weaker
    than front, still better than NaN.
    Both invalid -> NaN, degraded = True with reason.
    Returns (total_N, degraded, reason).
    """
    left_valid = damper_result[left_c]["valid"]
    right_valid = damper_result[right_c]["valid"]
    left_fz = damper_result[left_c]["fz_N"]
    right_fz = damper_result[right_c]["fz_N"]

    left_ratio = corner_weight_kg[left_c] / corner_weight_kg[right_c]
    right_ratio = corner_weight_kg[right_c] / corner_weight_kg[left_c]

    with np.errstate(invalid="ignore"):
        left_value = np.where(left_valid, left_fz, right_fz * left_ratio)
        right_value = np.where(right_valid, right_fz, left_fz * right_ratio)
        total = left_value + right_value

    both_invalid = ~left_valid & ~right_valid
    total = np.where(both_invalid, np.nan, total)

    degraded = bool(np.any(both_invalid))
    reason = None
    if degraded:
        frac = float(np.mean(both_invalid))
        reason = (f"{left_c}/{right_c}: both corners invalid for {frac*100:.1f}% of samples -- "
                  "axle total is NaN there, no mate to proxy from on either side")
    return total, degraded, reason


def effective_cl_area(c_session, air_density_kgm3):
    """c_session [N/(m/s)^2] as the Cl*A [m^2] of the config aero model.
    From c*v^2 = -0.5*rho*A*Cl*v^2 (Cl < 0 = downforce, same sign as
    estimate_vertical_loads). Display only -- config is not filled from it:
    the fit is level 2 and already outranks a level-1 config value."""
    return -2.0 * c_session / air_density_kgm3


def estimate_session_corrected_axle_totals(state, damper_result, params):
    """Session-measured axle totals for the corner reconstruction only --
    estimate_vertical_loads and config lift_coeff stay untouched. Level 2.

    Static model under-reads the rear total by ~25%: config mass too low,
    lift_coeff = 0 has no aero.
    (1)/(2) one joint fit on straight_wide (moving, |ay| < 1.5):
        total(v) = static_total + c_session * v^2
        intercept -> mass_kg_session, all speed dependence -> c_session, no
        double count. No ax term -- axle total is invariant to long. transfer.
    (3) front_mass_fraction = measured front/(front+rear) on straights, for
        the static term only. Aero split stays config (aero_front_fraction):
        straights give no signal for it.
    (4) rear_left_fraction: measured RL/(RL+RR), reported only, never proxied.
        NaN if a rear corner is dead all session.

    Only real corner data feeds the fits (_axle_total_with_proxy), never the
    reconstruction itself -- no self-correction loop.
    Check: diagnostics/inspect_v3_reconstruction_ground_truth.py.

    Returns fz_f_N/fz_r_N + mass_kg_session, c_session_N_per_mps2,
    aero_front_fraction. Never written back to config.
    """
    ls = params["stability_estimation"]
    vp = params["vehicle"]
    wl = params["wheel_loads"]
    t_ref = state["time"]
    v = state["v_mps"]
    ax = state["ax_mps2"]
    ay = state["ay_mps2"]
    g = 9.81
    n = len(t_ref)

    corner_weight_kg = {
        "fl": vp["corner_weights"]["FL_kg"], "fr": vp["corner_weights"]["FR_kg"],
        "rl": vp["corner_weights"]["RL_kg"], "rr": vp["corner_weights"]["RR_kg"],
    }

    moving = v >= ls["moving_speed_min_mps"]
    straight_tight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    straight_wide = moving & (np.abs(ay) < 1.5)

    front_total_N, front_degraded, front_degraded_reason = _axle_total_with_proxy(
        damper_result, corner_weight_kg, "fl", "fr")
    rear_total_N, rear_degraded, rear_degraded_reason = _axle_total_with_proxy(
        damper_result, corner_weight_kg, "rl", "rr")
    total_fz_for_fit_N = front_total_N + rear_total_N

    # joint fit total(v) = m*g + c*v^2 -- intercept = static, v^2 = aero, no
    # overlap; no ax term (docstring (1)/(2))
    X_total = np.column_stack([np.ones(int(straight_wide.sum())), v[straight_wide] ** 2])
    y_total = total_fz_for_fit_N[straight_wide]
    coeffs_total, _, _, _ = np.linalg.lstsq(X_total, y_total, rcond=None)
    static_total_N = float(coeffs_total[0])
    c_session = float(coeffs_total[1])
    mass_kg_session = static_total_N / g

    mean_front_straight_N = float(np.mean(front_total_N[straight_tight]))
    mean_rear_straight_N = float(np.mean(rear_total_N[straight_tight]))
    front_mass_fraction = mean_front_straight_N / (mean_front_straight_N + mean_rear_straight_N)

    # (4) reported only, no proxy; NaN if a rear corner is dead
    mean_rl_straight_N = float(np.mean(damper_result["rl"]["fz_N"][straight_tight]))
    mean_rr_straight_N = float(np.mean(damper_result["rr"]["fz_N"][straight_tight]))
    rear_left_fraction = mean_rl_straight_N / (mean_rl_straight_N + mean_rr_straight_N)

    aero_front_fraction = wl["aero_front_fraction"]  # not measurable from straights

    wb = vp["wheelbase_m"]
    h_cog = vp["cog_height_m"]

    fz_static_f_N = mass_kg_session * g * front_mass_fraction
    fz_static_r_N = mass_kg_session * g * (1.0 - front_mass_fraction)
    dfz_aero_f_N = aero_front_fraction * c_session * v ** 2
    dfz_aero_r_N = (1.0 - aero_front_fraction) * c_session * v ** 2
    dfz_long_transfer_N = mass_kg_session * ax * h_cog / wb

    fz_f_N = fz_static_f_N + dfz_aero_f_N - dfz_long_transfer_N
    fz_r_N = fz_static_r_N + dfz_aero_r_N + dfz_long_transfer_N

    return {
        "fz_f_N": fz_f_N,
        "fz_r_N": fz_r_N,
        "mass_kg_session": mass_kg_session,
        "c_session_N_per_mps2": c_session,
        "aero_front_fraction": aero_front_fraction,
        "front_mass_fraction": front_mass_fraction,
        "rear_left_fraction": rear_left_fraction,
        # True where both corners of the axle are invalid somewhere (NaN there)
        "front_correction_degraded": front_degraded,
        "front_correction_degraded_reason": front_degraded_reason,
        "rear_correction_degraded": rear_degraded,
        "rear_correction_degraded_reason": rear_degraded_reason,
    }


def reconstruct_missing_corner(damper_result, fz_axle_totals):
    """One invalid corner = axle total - measured mate (Segers modal
    decomposition, heave+pitch mode only). Roll only shifts load within an
    axle -> no roll/ARB model; the mate carries the real split.

    Limits:
    - single-wheel events on the reconstructed corner are invisible
    - warp (FL+RR)-(FR+RL) unobservable with three sensors
    - inherits the L1 placeholders in fz_axle_totals -> registered Level 1

    Both corners invalid -> NaN, caller falls back to static.
    Returns per corner {"fz_N", "reconstructable"}.
    """
    n = len(damper_result["fl"]["valid"])
    out = {}
    for c in CORNERS:
        mate = AXLE_MATE[c]
        mate_valid = damper_result[mate]["valid"]
        this_invalid = ~damper_result[c]["valid"]
        reconstructable = this_invalid & mate_valid
        axle_total = fz_axle_totals[AXLE_TOTAL_KEY[c]]
        fz = np.where(reconstructable, axle_total - damper_result[mate]["fz_N"], np.nan)
        out[c] = {"fz_N": fz, "reconstructable": reconstructable}
    return out


def combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz):
    """Per corner and sample: damper (L4) -> reconstructed from mate + axle
    total (L1, see reconstruct_missing_corner) -> static split (L1).
    "source" = per-sample tier label.
    """
    reconstructed = reconstruct_missing_corner(damper_result, fz_axle_totals)
    combined = {}
    for c in CORNERS:
        dr = damper_result[c]
        rc = reconstructed[c]
        fz = np.where(dr["valid"], dr["fz_N"],
                       np.where(rc["reconstructable"], rc["fz_N"], static_fallback_fz[c]))
        source = np.where(dr["valid"], "damper",
                           np.where(rc["reconstructable"], "reconstructed", "static_fallback"))
        combined[c] = {"fz_N": fz, "source": source}
    return combined
