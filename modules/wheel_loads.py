# Damper/suspension-travel-derived per-wheel vertical load (Fz) for SetupTool.
# Pure Python/numpy. No Qt imports. Additive module, not yet wired into the
# production pipeline/UI -- same incremental scope as modules/longitudinal_
# forces.py's own first phase.
#
# Method anchor recorded in thesis_notes.md, the wheel-load estimation entry:
# Segers, "Analysis Techniques for Racecar Data Acquisition" (SAE, 2014),
# ch.9 (pushrod/damper force to wheel load via motion ratio) and ch.10
# (roll-centre geometric vs spring-path elastic load-transfer split) --
# page anchors (ch.9 p.199, ch.10 pp.221-256) verified 2026-09-03 by the
# reviewer against the docs/literature excerpt copy (thesis_notes.md).
#
# DECOMPOSITION (four additive terms per wheel):
#   1. sprung force at the wheel, from the measured pushrod/damper force
#      and the digitised motion-ratio table -- captures static sprung
#      weight, aero, longitudinal (pitch) transfer, AND the ELASTIC
#      (spring-compression) share of lateral load transfer, because all
#      of those act through the spring/damper and the pushrod gauge sees
#      them directly.
#   2. ARB force -- a separate load path in parallel with the main spring/
#      damper (the anti-roll bar does not compress the main spring), so it
#      is invisible to the pushrod gauge and must be added independently.
#   3. unsprung-mass lateral transfer -- the unsprung assembly's own
#      inertia reacts directly through the upright/tyre contact patch,
#      bypassing the spring/damper entirely; also invisible to the gauge.
#   4. sprung-mass GEOMETRIC lateral transfer (via the roll centre) -- the
#      portion of the sprung mass's own lateral transfer carried through
#      the suspension linkage geometry rather than through spring
#      compression. Only this portion is added; the ELASTIC portion is
#      already inside term 1 and must not be double-counted.
#
# Bump-rubber engagement is NOT modelled (config wheel_loads.bump_rubber_
# note) -- a documented underestimate at extreme compression, not a defect.

import numpy as np

CORNERS = ("fl", "fr", "rl", "rr")
CORNER_AXLE = {"fl": "front", "fr": "front", "rl": "rear", "rr": "rear"}
CORNER_SIDE = {"fl": "left", "fr": "right", "rl": "left", "rr": "right"}
DAMPER_CHANNEL = {c: f"log_dms_dam_{c}" for c in CORNERS}
TRAVEL_CHANNEL = {c: f"log_susp_travel_{c}" for c in CORNERS}
AXLE_CORNERS = {"front": ("fl", "fr"), "rear": ("rl", "rr")}

# Lateral-transfer sign convention: reused verbatim from modules.
# stability_analysis.estimate_vertical_loads (fz_fl = fz_f/2 -
# lateral_transfer/2, fz_fr = fz_f/2 + lateral_transfer/2) so a positive
# ay_mps2 adds load to the right-side wheels (fr/rr) and removes it from
# the left-side wheels (fl/rl) here too, exactly as it already does in
# the static-split estimator -- deliberately the SAME convention, not an
# independently chosen one, so the two estimators agree at zero-ay.
SIDE_SIGN = {"left": -1.0, "right": +1.0}


def _interp_channel(channels, ch_name, t_ref):
    ch = channels.get(ch_name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def _motion_ratio(travel_mm, axle, car_data):
    """damper_ratio = damper_travel / wheel_travel, from car_data.json's
    motion_ratio_vs_wheel_travel[axle] table. Every digitised point on
    both axles sits below 1 (front 0.615-0.691, rear 0.763-0.781) --
    checked directly, not assumed -- which confirms the standard pushrod/
    rocker convention by conservation of virtual work through the
    linkage: F_damper * damper_travel = F_wheel * wheel_travel, so
    F_wheel = F_damper * (damper_travel / wheel_travel) = F_damper *
    motion_ratio. Interpolated against the table's own wheel_travel_mm
    axis; travel outside the table's digitised range clips to the
    nearest endpoint (a fixed lookup table, no physical basis for
    extrapolating beyond its own measured points).
    """
    table = car_data["motion_ratio_vs_wheel_travel"][axle]
    xs = np.array([p[0] for p in table["points"]], dtype=float)
    ys = np.array([p[1] for p in table["points"]], dtype=float)
    order = np.argsort(xs)
    return np.interp(travel_mm, xs[order], ys[order])


def _arb_rate_n_per_mm(axle, position, car_data):
    """ARB rate at the drop link (config/car_data.json arb[axle]), scaled
    to the wheel via the table's own ratio_to_wheel factor. position is
    clamped to the digitised 1-7 range rather than raising, since a
    session with no real setup record (e.g. GT3_PRC_MLA-v3.txt) supplies
    only the config fallback position, which must always resolve.
    """
    arb = car_data["arb"][axle]
    position = int(min(max(position, 1), 7))
    rate_drop_link = arb["positions"][str(position)]
    return rate_drop_link * arb["ratio_to_wheel"]


def estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=None):
    """Per-wheel vertical load Fz_c_N (c in fl/fr/rl/rr), damper/suspension-
    travel-derived where both channels validate for that corner, else the
    static-split fallback (modules.stability_analysis.estimate_vertical_
    loads's own fz_fl_N/fz_fr_N/fz_rl_N/fz_rr_N, passed in via
    static_fallback_fz).

    arb_position: optional {"front": int, "fl": int, "fr": int, ...} or a
    single int applied to all four corners -- per-corner session setup
    values (setup_parameters arb_fl/fr/rl/rr) when available. Falls back
    to config wheel_loads.arb_position_fallback per corner otherwise.

    Returns a dict keyed by corner with fz_N (array), source ("damper" or
    "static_fallback", array of str, per-sample), and the four component
    arrays (sprung_N, arb_N, unsprung_transfer_N, geometric_transfer_N,
    NaN where the fallback applies) for validation/plotting.
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

    # Raw per-corner channels, damper force offset-corrected and motion-
    # ratio-scaled to a sprung wheel force; travel channels for the
    # motion-ratio lookup and the ARB's left-right delta.
    pushrod_N = {}
    travel_mm = {}
    mr = {}
    valid = {}
    for c in CORNERS:
        axle = CORNER_AXLE[c]
        raw_force = _interp_channel(channels, DAMPER_CHANNEL[c], t_ref)
        raw_travel = _interp_channel(channels, TRAVEL_CHANNEL[c], t_ref)
        ch_force = channels.get(DAMPER_CHANNEL[c])
        ch_travel = channels.get(TRAVEL_CHANNEL[c])
        ok = (raw_force is not None and raw_travel is not None
              and ch_force.get("quality") == "valid" and ch_travel.get("quality") == "valid")
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

    # ARB: per-axle left-right travel delta x ARB table rate / ARB motion
    # ratio (approximated by the same axle's own damper motion ratio,
    # wheel_loads.arb_motion_ratio_approximation_note -- no dedicated ARB
    # linkage motion-ratio table exists in car_data.json). Sign convention
    # matches SIDE_SIGN: the wheel with the LARGER travel value under this
    # file's own logging convention is treated as the more-compressed
    # (outside, in a corner) wheel and gets the positive ARB contribution
    # -- same-direction as the existing lateral-transfer term, since the
    # ARB amplifies (not opposes) the outside wheel's load gain. VERIFIED
    # against real ay correlation in the Phase 2 validation run (thesis_
    # notes.md); flip SIDE_SIGN here if a future session contradicts it.
    arb_N = {c: np.full(n, np.nan) for c in CORNERS}
    for axle, (left_c, right_c) in (("front", ("fl", "fr")), ("rear", ("rl", "rr"))):
        both_ok = valid[left_c] & valid[right_c]
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

    # Unsprung-mass and sprung-mass-geometric lateral transfer, one pair
    # per axle, split by SIDE_SIGN (see module docstring/comment above).
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
        fz_damper = pushrod_N[c] + np.nan_to_num(arb_N[c], nan=0.0) + unsprung_N[c] + geometric_N[c]
        result[c] = {
            "fz_N": fz_damper,
            "valid": valid[c],
            "sprung_N": pushrod_N[c],
            "arb_N": arb_N[c],
            "unsprung_transfer_N": unsprung_N[c],
            "geometric_transfer_N": geometric_N[c],
            "motion_ratio": mr[c],
        }
    return result


def combine_with_static_fallback(damper_result, static_fallback_fz):
    """Per-corner, per-sample: use the damper-derived Fz where valid, the
    static-split estimate (modules.stability_analysis.estimate_vertical_
    loads's fz_fl_N/fz_fr_N/fz_rl_N/fz_rr_N) elsewhere. static_fallback_fz
    is a dict keyed the same way as CORNERS. Never a whole-session
    switch -- GT3_PRC_MLA-v3.txt's own faulted log_dms_dam_fr means FR
    falls back for every sample while FL/RL/RR use the damper source.
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


def estimate_session_corrected_axle_totals(state, damper_result, params):
    """Session-derived correction to the axle-total model consumed by
    reconstruct_missing_corner/combine_with_reconstruction_and_fallback
    ONLY -- never touches modules.stability_analysis.estimate_vertical_
    loads or vehicle.aero.lift_coeff, both of which stay exactly as they
    are for every other production consumer. Config Cl remains a global,
    unsourced Level-1 placeholder; the correction here is a genuinely
    per-session MEASUREMENT (Level 2), not a new global constant.

    MOTIVATION (thesis_notes.md "Closing the reconstruction's aero gap"):
    the ground-truth reconstruction test (drop a real RL/RR, reconstruct
    it, compare to its own measurement) found the static axle-total model
    underestimates the true rear axle total by ~25% on a fast lap --
    traced to two separate, additive gaps in the STATIC model: (a) config
    vehicle.mass_kg under-reads this session's own real loaded mass (the
    already-recorded +10.4% straight-line total finding), and (b) config
    aero.lift_coeff=0.0 omits real, substantial aero downforce entirely.
    Both are corrected here from this session's OWN damper data.

    (1) MASS: mass_kg_session is the session's own measured straight-line
    mean total (same tight mask as the +10.4% finding: moving, |ax|<0.5,
    |ay|<0.5), replacing vehicle.mass_kg for this function's own static-
    split term only.
    (2) AERO: F_aero(v) = c_session * v^2, c_session fit from THIS
    session's own straight-line damper data via the same 3-term
    regression as diagnostics/inspect_v3_aero_load_diagnostic.py's Phase
    5 method (Fz_total = a + b*ax + c*v^2, widened |ay|<1.5 mask), split
    front/rear by wheel_loads.aero_front_fraction (Level 1 placeholder,
    config-stated -- see (3) below for why this one stays a placeholder).
    (3) FRONT/REAR MASS SPLIT: front_mass_fraction is the session's own
    measured axle-total ratio at straight-line samples (front_total /
    (front_total+rear_total), same FL-doubling proxy as (1)/(2) for the
    invalid FR), replacing the static geometric fraction (cog_to_front/
    rear_axle_m / wheelbase_m) for the MASS/static term only -- the
    longitudinal-transfer term keeps the geometric h_cog/wheelbase_m
    formula unchanged (a different physical quantity, not a weight
    split). This closes the gap the first version of this function left
    open (thesis_notes.md "...closing the reconstruction's aero gap":
    funnelling the corrected, larger total through the OLD static
    fraction over-allocated load to the front, since this car's real
    dynamic front/rear split (found separately, Phase 2(d), 37%/63%)
    differs from the static config split). The AERO front/rear fraction
    is explicitly NOT given the same treatment: straight-line data alone
    cannot measure how aero downforce splits front/rear (both wheels of
    an axle see essentially the same speed and near-zero roll at
    straight line, so there is no differential signal to fit an aero
    split from) -- wheel_loads.aero_front_fraction stays the Level 1
    config placeholder, unchanged, and this is stated here rather than
    silently left implied.
    (4) REAR LEFT/RIGHT (reported, not consumed): rear_left_fraction is
    the session's own measured RL/(RL+RR) ratio at straight-line samples
    -- both rear corners are real all session, so this needs no proxy at
    all, unlike every fraction above. Not used anywhere in this
    function's own fz_f_N/fz_r_N (the per-wheel L/R split still comes
    from estimate_wheel_loads_from_dampers's own ARB/unsprung/geometric
    decomposition, which already uses real per-corner data) -- returned
    purely for the caller's own reporting/comparison against config's
    static RL_kg/RR_kg split.

    NON-CIRCULARITY: both fits need a whole-car Fz_total, but FR is
    invalid all session -- using the (biased) axle-total-model-based
    reconstruction for FR here would make this function correct itself
    against its own error. Instead FR is proxied by its own axle-mate FL
    for the PURPOSE OF THIS FIT ONLY (front-axle static symmetry: config
    vehicle.corner_weights states FL_kg==FR_kg exactly, so assuming near-
    zero real roll asymmetry at low-ay/straight-line conditions is a
    small, bounded approximation, not an arbitrary guess) -- ONLY FL/RL/
    RR's real measurements feed these two fits, never a model output.

    KNOWN IMPERFECTION, stated not hidden: mass_kg_session (a straight-
    line MEAN across a real speed range) already contains some of the
    real aero present at that range's own typical speed; adding a full,
    separate c_session*v^2 term on top therefore double-counts a small
    aero share already implicit in the mean, rather than being a clean
    zero-speed/speed-dependent split (which would require fitting the
    intercept of the SAME regression, deliberately not done here per the
    work order's own explicit instruction to use "the +10.4% finding"
    directly). Expected to be a second-order effect against the ~25%
    gap being closed -- checked empirically in the Phase 3 re-run, not
    just argued.

    Returns fz_f_N/fz_r_N arrays plus the derived scalars (mass_kg_
    session, c_session_N_per_mps2, aero_front_fraction) for the caller to
    record/report -- computed at analysis time, never persisted to config.
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

    moving = v >= ls["moving_speed_min_mps"]
    straight_tight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    straight_wide = moving & (np.abs(ay) < 1.5)

    fr_proxy_N = damper_result["fl"]["fz_N"]  # non-circular FR stand-in, see docstring
    front_total_N = damper_result["fl"]["fz_N"] + fr_proxy_N
    rear_total_N = damper_result["rl"]["fz_N"] + damper_result["rr"]["fz_N"]
    total_fz_for_fit_N = front_total_N + rear_total_N

    mass_kg_session = float(np.mean(total_fz_for_fit_N[straight_tight])) / g

    mean_front_straight_N = float(np.mean(front_total_N[straight_tight]))
    mean_rear_straight_N = float(np.mean(rear_total_N[straight_tight]))
    front_mass_fraction = mean_front_straight_N / (mean_front_straight_N + mean_rear_straight_N)

    # Reported only -- both real, no proxy needed. Not consumed below.
    mean_rl_straight_N = float(np.mean(damper_result["rl"]["fz_N"][straight_tight]))
    mean_rr_straight_N = float(np.mean(damper_result["rr"]["fz_N"][straight_tight]))
    rear_left_fraction = mean_rl_straight_N / (mean_rl_straight_N + mean_rr_straight_N)

    X = np.column_stack([np.ones(int(straight_wide.sum())), ax[straight_wide], v[straight_wide] ** 2])
    y = total_fz_for_fit_N[straight_wide]
    coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    c_session = float(coeffs[2])

    aero_front_fraction = wl["aero_front_fraction"]  # NOT session-measurable, see docstring (3)

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
    }


def reconstruct_missing_corner(damper_result, fz_axle_totals):
    """Reconstruct a single invalid corner from the surviving three gauges
    via a Segers-style modal decomposition (thesis_notes.md "FR
    reconstruction: quasi-static modal decomposition" -- pointer only,
    full method there), restricted to the ONE mode that is exactly
    observable with three real corners and no roll/ARB model at all:

    HEAVE+PITCH (axle total): fz_axle_totals["fz_f_N"]/["fz_r_N"] --
    modules.stability_analysis.estimate_vertical_loads's own weight +
    longitudinal-transfer + aero-share axle total, which does not depend
    on ay/roll at all (roll only redistributes load WITHIN an axle, never
    changes the axle's own total). Left+right on that axle must sum to
    this total regardless of how roll splits them -- so when exactly one
    corner of an axle is invalid and its axle-mate IS damper-valid, the
    missing corner is exactly axle_total - measured_mate, with NO roll/
    ARB model needed for the split at all (the real mate measurement
    already contains whatever the true roll split was).

    This is DELIBERATELY not "axle total x modelled roll-balance
    fraction" (a plausible alternative reading of "left/right from roll
    balance") -- that would throw away the real axle-mate measurement in
    favour of the same approximate ARB/lateral-transfer model already
    used for the full static fallback, which is strictly less accurate
    once a real measurement on that axle exists.

    LIMITATIONS (recorded here and in thesis_notes.md, not glossed over):
    - Single-wheel events on the RECONSTRUCTED corner are INVISIBLE: the
      axle-total model is a heave/pitch/aero estimate with no knowledge
      of a road input (bump, kerb) unique to one wheel. If the true axle
      total spikes because the missing wheel alone hit something, the
      model does not see that spike, and the reconstruction silently
      absorbs the whole model/reality gap into an UNDER-reaction at the
      reconstructed corner while leaving the measured mate untouched.
    - WARP (the fourth modal DOF, (FL+RR)-(FR+RL), chassis torsion or an
      unevenly-loaded three-wheel condition) is UNOBSERVABLE with three
      sensors and a heave/pitch-only axle-total model -- this
      reconstruction has no mechanism to represent it, by construction.
    - Inherits every Level-1 placeholder already inside fz_axle_totals
      (cog_height_m, aero lift_coeff/cross_track_area_m2/diff_cog_x_m) --
      accuracy_levels.wheel_load_damper_reconstructed is registered at
      Level 1 for exactly this reason (chained-constant), even though it
      is built from three real sensors, since the accuracy-level system
      records provenance TIER, not within-tier confidence.

    Only applies where EXACTLY ONE of an axle's two corners is invalid
    (per sample) -- if both are invalid, no reconstruction is possible
    from this method and the caller must fall back further (static
    split). Returns a dict per corner: {"fz_N": reconstructed value or
    NaN, "reconstructable": bool array marking where this method could
    apply}.
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
    """Three-tier per-corner, per-sample cascade: damper-measured (Level
    4) where valid; else RECONSTRUCTED from the axle-mate + axle-total
    model (accuracy_levels.wheel_load_damper_reconstructed, Level 1 --
    see reconstruct_missing_corner's own docstring for why a real-sensor-
    informed method still sits at Level 1) where exactly one corner of
    that axle is invalid and its mate is damper-valid; else the plain
    static-split estimate (per_wheel_load_split, Level 1). "source" is a
    per-sample string array so a caller/plot never has to re-derive which
    tier produced a given value.
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
