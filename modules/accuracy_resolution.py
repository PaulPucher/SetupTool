# Per-session accuracy-level resolution (WP-C).
#
# Resolves the WP-A static registry (config/parameters.json accuracy_levels)
# against per-outing setup_data and an optional global cap, producing both
# the values Modules 1-5 actually consume and the level map the UI/cache
# layers report. Pure Python, no Qt -- the cap crosses the UI/modules
# boundary as a plain int (1-4) or None ("best available"), the same shape
# lap_filter already uses (ui/views/outing_form.py).
#
# Two dynamically-wired leaf nodes today: mass, corner_weights. cog_position
# is a pure cascade from corner_weights (no source list of its own). Every
# other registry node (yaw_inertia, steering_ratio, lateral_force_split,
# sideslip_angle, speed, yaw_rate, steering_angle, lateral_acc, wheelbase_m)
# stays single-source at its registry-declared level regardless of cap --
# there is no alternate value to fall back to yet, so capping its label
# without capping its computation would misrepresent what was actually
# used. yaw_inertia and lateral_force_split are chain-limited by mass_kg/
# corner_weights per the registry's capped_by field, but are not cascaded
# dynamically here: yaw_inertia's m*a*b estimate carries its own
# method_ceiling (1) regardless of how well its inputs are known, so
# min(method_ceiling=1, mass_level, cog_position_level) always equals 1
# under today's registry -- wiring the cascade would be a no-op until the
# ceiling itself changes (a different Iz measurement method, not a better-
# measured mass or cog position). lateral_force_split inherits that same
# ceiling transitively through yaw_inertia, for the same reason.

import copy

MASS_CORNER_SUM_TOLERANCE = 0.01  # relative fraction; calibration tunable


def _cap_ceiling(cap):
    return cap if cap is not None else 4


def _resolve_corner_weights(params, setup_data, cap):
    config_value = params["vehicle"]["corner_weights"]
    ceiling = _cap_ceiling(cap)

    session_car = (setup_data or {}).get("car", {}) or {}
    keys = ("corner_weight_fl", "corner_weight_fr", "corner_weight_rl", "corner_weight_rr")
    raw = [session_car.get(k) for k in keys]
    # Zero-sentinel availability check, scoped to this field specifically: a
    # real car's corner load can never be 0 kg, so 0.0 (the setup_data JSON
    # blob's unfilled default) is a safe "not entered" proxy here. All four
    # must be present -- a partial fill would silently mix a measured corner
    # with a defaulted one in the same front/rear fraction split.
    session_available = all(v is not None and v != 0.0 for v in raw)
    best_available_level = 2 if session_available else 1

    if session_available and 2 <= ceiling:
        value = {"FL_kg": raw[0], "FR_kg": raw[1], "RL_kg": raw[2], "RR_kg": raw[3]}
        return {
            "level": 2,
            "value": value,
            "source": "session measurement (Outing.setup_data.car.corner_weight_fl/fr/rl/rr)",
            "best_available_level": best_available_level,
        }

    return {
        "level": 1,
        "value": {k: config_value[k] for k in ("FL_kg", "FR_kg", "RL_kg", "RR_kg")},
        "source": "config default (vehicle.corner_weights)",
        "best_available_level": best_available_level,
    }


def _resolve_mass(params, setup_data, cap, corner_weights_resolved):
    ceiling = _cap_ceiling(cap)
    config_value = params["vehicle"]["mass_kg"]

    session_car = (setup_data or {}).get("car", {}) or {}
    total_raw = session_car.get("total_weight")
    # Same zero-sentinel scoping as corner_weights: total mass can never be
    # 0 kg for a real car.
    explicit_available = total_raw is not None and total_raw != 0.0

    derived_available = corner_weights_resolved["level"] == 2
    derived_value = None
    if derived_available:
        cw = corner_weights_resolved["value"]
        derived_value = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]

    warnings = []
    if explicit_available and derived_available and derived_value:
        rel_diff = abs(total_raw - derived_value) / derived_value
        if rel_diff > MASS_CORNER_SUM_TOLERANCE:
            warnings.append(
                f"session mass inconsistent: total {total_raw:.1f} vs corner sum {derived_value:.1f}"
            )

    best_available_level = 2 if (explicit_available or derived_available) else 1

    # Priority when multiple L2 sources are available at once: explicit
    # setup_data.total_weight wins over the derived corner-weight sum --
    # never blended/averaged, per the standing "highest available wins"
    # rule. The consistency warning above fires independently of which one
    # is actually used.
    if explicit_available and 2 <= ceiling:
        return (
            {
                "level": 2,
                "value": total_raw,
                "source": "session measurement (Outing.setup_data.car.total_weight)",
                "best_available_level": best_available_level,
            },
            warnings,
        )
    if derived_available and 2 <= ceiling:
        return (
            {
                "level": 2,
                "value": derived_value,
                "source": "sum(corner_weights)",
                "best_available_level": best_available_level,
            },
            warnings,
        )
    return (
        {
            "level": 1,
            "value": config_value,
            "source": "config default (vehicle.mass_kg)",
            "best_available_level": best_available_level,
        },
        warnings,
    )


def _resolve_cog_position(params, corner_weights_resolved):
    # Pure cascade -- no cap check of its own, since corner_weights_resolved
    # already reflects whatever cap was applied. At Level 1 this reads the
    # stored config constants directly rather than recomputing a = L*rear_
    # fraction from config's own corner weights: the two are numerically
    # equal in principle (the constants were derived that way, per vehicle.
    # cog_note), but config stores them rounded to 3 decimals, so recomputing
    # would introduce a sub-millimetre floating-point drift against every
    # existing byte-identical baseline. Only Level 2 (real session corner
    # weights, no precomputed constant to fall back on) actually recomputes.
    if corner_weights_resolved["level"] == 1:
        value = {
            "cog_to_front_axle_m": params["vehicle"]["cog_to_front_axle_m"],
            "cog_to_rear_axle_m": params["vehicle"]["cog_to_rear_axle_m"],
        }
        source = "config default (vehicle.cog_to_front_axle_m/cog_to_rear_axle_m)"
    else:
        cw = corner_weights_resolved["value"]
        wheelbase = params["vehicle"]["wheelbase_m"]
        w_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
        rear_fraction = (cw["RL_kg"] + cw["RR_kg"]) / w_total
        front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / w_total
        value = {
            "cog_to_front_axle_m": wheelbase * rear_fraction,
            "cog_to_rear_axle_m": wheelbase * front_fraction,
        }
        source = f"derived from corner_weights ({corner_weights_resolved['source']})"

    return {
        "level": corner_weights_resolved["level"],
        "value": value,
        "source": source,
        "best_available_level": corner_weights_resolved["best_available_level"],
    }


def resolve_accuracy(params, setup_data=None, cap=None):
    """Resolve per-session accuracy for the dynamically-wired registry
    nodes (mass, corner_weights, cog_position) against setup_data and an
    optional global cap (int 1-4, or None for "best available" -- no
    ceiling). Every other registry node mirrors its static declared level
    unchanged.

    Returns {"levels": {node: level}, "values": {mass_kg, corner_weights,
    cog_to_front_axle_m, cog_to_rear_axle_m}, "clipped": bool, "warnings":
    [str, ...]}. "clipped" is true iff the cap actually lowered a
    dynamically-resolved node below its own best-available level for this
    setup_data -- selecting a cap that happens not to bind on today's data
    must not read as a comparison run.
    """
    registry = params["accuracy_levels"]

    corner_weights = _resolve_corner_weights(params, setup_data, cap)
    mass, mass_warnings = _resolve_mass(params, setup_data, cap, corner_weights)
    cog_position = _resolve_cog_position(params, corner_weights)

    clipped = (
        mass["level"] < mass["best_available_level"]
        or corner_weights["level"] < corner_weights["best_available_level"]
        or cog_position["level"] < cog_position["best_available_level"]
    )

    levels = {node: entry["level"] for node, entry in registry.items() if node != "_comment"}
    levels["mass"] = mass["level"]
    levels["corner_weights"] = corner_weights["level"]
    levels["cog_position"] = cog_position["level"]

    values = {
        "mass_kg": mass["value"],
        "corner_weights": corner_weights["value"],
        "cog_to_front_axle_m": cog_position["value"]["cog_to_front_axle_m"],
        "cog_to_rear_axle_m": cog_position["value"]["cog_to_rear_axle_m"],
    }

    return {
        "levels": levels,
        "values": values,
        "clipped": clipped,
        "warnings": mass_warnings,
    }


def apply_resolved_vehicle(params, resolved):
    """Return a deep-copied params dict with vehicle.mass_kg/corner_weights/
    cog_to_front_axle_m/cog_to_rear_axle_m overridden by resolved["values"]
    (resolve_accuracy's output). prepare_vehicle_state and
    estimate_lateral_forces read these same params["vehicle"] keys exactly
    as before -- this is the only call-site change needed to wire
    per-session resolution through the existing pipeline; neither
    function's own body changes.
    """
    effective = copy.deepcopy(params)
    effective["vehicle"]["mass_kg"] = resolved["values"]["mass_kg"]
    effective["vehicle"]["corner_weights"] = dict(resolved["values"]["corner_weights"])
    effective["vehicle"]["cog_to_front_axle_m"] = resolved["values"]["cog_to_front_axle_m"]
    effective["vehicle"]["cog_to_rear_axle_m"] = resolved["values"]["cog_to_rear_axle_m"]
    return effective
