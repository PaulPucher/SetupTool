# Per-session accuracy levels: registry (parameters.json accuracy_levels)
# + outing setup_data + optional cap (int 1-4, None = best available)
# -> values the pipeline uses and the level map the UI shows.
#
# Resolved live: mass, corner_weights, steering_ratio; cog_position and
# steering_angle cascade from those. Everything else stays at its registry
# level -- no alternative value to switch to.
# yaw_inertia not cascaded: method ceiling 1 (m*a*b) -> always level 1.
#
# steering_ratio L4 = manufacturer table in car_data.json (local,
# gitignored); file missing or table bad -> L1 constant.

import copy

import numpy as np

from modules.stability_analysis import load_car_data

MASS_CORNER_SUM_TOLERANCE = 0.01  # relative fraction; calibration tunable


def _cap_ceiling(cap):
    return cap if cap is not None else 4


def _load_steering_ratio_table():
    """(angle_deg, ratio) as plain lists (goes into JSON-serialised "values"),
    or None if missing, malformed, or angle axis not strictly increasing
    (np.interp needs that).
    """
    car_data = load_car_data()
    if not car_data:
        return None
    table = car_data.get("steering_ratio_table")
    if not table:
        return None
    columns = table.get("columns")
    rows = table.get("rows")
    if not columns or not rows:
        return None
    try:
        angle_idx = columns.index("steering_wheel_angle_deg")
        ratio_idx = columns.index("steering_ratio")
        angle_deg = [float(row[angle_idx]) for row in rows]
        ratio = [float(row[ratio_idx]) for row in rows]
    except (ValueError, TypeError, IndexError):
        return None
    if len(angle_deg) < 2:
        return None
    if any(b <= a for a, b in zip(angle_deg, angle_deg[1:])):
        return None
    return angle_deg, ratio


def _resolve_steering_ratio(params, cap):
    ceiling = _cap_ceiling(cap)
    config_value = params["vehicle"]["steering_ratio"]
    table = _load_steering_ratio_table()
    best_available_level = 4 if table is not None else 1

    if table is not None and 4 <= ceiling:
        angle_deg, ratio = table
        return {
            "level": 4,
            "value": {
                "mode": "table",
                "table_angle_deg": angle_deg,
                "table_ratio": ratio,
                "constant": config_value,
            },
            "source": "car_data.json steering_ratio_table (WP-B, manufacturer digitised)",
            "best_available_level": best_available_level,
        }

    return {
        "level": 1,
        "value": {
            "mode": "constant",
            "table_angle_deg": None,
            "table_ratio": None,
            "constant": config_value,
        },
        "source": "config default (vehicle.steering_ratio)",
        "best_available_level": best_available_level,
    }


def _resolve_steering_angle(steering_ratio_resolved):
    # pure cascade, no approximation on top
    return {
        "level": steering_ratio_resolved["level"],
        "source": f"derived from steering_ratio ({steering_ratio_resolved['source']})",
        "best_available_level": steering_ratio_resolved["best_available_level"],
    }


def _resolve_corner_weights(params, setup_data, cap):
    config_value = params["vehicle"]["corner_weights"]
    ceiling = _cap_ceiling(cap)

    session_car = (setup_data or {}).get("car", {}) or {}
    keys = ("corner_weight_fl", "corner_weight_fr", "corner_weight_rl", "corner_weight_rr")
    raw = [session_car.get(k) for k in keys]
    # 0.0 = not entered (no real corner weighs 0 kg). All four or none --
    # no mixing measured and default corners.
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
    # 0.0 = not entered
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

    # explicit total_weight beats corner sum; never averaged
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
    # Cascade from corner_weights (cap already applied there).
    # L1 reads the stored a/b constants rather than recomputing -- config
    # rounds them to 3 dp, recomputing would drift the baselines.
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
    """Resolve mass, corner_weights, cog_position, steering_ratio and
    steering_angle for this session; other nodes keep their registry level.

    Returns {"levels", "values", "clipped", "warnings"}. "values" is
    JSON-serialisable (goes into the cache identity check).
    clipped = cap actually lowered a node below its best available level.
    """
    registry = params["accuracy_levels"]
    vehicle = params["vehicle"]
    aero = vehicle["aero"]

    corner_weights = _resolve_corner_weights(params, setup_data, cap)
    mass, mass_warnings = _resolve_mass(params, setup_data, cap, corner_weights)
    cog_position = _resolve_cog_position(params, corner_weights)
    steering_ratio = _resolve_steering_ratio(params, cap)
    steering_angle = _resolve_steering_angle(steering_ratio)

    clipped = (
        mass["level"] < mass["best_available_level"]
        or corner_weights["level"] < corner_weights["best_available_level"]
        or cog_position["level"] < cog_position["best_available_level"]
        or steering_ratio["level"] < steering_ratio["best_available_level"]
    )

    levels = {node: entry["level"] for node, entry in registry.items() if node != "_comment"}
    levels["mass"] = mass["level"]
    levels["corner_weights"] = corner_weights["level"]
    levels["cog_position"] = cog_position["level"]
    levels["steering_ratio"] = steering_ratio["level"]
    levels["steering_angle"] = steering_angle["level"]

    values = {
        "mass_kg": mass["value"],
        "corner_weights": corner_weights["value"],
        "cog_to_front_axle_m": cog_position["value"]["cog_to_front_axle_m"],
        "cog_to_rear_axle_m": cog_position["value"]["cog_to_rear_axle_m"],
        "steering_ratio": steering_ratio["value"],
        # passthrough for cache identity only -- a settings edit must
        # invalidate the cache; apply_resolved_vehicle ignores these
        "cog_height_m": vehicle["cog_height_m"],
        "track_width_front_m": vehicle["track_width_front_m"],
        "track_width_rear_m": vehicle["track_width_rear_m"],
        "wheelbase_m": vehicle["wheelbase_m"],
        "yaw_inertia_kgm2": vehicle["yaw_inertia_kgm2"],
        "aero_air_density_kgm3": aero["air_density_kgm3"],
        "aero_lift_coeff": aero["lift_coeff"],
        "aero_cross_track_area_m2": aero["cross_track_area_m2"],
        "aero_diff_cog_x_m": aero["diff_cog_x_m"],
    }

    return {
        "levels": levels,
        "values": values,
        "clipped": clipped,
        "warnings": mass_warnings,
    }


def apply_resolved_vehicle(params, resolved):
    """Deep copy of params with the resolved vehicle values written in."""
    effective = copy.deepcopy(params)
    effective["vehicle"]["mass_kg"] = resolved["values"]["mass_kg"]
    effective["vehicle"]["corner_weights"] = dict(resolved["values"]["corner_weights"])
    effective["vehicle"]["cog_to_front_axle_m"] = resolved["values"]["cog_to_front_axle_m"]
    effective["vehicle"]["cog_to_rear_axle_m"] = resolved["values"]["cog_to_rear_axle_m"]

    steering_ratio_value = resolved["values"].get("steering_ratio")
    if steering_ratio_value and steering_ratio_value.get("mode") == "table":
        # L4 only; absent -> prepare_vehicle_state uses the scalar
        effective["vehicle"]["steering_ratio_table"] = {
            "angle_deg": np.array(steering_ratio_value["table_angle_deg"], dtype=float),
            "ratio": np.array(steering_ratio_value["table_ratio"], dtype=float),
        }
    return effective
