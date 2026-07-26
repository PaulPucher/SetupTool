# Per-session accuracy-level resolution (WP-C).
#
# Resolves the WP-A static registry (config/parameters.json accuracy_levels)
# against per-outing setup_data and an optional global cap, producing both
# the values Modules 1-5 actually consume and the level map the UI/cache
# layers report. Pure Python, no Qt -- the cap crosses the UI/modules
# boundary as a plain int (1-4) or None ("best available"), the same shape
# lap_filter already uses (ui/views/outing_form.py).
#
# Three dynamically-wired leaf nodes today: mass, corner_weights,
# steering_ratio. cog_position and steering_angle are pure cascades (no
# source list of their own -- cog_position from corner_weights,
# steering_angle from steering_ratio). Every other registry node
# (yaw_inertia, lateral_force_split, sideslip_angle, speed, yaw_rate,
# lateral_acc, wheelbase_m) stays single-source at its registry-declared
# level regardless of cap -- there is no alternate value to fall back to
# yet, so capping its label without capping its computation would
# misrepresent what was actually used. yaw_inertia and lateral_force_split
# are chain-limited by mass_kg/corner_weights per the registry's capped_by
# field, but are not cascaded dynamically here: yaw_inertia's m*a*b
# estimate carries its own method_ceiling (1) regardless of how well its
# inputs are known, so min(method_ceiling=1, mass_level, cog_position_level)
# always equals 1 under today's registry -- wiring the cascade would be a
# no-op until the ceiling itself changes (a different Iz measurement
# method, not a better-measured mass or cog position). lateral_force_split
# inherits that same ceiling transitively through yaw_inertia, for the
# same reason.
#
# steering_ratio (WP-B) is a genuine parameterization upgrade, not a
# deviation from any chair scientific position -- the 15.7 constant was
# never a chair-adopted method, it is this car's own mechanical steering
# geometry, digitised from a manufacturer table (config/car_data.json
# steering_ratio_table) at Level 4. Unlike mass/corner_weights, this node
# has no per-outing setup_data involvement at all: availability depends
# only on whether the local, gitignored car_data.json file exists and its
# table parses and is monotonic, never on anything about the current
# outing -- so a run on a machine without that file falls back to the
# Level 1 constant transparently, by construction, not by special-casing.

import copy

import numpy as np

from modules.stability_analysis import load_car_data

MASS_CORNER_SUM_TOLERANCE = 0.01  # relative fraction; calibration tunable


def _cap_ceiling(cap):
    return cap if cap is not None else 4


def _load_steering_ratio_table():
    """Load and validate car_data.json's steering_ratio_table. Returns
    (angle_deg, ratio) as plain Python lists (not numpy arrays -- this
    result flows into resolve_accuracy's JSON-serialised "values", which
    the WP5 cache payload and WP6 identity check both need to compare/
    persist directly), or None if the file is absent, malformed, or the
    table's lookup axis (steering_wheel_angle_deg) isn't strictly
    increasing -- np.interp requires that precondition, and a table
    failing it is not safely usable regardless of why it failed.
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
    # Pure cascade -- delta_f_rad's accuracy is exactly steering_ratio's
    # own, nothing else approximates on top of the conversion (unlike
    # yaw_inertia's method ceiling).
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
    nodes (mass, corner_weights, cog_position, steering_ratio,
    steering_angle) against setup_data and an optional global cap (int
    1-4, or None for "best available" -- no ceiling). Every other
    registry node mirrors its static declared level unchanged.

    Returns {"levels": {node: level}, "values": {mass_kg, corner_weights,
    cog_to_front_axle_m, cog_to_rear_axle_m, steering_ratio, plus the
    static section-1 physics constants below}, "clipped": bool,
    "warnings": [str, ...]}. "values" is JSON-serialisable (plain
    floats/lists/dicts, no numpy arrays) since it flows directly into the
    WP5 cache payload and the WP6 identity check. "clipped" is true iff
    the cap actually lowered a dynamically-resolved node below its own
    best-available level -- selecting a cap that happens not to bind on
    today's data (or today's car_data.json availability) must not read
    as a comparison run.

    PART B amendment (2026-07-27): "values" also carries cog_height_m,
    track_width_front_m, track_width_rear_m, wheelbase_m,
    yaw_inertia_kgm2, and the four aero.* fields -- straight passthrough
    from params["vehicle"], no per-session resolution logic of their own
    (unlike the five dynamically-wired fields above). They exist in this
    dict SOLELY so the WP5/WP6 cache identity checks (both compare this
    whole dict for equality) notice a settings-window edit to any of
    them -- apply_resolved_vehicle below never reads these keys, so
    adding them cannot change what Modules 1-5 compute, only whether a
    cached result is judged reusable. A settings save that changes one of
    these values makes this dict compare unequal to any previously
    cached/persisted snapshot; an OLD snapshot recorded before this
    amendment simply lacks these keys entirely, which is already unequal
    to a dict that has them -- no ANALYSIS_SCHEMA_VERSION bump needed,
    same "no cache" fallback Guard B already provides for a schema
    mismatch.
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
        # PART B amendment: static passthrough, cache-identity only (see
        # docstring above) -- never read by apply_resolved_vehicle.
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
    """Return a deep-copied params dict with vehicle.mass_kg/corner_weights/
    cog_to_front_axle_m/cog_to_rear_axle_m/steering_ratio overridden by
    resolved["values"] (resolve_accuracy's output). prepare_vehicle_state
    and estimate_lateral_forces read these same params["vehicle"] keys
    (plus the new optional steering_ratio_table) exactly as documented at
    each call site -- this is the only call-site change needed to wire
    per-session resolution through the existing pipeline.
    """
    effective = copy.deepcopy(params)
    effective["vehicle"]["mass_kg"] = resolved["values"]["mass_kg"]
    effective["vehicle"]["corner_weights"] = dict(resolved["values"]["corner_weights"])
    effective["vehicle"]["cog_to_front_axle_m"] = resolved["values"]["cog_to_front_axle_m"]
    effective["vehicle"]["cog_to_rear_axle_m"] = resolved["values"]["cog_to_rear_axle_m"]

    steering_ratio_value = resolved["values"].get("steering_ratio")
    if steering_ratio_value and steering_ratio_value.get("mode") == "table":
        # Injected only at Level 4 -- prepare_vehicle_state checks for this
        # key's presence (vp.get("steering_ratio_table")) and falls back to
        # the plain vehicle.steering_ratio scalar (already correct via the
        # deepcopy above) when it's absent, exactly as a raw, un-resolved
        # params dict already does today.
        effective["vehicle"]["steering_ratio_table"] = {
            "angle_deg": np.array(steering_ratio_value["table_angle_deg"], dtype=float),
            "ratio": np.array(steering_ratio_value["table_ratio"], dtype=float),
        }
    return effective
