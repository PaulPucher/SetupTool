# Current setup value and value-space logic for the setup engine: where a
# lever stands on the setup sheet, whether a step stays legal, and how close
# it sits to its typical window. Moved unchanged from
# modules/recommendation.py and modules/decision_frame.py (overhaul Part 1,
# 2026-10-22); windows now live in config/setup_windows.json.

import json

import numpy as np

SETUP_PARAMETERS_CONFIG_PATH = "config/setup_parameters.json"
SETUP_WINDOWS_CONFIG_PATH = "config/setup_windows.json"


def load_setup_parameters_registry():
    with open(SETUP_PARAMETERS_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {k: v for k, v in data.items() if not k.startswith("_comment")}


def load_setup_windows():
    with open(SETUP_WINDOWS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


# levers the setup sheet has no field for but a logged channel records; the
# ABS map is read from the dash / steering-wheel position channel the
# registry trusts (config/channels.json log_abs_pos, decision 2026-09-18)
LOGGED_SETUP_CHANNELS = {"abs_position": "log_abs_pos"}


def logged_setup_values(channels, laps=None, valid_laps=None):
    """{lever: {"value", "source", "note"}} from logged switch channels.

    The value is the session mode: a map position is a discrete switch, so
    the most frequent position is the one the session ran on. When the
    position changes within the session the note names the position on the
    last valid lap, because that is the state the next change starts from."""
    out = {}
    for key, name in LOGGED_SETUP_CHANNELS.items():
        ch = (channels or {}).get(name)
        if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("data") is None:
            continue
        d = np.asarray(ch["data"], float)
        fin = np.isfinite(d)
        if not fin.any():
            continue
        vals, counts = np.unique(np.round(d[fin]).astype(int), return_counts=True)
        entry = {"value": int(vals[np.argmax(counts)]), "source": name, "note": None}
        if len(vals) > 1:
            entry["note"] = f"map changed in session (last valid lap: {_last_valid_lap_value(ch, laps, valid_laps)})"
        out[key] = entry
    return out


def _last_valid_lap_value(ch, laps, valid_laps):
    # mode inside the last valid lap's time window; "unknown" without laps
    cand = [l for l in laps or [] if valid_laps is None or l["lap_number"] in valid_laps]
    if not cand or ch.get("time") is None:
        return "unknown"
    lap = max(cand, key=lambda l: l["lap_number"])
    t, d = np.asarray(ch["time"]), np.asarray(ch["data"], float)
    m = (t >= lap["start_time"]) & (t <= lap["end_time"]) & np.isfinite(d)
    if not m.any():
        return "unknown"
    vals, counts = np.unique(np.round(d[m]).astype(int), return_counts=True)
    return int(vals[np.argmax(counts)])


def _numeric_bounds(entry):
    # ride_height_*: bounds from standard + typical_window; None = no edge
    # on that side
    vs = entry["value_space"]
    if vs is None:
        return None, None
    # a lever whose scale is only monotonic on part of its range (TC: 2-11)
    # can be stepped only inside that part
    if "feasible_min" in vs and "feasible_max" in vs:
        return vs["feasible_min"], vs["feasible_max"]
    if "min" in vs and "max" in vs:
        return vs["min"], vs["max"]
    tw = entry.get("typical_window") or {}
    standard = vs.get("standard")
    if standard is None:
        return None, None
    if "max_delta_from_standard" in tw:
        # front ride height: a floor only (standard - 2 mm, diffuser floor
        # limit); the car is often run at the minimum but nothing caps it
        # above the standard (author ruling 2026-10-23, 4g item 3)
        return standard + tw["max_delta_from_standard"], None
    if "delta_from_standard" in tw:
        lo, hi = tw["delta_from_standard"]
        return standard + lo, standard + hi
    return None, None


def _check_feasible(entry, current_value, delta_value):
    # True/False, or None = no bounds known (not the same as unknown current value)
    vs = entry["value_space"]
    if vs and vs.get("type") == "enum" and "options" in vs:
        options = vs["options"]
        if current_value not in options:
            return None
        idx = options.index(current_value) + int(delta_value)
        return 0 <= idx < len(options)
    lo, hi = _numeric_bounds(entry)
    if lo is None and hi is None:
        return None
    try:
        new_value = float(current_value) + delta_value
    except (TypeError, ValueError):
        return None
    return (lo is None or new_value >= lo) and (hi is None or new_value <= hi)


def _current_setup_value(setup_data, entry):
    # current value known = present and nonzero. For most keys 0 is out of
    # range (spinbox default); damper clicks can be 0 but a real 0 can't be
    # told from the default -> 0 = unknown everywhere.
    maps_to = entry.get("maps_to")
    if not maps_to or not setup_data:
        return None
    parts = maps_to[0].split(".")[1:]  # drop "setup_parameters"
    node = setup_data
    for p in parts:
        if not isinstance(node, dict) or p not in node:
            return None
        node = node[p]
    if node is None or node == "":
        return None
    if isinstance(node, (int, float)) and node == 0:
        return None
    return node


# Window edge: HARD = registry value_space min/max (physical); SOFT =
# typical window nominal +/- span ("engineer may exceed"). Directional: only
# a step that pushes further past the edge counts. Missing value for this
# lever -> not assessable (per lever).
def _window_edge_check(action, setup_data, registry, config):
    param = action["parameter"]
    delta = action.get("delta")
    if delta is None or delta == 0:
        return None  # target action, no push
    entry = registry.get(param)
    if entry is None:
        return None
    if not entry.get("maps_to"):
        # the sheet has no field for this lever (TC, ABS, brake bias): no
        # edge check; ABS may carry a logged position, but its map is
        # categorical, so there is no window to sit at the edge of
        return None
    current = _current_setup_value(setup_data, entry)
    if current is None:
        return ("not_assessable", f"setup sheet unfilled: {param}")
    try:
        current = float(current)
    except (TypeError, ValueError):
        return None  # enum label, not numeric

    push_sign = 1 if delta > 0 else -1

    value_space = entry.get("value_space") or {}
    vmin, vmax = value_space.get("min"), value_space.get("max")
    if push_sign > 0 and vmax is not None and current >= vmax:
        return ("hard", f"{param} already at its hard maximum ({current} >= {vmax})")
    if push_sign < 0 and vmin is not None and current <= vmin:
        return ("hard", f"{param} already at its hard minimum ({current} <= {vmin})")

    window = config["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or span is None or not span:
        return None
    distance = current - nominal
    if (push_sign > 0 and distance >= span) or (push_sign < 0 and distance <= -span):
        return ("soft", f"{param} already at its typical-window edge "
                         f"(current={current}, nominal={nominal}, span={span})")
    return None


def _edge_normal_state_label(action, config):
    # an edge that is the lever's normal resting state is annotated, not blocked
    for entry in config.get("edge_normal_state_params", []):
        if entry["parameter"] == action["parameter"] and entry["direction"] == action.get("direction"):
            return entry["label"]
    return None


def window_headroom(param, current, delta, registry, config):
    # 1 - |new - nominal| / span, clipped to [0, 1]: how far the step keeps
    # the lever inside its typical window. None when no window or no numeric
    # value (same neutral case as the retired scoring's window term).
    window = config["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or not span or current is None:
        return None
    try:
        new_value = float(current) + delta
    except (TypeError, ValueError):
        return None
    return 1.0 - min(1.0, abs(new_value - nominal) / span)
