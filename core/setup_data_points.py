# Splitter/diffuser measurement-point reshaping, outside outing_form.py so
# it's testable without Qt.
# Stored: car["splitter_points"] / car["diffuser_points"], 5-element
# arrays (index 0..4 = point 1..5, blank -> null). Widgets use flat
# splitter_point_1.._5 / diffuser_point_1.._5 keys.

import json

POINT_GROUPS = [("splitter_point", "splitter_points", 5), ("diffuser_point", "diffuser_points", 5)]

# Point positions, shared by measurement_points_widget and pdf_export.
# fx, fy in [0, 1] within each shape's bounding box; fy = 0 = front.
# Extracted from the reference image, then symmetrised about x = 0.5
# (mirror pairs averaged, centre points snapped) -- the car is symmetric,
# the hand-placed dots weren't.
#
# Splitter: point 5 = front-middle offset reference (fy = 0); 1-4 numbered
# left to right by x. Mirror pairs 1&4, 2&3.
SPLITTER_POINT_POSITIONS = [(0.045, 0.77), (0.145, 0.255), (0.855, 0.255), (0.955, 0.77), (0.5, 0.0)]

# Diffuser: reading order, upper pair then lower three (no rule given).
# Mirror pairs 1&2, 3&5; 4 = centre.
DIFFUSER_POINT_POSITIONS = [(0.015, 0.375), (0.985, 0.375), (0.03, 0.955), (0.5, 0.95), (0.97, 0.955)]


def reshape_points_out(json_string):
    data = json.loads(json_string)
    car = data.get("car")
    if isinstance(car, dict):
        for prefix, array_key, n in POINT_GROUPS:
            if not any(f"{prefix}_{i}" in car for i in range(1, n + 1)):
                continue
            points = []
            for i in range(1, n + 1):
                raw = car.pop(f"{prefix}_{i}", None)
                if raw in (None, ""):
                    points.append(None)
                else:
                    try:
                        points.append(float(raw))
                    except (ValueError, TypeError):
                        points.append(None)
            car[array_key] = points
    return json.dumps(data)


def reshape_points_in(json_string):
    if not json_string:
        return json_string
    try:
        data = json.loads(json_string)
    except (json.JSONDecodeError, TypeError):
        return json_string
    car = data.get("car")
    if isinstance(car, dict):
        for prefix, array_key, n in POINT_GROUPS:
            points = car.pop(array_key, None)
            if isinstance(points, list):
                for i in range(1, n + 1):
                    value = points[i - 1] if i - 1 < len(points) else None
                    car[f"{prefix}_{i}"] = "" if value is None else str(value)
    return json.dumps(data)
