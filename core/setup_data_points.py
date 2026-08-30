# Splitter/diffuser measurement-point reshape helpers, extracted out of
# ui/views/outing_form.py so they're testable without importing a PyQt6
# module -- same reason tests/conftest.py's pipeline_result fixture keeps
# outing_form.py itself out of the regression suite. Pure JSON-string
# transforms, no Qt, no behaviour change from the methods they replaced
# (thesis_notes.md "8. Splitter/diffuser measurement points").
#
# Storage shape: car["splitter_points"] / car["diffuser_points"], each a
# plain 5-element array, index 0..4 = point 1..5, missing/blank -> null.
# Widgets bind to flat splitter_point_1.._5 / diffuser_point_1.._5 keys
# (ui/views/measurement_points_widget.py); these functions fold between
# the two shapes, same pop-based mechanism as the pre-existing diff-
# torque reshape (_reshape_diff_torque_out/_in, which stayed in
# outing_form.py -- it has only the one call site pair and no test
# needed it moved).

import json

POINT_GROUPS = [("splitter_point", "splitter_points", 5), ("diffuser_point", "diffuser_points", 5)]

# Physical positions, single source for both consumers (ui/views/
# measurement_points_widget.py and core/pdf_export.py both import these --
# eliminates the hand-kept-in-sync duplication the Phase 4 contract test
# flagged). fx, fy in [0, 1] x [0, 1] within each shape's own bounding box;
# fy=0 is the front of the car (top of the drawn shape).
#
# Extracted programmatically (connected-component clustering of pure-green
# marker pixels, user-annotated screenshot, 2026-08-30) from the user's own
# reference image, then SYMMETRIZED across the vertical centreline (x=0.5)
# -- the dots were hand-placed and the car is left/right symmetric, so raw
# mirror-pair coordinates carried hand jitter the underlying geometry does
# not have. Mirror pairs: each pair's x became (x_left, 1-x_left) with
# x_left = (x_left_raw + (1 - x_right_raw)) / 2, and both points took the
# pair's mean y. Centre points snapped to x=0.5 exactly, y unchanged. Raw
# extracted values and the pixel-space bounding boxes/centroids this was
# computed from are recorded in thesis_notes.md "8. Splitter/diffuser
# measurement points, position re-extraction" and "...symmetrized". These
# are now plain literals -- no runtime symmetrization logic, this was a
# one-time cleanup of the extracted constants.
#
# Splitter: point 5 is the front-middle offset-check reference (sits right
# at the leading-edge boundary, fy=0, x snapped to 0.5); points 1-4
# numbered strictly by x (1=leftmost .. 4=rightmost), regardless of their
# differing fy -- the reference sheet's own points are not a single
# straight row. Mirror pairs: 1&4, 2&3.
SPLITTER_POINT_POSITIONS = [(0.045, 0.77), (0.145, 0.255), (0.855, 0.255), (0.955, 0.77), (0.5, 0.0)]

# Diffuser: no explicit numbering rule was given for this shape (unlike
# splitter's "1-4 left to right") -- numbered here in natural reading
# order, upper pair first then the lower row of three, left to right
# within each row. Flagged for the user's confirmation alongside the
# positions themselves. Mirror pairs: 1&2, 3&5; point 4 is the centre.
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
