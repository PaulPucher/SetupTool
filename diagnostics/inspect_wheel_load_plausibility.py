# Census behind wheel_loads.plausibility_min_static_fraction (work order
# DEV-4 item 3, author ruling 2026-10-24): per session the straight-line
# damper-path total over the static weight, with the production function
# (wheel_loads.straight_line_plausibility, same straight-line mask and mate
# proxy as the session fit). 14 development sessions at best available
# (setup sheet None, as the threshold derivation) and the 8 labelled
# outings with their stored setup sheets (as the app resolves them).
# Output diagnostics/results_wheel_load_plausibility_20261024/.
# Run: python -m diagnostics.inspect_wheel_load_plausibility

import json
import os
import sqlite3
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from diagnostics.inspect_cs_margin_l import CANDIDATES  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_wheel_load_plausibility_20261024")
WEEKENDS = ("Labeled PRC", "Labeled NBR")


def ratio(path, setup_data):
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_car_data, load_parameters, prepare_vehicle_state
    from modules.wheel_loads import CORNERS, estimate_wheel_loads_from_dampers, straight_line_plausibility
    params = load_parameters()
    p = apply_resolved_vehicle(params, resolve_accuracy(params, setup_data, None))
    data = parse_csv(path)
    state = prepare_vehicle_state(data["channels"], p)
    if state is None:
        return {"status": "no vehicle state (required channel missing)"}
    dr = estimate_wheel_loads_from_dampers(state, data["channels"], p, load_car_data())
    if not any(dr[c]["valid"].any() for c in CORNERS):
        return {"status": "no valid damper corner"}
    out = straight_line_plausibility(state, dr, p)
    out.update(mass_kg=p["vehicle"]["mass_kg"], event_zero=(data.get("event_zero_entry") or {}).get("event"),
               valid_corners=[c for c in CORNERS if dr[c]["valid"].any()])
    return out


def main():
    from modules.data_paths import resolve_saved_path
    from tests.conftest import resolve_data_path
    os.makedirs(OUT, exist_ok=True)
    res = {"dev": {}, "labelled": {}}
    for c in CANDIDATES:
        res["dev"][c[0]] = dict(ratio(resolve_data_path(c[4][1]), None), label=c[3])
        print(c[0], res["dev"][c[0]], flush=True)
    con = sqlite3.connect(f"file:{os.path.join(ROOT, 'data', 'setuptool.db')}?mode=ro", uri=True)
    rows = con.execute("SELECT o.id, o.name, o.number, w.track, o.csv_path, o.setup_data FROM outings o JOIN "
                       "race_weekends w ON o.race_weekend_id = w.id WHERE w.track IN (?, ?) ORDER BY w.track, o.number",
                       WEEKENDS).fetchall()
    for oid, name, number, wk, csv_path, setup in rows:
        key = f"{wk} #{number} {name} (outing {oid})"
        path = resolve_saved_path(csv_path)
        res["labelled"][key] = (ratio(path, json.loads(setup) if setup else None) if path
                                else {"status": "file not found"})
        print(key, res["labelled"][key], flush=True)
    with open(os.path.join(OUT, "ratios.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
