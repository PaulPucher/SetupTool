# Read-only post-fix check (FIX ruling turn, item 4): reproduces the real
# persisted Dubai outing and calls the real generate_recommendations()
# after the _apply_undrivable_escalation lap-level-evidence repair. Prints
# every urgent-tier row (pierced/synthesized/gap/contradiction) across all
# corners carrying qualifying feedback, so C12 (expected: now fires
# US-APX-low as URGENT-RECOMMENDED) and any corner with no data support
# anywhere (expected: unchanged gap/contradiction row) can both be checked
# against the real data in one pass.

import sqlite3
import json

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.recommendation import (
    generate_recommendations, load_recommendations_config,
    aggregate_by_corner, _group_by_corner, _worst_feedback, _feedback_row,
)
from ui.views.outing_form import OutingForm

classify_fn = lambda s: OutingForm._classify_corner(None, s)

con = sqlite3.connect("data/setuptool.db")
con.row_factory = sqlite3.Row
row = con.execute(
    "SELECT feedback_data, setup_data, analysis_data, driver_id FROM outings WHERE id=1"
).fetchone()
feedback_data = json.loads(row["feedback_data"])
setup_data = json.loads(row["setup_data"])
analysis_meta = json.loads(row["analysis_data"])
lap_filter = analysis_meta["lap_filter"]
cap = analysis_meta["accuracy_cap"]

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
resolved = resolve_accuracy(params, setup_data, cap)
effective_params = apply_resolved_vehicle(params, resolved)

state = prepare_vehicle_state(data["channels"], effective_params)
beta = estimate_sideslip(state, effective_params)
slip = estimate_slip_angles(state, beta, effective_params)
forces = estimate_lateral_forces(state, effective_params)
cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
fz = estimate_vertical_loads(state, forces, effective_params)
summaries = summarise_corners(data["corners"], cs, stab, state, fz=fz, lap_filter=lap_filter)

config = load_recommendations_config()
raw_min = config["settings"]["consistency_gate"]["feedback_override"]["feedback_override_raw_min"]

driver_row = con.execute(
    "SELECT driving_level FROM drivers WHERE id=?", (row["driver_id"],)
).fetchone()
driving_level = driver_row["driving_level"] if driver_row else None

print("=" * 78)
print("CORNERS WHOSE WORST FEEDBACK CLEARS THE UNDRIVABLE-TIER FLOOR")
print("=" * 78)
qualifying_cids = []
for i, c in enumerate(feedback_data["corners"], 1):
    phase, raw_fb = _worst_feedback(c)
    if phase is not None and abs(raw_fb) >= raw_min:
        qualifying_cids.append(i)
        print(f"  C{i}: worst phase={phase} raw_fb={raw_fb:+g}  full row={c}")
if not qualifying_cids:
    print("  (none)")

print()
print("=" * 78)
print("GROUND TRUTH: generate_recommendations() result rows, driving_level=real and =10")
print("=" * 78)
for label, level in [("real", driving_level), ("override-10", 10)]:
    results = generate_recommendations(
        summaries, classify_fn, feedback_data, setup_data, config,
        outing=None, driving_level=level,
    )
    urgent_rows = [r for r in results if r.get("urgent")]
    print(f"  [{label}] driving_level={level}: {len(results)} total row(s), "
          f"{len(urgent_rows)} urgent")
    for r in urgent_rows:
        cids = [c["stable_corner_id"] for c in r["corners"]]
        print(f"      action_class={r['action_class']} actions={r['actions']} "
              f"corners={cids} conflicts={[c['stable_corner_id'] for c in r['conflicts']]}")
        for x in r["rationale"]:
            print(f"        rationale: {x['rationale']}")

print()
print("=" * 78)
print("PER-QUALIFYING-CORNER CROSS-CHECK (every corner named above)")
print("=" * 78)
results_real = generate_recommendations(
    summaries, classify_fn, feedback_data, setup_data, config,
    outing=None, driving_level=driving_level,
)
for cid in qualifying_cids:
    hits = [r for r in results_real if any(c["stable_corner_id"] == cid for c in r["corners"])]
    print(f"  C{cid}: {len(hits)} row(s)")
    for r in hits:
        print(f"      action_class={r['action_class']} urgent={r.get('urgent')} "
              f"actions={r['actions']}")
        for x in r["rationale"]:
            print(f"        rationale: {x['rationale']}")
