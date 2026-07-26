# Read-only trace of the WP2b-2 recommendation engine against the real,
# persisted Dubai outing (data/setuptool.db, outing id=1: Sample_Dubai.txt,
# real setup_data from the setup sheet). feedback_data was all-zero when
# this script was first written -- no longer true, the user has since
# entered real feedback for this outing (see the printed source-echo line
# below, added for exactly this reason: a stale assumption here could
# otherwise silently mislead a future reader). Default view: lap_filter=None
# (matches the app's own default -- "Exclude In/Out Laps" unchecked, all 7
# laps 0-6 included), not the 5-valid-lap subset. No accuracy resolution
# override (plain load_parameters(), consistent with other diagnostics/*.py
# scripts) -- this trace is about which RULES fire, not vehicle-parameter
# accuracy level.
#
# For every rule that produces at least one match: cell_id, status, matched
# corners (stable_corner_id -- the grid/map identity -- alongside the raw
# per-lap corner_number, since these can legitimately differ, see the
# corner-numbering bug writeup elsewhere this session), per-lap verdict
# pattern, aggregate severity, and where it landed after bucketing
# (action_class, selected, parameter_conflict).

import sqlite3
import json

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)
from modules.recommendation import (
    generate_recommendations, load_recommendations_config,
    aggregate_by_corner, _group_by_corner, _evaluate_rule, _phase_verdict,
    _NON_FIRING_STATUSES,
)
from ui.views.outing_form import OutingForm

classify_fn = lambda s: OutingForm._classify_corner(None, s)

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
summaries = summarise_corners(data["corners"], cs, stab, state, lap_filter=None)

con = sqlite3.connect("data/setuptool.db")
con.row_factory = sqlite3.Row
row = con.execute(
    "SELECT feedback_data, setup_data FROM outings WHERE id=1"
).fetchone()
feedback_data = json.loads(row["feedback_data"])
setup_data = json.loads(row["setup_data"])
any_feedback = any(
    c["worst"] or any(c[k] != 0 for k in ("e1", "e2", "a3", "x4", "x5"))
    for c in feedback_data["corners"]
)
# Explicit source echo (added after this script's own header comment was
# found to be stale -- it used to say feedback_data was "all-zero", which
# stopped being true once the user actually entered feedback for this
# outing). generate_recommendations() itself has two possible feedback
# sources depending on the caller: the running app's live-form widgets
# (ui/views/outing_form.py _generate_recommendations, via
# self._collect_feedback_data()) or a persisted-db read like this one.
# This script has no live QWidget state to read from -- it can ONLY ever
# be "persisted-db" mode, never "live-form" -- so a future run's numbers
# reflect the last SAVED state, not necessarily what's currently typed
# into an open form. See diagnostics/inspect_recommendation_eligibility_
# trace.py for the fuller live-form-vs-persisted-db explanation.
print("feedback source: persisted-db (data/setuptool.db, outing id=1) "
      "-- NOT live-form; this script cannot read an open form's widgets.")
print(f"Persisted feedback_data: corner_count={feedback_data['corner_count']}, "
      f"any non-zero/worst entry: {any_feedback}")

config = load_recommendations_config()
settings = config["settings"]

aggregated = aggregate_by_corner(summaries)
by_corner_laps = _group_by_corner(summaries)

# Raw per-lap corner_number for each stable_corner_id, for the numbering
# cross-check this task explicitly asks for.
raw_numbers = {}
for s in summaries:
    cid = s.get("stable_corner_id")
    if cid is None:
        continue
    raw_numbers.setdefault(cid, {})[s["lap_number"]] = s["corner_number"]

print(f"\n{len(aggregated)} stable corners, {len(summaries)} corner-lap instances.\n")

results = generate_recommendations(summaries, classify_fn, feedback_data, setup_data, config)
bucket_by_rule = {}
for r in results:
    for rid in r["rules_fired"]:
        bucket_by_rule[rid] = r

fired_any = False
for rule in config["rules"]:
    if rule.get("status") in _NON_FIRING_STATUSES:
        continue
    matches = _evaluate_rule(rule, aggregated, by_corner_laps, feedback_data,
                              classify_fn, settings, settings["source_balance"])
    if not matches:
        continue
    fired_any = True
    print(f"=== {rule['cell_id']}  (id={rule['id']}, status={rule['status']}) ===")
    print(f"    phases={rule['phases']}  verdict={rule['condition']['verdict']}  "
          f"speed_class={rule['condition'].get('speed_class')}")
    for m in matches:
        cid = m["stable_corner_id"]
        laps = sorted(by_corner_laps.get(cid, []), key=lambda s: s["lap_number"])
        pattern = []
        for lap_s in laps:
            sev, short = _phase_verdict(lap_s, rule["phases"], classify_fn)
            pattern.append(f"L{lap_s['lap_number']}:{sev[:3]}")
        raw_nums = raw_numbers.get(cid, {})
        raw_str = ",".join(f"L{lap}:{n}" for lap, n in sorted(raw_nums.items()))
        bucket = bucket_by_rule.get(rule["id"])
        print(f"    C{cid} (raw corner_number per lap: {raw_str})")
        print(f"        per-lap verdict pattern: {' '.join(pattern)}")
        print(f"        aggregate severity={m['severity']}  corroborated={m['corroborated']}  "
              f"conflict(driver/data)={m['conflict']}  score_contrib={m['score']:.3f}")
        if bucket is not None:
            print(f"        -> bucket action_class={bucket['action_class']}  "
                  f"selected={bucket['selected']}  parameter_conflict={bucket['parameter_conflict']}  "
                  f"limit_status={bucket['limit_status']}  bucket_score={bucket['score']}")
        else:
            print("        -> did not clear min_score_to_show (no bucket)")
    print()

if not fired_any:
    print("No rule produced any match against this outing's data.")

print("\n=== Final ranked results ===")
for r in results:
    actions_str = " + ".join(
        f"{a['parameter']}->{a['target']}" if "target" in a
        else f"{a['parameter']} {a['direction']} ({a['delta']:+g})"
        for a in r["actions"]
    )
    corner_ids = ",".join(f"C{c['stable_corner_id']}" for c in r["corners"])
    print(f"[{'/'.join(r['cell_ids'])}] {actions_str}  score={r['score']}  "
          f"class={r['action_class']}  selected={r['selected']}  "
          f"conflict={r['parameter_conflict']}  limit={r['limit_status']}  corners={corner_ids}")

print("\n=== Front-ARB cross-check ===")
front_arb_hits = [r for r in results if any(a["parameter"] in ("arb_fl", "arb_fr") for a in r["actions"])]
for r in front_arb_hits:
    dirs = {a["direction"] for a in r["actions"] if a["parameter"] in ("arb_fl", "arb_fr")}
    print(f"  cell_ids={r['cell_ids']}  front-ARB direction(s)={dirs}  "
          f"corners={[c['stable_corner_id'] for c in r['corners']]}  "
          f"parameter_conflict={r['parameter_conflict']}")
if not front_arb_hits:
    print("  no result touches arb_fl/arb_fr at all")
