# Read-only verify pass (FIX ruling turn, urgent-tier lap-level evidence).
# Question: does the undrivable-feedback tier's synthetic gap row at C12
# fire because the AGGREGATED (median-of-medians) corner verdict dilutes
# a pattern individual laps actually show? Reproduces the real persisted
# Dubai outing (data/setuptool.db, id=1) and the exact internals
# _apply_undrivable_escalation uses -- no rule/threshold/code changed here.

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
    aggregate_by_corner, _group_by_corner, _phase_verdict, _axle_verdict,
    _feedback_row, _worst_feedback, _rule_structurally_covers,
    PHASE_KEYS, _resolve_feedback_weight,
)
from ui.views.outing_form import OutingForm

classify_fn = lambda s: OutingForm._classify_corner(None, s)
TARGET_CID = 12

# --- reproduce the real pipeline, same inputs the live app used ------------
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
settings = config["settings"]
aggregated = aggregate_by_corner(summaries)
by_corner_laps = _group_by_corner(summaries)

driver_row = con.execute(
    "SELECT driving_level FROM drivers WHERE id=?", (row["driver_id"],)
).fetchone()
driving_level = driver_row["driving_level"] if driver_row else None

print("=" * 78)
print(f"C{TARGET_CID} -- PER-LAP VERDICTS (all 5 phases, classify_fn per lap)")
print("=" * 78)
laps = by_corner_laps.get(TARGET_CID, [])
print(f"n laps contributing to C{TARGET_CID}: {len(laps)}")
for lap_summary in sorted(laps, key=lambda s: s["lap_number"]):
    print(f"  lap {lap_summary['lap_number']}:")
    for phase in PHASE_KEYS:
        if phase not in lap_summary["phases"] or lap_summary["phases"][phase].get("n_samples", 0) == 0:
            print(f"    {phase:16s}: no samples")
            continue
        severity, short = _phase_verdict(lap_summary, [phase], classify_fn)
        print(f"    {phase:16s}: severity={severity:8s} short={short}")

print()
print("=" * 78)
print(f"C{TARGET_CID} -- AGGREGATED (median-of-medians) VERDICT, same classify_fn")
print("=" * 78)
agg_corner = aggregated[TARGET_CID]
sev_overall, short_overall, long_overall, _colour = classify_fn(agg_corner)
print(f"  overall (all phases): severity={sev_overall} short={short_overall}")
print(f"  long: {long_overall}")
for phase in PHASE_KEYS:
    severity, short = _phase_verdict(agg_corner, [phase], classify_fn)
    print(f"  {phase:16s}: severity={severity:8s} short={short}")
print(f"  speed_class (modal): {agg_corner['speed_class']}")

print()
print("=" * 78)
print(f"C{TARGET_CID} -- FEEDBACK + UNDRIVABLE-TIER MECHANISM "
      "(reproducing _apply_undrivable_escalation's own steps)")
print("=" * 78)
fb_row = _feedback_row(feedback_data, TARGET_CID)
print(f"  feedback row: {fb_row}")
phase, raw_fb = _worst_feedback(fb_row)
print(f"  _worst_feedback -> phase={phase} raw_fb={raw_fb:+g}")
esc_cfg = settings.get("consistency_gate", {}).get("feedback_override", {})
raw_min = esc_cfg.get("feedback_override_raw_min")
print(f"  escalation_enabled={esc_cfg.get('escalation_enabled')} raw_min={raw_min}  "
      f"clears floor={phase is not None and abs(raw_fb) >= (raw_min or 0)}")

if phase is not None and raw_min is not None and abs(raw_fb) >= raw_min:
    implied_verdict = "oversteer" if raw_fb > 0 else "understeer"
    severity, short = _phase_verdict(agg_corner, [phase], classify_fn)
    axle = _axle_verdict(short)
    print(f"  implied_verdict={implied_verdict}")
    print(f"  aggregate severity/short AT THIS PHASE ({phase}): severity={severity} short={short}  "
          f"axle_verdict={axle}")

    covering = [
        r["id"] for r in config["rules"]
        if _rule_structurally_covers(r, phase, implied_verdict, agg_corner.get("speed_class"))
    ]
    print(f"  rules structurally covering (phase={phase}, verdict={implied_verdict}, "
          f"speed_class={agg_corner.get('speed_class')}): {covering}")

    print()
    print(f"  --- per-lap check AT THE SAME PHASE ({phase}) -- what the aggregate hid ---")
    lap_hits = 0
    for lap_summary in sorted(laps, key=lambda s: s["lap_number"]):
        if phase not in lap_summary["phases"] or lap_summary["phases"][phase].get("n_samples", 0) == 0:
            print(f"    lap {lap_summary['lap_number']}: no samples")
            continue
        lap_sev, lap_short = _phase_verdict(lap_summary, [phase], classify_fn)
        lap_axle = _axle_verdict(lap_short)
        hit = lap_axle == implied_verdict
        lap_hits += hit
        print(f"    lap {lap_summary['lap_number']}: severity={lap_sev:8s} short={lap_short}  "
              f"axle={lap_axle}  matches feedback direction={hit}")
    print(f"  -> {lap_hits}/{len(laps)} laps show a verdict matching the feedback "
          f"direction at phase {phase} (aggregate axle_verdict was {axle})")
else:
    print("  feedback does not clear the undrivable-tier floor for this corner -- tier inactive.")

print()
print("=" * 78)
print(f"GROUND TRUTH: generate_recommendations() actual result rows touching C{TARGET_CID}")
print("=" * 78)
for label, level in [("real", driving_level), ("override-10", 10)]:
    results = generate_recommendations(
        summaries, classify_fn, feedback_data, setup_data, config,
        outing=None, driving_level=level,
    )
    hits = [r for r in results if any(c["stable_corner_id"] == TARGET_CID for c in r["corners"])]
    print(f"  [{label}] driving_level={level}: {len(hits)} row(s) touching C{TARGET_CID}")
    for r in hits:
        print(f"      action_class={r['action_class']} urgent={r.get('urgent')} "
              f"actions={r['actions']} rationale={[x['rationale'] for x in r['rationale']]}")
