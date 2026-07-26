# Read-only eligibility trace for the recommendation engine against the
# real, persisted Dubai outing (data/setuptool.db, outing id=1), WITH the
# driver's actually-entered feedback (non-zero this time -- see PLAN.md/
# session notes; unlike the WP2b-2 trace's all-zero baseline). For every
# corner carrying non-zero feedback and every data-flagged corner: full
# per-rule eligibility chain (severity, consistency gate, feedback
# scaling, corroboration, action_class, budget selection), reusing the
# real modules/recommendation.py internals directly rather than
# re-deriving the logic. Extends diagnostics/inspect_wp2b2_recommendation_
# trace.py's setup with the deeper per-corner chain-of-custody detail.
#
# Session addendum: the user ran Generate with driving_level=10 (weight
# 1.5, the top of the table) and STILL saw zero RECOMMENDED rows -- so
# this script also explicitly dumps (a) the feedback values the engine
# actually receives at call time, (b) whether the live-form values match
# what's persisted, and (c) traces both driving_level=None (resolved from
# the real driver) and driving_level=10 side by side per corner.

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
    aggregate_by_corner, _group_by_corner, _phase_verdict, _verdict_present,
    _consistency_gate_ok, _feedback_row, _feedback_value,
    _evaluate_rule, _match_is_recommended, _resolve_feedback_weight,
    _resolve_source_balance, SEVERITY_RANK, _NON_FIRING_STATUSES,
    PHASE_TO_FEEDBACK_KEY,
)
from ui.views.outing_form import OutingForm

classify_fn = lambda s: OutingForm._classify_corner(None, s)

# --- 1. Reproduce the real pipeline, same inputs the live app used ---------
con = sqlite3.connect("data/setuptool.db")
con.row_factory = sqlite3.Row
row = con.execute(
    "SELECT feedback_data, setup_data, analysis_data, driver_id FROM outings WHERE id=1"
).fetchone()
feedback_data = json.loads(row["feedback_data"])
setup_data = json.loads(row["setup_data"])
analysis_meta = json.loads(row["analysis_data"])
lap_filter = analysis_meta["lap_filter"]  # persisted: [1, 2, 3, 4] -- Exclude In/Out Laps was on
cap = analysis_meta["accuracy_cap"]       # persisted: None ("best available")

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

print("=" * 78)
print("PIPELINE REPRODUCTION CHECK")
print("=" * 78)
print(f"lap_filter used (matches persisted analysis_data): {lap_filter}")
print(f"accuracy_cap used: {cap}")
print(f"resolved levels this run: {resolved['levels']}")
print(f"resolved levels persisted: {analysis_meta['resolved_levels']}")
print(f"resolved levels MATCH persisted: {resolved['levels'] == analysis_meta['resolved_levels']}")
print(f"n summaries this run: {len(summaries)}  (persisted: {len(analysis_meta['summaries'])})")

# --- 2. Where does the engine read feedback from? persisted vs live form ---
print()
print("=" * 78)
print("WHERE THE ENGINE READS FEEDBACK FROM")
print("=" * 78)
print("Code path: ui/views/outing_form.py _generate_recommendations() calls")
print("  feedback_data = json.loads(self._collect_feedback_data())")
print("_collect_feedback_data() reads the LIVE FEEDBACK TABLE WIDGETS in the")
print("running form (QSpinBox/QComboBox values currently on screen), NOT")
print("self.outing.feedback_data (the persisted DB column) directly. The two")
print("only coincide if the user has clicked Save since last editing the")
print("feedback table -- Generate always uses whatever is CURRENTLY TYPED IN,")
print("saved or not.")
print()
print("This script reads the PERSISTED column (data/setuptool.db) since it has")
print("no live form to read from -- it can only prove what's ON DISK, not what")
print("was on screen at the moment the user clicked Generate. Persisted values")
print("used below (outing id=1, non-zero rows only):")
for i, c in enumerate(feedback_data["corners"], 1):
    nonzero = {k: v for k, v in c.items() if k != "worst" and v}
    if nonzero or c.get("worst"):
        print(f"  corner {i}: {c}")
print()
print("-> IF the user's on-screen Generate click used different values than the")
print("   table above (e.g. entered but never Saved, or edited after the last")
print("   Save), this trace's feedback-dependent branches will not match what")
print("   the live run actually saw. Re-run this script's target corner/values")
print("   with the live session's exact entries if that's suspected -- the")
print("   FEEDBACK_ROWS dict below is the one variable to substitute.")

# --- 3. Aggregate + identify target corners ---------------------------------
aggregated = aggregate_by_corner(summaries)
by_corner_laps = _group_by_corner(summaries)
config = load_recommendations_config()
settings = config["settings"]

feedback_corner_ids = {
    i for i, c in enumerate(feedback_data["corners"], 1)
    if any(c.get(k) for k in ("e1", "e2", "a3", "x4", "x5")) or c.get("worst")
}

data_flagged_ids = set()
corner_verdicts = {}
for cid, corner in aggregated.items():
    severity, short, long_v, _colour = classify_fn(corner)
    corner_verdicts[cid] = (severity, short, long_v)
    if severity != "normal":
        data_flagged_ids.add(cid)

target_ids = sorted(feedback_corner_ids | data_flagged_ids)
print()
print("=" * 78)
print(f"TARGET CORNERS: feedback={sorted(feedback_corner_ids)}  "
      f"data-flagged={sorted(data_flagged_ids)}  union={target_ids}")
print("=" * 78)
for cid in target_ids:
    sev, short, long_v = corner_verdicts.get(cid, ("?", "?", "?"))
    fb_row = feedback_data["corners"][cid - 1] if cid - 1 < len(feedback_data["corners"]) else {}
    print(f"  C{cid}: aggregate verdict = {sev} / {short}   feedback = {fb_row}")

# --- 4. Driver level(s) to trace --------------------------------------------
# Raw sqlite3, not the ORM (Session/Driver) -- avoids needing every mapped
# model importable/registered for SQLAlchemy's relationship string
# resolution (Outing.driver_weekend etc.), consistent with this script's
# existing raw-connection use above.
driver_row = con.execute(
    "SELECT driving_level FROM drivers WHERE id=?", (row["driver_id"],)
).fetchone()
real_level = driver_row["driving_level"] if driver_row else None
print()
print(f"Driver (id={row['driver_id']}) driving_level on file: {real_level}")
levels_to_trace = [("real", real_level), ("override-10", 10)]

# --- 5. Full eligibility chain per rule x target corner ---------------------
print()
print("=" * 78)
print("FULL ELIGIBILITY CHAIN, PER RULE x TARGET CORNER")
print("=" * 78)

any_potential_match = False
for rule in config["rules"]:
    if rule.get("status") in _NON_FIRING_STATUSES:
        continue
    phases = rule["phases"]
    condition = rule["condition"]
    required_speed_class = condition.get("speed_class")
    trigger = condition["trigger"]

    for cid in target_ids:
        corner = aggregated.get(cid)
        if corner is None:
            continue
        # Cheap pre-filter: does this rule's phase set touch a phase this
        # corner has non-zero feedback in, OR does the corner carry a
        # data verdict this rule's phases could plausibly match? Trace
        # every rule/corner pair whose phases overlap the corner's
        # feedback OR whose verdict the rule's condition names -- avoids
        # printing hundreds of obviously-irrelevant rule/corner pairs.
        fb_row = _feedback_row(feedback_data, cid)
        fb_touches_phase = any(fb_row.get(PHASE_TO_FEEDBACK_KEY.get(p), 0) for p in phases)
        severity_here, short_here = _phase_verdict(corner, phases, classify_fn)
        verdict_name = condition.get("verdict")
        data_relevant = verdict_name and _verdict_present(short_here, verdict_name)
        if not (fb_touches_phase or data_relevant):
            continue

        speed_gate_ok = (required_speed_class is None
                         or corner.get("speed_class") == required_speed_class)

        any_potential_match = True
        print()
        print(f"--- rule {rule['id']}  cell_id={rule.get('cell_id')}  "
              f"status={rule['status']}  trigger={trigger} ---")
        print(f"    provenance={rule.get('elicitation_provenance')}  "
              f"situational={rule.get('situational', False)}")
        print(f"    phases={phases}  condition_verdict={verdict_name}  "
              f"required_speed_class={required_speed_class}")
        print(f"    C{cid}: corner speed_class={corner.get('speed_class')}  "
              f"speed_gate_ok={speed_gate_ok}")
        print(f"    data severity on these phases = {severity_here} / {short_here}  "
              f"(min_severity required: {condition.get('min_severity', 'n/a')})")

        min_sev = condition.get("min_severity", "normal")
        severity_ok = SEVERITY_RANK.get(severity_here, 0) >= SEVERITY_RANK.get(min_sev, 0)
        verdict_ok = (verdict_name is None) or _verdict_present(short_here, verdict_name)
        print(f"    verdict_present={verdict_ok}  severity_ok(>= {min_sev})={severity_ok}")

        if trigger in ("data", "both"):
            gate_ok = _consistency_gate_ok(
                cid, by_corner_laps, phases, verdict_name, min_sev, classify_fn, settings
            )
            laps = by_corner_laps.get(cid, [])
            repeat = sum(
                1 for lap_s in laps
                if _verdict_present(_phase_verdict(lap_s, phases, classify_fn)[1], verdict_name)
                and SEVERITY_RANK[_phase_verdict(lap_s, phases, classify_fn)[0]] >= SEVERITY_RANK[min_sev]
            ) if verdict_name else None
            cg = settings.get("consistency_gate", {})
            print(f"    consistency_gate: repeat={repeat}/{len(laps)} laps  "
                  f"need >= {cg.get('min_repeat_laps')} laps AND "
                  f">= {cg.get('min_repeat_fraction')} fraction  -> pass={gate_ok}")
        else:
            gate_ok = True
            print("    consistency_gate: n/a (driver-trigger rule, exempt)")

        raw_fb = _feedback_value(fb_row, phases)
        min_abs = condition.get("min_feedback_abs")
        feedback_sign = condition.get("feedback_sign")

        # Authoritative fire/action_class determination: call the REAL
        # _evaluate_rule (restricted to this one corner) rather than
        # re-deriving its gate logic here -- eliminates any risk of this
        # trace script's own boolean composition drifting from what the
        # engine actually does (sign checks, both-trigger's own floor,
        # source_balance zeroing a score to below min_score_to_show, etc).
        single_corner = {cid: corner}
        single_laps = {cid: by_corner_laps.get(cid, [])}
        source_balance = _resolve_source_balance(config, outing=None)
        for label, level in levels_to_trace:
            fw = _resolve_feedback_weight(config, level)
            scaled_fb = raw_fb * fw
            clears_floor = (min_abs is None) or (abs(scaled_fb) >= min_abs)
            sign_ok = (feedback_sign is None) or (
                (feedback_sign == "negative" and scaled_fb < 0)
                or (feedback_sign == "positive" and scaled_fb > 0)
            )
            print(f"    [{label}] driving_level={level} weight={fw:.2f}  "
                  f"raw_fb={raw_fb:+g} -> scaled={scaled_fb:+.3f}  "
                  f"min_feedback_abs={min_abs}  clears_floor={clears_floor}  "
                  f"sign_ok={sign_ok}")

            matches = _evaluate_rule(rule, single_corner, single_laps, feedback_data,
                                      classify_fn, settings, source_balance, fw)
            if not matches:
                print(f"    [{label}] -> DOES NOT FIRE (no match produced by "
                      f"_evaluate_rule -- see verdict/severity/gate/sign checks above)")
                continue
            m = matches[0]
            is_rec = _match_is_recommended(m, rule, settings)
            # NOTE: min_score_to_show is checked at the BUCKET level in the
            # real generate_recommendations (summed across every corner/rule
            # landing in the same parameter+direction bucket), not per match
            # -- this single match's score is a lower bound on that bucket's
            # total, not the final show/hide decision by itself.
            print(f"    [{label}] FIRES: this match's score={m['score']:.3f} "
                  f"(bucket total may be higher if other corners/rules share "
                  f"this bucket; min_score_to_show={settings['min_score_to_show']})  "
                  f"corroborated={m['corroborated']}  conflict={m['conflict']}  "
                  f"-> action_class={'RECOMMENDED' if is_rec else 'advisory'}")

if not any_potential_match:
    print("  (no rule/corner pair had overlapping phases with any non-zero "
          "feedback or a matching data verdict -- see summary below)")

# --- 6. Ground truth: what generate_recommendations() actually returns -----
print()
print("=" * 78)
print("GROUND TRUTH: generate_recommendations() actual output")
print("=" * 78)
for label, level in levels_to_trace:
    results = generate_recommendations(
        summaries, classify_fn, feedback_data, setup_data, config,
        outing=None, driving_level=level,
    )
    n_rec = sum(1 for r in results if r["action_class"] == "recommended")
    n_adv = sum(1 for r in results if r["action_class"] == "advisory")
    print(f"  [{label}] driving_level={level}: {len(results)} bucket(s) total "
          f"({n_rec} recommended, {n_adv} advisory)")
    for r in results:
        print(f"      {r['actions']}  score={r['score']}  action_class={r['action_class']}  "
              f"selected={r['selected']}  corners={[c['stable_corner_id'] for c in r['corners']]}  "
              f"cell_ids={r['cell_ids']}")
