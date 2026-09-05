# Frame-Stage-2 work package, Phase 3(d): parity verification. On Dubai
# AND v3 (production ekf_auto_pacejka default, accuracy cap=1, no feedback/
# setup data -- same fixed configuration tests/generate_golden.py uses),
# every recommendation the OLD 39-rule engine (modules.recommendation.
# generate_recommendations) produces must appear among the NEW decision-
# frame's own candidates (modules.decision_frame.generate_candidates) with
# a consistent parameter+direction/target -- rank may differ, that is the
# point. Read-only; no config/production change. Discrepancy = STOP per
# the work order.

import copy

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, load_car_data,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.recommendation import (
    generate_recommendations, load_recommendations_config, _action_key,
)
from modules.decision_frame import (
    build_evidence, aggregate_ls_by_corner, load_decision_frame_config,
    generate_candidates, generate_shortlist, resolve_conflicts, rule_bridge_status,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"
FIXED_CAP = 1


def classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def run_full_pipeline(raw_file, sideslip_source=None):
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved)

    live_default = sideslip_source or effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    if sideslip_source is not None:
        effective_params = copy.deepcopy(effective_params)
        effective_params["stability_estimation"]["sideslip_source"] = sideslip_source
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    if state is None:
        raise RuntimeError(f"{raw_file}: prepare_vehicle_state returned None")
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, live_default, csv_path=raw_file
    )
    if fallback_used:
        print(f"  ** NOTE: {raw_file} fell back to kinematic: {fallback_reason}")
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params,
                                  channels=data["channels"], car_data=load_car_data())
    long_forces = estimate_longitudinal_forces(state, data["channels"], effective_params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], effective_params)
    ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, effective_params)
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls, lap_filter=None)
    return {"data": data, "state": state, "corners": corners, "summaries": summaries,
            "channels": data["channels"], "params": effective_params}


def _action_pairs(actions):
    return {(a["parameter"], _action_key(a)) for a in actions}


def run_parity(raw_file, label, sideslip_source=None, loosen_consistency_gate=False):
    tag = f"{label} (sideslip_source={sideslip_source or 'production default'}, " \
          f"gate={'LOOSENED (diagnostic-only)' if loosen_consistency_gate else 'production'})"
    print(f"\n{'='*70}\nPARITY CHECK: {tag}\n{'='*70}")
    pipe = run_full_pipeline(raw_file, sideslip_source=sideslip_source)
    summaries = pipe["summaries"]

    rec_config = load_recommendations_config()
    if loosen_consistency_gate:
        # Diagnostic-only stress test: never written back to config. The
        # production consistency gate (min_repeat_laps=2, min_repeat_
        # fraction=0.4) currently produces ZERO recommendations on both
        # real sessions under every sideslip_source tried (verified, not
        # assumed) -- a vacuously-true parity check ("0 old results, 0
        # missing") is technically correct but not a real test of whether
        # the migration bridge actually reproduces old-engine firing
        # behaviour. Loosening the gate here forces the old engine to
        # produce real results FROM THE SAME REAL DATA so coverage can
        # actually be exercised, without touching the live config at all.
        rec_config = copy.deepcopy(rec_config)
        rec_config["settings"]["consistency_gate"]["min_repeat_laps"] = 1
        rec_config["settings"]["consistency_gate"]["min_repeat_fraction"] = 0.0

    old_results = generate_recommendations(
        summaries, classify_fn, feedback_data={}, setup_data=None, config=rec_config,
        outing=None, driving_level=None,
    )
    old_action_pairs = []
    for r in old_results:
        if not r["actions"]:
            continue  # synthetic urgent_gap rows carry no lever -- nothing to match on the new side
        old_action_pairs.append((r, _action_pairs(r["actions"])))

    df_config = load_decision_frame_config()
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, df_config, classify_fn,
                               corners=pipe["corners"], state=pipe["state"], channels=pipe["channels"])
    registry_module = __import__("modules.recommendation", fromlist=["load_setup_parameters_registry"])
    registry = registry_module.load_setup_parameters_registry()
    candidates = generate_candidates(evidence, registry, df_config)
    shortlist = generate_shortlist(candidates, evidence, None, df_config)
    resolve_conflicts(shortlist)

    new_action_pairs = set()
    for c in candidates:
        new_action_pairs |= _action_pairs(c["actions"])

    print(f"old engine: {len(old_results)} result(s), {len(old_action_pairs)} with a real lever action")
    print(f"new frame: {len(evidence)} evidence item(s), {len(candidates)} candidate(s), "
          f"{len(new_action_pairs)} distinct (parameter, direction/target) pair(s)")

    missing = []
    for r, pairs in old_action_pairs:
        uncovered = pairs - new_action_pairs
        status = "COVERED" if not uncovered else "MISSING"
        badge = f"C{r['corners'][0]['stable_corner_id']}" if r["corners"] else "?"
        print(f"  [{status}] {badge} action_class={r['action_class']} rules={r['rules_fired']} "
              f"pairs={sorted(pairs)}" + (f" UNCOVERED={sorted(uncovered)}" if uncovered else ""))
        if uncovered:
            missing.append((r, uncovered))

    print(f"\nSUMMARY: {len(old_action_pairs)-len(missing)}/{len(old_action_pairs)} old-engine results "
          f"have full coverage in the new frame's candidates.")
    if missing:
        print("*** PARITY FAILURE -- discrepancy found, per the work order this is a STOP condition. ***")
    else:
        print("PARITY OK.")
    return missing, pipe, evidence, candidates, shortlist


def print_rule_accounting():
    rec_config = load_recommendations_config()
    rules = rec_config["rules"]
    print(f"\n{'='*70}\nRULE MIGRATION ACCOUNTING ({len(rules)} rules)\n{'='*70}")
    counts = {}
    for r in rules:
        status = rule_bridge_status(r)
        counts[status] = counts.get(status, 0) + 1
        print(f"  {r['id']:<30} status={r.get('status'):<10} -> {status}")
    print(f"\nTotals: {counts}  (sum={sum(counts.values())}, expected {len(rules)})")


def main():
    print_rule_accounting()
    all_missing = []
    for raw_file, label in [(DUBAI_FILE, "Dubai"), (V3_FILE, "v3")]:
        missing_prod, *_ = run_parity(raw_file, label)
        all_missing.append((f"{label} production", missing_prod))
        # Diagnostic-only stress test: forces the old engine to actually
        # fire (see run_parity's own comment) so coverage is exercised for
        # real, not vacuously true. Never touches the live config.
        missing_loose, *_ = run_parity(raw_file, label, loosen_consistency_gate=True)
        all_missing.append((f"{label} loosened-gate stress test", missing_loose))

    print(f"\n{'='*70}\nOVERALL\n{'='*70}")
    for name, missing in all_missing:
        print(f"  {name}: missing={len(missing)}")


if __name__ == "__main__":
    main()
