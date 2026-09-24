# Deepening work package, Phase 4f: end-to-end frame output on both real
# sessions, BEFORE (git HEAD -- the state main was in when this session's
# own branch started) vs AFTER (this session's own Phase 4a-d changes),
# reporting the shortlist deltas. Read-only against the real repo: the
# "before" snapshot is loaded from git HEAD's own blob content via a
# dynamically-exec'd module (never checks out/stashes/touches the working
# tree), so this can run safely alongside a dirty tree with real
# uncommitted Phase 1-4 work still in place.

import copy
import importlib.util
import json
import subprocess
import sys
import types

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.recommendation import load_setup_parameters_registry

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"
V3_SETUP_DATA = {"car": {
    "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
    "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
}}


def _git_show(path):
    return subprocess.run(["git", "show", f"HEAD:{path}"], capture_output=True, text=True, check=True).stdout


def load_old_decision_frame_module():
    """Dynamically execs modules/decision_frame.py's own git-HEAD content
    against a git-HEAD copy of config/decision_frame.json (written to a
    scratch path, patched into the old module's own config constant) --
    the OLD module's `from modules.recommendation import (...)` line
    resolves against the CURRENT modules/recommendation.py on disk, which
    is unchanged by this whole work package (verified: git diff HEAD --
    modules/recommendation.py is empty), so this is safe.
    """
    old_src = _git_show("modules/decision_frame.py")
    old_config_json = _git_show("config/decision_frame.json")
    scratch_config_path = "diagnostics/_scratch_old_decision_frame_config.json"
    with open(scratch_config_path, "w", encoding="utf-8") as f:
        f.write(old_config_json)
    old_src = old_src.replace(
        'DECISION_FRAME_CONFIG_PATH = "config/decision_frame.json"',
        f'DECISION_FRAME_CONFIG_PATH = "{scratch_config_path}"',
    )
    spec = importlib.util.spec_from_loader("old_decision_frame", loader=None)
    old_module = importlib.util.module_from_spec(spec)
    sys.modules["old_decision_frame"] = old_module
    exec(compile(old_src, "old_decision_frame.py (git HEAD)", "exec"), old_module.__dict__)
    return old_module, scratch_config_path


def run_pipeline(raw_file, setup_data=None):
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    beta, _fm, _gate, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, live_default, csv_path=raw_file)
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
    return {"summaries": summaries, "corners": corners, "state": state, "channels": data["channels"]}


def classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def _shortlist_summary(shortlist):
    return [(c["id"], c.get("grade"), round(c["score"], 3),
             tuple((a["parameter"], a.get("direction") or a.get("target")) for a in c["actions"]))
            for c in shortlist]


def compare(label, raw_file, setup_data, old_module, registry):
    print(f"\n{'='*70}\n{label}\n{'='*70}")
    pipe = run_pipeline(raw_file, setup_data=setup_data)
    summaries = pipe["summaries"]

    new_config = __import__("modules.decision_frame", fromlist=["load_decision_frame_config"]).load_decision_frame_config()
    from modules.decision_frame import build_evidence, aggregate_ls_by_corner, generate_candidates, generate_shortlist, resolve_conflicts
    ls_stats = aggregate_ls_by_corner(summaries)
    new_evidence = build_evidence(summaries, ls_stats, new_config, classify_fn,
                                   corners=pipe["corners"], state=pipe["state"], channels=pipe["channels"])
    new_candidates = generate_candidates(new_evidence, registry, new_config)
    new_shortlist = generate_shortlist(new_candidates, new_evidence, None, new_config)
    resolve_conflicts(new_shortlist)

    old_config = old_module.load_decision_frame_config()
    old_ls_stats = old_module.aggregate_ls_by_corner(summaries)
    old_evidence = old_module.build_evidence(summaries, old_ls_stats, old_config, classify_fn)
    old_candidates = old_module.generate_candidates(old_evidence, registry, old_config)
    old_shortlist = old_module.generate_shortlist(old_candidates, old_evidence, None, old_config)

    print(f"BEFORE (git HEAD): {len(old_evidence)} evidence, {len(old_candidates)} candidates, "
          f"{len(old_shortlist)} shortlist entries")
    print(f"AFTER  (this session): {len(new_evidence)} evidence, {len(new_candidates)} candidates, "
          f"{len(new_shortlist)} shortlist entries")

    old_ids = {c["id"] for c in old_shortlist}
    new_ids = {c["id"] for c in new_shortlist}
    print(f"\nNEW candidate ids not in BEFORE ({len(new_ids - old_ids)}):")
    for cid in sorted(new_ids - old_ids)[:20]:
        c = next(c for c in new_shortlist if c["id"] == cid)
        print(f"  + {cid} (grade={c['grade']}, score={c['score']:.3f})")
    print(f"\nRemoved (in BEFORE, not in AFTER) ({len(old_ids - new_ids)}):")
    for cid in sorted(old_ids - new_ids)[:20]:
        print(f"  - {cid}")

    common = old_ids & new_ids
    print(f"\nScore deltas for the {len(common)} candidate(s) present in both:")
    for cid in sorted(common):
        old_score = next(c["score"] for c in old_shortlist if c["id"] == cid)
        new_score = next(c["score"] for c in new_shortlist if c["id"] == cid)
        if abs(old_score - new_score) > 1e-6:
            print(f"  {cid}: {old_score:.3f} -> {new_score:.3f} (delta {new_score-old_score:+.3f})")


def main():
    old_module, scratch_path = load_old_decision_frame_module()
    registry = load_setup_parameters_registry()
    try:
        compare("Dubai", DUBAI_FILE, None, old_module, registry)
        compare("v3", V3_FILE, V3_SETUP_DATA, old_module, registry)
    finally:
        import os
        if os.path.exists(scratch_path):
            os.remove(scratch_path)


if __name__ == "__main__":
    main()
