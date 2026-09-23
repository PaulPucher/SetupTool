# Decision-matrix frame, Stage 1, Phase 6: targeted unit tests for
# modules/decision_frame.py. NOT a full-suite/golden-file package (the
# work order's own scope: "targeted tests only" -- this module is
# additive, no existing production import path changes). Hand-crafted
# summary/evidence/candidate dicts for pure-function unit tests, same
# convention tests/test_longitudinal_stiffness.py's own synthetic-ramp
# tests already use for a formula-correctness check -- the CLAUDE.md
# "real data only" rule governs ANALYSIS validation claims, not a minimal
# fixture proving a scoring formula's arithmetic. The one end-to-end test
# (test_end_to_end_real_dubai) uses the real, shared pipeline_result
# fixture (conftest.py), per the work order's own instruction.

import copy

import numpy as np
import pytest

from modules.decision_frame import (
    EFFORT_RANK,
    PHASE_KEYS,
    STATUS_BLOCKED_AT_EDGE,
    STATUS_CONTRADICTED,
    STATUS_NO_TRIGGER,
    STATUS_NOT_ASSESSABLE,
    STATUS_PROPOSED,
    TRIGGER_BOTH_AGREEING,
    TRIGGER_DATA_ONLY,
    TRIGGER_FEEDBACK_ONLY,
    aggregate_ls_by_corner,
    build_evidence,
    candidate_severity,
    generate_candidates,
    generate_display_split,
    generate_lever_inventory,
    generate_shortlist,
    group_display_rows,
    load_decision_frame_config,
    reachable_lever_keys,
    render_action_line,
    render_tail_line,
    render_top_line,
    resolve_conflicts,
    rule_bridge_status,
    score,
    tyre_pressure_flags,
)
from modules.recommendation import load_recommendations_config, load_setup_parameters_registry


def classify_fn(summary):
    # Same "None-self reuse" convention tests/generate_golden.py and
    # tests/test_golden_pipeline.py already use -- _classify_corner makes
    # no Qt calls and never touches self, confirmed by reading its body
    # (ui/views/outing_form.py) before relying on this.
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def _stat(median, n=50):
    return {"median": median, "p25": median, "p75": median, "n": n}


def _make_summary(lap_number, stable_corner_id, speed_class,
                   csf_by_phase=None, csr_by_phase=None, n_samples_by_phase=None,
                   ls_r_by_phase=None, ls_f_by_phase=None):
    csf_by_phase = csf_by_phase or {}
    csr_by_phase = csr_by_phase or {}
    n_samples_by_phase = n_samples_by_phase or {}
    ls_r_by_phase = ls_r_by_phase or {}
    ls_f_by_phase = ls_f_by_phase or {}
    phases = {}
    for phase in PHASE_KEYS:
        n = n_samples_by_phase.get(phase, 50)
        if n == 0:
            # No-signal phase: every stat block reads NaN, n=0 -- same
            # shape modules.stability_analysis.summarise_corners produces
            # for an empty phase slice.
            entry = {
                "n_samples": 0,
                "cs_ratio_f": _stat(float("nan"), 0),
                "cs_ratio_r": _stat(float("nan"), 0),
                "stability_observed_Nm_per_deg": _stat(float("nan"), 0),
            }
        else:
            entry = {
                "n_samples": n,
                "cs_ratio_f": _stat(csf_by_phase.get(phase, 1.0), n),
                "cs_ratio_r": _stat(csr_by_phase.get(phase, 1.0), n),
                "stability_observed_Nm_per_deg": _stat(500.0, n),
            }
        if phase in ls_r_by_phase or phase in ls_f_by_phase:
            entry["ls_ratio_f"] = _stat(ls_f_by_phase.get(phase, 1.0), n)
            entry["ls_ratio_r"] = _stat(ls_r_by_phase.get(phase, 1.0), n)
        phases[phase] = entry
    return {
        "lap_number": lap_number,
        "stable_corner_id": stable_corner_id,
        "speed_class": speed_class,
        "phases": phases,
        "apex_region": None,
    }


# --- Scoring determinism ------------------------------------------------

def _dummy_candidate(param="arb_rl", direction="soften", delta=-1, effort_class="minutes",
                      effect_class="primary", evidence_refs=None):
    return {
        "id": "dummy", "scenario": "exit_oversteer", "corner": 4, "phase": "exit_4",
        "lever_family": "arb_spring",
        "actions": [{"parameter": param, "direction": direction, "delta": delta}],
        "effort_class": effort_class, "effect_class": effect_class,
        "grade": "proposed", "cell_id": None,
        "evidence_refs": evidence_refs if evidence_refs is not None else [
            {"type": "corner_verdict", "corner": 4, "phase": "exit_4",
             "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"},
        ],
        "rationale": "test",
    }


def test_scoring_determinism():
    config = load_decision_frame_config()
    candidate = _dummy_candidate()
    evidence = candidate["evidence_refs"]
    result_a = score(candidate, evidence, None, config)
    result_b = score(candidate, evidence, None, config)
    assert result_a == result_b


# --- Cheap check outranks on equal severity ------------------------------

def test_cheap_check_outranks_on_equal_severity():
    # Identical in every respect (same evidence, same effect_class, same
    # severity/confidence) except effort_class -- isolates the inverse-
    # effort component. A parameter absent from both parameter_windows and
    # interaction_table (a made-up name) keeps those two components at a
    # clean, identical 0 for both candidates, so only effort can move the
    # ranking.
    config = load_decision_frame_config()
    evidence = [{"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                 "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}]
    cheap = _dummy_candidate(param="test_param_not_in_registry", effort_class="seconds",
                              evidence_refs=evidence)
    cheap["id"] = "cheap"
    expensive = _dummy_candidate(param="test_param_not_in_registry", effort_class="garage_hours",
                                  evidence_refs=evidence)
    expensive["id"] = "expensive"

    score_cheap = score(cheap, evidence, None, config)
    score_expensive = score(expensive, evidence, None, config)

    assert score_cheap["components"]["headroom"] == 0.0
    assert score_expensive["components"]["headroom"] == 0.0
    assert score_cheap["components"]["interaction"] == 0.0
    assert score_expensive["components"]["interaction"] == 0.0
    assert score_cheap["total"] > score_expensive["total"]


# --- Interaction penalty sign --------------------------------------------

def test_interaction_penalty_sign_negative_against_other_active_problem():
    # arb_rl soften -> understeer_tendency, sign=-1 (config/decision_frame.
    # json interaction_table). This candidate's OWN evidence is oversteer
    # at C4 exit_4; an OTHER active understeer evidence item at the SAME
    # corner (different phase, excluded from evidence_refs) should pull
    # the interaction_penalty component negative.
    config = load_decision_frame_config()
    own_evidence = {"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                     "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    other_understeer = {"type": "corner_verdict", "corner": 4, "phase": "entry_2_turnin",
                         "verdict": "understeer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    candidate = _dummy_candidate(param="arb_rl", direction="soften", evidence_refs=[own_evidence])

    with_other_problem = score(candidate, [own_evidence, other_understeer], None, config)
    assert with_other_problem["components"]["interaction"] < 0.0
    assert with_other_problem["interaction_notes"]

    without_other_problem = score(candidate, [own_evidence], None, config)
    assert without_other_problem["components"]["interaction"] == 0.0
    assert without_other_problem["interaction_notes"] == []


# --- No-signal evidence lowers confidence --------------------------------

def test_no_signal_phases_lower_confidence():
    # Two 4-lap corners, repeat count held FIXED at 2 (laps 1-2 show
    # moderate oversteer both times) -- the only difference is whether
    # laps 3-4 carry real "normal" signal or none at all. Isolates signal
    # validity from repeat count: confidence must be strictly lower when
    # the non-matching laps have no signal, not merely different.
    oversteer_csr = {"exit_4": -0.15}  # beyond STRONG_CSR -- real, moderate+ oversteer
    normal_csr = {"exit_4": 1.0}

    full_signal_summaries = [
        _make_summary(1, 4, "high", csr_by_phase=oversteer_csr),
        _make_summary(2, 4, "high", csr_by_phase=oversteer_csr),
        _make_summary(3, 4, "high", csr_by_phase=normal_csr),
        _make_summary(4, 4, "high", csr_by_phase=normal_csr),
    ]
    no_signal_summaries = [
        _make_summary(1, 4, "high", csr_by_phase=oversteer_csr),
        _make_summary(2, 4, "high", csr_by_phase=oversteer_csr),
        _make_summary(3, 4, "high", n_samples_by_phase={"exit_4": 0}),
        _make_summary(4, 4, "high", n_samples_by_phase={"exit_4": 0}),
    ]

    config = load_decision_frame_config()
    ev_full = build_evidence(full_signal_summaries, aggregate_ls_by_corner(full_signal_summaries),
                              config, classify_fn)
    ev_no_signal = build_evidence(no_signal_summaries, aggregate_ls_by_corner(no_signal_summaries),
                                   config, classify_fn)

    full_item = next(e for e in ev_full if e["type"] == "corner_verdict"
                      and e["phase"] == "exit_4" and e["verdict"] == "oversteer")
    no_signal_item = next(e for e in ev_no_signal if e["type"] == "corner_verdict"
                           and e["phase"] == "exit_4" and e["verdict"] == "oversteer")

    assert full_item["confidence"] == pytest.approx(0.5)      # repeat 2/4 * signal 4/4
    assert no_signal_item["confidence"] == pytest.approx(0.25)  # repeat 2/4 * signal 2/4
    assert no_signal_item["confidence"] < full_item["confidence"]


# --- LS branch routing ----------------------------------------------------

def _make_oversteer_evidence(corner=4, phase="exit_4", speed_class="high"):
    return {"type": "corner_verdict", "corner": corner, "phase": phase, "speed_class": speed_class,
            "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}


def _make_ls_evidence(ls_class, corner=4, phase="exit_4"):
    return {"type": "ls_disambiguation", "corner": corner, "phase": phase, "speed_class": "high",
            "verdict": "oversteer", "severity": "moderate", "confidence": 0.5,
            "ls_class": ls_class, "source": "test"}


def test_ls_branch_routing_cornering_limited_only_arb_family():
    from modules.recommendation import load_setup_parameters_registry
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence(), _make_ls_evidence("cornering_limited")]
    candidates = generate_candidates(evidence, registry, config)
    assert {c["lever_family"] for c in candidates} == {"arb_spring"}


def test_ls_branch_routing_traction_limited_only_diff_tc_family():
    from modules.recommendation import load_setup_parameters_registry
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence(), _make_ls_evidence("traction_limited")]
    candidates = generate_candidates(evidence, registry, config)
    assert {c["lever_family"] for c in candidates} == {"diff_tc"}


def test_ls_branch_routing_no_disambiguation_generates_both():
    from modules.recommendation import load_setup_parameters_registry
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence()]  # no ls_disambiguation evidence at all
    candidates = generate_candidates(evidence, registry, config)
    assert {c["lever_family"] for c in candidates} == {"arb_spring", "diff_tc"}


# --- Regression: springs_rear_soften missing "delta" (found 2026-09-20) --
#
# _settings_window_component does float(current) + action["delta"] once
# parameter_windows has a real nominal/span for the parameter (springs_rear
# does) -- the hardcoded springs_rear_soften action had no "delta" key at
# all, a latent KeyError on any real setup sheet with a springs_rear value
# on record. Never caught by test_end_to_end_real_dubai (that fixture's own
# session has no current_setup wired through score()) or by any other
# existing test (all call score() with current_setup=None, which short-
# circuits _current_setup_value before the crashing line is ever reached).
# Fixed alongside the lever_bridges package since the fix uses the exact
# delta convention (+1 stiffen / -1 soften) that package introduces.

def test_springs_rear_soften_settings_window_no_keyerror():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # springs_rear is a heavy corrector (DECISION LAYER SPEC B3) -- needs
    # strong severity plus a second same-direction corner to survive the
    # eligibility gate; corner 9 supplies the multi-corner support, only
    # corner 4's own candidate is asserted on below.
    evidence = [
        {**_make_oversteer_evidence(), "severity": "strong"},
        {**_make_oversteer_evidence(corner=9), "severity": "strong"},
    ]
    candidates = generate_candidates(evidence, registry, config)
    springs_candidate = next(c for c in candidates if c["id"] == "springs_rear_soften:C4:exit_4")
    assert springs_candidate["actions"][0]["delta"] == -1

    # Filled springs window: rear_left/rear_right.springs is a real, nonzero
    # legal value (config/setup_parameters.json springs_rear.value_space
    # options), so _current_setup_value resolves a real `current` and the
    # crashing line is actually reached.
    current_setup = {"rear_left": {"springs": 260}, "rear_right": {"springs": 260}}
    result = score(springs_candidate, evidence, current_setup, config)  # must not raise
    assert not any("springs_rear" in flag for flag in result["flags"])
    assert result["components"]["headroom"] != 0.0


# --- End to end: real Dubai analysis -> frame output ---------------------

def test_end_to_end_real_dubai(pipeline_result):
    """Real Dubai session through the full evidence -> candidate -> scoring
    chain, under whatever sideslip_source the live config carries (same
    fixture every other golden test uses, currently ekf_auto_pacejka).
    Deliberately no hard-coded corner/count assertions -- the work order's
    own framing: with the newly anchored thresholds the evidence may be
    sparse, and a near-empty shortlist is a valid, honestly-reported
    result, not a test failure. Structural invariants only, plus a
    determinism check on the full real output.
    """
    from modules.recommendation import load_setup_parameters_registry

    summaries = pipeline_result["summaries"]
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    ls_stats = aggregate_ls_by_corner(summaries)

    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    for e in evidence:
        assert 0.0 <= e["confidence"] <= 1.0
        # Stage 2 (2026-09-04) added matrix_verdict (the 39-rule migration
        # bridge's own evidence source); intervention_abs/intervention_tc
        # never appear here since this call passes no corners/state/
        # channels (build_evidence's own Stage-1-compatible default).
        assert e["type"] in ("corner_verdict", "ls_disambiguation", "plausibility_brake_balance",
                              "matrix_verdict")

    candidates = generate_candidates(evidence, registry, config)
    for c in candidates:
        assert c["grade"] in ("derived-from-matrix", "proposed")

    shortlist_a = generate_shortlist(candidates, evidence, None, config)
    shortlist_b = generate_shortlist(candidates, evidence, None, config)
    assert [c["id"] for c in shortlist_a] == [c["id"] for c in shortlist_b]
    assert [c["score"] for c in shortlist_a] == [c["score"] for c in shortlist_b]

    print(f"\n[decision_frame end-to-end, real Dubai] evidence={len(evidence)} "
          f"candidates={len(candidates)} shortlist={len(shortlist_a)}")


# ==========================================================================
# Stage 2 (Frame-Stage-2 Phase 3f, 2026-09-04): migration completeness,
# rule-bridge candidates, intervention-evidence off/on, conflict resolver.
# ==========================================================================

# --- Migration completeness: all 39 rules accounted -----------------------

def test_migration_completeness_all_39_accounted():
    rec_config = load_recommendations_config()
    rules = rec_config["rules"]
    assert len(rules) == 39

    counts = {}
    for r in rules:
        status = rule_bridge_status(r)
        assert status in ("primary", "secondary(held)", "inactive(dropped)",
                           "inactive(retired)", "inactive(other-status)", "non-matrix(trigger)")
        counts[status] = counts.get(status, 0) + 1

    # Real counts, verified against config/recommendations.json directly
    # (2026-09-04 census): 7 pre-matrix seed rules (status=retired), 2
    # dropped matrix cells (OS-BRK-low, INST-ENT), 4 held escalations, 26
    # live "elicited" matrix rules.
    assert counts.get("inactive(retired)", 0) == 7
    assert counts.get("inactive(dropped)", 0) == 2
    assert counts.get("secondary(held)", 0) == 4
    assert counts.get("primary", 0) == 26
    assert sum(counts.values()) == 39


# --- Rule-bridge candidate generation --------------------------------------

def _matrix_verdict_evidence(corner, phases, verdict, severity, speed_class, confidence=0.5):
    return {"type": "matrix_verdict", "corner": corner, "phases": tuple(phases),
            "verdict": verdict, "severity": severity, "speed_class": speed_class,
            "confidence": confidence, "source": "test"}


def test_bridge_candidate_matches_matrix_us_brk_med():
    # matrix_us_brk_med: entry_1_brake, understeer, medium, min_severity
    # moderate -> damper_bump_ls_fl/fr soften 3, elicitation_provenance
    # "engineer-verbatim" (config/recommendations.json, verified directly).
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]
    assert len(matches) == 1
    c = matches[0]
    assert c["corner"] == 6
    assert c["grade"] == "derived-from-matrix"
    assert c["cell_id"] == "US-BRK-med"
    assert c["effect_class"] == "primary"
    params = {a["parameter"] for a in c["actions"]}
    assert params == {"damper_bump_ls_fl", "damper_bump_ls_fr"}
    assert all(a["direction"] == "soften" for a in c["actions"])


def test_bridge_candidate_absent_below_min_severity():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "normal", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]


def test_bridge_candidate_absent_wrong_speed_class():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "strong", "high")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]


def test_dropped_and_retired_rules_never_produce_candidates():
    # OS-BRK-low (dropped, matrix_os_brk_low) has suggestion=null; even
    # with matching evidence it must never appear as a candidate.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(3, ["entry_1_brake"], "oversteer", "strong", "low")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c.get("rule_id") == "matrix_os_brk_low"]


def test_held_escalation_secondary_only_alongside_base():
    # matrix_us_brk_low (base, US-BRK-low) + matrix_us_brk_low_esc (held,
    # escalation_of="US-BRK-low", action abs_position more_fa_stability).
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()

    # No matching evidence at all -- neither the base nor the held rule
    # should ever appear.
    candidates_none = generate_candidates([], registry, config)
    assert not [c for c in candidates_none if c.get("rule_id") in ("matrix_us_brk_low", "matrix_us_brk_low_esc")]

    # Base condition satisfied (entry_1_brake, understeer, low, moderate+).
    # Base rule's own action (toe_front) is a heavy corrector (DECISION
    # LAYER SPEC B3) -- needs strong severity and a second same-direction
    # corner (12) to survive the eligibility gate; only corner 2's own
    # candidates are asserted on below.
    evidence = [
        _matrix_verdict_evidence(2, ["entry_1_brake"], "understeer", "strong", "low"),
        _matrix_verdict_evidence(12, ["entry_1_brake"], "understeer", "strong", "low"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    base = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_low" and c["corner"] == 2]
    esc = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_low_esc" and c["corner"] == 2]
    assert len(base) == 1
    assert len(esc) == 1
    assert base[0]["effect_class"] == "primary"
    assert esc[0]["effect_class"] == "secondary"
    # Forced 'proposed' regardless of its own elicitation_provenance
    # ("project-lead-reviewed", which WOULD otherwise grade as
    # derived-from-matrix) -- held means not yet automated.
    rec_config = load_recommendations_config()
    held_rule = next(r for r in rec_config["rules"] if r["id"] == "matrix_us_brk_low_esc")
    assert held_rule["elicitation_provenance"] == "project-lead-reviewed"
    assert esc[0]["grade"] == "proposed"
    assert esc[0]["actions"][0]["parameter"] == "abs_position"


# --- Generic lever-bridge candidate mechanism (BACKLOG item H, 2026-09-20) -
#
# config/decision_frame.json's lever_bridges list, consumed generically by
# _bridge_candidates_for_levers -- see that function's own comment and the
# config key's _comment_lever_bridges for the schema and the springs_rear/
# soften/exit dedupe rule.

def test_lever_bridges_schema_grade_always_proposed():
    # Structural cap: no lever_bridges entry may declare a grade other than
    # "proposed" -- none carries a real matrix cell_id, so nothing should be
    # able to promote one to action-eligible via a config edit alone. The
    # generator itself hardcodes grade="proposed" regardless of this field
    # (test_lever_bridge_* below confirm that); this test guards the
    # config's own self-documentation from drifting out of sync with it.
    config = load_decision_frame_config()
    bridges = config["lever_bridges"]
    # DECISION LAYER SPEC B7 (2026-09-22) added diff_position/increase
    # (braking-phase instability), alongside the 4 springs entries BACKLOG
    # item H originally shipped; Phase C added the two splitter_offset
    # entries once the author resolved its direction sign convention.
    assert len(bridges) == 7
    assert {(b["lever"], b["direction"]) for b in bridges} == {
        ("springs_front", "soften"), ("springs_front", "stiffen"),
        ("springs_rear", "stiffen"), ("springs_rear", "soften"),
        ("diff_position", "increase"),
        ("splitter_offset", "decrease"), ("splitter_offset", "increase"),
    }
    for b in bridges:
        assert b["grade"] == "proposed"


def test_lever_bridge_springs_front_soften_fires_on_understeer():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # springs_front is a heavy corrector (DECISION LAYER SPEC B3) -- needs
    # strong severity plus a second same-direction corner (9) to survive
    # the eligibility gate; only corner 7's own candidate is asserted on.
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1
    assert matches[0]["grade"] == "proposed"
    assert matches[0]["actions"] == [{"parameter": "springs_front", "direction": "soften", "delta": -1}]


def test_lever_bridge_springs_front_stiffen_fires_on_oversteer():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "oversteer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "oversteer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:stiffen:C7:apex_3"]
    assert len(matches) == 1
    assert matches[0]["actions"] == [{"parameter": "springs_front", "direction": "stiffen", "delta": 1}]


def test_lever_bridge_corrected_acceptance_springs_rear_stiffen_on_understeer():
    # Corrected acceptance case (2026-09-20 work order): the original BACKLOG
    # item H acceptance line paired "springs_rear stiffen" with a synthetic
    # ENTRY-OVERSTEER case -- physically backwards per config/decision_frame.
    # json's own interaction_table (springs_rear stiffen carries sign=-1 on
    # oversteer_tendency, i.e. WORSENS oversteer; sign=+1 on understeer_
    # tendency, i.e. HELPS understeer -- rear roll stiffness up shifts
    # balance toward oversteer, not away from it). The genuinely new,
    # previously IMPOSSIBLE-to-write candidate this package unlocks is
    # springs_rear stiffen on an UNDERSTEER case -- no code path before this
    # package could ever produce it (only soften had a bridge, hardcoded in
    # _exit_oversteer_candidates, and only for oversteer).
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # springs_rear is a heavy corrector (DECISION LAYER SPEC B3) -- strong
    # severity plus a second same-direction corner (12) needed to survive
    # the eligibility gate.
    evidence = [
        _matrix_verdict_evidence(7, ["entry_2_turnin"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(12, ["entry_2_turnin"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_rear:stiffen:C7:entry_2_turnin"]
    assert len(matches) == 1
    c = matches[0]
    assert c["grade"] == "proposed"
    assert c["actions"] == [{"parameter": "springs_rear", "direction": "stiffen", "delta": 1}]


def test_lever_bridge_springs_rear_soften_fires_at_turnin_no_hardcoded_equivalent():
    # No dedupe collision -- _exit_oversteer_candidates only ever fires at
    # EXIT_PHASES, never turn-in, so this is genuinely new territory the
    # hardcoded path never covered. springs_rear is a heavy corrector
    # (B3) -- strong severity plus a second same-direction corner needed.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(9, ["entry_2_turnin"], "oversteer", "strong", "low"),
        _matrix_verdict_evidence(13, ["entry_2_turnin"], "oversteer", "strong", "low"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_rear:soften:C9:entry_2_turnin"]
    assert len(matches) == 1


def test_lever_bridge_dedupe_springs_rear_soften_exit_oversteer_hardcoded_wins():
    # springs_rear/soften/oversteer at exit_4+exit_5 OVERLAPS the hardcoded
    # _exit_oversteer_candidates' own springs_rear_soften secondary
    # candidate. Both corner_verdict (drives the hardcoded path) and
    # matrix_verdict (drives the generic path) evidence present at the same
    # corner/phase -- the real-session collision shape named in the work
    # order. The hardcoded path must win: richer, LS-disambiguation-aware
    # evidence, generated first.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # springs_rear is a heavy corrector (DECISION LAYER SPEC B3) -- corner
    # 8 supplies the second same-direction corner the eligibility gate
    # needs (its own springs_rear_soften:C8:exit_4 candidate, hardcoded
    # path, not otherwise asserted on); both corner-4 evidence items
    # bumped to strong so the multi-corner group actually forms.
    evidence = [
        {**_make_oversteer_evidence(corner=4, phase="exit_4"), "severity": "strong"},
        _matrix_verdict_evidence(4, ["exit_4", "exit_5"], "oversteer", "strong", "high"),
        {**_make_oversteer_evidence(corner=8, phase="exit_4"), "severity": "strong"},
    ]
    candidates = generate_candidates(evidence, registry, config)
    springs_rear_candidates = [c for c in candidates
                                if any(a["parameter"] == "springs_rear" for a in c["actions"])
                                and c["corner"] == 4]
    assert len(springs_rear_candidates) == 1
    assert springs_rear_candidates[0]["id"] == "springs_rear_soften:C4:exit_4"
    assert springs_rear_candidates[0]["scenario"] == "exit_oversteer"


def test_lever_bridges_absent_when_config_empty():
    config = copy.deepcopy(load_decision_frame_config())
    config["lever_bridges"] = []
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["id"].startswith("lever_bridge:")]


# --- FRAME DEPTH PROGRAMME Step 1: condition schema -------------------------
#
# evaluate_conditions() is a pure function -- most paths are tested directly
# against it (fast, precise). Two integration tests confirm the wiring into
# _bridge_candidates_for_levers/generate_candidates actually suppresses/caps
# a real candidate, not just that the pure function returns the right enum.

def test_condition_all_pass():
    from modules.decision_frame import evaluate_conditions
    conditions = [{"type": "phase_transient", "required": True}]
    verdict, reasons = evaluate_conditions(conditions, 4, "entry_2_turnin", [], None, {})
    assert verdict == "PASS"
    assert reasons == []


def test_condition_required_fail_suppresses():
    from modules.decision_frame import evaluate_conditions
    conditions = [{"type": "phase_transient", "required": True}]
    verdict, reasons = evaluate_conditions(conditions, 4, "apex_3", [], None, {})
    assert verdict == "SUPPRESS"
    assert reasons and "apex_3" in reasons[0]


def test_condition_non_required_fail_caps():
    from modules.decision_frame import evaluate_conditions
    conditions = [{"type": "phase_transient", "required": False}]
    verdict, reasons = evaluate_conditions(conditions, 4, "apex_3", [], None, {})
    assert verdict == "CAP_ADVISORY"
    assert reasons and "apex_3" in reasons[0]


def test_condition_not_evaluable_evidence_type_absent_caps():
    from modules.decision_frame import evaluate_conditions
    conditions = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                   "presence": "present", "required": False}]
    # evidence_items carries OTHER types but never damper_motion at all --
    # the evidence source itself was never built this run, not merely
    # silent at this corner/phase.
    evidence_items = [{"type": "corner_verdict", "corner": 4, "phase": "entry_2_turnin"}]
    verdict, reasons = evaluate_conditions(conditions, 4, "entry_2_turnin", evidence_items, None, {})
    assert verdict == "CAP_ADVISORY"
    assert "no damper_motion evidence available" in reasons[0]


def test_condition_not_evaluable_setup_data_none_caps():
    from modules.decision_frame import evaluate_conditions
    registry = load_setup_parameters_registry()
    conditions = [{"type": "setup_state", "parameter": "toe_front", "check": "within_window",
                   "required": False}]
    verdict, reasons = evaluate_conditions(conditions, 4, "entry_2_turnin", [], None, registry)
    assert verdict == "CAP_ADVISORY"
    assert "setup sheet unfilled: toe_front" in reasons[0]


def test_condition_not_evaluable_registry_window_missing_caps():
    from modules.decision_frame import evaluate_conditions
    registry = load_setup_parameters_registry()
    # abs_position's own parameter_windows entry is nominal=null/span=null
    # (categorical, direction_semantics.type is explicitly non-monotonic --
    # config/decision_frame.json's own note on this parameter).
    conditions = [{"type": "setup_state", "parameter": "abs_position", "check": "within_window",
                   "required": False}]
    setup_data = {"electronics": {"abs_position": 5}}
    verdict, reasons = evaluate_conditions(conditions, 4, "entry_1_brake", [], setup_data, registry)
    assert verdict == "CAP_ADVISORY"
    assert "no settings window: abs_position" in reasons[0]


def test_condition_phase_transient_passes_exit_fails_apex():
    from modules.decision_frame import evaluate_conditions
    conditions = [{"type": "phase_transient", "required": True}]
    verdict, _ = evaluate_conditions(conditions, 4, "exit_4", [], None, {})
    assert verdict == "PASS"
    verdict, _ = evaluate_conditions(conditions, 4, "apex_3", [], None, {})
    assert verdict == "SUPPRESS"


def test_condition_evidence_corroboration_present_and_absent():
    from modules.decision_frame import evaluate_conditions
    damper_ev = {"type": "damper_motion", "corner": 4, "phase": "entry_2_turnin"}
    present_cond = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                      "presence": "present", "required": False}]
    absent_cond = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                     "presence": "absent", "required": False}]
    # Evidence type WAS built this run (damper_ev exists somewhere), and a
    # matching item exists at this exact corner/phase.
    verdict, _ = evaluate_conditions(present_cond, 4, "entry_2_turnin", [damper_ev], None, {})
    assert verdict == "PASS"
    verdict, reasons = evaluate_conditions(absent_cond, 4, "entry_2_turnin", [damper_ev], None, {})
    assert verdict == "CAP_ADVISORY"  # non-required presence violated
    # Evidence type built, but nothing at THIS corner -- a real, evaluable absence.
    verdict, reasons = evaluate_conditions(present_cond, 9, "entry_2_turnin", [damper_ev], None, {})
    assert verdict == "CAP_ADVISORY"
    assert "no damper_motion evidence at C9 entry_2_turnin" in reasons[0]
    verdict, _ = evaluate_conditions(absent_cond, 9, "entry_2_turnin", [damper_ev], None, {})
    assert verdict == "PASS"


def test_condition_no_conditions_key_byte_identical_to_today():
    # Entry with no "conditions" key at all -- the exact shape every one of
    # the 4 shipped lever_bridges entries has today. Confirms generate_
    # candidates' new setup_data parameter and the evaluate_conditions call
    # it now makes are completely inert for a bridge that doesn't opt in.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    assert all("conditions" not in b for b in config["lever_bridges"])
    # springs_front is a heavy corrector (DECISION LAYER SPEC B3) -- strong
    # severity plus a second same-direction corner needed to survive the
    # eligibility gate.
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1
    assert matches[0]["evidence_refs"] == [evidence[0]]  # no synthetic condition_gap item appended
    assert matches[0]["condition_reasons"] == []


# --- Integration: conditions actually suppress/cap a real candidate --------

def test_condition_integration_required_condition_suppresses_real_candidate():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    # springs_front/soften normally fires at apex_3 too (see the config's
    # own phase_groups) -- attach a required phase_transient condition and
    # confirm the apex_3 candidate specifically disappears while another
    # phase (entry_2_turnin, a real transient) is unaffected.
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "phase_transient", "required": True}]
    # springs_front is a heavy corrector (DECISION LAYER SPEC B3) -- strong
    # severity plus a second same-direction corner (9, entry_2_turnin)
    # needed for the surviving entry_2_turnin candidate to clear the
    # eligibility gate; apex_3 is suppressed by the phase_transient
    # condition itself regardless of the gate.
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(7, ["entry_2_turnin"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["entry_2_turnin"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    ids = {c["id"] for c in candidates}
    assert "lever_bridge:springs_front:soften:C7:apex_3" not in ids
    assert "lever_bridge:springs_front:soften:C7:entry_2_turnin" in ids


def test_condition_integration_not_evaluable_caps_real_candidate_confidence():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    cap = config["conditions"]["not_evaluable_confidence_cap"]
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                                 "presence": "present", "required": False}]
    # springs_front is a heavy corrector (DECISION LAYER SPEC B3) -- strong
    # severity plus a second same-direction corner needed to survive the
    # eligibility gate.
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium", confidence=0.9),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1
    c = matches[0]
    assert c["condition_reasons"] == ["no damper_motion evidence available this run"]
    # DECISION LAYER SPEC B1: a CAP_ADVISORY condition marks the candidate
    # not_assessable in the lever inventory, not just a lowered confidence.
    assert c["status"] == "not_assessable"
    gap_items = [e for e in c["evidence_refs"] if e["type"] == "condition_gap"]
    assert len(gap_items) == 1
    assert gap_items[0]["confidence"] == pytest.approx(cap)
    # The candidate's own overall confidence (MIN across evidence_refs) is
    # pulled DOWN to the cap even though the firing evidence itself was 0.9
    # -- min() never inflates, only lowers.
    from modules.decision_frame import _candidate_confidence
    assert _candidate_confidence(c) == pytest.approx(cap)


# --- Intervention evidence, off/on -----------------------------------------

def _synthetic_state_channels(n=200, sample_rate_hz=50.0):
    t = np.arange(n) / sample_rate_hz
    state = {"time": t}
    return state, t


_ABS_TEST_CONFIG = {
    "confidence": 0.8,
    "abs_heavy_masks_verdict": {"heavy_duty_cycle_threshold": 0.5},
}


def test_intervention_evidence_per_source_defaults():
    # Deepening Phase 4c (2026-09-18, user decision): ABS defaults ON, TC
    # stays dormant -- replaces Stage 2's own single global flag.
    config = load_decision_frame_config()
    assert config["intervention_evidence"]["abs"]["enabled"] is True
    assert config["intervention_evidence"]["tc"]["enabled"] is False
    assert config["intervention_evidence"]["abs"]["confidence"] == pytest.approx(0.8)


def test_intervention_abs_evidence_fires_via_build_evidence_default_config():
    # End-to-end through build_evidence with the REAL, unmodified live
    # config (abs.enabled=True by default) -- not just the unit-level
    # _build_intervention_abs_evidence tests above.
    config = load_decision_frame_config()
    state, t = _synthetic_state_channels()
    channels = {"abs_active": {"time": t, "data": np.zeros(len(t)), "quality": "valid"}}
    corners = [
        {"stable_corner_id": 5, "lap_number": 1, "segments": {"entry_1_brake": (0.5, 1.5)}},
        {"stable_corner_id": 5, "lap_number": 2, "segments": {"entry_1_brake": (0.5, 1.5)}},
    ]
    evidence = build_evidence([], {}, config, lambda s: ("normal", "", "", ""),
                               corners=corners, state=state, channels=channels)
    assert [e for e in evidence if e["type"] == "intervention_abs" and e["corner"] == 5]


def test_intervention_evidence_tc_off_by_default_via_build_evidence():
    config = load_decision_frame_config()
    state, t = _synthetic_state_channels()
    channels = {
        "abs_active": {"time": t, "data": np.zeros(len(t)), "quality": "valid"},
        "ecu_B_tc_act": {"time": t, "data": np.ones(len(t)), "quality": "valid"},
    }
    corners = [{"stable_corner_id": 1, "lap_number": 1, "segments": {"entry_1_brake": (0.0, 1.0),
                                                                      "exit_4": (1.0, 1.5), "exit_5": (1.5, 2.0)}}]
    evidence = build_evidence([], {}, config, lambda s: ("normal", "", "", ""),
                               corners=corners, state=state, channels=channels)
    # TC stays dormant -- never fires even with a channel that would
    # otherwise trigger it, since intervention_evidence.tc.enabled=False.
    assert not [e for e in evidence if e["type"] == "intervention_tc"]


def test_intervention_abs_evidence_fires_when_enabled():
    state, t = _synthetic_state_channels()
    # abs_active = 0 for the whole session -- "inactive throughout" for any
    # phase window drawn from it.
    channels = {"abs_active": {"time": t, "data": np.zeros(len(t)), "quality": "valid"}}
    corners = [
        {"stable_corner_id": 5, "lap_number": 1, "segments": {"entry_1_brake": (0.5, 1.5)}},
        {"stable_corner_id": 5, "lap_number": 2, "segments": {"entry_1_brake": (0.5, 1.5)}},
    ]
    aggregated = {5: {"speed_class": "medium"}}
    from modules.decision_frame import _build_intervention_abs_evidence
    evidence = _build_intervention_abs_evidence(corners, {}, state, channels, aggregated, _ABS_TEST_CONFIG)
    inactive_ev = [e for e in evidence if e["type"] == "intervention_abs"]
    assert len(inactive_ev) == 1
    ev = inactive_ev[0]
    assert ev["corner"] == 5
    # Deepening Phase 4c: confidence is capped at abs_config["confidence"]
    # (0.8), not the raw repeat fraction (1.0 here, inactive on both of
    # the 2 analysed instances) -- "supersedes Stage 2's own autonomous
    # 1.0 default" per the user's own decision.
    assert ev["confidence"] == pytest.approx(0.8)


def test_intervention_abs_evidence_confidence_caps_not_inflates():
    # The 0.8 figure is a CEILING on the real repeat-fraction signal, not
    # a replacement for it -- a weaker repeat pattern must still report
    # honestly below 0.8, never inflated up to it.
    state, t = _synthetic_state_channels(n=400)
    channels = {"abs_active": {"time": t, "data": np.zeros(len(t)), "quality": "valid"}}
    corners = [
        {"stable_corner_id": 5, "lap_number": 1, "segments": {"entry_1_brake": (0.5, 1.5)}},
        {"stable_corner_id": 5, "lap_number": 2, "segments": {"entry_1_brake": (2.5, 3.5)}},
        {"stable_corner_id": 5, "lap_number": 3, "segments": {"entry_1_brake": (4.5, 5.5)}},
        {"stable_corner_id": 5, "lap_number": 4, "segments": {"entry_1_brake": (6.5, 7.5)}},
    ]
    aggregated = {5: {"speed_class": "medium"}}
    from modules.decision_frame import _build_intervention_abs_evidence
    evidence = _build_intervention_abs_evidence(corners, {}, state, channels, aggregated, _ABS_TEST_CONFIG)
    inactive_ev = [e for e in evidence if e["type"] == "intervention_abs"]
    assert len(inactive_ev) == 1
    # abs_active is 0 everywhere -- all 4 instances are "inactive", so the
    # raw repeat fraction really is 1.0, capped down to 0.8. Confirms the
    # cap direction (only ever lowers, matching min()) via the OTHER
    # existing test above; this test's own job is just to not regress
    # that with a differently-shaped fixture (more instances).
    assert inactive_ev[0]["confidence"] == pytest.approx(0.8)


def test_intervention_abs_evidence_absent_when_abs_fires_lightly():
    state, t = _synthetic_state_channels()
    data = np.zeros(len(t))
    data[50:70] = 1.0  # ABS fires inside the entry_1_brake window below, 20/50 samples = 40% duty cycle
    channels = {"abs_active": {"time": t, "data": data, "quality": "valid"}}
    corners = [{"stable_corner_id": 5, "lap_number": 1, "segments": {"entry_1_brake": (0.5, 1.5)}}]
    from modules.decision_frame import _build_intervention_abs_evidence
    evidence = _build_intervention_abs_evidence(corners, {}, state, channels, {5: {"speed_class": "medium"}},
                                                  _ABS_TEST_CONFIG)
    # Neither fully inactive (ABS did fire) nor heavy (40% < the 50% test
    # threshold) -- correctly produces no evidence of either type.
    assert evidence == []


def test_intervention_abs_masking_fires_on_heavy_duty_cycle():
    # NEW, Deepening Phase 4c: "ABS regulating heavily -> flag as masking".
    state, t = _synthetic_state_channels()
    data = np.zeros(len(t))
    data[25:65] = 1.0  # 40/50 samples = 80% duty cycle within the window below, >= 50% threshold
    channels = {"abs_active": {"time": t, "data": data, "quality": "valid"}}
    corners = [{"stable_corner_id": 5, "lap_number": 1, "segments": {"entry_1_brake": (0.5, 1.5)}}]
    from modules.decision_frame import _build_intervention_abs_evidence
    evidence = _build_intervention_abs_evidence(corners, {}, state, channels, {5: {"speed_class": "medium"}},
                                                  _ABS_TEST_CONFIG)
    masking_ev = [e for e in evidence if e["type"] == "intervention_abs_masking"]
    assert len(masking_ev) == 1
    ev = masking_ev[0]
    assert ev["corner"] == 5
    assert ev["verdict"] is None  # a masking flag, not a verdict-corroborating item
    assert ev["masked_by_heavy_abs"] is True
    assert ev["confidence"] == pytest.approx(0.8)  # capped, same as the inactive-corroboration entry
    # Must not ALSO report inactive-corroboration for the same instance --
    # exactly one of the two conditions can hold per instance.
    assert not [e for e in evidence if e["type"] == "intervention_abs"]


def test_intervention_evidence_skipped_without_raw_inputs():
    # build_evidence's own Stage-1-compatible default (corners/state/
    # channels all None) must never attempt intervention evidence, flag
    # or no flag -- summaries-only callers (every existing Stage 1 test)
    # must be completely unaffected.
    config = load_decision_frame_config()
    evidence = build_evidence([], {}, config, lambda s: ("normal", "", "", ""))
    assert not [e for e in evidence if e["type"].startswith("intervention_")]


# --- Driver-feedback magnitude weighting (Deepening Phase 4d) -------------

def _feedback_data_for(cid, **phase_values):
    corners = [{} for _ in range(cid)]
    corners[cid - 1] = dict(phase_values)
    return {"corners": corners}


_BAND_CFG = {"confidence_bands": [
    {"max_abs": 1, "value": 0.1}, {"max_abs": 3, "value": 0.75}, {"max_abs": 5, "value": 1.0},
]}
# The formula ITEM 2(c) replaced (linear ramp, floor=0.1, full_at=4) -- kept
# here ONLY as a literal comparison point for the "band beats old linear"
# regression test below, never imported from production (that formula no
# longer exists in modules/decision_frame.py).
_OLD_LINEAR = lambda magnitude, floor=0.1, full_at=4: round(
    floor + (1.0 - floor) * min(1.0, max(0.0, (magnitude - 1.0) / (full_at - 1.0))), 3
)


def test_driver_feedback_confidence_floor_at_magnitude_1():
    from modules.decision_frame import _build_driver_feedback_evidence
    feedback_data = _feedback_data_for(4, x4=1)  # smallest possible complaint, exit_4
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, _BAND_CFG)
    assert len(evidence) == 1
    ev = evidence[0]
    assert ev["corner"] == 4 and ev["phase"] == "exit_4"
    assert ev["verdict"] == "oversteer"  # positive raw value
    assert ev["confidence"] == pytest.approx(0.1)  # low band, unchanged from the old floor


def test_driver_feedback_confidence_mid_band_flat_at_2_and_3():
    # Phase D ITEM 2(c) (2026-09-23): band-shaped, not linear -- |2| and |3|
    # both land in the same "clearly felt" band and must read the SAME
    # confidence, unlike the old linear ramp's own 0.4/0.7 split.
    from modules.decision_frame import _build_driver_feedback_evidence
    fb2 = _feedback_data_for(4, a3=2)
    fb3 = _feedback_data_for(4, a3=-3)
    ev2 = _build_driver_feedback_evidence(fb2, {4: {"speed_class": "medium"}}, _BAND_CFG)
    ev3 = _build_driver_feedback_evidence(fb3, {4: {"speed_class": "medium"}}, _BAND_CFG)
    assert ev2[0]["confidence"] == pytest.approx(0.75)
    assert ev3[0]["confidence"] == pytest.approx(0.75)


def test_driver_feedback_confidence_undrivable_band_saturates_at_4_and_5():
    from modules.decision_frame import _build_driver_feedback_evidence
    fb4 = _feedback_data_for(4, e1=-4)
    fb5 = _feedback_data_for(4, e1=-5)
    ev4 = _build_driver_feedback_evidence(fb4, {4: {"speed_class": "medium"}}, _BAND_CFG)
    ev5 = _build_driver_feedback_evidence(fb5, {4: {"speed_class": "medium"}}, _BAND_CFG)
    assert ev4[0]["confidence"] == pytest.approx(1.0)
    assert ev5[0]["verdict"] == "understeer"
    assert ev5[0]["confidence"] == pytest.approx(1.0)  # |5| = 1.0 exactly, per the work order


def test_driver_feedback_confidence_band_beats_old_linear_in_mid_band():
    # "Proportionate feedback strengthening": the new band value at |2| and
    # |3| must be >= what the old linear ramp gave at that same input --
    # a clearly-felt complaint now carries at least as much weight as
    # before, never less. |4|/|5| already saturated under the old formula
    # too (both already 1.0), so the strengthening is entirely in this
    # mid band -- the regression this test actually guards.
    from modules.decision_frame import _feedback_confidence
    for magnitude in (2, 3):
        assert _feedback_confidence(magnitude, _BAND_CFG) >= _OLD_LINEAR(magnitude)
    for magnitude in (4, 5):
        assert _feedback_confidence(magnitude, _BAND_CFG) == _OLD_LINEAR(magnitude) == 1.0


def test_driver_feedback_zero_value_produces_no_evidence():
    from modules.decision_frame import _build_driver_feedback_evidence
    feedback_data = _feedback_data_for(4, e1=0, x4=0)
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, _BAND_CFG)
    assert evidence == []


def test_build_evidence_skips_feedback_without_feedback_data():
    config = load_decision_frame_config()
    evidence = build_evidence([], {}, config, lambda s: ("normal", "", "", ""))
    assert not [e for e in evidence if e["type"] == "driver_feedback"]


def test_attach_feedback_evidence_matches_corner_phase_verdict():
    from modules.decision_frame import _attach_feedback_evidence
    matrix_ev = {"type": "matrix_verdict", "corner": 6, "phases": ("exit_4", "exit_5"),
                 "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    feedback_ev = {"type": "driver_feedback", "corner": 6, "phase": "exit_4", "verdict": "oversteer",
                   "severity": None, "confidence": 0.2, "raw_feedback": 1, "source": "test"}
    candidate = {"id": "c1", "corner": 6, "phase": "exit_5", "phases": ("exit_4", "exit_5"),
                 "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
                 "evidence_refs": [matrix_ev]}
    result = _attach_feedback_evidence([candidate], [matrix_ev, feedback_ev])
    assert feedback_ev in result[0]["evidence_refs"]
    assert len(result[0]["evidence_refs"]) == 2


def test_attach_feedback_evidence_skips_mismatched_verdict():
    from modules.decision_frame import _attach_feedback_evidence
    matrix_ev = {"type": "matrix_verdict", "corner": 6, "phases": ("exit_4", "exit_5"),
                 "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    feedback_ev = {"type": "driver_feedback", "corner": 6, "phase": "exit_4", "verdict": "understeer",
                   "severity": None, "confidence": 0.9, "raw_feedback": -1, "source": "test"}
    candidate = {"id": "c1", "corner": 6, "phase": "exit_5", "phases": ("exit_4", "exit_5"),
                 "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
                 "evidence_refs": [matrix_ev]}
    result = _attach_feedback_evidence([candidate], [matrix_ev, feedback_ev])
    assert result[0]["evidence_refs"] == [matrix_ev]


def test_attach_feedback_evidence_never_duplicates():
    from modules.decision_frame import _attach_feedback_evidence
    matrix_ev = {"type": "matrix_verdict", "corner": 6, "phases": ("exit_4",),
                 "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    feedback_ev = {"type": "driver_feedback", "corner": 6, "phase": "exit_4", "verdict": "oversteer",
                   "severity": None, "confidence": 0.2, "raw_feedback": 1, "source": "test"}
    candidate = {"id": "c1", "corner": 6, "phase": "exit_4", "phases": ("exit_4",),
                 "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
                 "evidence_refs": [matrix_ev]}
    result = _attach_feedback_evidence([candidate], [matrix_ev, feedback_ev])
    result = _attach_feedback_evidence(result, [matrix_ev, feedback_ev])  # idempotent on a second pass
    assert result[0]["evidence_refs"].count(feedback_ev) == 1


def test_feedback_weighting_config_present():
    # Phase D ITEM 2(c) (2026-09-23): band-shaped config, replacing the
    # original confidence_floor/full_confidence_at_raw_abs pair.
    config = load_decision_frame_config()
    fw = config["driver_feedback_weighting"]
    bands = fw["confidence_bands"]
    assert [b["max_abs"] for b in bands] == [1, 3, 5]
    assert [b["value"] for b in bands] == [pytest.approx(0.1), pytest.approx(0.75), pytest.approx(1.0)]
    # Same anchor modules.recommendation's own consistency-gate override
    # uses (|4|="approaching undrivable") must still fall inside the
    # undrivable band (max_abs=5), past the mid band's own max_abs=3.
    rec_config = load_recommendations_config()
    anchor = rec_config["settings"]["consistency_gate"]["feedback_override"]["feedback_override_raw_min"]
    assert bands[1]["max_abs"] < anchor <= bands[2]["max_abs"]


# --- Metrology Phase 2: verdict-stability [MARGINAL] confidence cap --------

def test_corner_verdict_evidence_marginal_caps_confidence():
    # CSf=-0.15 sits 0.05 from STRONG_CSF(-0.10), inside the anchored 0.11
    # margin -- classify_fn tags it [MARGINAL]; every lap agrees (repeat=
    # total), so the plain repeat-fraction confidence would be 1.0 -- it
    # must be capped at intervention_evidence.abs.confidence (reused
    # anchor, not a new constant), not merely reduced by some amount.
    # exit_4, not apex_3: apex_3 rides on aggregate_by_corner's own
    # apex_region substitution, which this synthetic fixture (apex_region
    # always None per lap) aggregates into a NaN-filled dict rather than
    # None, masking the phase's own cs_ratio_f -- an unrelated pre-existing
    # fixture quirk, sidestepped by testing on a phase apex_region never
    # touches.
    config = load_decision_frame_config()
    cap = config["intervention_evidence"]["abs"]["confidence"]
    summaries = [
        _make_summary(lap, 7, "medium", csf_by_phase={"exit_4": -0.15})
        for lap in range(1, 5)
    ]
    evidence = build_evidence(summaries, {}, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "corner_verdict"
               and e["corner"] == 7 and e["phase"] == "exit_4"]
    assert matches
    assert matches[0]["marginal"] is True
    assert matches[0]["confidence"] == pytest.approx(cap)


def test_corner_verdict_evidence_not_marginal_stays_uncapped():
    # CSf=-0.5 sits 0.40 from STRONG_CSF -- well outside the margin, a
    # confident verdict. All 4 laps agree -> repeat-fraction confidence
    # 1.0, untouched by the cap.
    config = load_decision_frame_config()
    summaries = [
        _make_summary(lap, 8, "medium", csf_by_phase={"exit_4": -0.5})
        for lap in range(1, 5)
    ]
    evidence = build_evidence(summaries, {}, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "corner_verdict"
               and e["corner"] == 8 and e["phase"] == "exit_4"]
    assert matches
    assert matches[0]["marginal"] is False
    assert matches[0]["confidence"] == pytest.approx(1.0)


def test_matrix_verdict_evidence_marginal_also_caps_confidence():
    # Same cap must apply to matrix_verdict evidence (the type that
    # actually feeds most of the 39-rule matrix candidates), not just
    # corner_verdict -- otherwise a MARGINAL verdict would still drive a
    # full-confidence recommendation via the matrix path.
    config = load_decision_frame_config()
    cap = config["intervention_evidence"]["abs"]["confidence"]
    summaries = [
        _make_summary(lap, 9, "medium", csf_by_phase={"exit_4": -0.15, "exit_5": -0.15})
        for lap in range(1, 5)
    ]
    evidence = build_evidence(summaries, {}, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "matrix_verdict" and e["corner"] == 9]
    assert matches
    assert all(e["marginal"] is True for e in matches)
    assert all(e["confidence"] == pytest.approx(cap) for e in matches)


# --- LS threshold decision (2026-09-20, user + reviewer) -------------------
# ls_threshold_evidence: an ABSOLUTE-threshold evidence source, independent
# of ls_disambiguation's own population-relative split, phase-conditioned
# confidence (braking/turn-in uncapped, exit-phase capped via the existing
# MIN-confidence mechanism). See modules/decision_frame.py's own
# _build_ls_threshold_evidence docstring and config/decision_frame.json's
# ls_threshold_evidence block for the full derivation/reopen-condition text.

def test_ls_threshold_evidence_fires_braking_uncapped():
    # STRONG_LSR=-0.60 in the real config -- entry_1_brake, rear, -0.70
    # crosses it on all 3 laps -> repeat=3/3, valid=3/3 -> confidence=1.0,
    # UNCAPPED (braking is not in exit_phases).
    config = load_decision_frame_config()
    summaries = [
        _make_summary(lap, 20, "medium", ls_r_by_phase={"entry_1_brake": -0.70})
        for lap in range(1, 4)
    ]
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 20]
    assert matches
    assert matches[0]["phase"] == "entry_1_brake"
    assert matches[0]["axle"] == "rear"
    assert matches[0]["verdict"] == "traction_limited"
    assert matches[0]["phase_scope"] == "braking_turnin"
    assert matches[0]["confidence"] == pytest.approx(1.0)


def test_ls_threshold_evidence_exit_phase_confidence_capped():
    # Same fully-repeatable (3/3) crossing, but at exit_4 -- the raw
    # repeat-fraction confidence (1.0) must be capped down to the config's
    # own exit_phase_confidence_discount (0.5), not reported as 1.0.
    config = load_decision_frame_config()
    discount = config["ls_threshold_evidence"]["exit_phase_confidence_discount"]
    summaries = [
        _make_summary(lap, 21, "medium", ls_r_by_phase={"exit_4": -0.70})
        for lap in range(1, 4)
    ]
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 21]
    assert matches
    assert matches[0]["phase_scope"] == "exit"
    assert matches[0]["confidence"] == pytest.approx(discount)


def test_ls_threshold_evidence_exit_phase_low_repeatability_not_inflated():
    # A one-off exit crossing (1 of 3 laps) must report its own honestly-low
    # fraction, never inflated UP to the discount ceiling -- min() can only
    # lower a value, never raise one.
    config = load_decision_frame_config()
    discount = config["ls_threshold_evidence"]["exit_phase_confidence_discount"]
    summaries = [_make_summary(lap, 22, "medium") for lap in range(1, 4)]
    summaries[0]["phases"]["exit_5"]["ls_ratio_r"] = _stat(-0.70, 50)
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 22]
    assert matches
    raw_fraction = 1 / 3
    assert matches[0]["confidence"] == pytest.approx(round(raw_fraction * 1.0, 3))
    assert matches[0]["confidence"] < discount


def test_ls_threshold_evidence_reproduces_c3_repeatability_pattern():
    # The census's own "C3 pattern": exit phase, fully repeatable (here
    # 4/4, matching C3's own real repeat count), zero ABS/TC involved in
    # this evidence source at all (it never reads those channels) --
    # repeatability alone lifts confidence UP TO the exit discount ceiling,
    # not left at some lower ad-hoc value.
    config = load_decision_frame_config()
    discount = config["ls_threshold_evidence"]["exit_phase_confidence_discount"]
    summaries = [
        _make_summary(lap, 3, "medium", ls_r_by_phase={"exit_5": -0.65})
        for lap in range(1, 5)
    ]
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 3
               and e["phase"] == "exit_5"]
    assert matches
    assert matches[0]["confidence"] == pytest.approx(discount)


def test_ls_threshold_evidence_absent_when_value_above_threshold():
    config = load_decision_frame_config()
    summaries = [
        _make_summary(lap, 23, "medium", ls_r_by_phase={"entry_1_brake": -0.10})
        for lap in range(1, 4)
    ]  # -0.10 does not cross STRONG_LSR=-0.60
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 23]
    assert not matches


def test_ls_threshold_evidence_absent_when_disabled():
    config = copy.deepcopy(load_decision_frame_config())
    config["ls_threshold_evidence"]["enabled"] = False
    summaries = [
        _make_summary(lap, 24, "medium", ls_r_by_phase={"entry_1_brake": -0.70})
        for lap in range(1, 4)
    ]
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    assert not [e for e in evidence if e["type"] == "ls_threshold"]


def test_ls_threshold_evidence_front_axle_verdict_is_brake_limited():
    config = load_decision_frame_config()
    summaries = [
        _make_summary(lap, 25, "medium", ls_f_by_phase={"entry_1_brake": -0.90})
        for lap in range(1, 4)
    ]  # -0.90 crosses STRONG_LSF=-0.79
    ls_stats = aggregate_ls_by_corner(summaries)
    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    matches = [e for e in evidence if e["type"] == "ls_threshold" and e["corner"] == 25]
    assert matches
    assert matches[0]["axle"] == "front"
    assert matches[0]["verdict"] == "brake_limited"
    assert matches[0]["phase_scope"] == "braking_turnin"


def test_ls_threshold_evidence_config_values_match_the_ls_evidence_census():
    config = load_decision_frame_config()
    ls_cfg = config["ls_threshold_evidence"]
    assert ls_cfg["STRONG_LSF"] == pytest.approx(-0.79)
    assert ls_cfg["STRONG_LSR"] == pytest.approx(-0.60)
    assert ls_cfg["exit_phase_confidence_discount"] == pytest.approx(0.5)
    assert set(ls_cfg["exit_phases"]) == {"exit_4", "exit_5"}
    assert set(ls_cfg["braking_turnin_phases"]) == {"entry_1_brake", "entry_2_turnin", "apex_3"}


# --- Conflict resolver -----------------------------------------------------

def _shortlist_candidate(cid, corner, phase, parameter, direction, verdict, severity,
                          score_val=1.0, extra_evidence=None):
    ev = [{"type": "corner_verdict", "corner": corner, "phase": phase, "verdict": verdict,
           "severity": severity, "confidence": 0.5, "source": "test"}]
    if extra_evidence:
        ev += extra_evidence
    return {
        "id": cid, "corner": corner, "phase": phase, "phases": (phase,),
        "actions": [{"parameter": parameter, "direction": direction, "delta": -1}],
        "evidence_refs": ev, "score": score_val,
    }


def test_resolve_conflicts_time_loss_prefers_higher_phase_importance():
    # Same parameter, opposing directions, one anchored to exit_4 (weight
    # 1.2) and one to entry_2_turnin (weight 0.9) -- exit must win, per the
    # user's own elicited exit>entry ordering (config/decision_frame.json
    # scoring_weights.phase_importance).
    entry_c = _shortlist_candidate("entry", 4, "entry_2_turnin", "arb_rl", "soften",
                                    "understeer", "moderate")
    exit_c = _shortlist_candidate("exit", 4, "exit_4", "arb_rl", "stiffen",
                                   "oversteer", "moderate")
    shortlist = [entry_c, exit_c]
    resolve_conflicts(shortlist)

    assert exit_c["conflict_status"] == "wins_time_loss"
    assert entry_c["conflict_status"] == "superseded_by_time_loss"
    assert entry_c["conflict_with"] == ["exit"]
    assert exit_c["conflict_with"] == ["entry"]


def test_resolve_conflicts_no_conflict_for_different_parameters():
    a = _shortlist_candidate("a", 4, "entry_2_turnin", "arb_rl", "soften", "understeer", "moderate")
    b = _shortlist_candidate("b", 4, "exit_4", "tc_lon", "increase", "oversteer", "moderate")
    shortlist = [a, b]
    resolve_conflicts(shortlist)
    assert a["conflict_status"] is None
    assert b["conflict_status"] is None
    assert a["conflict_with"] == []


def test_resolve_conflicts_platform_calming_preferred():
    # Two single-purpose, conflicting candidates on arb_rl (understeer-
    # fixing soften vs oversteer-fixing stiffen) plus a THIRD candidate
    # (different parameter, e.g. diff_position) whose own evidence_refs
    # span BOTH verdicts -- the platform-calming candidate must be
    # preferred over resolving by time-loss.
    understeer_c = _shortlist_candidate("us", 4, "entry_2_turnin", "arb_rl", "soften",
                                         "understeer", "moderate")
    oversteer_c = _shortlist_candidate("os", 4, "exit_4", "arb_rl", "stiffen",
                                        "oversteer", "moderate")
    platform_ev = [
        {"type": "corner_verdict", "corner": 4, "phase": "entry_2_turnin", "verdict": "understeer",
         "severity": "moderate", "confidence": 0.5, "source": "test"},
        {"type": "corner_verdict", "corner": 4, "phase": "exit_4", "verdict": "oversteer",
         "severity": "moderate", "confidence": 0.5, "source": "test"},
    ]
    platform_c = {
        "id": "platform", "corner": 4, "phase": "exit_4", "phases": ("exit_4",),
        "actions": [{"parameter": "diff_position", "direction": "increase", "delta": 1}],
        "evidence_refs": platform_ev, "score": 0.5,
    }
    shortlist = [understeer_c, oversteer_c, platform_c]
    resolve_conflicts(shortlist)

    assert platform_c["conflict_status"] == "platform_calming_available"
    assert understeer_c["conflict_status"] == "superseded_by_platform_calming"
    assert oversteer_c["conflict_status"] == "superseded_by_platform_calming"


def test_resolve_conflicts_never_removes_candidates():
    entry_c = _shortlist_candidate("entry", 4, "entry_2_turnin", "arb_rl", "soften",
                                    "understeer", "moderate")
    exit_c = _shortlist_candidate("exit", 4, "exit_4", "arb_rl", "stiffen",
                                   "oversteer", "moderate")
    shortlist = [entry_c, exit_c]
    result = resolve_conflicts(shortlist)
    assert len(result) == 2
    assert {c["id"] for c in result} == {"entry", "exit"}


# ==========================================================================
# Literature-bridge work package (2026-09-20): ride_height platform_
# stability entries (Segers ch.9/10, coverage-check follow-up to Deepening
# Phase 4b's springs entries). Config-only addition -- these tests confirm
# the two guarantees the work order requires: (1) grade never exceeds
# advisory ("proposed"-family, never derived-from-matrix/engineer-verbatim,
# so nothing above ADVISORY output can result), and (2) the new entries are
# provably inert on real scoring output (platform_stability has no
# _AXIS_TO_VERDICT mapping) -- proving candidate/shortlist identity and
# scores are byte-stable by construction, not just by a single sampled run.
# ==========================================================================

_MATRIX_ELIGIBLE_GRADES = {"derived-from-matrix", "engineer-verbatim", "project-lead-reviewed"}


def test_literature_bridge_grades_never_exceed_advisory():
    config = load_decision_frame_config()
    new_entries = [e for e in config["interaction_table"]
                   if e["parameter"] in ("ride_height_front", "ride_height_rear")
                   and e["performance_axis"] == "platform_stability"]
    assert len(new_entries) == 4  # front/rear x increase/decrease
    for e in new_entries:
        assert e["grade"] == "proposed (Segers ch.9/10)"
        assert e["grade"] not in _MATRIX_ELIGIBLE_GRADES
        assert e["sign"] == -1
        assert {"decrease", "increase"} == {
            x["direction"] for x in new_entries if x["parameter"] == e["parameter"]
        }


def test_ride_height_platform_entries_never_contribute_a_penalty():
    # NOT the same claim as "ride_height's total interaction_penalty is
    # always 0" -- ride_height_front/rear already carry PRE-EXISTING
    # derived-from-matrix entries on understeer_tendency/yaw_stability
    # (real, active axes; Deepening Phase 4b), so a scenario with both an
    # understeer and an unstable_yaw problem active DOES legitimately
    # score a nonzero penalty via THOSE entries -- confirmed below, not
    # papered over. The narrower, correct claim: the platform_stability
    # entries added by this package never contribute to that penalty or
    # its notes, regardless of what else is active, because
    # _AXIS_TO_VERDICT has no mapping for platform_stability at all (a
    # structural guarantee, not a scenario-dependent one).
    config = load_decision_frame_config()
    own_evidence = {"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                     "verdict": "oversteer", "severity": "moderate", "confidence": 0.5, "source": "test"}
    other_active = [
        {"type": "corner_verdict", "corner": 4, "phase": "entry_2_turnin", "verdict": "understeer",
         "severity": "moderate", "confidence": 0.5, "source": "test"},
        {"type": "matrix_verdict", "corner": 4, "phases": ("entry_1_brake",), "verdict": "unstable_yaw",
         "severity": "strong", "confidence": 0.5, "source": "test"},
    ]
    saw_nonzero_penalty = False
    for param in ("ride_height_front", "ride_height_rear"):
        for direction in ("increase", "decrease"):
            candidate = _dummy_candidate(param=param, direction=direction,
                                          evidence_refs=[own_evidence])
            result = score(candidate, [own_evidence] + other_active, None, config)
            if result["components"]["interaction"] != 0.0:
                saw_nonzero_penalty = True
            assert not any("platform_stability" in note for note in result["interaction_notes"])
    # Confirms the scenario was a real exercise of ride_height's existing
    # entries, not an accidentally-inert setup that would make the
    # assertion above trivially true.
    assert saw_nonzero_penalty


def test_decision_frame_config_still_validates():
    # Config-schema regression check, per the work order's own VERIFY step
    # -- the file must still load cleanly and every interaction_table entry
    # must carry the required fields with a legal grade string, not just
    # the four new ones checked above.
    config = load_decision_frame_config()
    required_keys = {"parameter", "direction", "performance_axis", "sign", "grade", "note"}
    legal_grades = {"derived-from-matrix", "proposed", "proposed (Segers ch.9/10)"}
    for e in config["interaction_table"]:
        assert required_keys <= e.keys()
        assert e["grade"] in legal_grades
        assert e["sign"] in (1, -1)


# --- DECISION LAYER SPEC Phase A: registry/config schema (2026-09-22) -------
#
# Data-only additions -- no candidate-generation logic changes in this
# phase. These tests guard the NEW config/registry shape itself, not any
# scoring behaviour (Phase B/C's own job).

def test_effort_rank_gained_half_hour_between_minutes_and_garage_hours():
    # camber's new "half_hour" (20-30 min) class must rank strictly between
    # minutes and garage_hours, and every existing class keeps its relative
    # order -- a pure insertion, not a renumbering that would silently flip
    # any existing _effort_class_for_actions max() comparison.
    assert EFFORT_RANK["seconds"] < EFFORT_RANK["minutes"] < EFFORT_RANK["half_hour"] < EFFORT_RANK["garage_hours"]


def test_eligibility_classes_partition_registry_recommendation_targets():
    # heavy_correctors and click_class together must equal EXACTLY the set
    # of recommendation_target=true registry keys -- no lever silently
    # unreachable (missing from both) and no lever double-counted.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    true_keys = {k for k, v in registry.items() if isinstance(v, dict) and v.get("recommendation_target") is True}
    ec = config["eligibility_classes"]
    heavy = set(ec["heavy_correctors"])
    click = set(ec["click_class"])
    assert heavy & click == set(), "a lever cannot be both heavy_corrector and click_class"
    assert heavy | click == true_keys, "eligibility_classes must cover exactly the recommendation-target levers"


def test_heavy_correctors_are_exactly_springs_camber_toe():
    # Spec fidelity: heavy correctors are springs/camber/toe, nothing else
    # (arb, dampers, ride_height, diff_position, wing_position, splitter,
    # tc/abs/brake_bias all stay click_class per Stage 1/6).
    config = load_decision_frame_config()
    heavy = set(config["eligibility_classes"]["heavy_correctors"])
    assert heavy == {
        "springs_front", "springs_rear",
        "camber_fl", "camber_fr", "camber_rl", "camber_rr",
        "toe_front", "toe_rear",
    }


def test_cost_function_and_display_threshold_are_placeholders():
    config = load_decision_frame_config()
    cost = config["cost_function"]
    for key in ("severity", "change_time", "breadth", "headroom", "interaction"):
        assert key in cost
        assert isinstance(cost[key], (int, float))
    assert "placeholder" in cost["derived_from"]
    threshold = config["display_score_threshold"]
    assert isinstance(threshold["value"], (int, float))
    assert "placeholder" in threshold["derived_from"]


def test_cost_function_six_governed_keys_scoring_weights_retired():
    # DECISION LAYER SPEC C1 (2026-09-22): SIX governed keys, phase_
    # importance and effect_class migrated in as dict-valued sub-keys;
    # scoring_weights fully retired, no dual weight system.
    config = load_decision_frame_config()
    assert "scoring_weights" not in config
    cost = config["cost_function"]
    for phase, value in {"entry_1_brake": 0.8, "entry_2_turnin": 0.9, "apex_3": 1.0,
                          "exit_4": 1.2, "exit_5": 1.2}.items():
        assert cost["phase_importance"][phase] == value
    assert cost["effect_class"]["primary"] == 1.0
    assert cost["effect_class"]["secondary"] == 0.6


def test_tyre_pressure_target_ships_all_null():
    # Per-corner (not per-axle) target, ALL null -- no target pressure
    # exists anywhere in this repo (checked directly, not assumed); this
    # must stay silent, same honesty posture as plausibility_checks.
    # tyre_pressure_window, until a real number is supplied.
    config = load_decision_frame_config()
    target = config["tyre_pressure_target"]
    for corner in ("fl", "fr", "rl", "rr"):
        assert target[corner]["min_psi"] is None
        assert target[corner]["max_psi"] is None
    assert target["compound_note"] is None


def test_splitter_offset_and_brake_bias_promoted_to_recommendation_targets():
    # Both were deliberately omitted before the DECISION LAYER SPEC (an
    # oversight for brake_bias, context-only scope for splitter) -- the
    # spec promotes both to real levers/targets.
    registry = load_setup_parameters_registry()
    splitter = registry["splitter_offset"]
    assert splitter["recommendation_target"] is True
    assert splitter["change_effort"] == "minutes"
    assert splitter["value_space"]["min"] == -4 and splitter["value_space"]["max"] == 4
    bias = registry["brake_bias"]
    assert bias["recommendation_target"] is True
    assert bias["change_effort"] == "seconds"
    assert bias["escalation_tier"] == "cockpit"


def test_diff_position_tier_and_effort_corrected():
    # Registry previously carried garage/seconds -- spec text says
    # pitlane/minutes; both were wrong the same way arb_front_mount's
    # sibling entries already read.
    registry = load_setup_parameters_registry()
    diff = registry["diff_position"]
    assert diff["escalation_tier"] == "pitlane"
    assert diff["change_effort"] == "minutes"


def test_camber_entries_use_half_hour_effort():
    registry = load_setup_parameters_registry()
    for corner in ("fl", "fr", "rl", "rr"):
        assert registry[f"camber_{corner}"]["change_effort"] == "half_hour"


# --- DECISION LAYER SPEC Phase B1: lever-status model (2026-09-22) ----------
#
# Granularity reviewer-confirmed same day (thesis_notes.md "B1 design
# resolution"): status is per-candidate, corner+phase-specific as today,
# never merged across corners; STATUS_NO_TRIGGER is the one synthetic
# exception for a lever with zero candidates anywhere this session.

def test_every_candidate_defaults_to_proposed_status():
    # None of the three non-lever_bridges generators has a blocking/
    # contradiction mechanism yet (B5/B6's own job) -- every candidate they
    # produce must default to proposed via generate_candidates' own
    # setdefault pass.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium"),
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    assert candidates  # sanity: the fixture actually produced something
    for c in candidates:
        assert c["status"] == STATUS_PROPOSED


def test_reachable_lever_keys_matches_registry_recommendation_targets():
    registry = load_setup_parameters_registry()
    true_keys = {k for k, v in registry.items() if isinstance(v, dict) and v.get("recommendation_target") is True}
    assert reachable_lever_keys(registry) == true_keys
    assert len(true_keys) == 42


def test_generate_shortlist_excludes_non_proposed_status():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                                 "presence": "present", "required": False}]
    # springs_front is a heavy corrector (DECISION LAYER SPEC B3) -- strong
    # severity plus a second same-direction corner needed to survive the
    # eligibility gate before the CAP_ADVISORY condition check even runs.
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    not_assessable_ids = {c["id"] for c in candidates if c["status"] == STATUS_NOT_ASSESSABLE}
    assert not_assessable_ids  # sanity: the fixture actually produced a not_assessable candidate
    shortlist = generate_shortlist(candidates, evidence, None, config)
    shortlist_ids = {c["id"] for c in shortlist}
    assert not_assessable_ids.isdisjoint(shortlist_ids)
    # Every proposed candidate the fixture also produced is still present.
    proposed_ids = {c["id"] for c in candidates if c["status"] == STATUS_PROPOSED}
    assert proposed_ids <= shortlist_ids


def test_generate_lever_inventory_adds_no_trigger_rows_for_untouched_levers():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    touched = {a["parameter"] for c in candidates for a in c["actions"]}
    inventory = generate_lever_inventory(candidates, evidence, None, config, registry)
    no_trigger_levers = {e["lever"] for e in inventory if e["status"] == STATUS_NO_TRIGGER}
    # Every reachable lever ends up in the inventory, one way or another.
    assert no_trigger_levers == (reachable_lever_keys(registry) - touched)
    assert len(inventory) == len(candidates) + len(no_trigger_levers)
    for row in inventory:
        if row["status"] == STATUS_NO_TRIGGER:
            assert row["actions"] == []
            assert row["corner"] is None
            assert "score" not in row


def test_generate_lever_inventory_ordering_proposed_then_tail_real_then_no_trigger():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                                 "presence": "present", "required": False}]
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium"),  # stays proposed
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium"),  # becomes not_assessable
    ]
    candidates = generate_candidates(evidence, registry, config)
    inventory = generate_lever_inventory(candidates, evidence, None, config, registry)
    statuses = [row["status"] for row in inventory]
    # proposed block, then non-proposed-real block, then no_trigger block --
    # never interleaved.
    first_non_proposed = next(i for i, s in enumerate(statuses) if s != STATUS_PROPOSED)
    assert all(s == STATUS_PROPOSED for s in statuses[:first_non_proposed])
    first_no_trigger = next(i for i, s in enumerate(statuses) if s == STATUS_NO_TRIGGER)
    assert all(s != STATUS_PROPOSED and s != STATUS_NO_TRIGGER
               for s in statuses[first_non_proposed:first_no_trigger])
    assert all(s == STATUS_NO_TRIGGER for s in statuses[first_no_trigger:])


# --- DECISION LAYER SPEC Phase B2: feedback-only trigger (2026-09-22) ------
#
# Routing reviewer-confirmed same day (thesis_notes.md "B2 design
# resolution"): interaction_table signed entries, click-class only,
# cheapest by EFFORT_RANK, tie-break by _interaction_penalty magnitude,
# unresolved tie raises. Matrix-rule relaxation explicitly rejected.
#
# A GAP was found and reported (thesis_notes.md): the REAL config/
# decision_frame.json has zero click-class entries with sign=+1 on either
# tendency axis today (every such entry belongs to springs, a heavy
# corrector) -- so every test below that needs a real routable lever
# injects one synthetic interaction_table entry into a deepcopy of the
# config, exactly as the existing test_condition_integration_* tests
# already do for lever_bridges conditions.

def _feedback_evidence(corner, phase, raw, speed_class="medium"):
    return {"type": "driver_feedback", "corner": corner, "phase": phase, "speed_class": speed_class,
            "severity": None, "confidence": 0.5, "raw_feedback": raw,
            "verdict": "oversteer" if raw > 0 else "understeer", "source": "test"}


def _inject_click_class_bridge(config, parameter, direction, axis, sign=1, grade="proposed"):
    config["interaction_table"].append({
        "parameter": parameter, "direction": direction, "performance_axis": axis,
        "sign": sign, "grade": grade, "note": "test-injected",
    })
    assert parameter in config["eligibility_classes"]["click_class"], (
        f"test fixture error: {parameter} is not click-class-eligible")


def test_feedback_only_gap_zero_candidates_on_real_config():
    # The reported gap itself: real config, real magnitude-2 feedback, no
    # matching data verdict -- must produce nothing (honest gap, no
    # fallback), for both oversteer and understeer.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_feedback_evidence(4, "exit_4", 3), _feedback_evidence(4, "exit_4", -3)]
    candidates = generate_candidates(evidence, registry, config)
    assert not any(c["trigger_provenance"] == TRIGGER_FEEDBACK_ONLY for c in candidates)


def test_feedback_only_fires_at_magnitude_2_with_injected_click_class_entry():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    evidence = [_feedback_evidence(4, "exit_4", -2)]  # understeer, magnitude 2
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["trigger_provenance"] == TRIGGER_FEEDBACK_ONLY]
    assert len(matches) == 1
    c = matches[0]
    assert c["actions"] == [{"parameter": "arb_rl", "direction": "soften", "delta": -1}]
    assert c["status"] == STATUS_PROPOSED
    assert c["evidence_refs"] == [evidence[0]]


def test_feedback_magnitude_1_never_triggers_a_candidate():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    evidence = [_feedback_evidence(4, "exit_4", -1)]  # magnitude 1 -- corroboration-only
    candidates = generate_candidates(evidence, registry, config)
    assert not any(c["trigger_provenance"] == TRIGGER_FEEDBACK_ONLY for c in candidates)


def test_feedback_only_picks_cheapest_eligible_lever():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    # arb_rl (minutes) vs tc_lon (seconds, strictly cheaper) -- both
    # click-class, both compatible with exit_4 (arb has null phase_affinity,
    # tc_lon's own phase_affinity includes exit_4/exit_5).
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    _inject_click_class_bridge(config, "tc_lon", "increase", "understeer_tendency")
    evidence = [_feedback_evidence(4, "exit_4", -2)]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["trigger_provenance"] == TRIGGER_FEEDBACK_ONLY]
    assert len(matches) == 1
    assert matches[0]["lever_family"] == "tc_lon"


def test_feedback_only_phase_affinity_filters_out_incompatible_lever():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    # toe_front's own phase_affinity is ["entry_2_turnin"] only -- must not
    # fire for exit_4 feedback even though it would be cheaper (minutes vs
    # arb's minutes -- tie irrelevant here, toe_front should never enter
    # the pool at all for this phase). toe_front is a heavy corrector
    # (not click-class) anyway; use arb_front_mount instead (click-class,
    # phase_affinity=["entry_2_turnin"]) as the incompatible lever, and
    # arb_rl (null phase_affinity, always compatible) as the one that must
    # still fire.
    _inject_click_class_bridge(config, "arb_front_mount", "stiffen", "understeer_tendency")
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    evidence = [_feedback_evidence(4, "exit_4", -2)]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["trigger_provenance"] == TRIGGER_FEEDBACK_ONLY]
    assert len(matches) == 1
    assert matches[0]["lever_family"] == "arb_rl"


def test_feedback_only_dedupes_against_existing_data_candidate():
    # springs_rear/soften already has a real lever_bridges entry that fires
    # on oversteer -- injecting the SAME (parameter, direction) as a
    # click-class-tagged interaction_table entry must not spawn a second,
    # competing feedback-only candidate for the corner a data candidate
    # already covers there. Use a corner/phase where springs_rear/soften's
    # own phase_groups fire (exit_4+exit_5) and matrix_verdict evidence
    # supplies the data half.
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    config["eligibility_classes"]["click_class"].append("springs_rear")
    config["eligibility_classes"]["heavy_correctors"].remove("springs_rear")
    _inject_click_class_bridge(config, "springs_rear", "soften", "oversteer_tendency")
    evidence = [
        _matrix_verdict_evidence(9, ["exit_4", "exit_5"], "oversteer", "moderate", "medium"),
        _feedback_evidence(9, "exit_5", 3),
    ]
    candidates = generate_candidates(evidence, registry, config)
    springs_rear_soften = [c for c in candidates
                            if any(a["parameter"] == "springs_rear" and a["direction"] == "soften"
                                   for a in c["actions"]) and c["corner"] == 9]
    assert len(springs_rear_soften) == 1  # not two competing candidates for the same lever+corner
    assert springs_rear_soften[0]["trigger_provenance"] == TRIGGER_BOTH_AGREEING


def test_feedback_only_tie_raises_when_unresolvable():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    # Two levers, identical effort class (both "minutes"), identical
    # (zero) interaction penalty (no other active evidence at this corner
    # for either to collide with) -- genuinely unresolvable without
    # picking arbitrarily.
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    _inject_click_class_bridge(config, "arb_rr", "soften", "understeer_tendency")
    evidence = [_feedback_evidence(4, "exit_4", -2)]
    with pytest.raises(ValueError, match="tie unresolved"):
        generate_candidates(evidence, registry, config)


def test_data_only_candidate_stays_data_only_without_feedback():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert candidates
    for c in candidates:
        assert c["trigger_provenance"] == TRIGGER_DATA_ONLY


def test_eligibility_gate_blocks_heavy_corrector_on_moderate_single_corner():
    # The B3 headline case: mild/single-corner heavy-corrector evidence
    # must not reach the shortlist at all.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]


def test_eligibility_gate_allows_heavy_corrector_on_strong_multi_corner():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1


def test_eligibility_gate_camber_blocked_single_corner_even_at_strong_high_speed():
    # US-APX-high: camber_fl/fr more_negative, understeer, speed_class high,
    # min_severity moderate -- fires at "moderate" in the OLD engine, but a
    # single corner (even strong) must not survive B3's own gate.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(3, ["apex_3"], "understeer", "strong", "high")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c.get("rule_id") == "matrix_us_apx_high"]


def test_eligibility_gate_camber_fires_on_strong_multi_corner():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(3, ["apex_3"], "understeer", "strong", "high"),
        _matrix_verdict_evidence(5, ["apex_3"], "understeer", "strong", "high"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_apx_high" and c["corner"] == 3]
    assert len(matches) == 1


def test_eligibility_gate_camber_never_gets_feedback_bypass():
    # B3's own wording: "Camber ADDITIONALLY always requires the multi-
    # corner gate" -- |feedback|>=4 at the SAME corner/phase must NOT be
    # enough on its own, unlike a non-camber heavy corrector.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(3, ["apex_3"], "understeer", "strong", "high"),
        _feedback_evidence(3, "apex_3", -4),
    ]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c.get("rule_id") == "matrix_us_apx_high"]


def test_eligibility_gate_non_camber_heavy_corrector_gets_feedback_bypass():
    # springs_front (not camber) -- a single strong corner PLUS matching
    # |feedback|>=4 at that same corner/phase must be enough on its own.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _feedback_evidence(7, "apex_3", -4),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1


def test_eligibility_gate_feedback_magnitude_3_not_enough_for_bypass():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _feedback_evidence(7, "apex_3", -3),
    ]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]


def test_eligibility_gate_direct_axle_and_direction_grouping():
    # Direct unit test of _apply_eligibility_gate's own grouping logic --
    # hand-built candidates, since no real rear-camber matrix rule exists
    # to exercise a front-vs-rear axle mismatch through generate_candidates
    # (verified: config/recommendations.json has zero camber_rl/rr
    # suggestions anywhere).
    from modules.decision_frame import _apply_eligibility_gate

    def _heavy_candidate(corner, parameter, direction, severity="strong"):
        return {"id": f"{parameter}:{direction}:C{corner}", "corner": corner, "phase": "apex_3",
                "actions": [{"parameter": parameter, "direction": direction}],
                "evidence_refs": [{"severity": severity}]}

    config = load_decision_frame_config()

    # Same axle (front), same direction, 2 corners -> both survive.
    same_axle_same_dir = [
        _heavy_candidate(3, "camber_fl", "more_negative"),
        _heavy_candidate(9, "camber_fr", "more_negative"),
    ]
    kept = _apply_eligibility_gate(same_axle_same_dir, [], config)
    assert len(kept) == 2

    # Different axle (front vs rear) -- must NOT combine.
    diff_axle = [
        _heavy_candidate(3, "camber_fl", "more_negative"),
        _heavy_candidate(9, "camber_rl", "more_negative"),
    ]
    kept = _apply_eligibility_gate(diff_axle, [], config)
    assert kept == []

    # Same axle, different direction -- must NOT combine.
    diff_direction = [
        _heavy_candidate(3, "camber_fl", "more_negative"),
        _heavy_candidate(9, "camber_fr", "less_negative"),
    ]
    kept = _apply_eligibility_gate(diff_direction, [], config)
    assert kept == []


# --- Eligibility gate amendment (2026-09-23, reviewer decision from item
# (a) findings): |feedback|>=4 matches at CORNER level with SIGN
# CONSISTENCY, not phase-exact -- item (a)'s own v3 run found every real
# springs candidate fires at apex_3/exit_5 while the driver's own
# feedback was entered at entry_1_brake, so the OLD phase-exact key could
# never unlock a springs candidate no matter the feedback magnitude.

def test_eligibility_gate_feedback_bypass_matches_corner_not_phase():
    # Moderate, single-corner (multi_corner_ok=False on its own) -- only
    # the feedback bypass can make this eligible. Feedback fires at a
    # DIFFERENT phase (entry_1_brake) than the matrix evidence
    # (apex_3) -- same corner, same verdict sign (understeer).
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium"),
        _feedback_evidence(7, "entry_1_brake", -4),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1


def test_eligibility_gate_feedback_bypass_sign_mismatch_does_not_unlock():
    # Same corner, |feedback|>=4, but the OPPOSITE sign (oversteer) from
    # the candidate's own understeer verdict -- must NOT unlock.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium"),
        _feedback_evidence(7, "entry_1_brake", 4),  # positive raw -> oversteer
    ]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]


def test_eligibility_gate_feedback_bypass_different_corner_does_not_unlock():
    # Sanity check: the relaxation is still CORNER-scoped, not session-
    # wide -- feedback at a different corner must not unlock this one.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "moderate", "medium"),
        _feedback_evidence(9, "entry_1_brake", -4),
    ]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]


def test_eligibility_gate_data_path_unchanged_by_amendment():
    # The >=2-corner strong-severity DATA path stays phase-scoped and
    # unaffected by the feedback-keying change -- same assertion as the
    # pre-amendment test_eligibility_gate_allows_heavy_corrector_on_
    # strong_multi_corner, re-confirmed after this change, with zero
    # feedback evidence in play at all.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1


# --- DECISION LAYER SPEC Phase B4: breadth (2026-09-22) --------------------
#
# N resolved reviewer-side (thesis_notes.md "B4 breadth design
# resolution", two rounds): N = corners assessed this session INCLUDING
# normal verdicts. NOT derivable from evidence alone -- additive optional
# `assessed_corner_ids` parameter; None preserves pre-existing behaviour
# byte-identical (proven below). Dampers exempt (shaft-speed-range
# selectivity, per spec).

def test_breadth_fields_null_when_assessed_corner_ids_omitted():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)  # no assessed_corner_ids
    assert candidates
    for c in candidates:
        assert c["corners_helped"] is None
        assert c["corners_touched"] is None
        assert c["breadth_note"] is None


def test_breadth_helps_one_corner_rebalances_others_assessed_good():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # matrix_us_tin_med: diff_position decrease, click-class, non-damper --
    # damper rules are breadth-exempt (see test_breadth_dampers_exempt_
    # from_penalty below), so this needs a non-damper rule instead.
    evidence = [_matrix_verdict_evidence(6, ["entry_2_turnin"], "understeer", "moderate", "medium")]
    # Corner 6 is the only one with a finding; corners 1 and 2 were also
    # assessed this session and came back normal (no evidence item at all,
    # per _build_matrix_verdict_evidence's own severity=="normal" skip).
    candidates = generate_candidates(evidence, registry, config, assessed_corner_ids={1, 2, 6})
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_tin_med"]
    assert len(matches) == 1
    c = matches[0]
    assert c["corners_helped"] == [6]
    assert c["corners_touched"] == [1, 2, 6]
    assert "helps C6" in c["breadth_note"]
    assert "rebalances 2 corner(s)" in c["breadth_note"]
    assert "C1/2" in c["breadth_note"]


def test_breadth_no_note_when_helped_covers_every_assessed_corner():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_2_turnin"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config, assessed_corner_ids={6})
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_tin_med"]
    assert len(matches) == 1
    assert matches[0]["breadth_note"] is None


def test_breadth_reports_directly_opposed_corner_separately():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    _inject_click_class_bridge(config, "arb_rl", "soften", "understeer_tendency")
    # Two feedback-only candidates for the SAME lever, opposite directions,
    # at two different corners.
    evidence = [
        _feedback_evidence(4, "exit_4", -2),  # understeer -> arb_rl soften
    ]
    # Add a second, independent oversteer feedback at a different corner
    # routed to the SAME lever in the opposite direction via a second
    # injected entry.
    _inject_click_class_bridge(config, "arb_rl", "stiffen", "oversteer_tendency")
    evidence.append(_feedback_evidence(8, "exit_4", 2))  # oversteer -> arb_rl stiffen
    candidates = generate_candidates(evidence, registry, config, assessed_corner_ids={4, 8, 12})
    soften_candidates = [c for c in candidates
                          if any(a["parameter"] == "arb_rl" and a["direction"] == "soften"
                                 for a in c["actions"])]
    assert len(soften_candidates) == 1
    note = soften_candidates[0]["breadth_note"]
    assert "directly opposed at C8" in note


def test_breadth_dampers_exempt_from_penalty():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # matrix_us_brk_med: damper_bump_ls_fl/fr soften, engineer-verbatim.
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config, assessed_corner_ids={1, 2, 6})
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]
    assert len(matches) == 1
    c = matches[0]
    assert c["breadth_note"] is None
    assert c["corners_touched"] == c["corners_helped"]  # exempt: touched never expands to N


# --- DECISION LAYER SPEC Phase B5: window edge (2026-09-22) ----------------
#
# Universal per-candidate check, gated on setup_data being supplied at all
# (same additive/opt-in pattern as B4's assessed_corner_ids). Hard = the
# registry's own value_space min/max; soft = decision_frame.json's own
# parameter_windows (nominal+-span). Directional: only blocks when the
# delta pushes further past an edge already reached, never when it
# corrects back toward nominal.

def test_window_edge_check_soft_blocks_pushing_further_past_edge():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # arb_rl: value_space 1-7, parameter_windows nominal=4/span=1 -- current
    # 3 is already AT the soft edge (|3-4|=1>=1); softening (delta=-1)
    # pushes further away from nominal.
    setup_data = {"rear_left": {"arb": 3}}
    result = _window_edge_check({"parameter": "arb_rl", "direction": "soften", "delta": -1},
                                 setup_data, registry, config)
    assert result[0] == "soft"
    assert "arb_rl" in result[1]


def test_window_edge_check_no_block_correcting_back_toward_nominal():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # Same current value (3, at the soft edge), but stiffening (delta=+1)
    # moves BACK toward nominal (4) -- must not block.
    setup_data = {"rear_left": {"arb": 3}}
    result = _window_edge_check({"parameter": "arb_rl", "direction": "stiffen", "delta": 1},
                                 setup_data, registry, config)
    assert result is None


def test_window_edge_check_no_block_within_window():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    setup_data = {"rear_left": {"arb": 4}}  # exactly nominal
    result = _window_edge_check({"parameter": "arb_rl", "direction": "soften", "delta": -1},
                                 setup_data, registry, config)
    assert result is None


def test_window_edge_check_hard_blocks_at_value_space_limit():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    # arb_rl value_space min=1 -- already at the physical/legal floor.
    setup_data = {"rear_left": {"arb": 1}}
    result = _window_edge_check({"parameter": "arb_rl", "direction": "soften", "delta": -1},
                                 setup_data, registry, config)
    assert result == ("hard", result[1])
    assert "hard minimum" in result[1]


def test_window_edge_check_not_assessable_when_sheet_unfilled():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    result = _window_edge_check({"parameter": "arb_rl", "direction": "soften", "delta": -1},
                                 {}, registry, config)
    assert result[0] == "not_assessable"
    assert "arb_rl" in result[1]


def test_window_edge_check_skips_non_numeric_enum_value():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    setup_data = {"car": {"wing_position": "P9"}}
    result = _window_edge_check({"parameter": "wing_position", "direction": "increase", "delta": 1},
                                 setup_data, registry, config)
    assert result is None  # not numerically checkable -- same defensive skip as scoring's own


def test_window_edge_check_skips_target_style_actions_with_no_delta():
    from modules.decision_frame import _window_edge_check
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    result = _window_edge_check({"parameter": "brake_bias", "target": "rearward"},
                                 {}, registry, config)
    assert result is None


def test_window_edge_integration_none_setup_data_is_byte_identical():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence()]
    candidates = generate_candidates(evidence, registry, config)  # no setup_data
    assert candidates
    assert not any(c["status"] == STATUS_BLOCKED_AT_EDGE for c in candidates)


def test_window_edge_integration_real_candidate_blocked_and_excluded_from_shortlist():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence()]  # arb_soften:C4:exit_4 -- arb_rl/rr soften
    setup_data = {"rear_left": {"arb": 1}, "rear_right": {"arb": 1}}  # both at hard floor
    candidates = generate_candidates(evidence, registry, config, setup_data=setup_data)
    arb_candidate = next(c for c in candidates if c["id"] == "arb_soften:C4:exit_4")
    assert arb_candidate["status"] == STATUS_BLOCKED_AT_EDGE
    assert arb_candidate["edge_kind"] == "hard"
    assert arb_candidate["edge_label"] == "hard limit"
    shortlist = generate_shortlist(candidates, evidence, setup_data, config)
    assert "arb_soften:C4:exit_4" not in {c["id"] for c in shortlist}


def test_window_edge_integration_does_not_override_existing_not_assessable():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                                 "presence": "present", "required": False}]
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
    ]
    # springs_front has no numeric setup_data entry supplied at all here, so
    # the window-edge check (if it ran) would ALSO want to mark
    # not_assessable -- confirms it never runs at all once B1's own
    # not_assessable is already set, rather than silently agreeing by luck.
    setup_data = {}
    candidates = generate_candidates(evidence, registry, config, setup_data=setup_data)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1
    assert matches[0]["status"] == STATUS_NOT_ASSESSABLE
    assert "edge_kind" not in matches[0]  # B5 never touched this candidate


# --- DECISION LAYER SPEC Phase B6: contradiction (2026-09-22) --------------
#
# data-vs-data: a required evidence_corroboration condition failing on a
# contradiction_sources-listed evidence type -> CONTRADICTED (still
# emitted, out of the shortlist, reason "contradicted by X"). Every other
# required failure stays SUPPRESS (structural inapplicability, not a data
# disagreement). driver-vs-data: NEVER suppresses, side-by-side display
# data only (conflicting_feedback).

def test_evaluate_conditions_contradiction_source_produces_contradicted():
    from modules.decision_frame import evaluate_conditions
    damper_ev = {"type": "damper_motion", "corner": 4, "phase": "entry_2_turnin", "direction": "loading"}
    # required "absent" fails because damper_motion IS present here.
    cond = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
             "presence": "absent", "required": True}]
    verdict, reasons = evaluate_conditions(cond, 4, "entry_2_turnin", [damper_ev], None, {},
                                            contradiction_sources=["damper_motion"])
    assert verdict == "CONTRADICTED"
    assert "damper_motion" in reasons[0]


def test_evaluate_conditions_evidence_corroboration_not_listed_stays_suppress():
    from modules.decision_frame import evaluate_conditions
    damper_ev = {"type": "damper_motion", "corner": 4, "phase": "entry_2_turnin"}
    cond = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
             "presence": "absent", "required": True}]
    # Same failing condition, but contradiction_sources doesn't list it.
    verdict, _ = evaluate_conditions(cond, 4, "entry_2_turnin", [damper_ev], None, {},
                                      contradiction_sources=["lockup"])
    assert verdict == "SUPPRESS"
    verdict, _ = evaluate_conditions(cond, 4, "entry_2_turnin", [damper_ev], None, {})  # omitted entirely
    assert verdict == "SUPPRESS"


def test_evaluate_conditions_phase_transient_never_contradicts():
    from modules.decision_frame import evaluate_conditions
    cond = [{"type": "phase_transient", "required": True}]
    verdict, _ = evaluate_conditions(cond, 4, "apex_3", [], None, {},
                                      contradiction_sources=["damper_motion", "phase_transient"])
    assert verdict == "SUPPRESS"  # not a data-vs-data type at all, never CONTRADICTED


def test_contradiction_integration_real_candidate_contradicted_and_excluded():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    for b in config["lever_bridges"]:
        if b["lever"] == "springs_front" and b["direction"] == "soften":
            b["conditions"] = [{"type": "evidence_corroboration", "evidence_type": "damper_motion",
                                 "presence": "absent", "required": True}]
    damper_ev = {"type": "damper_motion", "corner": 7, "phase": "apex_3", "direction": "loading"}
    evidence = [
        _matrix_verdict_evidence(7, ["apex_3"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["apex_3"], "understeer", "strong", "medium"),
        damper_ev,
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["id"] == "lever_bridge:springs_front:soften:C7:apex_3"]
    assert len(matches) == 1
    c = matches[0]
    assert c["status"] == STATUS_CONTRADICTED
    assert c["condition_reasons"][0].startswith("contradicted by")
    # confidence untouched -- no synthetic condition_gap item, unlike CAP_ADVISORY
    assert c["evidence_refs"] == [evidence[0]]
    shortlist = generate_shortlist(candidates, evidence, None, config)
    assert c["id"] not in {sc["id"] for sc in shortlist}


def test_conflicting_feedback_attached_for_opposite_verdict():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium"),
        _feedback_evidence(6, "entry_1_brake", 3),  # oversteer -- opposite of understeer
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]
    assert len(matches) == 1
    c = matches[0]
    assert c["status"] == STATUS_PROPOSED  # never suppressed
    assert len(c["conflicting_feedback"]) == 1
    assert c["conflicting_feedback"][0]["verdict"] == "oversteer"
    assert c["conflicting_feedback"][0] not in c["evidence_refs"]  # display-only, not corroboration


def test_conflicting_feedback_empty_when_feedback_agrees():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium"),
        _feedback_evidence(6, "entry_1_brake", -3),  # understeer -- agrees
    ]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]
    assert len(matches) == 1
    assert matches[0]["conflicting_feedback"] == []


# --- DECISION LAYER SPEC Phase B7: three bridges (2026-09-22) --------------
#
# brake_bias: Segers C5-1 encoding target, reviewer-confirmed direction
# convention (bias moves AWAY from the limiting axle). diff_position:
# braking-phase addition alongside the pre-existing, already-ls-
# disambiguation-gated exit trigger. splitter_offset: interaction_table
# platform_stability pairing only -- no directional lever_bridges entry
# (unresolved sign convention, reported not invented).

def test_brake_bias_understeer_at_brake_produces_more_rear():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["lever_family"] == "brake_bias"]
    assert len(matches) == 1
    assert matches[0]["actions"] == [{"parameter": "brake_bias", "direction": "more_rear", "delta": 1}]


def test_brake_bias_understeer_at_turnin_also_produces_more_rear():
    # Book's own "corner-entry followed by mid-corner" phrasing -- more_rear
    # covers both entry_1_brake and entry_2_turnin.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_2_turnin"], "understeer", "strong", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["lever_family"] == "brake_bias"]
    assert len(matches) == 1
    assert matches[0]["actions"][0]["direction"] == "more_rear"
    assert matches[0]["actions"][0]["delta"] == 2  # strong -> SEVERITY_RANK 2


def test_brake_bias_oversteer_at_brake_produces_more_front():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "oversteer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if c["lever_family"] == "brake_bias"]
    assert len(matches) == 1
    assert matches[0]["actions"] == [{"parameter": "brake_bias", "direction": "more_front", "delta": 1}]


def test_brake_bias_oversteer_at_turnin_does_not_fire():
    # Book scopes the rear-bias/oversteer case to corner-ENTRY only, not
    # mid-corner -- entry_2_turnin must not trigger more_front.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_2_turnin"], "oversteer", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["lever_family"] == "brake_bias"]


def test_brake_bias_never_fires_outside_braking_phase_groups():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["exit_4", "exit_5"], "understeer", "strong", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    assert not [c for c in candidates if c["lever_family"] == "brake_bias"]


def test_diff_position_braking_instability_bridge_fires():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["entry_1_brake"], "unstable_yaw", "moderate", "medium")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates
               if any(a["parameter"] == "diff_position" and a["direction"] == "increase" for a in c["actions"])
               and c["corner"] == 6]
    assert len(matches) == 1
    assert "EB" in matches[0]["rationale"] or "engine-braking" in matches[0]["rationale"]


def test_diff_position_exit_trigger_already_gated_by_ls_disambiguation():
    # Confirms the report's own claim: the EXISTING exit-phase diff bridge
    # already keys off (and includes in evidence_refs) the ls_disambiguation
    # traction_limited signal -- nothing needed adding for the
    # "acceleration side" of B7's diff item.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_make_oversteer_evidence(), _make_ls_evidence("traction_limited")]
    candidates = generate_candidates(evidence, registry, config)
    diff_exit = next(c for c in candidates if c["id"] == "diff_position_increase:C4:exit_4")
    ls_refs = [e for e in diff_exit["evidence_refs"] if e["type"] == "ls_disambiguation"]
    assert len(ls_refs) == 1
    assert ls_refs[0]["ls_class"] == "traction_limited"
    # And cornering_limited correctly EXCLUDES the diff/TC family entirely
    # (already-existing routing, re-confirmed here in the same breath).
    evidence_cornering = [_make_oversteer_evidence(), _make_ls_evidence("cornering_limited")]
    candidates_cornering = generate_candidates(evidence_cornering, registry, config)
    assert not [c for c in candidates_cornering if c["id"] == "diff_position_increase:C4:exit_4"]


def test_splitter_offset_platform_stability_pairing_present():
    config = load_decision_frame_config()
    entries = [e for e in config["interaction_table"] if e["parameter"] == "splitter_offset"]
    assert {e["direction"] for e in entries} == {"increase", "decrease"}
    for e in entries:
        assert e["performance_axis"] == "platform_stability"
        assert e["sign"] == -1


def test_splitter_offset_direction_convention_resolved_phase_c():
    # RESOLVED Phase C (2026-09-22, author-elicited): negative=more front
    # downforce (decrease fixes understeer), positive=less (increase fixes
    # oversteer) -- supersedes the earlier Phase A/B7 "reported gap, no
    # directional entry" state.
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    splitter_bridges = {(b["lever"], b["direction"]) for b in config["lever_bridges"]
                         if b["lever"] == "splitter_offset"}
    assert splitter_bridges == {("splitter_offset", "decrease"), ("splitter_offset", "increase")}
    assert registry["splitter_offset"]["direction_semantics"]["negative"] == "more front downforce"
    assert registry["splitter_offset"]["direction_semantics"]["positive"] == "less front downforce"


def test_splitter_offset_bridges_fire_only_at_high_speed_class():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence_high = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "high")]
    candidates_high = generate_candidates(evidence_high, registry, config)
    matches_high = [c for c in candidates_high
                     if any(a["parameter"] == "splitter_offset" for a in c["actions"])]
    assert len(matches_high) == 1
    assert matches_high[0]["actions"][0]["direction"] == "decrease"

    evidence_medium = [_matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium")]
    candidates_medium = generate_candidates(evidence_medium, registry, config)
    assert not [c for c in candidates_medium
                if any(a["parameter"] == "splitter_offset" for a in c["actions"])]


def test_splitter_offset_oversteer_high_speed_produces_increase():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [_matrix_verdict_evidence(6, ["exit_4", "exit_5"], "oversteer", "moderate", "high")]
    candidates = generate_candidates(evidence, registry, config)
    matches = [c for c in candidates if any(a["parameter"] == "splitter_offset" for a in c["actions"])]
    assert len(matches) == 1
    assert matches[0]["actions"][0]["direction"] == "increase"


# --- DECISION LAYER SPEC Phase C1: scoring-term fold (2026-09-22) ----------
#
# Six cost_function-governed terms: problem_weight (severity x
# phase_importance x confidence), change_time, breadth, headroom,
# interaction, effect_class. scoring_weights fully retired.

def test_breadth_penalty_zero_when_helps_all_assessed():
    from modules.decision_frame import _breadth_penalty
    penalty, flags = _breadth_penalty({"corners_helped": [3, 9], "corners_touched": [3, 9]}, 1.0)
    assert penalty == 0.0
    assert flags == []


def test_breadth_penalty_negative_fraction_when_helps_subset():
    from modules.decision_frame import _breadth_penalty
    penalty, flags = _breadth_penalty({"corners_helped": [6], "corners_touched": [1, 2, 6]}, 1.0)
    assert penalty == pytest.approx(-(1 - 1 / 3))
    assert flags == []


def test_breadth_penalty_neutral_and_flagged_when_data_unavailable():
    from modules.decision_frame import _breadth_penalty
    penalty, flags = _breadth_penalty({"corners_helped": None, "corners_touched": None}, 1.0)
    assert penalty == 0.0
    assert flags


def test_score_components_are_the_six_named_cost_function_terms():
    config = load_decision_frame_config()
    candidate = _dummy_candidate()
    result = score(candidate, candidate["evidence_refs"], None, config)
    assert set(result["components"].keys()) == {
        "problem_weight", "change_time", "breadth", "headroom", "interaction", "effect_class",
    }


def test_score_term_order_severity_beats_change_time():
    # A STRONG candidate with the WORST effort class still outranks a
    # MODERATE candidate with the BEST effort class -- severity's own
    # multiplicative reach (via sev_rank) exceeds change_time's bounded
    # inverse-effort spread at today's placeholder (all 1.0) weights.
    config = load_decision_frame_config()
    strong_evidence = [{"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                         "verdict": "oversteer", "severity": "strong", "confidence": 1.0, "source": "test"}]
    moderate_evidence = [{"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                           "verdict": "oversteer", "severity": "moderate", "confidence": 1.0, "source": "test"}]
    strong_expensive = _dummy_candidate(effort_class="garage_hours", evidence_refs=strong_evidence)
    moderate_cheap = _dummy_candidate(effort_class="seconds", evidence_refs=moderate_evidence)
    assert (score(strong_expensive, strong_evidence, None, config)["total"]
            > score(moderate_cheap, moderate_evidence, None, config)["total"])


def test_score_term_order_change_time_beats_breadth():
    # Same severity/confidence/effect_class; cheap effort + a SMALL
    # breadth penalty still outranks expensive effort + zero breadth --
    # change_time's own swing (seconds vs garage_hours) exceeds this
    # constructed breadth difference at today's placeholder weights.
    config = load_decision_frame_config()
    evidence = [{"type": "corner_verdict", "corner": 4, "phase": "exit_4",
                 "verdict": "oversteer", "severity": "moderate", "confidence": 1.0, "source": "test"}]
    cheap_partial_breadth = _dummy_candidate(effort_class="seconds", evidence_refs=evidence)
    cheap_partial_breadth["corners_helped"] = [4]
    cheap_partial_breadth["corners_touched"] = [4, 9, 13]  # helps 1 of 3 -- small penalty
    expensive_full_breadth = _dummy_candidate(effort_class="garage_hours", evidence_refs=evidence)
    expensive_full_breadth["corners_helped"] = expensive_full_breadth["corners_touched"] = [4]  # zero penalty
    assert (score(cheap_partial_breadth, evidence, None, config)["total"]
            > score(expensive_full_breadth, evidence, None, config)["total"])


def test_display_score_threshold_splits_shortlist_and_tail():
    config = copy.deepcopy(load_decision_frame_config())
    registry = load_setup_parameters_registry()
    config["display_score_threshold"]["value"] = 1.0  # deliberately high, to force a real split
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "strong", "medium", confidence=1.0),
        _matrix_verdict_evidence(6, ["entry_2_turnin"], "understeer", "moderate", "medium", confidence=0.1),
    ]
    candidates = generate_candidates(evidence, registry, config)
    split = generate_display_split(candidates, evidence, None, config, registry)
    assert split["shortlist"]
    assert split["tail"]
    shortlist_ids = {c["id"] for c in split["shortlist"]}
    tail_ids = {c["id"] for c in split["tail"]}
    assert shortlist_ids.isdisjoint(tail_ids)
    for c in split["shortlist"]:
        assert c["status"] == STATUS_PROPOSED
        assert c["score"] >= 1.0
    # No fixed candidate count anywhere -- every inventory entry lands in
    # exactly one of the two lists, none dropped.
    inventory = generate_lever_inventory(candidates, evidence, None, config, registry)
    assert len(split["shortlist"]) + len(split["tail"]) == len(inventory)


def test_weight_change_reranks_only_verdict_and_evidence_byte_identical():
    # DECISION LAYER SPEC C2's own required test: weight changes re-rank
    # only -- every non-score field of every candidate (verdict/evidence-
    # bearing content) stays byte-identical under a cost_function
    # perturbation; only score/order may change.
    config_a = load_decision_frame_config()
    config_b = copy.deepcopy(config_a)
    config_b["cost_function"]["severity"] = 3.0
    config_b["cost_function"]["change_time"] = 0.1
    config_b["cost_function"]["breadth"] = 5.0
    config_b["cost_function"]["headroom"] = 2.0
    config_b["cost_function"]["interaction"] = 4.0
    config_b["cost_function"]["effect_class"] = {"primary": 2.0, "secondary": 0.1}

    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(9, ["entry_1_brake"], "understeer", "strong", "medium"),
        _matrix_verdict_evidence(3, ["apex_3"], "oversteer", "moderate", "low"),
    ]
    candidates_a = generate_candidates(evidence, registry, config_a)
    candidates_b = generate_candidates(evidence, registry, config_b)
    # generate_candidates itself never reads cost_function at all -- the
    # two candidate lists must already be identical before scoring even
    # runs (the strongest possible form of this guarantee).
    assert candidates_a == candidates_b

    shortlist_a = generate_shortlist(candidates_a, evidence, None, config_a)
    shortlist_b = generate_shortlist(candidates_b, evidence, None, config_b)
    non_score_keys = lambda c: {k: v for k, v in c.items()
                                 if k not in ("score", "score_components", "score_interaction_notes",
                                              "score_flags")}
    ids_a = {c["id"] for c in shortlist_a}
    ids_b = {c["id"] for c in shortlist_b}
    assert ids_a == ids_b  # same set of proposed candidates, order may differ
    by_id_a = {c["id"]: non_score_keys(c) for c in shortlist_a}
    by_id_b = {c["id"]: non_score_keys(c) for c in shortlist_b}
    assert by_id_a == by_id_b
    # And scores actually DID change -- confirms the perturbation was real,
    # not a no-op that would make this test trivially pass.
    scores_a = {c["id"]: c["score"] for c in shortlist_a}
    scores_b = {c["id"]: c["score"] for c in shortlist_b}
    assert scores_a != scores_b


def test_data_and_feedback_agreeing_upgrades_to_both_agreeing():
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = [
        _matrix_verdict_evidence(6, ["entry_1_brake"], "understeer", "moderate", "medium"),
        _feedback_evidence(6, "entry_1_brake", -2),
    ]
    candidates = generate_candidates(evidence, registry, config)
    matrix_candidates = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_med"]
    assert len(matrix_candidates) == 1
    assert matrix_candidates[0]["trigger_provenance"] == TRIGGER_BOTH_AGREEING


# --- WP-DL Phase D (2026-09-22): top-line/tail rendering, D6 rule -------
#
# D6 STOP resolution (thesis_notes.md "Phase D: D6 top-line rendering
# rule resolved"): a magnitude renders only when the action carries a
# real delta AND the registry defines a linear unit for the lever; an
# enum lever's symbolic +-1 (a routing sign, never a physical step) must
# never render as a number. Real registry, hand-crafted action/candidate
# dicts -- same convention this file's own scoring tests already use.

def test_render_action_line_renders_magnitude_when_delta_and_unit_present():
    registry = load_setup_parameters_registry()
    line = render_action_line({"parameter": "arb_rl", "direction": "soften", "delta": -1}, registry)
    assert line == f"{registry['arb_rl']['label']} -1 blade position"


def test_render_action_line_direction_only_when_no_delta():
    registry = load_setup_parameters_registry()
    line = render_action_line({"parameter": "tc_lon", "direction": "decrease"}, registry)
    assert line == f"{registry['tc_lon']['label']}: less intervention"
    assert not any(ch.isdigit() for ch in line)


def test_render_action_line_enum_lever_never_renders_symbolic_delta_as_number():
    registry = load_setup_parameters_registry()
    line = render_action_line({"parameter": "wing_position", "direction": "increase", "delta": 1}, registry)
    assert line == f"{registry['wing_position']['label']}: higher position"
    assert not any(ch.isdigit() for ch in line)


def test_render_action_line_diff_position_gained_unit_phase_d():
    registry = load_setup_parameters_registry()
    assert registry["diff_position"]["value_space"]["unit"] == "position"
    line = render_action_line({"parameter": "diff_position", "direction": "increase", "delta": 1}, registry)
    assert line == f"{registry['diff_position']['label']} +1 position"


def test_render_action_line_brake_bias_appends_direction_word_to_magnitude():
    registry = load_setup_parameters_registry()
    line = render_action_line({"parameter": "brake_bias", "direction": "more_rear", "delta": 2}, registry)
    assert line == f"{registry['brake_bias']['label']} +2 clicks rearward"


def test_render_top_line_no_actions_names_no_corner():
    line = render_top_line({"actions": [], "corner": 7}, {})
    assert line == "Engineer attention: no routed action"
    assert "7" not in line


def test_render_top_line_joins_multiple_actions():
    registry = load_setup_parameters_registry()
    c = {"actions": [
        {"parameter": "arb_rl", "direction": "soften", "delta": -1},
        {"parameter": "arb_rr", "direction": "soften", "delta": -1},
    ]}
    line = render_top_line(c, registry)
    assert " + " in line
    assert line.count("blade position") == 2


def test_candidate_severity_first_severity_bearing_evidence_ref():
    c = {"evidence_refs": [{"severity": None}, {"severity": "strong"}, {"severity": "moderate"}]}
    assert candidate_severity(c) == "strong"


def test_candidate_severity_none_when_nothing_backs_it():
    c = {"evidence_refs": [{"type": "intervention_abs"}]}
    assert candidate_severity(c) is None


def test_render_tail_line_no_trigger_names_lever_only():
    registry = load_setup_parameters_registry()
    entry = {"status": STATUS_NO_TRIGGER, "lever": "camber_fl"}
    line = render_tail_line(entry, registry)
    assert line == f"{registry['camber_fl']['label']}: no trigger this session"


def test_render_tail_line_blocked_at_edge_states_reason():
    registry = load_setup_parameters_registry()
    entry = {
        "status": STATUS_BLOCKED_AT_EDGE,
        "actions": [{"parameter": "splitter_offset", "direction": "increase", "delta": 1}],
        "edge_reason": "splitter_offset already at its typical-window edge",
    }
    line = render_tail_line(entry, registry)
    assert "BLOCKED" in line
    assert "typical-window edge" in line


def test_render_tail_line_contradicted_states_first_reason():
    registry = load_setup_parameters_registry()
    entry = {
        "status": STATUS_CONTRADICTED,
        "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
        "condition_reasons": ["contradicted by C4 matrix_verdict"],
    }
    line = render_tail_line(entry, registry)
    assert "contradicted by C4" in line


def test_render_tail_line_not_assessable_states_reason():
    registry = load_setup_parameters_registry()
    entry = {
        "status": STATUS_NOT_ASSESSABLE,
        "actions": [{"parameter": "arb_rl", "direction": "soften", "delta": -1}],
        "edge_reason": "setup sheet unfilled: arb_rl",
    }
    line = render_tail_line(entry, registry)
    assert "not assessable" in line
    assert "setup sheet unfilled: arb_rl" in line


def test_render_tail_line_proposed_below_threshold_states_score():
    registry = load_setup_parameters_registry()
    entry = {
        "status": STATUS_PROPOSED,
        "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
        "score": 0.12,
    }
    line = render_tail_line(entry, registry)
    assert "0.12" in line
    assert "below display threshold" in line


def test_tyre_pressure_flags_empty_while_target_all_null():
    # Stage 1 check-only item: no per-session pressure-vs-target evidence
    # source exists yet, and tyre_pressure_target is all-null (channel-
    # census correction, thesis_notes.md 2026-09-22) -- must stay silent,
    # never fabricate a flag.
    config = load_decision_frame_config()
    assert tyre_pressure_flags(config) == []


# --- Phase D feedback round, ITEM 1 (2026-09-23): display-layer grouping -

def test_group_display_rows_collapses_identical_top_line():
    registry = load_setup_parameters_registry()
    a = {"id": "a", "corner": 4, "phase": "exit_4", "score": 0.30,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    b = {"id": "b", "corner": 9, "phase": "exit_4", "score": 0.55,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    grouped = group_display_rows([a, b], registry)
    assert len(grouped) == 1
    assert grouped[0]["group_members"] == [a, b]


def test_group_display_rows_score_is_max_never_sum():
    registry = load_setup_parameters_registry()
    a = {"id": "a", "corner": 4, "phase": "exit_4", "score": 0.30,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    b = {"id": "b", "corner": 9, "phase": "exit_4", "score": 0.55,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    grouped = group_display_rows([a, b], registry)
    assert grouped[0]["score"] == 0.55
    assert grouped[0]["corner"] == 9  # representative fields come from the max-score member


def test_group_display_rows_different_parameters_stay_separate():
    registry = load_setup_parameters_registry()
    a = {"id": "a", "corner": 4, "phase": "exit_4", "score": 0.30,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    b = {"id": "b", "corner": 6, "phase": "exit_4", "score": 0.40,
         "actions": [{"parameter": "diff_position", "direction": "increase", "delta": 1}]}
    grouped = group_display_rows([a, b], registry)
    assert len(grouped) == 2
    assert all("group_members" not in g for g in grouped)


def test_group_display_rows_no_trigger_rows_never_collapse_across_levers():
    registry = load_setup_parameters_registry()
    a = {"status": STATUS_NO_TRIGGER, "lever": "camber_fl"}
    b = {"status": STATUS_NO_TRIGGER, "lever": "camber_fr"}
    grouped = group_display_rows([a, b], registry)
    assert len(grouped) == 2


def test_group_display_rows_unrouted_candidates_never_collapse():
    # No parameter-set to match on -- two distinct corners each needing
    # unrouted engineer attention must stay two distinct flags.
    registry = load_setup_parameters_registry()
    a = {"id": "a", "corner": 4, "phase": "entry_1_brake", "score": 0.2, "actions": []}
    b = {"id": "b", "corner": 9, "phase": "entry_1_brake", "score": 0.3, "actions": []}
    grouped = group_display_rows([a, b], registry)
    assert len(grouped) == 2


def test_group_display_rows_single_member_passes_through_unchanged():
    registry = load_setup_parameters_registry()
    a = {"id": "a", "corner": 4, "phase": "exit_4", "score": 0.30,
         "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}]}
    grouped = group_display_rows([a], registry)
    assert grouped == [a]
    assert "group_members" not in grouped[0]
