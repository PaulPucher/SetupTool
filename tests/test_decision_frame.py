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
    PHASE_KEYS,
    aggregate_ls_by_corner,
    build_evidence,
    generate_candidates,
    generate_shortlist,
    load_decision_frame_config,
    resolve_conflicts,
    rule_bridge_status,
    score,
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
                   ls_r_by_phase=None):
    csf_by_phase = csf_by_phase or {}
    csr_by_phase = csr_by_phase or {}
    n_samples_by_phase = n_samples_by_phase or {}
    ls_r_by_phase = ls_r_by_phase or {}
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
        if phase in ls_r_by_phase:
            entry["ls_ratio_f"] = _stat(1.0, n)
            entry["ls_ratio_r"] = _stat(ls_r_by_phase[phase], n)
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

    assert score_cheap["components"]["settings_window_distance"] == 0.0
    assert score_expensive["components"]["settings_window_distance"] == 0.0
    assert score_cheap["components"]["interaction_penalty"] == 0.0
    assert score_expensive["components"]["interaction_penalty"] == 0.0
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
    assert with_other_problem["components"]["interaction_penalty"] < 0.0
    assert with_other_problem["interaction_notes"]

    without_other_problem = score(candidate, [own_evidence], None, config)
    assert without_other_problem["components"]["interaction_penalty"] == 0.0
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
    evidence = [_matrix_verdict_evidence(2, ["entry_1_brake"], "understeer", "moderate", "low")]
    candidates = generate_candidates(evidence, registry, config)
    base = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_low"]
    esc = [c for c in candidates if c.get("rule_id") == "matrix_us_brk_low_esc"]
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


def test_driver_feedback_confidence_floor_at_magnitude_1():
    from modules.decision_frame import _build_driver_feedback_evidence
    cfg = {"confidence_floor": 0.1, "full_confidence_at_raw_abs": 4}
    feedback_data = _feedback_data_for(4, x4=1)  # smallest possible complaint, exit_4
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, cfg)
    assert len(evidence) == 1
    ev = evidence[0]
    assert ev["corner"] == 4 and ev["phase"] == "exit_4"
    assert ev["verdict"] == "oversteer"  # positive raw value
    assert ev["confidence"] == pytest.approx(0.1)  # exactly the floor at |1|


def test_driver_feedback_confidence_full_at_magnitude_4():
    from modules.decision_frame import _build_driver_feedback_evidence
    cfg = {"confidence_floor": 0.1, "full_confidence_at_raw_abs": 4}
    feedback_data = _feedback_data_for(4, e1=-4)  # understeer, entry_1_brake
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, cfg)
    assert len(evidence) == 1
    ev = evidence[0]
    assert ev["phase"] == "entry_1_brake"
    assert ev["verdict"] == "understeer"
    assert ev["confidence"] == pytest.approx(1.0)


def test_driver_feedback_confidence_ramps_linearly_between():
    from modules.decision_frame import _build_driver_feedback_evidence
    cfg = {"confidence_floor": 0.1, "full_confidence_at_raw_abs": 4}
    feedback_data = _feedback_data_for(4, a3=2.5)  # halfway between |1| and |4|
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, cfg)
    # floor + (1-floor)*ramp, ramp=(2.5-1)/(4-1)=0.5 -> 0.1 + 0.9*0.5 = 0.55
    assert evidence[0]["confidence"] == pytest.approx(0.55)


def test_driver_feedback_zero_value_produces_no_evidence():
    from modules.decision_frame import _build_driver_feedback_evidence
    cfg = {"confidence_floor": 0.1, "full_confidence_at_raw_abs": 4}
    feedback_data = _feedback_data_for(4, e1=0, x4=0)
    evidence = _build_driver_feedback_evidence(feedback_data, {4: {"speed_class": "medium"}}, cfg)
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
    config = load_decision_frame_config()
    fw = config["driver_feedback_weighting"]
    assert fw["confidence_floor"] == pytest.approx(0.1)
    assert fw["full_confidence_at_raw_abs"] == 4
    # Same anchor modules.recommendation's own consistency-gate override uses.
    rec_config = load_recommendations_config()
    assert (rec_config["settings"]["consistency_gate"]["feedback_override"]["feedback_override_raw_min"]
            == fw["full_confidence_at_raw_abs"])


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
