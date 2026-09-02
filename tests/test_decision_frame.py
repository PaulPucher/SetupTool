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

import pytest

from modules.decision_frame import (
    PHASE_KEYS,
    aggregate_ls_by_corner,
    build_evidence,
    generate_candidates,
    generate_shortlist,
    load_decision_frame_config,
    score,
)


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
        assert e["type"] in ("corner_verdict", "ls_disambiguation", "plausibility_brake_balance")

    candidates = generate_candidates(evidence, registry, config)
    for c in candidates:
        assert c["grade"] in ("derived-from-matrix", "proposed")

    shortlist_a = generate_shortlist(candidates, evidence, None, config)
    shortlist_b = generate_shortlist(candidates, evidence, None, config)
    assert [c["id"] for c in shortlist_a] == [c["id"] for c in shortlist_b]
    assert [c["score"] for c in shortlist_a] == [c["score"] for c in shortlist_b]

    print(f"\n[decision_frame end-to-end, real Dubai] evidence={len(evidence)} "
          f"candidates={len(candidates)} shortlist={len(shortlist_a)}")
