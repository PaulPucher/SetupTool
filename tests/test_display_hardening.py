# WP-HARDEN: display-layer hardening of the decision frame (evidence floor,
# session-level lever-conflict cards). Pure-function tests use minimal
# scored-row dicts, the convention tests/test_decision_frame.py states in
# its header; the candidate-set invariance test runs on the real Dubai
# session (shared pipeline_result fixture).

import copy

import pytest

from modules.decision_frame import (
    CONFLICT_ACROSS_CORNERS,
    DISPLAY_STATE_BELOW_FLOOR,
    STATUS_PROPOSED,
    aggregate_ls_by_corner,
    apply_display_top_n,
    apply_evidence_floor,
    build_evidence,
    card_driver_disagrees,
    generate_candidates,
    generate_display_split,
    group_display_rows,
    harden_display_rows,
    load_decision_frame_config,
    merge_lever_conflicts,
    render_card_header,
    render_tail_line,
    resolve_conflicts,
)
from modules.recommendation import load_setup_parameters_registry, _group_by_corner

from tests.test_decision_frame import classify_fn


def _row(cid, corner, parameter, direction, pw, score=1.0, verdict="oversteer"):
    return {
        "id": cid, "status": STATUS_PROPOSED, "corner": corner, "phase": "exit_4",
        "actions": [{"parameter": parameter, "direction": direction}],
        "evidence_refs": [{"type": "corner_verdict", "corner": corner, "phase": "exit_4",
                           "verdict": verdict, "severity": "moderate", "confidence": 0.5}],
        "score": score, "score_components": {"problem_weight": pw},
    }


def _grouped(*rows):
    merged = dict(rows[0])
    merged["group_members"] = list(rows)
    return merged


@pytest.fixture
def config():
    cfg = copy.deepcopy(load_decision_frame_config())
    cfg["display_evidence_floor"]["value"] = 0.5
    return cfg


@pytest.fixture(scope="module")
def registry():
    return load_setup_parameters_registry()


# --- floor: both branches ---------------------------------------------------

def test_floor_passes_on_problem_weight(config):
    ranked, below = apply_evidence_floor([_row("a", 3, "wing_position", "increase", 0.6)], config)
    assert [r["id"] for r in ranked] == ["a"] and below == []


def test_floor_passes_on_two_corners_with_zero_weight(config):
    row = _grouped(_row("a", 3, "tc_lon", "increase", 0.0), _row("b", 7, "tc_lon", "increase", 0.0))
    ranked, below = apply_evidence_floor([row], config)
    assert len(ranked) == 1 and below == []


def test_floor_demotes_thin_single_corner_row_with_named_state(config):
    ranked, below = apply_evidence_floor([_row("a", 3, "tc_lon", "increase", 0.3)], config)
    assert ranked == []
    assert below[0]["display_state"] == DISPLAY_STATE_BELOW_FLOOR


def test_floor_value_read_from_config(config):
    config["display_evidence_floor"]["value"] = 0.25
    ranked, _below = apply_evidence_floor([_row("a", 3, "tc_lon", "increase", 0.3)], config)
    assert len(ranked) == 1


def _feedback_row(cid, raw, provenance="driver_reported"):
    row = _row(cid, 3, "wing_position", "increase", 0.0)
    row["trigger_provenance"] = provenance
    row["evidence_refs"] = [{"type": "driver_feedback", "corner": 3, "phase": "exit_4",
                             "verdict": "oversteer", "severity": None, "confidence": 0.5,
                             "raw_feedback": raw}]
    return row


def test_feedback_single_corner_passes_on_generation_gate(config):
    # amendment 2026-09-30: |fb| >= 2 generated the candidate, so it clears the floor
    data_thin = _row("data", 5, "tc_lon", "increase", 0.3)
    both = _row("both", 7, "diff_position", "increase", 0.3)
    both["trigger_provenance"] = "both_agreeing"
    both["evidence_refs"].append(_feedback_row("x", -2)["evidence_refs"][0])
    ranked, below = apply_evidence_floor([_feedback_row("fb", 2), both, data_thin], config)
    assert [r["id"] for r in ranked] == ["fb", "both"]
    assert [r["id"] for r in below] == ["data"]


def test_feedback_below_generation_gate_does_not_pass(config):
    ranked, below = apply_evidence_floor([_feedback_row("fb", 1)], config)
    assert ranked == [] and len(below) == 1


# --- conflict merge ---------------------------------------------------------

def test_opposite_directions_on_one_lever_merge_into_one_card():
    rows = [_row("up", 3, "tc_lon", "increase", 0.6, score=2.0),
            _row("other", 5, "wing_position", "increase", 0.6, score=1.5),
            _row("down", 7, "tc_lon", "decrease", 0.3, score=1.0)]
    merged = merge_lever_conflicts(rows)
    assert [r["id"] for r in merged] == ["up", "other"]  # card at its best member's rank
    card = merged[0]
    assert card["conflict_status"] == CONFLICT_ACROSS_CORNERS
    assert [m["id"] for m in card["conflict_members"]] == ["up", "down"]
    assert card["conflict_parameters"] == ["tc_lon"]
    assert card["conflict_with"] == ["down"]


def test_same_direction_on_one_lever_is_not_a_conflict():
    rows = [_row("a", 3, "tc_lon", "increase", 0.6), _row("b", 7, "tc_lon", "increase", 0.3)]
    assert [r.get("conflict_members") for r in merge_lever_conflicts(rows)] == [None, None]


def test_conflict_card_header_shows_both_directions(registry):
    card = merge_lever_conflicts([_row("up", 3, "tc_lon", "increase", 0.6),
                                  _row("down", 7, "tc_lon", "decrease", 0.3, verdict="understeer")])[0]
    header = render_card_header(card, registry)
    assert header.startswith("Lever conflict (")
    assert "more intervention" in header and "less intervention" in header
    assert "C3" in header and "C7" in header


# --- per-member judgement -----------------------------------------------------

def test_card_of_thin_members_goes_to_tail_as_one_line(config, registry):
    # two corners by construction -- judged per card it would clear the floor
    rows = [_row("up", 3, "tc_lon", "increase", 0.3), _row("down", 7, "tc_lon", "decrease", 0.0)]
    ranked, below = harden_display_rows(rows, config)
    assert ranked == [] and len(below) == 1
    line = render_tail_line(below[0], registry)
    assert line.startswith("Lever conflict (")
    assert "below evidence floor (problem weight 0.30, 1 corner / problem weight 0.00, 1 corner)" in line


def test_card_ranks_when_one_member_clears_the_floor(config):
    rows = [_row("up", 3, "tc_lon", "increase", 0.6), _row("down", 7, "tc_lon", "decrease", 0.0)]
    ranked, below = harden_display_rows(rows, config)
    assert len(ranked) == 1 and ranked[0]["conflict_members"] and below == []


def test_member_spanning_two_corners_clears_the_floor(config):
    up = _grouped(_row("up3", 3, "tc_lon", "increase", 0.0), _row("up5", 5, "tc_lon", "increase", 0.0))
    ranked, _below = harden_display_rows([up, _row("down", 7, "tc_lon", "decrease", 0.0)], config)
    assert len(ranked) == 1


# --- tail naming and driver markers -------------------------------------------

def test_below_floor_rows_follow_overflow_in_the_tail(config, registry):
    rows = [_row(f"r{i}", i, p, "increase", 0.6, score=5 - i)
            for i, p in enumerate(("wing_position", "diff_position", "brake_bias", "ride_height_front"))]
    rows.append(_row("thin", 9, "tc_lat", "increase", 0.1, score=0.1))
    ranked, below = harden_display_rows(rows, config)
    visible, tail, note = apply_display_top_n(ranked, below, config)
    assert [r["id"] for r in tail] == ["r3", "thin"]
    assert "below display threshold" in render_tail_line(tail[0], registry)
    assert "below evidence floor" in render_tail_line(tail[1], registry)


def test_driver_disagrees_survives_merge_and_demotion(config):
    down = _row("down", 7, "tc_lon", "decrease", 0.0)
    down["conflicting_feedback"] = [{"verdict": "understeer", "phase": "exit_4"}]
    ranked, below = harden_display_rows([_row("up", 3, "tc_lon", "increase", 0.2), down], config)
    assert card_driver_disagrees(below[0])
    ranked, below = harden_display_rows([down], config)
    assert card_driver_disagrees(below[0])


# --- candidate-set invariance on a real session -------------------------------

def test_candidate_set_invariance_real_dubai(pipeline_result):
    summaries = pipeline_result["summaries"]
    config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    evidence = build_evidence(summaries, aggregate_ls_by_corner(summaries), config, classify_fn)
    candidates = generate_candidates(evidence, registry, config,
                                     assessed_corner_ids=set(_group_by_corner(summaries).keys()))
    shortlist = generate_display_split(candidates, evidence, None, config, registry)["shortlist"]
    resolve_conflicts(shortlist)
    rows = group_display_rows(shortlist, registry)
    before = copy.deepcopy(rows)

    ranked, below = harden_display_rows(rows, config)

    def ids(rs):
        return sorted(m["id"] for r in rs for m in (r.get("group_members") or [r]))
    proposed = sorted(c["id"] for c in candidates if c.get("status", STATUS_PROPOSED) == STATUS_PROPOSED)
    assert ids(ranked + below) == ids(rows) == proposed
    # inputs untouched: candidates, evidence refs and scores byte-identical
    assert rows == before
