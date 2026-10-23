# Ruling D2 (author, 2026-10-22): the classifier's boundary and severity
# rules. Values are placed relative to the live config thresholds (the
# test_cs_validity_repair precedent), so a re-derived tier or a new margin
# ruling does not need a test edit.

import pytest

from modules.classifier import classify_corner, cs_value_state, is_low_priority, phase_symptoms
from modules.corner_aggregation import PHASE_KEYS
from modules.stability_analysis import load_parameters


@pytest.fixture(scope="module")
def thr():
    cl = load_parameters()["classification"]
    return {"bf": cl["cs_boundary_margin_front"]["value"], "br": cl["cs_boundary_margin_rear"]["value"], "sf": cl["STRONG_CSF"]["value"],
            "mr": cl["MODERATE_CSR"]["value"], "stab": cl["stab_neg_thresh_Nm_per_deg"]["value"]}


def _summary(override=None, speed_class="medium"):
    # override: {(phase, key): value}, key in csf / csr / stab
    override = override or {}
    phases = {}
    for ph in PHASE_KEYS:
        phases[ph] = {"cs_ratio_f": {"median": override.get((ph, "csf"), 1.0)},
                      "cs_ratio_r": {"median": override.get((ph, "csr"), 1.0)},
                      "stability_observed_Nm_per_deg": {"median": override.get((ph, "stab"), 500.0)}}
    return {"phases": phases, "speed_class": speed_class}


def test_front_past_boundary_is_moderate_understeer(thr):
    v = -thr["bf"] - 0.001
    assert v > thr["sf"]  # outside the strong tier
    sev, short, _l, _s = classify_corner(_summary({("exit_4", "csf"): v}))
    assert sev == "moderate" and short.startswith("understeer @ exit")


def test_front_in_strong_tier_is_strong(thr):
    sev, short, _l, _s = classify_corner(_summary({("apex_3", "csf"): thr["sf"] - 0.05}))
    assert sev == "strong" and "understeer" in short


def test_front_inside_margin_is_not_limiting(thr):
    sev, short, _l, _s = classify_corner(_summary({("apex_3", "csf"): -thr["bf"] / 2}))
    assert sev == "normal" and short.startswith("ok")


def test_rear_limiting_is_strong_and_wins_over_front(thr):
    s = _summary({("exit_4", "csr"): -thr["br"] - 0.02, ("apex_3", "csf"): thr["sf"] - 0.1})
    sev, short, long_, _s = classify_corner(s)
    assert sev == "strong" and short.startswith("oversteer @ exit")
    assert "rear CS" in long_


def test_rear_between_zero_and_moderate_tier_is_info_only(thr):
    v = thr["mr"] / 2
    assert 0 < v < thr["mr"]
    res = phase_symptoms(_summary({("apex_3", "csr"): v}))
    assert res["symptoms"] == [] and len(res["info"]) == 1
    sev, short, _l, _s = classify_corner(_summary({("apex_3", "csr"): v}))
    assert sev == "normal" and short.startswith("ok")


@pytest.mark.parametrize("phase", PHASE_KEYS)
def test_unstable_yaw_is_strong_inst_in_every_phase(thr, phase):
    s = _summary({(phase, "stab"): thr["stab"] - 50.0})
    res = phase_symptoms(s)
    assert [(x["symptom"], x["phase"], x["severity"]) for x in res["symptoms"]] == [("INST", phase, "strong")]
    sev, short, _l, _s = classify_corner(s)
    assert sev == "strong" and "open-loop unstable" in short


def test_low_priority_only_for_high_speed_front_only(thr):
    us = phase_symptoms(_summary({("apex_3", "csf"): -thr["bf"] - 0.05}))["symptoms"]
    os_ = phase_symptoms(_summary({("apex_3", "csr"): -thr["br"] - 0.05}))["symptoms"]
    assert is_low_priority(us, "high")
    assert not is_low_priority(us, "medium")
    assert not is_low_priority(us + os_, "high")
    assert not is_low_priority([], "high")
    _sev, _short, long_, _s = classify_corner(_summary({("apex_3", "csf"): -thr["bf"] - 0.05}, "high"))
    assert "low priority" in long_


def test_colour_state_matches_symptom_rule(thr):
    for v in (-thr["bf"] - 0.001, thr["sf"] - 0.01, -thr["bf"] / 2, 0.5):
        res = phase_symptoms(_summary({("apex_3", "csf"): v}))["symptoms"]
        state = cs_value_state(v, "f")
        assert (state in ("moderate", "strong")) == bool(res)
        if res:
            assert res[0]["severity"] == state
    for v in (-thr["br"] - 0.001, thr["mr"] / 2, 0.5):
        res = phase_symptoms(_summary({("apex_3", "csr"): v}))
        state = cs_value_state(v, "r")
        assert (state == "strong") == bool(res["symptoms"])
        assert (state == "info") == bool(res["info"])
