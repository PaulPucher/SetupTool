# ORDER INST-3 (author, 2026-11-07): the INST band and the INST vote weight.
# Rule tests in the test_classifier_d2 pattern: values are placed relative to
# the live config, so a re-derived band needs no test edit.

import pytest

from modules.classifier import PHASE_KEYS, classify_corner, phase_symptoms
from modules.setup_engine import session_vote, symptom_votes
from modules.stability_analysis import load_parameters


@pytest.fixture(scope="module")
def cfg():
    p = load_parameters()
    cl = p["classification"]
    return {"stab": cl["stab_neg_thresh_Nm_per_deg"]["value"], "band": cl["inst_band_Nm_per_deg"]["value"],
            "vote": p["setup_engine"]["session_vote"]}


def _summary(stab_apex):
    phases = {ph: {"cs_ratio_f": {"median": 1.0}, "cs_ratio_r": {"median": 1.0},
                   "stability_observed_Nm_per_deg": {"median": stab_apex if ph == "apex_3" else 500.0}}
              for ph in PHASE_KEYS}
    return {"phases": phases, "speed_class": "medium"}


def test_value_inside_band_gives_no_inst(cfg):
    # negative, so the bare sign criterion would call it unstable
    v = cfg["stab"] - 0.5 * cfg["band"]
    assert v < 0
    assert phase_symptoms(_summary(v))["symptoms"] == []
    assert classify_corner(_summary(v))[1] == "ok"


def test_value_below_band_gives_strong_inst(cfg):
    v = cfg["stab"] - cfg["band"] - 0.01
    syms = phase_symptoms(_summary(v))["symptoms"]
    assert [(s["symptom"], s["phase"], s["severity"]) for s in syms] == [("INST", "apex_3", "strong")]


def test_inst_vote_carries_weight_one(cfg):
    assert cfg["vote"]["inst_weight"] == 1
    corner = {"n_laps": 4, "symptoms": [{"symptom": "INST", "phase": "apex_3", "laps": 4, "lap_numbers": [1, 2, 3, 4],
                                         "severity": "strong", "value": -100.0}]}
    cand = {"cockpit": False, "phase": "apex_3", "symptom": "INST", "change": {"sign": -1},
            "levers": ["arb_rl", "arb_rr"], "condition_demoted": False, "condition_note": None,
            "mechanism_id": "test", "preference": 0}
    votes = symptom_votes(1, corner, [cand], cfg["vote"])
    assert len(votes) == 1
    v = votes[0]
    # severity label stays strong, the weight is the INST weight, not severity_weight["strong"]
    assert v["severity"] == "strong" and v["severity_weight"] == 1 and v["weight"] == 1.0
    details = {v["option"]: {"option": v["option"], "levers": ["arb_rl", "arb_rr"], "sign": -1, "axle": "rear",
                             "change": {"feasibility": "ok", "effort": None, "headroom": None}}}
    assert session_vote(votes, details, 2)["ranked"][0]["net"] == 1.0
