# Setup engine (overhaul D4-D7) on real data: the corner summaries of the two
# default-path goldens. Where a rule needs a symptom the goldens do not repeat
# often enough, the repetition rule is relaxed through the engine's cfg
# argument -- the data stay real, only the rule parameter moves.

import copy
import json

import pytest

from modules.setup_engine import (
    _lever_change, _preference, _select, _data_rank_key, agreement, cell_supports, change_text, data_layer,
    engineer_layer, load_mechanisms, repeats, run, unsupported_preference_cells,
)
from modules.setup_state import load_setup_parameters_registry, load_setup_windows, logged_setup_values
from modules.stability_analysis import load_parameters

NBR29 = "tests/golden/default_nbr_jst29.json"
LC32 = "tests/golden/default_pr_lc32.json"


def _summaries(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)["summaries"]


def _relaxed():
    return dict(load_parameters()["setup_engine"], min_repeat_fraction=0.0, min_repeat_laps=1)


def test_majority_rule():
    cfg = load_parameters()["setup_engine"]
    f, m = cfg["min_repeat_fraction"], cfg["min_repeat_laps"]
    assert repeats(6, 7, f, m)
    assert not repeats(1, 6, f, m)
    assert not repeats(2, 9, f, m)
    assert not repeats(3, 6, f, m)      # exactly half is not a majority
    assert not repeats(1, 1, f, m)      # one lap never suffices


def test_majority_rule_on_real_session():
    # NBR JST 29: INST repeats on 6 / 6 laps at C8 and C11 and counts;
    # understeer appears on at most 2 / 6 laps per corner-phase and does not
    layer = data_layer(_summaries(NBR29))
    counted = {(c["corner"], s["symptom"]) for c in layer["corners"] for s in c["symptoms"]}
    assert counted == {(8, "INST"), (11, "INST")}
    assert all(s["laps"] == 6 for c in layer["corners"] for s in c["symptoms"])


def test_one_change_per_corner_with_at_most_two_alternatives():
    layer = data_layer(_summaries(LC32), cfg=_relaxed())
    assert layer["corners"]
    for c in layer["corners"]:
        assert c["recommended"] is not None
        assert len(c["alternatives"]) <= 2
        keys = [tuple(x["levers"]) for x in [c["recommended"]] + c["alternatives"]]
        assert len(keys) == len(set(keys))


def test_lever_at_limit_is_skipped():
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    sheet = {"front_left": {"arb": 7}, "front_right": {"arb": 7}}   # 7 = registry max
    ch = _lever_change(["arb_fl", "arb_fr"], "stiffer", sheet, registry, windows, "fine")
    assert ch["feasibility"] == "at_limit"
    ok = _lever_change(["arb_fl", "arb_fr"], "softer", sheet, registry, windows, "fine")
    assert ok["feasibility"] == "ok" and [k["to"] for k in ok["keys"]] == [6, 6]
    base = {"severity": "strong", "laps": 3, "condition_demoted": False, "preference": 0, "direction": "stiffer"}
    at_limit = dict(base, levers=["arb_fl", "arb_fr"], change=ch)
    other = dict(base, levers=["arb_rl", "arb_rr"], direction="softer",
                 change=_lever_change(["arb_rl", "arb_rr"], "softer", {}, registry, windows, "fine"))
    sel = _select([at_limit, other], _data_rank_key, 2)
    assert sel["recommended"] is other and sel["at_limit"] == [at_limit]


def test_damper_step_signs_follow_registry():
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    sheet = {"rear_left": {"rebound_ls": 10}, "rear_right": {"rebound_ls": 10}}
    stiffer = _lever_change(["damper_rebound_ls_rl", "damper_rebound_ls_rr"], "stiffer", sheet, registry, windows,
                            "fine")
    assert [k["to"] for k in stiffer["keys"]] == [9, 9]   # increasing clicks = softer


def test_data_layer_identical_with_and_without_feedback():
    summ = _summaries(NBR29)
    fb = {"corners": [{"e1": -5, "e2": 5, "a3": -4, "x4": 3, "x5": -1}] * 15}
    a = run(summ)["data_layer"]
    b = run(summ, feedback_data=fb)["data_layer"]
    assert json.dumps(a, sort_keys=True, default=str) == json.dumps(b, sort_keys=True, default=str)


def test_agreement_values():
    assert agreement("US", -2) == "agree" and agreement("US", 3) == "conflict"
    assert agreement("OS", 1) == "agree" and agreement("OS", -1) == "conflict"
    assert agreement("US", 0) == "neutral" and agreement("INST", 0) == "neutral"
    # 4e: INST = rear saturation, felt as a loose rear -> agrees with
    # oversteer feedback, conflicts with understeer feedback
    assert agreement("INST", 2) == "agree" and agreement("INST", -5) == "conflict"


def test_agreement_orders_right_after_severity():
    # LC32 relaxed: a corner whose ranked list holds both US and OS candidates;
    # the driver reports oversteer at both phases -> OS agrees, US conflicts.
    # Severity stays first; inside one severity, agree > neutral > conflict.
    from modules.corner_aggregation import PHASE_TO_FEEDBACK_KEY, SEVERITY_RANK
    summ = _summaries(LC32)
    cfg = _relaxed()
    data = data_layer(summ, cfg=cfg)
    corner = next(c for c in data["corners"] if {x["symptom"] for x in c["ranked"]} >= {"US", "OS"})
    cid = corner["corner"]
    rows = [{k: 0 for k in ("e1", "e2", "a3", "x4", "x5")} for _ in range(cid)]
    for x in corner["ranked"]:
        rows[cid - 1][PHASE_TO_FEEDBACK_KEY[x["phase"]]] = 2
    eng = engineer_layer(data, summ, {"corners": rows}, cfg=cfg)
    ranked = next(c for c in eng["corners"] if c["corner"] == cid)["ranked"]
    agr = {"agree": 2, "neutral": 1, "conflict": 0}
    keys = [(-SEVERITY_RANK[x["severity"]], -agr[x["agreement"]]) for x in ranked]
    assert keys == sorted(keys)
    assert {x["agreement"] for x in ranked} >= {"agree", "conflict"}


def test_driver_only_row_without_data_support():
    summ = _summaries(NBR29)
    data = data_layer(summ)
    # C1 has no counted symptom on NBR JST 29; a -5 turn-in complaint there
    assert 1 not in {c["corner"] for c in data["corners"]}
    rows = [{k: 0 for k in ("e1", "e2", "a3", "x4", "x5")} for _ in range(15)]
    rows[0]["e2"] = -5
    rows[1]["e2"] = -3   # below the driver-only floor: no row
    eng = engineer_layer(data, summ, {"corners": rows})
    rows_out = [(d["corner"], d["phase"], d["symptom"]) for d in eng["driver_only"]]
    assert rows_out == [(1, "entry_2_turnin", "US")]
    d = eng["driver_only"][0]
    assert d["recommended"]["label"] == "no data support"
    assert d["recommended"]["agreement"] == "driver_only"


def test_engineer_layer_does_not_touch_data_layer():
    summ = _summaries(NBR29)
    data = data_layer(summ)
    before = copy.deepcopy(data)
    engineer_layer(data, summ, {"corners": [{"e1": -5, "e2": 5, "a3": 0, "x4": 0, "x5": 0}] * 15})
    assert json.dumps(before, sort_keys=True, default=str) == json.dumps(data, sort_keys=True, default=str)


def test_session_layer_pending_threshold():
    layer = data_layer(_summaries(NBR29))
    s = layer["session"]
    assert s["threshold"] is None and s["pattern"] is None
    assert s["shares"]["INST"]["corners"] == 2 and s["n_corners"] == 15
    assert s["corners_counted"] == 2


def test_session_cells_use_their_own_denominators():
    # per-lap cells without the repetition rule: US / OS over cells with a
    # CS value on that axle, INST over cells with a stability value
    from modules.classifier import phase_symptoms, verdict_cs_stat
    summ = _summaries(NBR29)
    cells = data_layer(summ)["session"]["cells"]
    n_stab = sum(1 for x in summ for p in x["phases"].values()
                 if p is not None and p["stability_observed_Nm_per_deg"]["median"] == p["stability_observed_Nm_per_deg"]["median"])
    n_cs_f = sum(1 for x in summ for ph, p in x["phases"].items()
                 if p is not None and verdict_cs_stat(x, ph, "f")["median"] == verdict_cs_stat(x, ph, "f")["median"])
    assert cells["INST"]["n"] == n_stab and cells["US"]["n"] == n_cs_f
    k_inst = sum(1 for x in summ for y in phase_symptoms(x)["symptoms"] if y["symptom"] == "INST")
    assert cells["INST"]["k"] == k_inst > 0
    for v in cells.values():
        assert 0 <= v["k"] <= v["n"]


def test_rank_order_preference_before_effort():
    # 4e: severity -> repetition -> (traction demotion) -> preference -> effort
    base = {"severity": "strong", "laps": 3, "condition_demoted": False}
    pref_slow = dict(base, preference=1, change={"effort": "garage_hours", "headroom": None})
    fast = dict(base, preference=0, change={"effort": "seconds", "headroom": None})
    assert sorted([fast, pref_slow], key=_data_rank_key)[0] is pref_slow
    demoted = dict(pref_slow, condition_demoted=True)
    assert sorted([demoted, fast], key=_data_rank_key)[0] is fast
    more_laps = dict(fast, laps=4)
    assert sorted([pref_slow, more_laps], key=_data_rank_key)[0] is more_laps


def _candidate_set(layer):
    return sorted((c["corner"], x["symptom"], x["phase"], tuple(x["levers"]), x["direction"])
                  for c in layer["corners"] for x in c["ranked"] + c["conflicts"] + c["at_limit"])


@pytest.mark.parametrize("path", [NBR29, LC32])
def test_preference_never_adds_or_reverses_a_lever(path):
    summ = _summaries(path)
    mech = load_mechanisms()
    registry = load_setup_parameters_registry()
    no_pref = dict(mech, engineer_preference=[])
    with_pref = data_layer(summ, mechanisms=mech, cfg=_relaxed())
    without = data_layer(summ, mechanisms=no_pref, cfg=_relaxed())
    assert _candidate_set(with_pref) == _candidate_set(without)
    # every preferred candidate is backed by a cell naming its lever and direction
    rows = {r["id"]: r for r in mech["mechanisms"]}
    for c in with_pref["corners"]:
        for x in c["ranked"]:
            if x["preference"]:
                assert any(cell_supports(cell, rows[x["mechanism_id"]], registry)
                           for cell in mech["engineer_preference"]), x["mechanism_id"]


def test_contradicting_cell_gives_no_preference():
    # US-BRK-high names wing increase; the table says decrease for US
    mech, registry = load_mechanisms(), load_setup_parameters_registry()
    wing_us = next(r for r in mech["mechanisms"] if r["id"] == "M4-wing-US")
    assert _preference(wing_us, "US", "entry_1_brake", "high", mech, registry) == 0
    assert _preference(wing_us, "US", "entry_2_turnin", "high", mech, registry) == 1   # US-TIN-high: decrease


def test_unsupported_cells_match_the_config_list():
    mech = load_mechanisms()
    listed = {k for k in mech["_comment_engineer_unsupported"] if not k.startswith("_")}
    assert set(unsupported_preference_cells(mech, load_setup_parameters_registry())) == listed


def test_abs_current_value_from_logged_channel():
    # the ABS map has no sheet field; a logged position fills "from", the
    # target stays open (categorical map)
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    logged = {"abs_position": {"value": 5, "source": "log_abs_pos", "note": None}}
    ch = _lever_change(["abs_position"], "toward the rear-axle-stable map", None, registry, windows, "fine", logged)
    assert [k["from"] for k in ch["keys"]] == [5] and [k["to"] for k in ch["keys"]] == [None]
    c = {"levers": ["abs_position"], "direction": "toward the rear-axle-stable map", "change": ch}
    assert change_text(c).endswith("(current 5)")


def test_logged_abs_on_real_file():
    # NBR JST 29 read directly (channel census rule); skipped without the data
    from tests.conftest import find_data_file
    from modules.csv_parser import parse_csv
    path = find_data_file("NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")
    if path is None:
        pytest.skip("NBR JST 29 not on this machine")
    data = parse_csv(path)
    valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
    out = logged_setup_values(data["channels"], data["laps"], valid)
    assert out["abs_position"]["source"] == "log_abs_pos"
    assert 0 <= out["abs_position"]["value"] <= 11


@pytest.mark.parametrize("path", [NBR29, LC32])
def test_layers_are_json_serialisable(path):
    json.dumps(run(_summaries(path)))
