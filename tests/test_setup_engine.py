# Setup engine (overhaul D4-D7) on real data: the corner summaries of the two
# default-path goldens. Where a rule needs a symptom the goldens do not repeat
# often enough, the repetition rule is relaxed through the engine's cfg
# argument -- the data stay real, only the rule parameter moves.

import copy
import json

import pytest

from modules.setup_engine import (
    _lever_change, _preference, _select, _data_rank_key, agreement, cell_supports, change_text, corner_summary,
    data_layer,
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
    base = {"severity": "strong", "laps": 3, "condition_demoted": False, "preference": 0, "direction": "stiffer",
            "axle_class": 1}
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
    base = {"severity": "strong", "laps": 3, "condition_demoted": False, "axle_class": 1}
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


def test_tc_decrease_from_2_is_at_limit():
    # 4f item 3: the TC scale is monotonic only from 2 to 11
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    for lever in ("tc_lat", "tc_lon"):
        at2 = {lever: {"value": 2, "source": "test", "note": None}}
        assert _lever_change([lever], "decrease", None, registry, windows, "fine", at2)["feasibility"] == "at_limit"
        ok = _lever_change([lever], "increase", None, registry, windows, "fine", at2)
        assert ok["feasibility"] == "ok" and ok["keys"][0]["to"] == 3


def test_camber_step_is_two_clicks():
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    sheet = {"rear_left": {"camber": -3.0}, "rear_right": {"camber": -3.0}}
    ch = _lever_change(["camber_rl", "camber_rr"], "more negative camber", sheet, registry, windows, "fine")
    assert [k["to"] for k in ch["keys"]] == [-3.2, -3.2]


COCKPIT = {"brake_bias", "abs_position", "tc_lat", "tc_lon"}


@pytest.mark.parametrize("path", [NBR29, LC32])
def test_setup_and_cockpit_outputs_are_split(path):
    # 4g: the setup recommendation and its alternatives are mechanical levers
    # only; the cockpit line is a driver-adjustable lever (registry tier)
    summ = _summaries(path)
    out = run(summ, feedback_data={"corners": [{"e1": 2, "e2": -2, "a3": 2, "x4": 2, "x5": 2}] * 20})
    for layer in (out["data_layer"], out["engineer_layer"]):
        for c in data_layer(summ, cfg=_relaxed())["corners"] + layer["corners"]:
            for x in ([c["recommended"]] if c["recommended"] else []) + c["alternatives"]:
                assert not set(x["levers"]) & COCKPIT
            if c["cockpit"] is not None:
                assert set(c["cockpit"]["levers"]) <= COCKPIT


def test_cockpit_line_on_nbr29():
    # NBR JST 29 C8: brake bias more front at turn-in is the cockpit option,
    # the setup recommendation a mechanical lever
    layer = data_layer(_summaries(NBR29))
    c8 = next(c for c in layer["corners"] if c["corner"] == 8)
    assert c8["cockpit"]["levers"] == ["brake_bias"] and c8["cockpit"]["direction"] == "more front"
    assert not set(c8["recommended"]["levers"]) & COCKPIT
    line, _alt, details = corner_summary(c8)
    assert line.endswith("| driver: brake bias forward")
    assert details[-1].startswith("driver can also: ") and "brake_bias more front" in details[-1]


def test_front_ride_height_floor_only():
    # 4g item 3: floor standard - 2 mm, no upper limit
    registry, windows = load_setup_parameters_registry(), load_setup_windows()
    std = registry["ride_height_front"]["value_space"]["standard"]
    sheet = {s: {"ride_height_fia": std, "ride_height_aero": std} for s in ("front_left", "front_right")}
    up = _lever_change(["ride_height_front"], "increase", sheet, registry, windows, "fine")
    assert up["feasibility"] == "ok" and up["keys"][0]["to"] == std + 1
    low = {s: {"ride_height_fia": std - 2, "ride_height_aero": std - 2} for s in ("front_left", "front_right")}
    down = _lever_change(["ride_height_front"], "decrease", low, registry, windows, "fine")
    assert down["feasibility"] == "at_limit"


# work order 4h: session setup recommendation from the corner vote

def _options(rec):
    return {o["option"]: o for o in rec["ranked"] + rec["at_limit"] + rec["not_recommended"]}


def test_session_opposition_is_subtracted():
    # NBR JST 29: rear camber more negative is voted by C8 / C11 at the apex
    # (2 x 2.0) and opposed by C11 early exit (less negative, 2.0)
    rec = data_layer(_summaries(NBR29))["session_recommendation"]
    opt = _options(rec)
    more_neg = opt["camber_rl+camber_rr -1"]
    assert (more_neg["support"], more_neg["opposition"], more_neg["net"]) == (4.0, 2.0, 2.0)
    assert {v["corner"] for v in more_neg["opposing"]} == {11}
    assert opt["camber_rl+camber_rr +1"]["net"] == -2.0


def test_session_net_not_positive_is_never_recommended():
    # rear LS bump: C8 turn-in softer vs C11 early exit stiffer, 2.0 each
    rec = data_layer(_summaries(NBR29))["session_recommendation"]
    shown = [rec["recommended"]] + rec["alternatives"] + rec["equivalent_options"] + rec["ranked"]
    assert all(o["net"] > 0 for o in shown if o)
    bumps = [o for o in rec["not_recommended"] if o["levers"] == ["damper_bump_ls_rl", "damper_bump_ls_rr"]]
    assert len(bumps) == 2 and all(o["net"] == 0.0 for o in bumps)


def test_limiting_end_rear_arb_before_front_arb():
    # INST = rear limiting: rear ARB softer (limiting axle) before diff
    # (global) before front ARB stiffer (other axle), per corner and session
    layer = data_layer(_summaries(NBR29))
    rec = layer["session_recommendation"]
    order = [o["option"] for o in rec["ranked"]]
    assert rec["recommended"]["option"] == "arb_rl+arb_rr -1"
    assert order.index("arb_rl+arb_rr -1") < order.index("diff_position +1") < order.index("arb_fl+arb_fr +1")
    for c in layer["corners"]:
        assert c["recommended"]["levers"] == ["arb_rl", "arb_rr"]
        ranked = [tuple(x["levers"]) for x in c["ranked"]]
        assert ranked.index(("arb_rl", "arb_rr")) < ranked.index(("arb_fl", "arb_fr"))


def test_exact_tie_gives_equivalent_options():
    # NBR JST 29 restricted to C8 (real summaries, one corner): rear ARB,
    # rear ride height and wing tie on every key -> no winner
    summ = [s for s in _summaries(NBR29) if s["stable_corner_id"] == 8]
    rec = data_layer(summ)["session_recommendation"]
    assert rec["recommended"] is None
    assert {o["option"] for o in rec["equivalent_options"]} == {
        "arb_rl+arb_rr -1", "ride_height_rear -1", "wing_position +1"}
    assert all(o["option"] not in {"arb_rl+arb_rr -1"} for o in rec["alternatives"])


@pytest.mark.parametrize("path", [NBR29, LC32])
def test_one_session_recommendation_per_layer(path):
    out = run(_summaries(path), feedback_data={"corners": [{"e1": 2, "e2": -2, "a3": 2, "x4": 2, "x5": 2}] * 20})
    for layer in (out["data_layer"], out["engineer_layer"]):
        rec = layer["session_recommendation"]
        assert rec["recommended"] is None or isinstance(rec["recommended"], dict)
        assert len(rec["alternatives"]) <= 2
        keys = [o["option"] for o in ([rec["recommended"]] if rec["recommended"] else []) + rec["alternatives"]]
        assert len(keys) == len(set(keys))
        for o in rec["ranked"]:
            assert not set(o["levers"]) & COCKPIT


def test_session_data_layer_independent_of_feedback():
    summ = _summaries(NBR29)
    a = run(summ)["data_layer"]["session_recommendation"]
    b = run(summ, feedback_data={"corners": [{"e1": -5, "e2": 5, "a3": -4, "x4": 3, "x5": -1}] * 15})
    assert json.dumps(a, sort_keys=True) == json.dumps(b["data_layer"]["session_recommendation"], sort_keys=True)
    # below the driver-only floor the engineer layer only marks agreement per
    # supporting corner; the weights stay those of the data layer
    mild = run(summ, feedback_data={"corners": [{"e1": -3, "e2": 3, "a3": -2, "x4": 3, "x5": -1}] * 15})
    e = mild["engineer_layer"]["session_recommendation"]["recommended"]
    assert e["net"] == a["recommended"]["net"]
    assert {v["agreement"] for v in e["supporting"]} <= {"agree", "neutral", "conflict"}


def test_driver_only_votes_with_weight_one():
    # a -5 turn-in complaint at C1 (no data symptom) votes 1 x 1 for the
    # understeer levers there, marked driver only
    summ = _summaries(NBR29)
    data = data_layer(summ)
    rows = [{k: 0 for k in ("e1", "e2", "a3", "x4", "x5")} for _ in range(15)]
    rows[0]["e2"] = -5
    rec = engineer_layer(data, summ, {"corners": rows})["session_recommendation"]
    driver = [v for v in rec["votes"] if v["driver_only"]]
    assert driver and all(v["corner"] == 1 and v["weight"] == 1.0 and v["agreement"] == "driver_only"
                          for v in driver)
    opt = _options(rec)
    # front ARB stiffer (INST vote 8.0) is now opposed by the C1 US vote for softer
    assert opt["arb_fl+arb_fr +1"]["opposition"] == 1.0


def test_axle_class_ranks_before_preference():
    # 4h ruling b: severity -> repetition -> (demotion) -> axle class ->
    # preference -> effort -> headroom
    base = {"severity": "strong", "laps": 3, "condition_demoted": False,
            "change": {"effort": "minutes", "headroom": None}}
    limiting = dict(base, axle_class=0, preference=0)
    preferred_other = dict(base, axle_class=2, preference=1)
    assert sorted([preferred_other, limiting], key=_data_rank_key)[0] is limiting
    assert sorted([dict(limiting, laps=2), preferred_other], key=_data_rank_key)[0] is preferred_other


E4 = "diagnostics/results_engine_4e_20261023/{}.pkl"


def _frozen(key):
    # frozen-chain summaries and the real TC flag map of a development
    # session (tracked snapshot of work order 4e)
    import os
    import pickle
    fn = E4.format(key)
    if not os.path.exists(fn):
        pytest.skip(f"{fn} not present")
    with open(fn, "rb") as f:
        return pickle.load(f)


def test_traction_limited_diff_does_not_vote():
    # NBR 63 C8, relaxed repetition: the early-exit diff increase is
    # demoted (TC on most symptom laps) -> its vote is excluded, while the
    # other exit rows of the same symptom vote
    d = _frozen("nbr_o63_cdr")
    layer = data_layer(d["summaries"], tc_map=d["tc_map"], cfg=_relaxed())
    rec = layer["session_recommendation"]
    ex = [x for x in rec["excluded"] if x["corner"] == 8 and x["phase"] == "exit_4"]
    assert [x["option"] for x in ex] == ["diff_position +1"]
    assert not any(v["corner"] == 8 and v["phase"] == "exit_4" and v["option"] == "diff_position +1"
                   for v in rec["votes"])
    assert any(v["corner"] == 8 and v["phase"] == "exit_4" for v in rec["votes"])


def test_low_priority_corner_does_not_vote():
    # NBR qualifying MLA C8, relaxed repetition: high-speed front-only ->
    # kept in the corner diagnosis, no session vote
    d = _frozen("nbr_qp_mla")
    layer = data_layer(d["summaries"], tc_map=d["tc_map"], cfg=_relaxed())
    c8 = next(c for c in layer["corners"] if c["corner"] == 8)
    assert c8["low_priority"] and c8["recommended"] is not None
    rec = layer["session_recommendation"]
    assert not any(v["corner"] == 8 for v in rec["votes"])
    assert {x["corner"] for x in rec["excluded"] if x["option"] is None} == {8}


# UI-1: panel text -- short lines, numbers and sources in the details

def test_session_box_text_on_nbr29():
    from modules.setup_engine import session_summary
    head, why, against, alts, details = session_summary(data_layer(_summaries(NBR29))["session_recommendation"])
    assert head == "Rear ARB softer (1 step) | -"
    assert why == "Why: unstable yaw - C8 turn-in, apex; C11 apex, exit - 6/6 laps"
    assert against is None
    assert alts == "Alternatives: rear springs softer, diff +1"
    assert "net support 8.00" in details[0] and "MM p. 410" in details[0]


def test_corner_line_on_nbr29():
    c8 = next(c for c in data_layer(_summaries(NBR29))["corners"] if c["corner"] == 8)
    line, alt, _details = corner_summary(c8)
    assert line == ("C8 - high - unstable yaw - turn-in, apex - 6/6 -> rear ARB softer"
                    " | driver: brake bias forward")
    assert alt == "alt: rear LS bump softer, wing +1"


def test_engineer_lines_list_differences_only():
    from modules.setup_engine import engineer_lines, feedback_entered
    summ = _summaries(NBR29)
    plain = run(summ)
    assert engineer_lines(plain["data_layer"], plain["engineer_layer"]) == ["No difference to the data layer"]
    assert not feedback_entered({"corners": [{"e1": 0, "e2": 0}]}) and feedback_entered({"corners": [{"e1": 2}]})
    rows = [{k: 0 for k in ("e1", "e2", "a3", "x4", "x5")} for _ in range(15)]
    rows[0]["e2"] = -5
    out = run(summ, feedback_data={"corners": rows})
    lines = engineer_lines(out["data_layer"], out["engineer_layer"])
    assert any(l.startswith("C1 turn-in - driver -5, no data -> ") for l in lines)


def test_advisory_lever_is_never_the_recommended_option():
    # review ruling 2026-10-28: real NBR JST 29 options, the advisory flag
    # set on the top-ranked one; rank, votes and net support stay, only the
    # top position moves to the best non-advisory option
    from modules.setup_engine import ADVISORY_REASON, _all_options, option_line, session_vote
    rec = data_layer(_summaries(NBR29))["session_recommendation"]
    first, second = rec["ranked"][0], rec["ranked"][1]
    details = {o["option"]: copy.deepcopy(o) for o in _all_options(rec)}
    details[first["option"]]["advisory"] = True
    out = session_vote(copy.deepcopy(rec["votes"]), details, 2)
    assert [o["option"] for o in out["ranked"]] == [o["option"] for o in rec["ranked"]]
    assert [o["net"] for o in out["ranked"]] == [o["net"] for o in rec["ranked"]]
    assert out["recommended"]["option"] == second["option"]
    assert out["alternatives"][0]["option"] == first["option"]
    assert ADVISORY_REASON in option_line(out["alternatives"][0])
    for d in details.values():
        d["advisory"] = True
    none = session_vote(copy.deepcopy(rec["votes"]), details, 2)
    assert none["recommended"] is None and none["equivalent_options"] == []
    assert [o["option"] for o in none["alternatives"]] == [o["option"] for o in rec["ranked"][:2]]


def test_advisory_candidate_is_never_the_corner_line():
    corner = data_layer(_summaries(NBR29))["corners"][0]
    ranked = copy.deepcopy(corner["ranked"])
    ranked[0]["advisory"] = True
    sel = _select(ranked, _data_rank_key, 2)
    assert sel["recommended"] is ranked[1] and sel["alternatives"][0] is ranked[0]
    for c in ranked:
        c["advisory"] = True
    sel = _select(ranked, _data_rank_key, 2)
    assert sel["recommended"] is None and sel["alternatives"] == ranked[:2]
    line = corner_summary(dict(corner, recommended=None, ranked=ranked, alternatives=ranked[:2]))[0]
    assert "-> only advisory levers" in line
