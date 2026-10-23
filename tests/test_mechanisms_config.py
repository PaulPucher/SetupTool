# Structural checks of config/mechanisms.json (overhaul D3): every row is
# anchored, names real registry levers and valid phases / speeds, and its
# direction word resolves through the registry (or is one of the declared
# no-convention levers). Plus the phase / speed scope rules of the work order.

import pytest

from modules.corner_aggregation import PHASE_KEYS
from modules.setup_engine import _matching_rows, direction_sign, load_mechanisms
from modules.setup_state import load_setup_parameters_registry

# levers whose current value / sign convention is not in the registry: the
# engine emits direction + step only (brake bias channel TBD; ABS categorical)
NO_CONVENTION = {"brake_bias", "abs_position"}


@pytest.fixture(scope="module")
def mech():
    return load_mechanisms()


@pytest.fixture(scope="module")
def registry():
    return load_setup_parameters_registry()


def test_every_row_is_anchored_and_valid(mech, registry):
    groups = set(mech["phase_groups"])
    for r in mech["mechanisms"]:
        assert r["anchor"].strip(), r["id"]
        assert r["mechanism"] in mech["mechanism_names"], r["id"]
        assert r["levers"] and all(k in registry for k in r["levers"]), r["id"]
        assert set(r["phases"]) <= groups and r["phases"], r["id"]
        assert set(r["symptoms"]) <= {"US", "OS", "INST"} and r["symptoms"], r["id"]
        assert set(r["speed_classes"]) <= {"low", "medium", "high"} and r["speed_classes"], r["id"]
        if not set(r["levers"]) & NO_CONVENTION:
            assert direction_sign(registry[r["levers"][0]], r["direction"]) in (1, -1), r["id"]


def test_phase_groups_cover_the_summary_phases(mech):
    covered = [p for members in mech["phase_groups"].values() for p in members]
    assert sorted(covered) == sorted(PHASE_KEYS)


def test_left_right_levers_move_together(mech):
    for r in mech["mechanisms"]:
        sides = {k[-2:] for k in r["levers"] if k[-3] == "_" and k[-2:] in ("fl", "fr", "rl", "rr")}
        assert sides in (set(), {"fl", "fr"}, {"rl", "rr"}), r["id"]


def test_damper_clicks_convert_via_registry(registry):
    # registry: damper clicks "increasing = softer", ARB "increasing = stiffer"
    for k in ("damper_bump_ls_fl", "damper_rebound_ls_rr"):
        assert registry[k]["direction_semantics"]["increasing"] == "softer"
        assert direction_sign(registry[k], "stiffer") == -1
        assert direction_sign(registry[k], "softer") == 1
    assert direction_sign(registry["arb_fl"], "stiffer") == 1
    assert direction_sign(registry["toe_front"], "more toe-out") == -1
    assert direction_sign(registry["toe_rear"], "more toe-in") == 1
    assert direction_sign(registry["camber_rl"], "less negative camber") == 1
    assert direction_sign(registry["splitter_offset"], "more front downforce") == -1


def test_m2_never_at_apex(mech):
    for sym in ("US", "OS", "INST"):
        for sp in ("low", "medium", "high"):
            assert not [r for r in _matching_rows(sym, "apex_3", sp, mech) if r["mechanism"] == "M2"]


def test_m4_only_at_high_speed(mech):
    for sym in ("US", "OS", "INST"):
        for ph in PHASE_KEYS:
            for sp in ("low", "medium"):
                assert not [r for r in _matching_rows(sym, ph, sp, mech) if r["mechanism"] == "M4"]
    assert [r for r in _matching_rows("US", "apex_3", "high", mech) if r["mechanism"] == "M4"]


def test_toe_front_only_turn_in_low_speed(mech):
    for sym in ("US", "OS", "INST"):
        for ph in PHASE_KEYS:
            for sp in ("low", "medium", "high"):
                hit = [r for r in _matching_rows(sym, ph, sp, mech) if r["levers"] == ["toe_front"]]
                if hit:
                    assert ph == "entry_2_turnin" and sp == "low"


def test_inst_takes_os_direction_unless_named(mech):
    # BRK: M2 names INST explicitly, so no OS-only damper row is inherited there
    brk = _matching_rows("INST", "entry_1_brake", "medium", mech)
    assert {r["id"] for r in brk if r["mechanism"] == "M2"} == {"M2-brk-os-inst-front-bump",
                                                                 "M2-brk-os-inst-rear-rebound"}
    # APX: the rear camber row names OS and INST (4f item 6)
    apx = _matching_rows("INST", "apex_3", "medium", mech)
    assert "M5-camber-rear-APX-OS" in {r["id"] for r in apx}


def test_preference_cells_name_registry_levers(mech, registry):
    for cell in mech["engineer_preference"]:
        assert all(k in registry for k in cell["levers"]), cell["cell"]
        assert cell["phase"] in mech["phase_groups"] and cell["symptom"] in ("US", "OS", "INST")


def test_side_effect_text_only_on_the_ruled_directions(mech):
    # 4e item 7 (TE Tab. 5.14): text on camber more negative, toe_front more
    # toe-out and diff increase; the opposite directions keep null
    ruled = {"more negative camber", "more toe-out"}
    for r in mech["mechanisms"]:
        se = r.get("side_effect")
        if se is None:
            continue
        expect = (r["direction"] == "increase") if r["levers"] == ["diff_position"] else (r["direction"] in ruled)
        assert bool(se["text"]) == expect, r["id"]


def test_os_under_braking_takes_the_braking_instability_rows(mech):
    # 4f item 5 (TE Tab. 5.9 p. 254, Tab. 5.13 p. 295): braking OS gets the
    # rows that were INST-only
    ids = {r["id"] for r in _matching_rows("OS", "entry_1_brake", "medium", mech)}
    assert {"M2-brk-os-inst-front-bump", "M2-brk-os-inst-rear-rebound", "M5-camber-rear-BRK-OS-INST",
            "M7-abs-OS-INST", "M7-bias-OS"} <= ids


def test_bias_os_at_turn_in(mech):
    assert "M7-bias-OS" in {r["id"] for r in _matching_rows("OS", "entry_2_turnin", "low", mech)}


def test_ride_heights_m4_high_speed_only(mech, registry):
    rh = [r for r in mech["mechanisms"] if set(r["levers"]) & {"ride_height_front", "ride_height_rear"}]
    assert len(rh) == 4
    for r in rh:
        assert r["mechanism"] == "M4" and r["speed_classes"] == ["high"]
        assert r["condition"] == "outside_stall_regime"
    # nose down for understeer: front lower, rear higher (Katz Fig. 5.26)
    want = {("ride_height_front", "US"): -1, ("ride_height_front", "OS"): 1,
            ("ride_height_rear", "US"): 1, ("ride_height_rear", "OS"): -1}
    for r in rh:
        assert direction_sign(registry[r["levers"][0]], r["direction"]) == want[(r["levers"][0], r["symptoms"][0])]
    for ph in PHASE_KEYS:
        for sp in ("low", "medium"):
            assert not [r for r in _matching_rows("US", ph, sp, mech) if r in rh]


def test_every_row_names_its_axle(mech):
    # 4h: limiting-end tie-break input; the rule of the work order -- axle
    # levers by their axle, wing rear, splitter front, diff / TC global (bias
    # and ABS global as the implementation classification)
    glob = {"diff_position", "tc_lat", "tc_lon", "brake_bias", "abs_position"}
    for r in mech["mechanisms"]:
        lv = r["levers"][0]
        if lv in glob:
            expected = "global"
        elif lv == "wing_position":
            expected = "rear"
        elif lv == "splitter_offset":
            expected = "front"
        else:
            expected = "front" if lv.endswith(("_fl", "_fr", "_front")) else "rear"
        assert r["axle"] == expected, r["id"]
