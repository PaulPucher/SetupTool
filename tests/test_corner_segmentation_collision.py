# Residual same-lap collision after the seeded split (evaluation-package
# order Part 1, 2026-10-27). Bracket geometry copied from PRC5 laps 14 / 17 /
# 18 (Le Castellet, outing 27): a compound bracket on one lap links three
# corners into one component, and another lap splits the first corner into
# two brackets that both match the same seed. Synthetic by author order:
# the real file sits outside the test data roots.

import numpy as np
import pytest

from modules.corner_analysis import assign_stable_corner_ids, _realize_canonical_corners, _load_config
from tests.test_corner_representative_laps import _corner, _lap

# lap start times; lap distance runs 10 m/s from 0 m at each start
LAP_START = {1: 0.0, 2: 1000.0, 3: 2000.0}


def _bracket(lap, number, start_m, end_m):
    t0 = LAP_START[lap]
    return _corner(lap, number, bracket_start_t=t0 + start_m / 10.0, bracket_end_t=t0 + end_m / 10.0,
                   apex_t=t0 + (start_m + 0.8 * (end_m - start_m)) / 10.0)


@pytest.fixture
def collision_scenario():
    laps = [_lap(n, valid=True, lap_time=200.0, start_time=t0, end_time=t0 + 200.0)
            for n, t0 in LAP_START.items()]
    ld_time = np.concatenate([np.array([0.0, 200.0]) + t0 for t0 in LAP_START.values()])
    ld_data = np.tile(np.array([0.0, 2000.0]), len(LAP_START))
    speed_time = np.linspace(0.0, 2200.0, 22001)
    channels = {
        "lap_distance": {"time": ld_time, "data": ld_data, "unit_raw": "m", "quality": "ok"},
        "ecu_speed": {"time": speed_time, "data": np.full_like(speed_time, 100.0), "quality": "ok"},
    }
    corners = [
        _bracket(1, 2, 897.0, 1100.5), _bracket(1, 3, 1133.9, 1309.2), _bracket(1, 4, 1319.9, 1417.6),
        _bracket(2, 2, 902.9, 1416.1),  # compound, links the three
        _bracket(3, 3, 897.3, 958.2), _bracket(3, 4, 1009.5, 1096.7),  # one corner, two pieces
        _bracket(3, 5, 1172.0, 1413.8),
    ]
    return laps, channels, corners


def test_collision_excludes_only_the_colliding_lap_from_that_group(collision_scenario):
    laps, channels, corners = collision_scenario
    with pytest.warns(UserWarning, match="lap 3 excluded from stable corner 1"):
        excluded = assign_stable_corner_ids(corners, channels, {1, 2, 3})

    assert len(excluded) == 1
    rec = excluded[0]
    assert (rec["lap_number"], rec["stable_corner_id"]) == (3, 1)
    assert rec["reason"] == "ambiguous segmentation, excluded"
    assert rec["range_m"] == pytest.approx([897.3, 1096.7])
    assert sorted(b["corner_number"] for b in rec["brackets_dropped"]) == [3, 4]

    members = {}
    for c in corners:
        members.setdefault(c["stable_corner_id"], []).append((c["lap_number"], c["corner_number"]))
    assert members[1] == [(1, 2)]
    assert sorted(members[2]) == [(1, 3), (2, 2)]
    assert sorted(members[3]) == [(1, 4), (3, 5)]
    for cid, pairs in members.items():
        laps_in = [lap for lap, _ in pairs]
        assert len(laps_in) == len(set(laps_in)), f"stable id {cid} holds a lap twice"


def test_excluded_lap_gets_no_instance_in_that_group(collision_scenario):
    laps, channels, corners = collision_scenario
    with pytest.warns(UserWarning):
        excluded = assign_stable_corner_ids(corners, channels, {1, 2, 3})
    cd = _load_config()["corner_detection"]
    speed_thresholds = _load_config()["corner_speed_thresholds"]
    pairs = {(e["lap_number"], e["stable_corner_id"]) for e in excluded}
    realized = _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds, {1, 2, 3},
                                          excluded_pairs=pairs)

    by_id = {}
    for inst in realized:
        by_id.setdefault(inst["stable_corner_id"], []).append(inst["lap_number"])
    assert 3 not in by_id[1]
    assert sorted(by_id[1]) == [1, 2]
    for cid, laps_in in by_id.items():
        assert len(laps_in) == len(set(laps_in))
    assert 3 in by_id[3]  # the lap stays in every other group