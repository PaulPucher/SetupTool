# Corner canonicalisation fix (2026-09-03 work order): representative-lap
# filtering. _representative_lap_numbers is a pure function tested directly;
# the assign_stable_corner_ids / _realize_canonical_corners integration
# tests use hand-built corners + a synthetic two-lap lap_distance channel
# (no real telemetry file -- keeps the suite fast and portable, matching
# tests/test_wheel_loads.py's own synthetic-only convention) to prove the
# two concrete claims the work order makes: (1) a stable corner seeded only
# by non-representative laps never comes into existence, and (2) a stable
# corner's canonical geometry is the median of its REPRESENTATIVE members
# only, even when a non-representative lap is also a real member.

import numpy as np
import pytest

from modules.corner_analysis import (
    _representative_lap_numbers, assign_stable_corner_ids, _realize_canonical_corners, _load_config,
)


def _lap(number, valid=True, lap_time=None, lap_time_precise=None, start_time=None, end_time=None):
    return {
        "lap_number": number,
        "is_valid_for_analysis": valid,
        "lap_time": lap_time,
        "lap_time_precise": lap_time_precise,
        "start_time": start_time,
        "end_time": end_time,
    }


class TestRepresentativeLapNumbers:
    def test_excludes_lap_outside_factor(self):
        laps = [_lap(1, lap_time=125.0), _lap(2, lap_time=137.0)]
        assert _representative_lap_numbers(laps, 1.05) == {1}

    def test_includes_lap_inside_factor(self):
        laps = [_lap(1, lap_time=125.0), _lap(2, lap_time=126.0)]
        assert _representative_lap_numbers(laps, 1.05) == {1, 2}

    def test_boundary_is_inclusive(self):
        laps = [_lap(1, lap_time=100.0), _lap(2, lap_time=105.0)]
        assert _representative_lap_numbers(laps, 1.05) == {1, 2}

    def test_prefers_lap_time_precise_over_lap_time(self):
        # lap_time alone would put lap 2 outside the factor; lap_time_precise
        # (the value _effective_lap_time actually uses, csv_parser.py) pulls
        # it back inside -- proves the fallback logic is wired, not just the
        # plain lap_time field.
        laps = [
            _lap(1, lap_time=125.0, lap_time_precise=125.0),
            _lap(2, lap_time=140.0, lap_time_precise=126.0),
        ]
        assert _representative_lap_numbers(laps, 1.05) == {1, 2}

    def test_ignores_invalid_laps_both_as_candidate_and_as_fastest(self):
        # An invalid lap that happens to be the shortest must not set
        # fastest_time, and must never appear in the returned set itself.
        laps = [
            _lap(1, valid=False, lap_time=50.0),
            _lap(2, lap_time=125.0),
            _lap(3, lap_time=133.0),  # 133/125 = 1.064 > 1.05 -- outside the factor
        ]
        assert _representative_lap_numbers(laps, 1.05) == {2}

    def test_no_valid_laps_returns_empty_set(self):
        laps = [_lap(1, valid=False, lap_time=50.0)]
        assert _representative_lap_numbers(laps, 1.05) == set()


def _corner(lap_number, corner_number, bracket_start_t, bracket_end_t, apex_t):
    # Phase boundaries placed proportionally inside [bracket_start_t,
    # bracket_end_t]; only entry_2_turnin[0] and exit_5[1] feed
    # bracket_start_m/bracket_end_m (assign_stable_corner_ids), and
    # entry_1_brake[0]/entry_2_turnin[0]/exit_4[1] feed brake_s/turnin_s/
    # half_s (_realize_canonical_corners) -- both exercised here.
    span = bracket_end_t - bracket_start_t
    return {
        "lap_number": lap_number,
        "corner_number": corner_number,
        "speed_class": None,
        "apex_time": apex_t,
        "apex_speed": 0.0,
        "apex_lateral_g": None,
        "segments": {
            "entry_1_brake": (bracket_start_t - 1.0, bracket_start_t),
            "entry_2_turnin": (bracket_start_t, apex_t),
            "apex_3": (apex_t, apex_t),
            "exit_4": (apex_t, apex_t + span * 0.3),
            "exit_5": (apex_t + span * 0.3, bracket_end_t),
        },
        "method": "steering",
        "warnings": [],
        "stable_corner_id": None,
    }


@pytest.fixture
def two_lap_scenario():
    """
    Lap 1: t=[0,20], distance d=10*t (0..200 m). One bracket A1, d=[40,60].
    Lap 2: t=[100,120], distance d=10*(t-100) (0..200 m). Bracket A2,
    d=[44,56] -- overlaps A1 (frac=1.0, well above bracket_overlap_min_
    fraction), a genuine shared corner. Bracket B2, d=[150,170] -- has no
    counterpart on lap 1 at all, a lap-2-only detection (stands in for v3's
    lap 9 fragments in the real C17-C20 case).
    """
    laps = [
        _lap(1, valid=True, lap_time=100.0, start_time=0.0, end_time=20.0),
        _lap(2, valid=True, lap_time=100.0, start_time=100.0, end_time=120.0),
    ]
    ld_time = np.array([0.0, 5.0, 10.0, 15.0, 20.0, 100.0, 105.0, 110.0, 115.0, 120.0])
    ld_data = np.array([0.0, 50.0, 100.0, 150.0, 200.0, 0.0, 50.0, 100.0, 150.0, 200.0])
    speed_time = np.linspace(0.0, 120.0, 241)
    channels = {
        "lap_distance": {"time": ld_time, "data": ld_data, "unit_raw": "m", "quality": "ok"},
        "ecu_speed": {"time": speed_time, "data": np.full_like(speed_time, 100.0), "quality": "ok"},
    }
    corners = [
        _corner(1, 1, bracket_start_t=4.0, bracket_end_t=6.0, apex_t=4.8),      # A1
        _corner(2, 1, bracket_start_t=104.4, bracket_end_t=105.6, apex_t=105.0),  # A2
        _corner(2, 2, bracket_start_t=115.0, bracket_end_t=117.0, apex_t=115.8),  # B2, isolated
    ]
    return laps, channels, corners


def test_cluster_seeded_only_by_non_representative_laps_is_dropped(two_lap_scenario):
    laps, channels, corners = two_lap_scenario
    representative_laps = {1}  # lap 2 excluded, matching the real v3 case

    assign_stable_corner_ids(corners, channels, representative_laps)

    # B2 (lap 2 only) must be gone entirely -- not present with any
    # stable_corner_id, not left at None either (that would dump it into a
    # bogus shared group downstream).
    assert len(corners) == 2
    assert {(c["lap_number"], c["corner_number"]) for c in corners} == {(1, 1), (2, 1)}
    assert {c["stable_corner_id"] for c in corners} == {1}


def test_surviving_cluster_geometry_uses_representative_members_only(two_lap_scenario):
    laps, channels, corners = two_lap_scenario
    representative_laps = {1}

    assign_stable_corner_ids(corners, channels, representative_laps)
    cd = _load_config()["corner_detection"]
    speed_thresholds = _load_config()["corner_speed_thresholds"]
    realized = _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds, representative_laps)

    assert len(realized) == 2  # both laps re-realized against the one surviving corner
    for inst in realized:
        # Canon geometry must equal lap 1's OWN bracket exactly (40/60 m) --
        # not the naive full-membership median (42/58 m), which is what
        # median([40, 44]) / median([60, 56]) would give if lap 2 (non-
        # representative) still contributed to the geometry.
        assert inst["bracket_start_m"] == pytest.approx(40.0)
        assert inst["bracket_end_m"] == pytest.approx(60.0)
        # Lap 2 IS a genuine member of this corner (it detected A2 there) --
        # excluding it from shaping the geometry must not mislabel it quiet.
        assert "canonical_quiet" not in inst["warnings"]


def test_all_laps_representative_keeps_the_isolated_corner_too(two_lap_scenario):
    # Control case: when lap 2 is ALSO representative (e.g. Dubai, where
    # every valid lap is within the factor), B2 must survive as its own
    # stable corner -- the filter must not fire when there is nothing to
    # exclude.
    laps, channels, corners = two_lap_scenario
    representative_laps = {1, 2}

    assign_stable_corner_ids(corners, channels, representative_laps)

    assert len(corners) == 3
    assert {c["stable_corner_id"] for c in corners} == {1, 2}
