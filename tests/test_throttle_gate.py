# Throttle range gate and the braking phase (2026-10-21, thesis_notes
# "Corner phases as a declared adaptation; throttle range-gate fix ruled").
#
# The pedal channel ecu_aps reads 104-106 % at full throttle. Under the old
# [0, 100] gate NBR JST outing 29 had an in-range share of 0.389, the channel
# was graded failed, corner_analysis treated the throttle as missing and all
# 90 analysis-lap brake phases were zero-length -- silently, because the
# canonical rebuild dropped the per-lap warning. Real data only: the second
# test removes the real channel from the real file, no values are invented.

import copy

import pytest

from tests.conftest import find_data_file
from tests.generate_golden import GOLDEN_SESSIONS

NBR29_REL = GOLDEN_SESSIONS["nbr_jst29"][1]


@pytest.fixture(scope="module")
def nbr29():
    if find_data_file(NBR29_REL) is None:
        pytest.skip("NBR JST outing 29 export not on this machine")
    from modules.csv_parser import parse_csv
    return parse_csv(find_data_file(NBR29_REL))


def _analysis_corners(data):
    valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
    return [c for c in data["corners"] if c.get("lap_number") in valid]


def test_throttle_graded_usable_and_brake_phases_populated(nbr29):
    assert nbr29["channels"]["ecu_aps"]["quality"] not in ("missing", "failed")
    corners = _analysis_corners(nbr29)
    assert corners
    populated = [c for c in corners
                 if c["segments"]["entry_1_brake"][1] - c["segments"]["entry_1_brake"][0] > 0]
    # most, not all: corners taken flat or with braking after turn-in keep a
    # zero-length brake phase by construction (2026-08-20 verification)
    assert len(populated) / len(corners) > 0.5, f"{len(populated)} of {len(corners)} brake phases populated"


def test_missing_throttle_is_reported_on_every_corner(nbr29):
    from modules.corner_analysis import analyse_corners, WARN_THROTTLE_MISSING
    data = copy.copy(nbr29)
    data["channels"] = {k: v for k, v in nbr29["channels"].items() if k != "ecu_aps"}
    corners = analyse_corners(data)
    assert corners
    silent = [c for c in corners if WARN_THROTTLE_MISSING not in c["warnings"]]
    assert not silent, f"{len(silent)} of {len(corners)} corners lack the throttle-missing warning"
