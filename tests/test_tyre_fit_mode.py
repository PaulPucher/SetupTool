# tyre_fit_mode after the 2026-10-22 pruning: fit L (literature_bounded) is
# the only mode, in the config and as the function's default; any other
# name, including the removed per_session and prior_anchored, is refused
# rather than silently run as fit L. The default pipeline's numbers are
# guarded by tests/test_golden_pipeline.py, not repeated here.

import copy

import pytest

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import TYRE_FIT_MODES, fit_session_pacejka
from tests.conftest import DUBAI_SAMPLE, resolve_data_path


@pytest.fixture(scope="module")
def dubai():
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
    return {"params": params, "data": parse_csv(resolve_data_path(DUBAI_SAMPLE))}


def test_default_mode_is_literature_bounded():
    assert load_parameters()["tyre_fit_auto"]["tyre_fit_mode"] == "literature_bounded"
    assert TYRE_FIT_MODES == ("literature_bounded",)
    assert fit_session_pacejka.__defaults__[-1] == "literature_bounded"


@pytest.mark.parametrize("mode", ["per_track", "per_session", "prior_anchored"])
def test_unknown_or_removed_mode_raises(dubai, mode):
    params = copy.deepcopy(dubai["params"])
    with pytest.raises(ValueError, match="tyre_fit_mode"):
        fit_session_pacejka(dubai["data"], params, tyre_fit_mode=mode)
