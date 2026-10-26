# Damper-path plausibility check (work order DEV-4, 2026-10-24) on real
# files: the PRC JST longrun, whose damper gauges are zeroed against another
# reference (DEV-2), loses the damper path for the session and runs on the
# static split; a healthy session keeps it. Best available accuracy, as the
# app runs, so this reads the machine-local config/car_data.json.

import os

import numpy as np
import pytest

from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    estimate_lateral_forces, estimate_vertical_loads, load_car_data, load_parameters, prepare_vehicle_state,
)
from tests.conftest import find_data_file

# not under a data root: the path the labelled outing (DB id 10) stores
JST_LONGRUN = "H:/Final analysis/PRC/JST-LongRunGT3-CRV.txt"
NBR29 = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"


def _loads(path):
    raw = load_parameters()
    p = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    data = parse_csv(path)
    state = prepare_vehicle_state(data["channels"], p)
    forces = estimate_lateral_forces(state, p)
    fz = estimate_vertical_loads(state, forces, p, channels=data["channels"], car_data=load_car_data())
    static = estimate_vertical_loads(state, forces, {
        **p, "stability_estimation": {**p["stability_estimation"], "vertical_load_source": "static"}})
    return fz, static, p


def test_offset_zeroed_dampers_fall_back_to_static_split():
    if not os.path.exists(JST_LONGRUN):
        pytest.skip("PRC JST longrun export not on this machine")
    fz, static, p = _loads(JST_LONGRUN)
    plaus = fz["damper_plausibility"]
    assert not plaus["passed"] and plaus["ratio"] < p["wheel_loads"]["plausibility_min_static_fraction"]
    assert "damper path rejected" in plaus["reason"]
    assert fz["vertical_load_source_used"] == "static (damper path rejected)"
    for c in ("fl", "fr", "rl", "rr"):
        assert (fz["vertical_load_source_per_sample"][c] == "static_fallback").all()
        np.testing.assert_allclose(fz[f"fz_{c}_N"], static[f"fz_{c}_N"])


def test_healthy_session_keeps_the_damper_path():
    path = find_data_file(NBR29)
    if path is None:
        pytest.skip("NBR outing 29 export not on this machine")
    fz, _static, p = _loads(path)
    plaus = fz["damper_plausibility"]
    assert plaus["passed"] and plaus["ratio"] >= p["wheel_loads"]["plausibility_min_static_fraction"]
    assert fz["vertical_load_source_used"] == "measured"
    assert all((fz["vertical_load_source_per_sample"][c] == "damper").any() for c in ("fl", "fr", "rl", "rr"))
