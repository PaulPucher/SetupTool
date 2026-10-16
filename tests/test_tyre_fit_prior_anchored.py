# tyre_fit_mode switch (PLAN.md EXPERIMENT PLAN (d2), Part 5): the
# per-session default stays the production path, the experimental
# prior-anchored fit reproduces the (d2) diagnostic on real data, and an
# unknown mode is refused. The bit-identity of the default pipeline is
# guarded by tests/test_golden_pipeline.py, not repeated here.

import copy

import pytest

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import _base_mask, _fit_axle_pacejka_anchored, fit_session_pacejka
from tests.conftest import DUBAI_SAMPLE, resolve_data_path

# (d2) Part 2, Dubai at w = 1 through the production chain
# (diagnostics/inspect_tyre_prior_d2.py achain dubai 1, 2026-10-01).
# RE-PINNED 2026-10-03: Q-CAL-1 mask widening moved the fit population
# (kerb_dilation_s 0.10 = +/-100 ms; base mask 48775 -> 48095 samples,
# -1.4 %). Confirmed before re-pinning: the old +/-50 ms mask reproduces the
# 2026-10-01 values exactly (rel diff 0.0, both axles). Shift old -> new:
# front B +0.01 %, C -0.06 %, D +0.17 %, E -1.494 -> -1.432 (|E| -4.1 %);
# rear B -0.17 %, C -0.01 %, D +0.02 %, E -3.366 -> -3.399 (|E| +1.0 %).
# E (curvature) is the parameter that moves; B, C, D stay within 0.2 %.
D2_DUBAI_W1 = {
    "front": (8.81213657801929, 1.9914468483841652, 8717.917657212922, -1.4321468927228596),
    "rear": (10.362667750697602, 1.906073785684989, 10607.203841931478, -3.3991980650958786),
}


@pytest.fixture(scope="module")
def dubai():
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
    data = parse_csv(resolve_data_path(DUBAI_SAMPLE))
    state = prepare_vehicle_state(data["channels"], params)
    return {"params": params, "data": data, "state": state}


def test_default_mode_is_per_session():
    cfg = load_parameters()["tyre_fit_auto"]
    assert cfg["tyre_fit_mode"] == "per_session"
    assert fit_session_pacejka.__defaults__[-1] == "per_session"


def test_anchored_fit_reproduces_d2_dubai_w1(dubai):
    params, state = dubai["params"], dubai["state"]
    cfg = params["tyre_fit_auto"]
    assert cfg["prior_weight"] == 1.0
    slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    forces = estimate_lateral_forces(state, params)
    bm = _base_mask(state, dubai["data"]["laps"])
    for ax, a, f in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        fit = _fit_axle_pacejka_anchored(slip[a], forces[f], bm, cfg["tyre_fit_prior"][ax], cfg["prior_weight"])
        assert fit["powell_converged"] and fit["sign_ok"]
        assert tuple(fit[p] for p in "BCDE") == pytest.approx(D2_DUBAI_W1[ax], rel=1e-9)


def test_unknown_mode_raises(dubai):
    params = copy.deepcopy(dubai["params"])
    with pytest.raises(ValueError, match="tyre_fit_mode"):
        fit_session_pacejka(dubai["data"], params, tyre_fit_mode="per_track")
