# R26/R27 band rule (author ruling 2026-10-17): a clean linear window may set
# the CS reference only if its stiffness lies inside the tyre-fit stiffness
# band (tyre_fit_auto.stiffness_min_npr / stiffness_max_npr); otherwise the
# previous reference stays in force. Real Dubai data, kinematic sideslip as in
# test_kerb_smear_grade (the rule does not depend on the beta source).

import copy

import numpy as np
import pytest

from modules.stability_analysis import (
    _build_non_analysis_lap_mask, estimate_cornering_stiffness, estimate_lateral_forces,
    estimate_sideslip, estimate_slip_angles, reconstruct_cs_window_start, resolve_cs_min_window_samples,
)


@pytest.fixture(scope="module")
def slip_forces(state, effective_params):
    beta = estimate_sideslip(state, effective_params)
    return estimate_slip_angles(state, beta, effective_params), estimate_lateral_forces(state, effective_params)


@pytest.fixture(scope="module")
def candidates(state, effective_params, parsed_data, slip_forces):
    """Per axle, the samples whose regression window would set the reference
    without the band: max |alpha| under the linear threshold and no kerb,
    smear or off-analysis-lap sample in [start, i)."""
    se = effective_params["stability_estimation"]
    laps = parsed_data.get("laps", [])
    slip, forces = slip_forces
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params, laps)
    blocked = state["kerb_mask"] | state["kerb_smear_zone"] | _build_non_analysis_lap_mask(state["time"], laps)
    min_window = resolve_cs_min_window_samples(effective_params, state["sample_rate_hz"])
    out = {}
    for ax in "fr":
        alpha = slip[f"alpha_{ax}_filt"]
        idx = []
        for i in np.flatnonzero(np.isfinite(cs[f"C_alpha_{ax}"])):
            start = reconstruct_cs_window_start(alpha, int(i), min_window, se["cs_min_slip_angle_span_rad"],
                                                state["s_m"], se["cs_max_window_m"])
            if np.max(np.abs(alpha[start:i])) < se["cs_linear_slip_threshold_rad"] and not blocked[start:i].any():
                idx.append(int(i))
        out[ax] = np.array(idx)
    return out


def _check_band(cs, cand, lo, hi):
    """Inside the band a candidate sets the reference to its own stiffness;
    outside, the reference equals the one in force at the previous computed
    sample. Returns the rejected candidates' stiffness values."""
    rejected = []
    for ax in "fr":
        c_alpha, ref = cs[f"C_alpha_{ax}"], cs[f"C_linear_ref_{ax}"]
        computed = np.flatnonzero(np.isfinite(c_alpha))
        pos = np.searchsorted(computed, cand[ax])
        for i, p in zip(cand[ax], pos):
            if lo <= c_alpha[i] <= hi:
                assert ref[i] == c_alpha[i], f"axle {ax} sample {i}: in-band window did not set the reference"
            else:
                prev = ref[computed[p - 1]] if p > 0 else np.nan
                assert ref[i] == prev or (np.isnan(ref[i]) and np.isnan(prev)), \
                    f"axle {ax} sample {i}: out-of-band window {c_alpha[i]:.0f} N/rad changed the reference"
                rejected.append(c_alpha[i])
        in_force = ref[np.isfinite(ref)]
        assert ((in_force >= lo) & (in_force <= hi)).all()
    return np.array(rejected)


def test_below_band_zero_and_negative_slopes_cannot_set_the_reference(
        state, effective_params, parsed_data, slip_forces, candidates):
    cfg = effective_params["tyre_fit_auto"]
    slip, forces = slip_forces
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params, parsed_data.get("laps", []))
    rejected = _check_band(cs, candidates, cfg["stiffness_min_npr"], cfg["stiffness_max_npr"])
    # Dubai holds clean low-excitation windows below the floor, zero/negative ones among them
    assert (rejected < cfg["stiffness_min_npr"]).any()
    assert (rejected <= 0).any()
    for ax in "fr":
        assert not (cs[f"C_linear_ref_{ax}"] <= 0).any()


def test_above_band_slopes_cannot_set_the_reference(state, effective_params, parsed_data, slip_forces, candidates):
    """No Dubai window exceeds the configured ceiling, so the ceiling is
    lowered to the front candidates' median: the same rule must then hold
    the reference against every steeper window."""
    slip, forces = slip_forces
    params = copy.deepcopy(effective_params)
    cfg = params["tyre_fit_auto"]
    ceiling = float(np.median(estimate_cornering_stiffness(
        slip, forces, state, effective_params, parsed_data.get("laps", []))["C_alpha_f"][candidates["f"]]))
    cfg["stiffness_max_npr"] = ceiling
    cs = estimate_cornering_stiffness(slip, forces, state, params, parsed_data.get("laps", []))
    rejected = _check_band(cs, candidates, cfg["stiffness_min_npr"], ceiling)
    assert (rejected > ceiling).any()
