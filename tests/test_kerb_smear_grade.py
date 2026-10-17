# Q-CAL-1 options 0 + 4 (PLAN.md Q-CAL item 1, author rulings 2026-10-03).
# Real Dubai data only. The kinematic sideslip is used instead of the
# pipeline fixture's EKF chain: the mark and the mask do not depend on the
# beta source, and it keeps these tests seconds instead of minutes.

import json

import numpy as np

from modules.stability_analysis import (
    _build_non_analysis_lap_mask, _compute_kerb_mask_from_az, _kerb_smear_zone, _seconds_to_samples,
    estimate_cornering_stiffness, estimate_lateral_forces, estimate_sideslip,
    estimate_slip_angles, estimate_yaw_moment_stability, reconstruct_cs_window_start,
    resolve_cs_min_window_samples, summarise_corners,
)


def _runs(mask):
    m = np.concatenate(([False], mask, [False])).astype(np.int8)
    d = np.diff(m)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def _cs(state, params, laps):
    beta = estimate_sideslip(state, params)
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    return beta, slip, estimate_cornering_stiffness(slip, forces, state, params, laps)


def test_kerb_dilation_is_time_based(parsed_data, effective_params):
    """The padding around every kerb run is kerb_dilation_s in seconds at both
    50 and 100 Hz -- the old sample count gave 100 ms at 50 Hz but 50 ms at
    100 Hz. Real Dubai az, interpolated onto each grid as prepare_vehicle_state
    does."""
    se = effective_params["stability_estimation"]
    az = parsed_data["channels"]["log_acc_z"]
    t0, t1 = az["time"][0], az["time"][-1]
    for sr in (50.0, 100.0):
        t = np.linspace(t0, t1, int(round((t1 - t0) * sr)) + 1)
        az_g = np.interp(t, az["time"], az["data"])
        dil = _seconds_to_samples(se["kerb_dilation_s"], sr)
        mask = _compute_kerb_mask_from_az(az_g, se["kerb_z_deviation_threshold_g"], se["kerb_baseline_g"], dil)
        raw = np.abs(az_g - se["kerb_baseline_g"]) > se["kerb_z_deviation_threshold_g"]
        interior = [(a, b) for a, b in _runs(mask) if a > 0 and b < mask.size - 1]
        assert interior, f"no kerb runs at {sr} Hz -- Dubai must contain kerb strikes"
        for a, b in interior:
            hit = np.flatnonzero(raw[a:b + 1])
            pre_s, post_s = hit[0] / sr, (b - a - hit[-1]) / sr
            assert abs(pre_s - se["kerb_dilation_s"]) < 1e-9
            assert abs(post_s - se["kerb_dilation_s"]) < 1e-9


def test_smear_zone_matches_definition(state, effective_params):
    """Zone = unmasked samples within kerb_smear_grade_s of a mask sample,
    either side -- checked against a direct nearest-mask-distance count."""
    se = effective_params["stability_estimation"]
    mask = state["kerb_mask"]
    g = _seconds_to_samples(se["kerb_smear_grade_s"], state["sample_rate_hz"])
    midx = np.flatnonzero(mask)
    assert midx.size > 0
    idx = np.arange(mask.size)
    k = np.searchsorted(midx, idx)
    d_left = np.where(k > 0, idx - midx[np.clip(k - 1, 0, midx.size - 1)], np.iinfo(np.int64).max)
    d_right = np.where(k < midx.size, midx[np.clip(k, 0, midx.size - 1)] - idx, np.iinfo(np.int64).max)
    expected = (np.minimum(d_left, d_right) <= g) & ~mask
    assert np.array_equal(state["kerb_smear_zone"], expected)
    assert np.array_equal(_kerb_smear_zone(mask, g), expected)


def test_near_strike_marks_exactly_the_zone_windows(state, effective_params, parsed_data):
    """near_strike[i] is True exactly when the production CS window [start, i)
    of a finite CS sample holds a zone sample; False wherever CS is NaN."""
    se = effective_params["stability_estimation"]
    _beta, slip, cs = _cs(state, effective_params, parsed_data.get("laps", []))
    zone = state["kerb_smear_zone"]
    min_window = resolve_cs_min_window_samples(effective_params, state["sample_rate_hz"])
    marked = 0
    for ax in ("f", "r"):
        alpha = slip[f"alpha_{ax}_filt"]
        ratio, mark = cs[f"CS_ratio_{ax}"], cs[f"near_strike_{ax}"]
        assert not mark[~np.isfinite(ratio)].any()
        for i in np.flatnonzero(np.isfinite(ratio)):
            start = reconstruct_cs_window_start(alpha, int(i), min_window, se["cs_min_slip_angle_span_rad"],
                                                state["s_m"], se["cs_max_window_m"])
            assert mark[i] == bool(zone[start:i].any()), f"axle {ax} sample {i}"
        marked += int(mark.sum())
    assert marked > 0, "Dubai has kerb strikes in corners; some CS windows must carry the mark"


def test_near_strike_is_flag_only(state, effective_params, parsed_data):
    """The smear zone stays out of the regression but not out of the
    reference (Class B Fix 0): (a) removing the zone changes no stiffness
    array; (b) every linear reference in force was set by a clean window --
    no kerb, zone or off-analysis-lap sample in its [start, i). The corner
    summaries follow the clean-population rule (2026-10-05): their CS
    statistics skip the marked samples, exactly as if those samples were NaN."""
    se = effective_params["stability_estimation"]
    laps = parsed_data.get("laps", [])
    beta, slip, cs = _cs(state, effective_params, laps)
    forces = estimate_lateral_forces(state, effective_params)
    bare = dict(state, kerb_smear_zone=None)
    cs_bare = estimate_cornering_stiffness(slip, forces, bare, effective_params, laps)
    for k in ("C_alpha", "C_window", "C_section", "R2"):
        for ax in "fr":
            assert np.array_equal(cs[f"{k}_{ax}"], cs_bare[f"{k}_{ax}"], equal_nan=True), f"{k}_{ax}"
    assert not cs_bare["near_strike_f"].any() and not cs_bare["near_strike_r"].any()

    blocked = state["kerb_mask"] | state["kerb_smear_zone"] | _build_non_analysis_lap_mask(state["time"], laps)
    min_window = resolve_cs_min_window_samples(effective_params, state["sample_rate_hz"])
    n_set = 0
    for ax in "fr":
        alpha, ref, c_alpha = slip[f"alpha_{ax}_filt"], cs[f"C_linear_ref_{ax}"], cs[f"C_alpha_{ax}"]
        last = np.nan
        for i in np.flatnonzero(np.isfinite(ref)):
            if ref[i] == last:
                continue
            # a new value in force: set at i by the window the loop used
            start = reconstruct_cs_window_start(alpha, int(i), min_window, se["cs_min_slip_angle_span_rad"],
                                                state["s_m"], se["cs_max_window_m"])
            assert ref[i] == c_alpha[i], f"axle {ax} sample {i}"
            assert not blocked[start:i].any(), f"axle {ax}: reference at {i} set by a dirty window"
            last = ref[i]
            n_set += 1
    assert n_set > 0
    # the zone does reach the reference on Dubai: without it some reference moves
    assert not np.array_equal(cs["C_linear_ref_f"], cs_bare["C_linear_ref_f"], equal_nan=True) or         not np.array_equal(cs["C_linear_ref_r"], cs_bare["C_linear_ref_r"], equal_nan=True)

    stab = estimate_yaw_moment_stability(state, beta, effective_params, laps)
    corners = parsed_data.get("corners", [])
    with_mark = summarise_corners(corners, cs, stab, state)
    unmarked = {k: v for k, v in cs.items() if not k.startswith("near_strike")}
    blanked = dict(unmarked)
    for ax in "fr":
        blanked[f"CS_ratio_{ax}"] = np.where(cs[f"near_strike_{ax}"], np.nan, cs[f"CS_ratio_{ax}"])
    as_if_nan = summarise_corners(corners, blanked, stab, state)
    without = summarise_corners(corners, unmarked, stab, state)
    # n_kerb_excluded (schema 14, display only) counts the marked samples, so
    # it differs from a plain NaN by construction; every statistic must not
    def _drop_count(summ):
        import re
        return re.sub(r', "n_kerb_excluded": \d+', "", json.dumps(summ, sort_keys=True, default=str))
    assert _drop_count(with_mark) == _drop_count(as_if_nan)
    assert any(p[k]["n_kerb_excluded"] > 0 for s in with_mark for p in s["phases"].values()
               for k in ("cs_ratio_f", "cs_ratio_r"))
    # Dubai carries marked samples, so the clean rule must actually show
    assert _drop_count(with_mark) != _drop_count(without)
