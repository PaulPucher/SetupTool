# PLAN.md STEP 3 (LS_ratio), Phase 2 -- unit tests on
# modules/longitudinal_stiffness.py, hand-computed expected values
# where a closed form exists, same discipline as tests/
# test_pure_functions.py (see that file's own docstring on what a
# passing test does and does not establish: this checks the code
# against the chair's OWN stated method, not an independent claim
# that the method is the right one for this car).
#
# The 50 Hz-vs-chair-defaults short-window finding (thesis_notes.md
# "PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 2 --
# estimator") led to a decided, documented adaptation (thesis_notes.md
# "PLAN.md STEP 3: 50 Hz min_samples adaptation"): min_samples is no
# longer a transplanted chair literal, it is derived at runtime from
# the chair's own PHYSICAL window (regression_window_s, unchanged) and
# the actual sample rate, floored at min_samples_floor. The test that
# used to pin "never validates at 50 Hz" (test_real_config_at_50hz_
# never_reaches_min_samples) is RETIRED -- that was correct behaviour
# for the pre-adaptation code and is now intentional history, not
# current truth -- and replaced below by
# test_real_config_at_50hz_now_validates_with_rate_derived_min_samples.

import numpy as np
import pytest

from modules.longitudinal_stiffness import (
    estimate_longitudinal_stiffness, _stiffness_ratio, _centered_slopes,
    _plausibility_exclude_mask, _az_disturbed_recently,
)
from modules.stability_analysis import load_parameters


def _se(cutoff_hz=20.0, regression_window_s=0.45, min_samples_floor=15,
        min_slip_span=0.004, linear_slip_threshold=0.015, min_speed_mps=5.0,
        plausibility_kappa_bound=0.12, plausibility_az_window_front_s=0.15,
        plausibility_az_window_rear_s=0.6):
    return {
        "cutoff_hz": cutoff_hz,
        "regression_window_s": regression_window_s,
        "min_samples_floor": min_samples_floor,
        "min_slip_span": min_slip_span,
        "linear_slip_threshold": linear_slip_threshold,
        "min_speed_mps": min_speed_mps,
        "plausibility_kappa_bound": plausibility_kappa_bound,
        "plausibility_az_window_front_s": plausibility_az_window_front_s,
        "plausibility_az_window_rear_s": plausibility_az_window_rear_s,
    }


def _kerb_se(kerb_z_deviation_threshold_g=1.2, kerb_baseline_g=1.0):
    return {
        "kerb_z_deviation_threshold_g": kerb_z_deviation_threshold_g,
        "kerb_baseline_g": kerb_baseline_g,
    }


def _params(se, kerb_se=None):
    return {"longitudinal_stiffness": se, "stability_estimation": kerb_se or _kerb_se()}


def _state(n, sample_rate_hz=100.0, v_mps=30.0):
    return {"sample_rate_hz": sample_rate_hz, "v_mps": np.full(n, v_mps, dtype=float)}


# --- hand-computed slope on a noiseless linear ramp --------------------------

def test_linear_ramp_recovers_exact_stiffness_and_ratio_one():
    """Fx = C_true * kappa exactly, no saturation anywhere -- the
    windowed OLS slope must recover C_true (to numerical tolerance) at
    every interior sample where the window is fully populated, and
    LS_ratio must be 1.0 there (globally linear, so the low-slip
    reference equals the everywhere-slope)."""
    n = 400
    sr = 100.0
    se = _se(cutoff_hz=20.0, regression_window_s=0.45, min_samples_floor=15,
              min_slip_span=0.004, linear_slip_threshold=0.03, min_speed_mps=5.0)
    C_true = 850_000.0
    kappa = np.linspace(-0.05, 0.05, n)
    fx = C_true * kappa

    long_forces = {"fx_f_N": fx, "fx_r_N": fx}
    slip = {"kappa_f": kappa, "kappa_r": kappa}
    state = _state(n, sample_rate_hz=sr)
    result = estimate_longitudinal_stiffness(long_forces, slip, state, _params(se))

    half_window = max(2, int(round(se["regression_window_s"] * sr / 2.0)))
    interior = slice(half_window + 5, n - half_window - 5)

    assert np.all(result["valid_f"][interior]), "interior samples must satisfy the window/span/count gates"
    assert result["stiffness_f"][interior] == pytest.approx(C_true, rel=1e-6)
    assert result["LS_ratio_f"][interior] == pytest.approx(1.0, rel=1e-6)


def test_ratio_clips_at_one_when_stiffness_exceeds_linear_reference():
    """A synthetic super-linear region (slope rises above the low-slip
    reference slope) must clip LS_ratio at exactly 1.0, never report a
    value above it -- calculate_longitudinal_stiffness_ratio's own
    documented np.clip(ratio, None, 1.0)."""
    n = 400
    sr = 100.0
    se = _se(cutoff_hz=20.0, regression_window_s=0.45, min_samples_floor=15,
              min_slip_span=0.004, linear_slip_threshold=0.01, min_speed_mps=5.0)
    kappa = np.linspace(-0.06, 0.06, n)
    # Piecewise: gentle slope inside +-0.01 (the linear reference region),
    # much steeper slope outside it -- the windows centred outside the
    # linear region see a higher local OLS slope than the reference.
    C_low, C_high = 300_000.0, 900_000.0
    fx = np.where(np.abs(kappa) <= 0.01, C_low * kappa,
                  np.sign(kappa) * (C_low * 0.01 + C_high * (np.abs(kappa) - 0.01)))

    long_forces = {"fx_f_N": fx, "fx_r_N": fx}
    slip = {"kappa_f": kappa, "kappa_r": kappa}
    state = _state(n, sample_rate_hz=sr)
    result = estimate_longitudinal_stiffness(long_forces, slip, state, _params(se))

    ratio = result["LS_ratio_f"]
    valid = result["valid_f"]
    finite = valid & np.isfinite(ratio)
    assert finite.any(), "expected at least one valid, finite ratio sample"
    assert np.nanmax(ratio[finite]) <= 1.0 + 1e-9
    # the steep outer region's raw slope is well above the reference --
    # confirm clipping actually engaged, not just coincidentally <=1.0
    outer = finite & (np.abs(kappa) > 0.02)
    assert outer.any()
    assert np.allclose(ratio[outer], 1.0, atol=1e-6)


# --- below-threshold linear-reference fallback rule ---------------------------

def test_stiffness_ratio_falls_back_to_positive_median_when_no_linear_samples():
    """calculate_longitudinal_stiffness_ratio's own documented fallback:
    when no sample's |slip| sits inside linear_slip_threshold, the
    reference falls back to the median of ALL valid positive-stiffness
    samples instead of leaving the ratio all-NaN."""
    stiffness = np.array([100.0, 200.0, 300.0, 400.0, np.nan])
    slip_filtered = np.array([0.05, 0.06, 0.07, 0.08, 0.09])  # all beyond any reasonable threshold
    valid = np.array([True, True, True, True, True])
    linear_slip_threshold = 0.015  # no sample qualifies

    ratio, reference = _stiffness_ratio(stiffness, slip_filtered, valid, linear_slip_threshold)

    assert reference == pytest.approx(np.median([100.0, 200.0, 300.0, 400.0]))
    assert ratio[0] == pytest.approx(100.0 / reference)
    assert np.isnan(ratio[4])  # NaN stiffness in, NaN ratio out


def test_stiffness_ratio_all_nan_when_no_positive_stiffness_at_all():
    """No linear-region sample AND no positive-stiffness sample anywhere
    -- reference is undefined (NaN), ratio must be all-NaN, not crash
    and not silently report a fabricated number."""
    stiffness = np.array([-100.0, -200.0, np.nan])
    slip_filtered = np.array([0.05, 0.06, 0.07])
    valid = np.array([True, True, True])
    ratio, reference = _stiffness_ratio(stiffness, slip_filtered, valid, linear_slip_threshold=0.015)
    assert np.isnan(reference)
    assert np.all(np.isnan(ratio))


# --- NaN / short-window / empty-array degradation -----------------------------

def test_empty_array_input_does_not_crash():
    se = _se()
    long_forces = {"fx_f_N": np.array([]), "fx_r_N": np.array([])}
    slip = {"kappa_f": np.array([]), "kappa_r": np.array([])}
    state = {"sample_rate_hz": 50.0, "v_mps": np.array([])}
    result = estimate_longitudinal_stiffness(long_forces, slip, state, _params(se))
    assert len(result["LS_ratio_f"]) == 0
    assert len(result["LS_ratio_r"]) == 0


def test_all_nan_input_does_not_crash_and_stays_nan():
    n = 100
    se = _se(min_samples_floor=10, regression_window_s=0.2)
    nan_arr = np.full(n, np.nan)
    long_forces = {"fx_f_N": nan_arr, "fx_r_N": nan_arr}
    slip = {"kappa_f": nan_arr.copy(), "kappa_r": nan_arr.copy()}
    state = _state(n, sample_rate_hz=100.0)
    result = estimate_longitudinal_stiffness(long_forces, slip, state, _params(se))
    assert not np.any(result["valid_f"])
    assert np.all(np.isnan(result["LS_ratio_f"]))


def test_window_below_floor_still_never_validates():
    """Direct unit test of _centered_slopes's own floor mechanism.
    min_samples is now derived as max(min_samples_floor, half_window+1)
    -- half_window+1 can never exceed the window's own ceiling
    (2*half_window+1), so the ONLY way for min_samples to exceed the
    window ceiling (and therefore for count >= min_samples to be
    unsatisfiable everywhere) is for min_samples_floor itself to exceed
    it, at an unusually low sample rate. PRE-ADAPTATION this test used
    a literal min_samples=25 to force the same structural ceiling; POST-
    ADAPTATION that lever no longer exists (min_samples is derived, not
    freely settable), so this reproduces the same structural guarantee
    via a low sample rate instead -- the mechanism this test protects
    (a window that's too short can never validate, regardless of data)
    is unchanged, only how it is provoked."""
    n = 200
    sr = 5.0  # deliberately very low -- half_window collapses to the code's own floor of 2
    se = _se(regression_window_s=0.45, min_samples_floor=15, min_slip_span=0.0, min_speed_mps=0.0)
    half_window = max(2, int(round(se["regression_window_s"] * sr / 2.0)))
    max_window_samples = 2 * half_window + 1
    assert max_window_samples < se["min_samples_floor"], (
        "test precondition: window ceiling must be below min_samples_floor"
    )

    kappa = np.linspace(-0.05, 0.05, n)
    fx = 500_000.0 * kappa
    valid_mask = np.ones(n, dtype=bool)
    stiffness, valid = _centered_slopes(kappa, fx, valid_mask, sr, se)
    assert not np.any(valid)
    assert np.all(np.isnan(stiffness))


def test_real_config_at_50hz_now_validates_with_rate_derived_min_samples():
    """REPLACES test_real_config_at_50hz_never_reaches_min_samples
    (retired, thesis_notes.md 'PLAN.md STEP 3: 50 Hz min_samples
    adaptation'): that test correctly pinned the PRE-adaptation
    behaviour (chair's literal min_samples=25 unsatisfiable at 50 Hz,
    max window 23 samples) -- true then, no longer true now that
    min_samples is rate-derived, so pinning it further would pin
    retired behaviour as if it were still current, the opposite of
    what a regression test should do. This test pins the NEW rule
    instead, both halves of it against config/parameters.json's real
    longitudinal_stiffness block:
    1. at this car's real 50 Hz, the derived min_samples (max(floor,
       half_window+1) = max(15, 12) = 15) is now BELOW the 23-sample
       window ceiling -- validation is possible again, confirmed here
       both analytically and against a real synthetic regression.
    2. the floor still binds and still refuses below it (covered by
       test_window_below_floor_still_never_validates above, at a
       lower rate) -- restated here in one place for clarity.
    """
    se = load_parameters()["longitudinal_stiffness"]
    sample_rate_hz = 50.0  # this session's own state["sample_rate_hz"], confirmed in thesis_notes.md
    half_window = max(2, int(round(se["regression_window_s"] * sample_rate_hz / 2.0)))
    max_window_samples = 2 * half_window + 1
    derived_min_samples = max(se["min_samples_floor"], half_window + 1)
    assert derived_min_samples <= max_window_samples, (
        f"derived min_samples={derived_min_samples} exceeds the 50 Hz window ceiling "
        f"{max_window_samples} -- the adaptation this test pins no longer holds; update "
        "deliberately if min_samples_floor or regression_window_s changed again"
    )

    # Confirmed against a real synthetic regression, not just the analytic inequality above.
    n = 200
    kappa = np.linspace(-0.05, 0.05, n)
    fx = 500_000.0 * kappa
    valid_mask = np.ones(n, dtype=bool)
    stiffness, valid = _centered_slopes(kappa, fx, valid_mask, sample_rate_hz, se)
    assert np.any(valid), "expected at least one valid window at 50 Hz under the real, current config"


# --- LS plausibility guard (PLAN.md STEP 3 follow-up, 2026-08-30) ------------
# az-coincidence is the load-bearing design constraint: kappa alone must
# NEVER trigger exclusion (that would erase genuine traction signal,
# thesis_notes.md 'LS plausibility guard'). Every test below uses the
# SAME implausible kappa magnitude (0.20, above the 0.12 bound) so the
# only variable between "excluded" and "kept" is az disturbance.

def test_plausibility_guard_excludes_az_coincident_spike():
    n = 50
    sr = 50.0
    kappa = np.zeros(n)
    kappa[25] = 0.20  # implausible: exceeds plausibility_kappa_bound=0.12
    az_g = np.full(n, 1.0)  # kerb_baseline_g
    az_g[24] = 3.0  # kerb-like disturbance one sample before the spike -- inside a 0.15s trailing window at 50 Hz
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, az_g, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=sr)
    assert exclude[25]
    assert not np.any(exclude[:24]) and not np.any(exclude[26:]), "only the coincident sample should be excluded"


def test_plausibility_guard_keeps_spike_without_az_disturbance():
    """SAME kappa excursion magnitude as the test above, but az_g stays
    flat throughout -- must be KEPT (not excluded). This is the design
    constraint's own test: without az-coincidence, a large kappa is
    presumed genuine traction signal (PLAN.md STEP 3 Phase 4's C3
    finding), never excluded on magnitude alone."""
    n = 50
    sr = 50.0
    kappa = np.zeros(n)
    kappa[25] = 0.20
    az_g = np.full(n, 1.0)  # no disturbance anywhere
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, az_g, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=sr)
    assert not exclude[25]


def test_plausibility_guard_disturbance_outside_window_does_not_exclude():
    """A real az disturbance exists, but it is OUTSIDE the trailing
    window -- must not exclude. Confirms the window is actually
    bounded, not an unconditional 'any disturbance ever' check."""
    n = 50
    sr = 50.0
    kappa = np.zeros(n)
    kappa[40] = 0.20
    az_g = np.full(n, 1.0)
    az_g[5] = 3.0  # far earlier in the array, well outside a 0.15s (7-8 sample) trailing window from index 40
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, az_g, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=sr)
    assert not exclude[40]


def test_plausibility_guard_none_az_excludes_nothing():
    """az_g unavailable (state['az_g'] is optional, graceful
    degradation elsewhere in the pipeline) -- az-coincidence cannot be
    confirmed, so the guard must exclude NOTHING, never fall back to
    kappa-alone exclusion."""
    n = 50
    kappa = np.zeros(n)
    kappa[25] = 0.5  # deliberately far beyond the bound
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, None, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=50.0)
    assert not np.any(exclude)


def test_plausibility_guard_nan_kappa_and_az_do_not_crash():
    n = 50
    kappa = np.full(n, np.nan)
    az_g = np.full(n, np.nan)
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, az_g, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=50.0)
    assert exclude.dtype == bool
    assert not np.any(exclude), "NaN kappa/az must never register as 'implausible & disturbed'"


def test_plausibility_guard_empty_arrays_do_not_crash():
    kappa = np.array([])
    az_g = np.array([])
    se_ls = _se(plausibility_kappa_bound=0.12)
    exclude = _plausibility_exclude_mask(kappa, az_g, _kerb_se(), se_ls, window_s=0.15, sample_rate_hz=50.0)
    assert len(exclude) == 0


def test_az_disturbed_recently_is_backward_looking_only():
    """Direct unit test of the window primitive: a disturbance at
    index i must mark i and the following window_samples-1 indices as
    disturbed, and NOTHING before i -- backward-looking from each
    query point, not centred, matching kerb ringdown's own causal
    direction (thesis_notes.md 'Kerb-strike wheel-speed spikes')."""
    n = 20
    az_g = np.full(n, 1.0)
    az_g[10] = 5.0
    disturbed = _az_disturbed_recently(az_g, threshold_g=1.2, baseline_g=1.0, window_samples=5)
    assert not disturbed[9], "before the disturbance must be False"
    assert disturbed[10], "at the disturbance must be True"
    assert disturbed[14], "4 samples after (within the 5-sample trailing window) must be True"
    assert not disturbed[15], "5 samples after (just outside the trailing window) must be False"


def test_estimate_longitudinal_stiffness_end_to_end_guard_recovers_true_slope():
    """Integration-level confirmation: inject one implausible, az-
    coincident outlier into an otherwise-perfectly-linear synthetic
    ramp. The window containing it must still recover close to the
    true slope (the guard excluded the outlier); an identical outlier
    WITHOUT az-coincidence is left to distort the window's slope (the
    design constraint working as intended, not merely inert)."""
    n = 400
    sr = 100.0
    C_true = 850_000.0
    kappa_base = np.linspace(-0.05, 0.05, n)
    fx_base = C_true * kappa_base
    spike_idx = 200
    half_window = max(2, int(round(0.45 * sr / 2.0)))
    probe_idx = spike_idx + half_window - 2  # a window that includes spike_idx but is not centred on it

    se_ls = _se(cutoff_hz=20.0, regression_window_s=0.45, min_samples_floor=15,
                min_slip_span=0.004, linear_slip_threshold=0.03, min_speed_mps=5.0,
                plausibility_kappa_bound=0.12)
    state_base = {"sample_rate_hz": sr, "v_mps": np.full(n, 30.0)}

    # WITH az-coincidence -- guard should exclude the outlier, slope stays close to C_true.
    kappa_guarded = kappa_base.copy()
    kappa_guarded[spike_idx] = 0.9
    az_g = np.full(n, 1.0)
    az_g[spike_idx] = 5.0
    long_forces = {"fx_f_N": fx_base, "fx_r_N": fx_base}
    slip = {"kappa_f": kappa_guarded, "kappa_r": kappa_guarded}
    params = _params(se_ls)
    result_guarded = estimate_longitudinal_stiffness(
        long_forces, slip, {**state_base, "az_g": az_g}, params
    )
    assert result_guarded["stiffness_f"][probe_idx] == pytest.approx(C_true, rel=0.05)

    # SAME outlier, NO az disturbance anywhere -- design constraint means it is NOT
    # excluded, so it is free to distort this window's slope away from C_true.
    az_g_flat = np.full(n, 1.0)
    result_unguarded = estimate_longitudinal_stiffness(
        long_forces, slip, {**state_base, "az_g": az_g_flat}, params
    )
    assert result_unguarded["stiffness_f"][probe_idx] != pytest.approx(C_true, rel=0.05)
