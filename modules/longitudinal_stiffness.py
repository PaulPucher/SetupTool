# Longitudinal stiffness ratio (LS_ratio) for SetupTool.
# Pure Python/numpy/scipy. No Qt imports.
# Units: SI throughout (m, s, N, kg), kappa/LS_ratio dimensionless.
#
# PLAN.md STEP 3 (LS_ratio), Phase 2. Mirrors the chair performance_
# analysis tooling's own estimate_longitudinal_stiffness /
# calculate_longitudinal_stiffness_ratio (docs/literature/longitudinal_
# stiffness_estimator.py, internal) structure directly: Butterworth
# low-pass on slip ratio and longitudinal force, a sliding-window local
# least-squares slope dFx/dkappa (prefix-sum implementation, same as
# the chair's own _centered_slopes), reported as a ratio against a
# low-slip linear reference, clipped at 1.0 -- the SAME scale
# estimate_cornering_stiffness (modules/stability_analysis.py) reports
# CS_ratio on (1 = linear region, 0 = at the peak, below 0 = beyond
# it). Every tunable below is read from config/parameters.json's
# longitudinal_stiffness namespace, chair's own dataclass defaults,
# unchanged -- TWO DOCUMENTED DEVIATIONS, both decided 2026-08-30, full
# record thesis_notes.md. (1) min_samples is no longer a
# transplanted literal (the chair's 25 is structurally unsatisfiable at
# this car's 50 Hz log, proven in thesis_notes.md's Phase 2 entry --
# max window at 50 Hz is 23 samples). _centered_slopes below derives it
# from the chair's own PHYSICAL window (regression_window_s, preserved
# unchanged) and the actual log rate instead; see that function's own
# comment and thesis_notes.md "PLAN.md STEP 3: 50 Hz min_samples
# adaptation" for the forcing fact and reasoning. (2) a NEW, additive
# plausibility guard (no chair equivalent -- SetupTool-specific, forced
# by log_speed_*'s own known kerb-spike behaviour on this car's data,
# thesis_notes.md "Kerb-strike wheel-speed spikes"): _apply_
# plausibility_guard excludes a sample from the regression windows
# only when its kappa is implausible AND the vertical-acceleration
# channel shows kerb-like disturbance nearby -- see that function's
# own docstring for the load-bearing design constraint (az-coincidence
# required, kappa alone is never sufficient).

import numpy as np
from scipy.signal import butter, filtfilt


def _filtered(values, sample_rate_hz, cutoff_hz):
    import pandas as pd
    series = pd.Series(values).interpolate(limit_direction="both").bfill().ffill()
    values = series.to_numpy(dtype=float)
    if len(values) < 12 or not np.isfinite(sample_rate_hz) or sample_rate_hz <= 0:
        return values
    nyquist = sample_rate_hz / 2.0
    normalized_cutoff = min(max(cutoff_hz / nyquist, 1e-4), 0.95)
    b, a = butter(N=4, Wn=normalized_cutoff, btype="low", analog=False)
    padlen = min(3 * (max(len(a), len(b)) - 1), len(values) - 1)
    if padlen <= 0:
        return values
    return filtfilt(b, a, values, padlen=padlen)


def _az_disturbed_recently(az_g, threshold_g, baseline_g, window_samples):
    """Boolean array: True at sample i if a kerb-like vertical-
    acceleration disturbance (same raw flag modules/stability_
    analysis.py's _compute_kerb_mask_from_az itself uses -- |az_g -
    baseline_g| > threshold_g) occurred at i or in the window_samples
    samples immediately BEFORE it. Backward-looking, not symmetric:
    kerb ringdown happens AFTER the physical strike, so what matters
    for a sample at i is whether a strike happened recently, not
    whether one is coming. Prefix-sum windowed OR (same primitive
    _centered_slopes uses for its own sliding sums below) rather than
    np.convolve -- convolve's 'valid' mode output length depends on
    BOTH input lengths, which silently breaks (mismatched array shape)
    when window_samples exceeds a short/empty input; prefix sums always
    return exactly len(az_g), including at n=0.
    """
    n = len(az_g)
    if n == 0:
        return np.zeros(0, dtype=bool)
    raw = np.abs(az_g - baseline_g) > threshold_g
    idx = np.arange(n)
    start = np.maximum(0, idx - max(window_samples, 1) + 1)
    stop = idx + 1
    counts = _window_sum(_prefix_sum(raw.astype(float)), start, stop)
    return counts > 0


def _plausibility_exclude_mask(kappa_raw, az_g, se, ls, window_s, sample_rate_hz):
    """Which samples the LS plausibility guard excludes: |kappa_raw|
    exceeds ls['plausibility_kappa_bound'] AND az_g shows kerb-like
    disturbance within the trailing window_s seconds (axle-specific --
    rear rings down far longer than front, thesis_notes.md 'Kerb-
    strike wheel-speed spikes' PART 1b).

    DESIGN CONSTRAINT, load-bearing: this guard keys on az-coincidence,
    NEVER on kappa alone. A large kappa excursion with no az
    disturbance nearby is exactly the traction-limited signal this
    estimator exists to measure (PLAN.md STEP 3 Phase 4's C3 finding)
    -- excluding on kappa magnitude alone would silently erase that
    signal, the specific failure mode thesis_notes.md's kerb
    investigation entry warned against when it recommended AGAINST a
    general-purpose hybrid kerb detector for exactly this reason.

    az_g is optional (state['az_g'] can be None; other pipeline stages
    already handle its absence) -- when absent, az-coincidence cannot
    be confirmed, so nothing is excluded (fails toward the pre-guard
    behaviour, never toward over-exclusion on kappa alone).

    Caller applies this mask in TWO places, not one -- see
    estimate_longitudinal_stiffness's compute_for_axle: (1) the
    excluded raw kappa is set to NaN BEFORE Butterworth filtering, so
    _filtered's own interpolate-over-NaN step bridges over it instead
    of letting filtfilt (a whole-array, zero-phase operation) smear
    the outlier's energy into neighbouring samples' FILTERED values;
    (2) the same index is also excluded from the window-sum valid_mask,
    since even its interpolated filtered value is not a real
    measurement. Guarding only the window sums (an earlier version of
    this function did that alone) leaves the filtered signal itself
    corrupted near the excluded sample -- confirmed by test_estimate_
    longitudinal_stiffness_end_to_end_guard_recovers_true_slope failing
    against that design before this fix.
    """
    n = len(kappa_raw)
    if az_g is None or n == 0:
        return np.zeros(n, dtype=bool)
    window_samples = max(1, int(round(window_s * sample_rate_hz)))
    disturbed = _az_disturbed_recently(
        az_g, se["kerb_z_deviation_threshold_g"], se["kerb_baseline_g"], window_samples
    )
    implausible = np.abs(kappa_raw) > ls["plausibility_kappa_bound"]  # NaN-safe: np.abs(nan) > x is False, not NaN
    return implausible & disturbed


def _prefix_sum(values):
    return np.concatenate(([0.0], np.cumsum(values, dtype=float)))


def _window_sum(prefix, start, stop):
    return prefix[stop] - prefix[start]


def resolve_ls_min_window_samples(ls, sample_rate_hz):
    """LS validity repair, Phase 1 (2026-09-19, thesis_notes.md 'Metrology
    extension Phase 2: LS_ratio validity repair'), mirroring modules.
    stability_analysis.resolve_cs_min_window_samples exactly: min_window_s
    is a PHYSICAL window duration, not a sample count, converted to
    samples at THIS file's own measured rate, floored at min_window_
    samples_floor so an unusually low sample rate cannot validate a
    near-empty window. `ls` is the longitudinal_stiffness config sub-dict
    directly (this module's own existing convention -- _apply_
    plausibility_guard and friends take sub-dicts, not the full params),
    not the top-level params dict CS's own analogous function takes.
    """
    return max(ls["min_window_samples_floor"], int(round(ls["min_window_s"] * sample_rate_hz)))


def reconstruct_ls_window_start(kappa, i, min_window, min_span, s_m=None, max_window_m=None):
    """Reconstruct the adaptive window's own start index for target index
    `i` -- mirrors modules.stability_analysis.reconstruct_cs_window_start
    exactly (kappa in place of alpha), and _centered_slopes's own internal
    growth loop below calls this directly rather than duplicating it, so
    the two can never drift apart. min_window is already a resolved
    SAMPLE COUNT (see resolve_ls_min_window_samples). s_m/max_window_m
    cap how far the window may grow by real track distance travelled
    (metres, not samples) -- omitting either falls back to NO distance
    cap, safe only when called on an index already known to carry a
    finite LS_ratio.
    """
    start = i - min_window
    s_i = s_m[i - 1] if (s_m is not None and max_window_m is not None) else None
    if s_i is not None and not np.isfinite(s_i):
        s_i = None
    # WP-PERF (2026-09-23, thesis_notes.md): the window only ever grows by
    # one element per iteration (start decrements, i is fixed, no break
    # path mutates start) -- an incrementally maintained running max/min
    # is therefore exact, not an approximation: max/min composition has no
    # floating-point rounding and np.maximum/np.minimum propagate NaN the
    # same order-independent way np.max/np.min's own reduce does. This
    # replaces re-scanning the whole growing slice with np.max/np.min
    # (O(window) numpy array reduction) at every widening step with an
    # O(1) scalar ufunc update -- profiled as >95% of this function's own
    # cost before this change (thesis_notes.md, "WP-PERF Phase 0").
    if start > 0:
        window_max = np.max(kappa[start:i])
        window_min = np.min(kappa[start:i])
    while start > 0:
        span = window_max - window_min
        if span >= min_span:
            break
        if s_i is not None:
            s_start = s_m[start]
            if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                break
        start -= 1
        window_max = np.maximum(window_max, kappa[start])
        window_min = np.minimum(window_min, kappa[start])
    return max(start, 0)


def _centered_slopes(slip, force, valid_mask, sample_rate_hz, se, s_m=None):
    """LS validity repair (2026-09-19, thesis_notes.md 'Metrology
    extension Phase 2: LS_ratio validity repair'), applying the CS_ratio
    playbook: min_window_s/min_window_samples_floor/min_slip_span are now
    FLOORS, not a fixed window -- the window ADAPTIVELY WIDENS past the
    floor until it clears min_slip_span, capped at max_window_m (a real
    track DISTANCE, not a sample count, so the cap means the same thing
    at a slow and a fast corner) via reconstruct_ls_window_start. This
    replaces the prior fixed-half-window prefix-sum implementation (chair-
    identical, but with no widening and no span-floor enforcement against
    a genuinely too-narrow window -- see thesis_notes.md for the finding
    that motivated this repair: the CS playbook's own 'min-driven
    statistics amplify garbage windows' failure mode, reproduced here as
    the Phase 4e wholesale-negative finding).

    NaN/no-signal semantics mirror CS exactly: a sample whose window
    cannot clear BOTH floors within the cap reports NaN (no signal), not
    a slope computed from an under-qualified window.
    """
    n = len(slip)
    if n == 0:
        return np.array([]), np.array([], dtype=bool)

    min_window = resolve_ls_min_window_samples(se, sample_rate_hz)
    min_span = se["min_slip_span"]
    max_window_m = se["max_window_m"]

    finite = np.isfinite(slip) & np.isfinite(force) & valid_mask
    x = np.where(finite, slip, 0.0)
    y = np.where(finite, force, 0.0)

    slopes = np.full(n, np.nan)
    valid = np.zeros(n, dtype=bool)

    for i in range(min_window, n):
        if not finite[i]:
            continue
        start = reconstruct_ls_window_start(x, i, min_window, min_span, s_m=s_m, max_window_m=max_window_m)
        window_x = x[start:i]
        window_y = y[start:i]
        achieved_span = np.max(window_x) - np.min(window_x)
        if achieved_span < min_span:
            continue  # widening could not clear the span floor within the cap -- no signal

        x_mean = np.mean(window_x)
        y_mean = np.mean(window_y)
        denom = np.sum((window_x - x_mean) ** 2)
        if denom < 1e-10:
            continue

        slopes[i] = np.sum((window_x - x_mean) * (window_y - y_mean)) / denom
        valid[i] = True

    return slopes, valid


def _stiffness_ratio(stiffness, slip_filtered, valid, linear_slip_threshold):
    stiffness = np.asarray(stiffness, dtype=float)
    slip_filtered = np.asarray(slip_filtered, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    linear_mask = (
        valid
        & np.isfinite(stiffness)
        & (stiffness > 0)
        & (np.abs(slip_filtered) <= linear_slip_threshold)
    )
    if np.any(linear_mask):
        reference = float(np.nanmedian(stiffness[linear_mask]))
    else:
        positive = valid & np.isfinite(stiffness) & (stiffness > 0)
        reference = float(np.nanmedian(stiffness[positive])) if np.any(positive) else np.nan

    ratio = np.full_like(stiffness, np.nan, dtype=float)
    if np.isfinite(reference) and abs(reference) > 1e-9:
        ratio = stiffness / reference
        ratio = np.clip(ratio, None, 1.0)
    return ratio, reference


def estimate_longitudinal_stiffness(long_forces, slip, state, params):
    """Module 4b-equivalent: LS_ratio_f/LS_ratio_r, per-sample dFx/dkappa
    ratio against each axle's own low-slip linear reference.

    Tier A/B split, same as Module 4b's own CS_ratio: the ratio
    construction itself (windowed OLS slope, low-slip reference,
    clip-at-1.0) is the chair's estimator, adopted as-is (no
    literature-anchored physical claim of its own beyond the slip-
    ratio/longitudinal-force relationship already anchored for the
    inputs -- thesis_notes.md, "Citation cross-reference, modules/
    longitudinal_forces.py" entry); the Butterworth pre-filter and
    sliding-window parameters are Tier B signal conditioning, config-driven
    (longitudinal_stiffness namespace), chair-sourced defaults --
    EXCEPT min_samples, rate-derived rather than chair-sourced (see
    _centered_slopes's own comment and thesis_notes.md), a documented
    deviation forced by this car's 50 Hz log.
    """
    ls = params["longitudinal_stiffness"]
    se = params["stability_estimation"]
    sr = state["sample_rate_hz"]
    v_mps = state["v_mps"]
    az_g = state.get("az_g")
    s_m = state.get("s_m")

    speed_valid = v_mps >= ls["min_speed_mps"]

    def compute_for_axle(kappa_raw, fx_raw, plausibility_window_s):
        # LS plausibility guard (PLAN.md STEP 3 follow-up, decided
        # 2026-08-30 -- thesis_notes.md 'LS plausibility guard'): az-
        # coincident implausible kappa excluded from the regression
        # windows, never on kappa magnitude alone (see _plausibility_
        # exclude_mask's own docstring). Applied BEFORE filtering (NaN
        # the excluded raw sample so _filtered's own interpolate step
        # bridges over it, instead of letting filtfilt smear the
        # outlier into neighbouring FILTERED samples) as well as after
        # (excluded from the window-sum valid_mask too).
        exclude = _plausibility_exclude_mask(kappa_raw, az_g, se, ls, plausibility_window_s, sr)
        kappa_for_filter = np.where(exclude, np.nan, kappa_raw)

        kappa_filt = _filtered(kappa_for_filter, sr, ls["cutoff_hz"])
        fx_filt = _filtered(fx_raw, sr, ls["cutoff_hz"])
        valid_mask = np.isfinite(kappa_raw) & np.isfinite(fx_raw) & speed_valid & ~exclude

        stiffness, valid = _centered_slopes(kappa_filt, fx_filt, valid_mask, sr, ls, s_m=s_m)
        ratio, reference = _stiffness_ratio(stiffness, kappa_filt, valid, ls["linear_slip_threshold"])

        return {
            "kappa_filt": kappa_filt,
            "fx_filt": fx_filt,
            "stiffness": stiffness,
            "valid": valid,
            "LS_ratio": ratio,
            "linear_reference_N": reference,
        }

    front = compute_for_axle(slip["kappa_f"], long_forces["fx_f_N"], ls["plausibility_az_window_front_s"])
    rear = compute_for_axle(slip["kappa_r"], long_forces["fx_r_N"], ls["plausibility_az_window_rear_s"])

    return {
        "kappa_f_filt": front["kappa_filt"],
        "kappa_r_filt": rear["kappa_filt"],
        "fx_f_filt": front["fx_filt"],
        "fx_r_filt": rear["fx_filt"],
        "stiffness_f": front["stiffness"],
        "stiffness_r": rear["stiffness"],
        "valid_f": front["valid"],
        "valid_r": rear["valid"],
        "LS_ratio_f": front["LS_ratio"],
        "LS_ratio_r": rear["LS_ratio"],
        "linear_reference_f_N": front["linear_reference_N"],
        "linear_reference_r_N": rear["linear_reference_N"],
    }
