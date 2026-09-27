# LS_ratio per axle: windowed dFx/dkappa / low-slip reference, clipped at 1.
# Same scale as CS_ratio: 1 = linear, 0 = force peak, <0 = past peak. SI units.
# After the chair performance_analysis tooling (internal).

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
    """True where the kerb flag (|az - baseline| > threshold) fired at i or in
    the window_samples before it. Backward only -- ringdown follows the strike.
    Prefix sums, not np.convolve: output length always len(az_g).
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
    """Exclude where |kappa| > bound AND az disturbed within the trailing
    window_s (per axle, rear rings down longer).

    Never exclude on kappa alone -- big slip without kerb = real traction limit.
    No az_g -> nothing excluded.
    """
    n = len(kappa_raw)
    if az_g is None or n == 0:
        return np.zeros(n, dtype=bool)
    window_samples = max(1, int(round(window_s * sample_rate_hz)))
    disturbed = _az_disturbed_recently(
        az_g, se["kerb_z_deviation_threshold_g"], se["kerb_baseline_g"], window_samples
    )
    implausible = np.abs(kappa_raw) > ls["plausibility_kappa_bound"]  # NaN compares False
    return implausible & disturbed


def _prefix_sum(values):
    return np.concatenate(([0.0], np.cumsum(values, dtype=float)))


def _window_sum(prefix, start, stop):
    return prefix[stop] - prefix[start]


def resolve_ls_min_window_samples(ls, sample_rate_hz):
    """min_window_s -> samples at this log's rate, floored at
    min_window_samples_floor. Takes the ls sub-dict, not full params (unlike
    the CS twin).
    """
    return max(ls["min_window_samples_floor"], int(round(ls["min_window_s"] * sample_rate_hz)))


def reconstruct_ls_window_start(kappa, i, min_window, min_span, s_m=None, max_window_m=None):
    """Start index of the adaptive window ending at i (kappa twin of
    reconstruct_cs_window_start). Grows back from min_window until kappa
    span >= min_span or window >= max_window_m of track.
    No s_m/max_window_m -> no distance cap.
    """
    start = i - min_window
    s_i = s_m[i - 1] if (s_m is not None and max_window_m is not None) else None
    if s_i is not None and not np.isfinite(s_i):
        s_i = None
    # window grows one sample per step -> running extrema exact, no rescan
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
    """Per-sample OLS slope dFx/dkappa over an adaptive trailing window.
    Widens until kappa span >= min_slip_span; cap in track metres, not
    samples. Never qualifies -> NaN, same as CS.
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
            continue  # span floor not reached -> no signal

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
    """LS_ratio_f/r = windowed dFx/dkappa / median slope in each axle's
    low-slip region, clipped at 1.0. Cutoff and windows from config.
    """
    ls = params["longitudinal_stiffness"]
    se = params["stability_estimation"]
    sr = state["sample_rate_hz"]
    v_mps = state["v_mps"]
    az_g = state.get("az_g")
    s_m = state.get("s_m")

    speed_valid = v_mps >= ls["min_speed_mps"]

    def compute_for_axle(kappa_raw, fx_raw, plausibility_window_s):
        # kerb spikes -> NaN before filtering, filtfilt would smear them
        exclude = _plausibility_exclude_mask(kappa_raw, az_g, se, ls, plausibility_window_s, sr)
        kappa_for_filter = np.where(exclude, np.nan, kappa_raw)

        kappa_filt = _filtered(kappa_for_filter, sr, ls["cutoff_hz"])
        fx_filt = _filtered(fx_raw, sr, ls["cutoff_hz"])
        valid_mask = np.isfinite(kappa_raw) & np.isfinite(fx_raw) & speed_valid & ~exclude  # interpolated != measured

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
