# NIS tyre-mismatch gate: does the fitted tyre curve fit this session well
# enough to trust EKF beta? pass / warn / fail (-> kinematic beta).
# Session score = share of samples whose windowed NIS exceedance sits in band.
# Thresholds provisional (one session). Prototype:
# diagnostics/inspect_nis_tyre_mismatch_gate.py.
#
# Score ceiling ~60-65% even for a perfect filter (small-window binomial noise).

import numpy as np
from scipy.stats import chi2

_CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))


def compute_health_score(nis_combined_full, mask, window_samples, band_low, band_high):
    """Fraction of masked samples whose trailing-window NIS exceedance lies
    in [band_low, band_high]. nis_combined_full is unmasked -- the window
    looks back across mask gaps, like the EKF divergence monitor.
    NaN on degenerate input; NaN never counts as a pass.
    """
    window_samples = int(window_samples)
    n = len(nis_combined_full)
    if window_samples < 1 or window_samples > n or not np.any(mask):
        return float("nan")
    exceed = (nis_combined_full > _CHI2_DF2_95).astype(float)
    cw = np.cumsum(np.insert(exceed, 0, 0.0))
    win_frac = np.full(n, np.nan)
    for i in range(window_samples - 1, n):
        win_frac[i] = (cw[i + 1] - cw[i + 1 - window_samples]) / window_samples
    in_band = (win_frac >= band_low) & (win_frac <= band_high)
    valid = np.asarray(mask, dtype=bool) & np.isfinite(win_frac)
    if not valid.any():
        return float("nan")
    return float(in_band[valid].mean())


def classify_score(health_score, threshold_use_ekf, threshold_warn):
    """Threshold logic only, testable without an EKF run.
    NaN -> 'fail'. Both boundaries inclusive (>=).
    """
    if health_score != health_score:
        return "fail"
    if health_score >= threshold_use_ekf:
        return "pass"
    if health_score >= threshold_warn:
        return "warn"
    return "fail"


def resolve_nis_window_samples(params, sample_rate_hz):
    """nis_window_s [s] -> samples at this grid rate. No floor needed:
    min_sample_rate_hz (50 Hz) already keeps the count sane.
    """
    return int(round(params["nis_gate"]["nis_window_s"] * sample_rate_hz))


def evaluate_gate(nis_combined_full, mask, params, sample_rate_hz):
    """Entry point. Returns the verdict plus every number behind it."""
    cfg = params["nis_gate"]
    window_samples = resolve_nis_window_samples(params, sample_rate_hz)
    band_low, band_high = cfg["nis_band_low"], cfg["nis_band_high"]
    threshold_use_ekf, threshold_warn = cfg["threshold_use_ekf"], cfg["threshold_warn"]

    n = len(nis_combined_full)
    masked_n = int(np.sum(np.asarray(mask, dtype=bool)))
    degenerate_reason = None
    if window_samples > n:
        degenerate_reason = f"window_samples ({window_samples}) exceeds session length ({n})"
    elif masked_n == 0:
        degenerate_reason = "mask selects zero samples"

    health_score = compute_health_score(nis_combined_full, mask, window_samples, band_low, band_high)
    if degenerate_reason is None and health_score != health_score:
        degenerate_reason = "health score is NaN -- window/mask produced no valid in-band measurement"

    verdict = classify_score(health_score, threshold_use_ekf, threshold_warn)

    return {
        "verdict": verdict,
        "health_score": health_score,
        "window_samples": window_samples,
        "nis_band_low": band_low,
        "nis_band_high": band_high,
        "threshold_use_ekf": threshold_use_ekf,
        "threshold_warn": threshold_warn,
        "masked_population_n": masked_n,
        "degenerate_reason": degenerate_reason,
    }
