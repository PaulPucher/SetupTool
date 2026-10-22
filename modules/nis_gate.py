# NIS tyre-mismatch gate: does the fitted tyre curve fit this session well
# enough to trust EKF beta? pass / warn / fail (-> kinematic beta).
# Session score = share of samples whose windowed NIS exceedance sits in band.
# Thresholds re-derived on the fit-L default 2026-10-20 (14 development
# sessions, config threshold_derived_from); the 2026-10-11 values belong to
# the per_session estimator. Prototyped
# on a Dugoff fit; the prototype script was removed 2026-10-17 with the
# Dugoff path (git history).
#
# Score ceiling ~60-65% even for a perfect filter (small-window binomial noise).

import numpy as np
from scipy.stats import chi2

_CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))

# A window's exceedance fraction is only read when most of it is measured;
# below half valid it is skipped rather than estimated from a few samples.
MIN_WINDOW_VALID_FRACTION = 0.5


def _score_and_skips(nis_combined_full, mask, window_samples, band_low, band_high):
    """(health score, skipped count). Score = fraction of masked samples whose
    trailing-window NIS exceedance lies in [band_low, band_high].
    nis_combined_full is unmasked -- the window looks back across mask gaps,
    like the EKF divergence monitor. Windows less than half covered by
    finite NIS are skipped. NaN on degenerate input; NaN never counts as a
    pass.
    """
    window_samples = int(window_samples)
    n = len(nis_combined_full)
    if window_samples < 1 or window_samples > n or not np.any(mask):
        return float("nan"), 0
    nis = np.asarray(nis_combined_full, dtype=float)
    # NaN NIS (no EKF update, e.g. standing) leaves both numerator and
    # denominator (#24) -- it used to count as a non-exceedance and pull the
    # fraction down.
    finite = np.isfinite(nis)
    exceed = (finite & (nis > _CHI2_DF2_95)).astype(float)
    cw = np.cumsum(np.insert(exceed, 0, 0.0))
    cv = np.cumsum(np.insert(finite.astype(float), 0, 0.0))
    ends = np.arange(window_samples - 1, n)
    n_exceed = cw[ends + 1] - cw[ends + 1 - window_samples]
    n_valid = cv[ends + 1] - cv[ends + 1 - window_samples]
    covered = n_valid >= MIN_WINDOW_VALID_FRACTION * window_samples
    win_frac = np.full(n, np.nan)
    win_frac[ends[covered]] = n_exceed[covered] / n_valid[covered]
    low_coverage = np.zeros(n, dtype=bool)
    low_coverage[ends[~covered]] = True
    m = np.asarray(mask, dtype=bool)
    n_skipped = int((m & low_coverage).sum())
    in_band = (win_frac >= band_low) & (win_frac <= band_high)
    valid = m & np.isfinite(win_frac)
    if not valid.any():
        return float("nan"), n_skipped
    return float(in_band[valid].mean()), n_skipped


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

    health_score, n_skipped = _score_and_skips(nis_combined_full, mask, window_samples, band_low, band_high)
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
        # masked samples whose window was skipped as less than half valid
        "n_windows_skipped_low_coverage": n_skipped,
        "degenerate_reason": degenerate_reason,
    }
