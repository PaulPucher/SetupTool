# NIS tyre-mismatch health gate. PROVISIONAL (config/parameters.json
# nis_gate namespace, every threshold commented as such -- five data
# points from one session, thesis_notes.md "Phase 4: NIS tyre-mismatch
# gate -- results"). Ports diagnostics/inspect_nis_tyre_mismatch_gate.py's
# WP-N3 prototype into a reusable module -- same windowed-NIS-exceedance
# statistic, same acceptance band, now with an explicit "warn" tier
# between "pass" (trust EKF beta) and "fail" (fall back to kinematic).
#
# Answers: "does the fitted tyre curve match this session's data well
# enough to trust EKF beta?" Reuses the EKF's own windowed-NIS
# machinery (a rolling exceedance-fraction check) as a continuous
# session-level score rather than a new statistic -- see the WP-N3
# Phase 4 pre-registration/results entries in thesis_notes.md for the
# full design rationale and the two pre-registered predictions that
# were tested and found to have FAILED (recorded there, not restated
# here): the score's absolute ceiling is far below the naive ~85%
# expectation (small-window binomial noise caps it near 60-65% even
# for a perfectly calibrated filter), and mu_fz mismatches hurt scoring
# MORE than c_alpha mismatches, not less as originally predicted.

import numpy as np
from scipy.stats import chi2

_CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))


def compute_health_score(nis_combined_full, mask, window_samples, band_low, band_high):
    """Compute the rolling trailing-window (width window_samples)
    combined-NIS exceedance fraction at every sample; return the
    fraction of MASKED samples whose local window's exceedance
    fraction sits inside [band_low, band_high]. nis_combined_full is
    the EKF's full-length nis array (not pre-masked) so the rolling
    window can look back across mask boundaries exactly like the
    EKF's own divergence monitor does. Returns NaN if the window is
    longer than the data or no masked sample has a fully-populated
    window (degenerate/short-session inputs) -- callers must not
    treat NaN as a passing score.
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
    """Classify health_score by pure threshold logic, split out from
    compute_health_score so boundary behaviour can be unit-tested without
    a real EKF run.
    NaN (score != score) always classifies 'fail' -- never 'pass',
    matching the "never silently pass" requirement for degenerate
    inputs. Boundaries are >= on both sides (a score exactly at
    threshold_use_ekf passes, exactly at threshold_warn warns).
    """
    if health_score != health_score:
        return "fail"
    if health_score >= threshold_use_ekf:
        return "pass"
    if health_score >= threshold_warn:
        return "warn"
    return "fail"


def evaluate_gate(nis_combined_full, mask, params):
    """Evaluate the gate as the top-level entry point. params is the raw
    or accuracy-resolved parameters dict (reads params["nis_gate"] only).
    Returns a dict with the verdict plus every number behind it, so a
    caller (the production analysis thread, a diagnostic script) never
    has to re-derive what produced the verdict.
    """
    cfg = params["nis_gate"]
    window_samples = int(cfg["window_samples"])
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
