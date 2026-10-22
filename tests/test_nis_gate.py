# Tests for modules/nis_gate.py: window rate-derivation, boundary
# classification, the live thresholds against the real-session scores, and
# NaN/short-session degradation to "fail".
#
# 2026-10-17 (Class B rebuild, Dugoff removal): the synthetic tyre-mismatch
# scenarios (healthy Dugoff fit on Dubai, c_alpha/mu_fz scaled 0.5x/2x) were
# deleted with the Dugoff path. They had been skipped since 2026-09-02 (the
# Dugoff rear fit degenerated on Dubai); the degradation tests that only
# borrowed that fixture for params and the grid rate now run on the real
# Dubai state instead.

import numpy as np
import pytest

from modules.stability_analysis import load_parameters
from modules.nis_gate import _score_and_skips, classify_score, evaluate_gate, resolve_nis_window_samples


@pytest.fixture(scope="module")
def gate_inputs(state):
    """Live params and the Dubai grid rate -- all the degradation tests need."""
    return {"params": load_parameters(), "sample_rate_hz": state["sample_rate_hz"]}


# --- window rate-derivation (morning follow-up, NIS gate rate-correction) --

def test_resolve_nis_window_samples_at_100hz_is_not_the_old_literal_20():
    # The bug this fix closes: window_samples=20 was a 50Hz-calibrated
    # 0.4s literal, silently representing 0.2s at 100Hz. The corrected
    # resolver must recover the full 0.4s (40 samples) at 100Hz, not 20.
    params = {"nis_gate": {"nis_window_s": 0.4}}
    assert resolve_nis_window_samples(params, 100.0) == 40
    assert resolve_nis_window_samples(params, 50.0) == 20


def test_resolve_nis_window_samples_rounds_to_nearest_sample():
    params = {"nis_gate": {"nis_window_s": 0.4}}
    assert resolve_nis_window_samples(params, 33.0) == round(0.4 * 33.0)


# --- boundary classification (pure function, no EKF run needed) ------------

@pytest.mark.parametrize("score,expected", [
    (0.1385, "pass"),        # exactly at threshold_use_ekf -- boundary is inclusive (>=)
    (0.1385 + 1e-9, "pass"),
    (0.1385 - 1e-9, "warn"),
    (0.1006, "warn"),        # exactly at threshold_warn -- boundary is inclusive (>=)
    (0.1006 + 1e-9, "warn"),
    (0.1006 - 1e-9, "fail"),
    (1.0, "pass"),
    (0.0, "fail"),
])
def test_classify_score_boundaries(score, expected):
    assert classify_score(score, threshold_use_ekf=0.1385, threshold_warn=0.1006) == expected


def test_classify_score_nan_is_fail():
    assert classify_score(float("nan"), threshold_use_ekf=0.1385, threshold_warn=0.1006) == "fail"


# --- NIS gate band decision (2026-09-03): divergence-not-quality redesign --
# thesis_notes.md "NIS gate band decision: divergence-not-quality redesign".
# Verifies the LIVE config thresholds (not literal values) against the exact
# health scores measured this session (diagnostics/inspect_v3_nis_gate_
# failure.py, live rate-corrected 100Hz window) for both real sessions, plus
# the fail/warn/pass gate paths on synthetic scores per the work order's own
# verification requirement.
# Scores re-anchored 2026-10-20 to the fit-L default (the thresholds were
# re-derived on it; diagnostics/inspect_closeout_l.py): the lowest
# development score (Le Castellet outing 25) and NBR JST outing 29.
# Supersedes Dubai 0.0549 / Paul Ricard outing 25 0.0473 (2026-10-11,
# per_session estimator).

LOWEST_DEV_HEALTH_SCORE = 0.1107   # Le Castellet outing 25
JST29_HEALTH_SCORE = 0.2089


def test_both_real_sessions_pass_under_live_thresholds():
    cfg = load_parameters()["nis_gate"]
    assert classify_score(LOWEST_DEV_HEALTH_SCORE, cfg["threshold_use_ekf"], cfg["threshold_warn"]) == "pass"
    assert classify_score(JST29_HEALTH_SCORE, cfg["threshold_use_ekf"], cfg["threshold_warn"]) == "pass"


def test_order_of_magnitude_collapse_still_fails_under_live_thresholds():
    cfg = load_parameters()["nis_gate"]
    # Threshold-relative by design (2026-10-11): half the live fail line, so a
    # re-derivation of the lines cannot break the premise again (the former
    # v3-score / 10 read 'warn' once the fail line moved to 0.0025). A
    # collapsed fit is the case FAIL is reserved for.
    collapsed_score = cfg["threshold_warn"] * 0.5
    assert classify_score(collapsed_score, cfg["threshold_use_ekf"], cfg["threshold_warn"]) == "fail"


def test_intermediate_score_warns_under_live_thresholds():
    cfg = load_parameters()["nis_gate"]
    # Between threshold_warn and threshold_use_ekf -- the buffer band.
    mid_score = (cfg["threshold_warn"] + cfg["threshold_use_ekf"]) / 2.0
    assert classify_score(mid_score, cfg["threshold_use_ekf"], cfg["threshold_warn"]) == "warn"


# --- NaN / short-session degradation ----------------------------------------

def test_health_score_window_longer_than_session_is_nan():
    # the score function itself (compute_health_score, its old public
    # wrapper, was removed in the 2026-10-22 pruning)
    nis = np.array([1.0, 2.0, 3.0])
    mask = np.array([True, True, True])
    score = _score_and_skips(nis, mask, window_samples=20, band_low=0.03, band_high=0.15)[0]
    assert score != score  # NaN


def test_health_score_empty_mask_is_nan():
    nis = np.random.default_rng(0).uniform(0, 3, size=100)
    mask = np.zeros(100, dtype=bool)
    score = _score_and_skips(nis, mask, window_samples=20, band_low=0.03, band_high=0.15)[0]
    assert score != score  # NaN


def test_evaluate_gate_short_session_degrades_to_fail(gate_inputs):
    short_nis = np.array([1.0, 2.0, 3.0])
    short_mask = np.array([True, True, True])
    result = evaluate_gate(short_nis, short_mask, gate_inputs["params"], gate_inputs["sample_rate_hz"])
    assert result["verdict"] == "fail"
    assert result["degenerate_reason"] is not None


def test_evaluate_gate_all_nan_nis_degrades_to_fail(gate_inputs):
    n = 200
    nis = np.full(n, np.nan)
    mask = np.ones(n, dtype=bool)
    result = evaluate_gate(nis, mask, gate_inputs["params"], gate_inputs["sample_rate_hz"])
    assert result["verdict"] == "fail"


def test_evaluate_gate_never_raises_on_degenerate_input(gate_inputs):
    # Explicit "never crash" check across a few adversarial shapes.
    params = gate_inputs["params"]
    sample_rate_hz = gate_inputs["sample_rate_hz"]
    for nis, mask in (
        (np.array([]), np.array([], dtype=bool)),
        (np.full(5, np.nan), np.ones(5, dtype=bool)),
        (np.zeros(50), np.zeros(50, dtype=bool)),
    ):
        result = evaluate_gate(nis, mask, params, sample_rate_hz)
        assert result["verdict"] == "fail"
