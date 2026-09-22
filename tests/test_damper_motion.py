# Damper motion-state evidence, FRAME DEPTH PROGRAMME Step 2 (PLAN.md
# "FRAME DEPTH PROGRAMME", 2026-09-22). Targeted unit tests, synthetic
# fixtures -- same convention tests/test_wheel_loads.py and tests/
# test_decision_frame.py already use for pure-function/formula-correctness
# checks, not an ANALYSIS validation claim (CLAUDE.md's "real data only"
# rule governs the latter, not a minimal fixture proving classification
# logic). Real-session verification (sign convention, threshold
# derivation, Dubai RR dead-channel confirmation) is diagnostics/
# inspect_damper_motion_sign_and_threshold.py and thesis_notes.md, not
# duplicated here as a synthetic assertion.

import numpy as np
import pytest

from modules.decision_frame import TRANSIENT_PHASES
from modules.damper_motion import classify_window_motion, build_damper_motion_evidence


RATE_THRESHOLD = 1.0
MIN_VALID_FRACTION = 0.5


def _ramp_window(n=100, dt=0.01, slope_mm_s=20.0):
    t = np.arange(n) * dt
    travel = slope_mm_s * t
    return t, travel


# --- classify_window_motion: derivative sign and classification -----------
#
# Sign convention per the empirical check (thesis_notes.md "Damper motion
# sign-convention check", both real sessions agree): DECREASING travel =
# compression = "loading"; INCREASING travel = extension = "unloading".

def test_classify_window_motion_positive_rate_is_unloading():
    t, travel = _ramp_window(slope_mm_s=20.0)
    direction, rate, vf = classify_window_motion(travel, t, 0, len(t), None, RATE_THRESHOLD, MIN_VALID_FRACTION)
    assert direction == "unloading"
    assert rate == pytest.approx(20.0, abs=0.5)
    assert vf == pytest.approx(1.0)


def test_classify_window_motion_negative_rate_is_loading():
    t, travel = _ramp_window(slope_mm_s=-15.0)
    direction, rate, vf = classify_window_motion(travel, t, 0, len(t), None, RATE_THRESHOLD, MIN_VALID_FRACTION)
    assert direction == "loading"
    assert rate == pytest.approx(-15.0, abs=0.5)


def test_classify_window_motion_below_threshold_is_no_motion():
    t, travel = _ramp_window(slope_mm_s=0.1)  # well below RATE_THRESHOLD=1.0
    direction, rate, vf = classify_window_motion(travel, t, 0, len(t), None, RATE_THRESHOLD, MIN_VALID_FRACTION)
    assert direction == "no-motion"


# --- kerb-masking -----------------------------------------------------------

def test_classify_window_motion_kerb_masked_samples_excluded():
    # Median rate is robust to a MINORITY-share spike (by design -- a
    # single noisy sample at a window edge must not dominate), so to
    # actually exercise the exclusion path the spike must occupy a
    # MAJORITY of the window; a lower local valid-fraction floor (0.3,
    # not the module-level 0.5 used elsewhere in this file) is needed so
    # the masked call still has enough real samples left to classify
    # rather than returning None for being too sparse -- both numbers are
    # local to this one test, not a claim about the production floor.
    n = 100
    t, travel = _ramp_window(n=n, slope_mm_s=0.1)  # genuine motion: near-zero, below threshold
    travel = travel.copy()
    travel[10:65] += np.linspace(0, 200, 55)  # majority-share kerb-impact-shaped spike (55/100 samples)
    kerb_mask = np.zeros(n, dtype=bool)
    kerb_mask[10:65] = True
    local_floor = 0.3

    direction_with_mask, rate_with_mask, vf_with_mask = classify_window_motion(
        travel, t, 0, n, kerb_mask, RATE_THRESHOLD, local_floor)
    direction_without_mask, rate_without_mask, _ = classify_window_motion(
        travel, t, 0, n, None, RATE_THRESHOLD, local_floor)

    assert vf_with_mask == pytest.approx(0.45)  # 45/100 samples survive the mask
    assert direction_with_mask == "no-motion"  # kerb spike excluded, real near-zero motion recovered
    assert direction_without_mask == "unloading"  # unmasked, the majority-share spike dominates the median
    assert rate_with_mask != pytest.approx(rate_without_mask, abs=1.0)


# --- validity floor ----------------------------------------------------------

def test_classify_window_motion_sparse_window_returns_none():
    n = 100
    t, travel = _ramp_window(n=n, slope_mm_s=20.0)
    travel = travel.copy()
    travel[10:] = np.nan  # only the first 10/100 samples are real -- 10% valid, below the 50% floor
    direction, rate, vf = classify_window_motion(travel, t, 0, n, None, RATE_THRESHOLD, MIN_VALID_FRACTION)
    assert direction is None
    assert vf == pytest.approx(0.10)


def test_classify_window_motion_too_short_window_returns_none():
    t, travel = _ramp_window(n=100, slope_mm_s=20.0)
    direction, rate, vf = classify_window_motion(travel, t, 5, 6, None, RATE_THRESHOLD, MIN_VALID_FRACTION)
    assert direction is None
    assert vf == 0.0


# --- build_damper_motion_evidence: dead-channel guard and phase scope -----

def _channel(data, t, unit_raw="mm", quality="valid"):
    return {"data": data, "time": t, "unit_raw": unit_raw, "quality": quality}


def _synthetic_pipeline(n=1000, dt=0.01, frozen_wheel=None, rate_mm_s=20.0):
    t = np.arange(n) * dt
    channels = {}
    for wheel in ("fl", "fr", "rl", "rr"):
        if wheel == frozen_wheel:
            data = np.full(n, 5.0)  # perfectly flat, std=0 -- the mandatory dead-channel case
        else:
            # A slow drift so the whole-session std is well above the dead-channel floor.
            data = rate_mm_s * np.sin(t * 0.5)
        channels[f"log_susp_travel_{wheel}"] = _channel(data, t)
    state = {"time": t, "kerb_mask": np.zeros(n, dtype=bool)}
    # One lap instance per corner id, with a segment for every phase --
    # 1s-wide windows, non-overlapping, well inside the session.
    phase_starts = {"entry_1_brake": 1.0, "entry_2_turnin": 2.0, "apex_3": 3.0, "exit_4": 4.0, "exit_5": 5.0}
    segments = {p: (s, s + 0.5) for p, s in phase_starts.items()}
    corners = [{"stable_corner_id": 1, "segments": segments}]
    aggregated = {1: {"speed_class": "medium"}}
    return corners, state, channels, aggregated


def _dm_cfg():
    return {"rate_threshold_mm_s": RATE_THRESHOLD, "min_valid_fraction": MIN_VALID_FRACTION}


def _wl_cfg():
    from modules.stability_analysis import load_parameters
    return load_parameters()["wheel_loads"]


def test_build_damper_motion_evidence_dead_channel_not_evaluable():
    corners, state, channels, aggregated = _synthetic_pipeline(frozen_wheel="rr")
    evidence, summary = build_damper_motion_evidence(corners, state, channels, aggregated, _dm_cfg(), _wl_cfg())
    assert "rr" in summary["dead_wheels"]
    assert not any(e["wheel"] == "rr" for e in evidence)  # never emitted as "no-motion", simply absent
    assert any(e["wheel"] == "fl" for e in evidence)  # the other 3 wheels are unaffected


def test_build_damper_motion_evidence_transient_phases_only():
    corners, state, channels, aggregated = _synthetic_pipeline()
    evidence, summary = build_damper_motion_evidence(corners, state, channels, aggregated, _dm_cfg(), _wl_cfg())
    phases_seen = {e["phase"] for e in evidence}
    assert phases_seen <= set(TRANSIENT_PHASES)
    assert "apex_3" not in phases_seen
    assert phases_seen  # real motion was injected, so at least one transient phase fires
