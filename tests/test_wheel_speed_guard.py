# Fz-integration Phase 5 (2026-09-03): wheel-speed plausibility guard +
# ABS-domain fallback (modules/longitudinal_forces.py). Synthetic
# fixtures only -- hand-built arrays, not real telemetry (same category
# as tests/test_wheel_loads.py's own synthetic fixtures).
#
# MOTIVATION, real bug caught before shipping (thesis_notes.md "Fz-
# integration Phase 5..."): the FIRST version of the stuck-variance
# threshold (std_min_kmh=1.0) flagged 22-45% of samples on EVERY corner
# of BOTH real sessions, including sessions/corners with no known fault
# -- a real speed channel's own healthy 0.5s-window std routinely sits
# below 1.6 kph. A SECOND bug, also caught before shipping: a mate-only
# ratio-disagreement check cannot tell which of a disagreeing pair is at
# fault -- v3's own healthy log_speed_rl was flagged almost as often as
# its genuinely faulty mate log_speed_rr. Fixed by (1) lowering std_min_
# kmh to a real margin below observed healthy variance, (2) using
# ecu_speed as an axle-independent tie-breaker that flags only the
# corner whose own deviation from it exceeds its mate's.

import numpy as np

from modules.longitudinal_forces import (
    _normalize_wheel_speed_to_kmh, _rolling_plausibility_mask, _guarded_wheel_speed_kmh,
)

WINDOW = 10
STD_MIN = 0.1
RATIO_MAX_DEV = 0.10


def test_normalize_wheel_speed_identity_for_kph():
    data = np.array([100.0, 150.0, 200.0])
    assert np.array_equal(_normalize_wheel_speed_to_kmh(data, "kph"), data)
    assert np.array_equal(_normalize_wheel_speed_to_kmh(data, "km/h"), data)


def test_normalize_wheel_speed_scales_mph():
    data = np.array([100.0])
    np.testing.assert_allclose(_normalize_wheel_speed_to_kmh(data, "mph"), [160.9344])


def test_normalize_wheel_speed_raises_on_unknown_unit():
    import pytest
    with pytest.raises(ValueError):
        _normalize_wheel_speed_to_kmh(np.zeros(3), "m/s")


def _real_speed_like_signal(n, base):
    # A monotonic trend (no local plateau a smooth sine's own turning
    # points would create) plus alternating +/-0.3 jitter -- guarantees
    # every non-overlapping WINDOW-sized slice has std comfortably above
    # STD_MIN regardless of where the window boundary falls (a smooth
    # sine's own near-zero-derivative region, at exactly the window
    # boundaries WINDOW=10 happened to land on, is what the first version
    # of this fixture got caught by -- a fixture bug, not a production
    # one, kept here as the reason for this shape, not a smooth sine).
    trend = np.linspace(0.0, 2.0, n)
    jitter = 0.3 * np.array([(-1) ** i for i in range(n)], dtype=float)
    return base + trend + jitter


def _healthy_pair(n=40, base=150.0):
    # Corner, mate, and ecu_speed all track each other exactly -- must
    # never be flagged (zero mate disagreement, comfortably-above-floor
    # variance).
    signal = _real_speed_like_signal(n, base)
    moving = np.full(n, True)
    return signal.copy(), signal.copy(), signal.copy(), moving


def test_healthy_population_never_flagged():
    corner, mate, ecu, moving = _healthy_pair()
    valid = _rolling_plausibility_mask(corner, mate, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)
    assert valid.all()


def test_stuck_corner_flagged_directly():
    n = 40
    corner = np.full(n, 150.0)  # perfectly frozen
    mate = _real_speed_like_signal(n, 150.0)
    ecu = mate.copy()
    moving = np.full(n, True)
    valid = _rolling_plausibility_mask(corner, mate, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)
    assert not valid.any()


def test_stuck_check_ignored_at_genuine_standstill():
    n = 40
    corner = np.zeros(n)  # genuinely stationary, not stuck
    mate = np.zeros(n)
    ecu = np.zeros(n)
    moving = np.full(n, False)
    valid = _rolling_plausibility_mask(corner, mate, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)
    assert valid.all(), "no moving samples in any window -- must never flag on standstill data"


def test_mate_disagreement_attributed_to_the_corner_that_deviates_from_ecu():
    # rl/rr-style pair: mate tracks ecu_speed closely, corner (the "rr")
    # spikes far above both -- only the corner should be flagged, not the
    # mate, even though the mate-ratio check alone cannot tell them apart.
    n = 40
    ecu = _real_speed_like_signal(n, 150.0)
    mate = ecu.copy()
    corner = ecu * 1.30  # 30% high -- well past ratio_max_dev and clearly the outlier vs ecu
    moving = np.full(n, True)

    valid_corner = _rolling_plausibility_mask(corner, mate, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)
    valid_mate = _rolling_plausibility_mask(mate, corner, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)

    assert not valid_corner.any(), "the spiking corner must be flagged"
    assert valid_mate.all(), "the healthy mate must NOT be flagged just because its mate disagrees"


def test_mate_disagreement_with_no_ecu_evidence_flags_conservatively():
    # ecu_speed itself unusable (NaN) at the disagreement -- cannot
    # attribute, must flag rather than silently trust either side.
    n = 40
    base = _real_speed_like_signal(n, 150.0)
    corner = base * 1.30
    mate = base.copy()
    ecu = np.full(n, np.nan)
    moving = np.full(n, True)
    valid = _rolling_plausibility_mask(corner, mate, ecu, moving, WINDOW, STD_MIN, RATIO_MAX_DEV)
    assert not valid.any()


def _channel(time, data, quality="valid", unit_raw="kph"):
    return {"time": time, "data": data, "quality": quality, "unit_raw": unit_raw}


def test_guarded_wheel_speed_falls_back_to_abs_speed_when_guard_trips():
    n = 40
    t = np.arange(n, dtype=float)
    ecu = _real_speed_like_signal(n, 150.0)
    rl = ecu.copy()
    rr_raw = ecu * 1.30  # faulty, matches the attribution test above
    abs_rr = ecu.copy()  # the ABS-domain channel reads correctly

    channels = {
        "log_speed_rl": _channel(t, rl), "log_speed_rr": _channel(t, rr_raw),
        "ecu_speed": _channel(t, ecu), "abs_speed_rr": _channel(t, abs_rr, unit_raw="kph"),
    }
    params = {"wheel_speed_guard": {"window_s": WINDOW / 1.0, "std_min_kmh": STD_MIN,
                                     "ratio_max_deviation": RATIO_MAX_DEV}}
    moving = np.full(n, True)
    out_kmh, source = _guarded_wheel_speed_kmh(channels, "rr", t, moving, sample_rate_hz=1.0, params=params)

    assert (source == "abs_speed_fallback").all()
    np.testing.assert_allclose(out_kmh, abs_rr)


def test_guarded_wheel_speed_nan_when_no_fallback_channel():
    n = 40
    t = np.arange(n, dtype=float)
    ecu = _real_speed_like_signal(n, 150.0)
    rl = ecu.copy()
    rr_raw = ecu * 1.30

    channels = {
        "log_speed_rl": _channel(t, rl), "log_speed_rr": _channel(t, rr_raw),
        "ecu_speed": _channel(t, ecu),  # no abs_speed_rr at all
    }
    params = {"wheel_speed_guard": {"window_s": WINDOW / 1.0, "std_min_kmh": STD_MIN,
                                     "ratio_max_deviation": RATIO_MAX_DEV}}
    moving = np.full(n, True)
    out_kmh, source = _guarded_wheel_speed_kmh(channels, "rr", t, moving, sample_rate_hz=1.0, params=params)

    assert (source == "nan_no_fallback").all()
    assert np.isnan(out_kmh).all()


def test_guarded_wheel_speed_untouched_when_healthy():
    n = 40
    t = np.arange(n, dtype=float)
    ecu = _real_speed_like_signal(n, 150.0)
    rl = ecu.copy()
    rr = ecu.copy()

    channels = {
        "log_speed_rl": _channel(t, rl), "log_speed_rr": _channel(t, rr), "ecu_speed": _channel(t, ecu),
    }
    params = {"wheel_speed_guard": {"window_s": WINDOW / 1.0, "std_min_kmh": STD_MIN,
                                     "ratio_max_deviation": RATIO_MAX_DEV}}
    moving = np.full(n, True)
    out_kmh, source = _guarded_wheel_speed_kmh(channels, "rr", t, moving, sample_rate_hz=1.0, params=params)

    assert (source == "log_speed").all()
    np.testing.assert_allclose(out_kmh, rr)
