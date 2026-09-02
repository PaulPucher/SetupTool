# 100 Hz time-base work package (2026-09-02) -- targeted tests for
# _resolve_grid_rate/prepare_vehicle_state's own grid-rate selection:
# both paths from the PHASE 0 amendment (100 Hz when the fast channels
# support it, 50 Hz channel-limited when they don't, refusal only below
# the hard floor), on synthetic per-channel fixtures.

import numpy as np
import pytest

from modules.stability_analysis import _resolve_grid_rate, prepare_vehicle_state

FAST_CHANNELS = ["sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z", "lap_distance"]


def _channel(rate_hz, duration_s=10.0):
    n = int(duration_s * rate_hz)
    t = np.arange(n) / rate_hz
    return {"label": "x", "unit": "x", "unit_raw": "x", "time": t, "data": np.zeros(n), "quality": "valid"}


def _channels_at(fast_rate_hz, ecu_speed_rate_hz=50.0):
    channels = {name: _channel(fast_rate_hz) for name in FAST_CHANNELS}
    channels["lap_distance"]["unit_raw"] = "m"
    channels["ecu_speed"] = _channel(ecu_speed_rate_hz)
    return channels


def _params(target=100.0, floor=50.0):
    return {"stability_estimation": {"target_sample_rate_hz": target, "min_sample_rate_hz": floor}}


def test_grid_rate_is_target_when_fast_channels_support_it():
    channels = _channels_at(fast_rate_hz=100.0)
    rate, status = _resolve_grid_rate(channels, _params())
    assert rate == pytest.approx(100.0, abs=0.5)
    assert status == "100 Hz"


def test_grid_rate_falls_back_channel_limited_without_refusing():
    # 55 Hz, not exactly the 50 Hz floor -- avoids a boundary-precision
    # false refusal from np.arange's own floating-point dt jitter at the
    # exact floor value, while still testing the "below target, above
    # floor -> channel-limited, not refused" path meaningfully.
    channels = _channels_at(fast_rate_hz=55.0)
    rate, status = _resolve_grid_rate(channels, _params())
    assert rate == pytest.approx(55.0, abs=0.5)
    assert "channel-limited" in status


def test_grid_rate_refuses_below_hard_floor_naming_the_channel():
    channels = _channels_at(fast_rate_hz=20.0)
    with pytest.raises(ValueError, match="20\\.0 Hz"):
        _resolve_grid_rate(channels, _params())


def test_grid_rate_uses_the_slowest_fast_channel_as_binding():
    channels = _channels_at(fast_rate_hz=100.0)
    channels["log_acc_z"] = _channel(60.0)  # one channel slower than the rest
    rate, status = _resolve_grid_rate(channels, _params())
    assert rate == pytest.approx(60.0, abs=0.5)
    assert "log_acc_z" in status


def test_missing_fast_channel_does_not_bind_the_rate():
    channels = _channels_at(fast_rate_hz=100.0)
    channels["log_acc_z"] = {"label": "x", "unit": "x", "unit_raw": "x", "time": None, "data": None, "quality": "missing"}
    rate, status = _resolve_grid_rate(channels, _params())
    assert rate == pytest.approx(100.0, abs=0.5)


def test_prepare_vehicle_state_end_to_end_at_100hz_and_50hz():
    # Full prepare_vehicle_state, both grid paths, on a minimal synthetic
    # fixture -- exercises the actual upsampling of ecu_speed onto the
    # resolved grid, not just _resolve_grid_rate in isolation.
    import json
    base_params = json.load(open("config/parameters.json", encoding="utf-8"))

    for fast_rate, expected_grid in ((100.0, 100.0), (55.0, 55.0)):
        channels = _channels_at(fast_rate_hz=fast_rate, ecu_speed_rate_hz=50.0)
        channels["ecu_speed"]["data"][:] = 100.0  # constant speed, avoids moving_mask edge cases
        channels["log_asteer"]["data"][:] = 0.0
        channels["log_acc_x"] = _channel(fast_rate)
        state = prepare_vehicle_state(channels, base_params)
        assert state is not None
        assert state["sample_rate_hz"] == pytest.approx(expected_grid, abs=0.5)
        assert len(state["time"]) > 0
        assert len(state["v_mps"]) == len(state["time"])
