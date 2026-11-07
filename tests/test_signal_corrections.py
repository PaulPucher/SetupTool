# ORDER SPEED-1: unit tests of the three input corrections R1-R3
# (modules/signal_corrections.py). Synthetic signals by order of the work
# order: each test needs a known spike / force / offset to check against,
# which no real export provides.

import numpy as np
import pytest

from modules import signal_corrections as sc
from modules.csv_parser import UNIT_CONVERSIONS, _canonical_unit
from modules.stability_analysis import _estimate_sample_rate, load_parameters

R1_CFG = {"enabled": True, "spike_margin_mps": 1.0, "comparison_half_window_s": 0.1, "widen_s": 0.5}
EB_CFG = {"enabled": True, "drag_force_N": 1500.0, "throttle_closed_max_pct": 1.0}


def _ch(t, x, unit):
    return {"time": t, "data": np.asarray(x, dtype=float), "unit_raw": unit, "quality": "valid"}


def _speed_channels(abs_unit="kph"):
    # ecu_speed at 50 Hz, 100 km/h; one 60 ms spike to 130 km/h at t = 10 s.
    # ABS at 100 Hz: fronts 99 / 99.5 km/h, rears 100 km/h, so a replaced
    # sample (99.5) is told apart from an untouched one (100).
    t50 = np.arange(0, 20, 0.02)
    v = np.full(t50.size, 100.0)
    spike = (t50 >= 10.0) & (t50 < 10.06)
    v[spike] = 130.0
    t100 = np.arange(0, 20, 0.01)
    f = 1.0 / 1.609344 if abs_unit == "mph" else 1.0
    ch = {"ecu_speed": _ch(t50, v, "kph")}
    for name, val in (("abs_speed_fl", 99.0), ("abs_speed_fr", 99.5), ("abs_speed_rl", 100.0),
                      ("abs_speed_rr", 100.0)):
        ch[name] = _ch(t100, np.full(t100.size, val * f), abs_unit)
    return ch, t50, spike


def _r1(ch):
    return sc.correct_speed_reference(ch, R1_CFG, _canonical_unit, UNIT_CONVERSIONS, _estimate_sample_rate)


@pytest.mark.parametrize("abs_unit", ["kph", "mph"])
def test_r1_spike_replaced_clean_samples_untouched(abs_unit):
    ch, t, spike = _speed_channels(abs_unit)
    rec = _r1(ch)
    v = ch["ecu_speed"]["data"]
    assert rec["status"] == "applied" and rec["flagged_samples"] == int(spike.sum())
    # the spike and its flanks (widen 0.5 s) take the larger front ABS speed
    near = (t >= 10.0 - 0.5 - 1e-9) & (t < 10.06 + 0.5 - 1e-9)
    assert np.allclose(v[spike], 99.5)
    assert np.allclose(v[near], 99.5)
    # everything else keeps the exact logged value
    far = (t < 9.4) | (t > 10.7)
    assert np.array_equal(v[far], np.full(far.sum(), 100.0))


def test_r1_without_abs_channels_flags_and_leaves_speed():
    ch, _t, _spike = _speed_channels()
    before = ch["ecu_speed"]["data"].copy()
    del ch["abs_speed_rr"]
    with pytest.warns(UserWarning, match="speed reference"):
        rec = _r1(ch)
    assert rec["status"] == "no_abs_channels" and rec["applied"] is False
    assert np.array_equal(ch["ecu_speed"]["data"], before)


def test_r2_axle_sum_preserved_and_rear_capped():
    fx = np.array([-5000.0, -1000.0, -1500.0, -5000.0, 2000.0])
    thr = np.array([0.0, 0.0, 0.0, 80.0, 80.0])
    share_f = np.full(fx.size, 0.6)
    front, eb = sc.engine_braking_front_fx(fx, share_f, thr, EB_CFG)
    rear = fx - front
    # closed throttle, decelerating: 1500 N to the rear, remainder by load
    assert front[0] == pytest.approx(-3500.0 * 0.6) and rear[0] == pytest.approx(-1500.0 - 3500.0 * 0.4)
    # less deceleration than the engine drag: all on the rear, never more
    assert front[1] == 0.0 and rear[1] == pytest.approx(-1000.0)
    assert front[2] == 0.0 and rear[2] == pytest.approx(-1500.0)
    assert np.all(np.abs(rear[fx < 0]) <= np.abs(fx[fx < 0]) + 1e-9)
    assert np.allclose(front + rear, fx)
    assert list(eb) == [True, True, True, False, False]


def test_r2_nothing_changes_under_throttle():
    fx = np.array([-5000.0, -200.0, 3000.0])
    thr = np.array([30.0, 100.0, 100.0])
    share_f = np.array([0.55, 0.6, 0.5])
    front, eb = sc.engine_braking_front_fx(fx, share_f, thr, EB_CFG)
    assert not eb.any()
    assert np.array_equal(front, np.where(fx < 0, fx * share_f, 0.0))


R3_CFG = {"enabled": True, "standstill_speed_max_kmh": 1.0, "on_wheels_travel_mm": 5.0,
          "pit_speed_range_kmh": [5.0, 70.0], "min_standstill_s": 1.0,
          "steering_straight_yaw_max_deg_s": 0.5, "min_straight_s": 5.0}
YAW_OFF_DEG_S, AY_OFF_MPS2, SW_OFF_DEG = -0.3, -0.15, 3.0
AY_STRAIGHT_MPS2 = 0.3   # crossfall-like straight-line specific force, real, must stay


def _offset_session(with_standstill=True):
    # 100 Hz. 0-10 s pit lane at 60 km/h with the limiter on, 10-15 s standstill
    # on wheels, then three closed 100 s laps at 180 km/h from t = 20 s. Each lap:
    # 30 s straight running truly straight, 10 s straight with a gentle real bend
    # (2 deg/s, steering still near centre), 60 s of corners turning clockwise so
    # the lap closes at -360 deg. The steering carries slow noise (about 1 s
    # correlation, 0.3 deg) so the straight thresholds are finite. True a_y on
    # the straights is a crossfall-like 0.3 m/s^2 that R3 must leave in.
    from scipy.ndimage import gaussian_filter1d
    rng = np.random.default_rng(1)
    sr, lap_s, t0 = 100.0, 100.0, 20.0
    t = np.arange(0, t0 + 3 * lap_s, 1 / sr)
    on_lap = t >= t0
    in_lap = (t - t0) % lap_s
    straight = on_lap & (in_lap < 40.0)
    bend = on_lap & (in_lap >= 30.0) & (in_lap < 40.0)
    corner = on_lap & ~straight
    v = np.where(t < 10.0, 60.0, np.where(t < 15.0, 0.0, 180.0))
    slow = gaussian_filter1d(rng.normal(0, 1, t.size), 50)
    sw = np.where(corner, 30.0, 0.0) + 0.3 * slow / slow.std() + np.where(bend, 0.4, 0.0)
    yaw_true = np.where(bend, 2.0, 0.0) + np.where(corner, -(360.0 + 20.0) / 60.0, 0.0)
    ay_true = np.where(straight, AY_STRAIGHT_MPS2, 0.0) + np.where(corner, -15.0, 0.0)
    t50 = t[::2]
    ch = {"ecu_speed": _ch(t50, v[::2], "kph"),
          "sclu_yaw_rate": _ch(t, (yaw_true + YAW_OFF_DEG_S) / 6.0, "rpm"),
          "log_asteer": _ch(t, sw + SW_OFF_DEG, "deg"),
          "log_acc_y": _ch(t, (ay_true + AY_OFF_MPS2) / 9.81, "G"),
          "log_acc_x": _ch(t, np.zeros(t.size), "G"),
          "log_acc_z": _ch(t, np.full(t.size, 1.0), "G"),
          "ecu_aps": _ch(t, np.full(t.size, 100.0), "%"),
          "ecu_B_speedlimit_en": _ch(t, (t < 15.0).astype(float), "")}
    if with_standstill:
        for c in ("fl", "fr", "rl", "rr"):
            ch[f"log_susp_travel_{c}"] = _ch(t, np.where(on_lap, 12.0, 0.0), "mm")
    laps = [{"lap_number": i + 1, "start_time": t0 + i * lap_s, "end_time": t0 + (i + 1) * lap_s - 1 / sr,
             "is_valid_for_analysis": True} for i in range(3)]
    return ch, laps, straight & ~bend


def test_r3_standstill_offsets_applied():
    params = load_parameters()
    ch, laps, true_straight = _offset_session()
    rec = sc.remove_sensor_offsets(ch, laps, params, R3_CFG)
    c = rec["channels"]
    assert rec["status"] == "applied" and rec["standstill_s"] == pytest.approx(5.0, abs=0.05)
    assert c["sclu_yaw_rate"]["source"] == "standstill" and c["log_acc_y"]["source"] == "standstill"
    assert c["sclu_yaw_rate"]["offset_physical"] == pytest.approx(YAW_OFF_DEG_S, abs=1e-5)
    assert c["log_acc_y"]["offset_physical"] == pytest.approx(AY_OFF_MPS2, abs=1e-9)
    assert c["log_asteer"]["source"] == "straights_near_zero_yaw"
    assert c["log_asteer"]["offset_physical"] == pytest.approx(SW_OFF_DEG, abs=0.1)
    # the crossfall-like straight-line specific force stays in the signal
    assert np.median(ch["log_acc_y"]["data"][true_straight] * 9.81) == pytest.approx(AY_STRAIGHT_MPS2, abs=1e-6)


def test_r3_heading_excess_fallback_for_yaw_and_ay_flagged():
    # no travel channels -> no valid standstill: yaw rate from the closed laps,
    # a_y not corrected and flagged
    params = load_parameters()
    ch, laps, _ = _offset_session(with_standstill=False)
    ay_before = ch["log_acc_y"]["data"].copy()
    with pytest.warns(UserWarning, match="log_acc_y"):
        rec = sc.remove_sensor_offsets(ch, laps, params, R3_CFG)
    c = rec["channels"]
    assert c["sclu_yaw_rate"]["source"] == "heading_excess_per_lap" and c["sclu_yaw_rate"]["n"] == 3
    assert c["sclu_yaw_rate"]["offset_physical"] == pytest.approx(YAW_OFF_DEG_S, abs=0.01)
    assert c["log_acc_y"]["applied"] is False and rec["not_corrected"] == ["log_acc_y"]
    assert rec["status"] == "partial"
    assert np.array_equal(ch["log_acc_y"]["data"], ay_before)


def test_r3_steering_only_from_near_zero_yaw_straight_samples():
    # half of the straight samples run truly straight at centre 1.0 deg, the
    # other half in a real bend (yaw 1 deg/s) at 3.0 deg: only the first count
    sw = np.r_[np.full(600, 1.0), np.full(600, 3.0)]
    yaw = np.r_[np.zeros(600), np.full(600, 1.0)]
    mask = np.ones(1200, bool)
    off, n = sc.steering_offset(sw, yaw, mask, 0.5, 500)
    assert off == pytest.approx(1.0) and n == 600
    # too few qualifying samples -> no correction
    off, n = sc.steering_offset(sw, yaw, mask, 0.5, 700)
    assert off is None and n == 600


def test_r3_too_few_steering_samples_flags_and_leaves_steering():
    params = load_parameters()
    ch, laps, _ = _offset_session()
    sw_before = ch["log_asteer"]["data"].copy()
    with pytest.warns(UserWarning, match="log_asteer"):
        rec = sc.remove_sensor_offsets(ch, laps, params, dict(R3_CFG, min_straight_s=1000.0))
    assert rec["channels"]["log_asteer"]["applied"] is False and rec["status"] == "partial"
    assert np.array_equal(ch["log_asteer"]["data"], sw_before)


def test_switch_off_or_missing_block_means_uncorrected():
    assert sc.config({}, "speed_reference") is None
    assert sc.config({"signal_corrections": {"engine_braking": {"enabled": False}}}, "engine_braking") is None
    cfg = load_parameters()["signal_corrections"]
    assert all(cfg[k]["enabled"] for k in ("speed_reference", "engine_braking", "sensor_offset_removal"))
