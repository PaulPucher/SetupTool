# Level-4 steering conversion (corrected 2026-10-04, schema 10): the wheel
# angle is read from the manufacturer map in config/car_data.json, mean of
# both front wheels (Rajamani sec. 2.2), instead of dividing by the table's
# steering_ratio column, which is the LOCAL ratio d(sw)/d(wheel).
# car_data.json is protected (local, gitignored) -> the whole module skips
# without it, which is also why the goldens stay at cap 1.

import numpy as np
import pytest

from modules.stability_analysis import load_car_data, prepare_vehicle_state
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle

pytestmark = pytest.mark.skipif(
    load_car_data() is None or "steering_ratio_table" not in (load_car_data() or {}),
    reason="config/car_data.json (protected, local) not present")


def _raw_table():
    t = load_car_data()["steering_ratio_table"]
    cols = t["columns"]
    sw = np.array([r[cols.index("steering_wheel_angle_deg")] for r in t["rows"]], float)
    wheel = np.array([r[cols.index("wheel_angle_deg")] for r in t["rows"]], float)
    ratio = np.array([r[cols.index("steering_ratio")] for r in t["rows"]], float)
    return sw, wheel, ratio


def _resolved_table(params):
    eff = apply_resolved_vehicle(params, resolve_accuracy(params, setup_data=None, cap=None))
    tab = eff["vehicle"]["steering_wheel_table"]
    return tab["sw_deg"], tab["wheel_deg"]


def test_resolves_to_level_4_map(raw_params):
    resolved = resolve_accuracy(raw_params, setup_data=None, cap=None)
    assert resolved["levels"]["steering_ratio"] == 4
    assert resolved["values"]["steering_ratio"]["mode"] == "table"


def test_two_wheel_mean_at_the_table_rows(raw_params):
    # expected value built independently from the raw rows: + branch wheel
    # angle and the mirrored - branch interpolated at the same |sw|
    sw, wheel, _ = _raw_table()
    # both branches include the centre row, or interp clamps below the
    # first non-zero row
    pos, neg = sw >= 0, sw <= 0
    neg_sw, neg_wheel = -sw[neg][::-1], -wheel[neg][::-1]
    t_sw, t_wheel = _resolved_table(raw_params)
    reach = min(sw[pos].max(), neg_sw.max())
    for s, w in zip(sw[pos], wheel[pos]):
        if s == 0 or s > reach:
            continue
        expected = 0.5 * (w + np.interp(s, neg_sw, neg_wheel))
        assert np.interp(s, t_sw, t_wheel) == pytest.approx(expected, abs=1e-9)
    # full lock: overall ratio ~13.7, nowhere near the local ratio 11.9
    assert np.interp(reach, t_sw, t_wheel) == pytest.approx(21.27, abs=0.01)


def test_map_is_odd_and_monotonic(raw_params):
    t_sw, t_wheel = _resolved_table(raw_params)
    assert np.all(np.diff(t_sw) > 0) and np.all(np.diff(t_wheel) > 0)
    probe = np.linspace(-280, 280, 57)
    assert np.allclose(np.interp(-probe, t_sw, t_wheel), -np.interp(probe, t_sw, t_wheel), atol=1e-12)


def test_not_the_local_ratio_division(raw_params):
    # the pre-fix conversion overstated the wheel angle at large lock
    sw, _, ratio = _raw_table()
    t_sw, t_wheel = _resolved_table(raw_params)
    s = 152.2  # largest steer in the US-v2 fit population
    old = s / np.interp(s, sw, ratio)
    new = np.interp(s, t_sw, t_wheel)
    assert old - new == pytest.approx(1.07, abs=0.02)


def test_prepare_vehicle_state_uses_the_map_on_dubai(raw_params, parsed_data):
    eff = apply_resolved_vehicle(raw_params, resolve_accuracy(raw_params, setup_data=None, cap=None))
    st = prepare_vehicle_state(parsed_data["channels"], eff)
    t_sw, t_wheel = _resolved_table(raw_params)
    expected = np.radians(np.interp(np.degrees(st["steer_sw_rad"]), t_sw, t_wheel))
    assert np.allclose(st["delta_f_rad"], expected, rtol=0, atol=1e-12)
