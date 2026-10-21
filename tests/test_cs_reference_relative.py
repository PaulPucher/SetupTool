# R26 source fix (author ruling 2026-10-20) on real data: NBR qualifying CDR
# (outing 69), the development session with the most small front references
# before the fix. A band candidate sets the CS linear reference only within
# tyre_fit_auto.L_REF_RELATIVE x the session median of that axle's band
# candidates; a rejected candidate is never set, the previous reference holds,
# and the recorded R26 test (verdict-read samples under a reference below 0.5 x
# that median) finds nothing.

import numpy as np
import pytest

from modules.csv_parser import parse_csv, load_channels_config
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.corner_analysis import reclassify_speed_classes
import modules.tyre_fit_auto as tfa
from tests.conftest import find_data_file

QP_CDR = "NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt"


@pytest.fixture(scope="module")
def qp_cdr():
    path = find_data_file(QP_CDR)
    if path is None:
        pytest.skip("NBR qualifying CDR export not on this machine")
    raw = load_parameters()
    p = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    st = prepare_vehicle_state(data["channels"], p)
    beta, _man, _gate, fb, _fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
        st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    assert not fb
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    kw = cs_in["cs_kwargs"]
    assert kw["ref_relative"] == tfa.L_REF_RELATIVE == (0.5, 2.0)
    before = estimate_cornering_stiffness(slip, forces, st, p, data["laps"],
                                          **{k: v for k, v in kw.items() if k != "ref_relative"})
    after = estimate_cornering_stiffness(slip, forces, st, p, data["laps"], **kw)
    return {"before": before, "after": after, "band": kw["ref_band"]}


def _ffill(x):
    x = np.asarray(x, float)
    idx = np.where(np.isfinite(x), np.arange(x.size), 0)
    np.maximum.accumulate(idx, out=idx)
    y = x[idx]
    y[: np.argmax(np.isfinite(x))] = np.nan
    return y


def test_rejected_candidates_are_never_set(qp_cdr):
    lo_b, hi_b = qp_cdr["band"]
    lo_r, hi_r = tfa.L_REF_RELATIVE
    a = qp_cdr["after"]
    for ax in "fr":
        rc = np.asarray(a[f"ref_candidates_{ax}"], float)
        band = rc[(rc >= lo_b) & (rc <= hi_b)]
        med = float(np.median(band))
        assert med == pytest.approx(a[f"ref_relative_median_{ax}"])
        rejected = band[(band < lo_r * med) | (band > hi_r * med)]
        assert rejected.size == a[f"n_ref_relative_low_{ax}"] + a[f"n_ref_relative_high_{ax}"]
        refs = np.unique(a[f"C_linear_ref_{ax}"][np.isfinite(a[f"C_linear_ref_{ax}"])])
        # every reference in force is an accepted candidate, inside the window
        assert np.all((refs >= lo_r * med) & (refs <= hi_r * med))
        assert not np.isin(rejected, refs).any()
    # the front carried small references before the fix; they existed and are gone
    assert a["n_ref_relative_low_f"] > 0
    rc_b = np.unique(qp_cdr["before"]["C_linear_ref_f"][np.isfinite(qp_cdr["before"]["C_linear_ref_f"])])
    assert (rc_b < lo_r * a["ref_relative_median_f"]).any()


def test_previous_reference_holds_and_r26_count_is_zero(qp_cdr):
    lo_b, hi_b = qp_cdr["band"]
    a = qp_cdr["after"]
    for ax in "fr":
        ref = np.asarray(a[f"C_linear_ref_{ax}"], float)
        med = a[f"ref_relative_median_{ax}"]
        # the reference changes only to an accepted value: wherever it is in
        # force it lies in the window, so a rejection left the old one standing
        held = _ffill(ref)
        fin = np.isfinite(held)
        assert np.all(held[fin] >= 0.5 * med)
        # recorded R26 test: CS samples under a reference below 0.5 x median
        cs = np.asarray(a[f"CS_ratio_{ax}"], float)
        small = np.isfinite(cs) & np.isfinite(held) & (held < 0.5 * med)
        assert int(small.sum()) == 0
