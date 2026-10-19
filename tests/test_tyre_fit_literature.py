# Fit L (tyre_fit_mode "literature_bounded", Q-FIT-LIT 2026-10-18) on real
# data, NBR JST outing 29: the method constants hold, the residuals are the
# load-scaled curve error in N, the tyre-frame transform inverts exactly, and
# the mode runs end to end with the load provenance in the manifest. The
# bit-identity of the production path is shown by
# diagnostics/check_fit_lit_inactive.py (three sessions, old vs new code).

import numpy as np
import pytest

from modules.csv_parser import parse_csv, load_channels_config
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.corner_analysis import reclassify_speed_classes
from modules.tyre_model_pacejka import pacejka_lateral_force
import modules.tyre_fit_auto as tfa
from tests.conftest import find_data_file

JST29 = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"


@pytest.fixture(scope="module")
def jst29():
    path = find_data_file(JST29)
    if path is None:
        pytest.skip("NBR JST outing 29 export not on this machine")
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    state = prepare_vehicle_state(data["channels"], params)
    base = tfa._base_mask(state, data["laps"])
    window, _ = tfa._speed_window_mask(state, data, params["tyre_fit_auto"])
    fit_mask = base & window
    forces = estimate_lateral_forces(state, params)
    slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    load_in = tfa._axle_load_inputs(state, data, params, forces, fit_mask)
    return {"params": params, "data": data, "state": state, "fit_mask": fit_mask, "forces": forces,
            "slip": slip, "load_in": load_in, "path": path}


def test_tyre_frame_transform_inverts(jst29):
    fy, li = jst29["forces"]["Fy_f_filt"], jst29["load_in"]
    tyre = tfa._front_tyre_frame_force(fy, li["fx_f_N"], li["delta_f"])
    back = tfa._front_body_force(tyre, li["fx_f_N"], li["delta_f"])
    np.testing.assert_allclose(back, fy, rtol=0, atol=1e-6)


def test_fit_l_constants_bounds_and_residuals(jst29):
    li, m = jst29["load_in"], jst29["fit_mask"]
    alpha, fz = jst29["slip"]["alpha_f_filt"], li["fz_f_N"]
    fy = tfa._front_tyre_frame_force(jst29["forces"]["Fy_f_filt"], li["fx_f_N"], li["delta_f"])
    fit = tfa._fit_axle_l(alpha, fy, fz, m, jst29["params"]["tyre_fit_auto"]["fit_density_bin_rad"])
    assert fit["C"] == tfa.L_SHAPE_C and fit["E"] == tfa.L_CURVATURE_E
    assert tfa.L_B_BOUNDS[0] <= fit["B"] <= tfa.L_B_BOUNDS[1]
    assert tfa.L_MU_BOUNDS[0] <= fit["mu"] <= tfa.L_MU_BOUNDS[1]
    pop = fit["residual_mask"]
    assert fit["fz_ref_N"] == pytest.approx(float(np.median(fz[pop])))
    assert fit["D"] == pytest.approx(fit["mu"] * fit["fz_ref_N"])
    # residual = measured force minus load-scaled curve, in N
    expect = fy[pop] - fit["mu"] * fz[pop] * pacejka_lateral_force(alpha[pop], fit["B"], fit["C"], 1.0, fit["E"])
    np.testing.assert_allclose(fit["residuals"], expect, rtol=1e-12, atol=1e-9)


@pytest.fixture(scope="module")
def l_manifest(jst29):
    return tfa.fit_session_pacejka(jst29["data"], jst29["params"], data_file_path=jst29["path"],
                                   tyre_fit_mode="literature_bounded")


def test_load_path_matches_constant_d_at_constant_load(jst29, l_manifest):
    # With Fx = 0 and Fz held at Fz_ref the load path differs from the
    # constant-D path only by cos(delta) on the front, so the observer must
    # predict lateral acceleration equally well. Catches a step evaluated
    # with D = mu as a force (the 2026-10-18 defect: 9.6 vs 2.0 m/s^2).
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
    st, man = jst29["state"], l_manifest
    cfg_mu = dict(man["final_config"])
    assert cfg_mu["d_is_mu"] is True
    fz_f, fz_r = man["axles"]["front"]["fz_ref_N"], man["axles"]["rear"]["fz_ref_N"]
    st_mu = dict(st, fz_axle_f_N=np.full(st["time"].size, fz_f), fz_axle_r_N=np.full(st["time"].size, fz_r),
                 fx_axle_f_N=np.zeros(st["time"].size))
    cfg_d = {k: v for k, v in cfg_mu.items() if k != "d_is_mu"}
    cfg_d["d_front"], cfg_d["d_rear"] = cfg_mu["d_front"] * fz_f, cfg_mu["d_rear"] * fz_r
    base = tfa._base_mask(st, jst29["data"]["laps"])
    out = {}
    for name, cfg, state in (("mu", cfg_mu, st_mu), ("d", cfg_d, st)):
        p = dict(jst29["params"])
        p["tyre_model_ekf_pacejka"] = {"_t": cfg}
        r = estimate_sideslip_ekf_pacejka(state, p, pass_id="_t")
        inn, sd = r["innovation"][base][:, 1], r["S_diag"][base][:, 1]
        out[name] = (float(np.sqrt(np.mean(inn ** 2))), float(np.mean(inn ** 2 / sd > tfa.CHI2_DF1_95)))
    print(f"ay innovation RMS load path {out['mu'][0]:.4f} / constant D {out['d'][0]:.4f} m/s^2; "
          f"ay exceedance {out['mu'][1]:.4f} / {out['d'][1]:.4f}")
    assert out["mu"][0] == pytest.approx(out["d"][0], rel=0.05)
    assert abs(out["mu"][1] - out["d"][1]) <= 0.01


def test_literature_bounded_runs_with_load_provenance(l_manifest):
    man = l_manifest
    assert man["tyre_fit_mode"] == "literature_bounded"
    assert set(man["axle_load"]["source_shares"]) == {"fl", "fr", "rl", "rr"}
    # census 2026-10-18: every corner of this export is damper-sourced
    for shares in man["axle_load"]["source_shares"].values():
        assert shares["moving_analysis_laps"]["damper"] == 1.0
    assert man["axle_load"]["lowest_level"] == 4
    for axle in ("front", "rear"):
        assert {"mu", "fz_ref_N", "bound_active"} <= set(man["axles"][axle])
    if "final_config" in man:
        assert man["final_config"]["d_is_mu"] is True
        assert man["final_config"]["d_front"] == man["axles"]["front"]["mu"]


def test_literature_gate_reports_slope_and_coverage(l_manifest):
    # D4 under L (reopen 2026-10-18): no peak-band gate; the normalised
    # origin slope is checked, the peak only reported and flagged
    for axle in ("front", "rear"):
        g, fit = l_manifest["plausibility_gate"][axle], l_manifest["axles"][axle]
        assert not any("g3" in f for f in g["failed"])
        assert g["stiffness_norm_per_rad"] == pytest.approx(fit["B"] * fit["C"] * fit["mu"])
        assert g["mu_extrapolated"] is (not fit["peak_in_visited_range"])
        assert fit["mu_extrapolated"] is g["mu_extrapolated"]
        in_band = tfa.L_K_BAND[0] <= g["stiffness_norm_per_rad"] <= tfa.L_K_BAND[1]
        assert ("g2 normalised stiffness" in g["failed"]) is (not in_band)


def test_literature_curve_is_monotonic(l_manifest):
    # C = 1.0 with E = -1 (2026-10-18): no peak and no falling branch -- the
    # slope stays positive out to 90 deg, the gate reports no peak and mu as
    # the extrapolated asymptotic grip
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    assert tfa.L_SHAPE_C == 1.0
    g = np.radians(np.linspace(0.01, 89.9, 4000))
    for axle in ("front", "rear"):
        fit, gate = l_manifest["axles"][axle], l_manifest["plausibility_gate"][axle]
        assert np.all(pacejka_lateral_stiffness(g, fit["B"], fit["C"], 1.0, fit["E"]) > 0)
        assert fit["peak_alpha_deg"] != fit["peak_alpha_deg"]
        assert gate["peak_note"] == "no peak (monotonic)" and gate["mu_extrapolated"] is True
