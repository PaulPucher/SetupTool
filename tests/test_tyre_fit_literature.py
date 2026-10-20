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


def test_fit_l_constants_mu_demo_and_residuals(jst29):
    # mu is the demonstrated utilisation (2026-10-19): the configured
    # percentile of |Fy / Fz| on the population the fit used; B alone fitted
    li, m = jst29["load_in"], jst29["fit_mask"]
    cfg = jst29["params"]["tyre_fit_auto"]
    alpha, fz = jst29["slip"]["alpha_f_filt"], li["fz_f_N"]
    fy = tfa._front_tyre_frame_force(jst29["forces"]["Fy_f_filt"], li["fx_f_N"], li["delta_f"])
    fit = tfa._fit_axle_l(alpha, fy, fz, m, cfg["fit_density_bin_rad"], cfg["mu_demo_percentile"])
    assert fit["C"] == tfa.L_SHAPE_C and fit["E"] == tfa.L_CURVATURE_E
    assert tfa.L_B_BOUNDS[0] <= fit["B"] <= tfa.L_B_BOUNDS[1]
    pop = fit["residual_mask"]
    assert fit["mu"] == float(np.percentile(np.abs(fy[pop] / fz[pop]), cfg["mu_demo_percentile"]))
    assert fit["bound_active"]["mu"] is None and "demonstrated utilisation" in fit["mu_source"]
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
        assert ("mu_demo above the physical ceiling 2.0" in g["failed"]) is (fit["mu"] > tfa.L_MU_CEILING)


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


def test_braking_filter_removes_only_heavy_braking(jst29, l_manifest):
    # braking side only (2026-10-19): a sample leaves the axle fit population
    # iff Fx < 0 and |Fx| / Fz > fit_long_util_max; drive above the threshold
    # (the driven rear) stays in
    li, m = jst29["load_in"], jst29["fit_mask"]
    cfg = jst29["params"]["tyre_fit_auto"]
    thr = cfg["fit_long_util_max"]
    for axle, fx_k, fz_k, a_k, fy in (("front", "fx_f_N", "fz_f_N", "alpha_f_filt",
                                       tfa._front_tyre_frame_force(jst29["forces"]["Fy_f_filt"], li["fx_f_N"], li["delta_f"])),
                                      ("rear", "fx_r_N", "fz_r_N", "alpha_r_filt", jst29["forces"]["Fy_r_filt"])):
        util = np.abs(li[fx_k]) / li[fz_k]
        heavy_braking = (li[fx_k] < 0) & (util > thr)
        heavy_drive = (li[fx_k] > 0) & (util > thr)
        assert l_manifest["axle_load"]["braking_excluded"][axle]["n"] == int((m & heavy_braking).sum())
        fit = tfa._fit_axle_l(jst29["slip"][a_k], fy, li[fz_k], m & ~heavy_braking, cfg["fit_density_bin_rad"],
                              cfg["mu_demo_percentile"])
        pop = fit["residual_mask"]
        finite = np.isfinite(jst29["slip"][a_k]) & np.isfinite(fy) & np.isfinite(li[fz_k]) & (li[fz_k] > 0)
        assert not (pop & heavy_braking).any()
        assert np.array_equal(pop & heavy_drive, m & heavy_drive & finite)
        assert np.array_equal(pop, m & ~heavy_braking & finite)
        assert fit["mu"] == pytest.approx(l_manifest["axles"][axle]["mu"])
    # the driven rear has drive samples above the threshold, and they are kept
    assert ((li["fx_r_N"] > 0) & (np.abs(li["fx_r_N"]) / li["fz_r_N"] > thr) & m).any()


def test_reset_mark_spans_exactly_the_windows_with_a_reset(jst29, l_manifest):
    # observer_reset_affected marks a CS sample iff its regression window
    # [start, i) holds an observer reset; without a reset mask the CS output
    # gains no key and is unchanged
    from modules.stability_analysis import (estimate_cornering_stiffness, reconstruct_cs_window_start,
                                            resolve_cs_min_window_samples)
    # The shipped run has no hard-bound re-entry on this session since Q-UPPER
    # (2026-10-19), so the marking is checked with the real reset sequence of
    # the same observer under the NIS-window trigger; the windowing does not
    # depend on where the mask comes from.
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
    st, p, laps = jst29["state"], jst29["params"], jst29["data"]["laps"]
    li = jst29["load_in"]
    st_l = dict(st, fz_axle_f_N=li["fz_f_N"], fz_axle_r_N=li["fz_r_N"], fx_axle_f_N=li["fx_f_N"])
    p_nw = dict(p)
    p_nw["tyre_model_ekf_pacejka"] = {"_t": dict(l_manifest["final_config"], divergence_trigger="nis_window")}
    reset = estimate_sideslip_ekf_pacejka(st_l, p_nw, pass_id="_t")["diverged_mask"]
    assert reset.any()
    slip = estimate_slip_angles(st, l_manifest["beta_ekf_with_fallback"], p)
    plain = estimate_cornering_stiffness(slip, jst29["forces"], st, p, laps)
    marked = estimate_cornering_stiffness(slip, jst29["forces"], st, p, laps, reset_mask=reset)
    assert "observer_reset_affected_f" not in plain
    for k, v in plain.items():
        assert np.array_equal(v, marked[k], equal_nan=True) if isinstance(v, np.ndarray) else v == marked[k]
    se = p["stability_estimation"]
    mw = resolve_cs_min_window_samples(p, st["sample_rate_hz"])
    cum = np.concatenate(([0], np.cumsum(reset)))
    for ax, key in (("f", "alpha_f_filt"), ("r", "alpha_r_filt")):
        idx = np.flatnonzero(np.isfinite(marked[f"CS_ratio_{ax}"]))
        mark = marked[f"observer_reset_affected_{ax}"]
        assert mark[idx].any() and not mark[idx].all()
        for i in idx[::max(1, idx.size // 3000)]:
            s0 = reconstruct_cs_window_start(slip[key], i, mw, se["cs_min_slip_angle_span_rad"], st.get("s_m"),
                                             se["cs_max_window_m"])
            assert bool(mark[i]) == bool(cum[i] > cum[s0])


def _base_runs(mask):
    # (start, stop) of contiguous True runs
    d = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def test_change_detector_step_on_real_nis_segments(jst29, l_manifest):
    # Rajamani sec. 14.4.2 change detector on the shipped L run's NIS (JST 29,
    # real data only). Reference: the closed form of the recursion started at
    # zero, a_t = S_t - min(0, S_1 .. S_t) with S the cumulative sum of
    # (NIS - d) -- independent of the step code. A quiet 2 s stretch (closed
    # form below h / 2 throughout) gives no alarm; the burst with the largest
    # closed-form value alarms at exactly its first crossing of h.
    from modules.sideslip_ekf_pacejka import change_detector_step
    cfg = jst29["params"]["tyre_fit_auto"]
    d, h = cfg["l_cd_drift"], cfg["l_cd_threshold_h"]
    nis, base = l_manifest["nis_full"], l_manifest["base_mask"] & np.isfinite(l_manifest["nis_full"])
    sr = jst29["state"]["sample_rate_hz"]

    def closed_form(seg):
        s = np.cumsum(seg - d)
        return s - np.minimum(0.0, np.minimum.accumulate(s))

    def stepped(seg):
        a, out, alarms = 0.0, [], []
        for x in seg:
            a, al = change_detector_step(a, float(x), d, h)
            out.append(a)
            alarms.append(al)
        return np.array(out), np.array(alarms)

    quiet = burst = None
    for a0, a1 in _base_runs(base):
        seg = nis[a0:a1]
        ref = closed_form(seg)
        n2 = int(2 * sr)
        if quiet is None and seg.size >= n2 and np.all(ref[:n2] < h / 2):
            quiet = seg[:n2]
        if burst is None or ref.max() > closed_form(burst).max():
            burst = seg
    assert quiet is not None and burst is not None
    a_q, al_q = stepped(quiet)
    np.testing.assert_allclose(a_q, closed_form(quiet), rtol=0, atol=1e-6)
    assert not al_q.any()
    ref_b = closed_form(burst)
    assert ref_b.max() > h
    a_b, al_b = stepped(burst)
    k = int(np.argmax(ref_b > h))
    np.testing.assert_allclose(a_b[:k + 1], ref_b[:k + 1], rtol=0, atol=1e-6)
    assert int(np.argmax(al_b)) == k
    print(f"quiet stretch max a {a_q.max():.2f}; burst first alarm at sample {k} (a {a_b[k]:.1f} > h {h:g})")


def test_observer_reset_mask_is_hard_bound_only(jst29, l_manifest):
    # mask scope ruling 2026-10-19: only hard-bound re-entries mark CS
    # windows; a hard-bound reset is exactly a raw update beyond the bound
    reset = l_manifest["observer_reset_mask"]
    bound = np.radians(jst29["params"]["tyre_fit_auto"]["beta_hard_bound_deg"])
    # since Q-UPPER this session has no hard-bound re-entry; the equality
    # below holds either way
    np.testing.assert_array_equal(reset, np.abs(l_manifest["beta_ekf"]) > bound)
    base = l_manifest["base_mask"]
    print(f"NIS on the base mask, shipped L run: mean {np.nanmean(l_manifest['nis_full'][base]):.3f}, "
          f"median {np.nanmedian(l_manifest['nis_full'][base]):.3f}")


def test_change_detector_trigger_resets_once_per_alarm(jst29, l_manifest):
    # Real data, final L config with the change detector switched on: every
    # reset that is not a hard-bound one is a detector alarm, the statistic
    # restarts after it, so two alarms are at least ceil(h / max NIS step)
    # apart; a placeholder h is enough to check the rule. With the default
    # trigger the final config reproduces the shipped run exactly.
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka, change_detector_step
    st, man = jst29["state"], l_manifest
    cfg = dict(man["final_config"])
    assert cfg["divergence_trigger"] == "hard_bound_only"
    li = jst29["load_in"]
    st_l = dict(st, fz_axle_f_N=li["fz_f_N"], fz_axle_r_N=li["fz_r_N"], fx_axle_f_N=li["fx_f_N"])
    p = dict(jst29["params"])
    p["tyre_model_ekf_pacejka"] = {"_t": cfg}
    r0 = estimate_sideslip_ekf_pacejka(st_l, p, pass_id="_t")
    np.testing.assert_array_equal(r0["beta_with_fallback"], man["beta_ekf_with_fallback"])
    cfg_cd = dict(cfg, divergence_trigger="change_detector", cd_drift=2.0, cd_threshold=50.0)
    p["tyre_model_ekf_pacejka"] = {"_t": cfg_cd}
    r = estimate_sideslip_ekf_pacejka(st_l, p, pass_id="_t")
    assert not (r["hard_bound_mask"] & ~r["diverged_mask"]).any()
    alarms = r["diverged_mask"] & ~r["hard_bound_mask"]
    assert alarms.any()
    # replay the statistic from the logged NIS with the loop's reset rule
    a, expect = 0.0, np.zeros_like(alarms)
    moving = st["moving_mask"]
    for i in range(alarms.size):
        if not moving[i]:
            a = 0.0
            continue
        a, al = change_detector_step(a, r["nis"][i], 2.0, 50.0)
        expect[i] = al and not r["hard_bound_mask"][i]
        if al or r["hard_bound_mask"][i]:
            a = 0.0
    np.testing.assert_array_equal(alarms, expect)
    print(f"change detector at placeholder h 50: {int(alarms.sum())} alarms, "
          f"{int(r['hard_bound_mask'].sum())} hard-bound resets")


def test_observer_q_reproduces_q_phys(l_manifest):
    # Q-UPPER in the chain (2026-10-19) is the held-over-step value of
    # diagnostics/inspect_q_phys.py on the same session
    import json
    import os
    f = os.path.join("diagnostics", "plots", "q_phys", "q_phys_nbr_o29_jst.json")
    if not os.path.exists(f):
        pytest.skip("inspect_q_phys.py output for NBR outing 29 not on this machine")
    with open(f, encoding="utf-8") as fh:
        ref = json.load(fh)
    q = l_manifest["observer_q"]
    assert q["q_beta_var"] == pytest.approx(ref["e_beta_dot"]["q_held_step"], rel=1e-9)
    assert q["q_yaw_rate_var"] == pytest.approx(ref["e_r_dot"]["q_held_step"], rel=1e-9)
    assert q["n_joint"] == ref["n_joint"]
    assert l_manifest["final_config"]["Q_beta_var"] == q["q_beta_var"]
    assert l_manifest["final_config"]["Q_yaw_rate_var"] == q["q_yaw_rate_var"]


def test_hard_bound_only_makes_no_soft_reset(jst29, l_manifest):
    # fit-L default trigger: every reset of the shipped final run is a
    # hard-bound one
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
    cfg = dict(l_manifest["final_config"])
    assert cfg["divergence_trigger"] == "hard_bound_only"
    li = jst29["load_in"]
    st_l = dict(jst29["state"], fz_axle_f_N=li["fz_f_N"], fz_axle_r_N=li["fz_r_N"], fx_axle_f_N=li["fx_f_N"])
    p = dict(jst29["params"])
    p["tyre_model_ekf_pacejka"] = {"_t": cfg}
    r = estimate_sideslip_ekf_pacejka(st_l, p, pass_id="_t")
    np.testing.assert_array_equal(r["beta_with_fallback"], l_manifest["beta_ekf_with_fallback"])
    np.testing.assert_array_equal(r["diverged_mask"], r["hard_bound_mask"])
    print(f"hard-bound resets {int(r['hard_bound_mask'].sum())}, soft resets 0")


def test_noise_floors_are_taken_on_the_consumed_signals(jst29, l_manifest):
    # floors (2026-10-19): variance above the 2 Hz zero-phase low-pass on the
    # steering-defined straights, of the very arrays the measurement update
    # reads (state ay_mps2 and yaw_rate_radps)
    from modules.stability_analysis import _butterworth_lowpass
    from modules.straights import steering_straights
    st, p = jst29["state"], jst29["params"]
    m = steering_straights(st, jst29["data"]["laps"], p["tyre_fit_auto"])["mask"]
    fl = l_manifest["noise_floors"]
    for name, key in (("ay", "ay_mps2"), ("yaw_rate", "yaw_rate_radps")):
        x = st[key]
        hp = x - _butterworth_lowpass(x, p["stability_estimation"]["cs_filter_cutoff_hz"], st["sample_rate_hz"])
        assert fl[name]["var"] == pytest.approx(float(np.var(hp[m & np.isfinite(hp)])), rel=1e-12)
        assert fl[name]["n_samples"] == int((m & np.isfinite(hp)).sum()) > 0


def test_ay_floor_is_a_candidate_and_wins_when_the_sweep_prefers_lower(jst29, l_manifest):
    # the floor value is on the R_ay grid; no pick below it; if the sweep
    # without the floor goes lower and the floor point itself is in band and
    # passes the guard, the pick is the floor, not a grid point above it
    fl, sw = l_manifest["noise_floors"]["ay"], l_manifest["r_sweep"]
    f = fl["r_ay_scale"]
    pts = [g for g in sw["grid_points"] if g["r_ay_scale"] == f]
    assert pts, "floor value missing from the R_ay grid"
    assert min(g["r_ay_scale"] for g in sw["grid_points"]) < f
    assert sw["chosen"]["r_ay_scale"] >= f
    guard = jst29["params"]["tyre_fit_auto"]["sign_check_marginal_fraction"]
    floor_ok = [g for g in pts if g["ay_in_band"] and g["sign_fraction"] >= guard]
    if fl["floor_binding"] and floor_ok:
        assert sw["chosen"]["r_ay_scale"] == f and fl["pick_location"] == "floor"
    print(f"ay floor r_ay_scale {f:.4g}; binding {fl['floor_binding']}; pick {sw['chosen']['r_ay_scale']:.4g} "
          f"({fl['pick_location']})")


def test_r_yaw_is_the_measured_yaw_rate_floor(l_manifest):
    # fit L: R_yaw is not swept; every grid point and the final run use the
    # session's yaw-rate noise variance
    v = l_manifest["noise_floors"]["yaw_rate"]["var"]
    assert v > 0
    for g in l_manifest["r_sweep"]["grid_points"]:
        assert g["R_yaw_rate_var"] == pytest.approx(v, rel=1e-12)
    assert l_manifest["final_config"]["R_yaw_rate_var"] == pytest.approx(v, rel=1e-12)


def test_module_straights_match_the_diagnostic_definition(jst29):
    # modules/straights.py is the definition of
    # diagnostics/inspect_kinematic_inputs.steering_straights, moved
    from modules.straights import steering_straights
    from diagnostics.inspect_kinematic_inputs import steering_straights as diag
    st, laps = jst29["state"], jst29["data"]["laps"]
    valid = [l for l in laps if l.get("is_valid_for_analysis")]
    lap_id = np.full(st["time"].size, -1)
    for l in valid:
        lap_id[(st["time"] >= l["start_time"]) & (st["time"] <= l["end_time"])] = l["lap_number"]
    ref = diag(st, valid, lap_id, st["kerb_mask"] | st["kerb_smear_zone"])
    got = steering_straights(st, laps, jst29["params"]["tyre_fit_auto"])
    assert [tuple(map(int, s)) for s in got["segs"]] == [tuple(map(int, s)) for s in ref["segs"]]
    assert got["d_min"] == ref["d_min"]
