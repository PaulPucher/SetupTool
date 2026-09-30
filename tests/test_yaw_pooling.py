# (c)-display: core/yaw_pooling and the trace windows' yaw panels. The
# pooling is checked on synthetic laps with a known envelope and planted
# flags; the panel path on real Dubai data must leave every analysis array
# untouched and must disappear with the config flag off.

import copy
import os

import numpy as np
import pytest

from core.yaw_pooling import build_panel_data, pool_yaw_rate, robust_std, session_masks

SR = 100.0          # Hz
V = 50.0            # m/s -> 0.5 m per sample
LAP_M = 1000.0
BRACKET = (300.0, 500.0)
OFFSETS = np.linspace(-2.0, 2.0, 9)   # per-lap yaw offset, deg/s
CFG = dict(fc_hz=4.0, trend_hz=0.5, grid_m=1.0, gap_m=2.0, envelope_percentiles=(10, 90), min_laps=3,
           margin_sigma=3.0, min_run_pts=5)


def _base(s, amp=20.0):
    # smooth one-direction corner: 0 at the bracket edges, amp mid-corner
    x = np.clip((s - BRACKET[0]) / (BRACKET[1] - BRACKET[0]), 0.0, 1.0)
    return amp * np.sin(np.pi * x)


def _bump(s, lo, hi, amp):
    x = np.clip((s - lo) / (hi - lo), 0.0, 1.0)
    return amp * np.sin(np.pi * x) ** 2


def _session(noise=1.0, offsets=OFFSETS, bumps=None, seed=0, amp=20.0, noise_gap_m=100.0):
    """Nine laps of one corner. bumps: {lap_index: (s_lo, s_hi, amp)}.
    Noise only on straights, >= noise_gap_m away from the bracket, so the
    corner rows are exactly base + offset (+ bump) up to the filter's
    smoothing. noise 1.0 puts the margin (~1 deg/s) above the 0.4 deg/s by
    which the +/-2 extreme laps sit beyond p10/p90 -- smaller noise makes
    the method correctly flag them."""
    rng = np.random.default_rng(seed)
    n = int(LAP_M / (V / SR))
    s_lap = np.arange(n) * (V / SR)
    t, s, yaw, laps, corners = [], [], [], [], []
    for k, off in enumerate(offsets):
        y = _base(s_lap, amp) + off
        if bumps and k in bumps:
            y = y + _bump(s_lap, *bumps[k])
        far = (s_lap < BRACKET[0] - noise_gap_m) | (s_lap > BRACKET[1] + noise_gap_m)
        y = y + np.where(far, noise * rng.standard_normal(n), 0.0)
        tk = (k * n + np.arange(n)) / SR
        t.append(tk); s.append(s_lap); yaw.append(y)
        laps.append({"lap_number": k + 1, "start_time": tk[0], "end_time": tk[-1], "is_valid_for_analysis": True})
        corners.append({"lap_number": k + 1, "stable_corner_id": 1,
                        "bracket_start_m": BRACKET[0], "bracket_end_m": BRACKET[1]})
    t, s, yaw = np.concatenate(t), np.concatenate(s), np.concatenate(yaw)
    return t, s, yaw, laps, corners


def _pool(t, s, yaw, laps, corners, **over):
    m = session_masks(t, s, np.ones(t.size, bool), None, laps, corners)
    return pool_yaw_rate(t, s, yaw, SR, m["racing"], m["straight"], None, m["traversals"], **{**CFG, **over}), m


def test_known_median_envelope_and_no_flags_without_outliers():
    t, s, yaw, laps, corners = _session()
    out, _ = _pool(t, s, yaw, laps, corners)
    c = out["corners"][1]
    assert c["laps"] == list(range(1, 10))
    g = c["grid"]
    inner = (g > BRACKET[0] + 20) & (g < BRACKET[1] - 20)
    # median offset 0, numpy-linear p10/p90 of nine evenly spaced offsets
    # = -1.6 / +1.6; tolerance covers the 4 Hz filter's smoothing of the base
    assert np.allclose(c["med"][inner], _base(g[inner]), atol=0.05)
    assert np.allclose(c["p_lo"][inner], _base(g[inner]) - 1.6, atol=0.05)
    assert np.allclose(c["p_hi"][inner], _base(g[inner]) + 1.6, atol=0.05)
    assert c["flags"] == []


def test_planted_excess_and_deficit_are_flagged_on_their_laps_only():
    # lap 5 (offset 0) gains +10 deg/s at 380-420 m, lap 7 loses 10 at
    # 440-480 m; the turn is positive there, so +10 is EXCESS, -10 DEFICIT
    t, s, yaw, laps, corners = _session(bumps={4: (380.0, 420.0, 10.0), 6: (440.0, 480.0, -10.0)})
    out, _ = _pool(t, s, yaw, laps, corners)
    flags = out["corners"][1]["flags"]
    assert [(f["lap"], f["kind"]) for f in flags] == [(5, "EXCESS"), (7, "DEFICIT")]
    exc, dfc = flags
    assert 380.0 <= exc["s_on"] < 410.0 and exc["i1"] - exc["i0"] >= CFG["min_run_pts"]
    assert 440.0 <= dfc["s_on"] < 470.0
    assert exc["peak"] > out["margin"] and dfc["peak"] > out["margin"]


def test_short_excursion_below_min_run_is_not_flagged():
    # 4 m wide bump: beyond the margin for fewer than 5 grid points
    t, s, yaw, laps, corners = _session(bumps={4: (398.0, 402.0, 10.0)})
    out, _ = _pool(t, s, yaw, laps, corners)
    assert out["corners"][1]["flags"] == []


def test_margin_is_sigma_times_robust_noise_of_filtered_yaw_on_straights():
    from scipy.signal import butter, filtfilt
    from modules.stability_analysis import BUTTERWORTH_ORDER
    # flat session, noise on every straight sample: the margin is then linear
    # in the noise (filters linear, MAD homogeneous), so a doubled noise with
    # the same seed must double it exactly
    t, s, yaw, laps, corners = _session(noise=0.4, offsets=np.zeros(9), amp=0.0, noise_gap_m=0.0)
    out, masks = _pool(t, s, yaw, laps, corners)
    # independent recomputation of the documented rule
    lp = lambda x, fc: filtfilt(*butter(BUTTERWORTH_ORDER, fc / (0.5 * SR), btype="low"), x)
    dev = (lp(yaw, 4.0) - lp(yaw, 0.5))[masks["straight"]]
    sigma = 1.4826 * np.median(np.abs(dev - np.median(dev)))
    assert np.isclose(out["sigma"], sigma) and np.isclose(out["margin"], 3.0 * sigma)
    # straights exclude every bracket sample
    s_st = s[masks["straight"]]
    assert not np.any((s_st >= BRACKET[0]) & (s_st <= BRACKET[1]))
    # margin scales with the noise and with margin_sigma
    t2, s2, yaw2, *_ = _session(noise=0.8, offsets=np.zeros(9), amp=0.0, noise_gap_m=0.0)
    out2, _ = _pool(t2, s2, yaw2, laps, corners)
    assert np.isclose(out2["margin"] / out["margin"], 2.0, rtol=1e-9)
    out3, _ = _pool(t, s, yaw, laps, corners, margin_sigma=2.0)
    assert np.isclose(out3["margin"], 2.0 * sigma)


def test_corner_with_too_few_laps_carries_no_envelope():
    t, s, yaw, laps, corners = _session()
    for lap in laps[2:]:
        lap["is_valid_for_analysis"] = False
    out, _ = _pool(t, s, yaw, laps, corners)
    assert out["corners"] == {}


def _cfg(raw_params):
    return raw_params["corner_trace_display"]["yaw_pooling"]


def test_panel_data_leaves_inputs_untouched_and_matches_diagnostic_margin(state, parsed_data, raw_params):
    before = {k: np.asarray(v).tobytes() for k, v in state.items() if isinstance(v, np.ndarray)}
    corners_before = copy.deepcopy(parsed_data["corners"])
    pooled = build_panel_data(state, parsed_data["laps"], parsed_data["corners"], _cfg(raw_params))
    assert {k: np.asarray(state[k]).tobytes() for k in before} == before
    assert parsed_data["corners"] == corners_before
    # Dubai margin recorded by the diagnostic ("(c) Part 1"): 1.52 deg/s
    assert abs(pooled["margin"] - 1.52) < 0.01
    assert pooled["n_analysis_laps"] == sum(1 for l in parsed_data["laps"] if l.get("is_valid_for_analysis"))
    assert pooled["steer_rate"] is not None and pooled["steer_rate"].shape == state["time"].shape


# --- dialog path (offscreen Qt) -------------------------------------------

@pytest.fixture
def qt_app():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    QtWidgets = pytest.importorskip("PyQt6.QtWidgets")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _with_flag(monkeypatch, raw_params, flag):
    # patched copy only: never writes config/parameters.json
    import modules.stability_analysis as sa
    p = copy.deepcopy(raw_params)
    p["corner_trace_display"]["yaw_pooling_panel"] = flag
    monkeypatch.setattr(sa, "load_parameters", lambda *a, **k: copy.deepcopy(p))


def _result(pipeline_result, state):
    return {"state": state, "cs": pipeline_result["cs"], "stab": pipeline_result["stab"],
            "corners": pipeline_result["corners"], "summaries": pipeline_result["summaries"],
            "slip": pipeline_result["slip"], "forces": pipeline_result["forces"], "ls": None,
            "lap_filter": None, "sideslip_source": None, "fit_manifest": None}


def _analysis_bytes(result):
    out = {f"state.{k}": v.tobytes() for k, v in result["state"].items() if isinstance(v, np.ndarray)}
    for group in ("cs", "stab", "slip", "forces"):
        out.update({f"{group}.{k}": v.tobytes() for k, v in result[group].items() if isinstance(v, np.ndarray)})
    return out


def test_panels_render_and_analysis_arrays_untouched(qt_app, monkeypatch, raw_params, pipeline_result,
                                                     parsed_data, state):
    from ui.views.corner_trace_dialog import CornerTraceDialog, LapTraceDialog
    _with_flag(monkeypatch, raw_params, True)
    result = _result(pipeline_result, state)
    before = _analysis_bytes(result)
    summaries = copy.deepcopy(result["summaries"])

    corner_dlg = CornerTraceDialog()
    assert "yaw" in corner_dlg.plots and "steer" in corner_dlg.plots
    pooled_ids = set(corner_dlg._yaw_panel_data(result, parsed_data)["corners"])
    summary = next(s for s in result["summaries"] if s["stable_corner_id"] in pooled_ids)
    corner_dlg.show_corner(summary, result, parsed_data)
    assert len(corner_dlg.plots["yaw"].listDataItems()) > 0
    assert len(corner_dlg.plots["steer"].listDataItems()) > 0

    lap_dlg = LapTraceDialog()
    lap = next(l["lap_number"] for l in parsed_data["laps"] if l.get("is_valid_for_analysis"))
    lap_dlg.show_lap(lap, result, parsed_data, lambda s: ("normal", "", "", "#000000"))
    assert len(lap_dlg.plots["yaw"].listDataItems()) > 0

    assert _analysis_bytes(result) == before, "rendering the yaw panels must not touch analysis arrays"
    assert result["summaries"] == summaries
    corner_dlg.close(); lap_dlg.close()


def test_flag_off_builds_no_yaw_panels(qt_app, monkeypatch, raw_params, pipeline_result, parsed_data, state):
    from ui.views.corner_trace_dialog import CornerTraceDialog, LapTraceDialog
    _with_flag(monkeypatch, raw_params, False)
    for dlg in (CornerTraceDialog(), LapTraceDialog()):
        assert "yaw" not in dlg.plots and "steer" not in dlg.plots
        assert dlg._yaw_panel_data(_result(pipeline_result, state), parsed_data) is None
        dlg.close()


def test_no_analysis_module_reads_the_yaw_pooling():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name in os.listdir(os.path.join(root, "modules")):
        if name.endswith(".py"):
            with open(os.path.join(root, "modules", name), encoding="utf-8") as fh:
                assert "yaw_pooling" not in fh.read(), f"modules/{name} references the display-only pooling"
