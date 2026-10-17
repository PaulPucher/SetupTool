# Kerb exclusion (one concept, 2026-10-05): kerb-affected CS samples (internal
# mark near_strike) are greyed or left out of the plots by
# corner_trace_display.kerb_exclusion_display. Drawing
# only -- these tests pin that no computed result changes and that the export
# path draws exactly the marked samples. Real Dubai data.

import copy
import os
import re

import numpy as np
import pytest

from core import plot_style
from core.trace_display import near_strike_split, kerb_exclusion_caption, kerb_scatter_split


def _lap_arrays(state, cs, lap):
    t = state["time"]
    m = (t >= lap["start_time"]) & (t <= lap["end_time"])
    order = np.argsort(state["s_m"][m])
    return {ax: (np.asarray(cs[f"CS_ratio_{ax}"])[m][order], np.asarray(cs[f"near_strike_{ax}"])[m][order])
            for ax in "fr"}, state["s_m"][m][order], m, order


def test_dubai_carries_near_strike_samples(pipeline_result):
    cs = pipeline_result["cs"]
    n = int(cs["near_strike_f"].sum() + cs["near_strike_r"].sum())
    assert n > 0, "Dubai has kerb strikes inside CS windows; expected marked samples"


@pytest.mark.parametrize("mode", ["grey", "exclude", "off"])
def test_split_hides_exactly_the_marked_samples(pipeline_result, mode):
    cs = pipeline_result["cs"]
    for ax in "fr":
        y = np.asarray(cs[f"CS_ratio_{ax}"], dtype=float)
        mark = np.asarray(cs[f"near_strike_{ax}"], dtype=bool)
        before = y.copy()
        main, overlay = near_strike_split(y, mark, mode)
        assert np.array_equal(y, before, equal_nan=True), "input must not be modified"
        hit = mark & np.isfinite(y)
        if mode == "off":
            assert np.array_equal(main, y, equal_nan=True) and overlay is None
            continue
        assert np.all(np.isnan(main[hit]))
        assert np.array_equal(main[~hit], y[~hit], equal_nan=True)
        if mode == "exclude":
            assert overlay is None
        else:
            assert np.array_equal(overlay[hit], y[hit])
            # overlay = marked samples plus their direct neighbours, nothing else
            grow = hit.copy()
            grow[1:] |= hit[:-1]
            grow[:-1] |= hit[1:]
            assert np.all(np.isnan(overlay[~grow]))


def test_analysis_output_identical_for_every_display_mode(parsed_data, effective_params):
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
    )

    def run(mode):
        p = copy.deepcopy(effective_params)
        p["corner_trace_display"]["kerb_exclusion_display"] = mode
        st = prepare_vehicle_state(parsed_data["channels"], p)
        beta = estimate_sideslip(st, p)
        cs = estimate_cornering_stiffness(estimate_slip_angles(st, beta, p), estimate_lateral_forces(st, p), st, p,
                                          parsed_data.get("laps", []))
        stab = estimate_yaw_moment_stability(st, beta, p, parsed_data.get("laps", []))
        return cs, stab, summarise_corners(parsed_data.get("corners", []), cs, stab, st)

    ref = run("off")
    for mode in ("grey", "exclude"):
        got = run(mode)
        for k in ("CS_ratio_f", "CS_ratio_r", "near_strike_f", "near_strike_r"):
            assert got[0][k].tobytes() == ref[0][k].tobytes()
        assert got[1]["stability_observed_Nm_per_deg"].tobytes() == ref[1]["stability_observed_Nm_per_deg"].tobytes()
        assert repr(got[2]) == repr(ref[2])


def test_no_analysis_module_reads_the_display_setting():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pat = re.compile(r"kerb_exclusion_display|near_strike_split|kerb_exclusion_caption|kerb_scatter_split")
    for name in os.listdir(os.path.join(root, "modules")):
        if name.endswith(".py"):
            with open(os.path.join(root, "modules", name), encoding="utf-8") as fh:
                assert not pat.search(fh.read()), f"modules/{name} reads the near-strike display setting"


def _export_laps(parsed_data, state, cs):
    laps = [l for l in parsed_data["laps"] if l.get("is_valid_for_analysis")]
    out = []
    for lap in laps[:2]:
        arr, s, m, order = _lap_arrays(state, cs, lap)
        out.append({"lap_number": lap["lap_number"], "s": s, "v_kmh": state["v_mps"][m][order] * 3.6,
                    "stab": np.full(s.size, np.nan), "cs_f": arr["f"][0], "cs_r": arr["r"][0],
                    "near_strike_f": arr["f"][1], "near_strike_r": arr["r"][1], "kerb": None})
    return out


THRESH = {"stab": -50.0, "strong_csf": -0.5, "moderate_csf": 0.5, "strong_csr": -0.5, "moderate_csr": 0.5}


def _grey_lines(fig):
    return [ln for ax in fig.axes for ln in ax.get_lines()
            if ln.get_color() == plot_style.NEAR_STRIKE_COLOR and len(ln.get_xdata()) > 0]


@pytest.mark.parametrize("mode", ["grey", "exclude", "off"])
def test_lap_figure_export_follows_the_mode(parsed_data, state, pipeline_result, mode):
    import matplotlib.pyplot as plt
    from core import figure_render
    laps = _export_laps(parsed_data, state, pipeline_result["cs"])
    n_mark = sum(int((lap[f"near_strike_{a}"] & np.isfinite(lap[f"cs_{a}"])).sum()) for lap in laps for a in "fr")
    n_fin = sum(int(np.isfinite(lap[f"cs_{a}"]).sum()) for lap in laps for a in "fr")
    fig = figure_render.render_lap_figure("Lap test", laps, THRESH, [], near_strike={"mode": mode})
    texts = [t.get_text() for t in fig.texts]
    # one caption in every mode, always with the count
    expected = kerb_exclusion_caption(mode, n_mark, n_fin)
    assert expected in texts and f"n = {n_mark}," in expected
    assert n_mark > 0
    assert bool(_grey_lines(fig)) == (mode == "grey")
    plt.close(fig)


def test_caption_is_the_one_ruled_sentence():
    # author wording 2026-10-07, "CS" added by author ruling 2026-10-17 (R5);
    # verbatim, plus the mode prefix and the count
    ruled = "kerb-affected samples excluded from CS verdict statistics, retained in CS arrays"
    assert kerb_exclusion_caption("grey", 3, 200) == f"Grey: {ruled} (n = 3, 1.5 %)."
    assert kerb_exclusion_caption("exclude", 3, 200) == f"Not drawn: {ruled} (n = 3, 1.5 %)."
    assert kerb_exclusion_caption("off", 3, 200) == f"Drawn normally: {ruled} (n = 3, 1.5 %)."


@pytest.mark.parametrize("mode", ["grey", "exclude", "off"])
def test_tyre_scatter_uses_the_same_kerb_affected_definition(mode):
    valid = np.array([1, 1, 1, 1, 0], bool)
    kerb = np.array([0, 1, 0, 0, 1], bool)
    mark = np.array([0, 0, 1, 0, 0], bool)
    filled, hollow = kerb_scatter_split(valid, kerb, mark, mode)
    if mode == "off":
        assert filled.tolist() == [1, 1, 1, 1, 0] and not hollow.any()
    else:
        assert filled.tolist() == [1, 0, 0, 1, 0]
        assert hollow.tolist() == ([0, 1, 1, 0, 0] if mode == "grey" else [0] * 5)


def test_retired_kerb_wording_is_gone_from_user_facing_text():
    # "near-strike" / "smear" are internal; product text says kerb-affected
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # "retained in all computed results": inaccurate since schema 12, superseded 2026-10-07
    pat = re.compile(r"""["'][^"'\n]*(near[- ]strike|smear|retained in all computed)[^"'\n]*["']""", re.I)
    for sub in ("ui", "core"):
        for dirpath, _dirs, files in os.walk(os.path.join(root, sub)):
            for name in files:
                if name.endswith(".py"):
                    with open(os.path.join(dirpath, name), encoding="utf-8") as fh:
                        hits = [m.group(0) for m in pat.finditer(fh.read()) if "near_strike" not in m.group(0)]
                    assert not hits, f"{sub}/{name}: retired kerb wording in a string: {hits}"
