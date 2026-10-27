# Kerb-gap bridge in the trace windows is display-only: it must draw across
# kerb holes without ever reaching an analysis array. Real Dubai data.

import copy
import os
import re

import numpy as np

from core.trace_display import kerb_gap_bridge


def _nan_equal(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return a.shape == b.shape and np.array_equal(np.isnan(a), np.isnan(b)) and \
        np.array_equal(a[~np.isnan(a)], b[~np.isnan(b)])


def test_bridge_spans_kerb_gaps_only_and_leaves_input_untouched(pipeline_result, parsed_data, state):
    cs = pipeline_result["cs"]
    t, s_m, kerb = state["time"], state["s_m"], state["kerb_mask"]
    bridged_runs = 0
    for lap in parsed_data["laps"]:
        if not lap.get("is_valid_for_analysis"):
            continue
        m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
        x, k = s_m[m], kerb[m]
        for key in ("CS_ratio_f", "CS_ratio_r"):
            y = cs[key][m]
            before = y.copy()
            yb = kerb_gap_bridge(x, y, k)
            assert _nan_equal(y, before), "the bridge must not modify its input"
            assert yb is not y
            fin = np.isfinite(yb)
            # every bridged sample is either a finite anchor equal to y, or a
            # NaN sample of y inside a run that holds a kerb sample
            assert np.allclose(yb[fin & np.isfinite(y)], y[fin & np.isfinite(y)])
            gap = fin & ~np.isfinite(y)
            edges = np.diff(np.concatenate(([0], gap.astype(np.int8), [0])))
            for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
                assert k[a:b].any(), "a bridged gap must contain a kerb sample"
                bridged_runs += 1
            # a NaN run without kerb samples is never bridged
            bad = ~np.isfinite(y)
            e2 = np.diff(np.concatenate(([0], bad.astype(np.int8), [0])))
            for a, b in zip(np.where(e2 == 1)[0], np.where(e2 == -1)[0]):
                if not k[a:b].any():
                    assert not np.isfinite(yb[a:b]).any()
    assert bridged_runs > 0, "Dubai has kerb strikes inside analysis laps; expected at least one bridge"


def test_analysis_output_identical_with_bridge_flag_on_and_off(parsed_data, effective_params):
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
    )

    def run(flag):
        p = copy.deepcopy(effective_params)
        p["corner_trace_display"]["kerb_gap_bridge"] = flag
        st = prepare_vehicle_state(parsed_data["channels"], p)
        beta = estimate_sideslip(st, p)
        slip = estimate_slip_angles(st, beta, p)
        forces = estimate_lateral_forces(st, p)
        cs = estimate_cornering_stiffness(slip, forces, st, p, parsed_data.get("laps", []))
        stab = estimate_yaw_moment_stability(st, beta, p, parsed_data.get("laps", []))
        summ = summarise_corners(parsed_data.get("corners", []), cs, stab, st)
        return st, beta, cs, stab, summ

    on, off = run(True), run(False)
    for k in ("yaw_rate_radps", "ay_mps2", "v_mps", "kerb_mask"):
        assert np.asarray(on[0][k]).tobytes() == np.asarray(off[0][k]).tobytes()
    assert on[1].tobytes() == off[1].tobytes()
    for k in ("CS_ratio_f", "CS_ratio_r"):
        assert on[2][k].tobytes() == off[2][k].tobytes()
    assert on[3]["stability_observed_Nm_per_deg"].tobytes() == off[3]["stability_observed_Nm_per_deg"].tobytes()
    assert repr(on[4]) == repr(off[4])


def test_no_analysis_module_reads_the_bridge():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pat = re.compile(r"kerb_gap_bridge|trace_display")
    for d in ("modules", "core"):
        for name in os.listdir(os.path.join(root, d)):
            # figure_render is the export render path (display only, draws
            # the arrays it is given); it applies the bridge like the dialog.
            # Widening confirmed by the author 2026-09-30: modules/ staying
            # fully guarded is the part that matters.
            if not name.endswith(".py") or (d == "core" and name in ("trace_display.py", "figure_render.py")):
                continue
            with open(os.path.join(root, d, name), encoding="utf-8") as fh:
                assert not pat.search(fh.read()), f"{d}/{name} references the display-only bridge"


def test_export_draws_bridge_and_leaves_analysis_arrays_untouched(pipeline_result, parsed_data, state,
                                                                  effective_params):
    from core import figure_render, plot_style as ps
    cs, stab = pipeline_result["cs"], pipeline_result["stab"]
    stab_obs = stab["stability_observed_Nm_per_deg"]
    before = {k: cs[k].tobytes() for k in ("CS_ratio_f", "CS_ratio_r")}
    before_stab = stab_obs.tobytes()
    t, s_m, kerb = state["time"], state["s_m"], state["kerb_mask"]
    cls = effective_params["classification"]
    from modules.classifier import cs_decision_lines
    thresholds = {"stab": cls["stab_neg_thresh_Nm_per_deg"]["value"], **cs_decision_lines()}

    def laps(with_kerb):
        out = []
        for lap in parsed_data["laps"]:
            if not lap.get("is_valid_for_analysis"):
                continue
            m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
            out.append({"lap_number": lap["lap_number"], "s": s_m[m], "v_kmh": state["v_mps"][m] * 3.6,
                        "stab": stab_obs[m], "cs_f": cs["CS_ratio_f"][m], "cs_r": cs["CS_ratio_r"][m],
                        "kerb": kerb[m] if with_kerb else None})
        return out

    def bridge_lines(fig):
        # stab, CS f, CS r data axes of render_lap_figure: every other axes
        # is a side-legend strip
        data_axes = fig.axes[0::2][1:4]
        return sum(1 for ax in data_axes for ln in ax.get_lines()
                   if ln.get_linestyle() == "--" and ln.get_alpha() == ps.KERB_BRIDGE_ALPHA_PRINT
                   and ln.get_label() == "_nolegend_")

    import matplotlib.pyplot as plt
    fig_on = figure_render.render_lap_figure("Dubai", laps(True), thresholds, [])
    fig_off = figure_render.render_lap_figure("Dubai", laps(False), thresholds, [])
    try:
        assert bridge_lines(fig_on) > 0, "Dubai analysis laps hold kerb gaps; the export must bridge them"
        assert bridge_lines(fig_off) == 0, "no kerb mask passed (flag off) -> no bridge drawn"
    finally:
        plt.close(fig_on)
        plt.close(fig_off)
    for k in before:
        assert cs[k].tobytes() == before[k], "rendering must not touch analysis arrays"
    assert stab_obs.tobytes() == before_stab


def test_export_png_holds_every_artist(pipeline_result, parsed_data, state, effective_params, tmp_path):
    # constrained_layout leaves side-legend text partly outside the nominal
    # canvas; save_png's tight crop must widen the file to every axes' drawn
    # extent (legends included) instead of cutting at the edge
    import matplotlib.pyplot as plt
    from PIL import Image
    from core import figure_render, plot_style as ps
    cs, stab = pipeline_result["cs"], pipeline_result["stab"]
    t, s_m, kerb = state["time"], state["s_m"], state["kerb_mask"]
    cls = effective_params["classification"]
    from modules.classifier import cs_decision_lines
    thresholds = {"stab": cls["stab_neg_thresh_Nm_per_deg"]["value"], **cs_decision_lines()}
    laps = []
    for lap in parsed_data["laps"]:
        if lap.get("is_valid_for_analysis"):
            m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
            laps.append({"lap_number": lap["lap_number"], "s": s_m[m], "v_kmh": state["v_mps"][m] * 3.6,
                         "stab": stab["stability_observed_Nm_per_deg"][m], "cs_f": cs["CS_ratio_f"][m],
                         "cs_r": cs["CS_ratio_r"][m], "kerb": kerb[m]})
    fig = figure_render.render_lap_figure("Dubai", laps, thresholds, [])
    try:
        fig.canvas.draw()
        r = fig.canvas.get_renderer()
        tight = fig.get_tightbbox(r)            # inches
        dpi = fig.dpi
        for ax in fig.axes:
            bb = ax.get_tightbbox(r)
            if bb is None:
                continue
            assert bb.x0 / dpi >= tight.x0 - 0.01 and bb.x1 / dpi <= tight.x1 + 0.01
            assert bb.y0 / dpi >= tight.y0 - 0.01 and bb.y1 / dpi <= tight.y1 + 0.01
        path = tmp_path / "lap.png"
        figure_render.save_png(fig, str(path))
    finally:
        plt.close(fig)
    w, h = Image.open(path).size
    pad = 2 * ps.PRINT_PAD_INCHES
    assert abs(w - (tight.width + pad) * ps.PRINT_DPI) <= 2
    assert abs(h - (tight.height + pad) * ps.PRINT_DPI) <= 2


# --- shading budget + cross-lap envelope (author rulings 2026-10-06) ---------

def _exclusion_runs(x, mask):
    from ui.views.corner_trace_dialog import _contiguous_runs
    return _contiguous_runs(x, mask)


def test_exclusion_union_and_tint_never_overlap(parsed_data, state):
    # real Dubai laps: kerb and not-moving spans of every analysis lap
    from core.trace_display import interval_subtract, interval_union
    t, s_m = state["time"], state["s_m"]
    spans = []
    for lap in parsed_data["laps"]:
        if not lap.get("is_valid_for_analysis"):
            continue
        m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
        spans += _exclusion_runs(s_m[m], state["kerb_mask"][m])
        spans += _exclusion_runs(s_m[m], ~state["moving_mask"][m])
    holes = interval_union(spans)
    assert all(a < b for a, b in holes)
    assert all(holes[i][1] < holes[i + 1][0] for i in range(len(holes) - 1)), "union must be disjoint"
    # a tint over the whole lap minus the holes: no piece overlaps a hole
    pieces = interval_subtract((float(np.nanmin(s_m)), float(np.nanmax(s_m))), holes)
    for lo, hi in pieces:
        assert lo < hi
        assert not any(lo < b and hi > a for a, b in holes)


def test_cross_lap_envelope_brackets_the_median(pipeline_result, parsed_data, state, effective_params):
    from core.trace_display import cross_lap_envelope
    cfg = effective_params["corner_trace_display"]["cs_envelope"]
    assert list(cfg["percentiles"]) == [10, 90]
    t, s_m = state["time"], state["s_m"]
    laps_xy = []
    for lap in parsed_data["laps"]:
        if lap.get("is_valid_for_analysis"):
            m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
            laps_xy.append((s_m[m], pipeline_result["cs"]["CS_ratio_f"][m]))
    env = cross_lap_envelope(laps_xy, cfg)
    assert env is not None
    grid, p_lo, p_hi = env
    ok = np.isfinite(p_lo) & np.isfinite(p_hi)
    assert ok.any()
    assert np.all(p_lo[ok] <= p_hi[ok] + 1e-12)
    assert np.all(np.diff(grid) > 0)


def test_unchecked_lap_leaves_no_mark_in_the_corner_window(pipeline_result, parsed_data, state):
    # author defect 2026-10-07: an unchecked lap's kerb dots stayed visible.
    # One lap-selection state: after unchecking, none of that lap's samples
    # may remain in the tyre-curve tab, and it owns no curve or bridge item.
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import pytest
    QtWidgets = pytest.importorskip("PyQt6.QtWidgets")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    import pyqtgraph as pg
    from ui.views.corner_trace_dialog import CornerTraceDialog

    result = {"state": state, "cs": pipeline_result["cs"], "stab": pipeline_result["stab"],
              "corners": pipeline_result["corners"], "summaries": pipeline_result["summaries"],
              "slip": pipeline_result["slip"], "forces": pipeline_result["forces"], "ls": None,
              "lap_filter": None, "sideslip_source": None, "fit_manifest": None, "events": None}
    valid = {l["lap_number"] for l in parsed_data["laps"] if l.get("is_valid_for_analysis")}
    summary = next(s for s in result["summaries"] if s["lap_number"] in valid)
    dlg = CornerTraceDialog()
    dlg.show_corner(summary, result, parsed_data, preserve_visible={ln: True for ln in valid})
    off = next(ln for ln in sorted(valid) if ln != summary["lap_number"])
    dlg.show_corner(summary, result, parsed_data,
                    preserve_visible={ln: ln != off for ln in valid})
    app.processEvents()

    lap = next(l for l in parsed_data["laps"] if l["lap_number"] == off)
    t = state["time"]
    in_lap = (t >= lap["start_time"]) & (t <= lap["end_time"])
    assert off not in dlg.lap_curve_items and off not in dlg.lap_bridge_items
    for axle, a_key, f_key in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        a = pipeline_result["slip"][a_key][in_lap]
        f = pipeline_result["forces"][f_key][in_lap]
        ok = np.isfinite(a) & np.isfinite(f)
        lap_pts = set(zip(np.round(np.degrees(a[ok]), 9), np.round(f[ok], 6)))
        drawn = set()
        for item in dlg.tyre_plots[axle].items:
            if isinstance(item, pg.PlotDataItem):
                x, y = item.getData()
                if x is not None and len(x):
                    drawn |= set(zip(np.round(np.asarray(x, float), 9), np.round(np.asarray(y, float), 6)))
        assert not (lap_pts & drawn), f"{axle}: unchecked lap {off} still drawn in the tyre-curve tab"
    dlg.close()
