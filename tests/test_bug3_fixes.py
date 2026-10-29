# Work order BUG-3, part B: each fix has a test that failed before it.
# Real data only: the real NBR JST outing 29 export (channels graded failed
# where a fault is needed, as the range gate would grade them), real corner
# dicts and fits from it, and the real exported thesis-table row. Nothing
# here writes config/, the app DB or tools/thesis_figures/local/.

import csv
import json
import os
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from tests.conftest import find_data_file

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JST29 = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"


@pytest.fixture(scope="module")
def jst29_path():
    path = find_data_file(JST29)
    if path is None:
        pytest.skip("NBR JST outing 29 export not on this machine")
    return path


@pytest.fixture(scope="module")
def jst29(jst29_path):
    from modules.csv_parser import parse_csv
    return parse_csv(jst29_path)


def _graded_failed(parsed, names):
    channels = dict(parsed["channels"])
    for n in names:
        channels[n] = dict(channels[n], quality="failed")
    return dict(parsed, channels=channels)


@pytest.fixture
def app():
    QtWidgets = pytest.importorskip("PyQt6.QtWidgets")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def db(tmp_path):
    # throwaway SQLite DB (pattern of test_bug1_fixes.py)
    from sqlalchemy import create_engine
    import models.base as base
    original_bind = base.Session.kw.get("bind")
    engine = create_engine(f"sqlite:///{tmp_path / 'throwaway.db'}")
    base.Session.configure(bind=engine)
    import models.driver, models.raceweekend, models.outing  # noqa: F401 -- table registration
    base.Base.metadata.create_all(engine)
    yield base
    base.Session.configure(bind=original_bind)
    engine.dispose()


def _weekend(base, **kw):
    from models.raceweekend import RaceWeekend
    s = base.Session()
    w = RaceWeekend(**{"track": "Nordschleife", "series": "GT3", "car_number": 1, "year": 2026, **kw})
    s.add(w)
    s.commit()
    s.refresh(w)
    s.expunge(w)
    s.close()
    return w


# --- B13: one axle without any damper corner -> no NaN session fit ---

def test_b13_one_axle_without_dampers_keeps_finite_loads(jst29):
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.stability_analysis import (estimate_lateral_forces, estimate_vertical_loads, load_car_data,
                                            load_parameters, prepare_vehicle_state)
    raw = load_parameters()
    assert raw["stability_estimation"]["vertical_load_source"] == "measured"
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    parsed = _graded_failed(jst29, ["log_dms_dam_rl", "log_dms_dam_rr"])
    state = prepare_vehicle_state(parsed["channels"], params)
    forces = estimate_lateral_forces(state, params)
    fz = estimate_vertical_loads(state, forces, params, channels=parsed["channels"], car_data=load_car_data())
    # used to fit c_session on an all-NaN rear total -> NaN axle totals and
    # NaN "reconstructed" corners
    assert fz["c_session_N_per_mps2"] is None
    for k in ("fz_f_N", "fz_r_N", "fz_fl_N", "fz_fr_N", "fz_rl_N", "fz_rr_N"):
        assert np.all(np.isfinite(fz[k])), k
    src = fz["vertical_load_source_per_sample"]
    assert set(np.unique(src["rl"])) == {"static_fallback"} and set(np.unique(src["rr"])) == {"static_fallback"}


# --- B21: per-lap speed-class tie independent of the hash seed ---

def test_b21_speed_class_tie_does_not_depend_on_the_hash_seed(jst29):
    # two real corners of different classes under one id, the per-lap form
    # (no canonical bracket keys); tie rule of corner_aggregation: closest
    # to medium, then the lower class -> low against high gives low
    by_class = {}
    for c in jst29["corners"]:
        by_class.setdefault(c["speed_class"], c)
    assert {"low", "high"} <= set(by_class)
    pair = [{"stable_corner_id": 1, "speed_class": by_class[k]["speed_class"]} for k in ("high", "low")]
    code = ("import json, sys; from modules.corner_analysis import canonical_speed_classes; "
            "print(canonical_speed_classes(json.loads(sys.argv[1]))[1])")
    got = set()
    for seed in range(16):
        env = dict(os.environ, PYTHONHASHSEED=str(seed))
        out = subprocess.run([sys.executable, "-c", code, json.dumps(pair)], cwd=ROOT, env=env,
                             capture_output=True, text=True, check=True)
        got.add(out.stdout.strip())
    assert got == {"low"}  # used to follow set order, i.e. the seed


# --- B15b: CSV fields with a comma stay in one column ---

PRC2_ROW = ["PRC 2", "PRC", "Practice", "D1", 3, 5, 16, "excluded",
            "damper loads rejected, straight-line total 0.08 x static weight",
            "marginal (mu in the edge zone)", 0.146144]
NBR1_ROW = ["NBR 1", "NBR", "Practice", "D1", 2, 4, 15, "observer", "-", "ok", 0.227406]
COLS = ("label", "track", "session_type", "driver_code", "analysis_laps", "total_laps", "n_corners",
        "estimation_path", "path_reason", "fit_status", "health_score")


def test_b15b_comma_field_stays_in_its_column(tmp_path, monkeypatch):
    from tools.thesis_figures import common
    monkeypatch.setattr(common, "config_hash", lambda: "test")
    cols = {k: [PRC2_ROW[i], NBR1_ROW[i]] for i, k in enumerate(COLS)}
    path = tmp_path / "tab.csv"
    common.write_csv(str(path), cols, {"script": "test"})
    text = path.read_text(encoding="utf-8")
    rows = list(csv.reader(l for l in text.splitlines() if not l.startswith("#")))
    assert all(len(r) == len(COLS) for r in rows)  # PRC 2 used to spill over 12 columns
    assert rows[1][8] == PRC2_ROW[8]
    # a row without a comma is written exactly as before
    assert "NBR 1,NBR,Practice,D1,2,4,15,observer,-,ok,0.227406" in text.splitlines()


# --- B17: fig_example_fit applies the production degenerate rule ---

def test_b17_unconverged_fit_is_not_marked_ok(monkeypatch, tmp_path):
    from tools.thesis_figures import common, fig_example_fit
    row = common.db_outing(12)
    if row is None or row["csv_path"] is None or common.load_labels().get(row["weekend"]) is None:
        pytest.skip("outing 12 not labelled on this machine")
    real_capture = common.capture_fit

    def capture_unconverged(session):
        # the real fits of outing 12, front reported as a Powell run that
        # did not converge (as _fit_axle_l reports one)
        got = real_capture(session)
        got["fits"][0]["fit"] = dict(got["fits"][0]["fit"], powell_converged=False)
        return got

    monkeypatch.setattr(common, "capture_fit", capture_unconverged)
    monkeypatch.setattr(common, "write_label_map", lambda *a, **k: None)
    with pytest.raises(common.Refused) as e:  # used to pass as status ok and write the figure
        fig_example_fit.build(12, out=str(tmp_path))
    assert "degenerate" in str(e.value.code)
    assert not any(files for _, _, files in os.walk(tmp_path))


# --- B12: an early return leaves nothing of the previous corner to export ---

def test_b12_corner_export_cleared_on_early_return(app):
    from ui.views.corner_trace_dialog import CornerTraceDialog
    dlg = CornerTraceDialog()
    dlg._export_data = {"corner_label": "C3"}  # what the previous corner left behind
    dlg.track_map_plot.plot([0.0, 1.0], [0.0, 1.0])
    summary = {"stable_corner_id": 5, "lap_number": 2}
    dlg.show_corner(summary, {"summaries": [summary]}, {"laps": []})  # cached summaries only
    assert dlg._export_data is None
    assert not dlg.track_map_plot.listDataItems()
    dlg.close()


def test_b12_lap_export_cleared_on_early_return(app):
    from ui.views.corner_trace_dialog import LapTraceDialog
    dlg = LapTraceDialog()
    dlg._export_data = {"lap_label": "Lap 3"}
    dlg.show_lap(4, {"summaries": []}, {"laps": []}, classify_fn=None)
    assert dlg._export_data is None
    dlg.close()


# --- B9: a second file in the same form starts without the first one's results ---

def test_b9_second_file_clears_the_previous_results(db, app, jst29, jst29_path):
    from PyQt6.QtWidgets import QLabel
    from ui.views.outing_form import OutingForm
    form = OutingForm(_weekend(db), on_back=lambda: None)
    form.progress = SimpleNamespace(close=lambda: None)
    form.loader_thread = SimpleNamespace(path=jst29_path)
    form._on_csv_loaded(jst29)
    # what a rendered analysis of the first file leaves on the form
    form._cached_schema_mismatch = (26, 27)
    form.stability_status_label.setText("Analysed laps [1, 2]")
    form.cards_host_layout.insertWidget(0, QLabel("C1 verdict card"))
    for host in form.setup_layer_hosts.values():
        host.insertWidget(0, QLabel("setup row"))
    form._on_csv_loaded(jst29)
    app.processEvents()
    assert form._cached_schema_mismatch is None
    assert form.stability_status_label.text() == ""
    assert form.cards_host_layout.count() == 1
    assert all(host.count() == 1 for host in form.setup_layer_hosts.values())
    assert form.lap_table.rowCount() > 0 and form.btn_analyse.isEnabled()


# --- Deleting an empty weekend asks first ---

def test_empty_weekend_delete_asks_first(db, app, monkeypatch):
    from models.raceweekend import RaceWeekend
    from ui.views import weekend_dialog
    w = _weekend(db)
    asked = []

    def answer_no(*a, **k):
        asked.append(a)
        return weekend_dialog.QMessageBox.StandardButton.No

    monkeypatch.setattr(weekend_dialog.QMessageBox, "question", staticmethod(answer_no))
    dlg = weekend_dialog.WeekendDialog(weekend=w)
    dlg._delete()  # used to delete at once
    s = db.Session()
    assert s.get(RaceWeekend, w.id) is not None
    s.close()
    assert len(asked) == 1
    assert dlg.result() != weekend_dialog.QDialog.DialogCode.Accepted


# --- B24: every detect_events return path carries the same keys ---

@pytest.fixture(scope="module")
def jst29_chain():
    # the golden default chain (tests/generate_golden.run_default_chain) up to
    # the CS estimate, on NBR JST 29: it has floor events, the Dubai sample
    # of the shared fixtures has none
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs
    from tests.conftest import resolve_data_path
    from tests.generate_golden import ACCURACY_CAP, GOLDEN_SESSIONS
    path = resolve_data_path(GOLDEN_SESSIONS["nbr_jst29"][1])
    p = load_parameters()
    p = apply_resolved_vehicle(p, resolve_accuracy(p, setup_data=None, cap=ACCURACY_CAP))
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    state = prepare_vehicle_state(data["channels"], p)
    beta, man, _gate, _fb, _fr, cs_in = resolve_sideslip_beta_and_cs_inputs(
        state, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    slip = estimate_slip_angles(state, beta, p)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(state, p), state, p, data["laps"],
                                      **cs_in["cs_kwargs"])
    return SimpleNamespace(args=(cs, state, data["corners"], data["laps"]), man=man, params=p, slip=slip)


def test_b24_event_record_has_one_shape(jst29_chain):
    import copy
    from modules.limit_events import detect_events, detect_floor_events
    c = jst29_chain
    computed = detect_events(*c.args, c.man, c.params, slip=c.slip)
    off = copy.deepcopy(c.params)
    off["limit_events"]["enabled"] = False
    disabled = detect_events(*c.args, c.man, off)
    no_fit = detect_events(*c.args, None, c.params)
    assert set(disabled) == set(no_fit) == set(computed)  # early returns lacked line / floor / label
    # without the slip input an event still has window_start_t, as None
    no_slip = detect_floor_events(*c.args, c.params, None)
    assert computed["events"] and no_slip["events"]
    with_slip = {k for e in computed["events"] for k in e} - {"recurrence", "recurring_laps"}
    for e in no_slip["events"]:
        assert set(e) == with_slip and e["window_start_t"] is None


# --- B10: screen, export and thesis figure draw the classifier's CS lines ---

def _cs_level_sets():
    from modules.classifier import cs_decision_lines
    from modules.stability_analysis import load_parameters
    v = cs_decision_lines()
    cl = load_parameters()["classification"]
    unused = {cl["MODERATE_CSF"]["value"], cl["STRONG_CSR"]["value"]}  # read by no verdict rule
    return {"cs_f": {v["boundary_f"], v["strong_f"]}, "cs_r": {v["boundary_r"], v["info_r"]}}, unused, v


def test_b10_screen_draws_the_classifier_levels(app):
    import pyqtgraph as pg
    from ui.views.corner_trace_dialog import CornerTraceDialog
    want, unused, values = _cs_level_sets()
    dlg = CornerTraceDialog(None)
    dlg._add_cs_lines(values)
    for panel, levels in want.items():
        got = {round(i.value(), 9) for i in dlg.plots[panel].items if isinstance(i, pg.InfiniteLine)}
        assert got == {round(x, 9) for x in levels} and not got & unused


def test_b10_export_draws_the_classifier_levels(pipeline_result, parsed_data, state, effective_params):
    import matplotlib.pyplot as plt
    from core import figure_render
    want, unused, values = _cs_level_sets()
    cs, stab = pipeline_result["cs"], pipeline_result["stab"]
    t, s_m = state["time"], state["s_m"]
    laps = []
    for lap in parsed_data["laps"]:
        if lap.get("is_valid_for_analysis"):
            m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
            laps.append({"lap_number": lap["lap_number"], "s": s_m[m], "v_kmh": state["v_mps"][m] * 3.6,
                         "stab": stab["stability_observed_Nm_per_deg"][m], "cs_f": cs["CS_ratio_f"][m],
                         "cs_r": cs["CS_ratio_r"][m], "kerb": None})
    thresholds = {"stab": effective_params["classification"]["stab_neg_thresh_Nm_per_deg"]["value"], **values}
    fig = figure_render.render_lap_figure("Lap", laps, thresholds, [])
    try:
        # data axes speed, stab, CS f, CS r; every other axes is a legend strip
        ax_f, ax_r = fig.axes[0::2][2:4]
        for panel, ax in (("cs_f", ax_f), ("cs_r", ax_r)):
            got = {float(ln.get_ydata()[0]) for ln in ax.get_lines()
                   if list(ln.get_xdata()) == [0, 1] and len(set(ln.get_ydata())) == 1}
            assert got == want[panel] and not got & unused
    finally:
        plt.close(fig)


def test_b10_thesis_figure_draws_the_classifier_levels():
    import re
    from tools.thesis_figures import common, fig_example_corner
    data_dir = os.path.join(json.load(open(os.path.join(ROOT, "config", "thesis_figures.json"),
                                           encoding="utf-8"))["thesis_figures_dir"], "data")
    if not os.path.exists(os.path.join(data_dir, "fig_example_corner.csv")):
        pytest.skip("thesis figure data not on this machine")

    def read(name):
        with open(os.path.join(data_dir, name), encoding="utf-8") as f:
            rows = list(csv.reader(l for l in f if not l.startswith("#")))
        return {c: [r[i] for r in rows[1:]] for i, c in enumerate(rows[0])}
    cols = {k: np.array(v, dtype=float) for k, v in read("fig_example_corner.csv").items()}
    ph = read("fig_example_corner_phases.csv")
    phases = {"s_start_m": [float(x) for x in ph["s_start_m"]], "s_end_m": [float(x) for x in ph["s_end_m"]],
              "phase_label": ph["phase_label"]}
    want, unused, v = _cs_level_sets()
    fit_range = {"f": (-6.0, 6.5), "r": (-4.5, 4.8)}
    tex = fig_example_corner._tex("b", "p/", cols, phases, common.lines(), cols["s_m"][0], cols["s_m"][-1], [],
                                  fit_range)
    got = {float(m) for m in re.findall(r"\\addplot\[axle(?:Front|Rear), [^\]]*\] coordinates \{\([-\d.]+,([-\d.e]+)\)", tex)}
    # B10 amended 2026-10-27: the thesis figure draws only the lines a
    # verdict rule acts on; the rear information tier stays on the app screen
    assert got == {float(f"{x:.4g}") for x in (want["cs_f"] | want["cs_r"]) - {v["info_r"]}}
    assert float(f"{v['info_r']:.4g}") not in got and not got & unused


# --- B11: the corner window tints the phases the verdict names ---

def test_b11_named_phases_match_the_verdict_text():
    from modules.classifier import classify_corner, verdict_phases
    from modules.corner_aggregation import PHASE_WORD_LONG
    with open(os.path.join(ROOT, "tests", "golden", "default_nbr_jst29.json"), encoding="utf-8") as f:
        summaries = json.load(f)["summaries"]
    n_named = 0
    for s in summaries:
        _sev, _short, long_, _state = classify_corner(s)
        # long parts "Understeer, <phase>, ..." / "Open-loop unstable, <phase>, ..."
        said = {p.split(", ")[1] for p in long_.split(" | ") if p.split(", ")[0] in
                ("Understeer", "Oversteer", "Open-loop unstable")}
        named = verdict_phases(s)
        assert {PHASE_WORD_LONG[p] for p in named} == said
        n_named += bool(named)
    assert n_named >= 15   # NBR 29: 18 corner verdicts name a phase


def test_b11_corner_window_tints_named_phases_only(app, jst29_chain):
    import pyqtgraph as pg
    from ui.style import BAD, BORDER, TEXT_DIM
    from ui.views.corner_trace_dialog import CornerTraceDialog, _phase_bands_for_lap
    cs, state, corners, _laps = jst29_chain.args
    corner = next(c for c in corners if c.get("apex_lap_distance_m") is not None
                  and all(c["segments"][p][1] > c["segments"][p][0] for p in ("entry_2_turnin", "exit_4")))
    t, s_m = state["time"], state["s_m"]
    bands = [p for p, _a, _b in _phase_bands_for_lap(corner, t, s_m)]

    def run(named):
        dlg = CornerTraceDialog(None)
        specs = dlg._add_phase_bands(corner, t, s_m, named_phases=named)
        apex = [i for i in dlg.plots["cs_f"].items if isinstance(i, pg.InfiniteLine)]
        return {p: (spec[2], spec[3]) for p, spec in zip(bands, specs)}, apex[0].pen.color().name()

    tints, apex_colour = run(["entry_2_turnin", "apex_3", "exit_4"])
    assert tints["entry_2_turnin"] == (BAD, 70) and tints["exit_4"] == (BAD, 70)
    assert all(v == (BORDER, 30) for p, v in tints.items() if p not in ("entry_2_turnin", "exit_4"))
    assert apex_colour == pg.mkColor(BAD).name()
    tints, apex_colour = run([])          # verdict ok: neutral bands, grey apex line
    assert all(v == (BORDER, 30) for v in tints.values())
    assert apex_colour == pg.mkColor(TEXT_DIM).name()
