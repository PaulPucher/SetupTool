# Work order BUG-2, items 2 and 3: each fix has a test that failed before it.
# Real data only: channel-loss cases are the real NBR JST outing 29 export
# with channels graded failed, as the range gate would grade them; manifests
# and payloads come from the real chain or the app DB, read-only. Nothing
# here writes config/, the app DB or tools/thesis_figures/local/.

import datetime
import json
import os
import shutil
from types import SimpleNamespace

import pytest

from tests.conftest import find_data_file

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

JST29 = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"


@pytest.fixture(scope="module")
def jst29():
    from modules.csv_parser import parse_csv
    path = find_data_file(JST29)
    if path is None:
        pytest.skip("NBR JST outing 29 export not on this machine")
    return parse_csv(path)


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


# --- B5: plot update with every plot channel unusable ---

def test_b5_plots_without_any_plot_channel(db, app, jst29):
    from ui.views.outing_form import OutingForm
    form = OutingForm(_weekend(db), on_back=lambda: None)
    form.parsed_data = _graded_failed(jst29, [c["key"] for c in form.plot_channels])
    form._update_plots(None)  # used to raise UnboundLocalError (first_key)
    lap = next(l["lap_number"] for l in jst29["laps"] if l.get("is_valid_for_analysis"))
    form._update_plots(lap)


# --- B8: weekend deleted from the outings view's Edit dialog ---

def test_b8_edit_then_delete_returns_to_the_weekend_list(db, app, monkeypatch):
    from ui.views import outings
    w = _weekend(db)
    went_back = []
    view = outings.OutingsView(w, on_back=lambda: went_back.append(True))
    # the empty-weekend delete asks first since BUG-3; a real modal box
    # would block the run
    from ui.views import weekend_dialog
    monkeypatch.setattr(weekend_dialog.QMessageBox, "question",
                        staticmethod(lambda *a, **k: weekend_dialog.QMessageBox.StandardButton.Yes))

    def exec_and_delete(dlg):
        dlg._delete()  # no outings, confirmed -> deleted, accept()
        return dlg.result()
    monkeypatch.setattr(outings.WeekendDialog, "exec", exec_and_delete)
    view._open_edit_dialog()  # used to raise AttributeError on None.track
    assert went_back == [True]


# --- B7: double-click after a header sort opens the clicked row ---

def _sorted_desc(table):
    from PyQt6.QtCore import Qt
    table.sortItems(0, Qt.SortOrder.DescendingOrder)


def test_b7_weekend_row_after_sort(db, app, monkeypatch):
    from ui.views import weekends
    a = _weekend(db, track="Aragon", date=datetime.date(2026, 7, 1))
    _weekend(db, track="Zandvoort", date=datetime.date(2026, 5, 1))
    view = weekends.WeekendsView()
    _sorted_desc(view.table)  # Zandvoort first, query order has Aragon first
    opened = []
    monkeypatch.setattr(weekends, "OutingsView", lambda wk, on_back: opened.append(wk.track) or SimpleNamespace())
    monkeypatch.setattr(view.stack, "addWidget", lambda w: None)
    monkeypatch.setattr(view.stack, "setCurrentWidget", lambda w: None)
    row = next(r for r in range(view.table.rowCount()) if view.table.item(r, 0).text() == a.track)
    view._open_weekend(row, 0)
    assert opened == ["Aragon"]


def test_b7_driver_row_after_sort(db, app, monkeypatch):
    from models.driver import Driver
    from ui.views import drivers
    s = db.Session()
    s.add_all([Driver(name="Driver A"), Driver(name="Driver B")])
    s.commit()
    s.close()
    view = drivers.DriversView()
    _sorted_desc(view.table)
    opened = []

    class Dlg:
        def __init__(self, parent, driver=None):
            opened.append(driver.name)

        def exec(self):
            return False
    monkeypatch.setattr(drivers, "DriverDialog", Dlg)
    row = next(r for r in range(view.table.rowCount()) if view.table.item(r, 0).text() == "Driver A")
    view._open_edit_dialog(row, 0)
    assert opened == ["Driver A"]


def test_b7_rows_filled_while_sorted_keep_their_cells(db, app):
    # filling a sorted table re-sorts after every setItem and scatters a row's
    # cells over several rows; each row must still hold one outing
    from models.outing import Outing
    from ui.views import outings
    w = _weekend(db)
    s = db.Session()
    for n, name in ((1, "B run"), (2, "A run"), (3, "C run")):
        s.add(Outing(race_weekend_id=w.id, number=n, name=name, date_time=datetime.datetime(2026, 7, 1, 10 + n)))
    s.commit()
    s.close()
    view = outings.OutingsView(w, on_back=lambda: None)
    view.table.sortItems(1)
    view.load_data()
    pairs = {(view.table.item(r, 0).text(), view.table.item(r, 1).text(), view.table.item(r, 5).text())
             for r in range(view.table.rowCount())}
    assert pairs == {("1", "B run", "01.07.2026 11:00"), ("2", "A run", "01.07.2026 12:00"),
                     ("3", "C run", "01.07.2026 13:00")}


# --- B15a: tab_sessions with a session the grid rate refuses ---

def test_b15a_grid_rate_refusal_is_a_not_analysable_row(monkeypatch, jst29):
    import modules.csv_parser as csv_parser
    from modules.stability_analysis import CS_CHAIN_FAST_CHANNELS
    from tools.thesis_figures import common, tab_sessions
    if common.db_outing(12)["csv_path"] is None or common.load_labels().get(common.db_outing(12)["weekend"]) is None:
        pytest.skip("outing 12 not labelled on this machine")
    monkeypatch.setattr(csv_parser, "parse_csv", lambda path: _graded_failed(jst29, CS_CHAIN_FAST_CHANNELS))
    rows = tab_sessions.collect_rows([12])  # used to raise ValueError (no grid rate)
    (o, row), = rows
    assert o == 12 and row["estimation_path"] == "not analysable"
    assert "grid rate" in row["path_reason"]


# --- B16: fig_k_sessions / find_example_corner argument and status guards ---

@pytest.fixture(scope="module")
def degenerate_manifest(jst29):
    # the real fit L on the real file without its yaw-rate channel
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.stability_analysis import load_parameters
    from modules.tyre_fit_auto import fit_session_pacejka
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    man = fit_session_pacejka(_graded_failed(jst29, ["sclu_yaw_rate"]), params)
    assert man["status"] == "degenerate" and "axles" not in man
    return man


def _stub_session(monkeypatch, common, man):
    monkeypatch.setattr(common, "load_session", lambda o: {"row": {"id": o}})
    monkeypatch.setattr(common, "stored_payload", lambda s: ({"fit_manifest": man, "fallback_used": True}, None))
    monkeypatch.setattr(common, "write_label_map", lambda *a, **k: None)


def test_b16_k_sessions_refuses_a_fit_without_curve(monkeypatch, tmp_path, degenerate_manifest):
    from tools.thesis_figures import common, fig_k_sessions
    _stub_session(monkeypatch, common, degenerate_manifest)
    with pytest.raises(common.Refused) as e:  # used to raise KeyError 'axles'
        fig_k_sessions.build([12], labels=["x"], allow_status=True, out=str(tmp_path))
    assert "outing 12" in str(e.value.code)


def _stored_rows(outings):
    from tools.thesis_figures import common
    rows = []
    for o in outings:
        row = common.db_outing(o)
        rows.append((o, {"row": row}, json.loads(row["analysis_data"])["fit_manifest"], []))
    return rows


def test_b16_k_sessions_separator_not_in_outings(monkeypatch, tmp_path):
    from tools.thesis_figures import common, fig_k_sessions
    monkeypatch.setattr(fig_k_sessions, "collect", lambda o, *a: _stored_rows(o))
    monkeypatch.setattr(common, "write_label_map", lambda *a, **k: None)
    with pytest.raises(common.Refused):  # used to raise IndexError after writing the CSV
        fig_k_sessions.build([11, 12], labels=["a", "b"], separator_after=99, out=str(tmp_path))
    assert not os.listdir(tmp_path)


def test_b16_k_sessions_label_count(monkeypatch, tmp_path):
    from tools.thesis_figures import common, fig_k_sessions
    monkeypatch.setattr(fig_k_sessions, "collect", lambda o, *a: _stored_rows(o))
    monkeypatch.setattr(common, "write_label_map", lambda *a, **k: None)
    with pytest.raises(common.Refused):  # used to drop outing 12 silently
        fig_k_sessions.build([11, 12], labels=["only one"], out=str(tmp_path))


def test_b16_find_example_corner_gates_before_reading_the_fit(monkeypatch, degenerate_manifest):
    from tools.thesis_figures import common, find_example_corner
    _stub_session(monkeypatch, common, degenerate_manifest)
    with pytest.raises(common.Refused) as e:  # used to raise KeyError 'axles'
        find_example_corner.candidates(12)
    assert "fit status 'degenerate'" in str(e.value.code)


# --- B18: PDF export with markup characters in the text fields ---

def test_b18_pdf_with_markup_characters(tmp_path):
    from core.pdf_export import generate_setup_pdf
    from tools.thesis_figures import common
    setup = json.loads(common.db_outing(16)["setup_data"])  # real sheet
    # reportlab reads the text as markup: "<" before a letter opens a tag
    # that never closes; a bare "&" passes, an unknown tag is dropped silently
    setup.setdefault("car", {})["notes"] = "rear ARB 3 -> 4 & diff +1, check <rear toe"
    outing = SimpleNamespace(number=3, name="Quali<b>2", session_type="Q <1>",
                             date_time=datetime.datetime(2026, 7, 1, 17, 34), driver_name="A & B",
                             setup_data=json.dumps(setup))
    out = tmp_path / "sheet.pdf"
    generate_setup_pdf(outing, SimpleNamespace(track="Spa & Francorchamps"), str(out))  # used to raise ValueError
    assert out.stat().st_size > 0


# --- B22: Settings Save writes nothing the user did not change ---

@pytest.fixture
def settings_copy(tmp_path, monkeypatch):
    from ui.views import settings_view
    p, c = tmp_path / "parameters.json", tmp_path / "channels.json"
    shutil.copyfile("config/parameters.json", p)
    shutil.copyfile("config/channels.json", c)
    monkeypatch.setattr(settings_view, "PARAMETERS_PATH", str(p))
    monkeypatch.setattr(settings_view, "CHANNELS_PATH", str(c))
    # the save clears the loaders' caches; restore them for the next test
    from modules.stability_analysis import load_parameters, load_car_data
    yield p, c
    load_parameters.cache_clear()
    load_car_data.cache_clear()


def test_b22_save_without_edits_writes_nothing(app, settings_copy):
    from ui.views.settings_view import SettingsView
    p, c = settings_copy
    before = p.read_bytes(), c.read_bytes()
    SettingsView()._on_save_clicked()  # used to re-dump both files (0.10 -> 0.1, arrays split)
    assert (p.read_bytes(), c.read_bytes()) == before


def test_b22_value_finer_than_the_widget_survives_a_save(app, settings_copy):
    # a config value with more decimals than its spinbox shows (kerb
    # dilation, 2 decimals) must not be rounded by a Save of another field
    from ui.views.settings_view import SettingsView
    p, _c = settings_copy
    d = json.loads(p.read_text(encoding="utf-8"))
    d["stability_estimation"]["kerb_dilation_s"] = 0.125
    p.write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
    view = SettingsView()
    view.section2_widgets[("parameters", "stability_estimation", "moving_speed_min_mps")].setValue(6.0)
    view._on_save_clicked()
    se = json.loads(p.read_text(encoding="utf-8"))["stability_estimation"]
    assert se["moving_speed_min_mps"] == 6.0 and se["kerb_dilation_s"] == 0.125


# --- Outing delete: confirmation, and the pipeline sidecar goes too ---

def _outing(base, w):
    from models.outing import Outing
    s = base.Session()
    o = Outing(race_weekend_id=w.id, number=1, date_time=datetime.datetime(2026, 7, 1, 17, 34))
    s.add(o)
    s.commit()
    s.refresh(o)
    s.expunge(o)
    s.close()
    return o


def test_outing_delete_asks_first(db, app, monkeypatch):
    from PyQt6.QtWidgets import QMessageBox
    from models.outing import Outing
    from ui.views.outing_form import OutingForm
    w = _weekend(db)
    o = _outing(db, w)
    asked = []
    monkeypatch.setattr(QMessageBox, "question",
                        staticmethod(lambda *a, **k: asked.append(a) or QMessageBox.StandardButton.No))
    form = OutingForm(w, on_back=lambda: None, outing=o)
    form._delete_outing()  # used to delete at once
    s = db.Session()
    assert asked and s.get(Outing, o.id) is not None
    s.close()


def test_outing_delete_removes_its_sidecar(db, app, monkeypatch, tmp_path):
    from PyQt6.QtWidgets import QMessageBox
    from models.outing import Outing
    import modules.pipeline_sidecar as ps
    from ui.views.outing_form import OutingForm
    monkeypatch.setattr(ps, "SIDECAR_DIR", str(tmp_path))
    w = _weekend(db)
    o = _outing(db, w)
    ps.write_sidecar(o.id, {"schema_version": 0}, {"x": 1})
    assert os.path.exists(ps._sidecar_path(o.id))
    monkeypatch.setattr(QMessageBox, "question", staticmethod(lambda *a, **k: QMessageBox.StandardButton.Yes))
    OutingForm(w, on_back=lambda: None, outing=o)._delete_outing()
    s = db.Session()
    assert s.get(Outing, o.id) is None
    s.close()
    assert not os.path.exists(ps._sidecar_path(o.id))
