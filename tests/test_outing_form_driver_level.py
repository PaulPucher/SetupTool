# Regression: decision-frame Generate crashed on detached/absent Outing
# objects (lazy Outing.driver after session close). Headless Qt form against
# a throwaway SQLite DB -- the shared sessionmaker is rebound per test and
# restored, data/setuptool.db is never connected. Stability input = real
# Dubai summaries from the pipeline golden.
import datetime
import json
import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PyQt6.QtWidgets")

from sqlalchemy import create_engine

import models.base as base

GOLDEN = Path(__file__).parent / "golden" / "pipeline_dubai_ekf_auto_pacejka_cap1.json"


@pytest.fixture(scope="module")
def real_summaries():
    return json.loads(GOLDEN.read_text(encoding="utf-8"))["summaries"]


@pytest.fixture
def db(tmp_path):
    original_bind = base.Session.kw.get("bind")
    engine = create_engine(f"sqlite:///{tmp_path / 'throwaway.db'}")
    base.Session.configure(bind=engine)
    import models.driver, models.raceweekend, models.outing  # noqa: F401 -- table registration
    base.Base.metadata.create_all(engine)
    yield
    base.Session.configure(bind=original_bind)
    engine.dispose()


@pytest.fixture
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _seed(drivers):
    from models.driver import Driver
    from models.raceweekend import RaceWeekend
    s = base.Session()
    rows = [Driver(name=n, driving_level=lvl) for n, lvl in drivers]
    weekend = RaceWeekend(track="Dubai", series="GT3", car_number=1, year=2026)
    s.add_all(rows + [weekend])
    s.commit()
    ids = {"drivers": [d.id for d in rows], "weekend": weekend.id}
    s.close()
    return ids


def _add_outing(weekend_id, driver_id, number):
    from models.outing import Outing
    s = base.Session()
    o = Outing(race_weekend_id=weekend_id, driver_id=driver_id, number=number,
               date_time=datetime.datetime(2026, 9, 1, 10, number))
    s.add(o)
    s.commit()
    oid = o.id
    s.close()
    return oid


def _reopen(model, row_id):
    # exactly the production reopen shape: get, close, hand over detached
    s = base.Session()
    obj = s.get(model, row_id)
    s.close()
    return obj


@pytest.fixture
def spy(monkeypatch):
    import modules.decision_frame as df
    seen = {}
    real = df.generate_candidates

    def wrapper(*args, **kwargs):
        seen["driving_level"] = kwargs.get("driving_level")
        return real(*args, **kwargs)

    monkeypatch.setattr(df, "generate_candidates", wrapper)
    return seen


def _form(weekend_id, outing=None):
    from models.raceweekend import RaceWeekend
    from ui.views.outing_form import OutingForm
    return OutingForm(_reopen(RaceWeekend, weekend_id), on_back=lambda: None, outing=outing)


def _generate(form, summaries):
    form.stability_result = {"summaries": summaries}
    form._generate_decision_frame()


def test_reopened_outing_with_driver(db, app, spy, real_summaries):
    from models.outing import Outing
    ids = _seed([("A Driver", 3)])
    oid = _add_outing(ids["weekend"], ids["drivers"][0], 1)
    form = _form(ids["weekend"], _reopen(Outing, oid))
    assert form.driver_combo.currentData() == ids["drivers"][0]  # real driver_id unaffected
    _generate(form, real_summaries)
    assert spy["driving_level"] == 3


def test_new_outing_after_first_save(db, app, spy, real_summaries):
    ids = _seed([("A Driver", 3)])
    form = _form(ids["weekend"])
    form.driver_combo.setCurrentIndex(form.driver_combo.findData(ids["drivers"][0]))
    form._persist_outing()
    _generate(form, real_summaries)
    assert spy["driving_level"] == 3


def test_new_outing_never_saved(db, app, spy, real_summaries):
    ids = _seed([("A Driver", 3)])
    form = _form(ids["weekend"])
    assert form.outing is None
    form.driver_combo.setCurrentIndex(form.driver_combo.findData(ids["drivers"][0]))
    _generate(form, real_summaries)
    assert spy["driving_level"] == 3


def test_new_outing_defaults_to_no_driver(db, app, spy, real_summaries):
    # drivers exist but nothing is stored for this weekend -> "(no driver)",
    # never the alphabetically first driver; veto gets the unknown level
    ids = _seed([("A Driver", 3), ("B Driver", 8)])
    form = _form(ids["weekend"])
    assert form.driver_combo.currentData() is None
    _generate(form, real_summaries)
    assert spy["driving_level"] is None


def test_no_driver_save_reopen_round_trip(db, app, spy, real_summaries):
    from models.outing import Outing
    ids = _seed([("A Driver", 3)])
    form = _form(ids["weekend"])
    form._persist_outing()
    oid = form.outing.id
    assert _reopen(Outing, oid).driver_id is None  # stored as NULL
    reopened = _form(ids["weekend"], _reopen(Outing, oid))
    assert reopened.driver_combo.currentData() is None
    _generate(reopened, real_summaries)
    assert spy["driving_level"] is None


def test_veto_follows_displayed_driver_before_save(db, app, spy, real_summaries):
    # outing saved with driver A; combo switched to B, not saved -> B's level,
    # same on-screen convention as setup_data
    from models.outing import Outing
    ids = _seed([("A Driver", 3), ("B Driver", 8)])
    oid = _add_outing(ids["weekend"], ids["drivers"][0], 1)
    form = _form(ids["weekend"], _reopen(Outing, oid))
    form.driver_combo.setCurrentIndex(form.driver_combo.findData(ids["drivers"][1]))
    _generate(form, real_summaries)
    assert spy["driving_level"] == 8


def test_no_driver_available_passes_none(db, app, spy, real_summaries):
    ids = _seed([])
    _generate(_form(ids["weekend"]), real_summaries)
    assert spy["driving_level"] is None


def test_driver_lookup_none_is_clean(db):
    # PDF export path: no driver -> no name, never a fabricated one
    from models.driver import driver_name_and_level
    assert driver_name_and_level(None) == (None, None)


def test_generate_failure_is_logged_and_shown(db, app, capsys, monkeypatch):
    ids = _seed([])
    form = _form(ids["weekend"])

    def boom():
        raise RuntimeError("boom")

    monkeypatch.setattr(form, "_generate_decision_frame", boom)
    form._on_generate_decision_frame()
    out = capsys.readouterr()
    assert "[DECISION_FRAME] Generate failed:" in out.out
    assert "RuntimeError: boom" in out.err  # traceback.print_exc -> stderr
    assert form.decision_frame_summary_label.text() == "Generate failed: RuntimeError: boom"
