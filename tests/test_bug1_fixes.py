# Work order BUG-1, part A: each fix has a test that failed before it.
# Real data only: the channel-loss case is the real NBR JST outing 29 export
# with one channel graded failed, as the range gate would grade it.

import numpy as np
import pytest

from tests.conftest import find_data_file

JST29 = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"


@pytest.fixture(scope="module")
def jst29_no_az():
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    path = find_data_file(JST29)
    if path is None:
        pytest.skip("NBR JST outing 29 export not on this machine")
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=None))
    data = parse_csv(path)
    channels = dict(data["channels"])
    channels["log_acc_z"] = dict(channels["log_acc_z"], quality="failed")
    state = prepare_vehicle_state(channels, params)
    return {"state": state, "laps": data["laps"], "cfg": params["tyre_fit_auto"]}


def test_b1_straights_run_without_a_kerb_mask(jst29_no_az):
    # no usable az -> no kerb mask (None), as everywhere else in the chain;
    # the straights must then exclude nothing rather than crash on None | None
    from modules.straights import steering_straights
    st = jst29_no_az["state"]
    assert st["kerb_mask"] is None and st["kerb_smear_zone"] is None
    got = steering_straights(st, jst29_no_az["laps"], jst29_no_az["cfg"])
    zeros = np.zeros(st["time"].size, dtype=bool)
    ref = steering_straights(dict(st, kerb_mask=zeros, kerb_smear_zone=zeros), jst29_no_az["laps"],
                             jst29_no_az["cfg"])
    assert got["segs"] and got["segs"] == ref["segs"] and got["d_min"] == ref["d_min"]


# --- B4: Back must not erase a stored path / analysis it never replaced ---
# Headless form against a throwaway SQLite DB (same pattern as
# test_outing_form_driver_level.py); the stored analysis is the real NBR
# JST 29 golden payload, the saved path one that does not resolve here.

import datetime
import json
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

GOLDEN = Path(__file__).parent / "golden" / "default_nbr_jst29.json"
MISSING_PATH = "Q:/not_a_data_root/NBR-73-029-101-JST-26.07.01.1734.txt"


@pytest.fixture
def db(tmp_path):
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


@pytest.fixture
def app():
    QtWidgets = pytest.importorskip("PyQt6.QtWidgets")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _saved_outing(base):
    from models.outing import Outing
    from models.raceweekend import RaceWeekend
    s = base.Session()
    w = RaceWeekend(track="Nordschleife", series="GT3", car_number=1, year=2026)
    s.add(w)
    s.commit()
    payload = GOLDEN.read_text(encoding="utf-8")
    o = Outing(race_weekend_id=w.id, number=1, date_time=datetime.datetime(2026, 7, 1, 17, 34),
               csv_path=MISSING_PATH, analysis_data=payload)
    s.add(o)
    s.commit()
    ids = (w.id, o.id)
    s.close()
    return ids, payload


def _reopen(base, model, row_id):
    s = base.Session()
    obj = s.get(model, row_id)
    s.close()
    return obj


def _form(base, wid, oid):
    from models.outing import Outing
    from models.raceweekend import RaceWeekend
    from ui.views.outing_form import OutingForm
    return OutingForm(_reopen(base, RaceWeekend, wid), on_back=lambda: None, outing=_reopen(base, Outing, oid))


def test_b4_back_after_file_not_found_keeps_stored_data(db, app):
    from models.outing import Outing
    (wid, oid), payload = _saved_outing(db)
    form = _form(db, wid, oid)
    form._auto_load_csv(MISSING_PATH)  # "Data file not found at saved path"
    assert form.loaded_csv_path is None
    form._save_outing()  # what Back does
    row = _reopen(db, Outing, oid)
    assert row.csv_path == MISSING_PATH and row.analysis_data == payload


def test_b4_back_before_the_load_finished_keeps_stored_data(db, app):
    # the auto-load timer has not fired yet (no event loop here)
    from models.outing import Outing
    (wid, oid), payload = _saved_outing(db)
    form = _form(db, wid, oid)
    form._save_outing()
    row = _reopen(db, Outing, oid)
    assert row.csv_path == MISSING_PATH and row.analysis_data == payload


def test_b4_explicit_clear_still_removes_the_data(db, app):
    # Clear data (confirmed) + Save is the one way to drop path and analysis
    from models.outing import Outing
    (wid, oid), _payload = _saved_outing(db)
    form = _form(db, wid, oid)
    form._reset_data_state()
    form._persist_outing()
    row = _reopen(db, Outing, oid)
    assert row.csv_path == "" and row.analysis_data is None


# --- Weekend delete with outings: refused with a message, nothing deleted ---

def test_weekend_with_outings_is_not_deleted(db, app, monkeypatch):
    from models.outing import Outing
    from models.raceweekend import RaceWeekend
    from ui.views import weekend_dialog
    (wid, oid), payload = _saved_outing(db)
    shown = []
    monkeypatch.setattr(weekend_dialog.QMessageBox, "warning",
                        staticmethod(lambda *a, **k: shown.append(a[2] if len(a) > 2 else k)))
    dlg = weekend_dialog.WeekendDialog(weekend=_reopen(db, RaceWeekend, wid))
    dlg._delete()  # used to raise IntegrityError (NOT NULL race_weekend_id)
    assert _reopen(db, RaceWeekend, wid) is not None
    row = _reopen(db, Outing, oid)
    assert row is not None and row.analysis_data == payload
    assert len(shown) == 1 and "1 outing" in shown[0]
    assert dlg.result() != weekend_dialog.QDialog.DialogCode.Accepted


# --- Thesis tables: estimation path and fit status are two statements ---
# Real record: labelled NBR outing 16, observer path on a marginal fit
# (K at the band edge, mu in the edge zone).

DEV16 = Path(__file__).parent.parent / "diagnostics" / "results_labeled_dev_20261024_schema27" / \
    "Labeled_NBR_04_outing16.json"


def test_estimation_path_names_the_path_only():
    from tools.thesis_figures import common
    man = json.loads(DEV16.read_text(encoding="utf-8"))["fit_manifest"]
    assert man["status"] == "marginal"
    assert common.estimation_path(man, False, None) == ("observer", "")
    assert common.fit_status(man) == ("marginal", "K at the band edge, mu in the edge zone")


def test_fit_status_maps_the_r_sweep_reasons():
    # the R-sweep marginal reasons are the selection codes tyre_fit_auto
    # writes; read them from the source so a renamed code fails here
    import re
    from tools.thesis_figures import common
    src = (Path(__file__).parent.parent / "modules" / "tyre_fit_auto.py").read_text(encoding="utf-8")
    codes = set(re.findall(r'"(r_sweep_[a-z_]+)"', src))
    assert codes >= {"r_sweep_guard_fail", "r_sweep_no_in_band"}
    for c in codes:
        assert common.fit_status({"status": "marginal", "marginal_reason": c}) == ("marginal", "default R")
