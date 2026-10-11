# Headless reproduction of defect (b) (author app check 2026-10-06): editing
# the driver-feedback corner count (10 -> 15) while an analysis runs. Real
# form, real Dubai file, real analysis thread; the edit is delivered as user
# key events (QTest), so a locked (disabled) widget refuses it exactly as it
# would refuse the author. Every exception raised in the Qt event loop is
# captured (custom excepthook) instead of aborting the app.
# Throwaway RaceWeekend row, deleted in the finally block; no outing is
# created, so nothing is persisted.
#
# Run: python diagnostics/smoke_test_analysis_lock.py   [keep-reproduces]

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys
import time
import traceback
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6.QtCore import Qt
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication

app = QApplication([])

import models.driver, models.outing, models.raceweekend  # noqa: F401,E401
from models.base import Session
from models.raceweekend import RaceWeekend
from modules.csv_parser import parse_csv
from ui.views.outing_form import OutingForm

DUBAI = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
EDIT_AFTER_S = 2.0
TIMEOUT_S = 1800

errors = []


def _hook(etype, value, tb):
    errors.append("".join(traceback.format_exception(etype, value, tb)))


sys.excepthook = _hook


def _pump(seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        app.processEvents()
        time.sleep(0.02)


session = Session()
weekend = RaceWeekend(track="analysis-lock smoke test", series="test", car_number=0, year=2026)
session.add(weekend)
session.commit()
try:
    form = OutingForm(weekend, lambda: None, outing=None)
    form.show()
    data = parse_csv(DUBAI)
    form.loader_thread = types.SimpleNamespace(path=DUBAI)
    form.progress = types.SimpleNamespace(close=lambda: None)
    form._on_csv_loaded(data)
    print(f"feedback corner count before: {form.corner_count_spin.value()}, rows {len(form.corner_rows)}")

    form._run_stability_analysis()
    _pump(EDIT_AFTER_S)
    running = form.stab_thread.isRunning()
    spin = form.corner_count_spin
    print(f"analysis running at the edit: {running}; corner-count control enabled: {spin.isEnabled()}; "
          f"feedback table enabled: {form.feedback_table.isEnabled()}")
    spin.setFocus()
    QTest.keyClick(spin, Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier)
    QTest.keyClicks(spin, "15")
    QTest.keyClick(spin, Qt.Key.Key_Return)
    print(f"after the typed edit: corner count {spin.value()}, rows {len(form.corner_rows)}")

    t0 = time.perf_counter()
    while form.stab_thread.isRunning() and time.perf_counter() - t0 < TIMEOUT_S:
        _pump(0.5)
    _pump(2.0)
    done = form.stability_result is not None and "summaries" in (form.stability_result or {})
    print(f"thread finished: {not form.stab_thread.isRunning()}; result rendered: {done}; "
          f"corner count now {spin.value()}, rows {len(form.corner_rows)}; inputs enabled again: "
          f"{spin.isEnabled() and form.feedback_table.isEnabled()}")
    print(f"exceptions in the event loop (analysis): {len(errors)}")

    # trace dialogs on the finished result (shading budget / default view):
    # corner trace for the first corner with an event, else the first corner;
    # lap trace for its lap; one opt-in panel toggled; screenshots saved
    shot_dir = os.environ.get("SMOKE_SHOT_DIR")
    summaries = form.stability_result["summaries"]
    evs = ((form.stability_result.get("events") or {}).get("events") or [])
    pick = next((s for s in summaries for e in evs
                 if e["lap"] == s["lap_number"] and e["corner"] == s["stable_corner_id"]), summaries[0])
    n_before = len(errors)
    form._open_corner_trace(pick)
    _pump(3.0)
    dlg = form._corner_trace_dialog
    print(f"corner trace: C{pick['stable_corner_id']} lap {pick['lap_number']}; visible panels "
          f"{sorted(dlg._visible_panels)}; checked laps {[k for k, v in dlg.lap_visible.items() if v]}")
    if shot_dir:
        dlg.grab().save(os.path.join(shot_dir, "corner_trace_default.png"))
    dlg._on_panel_toggled("speed", True)
    _pump(1.0)
    if shot_dir:
        dlg.grab().save(os.path.join(shot_dir, "corner_trace_speed_on.png"))
    form._open_lap_trace()
    _pump(3.0)
    if shot_dir and form._lap_trace_dialog is not None:
        form._lap_trace_dialog.grab().save(os.path.join(shot_dir, "lap_trace.png"))
    print(f"exceptions in the event loop (trace dialogs): {len(errors) - n_before}")
    for e in errors:
        print(e)
finally:
    session.delete(weekend)
    session.commit()
    print("throwaway weekend removed")
