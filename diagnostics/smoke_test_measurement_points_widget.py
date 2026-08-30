# One-off headless smoke test for the splitter/diffuser measurement-point
# widget binding (thesis_notes.md "8. Splitter/diffuser measurement
# points", Phase 4) -- NOT part of the regression suite, run manually.
# Uses Qt's offscreen platform plugin, same technique as smoke_test_
# corner_trace_dialog.py, for the same reason: tests/conftest.py's own
# pipeline_result fixture deliberately keeps PyQt6 modules out of pytest,
# so the widget<->_active_inputs<->_collect_inputs/_load_inputs binding
# (as opposed to the pure reshape logic, covered by tests/test_setup_
# data_points.py) is verified here instead, against a real constructed
# OutingForm and a real Dubai outing.

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

app = QApplication([])

import models.driver, models.outing, models.raceweekend
from models.base import Session
from models.outing import Outing
from models.raceweekend import RaceWeekend
from ui.views.outing_form import OutingForm
from ui.views.measurement_points_widget import MeasurementPointsWidget

s = Session()
weekend = s.get(RaceWeekend, 1)
outing = s.get(Outing, 1)  # real Dubai "Warmup" outing, setup_data predates this feature

print("--- OutingForm: constructing against a real pre-existing outing ---")
form = OutingForm(weekend, lambda: None, outing=outing)
print("constructed OK")

mp_widgets = form.findChildren(MeasurementPointsWidget)
assert len(mp_widgets) == 4, f"expected 4 (splitter+diffuser x setup+setdown), got {len(mp_widgets)}"
splitter_widgets = [w for w in mp_widgets if w.outline == "splitter"]
diffuser_widgets = [w for w in mp_widgets if w.outline == "diffuser"]
assert len(splitter_widgets) == 2 and len(diffuser_widgets) == 2
print(f"found {len(mp_widgets)} MeasurementPointsWidget instances (2 splitter + 2 diffuser) OK")

print("\n--- Old-outing load: every point box must be empty ---")
for w in mp_widgets:
    for i, edit in enumerate(w.point_widgets, start=1):
        assert edit.text() == "", f"{w.outline} point {i} should be blank for a pre-existing outing, got {edit.text()!r}"
print("all 20 point boxes empty OK")

print("\n--- Registration: point widgets are the exact objects in _active_inputs ---")
setup_splitter = [form.setup_inputs["car"][f"splitter_point_{n}"] for n in range(1, 6)]
setdown_splitter = [form.setdown_inputs["car"][f"splitter_point_{n}"] for n in range(1, 6)]
assert setup_splitter is not setdown_splitter
assert setup_splitter[0] is not setdown_splitter[0], "setup and setdown must be independent widget instances"
print("setup/setdown widget instances are independent OK")

print("\n--- Type widget in, collect, save, reload, verify round trip ---")
setup_splitter[0].setText("12.5")
setup_splitter[2].setText("8")
saved_json = form._collect_setup_data()
import json
saved = json.loads(saved_json)
print(f"collected splitter_points: {saved['car'].get('splitter_points')}")
assert saved["car"]["splitter_points"] == [12.5, None, 8.0, None, None]

# Clear the widgets, then reload from the saved JSON -- confirms the full
# widget -> collect -> reshape -> save -> reshape -> load -> widget loop,
# not just the reshape functions in isolation.
for edit in setup_splitter:
    edit.setText("")
form._load_setup_data(saved_json)
reloaded_texts = [e.text() for e in setup_splitter]
print(f"reloaded point box text: {reloaded_texts}")
assert reloaded_texts == ["12.5", "", "8.0", "", ""]
print("full widget round trip OK")

s.close()
print("\nALL SMOKE TESTS PASSED -- no exception raised during construction, binding, or round trip.")
