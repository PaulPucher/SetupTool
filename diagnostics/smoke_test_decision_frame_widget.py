# Headless smoke test for the "Decision Frame" section (decision-matrix
# frame -- Stage 1 Phase 5/6 preview UI, now the only recommendation UI
# since Frame-Stage-2 Phase 3e removed the old Recommendations section and
# dropped "(preview)" from the toggle label). Same technique as smoke_test_
# measurement_points_widget.py: Qt's offscreen platform plugin, since
# tests/conftest.py deliberately keeps PyQt6 out of pytest -- the widget
# binding (toggle, button enable/disable, row rendering) is verified here
# against a real constructed OutingForm, not the pure decision_frame.py
# logic (covered separately by tests/test_decision_frame.py). [keep-
# reproduces] per diagnostics/README.md -- a reusable headless Qt smoke
# test, same category as smoke_test_corner_trace_dialog.py.
#
# Deliberately uses sideslip_source="kinematic" directly (never touching
# the live config, no flip-restore needed) purely to keep this widget-
# binding smoke test fast -- the real ekf_auto_pacejka fit chain was
# already exercised end to end against modules/decision_frame.py itself
# in diagnostics/inspect_decision_frame_phase2_check.py during
# development; this file's own job is the Qt binding, not the estimator.

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

app = QApplication([])

import models.driver, models.outing, models.raceweekend
from models.base import Session
from models.outing import Outing
from models.raceweekend import RaceWeekend
from ui.views.outing_form import OutingForm
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

s = Session()
weekend = s.get(RaceWeekend, 1)
outing = s.get(Outing, 1)

print("--- OutingForm: constructing against a real pre-existing outing ---")
form = OutingForm(weekend, lambda: None, outing=outing)
print("constructed OK")

assert hasattr(form, "btn_generate_decision_frame"), "Generate button not built"
assert hasattr(form, "decision_frame_panel"), "collapsible panel not built"
assert hasattr(form, "decision_frame_host_layout"), "row host not built"
print("Decision Frame widgets present OK")

assert form.decision_frame_panel.isHidden(), "panel should start collapsed"
assert form.btn_generate_decision_frame.isEnabled() is False, \
    "Generate button should start disabled (no analysis yet)"
print("initial collapsed/disabled state OK")

print("\n--- Toggle open/closed ---")
# isVisible() reflects the WHOLE ancestor chain (false here regardless of
# this widget's own flag, since `form` itself is never shown) -- isHidden()
# reflects only this widget's own explicit setVisible/hide state, which is
# what the toggle handler actually controls.
toggle_btns = [w for w in form.findChildren(type(form.btn_generate_decision_frame))
               if w.text() in ("> Decision Frame", "v Decision Frame")]
assert len(toggle_btns) == 1, f"expected exactly one toggle button, found {len(toggle_btns)}"
toggle_btn = toggle_btns[0]
toggle_btn.setChecked(True)
assert not form.decision_frame_panel.isHidden()
assert toggle_btn.text() == "v Decision Frame"
toggle_btn.setChecked(False)
assert form.decision_frame_panel.isHidden()
assert toggle_btn.text() == "> Decision Frame"
print("toggle show/hide + label swap OK")

print("\n--- Real pipeline (kinematic, fast) -> Generate -> rows render ---")
data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
long_forces = estimate_longitudinal_forces(state, data["channels"], params)
slip_ratio = estimate_slip_ratio(state, data["channels"], params)
ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)
summaries = summarise_corners(data.get("corners", []), cs, stab, state, ls=ls, lap_filter=None)

form.stability_result = {"summaries": summaries}
form.btn_generate_decision_frame.setEnabled(True)
form._generate_decision_frame()
print(f"summary label: {form.decision_frame_summary_label.text()!r}")

row_count = form.decision_frame_host_layout.count() - 1  # minus the trailing stretch
print(f"rows rendered: {row_count}")
assert row_count >= 0
print("Generate ran without raising OK")

print("\n--- Clear rows ---")
form._clear_decision_frame_rows()
assert form.decision_frame_host_layout.count() == 1
print("clear OK")

s.close()
print("\nALL SMOKE TESTS PASSED -- no exception raised during construction, toggling, or generation.")
