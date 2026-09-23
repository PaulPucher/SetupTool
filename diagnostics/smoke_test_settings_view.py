# Headless smoke test for SettingsView's Section 4 (decision-frame cost-
# function weights, DECISION LAYER SPEC C2, 2026-09-22). Same offscreen-Qt
# technique as smoke_test_decision_frame_widget.py / smoke_test_
# measurement_points_widget.py. Verifies the specific claim C2 needed
# checked: values ACTUALLY persist across a restart, not just within one
# already-running instance (SettingsView's own JSON read-modify-write
# pattern, confirmed distinct from outing_form.py's accuracy_cap_combo,
# which has no write path at all and never persists).
#
# Restores config/decision_frame.json's cost_function.severity to its
# original value on exit either way (success or failure) -- this script
# writes to the real config file to prove real persistence, so it must
# never leave that file mutated behind it.

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import json

from PyQt6.QtWidgets import QApplication

app = QApplication([])

from ui.views.settings_view import SettingsView, DECISION_FRAME_PATH

with open(DECISION_FRAME_PATH, encoding="utf-8") as f:
    original = json.load(f)
original_severity = original["cost_function"]["severity"]

try:
    print("--- SettingsView: construct, check Section 4 widgets populated from disk ---")
    view = SettingsView()
    widget = view.section4_widgets[("cost_function", "severity")]
    assert widget.value() == original_severity, (widget.value(), original_severity)
    print(f"OK -- severity widget shows {widget.value()} (matches file)")

    print("--- change value, click Save, confirm file on disk actually changed ---")
    new_value = round(original_severity + 1.23, 2)
    widget.setValue(new_value)
    view._on_save_clicked()
    with open(DECISION_FRAME_PATH, encoding="utf-8") as f:
        on_disk = json.load(f)
    assert on_disk["cost_function"]["severity"] == new_value, on_disk["cost_function"]["severity"]
    print(f"OK -- file on disk now shows {on_disk['cost_function']['severity']}")

    print("--- simulate a restart: construct a FRESH SettingsView, confirm it shows the new value ---")
    view2 = SettingsView()
    widget2 = view2.section4_widgets[("cost_function", "severity")]
    assert widget2.value() == new_value, (widget2.value(), new_value)
    print(f"OK -- fresh instance shows {widget2.value()} -- real cross-restart persistence confirmed")

    print("--- confirm provenance note/tooltip is actually populated (C2: 'each with provenance note display') ---")
    assert widget.toolTip(), "severity weight widget has no tooltip -- provenance note not wired"
    print(f"OK -- tooltip present: {widget.toolTip()[:80]}...")

    print("\nALL CHECKS PASSED")
finally:
    with open(DECISION_FRAME_PATH, "w", encoding="utf-8", newline="") as f:
        json.dump(original, f, indent=2)
        f.write("\n")
    print("Restored config/decision_frame.json to its original content.")
