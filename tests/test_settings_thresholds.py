# WP-SETTINGS (2026-09-28): classification thresholds editable in Settings
# as recorded manual overrides. Settings writes the real config files, so
# every test that saves restores their exact bytes afterwards.

import copy
import json
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core.threshold_overrides import apply_threshold_edit, is_manual

CONFIG_FILES = ("config/parameters.json", "config/channels.json",
                "config/recommendations.json", "config/decision_frame.json")
GOLDEN_SUMMARIES = "tests/golden/pipeline_dubai_ekf_auto_pacejka_cap1.json"


@pytest.fixture
def restore_config():
    saved = {p: open(p, "rb").read() for p in CONFIG_FILES}
    yield
    for p, raw in saved.items():
        with open(p, "wb") as f:
            f.write(raw)
    from modules.stability_analysis import load_parameters
    load_parameters.cache_clear()


@pytest.fixture(scope="module")
def qapp():
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _entry():
    return {"value": -0.07, "derived_from": "2026-09-02, gap-selected"}


def test_first_override_records_derived_default_and_prose():
    e = _entry()
    assert apply_threshold_edit(e, 0.1, "2026-09-28") == "manual"
    assert e["value"] == 0.1 and e["derived_default"] == -0.07
    assert e["derived_from"].endswith("; manually set 2026-09-28; data-derived default: -0.07")


def test_second_override_never_rewrites_derived_default():
    e = _entry()
    apply_threshold_edit(e, 0.1, "2026-09-28")
    apply_threshold_edit(e, 0.2, "2026-09-29")
    assert e["derived_default"] == -0.07 and e["value"] == 0.2


def test_back_to_derived_default_is_a_restore():
    e = _entry()
    apply_threshold_edit(e, 0.1, "2026-09-28")
    assert apply_threshold_edit(e, -0.07, "2026-09-29") == "restored"
    assert not is_manual(e) and e["value"] == -0.07
    assert e["derived_from"].endswith("; restored to data-derived default 2026-09-29")


def test_unchanged_value_leaves_entry_untouched():
    e = _entry()
    before = copy.deepcopy(e)
    assert apply_threshold_edit(e, -0.07, "2026-09-28") == "unchanged"
    assert e == before


def _real_summaries():
    with open(GOLDEN_SUMMARIES, encoding="utf-8") as f:
        return json.load(f)["summaries"]


def _severities(summaries):
    from modules.stability_analysis import load_parameters
    from ui.views.outing_form import OutingForm
    load_parameters.cache_clear()  # what a fresh process sees
    return [OutingForm._classify_corner(None, s)[0] for s in summaries]


def test_settings_round_trip_edit_persist_reload_classify(qapp, restore_config):
    # edit -> Save -> file on disk -> fresh view shows "manual" + restore ->
    # classifier reads the new value (real Dubai summaries)
    from ui.views.settings_view import SettingsView
    summaries = _real_summaries()
    before = _severities(summaries)

    view = SettingsView()
    view.section3_widgets["STRONG_CSR"].setValue(0.5)  # nearly every rear reading becomes "strong"
    view._on_save_clicked()

    with open("config/parameters.json", encoding="utf-8") as f:
        entry = json.load(f)["classification"]["STRONG_CSR"]
    assert entry["value"] == 0.5 and entry["derived_default"] == -0.07
    assert "manually set" in entry["derived_from"]

    fresh = SettingsView()
    assert not fresh.section3_manual_labels["STRONG_CSR"].isHidden()
    assert not fresh.section3_restore_buttons["STRONG_CSR"].isHidden()
    assert fresh.section3_manual_labels["STRONG_CSF"].isHidden()

    # severity "strong" also needs destabilising yaw, so the raised rear
    # threshold shows as more non-normal verdicts, not necessarily "strong"
    after = _severities(summaries)
    assert sum(s != "normal" for s in after) > sum(s != "normal" for s in before)


def test_restore_control_returns_derived_value_and_clears_tag(qapp, restore_config):
    from ui.views.settings_view import SettingsView
    view = SettingsView()
    view.section3_widgets["MODERATE_CSF"].setValue(0.3)
    view._on_save_clicked()
    view._restore_threshold("MODERATE_CSF")
    view._on_save_clicked()

    with open("config/parameters.json", encoding="utf-8") as f:
        entry = json.load(f)["classification"]["MODERATE_CSF"]
    assert entry["value"] == 0.15 and "derived_default" not in entry
    assert "restored to data-derived default" in entry["derived_from"]
    fresh = SettingsView()
    assert fresh.section3_manual_labels["MODERATE_CSF"].isHidden()
    assert fresh.section3_restore_buttons["MODERATE_CSF"].isHidden()


def test_untouched_save_records_no_override(qapp, restore_config):
    from ui.views.settings_view import SettingsView
    SettingsView()._on_save_clicked()
    with open("config/parameters.json", encoding="utf-8") as f:
        cls = json.load(f)["classification"]
    assert not any("derived_default" in cls[k] for k in
                   ("STRONG_CSF", "STRONG_CSR", "MODERATE_CSF", "MODERATE_CSR", "stab_neg_thresh_Nm_per_deg"))


def test_stab_row_states_kinematic_era_default(qapp):
    # the verdict marker stays keyed on stab_thresh_calibrated_for_sideslip_source;
    # the row states the same fact while the estimators differ
    from ui.views.settings_view import SettingsView, STAB_KEY, STAB_KINEMATIC_ERA_NOTE
    params = json.load(open("config/parameters.json", encoding="utf-8"))
    differs = (params["stability_estimation"]["sideslip_source"]
               != params["classification"]["stab_thresh_calibrated_for_sideslip_source"])
    note = SettingsView().section3_note_labels[STAB_KEY].text()
    assert note.startswith("Yaw stability below this = unstable yaw.")
    assert (STAB_KINEMATIC_ERA_NOTE in note) == differs
    assert differs  # live config: ekf_auto_pacejka vs kinematic


def test_notes_carry_no_dates_and_dead_rows_are_gone(qapp):
    import re
    from PyQt6.QtWidgets import QLabel
    from ui.views.settings_view import SettingsView
    view = SettingsView()
    texts = [w.text() for w in view.findChildren(QLabel)]
    assert not any(re.search(r"\d{4}-\d{2}-\d{2}", t) for t in texts)
    assert not any("Derived from data" in t or "read-only here by design" in t for t in texts)
    assert not any(t.startswith(("Display cutoff score", "Driver weighting: neutral level")) for t in texts)


def test_effective_cl_area_sign_and_value():
    from modules.wheel_loads import effective_cl_area
    # v3 session fit c = 1.5979 N/(m/s)^2 -> downforce, Cl*A negative
    assert effective_cl_area(1.5979, 1.225) == pytest.approx(-2.6088, abs=1e-4)


def test_tyre_band_rows_round_trip_and_flag_reads_live_config(qapp, restore_config):
    # WP-POLISH-2 addendum: the front band row writes FL and FR; the flag
    # reads config at call time (Generate), so the edit acts immediately
    import numpy as np
    from modules.decision_frame import load_decision_frame_config, tyre_pressure_flags
    from ui.views.settings_view import SettingsView

    t = np.arange(200) / 50.0
    state = {"time": t}
    channels = {"tpms_press_fl": {"time": t, "data": np.full(200, 1.72), "quality": "valid"}}
    corners = [{"stable_corner_id": 1, "lap_number": 1, "segments": {"apex_3": (1.8, 2.2)}}]

    def fl_state():
        flags = tyre_pressure_flags(load_decision_frame_config(), channels=channels, corners=corners, state=state)
        return next(f.split(":")[0] for f in flags if "FL 1.72" in f)

    assert fl_state() == "under target"  # elicited front band 1.85-1.95
    view = SettingsView()
    spec, widget = next((s, w) for s, w in view.section5_widgets if s["label"] == "Front band min")
    widget.setValue(1.70)
    view._on_save_clicked()

    targets = json.load(open("config/decision_frame.json", encoding="utf-8"))["tyre_pressure_target"]
    assert targets["fl"]["min_bar"] == targets["fr"]["min_bar"] == 1.70
    assert targets["rl"]["min_bar"] == 1.8  # rear untouched
    fresh = SettingsView()
    assert next(w for s, w in fresh.section5_widgets if s["label"] == "Front band min").value() == 1.70
    assert fl_state() == "in target"


def test_inverted_tyre_band_refuses_save_and_persists_nothing(qapp, restore_config):
    # WP-POLISH-2 addendum: min above max for an axle -> message, no write
    from ui.views.settings_view import SettingsView
    before = {p: open(p, "rb").read() for p in CONFIG_FILES}
    view = SettingsView()
    next(w for s, w in view.section5_widgets if s["label"] == "Rear band min").setValue(2.00)  # max is 1.90
    view.section3_widgets["STRONG_CSR"].setValue(0.5)  # an otherwise valid edit must not slip through
    view._on_save_clicked()
    assert {p: open(p, "rb").read() for p in CONFIG_FILES} == before
    assert "rear" in view.warning_label.text() and not view.warning_label.isHidden()
    assert view.status_label.text() == "Not saved."
