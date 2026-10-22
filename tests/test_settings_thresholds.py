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
# real summaries of the default path (Dubai goldens retired 2026-10-20)
GOLDEN_SUMMARIES = "tests/golden/default_nbr_jst29.json"


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
    # classifier reads the new value (real NBR JST 29 summaries)
    from ui.views.settings_view import SettingsView
    summaries = _real_summaries()
    before = _severities(summaries)
    with open("config/parameters.json", encoding="utf-8") as f:
        derived = json.load(f)["classification"]["STRONG_CSR"]["value"]

    view = SettingsView()
    view.section3_widgets["STRONG_CSR"].setValue(0.5)  # nearly every rear reading becomes "strong"
    view._on_save_clicked()

    with open("config/parameters.json", encoding="utf-8") as f:
        entry = json.load(f)["classification"]["STRONG_CSR"]
    assert entry["value"] == 0.5 and entry["derived_default"] == derived
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
    with open("config/parameters.json", encoding="utf-8") as f:
        derived = json.load(f)["classification"]["MODERATE_CSF"]["value"]
    view = SettingsView()
    view.section3_widgets["MODERATE_CSF"].setValue(0.3)
    view._on_save_clicked()
    view._restore_threshold("MODERATE_CSF")
    view._on_save_clicked()

    with open("config/parameters.json", encoding="utf-8") as f:
        entry = json.load(f)["classification"]["MODERATE_CSF"]
    assert entry["value"] == derived and "derived_default" not in entry
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
    # 2026-10-06 (author app-check ruling): no source names in product text
    assert note.startswith("Yaw stability below this = open-loop unstable.")
    assert "Werner" not in note
    assert (STAB_KINEMATIC_ERA_NOTE in note) == differs
    # live config since 2026-10-05: stab_neg_thresh anchored for ekf_auto_pacejka
    # (Werner sign criterion), so the kinematic-era note is gone
    assert not differs


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
