# Targeted tests for the splitter/diffuser measurement-point persistence
# added in the "8. Splitter/diffuser measurement points" work package
# (thesis_notes.md). Tests core/setup_data_points.py directly rather than
# ui/views/outing_form.py's thin delegating wrappers -- same reason
# tests/conftest.py's own pipeline_result fixture keeps that PyQt6 module
# out of this suite (see its docstring). Persistence-shape coverage only;
# the widget<->_active_inputs binding itself is covered by diagnostics/
# smoke_test_measurement_points_widget.py (Qt required, run manually, same
# precedent as smoke_test_corner_trace_dialog.py). No golden regeneration.

import json

from core.setup_data_points import reshape_points_out, reshape_points_in, POINT_GROUPS


def test_old_outing_load_is_a_no_op():
    """An outing saved before this feature existed has no point keys at
    all -- reshape_in must not invent any, leaving _load_inputs' own
    "skip unknown param" path to show empty boxes (Phase 1/2's own
    verification target, restated here as a pinned regression check).
    """
    old = json.dumps({
        "car": {"splitter_offset": 5.0, "differential_preload": 12.0},
        "front_left": {"toe": 0.0},
    })
    result = json.loads(reshape_points_in(old))
    assert result["car"] == {"splitter_offset": 5.0, "differential_preload": 12.0}
    assert "splitter_points" not in result["car"]
    assert "diffuser_points" not in result["car"]
    for i in range(1, 6):
        assert f"splitter_point_{i}" not in result["car"]
        assert f"diffuser_point_{i}" not in result["car"]


def test_reshape_out_with_some_points_set():
    """Mirrors what _collect_inputs produces once the widgets exist:
    QLineEdit.text() values as strings, blank string for an empty box.
    """
    collected = json.dumps({"car": {
        "splitter_point_1": "12.5", "splitter_point_2": "", "splitter_point_3": "",
        "splitter_point_4": "", "splitter_point_5": "8.0",
    }})
    result = json.loads(reshape_points_out(collected))
    assert result["car"]["splitter_points"] == [12.5, None, None, None, 8.0]
    assert "diffuser_points" not in result["car"], "diffuser group untouched when no diffuser_point_* key was present"
    for i in range(1, 6):
        assert f"splitter_point_{i}" not in result["car"], "flat keys must not survive alongside the array"


def test_reshape_out_with_no_points_set():
    """Every box left blank -- still produces a full 5-null array, not an
    absent key, since the widgets always exist once this feature ships
    (distinct from the old-outing case, where the keys are absent
    entirely because the widgets didn't exist yet when it was saved).
    """
    collected = json.dumps({"car": {f"splitter_point_{i}": "" for i in range(1, 6)}})
    result = json.loads(reshape_points_out(collected))
    assert result["car"]["splitter_points"] == [None, None, None, None, None]


def test_round_trip_out_then_in_recovers_flat_keys():
    collected = json.dumps({"car": {
        "diffuser_point_1": "22", "diffuser_point_2": "", "diffuser_point_3": "18.5",
        "diffuser_point_4": "19", "diffuser_point_5": "",
    }})
    saved = reshape_points_out(collected)
    reloaded = json.loads(reshape_points_in(saved))
    assert reloaded["car"] == {
        "diffuser_point_1": "22.0", "diffuser_point_2": "", "diffuser_point_3": "18.5",
        "diffuser_point_4": "19.0", "diffuser_point_5": "",
    }


def test_round_trip_both_groups_independently():
    collected = json.dumps({"car": {
        "splitter_point_1": "1.1", "splitter_point_2": "", "splitter_point_3": "",
        "splitter_point_4": "", "splitter_point_5": "",
        "diffuser_point_1": "", "diffuser_point_2": "2.2", "diffuser_point_3": "",
        "diffuser_point_4": "", "diffuser_point_5": "",
    }})
    saved = json.loads(reshape_points_out(collected))
    assert saved["car"]["splitter_points"] == [1.1, None, None, None, None]
    assert saved["car"]["diffuser_points"] == [None, 2.2, None, None, None]
    reloaded = json.loads(reshape_points_in(json.dumps(saved)))
    assert reloaded["car"]["splitter_point_1"] == "1.1"
    assert reloaded["car"]["diffuser_point_2"] == "2.2"


def test_splitter_offset_untouched_throughout():
    """The existing SETTING must survive both reshape passes byte-for-
    byte -- it shares the same 'car' dict as the new CHECK points but is
    a completely different key, never popped or renamed.
    """
    collected = json.dumps({"car": {"splitter_offset": 3.3, "splitter_point_1": "1"}})
    saved = reshape_points_out(collected)
    reloaded = json.loads(reshape_points_in(saved))
    assert reloaded["car"]["splitter_offset"] == 3.3


def test_unparseable_value_becomes_null_not_an_exception():
    collected = json.dumps({"car": {"splitter_point_1": "not-a-number"}})
    result = json.loads(reshape_points_out(collected))
    assert result["car"]["splitter_points"][0] is None


def test_array_shorter_than_five_pads_with_blank_on_load():
    """Defensive: a hand-edited or future-truncated array must not raise
    an IndexError -- missing trailing points load as blank, not a crash.
    """
    saved = json.dumps({"car": {"splitter_points": [5.0, 6.0]}})
    reloaded = json.loads(reshape_points_in(saved))
    assert reloaded["car"]["splitter_point_1"] == "5.0"
    assert reloaded["car"]["splitter_point_2"] == "6.0"
    assert reloaded["car"]["splitter_point_3"] == ""
    assert reloaded["car"]["splitter_point_5"] == ""


def test_point_groups_shape_matches_the_pdf_renderer():
    """Contract check against core/pdf_export.py's SPLITTER_POINT_
    POSITIONS/DIFFUSER_POINT_POSITIONS (duplicated there because core/
    cannot import the PyQt6 widget module) -- both must stay 5 points
    per group, or the two hand-kept copies have silently diverged.
    """
    from core.pdf_export import SPLITTER_POINT_POSITIONS, DIFFUSER_POINT_POSITIONS
    counts = {array_key: n for _prefix, array_key, n in POINT_GROUPS}
    assert len(SPLITTER_POINT_POSITIONS) == counts["splitter_points"]
    assert len(DIFFUSER_POINT_POSITIONS) == counts["diffuser_points"]
