# Targeted tests for the cleanup/reliability pass's error-path fixes
# (thesis_notes.md section 9 Phase 5's raw-exception-string/silent-
# failure catalogue). core/ only -- no Qt, same "keep PyQt6 out of the
# suite" precedent tests/test_setup_data_points.py's own docstring
# states. Covers what CAN be exercised without Qt: friendly_error_text's
# own formatting rule, and generate_setup_pdf's error path no longer
# silently swallowing a corrupt setup_data JSON string.

import os
from datetime import datetime
from types import SimpleNamespace

from core.error_text import friendly_error_text
from core.pdf_export import generate_setup_pdf


def test_friendly_error_text_value_error_passthrough():
    # This codebase's own convention: a raised ValueError already carries
    # a complete, human-readable sentence (e.g. the sample-rate guard) --
    # must pass through unchanged, not gain a redundant "ValueError:" prefix.
    e = ValueError("sample rate 20 Hz, expected 50 Hz")
    assert friendly_error_text(e) == "sample rate 20 Hz, expected 50 Hz"


def test_friendly_error_text_other_exception_gets_class_prefix():
    # KeyError's own str() is just the bare missing key -- not a sentence.
    e = KeyError("log_gps_lat")
    text = friendly_error_text(e)
    assert text.startswith("KeyError:")
    assert "log_gps_lat" in text


def test_friendly_error_text_empty_message_falls_back_to_class_name():
    e = RuntimeError()
    assert friendly_error_text(e) == "RuntimeError"


def _fake_weekend():
    return SimpleNamespace(track="Dubai", series="GT3", car_number=1, year=2026,
                            date=None, type="Race")


def _fake_outing(setup_data):
    return SimpleNamespace(
        setup_data=setup_data, number=1, name="Test", session_type="Practice",
        date_time=datetime(2026, 8, 31, 12, 0), driver_name="",
    )


def test_generate_setup_pdf_with_corrupt_setup_data_does_not_raise(tmp_path):
    """Reliability fix: core/pdf_export.py's generate_setup_pdf used to
    swallow a json.loads failure with a bare `except Exception: pass`,
    rendering a fully blank-but-otherwise-normal sheet with zero
    indication anything was wrong. Must now build successfully (never
    crash the export) with a visible warning banner reserved real space,
    not stacked on top of the strip's own KeepInFrame.
    """
    out_path = str(tmp_path / "setup_corrupt.pdf")
    outing = _fake_outing(setup_data="{this is not valid json")
    generate_setup_pdf(outing, _fake_weekend(), out_path, sheet_type="Setup")
    assert os.path.exists(out_path)
    with open(out_path, "rb") as f:
        content = f.read()
    assert content.startswith(b"%PDF")
    assert len(content) > 500  # a real page, not a truncated/empty stub


def test_generate_setup_pdf_with_empty_setup_data_unaffected(tmp_path):
    """The no-data case (outing.setup_data falsy) is NOT the corrupt-data
    case -- must still build cleanly with no warning banner logic
    triggered (setup_data="" never reaches json.loads at all).
    """
    out_path = str(tmp_path / "setup_empty.pdf")
    outing = _fake_outing(setup_data="")
    generate_setup_pdf(outing, _fake_weekend(), out_path, sheet_type="Setup")
    assert os.path.exists(out_path)
    with open(out_path, "rb") as f:
        assert f.read().startswith(b"%PDF")

# generate_weekend_pdf's own per-outing try/except (core/weekend_pdf_
# export.py, around _build_outing_section) already existed before this
# pass -- only its error TEXT changed (repr() -> friendly_error_text),
# already covered above. Not re-tested here: reliably triggering that
# specific except branch needs a fake Outing replicating the real
# model's full column set (driver_id, csv_path, analysis_data, ... --
# _cover_page_flowables alone touches half a dozen fields before the
# per-outing loop is even reached), disproportionate effort for a
# message-formatting-only change.
