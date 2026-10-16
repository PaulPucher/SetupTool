# Q-CAL-2 event logger zero entries (config/channels.json event_zero_entries,
# applied in modules/csv_parser.py before the range gate). Real files only:
# Dubai from the repo fixture, the NBR file from the thesis data drive
# (skipped with a stated reason where that drive is absent).

import copy

import numpy as np
import pytest

import modules.csv_parser as csv_parser
from tests.conftest import find_data_file

NBR_FILE = find_data_file("NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")
DAMPERS = ("log_dms_dam_fl", "log_dms_dam_fr", "log_dms_dam_rl", "log_dms_dam_rr")
needs_nbr = pytest.mark.skipif(NBR_FILE is None, reason="NBR thesis data drive not present")


def test_dubai_matches_no_entry(parsed_data):
    """Dubai (track Dubai, logger 147) matches no entry: nothing is applied."""
    assert parsed_data["event_zero_entry"] == {"event": None, "channels": {}}


def _parse_with(monkeypatch, config):
    monkeypatch.setattr(csv_parser, "load_channels_config", lambda: config)
    return csv_parser.parse_csv(NBR_FILE)


@pytest.fixture(scope="module")
def nbr_config():
    return csv_parser.load_channels_config()


@needs_nbr
def test_nbr_entry_applied_and_channels_pass_the_gate(monkeypatch, nbr_config):
    """JST 29: all four damper channels get the NBR_221 zero subtracted, pass
    the range gate, and equal raw - zero_N exactly."""
    applied = _parse_with(monkeypatch, nbr_config)
    raw_cfg = copy.deepcopy(nbr_config)
    raw_cfg.pop("event_zero_entries")
    raw = _parse_with(monkeypatch, raw_cfg)
    entry = nbr_config["event_zero_entries"]["NBR_221"]["channels"]
    assert applied["event_zero_entry"]["event"] == "NBR_221"
    for ch in DAMPERS:
        assert applied["event_zero_entry"]["channels"][ch] == "applied"
        assert raw["channels"][ch]["quality"] == "failed"
        assert applied["channels"][ch]["quality"] == "valid"
        assert np.array_equal(applied["channels"][ch]["data"], raw["channels"][ch]["data"] - entry[ch]["zero_N"])


@needs_nbr
def test_gate_window_guard_blocks_a_mismatched_zero(monkeypatch, nbr_config):
    """A gauge re-zeroed at another event of the same track: with FL's entry
    30 kN off, the corrected mean leaves the gate window, the entry is not
    applied and FL stays failed (static split); the other corners apply."""
    cfg = copy.deepcopy(nbr_config)
    cfg["event_zero_entries"]["NBR_221"]["channels"]["log_dms_dam_fl"]["zero_N"] -= 30000.0
    out = _parse_with(monkeypatch, cfg)
    status = out["event_zero_entry"]["channels"]
    assert status["log_dms_dam_fl"] == "guard_failed"
    assert out["channels"]["log_dms_dam_fl"]["quality"] == "failed"
    for ch in DAMPERS[1:]:
        assert status[ch] == "applied"
