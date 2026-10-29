# Q-CAL-2 event logger zero entries (config/channels.json event_zero_entries,
# applied in modules/csv_parser.py before the range gate). Real files only:
# Dubai from the repo fixture, the NBR file from the thesis data drive
# (skipped with a stated reason where that drive is absent).

import copy
import os

import numpy as np
import pytest

import modules.csv_parser as csv_parser
from tests.conftest import find_data_file

NBR_FILE = find_data_file("NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")
LC32_FILE = find_data_file("paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt")
# evaluation package (outside the data roots): Paul Ricard outing 43, logger
# software 24996, one of the PRC_221_24996 source sessions
PRC2_FILE = "H:/dani_try 100/PRC2.txt"
needs_prc2 = pytest.mark.skipif(not os.path.exists(PRC2_FILE), reason="evaluation package drive not present")
DAMPERS = ("log_dms_dam_fl", "log_dms_dam_fr", "log_dms_dam_rl", "log_dms_dam_rr")
needs_nbr = pytest.mark.skipif(NBR_FILE is None, reason="NBR thesis data drive not present")


def test_dubai_matches_no_entry(parsed_data):
    """Dubai (track Dubai, logger 147) matches no entry: nothing is applied."""
    assert parsed_data["event_zero_entry"] == {"event": None, "channels": {}, "source_session": False}


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


def _header(path):
    # the match reads only the {OutingInformation} block
    meta = {}
    with open(path, encoding="latin-1") as f:
        for line in f:
            line = line.strip()
            if line == "{ChannelBlock}":
                break
            parts = line.split("	")
            if len(parts) == 2:
                meta[parts[0].strip()] = parts[1].strip()
    return meta


@needs_prc2
def test_prc_24996_entry_applied_on_its_source_session(nbr_config):
    """PRC outing 43 (Metafile 24996): PRC_221_24996 applies on all four
    corners, the channels pass the gate, and the file is flagged as a source
    session of the entry."""
    out = csv_parser.parse_csv(PRC2_FILE)
    zero = out["event_zero_entry"]
    assert zero["event"] == "PRC_221_24996"
    assert zero["source_session"] is True
    for ch in DAMPERS:
        assert zero["channels"][ch] == "applied"
        assert out["channels"][ch]["quality"] == "valid"


@pytest.mark.skipif(LC32_FILE is None, reason="Le Castellet 32 not present")
def test_prc_entry_not_matched_on_25016_export(nbr_config):
    """Le Castellet 32 (same track and serial, Metafile 25016): no entry
    matches -- the software key keeps the 24996 zero off the healthy export,
    whose gate-window guard alone would have let it through."""
    meta = _header(LC32_FILE)
    assert "Metafile Version: 25016" in meta["SystemDetails"]
    name, zeros = csv_parser._match_event_zero_entry(nbr_config["event_zero_entries"], meta)
    assert (name, zeros) == (None, {})


@needs_nbr
def test_nbr_entry_key_unchanged(nbr_config):
    """NBR 29 (Metafile 24996): still NBR_221, which declares no software
    key; the PRC entry's software key does not reach another track."""
    assert "logger_software" not in nbr_config["event_zero_entries"]["NBR_221"]
    name, zeros = csv_parser._match_event_zero_entry(nbr_config["event_zero_entries"], _header(NBR_FILE))
    assert name == "NBR_221"
    assert zeros == {ch: v["zero_N"] for ch, v in nbr_config["event_zero_entries"]["NBR_221"]["channels"].items()}
