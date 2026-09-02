# Targeted tests for the 2026-08-31 GT3 Paul Ricard investigation fixes:
# (A) wide-table {ChannelBlock} parsing (one section, many columns, one
#     row per timestamp) alongside the pre-existing narrow format (one
#     section per channel); (B) lap_distance unit normalisation (ft vs
#     m, read from the file's own [unit] text, not assumed); (C) the
#     sample-rate guard in prepare_vehicle_state.
#
# REGRESSION-ADJACENT BUT NEW SURFACE: unlike most of this suite, these
# behaviours did not exist before this session, so there is no prior
# production behaviour to pin -- these tests assert the INTENDED
# behaviour directly, per the diagnosis and fix proposal this session
# produced (see thesis_notes.md "GT3 Paul Ricard export: diagnosis and
# fix" and the fix-implementation entry). Synthetic fixtures throughout:
# testing parser/format-handling LOGIC, not making any claim about real
# vehicle behaviour -- same category as test_pure_functions.py's hand-
# derived checks, not the "real data only" rule's target (that rule is
# about not fabricating telemetry to stand in for real car behaviour).
#
# The real GT3_PRC_MLA.txt file (534 MB, 20 Hz, partial session) is
# deliberately NOT used anywhere in this suite -- explicit instruction
# this session: its rate is not native and it must not be used for
# analysis validation. These fixtures are minimal, hand-built, and
# exercise the same code paths without that file's own problems
# (partial session, wrong rate) contaminating what a pass/fail here
# actually means.

import numpy as np
import pytest

from modules.csv_parser import parse_csv, _split_name_unit
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    _normalize_lap_distance_to_metres,
)

# Same logical data (4 samples, 50 Hz) in both layouts, differing only
# in lap_distance's unit -- narrow uses ft (Dubai's own convention),
# wide uses m (the real Paul Ricard export's convention) -- so a single
# pair of tests exercises format x unit together rather than needing a
# 2x2 matrix of fixtures.
NARROW_FIXTURE = """PiToolboxVersionedASCIIDataSet
Version\t2

{OutingInformation}
CarName\tTest
TrackName\tTestTrack

{ChannelBlock}
Time\tecu_speed[kph]
0.00\t100.0
0.02\t101.0
0.04\t102.0
0.06\t103.0

{ChannelBlock}
Time\tlap_number
0.00\t1
0.02\t1
0.04\t1
0.06\t1

{ChannelBlock}
Time\tsclu_yaw_rate[rpm]
0.00\t5.0
0.02\t5.5
0.04\t6.0
0.06\t6.5

{ChannelBlock}
Time\tlog_asteer[deg]
0.00\t10.0
0.02\t10.0
0.04\t10.0
0.06\t10.0

{ChannelBlock}
Time\tlog_acc_y[G]
0.00\t0.5
0.02\t0.5
0.04\t0.5
0.06\t0.5

{ChannelBlock}
Time\tlog_acc_x[G]
0.00\t0.1
0.02\t0.1
0.04\t0.1
0.06\t0.1

{ChannelBlock}
Time\tlap_distance[ft]
0.00\t0.0
0.02\t3.0
0.04\t6.0
0.06\t9.0
"""

WIDE_FIXTURE = """PiToolboxVersionedASCIIDataSet
Version\t2

{OutingInformation}
CarName\tTest
TrackName\tTestTrackWide

{ChannelBlock}
Time\tecu_speed[kph]\tlap_number\tsclu_yaw_rate[rpm]\tlog_asteer[deg]\tlog_acc_y[G]\tlog_acc_x[G]\tlap_distance[m]
0.00\t100.0\t1\t5.0\t10.0\t0.5\t0.1\t0.0
0.02\t101.0\t1\t5.5\t10.0\t0.5\t0.1\t1.0
0.04\t102.0\t1\t6.0\t10.0\t0.5\t0.1\t2.0
0.06\t\t1\t6.5\t10.0\t0.5\t0.1\t3.0
"""


# Locale/malformed-row tolerance, all real observed cases (2026-08-31
# investigation, raw file views of both Dubai and the Paul Ricard
# export): comma AND dot decimal separators (locale, not format --
# Dubai uses commas, Paul Ricard uses dots, in the TIME column too, not
# just values); a bare-timestamp-only row (a real row shape seen in the
# raw file, not malformed data); a "nan" token, which Python's float()
# parses successfully (unlike "-nan(ind)", MSVC's textual NaN, already
# covered by the existing except ValueError path) and so needs its own
# explicit rejection to be treated as a missing cell rather than a
# silently-included NaN sample.
WIDE_FIXTURE_EDGE_CASES = """PiToolboxVersionedASCIIDataSet
Version\t2

{OutingInformation}
CarName\tTest
TrackName\tTestTrackEdgeCases

{ChannelBlock}
Time\tecu_speed[kph]\tlap_number\tsclu_yaw_rate[rpm]\tlog_asteer[deg]\tlog_acc_y[G]\tlog_acc_x[G]\tlap_distance[m]
0.00\t100.0\t1\t5.0\t10.0\t0.5\t0.1\t0.0
0.02\t101.0\t1\t5.5\t10.0\t0.5\t0.1\t1.0
0.04
0,06\t103,5\t1\t6,5\t10,0\t0,5\t0,1\t3,0
0.08\tnan\t1\t7.0\t10.0\t0.5\t0.1\t4.0
"""


def _write(tmp_path, name, content):
    p = tmp_path / name
    p.write_text(content, encoding="latin-1")
    return str(p)


# --- A: format detection and parsing ----------------------------------------

def test_narrow_format_parses(tmp_path):
    path = _write(tmp_path, "narrow.txt", NARROW_FIXTURE)
    result = parse_csv(path)
    speed = result["channels"]["ecu_speed"]
    assert speed["quality"] != "missing"
    np.testing.assert_allclose(speed["data"], [100.0, 101.0, 102.0, 103.0])
    assert result["channels"]["lap_number"]["quality"] != "missing"


def test_wide_format_parses(tmp_path):
    path = _write(tmp_path, "wide.txt", WIDE_FIXTURE)
    result = parse_csv(path)
    speed = result["channels"]["ecu_speed"]
    assert speed["quality"] != "missing"
    # the deliberately-empty cell at t=0.06 must be skipped, not crash
    # or fabricate a value -- 3 samples, not 4.
    np.testing.assert_allclose(speed["data"], [100.0, 101.0, 102.0])
    yaw = result["channels"]["sclu_yaw_rate"]
    np.testing.assert_allclose(yaw["data"], [5.0, 5.5, 6.0, 6.5])
    assert result["channels"]["lap_number"]["quality"] != "missing"


def test_wide_format_channel_not_missing_from_narrow_regression(tmp_path):
    # The original bug, reproduced at fixture scale: before the fix, a
    # wide-table {ChannelBlock} made every whitelisted channel resolve
    # to "missing" because the header split into more than 2 parts.
    path = _write(tmp_path, "wide.txt", WIDE_FIXTURE)
    result = parse_csv(path)
    n_missing = sum(1 for ch in result["channels"].values() if ch["quality"] == "missing")
    n_total = len(result["channels"])
    assert n_missing < n_total, "wide-table parsing regressed to the all-missing failure mode"


def test_split_name_unit():
    assert _split_name_unit("log_asteer[deg]") == ("log_asteer", "deg")
    assert _split_name_unit("lap_number") == ("lap_number", None)
    assert _split_name_unit("lap_distance[m]") == ("lap_distance", "m")


def test_measured_sample_rate_exposed(tmp_path):
    path = _write(tmp_path, "narrow.txt", NARROW_FIXTURE)
    result = parse_csv(path)
    assert result["measured_sample_rate_hz"] == pytest.approx(50.0, rel=1e-6)


def test_wide_format_tolerates_locale_and_malformed_rows(tmp_path):
    path = _write(tmp_path, "edge_cases.txt", WIDE_FIXTURE_EDGE_CASES)
    result = parse_csv(path)  # must not raise

    speed = result["channels"]["ecu_speed"]
    # t=0.04 (bare timestamp) contributes nothing; t=0.08's "nan" token
    # is skipped for ecu_speed specifically -- 3 samples, not 5.
    np.testing.assert_allclose(speed["data"], [100.0, 101.0, 103.5])
    np.testing.assert_allclose(speed["time"], [0.00, 0.02, 0.06])

    # Other channels on the "nan" row (t=0.08) are unaffected -- only
    # ecu_speed's own cell was invalid, not the whole row. The bare-
    # timestamp row (t=0.04) still contributes nothing to any channel.
    yaw = result["channels"]["sclu_yaw_rate"]
    np.testing.assert_allclose(yaw["data"], [5.0, 5.5, 6.5, 7.0])
    np.testing.assert_allclose(yaw["time"], [0.00, 0.02, 0.06, 0.08])

    lap_num = result["channels"]["lap_number"]
    assert len(lap_num["data"]) == 4

    # comma-decimal row (t=0,06) parsed correctly, same as dot-decimal
    # rows -- this is the same value whichever separator wrote it.
    assert 103.5 in speed["data"]


def test_unit_raw_captured_both_formats(tmp_path):
    narrow = parse_csv(_write(tmp_path, "narrow.txt", NARROW_FIXTURE))
    wide = parse_csv(_write(tmp_path, "wide.txt", WIDE_FIXTURE))
    assert narrow["channels"]["lap_distance"]["unit_raw"] == "ft"
    assert wide["channels"]["lap_distance"]["unit_raw"] == "m"


# --- B: lap_distance unit normalisation --------------------------------------

def test_normalize_lap_distance_feet():
    data = np.array([0.0, 10.0, 20.0])
    result = _normalize_lap_distance_to_metres(data, "ft")
    np.testing.assert_allclose(result, data * 0.3048)


def test_normalize_lap_distance_metres_unchanged():
    data = np.array([0.0, 10.0, 20.0])
    result = _normalize_lap_distance_to_metres(data, "m")
    np.testing.assert_allclose(result, data)


def test_normalize_lap_distance_unknown_unit_raises():
    with pytest.raises(ValueError, match="not recognised"):
        _normalize_lap_distance_to_metres(np.array([0.0, 10.0]), "yards")


def test_wide_format_lap_distance_correctly_normalised(tmp_path):
    # End-to-end: the wide fixture's lap_distance is logged in METRES.
    # Before the fix this would have been silently multiplied by 0.3048
    # regardless. After the fix, prepare_vehicle_state's s_m should
    # match the raw values exactly (no conversion applied for [m]).
    path = _write(tmp_path, "wide.txt", WIDE_FIXTURE)
    result = parse_csv(path)
    params = load_parameters()
    state = prepare_vehicle_state(result["channels"], params)
    assert state is not None
    # s_m is guarded/interpolated onto t_ref (ecu_speed's own timeline,
    # 3 valid samples after the empty-cell skip) -- check it lands near
    # the raw metre values, not scaled down by ~3.28x.
    assert state["s_m"] is not None
    assert np.nanmax(state["s_m"]) > 1.5  # would be < 1.0 if wrongly treated as feet


# --- C: sample-rate guard -----------------------------------------------------

def _make_state_channels(rate_hz, n=20):
    dt = 1.0 / rate_hz
    t = np.arange(n) * dt

    def ch(data):
        return {"time": t, "data": np.asarray(data, dtype=float), "quality": "valid", "unit_raw": None}

    return {
        "ecu_speed": ch(np.full(n, 100.0)),
        "sclu_yaw_rate": ch(np.zeros(n)),
        "log_asteer": ch(np.zeros(n)),
        "log_acc_y": ch(np.zeros(n)),
        "log_acc_x": ch(np.zeros(n)),
    }


def test_rate_guard_refuses_mismatched_rate():
    # 100 Hz time-base work package: the guard's own wording changed from
    # an exact-match "Sample rate mismatch" to a range-based "Sample rate
    # too low" (min_sample_rate_hz is now a FLOOR, not an exact target --
    # a file at 50-100 Hz is accepted, only below 50 Hz is refused).
    channels = _make_state_channels(20.0)
    params = load_parameters()
    with pytest.raises(ValueError, match=r"[Ss]ample rate too low"):
        prepare_vehicle_state(channels, params)


def test_rate_guard_message_names_measured_and_expected():
    channels = _make_state_channels(20.0)
    params = load_parameters()
    with pytest.raises(ValueError) as exc_info:
        prepare_vehicle_state(channels, params)
    msg = str(exc_info.value)
    assert "20.0" in msg
    assert "50" in msg


def test_rate_guard_accepts_expected_rate():
    channels = _make_state_channels(50.0)
    params = load_parameters()
    state = prepare_vehicle_state(channels, params)
    assert state is not None
    assert round(state["sample_rate_hz"]) == 50


# --- Fastest-lap candidacy must respect the same reliability bar as
# is_valid_for_analysis (v3 work package, real bug, 2026-09-02) -----------
#
# GT3_PRC_MLA-v3.txt has a genuine 15.8 s pit-exit-crossing-start/finish
# fragment lap. Its OWN lap_time channel disagreed wildly with the
# computed duration (716.5 s vs 15.8 s) -- _verify_laps correctly flagged
# this as a warning -- but the fastest-lap candidate list used to check
# ONLY lap_time_min_s, not that same warning. The fragment won min() for
# being short, every genuine lap then read as "too far above fastest_time"
# by valid_lap_max_ratio and was excluded from is_valid_for_analysis --
# zero valid laps, zero corners detected, and (traced separately, tests/
# test_nan_empty_paths.py) an IndexError several stages downstream when an
# empty fit population reached np.percentile. Fixed by requiring the
# fastest-lap candidate list to pass the same outlap/inlap/warnings bar
# is_valid_for_analysis already does.

def _lap_splitting_fixture_text():
    # lap 1/3/4: three genuine ~24.8s laps -- deliberately ABOVE
    # _merge_trailing_pit_fragment's own pit_fragment_max_duration_s
    # default (20s), which would otherwise merge the session's LAST lap
    # into its predecessor whenever no ecu_B_speedlimit_en channel is
    # present (not present in this minimal fixture, so that Level-1
    # fallback is exactly what would fire on a too-short final lap --
    # a real fixture-authoring trap hit once while writing this test).
    # lap 2: an 11.6s fragment whose own lap_time channel disagrees by
    # nearly 500s, same shape as the real v3 fragment -- short enough to
    # win a naive min() on duration alone, long enough to clear
    # lap_time_min_s (10s default).
    lines = ["PiToolboxVersionedASCIIDataSet", "Version\t2", "",
             "{OutingInformation}", "CarName\tTest", "TrackName\tTestTrack", ""]

    lines += ["{ChannelBlock}", "Time\tlap_number"]
    lap_bounds = [(1, 0.0, 24.8), (2, 25.0, 36.6), (3, 36.8, 61.6), (4, 61.8, 86.6)]
    for lap_n, start, end in lap_bounds:
        t = start
        while t <= end + 1e-9:
            lines.append(f"{t:.1f}\t{lap_n}")
            t = round(t + 0.2, 1)
    lines.append("")

    lines += ["{ChannelBlock}", "Time\tlap_time"]
    for t in [round(0.0 + i, 1) for i in range(25)]:  # lap 1: 0..24, tracks duration
        lines.append(f"{t:.1f}\t{t:.1f}")
    lines.append("25.0\t500.0")  # lap 2 fragment: wildly disagreeing value
    for t in [round(37.0 + i, 1) for i in range(25)]:  # lap 3: tracks (t - 36.8)
        lines.append(f"{t:.1f}\t{t - 36.8:.1f}")
    for t in [round(62.0 + i, 1) for i in range(25)]:  # lap 4: tracks (t - 61.8)
        lines.append(f"{t:.1f}\t{t - 61.8:.1f}")
    lines.append("")

    return "\n".join(lines) + "\n"


def test_fastest_lap_selection_excludes_warned_fragment(tmp_path):
    path = _write(tmp_path, "lap_fragment.txt", _lap_splitting_fixture_text())
    result = parse_csv(path)
    laps = {lap["lap_number"]: lap for lap in result["laps"]}

    assert len(laps) == 4
    assert laps[2]["warnings"], "the fragment lap must carry the lap_time disagreement warning"
    assert laps[2]["is_fastest"] is False, "the warned fragment must never win fastest, however short"
    assert laps[2]["is_valid_for_analysis"] is False

    assert laps[1]["is_fastest"] is True, "the first genuine lap wins the tie deterministically"
    for lap_n in (1, 3, 4):
        assert laps[lap_n]["warnings"] == []
        assert laps[lap_n]["is_valid_for_analysis"] is True, (
            f"lap {lap_n} is a genuine ~19.8s lap and must be valid once the fragment "
            "is correctly excluded from fastest-lap candidacy"
        )
