# Fz-integration Phase 4 (2026-09-03): pit-limiter-based out/in-lap
# classification (modules.csv_parser._classify_out_in_laps_by_limiter).
# Synthetic fixtures only -- minimal Pi Toolbox narrow-format text files,
# same construction convention as tests/test_csv_parser_formats.py's own
# _lap_splitting_fixture_text (that file's own real-bug precedent this
# phase extends: the fastest-lap-candidacy fix already found and fixed a
# real v3 fragment-lap bug the same way -- built a minimal synthetic
# fixture reproducing the real file's shape, not the real file itself).
#
# MOTIVATION, real bug found via diagnostics/inspect_v3_pit_limiter_lap_
# census.py (thesis_notes.md "Fz-integration Phase 4..."): GT3_PRC_MLA-
# v3.txt's own pit lane exit sits BEFORE the start/finish line -- the
# pit limiter is still engaged after the lap-counter has already
# incremented past the outlap's own lap_number 0 boundary (v3's own lap
# numbers do not even include 0 -- this file is a mid-session fragment,
# lap_number 4-9). The session's LAST lap similarly has no separate
# short trailing fragment (unlike Dubai) -- the pit-committed tail runs
# inside the SAME lap_number as the rest of that lap. The PRE-Phase-4
# code (purely positional, is_outlap = lap_number==0) misses BOTH: v3's
# real last lap (lap 9) was is_valid_for_analysis=True despite its final
# ~22s running under the pit limiter, not at racing pace.

from modules.csv_parser import parse_csv


def _write(tmp_path, name, content):
    p = tmp_path / name
    p.write_text(content, encoding="latin-1")
    return str(p)


def _lap_number_block(lap_bounds):
    lines = ["{ChannelBlock}", "Time\tlap_number"]
    for lap_n, start, end in lap_bounds:
        t = start
        while t <= end + 1e-9:
            lines.append(f"{t:.1f}\t{lap_n}")
            t = round(t + 0.2, 1)
    lines.append("")
    return lines


def _limiter_block(active_windows, t_end, dt=0.2):
    lines = ["{ChannelBlock}", "Time\tecu_B_speedlimit_en"]
    t = 0.0
    while t <= t_end + 1e-9:
        active = any(lo <= t <= hi for lo, hi in active_windows)
        lines.append(f"{t:.1f}\t{1 if active else 0}")
        t = round(t + dt, 1)
    lines.append("")
    return lines


def _box_before_line_fixture_text():
    # Three ~24.8s laps, numbered 1/2/3 (deliberately NOT starting at 0 --
    # mirrors v3's own mid-session-fragment numbering, and proves the
    # classification is not secretly still keying off lap_number==0).
    # Limiter active [0.0, 3.0]s (spans INTO lap 1's own start at t=0 --
    # the box-before-line signature: the pit-exit zone straddles the
    # lap-counter boundary) and [72.0, 75.0]s (spans lap 3's own end at
    # t=75.0 -- the pit-committed tail, no separate trailing fragment).
    lines = ["PiToolboxVersionedASCIIDataSet", "Version\t2", "",
              "{OutingInformation}", "CarName\tTest", "TrackName\tTestTrack", ""]
    lap_bounds = [(1, 0.0, 24.8), (2, 25.0, 49.8), (3, 50.0, 75.0)]
    lines += _lap_number_block(lap_bounds)
    lines += _limiter_block([(0.0, 3.0), (72.0, 75.0)], t_end=75.0)
    return "\n".join(lines) + "\n"


def test_box_before_line_outlap_continuation_flagged(tmp_path):
    path = _write(tmp_path, "box_before_line.txt", _box_before_line_fixture_text())
    result = parse_csv(path)
    laps = {l["lap_number"]: l for l in result["laps"]}

    assert len(laps) == 3
    # Lap 1 is NOT lap_number 0, but the limiter is active at its own
    # start (t=0.0) -- must still be flagged is_outlap via the channel,
    # not the position.
    assert laps[1]["is_outlap"] is True, "limiter active at lap 1's own start must flag it as an outlap continuation"
    assert laps[1]["is_valid_for_analysis"] is False

    assert laps[2]["is_outlap"] is False
    assert laps[2]["is_inlap"] is False
    assert laps[2]["is_valid_for_analysis"] is True

    # Lap 3 is a single lap_number all the way through (no separate short
    # trailing fragment for _merge_trailing_pit_fragment to catch) -- the
    # limiter active at its own END must flag it is_inlap via the new
    # mid/end-of-lap classification, not silently pass as a valid lap.
    assert laps[3]["is_inlap"] is True, "limiter active at lap 3's own end must flag it as an inlap, no separate fragment needed"
    assert laps[3]["is_valid_for_analysis"] is False


def test_channel_absent_falls_back_to_positional_logic_unchanged(tmp_path):
    # Same lap boundaries, NO ecu_B_speedlimit_en channel at all -- every
    # lap must classify exactly as the pre-Phase-4 positional-only logic
    # would (is_outlap only true for lap_number==0, is_inlap never set by
    # this mechanism at all since none of these laps is lap_number 0 and
    # there is no limiter channel to add anything).
    lines = ["PiToolboxVersionedASCIIDataSet", "Version\t2", "",
              "{OutingInformation}", "CarName\tTest", "TrackName\tTestTrack", ""]
    lap_bounds = [(1, 0.0, 24.8), (2, 25.0, 49.8), (3, 50.0, 75.0)]
    lines += _lap_number_block(lap_bounds)
    text = "\n".join(lines) + "\n"

    path = _write(tmp_path, "no_limiter.txt", text)
    result = parse_csv(path)
    laps = {l["lap_number"]: l for l in result["laps"]}

    for n in (1, 2, 3):
        assert laps[n]["is_outlap"] is False, f"lap {n}: no limiter channel, lap_number != 0 -- must stay False"
        assert laps[n]["is_inlap"] is False, f"lap {n}: no limiter channel -- must stay False"
        assert laps[n]["is_valid_for_analysis"] is True


def test_lap_number_zero_still_flagged_outlap_even_when_limiter_disagrees(tmp_path):
    # Dubai byte-identical guard, as a synthetic case: lap_number==0 must
    # STILL be flagged is_outlap even if the limiter channel (present,
    # usable) happens to NOT be active exactly at that lap's own start --
    # additive-OR composition, the channel can only ADD flags, never
    # remove the positional one. Mirrors the real Dubai shape: its own
    # limiter engages ~11.5s AFTER lap 0's start, not AT it.
    lines = ["PiToolboxVersionedASCIIDataSet", "Version\t2", "",
              "{OutingInformation}", "CarName\tTest", "TrackName\tTestTrack", ""]
    lap_bounds = [(0, 0.0, 24.8), (1, 25.0, 49.8)]
    lines += _lap_number_block(lap_bounds)
    # Limiter active only well inside lap 0 (not at t=0.0 itself) and
    # never active anywhere near lap 1.
    lines += _limiter_block([(10.0, 13.0)], t_end=49.8)
    text = "\n".join(lines) + "\n"

    path = _write(tmp_path, "dubai_shape.txt", text)
    result = parse_csv(path)
    laps = {l["lap_number"]: l for l in result["laps"]}

    assert laps[0]["is_outlap"] is True, "lap_number==0 must stay an outlap regardless of the limiter's own timing"
    assert laps[1]["is_outlap"] is False
    assert laps[1]["is_inlap"] is False
