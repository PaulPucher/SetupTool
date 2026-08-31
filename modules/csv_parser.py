# Pi Toolbox ASCII CSV parser for Cosworth datalogger files.
# Reads only channels defined in config/channels.json.
# Handles European decimal notation, variable sample rates,
# lap splitting, and corner detection with speed classification.
# Two ChannelBlock layouts, both real Pi Toolbox exports (2026-08-31,
# GT3 Paul Ricard investigation): NARROW, one {ChannelBlock} section per
# channel with its own Time/Value pairs (Dubai's own export); WIDE, a
# single {ChannelBlock} section whose header row is Time followed by
# every channel name as a column, one data row per timestamp. Detected
# per file from the header row's own column count -- both may in
# principle appear in the same file (untested, no such export seen),
# handled independently per {ChannelBlock} section either way.
# Pure Python/numpy/pandas -- no Qt imports.

import numpy as np
import pandas as pd
import json
import os
from modules.corner_analysis import analyse_corners
from modules.stability_analysis import _estimate_sample_rate

CHANNELS_CONFIG_PATH = "config/channels.json"


def load_channels_config():
    with open(CHANNELS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _split_name_unit(raw_name):
    # "log_asteer[deg]" -> ("log_asteer", "deg"); "lap_number" (no
    # brackets) -> ("lap_number", None). unit_raw is the FILE's own
    # claim, never validated against config's "unit" label anywhere in
    # this parser -- consumers that convert by unit (e.g. lap_distance's
    # ft/m normalisation) must check unit_raw themselves, not assume it
    # matches config.
    raw_name = raw_name.strip()
    if "[" in raw_name and raw_name.endswith("]"):
        return raw_name[:raw_name.index("[")].strip(), raw_name[raw_name.index("[") + 1:-1].strip()
    return raw_name, None


def parse_csv(file_path):
    config = load_channels_config()
    wanted_channels = set(config["channels"].keys())
    thresholds = config["corner_speed_thresholds"]

    metadata = {}
    raw_channels = {}

    # latin-1 (ISO-8859-1): both real exports seen (Dubai, Paul Ricard)
    # are single-byte Pi Toolbox text, confirmed via `file`. latin-1
    # maps every byte 0x00-0xFF to a character, so it never raises and
    # never needs errors="replace" -- the previous utf-8+replace combination
    # silently turned every degree sign (and any other non-ASCII byte) into
    # U+FFFD, which would have defeated a unit check like lap_distance's
    # ft/m normalisation the day a unit string needed a real symbol.
    with open(file_path, "r", encoding="latin-1") as f:
        lines = f.readlines()

    i = 0
    n = len(lines)

    while i < n:
        line = lines[i].strip()

        if line == "{OutingInformation}":
            i += 1
            while i < n and not lines[i].strip().startswith("{"):
                parts = lines[i].strip().split("\t")
                if len(parts) == 2:
                    metadata[parts[0].strip()] = parts[1].strip()
                i += 1

        elif line == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    # NARROW: this section is one channel.
                    channel_name, unit_raw = _split_name_unit(header_parts[1])
                    i += 1
                    if channel_name in wanted_channels:
                        times, values = [], []
                        while i < n and not lines[i].strip().startswith("{"):
                            parts = lines[i].strip().split("\t")
                            if len(parts) == 2:
                                try:
                                    t = float(parts[0].replace(",", "."))
                                    v = float(parts[1].replace(",", "."))
                                    times.append(t)
                                    values.append(v)
                                except ValueError:
                                    pass
                            i += 1
                        raw_channels[channel_name] = {
                            "time": np.array(times),
                            "data": np.array(values),
                            "unit_raw": unit_raw,
                        }
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                elif len(header_parts) > 2 and header_parts[0].strip() == "Time":
                    # WIDE: this section is every channel, one row per
                    # timestamp. Only build column->channel for the ones
                    # this app actually wants -- a 4000+-column row is
                    # expensive to fully materialise per sample otherwise.
                    wanted_cols = {}
                    for col_idx, raw in enumerate(header_parts[1:], start=1):
                        name, unit_raw = _split_name_unit(raw)
                        if name in wanted_channels:
                            wanted_cols[col_idx] = (name, unit_raw)
                    col_times = {idx: [] for idx in wanted_cols}
                    col_values = {idx: [] for idx in wanted_cols}
                    i += 1
                    while i < n and not lines[i].strip().startswith("{"):
                        # Tolerate short/partial rows (a bare timestamp with
                        # no values at all is a real thing seen in a real
                        # export, not malformed data) -- len(parts) > 1 just
                        # means "at least a timestamp plus something", every
                        # per-column read below already tolerates parts
                        # being shorter than the header via the col_idx <
                        # len(parts) bound, so a row with only some columns
                        # present degrades per-channel, not row-by-row.
                        parts = lines[i].rstrip("\r\n").split("\t")
                        if len(parts) > 1:
                            try:
                                t = float(parts[0].replace(",", "."))
                            except ValueError:
                                i += 1
                                continue
                            if t != t:  # NaN timestamp -- positionally meaningless, skip the row
                                i += 1
                                continue
                            for col_idx in wanted_cols:
                                if col_idx < len(parts) and parts[col_idx] != "":
                                    try:
                                        v = float(parts[col_idx].replace(",", "."))
                                    except ValueError:
                                        # covers non-Python-parseable tokens
                                        # like "-nan(ind)" (MSVC's textual
                                        # NaN) -- a missing cell for this one
                                        # channel/sample, not a row failure.
                                        continue
                                    if v != v:  # NaN idiom -- "nan" DOES parse via float(), unlike "-nan(ind)"
                                        continue
                                    col_times[col_idx].append(t)
                                    col_values[col_idx].append(v)
                        i += 1
                    for col_idx, (name, unit_raw) in wanted_cols.items():
                        raw_channels[name] = {
                            "time": np.array(col_times[col_idx]),
                            "data": np.array(col_values[col_idx]),
                            "unit_raw": unit_raw,
                        }
                else:
                    i += 1
        else:
            i += 1

    # Build result with quality flags
    channels_config = config["channels"]
    quality_gates = config["channel_quality_gates"]
    result_channels = {}

    for ch_name, ch_config in channels_config.items():
        if ch_name not in raw_channels:
            result_channels[ch_name] = {
                "label": ch_config["label"],
                "unit": ch_config["unit"],
                "unit_raw": None,
                "time": None,
                "data": None,
                "quality": "missing"
            }
            continue

        raw = raw_channels[ch_name]
        time_arr = raw["time"]
        data_arr = raw["data"]

        if len(data_arr) == 0:
            quality = "failed"
        else:
            lo, hi = ch_config["range"]
            valid_mask = (data_arr >= lo) & (data_arr <= hi)
            valid_ratio = valid_mask.sum() / len(valid_mask)
            if valid_ratio < quality_gates["failed_below"]:
                quality = "failed"
            elif valid_ratio < quality_gates["partial_below"]:
                quality = "partial"
            else:
                quality = "valid"

        result_channels[ch_name] = {
            "label": ch_config["label"],
            "unit": ch_config["unit"],
            "unit_raw": raw.get("unit_raw"),
            "time": time_arr,
            "data": data_arr,
            "quality": quality
        }

    laps = _split_laps(result_channels, config)

    # Measured, not assumed -- modules.stability_analysis.prepare_vehicle_
    # state's rate guard (config stability_estimation.expected_sample_
    # rate_hz) reads this rather than re-deriving it, so a file whose
    # primary time reference (ecu_speed) is missing/unusable is reported
    # as "rate unknown" (None) here, not silently treated as matching.
    measured_rate = None
    speed_ch = result_channels.get("ecu_speed")
    if speed_ch is not None and speed_ch["time"] is not None and len(speed_ch["time"]) > 1:
        try:
            measured_rate = _estimate_sample_rate(speed_ch["time"])
        except ValueError:
            measured_rate = None

    result = {
        "metadata": metadata,
        "channels": result_channels,
        "laps": laps,
        "corners": [],
        "measured_sample_rate_hz": measured_rate,
    }

    result["corners"] = analyse_corners(result)

    return result


def _merge_trailing_pit_fragment(laps, channels, config):
    # SCOPE: handles the SESSION-TRAILING fragment only -- the final
    # lap_number segment after the pit-in beacon, e.g. Dubai's 8s "lap 6"
    # (log_beacon_pitin at t=1121.65s, 0.27s before the lap_number 5->6
    # transition). The true inlap -- pit committed, decelerating to pit
    # speed under the limiter -- is the PRECEDING lap: on Dubai the
    # pit-speed limiter (ecu_B_speedlimit_en) engages at t=1108.78s, 13.14s
    # before that transition, entirely inside what was "lap 5".
    # Multi-stint race files have no such trailing fragment (the stop lap
    # runs line-to-line through the pit box as one lap_number) -- they will
    # need stint-aware in/out/stop-lap classification via MID-lap limiter
    # engagement instead. That is deferred until multi-stint data arrives
    # (see PLAN.md IDEAS); the limiter channel whitelisted here is the
    # enabler for that later work, not a solution to it.
    if len(laps) < 2:
        return

    last = laps[-1]
    prev = laps[-2]

    limiter_ch = channels.get("ecu_B_speedlimit_en")
    merge = False
    if (limiter_ch is not None and limiter_ch.get("quality") not in ("missing", "failed")
            and limiter_ch.get("time") is not None and len(limiter_ch["time"]) > 0):
        # Level 3: limiter already engaged at the fragment's first sample.
        idx = min(np.searchsorted(limiter_ch["time"], last["start_time"]),
                  len(limiter_ch["data"]) - 1)
        merge = bool(limiter_ch["data"][idx] >= 0.5)
    else:
        # Level 1 fallback: no limiter channel -- fragment shorter than
        # any real lap could plausibly be.
        max_dur = config.get("lap_splitting", {}).get("pit_fragment_max_duration_s", 20)
        merge = last["lap_time"] < max_dur

    if not merge:
        return

    prev["end_time"] = last["end_time"]
    prev["lap_time"] = prev["end_time"] - prev["start_time"]
    prev["is_inlap"] = True
    laps.pop()


def _attach_precise_lap_time(laps, channels, config):
    # lap_time (computed) is bounded by the lap_number channel's own
    # sample interval (0.2 s on Dubai) -- boundaries land on that grid, so
    # two genuinely different lap durations can quantise to the identical
    # float and only "tie-break" by list order. The file's own lap_time
    # channel is logged independently, at its own (finer) sample interval,
    # and carries the logger's real sub-tenth timing. We take its max
    # value inside the lap's window (same pattern _verify_laps already
    # uses) as lap_time_precise, gated against the computed duration so a
    # boundary that doesn't correspond to this channel's own lap concept
    # (the outlap -- channel hasn't started counting; the merged inlap --
    # channel reflects the pre-merge, un-merged lap) falls back to
    # computed rather than silently substituting an unrelated number. Max
    # possible undercount from this method is bounded by one lap_time
    # channel sample interval, which is well inside the gate.
    max_delta = config.get("lap_splitting", {}).get("lap_time_precise_max_delta_s", 1.0)
    lt_ch = channels.get("lap_time")
    for lap in laps:
        lap["lap_time_precise"] = None
        if lt_ch is None or lt_ch.get("quality") in ("missing", "failed") or lt_ch.get("time") is None:
            continue
        t, v = lt_ch["time"], lt_ch["data"]
        mask = (t >= lap["start_time"]) & (t <= lap["end_time"])
        if not mask.any():
            continue
        candidate = float(v[mask].max())
        if abs(candidate - lap["lap_time"]) <= max_delta:
            lap["lap_time_precise"] = candidate


def _effective_lap_time(lap):
    return lap["lap_time_precise"] if lap.get("lap_time_precise") is not None else lap["lap_time"]


def _split_laps(channels, config=None):
    config = config or {}
    laps = []
    lap_ch = channels.get("lap_number")

    if not lap_ch or lap_ch["quality"] == "missing":
        return laps

    time_arr = lap_ch["time"]
    data_arr = lap_ch["data"]

    if len(time_arr) == 0:
        return laps

    lap_nums = data_arr.astype(int)
    unique_laps = sorted(set(lap_nums))

    for lap_n in unique_laps:
        mask = lap_nums == lap_n
        lap_times = time_arr[mask]
        if len(lap_times) == 0:
            continue
        start_t = float(lap_times[0])
        end_t = float(lap_times[-1])
        duration = end_t - start_t
        laps.append({
            "lap_number": int(lap_n),
            "start_time": start_t,
            "end_time": end_t,
            "lap_time": duration,
            "is_fastest": False,
            "is_valid_for_analysis": False,
            "is_outlap": int(lap_n) == 0,
            "is_inlap": False,
            "warnings": []
        })

    _merge_trailing_pit_fragment(laps, channels, config)
    _attach_precise_lap_time(laps, channels, config)
    _verify_laps(laps, channels, config)

    ls = config.get("lap_splitting", {})
    lap_time_min_s = ls.get("lap_time_min_s", 10)
    valid_lap_max_ratio = ls.get("valid_lap_max_ratio", 1.10)

    valid = [lap for lap in laps if _effective_lap_time(lap) > lap_time_min_s]
    if valid:
        fastest_lap = min(valid, key=_effective_lap_time)
        fastest_time = _effective_lap_time(fastest_lap)
        for lap in laps:
            lap["is_fastest"] = (lap is fastest_lap)
            lap["is_valid_for_analysis"] = (
                not lap["is_outlap"]
                and not lap["is_inlap"]
                and _effective_lap_time(lap) <= fastest_time * valid_lap_max_ratio
                and _effective_lap_time(lap) > lap_time_min_s
                and len(lap["warnings"]) == 0
            )

    return laps


def _verify_laps(laps, channels, config=None):
    config = config or {}
    ls = config.get("lap_splitting", {})
    time_disagreement_max_s = ls.get("lap_time_disagreement_max_s", 2.0)
    distance_min_travelled_m = ls.get("lap_distance_min_travelled_m", 1000)
    distance_check_min_duration_s = ls.get("lap_distance_check_min_duration_s", 30)

    file_lap_time = channels.get("lap_time")
    lap_distance = channels.get("lap_distance")

    for lap in laps:
        start_t = lap["start_time"]
        end_t = lap["end_time"]
        duration = lap["lap_time"]

        # Check 1 -- file's own lap_time channel agrees with our computed duration
        if file_lap_time and file_lap_time["quality"] not in ("missing", "failed"):
            t = file_lap_time["time"]
            d = file_lap_time["data"]
            mask = (t >= start_t) & (t <= end_t)
            if mask.any():
                file_max = float(d[mask].max())
                if file_max > 5 and abs(file_max - duration) > time_disagreement_max_s:
                    lap["warnings"].append(
                        f"lap_time channel ({file_max:.1f}s) disagrees with "
                        f"computed duration ({duration:.1f}s)"
                    )

        # Check 2 -- lap_distance should ramp up within the lap (skip outlap)
        if lap["lap_number"] != 0 and lap_distance and lap_distance["quality"] not in ("missing", "failed"):
            t = lap_distance["time"]
            d = lap_distance["data"]
            mask = (t >= start_t) & (t <= end_t)
            if mask.any():
                lap_d = d[mask]
                d_start = float(lap_d[0])
                d_peak = float(lap_d.max())
                d_traveled = d_peak - d_start
                if d_traveled < distance_min_travelled_m and duration > distance_check_min_duration_s:
                    lap["warnings"].append(
                        f"lap_distance only rose by {d_traveled:.0f} units "
                        f"despite {duration:.1f}s duration"
                    )

def get_available_channels(parsed_data):
    return [
        {
            "name": name,
            "label": ch["label"],
            "unit": ch["unit"],
            "quality": ch["quality"]
        }
        for name, ch in parsed_data.get("channels", {}).items()
        if ch["quality"] not in ("missing", "failed")
    ]