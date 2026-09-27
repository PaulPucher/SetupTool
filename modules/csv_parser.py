# Pi Toolbox ASCII export parser (Cosworth logger). Only channels listed in
# config/channels.json; comma decimals, per-channel rates, lap splitting.
# Two {ChannelBlock} layouts, detected per section from the header width:
#   narrow = one channel per section (Dubai), wide = Time + all channels
#   as columns (Paul Ricard).

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
    # "log_asteer[deg]" -> ("log_asteer", "deg"); no brackets -> unit None.
    # unit_raw = the file's claim, not checked against config -- unit
    # conversions downstream must read it themselves.
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

    # latin-1: exports are single-byte text; maps every byte, never raises.
    # utf-8 + replace turned the degree sign into U+FFFD.
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
                    # narrow
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
                    # wide -- map wanted columns only, rows can be 4000+ wide
                    wanted_cols = {}
                    for col_idx, raw in enumerate(header_parts[1:], start=1):
                        name, unit_raw = _split_name_unit(raw)
                        if name in wanted_channels:
                            wanted_cols[col_idx] = (name, unit_raw)
                    col_times = {idx: [] for idx in wanted_cols}
                    col_values = {idx: [] for idx in wanted_cols}
                    i += 1
                    while i < n and not lines[i].strip().startswith("{"):
                        # short rows are real (bare timestamps) -- missing
                        # cells drop per channel, not per row
                        parts = lines[i].rstrip("\r\n").split("\t")
                        if len(parts) > 1:
                            try:
                                t = float(parts[0].replace(",", "."))
                            except ValueError:
                                i += 1
                                continue
                            if t != t:  # NaN timestamp -> skip row
                                i += 1
                                continue
                            for col_idx in wanted_cols:
                                if col_idx < len(parts) and parts[col_idx] != "":
                                    try:
                                        v = float(parts[col_idx].replace(",", "."))
                                    except ValueError:
                                        # e.g. "-nan(ind)" (MSVC NaN)
                                        continue
                                    if v != v:  # plain "nan" does parse
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

    channels_config = config["channels"]
    quality_gates = config["channel_quality_gates"]
    # Offset corrections for known-corrupt channels (e.g. v3 log_dms_dam_fr).
    # Applied only if this file's raw mean is inside precondition_mean_range
    # -- a healthy channel of the same name stays untouched.
    corrections = config.get("channel_corrections", {})
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

        correction = corrections.get(ch_name)
        if correction is not None and len(data_arr) > 0:
            lo_pre, hi_pre = correction["precondition_mean_range"]
            if lo_pre <= float(np.mean(data_arr)) <= hi_pre:
                data_arr = data_arr + correction["offset"]

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

    # measured from ecu_speed; None = unknown, not "matches expected"
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
    # Session-trailing fragment only (e.g. Dubai's 8 s "lap 6" after the
    # pit-in beacon) -> merged into the previous lap, which is the real inlap.
    if len(laps) < 2:
        return

    last = laps[-1]
    prev = laps[-2]

    limiter_ch = channels.get("ecu_B_speedlimit_en")
    merge = False
    if (limiter_ch is not None and limiter_ch.get("quality") not in ("missing", "failed")
            and limiter_ch.get("time") is not None and len(limiter_ch["time"]) > 0):
        # L3: limiter on at the fragment's first sample
        idx = min(np.searchsorted(limiter_ch["time"], last["start_time"]),
                  len(limiter_ch["data"]) - 1)
        merge = bool(limiter_ch["data"][idx] >= 0.5)
    else:
        # L1: no limiter channel -> too short to be a real lap
        max_dur = config.get("lap_splitting", {}).get("pit_fragment_max_duration_s", 20)
        merge = last["lap_time"] < max_dur

    if not merge:
        return

    prev["end_time"] = last["end_time"]
    prev["lap_time"] = prev["end_time"] - prev["start_time"]
    prev["is_inlap"] = True
    laps.pop()


def _limiter_active_at(limiter_ch, t):
    """Limiter on (>= 0.5) at t. None = channel unusable, not the same as False."""
    if (limiter_ch is None or limiter_ch.get("quality") in ("missing", "failed")
            or limiter_ch.get("time") is None or len(limiter_ch["time"]) == 0):
        return None
    idx = min(np.searchsorted(limiter_ch["time"], t), len(limiter_ch["data"]) - 1)
    return bool(limiter_ch["data"][idx] >= 0.5)


def _classify_out_in_laps_by_limiter(laps, channels):
    # Limiter on at lap start -> outlap, at lap end -> inlap. Only adds
    # flags, never clears the positional ones.
    # Catches pit boxes before start/finish, where the limiter is still on
    # after the lap counter ticks (v3 lap 5,
    # diagnostics/inspect_v3_pit_limiter_lap_census.py), and inlaps with no
    # trailing fragment. No channel -> positional result unchanged.
    limiter_ch = channels.get("ecu_B_speedlimit_en")
    for lap in laps:
        if _limiter_active_at(limiter_ch, lap["start_time"]):
            lap["is_outlap"] = True
        if _limiter_active_at(limiter_ch, lap["end_time"]):
            lap["is_inlap"] = True


def _attach_precise_lap_time(laps, channels, config):
    # Computed lap_time is quantised to lap_number's 0.2 s grid -> ties.
    # lap_time channel has the logger's finer timing: take its max in the
    # lap window, but only if within max_delta of the computed value
    # (outlap / merged inlap don't match the channel's own lap).
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
    _classify_out_in_laps_by_limiter(laps, channels)
    _attach_precise_lap_time(laps, channels, config)
    _verify_laps(laps, channels, config)

    ls = config.get("lap_splitting", {})
    lap_time_min_s = ls.get("lap_time_min_s", 10)
    valid_lap_max_ratio = ls.get("valid_lap_max_ratio", 1.10)

    # Fastest-lap candidates need the same bar as is_valid_for_analysis --
    # a short corrupt fragment winning min() would push every real lap
    # past valid_lap_max_ratio -> zero laps analysed.
    valid = [lap for lap in laps
             if _effective_lap_time(lap) > lap_time_min_s
             and not lap["is_outlap"]
             and not lap["is_inlap"]
             and len(lap["warnings"]) == 0]
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

        # lap_time channel vs computed duration
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

        # lap_distance must actually ramp up (outlap skipped)
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