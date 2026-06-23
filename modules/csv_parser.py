# Pi Toolbox ASCII CSV parser for Cosworth datalogger files.
# Reads only channels defined in config/channels.json.
# Handles European decimal notation, variable sample rates,
# lap splitting, and corner detection with speed classification.
# Pure Python/numpy/pandas — no Qt imports.

import numpy as np
import pandas as pd
import json
import os

CHANNELS_CONFIG_PATH = "config/channels.json"


def load_channels_config():
    with open(CHANNELS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def parse_csv(file_path):
    config = load_channels_config()
    wanted_channels = set(config["channels"].keys())
    thresholds = config["corner_speed_thresholds"]

    metadata = {}
    raw_channels = {}

    with open(file_path, "r", encoding="utf-8", errors="replace") as f:
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
                    raw_name = header_parts[1].strip()
                    channel_name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
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
                            "data": np.array(values)
                        }
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                else:
                    i += 1
        else:
            i += 1

    # Build result with quality flags
    channels_config = config["channels"]
    result_channels = {}

    for ch_name, ch_config in channels_config.items():
        if ch_name not in raw_channels:
            result_channels[ch_name] = {
                "label": ch_config["label"],
                "unit": ch_config["unit"],
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
            if valid_ratio < 0.5:
                quality = "failed"
            elif valid_ratio < 0.95:
                quality = "partial"
            else:
                quality = "valid"

        result_channels[ch_name] = {
            "label": ch_config["label"],
            "unit": ch_config["unit"],
            "time": time_arr,
            "data": data_arr,
            "quality": quality
        }

    laps = _split_laps(result_channels)
    corners = _detect_corners(result_channels, laps, thresholds)

    return {
        "metadata": metadata,
        "channels": result_channels,
        "laps": laps,
        "corners": corners
    }


def _split_laps(channels):
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
            "is_fastest": False
        })

    # Mark fastest — only laps longer than 10s to filter out partial laps
    valid = [l for l in laps if l["lap_time"] > 10]
    if valid:
        fastest_lap = min(valid, key=lambda l: l["lap_time"])
        for l in laps:
            l["is_fastest"] = (l is fastest_lap)

    return laps


def _detect_corners(channels, laps, thresholds):
    corners = []
    speed_ch = channels.get("Team_vCar")

    if not speed_ch or speed_ch["quality"] in ("missing", "failed"):
        return corners

    speed_time = speed_ch["time"]
    speed_data = speed_ch["data"]
    low_max = thresholds["low_max"]
    medium_max = thresholds["medium_max"]

    for lap in laps:
        if lap["lap_time"] < 10:
            continue

        mask = (speed_time >= lap["start_time"]) & (speed_time <= lap["end_time"])
        lap_time = speed_time[mask]
        lap_speed = speed_data[mask]

        if len(lap_speed) < 20:
            continue

        smoothed = pd.Series(lap_speed).rolling(
            window=10, center=True, min_periods=1
        ).mean().values

        minima = _find_local_minima(smoothed, min_prominence=15, min_distance=20)

        for corner_num, idx in enumerate(minima, start=1):
            if idx >= len(lap_time):
                continue

            apex_speed = float(smoothed[idx])
            apex_time = float(lap_time[idx])

            if apex_speed < low_max:
                speed_class = "low"
            elif apex_speed < medium_max:
                speed_class = "medium"
            else:
                speed_class = "high"

            entry_idx = _find_preceding_peak(smoothed, idx)
            exit_idx = _find_following_peak(smoothed, idx)

            corners.append({
                "lap": lap["lap_number"],
                "corner": corner_num,
                "apex_time": apex_time,
                "apex_speed": apex_speed,
                "speed_class": speed_class,
                "entry_start_time": float(lap_time[entry_idx]),
                "exit_end_time": float(lap_time[exit_idx]),
            })

    return corners


def _find_local_minima(data, min_prominence=15, min_distance=20):
    n = len(data)
    minima = []
    for i in range(1, n - 1):
        if data[i] <= data[i - 1] and data[i] <= data[i + 1]:
            window_start = max(0, i - min_distance)
            window_end = min(n, i + min_distance)
            local_max = max(data[window_start:window_end])
            if local_max - data[i] >= min_prominence:
                if not minima or i - minima[-1] >= min_distance:
                    minima.append(i)
    return minima


def _find_preceding_peak(data, idx):
    peak = max(0, idx - 1)
    while peak > 0 and data[peak] < data[peak - 1]:
        peak -= 1
    return peak


def _find_following_peak(data, idx):
    n = len(data)
    peak = min(n - 1, idx + 1)
    while peak < n - 1 and data[peak] < data[peak + 1]:
        peak += 1
    return peak


def get_lap_summary(parsed_data):
    return [
        {
            "lap_number": l["lap_number"],
            "lap_time": l["lap_time"],
            "is_fastest": l["is_fastest"],
            "start_time": l["start_time"],
            "end_time": l["end_time"],
        }
        for l in parsed_data.get("laps", [])
        if l["lap_time"] > 10
    ]


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