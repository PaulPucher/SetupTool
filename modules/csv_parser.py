# Pi Toolbox ASCII CSV parser for Cosworth datalogger files.
# Reads only channels defined in config/channels.json.
# Handles European decimal notation, variable sample rates,
# lap splitting, and corner detection with speed classification.
# Pure Python/numpy/pandas — no Qt imports.

import numpy as np
import pandas as pd
import json
import os
from modules.corner_analysis import analyse_corners

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

    result = {
        "metadata": metadata,
        "channels": result_channels,
        "laps": laps,
        "corners": []
    }

    result["corners"] = analyse_corners(result)

    return result


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
            "is_fastest": False,
            "is_valid_for_analysis": False,
            "warnings": []
        })

    _verify_laps(laps, channels)

    valid = [l for l in laps if l["lap_time"] > 10]
    if valid:
        fastest_lap = min(valid, key=lambda l: l["lap_time"])
        fastest_time = fastest_lap["lap_time"]
        for l in laps:
            l["is_fastest"] = (l is fastest_lap)
            l["is_valid_for_analysis"] = (
                l["lap_number"] != 0
                and l["lap_time"] <= fastest_time * 1.10
                and l["lap_time"] > 10
                and len(l["warnings"]) == 0
            )

    return laps


def _verify_laps(laps, channels):
    file_lap_time = channels.get("lap_time")
    lap_distance = channels.get("lap_distance")

    for lap in laps:
        start_t = lap["start_time"]
        end_t = lap["end_time"]
        duration = lap["lap_time"]

        # Check 1 — file's own lap_time channel agrees with our computed duration
        if file_lap_time and file_lap_time["quality"] not in ("missing", "failed"):
            t = file_lap_time["time"]
            d = file_lap_time["data"]
            mask = (t >= start_t) & (t <= end_t)
            if mask.any():
                file_max = float(d[mask].max())
                if file_max > 5 and abs(file_max - duration) > 2.0:
                    lap["warnings"].append(
                        f"lap_time channel ({file_max:.1f}s) disagrees with "
                        f"computed duration ({duration:.1f}s)"
                    )

                # Check 2 — lap_distance should ramp up within the lap (skip outlap)
        if lap["lap_number"] != 0 and lap_distance and lap_distance["quality"] not in ("missing", "failed"):
            t = lap_distance["time"]
            d = lap_distance["data"]
            mask = (t >= start_t) & (t <= end_t)
            if mask.any():
                lap_d = d[mask]
                d_start = float(lap_d[0])
                d_peak = float(lap_d.max())
                d_traveled = d_peak - d_start
                if d_traveled < 1000 and duration > 30:
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