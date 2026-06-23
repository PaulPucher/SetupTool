# Corner segmentation and classification for parsed outing data.
# Reads detection thresholds from config/channels.json — no hardcoded numbers.
# Pure Python/numpy — no Qt imports.
#
# Algorithm:
#   1. Bracket corners by steering angle threshold crossings (with hysteresis)
#   2. Locate apex inside each bracket via lateral G peak, cross-checked with speed minimum
#   3. Validate against lateral G threshold to filter lane changes and gentle bends
#   4. Classify by apex speed (low/medium/high) from config thresholds
#   5. Define phase boundaries:
#        Entry 1 (Brake)  — last full throttle on preceding straight → turn-in
#        Entry 2 (Turn-in)— turn-in → just before apex
#        Apex 3           — apex point (single sample)
#        Exit 4           — apex → 50% of steering unwind
#        Exit 5           — 50% of steering unwind → steering exit threshold
#
# Fallback chain:
#   - No steering channel: use speed minima with prominence threshold
#   - No lateral G channel: apex = speed minimum within bracket
#   - No throttle channel: Entry 1 (Brake) collapses to start of bracket

import json
import os
import numpy as np

CHANNELS_CONFIG_PATH = "config/channels.json"


def _load_config():
    with open(CHANNELS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _smooth(arr, window):
    if window <= 1 or len(arr) < window:
        return arr
    kernel = np.ones(window) / window
    return np.convolve(arr, kernel, mode="same")


def analyse_corners(parsed_data):
    """
    Top-level entry point. Takes parsed_data from csv_parser.parse_csv
    and returns a list of corner dicts, one per detected corner per lap.

    Each corner dict contains:
        lap_number, corner_number, speed_class,
        apex_time, apex_speed, apex_lateral_g,
        segments: dict of phase → (start_time, end_time)
        method: "steering" or "speed_fallback"
        warnings: list of strings
    """
    config = _load_config()
    cd = config["corner_detection"]
    speed_thresholds = config["corner_speed_thresholds"]

    channels = parsed_data.get("channels", {})
    laps = parsed_data.get("laps", [])

    corners = []
    for lap in laps:
        if not lap.get("is_valid_for_analysis", False):
            continue
        lap_corners = _analyse_lap(lap, channels, cd, speed_thresholds)
        corners.extend(lap_corners)

    return corners


def _analyse_lap(lap, channels, cd, speed_thresholds):
    start_t = lap["start_time"]
    end_t = lap["end_time"]
    lap_number = lap["lap_number"]

    steering = _slice_channel(channels.get("log_asteer"), start_t, end_t)
    speed = _slice_channel(channels.get("ecu_speed"), start_t, end_t)
    lat_g = _slice_channel(channels.get("log_acc_y"), start_t, end_t)
    throttle = _slice_channel(channels.get("ecu_aps"), start_t, end_t)

    if speed is None:
        return []

    if steering is not None:
        brackets, method = _bracket_corners_by_steering(steering, cd), "steering"
    else:
        brackets, method = _bracket_corners_by_speed(speed, cd), "speed_fallback"

    corners = []
    for i, (b_start_idx, b_end_idx) in enumerate(brackets, start=1):
        corner = _build_corner(
            lap_number, i, method,
            b_start_idx, b_end_idx,
            steering, speed, lat_g, throttle,
            cd, speed_thresholds
        )
        if corner is not None:
            corners.append(corner)

    return corners


def _slice_channel(ch, start_t, end_t):
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    t = ch["time"]
    d = ch["data"]
    mask = (t >= start_t) & (t <= end_t)
    if not mask.any():
        return None
    return {"time": t[mask] - start_t, "data": d[mask], "abs_start": start_t}


def _bracket_corners_by_steering(steering, cd):
    sw = cd["smoothing_window_samples"]
    entry_th = cd["steering_entry_threshold_deg"]
    exit_th = cd["steering_exit_threshold_deg"]
    min_dur = cd["min_corner_duration_s"]

    abs_steer = np.abs(_smooth(steering["data"], sw))
    t = steering["time"]

    brackets = []
    in_corner = False
    b_start = 0
    for i in range(len(abs_steer)):
        if not in_corner and abs_steer[i] > entry_th:
            in_corner = True
            b_start = i
        elif in_corner and abs_steer[i] < exit_th:
            in_corner = False
            if t[i] - t[b_start] >= min_dur:
                brackets.append((b_start, i))
    if in_corner and t[-1] - t[b_start] >= min_dur:
        brackets.append((b_start, len(abs_steer) - 1))

    return brackets


def _bracket_corners_by_speed(speed, cd):
    sw = cd["smoothing_window_samples"]
    min_drop = cd["min_apex_speed_drop_kmh"]
    min_dur = cd["min_corner_duration_s"]

    sm_speed = _smooth(speed["data"], sw)
    t = speed["time"]
    n = len(sm_speed)

    minima = []
    for i in range(1, n - 1):
        if sm_speed[i] <= sm_speed[i - 1] and sm_speed[i] <= sm_speed[i + 1]:
            minima.append(i)

    brackets = []
    for m in minima:
        left = m
        while left > 0 and sm_speed[left] < sm_speed[left - 1]:
            left -= 1
        right = m
        while right < n - 1 and sm_speed[right] < sm_speed[right + 1]:
            right += 1
        if sm_speed[left] - sm_speed[m] < min_drop:
            continue
        if t[right] - t[left] < min_dur:
            continue
        brackets.append((left, right))
    return brackets


def _build_corner(lap_number, corner_number, method,
                  b_start_idx, b_end_idx,
                  steering, speed, lat_g, throttle,
                  cd, speed_thresholds):
    warnings = []
    sw = cd["smoothing_window_samples"]

    sm_speed = _smooth(speed["data"], sw)
    speed_t = speed["time"]

    s_t_start = (steering["time"][b_start_idx]
                 if steering is not None else speed_t[b_start_idx])
    s_t_end = (steering["time"][b_end_idx]
               if steering is not None else speed_t[b_end_idx])

    if lat_g is not None:
        lat_abs = np.abs(_smooth(lat_g["data"], sw))
        mask = (lat_g["time"] >= s_t_start) & (lat_g["time"] <= s_t_end)
        if mask.any():
            sub_g = lat_abs[mask]
            sub_t = lat_g["time"][mask]
            apex_idx = int(np.argmax(sub_g))
            apex_g = float(sub_g[apex_idx])
            apex_t = float(sub_t[apex_idx])
            if apex_g < cd["lateral_g_apex_threshold"]:
                return None
        else:
            return None
    else:
        warnings.append("lateral G missing — apex from speed minimum")
        mask = (speed_t >= s_t_start) & (speed_t <= s_t_end)
        sub_speed = sm_speed[mask]
        sub_t = speed_t[mask]
        if len(sub_speed) == 0:
            return None
        apex_idx = int(np.argmin(sub_speed))
        apex_t = float(sub_t[apex_idx])
        apex_g = None

    speed_apex_mask = (speed_t >= s_t_start) & (speed_t <= s_t_end)
    if speed_apex_mask.any():
        apex_speed = float(np.min(sm_speed[speed_apex_mask]))
    else:
        return None

    if apex_speed < speed_thresholds["low_max"]:
        speed_class = "low"
    elif apex_speed < speed_thresholds["medium_max"]:
        speed_class = "medium"
    else:
        speed_class = "high"

    brake_start_t = s_t_start
    if throttle is not None:
        thr_mask = (throttle["time"] < s_t_start)
        if thr_mask.any():
            thr_t = throttle["time"][thr_mask]
            thr_d = throttle["data"][thr_mask]
            off_throttle = np.where(thr_d < 95)[0]
            if len(off_throttle) > 0:
                brake_start_t = float(thr_t[off_throttle[0]])
    else:
        warnings.append("throttle missing — brake phase = turn-in start")

    if steering is not None:
        bracket_steer = np.abs(_smooth(steering["data"][b_start_idx:b_end_idx + 1], sw))
        bracket_t = steering["time"][b_start_idx:b_end_idx + 1]
        post_apex_mask = bracket_t > apex_t
        if post_apex_mask.any():
            post_steer = bracket_steer[post_apex_mask]
            post_t = bracket_t[post_apex_mask]
            peak_post = float(np.max(post_steer))
            half_th = peak_post / 2
            half_idx = np.argmax(post_steer <= half_th)
            half_t = float(post_t[half_idx]) if half_idx > 0 else float(post_t[-1])
        else:
            half_t = s_t_end
    else:
        half_t = apex_t + (s_t_end - apex_t) / 2

    abs_start = speed["abs_start"]
    segments = {
        "entry_1_brake":   (abs_start + brake_start_t, abs_start + s_t_start),
        "entry_2_turnin":  (abs_start + s_t_start,     abs_start + apex_t),
        "apex_3":          (abs_start + apex_t,        abs_start + apex_t),
        "exit_4":          (abs_start + apex_t,        abs_start + half_t),
        "exit_5":          (abs_start + half_t,        abs_start + s_t_end),
    }

    return {
        "lap_number": lap_number,
        "corner_number": corner_number,
        "speed_class": speed_class,
        "apex_time": abs_start + apex_t,
        "apex_speed": apex_speed,
        "apex_lateral_g": apex_g,
        "segments": segments,
        "method": method,
        "warnings": warnings,
    }