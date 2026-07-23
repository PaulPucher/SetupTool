# Corner segmentation and classification for parsed outing data.
# Reads detection thresholds from config/channels.json — no hardcoded numbers.
# Pure Python/numpy — no Qt imports.
#
# Algorithm:
#   1. Bracket corners by steering angle OR lateral G threshold crossings
#      (with hysteresis; exit requires both signals below their thresholds)
#   2. Locate apex inside each bracket via lateral G peak, cross-checked with speed minimum
#   3. Validate against lateral G threshold to filter lane changes and gentle bends
#   4. Classify by apex speed (low/medium/high) from config thresholds
#   5. Define phase boundaries:
#        Entry 1 (Brake)  — last full throttle on preceding straight → turn-in
#        Entry 2 (Turn-in)— turn-in → just before apex
#        Apex 3           — apex point (single sample)
#        Exit 4           — apex → 50% of steering unwind
#        Exit 5           — 50% of steering unwind → steering exit threshold
#   6. Merge same-direction adjacent brackets separated by a short time gap
#      (stabilises corners with a momentary mid-corner steering dip; opposite-
#      direction pairs, i.e. chicanes, are left as separate brackets)
#   7. Cross-lap identity (assign_stable_corner_ids): link corners across laps
#      by bracket-span overlap fraction along lap_distance; connected
#      components are candidate clusters, with same-lap exclusivity enforced
#      by a deterministic seeded split where one lap's bracket straddles what
#      other laps detect as two distinct corners
#
# Fallback chain:
#   - No steering channel: use speed minima with prominence threshold
#   - No lateral G channel: apex = speed minimum within bracket
#   - No throttle channel: Entry 1 (Brake) collapses to start of bracket

import json
import numpy as np
from modules.geo import compute_gps_origin, project_latlon_to_xy

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

    assign_stable_corner_ids(corners, channels)

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
        brackets, method = _bracket_corners_by_steering(steering, cd, lat_g), "steering"
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


def _bracket_corners_by_steering(steering, cd, lat_g=None):
    sw = cd["smoothing_window_samples"]
    entry_th = cd["steering_entry_threshold_deg"]
    exit_th = cd["steering_exit_threshold_deg"]
    min_dur = cd["min_corner_duration_s"]
    ay_entry_th = cd["ay_entry_threshold_g"]
    ay_exit_th = cd["ay_exit_threshold_g"]

    smoothed = _smooth(steering["data"], sw)
    abs_steer = np.abs(smoothed)
    t = steering["time"]

    # Lateral G is the vehicle's cornering response (steering is only the
    # driver input, which is scale-dependent on corner radius and carries
    # line variance). Enter on either signal, exit only when both have
    # dropped -- this catches fast corners where a fixed steering-angle
    # threshold is systematically marginal, and keeps a bracket open through
    # a mid-corner steering correction as long as the car is still turning.
    if lat_g is not None:
        ay_on_steer_grid = np.interp(t, lat_g["time"], lat_g["data"])
        ay_abs = np.abs(_smooth(ay_on_steer_grid, sw))
    else:
        ay_abs = np.zeros_like(abs_steer)

    brackets = []
    in_corner = False
    b_start = 0
    for i in range(len(abs_steer)):
        entering = (abs_steer[i] > entry_th) or (ay_abs[i] > ay_entry_th)
        exiting = (abs_steer[i] < exit_th) and (ay_abs[i] < ay_exit_th)
        if not in_corner and entering:
            in_corner = True
            b_start = i
        elif in_corner and exiting:
            in_corner = False
            if t[i] - t[b_start] >= min_dur:
                brackets.append((b_start, i))
    if in_corner and t[-1] - t[b_start] >= min_dur:
        brackets.append((b_start, len(abs_steer) - 1))

    # Merge same-direction adjacent brackets separated by a short gap --
    # stabilises corners where steering dips briefly below threshold without
    # changing lock direction. Opposite-direction pairs (chicanes) are left
    # separate since they are distinct steering events.
    merge_gap = cd["bracket_merge_gap_s"]
    merged = []
    for b in brackets:
        if merged:
            prev = merged[-1]
            gap = t[b[0]] - t[prev[1]]
            same_dir = (np.sign(np.mean(smoothed[prev[0]:prev[1] + 1]))
                        == np.sign(np.mean(smoothed[b[0]:b[1] + 1])))
            if gap < merge_gap and same_dir:
                merged[-1] = (prev[0], b[1])
                continue
        merged.append(b)

    return merged


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
        "stable_corner_id": None,
    }


def _overlap_fraction(a, b):
    ov = min(a["bracket_end_m"], b["bracket_end_m"]) - max(a["bracket_start_m"], b["bracket_start_m"])
    if ov <= 0:
        return 0.0
    len_a = a["bracket_end_m"] - a["bracket_start_m"]
    len_b = b["bracket_end_m"] - b["bracket_start_m"]
    return ov / min(len_a, len_b)


def assign_stable_corner_ids(corners, channels):
    """
    Cross-lap corner identity: interpolate each corner's bracket span
    (steering/ay threshold-crossing start -> end, ft -> m) along lap_distance,
    then link corners across laps whose spans overlap by at least
    bracket_overlap_min_fraction of the shorter bracket -- bracket boundaries
    are stable across laps, a single peak-G apex point is not (see
    double-apex/compound corners; a proportional criterion also avoids a
    coincidental few-metre overlap between two genuinely different corners
    being mistaken for a real link). Connected components of that link graph
    are candidate clusters. Same-lap exclusivity is a hard constraint: a
    component where it's violated (the compound-straddle case, one lap's
    single wide bracket spanning two other laps' distinct corners) is split
    deterministically, seeded from the lap contributing the most brackets.
    Leaves stable_corner_id as None (no clustering) if lap_distance is
    unavailable or invalid quality.
    """
    lap_distance = channels.get("lap_distance")
    if (lap_distance is None or lap_distance.get("time") is None
            or lap_distance.get("quality") in ("missing", "failed")):
        return

    ld_time = lap_distance["time"]
    ld_data = lap_distance["data"]

    cd = _load_config()["corner_detection"]
    compound_min_len = cd["compound_corner_min_length_m"]
    min_frac = cd["bracket_overlap_min_fraction"]

    for c in corners:
        c["apex_lap_distance_m"] = float(np.interp(c["apex_time"], ld_time, ld_data)) * 0.3048

        bracket_start_t, _ = c["segments"]["entry_2_turnin"]
        _, bracket_end_t = c["segments"]["exit_5"]
        c["bracket_start_m"] = float(np.interp(bracket_start_t, ld_time, ld_data)) * 0.3048
        c["bracket_end_m"] = float(np.interp(bracket_end_t, ld_time, ld_data)) * 0.3048
        if (c["bracket_end_m"] - c["bracket_start_m"]) > compound_min_len:
            c["warnings"].append("compound_corner")

    n = len(corners)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    for i in range(n):
        for j in range(i + 1, n):
            if corners[i]["lap_number"] == corners[j]["lap_number"]:
                continue
            if _overlap_fraction(corners[i], corners[j]) >= min_frac:
                union(i, j)

    components = {}
    for i in range(n):
        components.setdefault(find(i), []).append(corners[i])

    final_clusters = []
    for comp in components.values():
        lap_counts = {}
        for c in comp:
            lap_counts[c["lap_number"]] = lap_counts.get(c["lap_number"], 0) + 1

        if max(lap_counts.values()) <= 1:
            final_clusters.append(comp)
            continue

        # Compound-straddle case: seed sub-clusters from the lap with the
        # most brackets in this component (finest granularity available),
        # then assign every other bracket to its best-overlap seed.
        max_count = max(lap_counts.values())
        candidate_laps = [lap for lap, cnt in lap_counts.items() if cnt == max_count]
        seed_lap = min(candidate_laps)
        seeds = sorted(
            (c for c in comp if c["lap_number"] == seed_lap),
            key=lambda c: c["bracket_start_m"]
        )

        sub_clusters = [[s] for s in seeds]
        for c in comp:
            if c["lap_number"] == seed_lap:
                continue
            fracs = [_overlap_fraction(c, s) for s in seeds]
            best_idx = max(range(len(seeds)), key=lambda k: (fracs[k], -seeds[k]["bracket_start_m"]))
            sub_clusters[best_idx].append(c)

            ranked = sorted(fracs, reverse=True)
            if ranked[1] >= min_frac:
                c["warnings"].append("straddles_adjacent_corners")

        for sub in sub_clusters:
            seen = set()
            for c in sub:
                if c["lap_number"] in seen:
                    raise RuntimeError(
                        f"Residual same-lap collision after seeded split at "
                        f"lap {c['lap_number']}, corner {c['corner_number']} -- "
                        f"needs manual review, not auto-resolved."
                    )
                seen.add(c["lap_number"])

        final_clusters.extend(sub_clusters)

    final_clusters.sort(key=lambda cluster: min(c["bracket_start_m"] for c in cluster))
    for cluster_id, cluster in enumerate(final_clusters, start=1):
        for c in cluster:
            c["stable_corner_id"] = cluster_id


def compute_stable_corner_positions(corners, channels):
    """
    Median GPS apex position (x/y metres, local projection) per
    stable_corner_id, computed directly from raw channels -- no vehicle
    state or stability analysis required. This is what lets the corner
    map render its markers immediately after parsing, before Analyse has
    ever run; only their severity colour needs the later stability pass.

    Interpolates log_gps_lat/lon at each corner's own apex_time (each
    channel's native time base, not a resampled common one), projects via
    the shared modules.geo formula, then takes the per-axis median across
    every lap contributing to that stable_corner_id -- the same
    median-of-medians-style philosophy used elsewhere (a single lap's GPS
    noise at the apex instant doesn't move the marker).

    Returns {stable_corner_id: {"x_m", "y_m", "n_laps"}}. Empty dict if
    GPS is missing/failed quality or no corner has a stable_corner_id.
    """
    gps_lat_ch = channels.get("log_gps_lat")
    gps_lon_ch = channels.get("log_gps_lon")
    origin_lat, origin_lon = compute_gps_origin(gps_lat_ch, gps_lon_ch)
    if origin_lat is None:
        return {}

    lat_t, lat_d = gps_lat_ch["time"], gps_lat_ch["data"]
    lon_t, lon_d = gps_lon_ch["time"], gps_lon_ch["data"]

    by_id = {}
    for c in corners:
        cid = c.get("stable_corner_id")
        if cid is None:
            continue
        apex_lat = np.interp(c["apex_time"], lat_t, lat_d)
        apex_lon = np.interp(c["apex_time"], lon_t, lon_d)
        x, y = project_latlon_to_xy(apex_lat, apex_lon, origin_lat, origin_lon)
        by_id.setdefault(cid, {"x": [], "y": []})
        by_id[cid]["x"].append(float(x))
        by_id[cid]["y"].append(float(y))

    return {
        cid: {
            "x_m": float(np.median(vals["x"])),
            "y_m": float(np.median(vals["y"])),
            "n_laps": len(vals["x"]),
        }
        for cid, vals in by_id.items()
    }