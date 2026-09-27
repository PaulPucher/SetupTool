# Corner detection, classification and cross-lap identity. Thresholds
# from config/channels.json.
#
# 1. brackets: steering OR lateral g above threshold (hysteresis, exit
#    needs both below)
# 2. apex = lateral g peak, cross-checked against speed minimum
# 3. lateral g check drops lane changes / gentle bends
# 4. speed class from apex speed
# 5. phases: entry_1 brake (last full throttle -> turn-in), entry_2 turn-in
#    (-> apex), apex, exit_4 (-> steering_unwind_fraction), exit_5 (-> exit)
# 6. same-direction brackets with a short gap merged; chicanes not
# 7. cross-lap identity by bracket overlap on lap_distance, two-pass split
# 8. canonical window per stable corner (median boundaries), realized on
#    every valid lap
# 9. laps slower than lap_time_representative_factor x fastest can't shape
#    canonical corners
#
# Fallbacks (only when a channel is missing):
#   no steering -> speed minima with min_apex_speed_drop_kmh valley depth
#   no lateral g -> apex = speed minimum
#   no throttle -> entry_1 starts at bracket start

import json
import numpy as np
from modules.geo import compute_gps_origin, project_latlon_to_xy
from modules.stability_analysis import _interp_lap_distance_guarded, _normalize_lap_distance_to_metres

CHANNELS_CONFIG_PATH = "config/channels.json"


def _load_config():
    with open(CHANNELS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _smooth(arr, window):
    if window <= 1 or len(arr) < window:
        return arr
    kernel = np.ones(window) / window
    return np.convolve(arr, kernel, mode="same")


def _representative_lap_numbers(laps, factor):
    """Valid laps within factor x the fastest valid lap. Others are still
    analysed but can't seed or shape a canonical corner.
    """
    from modules.csv_parser import _effective_lap_time  # local import, circular otherwise

    valid = [l for l in laps if l.get("is_valid_for_analysis", False)]
    if not valid:
        return set()
    fastest_time = min(_effective_lap_time(l) for l in valid)
    return {l["lap_number"] for l in valid if _effective_lap_time(l) <= fastest_time * factor}


def analyse_corners(parsed_data):
    """Detect and classify corners on all valid laps; one dict per corner per lap:
        lap_number, corner_number, speed_class, apex_time, apex_speed,
        apex_lateral_g, segments {phase: (start, end)},
        method ("steering" / "speed_fallback"), warnings
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

    representative_laps = _representative_lap_numbers(laps, cd["lap_time_representative_factor"])
    assign_stable_corner_ids(corners, channels, representative_laps)
    corners = _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds, representative_laps)

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
    prev_corner_end_t = None
    for i, (b_start_idx, b_end_idx) in enumerate(brackets, start=1):
        corner = _build_corner(
            lap_number, i, method,
            b_start_idx, b_end_idx,
            steering, speed, lat_g, throttle,
            cd, speed_thresholds,
            prev_corner_end_t
        )
        if corner is not None:
            corners.append(corner)
            # lookback bound for the next corner's brake phase; rejected brackets
            # don't move it
            prev_corner_end_t = corner["segments"]["exit_5"][1]

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

    # Enter on steering OR lateral g, exit only when both have dropped.
    # Steering alone is marginal in fast corners (small angle) and drops
    # out on mid-corner corrections.
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

    # merge same-direction brackets split by a short dip; chicanes stay separate
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
                  cd, speed_thresholds,
                  prev_corner_end_t=None):
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
        warnings.append("lateral G missing -- apex from speed minimum")
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
        # lookback floor = previous corner's end, so the lift-off search can't
        # reach into its exit. prev_corner_end_t absolute, throttle time
        # lap-relative -> subtract abs_start.
        lookback_floor_t = (prev_corner_end_t - speed["abs_start"]
                             if prev_corner_end_t is not None else -np.inf)
        thr_mask = (throttle["time"] < s_t_start) & (throttle["time"] >= lookback_floor_t)
        if thr_mask.any():
            thr_t = throttle["time"][thr_mask]
            thr_d = throttle["data"][thr_mask]
            # last full-throttle sample before turn-in = lift-off. Not the last
            # off-throttle one -- coasting in would collapse the phase.
            # none in the window -> zero-length phase (s_t_start default)
            on_throttle = np.where(thr_d >= cd["brake_throttle_max_pct"])[0]
            if len(on_throttle) > 0:
                brake_start_t = float(thr_t[on_throttle[-1]])
    else:
        warnings.append("throttle missing -- brake phase = turn-in start")

    if steering is not None:
        bracket_steer = np.abs(_smooth(steering["data"][b_start_idx:b_end_idx + 1], sw))
        bracket_t = steering["time"][b_start_idx:b_end_idx + 1]
        post_apex_mask = bracket_t > apex_t
        if post_apex_mask.any():
            post_steer = bracket_steer[post_apex_mask]
            post_t = bracket_t[post_apex_mask]
            peak_post = float(np.max(post_steer))
            half_th = peak_post * cd["steering_unwind_fraction"]
            half_idx = np.argmax(post_steer <= half_th)
            half_t = float(post_t[half_idx]) if half_idx > 0 else float(post_t[-1])
        else:
            half_t = s_t_end
    else:
        half_t = apex_t + (s_t_end - apex_t) * cd["steering_unwind_fraction"]

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


def assign_stable_corner_ids(corners, channels, representative_laps):
    """Cross-lap corner identity. Bracket spans mapped onto lap_distance (m);
    corners linked if spans overlap >= bracket_overlap_min_fraction of the
    shorter one. Brackets are stable across laps, a peak-G apex isn't; a
    fraction avoids linking on a few metres of coincidental overlap.
    Connected components = clusters. Same-lap exclusivity is hard: a violating
    component is split, seeded from the lap with the most brackets.

    Clusters with only non-representative laps dropped after clustering
    (link logic unchanged).
    No usable lap_distance -> stable_corner_id stays None.
    """
    lap_distance = channels.get("lap_distance")
    if (lap_distance is None or lap_distance.get("time") is None
            or lap_distance.get("quality") in ("missing", "failed")):
        return

    ld_time = lap_distance["time"]
    ld_data = _normalize_lap_distance_to_metres(lap_distance["data"], lap_distance.get("unit_raw"))

    cd = _load_config()["corner_detection"]
    compound_min_len = cd["compound_corner_min_length_m"]
    min_frac = cd["bracket_overlap_min_fraction"]

    for c in corners:
        # reset guard as in prepare_vehicle_state -- plain interp across a lap
        # reset invents a mid-lap distance
        c["apex_lap_distance_m"] = float(_interp_lap_distance_guarded(c["apex_time"], ld_time, ld_data))

        bracket_start_t, _ = c["segments"]["entry_2_turnin"]
        _, bracket_end_t = c["segments"]["exit_5"]
        # bracket edges: same guard (returns metres already)
        c["bracket_start_m"] = float(_interp_lap_distance_guarded(bracket_start_t, ld_time, ld_data))
        c["bracket_end_m"] = float(_interp_lap_distance_guarded(bracket_end_t, ld_time, ld_data))
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

        # compound straddle: seed from the lap with the most brackets, assign the
        # rest by best overlap
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

    _reassign_straddlers_pass2(final_clusters, min_frac)

    # all-non-representative clusters -> corners dropped outright; id None
    # would lump unrelated corners into one group
    final_clusters = [cluster for cluster in final_clusters
                       if any(c["lap_number"] in representative_laps for c in cluster)]
    kept = {id(c) for cluster in final_clusters for c in cluster}
    corners[:] = [c for c in corners if id(c) in kept]

    final_clusters.sort(key=lambda cluster: min(c["bracket_start_m"] for c in cluster))
    for cluster_id, cluster in enumerate(final_clusters, start=1):
        for c in cluster:
            c["stable_corner_id"] = cluster_id


def _reassign_straddlers_pass2(final_clusters, min_frac):
    """Pass 2 of the split. Pass 1 assigns a straggler to the best-overlapping
    seed lap bracket -- a per-lap race. Dubai: a 46 m sub-feature and the
    305 m complex around it ended up in different ids.
    Pass 2 compares each "straddles_adjacent_corners" corner against each
    cluster's confident-member median window (always >= 1 -- seeds are never
    straddle-tagged). Clusters without straddlers untouched.
    Mutates final_clusters; the sort+enumerate after it renumbers.
    """
    def canonical_window(members):
        confident = [m for m in members if "straddles_adjacent_corners" not in m["warnings"]]
        if not confident:
            confident = members
        return (float(np.median([m["bracket_start_m"] for m in confident])),
                float(np.median([m["bracket_end_m"] for m in confident])))

    canon = [canonical_window(cluster) for cluster in final_clusters]
    straddlers = [(idx, c) for idx, cluster in enumerate(final_clusters) for c in cluster
                  if "straddles_adjacent_corners" in c["warnings"]]

    for cur_idx, c in straddlers:
        best_idx, best_frac = cur_idx, -1.0
        for idx, (start_m, end_m) in enumerate(canon):
            ov = min(c["bracket_end_m"], end_m) - max(c["bracket_start_m"], start_m)
            if ov <= 0:
                continue
            frac = ov / min(c["bracket_end_m"] - c["bracket_start_m"], end_m - start_m)
            if frac > best_frac:
                best_frac, best_idx = frac, idx
        if best_idx == cur_idx or best_frac < min_frac:
            continue
        collision = any(m["lap_number"] == c["lap_number"] for m in final_clusters[best_idx])
        if collision:
            # not auto-resolved -> manual review, as in pass 1
            c["warnings"].append("pass2_reassignment_blocked_by_collision")
            continue
        final_clusters[cur_idx].remove(c)
        final_clusters[best_idx].append(c)
        c["warnings"].append("canonical_split_reassigned")


def _slice_channel_abs(ch, start_t, end_t):
    # like _slice_channel but absolute time -- lap_distance's time base
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    t, d = ch["time"], ch["data"]
    mask = (t >= start_t) & (t <= end_t)
    if not mask.any():
        return None
    return {"time": t[mask], "data": d[mask]}


def _invert_s_to_t(target_s_m, lap_start_t, lap_end_t, ld_time, ld_data_m):
    # Inverse of _interp_lap_distance_guarded: time at which this lap crosses
    # target_s_m. maximum.accumulate = noise guard (distance is monotonic
    # within a lap). NaN = lap never got there (absence, not a quiet pass).
    # ld_data_m already in metres.
    mask = (ld_time >= lap_start_t) & (ld_time <= lap_end_t)
    if not mask.any():
        return float("nan")
    t = ld_time[mask]
    s_m = np.maximum.accumulate(ld_data_m[mask])
    if target_s_m < s_m[0] or target_s_m > s_m[-1]:
        return float("nan")
    return float(np.interp(target_s_m, s_m, t))


def _pooled_median_ay_profile(apex_lo, apex_hi, valid_laps, ld_time, ld_data, lat_g_smoothed, grid_step):
    # cross-lap median |ay| over track position (s-grid, as in the stability
    # regression); grid_step from config
    n_steps = max(2, int(round((apex_hi - apex_lo) / grid_step)) + 1)
    grid = np.linspace(apex_lo, apex_hi, n_steps)
    pooled = np.full(n_steps, np.nan)
    for i, s in enumerate(grid):
        vals = []
        for lap_number, lap in valid_laps.items():
            if lap_number not in lat_g_smoothed:
                continue
            t = _invert_s_to_t(s, lap["start_time"], lap["end_time"], ld_time, ld_data)
            if np.isnan(t):
                continue
            g_t, sm_g = lat_g_smoothed[lap_number]
            vals.append(float(np.interp(t, g_t, np.abs(sm_g))))
        if vals:
            pooled[i] = float(np.median(vals))
    return grid, pooled


def _resolve_canonical_overlaps(canon_by_id, valid_laps, ld_time, ld_data, lat_g_smoothed,
                                 overlap_max, grid_step):
    """Windows overlapping > overlap_max (of the smaller one) -> partitioned,
    not merged: shared boundary at the |ay| minimum of the cross-lap median
    profile between the two apexes.
    Inner boundaries re-clamped; a phase that loses its event collapses to
    zero length -> no signal downstream.
    Returns (canon_by_id, [(lo_id, hi_id, boundary_s, overlap_before)]).
    """
    ids = sorted(canon_by_id)
    resolved = []
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            wa, wb = canon_by_id[a], canon_by_id[b]
            ov = min(wa["end"], wb["end"]) - max(wa["start"], wb["start"])
            if ov <= 0:
                continue
            frac = ov / min(wa["end"] - wa["start"], wb["end"] - wb["start"])
            if frac <= overlap_max:
                continue
            lo, hi = (a, b) if wa["apex"] <= wb["apex"] else (b, a)
            apex_lo, apex_hi = canon_by_id[lo]["apex"], canon_by_id[hi]["apex"]
            if apex_hi <= apex_lo:
                continue  # coincident/inverted apexes, shouldn't happen
            grid, pooled = _pooled_median_ay_profile(
                apex_lo, apex_hi, valid_laps, ld_time, ld_data, lat_g_smoothed, grid_step)
            if np.all(np.isnan(pooled)):
                boundary = (apex_lo + apex_hi) / 2.0  # no lap covers the gap
            else:
                boundary = float(grid[np.nanargmin(pooled)])
            canon_by_id[lo]["end"] = min(canon_by_id[lo]["end"], boundary)
            canon_by_id[hi]["start"] = max(canon_by_id[hi]["start"], boundary)
            for cid in (lo, hi):
                w = canon_by_id[cid]
                w["brake_s"] = float(np.clip(w["brake_s"], w["start"], w["end"]))
                w["turnin_s"] = float(np.clip(w["turnin_s"], w["start"], w["end"]))
                w["half_s"] = float(np.clip(w["half_s"], w["start"], w["end"]))
            resolved.append((lo, hi, boundary, frac))
    return resolved


def _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds, representative_laps):
    """One canonical window + phase boundaries per stable corner, applied to
    every valid lap. Membership already fixed here.

    Each boundary = median over representative members, per boundary (one
    odd boundary doesn't drag the others).
    Every valid lap re-realized over that window by inverting its s_m(t):
    no bracket there -> "canonical_quiet" instance (real, quiet pass);
    never reaches the window -> absent.
    apex_speed / apex_lateral_g stay per lap. speed_class canonical, from
    the median apex speed -- per-lap class jittered on borderline corners.
    Overlapping windows partitioned first (_resolve_canonical_overlaps)
    -> tagged "canonical_boundary_resolved".
    """
    lap_distance = channels.get("lap_distance")
    if (lap_distance is None or lap_distance.get("time") is None
            or lap_distance.get("quality") in ("missing", "failed")):
        return corners  # no lap_distance -> keep per-lap

    ld_time = lap_distance["time"]
    ld_data = _normalize_lap_distance_to_metres(lap_distance["data"], lap_distance.get("unit_raw"))
    compound_min_len = cd["compound_corner_min_length_m"]
    sw = cd["smoothing_window_samples"]
    low_max, medium_max = speed_thresholds["low_max"], speed_thresholds["medium_max"]

    by_id = {}
    for c in corners:
        by_id.setdefault(c["stable_corner_id"], []).append(c)
    valid_laps = {l["lap_number"]: l for l in laps if l.get("is_valid_for_analysis", False)}

    # smooth the full lap, then slice -- smoothing a short slice biases the
    # edges ("same" convolution), right where min/max would lock on
    lap_speed_smoothed = {}
    lap_lat_g_smoothed = {}
    for lap_number, lap in valid_laps.items():
        speed_full = _slice_channel_abs(channels.get("ecu_speed"), lap["start_time"], lap["end_time"])
        if speed_full is not None:
            lap_speed_smoothed[lap_number] = (speed_full["time"], _smooth(speed_full["data"], sw))
        lat_g_full = _slice_channel_abs(channels.get("log_acc_y"), lap["start_time"], lap["end_time"])
        if lat_g_full is not None:
            lap_lat_g_smoothed[lap_number] = (lat_g_full["time"], _smooth(lat_g_full["data"], sw))

    canon_by_id = {}
    for cid, members in by_id.items():
        rep_members = [m for m in members if m["lap_number"] in representative_laps]
        if not rep_members:
            raise RuntimeError(
                f"stable_corner_id {cid} has zero representative-lap members -- "
                f"assign_stable_corner_ids should already have dropped this cluster."
            )
        brake_s, turnin_s, half_s = [], [], []
        for m in rep_members:
            brake_t, turnin_t = m["segments"]["entry_1_brake"][0], m["segments"]["entry_2_turnin"][0]
            half_t = m["segments"]["exit_4"][1]
            brake_s.append(float(_interp_lap_distance_guarded(brake_t, ld_time, ld_data)))
            turnin_s.append(float(_interp_lap_distance_guarded(turnin_t, ld_time, ld_data)))
            half_s.append(float(_interp_lap_distance_guarded(half_t, ld_time, ld_data)))
        canon_by_id[cid] = {
            "start": float(np.median([m["bracket_start_m"] for m in rep_members])),
            "end": float(np.median([m["bracket_end_m"] for m in rep_members])),
            "apex": float(np.median([m["apex_lap_distance_m"] for m in rep_members])),
            "brake_s": float(np.nanmedian(brake_s)),
            "turnin_s": float(np.nanmedian(turnin_s)),
            "half_s": float(np.nanmedian(half_s)),
        }

    # before the compound_corner decision -- truncation can shrink a window
    # below the compound threshold
    overlap_max = cd["canonical_overlap_max"]
    grid_step = cd["canonical_boundary_grid_step_m"]
    resolved_pairs = _resolve_canonical_overlaps(
        canon_by_id, valid_laps, ld_time, ld_data, lap_lat_g_smoothed, overlap_max, grid_step)
    boundary_resolved_ids = {cid for pair in resolved_pairs for cid in pair[:2]}

    realized = []
    for cid, members in by_id.items():
        canon_start_m = canon_by_id[cid]["start"]
        canon_end_m = canon_by_id[cid]["end"]
        canon_apex_m = canon_by_id[cid]["apex"]
        canon_brake_s = canon_by_id[cid]["brake_s"]
        canon_turnin_s = canon_by_id[cid]["turnin_s"]
        canon_half_s = canon_by_id[cid]["half_s"]
        is_compound = (canon_end_m - canon_start_m) > compound_min_len

        quiet_laps = set(valid_laps) - {m["lap_number"] for m in members}
        cluster_method = members[0]["method"]

        instances = []
        for lap_number, lap in valid_laps.items():
            lap_start_t, lap_end_t = lap["start_time"], lap["end_time"]
            t_brake = _invert_s_to_t(canon_brake_s, lap_start_t, lap_end_t, ld_time, ld_data)
            t_turnin = _invert_s_to_t(canon_turnin_s, lap_start_t, lap_end_t, ld_time, ld_data)
            t_apex = _invert_s_to_t(canon_apex_m, lap_start_t, lap_end_t, ld_time, ld_data)
            t_half = _invert_s_to_t(canon_half_s, lap_start_t, lap_end_t, ld_time, ld_data)
            t_end = _invert_s_to_t(canon_end_m, lap_start_t, lap_end_t, ld_time, ld_data)
            if any(np.isnan(v) for v in (t_brake, t_turnin, t_apex, t_half, t_end)):
                continue  # lap never reached the window

            if lap_number not in lap_speed_smoothed:
                continue
            speed_t, sm_speed_full = lap_speed_smoothed[lap_number]
            window_mask = (speed_t >= t_turnin) & (speed_t <= t_end)
            if not window_mask.any():
                continue
            apex_speed = float(np.min(sm_speed_full[window_mask]))

            apex_g = None
            if lap_number in lap_lat_g_smoothed:
                g_t, sm_g_full = lap_lat_g_smoothed[lap_number]
                g_mask = (g_t >= t_turnin) & (g_t <= t_end)
                if g_mask.any():
                    apex_g = float(np.max(np.abs(sm_g_full[g_mask])))

            warnings = ["compound_corner"] if is_compound else []
            if lap_number in quiet_laps:
                warnings.append("canonical_quiet")
            if cid in boundary_resolved_ids:
                warnings.append("canonical_boundary_resolved")

            instances.append({
                "lap_number": lap_number,
                "corner_number": next((m["corner_number"] for m in members
                                       if m["lap_number"] == lap_number), None),
                "speed_class": None,  # set below, per stable corner
                "apex_time": t_apex,
                "apex_speed": apex_speed,
                "apex_lateral_g": apex_g,
                "segments": {
                    "entry_1_brake":  (t_brake, t_turnin),
                    "entry_2_turnin": (t_turnin, t_apex),
                    "apex_3":         (t_apex, t_apex),
                    "exit_4":         (t_apex, t_half),
                    "exit_5":         (t_half, t_end),
                },
                "method": cluster_method,
                "warnings": warnings,
                "stable_corner_id": cid,
                "bracket_start_m": canon_start_m,
                "bracket_end_m": canon_end_m,
                "apex_lap_distance_m": canon_apex_m,
            })

        if instances:
            canon_apex_speed = float(np.median([i["apex_speed"] for i in instances]))
            if canon_apex_speed < low_max:
                canon_class = "low"
            elif canon_apex_speed < medium_max:
                canon_class = "medium"
            else:
                canon_class = "high"
            for i in instances:
                i["speed_class"] = canon_class

        realized.append((cid, instances))

    out = []
    for _cid, instances in realized:
        out.extend(instances)
    return out


def compute_stable_corner_positions(corners, channels):
    """Median GPS apex position (local x/y m) per stable_corner_id, from raw
    channels -- map markers work before Analyse has run.
    GPS interpolated at each apex_time, projected via modules.geo, median per
    axis across laps.
    Returns {id: {"x_m", "y_m", "n_laps"}}; {} if no GPS or no ids.
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