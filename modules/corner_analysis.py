# Corner segmentation and classification for parsed outing data.
# Reads detection thresholds from config/channels.json -- no hardcoded numbers.
# Pure Python/numpy -- no Qt imports.
#
# Algorithm:
#   1. Bracket corners by steering angle OR lateral G threshold crossings
#      (with hysteresis; exit requires both signals below their thresholds)
#   2. Locate apex inside each bracket via lateral G peak, cross-checked with speed minimum
#   3. Validate against lateral G threshold to filter lane changes and gentle bends
#   4. Classify by apex speed (low/medium/high) from config thresholds
#   5. Define phase boundaries:
#        Entry 1 (Brake)   -- last full throttle on preceding straight -> turn-in
#        Entry 2 (Turn-in) -- turn-in -> just before apex
#        Apex 3            -- apex point (single sample)
#        Exit 4            -- apex -> steering_unwind_fraction of steering unwind
#        Exit 5            -- steering_unwind_fraction of unwind -> steering exit threshold
#   6. Merge same-direction adjacent brackets separated by a short time gap
#      (stabilises corners with a momentary mid-corner steering dip; opposite-
#      direction pairs, i.e. chicanes, are left as separate brackets)
#   7. Cross-lap identity (assign_stable_corner_ids): link corners across laps
#      by bracket-span overlap fraction along lap_distance; connected
#      components are candidate clusters, with same-lap exclusivity enforced
#      by a TWO-PASS deterministic split: pass 1 seeds sub-clusters from the
#      lap with the most brackets and assigns every straggler to its best-
#      overlap seed LAP; pass 2 (_reassign_straddlers_pass2, WP1 Turn 1)
#      re-checks only the straggler brackets pass 1 flagged as ambiguous
#      ("straddles_adjacent_corners") against each candidate cluster's
#      CONFIDENT-member canonical window instead of one seed lap's own
#      bracket -- a once-per-corner-pair decision, not a per-lap race.
#   8. Canonical realization (_realize_canonical_corners, WP1 Turn 1): once
#      stable_corner_id membership is fixed (steps 1-7, untouched by this
#      step), derive one canonical bracket + set of phase boundaries per
#      stable corner (median per boundary across members) and re-realize
#      every valid lap's instance over that SAME window by inverting its
#      own lap_distance(t) -- including laps that detected no bracket there
#      at all (tagged "canonical_quiet": real telemetry, a quiet pass, not
#      an error) and excluding only laps whose own lap_distance range never
#      reaches the window (a genuine absence). Canonical speed_class is
#      then assigned once per stable corner from the median of these
#      re-realized per-lap apex speeds.
#
# Fallback chain (Tier B heuristics, used only when a channel is missing --
# see thesis_notes.md for the primary dual-criterion method):
#   - No steering channel: use speed minima, keeping a minimum only if its
#     rise back to the surrounding local peak clears min_apex_speed_drop_kmh
#     (a valley-depth check, not a formal peak-prominence algorithm)
#   - No lateral G channel: apex = speed minimum within bracket
#   - No throttle channel: Entry 1 (Brake) collapses to start of bracket

import json
import numpy as np
from modules.geo import compute_gps_origin, project_latlon_to_xy
from modules.stability_analysis import _interp_lap_distance_guarded

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
        segments: dict of phase -> (start_time, end_time)
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
    corners = _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds)

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
            # Bound for the NEXT corner's own brake-phase lookback -- see
            # _build_corner. A rejected bracket (corner is None) leaves this
            # unchanged, so the bound tracks the last ACCEPTED corner, not
            # every raw steering bracket.
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
        # Lookback floor: the preceding corner's own bracket end, so the
        # search for this corner's lift-off cannot reach back into a
        # different corner's exit (found occurring on corner 5 every lap,
        # 2026-08-20, see thesis_notes.md "entry_1_brake phase-boundary
        # bug"). No preceding corner (first of the lap) -> no floor beyond
        # what throttle's own lap-slice already gives. prev_corner_end_t is
        # ABSOLUTE (built with abs_start added, see segments below) while
        # throttle["time"] is lap-relative (_slice_channel) -- convert
        # before comparing, same lap so the same abs_start applies.
        lookback_floor_t = (prev_corner_end_t - speed["abs_start"]
                             if prev_corner_end_t is not None else -np.inf)
        thr_mask = (throttle["time"] < s_t_start) & (throttle["time"] >= lookback_floor_t)
        if thr_mask.any():
            thr_t = throttle["time"][thr_mask]
            thr_d = throttle["data"][thr_mask]
            # Last sample still at/above full throttle before turn-in -- the
            # lift-off transition itself ("last full throttle on preceding
            # straight", see module docstring). NOT the last off-throttle
            # sample: if the driver coasts continuously into the corner, that
            # sample sits right next to turn-in and collapses the phase to
            # near-zero (found and corrected 2026-08-20, see thesis_notes.md
            # "entry_1_brake phase-boundary bug"). If no sample reaches full
            # throttle within the (now bounded) lookback window, brake_start_t
            # keeps its s_t_start default above (zero-length phase) rather
            # than reaching past the floor or crashing.
            on_throttle = np.where(thr_d >= cd["brake_throttle_max_pct"])[0]
            if len(on_throttle) > 0:
                brake_start_t = float(thr_t[on_throttle[-1]])
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
        # Reset-guard shared with prepare_vehicle_state's s_m interpolation
        # (modules/stability_analysis.py) -- plain np.interp across a lap-
        # boundary reset would fabricate a mid-range distance corresponding
        # to no real track position. bracket_start_m/bracket_end_m below are
        # NOT guarded (out of scope for this fix, PLAN.md/thesis_notes.md).
        c["apex_lap_distance_m"] = float(_interp_lap_distance_guarded(c["apex_time"], ld_time, ld_data))

        bracket_start_t, _ = c["segments"]["entry_2_turnin"]
        _, bracket_end_t = c["segments"]["exit_5"]
        # WP1 Turn 1: closes the gap the reset-guard fix explicitly left
        # open (corner_analysis.py notes above, thesis_notes.md WP1-freeze-
        # proof entry) -- bracket edges now use the same guard as
        # apex_lap_distance_m. _interp_lap_distance_guarded already returns
        # metres (converts internally), unlike the plain np.interp this
        # replaces.
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

    _reassign_straddlers_pass2(final_clusters, min_frac)

    final_clusters.sort(key=lambda cluster: min(c["bracket_start_m"] for c in cluster))
    for cluster_id, cluster in enumerate(final_clusters, start=1):
        for c in cluster:
            c["stable_corner_id"] = cluster_id


def _reassign_straddlers_pass2(final_clusters, min_frac):
    """
    WP1 Turn 1: pass 2 of the two-pass canonical split. Pass 1 above (the
    connected-components + seeded split) is unchanged by this function --
    it assigns each straggler bracket to whichever seed LAP's own raw
    bracket it best-overlaps, a per-lap decision sensitive to that one
    lap's own bracket geometry. Real evidence this matters (Dubai,
    2026-07-26 diagnostics): a lap detecting only a short, late, high-speed
    sub-feature of a compound complex (45.7 m, 151.7 km/h apex) vs. another
    lap detecting the whole sustained low-speed complex (305 m, 75.8 km/h
    apex) landed in different stable_corner_ids despite the short bracket
    sitting almost entirely inside the wide one -- exactly the C10/C11
    speed-class straddling and the corner_radius_filtered overlap
    disagreement this session's diagnostics flagged.

    Pass 2 instead compares every straddle-tagged corner against each
    candidate cluster's CONFIDENT-member canonical window (median bracket
    over members NOT tagged "straddles_adjacent_corners" -- always >=1,
    since a split's seed corner is never itself straddle-tagged) -- a
    stable, once-per-corner-pair decision instead of a per-lap race.
    Clusters with no straddle-tagged member (the common case) are
    untouched. Mutates `final_clusters` in place; called just before the
    existing sort+enumerate step, so no separate renumbering pass is
    needed -- that step already consumes whatever membership pass 2
    leaves behind.
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
            # Not auto-resolved -- same convention as pass 1's residual
            # same-lap-collision guard above (needs manual review).
            c["warnings"].append("pass2_reassignment_blocked_by_collision")
            continue
        final_clusters[cur_idx].remove(c)
        final_clusters[best_idx].append(c)
        c["warnings"].append("canonical_split_reassigned")


def _slice_channel_abs(ch, start_t, end_t):
    # Same quality/window checks as _slice_channel, but keeps ABSOLUTE
    # time (no shift to lap-relative) -- needed here because canonical
    # s-positions are inverted against lap_distance's own absolute time
    # base, matching apex_time/bracket_start_m elsewhere in this module.
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    t, d = ch["time"], ch["data"]
    mask = (t >= start_t) & (t <= end_t)
    if not mask.any():
        return None
    return {"time": t[mask], "data": d[mask]}


def _invert_s_to_t(target_s_m, lap_start_t, lap_end_t, ld_time, ld_data_ft):
    # Inverse of _interp_lap_distance_guarded: given a target track
    # position, find the time within this lap's own span at which its
    # lap_distance channel crosses it. lap_distance is physically
    # monotonic increasing within one lap (distance travelled cannot
    # decrease); np.maximum.accumulate is a Tier B numerical-safety guard
    # against small sensor noise before inversion, not a science claim.
    # Returns NaN if target_s_m falls outside this lap's own covered
    # range -- a genuine absence (e.g. a shortened lap that never reached
    # this track position), distinct from a lap that reached it quietly.
    mask = (ld_time >= lap_start_t) & (ld_time <= lap_end_t)
    if not mask.any():
        return float("nan")
    t = ld_time[mask]
    s_m = np.maximum.accumulate(ld_data_ft[mask] * 0.3048)
    if target_s_m < s_m[0] or target_s_m > s_m[-1]:
        return float("nan")
    return float(np.interp(target_s_m, s_m, t))


def _pooled_median_ay_profile(apex_lo, apex_hi, valid_laps, ld_time, ld_data, lat_g_smoothed, grid_step):
    # Cross-lap median |ay| vs track position, s-anchored, between two apex
    # positions -- the same "sort samples by track position, pool across
    # laps" idea Module 5's stability regression already uses (thesis_
    # notes.md, "Module 5 chair-basis alignment"), reused here as a Tier B
    # geometric post-pass, not a science claim. grid_step is a config
    # tunable (default 2 m, matching that existing s-grid convention).
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
    """
    WP1 Turn 3 (reviewer decision: partition, not merge). Any pair of
    canonical windows overlapping more than `overlap_max` (fraction of the
    smaller window) is resolved by placing a shared boundary at the |ay|
    minimum of the pooled (cross-lap median) lateral-g profile between the
    two apex positions, then truncating both windows to it -- non-
    overlapping by construction. Tier B geometric post-pass (the same
    "split a compound at an ay minimum" idea already named as an open
    refinement in thesis_notes.md's original compound-corner finding,
    applied here to canonical WINDOWS, not a per-lap phase display).

    Phase-internal boundaries (brake_s/turnin_s/half_s) are re-clamped into
    the truncated range afterward -- a boundary whose defining event no
    longer sits inside its sub-window collapses to a degenerate (zero-
    length) phase at the window edge. summarise_corners/_classify_corner
    already read that as "no signal" (empty time slice -> n_samples=0 ->
    NaN stats -> skipped by the worst-value search) -- no change needed to
    either of those functions for this to be handled honestly.

    Mutates and returns `canon_by_id`; also returns the list of resolved
    (lo_id, hi_id, boundary_s, overlap_fraction_before) tuples for
    traceability.
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
                continue  # degenerate (coincident/inverted apexes) -- should not occur
            grid, pooled = _pooled_median_ay_profile(
                apex_lo, apex_hi, valid_laps, ld_time, ld_data, lat_g_smoothed, grid_step)
            if np.all(np.isnan(pooled)):
                boundary = (apex_lo + apex_hi) / 2.0  # documented fallback: no lap covers the gap
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


def _realize_canonical_corners(corners, channels, laps, cd, speed_thresholds):
    """
    WP1 Turn 1: canonical bracket + phase realization. Everything above
    (per-lap detection, connected-components clustering, the two-pass
    split) is finished and untouched by this function -- stable_corner_id
    membership is already fixed. This only decides how big each stable
    corner's window is and applies it identically to every valid lap.

    For each stable_corner_id: canonical bracket_start_m/end_m and the
    phase-internal boundaries (brake_start_s, turnin_s, half_s -- apex_s
    reuses the already-guarded apex_lap_distance_m) are each the MEDIAN
    across cluster members, independently per boundary (robust to one lap
    being atypical on a single boundary without importing its other
    boundaries too -- same reasoning as the project's existing median-of-
    medians aggregations elsewhere). Every valid lap is then re-realized
    over this canonical window by inverting its own guarded s_m(t): a lap
    that detected no bracket there at all still gets an instance (tagged
    "canonical_quiet" -- real telemetry, a quiet pass, informative, not an
    error); a lap whose own lap_distance range never reaches the window is
    the only case left absent, consistent with the existing "missing
    corner = empty grid cell" convention.

    apex_speed/apex_lateral_g stay genuinely per-lap (min speed / max
    |lat_g| within that lap's own canonical window -- real signal). Only
    speed_class is made canonical: assigned once per stable_corner_id from
    the median of these per-lap apex speeds, then written identically onto
    every instance (previously each instance classified from its own,
    per-lap-jittered apex speed -- see this session's speed-class-
    boundary diagnostic for why that was unstable for a borderline
    corner).

    WP1 Turn 3: before per-lap realization, _resolve_canonical_overlaps
    truncates any pair of canonical windows overlapping more than
    canonical_overlap_max to a shared, ay-minimum-derived boundary
    (reviewer decision: partition, not merge) -- see that function's
    docstring. Instances whose stable corner was truncated are tagged
    "canonical_boundary_resolved".
    """
    lap_distance = channels.get("lap_distance")
    if (lap_distance is None or lap_distance.get("time") is None
            or lap_distance.get("quality") in ("missing", "failed")):
        return corners  # no spatial anchor -- leave per-lap realization as-is

    ld_time, ld_data = lap_distance["time"], lap_distance["data"]
    compound_min_len = cd["compound_corner_min_length_m"]
    sw = cd["smoothing_window_samples"]
    low_max, medium_max = speed_thresholds["low_max"], speed_thresholds["medium_max"]

    by_id = {}
    for c in corners:
        by_id.setdefault(c["stable_corner_id"], []).append(c)
    valid_laps = {l["lap_number"]: l for l in laps if l.get("is_valid_for_analysis", False)}

    # Smooth each lap's FULL speed/lat_g trace once, then slice the window
    # out of the already-smoothed array -- matching _build_corner's order
    # of operations. Slicing to a short bracket window BEFORE smoothing
    # would run the convolution on a short array, where "same"-mode edge
    # effects (fewer real taps near the boundary, kernel weights not
    # renormalised) bias the smoothed value toward zero right at the
    # window edges -- exactly where a min/max search would wrongly lock on.
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
        brake_s, turnin_s, half_s = [], [], []
        for m in members:
            brake_t, turnin_t = m["segments"]["entry_1_brake"][0], m["segments"]["entry_2_turnin"][0]
            half_t = m["segments"]["exit_4"][1]
            brake_s.append(float(_interp_lap_distance_guarded(brake_t, ld_time, ld_data)))
            turnin_s.append(float(_interp_lap_distance_guarded(turnin_t, ld_time, ld_data)))
            half_s.append(float(_interp_lap_distance_guarded(half_t, ld_time, ld_data)))
        canon_by_id[cid] = {
            "start": float(np.median([m["bracket_start_m"] for m in members])),
            "end": float(np.median([m["bracket_end_m"] for m in members])),
            "apex": float(np.median([m["apex_lap_distance_m"] for m in members])),
            "brake_s": float(np.nanmedian(brake_s)),
            "turnin_s": float(np.nanmedian(turnin_s)),
            "half_s": float(np.nanmedian(half_s)),
        }

    # WP1 Turn 3 (reviewer decision: partition, not merge) -- resolves any
    # pair of canonical windows overlapping more than canonical_overlap_max
    # by truncating both to a shared boundary. Mutates canon_by_id in
    # place; must run before compound_corner is decided below, since
    # truncation can shrink a window below the compound-length threshold.
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
                continue  # genuine absence: this lap never reached the canonical window

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
                "speed_class": None,  # filled in below, canonical per stable_corner_id
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