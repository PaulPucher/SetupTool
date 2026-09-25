# WP-ELICIT Phase C3 (2026-09-24), read-only census. Per reviewer
# redirect: the blowoff trigger is kerb-strike SEVERITY repeating across
# laps at a corner (peak |az_g| inside the EXISTING kerb_mask, within
# that corner instance's own overall time extent), never kerb_fraction
# alone (kerb contact itself is normal driving, not evidence). This
# script censuses the real per-corner-instance peak severity on both real
# sessions, pooled, gap-selects a threshold via the reviewer's own
# minority-firing rule, and attributes the firing corners' axle via
# per-wheel travel VELOCITY inside the kerb window (WP-FD1+2 damper_
# motion channels/dead-channel guard, reused) -- data only, no config/
# production change (propose, wait for confirmation before wiring).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.wheel_loads import (
    CORNERS, CORNER_AXLE, TRAVEL_CHANNEL, AXLE_CORNERS,
    _interp_channel, _normalize_travel_to_mm, _channel_is_dead,
)

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "C:/UNI/Bachelorarbeit/outings/GT3_PRC_MLA-v3.txt"
CHOSEN_THRESHOLD_G = 2.9571  # pooled p75, nonzero-instance population -- see report


def corner_window(corner, t):
    segments = corner.get("segments", {})
    starts = [s for s, e in segments.values() if e >= s]
    ends = [e for s, e in segments.values() if e >= s]
    if not starts:
        return None
    start_t, end_t = min(starts), max(ends)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        return None
    return lo, hi


def peak_kerb_severity(lo, hi, az_g, kerb_mask):
    window_kerb = kerb_mask[lo:hi]
    if not window_kerb.any():
        return 0.0  # no kerb contact this lap at this corner -- real zero, not missing
    return float(np.max(np.abs(az_g[lo:hi][window_kerb])))


def run(label, raw_file):
    print(f"\n=== {label} ({raw_file}) ===")
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=1)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(raw_file)
    channels = data["channels"]
    state = prepare_vehicle_state(channels, effective_params)
    if state is None:
        print("prepare_vehicle_state returned None -- aborting this session")
        return None

    t = state["time"]
    az_g = state.get("az_g")
    kerb_mask = state.get("kerb_mask")
    if az_g is None or kerb_mask is None:
        print("az_g or kerb_mask unavailable -- aborting this session")
        return None

    corners = data.get("corners", [])
    by_corner = {}  # cid -> list of (lap_number, peak, lo, hi)
    for c in corners:
        cid = c["stable_corner_id"]
        w = corner_window(c, t)
        if w is None:
            continue
        lo, hi = w
        peak = peak_kerb_severity(lo, hi, az_g, kerb_mask)
        by_corner.setdefault(cid, []).append((c["lap_number"], peak, lo, hi))

    all_peaks = [inst[1] for insts in by_corner.values() for inst in insts]
    nonzero_peaks = [p for p in all_peaks if p > 0]
    print(f"corner instances with a computable window: {len(all_peaks)} "
          f"(distinct corners: {len(by_corner)})")
    print(f"instances with >=1 kerb-masked sample in window: {len(nonzero_peaks)}")
    if nonzero_peaks:
        arr = np.array(nonzero_peaks)
        for pct in (10, 25, 50, 75, 90, 95):
            print(f"  peak|az_g| p{pct} (nonzero instances only) = {np.percentile(arr, pct):.4f} g")
        print(f"  min={arr.min():.4f} max={arr.max():.4f} g")

    return by_corner, all_peaks, t, kerb_mask, channels


def firing_report(label, by_corner, thresholds, n_total_corners):
    print(f"\n-- {label}: firing-corner counts per candidate threshold (of {n_total_corners} distinct corners) --")
    firing_by_threshold = {}
    for thr in thresholds:
        firing = [cid for cid, insts in by_corner.items()
                  if sum(1 for _, p, _, _ in insts if p >= thr) >= 2]
        firing_by_threshold[thr] = firing
        print(f"  threshold={thr:.4f}g: firing corners = {len(firing)} {sorted(firing)}")
    return firing_by_threshold


def attribute_axle(cid, instances, threshold, t, channels, wl_cfg):
    dead_std_max = wl_cfg["dead_channel_std_max_travel_mm"]
    travel_by_wheel = {}
    for wheel in CORNERS:
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        if _channel_is_dead(raw_mm, dead_std_max):
            continue
        travel_by_wheel[wheel] = raw_mm

    firing_instances = [(lo, hi) for _, p, lo, hi in instances if p >= threshold]

    peak_rate_by_wheel = {}
    for wheel, travel_mm in travel_by_wheel.items():
        peaks = []
        for lo, hi in firing_instances:
            seg_t = t[lo:hi]
            seg_travel = travel_mm[lo:hi]
            valid = np.isfinite(seg_travel)
            if valid.sum() < 2:
                continue
            idx = np.where(valid)[0]
            rates = np.gradient(seg_travel[idx], seg_t[idx])
            peaks.append(float(np.max(np.abs(rates))))
        if peaks:
            peak_rate_by_wheel[wheel] = max(peaks)

    axle_score = {}
    for axle, wheels in AXLE_CORNERS.items():
        scores = [peak_rate_by_wheel[w] for w in wheels if w in peak_rate_by_wheel]
        if len(scores) == len(wheels):  # both wheels of this axle evaluable
            axle_score[axle] = max(scores)

    print(f"  corner {cid}: peak travel-rate by wheel (mm/s) = "
          f"{ {w: round(v, 1) for w, v in peak_rate_by_wheel.items()} }")
    if len(axle_score) < 2:
        print(f"  corner {cid}: axle attribution -- NOT ATTRIBUTABLE (car-wide kerb evidence)")
        return None
    dominant = max(axle_score, key=axle_score.get)
    print(f"  corner {cid}: axle attribution -- {dominant} "
          f"(front={axle_score.get('front')}, rear={axle_score.get('rear')})")
    return dominant


if __name__ == "__main__":
    import json
    wl_cfg = json.load(open("config/parameters.json", encoding="utf-8"))["wheel_loads"]

    dubai_result = run("Dubai", DUBAI_FILE)
    v3_result = run("v3", V3_FILE)

    pooled_nonzero = []
    if dubai_result:
        pooled_nonzero += [p for p in dubai_result[1] if p > 0]
    if v3_result:
        pooled_nonzero += [p for p in v3_result[1] if p > 0]
    if pooled_nonzero:
        arr = np.array(pooled_nonzero)
        print(f"\n=== POOLED (both sessions, nonzero instances only, n={len(arr)}) ===")
        for pct in (10, 25, 50, 75, 90, 95):
            print(f"  p{pct} = {np.percentile(arr, pct):.4f} g")
        candidate_thresholds = [float(np.percentile(arr, p)) for p in (50, 75, 90)]

        firing_by_session = {}
        if dubai_result:
            firing_by_session["Dubai"] = firing_report("Dubai", dubai_result[0], candidate_thresholds,
                                                         len(dubai_result[0]))
        if v3_result:
            firing_by_session["v3"] = firing_report("v3", v3_result[0], candidate_thresholds,
                                                      len(v3_result[0]))

        print(f"\n=== AXLE ATTRIBUTION at chosen threshold {CHOSEN_THRESHOLD_G}g ===")
        for label, result in (("Dubai", dubai_result), ("v3", v3_result)):
            if result is None:
                continue
            by_corner, _, t, kerb_mask, channels = result
            firing = [cid for cid, insts in by_corner.items()
                      if sum(1 for _, p, _, _ in insts if p >= CHOSEN_THRESHOLD_G) >= 2]
            print(f"{label} firing corners: {sorted(firing)}")
            for cid in sorted(firing):
                attribute_axle(cid, by_corner[cid], CHOSEN_THRESHOLD_G, t, channels, wl_cfg)
