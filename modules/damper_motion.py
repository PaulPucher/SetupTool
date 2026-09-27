# Damper motion evidence: loading / unloading / no-motion per corner,
# transient phase and wheel, from log_susp_travel_* (100 Hz).
# Dampers only act with shaft velocity, not at steady apex -- Segers ch. 11.
# Plain signal processing (derivative, threshold, validity floor).
# Channel helpers shared with wheel_loads.

import numpy as np

from modules.decision_frame import TRANSIENT_PHASES, _phase_window_indices
from modules.wheel_loads import (
    CORNERS, CORNER_AXLE, TRAVEL_CHANNEL, _interp_channel, _normalize_travel_to_mm,
    _channel_is_dead,
)


def classify_window_motion(travel_mm, t, lo, hi, kerb_mask, rate_threshold_mm_s, min_valid_fraction):
    """One phase window of one wheel -> "loading" / "unloading" /
    "no-motion", or None if under min_valid_fraction valid samples.

    travel_mm, t: full-session arrays; lo/hi index into them.
    kerb_mask: True = exclude, or None.
    Rate = median of d(travel)/dt, not an endpoint slope -- robust to one
    noisy edge sample. |rate| < rate_threshold_mm_s -> "no-motion".
    Returns (direction, rate_mm_s, valid_fraction).
    """
    window_len = hi - lo
    if window_len < 2:
        return None, None, 0.0

    seg_t = t[lo:hi]
    seg_travel = travel_mm[lo:hi]
    valid = np.isfinite(seg_travel)
    if kerb_mask is not None:
        valid = valid & ~kerb_mask[lo:hi]
    valid_fraction = float(valid.sum()) / window_len

    if valid_fraction < min_valid_fraction:
        return None, None, valid_fraction

    # gradient over valid samples only -- across a masked gap it would mix in the kerb hit
    idx = np.where(valid)[0]
    if idx.size < 2:
        return None, None, valid_fraction
    rates = np.gradient(seg_travel[idx], seg_t[idx])
    rate = float(np.median(rates))

    if abs(rate) < rate_threshold_mm_s:
        return "no-motion", rate, valid_fraction
    # Sign checked on both real sessions: front travel falls under braking
    # (corr(travel, ax) > 0) -> decreasing travel = compression = loading.
    return ("unloading" if rate > 0 else "loading"), rate, valid_fraction


def _corner_instances_by_id(corners):
    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)
    return by_corner


def build_damper_motion_evidence(corners, state, channels, aggregated, dm_cfg, wl_cfg):
    """Returns (evidence_list, summary). evidence_list has build_evidence's
    flat shape. summary (dead wheels, per-(corner, phase, wheel) evaluable
    counts) is diagnostics only -- a missing evidence item alone can't tell
    "not evaluable" from "never emitted".

    dm_cfg = decision_frame.json damper_motion block. wl_cfg = parameters.json
    wheel_loads block, used only for dead_channel_std_max_travel_mm.
    """
    if state is None or channels is None:
        return [], {"skipped": "state or channels not supplied"}

    t = state["time"]
    kerb_mask = state.get("kerb_mask")
    rate_threshold = dm_cfg["rate_threshold_mm_s"]
    min_valid_fraction = dm_cfg["min_valid_fraction"]
    dead_std_max = wl_cfg["dead_channel_std_max_travel_mm"]

    dead_wheels = set()
    travel_by_wheel = {}
    for wheel in CORNERS:
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            dead_wheels.add(wheel)  # missing/failed = same as frozen
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        if _channel_is_dead(raw_mm, dead_std_max):
            dead_wheels.add(wheel)
            continue
        travel_by_wheel[wheel] = raw_mm

    by_corner = _corner_instances_by_id(corners)
    evidence = []
    instance_counts = {}  # (cid, phase, wheel) -> {"evaluable": n, "not_evaluable": n}

    for wheel in CORNERS:
        if wheel in dead_wheels:
            continue
        travel_mm = travel_by_wheel[wheel]
        axle = CORNER_AXLE[wheel]
        for cid, instances in by_corner.items():
            for phase in TRANSIENT_PHASES:
                directions, rates, valid_fracs = [], [], []
                n_not_evaluable = 0
                n_total = 0
                for inst in instances:
                    idx = _phase_window_indices(t, inst.get("segments", {}).get(phase))
                    if idx is None:
                        continue
                    n_total += 1
                    lo, hi = idx
                    direction, rate, vf = classify_window_motion(
                        travel_mm, t, lo, hi, kerb_mask, rate_threshold, min_valid_fraction)
                    if direction is None:
                        n_not_evaluable += 1
                        continue
                    directions.append(direction)
                    rates.append(rate)
                    valid_fracs.append(vf)

                instance_counts[(cid, phase, wheel)] = {
                    "evaluable": len(directions), "not_evaluable": n_not_evaluable, "total": n_total,
                }
                if not directions or n_total == 0:
                    continue  # no evidence rather than a guess

                # majority direction; confidence = repeat fraction x evaluable
                # fraction, as in the other evidence builders
                counts = {}
                for d in directions:
                    counts[d] = counts.get(d, 0) + 1
                majority = max(counts, key=counts.get)
                repeat = counts[majority]
                confidence = round((repeat / n_total) * (len(directions) / n_total), 3)
                rep_rate = float(np.median([r for d, r in zip(directions, rates) if d == majority]))

                evidence.append({
                    "type": "damper_motion",
                    "corner": cid, "phase": phase,
                    "wheel": wheel, "axle": axle,
                    "speed_class": aggregated.get(cid, {}).get("speed_class"),
                    "verdict": None, "severity": None,
                    "direction": majority, "rate_mm_s": round(rep_rate, 2),
                    "confidence": confidence,
                    "source": f"C{cid} {phase} {wheel}: {majority} (median rate {rep_rate:+.2f} mm/s), "
                              f"{repeat}/{n_total} instances agree, {len(directions)}/{n_total} evaluable "
                              f"(threshold {rate_threshold} mm/s, min_valid_fraction {min_valid_fraction})",
                })

    summary = {
        "dead_wheels": sorted(dead_wheels),
        "instance_counts": instance_counts,
    }
    return evidence, summary
