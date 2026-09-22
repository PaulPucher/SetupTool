# Damper motion-state evidence, FRAME DEPTH PROGRAMME Step 2 (PLAN.md
# "FRAME DEPTH PROGRAMME", 2026-09-22). Per-corner, per-TRANSIENT-phase
# loading/unloading classification from the 100 Hz log_susp_travel_*
# channels -- the evidence source Step 1's condition schema (modules.
# decision_frame.evaluate_conditions, evidence_corroboration type) was
# built to consume, and the missing ingredient docs/segers_bridge_
# review.md's C11-2 named as blocking a front-LS-rebound/turn-in-
# understeer bridge (method pointer: thesis_notes.md "Frame depth
# programme..." entries; docs/segers_bridge_review.md C11-1/C11-2 --
# Segers ch.11, dampers develop force only while the shaft has velocity,
# never at steady-state apex cornering). Tier B throughout: standard
# signal engineering (derivative, threshold, validity floor), never
# presented as a vehicle-dynamics method of its own. No Qt.
#
# Reuses modules.wheel_loads' own channel-access/dead-channel/unit-
# normalisation helpers directly (same channels, same known dead-channel
# case -- Dubai RR travel pot, std~0.1mm) rather than a second,
# independently-maintained copy of that logic.

import numpy as np

from modules.decision_frame import TRANSIENT_PHASES, _phase_window_indices
from modules.wheel_loads import (
    CORNERS, CORNER_AXLE, TRAVEL_CHANNEL, _interp_channel, _normalize_travel_to_mm,
    _channel_is_dead,
)


def classify_window_motion(travel_mm, t, lo, hi, kerb_mask, rate_threshold_mm_s, min_valid_fraction):
    """Pure function -- the testable core unit. Classifies ONE phase-
    window instance of one corner's own travel trace as "loading" /
    "unloading" / "no-motion", or None if the window's own signal is too
    sparse to trust (below min_valid_fraction of real, non-kerb-masked
    samples).

    `travel_mm`/`t` are the FULL-SESSION arrays (already unit-normalised);
    lo/hi are this window's own index bounds into them (modules.decision_
    frame._phase_window_indices' own output shape). `kerb_mask` is state's
    own full-session boolean array (True = kerb-affected, to EXCLUDE) or
    None. Returns (direction_or_None, rate_mm_s_or_None, valid_fraction).

    Rate = the MEDIAN of the windowed travel's own first derivative
    (np.gradient against time, not a simple endpoint slope -- robust to a
    single noisy sample at either edge of a short window). direction is
    "no-motion" when |rate| stays below rate_threshold_mm_s (the noise-
    vs-real-motion floor, config-derived from both sessions' own rate
    distributions -- see config/decision_frame.json damper_motion block),
    "loading"/"unloading" by SIGN once the empirical sign-convention check
    (thesis_notes.md) has determined which sign corresponds to which.
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

    # Kerb-masked/NaN samples are excluded from the RATE computation (not
    # just counted) by working on the valid-only sub-sequence -- a
    # gradient across a masked gap would otherwise blend a real motion
    # rate with a kerb-impact artifact.
    idx = np.where(valid)[0]
    if idx.size < 2:
        return None, None, valid_fraction
    rates = np.gradient(seg_travel[idx], seg_t[idx])
    rate = float(np.median(rates))

    if abs(rate) < rate_threshold_mm_s:
        return "no-motion", rate, valid_fraction
    # SIGN CONVENTION (thesis_notes.md "Damper motion sign-convention
    # check", empirically verified on BOTH real sessions, not assumed):
    # corr(front-axle travel, ax) is POSITIVE on both Dubai (+0.49) and v3
    # (+0.20) under braking -- travel DECREASES as braking gets harder
    # (ax more negative), and harder braking physically COMPRESSES the
    # front axle. Therefore DECREASING log_susp_travel_* = compression =
    # "loading"; INCREASING = extension = "unloading". If a future
    # session's own check ever disagrees, that entry is the one to
    # revisit, not this line silently.
    return ("unloading" if rate > 0 else "loading"), rate, valid_fraction


def _corner_instances_by_id(corners):
    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)
    return by_corner


def build_damper_motion_evidence(corners, state, channels, aggregated, dm_cfg, wl_cfg):
    """Returns (evidence_list, summary). `evidence_list` is build_evidence's
    own flat shape (appended directly, same as every other _build_*_
    evidence function). `summary` is a diagnostics-only dict (per-wheel
    dead-channel flags, per-(corner,phase,wheel) evaluable/not-evaluable
    instance counts) -- build_evidence itself discards it (matching the
    fixed interface every other evidence builder already has); the real-
    session verification script (Step 2 item 8) uses it directly to report
    "Dubai RR not-evaluable" precisely, since an evidence item's own
    ABSENCE is ambiguous on its own (never emitted at all vs. genuinely
    evaluated as no-motion) without this side channel.

    `dm_cfg` is config/decision_frame.json's own damper_motion block
    (rate_threshold_mm_s, min_valid_fraction). `wl_cfg` is config/
    parameters.json's own wheel_loads block, reused ONLY for its existing
    dead_channel_std_max_travel_mm -- the exact same channel, the exact
    same known-frozen case (Dubai RR) that value was already derived from
    (modules.wheel_loads._channel_is_dead's own docstring), not a second,
    duplicate number.
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
            dead_wheels.add(wheel)  # missing/failed channel -- same not-evaluable treatment as a frozen one
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
                    continue  # nothing usable this corner/phase/wheel -- no evidence item, not a guess

                # Majority-direction aggregation, same repeat/total-style
                # confidence formula every other evidence builder in this
                # frame already uses (repeat_fraction x valid_fraction) --
                # no new confidence formula introduced here.
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
