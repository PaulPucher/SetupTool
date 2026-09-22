# DIAGNOSTIC (read-only, [keep-reproduces]): FRAME DEPTH PROGRAMME Step 2
# groundwork -- two real-data questions that must be answered from data,
# never assumed, before modules/damper_motion.py's classification logic
# can be trusted:
#
# (1) SIGN CONVENTION: does a positive log_susp_travel_* rate mean
#     compression (loading) or extension (unloading)? Checked the same
#     way the existing ARB sign-convention check was (thesis_notes.md
#     "Damper package"): correlate FRONT AXLE travel against braking ax
#     on real data. Under braking, weight transfers forward -> the front
#     axle physically COMPRESSES. If travel increases (more positive) as
#     braking gets harder (ax more negative), positive travel = compression.
#
# (2) MOTION-VS-NOISE RATE THRESHOLD: derived from both sessions' own
#     travel-RATE distributions, comparing a STRAIGHT-LINE reference
#     population (|ax|<0.5 & |ay|<0.5 -- REUSING the exact mask this
#     project's own Fz-integration/aero work already established as its
#     "nothing dynamic happening" convention, not a new one invented
#     here) against the entry/exit TRANSIENT phases where real motion is
#     expected -- percentile basis reported explicitly, not a round/
#     guessed number. apex_3 was tried first and abandoned: BOTH real
#     sessions' own apex_3 phase segments are too narrow/instantaneous to
#     ever satisfy _phase_window_indices' own >=2-sample requirement (n=0
#     rate samples collected on both sessions) -- a genuine property of
#     this project's phase segmentation, not a bug in this script, and
#     the reason the reference population changed from the originally
#     planned "apex_3 vs transient" to "straight-line vs transient".

import numpy as np

from diagnostics.inspect_frame_stage2_parity import run_full_pipeline, DUBAI_FILE, V3_FILE
from modules.decision_frame import TRANSIENT_PHASES, _phase_window_indices
from modules.stability_analysis import load_parameters
from modules.wheel_loads import (
    CORNERS, TRAVEL_CHANNEL, _interp_channel, _normalize_travel_to_mm, _channel_is_dead,
)

BRAKING_AX_THRESHOLD_MPS2 = -1.0  # same moderate-braking gate used elsewhere in this project's diagnostics


def _corner_instances_by_id(corners):
    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)
    return by_corner


def sign_check(pipe, label):
    print(f"\n--- Sign check: {label} ---")
    state = pipe["state"]
    channels = pipe["channels"]
    t = state["time"]
    ax = state["ax_mps2"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    mask = moving & (ax < BRAKING_AX_THRESHOLD_MPS2)
    if kerb_mask is not None:
        mask = mask & ~kerb_mask

    front_travels = []
    for wheel in ("fl", "fr"):
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            print(f"  {wheel}: channel unavailable, skipped")
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        wl_cfg = load_parameters()["wheel_loads"]
        if _channel_is_dead(raw_mm, wl_cfg["dead_channel_std_max_travel_mm"]):
            print(f"  {wheel}: DEAD CHANNEL, skipped")
            continue
        front_travels.append(raw_mm)

    if not front_travels:
        print("  no usable front-axle travel channel this session")
        return None

    travel_front = np.mean(front_travels, axis=0)
    n = int(mask.sum())
    if n < 10:
        print(f"  only {n} braking samples (ax < {BRAKING_AX_THRESHOLD_MPS2}) -- too few to trust")
        return None
    corr = float(np.corrcoef(travel_front[mask], ax[mask])[0, 1])
    print(f"  n={n} braking samples (ax<{BRAKING_AX_THRESHOLD_MPS2}, moving, kerb-excluded)")
    print(f"  corr(travel_front, ax) = {corr:+.4f}")
    interpretation = ("NEGATIVE correlation: travel INCREASES as ax DECREASES (harder braking) "
                       "-> positive travel = COMPRESSION = loading" if corr < 0 else
                       "POSITIVE correlation: travel DECREASES as ax DECREASES (harder braking) "
                       "-> positive travel = EXTENSION = unloading")
    print(f"  interpretation: {interpretation}")
    return corr


STRAIGHT_LINE_AX_MAX = 0.5  # reused verbatim from the Fz-integration/aero straight-line mask precedent
STRAIGHT_LINE_AY_MAX = 0.5


def rate_distributions(pipe, label):
    print(f"\n--- Rate distribution: {label} ---")
    state = pipe["state"]
    channels = pipe["channels"]
    corners = pipe["corners"]
    t = state["time"]
    ax_g = state["ax_mps2"] / 9.81
    ay_g = state["ay_mps2"] / 9.81
    kerb_mask = state.get("kerb_mask")
    straight_mask = (np.abs(ax_g) < STRAIGHT_LINE_AX_MAX) & (np.abs(ay_g) < STRAIGHT_LINE_AY_MAX)
    if kerb_mask is not None:
        straight_mask = straight_mask & ~kerb_mask
    wl_cfg = load_parameters()["wheel_loads"]

    by_corner = _corner_instances_by_id(corners)
    straight_rates = []
    transient_rates = []
    valid_fractions = []

    for wheel in CORNERS:
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        if _channel_is_dead(raw_mm, wl_cfg["dead_channel_std_max_travel_mm"]):
            print(f"  {wheel}: DEAD CHANNEL, excluded from rate distribution")
            continue

        # Straight-line reference population: full-session rolling windows
        # (0.5s, matching a typical phase-window duration) inside the
        # straight-line mask, same rate computation as the production
        # classifier (median gradient over valid, non-kerb-masked samples).
        window_samples = max(2, int(round(0.5 / np.median(np.diff(t)))))
        for start in range(0, len(t) - window_samples, window_samples):
            end = start + window_samples
            if not straight_mask[start:end].all():
                continue
            seg_t, seg_travel = t[start:end], raw_mm[start:end]
            valid = np.isfinite(seg_travel)
            if valid.sum() < 2:
                continue
            vidx = np.where(valid)[0]
            rate = float(np.median(np.gradient(seg_travel[vidx], seg_t[vidx])))
            straight_rates.append(abs(rate))

        for cid, instances in by_corner.items():
            for phase in TRANSIENT_PHASES:
                for inst in instances:
                    idx = _phase_window_indices(t, inst.get("segments", {}).get(phase))
                    if idx is None:
                        continue
                    lo, hi = idx
                    if hi - lo < 2:
                        continue
                    seg_t, seg_travel = t[lo:hi], raw_mm[lo:hi]
                    valid = np.isfinite(seg_travel)
                    if kerb_mask is not None:
                        valid = valid & ~kerb_mask[lo:hi]
                    vf = float(valid.sum()) / (hi - lo)
                    valid_fractions.append(vf)
                    vidx = np.where(valid)[0]
                    if vidx.size < 2:
                        continue
                    rate = float(np.median(np.gradient(seg_travel[vidx], seg_t[vidx])))
                    transient_rates.append(abs(rate))

    def _pct(arr, p):
        return float(np.percentile(arr, p)) if arr else None

    print(f"  straight-line (|ax|<{STRAIGHT_LINE_AX_MAX},|ay|<{STRAIGHT_LINE_AY_MAX}) |rate| "
          f"(n={len(straight_rates)}): p50={_pct(straight_rates,50)}, p75={_pct(straight_rates,75)}, "
          f"p90={_pct(straight_rates,90)}, p95={_pct(straight_rates,95)}")
    print(f"  transient |rate| (n={len(transient_rates)}): "
          f"p10={_pct(transient_rates,10)}, p25={_pct(transient_rates,25)}, p50={_pct(transient_rates,50)}")
    print(f"  window valid_fraction (n={len(valid_fractions)}): "
          f"p5={_pct(valid_fractions,5)}, p10={_pct(valid_fractions,10)}, p25={_pct(valid_fractions,25)}, "
          f"p50={_pct(valid_fractions,50)}")

    return {
        "straight_rates": straight_rates, "transient_rates": transient_rates, "valid_fractions": valid_fractions,
    }


def main():
    results = {}
    for raw_file, label in [(DUBAI_FILE, "Dubai"), (V3_FILE, "v3")]:
        pipe = run_full_pipeline(raw_file)
        corr = sign_check(pipe, label)
        dist = rate_distributions(pipe, label)
        results[label] = {"corr": corr, **dist}

    print(f"\n{'='*70}\nPOOLED (both sessions)\n{'='*70}")
    all_straight = results["Dubai"]["straight_rates"] + results["v3"]["straight_rates"]
    all_transient = results["Dubai"]["transient_rates"] + results["v3"]["transient_rates"]
    all_vf = results["Dubai"]["valid_fractions"] + results["v3"]["valid_fractions"]
    print(f"straight-line |rate| pooled (n={len(all_straight)}): p50={np.percentile(all_straight,50):.3f}, "
          f"p75={np.percentile(all_straight,75):.3f}, p90={np.percentile(all_straight,90):.3f}, "
          f"p95={np.percentile(all_straight,95):.3f} mm/s")
    print(f"transient |rate| pooled (n={len(all_transient)}): p10={np.percentile(all_transient,10):.3f}, "
          f"p25={np.percentile(all_transient,25):.3f}, p50={np.percentile(all_transient,50):.3f} mm/s")
    print(f"window valid_fraction pooled (n={len(all_vf)}): p5={np.percentile(all_vf,5):.3f}, "
          f"p10={np.percentile(all_vf,10):.3f}, p25={np.percentile(all_vf,25):.3f} mm/s")
    print(f"\nSign check: Dubai corr={results['Dubai']['corr']}, v3 corr={results['v3']['corr']}")
    return results


if __name__ == "__main__":
    main()
