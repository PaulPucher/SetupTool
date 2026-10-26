# Steering-defined straights of one session, where the fit-L chain
# measures the sensor noise. Same definition as
# diagnostics/inspect_kinematic_inputs.steering_straights (test cross-check).
# Thresholds come from steering and speed only, never from the IMU signals
# whose noise is measured there; centre and widths are measured per session.
# Tier B preprocessing: a segmentation rule, config-driven
# (tyre_fit_auto.l_straight_*).

import numpy as np


def _robust_sigma(x):
    x = np.asarray(x, dtype=float)
    return 1.4826 * float(np.median(np.abs(x - np.median(x))))


def _runs(mask):
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def steering_straights(state, laps, cfg):
    """Straight segments (start, stop) on analysis laps, outside the kerb mask
    and smear zone, plus their mask. Speed at least the 25th percentile of
    analysis-lap speed (floored to 5 m/s); steering angle within a_thr of the
    session's straight-line centre (mode of the angle), steering rate within
    r_thr (both l_straight_sigma_factor x robust sigma of the straight peak);
    minimum duration = the longest candidate that still leaves
    l_straight_d_min_segments_per_lap segments on every analysis lap."""
    t, sr, v = state["time"], state["sample_rate_hz"], state["v_mps"]
    sw = np.degrees(state["steer_sw_rad"])
    valid = [l for l in laps if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in valid:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    # no usable az -> no kerb mask (None); exclude nothing, as the rest of
    # the chain does, instead of failing the whole fit
    kerb = np.zeros(t.size, dtype=bool)
    for m in (state["kerb_mask"], state["kerb_smear_zone"]):
        if m is not None:
            kerb |= m
    bin_deg, k = cfg["l_straight_hist_bin_deg"], cfg["l_straight_sigma_factor"]
    an = lap_id >= 0
    v_min = float(np.floor(np.percentile(v[an], 25) / 5.0) * 5.0)
    fast = an & (v >= v_min) & (np.abs(sw) < 20)
    h, e = np.histogram(sw[fast], bins=np.arange(-20, 20 + bin_deg / 2, bin_deg))
    centres = 0.5 * (e[1:] + e[:-1])
    centre = float(centres[np.argmax(h)])
    peak = h >= h.max() / 2
    lo, hi = centres[peak].min(), centres[peak].max()
    a_thr = round(k * _robust_sigma(sw[fast & (sw >= lo) & (sw <= hi)]), 1)
    n_avg = max(1, int(cfg["l_straight_rate_smooth_s"] * sr))
    rate = np.gradient(np.convolve(sw, np.ones(n_avg) / n_avg, mode="same"), 1.0 / sr)
    near = fast & (np.abs(sw - centre) < a_thr)
    r_thr = round(k * _robust_sigma(rate[near]), 1)
    base = an & ~kerb & (v >= v_min) & (np.abs(sw - centre) < a_thr) & (np.abs(rate) < r_thr)

    def segments(d_min):
        return [(a, b) for a, b in _runs(base) if (b - a) / sr >= d_min and lap_id[a] == lap_id[b - 1]]

    cands = cfg["l_straight_d_min_candidates_s"]
    ok = []
    for d in cands:
        segs = segments(d)
        per_lap = [sum(1 for a, _b in segs if lap_id[a] == l["lap_number"]) for l in valid]
        if (min(per_lap) if per_lap else 0) >= cfg["l_straight_d_min_segments_per_lap"]:
            ok.append(d)
    d_min = max(ok) if ok else min(cands)
    segs = segments(d_min)
    mask = np.zeros(t.size, dtype=bool)
    for a, b in segs:
        mask[a:b] = True
    return {"segs": segs, "mask": mask, "v_min": v_min, "centre": centre, "a_thr": a_thr, "r_thr": r_thr,
            "d_min": d_min}
