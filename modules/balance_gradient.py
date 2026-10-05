# Measured balance of the session: one understeer gradient K per session,
# beta-free, from steering, yaw rate, speed and lateral acceleration in quasi-
# steady cornering. Rajamani sec. 3.2 Eq. 3.23: delta = L / R + K a_y with
# 1 / R = r / v, so K is the slope of (delta - L r / v) over a_y. Level 2: a
# session measurement, read beside the per-corner verdicts -- it is the car's
# steady-state balance, not the local axle saturation a corner verdict reads.
# Lifted unchanged from the diagnostic that established it (Q-PUSH P1).

import numpy as np

from modules.stability_analysis import _butterworth_lowpass
from modules.tyre_fit_auto import _canonical_window_slice

RAD2DEG = 180.0 / np.pi
G = 9.81


def _otsu_threshold(x, nbins):
    # Otsu's between-class-variance split of |a_y|: the session's own
    # straight / corner boundary (standard segmentation, no fixed g value)
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    hist, edges = np.histogram(x, bins=nbins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    w = hist.astype(float)
    w0 = np.cumsum(w)
    w1 = w0[-1] - w0
    m0 = np.cumsum(w * centers)
    mu0 = np.divide(m0, w0, out=np.zeros_like(m0), where=w0 > 0)
    mu1 = np.divide(m0[-1] - m0, w1, out=np.zeros_like(m0), where=w1 > 0)
    return float(centers[int(np.argmax(w0 * w1 * (mu0 - mu1) ** 2))])


def estimate_understeer_gradient(state, params, laps, corners):
    """Session understeer gradient with a 95 % corner-traversal cluster
    bootstrap range, or None when no quasi-steady sample exists.

    Population: moving, off-kerb samples of analysis laps inside corner
    brackets, |r v| above the session's Otsu |a_y| split, |d r/dt| and |a_x|
    small, |a_y| below a clip. Steering zero = median road-wheel angle on the
    same laps outside corners. Traversals, not samples, are resampled: samples
    inside one corner pass are not independent.
    """
    cfg = params["understeer_gradient"]
    vp = params["vehicle"]
    t = state["time"]
    sr = state["sample_rate_hz"]
    fc = params["stability_estimation"]["cs_filter_cutoff_hz"]
    L = vp["cog_to_front_axle_m"] + vp["cog_to_rear_axle_m"]

    racing = state["moving_mask"].copy()
    if state.get("kerb_mask") is not None:
        racing &= ~state["kerb_mask"]
    on_lap = np.zeros(t.size, dtype=bool)
    lap_by = {}
    for lap in laps:
        if lap.get("is_valid_for_analysis"):
            on_lap |= (t >= lap["start_time"]) & (t <= lap["end_time"])
            lap_by[lap["lap_number"]] = lap
    racing &= on_lap

    trav = np.full(t.size, -1)
    insts = []
    for c in corners:
        lap = lap_by.get(c.get("lap_number"))
        if c.get("stable_corner_id") is None or lap is None:
            continue
        if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
            continue
        sl = _canonical_window_slice(t, state["s_m"], lap["start_time"], lap["end_time"],
                                     c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            insts.append(sl)
    insts.sort(key=lambda s: s.start)
    for j, sl in enumerate(insts):
        trav[sl] = j
    in_corner = trav >= 0

    thr = _otsu_threshold(np.abs(state["ay_mps2"][racing]), cfg["otsu_bins"])
    rf = _butterworth_lowpass(state["yaw_rate_radps"], fc, sr)
    vf = _butterworth_lowpass(state["v_mps"], fc, sr)
    axf = _butterworth_lowpass(state["ax_mps2"], fc, sr)
    af = _butterworth_lowpass(state["ay_mps2"], fc, sr)
    psidd = np.gradient(rf, t)
    clip = np.abs(state["ay_mps2"]) >= cfg["ay_clip_g"] * G
    pop = (racing & in_corner & (np.abs(rf * vf) > thr) & (np.abs(psidd) < cfg["psidd_max_radps2"])
           & (np.abs(axf) < cfg["ax_max_mps2"]) & ~clip)

    offset = float(np.median(state["delta_f_rad"][racing & ~in_corner]))
    y = _butterworth_lowpass(state["delta_f_rad"], fc, sr) - offset - L * rf / np.where(vf > 1.0, vf, np.nan)
    ok = pop & np.isfinite(y)
    if not ok.any():
        return None
    xs, ys, tr = af[ok], y[ok], trav[ok]
    coef, *_ = np.linalg.lstsq(np.column_stack([xs, np.ones(xs.size)]), ys, rcond=None)
    ids = np.unique(tr)
    by = {i: np.where(tr == i)[0] for i in ids}
    rng = np.random.default_rng(cfg["bootstrap_seed"])
    ks = []
    for _ in range(cfg["bootstrap_draws"]):
        pick = np.concatenate([by[i] for i in rng.choice(ids, size=ids.size, replace=True)])
        c, *_ = np.linalg.lstsq(np.column_stack([xs[pick], np.ones(pick.size)]), ys[pick], rcond=None)
        ks.append(c[0])
    to_deg_g = RAD2DEG * G
    return {"K_deg_per_g": float(coef[0]) * to_deg_g,
            "ci95_deg_per_g": [float(np.percentile(ks, 2.5)) * to_deg_g, float(np.percentile(ks, 97.5)) * to_deg_g],
            "n_traversals": int(ids.size), "n_samples": int(ok.sum()),
            "accuracy_level": params["accuracy_levels"]["understeer_gradient"]["level"]}


def balance_reading(balance):
    # the reading word only when the 95 % range excludes zero
    lo, hi = balance["ci95_deg_per_g"]
    if lo > 0:
        return "the car understeers"
    if hi < 0:
        return "the car oversteers"
    return "balance not resolved from neutral"
