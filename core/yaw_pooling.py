# Cross-lap yaw-rate pooling for the trace windows' yaw panel (display
# only). Method lifted unchanged from the read-only prototype
# diagnostics/inspect_yaw_pooling.py, whose header pre-registers every
# value; that script now calls this module, so the app panel and the
# diagnostic figures come from one implementation. Tier B preprocessing
# (grid resampling, zero-phase low-pass, robust noise floor, percentile
# band) -- standard techniques, not a method contribution. Nothing here
# reads config: callers pass the values (the trace dialog from the
# yaw_pooling block of parameters.json, the diagnostic from its own
# constants).

import warnings

import numpy as np

# 1.4826 * MAD estimates a Gaussian sigma; robust against the corner
# transients that leak into the straight population
MAD_TO_SIGMA = 1.4826


def lowpass(x, fc_hz, sample_rate_hz):
    """Zero-phase Butterworth low-pass (filtfilt: no lag, so the pooled
    traces stay aligned in track position)."""
    from scipy.signal import butter, filtfilt
    from modules.stability_analysis import BUTTERWORTH_ORDER
    b, a = butter(BUTTERWORTH_ORDER, fc_hz / (0.5 * sample_rate_hz), btype="low")
    return filtfilt(b, a, x)


def steering_rate(time, steer_deg, sample_rate_hz, fc_hz):
    """Steering-wheel rate (deg/s) from the angle low-passed at fc_hz;
    differentiating the raw angle would amplify quantisation noise."""
    return np.gradient(lowpass(steer_deg, fc_hz, sample_rate_hz), time)


def robust_std(x):
    x = x[np.isfinite(x)]
    return float(MAD_TO_SIGMA * np.median(np.abs(x - np.median(x))))


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # same lap-time / lap-distance intersection as modules/tyre_fit_auto
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    start_s = max(float(np.min(lap_s[finite])), bracket_start_m)
    end_s = min(float(np.max(lap_s[finite])), bracket_end_m)
    a = int(np.searchsorted(lap_s, start_s, side="left"))
    b = int(np.searchsorted(lap_s, end_s, side="right"))
    return slice(lo + a, lo + b)


def session_masks(time, s_m, moving, kerb, laps, corners):
    """Populations the pooling needs, from the parsed laps and corners.

    racing: moving, off-kerb samples inside analysis-valid laps.
    straight: racing samples outside every corner bracket -- the population
    the yaw noise floor is measured on (S0 of inspect_imu_offsets).
    traversals: [(stable_id, lap_number, slice)] for every corner instance
    on an analysis-valid lap, time-ordered.
    """
    racing = np.asarray(moving, dtype=bool).copy()
    if kerb is not None:
        racing &= ~np.asarray(kerb, dtype=bool)
    in_lap = np.zeros(time.size, dtype=bool)
    for lap in laps:
        if lap.get("is_valid_for_analysis"):
            in_lap |= (time >= lap["start_time"]) & (time <= lap["end_time"])
    racing &= in_lap

    by_number = {l["lap_number"]: l for l in laps}
    traversals = []
    for c in corners:
        sid = c.get("stable_corner_id")
        lap = by_number.get(c.get("lap_number"))
        if sid is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
            continue
        sl = _canonical_window_slice(time, s_m, lap["start_time"], lap["end_time"],
                                     c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            traversals.append((sid, c["lap_number"], sl))
    traversals.sort(key=lambda x: x[2].start)

    in_corner = np.zeros(time.size, dtype=bool)
    for _, _, sl in traversals:
        in_corner[sl] = True
    return {"racing": racing, "straight": racing & ~in_corner, "traversals": traversals}


def _runs(mask):
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def pool_yaw_rate(time, s_m, yaw_rate_degps, sample_rate_hz, racing, straight, kerb, traversals, *,
                  fc_hz, trend_hz, grid_m, gap_m, envelope_percentiles, min_laps, margin_sigma,
                  min_run_pts):
    """Pool yaw rate per corner across analysis laps and flag laps outside
    the cross-lap envelope.

    Each corner traversal is resampled on a common track-position grid, so
    laps compare at the same place on track rather than the same time.
    Per grid point the pooled median and the percentile envelope describe
    what this car did here on the other laps. The margin m = margin_sigma x
    robust std of the filtered yaw rate's deviation from its own trend_hz
    trend on straights: the noise floor of the plotted signal in this
    session, so a flag means "beyond the lap spread by more than the
    signal's own noise", not a vehicle-dynamics threshold.
    Flag = at least min_run_pts consecutive grid points beyond the envelope
    by more than m. EXCESS = more rotation than the envelope in the turn
    direction (sign of the pooled median), DEFICIT = less.

    Returns {"sigma", "margin", "yaw_filt", "corners": {stable_id: {...}}};
    per corner "grid", per-lap rows "Y" (yaw), "T" (time on grid), "K"
    (grid point next to a kerb sample, for the display bridge), "laps",
    "med", "p_lo", "p_hi", "flags" [{"lap", "kind", "i0", "i1", "s_on",
    "peak"}], i0/i1 grid indices of the run (i1 exclusive).
    Inputs are never modified.
    """
    pct_lo, pct_hi = envelope_percentiles
    r_f = lowpass(yaw_rate_degps, fc_hz, sample_rate_hz)
    trend = lowpass(yaw_rate_degps, trend_hz, sample_rate_hz)
    sigma = robust_std((r_f - trend)[straight])
    m = margin_sigma * sigma
    kerb = np.asarray(kerb, dtype=bool) if kerb is not None else np.zeros(time.size, dtype=bool)

    by_corner = {}
    for sid, lap, sl in traversals:
        by_corner.setdefault(sid, []).append((lap, sl))
    out = {"sigma": sigma, "margin": m, "yaw_filt": r_f, "corners": {}}
    for sid, trs in sorted(by_corner.items()):
        s_lo = min(float(np.nanmin(s_m[sl])) for _, sl in trs)
        s_hi = max(float(np.nanmax(s_m[sl])) for _, sl in trs)
        grid = np.arange(np.floor(s_lo), np.ceil(s_hi) + grid_m, grid_m)
        Y, TT, KK, LAPS = [], [], [], []
        for lap, sl in trs:
            idx_all = np.arange(sl.start, sl.stop)
            idx = idx_all[racing[idx_all] & ~kerb[idx_all]]
            s = s_m[idx]
            ok = np.isfinite(s)
            idx, s = idx[ok], s[ok]
            if s.size < 5:
                continue
            o = np.argsort(s)
            idx, s = idx[o], s[o]
            y = np.interp(grid, s, r_f[idx])
            tg = np.interp(grid, s, time[idx])
            # gaps wider than gap_m (kerb or not-moving samples dropped) stay
            # NaN rather than being drawn as measured
            pos = np.clip(np.searchsorted(s, grid), 1, s.size - 1)
            near = np.minimum(np.abs(grid - s[pos - 1]), np.abs(grid - s[pos]))
            bad = (near > gap_m) | (grid < s[0]) | (grid > s[-1])
            y[bad] = np.nan
            tg[bad] = np.nan
            # kerb position on the grid from ALL traversal samples, so the
            # display can tell a kerb hole from a not-moving one
            fa = idx_all[np.isfinite(s_m[idx_all])]
            fa = fa[np.argsort(s_m[fa])]
            kg = np.interp(grid, s_m[fa], kerb[fa].astype(float)) > 0 if fa.size else np.zeros(grid.size, bool)
            Y.append(y); TT.append(tg); KK.append(kg); LAPS.append(lap)
        if len(Y) < min_laps:
            continue
        Y, TT, KK = np.array(Y), np.array(TT), np.array(KK)
        nlap = np.sum(np.isfinite(Y), axis=0)
        # all-NaN grid columns (no lap reaches that point) are expected
        with np.errstate(all="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            med = np.nanmedian(Y, axis=0)
            p_lo = np.nanpercentile(Y, pct_lo, axis=0)
            p_hi = np.nanpercentile(Y, pct_hi, axis=0)
        env_ok = nlap >= min_laps
        sgn = np.sign(med)
        outer = np.where(sgn > 0, p_hi, p_lo)
        inner = np.where(sgn > 0, p_lo, p_hi)
        flags = []
        for j, lap in enumerate(LAPS):
            y = Y[j]
            exc = env_ok & np.isfinite(y) & (sgn * (y - outer) > m)
            dfc = env_ok & np.isfinite(y) & (sgn * (inner - y) > m)
            for kind, mask in (("EXCESS", exc), ("DEFICIT", dfc)):
                for a, b in _runs(mask):
                    if b - a < min_run_pts:
                        continue
                    if kind == "EXCESS":
                        peak = float(np.max(sgn[a:b] * (y[a:b] - outer[a:b])))
                    else:
                        peak = float(np.max(sgn[a:b] * (inner[a:b] - y[a:b])))
                    flags.append({"lap": lap, "kind": kind, "i0": int(a), "i1": int(b),
                                  "s_on": float(grid[a]), "peak": peak})
        out["corners"][sid] = {"grid": grid, "Y": Y, "T": TT, "K": KK, "laps": LAPS, "med": med,
                               "p_lo": p_lo, "p_hi": p_hi, "flags": flags}
    return out


def build_panel_data(state, laps, corners, cfg):
    """Everything the trace windows' yaw and steering-rate panels draw, for
    one session. cfg = the yaw_pooling config values (the caller
    reads config). The envelope pools ALL analysis laps -- a fixed session
    reference; which laps are drawn is the dialog's business (author
    ruling 2026-09-30). None when a required channel is missing.
    """
    t = state.get("time")
    s_m = state.get("s_m")
    yaw = state.get("yaw_rate_radps")
    if t is None or s_m is None or yaw is None or state.get("moving_mask") is None:
        return None
    sr = state["sample_rate_hz"]
    masks = session_masks(t, s_m, state["moving_mask"], state.get("kerb_mask"), laps, corners)
    pooled = pool_yaw_rate(
        t, s_m, np.degrees(yaw), sr, masks["racing"], masks["straight"], state.get("kerb_mask"),
        masks["traversals"],
        fc_hz=cfg["lowpass_hz"], trend_hz=cfg["noise_trend_hz"], grid_m=cfg["grid_m"],
        gap_m=cfg["max_gap_m"], envelope_percentiles=tuple(cfg["envelope_percentiles"]),
        min_laps=cfg["min_laps"], margin_sigma=cfg["margin_sigma"], min_run_pts=cfg["min_run_points"],
    )
    steer = state.get("steer_sw_rad")
    pooled["steer_rate"] = (steering_rate(t, np.degrees(steer), sr, cfg["lowpass_hz"])
                            if steer is not None else None)
    pooled["n_analysis_laps"] = sum(1 for l in laps if l.get("is_valid_for_analysis"))
    return pooled
