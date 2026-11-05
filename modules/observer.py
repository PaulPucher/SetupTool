# Sideslip observer of the fit-L chain: the single-track EKF, the straights
# its sensor noise is measured on, and the NIS gate that decides whether its
# beta is trusted.

import numpy as np
from scipy.stats import chi2

from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness


# Single-track EKF sideslip observer, Magic Formula tyres. Used by
# tyre_fit_auto.
# Model: Rajamani sec. 2.3/2.6, Ulsoy/Peng/Cakmakci sec. 14.1/14.3.
# EKF: Rajamani ch. 14.
#
# x = [beta, yaw_rate], u = delta_f, z = [yaw_rate, ay]. Vx scheduled per
# sample (floored at moving_speed_min_mps), not a state.
# Tyre params (B, C, mu, E per axle) and Q/R/P0 from the pass config
# (tyre_model_ekf_pacejka.<pass_id>, built per session by tyre_fit_auto).
# D is a friction coefficient (d_is_mu, fit L): the axle loads and
# the front Fx come from the state (fz_axle_f_N, fz_axle_r_N, fx_axle_f_N),
# so config plus the session's load cascade reproduce a run.
#
# Explicit Euler. State propagated through the nonlinear f(x,u); Jacobian F
# only for P. F at x_k|k, H at x_k+1|k. Tyre slopes from the analytic
# pacejka_lateral_stiffness.
#
# Divergence: |beta| > beta_hard_bound_deg only. The bound is physical
# (15 deg), not from the
# kinematic beta range -- that under-reads mid-corner and would clip the
# real signal. Re-entry: see estimate_sideslip_ekf_pacejka. Raw EKF beta is
# returned alongside the shipped one, plus hard_bound_mask (the resets).

def slip_angles(beta, r, u, Vx, a, b):
    alpha_f = u - beta - a * r / Vx
    alpha_r = -beta + b * r / Vx
    return alpha_f, alpha_r


def process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz):
    return np.array([
        [-(Cf_eff + Cr_eff) / (m * Vx), (-a * Cf_eff + b * Cr_eff) / (m * Vx ** 2) - 1.0],
        [(-a * Cf_eff + b * Cr_eff) / Iz, -(a ** 2 * Cf_eff + b ** 2 * Cr_eff) / (Iz * Vx)],
    ])


def measurement_jacobian(Cf_eff, Cr_eff, Vx, m, a, b):
    return np.array([
        [0.0, 1.0],
        [-(Cf_eff + Cr_eff) / m, (-a * Cf_eff + b * Cr_eff) / (m * Vx)],
    ])


def axle_forces(alpha_f, alpha_r, tyre, u, i, load):
    """Body-lateral axle forces and their slip slopes for one sample, the one
    force model of the prediction and the measurement update, so both
    always see the same D.
    tyre = (Bf, Cf, mu_f, Ef, Br, Cr, mu_r, Er), load = (fz_f, fz_r, fx_f)
    arrays: the peak force follows the axle load, D = mu Fz(t) (Pacejka
    Exercise 1.1); the front curve is in the tyre frame, so the body force
    adds the Fx share of the steered wheel and only the tyre-frame part,
    scaled by cos(delta), depends on the state (Goblirsch et al. 2025
    Eq. 13)."""
    Bf, Cf, Df, Ef, Br, Cr, Dr, Er = tyre
    fz_f, fz_r, fx_f = load
    Df_i = Df * float(fz_f[i])
    Dr_i = Dr * float(fz_r[i])
    cos_d, sin_d = np.cos(u), np.sin(u)
    return (pacejka_lateral_force(alpha_f, Bf, Cf, Df_i, Ef) * cos_d + float(fx_f[i]) * sin_d,
            pacejka_lateral_force(alpha_r, Br, Cr, Dr_i, Er),
            pacejka_lateral_stiffness(alpha_f, Bf, Cf, Df_i, Ef) * cos_d,
            pacejka_lateral_stiffness(alpha_r, Br, Cr, Dr_i, Er))


def estimate_sideslip_ekf_pacejka(state, params, pass_id):
    """Divergence re-entry on the hard bound only: with the process noise
    sized from the session's force-model error the filter needs no
    covariance-only reset. On |beta| beyond the
    bound: beta state to 0, yaw rate state to the measured value, P to P0.
    Keeping an out-of-range state runs away -- the curve is flat out there,
    so the measurements cannot pull it back. Zero carries no
    seed bias; the kinematic beta is never written into the state. Villano
    et al. 2021 (Meccanica, sec. 3.1) force the lateral velocity of their
    kinematic filter to zero at small yaw rate, where it is unobservable;
    using a zero reset on divergence is our adaptation of that.
    beta_with_fallback keeps its name for the callers and carries the
    post-re-entry state; beta is the raw update (a re-entry shows with
    |beta| > bound).
    """
    vp = params["vehicle"]
    se = params["stability_estimation"]
    cfg = params["tyre_model_ekf_pacejka"][pass_id]

    m = vp["mass_kg"]
    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]
    Iz = vp["yaw_inertia_kgm2"]

    tyre = (cfg["b_front"], cfg["c_front"], cfg["d_front"], cfg["e_front"],
            cfg["b_rear"], cfg["c_rear"], cfg["d_rear"], cfg["e_rear"])
    load = (state["fz_axle_f_N"], state["fz_axle_r_N"], state["fx_axle_f_N"])

    v = state["v_mps"]
    yaw_rate_meas = state["yaw_rate_radps"]
    ay_meas = state["ay_mps2"]
    delta_f = state["delta_f_rad"]
    moving = state["moving_mask"]
    sr = state["sample_rate_hz"]
    v_min = se["moving_speed_min_mps"]

    n = len(v)
    dt = 1.0 / sr

    # Q entries are per-step variances tuned on the 50 Hz grid. White process
    # noise integrated over one step adds variance in proportion to the step
    # length (first-order discretisation, Q_d = Q_c * dt), so scale by
    # dt / dt_ref: exactly 1 at the reference rate (bit-identical), half per
    # step at 100 Hz, i.e. the same variance per second. [Tier B: standard
    # discretisation, preprocessing of a tuned value]
    q_scale = cfg["q_reference_rate_hz"] / sr
    Q = np.diag([cfg["Q_beta_var"], cfg["Q_yaw_rate_var"]]) * q_scale
    R = np.diag([cfg["R_yaw_rate_var"], cfg["R_ay_var"]])
    P0 = np.diag([cfg["P0_beta_var"], cfg["P0_yaw_rate_var"]])

    beta_hard_bound_rad = np.radians(cfg["beta_hard_bound_deg"])
    eye2 = np.eye(2)

    beta_out = np.zeros(n)
    nis_out = np.full(n, np.nan)
    hard_bound_mask = np.zeros(n, dtype=bool)
    beta_with_fallback = np.zeros(n)
    innovation_out = np.full((n, 2), np.nan)
    S_diag_out = np.full((n, 2), np.nan)

    x = np.zeros(2)
    P = P0.copy()

    for i in range(n):
        if not moving[i]:
            x = np.zeros(2)
            P = P0.copy()
            beta_out[i] = 0.0
            beta_with_fallback[i] = 0.0
            continue

        Vx = max(float(v[i]), v_min)
        u = float(delta_f[i])

        beta_x, r_x = x[0], x[1]
        alpha_f, alpha_r = slip_angles(beta_x, r_x, u, Vx, a, b)

        Fy_f, Fy_r, Cf_eff, Cr_eff = axle_forces(alpha_f, alpha_r, tyre, u, i, load)

        beta_dot = (Fy_f + Fy_r) / (m * Vx) - r_x
        r_dot = (a * Fy_f - b * Fy_r) / Iz
        x_pred = x + dt * np.array([beta_dot, r_dot])

        F = process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz)
        Ad = eye2 + F * dt
        P_pred = Ad @ P @ Ad.T + Q

        beta_p, r_p = x_pred[0], x_pred[1]
        alpha_f_p, alpha_r_p = slip_angles(beta_p, r_p, u, Vx, a, b)

        Fy_f_p, Fy_r_p, Cf_eff_p, Cr_eff_p = axle_forces(alpha_f_p, alpha_r_p, tyre, u, i, load)

        h = np.array([r_p, (Fy_f_p + Fy_r_p) / m])
        H = measurement_jacobian(Cf_eff_p, Cr_eff_p, Vx, m, a, b)

        z = np.array([float(yaw_rate_meas[i]), float(ay_meas[i])])
        nu = z - h
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_upd = x_pred + K @ nu
        P_upd = (eye2 - K @ H) @ P_pred

        nis = float(nu @ np.linalg.solve(S, nu))
        nis_out[i] = nis
        innovation_out[i] = nu
        S_diag_out[i] = np.diag(S)

        bound_flag = abs(x_upd[0]) > beta_hard_bound_rad

        beta_out[i] = float(x_upd[0])

        hard_bound_mask[i] = bound_flag

        if bound_flag:
            x = np.array([0.0, float(yaw_rate_meas[i])])
            P = P0.copy()
        else:
            x = x_upd
            P = P_upd
        beta_with_fallback[i] = x[0]

    return {
        "beta": beta_out,
        "nis": nis_out,
        "hard_bound_mask": hard_bound_mask,
        "beta_with_fallback": beta_with_fallback,
        "innovation": innovation_out,
        "S_diag": S_diag_out,
    }


# Steering-defined straights of one session, where the fit-L chain
# measures the sensor noise. Same definition as
# diagnostics/inspect_kinematic_inputs.steering_straights (test cross-check).
# Thresholds come from steering and speed only, never from the IMU signals
# whose noise is measured there; centre and widths are measured per session.
# Tier B preprocessing: a segmentation rule, config-driven
# (tyre_fit_auto.l_straight_*).

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


# NIS tyre-mismatch gate: does the fitted tyre curve fit this session well
# enough to trust EKF beta? pass / warn / fail (-> kinematic beta).
# Session score = share of samples whose windowed NIS exceedance sits in band.
# Thresholds derived on the fit-L chain (14 development sessions, config
# threshold_derived_from).
#
# Score ceiling ~60-65% even for a perfect filter (small-window binomial noise).

_CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))

# A window's exceedance fraction is only read when most of it is measured;
# below half valid it is skipped rather than estimated from a few samples.
MIN_WINDOW_VALID_FRACTION = 0.5


def _score_and_skips(nis_combined_full, mask, window_samples, band_low, band_high):
    """(health score, skipped count). Score = fraction of masked samples whose
    trailing-window NIS exceedance lies in [band_low, band_high].
    nis_combined_full is unmasked -- the window looks back across mask
    gaps. Windows less than half covered by
    finite NIS are skipped. NaN on degenerate input; NaN never counts as a
    pass.
    """
    window_samples = int(window_samples)
    n = len(nis_combined_full)
    if window_samples < 1 or window_samples > n or not np.any(mask):
        return float("nan"), 0
    nis = np.asarray(nis_combined_full, dtype=float)
    # NaN NIS (no EKF update, e.g. standing) leaves both numerator and
    # denominator -- counted as a non-exceedance it would pull the fraction
    # down.
    finite = np.isfinite(nis)
    exceed = (finite & (nis > _CHI2_DF2_95)).astype(float)
    cw = np.cumsum(np.insert(exceed, 0, 0.0))
    cv = np.cumsum(np.insert(finite.astype(float), 0, 0.0))
    ends = np.arange(window_samples - 1, n)
    n_exceed = cw[ends + 1] - cw[ends + 1 - window_samples]
    n_valid = cv[ends + 1] - cv[ends + 1 - window_samples]
    covered = n_valid >= MIN_WINDOW_VALID_FRACTION * window_samples
    win_frac = np.full(n, np.nan)
    win_frac[ends[covered]] = n_exceed[covered] / n_valid[covered]
    low_coverage = np.zeros(n, dtype=bool)
    low_coverage[ends[~covered]] = True
    m = np.asarray(mask, dtype=bool)
    n_skipped = int((m & low_coverage).sum())
    in_band = (win_frac >= band_low) & (win_frac <= band_high)
    valid = m & np.isfinite(win_frac)
    if not valid.any():
        return float("nan"), n_skipped
    return float(in_band[valid].mean()), n_skipped


def classify_score(health_score, threshold_use_ekf, threshold_warn):
    """Threshold logic only, testable without an EKF run.
    NaN -> 'fail'. Both boundaries inclusive (>=).
    """
    if health_score != health_score:
        return "fail"
    if health_score >= threshold_use_ekf:
        return "pass"
    if health_score >= threshold_warn:
        return "warn"
    return "fail"


def resolve_nis_window_samples(params, sample_rate_hz):
    """nis_window_s [s] -> samples at this grid rate. No floor needed:
    min_sample_rate_hz (50 Hz) already keeps the count sane.
    """
    return int(round(params["nis_gate"]["nis_window_s"] * sample_rate_hz))


def evaluate_gate(nis_combined_full, mask, params, sample_rate_hz):
    """Returns the verdict plus every number behind it."""
    cfg = params["nis_gate"]
    window_samples = resolve_nis_window_samples(params, sample_rate_hz)
    band_low, band_high = cfg["nis_band_low"], cfg["nis_band_high"]
    threshold_use_ekf, threshold_warn = cfg["threshold_use_ekf"], cfg["threshold_warn"]

    n = len(nis_combined_full)
    masked_n = int(np.sum(np.asarray(mask, dtype=bool)))
    degenerate_reason = None
    if window_samples > n:
        degenerate_reason = f"window_samples ({window_samples}) exceeds session length ({n})"
    elif masked_n == 0:
        degenerate_reason = "mask selects zero samples"

    health_score, n_skipped = _score_and_skips(nis_combined_full, mask, window_samples, band_low, band_high)
    if degenerate_reason is None and health_score != health_score:
        degenerate_reason = "health score is NaN -- window/mask produced no valid in-band measurement"

    verdict = classify_score(health_score, threshold_use_ekf, threshold_warn)

    return {
        "verdict": verdict,
        "health_score": health_score,
        "window_samples": window_samples,
        "nis_band_low": band_low,
        "nis_band_high": band_high,
        "threshold_use_ekf": threshold_use_ekf,
        "threshold_warn": threshold_warn,
        "masked_population_n": masked_n,
        # masked samples whose window was skipped as less than half valid
        "n_windows_skipped_low_coverage": n_skipped,
        "degenerate_reason": degenerate_reason,
    }
