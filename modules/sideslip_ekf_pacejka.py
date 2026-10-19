# Single-track EKF sideslip observer, Magic Formula tyres. Used by
# tyre_fit_auto.
# Model: Rajamani sec. 2.3/2.6, Ulsoy/Peng/Cakmakci sec. 14.1/14.3.
# EKF: Rajamani ch. 14 (chapter confirmed for the 2nd ed.; edition and page
# still to verify).
#
# x = [beta, yaw_rate], u = delta_f, z = [yaw_rate, ay]. Vx scheduled per
# sample (floored at moving_speed_min_mps), not a state.
# Tyre params (B, C, D, E per axle) and Q/R/P0 all from the pass config
# (tyre_model_ekf_pacejka.<pass_id>, built per session by tyre_fit_auto) --
# a run is reproducible from that config alone. Exception: with d_is_mu
# (fit L, Q-FIT-LIT) D is a friction coefficient and the axle loads and the
# front Fx come from the state as well (fz_axle_f_N, fz_axle_r_N,
# fx_axle_f_N), so config plus the session's load cascade reproduce a run.
#
# Explicit Euler. State propagated through the nonlinear f(x,u); Jacobian F
# only for P. F at x_k|k, H at x_k+1|k. Tyre slopes from the analytic
# pacejka_lateral_stiffness.
#
# Divergence: |beta| > beta_hard_bound_deg, plus a NIS trigger -- the
# windowed NIS vs chi^2 bound (default), the change detector on NIS
# (divergence_trigger "change_detector"), or none ("hard_bound_only", the
# fit-L default since 2026-10-19). Bound is physical
# (15 deg), not from the kinematic beta range -- that under-reads mid-corner
# and would clip the real signal. Re-entry: see
# estimate_sideslip_ekf_pacejka. Raw EKF beta returned alongside, plus
# diverged_mask (every reset) and hard_bound_mask (hard-bound resets only).
# (Method statement moved here from the removed Dugoff observer,
# 2026-10-17; the two shared it apart from tyre model and re-entry.)

import numpy as np

from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness


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


def axle_forces(alpha_f, alpha_r, tyre, u, i, load=None):
    """Body-lateral axle forces and their slip slopes for one sample, the one
    force model of the prediction and the measurement update (a split
    evaluation once left the update on the wrong D, Q-FIT-LIT 2026-10-18).
    tyre = (Bf, Cf, Df, Ef, Br, Cr, Dr, Er). load None: constant D. load =
    (fz_f, fz_r, fx_f) arrays: D is mu and the peak force follows the axle
    load, D = mu Fz(t) (Pacejka Exercise 1.1); the front curve is in the tyre
    frame, so the body force adds the Fx share of the steered wheel and only
    the tyre-frame part, scaled by cos(delta), depends on the state
    (Goblirsch et al. 2025 Eq. 13)."""
    Bf, Cf, Df, Ef, Br, Cr, Dr, Er = tyre
    if load is None:
        return (pacejka_lateral_force(alpha_f, Bf, Cf, Df, Ef), pacejka_lateral_force(alpha_r, Br, Cr, Dr, Er),
                pacejka_lateral_stiffness(alpha_f, Bf, Cf, Df, Ef), pacejka_lateral_stiffness(alpha_r, Br, Cr, Dr, Er))
    fz_f, fz_r, fx_f = load
    Df_i = Df * float(fz_f[i])
    Dr_i = Dr * float(fz_r[i])
    cos_d, sin_d = np.cos(u), np.sin(u)
    return (pacejka_lateral_force(alpha_f, Bf, Cf, Df_i, Ef) * cos_d + float(fx_f[i]) * sin_d,
            pacejka_lateral_force(alpha_r, Br, Cr, Dr_i, Er),
            pacejka_lateral_stiffness(alpha_f, Bf, Cf, Df_i, Ef) * cos_d,
            pacejka_lateral_stiffness(alpha_r, Br, Cr, Dr_i, Er))


def change_detector_step(a_prev, nis, drift, threshold):
    """One step of the change detector (Rajamani 2nd ed. sec. 14.4.2) on
    NIS: returns the new statistic and whether it is in alarm."""
    a = max(a_prev + nis - drift, 0.0)
    return a, a > threshold


def estimate_sideslip_ekf_pacejka(state, params, pass_id, nis_window_reset=True):
    """Divergence re-entry, two cases (author rulings 2026-10-07). The
    kinematic beta is never written into the state or the output any more --
    it carried the washout's corner bias into E (round 3).
    - NIS trigger (state in range, measurements disagree): keep the state,
      reset P to P0 so the next samples lean on the measurements again. A
      project rule (Tier B). Related, not the same: Rajamani 2nd ed. sec.
      14.4.2 raises the covariance of an RLS estimator by a constant factor
      while a change detector is in alarm; Goblirsch et al. 2024 (IROS,
      sec. III-D) adapts the measurement covariance after a signal dropout.
      Neither uses a NIS window or a reset to P0.
    - Hard-bound divergence (|beta| beyond the bound): beta state to 0, yaw
      rate state to the measured value, P to P0. Keeping an out-of-range
      state runs away -- the curve is flat out there, so the measurements
      cannot pull it back (phase-1 stop). Zero carries no seed bias. Villano
      et al. 2021 (Meccanica, sec. 3.1) force the lateral velocity of their
      kinematic filter to zero at small yaw rate, where it is unobservable;
      using a zero reset on divergence is our adaptation of that.
    Hard bound takes precedence when both fire. beta_with_fallback keeps its
    name for the callers and carries the post-re-entry state; beta is the raw
    update (a hard-bound re-entry shows as diverged with |beta| > bound).

    NIS trigger, cfg divergence_trigger (absent -> "nis_window"):
    - "nis_window": share of NIS > nis_chi2_bound over nis_window_s above
      nis_flag_fraction (placeholder values, never validated).
    - "change_detector" (fit L): the change detector of Rajamani 2nd ed.
      sec. 14.4.2 (Eqs. 14.24-14.25, after Gustafsson 2000) on the combined
      NIS, a_t = max(a_{t-1} + NIS_t - cd_drift, 0), alarm when a_t >
      cd_threshold. cd_drift = 2 is E[NIS] for two measurements, so a
      consistent filter does not accumulate. a_t returns to 0 after any
      reset: the reset restarts P, so the accumulated evidence no longer
      describes the current covariance, and without the return every sample
      while a_t stays above h would reset P again (a one-shot reset replaces
      Rajamani's boost that lasts while the alarm holds).
    - "hard_bound_only" (fit L default, author ruling 2026-10-19): no NIS
      trigger; with the process noise sized from the session's force-model
      error (Q-UPPER) the filter needs no covariance reset, only the
      hard-bound re-entry stays.
    nis_window_reset=False (diagnostics only, Q-NIS-RESET 2026-10-19) skips
    the NIS-trigger reset so its effect on the CS can be isolated; the
    hard-bound reset stays, and diverged_mask then holds hard-bound resets
    only. hard_bound_mask always holds the hard-bound resets.
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
    # fit L (d_is_mu): D is mu, loads and front Fx from the state, see
    # axle_forces; flag absent -> constant D
    load = None
    if cfg.get("d_is_mu", False):
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
    # divergence window is a duration; the old 20 samples meant 0.4 s at 50 Hz
    nis_window = int(round(cfg["nis_window_s"] * sr))
    nis_chi2_bound = cfg["nis_chi2_bound"]
    nis_flag_fraction = cfg["nis_flag_fraction"]
    trigger = cfg.get("divergence_trigger", "nis_window")
    use_cd = trigger == "change_detector"
    hb_only = trigger == "hard_bound_only"
    if use_cd:
        cd_drift, cd_h = cfg["cd_drift"], cfg["cd_threshold"]
    cd_a = 0.0
    eye2 = np.eye(2)

    beta_out = np.zeros(n)
    yaw_rate_state_out = np.zeros(n)
    nis_out = np.full(n, np.nan)
    diverged_mask = np.zeros(n, dtype=bool)
    hard_bound_mask = np.zeros(n, dtype=bool)
    beta_with_fallback = np.zeros(n)
    innovation_out = np.full((n, 2), np.nan)
    S_diag_out = np.full((n, 2), np.nan)
    K_ay_out = np.full((n, 2), np.nan)

    x = np.zeros(2)
    P = P0.copy()
    nis_recent = []

    for i in range(n):
        if not moving[i]:
            x = np.zeros(2)
            P = P0.copy()
            nis_recent.clear()
            cd_a = 0.0
            beta_out[i] = 0.0
            yaw_rate_state_out[i] = 0.0
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
        K_ay_out[i] = K[:, 1]

        if hb_only:
            window_flag = False
        elif use_cd:
            cd_a, window_flag = change_detector_step(cd_a, nis, cd_drift, cd_h)
        else:
            nis_recent.append(nis > nis_chi2_bound)
            if len(nis_recent) > nis_window:
                nis_recent.pop(0)
            window_flag = (len(nis_recent) == nis_window
                            and (sum(nis_recent) / nis_window) > nis_flag_fraction)
        bound_flag = abs(x_upd[0]) > beta_hard_bound_rad

        raw_beta, raw_r = float(x_upd[0]), float(x_upd[1])
        beta_out[i] = raw_beta
        yaw_rate_state_out[i] = raw_r

        diverged = (window_flag and nis_window_reset) or bound_flag
        diverged_mask[i] = diverged
        hard_bound_mask[i] = bound_flag

        if bound_flag:
            x = np.array([0.0, float(yaw_rate_meas[i])])
        else:
            x = x_upd
        if diverged:
            P = P0.copy()
            nis_recent.clear()
            cd_a = 0.0
        else:
            P = P_upd
        beta_with_fallback[i] = x[0]

    return {
        "beta": beta_out,
        "yaw_rate_state": yaw_rate_state_out,
        "nis": nis_out,
        "diverged_mask": diverged_mask,
        "hard_bound_mask": hard_bound_mask,
        "beta_with_fallback": beta_with_fallback,
        "innovation": innovation_out,
        "S_diag": S_diag_out,
        "K_ay": K_ay_out,
    }
