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
    if not cfg.get("d_is_mu", False):
        raise ValueError("estimate_sideslip_ekf_pacejka needs d_is_mu (fit L); the constant-D path was removed")
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
