# Nonlinear single-track EKF sideslip observer, Pacejka (reduced
# 4-parameter Magic Formula) tyre model -- Phase 3 variant.
#
# NEW CODE PATH per PLAN.md's Phase 3 work order ("do not modify the
# existing Dugoff path"): a structural mirror of diagnostics/
# sideslip_ekf_dugoff.py with modules/tyre_model_pacejka.py's
# pacejka_lateral_force/pacejka_lateral_stiffness substituted for
# Dugoff's dugoff_lateral_force/dugoff_lateral_stiffness in both the
# state propagation and both Jacobians. Everything else (states,
# measurements, discretization, divergence monitor, fallback
# behaviour) is IDENTICAL to the Dugoff filter -- see that file's own
# header for the full method anchors and design rationale, not
# repeated here.
#
# Per-axle tyre parameters are (B, C, D, E) instead of Dugoff's
# (c_alpha, mu_fz) -- the pass config dict's keys change shape
# accordingly (b_front/c_front/d_front/e_front, b_rear/... ).

import numpy as np

from modules.stability_analysis import estimate_sideslip
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


def estimate_sideslip_ekf_pacejka(state, params, pass_id):
    vp = params["vehicle"]
    se = params["stability_estimation"]
    cfg = params["tyre_model_ekf_pacejka"][pass_id]

    m = vp["mass_kg"]
    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]
    Iz = vp["yaw_inertia_kgm2"]

    Bf, Cf, Df, Ef = cfg["b_front"], cfg["c_front"], cfg["d_front"], cfg["e_front"]
    Br, Cr, Dr, Er = cfg["b_rear"], cfg["c_rear"], cfg["d_rear"], cfg["e_rear"]

    Q = np.diag([cfg["Q_beta_var"], cfg["Q_yaw_rate_var"]])
    R = np.diag([cfg["R_yaw_rate_var"], cfg["R_ay_var"]])
    P0 = np.diag([cfg["P0_beta_var"], cfg["P0_yaw_rate_var"]])

    beta_hard_bound_rad = np.radians(cfg["beta_hard_bound_deg"])
    nis_window = int(cfg["nis_window_samples"])
    nis_chi2_bound = cfg["nis_chi2_bound"]
    nis_flag_fraction = cfg["nis_flag_fraction"]

    v = state["v_mps"]
    yaw_rate_meas = state["yaw_rate_radps"]
    ay_meas = state["ay_mps2"]
    delta_f = state["delta_f_rad"]
    moving = state["moving_mask"]
    sr = state["sample_rate_hz"]
    v_min = se["moving_speed_min_mps"]

    beta_kinematic = estimate_sideslip(state, params)

    n = len(v)
    dt = 1.0 / sr
    eye2 = np.eye(2)

    beta_out = np.zeros(n)
    yaw_rate_state_out = np.zeros(n)
    nis_out = np.full(n, np.nan)
    diverged_mask = np.zeros(n, dtype=bool)
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
            beta_out[i] = 0.0
            yaw_rate_state_out[i] = 0.0
            beta_with_fallback[i] = 0.0
            continue

        Vx = max(float(v[i]), v_min)
        u = float(delta_f[i])

        beta_x, r_x = x[0], x[1]
        alpha_f, alpha_r = slip_angles(beta_x, r_x, u, Vx, a, b)

        Fy_f = pacejka_lateral_force(alpha_f, Bf, Cf, Df, Ef)
        Fy_r = pacejka_lateral_force(alpha_r, Br, Cr, Dr, Er)
        Cf_eff = pacejka_lateral_stiffness(alpha_f, Bf, Cf, Df, Ef)
        Cr_eff = pacejka_lateral_stiffness(alpha_r, Br, Cr, Dr, Er)

        beta_dot = (Fy_f + Fy_r) / (m * Vx) - r_x
        r_dot = (a * Fy_f - b * Fy_r) / Iz
        x_pred = x + dt * np.array([beta_dot, r_dot])

        F = process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz)
        Ad = eye2 + F * dt
        P_pred = Ad @ P @ Ad.T + Q

        beta_p, r_p = x_pred[0], x_pred[1]
        alpha_f_p, alpha_r_p = slip_angles(beta_p, r_p, u, Vx, a, b)

        Fy_f_p = pacejka_lateral_force(alpha_f_p, Bf, Cf, Df, Ef)
        Fy_r_p = pacejka_lateral_force(alpha_r_p, Br, Cr, Dr, Er)
        Cf_eff_p = pacejka_lateral_stiffness(alpha_f_p, Bf, Cf, Df, Ef)
        Cr_eff_p = pacejka_lateral_stiffness(alpha_r_p, Br, Cr, Dr, Er)

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

        nis_recent.append(nis > nis_chi2_bound)
        if len(nis_recent) > nis_window:
            nis_recent.pop(0)
        window_flag = (len(nis_recent) == nis_window
                        and (sum(nis_recent) / nis_window) > nis_flag_fraction)
        bound_flag = abs(x_upd[0]) > beta_hard_bound_rad

        raw_beta, raw_r = float(x_upd[0]), float(x_upd[1])
        beta_out[i] = raw_beta
        yaw_rate_state_out[i] = raw_r

        diverged = window_flag or bound_flag
        diverged_mask[i] = diverged

        if diverged:
            beta_with_fallback[i] = beta_kinematic[i]
            x = np.array([beta_kinematic[i], yaw_rate_meas[i]])
            P = P0.copy()
            nis_recent.clear()
        else:
            beta_with_fallback[i] = raw_beta
            x = x_upd
            P = P_upd

    return {
        "beta": beta_out,
        "yaw_rate_state": yaw_rate_state_out,
        "nis": nis_out,
        "diverged_mask": diverged_mask,
        "beta_with_fallback": beta_with_fallback,
        "innovation": innovation_out,
        "S_diag": S_diag_out,
        "K_ay": K_ay_out,
    }
