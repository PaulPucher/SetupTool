# Nonlinear single-track EKF sideslip observer, Dugoff tyre model,
# pass 0. Diagnostics-only candidate, mirrors diagnostics/
# sideslip_kalman_observer.py's placement and never-production status --
# no modules/ or ui/ consumer.
#
# Method anchors recorded in thesis_notes.md, "WP-N2: nonlinear Dugoff
# EKF proposal" entry (model equations: Rajamani sec. 2.3/2.6 + Ulsoy,
# Peng, Cakmakci sec. 14.1/14.3; EKF/Kalman treatment: Rajamani Ch. 14).
#
# States x = [beta, yaw_rate]. Input u = delta_f (front steering angle).
# Vx is a per-sample scheduled parameter (production ecu_speed, floored
# at moving_speed_min_mps), not a state -- same convention as the
# rejected linear observer. Measurements z = [yaw_rate, ay]: IDENTICAL
# sensor set to the rejected filter (sclu_yaw_rate, log_acc_y); only h()
# is now nonlinear instead of a fixed C matrix.
#
# c_alpha_f/r, mu_fz_f/r are FROZEN pass-0 parameters (config/
# parameters.json tyre_model_ekf.pass_0, sourced from WP-N1b's Module-4b-
# seeded Dugoff fit) -- this file never refits them. Q/R/P0 are also read
# from that same config block (seeded from the tuned linear observer,
# QR_RATIO=0.3162, see the block's own seeded_from note) -- unlike the
# rejected filter, nothing here is a hardcoded module-level constant: a
# numbered pass must be fully reproducible from recorded parameters
# alone.
#
# Discretization: explicit Euler, matching the rejected filter's own
# documented choice, for direct comparability. IMPORTANT mechanical
# difference from that filter (which was fully linear, so Ad@x and F@x
# were the same operation): the STATE is propagated by integrating the
# true nonlinear f(x,u) directly (x_pred = x + dt*f(x,u)), never by
# F@x -- the Jacobian F is used ONLY to propagate the covariance P. F is
# evaluated at the prior state estimate x_k|k; the measurement Jacobian H
# is evaluated at the predicted state x_k+1|k -- both standard EKF
# convention. Tyre slope terms (Cf_eff, Cr_eff) come exclusively from
# modules/tyre_model.py's analytic dugoff_lateral_stiffness -- no
# numerical differencing, no re-derivation.
#
# Divergence monitoring: a windowed Normalized Innovation Squared (NIS)
# check against a chi-square bound, plus a hard physical ceiling on
# |beta|. The ceiling (config: beta_hard_bound_deg) is deliberately NOT
# derived from the kinematic estimate's own observed range -- that
# estimate is documented elsewhere (thesis_notes.md, "Linear observer"
# entries) to under-read mid-corner, so its range would clip exactly the
# signal this filter exists to recover. 15 deg is a physically anchored
# ceiling instead: controlled racing sideslip on this class of car stays
# well below it, while a genuinely diverged filter exceeds it by orders
# of magnitude. On either trigger the sample's state resets: beta -> the
# kinematic estimate at that instant, yaw_rate state -> the measured yaw
# rate at that instant, P -> P0. The raw (pre-fallback) EKF output is
# still returned alongside the fallback-corrected series and the
# diverged_mask flag -- never a silent substitution. Per-channel
# innovation and predicted-variance diagonal (S_diag) are also returned,
# additive-only -- not used by the divergence monitor itself (which acts
# on the combined 2-DOF nis_out), exposed so a caller can compute
# per-channel NIS without duplicating the measurement-update math.

import numpy as np

from modules.stability_analysis import estimate_sideslip
from modules.tyre_model import dugoff_lateral_force, dugoff_lateral_stiffness


def slip_angles(beta, r, u, Vx, a, b):
    # Small-angle form, same definitions/sign convention as modules/
    # stability_analysis.py estimate_slip_angles.
    alpha_f = u - beta - a * r / Vx
    alpha_r = -beta + b * r / Vx
    return alpha_f, alpha_r


def process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz):
    # F = df/dx at the state the slip angles (hence Cf_eff/Cr_eff) were
    # evaluated at. Exposed as its own function so the unit-level sanity
    # check (diagnostics/inspect_ekf_dugoff_sanity_checks.py) can call the
    # exact same formula the filter loop uses, not a duplicate copy.
    return np.array([
        [-(Cf_eff + Cr_eff) / (m * Vx), (-a * Cf_eff + b * Cr_eff) / (m * Vx ** 2) - 1.0],
        [(-a * Cf_eff + b * Cr_eff) / Iz, -(a ** 2 * Cf_eff + b ** 2 * Cr_eff) / (Iz * Vx)],
    ])


def measurement_jacobian(Cf_eff, Cr_eff, Vx, m, a, b):
    # H = dh/dx, same exposure rationale as process_jacobian above.
    return np.array([
        [0.0, 1.0],
        [-(Cf_eff + Cr_eff) / m, (-a * Cf_eff + b * Cr_eff) / (m * Vx)],
    ])


def estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_0"):
    # pass_id selects which config/parameters.json tyre_model_ekf.pass_N
    # block to run with -- defaults to pass_0 so existing call sites are
    # unaffected. Each block's own changed_from_previous field records
    # what differs from the prior pass (tyre curve, noise model, or both).
    vp = params["vehicle"]
    se = params["stability_estimation"]
    cfg = params["tyre_model_ekf"][pass_id]

    m = vp["mass_kg"]
    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]
    # AMENDMENT (this WP): production Iz, not yaw_inertia_kalman_kgm2 --
    # estimate_lateral_forces built the Fy_f/Fy_r that WP-N1b's frozen
    # c_alpha/mu_fz were fit against using this same Iz. Using the
    # Kalman-candidate Iz (1800.0) instead would make this filter's own
    # moment balance inconsistent with its training data's Iz by ~14%.
    Iz = vp["yaw_inertia_kgm2"]

    c_alpha_f = cfg["c_alpha_front_n_per_rad"]
    c_alpha_r = cfg["c_alpha_rear_n_per_rad"]
    mu_fz_f = cfg["mu_fz_front_N"]
    mu_fz_r = cfg["mu_fz_rear_N"]

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

    beta_out = np.zeros(n)          # raw EKF beta, pre-fallback -- kept for transparency
    yaw_rate_state_out = np.zeros(n)
    nis_out = np.full(n, np.nan)
    diverged_mask = np.zeros(n, dtype=bool)
    beta_with_fallback = np.zeros(n)
    # Per-channel innovation/predicted-variance, additive-only outputs --
    # not consumed by the divergence monitor itself (which uses the
    # combined 2-DOF nis_out above), exposed so per-channel NIS
    # (innovation^2 / S_diag) can be computed downstream without
    # duplicating the measurement-update math elsewhere.
    innovation_out = np.full((n, 2), np.nan)   # columns: yaw_rate, ay
    S_diag_out = np.full((n, 2), np.nan)       # columns: yaw_rate, ay
    K_ay_out = np.full((n, 2), np.nan)         # Kalman gain's ay column: [K_beta_ay, K_r_ay]

    x = np.zeros(2)
    P = P0.copy()
    nis_recent = []  # rolling window of bool(NIS > bound)

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

        # --- predict: nonlinear state propagation, Jacobian at prior x ---
        beta_x, r_x = x[0], x[1]
        alpha_f, alpha_r = slip_angles(beta_x, r_x, u, Vx, a, b)

        Fy_f = dugoff_lateral_force(alpha_f, c_alpha_f, mu_fz_f)
        Fy_r = dugoff_lateral_force(alpha_r, c_alpha_r, mu_fz_r)
        Cf_eff = dugoff_lateral_stiffness(alpha_f, c_alpha_f, mu_fz_f)
        Cr_eff = dugoff_lateral_stiffness(alpha_r, c_alpha_r, mu_fz_r)

        beta_dot = (Fy_f + Fy_r) / (m * Vx) - r_x
        r_dot = (a * Fy_f - b * Fy_r) / Iz
        x_pred = x + dt * np.array([beta_dot, r_dot])

        F = process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz)
        Ad = eye2 + F * dt
        P_pred = Ad @ P @ Ad.T + Q

        # --- update: measurement Jacobian at predicted state ---
        beta_p, r_p = x_pred[0], x_pred[1]
        alpha_f_p, alpha_r_p = slip_angles(beta_p, r_p, u, Vx, a, b)

        Fy_f_p = dugoff_lateral_force(alpha_f_p, c_alpha_f, mu_fz_f)
        Fy_r_p = dugoff_lateral_force(alpha_r_p, c_alpha_r, mu_fz_r)
        Cf_eff_p = dugoff_lateral_stiffness(alpha_f_p, c_alpha_f, mu_fz_f)
        Cr_eff_p = dugoff_lateral_stiffness(alpha_r_p, c_alpha_r, mu_fz_r)

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
            # Fixed fallback (not optional): beta -> kinematic at this
            # instant, yaw-rate state -> measured yaw rate, P -> P0.
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
