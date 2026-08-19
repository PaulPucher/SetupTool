# WP-S4 (Open Board item B, sideslip methods comparison): linear Kalman
# sideslip observer, diagnostics-only candidate. Read-only, Tier A model
# (bicycle-model state-space form) + Tier B estimator implementation
# (discrete recursion, hand-tuned Q/R). No production wiring, no
# consumer -- not imported anywhere outside diagnostics/. Method anchors
# recorded in thesis_notes.md, WP-S4 entry.
#
# States x = [beta, yaw_rate]. Input u = delta_f (front steering angle,
# existing Level-4 steering_ratio path). Measurements z = [yaw_rate,
# ay] (sclu_yaw_rate, log_acc_y). Speed v is a time-varying model
# parameter (production ecu_speed), not a state.
#
# Continuous-time model (derived from force/moment balance + a linear
# tyre model Fy_f=Caf*alpha_f, Fy_r=Car*alpha_r, using THIS codebase's
# own alpha_f/alpha_r sign convention -- modules/stability_analysis.py
# estimate_slip_angles -- and its own ay = v*(beta_dot + yaw_rate)
# kinematic identity, estimate_sideslip):
#
#   beta_dot = -(Caf+Car)/(m*Vx) * beta
#              + [-1 - (Caf*lf-Car*lr)/(m*Vx^2)] * yaw_rate
#              + Caf/(m*Vx) * delta_f
#   yaw_rate_dot = -(Caf*lf-Car*lr)/Iz * beta
#              - (Caf*lf^2+Car*lr^2)/(Iz*Vx) * yaw_rate
#              + Caf*lf/Iz * delta_f
#
# ay measurement is beta_dot substituted back into the kinematic
# identity (keeps the measurement equation in x, u only):
#   ay = -(Caf+Car)/m * beta - (Caf*lf-Car*lr)/(m*Vx) * yaw_rate
#        + Caf/m * delta_f
# yaw_rate measurement is the second state directly.
#
# Caf, Car (tyre-stiffness prior) use circularity option 2: fixed
# reference values (config/parameters.json stability_estimation.
# cs_front/rear_fallback_reference_n_per_rad), NOT the production
# alpha-derived C_linear_ref -- a beta-derived stiffness would make this
# observer circular against the very estimate it is meant to
# cross-check.
#
# Discretization: first-order (Euler) Ad = I + A*dt, Bd = B*dt at each
# sample's own instantaneous Vx -- a Tier B numerical simplification,
# adequate at this session's 50 Hz sample rate relative to yaw-dynamics
# bandwidth, not a modelling claim.

import numpy as np

# Process noise covariance (state uncertainty growth per step). Base
# values hand-tuned (no ground-truth beta or yaw-rate-error signal
# exists on this dataset to tune against); scaled by QR_RATIO, chosen
# from the WP-S5b 7-point ratio sweep (diagnostics/inspect_kalman_qr_
# ratio_sweep.py) -- an INTERIOR recommendation, not the steady-state-
# optimal zone (ratio~0.007-0.05), because that zone measurably
# degrades transient tracking (the sweep's d(beta)/dt vs d(ay)/dt
# correlation during corner entry/exit: -0.70 to -0.91 there vs -0.99
# at the chosen ratio). Full trade-off record: thesis_notes.md, WP-S5b
# tuning-outcome entry. A linear KF's steady-state gain depends only on
# the Q/R ratio (confirmed in that sweep), so only Q is scaled here;
# R (below) stays at its own original hand-tuned value, unscaled.
QR_RATIO = 0.3162
Q_BETA_VAR = np.radians(0.1) ** 2 * QR_RATIO        # rad^2 per step
Q_YAW_RATE_VAR = np.radians(0.05) ** 2 * QR_RATIO   # (rad/s)^2 per step

# Measurement noise covariance. Hand-tuned initial values -- no
# independent yaw-rate or ay reference exists on this dataset to tune
# against; larger values trust the sensor less and the model more.
# Unchanged by WP-S5b tuning: the ratio sweep held R fixed and varied
# only Q_scale, per the ratio-invariance finding above.
R_YAW_RATE_VAR = np.radians(0.1) ** 2    # (rad/s)^2
R_AY_VAR = 0.05 ** 2                     # (m/s^2)^2

# Initial state covariance at each moving-mask rising edge (car assumed
# stationary/beta=0 immediately before). Hand-tuned initial values, same
# no-ground-truth caveat as Q/R above.
P0_BETA_VAR = np.radians(1.0) ** 2
P0_YAW_RATE_VAR = np.radians(1.0) ** 2


def estimate_sideslip_kalman(state, params):
    vp = params["vehicle"]
    se = params["stability_estimation"]

    v = state["v_mps"]
    yaw_rate_meas = state["yaw_rate_radps"]
    ay_meas = state["ay_mps2"]
    delta_f = state["delta_f_rad"]
    moving = state["moving_mask"]
    sr = state["sample_rate_hz"]
    n = len(v)
    dt = 1.0 / sr

    m = vp["mass_kg"]
    lf = vp["cog_to_front_axle_m"]
    lr = vp["cog_to_rear_axle_m"]
    Iz = vp["yaw_inertia_kalman_kgm2"]
    Caf = se["cs_front_fallback_reference_n_per_rad"]
    Car = se["cs_rear_fallback_reference_n_per_rad"]
    v_min = se["moving_speed_min_mps"]

    Q = np.diag([Q_BETA_VAR, Q_YAW_RATE_VAR])
    R = np.diag([R_YAW_RATE_VAR, R_AY_VAR])
    P0 = np.diag([P0_BETA_VAR, P0_YAW_RATE_VAR])
    eye2 = np.eye(2)

    beta_out = np.zeros(n)
    x = np.zeros(2)
    P = P0.copy()

    for i in range(n):
        if not moving[i]:
            x = np.zeros(2)
            P = P0.copy()
            beta_out[i] = 0.0
            continue

        Vx = max(float(v[i]), v_min)
        u = float(delta_f[i])

        A = np.array([
            [-(Caf + Car) / (m * Vx),        -1.0 - (Caf * lf - Car * lr) / (m * Vx ** 2)],
            [-(Caf * lf - Car * lr) / Iz,    -(Caf * lf ** 2 + Car * lr ** 2) / (Iz * Vx)],
        ])
        B = np.array([Caf / (m * Vx), Caf * lf / Iz])
        C = np.array([
            [0.0, 1.0],
            [-(Caf + Car) / m, -(Caf * lf - Car * lr) / (m * Vx)],
        ])
        D = np.array([0.0, Caf / m])

        Ad = eye2 + A * dt
        Bd = B * dt

        x_pred = Ad @ x + Bd * u
        P_pred = Ad @ P @ Ad.T + Q

        z = np.array([float(yaw_rate_meas[i]), float(ay_meas[i])])
        innovation = z - (C @ x_pred + D * u)
        S = C @ P_pred @ C.T + R
        K = P_pred @ C.T @ np.linalg.inv(S)

        x = x_pred + K @ innovation
        P = (eye2 - K @ C) @ P_pred

        beta_out[i] = x[0]

    return beta_out
