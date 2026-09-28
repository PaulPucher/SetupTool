# Yaw-damping term of the measurement-side yaw moment (Werner Eq. 4.3/4.4).
# Pure numpy, no Qt, no config reads: every input is passed in. Not called
# by the production pipeline (v1 consumer is a diagnostic only).

import numpy as np


def estimate_yaw_damping(c_alpha_f, c_alpha_r, l_f, l_r, v_mps, v_floor_mps):
    """D_psi = (C_af_eff*l_f^2 + C_ar_eff*l_r^2) / v  [N m s/rad].

    Werner 2021 sec. 4.5.2 (Eqs. 4.3/4.4); Milliken RCVD ch. 5 (sec. 5.7,
    Eq. 5.6). The effective stiffnesses are Module 4b's model-free local
    slopes (C_alpha_f/r), so no wheel load enters -- Werner needed Fz only
    to get the same slope out of his tyre model.

    Undefined (NaN) wherever either axle's stiffness is NaN or v is at or
    below v_floor_mps: a missing axle is never filled from another source,
    and a near-zero speed would only produce a meaningless large number.
    Negative stiffness (post-peak axle) passes through as negative damping.

    Returns D_psi plus the two axle contributions, each NaN only where its
    own stiffness or v is undefined, so a reader can see which axle is
    missing.
    """
    c_f = np.asarray(c_alpha_f, dtype=float)
    c_r = np.asarray(c_alpha_r, dtype=float)
    v = np.asarray(v_mps, dtype=float)
    v_ok = np.isfinite(v) & (v > v_floor_mps)
    v_safe = np.where(v_ok, v, np.nan)
    d_front = c_f * l_f ** 2 / v_safe
    d_rear = c_r * l_r ** 2 / v_safe
    # NaN in either contribution makes the sum NaN: no partial D_psi.
    return {"D_psi": d_front + d_rear, "D_psi_front": d_front, "D_psi_rear": d_rear}


def yaw_damping_moment(d_psi, yaw_rate_radps):
    """Mz_damping = D_psi * psi_dot [N m] -- the second term of Eq. 4.3."""
    return np.asarray(d_psi, dtype=float) * np.asarray(yaw_rate_radps, dtype=float)
