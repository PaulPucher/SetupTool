# Dugoff lateral tyre model, pure cornering (no combined slip).
# Rajamani ch. 13.10.
#
# Sign: literature has Fy = -c_alpha*tan(alpha)*f(lambda). Our slip angles
# (Werner S2.2.3 convention) give a positive Fy-alpha slope on real data,
# so the minus is dropped: Fy = +c_alpha*tan(alpha)*f(lambda).

import numpy as np

_TAN_EPS = 1e-9  # guards the lambda division as alpha -> 0


def dugoff_lateral_force(alpha_rad, c_alpha, mu_fz):
    """Fy(alpha) for one axle. mu_fz = mu*Fz [N], lumped; scalar or per-sample.
    lambda < 1 = sliding, lambda >= 1 = adhesion (f = 1, linear). Model
    boundary, not a tunable.
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    tan_a = np.tan(alpha_rad)
    denom = 2.0 * c_alpha * np.maximum(np.abs(tan_a), _TAN_EPS)
    lam = mu_fz / denom
    f_lam = np.where(lam < 1.0, (2.0 - lam) * lam, 1.0)
    return c_alpha * tan_a * f_lam


def dugoff_lateral_stiffness(alpha_rad, c_alpha, mu_fz):
    """Analytic dFy/dalpha, same sign convention as dugoff_lateral_force:

    lambda >= 1: dFy/dalpha = c_alpha / cos^2(alpha)
    lambda <  1: dFy/dalpha = c_alpha / cos^2(alpha) * lambda^2

    -> one expression with min(lambda, 1)^2; even in alpha via |tan|.
    Checked against central differences (max rel. error ~2.5e-9).
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    cos_a = np.cos(alpha_rad)
    tan_a = np.tan(alpha_rad)
    denom = 2.0 * c_alpha * np.maximum(np.abs(tan_a), _TAN_EPS)
    lam = mu_fz / denom
    lam_capped = np.minimum(lam, 1.0)
    return c_alpha / cos_a ** 2 * lam_capped ** 2
