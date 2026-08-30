# Dugoff lateral tyre model. Pure Python/numpy, no Qt imports.
#
# Tier A anchor: Rajamani, Vehicle Dynamics and Control, 2nd ed., Ch. 13.10
# "Dugoff Tire Model" (eqs. 13.72-13.76, page TBD verify against the actual
# printed edition). Pure-cornering reduction (no combined slip, no
# longitudinal slip ratio term) -- the form this project needs, since the
# nonlinear observer this feeds only estimates lateral state.
#
# Sign convention: the literature form is Fy = -c_alpha*tan(alpha)*f(lambda)
# (SAE-style, force opposes slip). This codebase's slip-angle definitions
# (modules/stability_analysis.py estimate_slip_angles, Werner (2021) S2.2.3
# sign convention, alpha_r carries its own leading minus) already produce a
# POSITIVE Fy-vs-alpha slope -- confirmed empirically on Dubai data (both
# axles, corr(alpha, Fy_filt) > 0, matching the positive C_alpha the
# pipeline reports throughout Module 4b). This module drops the literature
# minus sign to match: Fy = +c_alpha*tan(alpha)*f(lambda). c_alpha and
# mu_fz are still both positive parameters; only the overall sign differs
# from the textbook formula, not the shape.

import numpy as np

_TAN_EPS = 1e-9  # guards the lambda division as alpha -> 0


def dugoff_lateral_force(alpha_rad, c_alpha, mu_fz):
    """Compute Fy(alpha) for one axle. mu_fz is the friction force ceiling
    (mu * Fz, Newtons) -- a single lumped parameter here, not mu and Fz
    passed separately; the caller decides whether it's a fixed scalar or
    a per-sample array.

    lambda < 1 is the sliding/saturated branch (large slip relative to
    the available friction force); lambda >= 1 is the adhesion branch,
    where f is capped at 1 and Fy reduces to the linear c_alpha*tan(alpha)
    relation. This threshold is the model's adhesion/sliding boundary, not
    a tunable.
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    tan_a = np.tan(alpha_rad)
    denom = 2.0 * c_alpha * np.maximum(np.abs(tan_a), _TAN_EPS)
    lam = mu_fz / denom
    f_lam = np.where(lam < 1.0, (2.0 - lam) * lam, 1.0)
    return c_alpha * tan_a * f_lam


def dugoff_lateral_stiffness(alpha_rad, c_alpha, mu_fz):
    """Compute dFy/dalpha analytically, matching dugoff_lateral_force's
    sign convention. Piecewise-continuous at lambda=1 by construction (the
    Dugoff f(lambda) has a continuous first derivative there):

    lambda >= 1: dFy/dalpha = c_alpha / cos^2(alpha)
    lambda <  1: dFy/dalpha = c_alpha / cos^2(alpha) * lambda^2

    Both branches collapse to the single expression below using
    min(lambda, 1)^2 -- and since lambda depends on |tan(alpha)|, the
    result is already an even function of alpha (correct: stiffness is
    the derivative of an odd force curve), no separate sign handling
    needed. Verified against central-difference numerical differentiation
    of dugoff_lateral_force (max relative error ~2.5e-9) during
    implementation.
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    cos_a = np.cos(alpha_rad)
    tan_a = np.tan(alpha_rad)
    denom = 2.0 * c_alpha * np.maximum(np.abs(tan_a), _TAN_EPS)
    lam = mu_fz / denom
    lam_capped = np.minimum(lam, 1.0)
    return c_alpha / cos_a ** 2 * lam_capped ** 2
