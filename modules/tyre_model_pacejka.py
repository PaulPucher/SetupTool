# Reduced 4-parameter Magic Formula lateral tyre model. Pure Python/
# numpy, no Qt imports.
#
# PLAN.md unsupervised package, Phase 3. Cited as "chair performance_
# analysis tooling (internal)" -- the chair's own reference model for
# this evaluation; the published general form is Rajamani, Vehicle
# Dynamics and Control, 2nd ed., Ch. 13 "Tire Models" (Magic Formula
# section, page TBD verify). New file rather than an edit to modules/
# tyre_model.py (Dugoff): PLAN.md's Phase 3 work order calls this a
# "new code path", and modules/tyre_model.py is an existing production
# file this package's hard constraints do not authorise editing --
# kept as a fully separate pair of functions instead, same shape as
# tyre_model.py's Dugoff pair, no shared state.
#
# Form: Fy = D*sin(C*arctan(B*alpha - E*(B*alpha - arctan(B*alpha))))
# Parameter order (B, C, D, E) matches the chair's own starting guess
# ([12, 1.9, 8000, 0.97], cited in the work order) -- B=stiffness
# factor, C=shape factor, D=peak factor (peak Fy, N), E=curvature
# factor. No sign-convention note is needed here the way Dugoff's
# module needed one: alpha and Fy enter this formula through a plain
# odd sine/arctan composition with no separate literature sign to
# reconcile against this codebase's own alpha/Fy convention.

import numpy as np


def pacejka_lateral_force(alpha_rad, B, C, D, E):
    """Compute Fy(alpha) for one axle, reduced 4-parameter Magic Formula."""
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    b_alpha = B * alpha_rad
    u = b_alpha - E * (b_alpha - np.arctan(b_alpha))
    return D * np.sin(C * np.arctan(u))


def pacejka_lateral_stiffness(alpha_rad, B, C, D, E):
    """Compute dFy/dalpha analytically. u = B*alpha*(1-E) + E*arctan(B*alpha) is
    the Magic Formula's own inner argument; by the chain rule,

      dFy/dalpha = D*C*cos(C*arctan(u)) * (du/dalpha) / (1 + u^2)
      du/dalpha  = B*(1-E) + E*B / (1 + (B*alpha)^2)

    Verified against central-difference numerical differentiation of
    pacejka_lateral_force in tests/test_pacejka_model.py (same
    verification discipline as modules/tyre_model.py's Dugoff
    stiffness docstring).
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    b_alpha = B * alpha_rad
    u = b_alpha - E * (b_alpha - np.arctan(b_alpha))
    du_dalpha = B * (1.0 - E) + E * B / (1.0 + b_alpha ** 2)
    dFy_du_arctan = D * C * np.cos(C * np.arctan(u))
    return dFy_du_arctan * du_dalpha / (1.0 + u ** 2)
