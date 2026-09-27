# Reduced 4-parameter Magic Formula, lateral only.
# After the chair performance_analysis tooling (internal).
# General form: Rajamani ch. 13.
#
# Fy = D*sin(C*arctan(B*alpha - E*(B*alpha - arctan(B*alpha))))
# B = stiffness, C = shape, D = peak Fy [N], E = curvature. Odd in alpha,
# no sign flip needed (unlike Dugoff).

import numpy as np


def pacejka_lateral_force(alpha_rad, B, C, D, E):
    """Fy(alpha) for one axle."""
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    b_alpha = B * alpha_rad
    u = b_alpha - E * (b_alpha - np.arctan(b_alpha))
    return D * np.sin(C * np.arctan(u))


def pacejka_lateral_stiffness(alpha_rad, B, C, D, E):
    """Analytic dFy/dalpha, chain rule through u = B*alpha*(1-E) + E*arctan(B*alpha):

      dFy/dalpha = D*C*cos(C*arctan(u)) * (du/dalpha) / (1 + u^2)
      du/dalpha  = B*(1-E) + E*B / (1 + (B*alpha)^2)

    Checked against central differences in tests/test_pacejka_model.py.
    """
    alpha_rad = np.asarray(alpha_rad, dtype=float)
    b_alpha = B * alpha_rad
    u = b_alpha - E * (b_alpha - np.arctan(b_alpha))
    du_dalpha = B * (1.0 - E) + E * B / (1.0 + b_alpha ** 2)
    dFy_du_arctan = D * C * np.cos(C * np.arctan(u))
    return dFy_du_arctan * du_dalpha / (1.0 + u ** 2)
