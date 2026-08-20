# PLAN.md unsupervised package, Phase 3 -- finite-difference
# verification of modules/tyre_model_pacejka.py's analytic stiffness
# derivative. Same discipline as test_pure_functions.py's Dugoff
# checks: an independent numerical check of the CODE's own claimed
# formula, not a re-derivation of whether Pacejka is the right model
# for this car.

import numpy as np
import pytest

from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness

# Chair's own starting guess (PLAN.md Phase 3 work order), used here as a
# representative, not arbitrary, parameter point.
B, C, D, E = 12.0, 1.9, 8000.0, 0.97
FD_STEP = 1e-6


@pytest.mark.parametrize("alpha_deg", [-10.0, -3.0, -0.5, 0.0, 0.5, 3.0, 10.0])
def test_pacejka_stiffness_matches_central_difference(alpha_deg):
    alpha_rad = np.radians(alpha_deg)
    analytic = pacejka_lateral_stiffness(alpha_rad, B, C, D, E)
    fy_plus = pacejka_lateral_force(alpha_rad + FD_STEP, B, C, D, E)
    fy_minus = pacejka_lateral_force(alpha_rad - FD_STEP, B, C, D, E)
    numeric = (fy_plus - fy_minus) / (2 * FD_STEP)
    rel_err = abs(analytic - numeric) / max(abs(numeric), 1.0)
    assert rel_err < 1e-6, f"alpha={alpha_deg} deg: analytic={analytic}, numeric={numeric}, rel_err={rel_err}"


def test_pacejka_force_is_odd():
    # Fy(-alpha) == -Fy(alpha): the formula is a composition of odd
    # functions (sin/arctan) of an odd argument in alpha -- exact, not
    # approximate, so an equality check (not a tolerance) is appropriate.
    alpha_rad = np.radians(4.5)
    fy_pos = pacejka_lateral_force(alpha_rad, B, C, D, E)
    fy_neg = pacejka_lateral_force(-alpha_rad, B, C, D, E)
    assert fy_pos == pytest.approx(-fy_neg, rel=1e-12)


def test_pacejka_stiffness_is_even():
    # dFy/dalpha is the derivative of an odd function, hence even.
    alpha_rad = np.radians(4.5)
    k_pos = pacejka_lateral_stiffness(alpha_rad, B, C, D, E)
    k_neg = pacejka_lateral_stiffness(-alpha_rad, B, C, D, E)
    assert k_pos == pytest.approx(k_neg, rel=1e-12)


def test_pacejka_positive_stiffness_at_origin():
    # At alpha=0 the tyre must present positive cornering stiffness for
    # this to be a physically usable model in the linear regime.
    k0 = pacejka_lateral_stiffness(0.0, B, C, D, E)
    assert k0 > 0
