# modules/yaw_damping.py: Werner Eq. 4.4 yaw-damping term. Hand-worked
# cases only -- the function is arithmetic, so the checks pin the formula
# and its NaN/sign semantics, not any session's numbers.

import numpy as np

from modules.yaw_damping import estimate_yaw_damping, yaw_damping_moment

# Hand-worked case, every input stated:
C_AF_EFF = 100000.0   # N/rad, front effective cornering stiffness
C_AR_EFF = 100000.0   # N/rad, rear effective cornering stiffness
L_F = 1.1             # m, CG to front axle
L_R = 1.4             # m, CG to rear axle
V = 40.0              # m/s
V_FLOOR = 5.0         # m/s, stand-in for moving_speed_min_mps
# (100000*1.1^2 + 100000*1.4^2) / 40 = (121000 + 196000) / 40 = 7925
D_PSI_EXPECTED = 7925.0   # N m s/rad


def test_hand_worked_case():
    out = estimate_yaw_damping([C_AF_EFF], [C_AR_EFF], L_F, L_R, [V], V_FLOOR)
    assert np.isclose(out["D_psi"][0], D_PSI_EXPECTED)
    assert np.isclose(out["D_psi_front"][0], 121000.0 / 40.0)
    assert np.isclose(out["D_psi_rear"][0], 196000.0 / 40.0)
    # Eq. 4.3 second term at psi_dot = 0.5 rad/s: 7925 * 0.5 = 3962.5 N m
    assert np.isclose(yaw_damping_moment(out["D_psi"], [0.5])[0], 3962.5)


def test_one_axle_invalid_gives_nan_but_keeps_other_contribution():
    out = estimate_yaw_damping([np.nan], [C_AR_EFF], L_F, L_R, [V], V_FLOOR)
    assert np.isnan(out["D_psi"][0])
    assert np.isnan(out["D_psi_front"][0])
    assert np.isclose(out["D_psi_rear"][0], 196000.0 / 40.0)


def test_both_axles_invalid_gives_nan():
    out = estimate_yaw_damping([np.nan], [np.nan], L_F, L_R, [V], V_FLOOR)
    assert np.isnan(out["D_psi"][0])


def test_speed_at_or_below_floor_is_undefined_not_inf():
    out = estimate_yaw_damping([C_AF_EFF] * 3, [C_AR_EFF] * 3, L_F, L_R, [0.0, V_FLOOR, 1e-9], V_FLOOR)
    assert np.all(np.isnan(out["D_psi"]))
    assert not np.any(np.isinf(out["D_psi"]))


def test_negative_stiffness_passes_through_as_negative_damping():
    # Post-peak front axle: C_af_eff = -50000 N/rad.
    # (-50000*1.21 + 100000*1.96) / 40 = (-60500 + 196000) / 40 = 3387.5
    out = estimate_yaw_damping([-50000.0], [C_AR_EFF], L_F, L_R, [V], V_FLOOR)
    assert np.isclose(out["D_psi"][0], 3387.5)
    assert out["D_psi_front"][0] < 0
    # Both axles post-peak -> negative D_psi, not clipped:
    # (-50000*1.21 - 50000*1.96) / 40 = -3962.5
    out2 = estimate_yaw_damping([-50000.0], [-50000.0], L_F, L_R, [V], V_FLOOR)
    assert np.isclose(out2["D_psi"][0], -3962.5)
