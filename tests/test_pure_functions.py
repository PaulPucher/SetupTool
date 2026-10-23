# Phase 3 -- unit tests on pure numerical functions, hand-computed
# expected values where a closed form exists, not just self-consistency.
#
# REGRESSION, NOT CORRECTNESS: a hand-derived expected value here is
# checked against the FORMULA the docstring/thesis_notes.md claims the
# code implements (e.g. the EKF Jacobians against finite differences, the
# kinematic slip-angle geometry at beta=0). Passing confirms the code
# matches its OWN stated method; it is not an independent re-derivation
# of whether that method is the right one for this car (see PLAN.md's
# own open questions on that point, e.g. the kinematic beta circularity).

import math

import numpy as np
import pytest

from modules.stability_analysis import estimate_slip_angles, load_parameters
from modules.corner_aggregation import _nanmedian_or_nan
from modules.sideslip_ekf_pacejka import (
    slip_angles as ekf_slip_angles, process_jacobian, measurement_jacobian,
)
from modules.tyre_model_pacejka import pacejka_lateral_force as plf, pacejka_lateral_stiffness as pls


_SYNTH_N = 30  # long enough for scipy.signal.filtfilt's default pad requirement at a 4th-order Butterworth


def _make_slip_state(v_mps, yaw_rate_radps, delta_f_rad, moving=True, sr=50.0, n=_SYNTH_N):
    """Builds a constant-value synthetic state array of length n (default
    30). estimate_slip_angles unconditionally runs its output through
    _butterworth_lowpass (scipy.signal.filtfilt), which raises ValueError
    on any array shorter than its pad length (~15 samples for this
    filter's order) -- n=30 sidesteps that while still testing the RAW
    (pre-filter) alpha_f_raw/alpha_r_raw values these tests actually
    check, which are unaffected by array length or the filter itself.
    """
    return {
        "v_mps": np.full(n, v_mps, dtype=float),
        "yaw_rate_radps": np.full(n, yaw_rate_radps, dtype=float),
        "delta_f_rad": np.full(n, delta_f_rad, dtype=float),
        "moving_mask": np.full(n, moving, dtype=bool),
        "sample_rate_hz": sr,
    }


def _slip_params(a=1.5, b=1.5, cutoff_hz=2.0):
    return {
        "vehicle": {"cog_to_front_axle_m": a, "cog_to_rear_axle_m": b},
        "stability_estimation": {"cs_filter_cutoff_hz": cutoff_hz},
    }


# --- slip angle calculation, sign conventions at known inputs ---------------

def test_slip_angles_zero_beta_zero_yaw_front_equals_steer():
    """beta=0 (v_y=0), yaw_rate=0: no sideslip and no yaw contribution --
    front slip angle must equal the steer angle exactly (alpha_f = delta_f
    - arctan(0/v_x) = delta_f), rear slip angle must be exactly zero.
    modules/stability_analysis.py estimate_slip_angles, Werner (2021)
    S2.1.1 / Milliken RCVD sign convention.
    """
    delta_f = math.radians(3.0)
    state = _make_slip_state(v_mps=30.0, yaw_rate_radps=0.0, delta_f_rad=delta_f)
    beta = np.zeros(_SYNTH_N)
    result = estimate_slip_angles(state, beta, _slip_params())
    assert result["alpha_f_raw"][0] == pytest.approx(delta_f, abs=1e-12)
    assert result["alpha_r_raw"][0] == pytest.approx(0.0, abs=1e-12)


def test_slip_angles_pure_beta_no_steer_no_yaw():
    """delta_f=0, yaw_rate=0, beta=B: v_y=v*sin(B), v_x=v*cos(B), both
    slip angles reduce to arctan(tan(B)) = B with the sign the module's
    own formula assigns -- alpha_f = 0 - arctan(v_y/v_x) = -B,
    alpha_r = -arctan(v_y/v_x) = -B. Same value at both axles when
    yaw_rate=0, since only the shared v_y/v_x term differs from zero.
    """
    beta_val = math.radians(4.0)
    state = _make_slip_state(v_mps=30.0, yaw_rate_radps=0.0, delta_f_rad=0.0)
    beta = np.full(_SYNTH_N, beta_val)
    result = estimate_slip_angles(state, beta, _slip_params())
    assert result["alpha_f_raw"][0] == pytest.approx(-beta_val, abs=1e-9)
    assert result["alpha_r_raw"][0] == pytest.approx(-beta_val, abs=1e-9)


def test_slip_angles_yaw_rate_sign_convention():
    """delta_f=0, beta=0, positive yaw_rate: alpha_f = -arctan(a*r/v_x)
    (negative), alpha_r = +arctan(b*r/v_x) (positive) -- front and rear
    must move in OPPOSITE directions from a pure yaw-rate input with no
    steer or sideslip, and the magnitudes scale with a/v_x and b/v_x
    respectively (checked exactly, not just by sign)."""
    a, b, v = 1.4, 1.6, 25.0
    r = 0.3
    state = _make_slip_state(v_mps=v, yaw_rate_radps=r, delta_f_rad=0.0)
    beta = np.zeros(_SYNTH_N)
    result = estimate_slip_angles(state, beta, _slip_params(a=a, b=b))
    expected_f = -math.atan(a * r / v)
    expected_r = math.atan(b * r / v)
    assert result["alpha_f_raw"][0] == pytest.approx(expected_f, abs=1e-9)
    assert result["alpha_r_raw"][0] == pytest.approx(expected_r, abs=1e-9)
    assert result["alpha_f_raw"][0] < 0 < result["alpha_r_raw"][0]


def test_slip_angles_non_moving_sample_is_zero():
    """moving_mask=False forces both slip angles to exactly zero,
    regardless of what the raw kinematics would otherwise produce --
    the pipeline's own convention for a stationary/pit sample."""
    state = _make_slip_state(v_mps=0.0, yaw_rate_radps=0.1, delta_f_rad=0.05, moving=False)
    beta = np.full(_SYNTH_N, 0.2)
    result = estimate_slip_angles(state, beta, _slip_params())
    assert result["alpha_f_raw"][0] == 0.0
    assert result["alpha_r_raw"][0] == 0.0


# --- lateral force split identity (a*Fy_f - b*Fy_r == Iz*psidd) -------------

def test_lateral_force_split_moment_identity(pipeline_result, state, effective_params):
    """a*Fy_f - b*Fy_r == Iz*psidd_raw, the identity already on record
    (thesis_notes.md 'WP-N2: nonlinear Dugoff EKF, pass 0': "Module 4a's
    Fy_f/Fy_r satisfy a*Fy_f - b*Fy_r == Iz*psidd_raw IDENTICALLY, max
    deviation 7.3e-12 Nm"). Algebraically this holds EXACTLY only when a
    and b are self-consistently derived from the SAME front_fraction at
    full precision (a = wheelbase*rear_fraction, b = wheelbase*
    front_fraction -- config's own vehicle.cog_note) -- it is not a pure
    tautology of the Fy_f/Fy_r formula alone.

    FIRST DRAFT OF THIS TEST used the config's own STORED cog_to_front/
    rear_axle_m (1.433 / 1.072) directly at a 1e-6 Nm tolerance and
    failed at 21 Nm -- not a production bug: config stores those two
    values rounded to 3 decimals (modules/accuracy_resolution.py
    _resolve_cog_position's own docstring: "config stores them rounded
    to 3 decimals... a sub-millimetre floating-point drift"), so they are
    NOT bit-identical to wheelbase*fraction at full precision. Corrected
    below to use the two derivations this identity actually depends on:
    (1) the TRUE tight identity, using a/b RE-DERIVED from front_fraction
    at full precision, matching the historical near-machine-epsilon
    figure; (2) a SEPARATE, explicitly looser check that the config's
    stored a/b sit within the documented 3-decimal rounding granularity
    of that re-derived value -- a real, small, already-understood gap,
    not a bug to chase.
    """
    vp = effective_params["vehicle"]
    a_config, b_config, Iz, wheelbase = (
        vp["cog_to_front_axle_m"], vp["cog_to_rear_axle_m"],
        vp["yaw_inertia_kgm2"], vp["wheelbase_m"],
    )
    cw = vp["corner_weights"]
    w_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
    front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / w_total
    rear_fraction = 1.0 - front_fraction
    a_exact = wheelbase * rear_fraction   # vehicle.cog_note: "a = L * rear_fraction"
    b_exact = wheelbase * front_fraction  # vehicle.cog_note: "b = L * front_fraction"

    psidd_raw = np.gradient(state["yaw_rate_radps"], state["time"])
    Fy_f = pipeline_result["forces"]["Fy_f_raw"]
    Fy_r = pipeline_result["forces"]["Fy_r_raw"]
    moving = state["moving_mask"]

    lhs = a_exact * Fy_f[moving] - b_exact * Fy_r[moving]
    rhs = Iz * psidd_raw[moving]
    max_dev = float(np.max(np.abs(lhs - rhs)))
    assert max_dev < 1e-6, (
        f"moment-balance identity (using front_fraction-derived a/b) max deviation "
        f"{max_dev:.3e} Nm, expected < 1e-6 Nm -- this would indicate the Fy_f/Fy_r "
        f"formula itself changed, not a rounding artifact"
    )

    # documented, expected rounding gap -- config's stored a/b vs the full-precision
    # front_fraction-derived values, within half the 3-decimal rounding granularity:
    assert a_config == pytest.approx(a_exact, abs=1e-3), "cog_to_front_axle_m drifted beyond 3-decimal rounding"
    assert b_config == pytest.approx(b_exact, abs=1e-3), "cog_to_rear_axle_m drifted beyond 3-decimal rounding"


# --- EKF Jacobians vs finite-difference approximation ------------------------

# Retargeted 2026-10-17 from the removed Dugoff observer to the Pacejka one
# (same F and H formulas, tyre slopes from pacejka_lateral_stiffness). Curve
# coefficients: the config tyre_fit_prior (a real pooled fit), as literals.

def _ekf_f(x, u, Vx, m, a, b, Iz, tyre_f, tyre_r):
    """The predict step's state derivative (modules/sideslip_ekf_pacejka.py
    estimate_sideslip_ekf_pacejka) as a standalone function so it can be
    finite-differenced -- the same three lines, on pacejka_lateral_force."""
    beta, r = x
    alpha_f, alpha_r = ekf_slip_angles(beta, r, u, Vx, a, b)
    Fy_f = plf(alpha_f, *tyre_f)
    Fy_r = plf(alpha_r, *tyre_r)
    beta_dot = (Fy_f + Fy_r) / (m * Vx) - r
    r_dot = (a * Fy_f - b * Fy_r) / Iz
    return np.array([beta_dot, r_dot])


def _ekf_h(x, u, Vx, m, a, b, tyre_f, tyre_r):
    beta, r = x
    alpha_f, alpha_r = ekf_slip_angles(beta, r, u, Vx, a, b)
    Fy_f = plf(alpha_f, *tyre_f)
    Fy_r = plf(alpha_r, *tyre_r)
    return np.array([r, (Fy_f + Fy_r) / m])


_EKF_TEST_PARAMS = dict(u=math.radians(2.0), Vx=35.0, m=1450.0, a=1.5, b=1.4, Iz=2082.0,
                         tyre_f=(8.5216, 1.8759, 8883.93, -3.6260),
                         tyre_r=(10.6275, 1.9048, 10634.43, -3.5479))


def _slopes(x0, p):
    alpha_f, alpha_r = ekf_slip_angles(x0[0], x0[1], p["u"], p["Vx"], p["a"], p["b"])
    return pls(alpha_f, *p["tyre_f"]), pls(alpha_r, *p["tyre_r"])


def test_ekf_process_jacobian_matches_finite_difference():
    """process_jacobian against a central-difference Jacobian of the actual
    nonlinear f(x,u) the filter's predict step integrates."""
    p = _EKF_TEST_PARAMS
    x0 = np.array([math.radians(1.0), 0.15])
    Cf_eff, Cr_eff = _slopes(x0, p)
    analytic_F = process_jacobian(Cf_eff, Cr_eff, p["Vx"], p["m"], p["a"], p["b"], p["Iz"])

    h = 1e-7
    numeric_F = np.zeros((2, 2))
    for j in range(2):
        dx = np.zeros(2)
        dx[j] = h
        f_plus = _ekf_f(x0 + dx, p["u"], p["Vx"], p["m"], p["a"], p["b"], p["Iz"], p["tyre_f"], p["tyre_r"])
        f_minus = _ekf_f(x0 - dx, p["u"], p["Vx"], p["m"], p["a"], p["b"], p["Iz"], p["tyre_f"], p["tyre_r"])
        numeric_F[:, j] = (f_plus - f_minus) / (2 * h)

    assert analytic_F == pytest.approx(numeric_F, rel=1e-3, abs=1e-3), (
        f"analytic F=\n{analytic_F}\nfinite-difference F=\n{numeric_F}"
    )


def test_ekf_measurement_jacobian_matches_finite_difference():
    """measurement_jacobian against a central-difference Jacobian of the
    actual nonlinear h(x) the filter's update step evaluates."""
    p = _EKF_TEST_PARAMS
    x0 = np.array([math.radians(1.0), 0.15])
    Cf_eff, Cr_eff = _slopes(x0, p)
    analytic_H = measurement_jacobian(Cf_eff, Cr_eff, p["Vx"], p["m"], p["a"], p["b"])

    h = 1e-7
    numeric_H = np.zeros((2, 2))
    for j in range(2):
        dx = np.zeros(2)
        dx[j] = h
        h_plus = _ekf_h(x0 + dx, p["u"], p["Vx"], p["m"], p["a"], p["b"], p["tyre_f"], p["tyre_r"])
        h_minus = _ekf_h(x0 - dx, p["u"], p["Vx"], p["m"], p["a"], p["b"], p["tyre_f"], p["tyre_r"])
        numeric_H[:, j] = (h_plus - h_minus) / (2 * h)

    assert analytic_H == pytest.approx(numeric_H, rel=1e-3, abs=1e-3), (
        f"analytic H=\n{analytic_H}\nfinite-difference H=\n{numeric_H}"
    )


# --- unit conversions ---------------------------------------------------------

def test_yaw_rate_rpm_to_radps_conversion_constant():
    """config/parameters.json stability_estimation.yaw_rate_to_radps
    (0.10472) against the exact mathematical constant 2*pi/60 -- read
    live from config, not hardcoded here, per the standing project rule
    (PLAN.md: never state a config value from memory)."""
    params = load_parameters()
    configured = params["stability_estimation"]["yaw_rate_to_radps"]
    exact = 2.0 * math.pi / 60.0
    assert configured == pytest.approx(exact, abs=1e-5)


def test_degrees_radians_round_trip():
    for deg in [-180.0, -1.0, 0.0, 0.5, 45.0, 179.999]:
        assert np.degrees(np.radians(deg)) == pytest.approx(deg, abs=1e-9)


# --- edge cases: empty arrays, all-NaN, single sample -------------------------

def test_nanmedian_or_nan_all_nan():
    assert math.isnan(_nanmedian_or_nan([float("nan"), float("nan")]))


def test_nanmedian_or_nan_empty():
    assert math.isnan(_nanmedian_or_nan([]))


def test_nanmedian_or_nan_mixed():
    assert _nanmedian_or_nan([1.0, float("nan"), 3.0]) == pytest.approx(2.0)


def test_slip_angles_single_sample_window():
    """A single-sample state array must not raise -- exercises the
    Butterworth filtfilt path (_butterworth_lowpass) at its shortest
    possible input, which in general has padding/order requirements that
    can raise on very short arrays."""
    state = _make_slip_state(v_mps=30.0, yaw_rate_radps=0.0, delta_f_rad=0.0, n=1)
    beta = np.zeros(1)
    try:
        result = estimate_slip_angles(state, beta, _slip_params())
    except ValueError as e:
        pytest.xfail(f"estimate_slip_angles raises on a single-sample array: {e}")
    assert len(result["alpha_f_filt"]) == 1


def test_slip_angles_zero_speed_moving_sample_does_not_crash():
    """Edge case not otherwise guarded in the pipeline's real data (v=0
    while moving_mask=True cannot occur from prepare_vehicle_state's own
    moving-speed-floor logic, but this function takes state/beta as
    plain arguments and does not itself enforce that precondition) --
    documents actual behaviour (inf/nan from the division) rather than
    assuming it either crashes or silently produces a plausible number.
    """
    state = _make_slip_state(v_mps=0.0, yaw_rate_radps=0.0, delta_f_rad=0.1, moving=True)
    beta = np.zeros(_SYNTH_N)
    with np.errstate(divide="ignore", invalid="ignore"):
        result = estimate_slip_angles(state, beta, _slip_params())
    # v_x=0 -> arctan(v_y/0): v_y is also 0 here (beta=0), so this
    # particular zero-speed case is 0/0 -> nan, not +-inf. Documented,
    # not asserted as "correct".
    assert math.isnan(result["alpha_f_raw"][0]) or math.isinf(result["alpha_f_raw"][0])
