# Unit-level sanity checks for diagnostics/sideslip_ekf_dugoff.py
# (pass-0 nonlinear Dugoff EKF). Read-only, no production/config change
# (only reads config, writes nothing). Three sections:
#
#   1. Jacobian-collapse check: at a constant, non-saturating slip angle,
#      the new EKF's F/H (built from the Dugoff tyre model's analytic
#      stiffness) must reduce to the rejected linear observer's own A/C
#      matrices (diagnostics/sideslip_kalman_observer.py) when the same
#      fixed stiffness values are substituted in. Confirms the Jacobian
#      derivation is algebraically correct, independent of pass 0's own
#      operational parameters -- this section deliberately uses the
#      rejected filter's own Caf/Car/Iz for a literal numeric comparison
#      against its actual code, not pass 0's frozen Dugoff/Iz values.
#   2. h2-vs-ay SIGN/UNIT CONSISTENCY check (NOT validation): does the
#      measurement model's predicted ay land in the right ballpark of
#      measured ay, over a population of steady-state cornering samples.
#      This is partly circular -- see the docstring on the check function
#      -- and must never be cited later as evidence the tyre model or the
#      filter is correct.
#   3. Fy-axle dependency identity: a*Fy_f - b*Fy_r == Iz*psidd_raw,
#      verified numerically on Dubai data (this is the amendment-2 check,
#      also recorded in config/parameters.json tyre_model_ekf.pass_0.
#      fy_axle_dependency_note).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_lateral_forces,
)
from modules.tyre_model import dugoff_lateral_force, dugoff_lateral_stiffness
from diagnostics.sideslip_ekf_dugoff import slip_angles, process_jacobian, measurement_jacobian

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"


def check_jacobian_collapse(params):
    """Section 1. At alpha=0 (exact, non-saturating regardless of mu_fz --
    the lambda-division guard forces f(lambda)=1 there), dugoff_lateral_
    stiffness(0, C, mu_fz) == C exactly, so F/H must match the rejected
    filter's A/C bit-for-bit (up to floating point) when C=Caf/Car. A
    second, nonzero alpha is also checked to characterize how fast the
    match degrades -- Dugoff's Fy=C*tan(alpha)*f(lambda) is NOT linear in
    alpha even before saturation (tan != identity), so even the
    unsaturated branch's stiffness is C/cos^2(alpha), not the rejected
    filter's constant C. Exact collapse holds only in the alpha->0 limit.
    """
    vp = params["vehicle"]
    se = params["stability_estimation"]
    m = vp["mass_kg"]
    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]
    Iz_rejected = vp["yaw_inertia_kalman_kgm2"]  # rejected filter's own Iz, for this literal comparison only
    Caf = se["cs_front_fallback_reference_n_per_rad"]
    Car = se["cs_rear_fallback_reference_n_per_rad"]
    Vx = 30.0  # arbitrary representative speed, m/s
    mu_fz_placeholder = 1.0e9  # irrelevant at alpha=0, present for clarity only

    def rejected_A_C(Vx):
        A = np.array([
            [-(Caf + Car) / (m * Vx), -1.0 - (Caf * a - Car * b) / (m * Vx ** 2)],
            [-(Caf * a - Car * b) / Iz_rejected, -(Caf * a ** 2 + Car * b ** 2) / (Iz_rejected * Vx)],
        ])
        C = np.array([
            [0.0, 1.0],
            [-(Caf + Car) / m, -(Caf * a - Car * b) / (m * Vx)],
        ])
        return A, C

    A_ref, C_ref = rejected_A_C(Vx)

    print("=" * 100)
    print("SECTION 1 -- Jacobian collapse check (F/H vs rejected filter's A/C)")
    print("=" * 100)
    print(f"Vx={Vx} m/s  Caf={Caf} N/rad  Car={Car} N/rad  Iz(rejected filter)={Iz_rejected} kg*m^2")
    print()

    for alpha_test, label in ((0.0, "alpha=0.0 exact"), (0.02, "alpha=0.02 rad (~1.15 deg)")):
        Cf_eff = dugoff_lateral_stiffness(alpha_test, Caf, mu_fz_placeholder)
        Cr_eff = dugoff_lateral_stiffness(alpha_test, Car, mu_fz_placeholder)
        F = process_jacobian(Cf_eff, Cr_eff, Vx, m, a, b, Iz_rejected)
        H = measurement_jacobian(Cf_eff, Cr_eff, Vx, m, a, b)

        F_diff = np.max(np.abs(F - A_ref))
        H_diff = np.max(np.abs(H - C_ref))
        print(f"  {label}: Cf_eff={Cf_eff:.4f} (vs Caf={Caf})  Cr_eff={Cr_eff:.4f} (vs Car={Car})")
        print(f"    max|F-A|={F_diff:.6e}   max|H-C|={H_diff:.6e}")
    print()


def check_h2_vs_ay_consistency(state, params, data, verbose=True):
    """Section 2. SIGN/UNIT CONSISTENCY CHECK ONLY -- not validation, and
    not independent evidence the model is correct. estimate_lateral_
    forces builds Fy_f/Fy_r (Module 4a) directly from measured ay
    (Fy_total = m*ay) plus measured yaw acceleration; WP-N1b's frozen
    c_alpha/mu_fz were fit against those same forces. So a close match
    between this check's predicted h2=(Fy_f+Fy_r)/m and measured ay is
    expected partly BECAUSE of that shared ancestry, not because the
    Dugoff model has been independently confirmed against a measurement
    it never saw. This check exists only to catch a gross sign/unit
    mistake in the measurement equation, nothing stronger.

    Returns (h2_pred, ay_meas_at_population) so a caller can reuse this
    exact population (e.g. for a correlation/regression-slope sign
    check) without recomputing the mask.
    """
    vp = params["vehicle"]
    cfg = params["tyre_model_ekf"]["pass_0"]
    m = vp["mass_kg"]
    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]
    c_alpha_f, c_alpha_r = cfg["c_alpha_front_n_per_rad"], cfg["c_alpha_rear_n_per_rad"]
    mu_fz_f, mu_fz_r = cfg["mu_fz_front_N"], cfg["mu_fz_rear_N"]

    t = state["time"]
    v = state["v_mps"]
    yaw_rate = state["yaw_rate_radps"]
    delta_f = state["delta_f_rad"]
    ay_meas = state["ay_mps2"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    moving_clean = moving & ~kerb_mask if kerb_mask is not None else moving
    v_min = params["stability_estimation"]["moving_speed_min_mps"]

    laps = data.get("laps", [])
    valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
    racing_mask = np.zeros_like(t, dtype=bool)
    for s, e in valid_windows:
        racing_mask |= (t >= s) & (t <= e)
    base_mask = moving_clean & racing_mask

    # Steady-state cornering population: each corner instance's own
    # apex_3 phase (quasi-constant curvature), reusing the pipeline's own
    # phase segmentation rather than inventing a new threshold. apex_3 is
    # a single time instant (start_t==end_t, modules/corner_analysis.py
    # _build_corner) -- mirror summarise_corners' own _phase_slice apex
    # expansion (+/- apex_half_window_samples) or the population is
    # near-empty (most exact-instant slices land on zero samples).
    apex_half_window = params["stability_estimation"]["apex_half_window_samples"]
    apex_mask = np.zeros_like(t, dtype=bool)
    for c in data.get("corners", []):
        start_t, end_t = c["segments"]["apex_3"]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi <= lo:
            lo = max(0, lo - apex_half_window)
            hi = min(len(t), hi + apex_half_window + 1)
        apex_mask[lo:hi] = True

    check_mask = base_mask & apex_mask
    idx = np.where(check_mask)[0]

    beta_kinematic = estimate_sideslip(state, params)

    h2_pred = np.full(len(idx), np.nan)
    for k, i in enumerate(idx):
        Vx = max(float(v[i]), v_min)
        alpha_f, alpha_r = slip_angles(beta_kinematic[i], yaw_rate[i], delta_f[i], Vx, a, b)
        Fy_f = dugoff_lateral_force(alpha_f, c_alpha_f, mu_fz_f)
        Fy_r = dugoff_lateral_force(alpha_r, c_alpha_r, mu_fz_r)
        h2_pred[k] = (Fy_f + Fy_r) / m

    resid = h2_pred - ay_meas[idx]
    p10, p50, p90 = np.percentile(resid, [10, 50, 90])

    if verbose:
        print("=" * 100)
        print("SECTION 2 -- h2-vs-ay consistency check (steady-state/apex population, NOT validation)")
        print("=" * 100)
        print(f"n samples (apex_3 phase, valid-lap/moving/kerb-excluded): {len(idx)}")
        print(f"residual (h2_pred - ay_meas), m/s^2: p10={p10:.4f}  median={p50:.4f}  p90={p90:.4f}")
        print(f"ay_meas at these samples, m/s^2: min={np.min(ay_meas[idx]):.3f}  max={np.max(ay_meas[idx]):.3f}")
        print()

    return h2_pred, ay_meas[idx]


def check_fy_axle_identity(state, params):
    """Section 3 (amendment 2). a*Fy_f - b*Fy_r == Iz*psidd_raw
    identically, given a/b computed from the same static split as Module
    4a's front/rear_fraction. Checked both with config's own stored
    (rounded) cog_to_front/rear_axle_m and with a/b recomputed live from
    wheelbase*fraction, to separate true algebraic identity from
    config-rounding residual.
    """
    vp = params["vehicle"]
    forces = estimate_lateral_forces(state, params)

    a_cfg = vp["cog_to_front_axle_m"]
    b_cfg = vp["cog_to_rear_axle_m"]
    Iz = vp["yaw_inertia_kgm2"]
    wb = vp["wheelbase_m"]
    cw = vp["corner_weights"]
    W_f = cw["FL_kg"] + cw["FR_kg"]
    W_r = cw["RL_kg"] + cw["RR_kg"]
    front_fraction = W_f / (W_f + W_r)
    rear_fraction = W_r / (W_f + W_r)
    a_exact = wb * rear_fraction
    b_exact = wb * front_fraction

    moving = state["moving_mask"]
    t = state["time"]
    psidd_raw = np.gradient(state["yaw_rate_radps"], t)

    Fy_f = forces["Fy_f_raw"][moving]
    Fy_r = forces["Fy_r_raw"][moving]
    psidd = psidd_raw[moving]
    rhs = Iz * psidd

    print("=" * 100)
    print("SECTION 3 -- Fy-axle dependency identity: a*Fy_f - b*Fy_r == Iz*psidd_raw")
    print("=" * 100)
    print(f"config a={a_cfg:.6f} m  exact a={a_exact:.6f} m  delta={a_cfg - a_exact:.6f} m")
    print(f"config b={b_cfg:.6f} m  exact b={b_exact:.6f} m  delta={b_cfg - b_exact:.6f} m")
    print()

    for label, aa, bb in (("config a/b (stored, 3-decimal-rounded)", a_cfg, b_cfg),
                          ("exact a/b (wheelbase*fraction, live)", a_exact, b_exact)):
        diff = (aa * Fy_f - bb * Fy_r) - rhs
        print(f"  {label}: n={len(diff)}  max|diff|={np.max(np.abs(diff)):.6e} Nm  "
              f"mean|diff|={np.mean(np.abs(diff)):.6e} Nm")

    diff_cfg = (a_cfg * Fy_f - b_cfg * Fy_r) - rhs
    floor = np.percentile(np.abs(rhs), 50)
    rel_mask = np.abs(rhs) > floor
    rel = np.abs(diff_cfg[rel_mask]) / np.abs(rhs[rel_mask])
    print(f"  config a/b relative deviation (|rhs|>{floor:.1f} Nm floor, n={rel_mask.sum()}): "
          f"median={np.median(rel):.4f}  max={np.max(rel):.4f}")
    print()


if __name__ == "__main__":
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    state = prepare_vehicle_state(data["channels"], params)

    check_jacobian_collapse(params)
    check_h2_vs_ay_consistency(state, params, data)
    check_fy_axle_identity(state, params)
