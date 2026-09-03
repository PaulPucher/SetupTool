# Damper package, Phase 5: aero diagnostic from damper-derived total wheel
# load, GT3_PRC_MLA-v3.txt. [keep-reproduces] per diagnostics/README.md --
# a reusable check for any future damper-equipped session, not a one-off.
# Read-only, no config/production changes -- config vehicle.aero.lift_
# coeff/cross_track_area_m2 stay at their 0.0 placeholders regardless of
# what this script finds (their PRODUCT, not either one alone, is what a
# constant-speed v^2 regression can identify; recovering Cl on its own
# needs a separately known reference area, which this project does not
# have -- reported honestly as a diagnostic finding, not written back).
#
# METHOD: total damper-derived Fz (sum of 4 corners, combined with the
# static-split fallback per corner exactly as modules/wheel_loads.py's
# own combine_with_static_fallback intends) regressed against v^2 with an
# ax term included to remove residual longitudinal-transfer contamination
# from samples that are not perfectly zero-ax ("straights" is a speed
# range, not a zero-acceleration instant): Fz_total = a + b*ax + c*v^2.
# If the damper-derived load is picking up real aero downforce, c > 0 (a
# downforce car carries more total load at higher speed, independent of
# ax) and the residual scatter should be small and unstructured. Trusts
# nothing downstream: does NOT feed into CS_ratio, classification, or any
# other estimator; a pure top-level Fz regression.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state, estimate_lateral_forces,
    estimate_vertical_loads,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    if state is None or car_data is None:
        print("cannot run -- missing state or car_data.json")
        return

    forces = estimate_lateral_forces(state, params)
    fz_static = estimate_vertical_loads(state, forces, params)
    static_fallback_fz = {
        "fl": fz_static["fz_fl_N"], "fr": fz_static["fz_fr_N"],
        "rl": fz_static["fz_rl_N"], "rr": fz_static["fz_rr_N"],
    }
    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    combined = combine_with_static_fallback(damper_result, static_fallback_fz)
    total_fz = sum(combined[c]["fz_N"] for c in CORNERS)

    ax = state["ax_mps2"]
    v = state["v_mps"]
    moving = v >= params["stability_estimation"]["moving_speed_min_mps"]
    # "Straights" widened from Phase 2's tight |ay|<0.5 band (n=1804, one
    # narrow speed range) to |ay|<1.5 so the regression sees real speed
    # variation across the session's straights -- still excludes any
    # meaningfully cornering sample. ax is a REGRESSOR here, not filtered
    # to near-zero, precisely so its own contaminating effect can be
    # separated out by the fit rather than by a tighter mask.
    mask = moving & (np.abs(state["ay_mps2"]) < 1.5)
    n = mask.sum()
    print(f"regression population: n={n} samples (moving, |ay|<1.5 m/s^2)")
    if n < 20:
        print("too few samples for a meaningful fit")
        return

    X = np.column_stack([np.ones(n), ax[mask], v[mask] ** 2])
    y = total_fz[mask]
    coeffs, residuals, rank, sv = np.linalg.lstsq(X, y, rcond=None)
    a, b, c = coeffs
    y_pred = X @ coeffs
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    resid = y - y_pred

    print(f"\nFz_total = a + b*ax + c*v^2")
    print(f"  a (N)          = {a:.1f}")
    print(f"  b (N per m/s^2)= {b:.2f}")
    print(f"  c (N per (m/s)^2) = {c:.4f}  {'(downforce-consistent, c>0)' if c > 0 else '(LIFT-consistent or noise, c<=0)'}")
    print(f"  R^2 = {r2:.4f}")
    print(f"  residual: mean={resid.mean():.1f} N, std={resid.std():.1f} N, "
          f"|resid| p90={np.percentile(np.abs(resid), 90):.1f} N")

    rho = params["vehicle"]["aero"]["air_density_kgm3"]
    cl_times_a = -2.0 * c / rho  # from c = -0.5*rho*Cl*A (config sign convention, Cl<0 for downforce)
    print(f"\nImplied Cl*A_ref product (config sign convention, Cl<0=downforce): {cl_times_a:.4f} m^2")
    print("NOT written back to config -- cross_track_area_m2 and lift_coeff both stay at their 0.0 "
          "placeholders; only their product is identifiable from this regression, per this script's own docstring.")

    v_kmh_range = (float(np.min(v[mask])) * 3.6, float(np.max(v[mask])) * 3.6)
    print(f"\nspeed range in regression population: {v_kmh_range[0]:.0f}-{v_kmh_range[1]:.0f} km/h")


if __name__ == "__main__":
    main()
