# Fz-integration Phase 2 gate resolution, decisive cross-check (2026-09-03,
# user instruction): for each axle/session, mu_check = (free-D fit's own D)
# / (median MEASURED Fz in that axle's fit population). If mu_check lands
# near the joint (B,C,mu,E) fit's own mu, the mu values are honest dynamic-
# load ratios (D and Fz both real, mu is just their quotient) and the joint
# fit is not doing something structurally different from the free-D fit's
# own peak force. If they diverge, the joint fit is fitting something else
# (e.g. trading mu against B/C/E along the same ambiguity direction the
# synthetic test already found, tests/test_tyre_fit_auto_mu.py) and the
# joint-fit mu should not be trusted at face value.
#
# Read-only. Reuses diagnostics/fz_mu_tyre_fit_results.json (already on
# disk, both sessions both modes) for the free-D D values and the joint
# fit's own mu -- does not re-run the expensive EKF/sweep chain. Only
# recomputes the fit population's own median Fz (cheap: state prep + one
# kinematic beta pass + estimate_lateral_forces + estimate_vertical_loads,
# no EKF sweep).

import json

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_sideslip, estimate_slip_angles, estimate_lateral_forces, estimate_vertical_loads,
)
from modules.tyre_fit_auto import _base_mask

SESSIONS = (
    ("dubai", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("v3", "GT3_PRC_MLA-v3.txt"),
)
RESULTS_PATH = "diagnostics/fz_mu_tyre_fit_results.json"


def _median_fz_in_fit_population(raw_file):
    data = parse_csv(raw_file)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    base_mask = _base_mask(state, data.get("laps", []))

    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)

    params_measured = dict(params)
    params_measured["stability_estimation"] = dict(params["stability_estimation"])
    params_measured["stability_estimation"]["vertical_load_source"] = "measured"
    fz = estimate_vertical_loads(state, forces, params_measured, channels=data["channels"], car_data=car_data)
    assert fz["vertical_load_source_used"] == "measured", f"{raw_file}: measured Fz not resolved"

    out = {}
    for axle, alpha_key, fz_key in (("front", "alpha_f_filt", "fz_f_N"), ("rear", "alpha_r_filt", "fz_r_N")):
        alpha = slip_kin[alpha_key]
        Fy = forces[f"Fy_{axle[0]}_filt"]
        Fz = fz[fz_key]
        m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz)
        out[axle] = {
            "median_fz_N": float(np.median(Fz[m2])),
            "mean_fz_N": float(np.mean(Fz[m2])),
            "n": int(m2.sum()),
        }
    return out


def main():
    with open(RESULTS_PATH, "r", encoding="utf-8") as f:
        results = json.load(f)

    print(f"{'session':<8} {'axle':<6} {'D_freeD':>10} {'median_Fz':>10} {'mu_check':>9} "
          f"{'mu_joint':>9} {'ratio(joint/check)':>18} {'pct_diff':>9}")
    for session_label, raw_file in SESSIONS:
        fz_pop = _median_fz_in_fit_population(raw_file)
        free_d = results[f"{session_label}/free-D"]["axles"]
        mu_fit = results[f"{session_label}/mu"]["axles"]
        for axle in ("front", "rear"):
            D_freeD = free_d[axle]["D"]
            median_fz = fz_pop[axle]["median_fz_N"]
            mu_check = D_freeD / median_fz
            mu_joint = mu_fit[axle]["mu"]
            ratio = mu_joint / mu_check
            pct_diff = (mu_joint - mu_check) / mu_check * 100.0
            print(f"{session_label:<8} {axle:<6} {D_freeD:>10.1f} {median_fz:>10.1f} {mu_check:>9.4f} "
                  f"{mu_joint:>9.4f} {ratio:>18.4f} {pct_diff:>+8.2f}%")


if __name__ == "__main__":
    main()
