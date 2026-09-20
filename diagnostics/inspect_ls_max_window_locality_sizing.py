# Metrology extension Phase 2b: LS_ratio max_window_m (locality cap)
# sizing, mirroring modules.stability_analysis's own cs_max_window_m
# REVISION methodology (thesis_notes.md "CS validity repair, part A,
# Phase 1 REVISION": measure the natural, UNCAPPED window's own metre
# footprint under the just-chosen floors, restricted to genuine demand
# samples only, size the cap at 1.5x the more demanding axle/session's
# own median). Fast path, no EKF needed.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import _filtered, reconstruct_ls_window_start

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

MIN_WINDOW_SAMPLES = 40  # Phase 2a's own chosen floor (governing case: V3 rear, L=20)
MIN_SLIP_SPAN = 0.016
COMPUTATIONAL_MAX_WINDOW_M = 1.0e6


def run_session(label, raw_file):
    params = load_parameters()
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    sr = state["sample_rate_hz"]
    s_m = state["s_m"]

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip = estimate_slip_ratio(state, data["channels"], params)
    ls_cfg = params["longitudinal_stiffness"]

    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    ax = state["ax_mps2"]
    demand_mask = moving & (np.abs(ax) > 1.0)

    print(f"\n{'='*80}\n{label}\n{'='*80}")
    for axle_label, kappa_raw, fx_raw in (("front", slip["kappa_f"], long_forces["fx_f_N"]),
                                            ("rear", slip["kappa_r"], long_forces["fx_r_N"])):
        kappa = _filtered(kappa_raw, sr, ls_cfg["cutoff_hz"])
        n = len(kappa)
        footprints_m = []
        for i in range(MIN_WINDOW_SAMPLES, n):
            if not demand_mask[i]:
                continue
            start = reconstruct_ls_window_start(kappa, i, MIN_WINDOW_SAMPLES, MIN_SLIP_SPAN,
                                                 s_m=s_m, max_window_m=COMPUTATIONAL_MAX_WINDOW_M)
            s_start, s_end = s_m[start], s_m[i - 1]
            if np.isfinite(s_start) and np.isfinite(s_end):
                footprints_m.append(abs(s_end - s_start))
        footprints_m = np.array(footprints_m)
        pcts = np.percentile(footprints_m, [50, 90, 95, 99])
        print(f"  {axle_label}: n={len(footprints_m)}, median={pcts[0]:.1f}m, p90={pcts[1]:.1f}m, "
              f"p95={pcts[2]:.1f}m, p99={pcts[3]:.1f}m, max={footprints_m.max():.1f}m")


def main():
    for label, raw_file in (("DUBAI", DUBAI_FILE), ("V3", V3_FILE)):
        run_session(label, raw_file)


if __name__ == "__main__":
    main()
