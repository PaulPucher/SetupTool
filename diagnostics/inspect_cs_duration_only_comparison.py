# Follow-up to the Phase 1 FINAL derivation: isolated DURATION-only
# comparison, holding span fixed at the chosen 0.02 rad, to directly
# answer "what did the phase-median bootstrap show at 0.1 s (the
# chair's own reference, n=10 samples @ 100 Hz) vs the chosen 0.2 s
# (n=20)?" Same cornering-only population and method as diagnostics/
# inspect_cs_phase_median_floor_derivation_v2.py -- reused directly, not
# reimplemented, via import.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
import diagnostics.inspect_cs_phase_median_floor_derivation_v2 as v2

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
CANDIDATES = [(10, 0.020), (20, 0.020)]  # n=10 (0.1s, chair's own reference) vs n=20 (0.2s, chosen)
L_VALUES = [80, 152, 324]  # 0.8s, 1.52s, 3.24s @ 100 Hz -- shortest/mid/longest of the original v2 set


def main():
    global v2
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    sample_rate_hz = state["sample_rate_hz"]
    v2._INDEX_AS_METRES = np.arange(len(state["time"]), dtype=float)

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason})")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask

    t = state["time"]
    ay_g = np.abs(state["ay_mps2"]) / 9.81
    ay_threshold_g = params["stability_estimation"]["gps_course_anchor_max_ay_g"]

    in_corner = np.zeros(len(t), dtype=bool)
    for c in data.get("corners", []):
        start_t, _ = c["segments"]["entry_1_brake"]
        _, end_t = c["segments"]["exit_5"]
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi > lo:
            in_corner[lo:hi] = True
    cornering_mask = moving & (in_corner | (ay_g > ay_threshold_g))

    rng = np.random.default_rng(42)

    for axle_label, alpha_key, Fy_key in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
        alpha = slip[alpha_key]
        Fy = forces[Fy_key]
        print(f"\n{'=' * 90}\nAXLE: {axle_label}\n{'=' * 90}")
        for L in L_VALUES:
            print(f"\n-- L={L} samples ({L / sample_rate_hz:.2f}s @ {sample_rate_hz:.0f} Hz) --")
            for n, span in CANDIDATES:
                rel_stds, no_signal, achieved_lengths, attempts = v2.phase_median_bootstrap(
                    alpha, Fy, cornering_mask, n, span, L, rng
                )
                if not rel_stds:
                    print(f"  n={n:3d} ({n/sample_rate_hz:.2f}s) span={span}: NO SIGNAL (no_signal={no_signal})")
                    continue
                med = float(np.median(rel_stds))
                p75 = float(np.percentile(rel_stds, 75))
                mean_len = float(np.mean(achieved_lengths))
                flag = "  <== clears 15%" if med <= 0.15 else "  EXCEEDS 15%"
                print(f"  n={n:3d} ({n/sample_rate_hz:.2f}s) span={span}: n_stretches={len(rel_stds)} "
                      f"median_rel_std={med:.3f} p75_rel_std={p75:.3f} mean_achieved_n={mean_len:.1f}{flag}")


if __name__ == "__main__":
    main()
