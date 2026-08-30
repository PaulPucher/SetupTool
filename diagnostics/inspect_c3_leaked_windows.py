# PLAN.md STEP 3 follow-up: does C3's traction-limited finding (Phase
# 4, "PLAN.md STEP 3 Phase 4, run for real") survive a kerb-spike-
# leakage check? Read-only, before any production change this turn.
#
# For each of C3's 4 valid-lap instances, find the PHASE that produced
# its reported (worst-phase-median) rear LS_ratio, then check how many
# of that phase's valid rear-LS samples sit inside a "leaked" window
# (modules/longitudinal_stiffness.py's own 0.45s regression window
# containing a wheel-speed sample that is anomalous per diagnostics/
# inspect_kerb_wheel_speed_spikes.py's PART 2 criterion AND outside the
# current az-based kerb_mask) -- same anomaly threshold/definition,
# reproduced here rather than imported (that script is a standalone
# diagnostic, not a module).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
TARGET_STABLE_ID = 3


def worst_phase(summary, key):
    worst_val, worst_ph = None, None
    for phase in PHASE_KEYS:
        v = summary["phases"][phase][key]["median"]
        if v != v:
            continue
        if worst_val is None or v < worst_val:
            worst_val, worst_ph = v, phase
    return worst_val, worst_ph


def build_pipeline():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    live_default = params["stability_estimation"].get("sideslip_source", "kinematic")
    print(f"live config sideslip_source = {live_default!r} (left untouched, diagnostic uses kinematic)")

    state = prepare_vehicle_state(data["channels"], params)
    beta = estimate_sideslip(state, params)
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], params)
    return data, params, state, cs, stab, long_forces, slip_ratio


def main():
    data, params, state, cs, stab, long_forces, slip_ratio = build_pipeline()
    channels = data["channels"]
    t = state["time"]
    n = len(t)
    sr = state["sample_rate_hz"]
    v_ecu_kmh = state["v_mps"] * 3.6
    moving = state["moving_mask"]
    kerb_mask = state["kerb_mask"]
    ls_cfg = params["longitudinal_stiffness"]

    laps = data.get("laps", [])
    valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    racing_mask = np.zeros(n, dtype=bool)
    for l in laps:
        if l["lap_number"] in valid_lap_numbers:
            racing_mask |= (t >= l["start_time"]) & (t <= l["end_time"])
    population_mask = moving & racing_mask  # kerb NOT excluded, same as the kerb-spike script

    def interp(name):
        ch = channels.get(name)
        return np.interp(t, ch["time"], ch["data"])

    v_fl, v_fr = interp("log_speed_fl"), interp("log_speed_fr")
    v_rl, v_rr = interp("log_speed_rl"), interp("log_speed_rr")
    rear_offset = ls_cfg["rear_rolling_radius_offset"]
    v_floor_kmh = ls_cfg["min_speed_mps"] * 3.6
    speed_ok = v_ecu_kmh >= v_floor_kmh

    def wheel_kappa(v_wheel, correction=1.0):
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(speed_ok, (v_wheel / correction - v_ecu_kmh) / v_ecu_kmh, np.nan)

    kappa_wheel = {
        "fl": wheel_kappa(v_fl), "fr": wheel_kappa(v_fr),
        "rl": wheel_kappa(v_rl, 1.0 + rear_offset), "rr": wheel_kappa(v_rr, 1.0 + rear_offset),
    }
    pooled_nonkerb = np.concatenate([np.abs(kappa_wheel[w][population_mask & ~kerb_mask]) for w in kappa_wheel])
    pooled_nonkerb = pooled_nonkerb[np.isfinite(pooled_nonkerb)]
    anomaly_threshold = float(np.ceil(np.percentile(pooled_nonkerb, 99.9) * 100)) / 100.0
    print(f"anomaly threshold (re-derived fresh, same method as the kerb-spike diagnostic): {anomaly_threshold*100:.1f}%")

    kappa_axle = {"f": slip_ratio["kappa_f"], "r": slip_ratio["kappa_r"]}
    anomalous_axle = (np.abs(kappa_axle["f"]) > anomaly_threshold) | (np.abs(kappa_axle["r"]) > anomaly_threshold)
    anomalous_axle = anomalous_axle & population_mask & np.isfinite(kappa_axle["f"]) & np.isfinite(kappa_axle["r"])
    leaked = anomalous_axle & ~kerb_mask
    leaked_idx = np.where(leaked)[0]
    print(f"leaked samples in racing population: n={int(leaked.sum())}")

    half_window = max(2, int(round(ls_cfg["regression_window_s"] * sr / 2.0)))
    idx = np.arange(n)
    w_start = np.maximum(0, idx - half_window)
    w_stop = np.minimum(n, idx + half_window + 1)

    def window_leaked(i):
        lo, hi = w_start[i], w_stop[i]
        if hi <= lo:
            return False
        j = np.searchsorted(leaked_idx, lo)
        return j < len(leaked_idx) and leaked_idx[j] < hi

    from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
    ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)
    valid_r = ls["valid_r"]
    ls_ratio_r_arr = ls["LS_ratio_r"]

    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, ls=ls, lap_filter=None)

    c3_instances = [s for s in summaries if s.get("stable_corner_id") == TARGET_STABLE_ID
                    and s["lap_number"] in valid_lap_numbers]
    c3_instances.sort(key=lambda s: s["lap_number"])

    print("=" * 78)
    print(f"C3 (stable_corner_id={TARGET_STABLE_ID}) leaked-window check, per lap")
    print("=" * 78)
    for s in c3_instances:
        worst_val, worst_ph = worst_phase(s, "ls_ratio_r")
        c = next(c for c in corners if c["lap_number"] == s["lap_number"]
                 and c.get("stable_corner_id") == TARGET_STABLE_ID)
        start_t, end_t = c["segments"][worst_ph]
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        phase_idx = np.arange(lo, hi)
        phase_valid_idx = phase_idx[valid_r[lo:hi]] if hi > lo else np.array([], dtype=int)

        n_valid = len(phase_valid_idx)
        contaminated = np.array([window_leaked(i) for i in phase_valid_idx], dtype=bool)
        n_contaminated = int(contaminated.sum())

        median_all = float(np.median(ls_ratio_r_arr[phase_valid_idx])) if n_valid else float("nan")
        clean_idx = phase_valid_idx[~contaminated]
        median_clean = float(np.median(ls_ratio_r_arr[clean_idx])) if len(clean_idx) else float("nan")

        print(f"  lap={s['lap_number']}: worst phase={worst_ph}, reported ls_ratio_r={worst_val:.3f}, "
              f"t=[{start_t:.2f},{end_t:.2f}]s")
        print(f"    valid rear-LS samples in this phase: n={n_valid}, contaminated (leaked-window): n={n_contaminated} "
              f"({n_contaminated/max(n_valid,1)*100:.1f}%)")
        print(f"    median LS_ratio_r, ALL valid samples:   {median_all:.4f}")
        print(f"    median LS_ratio_r, EXCLUDING contaminated: {median_clean:.4f} "
              f"(matches reported worst-phase median only if this phase's reported worst-phase VALUE equals "
              f"this phase's own median -- reported value is whichever of the 5 phases is worst, so compare directly)")


if __name__ == "__main__":
    main()
