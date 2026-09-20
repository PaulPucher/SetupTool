# Deepening work package, Phase 4e: LS_ratio threshold groundwork.
# DIAGNOSTIC + PROPOSAL ONLY -- explicitly NOT shipped this phase (no
# config/production change). Distribution of LS_ratio worst-phase,
# worst-lap values on both real sessions, a percentile figure, and a
# PROPOSED traction-limited threshold anchored the same way CS_ratio's
# own STRONG_CSF/CSR were (thesis_notes.md "Threshold anchoring, Phase
# 2..."): physically at the ratio's own zero (0 = longitudinal force
# peak, same windowed-slope/low-slip-reference construction as CS_ratio,
# confirmed by reading modules/longitudinal_stiffness.py directly) minus
# a data-sized noise margin.

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.decision_frame import aggregate_ls_by_corner

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

# v3's own real weighing, this work package's Phase 1 -- Dubai's outing
# already has real setup_data (per Phase 1's own DB census), so setup_data
# is left None for Dubai (production default: resolve_accuracy falls back
# to that outing's OWN stored setup_data only inside the real app; here,
# for a pure diagnostic against config's Level-1 default, None is fine --
# this phase is about the LS_ratio distribution's own shape, not about
# reproducing a specific outing's exact mass).
V3_SETUP_DATA = {"car": {
    "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
    "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
}}


def run_pipeline(raw_file, setup_data=None):
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    if state is None:
        raise RuntimeError(f"{raw_file}: prepare_vehicle_state returned None")
    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    beta, _fm, _gate, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, live_default, csv_path=raw_file)
    if fallback_used:
        print(f"  ** NOTE: {raw_file} fell back to kinematic: {fallback_reason}")
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params,
                                  channels=data["channels"], car_data=load_car_data())
    long_forces = estimate_longitudinal_forces(state, data["channels"], effective_params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], effective_params)
    ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, effective_params)
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls, lap_filter=None)
    return summaries


def worst_phase_worst_lap_values(summaries):
    """Per corner, per axle: MIN LS_ratio across all 5 phases of the
    already worst-lap-aggregated (min-then-min) per-phase value -- same
    "worst-lap, worst-phase" framing this project's own CS threshold-
    anchoring arc used (thesis_notes.md "Threshold anchoring, Phase 2").
    """
    ls_by_corner = aggregate_ls_by_corner(summaries)
    front_vals, rear_vals = [], []
    for cid, phases in ls_by_corner.items():
        f = [p["ls_ratio_f"] for p in phases.values() if p["ls_ratio_f"] == p["ls_ratio_f"]]
        r = [p["ls_ratio_r"] for p in phases.values() if p["ls_ratio_r"] == p["ls_ratio_r"]]
        if f:
            front_vals.append((cid, min(f)))
        if r:
            rear_vals.append((cid, min(r)))
    return front_vals, rear_vals


def report(label, front_vals, rear_vals):
    print(f"\n--- {label} ---")
    for axle, vals in (("front", front_vals), ("rear", rear_vals)):
        if not vals:
            print(f"  {axle}: no valid worst-phase LS_ratio values")
            continue
        arr = np.array([v for _, v in vals])
        pct = np.percentile(arr, [0, 10, 25, 50, 75, 90, 100])
        print(f"  {axle}: n={len(arr)} percentiles[0,10,25,50,75,90,100]={np.round(pct, 3).tolist()}")
        negative = arr[arr < 0]
        print(f"    negative (beyond-peak) count: {len(negative)}/{len(arr)}"
              + (f", values={sorted(negative.tolist())}" if len(negative) else ""))
    return


def main():
    print("Running Dubai (setup_data=None -- production default, config Level-1 weighing)...")
    dubai_summaries = run_pipeline(DUBAI_FILE, setup_data=None)
    dubai_front, dubai_rear = worst_phase_worst_lap_values(dubai_summaries)
    report("Dubai", dubai_front, dubai_rear)

    print("\nRunning v3 (setup_data=this phase's own real weighing, Phase 1)...")
    v3_summaries = run_pipeline(V3_FILE, setup_data=V3_SETUP_DATA)
    v3_front, v3_rear = worst_phase_worst_lap_values(v3_summaries)
    report("v3", v3_front, v3_rear)

    all_front = np.array([v for _, v in dubai_front] + [v for _, v in v3_front])
    all_rear = np.array([v for _, v in dubai_rear] + [v for _, v in v3_rear])
    print(f"\n--- POOLED (both sessions) ---")
    for axle, arr in (("front", all_front), ("rear", all_rear)):
        if arr.size == 0:
            continue
        print(f"  {axle}: n={arr.size} percentiles[0,10,25,50,75,90,100]="
              f"{np.round(np.percentile(arr, [0,10,25,50,75,90,100]), 3).tolist()}")

    # PROPOSAL (not shipped): same anchor logic as CS_ratio's own STRONG_
    # CSF/CSR (thesis_notes.md "Threshold anchoring, Phase 2") -- physically
    # at 0 (the ratio's own peak-force point) minus a noise margin sized
    # from THIS data's own p10 (a round, defensible choice matching the
    # CS precedent's own "data-sized noise margin" framing, not gap-
    # selected against a larger population since n is small here).
    print(f"\n--- PROPOSAL (diagnostic only, NOT shipped) ---")
    for axle, arr in (("front", all_front), ("rear", all_rear)):
        if arr.size == 0:
            continue
        p10 = float(np.percentile(arr, 10))
        margin = max(0.0, -p10) if p10 < 0 else 0.0
        proposed = -round(margin, 2) if margin > 0 else 0.0
        print(f"  {axle}: p10={p10:.3f} -> PROPOSED STRONG_LS{axle[0].upper()}={proposed:.2f} "
              f"(0 minus a data-sized margin, same anchor logic as CS_ratio's own STRONG_CSF/CSR; "
              f"n={arr.size} is small -- flagged, not a confident derivation)")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    for ax, axle, dvals, vvals in ((axes[0], "front", dubai_front, v3_front),
                                    (axes[1], "rear", dubai_rear, v3_rear)):
        d = [v for _, v in dvals]
        v = [v for _, v in vvals]
        ax.hist(d, bins=12, alpha=0.6, label=f"Dubai (n={len(d)})", color="tab:blue")
        ax.hist(v, bins=12, alpha=0.6, label=f"v3 (n={len(v)})", color="tab:orange")
        ax.axvline(0.0, color="black", linewidth=1.0, linestyle="--", label="0 = longitudinal force peak")
        ax.set_title(f"LS_ratio_{axle[0]}, worst-phase worst-lap")
        ax.set_xlabel("LS_ratio")
        ax.legend(fontsize=8)
    fig.tight_layout()
    import os
    os.makedirs("diagnostics/plots_deepening", exist_ok=True)
    path = "diagnostics/plots_deepening/ls_ratio_worst_phase_distribution.png"
    fig.savefig(path, dpi=130)
    print(f"\nfigure saved: {path}")


if __name__ == "__main__":
    main()
