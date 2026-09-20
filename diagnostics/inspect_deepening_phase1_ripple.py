# Deepening work package, Phase 1: mandatory ripple check for the v3
# corner-weight update (295.2/297.9/397.8/379.9, mass from explicit
# total_weight=1370.9 -> 300.3/298.9/396.2/386.5, mass from corner-weight
# sum after clearing total_weight). Read-only against the real file;
# the DB write already happened (user-approved) -- this script does NOT
# touch the DB, it reconstructs the OLD setup_data dict from the
# pre-edit census to run a true before/after through the real production
# chain (modules.accuracy_resolution -> modules.stability_analysis), at
# cap=None ("best available"), the same resolution production actually
# uses for a real outing (not cap=1, which the golden/diagnostic
# convention uses specifically to bypass session-measured Level 2 data
# for reproducibility -- that convention is exactly why this ripple was
# invisible to every prior diagnostic in this project).
#
# Pre-registered (work order): small shifts, no flips. A flip = STOP.

import copy

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta

V3_FILE = "GT3_PRC_MLA-v3.txt"

OLD_CAR_SETUP = {
    "corner_weight_fl": 295.2, "corner_weight_fr": 297.9,
    "corner_weight_rl": 397.8, "corner_weight_rr": 379.9,
    "total_weight": 1370.9, "cross_percentage": 50.7,
}
NEW_CAR_SETUP = {
    "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
    "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    "total_weight": 0.0, "cross_percentage": 0.0,
}


def classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def run_pipeline(setup_car_dict, label):
    print(f"\n--- {label} ---")
    params = load_parameters()
    setup_data = {"car": setup_car_dict}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    print(f"  mass resolution: level={resolved['levels']['mass']} value={resolved['values']['mass_kg']:.2f} kg")
    print(f"  corner_weights resolution: level={resolved['levels']['corner_weights']} value={resolved['values']['corner_weights']}")
    print(f"  cog_to_front_axle_m={resolved['values']['cog_to_front_axle_m']:.4f} "
          f"cog_to_rear_axle_m={resolved['values']['cog_to_rear_axle_m']:.4f}")
    if resolved["warnings"]:
        print(f"  ** WARNINGS: {resolved['warnings']}")
    effective_params = apply_resolved_vehicle(params, resolved)

    cw = effective_params["vehicle"]["corner_weights"]
    w_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
    front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / w_total
    print(f"  front_fraction={front_fraction*100:.3f}%  mass_kg={effective_params['vehicle']['mass_kg']:.2f}")

    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    data = parse_csv(V3_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, live_default, csv_path=V3_FILE
    )
    if fallback_used:
        print(f"  ** NOTE: fell back to kinematic: {fallback_reason}")
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params,
                                  channels=data["channels"], car_data=load_car_data())
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)
    return front_fraction, effective_params["vehicle"]["mass_kg"], summaries


def _classify_all(summaries):
    from modules.recommendation import aggregate_by_corner, PHASE_KEYS
    aggregated = aggregate_by_corner(summaries)
    out = {}
    for cid, corner in aggregated.items():
        for phase in PHASE_KEYS:
            if phase not in corner["phases"]:
                continue
            sliced = {"phases": {phase: corner["phases"][phase]}}
            if phase == "apex_3" and corner.get("apex_region") is not None:
                sliced["apex_region"] = corner["apex_region"]
            severity, short, _long, _colour = classify_fn(sliced)
            out[(cid, phase)] = (severity, short)
    return out, aggregated


def main():
    ff_old, mass_old, summaries_old = run_pipeline(OLD_CAR_SETUP, "OLD (v3 outing's own prior stored weighing)")
    ff_new, mass_new, summaries_new = run_pipeline(NEW_CAR_SETUP, "NEW (this phase's own weighing)")

    print(f"\n=== RIPPLE SUMMARY ===")
    print(f"front_fraction: {ff_old*100:.3f}% -> {ff_new*100:.3f}%  (delta {(ff_new-ff_old)*100:+.3f} pp)")
    print(f"mass_kg: {mass_old:.2f} -> {mass_new:.2f} kg  (delta {mass_new-mass_old:+.2f} kg)")

    verdicts_old, aggregated_old = _classify_all(summaries_old)
    verdicts_new, aggregated_new = _classify_all(summaries_new)

    print(f"\n=== PER-CORNER-PHASE VERDICT COMPARISON ===")
    keys = sorted(set(verdicts_old) | set(verdicts_new))
    flips = []
    changed = []
    for key in keys:
        old = verdicts_old.get(key, ("MISSING", "MISSING"))
        new = verdicts_new.get(key, ("MISSING", "MISSING"))
        if old == new:
            continue
        cid, phase = key
        old_axle = "understeer" if "understeer" in old[1] else ("oversteer" if "oversteer" in old[1] else None)
        new_axle = "understeer" if "understeer" in new[1] else ("oversteer" if "oversteer" in new[1] else None)
        is_flip = old_axle is not None and new_axle is not None and old_axle != new_axle
        tag = "FLIP" if is_flip else "shift"
        print(f"  [{tag}] C{cid} {phase}: {old[0]}/{old[1]!r} -> {new[0]}/{new[1]!r}")
        changed.append(key)
        if is_flip:
            flips.append(key)

    print(f"\n{len(changed)} of {len(keys)} corner-phase verdicts changed; {len(flips)} FLIP(s).")
    if flips:
        print("*** PRE-REGISTRATION VIOLATED: flip(s) found -- STOP condition per the work order. ***")
    else:
        print("Pre-registration HOLDS: no flips, small shifts only.")

    print(f"\n=== CS_ratio worst-lap medians, per corner/phase (front/rear), OLD -> NEW ===")
    for cid in sorted(set(aggregated_old) | set(aggregated_new)):
        for phase in ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"):
            po = aggregated_old.get(cid, {}).get("phases", {}).get(phase)
            pn = aggregated_new.get(cid, {}).get("phases", {}).get(phase)
            if po is None or pn is None:
                continue
            csf_o, csr_o = po["cs_ratio_f"]["median"], po["cs_ratio_r"]["median"]
            csf_n, csr_n = pn["cs_ratio_f"]["median"], pn["cs_ratio_r"]["median"]
            if (csf_o == csf_o or csr_o == csr_o):  # at least one finite
                d_f = "nan" if (csf_o != csf_o or csf_n != csf_n) else f"{csf_n-csf_o:+.4f}"
                d_r = "nan" if (csr_o != csr_o or csr_n != csr_n) else f"{csr_n-csr_o:+.4f}"
                print(f"  C{cid:2d} {phase:15s} CSf {csf_o:7.3f}->{csf_n:7.3f} (d={d_f})  "
                      f"CSr {csr_o:7.3f}->{csr_n:7.3f} (d={d_r})")


if __name__ == "__main__":
    main()
