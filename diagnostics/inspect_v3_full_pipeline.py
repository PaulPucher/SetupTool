# v3 work package, Phase 1 (2026-09-02): full production pipeline
# (Modules 1-6, ekf_auto_pacejka) against GT3_PRC_MLA-v3.txt, same call
# order as ui/views/outing_form.py's StabilityAnalysisThread.run(). Also
# the confirmation run for the two priority bug fixes (fastest-lap
# candidate-selection consistency in modules/csv_parser.py). Read-only,
# no config/production-behaviour changes made by this script itself.
# Disposable per CLAUDE.md's diagnostics/ rule -- finding recorded in
# thesis_notes.md, script deleted once the report is written.

import traceback

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.tyre_fit_auto import resolve_sideslip_beta
from ui.views.outing_form import OutingForm

RAW_FILE = "GT3_PRC_MLA-v3.txt"


def classify_fn(summary):
    return OutingForm._classify_corner(None, summary)


def main():
    print("--- parse_csv ---")
    data = parse_csv(RAW_FILE)
    print(f"metadata: CarName={data['metadata'].get('CarName')} TrackName={data['metadata'].get('TrackName')}")
    print(f"measured_sample_rate_hz: {data['measured_sample_rate_hz']}")
    for lap in data["laps"]:
        print(f"  lap={lap['lap_number']:>3} dur={lap['lap_time']:.2f} "
              f"outlap={lap['is_outlap']} inlap={lap['is_inlap']} "
              f"fastest={lap['is_fastest']} valid={lap['is_valid_for_analysis']} "
              f"warnings={lap['warnings']}")
    print(f"corners detected (raw, pre-stable-id): {len(data['corners'])}")
    stable_ids = {c['stable_corner_id'] for c in data['corners']} - {None}
    print(f"distinct stable_corner_id count: {len(stable_ids)}")

    print("\n--- Modules 1-5 (ekf_auto_pacejka) ---")
    params = load_parameters()
    live_default = params["stability_estimation"].get("sideslip_source", "kinematic")
    print(f"live sideslip_source: {live_default!r}")

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        print("prepare_vehicle_state returned None -- required channels missing")
        return
    print(f"state time base: {len(state['time'])} samples at {state['sample_rate_hz']:.2f} Hz")

    try:
        beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
            state, params, data, live_default, csv_path=RAW_FILE
        )
    except Exception:
        print("\n*** CRASH during resolve_sideslip_beta ***")
        traceback.print_exc()
        return
    print(f"fallback_used={fallback_used} fallback_reason={fallback_reason}")
    print(f"fit_manifest keys: {list(fit_manifest.keys()) if fit_manifest else None}")
    print(f"gate_verdict: {gate_verdict}")

    try:
        slip = estimate_slip_angles(state, beta, params)
        forces = estimate_lateral_forces(state, params)
        cs = estimate_cornering_stiffness(slip, forces, state, params)
        stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
        fz = estimate_vertical_loads(state, forces, params)
        long_forces = estimate_longitudinal_forces(state, data["channels"], params)
        slip_ratio = estimate_slip_ratio(state, data["channels"], params)
        ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)
        corners = data.get("corners", [])
        summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls, lap_filter=None)
    except Exception:
        print("\n*** CRASH during Modules 1-5/summarise_corners ***")
        traceback.print_exc()
        return

    print(f"\nsummaries: n={len(summaries)}")

    print("\n--- Verdict summary (classify_fn per corner, worst-lap aggregate) ---")
    from modules.recommendation import aggregate_by_corner
    aggregated = aggregate_by_corner(summaries)
    for cid, corner in sorted(aggregated.items()):
        severity, short, _long, _colour = classify_fn(corner)
        if severity != "normal":
            print(f"  C{cid} ({corner['speed_class']}): {short} [{severity}]")
    normal_count = sum(1 for cid, corner in aggregated.items()
                       if classify_fn(corner)[0] == "normal")
    print(f"total stable corners: {len(aggregated)}, normal: {normal_count}, "
          f"non-normal: {len(aggregated) - normal_count}")

    print("\nALL PHASES COMPLETED WITHOUT EXCEPTION")


if __name__ == "__main__":
    main()
