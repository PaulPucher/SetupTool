# DIAGNOSTIC (read-only, [keep-reproduces]): WP-CACHE Phase 1e -- real
# sidecar size and load-wall-time measurement on both real sessions,
# production defaults (ekf_auto_pacejka, cap=1, same fixed configuration
# diagnostics/inspect_frame_stage2_parity.py's own FIXED_CAP uses).
# Mirrors ui/views/outing_form.py's StabilityAnalysisThread.run() call
# sequence and _on_stability_done's pipeline_cache_entry shape exactly
# (same functions, same dict keys) so the measured payload is the real
# thing the app would write, not an approximation. No production write --
# writes into a throwaway temp dir, never data/analysis_cache/.
#
# Acceptance gate, per the work order: v3 sidecar > ~500MB or v3 load
# wall-time > ~10s -> STOP and report, do not proceed to Phase 1's own
# close-out or Phase 2. Trimming what gets stored is a reviewer design
# decision, not this script's or the implementer's to make.

import gzip
import os
import tempfile
import time

from modules.stability_analysis import (
    ANALYSIS_SCHEMA_VERSION, load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.csv_parser import parse_csv
from modules import pipeline_sidecar

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"
FIXED_CAP = 1
SIZE_STOP_BYTES = 500 * 1024 * 1024
LOAD_TIME_STOP_S = 10.0


def _norm_path(p):
    return os.path.normcase(os.path.normpath(p)) if p else p


def run_and_build_payload(raw_file):
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved)
    sideslip_source = effective_params["stability_estimation"].get("sideslip_source", "kinematic")

    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    if state is None:
        raise RuntimeError(f"{raw_file}: prepare_vehicle_state returned None")
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, sideslip_source, csv_path=raw_file
    )
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

    # Exactly outing_form.py's own pipeline_cache_entry shape (_on_stability_done).
    payload = {
        "csv_path": _norm_path(raw_file),
        "corners": corners, "state": state, "cs": cs, "stab": stab,
        "fz": fz, "ls": ls, "slip": slip, "forces": forces,
        "accuracy_cap": FIXED_CAP,
        "resolved_vehicle_snapshot": resolved["values"],
        "sideslip_source": sideslip_source,
        "grid_rate_hz": state["sample_rate_hz"],
        "fit_manifest": fit_manifest, "gate_verdict": gate_verdict,
        "fallback_used": fallback_used, "fallback_reason": fallback_reason,
    }
    identity = pipeline_sidecar.build_identity(
        schema_version=ANALYSIS_SCHEMA_VERSION,
        csv_path=_norm_path(raw_file), accuracy_cap=FIXED_CAP,
        resolved_vehicle_snapshot=resolved["values"], sideslip_source=sideslip_source,
        grid_rate_hz=state["sample_rate_hz"], lap_filter=sorted({l["lap_number"] for l in data.get("laps", [])}),
    )
    return identity, payload, summaries


def measure(label, raw_file, sidecar_dir):
    pipeline_sidecar.SIDECAR_DIR = sidecar_dir
    print(f"\n{'='*70}\n{label}: running full pipeline (Modules 1-5 + fit chain)...\n{'='*70}")
    t0 = time.perf_counter()
    identity, payload, summaries = run_and_build_payload(raw_file)
    t1 = time.perf_counter()
    print(f"  pipeline wall time: {t1 - t0:.1f}s ({len(summaries)} corner summaries)")

    t_write0 = time.perf_counter()
    ok = pipeline_sidecar.write_sidecar(label, identity, payload)
    t_write1 = time.perf_counter()
    assert ok, f"{label}: write_sidecar reported failure"

    path = pipeline_sidecar._sidecar_path(label)
    size_bytes = os.path.getsize(path)
    print(f"  write time: {t_write1 - t_write0:.3f}s")
    print(f"  sidecar size: {size_bytes / 1024 / 1024:.2f} MB ({size_bytes} bytes)")

    t_load0 = time.perf_counter()
    loaded_payload, reason = pipeline_sidecar.load_sidecar(label, identity)
    t_load1 = time.perf_counter()
    load_time = t_load1 - t_load0
    assert reason is None, f"{label}: unexpected load mismatch/failure: {reason}"
    assert loaded_payload["corners"] == payload["corners"]
    print(f"  load time (decompress+unpickle+render-ready): {load_time:.3f}s")

    if size_bytes > SIZE_STOP_BYTES:
        print(f"  *** STOP CONDITION: size {size_bytes/1024/1024:.1f}MB exceeds "
              f"{SIZE_STOP_BYTES/1024/1024:.0f}MB gate ***")
    if load_time > LOAD_TIME_STOP_S:
        print(f"  *** STOP CONDITION: load time {load_time:.1f}s exceeds "
              f"{LOAD_TIME_STOP_S:.0f}s gate ***")
    return size_bytes, load_time


def main():
    with tempfile.TemporaryDirectory(prefix="sidecar_size_check_") as tmp:
        dubai_size, dubai_load = measure("dubai", DUBAI_FILE, tmp)
        v3_size, v3_load = measure("v3", V3_FILE, tmp)

    print(f"\n{'='*70}\nSUMMARY\n{'='*70}")
    print(f"Dubai: {dubai_size/1024/1024:.2f} MB, load {dubai_load:.3f}s")
    print(f"v3:    {v3_size/1024/1024:.2f} MB, load {v3_load:.3f}s")
    stop = v3_size > SIZE_STOP_BYTES or v3_load > LOAD_TIME_STOP_S
    print("VERDICT:", "STOP -- report before shipping" if stop else "within gate, proceed")


if __name__ == "__main__":
    main()
