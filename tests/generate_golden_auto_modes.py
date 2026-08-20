# Fresh-session work package, Phase 4e: golden-file generator for the
# two new auto-fit modes. Separate file from tests/generate_golden.py
# deliberately -- "old goldens untouched", and the existing generator
# stays exactly as it was (still the kinematic/cap=1 generator, still
# asserts sideslip_source=="kinematic" before writing). NOT a test
# itself (pytest does not collect this file). Run manually and
# deliberately:
#   python -m tests.generate_golden_auto_modes
#
# REGRESSION, NOT CORRECTNESS -- same caveat as tests/generate_golden.py.
# Uses modules.tyre_fit_auto.resolve_sideslip_beta (the real production
# wiring function, not a reimplementation) so these goldens pin what
# production actually computes, not an idealised version of it.

import datetime
import json
import subprocess

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
FIXED_CAP = 1
GOLDEN_PATHS = {
    "ekf_auto_dugoff": "tests/golden/pipeline_dubai_ekf_auto_dugoff_cap1.json",
    "ekf_auto_pacejka": "tests/golden/pipeline_dubai_ekf_auto_pacejka_cap1.json",
}


def _git_hash():
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except Exception as e:
        return f"UNAVAILABLE ({e})"


def _numpy_safe(obj):
    if isinstance(obj, dict):
        return {k: _numpy_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_numpy_safe(v) for v in obj]
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return obj


def main():
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)

    for mode, path in GOLDEN_PATHS.items():
        beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
            state, effective_params, data, mode, csv_path=RAW_FILE
        )
        if fallback_used:
            raise SystemExit(
                f"refusing to generate a golden file for {mode!r} while it fell back to kinematic "
                f"({fallback_reason}) -- this mode is not currently 'ok' on Dubai, generating a "
                f"golden under a fallback would silently pin the WRONG estimator's numbers"
            )
        slip = estimate_slip_angles(state, beta, effective_params)
        forces = estimate_lateral_forces(state, effective_params)
        cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
        stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
        fz = estimate_vertical_loads(state, forces, effective_params)
        corners = data.get("corners", [])
        summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)

        payload = {
            "_meta": {
                "purpose": "REGRESSION baseline, not a correctness claim -- see "
                           "tests/test_golden_auto_modes.py's module docstring. "
                           "AUTO-FIT MODE: the underlying tyre curve is fit fresh on this data "
                           "every time modules.tyre_fit_auto.fit_session(_pacejka) runs -- this "
                           "golden pins CURRENT fit-chain output, not a frozen prior-session curve "
                           "(contrast tests/golden/pipeline_dubai_kinematic_cap1.json).",
                "git_commit_hash": _git_hash(),
                "generated_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "data_file": RAW_FILE,
                "sideslip_source": mode,
                "accuracy_cap": FIXED_CAP,
                "setup_data": None,
                "fit_manifest": fit_manifest,
                "gate_verdict": gate_verdict,
            },
            "summaries": summaries,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(_numpy_safe(payload), f, indent=2, allow_nan=True)
        print(f"wrote {path} ({len(summaries)} corner summaries, gate={gate_verdict['verdict']})")


if __name__ == "__main__":
    main()
