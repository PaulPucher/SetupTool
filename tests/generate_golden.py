# One-off generator for tests/golden/*.json -- NOT a test itself (pytest
# does not collect this file; it doesn't match test_*.py). Run manually
# and deliberately, only when a golden file needs regenerating after an
# intentional behaviour change:
#   python -m tests.generate_golden
#
# REGRESSION, NOT CORRECTNESS: this snapshots CURRENT pipeline output. It
# does not claim the numbers are right -- see tests/test_golden_pipeline.py.
#
# Fixed configuration (must match tests/conftest.py exactly, or the golden
# files and the comparison test silently diverge in what they represent):
#   - Dubai sample outing, full file (all laps)
#   - sideslip_source = "kinematic" (asserted, not just assumed)
#   - accuracy cap = 1 (reproducible independent of the gitignored
#     config/car_data.json -- see conftest.py's FIXED_CAP comment)
#   - setup_data = None, feedback_data = {} (fresh outing, no session data)

import datetime
import json
import subprocess

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.recommendation import generate_recommendations, load_recommendations_config

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
FIXED_CAP = 1
PIPELINE_GOLDEN_PATH = "tests/golden/pipeline_dubai_kinematic_cap1.json"
RECOMMENDATIONS_GOLDEN_PATH = "tests/golden/recommendations_dubai_kinematic_cap1.json"


def _git_hash():
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except Exception as e:
        return f"UNAVAILABLE ({e})"


def _numpy_safe(obj):
    """json.dump can't serialise numpy scalar types (np.float64/np.int64
    etc, which summarise_corners' output should not contain but this
    guards defensively rather than trusting that by inspection alone)."""
    if isinstance(obj, dict):
        return {k: _numpy_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_numpy_safe(v) for v in obj]
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    return obj


def main():
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved)

    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    assert live_default == "kinematic", (
        f"config default is {live_default!r}, not 'kinematic' -- refusing to generate a golden "
        "file under a non-default configuration. Set config/parameters.json stability_estimation."
        "sideslip_source back to 'kinematic' first."
    )

    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)
    beta = estimate_sideslip(state, effective_params)
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params)
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)

    meta = {
        "_meta": {
            "purpose": "REGRESSION baseline, not a correctness claim -- see "
                       "tests/test_golden_pipeline.py's module docstring.",
            "git_commit_hash": _git_hash(),
            "generated_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "data_file": RAW_FILE,
            "sideslip_source": live_default,
            "accuracy_cap": FIXED_CAP,
            "setup_data": None,
            "config_snapshot": {
                "stability_estimation": effective_params["stability_estimation"],
                "classification": effective_params["classification"],
            },
        }
    }

    pipeline_payload = dict(meta)
    pipeline_payload["summaries"] = summaries
    with open(PIPELINE_GOLDEN_PATH, "w", encoding="utf-8") as f:
        json.dump(_numpy_safe(pipeline_payload), f, indent=2, allow_nan=True)
    print(f"wrote {PIPELINE_GOLDEN_PATH} ({len(summaries)} corner summaries)")

    # Recommendation engine golden -- reuses OutingForm._classify_corner
    # unmodified (self=None call, same precedent core/weekend_pdf_export.py
    # already uses in production) so this can never silently drift from
    # what the app itself would classify.
    from ui.views.outing_form import OutingForm

    def classify_fn(summary):
        return OutingForm._classify_corner(None, summary)

    rec_config = load_recommendations_config()
    results = generate_recommendations(
        summaries, classify_fn, feedback_data={}, setup_data=None, config=rec_config,
        outing=None, driving_level=None,
    )
    rec_payload = dict(meta)
    rec_payload["recommendations"] = results
    with open(RECOMMENDATIONS_GOLDEN_PATH, "w", encoding="utf-8") as f:
        json.dump(_numpy_safe(rec_payload), f, indent=2, allow_nan=True)
    print(f"wrote {RECOMMENDATIONS_GOLDEN_PATH} ({len(results)} recommendation(s))")


if __name__ == "__main__":
    main()
