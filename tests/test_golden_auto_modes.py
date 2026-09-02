# Originally Fresh-session work package, Phase 4e -- golden-value
# regression for the two new auto-fit modes. RETARGETED 2026-09-01
# (thesis_notes.md "Production sideslip source set to ekf_auto_pacejka"):
# ekf_auto_pacejka became the production default and moved to PRIMARY
# golden coverage in tests/test_golden_pipeline.py (via tests/conftest.py's
# live-config-driven pipeline_result fixture); this file now covers the
# two SECONDARY, explicit-mode regression sets -- "kinematic" (the prior
# default) and "ekf_auto_dugoff" (the auto-fit alternative never made
# default). Both are tested via resolve_sideslip_beta called with an
# explicit mode string, independent of whatever the live config's
# sideslip_source actually is -- this is what lets the full suite run
# against any live config value without flipping it first.
#
# Same REGRESSION-NOT-CORRECTNESS caveat as tests/test_golden_pipeline.py:
# pins CURRENT output, makes a future unintended change visible, claims
# nothing about correctness.

import json

import pytest

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from tests._json_utils import diff_json

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
FIXED_CAP = 1
GOLDEN_PATHS = {
    "kinematic": "tests/golden/pipeline_dubai_kinematic_cap1.json",
    "ekf_auto_dugoff": "tests/golden/pipeline_dubai_ekf_auto_dugoff_cap1.json",
}


@pytest.fixture(scope="module")
def secondary_mode_results():
    """Both secondary modes' full pipeline output, computed once for this
    module (the ekf_auto_dugoff fit chain alone runs ~15-25s) via the
    exact production wiring function (resolve_sideslip_beta), same
    call-order/arguments as StabilityAnalysisThread.run() -- each mode
    passed explicitly, so this is independent of whatever the live
    config's own sideslip_source is set to.
    """
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)

    results = {}
    for mode in GOLDEN_PATHS:
        beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
            state, effective_params, data, mode, csv_path=RAW_FILE
        )
        slip = estimate_slip_angles(state, beta, effective_params)
        forces = estimate_lateral_forces(state, effective_params)
        cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
        stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
        fz = estimate_vertical_loads(state, forces, effective_params)
        corners = data.get("corners", [])
        summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)
        results[mode] = {
            "summaries": summaries, "fallback_used": fallback_used,
            "fallback_reason": fallback_reason, "gate_verdict": gate_verdict,
        }
    return results


def _load_golden(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        pytest.fail(
            f"{path} not found -- run `python -m tests.generate_golden_auto_modes` first "
            "(deliberately, not automatically)."
        )


@pytest.mark.parametrize("mode", list(GOLDEN_PATHS))
def test_golden_file_metadata_present(mode):
    golden = _load_golden(GOLDEN_PATHS[mode])
    meta = golden.get("_meta", {})
    for key in ("git_commit_hash", "generated_at_utc", "data_file", "sideslip_source", "accuracy_cap"):
        assert key in meta and meta[key] not in (None, ""), f"golden file _meta missing/empty {key!r}"
    assert meta["sideslip_source"] == mode
    assert meta["accuracy_cap"] == 1


@pytest.mark.parametrize("mode", list(GOLDEN_PATHS))
def test_secondary_mode_did_not_fall_back(secondary_mode_results, mode):
    """A golden comparison against an UNEXPECTED fallback run would
    silently pin kinematic numbers under a label that claims otherwise --
    checked explicitly so that failure mode reads as its own clear
    assertion, not a buried diff in the summaries comparison below.
    Trivially true for the 'kinematic' entry itself (resolve_sideslip_
    beta's fallback concept only applies to the two auto-fit modes).

    DELIBERATE EXCEPTION (CS validity repair, Phase 4, 2026-09-02,
    thesis_notes.md "Threshold anchoring + arc closure, Phase 4"):
    ekf_auto_dugoff's rear-axle mu_fz fit degenerates on Dubai under the
    final CS window floor (100 Hz grid) and now falls back to kinematic
    on every run -- a designed, loud, non-silent fallback, not a bug.
    The golden for this mode was regenerated to PIN that fallback path
    deliberately (tests/generate_golden_auto_modes.py), so fallback_used
    must be True here, not False -- asserting not-fallback for this mode
    would itself be the stale expectation now."""
    r = secondary_mode_results[mode]
    if mode == "ekf_auto_dugoff":
        assert r["fallback_used"], (
            f"{mode} did NOT fall back on this run ({r['fallback_reason']}) -- "
            "the golden was regenerated expecting a deliberate fallback (rear mu_fz "
            "degeneracy); if this mode now converges, the golden and this test's own "
            "exception both need revisiting, not silently left as-is"
        )
        return
    assert not r["fallback_used"], (
        f"{mode} fell back to kinematic on this run ({r['fallback_reason']}) -- "
        "the golden comparison below would be meaningless under fallback"
    )


@pytest.mark.parametrize("mode", list(GOLDEN_PATHS))
def test_secondary_mode_output_matches_golden(secondary_mode_results, mode):
    golden = _load_golden(GOLDEN_PATHS[mode])
    got = {"summaries": secondary_mode_results[mode]["summaries"]}
    expected = {"summaries": golden["summaries"]}
    diffs = diff_json(got, expected)
    assert not diffs, (
        f"{len(diffs)} field(s) differ from {GOLDEN_PATHS[mode]} "
        f"(golden generated at commit {golden['_meta']['git_commit_hash']}, "
        f"{golden['_meta']['generated_at_utc']}):\n" +
        "\n".join(f"  {p}: got={a!r} expected={b!r}" for p, a, b in diffs[:30]) +
        (f"\n  ... and {len(diffs) - 30} more" if len(diffs) > 30 else "")
    )


@pytest.mark.parametrize("mode", list(GOLDEN_PATHS))
def test_secondary_mode_corner_count_matches_golden(secondary_mode_results, mode):
    golden = _load_golden(GOLDEN_PATHS[mode])
    assert len(secondary_mode_results[mode]["summaries"]) == len(golden["summaries"])
