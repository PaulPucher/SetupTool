# Shared fixtures for SetupTool's regression test suite (tests/).
#
# REGRESSION, NOT CORRECTNESS: every fixture here reproduces CURRENT
# production behaviour so tests can detect unintended future changes. A
# fixture value being used as an expected/golden baseline does NOT mean
# it is scientifically correct -- some of what this suite pins may still
# be wrong. See each test module's own docstring for what it specifically
# does and does not establish.
#
# Session-scoped: the full Modules 1-6 chain is expensive (estimate_
# cornering_stiffness alone runs ~106s on this machine, see thesis_notes.
# md "WP-N2 Step 1a" for the measured breakdown) -- computed once per test
# run, not once per test function.

import pytest

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

# FIXED accuracy cap for every test in this suite: Level 1, not "Best
# available". Chosen for reproducibility, not realism -- "Best available"
# resolves vehicle.steering_ratio to a Level-4 lookup table sourced from
# config/car_data.json, which is in the protected set (gitignored,
# machine-local, never committed -- CLAUDE.md/PLAN.md). A golden file
# generated under "Best available" would silently fail to reproduce on
# any clone that lacks that file (e.g. the eventual submission branch),
# for a reason that has nothing to do with a real regression. cap=1
# forces every dynamically-resolved node (mass, corner_weights,
# cog_position, steering_ratio) to its config/parameters.json default,
# independent of any local/gitignored file or session setup_data.
FIXED_CAP = 1


@pytest.fixture(scope="session")
def raw_params():
    return load_parameters()


@pytest.fixture(scope="session")
def parsed_data():
    return parse_csv(RAW_FILE)


@pytest.fixture(scope="session")
def effective_params(raw_params):
    resolved = resolve_accuracy(raw_params, setup_data=None, cap=FIXED_CAP)
    return apply_resolved_vehicle(raw_params, resolved)


@pytest.fixture(scope="session")
def state(parsed_data, effective_params):
    s = prepare_vehicle_state(parsed_data["channels"], effective_params)
    assert s is not None, "prepare_vehicle_state returned None -- required channels missing from Dubai sample"
    return s


@pytest.fixture(scope="session")
def pipeline_result(parsed_data, effective_params, state):
    """Full Modules 1-6 chain, replicating ui/views/outing_form.py's
    StabilityAnalysisThread.run() non-cache-hit branch function-for-
    function (same call order, same arguments) at whatever
    stability_estimation.sideslip_source the live config carries --
    asserted below to equal "ekf_auto_pacejka", the production DEFAULT
    since 2026-09-01 (thesis_notes.md "Production sideslip source set
    to ekf_auto_pacejka"), rather than assumed, so a future default
    change is caught here, not silently tested against a stale
    assumption. Prior to 2026-09-01 this fixture hard-asserted
    "kinematic" and running the suite against a live config already
    pointed at an auto mode required manually flipping
    config/parameters.json to kinematic and restoring it afterward --
    that assertion now matches the live default directly instead of
    fighting it, which is what removed the need to flip. "kinematic"
    and "ekf_auto_dugoff" still get golden-file regression coverage,
    just as explicit, live-config-independent secondary modes in
    tests/test_golden_auto_modes.py rather than through this fixture.

    Uses modules.tyre_fit_auto.resolve_sideslip_beta -- the same
    dispatch function StabilityAnalysisThread.run() calls -- so this
    fixture can never silently diverge from what the app itself would
    compute for beta, the same reason ekf_auto_dugoff/ekf_auto_pacejka
    testing in test_golden_auto_modes.py uses it too.

    Deliberately does not import ui/views/outing_form.py itself: that
    module is PyQt6-based (QThread), and the config-switch branch it
    adds (WP-N2 Step 1b) is a one-line dispatch already covered by
    reading its source in Phase 4's schema-integrity checks -- pulling
    Qt into a headless test run for a two-line branch was judged not
    worth the fragility (see the final report's "chose not to do"
    section).
    """
    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    assert live_default == "ekf_auto_pacejka", (
        f"config default changed to {live_default!r} -- this fixture (and the golden files it feeds) "
        "assume 'ekf_auto_pacejka'; regenerate golden files deliberately if this is an intended change"
    )

    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, parsed_data, live_default, csv_path=RAW_FILE
    )
    # fallback_used is NOT asserted here -- returned instead so callers get one
    # clear, dedicated failure (test_golden_pipeline.py's test_pipeline_did_not_
    # fall_back) rather than every pipeline_result-dependent test erroring out
    # on fixture setup, same pattern as test_golden_auto_modes.py's fixture.
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, parsed_data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params)
    corners = parsed_data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)

    return {
        "beta": beta,
        "slip": slip,
        "forces": forces,
        "cs": cs,
        "stab": stab,
        "fz": fz,
        "corners": corners,
        "summaries": summaries,
        "fit_manifest": fit_manifest,
        "gate_verdict": gate_verdict,
        "fallback_used": fallback_used,
        "fallback_reason": fallback_reason,
    }


@pytest.fixture(scope="session")
def valid_lap_corners(parsed_data):
    """Corners belonging to a lap flagged is_valid_for_analysis, sorted
    by (lap_number, apex_time) -- the population Phase 2's invariant
    tests run against, matching diagnostics/inspect_entry1_brake_fix_
    verification.py's own scoping.
    """
    laps = parsed_data.get("laps", [])
    valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    corners = [c for c in parsed_data.get("corners", []) if c["lap_number"] in valid_lap_numbers]
    return sorted(corners, key=lambda c: (c["lap_number"], c["apex_time"]))


@pytest.fixture(scope="session")
def laps_by_number(parsed_data):
    return {l["lap_number"]: l for l in parsed_data.get("laps", [])}
