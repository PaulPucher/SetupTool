# Fresh-session work package, Phase 4: validation of the production
# wiring built in Phases 1-3. Calls modules.tyre_fit_auto.
# resolve_sideslip_beta_and_cs_inputs directly -- the exact function ui/views/
# outing_form.py's StabilityAnalysisThread.run() calls, extracted
# during Phase 1 specifically so this validation does not need a Qt
# event loop (same "not worth the fragility" reasoning tests/conftest.
# py's pipeline_result fixture already documented for the pre-existing
# ekf_pass_1 branch, now made moot by the extraction). Phase 4d's cache
# test does import ui.views.outing_form for the plain (non-Qt-instance)
# module-level cache functions -- these do not require a QApplication.
#
# No production edits in this phase -- diagnostics/tests only.

import copy

import numpy as np
import pytest

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import SIDESLIP_SOURCES, resolve_sideslip_beta_and_cs_inputs, fit_session_pacejka
from tests.conftest import DUBAI_SAMPLE, find_data_file, resolve_data_path

RAW_FILE = find_data_file(DUBAI_SAMPLE)  # None without the data drive; the fixture raises clearly
FIXED_CAP = 1  # matches tests/conftest.py's own convention


def resolve_sideslip_beta(state, params, data, sideslip_source, csv_path=None):
    # the first five returns (beta, manifest, gate, fallback, reason); the
    # public wrapper of that name was removed in the 2026-10-22 pruning
    return resolve_sideslip_beta_and_cs_inputs(state, params, data, sideslip_source, csv_path)[:5]


@pytest.fixture(scope="module")
def wiring_fixture():
    raw_params = load_parameters()
    resolved = resolve_accuracy(raw_params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(raw_params, resolved)
    # default path (fit L) since the 2026-10-22 pruning removed per_session
    data = parse_csv(resolve_data_path(DUBAI_SAMPLE))
    state = prepare_vehicle_state(data["channels"], effective_params)
    assert state is not None
    return {"params": effective_params, "data": data, "state": state}


# --- unknown sideslip_source refused (Class B fix 7, 2026-10-17) -----------
# Replaces the two Phase-4a ekf_auto_dugoff wiring tests, deleted with the
# Dugoff path.

@pytest.mark.parametrize("source", ["ekf_auto_dugoff", "ekf_pass_1", "kinematik", ""])
def test_unknown_sideslip_source_is_refused(wiring_fixture, source):
    """An unknown source used to run kinematic silently; it must stop the
    run before any estimator is called, removed Dugoff sources included."""
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    assert source not in SIDESLIP_SOURCES
    with pytest.raises(ValueError, match="not recognised"):
        resolve_sideslip_beta(state, params, data, source, csv_path=RAW_FILE)


def test_kinematic_source_is_plain_estimate_sideslip(wiring_fixture):
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, "kinematic", csv_path=RAW_FILE
    )
    assert np.array_equal(beta, estimate_sideslip(state, params))
    assert (fit_manifest, gate_verdict, fallback_used, fallback_reason) == (None, None, False, None)


# --- Phase 4b: ekf_auto_pacejka wiring ------------------------------------

def test_ekf_auto_pacejka_beta_matches_standalone_chain_exactly(wiring_fixture):
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta_wired, _, gate_verdict, fallback_used, _ = resolve_sideslip_beta(
        state, params, data, "ekf_auto_pacejka", csv_path=RAW_FILE
    )
    assert not fallback_used and gate_verdict["verdict"] != "fail"

    standalone_manifest = fit_session_pacejka(data, params, data_file_path=RAW_FILE)
    beta_standalone = standalone_manifest["beta_ekf_with_fallback"]

    assert np.array_equal(beta_wired, beta_standalone), (
        "resolve_sideslip_beta's Pacejka beta differs from a direct fit_session_pacejka "
        "call's beta_ekf_with_fallback -- wiring bug"
    )


# --- Phase 4c: forced fallback, end to end --------------------------------

def test_forced_fallback_via_impossible_gate_thresholds(wiring_fixture):
    """Params injection (config/parameters.json untouched) -- a deep
    copy of the live params with nis_gate.threshold_use_ekf/threshold_
    warn overridden so no health score can ever clear them, forcing the
    gate to verdict 'fail' regardless of the actual fit quality. Verifies
    the fallback path end to end: kinematic beta used (bit-identical to
    a direct estimate_sideslip call), reason recorded and mentions the
    gate, fit_manifest still present (the fit reached the gate; only the
    gate failed it) -- the two failure causes (degenerate fit vs failed
    gate) must stay distinguishable in the reason text for the status
    line to describe correctly.

    No assertion on the fit status (author ruling 2026-10-22, P1c): under
    fit L the Dubai fit grades 'marginal', not 'ok'; 'ok' was a property
    of the removed per_session fit. Any non-degenerate fit reaches the
    gate, which is all this test needs.
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    forced_params = copy.deepcopy(params)
    forced_params["nis_gate"]["threshold_use_ekf"] = 2.0   # health_score is a fraction in [0,1] -- unreachable
    forced_params["nis_gate"]["threshold_warn"] = 1.5      # same -- everything verdicts 'fail'

    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, forced_params, data, "ekf_auto_pacejka", csv_path=RAW_FILE
    )

    assert fallback_used is True
    assert fallback_reason is not None and "NIS gate verdict 'fail'" in fallback_reason
    assert gate_verdict is not None and gate_verdict["verdict"] == "fail"
    assert fit_manifest is not None, "the fit should have reached the gate -- only the gate was forced to fail"

    beta_kinematic = estimate_sideslip(state, forced_params)
    assert np.array_equal(beta, beta_kinematic), (
        "fallback beta is not bit-identical to a direct estimate_sideslip call"
    )


def test_forced_fallback_status_line_text_names_kinematic_and_reason(wiring_fixture):
    """Phase 3b/3d requirement, checked directly against the real
    formatter: the status line must be impossible to mistake for a
    real EKF render on fallback -- checks OutingForm._format_estimator_
    status (None-self call, same precedent as _classify_corner) produces
    text that names KINEMATIC (not the requested EKF mode) and includes the
    fallback reason. (The weekend PDF's copy of this text was removed with
    the export, 2026-10-22.)
    """
    try:
        from ui.views.outing_form import OutingForm
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")

    fallback_reason = "NIS gate verdict 'fail' (health_score=0.05, threshold_warn=2.0)"
    text, colour = OutingForm._format_estimator_status(
        None, "ekf_auto_pacejka", {"status": "ok"}, {"verdict": "fail", "health_score": 0.05},
        True, fallback_reason,
    )
    # 2026-10-06 (author app-check ruling): the reason is shown as a plain
    # sentence (raw reasons can name optimisers / models); the raw text stays
    # in the saved fit record
    plain = "the fitted tyre curve failed its self-check this session"
    assert "KINEMATIC" in text
    assert plain in text
    assert "Dugoff" not in text and "Pacejka" not in text


# --- Phase 4d: mode-switch cache test ---------------------------------------

def test_pipeline_cache_rejects_stale_entry_on_mode_switch():
    """Reproduces the exact hit-check condition from ui/views/outing_
    form.py's _run_stability_analysis (accuracy_cap, resolved_vehicle_
    snapshot, sideslip_source must all match) against real cache
    entries built via the real _pipeline_cache_put/_pipeline_cache_get
    functions -- confirms a mode switch (same csv_path, same cap, same
    resolved snapshot, different sideslip_source) is correctly treated
    as a cache MISS, and switching back to the original mode is a HIT
    again once that mode's own entry is (re-)stored. Plain module-level
    functions, no QApplication needed.
    """
    try:
        from ui.views.outing_form import (
            _pipeline_cache_put, _pipeline_cache_get, invalidate_all_pipeline_caches,
        )
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")

    invalidate_all_pipeline_caches()
    csv_path = "C:/fake/does_not_need_to_exist_for_this_test.txt"
    cap = 1
    snapshot = {"mass_kg": 1356.0}

    def hit_check(cached_entry, cap, snapshot, sideslip_source):
        # Exact condition from _run_stability_analysis, reproduced (not
        # imported -- it's inline in a much larger Qt method).
        return (cached_entry is not None
                and cached_entry.get("accuracy_cap") == cap
                and cached_entry.get("resolved_vehicle_snapshot") == snapshot
                and cached_entry.get("sideslip_source") == sideslip_source)

    _pipeline_cache_put(csv_path, {
        "corners": [], "state": None, "cs": None, "stab": None, "fz": None,
        "slip": None, "forces": None, "accuracy_cap": cap,
        "resolved_vehicle_snapshot": snapshot, "sideslip_source": "kinematic",
        "fit_manifest": None, "gate_verdict": None, "fallback_used": False, "fallback_reason": None,
    })

    cached = _pipeline_cache_get(csv_path)
    assert hit_check(cached, cap, snapshot, "kinematic") is True, "same-mode lookup should hit"
    assert hit_check(cached, cap, snapshot, "ekf_auto_pacejka") is False, (
        "switching to a different mode against a kinematic-mode cache entry must MISS, "
        "not silently serve the wrong estimator's results"
    )

    # Switch to ekf_auto_pacejka and store its own entry -- overwrites the
    # single per-csv_path slot, matching production's own _pipeline_cache_
    # put behaviour (one entry per csv_path, not one per mode).
    _pipeline_cache_put(csv_path, {
        "corners": [], "state": None, "cs": None, "stab": None, "fz": None,
        "slip": None, "forces": None, "accuracy_cap": cap,
        "resolved_vehicle_snapshot": snapshot, "sideslip_source": "ekf_auto_pacejka",
        "fit_manifest": {"status": "ok"}, "gate_verdict": {"verdict": "pass"},
        "fallback_used": False, "fallback_reason": None,
    })
    cached2 = _pipeline_cache_get(csv_path)
    assert hit_check(cached2, cap, snapshot, "ekf_auto_pacejka") is True
    assert hit_check(cached2, cap, snapshot, "kinematic") is False, (
        "switching BACK to kinematic against the now-auto-mode cache entry must also MISS -- "
        "this is what forces the recompute the work order requires, not a stale kinematic serve"
    )

    invalidate_all_pipeline_caches()
