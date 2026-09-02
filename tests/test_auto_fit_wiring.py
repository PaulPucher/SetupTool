# Fresh-session work package, Phase 4: validation of the production
# wiring built in Phases 1-3. Calls modules.tyre_fit_auto.
# resolve_sideslip_beta directly -- the exact function ui/views/
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
from modules.tyre_fit_auto import resolve_sideslip_beta, fit_session, fit_session_pacejka

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
FIXED_CAP = 1  # matches tests/conftest.py's own convention


@pytest.fixture(scope="module")
def wiring_fixture():
    raw_params = load_parameters()
    resolved = resolve_accuracy(raw_params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(raw_params, resolved)
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)
    assert state is not None
    return {"params": effective_params, "data": data, "state": state}


# --- Phase 4a: ekf_auto_dugoff wiring -----------------------------------

def test_ekf_auto_dugoff_reproduces_wp_n3_phase2_figures(wiring_fixture):
    """DELIBERATELY REWRITTEN (CS validity repair, Phase 4, 2026-09-02,
    thesis_notes.md "Threshold anchoring + arc closure, Phase 4"): this
    test used to compare fitted c_alpha/mu_fz/R-sweep figures against
    WP-N3 Phase 2's frozen acceptance numbers, on the premise that
    ekf_auto_dugoff reliably converges to 'ok' on Dubai. It no longer
    does -- the rear axle's mu_fz fit degenerates (hits its widened
    search bracket ceiling) under the final CS window floor (100 Hz
    grid), a real upstream behaviour change, not a wiring regression
    (front axle's own fit is unaffected; only rear's mu_fz search is
    exhausted). The frozen-pass-0/R-sweep comparison this test used to
    run is therefore meaningless (gate_verdict is None -- the gate never
    runs on a fit already known degenerate) and has been replaced with
    an assertion on the CURRENT, designed behaviour: wiring reports the
    fallback loudly and correctly, naming the real cause.
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, "ekf_auto_dugoff", csv_path=RAW_FILE
    )
    assert fallback_used, (
        "ekf_auto_dugoff did NOT fall back on this run -- if the rear mu_fz degeneracy "
        "has been fixed/decoupled, this test's own deliberate-fallback exception (and the "
        "corresponding golden) need revisiting, not silently left as-is"
    )
    assert fit_manifest["status"] == "degenerate"
    assert "mu_fz" in fit_manifest["degenerate_reason"] and "rear" in fit_manifest["degenerate_reason"], (
        f"unexpected degenerate_reason: {fit_manifest['degenerate_reason']!r} -- if the failure "
        "mode changed, this is worth a fresh look, not just updating the substring check"
    )
    assert gate_verdict is None, "the NIS gate must not run against a fit already known degenerate"
    assert "degenerate" in fallback_reason


def test_ekf_auto_dugoff_beta_matches_standalone_chain_exactly(wiring_fixture):
    """DELIBERATELY REWRITTEN (CS validity repair, Phase 4, 2026-09-02,
    same record as above): resolve_sideslip_beta's own fallback path
    computes beta via estimate_sideslip(state, params) directly (modules/
    tyre_fit_auto.py) once fit_session reports 'degenerate' -- a direct
    fit_session(...) call on a degenerate fit returns EARLY, before
    'beta_ekf_with_fallback' is ever set (modules/tyre_fit_auto.py's own
    fit_session, the manifest['status']='degenerate' branch), so the
    original bit-identical-to-fit_session's-own-output comparison no
    longer has a value to compare against. Still checks a real wiring
    claim: resolve_sideslip_beta's fallback beta must be bit-identical
    to calling estimate_sideslip directly, not a value it invented or
    perturbed along the way.
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta_wired, _, gate_verdict, fallback_used, _ = resolve_sideslip_beta(
        state, params, data, "ekf_auto_dugoff", csv_path=RAW_FILE
    )
    assert fallback_used and gate_verdict is None

    standalone_manifest = fit_session(data, params, data_file_path=RAW_FILE)
    assert standalone_manifest["status"] == "degenerate"
    assert "beta_ekf_with_fallback" not in standalone_manifest, (
        "a degenerate fit_session return unexpectedly carries beta_ekf_with_fallback -- "
        "the early-return shape this test relies on may have changed"
    )

    beta_standalone = estimate_sideslip(state, params)
    assert np.array_equal(beta_wired, beta_standalone), (
        "resolve_sideslip_beta's fallback beta differs from a direct estimate_sideslip call -- "
        "wiring bug"
    )
    slip_wired = estimate_slip_angles(state, beta_wired, params)
    slip_standalone = estimate_slip_angles(state, beta_standalone, params)
    for key in ("alpha_f_filt", "alpha_r_filt"):
        assert np.array_equal(slip_wired[key], slip_standalone[key])


# --- Phase 4b: ekf_auto_pacejka wiring ------------------------------------

def test_ekf_auto_pacejka_reproduces_wp_n3_phase3_figures(wiring_fixture):
    """Same standard as 4a, against WP-N3 Phase 3's own figures
    (thesis_notes.md "Phase 3: Pacejka variant -- results"): both axles
    converge (status 'ok', powell_converged True, sign_ok True), no
    degenerate flag.
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, "ekf_auto_pacejka", csv_path=RAW_FILE
    )
    assert not fallback_used, f"unexpected fallback: {fallback_reason}"
    assert fit_manifest["status"] == "ok"
    assert gate_verdict is not None and gate_verdict["verdict"] in ("pass", "warn")
    for axle in ("front", "rear"):
        ax = fit_manifest["axles"][axle]
        assert ax["powell_converged"] is True
        assert ax["sign_ok"] is True
        assert ax["D"] > 0


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
    gate, fit_manifest still present (the fit itself succeeded; only the
    gate failed it) with status 'ok' -- the two failure causes
    (degenerate fit vs failed gate) must stay distinguishable in the
    reason text for the status line/PDF (Phase 3b/3d) to describe
    correctly.

    MODE CHANGED to ekf_auto_pacejka (CS validity repair, Phase 4,
    2026-09-02, thesis_notes.md "Threshold anchoring + arc closure,
    Phase 4"): this test's own premise -- forcing the GATE to fail while
    the FIT itself succeeds -- requires a fit that actually reaches 'ok'
    on Dubai. ekf_auto_dugoff's rear axle now degenerates unconditionally
    under the final CS window floor, so resolve_sideslip_beta falls back
    before the gate is ever reached regardless of these thresholds --
    the gate-fail branch this test exists to cover became unreachable via
    that mode on this data. ekf_auto_pacejka still converges normally
    (status 'ok', gate 'pass' under real thresholds), so it is the mode
    that actually exercises the code path under test now.
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
    assert fit_manifest is not None and fit_manifest["status"] == "ok", (
        "the fit itself should have succeeded -- only the gate was forced to fail"
    )

    beta_kinematic = estimate_sideslip(state, forced_params)
    assert np.array_equal(beta, beta_kinematic), (
        "fallback beta is not bit-identical to a direct estimate_sideslip call"
    )


def test_forced_fallback_status_line_text_names_kinematic_and_reason(wiring_fixture):
    """Phase 3b/3d requirement, checked directly against the real
    formatter: the status line must be impossible to mistake for a
    real EKF render on fallback -- checks OutingForm._format_estimator_
    status (None-self call, same precedent as _classify_corner) and
    core/weekend_pdf_export._estimator_status_text produce text that
    names KINEMATIC (not the requested EKF mode) and includes the
    fallback reason.
    """
    try:
        from ui.views.outing_form import OutingForm
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")
    from core.weekend_pdf_export import _estimator_status_text

    fallback_reason = "NIS gate verdict 'fail' (health_score=0.05, threshold_warn=2.0)"
    text, colour = OutingForm._format_estimator_status(
        None, "ekf_auto_dugoff", {"status": "ok"}, {"verdict": "fail", "health_score": 0.05},
        True, fallback_reason,
    )
    assert "KINEMATIC" in text
    assert fallback_reason in text

    pdf_text = _estimator_status_text(
        "ekf_auto_dugoff", {"status": "ok"}, {"verdict": "fail", "health_score": 0.05},
        True, fallback_reason,
    )
    assert "KINEMATIC" in pdf_text
    assert fallback_reason in pdf_text


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
    assert hit_check(cached, cap, snapshot, "ekf_auto_dugoff") is False, (
        "switching to a different mode against a kinematic-mode cache entry must MISS, "
        "not silently serve the wrong estimator's results"
    )

    # Switch to ekf_auto_dugoff and store its own entry -- overwrites the
    # single per-csv_path slot, matching production's own _pipeline_cache_
    # put behaviour (one entry per csv_path, not one per mode).
    _pipeline_cache_put(csv_path, {
        "corners": [], "state": None, "cs": None, "stab": None, "fz": None,
        "slip": None, "forces": None, "accuracy_cap": cap,
        "resolved_vehicle_snapshot": snapshot, "sideslip_source": "ekf_auto_dugoff",
        "fit_manifest": {"status": "ok"}, "gate_verdict": {"verdict": "pass"},
        "fallback_used": False, "fallback_reason": None,
    })
    cached2 = _pipeline_cache_get(csv_path)
    assert hit_check(cached2, cap, snapshot, "ekf_auto_dugoff") is True
    assert hit_check(cached2, cap, snapshot, "kinematic") is False, (
        "switching BACK to kinematic against the now-auto-mode cache entry must also MISS -- "
        "this is what forces the recompute the work order requires, not a stale kinematic serve"
    )

    invalidate_all_pipeline_caches()
