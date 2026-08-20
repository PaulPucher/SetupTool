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
    """Fitted parameters, run through the SAME wiring production uses
    (resolve_sideslip_beta), must reproduce WP-N3 Phase 2's own
    acceptance figures (thesis_notes.md "Phase 2: one-shot per-session
    Dugoff fit + EKF chain -- ..."): c_alpha/mu_fz exact match to pass-0
    (rel tol 1e-6, deterministic optimizer over identical inputs), the
    R-sweep's chosen grid point (r_ay_scale=0.1, r_yaw_scale=4.0),
    status 'ok'. Any drift here is a WIRING bug (resolve_sideslip_beta
    calling fit_session with different arguments/handling than
    intended), not a re-litigation of fit_session's own correctness
    (already covered by diagnostics/inspect_tyre_fit_auto_acceptance.py
    and this file's own reference-comparison test below).
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, "ekf_auto_dugoff", csv_path=RAW_FILE
    )
    assert not fallback_used, f"unexpected fallback: {fallback_reason}"
    assert fit_manifest["status"] == "ok"
    assert gate_verdict is not None and gate_verdict["verdict"] in ("pass", "warn")

    live_pass0 = params["tyre_model_ekf"]["pass_0"]
    for axle in ("front", "rear"):
        got_c = fit_manifest["axles"][axle]["c_alpha_n_per_rad"]
        exp_c = live_pass0[f"c_alpha_{axle}_n_per_rad"]
        assert abs(got_c - exp_c) / abs(exp_c) < 1e-6, f"{axle} c_alpha drift: got={got_c} expect={exp_c}"
        got_m = fit_manifest["axles"][axle]["mu_fz_N"]
        exp_m = live_pass0[f"mu_fz_{axle}_N"]
        assert abs(got_m - exp_m) / abs(exp_m) < 1e-6, f"{axle} mu_fz drift: got={got_m} expect={exp_m}"

    assert fit_manifest["r_sweep"]["chosen"]["r_ay_scale"] == 0.1
    assert fit_manifest["r_sweep"]["chosen"]["r_yaw_scale"] == 4.0
    assert fit_manifest["r_sweep"]["found_in_band"] is True


def test_ekf_auto_dugoff_beta_matches_standalone_chain_exactly(wiring_fixture):
    """resolve_sideslip_beta's output, run through estimate_slip_angles,
    must be BIT-IDENTICAL to calling fit_session directly and doing the
    same -- both call the exact same functions with the exact same
    inputs deterministically. Any difference is a wiring bug (e.g. the
    wrong manifest key used, an extra transformation applied) -- "any
    drift = wiring bug, blocks the package" per the work order.
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    beta_wired, _, gate_verdict, fallback_used, _ = resolve_sideslip_beta(
        state, params, data, "ekf_auto_dugoff", csv_path=RAW_FILE
    )
    assert not fallback_used and gate_verdict["verdict"] != "fail"

    standalone_manifest = fit_session(data, params, data_file_path=RAW_FILE)
    beta_standalone = standalone_manifest["beta_ekf_with_fallback"]

    assert np.array_equal(beta_wired, beta_standalone), (
        "resolve_sideslip_beta's beta differs from a direct fit_session call's "
        "beta_ekf_with_fallback -- wiring bug"
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
    """
    params, data, state = wiring_fixture["params"], wiring_fixture["data"], wiring_fixture["state"]
    forced_params = copy.deepcopy(params)
    forced_params["nis_gate"]["threshold_use_ekf"] = 2.0   # health_score is a fraction in [0,1] -- unreachable
    forced_params["nis_gate"]["threshold_warn"] = 1.5      # same -- everything verdicts 'fail'

    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, forced_params, data, "ekf_auto_dugoff", csv_path=RAW_FILE
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
