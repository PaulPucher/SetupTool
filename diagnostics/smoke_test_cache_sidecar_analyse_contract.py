# Headless smoke test, WP-CACHE Phases 1+2, proving the package's own
# acceptance condition on real data: "restart the app, open the v3
# outing, graphs and trace dialogs fully usable in seconds, ZERO
# pipeline run." [keep-reproduces] per diagnostics/README.md.
#
# Uses a THROWAWAY RaceWeekend+Outing row (created here, deleted in a
# finally block) rather than any real outing -- this test WRITES a real
# analysis_data payload and a real sidecar file, and must never leave
# either behind or overwrite anything belonging to the user's own data.
#
# One real full pipeline run is unavoidable (v3, production sideslip_
# source, cap=1) to produce the sidecar this test then exercises --
# ~12-16 minutes, matching this project's own established cost for a
# real-data smoke test (thesis_notes.md "Cache-miss reproduction
# attempt"). The two "prove a real recompute got triggered" checks below
# terminate their own thread early (decision/control proof only, not a
# second full run's numeric output -- that is covered by the golden
# suite, not this script).

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import datetime
import time

from PyQt6.QtWidgets import QApplication

app = QApplication([])

import models.driver, models.outing, models.raceweekend
from models.base import Session
from models.outing import Outing
from models.raceweekend import RaceWeekend
from ui.views.outing_form import (
    OutingForm, invalidate_all_pipeline_caches, _pipeline_cache_store, _norm_path,
)

from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.csv_parser import parse_csv
from modules import pipeline_sidecar

V3_FILE = "GT3_PRC_MLA-v3.txt"
# None ("Best available") -- not diagnostics/inspect_frame_stage2_parity.
# py's own FIXED_CAP=1 convention (that one exists to pin a comparison
# against golden/reference numbers at a specific accuracy level, a
# different purpose). This test's own cap must match what a genuinely
# fresh OutingForm's accuracy_cap_combo actually defaults to on
# construction -- confirmed by direct read, not assumed: the widget's own
# construction comment (outing_form.py:794) states "accuracy_cap_combo
# has NO cross-restart persistence at all", so it always starts at "Best
# available" regardless of what an earlier session used. Running the
# initial pipeline under a DIFFERENT cap (e.g. 1) would make a clean
# reopen correctly MISS on the accuracy_cap identity field -- a real,
# already-documented gap (thesis_notes.md, WP-DL Phase D feedback round
# ITEM 3), but a different one from what this test exists to prove.
FIXED_CAP = None


def run_real_pipeline_result(raw_file):
    params = load_parameters()
    resolved_accuracy = resolve_accuracy(params, setup_data=None, cap=FIXED_CAP)
    effective_params = apply_resolved_vehicle(params, resolved_accuracy)
    sideslip_source = effective_params["stability_estimation"].get("sideslip_source", "kinematic")

    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    assert state is not None, "prepare_vehicle_state returned None on real v3 data"
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
    # Same policy as OutingForm._get_lap_filter_from_selector (Decisions
    # batch Phase 2b): every is_valid_for_analysis lap, or every lap if
    # none are valid -- lap_filter=None here would mean "all laps
    # unconditionally" to summarise_corners, a DIFFERENT set from what a
    # real Analyse click always computes and stores. Using the wrong one
    # made step C fail: the stored analysis_data's own lap_filter must
    # match what a fresh reopen's selector recomputes, or the DB-cache
    # identity check correctly (and unhelpfully, for THIS test) misses.
    valid_laps = sorted(l["lap_number"] for l in data.get("laps", [])
                         if l.get("is_valid_for_analysis", False))
    all_laps = sorted({l["lap_number"] for l in data.get("laps", [])})
    lap_filter = valid_laps if valid_laps else all_laps
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls, lap_filter=lap_filter)
    return data, lap_filter, {
        "summaries": summaries, "state": state, "cs": cs, "stab": stab, "fz": fz, "ls": ls,
        "slip": slip, "forces": forces, "corners": corners, "cap": FIXED_CAP,
        "resolved_accuracy": resolved_accuracy, "sideslip_source": sideslip_source,
        "fit_manifest": fit_manifest, "gate_verdict": gate_verdict,
        "fallback_used": fallback_used, "fallback_reason": fallback_reason,
    }


session = Session()
weekend = RaceWeekend(track="WP-CACHE smoke test", series="test", car_number=0, year=2026)
session.add(weekend)
session.commit()
outing = Outing(
    date_time=datetime.datetime.now(), name="WP-CACHE smoke test -- safe to delete",
    race_weekend_id=weekend.id, csv_path=V3_FILE,
)
session.add(outing)
session.commit()
outing_id = outing.id
sidecar_path = pipeline_sidecar._sidecar_path(outing_id)
print(f"throwaway outing id={outing_id}, sidecar path={sidecar_path}")

try:
    print("\n--- step A: one real full pipeline run (v3, production defaults) ---")
    t0 = time.perf_counter()
    data, lap_filter, result = run_real_pipeline_result(V3_FILE)
    print(f"pipeline wall time: {time.perf_counter() - t0:.1f}s, lap_filter={lap_filter}")

    form1 = OutingForm(weekend, lambda: None, outing=outing)
    form1.parsed_data = data
    form1.loaded_csv_path = V3_FILE
    # _on_stability_done reads self.stab_thread.lap_filter (it's normally
    # set by _force_recompute right before constructing the real thread) --
    # bypassing the thread here means providing that one attribute directly
    # rather than running a real QThread just to get it. Must be the SAME
    # lap_filter summarise_corners was actually called with above, or the
    # stored analysis_data disagrees with itself.
    import types
    form1.stab_thread = types.SimpleNamespace(lap_filter=lap_filter)
    form1._on_stability_done(result)
    assert os.path.exists(sidecar_path), "sidecar was not written by _on_stability_done"
    print(f"sidecar written: {os.path.getsize(sidecar_path)/1024/1024:.2f} MB")
    session.refresh(outing)
    assert outing.analysis_data, "DB analysis_data was not persisted"

    print("\n--- step B: restart-simulate (clear WP6 in-memory cache) ---")
    invalidate_all_pipeline_caches()
    assert _pipeline_cache_store.get(os.path.normcase(os.path.normpath(V3_FILE))) is None, \
        "WP6 cache did not actually clear -- restart simulation is not real"

    print("\n--- step C: fresh OutingForm reopen, DB-hit + sidecar-hit (Phase 1d) ---")
    form2 = OutingForm(weekend, lambda: None, outing=outing)
    # QWidget.isVisible() reflects effective on-screen visibility, which
    # requires the top-level window to have been shown at least once --
    # setVisible(True) on a child alone is not enough, even in offscreen
    # mode (verified directly: false without this, true with it). The real
    # app always shows its OutingForm, so this only restores that
    # precondition for a widget constructed headlessly here.
    form2.show()
    parsed2 = parse_csv(V3_FILE)  # mirrors CsvLoaderThread.run() synchronously, no Qt thread needed
    form2.parsed_data = parsed2
    form2.loaded_csv_path = V3_FILE

    # Diagnostic dry-run of _try_render_cached_analysis's own 7-field
    # check, side by side, BEFORE the real call -- so a failure here names
    # the exact field instead of just "did not hit" (same technique
    # thesis_notes.md's "Cache-miss reproduction attempt" entry used).
    import json as _json
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION as _SV, load_parameters as _lp, _resolve_grid_rate as _grr
    from modules.accuracy_resolution import resolve_accuracy as _ra
    _cached = _json.loads(outing.analysis_data)
    _cap = form2._get_accuracy_cap_from_selector()
    _resolved = _ra(_lp(), form2._get_setup_data_dict(), _cap)
    _sideslip = _lp()["stability_estimation"].get("sideslip_source", "kinematic")
    _grid, _ = _grr(parsed2["channels"], _lp())
    _lap_filter = form2._get_lap_filter_from_selector()
    print(f"  schema_version: stored={_cached.get('schema_version')!r} current={_SV!r}")
    print(f"  csv_path: stored={_cached.get('csv_path')!r} current={_norm_path(V3_FILE)!r}")
    print(f"  accuracy_cap: stored={_cached.get('accuracy_cap')!r} current={_cap!r}")
    print(f"  resolved_vehicle_snapshot equal: "
          f"{_resolved['values'] == _cached.get('resolved_vehicle_snapshot')}")
    print(f"  sideslip_source: stored={_cached.get('sideslip_source')!r} current={_sideslip!r}")
    print(f"  grid_rate_hz: stored={_cached.get('grid_rate_hz')!r} current={_grid!r}")
    print(f"  lap_filter: stored={sorted(_cached.get('lap_filter') or [])!r} "
          f"current={sorted(_lap_filter or [])!r}")
    print(f"  summaries present: {bool(_cached.get('summaries'))}")

    t_open0 = time.perf_counter()
    hit = form2._try_render_cached_analysis()
    t_open1 = time.perf_counter()
    print(f"reopen (DB+sidecar) wall time: {t_open1 - t_open0:.3f}s")
    assert hit is True, "DB cache did not hit on a clean reopen -- cannot test the sidecar at all"
    assert form2._sidecar_hit is True, f"sidecar did not hit: {form2._sidecar_miss_reason}"
    assert "state" in form2.stability_result, "sidecar hit but full pipeline fields absent"
    for key in ("cs", "stab", "corners", "slip", "forces", "ls"):
        assert key in form2.stability_result, f"sidecar payload missing '{key}'"
    assert len(form2.stability_result["state"]["time"]) > 0, "state arrays are empty -- still a hull"
    print(f"OK -- ZERO pipeline run, full pipeline result available in "
          f"{t_open1 - t_open0:.3f}s (graphs/trace dialogs are real, not an empty hull)")

    print("\n--- step D: Analyse click takes the fast path (2a), zero recompute ---")
    assert not hasattr(form2, "stab_thread"), "a thread already exists before the fast-path click"
    t_fast0 = time.perf_counter()
    form2._run_stability_analysis()
    t_fast1 = time.perf_counter()
    assert not hasattr(form2, "stab_thread"), "fast path spawned a StabilityAnalysisThread -- not fast"
    assert form2.stability_status_label.text() == "results current (cached) - recompute", \
        f"unexpected status text: {form2.stability_status_label.text()!r}"
    assert form2.btn_recompute_stale_cache.isVisible(), "recompute control not shown on fast path"
    print(f"OK -- fast path took {t_fast1 - t_fast0:.4f}s, status text and recompute control correct")

    print("\n--- step E: identity mismatch (2b) names the changed field, forces a real run ---")
    form2.accuracy_cap_combo.setCurrentText("Level 2")
    form2._run_stability_analysis()
    assert hasattr(form2, "stab_thread"), "identity mismatch did not trigger a real run"
    status_text = form2.stability_status_label.text()
    # Changing the cap also changes resolve_accuracy's own resolved
    # snapshot (the cap clips/selects different accuracy levels) -- the
    # correct, honest reason names BOTH ("accuracy cap and vehicle setup
    # changed"), not just the field the test directly touched. Substring
    # check, not an exact match, for that reason.
    assert status_text.startswith("recomputing: "), f"unexpected status text: {status_text!r}"
    assert "accuracy cap" in status_text, f"unexpected status text: {status_text!r}"
    assert form2.stab_thread.isRunning() or form2.stab_thread.isFinished()
    form2.stab_thread.terminate()
    form2.stab_thread.wait(5000)
    print(f"OK -- status text correctly named the change: {status_text!r}")
    form2.accuracy_cap_combo.setCurrentText("Best available")

    print("\n--- step F: explicit recompute control forces a run unconditionally (2a) ---")
    # form2.stability_result/_last_render_kwargs are UNCHANGED from step C
    # (step E's thread was terminated before _on_stability_done could ever
    # run) -- _on_recompute_clicked ignores both anyway (forced=True
    # bypasses the identity check entirely), the exact property this step
    # is testing.
    del form2.stab_thread
    form2._on_recompute_clicked()
    assert hasattr(form2, "stab_thread"), "recompute control did not start a thread"
    status_text = form2.stability_status_label.text()
    assert "recomputing: forced recompute" in status_text, f"unexpected status text: {status_text!r}"
    form2.stab_thread.terminate()
    form2.stab_thread.wait(5000)
    print(f"OK -- status text: {status_text!r}")

    print("\n=== ACCEPTANCE CONDITION MET: restart -> open v3 outing -> "
          "graphs/trace dialogs usable, zero pipeline run ===")
finally:
    print("\n--- cleanup: removing throwaway outing/weekend and sidecar file ---")
    if os.path.exists(sidecar_path):
        os.remove(sidecar_path)
    session2 = Session()
    session2.query(Outing).filter(Outing.id == outing_id).delete()
    session2.query(RaceWeekend).filter(RaceWeekend.id == weekend.id).delete()
    session2.commit()
    session2.close()
    session.close()
    print("cleanup complete")
