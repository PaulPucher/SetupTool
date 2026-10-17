# Limit events (modules/limit_events.py) on the real Dubai sample, through the
# shared pipeline fixture. The guard test is the contract the author ruled:
# events are display + record only, so verdict inputs, corner summaries,
# verdicts and recommendations must be byte-identical with events on or off.

import copy
import json
import os

import numpy as np
import pytest

from modules.limit_events import (
    EVENT_FLOOR_MAX_DEG, aggregate_by_corner_phase, curve_floor, detect_events,
)
from modules.stability_analysis import summarise_corners


def _events(pipeline_result, state, parsed_data, params):
    return detect_events(pipeline_result["cs"], state, pipeline_result["corners"], parsed_data["laps"],
                         pipeline_result["fit_manifest"], params, slip=pipeline_result["slip"])


def _verdict_snapshot(pipeline_result, state):
    # summaries + live classification + recommendations, as JSON text
    from ui.views.outing_form import OutingForm
    from modules.recommendation import generate_recommendations, load_recommendations_config
    summaries = summarise_corners(pipeline_result["corners"], pipeline_result["cs"], pipeline_result["stab"],
                                  state, fz=pipeline_result["fz"], lap_filter=None)
    verdicts = [OutingForm._classify_corner(None, s) for s in summaries]
    recs = generate_recommendations(summaries, lambda s: OutingForm._classify_corner(None, s),
                                    feedback_data={}, setup_data=None, config=load_recommendations_config())
    return json.dumps({"summaries": summaries, "verdicts": verdicts, "recs": recs}, default=str, sort_keys=True)


def test_floor_definition_on_the_session_fit(pipeline_result):
    # the floor is the curve's own CS analogue: 1 at the origin, and a wider
    # slip range can only lower it
    for axle in ("front", "rear"):
        ax = pipeline_result["fit_manifest"]["axles"][axle]
        f0, _ = curve_floor(ax, 0.0)
        assert f0 == pytest.approx(1.0, abs=1e-9)
        f_vis, _ = curve_floor(ax, ax["visited_alpha_p99_deg"])
        f30, at30 = curve_floor(ax, EVENT_FLOOR_MAX_DEG)
        assert f30 <= f_vis
        assert 0.0 <= at30 <= EVENT_FLOOR_MAX_DEG


def test_events_are_display_only(pipeline_result, state, parsed_data, effective_params):
    """GUARD: verdicts, summaries and recommendations byte-identical with the
    event layer on and off; detect_events mutates none of its inputs."""
    try:
        import ui.views.outing_form  # noqa: F401
    except Exception as e:
        pytest.skip(f"ui.views.outing_form not importable here ({e})")
    cs_before = {k: np.array(v, copy=True) for k, v in pipeline_result["cs"].items() if isinstance(v, np.ndarray)}
    state_before = {k: np.array(v, copy=True) for k, v in state.items() if isinstance(v, np.ndarray)}
    corners_before = json.dumps(pipeline_result["corners"], default=str, sort_keys=True)
    snap_off = _verdict_snapshot(pipeline_result, state)

    params_on = copy.deepcopy(effective_params)
    params_on["limit_events"]["enabled"] = True
    events = _events(pipeline_result, state, parsed_data, params_on)
    assert events["status"] == "computed"

    for k, v in cs_before.items():
        assert np.array_equal(v, pipeline_result["cs"][k], equal_nan=True), f"cs[{k}] mutated"
    for k, v in state_before.items():
        assert np.array_equal(v, state[k], equal_nan=True), f"state[{k}] mutated"
    assert json.dumps(pipeline_result["corners"], default=str, sort_keys=True) == corners_before
    assert _verdict_snapshot(pipeline_result, state) == snap_off

    params_off = copy.deepcopy(effective_params)
    params_off["limit_events"]["enabled"] = False
    assert _events(pipeline_result, state, parsed_data, params_off) == {
        "status": "disabled in config", "floors": None, "events": [], "n_instances_by_corner": {}}


def test_no_verdict_module_reads_events():
    # structural guard: only the UI display paths import the event module
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for rel in ("modules/stability_analysis.py", "modules/recommendation.py", "modules/decision_frame.py",
                "modules/tyre_fit_auto.py"):
        src = open(os.path.join(root, rel), encoding="utf-8").read()
        # imports, not mentions (the schema history comment names the module)
        assert "import limit_events" not in src and "modules.limit_events" not in src, \
            f"{rel} imports the event layer"
    form = open(os.path.join(root, "ui/views/outing_form.py"), encoding="utf-8").read()
    body = form[form.index("def _classify_corner"):form.index("def _build_header")]
    assert "event" not in body.lower(), "the classifier must not read events"


def test_event_record_properties(pipeline_result, state, parsed_data, effective_params):
    ev = _events(pipeline_result, state, parsed_data, effective_params)
    json.dumps(ev)  # JSON-safe: it goes into the persisted payload
    valid = {l["lap_number"] for l in parsed_data["laps"] if l.get("is_valid_for_analysis")}
    t = state["time"]
    for e in ev["events"]:
        axle = "f" if e["axle"] == "front" else "r"
        assert e["cs_min"] < e["floor"] == ev["floors"][e["axle"]]
        assert e["lap"] in valid
        assert e["t_start"] <= e["t_end"] and e["duration_s"] > 0
        assert e["s_start_m"] <= e["s_min_m"] <= e["s_end_m"]
        assert e["recurrence"] == ("recurring" if len(e["recurring_laps"]) >= 2 else "single")
        assert e["lap"] in e["recurring_laps"]
        i0, i1 = np.searchsorted(t, [e["t_start"], e["t_end"]])
        # the minimum is the deepest CS inside the event span
        assert np.nanmin(pipeline_result["cs"][f"CS_ratio_{axle}"][i0:i1 + 1]) == pytest.approx(e["cs_min"])
        assert e["window_start_t"] <= e["t_start"]
    rows = aggregate_by_corner_phase(ev["events"])
    assert sum(r["n_events"] for r in rows) == len(ev["events"])


def test_minimum_duration_is_one_cs_window(effective_params):
    # author ruling 2026-10-06 (round 3): min duration = one CS regression
    # window, the estimator's own resolution
    cfg = effective_params["limit_events"]
    assert cfg["min_duration_s"] == effective_params["stability_estimation"]["cs_min_window_s"] == 0.1
    assert cfg["bridge_gap_s"] == 0.5
    assert cfg["recurrence_radius_m"] == 10.0


def test_events_respect_minimum_duration(pipeline_result, state, parsed_data, effective_params):
    ev = _events(pipeline_result, state, parsed_data, effective_params)
    sr = state["sample_rate_hz"]
    assert all(e["duration_s"] >= effective_params["limit_events"]["min_duration_s"] - 0.5 / sr
               for e in ev["events"])


def test_no_event_line_from_a_failed_nis_gate(pipeline_result, state, parsed_data, effective_params):
    """R14: a session whose NIS gate failed ships kinematic beta, so its
    fitted curve draws no event line; a passing or absent gate changes nothing."""
    base = _events(pipeline_result, state, parsed_data, effective_params)
    args = (pipeline_result["cs"], state, pipeline_result["corners"], parsed_data["laps"],
            pipeline_result["fit_manifest"], effective_params)
    failed = detect_events(*args, slip=pipeline_result["slip"], gate_verdict={"verdict": "fail"})
    assert failed["events"] == [] and failed["floors"] is None
    assert "NIS gate failed" in failed["status"]
    passed = detect_events(*args, slip=pipeline_result["slip"], gate_verdict={"verdict": "pass"})
    assert json.dumps(passed, sort_keys=True, default=str) == json.dumps(base, sort_keys=True, default=str)
