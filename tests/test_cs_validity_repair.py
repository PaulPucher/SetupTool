# CS validity repair, part A (2026-09-02) -- targeted tests for the new
# mechanics: adaptive-widening's cap-driven NaN path, the per-phase
# min-valid-samples gate, the apex_region distance-based statistic, and
# the wiring that lets apex_3-keyed recommendation rules read it instead
# of apex_3's own structurally undersized slice. REGRESSION-STYLE unit
# tests on synthetic, hand-constructed inputs -- not a re-derivation of
# the real Dubai numbers (that lives in thesis_notes.md/PLAN.md, from
# diagnostics/inspect_cs_window_floor_derivation.py (script at tag pre-prune-2026-10-22) and friends).

import numpy as np
import pytest

from modules.stability_analysis import (
    estimate_cornering_stiffness, reconstruct_cs_window_start, resolve_cs_min_window_samples,
    summarise_corners,
)


# --- rate-derived min_window (Phase 1 REVISION, sample-rate correction) -----

def test_resolve_cs_min_window_samples_scales_with_log_rate():
    # cs_min_window_s is a PHYSICAL duration -- a faster log must derive
    # MORE samples for the SAME duration, never a fixed count (the exact
    # bug this correction fixes: the chair's own 10-sample default was
    # always a 100 Hz-calibrated 0.1 s window, silently treated as if
    # rate-independent).
    params = {"stability_estimation": {"cs_min_window_s": 0.5, "cs_min_window_samples_floor": 1}}
    assert resolve_cs_min_window_samples(params, sample_rate_hz=10.0) == 5
    assert resolve_cs_min_window_samples(params, sample_rate_hz=100.0) == 50


def test_resolve_cs_min_window_samples_floor_binds_on_a_slow_log():
    params = {"stability_estimation": {"cs_min_window_s": 0.5, "cs_min_window_samples_floor": 10}}
    # 0.5s @ 5 Hz would derive only 2.5 -> 2 or 3 samples, below the floor
    assert resolve_cs_min_window_samples(params, sample_rate_hz=5.0) == 10


def _base_se(**overrides):
    # cs_min_window_s=0.5 @ sample_rate_hz=10 (see _cs_inputs) resolves to
    # min_window=5 samples -- same numeric floor the pre-rate-correction
    # tests used, now expressed physically. cs_max_window_m=20.0 with
    # _cs_inputs's own 1-metre-per-sample synthetic s_m resolves to the
    # same 20-sample cap the pre-rate-correction tests used.
    se = {
        "cs_min_slip_angle_span_rad": 0.1,
        "cs_linear_slip_threshold_rad": 0.05,
        "cs_min_window_s": 0.5,
        "cs_min_window_samples_floor": 1,
        "cs_max_window_m": 20.0,
    }
    se.update(overrides)
    # R26/R27 band on the CS reference (R5, 2026-10-17): the estimator reads
    # the D4 tyre-fit stiffness band (implied linear axle stiffness, magnitudes
    # after Bechtloff Fig. 7.2); literal config values, this dict is isolated
    return {"stability_estimation": se,
            "tyre_fit_auto": {"stiffness_min_npr": 30000, "stiffness_max_npr": 600000}}


def _cs_inputs(alpha, Fy, moving=None, s_m=None, sample_rate_hz=10.0):
    n = len(alpha)
    slip = {"alpha_f_filt": alpha, "alpha_r_filt": alpha}
    forces = {"Fy_f_filt": Fy, "Fy_r_filt": Fy}
    state = {
        "moving_mask": np.ones(n, dtype=bool) if moving is None else moving,
        "sample_rate_hz": sample_rate_hz,
        # 1 metre/sample by construction -- lets cs_max_window_m be tested
        # with the same clean numbers as a sample-count cap would use.
        "s_m": np.arange(n, dtype=float) if s_m is None else s_m,
    }
    return slip, forces, state


# --- adaptive widening + cap (Phase 2) ---------------------------------------

def test_flat_alpha_never_meets_span_floor_and_reports_nan():
    # Span floor (0.1 rad) can never be met -- alpha never moves. Every
    # sample from cs_min_window_samples onward must report NaN, not a
    # degenerate zero-variance fit.
    n = 60
    alpha = np.zeros(n)
    Fy = np.zeros(n)
    slip, forces, state = _cs_inputs(alpha, Fy)
    params = _base_se()
    out = estimate_cornering_stiffness(slip, forces, state, params, [])
    assert np.all(np.isnan(out["CS_ratio_f"]))
    assert np.all(np.isnan(out["C_alpha_f"]))


def test_widening_succeeds_within_cap_for_a_genuine_ramp():
    # A steady ramp reaches the 0.1 rad span comfortably within the 20 m
    # cap (min_window=5, 1 m/sample synthetic s_m -- see _cs_inputs) --
    # the window should widen just enough to clear the floor and produce
    # a finite, correctly-signed slope.
    n = 60
    alpha = np.linspace(0.0, 0.6, n)  # step ~0.0102 rad/sample -- a 20-sample-equivalent window clears 0.1 rad comfortably
    Fy = -200000.0 * alpha  # clean linear relation, known slope
    slip, forces, state = _cs_inputs(alpha, Fy)
    params = _base_se()
    out = estimate_cornering_stiffness(slip, forces, state, params, [])
    i = 40
    assert np.isfinite(out["C_alpha_f"][i])
    assert out["C_alpha_f"][i] == pytest.approx(-200000.0, rel=1e-6)
    s_m = state["s_m"]
    start = reconstruct_cs_window_start(alpha, i, 5, 0.1, s_m=s_m, max_window_m=20.0)
    assert 5 <= i - start <= 20
    assert (alpha[i - 1] - alpha[start]) >= 0.1 - 1e-9


def test_cap_truncates_widening_and_reports_nan_when_span_still_short():
    # alpha creeps far too slowly to reach the 0.1 rad span within a 20 m
    # cap (it would need the whole 60-sample/60 m history) -- the capped
    # window must stop at max_window_m and, since it still falls short of
    # the span floor, report NaN rather than accepting an under-qualified
    # fit.
    n = 60
    alpha = np.linspace(0.0, 0.05, n)  # total span over the WHOLE array is only 0.05 rad
    Fy = -200000.0 * alpha
    slip, forces, state = _cs_inputs(alpha, Fy)
    params = _base_se()
    out = estimate_cornering_stiffness(slip, forces, state, params, [])
    assert np.all(np.isnan(out["CS_ratio_f"]))
    i = 59
    s_m = state["s_m"]
    start = reconstruct_cs_window_start(alpha, i, 5, 0.1, s_m=s_m, max_window_m=20.0)
    assert i - start < 40  # capped well short of walking back to the session start (would be 59)
    assert (s_m[i - 1] - s_m[start]) >= 20.0  # cap enforced: at least the target distance was covered
    assert (alpha[i - 1] - alpha[start]) < 0.1  # and still short of the floor there


def test_reconstruct_cs_window_start_omitting_cap_is_unbounded_as_before():
    # Backward-compatibility contract for callers that don't pass
    # s_m/max_window_m (pre-Phase-2 diagnostics scripts) -- unchanged
    # behaviour.
    n = 60
    alpha = np.linspace(0.0, 0.05, n)
    i = 59
    start_uncapped = reconstruct_cs_window_start(alpha, i, 5, 0.1)
    assert start_uncapped == 0  # walks all the way back, never reaching 0.1 rad


def test_reconstruct_cs_window_start_true_extremum_at_oldest_end():
    """WP-PERF (thesis_notes.md): reconstruct_cs_window_start now
    maintains a running max/min incrementally instead of re-scanning the
    whole growing slice every widening step (compute_cs_for_axle's own
    inline loop mirrors this exactly, kept in sync). The bug an
    incremental fold gets wrong first is discarding or never folding in
    the value at the OLDEST (just-added, final) end of the window -- here
    the span floor is only clearable once the very last widening step
    (start=0) admits a single large value; everything else is flat."""
    n = 60
    alpha = np.full(n, 0.001)
    alpha[0] = 0.2  # true max, reachable ONLY at the final widening step
    i = 60
    start = reconstruct_cs_window_start(alpha, i, 10, 0.05)
    assert start == 0, "must have widened all the way to the oldest sample to admit the spike"
    achieved_span = alpha[start:i].max() - alpha[start:i].min()
    assert achieved_span >= 0.05 - 1e-9


def test_reconstruct_cs_window_start_nan_in_window_forces_full_widening():
    """WP-PERF: the running-extremum update uses np.maximum/np.minimum
    (not np.max/np.min's array reduction) because they propagate NaN the
    same order-independent way np.max/np.min's own reduce does --
    verified, not assumed (thesis_notes.md). A NaN entering the window
    must poison the running max/min for the rest of THIS call, exactly as
    it would poison a fresh np.max/np.min recompute over any slice that
    includes it: span becomes NaN, `NaN >= min_span` is always False, so
    widening can never clear the span condition again, even though a
    legitimate large span exists further back, past the NaN."""
    n = 50
    alpha = np.full(n, 0.001)
    alpha[20] = np.nan  # sits inside the window once widening reaches it
    alpha[5] = 100.0  # would trivially clear the span floor IF reachable
    i = 40
    start = reconstruct_cs_window_start(alpha, i, 10, 0.05)
    assert start == 0, "NaN must force widening to the start=0 boundary, never clearing span"


def test_distance_cap_disabled_when_s_m_is_none():
    # No distance channel available -- the metre-based cap must fall back
    # to no cap at all (min_window/min_span floors alone), not crash.
    n = 60
    alpha = np.linspace(0.0, 0.05, n)
    Fy = -200000.0 * alpha
    slip, forces, state = _cs_inputs(alpha, Fy, s_m=None)
    state["s_m"] = None  # explicit override -- _cs_inputs itself always fills a synthetic one
    params = _base_se()
    out = estimate_cornering_stiffness(slip, forces, state, params, [])
    # Same alpha profile as test_cap_truncates_widening... but WITHOUT a
    # cap this time -- the window can walk all the way back and finds the
    # full 0.05 rad span still short of the 0.1 rad floor -> still NaN,
    # but for a different reason (ran out of history, not capped).
    assert np.all(np.isnan(out["CS_ratio_f"]))


# --- per-phase min-valid-samples gate (Phase 2) ------------------------------

def _make_corner(lap_number=1, cid=1, speed_class="medium", apex_time=5.0,
                  apex_lap_distance_m=50.0, segments=None):
    if segments is None:
        segments = {
            "entry_1_brake": (0.0, 1.0),
            "entry_2_turnin": (1.0, 4.0),
            "apex_3": (5.0, 4.0),  # end < start -- apex is a point, per production convention
            "exit_4": (6.0, 8.0),
            "exit_5": (8.0, 10.0),
        }
    return {
        "lap_number": lap_number, "corner_number": cid, "speed_class": speed_class,
        "apex_time": apex_time, "apex_speed": 30.0, "apex_lateral_g": 1.0,
        "method": "test", "warnings": [], "stable_corner_id": cid,
        "bracket_start_m": segments["entry_1_brake"][0], "bracket_end_m": segments["exit_5"][1],
        "segments": segments, "apex_lap_distance_m": apex_lap_distance_m,
    }


def _base_state(n, t, s_m=None, moving=None):
    return {
        "time": t,
        "s_m": s_m,
        "moving_mask": np.ones(n, dtype=bool) if moving is None else moving,
    }


def test_phase_below_min_valid_samples_reports_nan_but_stability_is_unaffected():
    n = 20
    t = np.linspace(0.0, 10.0, n)
    cs_f = np.full(n, np.nan)
    cs_f[2:4] = 0.5  # 2 finite samples fall inside entry_2_turnin's own index range -- below the min of 3 used here
    cs_r = np.full(n, np.nan)
    stab = np.full(n, 100.0)
    corner = _make_corner()
    state = _base_state(n, t)
    out = summarise_corners(
        [corner], {"CS_ratio_f": cs_f, "CS_ratio_r": cs_r},
        {"stability_observed_Nm_per_deg": stab, "stability_valid": np.ones(n, dtype=bool)},
        state, apex_half_window_samples=1, cs_phase_min_valid_samples=3,
        cs_apex_region_half_length_m=5.0,
    )
    phase = out[0]["phases"]["entry_2_turnin"]
    assert phase["n_samples"] > 0
    assert phase["cs_ratio_f"]["median"] != phase["cs_ratio_f"]["median"]  # NaN
    assert phase["stability_observed_Nm_per_deg"]["median"] == 100.0  # gate does not touch stability


def test_phase_at_or_above_min_valid_samples_reports_real_median():
    n = 20
    t = np.linspace(0.0, 10.0, n)
    cs_f = np.full(n, np.nan)
    cs_f[2:5] = 0.5  # 3 finite samples, all falling inside entry_2_turnin's own index range -- meets the min of 3
    cs_r = np.full(n, np.nan)
    stab = np.full(n, 100.0)
    corner = _make_corner()
    state = _base_state(n, t)
    out = summarise_corners(
        [corner], {"CS_ratio_f": cs_f, "CS_ratio_r": cs_r},
        {"stability_observed_Nm_per_deg": stab, "stability_valid": np.ones(n, dtype=bool)},
        state, apex_half_window_samples=1, cs_phase_min_valid_samples=3,
        cs_apex_region_half_length_m=5.0,
    )
    phase = out[0]["phases"]["entry_2_turnin"]
    assert phase["cs_ratio_f"]["median"] == 0.5


# --- apex_region (Phase 3) ----------------------------------------------------

def test_apex_region_empty_when_s_m_missing():
    n = 20
    t = np.linspace(0.0, 10.0, n)
    cs_f = np.full(n, 0.5)
    cs_r = np.full(n, 0.5)
    stab = np.full(n, 100.0)
    corner = _make_corner()
    state = _base_state(n, t, s_m=None)
    out = summarise_corners(
        [corner], {"CS_ratio_f": cs_f, "CS_ratio_r": cs_r},
        {"stability_observed_Nm_per_deg": stab, "stability_valid": np.ones(n, dtype=bool)},
        state, apex_half_window_samples=1, cs_phase_min_valid_samples=1,
        cs_apex_region_half_length_m=5.0,
    )
    assert out[0]["apex_region"]["n_samples"] == 0
    assert out[0]["apex_region"]["cs_ratio_f"]["median"] != out[0]["apex_region"]["cs_ratio_f"]["median"]


def test_apex_region_selects_only_samples_within_distance_band_and_corner_time_bounds():
    # 100 samples, 1 m apart (s_m = 0..99), corner spans t in [1,10]s (s_m
    # 10..99 given the 1:1 mapping below), apex at s=50. Half-length=5m ->
    # samples with s_m in [45,55] AND inside the corner's own time window
    # should be selected; a sample at the SAME distance but OUTSIDE the
    # corner's time window (simulating a different lap at the same track
    # distance) must not leak in.
    n = 100
    t = np.linspace(0.0, 99.0, n)  # 1 s apart
    s_m = np.arange(n, dtype=float)  # 1 m apart, same index as t here
    cs_f = np.arange(n, dtype=float)  # distinct value per sample, easy to check selection
    cs_r = np.arange(n, dtype=float)
    stab = np.full(n, 100.0)
    segments = {
        "entry_1_brake": (10.0, 30.0), "entry_2_turnin": (30.0, 45.0),
        "apex_3": (50.0, 45.0), "exit_4": (50.0, 70.0), "exit_5": (70.0, 90.0),
    }
    corner = _make_corner(apex_lap_distance_m=50.0, segments=segments)
    state = _base_state(n, t, s_m=s_m)
    out = summarise_corners(
        [corner], {"CS_ratio_f": cs_f, "CS_ratio_r": cs_r},
        {"stability_observed_Nm_per_deg": stab, "stability_valid": np.ones(n, dtype=bool)},
        state, apex_half_window_samples=1, cs_phase_min_valid_samples=1,
        cs_apex_region_half_length_m=5.0,
    )
    # Expected selected indices: s_m in [45,55] AND t in [10,90] (corner's
    # own total span) -- here s_m == index == t, so indices 45..55 inclusive.
    expected = np.arange(45, 56)
    ar = out[0]["apex_region"]
    assert ar["n_samples"] == len(expected)
    assert ar["cs_ratio_f"]["median"] == pytest.approx(float(np.median(cs_f[expected])))


# --- classify_fn wiring (Phase 3) --------------------------------------------

def _strong_value(axle, below):
    # a CS value `below` under the live STRONG threshold of the axle, so the
    # tests follow re-derived thresholds instead of pinning retired ones
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    return cl["STRONG_CSF" if axle == "f" else "STRONG_CSR"]["value"] - below


def test_classify_corner_reads_apex_region_for_apex_3_cs():
    from ui.views.outing_form import OutingForm

    healthy = {"median": 1.0}
    stab_ok = {"median": 500.0}
    summary_with_apex_region = {
        "phases": {
            "entry_1_brake": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "entry_2_turnin": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            # apex_3's OWN slice reads healthy -- only apex_region signals trouble.
            "apex_3": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "exit_4": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "exit_5": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
        },
        "apex_region": {"cs_ratio_f": {"median": _strong_value("f", 0.15)}, "cs_ratio_r": healthy},
    }
    severity, short, _long, _colour = OutingForm._classify_corner(None, summary_with_apex_region)
    # ruling D2 (2026-10-22): front limiting inside the strong tier is strong
    assert severity == "strong"
    assert "understeer" in short
    assert "apex" in short


def test_classify_corner_without_apex_region_falls_back_to_apex_3_slice():
    # Backward compatibility: a pre-schema-8 summary with no "apex_region"
    # key must classify exactly as before (apex_3's own slice used).
    healthy = {"median": 1.0}
    stab_ok = {"median": 500.0}
    summary_no_apex_region = {
        "phases": {
            "entry_1_brake": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "entry_2_turnin": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "apex_3": {"cs_ratio_f": {"median": _strong_value("f", 0.15)}, "cs_ratio_r": healthy,
                       "stability_observed_Nm_per_deg": stab_ok},
            "exit_4": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
            "exit_5": {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok},
        },
    }
    from ui.views.outing_form import OutingForm
    severity, short, _long, _colour = OutingForm._classify_corner(None, summary_no_apex_region)
    assert severity == "strong"  # D2: front in the strong tier
    assert "understeer" in short


# --- Metrology Phase 2: verdict-stability [MARGINAL] annotation -------------

def _healthy_phases():
    healthy = {"median": 1.0}
    stab_ok = {"median": 500.0}
    return {
        phase: {"cs_ratio_f": healthy, "cs_ratio_r": healthy, "stability_observed_Nm_per_deg": stab_ok}
        for phase in ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
    }


def _cs_margin():
    from modules.stability_analysis import load_parameters
    return load_parameters()["classification"]["verdict_stability_margin"]["cs_margin"]


def test_classify_corner_marginal_marker_fires_within_anchored_margin():
    # half the anchored margin below STRONG_CSF (config classification.
    # verdict_stability_margin.cs_margin, re-derived 2026-10-20) -- a real
    # understeer verdict this close to the threshold must be tagged.
    from ui.views.outing_form import OutingForm
    summary = {"phases": _healthy_phases()}
    summary["phases"]["apex_3"]["cs_ratio_f"] = {"median": _strong_value("f", _cs_margin() / 2)}
    severity, short, long_, _colour = OutingForm._classify_corner(None, summary)
    assert severity == "strong"  # D2: front in the strong tier
    assert "understeer" in short
    assert "[MARGINAL]" in short
    # 2026-10-06 (author app-check ruling): the card text carries no
    # qualifiers or meta-sentences; MARGINAL lives in the machine string (the
    # decision frame caps confidence on it) and in the tooltip
    assert "[MARGINAL]" not in long_ and "flip" not in long_
    assert long_.startswith("Understeer, ") and "front CS" in long_
    assert "~1 % measurement" in OutingForm._verdict_tooltip(short)
    assert OutingForm._verdict_cell_text(short) == short.replace(" [MARGINAL]", "")


def test_classify_corner_marginal_marker_absent_far_from_threshold():
    # 0.15 below STRONG_CSF (and further from MODERATE_CSF) -- outside the
    # anchored margin -- a confident verdict, not a marginal one.
    assert _cs_margin() < 0.15
    from ui.views.outing_form import OutingForm
    summary = {"phases": _healthy_phases()}
    summary["phases"]["apex_3"]["cs_ratio_f"] = {"median": _strong_value("f", 0.15)}
    severity, short, _long, _colour = OutingForm._classify_corner(None, summary)
    assert severity == "strong"  # D2: front in the strong tier
    assert "understeer" in short
    assert "[MARGINAL]" not in short


def test_classify_corner_marginal_marker_absent_when_no_verdict():
    # A clean corner has no primary axle to attach a stability marker to.
    from ui.views.outing_form import OutingForm
    summary = {"phases": _healthy_phases()}
    severity, short, _long, _colour = OutingForm._classify_corner(None, summary)
    assert severity == "normal"
    assert short == "ok"
    assert "[MARGINAL]" not in short


def test_classify_corner_marginal_marker_fires_on_rear_axle_too():
    # D2: the rear class is decided by the limiting boundary alone (STRONG_CSR
    # no longer grades it); half the anchored margin past -cs_boundary_margin_rear
    from modules.stability_analysis import load_parameters
    from ui.views.outing_form import OutingForm
    boundary = load_parameters()["classification"]["cs_boundary_margin_rear"]["value"]
    summary = {"phases": _healthy_phases()}
    summary["phases"]["apex_3"]["cs_ratio_r"] = {"median": -boundary - _cs_margin() / 2}
    severity, short, _long, _colour = OutingForm._classify_corner(None, summary)
    assert severity == "strong"
    assert "oversteer" in short
    assert "[MARGINAL]" in short


# --- end-to-end: apex symptoms from apex_region alone (setup engine) --------
# Successor of the six apex-keyed matrix-rule checks (old engine retired
# 2026-10-22): apex_3's own slice is healthy, only apex_region carries the
# fault on every lap, and the setup engine must still count the apex symptom
# -- the Phase 3 wiring (summarise_corners -> aggregate_by_corner ->
# verdict_cs_stat) feeds the new engine as it fed the old rules.

@pytest.mark.parametrize("speed_class,axle_key,symptom", [
    ("low", "cs_ratio_f", "US"), ("medium", "cs_ratio_f", "US"), ("high", "cs_ratio_f", "US"),
    ("low", "cs_ratio_r", "OS"), ("medium", "cs_ratio_r", "OS"), ("high", "cs_ratio_r", "OS"),
])
def test_apex_symptom_counts_via_apex_region_alone(speed_class, axle_key, symptom):
    from modules.setup_engine import data_layer

    healthy = {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 50}
    stab_ok = {"median": 500.0, "p25": 400.0, "p75": 600.0, "n": 50}
    other_axle_key = "cs_ratio_r" if axle_key == "cs_ratio_f" else "cs_ratio_f"
    bad_median = _strong_value(axle_key[-1], 0.15)

    def make_phase():
        return {"cs_ratio_f": dict(healthy), "cs_ratio_r": dict(healthy),
                "stability_observed_Nm_per_deg": dict(stab_ok), "n_samples": 50,
                "kerb_fraction": 0.0, "valid_fraction_stab": 1.0}

    summaries = []
    for lap in range(1, 5):
        phases = {p: make_phase() for p in
                  ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")}
        apex_region = {axle_key: {"median": bad_median}, other_axle_key: dict(healthy)}
        summaries.append({
            "lap_number": lap, "corner_number": 1, "speed_class": speed_class,
            "apex_time": 5.0, "apex_speed": 25.0, "apex_lateral_g": 1.0,
            "method": "test", "warnings": [], "apex_position_x_m": None, "apex_position_y_m": None,
            "stable_corner_id": 1, "bracket_start_m": 0.0, "bracket_end_m": 100.0,
            "phases": phases, "apex_region": apex_region,
        })

    layer = data_layer(summaries)
    counted = [(s["symptom"], s["phase"], s["laps"]) for c in layer["corners"] for s in c["symptoms"]]
    assert counted == [(symptom, "apex_3", 4)]
    assert layer["corners"][0]["recommended"] is not None


# --- Phase 3: config-driven cross-lap aggregation (median vs worst_lap) -----
#
# classification.cs_cross_lap_aggregation ("median"|"worst_lap", absent ==
# "median"): the cross-LAP combiner aggregate_by_corner applies to each
# phase's CS_ratio before classify_fn's own existing min-across-PHASES
# search (_classify_corner) runs on top. "worst_lap" is min-then-min
# (thesis_notes.md "Gated Stage-2 recomputation...") -- min is associative,
# so composing a per-phase min-across-laps here with classify_fn's own
# min-across-phases IS the true global min over every (lap, phase) pair,
# with no separate flattening step needed. Stability stays median
# unconditionally in both modes.

def _healthy_phase_block():
    return {"cs_ratio_f": {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 50},
            "cs_ratio_r": {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 50},
            "stability_observed_Nm_per_deg": {"median": 500.0, "p25": 400.0, "p75": 600.0, "n": 50},
            "n_samples": 50, "kerb_fraction": 0.0, "valid_fraction_stab": 1.0}


def _corner_summaries_one_bad_lap(bad_lap, axle_key, bad_value, phase="exit_4", n_laps=4):
    # Exactly one of n_laps laps carries a bad value on the named phase/axle
    # (default exit_4/cs_ratio_f -- NOT apex_3, so apex_region's own
    # (here NaN, no apex_region data supplied) substitution in
    # _classify_corner never masks it); every other phase/lap is healthy.
    # Deliberately avoids apex_3 so this test isolates the cross-lap
    # combiner from the separate apex_region substitution mechanism.
    summaries = []
    for lap in range(1, n_laps + 1):
        phases = {p: _healthy_phase_block() for p in
                   ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")}
        if lap == bad_lap:
            phases[phase][axle_key] = {"median": bad_value, "p25": bad_value, "p75": bad_value, "n": 50}
        summaries.append({
            "lap_number": lap, "corner_number": 1, "speed_class": "medium",
            "apex_time": 5.0, "apex_speed": 25.0, "apex_lateral_g": 1.0,
            "method": "test", "warnings": [], "apex_position_x_m": None, "apex_position_y_m": None,
            "stable_corner_id": 1, "bracket_start_m": 0.0, "bracket_end_m": 100.0,
            "phases": phases, "apex_region": None,
        })
    return summaries


def _monkeypatch_cs_aggregation(monkeypatch, mode):
    # cs_cross_lap_aggregation is read via a local `from modules.
    # stability_analysis import load_parameters` inside aggregate_by_corner
    # (and _classify_corner does the identical local import for the rest of
    # the classification block) -- patching the module attribute is seen by
    # both. mode=None removes the key entirely (tests the documented
    # "absent -> median" default, not just an explicit "median" value).
    import modules.stability_analysis as sa
    real_params = sa.load_parameters()
    fake = dict(real_params)
    fake["classification"] = dict(real_params["classification"])
    if mode is None:
        fake["classification"].pop("cs_cross_lap_aggregation", None)
    else:
        fake["classification"]["cs_cross_lap_aggregation"] = mode
    monkeypatch.setattr(sa, "load_parameters", lambda: fake)


def test_aggregate_by_corner_worst_lap_mode_takes_min_across_laps(monkeypatch):
    _monkeypatch_cs_aggregation(monkeypatch, "worst_lap")
    from modules.corner_aggregation import aggregate_by_corner

    summaries = _corner_summaries_one_bad_lap(bad_lap=2, axle_key="cs_ratio_f", bad_value=-0.7)
    aggregated = aggregate_by_corner(summaries)
    exit4 = aggregated[1]["phases"]["exit_4"]
    assert exit4["cs_ratio_f"]["median"] == -0.7
    # stability must stay median regardless of the CS mode -- the one bad
    # lap's stability value (500.0, unchanged) must not shift the aggregate.
    assert exit4["stability_observed_Nm_per_deg"]["median"] == 500.0


def test_aggregate_by_corner_median_mode_is_default_when_key_absent(monkeypatch):
    _monkeypatch_cs_aggregation(monkeypatch, None)
    from modules.corner_aggregation import aggregate_by_corner

    summaries = _corner_summaries_one_bad_lap(bad_lap=2, axle_key="cs_ratio_f", bad_value=-0.7)
    aggregated = aggregate_by_corner(summaries)
    # 3 healthy (1.0) laps + 1 bad (-0.7) lap -> median([1, 1, 1, -0.7]) = 1.0:
    # the one-off lap washes out, exactly the pre-Phase-3 behaviour.
    assert aggregated[1]["phases"]["exit_4"]["cs_ratio_f"]["median"] == 1.0


def test_repeat_rule_blocks_one_off_lap_under_worst_lap_aggregation(monkeypatch):
    # worst_lap aggregation makes the AGGREGATE sensitive to a single bad lap
    # (min is outlier-sensitive by construction), but the setup engine counts
    # a symptom only when it repeats per lap (D5: majority of valid laps, at
    # least 2) -- a lone bad lap must not, by itself, produce a
    # recommendation. Successor of the retired consistency-gate check.
    _monkeypatch_cs_aggregation(monkeypatch, "worst_lap")
    from modules.corner_aggregation import aggregate_by_corner
    from modules.classifier import classify_corner
    from modules.setup_engine import data_layer

    # threshold-relative: 0.2 below the live STRONG_CSF
    import modules.stability_analysis as sa
    bad_value = sa.load_parameters()["classification"]["STRONG_CSF"]["value"] - 0.2
    summaries = _corner_summaries_one_bad_lap(bad_lap=2, axle_key="cs_ratio_f", bad_value=bad_value)

    aggregated = aggregate_by_corner(summaries)
    severity, short, _long, _state = classify_corner(aggregated[1])
    # the aggregate surfaces the bad lap (D2: front in the strong tier)
    assert severity == "strong" and "understeer" in short

    layer = data_layer(summaries)
    assert not layer["corners"], (
        f"a single one-off bad lap produced a recommendation under worst_lap aggregation: {layer['corners']}")
