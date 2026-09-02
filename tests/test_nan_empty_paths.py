# Phase 5 -- NaN and empty-path coverage.
#
# The entry_1_brake phase-boundary fix (thesis_notes.md, 2026-08-20) left
# 22 of 56 corner instances with a zero-length braking phase -- a
# structural NaN/empty-input path that is now a REAL, frequently-
# exercised part of production data, not a hypothetical edge case. This
# file confirms none of the four consumers named in the work order raise
# on it, and PINS what they actually return (regression, not
# correctness -- see test_golden_pipeline.py's docstring for that
# distinction; a NaN/zero return being "handled without crashing" is not
# a claim that it is the most informative possible return value).

import math

import pytest

from modules.recommendation import aggregate_by_corner, PHASE_KEYS


def _all_nan_phase():
    return {
        "cs_ratio_f": {"median": float("nan")},
        "cs_ratio_r": {"median": float("nan")},
        "stability_observed_Nm_per_deg": {"median": float("nan")},
    }


def _full_phase_stub(n_samples=0):
    """Matches summarise_corners' own zero-sample phase shape exactly
    (modules/stability_analysis.py, the n_samples==0 branch) --
    replicated here rather than imported since it's an inline dict
    literal inside a loop body, not its own function."""
    nan_stat = {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": 0}
    return {
        "n_samples": n_samples,
        "valid_fraction_stab": 0.0,
        "kerb_fraction": 0.0,
        "cs_ratio_f": nan_stat,
        "cs_ratio_r": nan_stat,
        "stability_observed_Nm_per_deg": nan_stat,
    }


# --- summarise_corners with a zero-sample phase (real production data) ------

def test_zero_length_phases_actually_occur(pipeline_result):
    """Confirms this path is genuinely exercised by the fixture data, not
    accidentally avoided -- if this ever reads 0, the other tests in this
    section are vacuously passing and that must be investigated, not
    treated as a clean bill of health."""
    n_zero = sum(
        1 for s in pipeline_result["summaries"] for p in s["phases"].values() if p["n_samples"] == 0
    )
    assert n_zero > 0, (
        "no zero-sample phase found in the fixture data -- either the entry_1_brake fix's "
        "known zero-length-phase population (thesis_notes.md) has changed, or this fixture's "
        "configuration differs from what that finding was measured under"
    )


def test_zero_length_phase_stats_are_well_formed_nan(pipeline_result):
    """Every zero-sample phase's stat blocks must be the well-formed NaN
    sentinel {median: nan, p25: nan, p75: nan, n: 0} -- not, say, a
    silently substituted 0.0 that would read as a real (and misleading)
    measurement."""
    violations = []
    for s in pipeline_result["summaries"]:
        for phase_name, p in s["phases"].items():
            if p["n_samples"] != 0:
                continue
            for stat_key in ("cs_ratio_f", "cs_ratio_r", "stability_observed_Nm_per_deg"):
                stat = p[stat_key]
                if stat["n"] != 0 or not (math.isnan(stat["median"]) and math.isnan(stat["p25"])
                                          and math.isnan(stat["p75"])):
                    violations.append((s["lap_number"], s["corner_number"], phase_name, stat_key, stat))
    assert not violations, f"{len(violations)} zero-sample phase(s) with non-NaN stats: {violations[:10]}"


# --- aggregate_by_corner with all-NaN input across laps ----------------------

def test_aggregate_by_corner_all_nan_across_laps_does_not_raise():
    summaries = [
        {"stable_corner_id": 1, "speed_class": "medium",
         "phases": {phase: _all_nan_phase() for phase in PHASE_KEYS}}
        for _lap in range(4)
    ]
    aggregated = aggregate_by_corner(summaries)
    assert 1 in aggregated
    for phase in PHASE_KEYS:
        p = aggregated[1]["phases"][phase]
        assert math.isnan(p["cs_ratio_f"]["median"])
        assert math.isnan(p["cs_ratio_r"]["median"])
        assert math.isnan(p["stability_observed_Nm_per_deg"]["median"])


def test_aggregate_by_corner_empty_summaries_does_not_raise():
    assert aggregate_by_corner([]) == {}


# --- the classifier with NaN inputs -------------------------------------------

def test_classifier_all_nan_phases_does_not_raise_and_reads_normal():
    """A corner where every phase's stats are NaN must classify as
    "normal" (no verdict can be formed from no data) rather than raising
    or defaulting to a false positive -- _classify_corner's own
    worst_f_val/worst_r_val/worst_stab_val search starts at a sentinel
    (1.0 / 1.0 / 1e9) and only updates on a value that compares as
    strictly better via `csf == csf` (NaN-safe), so an all-NaN input
    should leave every "worst" tracker at its untouched sentinel and
    fall through to severity="normal"."""
    try:
        from ui.views.outing_form import OutingForm
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")

    summary = {"phases": {phase: _full_phase_stub(n_samples=0) for phase in PHASE_KEYS}}
    severity, short, long_v, colour = OutingForm._classify_corner(None, summary)
    assert severity == "normal"
    assert "ok" in short


def test_classifier_partial_nan_phases_does_not_raise():
    """Mix of real and all-NaN phases (the actual shape of a corner with
    a zero-length entry_1_brake but normal other phases) -- must not
    raise, and must classify from the non-NaN phases only."""
    try:
        from ui.views.outing_form import OutingForm
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")

    phases = {phase: _full_phase_stub(n_samples=0) for phase in PHASE_KEYS}
    phases["apex_3"] = {
        "n_samples": 20, "valid_fraction_stab": 1.0, "kerb_fraction": 0.0,
        # -0.20 is below STRONG_CSF (-0.10, threshold anchoring, 2026-09-02 --
        # updated from the old +0.05/0.1 pair, which pre-dates the physical-
        # anchor-at-zero threshold redesign; thesis_notes.md "Threshold
        # anchoring, Phase 2")
        "cs_ratio_f": {"median": -0.20, "p25": -0.22, "p75": -0.18, "n": 20},
        "cs_ratio_r": {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 20},
        "stability_observed_Nm_per_deg": {"median": 500.0, "p25": 480.0, "p75": 520.0, "n": 20},
    }
    summary = {"phases": phases}
    severity, short, long_v, colour = OutingForm._classify_corner(None, summary)
    assert severity in ("strong", "moderate")
    assert "understeer" in short


# --- recommendation rules against a corner with no computable braking signal -

def test_recommendation_engine_handles_real_zero_braking_corner(pipeline_result):
    """Runs the actual recommendation engine against the real, current
    pipeline output restricted to a corner known to have a zero-length
    entry_1_brake phase for at least one lap -- confirms no exception,
    and that entry_1_brake-keyed rules do not fire from a NaN "verdict"
    (a rule requiring a repeated verdict across laps cannot be satisfied
    by a phase with nothing to classify)."""
    try:
        from ui.views.outing_form import OutingForm
    except ImportError as e:
        pytest.skip(f"ui.views.outing_form not importable in this environment ({e})")

    from modules.recommendation import generate_recommendations, load_recommendations_config

    zero_brake_corner_ids = {
        s["stable_corner_id"] for s in pipeline_result["summaries"]
        if s["phases"]["entry_1_brake"]["n_samples"] == 0 and s.get("stable_corner_id") is not None
    }
    assert zero_brake_corner_ids, "no real zero-length entry_1_brake corner found in fixture data"

    target_id = sorted(zero_brake_corner_ids)[0]
    restricted_summaries = [
        s for s in pipeline_result["summaries"] if s.get("stable_corner_id") == target_id
    ]

    def classify_fn(summary):
        return OutingForm._classify_corner(None, summary)

    rec_config = load_recommendations_config()
    # Must not raise -- that is the primary assertion this test exists for.
    results = generate_recommendations(
        restricted_summaries, classify_fn, feedback_data={}, setup_data=None,
        config=rec_config, outing=None, driving_level=None,
    )
    for r in results:
        for cell_id in r.get("cell_ids", []):
            assert "brk" not in cell_id.lower() or any(
                c["stable_corner_id"] != target_id for c in r.get("corners", [])
            ), (
                f"rule {cell_id!r} fired keyed on entry_1_brake for corner {target_id}, "
                "which has no computable braking signal on the restricted lap set"
            )
