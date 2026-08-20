# Phase 2 -- invariant tests on corner phase boundaries
# (modules/corner_analysis.py's canonical, production-facing corner
# output -- what analyse_corners() actually returns and ui/views/
# outing_form.py renders).
#
# UNLIKE the golden-file tests in test_golden_pipeline.py, a FAILURE HERE
# IS A GENUINE FINDING, not a "something changed" signal -- these are
# general physical/structural properties any correct phase segmentation
# must satisfy, independent of the exact numbers. Per instruction: if any
# test below fails against current behaviour, that failure is recorded
# prominently in the session report, the test is NOT weakened to pass,
# and the underlying code is NOT fixed (that is the user's call, not
# this suite's).
#
# These are exactly the class of test that would have caught this week's
# two broken entry_1_brake fixes (thesis_notes.md "entry_1_brake
# phase-boundary bug: mechanism, blast radius, and fix", 2026-08-20):
# the first attempt produced a healthy-looking bounded/non-monotonic
# duration distribution and would have passed a naive "is it bounded"
# check, but failed the brake-pressure cross-check below (test 5) --
# which is why that check is marked MANDATORY in this file too, exactly
# as it was in the diagnostic that caught the bug.

import numpy as np
import pytest

PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]

# Physically-reasoned bound, not a pinned historical value (a pinned
# value would just be a second golden test, not a general invariant).
# The longest entry_1_brake ever measured on this dataset is 3.421s
# (thesis_notes.md, corrected-construction re-verification) -- 8s gives
# a >2x margin, corresponding to braking from well beyond 450m at
# typical GT3 corner-entry speeds (~55 m/s), which is not a plausible
# single continuous lift-coast-brake zone on any real circuit corner.
MAX_PLAUSIBLE_BRAKE_PHASE_S = 8.0

# "Implausible fraction of a lap" for a SINGLE PHASE of a SINGLE corner.
# Dubai has 14 stable corners per lap (thesis_notes.md WP1 arc); if
# corner brackets were evenly distributed, one whole corner would average
# ~1/14 = 7% of a lap. 25% for a single PHASE (never mind a whole corner)
# is already >3x that average whole-corner share -- a generous ceiling
# chosen to catch a gross structural bug (like the entry_1_brake
# unbounded-lookback bug, which consumed 85% of the entire DATASET) long
# before it approaches anything a real corner geometry could produce.
MAX_PHASE_FRACTION_OF_LAP = 0.25

BRAKE_RISE_BAR = 5.0  # matches diagnostics/inspect_entry1_brake_fix_verification.py exactly


def _phase_span(corner, phase):
    start_t, end_t = corner["segments"][phase]
    return start_t, end_t, end_t - start_t


def test_entry_1_brake_bounded(valid_lap_corners):
    """Invariant 1a: entry_1_brake duration never exceeds a physically
    plausible bound. This is the direct check the pre-fix bug would have
    failed outright (106.8s measured against an 8s bound)."""
    violations = []
    for c in valid_lap_corners:
        _, _, dur = _phase_span(c, "entry_1_brake")
        if dur > MAX_PLAUSIBLE_BRAKE_PHASE_S:
            violations.append((c["lap_number"], c["corner_number"], c.get("stable_corner_id"), dur))
    assert not violations, (
        f"entry_1_brake exceeds {MAX_PLAUSIBLE_BRAKE_PHASE_S}s bound for "
        f"{len(violations)} corner instance(s): {violations}"
    )


def test_entry_1_brake_not_monotonic_within_lap(valid_lap_corners):
    """Invariant 1b: entry_1_brake duration must not grow monotonically
    (weakly non-decreasing at every step) across an entire lap's corner
    sequence. This is the exact signature the pre-fix unbounded-lookback
    bug produced (1.9s -> 106.8s, growing with every corner because the
    search reached further into the lap each time).

    HEURISTIC, not a fully general invariant: a real, coincidentally
    monotonic braking-duration pattern across one lap's corners is not
    impossible in principle. On this dataset it does not occur under
    correct behaviour (verified below); if a future lap/track genuinely
    triggers this, that is worth a human look, not an automatic "the
    code is broken" verdict -- noted here so a future failure is read
    with that context, not treated as an automatic finding.
    """
    by_lap = {}
    for c in valid_lap_corners:
        by_lap.setdefault(c["lap_number"], []).append(c)

    monotonic_laps = []
    for lap_num, corners in by_lap.items():
        corners_sorted = sorted(corners, key=lambda c: c["apex_time"])
        durs = [_phase_span(c, "entry_1_brake")[2] for c in corners_sorted]
        if len(durs) < 2:
            continue
        if all(b >= a for a, b in zip(durs, durs[1:])):
            monotonic_laps.append((lap_num, durs))
    assert not monotonic_laps, (
        f"entry_1_brake duration is weakly monotonically increasing across the whole lap for "
        f"{len(monotonic_laps)} lap(s) -- matches the pre-fix bug's signature: {monotonic_laps}"
    )


def test_no_phase_spans_implausible_fraction_of_lap(valid_lap_corners, laps_by_number):
    """Invariant 2: no single phase of a single corner instance consumes
    more than MAX_PHASE_FRACTION_OF_LAP of that lap's total duration."""
    violations = []
    for c in valid_lap_corners:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None:
            continue
        lap_dur = lap["end_time"] - lap["start_time"]
        if lap_dur <= 0:
            continue
        for phase in PHASE_KEYS:
            _, _, dur = _phase_span(c, phase)
            frac = dur / lap_dur
            if frac > MAX_PHASE_FRACTION_OF_LAP:
                violations.append((c["lap_number"], c["corner_number"], phase, round(frac, 3)))
    assert not violations, (
        f"{len(violations)} phase instance(s) exceed {MAX_PHASE_FRACTION_OF_LAP:.0%} of their "
        f"lap's duration: {violations}"
    )


def test_phases_ordered_and_non_overlapping(valid_lap_corners):
    """Invariant 3: within one corner instance, the five phases form a
    single contiguous, non-decreasing chain -- entry_1_brake.end ==
    entry_2_turnin.start, entry_2_turnin.end == apex_3.start == apex_3.end
    == exit_4.start, exit_4.end == exit_5.start, and every phase has
    end >= start. This is NOT structurally guaranteed for the realized
    (production) corners: modules/corner_analysis.py's
    _realize_canonical_corners computes each canonical boundary
    (brake_s/turnin_s/apex_m/half_s/end_m) as an INDEPENDENT per-boundary
    median across cluster members, then inverts each independently back
    to a per-lap time via _invert_s_to_t -- unlike the raw per-lap
    detection in _build_corner, where the chain is built from a single
    shared bracket and is ordered by construction. A violation here would
    be a genuine, currently-unverified finding about the canonicalization
    step, not a re-statement of something corner_analysis.py already
    guarantees.
    """
    violations = []
    for c in valid_lap_corners:
        spans = {phase: _phase_span(c, phase) for phase in PHASE_KEYS}
        for phase in PHASE_KEYS:
            start_t, end_t, dur = spans[phase]
            if end_t < start_t:
                violations.append((c["lap_number"], c["corner_number"], phase, "end < start", dur))
        chain = [
            ("entry_1_brake.end", "entry_2_turnin.start", spans["entry_1_brake"][1], spans["entry_2_turnin"][0]),
            ("entry_2_turnin.end", "apex_3.start", spans["entry_2_turnin"][1], spans["apex_3"][0]),
            ("apex_3.end", "exit_4.start", spans["apex_3"][1], spans["exit_4"][0]),
            ("exit_4.end", "exit_5.start", spans["exit_4"][1], spans["exit_5"][0]),
        ]
        for name_a, name_b, va, vb in chain:
            if not np.isclose(va, vb, rtol=0, atol=1e-9):
                violations.append((c["lap_number"], c["corner_number"], f"{name_a} != {name_b}", va, vb))
    assert not violations, (
        f"{len(violations)} phase-ordering violation(s) found: {violations[:20]}"
        + (f" ... and {len(violations) - 20} more" if len(violations) > 20 else "")
    )


def test_brake_start_never_precedes_prior_corner_bracket(valid_lap_corners):
    """Invariant 4: within a lap, sorted by apex_time, no corner's
    entry_1_brake start may precede the PRECEDING corner's own exit_5 end
    -- reproduces diagnostics/inspect_entry1_brake_fix_verification.py's
    "(3) INHERITED-LOOKBACK RISK" check exactly (same sort key, same
    comparison), which that diagnostic ran once, post-fix, and found zero
    violations. This test makes that a standing, re-run-on-every-change
    guarantee instead of a one-off observation.
    """
    by_lap = {}
    for c in valid_lap_corners:
        by_lap.setdefault(c["lap_number"], []).append(c)

    violations = []
    for lap_num, corners in by_lap.items():
        corners_sorted = sorted(corners, key=lambda c: c["apex_time"])
        for i in range(1, len(corners_sorted)):
            prev_c, cur_c = corners_sorted[i - 1], corners_sorted[i]
            prev_end = prev_c["segments"]["exit_5"][1]
            cur_brake_start = cur_c["segments"]["entry_1_brake"][0]
            if cur_brake_start < prev_end:
                violations.append((
                    lap_num, cur_c["corner_number"], prev_c["corner_number"],
                    round(prev_end - cur_brake_start, 4)
                ))
    assert not violations, (
        f"{len(violations)} corner(s) whose entry_1_brake reaches back into the preceding corner's "
        f"own bracket (lap, corner, prev_corner, overlap_s): {violations}"
    )


def test_brake_start_precedes_brake_pressure_rise(valid_lap_corners, parsed_data):
    """Invariant 5, MANDATORY -- the external physical check.
    entry_1_brake's start must precede the corresponding brake-pressure
    rise (log_pbrake_f/log_pbrake_r crossing BRAKE_RISE_BAR) wherever a
    rise is found in the search window, reproducing diagnostics/
    inspect_entry1_brake_fix_verification.py check (b) EXACTLY (same
    5.0 bar threshold, same window logic) -- this is the check that
    caught the first, subtly-wrong fix attempt (median offset ~0.004s,
    statistically indistinguishable from zero) when two other checks
    (bounded, non-monotonic) both looked healthy. This is a real sensor
    channel, not a value derived from the same estimator being tested --
    exactly the kind of independent check the standing project rule
    requires (distributional plausibility is not verification).
    """
    channels = parsed_data.get("channels", {})
    laps_by_number = {l["lap_number"]: l for l in parsed_data.get("laps", [])}

    violations = []
    n_checked = 0
    for label in ("log_pbrake_f", "log_pbrake_r"):
        ch = channels.get(label)
        if ch is None or ch.get("time") is None:
            pytest.skip(f"{label} channel not available in parsed Dubai data -- cannot run the external check")
        for c in valid_lap_corners:
            brake_start_t, s_t_start = c["segments"]["entry_1_brake"]
            window_mask = (ch["time"] >= brake_start_t) & (ch["time"] <= s_t_start + 5.0)
            if not window_mask.any():
                continue
            wt, wd = ch["time"][window_mask], ch["data"][window_mask]
            rise_idx = np.where(wd > BRAKE_RISE_BAR)[0]
            if len(rise_idx) == 0:
                continue  # no rise in window for this corner -- "where a rise exists" does not apply
            n_checked += 1
            rise_t = wt[rise_idx[0]]
            if rise_t < brake_start_t:
                violations.append((label, c["lap_number"], c["corner_number"], round(rise_t - brake_start_t, 4)))

    assert n_checked > 0, "no corner had a detectable brake-pressure rise in either channel -- check is vacuous"
    assert not violations, (
        f"{len(violations)}/{n_checked} instance(s) where the brake-pressure rise PRECEDES "
        f"entry_1_brake's own start (label, lap, corner, rise_minus_start_s): {violations}"
    )


def test_phase_sample_count_consistent_with_span(valid_lap_corners, state):
    """Invariant 6: the raw sample count found in [start_t, end_t) via
    the same searchsorted convention summarise_corners itself uses
    matches what the time span and sample rate predict, within rounding
    at both boundaries (+/-1 sample per edge, so +/-2 total). Independent
    of the moving/kerb mask (that's summarise_corners' own n_samples
    field, a different, additionally-filtered count) -- this tests the
    phase WINDOW itself against the sample grid, not which of those
    samples later get counted as "moving".
    """
    t = state["time"]
    sr = state["sample_rate_hz"]
    violations = []
    for c in valid_lap_corners:
        for phase in PHASE_KEYS:
            start_t, end_t, dur = _phase_span(c, phase)
            if end_t < start_t:
                continue  # already flagged by test_phases_ordered_and_non_overlapping
            lo = int(np.searchsorted(t, start_t, side="left"))
            hi = int(np.searchsorted(t, end_t, side="right"))
            n_actual = hi - lo
            n_expected = dur * sr
            if abs(n_actual - n_expected) > 2:
                violations.append((c["lap_number"], c["corner_number"], phase,
                                    n_actual, round(n_expected, 2)))
    assert not violations, (
        f"{len(violations)} phase instance(s) where sample count disagrees with span*sample_rate "
        f"by more than 2 samples (lap, corner, phase, n_actual, n_expected): {violations[:20]}"
    )
