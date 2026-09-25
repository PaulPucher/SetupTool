# Decision-matrix frame. Three-layer recommendation frame (evidence ->
# candidates -> scoring), plus a conflict resolver. Pure Python, no Qt.
#
# Stage 1 (2026-09-02): one fully worked scenario (exit oversteer, LS-
# disambiguated) end to end, plus the two cheap-first plausibility checks,
# additive and parallel to the 39-rule engine in modules/recommendation.py.
#
# Stage 2 (Frame-Stage-2 Phase 3, 2026-09-04): full migration -- ALL 39
# config/recommendations.json rules re-expressed as candidate bridges
# (_bridge_candidates_for_matrix_rules), a conflict resolver
# (resolve_conflicts), and config-gated intervention evidence (Phase 3c).
# Parity-verified against the old engine on both real sessions
# (diagnostics/inspect_frame_stage2_parity.py) -- this is now the
# production recommendation UI (ui/views/outing_form.py's old
# Recommendations section is removed). modules/recommendation.py itself is
# UNCHANGED -- this module calls its rule definitions/config, it does not
# reimplement them; config/decision_frame.json documents every remaining
# seeded/null entry (parameter_windows, interaction_table gaps) for the
# user -- this module only acts on what that config actually populates.

import json

import numpy as np

from modules.stability_analysis import load_parameters
from modules.csv_parser import load_channels_config
from modules.wheel_loads import (
    CORNERS as _WHEEL_CORNERS, AXLE_CORNERS, TRAVEL_CHANNEL,
    _interp_channel, _normalize_travel_to_mm, _channel_is_dead,
)
from modules.recommendation import (
    PHASE_KEYS,
    PHASE_TO_FEEDBACK_KEY,
    SEVERITY_RANK,
    _NON_FIRING_STATUSES,
    _action_key,
    _axle_verdict,
    _current_setup_value,
    _feedback_row,
    _group_by_corner,
    _nanmin_or_nan,
    _phase_verdict,
    _verdict_present,
    aggregate_by_corner,
    load_recommendations_config,
    load_setup_parameters_registry,
)

DECISION_FRAME_CONFIG_PATH = "config/decision_frame.json"


def load_decision_frame_config():
    with open(DECISION_FRAME_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _axle_cs_severity(cs_median, strong_thresh, moderate_thresh):
    # Mirrors ui/views/outing_form.py _classify_corner's front_strong_cs/
    # front_moderate_cs boolean logic exactly (same "< strong, elif <
    # moderate" boundaries) but per-axle, not classify_fn's own combined/
    # dominant-axle output -- brake_balance_signature needs front and rear
    # severity INDEPENDENTLY on the same phase, which classify_fn's single
    # 4-tuple return cannot give (it already picked the worse axle).
    if cs_median != cs_median:  # NaN: no signal for this axle/phase
        return None
    if cs_median < strong_thresh:
        return "strong"
    if cs_median < moderate_thresh:
        return "moderate"
    return "normal"


def _count_repeating(cid, by_corner_laps, predicate):
    # Same counting shape as modules.recommendation._consistency_gate_ok
    # (how many of this corner's own laps independently satisfy
    # `predicate`), reused here to feed a graded confidence score rather
    # than a pass/fail gate -- the evidence layer reports uncertainty, it
    # does not suppress evidence outright the way the rule engine's
    # consistency gate does.
    laps = by_corner_laps.get(cid, [])
    if not laps:
        return 0, 0
    return sum(1 for lap in laps if predicate(lap)), len(laps)


def _fraction(numerator, denominator):
    return 0.0 if denominator == 0 else numerator / denominator


def aggregate_ls_by_corner(summaries):
    """Per-corner, per-phase worst-lap LS_ratio_f/r (min-then-min), same
    combiner as CS_ratio's own aggregation policy (config/parameters.json
    classification.cs_cross_lap_aggregation). LS_ratio has no separately
    derived cross-lap policy of its own (PLAN.md STEP 3: LS_ratio remains
    DISPLAY ONLY outside this evidence layer) -- reusing CS's policy here
    is a deliberate Stage 1 simplification, not a claim that LS_ratio's own
    aggregation has been independently derived; flagged as a Stage 2 open
    item. `summaries` are the raw per-lap-per-corner dicts from modules.
    stability_analysis.summarise_corners; entries built without ls=...
    passed to that call simply carry no ls_ratio_f/r keys per phase, and
    are treated as no-signal here (NaN), never fabricated.
    """
    by_id = _group_by_corner(summaries)
    out = {}
    for cid, laps in by_id.items():
        phases = {}
        for phase in PHASE_KEYS:
            f_vals, r_vals = [], []
            for lap in laps:
                p = lap["phases"].get(phase)
                if p is None:
                    continue
                lf = p.get("ls_ratio_f")
                lr = p.get("ls_ratio_r")
                if lf is not None and lf["median"] == lf["median"]:
                    f_vals.append(lf["median"])
                if lr is not None and lr["median"] == lr["median"]:
                    r_vals.append(lr["median"])
            phases[phase] = {
                "ls_ratio_f": _nanmin_or_nan(f_vals),
                "ls_ratio_r": _nanmin_or_nan(r_vals),
                "n_contributing_laps_f": len(f_vals),
                "n_contributing_laps_r": len(r_vals),
            }
        out[cid] = phases
    return out


def _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn, config=None):
    # Source (a): corner verdicts via the existing classify path -- the
    # same worst-lap aggregate and anchored thresholds the stability grid
    # already shows, so this evidence can never disagree with the UI for
    # the same corner/phase (identical mechanism modules.recommendation
    # relies on for the same reason).
    #
    # Metrology Phase 2 (2026-09-19, PLAN.md PARKED "Verdict-stability
    # annotation", now implemented): classify_fn (ui/views/outing_form.py
    # _classify_corner) appends a "[MARGINAL]" marker to `short` when the
    # verdict-driving axle sits within classification.verdict_stability_
    # margin.cs_margin of a threshold (Metrology Phase 1's own empirically
    # anchored value). A MARGINAL verdict discounts like low repeatability
    # already does here -- NOT a new confidence formula: it caps the same
    # repeat-fraction confidence via min(), reusing intervention_evidence.
    # abs.confidence's own 0.8 cap (Deepening Phase 4c) as the anchor,
    # rather than inventing a second discount constant. A weak repeat
    # pattern still reports honestly below 0.8; a MARGINAL verdict with a
    # strong repeat pattern is pulled down to 0.8, never inflated to it.
    margin_confidence_cap = (config or {}).get("intervention_evidence", {}).get("abs", {}).get("confidence", 0.8)
    evidence = []
    for cid, corner in aggregated.items():
        for phase in PHASE_KEYS:
            if phase not in corner["phases"]:
                continue
            severity, short = _phase_verdict(corner, [phase], classify_fn)
            if severity == "normal":
                continue

            verdicts_here = []
            axle = _axle_verdict(short)
            if axle is not None:
                verdicts_here.append(axle)
            if _verdict_present(short, "unstable_yaw"):
                verdicts_here.append("unstable_yaw")
            # classify_fn reports ONE severity for the phase as a whole
            # (its own combined/worse-of logic) -- when both an axle
            # verdict and yaw instability co-occur at the same phase, both
            # evidence items below carry that same phase-level severity;
            # this codebase has no separate per-verdict-type severity
            # scale to split it against (a documented Stage 1
            # simplification, not a fabricated distinction).
            for verdict in verdicts_here:
                def _lap_matches(lap_summary, phase=phase, verdict=verdict, severity=severity):
                    lap_sev, lap_short = _phase_verdict(lap_summary, [phase], classify_fn)
                    return (_verdict_present(lap_short, verdict)
                            and SEVERITY_RANK[lap_sev] >= SEVERITY_RANK[severity])

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_matches)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_marginal = "[MARGINAL]" in short
                if is_marginal:
                    confidence = min(confidence, margin_confidence_cap)

                evidence.append({
                    "type": "corner_verdict",
                    "corner": cid,
                    "phase": phase,
                    "speed_class": corner.get("speed_class"),
                    "verdict": verdict,
                    "severity": severity,
                    "confidence": confidence,
                    "marginal": is_marginal,
                    "source": f"classify_fn (worst-lap aggregate, anchored thresholds): "
                              f"C{cid} {phase} '{short}' -- repeats on {repeat}/{total} laps, "
                              f"signal present on {valid_laps}/{total} laps"
                              + (f", MARGINAL: capped at {margin_confidence_cap}" if is_marginal else ""),
                })
    return evidence


def _build_ls_disambiguation_evidence(aggregated, aggregated_ls, corner_verdict_evidence):
    # Source (b): traction-limited vs cornering-limited, for exit-phase
    # oversteer evidence only, only where LS_ratio is valid. Method carried
    # over UNCHANGED from diagnostics/inspect_ls_cs_disambiguation.py (its
    # own header: "no LS_ratio classification threshold exists in config...
    # DISPLAY-ONLY... not a production rule") -- Stage 1 promotes it to a
    # real evidence source, but the split itself is still population-
    # relative (median LS_ratio_r among this session's own oversteer-
    # evidence corners at the phase in question), not an absolute config
    # threshold, because no absolute LS_ratio threshold has ever been
    # derived for this project (PLAN.md STEP 4: "whether LS_ratio enters
    # the recommendation rules at all is UNDECIDED"). Flagged here and in
    # the Phase 7 close-out as a Stage 2 open item.
    exit_phases = ("exit_4", "exit_5")
    candidates = [e for e in corner_verdict_evidence
                  if e["verdict"] == "oversteer" and e["phase"] in exit_phases]
    if not candidates:
        return []

    population = []
    for e in candidates:
        ls_r = aggregated_ls.get(e["corner"], {}).get(e["phase"], {}).get("ls_ratio_r")
        if ls_r is not None and ls_r == ls_r:
            population.append(ls_r)
    if len(population) < 2:
        # A relative split needs at least two real values to be relative
        # TO -- one value has no population, same "only where valid" floor
        # as a single corner's own signal check below.
        return []
    ls_median = float(np.median(population))

    evidence = []
    for e in candidates:
        phase_ls = aggregated_ls.get(e["corner"], {}).get(e["phase"], {})
        ls_r = phase_ls.get("ls_ratio_r")
        if ls_r is None or ls_r != ls_r:
            continue  # LS invalid for this corner/phase -- no evidence, not a guess
        n_contrib = phase_ls.get("n_contributing_laps_r", 0)
        # n_laps is this corner's own analysed-lap count (aggregate_by_
        # corner's own field) -- a true fraction of THIS corner's laps
        # that contributed a valid LS reading, never an assumed/hardcoded
        # lap count.
        corner_n_laps = aggregated.get(e["corner"], {}).get("n_laps", 0)
        confidence = round(_fraction(n_contrib, corner_n_laps), 3)
        evidence.append({
            "type": "ls_disambiguation",
            "corner": e["corner"],
            "phase": e["phase"],
            "speed_class": e.get("speed_class"),
            "verdict": e["verdict"],
            "severity": e["severity"],
            "confidence": confidence,
            "ls_class": "traction_limited" if ls_r < ls_median else "cornering_limited",
            "source": f"LS_ratio_r={ls_r:.3f} vs session median {ls_median:.3f} "
                      f"(population-relative split, n={len(population)} corners; "
                      f"method: diagnostics/inspect_ls_cs_disambiguation.py, "
                      f"no absolute LS_ratio threshold exists in config -- see PLAN.md STEP 4)",
        })
    return evidence


def _build_ls_threshold_evidence(aggregated_ls, by_corner_laps, config):
    """LS threshold decision (2026-09-20, user + reviewer, resolving
    PLAN.md PARKED "LS_ratio threshold proposal"): an ABSOLUTE-threshold
    evidence source, independent of _build_ls_disambiguation_evidence's
    own population-relative split above -- config/decision_frame.json's
    own ls_threshold_evidence.STRONG_LSF/LSR (-0.79/-0.60, LS-evidence
    work package census, thesis_notes.md "LS-evidence work package:
    negative-population co-occurrence census") fire whenever a corner-
    phase's own worst-lap LS_ratio_f/r crosses the threshold, on ANY
    phase (not gated on an existing oversteer corner_verdict the way
    ls_disambiguation is) -- STILL NOT a classification verdict tier:
    config/parameters.json's own classification block is untouched, this
    evidence never reaches _classify_corner/the UI's severity colour.

    PHASE-CONDITIONED CONFIDENCE, per the census's own finding that phase
    type (not axle) drives whether a negative reading corroborates:
    confidence is the SAME repeat-fraction x valid-fraction formula
    _build_corner_verdict_evidence already uses (no new formula) --
    fraction of this corner's OWN other laps that also cross the
    threshold at this phase, times fraction of laps with real signal.
    For exit_4/exit_5 specifically, that confidence is additionally
    capped via min() against ls_threshold_evidence.exit_phase_confidence_
    discount (0.5) -- the SAME MIN-confidence-cap mechanism the ABS-
    masking bridge and the MARGINAL-verdict cap already use, reusing
    intervention_evidence.abs.confidence's own precedent rather than a
    new formula. Repeatability across laps is what "lifts" a high-
    repeatability exit reading UP TO that discount ceiling rather than
    leaving it at whatever a low, unrepeated reading's own raw fraction
    would give (the C3 pattern: exit_5 rear, repeat=4/4, corroborated in
    the census via repeatability alone with zero TC activity) -- min()
    never raises a value, so a one-off exit reading still reports its
    own honestly-low fraction, never inflated to the discount.

    Braking/turn-in/apex phases carry NO discount (89%/72% corroborated
    in the census) -- their own repeat-fraction confidence is reported
    as-is, uncapped.
    """
    ls_cfg = config.get("ls_threshold_evidence", {})
    if not ls_cfg.get("enabled", False):
        return []
    strong_lsf = ls_cfg["STRONG_LSF"]
    strong_lsr = ls_cfg["STRONG_LSR"]
    exit_phases = tuple(ls_cfg.get("exit_phases", ("exit_4", "exit_5")))
    braking_turnin_phases = tuple(ls_cfg.get("braking_turnin_phases",
                                              ("entry_1_brake", "entry_2_turnin", "apex_3")))
    exit_discount = ls_cfg["exit_phase_confidence_discount"]

    evidence = []
    for cid, phases in aggregated_ls.items():
        for phase in braking_turnin_phases + exit_phases:
            p = phases.get(phase)
            if p is None:
                continue
            for axle, key, thresh, verdict in (
                ("front", "ls_ratio_f", strong_lsf, "brake_limited"),
                ("rear", "ls_ratio_r", strong_lsr, "traction_limited"),
            ):
                val = p.get(key)
                if val is None or val != val or val >= thresh:
                    continue

                def _lap_crosses(lap_summary, phase=phase, key=key, thresh=thresh):
                    lv_entry = lap_summary["phases"].get(phase, {}).get(key)
                    lv = lv_entry.get("median") if isinstance(lv_entry, dict) else None
                    return lv is not None and lv == lv and lv < thresh

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_crosses)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_exit = phase in exit_phases
                if is_exit:
                    confidence = min(confidence, exit_discount)

                evidence.append({
                    "type": "ls_threshold",
                    "corner": cid,
                    "phase": phase,
                    "axle": axle,
                    "verdict": verdict,
                    "confidence": confidence,
                    "phase_scope": "exit" if is_exit else "braking_turnin",
                    "source": f"LS_ratio_{('f' if axle=='front' else 'r')}={val:.3f} < "
                              f"{'STRONG_LSF' if axle=='front' else 'STRONG_LSR'}={thresh:.2f} "
                              f"(LS-evidence census, thesis_notes.md 'LS-evidence work package'); "
                              f"repeats on {repeat}/{total} laps, signal on {valid_laps}/{total} laps"
                              + (f"; exit-phase confidence capped at {exit_discount} "
                                 f"(TC corroboration structurally unavailable this session, "
                                 f"34% corroborated in the census)" if is_exit else ""),
                })
    return evidence


def _build_brake_balance_evidence(aggregated, by_corner_laps, config):
    # Source (c): front axle beyond its own limit while rear stays healthy
    # during braking (config/decision_frame.json plausibility_checks.
    # brake_balance_signature). Reads config/parameters.json's own
    # classification thresholds directly (never duplicated as raw numbers
    # in config/decision_frame.json, which states only the severity labels
    # -- see that file's own comment) so this can never drift from
    # classify_fn's thresholds when they are re-derived.
    settings = config["plausibility_checks"]["brake_balance_signature"]
    phase = settings["phase"]
    front_min = settings["front_beyond_severity_min"]
    rear_max = settings["rear_healthy_severity_max"]
    cls_cfg = load_parameters()["classification"]
    strong_csf = cls_cfg["STRONG_CSF"]["value"]
    moderate_csf = cls_cfg["MODERATE_CSF"]["value"]
    strong_csr = cls_cfg["STRONG_CSR"]["value"]
    moderate_csr = cls_cfg["MODERATE_CSR"]["value"]

    def _axle_severities(summary):
        p = summary["phases"].get(phase)
        if p is None:
            return None, None
        f_sev = _axle_cs_severity(p["cs_ratio_f"]["median"], strong_csf, moderate_csf)
        r_sev = _axle_cs_severity(p["cs_ratio_r"]["median"], strong_csr, moderate_csr)
        return f_sev, r_sev

    def _fires(summary):
        f_sev, r_sev = _axle_severities(summary)
        if f_sev is None or r_sev is None:
            return False
        return (SEVERITY_RANK[f_sev] >= SEVERITY_RANK[front_min]
                and SEVERITY_RANK[r_sev] <= SEVERITY_RANK[rear_max])

    evidence = []
    for cid, corner in aggregated.items():
        if not _fires(corner):
            continue
        f_sev, r_sev = _axle_severities(corner)
        repeat, total = _count_repeating(cid, by_corner_laps, _fires)
        valid_laps = sum(
            1 for lap in by_corner_laps.get(cid, [])
            if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
        )
        confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
        evidence.append({
            "type": "plausibility_brake_balance",
            "corner": cid,
            "phase": phase,
            "speed_class": corner.get("speed_class"),
            "verdict": None,
            "severity": f_sev,
            "confidence": confidence,
            "source": f"C{cid} {phase}: front CS severity={f_sev} (>= {front_min}), "
                      f"rear CS severity={r_sev} (<= {rear_max}) -- front beyond limit, "
                      f"rear healthy; repeats on {repeat}/{total} laps",
        })
    return evidence


# --- Stage 2: matrix-rule verdict evidence -----------------------------
#
# Source (d): the same classify_fn/_phase_verdict call the OLD engine's
# own "data"-trigger rules use (modules.recommendation._evaluate_rule),
# generalised from Stage 1's own single-PHASE_KEYS loop to the small set
# of PHASE GROUPS the 39-rule matrix actually uses (derived from config/
# recommendations.json's own rules, never hardcoded -- verified 2026-09-04
# that every live ("elicited") matrix rule uses one of five groups: the
# four single phases already covered by _build_corner_verdict_evidence,
# plus (exit_4, exit_5) as one 2-phase unit for the six matrix_*_exit_*
# rules specifically. A SEPARATE evidence type ("matrix_verdict", not
# "corner_verdict") is used rather than changing corner_verdict's own
# shape, so Stage 1's existing exit-oversteer/ls_disambiguation code (and
# its own passing tests) are completely unaffected by this addition.

def _distinct_phase_groups(config_recs):
    return sorted({tuple(r["phases"]) for r in config_recs["rules"]})


def _build_matrix_verdict_evidence(aggregated, by_corner_laps, classify_fn, config_recs, config=None):
    # Metrology Phase 2: same MARGINAL discount as _build_corner_verdict_
    # evidence, applied here too -- this is the SAME classify_fn/short
    # output, grouped over multiple phases rather than one; leaving this
    # path uncapped would let a MARGINAL verdict still drive the 39-rule
    # matrix engine (this evidence type, not corner_verdict, feeds most
    # real candidates) at full confidence.
    margin_confidence_cap = (config or {}).get("intervention_evidence", {}).get("abs", {}).get("confidence", 0.8)
    evidence = []
    for phase_group in _distinct_phase_groups(config_recs):
        phases = list(phase_group)
        for cid, corner in aggregated.items():
            if not any(p in corner["phases"] for p in phases):
                continue
            severity, short = _phase_verdict(corner, phases, classify_fn)
            if severity == "normal":
                continue

            verdicts_here = []
            axle = _axle_verdict(short)
            if axle is not None:
                verdicts_here.append(axle)
            if _verdict_present(short, "unstable_yaw"):
                verdicts_here.append("unstable_yaw")

            for verdict in verdicts_here:
                def _lap_matches(lap_summary, phases=phases, verdict=verdict, severity=severity):
                    lap_sev, lap_short = _phase_verdict(lap_summary, phases, classify_fn)
                    return (_verdict_present(lap_short, verdict)
                            and SEVERITY_RANK[lap_sev] >= SEVERITY_RANK[severity])

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_matches)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if any(lap["phases"].get(p, {}).get("n_samples", 0) > 0 for p in phases)
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_marginal = "[MARGINAL]" in short
                if is_marginal:
                    confidence = min(confidence, margin_confidence_cap)

                evidence.append({
                    "type": "matrix_verdict",
                    "corner": cid,
                    "phases": phase_group,
                    "speed_class": corner.get("speed_class"),
                    "verdict": verdict,
                    "severity": severity,
                    "confidence": confidence,
                    "marginal": is_marginal,
                    "source": f"classify_fn (worst-lap aggregate, anchored thresholds), phases="
                              f"{'+'.join(phase_group)}: C{cid} '{short}' -- repeats on {repeat}/{total} "
                              f"laps, signal present on {valid_laps}/{total} laps"
                              + (f", MARGINAL: capped at {margin_confidence_cap}" if is_marginal else ""),
                })
    return evidence


# --- Stage 2: intervention evidence (Phase 3c, config-gated, default off) -
#
# USABLE-NOW booleans only (Frame-Stage-2 Phase 2 classification survey,
# thesis_notes.md), wired at engineer-verbatim grade per the user's own
# rules (PLAN.md "DECISION FRAME -- STAGE 2 BACKLOG"). Corroborating
# evidence only -- these never generate a standalone candidate/action of
# their own, they are appended to an existing matching candidate's
# evidence_refs (see _bridge_candidates_for_matrix_rules and
# _exit_oversteer_candidates). Needs raw channel/state/corner data (per-
# lap phase time windows, config/recommendations.json "segments" field)
# that summaries alone do not carry -- corners/state/channels are all
# optional parameters of build_evidence, defaulting to None (skips
# intervention evidence entirely, same as use_intervention_evidence=False)
# so every existing Stage 1 caller/test is completely unaffected.

def _phase_window_indices(t, segment):
    # Mirrors modules.stability_analysis.summarise_corners's own internal
    # _phase_slice mechanism exactly (searchsorted on the phase's own
    # (start_t, end_t) segment) -- not reimplemented differently, just not
    # importable (that function is nested/private inside summarise_corners).
    if segment is None:
        return None
    start_t, end_t = segment
    if end_t < start_t:
        return None
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        return None
    return lo, hi


def _log_abs_pos_at(channels, t, idx):
    # Deepening Phase 4c: reports the trusted ABS map position (Phase 3
    # decision -- log_abs_pos, not abs_switch_pos) for traceability inside
    # an evidence item's own source string. Never used to change firing
    # logic (abs_position's own registry entry is explicitly non-
    # monotonic/categorical, so there is no validated "more/less
    # aggressive" ordering to route on).
    ch = (channels or {}).get("log_abs_pos")
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None or idx is None:
        return None
    lo, hi = idx
    val = np.interp(t, ch["time"], ch["data"])[lo:hi]
    if val.size == 0:
        return None
    return float(np.median(val))


def _build_intervention_abs_evidence(corners, by_corner_laps_ignored, state, channels, aggregated, abs_config):
    abs_ch = (channels or {}).get("abs_active")
    if state is None or abs_ch is None or abs_ch.get("quality") in ("missing", "failed") or abs_ch.get("time") is None:
        return []
    t = state["time"]
    abs_on_ref = np.interp(t, abs_ch["time"], abs_ch["data"]) > 0.5
    confidence_cap = abs_config.get("confidence", 1.0)
    heavy_cfg = abs_config.get("abs_heavy_masks_verdict", {})
    heavy_threshold = heavy_cfg.get("heavy_duty_cycle_threshold", 1.0)

    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        inactive_count, heavy_count, total = 0, 0, 0
        example_idx_inactive, example_idx_heavy = None, None
        for c in instances:
            idx = _phase_window_indices(t, c.get("segments", {}).get("entry_1_brake"))
            if idx is None:
                continue
            lo, hi = idx
            total += 1
            window = abs_on_ref[lo:hi]
            if not window.any():
                inactive_count += 1
                example_idx_inactive = idx
            elif window.mean() >= heavy_threshold:
                heavy_count += 1
                example_idx_heavy = idx

        if total and inactive_count:
            # User rule, verbatim: "instability/locking under braking +
            # ABS not intervening -> ABS map up". Corroborates unstable_
            # yaw matrix-verdict evidence on entry_1_brake -- see this
            # function's own caller in build_evidence for that gate.
            confidence = min(confidence_cap, round(inactive_count / total, 3))
            pos = _log_abs_pos_at(channels, t, example_idx_inactive)
            pos_txt = f", log_abs_pos={pos:.0f}" if pos is not None else ""
            evidence.append({
                "type": "intervention_abs",
                "corner": cid, "phases": ("entry_1_brake",),
                "speed_class": aggregated.get(cid, {}).get("speed_class"),
                "verdict": "unstable_yaw", "severity": None, "confidence": confidence,
                "source": f"abs_active read 0 throughout entry_1_brake on {inactive_count}/{total} analysed "
                          f"laps{pos_txt} -- corroborates braking-phase instability per the user's own rule: "
                          f"'ABS inactive + instability under braking -> more ABS'.",
            })

        if total and heavy_count:
            # NEW, Deepening Phase 4c. User rule, verbatim: "ABS
            # regulating heavily through braking zones -> flag as
            # masking, prefer brake-balance/platform levers". A masking
            # FLAG, not a verdict-corroborating evidence item -- verdict
            # is None on purpose (this fires independent of what the
            # phase's own CS/matrix verdict says, it questions whether
            # that verdict should be trusted at all).
            confidence = min(confidence_cap, round(heavy_count / total, 3))
            pos = _log_abs_pos_at(channels, t, example_idx_heavy)
            pos_txt = f", log_abs_pos={pos:.0f}" if pos is not None else ""
            evidence.append({
                "type": "intervention_abs_masking",
                "corner": cid, "phases": ("entry_1_brake",),
                "speed_class": aggregated.get(cid, {}).get("speed_class"),
                "verdict": None, "severity": None, "confidence": confidence,
                "masked_by_heavy_abs": True,
                "source": f"abs_active duty cycle >= {heavy_threshold:.0%} of entry_1_brake on {heavy_count}/"
                          f"{total} analysed laps{pos_txt} -- per the user's own rule, this corner/phase's own "
                          f"braking-phase verdict may reflect ABS regulation rather than raw mechanical "
                          f"balance; prefer brake-balance/platform levers over a literal reading.",
            })
    return evidence


def _build_intervention_tc_evidence(corners, state, channels, aggregated):
    tc_ch = (channels or {}).get("ecu_B_tc_act")
    if state is None or tc_ch is None or tc_ch.get("quality") in ("missing", "failed") or tc_ch.get("time") is None:
        return []
    t = state["time"]
    tc_on_ref = np.interp(t, tc_ch["time"], tc_ch["data"]) > 0.5

    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        active_count, total = 0, 0
        for c in instances:
            segs = c.get("segments", {})
            windows = [w for p in EXIT_PHASES for w in [_phase_window_indices(t, segs.get(p))] if w is not None]
            if not windows:
                continue
            total += 1
            if any(tc_on_ref[lo:hi].any() for lo, hi in windows):
                active_count += 1
        if total == 0 or active_count == 0:
            continue
        confidence = round(active_count / total, 3)
        evidence.append({
            "type": "intervention_tc",
            "corner": cid, "phases": EXIT_PHASES,
            "speed_class": aggregated.get(cid, {}).get("speed_class"),
            "verdict": "oversteer", "severity": None, "confidence": confidence,
            "source": f"ecu_B_tc_act read 1 during exit_4/exit_5 on {active_count}/{total} analysed laps "
                      f"(USABLE-NOW boolean, Frame-Stage-2 Phase 2) -- corroborates traction-limited per "
                      f"the user's own rule: 'TC cutting hard + exit oversteer -> corroborates "
                      f"traction-limited'.",
        })
    return evidence


# --- Kerb-strike-severity blowoff evidence (WP-ELICIT Phase C3, 2026-09-24) -
#
# Author-elicited direction: "curb strikes / hard bumps -> more blowoff".
# Reviewer-redirected trigger: kerb-strike SEVERITY repeating across laps
# at a corner (peak |log_acc_z| inside the EXISTING kerb_mask -- modules.
# stability_analysis._compute_kerb_mask_from_az, the SAME signal estimators
# already mask kerb events with), never kerb_fraction alone -- kerb
# contact itself is normal driving, not evidence on its own. Threshold
# gap-selected against both real sessions' own per-corner-instance peak-
# severity distribution (diagnostics/inspect_kerb_severity_census.py,
# thesis_notes.md): pooled p75 (2.9571 g) is the LOWEST candidate where
# firing is a clear minority of corners on BOTH sessions (Dubai 1/14, v3
# 4/17) while still firing on both -- p50 fires on ~half the corners on
# both sessions (too permissive for a standout, actionable pattern), p90
# is more selective still but p75 is already the lowest minority point.
# Repeat requirement reuses _count_repeating's own cross-lap shape (>=2 of
# a corner's own lap instances over threshold), exactly like every other
# evidence source in this file -- not a new counting convention.

def _corner_overall_window(corner, t):
    segments = corner.get("segments", {})
    starts = [s for s, e in segments.values() if e >= s]
    ends = [e for s, e in segments.values() if e >= s]
    if not starts:
        return None
    return _phase_window_indices(t, (min(starts), max(ends)))


def _kerb_axle_attribution(instances_lo_hi, t, channels, wl_cfg):
    # Per-wheel peak |travel rate| INSIDE the firing kerb windows (the
    # opposite filtering direction from modules.damper_motion.
    # classify_window_motion, which EXCLUDES kerb-masked samples to keep
    # its own platform-loading read clean -- here the kerb impact itself
    # is exactly what is being measured). Reuses modules.wheel_loads' own
    # channel-access/dead-channel/unit-normalisation helpers, the same
    # ones modules.damper_motion.py already uses for this exact channel
    # family -- not a second, independently-maintained copy.
    dead_std_max = wl_cfg["dead_channel_std_max_travel_mm"]
    peak_rate_by_wheel = {}
    for wheel in _WHEEL_CORNERS:
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        if _channel_is_dead(raw_mm, dead_std_max):
            continue
        peaks = []
        for lo, hi in instances_lo_hi:
            seg_t, seg_travel = t[lo:hi], raw_mm[lo:hi]
            valid = np.isfinite(seg_travel)
            if valid.sum() < 2:
                continue
            idx = np.where(valid)[0]
            rates = np.gradient(seg_travel[idx], seg_t[idx])
            peaks.append(float(np.max(np.abs(rates))))
        if peaks:
            peak_rate_by_wheel[wheel] = max(peaks)

    axle_score = {}
    for axle, wheels in AXLE_CORNERS.items():
        scores = [peak_rate_by_wheel[w] for w in wheels if w in peak_rate_by_wheel]
        if len(scores) == len(wheels):  # both wheels of this axle evaluable
            axle_score[axle] = max(scores)

    if len(axle_score) < 2:
        return None  # not attributable -- car-wide evidence, propose both axles
    return max(axle_score, key=axle_score.get)


def _build_kerb_blowoff_evidence(corners, state, channels, kb_cfg, wl_cfg):
    if state is None or channels is None or not corners:
        return []
    t = state["time"]
    az_g = state.get("az_g")
    kerb_mask = state.get("kerb_mask")
    if az_g is None or kerb_mask is None:
        return []

    threshold = kb_cfg["severity_threshold_g"]
    repeat_min = kb_cfg.get("repeat_min_laps", 2)

    by_corner = {}
    for c in corners:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        peaks = []  # (lo, hi, peak_g)
        for c in instances:
            w = _corner_overall_window(c, t)
            if w is None:
                continue
            lo, hi = w
            window_kerb = kerb_mask[lo:hi]
            peak = float(np.max(np.abs(az_g[lo:hi][window_kerb]))) if window_kerb.any() else 0.0
            peaks.append((lo, hi, peak))
        if not peaks:
            continue
        total = len(peaks)
        firing = [(lo, hi) for lo, hi, p in peaks if p >= threshold]
        if len(firing) < repeat_min:
            continue  # _count_repeating shape: not enough of this corner's own laps repeat the pattern

        axle = _kerb_axle_attribution(firing, t, channels, wl_cfg)
        max_severity = max(p for _, _, p in peaks if p >= threshold)
        evidence.append({
            "type": "kerb_blowoff",
            "corner": cid, "phase": None,
            "axle": axle,  # None -> not attributable, both axles proposed
            "severity": None, "confidence": round(len(firing) / total, 3),
            "peak_severity_g": round(max_severity, 3),
            "source": f"peak |log_acc_z| inside kerb_mask exceeded {threshold:.4f}g on "
                      f"{len(firing)}/{total} laps at this corner -- author-elicited 2026-09-24: "
                      f"curb strikes / hard bumps -> more blowoff. Axle attribution: "
                      f"{axle or 'not attributable (car-wide kerb evidence)'}.",
        })
    return evidence


def _kerb_blowoff_candidates(evidence, registry):
    """WP-ELICIT Phase C3 (2026-09-24): click-class, advisory, one
    exploratory step per axle (config/setup_parameters.json damper_
    blowoff_*'s own adjustment_step.exploratory, Phase B1) -- rear
    blowoff is proposable exactly like front, no policy assumption
    encoded here (config/setup_parameters.json's typical front-6/rear-0
    reference values are a registry FACT, not restated as a reason not to
    propose rear). When the evidence's own axle attribution is None (car-
    wide kerb hit, not attributable to one axle -- see
    _kerb_axle_attribution), BOTH axles are proposed, honestly labelled
    rather than guessed.
    """
    candidates = []
    for e in evidence:
        if e["type"] != "kerb_blowoff":
            continue
        attributable = e["axle"] is not None
        axles = [e["axle"]] if attributable else ["front", "rear"]
        for axle in axles:
            params = [f"damper_blowoff_{w}" for w in AXLE_CORNERS[axle]]
            step = registry[params[0]].get("adjustment_step", {}).get("exploratory", 1)
            actions = [{"parameter": p, "direction": "increase", "delta": step} for p in params]
            axle_note = (f"{axle} axle" if attributable else
                         "car-wide kerb evidence, axle not attributable -- both axles proposed")
            candidates.append({
                "id": f"kerb_blowoff_increase_{axle}:C{e['corner']}",
                "scenario": "kerb_blowoff",
                "corner": e["corner"], "phase": None,
                "lever_family": "damper_blowoff",
                "actions": actions,
                "effort_class": _effort_class_for_actions(params, registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": [e],
                "rationale": f"Repeated hard kerb contact at this corner (peak "
                             f"{e['peak_severity_g']}g, {axle_note}) -- author-elicited 2026-09-24: "
                             f"curb strikes / hard bumps call for more blowoff relief to protect the "
                             f"contact patch over the worst hits.",
            })
    return candidates


# --- Driver-feedback evidence (Deepening Phase 4d, 2026-09-18) ---------
#
# A genuinely new evidence dimension -- Stage 1/2 had none ("this frame
# has no feedback/driver-trigger axis at all yet", the Stage 1 close-out's
# own recorded open item). A driver_feedback item never generates its own
# candidate; it corroborates an EXISTING candidate's evidence_refs (same
# MIN-confidence rule every other corroborating source already uses) when
# its own corner/phase/verdict agrees, via _attach_feedback_evidence.

def _feedback_confidence(magnitude, feedback_cfg):
    """Phase D feedback round, ITEM 2(c) (2026-09-23, reviewer+user
    decision): band-shaped, not linear -- the feedback scale's own
    recorded semantics (ui/views/outing_form.py's caption: |1|=slight,
    |3|=strong, |5|=undrivable) are three qualitatively different
    regimes, not one continuous ramp. Bands are config-driven (SHAPE is
    the decision; each band's own VALUE stays tunable, provisional
    pending elicitation -- see config/decision_frame.json's own
    derived_from per band). First band whose max_abs the magnitude does
    not exceed wins; a magnitude past every band's own max_abs (should
    not occur for the current -5..+5 scale) falls to the last band's own
    value, still 1.0, never higher (confidence is capped at 1.0 project-
    wide) rather than raising an error over an out-of-range input.
    """
    bands = feedback_cfg["confidence_bands"]
    for band in bands:
        if magnitude <= band["max_abs"]:
            return band["value"]
    return bands[-1]["value"]


def _build_driver_feedback_evidence(feedback_data, aggregated, feedback_cfg):
    if not feedback_data:
        return []

    evidence = []
    for cid, corner in aggregated.items():
        fb_row = _feedback_row(feedback_data, cid)
        if not fb_row:
            continue
        for phase in PHASE_KEYS:
            key = PHASE_TO_FEEDBACK_KEY.get(phase)
            if key is None:
                continue
            raw = fb_row.get(key, 0)
            if not raw:
                continue
            magnitude = abs(raw)
            confidence = _feedback_confidence(magnitude, feedback_cfg)
            evidence.append({
                "type": "driver_feedback",
                "corner": cid, "phase": phase,
                "speed_class": corner.get("speed_class"),
                "verdict": "oversteer" if raw > 0 else "understeer",
                "severity": None, "confidence": confidence, "raw_feedback": raw,
                "source": f"driver feedback {raw:+g} at {phase} (magnitude {magnitude:g}; "
                          f"band-shaped confidence {confidence} -- Phase D ITEM 2(c), 2026-09-23, "
                          f"band values provisional pending elicitation)",
            })
    return evidence


def _attach_feedback_evidence(candidates, evidence):
    feedback_by_key = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            feedback_by_key.setdefault((e["corner"], e["phase"]), []).append(e)
    if not feedback_by_key:
        return candidates

    for c in candidates:
        own_verdicts_by_phase = {}
        for ref in c["evidence_refs"]:
            verdict = ref.get("verdict")
            if verdict is None:
                continue
            phases = ref["phases"] if "phases" in ref else (ref.get("phase"),)
            for p in phases:
                if p is not None:
                    own_verdicts_by_phase.setdefault(p, set()).add(verdict)
        own_ids = {id(r) for r in c["evidence_refs"]}
        for phase, verdicts in own_verdicts_by_phase.items():
            for fb in feedback_by_key.get((c["corner"], phase), []):
                if fb["verdict"] in verdicts and id(fb) not in own_ids:
                    c["evidence_refs"].append(fb)
                    own_ids.add(id(fb))
                    # DECISION LAYER SPEC B2 (2026-09-22): data agreeing with
                    # driver feedback is the strongest trigger class (Stage
                    # 2). Never touches a feedback-only candidate's own
                    # provenance -- it has no data verdict to agree with.
                    if c.get("trigger_provenance") == TRIGGER_DATA_ONLY:
                        c["trigger_provenance"] = TRIGGER_BOTH_AGREEING
    return candidates


# DECISION LAYER SPEC B6 (2026-09-22): driver-vs-data disagreement. Only
# oversteer/understeer have a natural opposite -- unstable_yaw has none,
# so a candidate whose own verdict is unstable_yaw never gets a
# conflicting_feedback entry from this function (no invented opposite).
_OPPOSITE_VERDICT = {"oversteer": "understeer", "understeer": "oversteer"}


def _attach_conflicting_feedback(candidates, evidence):
    """Stage 5: 'driver contradicts data: NEVER suppresses -- both shown
    side by side; the system supervises the driver too; engineer
    arbitrates.' Attaches disagreeing driver_feedback items to a
    candidate's own `conflicting_feedback` list -- never touches status,
    confidence, or evidence_refs (agreement has its own path above; this
    is strictly the opposite-verdict case, so a single feedback item can
    never land in both lists for the same candidate)."""
    feedback_by_key = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            feedback_by_key.setdefault((e["corner"], e["phase"]), []).append(e)

    for c in candidates:
        if not feedback_by_key:
            c["conflicting_feedback"] = []
            continue
        own_verdicts_by_phase = {}
        for ref in c["evidence_refs"]:
            verdict = ref.get("verdict")
            if verdict is None:
                continue
            phases = ref["phases"] if "phases" in ref else (ref.get("phase"),)
            for p in phases:
                if p is not None:
                    own_verdicts_by_phase.setdefault(p, set()).add(verdict)
        conflicting = []
        seen_ids = set()
        for phase, verdicts in own_verdicts_by_phase.items():
            opposites = {_OPPOSITE_VERDICT[v] for v in verdicts if v in _OPPOSITE_VERDICT}
            if not opposites:
                continue
            for fb in feedback_by_key.get((c["corner"], phase), []):
                if fb["verdict"] in opposites and id(fb) not in seen_ids:
                    conflicting.append(fb)
                    seen_ids.add(id(fb))
        c["conflicting_feedback"] = conflicting
    return candidates


def build_evidence(summaries, ls_stats, config, classify_fn, corners=None, state=None, channels=None,
                    feedback_data=None):
    """Evidence layer, Stage 1. Turns per-lap-per-corner stability
    summaries (modules.stability_analysis.summarise_corners' own output
    shape) into a flat list of evidence items: {type, corner, phase,
    verdict, severity, confidence, source, ...}. Confidence is always a
    plain fraction of real counts (laps repeating x signal validity) --
    never an invented normalising constant.

    Sources, per the Stage 1 work order: (a) corner_verdict, via the
    caller's own classify_fn -- the identical worst-lap aggregate and
    anchored thresholds the stability grid uses, so this evidence can
    never disagree with what the UI shows for the same corner/phase; (b)
    ls_disambiguation, traction-limited vs cornering-limited for exit-phase
    oversteer evidence, only where LS_ratio is valid (see
    _build_ls_disambiguation_evidence for the method and its Stage 2
    caveat); (c) plausibility_brake_balance, front axle beyond its own
    limit while rear stays healthy during braking. plausibility_tyre_
    pressure is NOT evaluated in Stage 1: config/decision_frame.json's
    tyre_pressure_window is null (no target window exists anywhere in this
    project, verified by search) and no measured-pressure channel is wired
    into this function's inputs -- both are open items, not silent
    no-signal evidence.

    `summaries` is the caller's raw per-lap-per-corner list (same shape
    modules.recommendation.generate_recommendations consumes). `ls_stats`
    is this module's own aggregate_ls_by_corner(summaries) output, accepted
    as a parameter rather than computed internally so a caller holding it
    across repeated calls never rebuilds it. `config` is load_decision_
    frame_config()'s own dict. `classify_fn` is the caller's corner
    classifier (in the UI thread, self._classify_corner) -- not part of
    the work order's literally-stated 3-argument signature, but required
    to satisfy source (a)'s own text ("via the existing classify path");
    added here rather than reimplementing classify logic (which would risk
    silently diverging from the UI) or importing ui/ from modules/ (which
    would violate CLAUDE.md's "no PyQt6 imports in modules/" in spirit, via
    outing_form.py's own Qt imports) -- same parameter modules.
    recommendation.generate_recommendations already takes for the same
    reason.

    Stage 2 additions (Frame-Stage-2 Phase 3, 2026-09-04): (d) matrix_
    verdict, the same classify_fn/_phase_verdict call the OLD 39-rule
    engine's own "data"-trigger rules use, generalised to the small set of
    multi-phase groups the matrix actually uses -- feeds
    _bridge_candidates_for_matrix_rules, the migrated-rule candidate
    bridge. (e)/(f) intervention_abs/intervention_abs_masking/
    intervention_tc, config-gated PER SOURCE (decision_frame.json
    intervention_evidence.abs.enabled / .tc.enabled -- Deepening Phase 4c,
    2026-09-18, replaced Stage 2's own single global use_intervention_
    evidence flag; ABS defaults on, TC stays dormant) -- corners/state/
    channels are optional (default None, Stage-1-caller-compatible); when
    any is missing, or a source's own flag is off, that source is silently
    skipped (an honest [], never a fabricated fallback), since the per-lap
    phase-window channel check they need cannot be computed from summaries
    alone.

    Deepening Phase 4d (2026-09-18): `feedback_data` (the outing's own
    driver-feedback table, same shape modules.recommendation.
    generate_recommendations' own feedback_data parameter takes) is
    optional, default None -- when supplied, adds driver_feedback evidence
    items (see _build_driver_feedback_evidence); generate_candidates then
    attaches matching items to existing candidates' evidence_refs.

    LS threshold decision (2026-09-20): (g) ls_threshold, config-gated
    (decision_frame.json ls_threshold_evidence.enabled), an ABSOLUTE-
    threshold evidence source independent of (b) -- see _build_ls_
    threshold_evidence for the phase-conditioned confidence treatment
    (braking/turn-in uncapped, exit-phase capped via the existing MIN-
    confidence mechanism). NOT a classification verdict tier -- config/
    parameters.json's classification block is untouched by this source.

    FRAME DEPTH PROGRAMME Step 2 (2026-09-22): (h) damper_motion, config-
    gated (decision_frame.json damper_motion.enabled), same corners-not-
    None gate as (e)/(f) above -- per-corner, per-TRANSIENT-phase (entry/
    exit only, never apex) loading/unloading classification from log_
    susp_travel_*, modules.damper_motion.build_damper_motion_evidence. No
    candidate-generation consumer reads this evidence type yet (Step 1's
    evaluate_conditions can, once a lever_bridges entry's own "conditions"
    list names it -- that wiring is Step 3's own job, not this one's).
    """
    aggregated = aggregate_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)
    config_recs = load_recommendations_config()

    evidence = []
    evidence += _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn, config)
    evidence += _build_ls_disambiguation_evidence(
        aggregated, ls_stats, [e for e in evidence if e["type"] == "corner_verdict"]
    )
    evidence += _build_brake_balance_evidence(aggregated, by_corner_laps, config)
    evidence += _build_matrix_verdict_evidence(aggregated, by_corner_laps, classify_fn, config_recs, config)
    evidence += _build_ls_threshold_evidence(ls_stats, by_corner_laps, config)

    # Deepening Phase 4c (2026-09-18, user decision): ABS/TC now gate
    # independently -- ABS defaults ON (Phase 3 resolved the trusted
    # position channel, treated as reviewed-enough to ship); TC stays
    # dormant pending its own channel-identity mapping. Replaces Stage 2's
    # single global use_intervention_evidence flag.
    intervention_cfg = config.get("intervention_evidence", {})
    if corners is not None:
        abs_cfg = intervention_cfg.get("abs", {})
        if abs_cfg.get("enabled", False):
            evidence += _build_intervention_abs_evidence(corners, by_corner_laps, state, channels, aggregated, abs_cfg)
        tc_cfg = intervention_cfg.get("tc", {})
        if tc_cfg.get("enabled", False):
            evidence += _build_intervention_tc_evidence(corners, state, channels, aggregated)

        # FRAME DEPTH PROGRAMME Step 2 (2026-09-22): damper motion-state
        # evidence, same corners-is-not-None gate as ABS/TC above (needs
        # per-lap phase-window segments summaries alone do not carry).
        # Lazy import to break the circular dependency -- modules/damper_
        # motion.py itself imports TRANSIENT_PHASES/_phase_window_indices
        # from this module, so a module-level import here would deadlock
        # at import time; by the time build_evidence is actually CALLED
        # both modules are already fully loaded, same resolution this
        # project's own test helpers already use for a similar late-bound
        # dependency (tests/test_decision_frame.py's classify_fn).
        dm_cfg = config.get("damper_motion", {})
        if dm_cfg.get("enabled", False):
            from modules.damper_motion import build_damper_motion_evidence
            wl_cfg = load_parameters()["wheel_loads"]
            dm_evidence, _dm_summary = build_damper_motion_evidence(
                corners, state, channels, aggregated, dm_cfg, wl_cfg)
            evidence += dm_evidence

        # WP-ELICIT Phase C3 (2026-09-24): kerb-strike-severity blowoff
        # evidence, same corners-not-None gate as ABS/TC/damper-motion
        # above (needs per-instance overall time windows summaries alone
        # do not carry).
        kb_cfg = config.get("kerb_blowoff_evidence", {})
        if kb_cfg.get("enabled", False):
            wl_cfg = load_parameters()["wheel_loads"]
            evidence += _build_kerb_blowoff_evidence(corners, state, channels, kb_cfg, wl_cfg)

    if feedback_data:
        evidence += _build_driver_feedback_evidence(
            feedback_data, aggregated, config.get("driver_feedback_weighting", {}))

    return evidence


# --- Candidate layer, Stage 1 (exit-oversteer scenario) ----------------
#
# Scope, per the Stage 1 work order: the exit-oversteer scenario ONLY, both LS
# branches (cornering-limited -> ARB/spring family; traction-limited ->
# diff/TC family), plus the brake-balance plausibility candidate wherever
# its evidence fires. Every candidate carries the provenance grade of the
# bridge it came from: 'derived-from-matrix' when a real config/
# recommendations.json cell_id backs this exact parameter+direction+
# scenario, 'proposed' otherwise -- advisory-capped downstream (Phase 4),
# same policy as modules.recommendation._match_is_recommended's own
# provenance cap.

EXIT_PHASES = ("exit_4", "exit_5")

# FRAME DEPTH PROGRAMME Step 1: entry/exit families act as transients for
# phase_transient conditions (Segers ch.11 boundary condition, docs/
# segers_bridge_review.md C11-1 -- dampers only develop force while the
# shaft has velocity, never at steady-state apex cornering). Derived from
# PHASE_KEYS itself (not a second, hand-typed list) so this can never
# silently disagree if PHASE_KEYS ever changes.
TRANSIENT_PHASES = tuple(p for p in PHASE_KEYS if p != "apex_3")

# Enum structure (a fixed ordering of setup_parameters.json's own
# change_effort vocabulary), not a per-car tunable -- CLAUDE.md
# method-defining-constant guidance, same status as modules.recommendation.
# SEVERITY_RANK/ESCALATION_TIER_RANK. DECISION LAYER SPEC 2026-09-22 adds
# half_hour (camber's 20-30 min class) between minutes and garage_hours.
EFFORT_RANK = {"seconds": 0, "minutes": 1, "half_hour": 2, "garage_hours": 3}

# DECISION LAYER SPEC design principle (2026-09-22): every reachable lever
# always resolves to exactly one status. Granularity is PER-CANDIDATE
# (reviewer-confirmed 2026-09-22, thesis_notes.md "B1 design resolution") --
# a real candidate stays corner+phase-specific exactly as today, never
# merged across corners; STATUS_NO_TRIGGER is the one exception, a
# synthetic placeholder for a lever with zero real candidates anywhere
# this session (see generate_lever_inventory).
STATUS_PROPOSED = "proposed"
STATUS_NO_TRIGGER = "no_trigger"
STATUS_BLOCKED_AT_EDGE = "blocked_at_edge"
STATUS_CONTRADICTED = "contradicted"
STATUS_NOT_ASSESSABLE = "not_assessable"
LEVER_STATUSES = (STATUS_PROPOSED, STATUS_NO_TRIGGER, STATUS_BLOCKED_AT_EDGE,
                   STATUS_CONTRADICTED, STATUS_NOT_ASSESSABLE)

# Provenance grades that the existing 39-rule engine already treats as
# action-eligible (config/recommendations.json settings.action_class.
# action_eligible_provenances) -- reused here as the derived-from-matrix
# cutoff so this module's grading can never disagree with that policy.
_MATRIX_ELIGIBLE_PROVENANCES = frozenset({"engineer-verbatim", "project-lead-reviewed"})


def _effort_class_for_actions(param_keys, registry):
    efforts = [registry[p]["change_effort"] for p in param_keys
               if p in registry and registry[p].get("change_effort")]
    if not efforts:
        return None
    return max(efforts, key=lambda e: EFFORT_RANK.get(e, 0))


def _matrix_cell(config_recs, cell_id):
    if cell_id is None:
        return None
    for rule in config_recs["rules"]:
        if rule.get("cell_id") == cell_id:
            return rule
    return None


def _grade_for_provenance(provenance):
    return "derived-from-matrix" if provenance in _MATRIX_ELIGIBLE_PROVENANCES else "proposed"


def _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs,
                                intervention_tc_by_corner=None):
    intervention_tc_by_corner = intervention_tc_by_corner or {}
    candidates = []
    for (cid, phase), items in corner_verdicts_by_key.items():
        if phase not in EXIT_PHASES:
            continue
        cv = next((e for e in items if e["verdict"] == "oversteer"), None)
        if cv is None:
            continue
        speed_class = cv.get("speed_class")
        ls_evidence = ls_by_key.get((cid, phase))
        ls_class = ls_evidence["ls_class"] if ls_evidence else None
        evidence_refs = [cv] + ([ls_evidence] if ls_evidence else [])
        # Stage 2 Phase 3c: TC-activity intervention evidence corroborates
        # the TRACTION-limited branch specifically (config-gated, empty
        # dict when off/unavailable -- see build_evidence) -- never added
        # to the cornering-limited/ARB branch, a different mechanism.
        tc_evidence_refs = evidence_refs + (
            [intervention_tc_by_corner[cid]] if cid in intervention_tc_by_corner else []
        )

        if ls_class in (None, "cornering_limited"):
            # ARB/spring family. Primary lever: rear ARB soften -- matrix-
            # exact only when this corner's speed_class matches OS-EXIT-
            # med (the only OS-EXIT cell whose own action is rear ARB);
            # otherwise the same lever, reasoned-generalised (proposed),
            # since the LS-based routing this candidate comes from has no
            # speed-class axis of its own.
            cell = _matrix_cell(config_recs, "OS-EXIT-med") if speed_class == "medium" else None
            arb_params = ["arb_rl", "arb_rr"]
            candidates.append({
                "id": f"arb_soften:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "arb_spring",
                "actions": [{"parameter": p, "direction": "soften", "delta": -1} for p in arb_params],
                "effort_class": _effort_class_for_actions(arb_params, registry),
                "effect_class": "primary",
                "grade": _grade_for_provenance(cell["elicitation_provenance"]) if cell else "proposed",
                "cell_id": "OS-EXIT-med" if cell else None,
                "evidence_refs": evidence_refs,
                "rationale": (cell["rationale"] if cell else
                              "Cornering-limited (LS disambiguation): softening the rear ARB frees up "
                              "rear grip under lateral load, addressing exit oversteer at the "
                              "roll-stiffness-distribution level. Matrix-exact only at medium-speed "
                              "exit (cell OS-EXIT-med); generalised here across speed classes."),
            })
            candidates.append({
                "id": f"springs_rear_soften:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "arb_spring",
                "actions": [{"parameter": "springs_rear", "direction": "soften", "delta": -1}],
                "effort_class": _effort_class_for_actions(["springs_rear"], registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": evidence_refs,
                "rationale": "Same rear-grip goal as the ARB candidate above, at the garage-effort "
                             "tier: softer rear springs reduce rear vertical stiffness generally "
                             "(registry mechanism: sets rear-axle ride stiffness independent of roll "
                             "stiffness). Not itself a matrix cell for this scenario -- proposed "
                             "grade, advisory-capped, a heavier-effort alternative to the ARB lever, "
                             "not a substitute recommendation on equal footing. Companion note "
                             "(author-elicited 2026-09-24, WP-ELICIT Phase B1): softer rear springs "
                             "and stiffer rear dampers are compensatory, same axle -- if the springs "
                             "move, the rear dampers may need a matching stiffen to hold the platform "
                             "where the springs alone would let it move. Informational only, not a "
                             "competing candidate; no separate damper action is generated from this "
                             "note.",
            })

        if ls_class in (None, "traction_limited"):
            # Diff/TC family. TC LON is matrix-exact only at low-speed
            # exit (OS-EXIT-low); diff_position has no matrix cell for
            # EXIT oversteer anywhere (only turn-in/apex) -- the
            # traction-limited -> diff bridge is PROPOSED grade per the
            # work order, advisory-capped regardless of speed class.
            cell = _matrix_cell(config_recs, "OS-EXIT-low") if speed_class == "low" else None
            candidates.append({
                "id": f"tc_lon_increase:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "diff_tc",
                "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
                "effort_class": _effort_class_for_actions(["tc_lon"], registry),
                "effect_class": "primary",
                "grade": _grade_for_provenance(cell["elicitation_provenance"]) if cell else "proposed",
                "cell_id": "OS-EXIT-low" if cell else None,
                "evidence_refs": tc_evidence_refs,
                "rationale": (cell["rationale"] if cell else
                              "Traction-limited (LS disambiguation): raising TC LON cuts wheel spin "
                              "under power, addressing power-on rear-axle oversteer directly. "
                              "Matrix-exact only at low-speed exit (cell OS-EXIT-low); generalised "
                              "here across speed classes for the same LS-routing reason as the ARB "
                              "candidate above."),
            })
            candidates.append({
                "id": f"diff_position_increase:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "diff_tc",
                "actions": [{"parameter": "diff_position", "direction": "increase", "delta": 1}],
                "effort_class": _effort_class_for_actions(["diff_position"], registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": tc_evidence_refs,
                "rationale": "More locking torque resists axle-speed difference under power, "
                             "stabilising the rear on corner exit -- the same mechanism the matrix "
                             "already uses for turn-in/apex oversteer (cells OS-TIN-med, OS-APX-med), "
                             "generalised here to exit. No matrix cell exists for diff_position at "
                             "exit specifically -- PROPOSED grade, advisory-capped, per the work "
                             "order's own instruction.",
            })
    return candidates


def _brake_balance_candidates(evidence, registry, config_recs):
    # Plausibility-check candidate: reuses the EXISTING US-BRK-{speed_class}
    # matrix cell's own suggestion/rationale directly from config/
    # recommendations.json (never duplicated as a second copy of the same
    # numbers) -- brake_balance_signature is an additional, cheaper
    # detection pathway for the same front-axle-under-braking phenomenon
    # those cells already address, not a new lever.
    speed_class_to_cell = {"low": "US-BRK-low", "medium": "US-BRK-med", "high": "US-BRK-high"}
    candidates = []
    for e in evidence:
        if e["type"] != "plausibility_brake_balance":
            continue
        cell_id = speed_class_to_cell.get(e.get("speed_class"))
        cell = _matrix_cell(config_recs, cell_id)
        if cell is None:
            candidates.append({
                "id": f"brake_balance_unrouted:C{e['corner']}:{e['phase']}",
                "scenario": "plausibility_brake_balance",
                "corner": e["corner"], "phase": e["phase"],
                "lever_family": None, "actions": [],
                "effort_class": None, "effect_class": None,
                "grade": "proposed", "cell_id": None,
                "evidence_refs": [e],
                "rationale": f"Front-beyond/rear-healthy brake-balance signature fired at "
                             f"C{e['corner']} {e['phase']}, but speed_class "
                             f"({e.get('speed_class')!r}) does not map to a known US-BRK-* cell -- "
                             f"no candidate action generated, engineer attention needed.",
            })
            continue
        actions = cell["suggestion"] if isinstance(cell["suggestion"], list) else [cell["suggestion"]]
        param_keys = [a["parameter"] for a in actions]
        candidates.append({
            "id": f"brake_balance:{cell_id}:C{e['corner']}:{e['phase']}",
            "scenario": "plausibility_brake_balance",
            "corner": e["corner"], "phase": e["phase"],
            "lever_family": "brake_balance",
            "actions": actions,
            "effort_class": _effort_class_for_actions(param_keys, registry),
            "effect_class": "primary",
            "grade": _grade_for_provenance(cell["elicitation_provenance"]),
            "cell_id": cell_id,
            "evidence_refs": [e],
            "rationale": cell["rationale"] + " (Reached here via the brake_balance_signature "
                         "plausibility check, not the full CS-verdict classify path -- same "
                         f"underlying matrix cell and lever as the existing {cell_id} rule in "
                         "config/recommendations.json.)",
        })
    return candidates


# --- DECISION LAYER SPEC B7: brake_bias bridge (2026-09-22) -------------
#
# Encoding target reviewed and fixed by docs/segers_bridge_review.md
# C5-1 (ch.5 p.107, Eq.5.3): too much FRONT bias uses up front-tyre grip
# capacity under straight-line braking, unavailable for turn-in cornering
# force -> understeer (corner-entry, continuing mid-corner) -> the fix is
# LESS front bias, i.e. more_rear. Too much REAR bias risks the rear
# stepping out under trail-braking -> oversteer (corner-entry only, the
# book's own "not mid-corner" scoping) -> the fix is more_front. Direction
# convention reviewer-confirmed 2026-09-22 (thesis_notes.md "B7 brake_bias
# direction convention"): bias moves AWAY from the limiting axle.
# "Forward"/"rearward" are output WORDS only -- the channel's own sign
# and numeric scale stay elicitation item 4 (config/setup_parameters.json
# brake_bias notes); this bridge never emits a numeric channel delta, only
# a direction word plus a severity-scaled click count.
#
# Magnitude reuses SEVERITY_RANK directly (moderate=1, strong=2) rather
# than inventing a third value for "3 clicks max" -- the spec's own
# ceiling is a cap, not a target every firing must reach.

def _brake_bias_candidates(evidence):
    direction_by_verdict = {
        "understeer": ("more_rear", [("entry_1_brake",), ("entry_2_turnin",)]),
        "oversteer": ("more_front", [("entry_1_brake",)]),
    }
    candidates = []
    for e in evidence:
        if e["type"] != "matrix_verdict":
            continue
        spec = direction_by_verdict.get(e["verdict"])
        if spec is None:
            continue
        direction, allowed_phase_groups = spec
        if e["phases"] not in allowed_phase_groups:
            continue
        if SEVERITY_RANK[e["severity"]] < SEVERITY_RANK["moderate"]:
            continue
        clicks = min(3, SEVERITY_RANK[e["severity"]])
        if direction == "more_rear":
            mechanism = ("Too much front bias uses up front-tyre grip capacity under "
                         "straight-line braking, unavailable for turn-in cornering force "
                         "(understeer) -- move REARWARD to free up front grip.")
        else:
            mechanism = ("Too much rear bias risks the rear stepping out under "
                         "trail-braking (oversteer) -- move FORWARD to stabilise the rear.")
        candidates.append({
            "id": f"brake_bias:{direction}:C{e['corner']}:{'+'.join(e['phases'])}",
            "scenario": f"brake_bias:{direction}",
            "corner": e["corner"], "phase": e["phases"][-1], "phases": e["phases"],
            "lever_family": "brake_bias",
            "actions": [{"parameter": "brake_bias", "direction": direction, "delta": clicks}],
            "effort_class": "seconds",
            "effect_class": "secondary",
            "grade": "proposed",
            "cell_id": None,
            "evidence_refs": [e],
            "derived_from": "Segers ch.5 p.107 Eq.5.3 (docs/segers_bridge_review.md C5-1), "
                             "reviewer-confirmed direction convention 2026-09-22",
            "rationale": mechanism + f" Severity-scaled {clicks} click(s) (max 3). Direction is "
                         "an output word only -- the channel's own sign/scale and current-state "
                         "window are elicitation item 4, not resolved here.",
        })
    return candidates


# --- Stage 2: generic 39-rule migration bridge --------------------------
#
# Every rule in config/recommendations.json re-expressed as a candidate
# bridge: scenario evidence (matrix_verdict, same classify_fn call the old
# engine's own "data" trigger uses) -> lever family (the rule's own
# suggestion), effort/effect class from the registry, provenance grade
# carried over via _grade_for_provenance (identical policy to Stage 1's
# own exit-oversteer bridge -- never a second, disagreeing grading rule).
#
# Rule status -> bridge behaviour (mirrors modules.recommendation's own
# firing rules exactly, never a second policy):
#   elicited/reviewed -- a real "primary" candidate whenever matching
#     matrix_verdict evidence exists (same verdict/min_severity/speed_class
#     gate _evaluate_rule's "data" trigger already applies).
#   held (escalation) -- NEVER a primary candidate on its own (status is in
#     _NON_FIRING_STATUSES, same exclusion set the old engine already
#     uses) -- instead, whenever its OWN base cell's candidate fires, the
#     held rule's action is ALSO emitted as a "secondary" candidate on the
#     identical evidence, mirroring the old engine's unconditional
#     escalation_notes attachment (_add_rule_matches_to_buckets) exactly,
#     but as a real scored candidate instead of a display-only string.
#     Forced to grade="proposed" regardless of its own elicitation_
#     provenance -- "held" itself means "not yet automated, no applied-
#     recommendations history", a weaker evidentiary status than the
#     wording of its rationale alone implies; promoting it to
#     'derived-from-matrix' would overstate what the old engine ever
#     claimed for it.
#   dropped/retired -- documented-inactive: accounted for (see
#     rule_bridge_status below, migration-completeness test) but never
#     produce a candidate, exactly matching the old engine's own
#     _NON_FIRING_STATUSES.
#   trigger != "data" -- out of Stage 2's own scope (the matrix's 32 cells
#     are all "data"-trigger; the 7 pre-matrix seed rules using "driver"/
#     "both" are already status=retired, so this branch is presently dead
#     code, kept defensively should a future non-retired non-data rule
#     ever be added).

def rule_bridge_status(rule):
    """One of 'primary' | 'secondary(held)' | 'inactive(dropped)' |
    'inactive(retired)' | 'inactive(other-status)' | 'non-matrix(trigger)'
    -- used both by the bridge itself and by the migration-completeness
    test (Phase 3f: 'all 39 accounted')."""
    status = rule.get("status")
    if status == "retired":
        return "inactive(retired)"
    if status == "dropped":
        return "inactive(dropped)"
    if status == "held":
        return "secondary(held)"
    if rule["condition"].get("trigger") != "data" or rule.get("suggestion") is None:
        return "non-matrix(trigger)"
    if status in _NON_FIRING_STATUSES:
        return "inactive(other-status)"
    return "primary"


def _bridge_candidates_for_matrix_rules(evidence, registry, config_recs, intervention_abs_by_corner=None,
                                         intervention_abs_masking_by_corner=None):
    intervention_abs_by_corner = intervention_abs_by_corner or {}
    intervention_abs_masking_by_corner = intervention_abs_masking_by_corner or {}
    matrix_by_group = {}
    for e in evidence:
        if e["type"] == "matrix_verdict":
            matrix_by_group.setdefault(e["phases"], {}).setdefault(e["corner"], []).append(e)

    held_by_base = {r["escalation_of"]: r for r in config_recs["rules"]
                     if r.get("status") == "held" and r.get("escalation_of")}

    def _make_candidate(rule, cid, evidence_refs, effect_class, grade, phase_group):
        actions = rule["suggestion"] if isinstance(rule["suggestion"], list) else [rule["suggestion"]]
        param_keys = [a["parameter"] for a in actions]
        return {
            "id": f"{rule['id']}:C{cid}:{'+'.join(phase_group)}",
            "scenario": rule.get("cell_id") or rule["id"],
            "corner": cid, "phase": phase_group[-1], "phases": phase_group,
            "lever_family": f"matrix:{rule.get('cell_id') or rule['id']}",
            "actions": actions,
            "effort_class": _effort_class_for_actions(param_keys, registry),
            "effect_class": effect_class,
            "grade": grade,
            "cell_id": rule.get("cell_id"),
            "evidence_refs": evidence_refs,
            "rationale": rule["rationale"],
            "practice_note": rule.get("practice_note"),
            "rule_id": rule["id"], "rule_status": rule.get("status"),
        }

    candidates = []
    for rule in config_recs["rules"]:
        if rule_bridge_status(rule) != "primary":
            continue
        condition = rule["condition"]
        phase_group = tuple(rule["phases"])
        verdict = condition["verdict"]
        min_sev = condition.get("min_severity", "normal")
        req_speed_class = condition.get("speed_class")

        for cid, items in matrix_by_group.get(phase_group, {}).items():
            for ev in items:
                if ev["verdict"] != verdict:
                    continue
                if SEVERITY_RANK[ev["severity"]] < SEVERITY_RANK[min_sev]:
                    continue
                if req_speed_class is not None and ev.get("speed_class") != req_speed_class:
                    continue

                evidence_refs = [ev]
                if verdict == "unstable_yaw" and phase_group == ("entry_1_brake",) and cid in intervention_abs_by_corner:
                    evidence_refs = evidence_refs + [intervention_abs_by_corner[cid]]
                # Deepening Phase 4c: "ABS regulating heavily -> flag as
                # masking" applies to ANY braking-phase verdict (under-
                # steer/oversteer/unstable_yaw alike), not just the ABS-
                # inactive corroboration above -- a heavily-regulated
                # braking zone's own CS/matrix reading is suspect
                # regardless of which axle/direction it points. Appended
                # to evidence_refs so the MIN-confidence rule pulls this
                # candidate's own confidence down toward the masking
                # evidence's -- a real, if partial, realisation of
                # "prefer brake-balance/platform levers": this candidate
                # scores lower, so a brake_balance_signature candidate at
                # the same corner (unaffected by this flag, a different
                # evidence source) naturally outranks it if one exists.
                # No hard reordering/exclusion rule implemented -- flagged
                # as a partial realisation, not the full "prefer" semantics.
                if phase_group == ("entry_1_brake",) and cid in intervention_abs_masking_by_corner:
                    evidence_refs = evidence_refs + [intervention_abs_masking_by_corner[cid]]

                candidates.append(_make_candidate(
                    rule, cid, evidence_refs, "primary",
                    _grade_for_provenance(rule.get("elicitation_provenance")), phase_group,
                ))

                held = held_by_base.get(rule.get("cell_id"))
                if held is not None and held.get("suggestion") is not None:
                    candidates.append(_make_candidate(
                        held, cid, evidence_refs, "secondary", "proposed", phase_group,
                    ))
    return candidates


# --- FRAME DEPTH PROGRAMME Step 1: per-bridge condition evaluator ------
#
# PLAN.md "FRAME DEPTH PROGRAMME", reviewer-approved fixed design (2026-09-
# 22). See config/decision_frame.json's own "conditions"/"_comment_
# conditions" keys for the config-side schema and rationale (in particular
# why no cross-lap repeat count is a condition type here -- the 2026-09-22
# candidate census found every corner_verdict/matrix_verdict evidence item
# on both real sessions rests on exactly 1 repeating lap, so a repeat-count
# floor would suppress 100% of candidates; repeatability stays the
# existing per-evidence-item confidence discount's own job).

def _eval_evidence_corroboration(cond, corner, phase, evidence_items):
    """PASS/FAIL/not-evaluable for one evidence_corroboration condition.
    Scope is the SAME corner AND phase as the candidate being generated,
    per the work order. Distinguishes "this evidence source was never even
    built this run" (config-gated off, e.g. TC by default; or a source
    whose own required inputs -- corners/state/channels -- were not
    supplied) from "this evidence source ran but has nothing at this
    corner/phase" -- only the former is not-evaluable; the latter is a
    real, evaluable answer (a genuine absence), matching build_evidence's
    own "silently skipped... an honest [], never a fabricated fallback"
    posture for the source-level case, extended here to the per-condition
    case.
    """
    evidence_type = cond["evidence_type"]
    presence = cond["presence"]
    if not any(e["type"] == evidence_type for e in evidence_items):
        return "not_evaluable", f"no {evidence_type} evidence available this run"

    def _matches(e):
        if e["type"] != evidence_type or e.get("corner") != corner:
            return False
        return phase == e.get("phase") or phase in (e.get("phases") or ())

    found = any(_matches(e) for e in evidence_items)
    if presence == "present":
        return (True, None) if found else (False, f"no {evidence_type} evidence at C{corner} {phase}")
    return (True, None) if not found else (False, f"{evidence_type} evidence present at C{corner} {phase} (expected absent)")


def _eval_setup_state(cond, setup_data, registry):
    """PASS/FAIL/not-evaluable for one setup_state condition. Nominal/span
    come from load_decision_frame_config()'s own parameter_windows -- the
    SAME already-derived, human-reviewed window _settings_window_component
    (Phase 4 scoring) already uses for this exact purpose, not re-derived
    a second time from setup_parameters.json's own heterogeneous typical_
    window/value_space fields (which use different field shapes per
    parameter -- min/max, base, baseline, enum options -- re-deriving here
    would duplicate that conversion logic a second time with real drift
    risk). `registry` (this function's own fixed-signature parameter) is
    used exactly as Phase 4 scoring already uses it: to resolve maps_to for
    _current_setup_value, nothing else. Loading decision-frame config fresh
    here mirrors score()'s own existing precedent in this file (score()
    takes `config` as a parameter and loads `registry` fresh internally --
    the mirror image of that same asymmetry, not a new pattern)."""
    param = cond["parameter"]
    check = cond["check"]
    entry = registry.get(param)
    if entry is None:
        return "not_evaluable", f"no registry entry: {param}"

    window = load_decision_frame_config()["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or span is None or not span:
        return "not_evaluable", f"no settings window: {param}"

    current = _current_setup_value(setup_data, entry)
    if current is None:
        return "not_evaluable", f"setup sheet unfilled: {param}"
    try:
        current = float(current)
    except (TypeError, ValueError):
        return "not_evaluable", f"setup sheet value non-numeric (likely an enum label): {param}"

    if check == "within_window":
        ok = abs(current - nominal) <= span
        return (True, None) if ok else (False, f"{param} outside window (current={current}, nominal={nominal}, span={span})")
    if check == "at_window_edge":
        ok = abs(current - nominal) >= span
        return (True, None) if ok else (False, f"{param} not at window edge (current={current}, nominal={nominal}, span={span})")
    if check == "moved_from_nominal":
        direction = cond.get("direction")
        if direction == "increase":
            ok = current > nominal
        elif direction == "decrease":
            ok = current < nominal
        else:
            return "not_evaluable", f"moved_from_nominal requires a direction: {param}"
        return (True, None) if ok else (False, f"{param} not moved {direction} from nominal (current={current}, nominal={nominal})")
    return "not_evaluable", f"unknown setup_state check: {check!r}"


def _eval_phase_transient(phase):
    ok = phase in TRANSIENT_PHASES
    return (True, None) if ok else (False, f"{phase} is not a transient phase (apex/steady-state)")


def evaluate_conditions(conditions, corner, phase, evidence_items, setup_data, registry,
                         contradiction_sources=None):
    """Pure function. Returns (verdict, reasons) where verdict is one of
    "PASS" / "SUPPRESS" / "CONTRADICTED" / "CAP_ADVISORY", exactly:
      - every condition evaluable and passing -> PASS, confidence untouched;
      - any REQUIRED evidence_corroboration condition evaluable, FAILING,
        and whose own evidence_type is in `contradiction_sources` ->
        CONTRADICTED (DECISION LAYER SPEC B6, 2026-09-22: "data contradicts
        data" -- the evidence type exists and was actually checked, it
        just disagrees here; a real, evidenced conflict, not an absence).
        Still checked first per condition, short-circuits;
      - any OTHER required condition evaluable and FAILING -> SUPPRESS
        (phase_transient, setup_state, or an evidence_corroboration type
        not listed in contradiction_sources -- structural inapplicability
        or an untracked corroboration gap, not a data disagreement; a
        candidate this frame can affirmatively rule out is not emitted
        at all);
      - otherwise, if any non-required condition failed OR any condition
        (required or not) was not-evaluable -> CAP_ADVISORY, reasons is
        every non-passing condition's own human-readable explanation ("no
        damper_motion evidence available", "setup sheet unfilled:
        toe_front"). An evidence GAP caps, it never suppresses -- "cannot
        corroborate" is not "contradicted" (config/decision_frame.json's
        own "conditions" comment states this as the load-bearing rule).
    No conditions (absent/empty list) -> PASS, [] -- byte-identical to
    today's unconditional behaviour. `contradiction_sources` (additive,
    default None == treated as empty) is config/decision_frame.json's own
    conditions.contradiction_sources list -- omitting it reproduces the
    pre-B6 SUPPRESS-only behaviour exactly, for every direct caller/test
    that doesn't pass it.
    """
    if not conditions:
        return "PASS", []
    contradiction_sources = contradiction_sources or ()

    reasons = []
    degraded = False
    for cond in conditions:
        ctype = cond["type"]
        required = cond.get("required", False)

        if ctype == "evidence_corroboration":
            result, reason = _eval_evidence_corroboration(cond, corner, phase, evidence_items)
        elif ctype == "setup_state":
            result, reason = _eval_setup_state(cond, setup_data, registry)
        elif ctype == "phase_transient":
            result, reason = _eval_phase_transient(phase)
        else:
            result, reason = "not_evaluable", f"unknown condition type: {ctype!r}"

        if result == "not_evaluable":
            degraded = True
            reasons.append(reason)
            continue
        if result is False:
            if required:
                if ctype == "evidence_corroboration" and cond.get("evidence_type") in contradiction_sources:
                    return "CONTRADICTED", [reason]
                return "SUPPRESS", [reason]
            degraded = True
            reasons.append(reason)

    return ("CAP_ADVISORY", reasons) if degraded else ("PASS", [])


# --- Generic per-lever candidate-bridge mechanism (BACKLOG item H, 2026-09-20) -
#
# config/decision_frame.json's own lever_bridges list (see that key's _comment)
# re-expressed as candidates: one row per (lever, direction), each firing
# across the phase groups where its mechanism actually acts. Mirrors
# _bridge_candidates_for_matrix_rules's own matrix_verdict grouping and
# verdict/min_severity gate exactly -- the only structural difference is that
# a lever_bridges row names its OWN list of phase groups (a lever can act
# across several) instead of the one group a specific matrix cell_id/
# rationale is tied to.

def _bridge_candidates_for_levers(evidence, registry, decision_config, existing_candidates, setup_data=None):
    """Tier B (candidate-generation plumbing) -- the Segers ch.9/10 physics
    itself is already anchored and reviewed via config/decision_frame.json's
    interaction_table; this function only decides how an already-approved
    lever_bridges row becomes a visible candidate.

    Dedupe: a lever_bridges row can overlap a candidate the hardcoded
    _exit_oversteer_candidates (or the matrix bridge above) already emits for
    the same (parameter, direction, corner) -- springs_rear/soften/exit is
    the one live case (config comment names it). `existing_candidates` is
    every candidate generated so far this call; the hardcoded/matrix path
    always wins the collision (generated first, richer evidence_refs), so
    this function simply skips a key already covered rather than re-scoring
    or merging -- never a second, disagreeing grading rule.

    FRAME DEPTH PROGRAMME Step 1 (2026-09-22): `setup_data` (additive,
    default None -- every pre-existing caller, including ui/views/
    outing_form.py's own _generate_decision_frame via generate_candidates,
    is unaffected) feeds evaluate_conditions' own setup_state checks. Each
    bridge's OPTIONAL "conditions" list (config/decision_frame.json) is
    evaluated once per firing (corner, phase_group) via evaluate_conditions
    -- PASS emits the candidate unchanged; SUPPRESS skips it entirely;
    CAP_ADVISORY emits it with a synthetic confidence-capping item appended
    to evidence_refs (reusing _candidate_confidence's own existing min()
    machinery, exactly like the MARGINAL-verdict/exit-phase-LS caps already
    do -- no new scoring formula) plus a machine-readable "condition_
    reasons" list on the candidate dict (no UI rendering this package).
    None of the 4 shipped bridges carries a "conditions" list yet, so
    evaluate_conditions([], ...) short-circuits to PASS for every one of
    them today -- byte-identical to pre-Step-1 behaviour, confirmed by the
    2026-09-22 candidate-census re-run.
    """
    cap = decision_config.get("conditions", {}).get("not_evaluable_confidence_cap", 1.0)
    bridges = decision_config.get("lever_bridges", [])
    if not bridges:
        return []

    existing_keys = {
        (action["parameter"], action["direction"], c["corner"])
        for c in existing_candidates for action in c["actions"]
    }

    matrix_by_group = {}
    for e in evidence:
        if e["type"] == "matrix_verdict":
            matrix_by_group.setdefault(e["phases"], {}).setdefault(e["corner"], []).append(e)

    candidates = []
    for bridge in bridges:
        param = bridge["lever"]
        direction = bridge["direction"]
        condition = bridge["condition"]
        verdict = condition["verdict"]
        min_sev = condition.get("min_severity", "moderate")
        # DECISION LAYER SPEC Phase C (2026-09-22): optional speed_class
        # filter, added for the splitter_offset bridges below (mirrors
        # wing_position's own matrix cells, which ARE speed_class="high"-
        # gated) -- absent (every pre-existing entry) means no filter,
        # byte-identical to before this addition.
        required_speed_class = condition.get("speed_class")
        # Method-defining, not a car tunable: +1/-1 encodes ONE step in this
        # lever's own direction_semantics (config/setup_parameters.json
        # springs_front/rear both declare increasing="stiffer"), matching the
        # ARB actions' existing delta convention -- not a physical magnitude.
        # DECISION LAYER SPEC B7 (2026-09-22): widened from a springs-only
        # "stiffen"/else check to _DIRECTION_SIGN (already established by B2
        # for exactly this purpose) so this generic mechanism also handles
        # diff_position's "increase"/"decrease" vocabulary correctly --
        # stiffen/soften still map identically, byte-identical for springs.
        delta = _DIRECTION_SIGN.get(direction, 1)

        for phase_group in bridge["phase_groups"]:
            phase_group = tuple(phase_group)
            for cid, items in matrix_by_group.get(phase_group, {}).items():
                key = (param, direction, cid)
                if key in existing_keys:
                    continue
                for ev in items:
                    if ev["verdict"] != verdict:
                        continue
                    if SEVERITY_RANK[ev["severity"]] < SEVERITY_RANK[min_sev]:
                        continue
                    if required_speed_class is not None and ev.get("speed_class") != required_speed_class:
                        continue

                    contradiction_sources = decision_config.get("conditions", {}).get(
                        "contradiction_sources", [])
                    verdict_result, reasons = evaluate_conditions(
                        bridge.get("conditions", []), cid, phase_group[-1], evidence, setup_data, registry,
                        contradiction_sources)
                    if verdict_result == "SUPPRESS":
                        continue

                    evidence_refs = [ev]
                    status = STATUS_PROPOSED
                    if verdict_result == "CAP_ADVISORY":
                        evidence_refs = evidence_refs + [{
                            "type": "condition_gap", "corner": cid, "phase": phase_group[-1],
                            "verdict": None, "severity": None, "confidence": cap,
                            "source": f"condition gap, capped at {cap}: " + "; ".join(reasons),
                        }]
                        # DECISION LAYER SPEC Stage 3: "unfilled sheet -> not-
                        # assessable, honest degrade" -- CAP_ADVISORY already
                        # means some condition could not be evaluated (missing
                        # evidence source, unfilled setup sheet, missing
                        # registry window); this status makes that visible in
                        # the lever inventory rather than only in a confidence
                        # number and a machine-readable reasons list.
                        status = STATUS_NOT_ASSESSABLE
                    elif verdict_result == "CONTRADICTED":
                        # DECISION LAYER SPEC B6 (2026-09-22): data contradicts
                        # data -- SUPPRESSED from the shortlist but NOT
                        # dropped from the full inventory (Stage 5's own
                        # wording: "SUPPRESSED from shortlist -> tail,
                        # 'contradicted by X'"). Unlike CAP_ADVISORY this
                        # candidate's own confidence is left untouched -- the
                        # contradiction is reported via status/reason, not by
                        # further degrading a number that already reads
                        # honestly for the firing evidence itself.
                        status = STATUS_CONTRADICTED
                        reasons = [f"contradicted by {reasons[0]}"] if reasons else ["contradicted"]

                    candidates.append({
                        "id": f"lever_bridge:{param}:{direction}:C{cid}:{'+'.join(phase_group)}",
                        "scenario": f"lever_bridge:{param}:{direction}",
                        "corner": cid, "phase": phase_group[-1], "phases": phase_group,
                        "lever_family": bridge.get("lever_family", param),
                        "actions": [{"parameter": param, "direction": direction, "delta": delta}],
                        "effort_class": _effort_class_for_actions([param], registry),
                        "effect_class": bridge.get("effect_class", "secondary"),
                        "grade": "proposed",  # structural cap -- see lever_bridges' own config comment
                        "cell_id": None,
                        "evidence_refs": evidence_refs,
                        "rationale": bridge["rationale"],
                        "derived_from": bridge["derived_from"],
                        "condition_reasons": reasons,
                        "status": status,
                    })
    return candidates


# --- DECISION LAYER SPEC B2: feedback-only trigger (2026-09-22) --------
#
# |driver feedback|>=2 generates a candidate with ZERO data verdict.
# Routing mechanism reviewer-confirmed 2026-09-22 (thesis_notes.md "B2
# design resolution"): interaction_table's own signed (parameter,
# direction, performance_axis, sign) entries, repurposed as a "what
# helps this axis" lookup, filtered to click-class-eligible levers only
# -- matrix-rule relaxation was explicitly REJECTED (severity floors are
# part of what the engineer elicited, not a gate to bypass for
# subjective-only input). |1| feedback never reaches this function (see
# generate_candidates' own bucketing) -- Stage 2: "|1| is a note,
# corroboration-only, never triggers".
#
# Trigger provenance (Stage 2's three labelled classes): every candidate
# from the other three generators is TRIGGER_DATA_ONLY (set in
# generate_candidates); a candidate from this function is
# TRIGGER_FEEDBACK_ONLY ("driver_reported", the spec's own wording); a
# data-only candidate that ALSO receives matching feedback corroboration
# is upgraded to TRIGGER_BOTH_AGREEING in _attach_feedback_evidence
# ("both-agreeing = strongest class").
TRIGGER_DATA_ONLY = "data_only"
TRIGGER_FEEDBACK_ONLY = "driver_reported"
TRIGGER_BOTH_AGREEING = "both_agreeing"

# Symbolic one-step direction sign -- same convention _bridge_candidates_
# for_levers already uses (delta=+-1 regardless of a parameter's real
# physical step size; the settings-window component's own distance
# formula is a heuristic already, not a literal physical delta). Matches
# config/recommendations.json's own established sign convention for
# these exact direction words (increase/decrease/more_negative/
# less_negative all verified there directly, not assumed).
_DIRECTION_SIGN = {
    "stiffen": 1, "soften": -1,
    "increase": 1, "decrease": -1,
    "more_negative": -1, "less_negative": 1,
    "more_positive": 1, "less_positive": -1,
}


def _phase_compatible(phase, phase_affinity):
    # None = global lever (no phase_affinity recorded), compatible with
    # every phase. wing_position's own phase_affinity is a speed-class
    # tag ("high_speed_corners"), not a PHASE_KEYS value -- it correctly
    # never matches a real feedback phase here; feedback carries no
    # speed_class routing today, so this is an honest exclusion, not a
    # special case this function invents.
    return phase_affinity is None or phase in phase_affinity


_AXLE_PAIR_SUFFIXES = {"_fl": "_fr", "_fr": "_fl", "_rl": "_rr", "_rr": "_rl"}


_AXLE_PAIRED_PREFIXES = ("damper_", "arb_")


def _axle_partner_param(param):
    # WP-ELICIT Phase C4 (2026-09-24, reviewer-redirected then corrected
    # again after real-suite fallout, same session): dampers are AXLE-
    # PAIRED, never asymmetric (Stage 1 design principle). ARB was
    # initially left OUT of this pairing (the registry's own arb_fl note
    # says "Four fully independent per-corner targets, no pairing
    # concept" -- describing VALUE STORAGE independence, not the
    # recommendation convention) -- but every OTHER ARB-generating
    # candidate in this codebase already issues fl+fr (or rl+rr) as ONE
    # paired action (_exit_oversteer_candidates' own arb_soften
    # candidate), and running the real suite after adding this package's
    # 4 same-cost/same-axis ARB entries per axis proved the omission was
    # a real bug, not a faithful reading of that note: an fl-vs-fr (or
    # rl-vs-rr) same-direction "tie" is never a genuine choice, it is the
    # SAME physical recommendation asked twice. Extending pairing to ARB
    # here fixes exactly that false tie -- it does NOT and cannot resolve
    # a genuine front-vs-rear cross-family tie (soften one axle vs
    # stiffen the other), which the ValueError correctly still raises on;
    # see thesis_notes.md WP-ELICIT Phase C4 for that open item.
    if not param.startswith(_AXLE_PAIRED_PREFIXES):
        return None
    for suffix, partner_suffix in _AXLE_PAIR_SUFFIXES.items():
        if param.endswith(suffix):
            return param[: -len(suffix)] + partner_suffix
    return None


def _group_axle_pairs(pool):
    # Collapses two same-direction, axle-paired pool entries (fl+fr, or
    # rl+rr) into ONE grouped item carrying both parameters -- the router
    # then competes AXLE-PAIRED FAMILIES against each other (one damper
    # bridge vs one ARB entry), never a same-family fl-vs-fr false choice
    # that would otherwise tie and crash despite there being no real
    # alternative (WP-ELICIT Phase C4: "no tie exists because no competing
    # single-wheel candidates exist"). A partner absent from the pool
    # (filtered out earlier for its own reason -- phase/click-class/
    # existing-key) degrades to an ungrouped singleton, never blocked on
    # its missing sibling.
    by_key = {(p, d): (p, d, r, e) for p, d, r, e in pool}
    grouped, seen = [], set()
    for param, direction, reg_entry, entry in pool:
        if (param, direction) in seen:
            continue
        partner = _axle_partner_param(param)
        partner_item = by_key.get((partner, direction)) if partner else None
        if partner_item is not None:
            seen.add((param, direction))
            seen.add((partner, direction))
            grouped.append({
                "params": tuple(sorted((param, partner))),
                "direction": direction,
                "reg_entries": {param: reg_entry, partner: partner_item[2]},
                "entry": entry,
            })
        else:
            seen.add((param, direction))
            grouped.append({
                "params": (param,), "direction": direction,
                "reg_entries": {param: reg_entry}, "entry": entry,
            })
    return grouped


def _feedback_only_candidates(evidence, existing_candidates, registry, config, current_setup=None):
    feedback_items = [e for e in evidence if e["type"] == "driver_feedback" and abs(e["raw_feedback"]) >= 2]
    if not feedback_items:
        return []

    click_class = set(config.get("eligibility_classes", {}).get("click_class", []))
    table = config["interaction_table"]
    existing_keys = {
        (action["parameter"], action.get("direction"), c["corner"])
        for c in existing_candidates for action in c["actions"]
    }

    candidates = []
    for fb in feedback_items:
        axis = "oversteer_tendency" if fb["verdict"] == "oversteer" else "understeer_tendency"
        pool = []
        for entry in table:
            if entry["performance_axis"] != axis or entry["sign"] != 1:
                continue
            param = entry["parameter"]
            if param not in click_class:
                continue
            reg_entry = registry.get(param)
            if reg_entry is None:
                continue
            if (param, entry["direction"], fb["corner"]) in existing_keys:
                continue
            if not _phase_compatible(fb["phase"], reg_entry.get("phase_affinity")):
                continue
            pool.append((param, entry["direction"], reg_entry, entry))
        if not pool:
            continue  # honest gap -- no fallback, no invention (B2 reviewer decision)

        grouped_pool = _group_axle_pairs(pool)

        def _effort_rank(item):
            efforts = [item["reg_entries"][p].get("change_effort") for p in item["params"]]
            return max(EFFORT_RANK.get(e, len(EFFORT_RANK)) for e in efforts)

        min_rank = min(_effort_rank(item) for item in grouped_pool)
        cheapest = [item for item in grouped_pool if _effort_rank(item) == min_rank]

        if len(cheapest) > 1:
            # HANDOFF item 1 (reviewer, 2026-09-24): window headroom breaks
            # the tie BEFORE interaction penalty -- the axle with more room
            # left before its own window edge wins, reusing
            # _settings_window_component's own score (weight*(1-distance),
            # higher = closer to nominal = more room). Only applied when
            # EVERY tied item is actually computable (the setup sheet is
            # filled for every parameter involved) -- a partially-filled
            # sheet would make this step guess for the missing side, so it
            # is skipped entirely rather than half-trusted, same "neutral
            # unless fully computable" posture _settings_window_component
            # itself already uses per action.
            def _headroom_score(item):
                probe_actions = [{"parameter": p, "direction": item["direction"],
                                   "delta": _DIRECTION_SIGN.get(item["direction"], 1)}
                                  for p in item["params"]]
                score, flags = _settings_window_component(
                    {"actions": probe_actions}, current_setup, registry, config, 1.0)
                return score, len(flags) < len(probe_actions)
            headroom = [_headroom_score(item) for item in cheapest]
            if all(computable for _, computable in headroom):
                max_headroom = max(score for score, _ in headroom)
                cheapest = [item for item, (score, _) in zip(cheapest, headroom) if score == max_headroom]

        if len(cheapest) > 1:
            def _penalty_magnitude(item):
                probe = {"corner": fb["corner"], "evidence_refs": [fb],
                         "actions": [{"parameter": p, "direction": item["direction"]} for p in item["params"]]}
                penalty, _notes = _interaction_penalty(probe, evidence, config,
                                                        config["cost_function"]["interaction"])
                return abs(penalty)
            max_pen = max(_penalty_magnitude(item) for item in cheapest)
            cheapest = [item for item in cheapest if _penalty_magnitude(item) == max_pen]

        # HANDOFF item 1 (reviewer, 2026-09-24): a genuine remaining tie --
        # different lever families, equal cost, equal headroom, equal
        # interaction penalty -- is real and ORDINARY for isolated feedback
        # with nothing else to break it (the Q9 two-sided-balance case:
        # soften one axle / stiffen the other). No longer resolved
        # arbitrarily and no longer raised as an error (that crashed
        # production on the ordinary case, not a rare one) -- both
        # alternatives are emitted as separate, ordinary pick-one
        # candidates, exactly like any other two competing shortlist rows
        # (DECISION LAYER SPEC Stage 4: "alternative single moves... ranked
        # normally, no suppression machinery"). Axle-paired same-family
        # fl/fr entries never reach this point (grouped above), so this
        # only ever fires for a real cross-family tie.
        tie_labels = ["+".join(item["params"]) + " " + item["direction"] for item in cheapest]

        for chosen, own_label in zip(cheapest, tie_labels):
            params, direction, entry = chosen["params"], chosen["direction"], chosen["entry"]
            delta = _DIRECTION_SIGN.get(direction, 1)
            param_label = "+".join(params)
            paired_note = " (axle-paired action, Stage 1 design principle)" if len(params) > 1 else ""
            tie_note = ""
            if len(cheapest) > 1:
                others = [label for label in tie_labels if label != own_label]
                tie_note = (f" ALTERNATIVE: genuinely tied with {', '.join(others)} -- cost, window "
                            f"headroom and interaction penalty all agree; no evidence here breaks it, "
                            f"engineer's call.")
            candidates.append({
                "id": f"feedback_only:{param_label}:{direction}:C{fb['corner']}:{fb['phase']}",
                "scenario": f"feedback_only:{param_label}:{direction}",
                "corner": fb["corner"], "phase": fb["phase"],
                "lever_family": param_label,
                "actions": [{"parameter": p, "direction": direction, "delta": delta} for p in params],
                "effort_class": _effort_class_for_actions(list(params), registry),
                "effect_class": "secondary",
                "grade": entry["grade"],
                "cell_id": None,
                "evidence_refs": [fb],
                "rationale": f"Driver reported {fb['verdict']} at {fb['phase']} "
                             f"(feedback {fb['raw_feedback']:+g}) -- {param_label} {direction} routed via "
                             f"interaction_table's own {axis} entry (grade={entry['grade']}), "
                             f"cheapest click-class lever available{paired_note}.{tie_note}",
                "status": STATUS_PROPOSED,
                "trigger_provenance": TRIGGER_FEEDBACK_ONLY,
            })
    return candidates


# --- DECISION LAYER SPEC B3: eligibility gate (2026-09-22) -------------
#
# Stage 6: "ELIGIBILITY CLASS = magnitude matching: mild/single-corner
# problems reach click-class levers only; springs/camber/toe unlock only
# at strong+multi-corner or |feedback|>=4 [A]... Camber ADDITIONALLY
# always requires the multi-corner gate per spec" (B3's own wording,
# no |feedback|>=4 bypass for camber specifically). This RAISES the
# existing matrix-rule floor for camber/toe from "moderate" (config/
# recommendations.json US-APX-high/OS-APX-high/US-BRK-low/US-TIN-low/
# OS-TIN-low/INST-BRK-high, verified directly) to "strong", and adds a
# cross-corner requirement neither the matrix engine nor lever_bridges
# has ever enforced -- a real, expected behaviour change (not a bug),
# per the work order's own B8 instruction that census counts WILL
# change in this phase.
#
# "Same axle" only matters for camber (4 independent per-corner registry
# keys); springs_front/rear and toe_front/rear are already axle-level
# parameters, so the axle-family for those is just the parameter itself.
_CAMBER_AXLE_FAMILY = {
    "camber_fl": "camber_front", "camber_fr": "camber_front",
    "camber_rl": "camber_rear", "camber_rr": "camber_rear",
}


def _axle_family(parameter):
    return _CAMBER_AXLE_FAMILY.get(parameter, parameter)


def _apply_eligibility_gate(candidates, evidence, config, driving_level=None):
    heavy = set(config.get("eligibility_classes", {}).get("heavy_correctors", []))
    if not heavy:
        return candidates

    # WP-ELICIT Phase C1 (2026-09-24, reviewer-redirected -- the author's
    # own driver-level Q11 answer did NOT elicit a new single-corner
    # unlock route; it elicited TRUST ARBITRATION on the existing
    # multi-corner DATA path: "a calm expert's moderate rating is
    # evidence of manageability", not a route to a heavier move on its
    # own. driving_level is the outing's driver's plain Driver.
    # driving_level int (1-10) or None -- resolved by the UI caller from
    # self.outing.driver, never queried from a DB session in modules/,
    # same plain-value-boundary convention modules.recommendation.
    # generate_recommendations already uses for the identical field.
    driver_level_threshold = config.get("eligibility_classes", {}).get("driver_level_threshold", 5)
    # "Unknown trust never unlocks the heavier move": an unresolved level
    # sits AT the neutral threshold itself (>= comparison below reaches
    # it), not below it -- the same conservative-default direction
    # driver_level_weighting.neutral_level already uses project-wide.
    effective_driving_level = driving_level if driving_level is not None else driver_level_threshold
    feedback_cfg = config.get("driver_feedback_weighting", {})
    _confidence_bands = feedback_cfg.get("confidence_bands", [])
    # "Moderate |fb| 2-3" reuses driver_feedback_weighting's OWN band
    # boundaries (low/mid split) rather than a second hardcoded 2/3 --
    # the mid band IS this project's own definition of "moderate".
    _moderate_lo = _confidence_bands[0]["max_abs"] if _confidence_bands else 1
    _moderate_hi = _confidence_bands[1]["max_abs"] if len(_confidence_bands) > 1 else 3

    # ELIGIBILITY GATE AMENDMENT (2026-09-23, reviewer decision from item
    # (a) findings, thesis_notes.md "Eligibility gate amendment"): the
    # |feedback|>=4 relaxation matches at CORNER level with SIGN
    # CONSISTENCY, not phase-exact -- keyed on (corner, verdict), not
    # (corner, phase) as before. Rationale for record: driver feedback is
    # corner-granular testimony -- drivers do not phase-segment their own
    # complaint; phase attribution is the pipeline's own job, not
    # something the eligibility check should demand agreement on. The
    # >=2-corner strong-severity DATA path below is phase-scoped and
    # UNCHANGED by this amendment.
    feedback_magnitude_by_corner_verdict = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            key = (e["corner"], e["verdict"])
            feedback_magnitude_by_corner_verdict[key] = max(
                feedback_magnitude_by_corner_verdict.get(key, 0), abs(e["raw_feedback"])
            )

    def _is_heavy(c):
        return any(a["parameter"] in heavy for a in c["actions"])

    heavy_candidates = [c for c in candidates if _is_heavy(c)]
    if not heavy_candidates:
        return candidates
    other_candidates = [c for c in candidates if not _is_heavy(c)]

    # (axle_family, direction) -> distinct corners showing STRONG severity
    # for that group. Severity is read the SAME way score() already reads
    # it (evidence_refs[0]'s own severity) -- no second severity rule.
    strong_corners_by_group = {}
    for c in heavy_candidates:
        primary = c["evidence_refs"][0] if c["evidence_refs"] else None
        severity = primary.get("severity") if primary else None
        if severity != "strong":
            continue
        for a in c["actions"]:
            if a["parameter"] not in heavy:
                continue
            group = (_axle_family(a["parameter"]), a["direction"])
            strong_corners_by_group.setdefault(group, set()).add(c["corner"])

    kept = []
    for c in heavy_candidates:
        eligible = False
        primary = c["evidence_refs"][0] if c["evidence_refs"] else None
        candidate_verdict = primary.get("verdict") if primary else None
        fb_mag = feedback_magnitude_by_corner_verdict.get((c["corner"], candidate_verdict), 0)
        moderate_feedback = _moderate_lo < fb_mag <= _moderate_hi
        for a in c["actions"]:
            if a["parameter"] not in heavy:
                continue
            group = (_axle_family(a["parameter"]), a["direction"])
            multi_corner_ok = len(strong_corners_by_group.get(group, set())) >= 2
            # WP-ELICIT Phase C1 veto: holds back what the multi-corner
            # DATA path would otherwise admit when the driver's OWN
            # rating at this corner/verdict is only moderate AND their
            # trust level is at/above neutral -- read as "an experienced
            # driver finds this manageable", not as new evidence against
            # the data. A tightening only: no feedback, feedback outside
            # the moderate band, or a below-neutral (less-trusted) level
            # leaves multi_corner_ok exactly as the pre-existing rule
            # computed it. Applies to camber too (its multi-corner
            # requirement is the FLOOR this tightens, never a route
            # around it -- camber still gets no feedback_ok bypass below).
            if multi_corner_ok and moderate_feedback and effective_driving_level >= driver_level_threshold:
                multi_corner_ok = False
            # Camber never gets the feedback bypass -- always multi-corner.
            feedback_ok = fb_mag >= 4 and not a["parameter"].startswith("camber_")
            if multi_corner_ok or feedback_ok:
                eligible = True
                break
        if eligible:
            kept.append(c)
    return other_candidates + kept


# --- DECISION LAYER SPEC B4: breadth (2026-09-22) -----------------------
#
# Stage 6: "breadth penalty (global lever helping 1 of N corners is
# penalised, stated as 'helps CX, risks others'; per-corner-capable
# levers exempt". "Per-corner-capable" == dampers, per the spec (the
# bump/rebound LS/HS split gives them shaft-speed-range selectivity a
# single spring/ARB/camber setting doesn't have).
#
# N resolved (reviewer-confirmed 2026-09-22, thesis_notes.md "B4 breadth
# design resolution", two rounds): N = every corner ASSESSED this
# session, INCLUDING normal verdicts -- a normal verdict is itself
# positive evidence of a working state a global rebalance would risk.
# This is NOT derivable from the evidence list alone (_build_corner_
# verdict_evidence/_build_matrix_verdict_evidence both skip
# severity=="normal" -- confirmed directly): hence the additive optional
# `assessed_corner_ids` parameter below, rather than an evidence-only
# proxy (explicitly rejected -- it reproduces the same zeroing failure
# in the spec's own motivating case of one bad corner, rest good).

def _attach_breadth(candidates, assessed_corner_ids):
    by_action = {}
    for c in candidates:
        for a in c["actions"]:
            by_action.setdefault((a["parameter"], a.get("direction")), set()).add(c["corner"])

    if not assessed_corner_ids:
        # No corner census supplied -- every pre-existing caller. Breadth
        # fields present but null: no invented N, no evidence-only proxy.
        for c in candidates:
            c["corners_helped"] = None
            c["corners_touched"] = None
            c["breadth_note"] = None
        return candidates

    assessed = set(assessed_corner_ids)
    for c in candidates:
        exempt = any(a["parameter"].startswith("damper_") for a in c["actions"])
        helped = set()
        opposed = set()
        for a in c["actions"]:
            param, direction = a["parameter"], a.get("direction")
            helped |= by_action.get((param, direction), set())
            for (other_param, other_direction), corners in by_action.items():
                if other_param == param and other_direction != direction:
                    opposed |= corners
        c["corners_helped"] = sorted(helped)
        if exempt:
            c["corners_touched"] = sorted(helped)
            c["breadth_note"] = None
            continue

        c["corners_touched"] = sorted(assessed)
        helped_in_assessed = helped & assessed
        rebalanced = sorted(assessed - helped_in_assessed)
        notes = []
        if rebalanced:
            helped_str = "/".join(str(x) for x in sorted(helped_in_assessed)) or "none assessed"
            notes.append(
                f"helps C{helped_str} -- rebalances {len(rebalanced)} corner(s) "
                f"currently assessed good (C{'/'.join(str(x) for x in rebalanced)})"
            )
        opposed_only = sorted(opposed - helped)
        if opposed_only:
            notes.append(f"directly opposed at C{'/'.join(str(x) for x in opposed_only)}")
        c["breadth_note"] = "; ".join(notes) if notes else None
    return candidates


def _attach_tc_safety_note(candidates, config):
    # WP-ELICIT HANDOFF C6 (Q6b, author-elicited 2026-09-24): a LOWER-TC-
    # intervention action (direction=decrease on tc_lat/tc_lon -- "less
    # permitted rotation/spin" per the TC_reference table's own semantics,
    # never raw position arithmetic) trades safety margin for rotation.
    # Display-only dropdown caution -- never a score term, never a
    # suppression -- applied uniformly to every candidate regardless of
    # which generator produced it, same pattern as _attach_breadth above.
    # Text and the covered (parameter, direction) pairs both live in
    # config/decision_frame.json's tc_safety_note, not hardcoded.
    tc_cfg = config.get("tc_safety_note")
    if not tc_cfg:
        for c in candidates:
            c["tc_safety_note"] = None
        return candidates
    flagged = {(p, tc_cfg["direction"]) for p in tc_cfg["parameters"]}
    for c in candidates:
        c["tc_safety_note"] = tc_cfg["text"] if any(
            (a["parameter"], a.get("direction")) in flagged for a in c["actions"]
        ) else None
    return candidates


# --- DECISION LAYER SPEC B5: window edge -> blocked_at_edge (2026-09-22) -
#
# Stage 3: "AT EDGE: candidate shown BLOCKED at its earned rank, reason
# stated ('bias at rear limit'), alternative ranks up on its own merit --
# no suppression, no auto-promotion. Soft edge (typical_window) labelled
# 'engineer may exceed'; hard edge (physical, e.g. splitter contact)
# labelled as such." Universal per-candidate check (not the opt-in
# per-lever_bridges "conditions" list evaluate_conditions already serves)
# -- gated on `setup_data` being supplied at all, same additive/opt-in
# pattern B4 already established for assessed_corner_ids: setup_data=None
# (every pre-existing caller) skips this check entirely, byte-identical.
#
# Hard vs soft resolved without inventing new per-parameter data: HARD =
# the registry's own value_space min/max (a physical/legal bound no
# adjustment can cross -- e.g. arb position cannot exist outside 1-7;
# splitter_offset's own Phase A note already ties its value_space extremes
# to front-splitter/track contact). SOFT = decision_frame.json's own
# parameter_windows (nominal+-span, the typical-practice range an engineer
# may choose to exceed). Both are real, already-present structured data --
# no new config field, no guessed limit values.
#
# Directional: "at edge" only blocks when the candidate's OWN delta pushes
# FURTHER past that same edge (worsening/impossible), never when the
# proposed direction corrects back toward nominal from an edge already
# reached -- Stage 3's own "current state of the levers" framing (FRAME
# DEPTH PROGRAMME's motivating example: check toe/camber's CURRENT value
# before recommending more of the same direction).
#
# "Unfilled sheet -> not-assessable" (Stage 3) applies per-lever, only
# once setup_data is supplied but THIS parameter's own value is missing --
# never a blanket rule (a candidate whose own lever's value IS on record
# is checked normally even if some OTHER lever's sheet cell is empty).

def _window_edge_check(action, setup_data, registry, config):
    param = action["parameter"]
    delta = action.get("delta")
    if delta is None or delta == 0:
        return None  # target-style action, no directional push to check
    entry = registry.get(param)
    if entry is None:
        return None
    current = _current_setup_value(setup_data, entry)
    if current is None:
        return ("not_assessable", f"setup sheet unfilled: {param}")
    try:
        current = float(current)
    except (TypeError, ValueError):
        return None  # enum label (e.g. wing_position "P9") -- not numerically checkable, same
                     # defensive skip _settings_window_component already uses for this case

    push_sign = 1 if delta > 0 else -1

    value_space = entry.get("value_space") or {}
    vmin, vmax = value_space.get("min"), value_space.get("max")
    if push_sign > 0 and vmax is not None and current >= vmax:
        return ("hard", f"{param} already at its hard maximum ({current} >= {vmax})")
    if push_sign < 0 and vmin is not None and current <= vmin:
        return ("hard", f"{param} already at its hard minimum ({current} <= {vmin})")

    window = config["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or span is None or not span:
        return None  # no typical-window data for this parameter -- silent, same as scoring's own skip
    distance = current - nominal
    if (push_sign > 0 and distance >= span) or (push_sign < 0 and distance <= -span):
        return ("soft", f"{param} already at its typical-window edge "
                         f"(current={current}, nominal={nominal}, span={span})")
    return None


def _edge_normal_state_label(action, config):
    # WP-ELICIT Phase C2 (2026-09-24): config/decision_frame.json's
    # edge_normal_state_params -- see its own comment for the full
    # provenance-integrity reasoning (doctrine lands as annotation/
    # exemption here, never as an edit to an engineer-verbatim rule).
    for entry in config.get("edge_normal_state_params", []):
        if entry["parameter"] == action["parameter"] and entry["direction"] == action.get("direction"):
            return entry["label"]
    return None


def _apply_window_edge_status(candidates, setup_data, registry, config):
    if setup_data is None:
        return candidates
    for c in candidates:
        if c.get("status") != STATUS_PROPOSED:
            continue  # not_assessable (B1/evaluate_conditions) takes precedence, not overridden here
        for action in c["actions"]:
            result = _window_edge_check(action, setup_data, registry, config)
            if result is None:
                continue
            kind, reason = result
            normal_label = _edge_normal_state_label(action, config) if kind in ("hard", "soft") else None
            if normal_label is not None:
                # Exempted: this action's own edge is its documented normal
                # resting state, not a block -- annotate and keep checking
                # the candidate's OTHER actions (a real co-action edge must
                # still be able to block normally).
                c["edge_normal_state_note"] = f"{action['parameter']}: {normal_label} ({reason})"
                continue
            if kind == "not_assessable":
                c["status"] = STATUS_NOT_ASSESSABLE
                c["edge_reason"] = reason
            else:
                c["status"] = STATUS_BLOCKED_AT_EDGE
                c["edge_kind"] = kind
                c["edge_reason"] = reason
                c["edge_label"] = "engineer may exceed" if kind == "soft" else "hard limit"
            break  # first blocking/not-evaluable action found is enough to set this candidate's status
    return candidates


def generate_candidates(evidence, registry, config, setup_data=None, assessed_corner_ids=None,
                         driving_level=None):
    """Candidate layer. See module-level comment above for Stage 1's own
    scope; Stage 2 (Frame-Stage-2 Phase 3, 2026-09-04) adds
    _bridge_candidates_for_matrix_rules (all 39 config/recommendations.json
    rules re-expressed as bridges -- see that function's own comment) and
    TC-intervention-evidence wiring into the existing exit-oversteer
    candidates. `registry` is modules.recommendation.
    load_setup_parameters_registry()'s own dict; `config` is load_decision_
    frame_config()'s dict -- candidate generation otherwise reads config/
    recommendations.json directly for matrix-cell lookups, not config/
    decision_frame.json, EXCEPT `config`'s own lever_bridges list (BACKLOG
    item H, 2026-09-20), the one source that genuinely lives there (see
    _bridge_candidates_for_levers and that config key's own comment).

    `setup_data` (FRAME DEPTH PROGRAMME Step 1, 2026-09-22, additive,
    default None) is the outing's own setup-sheet dict, same shape
    generate_shortlist's own `current_setup` parameter already takes --
    threaded through to _bridge_candidates_for_levers only, for its
    optional per-bridge "conditions" setup_state checks. Every pre-
    existing caller (ui/views/outing_form.py's _generate_decision_frame
    calls generate_candidates without this argument) is unaffected.

    `assessed_corner_ids` (DECISION LAYER SPEC B4, 2026-09-22, additive,
    default None) is the set/iterable of every corner id assessed this
    session (modules.recommendation._group_by_corner(summaries).keys(),
    NOT derivable from `evidence` alone -- see _attach_breadth's own
    comment). None -> breadth fields present but null on every candidate,
    byte-identical to pre-B4 behaviour for every existing caller.

    `driving_level` (WP-ELICIT Phase C1, 2026-09-24, additive, default
    None) is the outing's driver's plain Driver.driving_level int (1-10)
    or None -- resolved by the UI caller from self.outing.driver, never
    queried from a DB session in modules/ (same plain-value-boundary
    convention modules.recommendation.generate_recommendations already
    uses for the identical field). Threaded through to
    _apply_eligibility_gate's trust-arbitration veto only; None leaves
    that veto exactly as conservative as an at-or-above-neutral level
    (see that function's own comment).
    """
    config_recs = load_recommendations_config()

    corner_verdicts_by_key = {}
    ls_by_key = {}
    intervention_abs_by_corner = {}
    intervention_abs_masking_by_corner = {}
    intervention_tc_by_corner = {}
    for e in evidence:
        if e["type"] == "corner_verdict":
            corner_verdicts_by_key.setdefault((e["corner"], e["phase"]), []).append(e)
        elif e["type"] == "ls_disambiguation":
            ls_by_key[(e["corner"], e["phase"])] = e
        elif e["type"] == "intervention_abs":
            intervention_abs_by_corner[e["corner"]] = e
        elif e["type"] == "intervention_tc":
            intervention_tc_by_corner[e["corner"]] = e
        elif e["type"] == "intervention_abs_masking":
            intervention_abs_masking_by_corner[e["corner"]] = e

    candidates = []
    candidates += _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs,
                                              intervention_tc_by_corner)
    candidates += _brake_balance_candidates(evidence, registry, config_recs)
    # DECISION LAYER SPEC B7 (2026-09-22): brake_bias bridge (Segers C5-1).
    candidates += _brake_bias_candidates(evidence)
    candidates += _bridge_candidates_for_matrix_rules(evidence, registry, config_recs, intervention_abs_by_corner,
                                                        intervention_abs_masking_by_corner)
    candidates += _bridge_candidates_for_levers(evidence, registry, config, candidates, setup_data)
    # WP-ELICIT Phase C3 (2026-09-24): kerb-strike-severity blowoff
    # candidates -- click-class (dampers), so untouched by the heavy-
    # corrector eligibility gate immediately below.
    candidates += _kerb_blowoff_candidates(evidence, registry)
    # DECISION LAYER SPEC B3 (2026-09-22): heavy correctors (springs/camber/
    # toe) only survive at strong+multi-corner or (non-camber) |feedback|>=4
    # -- applied BEFORE the feedback-only generator below, so a heavy
    # corrector gated out here never blocks a cheaper click-class
    # alternative from firing at the same corner via _feedback_only_
    # candidates' own existing_keys dedupe.
    candidates = _apply_eligibility_gate(candidates, evidence, config, driving_level=driving_level)
    # DECISION LAYER SPEC B1 (2026-09-22): every candidate carries a status.
    # Only _bridge_candidates_for_levers currently has a mechanism that can
    # produce anything other than "proposed" (its evaluate_conditions call);
    # the other three generators have no blocking/contradiction mechanism
    # yet (B5/B6's own job), so their candidates default to proposed here.
    # Same pass sets trigger_provenance=data_only -- every one of these four
    # generators keys off a real verdict-bearing evidence item.
    for c in candidates:
        c.setdefault("status", STATUS_PROPOSED)
        c.setdefault("trigger_provenance", TRIGGER_DATA_ONLY)
    # DECISION LAYER SPEC B2 (2026-09-22): feedback-only candidates, routed
    # independently of any data verdict -- deduped against everything
    # generated so far (existing_keys inside the function), so a lever
    # already covered by a data-driven candidate never gets a second,
    # competing feedback-only proposal for the same (parameter, direction,
    # corner).
    candidates += _feedback_only_candidates(evidence, candidates, registry, config, current_setup=setup_data)
    # Deepening Phase 4d: attaches any driver_feedback evidence (built by
    # build_evidence when feedback_data was supplied) to every candidate
    # whose own evidence_refs share its corner/phase/verdict -- corroborates
    # via the existing MIN-confidence rule, never a new candidate/action.
    # DECISION LAYER SPEC B2: also upgrades trigger_provenance to
    # both_agreeing when a data-only candidate receives matching feedback
    # (Stage 2: "both-agreeing = strongest class") -- never touches a
    # feedback-only candidate's own provenance (it has no data verdict to
    # agree with by definition).
    candidates = _attach_feedback_evidence(candidates, evidence)
    # DECISION LAYER SPEC B6 (2026-09-22): driver-vs-data disagreement is
    # display data only -- never suppresses, never touches status/score.
    candidates = _attach_conflicting_feedback(candidates, evidence)
    # DECISION LAYER SPEC B5 (2026-09-22): window edge -> blocked_at_edge.
    # setup_data is None for every pre-existing caller that doesn't pass it
    # -- byte-identical (status untouched).
    candidates = _apply_window_edge_status(candidates, setup_data, registry, config)
    # DECISION LAYER SPEC B4 (2026-09-22): breadth. assessed_corner_ids is
    # None for every pre-existing caller -- byte-identical (fields present,
    # null, no invented number).
    candidates = _attach_breadth(candidates, assessed_corner_ids)
    # WP-ELICIT HANDOFF C6 (2026-09-24): display-only TC safety caution,
    # applied last so it covers every candidate regardless of generator.
    candidates = _attach_tc_safety_note(candidates, config)
    return candidates


# --- Scoring layer (shared by every candidate, Stage 1 and Stage 2) ----
#
# DECISION LAYER SPEC C1 (2026-09-22) REVISION: six components exactly,
# per Stage 6's own term order -- problem weight (severity x
# phase_importance x confidence), change_time, breadth, headroom,
# interaction, effect_class. Every weight lives in config/decision_frame.
# json's cost_function (scoring_weights retired, see that key's own
# retirement comment) -- all placeholders pending engineer elicitation,
# except phase_importance/effect_class's already-elicited orderings,
# carried over unchanged. Deterministic: no randomness, no hidden global
# state -- identical (candidate, evidence, current_setup, config) always
# produces an identical breakdown.

# performance_axis -> the corner_verdict verdict string it corresponds to,
# for the interaction-penalty lookup below. braking_performance/
# traction_performance have no verdict-type analogue in this evidence
# schema (they are outcome axes, not classify_fn verdicts) -- Stage 1 has
# no OTHER-evidence source to check them against yet (the camber entries
# they back belong to a scenario this stage doesn't implement); they are
# simply never matched, not silently miscounted.
_AXIS_TO_VERDICT = {
    "understeer_tendency": "understeer",
    "oversteer_tendency": "oversteer",
    "yaw_stability": "unstable_yaw",
}


def _candidate_confidence(candidate):
    # A candidate is only as trustworthy as its weakest supporting
    # evidence -- MIN, not mean, so one shaky evidence_ref (e.g. an
    # ls_disambiguation split with a thin population) drags the whole
    # candidate down rather than being averaged away.
    confs = [e["confidence"] for e in candidate["evidence_refs"] if e.get("confidence") is not None]
    return min(confs) if confs else 0.0


def _settings_window_component(candidate, current_setup, registry, decision_config, weight):
    windows = decision_config["parameter_windows"]
    distances = []
    flags = []
    for action in candidate["actions"]:
        if "target" in action:
            continue  # absolute-target actions have no delta-based window distance
        param = action["parameter"]
        window = windows.get(param, {})
        nominal, span = window.get("nominal"), window.get("span")
        entry = registry.get(param)
        current = _current_setup_value(current_setup, entry) if entry else None
        if nominal is None or span is None or not span or current is None:
            flags.append(f"{param}: settings-window distance not computable "
                         f"(nominal={nominal}, span={span}, current={current}) -- neutral, contributes 0")
            continue
        # Deepening Phase 4a (2026-09-18): current comes from the outing's
        # own setup_data, which stores an ENUM-typed parameter's real value
        # as its LABEL string (e.g. wing_position="P9", arb_front_mount=
        # "P1"), not a number -- float() on that raises. A numeric-typed
        # parameter stored as a numeral string ("11" for a damper click
        # count) converts fine; only a genuine enum label does not. Caught
        # defensively rather than left to crash the whole scoring call --
        # same "neutral, never guessed" treatment as a missing value, since
        # no per-parameter label->number mapping exists yet to resolve it.
        try:
            new_value = float(current) + action["delta"]
        except (TypeError, ValueError):
            flags.append(f"{param}: settings-window distance not computable "
                         f"(current={current!r} is not numeric, likely an enum label) -- neutral, contributes 0")
            continue
        distances.append(min(1.0, abs(new_value - nominal) / span))
    if not distances:
        return 0.0, flags
    # Worst (max) distance across a package's own actions -- same
    # conservative "max across the package" convention modules.
    # recommendation._rank_key already uses for escalation tier.
    return weight * (1.0 - max(distances)), flags


def _interaction_penalty(candidate, evidence, decision_config, weight):
    # KNOWN LIMITATION (Stage 2, 2026-09-04): corner_verdict (single-phase)
    # and matrix_verdict (phase-group, e.g. exit_4+exit_5 as one unit) can
    # both describe the SAME real event at a corner from two different
    # evidence sources. own_refs excludes only this candidate's OWN
    # evidence_refs, so a same-event duplicate from the OTHER evidence type
    # could in principle count as an "other active problem" here. Currently
    # inert: none of the seeded interaction_table entries target
    # 'oversteer_tendency' (the only axis this exact duplication could
    # spuriously trigger, since same-verdict evidence never matches an
    # entry's OPPOSING-axis target) -- flagged for whoever next extends
    # interaction_table, not fixed here (would need evidence de-duplication
    # by (corner, verdict, phase-overlap), out of this phase's own scope).
    table = decision_config["interaction_table"]
    corner = candidate["corner"]
    own_refs = {id(e) for e in candidate["evidence_refs"]}
    other_active = [e for e in evidence
                    if e["corner"] == corner and id(e) not in own_refs
                    and e.get("severity") not in (None, "normal")]
    other_verdicts = {e["verdict"] for e in other_active if e.get("verdict")}

    param_directions = {(a["parameter"], a.get("direction")) for a in candidate["actions"]}
    penalty = 0.0
    notes = []
    for entry in table:
        if (entry["parameter"], entry["direction"]) not in param_directions:
            continue
        target_verdict = _AXIS_TO_VERDICT.get(entry["performance_axis"])
        if target_verdict is None or target_verdict not in other_verdicts:
            continue  # no OTHER active problem on this axis -- null, contributes 0
        penalty += weight * entry["sign"]
        notes.append(f"{entry['parameter']} {entry['direction']} -> {entry['performance_axis']} "
                     f"(sign={entry['sign']:+d}, grade={entry['grade']})")
    return penalty, notes


def _breadth_penalty(candidate, weight):
    """DECISION LAYER SPEC C1 (2026-09-22): Tier B formula turning B4's
    own corners_helped/corners_touched data into a scalar. penalty =
    -(1 - helped/touched) -- 0 when the candidate helps every assessed
    corner (or is breadth-exempt, since an exempt candidate's own
    corners_touched==corners_helped by construction, _attach_breadth
    above), 0 and flagged when breadth data was never computed at all
    (assessed_corner_ids omitted from generate_candidates). Same
    normalized-distance style _settings_window_component already uses
    ("1 - distance/span"), not a new invented style.
    """
    helped = candidate.get("corners_helped")
    touched = candidate.get("corners_touched")
    if helped is None or touched is None:
        return 0.0, ["breadth not computable (no assessed_corner_ids supplied to "
                      "generate_candidates) -- neutral, contributes 0"]
    if not touched:
        return 0.0, []
    return weight * -(1.0 - len(helped) / len(touched)), []


def score(candidate, evidence, current_setup, config):
    """Scoring layer. DECISION LAYER SPEC Stage 6 (2026-09-22, REVISED
    Phase C -- thesis_notes.md "C1: scoring-term fold"). Six components,
    each weighted by config['cost_function'] EXCLUSIVELY (the older
    scoring_weights block is retired, see its own config comment for the
    key-by-key migration), summed to one scalar with the full breakdown
    retained for the UI's own expandable reasoning.

    Term order matches Stage 6 exactly: (1) problem weight = severity x
    phase_importance x confidence (phase_importance folded in here --
    it qualifies the PROBLEM, not any one lever's fit to it); within
    class, (2) change_time (inverse effort), (3) breadth, (4) headroom
    (settings-window distance), (5) interaction, (6) effect_class
    (lever-fit/solution-quality, grouped with interaction -- deliberately
    NOT folded into problem weight, since the SAME problem can yield a
    different effect_class from different candidate levers; thesis_notes.
    md has the full category-error reasoning for this split).

    `evidence` is build_evidence()'s own full list (needed by the
    interaction-penalty component to see this corner's OTHER active
    problems). `current_setup` is the outing's setup-sheet dict (same
    shape modules.recommendation.generate_recommendations' setup_data
    parameter takes). `config` is load_decision_frame_config()'s dict.
    """
    weights = config["cost_function"]
    registry = load_setup_parameters_registry()
    flags = []

    primary = candidate["evidence_refs"][0] if candidate["evidence_refs"] else None
    sev_rank = SEVERITY_RANK.get(primary.get("severity") if primary else None, 0)
    phase_importance = weights["phase_importance"].get(candidate["phase"], 1.0)
    confidence = _candidate_confidence(candidate)
    c_problem_weight = weights["severity"] * sev_rank * phase_importance * confidence

    if candidate["effort_class"] is None:
        c_change_time = 0.0
        flags.append("effort_class unset (unrouted candidate)")
    else:
        c_change_time = weights["change_time"] / (EFFORT_RANK.get(candidate["effort_class"], 0) + 1)

    c_breadth, breadth_flags = _breadth_penalty(candidate, weights["breadth"])
    flags += breadth_flags

    c_window, window_flags = _settings_window_component(
        candidate, current_setup, registry, config, weights["headroom"])
    flags += window_flags

    c_interaction, interaction_notes = _interaction_penalty(
        candidate, evidence, config, weights["interaction"])

    if candidate["effect_class"] is None:
        c_effect_class = 0.0
        flags.append("effect_class unset (unrouted candidate)")
    else:
        c_effect_class = weights["effect_class"].get(candidate["effect_class"], 0.0)

    total = c_problem_weight + c_change_time + c_breadth + c_window + c_interaction + c_effect_class

    return {
        "total": round(total, 4),
        "components": {
            "problem_weight": round(c_problem_weight, 4),
            "change_time": round(c_change_time, 4),
            "breadth": round(c_breadth, 4),
            "headroom": round(c_window, 4),
            "interaction": round(c_interaction, 4),
            "effect_class": round(c_effect_class, 4),
        },
        "interaction_notes": interaction_notes,
        "flags": flags,
    }


def _score_and_sort(candidates, evidence, current_setup, config):
    # Shared by generate_shortlist and generate_lever_inventory -- every
    # real candidate scored via score() above, sorted by total score
    # descending. Deterministic tie-break on candidate id (lexical) so
    # identical inputs always produce an identical ordering, even when two
    # candidates score exactly equal.
    scored = []
    for c in candidates:
        result = score(c, evidence, current_setup, config)
        scored.append({
            **c,
            "score": result["total"],
            "score_components": result["components"],
            "score_interaction_notes": result["interaction_notes"],
            "score_flags": result["flags"],
        })
    scored.sort(key=lambda c: (-c["score"], c["id"]))
    return scored


def generate_shortlist(candidates, evidence, current_setup, config):
    """Ranked shortlist. DECISION LAYER SPEC B1 (reviewer-confirmed
    2026-09-22): the shortlist is status=="proposed" candidates ONLY --
    every other status (no_trigger/blocked_at_edge/contradicted/
    not_assessable) belongs in generate_lever_inventory's own tail, never
    here. Every existing real session's candidates are status=="proposed"
    today (no lever_bridges entry declares a "conditions" list yet), so
    this filter is currently a no-op against the 2026-09-22 census
    baseline -- it only starts excluding candidates once B5/B6/a
    conditions-bearing bridge actually produces a non-proposed status.
    """
    scored = _score_and_sort(candidates, evidence, current_setup, config)
    return [c for c in scored if c.get("status", STATUS_PROPOSED) == STATUS_PROPOSED]


def reachable_lever_keys(registry):
    """Every registry lever the DECISION LAYER SPEC design principle
    requires an inventory entry for: every recommendation_target=true key,
    session-wide -- NOT per corner. A registry key like ride_height_front
    covers both FL/FR wheel positions, and the same key can fire at many
    different track corners without being a different lever; "reachable"
    is a property of the registry entry, not of any one corner's evidence.
    """
    return {k for k, v in registry.items() if isinstance(v, dict) and v.get("recommendation_target")}


def generate_lever_inventory(candidates, evidence, current_setup, config, registry):
    """DECISION LAYER SPEC design principle, B1 (reviewer-confirmed
    granularity, thesis_notes.md 2026-09-22): every reachable lever always
    resolves to exactly one status; silent unreachability is structurally
    impossible. Ordering (reviewer-confirmed): proposed candidates first
    (identical order to generate_shortlist), then real non-proposed
    candidates (blocked_at_edge/contradicted/not_assessable) ranked by
    their own earned score, then synthetic no_trigger rows last, unranked
    -- no candidate object exists for a lever nothing ever pointed at, so
    there is nothing to score.
    """
    scored = _score_and_sort(candidates, evidence, current_setup, config)
    proposed = [c for c in scored if c.get("status", STATUS_PROPOSED) == STATUS_PROPOSED]
    tail_real = [c for c in scored if c.get("status", STATUS_PROPOSED) != STATUS_PROPOSED]
    touched = {action["parameter"] for c in candidates for action in c["actions"]}
    no_trigger = [
        {
            "id": f"no_trigger:{lever}",
            "status": STATUS_NO_TRIGGER,
            "lever": lever,
            "corner": None,
            "phase": None,
            "actions": [],
            "evidence_refs": [],
            "rationale": "No evidence pointed at this lever this session.",
        }
        for lever in sorted(reachable_lever_keys(registry) - touched)
    ]
    return proposed + tail_real + no_trigger


def generate_display_split(candidates, evidence, current_setup, config, registry):
    """DECISION LAYER SPEC C1 (2026-09-22), Stage 6: "display cutoff is a
    config threshold on score. Ranking never hides." SUPERSEDED by
    WP-ELICIT Phase A2 (author-elicited 2026-09-24): display_score_threshold
    is no longer read here -- the shortlist is every status==proposed
    candidate (any score), and the real visibility cutoff (rank-based top
    N distinct proposals) is applied downstream by apply_display_top_n,
    AFTER resolve_conflicts and group_display_rows have run on this
    shortlist (grouping must happen on distinct display rows, not on the
    raw per-corner candidate list -- see apply_display_top_n's own
    docstring). Every other inventory entry (blocked_at_edge, contradicted,
    not_assessable, no_trigger) lives in the tail here, same as before --
    the tail still carries every one of them with its own status, never a
    silent drop. No fixed candidate count anywhere: both lists can be any
    length, including empty.
    """
    inventory = generate_lever_inventory(candidates, evidence, current_setup, config, registry)
    visible = [c for c in inventory if c.get("status") == STATUS_PROPOSED]
    visible_ids = {id(c) for c in visible}
    tail = [c for c in inventory if id(c) not in visible_ids]
    return {"shortlist": visible, "tail": tail}


def apply_display_top_n(shortlist, tail, config):
    """WP-ELICIT Phase A2 (author-elicited 2026-09-24, elicitation item 9
    RESOLVED): rank-based top N distinct proposals is now the PRIMARY
    display cutoff, replacing display_score_threshold (config key kept,
    no longer read for visibility -- see its own retirement comment).

    Called AFTER group_display_rows, not before: `shortlist` here must
    already be the grouped, conflict-resolved display rows (one row per
    distinct parameter-set/direction/magnitude, DECISION LAYER SPEC Phase D
    feedback round ITEM 1) -- "distinct proposals" means distinct in that
    same sense the engineer already sees on screen, not distinct per
    underlying per-corner candidate. Applying this before grouping would
    silently under-fill the visible list whenever one lever's own
    candidates from several corners would have collapsed into a single row.

    "Ranking never hides" still holds: candidates[top_n:] move into the
    tail (already-sorted by score, so this is the lowest-ranked overflow),
    never dropped. Returns (visible, combined_tail, tail_note) where
    tail_note is a plain "N further alternatives" string, or None when
    nothing overflowed.
    """
    top_n = config["display_top_n"]["value"]
    visible = shortlist[:top_n]
    overflow = shortlist[top_n:]
    combined_tail = overflow + tail
    tail_note = None
    if overflow:
        tail_note = f"{len(overflow)} further alternative{'s' if len(overflow) != 1 else ''}"
    return visible, combined_tail, tail_note


# --- DECISION LAYER SPEC Phase D: top-line rendering (2026-09-22) -------
#
# D6 STOP raised during Phase D UI work and resolved by the reviewer
# (thesis_notes.md "Phase D D6 top-line rendering rule", 2026-09-22): a
# top line renders a magnitude ONLY when the action carries a real delta
# AND the registry defines a linear unit for the lever (value_space type
# int/float with a unit) -- never for an enum lever (springs_front/rear,
# wing_position, arb_front_mount), where the +-1 some generators attach is
# a routing SIGN (see _bridge_candidates_for_levers' own comment above),
# not a physical step. Otherwise the line is direction-only, in the
# lever's own registry vocabulary -- correct output, not a degraded
# fallback: TC/ABS's own Stage-1 vocabulary is direction-only by design.
#
# Phrase sourcing: symmetric registry words (soften/stiffen, more/less
# negative or positive) render as their plain-English opposite. Lever-
# specific words are pulled from that lever's own direction_semantics text
# where one exists (tc_lat/tc_lon "MORE/LESS...TC intervention",
# diff_position "locking torque" -> "more/less locking", wing_position's
# enum options are literally named positions -> "higher/lower position",
# brake_bias's forward/rearward wording already established at the B7
# bridge). Every phrase here is existing registry/candidate vocabulary in
# plain English -- no new physical claim, no invented magnitude.

_SYMMETRIC_PHRASES = {
    "soften": "softer", "stiffen": "stiffer",
    "more_negative": "more negative", "less_negative": "less negative",
    "more_positive": "more positive", "less_positive": "less positive",
}

_LEVER_DIRECTION_PHRASES = {
    ("brake_bias", "more_rear"): "rearward",
    ("brake_bias", "more_front"): "forward",
    ("abs_position", "more_fa_stability"): "more front-axle stability",
    ("abs_position", "more_ra_stability"): "more rear-axle stability",
    ("tc_lat", "increase"): "more intervention",
    ("tc_lat", "decrease"): "less intervention",
    ("tc_lon", "increase"): "more intervention",
    ("tc_lon", "decrease"): "less intervention",
    ("diff_position", "increase"): "more locking",
    ("diff_position", "decrease"): "less locking",
    ("wing_position", "increase"): "higher position",
    ("wing_position", "decrease"): "lower position",
    ("ride_height_front", "decrease"): "lower to the ground",
    ("ride_height_front", "increase"): "higher off the ground",
}


def _direction_phrase(parameter, direction):
    return (_LEVER_DIRECTION_PHRASES.get((parameter, direction))
            or _SYMMETRIC_PHRASES.get(direction)
            or direction.replace("_", " "))


def render_action_line(action, registry):
    """One action's own top-line fragment -- see module comment above for
    the D6 magnitude-vs-direction-only rule."""
    param = action["parameter"]
    entry = registry.get(param, {})
    label = entry.get("label", param)
    delta = action.get("delta")
    value_space = entry.get("value_space") or {}
    unit = value_space.get("unit")
    is_linear = value_space.get("type") in ("int", "float")

    if delta and is_linear and unit:
        sign = "+" if delta > 0 else "-"
        line = f"{label} {sign}{abs(delta)} {unit}"
        if param == "brake_bias":
            line += f" {_direction_phrase(param, action['direction'])}"
        return line
    return f"{label}: {_direction_phrase(param, action['direction'])}"


def render_top_line(candidate, registry):
    """Stage 6 output rule: 'top line = THE CHANGE ONLY' -- no corner id,
    no verdict, no provenance. A candidate with no routed action (the
    brake_balance_unrouted case) still names no corner on this line; the
    corner lives in the dropdown like everything else."""
    actions = candidate.get("actions")
    if not actions:
        return "Engineer attention: no routed action"
    return " + ".join(render_action_line(a, registry) for a in actions)


# --- DECISION LAYER SPEC Phase D: row severity colour (D1) --------------
#
# "Severity via existing row colour" -- reuses modules.recommendation's
# own SEVERITY_RANK vocabulary (strong/moderate/normal), the same mapping
# ui/views/outing_form.py already applies to stability-card severity
# (colour_map = {"strong": BAD, "moderate": WARN, "normal": OK}). No new
# colour convention invented here -- ui/ applies that existing map to this
# function's string result, falling back to NEUTRAL (the existing "no
# data" colour) when nothing backs a severity.

def candidate_severity(candidate):
    """First severity-bearing evidence_ref backing this candidate, or None
    when nothing in evidence_refs carries one (e.g. intervention-only or
    feedback-only evidence)."""
    for e in candidate.get("evidence_refs", []):
        sev = e.get("severity")
        if sev is not None:
            return sev
    return None


# --- DECISION LAYER SPEC Phase D: tail row rendering (D2) ---------------

def render_tail_line(entry, registry):
    """One line for the collapsed 'assessed, not proposed' tail (Stage 6):
    a no_trigger synthetic row names its lever only; a real non-proposed
    candidate reuses render_top_line for the change text and appends its
    own status reason -- never a second, disagreeing wording for the same
    change."""
    status = entry.get("status")
    if status == STATUS_NO_TRIGGER:
        lever = entry["lever"]
        label = registry.get(lever, {}).get("label", lever)
        return f"{label}: no trigger this session"

    line = render_top_line(entry, registry)
    if status == STATUS_BLOCKED_AT_EDGE:
        return f"{line} -- BLOCKED ({entry.get('edge_reason', 'at edge')})"
    if status == STATUS_CONTRADICTED:
        reasons = entry.get("condition_reasons") or ["contradicted"]
        return f"{line} -- {reasons[0]}"
    if status == STATUS_NOT_ASSESSABLE:
        reasons = entry.get("condition_reasons")
        reason = reasons[0] if reasons else entry.get("edge_reason", "not assessable")
        return f"{line} -- not assessable ({reason})"
    if status == STATUS_PROPOSED:
        return f"{line} -- below display threshold (score {entry.get('score', 0):.2f})"
    return line


# --- DECISION LAYER SPEC Stage 1: tyre-pressure check-only flags (D3) ---
#
# "Deviation flags shown beside shortlist, never in it." No per-session
# pressure-vs-target evidence source exists yet: TPMS channels are real
# (thesis_notes.md channel-census correction, 2026-09-22) but no target
# pressure is elicited anywhere in this repo, so tyre_pressure_target
# stays all-null. This returns [] honestly rather than fabricating a
# check -- wire a real evidence builder here once both a target and a
# per-corner cornering-phase pressure source exist.

_TPMS_CORNERING_PHASES = ("entry_2_turnin", "apex_3", "exit_4", "exit_5")
# WP-ELICIT Phase B2 (2026-09-24): DECISION LAYER SPEC Stage 1's own rule --
# "evaluated on CORNERING-PHASE samples only, straight-line pressure drop is
# physics, never a flag". entry_1_brake is excluded (braking zone, not yet
# turned in) -- same four phases used for the channel-census verification
# this activation is based on (diagnostics/inspect_tpms_pressure_cornering_
# phase.py, thesis_notes.md).

_TPMS_WHEEL_LABELS = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}


def _tpms_cornering_median(channels, corners, state, wheel):
    # Reuses _phase_window_indices (same searchsorted mechanism modules.
    # stability_analysis.summarise_corners's own private _phase_slice uses)
    # and the _log_abs_pos_at/_build_intervention_abs_evidence dead-channel
    # guard (ch.get("quality") in ("missing", "failed")) -- no new pattern.
    # Returns (median_bar, glitch_count) or (None, 0) if not evaluable.
    # glitch_count is the number of in-window samples excluded because they
    # fell outside this channel's OWN config/channels.json range -- reused
    # directly, not a second hardcoded bound, so this can never drift from
    # the range that already governs the channel's whole-session quality
    # flag. A single such sample cannot meaningfully move a median (the
    # real-data census behind this activation found zero such samples in
    # either real session's cornering-phase windows) -- excluded anyway,
    # per the reviewer's explicit requirement, and reported when nonzero.
    ch = (channels or {}).get(f"tpms_press_{wheel}")
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None or state is None:
        return None, 0
    t = state["time"]
    interp = np.interp(t, ch["time"], ch["data"])
    samples = []
    for c in corners or []:
        segments = c.get("segments", {})
        for phase in _TPMS_CORNERING_PHASES:
            idx = _phase_window_indices(t, segments.get(phase))
            if idx is None:
                continue
            lo, hi = idx
            samples.append(interp[lo:hi])
    if not samples:
        return None, 0
    vals = np.concatenate(samples)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return None, 0
    lo_range, hi_range = load_channels_config()["channels"][f"tpms_press_{wheel}"]["range"]
    in_range = (vals >= lo_range) & (vals <= hi_range)
    glitch_count = int((~in_range).sum())
    sane = vals[in_range]
    if sane.size == 0:
        return None, glitch_count
    return float(np.median(sane)), glitch_count


def tyre_pressure_flags(config, channels=None, corners=None, state=None):
    """DECISION LAYER SPEC Stage 1, activated WP-ELICIT Phase B2
    (2026-09-24): check-only, display-beside-shortlist ONLY -- no
    candidate, no score, no evidence_refs, never consumed by any
    generator (Stage 1's own "written exclusion": tyre pressures are
    NEVER recommended). Per wheel: session-level median over cornering-
    phase samples only, compared against tyre_pressure_target's band.

    Null target for a wheel -> fully silent for that wheel (unchanged
    honesty posture: no target exists, nothing to check). A FILLED
    target whose channel is dead/missing is NOT silent -- it reports
    "pressure not evaluable" rather than saying nothing, since a real
    target exists and the engineer should know it could not be checked.
    """
    targets = config.get("tyre_pressure_target", {})
    flags = []
    for wheel in ("fl", "fr", "rl", "rr"):
        target = targets.get(wheel)
        if not isinstance(target, dict) or target.get("min_bar") is None or target.get("max_bar") is None:
            continue
        label = _TPMS_WHEEL_LABELS[wheel]
        median, glitch_count = _tpms_cornering_median(channels, corners, state, wheel)
        if median is None:
            flags.append(f"{label}: pressure not evaluable (channel dead/missing)")
            continue
        lo, hi = target["min_bar"], target["max_bar"]
        if lo <= median <= hi:
            continue
        direction = "under" if median < lo else "over"
        line = f"{label} {median:.2f} — {direction} target {lo:.2f}-{hi:.2f}"
        compound_note = targets.get("compound_note")
        if compound_note:
            line += f" [compound: {compound_note}]"
        if glitch_count:
            line += f" ({glitch_count} glitch sample{'s' if glitch_count != 1 else ''} excluded)"
        flags.append(line)
    return flags


# --- Phase D feedback round, ITEM 1 (2026-09-23): display-layer grouping -
#
# Reviewer+user decision, user visual check on v3: identical (parameter-
# set, direction, rendered magnitude) rows collapse into ONE display row.
# The grouping key IS render_top_line's own output, not a separately
# re-derived tuple of (parameters, directions, deltas) -- render_top_line
# already folds parameter-set, direction and magnitude into one
# deterministic string (that is what D6 built it to do), so a second key
# definition would risk disagreeing with it over time. Group score = MAX
# member score, never summed -- Stage 6 severity is a property of each
# problem instance, not additive just because the same lever happens to
# also help another corner (that is what the breadth term already scores
# separately). The candidate model itself is untouched: this groups
# generate_display_split's own output for DISPLAY only, strictly after
# scoring/status/conflict-resolution has already run on the ungrouped
# list -- ui/ calls this once for the shortlist and once for the tail.
#
# Two candidate shapes are deliberately EXCLUDED from grouping, kept one
# row each regardless of a matching render_top_line string:
# - no_trigger synthetic rows (STATUS_NO_TRIGGER): render_top_line has no
#   lever-specific branch for these (they carry no "actions"), so every
#   no_trigger row would otherwise render the same generic "no routed
#   action" fallback and collapse into one meaningless group. Each is
#   already unique per lever by construction (generate_lever_inventory),
#   so keying on ("no_trigger", lever) is a correctness guard, not
#   expected to ever merge anything.
# - real candidates with NO routed action (the brake_balance_unrouted
#   case): these have no parameter-set to match on at all, so "identical
#   parameter-set" cannot apply by definition; two distinct corners each
#   needing unrouted engineer attention must stay two distinct flags, not
#   fold into one that loses which corner it was.

def _group_key(candidate, registry):
    if candidate.get("status") == STATUS_NO_TRIGGER:
        return ("no_trigger", candidate.get("lever"))
    if not candidate.get("actions"):
        return ("unrouted", id(candidate))
    return render_top_line(candidate, registry)


def group_display_rows(rows, registry):
    """Collapses rows whose _group_key matches into one display row,
    carrying every member under "group_members" (dropdown reads this to
    render one reasoning block per contributing corner+phase, reusing
    _build_decision_frame_detail_host per member unchanged). The
    representative row's own fields (score, status, actions, corner,
    phase, ...) are a shallow copy of whichever member scored highest --
    a single, consistent rule, not two disagreeing ones for "which score"
    versus "which corner/status to show up front." Order preserved:
    first occurrence of each key keeps its position, single-member groups
    pass through unchanged (not even given a "group_members" key), so
    every existing caller that never groups anything sees byte-identical
    output.
    """
    grouped, order = {}, []
    for c in rows:
        key = _group_key(c, registry)
        if key not in grouped:
            grouped[key] = []
            order.append(key)
        grouped[key].append(c)

    result = []
    for key in order:
        members = grouped[key]
        if len(members) == 1:
            result.append(members[0])
            continue
        best = max(members, key=lambda m: m.get("score", float("-inf")))
        merged = dict(best)
        merged["group_members"] = members
        result.append(merged)
    return result


# --- Conflict resolver, Stage 2 (Phase 3b) -------------------------------

def resolve_conflicts(shortlist):
    """Compound-problem conflict resolver, per the work order: a corner
    with two or more candidates recommending DIFFERENT directions/targets
    for the SAME registry parameter (same detection modules.recommendation.
    _apply_parameter_conflicts already uses for the old engine's own final
    results, generalised here across this frame's own candidate set) is
    resolved in two steps:

    1. PLATFORM-CALMING -- search this corner's own full candidate set
    (not just the conflicting ones) for one whose evidence_refs already
    cover EVERY verdict involved in the conflict (a single lever serving
    both problems at once). If found, it is preferred; every conflicting
    candidate is annotated, never silently dropped (transparency over
    suppression, the same project-wide principle build_evidence's own
    docstring states) -- 'platform_calming_available' on the winner,
    'superseded_by_platform_calming' on the rest.

    2. TIME-LOSS -- else, the candidate anchored to the higher phase-
    importance weight wins (config/decision_frame.json cost_function.
    phase_importance, reused directly -- exit > entry per the user's own
    elicited ordering, never a second copy of the same numbers; DECISION
    LAYER SPEC C1 moved this key from scoring_weights, now retired, into
    cost_function, same values). Ties
    break on the already-computed score. Annotated 'wins_time_loss' /
    'superseded_by_time_loss'.

    Mutates and returns `shortlist` in place (adds 'conflict_status',
    'conflict_with', and, for a superseded candidate, a human-readable
    'conflict_resolution_note') -- candidates outside any conflict get
    conflict_status=None, conflict_with=[]. Never removes a candidate from
    the list; ranking/display decisions based on conflict_status are the
    caller's own choice (same "surfaced, never netted/averaged" posture
    the old engine's own _apply_parameter_conflicts states).
    """
    weights = load_decision_frame_config()["cost_function"]["phase_importance"]

    for c in shortlist:
        c["conflict_status"] = None
        c["conflict_with"] = []

    by_corner = {}
    for c in shortlist:
        by_corner.setdefault(c["corner"], []).append(c)

    for cid, group in by_corner.items():
        param_to_entries = {}
        for c in group:
            for a in c["actions"]:
                param_to_entries.setdefault(a["parameter"], []).append((c, _action_key(a)))

        conflicting_ids = set()
        for param, entries in param_to_entries.items():
            if len({key for _, key in entries}) > 1:
                conflicting_ids.update(c["id"] for c, _ in entries)
        if not conflicting_ids:
            continue

        conflicting_candidates = [c for c in group if c["id"] in conflicting_ids]
        for c in conflicting_candidates:
            c["conflict_with"] = sorted({o["id"] for o in conflicting_candidates if o["id"] != c["id"]})

        conflicting_verdicts = {
            e["verdict"] for c in conflicting_candidates for e in c["evidence_refs"] if e.get("verdict")
        }

        platform_calming = None
        if len(conflicting_verdicts) > 1:
            for c in group:
                own_verdicts = {e["verdict"] for e in c["evidence_refs"] if e.get("verdict")}
                if conflicting_verdicts <= own_verdicts:
                    platform_calming = c
                    break

        if platform_calming is not None:
            for c in conflicting_candidates:
                if c["id"] != platform_calming["id"]:
                    c["conflict_status"] = "superseded_by_platform_calming"
                    c["conflict_resolution_note"] = (
                        f"Candidate '{platform_calming['id']}' addresses both conflicting "
                        f"problems at C{cid} at once -- preferred over a single-purpose lever."
                    )
            platform_calming["conflict_status"] = "platform_calming_available"
            continue

        def _time_loss(c):
            phases = c.get("phases") or (c["phase"],)
            return max(weights.get(p, 1.0) for p in phases)

        ranked = sorted(conflicting_candidates, key=lambda c: (-_time_loss(c), -c["score"]))
        winner = ranked[0]
        winner["conflict_status"] = "wins_time_loss"
        for c in ranked[1:]:
            c["conflict_status"] = "superseded_by_time_loss"
            c["conflict_resolution_note"] = (
                f"Superseded by '{winner['id']}' at C{cid} (higher phase-importance weight "
                f"{_time_loss(winner):.2f} vs {_time_loss(c):.2f})."
            )

    return shortlist
