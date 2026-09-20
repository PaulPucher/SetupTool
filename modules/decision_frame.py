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


# --- Driver-feedback evidence (Deepening Phase 4d, 2026-09-18) ---------
#
# A genuinely new evidence dimension -- Stage 1/2 had none ("this frame
# has no feedback/driver-trigger axis at all yet", the Stage 1 close-out's
# own recorded open item). A driver_feedback item never generates its own
# candidate; it corroborates an EXISTING candidate's evidence_refs (same
# MIN-confidence rule every other corroborating source already uses) when
# its own corner/phase/verdict agrees, via _attach_feedback_evidence.

def _build_driver_feedback_evidence(feedback_data, aggregated, feedback_cfg):
    if not feedback_data:
        return []
    floor = feedback_cfg.get("confidence_floor", 0.1)
    full_at = feedback_cfg.get("full_confidence_at_raw_abs", 4)

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
            ramp = min(1.0, max(0.0, (magnitude - 1.0) / (full_at - 1.0))) if full_at > 1 else 1.0
            confidence = round(floor + (1.0 - floor) * ramp, 3)
            evidence.append({
                "type": "driver_feedback",
                "corner": cid, "phase": phase,
                "speed_class": corner.get("speed_class"),
                "verdict": "oversteer" if raw > 0 else "understeer",
                "severity": None, "confidence": confidence, "raw_feedback": raw,
                "source": f"driver feedback {raw:+g} at {phase} (magnitude {magnitude:g}; confidence ramps "
                          f"{floor} at |1| to 1.0 at |{full_at}| -- Deepening Phase 4d, user decision "
                          f"'counts from 1, very little weight')",
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

# Enum structure (a fixed ordering of setup_parameters.json's own
# change_effort vocabulary), not a per-car tunable -- CLAUDE.md
# method-defining-constant guidance, same status as modules.recommendation.
# SEVERITY_RANK/ESCALATION_TIER_RANK.
EFFORT_RANK = {"seconds": 0, "minutes": 1, "garage_hours": 2}

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
                             "not a substitute recommendation on equal footing.",
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

def _bridge_candidates_for_levers(evidence, registry, decision_config, existing_candidates):
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
    """
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
        # Method-defining, not a car tunable: +1/-1 encodes ONE step in this
        # lever's own direction_semantics (config/setup_parameters.json
        # springs_front/rear both declare increasing="stiffer"), matching the
        # ARB actions' existing delta convention -- not a physical magnitude.
        delta = 1 if direction == "stiffen" else -1

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
                        "evidence_refs": [ev],
                        "rationale": bridge["rationale"],
                        "derived_from": bridge["derived_from"],
                    })
    return candidates


def generate_candidates(evidence, registry, config):
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
    candidates += _bridge_candidates_for_matrix_rules(evidence, registry, config_recs, intervention_abs_by_corner,
                                                        intervention_abs_masking_by_corner)
    candidates += _bridge_candidates_for_levers(evidence, registry, config, candidates)
    # Deepening Phase 4d: attaches any driver_feedback evidence (built by
    # build_evidence when feedback_data was supplied) to every candidate
    # whose own evidence_refs share its corner/phase/verdict -- corroborates
    # via the existing MIN-confidence rule, never a new candidate/action.
    candidates = _attach_feedback_evidence(candidates, evidence)
    return candidates


# --- Scoring layer (shared by every candidate, Stage 1 and Stage 2) ----
#
# Six components exactly, per the work order: severity x phase_importance,
# effect_class, inverse effort, confidence, settings-window distance,
# interaction penalty. Every weight lives in config/decision_frame.json's
# scoring_weights (all project-lead-elicited placeholders, Stage 2
# calibration item). Deterministic: no randomness, no hidden global state
# -- identical (candidate, evidence, current_setup, config) always
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


def score(candidate, evidence, current_setup, config):
    """Scoring layer, Stage 1. Six components (see module comment above),
    each weighted by config['scoring_weights'], summed to one scalar with
    the full breakdown retained for Phase 5's expandable UI reasoning.

    `evidence` is build_evidence()'s own full list (needed by the
    interaction-penalty component to see this corner's OTHER active
    problems). `current_setup` is the outing's setup-sheet dict (same
    shape modules.recommendation.generate_recommendations' setup_data
    parameter takes). `config` is load_decision_frame_config()'s dict.
    """
    weights = config["scoring_weights"]
    registry = load_setup_parameters_registry()
    flags = []

    primary = candidate["evidence_refs"][0] if candidate["evidence_refs"] else None
    sev_rank = SEVERITY_RANK.get(primary.get("severity") if primary else None, 0)
    phase_importance = weights["phase_importance"].get(candidate["phase"], 1.0)
    c_severity = weights["severity_weight"] * sev_rank * phase_importance

    if candidate["effect_class"] is None:
        c_effect = 0.0
        flags.append("effect_class unset (unrouted candidate)")
    else:
        c_effect = weights["effect_weight"] * weights["effect_class_multiplier"].get(
            candidate["effect_class"], 0.0)

    if candidate["effort_class"] is None:
        c_effort = 0.0
        flags.append("effort_class unset (unrouted candidate)")
    else:
        c_effort = weights["effort_weight"] / (EFFORT_RANK.get(candidate["effort_class"], 0) + 1)

    c_confidence = weights["confidence_weight"] * _candidate_confidence(candidate)

    c_window, window_flags = _settings_window_component(
        candidate, current_setup, registry, config, weights["settings_window_weight"])
    flags += window_flags

    c_interaction, interaction_notes = _interaction_penalty(
        candidate, evidence, config, weights["interaction_weight"])

    total = c_severity + c_effect + c_effort + c_confidence + c_window + c_interaction

    return {
        "total": round(total, 4),
        "components": {
            "severity_x_phase_importance": round(c_severity, 4),
            "effect_class": round(c_effect, 4),
            "inverse_effort": round(c_effort, 4),
            "confidence": round(c_confidence, 4),
            "settings_window_distance": round(c_window, 4),
            "interaction_penalty": round(c_interaction, 4),
        },
        "interaction_notes": interaction_notes,
        "flags": flags,
    }


def generate_shortlist(candidates, evidence, current_setup, config):
    """Ranked shortlist: every candidate scored via score() above, sorted
    by total score descending. Deterministic tie-break on candidate id
    (lexical) so identical inputs always produce an identical ordering,
    even when two candidates score exactly equal.
    """
    shortlist = []
    for c in candidates:
        result = score(c, evidence, current_setup, config)
        shortlist.append({
            **c,
            "score": result["total"],
            "score_components": result["components"],
            "score_interaction_notes": result["interaction_notes"],
            "score_flags": result["flags"],
        })
    shortlist.sort(key=lambda c: (-c["score"], c["id"]))
    return shortlist


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
    importance weight wins (config/decision_frame.json scoring_weights.
    phase_importance, reused directly -- exit > entry per the user's own
    elicited ordering, never a second copy of the same numbers). Ties
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
    weights = load_decision_frame_config()["scoring_weights"]["phase_importance"]

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
