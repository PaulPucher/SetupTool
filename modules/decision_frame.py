# Decision-matrix frame, Stage 1 (2026-09-02). Three-layer recommendation
# frame (evidence -> candidates -> scoring), additive and parallel to the
# existing 39-rule engine in modules/recommendation.py, which is unchanged
# and remains the production path. Pure Python, no Qt.
#
# Stage 1 scope, per the work order: one fully worked scenario (exit
# oversteer, LS-disambiguated) end to end, plus the two cheap-first
# plausibility checks. config/decision_frame.json documents everything
# else as seeded/null and listed for the user -- this module only acts on
# what that config actually populates.

import json

import numpy as np

from modules.stability_analysis import load_parameters
from modules.recommendation import (
    PHASE_KEYS,
    SEVERITY_RANK,
    _axle_verdict,
    _current_setup_value,
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


def _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn):
    # Source (a): corner verdicts via the existing classify path -- the
    # same worst-lap aggregate and anchored thresholds the stability grid
    # already shows, so this evidence can never disagree with the UI for
    # the same corner/phase (identical mechanism modules.recommendation
    # relies on for the same reason).
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

                evidence.append({
                    "type": "corner_verdict",
                    "corner": cid,
                    "phase": phase,
                    "speed_class": corner.get("speed_class"),
                    "verdict": verdict,
                    "severity": severity,
                    "confidence": confidence,
                    "source": f"classify_fn (worst-lap aggregate, anchored thresholds): "
                              f"C{cid} {phase} '{short}' -- repeats on {repeat}/{total} laps, "
                              f"signal present on {valid_laps}/{total} laps",
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


def build_evidence(summaries, ls_stats, config, classify_fn):
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
    """
    aggregated = aggregate_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)

    evidence = []
    evidence += _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn)
    evidence += _build_ls_disambiguation_evidence(
        aggregated, ls_stats, [e for e in evidence if e["type"] == "corner_verdict"]
    )
    evidence += _build_brake_balance_evidence(aggregated, by_corner_laps, config)
    return evidence


# --- Candidate layer, Stage 1 -----------------------------------------
#
# Scope, per the work order: the exit-oversteer scenario ONLY, both LS
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


def _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs):
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
                "actions": [{"parameter": "springs_rear", "direction": "soften"}],
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
                "evidence_refs": evidence_refs,
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
                "evidence_refs": evidence_refs,
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


def generate_candidates(evidence, registry, config):
    """Candidate layer, Stage 1. See module-level comment above for scope.
    `registry` is modules.recommendation.load_setup_parameters_registry()'s
    own dict; `config` is load_decision_frame_config()'s dict (accepted for
    signature symmetry with build_evidence/score -- Stage 1's candidate
    generation itself reads config/recommendations.json directly for
    matrix-cell lookups, not config/decision_frame.json, since that is
    where the actual lever/rationale/provenance data lives).
    """
    config_recs = load_recommendations_config()

    corner_verdicts_by_key = {}
    ls_by_key = {}
    for e in evidence:
        if e["type"] == "corner_verdict":
            corner_verdicts_by_key.setdefault((e["corner"], e["phase"]), []).append(e)
        elif e["type"] == "ls_disambiguation":
            ls_by_key[(e["corner"], e["phase"])] = e

    candidates = []
    candidates += _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs)
    candidates += _brake_balance_candidates(evidence, registry, config_recs)
    return candidates


# --- Scoring layer, Stage 1 --------------------------------------------
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
        new_value = float(current) + action["delta"]
        distances.append(min(1.0, abs(new_value - nominal) / span))
    if not distances:
        return 0.0, flags
    # Worst (max) distance across a package's own actions -- same
    # conservative "max across the package" convention modules.
    # recommendation._rank_key already uses for escalation tier.
    return weight * (1.0 - max(distances)), flags


def _interaction_penalty(candidate, evidence, decision_config, weight):
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
