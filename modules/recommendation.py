# Recommendation engine framework for SetupTool.
# Pure Python. No Qt imports. Converts per-lap-per-corner stability
# summaries + driver feedback + a rule table (config/recommendations.json)
# into a ranked, evidence-backed list of setup direction suggestions.
#
# WP2b-2: rules are now sourced from an external engineer decision matrix
# (scenario x speed-class grid, config/recommendations.json rule.cell_id),
# referencing real config/setup_parameters.json registry keys instead of
# the WP2 placeholder front_arb/rear_arb labels. Mild understeer is this
# car's deliberate stable baseline (June driver-report precedent) and the
# elicitation's own bias is against unnecessary changes when the driver is
# inconsistent with the data -- see the action_class split below, which
# keeps unsubstantiated moderate-severity data-only matches as ADVISORY
# (observation, non-imperative, never budget-eligible) rather than
# RECOMMENDED (ranked, budget-eligible).

import json
import numpy as np

RECOMMENDATIONS_CONFIG_PATH = "config/recommendations.json"
SETUP_PARAMETERS_CONFIG_PATH = "config/setup_parameters.json"

# Must match the phase_keys list in modules/stability_analysis.py
# summarise_corners(), and the e1..x5 feedback columns collected in
# ui/views/outing_form.py's feedback table (positional pairing below).
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
PHASE_TO_FEEDBACK_KEY = dict(zip(PHASE_KEYS, ["e1", "e2", "a3", "x4", "x5"]))

# Ordinal ordering of severity, not a magnitude -- defines the enum's
# structure, so it stays a named constant rather than a config value.
SEVERITY_RANK = {"normal": 0, "moderate": 1, "strong": 2}

# Canonical driver-feedback encoding (project-lead + reviewer decision,
# 2026-07-27; recorded scale definition: ui/views/outing_form.py's own
# feedback-table caption, "-5 undrivable understeer ... +5 undrivable
# oversteer" -- signed-bipolar, negative=understeer, positive=oversteer,
# |4..5|="approaching undrivable"/"undrivable"). Every rule's condition.
# verdict <-> condition.feedback_sign pairing must agree with this map --
# verified 2026-07-27 against the full ruleset (all 26 non-retired rules,
# plus the 7 retired seeds): every existing rule already agrees, no rule
# needed changing. This map is also what the consistency-gate feedback
# override (_consistency_gate_ok) uses to decide which SIGN of feedback
# corroborates which verdict; unstable_yaw (and any other verdict not
# listed) has no feedback-sign axis at all (see _feedback_modulation's own
# comment) and the override never applies to it.
VERDICT_EXPECTED_FEEDBACK_SIGN = {"understeer": "negative", "oversteer": "positive"}

# Ordinal ordering of a corner's speed_class (modules/corner_analysis.py,
# config/channels.json corner_speed_thresholds) -- enum structure, not a
# per-car tunable.
SPEED_CLASS_ORDER = ["low", "medium", "high"]

# Escalation-order tier from the decision matrix (cockpit -> pitlane ->
# garage): a distinct axis from setup_parameters.json's change_effort
# (time-to-change) -- e.g. diff_position is change_effort "seconds" but
# matrix-"garage" (driver-preference domain). Enum structure, not tunable.
ESCALATION_TIER_RANK = {"cockpit": 0, "pitlane": 1, "garage": 2}

# Method-defining constants (CLAUDE.md grounding rule): these fix the shape
# of the scoring formula, not a per-car/per-track calibration.
SOURCE_BALANCE_NORMALISER = 2.0  # makes source_balance=0.5 exactly neutral (both multipliers = 1.0)
FEEDBACK_SCALE_MAX = 5.0  # driver feedback is entered on a fixed -5..+5 scale

# Rule statuses that never fire. "retired": superseded (old ARB-only seeds).
# "held": escalation rule, fully specified for 1:1 cell traceability but not
# yet automated (no applied-recommendations history to know the base change
# was tried). "dropped": matrix cell deliberately defines no action.
_NON_FIRING_STATUSES = ("retired", "held", "dropped")


def load_recommendations_config():
    with open(RECOMMENDATIONS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def load_setup_parameters_registry():
    with open(SETUP_PARAMETERS_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {k: v for k, v in data.items() if not k.startswith("_comment")}


def _nanmedian_or_nan(values):
    valid = [v for v in values if v == v]  # drop NaN (NaN != NaN)
    if not valid:
        return float("nan")
    return float(np.median(valid))


def _nanmin_or_nan(values):
    valid = [v for v in values if v == v]  # drop NaN (NaN != NaN)
    if not valid:
        return float("nan")
    return float(np.min(valid))


def _group_by_corner(summaries):
    by_id = {}
    for s in summaries:
        cid = s.get("stable_corner_id")
        if cid is None:
            continue
        by_id.setdefault(cid, []).append(s)
    return by_id


def _aggregate_speed_class(lap_summaries):
    # Tier B (standard aggregation choice, config-documented -- CLAUDE.md
    # grounding rule): modal speed_class across the corner's laps, so a
    # corner sitting near a config/channels.json threshold doesn't flip a
    # rule's speed-class gate lap to lap. Tie-break prefers the class
    # closest to "medium"; a residual tie (e.g. a low/high split with no
    # medium instance at all) breaks toward the lower class -- deterministic,
    # not a physically-loaded choice.
    counts = {}
    for s in lap_summaries:
        sc = s.get("speed_class")
        if sc is None:
            continue
        counts[sc] = counts.get(sc, 0) + 1
    if not counts:
        return None
    max_n = max(counts.values())
    tied = [c for c, n in counts.items() if n == max_n]
    mid = SPEED_CLASS_ORDER.index("medium")
    tied.sort(key=lambda c: (abs(SPEED_CLASS_ORDER.index(c) - mid), SPEED_CLASS_ORDER.index(c)))
    return tied[0]


def aggregate_by_corner(summaries):
    # Cross-lap combiner for CS_ratio is config-driven (classification.
    # cs_cross_lap_aggregation, "median"|"worst_lap"): "median" privileges
    # behaviour that repeats every lap, washing out a one-off anomaly.
    # "worst_lap" (min-then-min, thesis_notes.md "Gated Stage-2
    # recomputation...", signed off as Stage 3) takes the worst phase of
    # the worst lap instead -- this is a per-phase min across laps here,
    # composed with classify_fn's own existing min-across-phases search
    # (_classify_corner) at the caller; since min is associative, that
    # composition IS the global min over every (lap, phase) pair, without
    # needing to flatten the two stages into one pass here. Ships only
    # together with thresholds re-derived against that same population
    # (CLAUDE.md deviation taxonomy: classification thresholds are always
    # re-derived for the population they will be read against). Stability
    # keeps median unconditionally -- its aggregation was never found
    # broken (PLAN.md "STEP 2"), only CS_ratio was. A single-lap CS anomaly
    # can therefore surface at the aggregate under "worst_lap" without
    # itself producing a recommendation: the consistency gate in
    # _evaluate_rule re-evaluates classify_fn per lap independently and
    # still requires the verdict to repeat across a minimum count/fraction
    # of that corner's laps before any rule fires.
    from modules.stability_analysis import load_parameters
    cs_aggregation = load_parameters()["classification"].get(
        "cs_cross_lap_aggregation", "median"
    )
    cs_combine = _nanmin_or_nan if cs_aggregation == "worst_lap" else _nanmedian_or_nan

    by_id = _group_by_corner(summaries)

    aggregated = {}
    for cid, corner_summaries in by_id.items():
        phases = {}
        for phase in PHASE_KEYS:
            csf, csr, stab = [], [], []
            for s in corner_summaries:
                p = s["phases"].get(phase)
                if p is None:
                    continue
                csf.append(p["cs_ratio_f"]["median"])
                csr.append(p["cs_ratio_r"]["median"])
                stab.append(p["stability_observed_Nm_per_deg"]["median"])
            phases[phase] = {
                "cs_ratio_f": {"median": cs_combine(csf)},
                "cs_ratio_r": {"median": cs_combine(csr)},
                "stability_observed_Nm_per_deg": {"median": _nanmedian_or_nan(stab)},
            }
        # CS validity repair part A, Phase 3: same cross-lap combiner for
        # apex_region too, mirroring the phases loop above -- classify_fn
        # (_classify_corner) reads this for apex_3-keyed CS, not phases[
        # "apex_3"] itself. Older summaries predating ANALYSIS_SCHEMA_VERSION
        # 8 have no "apex_region" key; s.get(...) then None is filtered out,
        # same NaN-on-empty behaviour as every other aggregated stat here.
        ar_csf = [s["apex_region"]["cs_ratio_f"]["median"] for s in corner_summaries if s.get("apex_region")]
        ar_csr = [s["apex_region"]["cs_ratio_r"]["median"] for s in corner_summaries if s.get("apex_region")]
        aggregated[cid] = {
            "stable_corner_id": cid,
            "n_laps": len(corner_summaries),
            "speed_class": _aggregate_speed_class(corner_summaries),
            "phases": phases,
            "apex_region": {
                "cs_ratio_f": {"median": cs_combine(ar_csf)},
                "cs_ratio_r": {"median": cs_combine(ar_csr)},
            },
        }
    return aggregated


def _phase_verdict(aggregated_corner, phases, classify_fn):
    # Slices the aggregate summary down to the rule's own phases and
    # reuses classify_fn (the UI's _classify_corner) unmodified. This is
    # what guarantees a recommendation can never disagree with the
    # verdict the stability grid shows for the same corner and phase --
    # both are the identical classifier, not two independent judgments.
    # apex_region rides along whenever the rule's phases include apex_3 --
    # classify_fn reads it for that phase's CS comparison instead of
    # apex_3's own slice (CS validity repair part A, Phase 3); omitted
    # otherwise so classify_fn's own apex_region-absent path (older,
    # pre-bump summaries) is exercised identically to before.
    sliced = {p: aggregated_corner["phases"][p]
              for p in phases if p in aggregated_corner["phases"]}
    call_arg = {"phases": sliced}
    if "apex_3" in phases and aggregated_corner.get("apex_region") is not None:
        call_arg["apex_region"] = aggregated_corner["apex_region"]
    severity, short, _long, _colour = classify_fn(call_arg)
    return severity, short


def _axle_verdict(short):
    # classify_fn's short verdict string carries at most one axle term
    # (its own primary/elif chain already picks a single dominant axle).
    if "understeer" in short:
        return "understeer"
    if "oversteer" in short:
        return "oversteer"
    return None


def _verdict_present(short, target):
    if target == "unstable_yaw":
        return "unstable yaw" in short
    return target in short


def _feedback_row(feedback_data, stable_corner_id):
    # Index-based mapping (feedback row i+1 <-> stable_corner_id i+1),
    # interim per WP3b in PLAN.md. Feedback has no lap dimension.
    if not feedback_data:
        return {}
    corners = feedback_data.get("corners", [])
    idx = stable_corner_id - 1
    if idx < 0 or idx >= len(corners):
        return {}
    return corners[idx]


def _feedback_value(feedback_row, phases):
    # Multi-phase rules use the max-|value| among their phases: the
    # strongest driver signal in the phases the rule cares about, not an
    # average that could wash out a sharp complaint.
    vals = [feedback_row.get(PHASE_TO_FEEDBACK_KEY[p], 0) for p in phases
            if p in PHASE_TO_FEEDBACK_KEY]
    if not vals:
        return 0
    return max(vals, key=abs)


def _feedback_modulation(fb_value, condition, settings):
    # "data"-triggered rules: phase-scoped feedback modulates the score
    # symmetrically. A rule whose condition omits feedback_sign (the yaw
    # rules -- no natural feedback axis) is never modulated, and can never
    # be reported as driver-corroborated.
    feedback_sign = condition.get("feedback_sign")
    if feedback_sign is None:
        return 1.0, False, False
    min_abs = condition.get("min_feedback_abs", 0)
    if abs(fb_value) < min_abs:
        return 1.0, False, False
    agrees = ((feedback_sign == "negative" and fb_value < 0)
              or (feedback_sign == "positive" and fb_value > 0))
    if agrees:
        return settings["agreement_bonus"], False, True
    return settings["conflict_penalty"], True, False


def _classifier_modulation(short, severity, agreement_ref, settings):
    # "driver"-triggered rules: the classifier verdict on the same phases
    # modulates the score instead. condition["verdict"] is only ever the
    # agreement reference here -- it never gates whether the rule fires.
    if severity == "normal":
        return 1.0, False
    axle = _axle_verdict(short)
    if axle is None:
        return 1.0, False
    if axle == agreement_ref:
        return settings["agreement_bonus"], False
    return settings["conflict_penalty"], True


def _resolve_source_balance(config, outing=None):
    # Single resolution point for settings.source_balance -- callers must
    # never read config["settings"]["source_balance"] directly. Today this
    # is just the global default; `outing` is accepted so call sites don't
    # need to change signature if a future per-outing override lands.
    # Per-driver weighting (the other half of the WP2b-2 note this
    # docstring used to point at) is now handled separately by
    # _resolve_feedback_weight below, not folded into source_balance --
    # the two are orthogonal (source_balance is data-vs-driver, this is
    # driver-vs-driver).
    return config["settings"]["source_balance"]


def _resolve_feedback_weight(config, driving_level):
    # PART A: config-resident driving_level -> feedback_weight mapping
    # (config/recommendations.json settings.driver_level_weighting).
    # driving_level is a plain int (Driver.driving_level, 1-10) or None --
    # resolved by the UI caller from Outing.driver_id, never read from a
    # live DB session here (modules/ stays a plain-value boundary, same
    # convention as the WP-C accuracy_cap). None or an out-of-table level
    # falls back to default_weight (1.0 -- today's unweighted behaviour).
    dlw = config["settings"].get("driver_level_weighting")
    if dlw is None or driving_level is None:
        return 1.0 if dlw is None else dlw.get("default_weight", 1.0)
    return dlw["weights"].get(str(driving_level), dlw.get("default_weight", 1.0))


def _override_direction_ok(verdict, raw_fb_value, raw_min):
    # Repair (2026-07-27): the override previously checked abs(raw_fb_value)
    # only -- a +5 (oversteer-direction) complaint could override an
    # UNDERSTEER rule's consistency gate, since magnitude alone doesn't
    # know which direction the rule actually wants. Direction now comes
    # from VERDICT_EXPECTED_FEEDBACK_SIGN: "understeer" needs raw <= -min
    # (negative AND at least raw_min in magnitude), "oversteer" needs
    # raw >= +min. A verdict with no sign axis (unstable_yaw, or any
    # future verdict not in the map) never qualifies -- there's no
    # direction to corroborate.
    expected_sign = VERDICT_EXPECTED_FEEDBACK_SIGN.get(verdict)
    if expected_sign == "negative":
        return raw_fb_value <= -raw_min
    if expected_sign == "positive":
        return raw_fb_value >= raw_min
    return False


def _consistency_gate_ok(cid, by_corner_laps, phases, verdict, min_severity, classify_fn, settings,
                          raw_fb_value=0.0, scaled_fb_value=0.0):
    # Global decision-matrix policy: no recommendation unless the
    # triggering verdict repeats across laps. Re-evaluates classify_fn
    # per lap (not just on the median-of-medians aggregate already tested
    # by the caller) and requires BOTH an absolute floor and a fraction of
    # this corner's analysed laps to show the verdict at/above min_severity.
    #
    # Feedback override (project-lead-elicited 2026-07-27, see config/
    # recommendations.json settings.consistency_gate.feedback_override and
    # thesis_notes.md): a strong, unprompted driver complaint on a corner
    # already showing a moderate+ data verdict is itself corroborating
    # evidence of repeatability -- a capable driver will not provoke the
    # same imbalance repeatedly just to make the data repeat. Fires only
    # when the feedback's DIRECTION matches the rule's own verdict
    # (_override_direction_ok, VERDICT_EXPECTED_FEEDBACK_SIGN -- repaired
    # 2026-07-27, previously magnitude-only: a +5 complaint could not have
    # overridden an understeer rule's gate before this fix, since abs()
    # doesn't see sign) AND the scaled (post-driver-level-weighting)
    # magnitude also clears its own floor. When both hold, a single
    # matching lap is sufficient (bypasses BOTH the absolute-laps floor and
    # the fraction check below, not just the laps floor in isolation --
    # with only 1 of typically 4 laps required, the 0.4 fraction default
    # would otherwise still reject it). raw_fb_value/scaled_fb_value
    # default to 0.0 (never overrides) for any caller that doesn't pass
    # them.
    gate = settings.get("consistency_gate")
    if not gate:
        return True
    laps = by_corner_laps.get(cid, [])
    if not laps:
        return False
    repeat = 0
    for lap_summary in laps:
        severity, short = _phase_verdict(lap_summary, phases, classify_fn)
        if _verdict_present(short, verdict) and SEVERITY_RANK[severity] >= SEVERITY_RANK[min_severity]:
            repeat += 1

    override = gate.get("feedback_override")
    if (override
            and _override_direction_ok(verdict, raw_fb_value, override["feedback_override_raw_min"])
            and abs(scaled_fb_value) >= override["feedback_override_scaled_min"]):
        return repeat >= 1

    return (repeat >= gate["min_repeat_laps"]) and (repeat / len(laps) >= gate["min_repeat_fraction"])


def _normalise_actions(suggestion):
    return suggestion if isinstance(suggestion, list) else [suggestion]


def _action_key(action):
    return action.get("direction") or f"target={action.get('target')}"


def _bucket_key(rule, actions):
    # A package (or an axle-symmetric FL+FR/RL+RR pair -- the registry has
    # no combined axle-level ARB/camber/damper key, see arb_fl notes in
    # setup_parameters.json) is applied and budgeted atomically: it gets
    # its own bucket, never merged with anything else, keyed by the rule's
    # own cell_id so two different packages never collide.
    if len(actions) == 1:
        a = actions[0]
        return (a["parameter"], _action_key(a))
    return ("__package__", rule.get("cell_id") or rule["id"])


def _provenance_note(rule, settings):
    # Fix turn (UI text humanization): screen text is now a short muted
    # suffix only -- provenance grade, source cell, and the ADVISORY cap
    # (already a separate header badge) are structured fields
    # (elicitation_provenance, cell_id, status), machine/thesis-side, not
    # screen-side. "project-lead-reviewed" is action-eligible, so it never
    # gets this suffix.
    ac = settings.get("action_class", {})
    eligible = set(ac.get("action_eligible_provenances", ["engineer-verbatim", "project-lead-reviewed"]))
    prov = rule.get("elicitation_provenance")
    if prov is None or prov in eligible or rule.get("status") == "reviewed":
        return None
    return " (engineer confirmation pending)"


def _match_is_recommended(match, rule, settings):
    # WP2b-2 amendment 7: which severity/trigger combos are action-eligible
    # is config, not hardcoded (settings["action_class"]).
    ac = settings.get("action_class", {})
    # Matrix v2: a "situational" rule (the matrix itself lists more than one
    # valid lever, grip-level-dependent, and declines to pick one) is
    # PERMANENTLY advisory -- an observation with alternatives listed in its
    # rationale, never budget-eligible, regardless of provenance grade,
    # severity, or corroboration.
    if rule.get("situational"):
        return False
    # WP2b-2 provenance cap, extended by matrix v2: only a cell stated
    # verbatim by the engineer OR project-lead-reviewed (a step below full
    # verbatim confirmation, but still action-eligible per the matrix v2
    # review) counts as recommended -- anything else (mirror-derived,
    # project-default) stays capped until that specific cell is confirmed
    # (status promoted to "reviewed"), a per-cell override independent of
    # the global policy switch.
    eligible = set(ac.get("action_eligible_provenances", ["engineer-verbatim", "project-lead-reviewed"]))
    prov = rule.get("elicitation_provenance")
    if (ac.get("cap_non_verbatim_to_advisory", True)
            and prov not in eligible
            and prov is not None
            and rule.get("status") != "reviewed"):
        return False
    always = set(ac.get("always_recommended_triggers", ["both", "driver"]))
    if match["trigger"] in always:
        return True
    rec_severities = set(ac.get("data_trigger_recommended_severities", ["strong"]))
    return (match["severity"] in rec_severities) or match["corroborated"]


def _worst_feedback(fb_row):
    # Undrivable-feedback tier: the corner's single strongest complaint
    # across ALL five phases (not a rule's own phase subset -- the tier
    # acts per corner, once, on whichever phase the driver actually rated
    # worst). Same max-|value| convention as _feedback_value. Returns
    # (phase, raw_value); (None, 0) if the row is empty/all-zero.
    if not fb_row:
        return None, 0
    best_phase, best_val = None, 0
    for phase in PHASE_KEYS:
        val = fb_row.get(PHASE_TO_FEEDBACK_KEY[phase], 0)
        if abs(val) > abs(best_val):
            best_phase, best_val = phase, val
    return best_phase, best_val


def _escalation_config(settings):
    gate = settings.get("consistency_gate") or {}
    override = gate.get("feedback_override") or {}
    return {
        "enabled": bool(override.get("escalation_enabled", False)),
        "raw_min": override.get("feedback_override_raw_min"),
        "scaled_min": override.get("feedback_override_scaled_min"),
    }


def _candidate_rules_for_verdict(config, verdict, speed_class):
    # Every non-retired data/both rule whose own condition could plausibly
    # cover this corner/verdict/speed_class combination, independent of
    # whether today's aggregate severity actually clears its min_severity
    # gate -- the undrivable tier (below) checks LAP-LEVEL evidence for
    # each candidate itself, rather than relying on a pre-built bucket or
    # restricting the search to a single phase.
    for rule in config["rules"]:
        if rule.get("status") in _NON_FIRING_STATUSES:
            continue
        condition = rule["condition"]
        if condition.get("trigger") not in ("data", "both"):
            continue
        if condition.get("verdict") != verdict:
            continue
        required_speed_class = condition.get("speed_class")
        if required_speed_class is not None and required_speed_class != speed_class:
            continue
        yield rule


def _qualifying_laps_for_rule(rule, laps, classify_fn):
    # Lap-level evidence for one rule's own verdict/min_severity, evaluated
    # on the rule's own phases per lap -- replaces checking the median-of-
    # medians aggregate, which can dilute a repeating per-lap pattern down
    # to "normal" (see thesis_notes.md, "Undrivable tier: lap-level cell
    # matching"). Returns a list of {"lap", "severity", "short"} dicts, one
    # per qualifying lap.
    condition = rule["condition"]
    min_sev = condition.get("min_severity", "normal")
    hits = []
    for lap_summary in laps:
        severity, short = _phase_verdict(lap_summary, rule["phases"], classify_fn)
        if not _verdict_present(short, condition["verdict"]):
            continue
        if SEVERITY_RANK[severity] < SEVERITY_RANK[min_sev]:
            continue
        hits.append({"lap": lap_summary, "severity": severity, "short": short})
    return hits


def _add_rule_matches_to_buckets(buckets, rule, matches, escalation_by_base_cell, settings):
    # Shared between generate_recommendations' own bucket-building loop and
    # the undrivable tier's lap-level re-fire (below) -- one construction
    # path for every bucket, so a lap-level-triggered row carries exactly
    # what a normally-fired row carries (suggestion, scoring, and eligible
    # for the feasibility/parameter_conflict passes downstream).
    actions = _normalise_actions(rule["suggestion"])
    key = _bucket_key(rule, actions)
    bucket = buckets.setdefault(key, {
        "actions": actions,
        "score": 0.0,
        "escalation_notes": [],
        "severity_rank": 0,
        "is_recommended": False,
        "corners": {},
        "rules_fired": [],
        "cell_ids": [],
        "trigger_source": set(),
        "conflicts": {},
        "rationale": [],
    })
    bucket["rules_fired"].append(rule["id"])
    if rule.get("cell_id"):
        bucket["cell_ids"].append(rule["cell_id"])
    bucket["trigger_source"].add(rule["condition"]["trigger"])
    provenance_note = _provenance_note(rule, settings)
    bucket["rationale"].append({
        "rule_id": rule["id"], "cell_id": rule.get("cell_id"),
        "rationale": rule["rationale"] + (provenance_note or ""),
    })
    held_escalation = escalation_by_base_cell.get(rule.get("cell_id"))
    if held_escalation is not None:
        esc_lever = _describe_actions(_normalise_actions(held_escalation["suggestion"]))
        bucket["escalation_notes"].append(
            f"If this isn't enough, the next step is: {esc_lever}."
        )
    for m in matches:
        bucket["score"] += m["score"]
        bucket["severity_rank"] = max(bucket["severity_rank"], m["severity_rank"])
        if _match_is_recommended(m, rule, settings):
            bucket["is_recommended"] = True
        cid = m["stable_corner_id"]
        bucket["corners"][cid] = {
            "stable_corner_id": cid,
            "n_laps": m["n_laps"],
            "worst_corner": m["worst_corner"],
            "short_verdict": m["short_verdict"],
        }
        if m["conflict"]:
            bucket["conflicts"][cid] = {"stable_corner_id": cid, "n_laps": m["n_laps"]}
    return key


_URGENT_TAG = "URGENT - driver reports near-undrivable"


def _urgent_row(cid, n_laps, verdict, text, conflict=False):
    # Shape matches a normal generate_recommendations() result closely
    # enough that ui/views/outing_form.py's _build_recommendation_row
    # renders it with only the small, explicit "urgent"/action_class
    # branches added there -- no separate rendering path duplicated.
    # n_laps must be the corner's real (int) lap count, not None -- the
    # UI's chip label compares it against analysed_lap_count directly.
    # limit_status is deliberately neither "at_limit" nor "unchecked":
    # there is no setup-parameter action here for a limit to apply to.
    return {
        "actions": [],
        "parameter": None,
        "direction": None,
        "score": None,
        "severity_rank": SEVERITY_RANK["strong"],
        "corners": [{"stable_corner_id": cid, "n_laps": n_laps,
                     "worst_corner": False, "short_verdict": verdict}],
        "rules_fired": [],
        "cell_ids": [],
        "trigger_source": ["driver"],
        "conflicts": ([{"stable_corner_id": cid, "n_laps": n_laps}] if conflict else []),
        "rationale": [{"rule_id": None, "cell_id": None, "rationale": text}],
        "action_class": "urgent_gap",
        "observation_lines": [],
        "escalation_notes": [],
        "parameter_conflict": False,
        "conflict_parameters": [],
        "limit_status": "not_applicable",
        "at_limit_parameters": [],
        "selected": False,
        "urgent": True,
        "urgent_tag": _URGENT_TAG,
    }


def _apply_undrivable_escalation(aggregated, by_corner_laps, feedback_data, classify_fn, config,
                                  source_balance, feedback_weight, buckets, escalation_by_base_cell):
    """Undrivable-feedback tier (design ruling, project-lead-elicited
    2026-07-28; repaired 2026-07-28 turn 2 to match against LAP-LEVEL
    verdict instances, not the median-of-medians aggregate -- see
    thesis_notes.md, "Undrivable tier: lap-level cell matching"). At
    |raw feedback| >= feedback_override_raw_min the tool must never render
    silent emptiness for that corner -- honesty via labeling, not
    suppression. Uses the corner's single strongest-|feedback| phase
    (_worst_feedback) only to decide direction and whether the tier
    activates at all; which DATA phase is checked is no longer restricted
    to that same phase -- a driver's overall "near-undrivable" impression
    of a corner need not localise to the exact phase column they rated
    worst (verified against the real C12/apex_3 case: feedback recorded on
    exit_4, the corner's actual repeating moderate-understeer pattern is
    at apex_3). Three exhaustive outcomes per corner:

    (a) PIERCE/SYNTHESIZE -- some non-retired data/both rule
    (_candidate_rules_for_verdict) shows LAP-LEVEL evidence
    (_qualifying_laps_for_rule: severity >= that rule's own min_severity,
    axle matching the feedback's implied direction) on at least one of
    this corner's analysed laps. If that rule already produced a real
    match against the aggregate (a bucket exists), its key is pierced. If
    it did not -- the aggregate diluted the same lap-level pattern to
    "normal", which is exactly the bug this repair fixes -- the rule is
    RE-EVALUATED through the identical _evaluate_rule/
    _add_rule_matches_to_buckets path every other rule uses, substituting
    the qualifying lap's own (real, unaggregated) phase data for the
    rule's phases in place of the aggregate. Either way, if the scaled
    feedback also clears its own floor (the same double-floor discipline
    as the consistency-gate override), the bucket's key is forced to
    "recommended", bypassing min_score_to_show and the situational/
    provenance advisory caps, tagged URGENT, with an added rationale line
    naming the real per-lap pattern.

    (b) CONTRADICTION -- no rule shows matching-direction lap-level
    evidence anywhere for this corner, but at least one shows the OPPOSITE
    axle direction. If a bucket already recorded this exact conflict
    (data-triggered rule matched with conflict=True for this corner), it
    is pierced the same way as (a), keeping its existing conflict badge.
    If no such bucket exists, a standalone contradiction row is emitted
    instead, carrying the conflict badge itself.

    (c) SYNTHETIC GAP -- no rule shows lap-level evidence in either
    direction anywhere for this corner: a standalone row naming the gap
    directly.

    Returns (pierced_bucket_keys: set, synthetic_rows: list).
    """
    settings = config["settings"]
    esc_cfg = _escalation_config(settings)
    if not esc_cfg["enabled"] or esc_cfg["raw_min"] is None:
        return set(), []

    raw_min = esc_cfg["raw_min"]
    scaled_min = esc_cfg["scaled_min"]
    pierced_keys = set()
    synthetic_rows = []

    for cid, corner in aggregated.items():
        fb_row = _feedback_row(feedback_data, cid)
        phase, raw_fb = _worst_feedback(fb_row)
        if phase is None or abs(raw_fb) < raw_min:
            continue
        scaled_fb = raw_fb * feedback_weight
        clears_scaled = scaled_min is None or abs(scaled_fb) >= scaled_min
        implied_verdict = "oversteer" if raw_fb > 0 else "understeer"
        opposite_verdict = "understeer" if implied_verdict == "oversteer" else "oversteer"
        laps = by_corner_laps.get(cid, [])
        speed_class = corner.get("speed_class")

        fired_any = False
        for rule in _candidate_rules_for_verdict(config, implied_verdict, speed_class):
            hits = _qualifying_laps_for_rule(rule, laps, classify_fn)
            if not hits:
                continue
            best = max(hits, key=lambda h: SEVERITY_RANK[h["severity"]])
            evidence = (f"C{cid}: {len(hits)} of {corner['n_laps']} laps show {best['short']} "
                        f"-- driver reports near-undrivable ({implied_verdict}).")

            real_matches = _evaluate_rule(rule, {cid: corner}, {cid: laps}, feedback_data,
                                           classify_fn, settings, source_balance, feedback_weight)
            key = _bucket_key(rule, _normalise_actions(rule["suggestion"]))
            if real_matches:
                if key in buckets and cid in buckets[key]["corners"]:
                    fired_any = True
                    if clears_scaled:
                        pierced_keys.add(key)
                        buckets[key]["rationale"].append(
                            {"rule_id": None, "cell_id": rule.get("cell_id"), "rationale": evidence})
                continue

            # Aggregate diluted this rule's own severity gate to below
            # min_severity -- substitute the qualifying lap's real phase
            # data for this rule's phases and re-fire through the normal
            # path (by_corner_laps stays untouched real per-lap data, so
            # the consistency gate below still checks genuine repetition).
            escalated_corner = dict(corner)
            escalated_corner["phases"] = dict(corner["phases"])
            for p in rule["phases"]:
                if p in best["lap"]["phases"]:
                    escalated_corner["phases"][p] = best["lap"]["phases"][p]
            if "apex_3" in rule["phases"] and best["lap"].get("apex_region") is not None:
                # Keep apex_region consistent with the just-substituted apex_3
                # phase -- both must come from the SAME qualifying lap, not mix
                # this lap's real apex_3 with the aggregate's median-of-4-laps
                # apex_region.
                escalated_corner["apex_region"] = best["lap"]["apex_region"]

            synth_matches = _evaluate_rule(rule, {cid: escalated_corner}, {cid: laps}, feedback_data,
                                            classify_fn, settings, source_balance, feedback_weight)
            if not synth_matches:
                continue
            _add_rule_matches_to_buckets(buckets, rule, synth_matches, escalation_by_base_cell, settings)
            fired_any = True
            if clears_scaled:
                pierced_keys.add(key)
                buckets[key]["rationale"].append(
                    {"rule_id": None, "cell_id": rule.get("cell_id"), "rationale": evidence})

        if fired_any:
            continue

        contradiction = None
        for rule in _candidate_rules_for_verdict(config, opposite_verdict, speed_class):
            hits = _qualifying_laps_for_rule(rule, laps, classify_fn)
            if hits:
                contradiction = max(hits, key=lambda h: SEVERITY_RANK[h["severity"]])
                break

        if contradiction is not None:
            # (b) direction contradiction, found at lap level across every
            # phase rather than the aggregate at one feedback-named phase.
            conflicted_keys = [
                key for key, bucket in buckets.items()
                if cid in bucket["conflicts"]
                and _axle_verdict(bucket["corners"].get(cid, {}).get("short_verdict") or "") == opposite_verdict
            ]
            if conflicted_keys:
                pierced_keys.update(conflicted_keys)
            else:
                synthetic_rows.append(_urgent_row(
                    cid, corner["n_laps"], implied_verdict,
                    f"C{cid}: driver reports near-undrivable ({implied_verdict}) but the "
                    f"data shows {contradiction['short']} -- direction contradiction, "
                    f"engineer attention required.",
                    conflict=True,
                ))
            continue

        # (c) no rule shows lap-level evidence in either direction, anywhere.
        synthetic_rows.append(_urgent_row(
            cid, corner["n_laps"], implied_verdict,
            f"Driver reports near-undrivable at C{cid} ({implied_verdict}) - no "
            f"elicited rule covers this case, engineer attention required.",
        ))

    return pierced_keys, synthetic_rows


def _evaluate_rule(rule, aggregated, by_corner_laps, feedback_data, classify_fn, settings,
                    source_balance, feedback_weight=1.0):
    condition = rule["condition"]
    trigger = condition["trigger"]
    phases = rule["phases"]
    required_speed_class = condition.get("speed_class")
    matches = []

    # Global multiplier on top of the per-trigger score (applied after
    # agreement/conflict modulation): balances how much weight data-raised
    # vs driver-raised hypotheses carry. Neutral at source_balance=0.5
    # (both multipliers = 1.0); "both"-triggered matches are corroborated
    # by construction and are never discounted by this factor.
    data_source_factor = (1.0 - source_balance) * SOURCE_BALANCE_NORMALISER
    driver_source_factor = source_balance * SOURCE_BALANCE_NORMALISER

    for cid, corner in aggregated.items():
        # Matrix speed-class gate: this scenario x speed-class cell only
        # applies to corners whose (modal, lap-aggregated) speed_class
        # matches. Rules that don't specify speed_class (e.g. any future
        # non-matrix rule) skip this check entirely.
        if required_speed_class is not None and corner.get("speed_class") != required_speed_class:
            continue

        fb_row = _feedback_row(feedback_data, cid)
        # PART A: single insertion point -- every downstream trigger branch
        # (data/driver/both) reuses this one fb_value, so scaling it here
        # by the driver's resolved feedback_weight is sufficient to weight
        # both the driver-trigger score AND the data/both-trigger
        # corroboration criterion (_feedback_modulation's min_feedback_abs/
        # sign check runs against this weighted magnitude). See
        # config/recommendations.json settings.driver_level_weighting.
        raw_fb_value = _feedback_value(fb_row, phases)
        fb_value = raw_fb_value * feedback_weight
        conflict = False
        corroborated = False
        severity = None
        short = None

        if trigger == "data":
            severity, short = _phase_verdict(corner, phases, classify_fn)
            if not _verdict_present(short, condition["verdict"]):
                continue
            min_sev = condition.get("min_severity", "normal")
            if SEVERITY_RANK[severity] < SEVERITY_RANK[min_sev]:
                continue
            if not _consistency_gate_ok(cid, by_corner_laps, phases, condition["verdict"],
                                         min_sev, classify_fn, settings,
                                         raw_fb_value=raw_fb_value, scaled_fb_value=fb_value):
                continue
            factor, conflict, corroborated = _feedback_modulation(fb_value, condition, settings)
            score = (rule["weight"] * settings["severity_factors"][severity]
                     * factor * data_source_factor)

        elif trigger == "driver":
            min_abs = condition["min_feedback_abs"]
            if abs(fb_value) < min_abs:
                continue
            feedback_sign = condition["feedback_sign"]
            agrees_sign = ((feedback_sign == "negative" and fb_value < 0)
                           or (feedback_sign == "positive" and fb_value > 0))
            if not agrees_sign:
                continue
            severity, short = _phase_verdict(corner, phases, classify_fn)
            factor, conflict = _classifier_modulation(short, severity, condition["verdict"], settings)
            corroborated = True  # driver is the trigger; always_recommended_triggers covers eligibility
            score = (rule["weight"] * (abs(fb_value) / FEEDBACK_SCALE_MAX)
                     * factor * driver_source_factor)

        elif trigger == "both":
            severity, short = _phase_verdict(corner, phases, classify_fn)
            if not _verdict_present(short, condition["verdict"]):
                continue
            min_sev = condition.get("min_severity", "normal")
            if SEVERITY_RANK[severity] < SEVERITY_RANK[min_sev]:
                continue
            min_abs = condition["min_feedback_abs"]
            if abs(fb_value) < min_abs:
                continue
            feedback_sign = condition["feedback_sign"]
            agrees_sign = ((feedback_sign == "negative" and fb_value < 0)
                           or (feedback_sign == "positive" and fb_value > 0))
            if not agrees_sign:
                continue
            if not _consistency_gate_ok(cid, by_corner_laps, phases, condition["verdict"],
                                         min_sev, classify_fn, settings,
                                         raw_fb_value=raw_fb_value, scaled_fb_value=fb_value):
                continue
            corroborated = True
            # Both conditions already independently confirm agreement --
            # score with the same agreement_bonus a "data" rule would earn
            # from matching feedback, not a further-inflated multiplier.
            score = rule["weight"] * settings["severity_factors"][severity] * settings["agreement_bonus"]

        else:
            continue

        # Driver's own prioritisation of this corner -- applied once, after
        # trigger scoring and source_balance, uniformly regardless of which
        # trigger produced the match. Orthogonal to source_balance (who may
        # raise a hypothesis) and agreement/conflict (what the other source
        # says about a specific match).
        worst_flag = bool(fb_row.get("worst", False))
        if worst_flag:
            score *= settings.get("worst_corner_multiplier", 1.0)

        matches.append({
            "stable_corner_id": cid,
            "n_laps": corner["n_laps"],
            "score": score,
            "conflict": conflict,
            "worst_corner": worst_flag,
            "severity": severity,
            "severity_rank": SEVERITY_RANK.get(severity, 0),
            "corroborated": corroborated,
            "trigger": trigger,
            "short_verdict": short,
        })

    return matches


def _describe_actions(actions):
    parts = []
    for a in actions:
        if "target" in a:
            parts.append(f"{a['parameter']} -> {a['target']}")
        else:
            parts.append(f"{a['parameter']} {a['direction']} ({a['delta']:+g})")
    return " + ".join(parts)


def _numeric_bounds(entry):
    # Most parameters carry min/max directly on value_space. ride_height_*
    # instead states a "standard" value plus a typical_window delta (see
    # setup_parameters.json) -- derive the equivalent bounds from that.
    vs = entry["value_space"]
    if vs is None:
        return None, None
    if "min" in vs and "max" in vs:
        return vs["min"], vs["max"]
    tw = entry.get("typical_window") or {}
    standard = vs.get("standard")
    if standard is None:
        return None, None
    if "max_delta_from_standard" in tw:
        return standard + tw["max_delta_from_standard"], standard
    if "delta_from_standard" in tw:
        lo, hi = tw["delta_from_standard"]
        return standard + lo, standard + hi
    return None, None


def _check_feasible(entry, current_value, delta_value):
    # Returns True/False (checked) or None (not checked -- no bounds known
    # for this parameter, distinct from "current value unknown").
    vs = entry["value_space"]
    if vs and vs.get("type") == "enum" and "options" in vs:
        options = vs["options"]
        if current_value not in options:
            return None
        idx = options.index(current_value) + int(delta_value)
        return 0 <= idx < len(options)
    lo, hi = _numeric_bounds(entry)
    if lo is None or hi is None:
        return None
    try:
        new_value = float(current_value) + delta_value
    except (TypeError, ValueError):
        return None
    return lo <= new_value <= hi


def _current_setup_value(setup_data, entry):
    # "Real current value" test: present AND nonzero. Every matrix-touched
    # numeric setup field either can't legitimately be 0 (arb/toe/camber/
    # ride_height/diff_position -- 0 is out of range, so a stored 0 can only
    # be an untouched QDoubleSpinBox default) or CAN legitimately be 0
    # (damper clicks, min=0) -- for the latter we cannot distinguish a real
    # 0 from the same default, so we accept the ambiguity and treat 0 as
    # unknown uniformly rather than guess (WP2b-2 amendment 6). A nonzero
    # stored value is never a default, for any parameter.
    maps_to = entry.get("maps_to")
    if not maps_to or not setup_data:
        return None
    parts = maps_to[0].split(".")[1:]  # drop the "setup_parameters" prefix
    node = setup_data
    for p in parts:
        if not isinstance(node, dict) or p not in node:
            return None
        node = node[p]
    if node is None or node == "":
        return None
    if isinstance(node, (int, float)) and node == 0:
        return None
    return node


def _apply_feasibility(results, setup_data, registry):
    # WP2b-2 amendment 6: current + delta against the registry's min/max.
    # Target-style actions (abs_position) are absolute, not relative to a
    # current value -- always feasible, never checked.
    for r in results:
        for action in r["actions"]:
            if "target" in action:
                continue
            entry = registry.get(action["parameter"])
            if entry is None:
                action["feasible"] = None
                continue
            current = _current_setup_value(setup_data, entry)
            action["feasible"] = (None if current is None
                                   else _check_feasible(entry, current, action["delta"]))
        delta_statuses = [a["feasible"] for a in r["actions"] if "target" not in a]
        if any(s is False for s in delta_statuses):
            r["limit_status"] = "at_limit"
        elif any(s is None for s in delta_statuses):
            r["limit_status"] = "unchecked"
        else:
            r["limit_status"] = "ok"
        r["at_limit_parameters"] = [a["parameter"] for a in r["actions"]
                                     if a.get("feasible") is False]
    return results


def _apply_parameter_conflicts(results):
    # A conflict is any two DIFFERENT direction/target values recommended
    # for the SAME registry parameter across different buckets (matching
    # directions already merged into one bucket by _bucket_key, so this can
    # only fire across buckets) -- surfaced, never netted/averaged into a
    # false middle value.
    param_keys = {}
    for r in results:
        for action in r["actions"]:
            param_keys.setdefault(action["parameter"], set()).add(_action_key(action))
    conflicted_params = {p for p, keys in param_keys.items() if len(keys) > 1}
    for r in results:
        touched = sorted({a["parameter"] for a in r["actions"]} & conflicted_params)
        r["parameter_conflict"] = bool(touched)
        r["conflict_parameters"] = touched
    return results


def _rank_key(result, tier_map):
    # Ranking per WP2b-2 approval: severity first, then corner count
    # (breadth of evidence), then escalation-order cheapness (cockpit <
    # pitlane < garage), then cell_id lexical order as a final deterministic
    # tie-break. A package/pair uses its most expensive action's tier.
    tiers = [ESCALATION_TIER_RANK.get(tier_map.get(a["parameter"]), ESCALATION_TIER_RANK["pitlane"])
             for a in result["actions"]]
    tier_rank = max(tiers) if tiers else ESCALATION_TIER_RANK["pitlane"]
    cell_key = min(result["cell_ids"]) if result["cell_ids"] else ""
    return (-result["severity_rank"], -len(result["corners"]), tier_rank, cell_key)


def _apply_change_budget(results, settings):
    # Tool never auto-applies -- this only marks which ranked, non-
    # conflicted, feasible results fit the engineer's change budget for
    # this run. absolute_cap is exposed for a future manual-override UI;
    # the tool itself never auto-selects past default_max.
    budget = settings.get("change_budget", {"default_max": 1, "absolute_cap": 2})
    remaining = budget.get("default_max", 1)
    for r in results:
        eligible = (r["action_class"] == "recommended"
                    and not r["parameter_conflict"]
                    and r["limit_status"] != "at_limit"
                    and remaining > 0)
        r["selected"] = eligible
        if eligible:
            remaining -= 1
    return results


def generate_recommendations(summaries, classify_fn, feedback_data, setup_data, config,
                              outing=None, driving_level=None):
    """
    Turn per-lap-per-corner stability summaries + driver feedback into a
    ranked list of setup direction suggestions, each with a full evidence
    trail (which corners, which rules/cell_ids, any driver/data conflicts,
    any cross-rule parameter conflicts, feasibility against the outing's
    current setup sheet).

    Pipeline: (1) aggregate `summaries` per stable_corner_id via
    median-of-medians across laps (`aggregate_by_corner`), including a modal
    speed_class per corner. (2) For each non-firing-excluded rule in
    `config["rules"]` (status retired/held/dropped never fire), test every
    aggregated corner against the rule's condition, including the matrix's
    speed_class gate and a per-lap consistency gate (verdict must repeat on
    >= min_repeat_laps AND >= min_repeat_fraction of that corner's laps,
    settings["consistency_gate"] -- OR a single repeat lap is sufficient
    when the feedback DIRECTION agrees with the rule's own verdict
    (VERDICT_EXPECTED_FEEDBACK_SIGN) and both the raw and scaled
    feedback-magnitude floors in settings["consistency_gate"]
    ["feedback_override"] are cleared, see `_consistency_gate_ok`/
    `_override_direction_ok`) -- "data"/"both" rules fire from
    `classify_fn` (the same per-corner classifier the stability grid uses);
    "driver" rules fire from the feedback table. Every match is scaled by
    `source_balance`, `worst_corner_multiplier`, and (for "data" rules)
    agreement/conflict against driver feedback on the same phases -- see
    `_comment_source_balance`/`_comment_worst_corner`/`_comment_trigger` in
    config/recommendations.json. (3) Matches are grouped into buckets by
    parameter+direction (or, for a package/axle-symmetric-pair suggestion,
    one bucket per rule, keyed by cell_id -- `_bucket_key`); buckets under
    `min_score_to_show` are dropped. (4) Each bucket is classified
    "recommended" (ranked, budget-eligible) or "advisory" (observation only,
    never budget-eligible) per settings["action_class"] -- a "data"-trigger
    match at moderate severity with no driver corroboration on the same
    phases stays advisory; "driver"/"both" triggers and strong severity are
    always recommended (WP2b-2 amendment 7: mild understeer is this car's
    deliberate stable baseline, data-only moderate verdicts are diagnosis,
    not mandate). (5) A parameter_conflict pass flags any two buckets that
    recommend different directions/targets for the same registry parameter
    (never auto-resolved). (6) A feasibility pass checks current setup-sheet
    value (from `setup_data`) + each action's delta against the registry's
    value range, marking `limit_status` "at_limit" / "unchecked" / "ok"
    (WP2b-2 amendment 6). (7) Results are ranked (severity, corner count,
    escalation_tier, cell_id -- `_rank_key`) and the top
    `change_budget.default_max` recommended, non-conflicted, feasible
    results are marked `selected` (`_apply_change_budget`) -- distinct from
    `max_recommendations`, the display cap applied last.

    `classify_fn` is the caller's corner classifier (in the UI thread,
    `self._classify_corner`) -- reusing it rather than reimplementing the
    thresholds here guarantees a recommendation can never disagree with the
    verdict the stability grid displays for the same corner and phase.
    `setup_data` (the outing's own setup-sheet values, distinct from the
    config/setup_parameters.json registry loaded internally here) now backs
    the feasibility pass. `outing` is reserved for a future per-driver/
    per-outing source_balance override (see `_resolve_source_balance`).
    `driving_level` (PART A) is the outing's driver's plain Driver.
    driving_level int (1-10) or None -- resolved by the UI caller from
    Outing.driver_id, never queried from a DB session here -- and is
    resolved to a feedback_weight multiplier (`_resolve_feedback_weight`,
    config/recommendations.json settings.driver_level_weighting) applied
    once where fb_value is computed in `_evaluate_rule`.

    Returns a list of dicts: {actions, parameter, direction (convenience
    fields, single-action buckets only), score, severity_rank, corners
    ([{stable_corner_id, n_laps, worst_corner, short_verdict}, ...]),
    rules_fired, cell_ids, trigger_source, conflicts (driver/data
    disagreement, per corner), parameter_conflict, conflict_parameters,
    action_class ("recommended"|"advisory"), observation_lines (advisory
    buckets only), escalation_notes (second-choice visibility -- display
    only, never fires: the held escalation's action, for any base rule that
    has one), limit_status, at_limit_parameters, selected, rationale}.

    (8) Undrivable-feedback tier (design ruling 2026-07-28, repaired
    2026-07-28 turn 2, `_apply_undrivable_escalation`): runs after buckets
    are built, before results are constructed. A corner whose single
    strongest feedback entry clears settings["consistency_gate"]
    ["feedback_override"]'s raw_min (and, only for the pierce/synthesize
    case, scaled_min too) can never render as silent emptiness -- exactly
    one of: pierced or synthesized (a rule shows LAP-LEVEL evidence --
    severity >= the rule's own min_severity on at least one analysed lap,
    checked across every phase/rule the driver's feedback direction could
    plausibly cover, not just the phase the feedback happened to name --
    forced to "recommended", `severity_rank` forced to "strong",
    `urgent`/`urgent_tag` set, every situational/provenance advisory cap
    bypassed, with an added rationale line naming the real per-lap
    pattern), a synthetic action_class="urgent_gap" row (no rule shows
    lap-level evidence in either direction anywhere for this corner), or a
    synthetic contradiction row carrying the conflict badge (some rule's
    lap-level evidence is the OPPOSITE axle direction from what the
    feedback implies). The repair (turn 2) replaced an earlier version
    that checked only the aggregate at the feedback's own named phase --
    that version could report a spurious "no elicited rule covers this
    case" gap for a corner whose real per-lap pattern (e.g. moderate
    understeer on 2 of 4 laps) diluted to "normal" in the aggregate, or
    that lived at a different phase than the one the feedback named (the
    real C12 case this was verified against, thesis_notes.md "Undrivable
    tier: lap-level cell matching"). Synthetic rows are prepended to the
    returned list, outside `max_recommendations`' display cap and never
    counted by `_apply_change_budget`. Gated entirely by
    settings["consistency_gate"]["feedback_override"]["escalation_enabled"]
    -- false restores pre-2026-07-28 behaviour.
    """
    settings = config["settings"]
    source_balance = _resolve_source_balance(config, outing)
    feedback_weight = _resolve_feedback_weight(config, driving_level)
    aggregated = aggregate_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)
    registry = load_setup_parameters_registry()
    tier_map = {k: v.get("escalation_tier", "pitlane") for k, v in registry.items()}
    advisory_prefix = settings.get("action_class", {}).get("advisory_rationale_prefix", "")
    # Second-choice visibility (display only -- these rules never fire,
    # _NON_FIRING_STATUSES already excludes "held" from the match loop
    # below): base cell_id -> its held escalation rule, if any.
    escalation_by_base_cell = {
        r["escalation_of"]: r for r in config["rules"]
        if r.get("status") == "held" and r.get("escalation_of")
    }

    buckets = {}
    for rule in config["rules"]:
        if rule.get("status") in _NON_FIRING_STATUSES:
            continue
        matches = _evaluate_rule(rule, aggregated, by_corner_laps, feedback_data,
                                  classify_fn, settings, source_balance, feedback_weight)
        if not matches:
            continue
        _add_rule_matches_to_buckets(buckets, rule, matches, escalation_by_base_cell, settings)

    # Undrivable-feedback tier (design ruling 2026-07-28, repaired 2026-07-28
    # turn 2 -- lap-level cell matching): must run against the fully-built
    # buckets (it needs to know which corners already have a corroborated,
    # or conflicted, match) but before the results list is built, since a
    # pierced bucket's action_class/severity_rank/urgent tag are decided at
    # result-construction time. May itself add new buckets (a rule whose
    # aggregate-level match never cleared its own severity gate, but whose
    # lap-level evidence does) via the same _add_rule_matches_to_buckets
    # helper the main loop above uses.
    pierced_keys, synthetic_rows = _apply_undrivable_escalation(
        aggregated, by_corner_laps, feedback_data, classify_fn, config,
        source_balance, feedback_weight, buckets, escalation_by_base_cell,
    )

    results = []
    for key, bucket in buckets.items():
        pierced = key in pierced_keys
        if bucket["score"] < settings["min_score_to_show"] and not pierced:
            continue
        actions = bucket["actions"]
        single = actions[0] if len(actions) == 1 else None
        action_class = "recommended" if (bucket["is_recommended"] or pierced) else "advisory"
        # Pierced buckets bypass the situational/provenance advisory caps
        # entirely (design ruling) -- action_class is forced above, and
        # severity_rank is forced to "strong" so the existing severity-
        # first ranking (_rank_key) surfaces it near the top without a
        # separate sort override.
        severity_rank = SEVERITY_RANK["strong"] if pierced else bucket["severity_rank"]
        rationale = bucket["rationale"]
        observation_lines = []
        if action_class == "advisory":
            rationale = [{**x, "rationale": advisory_prefix + x["rationale"]} for x in rationale]
            lever = _describe_actions(actions)
            # Fix turn: cell_id is already shown as its own header badge,
            # so it's dropped here rather than repeated in the observation
            # line; "@" -> "at" reads as plain English, not the classifier's
            # own short-verdict format.
            observation_lines = [
                f"C{c['stable_corner_id']}: slight {c['short_verdict'].replace(' @ ', ' at ')} - "
                f"likely lever if addressed: {lever}"
                for c in sorted(bucket["corners"].values(), key=lambda c: c["stable_corner_id"])
            ]
        results.append({
            "actions": actions,
            "parameter": single["parameter"] if single else None,
            "direction": single.get("direction") if single else None,
            "score": round(bucket["score"], 3),
            "severity_rank": severity_rank,
            "corners": sorted(bucket["corners"].values(), key=lambda c: c["stable_corner_id"]),
            "rules_fired": bucket["rules_fired"],
            "cell_ids": sorted(bucket["cell_ids"]),
            "trigger_source": sorted(bucket["trigger_source"]),
            "conflicts": sorted(bucket["conflicts"].values(), key=lambda c: c["stable_corner_id"]),
            "rationale": rationale,
            "action_class": action_class,
            "observation_lines": observation_lines,
            "escalation_notes": sorted(set(bucket["escalation_notes"])),
            "urgent": pierced,
            "urgent_tag": (_URGENT_TAG if pierced else None),
        })

    results = _apply_parameter_conflicts(results)
    results = _apply_feasibility(results, setup_data, registry)
    results.sort(key=lambda r: _rank_key(r, tier_map))
    results = _apply_change_budget(results, settings)

    # Synthetic gap/contradiction rows are never budget-counted as a setup
    # change and never subject to max_recommendations' display cap --
    # prepended after both, per the design ruling ("top of the list").
    return synthetic_rows + results[: settings["max_recommendations"]]
