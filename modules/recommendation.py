# Rule engine: stability summaries + driver feedback + rule table
# (config/recommendations.json) -> ranked setup suggestions with evidence.
# No longer shown directly in the UI -- decision_frame migrated all rules as
# bridges (parity: diagnostics/inspect_frame_stage2_parity.py) and still
# calls the rule table and helpers here.
#
# Rules = engineer decision matrix (scenario x speed class, rule.cell_id)
# on setup_parameters.json keys. Mild understeer is this car's stable
# baseline -> uncorroborated moderate data-only matches stay ADVISORY
# (never budget-eligible), not RECOMMENDED.

import json
import numpy as np

RECOMMENDATIONS_CONFIG_PATH = "config/recommendations.json"
SETUP_PARAMETERS_CONFIG_PATH = "config/setup_parameters.json"

# must match summarise_corners' phase keys and the e1..x5 feedback columns
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
PHASE_TO_FEEDBACK_KEY = dict(zip(PHASE_KEYS, ["e1", "e2", "a3", "x4", "x5"]))

# ordinal enum, method-defining
SEVERITY_RANK = {"normal": 0, "moderate": 1, "strong": 2}

# Feedback scale: -5 undrivable understeer .. +5 undrivable oversteer.
# Every rule's verdict <-> feedback_sign pairing must agree with this map;
# the consistency-gate override uses it for direction. Verdicts not listed
# (unstable_yaw) have no feedback axis -> override never applies.
VERDICT_EXPECTED_FEEDBACK_SIGN = {"understeer": "negative", "oversteer": "positive"}

# ordinal enum (corner_analysis speed_class)
SPEED_CLASS_ORDER = ["low", "medium", "high"]

# escalation tier (cockpit -> pitlane -> garage); separate axis from
# change_effort, e.g. diff_position = "seconds" but garage
ESCALATION_TIER_RANK = {"cockpit": 0, "pitlane": 1, "garage": 2}

# method-defining: shape of the scoring formula
SOURCE_BALANCE_NORMALISER = 2.0  # 0.5 -> both multipliers 1.0
FEEDBACK_SCALE_MAX = 5.0  # feedback entered on -5..+5

# Never fire. retired = superseded seeds; held = escalation rule, specified
# but not automated (no history of applied changes); dropped = cell has
# no action.
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
    # modal speed_class across laps -> a corner near a threshold doesn't flip
    # the rule gate per lap. Tie -> closest to medium, then lower class.
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
    # CS cross-lap combiner from config: "median" keeps what repeats every
    # lap; "worst_lap" = per-phase min across laps, composed with
    # _classify_corner's min over phases = global min over (lap, phase).
    # Thresholds must be re-derived for whichever population is used.
    # Stability always median.
    # A one-lap CS anomaly can show in the aggregate but still can't fire a
    # rule alone -- the consistency gate re-checks per lap.
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
        # same combiner for apex_region (read by _classify_corner for apex_3 CS);
        # older summaries lack the key -> None, filtered like any other gap
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
    # Same classifier as the stability grid -> a recommendation can't disagree
    # with the grid. apex_region passed along only when apex_3 is in the rule.
    sliced = {p: aggregated_corner["phases"][p]
              for p in phases if p in aggregated_corner["phases"]}
    call_arg = {"phases": sliced}
    if "apex_3" in phases and aggregated_corner.get("apex_region") is not None:
        call_arg["apex_region"] = aggregated_corner["apex_region"]
    severity, short, _long, _colour = classify_fn(call_arg)
    return severity, short


def _axle_verdict(short):
    # short verdict names at most one axle
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
    # feedback row i+1 <-> stable_corner_id i+1; no lap dimension
    if not feedback_data:
        return {}
    corners = feedback_data.get("corners", [])
    idx = stable_corner_id - 1
    if idx < 0 or idx >= len(corners):
        return {}
    return corners[idx]


def _feedback_value(feedback_row, phases):
    # max |value| over the rule's phases -- averaging would wash out a sharp complaint
    vals = [feedback_row.get(PHASE_TO_FEEDBACK_KEY[p], 0) for p in phases
            if p in PHASE_TO_FEEDBACK_KEY]
    if not vals:
        return 0
    return max(vals, key=abs)


def _feedback_modulation(fb_value, condition, settings):
    # data trigger: phase feedback modulates the score both ways. No
    # feedback_sign (yaw rules) -> never modulated, never "corroborated".
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
    # driver trigger: classifier verdict modulates instead; condition["verdict"]
    # is only the agreement reference, never a gate
    if severity == "normal":
        return 1.0, False
    axle = _axle_verdict(short)
    if axle is None:
        return 1.0, False
    if axle == agreement_ref:
        return settings["agreement_bonus"], False
    return settings["conflict_penalty"], True


def _resolve_source_balance(config, outing=None):
    # only place source_balance is read. outing kept for a later per-outing
    # override. Per-driver weighting is separate (_resolve_feedback_weight):
    # data-vs-driver vs driver-vs-driver.
    return config["settings"]["source_balance"]


def _resolve_feedback_weight(config, driving_level):
    # driving_level (1-10 or None, resolved by the UI) -> feedback_weight via
    # settings.driver_level_weighting; None / not in table -> default_weight
    dlw = config["settings"].get("driver_level_weighting")
    if dlw is None or driving_level is None:
        return 1.0 if dlw is None else dlw.get("default_weight", 1.0)
    return dlw["weights"].get(str(driving_level), dlw.get("default_weight", 1.0))


def _override_direction_ok(verdict, raw_fb_value, raw_min):
    # direction, not just magnitude: understeer needs raw <= -min, oversteer
    # raw >= +min. No sign axis -> never qualifies.
    expected_sign = VERDICT_EXPECTED_FEEDBACK_SIGN.get(verdict)
    if expected_sign == "negative":
        return raw_fb_value <= -raw_min
    if expected_sign == "positive":
        return raw_fb_value >= raw_min
    return False


def _consistency_gate_ok(cid, by_corner_laps, phases, verdict, min_severity, classify_fn, settings,
                          raw_fb_value=0.0, scaled_fb_value=0.0):
    # No recommendation unless the verdict repeats: re-classify per lap, need
    # both min laps and min fraction at/above min_severity.
    # Override: a strong driver complaint in the rule's direction, on a corner
    # already moderate+ in the data, counts as repeat evidence -- raw and
    # scaled magnitude both above their floors -> one matching lap is enough
    # (skips count and fraction). Values default 0.0 -> never overrides.
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
    # packages / axle pairs (no axle-level registry key) are budgeted as one:
    # own bucket keyed by cell_id
    if len(actions) == 1:
        a = actions[0]
        return (a["parameter"], _action_key(a))
    return ("__package__", rule.get("cell_id") or rule["id"])


def _provenance_note(rule, settings):
    # screen text = short suffix only; provenance, cell and advisory status
    # live in structured fields. project-lead-reviewed gets no suffix.
    ac = settings.get("action_class", {})
    eligible = set(ac.get("action_eligible_provenances", ["engineer-verbatim", "project-lead-reviewed"]))
    prov = rule.get("elicitation_provenance")
    if prov is None or prov in eligible or rule.get("status") == "reviewed":
        return None
    return " (engineer confirmation pending)"


def _match_is_recommended(match, rule, settings):
    # action-eligible severity/trigger combos come from settings["action_class"]
    ac = settings.get("action_class", {})
    # situational rule (matrix lists several levers) = always advisory
    if rule.get("situational"):
        return False
    # only verbatim or project-lead-reviewed cells can be recommended; others
    # capped until the cell is promoted to "reviewed"
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
    # strongest complaint over all five phases (not the rule's subset).
    # Returns (phase, raw); (None, 0) if empty.
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
    # all non-retired data/both rules that could cover this corner/verdict/
    # speed class, regardless of today's aggregate severity -- the undrivable
    # tier checks lap-level evidence itself
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
    # lap-level evidence for one rule -- the median-of-medians aggregate can
    # dilute a repeating per-lap pattern to "normal".
    # Returns [{"lap", "severity", "short"}].
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
    # one bucket builder for the main loop and the undrivable re-fire ->
    # identical row contents
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
    # same shape as a normal result so _build_recommendation_row renders it.
    # n_laps = real int (UI compares it). limit_status neither "at_limit" nor
    # "unchecked" -- no setup action here.
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
    """Undrivable-feedback tier: |raw feedback| >= feedback_override_raw_min ->
    the corner never shows silent emptiness. Strongest-|feedback| phase
    decides direction and activation only; data may come from any phase
    (real C12: feedback on exit_4, repeating understeer at apex_3).
    Evidence is lap-level (_qualifying_laps_for_rule), not the aggregate.

    (a) pierce/synthesize: some candidate rule has matching-direction lap
        evidence. Existing bucket -> pierced; none (aggregate diluted it) ->
        rule re-fired through _evaluate_rule with that lap's real phase data.
        Scaled feedback above its floor too -> forced "recommended", URGENT,
        min_score and advisory caps bypassed, extra rationale line.
    (b) contradiction: only opposite-direction evidence. Existing conflict
        bucket pierced, else standalone contradiction row.
    (c) gap: no evidence either way -> standalone row naming the gap.

    Returns (pierced_bucket_keys, synthetic_rows).
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

            # aggregate diluted the gate -> re-fire with this lap's real phase data;
            # by_corner_laps untouched, so the consistency gate still sees real repeats
            escalated_corner = dict(corner)
            escalated_corner["phases"] = dict(corner["phases"])
            for p in rule["phases"]:
                if p in best["lap"]["phases"]:
                    escalated_corner["phases"][p] = best["lap"]["phases"][p]
            if "apex_3" in rule["phases"] and best["lap"].get("apex_region") is not None:
                # apex_region from the same lap as apex_3
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
            # (b) contradiction, lap level, any phase
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

        # (c) no evidence either way
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

    # data vs driver weighting after agreement/conflict; neutral at 0.5.
    # "both" matches never discounted.
    data_source_factor = (1.0 - source_balance) * SOURCE_BALANCE_NORMALISER
    driver_source_factor = source_balance * SOURCE_BALANCE_NORMALISER

    for cid, corner in aggregated.items():
        # matrix speed-class gate; rules without speed_class skip it
        if required_speed_class is not None and corner.get("speed_class") != required_speed_class:
            continue

        fb_row = _feedback_row(feedback_data, cid)
        # single point where feedback_weight applies -- covers driver-trigger score
        # and data/both corroboration alike
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
            corroborated = True  # driver is the trigger
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
            # both sources agree already -> plain agreement_bonus, no extra inflation
            score = rule["weight"] * settings["severity_factors"][severity] * settings["agreement_bonus"]

        else:
            continue

        # driver's corner priority, applied once after trigger scoring and
        # source_balance, whatever the trigger
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
    # ride_height_*: bounds = standard +/- typical_window
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
    # True/False, or None = no bounds known (not the same as unknown current value)
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
    # current value known = present and nonzero. For most keys 0 is out of
    # range (spinbox default); damper clicks can be 0 but a real 0 can't be
    # told from the default -> 0 = unknown everywhere.
    maps_to = entry.get("maps_to")
    if not maps_to or not setup_data:
        return None
    parts = maps_to[0].split(".")[1:]  # drop "setup_parameters"
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
    # current + delta vs registry min/max; abs_position targets always feasible
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
    # different directions/targets for the same parameter across buckets ->
    # flagged, never averaged
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
    # severity, corner count, escalation tier (cockpit < pitlane < garage),
    # cell_id. Packages use their most expensive tier.
    tiers = [ESCALATION_TIER_RANK.get(tier_map.get(a["parameter"]), ESCALATION_TIER_RANK["pitlane"])
             for a in result["actions"]]
    tier_rank = max(tiers) if tiers else ESCALATION_TIER_RANK["pitlane"]
    cell_key = min(result["cell_ids"]) if result["cell_ids"] else ""
    return (-result["severity_rank"], -len(result["corners"]), tier_rank, cell_key)


def _apply_change_budget(results, settings):
    # never auto-applies -- only marks what fits the change budget;
    # absolute_cap reserved for a manual override
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
    """Stability summaries + driver feedback -> ranked setup suggestions with
    evidence (corners, rules/cell_ids, conflicts, feasibility).

    1. aggregate per stable corner (median of medians, modal speed_class)
    2. per firing rule: condition + speed-class gate + consistency gate
       (repeat on min laps and min fraction, or one lap with a matching-
       direction strong complaint). data/both fire from classify_fn,
       driver from feedback. Score x source_balance x worst_corner
       multiplier (+ agreement/conflict for data rules).
    3. bucket by parameter+direction (packages: per cell_id); drop below
       min_score_to_show
    4. recommended vs advisory per settings["action_class"]
    5. parameter_conflict: different directions on one parameter, flagged
    6. feasibility: current setup value + delta vs registry range ->
       limit_status "at_limit" / "unchecked" / "ok"
    7. rank (_rank_key); top change_budget.default_max eligible -> selected;
       max_recommendations = display cap, applied last
    8. undrivable tier (_apply_undrivable_escalation): synthetic rows
       prepended, outside display cap and budget. Off via
       feedback_override.escalation_enabled.

    classify_fn = the grid's own classifier -> no disagreement with the grid.
    setup_data = the outing's setup sheet (feasibility). driving_level = int
    1-10 or None, resolved by the UI.

    Returns list of dicts: actions, parameter/direction (single-action only),
    score, severity_rank, corners, rules_fired, cell_ids, trigger_source,
    conflicts, parameter_conflict, conflict_parameters, action_class,
    observation_lines, escalation_notes (display only), limit_status,
    at_limit_parameters, selected, rationale.
    """
    settings = config["settings"]
    source_balance = _resolve_source_balance(config, outing)
    feedback_weight = _resolve_feedback_weight(config, driving_level)
    aggregated = aggregate_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)
    registry = load_setup_parameters_registry()
    tier_map = {k: v.get("escalation_tier", "pitlane") for k, v in registry.items()}
    advisory_prefix = settings.get("action_class", {}).get("advisory_rationale_prefix", "")
    # display only: base cell_id -> its held escalation rule
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

    # after buckets exist (needs to know corroborated/conflicted corners),
    # before results -- pierced action_class/severity/urgent set there. May add
    # buckets via the same helper.
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
        # pierced: advisory caps bypassed, severity "strong" -> ranks near the top
        severity_rank = SEVERITY_RANK["strong"] if pierced else bucket["severity_rank"]
        rationale = bucket["rationale"]
        observation_lines = []
        if action_class == "advisory":
            rationale = [{**x, "rationale": advisory_prefix + x["rationale"]} for x in rationale]
            lever = _describe_actions(actions)
            # cell_id already a header badge; "@" -> "at"
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

    # synthetic rows: not budgeted, not display-capped, prepended
    return synthetic_rows + results[: settings["max_recommendations"]]
