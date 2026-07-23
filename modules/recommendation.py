# Recommendation engine framework for SetupTool.
# Pure Python. No Qt imports. Converts per-lap-per-corner stability
# summaries + driver feedback + a rule table (config/recommendations.json)
# into a ranked, evidence-backed list of setup direction suggestions.

import json
import numpy as np

RECOMMENDATIONS_CONFIG_PATH = "config/recommendations.json"

# Must match the phase_keys list in modules/stability_analysis.py
# summarise_corners(), and the e1..x5 feedback columns collected in
# ui/views/outing_form.py's feedback table (positional pairing below).
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
PHASE_TO_FEEDBACK_KEY = dict(zip(PHASE_KEYS, ["e1", "e2", "a3", "x4", "x5"]))

SEVERITY_RANK = {"normal": 0, "moderate": 1, "strong": 2}
SEVERITY_FACTOR = {"normal": 0.0, "moderate": 1.0, "strong": 2.0}


def load_recommendations_config():
    with open(RECOMMENDATIONS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _nanmedian_or_nan(values):
    valid = [v for v in values if v == v]  # drop NaN (NaN != NaN)
    if not valid:
        return float("nan")
    return float(np.median(valid))


def aggregate_by_corner(summaries):
    # Median-of-medians per stable_corner_id: each lap already reduces a
    # corner instance to a per-phase median (summarise_corners); taking
    # the median across laps of those medians privileges behaviour that
    # repeats every lap. A single-lap anomaly washes out and cannot by
    # itself drive a recommendation -- setup changes should address
    # repeatable patterns, not one-off excursions.
    by_id = {}
    for s in summaries:
        cid = s.get("stable_corner_id")
        if cid is None:
            continue
        by_id.setdefault(cid, []).append(s)

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
                "cs_ratio_f": {"median": _nanmedian_or_nan(csf)},
                "cs_ratio_r": {"median": _nanmedian_or_nan(csr)},
                "stability_observed_Nm_per_deg": {"median": _nanmedian_or_nan(stab)},
            }
        aggregated[cid] = {
            "stable_corner_id": cid,
            "n_laps": len(corner_summaries),
            "phases": phases,
        }
    return aggregated


def _phase_verdict(aggregated_corner, phases, classify_fn):
    # Slices the aggregate summary down to the rule's own phases and
    # reuses classify_fn (the UI's _classify_corner) unmodified. This is
    # what guarantees a recommendation can never disagree with the
    # verdict the stability grid shows for the same corner and phase --
    # both are the identical classifier, not two independent judgments.
    sliced = {p: aggregated_corner["phases"][p]
              for p in phases if p in aggregated_corner["phases"]}
    severity, short, _long, _colour = classify_fn({"phases": sliced})
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
    # rule -- no natural feedback axis) is never modulated.
    feedback_sign = condition.get("feedback_sign")
    if feedback_sign is None:
        return 1.0, False
    min_abs = condition.get("min_feedback_abs", 0)
    if abs(fb_value) < min_abs:
        return 1.0, False
    agrees = ((feedback_sign == "negative" and fb_value < 0)
              or (feedback_sign == "positive" and fb_value > 0))
    if agrees:
        return settings["agreement_bonus"], False
    return settings["conflict_penalty"], True


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
    # is just the global default; WP2b-2 may extend the order to
    # per-driver feedback weighting > outing override > global default
    # (see PLAN.md). `outing` is accepted now so call sites don't need to
    # change signature when that resolution order lands.
    return config["settings"]["source_balance"]


def _evaluate_rule(rule, aggregated, feedback_data, classify_fn, settings, source_balance):
    condition = rule["condition"]
    trigger = condition["trigger"]
    phases = rule["phases"]
    matches = []

    # Global multiplier on top of the per-trigger score (applied after
    # agreement/conflict modulation): balances how much weight data-raised
    # vs driver-raised hypotheses carry. Neutral at source_balance=0.5
    # (both multipliers = 1.0); "both"-triggered matches are corroborated
    # by construction and are never discounted by this factor.
    data_source_factor = (1.0 - source_balance) * 2.0
    driver_source_factor = source_balance * 2.0

    for cid, corner in aggregated.items():
        fb_row = _feedback_row(feedback_data, cid)
        fb_value = _feedback_value(fb_row, phases)
        conflict = False

        if trigger == "data":
            severity, short = _phase_verdict(corner, phases, classify_fn)
            if not _verdict_present(short, condition["verdict"]):
                continue
            min_sev = condition.get("min_severity", "normal")
            if SEVERITY_RANK[severity] < SEVERITY_RANK[min_sev]:
                continue
            factor, conflict = _feedback_modulation(fb_value, condition, settings)
            score = (rule["suggestion"]["weight"] * SEVERITY_FACTOR[severity]
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
            factor, conflict = _classifier_modulation(
                short, severity, condition["verdict"], settings)
            score = (rule["suggestion"]["weight"] * (abs(fb_value) / 5.0)
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
            # Both conditions already independently confirm agreement --
            # score with the same agreement_bonus a "data" rule would earn
            # from matching feedback, not a further-inflated multiplier.
            score = (rule["suggestion"]["weight"] * SEVERITY_FACTOR[severity]
                     * settings["agreement_bonus"])

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
        })

    return matches


def generate_recommendations(summaries, classify_fn, feedback_data, setup_data, config, outing=None):
    """
    Turn per-lap-per-corner stability summaries + driver feedback into a
    ranked list of setup direction suggestions, each with a full evidence
    trail (which corners, which rules, any driver/data conflicts).

    Pipeline: (1) aggregate `summaries` per stable_corner_id via
    median-of-medians across laps, so a single-lap anomaly cannot drive a
    recommendation. (2) For each rule in `config["rules"]`, test every
    aggregated corner against the rule's condition -- "data" rules fire
    from `classify_fn` (the same per-corner classifier the stability grid
    uses, sliced to the rule's own phases) with feedback as a symmetric
    modulator; "driver" rules fire from the feedback table (e1..x5, mapped
    onto the five phase keys, max-|value| across a multi-phase rule) with
    the classifier as the modulator instead; "both" rules require the data
    and driver conditions to hold independently, with no further
    modulation. Every match is then scaled by a global data/driver
    `source_balance` factor (resolved once via `_resolve_source_balance`,
    neutral at the 0.5 default -- see `_comment_source_balance` in
    config/recommendations.json), and again by `worst_corner_multiplier`
    if the driver flagged that corner "worst" in the feedback table --
    applied once, uniformly, regardless of which trigger produced the
    match (see `_comment_worst_corner`). (3) Matches are summed per
    (parameter, direction), sorted by score, and results under
    `config["settings"]["min_score_to_show"]` or beyond
    `max_recommendations` are dropped.

    `classify_fn` is the caller's corner classifier (in the UI thread,
    `self._classify_corner`) -- reusing it rather than reimplementing the
    thresholds here guarantees a recommendation can never disagree with
    the verdict the stability grid displays for the same corner and phase.
    `setup_data` is accepted per the WP2 interface but not yet read by any
    seed rule (see WP2b-1/WP2b-2 in PLAN.md). `outing` is accepted for the
    same forward-compatibility reason as `_resolve_source_balance` -- not
    read yet, reserved for a future per-driver/per-outing source_balance
    override.

    Returns a list of dicts: {parameter, direction, score, corners
    ([{stable_corner_id, n_laps, worst_corner}, ...]), rules_fired,
    trigger_source, conflicts ([{stable_corner_id, n_laps}, ...]),
    rationale ([{rule_id, rationale}, ...])}.
    """
    settings = config["settings"]
    source_balance = _resolve_source_balance(config, outing)
    aggregated = aggregate_by_corner(summaries)

    buckets = {}
    for rule in config["rules"]:
        matches = _evaluate_rule(rule, aggregated, feedback_data, classify_fn, settings, source_balance)
        if not matches:
            continue
        key = (rule["suggestion"]["parameter"], rule["suggestion"]["direction"])
        bucket = buckets.setdefault(key, {
            "parameter": key[0],
            "direction": key[1],
            "score": 0.0,
            "corners": {},
            "rules_fired": [],
            "trigger_source": set(),
            "conflicts": {},
            "rationale": [],
        })
        bucket["rules_fired"].append(rule["id"])
        bucket["trigger_source"].add(rule["condition"]["trigger"])
        bucket["rationale"].append({"rule_id": rule["id"], "rationale": rule["rationale"]})
        for m in matches:
            bucket["score"] += m["score"]
            cid = m["stable_corner_id"]
            bucket["corners"][cid] = {
                "stable_corner_id": cid,
                "n_laps": m["n_laps"],
                "worst_corner": m["worst_corner"],
            }
            if m["conflict"]:
                bucket["conflicts"][cid] = {"stable_corner_id": cid, "n_laps": m["n_laps"]}

    results = []
    for bucket in buckets.values():
        if bucket["score"] < settings["min_score_to_show"]:
            continue
        results.append({
            "parameter": bucket["parameter"],
            "direction": bucket["direction"],
            "score": round(bucket["score"], 3),
            "corners": sorted(bucket["corners"].values(), key=lambda c: c["stable_corner_id"]),
            "rules_fired": bucket["rules_fired"],
            "trigger_source": sorted(bucket["trigger_source"]),
            "conflicts": sorted(bucket["conflicts"].values(), key=lambda c: c["stable_corner_id"]),
            "rationale": bucket["rationale"],
        })

    results.sort(key=lambda r: r["score"], reverse=True)
    return results[: settings["max_recommendations"]]
