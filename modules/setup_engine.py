"""Setup engine: corner symptoms -> mechanism -> one lever change per corner.

Replaces the rule engine (recommendation.py) and the decision frame
(recommendation overhaul D1-D9, author 2026-10-22). The method in four steps:

1. Symptoms per lap and phase come from the classifier (ruling D2): US = front
   axle limiting, OS = rear axle limiting, INST = open-loop unstable yaw.
2. Repetition (D5): a corner-phase symptom counts only when it appears in a
   majority of the corner's valid laps and in at least two of them, so a
   single disturbed lap cannot drive a setup change. Its severity is read from
   the cross-lap aggregate, the population the classification tiers were
   derived on.
3. Candidates come from the mechanism table (config/mechanisms.json, D3):
   each row ties a lever and a direction to a symptom, phase group and speed
   class with a literature anchor. The registry turns the direction word into
   a signed step and checks it against the current setup sheet.
4. Ranking (D4, order ruled 2026-10-22, 4e): severity -> repetition ->
   engineer preference -> change effort -> window headroom; a traction-limited
   exit demotes the differential ahead of preference. Preference only counts
   where its cell agrees with the table (lever and direction), so it orders
   table candidates and can neither add nor reverse a lever. One change per
   corner with up to two
   alternatives, because the effect of a setup change is only attributable
   when one thing changes at a time (Milliken 1995, setup procedure;
   Trzesniowski & Eder 2019, Bd. 5 ch. 5).
5. Two outputs per corner (author ruling 2026-10-23, 4g): the setup
   recommendation is chosen among the mechanical levers only; the best
   cockpit lever (registry escalation_tier "cockpit": brake bias, ABS map,
   TC lat / lon) is a separate "driver can also" line, ranked by the same key
   from the same table. Team practice: cockpit values are changed by the
   drivers during an outing and are not part of the engineer's setup
   progression (Setup_change_overview); the tool's focus is the physical
   setup change. The thesis comparison with the engineer's changes reads the
   setup recommendation only (D8 amendment).
6. Limiting-end tie-break (4h, MM p. 409: improve the limiting end first,
   degrade the other end only when that is not possible): the limiting axle
   is front for US and rear for OS / INST; a row on that axle ranks before a
   global row (diff, TC, bias, ABS), which ranks before a row on the other
   axle. It replaces the row order of mechanisms.json; per corner it ranks
   right after repetition and ahead of preference (author 2026-10-23:
   physics before engineer preference).
7. Session setup recommendation (4h): the setup is changed for the whole
   lap, so the corner output is a diagnosis and the setup decision is one
   change per session (MM p. 572 item 4: balance cannot be optimal
   everywhere, seek the best circuit performance; MM p. 391: one change at
   a time; TE p. 254: align the settings to the most important corners --
   corner importance is NOT weighted here, a stated limitation). Every
   counted symptom votes for its mechanical lever groups with weight
   severity x k / n; votes for the same lever group in the opposite
   direction are subtracted. Not counted: a row whose condition fails (diff
   increase on a traction-limited exit, MM p. 740) and low-priority corners
   (high speed, front limiting only; MM p. 396 item 21, p. 401 item 1F),
   which stay in the per-corner diagnosis. Rank: net support -> limiting-end class ->
   preference -> change effort -> headroom; an exact tie on all keys has no
   winner and is reported as equivalent options.

Two layers (D6): the data layer never sees driver feedback and is stored as
it is (thesis comparison); the engineer layer adds feedback ordinally (agree /
neutral / conflict) and driver-only rows for strong complaints without data
support, which also vote at session level with weight 1 x 1. The session
layer (D7) states the share of corners per symptom and the per-lap share of
limiting cells beside the session recommendation.
"""

import copy
import json
from fractions import Fraction

import numpy as np

from modules.classifier import phase_symptoms, is_low_priority, verdict_cs_stat, _thresholds
from modules.corner_aggregation import (
    PHASE_KEYS, PHASE_TO_FEEDBACK_KEY, SEVERITY_RANK, _group_by_corner, aggregate_by_corner,
)
from modules.setup_state import (
    load_setup_parameters_registry, load_setup_windows, logged_setup_values, _current_setup_value,
    _check_feasible, _window_edge_check, _edge_normal_state_label, window_headroom,
)

MECHANISMS_CONFIG_PATH = "config/mechanisms.json"

# ordinal enums, method-defining (D4: cheaper changes first, MM p. 391)
EFFORT_RANK = {"seconds": 0, "minutes": 1, "half_hour": 2, "garage_hours": 3}
AGREEMENT_RANK = {"agree": 2, "neutral": 1, "conflict": 0}

# feedback scale -5 (undrivable understeer) .. +5 (undrivable oversteer);
# FEEDBACK_MIN counts as a statement, DRIVER_ONLY_MIN is "approaching
# undrivable" (the scale's own |4| anchor, D6)
FEEDBACK_MIN = 1
DRIVER_ONLY_MIN = 4

_OPPOSITE_WORD = {"stiffer": "softer", "softer": "stiffer"}
# the axle whose grip limits, per symptom (D2): US front, OS / INST rear
LIMITING_AXLE = {"US": "front", "OS": "rear", "INST": "rear"}
# the elicited matrix's direction words in the table's vocabulary
# (more_negative / less_negative go through the registry semantics)
# registry escalation_tier of the driver-adjustable levers (4g split)
COCKPIT_TIER = "cockpit"
_CELL_WORD = {"soften": "softer", "stiffen": "stiffer", "more_ra_stability": "toward the rear-axle-stable map"}
_SIGN_KEYS = {"more_negative": -1, "negative": -1, "more_positive": 1, "positive": 1}


def load_mechanisms():
    with open(MECHANISMS_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def direction_sign(entry, direction):
    """+1 / -1 on the lever's numeric scale, or None when the registry gives
    no convention (brake bias, ABS map). Read from direction_semantics so a
    damper's "increasing = softer" is never hard-coded."""
    if direction == "increase":
        return 1
    if direction == "decrease":
        return -1
    ds = entry.get("direction_semantics") or {}
    inc = ds.get("increasing")
    if inc == direction:
        return 1
    if inc is not None and _OPPOSITE_WORD.get(inc) == direction:
        return -1
    for key, word in ds.items():
        if key not in _SIGN_KEYS or not isinstance(word, str):
            continue
        if word == direction:
            return _SIGN_KEYS[key]
        if direction.startswith("less ") and word == "more " + direction[5:]:
            return -_SIGN_KEYS[key]
    return None


def lever_step(entry, damper_step_key="fine"):
    # registry adjustment_step, else value_space.step, else one unit (one
    # option for an enum)
    st = entry.get("adjustment_step")
    if isinstance(st, dict):
        return st.get(damper_step_key, 1)
    if isinstance(st, (int, float)):
        return st
    vs = entry.get("value_space") or {}
    if vs.get("type") != "enum" and vs.get("step"):
        return vs["step"]
    return 1


def _lever_change(levers, direction, setup_data, registry, windows, damper_step_key, logged=None):
    """Signed step, from -> to and feasibility for one lever group (left and
    right always move together). A lever without a sheet field takes its
    current value from a logged channel when one exists (ABS map)."""
    entries = [registry[k] for k in levers]
    sign = direction_sign(entries[0], direction)
    step = lever_step(entries[0], damper_step_key)
    per_key, notes, headrooms = [], [], []
    for key, entry in zip(levers, entries):
        current = _current_setup_value(setup_data, entry)
        if current is None and (logged or {}).get(key):
            current = logged[key]["value"]
            if logged[key]["note"]:
                notes.append(f"{key}: {logged[key]['note']}")
        if sign is None or current is None:
            per_key.append({"parameter": key, "from": current, "to": None, "feasible": None})
            continue
        vs = entry.get("value_space") or {}
        if vs.get("type") == "enum":
            delta = sign  # one option along the ordered enum
            ok = _check_feasible(entry, current, delta)
            to = vs["options"][vs["options"].index(current) + delta] if ok else None
        else:
            delta = sign * step
            ok = _check_feasible(entry, current, delta)
            try:
                to = round(float(current) + delta, 3) if ok is not False else None
            except (TypeError, ValueError):
                to = None
        per_key.append({"parameter": key, "from": current, "to": to, "feasible": ok})
        action = {"parameter": key, "delta": delta, "direction": direction}
        edge = _window_edge_check(action, setup_data, registry, windows)
        if edge is not None and edge[0] == "soft":
            label = _edge_normal_state_label(action, windows)
            notes.append(f"{key}: {label}" if label else f"{key}: beyond the typical window (engineer may exceed)")
        h = window_headroom(key, current, delta, registry, windows) if vs.get("type") != "enum" else None
        if h is not None:
            headrooms.append(h)
    states = [k["feasible"] for k in per_key]
    if any(s is False for s in states):
        feasibility = "at_limit"
    elif states and all(s is True for s in states):
        feasibility = "ok"
    else:
        feasibility = "not_assessable"
    return {
        "sign": sign, "step": step, "keys": per_key, "feasibility": feasibility,
        "headroom": min(headrooms) if headrooms else None, "notes": notes,
        "effort": entries[0].get("change_effort"),
    }


def _phase_group(phase, mechanisms):
    for g, members in mechanisms["phase_groups"].items():
        if phase in members:
            return g
    return None


def axle_class(row_axle, limiting):
    """0 = row acts on the limiting axle, 1 = global row (or no single
    limiting axle), 2 = row acts on the other axle (MM p. 409)."""
    if row_axle == "global" or limiting is None:
        return 1
    return 0 if row_axle == limiting else 2


def _matching_rows(symptom, phase, speed_class, mechanisms):
    group = _phase_group(phase, mechanisms)
    rows = [r for r in mechanisms["mechanisms"]
            if group in r["phases"] and (speed_class is None and len(r["speed_classes"]) == 3
                                         or speed_class in r["speed_classes"])]
    if symptom != "INST":
        return [r for r in rows if symptom in r["symptoms"]]
    # INST takes the OS direction of a lever unless a row names INST for that
    # lever group in this phase (rear saturation = stability loss)
    explicit = [r for r in rows if "INST" in r["symptoms"]]
    covered = {tuple(r["levers"]) for r in explicit}
    inherited = [r for r in rows if "OS" in r["symptoms"] and "INST" not in r["symptoms"]
                 and tuple(r["levers"]) not in covered]
    return explicit + inherited


def _cell_word(entry, word):
    if word in _CELL_WORD:
        return _CELL_WORD[word]
    ds = (entry or {}).get("direction_semantics") or {}
    if isinstance(ds.get(word), str):
        return ds[word]
    if word.startswith("less_") and isinstance(ds.get("more_" + word[5:]), str):
        w = ds["more_" + word[5:]]
        return "less " + w[5:] if w.startswith("more ") else None
    return word


def cell_supports(cell, row, registry):
    """True when the cell names one of the row's levers with the row's
    direction. A cell that contradicts the table, or names a lever the table
    has no row for at that symptom / phase / speed, supports nothing and is
    ignored (author ruling 2026-10-22, 4e item 6)."""
    for lever, word in zip(cell["levers"], cell["cell_directions"]):
        if lever not in row["levers"]:
            continue
        entry = registry.get(lever) or {}
        mapped = _cell_word(entry, word)
        if mapped == row["direction"]:
            return True
        sign = direction_sign(entry, mapped) if mapped else None
        if sign is not None and sign == direction_sign(entry, row["direction"]):
            return True
    return False


def _cell_applies(cell, symptom, phase, speed_class, mechanisms):
    return (cell["symptom"] == symptom and cell["phase"] == _phase_group(phase, mechanisms)
            and cell["speed_class"] == speed_class)


def _preference(row, symptom, phase, speed_class, mechanisms, registry):
    return int(any(_cell_applies(cell, symptom, phase, speed_class, mechanisms)
                   and cell_supports(cell, row, registry)
                   for cell in mechanisms.get("engineer_preference", [])))


def unsupported_preference_cells(mechanisms, registry):
    # cells that no table row at their own symptom / phase group / speed
    # class supports -- the list recorded in mechanisms.json
    out = []
    for cell in mechanisms.get("engineer_preference", []):
        phase = mechanisms["phase_groups"][cell["phase"]][0]
        rows = _matching_rows(cell["symptom"], phase, cell["speed_class"], mechanisms)
        if not any(cell_supports(cell, r, registry) for r in rows):
            out.append(cell["cell"])
    return out


def tc_activity(corners, state, channels):
    """{(stable_corner_id, lap_number, phase): bool} -- TC flag (ecu_B_tc_act)
    set anywhere inside that corner-lap's phase window. Empty when the
    channel or the time base is missing."""
    tc_ch = (channels or {}).get("ecu_B_tc_act")
    if state is None or tc_ch is None or tc_ch.get("quality") in ("missing", "failed") or tc_ch.get("time") is None:
        return {}
    t = np.asarray(state["time"])
    on = np.interp(t, tc_ch["time"], tc_ch["data"]) > 0.5  # boolean channel
    out = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is None:
            continue
        for phase, seg in (c.get("segments") or {}).items():
            if seg is None or seg[1] < seg[0]:
                continue
            lo, hi = int(np.searchsorted(t, seg[0], "left")), int(np.searchsorted(t, seg[1], "right"))
            out[(cid, c.get("lap_number"), phase)] = bool(hi > lo and on[lo:hi].any())
    return out


def repeats(k, n, frac, min_laps):
    # D5: strictly more than frac of the n valid laps, and at least min_laps
    return k > frac * n and k >= min_laps


def counted_symptoms(summaries, cfg, thr=None):
    """Per corner: the symptoms that pass the repetition rule (D5).

    Returns {cid: {"n_laps", "speed_class", "symptoms": [...]}}; a symptom is
    {"symptom", "phase", "laps", "lap_numbers", "severity", "value"}."""
    thr = thr or _thresholds()
    frac, min_laps = cfg["min_repeat_fraction"], cfg["min_repeat_laps"]
    aggregated = aggregate_by_corner(summaries)
    out = {}
    for cid, laps in sorted(_group_by_corner(summaries).items()):
        n = len(laps)
        hits = {}
        for s in laps:
            seen = set()
            for sym in phase_symptoms(s, thr)["symptoms"]:
                k = (sym["phase"], sym["symptom"])
                if k in seen:
                    continue
                seen.add(k)
                hits.setdefault(k, []).append(s["lap_number"])
        agg = aggregated[cid]
        agg_syms = {(x["phase"], x["symptom"]): x for x in phase_symptoms(agg, thr)["symptoms"]}
        counted = []
        for (phase, symptom), lap_numbers in hits.items():
            if not repeats(len(lap_numbers), n, frac, min_laps):
                continue
            a = agg_syms.get((phase, symptom))
            # the aggregate is the worst lap for CS and the median for
            # stability, so a majority symptom is present in it; the per-lap
            # fallback only guards an aggregation without that property
            counted.append({"symptom": symptom, "phase": phase, "laps": len(lap_numbers),
                            "lap_numbers": sorted(lap_numbers),
                            "severity": a["severity"] if a else "moderate",
                            "value": a["value"] if a else None})
        counted.sort(key=lambda x: (PHASE_KEYS.index(x["phase"]), x["symptom"]))
        out[cid] = {"n_laps": n, "speed_class": agg.get("speed_class"), "symptoms": counted}
    return out


def _candidates(cid, corner, mechanisms, registry, windows, setup_data, tc_map, cfg, logged=None):
    cands = []
    for sym in corner["symptoms"]:
        for row in _matching_rows(sym["symptom"], sym["phase"], corner["speed_class"], mechanisms):
            ch = _lever_change(row["levers"], row["direction"], setup_data, registry, windows, cfg["damper_step"],
                               logged)
            demoted, cond_note = False, None
            if row.get("condition") == "not_traction_limited":
                flags = [tc_map.get((cid, lap, sym["phase"])) for lap in sym["lap_numbers"]]
                n_on = sum(bool(f) for f in flags)
                if flags and n_on > cfg["min_repeat_fraction"] * len(flags):
                    demoted = True
                    cond_note = f"traction-limited: TC active on {n_on}/{len(flags)} laps -> tc_lon first"
            elif row.get("condition") == "outside_stall_regime":
                # no ride-height / ground-clearance signal decides the stall
                # regime here, so the condition is stated for the engineer,
                # not tested; the candidate keeps its rank
                cond_note = "valid outside the ground-effect stall regime (engineer check)"
            cands.append({
                "corner": cid, "phase": sym["phase"], "symptom": sym["symptom"], "severity": sym["severity"],
                "laps": sym["laps"], "n_laps": corner["n_laps"], "mechanism_id": row["id"],
                "mechanism": row["mechanism"], "mechanism_name": mechanisms["mechanism_names"][row["mechanism"]],
                "levers": row["levers"], "direction": row["direction"], "anchor": row["anchor"],
                "advisory": row["advisory"], "note": row.get("note") or "",
                "side_effect": row.get("side_effect"), "change": ch, "condition_demoted": demoted,
                "condition_note": cond_note,
                "preference": _preference(row, sym["symptom"], sym["phase"], corner["speed_class"],
                                          mechanisms, registry),
                "cockpit": all(registry[k].get("escalation_tier") == COCKPIT_TIER for k in row["levers"]),
                "axle": row["axle"],
                "axle_class": axle_class(row["axle"], LIMITING_AXLE[sym["symptom"]]),
            })
    return cands


def _data_rank_key(c):
    # limiting-end class before preference (4h ruling b): which axle to work
    # on is physics (MM p. 409), the engineer's choice orders the levers
    # within it; preference before effort (4e), effort visible on every line.
    # The traction demotion stays ahead of both (4e ruling)
    return (-SEVERITY_RANK[c["severity"]], -c["laps"], c["condition_demoted"], c["axle_class"], -c["preference"],
            EFFORT_RANK.get(c["change"]["effort"], len(EFFORT_RANK)),
            -(c["change"]["headroom"] if c["change"]["headroom"] is not None else 0.0))


def _select(cands, rank_key, max_alt):
    """One change per corner: drop at-limit levers, merge the same lever
    change across phases, set aside opposite directions of an already chosen
    lever (conflict), then take the best and up to max_alt alternatives."""
    at_limit = [c for c in cands if c["change"]["feasibility"] == "at_limit"]
    ranked = sorted((c for c in cands if c["change"]["feasibility"] != "at_limit"), key=rank_key)
    chosen, conflicts, seen = [], [], {}
    for c in ranked:
        k = tuple(c["levers"])
        if k in seen:
            first = seen[k]
            if first["direction"] == c["direction"]:
                first.setdefault("also", []).append(f"{c['symptom']} @ {c['phase']}")
            else:
                conflicts.append(c)
            continue
        seen[k] = c
        chosen.append(c)
    # ranked keeps the whole order, so the engineer layer can re-rank it
    return {
        "recommended": chosen[0] if chosen else None,
        "alternatives": chosen[1:1 + max_alt],
        "ranked": chosen, "at_limit": at_limit, "conflicts": conflicts,
    }


def _split_select(cands, rank_key, max_alt):
    """Setup recommendation from the mechanical candidates (with alternatives),
    and the single best cockpit candidate beside it -- same rank key, so the
    cockpit line is the lever the mechanical ranking would have picked had it
    been allowed to."""
    setup = _select([c for c in cands if not c["cockpit"]], rank_key, max_alt)
    cockpit = _select([c for c in cands if c["cockpit"]], rank_key, 0)
    return {**setup, "cockpit": cockpit["recommended"], "cockpit_ranked": cockpit["ranked"],
            "cockpit_conflicts": cockpit["conflicts"], "cockpit_at_limit": cockpit["at_limit"]}


def cell_shares(summaries, thr=None):
    """Per-lap corner-phase cells WITHOUT the repetition rule: how often a
    single lap shows front limiting (US), rear limiting (OS) or INST. Each
    symptom has its own denominator -- US / OS over the cells with a CS value
    on that axle, INST over the cells with a stability value -- so a phase
    without stability data does not dilute the CS shares (ruling 4e B)."""
    thr = thr or _thresholds()
    n = {"US": 0, "OS": 0, "INST": 0}
    k = {"US": 0, "OS": 0, "INST": 0}
    for s in summaries:
        hit = {(x["phase"], x["symptom"]) for x in phase_symptoms(s, thr)["symptoms"]}
        for phase in PHASE_KEYS:
            p = s["phases"].get(phase)
            if p is None:
                continue
            vals = {"US": verdict_cs_stat(s, phase, "f")["median"], "OS": verdict_cs_stat(s, phase, "r")["median"],
                    "INST": p["stability_observed_Nm_per_deg"]["median"]}
            for sym, v in vals.items():
                if v is not None and v == v:
                    n[sym] += 1
                    k[sym] += (phase, sym) in hit
    return {sym: {"k": k[sym], "n": n[sym], "share": (k[sym] / n[sym]) if n[sym] else None} for sym in n}


def option_key(levers, sign):
    return f"{'+'.join(levers)} {sign:+d}"


def symptom_votes(cid, corner, cands, vote_cfg, driver_only=False, excluded=None):
    """One vote per counted symptom and mechanical lever group, signed by
    the registry direction. Weight = severity weight x k / n, so a strong
    symptom on every lap weighs twice a moderate one on every lap, and a
    symptom on a bare majority of laps weighs about half. A driver-only
    symptom has no lap count and weighs driver_only_weight x 1."""
    votes = []
    for sym in corner["symptoms"]:
        seen = set()
        for c in cands:
            if (c["cockpit"] or c["phase"] != sym["phase"] or c["symptom"] != sym["symptom"]
                    or c["change"]["sign"] is None):
                continue
            key = option_key(c["levers"], c["change"]["sign"])
            if key in seen:
                continue
            seen.add(key)
            if c["condition_demoted"]:
                # the row's own condition is not met (TC active on most
                # symptom laps: traction-limited, MM p. 740), so the row does
                # not apply and cannot vote
                if excluded is not None:
                    excluded.append({"corner": cid, "symptom": sym["symptom"], "phase": sym["phase"],
                                     "option": key, "reason": c["condition_note"]})
                continue
            if driver_only:
                w, k, n = vote_cfg["driver_only_weight"], 1, 1
            else:
                w, k, n = vote_cfg["severity_weight"][sym["severity"]], sym["laps"], corner["n_laps"]
            votes.append({"option": key, "corner": cid, "symptom": sym["symptom"], "phase": sym["phase"],
                          "severity": sym["severity"], "laps": sym["laps"], "n_laps": corner["n_laps"],
                          "severity_weight": w, "k": k, "n": n, "weight": w * k / n, "driver_only": driver_only,
                          "mechanism_id": c["mechanism_id"], "preference": c["preference"]})
    return votes


def option_details(cands, details):
    # lever-level facts of each session option; the change (from -> to,
    # feasibility, headroom) depends on the setup sheet only, so the first
    # candidate of a lever group carries it for all corners
    for c in cands:
        if c["cockpit"] or c["change"]["sign"] is None:
            continue
        key = option_key(c["levers"], c["change"]["sign"])
        d = details.setdefault(key, {
            "option": key, "levers": c["levers"], "direction": c["direction"], "sign": c["change"]["sign"],
            "axle": c["axle"], "change": c["change"], "mechanism": c["mechanism"],
            "mechanism_name": c["mechanism_name"], "mechanism_ids": [], "anchors": [], "advisory": False,
            "condition_notes": [], "side_effect": c["side_effect"]})
        for field, value in (("mechanism_ids", c["mechanism_id"]), ("anchors", c["anchor"]),
                             ("condition_notes", c["condition_note"])):
            if value and value not in d[field]:
                d[field].append(value)
        d["advisory"] = d["advisory"] or bool(c["advisory"])
    return details


def _vote_ref(v):
    return {k: v[k] for k in ("corner", "symptom", "phase", "severity", "laps", "n_laps", "weight",
                              "driver_only", "feedback", "agreement") if k in v}


def session_vote(votes, details, max_alt):
    """Session setup recommendation from the corner votes (4h).

    Net support = support - opposition, where opposition is the weight of
    every vote for the same lever group in the opposite direction (the two
    directions are one decision, so net(+) = -net(-)). Only net > 0 can be
    recommended. Sums are exact fractions so that an exact tie is a real
    tie and not a rounding accident."""
    def weight(v):
        return Fraction(v["severity_weight"]) * Fraction(v["k"], v["n"])

    by = {}
    for v in votes:
        by.setdefault(v["option"], []).append(v)
    options = []
    for key, sup in by.items():
        d = details[key]
        opp = by.get(option_key(d["levers"], -d["sign"]), [])
        support = sum(map(weight, sup), Fraction(0))
        opposition = sum(map(weight, opp), Fraction(0))
        # the option's limiting axle is the one its supporting votes report;
        # a lever group voted for by front and rear symptoms with equal weight
        # (LS dampers across phases) has none and ranks as a global row
        front = sum((weight(v) for v in sup if LIMITING_AXLE[v["symptom"]] == "front"), Fraction(0))
        limiting = "front" if 2 * front > support else "rear" if 2 * front < support else None
        options.append({**d, "support": float(support), "opposition": float(opposition),
                        "net": float(support - opposition), "_net": support - opposition,
                        "limiting_axle": limiting, "axle_class": axle_class(d["axle"], limiting),
                        "preference": max(v["preference"] for v in sup),
                        "supporting": [_vote_ref(v) for v in sup], "opposing": [_vote_ref(v) for v in opp]})

    def rank(o):
        return (-o["_net"], o["axle_class"], -o["preference"],
                EFFORT_RANK.get(o["change"]["effort"], len(EFFORT_RANK)),
                -(o["change"]["headroom"] if o["change"]["headroom"] is not None else 0.0))

    # the option name only makes the stored order deterministic, it never
    # breaks a tie
    options.sort(key=lambda o: (rank(o), o["option"]))
    positive = [o for o in options if o["_net"] > 0]
    ranked = [o for o in positive if o["change"]["feasibility"] != "at_limit"]
    top = [o for o in ranked if rank(o) == rank(ranked[0])] if ranked else []
    out = {
        "recommended": top[0] if len(top) == 1 else None,
        "equivalent_options": top if len(top) > 1 else [],
        "alternatives": ranked[len(top):len(top) + max_alt],
        "ranked": ranked,
        "at_limit": [o for o in positive if o["change"]["feasibility"] == "at_limit"],
        "not_recommended": [o for o in options if o["_net"] <= 0],
        "votes": votes,
    }
    for o in options:
        o.pop("_net")
    return out


def _all_options(rec):
    return rec["ranked"] + rec["at_limit"] + rec["not_recommended"]


def _session_layer(counted, summaries, cfg, thr=None):
    n = len(counted)
    shares = {}
    for sym in ("US", "OS", "INST"):
        k = sum(any(s["symptom"] == sym for s in c["symptoms"]) for c in counted.values())
        shares[sym] = {"corners": k, "share": (k / n) if n else None}
    n_counted = sum(bool(c["symptoms"]) for c in counted.values())
    thr_share = cfg.get("session_share_threshold")
    pattern = None if thr_share is None else [sym for sym, v in shares.items()
                                               if v["share"] is not None and v["share"] >= thr_share]
    return {"n_corners": n, "shares": shares, "corners_counted": n_counted,
            "counted_share": (n_counted / n) if n else None, "cells": cell_shares(summaries, thr),
            "threshold": thr_share, "pattern": pattern,
            "note": "shares reported only (session_share_threshold not set); the session setup recommendation "
                    "comes from the corner vote" if thr_share is None else ""}


def data_layer(summaries, setup_data=None, registry=None, mechanisms=None, windows=None, tc_map=None, cfg=None,
               logged=None):
    """The data-only recommendation (D6): no feedback enters here."""
    from modules.stability_analysis import load_parameters
    cfg = cfg or load_parameters()["setup_engine"]
    registry = registry or load_setup_parameters_registry()
    mechanisms = mechanisms or load_mechanisms()
    windows = windows or load_setup_windows()
    tc_map = tc_map or {}
    counted = counted_symptoms(summaries, cfg)
    corners, votes, details, excluded = [], [], {}, []
    for cid, corner in counted.items():
        if not corner["symptoms"]:
            continue
        cands = _candidates(cid, corner, mechanisms, registry, windows, setup_data, tc_map, cfg, logged)
        if is_low_priority(corner["symptoms"], corner["speed_class"]):
            # high-speed front-only limiting (MM p. 396 item 21, p. 401 item
            # 1F): diagnosis only, no session vote
            excluded += [{"corner": cid, "symptom": s["symptom"], "phase": s["phase"], "option": None,
                          "reason": "low priority (high speed, front limiting only)"} for s in corner["symptoms"]]
        else:
            votes += symptom_votes(cid, corner, cands, cfg["session_vote"], excluded=excluded)
        option_details(cands, details)
        sel = _split_select(cands, _data_rank_key, cfg["max_alternatives"])
        top = max(corner["symptoms"], key=lambda s: (SEVERITY_RANK[s["severity"]], s["laps"]))
        corners.append({
            "corner": cid, "speed_class": corner["speed_class"], "n_laps": corner["n_laps"],
            "symptoms": corner["symptoms"],
            "low_priority": is_low_priority(corner["symptoms"], corner["speed_class"]),
            "severity": top["severity"], "laps": top["laps"], **sel,
        })
    # low_priority sorts after the other corners of the same severity
    corners.sort(key=lambda c: (-SEVERITY_RANK[c["severity"]], c["low_priority"], -c["laps"], c["corner"]))
    return {"corners": corners, "session": _session_layer(counted, summaries, cfg),
            "session_recommendation": dict(session_vote(votes, details, cfg["max_alternatives"]),
                                           excluded=excluded),
            "rules": {"min_repeat_fraction": cfg["min_repeat_fraction"], "min_repeat_laps": cfg["min_repeat_laps"],
                      "damper_step": cfg["damper_step"]}}


def _feedback_value(feedback_data, cid, phase):
    rows = (feedback_data or {}).get("corners") or []
    if not isinstance(cid, int) or not 1 <= cid <= len(rows):
        return 0
    return rows[cid - 1].get(PHASE_TO_FEEDBACK_KEY[phase], 0) or 0


def agreement(symptom, fb):
    if abs(fb) < FEEDBACK_MIN:
        return "neutral"
    if symptom == "US":
        return "agree" if fb < 0 else "conflict"
    # OS and INST: rear saturation is a loss of stability (Hoffman 2008),
    # which the driver feels as a loose rear, i.e. oversteer feedback
    return "agree" if fb > 0 else "conflict"


def engineer_layer(data, summaries, feedback_data, setup_data=None, registry=None, mechanisms=None,
                   windows=None, cfg=None, logged=None):
    """Data layer plus driver feedback (D6): agreement ranks right after
    severity; |fb| >= DRIVER_ONLY_MIN without a data symptom adds a
    driver-only row labelled "no data support". Driver level is not used."""
    from modules.stability_analysis import load_parameters
    cfg = cfg or load_parameters()["setup_engine"]
    registry = registry or load_setup_parameters_registry()
    mechanisms = mechanisms or load_mechanisms()
    windows = windows or load_setup_windows()
    layer = copy.deepcopy(data)

    def key(c):
        base = _data_rank_key(c)
        return (base[0], -AGREEMENT_RANK[c["agreement"]]) + base[1:]

    for corner in layer["corners"]:
        pool = corner["ranked"] + corner["conflicts"] + corner["cockpit_ranked"] + corner["cockpit_conflicts"]
        for c in pool:
            c["feedback"] = _feedback_value(feedback_data, corner["corner"], c["phase"])
            c["agreement"] = agreement(c["symptom"], c["feedback"])
        sel = _split_select(pool, key, cfg["max_alternatives"])
        # at-limit lists carry no feedback rank, they stay as in the data layer
        sel.pop("at_limit")
        sel.pop("cockpit_at_limit")
        corner.update(sel)

    votes = copy.deepcopy(data["session_recommendation"]["votes"])
    for v in votes:
        v["feedback"] = _feedback_value(feedback_data, v["corner"], v["phase"])
        v["agreement"] = agreement(v["symptom"], v["feedback"])
    details = {o["option"]: o for o in copy.deepcopy(_all_options(data["session_recommendation"]))}

    # driver-only rows: strong complaint, no counted data symptom there
    counted = {c["corner"]: c for c in layer["corners"]}
    speed = {cid: a.get("speed_class") for cid, a in aggregate_by_corner(summaries).items()}
    rows = (feedback_data or {}).get("corners") or []
    driver_only = []
    for i, _row in enumerate(rows):
        cid = i + 1
        for phase in PHASE_KEYS:
            fb = _feedback_value(feedback_data, cid, phase)
            if abs(fb) < DRIVER_ONLY_MIN:
                continue
            if any(s["phase"] == phase for s in (counted.get(cid) or {}).get("symptoms", [])):
                continue
            symptom = "US" if fb < 0 else "OS"
            corner = {"n_laps": 0, "speed_class": speed.get(cid),
                      "symptoms": [{"symptom": symptom, "phase": phase, "laps": 0, "lap_numbers": [],
                                    "severity": "normal", "value": None}]}
            cands = _candidates(cid, corner, mechanisms, registry, windows, setup_data, {}, cfg, logged)
            for c in cands:
                c.update(feedback=fb, agreement="driver_only", label="no data support")
            sel = _split_select(cands, _data_rank_key, cfg["max_alternatives"])
            driver_only.append({"corner": cid, "phase": phase, "symptom": symptom, "feedback": fb,
                                "speed_class": speed.get(cid), **sel})
            for v in symptom_votes(cid, corner, cands, cfg["session_vote"], driver_only=True):
                v.update(feedback=fb, agreement="driver_only")
                votes.append(v)
            option_details(cands, details)
    layer["driver_only"] = driver_only
    # same vote as the data layer plus the driver-only votes; feedback
    # weights nothing, it is shown per supporting corner
    layer["session_recommendation"] = dict(session_vote(votes, details, cfg["max_alternatives"]),
                                           excluded=copy.deepcopy(data["session_recommendation"]["excluded"]))
    layer["feedback_used"] = bool(rows)
    return layer


def run(summaries, setup_data=None, feedback_data=None, corners=None, state=None, channels=None, laps=None):
    """Both layers for one analysis: data layer first (stored as is), then the
    engineer layer built on a copy of it."""
    from modules.stability_analysis import load_parameters
    cfg = load_parameters()["setup_engine"]
    registry = load_setup_parameters_registry()
    mechanisms = load_mechanisms()
    windows = load_setup_windows()
    tc_map = tc_activity(corners, state, channels)
    logged = logged_setup_values(channels, laps, {s["lap_number"] for s in summaries})
    data = data_layer(summaries, setup_data, registry, mechanisms, windows, tc_map, cfg, logged)
    eng = engineer_layer(data, summaries, feedback_data, setup_data, registry, mechanisms, windows, cfg, logged)
    return {"data_layer": data, "engineer_layer": eng}


def change_text(c):
    # "arb_fl+arb_fr softer (4 -> 3, 4 -> 3)" -- one line per candidate
    ch = c["change"]
    levers = "+".join(c["levers"])
    vals = ", ".join(f"{k['from']} -> {k['to']}" for k in ch["keys"] if k["from"] is not None and k["to"] is not None)
    if ch["feasibility"] == "not_assessable" and not vals:
        step = "" if ch["sign"] is None and c["levers"] == ["abs_position"] else f", step {ch['step']}"
        known = [str(k["from"]) for k in ch["keys"] if k["from"] is not None]
        if known:
            # categorical lever (ABS map): the current position is known, the
            # target map is the engineer's choice
            return f"{levers} {c['direction']}{step} (current {', '.join(known)})"
        return f"{levers} {c['direction']}{step} (current value not assessable)"
    return f"{levers} {c['direction']} ({vals})" if vals else f"{levers} {c['direction']}"


_PHASE_SHORT = {"entry_1_brake": "brake", "entry_2_turnin": "turn-in", "apex_3": "apex",
                "exit_4": "early exit", "exit_5": "late exit"}
_SYMPTOM_WORD = {"US": "understeer", "OS": "oversteer", "INST": "unstable yaw"}


def candidate_line(c, engineer=False):
    # one line per change: symptom and phase, laps, change from -> to,
    # effort, mechanism, source, and in the engineer layer the feedback flag
    where = f"{_SYMPTOM_WORD[c['symptom']]} @ {_PHASE_SHORT[c['phase']]}"
    if c.get("also"):
        where += " (also " + ", ".join(c["also"]) + ")"
    laps = f"{c['laps']}/{c['n_laps']} laps" if c.get("n_laps") else "no data laps"
    parts = [where, c["severity"], laps, change_text(c), c["change"]["effort"] or "effort ?",
             f"{c['mechanism']} {c['mechanism_name']}", c["anchor"]]
    if c["advisory"]:
        parts.append("advisory")
    if c.get("condition_note"):
        parts.append(c["condition_note"])
    parts += c["change"]["notes"]
    if engineer:
        flag = c.get("label") or c.get("agreement")
        if flag:
            parts.append(f"feedback {c.get('feedback', 0):+g}: {flag}")
    return " | ".join(str(p) for p in parts)


def _vote_text(v, engineer):
    t = f"C{v['corner']} {_SYMPTOM_WORD[v['symptom']]} @ {_PHASE_SHORT[v['phase']]} {v['weight']:.2f}"
    if v.get("driver_only"):
        return t + " (driver only, no data support)"
    if engineer and v.get("agreement"):
        t += f" ({v['agreement']})"
    return t


def option_line(o):
    parts = [change_text(o), o["change"]["effort"] or "effort ?", f"{o['mechanism']} {o['mechanism_name']}",
             "; ".join(o["anchors"]),
             f"net support {o['net']:.2f} (support {o['support']:.2f}, opposition {o['opposition']:.2f})"]
    if o["advisory"]:
        parts.append("advisory")
    parts += o["condition_notes"] + o["change"]["notes"]
    return " | ".join(str(p) for p in parts)


def session_details(rec, engineer=False):
    """Full session vote for the box details: every option with its from ->
    to, effort, mechanism, sources and the vote numbers."""
    def block(prefix, o):
        return [prefix + option_line(o),
                "    supporting: " + (", ".join(_vote_text(v, engineer) for v in o["supporting"]) or "none"),
                "    opposing: " + (", ".join(_vote_text(v, engineer) for v in o["opposing"]) or "none")]

    lines = []
    if rec["recommended"]:
        lines += block("change: ", rec["recommended"])
    elif rec["equivalent_options"]:
        lines.append("equivalent options (exact tie on every rank key, no winner):")
        for o in rec["equivalent_options"]:
            lines += block("  option: ", o)
    else:
        lines.append("no mechanical lever group with positive net support")
    for o in rec["alternatives"]:
        lines += block("  alternative: ", o)
    lines += ["  at limit: " + option_line(o) for o in rec["at_limit"]]
    if rec.get("excluded"):
        lines.append("  not voting: " + ", ".join(
            f"C{x['corner']} {_SYMPTOM_WORD[x['symptom']]} @ {_PHASE_SHORT[x['phase']]}"
            + (f" {x['option']}" if x["option"] else "") + f" ({x['reason']})" for x in rec["excluded"]))
    return lines


def corner_details(corner, engineer=False):
    """Full candidate lines of one corner for the box details."""
    lines = []
    if corner.get("recommended"):
        lines.append("candidate: " + candidate_line(corner["recommended"], engineer))
    lines += ["alternative: " + candidate_line(a, engineer) for a in corner.get("alternatives", [])]
    lines += ["at limit: " + change_text(a) for a in corner.get("at_limit", [])]
    if not lines:
        lines.append("no mechanical lever in the mechanism table for this symptom, phase and speed class")
    if corner.get("cockpit"):
        lines.append("driver can also: " + candidate_line(corner["cockpit"], engineer))
    return lines


# Panel text (UI-1, author 2026-10-24): engineer-style short lines; the
# sources, vote numbers, mechanism ids and effort classes stay in the details
_LEVER_SHORT = {
    ("arb_fl", "arb_fr"): "front ARB", ("arb_rl", "arb_rr"): "rear ARB",
    ("springs_front",): "front springs", ("springs_rear",): "rear springs",
    ("damper_bump_ls_fl", "damper_bump_ls_fr"): "front LS bump",
    ("damper_bump_ls_rl", "damper_bump_ls_rr"): "rear LS bump",
    ("damper_rebound_ls_fl", "damper_rebound_ls_fr"): "front LS rebound",
    ("damper_rebound_ls_rl", "damper_rebound_ls_rr"): "rear LS rebound",
    ("wing_position",): "wing", ("splitter_offset",): "splitter",
    ("ride_height_front",): "front ride height", ("ride_height_rear",): "rear ride height",
    ("camber_fl", "camber_fr"): "front camber", ("camber_rl", "camber_rr"): "rear camber",
    ("toe_front",): "front toe", ("toe_rear",): "rear toe",
    ("diff_position",): "diff", ("tc_lat",): "TC lat", ("tc_lon",): "TC lon",
    ("brake_bias",): "brake bias", ("abs_position",): "ABS map",
}
_DIRECTION_SHORT = {"more front": "forward", "more rear": "rearward",
                    "toward the rear-axle-stable map": "toward the rear-stable map",
                    "more negative camber": "more negative", "less negative camber": "less negative"}
_PHASE_WORD = {"entry_1_brake": "brake", "entry_2_turnin": "turn-in", "apex_3": "apex",
               "exit_4": "exit", "exit_5": "exit"}


def lever_name(levers):
    return _LEVER_SHORT.get(tuple(levers), "+".join(levers))


def lever_text(c, capital=False, step_count=False):
    # "rear ARB softer", "diff +1", "brake bias forward"; the step count
    # ("(1 step)") only where the change itself is the headline
    name, d, step = lever_name(c["levers"]), c["direction"], c["change"]["step"]
    if d in ("increase", "decrease"):
        txt = f"{name} {'+' if d == 'increase' else '-'}{step:g}"
    else:
        txt = f"{name} {_DIRECTION_SHORT.get(d, d)}"
        if step_count and c["change"]["sign"] is not None and step is not None:
            txt += f" ({step:g} step{'s' if step != 1 else ''})" if float(step).is_integer() else f" ({step:g})"
    return txt[0].upper() + txt[1:] if capital else txt


def values_text(c):
    # from -> to where the sheet (or a logged channel) gives both, once per
    # distinct pair (left and right usually agree)
    pairs = []
    for k in c["change"]["keys"]:
        if k["from"] is not None and k["to"] is not None:
            t = f"{k['from']} -> {k['to']}"
            if t not in pairs:
                pairs.append(t)
    return ", ".join(pairs)


def _phases(phases):
    out = []
    for p in sorted(phases, key=PHASE_KEYS.index):
        if _PHASE_WORD[p] not in out:
            out.append(_PHASE_WORD[p])
    return ", ".join(out)


def why_text(votes):
    """'unstable yaw - C8 turn-in, apex; C11 apex, exit - 6/6 laps': per
    symptom the corners and phases; the lap count once when all agree."""
    by_sym = {}
    for v in votes:
        by_sym.setdefault(v["symptom"], {}).setdefault(v["corner"], []).append(v)
    parts = []
    for sym, corners in by_sym.items():
        laps = {(v["laps"], v["n_laps"]) for vs in corners.values() for v in vs if not v["driver_only"]}
        once = len(laps) == 1
        cs = []
        for cid, vs in corners.items():
            t = f"C{cid} {_phases({v['phase'] for v in vs})}"
            if vs[0]["driver_only"]:
                t += f" (driver {vs[0].get('feedback', 0):+g})"
            elif not once:
                t += " " + ", ".join(sorted({f"{v['laps']}/{v['n_laps']}" for v in vs if not v["driver_only"]}))
            cs.append(t)
        part = f"{_SYMPTOM_WORD[sym]} - " + "; ".join(cs)
        if once:
            k, n = next(iter(laps))
            part += f" - {k}/{n} laps"
        parts.append(part)
    return " | ".join(parts)


def session_summary(rec, engineer=False):
    """(headline, why, against or None, alternatives or None, details lines)."""
    o = rec["recommended"]
    against = None
    if o:
        head = f"{lever_text(o, capital=True, step_count=True)} | {values_text(o) or '-'}"
        why = "Why: " + why_text(o["supporting"])
        if o["opposing"]:
            against = "Against: " + why_text(o["opposing"])
    elif rec["equivalent_options"]:
        head = "Tie, no winner: " + " / ".join(lever_text(x) for x in rec["equivalent_options"])
        seen, votes = set(), []
        for x in rec["equivalent_options"]:
            for v in x["supporting"]:
                key = (v["corner"], v["symptom"], v["phase"])
                if key not in seen:
                    seen.add(key)
                    votes.append(v)
        why = "Why: " + why_text(votes)
    else:
        head = "No setup change"
        why = "No repeated symptom" if not rec["votes"] else "No lever with net support"
    alts = ("Alternatives: " + ", ".join(lever_text(x) for x in rec["alternatives"])) if rec["alternatives"] else None
    return head, why, against, alts, session_details(rec, engineer)


def corner_summary(corner, engineer=False):
    """('C8 - high - unstable yaw - turn-in, apex - 6/6 -> rear ARB softer
    (1 step) | driver: brake bias forward', 'alt: ...' or None, details)."""
    by_sym = {}
    for s in corner.get("symptoms", []):
        by_sym.setdefault(s["symptom"], []).append(s)
    n = corner.get("n_laps", 0)
    syms = []
    for sym, ss in by_sym.items():
        laps = sorted({s["laps"] for s in ss})
        lap_txt = f"{laps[0]}/{n}" if len(laps) == 1 else ", ".join(f"{k}/{n}" for k in laps)
        syms.append(f"{_SYMPTOM_WORD[sym]} - {_phases({s['phase'] for s in ss})} - {lap_txt}")
    line = f"C{corner['corner']} - {corner.get('speed_class') or '-'} - " + "; ".join(syms)
    if corner.get("low_priority"):
        line += " (low priority)"
    rec = corner.get("recommended")
    if rec:
        vals = values_text(rec)
        line += f" -> {lever_text(rec)}" + (f" {vals}" if vals else "")
    else:
        line += " -> no mechanical lever"
    if corner.get("cockpit"):
        line += f" | driver: {lever_text(corner['cockpit'])}"
    alts = corner.get("alternatives") or []
    alt = ("alt: " + ", ".join(lever_text(a) for a in alts)) if alts else None
    return line, alt, corner_details(corner, engineer)


def repeated_symptoms_line(session):
    n = session["n_corners"]
    parts = [f"{sym} {v['corners']}/{n}" for sym, v in session["shares"].items() if v["corners"]]
    return "Repeated symptoms: " + (", ".join(parts) + " corners" if parts else "none")


def feedback_entered(feedback_data):
    # any non-zero rating in the feedback table
    for row in (feedback_data or {}).get("corners") or []:
        if any(isinstance(v, (int, float)) and v for v in row.values()):
            return True
    return False


def _rec_key(c):
    return (tuple(c["levers"]), c["direction"]) if c else None


def engineer_lines(data, eng):
    """Only what the feedback changes against the data layer, plus the agree
    / conflict flags of the corners behind the session recommendation."""
    lines = []
    d_head = session_summary(data["session_recommendation"])[0]
    e_rec = eng["session_recommendation"]
    e_head = session_summary(e_rec)[0]
    if e_head != d_head:
        lines.append(f"Session: {e_head} (data: {d_head})")
    top = e_rec["recommended"] or (e_rec["equivalent_options"] or [None])[0]
    if top:
        flags = [f"C{v['corner']} {_PHASE_WORD[v['phase']]} {v['agreement']}" for v in top["supporting"]
                 if v.get("agreement") in ("agree", "conflict")]
        if flags:
            lines.append("Feedback: " + ", ".join(flags))
    by_corner = {c["corner"]: c for c in data["corners"]}
    for c in eng["corners"]:
        a, b = (by_corner.get(c["corner"]) or {}).get("recommended"), c.get("recommended")
        if _rec_key(a) != _rec_key(b):
            lines.append(f"C{c['corner']}: {lever_text(b) if b else 'no lever'} (data: {lever_text(a) if a else 'no lever'})")
    for d in eng.get("driver_only", []):
        r = d.get("recommended")
        lines.append(f"C{d['corner']} {_PHASE_WORD[d['phase']]} - driver {d['feedback']:+g}, no data -> "
                     f"{lever_text(r) if r else 'no lever'}")
    return lines or ["No difference to the data layer"]
