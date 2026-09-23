# DIAGNOSTIC (read-only, [keep-reproduces]): decision-frame candidate
# census on both real sessions -- FRAME DEPTH PROGRAMME's own pre-work
# question (PLAN.md, Step ordering: understand today's shape before adding
# conditions on top of it). No config/production change; nothing written
# back anywhere.
#
# Reuses diagnostics/inspect_frame_stage2_parity.py's own run_full_pipeline
# (same accuracy cap=1, same file paths -- DUBAI_FILE/V3_FILE literally
# imported, not re-typed) for both sessions.
#
# DECISION LAYER SPEC B8 (2026-09-22) RECONCILIATION: this script used to
# hand-re-implement generate_candidates' own body (calling each generator
# function individually) purely to attach a "_generator" label per
# candidate. By Phase B's end that body had grown to 7 steps (four base
# generators, brake_bias, the eligibility gate, the feedback-only
# generator, window-edge status, breadth) and the hand-kept copy had
# already drifted twice (B2, B3) -- a maintenance trap by construction.
# Switched to calling generate_candidates() directly and deriving each
# candidate's generator label from fields the production dict ALREADY
# carries (id prefix, scenario, rule_id, effect_class) -- no logic
# reimplemented anywhere now, so this file cannot drift from production
# again the way the old hand-copy did.

import re
from collections import Counter, defaultdict

import numpy as np

from diagnostics.inspect_frame_stage2_parity import run_full_pipeline, DUBAI_FILE, V3_FILE, classify_fn
from modules.decision_frame import (
    aggregate_ls_by_corner, load_decision_frame_config,
    generate_candidates, generate_shortlist, resolve_conflicts,
    build_evidence,
)
from modules.recommendation import load_setup_parameters_registry, _group_by_corner

REPEAT_RE = re.compile(r"repeats on (\d+)/(\d+) laps")


def _generator_label(c):
    """Derives which generate_candidates() step produced this candidate
    from fields the production dict already carries -- see this file's
    own B8 reconciliation comment above."""
    cid = c["id"]
    if cid.startswith("lever_bridge:"):
        return "lever_bridges"
    if cid.startswith("feedback_only:"):
        return "feedback_only"
    if cid.startswith("brake_bias:"):
        return "brake_bias"
    if c.get("scenario") == "plausibility_brake_balance":
        return "brake_balance"
    if c.get("scenario") == "exit_oversteer":
        return "exit_oversteer"
    if c.get("rule_id") is not None:
        return "matrix_bridge_held_secondary" if c.get("effect_class") == "secondary" else "matrix_bridge"
    return "unlabelled"


def _repeat_laps(evidence_item):
    """Parses the (repeat, total) lap counts back out of a corner_verdict/
    matrix_verdict evidence item's own human-readable 'source' string --
    these two functions do not return the raw counts separately, only the
    already-combined confidence float, so this is the one place to recover
    'how many laps actually repeat this verdict' for the census below.
    Diagnostic-only string parsing; never used to drive production logic."""
    m = REPEAT_RE.search(evidence_item.get("source", ""))
    if not m:
        return None, None
    return int(m.group(1)), int(m.group(2))


def _percentile(values, p):
    if not values:
        return None
    return float(np.percentile(np.array(values, dtype=float), p))


def census(raw_file, label):
    print(f"\n{'='*78}\nCENSUS: {label} ({raw_file})\n{'='*78}")

    pipe = run_full_pipeline(raw_file)
    summaries = pipe["summaries"]
    df_config = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    ls_stats = aggregate_ls_by_corner(summaries)

    evidence = build_evidence(
        summaries, ls_stats, df_config, classify_fn,
        corners=pipe["corners"], state=pipe["state"], channels=pipe["channels"],
        # feedback_data intentionally omitted (defaults None) -- "no feedback"
    )

    # setup_data=None -- "no setup_data", same as run_parity's own call
    # (window-edge status inert); assessed_corner_ids real, for breadth.
    assessed_corner_ids = set(_group_by_corner(summaries).keys())
    candidates = generate_candidates(evidence, registry, df_config, assessed_corner_ids=assessed_corner_ids)
    for c in candidates:
        c["_generator"] = _generator_label(c)
    shortlist = generate_shortlist(candidates, evidence, None, df_config)
    resolve_conflicts(shortlist)

    # --- 1. evidence count by type; corner_verdict/matrix_verdict by lap-repeat count ---
    ev_type_counts = Counter(e["type"] for e in evidence)
    print("\n[1] Evidence count by type:")
    for t, n in sorted(ev_type_counts.items(), key=lambda kv: -kv[1]):
        print(f"    {t:<28} {n}")

    for ev_type in ("corner_verdict", "matrix_verdict"):
        repeat_counts = Counter()
        unparsed = 0
        for e in evidence:
            if e["type"] != ev_type:
                continue
            repeat, total = _repeat_laps(e)
            if repeat is None:
                unparsed += 1
                continue
            repeat_counts[repeat] += 1
        print(f"\n    {ev_type} items by number of laps repeating the verdict:")
        if not repeat_counts:
            print("      (none)")
        for n_laps in sorted(repeat_counts):
            tag = " <-- single-lap" if n_laps == 1 else ""
            print(f"      {n_laps} lap(s): {repeat_counts[n_laps]} item(s){tag}")
        if unparsed:
            print(f"      (source string unparsable for {unparsed} item(s) -- reported separately, not silently dropped)")

    # --- 2. candidate count total; by grade; by generator ---
    print(f"\n[2] Candidates: {len(candidates)} total")
    grade_counts = Counter(c["grade"] for c in candidates)
    print("    by grade:")
    for g, n in sorted(grade_counts.items(), key=lambda kv: -kv[1]):
        print(f"      {g:<12} {n}")
    gen_counts = Counter(c["_generator"] for c in candidates)
    print("    by generator:")
    for g, n in sorted(gen_counts.items(), key=lambda kv: -kv[1]):
        print(f"      {g:<28} {n}")

    # --- 3. candidates grouped by (parameter, direction) ---
    groups = defaultdict(list)
    for c in candidates:
        conf = min((e["confidence"] for e in c["evidence_refs"] if e.get("confidence") is not None), default=None)
        for a in c["actions"]:
            key = (a["parameter"], a.get("direction") or a.get("target"))
            groups[key].append((c, conf))

    print(f"\n[3] Candidates grouped by (parameter, direction/target) -- {len(groups)} group(s), sorted by corner count descending:")
    rows = []
    for (param, direction), items in groups.items():
        corners = sorted({c["corner"] for c, _ in items})
        confs = [conf for _, conf in items if conf is not None]
        # Per-corner lap support: the STRONGEST repeat count found among
        # that corner's own candidates' evidence_refs in this group (a
        # corner can appear via more than one candidate -- e.g. a primary
        # plus a held-secondary sharing the same evidence -- so max, not
        # sum, avoids double-counting the same underlying laps twice).
        # Summed ACROSS corners (a corner's laps cannot support another
        # corner's evidence, so cross-corner summation is the honest
        # combination).
        per_corner_best = {}
        for c, _ in items:
            best = 0
            for e in c["evidence_refs"]:
                r, t = _repeat_laps(e)
                if r is not None:
                    best = max(best, r)
            if best:
                per_corner_best[c["corner"]] = max(per_corner_best.get(c["corner"], 0), best)
        total_lap_support = sum(per_corner_best.values())
        rows.append((param, direction, len(corners), corners,
                      max(confs) if confs else None, min(confs) if confs else None,
                      total_lap_support))
    rows.sort(key=lambda r: -r[2])
    for param, direction, n_corners, corners, max_conf, min_conf, lap_support in rows:
        print(f"    {param:<22} {str(direction):<14} corners={n_corners} {corners} "
              f"conf=[{min_conf if min_conf is not None else 'n/a'}, "
              f"{max_conf if max_conf is not None else 'n/a'}] "
              f"lap_support_sum={lap_support}")

    # --- 4. confidence distribution ---
    all_confs = [min((e["confidence"] for e in c["evidence_refs"] if e.get("confidence") is not None), default=None)
                 for c in candidates]
    all_confs = [c for c in all_confs if c is not None]
    below_03 = sum(1 for c in all_confs if c < 0.3)
    print(f"\n[4] Candidate confidence distribution (n={len(all_confs)}):")
    print(f"    p10={_percentile(all_confs, 10)}, p50={_percentile(all_confs, 50)}, p90={_percentile(all_confs, 90)}")
    print(f"    below 0.3: {below_03}/{len(all_confs)}")

    # --- 5. hypothetical min_repeat_laps=2 floor, report only ---
    survive = 0
    fail = 0
    fail_reason_unparsable = 0
    for c in candidates:
        repeats = []
        ok = True
        for e in c["evidence_refs"]:
            if e["type"] not in ("corner_verdict", "matrix_verdict"):
                continue
            r, t = _repeat_laps(e)
            if r is None:
                continue
            repeats.append(r)
        if not repeats:
            # No corner_verdict/matrix_verdict evidence_ref to gate on
            # (e.g. a brake_balance-only or ls_threshold-only candidate) --
            # a hypothetical floor on THESE two evidence types simply does
            # not apply to it; reported as surviving by default, not
            # silently miscounted as failing.
            survive += 1
            continue
        if min(repeats) < 2:
            ok = False
        if ok:
            survive += 1
        else:
            fail += 1
    print(f"\n[5] Hypothetical min_repeat_laps=2 floor on corner_verdict/matrix_verdict "
          f"evidence_refs (report only, NOT implemented):")
    print(f"    would survive: {survive}/{len(candidates)}   would fail: {fail}/{len(candidates)}")

    return {
        "evidence": evidence, "candidates": candidates, "shortlist": shortlist,
        "ev_type_counts": ev_type_counts, "grade_counts": grade_counts, "gen_counts": gen_counts,
        "rows": rows, "all_confs": all_confs, "below_03": below_03,
        "min_repeat2_survive": survive, "min_repeat2_fail": fail,
    }


def main():
    results = {}
    for raw_file, label in [(DUBAI_FILE, "Dubai"), (V3_FILE, "v3")]:
        results[label] = census(raw_file, label)
    return results


if __name__ == "__main__":
    main()
