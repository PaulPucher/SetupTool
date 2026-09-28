# DIAGNOSTIC, read-only, [keep-reproduces]. Final CS read-out round: fair
# thresholds for the pooled-25th-percentile statistic (Part 1), the
# hardened-production display variant (Part 2), the three-way decision
# table (Part 3). Nothing in modules/ or config/ changes; candidate
# thresholds and display rules act in memory only.
#
# Reuses, does not rebuild: the session caches and loaders
# (inspect_new_laps_cs_sweep), the phase slicing and read-outs verified
# last package (inspect_cs_readout_evidence: 5,940/5,940 phase medians,
# 137/137 verdicts), and the recorded perturbation machinery
# (inspect_metrology_phase1_sensitivity: SESSIONS, build_variant, run_one's
# call sequence).
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_cs_decision_table sens      (22 runs)
#   python -m diagnostics.inspect_cs_decision_table thresholds
#
# PART 1 PROCEDURE, the recorded one made explicit (config derived_from of
# STRONG_CSF/CSR, MODERATE_CSF/CSR, verdict_stability_margin, 2026-09-02 /
# 2026-09-19), fixed before the run and checked against the recorded case:
# - Population: per stable corner, the worst phase of the statistic (min
#   over the five phases, apex from apex_region) -- what the classifier
#   reads. Per axle. Pooled over the eight distinct sessions (US-v1 is the
#   v3 outing and is left out); per-session values reported as well.
# - STRONG: physical anchor 0 (the force peak). Noise population = the
#   negative values minus (a) ground-truth-confirmed genuine events -- none
#   remain, C4 is unconfirmed -- and (b) the standout group, operationalised
#   as the values beyond the largest gap between consecutive negative
#   values. Noise floor = the most negative remaining value; STRONG = the
#   noise floor moved outward to the next 0.01 ("just past" it).
#   Check on the recorded Dubai worst-lap population: rear gives C4 as the
#   standout and -0.065 -> -0.07 exactly as recorded; front gives the
#   recorded standout group {C9, C3} -- the recorded -0.10 additionally
#   excluded C4 as ground truth, which no longer holds.
# - MODERATE: gap selection on the positive values below the 1.0 clip:
#   midpoint of the first gap >= 0.10 wide scanning upward from 0. Check on
#   the recorded populations: front 0.074 | 0.229 -> 0.15 (recorded 0.15),
#   rear 0.159 | 0.293 -> 0.23 (recorded 0.20, "inside the gap").
# - Margin: the recorded +/-1 % sensitivity runs (5 inputs x 2 signs x
#   Dubai, v3 + 2 baselines), verdicts recomputed per corner-phase under
#   the pooled statistic with the Part-1 thresholds; margin = the
#   2nd-largest baseline threshold-distance among the corner-phases that
#   flip, rounded up to 0.01 (the recorded 0.11 = 2nd-most-marginal flip
#   0.104 rounded up).
# FORBIDDEN and not done: any use of driver reports, A-vs-B resolution or
# other Part-3 criteria in the derivation.

import copy
import os
import pickle
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import load_all
from diagnostics.inspect_cs_readout_evidence import (
    phase_index_map, corner_readouts, aggregated_for, load_supp2, PHASES, ALL_KEYS,
)

OUT_DIR = "diagnostics/plots/cs_decision_table"
SENS_DIR = os.path.join(OUT_DIR, "sens")


def _sens_task(task):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    session_name, input_name, sign = task
    import modules.accuracy_resolution as accres
    from diagnostics.inspect_metrology_phase1_sensitivity import SESSIONS as MSESS, build_variant
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        load_parameters, load_car_data, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
    )
    from modules.tyre_fit_auto import resolve_sideslip_beta
    sess = MSESS[session_name]
    if input_name == "baseline":
        car_setup, raw_params, car_data_patch = sess["car_setup"], None, None
    else:
        car_setup, raw_params, car_data_patch, _d = build_variant(sess["car_setup"], input_name, sign)
    # run_one's own call sequence, keeping the arrays it discards
    params = raw_params if raw_params is not None else copy.deepcopy(load_parameters())
    setup_data = {"car": car_setup}
    if car_data_patch is not None:
        orig = accres.load_car_data
        accres.load_car_data = lambda: car_data_patch
        try:
            resolved = accres.resolve_accuracy(params, setup_data=setup_data, cap=None)
        finally:
            accres.load_car_data = orig
    else:
        resolved = accres.resolve_accuracy(params, setup_data=setup_data, cap=None)
    ep = accres.apply_resolved_vehicle(params, resolved)
    data = parse_csv(sess["file"])
    state = prepare_vehicle_state(data["channels"], ep)
    beta, _fm, _g, _fb, _fr = resolve_sideslip_beta(state, ep, data, ep["stability_estimation"].get("sideslip_source"),
                                                    csv_path=sess["file"])
    slip = estimate_slip_angles(state, beta, ep)
    forces = estimate_lateral_forces(state, ep)
    cs = estimate_cornering_stiffness(slip, forces, state, ep)
    stab = estimate_yaw_moment_stability(state, beta, ep, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, ep, channels=data["channels"], car_data=load_car_data())
    summaries = summarise_corners(data.get("corners", []), cs, stab, state, fz=fz, lap_filter=None)
    moving = state["moving_mask"].copy()
    if state.get("kerb_mask") is not None:
        moving &= ~state["kerb_mask"]
    r = {"time": state["time"], "s_m": state["s_m"], "moving": moving, "laps": data["laps"],
         "cs": {"A_baseline": (cs["CS_ratio_f"], cs["CS_ratio_r"])}, "summaries": {"A_baseline": summaries}}
    se = load_parameters()["stability_estimation"]
    pim = phase_index_map(r, {"corners": data.get("corners", [])}, se)
    laps = sorted({l for (_c, l) in pim})
    ro = corner_readouts(r, pim, laps, se["cs_phase_min_valid_samples"])
    label = f"{session_name}_{input_name}_{'base' if sign == 0 else ('plus1' if sign > 0 else 'minus1')}"
    os.makedirs(SENS_DIR, exist_ok=True)
    with open(os.path.join(SENS_DIR, label + ".pkl"), "wb") as fh:
        pickle.dump({"ro": ro, "laps": laps, "summaries": summaries}, fh)
    return label


def sens():
    from concurrent.futures import ProcessPoolExecutor, as_completed
    from diagnostics.inspect_metrology_phase1_sensitivity import SESSIONS as MSESS, INPUTS
    tasks = []
    for s in MSESS:
        tasks.append((s, "baseline", 0))
        for inp in INPUTS:
            tasks += [(s, inp, +1), (s, inp, -1)]
    print(f"{len(tasks)} runs", flush=True)
    with ProcessPoolExecutor(max_workers=6) as pool:
        futs = [pool.submit(_sens_task, t) for t in tasks]
        for f in as_completed(futs):
            print("done", f.result(), flush=True)


RO_PATH = "diagnostics/plots/cs_readout_evidence/readout_full.pkl"
FLOORS = (0.3, 0.5, 0.75)


# ---- Part 2: display-layer hardening (estimator untouched) ----
#
# Evidence floor: a row may sit in the visible top 3 only if its problem
#   weight >= floor OR its evidence spans >= 2 corners; otherwise it goes
#   to the tail as "below evidence floor". Problem weight of a grouped row
#   = the highest member's (the row's own score term).
# Lever conflict: two display rows moving one lever in opposite directions
#   (e.g. tc_lon up at one corner, down at another) become ONE conflict
#   card -- the existing conflict_status / conflict_with vocabulary of
#   resolve_conflicts, extended from one corner to the whole session --
#   placed at the better member's rank; neither direction is ranked alone.

def display_rows(key, r, override=None):
    """Grouped proposed rows exactly as the UI builds them (data only: no
    sheet, no feedback, no driver). override = aggregate replacement."""
    import modules.decision_frame as df
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    from modules.recommendation import load_setup_parameters_registry, _group_by_corner
    cfg = df.load_decision_frame_config()
    registry = load_setup_parameters_registry()
    sm = r["summaries"]["A_baseline"]
    original = df.aggregate_by_corner
    if override is not None:
        df.aggregate_by_corner = lambda _s: override
    try:
        ev = df.build_evidence(sm, df.aggregate_ls_by_corner(sm), cfg, classify_fn)
        cands = df.generate_candidates(ev, registry, cfg, setup_data=None,
                                       assessed_corner_ids=set(_group_by_corner(sm).keys()), driving_level=None)
        sl = df.generate_display_split(cands, ev, None, cfg, registry)["shortlist"]
        df.resolve_conflicts(sl)
        rows = df.group_display_rows(sl, registry)
    finally:
        df.aggregate_by_corner = original
    return rows, len(cands)


def row_facts(c):
    members = c.get("group_members") or [c]
    corners = sorted({m.get("corner") for m in members if m.get("corner") is not None})
    pw = max(m.get("score_components", {}).get("problem_weight", 0.0) for m in members)
    marginal = any(e.get("marginal") for m in members for e in m.get("evidence_refs", []))
    conf = [e.get("confidence") for m in members for e in m.get("evidence_refs", []) if e.get("confidence") is not None]
    levers = {(a.get("parameter"), a.get("direction")) for a in c.get("actions", [])}
    return {"corners": corners, "pw": pw, "marginal": marginal, "min_conf": min(conf) if conf else None,
            "levers": levers, "label": "; ".join(f"{p} {d}" for p, d in sorted(levers, key=str)),
            "is_tc": any((p or "").startswith("tc_") for p, _ in levers), "score": c.get("score")}


def merge_lever_conflicts(rows):
    """Opposite directions on one lever across the session -> one card."""
    facts = [row_facts(c) for c in rows]
    used, out = set(), []
    for i, c in enumerate(rows):
        if i in used:
            continue
        partner = None
        for j in range(i + 1, len(rows)):
            if j in used:
                continue
            opp = {(p, d) for p, d in facts[i]["levers"]}
            if any((p, {"increase": "decrease", "decrease": "increase"}.get(d)) in facts[j]["levers"] for p, d in opp):
                partner = j
                break
        if partner is None:
            out.append((c, facts[i], None))
            continue
        used.add(partner)
        card = dict(c)
        card["conflict_status"] = "lever_conflict_across_corners"
        card["conflict_with"] = [rows[partner].get("id")]
        f = dict(facts[i])
        f["corners"] = sorted(set(facts[i]["corners"]) | set(facts[partner]["corners"]))
        f["pw"] = max(facts[i]["pw"], facts[partner]["pw"])
        f["marginal"] = facts[i]["marginal"] or facts[partner]["marginal"]
        f["label"] = f"CONFLICT {facts[i]['label']} / {facts[partner]['label']}"
        out.append((card, f, facts[partner]))
    return out


def opposite_pairs(facts_list, top=None):
    fl = facts_list[:top] if top else facts_list
    n = 0
    for i in range(len(fl)):
        for j in range(i + 1, len(fl)):
            if any((p, {"increase": "decrease", "decrease": "increase"}.get(d)) in fl[j]["levers"] for p, d in fl[i]["levers"]):
                n += 1
    return n


def harden(rows, floor):
    """(visible top 3, demoted rows) after conflict merge + evidence floor."""
    merged = merge_lever_conflicts(rows)
    eligible = [(c, f) for c, f, _p in merged if f["pw"] >= floor or len(f["corners"]) >= 2]
    demoted = [(c, f) for c, f, _p in merged if not (f["pw"] >= floor or len(f["corners"]) >= 2)]
    return eligible[:3], demoted, merged


def part2():
    res = load_all()
    print("PART 2 -- hardened production (display layer only), data-only census per session")
    for key in ALL_KEYS:
        r = res[key]
        rows, n_c = display_rows(key, r)
        facts = [row_facts(c) for c in rows]
        pairs_before_top3 = opposite_pairs(facts, 3)
        pairs_before_all = opposite_pairs(facts)
        merged = merge_lever_conflicts(rows)
        cards = sum(1 for _c, f, p in merged if p is not None)
        print(f"\n## {key}: {n_c} candidates, {len(rows)} rows; #1 now: {facts[0]['label'] if facts else '-'} "
              f"(pw {facts[0]['pw']:.3f}, corners {facts[0]['corners']}, marginal {facts[0]['marginal']})" if facts else f"\n## {key}: no rows")
        print(f"  opposite-direction pairs: top-3 {pairs_before_top3}, all rows {pairs_before_all} -> after merge: "
              f"{opposite_pairs([f for _c, f, _p in merged], 3)} in top 3 ({cards} conflict card(s))")
        for floor in FLOORS:
            vis, dem, _m = harden(rows, floor)
            first = vis[0][1] if vis else None
            demoted_first = facts and first is not None and first["label"] != facts[0]["label"] and not merged[0][2]
            dem_marg = sum(1 for _c, f in dem if f["marginal"])
            print(f"  floor {floor}: new #1 {first['label'] if first else '-'} (pw {first['pw'] if first else float('nan'):.3f}); "
                  f"original #1 {'DEMOTED' if facts and facts[0]['label'] not in [f['label'] for _c, f in vis] and not merged[0][2] else 'kept or merged'}; "
                  f"rows below floor {len(dem)} (of which MARGINAL-tagged {dem_marg}); visible {len(vis)}")
        thin = [f for f in facts[:3] if f["pw"] < 0.5 and len(f["corners"]) < 2]
        print(f"  current top-3 thin rows (pw < 0.5, one corner): {len(thin)}; their MARGINAL tags {sum(f['marginal'] for f in thin)}, "
              f"min evidence confidence {[f['min_conf'] for f in thin]}")


def _worst(ro_c, ax, name):
    v = [ro_c[ph][ax][name] for ph in PHASES]
    v = [x for x in v if x == x]
    return min(v) if v else float("nan")


def strong_rule(vals):
    """(STRONG, trail) by the recorded procedure (header)."""
    neg = sorted(v for v in vals if v == v and v < 0)
    if not neg:
        return float("nan"), "no negative values -> no anchor"
    if len(neg) == 1:
        standout, rest = [], neg
    else:
        gaps = np.diff(neg)
        k = int(np.argmax(gaps))
        standout, rest = neg[:k + 1], neg[k + 1:]
    floor_ = rest[0]
    strong = -np.ceil(abs(floor_) * 100 + 1e-9) / 100
    trail = (f"negatives n={len(neg)}; standout beyond largest gap: {[round(x, 3) for x in standout]} "
             f"(gap {np.max(np.diff(neg)) if len(neg) > 1 else float('nan'):.3f}); noise floor {floor_:.3f} -> STRONG {strong:.2f}")
    return float(strong), trail


def moderate_rule(vals, min_gap=0.10):
    pos = sorted(v for v in vals if v == v and 0 <= v < 1.0)
    for a, b in zip(pos, pos[1:]):
        if b - a >= min_gap:
            m = round((a + b) / 2, 2)
            return m, f"positives below clip n={len(pos)}; first gap >= {min_gap}: {a:.3f} | {b:.3f} (width {b - a:.3f}) -> midpoint {m:.2f}"
    return float("nan"), f"positives below clip n={len(pos)}; no gap >= {min_gap} -> MODERATE undefined"


def derive(full, keys, name):
    out = {}
    for ax in ("f", "r"):
        vals = []
        for k in keys:
            _laps, ro = full[k]
            vals += [_worst(ro[c], ax, name) for c in ro]
        s, st = strong_rule(vals)
        m, mt = moderate_rule(vals)
        out[ax] = {"STRONG": s, "MODERATE": m, "strong_trail": st, "moderate_trail": mt,
                   "n": sum(1 for v in vals if v == v), "vals": vals}
    return out


def thresholds():
    with open(RO_PATH, "rb") as fh:
        full = pickle.load(fh)
    print("CHECK: rules on the recorded case (Dubai, PROD = worst lap)")
    d = derive(full, ["dubai"], "PROD")
    for ax in ("f", "r"):
        print(f"  {ax}: {d[ax]['strong_trail']}")
        print(f"     {d[ax]['moderate_trail']}")
    keys = [k for k in ALL_KEYS]
    print("\nPOOLED25, per session:")
    per = {}
    for k in keys:
        per[k] = derive(full, [k], "POOLED25")
        print(f"  {k}: STRONG f {per[k]['f']['STRONG']:+.2f} r {per[k]['r']['STRONG']:+.2f}; "
              f"MODERATE f {per[k]['f']['MODERATE']:+.2f} r {per[k]['r']['MODERATE']:+.2f}")
        for ax in ("f", "r"):
            print(f"     {ax}: {per[k][ax]['strong_trail']}; {per[k][ax]['moderate_trail']}")
    pooled = derive(full, keys, "POOLED25")
    print("\nPOOLED25, all 8 sessions pooled:")
    for ax in ("f", "r"):
        v = np.array([x for x in pooled[ax]["vals"] if x == x])
        print(f"  {ax}: n={v.size}, p5/p25/median/p75 {np.percentile(v, 5):+.3f}/{np.percentile(v, 25):+.3f}/"
              f"{np.median(v):+.3f}/{np.percentile(v, 75):+.3f}")
        print(f"     {pooled[ax]['strong_trail']}")
        print(f"     {pooled[ax]['moderate_trail']}")
    with open(os.path.join(OUT_DIR, "thresholds.pkl"), "wb") as fh:
        pickle.dump({"pooled": {ax: {k: pooled[ax][k] for k in ("STRONG", "MODERATE", "strong_trail", "moderate_trail")}
                                for ax in ("f", "r")},
                     "per": {k: {ax: {kk: per[k][ax][kk] for kk in ("STRONG", "MODERATE")} for ax in ("f", "r")} for k in per}},
                    fh)


def margin():
    """cs_margin analogue from the recorded sensitivity runs, pooled-p25
    statistic under the Part-1 pooled thresholds (header)."""
    from diagnostics.inspect_cs_readout_evidence import _calls
    th = _load_thresholds()
    runs = {f[:-4]: pickle.load(open(os.path.join(SENS_DIR, f), "rb")) for f in os.listdir(SENS_DIR) if f.endswith(".pkl")}
    print(f"{len(runs)} sensitivity runs loaded")
    flips = []
    with thresholds_patch(th):
        from modules.stability_analysis import load_parameters
        cls = load_parameters()["classification"]
        for sess in ("dubai", "v3"):
            base = runs[f"{sess}_baseline_base"]
            bv = _phase_calls(base, "POOLED25")
            for label, run in runs.items():
                if not label.startswith(sess) or label.endswith("_base"):
                    continue
                pv = _phase_calls(run, "POOLED25")
                for key, (short, val_f, val_r) in bv.items():
                    if key not in pv or pv[key][0] == short:
                        continue
                    # distance of the verdict-driving axle to its nearest threshold, baseline
                    drive = "r" if "oversteer" in short or "oversteer" in pv[key][0] else "f"
                    v = val_r if drive == "r" else val_f
                    ths = [cls["STRONG_CSR"]["value"], cls["MODERATE_CSR"]["value"]] if drive == "r" else \
                          [cls["STRONG_CSF"]["value"], cls["MODERATE_CSF"]["value"]]
                    d = min(abs(v - t) for t in ths) if v == v else float("nan")
                    flips.append((d, sess, label, key, short, pv[key][0]))
    flips.sort(key=lambda x: -x[0] if x[0] == x[0] else 0)
    print(f"flipped corner-phases: {len(flips)}")
    for f in flips[:12]:
        print(f"  distance {f[0]:.3f}  {f[2]} C{f[3][0]} {f[3][1]}: '{f[4]}' -> '{f[5]}'")
    d = [f[0] for f in flips if f[0] == f[0]]
    m = float(np.ceil(sorted(d, reverse=True)[1] * 100) / 100) if len(d) >= 2 else float("nan")
    print(f"cs_margin analogue (2nd-largest flip distance, rounded up to 0.01): {m:.2f}")
    t = _load_thresholds_raw()
    t["cs_margin"] = m
    t["margin_trail"] = (f"{len(flips)} flipped corner-phases over 20 perturbation runs; 2nd-largest distance "
                         f"{sorted(d, reverse=True)[1] if len(d) >= 2 else float('nan'):.3f} -> {m:.2f}")
    with open(os.path.join(OUT_DIR, "thresholds.pkl"), "wb") as fh:
        pickle.dump(t, fh)


def _phase_calls(run, name):
    """{(cid, phase): (short, csf, csr)} per-phase verdicts, as the recorded
    sensitivity classified them (classify_all), on one read-out."""
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    r = {"summaries": {"A_baseline": run["summaries"]}}
    agg = aggregated_for(r, run["laps"], run["ro"], name)
    out = {}
    for cid, corner in agg.items():
        for ph in PHASES:
            sliced = {"phases": {ph: corner["phases"][ph]}}
            if ph == "apex_3":
                sliced["apex_region"] = corner["apex_region"]
            _sev, short, _l, _c = classify_fn(sliced)
            src = corner["apex_region"] if ph == "apex_3" else corner["phases"][ph]
            out[(cid, ph)] = (short.replace(" [MARGINAL]", ""), src["cs_ratio_f"]["median"], src["cs_ratio_r"]["median"])
    return out


def _load_thresholds_raw():
    with open(os.path.join(OUT_DIR, "thresholds.pkl"), "rb") as fh:
        return pickle.load(fh)


def _load_thresholds():
    t = _load_thresholds_raw()["pooled"]
    sf, sr = t["f"]["STRONG"], t["r"]["STRONG"]
    mf, mr = t["f"]["MODERATE"], t["r"]["MODERATE"]
    # an undefined MODERATE (no gap found) = an empty moderate band
    th = {"STRONG_CSF": sf, "STRONG_CSR": sr,
          "MODERATE_CSF": mf if mf == mf else sf, "MODERATE_CSR": mr if mr == mr else sr}
    raw = _load_thresholds_raw()
    if "cs_margin" in raw:
        th["cs_margin"] = raw["cs_margin"]
    return th


class thresholds_patch:
    """Candidate thresholds into the cached parameter dict the classifier
    reads, in this process only; restored on exit."""
    def __init__(self, th):
        self.th = th

    def __enter__(self):
        from modules.stability_analysis import load_parameters
        cls = load_parameters()["classification"]
        self.saved = copy.deepcopy(cls)
        for k in ("STRONG_CSF", "STRONG_CSR", "MODERATE_CSF", "MODERATE_CSR"):
            cls[k]["value"] = self.th[k]
        if "cs_margin" in self.th:
            cls["verdict_stability_margin"]["cs_margin"] = self.th["cs_margin"]
        return self

    def __exit__(self, *exc):
        from modules.stability_analysis import load_parameters
        load_parameters()["classification"].clear()
        load_parameters()["classification"].update(self.saved)


def table():
    """Part 3: rows A0 (production), A (production + hardening), B (pooled
    p25 under Part-1 thresholds + hardening), all criteria."""
    from collections import Counter
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_cs_readout_evidence import _calls
    from diagnostics.inspect_cs_value_lens import _brackets
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    res = load_all()
    with open(RO_PATH, "rb") as fh:
        full = pickle.load(fh)
    pims = {k: phase_index_map(res[k], load_supp2(k), se) for k in ALL_KEYS}
    thB = _load_thresholds()
    print("row B thresholds (Part 1, pooled): " + ", ".join(f"{k} {v:+.2f}" for k, v in thB.items()))
    rows_def = {"A0": ("PROD", None), "A": ("PROD", None), "B": ("POOLED25", thB)}

    def agg_for(key, laps, ro, stat):
        return aggregated_for(res[key], laps, ro, stat)

    def with_th(th):
        return thresholds_patch(th) if th is not None else _nullctx()

    calls_A0 = {k: _calls(agg_for(k, *full[k], "PROD")) for k in ALL_KEYS}
    for rname, (stat, th) in rows_def.items():
        print(f"\n######## ROW {rname}: read-out {stat}, thresholds {'Part-1 pooled' if th else 'current'}, "
              f"display hardening {'yes' if rname != 'A0' else 'no'}")
        with with_th(th):
            cls = load_parameters()["classification"]
            S_F, S_R = cls["STRONG_CSF"]["value"], cls["STRONG_CSR"]["value"]
            calls = {k: _calls(agg_for(k, *full[k], stat)) for k in ALL_KEYS}
            # 1. battery
            r = res["dubai"]
            ls_neg = sum(1 for s in r["summaries"]["A_baseline"] if s["stable_corner_id"] == 3 and
                         (s["phases"]["exit_5"].get("ls_ratio_r", {}).get("median", float("nan")) or 0) < 0)
            print(f"[1] battery: C3 LS_r exit_5 negative {ls_neg}/4 -> {'PASS' if ls_neg == 4 else 'FAIL'}")
            for key, cid in (("dubai", 3), ("pr_us_v2", 13), ("nbr_jst_solution", 9)):
                ro = full[key][1]
                gap = _worst(ro[cid], "r", stat) - _worst(ro[cid], "f", stat)
                print(f"    utilization gap {key} C{cid}: worst-phase r - f = {gap:+.3f} -> {'PASS' if gap > 0 else 'FAIL'}; "
                      f"call '{calls[key][cid][2]}'")
            for key, lab in (("dubai", "Dubai mild understeer"), ("nbr_jst_problem", "NBR A understeer label")):
                c = Counter(v[0] for v in calls[key].values())
                ok = c.get("US", 0) >= c.get("OS", 0)
                print(f"    {lab}: US {c.get('US', 0)} vs OS {c.get('OS', 0)} -> {'PASS' if ok else 'FAIL'}")
            A, B = res["nbr_jst_problem"], res["nbr_jst_solution"]
            ba, bb = _brackets(A), _brackets(B)
            pairs = [(ca, cb) for ca, (a0, a1) in sorted(ba.items()) for cb, (b0, b1) in bb.items()
                     if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0)]
            roA, roB = full["nbr_jst_problem"][1], full["nbr_jst_solution"][1]
            dfr = [roB[cb]["apex_3"]["f"][stat] - roA[ca]["apex_3"]["f"][stat] for ca, cb in pairs]
            dgp = [(roB[cb]["apex_3"]["r"][stat] - roB[cb]["apex_3"]["f"][stat]) - (roA[ca]["apex_3"]["r"][stat] - roA[ca]["apex_3"]["f"][stat])
                   for ca, cb in pairs]
            dfr = [x for x in dfr if x == x]; dgp = [x for x in dgp if x == x]
            print(f"    NBR A->B direction: median front change {np.median(dfr):+.3f} ({'up' if np.median(dfr) > 0 else 'down'}), "
                  f"median gap change {np.median(dgp):+.3f} ({'narrower' if np.median(dgp) < 0 else 'wider'}) -> "
                  f"{'PASS' if np.median(dfr) > 0 and np.median(dgp) < 0 else 'FAIL'}")
            # 2. resolvability (threshold-independent)
            def jk(key, cid, what):
                laps = full[key][0]
                def stat_of(lset):
                    ro = corner_readouts(res[key], {k: v for k, v in pims[key].items() if k[0] == cid}, lset, gate)
                    if cid not in ro:
                        return float("nan")
                    f, rr = ro[cid]["apex_3"]["f"][stat], ro[cid]["apex_3"]["r"][stat]
                    return f if what == "f" else rr - f
                th_ = stat_of(laps)
                loo = np.array([stat_of([l for l in laps if l != x]) for x in laps])
                loo = loo[np.isfinite(loo)]
                n = loo.size
                return th_, (float(np.sqrt((n - 1) / n * np.sum((loo - loo.mean()) ** 2))) if n >= 2 else float("nan"))
            cnt = {"f": [0, 0], "gap": [0, 0]}
            for ca, cb in pairs:
                for what in ("f", "gap"):
                    ta, sa = jk("nbr_jst_problem", ca, what)
                    tb, sb = jk("nbr_jst_solution", cb, what)
                    if all(np.isfinite([ta, tb, sa, sb])):
                        cnt[what][1] += 1
                        cnt[what][0] += int(abs(tb - ta) > 2 * np.sqrt(sa ** 2 + sb ** 2))
            print(f"[2] A-vs-B resolved (2-sigma jackknife; A has 2 laps): front {cnt['f'][0]}/{cnt['f'][1]}, gap {cnt['gap'][0]}/{cnt['gap'][1]}")
            # 3 is in the battery lines above (driver directions)
            # 4. lap-count bias
            laps, ro = full["pr_us_v2"]
            def share(ro_):
                vf = [_worst(ro_[c], "f", stat) for c in ro_]; vr = [_worst(ro_[c], "r", stat) for c in ro_]
                vf = [v for v in vf if v == v]; vr = [v for v in vr if v == v]
                return 100 * np.mean(np.array(vf) < S_F), 100 * np.mean(np.array(vr) < S_R)
            fa, ra = share(ro)
            win = [share(corner_readouts(res["pr_us_v2"], pims["pr_us_v2"], laps[k:k + 3], gate)) for k in range(len(laps) - 2)]
            print(f"[4] lap-count PR US-v2 beyond STRONG: all laps f {fa:.0f} % r {ra:.0f} %; 3-lap mean f "
                  f"{np.mean([w[0] for w in win]):.0f} % r {np.mean([w[1] for w in win]):.0f} %")
            # 5. verdict shift vs A0
            print("[5] verdict distribution (US/OS/ok) and changed calls vs A0: " + "; ".join(
                f"{k} {Counter(v[0] for v in calls[k].values()).get('US', 0)}/{Counter(v[0] for v in calls[k].values()).get('OS', 0)}/"
                f"{Counter(v[0] for v in calls[k].values()).get('-', 0)} ({sum(1 for c in calls[k] if calls[k][c][0] != calls_A0[k][c][0])} changed)"
                for k in ALL_KEYS))
            # 6. census health
            print("[6] census health per session:")
            tc_top = {fl: 0 for fl in FLOORS}; tc_top0 = 0
            for key in ALL_KEYS:
                override = None if stat == "PROD" else agg_for(key, *full[key], stat)
                drows, n_c = display_rows(key, res[key], override)
                facts = [row_facts(c) for c in drows]
                if rname == "A0":
                    vis = facts[:3]
                    tc_top0 += int(bool(vis) and vis[0]["is_tc"])
                    print(f"    {key}: {n_c} candidates, {len(drows)} rows; #1 {vis[0]['label'] if vis else '-'}; "
                          f"both-direction pairs in top 3 {opposite_pairs(vis)}; thin #1 (pw < 0.5, 1 corner) "
                          f"{int(bool(vis) and vis[0]['pw'] < 0.5 and len(vis[0]['corners']) < 2)}")
                    continue
                parts = []
                for fl in FLOORS:
                    vis, dem, _m = harden(drows, fl)
                    vf = [f for _c, f in vis]
                    first = vf[0] if vf else None
                    is_card = bool(first) and first["label"].startswith("CONFLICT")
                    tc_top[fl] += int(bool(first) and first["is_tc"] and not is_card)
                    parts.append(f"floor {fl}: #1 {first['label'] if first else '-'}{' (card)' if is_card else ''}, "
                                 f"pairs left {opposite_pairs(vf)}, thin #1 {int(bool(first) and first['pw'] < fl and len(first['corners']) < 2)}, "
                                 f"demoted {len(dem)}")
                print(f"    {key}: {n_c} candidates, {len(drows)} rows | " + " | ".join(parts))
            if rname == "A0":
                print(f"    TC recommendation as #1: {tc_top0}/{len(ALL_KEYS)} sessions")
            else:
                print("    TC recommendation as #1 (conflict cards not counted): " + ", ".join(f"floor {fl} {tc_top[fl]}/{len(ALL_KEYS)}" for fl in FLOORS))
            # 7. call stability
            parts = []
            for key in ALL_KEYS:
                laps, _ = full[key]
                if len(laps) < 4:
                    continue
                h1, h2 = laps[0::2], laps[1::2]
                c1 = _calls(agg_for(key, h1, corner_readouts(res[key], pims[key], h1, gate), stat))
                c2 = _calls(agg_for(key, h2, corner_readouts(res[key], pims[key], h2, gate), stat))
                common = [c for c in c1 if c in c2]
                parts.append(f"{key} {sum(1 for c in common if c1[c][0] == c2[c][0])}/{len(common)}")
            print("[7] split-half same call (odd vs even laps; a session-level stability, not a lap-pair rate): " + ", ".join(parts))
            # 8. C4
            ro = full["dubai"][1]
            wf = {c: _worst(ro[c], "f", stat) for c in ro}
            order = sorted([c for c in wf if wf[c] == wf[c]], key=lambda c: wf[c])
            print(f"[8] C4 (not load-bearing): worst-phase front {wf[4]:+.3f}, rank {order.index(4) + 1}/{len(order)}, call '{calls['dubai'][4][2]}'")

    # sensitivity: pooled p25 with the sawtooth repair active
    print("\n######## SENSITIVITY: pooled p25 thresholds with the sawtooth-start repair")
    rep_full = {}
    for key in ALL_KEYS:
        r = dict(res[key])
        saw = np.load(os.path.join("diagnostics/plots/cs_readout_evidence", f"saw_{key}.npz"))
        r["cs"] = {"A_baseline": (saw["cs_rep_f"], saw["cs_rep_r"])}
        laps = full[key][0]
        rep_full[key] = (laps, corner_readouts(r, pims[key], laps, gate))
    d_rep = derive(rep_full, ALL_KEYS, "POOLED25")
    d_base = _load_thresholds_raw()["pooled"]
    for ax in ("f", "r"):
        print(f"  {ax}: STRONG {d_base[ax]['STRONG']:+.2f} -> {d_rep[ax]['STRONG']:+.2f} (shift {abs(d_rep[ax]['STRONG'] - d_base[ax]['STRONG']):.2f}); "
              f"MODERATE {d_base[ax]['MODERATE']:+.2f} -> {d_rep[ax]['MODERATE']:+.2f}; export-noise floor 0.016 / 0.159 (median / p90)")
        print(f"     repaired trail: {d_rep[ax]['strong_trail']}; {d_rep[ax]['moderate_trail']}")


C4_PROVENANCE = [
    # (thesis_notes.md line, heading as recorded, role, figures)
    (10052, "12. PLAN.md STEP 2: chair-comparable result plots, kinematic vs ekf_pass_1 [2026-08-31]",
     "established: C4 front saturation source-independent, not a beta artifact",
     "diagnostics/plots_step2/C04_kinematic.png, C04_ekf_pass_1.png"),
    (10634, "Mechanism investigation: wholesale-negative CS_ratio under ekf_auto_pacejka [2026-09-02]",
     "established: C4 fold at the population level, consistent with STEP 2", ""),
    (10804, "Phase-level validity diagnostic: apex_3's fixed window budget conflicts with the C4-vs-artifact distinction [2026-09-02]",
     "established (used as the reference genuine case)", ""),
    (11040, "Ground-truth workup: per-run verdicts for the long-run corners -- C4 REAL (fold/peak, all 5 runs) [2026-09-02]",
     "established: C4 REAL on 5 runs, 17-32 m, visual fold/peak reads",
     "diagnostics/plots_ground_truth/C04_f_lap1_run.png, C04_f_lap3_run.png, C04_f_lap4_run.png, C04_r_lap3_run.png, C04_r_lap4_run.png"),
    (11192, "Geometric fold-vs-loop candidates evaluated: none of three cleanly separate the ground truth [2026-09-02]",
     "context: no geometric criterion separated REAL from ARTIFACT", ""),
    (11448, "CS validity repair, part A, Phase 4: final distributions ... C4's own established genuine ... [2026-09-02]",
     "carried: C4 as the event the repair must keep", ""),
    (12321, "CS validity repair, limitation: C4's own short genuine events are only partially detected ... [2026-09-02]",
     "carried: C4 short events, partially detected by worst-lap/apex_region", ""),
    (12372, "Threshold anchoring, Phase 2: four CS_ratio thresholds derived as candidates ... [2026-09-02]",
     "load-bearing: STRONG_CSF/CSR derived with C4 excluded as ground truth",
     "diagnostics/plots_threshold_investigation/C04_ekf_auto_pacejka.png, C04_kinematic.png"),
    (21961, "Feedback trace, Dubai C4 [2026-09-28, RETROACTIVE record]", "context: driver feedback C4 exit", ""),
    (22593, "Gated-kinematic sideslip diagnostic [2026-09-28]", "used as bar: 'C4 front saturation must survive'",
     "diagnostics/plots_gated_kinematic/ (gitignored)"),
    (22881, "Werner Eq. 4.3 v1: damping vs inertial yaw moment [2026-09-29]", "cross-reference: C4 exit D_psi not negative", ""),
    (23125, "CS smoothing/persistence sweep, ten rows x nine sessions [2026-09-29]",
     "used as hard bar: rows losing C4 disqualified", "diagnostics/plots/new_laps/flapping_vs_fidelity.png"),
    (23204, "CS_ratio value lens ... fold/loop [2026-09-29]",
     "downgrade step 1: worst-window fold/loop test has no power", "diagnostics/plots/cs_value_lens/fold_loop.png"),
    (23285, "Slip-vs-force lag check [2026-09-29]", "C4 negative not lag-made (lap 3 -0.67 vs lag-only -0.07)", ""),
    (23332, "Fold/loop at run scale, with lag correction and a specificity control [2026-09-29]",
     "DOWNGRADED to UNCONFIRMED: fold not reproduced, 100 m test non-specific (controls 22/30 loop)",
     "diagnostics/plots/cs_readout_evidence/fold_run_scale.png"),
    (23402, "Pooled read-out prototype vs production, all sessions [2026-09-29]",
     "C4 negative only in the worst lap (pooled median +0.45)", ""),
    (23462, "Synthesis for the CS read-out decision [2026-09-29]", "C4 status stated: unconfirmed", ""),
    (23612, "Three-way decision table: A0 / A / B [2026-09-29]", "C4 reported, not load-bearing", ""),
]


def render():
    """Addendum: plan-B traces, worst-lap vs pooled-p25 side by side, C4
    provenance sheet. Value level only: no thresholds, no verdicts."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_cs_value_lens import _brackets
    P = load_parameters()
    se = P["stability_estimation"]
    y_lo, y_hi = None, None
    for sect in P.values():
        if isinstance(sect, dict) and "cs_ls_panel_y_min" in sect:
            y_lo, y_hi = sect["cs_ls_panel_y_min"], sect["cs_ls_panel_y_max"]
    res = load_all()
    with open(RO_PATH, "rb") as fh:
        full = pickle.load(fh)
    out = os.path.join(OUT_DIR, "render")
    os.makedirs(out, exist_ok=True)
    ink, thin, band, ref = "#1f2937", "#9ca3af", "#e5e7eb", "#6b7280"
    for key in ALL_KEYS:
        r = res[key]
        laps, ro = full[key]
        pim = phase_index_map(r, load_supp2(key), se)
        t, s = r["time"], r["s_m"]
        br = _brackets(r)
        # 1. plan-B traces
        fig, axes = plt.subplots(2, 1, figsize=(max(14, (np.nanmax(s) if np.isfinite(s).any() else 5000) / 400), 7),
                                 sharex=True, constrained_layout=True)
        for ax, ai, ax_key in ((axes[0], 0, "f"), (axes[1], 1, "r")):
            cs = r["cs"]["A_baseline"][ai].astype(float)
            for b0, b1 in br.values():
                ax.axvspan(b0, b1, color=band, lw=0, zorder=0)
            ax.axhline(0, color=ref, lw=0.8); ax.axhline(1, color=ref, lw=0.8, ls="--")
            for l in [x for x in r["laps"] if x["lap_number"] in laps]:
                m = (t >= l["start_time"]) & (t <= l["end_time"])
                ax.plot(s[m], cs[m], color=thin, lw=0.5, zorder=1)
            for cid in ro:
                # one flat segment per phase, never joined: the apex region
                # overlaps entry/exit, and a joined line would draw zigzags
                # that are not in the data
                for ph in PHASES:
                    ext = [(s[pim[(cid, l)][ph]].min(), s[pim[(cid, l)][ph]].max()) for l in laps
                           if (cid, l) in pim and pim[(cid, l)][ph].size]
                    v = ro[cid][ph][ax_key]["POOLED25"]
                    if not ext or v != v:
                        continue
                    a0, a1 = float(np.median([e[0] for e in ext])), float(np.median([e[1] for e in ext]))
                    ax.plot([a0, a1], [v, v], color=ink, lw=2.6, solid_capstyle="butt", zorder=3)
            ax.set_ylim(y_lo, y_hi)
            ax.set_ylabel(("front" if ax_key == "f" else "rear") + " CS_ratio", color=ink)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        axes[1].set_xlabel("lap distance [m]", color=ink)
        axes[0].plot([], [], color=thin, lw=0.5, label="per analysis lap")
        axes[0].plot([], [], color=ink, lw=2.6, label="pooled p25 per phase")
        axes[0].legend(frameon=False, fontsize=8, loc="lower left")
        fig.suptitle(f"{key}: per-lap CS_ratio and pooled 25th percentile per corner phase ({len(laps)} laps; "
                     f"y fixed {y_lo}..{y_hi})", color=ink, fontsize=10)
        fig.savefig(os.path.join(out, f"planb_trace_{key}.png"), dpi=110)
        plt.close(fig)
        # 2. worst lap vs pooled p25 per corner
        cids = sorted(ro, key=lambda c: br.get(c, (np.inf,))[0])
        fig, axes = plt.subplots(2, 1, figsize=(max(8, 0.5 * len(cids)), 6), sharex=True, constrained_layout=True)
        x = np.arange(len(cids))
        for ax, ax_key in ((axes[0], "f"), (axes[1], "r")):
            wl = [_worst(ro[c], ax_key, "PROD") for c in cids]
            p25 = [_worst(ro[c], ax_key, "POOLED25") for c in cids]
            for xi, a, b in zip(x, wl, p25):
                if a == a and b == b:
                    ax.plot([xi, xi], [a, b], color=thin, lw=1)
            ax.plot(x, wl, "o", mfc="white", mec=ink, ms=7, label="worst lap (production)")
            ax.plot(x, p25, "o", mfc=ink, mec=ink, ms=5, label="pooled p25")
            ax.axhline(0, color=ref, lw=0.8); ax.axhline(1, color=ref, lw=0.8, ls="--")
            ax.set_ylim(y_lo, y_hi)
            ax.set_ylabel(("front" if ax_key == "f" else "rear") + " worst-phase CS_ratio", color=ink)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        axes[0].legend(frameon=False, fontsize=8, loc="lower left")
        axes[1].set_xticks(x, [f"C{c}" for c in cids], fontsize=8)
        axes[1].set_xlabel("corner, in track order", color=ink)
        fig.suptitle(f"{key}: worst-lap vs pooled-p25 per corner (worst phase each)", color=ink, fontsize=10)
        fig.savefig(os.path.join(out, f"worstlap_vs_p25_{key}.png"), dpi=110)
        plt.close(fig)
    # 3. C4 provenance sheet
    lines = ["C4 PROVENANCE SHEET (Dubai C4 front saturation) -- references only, no re-argument",
             "Source: thesis_notes.md (line numbers as of 2026-09-29). Order = record order.", ""]
    for ln, head, role, figs in C4_PROVENANCE:
        lines.append(f"- line {ln}: {head}")
        lines.append(f"    role: {role}")
        if figs:
            lines.append(f"    figures: {figs}")
    p = os.path.join(out, "C4_provenance.txt")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"rendered {len(ALL_KEYS)} x 2 figures and {p} into {out}")


class _nullctx:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


if __name__ == "__main__":
    {"sens": sens, "thresholds": thresholds, "part2": part2, "margin": margin, "table": table,
     "render": render}[sys.argv[1]]()
