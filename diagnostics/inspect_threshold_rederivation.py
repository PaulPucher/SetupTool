# THRESHOLD RE-DERIVATION DIAGNOSTIC on the final estimator (PLAN.md Q-CAL:
# "threshold re-derivation happens ONCE after all three levers"; overnight
# package step 3, 2026-10-03). PROPOSALS ONLY: nothing in config/ or modules/
# changes; the author rules on the numbers. Method of record for verdict
# thresholds stays the author's visual check on rendered CS traces (PHASE 1
# PLAN rule 2a); this diagnostic supplies the derived candidates and the
# percentile each threshold encodes.
# Tier B (threshold selection on a data distribution, preprocessing-class),
# procedure = the recorded one made explicit in inspect_cs_decision_table.py
# (header PART 1), reused unchanged through its strong_rule / moderate_rule.
#
# RULES, frozen 2026-10-03 BEFORE any number.
# STATE: production code and config as of lever 2 (Q-CAL-1 kerb dilation +
#   near_strike mark, Q-CAL-2 NBR zero entries, schema 9). Per session a fresh
#   parse_csv and the production chain as tests/conftest.pipeline_result
#   (resolve_sideslip_beta at the configured sideslip_source, slip, forces,
#   CS, stability, summarise_corners), accuracy cap 1 (reproducible; the
#   2026-09-29 procedure ran at cap 1 too).
# POPULATION: what _classify_corner reads -- per stable corner the
#   aggregate_by_corner value (config cs_cross_lap_aggregation = worst_lap)
#   over the session's analysis laps, then the minimum over the five phases
#   (apex_3 CS from apex_region). Per axle. Stability: the same with the
#   median combiner (production) for stab_neg_thresh.
# SESSIONS: the standard eight of the recorded procedure (dubai, v3,
#   pr_us_v2, nbr_jst_problem, nbr_jst_solution, lmn_mla, nbr_qp_cdr,
#   nbr_qp_mla). PRIMARY pool = seven, Le Mans excluded (PHASE 1 PLAN rule 1:
#   banned from scoring, different tyres); the eight-session pool is reported
#   beside it as the recorded population.
# RULES (recorded, unchanged): STRONG = physical anchor 0, standout group
#   beyond the largest gap between consecutive negatives removed, noise floor
#   = most negative remaining value, moved outward to the next 0.01. MODERATE
#   = midpoint of the first gap >= 0.10 among positives below the 1.0 clip.
#   No ground-truth exclusions (none confirmed; Dubai C4 unconfirmed).
#   stab_neg_thresh: the same largest-gap rule applied to negative worst-phase
#   stability values, labelled mechanical (its recorded method is gap
#   selection on the worst-phase distribution); no negatives -> stays BLOCKED.
# PERCENTILE ENCODED: share of the population's finite corner values strictly
#   below the threshold, in percent, for current and proposed values.
# cs_margin: NOT re-derived here -- its recorded method (the 22-run +/-1 %
#   sensitivity sweep) measures distances to the STRONG / MODERATE values, so
#   it follows the author's ruling on those; reported as pending.
# JST 29 FRONT TAIL CHECK (carried from Q-CAL-1, ruled "looked at during
#   re-derivation"): on every session, share of finite analysis-lap CS < -1
#   per axle; of those, share inside the kerb smear zone or carrying the
#   near_strike mark, against the base share of all finite samples there
#   (enrichment). Plus the JST 29 front corner values below -1 and below the
#   proposed STRONG.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_threshold_rederivation compute <key>
#   python -m diagnostics.inspect_threshold_rederivation report
#   python -m diagnostics.inspect_threshold_rederivation counts
#
# MODE counts (author direction ruling 2, 2026-10-04; frozen before the run):
#   STRONG at p10 / p15, MODERATE at p30 / p40 of the PRIMARY pool's corner
#   values per axle (np.percentile, linear interpolation), four combinations.
#   Per session and axle: STRONG count = values < STRONG, MODERATE count =
#   STRONG <= value < MODERATE (the tiers of _classify_corner), of the
#   session's finite corners. Counts only, no config write. Le Mans listed
#   for information, outside the pool.

import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

OUT_DIR = "diagnostics/plots/threshold_rederivation"
CACHE = os.path.join(OUT_DIR, "cache")
KEYS = ["dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "lmn_mla", "nbr_qp_cdr", "nbr_qp_mla"]
EXCLUDED = ("lmn_mla",)
PHASES = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
NAMES = {"dubai": "Dubai sample", "v3": "Paul Ricard, MLA outing 25", "pr_us_v2": "Paul Ricard, MLA outing 32",
         "nbr_jst_problem": "Nurburgring, JST outing 21", "nbr_jst_solution": "Nurburgring, JST outing 29",
         "lmn_mla": "Le Mans, MLA", "nbr_qp_cdr": "Nurburgring qualifying, CDR",
         "nbr_qp_mla": "Nurburgring qualifying, MLA"}


def compute(key):
    from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.csv_parser import parse_csv
    from modules.recommendation import aggregate_by_corner
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, estimate_vertical_loads,
                                            summarise_corners)
    from modules.tyre_fit_auto import resolve_sideslip_beta
    path = SESSIONS[key]
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
    data = parse_csv(path)
    state = prepare_vehicle_state(data["channels"], params)
    src = params["stability_estimation"]["sideslip_source"]
    beta, _man, gate, fb, fr = resolve_sideslip_beta(state, params, data, src, csv_path=path)
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, params)
    summaries = summarise_corners(data.get("corners", []), cs, stab, state, fz=fz, lap_filter=None)
    laps = [l for l in data["laps"] if l.get("is_valid_for_analysis")]
    valid = {l["lap_number"] for l in laps}
    agg = aggregate_by_corner([s for s in summaries if s["lap_number"] in valid])
    corners = {}
    for cid, a in agg.items():
        if cid is None:
            continue
        row = {}
        for ax in "fr":
            vals = {}
            for ph in PHASES:
                v = a["phases"][ph][f"cs_ratio_{ax}"]["median"]
                if ph == "apex_3" and a.get("apex_region") is not None:
                    v = a["apex_region"][f"cs_ratio_{ax}"]["median"]
                vals[ph] = v
            fin = {p: v for p, v in vals.items() if v == v}
            row[ax] = (min(fin.values()), min(fin, key=fin.get)) if fin else (float("nan"), None)
        st = {ph: a["phases"][ph]["stability_observed_Nm_per_deg"]["median"] for ph in PHASES}
        fin = {p: v for p, v in st.items() if v == v}
        row["stab"] = (min(fin.values()), min(fin, key=fin.get)) if fin else (float("nan"), None)
        corners[cid] = row
    t = state["time"]
    on_lap = np.zeros(t.size, dtype=bool)
    for l in laps:
        on_lap |= (t >= l["start_time"]) & (t <= l["end_time"])
    tail = {}
    for ax in "fr":
        c = np.asarray(cs[f"CS_ratio_{ax}"], float)
        fin = np.isfinite(c) & on_lap
        near = (np.asarray(state["kerb_smear_zone"], bool) | np.asarray(cs[f"near_strike_{ax}"], bool))
        deep = fin & (c < -1)
        tail[ax] = {"n_fin": int(fin.sum()), "n_deep": int(deep.sum()), "n_neg": int((fin & (c < 0)).sum()),
                    "deep_near": int((deep & near).sum()), "fin_near": int((fin & near).sum())}
    out = {"key": key, "gate": (gate or {}).get("verdict"), "fallback": fb, "fallback_reason": fr,
           "n_laps": len(laps), "corners": corners, "tail": tail}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] gate {out['gate']} fallback {fb}; {len(corners)} corners, {len(laps)} analysis laps; "
          f"tail {tail}", flush=True)


def pct_below(vals, x):
    v = np.array([u for u in vals if u == u])
    return float(100 * np.mean(v < x)) if v.size else float("nan")


def report():
    import json
    from diagnostics.inspect_cs_decision_table import strong_rule, moderate_rule
    from modules.stability_analysis import load_parameters
    cls = load_parameters()["classification"]
    cur = {"STRONG_CSF": cls["STRONG_CSF"]["value"], "STRONG_CSR": cls["STRONG_CSR"]["value"],
           "MODERATE_CSF": cls["MODERATE_CSF"]["value"], "MODERATE_CSR": cls["MODERATE_CSR"]["value"],
           "stab_neg_thresh_Nm_per_deg": cls["stab_neg_thresh_Nm_per_deg"]["value"],
           "cs_margin": cls["verdict_stability_margin"]["cs_margin"]}
    res = {}
    for k in KEYS:
        p = os.path.join(CACHE, f"{k}.pkl")
        if os.path.exists(p):
            with open(p, "rb") as fh:
                res[k] = pickle.load(fh)
    lines = ["THRESHOLD RE-DERIVATION DIAGNOSTIC, final estimator (levers 1 + 2) -- PROPOSALS ONLY",
             f"sessions computed: {list(res)}; missing: {[k for k in KEYS if k not in res]}",
             "current config values: " + json.dumps(cur)]
    for k, r in res.items():
        lines.append(f"  {k} ({NAMES[k]}): gate {r['gate']}, fallback {r['fallback']}, {len(r['corners'])} corners, "
                     f"{r['n_laps']} analysis laps")
    pools = {"primary (7 sessions, Le Mans excluded)": [k for k in res if k not in EXCLUDED],
             "recorded population (8 sessions)": list(res)}
    vals = lambda keys, q: [res[k]["corners"][c][q][0] for k in keys for c in res[k]["corners"]]
    out = {"current": cur, "pools": {}, "per_session": {}}
    for pname, keys in pools.items():
        lines.append(f"\n=== POOL: {pname} ===")
        pool = {}
        for ax, tag in (("f", "CSF"), ("r", "CSR")):
            v = vals(keys, ax)
            fv = np.array([u for u in v if u == u])
            s, st = strong_rule(v)
            m, mt = moderate_rule(v)
            lines.append(f"  {tag}: n {fv.size} corners; p5 / p10 / p25 / median / p75 {np.percentile(fv, 5):+.3f} / "
                         f"{np.percentile(fv, 10):+.3f} / {np.percentile(fv, 25):+.3f} / {np.median(fv):+.3f} / "
                         f"{np.percentile(fv, 75):+.3f}")
            lines.append(f"     STRONG trail: {st}")
            lines.append(f"     MODERATE trail: {mt}")
            for name, c, p_ in ((f"STRONG_{tag}", cur[f"STRONG_{tag}"], s), (f"MODERATE_{tag}", cur[f"MODERATE_{tag}"], m)):
                lines.append(f"     {name}: current {c:+.2f} (encodes p{pct_below(v, c):.1f}) -> proposed "
                             f"{p_:+.2f} (encodes p{pct_below(v, p_):.1f})" if p_ == p_ else
                             f"     {name}: current {c:+.2f} (encodes p{pct_below(v, c):.1f}) -> proposed UNDEFINED")
                pool[name] = {"current": c, "current_pct": pct_below(v, c), "proposed": p_,
                              "proposed_pct": pct_below(v, p_) if p_ == p_ else None}
        sv = vals(keys, "stab")
        fsv = np.array([u for u in sv if u == u])
        neg = fsv[fsv < 0]
        c = cur["stab_neg_thresh_Nm_per_deg"]
        if neg.size:
            s, st = strong_rule(list(fsv))
        else:
            s, st = float("nan"), "no negative worst-phase stability -> BLOCKED (as 2026-09-02)"
        lines.append(f"  stability: n {fsv.size} corners; min {fsv.min():+.1f}, p5 {np.percentile(fsv, 5):+.1f}, "
                     f"median {np.median(fsv):+.1f} Nm/deg; negatives {neg.size}")
        lines.append(f"     stab_neg_thresh: current {c:+.1f} (encodes p{pct_below(sv, c):.1f}); mechanical rule: {st}")
        pool["stab_neg_thresh_Nm_per_deg"] = {"current": c, "current_pct": pct_below(sv, c), "n_negative": int(neg.size),
                                              "mechanical": s, "trail": st}
        out["pools"][pname] = pool
    lines.append("\n=== PER SESSION (recorded rules on one session at a time) ===")
    for k in res:
        row = {}
        parts = []
        for ax, tag in (("f", "CSF"), ("r", "CSR")):
            v = vals([k], ax)
            s, _ = strong_rule(v)
            m, _ = moderate_rule(v)
            row[tag] = (s, m)
            parts.append(f"STRONG_{tag} {s:+.2f} / MODERATE_{tag} {m:+.2f}; current STRONG encodes "
                         f"p{pct_below(v, cur['STRONG_' + tag]):.0f}")
        out["per_session"][k] = row
        lines.append(f"  {k}: " + " | ".join(parts))
    lines.append("\n=== JST 29 FRONT TAIL CHECK (carried from Q-CAL-1) ===")
    for k, r in res.items():
        for ax in "fr":
            t = r["tail"][ax]
            deep = 100 * t["n_deep"] / max(1, t["n_fin"])
            near_deep = 100 * t["deep_near"] / max(1, t["n_deep"])
            near_base = 100 * t["fin_near"] / max(1, t["n_fin"])
            lines.append(f"  {k} {ax}: CS < -1 {deep:.2f} % of {t['n_fin']} finite analysis-lap samples; "
                         f"of those near a strike (smear zone or near_strike) {near_deep:.1f} % vs base {near_base:.1f} % "
                         f"(enrichment x{near_deep / near_base if near_base else float('nan'):.2f})")
    if "nbr_jst_solution" in res:
        jf = {c: v["f"] for c, v in res["nbr_jst_solution"]["corners"].items()}
        sp = out["pools"]["primary (7 sessions, Le Mans excluded)"]["STRONG_CSF"]["proposed"]
        below1 = sorted((round(v[0], 3), c, v[1]) for c, v in jf.items() if v[0] == v[0] and v[0] < -1)
        belows = sorted((round(v[0], 3), c, v[1]) for c, v in jf.items() if v[0] == v[0] and v[0] < sp)
        lines.append(f"  JST 29 front corner values < -1: {below1 or 'none'}")
        lines.append(f"  JST 29 front corner values below proposed STRONG_CSF {sp:+.2f}: {belows or 'none'}")
    lines.append(f"\ncs_margin: current {cur['cs_margin']} -- NOT re-derived (recorded method = 22-run sensitivity sweep "
                 f"against the ruled STRONG / MODERATE values; pending the author's ruling)")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)
    print(text)
    render(res, cur, out)


def render(res, cur, out):
    from diagnostics import inspect_last_push as lp
    plt = lp._plt()
    keys = [k for k in res if k not in EXCLUDED]
    pp = out["pools"]["primary (7 sessions, Le Mans excluded)"]
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
    for A, ax, tag in ((axs[0], "f", "CSF"), (axs[1], "r", "CSR")):
        for i, k in enumerate(keys):
            v = [res[k]["corners"][c][ax][0] for c in res[k]["corners"]]
            v = np.clip([u for u in v if u == u], -2.0, 1.0)
            A.scatter(v, np.full(len(v), i) + np.linspace(-0.2, 0.2, len(v)), s=12, color=lp.INK, alpha=0.7)
        for name, col in ((f"STRONG_{tag}", lp.ACC), (f"MODERATE_{tag}", lp.ACC2)):
            A.axvline(cur[name], color=col, lw=1.2, ls="--", label=f"current {name.split('_')[0].lower()} {cur[name]:+.2f}")
            p = pp[name]["proposed"]
            if p == p:
                A.axvline(p, color=col, lw=1.6, label=f"re-derived {name.split('_')[0].lower()} {p:+.2f}")
        A.set_yticks(range(len(keys)))
        A.set_yticklabels([NAMES[k] for k in keys], fontsize=7)
        A.set_xlabel("worst-phase, worst-lap cornering-stiffness ratio per corner (clipped at -2)")
        A.set_title(f"{'front' if ax == 'f' else 'rear'} axle", fontsize=10)
        A.legend(fontsize=7)
        lp._style(A)
    fig.suptitle("Classifier input per corner on the final estimator, seven sessions (Le Mans excluded): current "
                 "verdict thresholds (dashed) vs re-derived proposals (solid)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT_DIR, "corner_population_thresholds.png"), dpi=130)
    plt.close(fig)


def counts():
    import json
    res = {}
    for k in KEYS:
        with open(os.path.join(CACHE, f"{k}.pkl"), "rb") as fh:
            res[k] = pickle.load(fh)
    pool = [k for k in KEYS if k not in EXCLUDED]
    lines = ["THRESHOLD COUNTS, final estimator: STRONG at p10 / p15, MODERATE at p30 / p40 of the seven-session "
             "pool -- counts only, no config write"]
    out = {}
    for ax, tag in (("f", "front"), ("r", "rear")):
        v = np.array([res[k]["corners"][c][ax][0] for k in pool for c in res[k]["corners"]], dtype=float)
        v = v[np.isfinite(v)]
        pc = {p: float(np.percentile(v, p)) for p in (10, 15, 30, 40)}
        lines.append(f"\n{tag.upper()} AXLE (pool n {v.size}): p10 {pc[10]:+.3f}, p15 {pc[15]:+.3f}, "
                     f"p30 {pc[30]:+.3f}, p40 {pc[40]:+.3f}")
        out[tag] = {"percentiles": pc, "combos": {}}
        for s in (10, 15):
            for m in (30, 40):
                S, M = pc[s], pc[m]
                lines.append(f"  STRONG p{s} ({S:+.3f}) / MODERATE p{m} ({M:+.3f}):")
                combo = {}
                for k in KEYS:
                    x = np.array([res[k]["corners"][c][ax][0] for c in res[k]["corners"]], dtype=float)
                    x = x[np.isfinite(x)]
                    ns, nm = int(np.sum(x < S)), int(np.sum((x >= S) & (x < M)))
                    combo[k] = {"strong": ns, "moderate": nm, "n": int(x.size)}
                    note = "  [Le Mans, outside the pool, information]" if k in EXCLUDED else ""
                    lines.append(f"    {NAMES[k]}: strong {ns}, moderate {nm}, of {x.size} corners{note}")
                tot = [combo[k] for k in pool]
                lines.append(f"    pool total: strong {sum(c['strong'] for c in tot)}, moderate "
                             f"{sum(c['moderate'] for c in tot)}, of {sum(c['n'] for c in tot)} corners")
                out[tag]["combos"][f"S_p{s}_M_p{m}"] = combo
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "counts.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "counts.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print(text)


if __name__ == "__main__":
    if sys.argv[1] == "compute":
        compute(sys.argv[2])
    elif sys.argv[1] == "counts":
        counts()
    else:
        report()
