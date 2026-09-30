# DIAGNOSTIC, read-only, [keep-reproduces]. PLAN.md EXPERIMENT PLAN (g) step
# 0 (approved 2026-09-30 with the author's amendments): in-memory
# counterfactual for "kerb samples excluded inside CS windows", alone and
# with the parked sawtooth window-start repair. Nothing in modules/, core/
# or config/ changes. Tier B (masking inside a regression window).
#
# Usage (repo root, venv active; caches only):
#   python -m diagnostics.inspect_cs_kerb_window compute <key>
#   python -m diagnostics.inspect_cs_kerb_window score tier1|all
# Inputs: new_laps cache (production alpha_f/r as float32, cached production
# CS, summaries, traversals, moving = moving & ~kerb), value-lens supp
# (production Fy_f/Fy_r filtered, float64), imu_offsets cache (production
# kerb mask); time bases asserted identical. Outputs in
# diagnostics/plots/cs_kerb_window/ (gitignored).
#
# RULES (fixed before the first run)
# Variants, computed in ONE pass per axle by a line-for-line copy of
# compute_cs_for_axle (after inspect_cs_readout_evidence._cs_axle_dual):
#  BASE  production rule. Parity vs the cached production CS reported (the
#        cached alpha is float32, so ~1e-7, not bit identity); all
#        comparisons use BASE from the same pass, so differences are the
#        variant's alone.
#  K     kerb samples excluded inside windows: the window grows back until
#        it holds >= min_window NON-kerb samples and their alpha span clears
#        cs_min_slip_angle_span_rad, same cs_max_window_m cap; otherwise no
#        estimate. Regression, R2, section slopes (per monotonic section, over
#        its non-kerb samples) and the linear-reference condition read
#        non-kerb samples only. Endpoints unchanged (already non-kerb).
#  KS    K plus the sawtooth start repair: start_rep(i) = max(cap limit,
#        min(start_K(i), start_rep(i-1) + 2)), history reset where the
#        previous sample produced no K window; regression over the non-kerb
#        samples of the repaired window.
#  Self-check: K with an all-false kerb mask reproduces BASE exactly on the
#  first 20 000 samples of JST 29.
# Summaries per variant: cached production summaries with the CS phase
# medians replaced by the variant's, via the recorded phase-index map
# (apex_3 = apex region; cs_phase_min_valid_samples gate); parity of the
# map against the cached summaries reported.
# EXIT LEGS (pre-registered; a variant earns the boundary only if all hold):
#  1 ARTIFACT COLLAPSE: near-edge enrichment of per-corner-lap CS minima
#    (inspect_cs_kerb_edge rule, 3 m) -- its excess over chance (enrichment
#    - 1) falls by at least half vs BASE, pooled, on BOTH axles.
#    (Operationalises "drops toward chance"; if it persists, the
#    association was apex physics and the fix does not ship.)
#  2 NO NEW DEEP DIPS: per session and axle, corner-lap minima that fall
#    below the BASE session p10 while their BASE minimum was above it, and
#    lie > 3 m from any kerb-mask edge: at most 1 per session and axle; and
#    the rendered standard trace sets (BASE / K / KS stacked) show no new
#    dip pattern on inspection (recorded in words).
#  3 SURVIVAL BATTERY, kerb-adjacent elements (author amendment): Dubai C3
#    LS_r exit_5 < 0 on 4/4 laps (LS is not a CS input -- identical by
#    construction, reported); worst-phase r - f > 0 at Dubai C3, US-v2 C13,
#    NBR B (JST 29) C9; NBR A->B median front change > 0 and gap change <
#    0. Any element passing at BASE must still pass. The kerb-sample content
#    of each element's windows is reported beside it.
#  Informative: coverage (finite CS share of corner samples), changed
#  corner calls (US/OS/ok), pooled lap-pair switch rate, median |phase-
#  median change| vs BASE, BASE/variant deep-tail membership.
# TIERING: Tier 1 = screening JST 29, US-v2, MLA (legs 1-2 and the US-v2
#  C13 / JST 29 C9 gaps). Tier 2 = all eight, only if a variant passes Tier
#  1; the exit decision is taken on all eight.

import gzip
import os
import pickle
import sys

import numpy as np

from diagnostics import inspect_cs_kerb_edge as ke

KEYS = ke.KEYS
TIER1 = ["nbr_jst_solution", "pr_us_v2", "nbr_qp_mla"]
LABEL = ke.LABEL
VARIANTS = ["BASE", "K", "KS"]
OUT_DIR = "diagnostics/plots/cs_kerb_window"
SELF_CHECK_N = 20000


def _cache(d, key):
    with gzip.open(os.path.join("diagnostics/plots", d, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def cs_triple(alpha, Fy, moving, kerb, s_m, params, min_window, variants=VARIANTS):
    """BASE / K / KS CS_ratio in one pass (header). Copy of
    compute_cs_for_axle for BASE; K and KS as specified."""
    from modules.stability_analysis import (_find_monotonic_sections, _section_slopes, _smooth_weight,
                                            SPAN_WEIGHT_EXPONENT, R2_WEIGHT_EXPONENT)
    se = params["stability_estimation"]
    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    max_window_m = se["cs_max_window_m"]
    n = len(alpha)
    valid = ~kerb
    sections, section_id = _find_monotonic_sections(alpha)
    sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)
    # section slopes over non-kerb samples (K, KS)
    ks_slopes, ks_spans = np.full(len(sections), np.nan), np.zeros(len(sections))
    for k, (a0, b0) in enumerate(sections):
        m = valid[a0:b0]
        if m.sum() < 2:
            continue
        a, f = alpha[a0:b0][m], Fy[a0:b0][m]
        den = np.sum((a - a.mean()) ** 2)
        if den < 1e-10:
            continue
        ks_slopes[k] = np.sum((a - a.mean()) * (f - f.mean())) / den
        ks_spans[k] = a.max() - a.min()
    S = {v: {"ref": np.nan, "cs": np.full(n, np.nan)} for v in variants}

    def estimate(v, st, i, use_valid):
        if use_valid:
            m = valid[st:i]
            wa, wf, ids = alpha[st:i][m], Fy[st:i][m], section_id[st:i][m]
            slopes, spans = ks_slopes, ks_spans
        else:
            wa, wf, ids = alpha[st:i], Fy[st:i], section_id[st:i]
            slopes, spans = sec_slopes, sec_spans
        if wa.size < 2:
            return
        a_mean, f_mean = np.mean(wa), np.mean(wf)
        denom = np.sum((wa - a_mean) ** 2)
        if denom < 1e-10:
            return
        C_w = np.sum((wa - a_mean) * (wf - f_mean)) / denom
        f_hat = C_w * wa + (f_mean - C_w * a_mean)
        ss_res = np.sum((wf - f_hat) ** 2)
        ss_tot = np.sum((wf - f_mean) ** 2)
        R2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0
        weights, sl = [], []
        for k in np.unique(ids):
            if np.isnan(slopes[k]):
                continue
            w_k = _smooth_weight(spans[k], 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
            if w_k <= 0:
                continue
            weights.append(w_k); sl.append(slopes[k])
        if weights:
            C_s = float(np.sum(np.array(weights) * np.array(sl)) / np.sum(weights))
            w_r2 = _smooth_weight(R2, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
            C_a = w_r2 * C_w + (1.0 - w_r2) * C_s
        else:
            C_a = C_w
        if np.max(np.abs(wa)) < linear_thresh:
            S[v]["ref"] = C_a
        if not np.isnan(S[v]["ref"]) and S[v]["ref"] > 0:
            S[v]["cs"][i] = min(C_a / S[v]["ref"], 1.0)

    def capped(s_i, st):
        s_start = s_m[st]
        return not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m

    prev_ks = None
    for i in range(min_window, n):
        if not moving[i]:
            prev_ks = None
            continue
        s_i = s_m[i - 1]
        if not np.isfinite(s_i):
            s_i = None
        # BASE: production growth
        if "BASE" in S:
            start = i - min_window
            if start > 0:
                wmax, wmin = np.max(alpha[start:i]), np.min(alpha[start:i])
            while start > 0:
                if wmax - wmin >= min_span:
                    break
                if s_i is not None and capped(s_i, start):
                    break
                start -= 1
                wmax, wmin = max(wmax, alpha[start]), min(wmin, alpha[start])
            w = alpha[start:i]
            if np.max(w) - np.min(w) >= min_span:
                estimate("BASE", start, i, False)
        if "K" not in S and "KS" not in S:
            continue
        # K: grow until >= min_window non-kerb samples and their span clears
        st = i - min_window
        m = valid[st:i]
        nv = int(m.sum())
        vmax = np.max(alpha[st:i][m]) if nv else -np.inf
        vmin = np.min(alpha[st:i][m]) if nv else np.inf
        while st > 0:
            if nv >= min_window and vmax - vmin >= min_span:
                break
            if s_i is not None and capped(s_i, st):
                break
            st -= 1
            if valid[st]:
                nv += 1
                vmax, vmin = max(vmax, alpha[st]), min(vmin, alpha[st])
        if nv < min_window or vmax - vmin < min_span:
            prev_ks = None
            continue
        if "K" in S:
            estimate("K", st, i, True)
        if "KS" in S:
            rep = st if prev_ks is None else min(st, prev_ks + 2)
            while s_i is not None and rep < st and (not np.isfinite(s_m[rep]) or s_i - s_m[rep] > max_window_m):
                rep += 1
            prev_ks = rep
            estimate("KS", rep, i, True)
    return {v: S[v]["cs"] for v in S}


def _inputs(key):
    nl = _cache("new_laps", key)
    st = _cache("imu_offsets", key)
    st = st.get("state", st)
    z = np.load(os.path.join("diagnostics/plots/cs_value_lens", f"supp_{key}.npz"))
    assert st["time"].tobytes() == nl["time"].tobytes() and z["time"].size == nl["time"].size, key
    return nl, np.asarray(st["kerb_mask"], bool), z


def _params():
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.stability_analysis import load_parameters
    return apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))


def compute(key):
    import time
    params = _params()
    nl, kerb, z = _inputs(key)
    s_m = nl["s_m"].astype(float)
    out = {}
    if key == "nbr_jst_solution":
        n0 = SELF_CHECK_N
        a = nl["alpha_f"][:n0].astype(float)
        chk = cs_triple(a, z["fy_f"][:n0], nl["moving"][:n0], np.zeros(n0, bool), s_m[:n0], params, nl["min_window"],
                        variants=["BASE", "K"])
        same = np.array_equal(np.isfinite(chk["BASE"]), np.isfinite(chk["K"])) and \
            np.array_equal(chk["BASE"][np.isfinite(chk["BASE"])], chk["K"][np.isfinite(chk["K"])])
        print(f"[{key}] self-check K(no kerb) == BASE on {n0} samples: {same}")
        out["self_check"] = same
    for ax, ai in (("f", 0), ("r", 1)):
        t0 = time.perf_counter()
        res = cs_triple(nl[f"alpha_{ax}"].astype(float), z[f"fy_{ax}"], nl["moving"], kerb, s_m, params, nl["min_window"])
        prod = nl["cs"]["A_baseline"][ai].astype(float)
        fa, fb = np.isfinite(prod), np.isfinite(res["BASE"])
        d = np.abs(prod[fa & fb] - res["BASE"][fa & fb])
        print(f"[{key}] {ax}: {time.perf_counter() - t0:.0f}s; BASE parity vs cached production: finite in one only "
              f"{int((fa ^ fb).sum())} of {int(fa.sum())}, |diff| median {np.median(d):.1e}, max {d.max():.1e}")
        for v, arr in res.items():
            out[f"{v}_{ax}"] = arr.astype(np.float64)
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    np.savez_compressed(os.path.join(OUT_DIR, "cache", f"{key}.npz"), **out)


def _load_var(key):
    z = np.load(os.path.join(OUT_DIR, "cache", f"{key}.npz"))
    return {v: (z[f"{v}_f"], z[f"{v}_r"]) for v in VARIANTS}


def _variant_summaries(nl, pim, cs_fr, gate):
    """Cached summaries with CS phase medians from cs_fr."""
    import copy
    from diagnostics.inspect_cs_readout_evidence import _gated_median, PHASES
    out = []
    for s in nl["summaries"]["A_baseline"]:
        s2 = copy.deepcopy(s)
        m = pim.get((s["stable_corner_id"], s["lap_number"]))
        if m is not None:
            for ph in PHASES:
                tgt = s2["apex_region"] if (ph == "apex_3" and s2.get("apex_region") is not None) else s2["phases"][ph]
                for ai, ax in ((0, "cs_ratio_f"), (1, "cs_ratio_r")):
                    tgt[ax]["median"] = _gated_median(cs_fr[ai][m[ph]], gate)
        out.append(s2)
    return out


def _r_with(nl, cs_fr):
    return {"time": nl["time"], "s_m": nl["s_m"], "moving": nl["moving"], "laps": nl["laps"],
            "traversals": nl["traversals"], "corners": nl["corners"], "cs": {"A_baseline": cs_fr}}


def _edge_stats(key, cs_fr):
    rows = ke.corner_laps(key, cs=cs_fr)
    return rows


def score(tier):
    from collections import Counter
    from modules.recommendation import aggregate_by_corner
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_cs_readout_evidence import (phase_index_map, load_supp2, corner_readouts, _calls,
                                                         _worst, _gated_median, PHASES)
    from diagnostics.inspect_new_laps_cs_sweep import phase_and_lap_flapping
    from diagnostics.inspect_cs_value_lens import _brackets
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    keys = TIER1 if tier == "tier1" else KEYS
    rng = np.random.default_rng(ke.SEED)
    D = {}
    for k in keys:
        nl, kerb, _z = _inputs(k)
        var = _load_var(k)
        pim = phase_index_map(nl, load_supp2(k), se)
        bad = 0
        for s in nl["summaries"]["A_baseline"]:
            m = pim.get((s["stable_corner_id"], s["lap_number"]))
            if m is None:
                continue
            for ph in PHASES:
                for ai, ax in ((0, "cs_ratio_f"), (1, "cs_ratio_r")):
                    prod = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                    mine = _gated_median(nl["cs"]["A_baseline"][ai].astype(float)[m[ph]], gate)
                    bad += int(not ((prod != prod and mine != mine) or abs(prod - mine) < 1e-5))
        summ = {v: _variant_summaries(nl, pim, var[v], gate) for v in VARIANTS}
        laps = sorted({l for (_c, l) in pim})
        ro = {v: corner_readouts(_r_with(nl, var[v]), pim, laps, gate) for v in VARIANTS}
        D[k] = {"nl": nl, "kerb": kerb, "var": var, "pim": pim, "summ": summ, "ro": ro, "laps": laps, "parity_bad": bad}
        print(f"[{LABEL[k]}] phase-map parity vs cached summaries: {bad} medians differ")

    print("\n=== informative: coverage, calls, switch rate, effect size")
    for k in keys:
        d = D[k]
        ti = np.unique(np.concatenate([np.arange(t["lo"], t["hi"]) for t in d["nl"]["traversals"]]))
        c0 = _calls(aggregate_by_corner(d["summ"]["BASE"]))
        for v in VARIANTS:
            cov = [float(np.mean(np.isfinite(d["var"][v][ai][ti]))) for ai in (0, 1)]
            cv = _calls(aggregate_by_corner(d["summ"][v]))
            cnt = Counter(x[0] for x in cv.values())
            changed = sum(1 for c in cv if c in c0 and cv[c][0] != c0[c][0])
            f = phase_and_lap_flapping(d["summ"][v])
            dm = []
            s0 = {(s["stable_corner_id"], s["lap_number"]): s for s in d["summ"]["BASE"]}
            for s in d["summ"][v]:
                b = s0[(s["stable_corner_id"], s["lap_number"])]
                for ph in PHASES:
                    for ax in ("cs_ratio_f", "cs_ratio_r"):
                        x = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                        y = (b["apex_region"] if ph == "apex_3" else b["phases"][ph])[ax]["median"]
                        if x == x and y == y:
                            dm.append(abs(x - y))
            print(f"  {LABEL[k]} {v}: coverage f/r {cov[0]:.3f}/{cov[1]:.3f}; calls changed {changed} "
                  f"(US {cnt.get('US', 0)}, OS {cnt.get('OS', 0)}, ok {cnt.get('-', 0)}); lap-pair switch "
                  f"{f['lap_trans']}/{f['lap_pairs']}; |phase-median change| median {np.median(dm):.4f} p90 "
                  f"{np.percentile(dm, 90):.4f}")

    print("\n=== LEG 1 -- near-edge enrichment of corner-lap minima (3 m), pooled over " + ", ".join(LABEL[k] for k in keys))
    leg1 = {}
    for v in VARIANTS:
        pooled = {"f": [], "r": []}
        for k in keys:
            rows = _edge_stats(k, D[k]["var"][v])
            for ax in "fr":
                pooled[ax] += rows[ax]
        leg1[v] = {ax: ke._stats(pooled[ax], rng) for ax in "fr"}
        print(f"  {v}: " + "; ".join(f"{ax} near {leg1[v][ax]['obs']}/{leg1[v][ax]['n']} ({leg1[v][ax]['share']:.3f}) vs "
                                   f"chance {leg1[v][ax]['chance']:.3f}, enrichment {leg1[v][ax]['enrich']:.2f}"
                                   for ax in "fr"))
    for v in ("K", "KS"):
        ok = all((leg1[v][ax]["enrich"] - 1) <= 0.5 * (leg1["BASE"][ax]["enrich"] - 1) for ax in "fr")
        leg1[v]["pass"] = ok
        print(f"  LEG 1 {v}: excess over chance f {leg1['BASE']['f']['enrich'] - 1:.2f} -> {leg1[v]['f']['enrich'] - 1:.2f}, "
              f"r {leg1['BASE']['r']['enrich'] - 1:.2f} -> {leg1[v]['r']['enrich'] - 1:.2f} -> {'PASS' if ok else 'FAIL'}")

    print("\n=== LEG 2 -- new deep minima away from kerb edges (count per session/axle; pass <= 1)")
    leg2 = {v: True for v in ("K", "KS")}
    for k in keys:
        base = _edge_stats(k, D[k]["var"]["BASE"])
        for v in ("K", "KS"):
            rows = _edge_stats(k, D[k]["var"][v])
            parts = []
            for ax in "fr":
                bmin = {(r["cid"], r["lap"]): r for r in base[ax]}
                bar = float(np.percentile([r["min"] for r in base[ax]], ke.TAIL_PCT))
                new = [r for r in rows[ax] if (r["cid"], r["lap"]) in bmin and r["min"] < bar
                       and bmin[(r["cid"], r["lap"])]["min"] >= bar and not r["near"]]
                gone = [r for r in rows[ax] if (r["cid"], r["lap"]) in bmin and r["min"] >= bar
                        and bmin[(r["cid"], r["lap"])]["min"] < bar]
                if len(new) > 1:
                    leg2[v] = False
                parts.append(f"{ax} new {len(new)}" + (" [" + "; ".join(f"C{r['cid']} lap {r['lap']} {r['min']:.2f} @ "
                             f"{r['s']:.0f} m" for r in new) + "]" if new else "") + f", left the tail {len(gone)}")
            print(f"  {LABEL[k]} {v}: " + "; ".join(parts))
    print("  LEG 2 counts: " + ", ".join(f"{v} {'PASS' if ok else 'FAIL'}" for v, ok in leg2.items()) +
          " (visual inspection of the trace sets recorded separately)")

    print("\n=== LEG 3 -- kerb-adjacent survival-battery elements")
    elems = [("dubai", 3), ("pr_us_v2", 13), ("nbr_jst_solution", 9)]
    leg3 = {v: True for v in ("K", "KS")}
    base_ok = {}
    for v in VARIANTS:
        parts = []
        if "dubai" in D:
            ls_neg = sum(1 for s in D["dubai"]["summ"][v] if s["stable_corner_id"] == 3 and
                         (s["phases"]["exit_5"].get("ls_ratio_r", {}).get("median", float("nan")) or 0) < 0)
            ok = ls_neg == 4
            parts.append(f"C3 LS_r exit_5 < 0 on {ls_neg}/4 {'PASS' if ok else 'FAIL'}")
            base_ok.setdefault("C3_LS", ok) if v == "BASE" else None
            if v != "BASE" and base_ok.get("C3_LS") and not ok:
                leg3[v] = False
        for key, cid in elems:
            if key not in D:
                continue
            ro = D[key]["ro"][v][cid]
            g = _worst(ro, "r", "PROD") - _worst(ro, "f", "PROD")
            ok = g > 0
            name = f"gap_{LABEL[key]}_C{cid}"
            if v == "BASE":
                base_ok[name] = ok
            elif base_ok.get(name) and not ok:
                leg3[v] = False
            parts.append(f"{name} {g:+.3f} {'PASS' if ok else 'FAIL'}")
        if "nbr_jst_problem" in D and "nbr_jst_solution" in D:
            A, B = D["nbr_jst_problem"], D["nbr_jst_solution"]
            ba, bb = _brackets(_r_with(A["nl"], A["var"][v])), _brackets(_r_with(B["nl"], B["var"][v]))
            pairs = [(ca, cb) for ca, (a0, a1) in sorted(ba.items()) for cb, (b0, b1) in bb.items()
                     if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0)]
            roA, roB = A["ro"][v], B["ro"][v]
            dfr = [roB[cb]["apex_3"]["f"]["PROD"] - roA[ca]["apex_3"]["f"]["PROD"] for ca, cb in pairs
                   if ca in roA and cb in roB]
            dgp = [(roB[cb]["apex_3"]["r"]["PROD"] - roB[cb]["apex_3"]["f"]["PROD"]) -
                   (roA[ca]["apex_3"]["r"]["PROD"] - roA[ca]["apex_3"]["f"]["PROD"]) for ca, cb in pairs
                   if ca in roA and cb in roB]
            dfr = [x for x in dfr if x == x]; dgp = [x for x in dgp if x == x]
            ok = np.median(dfr) > 0 and np.median(dgp) < 0
            if v == "BASE":
                base_ok["AB"] = ok
            elif base_ok.get("AB") and not ok:
                leg3[v] = False
            parts.append(f"NBR A->B front {np.median(dfr):+.3f}, gap {np.median(dgp):+.3f} {'PASS' if ok else 'FAIL'}")
        print(f"  {v}: " + "; ".join(parts))
    # kerb content of the element corners' windows (production rule)
    for key, cid in elems:
        if key not in D:
            continue
        rows = _edge_stats(key, D[key]["var"]["BASE"])
        for ax in "fr":
            rr = [r for r in rows[ax] if r["cid"] == cid]
            print(f"    {LABEL[key]} C{cid} {ax}: minima windows holding kerb samples {sum(r['win_kerb'] > 0 for r in rr)}/"
                  f"{len(rr)}, minima near an edge {sum(r['near'] for r in rr)}/{len(rr)}")
    print("  LEG 3: " + ", ".join(f"{v} {'PASS' if ok else 'FAIL'}" for v, ok in leg3.items()))
    render(D, keys)
    with open(os.path.join(OUT_DIR, f"score_{tier}.pkl"), "wb") as fh:
        pickle.dump({"leg1": leg1, "leg2": leg2, "leg3": leg3}, fh)


def render(D, keys):
    from PIL import Image
    from diagnostics.trace_render import render_cs_traces
    for k in keys:
        nl = D[k]["nl"]
        paths = []
        for v in VARIANTS:
            p = os.path.join(OUT_DIR, f"traces_{k}_{v}.png")
            render_cs_traces(p, nl["time"], nl["s_m"], D[k]["var"][v][0], D[k]["var"][v][1], nl["laps"], nl["corners"],
                             f"{LABEL[k]} {v}")
            paths.append(p)
        ims = [Image.open(p) for p in paths]
        w = max(i.size[0] for i in ims)
        comp = Image.new("RGB", (w, sum(i.size[1] for i in ims)), "white")
        y = 0
        for im in ims:
            comp.paste(im, (0, y)); y += im.size[1]
        comp.save(os.path.join(OUT_DIR, f"traces_{k}_stack.png"))


if __name__ == "__main__":
    if sys.argv[1] == "compute":
        compute(sys.argv[2])
    else:
        score(sys.argv[2])
