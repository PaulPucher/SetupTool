# STEP 3 POOLED FITS (author EXECUTE 2026-10-04, overnight order steps C / D /
# C+). Harness only: no production or config change. Production state = the
# corrected steering conversion (schema 10) + the speed window (schema 11).
#
# RULES, frozen 2026-10-04 BEFORE any pooled fit (conflict = record, do not
# re-tune after seeing numbers).
# POOLS (author-confirmed sets, thesis_notes "Pool definitions ..."):
#   T (same tyre spec): 16 NBR files + Paul Ricard v3 and pr_us_v2 (18).
#   D (driver JST): NBR outings 18, 19, 20, 21, 29, 46, 64 (7).
# STEP C (rebuild): d2.fit per member -> production chain with window. ENTRY
#   CHECK per member: status not degenerate, gate not fail, no fallback,
#   Powell converged on both axles, speed window applied. A member failing it
#   is EXCLUDED from its pools and the run continues.
# POOLED FIT: one production Pacejka fit per axle (tyre_fit_auto.
#   _fit_axle_pacejka, same start guess, signed, both axles) on the
#   concatenated members' production fit inputs -- kinematic-seed alpha and
#   axle Fy through the 2 Hz Butterworth, each member's own production fit
#   mask (base & speed window). The pool averages over tyre mileage and
#   pressure (same spec, not same tyres) and, for T, over two tracks.
# EVALUATION per member: the production chain with the member's own fit
#   replaced by the pooled curve (fitter hook of inspect_last_push.run_chain;
#   residuals for the R derivation are the member's own data against the
#   pooled curve, so R follows the curve exactly as in production). Baseline P =
#   the member's production chain from the rebuilt cache.
# ENTRY for the hook: the harness fit inputs (ma.inputs) must equal the cached
#   production fit inputs bit for bit (alpha, Fy, fit mask), else the member is
#   excluded. Tier-1 members additionally: the harness chain WITHOUT the hook
#   reproduces the cached production beta and CS (same finite sets, max
#   |diff| <= 1e-5: the cache stores float32, so the decisive frame's 1e-9
#   between two float64 runs is not attainable here).
# METRICS. Tier-1 members (full frozen frame): G, B1 M3 with P's bootstrap
#   band and the tie rule, B2 straight-line |beta|, B3 chatter (descriptive),
#   C in-corner CS < 0 / CS < -1 / coverage. Other members (author ruling 4):
#   G and C only. All members: F2 = RMS of the member's seed Fy against the
#   curve on its fit mask, own vs pooled (descriptive -- "does the pooled curve
#   describe this session's data"). F1 (leave-one-lap-out refit) and S (strike
#   sensitivity of the refit) measure the per-session fitting procedure; the
#   pooled curve is not refitted per session, so both are NOT REACHABLE here
#   (reported for the pooled fit itself: leave-one-member-out curve spread,
#   descriptive only).
# SANITY SCREEN (fit-shape rule, not physics -- Pacejka Fig. 1.7): pooled axle
#   with a non-finite parameter, B < 1, C <= 1, C > 3 or E > 1 is a HARM item.
# VERDICT per pool vs its members' own production fits:
#   HARM if any: screen fails; G fails on any member (degenerate, gate fail,
#     fallback); CS < 0 share +2.5 points or more on any member-axle; coverage
#     -1 point or more on any member-axle; Tier-1 member M3 above P's band;
#     Tier-1 member straight |beta| +0.1 deg or more.
#   EARNS = no harm AND >= 2 candidates met (absolute bar): (2) M3 credit
#     (below P's band AND <= 0.9 x P) on >= 2 Tier-1 members -- reachable only
#     with >= 2 Tier-1 members (T: 3; D: 1 -> not reachable); (3) pooled CS < -1
#     share -10 % rel; (5) pooled CS < 0 share -5 % rel. Pooled = counts summed
#     over all evaluated members and both axles. Reachable: T 3, D 2.
#   NEUTRAL otherwise.
#   NOISE CONTEXT (thesis_notes 2026-10-04): the CS < 0 share carries +/-1-2 pt
#   fallback-switch timing noise per session; the 2.5 pt bar sits above it.
# CONCEPT vs INSTANCE: a pooled curve is judged on its own merit; a verdict
#   here says nothing about pooling in general.
#
# Modes: rebuild <keys...> | pool | eval <pool> <keys...> | report | plots
# Output: diagnostics/plots/pool_fits/

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import modules.tyre_fit_auto as tfa
from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp
from diagnostics import inspect_fit_ma_calibration as ma
from modules.tyre_model_pacejka import pacejka_lateral_force

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "pool_fits")
NBR16 = ("nbr_jst_problem", "nbr_jst_solution", "nbr_o005_mla", "nbr_o010_jdr", "nbr_o018_jst", "nbr_o019_jst",
         "nbr_o020_jst", "nbr_o046_jst", "nbr_o053_mla", "nbr_o054_cdr", "nbr_o059_mla", "nbr_o061_jdr",
         "nbr_o063_cdr", "nbr_o064_jst", "nbr_qp_cdr", "nbr_qp_mla")
POOLS = {"T": NBR16 + ("v3", "pr_us_v2"),
         "D": ("nbr_o018_jst", "nbr_o019_jst", "nbr_o020_jst", "nbr_jst_problem", "nbr_jst_solution",
               "nbr_o046_jst", "nbr_o064_jst")}
POOL_NAME = {"T": "same-tyre pool (Nurburgring + Paul Ricard, 18 files)",
             "D": "driver pool JST (Nurburgring, 7 files)"}
TIER1 = lp.TIER1
AX = {"f": "front", "r": "rear"}
HARM_CS_NEG_PT = 2.5
HARM_COV_PT = 1.0
HARM_B2_DEG = 0.1
ORIG_FIT = tfa._fit_axle_pacejka
RAD2DEG = 180.0 / np.pi
G = 9.81


def _log(msg, name="run_log.txt"):
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, name), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def screen_ok(fit):
    p = [fit[k] for k in "BCDE"]
    return bool(all(np.isfinite(p)) and fit["B"] >= 1 and 1 < fit["C"] <= 3 and fit["E"] <= 1)


# ---------------------------------------------------------------- step C

def entry_check(rec):
    man = rec["prod"]["manifest"] or {}
    gate = rec["prod"]["gate"] or {}
    ax = man.get("axles", {})
    problems = []
    if man.get("status") == "degenerate":
        problems.append("status degenerate")
    if gate.get("verdict") == "fail":
        problems.append("gate fail")
    if rec["prod"]["fallback"]:
        problems.append(f"fallback ({rec['prod']['fallback_reason']})")
    for a in ("front", "rear"):
        if not ax.get(a, {}).get("powell_converged", False):
            problems.append(f"{a} Powell not converged")
    if (rec.get("window_info") or {}).get("status") != "applied":
        problems.append(f"window {(rec.get('window_info') or {}).get('status')}")
    return problems


def rebuild(keys):
    for k in keys:
        t0 = time.perf_counter()
        try:
            d2.fit(k)
            rec = d2.load_fit(k)
            prob = entry_check(rec)
        except Exception as e:  # a failing member is excluded, the run goes on
            prob = [f"exception {type(e).__name__}: {e}"]
        w = (d2.load_fit(k).get("window_info") or {}) if not any("exception" in p for p in prob) else {}
        _log(f"[rebuild] {k}: {'ENTRY OK' if not prob else 'EXCLUDED: ' + '; '.join(prob)}; window "
             f"{w.get('lo_kmh', float('nan')):.1f}-{w.get('hi_kmh', float('nan')):.1f} km/h; "
             f"{time.perf_counter() - t0:.0f} s", "rebuild_log.txt")


def excluded():
    out = {}
    path = os.path.join(OUT_DIR, "rebuild_log.txt")
    if os.path.exists(path):
        for line in open(path, encoding="utf-8"):
            if line.startswith("[rebuild]"):
                k = line.split()[1].rstrip(":")
                out[k] = "EXCLUDED" in line
    return out


def members(pool):
    ex = excluded()
    return [k for k in POOLS[pool] if ex.get(k) is False]


# ---------------------------------------------------------------- pooled fit

def pool():
    res = {}
    for name in POOLS:
        mem = members(name)
        recs = {k: d2.load_fit(k) for k in mem}
        res[name] = {"members": mem, "excluded": [k for k in POOLS[name] if k not in mem], "axes": {}}
        for ax in "fr":
            a = np.concatenate([np.asarray(recs[k][f"alpha_{ax}"], float) for k in mem])
            f = np.concatenate([np.asarray(recs[k][f"fy_{ax}"], float) for k in mem])
            m = np.concatenate([np.asarray(recs[k]["fit_mask"], bool) for k in mem])
            t0 = time.perf_counter()
            full = ORIG_FIT(a, f, m)
            fit = {k: full[k] for k in ("B", "C", "D", "E", "powell_converged", "fit_n_samples", "fit_rms_resid_N",
                                        "peak_alpha_deg", "visited_alpha_p99_deg", "sign_ok")}
            fit["origin_slope_N_per_rad"] = float(full["B"] * full["C"] * full["D"])
            fit["screen_ok"] = screen_ok(full)
            # descriptive: leave-one-member-out spread of the pooled curve
            grid = np.radians(np.linspace(0.5, 8.0, 25))
            cf = np.abs(pacejka_lateral_force(grid, full["B"], full["C"], full["D"], full["E"]))
            lomo, off = [], 0
            sizes = [np.asarray(recs[k]["fit_mask"]).size for k in mem]
            for i, n in enumerate(sizes):
                mm = m.copy()
                mm[off:off + n] = False
                off += n
                fl = ORIG_FIT(a, f, mm)
                if fl["powell_converged"] and fl["sign_ok"]:
                    lomo.append(np.abs(pacejka_lateral_force(grid, fl["B"], fl["C"], fl["D"], fl["E"])))
            if len(lomo) >= 2:
                c = np.array(lomo)
                fit["lomo_spread_median_rel"] = float(np.median(np.std(c, axis=0, ddof=1) / cf))
            fit["seconds"] = round(time.perf_counter() - t0, 1)
            res[name]["axes"][ax] = fit
            _log(f"[pool {name}] {AX[ax]}: B {fit['B']:.3f} C {fit['C']:.3f} D {fit['D']:.0f} E {fit['E']:.2f}, "
                 f"origin slope {fit['origin_slope_N_per_rad'] / 1000:.1f} kN/rad, n {fit['fit_n_samples']}, "
                 f"screen {'ok' if fit['screen_ok'] else 'FAIL'}, converged {fit['powell_converged']}, "
                 f"leave-one-member-out spread {fit.get('lomo_spread_median_rel', float('nan')) * 100:.2f} %; "
                 f"{fit['seconds']} s")
    with open(os.path.join(OUT_DIR, "pool_fits.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=lp._jsonable)


def load_pool():
    with open(os.path.join(OUT_DIR, "pool_fits.json")) as fh:
        return json.load(fh)


# ---------------------------------------------------------------- evaluation

def fixed_fitter(pfit, P, calls):
    """Returns the pooled curve for the matching axle; residuals are the
    member's own data against it, as production would compute them."""
    def f(a, fy, b):
        for ax in "fr":
            if np.array_equal(a, P[ax][0], equal_nan=True) and np.array_equal(fy, P[ax][1], equal_nan=True):
                calls.append(ax)
                p = pfit[ax]
                m2 = b & np.isfinite(a) & np.isfinite(fy)
                resid = fy[m2] - pacejka_lateral_force(a[m2], p["B"], p["C"], p["D"], p["E"])
                return {"B": p["B"], "C": p["C"], "D": p["D"], "E": p["E"],
                        "powell_converged": bool(p["powell_converged"]), "fit_n_samples": int(m2.sum()),
                        "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
                        "peak_alpha_deg": p["peak_alpha_deg"], "peak_in_visited_range": False,
                        "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a[m2])), 99)),
                        "sign_ok": p["D"] > 0, "residuals": resid, "residual_mask": m2}
        raise RuntimeError("STOP: fit called with arrays that are not the member's fit inputs")
    return f


def evaluate(pool_name, keys):
    pf = load_pool()[pool_name]
    pfit = {ax: pf["axes"][ax] for ax in "fr"}
    os.makedirs(os.path.join(OUT_DIR, "eval"), exist_ok=True)
    for k in keys:
        if k not in pf["members"]:
            _log(f"[eval {pool_name}] {k}: not a member (excluded at rebuild) -- skipped")
            continue
        t0 = time.perf_counter()
        try:
            ctx = lp.Ctx(k)
            st = ctx.state
            seed, P, _M = ma.inputs(ctx)
            rec = d2.load_fit(k)
            same = all(np.array_equal(P[ax][0], rec[f"alpha_{ax}"], equal_nan=True) and
                       np.array_equal(P[ax][1], rec[f"fy_{ax}"], equal_nan=True) for ax in "fr") and \
                np.array_equal(ctx.fit_mask, rec["fit_mask"])
            out = {"key": k, "pool": pool_name, "inputs_identical": bool(same)}
            if not same:
                _log(f"[eval {pool_name}] {k}: EXCLUDED -- harness fit inputs differ from the cached production inputs")
                continue
            if k in TIER1:
                beta_h, rec_h, dn_h = lp.run_chain(ctx, st, seed)
                bp = np.asarray(rec["prod"]["beta"], float)
                e = float(np.nanmax(np.abs(beta_h - bp)))
                fin_same = np.array_equal(np.isfinite(beta_h), np.isfinite(bp))
                for ax in "fr":
                    a_, b_ = np.asarray(dn_h[f"cs_{ax}"], float), np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)
                    fin_same &= np.array_equal(np.isfinite(a_), np.isfinite(b_))
                    ff = np.isfinite(a_) & np.isfinite(b_)
                    e = max(e, float(np.max(np.abs(a_[ff] - b_[ff]))) if ff.any() else 0.0)
                # cached beta is float32 -> compare at float32 resolution
                out["entry_max_diff"] = e
                out["entry_ok"] = bool(fin_same and e <= 1e-5)
                if not out["entry_ok"]:
                    _log(f"[eval {pool_name}] {k}: EXCLUDED -- Tier-1 entry failed (max diff {e:.2e}, finite sets "
                         f"{'same' if fin_same else 'differ'})")
                    continue
            calls = []
            beta, rv, dn = lp.run_chain(ctx, st, seed, fitter=fixed_fitter(pfit, P, calls))
            if sorted(calls) != ["f", "r"]:
                _log(f"[eval {pool_name}] {k}: EXCLUDED -- fitter calls {calls}")
                continue
            out["rec"] = rv
            np.savez_compressed(os.path.join(OUT_DIR, "eval", f"{pool_name}_{k}.npz"), beta=beta.astype(np.float32),
                                cs_f=np.asarray(dn["cs_f"], np.float32), cs_r=np.asarray(dn["cs_r"], np.float32))
            with open(os.path.join(OUT_DIR, "eval", f"{pool_name}_{k}.json"), "w") as fh:
                json.dump(out, fh, indent=1, default=lp._jsonable)
            _log(f"[eval {pool_name}] {k}: gate {rv['gate']} {rv['health']}, fallback {rv['fallback']}, status "
                 f"{rv['status']}; {time.perf_counter() - t0:.0f} s")
        except Exception as e:
            _log(f"[eval {pool_name}] {k}: EXCLUDED -- exception {type(e).__name__}: {e}")


# ---------------------------------------------------------------- report

def member_metrics(k, pool_name, pfit):
    ctx = lp.Ctx(k)
    st = ctx.state
    rec = d2.load_fit(k)
    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    inc = ctx.in_br & on_lap
    ev = np.load(os.path.join(OUT_DIR, "eval", f"{pool_name}_{k}.npz"))
    evj = json.load(open(os.path.join(OUT_DIR, "eval", f"{pool_name}_{k}.json")))
    man = rec["prod"]["manifest"]
    out = {"key": k, "tier1": k in TIER1, "P": {}, "V": {}, "axes": {}}
    gp = rec["prod"]["gate"] or {}
    out["P"]["G"] = bool(man.get("status") != "degenerate" and gp.get("verdict") != "fail" and not rec["prod"]["fallback"])
    rv = evj["rec"]
    out["V"]["G"] = bool(rv["status"] != "degenerate" and rv["gate"] != "fail" and not rv["fallback"])
    out["V"]["gate"] = [rv["gate"], rv["health"]]
    out["P"]["gate"] = [gp.get("verdict"), gp.get("health_score")]
    for ax in "fr":
        cp = np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)
        cv = np.asarray(ev[f"cs_{ax}"], float)
        a = {}
        for tag, cs in (("P", cp), ("V", cv)):
            fin = np.isfinite(cs) & inc
            a[tag] = {"cs_neg": float(np.mean(cs[fin] < 0)), "cs_deep": float(np.mean(cs[fin] < -1)),
                      "n_neg": int(np.sum(cs[fin] < 0)), "n_deep": int(np.sum(cs[fin] < -1)), "n_fin": int(fin.sum()),
                      "coverage": float(fin.sum() / max(1, inc.sum()))}
        own = man["axles"][AX[ax]]
        al = np.asarray(rec[f"alpha_{ax}"], float)
        fy = np.asarray(rec[f"fy_{ax}"], float)
        m = rec["fit_mask"] & np.isfinite(al) & np.isfinite(fy)
        a["F2_own_N"] = float(np.sqrt(np.mean((fy[m] - pacejka_lateral_force(al[m], own["B"], own["C"], own["D"], own["E"])) ** 2)))
        p = pfit[ax]
        a["F2_pool_N"] = float(np.sqrt(np.mean((fy[m] - pacejka_lateral_force(al[m], p["B"], p["C"], p["D"], p["E"])) ** 2)))
        a["own_fit"] = {kk: float(own[kk]) for kk in "BCDE"}
        a["own_origin_slope"] = float(own["B"] * own["C"] * own["D"])
        a["own_screen_ok"] = screen_ok(own)
        out["axes"][ax] = a
    if k in TIER1:
        bp = np.asarray(rec["prod"]["beta"], float)
        bv = np.asarray(ev["beta"], float)
        m3p, band = ma.m3_band(bp, ctx)
        m3v = float(lp.m3_pair(bv, ctx)[0])
        straight = ctx.base & (np.abs(st["ax_mps2"]) < ma.STR_G * G) & (np.abs(st["ay_mps2"]) < ma.STR_G * G) & \
            (st["v_mps"] * 3.6 > ma.STR_V_KPH)
        out["P"].update({"M3": m3p, "band": band, "B2_deg": float(np.median(np.abs(bp[straight])) * RAD2DEG),
                         **lp.chatter(bp, ctx.base, st["sample_rate_hz"])})
        out["V"].update({"M3": m3v, "B2_deg": float(np.median(np.abs(bv[straight])) * RAD2DEG),
                         **lp.chatter(bv, ctx.base, st["sample_rate_hz"])})
    return out


def report():
    pools = load_pool()
    lines, verdicts = [], {}
    for name in POOLS:
        pf = pools[name]
        pfit = pf["axes"]
        evald = [k for k in pf["members"] if os.path.exists(os.path.join(OUT_DIR, "eval", f"{name}_{k}.npz"))]
        rows = [member_metrics(k, name, pfit) for k in evald]
        harm, cand = [], {}
        for ax in "fr":
            if not pfit[ax]["screen_ok"]:
                harm.append(f"{AX[ax]} pooled curve fails the sanity screen")
        for r in rows:
            if not r["V"]["G"]:
                harm.append(f"{r['key']} G fails ({r['V']['gate']})")
            for ax in "fr":
                p_, v_ = r["axes"][ax]["P"], r["axes"][ax]["V"]
                if (v_["cs_neg"] - p_["cs_neg"]) * 100 >= HARM_CS_NEG_PT:
                    harm.append(f"{r['key']} {AX[ax]} CS<0 +{(v_['cs_neg'] - p_['cs_neg']) * 100:.2f} pt")
                if (p_["coverage"] - v_["coverage"]) * 100 >= HARM_COV_PT:
                    harm.append(f"{r['key']} {AX[ax]} coverage -{(p_['coverage'] - v_['coverage']) * 100:.2f} pt")
            if r["tier1"]:
                if r["V"]["M3"] > r["P"]["band"][1]:
                    harm.append(f"{r['key']} M3 {r['V']['M3']:.3f} above P's band {r['P']['band'][1]:.3f}")
                if r["V"]["B2_deg"] - r["P"]["B2_deg"] >= HARM_B2_DEG:
                    harm.append(f"{r['key']} straight |beta| +{r['V']['B2_deg'] - r['P']['B2_deg']:.3f} deg")
        t1 = [r for r in rows if r["tier1"]]
        if len(t1) >= 2:
            credit = sum(r["V"]["M3"] < r["P"]["band"][0] and r["V"]["M3"] <= 0.9 * r["P"]["M3"] for r in t1)
            cand["(2) M3 credit on >= 2 Tier-1 members"] = (credit, credit >= 2)

        def pooled(tag, field):
            return sum(r["axes"][ax][tag][field] for r in rows for ax in "fr")
        deep = (pooled("V", "n_deep") / pooled("V", "n_fin")) / (pooled("P", "n_deep") / pooled("P", "n_fin")) - 1
        neg = (pooled("V", "n_neg") / pooled("V", "n_fin")) / (pooled("P", "n_neg") / pooled("P", "n_fin")) - 1
        cand["(3) pooled CS<-1 -10 % rel"] = (deep, deep <= -0.10)
        cand["(5) pooled CS<0 -5 % rel"] = (neg, neg <= -0.05)
        met = sum(1 for v in cand.values() if v[1])
        verdict = "HARM" if harm else ("EARNS" if met >= 2 else "NEUTRAL")
        verdicts[name] = {"verdict": verdict, "harm": harm, "candidates": cand, "met": met, "reachable": len(cand),
                          "evaluated": evald, "excluded": pf["excluded"] + [k for k in pf["members"] if k not in evald]}
        lines.append(f"== Pool {name}: {POOL_NAME[name]}")
        lines.append(f"  members evaluated {len(evald)} of {len(POOLS[name])}; excluded {verdicts[name]['excluded'] or 'none'}")
        for ax in "fr":
            p = pfit[ax]
            lines.append(f"  pooled {AX[ax]}: B {p['B']:.3f} C {p['C']:.3f} D {p['D']:.0f} E {p['E']:.2f}, origin slope "
                         f"{p['origin_slope_N_per_rad'] / 1000:.1f} kN/rad, peak {p['peak_alpha_deg']:.2f} deg, n {p['fit_n_samples']}, "
                         f"screen {'ok' if p['screen_ok'] else 'FAIL'}, leave-one-member-out spread "
                         f"{p.get('lomo_spread_median_rel', float('nan')) * 100:.2f} %")
        for r in rows:
            s = f"  {r['key']}{' (Tier 1)' if r['tier1'] else ''}: gate P {r['P']['gate'][0]} {r['P']['gate'][1]:.3f} -> pooled {r['V']['gate'][0]} {r['V']['gate'][1]:.3f}"
            for ax in "fr":
                a = r["axes"][ax]
                s += (f"; {AX[ax]} CS<0 {a['P']['cs_neg'] * 100:.2f} -> {a['V']['cs_neg'] * 100:.2f} %, CS<-1 {a['P']['n_deep']} -> "
                      f"{a['V']['n_deep']}, coverage {a['P']['coverage'] * 100:.1f} -> {a['V']['coverage'] * 100:.1f} %, F2 own "
                      f"{a['F2_own_N']:.0f} / pooled {a['F2_pool_N']:.0f} N, own slope {a['own_origin_slope'] / 1000:.1f} kN/rad")
            if r["tier1"]:
                s += (f"; M3 {r['P']['M3']:.3f} [{r['P']['band'][0]:.3f}, {r['P']['band'][1]:.3f}] -> {r['V']['M3']:.3f}; "
                      f"straight |beta| {r['P']['B2_deg']:.3f} -> {r['V']['B2_deg']:.3f} deg; chatter "
                      f"{r['P']['chatter_rms_deg']:.3f} -> {r['V']['chatter_rms_deg']:.3f} deg")
            lines.append(s)
        lines.append(f"  VERDICT: {verdict} -- candidates met {met} of {len(cand)} reachable")
        lines.append(f"  HARM items: {harm or 'none'}")
        lines.append("  candidates: " + "; ".join(f"{k}: {v[0]:+.3f} -> {'yes' if v[1] else 'no'}" if isinstance(v[0], float)
                                                  else f"{k}: {v[0]} -> {'yes' if v[1] else 'no'}" for k, v in cand.items()))
        verdicts[name]["rows"] = rows
    txt = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(verdicts, fh, indent=1, default=lp._jsonable)
    print(txt)


# ---------------------------------------------------------------- plots (C+)

C_POOL, C_MEM, C_CLOUD = "#2a78d6", "#888888", "#b7d3f6"
C_F, C_R = "#2a78d6", "#eb6834"


def plots():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pools = load_pool()
    rng = np.random.default_rng(0)
    # 1. pooled curve over member curves, fit population as faint cloud
    for name in POOLS:
        pf = pools[name]
        recs = {k: d2.load_fit(k) for k in pf["members"]}
        for ax in "fr":
            fig, axp = plt.subplots(figsize=(9, 6))
            a = np.concatenate([np.degrees(np.asarray(recs[k][f"alpha_{ax}"], float))[recs[k]["fit_mask"]] for k in recs])
            f = np.concatenate([np.asarray(recs[k][f"fy_{ax}"], float)[recs[k]["fit_mask"]] for k in recs])
            ok = np.isfinite(a) & np.isfinite(f)
            idx = rng.choice(np.flatnonzero(ok), size=min(40000, ok.sum()), replace=False)
            axp.scatter(a[idx], f[idx] / 1000, s=1, color=C_CLOUD, alpha=0.25, rasterized=True,
                        label="fit population of all members (subsample)")
            lim = np.percentile(np.abs(a[ok]), 99.5)
            grid = np.linspace(-lim, lim, 400)
            for i, k in enumerate(recs):
                o = recs[k]["prod"]["manifest"]["axles"][AX[ax]]
                axp.plot(grid, pacejka_lateral_force(np.radians(grid), o["B"], o["C"], o["D"], o["E"]) / 1000,
                         color=C_MEM, lw=1, alpha=0.8, label="member's own production curve" if i == 0 else None)
            p = pf["axes"][ax]
            axp.plot(grid, pacejka_lateral_force(np.radians(grid), p["B"], p["C"], p["D"], p["E"]) / 1000,
                     color=C_POOL, lw=3, label="pooled curve")
            axp.set_xlabel("kinematic-seed slip angle [deg]")
            axp.set_ylabel("axle lateral force [kN]")
            axp.set_title(f"{POOL_NAME[name]}\n{AX[ax]} axle tyre curve: pooled curve vs each member's own "
                          "production fit", fontsize=11)
            axp.grid(alpha=0.25)
            axp.legend(loc="lower right", fontsize=9, markerscale=8)
            fig.tight_layout()
            fig.savefig(os.path.join(OUT_DIR, f"curves_pool{name}_{AX[ax]}.png"), dpi=120)
            plt.close(fig)
    cs_plots()
    print("plots done", flush=True)


def near_strike_marks(ctx, rec):
    """The d2 cache stores CS but not the near_strike grade mark, so the
    production CS is recomputed from the cached beta; the recompute must
    reproduce the cached CS (same finite sets, float32 resolution) before the
    mark is used."""
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness)
    st, prm = ctx.state, ctx.params
    beta = np.asarray(rec["prod"]["beta"], float)
    cs = estimate_cornering_stiffness(estimate_slip_angles(st, beta, prm), estimate_lateral_forces(st, prm), st, prm)
    for ax in "fr":
        a = np.asarray(cs[f"CS_ratio_{ax}"], float)
        b = np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)
        fin = np.isfinite(a) & np.isfinite(b)
        diff = float(np.max(np.abs(a[fin] - b[fin]))) if fin.any() else 0.0
        if not (np.array_equal(np.isfinite(a), np.isfinite(b)) and diff <= 1e-5):
            raise SystemExit(f"STOP {ctx.key} {AX[ax]}: recomputed CS does not reproduce the cache (max diff {diff:.2e})")
    return {ax: np.asarray(cs[f"near_strike_{ax}"], bool) for ax in "fr"}


def cs_plots(mode=None):
    """Tier-1 production CS for the thesis: full median lap and in-corner
    histograms, near-strike samples drawn per corner_trace_display.
    near_strike_display (kerb display package, 2026-10-05)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from core.plot_style import NEAR_STRIKE_COLOR
    from core.trace_display import near_strike_split, near_strike_caption
    from modules.stability_analysis import load_parameters
    prm = load_parameters()
    mode = mode or prm["corner_trace_display"]["near_strike_display"]
    grade_s = prm["stability_estimation"]["kerb_smear_grade_s"]
    grey_label = "near a kerb strike (kept in all results)"
    for k in TIER1:
        ctx = lp.Ctx(k)
        st = ctx.state
        rec = d2.load_fit(k)
        marks = near_strike_marks(ctx, rec)
        laps = ctx.analysis_laps()
        lt = np.array([l["end_time"] - l["start_time"] for l in laps])
        lap = laps[int(np.argsort(lt)[len(lt) // 2])]
        m = (st["time"] >= lap["start_time"]) & (st["time"] <= lap["end_time"])
        s = st["s_m"][m]
        fig, axs = plt.subplots(2, 1, figsize=(15, 7.6), sharex=True)
        n_mark = n_fin = 0
        for i, (ax, col) in enumerate((("f", C_F), ("r", C_R))):
            cs = np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)[m]
            main, overlay = near_strike_split(cs, marks[ax][m], mode)
            fin = np.isfinite(cs)
            n_fin += int(fin.sum())
            n_mark += int((marks[ax][m] & fin).sum())
            axs[i].plot(s, main, color=col, lw=1.2, label=f"{AX[ax]} axle")
            if overlay is not None:
                axs[i].plot(s, overlay, color=NEAR_STRIKE_COLOR, lw=1.2, label=grey_label)
            edges = np.flatnonzero(np.diff(np.r_[0, st["kerb_mask"][m].astype(int), 0]))
            for j, (a0, b0) in enumerate(zip(edges[::2], edges[1::2])):
                axs[i].axvspan(s[a0], s[min(b0, len(s) - 1)], color=NEAR_STRIKE_COLOR, alpha=0.18, lw=0,
                               label="kerb strike, masked (no estimate)" if j == 0 else None)
            axs[i].axhline(0, color="#444444", lw=0.8)
            axs[i].set_ylabel(f"cornering stiffness ratio, {AX[ax]}")
            axs[i].set_ylim(-3, 1.1)
            axs[i].grid(alpha=0.25)
            axs[i].legend(loc="lower right", fontsize=8)
        axs[1].set_xlabel("lap distance [m]")
        fig.suptitle(f"{lp.title_of(k)}: cornering stiffness ratio over the median-time lap (lap {lap['lap_number']}); "
                     f"1 = linear tyre range, 0 = force peak, below 0 = past the peak", fontsize=11)
        cap = near_strike_caption(mode, n_mark, n_fin, grade_s)
        fig.tight_layout(rect=(0, 0.04 if cap else 0, 1, 1))
        if cap:
            fig.text(0.5, 0.01, cap, ha="center", va="bottom", fontsize=9, wrap=True)
        fig.savefig(os.path.join(OUT_DIR, f"cs_lap_{k}.png"), dpi=120)
        plt.close(fig)
        on_lap = ctx.lap_mask({l["lap_number"] for l in laps})
        inc = ctx.in_br & on_lap
        fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))
        bins = np.linspace(-3, 1.05, 82)
        n_mark = n_fin = 0
        for i, (ax, col) in enumerate((("f", C_F), ("r", C_R))):
            cs = np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)
            sel = np.isfinite(cs) & inc
            mk = marks[ax] & sel
            clean, marked = np.clip(cs[sel & ~mk], bins[0], bins[-1]), np.clip(cs[mk], bins[0], bins[-1])
            n_fin += int(sel.sum())
            n_mark += int(mk.sum())
            if mode == "grey":
                axs[i].hist([clean, marked], bins=bins, stacked=True, color=[col, NEAR_STRIKE_COLOR], alpha=0.9,
                            label=["clean samples", grey_label])
                axs[i].legend(loc="upper left", fontsize=8)
            elif mode == "exclude":
                axs[i].hist(clean, bins=bins, color=col, alpha=0.9)
            else:
                axs[i].hist(np.clip(cs[sel], bins[0], bins[-1]), bins=bins, color=col, alpha=0.9)
            v = cs[sel]
            axs[i].set_yscale("log")
            axs[i].axvline(0, color="#444444", lw=0.8)
            axs[i].set_xlabel(f"cornering stiffness ratio in corners, {AX[ax]} (below -3 piled at -3)")
            axs[i].set_ylabel("samples (log scale)")
            axs[i].set_title(f"{AX[ax]}: below 0 {np.mean(v < 0) * 100:.1f} %, below -1 {np.mean(v < -1) * 100:.2f} %; "
                             f"near a kerb strike {mk.sum() / max(1, sel.sum()) * 100:.1f} % (n {v.size})", fontsize=10)
            axs[i].grid(alpha=0.25)
        fig.suptitle(f"{lp.title_of(k)}: distribution of the cornering stiffness ratio in corners, current production")
        cap = near_strike_caption(mode, n_mark, n_fin, grade_s)
        fig.tight_layout(rect=(0, 0.05 if cap else 0, 1, 1))
        if cap:
            fig.text(0.5, 0.01, cap, ha="center", va="bottom", fontsize=9, wrap=True)
        fig.savefig(os.path.join(OUT_DIR, f"cs_hist_{k}.png"), dpi=120)
        plt.close(fig)
    print("cs plots done", flush=True)


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    mode = sys.argv[1]
    if mode == "rebuild":
        rebuild(sys.argv[2:])
    elif mode == "pool":
        pool()
    elif mode == "eval":
        evaluate(sys.argv[2], sys.argv[3:])
    elif mode == "report":
        report()
    elif mode == "plots":
        plots()
    elif mode == "csplots":
        cs_plots(sys.argv[2] if len(sys.argv) > 2 else None)
