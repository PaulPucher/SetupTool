# STEP 4 ITERATION EXPERIMENT (author EXECUTE 2026-10-05). Harness only: no
# production or config change. Production state = corrected steering
# (schema 10) + speed window (schema 11).
#
# INTENT (author): does refitting the tyre curve on refined slip angles
# improve the per-session chain, and does the pooled curve as a STARTING
# POINT (the author's intended prior architecture) beat the kinematic start
# -- for use as the production fitting procedure if a branch earns.
#
# LITERATURE (verified sections, thesis_notes "Literature check before the
# iteration experiment"):
#   - The scheme belongs to the dual-EKF / two-stage family (parameters
#     estimated from filter states and fed back), as listed by Villano, Lenzo
#     & Sakhnevych (2021), Meccanica 56, sec. 1; their own cross-combination
#     (sec. 3.3) exchanges states with fixed tyre parameters, so it is not this
#     scheme.
#   - Convergence is judged on the parameter spread settling: Bechtloff
#     (2018), VDI 12/809, sec. 6.3 ("the standard deviations of the cornering
#     stiffnesses converge only after about 160 s").
#   - Divergence warning, the justification of the amplification exit:
#     Bechtloff (2018) sec. 1.2 -- stiffness adaptation from estimated axle
#     forces "works well for dynamic manoeuvres; for stationary manoeuvres the
#     estimation of the sideslip angle, and with it of the cornering
#     stiffnesses, can diverge"; Sierra, Tseng, Jain & Peng (2006), VSD 44
#     sup1, sec. 3 -- under-determined stiffness estimates need persistently
#     exciting information "for the estimation results to converge to the
#     true values". No work on disk states verbatim that reference-free
#     iteration amplifies bias; these are the closest warnings.
#   - Start dependence of Magic Formula regression: Ortiz, Cabrera, Guerra &
#     Simon (2006), VSD 44:9, sec. 1 -- why two starting points are tested.
#
# RULES, frozen 2026-10-05 before any round (conflict = record, no re-tuning).
# SESSIONS: Tier 1 -- pr_us_v2, v3, nbr_jst_solution.
# BRANCHES per session:
#   own start  -- round 0 = the session's production fit and chain (cache).
#   pool start -- round 0 = the pool curve (Pool T for Paul Ricard, Pool D for
#                 JST 29) and its chain from Step 3 (plots/pool_fits/eval/).
# ROUND r (r = 1..3): slip angles = production estimate_slip_angles(state,
#   beta_(r-1)), 2 Hz filtered; F_r = production _fit_axle_pacejka on those
#   slip angles, the measured axle Fy and the session's production fit mask
#   (base & speed window); chain = inspect_last_push.run_chain with F_r
#   behind the fitter hook (R derivation from the residuals of the arrays
#   production passes, as in the Step 3 pooled evaluation).
# STEP D_r(axle) = median over |alpha| in [0.5, 8] deg (25 points) of
#   |F_r - F_(r-1)| / |F_(r-1)| -- curve shape, not single Magic Formula
#   parameters (weakly identifiable, Ortiz).
# NOISE N(axle) = the session's lap-to-lap fit noise: leave-one-analysis-lap-
#   out refits of the production fit (seed slip angles, production fit mask),
#   median over the same grid of std / |median curve|.
# STOPS per branch, checked each round in this order:
#   1. sanity screen (fit-shape rule, not physics: finite, B >= 1,
#      1 < C <= 3, E <= 1) or Powell not converged on either axle -> stop,
#      final = round r-1;
#   2. chain G fails (degenerate, gate fail or fallback) -> stop, final = r-1;
#   3. AMPLIFICATION EXIT: r = 2 and D_2 > D_1 on either axle -> stop, report
#      "iteration amplifies seed bias", final = round 1;
#   4. CONVERGENCE: D_r <= N on both axles -> stop, final = round r;
#   5. r = 3 reached -> stop, final = round 3 (not converged).
# PRE-REGISTERED PREDICTION (circularity: a refit on slip angles produced with
#   a curve tends to return that curve): the own-start branch shows
#   D_1 <= N on both axles (returns to its own curve at round 1).
# JUDGING: each branch's final round vs production P on the Q-FIT-DECISIVE
#   frame verbatim -- HARM: G; screen; M3 above P's band (any session); CS < 0
#   +2.5 pt or coverage -1 pt (any cell); straight |beta| +0.1 deg; S > 1.5 x P
#   and +0.01. EARNS = no harm and >= 2 of: F1 median over 6 cells -10 %; M3
#   credit (below band and <= 0.9 x P) on >= 2 of 3; pooled CS < -1 -10 % rel;
#   S median -25 %; pooled CS < 0 -5 % rel. NEUTRAL otherwise. F1 / S on the
#   final round's own fit population (leave-one-lap-out refits; refit without
#   the smear zone), grids as in the decisive frame (P's |alpha| p10-p90 on the
#   base mask, 25 points). Beta metrics (M3, straight |beta|) arbitrate.
#   A final round of 0 (no round accepted) = production itself for own
#   start, = the pool-curve chain for pool start.
# POOL vs OWN START: SAME PLACE = final curves differ by D <= N on both axles
#   in all three sessions; POOL START BETTER = pool start EARNS and own start
#   does not, or both EARN with pool start meeting more candidates; otherwise
#   A DIFFERENT PLACE, NOT BETTER (per session the distance is reported).
#
# Modes: run <key> | report | plots
# Output: diagnostics/plots/iteration/

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
from diagnostics import inspect_pool_fits as pf
from modules.tyre_model_pacejka import pacejka_lateral_force

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "iteration")
TIER1 = lp.TIER1
POOL_OF = {"pr_us_v2": "T", "v3": "T", "nbr_jst_solution": "D"}
AX = {"f": "front", "r": "rear"}
BRANCH_NAME = {"own": "starting from the session's own fit", "pool": "starting from the pooled curve"}
MAX_ROUNDS = 3
STEP_GRID = np.radians(np.linspace(0.5, 8.0, 25))
HARM_CS_NEG_PT = 2.5
ORIG_FIT = tfa._fit_axle_pacejka
RAD2DEG = 180.0 / np.pi
G = 9.81


def _log(msg):
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, "run_log.txt"), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def curve(fit, grid):
    return np.abs(pacejka_lateral_force(grid, fit["B"], fit["C"], fit["D"], fit["E"]))


def step(f_new, f_old, grid=STEP_GRID):
    c0 = curve(f_old, grid)
    return float(np.median(np.abs(curve(f_new, grid) - c0) / c0))


def screen_ok(fit):
    return pf.screen_ok(fit)


def loo_noise(ctx, alpha, fy, mask, grid):
    curves = []
    for lap in ctx.analysis_laps():
        fl = ORIG_FIT(alpha, fy, mask & ~ctx.lap_mask({lap["lap_number"]}))
        if fl["powell_converged"] and fl["sign_ok"]:
            curves.append(curve(fl, grid))
    if len(curves) < 2:
        return float("nan")
    c = np.array(curves)
    return float(np.median(np.std(c, axis=0, ddof=1) / np.abs(np.median(c, axis=0))))


def g_ok(rv):
    return bool(rv["status"] != "degenerate" and rv["gate"] != "fail" and not rv["fallback"])


def run(key):
    from modules.stability_analysis import estimate_slip_angles
    os.makedirs(os.path.join(OUT_DIR, "rounds"), exist_ok=True)
    ctx = lp.Ctx(key)
    st, prm = ctx.state, ctx.params
    seed, P, _M = ma.inputs(ctx)
    rec = d2.load_fit(key)
    assert np.array_equal(ctx.fit_mask, rec["fit_mask"]), "fit mask differs from the production cache"
    fm = ctx.fit_mask
    fy = {ax: P[ax][1] for ax in "fr"}
    noise = {ax: loo_noise(ctx, P[ax][0], fy[ax], fm, STEP_GRID) for ax in "fr"}
    _log(f"[{key}] lap-to-lap fit noise N: front {noise['f'] * 100:.3f} %, rear {noise['r'] * 100:.3f} %")
    pool = pf.load_pool()[POOL_OF[key]]["axes"]
    starts = {
        "own": ({ax: {k: rec["prod"]["manifest"]["axles"][AX[ax]][k] for k in "BCDE"} for ax in "fr"},
                np.asarray(rec["prod"]["beta"], float)),
        "pool": ({ax: {k: pool[ax][k] for k in "BCDE"} for ax in "fr"},
                 np.asarray(np.load(os.path.join(pf.OUT_DIR, "eval", f"{POOL_OF[key]}_{key}.npz"))["beta"], float)),
    }
    out = {"key": key, "noise": noise, "branches": {}}
    for br, (f0, beta0) in starts.items():
        fits, betas, steps, stop, final = [f0], [beta0], [], None, 0
        for r in range(1, MAX_ROUNDS + 1):
            t0 = time.perf_counter()
            slip = estimate_slip_angles(st, betas[-1], prm)
            fr_ = {}
            for ax in "fr":
                full = ORIG_FIT(slip[f"alpha_{ax}_filt"], fy[ax], fm)
                fr_[ax] = {k: float(full[k]) for k in "BCDE"}
                fr_[ax].update(powell_converged=bool(full["powell_converged"]), sign_ok=bool(full["sign_ok"]),
                               peak_alpha_deg=full["peak_alpha_deg"])
            d = {ax: step(fr_[ax], fits[-1][ax]) for ax in "fr"}
            if not all(screen_ok(fr_[ax]) and fr_[ax]["powell_converged"] for ax in "fr"):
                stop = f"round {r}: sanity screen / Powell failed ({ {ax: {k: round(fr_[ax][k], 3) for k in 'BCDE'} for ax in 'fr'} })"
                steps.append(d)
                break
            calls = []
            beta, rv, dn = lp.run_chain(ctx, st, seed, fitter=pf.fixed_fitter(
                {ax: {**fr_[ax], "peak_alpha_deg": fr_[ax]["peak_alpha_deg"]} for ax in "fr"}, P, calls))
            if sorted(calls) != ["f", "r"]:
                raise SystemExit(f"STOP {key} {br} round {r}: fitter calls {calls}")
            steps.append(d)
            np.savez_compressed(os.path.join(OUT_DIR, "rounds", f"{key}_{br}_r{r}.npz"), beta=beta.astype(np.float32),
                                cs_f=np.asarray(dn["cs_f"], np.float32), cs_r=np.asarray(dn["cs_r"], np.float32),
                                alpha_f=slip["alpha_f_filt"].astype(np.float32), alpha_r=slip["alpha_r_filt"].astype(np.float32))
            _log(f"[{key}] {br} round {r}: step front {d['f'] * 100:.3f} % rear {d['r'] * 100:.3f} %; front B "
                 f"{fr_['f']['B']:.3f} C {fr_['f']['C']:.3f} D {fr_['f']['D']:.0f} E {fr_['f']['E']:.2f}; rear B "
                 f"{fr_['r']['B']:.3f} C {fr_['r']['C']:.3f} D {fr_['r']['D']:.0f} E {fr_['r']['E']:.2f}; gate {rv['gate']} "
                 f"{rv['health']}, fallback {rv['fallback']}; {time.perf_counter() - t0:.0f} s")
            if not g_ok(rv):
                stop = f"round {r}: chain G failed (gate {rv['gate']}, status {rv['status']}, fallback {rv['fallback']})"
                break
            fits.append(fr_)
            betas.append(beta)
            final = r
            if r == 2 and any(steps[1][ax] > steps[0][ax] for ax in "fr"):
                stop = "round 2: AMPLIFICATION EXIT (step 2 > step 1) -- iteration amplifies seed bias"
                final = 1
                break
            if all(d[ax] <= noise[ax] for ax in "fr"):
                stop = f"round {r}: CONVERGED (step below the lap-to-lap fit noise on both axles)"
                break
        if stop is None:
            stop = f"round {MAX_ROUNDS} reached, not converged"
        out["branches"][br] = {"fits": fits, "steps": steps, "stop": stop, "final_round": final}
        _log(f"[{key}] {br}: STOP -- {stop}; final round {final}")
    with open(os.path.join(OUT_DIR, f"rounds_{key}.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=lp._jsonable)


# ---------------------------------------------------------------- report

def frame_metrics(ctx, P, alpha, fit, beta, cs, grids, base_noz):
    st = ctx.state
    fm = ctx.fit_mask
    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    inc = ctx.in_br & on_lap
    straight = ctx.base & (np.abs(st["ax_mps2"]) < ma.STR_G * G) & (np.abs(st["ay_mps2"]) < ma.STR_G * G) & \
        (st["v_mps"] * 3.6 > ma.STR_V_KPH)
    m = {"M3": float(lp.m3_pair(beta, ctx)[0]), "B2_deg": float(np.median(np.abs(beta[straight])) * RAD2DEG),
         **lp.chatter(beta, ctx.base, st["sample_rate_hz"]), "axes": {}}
    for ax in "fr":
        a, f = alpha[ax], P[ax][1]
        full = ORIG_FIT(a, f, fm)
        noz = ORIG_FIT(a, f, fm & base_noz)
        c = np.asarray(cs[ax], float)
        fin = np.isfinite(c) & inc
        m["axes"][ax] = {"F1": loo_noise(ctx, a, f, fm, grids[ax]),
                         "S": float(np.median(np.abs(curve(noz, grids[ax]) - curve(full, grids[ax])) / curve(full, grids[ax]))),
                         "cs_neg": float(np.mean(c[fin] < 0)), "cs_deep": float(np.mean(c[fin] < -1)),
                         "n_neg": int(np.sum(c[fin] < 0)), "n_deep": int(np.sum(c[fin] < -1)), "n_fin": int(fin.sum()),
                         "coverage": float(fin.sum() / max(1, inc.sum())), "screen_ok": screen_ok(fit[ax]),
                         "origin_slope": float(fit[ax]["B"] * fit[ax]["C"] * fit[ax]["D"])}
    return m


def verdict(rows):
    harm = []
    for r in rows:
        p, v = r["P"], r["V"]
        if not r["G"]:
            harm.append(f"{r['key']} G")
        for ax in "fr":
            if not v["axes"][ax]["screen_ok"]:
                harm.append(f"{r['key']} {AX[ax]} sanity screen")
        if v["M3"] > r["band"][1]:
            harm.append(f"{r['key']} M3 above band ({v['M3']:.3f} > {r['band'][1]:.3f})")
        if v["B2_deg"] - p["B2_deg"] >= 0.1:
            harm.append(f"{r['key']} straight beta +{v['B2_deg'] - p['B2_deg']:.3f} deg")
        for ax in "fr":
            pa, va = p["axes"][ax], v["axes"][ax]
            if (va["cs_neg"] - pa["cs_neg"]) * 100 >= HARM_CS_NEG_PT:
                harm.append(f"{r['key']} {AX[ax]} CS<0 +{(va['cs_neg'] - pa['cs_neg']) * 100:.2f} pt")
            if (pa["coverage"] - va["coverage"]) * 100 >= 1.0:
                harm.append(f"{r['key']} {AX[ax]} coverage -{(pa['coverage'] - va['coverage']) * 100:.2f} pt")
            if va["S"] > 1.5 * pa["S"] and va["S"] - pa["S"] > 0.01:
                harm.append(f"{r['key']} {AX[ax]} S {pa['S']:.4f} -> {va['S']:.4f}")
    cells = [(r["P"]["axes"][ax], r["V"]["axes"][ax]) for r in rows for ax in "fr"]
    f1 = float(np.nanmedian([v["F1"] / p["F1"] - 1 for p, v in cells]))
    sm = float(np.nanmedian([v["S"] / p["S"] - 1 for p, v in cells]))
    credit = [bool(r["V"]["M3"] < r["band"][0] and r["V"]["M3"] <= 0.9 * r["P"]["M3"]) for r in rows]

    def pooled(t, k):
        return sum(r[t]["axes"][ax][k] for r in rows for ax in "fr")
    deep = pooled("V", "n_deep") / pooled("V", "n_fin") / (pooled("P", "n_deep") / pooled("P", "n_fin")) - 1
    neg = pooled("V", "n_neg") / pooled("V", "n_fin") / (pooled("P", "n_neg") / pooled("P", "n_fin")) - 1
    cand = {"F1 -10 %": (f1, f1 <= -0.10), "M3 credit on >= 2 of 3": (sum(credit), sum(credit) >= 2),
            "CS<-1 -10 % rel": (deep, deep <= -0.10), "S -25 %": (sm, sm <= -0.25), "CS<0 -5 % rel": (neg, neg <= -0.05)}
    met = sum(bool(v[1]) for v in cand.values())
    return {"verdict": "HARM" if harm else ("EARNS" if met >= 2 else "NEUTRAL"), "harm": harm,
            "candidates": {k: [v[0], bool(v[1])] for k, v in cand.items()}, "met": met}


def report():
    from modules.stability_analysis import estimate_slip_angles
    res, lines = {}, []
    branch_rows = {"own": [], "pool": []}
    for key in TIER1:
        R = json.load(open(os.path.join(OUT_DIR, f"rounds_{key}.json")))
        ctx = lp.Ctx(key)
        st = ctx.state
        seed, P, _M = ma.inputs(ctx)
        rec = d2.load_fit(key)
        bp = np.asarray(rec["prod"]["beta"], float)
        m3p, band = ma.m3_band(bp, ctx)
        grids = {}
        for ax in "fr":
            a = np.abs(P[ax][0][ctx.base & np.isfinite(P[ax][0])])
            grids[ax] = np.linspace(np.percentile(a, ma.GRID_LO), np.percentile(a, ma.GRID_HI), ma.GRID_N)
        noz = ~st["kerb_smear_zone"]
        own = {ax: {k: rec["prod"]["manifest"]["axles"][AX[ax]][k] for k in "BCDE"} for ax in "fr"}
        Pm = frame_metrics(ctx, P, {ax: P[ax][0] for ax in "fr"}, own, bp,
                           {ax: rec["prod"]["down"][f"cs_{ax}"] for ax in "fr"}, grids, noz)
        Pm["M3"] = m3p
        lines.append(f"== {lp.title_of(key)} ({key}): lap-to-lap fit noise front {R['noise']['f'] * 100:.3f} %, rear "
                     f"{R['noise']['r'] * 100:.3f} %")
        finals = {}
        for br in ("own", "pool"):
            b = R["branches"][br]
            fr_ = b["final_round"]
            fit = b["fits"][fr_]
            if fr_ == 0 and br == "own":
                beta, cs, alpha = bp, {ax: rec["prod"]["down"][f"cs_{ax}"] for ax in "fr"}, {ax: P[ax][0] for ax in "fr"}
                g = True
            else:
                if fr_ == 0:
                    z = np.load(os.path.join(pf.OUT_DIR, "eval", f"{POOL_OF[key]}_{key}.npz"))
                    j = json.load(open(os.path.join(pf.OUT_DIR, "eval", f"{POOL_OF[key]}_{key}.json")))
                    g = g_ok(j["rec"])
                    beta = np.asarray(z["beta"], float)
                    alpha = {ax: P[ax][0] for ax in "fr"}
                else:
                    z = np.load(os.path.join(OUT_DIR, "rounds", f"{key}_{br}_r{fr_}.npz"))
                    g = True
                    beta = np.asarray(z["beta"], float)
                    alpha = {ax: np.asarray(z[f"alpha_{ax}"], float) for ax in "fr"}
                cs = {ax: z[f"cs_{ax}"] for ax in "fr"}
            Vm = frame_metrics(ctx, P, alpha, fit, beta, cs, grids, noz)
            row = {"key": key, "G": g, "P": Pm, "V": Vm, "band": band}
            branch_rows[br].append(row)
            finals[br] = fit
            steps = "; ".join(f"round {i + 1}: front {s['f'] * 100:.3f} % rear {s['r'] * 100:.3f} %" for i, s in enumerate(b["steps"]))
            lines.append(f"  {BRANCH_NAME[br]}: rounds completed {len(b['steps'])}, final round {fr_}; steps {steps or 'none'}")
            lines.append(f"    stop: {b['stop']}")
            for ax in "fr":
                p_, v_ = Pm["axes"][ax], Vm["axes"][ax]
                lines.append(f"    {AX[ax]}: final B {fit[ax]['B']:.3f} C {fit[ax]['C']:.3f} D {fit[ax]['D']:.0f} E {fit[ax]['E']:.2f} "
                             f"(slope {v_['origin_slope'] / 1000:.1f} vs production {p_['origin_slope'] / 1000:.1f} kN/rad); CS<0 "
                             f"{p_['cs_neg'] * 100:.2f} -> {v_['cs_neg'] * 100:.2f} %, CS<-1 {p_['n_deep']} -> {v_['n_deep']}, coverage "
                             f"{p_['coverage'] * 100:.1f} -> {v_['coverage'] * 100:.1f} %, F1 {p_['F1']:.4f} -> {v_['F1']:.4f}, "
                             f"S {p_['S']:.4f} -> {v_['S']:.4f}")
            lines.append(f"    M3 {Pm['M3']:.3f} [{band[0]:.3f}, {band[1]:.3f}] -> {Vm['M3']:.3f}; straight |beta| "
                         f"{Pm['B2_deg']:.3f} -> {Vm['B2_deg']:.3f} deg; chatter {Pm['chatter_rms_deg']:.3f} -> {Vm['chatter_rms_deg']:.3f} deg")
        own_b = R["branches"]["own"]
        if own_b["steps"]:
            s1 = own_b["steps"][0]
            hit = all(s1[ax] <= R["noise"][ax] for ax in "fr")
            lines.append(f"  PREDICTION (own start returns to its own curve at round 1): step front {s1['f'] * 100:.3f} % vs noise "
                         f"{R['noise']['f'] * 100:.3f} %, rear {s1['r'] * 100:.3f} % vs {R['noise']['r'] * 100:.3f} % -> "
                         f"{'HELD' if hit else 'NOT HELD'}")
        dist = {ax: step(finals["pool"][ax], finals["own"][ax]) for ax in "fr"}
        same = all(dist[ax] <= R["noise"][ax] for ax in "fr")
        lines.append(f"  final curves, pool start vs own start: front {dist['f'] * 100:.3f} %, rear {dist['r'] * 100:.3f} % "
                     f"-> {'within' if same else 'beyond'} the fit noise")
        res[key] = {"distance": dist, "same": same}
    verdicts = {br: verdict(branch_rows[br]) for br in ("own", "pool")}
    for br, v in verdicts.items():
        lines.append(f"VERDICT {BRANCH_NAME[br]}: {v['verdict']} -- candidates met {v['met']} of 5; HARM items {v['harm'] or 'none'}")
        lines.append("  candidates: " + "; ".join(f"{k}: {c[0]:+.3f} -> {'yes' if c[1] else 'no'}" if isinstance(c[0], float)
                                                  else f"{k}: {c[0]} -> {'yes' if c[1] else 'no'}" for k, c in v["candidates"].items()))
    all_same = all(res[k]["same"] for k in TIER1)
    vo, vp = verdicts["own"], verdicts["pool"]
    if all_same:
        cmp_ = "SAME PLACE"
    elif (vp["verdict"] == "EARNS" and vo["verdict"] != "EARNS") or \
            (vp["verdict"] == vo["verdict"] == "EARNS" and vp["met"] > vo["met"]):
        cmp_ = "POOL START BETTER"
    else:
        cmp_ = "A DIFFERENT PLACE, NOT BETTER"
    lines.append(f"POOL START vs OWN START: {cmp_}")
    txt = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump({"sessions": res, "verdicts": verdicts, "comparison": cmp_, "rows": branch_rows}, fh, indent=1,
                  default=lp._jsonable)
    print(txt)


# ---------------------------------------------------------------- plots

ROUND_COLORS = ["#888888", "#2a78d6", "#1baf7a", "#eda100"]


def plots():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for key in TIER1:
        R = json.load(open(os.path.join(OUT_DIR, f"rounds_{key}.json")))
        ctx = lp.Ctx(key)
        st = ctx.state
        rec = d2.load_fit(key)
        # 1. curve per round, both starting points, per axle
        for ax in "fr":
            fig, axs = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
            grid = np.linspace(0, 10, 300)
            for i, br in enumerate(("own", "pool")):
                b = R["branches"][br]
                for r, fit in enumerate(b["fits"]):
                    lab = ("starting curve" if r == 0 else f"after refit {r}") + (" (final)" if r == b["final_round"] else "")
                    axs[i].plot(grid, curve(fit[ax], np.radians(grid)) / 1000, color=ROUND_COLORS[r % 4],
                                lw=3 if r == b["final_round"] else 1.5, label=lab)
                axs[i].set_title(BRANCH_NAME[br], fontsize=10)
                axs[i].set_xlabel("slip angle magnitude [deg]")
                axs[i].grid(alpha=0.25)
                axs[i].legend(fontsize=8, loc="lower right")
            axs[0].set_ylabel("axle lateral force [kN]")
            fig.suptitle(f"{lp.title_of(key)}: {AX[ax]} axle tyre curve after each refit on the refined slip angles",
                         fontsize=11)
            fig.tight_layout()
            fig.savefig(os.path.join(OUT_DIR, f"curves_{key}_{AX[ax]}.png"), dpi=120)
            plt.close(fig)
        # 2. sideslip on the median lap: production vs each branch's final round
        laps = ctx.analysis_laps()
        lt = np.array([l["end_time"] - l["start_time"] for l in laps])
        lap = laps[int(np.argsort(lt)[len(lt) // 2])]
        m = (st["time"] >= lap["start_time"]) & (st["time"] <= lap["end_time"])
        s = st["s_m"][m]
        fig, axp = plt.subplots(figsize=(15, 5))
        axp.plot(s, np.asarray(rec["prod"]["beta"], float)[m] * RAD2DEG, color="#444444", lw=1.2,
                 label="production (session's own fit, no refit)")
        for br, col in (("own", "#2a78d6"), ("pool", "#eb6834")):
            b = R["branches"][br]
            fr_ = b["final_round"]
            if fr_ == 0:
                if br == "own":
                    continue
                beta = np.load(os.path.join(pf.OUT_DIR, "eval", f"{POOL_OF[key]}_{key}.npz"))["beta"]
            else:
                beta = np.load(os.path.join(OUT_DIR, "rounds", f"{key}_{br}_r{fr_}.npz"))["beta"]
            axp.plot(s, np.asarray(beta, float)[m] * RAD2DEG, color=col, lw=1.2,
                     label=f"{BRANCH_NAME[br]}, after refit {fr_}" if fr_ else f"{BRANCH_NAME[br]}, no refit accepted")
        axp.axhline(0, color="#888888", lw=0.8)
        axp.set_xlabel("lap distance [m]")
        axp.set_ylabel("vehicle sideslip angle [deg]")
        axp.set_title(f"{lp.title_of(key)}: estimated sideslip angle on the median-time lap (lap {lap['lap_number']})",
                      fontsize=11)
        axp.grid(alpha=0.25)
        axp.legend(fontsize=8, loc="upper right")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"sideslip_{key}.png"), dpi=120)
        plt.close(fig)
    print("plots done", flush=True)


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    mode = sys.argv[1]
    if mode == "run":
        run(sys.argv[2])
    elif mode == "report":
        report()
    elif mode == "plots":
        plots()
