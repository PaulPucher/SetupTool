# RECORD ONLY since 2026-10-04: production now passes the speed-windowed
# fit mask (tyre_fit_auto.fit_speed_window_pct) to the axle fit. This
# frame's fitter hook does NOT check the mask, so a re-run would silently
# fit the windowed population and not reproduce the recorded results;
# re-running as recorded needs fit_speed_window_pct null.
# Q-CAL-3 CALIBRATION PASS -- the chair's 0.15 s moving average on the tyre-fit
# inputs (PLAN.md Q-CAL item 3, author rulings 2026-10-03). Read-only: no
# production or config change; the variant lives only in memory inside the
# existing chain harness (inspect_last_push.run_chain, fitter hook).
# Tier B (fit-input conditioning), anchor: chair performance_analysis tooling
# (internal). ASYMMETRY (recorded): the chair's fit feeds plots, ours feeds
# the EKF observer -- the variant is judged on outcomes, not conformity.
#
# RULES, frozen 2026-10-03 BEFORE any number (author EXECUTE with amendments
# 1a, 1b; any conflict = STOP and ask).
# VARIANTS. P = production: tyre fit on the kinematic-seed slip angle and axle
#   Fy through the 2 Hz Butterworth, base mask, per-session Pacejka free-D
#   (Powell), then EKF Pacejka -> NIS gate -> beta -> slip -> CS. M = chair-
#   exact: the fit gets the centred moving average of the RAW slip angle and
#   raw Fy, window = round(0.15 s x grid rate) (15 samples at 100 Hz),
#   center=True, min_periods=1, over the full series (kerb samples included),
#   base mask selecting afterwards. Everything else identical to P.
# ENTRY. [Session note, stated: the cached production CS predates Q-CAL-1's
#   mask change, so it cannot be the reference.] P through the harness must
#   reproduce a DIRECT production call made now (tyre_fit_auto.resolve_
#   sideslip_beta + the production downstream): beta and CS identical finite
#   sets, max |diff| <= 1e-9; else the session stops. M's fitter must be
#   called exactly once per axle with P's arrays recognised.
# SESSIONS. Tier 1: pr_us_v2, v3, nbr_jst_solution. Tier 2 (16 NBR + 2 PR,
#   no-harm bars only) only after EARNS -- a separate run.
# METRICS (identical for P and M).
#   G gate: status not degenerate, NIS verdict pass, no fallback, Powell
#     converged and sign_ok on both axles.
#   F1 (scored): leave-one-lap-out refits per axle (each analysis lap out in
#     turn); curve at 25 slip angles evenly spaced between p10 and p90 of
#     |alpha| visited on the base mask (P's 2 Hz alpha, same grid for both);
#     per point SD over the refits / |median Fy|; median over the grid.
#   F2 (descriptive only): RMS of (2 Hz Fy - curve(2 Hz alpha)) on the base
#     mask -- the CS chain's view, biased to P by construction.
#   B1 (scored): M3 lap-to-lap beta scatter at matched corner positions
#     (inspect_seed_sweep m3_value). P's two-sided bootstrap band (seed
#     20261002, ss.N_BOOT_M3 draws): [v0 + p2.5 - med, v0 + p97.5 - med].
#   B2 (no-harm only): median |beta| on straights (base, |a_x|, |a_y| < 0.1
#     g, v > 60 km/h), deg.
#   B3 (descriptive): beta chatter RMS above 2 Hz (run_chain's chatter).
#   C: in-corner (corner instances) on analysis laps, per axle: share CS < 0,
#     share CS < -1, finite-CS coverage (finite CS / in-corner samples).
#   S (scored): refit each variant on base & ~kerb_smear_zone (production,
#     0.20 s); shift = median over the grid of |dFy| / |Fy_full|. Also the
#     share of the fit population inside the zone.
# EXITS (Tier 1, three sessions, both axles).
#   HARM (M rejected) if any: G fails; M3_M above P's band (any session);
#     CS < 0 share +0.5 point or more (any session / axle); coverage -1 point
#     or more; straight |beta| +0.1 deg or more; S_M > 1.5 x S_P AND
#     S_M - S_P > 0.01.
#   EARNS (production proposal) = no harm AND at least TWO of FIVE:
#     (1) F1: median over the 6 cells of (F1_M / F1_P - 1) <= -10 %;
#     (2) M3 (amendment 1b tie rule): a session earns credit only if M3_M is
#         below P's band AND M3_M <= 0.9 x M3_P; inside the band = tie (no
#         harm, no credit); the candidate scores when >= 2 of 3 sessions
#         earn credit;
#     (3) CS < -1 share, pooled over sessions and axles, -10 % relative;
#     (4) S: median over the 6 cells of (S_M / S_P - 1) <= -25 %;
#     (5) amendment 1a: CS < 0 share, pooled, -5 % relative.
#   NEUTRAL otherwise -- recorded, production keeps its 2 Hz fit inputs.
# OUTPUTS diagnostics/plots/fit_ma_calibration/: fit_<key>.png (fit cloud and
#   both curves per axle), beta_<key>.png (P vs M on one representative lap),
#   metrics.png (scored metrics, P vs M), summary.txt, results.json.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_fit_ma_calibration
#   [metrics]  re-render metrics.png from results.json only, no recomputation

import copy
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import modules.tyre_fit_auto as tfa
from diagnostics import inspect_last_push as lp
from diagnostics import inspect_seed_sweep as ss
from diagnostics import inspect_tyre_prior_d2 as d2
from modules.tyre_model_pacejka import pacejka_lateral_force

OUT_DIR = "diagnostics/plots/fit_ma_calibration"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
MA_WINDOW_S = 0.15  # chair-exact window (filter_config, fast-dynamics family)
GRID_N, GRID_LO, GRID_HI = 25, 10, 90
STR_G, STR_V_KPH = 0.1, 60.0
ENTRY_TOL = 1e-9
ORIG_FIT = tfa._fit_axle_pacejka
RAD2DEG = 180.0 / np.pi
AX = {"f": "front", "r": "rear"}


def _log(m):
    print(m, flush=True)


def moving_average(x, sr):
    w = int(round(MA_WINDOW_S * sr))
    return pd.Series(np.asarray(x, dtype=float)).rolling(window=w, center=True, min_periods=1).mean().to_numpy()


def inputs(ctx):
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
    st, prm = ctx.state, ctx.params
    seed = estimate_sideslip(st, prm)
    slip = estimate_slip_angles(st, seed, prm)
    forces = estimate_lateral_forces(st, prm)
    sr = st["sample_rate_hz"]
    P = {ax: (slip[f"alpha_{ax}_filt"], forces[f"Fy_{ax}_filt"]) for ax in "fr"}
    M = {ax: (moving_average(slip[f"alpha_{ax}_raw"], sr), moving_average(forces[f"Fy_{ax}_raw"], sr)) for ax in "fr"}
    return seed, P, M


def make_ma_fitter(P, M, calls):
    def fitter(a, f, b):
        for ax in "fr":
            if np.array_equal(a, P[ax][0], equal_nan=True) and np.array_equal(f, P[ax][1], equal_nan=True):
                calls.append(ax)
                return ORIG_FIT(M[ax][0], M[ax][1], b)
        raise RuntimeError("STOP: fit called with arrays that are not P's fit inputs")
    return fitter


def curve(fit, grid):
    return np.abs(pacejka_lateral_force(grid, fit["B"], fit["C"], fit["D"], fit["E"]))


def lolo_f1(ctx, a, f, grid):
    curves = []
    for lap in ctx.analysis_laps():
        m = ctx.base & ~ctx.lap_mask({lap["lap_number"]})
        fit = ORIG_FIT(a, f, m)
        if fit["powell_converged"] and fit["sign_ok"]:
            curves.append(curve(fit, grid))
    if len(curves) < 2:
        return float("nan"), len(curves)
    c = np.array(curves)
    med = np.median(c, axis=0)
    return float(np.median(np.std(c, axis=0, ddof=1) / np.abs(med))), len(curves)


def m3_band(beta, ctx):
    curves = ss.corner_curves(beta, ctx.state, ctx.data, ctx.inst)
    v0, _n = ss.m3_value(curves)
    laps = sorted({l for cv in curves.values() for l in cv})
    rng = np.random.default_rng(20261002)
    bs = []
    for _ in range(ss.N_BOOT_M3):
        v, _n = ss.m3_value(curves, list(rng.choice(laps, size=len(laps), replace=True)))
        if np.isfinite(v):
            bs.append(v)
    med = np.median(bs)
    return float(v0), (float(v0 + np.percentile(bs, 2.5) - med), float(v0 + np.percentile(bs, 97.5) - med))


def session(key):
    ctx = lp.Ctx(key)
    st, sr = ctx.state, ctx.state["sample_rate_hz"]
    seed, P, M = inputs(ctx)
    # entry: harness P == direct production call now
    beta_dir, _man, _gate, _fb, _fr = tfa.resolve_sideslip_beta(st, ctx.params, ctx.data, "ekf_auto_pacejka",
                                                                csv_path=d2.sessions()[key]["path"])
    dn_dir = d2._downstream(ctx.params, ctx.data, st, beta_dir)
    beta_p, rec_p, dn_p = lp.run_chain(ctx, st, seed)
    entry = {"beta": float(np.nanmax(np.abs(np.asarray(beta_dir, float) - beta_p)))}
    same_set = np.array_equal(np.isfinite(np.asarray(beta_dir, float)), np.isfinite(beta_p))
    for ax in "fr":
        a, b = np.asarray(dn_dir[f"cs_{ax}"], float), np.asarray(dn_p[f"cs_{ax}"], float)
        same_set &= np.array_equal(np.isfinite(a), np.isfinite(b))
        fin = np.isfinite(a) & np.isfinite(b)
        entry[f"cs_{ax}"] = float(np.max(np.abs(a[fin] - b[fin]))) if fin.any() else 0.0
    entry_ok = bool(same_set and max(entry.values()) <= ENTRY_TOL)
    if not entry_ok:
        return {"key": key, "name": ctx.name, "entry": entry, "entry_ok": False}, None
    calls = []
    beta_m, rec_m, dn_m = lp.run_chain(ctx, st, seed, fitter=make_ma_fitter(P, M, calls))
    if sorted(calls) != ["f", "r"]:
        raise SystemExit(f"STOP {key}: MA fitter calls {calls}, expected one per axle")

    res = {"key": key, "name": ctx.name, "entry": entry, "entry_ok": True, "variants": {}}
    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    in_corner = ctx.in_br & on_lap
    straight = ctx.base & (np.abs(st["ax_mps2"]) < STR_G * 9.81) & (np.abs(st["ay_mps2"]) < STR_G * 9.81) & \
        (st["v_mps"] * 3.6 > STR_V_KPH)
    zone = st["kerb_smear_zone"]
    grids = {}
    for ax in "fr":
        a = np.abs(P[ax][0][ctx.base & np.isfinite(P[ax][0])])
        grids[ax] = np.linspace(np.percentile(a, GRID_LO), np.percentile(a, GRID_HI), GRID_N)
    m3p, band = m3_band(beta_p, ctx)
    for tag, inp, beta, rec, dn in (("P", P, beta_p, rec_p, dn_p), ("M", M, beta_m, rec_m, dn_m)):
        v = {"gate": rec["gate"], "status": rec["status"], "fallback": rec["fallback"],
             "health": rec["health"], "chatter_rms_deg": rec["chatter_rms_deg"], "axes": {}}
        v["G"] = bool(rec["status"] != "degenerate" and rec["gate"] == "pass" and not rec["fallback"] and
                      all(rec["axles"][AX[ax]]["powell_converged"] for ax in "fr"))
        v["M3"] = m3p if tag == "P" else float(lp.m3_pair(beta, ctx)[0])
        b = np.asarray(beta, float)
        v["B2_deg"] = float(np.median(np.abs(b[straight])) * RAD2DEG)
        for ax in "fr":
            a, f = inp[ax]
            full = ORIG_FIT(a, f, ctx.base)
            v["G"] &= bool(full["sign_ok"])
            f1, nfit = lolo_f1(ctx, a, f, grids[ax])
            noz = ORIG_FIT(a, f, ctx.base & ~zone)
            cf, cz = curve(full, grids[ax]), curve(noz, grids[ax])
            pa, pf = P[ax]
            mm = ctx.base & np.isfinite(pa) & np.isfinite(pf)
            f2 = float(np.sqrt(np.mean((pf[mm] - pacejka_lateral_force(pa[mm], full["B"], full["C"], full["D"], full["E"])) ** 2)))
            cs = np.asarray(dn[f"cs_{ax}"], float)
            fin = np.isfinite(cs) & in_corner
            v["axes"][ax] = {"fit": {k: float(full[k]) for k in "BCDE"}, "F1": f1, "F1_n": nfit, "F2_N": f2,
                             "S": float(np.median(np.abs(cz - cf) / cf)),
                             "zone_share": float(np.mean(zone[ctx.base & np.isfinite(a) & np.isfinite(f)])),
                             "cs_neg": float(np.mean(cs[fin] < 0)), "cs_deep": float(np.mean(cs[fin] < -1)),
                             "n_neg": int(np.sum(cs[fin] < 0)), "n_deep": int(np.sum(cs[fin] < -1)),
                             "n_fin": int(fin.sum()), "coverage": float(fin.sum() / max(1, in_corner.sum())),
                             "curve_full": cf.tolist(), "grid": grids[ax].tolist()}
        res["variants"][tag] = v
    res["M3_band_P"] = band
    return res, (ctx, beta_p, beta_m, P, M)


def judge(results):
    harm, cand = [], {}
    for r in results:
        p, m = r["variants"]["P"], r["variants"]["M"]
        if not m["G"]:
            harm.append(f"{r['key']} G")
        if m["M3"] > r["M3_band_P"][1]:
            harm.append(f"{r['key']} M3 above band")
        if m["B2_deg"] - p["B2_deg"] >= 0.1:
            harm.append(f"{r['key']} straight beta +{m['B2_deg'] - p['B2_deg']:.3f} deg")
        for ax in "fr":
            pa, ma = p["axes"][ax], m["axes"][ax]
            if (ma["cs_neg"] - pa["cs_neg"]) * 100 >= 0.5:
                harm.append(f"{r['key']} {ax} CS<0 +{(ma['cs_neg'] - pa['cs_neg']) * 100:.2f} pt")
            if (pa["coverage"] - ma["coverage"]) * 100 >= 1.0:
                harm.append(f"{r['key']} {ax} coverage -{(pa['coverage'] - ma['coverage']) * 100:.2f} pt")
            if ma["S"] > 1.5 * pa["S"] and ma["S"] - pa["S"] > 0.01:
                harm.append(f"{r['key']} {ax} S {pa['S']:.4f} -> {ma['S']:.4f}")
    cells = [(r["variants"]["P"]["axes"][ax], r["variants"]["M"]["axes"][ax]) for r in results for ax in "fr"]
    f1 = float(np.median([m["F1"] / p["F1"] - 1 for p, m in cells]))
    s = float(np.median([m["S"] / p["S"] - 1 for p, m in cells]))
    credit = [bool(r["variants"]["M"]["M3"] < r["M3_band_P"][0] and
                   r["variants"]["M"]["M3"] <= 0.9 * r["variants"]["P"]["M3"]) for r in results]
    pooled = lambda tag, k: sum(r["variants"][tag]["axes"][ax][k] for r in results for ax in "fr")
    deep = pooled("M", "n_deep") / pooled("M", "n_fin") / (pooled("P", "n_deep") / pooled("P", "n_fin")) - 1
    neg = pooled("M", "n_neg") / pooled("M", "n_fin") / (pooled("P", "n_neg") / pooled("P", "n_fin")) - 1
    cand = {"F1 -10 %": (f1, f1 <= -0.10), "M3 credit on >= 2 of 3": (sum(credit), sum(credit) >= 2),
            "CS<-1 -10 % rel": (deep, deep <= -0.10), "S -25 %": (s, s <= -0.25),
            "CS<0 -5 % rel (1a)": (neg, neg <= -0.05)}
    n_ok = sum(v[1] for v in cand.values())
    verdict = "HARM" if harm else ("EARNS" if n_ok >= 2 else "NEUTRAL")
    return verdict, harm, cand, credit


def render(results, extra):
    plt = lp._plt()
    for r in results:
        ctx, beta_p, beta_m, P, M = extra[r["key"]]
        fig, axs = plt.subplots(1, 2, figsize=(13, 5))
        for k, ax in enumerate("fr"):
            A = axs[k]
            m = ctx.base & np.isfinite(P[ax][0]) & np.isfinite(P[ax][1])
            sel = np.flatnonzero(m)[:: max(1, m.sum() // 15000)]
            A.scatter(P[ax][0][sel] * RAD2DEG, P[ax][1][sel] / 1000, s=1, color=lp.REF, alpha=0.4,
                      label="fit population, 2 Hz inputs")
            A.scatter(M[ax][0][sel] * RAD2DEG, M[ax][1][sel] / 1000, s=1, color=lp.ACC2, alpha=0.25,
                      label="fit population, 0.15 s moving-average inputs")
            g = np.linspace(-np.max(r["variants"]["P"]["axes"][ax]["grid"]) * 1.5,
                            np.max(r["variants"]["P"]["axes"][ax]["grid"]) * 1.5, 300)
            for tag, col, lab in (("P", lp.INK, "curve fitted on 2 Hz inputs (production)"),
                                  ("M", lp.ACC, "curve fitted on 0.15 s moving-average inputs")):
                fp = r["variants"][tag]["axes"][ax]["fit"]
                A.plot(g * RAD2DEG, pacejka_lateral_force(g, fp["B"], fp["C"], fp["D"], fp["E"]) / 1000,
                       color=col, lw=1.8, label=lab)
            A.set_xlabel("slip angle [deg]")
            A.set_ylabel("axle lateral force [kN]")
            A.set_title(f"{AX[ax]} axle", fontsize=10)
            lp._style(A)
        axs[0].legend(fontsize=7, markerscale=6)
        fig.suptitle(lp._wrap(f"{r['name']}: tyre-curve fit with production 2 Hz inputs vs the chair's 0.15 s "
                              f"moving average on raw inputs (calibration pass)", fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.92))
        fig.savefig(os.path.join(OUT_DIR, f"fit_{r['key']}.png"), dpi=130)
        plt.close(fig)
        laps = ctx.analysis_laps()
        lap = sorted(laps, key=lambda l: l["end_time"] - l["start_time"])[len(laps) // 2]
        lm = ctx.lap_mask({lap["lap_number"]})
        idx = np.flatnonzero(lm)
        idx = idx[np.argsort(ctx.state["s_m"][idx])]
        fig, A = plt.subplots(figsize=(13, 4.5))
        A.plot(ctx.state["s_m"][idx], np.asarray(beta_p)[idx] * RAD2DEG, color=lp.INK, lw=1.1,
               label="sideslip, production fit inputs")
        A.plot(ctx.state["s_m"][idx], np.asarray(beta_m)[idx] * RAD2DEG, color=lp.ACC, lw=1.1,
               label="sideslip, moving-average fit inputs")
        A.set_xlabel("lap distance [m]")
        A.set_ylabel("sideslip angle beta [deg]")
        A.legend(fontsize=8)
        lp._style(A)
        fig.suptitle(lp._wrap(f"{r['name']}, lap {lap['lap_number']} (median lap time): EKF sideslip with the "
                              f"production tyre fit vs the fit on 0.15 s moving-average inputs", fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.88))
        fig.savefig(os.path.join(OUT_DIR, f"beta_{r['key']}.png"), dpi=130)
        plt.close(fig)
    render_metrics(results)


def render_metrics(results):
    # Reads results.json content only, so mode "metrics" re-renders it without
    # recomputation. Tick labels name track, outing and axle in words (the
    # earlier key slice cut "nbr_jst_solution" to "lution").
    plt = lp._plt()
    metrics = [("F1 curve instability (leave-one-lap-out)", lambda v, ax: v["axes"][ax]["F1"]),
               ("S strike sensitivity (curve shift without the smear zone)", lambda v, ax: v["axes"][ax]["S"]),
               ("share of in-corner CS < 0", lambda v, ax: v["axes"][ax]["cs_neg"]),
               ("share of in-corner CS < -1", lambda v, ax: v["axes"][ax]["cs_deep"]),
               ("M3 sideslip lap-to-lap scatter [deg]", lambda v, ax: v["M3"])]
    fig, axs = plt.subplots(1, len(metrics), figsize=(18, 5.6))
    labels = [f"{r['name'].split(',')[0]}, {r['name'].split(',')[-1].strip()}\n{AX[ax]} axle"
              for r in results for ax in "fr"]
    for k, (title, fn) in enumerate(metrics):
        A = axs[k]
        for i, (r, ax) in enumerate([(r, ax) for r in results for ax in "fr"]):
            A.plot([i, i], [fn(r["variants"]["P"], ax), fn(r["variants"]["M"], ax)], color=lp.REF, lw=0.8)
            A.scatter([i], [fn(r["variants"]["P"], ax)], color=lp.INK, s=22, label="production 2 Hz" if i == 0 else None)
            A.scatter([i], [fn(r["variants"]["M"], ax)], color=lp.ACC, s=22, marker="D",
                      label="0.15 s moving average" if i == 0 else None)
        A.set_xticks(range(len(labels)))
        A.set_xticklabels(labels, rotation=55, ha="right", rotation_mode="anchor", fontsize=6.5)
        A.set_title(lp._wrap(title, fig), fontsize=8)
        lp._style(A)
    axs[0].legend(fontsize=7)
    fig.suptitle("Calibration pass, Tier 1: scored metrics with production 2 Hz fit inputs vs the chair's 0.15 s "
                 "moving average (lower is better)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT_DIR, "metrics.png"), dpi=130)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    results, extra, lines = [], {}, ["Q-CAL-3 CALIBRATION PASS: production 2 Hz fit inputs (P) vs chair 0.15 s MA (M)"]
    for key in TIER1:
        r, ex = session(key)
        lines.append(f"\n[{key}] {r['name']}: entry " + ", ".join(f"{k} {v:.1e}" for k, v in r["entry"].items())
                     + (" -> PASS" if r["entry_ok"] else " -> FAIL, session stopped"))
        if not r["entry_ok"]:
            continue
        results.append(r)
        extra[key] = ex
        for tag in ("P", "M"):
            v = r["variants"][tag]
            lines.append(f"  {tag}: G {v['G']} (gate {v['gate']}, health {v['health']}, fallback {v['fallback']}); "
                         f"M3 {v['M3']:.4f} deg; straight |beta| {v['B2_deg']:.3f} deg; chatter {v['chatter_rms_deg']:.4f} deg")
            for ax in "fr":
                a = v["axes"][ax]
                lines.append(f"     {AX[ax]}: fit B {a['fit']['B']:.3f} C {a['fit']['C']:.3f} D {a['fit']['D']:.0f} "
                             f"E {a['fit']['E']:.3f}; F1 {a['F1']:.4f} ({a['F1_n']} refits); F2 {a['F2_N']:.0f} N; "
                             f"S {a['S']:.4f} (zone share {a['zone_share']:.3f}); CS<0 {a['cs_neg'] * 100:.2f} %, "
                             f"CS<-1 {a['cs_deep'] * 100:.3f} %, coverage {a['coverage'] * 100:.2f} %")
        lines.append(f"  P's M3 band [{r['M3_band_P'][0]:.4f}, {r['M3_band_P'][1]:.4f}]")
        _log(f"[{key}] done")
    if len(results) < len(TIER1):
        lines.append("\nEXIT: STOPPED -- an entry check failed; no verdict")
    else:
        verdict, harm, cand, credit = judge(results)
        lines.append(f"\nHARM items: {harm or 'none'}")
        lines.append("EARNS candidates: " + "; ".join(f"{k}: {v[0]:+.3f} -> {'yes' if v[1] else 'no'}"
                                                         if isinstance(v[0], float) else f"{k}: {v[0]} -> {'yes' if v[1] else 'no'}"
                                                         for k, v in cand.items()))
        lines.append(f"M3 credit per session: {dict(zip([r['key'] for r in results], credit))}")
        lines.append(f"\nVERDICT (Tier 1): {verdict}")
        render(results, extra)
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=lp._jsonable)
    _log(text)


if __name__ == "__main__":
    if sys.argv[1:] == ["metrics"]:
        with open(os.path.join(OUT_DIR, "results.json")) as fh:
            render_metrics(json.load(fh))
    else:
        main()
