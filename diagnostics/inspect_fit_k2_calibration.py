# RECORD ONLY since 2026-10-04: production now passes the speed-windowed
# fit mask (tyre_fit_auto.fit_speed_window_pct) to the axle fit. This
# frame's fitter hook does NOT check the mask, so a re-run would silently
# fit the windowed population and not reproduce the recorded results;
# re-running as recorded needs fit_speed_window_pct null.
# Q-CAL-3 K2-CONSTRAINED CALIBRATION PASS (PLAN.md Q-CAL item 3, author ruling
# 2 of the Q-CAL-3 premise check, overnight package step 2, 2026-10-03).
# Read-only: no production or config change; the variant lives in memory
# inside the existing chain harness (inspect_last_push.run_chain, fitter hook),
# same pattern and entry check as inspect_fit_ma_calibration.py.
#
# PURPOSE: INFORMATIONAL FOR Q-K2. No production adoption follows from this
# pass regardless of outcome -- no production K2 exists yet. Verdict
# vocabulary HARM / PROMISING / NEUTRAL. Concept-vs-instance: a negative here
# rejects this instance on this chain, not the balance constraint as such.
#
# Tier A, author construction (NOT chair conditioning): Rajamani ch. 3, the
# steady-state cornering relation (sec. 3.2, Eq. 3.23) delta - L/R = K a_y
# with K = m_f / C_f - m_r / C_r (C = axle cornering stiffness, m_f = m l_r /
# L, m_r = m l_f / L). Q-PUSH P1 measured K2 beta-free per session; the tyre
# curves' origin slopes B C D imply their own K. Variant K pins that implied
# balance to the measured K2 as an equality constraint. Sign convention and
# expression inherited from inspect_last_push.p1 (K_fit), unchanged.
#
# RULES, frozen 2026-10-03 BEFORE any number (author package text; any
# conflict = STOP and ask). Identical to the MA pass frame except where
# marked AMENDED.
# VARIANTS. P = production: per-axle Pacejka free-D Powell fit on the
#   kinematic-seed slip angle and axle Fy through the 2 Hz Butterworth, base
#   mask, then EKF Pacejka -> NIS gate -> beta -> slip -> CS. K = joint
#   front/rear fit on THE SAME inputs and mask: minimise SSE_front + SSE_rear
#   (each on its own finite population, the production residual masks) subject
#   to m_f / (B_f C_f D_f) - m_r / (B_r C_r D_r) = K2. Unconstrained, this
#   objective separates into the two production fits, so K differs from P by
#   the constraint alone. Implementation: B_r eliminated through the
#   constraint, Powell (the production optimiser, default tolerances) on the
#   other seven parameters; infeasible points (B_f C_f D_f <= 0, m_f / C_f <=
#   K2, C_r D_r <= 0) return a large constant. Start = P's fit on the same
#   population with B_r replaced by its constraint-implied value (every K
#   refit -- full, leave-one-lap-out, no-smear-zone -- starts from the P refit
#   of the same population). Stated: rear forces are the larger, so SSE
#   weighting puts more of the balance correction on whichever axle's SSE is
#   flatter; no axle weighting is introduced.
# K2 SOURCE. The diagnostic per-session K2 recorded by Q-PUSH P1
#   (diagnostics/plots/last_push/p1_<key>.json, K_rad_per_mps2; +1.407 /
#   +1.356 / +1.272 deg/g on US-v2 / v3 / JST 29), read, not recomputed.
#   Level 2 (session measurement; steering ratio Level 1). Vehicle mass and
#   CoG Level 1 (config).
# ENTRY. P through the harness must reproduce a DIRECT production call made
#   now (beta and CS identical finite sets, max |diff| <= 1e-9), else the
#   session stops. K's fitter must be called exactly once per axle with P's
#   arrays recognised.
# DEGENERACY EXIT (pre-registered). If the constrained full-population fit
#   does not converge (Powell success false), returns non-finite parameters,
#   D <= 0 or B <= 0 on either axle, or misses the constraint by more than
#   1e-9 relative: the session is reported DEGENERATE (fit-level), no verdict
#   for it, and the package continues to step 3. A converged fit whose chain
#   then fails the gate is NOT degenerate -- that is the G harm item.
# SESSIONS. Tier 1: pr_us_v2, v3, nbr_jst_solution. No Tier 2 in this pass.
# METRICS (identical for P and K, definitions as the MA pass).
#   G gate: status not degenerate, NIS verdict pass, no fallback, Powell
#     converged and sign_ok on both axles.
#   F1 (scored): leave-one-lap-out refits (each analysis lap out in turn; K
#     refits are joint constrained); curve at 25 slip angles evenly spaced
#     between p10 and p90 of |alpha| on the base mask (P's 2 Hz alpha, same
#     grid for both); per point SD over the refits / |median Fy|; median.
#   F2 (descriptive): RMS of (2 Hz Fy - curve(2 Hz alpha)) on the base mask --
#     here the fit cost of the constraint (same inputs for both variants).
#   B1 (scored): M3 lap-to-lap beta scatter at matched corner positions; P's
#     two-sided bootstrap band (seed 20261002, ss.N_BOOT_M3 draws).
#   B2 (no-harm only): median |beta| on straights (base, |a_x|, |a_y| < 0.1 g,
#     v > 60 km/h), deg.
#   B3 (descriptive): beta chatter RMS above 2 Hz.
#   C: in-corner on analysis laps, per axle: share CS < 0, share CS < -1,
#     finite-CS coverage.
#   S (scored): refit on base & ~kerb_smear_zone (0.20 s); shift = median over
#     the grid of |dFy| / |Fy_full|.
#   Informational: K implied by P's curves now, constrained origin slopes and
#     their change vs P, constraint residual.
# EXITS (Tier 1, three sessions, both axles).
#   HARM if any: G fails; M3_K above P's band (any session); CS < 0 share
#     +2.5 points or more (any session / axle) [AMENDED: 2.5 replaces 0.5 --
#     the measured sensitivity of the share, Q-CAL-3 close-out]; coverage -1
#     point or more; straight |beta| +0.1 deg or more; S_K > 1.5 x S_P AND
#     S_K - S_P > 0.01.
#   PROMISING = no harm AND at least TWO of FIVE (the MA pass's EARNS set):
#     (1) F1 median over 6 cells of (F1_K / F1_P - 1) <= -10 %;
#     (2) M3 tie rule: credit only below P's band AND M3_K <= 0.9 x M3_P,
#         inside the band = tie; scores when >= 2 of 3 sessions earn credit;
#     (3) CS < -1 share pooled -10 % relative;
#     (4) S median over 6 cells of (S_K / S_P - 1) <= -25 %;
#     (5) CS < 0 share pooled -5 % relative.
#   NEUTRAL otherwise.
# OUTPUTS diagnostics/plots/fit_k2_calibration/: fit_<key>.png, beta_<key>.png,
#   metrics.png, summary.txt, results.json.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_fit_k2_calibration
#   [metrics]  re-render metrics.png from results.json only

import json
import os
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import modules.tyre_fit_auto as tfa
from diagnostics import inspect_fit_ma_calibration as ma
from diagnostics import inspect_last_push as lp
from diagnostics import inspect_tyre_prior_d2 as d2
from modules.tyre_model_pacejka import pacejka_lateral_force

OUT_DIR = "diagnostics/plots/fit_k2_calibration"
TIER1 = ma.TIER1
HARM_CS_NEG_PT = 2.5  # amended bar, see header
CONSTRAINT_TOL = 1e-9
INFEASIBLE = 1e300
ORIG_FIT = tfa._fit_axle_pacejka
RAD2DEG = ma.RAD2DEG
AX = ma.AX
G = 9.81


def _log(m):
    print(m, flush=True)


def k2_of(key):
    with open(f"diagnostics/plots/last_push/p1_{key}.json") as fh:
        k = json.load(fh)["K2"]
    return float(k["K_rad_per_mps2"]), float(k["K_deg_per_g"])


def implied_k(ctx, ff, fr):
    return ctx.m_f / (ff["B"] * ff["C"] * ff["D"]) - ctx.m_r / (fr["B"] * fr["C"] * fr["D"])


def _axle_dict(a, f, m2, B, C, D, E, ok):
    """Same keys as tyre_fit_auto._fit_axle_pacejka, for fixed parameters."""
    a2, f2 = a[m2], f[m2]
    resid = f2 - pacejka_lateral_force(a2, B, C, D, E)
    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    pk, inv = lp._peak(B, C, D, E, p99)
    return {"B": B, "C": C, "D": D, "E": E, "powell_converged": ok, "fit_n_samples": int(a2.size),
            "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))), "peak_alpha_deg": pk,
            "peak_in_visited_range": inv, "visited_alpha_p99_deg": p99,
            "sign_ok": bool(D > 0), "residuals": resid, "residual_mask": m2}


def joint_fit(ctx, inp, mask, K, pf, pr):
    """Constrained joint fit (header VARIANTS). pf / pr = P's fits on the same
    population, used as the start. Returns (front, rear, info)."""
    af, ff = inp["f"]
    ar, fr = inp["r"]
    mf = mask & np.isfinite(af) & np.isfinite(ff)
    mr = mask & np.isfinite(ar) & np.isfinite(fr)
    a1, f1, a2, f2 = af[mf], ff[mf], ar[mr], fr[mr]
    m_f, m_r = ctx.m_f, ctx.m_r

    def b_rear(p):
        Bf, Cf, Df, _Ef, Cr, Dr, _Er = p
        cf = Bf * Cf * Df
        if not np.isfinite(cf) or cf <= 0:
            return float("nan")
        den = m_f / cf - K
        crd = Cr * Dr
        if den <= 0 or not np.isfinite(crd) or crd <= 0:
            return float("nan")
        return m_r / (crd * den)

    def j(p):
        br = b_rear(p)
        if not np.isfinite(br):
            return INFEASIBLE
        Bf, Cf, Df, Ef, Cr, Dr, Er = p
        s = np.sum((pacejka_lateral_force(a1, Bf, Cf, Df, Ef) - f1) ** 2) + \
            np.sum((pacejka_lateral_force(a2, br, Cr, Dr, Er) - f2) ** 2)
        return float(s) if np.isfinite(s) else INFEASIBLE

    x0 = np.array([pf["B"], pf["C"], pf["D"], pf["E"], pr["C"], pr["D"], pr["E"]], dtype=float)
    info = {"start_feasible": bool(np.isfinite(b_rear(x0)))}
    if not info["start_feasible"]:
        info.update(ok=False, reason="start infeasible (m_f / C_f <= K2 at P's fit)")
        return None, None, info
    res = minimize(j, x0, method="Powell")
    Bf, Cf, Df, Ef, Cr, Dr, Er = (float(v) for v in res.x)
    br = b_rear(res.x)
    finite = all(np.isfinite([Bf, Cf, Df, Ef, Cr, Dr, Er, br]))
    front = _axle_dict(af, ff, mf, Bf, Cf, Df, Ef, bool(res.success)) if finite else None
    rear = _axle_dict(ar, fr, mr, br, Cr, Dr, Er, bool(res.success)) if finite else None
    resid = abs(implied_k(ctx, front, rear) - K) / abs(K) if finite else float("nan")
    reasons = []
    if not res.success:
        reasons.append("Powell not converged")
    if not finite:
        reasons.append("non-finite parameters")
    elif min(Bf, Df, br, Dr) <= 0:
        reasons.append("B or D <= 0")
    if finite and not resid <= CONSTRAINT_TOL:
        reasons.append(f"constraint residual {resid:.1e}")
    info.update(ok=not reasons, reason="; ".join(reasons) or None, constraint_rel_resid=resid,
                n_fev=int(res.nfev), sse=float(res.fun))
    return front, rear, info


def p_pair(inp, mask):
    return {ax: ORIG_FIT(inp[ax][0], inp[ax][1], mask) for ax in "fr"}


def k_pair(ctx, inp, mask, K, pp=None):
    pp = p_pair(inp, mask) if pp is None else pp
    f, r, info = joint_fit(ctx, inp, mask, K, pp["f"], pp["r"])
    return ({"f": f, "r": r} if info["ok"] else None), info


def make_k_fitter(P, kfit, calls):
    def fitter(a, f, b):
        for ax in "fr":
            if np.array_equal(a, P[ax][0], equal_nan=True) and np.array_equal(f, P[ax][1], equal_nan=True):
                calls.append(ax)
                return kfit[ax]
        raise RuntimeError("STOP: fit called with arrays that are not P's fit inputs")
    return fitter


def lolo_f1(ctx, inp, K, grid_f, grid_r):
    curves = {"P": {"f": [], "r": []}, "K": {"f": [], "r": []}}
    for lap in ctx.analysis_laps():
        m = ctx.base & ~ctx.lap_mask({lap["lap_number"]})
        pp = p_pair(inp, m)
        kp, _info = k_pair(ctx, inp, m, K, pp)
        for tag, pair in (("P", pp), ("K", kp)):
            if pair is None:
                continue
            for ax, g in (("f", grid_f), ("r", grid_r)):
                if pair[ax]["powell_converged"] and pair[ax]["sign_ok"]:
                    curves[tag][ax].append(ma.curve(pair[ax], g))
    out = {}
    for tag in "PK":
        out[tag] = {}
        for ax in "fr":
            c = curves[tag][ax]
            if len(c) < 2:
                out[tag][ax] = (float("nan"), len(c))
            else:
                c = np.array(c)
                out[tag][ax] = (float(np.median(np.std(c, axis=0, ddof=1) / np.abs(np.median(c, axis=0)))), len(c))
    return out


def entry(ctx, st, seed):
    beta_dir, _man, _gate, _fb, _fr = tfa.resolve_sideslip_beta(st, ctx.params, ctx.data, "ekf_auto_pacejka",
                                                                csv_path=d2.sessions()[ctx.key]["path"])
    dn_dir = d2._downstream(ctx.params, ctx.data, st, beta_dir)
    beta_p, rec_p, dn_p = lp.run_chain(ctx, st, seed)
    e = {"beta": float(np.nanmax(np.abs(np.asarray(beta_dir, float) - beta_p)))}
    same = np.array_equal(np.isfinite(np.asarray(beta_dir, float)), np.isfinite(beta_p))
    for ax in "fr":
        a, b = np.asarray(dn_dir[f"cs_{ax}"], float), np.asarray(dn_p[f"cs_{ax}"], float)
        same &= np.array_equal(np.isfinite(a), np.isfinite(b))
        fin = np.isfinite(a) & np.isfinite(b)
        e[f"cs_{ax}"] = float(np.max(np.abs(a[fin] - b[fin]))) if fin.any() else 0.0
    return e, bool(same and max(e.values()) <= ma.ENTRY_TOL), (beta_p, rec_p, dn_p)


def session(key):
    ctx = lp.Ctx(key)
    st = ctx.state
    K, K_deg = k2_of(key)
    seed, P, _M = ma.inputs(ctx)
    e, ok, (beta_p, rec_p, dn_p) = entry(ctx, st, seed)
    res = {"key": key, "name": ctx.name, "entry": e, "entry_ok": ok, "K2_rad_per_mps2": K, "K2_deg_per_g": K_deg,
           "m_f_kg": ctx.m_f, "m_r_kg": ctx.m_r}
    if not ok:
        return res, None
    pp = p_pair(P, ctx.base)
    kp, info = k_pair(ctx, P, ctx.base, K, pp)
    res["K_fit_info"] = info
    res["K_implied_by_P_deg_per_g"] = float(implied_k(ctx, pp["f"], pp["r"]) * RAD2DEG * G)
    if kp is None:
        res["degenerate"] = True
        return res, None
    res["degenerate"] = False
    calls = []
    beta_k, rec_k, dn_k = lp.run_chain(ctx, st, seed, fitter=make_k_fitter(P, kp, calls))
    if sorted(calls) != ["f", "r"]:
        raise SystemExit(f"STOP {key}: K fitter calls {calls}, expected one per axle")
    res["K_implied_by_K_deg_per_g"] = float(implied_k(ctx, kp["f"], kp["r"]) * RAD2DEG * G)
    res["origin_slopes_N_per_rad"] = {tag: {ax: float(pr[ax]["B"] * pr[ax]["C"] * pr[ax]["D"]) for ax in "fr"}
                                      for tag, pr in (("P", pp), ("K", kp))}

    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    in_corner = ctx.in_br & on_lap
    straight = ctx.base & (np.abs(st["ax_mps2"]) < ma.STR_G * G) & (np.abs(st["ay_mps2"]) < ma.STR_G * G) & \
        (st["v_mps"] * 3.6 > ma.STR_V_KPH)
    zone = st["kerb_smear_zone"]
    grids = {}
    for ax in "fr":
        a = np.abs(P[ax][0][ctx.base & np.isfinite(P[ax][0])])
        grids[ax] = np.linspace(np.percentile(a, ma.GRID_LO), np.percentile(a, ma.GRID_HI), ma.GRID_N)
    _log(f"[{key}] leave-one-lap-out refits ...")
    f1 = lolo_f1(ctx, P, K, grids["f"], grids["r"])
    noz_p = p_pair(P, ctx.base & ~zone)
    noz_k, noz_info = k_pair(ctx, P, ctx.base & ~zone, K, noz_p)
    res["noz_K_info"] = noz_info
    m3p, band = ma.m3_band(beta_p, ctx)
    res["M3_band_P"] = band
    res["variants"] = {}
    for tag, full, noz, beta, rec, dn in (("P", pp, noz_p, beta_p, rec_p, dn_p), ("K", kp, noz_k, beta_k, rec_k, dn_k)):
        v = {"gate": rec["gate"], "status": rec["status"], "fallback": rec["fallback"],
             "health": rec["health"], "chatter_rms_deg": rec["chatter_rms_deg"], "axes": {}}
        v["G"] = bool(rec["status"] != "degenerate" and rec["gate"] == "pass" and not rec["fallback"] and
                      all(rec["axles"][AX[ax]]["powell_converged"] for ax in "fr"))
        v["M3"] = m3p if tag == "P" else float(lp.m3_pair(beta, ctx)[0])
        b = np.asarray(beta, float)
        v["B2_deg"] = float(np.median(np.abs(b[straight])) * RAD2DEG)
        for ax in "fr":
            fit = full[ax]
            v["G"] &= bool(fit["sign_ok"])
            cf = ma.curve(fit, grids[ax])
            s = float(np.median(np.abs(ma.curve(noz[ax], grids[ax]) - cf) / cf)) if noz is not None else float("nan")
            pa, pf = P[ax]
            mm = ctx.base & np.isfinite(pa) & np.isfinite(pf)
            f2 = float(np.sqrt(np.mean((pf[mm] - pacejka_lateral_force(pa[mm], fit["B"], fit["C"], fit["D"], fit["E"])) ** 2)))
            cs = np.asarray(dn[f"cs_{ax}"], float)
            fin = np.isfinite(cs) & in_corner
            v["axes"][ax] = {"fit": {k: float(fit[k]) for k in "BCDE"}, "F1": f1[tag][ax][0], "F1_n": f1[tag][ax][1],
                             "F2_N": f2, "S": s,
                             "cs_neg": float(np.mean(cs[fin] < 0)), "cs_deep": float(np.mean(cs[fin] < -1)),
                             "n_neg": int(np.sum(cs[fin] < 0)), "n_deep": int(np.sum(cs[fin] < -1)),
                             "n_fin": int(fin.sum()), "coverage": float(fin.sum() / max(1, in_corner.sum())),
                             "grid": grids[ax].tolist()}
        res["variants"][tag] = v
    return res, (ctx, beta_p, beta_k, P)


def judge(results):
    harm = []
    for r in results:
        p, m = r["variants"]["P"], r["variants"]["K"]
        if not m["G"]:
            harm.append(f"{r['key']} G (gate {m['gate']}, status {m['status']}, fallback {m['fallback']})")
        if m["M3"] > r["M3_band_P"][1]:
            harm.append(f"{r['key']} M3 above band ({m['M3']:.3f} > {r['M3_band_P'][1]:.3f})")
        if m["B2_deg"] - p["B2_deg"] >= 0.1:
            harm.append(f"{r['key']} straight beta +{m['B2_deg'] - p['B2_deg']:.3f} deg")
        for ax in "fr":
            pa, ka = p["axes"][ax], m["axes"][ax]
            if (ka["cs_neg"] - pa["cs_neg"]) * 100 >= HARM_CS_NEG_PT:
                harm.append(f"{r['key']} {ax} CS<0 +{(ka['cs_neg'] - pa['cs_neg']) * 100:.2f} pt")
            if (pa["coverage"] - ka["coverage"]) * 100 >= 1.0:
                harm.append(f"{r['key']} {ax} coverage -{(pa['coverage'] - ka['coverage']) * 100:.2f} pt")
            if not np.isfinite(ka["S"]) or (ka["S"] > 1.5 * pa["S"] and ka["S"] - pa["S"] > 0.01):
                harm.append(f"{r['key']} {ax} S {pa['S']:.4f} -> {ka['S']:.4f}")
    cells = [(r["variants"]["P"]["axes"][ax], r["variants"]["K"]["axes"][ax]) for r in results for ax in "fr"]
    f1 = float(np.nanmedian([k["F1"] / p["F1"] - 1 for p, k in cells]))
    s = float(np.nanmedian([k["S"] / p["S"] - 1 for p, k in cells]))
    credit = [bool(r["variants"]["K"]["M3"] < r["M3_band_P"][0] and
                   r["variants"]["K"]["M3"] <= 0.9 * r["variants"]["P"]["M3"]) for r in results]
    pooled = lambda tag, k: sum(r["variants"][tag]["axes"][ax][k] for r in results for ax in "fr")
    deep = pooled("K", "n_deep") / pooled("K", "n_fin") / (pooled("P", "n_deep") / pooled("P", "n_fin")) - 1
    neg = pooled("K", "n_neg") / pooled("K", "n_fin") / (pooled("P", "n_neg") / pooled("P", "n_fin")) - 1
    cand = {"F1 -10 %": (f1, f1 <= -0.10), "M3 credit on >= 2 of 3": (sum(credit), sum(credit) >= 2),
            "CS<-1 -10 % rel": (deep, deep <= -0.10), "S -25 %": (s, s <= -0.25),
            "CS<0 -5 % rel": (neg, neg <= -0.05)}
    n_ok = sum(v[1] for v in cand.values())
    verdict = "HARM" if harm else ("PROMISING" if n_ok >= 2 else "NEUTRAL")
    return verdict, harm, cand, credit


def render(results, extra):
    plt = lp._plt()
    for r in results:
        ctx, beta_p, beta_k, P = extra[r["key"]]
        fig, axs = plt.subplots(1, 2, figsize=(13, 5))
        for k, ax in enumerate("fr"):
            A = axs[k]
            m = ctx.base & np.isfinite(P[ax][0]) & np.isfinite(P[ax][1])
            sel = np.flatnonzero(m)[:: max(1, m.sum() // 15000)]
            A.scatter(P[ax][0][sel] * RAD2DEG, P[ax][1][sel] / 1000, s=1, color=lp.REF, alpha=0.4,
                      label="fit population (2 Hz inputs, same for both)")
            gmax = np.max(r["variants"]["P"]["axes"][ax]["grid"]) * 1.5
            g = np.linspace(-gmax, gmax, 300)
            for tag, col, lab in (("P", lp.INK, "unconstrained fit (production)"),
                                  ("K", lp.ACC, "fit with balance pinned to measured understeer gradient")):
                fp = r["variants"][tag]["axes"][ax]["fit"]
                slope = fp["B"] * fp["C"] * fp["D"] / 1000
                A.plot(g * RAD2DEG, pacejka_lateral_force(g, fp["B"], fp["C"], fp["D"], fp["E"]) / 1000,
                       color=col, lw=1.8, label=f"{lab}, origin slope {slope:.0f} kN/rad")
            A.set_xlabel("slip angle [deg]")
            A.set_ylabel("axle lateral force [kN]")
            A.set_title(f"{AX[ax]} axle", fontsize=10)
            A.legend(fontsize=7, markerscale=6)
            lp._style(A)
        fig.suptitle(lp._wrap(f"{r['name']}: tyre-curve fit, production vs front/rear balance pinned to the "
                              f"measured understeer gradient ({r['K2_deg_per_g']:+.2f} deg/g; production curves "
                              f"imply {r['K_implied_by_P_deg_per_g']:+.2f} deg/g)", fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.90))
        fig.savefig(os.path.join(OUT_DIR, f"fit_{r['key']}.png"), dpi=130)
        plt.close(fig)
        laps = ctx.analysis_laps()
        lap = sorted(laps, key=lambda l: l["end_time"] - l["start_time"])[len(laps) // 2]
        idx = np.flatnonzero(ctx.lap_mask({lap["lap_number"]}))
        idx = idx[np.argsort(ctx.state["s_m"][idx])]
        fig, A = plt.subplots(figsize=(13, 4.5))
        A.plot(ctx.state["s_m"][idx], np.asarray(beta_p)[idx] * RAD2DEG, color=lp.INK, lw=1.1,
               label="sideslip, production tyre fit")
        A.plot(ctx.state["s_m"][idx], np.asarray(beta_k)[idx] * RAD2DEG, color=lp.ACC, lw=1.1,
               label="sideslip, balance-pinned tyre fit")
        A.set_xlabel("lap distance [m]")
        A.set_ylabel("sideslip angle beta [deg]")
        A.legend(fontsize=8)
        lp._style(A)
        fig.suptitle(lp._wrap(f"{r['name']}, lap {lap['lap_number']} (median lap time): EKF sideslip with the "
                              f"production tyre fit vs the fit with the balance pinned to the measured "
                              f"understeer gradient", fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.88))
        fig.savefig(os.path.join(OUT_DIR, f"beta_{r['key']}.png"), dpi=130)
        plt.close(fig)
    render_metrics(results)


def render_metrics(results):
    plt = lp._plt()
    metrics = [("F1 curve instability (leave-one-lap-out)", lambda v, ax: v["axes"][ax]["F1"]),
               ("S strike sensitivity (curve shift without the smear zone)", lambda v, ax: v["axes"][ax]["S"]),
               ("share of in-corner CS < 0", lambda v, ax: v["axes"][ax]["cs_neg"]),
               ("share of in-corner CS < -1", lambda v, ax: v["axes"][ax]["cs_deep"]),
               ("M3 sideslip lap-to-lap scatter [deg]", lambda v, ax: v["M3"])]
    fig, axs = plt.subplots(1, len(metrics), figsize=(18, 5.6))
    pairs = [(r, ax) for r in results for ax in "fr"]
    labels = [f"{r['name'].split(',')[0]}, {r['name'].split(',')[-1].strip()}\n{AX[ax]} axle" for r, ax in pairs]
    for k, (title, fn) in enumerate(metrics):
        A = axs[k]
        for i, (r, ax) in enumerate(pairs):
            A.plot([i, i], [fn(r["variants"]["P"], ax), fn(r["variants"]["K"], ax)], color=lp.REF, lw=0.8)
            A.scatter([i], [fn(r["variants"]["P"], ax)], color=lp.INK, s=22, label="production fit" if i == 0 else None)
            A.scatter([i], [fn(r["variants"]["K"], ax)], color=lp.ACC, s=22, marker="D",
                      label="balance pinned to measured understeer gradient" if i == 0 else None)
        A.set_xticks(range(len(labels)))
        A.set_xticklabels(labels, rotation=55, ha="right", rotation_mode="anchor", fontsize=6.5)
        A.set_title(lp._wrap(title, fig), fontsize=8)
        lp._style(A)
    axs[0].legend(fontsize=7)
    fig.suptitle("Calibration pass, Tier 1: scored metrics, production tyre fit vs the fit with front/rear balance "
                 "pinned to the measured understeer gradient (lower is better)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT_DIR, "metrics.png"), dpi=130)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    results, extra, lines = [], {}, ["Q-CAL-3 K2-CONSTRAINED CALIBRATION PASS: production fit (P) vs balance "
                                     "pinned to diagnostic K2 (K) -- informational for Q-K2"]
    degenerate = []
    for key in TIER1:
        r, ex = session(key)
        lines.append(f"\n[{key}] {r['name']}: K2 {r['K2_deg_per_g']:+.3f} deg/g; entry "
                     + ", ".join(f"{k} {v:.1e}" for k, v in r["entry"].items())
                     + (" -> PASS" if r["entry_ok"] else " -> FAIL, session stopped"))
        if not r["entry_ok"]:
            continue
        lines.append(f"  balance implied by production curves now: {r['K_implied_by_P_deg_per_g']:+.3f} deg/g")
        lines.append(f"  constrained fit: {r['K_fit_info']}")
        if r["degenerate"]:
            degenerate.append(key)
            lines.append("  DEGENERATE (fit-level, pre-registered exit) -- no verdict for this session")
            results.append(r)
            continue
        os_ = r["origin_slopes_N_per_rad"]
        lines.append(f"  origin slopes kN/rad: front {os_['P']['f'] / 1e3:.1f} -> {os_['K']['f'] / 1e3:.1f}, "
                     f"rear {os_['P']['r'] / 1e3:.1f} -> {os_['K']['r'] / 1e3:.1f}; K-fit balance "
                     f"{r['K_implied_by_K_deg_per_g']:+.3f} deg/g; no-zone refit {r['noz_K_info'].get('reason') or 'ok'}")
        results.append(r)
        extra[key] = ex
        for tag in ("P", "K"):
            v = r["variants"][tag]
            lines.append(f"  {tag}: G {v['G']} (gate {v['gate']}, health {v['health']}, fallback {v['fallback']}); "
                         f"M3 {v['M3']:.4f} deg; straight |beta| {v['B2_deg']:.3f} deg; chatter {v['chatter_rms_deg']:.4f} deg")
            for ax in "fr":
                a = v["axes"][ax]
                lines.append(f"     {AX[ax]}: fit B {a['fit']['B']:.3f} C {a['fit']['C']:.3f} D {a['fit']['D']:.0f} "
                             f"E {a['fit']['E']:.3f}; F1 {a['F1']:.4f} ({a['F1_n']} refits); F2 {a['F2_N']:.0f} N; "
                             f"S {a['S']:.4f}; CS<0 {a['cs_neg'] * 100:.2f} %, CS<-1 {a['cs_deep'] * 100:.3f} %, "
                             f"coverage {a['coverage'] * 100:.2f} %")
        lines.append(f"  P's M3 band [{r['M3_band_P'][0]:.4f}, {r['M3_band_P'][1]:.4f}]")
        _log(f"[{key}] done")
    scored = [r for r in results if r.get("entry_ok") and not r.get("degenerate")]
    if len(scored) < len(TIER1):
        lines.append(f"\nEXIT: no Tier-1 verdict -- entry failed or fit DEGENERATE on "
                     f"{[r['key'] for r in results if r not in scored] or 'a session'}; package continues to step 3")
        if scored:
            render(scored, extra)
    else:
        verdict, harm, cand, credit = judge(scored)
        lines.append(f"\nHARM items (CS<0 bar {HARM_CS_NEG_PT} points): {harm or 'none'}")
        lines.append("PROMISING candidates: " + "; ".join(
            f"{k}: {v[0]:+.3f} -> {'yes' if v[1] else 'no'}" if isinstance(v[0], float)
            else f"{k}: {v[0]} -> {'yes' if v[1] else 'no'}" for k, v in cand.items()))
        lines.append(f"M3 credit per session: {dict(zip([r['key'] for r in scored], credit))}")
        lines.append(f"\nVERDICT (Tier 1, informational for Q-K2): {verdict}")
        render(scored, extra)
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=lp._jsonable)
    _log(text)


if __name__ == "__main__":
    if sys.argv[1:] == ["metrics"]:
        with open(os.path.join(OUT_DIR, "results.json")) as fh:
            render_metrics([r for r in json.load(fh) if r.get("variants")])
    else:
        main()
