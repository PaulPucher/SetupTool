# RECORD ONLY since 2026-10-04: production now passes the speed-windowed
# fit mask (tyre_fit_auto.fit_speed_window_pct) to the axle fit, so this
# frame's fitter hook stops on its base-mask / input check by design. Its
# results stand as recorded; re-running needs fit_speed_window_pct null.
# Q-FIT-DECISIVE -- data-entry levers for the tyre fit, judged in one frame
# (PLAN.md 2026-10-04: standing ruling "the fit stays open until it works",
# Q-FIT-CHAIR-DATA findings, author rulings and EXECUTE 2026-10-04).
# Read-only: no production or config change; every variant changes only the
# FIT POPULATION and lives in memory behind the fitter hook of the existing
# chain harness (inspect_last_push.run_chain), MA-pass pattern and entry check.
#
# RULES, frozen 2026-10-04 BEFORE any fit (any conflict = STOP and ask).
# VARIANTS (fit on production inputs: kinematic-seed alpha and axle Fy through
#   the 2 Hz Butterworth; only the mask differs):
#   P  production fit, base mask (baseline).
#   SW speed window -- chair-convergent (the effective window of the chair's
#      speed-band union) and theory-consistent (narrows the aero-load spread
#      towards the given-Fz condition of the Magic Formula, Pacejka sec. 4.3.1
#      Eq. 4.49). Rule: per session, keep base samples with speed inside
#      [p1, p99] of the speed inside corner instances on analysis laps.
#      Window values = vehicle parameterisation, derived from our data.
#   Z  minus the kerb smear zone (shipped 0.20 s, outside the mask) -- forced by
#      our car (kerb strikes leak through the 2 Hz filter).
#   LX loose large-Fx exclusion -- Tier B, theory-consistent (Pacejka sec. 4.3.1
#      Eqs. 4.64-4.69 combined-slip weighting ~ 1 at moderate Fx; sec. 1.3.4),
#      not chair practice. Rule: per session, drop base samples whose 2 Hz a_x
#      lies below its p5 (heavy braking) or above its p95 (heavy traction);
#      the derived g-bounds are reported beside the percentiles.
#   STAGE 2: every combination of >= 2 stage-1 SURVIVORS (verdict not HARM,
#      sanity screen included) is fitted and judged on its own -- nothing is
#      implied from the singles.
# ENTRY: P through the harness reproduces a direct production call now (beta
#   and CS identical finite sets, max |diff| <= 1e-9), else the session stops.
#   Each variant's fitter must be called once per axle with P's arrays and the
#   production base mask.
# METRICS (the MA-pass frame verbatim): G gate; F1 leave-one-lap-out curve
#   instability on the variant's population (scored); F2 RMS residual vs the 2 Hz
#   view on the base mask (descriptive); B1 M3 lap-to-lap beta scatter with P's
#   bootstrap band and the tie rule (scored); B2 straight-line |beta|
#   (no-harm); B3 chatter (descriptive); C in-corner CS < 0, CS < -1, coverage;
#   S strike sensitivity = refit on variant mask minus the smear zone, median
#   relative curve shift (scored; NOT scored for variants that already exclude
#   the zone -- Z and every combination containing Z -- where it is 0 by
#   construction).
# SANITY SCREEN (K2 ruling, bounds confirmed 2026-10-04): any axle with a
#   non-finite parameter, B < 1, C <= 1, C > 3 or E > 1 is a HARM item (C <= 3
#   widened from the recorded 2: production US-v2 rear sits at 2.002).
# EXITS per variant vs P (Tier 1, three sessions, both axles):
#   HARM if any: G fails; screen fails; M3 above P's band (any session); CS < 0
#     share +2.5 points or more (any cell); coverage -1 point or more;
#     straight |beta| +0.1 deg or more; S > 1.5 x P AND +0.01 (where scored).
#   EARNS = no harm AND >= 2 candidates met (absolute bar for all): (1) F1
#     median over 6 cells <= -10 %; (2) M3 credit (below P's band AND <= 0.9 x
#     P) on >= 2 of 3 sessions; (3) pooled CS < -1 share -10 % rel; (4) S median
#     <= -25 % (where scored); (5) pooled CS < 0 share -5 % rel.
#   NEUTRAL otherwise.
# SHIPPING RULE (pre-registered): among EARNS variants, the highest SHARE of
#   reachable candidates (4 for zone-excluding variants, 5 otherwise) ships;
#   tie -> fewer levers; tie -> lower pooled in-corner CS < 0 share. None earns
#   -> nothing ships, production stays, the fit stays open. A shipped variant
#   enters config as its derivation percentiles, not as fixed values.
# SESSIONS: Tier 1 -- pr_us_v2, v3, nbr_jst_solution.
# OUTPUTS diagnostics/plots/fit_decisive/: fit_<key>.png, metrics.png,
#   summary.txt, results.json, run_log.txt.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_fit_decisive [lt]
#
# MODE lt -- NEW FRAME, frozen 2026-10-04 before any number (author ruling 2
# after Q-FIT-DECISIVE, EXECUTE). Variant LT (traction tail only): per session
# drop base samples whose 2 Hz a_x lies ABOVE its p95 (+0.51 g on Tier 1); heavy
# braking is KEPT. Physics: the front's high-slip support lives in braking
# samples (LX's JST 29 front lost its peak when the braking tail went), while
# the rear's large-Fx regime is traction (rear-wheel drive). Tier B,
# theory-consistent (Pacejka sec. 4.3.1 Eqs. 4.64-4.69, sec. 1.3.4), not chair
# practice. Pre-registered expectation: rear-side gains without the front
# degeneration. Same metrics, bars, sanity screen and verdict vocabulary as
# above; S is scored (LT keeps the smear zone). One variant, so no stage 2; the
# shipping rule applies to LT alone. Outputs carry an _lt suffix:
# fit_lt_<key>.png, metrics_lt.png, summary_lt.txt, results_lt.json,
# run_log_lt.txt.
#
# MODE cs (plots only, 2026-10-04, author request; PHASE 1 PLAN rule 2a: every
# CS-touching package renders per-session CS traces, which the decisive and LT
# passes had not): reruns the chain per variant (P, SW, Z, LX, LT; no metrics,
# no verdicts) and renders cs_trace_<key>.png (median-time lap, all variants),
# cs_distribution_<key>.png (in-corner CS histograms) and fit_all_<key>.png
# (all five tyre curves); arrays cached in cs_cache/.

import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import modules.tyre_fit_auto as tfa
from diagnostics import inspect_fit_k2_calibration as k2
from diagnostics import inspect_fit_ma_calibration as ma
from diagnostics import inspect_last_push as lp
from modules.tyre_model_pacejka import pacejka_lateral_force

OUT_DIR = "diagnostics/plots/fit_decisive"
TIER1 = ma.TIER1
LEVERS = ("SW", "Z", "LX")
SW_PCT = (1, 99)
LX_PCT = (5, 95)
HARM_CS_NEG_PT = 2.5
ORIG_FIT = tfa._fit_axle_pacejka
RAD2DEG = ma.RAD2DEG
AX = ma.AX
G = 9.81
DESC = {"SW": "speed window", "Z": "smear zone removed", "LX": "large-Fx tails removed",
        "LT": "traction tail removed"}


def _log(m):
    print(m, flush=True)


def vname(levers):
    return "+".join(levers) if levers else "P"


def vdesc(levers):
    return " + ".join(DESC[l] for l in levers) if levers else "production fit"


class Session:
    def __init__(self, key):
        self.key = key
        self.ctx = ctx = lp.Ctx(key)
        st = ctx.state
        self.seed, self.P, _M = ma.inputs(ctx)
        self.entry, self.entry_ok, (self.beta_p, self.rec_p, self.dn_p) = k2.entry(ctx, st, self.seed)
        sr = st["sample_rate_hz"]
        fc = ctx.params["stability_estimation"]["cs_filter_cutoff_hz"]
        v = st["v_mps"]
        on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
        self.in_corner = ctx.in_br & on_lap
        inc = self.in_corner & st["moving_mask"]
        lo, hi = np.percentile(v[inc], SW_PCT)
        axf = lp._lp(st["ax_mps2"], fc, sr)
        a_lo, a_hi = np.percentile(axf[ctx.base], LX_PCT)
        self.keep = {"SW": (v >= lo) & (v <= hi), "Z": ~st["kerb_smear_zone"], "LX": (axf >= a_lo) & (axf <= a_hi),
                     "LT": axf <= a_hi}
        nb = ctx.base.sum()
        self.bounds = {"SW_kmh": [float(lo * 3.6), float(hi * 3.6)], "LX_g": [float(a_lo / G), float(a_hi / G)],
                       "SW_removed_low": float((ctx.base & (v < lo)).sum() / nb),
                       "SW_removed_high": float((ctx.base & (v > hi)).sum() / nb)}
        self.straight = ctx.base & (np.abs(st["ax_mps2"]) < ma.STR_G * G) & (np.abs(st["ay_mps2"]) < ma.STR_G * G) & \
            (v * 3.6 > ma.STR_V_KPH)
        self.grids = {}
        for ax in "fr":
            a = np.abs(self.P[ax][0][ctx.base & np.isfinite(self.P[ax][0])])
            self.grids[ax] = np.linspace(np.percentile(a, ma.GRID_LO), np.percentile(a, ma.GRID_HI), ma.GRID_N)
        self.m3p, self.band = ma.m3_band(self.beta_p, ctx)

    def mask(self, levers):
        m = self.ctx.base.copy()
        for l in levers:
            m &= self.keep[l]
        return m

    def fitter(self, m, calls):
        P, base = self.P, self.ctx.base

        def f(a, fy, b):
            if not np.array_equal(b, base):
                raise RuntimeError("STOP: production base mask differs from the harness base mask")
            for ax in "fr":
                if np.array_equal(a, P[ax][0], equal_nan=True) and np.array_equal(fy, P[ax][1], equal_nan=True):
                    calls.append(ax)
                    return ORIG_FIT(a, fy, m)
            raise RuntimeError("STOP: fit called with arrays that are not P's fit inputs")
        return f

    def evaluate(self, levers):
        ctx, st, P = self.ctx, self.ctx.state, self.P
        m = self.mask(levers)
        if levers:
            calls = []
            beta, rec, dn = lp.run_chain(ctx, st, self.seed, fitter=self.fitter(m, calls))
            if sorted(calls) != ["f", "r"]:
                raise SystemExit(f"STOP {self.key} {vname(levers)}: fitter calls {calls}")
        else:
            beta, rec, dn = self.beta_p, self.rec_p, self.dn_p
        zone_free = "Z" in levers
        v = {"levers": list(levers), "retained": float(m.sum() / ctx.base.sum()), "gate": rec["gate"],
             "status": rec["status"], "fallback": rec["fallback"], "health": rec["health"],
             "chatter_rms_deg": rec["chatter_rms_deg"], "axes": {}, "S_scored": not zone_free}
        v["G"] = bool(rec["status"] != "degenerate" and rec["gate"] == "pass" and not rec["fallback"] and
                      all(rec["axles"][AX[ax]]["powell_converged"] for ax in "fr"))
        v["M3"] = self.m3p if not levers else float(lp.m3_pair(beta, ctx)[0])
        v["B2_deg"] = float(np.median(np.abs(np.asarray(beta, float)[self.straight])) * RAD2DEG)
        screen_ok = True
        for ax in "fr":
            a, f = P[ax]
            full = ORIG_FIT(a, f, m)
            pars = [full[k] for k in "BCDE"]
            ok = bool(all(np.isfinite(pars)) and full["B"] >= 1 and 1 < full["C"] <= 3 and full["E"] <= 1)
            screen_ok &= ok
            v["G"] &= bool(full["sign_ok"])
            curves = []
            for lap in ctx.analysis_laps():
                fl = ORIG_FIT(a, f, m & ~ctx.lap_mask({lap["lap_number"]}))
                if fl["powell_converged"] and fl["sign_ok"]:
                    curves.append(ma.curve(fl, self.grids[ax]))
            if len(curves) >= 2:
                c = np.array(curves)
                f1 = float(np.median(np.std(c, axis=0, ddof=1) / np.abs(np.median(c, axis=0))))
            else:
                f1 = float("nan")
            cf = ma.curve(full, self.grids[ax])
            if zone_free:
                s = 0.0
            else:
                noz = ORIG_FIT(a, f, m & ~st["kerb_smear_zone"])
                s = float(np.median(np.abs(ma.curve(noz, self.grids[ax]) - cf) / cf))
            mm = ctx.base & np.isfinite(a) & np.isfinite(f)
            f2 = float(np.sqrt(np.mean((f[mm] - pacejka_lateral_force(a[mm], full["B"], full["C"], full["D"], full["E"])) ** 2)))
            cs = np.asarray(dn[f"cs_{ax}"], float)
            fin = np.isfinite(cs) & self.in_corner
            v["axes"][ax] = {"fit": {k: float(full[k]) for k in "BCDE"}, "screen_ok": ok,
                             "origin_slope_N_per_rad": float(full["B"] * full["C"] * full["D"]),
                             "F1": f1, "F1_n": len(curves), "F2_N": f2, "S": s,
                             "cs_neg": float(np.mean(cs[fin] < 0)), "cs_deep": float(np.mean(cs[fin] < -1)),
                             "n_neg": int(np.sum(cs[fin] < 0)), "n_deep": int(np.sum(cs[fin] < -1)),
                             "n_fin": int(fin.sum()), "coverage": float(fin.sum() / max(1, self.in_corner.sum()))}
        v["screen_ok"] = screen_ok
        return v


def judge(sessions, res, name):
    harm = []
    s_scored = res[sessions[0].key][name]["S_scored"]
    for s in sessions:
        p, m = res[s.key]["P"], res[s.key][name]
        if not m["G"]:
            harm.append(f"{s.key} G (gate {m['gate']}, status {m['status']}, fallback {m['fallback']})")
        if not m["screen_ok"]:
            bad = {ax: m["axes"][ax]["fit"] for ax in "fr" if not m["axes"][ax]["screen_ok"]}
            harm.append(f"{s.key} sanity screen {bad}")
        if m["M3"] > s.band[1]:
            harm.append(f"{s.key} M3 above band ({m['M3']:.3f} > {s.band[1]:.3f})")
        if m["B2_deg"] - p["B2_deg"] >= 0.1:
            harm.append(f"{s.key} straight beta +{m['B2_deg'] - p['B2_deg']:.3f} deg")
        for ax in "fr":
            pa, va = p["axes"][ax], m["axes"][ax]
            if (va["cs_neg"] - pa["cs_neg"]) * 100 >= HARM_CS_NEG_PT:
                harm.append(f"{s.key} {ax} CS<0 +{(va['cs_neg'] - pa['cs_neg']) * 100:.2f} pt")
            if (pa["coverage"] - va["coverage"]) * 100 >= 1.0:
                harm.append(f"{s.key} {ax} coverage -{(pa['coverage'] - va['coverage']) * 100:.2f} pt")
            if s_scored and va["S"] > 1.5 * pa["S"] and va["S"] - pa["S"] > 0.01:
                harm.append(f"{s.key} {ax} S {pa['S']:.4f} -> {va['S']:.4f}")
    cells = [(res[s.key]["P"]["axes"][ax], res[s.key][name]["axes"][ax]) for s in sessions for ax in "fr"]
    f1 = float(np.nanmedian([m["F1"] / p["F1"] - 1 for p, m in cells]))
    credit = [bool(res[s.key][name]["M3"] < s.band[0] and res[s.key][name]["M3"] <= 0.9 * res[s.key]["P"]["M3"])
              for s in sessions]
    pooled = lambda n, k: sum(res[s.key][n]["axes"][ax][k] for s in sessions for ax in "fr")
    deep = pooled(name, "n_deep") / pooled(name, "n_fin") / (pooled("P", "n_deep") / pooled("P", "n_fin")) - 1
    neg_share = pooled(name, "n_neg") / pooled(name, "n_fin")
    neg = neg_share / (pooled("P", "n_neg") / pooled("P", "n_fin")) - 1
    cand = {"F1 -10 %": (f1, f1 <= -0.10), "M3 credit on >= 2 of 3": (sum(credit), sum(credit) >= 2),
            "CS<-1 -10 % rel": (deep, deep <= -0.10), "CS<0 -5 % rel": (neg, neg <= -0.05)}
    if s_scored:
        sm = float(np.nanmedian([m["S"] / p["S"] - 1 for p, m in cells]))
        cand["S -25 %"] = (sm, sm <= -0.25)
    n_ok = sum(v[1] for v in cand.values())
    verdict = "HARM" if harm else ("EARNS" if n_ok >= 2 else "NEUTRAL")
    return {"verdict": verdict, "harm": harm, "candidates": {k: [v[0], bool(v[1])] for k, v in cand.items()},
            "met": n_ok, "reachable": len(cand), "share": n_ok / len(cand), "m3_credit": credit,
            "pooled_cs_neg": float(neg_share), "n_levers": name.count("+") + 1}


def render(sessions, res, names, tag=""):
    plt = lp._plt()
    cols = [lp.INK, lp.ACC, lp.ACC2, lp.REF, "#7a5195", "#ef5675", "#ffa600", "#2f4b7c"]
    for s in sessions:
        fig, axs = plt.subplots(1, 2, figsize=(13, 5))
        for k, ax in enumerate("fr"):
            A = axs[k]
            P = s.P
            m = s.ctx.base & np.isfinite(P[ax][0]) & np.isfinite(P[ax][1])
            sel = np.flatnonzero(m)[:: max(1, m.sum() // 15000)]
            A.scatter(P[ax][0][sel] * RAD2DEG, P[ax][1][sel] / 1000, s=1, color=lp.REF, alpha=0.3,
                      label="production fit population")
            gmax = s.grids[ax][-1] * 1.6
            g = np.linspace(-gmax, gmax, 300)
            for i, n in enumerate(names):
                fp = res[s.key][n]["axes"][ax]["fit"]
                A.plot(g * RAD2DEG, pacejka_lateral_force(g, fp["B"], fp["C"], fp["D"], fp["E"]) / 1000,
                       color=cols[i % len(cols)], lw=1.8 if n == "P" else 1.2,
                       label=f"{vdesc(n.split('+') if n != 'P' else [])}, origin slope "
                             f"{res[s.key][n]['axes'][ax]['origin_slope_N_per_rad'] / 1000:.0f} kN/rad")
            A.set_xlabel("slip angle [deg]")
            A.set_ylabel("axle lateral force [kN]")
            A.set_title(f"{AX[ax]} axle", fontsize=10)
            A.legend(fontsize=6, markerscale=6)
            lp._style(A)
        title = (f"{s.ctx.name}: tyre-curve fit, production vs the fit without the traction tail (a_x above "
                 f"{s.bounds['LX_g'][1]:+.2f} g removed, heavy braking kept)") if tag else None
        fig.suptitle(lp._wrap(title, fig) if title else lp._wrap(f"{s.ctx.name}: tyre-curve fit, production vs fit populations restricted by speed "
                              f"window ({s.bounds['SW_kmh'][0]:.0f}-{s.bounds['SW_kmh'][1]:.0f} km/h), smear-zone "
                              f"removal and large-Fx tails ({s.bounds['LX_g'][0]:+.2f} / {s.bounds['LX_g'][1]:+.2f} g)",
                              fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.90))
        fig.savefig(os.path.join(OUT_DIR, f"fit{tag}_{s.key}.png"), dpi=130)
        plt.close(fig)
    metrics = [("F1 curve instability (leave-one-lap-out)", lambda v, ax: v["axes"][ax]["F1"]),
               ("S strike sensitivity (0 by construction without the smear zone)", lambda v, ax: v["axes"][ax]["S"]),
               ("share of in-corner CS < 0", lambda v, ax: v["axes"][ax]["cs_neg"]),
               ("share of in-corner CS < -1", lambda v, ax: v["axes"][ax]["cs_deep"]),
               ("M3 sideslip lap-to-lap scatter [deg]", lambda v, ax: v["M3"])]
    fig, axs = plt.subplots(1, len(metrics), figsize=(19, 5.8))
    pairs = [(s, ax) for s in sessions for ax in "fr"]
    labels = [f"{s.ctx.name.split(',')[0]}, {s.ctx.name.split(',')[-1].strip()}\n{AX[ax]} axle" for s, ax in pairs]
    off = np.linspace(-0.3, 0.3, len(names))
    for k, (title, fn) in enumerate(metrics):
        A = axs[k]
        for j, n in enumerate(names):
            A.scatter(np.arange(len(pairs)) + off[j], [fn(res[s.key][n], ax) for s, ax in pairs], s=16,
                      color=cols[j % len(cols)], label=vdesc(n.split('+') if n != 'P' else []) if k == 0 else None)
        A.set_xticks(range(len(labels)))
        A.set_xticklabels(labels, rotation=55, ha="right", rotation_mode="anchor", fontsize=6.5)
        A.set_title(lp._wrap(title, fig), fontsize=8)
        lp._style(A)
    axs[0].legend(fontsize=6)
    fig.suptitle(("Traction-tail pass" if tag else "Decisive fit pass") +
                 ", Tier 1: scored metrics per fit population (lower is better)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT_DIR, f"metrics{tag}.png"), dpi=130)
    plt.close(fig)


def describe(s, res, n):
    v = res[s.key][n]
    out = [f"  {n} ({vdesc(v['levers'])}): retained {v['retained'] * 100:.1f} %; G {v['G']} (gate {v['gate']}, "
           f"fallback {v['fallback']}); screen {v['screen_ok']}; M3 {v['M3']:.4f}; straight |beta| {v['B2_deg']:.3f}; "
           f"chatter {v['chatter_rms_deg']:.4f}"]
    for ax in "fr":
        a = v["axes"][ax]
        out.append(f"     {AX[ax]}: B {a['fit']['B']:.3f} C {a['fit']['C']:.3f} D {a['fit']['D']:.0f} E {a['fit']['E']:.3f} "
                   f"(slope {a['origin_slope_N_per_rad'] / 1000:.1f} kN/rad); F1 {a['F1']:.4f} ({a['F1_n']}); "
                   f"F2 {a['F2_N']:.0f} N; S {a['S']:.4f}; CS<0 {a['cs_neg'] * 100:.2f} %, CS<-1 "
                   f"{a['cs_deep'] * 100:.3f} %, coverage {a['coverage'] * 100:.2f} %")
    return out


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    lines = ["Q-FIT-DECISIVE: fit-population levers SW / Z / LX and survivor combinations vs production"]
    sessions, res = [], {}
    for key in TIER1:
        s = Session(key)
        lines.append(f"\n[{key}] {s.ctx.name}: entry " + ", ".join(f"{k} {v:.1e}" for k, v in s.entry.items())
                     + (" -> PASS" if s.entry_ok else " -> FAIL, session stopped"))
        lines.append(f"  bounds: speed window {s.bounds['SW_kmh'][0]:.0f}-{s.bounds['SW_kmh'][1]:.0f} km/h (removes "
                     f"{s.bounds['SW_removed_low'] * 100:.2f} % low, {s.bounds['SW_removed_high'] * 100:.2f} % high); "
                     f"large-Fx tails a_x < {s.bounds['LX_g'][0]:+.2f} g or > {s.bounds['LX_g'][1]:+.2f} g "
                     f"(p{LX_PCT[0]} / p{LX_PCT[1]})")
        if not s.entry_ok:
            continue
        sessions.append(s)
        res[key] = {"P": s.evaluate(())}
        _log(f"[{key}] P done")
    if len(sessions) < len(TIER1):
        lines.append("\nEXIT: STOPPED -- an entry check failed; no verdict")
        text = "\n".join(lines)
        open(os.path.join(OUT_DIR, "summary.txt"), "w").write(text)
        _log(text)
        return
    verdicts = {}
    for lever in LEVERS:
        for s in sessions:
            res[s.key][lever] = s.evaluate((lever,))
            _log(f"[{s.key}] {lever} done")
        verdicts[lever] = judge(sessions, res, lever)
        _log(f"stage 1 {lever}: {verdicts[lever]['verdict']}")
    survivors = [l for l in LEVERS if verdicts[l]["verdict"] != "HARM"]
    combos = [c for r in range(2, len(survivors) + 1) for c in itertools.combinations(survivors, r)]
    for c in combos:
        n = vname(c)
        for s in sessions:
            res[s.key][n] = s.evaluate(c)
            _log(f"[{s.key}] {n} done")
        verdicts[n] = judge(sessions, res, n)
        _log(f"stage 2 {n}: {verdicts[n]['verdict']}")
    names = ["P"] + list(LEVERS) + [vname(c) for c in combos]
    for s in sessions:
        lines.append(f"\n[{s.key}] {s.ctx.name}; P's M3 band [{s.band[0]:.4f}, {s.band[1]:.4f}]")
        for n in names:
            lines += describe(s, res, n)
    lines.append(f"\nSTAGE 1 survivors (not HARM): {survivors or 'none'}; stage 2 combinations: "
                 f"{[vname(c) for c in combos] or 'none'}")
    for n, j in verdicts.items():
        lines.append(f"\n{n} ({vdesc(n.split('+'))}): VERDICT {j['verdict']}; candidates met {j['met']} of "
                     f"{j['reachable']} (share {j['share']:.2f}); pooled in-corner CS<0 {j['pooled_cs_neg'] * 100:.2f} %")
        lines.append(f"  HARM items: {j['harm'] or 'none'}")
        lines.append("  candidates: " + "; ".join(f"{k}: {v[0]:+.3f} -> {'yes' if v[1] else 'no'}"
                                                   if isinstance(v[0], float) else f"{k}: {v[0]} -> {'yes' if v[1] else 'no'}"
                                                   for k, v in j["candidates"].items()))
    earns = [n for n, j in verdicts.items() if j["verdict"] == "EARNS"]
    if earns:
        ship = sorted(earns, key=lambda n: (-verdicts[n]["share"], verdicts[n]["n_levers"], verdicts[n]["pooled_cs_neg"]))[0]
        lines.append(f"\nSHIPPING RULE: EARNS variants {earns} -> SHIPS: {ship} ({vdesc(ship.split('+'))})")
    else:
        lines.append("\nSHIPPING RULE: no variant earns -> nothing ships; production stays; the fit stays open")
    render(sessions, res, names)
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    out = {"bounds": {s.key: s.bounds for s in sessions}, "bands": {s.key: s.band for s in sessions},
           "variants": res, "verdicts": verdicts}
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=lp._jsonable)
    _log(text)


CS_LEVERS = ("SW", "Z", "LX", "LT")
CS_COLS = {"P": "#1f2937", "SW": "#2563eb", "Z": "#d97706", "LX": "#9ca3af", "LT": "#16a34a"}


def cs_compute():
    """Mode cs (plots only, author request 2026-10-04): the CS arrays behind the
    decisive and LT verdicts were not kept, so the chains are rerun per variant
    (no metrics) and cached for rendering."""
    os.makedirs(os.path.join(OUT_DIR, "cs_cache"), exist_ok=True)
    for key in TIER1:
        s = Session(key)
        ctx, st = s.ctx, s.ctx.state
        out = {"P_f": np.asarray(s.dn_p["cs_f"], float), "P_r": np.asarray(s.dn_p["cs_r"], float)}
        fits = {"P": {ax: {k: float(v) for k, v in ORIG_FIT(s.P[ax][0], s.P[ax][1], ctx.base).items() if k in "BCDE"}
                      for ax in "fr"}}
        for lever in CS_LEVERS:
            m = s.mask((lever,))
            calls = []
            _b, _rec, dn = lp.run_chain(ctx, st, s.seed, fitter=s.fitter(m, calls))
            out[f"{lever}_f"] = np.asarray(dn["cs_f"], float)
            out[f"{lever}_r"] = np.asarray(dn["cs_r"], float)
            fits[lever] = {ax: {k: float(v) for k, v in ORIG_FIT(s.P[ax][0], s.P[ax][1], m).items() if k in "BCDE"}
                           for ax in "fr"}
            _log(f"[{key}] {lever} chain done")
        np.savez_compressed(os.path.join(OUT_DIR, "cs_cache", f"{key}.npz"), **out)
        with open(os.path.join(OUT_DIR, "cs_cache", f"{key}_fits.json"), "w") as fh:
            json.dump({"fits": fits, "bounds": s.bounds}, fh, indent=1)
        cs_render(s, out, fits)


def cs_render(s, cs, fits):
    plt = lp._plt()
    ctx, st = s.ctx, s.ctx.state
    names = ["P"] + list(CS_LEVERS)
    desc = lambda n: vdesc([] if n == "P" else [n])
    laps = ctx.analysis_laps()
    lap = sorted(laps, key=lambda l: l["end_time"] - l["start_time"])[len(laps) // 2]
    lm = ctx.lap_mask({lap["lap_number"]})
    idx = np.flatnonzero(lm)
    idx = idx[np.argsort(st["s_m"][idx])]
    y_lo, y_hi = -3.5, 1.2
    fig, axs = plt.subplots(2, 1, figsize=(16, 8), sharex=True)
    for A, ax in zip(axs, "fr"):
        for _sid, lapn, sl in ctx.inst:
            if lapn == lap["lap_number"]:
                ss = st["s_m"][sl]
                A.axvspan(np.nanmin(ss), np.nanmax(ss), color="#e5e7eb", lw=0, zorder=0)
        A.axhline(0, color=lp.REF, lw=0.8)
        A.axhline(1, color=lp.REF, lw=0.8, ls="--")
        for n in names:
            A.plot(st["s_m"][idx], np.clip(cs[f"{n}_{ax}"][idx], y_lo, y_hi), color=CS_COLS[n],
                   lw=1.6 if n == "P" else 0.9, alpha=0.95 if n == "P" else 0.8, label=desc(n))
        A.set_ylim(y_lo, y_hi)
        A.set_ylabel(f"{AX[ax]} cornering-stiffness ratio CS")
        lp._style(A)
    axs[0].legend(fontsize=7, ncol=5, loc="lower left")
    axs[1].set_xlabel("lap distance [m] (grey = corner brackets)")
    fig.suptitle(lp._wrap(f"{ctx.name}, lap {lap['lap_number']} (median lap time): cornering-stiffness ratio over the "
                          f"lap, production fit vs the four tested fit populations (values below {y_lo} clipped)",
                          fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT_DIR, f"cs_trace_{s.key}.png"), dpi=130)
    plt.close(fig)
    bins = np.arange(-2.0, 1.05 + 1e-9, 0.05)
    fig, axs = plt.subplots(1, 2, figsize=(14, 5))
    for A, ax in zip(axs, "fr"):
        for n in names:
            c = cs[f"{n}_{ax}"]
            v = np.clip(c[np.isfinite(c) & s.in_corner], -2.0, 1.05)
            A.hist(v, bins=bins, histtype="step", color=CS_COLS[n], lw=1.6 if n == "P" else 1.0,
                   label=f"{desc(n)} (CS<0 {np.mean(v < 0) * 100:.1f} %)")
        A.axvline(0, color=lp.REF, lw=0.8)
        A.set_yscale("log")
        A.set_xlabel(f"{AX[ax]} CS (in-corner, analysis laps; below -2 collected in the left bin)")
        A.set_ylabel("samples")
        A.legend(fontsize=7)
        lp._style(A)
    fig.suptitle(lp._wrap(f"{ctx.name}: distribution of the in-corner cornering-stiffness ratio, production fit vs "
                          f"the four tested fit populations", fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT_DIR, f"cs_distribution_{s.key}.png"), dpi=130)
    plt.close(fig)
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
    for A, ax in zip(axs, "fr"):
        P = s.P
        m = ctx.base & np.isfinite(P[ax][0]) & np.isfinite(P[ax][1])
        sel = np.flatnonzero(m)[:: max(1, m.sum() // 15000)]
        A.scatter(P[ax][0][sel] * RAD2DEG, P[ax][1][sel] / 1000, s=1, color=lp.REF, alpha=0.3,
                  label="production fit population")
        gmax = s.grids[ax][-1] * 1.6
        g = np.linspace(-gmax, gmax, 300)
        for n in names:
            fp = fits[n][ax]
            A.plot(g * RAD2DEG, pacejka_lateral_force(g, fp["B"], fp["C"], fp["D"], fp["E"]) / 1000,
                   color=CS_COLS[n], lw=1.8 if n == "P" else 1.1,
                   label=f"{desc(n)}, origin slope {fp['B'] * fp['C'] * fp['D'] / 1000:.0f} kN/rad")
        A.set_xlabel("slip angle [deg]")
        A.set_ylabel("axle lateral force [kN]")
        A.set_title(f"{AX[ax]} axle", fontsize=10)
        A.legend(fontsize=6, markerscale=6)
        lp._style(A)
    fig.suptitle(lp._wrap(f"{ctx.name}: tyre-curve fits, production vs speed window, smear zone removed, large-Fx "
                          f"tails removed and traction tail removed", fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(os.path.join(OUT_DIR, f"fit_all_{s.key}.png"), dpi=130)
    plt.close(fig)


def main_lt():
    os.makedirs(OUT_DIR, exist_ok=True)
    lines = ["Q-FIT-DECISIVE mode lt: traction tail removed (heavy braking kept) vs production -- new frame, same bars"]
    sessions, res = [], {}
    for key in TIER1:
        s = Session(key)
        lines.append(f"\n[{key}] {s.ctx.name}: entry " + ", ".join(f"{k} {v:.1e}" for k, v in s.entry.items())
                     + (" -> PASS" if s.entry_ok else " -> FAIL, session stopped"))
        nb = s.ctx.base.sum()
        lines.append(f"  traction tail: a_x > {s.bounds['LX_g'][1]:+.2f} g (p{LX_PCT[1]}); retained "
                     f"{s.mask(('LT',)).sum() / nb * 100:.1f} %")
        if not s.entry_ok:
            continue
        sessions.append(s)
        res[key] = {"P": s.evaluate(()), "LT": s.evaluate(("LT",))}
        _log(f"[{key}] P + LT done")
    if len(sessions) < len(TIER1):
        lines.append("\nEXIT: STOPPED -- an entry check failed; no verdict")
        text = "\n".join(lines)
        open(os.path.join(OUT_DIR, "summary_lt.txt"), "w").write(text)
        _log(text)
        return
    j = judge(sessions, res, "LT")
    for s in sessions:
        lines.append(f"\n[{s.key}] {s.ctx.name}; P's M3 band [{s.band[0]:.4f}, {s.band[1]:.4f}]")
        for n in ("P", "LT"):
            lines += describe(s, res, n)
    lines.append(f"\nLT (traction tail removed): VERDICT {j['verdict']}; candidates met {j['met']} of {j['reachable']} "
                 f"(share {j['share']:.2f}); pooled in-corner CS<0 {j['pooled_cs_neg'] * 100:.2f} %")
    lines.append(f"  HARM items: {j['harm'] or 'none'}")
    lines.append("  candidates: " + "; ".join(f"{k}: {v[0]:+.3f} -> {'yes' if v[1] else 'no'}"
                                               if isinstance(v[0], float) else f"{k}: {v[0]} -> {'yes' if v[1] else 'no'}"
                                               for k, v in j["candidates"].items()))
    lines.append("\nSHIPPING RULE: " + ("LT EARNS -> SHIPS (traction-tail percentile in config)" if j["verdict"] == "EARNS"
                                        else "LT does not earn -> nothing ships; production stays; the fit stays open"))
    render(sessions, res, ["P", "LT"], tag="_lt")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary_lt.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "results_lt.json"), "w") as fh:
        json.dump({"bounds": {s.key: s.bounds for s in sessions}, "bands": {s.key: s.band for s in sessions},
                   "variants": res, "verdict": j}, fh, indent=1, default=lp._jsonable)
    _log(text)


if __name__ == "__main__":
    if sys.argv[1:] == ["lt"]:
        main_lt()
    elif sys.argv[1:] == ["cs"]:
        cs_compute()
    else:
        main()
