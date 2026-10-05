# Q-MU -- measured tyre forces in the evaluation: the axle utilisation trace
# mu = |Fy_axle| / Fz_axle on the 16 NBR files (PLAN.md STATUS 2026-10-05,
# author order, read-only; EXECUTE granted with the session order).
#
# RULES, frozen 2026-10-05 BEFORE any run (conflict = record, never re-tune).
# SESSIONS: the 16 NBR files of inspect_pool_fits.NBR16. Plot session: JST
#   outing 29 (nbr_jst_solution). Driver-statement session: JST outing 21
#   (nbr_jst_problem, thesis_notes "Driver statement (iii), NBR outing A").
# CHAIN: production state, parameters and beta from the d2 caches (lp.Ctx,
#   best available). The cached parsed data predate the Q-CAL-2 zero entries
#   (2026-10-03), so every file is parsed FRESH with the current parse_csv and
#   the fresh channels feed estimate_vertical_loads (the app's own call).
# ENTRY CHECKS per file, a failure excludes the file and the run goes on:
#   E1 fresh parse matched event_zero_entries NBR_221, all four damper
#      channels "applied";
#   E2 fresh state time grid identical to the cached one and the axle Fy
#      identical (max |diff| <= 1e-9 N);
#   E3 CS recomputed from the cached production beta reproduces the cached CS
#      (same finite sets, max |diff| <= 1e-5, float32 cache; pool_fits rule);
#   E4 vertical_load_source_used == "measured".
# QUANTITY: mu_a = |Fy_a_filt| / LP(Fz_a), a = front / rear axle. Fy_a_filt is
#   the production Level-1 axle split through the 2 Hz Butterworth; Fz_a the
#   measured axle total (damper cascade) through the SAME production low-pass
#   (cs_filter_cutoff_hz), so both sides carry the same bandwidth. Samples
#   with LP(Fz_a) <= 0 are undefined and counted, never filled. Production's
#   own fy_*_norm_N divides by UNFILTERED Fz; reported beside (p50 / p90) as a
#   sensitivity line only.
# POPULATION: inside corner brackets on analysis laps (valid_corner_instances),
#   moving, outside the kerb mask. GREY = samples within kerb_smear_grade_s of
#   a kerb-mask sample (production _kerb_smear_zone). The shipped CS grade mark
#   is window-based; mu has no regression window, so the zone itself is the
#   pointwise reading of the same smear (labelled adaptation, display rule:
#   greyed, kept). Statistics: CLEAN (grey removed) primary, FULL beside, per
#   the 2026-10-05 population decision.
# SATURATION SITES: the position-locked CS dip sites of JST 29 (stiffness_cs/
#   spike_nbr_jst_solution.json, production variant, known_real), front and
#   rear. Site samples = population within +/-10 m lap distance of the site
#   (Q-PAIRING radius). mu of the site's axle at the sites vs the rest of the
#   population. Other NBR files: the same positions TRANSFERRED (same track;
#   labelled, secondary).
# VERDICT SPOT CHECK: production corner verdicts (summarise_corners on the
#   recomputed CS and stability, aggregate_by_corner, the app classifier);
#   class = primary word of the verdict (understeer / oversteer / none). Per
#   corner, clean in-corner medians of D = mu_f - mu_r and R = mu_f / mu_r.
#   PRE-STATED BY CONSTRUCTION (Milliken RCVD ch. 5, 2-DOF balance): with
#   Fy_f = m ay ff + Iz psidd / L and Fy_r = m ay - Fy_f, R = R_load * R_yaw
#   with R_load = (ff / rf) * (Fz_r / Fz_f) (what the static split alone gives)
#   and R_yaw the yaw-acceleration remainder. In quasi-steady cornering
#   R_yaw ~ 1, so front-vs-rear mu can reflect the LOAD distribution and
#   transients, not which tyre saturates. Both factors reported per class.
#   SEPARATION: sign test over files holding both classes -- count of files
#   with median D(understeer) > median D(oversteer), two-sided binomial p;
#   SEPARATES if p < 0.05, else NOT SHOWN.
#   DRIVER STATEMENT (JST 21): per phase (entry = entry_1 + entry_2, mid = apex
#   +/- apex_half_window_samples, exit = exit_4 + exit_5) clean medians of
#   D, R_load, R_yaw, JST 21 beside JST 29; descriptive only.
# PLOTS (plain words, render title rule): one per axle -- mu over the median-
#   time analysis lap of JST 29, grey overlay, kerb bands shaded; one
#   histogram per axle pooled over the 16 files, clean + grey stacked.
#
# Modes: run [keys...] | report | plots     Output: diagnostics/plots/mu_trace/

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp
from diagnostics.inspect_pool_fits import NBR16

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "mu_trace")
PLOT_KEY = "nbr_jst_solution"
STATEMENT_KEY = "nbr_jst_problem"
SITE_RADIUS_M = 10.0
AX = {"f": "front", "r": "rear"}
PHASES = {"entry": ("entry_1_brake", "entry_2_turnin"), "exit": ("exit_4", "exit_5")}


def _log(msg):
    os.makedirs(OUT_DIR, exist_ok=True)
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, "run_log.txt"), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def _pct(x, q):
    return float(np.percentile(x, q)) if x.size else float("nan")


def _stats(x):
    return {"n": int(x.size), "p50": _pct(x, 50), "p90": _pct(x, 90), "p99": _pct(x, 99)}


def _sites():
    with open(os.path.join(os.path.dirname(OUT_DIR), "stiffness_cs", f"spike_{PLOT_KEY}.json")) as fh:
        prod = json.load(fh)["variants"]["production"]
    return {ax: [d["pos_m"] for d in prod[ax]["dips"] if d.get("known_real")] for ax in "fr"}


def _classify(agg):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, agg)


def run_one(k):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (prepare_vehicle_state, estimate_lateral_forces, estimate_slip_angles,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            estimate_vertical_loads, summarise_corners, load_car_data,
                                            _butterworth_lowpass, _kerb_smear_zone, _seconds_to_samples)
    from modules.recommendation import aggregate_by_corner
    t0 = time.perf_counter()
    ctx = lp.Ctx(k)
    st, prm = ctx.state, ctx.params
    se = prm["stability_estimation"]
    fresh = parse_csv(d2.sessions()[k]["path"])
    problems = []
    ez = fresh.get("event_zero_entry") or {}
    if ez.get("event") != "NBR_221" or sorted(ez.get("channels", {}).values()) != ["applied"] * 4:
        problems.append(f"E1 zero entry {ez}")
    st_fresh = prepare_vehicle_state(fresh["channels"], prm)
    forces = estimate_lateral_forces(st, prm)
    if not np.array_equal(st_fresh["time"], st["time"]):
        problems.append("E2 time grid differs")
    else:
        ff = estimate_lateral_forces(st_fresh, prm)
        dmax = max(float(np.nanmax(np.abs(ff[f"Fy_{a}_filt"] - forces[f"Fy_{a}_filt"]))) for a in "fr")
        if dmax > 1e-9:
            problems.append(f"E2 Fy differs by {dmax:.2e} N")
    rec = ctx.cache
    beta = np.asarray(rec["prod"]["beta"], float)
    cs = estimate_cornering_stiffness(estimate_slip_angles(st, beta, prm), forces, st, prm)
    for a in "fr":
        x, y = np.asarray(cs[f"CS_ratio_{a}"], float), np.asarray(rec["prod"]["down"][f"cs_{a}"], float)
        fin = np.isfinite(x) & np.isfinite(y)
        diff = float(np.max(np.abs(x[fin] - y[fin]))) if fin.any() else 0.0
        if not (np.array_equal(np.isfinite(x), np.isfinite(y)) and diff <= 1e-5):
            problems.append(f"E3 CS {AX[a]} max diff {diff:.2e}")
    fz = estimate_vertical_loads(st, forces, prm, channels=fresh["channels"], car_data=load_car_data())
    if fz["vertical_load_source_used"] != "measured":
        problems.append(f"E4 load source {fz['vertical_load_source_used']}")
    if problems:
        _log(f"[{k}] EXCLUDED: {'; '.join(problems)}")
        return None

    sr = st["sample_rate_hz"]
    cut = se["cs_filter_cutoff_hz"]
    kerb = np.asarray(st["kerb_mask"], bool)
    grey = _kerb_smear_zone(kerb, _seconds_to_samples(se["kerb_smear_grade_s"], sr))
    pop = ctx.in_br & st["moving_mask"] & ~kerb
    fz_lp = {a: _butterworth_lowpass(np.asarray(fz[f"fz_{a}_N"], float), cut, sr) for a in "fr"}
    fy = {a: np.abs(forces[f"Fy_{a}_filt"]) for a in "fr"}
    undefined = {a: int((pop & ~(fz_lp[a] > 0)).sum()) for a in "fr"}
    ok = pop & (fz_lp["f"] > 0) & (fz_lp["r"] > 0)
    mu = {a: np.where(fz_lp[a] > 0, fy[a] / np.where(fz_lp[a] > 0, fz_lp[a], 1.0), np.nan) for a in "fr"}
    mu_raw = {a: fy[a] / np.asarray(fz[f"fz_{a}_N"], float) for a in "fr"}
    clean = ok & ~grey
    vf = forces["front_fraction"] / forces["rear_fraction"]
    R = mu["f"] / mu["r"]
    R_load = vf * fz_lp["r"] / fz_lp["f"]
    D = mu["f"] - mu["r"]

    out = {"key": k, "title": lp.title_of(k), "n_pop": int(ok.sum()), "n_clean": int(clean.sum()),
           "grey_share": float((ok & grey).sum() / max(1, ok.sum())), "undefined_fz": undefined,
           "source_share": {c: {s: float(np.mean(np.asarray(fz["vertical_load_source_per_sample"][c])[ok] == s))
                                for s in ("damper", "reconstructed", "static_fallback")}
                            for c in ("fl", "fr", "rl", "rr")},
           "c_session": fz["c_session_N_per_mps2"], "zero_entry": ez}
    for a in "fr":
        out[f"mu_{a}_clean"] = _stats(mu[a][clean])
        out[f"mu_{a}_full"] = _stats(mu[a][ok])
        out[f"mu_{a}_unfiltered_fz_clean"] = _stats(mu_raw[a][clean & np.isfinite(mu_raw[a])])

    # saturation sites (JST 29 positions)
    s_m = st["s_m"]
    sites = _sites()
    out["sites"] = {}
    for a in "fr":
        near = np.zeros(s_m.size, bool)
        for p in sites[a]:
            near |= np.abs(s_m - p) <= SITE_RADIUS_M
        out["sites"][a] = {"site_clean": _stats(mu[a][clean & near]), "rest_clean": _stats(mu[a][clean & ~near]),
                           "site_full": _stats(mu[a][ok & near]), "rest_full": _stats(mu[a][ok & ~near]),
                           "site_grey_share": float((ok & near & grey).sum() / max(1, (ok & near).sum()))}

    # verdicts: production chain summaries, app classifier
    laps = ctx.analysis_laps()
    stab = estimate_yaw_moment_stability(st, beta, prm, ctx.data["laps"])
    summ = summarise_corners(ctx.data["corners"], cs, stab, st, fz=fz, lap_filter=[l["lap_number"] for l in laps])
    agg = aggregate_by_corner(summ)
    by_sid = {}
    for sid, _lap, sl in ctx.inst:
        m = np.zeros(s_m.size, bool)
        m[sl] = True
        by_sid.setdefault(sid, np.zeros(s_m.size, bool))
        by_sid[sid] |= m
    out["corners"] = []
    for g in agg.values():
        sid = g["stable_corner_id"]
        _sev, short, _long, _c = _classify(g)
        word = short.split(" @")[0].split(" |")[0].strip()
        cls = word if word in ("understeer", "oversteer") else "none"
        m = by_sid.get(sid, np.zeros(s_m.size, bool)) & clean
        out["corners"].append({"sid": sid, "verdict": short, "class": cls, "n": int(m.sum()),
                               "D": _pct(D[m], 50), "R": _pct(R[m], 50), "R_load": _pct(R_load[m], 50),
                               "R_yaw": _pct((R / R_load)[m], 50)})

    # per phase (driver statement frame)
    half = se["apex_half_window_samples"] / sr
    t = st["time"]
    lapset = {l["lap_number"] for l in laps}
    ph = {p: np.zeros(t.size, bool) for p in ("entry", "mid", "exit")}
    for c in ctx.data["corners"]:
        if c["lap_number"] not in lapset or c.get("stable_corner_id") is None:
            continue
        seg = c["segments"]
        for p, keys in PHASES.items():
            ph[p] |= (t >= seg[keys[0]][0]) & (t <= seg[keys[1]][1])
        ph["mid"] |= np.abs(t - c["apex_time"]) <= half
    out["phases"] = {p: {"n": int((m & clean).sum()), "D": _pct(D[m & clean], 50),
                         "R_load": _pct(R_load[m & clean], 50), "R_yaw": _pct((R / R_load)[m & clean], 50),
                         "mu_f": _pct(mu["f"][m & clean], 50), "mu_r": _pct(mu["r"][m & clean], 50)}
                     for p, m in ph.items()}

    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez_compressed(os.path.join(OUT_DIR, f"arrays_{k}.npz"),
                        mu_f_clean=mu["f"][clean], mu_r_clean=mu["r"][clean],
                        mu_f_grey=mu["f"][ok & grey], mu_r_grey=mu["r"][ok & grey])
    if k == PLOT_KEY:
        np.savez_compressed(os.path.join(OUT_DIR, f"trace_{k}.npz"), t=t, s_m=s_m, mu_f=mu["f"], mu_r=mu["r"],
                            kerb=kerb, grey=grey, ok=ok, in_br=ctx.in_br,
                            laps=np.array([[l["lap_number"], l["start_time"], l["end_time"]] for l in laps]))
    with open(os.path.join(OUT_DIR, f"result_{k}.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)
    _log(f"[{k}] ok, pop {out['n_pop']}, grey {out['grey_share'] * 100:.1f} %, mu clean p50 f "
         f"{out['mu_f_clean']['p50']:.3f} r {out['mu_r_clean']['p50']:.3f}, {time.perf_counter() - t0:.0f} s")
    return out


def run(keys):
    for k in keys or NBR16:
        try:
            run_one(k)
        except Exception as e:  # one file failing never stops the run
            _log(f"[{k}] EXCEPTION {type(e).__name__}: {e}")


def _results():
    out = {}
    for k in NBR16:
        p = os.path.join(OUT_DIR, f"result_{k}.json")
        if os.path.exists(p):
            with open(p) as fh:
                out[k] = json.load(fh)
    return out


def report():
    from scipy.stats import binomtest
    R = _results()
    lines = [f"files with results: {len(R)} / {len(NBR16)}"]
    for a in "fr":
        cl = np.concatenate([np.load(os.path.join(OUT_DIR, f"arrays_{k}.npz"))[f"mu_{a}_clean"] for k in R])
        gr = np.concatenate([np.load(os.path.join(OUT_DIR, f"arrays_{k}.npz"))[f"mu_{a}_grey"] for k in R])
        fu = np.concatenate([cl, gr])
        lines.append(f"POOLED {AX[a]}: clean n {cl.size} p50 {_pct(cl, 50):.3f} p90 {_pct(cl, 90):.3f} p99 "
                     f"{_pct(cl, 99):.3f} | full p50 {_pct(fu, 50):.3f} p90 {_pct(fu, 90):.3f} p99 {_pct(fu, 99):.3f}"
                     f" | grey p50 {_pct(gr, 50):.3f} p99 {_pct(gr, 99):.3f}")
    for k, r in R.items():
        lines.append(f"{r['title']}: n {r['n_pop']}, grey {r['grey_share'] * 100:.1f} %, undefined Fz {r['undefined_fz']}"
                     f", c_session {r['c_session']}")
        for a in "fr":
            c, u = r[f"mu_{a}_clean"], r[f"mu_{a}_unfiltered_fz_clean"]
            lines.append(f"   {AX[a]:5s} clean p50 {c['p50']:.3f} p90 {c['p90']:.3f} p99 {c['p99']:.3f}; "
                         f"unfiltered-Fz p50 {u['p50']:.3f} p90 {u['p90']:.3f}")
        lines.append("   load source shares in population: " + "; ".join(
            f"{c} " + "/".join(f"{v * 100:.0f}" for v in r["source_share"][c].values()) for c in r["source_share"])
            + " (damper/reconstructed/static fallback %)")
    lines.append("SITES (JST 29 positions; first row = JST 29 itself):")
    for k in [PLOT_KEY] + [x for x in R if x != PLOT_KEY]:
        if k not in R:
            continue
        s = R[k]["sites"]
        lines.append(f"  {R[k]['title']}: " + "; ".join(
            f"{AX[a]} site p50 {s[a]['site_clean']['p50']:.3f} (n {s[a]['site_clean']['n']}) vs rest "
            f"{s[a]['rest_clean']['p50']:.3f}, site p90 {s[a]['site_clean']['p90']:.3f} vs "
            f"{s[a]['rest_clean']['p90']:.3f}, site grey {s[a]['site_grey_share'] * 100:.0f} %" for a in "fr"))
    lines.append("VERDICT CLASSES (corner medians, clean):")
    wins = n_both = 0
    pool = {"understeer": [], "oversteer": [], "none": []}
    for k, r in R.items():
        by = {c: [x for x in r["corners"] if x["class"] == c and np.isfinite(x["D"])] for c in pool}
        for c in pool:
            pool[c] += by[c]
        if by["understeer"] and by["oversteer"]:
            n_both += 1
            mu_ = np.median([x["D"] for x in by["understeer"]])
            mo_ = np.median([x["D"] for x in by["oversteer"]])
            wins += int(mu_ > mo_)
        lines.append(f"  {r['title']}: " + ", ".join(f"{c} {len(by[c])}" for c in pool))
    for c, xs in pool.items():
        if xs:
            lines.append(f"  POOLED {c}: corners {len(xs)}, D p50 {np.median([x['D'] for x in xs]):+.3f}, R p50 "
                         f"{np.median([x['R'] for x in xs]):.3f}, R_load p50 {np.median([x['R_load'] for x in xs]):.3f}"
                         f", R_yaw p50 {np.median([x['R_yaw'] for x in xs]):.3f}")
    if n_both:
        p = binomtest(wins, n_both, 0.5).pvalue
        lines.append(f"  SIGN TEST: D(understeer) > D(oversteer) in {wins} / {n_both} files, p {p:.3f} -> "
                     f"{'SEPARATES' if p < 0.05 else 'NOT SHOWN'}")
    else:
        lines.append("  SIGN TEST: no file holds both classes -> NOT EVALUABLE")
    lines.append("PHASES (JST 21 statement frame; JST 29 beside):")
    for k in (STATEMENT_KEY, PLOT_KEY):
        if k in R:
            lines.append(f"  {R[k]['title']}: " + "; ".join(
                f"{p} n {v['n']} mu_f {v['mu_f']:.3f} mu_r {v['mu_r']:.3f} D {v['D']:+.3f} R_load {v['R_load']:.3f} "
                f"R_yaw {v['R_yaw']:.3f}" for p, v in R[k]["phases"].items()))
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT_DIR, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")


def plots():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from core.plot_style import NEAR_STRIKE_COLOR, THRESHOLD_GREY
    from modules.stability_analysis import load_parameters
    from ui.style import INFO, BAD
    col = {"f": INFO, "r": BAD}
    grade = load_parameters()["stability_estimation"]["kerb_smear_grade_s"]
    grey_label = "near a kerb strike (kept in all results)"
    cap = (f"Grey: samples within {grade:g} s of a masked kerb strike (strike energy smeared by the 2 Hz filter); "
           "marked lower-quality, retained in all computed values.")
    qty = "lateral force over measured vertical load (Fy / Fz)"
    title = lp.title_of(PLOT_KEY)
    z = np.load(os.path.join(OUT_DIR, f"trace_{PLOT_KEY}.npz"))
    laps = z["laps"]
    lt = laps[:, 2] - laps[:, 1]
    ln, a0, b0 = laps[int(np.argsort(lt)[len(lt) // 2])]
    m = (z["t"] >= a0) & (z["t"] <= b0)
    s = z["s_m"][m]
    for a in "fr":
        y = np.where(z["ok"][m] & z["in_br"][m], z[f"mu_{a}"][m], np.nan)
        main = np.where(z["grey"][m], np.nan, y)
        over = np.where(z["grey"][m] | np.r_[z["grey"][m][1:], False] | np.r_[False, z["grey"][m][:-1]], y, np.nan)
        fig, ax = plt.subplots(figsize=(15, 4.6))
        ax.plot(s, main, color=col[a], lw=1.2, label=f"{AX[a]} axle, inside corners")
        ax.plot(s, over, color=NEAR_STRIKE_COLOR, lw=1.2, label=grey_label)
        kb = z["kerb"][m].astype(int)
        e = np.flatnonzero(np.diff(np.r_[0, kb, 0]))
        for j, (p, q) in enumerate(zip(e[::2], e[1::2])):
            ax.axvspan(s[p], s[min(q, len(s) - 1)], color=NEAR_STRIKE_COLOR, alpha=0.18, lw=0,
                       label="kerb strike, masked (no value)" if j == 0 else None)
        ax.set_xlabel("lap distance [m]")
        ax.set_ylabel(f"{AX[a]} axle: Fy / Fz [-]")
        ax.grid(alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)
        fig.suptitle(f"{title}: {AX[a]} axle {qty} over the median-time lap (lap {int(ln)}); "
                     "estimated lateral force, measured load", fontsize=11)
        fig.tight_layout(rect=(0, 0.05, 1, 1))
        fig.text(0.5, 0.01, cap, ha="center", va="bottom", fontsize=9, wrap=True)
        fig.savefig(os.path.join(OUT_DIR, f"mu_lap_{a}_{PLOT_KEY}.png"), dpi=120)
        plt.close(fig)
    R = _results()
    for a in "fr":
        cl = np.concatenate([np.load(os.path.join(OUT_DIR, f"arrays_{k}.npz"))[f"mu_{a}_clean"] for k in R])
        gr = np.concatenate([np.load(os.path.join(OUT_DIR, f"arrays_{k}.npz"))[f"mu_{a}_grey"] for k in R])
        bins = np.linspace(0, 2.5, 101)
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.hist([np.clip(cl, 0, 2.5), np.clip(gr, 0, 2.5)], bins=bins, stacked=True,
                color=[col[a], NEAR_STRIKE_COLOR], label=["clean samples", grey_label])
        for q, ls in ((50, "-"), (90, "--"), (99, ":")):
            ax.axvline(_pct(cl, q), color=THRESHOLD_GREY, lw=0.9, ls=ls, label=f"clean {q}th percentile {_pct(cl, q):.2f}")
        ax.set_xlabel(f"{AX[a]} axle: Fy / Fz inside corners [-] (above 2.5 piled at 2.5)")
        ax.set_ylabel("samples")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
        fig.suptitle(f"Nurburgring, {len(R)} outings pooled: {AX[a]} axle {qty}", fontsize=11)
        fig.tight_layout(rect=(0, 0.06, 1, 1))
        fig.text(0.5, 0.01, cap, ha="center", va="bottom", fontsize=8, wrap=True)
        fig.savefig(os.path.join(OUT_DIR, f"mu_hist_{a}_nbr16.png"), dpi=120)
        plt.close(fig)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "run"
    {"run": lambda: run(sys.argv[2:]), "report": report, "plots": plots}[mode]()
