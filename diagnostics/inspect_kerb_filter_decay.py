# Q-CAL-1 MEASUREMENT -- how far a kerb strike leaks past the kerb-mask edge
# through the CS chain's 2 Hz zero-phase low-pass (PLAN.md Q-CAL item 1,
# re-scoped by the author 2026-10-03 to the filter-smear path; raw-envelope
# extensions stay closed per the 2026-10-01 kerb-variant rulings).
# Read-only: nothing in modules/, core/ or config/ changes. Tier B
# (preprocessing measurement), no anchor claimed.
#
# WHY: production filters Fy and alpha over the whole series (strike samples
# included) and applies the kerb mask afterwards, as an endpoint exclusion
# only. A 4th-order 2 Hz filtfilt spreads a single sample over roughly
# +/-0.2 s (20 % of peak) to +/-0.4 s (10 %), so the strike reaches unmasked
# neighbours that the CS windows regress.
#
# RULES, frozen 2026-10-03 BEFORE the first run (author EXECUTE with rulings
# OPEN 1-3: null-control band; both sides counted, post the headline; Paul
# Ricard pr_us_v2 and v3 included).
# SESSIONS: the 16 NBR files with a cached production fit, plus pr_us_v2 and
#   v3. State = production prepare_vehicle_state on the cached data, so the
#   kerb mask is production's (|az - 1 g| > 1.2 g, +/-5 samples).
# INPUTS: raw and filtered Fy from production estimate_lateral_forces; raw and
#   filtered alpha from estimate_slip_angles with the cached production beta;
#   the filter is production _butterworth_lowpass, imported.
# ENTRY CHECK: production estimate_cornering_stiffness on these inputs must
#   reproduce the cached production CS (fit cache down.cs_f / cs_r, float32)
#   exactly, else the session stops. [Amended 2026-10-03 after the first
#   smoke run, rounding only; finite set must be identical: pass = identical
#   finite-sample set AND max |difference| <= 1e-6. The cached beta is
#   float32, so a recomputation can only match to its resolution (JST 29:
#   max 2.4e-7, same finite set). Author ruling.]
# EVENT: a contiguous production-mask run touching a racing sample (moving,
#   analysis lap). Runs are not merged.
# SMEAR: d = filt(x_raw) - filt(x_raw with EVERY mask run replaced by the
#   straight line between the samples either side of it), for x in Fy_f,
#   Fy_r, alpha_f, alpha_r. d is what the masked samples push through the
#   filter into their neighbours.
# NOISE BAND (null control): one pseudo-run per event, same length, random
#   start inside a corner instance on a racing sample, the whole run >= 2 s
#   from any mask sample and from other pseudo-runs, seed SEED. Same operator
#   (only the pseudo-runs substituted). Band = p95 of |d_null| over the samples
#   0-1 s beyond both pseudo-edges, per session and quantity.
# DECAY: walk forward from the mask end; the zone continues while the next
#   |d| > band sample lies within GAP_S of the previous one (bridges the
#   filter's ringing zero crossings); decay = last connected exceedance - mask
#   end (0 if none within GAP_S). Reaching the next mask run while the zone is
#   still active, or CAP_S, -> censored (counted). Mirror rule before the
#   onset. Post = headline, pre reported.
# CS WINDOWS: for every sample with a finite production CS per axle, window
#   [start, i) rebuilt with reconstruct_cs_window_start; shares of windows
#   holding >= 1 decay-zone sample (zone = pre or post, outside the mask, of
#   that axle's quantity), of windows whose endpoint i is in the zone, and of
#   windows holding >= 1 masked sample (comparison). All finite CS and the
#   in-corner subset.
# PLOTS: one per axle, pooled over all sessions: worst case (longest
#   uncensored Fy post decay) and median case (Fy post decay nearest the
#   pooled p50, first in session/time order); rows Fy, alpha, |d| / band.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_kerb_filter_decay [key ...]
# (no keys = all sessions above)
#        python -m diagnostics.inspect_kerb_filter_decay coincidence   (ruling 5, reads the records)
#        python -m diagnostics.inspect_kerb_filter_decay sites         (site check, reads the records)
#        python -m diagnostics.inspect_kerb_filter_decay verify        (options 4 + 0 check, Tier 1)

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/kerb_filter_decay"
KEYS = ["nbr_jst_problem", "nbr_jst_solution", "nbr_o005_mla", "nbr_o010_jdr", "nbr_o018_jst",
        "nbr_o019_jst", "nbr_o020_jst", "nbr_o046_jst", "nbr_o053_mla", "nbr_o054_cdr",
        "nbr_o059_mla", "nbr_o061_jdr", "nbr_o063_cdr", "nbr_o064_jst", "nbr_qp_cdr", "nbr_qp_mla",
        "pr_us_v2", "v3"]
QTY = ("fy_f", "fy_r", "al_f", "al_r")
SEED = 20261003
GAP_S = 0.10
CAP_S = 1.5
NULL_MIN_DIST_S = 2.0
BAND_SPAN_S = 1.0
BAND_PCTL = 95
ENTRY_TOL = 1e-6


def _log(m):
    print(m, flush=True)


def runs_of(mask):
    m = np.concatenate(([False], mask, [False])).astype(np.int8)
    d = np.diff(m)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))  # inclusive ends


def substitute(x, runs):
    y = x.copy()
    n = x.size
    for a, b in runs:
        lo, hi = a - 1, b + 1
        if lo < 0 and hi >= n:
            continue
        if lo < 0:
            y[a:b + 1] = x[hi]
        elif hi >= n:
            y[a:b + 1] = x[lo]
        else:
            y[a:b + 1] = np.interp(np.arange(a, b + 1), [lo, hi], [x[lo], x[hi]])
    return y


def walk(d_abs, band, mask, edge, step, gap, cap):
    """Connected-exceedance walk from a run edge; step +1 = after, -1 = before.
    Returns (decay_samples, censor) with censor None / 'neighbour' / 'cap'."""
    n = d_abs.size
    last = edge
    j = edge + step
    while 0 <= j < n:
        if abs(j - last) > gap:
            return abs(last - edge), None
        if abs(j - edge) > cap:
            return abs(last - edge), "cap"
        if mask[j]:
            return abs(last - edge), "neighbour"
        if d_abs[j] > band:
            last = j
        j += step
    return abs(last - edge), "cap"


def place_null(runs, racing, in_corner, mask, sr, rng):
    n = mask.size
    min_d = int(round(NULL_MIN_DIST_S * sr))
    blocked = mask.copy()
    # dilate the mask by min_d: a pseudo-run must sit >= 2 s from any mask sample
    idx = np.flatnonzero(mask)
    near = np.zeros(n, dtype=bool)
    if idx.size:
        cs = np.cumsum(np.concatenate(([0], mask.astype(int))))
        lo = np.clip(np.arange(n) - min_d, 0, n)
        hi = np.clip(np.arange(n) + min_d + 1, 0, n)
        near = (cs[hi] - cs[lo]) > 0
    ok = racing & in_corner & ~near & ~blocked
    pseudo = []
    taken = np.zeros(n, dtype=bool)
    for a, b in runs:
        L = b - a + 1
        cand = np.flatnonzero(ok)
        rng.shuffle(cand)
        placed = False
        for p in cand[:5000]:
            q = p + L - 1
            if q >= n or not ok[p:q + 1].all():
                continue
            lo, hi = max(0, p - min_d), min(n, q + min_d + 1)
            if taken[lo:hi].any():
                continue
            taken[p:q + 1] = True
            pseudo.append((p, q))
            placed = True
            break
        if not placed:
            pseudo.append(None)
    return pseudo


def session(key):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, _butterworth_lowpass,
                                            reconstruct_cs_window_start, resolve_cs_min_window_samples)
    ctx = lp.Ctx(key)
    st, params = ctx.state, ctx.params
    se = params["stability_estimation"]
    sr = st["sample_rate_hz"]
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    slip = estimate_slip_angles(st, beta, params)
    forces = estimate_lateral_forces(st, params)
    cs = estimate_cornering_stiffness(slip, forces, st, params)
    down = ctx.cache["prod"]["down"]
    entry = {}
    for ax in ("f", "r"):
        new = cs[f"CS_ratio_{ax}"]
        old = np.asarray(down[f"cs_{ax}"], dtype=float)
        same_set = bool(np.array_equal(np.isfinite(new), np.isfinite(old)))
        fin = np.isfinite(new) & np.isfinite(old)
        maxd = float(np.max(np.abs(new[fin] - old[fin]))) if fin.any() else 0.0
        entry[ax] = same_set and maxd <= ENTRY_TOL
        _log(f"[{key}] entry {ax}: same finite set {same_set}, max diff {maxd:.2e}")
    if not all(entry.values()):
        _log(f"[{key}] ENTRY CHECK FAILED {entry} -- session stopped")
        return None, None

    mask = st["kerb_mask"].astype(bool)
    racing, in_corner = ctx.racing, ctx.in_br
    cut = se["cs_filter_cutoff_hz"]
    raw = {"fy_f": forces["Fy_f_raw"], "fy_r": forces["Fy_r_raw"],
           "al_f": slip["alpha_f_raw"], "al_r": slip["alpha_r_raw"]}
    filt = {"fy_f": forces["Fy_f_filt"], "fy_r": forces["Fy_r_filt"],
            "al_f": slip["alpha_f_filt"], "al_r": slip["alpha_r_filt"]}
    all_runs = runs_of(mask)
    # ctx.racing is off-kerb by construction, so the event rule's "moving,
    # analysis lap" is built here with kerb samples kept
    on_lap = st["moving_mask"] & ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    events = [(a, b) for a, b in all_runs if on_lap[a:b + 1].any()]
    rng = np.random.default_rng(SEED)
    pseudo = place_null(events, racing, in_corner, mask, sr, rng)
    pseudo_ok = [p for p in pseudo if p is not None]
    pmask = np.zeros(mask.size, dtype=bool)
    for p, q in pseudo_ok:
        pmask[p:q + 1] = True

    gap, cap, span = int(round(GAP_S * sr)), int(round(CAP_S * sr)), int(round(BAND_SPAN_S * sr))
    out = {"key": key, "name": ctx.name, "sr": sr, "n_events": len(events), "n_null": len(pseudo_ok),
           "entry": entry, "q": {}}
    zones = {}
    for q in QTY:
        x = np.asarray(raw[q], dtype=float)
        d = np.abs(filt[q] - _butterworth_lowpass(substitute(x, all_runs), cut, sr))
        dn = np.abs(filt[q] - _butterworth_lowpass(substitute(x, pseudo_ok), cut, sr))
        bs = []
        for p, qq in pseudo_ok:
            bs.append(dn[qq + 1:qq + 1 + span])
            bs.append(dn[max(0, p - span):p])
        band = float(np.percentile(np.concatenate(bs), BAND_PCTL))
        ev = []
        zone = np.zeros(mask.size, dtype=bool)
        for a, b in events:
            post, cpost = walk(d, band, mask, b, +1, gap, cap)
            pre, cpre = walk(d, band, mask, a, -1, gap, cap)
            zone[b + 1:b + 1 + post] = True
            zone[a - pre:a] = True
            v = float(st["v_mps"][b])
            ev.append({"a": int(a), "b": int(b), "post": int(post), "pre": int(pre), "c_post": cpost,
                       "c_pre": cpre, "v_end": v, "t_end": float(st["time"][b]),
                       "s_end": float(st["s_m"][b]) if np.isfinite(st["s_m"][b]) else None})
        nul = []
        for p, qq in pseudo_ok:
            post, cpost = walk(dn, band, pmask | mask, qq, +1, gap, cap)
            pre, cpre = walk(dn, band, pmask | mask, p, -1, gap, cap)
            nul.append({"post": int(post), "pre": int(pre), "c_post": cpost, "c_pre": cpre})
        zones[q] = zone
        out["q"][q] = {"band": band, "events": ev, "null": nul}

    # CS windows per axle against that axle's Fy zone, alpha zone and the mask
    min_window = resolve_cs_min_window_samples(params, sr)
    min_span, max_m = se["cs_min_slip_angle_span_rad"], se["cs_max_window_m"]
    csw = {}
    for ax in ("f", "r"):
        alpha = slip[f"alpha_{ax}_filt"]
        fin = np.flatnonzero(np.isfinite(cs[f"CS_ratio_{ax}"]))
        pre = {k: np.concatenate(([0], np.cumsum(z.astype(int)))) for k, z in
               (("fy", zones[f"fy_{ax}"]), ("al", zones[f"al_{ax}"]),
                ("any", zones[f"fy_{ax}"] | zones[f"al_{ax}"]), ("mask", mask))}
        hit = {k: np.zeros(fin.size, dtype=bool) for k in pre}
        endp = {k: np.zeros(fin.size, dtype=bool) for k in ("fy", "al", "any")}
        for j, i in enumerate(fin):
            s0 = reconstruct_cs_window_start(alpha, int(i), min_window, min_span, st["s_m"], max_m)
            for k, c in pre.items():
                hit[k][j] = (c[i] - c[s0]) > 0
            endp["fy"][j] = zones[f"fy_{ax}"][i]
            endp["al"][j] = zones[f"al_{ax}"][i]
            endp["any"][j] = endp["fy"][j] or endp["al"][j]
        ic = in_corner[fin]
        csw[ax] = {"n": int(fin.size), "n_corner": int(ic.sum()),
                   "win": {k: float(h.mean()) for k, h in hit.items()},
                   "win_corner": {k: float(h[ic].mean()) for k, h in hit.items()},
                   "end": {k: float(h.mean()) for k, h in endp.items()},
                   "end_corner": {k: float(h[ic].mean()) for k, h in endp.items()}}
    out["cs"] = csw
    return out, ctx


def pct(xs):
    if not xs:
        return [None] * 3
    return [float(np.percentile(xs, p)) for p in (50, 75, 90)]


def summarise(results):
    lines = []
    groups = {"NBR (16)": [r for r in results if r["key"].startswith("nbr")],
              "Paul Ricard (2)": [r for r in results if not r["key"].startswith("nbr")]}
    for r in results:
        sr = r["sr"]
        lines.append(f"\n[{r['key']}] {r['name']}: events {r['n_events']}, null {r['n_null']}, entry {r['entry']}")
        for q in QTY:
            Q = r["q"][q]
            unc = [e for e in Q["events"] if e["c_post"] is None]
            po = pct([e["post"] for e in unc])
            pr = pct([e["pre"] for e in Q["events"] if e["c_pre"] is None])
            npo = pct([e["post"] for e in Q["null"] if e["c_post"] is None])
            cens = sum(e["c_post"] is not None for e in Q["events"])
            nz = np.mean([e["post"] > 0 for e in Q["events"]]) if Q["events"] else float("nan")
            nnz = np.mean([e["post"] > 0 for e in Q["null"]]) if Q["null"] else float("nan")
            lines.append(f"  {q}: band {Q['band']:.4g}; post p50/75/90 ms {_ms(po, sr)} (null {_ms(npo, sr)}); "
                         f"pre {_ms(pr, sr)}; censored post {cens}; share>0 {nz:.2f} vs null {nnz:.2f}")
        for ax in ("f", "r"):
            c = r["cs"][ax]
            lines.append(f"  CS {ax}: n {c['n']} (corner {c['n_corner']}); window holds Fy-zone {c['win']['fy']:.3f} "
                         f"alpha-zone {c['win']['al']:.3f} either {c['win']['any']:.3f} mask {c['win']['mask']:.3f}; "
                         f"endpoint in either zone {c['end']['any']:.3f}; corner: win either "
                         f"{c['win_corner']['any']:.3f} mask {c['win_corner']['mask']:.3f} end {c['end_corner']['any']:.3f}")
    for g, rs in groups.items():
        lines.append(f"\nPOOLED {g}")
        for q in QTY:
            ev = [(e, r["sr"]) for r in rs for e in r["q"][q]["events"]]
            nu = [(e, r["sr"]) for r in rs for e in r["q"][q]["null"]]
            post_ms = [e["post"] * 1000 / s for e, s in ev if e["c_post"] is None]
            post_m = [e["post"] / s * e["v_end"] for e, s in ev if e["c_post"] is None]
            pre_ms = [e["pre"] * 1000 / s for e, s in ev if e["c_pre"] is None]
            nul_ms = [e["post"] * 1000 / s for e, s in nu if e["c_post"] is None]
            cn = sum(e["c_post"] == "neighbour" for e, _ in ev)
            cc = sum(e["c_post"] == "cap" for e, _ in ev)
            lines.append(f"  {q}: n {len(ev)}; post ms p50/75/90 {_r(pct(post_ms))} m {_r(pct(post_m))}; "
                         f"pre ms {_r(pct(pre_ms))}; null post ms {_r(pct(nul_ms))}; censored neighbour {cn} cap {cc}")
        for ax in ("f", "r"):
            n = sum(r["cs"][ax]["n"] for r in rs)
            nc = sum(r["cs"][ax]["n_corner"] for r in rs)
            w = lambda k, sub="win", nn="n": sum(r["cs"][ax][sub][k] * r["cs"][ax][nn] for r in rs) / max(
                1, sum(r["cs"][ax][nn] for r in rs))
            lines.append(f"  CS {ax}: n {n}; windows holding Fy-zone {w('fy'):.3f} alpha-zone {w('al'):.3f} "
                         f"either {w('any'):.3f} masked {w('mask'):.3f}; endpoint in either zone "
                         f"{w('any', 'end'):.3f} | in-corner n {nc}: either {w('any', 'win_corner', 'n_corner'):.3f} "
                         f"masked {w('mask', 'win_corner', 'n_corner'):.3f} endpoint {w('any', 'end_corner', 'n_corner'):.3f}")
    return "\n".join(lines)


def _ms(p, sr):
    return "/".join("-" if v is None else f"{v * 1000 / sr:.0f}" for v in p)


def _r(p):
    return "/".join("-" if v is None else f"{v:.0f}" for v in p)


def pick_cases(results, ax):
    q = f"fy_{ax}"
    pool = [(r["key"], e["t_end"], e, r["sr"]) for r in results for e in r["q"][q]["events"]
            if e["c_post"] is None]
    pool.sort(key=lambda z: (KEYS.index(z[0]), z[1]))
    worst = max(pool, key=lambda z: z[2]["post"] / z[3])
    med = float(np.percentile([z[2]["post"] / z[3] for z in pool], 50))
    median = min(pool, key=lambda z: abs(z[2]["post"] / z[3] - med))
    return worst, median


def render(results):
    plt = lp._plt()
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            _butterworth_lowpass)
    for ax, axname in (("f", "front"), ("r", "rear")):
        cases = pick_cases(results, ax)
        fig, axs = plt.subplots(3, 2, figsize=(13, 9.5), sharex="col")
        for col, (label, (key, _t, ev, sr)) in enumerate(zip(("worst case", "median case"), cases)):
            ctx = lp.Ctx(key)
            st, params = ctx.state, ctx.params
            cut = params["stability_estimation"]["cs_filter_cutoff_hz"]
            beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
            slip = estimate_slip_angles(st, beta, params)
            forces = estimate_lateral_forces(st, params)
            mask = st["kerb_mask"].astype(bool)
            runs = runs_of(mask)
            r = next(x for x in results if x["key"] == key)
            a, b = ev["a"], ev["b"]
            lo, hi = max(0, a - int(1.0 * sr)), min(mask.size, b + int(1.5 * sr))
            t = st["time"][lo:hi] - st["time"][b]
            ev_al = next(e for e in r["q"][f"al_{ax}"]["events"] if e["a"] == a)
            rows = (("Fy_" + ax, forces[f"Fy_{ax}_raw"], forces[f"Fy_{ax}_filt"], 1e-3, "axle lateral force [kN]"),
                    ("alpha_" + ax, slip[f"alpha_{ax}_raw"], slip[f"alpha_{ax}_filt"], 180 / np.pi,
                     "slip angle [deg]"))
            dd = {}
            for k, (nm, xr, xf, sc, yl) in enumerate(rows):
                xs = _butterworth_lowpass(substitute(np.asarray(xr, float), runs), cut, sr)
                dd[k] = np.abs(xf - xs)
                A = axs[k, col]
                A.plot(t, xr[lo:hi] * sc, color=lp.REF, lw=0.6, label="raw (unfiltered)")
                A.plot(t, xf[lo:hi] * sc, color=lp.INK, lw=1.4, label="production, 2 Hz filtered")
                A.plot(t, xs[lo:hi] * sc, color=lp.ACC, lw=1.2, ls="--",
                       label="filtered with kerb samples bridged by a line")
                A.set_ylabel(yl)
                lp._style(A)
            bands = (r["q"][f"fy_{ax}"]["band"], r["q"][f"al_{ax}"]["band"])
            A = axs[2, col]
            A.plot(t, dd[0][lo:hi] / bands[0], color=lp.ACC, lw=1.2, label="force smear / noise band")
            A.plot(t, dd[1][lo:hi] / bands[1], color=lp.ACC2, lw=1.2, label="slip-angle smear / noise band")
            A.axhline(1.0, color=lp.INK, lw=0.8, ls=":")
            A.set_ylabel("smear / band [-]")
            A.set_xlabel("time from kerb-mask end [s]")
            lp._style(A)
            for k in range(3):
                A = axs[k, col]
                for ra, rb in runs:
                    if rb >= lo and ra < hi:
                        A.axvspan(st["time"][max(ra, lo)] - st["time"][b], st["time"][min(rb, hi - 1)] - st["time"][b],
                                  color=lp.BAND, alpha=0.9, lw=0)
                A.axvspan(0, ev["post"] / sr, color=lp.HL, alpha=0.7, lw=0)
                if ev["pre"]:
                    A.axvspan(st["time"][a - ev["pre"]] - st["time"][b], st["time"][a] - st["time"][b],
                              color=lp.HL, alpha=0.4, lw=0)
            lap = next((l["lap_number"] for l in ctx.data["laps"]
                        if l["start_time"] <= st["time"][b] <= l["end_time"]), "?")
            s_txt = f"{ev['s_end']:.0f} m" if ev["s_end"] is not None else "lap distance n/a"
            axs[0, col].set_title(lp._wrap(f"{label}: {ctx.name}, lap {lap}, mask end at {s_txt}\n"
                                           f"force smear {ev['post'] * 1000 / sr:.0f} ms past the edge, "
                                           f"slip-angle smear {ev_al['post'] * 1000 / sr:.0f} ms", fig), fontsize=9)
        axs[0, 0].legend(fontsize=7, loc="best")
        axs[2, 0].legend(fontsize=7, loc="best")
        fig.suptitle(lp._wrap(f"{axname.capitalize()} axle: how far a kerb strike leaks past the kerb-mask edge "
                              f"through the 2 Hz zero-phase filter (grey = kerb mask, yellow = smear zone above "
                              f"the clean-stretch noise band)", fig), fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        path = os.path.join(OUT_DIR, f"filter_smear_{axname}_axle.png")
        fig.savefig(path, dpi=130)
        plt.close(fig)
        _log(f"plot {path}: worst {cases[0][0]} t={cases[0][1]:.1f}s, median {cases[1][0]} t={cases[1][1]:.1f}s")


# SPIKE COINCIDENCE (author ruling 5, 2026-10-03; read-only from
# decay_records.json + the cached production CS, no session re-run). Decides
# whether the grade zone sits at p75 or at p90 / filter reach.
# - Population: finite cached production CS on analysis laps, NBR pooled (the
#   files that passed the entry check), per axle; in-corner subset by the
#   cached corner brackets. Cutoffs: CS < 0 (negative stiffness, physically
#   impossible on the tyre curve) and CS < -1 (the deep tail).
# - Mask = the recorded event runs (runs touching analysis laps; runs wholly
#   off-lap cannot reach analysis-lap CS except at lap seams). Zone = that
#   axle's Fy or alpha smear zone, pre and post, outside the mask.
# - Exact: endpoint in the zone; distance from the endpoint to the nearest
#   mask sample, shares within fixed widths. Bounded only (production alpha is
#   not cached): window contents, lower bound = the minimum window (0.1 s),
#   upper bound = everything within cs_max_window_m back on the same lap.
WIDTHS_MS = (50, 100, 140, 250, 360, 400)


def coincidence():
    from diagnostics import inspect_tyre_prior_d2 as d2
    params = d2._params()
    se = params["stability_estimation"]
    with open(os.path.join(OUT_DIR, "decay_records.json")) as fh:
        recs = [r for r in json.load(fh) if r["key"].startswith("nbr")]
    acc = {ax: {"all": [], "corner": []} for ax in ("f", "r")}
    for r in recs:
        fit = d2.load_fit(r["key"])
        sr = r["sr"]
        t = np.asarray(fit["time"], dtype=float)
        s = np.asarray(fit["s_m"], dtype=float)
        n = t.size
        min_w = int(round(se["cs_min_window_s"] * sr))
        lap_id = np.full(n, -1)
        an = [l for l in fit["laps"] if l.get("is_valid_for_analysis")]
        for l in an:
            lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
        corner = np.zeros(n, dtype=bool)
        for c in fit["corners"]:
            if c["bracket_start_m"] is None:
                continue
            on = lap_id == c["lap_number"]
            corner |= on & (s >= c["bracket_start_m"]) & (s <= c["bracket_end_m"])
        mask = np.zeros(n, dtype=bool)
        for e in r["q"]["fy_f"]["events"]:
            mask[e["a"]:e["b"] + 1] = True
        midx = np.flatnonzero(mask)
        # upper-bound window start: earliest sample on the same lap within the
        # metre cap (s made monotone per lap so a GPS wobble cannot end it early)
        lo_ub = np.arange(n)
        for l in an:
            idx = np.flatnonzero(lap_id == l["lap_number"])
            if idx.size == 0:
                continue
            sl = np.maximum.accumulate(np.nan_to_num(s[idx], nan=-np.inf))
            j = np.searchsorted(sl, sl - se["cs_max_window_m"], side="left")
            lo_ub[idx] = idx[0] + j
        for ax in ("f", "r"):
            zone = np.zeros(n, dtype=bool)
            for q in (f"fy_{ax}", f"al_{ax}"):
                for e in r["q"][q]["events"]:
                    zone[e["b"] + 1:e["b"] + 1 + e["post"]] = True
                    zone[e["a"] - e["pre"]:e["a"]] = True
            zone &= ~mask
            cs = np.asarray(fit["prod"]["down"][f"cs_{ax}"], dtype=float)
            fin = np.flatnonzero(np.isfinite(cs) & (lap_id >= 0))
            cz = np.concatenate(([0], np.cumsum(zone)))
            cm = np.concatenate(([0], np.cumsum(mask)))
            lo_lb = np.maximum(fin - min_w, 0)
            lo_u = lo_ub[fin]
            if midx.size:
                k = np.searchsorted(midx, fin)
                dl = np.where(k > 0, fin - midx[np.clip(k - 1, 0, midx.size - 1)], 10 ** 9)
                dr = np.where(k < midx.size, midx[np.clip(k, 0, midx.size - 1)] - fin, 10 ** 9)
                dist = np.minimum(dl, dr) * 1000.0 / sr
            else:
                dist = np.full(fin.size, 1e9)
            rows = {"cs": cs[fin], "end_zone": zone[fin], "dist_ms": dist,
                    "z_lb": (cz[fin] - cz[lo_lb]) > 0, "m_lb": (cm[fin] - cm[lo_lb]) > 0,
                    "z_ub": (cz[fin] - cz[lo_u]) > 0, "m_ub": (cm[fin] - cm[lo_u]) > 0}
            acc[ax]["all"].append(rows)
            acc[ax]["corner"].append({k: v[corner[fin]] for k, v in rows.items()})
    lines = [f"SPIKE COINCIDENCE, NBR pooled ({len(recs)} files), cached production CS, analysis laps"]
    for ax, axname in (("f", "front"), ("r", "rear")):
        for pop in ("all", "corner"):
            R = {k: np.concatenate([x[k] for x in acc[ax][pop]]) for k in acc[ax][pop][0]}
            lines.append(f"\n{axname} axle, {'in-corner' if pop == 'corner' else 'all analysis-lap'} CS, n {R['cs'].size}")
            for label, sel in (("all CS (base rate)", np.ones(R["cs"].size, dtype=bool)),
                               ("CS < 0", R["cs"] < 0), ("CS < -1", R["cs"] < -1)):
                m = int(sel.sum())
                if m == 0:
                    lines.append(f"  {label}: n 0")
                    continue
                f = lambda a: float(a[sel].mean())
                within = " ".join(f"{w}ms {float((R['dist_ms'][sel] <= w).mean()):.3f}" for w in WIDTHS_MS)
                lines.append(
                    f"  {label}: n {m}; endpoint in smear zone {f(R['end_zone']):.3f}; "
                    f"endpoint within X of a mask sample: {within}")
                for b in ("lb", "ub"):
                    z, mk = R[f"z_{b}"][sel], R[f"m_{b}"][sel]
                    lines.append(
                        f"    window {'lower' if b == 'lb' else 'upper'} bound: holds zone {z.mean():.3f}, "
                        f"holds mask {mk.mean():.3f}; zone only {(z & ~mk).mean():.3f}, mask only "
                        f"{(mk & ~z).mean():.3f}, both {(z & mk).mean():.3f}, neither {(~z & ~mk).mean():.3f}")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "coincidence.txt"), "w") as fh:
        fh.write(text)
    _log(text)


# SITE CHECK (author ruling, option (b), 2026-10-03; records only, no session
# re-run). Do the 22 position-locked dip sites of Q-PAIRING lie inside the
# grade zone the package will ship?
# - Sites and verdicts: diagnostics/plots/pairing/sites_<key>.json, Tier 1
#   (nbr_jst_solution, pr_us_v2, v3).
# - Dip sample per site and lap: Q-PAIRING's own rule (lap's CS minimum within
#   +/-SITE_TOL_M of the site, inside a corner instance), on the cached
#   production CS; entry check: equals the recorded per-lap CS (<= 1e-6).
# - Grade zone G: outside the kerb mask, within kerb_smear_grade_s = 0.25 s of
#   a mask sample, either side (plain reading of the ruled width; the
#   package's construction is not frozen yet). Mask = the recorded event runs.
# - Reported per site: laps whose dip sample lies in G; laps whose window
#   (rebuilt from the recorded window_m by track distance -- approximate)
#   reaches G; site INSIDE = dip in G on >= half its laps (mirrors Q-PAIRING's
#   B rule). Split: B-flagged sites vs the others.
GRADE_S = 0.25
SITE_TOL_M = 10.0
TIER1 = ("nbr_jst_solution", "pr_us_v2", "v3")


def site_check():
    from diagnostics import inspect_tyre_prior_d2 as d2
    from diagnostics.gated_kinematic_common import valid_corner_instances
    with open(os.path.join(OUT_DIR, "decay_records.json")) as fh:
        recs = {r["key"]: r for r in json.load(fh)}
    lines = ["SITE CHECK: position-locked dip sites vs the 0.25 s grade zone (records only)"]
    tally = {"B": [0, 0], "other": [0, 0]}
    entry_max = 0.0
    for key in TIER1:
        r = recs[key]
        sr = r["sr"]
        fit = d2.load_fit(key)
        t = np.asarray(fit["time"], dtype=float)
        s = np.asarray(fit["s_m"], dtype=float)
        n = t.size
        mask = np.zeros(n, dtype=bool)
        for e in r["q"]["fy_f"]["events"]:
            mask[e["a"]:e["b"] + 1] = True
        g = int(round(GRADE_S * sr))
        cm = np.concatenate(([0], np.cumsum(mask)))
        idx = np.arange(n)
        near = (cm[np.clip(idx + g + 1, 0, n)] - cm[np.clip(idx - g, 0, n)]) > 0
        zone = near & ~mask
        cz = np.concatenate(([0], np.cumsum(zone)))
        in_br = np.zeros(n, dtype=bool)
        for _sid, _lap, sl in valid_corner_instances({"laps": fit["laps"], "corners": fit["corners"]}, t, s):
            in_br[sl] = True
        laps = {l["lap_number"]: (t >= l["start_time"]) & (t <= l["end_time"])
                for l in fit["laps"] if l.get("is_valid_for_analysis")}
        with open(f"diagnostics/plots/pairing/sites_{key}.json") as fh:
            sites = json.load(fh)
        lines.append(f"\n[{key}] {r['name']}")
        for ax in ("f", "r"):
            cs = np.asarray(fit["prod"]["down"][f"cs_{ax}"], dtype=float)
            for site in sites["axles"][ax]["sites"]:
                in_dip = in_win = 0
                detail = []
                for row in site["laps"]:
                    k = laps[row["lap"]] & np.isfinite(cs) & (np.abs(s - site["pos_m"]) <= SITE_TOL_M) & in_br
                    j = int(np.flatnonzero(k)[np.argmin(cs[k])])
                    entry_max = max(entry_max, abs(cs[j] - row["cs"]))
                    # s_m resets every lap: search inside this lap only
                    l0 = int(np.flatnonzero(laps[row["lap"]])[0])
                    sl = np.maximum.accumulate(np.nan_to_num(s[l0:j], nan=-np.inf))
                    w0 = l0 + int(np.searchsorted(sl, s[j] - row["window_m"], side="left")) if j > l0 else j
                    w0 = min(max(w0, l0), j)
                    d_in = bool(zone[j])
                    w_in = bool(cz[j] - cz[w0] > 0) or d_in
                    in_dip += d_in
                    in_win += w_in
                    detail.append(f"{row['lap']}:{'Z' if d_in else ('w' if w_in else '-')}")
                inside = in_dip >= len(site["laps"]) / 2.0
                grp = "B" if "B" in site["flags"] else "other"
                tally[grp][0] += inside
                tally[grp][1] += 1
                lines.append(f"  {ax} site {site['pos_m']:.0f} m, verdict {site['verdict']}, median CS "
                             f"{site['median_cs']:.2f}: dip in zone {in_dip}/{len(site['laps'])} laps, window reaches "
                             f"zone {in_win}/{len(site['laps'])} -> {'INSIDE' if inside else 'outside'}  "
                             f"[{' '.join(detail)}]  (Z dip in zone, w window only, - neither)")
    lines.append(f"\nENTRY: max |rebuilt dip CS - recorded| = {entry_max:.2e}")
    lines.append(f"SITES INSIDE the 0.25 s zone: B-flagged {tally['B'][0]}/{tally['B'][1]}, others "
                 f"{tally['other'][0]}/{tally['other'][1]}, all {tally['B'][0] + tally['other'][0]}/"
                 f"{tally['B'][1] + tally['other'][1]}")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "site_check.txt"), "w") as fh:
        fh.write(text)
    _log(text)


# VERIFY (options 4 + 0 package, author EXECUTE 2026-10-03). Production code,
# cached production beta (no EKF re-run: the mask's effect on CS is isolated,
# a re-fit of the tyre curve under the wider mask is not part of this check).
# - Stage A (option 4 alone): kerb_dilation_s forced to 0.05 s = the old 5
#   samples at 100 Hz; CS must reproduce the cached production CS (identical
#   finite set, max |diff| <= ENTRY_TOL) -- the mark changed nothing.
# - Stage B (shipping config): per position-locked site and lap, dip rebuilt by
#   Q-PAIRING's rule on the new CS; before / after dip CS, whether the old dip
#   sample is now masked, whether the site lost every finite CS on a lap;
#   near_strike at the new dip; site MARKED = mark on >= half its laps.
def verify():
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, prepare_vehicle_state)
    from diagnostics.gated_kinematic_common import valid_corner_instances
    lines = ["VERIFY: options 4 + 0 on the Tier-1 sessions (cached production beta)"]
    tally = {"B": [0, 0], "other": [0, 0]}
    affected = []
    for key in TIER1:
        ctx = lp.Ctx(key)
        p_new = ctx.params
        p_old = json.loads(json.dumps(p_new))
        p_old["stability_estimation"]["kerb_dilation_s"] = 0.05
        beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
        down = ctx.cache["prod"]["down"]
        out = {}
        for tag, prm in (("A", p_old), ("B", p_new)):
            st = prepare_vehicle_state(ctx.data["channels"], prm)
            slip = estimate_slip_angles(st, beta, prm)
            forces = estimate_lateral_forces(st, prm)
            out[tag] = (st, estimate_cornering_stiffness(slip, forces, st, prm))
        stA, csA = out["A"]
        stB, csB = out["B"]
        lines.append(f"\n[{key}] {ctx.name}: mask samples {int(stA['kerb_mask'].sum())} -> {int(stB['kerb_mask'].sum())}")
        for ax in ("f", "r"):
            old = np.asarray(down[f"cs_{ax}"], dtype=float)
            a = csA[f"CS_ratio_{ax}"]
            same = bool(np.array_equal(np.isfinite(a), np.isfinite(old)))
            fin = np.isfinite(a) & np.isfinite(old)
            md = float(np.max(np.abs(a[fin] - old[fin]))) if fin.any() else 0.0
            lines.append(f"  stage A {ax}: same finite set {same}, max diff {md:.2e} -> "
                         f"{'PASS' if same and md <= ENTRY_TOL else 'FAIL'}")
        t, s = stB["time"], stB["s_m"]
        in_br = np.zeros(t.size, dtype=bool)
        for _sid, _lap, sl in valid_corner_instances({"laps": ctx.data["laps"], "corners": ctx.data["corners"]}, t, s):
            in_br[sl] = True
        laps = {l["lap_number"]: (t >= l["start_time"]) & (t <= l["end_time"])
                for l in ctx.data["laps"] if l.get("is_valid_for_analysis")}
        with open(f"diagnostics/plots/pairing/sites_{key}.json") as fh:
            sites = json.load(fh)
        for ax in ("f", "r"):
            old = np.asarray(down[f"cs_{ax}"], dtype=float)
            new = csB[f"CS_ratio_{ax}"]
            mark = csB[f"near_strike_{ax}"]
            for site in sites["axles"][ax]["sites"]:
                n_mark = 0
                per_lap = []
                for row in site["laps"]:
                    near = laps[row["lap"]] & (np.abs(s - site["pos_m"]) <= SITE_TOL_M) & in_br
                    k_old = near & np.isfinite(old)
                    j_old = int(np.flatnonzero(k_old)[np.argmin(old[k_old])])
                    k_new = near & np.isfinite(new)
                    if not k_new.any():
                        per_lap.append((row["lap"], old[j_old], None, True, False, None))
                        continue
                    j_new = int(np.flatnonzero(k_new)[np.argmin(new[k_new])])
                    masked = bool(stB["kerb_mask"][j_old])
                    n_mark += bool(mark[j_new])
                    per_lap.append((row["lap"], old[j_old], new[j_new], masked, bool(mark[j_new]),
                                    float(s[j_new] - s[j_old])))
                marked = n_mark >= len(site["laps"]) / 2.0
                grp = "B" if "B" in site["flags"] else "other"
                tally[grp][0] += marked
                tally[grp][1] += 1
                moved = [x for x in per_lap if x[2] is None or x[3] or abs(x[2] - x[1]) > 1e-6]
                lines.append(f"  {ax} site {site['pos_m']:.0f} m ({site['verdict']}): marked on {n_mark}/"
                             f"{len(site['laps'])} laps -> {'MARKED' if marked else 'unmarked'}; dip changed on "
                             f"{len(moved)}/{len(site['laps'])} laps")
                for lap, b, aft, masked, mk, ds in moved:
                    if aft is None:
                        txt = f"lap {lap}: {b:.3f} -> NO finite CS within +/-{SITE_TOL_M:.0f} m (site lost on this lap)"
                    else:
                        txt = (f"lap {lap}: {b:.3f} -> {aft:.3f}" + (" (old dip sample now masked" if masked else " (")
                               + f"{', ' if masked else ''}dip moved {ds:+.1f} m)")
                    lines.append("      " + txt)
                    affected.append(f"{key} {ax} {site['pos_m']:.0f} m {txt}")
    lines.append(f"\nSITES MARKED (near_strike on >= half the laps): B-flagged {tally['B'][0]}/{tally['B'][1]}, "
                 f"others {tally['other'][0]}/{tally['other'][1]}, all {tally['B'][0] + tally['other'][0]}/"
                 f"{tally['B'][1] + tally['other'][1]}")
    lines.append(f"SITE-LAPS WITH A CHANGED DIP (option 0): {len(affected)}")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "verify.txt"), "w") as fh:
        fh.write(text)
    _log(text)


def main():
    if sys.argv[1:2] == ["verify"]:
        verify()
        return
    if sys.argv[1:2] == ["coincidence"]:
        coincidence()
        return
    if sys.argv[1:2] == ["sites"]:
        site_check()
        return
    os.makedirs(OUT_DIR, exist_ok=True)
    results = []
    for key in (sys.argv[1:] or KEYS):
        res, _ctx = session(key)
        if res is None:
            continue
        results.append(res)
        _log(f"[{key}] done: {res['n_events']} events, {res['n_null']} null")
    with open(os.path.join(OUT_DIR, "decay_records.json"), "w") as fh:
        json.dump(results, fh, default=lp._jsonable)
    text = summarise(results)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    _log(text)
    render(results)


if __name__ == "__main__":
    main()
