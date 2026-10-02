# Q2d: rear-curve bias diagnostic (PLAN.md PHASE 1 PLAN Q2d; read-only).
#
# HYPOTHESIS (author, 2026-10-02): the rear pure-lateral Pacejka fit is biased
# LOW, most at large alpha, because combined-slip exit samples carry less
# lateral force than the tyre's pure-lateral capability ((e) Stage 0: rear
# ~37 % longitudinal share at the median exit sample, under-read >= 8 %, 41 %
# at p90) and sit where the peak is fitted; sub-limit driving adds one-sided
# under-cloud scatter.
#
# RULES, fixed before any run.
# Sessions: pr_us_v2, v3. Inputs: the (d2) production fit cache (kinematic
#   alpha_*_filt, Fy_*_filt, production base mask, manifest) -- the exact
#   production fit population. Fz is not an input (free-D fits).
# Fit 1 PRODUCTION: tyre_fit_auto._fit_axle_pacejka on the cached arrays;
#   ENTRY CHECK: B, C, D, E equal the cached manifest to 1e-9 relative.
# Fit 2 LOW-KAPPA: same fitter (Powell from the chair start guess), population
#   restricted to |kappa_axle - straight median| < 3 x straight robust sd
#   (1.4826 MAD), per session and axle. kappa = production estimate_slip_ratio
#   (wheel speeds, (e) route ii), low-passed like the chain (LS _filtered at
#   cs_filter_cutoff_hz). Straight = base & |a_x|, |a_y| < 0.1 g & v > 60 kph
#   (the (e) Stage 0 floors were 0.141 / 0.127 pp rear).
# Fit 3 UPPER-QUANTILE: fold x = |alpha|, y = Fy sign(alpha); 0.2 deg bins
#   from 0 to the population's p99 |alpha|, bins with >= 30 samples; q = 0.9
#   quantile of y per bin; Pacejka fitted (Powell, chair start) to the bin
#   points, unweighted. Concept anchor: the measured envelope reading of RCVD
#   9.3/9.4; method Tier B (binned quantile, standard).
# Deltas against fit 1 on the grid [0, p99 |alpha|] of the production
#   population: mean signed curve delta in the LOWER region [0, p50] and the
#   UPPER region [p50, p99] (% of production D), curve RMS, dD, peak alpha
#   (labelled extrapolated when beyond p99).
# Bar ("input-error family"): 3.8 % of D = quadrature of the recorded scale-
#   error components (GPS k up to 1.2 %, a_y scale ~2 %, mass/fuel ~3 %).
#   Stated with it: a common scale error multiplies every fit alike, so it
#   largely cancels in fit-to-fit deltas; the statistical context is a lap
#   bootstrap (B = 100, analysis laps resampled with replacement) of each
#   delta, reported beside the bar.
# CONVERGENT-EVIDENCE READ (rear, per session):
#   READ-L (as ordered): SUPPORTED iff fit 2 AND fit 3 upper-region deltas are
#     both > +3.8 % (or both move the peak later); CONTRADICTED iff both
#     < -3.8 %; else NOT SUPPORTED.
#   READ-C (construction-corrected, added before the run): an upper-quantile
#     fit lies above a least-squares fit BY CONSTRUCTION (about the scatter
#     width), so fit 3's raw upward shift is no evidence. Its informative part
#     is alpha-dependence: excess = upper-region delta - lower-region delta.
#     SUPPORTED iff fit 2's upper-region delta > +3.8 % AND fit 3's excess
#     > +3.8 %; else as READ-L.
#   Front is reported as control with no prediction (front carries its own
#   combined slip under braking).
# OPTION-2 SIGNATURE (pre-registered in the (e) proposal): rear production
#   residual (folded) on high-kappa EXIT samples (kappa excursion >= 10 x
#   straight robust sd) is negative with lap-bootstrap 95 % CI excluding 0 and
#   |mean| >= 5 % of production D.
# Outputs diagnostics/plots/rear_curve_bias/: q2d.json, q2d.txt,
#   curves_<key>_<axle>.png, residual_kappa_<key>.png.

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2

OUT_DIR = "diagnostics/plots/rear_curve_bias"
KEYS = ("pr_us_v2", "v3")
G = 9.81
KAPPA_K, HIGH_K = 3.0, 10.0
BIN_DEG, MIN_BIN, Q = 0.2, 30, 0.9
BAR_PCT = 3.8
N_BOOT = int(os.environ.get("Q2D_BOOT", 100))  # smoke runs only override it
RNG = np.random.default_rng(20261002)


def _pac(a, p):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    return pacejka_lateral_force(a, *p)


def _fit_points(x, y):
    from scipy.optimize import minimize
    from modules.tyre_fit_auto import PACEJKA_START_GUESS
    res = minimize(lambda p: float(np.sum((_pac(x, p) - y) ** 2)), PACEJKA_START_GUESS, method="Powell")
    return tuple(float(v) for v in res.x)


def _fit_prod(alpha, fy, mask):
    from modules.tyre_fit_auto import _fit_axle_pacejka
    r = _fit_axle_pacejka(alpha, fy, mask)
    return (r["B"], r["C"], r["D"], r["E"])


def _quantile_points(alpha, fy, mask, p99):
    x = np.abs(np.degrees(alpha[mask]))
    y = fy[mask] * np.sign(alpha[mask])
    xs, ys = [], []
    for lo in np.arange(0.0, p99, BIN_DEG):
        m = (x >= lo) & (x < lo + BIN_DEG)
        if m.sum() >= MIN_BIN:
            xs.append(np.radians(lo + BIN_DEG / 2))
            ys.append(np.quantile(y[m], Q))
    return np.array(xs), np.array(ys)


def _peak_deg(p, p99):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, 89.9, 4000)
    s = pacejka_lateral_stiffness(np.radians(g), *p)
    i = np.where(np.diff(np.sign(s)) < 0)[0]
    pk = float(g[i[0]]) if i.size else float("nan")
    return pk, bool(pk == pk and pk <= p99)


def _deltas(p, p0, p50, p99):
    g = np.radians(np.linspace(0.0, p99, 300))
    d = (_pac(g, p) - _pac(g, p0)) / abs(p0[2]) * 100.0
    gd = np.degrees(g)
    lo, up = gd <= p50, gd >= p50
    return {"lower_pct": float(np.mean(d[lo])), "upper_pct": float(np.mean(d[up])),
            "rms_pct": float(np.sqrt(np.mean(d ** 2))), "dD_pct": (p[2] - p0[2]) / abs(p0[2]) * 100.0}


def _kappa(key, axle):
    from modules.longitudinal_forces import estimate_slip_ratio
    from modules.longitudinal_stiffness import _filtered
    from modules.tyre_fit_auto import _base_mask
    params, data, state = d2._state(key)
    k = estimate_slip_ratio(state, data["channels"], params)[f"kappa_{axle[0]}"]
    sr, fc = state["sample_rate_hz"], params["stability_estimation"]["cs_filter_cutoff_hz"]
    kf = np.where(np.isfinite(k), _filtered(k, sr, fc), np.nan) * 100.0
    base = _base_mask(state, data["laps"])
    S = base & (np.abs(state["ax_mps2"] / G) < 0.1) & (np.abs(state["ay_mps2"] / G) < 0.1) & (state["v_mps"] * 3.6 > 60)
    ks = kf[S & np.isfinite(kf)]
    med = float(np.median(ks))
    sd = float(1.4826 * np.median(np.abs(ks - med)))
    an = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
    ex = np.zeros(state["time"].size, bool)
    for c in data["corners"]:
        if c.get("lap_number") in an:
            ex |= (state["time"] >= c["segments"]["exit_4"][0]) & (state["time"] <= c["segments"]["exit_5"][1])
    return kf - med, sd, ex, state["time"]


def run():
    os.makedirs(OUT_DIR, exist_ok=True)
    out = {}
    for key in KEYS:
        F = d2.load_fit(key)
        t, base = F["time"], F["base_mask"]
        laps = [l for l in F["laps"] if l.get("is_valid_for_analysis")]
        man = F["prod"]["manifest"] if "manifest" in F["prod"] else None
        rec = {}
        for axle in ("rear", "front"):
            a, fy = F[f"alpha_{axle[0]}"], F[f"fy_{axle[0]}"]
            m0 = base & np.isfinite(a) & np.isfinite(fy)
            p1 = _fit_prod(a, fy, m0)
            ref = (F["prod"].get("axles") or (man or {}).get("axles", {})).get(axle) if isinstance(F["prod"], dict) else None
            entry = None
            if ref:
                entry = max(abs(p1[i] / ref[k] - 1) for i, k in enumerate("BCDE"))
            dk, sd, ex, tk = _kappa(key, axle)
            dk = np.interp(t, tk, dk) if tk.size != t.size else dk
            ex = (np.interp(t, tk, ex.astype(float)) > 0.5) if tk.size != t.size else ex
            lowk = m0 & np.isfinite(dk) & (np.abs(dk) < KAPPA_K * sd)
            ax_deg = np.abs(np.degrees(a[m0]))
            p50, p99 = float(np.percentile(ax_deg, 50)), float(np.percentile(ax_deg, 99))
            p2 = _fit_prod(a, fy, lowk)
            qx, qy = _quantile_points(a, fy, m0, p99)
            p3 = _fit_points(qx, qy)
            fits = {"prod": p1, "lowk": p2, "q90": p3}
            d = {n: _deltas(p, p1, p50, p99) for n, p in fits.items() if n != "prod"}
            pk = {n: _peak_deg(p, p99) for n, p in fits.items()}
            # lap bootstrap of the deltas
            boot = {"lowk": [], "q90": []}
            lap_masks = [(t >= l["start_time"]) & (t <= l["end_time"]) for l in laps]
            for _ in range(N_BOOT):
                pick = RNG.integers(0, len(laps), len(laps))
                w = np.zeros(t.size, int)
                for i in pick:
                    w += lap_masks[i]
                idx = np.repeat(np.arange(t.size), w)
                ab, fb_, mb, lb = a[idx], fy[idx], m0[idx], lowk[idx]
                pb1 = _fit_prod(ab, fb_, mb)
                pb2 = _fit_prod(ab, fb_, lb)
                qxb, qyb = _quantile_points(ab, fb_, mb, p99)
                pb3 = _fit_points(qxb, qyb)
                boot["lowk"].append(_deltas(pb2, pb1, p50, p99))
                boot["q90"].append(_deltas(pb3, pb1, p50, p99))
            ci = {n: {f: [float(np.percentile([b[f] for b in boot[n]], 2.5)),
                          float(np.percentile([b[f] for b in boot[n]], 97.5))] for f in ("lower_pct", "upper_pct")}
                  for n in boot}
            for n in ci:
                ex_b = [b["upper_pct"] - b["lower_pct"] for b in boot[n]]
                ci[n]["excess_pct"] = [float(np.percentile(ex_b, 2.5)), float(np.percentile(ex_b, 97.5))]
            # Option-2 signature (rear only, pre-registered)
            sig = None
            res = (fy - _pac(a, p1)) * np.sign(a)
            if axle == "rear":
                hk = m0 & ex & np.isfinite(dk) & (dk >= HIGH_K * sd)
                bm = []
                for _ in range(N_BOOT):
                    pick = RNG.integers(0, len(laps), len(laps))
                    vals = np.concatenate([res[hk & lap_masks[i]] for i in pick])
                    if vals.size:
                        bm.append(float(np.mean(vals)))
                mean = float(np.mean(res[hk])) if hk.any() else float("nan")
                lo95, hi95 = (float(np.percentile(bm, 2.5)), float(np.percentile(bm, 97.5))) if bm else (np.nan, np.nan)
                sig = {"n": int(hk.sum()), "mean_N": mean, "ci95_N": [lo95, hi95], "pct_of_D": mean / abs(p1[2]) * 100,
                       "present": bool(hi95 < 0 and abs(mean) >= 0.05 * abs(p1[2]))}
                _render_resid(os.path.join(OUT_DIR, f"residual_kappa_{key}.png"), dk, res, m0, ex, sd, key, p1[2])
            up2, up3 = d["lowk"]["upper_pct"], d["q90"]["upper_pct"]
            exc3 = d["q90"]["upper_pct"] - d["q90"]["lower_pct"]
            later2 = pk["lowk"][0] > pk["prod"][0]
            later3 = pk["q90"][0] > pk["prod"][0]
            read_l = ("SUPPORTED" if (up2 > BAR_PCT and up3 > BAR_PCT) or (later2 and later3)
                      else "CONTRADICTED" if (up2 < -BAR_PCT and up3 < -BAR_PCT) else "NOT SUPPORTED")
            read_c = ("SUPPORTED" if (up2 > BAR_PCT and exc3 > BAR_PCT)
                      else "CONTRADICTED" if (up2 < -BAR_PCT and exc3 < -BAR_PCT) else "NOT SUPPORTED")
            rec[axle] = {"entry_check_max_rel": entry, "n_prod": int(m0.sum()), "n_lowk": int(lowk.sum()),
                         "lowk_share": float(lowk.sum() / m0.sum()), "kappa_sd_pp": sd, "p50_deg": p50, "p99_deg": p99,
                         "n_quantile_bins": int(qx.size), "params": fits, "peak_deg": pk, "deltas": d,
                         "boot_ci95": ci, "fit3_excess_pct": exc3, "READ_L": read_l, "READ_C": read_c,
                         "option2_signature": sig,
                         "lowk_share_upper_region": float(np.mean(np.abs(dk[m0 & (np.abs(np.degrees(a)) >= p50)]) < KAPPA_K * sd))}
            _render_curves(os.path.join(OUT_DIR, f"curves_{key}_{axle}.png"), a, fy, m0, lowk, qx, qy, fits, p99, key, axle)
            print(key, axle, json.dumps({k: rec[axle][k] for k in ("entry_check_max_rel", "deltas", "fit3_excess_pct",
                                                                    "READ_L", "READ_C")}), flush=True)
        out[key] = rec
    with open(os.path.join(OUT_DIR, "q2d.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)
    lines = []
    for key, rec in out.items():
        for axle, r in rec.items():
            lines.append(f"{key} {axle}: entry {r['entry_check_max_rel']}; n {r['n_prod']} lowk {r['n_lowk']} "
                         f"({r['lowk_share']:.2f}; upper-region {r['lowk_share_upper_region']:.2f}); bins {r['n_quantile_bins']}; "
                         f"p50/p99 {r['p50_deg']:.2f}/{r['p99_deg']:.2f} deg")
            for n in ("lowk", "q90"):
                dd = r["deltas"][n]
                lines.append(f"   {n}: lower {dd['lower_pct']:+.2f} upper {dd['upper_pct']:+.2f} rms {dd['rms_pct']:.2f} "
                             f"dD {dd['dD_pct']:+.2f} % | CI lower {r['boot_ci95'][n]['lower_pct']} upper "
                             f"{r['boot_ci95'][n]['upper_pct']} excess {r['boot_ci95'][n]['excess_pct']}")
            lines.append(f"   peaks deg: " + ", ".join(f"{n} {v[0]:.2f}{'' if v[1] else ' (extrap.)'}"
                                                       for n, v in r["peak_deg"].items()) +
                         f"; fit3 excess {r['fit3_excess_pct']:+.2f} %; READ-L {r['READ_L']}; READ-C {r['READ_C']}")
            lines.append("   params: " + "; ".join(f"{n} B={p[0]:.4f} C={p[1]:.4f} D={p[2]:.1f} E={p[3]:.4f}"
                                                   for n, p in r["params"].items()))
            if r["option2_signature"]:
                lines.append(f"   option-2 signature: {r['option2_signature']}")
    with open(os.path.join(OUT_DIR, "q2d.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def _render_curves(path, a, fy, m0, lowk, qx, qy, fits, p99, key, axle):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND, REF
    x = np.abs(np.degrees(a))
    y = fy * np.sign(a)
    rng = np.random.default_rng(1)
    pick = rng.choice(np.where(m0)[0], size=min(20000, int(m0.sum())), replace=False)
    lk = lowk[pick]
    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    ax.plot(x[pick][~lk], y[pick][~lk], ".", ms=1.2, color=BAND, label="excluded by low-kappa", zorder=1)
    ax.plot(x[pick][lk], y[pick][lk], ".", ms=1.2, color=REF, alpha=0.5, label="low-kappa population", zorder=2)
    ax.plot(np.degrees(qx), qy, "s", ms=4, color=INK, label=f"q{Q:.1f} bin points", zorder=4)
    g = np.radians(np.linspace(0, max(p99 * 1.5, 1), 300))
    for (n, p), ls in zip(fits.items(), ("-", "--", ":")):
        ax.plot(np.degrees(g), _pac(g, p), ls, color=INK, lw=1.6, label=f"fit {n}", zorder=5)
    ax.axvline(p99, color=REF, lw=0.8, ls="-.")
    ax.set_xlabel("|alpha| [deg] (folded); dash-dot = visited p99", color=INK)
    ax.set_ylabel(f"Fy_{axle[0]} * sign(alpha) [N]", color=INK)
    ax.set_xlim(0, max(p99 * 1.5, 1))
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.grid(color=BAND, lw=0.6)
    fig.suptitle(f"Q2d {key} {axle}: production / low-kappa / upper-quantile fits over the production population",
                 color=INK, fontsize=10)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _render_resid(path, dk, res, m0, ex, sd, key, D):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND, REF
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for mask, lab, col in ((m0 & ~ex, "non-exit", REF), (m0 & ex, "exit phases", INK)):
        m = mask & np.isfinite(dk)
        edges = np.arange(-2.0, 8.01, 0.5)
        xs, ys = [], []
        for lo in edges[:-1]:
            b = m & (dk >= lo) & (dk < lo + 0.5)
            if b.sum() >= 50:
                xs.append(lo + 0.25)
                ys.append(np.mean(res[b]) / abs(D) * 100)
        ax.plot(xs, ys, "o-", color=col, label=lab)
    ax.axhline(0, color=REF, lw=0.8)
    ax.axvline(HIGH_K * sd, color=REF, lw=0.8, ls="--")
    ax.set_xlabel("rear kappa excursion above straight median [pp]; dashed = 10 x straight sd", color=INK)
    ax.set_ylabel("mean production residual (folded) [% of D]", color=INK)
    ax.grid(color=BAND, lw=0.6)
    ax.legend(frameon=False, fontsize=8)
    fig.suptitle(f"Q2d {key} rear: production-fit residual vs measured kappa (Option-2 signature)", color=INK, fontsize=10)
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    run()
