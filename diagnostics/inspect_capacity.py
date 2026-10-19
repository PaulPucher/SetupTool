# Q-CAPACITY -- does the observer leave the bound where the measured demand
# reaches the fitted grip? (PLAN.md queue entry; author order and EXECUTE
# 2026-10-19.) Read-only; fit L (C = 1.0, E = -1, weighted, D4-L) as
# implemented, mode set in memory; no module or config change.
#
# FROZEN FRAME (written before any number exists)
#
# SESSIONS: NBR-73-010-026-JDR, MLA-SE07_US-v2, NBR JST 29.
# DEMAND per axle = |Fy_axle| / Fz_axle on L's own inputs (front in the tyre
#   frame, Goblirsch et al. 2025 Eq. 13; measured Fz; both 2 Hz).
# 1. DISTRIBUTION on the verdict population (moving, analysis laps, not kerb
#   mask, not smear zone): p95 / p99 / p99.5 / max of demand against the
#   fitted mu (L chain run), and the share of samples with demand > mu.
# 2. EVENTS: the hard-bound re-entries of the L run (inspect_obs_reentry
#   definition: base-mask samples with |raw beta| > bound); per event the
#   maximum demand on each axle in [event - 0.5 s, event].
# H-CAP (frozen): event share = share of events with demand >= mu_fit on
#   either axle within that window; base rate = the same window test on
#   every base-mask sample of the session; CONFIRMED if event share >= 80 %
#   and enrichment (event share / base rate) >= 2, REFUTED if the event share
#   is below 50 % or enrichment <= 1, MIXED otherwise. Scored per session
#   with events (>= 5).
# 3. CONDITIONAL (only if H-CAP is CONFIRMED on every scored session): mu
#   FIXED per axle at the demonstrated utilisation = p99.5 of demand on that
#   axle's fit population (a LOWER bound of the true grip; peak bounded by
#   acquired forces, Farroni et al. 2018 TRIP-ID; conservative grip
#   underestimate, Acosta Reche et al. 2018; chair mue_effective; Oeltjen et
#   al.); C 1.0 and E -1 fixed; B the only free parameter in [5, 40] (bounded
#   Powell, start 12, same inverse-density weights). Injected through the
#   swap harness (tyre_fit_auto._fit_axle_l replaced for the run, restored
#   in finally). Report per session and axle: B, K, mu_demo, in-range curve
#   change vs L C 1.0 (RMS / max over [p1, p99] of the visited slip, as a
#   share of mu_L), D4-L, chain function (status, gate, fallback, R pick),
#   re-entries and CS impact with inspect_obs_reentry's functions unchanged.
#   Physical check, NOT a selection. The percentile p99.5 is frozen here;
#   the maximum is reported beside it.
# RENDERS (render title rule): demand_<key>.png = demand distribution per
#   axle with the fitted mu (and mu_demo when run); events_nbr_jdr.png =
#   measured demand around each JDR event against the L curve.
# Outputs: diagnostics/plots/capacity/ (run.log, capacity.json).
# Run: python -m diagnostics.inspect_capacity

import json
import os
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session, keep_awake  # noqa: E402
from diagnostics.inspect_obs_reentry import run_chain, analyse, cs_impact, window_any  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness  # noqa: E402

OUT_DIR = "diagnostics/plots/capacity"
PRE_S = 0.5
PCT_DEMO = 99.5
MIN_EVENTS = 5
AXLES = ("front", "rear")

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def demo_fitter(alpha, Fy, Fz, base_mask, bin_rad, weighted=True):
    """_fit_axle_l with mu fixed at the demonstrated utilisation (p99.5 of
    |Fy/Fz| on the fit population); B the only free parameter. Same return
    dict, so the chain downstream is production code."""
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz) & (Fz > 0)
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]
    y = f2 / z2
    fz_ref = float(np.median(z2))
    mu = float(np.percentile(np.abs(y), PCT_DEMO))
    _, inv, cnt = np.unique(np.floor(a2 / bin_rad).astype(np.int64), return_inverse=True, return_counts=True)
    w = 1.0 / cnt[inv]

    def _j(p):
        r2 = (pacejka_lateral_force(a2, p[0], tfa.L_SHAPE_C, mu, tfa.L_CURVATURE_E) - y) ** 2
        return float(np.sum(w * r2))

    res = minimize(_j, (tfa.L_START_B,), method="Powell", bounds=[tfa.L_B_BOUNDS])
    B = float(res.x[0])
    shape = pacejka_lateral_force(a2, B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)
    resid = f2 - mu * z2 * shape
    lo, hi = tfa.L_B_BOUNDS
    tol = 1e-4 * (hi - lo)
    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    grid_deg = np.linspace(0.01, 89.9, 4000)
    st = pacejka_lateral_stiffness(np.radians(grid_deg), B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)
    sc = np.where(np.diff(np.sign(st)) < 0)[0]
    peak = float(grid_deg[sc[0]]) if len(sc) else float("nan")
    return {
        "B": B, "C": tfa.L_SHAPE_C, "D": mu * fz_ref, "E": tfa.L_CURVATURE_E,
        "mu": mu, "fz_ref_N": fz_ref, "weighted": True, "mu_fixed_demo_p995": True,
        "bound_active": {"B": "lower" if B - lo <= tol else ("upper" if hi - B <= tol else None), "mu": None},
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
        "fit_rms_resid_norm": float(np.sqrt(np.mean((y - mu * shape) ** 2))),
        "peak_alpha_deg": peak, "peak_in_visited_range": bool(peak == peak and peak <= p99),
        "visited_alpha_p99_deg": p99, "sign_ok": mu > 0,
        "residuals": resid, "residual_mask": m2,
    }


def demand(S):
    out = {}
    for a in AXLES:
        with np.errstate(invalid="ignore", divide="ignore"):
            d = np.abs(S["Fy"][a]) / S["Fz"][a]
        out[a] = np.where(np.isfinite(d) & (S["Fz"][a] > 0), d, np.nan)
    return out


def verdict_pop(S):
    st = S["state"]
    aff = np.zeros(st["time"].size, dtype=bool)
    for k in ("kerb_mask", "kerb_smear_zone"):
        if st.get(k) is not None:
            aff |= st[k]
    return st["moving_mask"] & tfa._analysis_lap_mask(st["time"], S["data"]["laps"]) & ~aff


def run_with(S, fitter):
    orig = tfa._fit_axle_l
    tfa._fit_axle_l = fitter
    try:
        return run_chain(S, S["params"])
    finally:
        tfa._fit_axle_l = orig
        assert tfa._fit_axle_l is orig


def curve_change(S, fit_new, fit_ref, axle):
    pm = S["fit_mask"] & np.isfinite(S["alpha"][axle])
    p1, p99 = np.percentile(S["alpha"][axle][pm], [1, 99])
    g = np.linspace(p1, p99, 2000)
    f = lambda fit: fit["mu"] * pacejka_lateral_force(g, fit["B"], tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)  # noqa: E731
    d = np.abs(f(fit_new) - f(fit_ref)) / fit_ref["mu"]
    return {"rms": float(np.sqrt(np.mean(d ** 2))), "max": float(d.max())}


# ------------------------------------------------------------------ renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def render_demand(S, dem, pop, mus, mus_demo, path):
    plt = _plt()
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5), sharey=True)
    for ax, a in zip(axs, AXLES):
        d = dem[a][pop]
        d = d[np.isfinite(d)]
        ax.hist(d, bins=np.linspace(0, max(2.2, float(np.max(d)) * 1.05), 120), color="#9ca3af")
        ax.axvline(mus[a], color="#2a78d6", lw=2, label=f"fitted friction coefficient {mus[a]:.3f}")
        if mus_demo:
            ax.axvline(mus_demo[a], color="#eb6834", lw=2, ls="--",
                       label=f"demonstrated utilisation, 99.5th percentile {mus_demo[a]:.3f}")
        ax.set_yscale("log")
        ax.set_xlabel(f"{a} axle lateral force / measured axle load (demand) [-]")
        ax.set_title(f"{a} axle")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.25)
    axs[0].set_ylabel("samples (log)")
    fig.suptitle(f"{S['label']}: lateral force demand per axle against the fitted grip, literature-bounded fit\n"
                 f"moving samples on analysis laps, kerb-affected samples excluded")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_events(S, events, fits, path):
    plt = _plt()
    sr = S["state"]["sample_rate_hz"]
    n = int(round(PRE_S * sr))
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), sharey=True)
    for ax, a in zip(axs, AXLES):
        pm = S["fit_mask"] & np.isfinite(S["alpha"][a])
        al = np.degrees(S["alpha"][a][pm])[::10]
        y = (S["Fy"][a][pm] / S["Fz"][a][pm])[::10]
        ax.scatter(al, y, s=1, color="#d1d5db", label="fit population (every 10th sample)")
        g = np.linspace(-15, 15, 600)
        ax.plot(g, fits[a]["mu"] * pacejka_lateral_force(np.radians(g), fits[a]["B"], tfa.L_SHAPE_C, 1.0,
                                                         tfa.L_CURVATURE_E), color="#1f2937", lw=2,
                label="literature-bounded fitted curve")
        for k, e in enumerate(events):
            i = e["i"]
            sl = slice(max(0, i - n), i + 1)
            ax.plot(np.degrees(S["alpha"][a][sl]), S["Fy"][a][sl] / S["Fz"][a][sl], color="#eb6834", lw=1.2,
                    label="half a second before each re-entry" if k == 0 else None)
        ax.set_xlim(-15, 15)
        ax.set_xlabel(f"{a} axle slip angle from the simple geometric estimate [deg]")
        ax.set_title(f"{a} axle")
        ax.grid(alpha=0.25)
    axs[0].set_ylabel("lateral force / measured axle load [-]")
    axs[1].legend(fontsize=8, loc="lower right", markerscale=6)
    fig.suptitle(f"{S['label']}: measured force demand before each observer hard-bound re-entry, "
                 f"literature-bounded fit (shape factor 1.0)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    from modules.stability_analysis import _butterworth_lowpass
    assert tfa.L_SHAPE_C == 1.0
    try:
        R, SS = {}, {}
        for key, label, rel in SESSIONS:
            S = load_session(key, label, rel)
            st = S["state"]
            S["ay_f"] = _butterworth_lowpass(st["ay_mps2"], S["params"]["stability_estimation"]["cs_filter_cutoff_hz"],
                                             st["sample_rate_hz"])
            run = run_chain(S, S["params"])
            fits = {a: run["man"]["axles"][a] for a in AXLES}
            mus = {a: fits[a]["mu"] for a in AXLES}
            dem = demand(S)
            pop = verdict_pop(S)
            rec = {"label": label, "status": run["man"].get("status"), "mu_fit": mus, "distribution": {}}
            for a in AXLES:
                d = dem[a][pop]
                d = d[np.isfinite(d)]
                rec["distribution"][a] = {q: float(np.percentile(d, v)) for q, v in
                                          (("p95", 95), ("p99", 99), ("p99_5", 99.5))}
                rec["distribution"][a].update(max=float(d.max()), share_above_mu=float(np.mean(d > mus[a])), n=int(d.size))
            curves = {"front": (fits["front"]["B"], mus["front"]), "rear": (fits["rear"]["B"], mus["rear"]),
                      "front_peak": fits["front"]["peak_alpha_deg"], "rear_peak": fits["rear"]["peak_alpha_deg"]}
            p99 = {a: fits[a]["visited_alpha_p99_deg"] for a in AXLES}
            A = analyse(S, run, "L", curves, p99)
            n_pre = int(round(PRE_S * st["sample_rate_hz"]))
            over = np.zeros(st["time"].size, dtype=bool)
            for a in AXLES:
                over |= np.nan_to_num(dem[a], nan=0.0) >= mus[a]
            win = window_any(over, n_pre, include_now=True)
            base_rate = float(win[S["base"]].mean())
            evs = A.get("events", [])
            per_event = []
            for e in evs:
                i = e["i"]
                sl = slice(max(0, i - n_pre), i + 1)
                per_event.append({"i": i, "lap": e["lap"], "stable_corner": e["stable_corner"],
                                  **{f"max_demand_{a}": float(np.nanmax(dem[a][sl])) for a in AXLES},
                                  "over": bool(win[i])})
            ev_share = (sum(p["over"] for p in per_event) / len(per_event)) if per_event else float("nan")
            rec["events"] = {"n": len(evs), "per_event": per_event, "event_share_over": ev_share,
                             "base_rate_over": base_rate,
                             "enrichment": ev_share / base_rate if per_event and base_rate > 0 else float("nan")}
            R[key] = rec
            SS[key] = (S, run, A, dem, pop, fits)
            log("=" * 100)
            log(f"{label}: L status {rec['status']}; mu_fit front {mus['front']:.3f} rear {mus['rear']:.3f}")
            for a in AXLES:
                dd = rec["distribution"][a]
                log(f"  demand {a} (n {dd['n']}): p95 {dd['p95']:.3f}, p99 {dd['p99']:.3f}, p99.5 {dd['p99_5']:.3f}, "
                    f"max {dd['max']:.3f}; share above mu_fit {dd['share_above_mu'] * 100:.3f} %")
            ev = rec["events"]
            log(f"  events {ev['n']}; share with demand >= mu_fit within 0.5 s {ev['event_share_over']:.3f}; base rate "
                f"{ev['base_rate_over']:.4f}; enrichment {ev['enrichment']:.2f}")
            for p in per_event:
                log(f"    event lap {p['lap']} corner {p['stable_corner']}: max demand front {p['max_demand_front']:.3f} "
                    f"rear {p['max_demand_rear']:.3f} -> {'>= mu' if p['over'] else '< mu'}")
        scored = {k: R[k]["events"] for k in R if R[k]["events"]["n"] >= MIN_EVENTS}
        verdicts = {}
        for k, ev in scored.items():
            if ev["event_share_over"] >= 0.8 and ev["enrichment"] >= 2:
                verdicts[k] = "CONFIRMED"
            elif ev["event_share_over"] < 0.5 or ev["enrichment"] <= 1:
                verdicts[k] = "REFUTED"
            else:
                verdicts[k] = "MIXED"
        hcap = ("NOT SCORABLE" if not verdicts else "CONFIRMED" if all(v == "CONFIRMED" for v in verdicts.values())
                else "REFUTED" if all(v == "REFUTED" for v in verdicts.values()) else "MIXED")
        log(f"\n## H-CAP: {hcap} -- per scored session {verdicts}")
        cond = {}
        if hcap == "CONFIRMED":
            log("\n## CONDITIONAL: mu fixed at the demonstrated utilisation (p99.5), B free")
            for key, (S, run, A, dem, pop, fits) in SS.items():
                r2 = run_with(S, demo_fitter)
                f2 = {a: r2["man"]["axles"][a] for a in AXLES}
                raw = r2["final"] is not None
                gate = (r2["man"].get("status"), r2["gate"])
                curves2 = {"front": (f2["front"]["B"], f2["front"]["mu"]), "rear": (f2["rear"]["B"], f2["rear"]["mu"]),
                           "front_peak": f2["front"]["peak_alpha_deg"], "rear_peak": f2["rear"]["peak_alpha_deg"]}
                A2 = analyse(S, r2, "demo", curves2, {a: f2[a]["visited_alpha_p99_deg"] for a in AXLES})
                imp = cs_impact(S, r2, A2["ev_mask"])
                c = {"status": r2["man"].get("status"), "reason": r2["man"].get("degenerate_reason"),
                     "gate": None if r2["gate"] is None else {"verdict": r2["gate"]["verdict"],
                                                              "health_score": r2["gate"]["health_score"]},
                     "fallback": r2["fallback"],
                     "R": (r2["man"].get("r_sweep") or {}).get("chosen", {}) and
                          {k2: r2["man"]["r_sweep"]["chosen"][k2] for k2 in ("r_ay_scale", "r_yaw_scale")},
                     "re_entries": A2.get("n_events"), "cs_impact": imp, "axles": {}}
                for a in AXLES:
                    cc = curve_change(S, f2[a], fits[a], a)
                    c["axles"][a] = {"B": f2[a]["B"], "mu_demo": f2[a]["mu"], "K": f2[a]["B"] * f2[a]["C"] * f2[a]["mu"],
                                     "B_bound": f2[a]["bound_active"]["B"], "curve_change": cc}
                d4 = None
                cond[key] = c
                log(f"  {R[key]['label']}: status {c['status']} ({c['reason'] or '-'}); gate {c['gate']}; fallback "
                    f"{c['fallback']}; R {c['R']}; re-entries {c['re_entries']}; CS impact front "
                    f"{imp['f']['share'] * 100:.2f} %, rear {imp['r']['share'] * 100:.2f} %")
                for a in AXLES:
                    x = c["axles"][a]
                    log(f"    {a}: B {x['B']:.3f} (bound {x['B_bound']}), mu_demo {x['mu_demo']:.3f}, K {x['K']:.2f} / rad; "
                        f"in-range change vs L C 1.0 RMS {x['curve_change']['rms'] * 100:.2f} %, max "
                        f"{x['curve_change']['max'] * 100:.2f} % of mu_L")
        else:
            log("\n## CONDITIONAL: not run (H-CAP not confirmed on every scored session)")
        for key, (S, run, A, dem, pop, fits) in SS.items():
            render_demand(S, dem, pop, R[key]["mu_fit"],
                          {a: cond[key]["axles"][a]["mu_demo"] for a in AXLES} if key in cond else None,
                          os.path.join(OUT_DIR, f"demand_{key}.png"))
            if R[key]["events"]["n"]:
                render_events(S, A["events"], fits, os.path.join(OUT_DIR, f"events_{key}.png"))
        with open(os.path.join(OUT_DIR, "capacity.json"), "w", encoding="utf-8") as fh:
            json.dump({"sessions": R, "h_cap": hcap, "per_session": verdicts, "conditional": cond}, fh, indent=1,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    sys.exit(main())
