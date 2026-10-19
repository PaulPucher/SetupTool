# Q-PR-TAIL -- why US-v2 keeps its observer re-entries under the
# demonstrated-utilisation grip rule (PLAN.md queue entry; author order and
# EXECUTE 2026-10-19). Read-only; fit L as implemented (C 1.0, E -1, D4-L)
# with mu fixed per axle at p99.5 of demand on the fit population
# (inspect_capacity.demo_fitter, swap harness, restored in finally).
#
# FROZEN FRAME (written before any number exists)
#
# SESSIONS: MLA-SE07_US-v2 (subject); NBR-73-010-026-JDR, NBR JST 29
#   (controls).
# DEMAND = |Fy_axle| / Fz_axle on L's inputs; verdict population = moving,
#   analysis laps, not kerb mask, not smear zone (inspect_capacity).
# TAIL SAMPLES per axle = verdict-population samples with demand above that
#   axle's p99.5 of the verdict population. EVENT = hard-bound re-entry
#   (inspect_obs_reentry definition) of the demonstrated-mu run.
# H-FORCE: share of the yaw term |Iz psidd / L| (2 Hz, psidd = gradient of
#   the yaw rate as in estimate_lateral_forces) in |Fy_axle|, and on the front
#   of the Fx term |Fx_f sin(delta)| (Eq. 13, L's filtered Fx and delta) in
#   |Fy_f, tyre frame|. SUPPORTED on a session if, for any term and axle, the
#   tail median share >= 2 x the verdict-population median share AND >= 0.10.
# H-KERB-MISS: az deviation |az - kerb_baseline_g| in [0.6, 1.2) g (below the
#   kerb threshold 1.2 g) within +/- 0.2 s. Enrichment = tail share / verdict
#   base share (also for events vs base-mask base share). SUPPORTED if the
#   tail enrichment >= 2 on either axle.
# H-LOC: tail samples (and events) in stable corners where they occur on >=
#   3 distinct analysis laps; enrichment vs the share of verdict samples in
#   those corners. SUPPORTED if the tail enrichment >= 2 on either axle.
# H-TAIL-SHAPE: per axle max / p99.5 and p99.9 / p99.5 of demand (verdict
#   population). US-v2 HEAVIER if on either axle both ratios exceed both
#   controls' ratios.
# US-v2 SPECIFIC = supported (heavier) on US-v2 and not on both controls.
# PREDICTION (frozen): H-KERB-MISS or H-FORCE dominant on US-v2 -- CONFIRMED
#   if either is supported and US-v2 specific; REFUTED if neither is
#   supported on US-v2; MIXED otherwise.
# OUTCOME (frozen): a named mechanism (H-FORCE or H-KERB-MISS, US-v2
#   specific) -> fix proposal next turn (data or mask level, no method
#   change); none shown -> the US-v2 events are recorded as a limitation with
#   their CS impact (inspect_obs_reentry.cs_impact) and the mu rule stands.
# RENDERS (render title rule): usv2_demand_track.png = US-v2 demand over lap
#   distance per axle with tail samples and events marked; usv2_events_curve.png
#   = measured demand before each US-v2 event against the demonstrated-mu curve.
# Outputs: diagnostics/plots/pr_tail/ (run.log, pr_tail.json).
# Run: python -m diagnostics.inspect_pr_tail

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session, keep_awake  # noqa: E402
from diagnostics.inspect_obs_reentry import analyse, cs_impact  # noqa: E402
from diagnostics.inspect_capacity import demo_fitter, run_with, demand, verdict_pop, render_events  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402

OUT_DIR = "diagnostics/plots/pr_tail"
SUBJECT, CONTROLS = "pr_us_v2", ("nbr_jdr", "nbr_jst29")
AXLES = ("front", "rear")
TAIL_PCT = 99.5
AZ_BAND = (0.6, 1.2)
AZ_WIN_S = 0.2
LOC_LAPS = 3

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def dilate(mask, n):
    c = np.concatenate([[0], np.cumsum(mask.astype(np.int64))])
    i = np.arange(mask.size)
    return (c[np.minimum(mask.size, i + n + 1)] - c[np.maximum(0, i - n)]) > 0


def locked_share(S, mask_pts, pop):
    """Share of mask_pts in corners hit on >= LOC_LAPS laps, and the share of pop there."""
    laps_by = {}
    for sid, lap, sl in S["inst"]:
        if mask_pts[sl].any():
            laps_by.setdefault(sid, set()).add(lap)
    locked = {sid for sid, laps in laps_by.items() if len(laps) >= LOC_LAPS}
    in_locked = np.zeros(mask_pts.size, dtype=bool)
    for sid, lap, sl in S["inst"]:
        if sid in locked:
            in_locked[sl] = True
    n = mask_pts.sum()
    return (float(in_locked[mask_pts].mean()) if n else float("nan"), float(in_locked[pop].mean()),
            sorted(locked, key=str))


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    from modules.stability_analysis import _butterworth_lowpass
    try:
        R, SS = {}, {}
        for key, label, rel in SESSIONS:
            S = load_session(key, label, rel)
            st, p = S["state"], S["params"]
            sr, cut = st["sample_rate_hz"], p["stability_estimation"]["cs_filter_cutoff_hz"]
            S["ay_f"] = _butterworth_lowpass(st["ay_mps2"], cut, sr)
            run = run_with(S, demo_fitter)
            fits = {a: run["man"]["axles"][a] for a in AXLES}
            curves = {"front": (fits["front"]["B"], fits["front"]["mu"]), "rear": (fits["rear"]["B"], fits["rear"]["mu"]),
                      "front_peak": fits["front"]["peak_alpha_deg"], "rear_peak": fits["rear"]["peak_alpha_deg"]}
            A = analyse(S, run, "demo", curves, {a: fits[a]["visited_alpha_p99_deg"] for a in AXLES})
            imp = cs_impact(S, run, A["ev_mask"])
            dem = demand(S)
            pop = verdict_pop(S)
            vp = p["vehicle"]
            psidd = np.gradient(st["yaw_rate_radps"], st["time"])
            yaw = np.abs(_butterworth_lowpass(np.where(st["moving_mask"], vp["yaw_inertia_kgm2"] * psidd / vp["wheelbase_m"],
                                                       0.0), cut, sr))
            fx_term = np.abs(S["li"]["fx_f_N"] * np.sin(S["li"]["delta_f"]))
            az = st.get("az_g")
            dev = np.abs(az - p["stability_estimation"]["kerb_baseline_g"]) if az is not None else np.zeros(st["time"].size)
            near = dilate((dev >= AZ_BAND[0]) & (dev < AZ_BAND[1]), int(round(AZ_WIN_S * sr)))
            rec = {"label": label, "status": run["man"].get("status"), "gate": run["gate"], "fallback": run["fallback"],
                   "mu_demo": {a: fits[a]["mu"] for a in AXLES}, "events": A.get("n_events"),
                   "cs_impact": imp, "axles": {}}
            ev_mask = A["ev_mask"]
            base = S["base"]
            rec["events_kerb_miss"] = {"event_share": float(near[ev_mask].mean()) if ev_mask.any() else float("nan"),
                                       "base_share": float(near[base].mean())}
            ev_loc = locked_share(S, ev_mask, base)
            rec["events_loc"] = {"event_share": ev_loc[0], "base_share": ev_loc[1], "corners": ev_loc[2]}
            tails = {}
            for a in AXLES:
                d = np.where(pop, dem[a], np.nan)
                thr = float(np.nanpercentile(d, TAIL_PCT))
                tail = pop & np.isfinite(dem[a]) & (dem[a] > thr)
                tails[a] = tail
                fy = np.abs(S["Fy"][a])
                with np.errstate(invalid="ignore", divide="ignore"):
                    ys = yaw / fy
                    xs = fx_term / fy if a == "front" else None
                terms = {"yaw": (float(np.nanmedian(ys[tail])), float(np.nanmedian(ys[pop])))}
                if xs is not None:
                    terms["fx_sin_delta"] = (float(np.nanmedian(xs[tail])), float(np.nanmedian(xs[pop])))
                dv = d[np.isfinite(d)]
                p995, p999 = np.percentile(dv, [99.5, 99.9])
                loc = locked_share(S, tail, pop)
                rec["axles"][a] = {
                    "tail_threshold": thr, "n_tail": int(tail.sum()),
                    "terms_tail_vs_base": terms,
                    "kerb_miss": {"tail_share": float(near[tail].mean()), "base_share": float(near[pop].mean())},
                    "loc": {"tail_share": loc[0], "base_share": loc[1], "corners": loc[2]},
                    "shape": {"max_over_p995": float(dv.max() / p995), "p999_over_p995": float(p999 / p995)},
                }
            R[key] = rec
            SS[key] = (S, run, A, dem, pop, fits, tails)
            log("=" * 100)
            log(f"{label}: status {rec['status']}; gate {rec['gate'] and {k: rec['gate'][k] for k in ('verdict', 'health_score')}}; "
                f"fallback {rec['fallback']}; mu_demo front {rec['mu_demo']['front']:.3f} rear {rec['mu_demo']['rear']:.3f}; "
                f"re-entries {rec['events']}; CS impact front {imp['f']['share'] * 100:.2f} %, rear {imp['r']['share'] * 100:.2f} %")
            log(f"  events: kerb-miss share {rec['events_kerb_miss']['event_share']:.3f} vs base "
                f"{rec['events_kerb_miss']['base_share']:.3f}; position-locked share {rec['events_loc']['event_share']:.3f} "
                f"vs base {rec['events_loc']['base_share']:.3f} (corners {rec['events_loc']['corners']})")
            for a in AXLES:
                x = rec["axles"][a]
                log(f"  {a}: tail threshold {x['tail_threshold']:.3f} (n {x['n_tail']}); term share tail / base "
                    + ", ".join(f"{t} {v[0]:.3f} / {v[1]:.3f}" for t, v in x["terms_tail_vs_base"].items())
                    + f"; kerb-miss tail {x['kerb_miss']['tail_share']:.3f} / base {x['kerb_miss']['base_share']:.3f}; "
                    f"locked tail {x['loc']['tail_share']:.3f} / base {x['loc']['base_share']:.3f} ({len(x['loc']['corners'])} corners); "
                    f"shape max/p99.5 {x['shape']['max_over_p995']:.3f}, p99.9/p99.5 {x['shape']['p999_over_p995']:.3f}")

        def supported(key):
            out = {}
            ax = R[key]["axles"]
            out["H-FORCE"] = any(tv[0] >= 2 * tv[1] and tv[0] >= 0.10 for a in AXLES for tv in ax[a]["terms_tail_vs_base"].values())
            out["H-KERB-MISS"] = any(ax[a]["kerb_miss"]["base_share"] > 0 and
                                     ax[a]["kerb_miss"]["tail_share"] / ax[a]["kerb_miss"]["base_share"] >= 2 for a in AXLES)
            out["H-LOC"] = any(ax[a]["loc"]["base_share"] > 0 and ax[a]["loc"]["tail_share"] / ax[a]["loc"]["base_share"] >= 2
                               for a in AXLES)
            return out

        sup = {k: supported(k) for k in R}
        heavier = any(all(R[SUBJECT]["axles"][a]["shape"][m] > R[c]["axles"][a]["shape"][m] for m in ("max_over_p995", "p999_over_p995")
                          for c in CONTROLS) for a in AXLES)
        specific = {h: sup[SUBJECT][h] and not all(sup[c][h] for c in CONTROLS) for h in sup[SUBJECT]}
        log("\n## HYPOTHESES")
        for h in ("H-FORCE", "H-KERB-MISS", "H-LOC"):
            log(f"- {h}: US-v2 {'SUPPORTED' if sup[SUBJECT][h] else 'not supported'}; controls "
                f"{ {c: sup[c][h] for c in CONTROLS} }; US-v2 specific {specific[h]}")
        log(f"- H-TAIL-SHAPE: US-v2 heavier than both controls on an axle: {heavier}")
        fk = [h for h in ("H-KERB-MISS", "H-FORCE") if sup[SUBJECT][h]]
        pred = ("CONFIRMED" if any(specific[h] for h in fk) else "REFUTED" if not fk else "MIXED")
        log(f"\n## PREDICTION (H-KERB-MISS or H-FORCE dominant on US-v2): {pred}")
        named = [h for h in ("H-FORCE", "H-KERB-MISS") if specific[h]]
        outcome = (f"named mechanism {named} -> fix proposal next turn" if named else
                   "NOT SHOWN -> US-v2 events recorded as a limitation with their CS impact; the mu rule stands")
        log(f"## OUTCOME: {outcome}")
        S, run, A, dem, pop, fits, tails = SS[SUBJECT]
        render_track(S, dem, pop, tails, A, os.path.join(OUT_DIR, "usv2_demand_track.png"))
        if A.get("events"):
            render_events(S, A["events"], fits, os.path.join(OUT_DIR, "usv2_events_curve.png"))
        with open(os.path.join(OUT_DIR, "pr_tail.json"), "w", encoding="utf-8") as fh:
            json.dump({"sessions": R, "supported": sup, "specific": specific, "tail_heavier": heavier,
                       "prediction": pred, "outcome": outcome}, fh, indent=1,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


def render_track(S, dem, pop, tails, A, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    s_m = S["state"]["s_m"]
    fig, axs = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    ev = np.array([e["i"] for e in A.get("events", [])], dtype=int)
    for ax, a in zip(axs, AXLES):
        ax.scatter(s_m[pop][::5], dem[a][pop][::5], s=1, color="#d1d5db", label="verdict samples (every 5th)")
        ax.scatter(s_m[tails[a]], dem[a][tails[a]], s=6, color="#2a78d6", label="tail samples (above the 99.5th percentile)")
        if ev.size:
            ax.scatter(s_m[ev], dem[a][ev], s=40, marker="x", color="#eb6834", label="observer hard-bound re-entries")
        ax.set_ylabel(f"{a} axle lateral force / measured axle load [-]")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc="upper right", markerscale=3)
    axs[1].set_xlabel("lap distance [m]")
    fig.suptitle(f"{S['label']}: lateral force demand over lap distance, literature-bounded fit with demonstrated grip\n"
                 f"all analysis laps overlaid")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
