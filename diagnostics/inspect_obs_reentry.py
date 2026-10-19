# Q-OBS-REENTRY -- where and why the observer leaves the 15 deg hard bound
# under fit L (PLAN.md queue entry; author order and EXECUTE 2026-10-18).
# Read-only; mode literature_bounded (weighted, with the redesigned D4) set
# in memory, production run beside it for change tracking only (no verdict).
#
# FROZEN FRAME (written before any number exists)
#
# SESSIONS: NBR-73-010-026-JDR, MLA-SE07_US-v2, NBR JST 29.
# EVENT: one per base-mask sample of the final observer run with |raw beta|
#   > beta_hard_bound_deg (the re-entry count of inspect_fit_lit); events
#   closer than 0.5 s are also grouped into clusters for the count.
# PER EVENT: observer slip alpha_f / alpha_r (from its own raw beta and yaw
#   rate state, alpha_f = delta - beta - a r / Vx, alpha_r = -beta + b r /
#   Vx, Vx floored as in the observer) at the event and in the 0.5 s
#   before; local stiffness of the L curve relative to its origin slope,
#   s(alpha) = f'(alpha) / (B C) (load cancels); |alpha| relative to the
#   axle's visited p99 |alpha| (kinematic fit population) and to the curve
#   peak; speed, |a_y| (2 Hz), |a_x|, kerb mask / smear zone, time since the
#   containing corner instance's bracket start, stable corner id and lap;
#   NIS mean / max in the 0.5 s before.
# CATEGORIES (an event may be in several; base rate = the same test on
#   every base-mask sample of the session):
#   H-OBS  (loss of observability on the extrapolated curve; Farrelly &
#          Wellstead 1996; Repka et al. 2025; Bertipaglia et al. 2024): in
#          the 0.5 s before, on either axle, |alpha| > visited p99 |alpha|
#          or s(alpha) < 0.2.
#   H-KERB kerb mask or smear zone within the 0.5 s before or at the event
#          (the base mask itself excludes both, so only the approach counts).
#   H-LOWV speed below the session's p10 base-mask speed, or a straight
#          (|a_y| < 1 m/s^2), at the event.
#   H-LOC  inside a stable corner that has events on >= 3 distinct laps
#          (base: base-mask samples inside those corners' brackets).
# SCORING: enrichment = event share / base share per session; sessions with
#   fewer than 5 events are not scored. Per hypothesis CONFIRMED if
#   enrichment >= 2 on every scored session, REFUTED if <= 1 on every scored
#   session, MIXED otherwise; NOT SCORABLE without a scored session.
# IMPACT ON CS: verdict population per axle (finite CS inside canonical
#   brackets on analysis laps, moving, not kerb mask, not the axle's
#   kerb-affected mark); a sample is affected when its regression window
#   [start, i) (stability_analysis.reconstruct_cs_window_start, the CS
#   loop's own rule) contains an event or the 1 s after one. Share per axle
#   and session, L and production.
# OUTCOME (frozen): impact < 1 % of verdict CS samples on every session and
#   axle (L) -> documented limitation, no observer change; otherwise -> the
#   mechanism with evidence and a literature-anchored fix proposal next turn
#   (no fix this cycle).
# RENDERS (render title rule): timeline_<key>.png = events over lap distance
#   x lap (L and production); slip_<key>.png = observer slip just before each
#   L event against the L curve, visited range shaded, coloured by category.
# Outputs: diagnostics/plots/obs_reentry/ (run.log, JSON).
# Run: python -m diagnostics.inspect_obs_reentry

import copy
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session, capture, keep_awake  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness  # noqa: E402

OUT_DIR = "diagnostics/plots/obs_reentry"
PRE_S, POST_S, CLUSTER_S = 0.5, 1.0, 0.5
OBS_STIFF = 0.2
STRAIGHT_AY = 1.0
LOC_LAPS = 3
MIN_EVENTS = 5
CATS = ("H-OBS", "H-KERB", "H-LOWV", "H-LOC")
CAT_COLOUR = {"H-OBS": "#2a78d6", "H-KERB": "#eb6834", "H-LOWV": "#1baf7a", "none": "#9ca3af"}
CAT_WORDS = {"H-OBS": "slip beyond the visited range or near the curve peak", "H-KERB": "kerb strike just before",
             "H-LOWV": "low speed or straight", "none": "none of these"}

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def run_chain(S, params):
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness
    holder = {}
    with capture(holder, True):
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(S["state"], params, S["data"], "ekf_auto_pacejka",
                                                             csv_path=S["path"])
    slip = estimate_slip_angles(S["state"], beta, params)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(S["state"], params), S["state"], params,
                                      S["data"]["laps"])
    return {"beta": beta, "man": man, "gate": gate, "fallback": fb, "final": holder.get("final"), "slip": slip, "cs": cs}


def window_any(mask, n_before, include_now=True):
    """True where mask is set anywhere in [i - n_before, i] (or [i - n_before, i))."""
    c = np.concatenate([[0], np.cumsum(mask.astype(np.int64))])
    i = np.arange(mask.size)
    lo = np.maximum(0, i - n_before)
    hi = i + 1 if include_now else i
    return (c[hi] - c[lo]) > 0


def window_reduce(x, n_before, fn):
    out = np.full(x.size, np.nan)
    for i in range(x.size):
        seg = x[max(0, i - n_before):i]
        seg = seg[np.isfinite(seg)]
        if seg.size:
            out[i] = fn(seg)
    return out


def observer_slip(S, final):
    st, vp = S["state"], S["params"]["vehicle"]
    vx = np.maximum(st["v_mps"], S["params"]["stability_estimation"]["moving_speed_min_mps"])
    beta, r = final["beta"], final["yaw_rate_state"]
    a, b = vp["cog_to_front_axle_m"], vp["cog_to_rear_axle_m"]
    return {"front": st["delta_f_rad"] - beta - a * r / vx, "rear": -beta + b * r / vx}


def analyse(S, run, label_run, curves, p99):
    st, base = S["state"], S["base"]
    sr = st["sample_rate_hz"]
    n_pre = int(round(PRE_S * sr))
    fin = run["final"]
    if fin is None:
        return {"note": f"{label_run}: no observer run (fit returned early)", "events": []}
    bound = np.radians(S["params"]["tyre_fit_auto"]["beta_hard_bound_deg"])
    ev_mask = (np.abs(fin["beta"]) > bound) & base
    ev = np.flatnonzero(ev_mask)
    t = st["time"]
    alpha = observer_slip(S, fin)
    # category tests on every sample (base rate) and at the events
    obs_now = np.zeros(t.size, dtype=bool)
    for ax in ("front", "rear"):
        B, mu = curves[ax]
        s_rel = pacejka_lateral_stiffness(alpha[ax], B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E) / (B * tfa.L_SHAPE_C)
        obs_now |= (np.abs(np.degrees(alpha[ax])) > p99[ax]) | (s_rel < OBS_STIFF)
    obs = window_any(obs_now, n_pre, include_now=False)
    kerb = np.zeros(t.size, dtype=bool)
    for k in ("kerb_mask", "kerb_smear_zone"):
        if st.get(k) is not None:
            kerb |= st[k]
    kerb_w = window_any(kerb, n_pre, include_now=True)
    v = st["v_mps"]
    v10 = float(np.percentile(v[base], 10))
    ayf = S["ay_f"]
    lowv = (v < v10) | (np.abs(ayf) < STRAIGHT_AY)
    # instance lookup
    inst_of = np.full(t.size, -1, dtype=np.int64)
    for j, (_sid, _lap, sl) in enumerate(S["inst"]):
        inst_of[sl] = j
    nis_mean = window_reduce(fin["nis"], n_pre, np.mean) if ev.size else None
    nis_max = window_reduce(fin["nis"], n_pre, np.max) if ev.size else None
    laps_by_corner = {}
    for i in ev:
        j = inst_of[i]
        if j >= 0:
            sid, lap, _ = S["inst"][j]
            laps_by_corner.setdefault(sid, set()).add(lap)
    locked = {sid for sid, laps in laps_by_corner.items() if len(laps) >= LOC_LAPS}
    in_locked = np.zeros(t.size, dtype=bool)
    for sid, lap, sl in S["inst"]:
        if sid in locked:
            in_locked[sl] = True
    cat_mask = {"H-OBS": obs, "H-KERB": kerb_w, "H-LOWV": lowv, "H-LOC": in_locked}
    base_share = {c: float(m[base].mean()) for c, m in cat_mask.items()}
    events = []
    for i in ev:
        j = inst_of[i]
        rec = {"i": int(i), "t": float(t[i]), "s_m": float(st["s_m"][i]) if st.get("s_m") is not None else None,
               "lap": None, "stable_corner": None, "since_entry_s": None,
               "v_mps": float(v[i]), "abs_ay": float(abs(ayf[i])), "abs_ax": float(abs(st["ax_mps2"][i])),
               "kerb_window": bool(kerb_w[i]), "nis_pre_mean": float(nis_mean[i]), "nis_pre_max": float(nis_max[i]),
               "cats": [c for c in CATS if cat_mask[c][i]]}
        if j >= 0:
            sid, lap, sl = S["inst"][j]
            rec.update(lap=int(lap), stable_corner=sid, since_entry_s=float(t[i] - t[sl.start]))
        for ax in ("front", "rear"):
            B, mu = curves[ax]
            lo = max(0, i - n_pre)
            pre = alpha[ax][lo:i]
            a_i = float(np.degrees(alpha[ax][i]))
            pre_max = float(np.degrees(np.max(np.abs(pre)))) if pre.size else float("nan")
            a_last = float(np.degrees(alpha[ax][i - 1])) if i > 0 else float("nan")
            s_last = float(pacejka_lateral_stiffness(np.radians(a_last), B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)
                           / (B * tfa.L_SHAPE_C))
            rec[ax] = {"alpha_event_deg": a_i, "alpha_last_deg": a_last, "pre_max_abs_deg": pre_max,
                       "stiff_rel_last": s_last, "pre_over_p99": pre_max / p99[ax],
                       "pre_over_peak": pre_max / curves[ax + "_peak"] if curves[ax + "_peak"] == curves[ax + "_peak"] else None}
        events.append(rec)
    clusters = 0
    last_t = -1e9
    for e in events:
        if e["t"] - last_t > CLUSTER_S:
            clusters += 1
        last_t = e["t"]
    ev_share = {c: (sum(c in e["cats"] for e in events) / len(events)) if events else float("nan") for c in CATS}
    enrich = {c: (ev_share[c] / base_share[c]) if events and base_share[c] > 0 else float("nan") for c in CATS}
    return {"n_events": len(events), "n_clusters": clusters, "events": events, "event_share": ev_share,
            "base_share": base_share, "enrichment": enrich, "locked_corners": sorted(locked, key=str),
            "v_p10_mps": v10, "ev_mask": ev_mask}


def cs_impact(S, run, ev_mask):
    from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples
    st, se = S["state"], S["params"]["stability_estimation"]
    sr = st["sample_rate_hz"]
    n_post = int(round(POST_S * sr))
    # event or the 1 s after it
    c = np.concatenate([[0], np.cumsum(ev_mask.astype(np.int64))])
    i = np.arange(ev_mask.size)
    after = (c[i + 1] - c[np.maximum(0, i - n_post)]) > 0
    ca = np.concatenate([[0], np.cumsum(after.astype(np.int64))])
    min_window = resolve_cs_min_window_samples(S["params"], sr)
    out = {}
    cs = run["cs"]
    kerb = st.get("kerb_mask")
    for ax, key in (("f", "alpha_f_filt"), ("r", "alpha_r_filt")):
        pop = S["in_br"] & st["moving_mask"] & ~cs[f"near_strike_{ax}"] & np.isfinite(cs[f"CS_ratio_{ax}"])
        if kerb is not None:
            pop &= ~kerb
        idx = np.flatnonzero(pop)
        alpha = run["slip"][key]
        hit = 0
        for k in idx:
            s0 = reconstruct_cs_window_start(alpha, k, min_window, se["cs_min_slip_angle_span_rad"],
                                             st.get("s_m"), se["cs_max_window_m"])
            if ca[k] - ca[s0] > 0:
                hit += 1
        out[ax] = {"n_verdict": int(idx.size), "n_affected": hit, "share": hit / idx.size if idx.size else float("nan")}
    return out


def score(R):
    res = {}
    for c in CATS:
        vals = [R[k]["L"]["enrichment"][c] for k in R if R[k]["L"]["n_events"] >= MIN_EVENTS]
        vals = [v for v in vals if v == v]
        if not vals:
            res[c] = ("NOT SCORABLE", vals)
        elif all(v >= 2 for v in vals):
            res[c] = ("CONFIRMED", vals)
        elif all(v <= 1 for v in vals):
            res[c] = ("REFUTED", vals)
        else:
            res[c] = ("MIXED", vals)
    return res


# ------------------------------------------------------------------ renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def render_timeline(S, A, path):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for name, col, mk in (("L", "#2a78d6", "o"), ("P", "#6b7280", "x")):
        evs = A[name]["events"]
        xs = [e["s_m"] for e in evs if e["lap"] is not None]
        ys = [e["lap"] for e in evs if e["lap"] is not None]
        lab = ("literature-bounded fit" if name == "L" else "production fit") + f" ({A[name]['n_events']} re-entries)"
        ax.scatter(xs, ys, s=30 if name == "L" else 40, color=col, marker=mk, label=lab, alpha=0.8)
    for sid, lap, sl in S["inst"]:
        pass
    ax.set_xlabel("lap distance [m]")
    ax.set_ylabel("lap")
    ax.set_title(f"{S['label']}: observer hard-bound re-entries (sideslip beyond 15 deg) over lap distance\n"
                 f"only re-entries inside a corner bracket carry a lap and are drawn")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_slip(S, A, curves, p99, path):
    plt = _plt()
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), sharey=True)
    for ax, axle in zip(axs, ("front", "rear")):
        B, mu = curves[axle]
        g = np.linspace(-25, 25, 1000)
        ax.axvspan(-p99[axle], p99[axle], color="#e5e7eb", alpha=0.7, lw=0, label="visited slip range (99 %)")
        ax.plot(g, mu * pacejka_lateral_force(np.radians(g), B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E),
                color="#1f2937", lw=2, label="literature-bounded fitted curve")
        drawn = set()
        for e in A["L"]["events"]:
            cat = next((c for c in ("H-OBS", "H-KERB", "H-LOWV") if c in e["cats"]), "none")
            a = e[axle]["alpha_last_deg"]
            y = mu * pacejka_lateral_force(np.radians(a), B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)
            ax.scatter([a], [y], s=28, color=CAT_COLOUR[cat], label=None if cat in drawn else CAT_WORDS[cat], zorder=3)
            drawn.add(cat)
        ax.set_xlim(-25, 25)
        ax.set_xlabel(f"{axle} axle slip angle of the observer just before the re-entry [deg]")
        ax.set_title(f"{axle} axle")
        ax.grid(alpha=0.25)
    axs[0].set_ylabel("lateral force / axle load on the fitted curve [-]")
    axs[1].legend(fontsize=8, loc="lower right")
    fig.suptitle(f"{S['label']}: observer slip angle just before each hard-bound re-entry, literature-bounded fit\n"
                 f"colour = first matching cause category")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    from modules.stability_analysis import _butterworth_lowpass
    try:
        R = {}
        for key, label, rel in SESSIONS:
            S = load_session(key, label, rel)
            st = S["state"]
            S["ay_f"] = _butterworth_lowpass(st["ay_mps2"], S["params"]["stability_estimation"]["cs_filter_cutoff_hz"],
                                             st["sample_rate_hz"])
            runs = {"L": run_chain(S, S["params"])}
            pp = copy.deepcopy(S["params"])
            pp["tyre_fit_auto"]["tyre_fit_mode"] = "per_session"
            runs["P"] = run_chain(S, pp)
            fitL = runs["L"]["man"]["axles"]
            curves = {"front": (fitL["front"]["B"], fitL["front"]["mu"]), "rear": (fitL["rear"]["B"], fitL["rear"]["mu"]),
                      "front_peak": fitL["front"]["peak_alpha_deg"], "rear_peak": fitL["rear"]["peak_alpha_deg"]}
            p99 = {a: fitL[a]["visited_alpha_p99_deg"] for a in ("front", "rear")}
            A = {name: analyse(S, runs[name], name, curves, p99) for name in ("L", "P")}
            imp = {name: cs_impact(S, runs[name], A[name]["ev_mask"]) for name in ("L", "P")}
            R[key] = {"label": label, "status": {n: runs[n]["man"].get("status") for n in runs},
                      "L": {k: v for k, v in A["L"].items() if k != "ev_mask"},
                      "P": {k: v for k, v in A["P"].items() if k != "ev_mask"}, "cs_impact": imp,
                      "p99_deg": p99, "peak_deg": {a: curves[a + "_peak"] for a in ("front", "rear")}}
            render_timeline(S, A, os.path.join(OUT_DIR, f"timeline_{key}.png"))
            render_slip(S, A, curves, p99, os.path.join(OUT_DIR, f"slip_{key}.png"))
            log("=" * 100)
            log(f"{label}: status L {runs['L']['man'].get('status')} / P {runs['P']['man'].get('status')}; "
                f"visited p99 front {p99['front']:.2f} rear {p99['rear']:.2f} deg; L peak front "
                f"{curves['front_peak']:.2f} rear {curves['rear_peak']:.2f} deg")
            for name in ("L", "P"):
                a = R[key][name]
                if "n_events" not in a:
                    log(f"  {name}: {a.get('note')}")
                    continue
                log(f"  {name}: re-entries {a['n_events']} in {a['n_clusters']} clusters; locked corners "
                    f"{a['locked_corners']}; base p10 speed {a['v_p10_mps'] * 3.6:.0f} km/h")
                log("    event share / base share / enrichment: " + "; ".join(
                    f"{c} {a['event_share'][c]:.2f} / {a['base_share'][c]:.3f} / {a['enrichment'][c]:.2f}"
                    for c in CATS))
                ci = imp[name]
                log(f"    CS impact: front {ci['f']['n_affected']} / {ci['f']['n_verdict']} = {ci['f']['share'] * 100:.2f} %, "
                    f"rear {ci['r']['n_affected']} / {ci['r']['n_verdict']} = {ci['r']['share'] * 100:.2f} %")
            evs = R[key]["L"].get("events", [])
            if evs:
                def med(fn):
                    x = [fn(e) for e in evs]
                    x = [v for v in x if v is not None and v == v]
                    return float(np.median(x)) if x else float("nan")
                log(f"    L events, medians: speed {med(lambda e: e['v_mps']) * 3.6:.0f} km/h, |ay| "
                    f"{med(lambda e: e['abs_ay']):.1f}, |ax| {med(lambda e: e['abs_ax']):.1f} m/s^2, since corner entry "
                    f"{med(lambda e: e['since_entry_s']):.2f} s, NIS pre mean {med(lambda e: e['nis_pre_mean']):.1f} / max "
                    f"{med(lambda e: e['nis_pre_max']):.1f}; front pre max |alpha| / p99 {med(lambda e: e['front']['pre_over_p99']):.2f}, "
                    f"rear {med(lambda e: e['rear']['pre_over_p99']):.2f}; front stiffness just before "
                    f"{med(lambda e: e['front']['stiff_rel_last']):.2f}, rear {med(lambda e: e['rear']['stiff_rel_last']):.2f} "
                    f"of the origin slope; in a corner {sum(e['lap'] is not None for e in evs)} / {len(evs)}")
        sc = score(R)
        log("\n## HYPOTHESES (L)")
        for c, (v, vals) in sc.items():
            log(f"- {c}: {v} -- enrichment on scored sessions {[round(x, 2) for x in vals]}")
        worst = max(R[k]["cs_impact"]["L"][ax]["share"] for k in R for ax in ("f", "r"))
        outcome = ("documented limitation, no observer change" if worst < 0.01
                   else "mechanism with evidence and a fix proposal next turn")
        log(f"\n## OUTCOME: largest L CS impact {worst * 100:.2f} % -> {outcome}")
        with open(os.path.join(OUT_DIR, "obs_reentry.json"), "w", encoding="utf-8") as fh:
            json.dump({"sessions": R, "hypotheses": sc, "outcome": outcome, "worst_impact": worst}, fh, indent=1,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    sys.exit(main())
