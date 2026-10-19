# Q-PR-ANOMALY -- why the US-v2 rear axle shows a low normalised origin
# slope (K 15.9 / rad vs 23.5-26.9 on the other five axles of the Q-FIT-LIT
# cycle) and a low-speed residual bias (PLAN.md queue entry; author order and
# EXECUTE 2026-10-18, addendum to item 0 the same day). Read-only.
#
# FROZEN FRAME (written before any number exists)
#
# SESSIONS: MLA-SE07_US-v2 (subject); NBR-73-010-026-JDR, NBR JST 29
#   (controls).
# INPUTS exactly as fit L builds them (inspect_fit_lit.load_session): fit
#   population = base mask x speed window, kinematic-seed alpha (2 Hz), Fy
#   front in the tyre frame (Goblirsch et al. 2025 Eq. 13, Eq. 9 Fx), rear
#   body Fy, measured axle Fz (2 Hz). CURVES = L's fitted B, mu per axle
#   from diagnostics/plots/fit_lit/fit_lit_<key>.json (the cycle's run), no
#   refit. Normalised residual r = Fy/Fz - mu f(alpha; B, C 1.3, E -1).
# UNIT OF ANALYSIS: corner instance (stable corner x analysis lap) --
#   samples are autocorrelated. Per instance and axle: the fit-population
#   samples inside its canonical bracket, >= 20 required, else dropped.
#   Instance residual = mean of r x s, s = sign of the instance's median
#   Fy/Fz (positive = measured |Fy/Fz| above the curve, independent of the
#   turn direction).
# 0. K CHECK: OLS slope of Fy/Fz on alpha over |alpha| < 2 deg of the fit
#   population, per session and axle, and (addendum) per analysis lap and
#   axle. laps_used per session = the fit manifest's definition (sorted
#   analysis-valid lap numbers; the run JSON does not store the manifest).
# H-Fz (load side): (a) per speed class, median measured axle Fz / median
#   quasi-static model Fz (estimate_vertical_loads with vertical_load_source
#   "static", same config), fit population; (b) wheel-sum sanity on
#   straights (base mask outside every canonical bracket): measured sum of
#   the four wheels / (m g + c_session v^2), median and p5 / p95;
#   (c) factor per instance: mean measured / model axle Fz.
# H-Fy (force side): factor per instance: mean |yaw term| / mean |Fy_axle|,
#   yaw term = Iz psidd / L (2 Hz, the Fy filter), psidd = gradient of the
#   yaw rate as in estimate_lateral_forces.
# H-alpha (slip side): factors per instance: corner duration [s]; time since
#   the last straight [s] at the instance midpoint, straight sample = |a_y|
#   (2 Hz) < 1 m/s^2 (washout high-pass 0.05 Hz, tau ~3.2 s); mean |a_x|
#   (combined slip); kerb-affected share (kerb mask or smear zone) of the
#   instance's moving samples.
# SCORING per session and axle: Spearman rho of each factor with the
#   instance residual (scipy.stats.spearmanr). A factor is ASSOCIATED if
#   |rho| >= 0.3 and p < 0.01. US-v2 DIFFERS on a factor if its median lies
#   outside the interquartile range of BOTH controls. A hypothesis is
#   SUPPORTED for the US-v2 rear if one of its factors is associated there
#   and (not associated on both control rears, or US-v2 differs on it).
# PREDICTION (frozen): H-alpha dominant on US-v2 slow corners, H-Fz and H-Fy
#   secondary. CONFIRMED if H-alpha is supported for the US-v2 rear, its
#   largest |rho| there exceeds those of H-Fz and H-Fy, and the low speed
#   class carries the most negative mean instance residual of the three
#   classes on the US-v2 rear; REFUTED if H-alpha is not supported; MIXED
#   otherwise.
# OUTCOME (frozen): a named input mechanism with its evidence -> a separate
#   fix proposal next turn (no fix in this cycle); none shown -> "not
#   shown", L's negative stands as scoped.
# RENDERS (render title rule): factors_<key>.png = per-instance residual vs
#   each factor, front and rear; usv2_rear_cloud_by_speed.png = US-v2 rear
#   slip angle vs Fy/Fz coloured by corner speed class with L's curve.
# Outputs: diagnostics/plots/pr_anomaly/ (run.log, JSON).
# Run: python -m diagnostics.inspect_pr_anomaly

import copy
import json
import os
import sys

import numpy as np
from scipy.stats import spearmanr

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force  # noqa: E402

OUT_DIR = "diagnostics/plots/pr_anomaly"
FIT_DIR = "diagnostics/plots/fit_lit"
SUBJECT, CONTROLS = "pr_us_v2", ("nbr_jdr", "nbr_jst29")
LIN_DEG = 2.0
MIN_SAMPLES = 20
STRAIGHT_AY = 1.0
ASSOC_RHO, ASSOC_P = 0.3, 0.01
HYP = {"H-Fz": ("fz_ratio",), "H-Fy": ("yaw_share",),
       "H-alpha": ("duration_s", "since_straight_s", "mean_abs_ax", "kerb_share")}
FACTOR_WORDS = {"fz_ratio": "measured / quasi-static axle load", "yaw_share": "yaw-acceleration share of axle force",
                "duration_s": "corner duration [s]", "since_straight_s": "time since the last straight [s]",
                "mean_abs_ax": "mean |longitudinal acceleration| [m/s^2]", "kerb_share": "kerb-affected share"}
AXLES = ("front", "rear")
CLASS_COLOURS = {"low": "#2a78d6", "medium": "#eb6834", "high": "#1baf7a"}

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def curve(a, B, mu):
    return mu * pacejka_lateral_force(a, B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)


def k_slope(a, y):
    m = np.abs(np.degrees(a)) < LIN_DEG
    if m.sum() < 50:
        return float("nan"), int(m.sum())
    return float(np.polyfit(a[m], y[m], 1)[0]), int(m.sum())


def session_inputs(key, label, rel):
    from modules.stability_analysis import (estimate_vertical_loads, estimate_lateral_forces, load_car_data,
                                            _butterworth_lowpass)
    S = load_session(key, label, rel)
    rec = json.load(open(os.path.join(FIT_DIR, f"fit_lit_{key}.json"), encoding="utf-8"))
    S["curves"] = {a: (rec["L"]["physical"][a]["B"], rec["L"]["physical"][a]["mu"]) for a in AXLES}
    st, p = S["state"], S["params"]
    sr, cut = st["sample_rate_hz"], p["stability_estimation"]["cs_filter_cutoff_hz"]
    forces = estimate_lateral_forces(st, p)
    p_static = copy.deepcopy(p)
    p_static["stability_estimation"]["vertical_load_source"] = "static"
    model = estimate_vertical_loads(st, forces, p_static)
    meas = estimate_vertical_loads(st, forces, p, channels=S["data"]["channels"], car_data=load_car_data())
    S["fz_model"] = {"front": _butterworth_lowpass(model["fz_f_N"], cut, sr),
                     "rear": _butterworth_lowpass(model["fz_r_N"], cut, sr)}
    S["meas"] = meas
    vp = p["vehicle"]
    psidd = np.gradient(st["yaw_rate_radps"], st["time"])
    S["yaw_term"] = _butterworth_lowpass(np.where(st["moving_mask"], vp["yaw_inertia_kgm2"] * psidd / vp["wheelbase_m"],
                                                  0.0), cut, sr)
    S["Fy_body"] = {"front": forces["Fy_f_filt"], "rear": forces["Fy_r_filt"]}
    S["ay_f"] = _butterworth_lowpass(st["ay_mps2"], cut, sr)
    S["ax"] = st["ax_mps2"]
    kerb = st.get("kerb_mask")
    smear = st.get("kerb_smear_zone")
    aff = np.zeros(st["time"].size, dtype=bool)
    if kerb is not None:
        aff |= kerb
    if smear is not None:
        aff |= smear
    S["kerb_aff"] = aff
    S["laps_used"] = sorted(l["lap_number"] for l in S["data"]["laps"] if l.get("is_valid_for_analysis"))
    S["corner_class"] = {}
    for c in S["data"]["corners"]:
        S["corner_class"][(c.get("stable_corner_id"), c.get("lap_number"))] = str(c.get("speed_class"))
    return S


def k_check(S):
    t = S["state"]["time"]
    out = {"session": {}, "per_lap": {}}
    for a in AXLES:
        pop = tfa_fit_pop(S, a)
        al, y = S["alpha"][a][pop], S["Fy"][a][pop] / S["Fz"][a][pop]
        out["session"][a] = k_slope(al, y)
        out["per_lap"][a] = {}
        for lap in S["data"]["laps"]:
            if not lap.get("is_valid_for_analysis"):
                continue
            lm = pop & (t >= lap["start_time"]) & (t <= lap["end_time"])
            out["per_lap"][a][lap["lap_number"]] = k_slope(S["alpha"][a][lm], S["Fy"][a][lm] / S["Fz"][a][lm])
    return out


def tfa_fit_pop(S, a):
    al, f, z = S["alpha"][a], S["Fy"][a], S["Fz"][a]
    return S["fit_mask"] & np.isfinite(al) & np.isfinite(f) & np.isfinite(z) & (z > 0)


def h_fz_session(S):
    st, meas = S["state"], S["meas"]
    out = {"by_class": {}}
    for a in AXLES:
        pop = tfa_fit_pop(S, a)
        out["by_class"][a] = {}
        for cls in ("low", "medium", "high"):
            m = pop & (S["speed_class"] == cls)
            if m.sum() < MIN_SAMPLES:
                continue
            out["by_class"][a][cls] = float(np.median(S["Fz"][a][m]) / np.median(S["fz_model"][a][m]))
    c = meas.get("c_session_N_per_mps2")
    s = S["straight"]
    tot = meas["fz_fl_N"] + meas["fz_fr_N"] + meas["fz_rl_N"] + meas["fz_rr_N"]
    if c is not None and s.any():
        ref = S["params"]["vehicle"]["mass_kg"] * 9.81 + c * st["v_mps"] ** 2
        r = tot[s] / ref[s]
        out["wheel_sum_straights"] = {"c_session": float(c), "median": float(np.median(r)),
                                      "p5": float(np.percentile(r, 5)), "p95": float(np.percentile(r, 95)),
                                      "n": int(s.sum())}
    else:
        out["wheel_sum_straights"] = {"c_session": c, "note": "no c_session or no straight samples"}
    return out


def since_straight(S):
    st = S["state"]
    t = st["time"]
    straight = np.abs(S["ay_f"]) < STRAIGHT_AY
    idx = np.where(straight, np.arange(t.size), -1)
    last = np.maximum.accumulate(idx)
    out = np.full(t.size, np.nan)
    ok = last >= 0
    out[ok] = t[ok] - t[last[ok]]
    return out


def instances(S):
    t = S["state"]["time"]
    since = since_straight(S)
    rows = []
    for sid, lap, sl in S["inst"]:
        cls = S["corner_class"].get((sid, lap), "")
        mid = (sl.start + sl.stop) // 2
        moving = S["state"]["moving_mask"][sl]
        base = {"sid": sid, "lap": lap, "speed_class": cls,
                "duration_s": float(t[sl.stop - 1] - t[sl.start]),
                "since_straight_s": float(since[mid]),
                "mean_abs_ax": float(np.mean(np.abs(S["ax"][sl][moving]))) if moving.any() else float("nan"),
                "kerb_share": float(np.mean(S["kerb_aff"][sl][moving])) if moving.any() else float("nan")}
        for a in AXLES:
            pop = tfa_fit_pop(S, a)[sl]
            if pop.sum() < MIN_SAMPLES:
                continue
            al = S["alpha"][a][sl][pop]
            y = S["Fy"][a][sl][pop] / S["Fz"][a][sl][pop]
            B, mu = S["curves"][a]
            s = np.sign(np.median(y)) or 1.0
            fyb = S["Fy_body"][a][sl][pop]
            row = dict(base, axle=a, n=int(pop.sum()),
                       resid=float(np.mean((y - curve(al, B, mu)) * s)),
                       fz_ratio=float(np.mean(S["Fz"][a][sl][pop]) / np.mean(S["fz_model"][a][sl][pop])),
                       yaw_share=float(np.mean(np.abs(S["yaw_term"][sl][pop])) / max(1e-9, np.mean(np.abs(fyb)))))
            rows.append(row)
    return rows


def correlations(rows):
    out = {}
    for a in AXLES:
        rr = [r for r in rows if r["axle"] == a]
        res = np.array([r["resid"] for r in rr])
        out[a] = {"n_instances": len(rr), "factors": {}}
        for fac in FACTOR_WORDS:
            x = np.array([r[fac] for r in rr])
            ok = np.isfinite(x) & np.isfinite(res)
            if ok.sum() < 10 or np.ptp(x[ok]) == 0:
                out[a]["factors"][fac] = {"rho": None, "p": None, "n": int(ok.sum())}
                continue
            rho, pv = spearmanr(x[ok], res[ok])
            out[a]["factors"][fac] = {"rho": float(rho), "p": float(pv), "n": int(ok.sum()),
                                      "associated": bool(abs(rho) >= ASSOC_RHO and pv < ASSOC_P),
                                      "median": float(np.median(x[ok])),
                                      "iqr": [float(np.percentile(x[ok], 25)), float(np.percentile(x[ok], 75))]}
        out[a]["resid_by_class"] = {c: float(np.mean([r["resid"] for r in rr if r["speed_class"] == c]))
                                    for c in ("low", "medium", "high") if any(r["speed_class"] == c for r in rr)}
        out[a]["n_by_class"] = {c: sum(r["speed_class"] == c for r in rr) for c in ("low", "medium", "high")}
    return out


def score(C):
    sub = C[SUBJECT]["rear"]["factors"]
    res = {}
    for h, facs in HYP.items():
        best, sup = 0.0, False
        detail = {}
        for f in facs:
            fs = sub.get(f, {})
            if fs.get("rho") is None:
                continue
            ctrl_assoc = [C[k]["rear"]["factors"][f].get("associated", False) for k in CONTROLS]
            differs = all(not (C[k]["rear"]["factors"][f]["iqr"][0] <= fs["median"] <= C[k]["rear"]["factors"][f]["iqr"][1])
                          for k in CONTROLS if C[k]["rear"]["factors"][f].get("iqr"))
            s = fs["associated"] and (not any(ctrl_assoc) or differs)
            detail[f] = {"rho": fs["rho"], "p": fs["p"], "associated": fs["associated"],
                         "controls_associated": ctrl_assoc, "usv2_differs": differs, "supports": s}
            sup |= s
            best = max(best, abs(fs["rho"]))
        res[h] = {"supported": sup, "max_abs_rho": best, "factors": detail}
    by_cls = C[SUBJECT]["rear"]["resid_by_class"]
    low_most_neg = bool(by_cls) and min(by_cls, key=by_cls.get) == "low"
    ha = res["H-alpha"]
    if not ha["supported"]:
        verdict = "REFUTED"
    elif ha["max_abs_rho"] > max(res["H-Fz"]["max_abs_rho"], res["H-Fy"]["max_abs_rho"]) and low_most_neg:
        verdict = "CONFIRMED"
    else:
        verdict = "MIXED"
    return res, {"verdict": verdict, "low_class_most_negative": low_most_neg, "resid_by_class": by_cls}


# ------------------------------------------------------------------ renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def render_factors(S, rows, path):
    plt = _plt()
    facs = list(FACTOR_WORDS)
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    for ax, f in zip(axs.flat, facs):
        for a, col in (("front", "#2a78d6"), ("rear", "#eb6834")):
            rr = [r for r in rows if r["axle"] == a]
            ax.scatter([r[f] for r in rr], [r["resid"] for r in rr], s=14, color=col, alpha=0.7, label=f"{a} axle")
        ax.axhline(0, color="#6b7280", lw=0.8)
        ax.set_xlabel(FACTOR_WORDS[f])
        ax.set_ylabel("corner residual, measured minus curve [Fy/Fz]")
        ax.grid(alpha=0.25)
    axs.flat[0].legend(fontsize=8)
    fig.suptitle(f"{S['label']}: per-corner tyre-curve residual against candidate input factors\n"
                 f"literature-bounded curve, one point per corner and lap; positive = more force than the curve")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def render_cloud(S, path):
    plt = _plt()
    a = "rear"
    pop = tfa_fit_pop(S, a)
    al, y = np.degrees(S["alpha"][a][pop]), S["Fy"][a][pop] / S["Fz"][a][pop]
    sc = S["speed_class"][pop]
    fig, ax = plt.subplots(figsize=(11, 7))
    for cls, col in CLASS_COLOURS.items():
        m = sc == cls
        ax.scatter(al[m][::5], y[m][::5], s=2, color=col, alpha=0.35, label=f"{cls}-speed corners")
    other = ~np.isin(sc, list(CLASS_COLOURS))
    ax.scatter(al[other][::5], y[other][::5], s=2, color="#9ca3af", alpha=0.25, label="outside corner brackets")
    g = np.radians(np.linspace(-10, 10, 800))
    B, mu = S["curves"][a]
    ax.plot(np.degrees(g), curve(g, B, mu), color="#1f2937", lw=2.2, label="literature-bounded fitted curve")
    ax.set_xlim(-10, 10)
    ax.set_xlabel("rear axle slip angle from the simple geometric estimate [deg]")
    ax.set_ylabel("rear axle lateral force / measured axle load [-]")
    ax.set_title(f"{S['label']}, rear axle: load-normalised force by corner speed class\n"
                 f"every 5th sample of the fit population")
    ax.legend(fontsize=8, markerscale=6, loc="lower right")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    try:
        C, K, HF, ROWS = {}, {}, {}, {}
        for key, label, rel in SESSIONS:
            S = session_inputs(key, label, rel)
            K[key] = k_check(S)
            HF[key] = h_fz_session(S)
            ROWS[key] = instances(S)
            C[key] = correlations(ROWS[key])
            render_factors(S, ROWS[key], os.path.join(OUT_DIR, f"factors_{key}.png"))
            if key == SUBJECT:
                render_cloud(S, os.path.join(OUT_DIR, "usv2_rear_cloud_by_speed.png"))
            log("=" * 100)
            log(f"{label} ({key}): laps_used {S['laps_used']}; curves L " +
                ", ".join(f"{a} B {S['curves'][a][0]:.3f} mu {S['curves'][a][1]:.3f}" for a in AXLES))
            for a in AXLES:
                k, n = K[key]["session"][a]
                log(f"  K check {a}: {k:.2f} / rad (n {n}); per lap: " + ", ".join(
                    f"{lap} {v[0]:.1f}" for lap, v in K[key]["per_lap"][a].items()))
            for a in AXLES:
                log(f"  H-Fz {a}: measured / model Fz by class {({c: round(v, 3) for c, v in HF[key]['by_class'][a].items()})}")
            log(f"  H-Fz wheel sum on straights / (m g + c v^2): {HF[key]['wheel_sum_straights']}")
            for a in AXLES:
                ca = C[key][a]
                log(f"  {a}: instances {ca['n_instances']} {ca['n_by_class']}; mean residual by class "
                    f"{({c: round(v, 4) for c, v in ca['resid_by_class'].items()})}")
                for f, d in ca["factors"].items():
                    if d["rho"] is None:
                        log(f"    {f}: not computable (n {d['n']})")
                        continue
                    log(f"    {f}: rho {d['rho']:+.3f} (p {d['p']:.2g}, n {d['n']}) {'ASSOCIATED' if d['associated'] else ''}; "
                        f"median {d['median']:.3f} IQR [{d['iqr'][0]:.3f}, {d['iqr'][1]:.3f}]")
        hyp, pred = score(C)
        log("\n## HYPOTHESES (US-v2 rear)")
        for h, d in hyp.items():
            log(f"- {h}: {'SUPPORTED' if d['supported'] else 'not supported'} (max |rho| {d['max_abs_rho']:.3f}); {d['factors']}")
        log(f"\n## PREDICTION: {pred['verdict']} -- low class most negative on US-v2 rear: "
            f"{pred['low_class_most_negative']} {pred['resid_by_class']}")
        named = [h for h, d in hyp.items() if d["supported"]]
        log(f"## OUTCOME: {'named mechanism candidate(s): ' + ', '.join(named) if named else 'NOT SHOWN'}")
        with open(os.path.join(OUT_DIR, "pr_anomaly.json"), "w", encoding="utf-8") as fh:
            json.dump({"K": K, "H_Fz": HF, "corr": C, "hypotheses": hyp, "prediction": pred,
                       "instances": ROWS}, fh, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
        return 0
    finally:
        _LOG.close()


if __name__ == "__main__":
    sys.exit(main())
