# Steering-fix impact assessment (STEP 1-FIX (d), author EXECUTE 2026-10-04).
# Harness only, report only, no verdict re-run.
#
# QUESTION: the earlier frozen-frame verdicts (Q-FIT-DECISIVE, LT, MA, K2) ran
#   at the constant 15.7, the app at the faulty Level-4 lookup. Does the
#   corrected conversion move beta, alpha, CS and the fits enough that those
#   verdicts must be re-run?
# PATHS per Tier-1 session (US-v2, v3, JST 29), same data, same masks:
#   (i)   faulty L4  -- sw divided by the table's LOCAL ratio column (the
#         production conversion before 2026-10-04). Reproduced through the
#         unchanged production code by handing prepare_vehicle_state a dense
#         sw -> sw / i_local(sw) table (0.05 deg grid, interpolation error
#         <= 1.3e-4 deg, checked), so the fit, EKF and CS chain all see it.
#   (ii)  constant 15.7 -- the archived cap-1 fit caches
#         (plots/tyre_prior_d2/cache_cap1/), i.e. exactly what the frozen
#         frames were built on.
#   (iii) corrected L4 -- production after the fix (wheel angle interpolated
#         from the manufacturer map, mean of both front wheels); d2.fit
#         rebuilds the Tier-1 cache with it (author ruling 3).
# METRICS (frozen-frame definitions): in-corner = corner brackets on analysis
#   laps (inspect_fit_decisive.Session.in_corner); CS shares over finite CS
#   in-corner; fit = production manifest Pacejka per axle, origin slope B C D;
#   beta = production chain beta (EKF, gate-checked); alpha_seed = kinematic-
#   seed alpha (the fit's input); alpha_ekf = alpha from the chain beta (CS's
#   input).
# ROBUSTNESS RULE (frozen 2026-10-04 by the author before any number):
#   earlier verdicts are robust iff, (ii) -> (iii), per session and axle:
#   |d CS<0 share| <= 1.0 pt, |d CS<-1 share| <= 20 % rel, |d origin slope|
#   <= 5 %; and per session median in-corner |d beta| <= 0.2 deg. The verdict
#   is the author's ruling; this script only reports the four numbers.
#
# Modes: run <key> (chains (i) and (iii)), report, render.
# Output: diagnostics/plots/steering_fix_impact/

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp

KEYS = ("pr_us_v2", "v3", "nbr_jst_solution")
TITLES = {"pr_us_v2": "Paul Ricard, MLA outing 32", "v3": "Paul Ricard, MLA outing 25",
          "nbr_jst_solution": "Nurburgring, JST outing 29"}
PATHS = ("faulty", "const", "fixed")
PATH_NAME = {"faulty": "faulty lookup (divided by local ratio)", "const": "constant ratio 15.7",
             "fixed": "corrected lookup (wheel-angle map)"}
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "steering_fix_impact")
CACHE_CAP1 = os.path.join(d2.OUT_DIR, "cache_cap1")

# the frozen rule, verbatim
RULE_CS_NEG_PT = 1.0
RULE_CS_DEEP_REL = 0.20
RULE_SLOPE_REL = 0.05
RULE_BETA_DEG = 0.2


def faulty_table():
    """Dense sw -> wheel table reproducing delta = sw / i_local(sw)."""
    from modules.stability_analysis import load_car_data
    t = load_car_data()["steering_ratio_table"]
    cols = t["columns"]
    sw_col = np.array([r[cols.index("steering_wheel_angle_deg")] for r in t["rows"]], float)
    i_col = np.array([r[cols.index("steering_ratio")] for r in t["rows"]], float)
    sw = np.arange(-400.0, 400.0 + 1e-9, 0.05)
    return {"sw_deg": sw, "wheel_deg": sw / np.interp(sw, sw_col, i_col)}


def faulty_params():
    p = d2._params()
    p["vehicle"]["steering_wheel_table"] = faulty_table()
    return p


def chain(key, params):
    """d2.fit's body with explicit params, returned instead of cached."""
    from modules.stability_analysis import (prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
                                            estimate_lateral_forces)
    from modules.tyre_fit_auto import _base_mask
    import modules.tyre_fit_auto as tfa
    data = d2.load_data(key)
    state = prepare_vehicle_state(data["channels"], params)
    beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka",
                                                       csv_path=d2.sessions()[key]["path"])
    slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    forces = estimate_lateral_forces(state, params)
    out = {"key": key, "time": state["time"], "base_mask": _base_mask(state, data["laps"]),
           "alpha_f": slip["alpha_f_filt"], "alpha_r": slip["alpha_r_filt"],
           "fy_f": forces["Fy_f_filt"], "fy_r": forces["Fy_r_filt"],
           "prod": d2._run_record(beta, man, gate, fb, fr)}
    out["prod"]["down"] = d2._downstream(params, data, state, beta)
    return out


def run(key):
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    # (iii): the Tier-1 cache rebuild under the corrected harness params
    if not os.path.exists(os.path.join(d2.CACHE, f"fit_{key}.pkl.gz")):
        d2.fit(key)
    # (i)
    path = os.path.join(OUT_DIR, "cache", f"fit_faulty_{key}.pkl.gz")
    if not os.path.exists(path):
        d2._save(path, chain(key, faulty_params()))
    print(f"[{key}] chains done", flush=True)


def load(key, which):
    if which == "fixed":
        # corrected steering, before the speed window; archived 2026-10-04
        return d2._load(os.path.join(d2.OUT_DIR, "cache_steerfix_prewindow", f"fit_{key}.pkl.gz"))
    if which == "const":
        return d2._load(os.path.join(CACHE_CAP1, f"fit_{key}.pkl.gz"))
    return d2._load(os.path.join(OUT_DIR, "cache", f"fit_faulty_{key}.pkl.gz"))


def path_params(which):
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    if which == "faulty":
        return faulty_params()
    if which == "const":
        return apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), None, cap=1))
    return d2._params()


def pct(x, q):
    return float(np.percentile(x, q)) if x.size else float("nan")


def session_metrics(key):
    from modules.stability_analysis import prepare_vehicle_state, estimate_slip_angles
    ctx = lp.Ctx(key)   # masks only: base / brackets / laps do not depend on steering
    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    inc = ctx.in_br & on_lap
    fitpop = ctx.base
    data = ctx.data
    res = {"n_in_corner": int(inc.sum()), "paths": {}}
    deltas, betas, aekf = {}, {}, {}
    recs = {w: load(key, w) for w in PATHS}
    for w in PATHS:
        p = path_params(w)
        st = prepare_vehicle_state(data["channels"], p)
        rec = recs[w]
        assert rec["time"].size == st["time"].size
        beta = np.asarray(rec["prod"]["beta"], float)
        slip = estimate_slip_angles(st, beta, p)
        deltas[w] = np.degrees(st["delta_f_rad"])
        betas[w] = np.degrees(beta)
        aekf[w] = {"f": np.degrees(slip["alpha_f_filt"]), "r": np.degrees(slip["alpha_r_filt"])}
        man = rec["prod"]["manifest"] or {}
        gate = rec["prod"]["gate"] or {}
        axes = {}
        for ax, a in (("f", "front"), ("r", "rear")):
            fit = man.get("axles", {}).get(a, {})
            cs = np.asarray(rec["prod"]["down"][f"cs_{ax}"], float)
            fin = np.isfinite(cs) & inc
            axes[ax] = {
                "fit": {k: float(fit[k]) for k in "BCDE"} if fit else None,
                "origin_slope": float(fit["B"] * fit["C"] * fit["D"]) if fit else float("nan"),
                "cs_neg": float(np.mean(cs[fin] < 0)), "cs_deep": float(np.mean(cs[fin] < -1)),
                "n_fin": int(fin.sum()),
                "abs_alpha_ekf_p50": pct(np.abs(aekf[w][ax][inc]), 50),
                "abs_alpha_ekf_p95": pct(np.abs(aekf[w][ax][inc]), 95),
                "abs_alpha_seed_p95": pct(np.abs(np.degrees(np.asarray(rec[f"alpha_{ax}"], float))[inc]), 95),
            }
        res["paths"][w] = {"gate": gate.get("verdict"), "health": gate.get("health_score"),
                           "fallback": rec["prod"]["fallback"], "status": man.get("status"),
                           "abs_beta_p50": pct(np.abs(betas[w][inc]), 50),
                           "abs_beta_p95": pct(np.abs(betas[w][inc]), 95), "axes": axes}
    # differences against (iii), in-corner and fit population
    res["diff_vs_fixed"] = {}
    for w in ("faulty", "const"):
        dd = np.abs(deltas[w] - deltas["fixed"])
        db = np.abs(betas[w] - betas["fixed"])
        daf = np.abs(aekf[w]["f"] - aekf["fixed"]["f"])
        dar = np.abs(aekf[w]["r"] - aekf["fixed"]["r"])
        res["diff_vs_fixed"][w] = {
            "delta_fitpop": [pct(dd[fitpop], q) for q in (50, 95, 100)],
            "delta_incorner": [pct(dd[inc], q) for q in (50, 95, 100)],
            "beta_incorner": [pct(db[inc], q) for q in (50, 95, 100)],
            "alpha_f_incorner": [pct(daf[inc], q) for q in (50, 95, 100)],
            "alpha_r_incorner": [pct(dar[inc], q) for q in (50, 95, 100)],
        }
    # the frozen rule, (ii) -> (iii)
    c, f = res["paths"]["const"], res["paths"]["fixed"]
    rule = {"beta_median_deg": res["diff_vs_fixed"]["const"]["beta_incorner"][0]}
    rule["beta_ok"] = rule["beta_median_deg"] <= RULE_BETA_DEG
    for ax in ("f", "r"):
        ca, fa = c["axes"][ax], f["axes"][ax]
        neg_pt = (fa["cs_neg"] - ca["cs_neg"]) * 100
        deep_rel = fa["cs_deep"] / ca["cs_deep"] - 1 if ca["cs_deep"] > 0 else float("nan")
        slope_rel = fa["origin_slope"] / ca["origin_slope"] - 1
        rule[ax] = {"cs_neg_pt": neg_pt, "cs_neg_ok": abs(neg_pt) <= RULE_CS_NEG_PT,
                    "cs_deep_rel": deep_rel, "cs_deep_ok": bool(abs(deep_rel) <= RULE_CS_DEEP_REL),
                    "slope_rel": slope_rel, "slope_ok": abs(slope_rel) <= RULE_SLOPE_REL}
    rule["all_ok"] = bool(rule["beta_ok"] and all(rule[a][k] for a in ("f", "r")
                                                 for k in ("cs_neg_ok", "cs_deep_ok", "slope_ok")))
    res["rule"] = rule
    return res


def report():
    out = {k: session_metrics(k) for k in KEYS}
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    L = []
    for k in KEYS:
        r = out[k]
        L.append(f"== {TITLES[k]} ({k}), in-corner samples {r['n_in_corner']}")
        for w in PATHS:
            p = r["paths"][w]
            L.append(f"  {PATH_NAME[w]}: status {p['status']}, gate {p['gate']} {p['health']:.4f}, fallback "
                     f"{p['fallback']}; |beta| p50/p95 {p['abs_beta_p50']:.3f}/{p['abs_beta_p95']:.3f} deg")
            for ax, name in (("f", "front"), ("r", "rear")):
                a = p["axes"][ax]
                fit = a["fit"]
                L.append(f"    {name}: B {fit['B']:.3f} C {fit['C']:.3f} D {fit['D']:.0f} E {fit['E']:.2f}, origin "
                         f"slope {a['origin_slope'] / 1000:.1f} kN/rad; CS<0 {a['cs_neg'] * 100:.2f} %, CS<-1 "
                         f"{a['cs_deep'] * 100:.3f} % (n {a['n_fin']}); |alpha| from chain beta p50/p95 "
                         f"{a['abs_alpha_ekf_p50']:.3f}/{a['abs_alpha_ekf_p95']:.3f} deg; seed |alpha| p95 "
                         f"{a['abs_alpha_seed_p95']:.3f} deg")
        for w in ("faulty", "const"):
            d = r["diff_vs_fixed"][w]
            L.append(f"  |difference| {PATH_NAME[w]} vs corrected, p50/p95/max deg: "
                     f"delta fit-pop {'/'.join(f'{x:.3f}' for x in d['delta_fitpop'])}; "
                     f"delta in-corner {'/'.join(f'{x:.3f}' for x in d['delta_incorner'])}; "
                     f"beta in-corner {'/'.join(f'{x:.3f}' for x in d['beta_incorner'])}; "
                     f"alpha_f {'/'.join(f'{x:.3f}' for x in d['alpha_f_incorner'])}; "
                     f"alpha_r {'/'.join(f'{x:.3f}' for x in d['alpha_r_incorner'])}")
        ru = r["rule"]
        L.append(f"  RULE (ii)->(iii): median in-corner |d beta| {ru['beta_median_deg']:.3f} deg "
                 f"({'ok' if ru['beta_ok'] else 'EXCEEDS 0.2'})")
        for ax, name in (("f", "front"), ("r", "rear")):
            a = ru[ax]
            L.append(f"    {name}: CS<0 {a['cs_neg_pt']:+.2f} pt ({'ok' if a['cs_neg_ok'] else 'EXCEEDS'}), "
                     f"CS<-1 {a['cs_deep_rel'] * 100:+.1f} % rel ({'ok' if a['cs_deep_ok'] else 'EXCEEDS'}), "
                     f"origin slope {a['slope_rel'] * 100:+.2f} % ({'ok' if a['slope_ok'] else 'EXCEEDS'})")
        L.append(f"  RULE all criteria met: {ru['all_ok']}")
    txt = "\n".join(L)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(txt + "\n")
    print(txt)


# default validated palette, slots 1-3; line style is the second cue
COLORS = {"faulty": "#eb6834", "const": "#2a78d6", "fixed": "#1baf7a"}
STYLES = {"faulty": ":", "const": "--", "fixed": "-"}


def render():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.tyre_model_pacejka import pacejka_lateral_force
    for k in KEYS:
        ctx = lp.Ctx(k)
        on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
        inc = ctx.in_br & on_lap
        recs = {w: load(k, w) for w in PATHS}
        # tyre curves
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
        for i, (ax, name) in enumerate((("f", "front"), ("r", "rear"))):
            a_seed = np.degrees(np.asarray(recs["fixed"][f"alpha_{ax}"], float))
            amax = np.nanpercentile(np.abs(a_seed[ctx.base]), 99.5)
            grid = np.linspace(-amax, amax, 300)
            for w in PATHS:
                fit = recs[w]["prod"]["manifest"]["axles"]["front" if ax == "f" else "rear"]
                axs[i].plot(grid, pacejka_lateral_force(np.radians(grid), fit["B"], fit["C"], fit["D"], fit["E"]) / 1000,
                            color=COLORS[w], ls=STYLES[w], lw=2, label=PATH_NAME[w])
            axs[i].set_xlabel("slip angle [deg]")
            axs[i].set_ylabel("axle lateral force [kN]")
            axs[i].set_title(f"{name} axle")
            axs[i].grid(alpha=0.25)
            axs[i].legend(fontsize=8, loc="lower right")
        fig.suptitle(f"{TITLES[k]}: fitted tyre curves under three steering conversions")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"fit_curves_{k}.png"), dpi=130)
        plt.close(fig)
        # CS distributions
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
        bins = np.linspace(-3, 1.05, 82)
        for i, (ax, name) in enumerate((("f", "front"), ("r", "rear"))):
            for w in PATHS:
                cs = np.asarray(recs[w]["prod"]["down"][f"cs_{ax}"], float)
                v = cs[np.isfinite(cs) & inc]
                h, e = np.histogram(np.clip(v, bins[0], bins[-1]), bins=bins, density=True)
                axs[i].step(0.5 * (e[1:] + e[:-1]), h, where="mid", color=COLORS[w], ls=STYLES[w], lw=2,
                            label=f"{PATH_NAME[w]} (CS<0 {np.mean(v < 0) * 100:.1f} %)")
            axs[i].axvline(0, color="#888888", lw=1)
            axs[i].set_xlabel("in-corner CS ratio (clipped to [-3, 1.05])")
            axs[i].set_ylabel("density")
            axs[i].set_title(f"{name} axle")
            axs[i].grid(alpha=0.25)
            axs[i].legend(fontsize=8, loc="upper left")
        fig.suptitle(f"{TITLES[k]}: in-corner CS distribution under three steering conversions")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"cs_distribution_{k}.png"), dpi=130)
        plt.close(fig)
    print("rendered", flush=True)


# ---------------------------------------------------------------- spikes
# Author ruling 3 (2026-10-04): locate the isolated 8-10 deg beta differences
# between (i) and (iii) before the robustness ruling. Report only.
# Suspected mechanism, from the EKF code: a divergence flag (NIS window or
# the 15 deg hard bound) resets the state to the kinematic beta; if a small
# input change moves where a flag fires, the two runs differ by the full
# EKF-minus-kinematic gap until they reconverge. Tested by rebuilding each
# path's final EKF pass from its manifest (fit + chosen R) -- the rebuild
# must reproduce the cached beta exactly before its diverged mask is used.
SPIKE_DEG = 2.0       # event threshold on |d beta|; reporting choice
MERGE_GAP_S = 0.2     # runs closer than this are one event


def final_ekf(params, state, man):
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
    cfg = params["tyre_fit_auto"]
    f, r = man["axles"]["front"], man["axles"]["rear"]
    ch = man["r_sweep_chosen"]
    fc = {"b_front": f["B"], "c_front": f["C"], "d_front": f["D"], "e_front": f["E"],
          "b_rear": r["B"], "c_rear": r["C"], "d_rear": r["D"], "e_rear": r["E"],
          "Q_beta_var": cfg["q_beta_var"], "Q_yaw_rate_var": cfg["q_yaw_rate_var"],
          "P0_beta_var": cfg["p0_beta_var"], "P0_yaw_rate_var": cfg["p0_yaw_rate_var"],
          "R_ay_var": ch["R_ay_var"], "R_yaw_rate_var": ch["R_yaw_rate_var"],
          "beta_hard_bound_deg": cfg["beta_hard_bound_deg"], "nis_window_samples": cfg["nis_window_samples"],
          "nis_chi2_bound": cfg["nis_chi2_bound"], "nis_flag_fraction": cfg["nis_flag_fraction"]}
    p = dict(params)
    p["tyre_model_ekf_pacejka"] = {"_auto_final": fc}
    return estimate_sideslip_ekf_pacejka(state, p, pass_id="_auto_final"), ch


def runs(mask):
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(idx[::2], idx[1::2]))


def spikes():
    from modules.stability_analysis import prepare_vehicle_state, estimate_sideslip
    L = []
    for k in KEYS:
        ctx = lp.Ctx(k)
        on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
        inc = ctx.in_br & on_lap
        sr = ctx.state["sample_rate_hz"]
        st, ekf, rec, chosen = {}, {}, {}, {}
        for w in ("faulty", "fixed"):
            p = path_params(w)
            st[w] = prepare_vehicle_state(ctx.data["channels"], p)
            rec[w] = load(k, w)
            ekf[w], chosen[w] = final_ekf(p, st[w], rec[w]["prod"]["manifest"])
            rep = np.max(np.abs(ekf[w]["beta_with_fallback"] - np.asarray(rec[w]["prod"]["beta"], float)))
            L.append(f"[{k}] {w}: rebuild vs cached beta max |diff| {np.degrees(rep):.2e} deg; chosen R_ay "
                     f"{chosen[w]['R_ay_var']:.4g} R_yaw {chosen[w]['R_yaw_rate_var']:.4g}; diverged samples "
                     f"{int(ekf[w]['diverged_mask'].sum())} (hard-bound {int((np.abs(ekf[w]['beta']) > np.radians(15)).sum())})")
        bf, bx = (np.degrees(np.asarray(rec[w]["prod"]["beta"], float)) for w in ("fixed", "faulty"))
        db = np.abs(bx - bf)
        bkin = np.degrees(estimate_sideslip(st["fixed"], path_params("fixed")))
        dd = np.abs(np.degrees(st["faulty"]["delta_f_rad"] - st["fixed"]["delta_f_rad"]))
        cs = {w: {ax: np.asarray(rec[w]["prod"]["down"][f"cs_{ax}"], float) for ax in "fr"} for w in ("faulty", "fixed")}
        L.append(f"[{k}] samples |d beta| > 1 / 2 / 5 deg: {int((db > 1).sum())} / {int((db > 2).sum())} / "
                 f"{int((db > 5).sum())} of {db.size}; in-corner: {int((db[inc] > 1).sum())} / "
                 f"{int((db[inc] > 2).sum())} / {int((db[inc] > 5).sum())} of {int(inc.sum())}")
        # events: merge runs closer than MERGE_GAP_S
        ev = []
        for a, b in runs(db > SPIKE_DEG):
            if ev and a - ev[-1][1] <= MERGE_GAP_S * sr:
                ev[-1] = (ev[-1][0], b)
            else:
                ev.append((a, b))
        div_f, div_x = ekf["fixed"]["diverged_mask"], ekf["faulty"]["diverged_mask"]
        L.append(f"[{k}] events (|d beta| > {SPIKE_DEG} deg, gaps <= {MERGE_GAP_S} s merged): {len(ev)}")
        for a, b in ev:
            pk = a + int(np.argmax(db[a:b]))
            # first divergence flag that differs between the runs, at or before the event start
            lo = max(0, a - int(5 * sr))
            diff_flag = np.flatnonzero(div_f[lo:b] != div_x[lo:b])
            first = (lo + diff_flag[0]) if diff_flag.size else None
            n_cs = {w: int(sum((np.isfinite(cs[w][ax][a:b]) & inc[a:b]).sum() for ax in "fr")) for w in cs}
            L.append(
                f"   t {ctx.state['time'][a]:8.2f}-{ctx.state['time'][b - 1]:8.2f} s ({b - a} samples): max |d beta| "
                f"{db[pk]:.2f} deg at v {ctx.state['v_mps'][pk]:.1f} m/s, |sw| {np.degrees(abs(st['fixed']['steer_sw_rad'][pk])):.0f} deg, "
                f"max |d delta| {dd[max(lo, 0):b].max():.3f} deg (5 s before .. end); beta fixed {bf[pk]:+.2f} faulty "
                f"{bx[pk]:+.2f} kinematic {bkin[pk]:+.2f}; diverged in event: fixed {int(div_f[a:b].sum())} faulty "
                f"{int(div_x[a:b].sum())}; first differing flag "
                f"{'none' if first is None else f'{(first - a) / sr:+.2f} s rel. start'}; in corner bracket "
                f"{inc[a:b].mean() * 100:.0f} %, analysis lap {on_lap[a:b].mean() * 100:.0f} %, base "
                f"{ctx.base[a:b].mean() * 100:.0f} %, kerb {st['fixed']['kerb_mask'][a:b].mean() * 100:.0f} %; "
                f"finite in-corner CS samples (f+r) fixed {n_cs['fixed']} faulty {n_cs['faulty']}")
    txt = "\n".join(L)
    with open(os.path.join(OUT_DIR, "spikes.txt"), "w") as fh:
        fh.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    mode = sys.argv[1]
    if mode == "run":
        run(sys.argv[2])
    elif mode == "report":
        report()
    elif mode == "render":
        render()
    elif mode == "spikes":
        spikes()
