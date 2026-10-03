# Q-SHARED-OFFSET -- one shared damper-gauge offset per corner across the 16
# NBR files (PLAN.md PHASE 1 PLAN Q-SHARED-OFFSET; read-only; EXECUTE received
# with the package text, 2026-10-03).
#
# PURPOSE: test, not adopt, the Q-WHEELLOAD observation (raw in-motion levels
# vary only 0.39-0.65 kN per corner across the NBR files, much less than Q2's
# per-file offsets) -- if one offset per corner describes all 16 files, the NBR
# gauges recover to measured (L4-rel) loads and Q2 reopens.
#
# RULES, frozen 2026-10-03 BEFORE any compute.
# DATA: the Q2 per-file chain at zero offset (damper_offset_q2/cache: fz0, MR,
#   a_y, v, base and straight masks, laps; wheel-load chain modules.wheel_loads,
#   fz(K) = fz(0) - K MR, K in gauge units as Q2), the Q2 per-file JSON
#   (stationary on-wheel offset C1 and straight-line motion ratio), and the
#   cumulative burn ecu_fuelcons_kg read fresh per file (target_c = W_c -
#   burn g / 4, W_c the config corner weight -- Level 1, event unstated).
# ESTIMATOR (derivation from in-motion levels with the existing validity masks):
#   per corner, pooled over the 16 files on Q2's straight mask S (base, off-kerb,
#   |a_x|, |a_y| < 0.1 g, v > 60 km/h): least squares of
#     fz0 - target = K MR + b v^2                  (CONSTANT variant)
#     fz0 - target = (K0 + K1 o) MR + b v^2        (LINEAR variant, o = outing
#                                                   rank 0..15 centred)
#   with ONE b per corner (one car, one aero map; stated) and every file weighted
#   equally (sample weight 1 / n_file). Both variants for all four corners; RR is
#   where the linear term is motivated (rho -0.68 with outing order); the simpler
#   (constant) is preferred wherever both pass.
# UNCERTAINTY of a recovered load per corner (wheel N):
#   u = sqrt( (MR_s x sqrt(SE^2 + s_between^2))^2 + 98^2 ), MR_s the median
#   straight-line motion ratio; SE from a file-cluster bootstrap (B = 200) of
#   the pooled K (or K0 at o = 0); s_between = SD across files of the per-file
#   offset K_f (K_f = least squares of the same model with b fixed at the
#   pooled value) around the shared value -- how wrong the shared offset is for
#   ONE file; 98 N = the fuel term of Q2 (+/-10 kg per corner).
# BAR: u <= 700 N per corner AND the recovered loads pass the existing shape-
#   validity checks on ALL 16 files: V1 p0.1 of the corner load >= -200 N; V2
#   corr(a_y, Fz) sign (left < 0, right > 0) and |corr| >= 0.85; V3 total
#   straight-line load rises with v^2 (b > 0); V4 STATIC PLAUSIBILITY (Q2's
#   stationary on-wheel windows): |C1 - K_shared| x MR_s <= 700 N on every file
#   that has a window (files without one: not assessable, stated).
# EXIT per corner: a corner is RECOVERED iff u <= 700 N and V1, V2, V4 pass on
#   all 16 files for it, and V3 passes on all 16 files with the shared offsets.
#   RECOVERED (all four) / PARTIAL (corners named) / NOT. If RECOVERED or
#   PARTIAL: Q2 reopens for those corners; the L4 upgrade path is stated; the
#   (e) reach gate is RE-SCORED as a count only (rear axle needs RL and RR;
#   recorded reach 2 of 20), no adoption.
# RENDERS: per-corner offsets with uncertainty (per-file K_f in outing order,
#   shared value and band, RR both variants); recovered load vs the production
#   static fallback (estimate_vertical_loads, static path) per corner for three
#   representative files (earliest, median-residual, latest outing) plus the
#   worst file (largest |K_f - K_shared| summed over corners).
# Outputs diagnostics/plots/shared_offset/.
#
# Usage: python -m diagnostics.inspect_shared_offset prep <key> | fit | renders

import gzip
import json
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/shared_offset"
Q2_DIR = "diagnostics/plots/damper_offset_q2"
CORNERS = ("fl", "fr", "rl", "rr")
G = 9.81
BAR_N, FUEL_N, V1_FLOOR_N, V2_CORR = 700.0, 98.0, -200.0, 0.85
B_BOOT = 200
SIGN = {"fl": -1, "fr": 1, "rl": -1, "rr": 1}
WORDS = {"fl": "front left", "fr": "front right", "rl": "rear left", "rr": "rear right"}
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2


def _log(m):
    print(m, flush=True)


def keys():
    return sorted([k for k in d2.usable_keys() if k.startswith("nbr_")], key=lambda k: int(d2.sessions()[k]["outing"]))


def cache(key):
    with gzip.open(os.path.join(Q2_DIR, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def q2(key):
    return json.load(open(os.path.join(Q2_DIR, f"partB_{key}.json")))


def prep(key):
    from diagnostics.inspect_imu_offsets import read_raw_blocks
    C = cache(key)
    fb = read_raw_blocks(d2.sessions()[key]["path"], ["ecu_fuelcons_kg"])["ecu_fuelcons_kg"]
    burn = np.interp(C["t"], fb[0], fb[1]) - np.interp(C["t"][0], fb[0], fb[1])
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    np.save(os.path.join(OUT_DIR, "cache", f"burn_{key}.npy"), burn.astype(np.float32))
    _log(f"[{key}] burn total {burn[-1]:.2f} kg")


def load(key):
    C = cache(key)
    C["burn"] = np.load(os.path.join(OUT_DIR, "cache", f"burn_{key}.npy")).astype(float)
    C["target"] = {c: C["W"][c] - C["burn"] * G / 4.0 for c in CORNERS}
    return C


def design(C, c, o, linear):
    m = C["S"] & np.isfinite(C["fz0"][c]) & np.isfinite(C["mr"][c])
    mr, v = C["mr"][c][m].astype(float), C["v"][m].astype(float) / 3.6
    y = C["fz0"][c][m].astype(float) - C["target"][c][m]
    X = np.vstack([mr, mr * o, v ** 2]).T if linear else np.vstack([mr, v ** 2]).T
    return X, y, np.full(y.size, 1.0 / max(1, y.size))


def pooled(D, linear, idx=None):
    idx = range(len(D)) if idx is None else idx
    X = np.vstack([D[i][0] for i in idx])
    y = np.concatenate([D[i][1] for i in idx])
    w = np.sqrt(np.concatenate([D[i][2] for i in idx]))
    coef, *_ = np.linalg.lstsq(X * w[:, None], y * w, rcond=None)
    return coef


def fit():
    from scipy.stats import spearmanr  # noqa: F401
    K = keys()
    data = {k: load(k) for k in K}
    o = {k: i - (len(K) - 1) / 2.0 for i, k in enumerate(K)}
    Q = {k: q2(k) for k in K}
    out = {"files_in_outing_order": K, "outings": [int(d2.sessions()[k]["outing"]) for k in K], "corners": {}}
    rng = np.random.default_rng(20261003)
    for c in CORNERS:
        mr_s = float(np.median(np.concatenate([data[k]["mr"][c][data[k]["S"]] for k in K])))
        cres = {"mr_straight": mr_s}
        for var, linear in (("constant", False), ("linear", True)):
            D = [design(data[k], c, o[k], linear) for k in K]
            coef = pooled(D, linear)
            boots = [pooled(D, linear, rng.integers(0, len(D), len(D)))[0] for _ in range(B_BOOT)]
            se = float(np.std(boots, ddof=1))
            b = coef[-1]
            Kf = []
            for i, k in enumerate(K):
                X, y, _w = D[i]
                mr = X[:, 0]
                Kf.append(float(np.sum(mr * (y - b * X[:, -1])) / np.sum(mr * mr)))
            pred = [coef[0] + (coef[1] * o[k] if linear else 0.0) for k in K]
            resid = np.array(Kf) - np.array(pred)
            s_between = float(np.std(resid, ddof=1))
            u = float(np.sqrt((mr_s * np.sqrt(se ** 2 + s_between ** 2)) ** 2 + FUEL_N ** 2))
            # validity with the shared offset, per file
            v1, v2, v4 = [], [], []
            for k, kp in zip(K, pred):
                C = data[k]
                fz = C["fz0"][c].astype(float) - kp * C["mr"][c].astype(float)
                x = fz[C["base"]]
                v1.append(float(np.nanpercentile(x, 0.1)))
                mm = C["base"] & np.isfinite(fz)
                v2.append(float(np.corrcoef(C["ay_g"][mm], fz[mm])[0, 1]))
                k1 = Q[k]["C1"][c]["K"]
                v4.append(float(abs(k1 - kp) * Q[k]["choice"][c]["mr_straight"]) if np.isfinite(k1) else float("nan"))
            v1_ok = all(x >= V1_FLOOR_N for x in v1)
            v2_ok = all(np.sign(x) == SIGN[c] and abs(x) >= V2_CORR for x in v2)
            v4_ok = all(x <= BAR_N for x in v4 if np.isfinite(x))
            cres[var] = {"K0": float(coef[0]), "K1_per_outing_rank": float(coef[1]) if linear else 0.0,
                         "aero_b_N_per_mps2": float(b), "SE_boot": se, "s_between": s_between, "u_wheel_N": u,
                         "per_file_K": dict(zip(K, Kf)), "per_file_pred": dict(zip(K, pred)),
                         "V1_p01_N": dict(zip(K, v1)), "V2_corr": dict(zip(K, v2)), "V4_static_err_N": dict(zip(K, v4)),
                         "V4_assessable": int(sum(np.isfinite(v4))), "V1_ok": v1_ok, "V2_ok": v2_ok, "V4_ok": v4_ok,
                         "u_ok": u <= BAR_N}
            _log(f"{c} {var}: K0 {coef[0]:.0f} K1 {cres[var]['K1_per_outing_rank']:+.1f} b {b:.3f} SE {se:.0f} "
                 f"s_between {s_between:.0f} u {u:.0f} N; V1 min {min(v1):.0f} N ok {v1_ok}; V2 min|r| "
                 f"{min(abs(x) for x in v2):.3f} ok {v2_ok}; V4 max {np.nanmax(v4):.0f} N ({cres[var]['V4_assessable']} files) ok {v4_ok}")
        out["corners"][c] = cres
    # V3 with the shared offsets (constant variant everywhere; RR also linear)
    def v3(use_linear_rr):
        res = {}
        for k in K:
            C = data[k]
            tot = 0.0
            for c in CORNERS:
                var = "linear" if (c == "rr" and use_linear_rr) else "constant"
                kp = out["corners"][c][var]["per_file_pred"][k]
                tot = tot + (C["fz0"][c].astype(float) - kp * C["mr"][c].astype(float))
            m = C["S"] & np.isfinite(tot)
            A = np.vstack([np.ones(m.sum()), (C["v"][m].astype(float) / 3.6) ** 2]).T
            (a_, b_), *_ = np.linalg.lstsq(A, tot[m], rcond=None)
            res[k] = float(b_)
        return res
    for name, flag in (("constant", False), ("rr_linear", True)):
        r = v3(flag)
        out[f"V3_{name}"] = {"b_per_file": r, "ok": all(x > 0 for x in r.values())}
    # exit
    rec = {}
    for c in CORNERS:
        ok = {}
        for var in ("constant", "linear"):
            x = out["corners"][c][var]
            v3ok = out["V3_rr_linear" if (c == "rr" and var == "linear") else "V3_constant"]["ok"]
            ok[var] = bool(x["u_ok"] and x["V1_ok"] and x["V2_ok"] and x["V4_ok"] and v3ok)
        chosen = "constant" if ok["constant"] else ("linear" if ok["linear"] else None)
        rec[c] = {"passes": ok, "recovered": chosen is not None, "variant": chosen}
    n = sum(r["recovered"] for r in rec.values())
    out["exit_per_corner"] = rec
    out["exit"] = "RECOVERED" if n == 4 else ("PARTIAL" if n > 0 else "NOT")
    rear = rec["rl"]["recovered"] and rec["rr"]["recovered"]
    out["e_reach_rescore"] = {"recorded_reach": "2 of 20", "rear_axle_recovered": rear,
                              "rescored_reach_count_only": "18 of 20" if rear else "2 of 20 (unchanged)"}
    _log(f"EXIT {out['exit']}: {json.dumps(rec)}; (e) reach {out['e_reach_rescore']}")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "fit.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=lp._jsonable)
    render_offsets(out)


def render_offsets(out):
    plt = lp._plt()
    K = out["files_in_outing_order"]
    x = np.arange(len(K))
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, constrained_layout=True)
    for axp, c in zip(axes.ravel(), CORNERS):
        cr = out["corners"][c]
        kf = np.array([cr["constant"]["per_file_K"][k] for k in K]) / 1000
        axp.plot(x, kf, "o", color=INK, label="per-file offset (shared aero term)")
        for var, col, ls in (("constant", ACC, "-"), ("linear", ACC2, "--")):
            p = np.array([cr[var]["per_file_pred"][k] for k in K]) / 1000
            band = cr[var]["u_wheel_N"] / cr["mr_straight"] / 1000
            lab = ("one shared offset" if var == "constant" else "shared offset with a season trend") + \
                  f" (load uncertainty {cr[var]['u_wheel_N']:.0f} N)"
            axp.plot(x, p, ls, color=col, lw=1.5, label=lab)
            axp.fill_between(x, p - band, p + band, color=col, alpha=0.12, lw=0)
        axp.set_xticks(x)
        axp.set_xticklabels(out["outings"], fontsize=7)
        axp.set_ylabel("gauge offset (kN, gauge units)", color=INK)
        axp.set_title(WORDS[c] + (" (season trend motivated here)" if c == "rr" else ""), fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
    for axp in axes[1]:
        axp.set_xlabel("outing number (chronological)", color=INK)
    fig.suptitle(lp._wrap(f"Nurburgring, 16 outings -- one shared damper-gauge offset per corner vs the per-file offsets; "
                          f"band = the 0.7 kN load-uncertainty bar converted to gauge units; exit {out['exit']}", fig),
                 fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "shared_offsets.png"), dpi=100)
    plt.close(fig)


def renders():
    from modules.stability_analysis import estimate_lateral_forces, estimate_vertical_loads
    out = json.load(open(os.path.join(OUT_DIR, "fit.json")))
    K = out["files_in_outing_order"]
    dev = {k: sum(abs(out["corners"][c]["constant"]["per_file_K"][k] - out["corners"][c]["constant"]["per_file_pred"][k])
                  for c in CORNERS) for k in K}
    med = sorted(K, key=lambda k: dev[k])[len(K) // 2]
    picks = {"earliest outing": K[0], "median-residual file": med, "latest outing": K[-1],
             "worst file (largest deviation from the shared offset)": max(K, key=lambda k: dev[k])}
    plt = lp._plt()
    for role, key in picks.items():
        params, data, state = d2._state(key)
        static = estimate_vertical_loads(state, estimate_lateral_forces(state, params), params)
        C = load(key)
        fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True, constrained_layout=True)
        for axp, c in zip(axes, CORNERS):
            var = out["exit_per_corner"][c]["variant"] or "constant"
            kp = out["corners"][c][var]["per_file_pred"][key]
            rec = C["fz0"][c].astype(float) - kp * C["mr"][c].astype(float)
            t, s = C["t"], C["s_m"]
            st_ = static[f"fz_{c}_N"]
            for l in C["laps"]:
                m = (t >= l["start_time"]) & (t <= l["end_time"])
                ms = (state["time"] >= l["start_time"]) & (state["time"] <= l["end_time"])
                axp.plot(s[m], rec[m] / 1000, color=INK, lw=0.5, alpha=0.6)
                axp.plot(state["s_m"][ms], st_[ms] / 1000, color=ACC2, lw=0.5, alpha=0.6)
            axp.plot([], [], color=INK, label="recovered with the shared offset (measured shape, L4-rel)")
            axp.plot([], [], color=ACC2, label="production static fallback model (Level 1, no aero)")
            axp.axhline(C["W"][c] / 1000, color=REF, ls="--", lw=0.8)
            axp.set_ylabel(f"{WORDS[c]} wheel load (kN)", color=INK)
            axp.legend(frameon=False, fontsize=7, loc="upper right")
            lp._style(axp)
        axes[-1].set_xlabel("lap distance (m)", color=INK)
        fig.suptitle(lp._wrap(f"Nurburgring, driver {d2.sessions()[key]['driver']}, outing "
                              f"{d2.sessions()[key]['outing']} ({role}) -- wheel "
                              f"load per corner recovered with one shared gauge offset vs the static fallback the chain "
                              f"uses today; dashed = config static corner weight", fig), fontsize=10, color=INK)
        slug = role.split(" ")[0]
        fig.savefig(os.path.join(OUT_DIR, f"recovered_{slug}_{key}.png"), dpi=100)
        plt.close(fig)
        _log(f"render {role}: {key}")


if __name__ == "__main__":
    m = sys.argv[1]
    if m == "prep":
        prep(sys.argv[2])
    elif m == "fit":
        fit()
    elif m == "renders":
        renders()
