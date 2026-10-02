# Q2: NBR damper-offset feasibility + limitation-11 load-source check
# (PLAN.md PHASE 1 PLAN Q2; proposal thesis_notes.md "Q2 proposal ...",
# approved 2026-10-02 as written). Read-only diagnostic.
#
# PART A (limitation 11): per-sample load-source tiers on the (d) NBR
#   sessions, and the rear mean Fz on the fit population (production base
#   mask) on the measured and the static path, NBR vs PR/Dubai.
#   SOURCE-EXPLAINED iff NBR rear tiers are static fallback on >= 95 % of the
#   fit population AND the PR static-path rear mean lies within +/-5 % of the
#   NBR measured-path rear mean. [impl] The proposal said "six" NBR sessions;
#   (d) had four (jst_problem, jst_solution, qp_cdr, qp_mla) -- those four.
#
# PART B (16 NBR files): raw log_dms_dam_* stats; within-file stationarity =
#   drift of the straight-line raw mean (|a_x|, |a_y| < 0.1 g, v > 60 kph,
#   off-kerb, base mask) between the first and last third of the run at
#   matched 20 kph speed bins (>= 20 samples per third and bin), weighted by
#   the smaller count. Bar: |drift| <= 100 N per corner (5 % of the healthy
#   corners' dynamic std 1.9-2.1 kN). Between-file: a shared logger-family
#   offset iff the per-file offsets (C2) agree within 100 N per corner.
#   [impl] fuel burn lowers the static load during the run, by about the size
#   of the bar on long runs; the expected burn effect (burn * g / 4 per corner,
#   equal split -- tank position unknown -- divided by the motion ratio for
#   pushrod units) is removed before scoring. Raw drift is reported alongside.
#   Stationary windows: v < 3 kph for >= 1 s.
#
# PART C (anchor): the chain is linear in the gauge offset K:
#   fz_c(K) = fz_c(0) - K * MR_c(travel)   (modules/wheel_loads.py)
#   so one chain run with K = 0 gives every candidate. Reference: config
#   corner weights W_c (Level 1; EVENT UNSTATED -- between-event base-weight
#   difference inherited on top of fuel uncertainty), target load
#   W_c * g - burn * g / 4.
#   C1 stationary: K1 = mean over on-wheel stationary samples of
#      (fz(0) - target) / MR. [impl] "on wheels" = all four travels inside
#      the p1..p99 band of the straight-line travel (no air jacks).
#   C2 straights: OLS (fz(0) - target) = K * MR + b * v^2, no intercept
#      (b absorbs aero). [impl] SE from the spread of per-lap K (samples are
#      autocorrelated).
#   C3 shared: median C2 offset across files, only if Part B finds a family.
#   Choice by the selection principle: C1 where available and within 300 N
#   (wheel units) of C2, else C2.
#   [impl] per-corner uncertainty u = sqrt(max(|K1 - K2|, 2 SE2)^2 * MR^2 +
#   98^2) in wheel-load N (98 N = +/-10 kg, the +/-40 kg fuel spread split
#   four ways).
# VALIDATION with the chosen K (base mask, off-kerb): V1 p0.1 of every corner
#   load >= -200 N; V2 corr(a_y, Fz_c) left < 0, right > 0, |corr| >= 0.85;
#   V3 total straight-line load rises with v^2 (b > 0).
# EXIT: FEASIBLE iff stationarity holds on >= 12 of 16 files, u <= 300 N on
#   all four corners on >= 12 of 16 files [impl: the count was not stated for
#   this leg; the other legs' count is used], and validation passes on >= 12
#   of 16. NOT FEASIBLE iff drift beyond the bar on > 4 files or no usable
#   reference on most files.
# LABEL: a recovered load is "L4-rel" (Level-4 shape on a Level-1 offset).
#
# Modes: partA | partB <key> | partB_all | report

import gzip
import json
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks

OUT_DIR = "diagnostics/plots/damper_offset_q2"
G = 9.81
CORNERS = ("fl", "fr", "rl", "rr")
PARTA_NBR = ("nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla")
PARTA_REF = ("v3", "pr_us_v2", "dubai")
DRIFT_BAR_N, FAMILY_BAR_N, U_BAR_N, FUEL_CORNER_N = 100.0, 100.0, 300.0, 98.0
COUNT_BAR = 12
STR_G, STR_V_KPH, BIN_KPH, MIN_BIN_N = 0.10, 60.0, 20.0, 20
STILL_KPH, STILL_MIN_S = 3.0, 1.0
V1_FLOOR_N, V2_CORR = -200.0, 0.85


def _nbr_keys():
    return [k for k in d2.usable_keys() if k.startswith("nbr")]


def part_a():
    from modules.stability_analysis import estimate_lateral_forces, estimate_vertical_loads, load_car_data
    from modules.tyre_fit_auto import _base_mask
    out = {}
    for k in PARTA_NBR + PARTA_REF:
        params, data, state = d2._state(k)
        base = _base_mask(state, data.get("laps", []))
        forces = estimate_lateral_forces(state, params)
        rec = {}
        for path in ("measured", "static"):
            p = dict(params)
            p["stability_estimation"] = dict(params["stability_estimation"], vertical_load_source=path)
            fz = estimate_vertical_loads(state, forces, p, channels=data["channels"], car_data=load_car_data())
            rec[path] = {"rear_mean_N": float(np.mean(fz["fz_r_N"][base])),
                         "front_mean_N": float(np.mean(fz["fz_f_N"][base])),
                         "path_label": fz["vertical_load_source_used"]}
            src = fz.get("vertical_load_source_per_sample")
            if path == "measured" and src:
                rec["tiers_on_base"] = {c: {t: float(np.mean(np.asarray(src[c])[base] == t))
                                            for t in sorted(set(np.asarray(src[c]).tolist()))} for c in CORNERS}
        rec["n_base"] = int(base.sum())
        out[k] = rec
        print(k, json.dumps(rec), flush=True)
    nbr_rear = np.mean([out[k]["measured"]["rear_mean_N"] for k in PARTA_NBR])
    nbr_static_share = min(out[k]["tiers_on_base"][c].get("static_fallback", 0.0)
                           for k in PARTA_NBR for c in ("rl", "rr"))
    pr_static = {k: out[k]["static"]["rear_mean_N"] for k in ("v3", "pr_us_v2")}
    within = {k: abs(v / nbr_rear - 1) <= 0.05 for k, v in pr_static.items()}
    verdict = {"nbr_rear_mean_measured_path_N": nbr_rear, "min_nbr_rear_static_share": nbr_static_share,
               "pr_static_rear_mean_N": pr_static, "pr_static_vs_nbr_ratio": {k: v / nbr_rear for k, v in pr_static.items()},
               "source_explained": bool(nbr_static_share >= 0.95 and all(within.values()))}
    out["verdict"] = verdict
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "partA.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("VERDICT", json.dumps(verdict, indent=1))


def _runs(mask, t, min_s):
    d = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    return [(a, b) for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]) if t[b - 1] - t[a] >= min_s]


def part_b(key):
    from modules.wheel_loads import estimate_wheel_loads_from_dampers, _normalize_travel_to_mm
    from modules.stability_analysis import load_car_data
    from modules.tyre_fit_auto import _base_mask

    params, data, state = d2._state(key)
    ch, laps = data["channels"], data.get("laps", [])
    t = state["time"]
    v = state["v_mps"] * 3.6
    ax_g, ay_g = state["ax_mps2"] / G, state["ay_mps2"] / G
    base = _base_mask(state, laps) & ~state["kerb_mask"]
    cw = params["vehicle"]["corner_weights"]
    W = {"fl": cw["FL_kg"] * G, "fr": cw["FR_kg"] * G, "rl": cw["RL_kg"] * G, "rr": cw["RR_kg"] * G}

    ch2 = dict(ch)
    raw = {}
    for c in CORNERS:
        n = f"log_dms_dam_{c}"
        ch2[n] = dict(ch[n], quality="valid")
        raw[c] = np.interp(t, np.asarray(ch[n]["time"], float), np.asarray(ch[n]["data"], float))
    wl = estimate_wheel_loads_from_dampers(state, ch2, params, load_car_data())
    fz0 = {c: wl[c]["fz_N"] for c in CORNERS}
    mr = {c: wl[c]["motion_ratio"] for c in CORNERS}
    trav = {c: _normalize_travel_to_mm(np.interp(t, np.asarray(ch[f"log_susp_travel_{c}"]["time"], float),
                                                 np.asarray(ch[f"log_susp_travel_{c}"]["data"], float)),
                                       ch[f"log_susp_travel_{c}"]["unit_raw"]) for c in CORNERS}

    fb = read_raw_blocks(d2.sessions()[key]["path"], ["ecu_fuelcons_kg"])["ecu_fuelcons_kg"]
    burn = np.interp(t, fb[0], fb[1]) - np.interp(t[0], fb[0], fb[1])
    target = {c: W[c] - burn * G / 4.0 for c in CORNERS}

    S = base & (np.abs(ax_g) < STR_G) & (np.abs(ay_g) < STR_G) & (v > STR_V_KPH)
    rec = {"key": key, "n_base": int(base.sum()), "n_straight": int(S.sum()), "burn_total_kg": float(burn[-1]),
           "raw": {c: {"mean": float(np.mean(raw[c])), "std": float(np.std(raw[c]))} for c in CORNERS}}

    # Part B drift
    tb = t[base]
    t0, t1 = (tb.min(), tb.max()) if tb.size else (t[0], t[-1])
    first = S & (t < t0 + (t1 - t0) / 3)
    last = S & (t > t1 - (t1 - t0) / 3)
    drift = {}
    for c in CORNERS:
        num_raw = num_cor = den = 0.0
        for lo in np.arange(STR_V_KPH, 300, BIN_KPH):
            b = (v >= lo) & (v < lo + BIN_KPH)
            m1, m2 = first & b, last & b
            n1, n2 = int(m1.sum()), int(m2.sum())
            if min(n1, n2) < MIN_BIN_N:
                continue
            d_raw = np.mean(raw[c][m2]) - np.mean(raw[c][m1])
            exp_push = -(np.mean(burn[m2]) - np.mean(burn[m1])) * G / 4.0 / np.mean(mr[c][m1 | m2])
            w = min(n1, n2)
            num_raw += w * d_raw
            num_cor += w * (d_raw - exp_push)
            den += w
        drift[c] = {"raw_N": num_raw / den if den else float("nan"),
                    "burn_corrected_N": num_cor / den if den else float("nan"), "weight": den}
    rec["drift"] = drift
    # SENSITIVITY, never scored (added after the smoke test: the pre-registered
    # |a_x| < 0.1 g straight filter leaves very few NBR samples): |a_x| < 0.2 g
    S2 = base & (np.abs(ax_g) < 2 * STR_G) & (np.abs(ay_g) < STR_G) & (v > STR_V_KPH)
    f2, l2 = S2 & (t < t0 + (t1 - t0) / 3), S2 & (t > t1 - (t1 - t0) / 3)
    sens = {}
    for c in CORNERS:
        num = den = 0.0
        for lo in np.arange(STR_V_KPH, 300, BIN_KPH):
            b = (v >= lo) & (v < lo + BIN_KPH)
            m1, m2 = f2 & b, l2 & b
            w = min(int(m1.sum()), int(m2.sum()))
            if w < MIN_BIN_N:
                continue
            exp_push = -(np.mean(burn[m2]) - np.mean(burn[m1])) * G / 4.0 / np.mean(mr[c][m1 | m2])
            num += w * (np.mean(raw[c][m2]) - np.mean(raw[c][m1]) - exp_push)
            den += w
        sens[c] = {"burn_corrected_N": num / den if den else float("nan"), "weight": den}
    rec["drift_SENSITIVITY_ax0p2_unscored"] = sens
    rec["stationary_pass"] = bool(all(np.isfinite(drift[c]["burn_corrected_N"]) and
                                      abs(drift[c]["burn_corrected_N"]) <= DRIFT_BAR_N for c in CORNERS))

    # stationary on-wheel windows (C1). [impl fix after the smoke test, 2026-10-02:
    # the band is taken from LOW-SPEED driving (5-80 kph, little downforce); the
    # first version used straights > 60 kph, where downforce compresses the
    # suspension, so every stationary sample fell outside it. On jacks the
    # wheels hang at droop, far outside a low-speed band.]
    low = state["moving_mask"] & (v > 5.0) & (v < 80.0)
    band = {c: np.percentile(trav[c][low], [0.5, 99.5]) if low.sum() > 100 else (np.nan, np.nan) for c in CORNERS}
    on_wheels = np.all([(trav[c] >= band[c][0]) & (trav[c] <= band[c][1]) for c in CORNERS], axis=0)
    still_runs = _runs(v < STILL_KPH, t, STILL_MIN_S)
    rec["stationary_windows"] = [{"t0": float(t[a]), "dur_s": float(t[b - 1] - t[a]),
                                  "on_wheels_frac": float(np.mean(on_wheels[a:b]))} for a, b in still_runs]
    K1 = {}
    for c in CORNERS:
        ks = []
        for a, b in still_runs:
            m = np.zeros(t.size, bool)
            m[a:b] = True
            m &= on_wheels & np.isfinite(fz0[c]) & np.isfinite(mr[c])
            if m.sum() >= 50:
                ks.append(float(np.mean((fz0[c][m] - target[c][m]) / mr[c][m])))
        K1[c] = {"K": float(np.mean(ks)) if ks else float("nan"), "n_windows": len(ks),
                 "sd_between_windows": float(np.std(ks)) if len(ks) > 1 else float("nan")}

    # C2 straights, per-lap spread for SE
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    K2 = {}
    for c in CORNERS:
        def fit(m):
            m = m & np.isfinite(fz0[c]) & np.isfinite(mr[c])
            if m.sum() < 50:
                return float("nan"), float("nan")
            A = np.vstack([mr[c][m], v[m] ** 2]).T
            (k, b), *_ = np.linalg.lstsq(A, fz0[c][m] - target[c][m], rcond=None)
            return float(k), float(b)
        k_all, b_all = fit(S)
        per_lap = [fit(S & (t >= l["start_time"]) & (t <= l["end_time"]))[0] for l in an]
        per_lap = [x for x in per_lap if np.isfinite(x)]
        se = float(np.std(per_lap) / np.sqrt(len(per_lap))) if len(per_lap) > 1 else float("nan")
        K2[c] = {"K": k_all, "aero_b": b_all, "per_lap": per_lap, "SE": se}

    choice, u = {}, {}
    for c in CORNERS:
        mr_s = float(np.nanmean(mr[c][S])) if S.any() else float("nan")
        k1, k2 = K1[c]["K"], K2[c]["K"]
        dis = abs(k1 - k2) if np.isfinite(k1) else float("nan")
        use_c1 = np.isfinite(k1) and dis * mr_s <= U_BAR_N
        choice[c] = {"method": "C1" if use_c1 else "C2", "K": k1 if use_c1 else k2, "mr_straight": mr_s,
                     "C1_minus_C2_wheel_N": dis * mr_s if np.isfinite(dis) else float("nan")}
        # [impl fix after the smoke test: an undefined statistical term (one
        # analysis lap, no C1) is insufficient evidence, not zero uncertainty]
        se2 = K2[c]["SE"]
        if not np.isfinite(se2) and not np.isfinite(dis):
            u[c] = float("nan")
        else:
            stat = max(dis if np.isfinite(dis) else 0.0, 2 * se2 if np.isfinite(se2) else 0.0)
            u[c] = float(np.sqrt((stat * mr_s) ** 2 + FUEL_CORNER_N ** 2))
    rec["C1"], rec["C2"], rec["choice"], rec["u_wheel_N"] = K1, K2, choice, u
    rec["u_pass"] = bool(all(np.isfinite(u[c]) and u[c] <= U_BAR_N for c in CORNERS))
    rec["validation"] = _validate({c: fz0[c] - choice[c]["K"] * mr[c] for c in CORNERS}, ay_g, v, base, S)

    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "wb") as fh:
        pickle.dump({"fz0": {c: fz0[c].astype(np.float32) for c in CORNERS},
                     "mr": {c: mr[c].astype(np.float32) for c in CORNERS},
                     "ay_g": ay_g.astype(np.float32), "v": v.astype(np.float32), "base": base, "S": S,
                     "t": t, "s_m": state["s_m"], "laps": an, "W": W}, fh, protocol=pickle.HIGHEST_PROTOCOL)
    with open(os.path.join(OUT_DIR, f"partB_{key}.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    _render(key, t, state["s_m"], {c: fz0[c] - choice[c]["K"] * mr[c] for c in CORNERS}, an, W,
            os.path.join(OUT_DIR, f"loads_{key}.png"), "chosen per-file anchor")
    print(key, "stationary", rec["stationary_pass"], "u_pass", rec["u_pass"], "valid", rec["validation"]["pass"],
          flush=True)


def _validate(fz, ay_g, v, base, S):
    out = {"V1": {}, "V2": {}}
    for c in CORNERS:
        x = fz[c][base]
        out["V1"][c] = float(np.nanpercentile(x, 0.1))
        m = base & np.isfinite(fz[c])
        out["V2"][c] = float(np.corrcoef(ay_g[m], fz[c][m])[0, 1])
    tot = sum(fz[c] for c in CORNERS)
    m = S & np.isfinite(tot)
    A = np.vstack([np.ones(m.sum()), v[m] ** 2]).T
    (a, b), *_ = np.linalg.lstsq(A, tot[m], rcond=None)
    out["V3_aero_b"] = float(b)
    v1 = all(out["V1"][c] >= V1_FLOOR_N for c in CORNERS)
    sign = {"fl": -1, "rl": -1, "fr": 1, "rr": 1}
    v2 = all(np.sign(out["V2"][c]) == sign[c] and abs(out["V2"][c]) >= V2_CORR for c in CORNERS)
    out["pass"] = bool(v1 and v2 and b > 0)
    out["V1_pass"], out["V2_pass"], out["V3_pass"] = bool(v1), bool(v2), bool(b > 0)
    return out


def _render(key, t, s_m, fz, an, W, out_path, note):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND, REF
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True, constrained_layout=True)
    for ax, cs, lab in ((axes[0], ("fl", "fr"), "front axle Fz [N]"), (axes[1], ("rl", "rr"), "rear axle Fz [N]")):
        y = fz[cs[0]] + fz[cs[1]]
        ax.axhline(W[cs[0]] + W[cs[1]], color=REF, lw=0.8, ls="--")
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s_m[m], y[m], color=INK, lw=0.5, alpha=0.6)
        ax.set_ylabel(lab, color=INK)
        ax.grid(axis="y", color=BAND, lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[1].set_xlabel("lap distance [m]", color=INK)
    fig.suptitle(f"Q2 {key}: recovered axle loads, label L4-rel ({note}); dashed = config static axle load "
                 f"(Level 1, event unstated)", color=INK, fontsize=10)
    fig.savefig(out_path, dpi=100)
    plt.close(fig)


def report():
    keys = _nbr_keys()
    recs = {}
    for k in keys:
        p = os.path.join(OUT_DIR, f"partB_{k}.json")
        if os.path.exists(p):
            with open(p) as fh:
                recs[k] = json.load(fh)
    fam = {}
    for c in CORNERS:
        ks = [r["C2"][c]["K"] for r in recs.values() if np.isfinite(r["C2"][c]["K"])]
        fam[c] = {"min": float(np.min(ks)), "max": float(np.max(ks)), "spread": float(np.ptp(ks)),
                  "median": float(np.median(ks))}
    family = all(fam[c]["spread"] <= FAMILY_BAR_N for c in CORNERS)
    n = len(recs)
    stat_n = sum(r["stationary_pass"] for r in recs.values())
    u_n = sum(r["u_pass"] for r in recs.values())
    val_n = sum(r["validation"]["pass"] for r in recs.values())
    c1_files = sum(any(r["C1"][c]["n_windows"] > 0 for c in CORNERS) for r in recs.values())
    c3 = None
    if family:
        c3 = {}
        for k in recs:
            with gzip.open(os.path.join(OUT_DIR, "cache", f"{k}.pkl.gz"), "rb") as fh:
                z = pickle.load(fh)
            fz = {c: z["fz0"][c].astype(float) - fam[c]["median"] * z["mr"][c].astype(float) for c in CORNERS}
            c3[k] = _validate(fz, z["ay_g"].astype(float), z["v"].astype(float), z["base"], z["S"])
    feasible = stat_n >= COUNT_BAR and u_n >= COUNT_BAR and val_n >= COUNT_BAR
    not_feasible = (n - stat_n) > 4
    summary = {"n_files": n, "stationary_pass": stat_n, "u_pass": u_n, "validation_pass": val_n,
               "files_with_C1_reference": c1_files, "C2_offsets_across_files": fam, "shared_family": family,
               "C3_validation_pass": (sum(v["pass"] for v in c3.values()) if c3 else None),
               "EXIT": "FEASIBLE" if feasible else ("NOT FEASIBLE" if not_feasible else "NEITHER (between bars)")}
    lines = [json.dumps(summary, indent=1)]
    for k, r in recs.items():
        lines.append(f"{k}: burn {r['burn_total_kg']:.1f} kg; drift corr " +
                     " ".join(f"{c} {r['drift'][c]['burn_corrected_N']:+.0f} (raw {r['drift'][c]['raw_N']:+.0f})"
                              for c in CORNERS) +
                     f"; C1 windows {[r['C1'][c]['n_windows'] for c in CORNERS]}; K2 " +
                     " ".join(f"{c} {r['C2'][c]['K']:.0f}+/-{r['C2'][c]['SE']:.0f}" for c in CORNERS) +
                     "; choice " + " ".join(f"{c} {r['choice'][c]['method']}" for c in CORNERS) +
                     "; u " + " ".join(f"{c} {r['u_wheel_N'][c]:.0f}" for c in CORNERS) +
                     f"; V1 {r['validation']['V1_pass']} V2 {r['validation']['V2_pass']} V3 {r['validation']['V3_pass']}"
                     f" corr " + " ".join(f"{c} {r['validation']['V2'][c]:+.2f}" for c in CORNERS) +
                     " p0.1 " + " ".join(f"{c} {r['validation']['V1'][c]:.0f}" for c in CORNERS))
    with open(os.path.join(OUT_DIR, "q2.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    with open(os.path.join(OUT_DIR, "q2_summary.json"), "w") as fh:
        json.dump({"summary": summary, "C3": c3}, fh, indent=1)
    print("\n".join(lines))


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "partA":
        part_a()
    elif mode == "partB":
        part_b(sys.argv[2])
    elif mode == "partB_all":
        for k in _nbr_keys():
            part_b(k)
    else:
        report()
