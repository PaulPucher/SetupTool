# Q-RECOVER-2 -- RR standing-rolling correction, target-side check, and the
# same-car voltage reference (PLAN.md PHASE 1 PLAN; author RE-ORDER 2026-10-03;
# READ-ONLY, no production or config change; runs only on the author's
# EXECUTE). Posture (author): a recovery exit asks "can this be characterised
# and subtracted", not "does the one-constant model pass first try"; the bar
# stays (shape validity + bounded uncertainty), the MODEL allowed to hit it
# widens; genuine gauge faults stay excluded.
#
# RULES, frozen 2026-10-03 BEFORE any compute (implementation follows EXECUTE
# and must match this header; any conflict = STOP and ask).
#
# INPUTS: Q-SHARED-OFFSET fit.json (K_shared, per-file rolling K_f, MR_s, u,
#   V1-V3 results), Q2 partB JSON (standing offsets C1_f), Q-CHANNEL-INVENTORY
#   inv_<key>.json (least-squares map gauge = gain x V + offset of the raw
#   amplifier voltages Input 21 / 29 / 31 / 39 Voltage -> FL / FR / RL / RR),
#   the Q2 caches and the raw exports for the Paul Ricard reference files.
#
# R1 -- CONSTANT + CHARACTERISED STANDING-ROLLING CORRECTION (re-order item 1;
#   all four corners reported, RR the target). Signed gap per file
#   d_f = (C1_f - K_shared) x MR_s (wheel N); correction g = median_f d_f.
#   CHARACTERISED iff the leave-one-file-out residual |d_f - g_(-f)| <= 700 N on
#   every file with a standing window (no file defines its own correction).
#   Standing-state uncertainty u_stand = sqrt(u^2 + SD_f(d_f)^2) <= 700 N.
#   The in-motion recovered loads are those of the constant variant, so V1-V3
#   stand as recorded (all pass); stated, not re-run. Linear (season) variant
#   reported beside it. EXIT per corner: RECOVERED WITH CORRECTION / NOT (files
#   named).
#
# R2 -- TARGET-SIDE CHECK (re-order item 2).
#   T1 (premise arithmetic, recorded, not re-run): the target (config corner
#      weight - burn g / 4) enters C1 and K_shared alike; their difference
#      moves only through MR(standing) vs MR(rolling) (RR 0.7797 vs 0.7808), i.e.
#      0.0014-0.0045 N of gap per N of target error -- a 0.6 kN gap would need
#      13-45 t of weight error. The target CANNOT produce the standing-rolling
#      gap; it CAN produce per-file and seasonal offset changes.
#   T2 drift decomposition: per file dW_f,c = (K_f,c - K_shared,c) x MR_s,c / g
#      (kg); projected onto the realisable weight patterns -- common mode
#      (all four, e.g. fuel / ballast), axle (rear minus front), cross-weight
#      diagonal (FL + RR - FR - RL) -- and the RR-only remainder (standard
#      corner-weight statics); Spearman rho of each with outing order. Drift
#      TARGET-EXPLAINED iff >= 50 % of the RR trend is carried by realisable
#      patterns with |rho| >= 0.5; else RR-SPECIFIC.
#   T3 independent reference -- the raw voltage path of the SAME CAR (serial
#      221: NBR and Paul Ricard; lmn_mla and Dubai are car 147, not used).
#      (a) PR zero validation: on v3 and pr_us_v2, the processed gauge loads in
#          on-wheel standing windows (Q2 window rule) vs the config corner
#          weights; PR zero VALID per corner iff within 700 N on both files.
#      (b) voltage-path offset per NBR file and corner K_V,f = offset_NBR,f -
#          offset_PR (inventory maps; FR only if a valid PR map exists -- the
#          v3 FR map is invalid, offset -16 MN) [AMENDED 2026-10-03, before
#          any compute: production already corrects PR FR (config/channels.json
#          channel_corrections, +15,954,360 N); the PR FR reference offset is
#          the inventory map offset PLUS that correction -- FR is assessable];
#          compared with C1_f (standing)
#          and K_f (rolling): per corner, which one the voltage path matches.
#      (c) config check: voltage-path standing loads vs config corner weights,
#          per file and corner. EXIT: CONFIG CONSISTENT (within 700 N on all
#          assessable corners / files -- the gauge reads the car config
#          describes; a remaining gap belongs to the rolling model) / CONFIG
#          MIS-STATES (corners and kg named) / UNDECIDED (PR zero fails (a)).
#
# R3 -- RECOVERY VIA THE REFERENCE (session addition beyond the re-order text;
#   the author may strike it). Offset per corner = the median K_V over the 16
#   NBR files (reference-pinned, not fitted to the NBR target). Uncertainty u_V
#   = sqrt((MR_s x sqrt(SD_f(K_V)^2 + e_PR^2))^2 + 98^2), e_PR the PR-zero
#   residual from T3(a) in gauge units. Same bar as Q-SHARED-OFFSET: u_V <= 700
#   N and V1-V4 on all 16 files. EXIT per corner: RECOVERED VIA REFERENCE / NOT.
#
# OUTPUTS diagnostics/plots/recover_rr/: recover.json, gap_correction.png
#   (signed d_f per corner in outing order with g and the LOO band),
#   drift_patterns.png (T2 components vs outing), voltage_reference.png (per
#   corner: C1_f, K_f, K_V,f in outing order). Titles in words.
#
# Usage: python -m diagnostics.inspect_recover_rr run   (after EXECUTE)
#
# EXECUTE 2026-10-03 (author, R3 IN). [impl] notes, written before compute:
# - Author rulings applied: the nbr_qp_mla jack-drop window is INVALID (ruling
#   2), so that file's standing offset C1 uses window 2 only (per-window values
#   from v4_windows/windows.json); the jst_solution windows stay valid (the E3
#   a_x item was ruled a check construction fault).
# - Rule 0 (author, 2026-10-03, after this header was frozen): config corner
#   weights are a guideline; absolute offsets vs config up to ~1 kN are never
#   alone a disqualifier. Every criterion is computed as frozen; an exit that
#   fails ONLY on a config-absolute item (T3(a), T3(c), R3's V4, R3's e_PR term)
#   is marked "rule 0 applies" for the author.
# - offset_PR per corner = mean of the v3 and pr_us_v2 inventory map offsets
#   (FR plus the production channel correction).
# - e_PR per corner = RMS over the two PR files of (standing on-wheel load -
#   target) / MR at rest, gauge units.
# - T2 basis (orthogonal, FL / FR / RL / RR): common [1,1,1,1], axle
#   [-1,-1,1,1], diagonal [1,-1,-1,1], left-right [-1,1,-1,1]; RR's value is
#   the sum of the four components' RR entries; "RR-only remainder" = the
#   left-right component (not a common, axle or diagonal weight pattern).
#   Trends = OLS slope vs outing rank; share = component slope / RR slope.

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_shared_offset as so
from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/recover_rr"
INV_DIR = "diagnostics/plots/v4_windows/inventory"
CORNERS = so.CORNERS
WORDS = so.WORDS
G = 9.81
BAR_N, FUEL_N, V1_FLOOR_N, V2_CORR = 700.0, 98.0, -200.0, 0.85
SIGN = so.SIGN
PR_KEYS = ("v3", "pr_us_v2")
VOLT = {"fl": "Input 21 Voltage", "fr": "Input 29 Voltage", "rl": "Input 31 Voltage", "rr": "Input 39 Voltage"}
STILL_KPH, STILL_MIN_S, MIN_ON = 3.0, 1.0, 50
BASIS = {"common": np.array([1, 1, 1, 1.0]), "axle": np.array([-1, -1, 1, 1.0]),
         "diagonal": np.array([1, -1, -1, 1.0]), "left-right": np.array([-1, 1, -1, 1.0])}
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2


def _log(m):
    print(m, flush=True)


def _map_offset(key, c):
    p = json.load(open(os.path.join(INV_DIR, f"inv_{key}.json")))["channels"][VOLT[c]]["pair_test"]
    if p["corner"] != c:
        raise SystemExit(f"STOP {key} {c}: voltage channel pairs with {p['corner']}")
    return p["offset_gauge"]


def _c1(keys):
    # Q2 standing offsets; qp_mla from its valid window 2 only (ruling 2)
    win = json.load(open("diagnostics/plots/v4_windows/windows.json"))
    out = {}
    for k in keys:
        if k == "nbr_qp_mla":
            out[k] = {c: win[k]["windows"][1]["corners"][c]["k_window"] for c in CORNERS}
        else:
            out[k] = {c: so.q2(k)["C1"][c]["K"] for c in CORNERS}
    return out


def _slope(y):
    x = np.arange(len(y)) - (len(y) - 1) / 2.0
    return float(np.polyfit(x, y, 1)[0])


def _pr_standing(key):
    from modules.wheel_loads import estimate_wheel_loads_from_dampers, _normalize_travel_to_mm
    from modules.stability_analysis import load_car_data
    from diagnostics.inspect_imu_offsets import read_raw_blocks
    params, data, state = d2._state(key)
    ch, t = data["channels"], state["time"]
    v = state["v_mps"] * 3.6
    ch2 = {**ch, **{f"log_dms_dam_{c}": dict(ch[f"log_dms_dam_{c}"], quality="valid") for c in CORNERS}}
    wl = estimate_wheel_loads_from_dampers(state, ch2, params, load_car_data())
    fz = {c: np.asarray(wl[c]["fz_N"], float) for c in CORNERS}
    mr = {c: np.asarray(wl[c]["motion_ratio"], float) for c in CORNERS}
    if not all(np.nanmedian(np.abs(fz[c])) < 5e4 for c in CORNERS):
        raise SystemExit(f"STOP {key}: a processed PR gauge is not at physical scale (FR correction not applied?)")
    trav = {c: _normalize_travel_to_mm(np.interp(t, np.asarray(ch[f"log_susp_travel_{c}"]["time"], float),
                                                 np.asarray(ch[f"log_susp_travel_{c}"]["data"], float)),
                                       ch[f"log_susp_travel_{c}"]["unit_raw"]) for c in CORNERS}
    fb = read_raw_blocks(d2.sessions()[key]["path"], ["ecu_fuelcons_kg"])["ecu_fuelcons_kg"]
    burn = np.interp(t, fb[0], fb[1]) - np.interp(t[0], fb[0], fb[1])
    cw = params["vehicle"]["corner_weights"]
    W = {"fl": cw["FL_kg"] * G, "fr": cw["FR_kg"] * G, "rl": cw["RL_kg"] * G, "rr": cw["RR_kg"] * G}
    low = state["moving_mask"] & (v > 5.0) & (v < 80.0)
    band = {c: np.percentile(trav[c][low], [0.5, 99.5]) for c in CORNERS}
    on = np.all([(trav[c] >= band[c][0]) & (trav[c] <= band[c][1]) for c in CORNERS], axis=0)
    d = np.diff(np.concatenate(([0], (v < STILL_KPH).astype(int), [0])))
    runs = [(a, b) for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]) if t[b - 1] - t[a] >= STILL_MIN_S]
    rec = {"n_windows": 0, "corners": {}}
    for c in CORNERS:
        diffs, gauge = [], []
        for a, b in runs:
            m = np.zeros(t.size, bool)
            m[a:b] = True
            m &= on & np.isfinite(fz[c])
            if m.sum() >= MIN_ON:
                e = fz[c][m] - (W[c] - burn[m] * G / 4.0)
                diffs.append(float(np.mean(e)))
                gauge.append(float(np.mean(e / mr[c][m])))
        rec["corners"][c] = {"standing_minus_config_N": float(np.mean(diffs)) if diffs else float("nan"),
                             "standing_minus_config_gauge": float(np.mean(gauge)) if gauge else float("nan"),
                             "n_windows": len(diffs)}
    rec["n_windows"] = max(r["n_windows"] for r in rec["corners"].values())
    return rec


def run():
    os.makedirs(OUT_DIR, exist_ok=True)
    fit = json.load(open(os.path.join(so.OUT_DIR, "fit.json")))
    K = fit["files_in_outing_order"]
    C1 = _c1(K)
    out = {"files_in_outing_order": K, "outings": fit["outings"], "R1": {}, "T2": {}, "T3": {}, "R3": {}}

    # R1 -- constant (and linear) + characterised standing-rolling correction
    for c in CORNERS:
        cr = fit["corners"][c]
        mrs = cr["mr_straight"]
        out["R1"][c] = {}
        for var in ("constant", "linear"):
            pred = cr[var]["per_file_pred"]
            dd = np.array([(C1[k][c] - pred[k]) * mrs for k in K])
            ok = np.isfinite(dd)
            g = float(np.median(dd[ok]))
            loo = []
            for i in range(len(K)):
                if not ok[i]:
                    loo.append(float("nan"))
                    continue
                rest = np.delete(dd, i)
                loo.append(float(abs(dd[i] - np.median(rest[np.isfinite(rest)]))))
            u_st = float(np.sqrt(cr[var]["u_wheel_N"] ** 2 + np.std(dd[ok], ddof=1) ** 2))
            char = all(x <= BAR_N for x in loo if np.isfinite(x))
            out["R1"][c][var] = {"d_f_N": dict(zip(K, dd.tolist())), "g_N": g, "loo_resid_N": dict(zip(K, loo)),
                                 "loo_max_N": float(np.nanmax(loo)), "u_N": cr[var]["u_wheel_N"], "u_stand_N": u_st,
                                 "characterised": char, "u_stand_ok": u_st <= BAR_N,
                                 "exit": "RECOVERED WITH CORRECTION" if (char and u_st <= BAR_N) else "NOT",
                                 "files_failing_loo": [k for k, x in zip(K, loo) if np.isfinite(x) and x > BAR_N]}
            _log(f"R1 {c} {var}: g {g:+.0f} N, LOO max {np.nanmax(loo):.0f} N, u_stand {u_st:.0f} N -> "
                 f"{out['R1'][c][var]['exit']}")

    # T2 -- drift as a weight change
    mrs = {c: fit["corners"][c]["mr_straight"] for c in CORNERS}
    dW = np.array([[(fit["corners"][c]["constant"]["per_file_K"][k] - fit["corners"][c]["constant"]["K0"]) * mrs[c] / G
                    for c in CORNERS] for k in K])
    from scipy.stats import spearmanr
    rank = np.arange(len(K))
    comp = {n: dW @ v / 4.0 for n, v in BASIS.items()}
    rr_contrib = {n: comp[n] * BASIS[n][3] for n in BASIS}
    s_rr = _slope(dW[:, 3])
    t2 = {"dW_kg": {k: dict(zip(CORNERS, row.tolist())) for k, row in zip(K, dW)},
          "rr_slope_kg_per_rank": s_rr, "components": {}}
    for n in BASIS:
        s = _slope(rr_contrib[n])
        rho = float(spearmanr(rank, comp[n])[0])
        t2["components"][n] = {"slope_into_rr_kg_per_rank": s, "share_of_rr_trend": s / s_rr if s_rr else float("nan"),
                               "rho_with_outing": rho, "per_file": comp[n].tolist()}
    real = [n for n in ("common", "axle", "diagonal")]
    share = sum(t2["components"][n]["share_of_rr_trend"] for n in real)
    carriers = [n for n in real if abs(t2["components"][n]["rho_with_outing"]) >= 0.5
                and t2["components"][n]["share_of_rr_trend"] > 0]
    t2["realisable_share"] = share
    t2["realisable_carriers_rho_ge_0p5"] = carriers
    t2["exit"] = "TARGET-EXPLAINED" if (share >= 0.5 and carriers) else "RR-SPECIFIC"
    out["T2"] = t2
    _log(f"T2: RR trend {s_rr:+.2f} kg / rank; realisable share {share:.2f}, carriers {carriers} -> {t2['exit']}; "
         + "; ".join(f"{n} share {t2['components'][n]['share_of_rr_trend']:+.2f} rho {t2['components'][n]['rho_with_outing']:+.2f}"
                     for n in BASIS))

    # T3(a) -- PR zero vs config
    pr = {k: _pr_standing(k) for k in PR_KEYS}
    t3a = {}
    for c in CORNERS:
        dn = [pr[k]["corners"][c]["standing_minus_config_N"] for k in PR_KEYS]
        dg = [pr[k]["corners"][c]["standing_minus_config_gauge"] for k in PR_KEYS]
        valid = all(np.isfinite(x) and abs(x) <= BAR_N for x in dn)
        t3a[c] = {"per_file_N": dict(zip(PR_KEYS, dn)), "valid": valid,
                  "e_PR_gauge": float(np.sqrt(np.nanmean(np.square(dg)))),
                  "rule0_applies": (not valid) and all(np.isfinite(x) and abs(x) <= 1000.0 for x in dn)}
        _log(f"T3a {c}: PR standing - config {', '.join(f'{x:+.0f}' for x in dn)} N -> valid {valid}"
             + (" (rule 0 applies)" if t3a[c]["rule0_applies"] else ""))
    out["T3"]["a"] = t3a

    # T3(b) -- voltage-path offsets
    from modules.csv_parser import load_channels_config
    fr_corr = load_channels_config()["channel_corrections"]["log_dms_dam_fr"]["offset"]
    o_pr = {c: float(np.mean([_map_offset(k, c) + (fr_corr if c == "fr" else 0.0) for k in PR_KEYS])) for c in CORNERS}
    KV = {k: {c: _map_offset(k, c) - o_pr[c] for c in CORNERS} for k in K}
    t3b = {"offset_PR_gauge": o_pr, "K_V": KV}
    for c in CORNERS:
        kf = fit["corners"][c]["constant"]["per_file_K"]
        vs_c1 = [(KV[k][c] - C1[k][c]) * mrs[c] for k in K if np.isfinite(C1[k][c])]
        vs_kf = [(KV[k][c] - kf[k]) * mrs[c] for k in K]
        t3b[c] = {"median_KV_minus_C1_N": float(np.median(vs_c1)), "max_abs_KV_minus_C1_N": float(np.max(np.abs(vs_c1))),
                  "median_KV_minus_Kf_N": float(np.median(vs_kf)), "max_abs_KV_minus_Kf_N": float(np.max(np.abs(vs_kf))),
                  "matches": "standing (C1)" if abs(np.median(vs_c1)) < abs(np.median(vs_kf)) else "rolling (K_f)"}
        _log(f"T3b {c}: K_V - C1 median {t3b[c]['median_KV_minus_C1_N']:+.0f} N (max |.| {t3b[c]['max_abs_KV_minus_C1_N']:.0f}); "
             f"K_V - K_f median {t3b[c]['median_KV_minus_Kf_N']:+.0f} N (max |.| {t3b[c]['max_abs_KV_minus_Kf_N']:.0f}) -> matches {t3b[c]['matches']}")
    out["T3"]["b"] = t3b

    # T3(c) -- voltage-path standing loads vs config: (C1 - K_V) x MR
    t3c = {}
    for c in CORNERS:
        e = {k: (C1[k][c] - KV[k][c]) * mrs[c] for k in K if np.isfinite(C1[k][c])}
        t3c[c] = {"per_file_N": e, "max_abs_N": float(np.max(np.abs(list(e.values())))),
                  "median_N": float(np.median(list(e.values()))),
                  "median_kg": float(np.median(list(e.values())) / G)}
    pr_ok = all(t3a[c]["valid"] for c in CORNERS)
    bad = [c for c in CORNERS if t3c[c]["max_abs_N"] > BAR_N]
    t3c["exit"] = ("UNDECIDED" if not pr_ok else ("CONFIG CONSISTENT" if not bad else "CONFIG MIS-STATES"))
    t3c["mis_stating_corners"] = {c: t3c[c]["median_kg"] for c in bad}
    t3c["rule0_applies"] = bool(bad) and all(t3c[c]["max_abs_N"] <= 1000.0 for c in bad)
    out["T3"]["c"] = t3c
    _log(f"T3c: {t3c['exit']}; " + "; ".join(f"{c} median {t3c[c]['median_N']:+.0f} N ({t3c[c]['median_kg']:+.1f} kg) max |.| "
                                             f"{t3c[c]['max_abs_N']:.0f}" for c in CORNERS)
         + (" (rule 0 applies)" if t3c["rule0_applies"] else ""))

    # R3 -- recovery via the reference
    data = {k: so.load(k) for k in K}
    r3 = {}
    tot = {k: 0.0 for k in K}
    for c in CORNERS:
        kv = np.array([KV[k][c] for k in K])
        Kc = float(np.median(kv))
        sd = float(np.std(kv, ddof=1))
        e_pr = t3a[c]["e_PR_gauge"]
        u = float(np.sqrt((mrs[c] * np.sqrt(sd ** 2 + e_pr ** 2)) ** 2 + FUEL_N ** 2))
        u_no_epr = float(np.sqrt((mrs[c] * sd) ** 2 + FUEL_N ** 2))
        v1, v2, v4 = {}, {}, {}
        for k in K:
            C = data[k]
            fz = C["fz0"][c].astype(float) - Kc * C["mr"][c].astype(float)
            tot[k] = tot[k] + fz
            v1[k] = float(np.nanpercentile(fz[C["base"]], 0.1))
            mm = C["base"] & np.isfinite(fz)
            v2[k] = float(np.corrcoef(C["ay_g"][mm], fz[mm])[0, 1])
            v4[k] = float(abs(C1[k][c] - Kc) * so.q2(k)["choice"][c]["mr_straight"]) if np.isfinite(C1[k][c]) else float("nan")
        ok = {"u": u <= BAR_N, "V1": all(x >= V1_FLOOR_N for x in v1.values()),
              "V2": all(np.sign(x) == SIGN[c] and abs(x) >= V2_CORR for x in v2.values()),
              "V4": all(x <= BAR_N for x in v4.values() if np.isfinite(x))}
        r3[c] = {"K_V_shared": Kc, "SD_KV": sd, "e_PR_gauge": e_pr, "u_V_N": u, "u_V_without_ePR_N": u_no_epr,
                 "V1_p01_N": v1, "V2_corr": v2, "V4_err_N": v4, "ok": ok}
    v3 = {}
    for k in K:
        C = data[k]
        m = C["S"] & np.isfinite(tot[k])
        A = np.vstack([np.ones(m.sum()), (C["v"][m].astype(float) / 3.6) ** 2]).T
        v3[k] = float(np.linalg.lstsq(A, tot[k][m], rcond=None)[0][1])
    v3ok = all(x > 0 for x in v3.values())
    out["R3"] = {"corners": r3, "V3_b_per_file": v3, "V3_ok": v3ok}
    for c in CORNERS:
        ok = r3[c]["ok"]
        passed = all(ok.values()) and v3ok
        failing = [n for n, x in ok.items() if not x] + ([] if v3ok else ["V3"])
        config_only = bool(failing) and set(failing) <= {"V4", "u"} and (
            "u" not in failing or r3[c]["u_V_without_ePR_N"] <= BAR_N)
        r3[c]["exit"] = "RECOVERED VIA REFERENCE" if passed else "NOT"
        r3[c]["failing"] = failing
        r3[c]["rule0_applies"] = config_only
        _log(f"R3 {c}: K_V {r3[c]['K_V_shared']:.0f} (SD {r3[c]['SD_KV']:.0f}), u_V {r3[c]['u_V_N']:.0f} N "
             f"(without e_PR {r3[c]['u_V_without_ePR_N']:.0f}); V1 min {min(r3[c]['V1_p01_N'].values()):.0f}; "
             f"V2 min|r| {min(abs(x) for x in r3[c]['V2_corr'].values()):.3f}; V4 max {np.nanmax(list(r3[c]['V4_err_N'].values())):.0f} N "
             f"-> {r3[c]['exit']}" + (f" (failing {failing}; rule 0 applies)" if config_only else (f" (failing {failing})" if failing else "")))
    _log(f"R3 V3: min b {min(v3.values()):.3f} ok {v3ok}")

    with open(os.path.join(OUT_DIR, "recover.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=lp._jsonable)
    _render(out, fit, C1, KV)


def _render(out, fit, C1, KV):
    plt = lp._plt()
    K = out["files_in_outing_order"]
    x = np.arange(len(K))
    # gap correction
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, constrained_layout=True)
    for axp, c in zip(axes.ravel(), CORNERS):
        r = out["R1"][c]["constant"]
        d = np.array([r["d_f_N"][k] for k in K])
        axp.plot(x, d, "o", color=INK, label="standing minus rolling offset, per file")
        axp.axhline(r["g_N"], color=ACC, lw=1.5, label=f"characterised correction {r['g_N']:+.0f} N")
        axp.fill_between(x, r["g_N"] - BAR_N, r["g_N"] + BAR_N, color=ACC, alpha=0.10, lw=0, label="700 N bar around it")
        axp.axhline(0, color=REF, lw=0.8)
        axp.set_xticks(x)
        axp.set_xticklabels(out["outings"], fontsize=7)
        axp.set_ylabel("wheel load (N)", color=INK)
        axp.set_title(f"{WORDS[c]}: {r['exit'].lower()} (leave-one-out max {r['loo_max_N']:.0f} N)", fontsize=9, loc="left")
        axp.legend(frameon=True, fontsize=7, loc="upper left", bbox_to_anchor=(1.0, 1.0))
        lp._style(axp)
    for axp in axes[1]:
        axp.set_xlabel("outing number (chronological)", color=INK)
    fig.suptitle(lp._wrap("Nurburgring, 16 outings -- gap between the gauge offset at rest and the shared rolling offset, "
                          "per corner, with the characterised correction (one shared offset, no season trend)", fig),
                 fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "gap_correction.png"), dpi=100)
    plt.close(fig)
    # drift patterns
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    t2 = out["T2"]
    rr = [t2["dW_kg"][k]["rr"] for k in K]
    ax.plot(x, rr, "o-", color=INK, lw=1.5, label=f"rear right offset as weight (trend {t2['rr_slope_kg_per_rank']:+.2f} kg per outing)")
    cols = {"common": ACC, "axle": ACC2, "diagonal": "#059669", "left-right": "#dc2626"}
    for n, comp in t2["components"].items():
        ax.plot(x, np.array(comp["per_file"]) * BASIS[n][3], "--", color=cols[n], lw=1,
                label=f"{n} pattern, its part of rear right (share {comp['share_of_rr_trend']:+.2f}, rho {comp['rho_with_outing']:+.2f})")
    ax.axhline(0, color=REF, lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(out["outings"], fontsize=7)
    ax.set_xlabel("outing number (chronological)", color=INK)
    ax.set_ylabel("offset change expressed as corner weight (kg)", color=INK)
    ax.legend(frameon=False, fontsize=7)
    lp._style(ax)
    fig.suptitle(lp._wrap(f"Nurburgring, 16 outings -- is the rear-right offset drift a real weight change? "
                          f"Split into corner-weight patterns; read: {t2['exit'].lower()}", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "drift_patterns.png"), dpi=100)
    plt.close(fig)
    # voltage reference
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, constrained_layout=True)
    for axp, c in zip(axes.ravel(), CORNERS):
        kf = fit["corners"][c]["constant"]["per_file_K"]
        axp.plot(x, [C1[k][c] / 1000 for k in K], "s", color=ACC2, label="offset from the car at rest (config weights)")
        axp.plot(x, [kf[k] / 1000 for k in K], "o", color=INK, label="offset fitted on rolling straights")
        axp.plot(x, [KV[k][c] / 1000 for k in K], "^", color=ACC, label="offset from the raw voltage, Paul Ricard zero")
        r3 = out["R3"]["corners"][c]
        axp.axhline(r3["K_V_shared"] / 1000, color=ACC, lw=1)
        axp.set_xticks(x)
        axp.set_xticklabels(out["outings"], fontsize=7)
        axp.set_ylabel("gauge offset (kN, gauge units)", color=INK)
        axp.set_title(f"{WORDS[c]}: {r3['exit'].lower()} (load uncertainty {r3['u_V_N']:.0f} N)", fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
    for axp in axes[1]:
        axp.set_xlabel("outing number (chronological)", color=INK)
    fig.suptitle(lp._wrap("Nurburgring, 16 outings -- damper-gauge offset per corner from three routes: the car at rest, "
                          "the rolling fit, and the raw amplifier voltage referenced to the same car's Paul Ricard zero", fig),
                 fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "voltage_reference.png"), dpi=100)
    plt.close(fig)


def render_only():
    # re-render from recover.json without recomputing (layout fixes only)
    out = json.load(open(os.path.join(OUT_DIR, "recover.json")))
    fit = json.load(open(os.path.join(so.OUT_DIR, "fit.json")))
    _render(out, fit, _c1(out["files_in_outing_order"]), out["T3"]["b"]["K_V"])


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run()
    elif sys.argv[1] == "render":
        render_only()
