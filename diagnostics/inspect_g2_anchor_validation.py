# G2 GPS corner anchor: Gate A, validation battery (Part 2), Gate B, first
# external beta read (Part 3). Extended package 2026-10-02 (PLAN.md PHASE 1
# PLAN Q3 brackets), run on the author's EXECUTE under option (c). Read-only.
#
# RULES, fixed before Part 1 (g2r2) produced any number. [impl] = detail the
# package left open, fixed here.
# Construction = inspect_phase1_q3 _beta_gps as repaired twice: smoothed
#   straight gate (0.3 s), course shifted earlier by 0.30 s, ISO sign (left
#   positive). DRIFT SCOPE LINE: heading between straight re-zeros closed
#   linearly in time (known method limitation, WP5b(c) 2026-07-26); every
#   number below carries it. Course source log_gps_course (NMEA is the same
#   receiver, G0). Per corner-lap value = mean beta_gps over the apex region
#   (>= 3 finite samples, GPS quality held); per corner = mean over laps, SE =
#   sd / sqrt(n), n >= 2.
# GATE A (package): PROMISING iff on >= 6 of the 12 new long NBR sessions the
#   share of anchor-capable corners (SE defined) with SE <= 0.39 deg is >= 0.5.
#   Read from g2_repair2.json. Fail -> closure "GPS corner anchor does not
#   reach discrimination precision at this logging - measured, route closed".
# PART 2 sessions [impl]: the 18 Part-3 sessions (12 new NBR + the pool six
#   v3, pr_us_v2, nbr_jst_problem, nbr_jst_solution, nbr_qp_cdr, nbr_qp_mla).
#   1. Sign/phase sanity, two legs, both must pass.
#      1a straights [impl]: samples with finite beta_gps, smoothed |a_y| <
#         0.1 g, smoothed |r| < 2 deg/s, v > 60 kph (anchor runs are zero by
#         construction and carry no beta; these are the straight samples
#         INSIDE drift-closed segments). Per session |median| < 0.2 deg;
#         pass iff on >= 80 % of sessions with >= 200 such samples.
#      1b sign [impl]: per anchor corner, sign(mean beta) x sign(mean yaw
#         rate at the apex); pass iff one sign holds on >= 80 % of anchor
#         corners pooled. Expected NEGATIVE (steady-state sideslip above the
#         tangent speed opposes the turn; Rajamani ch. 2) -- reported, the
#         bar is consistency, not the expected sign.
#   2. Latency robustness: recompute at 0.25 and 0.35 s; per session median
#      over corners of |corner mean - corner mean at 0.30|; pass iff < 0.2 deg
#      on >= 80 % of sessions, for both shifts.
#   3. SE vs apex speed: Spearman over all anchor corners pooled; reported,
#      no gate.
#   4. Kerb: corner-lap is KERB iff any state kerb_mask sample in its apex
#      region; per corner with >= 1 kerb and >= 1 clean lap, kerb mean minus
#      clean mean; if |median over corners| > 0.5 deg, kerb corner-laps are
#      EXCLUDED for checks 5, 6 and Part 3 and the yield and Gate-A shares
#      are recounted (reported, Gate A not re-decided).
#   5. Lap-count sensitivity: corners with >= 6 laps; for N = 2 .. n, 50
#      random N-subsets, SE_N = sd / sqrt(N); median over corners per N;
#      log-log slope vs N (sqrt scaling = -0.5); minimum N with median SE_N
#      <= 0.39 deg. Reported, no gate.
#   6. Cross-session consistency: same track, same stable corner id, every
#      session pair with SE defined in both; AGREE iff |m_i - m_j| <=
#      sqrt(SE_i^2 + SE_j^2) (1 x combined SE, as written); pass iff >= 60 %
#      of pairs pooled.
#   GATE B: VALIDATED iff checks 1, 2 and 6 pass. Fail -> closure "anchor
#   forms but fails validation on [check], route closed at this logging".
# PART 3 (only past Gate B), UNSCORED, label "validated-anchor first read":
#   validated anchor corner [impl] = SE <= 0.39 deg. Per session and corner:
#   beta_gps mean +/- SE vs production EKF beta (cached production chain) and
#   kinematic beta (estimate_sideslip) averaged over the SAME apex samples.
#   Report: deviation distributions (EKF - GPS, KIN - GPS), per-session median
#   (offset; a constant offset is EXPECTED, mounting degeneracy, Door 1) and
#   spread across corners (the information); Spearman across sessions of
#   offset and spread against the session's production fit B, C, D, E per
#   axle. No verdict, no criterion, no estimator conclusion.
# Outputs diagnostics/plots/g2_anchor_validation/. Mode: all (gated).

import json
import os
import sys
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_phase1_q3 as q3
from diagnostics.inspect_imu_offsets import read_raw_blocks as _rrb
from diagnostics.trace_render import INK, BAND, REF

OUT = "diagnostics/plots/g2_anchor_validation"
NEW12 = q3.NEW12
POOL6 = ("v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla")
SESS = tuple(NEW12) + POOL6
BAR = q3.ANCHOR_BAR_DEG
DELAYS = (0.25, 0.30, 0.35)
RNG = np.random.default_rng(20261002)
G = 9.81

_RAW = {}


def _cached_raw(path, names):
    k = (path, tuple(sorted(names)))
    if k not in _RAW:
        _RAW[k] = _rrb(path, names)
    return _RAW[k]


q3.read_raw_blocks = _cached_raw  # course block read once per session, not per delay


def _save(name, obj):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=float)


def _spearman(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 4:
        return float("nan"), int(m.sum())
    rx, ry = np.argsort(np.argsort(x[m])), np.argsort(np.argsort(y[m]))
    return float(np.corrcoef(rx, ry)[0, 1]), int(m.sum())


def gate_a():
    g = q3._load("g2_repair2.json")
    rows, n_pass = {}, 0
    for k in NEW12:
        c = g.get(k, {}).get("log_gps_course", {}).get("corners", {})
        ses = [v["se_deg"] for v in c.values() if np.isfinite(v["se_deg"])]
        share = float(np.mean(np.array(ses) <= BAR)) if ses else float("nan")
        ok = bool(ses and share >= 0.5)
        n_pass += ok
        rows[k] = {"capable_corners": len(ses), "share_le_bar": share, "pass": ok}
    res = {"sessions": rows, "n_pass": n_pass, "promising": n_pass >= 6}
    _save("gate_a.json", res)
    print(f"[gate A] {n_pass} of 12 sessions pass -> {'PROMISING' if res['promising'] else 'FAIL'}; "
          + ", ".join(f"{k[4:]} {r['share_le_bar']:.2f}" for k, r in rows.items()), flush=True)
    return res


# ---------------------------------------------------------------- per-session extraction

def _session(key, delays=DELAYS):
    from modules.stability_analysis import estimate_sideslip
    params, data, state = d2._state(key)
    # Part 3 reads EKF / kinematic beta on the same apex samples; computed here
    # so the full state is not held for 18 sessions
    ekf = np.degrees(np.asarray(d2.load_fit(key)["prod"]["beta"], float))
    kin = np.degrees(np.asarray(estimate_sideslip(state, params), float))
    pims = q3._pims(key, params, data, state)
    t = state["time"]
    r_d = np.degrees(state["yaw_rate_radps"])
    kerb = state.get("kerb_mask")
    out = {"delays": {}}
    for dl in delays:
        bg, info = q3._beta_gps(key, params, data, state, "log_gps_course", dl, smooth=True, iso=True)
        laps = {}
        for (cid, lap), ph in pims.items():
            idx = ph["apex_3"]
            if idx.size == 0:
                continue
            v = bg[idx]
            ok = np.isfinite(v)
            if ok.sum() < 3 or (info["qual"] is not None and not info["qual"][idx].all()):
                continue
            laps.setdefault(cid, []).append({"lap": lap, "beta": float(np.mean(v[ok])), "idx": idx,
                                             "r": float(np.mean(r_d[idx])), "v": float(np.mean(state["v_mps"][idx] * 3.6)),
                                             "kerb": bool(kerb is not None and np.any(kerb[idx])),
                                             "ekf": float(np.mean(ekf[idx])), "kin": float(np.mean(kin[idx]))})
        out["delays"][dl] = {"laps": laps, "anchor_runs": info["n_anchor_runs"]}
        if dl == 0.30:
            bg30 = bg
    if 0.30 in out["delays"]:
        bg = bg30
        ay_s = q3._mavg(state["ay_mps2"] / G, 30)
        r_s = q3._mavg(r_d, 30)
        st = np.isfinite(bg) & (np.abs(ay_s) < 0.1) & (np.abs(r_s) < 2) & (state["v_mps"] * 3.6 > 60)
        out["straight"] = {"n": int(st.sum()), "median": float(np.median(bg[st])) if st.any() else float("nan")}
    out["fit"] = {f"{a[0]}_{p}": float(d2.load_fit(key)["prod"]["manifest"]["axles"][a][p])
                  for a in ("front", "rear") for p in "BCDE"}
    for L in out["delays"][0.25]["laps"].values():
        for x in L:
            x.pop("idx", None)
    for L in out["delays"][0.35]["laps"].values():
        for x in L:
            x.pop("idx", None)
    for L in out["delays"][0.30]["laps"].values():
        for x in L:
            x.pop("idx", None)
    return out


def _corner_stats(laps, use_kerb=True):
    res = {}
    for cid, L in laps.items():
        vals = [x["beta"] for x in L if use_kerb or not x["kerb"]]
        if len(vals) >= 2:
            v = np.array(vals)
            res[cid] = {"n": len(v), "mean": float(v.mean()), "se": float(v.std(ddof=1) / np.sqrt(len(v))),
                        "r": float(np.mean([x["r"] for x in L])), "v": float(np.mean([x["v"] for x in L]))}
    return res


# ---------------------------------------------------------------- Part 2

def part2(S):
    res = {}
    # 1a / 1b
    st_ok = [abs(S[k]["straight"]["median"]) < 0.2 for k in S if S[k]["straight"]["n"] >= 200]
    signs = []
    for k in S:
        for c in _corner_stats(S[k]["delays"][0.30]["laps"]).values():
            if c["mean"] != 0 and c["r"] != 0:
                signs.append(np.sign(c["mean"]) * np.sign(c["r"]))
    signs = np.array(signs)
    dom = float(max(np.mean(signs > 0), np.mean(signs < 0))) if signs.size else float("nan")
    res["c1"] = {"straight_median_deg": {k: S[k]["straight"] for k in S},
                 "straight_share_pass": float(np.mean(st_ok)) if st_ok else float("nan"),
                 "sign_share_negative": float(np.mean(signs < 0)) if signs.size else float("nan"),
                 "sign_dominant_share": dom, "n_corners": int(signs.size)}
    res["c1"]["pass"] = bool(st_ok and np.mean(st_ok) >= 0.8 and dom >= 0.8)
    # 2 latency
    lat = {}
    for k in S:
        base = _corner_stats(S[k]["delays"][0.30]["laps"])
        row = {}
        for dl in (0.25, 0.35):
            other = _corner_stats(S[k]["delays"][dl]["laps"])
            d = [abs(other[c]["mean"] - base[c]["mean"]) for c in base if c in other]
            row[str(dl)] = float(np.median(d)) if d else float("nan")
        lat[k] = row
    shares = {dl: float(np.mean([lat[k][dl] < 0.2 for k in lat if np.isfinite(lat[k][dl])])) for dl in ("0.25", "0.35")}
    res["c2"] = {"per_session_median_shift_deg": lat, "share_sessions_lt_0.2": shares,
                 "pass": bool(all(v >= 0.8 for v in shares.values()))}
    # 3 SE vs speed
    se, sp = [], []
    for k in S:
        for c in _corner_stats(S[k]["delays"][0.30]["laps"]).values():
            se.append(c["se"])
            sp.append(c["v"])
    rho, n = _spearman(sp, se)
    res["c3"] = {"spearman_se_vs_apex_speed": rho, "n": n}
    # 4 kerb
    diffs = []
    for k in S:
        for L in S[k]["delays"][0.30]["laps"].values():
            kb = [x["beta"] for x in L if x["kerb"]]
            cl = [x["beta"] for x in L if not x["kerb"]]
            if kb and cl:
                diffs.append(np.mean(kb) - np.mean(cl))
    med = float(np.median(diffs)) if diffs else float("nan")
    exclude = bool(np.isfinite(med) and abs(med) > 0.5)
    res["c4"] = {"n_corners": len(diffs), "median_kerb_minus_clean_deg": med, "exclude_kerb": exclude}
    if exclude:
        recount = {}
        for k in NEW12:
            cs = _corner_stats(S[k]["delays"][0.30]["laps"], use_kerb=False)
            ses = [c["se"] for c in cs.values()]
            recount[k] = {"capable": len(ses), "share_le_bar": float(np.mean(np.array(ses) <= BAR)) if ses else float("nan"),
                          "clean_corner_laps": int(sum(1 for L in S[k]["delays"][0.30]["laps"].values() for x in L if not x["kerb"]))}
        res["c4"]["recount_new12"] = recount
    use_kerb = not exclude
    # 5 lap subsampling
    curves = {}
    for k in S:
        for L in S[k]["delays"][0.30]["laps"].values():
            v = np.array([x["beta"] for x in L if use_kerb or not x["kerb"]])
            if v.size < 6:
                continue
            for N in range(2, v.size + 1):
                sub = [RNG.choice(v, N, replace=False) for _ in range(50)]
                curves.setdefault(N, []).append(float(np.median([s.std(ddof=1) / np.sqrt(N) for s in sub])))
    Ns = sorted(N for N in curves if len(curves[N]) >= 5)
    med_se = {N: float(np.median(curves[N])) for N in Ns}
    slope = float(np.polyfit(np.log(Ns), np.log([med_se[N] for N in Ns]), 1)[0]) if len(Ns) >= 3 else float("nan")
    min_n = next((N for N in Ns if med_se[N] <= BAR), None)
    res["c5"] = {"median_se_by_N": med_se, "loglog_slope": slope, "min_laps_for_bar": min_n}
    # 6 cross-session
    tracks = {}
    for k in S:
        tracks.setdefault(d2.sessions()[k]["track"], []).append(k)
    agree = tot = 0
    for tr, ks in tracks.items():
        cs = {k: _corner_stats(S[k]["delays"][0.30]["laps"], use_kerb) for k in ks}
        for i, a in enumerate(ks):
            for b in ks[i + 1:]:
                for c in set(cs[a]) & set(cs[b]):
                    tot += 1
                    agree += abs(cs[a][c]["mean"] - cs[b][c]["mean"]) <= np.hypot(cs[a][c]["se"], cs[b][c]["se"])
    res["c6"] = {"pairs": tot, "agree": int(agree), "share": agree / tot if tot else float("nan")}
    res["c6"]["pass"] = bool(tot and agree / tot >= 0.6)
    res["gate_b"] = {"validated": bool(res["c1"]["pass"] and res["c2"]["pass"] and res["c6"]["pass"]),
                     "failed": [c for c in ("c1", "c2", "c6") if not res[c]["pass"]]}
    _save("part2.json", res)
    for c in ("c1", "c2", "c3", "c4", "c5", "c6"):
        print(f"[part2 {c}] " + json.dumps({k: v for k, v in res[c].items()
                                            if k not in ("straight_median_deg", "per_session_median_shift_deg", "recount_new12")}),
              flush=True)
    print(f"[gate B] {'VALIDATED' if res['gate_b']['validated'] else 'FAIL on ' + ', '.join(res['gate_b']['failed'])}",
          flush=True)
    _render_part2(S, res, use_kerb)
    return res, use_kerb


def _render_part2(S, res, use_kerb):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    ks = list(S)
    ax[0, 0].bar(range(len(ks)), [S[k]["straight"]["median"] for k in ks], color=INK)
    ax[0, 0].axhline(0.2, color=REF, lw=0.8)
    ax[0, 0].axhline(-0.2, color=REF, lw=0.8)
    ax[0, 0].set_title("1a straight median beta_gps [deg] per session", color=INK, fontsize=9)
    rr, bb = [], []
    for k in ks:
        for c in _corner_stats(S[k]["delays"][0.30]["laps"]).values():
            rr.append(c["r"])
            bb.append(c["mean"])
    ax[0, 1].scatter(rr, bb, s=6, color=INK)
    ax[0, 1].axhline(0, color=REF, lw=0.6)
    ax[0, 1].axvline(0, color=REF, lw=0.6)
    ax[0, 1].set_xlabel("apex yaw rate [deg/s]", color=INK)
    ax[0, 1].set_title("1b corner beta_gps [deg] vs yaw rate", color=INK, fontsize=9)
    lat = res["c2"]["per_session_median_shift_deg"]
    ax[0, 2].plot(range(len(ks)), [lat[k]["0.25"] for k in ks], "o", color=INK, label="0.25 s")
    ax[0, 2].plot(range(len(ks)), [lat[k]["0.35"] for k in ks], "s", color=REF, label="0.35 s")
    ax[0, 2].axhline(0.2, color=REF, lw=0.8)
    ax[0, 2].legend(frameon=False, fontsize=7)
    ax[0, 2].set_title("2 median |shift| vs 0.30 s [deg]", color=INK, fontsize=9)
    se, sp = [], []
    for k in ks:
        for c in _corner_stats(S[k]["delays"][0.30]["laps"]).values():
            se.append(c["se"])
            sp.append(c["v"])
    ax[1, 0].scatter(sp, se, s=6, color=INK)
    ax[1, 0].axhline(BAR, color=REF, lw=0.8)
    ax[1, 0].set_xlabel("apex speed [kph]", color=INK)
    ax[1, 0].set_title("3 corner SE [deg] vs apex speed", color=INK, fontsize=9)
    m5 = res["c5"]["median_se_by_N"]
    if m5:
        ax[1, 1].loglog(list(m5), list(m5.values()), "o-", color=INK)
        ax[1, 1].axhline(BAR, color=REF, lw=0.8)
    ax[1, 1].set_xlabel("laps averaged N", color=INK)
    ax[1, 1].set_title(f"5 median SE vs N (slope {res['c5']['loglog_slope']:.2f})", color=INK, fontsize=9)
    ax[1, 2].axis("off")
    ax[1, 2].text(0, 0.5, "\n".join([f"c1 pass {res['c1']['pass']}", f"c2 pass {res['c2']['pass']}",
                                     f"c4 kerb median {res['c4']['median_kerb_minus_clean_deg']:.2f} deg",
                                     f"c6 agree {res['c6']['agree']}/{res['c6']['pairs']} pass {res['c6']['pass']}",
                                     f"Gate B validated {res['gate_b']['validated']}"]), color=INK, fontsize=10)
    fig.suptitle("G2 anchor validation battery (drift closed linearly in time -- scope line)", color=INK, fontsize=10)
    fig.savefig(os.path.join(OUT, "part2_battery.png"), dpi=90)
    plt.close(fig)


# ---------------------------------------------------------------- Part 3

def part3(S, use_kerb):
    rows, per_sess = [], {}
    for k in S:
        laps = S[k]["delays"][0.30]["laps"]
        cs = _corner_stats(laps, use_kerb)
        dev_e, dev_k = [], []
        for cid, c in cs.items():
            if c["se"] > BAR:
                continue
            L = [x for x in laps[cid] if use_kerb or not x["kerb"]]
            e = float(np.mean([x["ekf"] for x in L]))
            kn = float(np.mean([x["kin"] for x in L]))
            rows.append({"session": k, "corner": cid, "gps": c["mean"], "gps_se": c["se"], "ekf": e, "kin": kn,
                         "apex_speed": c["v"], "apex_r": c["r"]})
            dev_e.append(e - c["mean"])
            dev_k.append(kn - c["mean"])
        per_sess[k] = {"n_corners": len(dev_e),
                       "ekf_minus_gps_median": float(np.median(dev_e)) if dev_e else float("nan"),
                       "ekf_minus_gps_spread_sd": float(np.std(dev_e, ddof=1)) if len(dev_e) >= 2 else float("nan"),
                       "kin_minus_gps_median": float(np.median(dev_k)) if dev_k else float("nan"),
                       "kin_minus_gps_spread_sd": float(np.std(dev_k, ddof=1)) if len(dev_k) >= 2 else float("nan"),
                       "fit": S[k]["fit"]}
    corr = {}
    for q in ("ekf_minus_gps_median", "ekf_minus_gps_spread_sd"):
        for p in per_sess[next(iter(per_sess))]["fit"]:
            corr[f"{q} ~ {p}"] = _spearman([per_sess[k][q] for k in per_sess], [per_sess[k]["fit"][p] for k in per_sess])
    pooled = {"ekf_minus_gps": np.array([r["ekf"] - r["gps"] for r in rows]),
              "kin_minus_gps": np.array([r["kin"] - r["gps"] for r in rows])}
    summ = {n: {"n": int(v.size), "median": float(np.median(v)) if v.size else float("nan"),
                "p10_p90": [float(np.percentile(v, 10)), float(np.percentile(v, 90))] if v.size else None}
            for n, v in pooled.items()}
    rho_e, _ = _spearman([r["gps"] for r in rows], [r["ekf"] for r in rows])
    rho_k, _ = _spearman([r["gps"] for r in rows], [r["kin"] for r in rows])
    res = {"label": "validated-anchor first read -- UNSCORED, no verdict", "pooled": summ,
           "spearman_corner_gps_vs_ekf": rho_e, "spearman_corner_gps_vs_kin": rho_k,
           "per_session": per_sess, "session_corr_with_fit": corr, "rows": rows}
    _save("part3.json", res)
    print(f"[part3] corners {len(rows)}; pooled {json.dumps(summ)}; corner rho gps~ekf {rho_e:.2f} gps~kin {rho_k:.2f}",
          flush=True)
    for k, v in per_sess.items():
        print(f"  {k}: n {v['n_corners']} ekf-gps med {v['ekf_minus_gps_median']:+.2f} sd {v['ekf_minus_gps_spread_sd']:.2f}; "
              f"kin-gps med {v['kin_minus_gps_median']:+.2f} sd {v['kin_minus_gps_spread_sd']:.2f}", flush=True)
    for c, (r, n) in corr.items():
        print(f"  corr {c}: rho {r:+.2f} (n {n})", flush=True)
    _render_part3(rows, per_sess)


def _render_part3(rows, per_sess):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    g = np.array([r["gps"] for r in rows])
    for a, key, t in ((ax[0], "ekf", "production EKF beta"), (ax[1], "kin", "kinematic beta")):
        y = np.array([r[key] for r in rows])
        a.errorbar(g, y, xerr=[r["gps_se"] for r in rows], fmt="o", ms=3, color=INK, ecolor=BAND, lw=0.8)
        lim = [np.nanmin(np.r_[g, y]), np.nanmax(np.r_[g, y])]
        a.plot(lim, lim, color=REF, lw=0.8)
        a.set_xlabel("beta_gps corner mean [deg] (ISO)", color=INK)
        a.set_ylabel(f"{t} [deg]", color=INK)
        a.set_title(f"{t} vs GPS anchor, validated corners", color=INK, fontsize=9)
    ks = list(per_sess)
    ax[2].errorbar(range(len(ks)), [per_sess[k]["ekf_minus_gps_median"] for k in ks],
                   yerr=[per_sess[k]["ekf_minus_gps_spread_sd"] for k in ks], fmt="o", color=INK, ecolor=REF)
    ax[2].set_xticks(range(len(ks)))
    ax[2].set_xticklabels([k.replace("nbr_", "") for k in ks], rotation=60, fontsize=7)
    ax[2].set_title("EKF - GPS per session: median (offset) +/- sd across corners", color=INK, fontsize=9)
    fig.suptitle("Validated-anchor first read -- UNSCORED (drift closed linearly in time -- scope line)", color=INK, fontsize=10)
    fig.savefig(os.path.join(OUT, "part3_first_read.png"), dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    try:
        ga = gate_a()
        if not ga["promising"]:
            print("[stop] Gate A failed -> closure, Part 5", flush=True)
            sys.exit(0)
        S = {}
        for k in SESS:
            try:
                S[k] = _session(k)
                print(f"[extract {k}] runs {S[k]['delays'][0.30]['anchor_runs']}, straight n {S[k]['straight']['n']}", flush=True)
            except Exception:
                print(f"[extract {k}] ERROR\n{traceback.format_exc()}", flush=True)
        res, use_kerb = part2(S)
        if not res["gate_b"]["validated"]:
            print("[stop] Gate B failed -> closure, Part 5", flush=True)
            sys.exit(0)
        part3(S, use_kerb)
    except Exception:
        print(f"[run] STEP ERROR\n{traceback.format_exc()}", flush=True)
