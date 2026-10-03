# Q-PAIRING -- what the position-locked deep CS dips ARE (PLAN.md PHASE 1 PLAN
# Q-PAIRING; governed by section 2b; read-only, every variant in memory).
#
# QUESTION: 17 of 22 position-locked dips exceed what the session's own fitted
# tyre curve can produce (Q-CHAIR-AUDIT exhibit). Three mechanisms are held
# open together: (A) slip-side error (alpha wrong at the site), (B) both
# channels disturbed by a kerb strike, (C) alpha-Fy TIMING mismatch (a lag
# between the slip path and the force path turns a real event into a false
# steep slope). Production is the candidate, not the standard.
#
# RULES, frozen 2026-10-03 BEFORE any run (author work order Q-PAIRING,
# EXECUTE received with the Part F(b) message).
# SESSIONS: Tier 1 = pr_us_v2, v3, nbr_jst_solution. Chain: production (cached
#   EKF beta, production parameters); CS by inspect_chair_audit.cs_loop with the
#   production rule (entry-checked there). SITES: the 22 position-locked sites
#   of stiffness_cs/spike_<key>.json (production, known_real = recurring).
#   Per site and analysis lap: the dip sample = the lap's CS minimum within
#   +/-10 m of the site inside a corner bracket.
# 1 PER-SITE WINDOW CONTENTS (per lap):
#   (C) LAG: cross-correlation of detrended alpha_filt and Fy_filt over the
#       segment dip +/- 1.0 s, lag searched in +/-0.3 s at the grid step;
#       positive = Fy lags alpha. RE-PAIRING: alpha shifted by the lap's lag
#       (alpha(t - lag) paired with Fy(t)) and CS re-estimated at the dip sample
#       with the production window rule, section blend and the production running
#       linear reference at that sample (single-sample estimator; ENTRY CHECK:
#       zero shift reproduces production CS at the dip).
#   (A) SLIP GAP: |alpha from EKF beta - alpha from the kinematic seed| at the
#       dip sample (deg), against the in-corner distribution of the same gap.
#   (B) DISTURBANCE: kerb-mask overlap of the CS window (any kerb sample);
#       az activity = max |az - its 2 Hz low-pass| in the window; damper
#       activity = max over the axle's two gauges of |load - its 2 Hz
#       low-pass| in the window (shape only; offsets cancel); each against
#       the in-corner p95 of the same quantity.
# 2 GLOBAL LAG: the same cross-correlation per corner instance (bracket segment,
#   >= 1.5 s) -> in-corner lag distribution per session and axle; sites vs
#   corners.
# 3 SYNTHETIC LAG (decisive for C): Fy delayed against alpha by tau in
#   {-0.10, -0.05, -0.02, 0, +0.02, +0.05, +0.10, +0.20} s over the whole
#   session, CS recomputed with the production rule; on CLEAN stretches (in-
#   bracket, non-kerb, > 30 m from every site): p1, minimum and count of
#   excursions below -1 per tau = the lag-vs-depth curve. C SUFFICIENT iff at
#   some |tau| <= the largest |median site lag| of the axle the clean-stretch
#   p1 reaches the median of the sites' median dip depth.
# 4 VERDICT per site (flags, combinable; none -> UNEXPLAINED):
#   C iff |median site lag - in-corner median lag| > in-corner lag IQR AND the
#     re-paired median CS lies at or above the session's curve-shape bound
#     (CS scale, Q-CHAIR-AUDIT) -- the dip leaves the model-incompatible range;
#   A iff the median slip gap at the site > in-corner p90 of the gap;
#   B iff on >= half the laps the window overlaps the kerb mask OR az or damper
#     activity exceeds its in-corner p95.
#   Mechanism shares = site counts per verdict. Read under the frame: flags say
#   which mechanism is CONSISTENT with the data at a site, not which is proven.
# Outputs diagnostics/plots/pairing/.
#
# Usage: python -m diagnostics.inspect_pairing sites <key> | synth <key> | report

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_last_push as lp
from diagnostics import inspect_stiffness_cs as isc
from diagnostics import inspect_chair_audit as ica

OUT_DIR = "diagnostics/plots/pairing"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
RAD2DEG = 180.0 / np.pi
SEG_S, LAG_MAX_S = 1.0, 0.3
SITE_TOL_M, CLEAN_EXCL_M = 10.0, 30.0
TAUS = (-0.10, -0.05, -0.02, 0.0, 0.02, 0.05, 0.10, 0.20)
MIN_CORNER_S = 1.5
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2


def _dump(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=lp._jsonable)


def _log(m):
    print(m, flush=True)


def xcorr_lag(a, f, sr):
    """Lag (s) maximising corr(a(t), f(t + lag)); positive = f lags a."""
    a = a - np.polyval(np.polyfit(np.arange(a.size), a, 1), np.arange(a.size))
    f = f - np.polyval(np.polyfit(np.arange(f.size), f, 1), np.arange(f.size))
    k = int(round(LAG_MAX_S * sr))
    best, bl = -np.inf, 0
    for s in range(-k, k + 1):
        if s >= 0:
            x, y = a[:a.size - s], f[s:]
        else:
            x, y = a[-s:], f[:f.size + s]
        if x.size < 10 or np.std(x) == 0 or np.std(y) == 0:
            continue
        c = float(np.corrcoef(x, y)[0, 1])
        if c > best:
            best, bl = c, s
    return bl / sr, best


def single_cs(alpha, Fy, i, moving, s_m, params, sr, ref):
    """Production window rule and C_alpha blend at one sample, ratio against a
    given running linear reference. Sections from a local slice."""
    from modules.stability_analysis import (_find_monotonic_sections, _section_slopes, _smooth_weight,
                                            SPAN_WEIGHT_EXPONENT, R2_WEIGHT_EXPONENT, resolve_cs_min_window_samples)
    se = params["stability_estimation"]
    min_span, max_m = se["cs_min_slip_angle_span_rad"], se["cs_max_window_m"]
    mw = resolve_cs_min_window_samples(params, sr)
    if not moving[i] or not np.isfinite(ref) or ref <= 0:
        return float("nan")
    start = i - mw
    s_i = s_m[i - 1]
    amax, amin = np.max(alpha[start:i]), np.min(alpha[start:i])
    while start > 0:
        if amax - amin >= min_span:
            break
        s0 = s_m[start]
        if not np.isfinite(s0) or s0 > s_i or (s_i - s0) >= max_m:
            break
        start -= 1
        amax, amin = max(amax, alpha[start]), min(amin, alpha[start])
    wa, wf = alpha[start:i], Fy[start:i]
    if np.max(wa) - np.min(wa) < min_span:
        return float("nan")
    lo = max(0, start - int(5 * sr))
    hi = min(alpha.size, i + int(5 * sr))
    secs, sid = _find_monotonic_sections(alpha[lo:hi])
    sl, sp = _section_slopes(alpha[lo:hi], Fy[lo:hi], secs)
    am, fm = np.mean(wa), np.mean(wf)
    den = np.sum((wa - am) ** 2)
    if den < 1e-10:
        return float("nan")
    Cw = np.sum((wa - am) * (wf - fm)) / den
    fh = Cw * wa + (fm - Cw * am)
    sst = np.sum((wf - fm) ** 2)
    R2 = 1 - np.sum((wf - fh) ** 2) / sst if sst > 1e-10 else 0.0
    ws, ss = [], []
    for k in np.unique(sid[start - lo:i - lo]):
        if np.isnan(sl[k]):
            continue
        w = _smooth_weight(sp[k], 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
        if w > 0:
            ws.append(w)
            ss.append(sl[k])
    if ws:
        Cs = float(np.sum(np.array(ws) * np.array(ss)) / np.sum(ws))
        wr = _smooth_weight(R2, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
        Ca = wr * Cw + (1 - wr) * Cs
    else:
        Ca = Cw
    return min(Ca / ref, 1.0)


class S:
    """Session context: production chain arrays."""

    def __init__(self, key):
        from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
        self.ctx = ctx = lp.Ctx(key)
        st, prm = ctx.state, ctx.params
        self.sr = st["sample_rate_hz"]
        self.mv = st["moving_mask"] & ~st["kerb_mask"] if st.get("kerb_mask") is not None else st["moving_mask"].copy()
        beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
        self.slip = estimate_slip_angles(st, beta, prm)
        self.kin = estimate_slip_angles(st, estimate_sideslip(st, prm), prm)
        self.fo = estimate_lateral_forces(st, prm)
        self.br = ctx.base & ctx.in_br
        self.spike = json.load(open(os.path.join(isc.OUT_DIR, f"spike_{key}.json")))
        self.audit = json.load(open(os.path.join(ica.OUT_DIR, f"audit_{key}.json")))
        self.laps = isc._lap_slices(ctx)

    def arrays(self, ax):
        a_key, f_key = ("alpha_f_filt", "Fy_f_filt") if ax == "f" else ("alpha_r_filt", "Fy_r_filt")
        return self.slip[a_key], self.kin[a_key], self.fo[f_key]

    def hp(self, x):
        from modules.stability_analysis import _butterworth_lowpass
        return np.abs(x - _butterworth_lowpass(x, 2.0, self.sr))


def corner_segments(ctx, min_n):
    out = []
    for sid, lap, sl in ctx.inst:
        if sl.stop - sl.start >= min_n:
            out.append((sid, lap, sl))
    return out


def sites(key):
    s = S(key)
    ctx, st, prm, sr = s.ctx, s.ctx.state, s.ctx.params, s.sr
    s_m, t = st["s_m"], st["time"]
    az = st.get("az_g")
    az_hp = s.hp(az) if az is not None else None
    dam = {k: ctx.data["channels"].get(f"log_dms_dam_{k}") for k in ("fl", "fr", "rl", "rr")}
    out = {"key": key, "session": ctx.name, "axles": {}}
    for ax in ("f", "r"):
        alpha, akin, Fy = s.arrays(ax)
        prod = ica.cs_loop(alpha, Fy, s.mv, s_m, prm, sr, "production")
        gap = np.abs(alpha - akin) * RAD2DEG
        pair = ("fl", "fr") if ax == "f" else ("rl", "rr")
        dhp = None
        if all(dam[k] is not None for k in pair):
            dhp = np.maximum(*(s.hp(np.interp(t, dam[k]["time"], dam[k]["data"])) for k in pair))
        # global in-corner references
        segs = corner_segments(ctx, int(MIN_CORNER_S * sr))
        lags = [xcorr_lag(alpha[sl], Fy[sl], sr)[0] for _sid, _lap, sl in segs]
        ref_gap_p90 = float(np.percentile(gap[s.br], 90))
        ref_az_p95 = float(np.percentile(az_hp[s.br], 95)) if az_hp is not None else float("nan")
        ref_dam_p95 = float(np.percentile(dhp[s.br], 95)) if dhp is not None else float("nan")
        lq = np.percentile(lags, [25, 50, 75])
        bound = s.audit["axles"][ax]["curve_bound"]["bound_cs_scale_model"]
        A = {"global": {"corner_lags_s": lags, "lag_median_s": float(lq[1]), "lag_iqr_s": float(lq[2] - lq[0]),
                        "gap_p90_deg": ref_gap_p90, "az_hp_p95_g": ref_az_p95, "damper_hp_p95_N": ref_dam_p95,
                        "curve_bound_cs_scale": bound}, "sites": []}
        entry = []
        for d in s.spike["variants"]["production"][ax]["dips"]:
            if not d["known_real"]:
                continue
            rows = []
            for lap, lm in s.laps:
                k = lm & np.isfinite(prod["cs"]) & (np.abs(s_m - d["pos_m"]) <= SITE_TOL_M) & ctx.in_br
                if not k.any():
                    continue
                j = int(np.where(k)[0][np.argmin(prod["cs"][k])])
                n = int(prod["n"][j])
                seg = slice(max(0, j - int(SEG_S * sr)), min(alpha.size, j + int(SEG_S * sr)))
                lag, corr = xcorr_lag(alpha[seg], Fy[seg], sr)
                cs0 = single_cs(alpha, Fy, j, s.mv, s_m, prm, sr, prod["ref"][j])
                entry.append(abs(cs0 - prod["cs"][j]))
                a_rp = np.interp(t - lag, t, alpha)
                cs_rp = single_cs(a_rp, Fy, j, s.mv, s_m, prm, sr, prod["ref"][j])
                w = slice(j - n, j)
                kerb = bool(st["kerb_mask"][w].any()) if st.get("kerb_mask") is not None else False
                rows.append({"lap": lap, "cs": float(prod["cs"][j]), "lag_s": lag, "lag_corr": corr,
                             "cs_repaired": cs_rp, "slip_gap_deg": float(gap[j]), "kerb_in_window": kerb,
                             "az_hp_g": float(np.max(az_hp[w])) if az_hp is not None else float("nan"),
                             "damper_hp_N": float(np.max(dhp[w])) if dhp is not None else float("nan"),
                             "window_m": float(prod["len_m"][j])})
            if not rows:
                continue
            ml = float(np.median([r["lag_s"] for r in rows]))
            mrp = float(np.nanmedian([r["cs_repaired"] for r in rows]))
            mgap = float(np.median([r["slip_gap_deg"] for r in rows]))
            half = len(rows) / 2.0
            b_hits = sum(r["kerb_in_window"] or r["az_hp_g"] > ref_az_p95 or r["damper_hp_N"] > ref_dam_p95 for r in rows)
            flags = []
            if abs(ml - lq[1]) > (lq[2] - lq[0]) and mrp >= bound:
                flags.append("C")
            if mgap > ref_gap_p90:
                flags.append("A")
            if b_hits >= half:
                flags.append("B")
            A["sites"].append({"pos_m": d["pos_m"], "median_cs": d["production"], "median_lag_s": ml,
                               "median_cs_repaired": mrp, "median_slip_gap_deg": mgap, "b_hits": b_hits,
                               "n_laps": len(rows), "flags": flags,
                               "verdict": "+".join(flags) if flags else "unexplained", "laps": rows})
        A["entry_single_cs_max_abs_diff"] = float(np.nanmax(entry)) if entry else float("nan")
        out["axles"][ax] = A
        _log(f"[{key}] {ax}: entry single-sample CS diff {A['entry_single_cs_max_abs_diff']:.2e}; corner lags median "
             f"{lq[1]:+.3f} s IQR {lq[2] - lq[0]:.3f} s (n {len(lags)}); gap p90 {ref_gap_p90:.2f} deg; bound {bound:+.2f}")
        for x in A["sites"]:
            _log(f"    site {x['pos_m']:.0f} m cs {x['median_cs']:+.2f}: lag {x['median_lag_s']:+.3f} s, re-paired "
                 f"{x['median_cs_repaired']:+.2f}, slip gap {x['median_slip_gap_deg']:.2f} deg, B hits {x['b_hits']}/"
                 f"{x['n_laps']} -> {x['verdict']}")
    render_lags(s, out)
    render_site_types(s, out)
    _dump(f"sites_{key}.json", out)


def synth(key):
    s = S(key)
    ctx, st, prm, sr = s.ctx, s.ctx.state, s.ctx.params, s.sr
    s_m, t = st["s_m"], st["time"]
    sp = s.spike["variants"]["production"]
    out = {"key": key, "session": ctx.name, "axles": {}}
    for ax in ("f", "r"):
        alpha, _akin, Fy = s.arrays(ax)
        clean = s.br & s.mv
        for d in sp[ax]["dips"]:
            clean &= ~(np.abs(s_m - d["pos_m"]) <= CLEAN_EXCL_M)
        rows = []
        for tau in TAUS:
            Fd = np.interp(t - tau, t, Fy)
            cs = ica.cs_loop(alpha, Fd, s.mv, s_m, prm, sr, "production")["cs"]
            v = cs[clean & np.isfinite(cs)]
            exc = 0
            for _lap, lm in s.laps:
                b = lm & clean & np.isfinite(cs) & (cs < -1.0)
                exc += int(np.sum(np.diff(np.concatenate(([0], b.astype(np.int8), [0]))) == 1))
            rows.append({"tau_s": tau, "p1": float(np.percentile(v, 1)), "min": float(v.min()),
                         "below_-1_excursions": exc, "n": int(v.size)})
            _log(f"[{key}] {ax} tau {tau:+.2f} s: clean p1 {rows[-1]['p1']:+.3f} min {rows[-1]['min']:+.3f} "
                 f"excursions<-1 {exc}")
        out["axles"][ax] = {"rows": rows,
                            "site_median_depth": float(np.median([d["production"] for d in sp[ax]["dips"]
                                                                  if d["known_real"]] or [np.nan]))}
    render_synth(s, out)
    _dump(f"synth_{key}.json", out)


# ---------------------------------------------------------------- renders

def render_lags(s, out):
    plt = lp._plt()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    for axp, ax in zip(axes, ("f", "r")):
        A = out["axles"][ax]
        lags = np.array(A["global"]["corner_lags_s"]) * 1000
        axp.hist(lags, bins=np.arange(-300, 310, 10), color=REF, label="every corner (whole bracket)")
        for x in A["sites"]:
            for r in x["laps"]:
                axp.axvline(r["lag_s"] * 1000, color=ACC, lw=0.8, alpha=0.6)
        axp.plot([], [], color=ACC, label="each lap at the position-locked dip sites")
        axp.set_xlabel("lag of force behind slip angle (ms)", color=INK)
        axp.set_ylabel("corners", color=INK)
        axp.set_title("front axle" if ax == "f" else "rear axle", fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
    fig.suptitle(lp._wrap(f"{s.ctx.name} -- timing between slip angle and lateral force: every corner vs the "
                          f"position-locked dip sites (cross-correlation, +/-0.3 s)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"lags_{s.ctx.key}.png"), dpi=110)
    plt.close(fig)


def render_site_types(s, out):
    """One render per site type: the deepest kerb-disturbed (B) site and the
    deepest site without B, if present."""
    plt = lp._plt()
    ctx, st = s.ctx, s.ctx.state
    t, s_m = st["time"], st["s_m"]
    az = st.get("az_g")
    allsites = [(ax, x) for ax in out["axles"] for x in out["axles"][ax]["sites"]]
    types = {"kerb-disturbed site (B flagged)": [z for z in allsites if "B" in z[1]["flags"]],
             "site without kerb disturbance": [z for z in allsites if "B" not in z[1]["flags"]]}
    for tname, lst in types.items():
        if not lst:
            continue
        ax, x = min(lst, key=lambda z: z[1]["median_cs"])
        alpha, akin, Fy = s.arrays(ax)
        prod = ica.cs_loop(alpha, Fy, s.mv, s_m, ctx.params, s.sr, "production")["cs"]
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        for lap, lm in s.laps:
            m = lm & (np.abs(s_m - x["pos_m"]) <= 60)
            axes[0].plot(s_m[m], alpha[m] * RAD2DEG, color=INK, lw=0.7, alpha=0.7)
            axes[0].plot(s_m[m], akin[m] * RAD2DEG, color=REF, lw=0.7, ls="--", alpha=0.7)
            axes[1].plot(s_m[m], Fy[m] / 1000, color=INK, lw=0.7, alpha=0.7)
            axes[2].plot(s_m[m], prod[m], color=INK, lw=0.7, alpha=0.7)
            if az is not None:
                axes[3].plot(s_m[m], az[m], color=INK, lw=0.6, alpha=0.7)
            if st.get("kerb_mask") is not None:
                km = m & st["kerb_mask"]
                axes[3].plot(s_m[km], az[km] if az is not None else np.zeros(km.sum()), ".", ms=2, color=ACC2)
        axes[0].plot([], [], color=INK, label="slip angle from the production EKF")
        axes[0].plot([], [], color=REF, ls="--", label="slip angle from the kinematic seed")
        axes[0].legend(frameon=False, fontsize=7)
        axes[0].set_ylabel("slip angle (deg)", color=INK)
        axes[1].set_ylabel("axle lateral force (kN)", color=INK)
        axes[2].set_ylabel("CS ratio", color=INK)
        axes[2].axhline(out["axles"][ax]["global"]["curve_bound_cs_scale"], color=ACC2, lw=1,
                        label="deepest CS the fitted curve allows")
        axes[2].legend(frameon=False, fontsize=7)
        axes[3].set_ylabel("vertical acceleration (g); orange = kerb mask", color=INK)
        axes[3].set_xlabel("lap distance (m)", color=INK)
        for a_ in axes:
            a_.axvline(x["pos_m"], color=ACC, ls=":", lw=1)
            lp._style(a_)
        fig.suptitle(lp._wrap(f"{ctx.name} -- {tname}: {'front' if ax == 'f' else 'rear'} axle at {x['pos_m']:.0f} m, "
                              f"all analysis laps; verdict {x['verdict']} (median lag {x['median_lag_s'] * 1000:+.0f} ms, "
                              f"re-paired CS {x['median_cs_repaired']:+.2f})", fig), fontsize=10, color=INK)
        slug = "kerb" if "B flagged" in tname else "nokerb"
        fig.savefig(os.path.join(OUT_DIR, f"site_{slug}_{ctx.key}.png"), dpi=110)
        plt.close(fig)


def render_synth(s, out):
    plt = lp._plt()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    for axp, ax in zip(axes, ("f", "r")):
        A = out["axles"][ax]
        tau = [r["tau_s"] * 1000 for r in A["rows"]]
        axp.plot(tau, [r["p1"] for r in A["rows"]], "o-", color=INK, label="1st percentile of CS on clean stretches")
        axp.plot(tau, [r["min"] for r in A["rows"]], "s--", color=REF, label="minimum on clean stretches")
        axp.axhline(A["site_median_depth"], color=ACC, lw=1.2, label="median depth of the position-locked dips")
        axp.set_xlabel("imposed delay of the force path behind slip angle (ms)", color=INK)
        axp.set_ylabel("CS ratio", color=INK)
        axp.set_title("front axle" if ax == "f" else "rear axle", fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
    fig.suptitle(lp._wrap(f"{s.ctx.name} -- can a timing mismatch manufacture the dips? CS on clean corner stretches "
                          f"with a known delay imposed between slip angle and lateral force", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"synth_lag_{s.ctx.key}.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    {"sites": sites, "synth": synth}[sys.argv[1]](sys.argv[2])
