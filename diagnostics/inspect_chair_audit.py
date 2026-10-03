# Q-CHAIR-AUDIT -- chair pipeline audit + CS foundation (PLAN.md PHASE 1 PLAN
# Q-CHAIR-AUDIT; governed by section 2b; read-only: nothing in modules/ or
# config/ changes, every variant lives in memory).
#
# RULES, frozen 2026-10-03 BEFORE any run (author rulings 1-6 and EXECUTE,
# PLAN.md Q-CHAIR-AUDIT). Gates escalate, never cancel; renders carry plain-
# language titles. GOVERNING FRAME (section 2b): the recurring deep CS dips are
# POSITION-LOCKED PHENOMENA OF UNPROVEN MAGNITUDE -- never quality anchors,
# never pass / fail; production is the CANDIDATE, not the standard -- "vs
# production" numbers track change only, never quality.
#
# SESSIONS: Tier 1 = pr_us_v2, v3, nbr_jst_solution (JST 29). State, beta and
#   CS chain as inspect_stiffness_cs Part 2 (cached production EKF beta,
#   production parameters). Position-locked sites = the recurring sites of
#   stiffness_cs/spike_<key>.json (production, five deepest dips per axle).
#
# PART A is a reading audit (no code); recorded in thesis_notes.
# PART B (relaxation length) is withdrawn as a filtering ground (ruling 1);
#   Rajamani sec. 13.11 Eqs. 13.86-13.88 recorded as the reason (bandwidth
#   V / (2 pi sigma), no sourced sigma, of order 10-20 Hz > 2 Hz). Segers
#   sec. 2.6 recorded as bounding "filter heavier" (ruling 6).
#
# PART D4-new -- CURVE-SHAPE BOUND (ruling 1, Tier A: Pacejka Magic Formula,
#   Rajamani ch. 13): per session / axle from the production fit (cached
#   manifest B, C, D, E): slope S(a) = dFy/da; origin slope S0 = B C D.
#   BOUND = min over a in [peak, 30 deg] of S(a) / S0 = the deepest slope ratio
#   the fitted model can produce (model-wide); also over [peak, visited p99
#   |alpha|] (data range). CS-scale form (labelled conversion, the CS
#   denominator is the running linear reference, not S0): min S(a) /
#   C_linear_ref, C_linear_ref = median of the production running reference
#   over the session. Per position-locked site: production depth vs both bounds
#   -> "beyond the model bound" or "within" (reported, never pass / fail).
#
# PART C -- FY NOISE DIAGNOSIS
#   C1 Fy_axle = m a_y (l_other / L) +/- I_z psi_dd / L (the chain's 2-DOF
#      balance, Rajamani sec. 2.3 / Milliken ch. 5 as in the module); both
#      terms through the same 2 Hz chain low-pass; inside corner brackets of
#      analysis laps (production base mask): variance of each term and their
#      covariance as shares of var(Fy); and the same in the JITTER BAND (term
#      minus its 0.5 Hz zero-phase low-pass, i.e. 0.5-2 Hz content).
#   C2 DAMPER WITNESS (labelled plausibility witness, NOT validation): damper
#      load gauges log_dms_dam_* (header unit N; NBR offset unknown, shape
#      only), DAMPER SIDE FILTERED FIRST at the same 2 Hz (author practice),
#      per axle the left-minus-right difference (lateral load-transfer shape);
#      correlation of the 0.5-2 Hz band of Fy_axle with the same band of the
#      damper difference, inside corner brackets; sign-free (|r|) and with its
#      sign. Overlay render at each axle's deepest position-locked site.
#   C3 WINDOW CONTENTS at every position-locked site, every analysis lap (the
#      production CS window at the lap's CS minimum within +/-10 m of the
#      site): window length (samples, m), slip span vs the noise floor F_a(n),
#      Fy span vs F_F(n), R^2, psi_dd-term share of the window's Fy variance;
#      STARVING iff slip span < 3 F_a(n) (the V-GRADE rule).
#
# PART D -- THE ADAPTIVE SPEC (author's constructions, definitions accepted)
#   D1 NOISE FLOORS per session and axle, on straight samples (S0 straights of
#      inspect_imu_offsets.populations, production base mask): F_a(n) = median
#      over sliding windows of n samples of the span (max - min) of the chain's
#      filtered slip angle; F_F(n) the same for filtered Fy; n on a geometric
#      grid 10..2000, interpolated. The floor is what noise and straight-line
#      drift alone produce in a window that long (no detrend: conservative).
#   D2 V-GRADE: production CS unchanged; per sample GRADE "insufficient signal"
#      iff the production window's slip span < 3 F_a(n). Nothing deleted.
#      Report: share of in-bracket CS samples graded insufficient, by corner
#      phase (steady mid-corner = apex region expected highest), and the grade
#      at every position-locked site.
#   D3 V-ADAPT: the production window loop, stopping rule replaced: grow
#      (backwards, as production) until slip span >= max(cs_min_slip_angle_
#      span_rad, 3 F_a(n)) AND Fy span >= 3 F_F(n); hard cap 53 m
#      (cs_max_window_m). A window that hits the cap unsatisfied is NOT deleted:
#      its estimate is kept and graded insufficient. Everything else exactly as
#      production (OLS + section blend + R^2 weighting, running linear reference
#      at |alpha| < cs_linear_slip_threshold_rad, ratio clipped at 1).
#      ENTRY CHECK: the same loop with the production stopping rule reproduces
#      production CS exactly. Compared with production and the fixed 0.2 s /
#      0.4 s rows (Q-STIFF-CS) on coverage, lap-to-lap residual (decomposition
#      of Q-STIFF-CS Part 2) and the spike table.
#   D4 V-SURGICAL: DROPPED (ruling 2). Slot = D4-new above.
#   D5 SPIKE TABLE, CORRECTED FRAMING: for production and V-ADAPT, counts below
#      -1 / -2 (recurring vs non-recurring by the Q-STIFF-CS rule) and, per
#      position-locked site, "depth kept + grade" (never pass / fail).
#
# PART E -- 4 s WELCH ROW (ruled GO, LABELLED POST-HOC): inspect_stiffness_cs
#   Part 1b rerun with 2 s -> 4 s Welch windows (0.25 Hz resolution), every
#   other rule unchanged; three-way exit VIABLE / DEGENERATE / CONDITIONAL-ON-
#   I_z; outputs into this folder.
# PART F -- TYRE-MODEL COMPARISON (author order 2026-10-03; frozen before it
#   ran). The chair's fit procedure, from tire_slip_observer_pajecejka_fit.py,
#   reproduced on our data: same reduced Magic Formula; folded data |alpha|,
#   |Fy|; FRONT axle (the chair tool fits the front only; rear run as a
#   labelled extension); four overlapping speed bands, reference speeds
#   linspace(18, 44, 4) m/s, weight = clip(1 - k (v - v_ref)^2, 0, 1) with
#   k = 1 / (mean spacing x 1.05)^2, samples with weight > 0.05; weighted
#   least squares (scipy least_squares, default trust-region, no bounds), start
#   [10, 1.3, max |Fy|, 0.97]. NOT reproducible from the copy, labelled: the
#   filter_config smoothing (filter_config.csv is EMPTY on disk) -> our 2 Hz
#   chain inputs used unsmoothed beyond that; the Fz-weighted slip angle
#   (per-wheel slip function lives in the external vehicle_handler_py) -> axle-
#   mean slip angle used. Population: production base mask, production seed's
#   kinematic alpha / Fy (the population our fit uses). Per band: B, C, D, E,
#   weighted R^2, peak, and the curve-shape bound (deepest post-peak slope /
#   origin slope to 30 deg; CS-scale with the session's running linear
#   reference) against our production fit's bound.
# Outputs diagnostics/plots/chair_audit/.
#
# Usage: python -m diagnostics.inspect_chair_audit run <key> | partE <key> | partF <key>

import copy
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_last_push as lp
from diagnostics import inspect_stiffness_cs as isc

OUT_DIR = "diagnostics/plots/chair_audit"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
RAD2DEG = 180.0 / np.pi
GRADE_K = 3.0
FLOOR_N = np.unique(np.round(np.geomspace(10, 2000, 40)).astype(int))
JITTER_LO_HZ = 0.5
BOUND_MAX_DEG = 30.0
SITE_TOL_M = 10.0
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2


def _dump(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=lp._jsonable)


def _log(m):
    print(m, flush=True)


# ---------------------------------------------------------------- noise floors

def span_floor(x, mask):
    """F(n): median span of x over sliding n-sample windows lying wholly in mask."""
    from numpy.lib.stride_tricks import sliding_window_view
    idx = np.where(mask)[0]
    runs = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1) if idx.size else []
    out = []
    for n in FLOOR_N:
        spans = []
        for r in runs:
            if r.size >= n:
                w = sliding_window_view(x[r], n)[:: max(1, n // 4)]
                spans.append(w.max(axis=1) - w.min(axis=1))
        out.append(float(np.median(np.concatenate(spans))) if spans else float("nan"))
    return np.array(out)


def floor_at(F, n):
    ok = np.isfinite(F)
    return np.interp(n, FLOOR_N[ok], F[ok])


# ---------------------------------------------------------------- CS loop

def cs_loop(alpha, Fy, moving, s_m, params, sr, rule="production", Fa=None, FF=None):
    """Production estimate_cornering_stiffness loop for one axle with the
    stopping rule as a parameter. rule='production' must reproduce production
    exactly (entry check). Returns CS ratio plus per-sample window facts."""
    from modules.stability_analysis import (_find_monotonic_sections, _section_slopes, _smooth_weight,
                                            SPAN_WEIGHT_EXPONENT, R2_WEIGHT_EXPONENT,
                                            resolve_cs_min_window_samples)
    se = params["stability_estimation"]
    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    min_window = resolve_cs_min_window_samples(params, sr)
    max_window_m = se["cs_max_window_m"]
    n = len(alpha)
    CS = np.full(n, np.nan)
    wn = np.full(n, np.nan)
    wspan = np.full(n, np.nan)
    fspan = np.full(n, np.nan)
    wlen_m = np.full(n, np.nan)
    r2 = np.full(n, np.nan)
    capped = np.zeros(n, dtype=bool)
    C_linear_ref = np.nan
    ref_arr = np.full(n, np.nan)
    sections, section_id = _find_monotonic_sections(alpha)
    sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)

    def satisfied(span, fsp, length):
        if rule == "production":
            return span >= min_span
        return span >= max(min_span, GRADE_K * floor_at(Fa, length)) and fsp >= GRADE_K * floor_at(FF, length)

    for i in range(min_window, n):
        if not moving[i]:
            continue
        start = i - min_window
        s_i = s_m[i - 1] if s_m is not None else None
        if s_i is not None and not np.isfinite(s_i):
            s_i = None
        if start > 0:
            amax, amin = np.max(alpha[start:i]), np.min(alpha[start:i])
            fmax, fmin = np.max(Fy[start:i]), np.min(Fy[start:i])
        hit_cap = False
        while start > 0:
            if satisfied(amax - amin, fmax - fmin, i - start):
                break
            if s_i is not None:
                s_start = s_m[start]
                if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                    hit_cap = True
                    break
            start -= 1
            amax, amin = max(amax, alpha[start]), min(amin, alpha[start])
            fmax, fmin = max(fmax, Fy[start]), min(fmin, Fy[start])
        wa, wf = alpha[start:i], Fy[start:i]
        span = float(np.max(wa) - np.min(wa))
        wn[i], wspan[i], fspan[i] = i - start, span, float(np.max(wf) - np.min(wf))
        if s_m is not None and np.isfinite(s_m[start]) and np.isfinite(s_m[i - 1]):
            wlen_m[i] = s_m[i - 1] - s_m[start]
        if rule == "production" and span < min_span:
            continue  # production: span floor not reached -> no signal
        capped[i] = hit_cap and not satisfied(span, fspan[i], i - start)
        am, fm = np.mean(wa), np.mean(wf)
        denom = np.sum((wa - am) ** 2)
        if denom < 1e-10:
            continue
        C_w = np.sum((wa - am) * (wf - fm)) / denom
        fh = C_w * wa + (fm - C_w * am)
        ss_res, ss_tot = np.sum((wf - fh) ** 2), np.sum((wf - fm) ** 2)
        R2_i = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0
        r2[i] = R2_i
        ws, sl = [], []
        for k in np.unique(section_id[start:i]):
            if np.isnan(sec_slopes[k]):
                continue
            w_k = _smooth_weight(sec_spans[k], 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
            if w_k <= 0:
                continue
            ws.append(w_k)
            sl.append(sec_slopes[k])
        if ws:
            C_s = float(np.sum(np.array(ws) * np.array(sl)) / np.sum(ws))
            w_r2 = _smooth_weight(R2_i, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
            C_a = w_r2 * C_w + (1.0 - w_r2) * C_s
        else:
            C_a = C_w
        if np.max(np.abs(wa)) < linear_thresh:
            C_linear_ref = C_a
        if not np.isnan(C_linear_ref) and C_linear_ref > 0:
            CS[i] = min(C_a / C_linear_ref, 1.0)
        ref_arr[i] = C_linear_ref
    return {"cs": CS, "n": wn, "span": wspan, "fspan": fspan, "len_m": wlen_m, "r2": r2, "capped": capped,
            "ref": ref_arr}


# ---------------------------------------------------------------- curve bound

def curve_bound(p, p99_deg):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    B, C, D, E = p["B"], p["C"], p["D"], p["E"]
    s0 = B * C * D
    g = np.radians(np.linspace(0.01, BOUND_MAX_DEG, 3000))
    s = pacejka_lateral_stiffness(g, B, C, D, E)
    i = np.where(np.diff(np.sign(s)) < 0)[0]
    if not i.size:
        return {"peak_deg": float("nan"), "bound_model": float("nan"), "bound_visited": float("nan"),
                "min_slope_model": float("nan"), "min_slope_visited": float("nan"), "S0": s0}
    pk = int(i[0])
    post = s[pk:]
    gv = np.degrees(g[pk:])
    vis = gv <= p99_deg
    return {"peak_deg": float(np.degrees(g[pk])), "S0": s0,
            "min_slope_model": float(post.min()), "bound_model": float(post.min() / s0),
            "min_slope_visited": float(post[vis].min()) if vis.any() else float("nan"),
            "bound_visited": float(post[vis].min() / s0) if vis.any() else float("nan")}


# ---------------------------------------------------------------- run

def run(key):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces, _butterworth_lowpass)
    ctx = lp.Ctx(key)
    st, sr, prm = ctx.state, ctx.state["sample_rate_hz"], ctx.params
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    slip = estimate_slip_angles(st, beta, prm)
    forces = estimate_lateral_forces(st, prm)
    moving = st["moving_mask"].copy()
    if st.get("kerb_mask") is not None:
        moving &= ~st["kerb_mask"]
    s_m = st["s_m"]
    spike = json.load(open(os.path.join(isc.OUT_DIR, f"spike_{key}.json")))
    out = {"key": key, "session": ctx.name, "axles": {}}
    fc = prm["stability_estimation"]["cs_filter_cutoff_hz"]
    # C1 ingredients: the two Fy terms through the same chain low-pass
    # exactly the production split (estimate_lateral_forces): corner-weight
    # fractions, wheelbase_m, non-moving samples zeroed before the low-pass
    vp = prm["vehicle"]
    cw = vp["corner_weights"]
    ff = (cw["FL_kg"] + cw["FR_kg"]) / sum(cw[k] for k in ("FL_kg", "FR_kg", "RL_kg", "RR_kg"))
    psidd = np.gradient(st["yaw_rate_radps"], st["time"])
    m, wb, mv = vp["mass_kg"], vp["wheelbase_m"], st["moving_mask"]
    may, yaw = m * st["ay_mps2"], vp["yaw_inertia_kgm2"] * psidd / wb
    terms = {"f": (np.where(mv, may * ff, 0.0), np.where(mv, yaw, 0.0)),
             "r": (np.where(mv, may * (1 - ff), 0.0), np.where(mv, -yaw, 0.0))}
    br = ctx.base & ctx.in_br
    straight = ctx.base & ctx.s0
    dam = {k: ctx.data["channels"].get(f"log_dms_dam_{k}") for k in ("fl", "fr", "rl", "rr")}
    from modules.tyre_model_pacejka import pacejka_lateral_force  # noqa: F401 (curve_bound import check)
    for ax, a_key, f_key in (("f", "alpha_f_filt", "Fy_f_filt"), ("r", "alpha_r_filt", "Fy_r_filt")):
        alpha, Fy = slip[a_key], forces[f_key]
        A = {}
        # entry check: re-implemented loop with the production rule = production
        prod = cs_loop(alpha, Fy, moving, s_m, prm, sr, "production")
        cached = np.asarray(ctx.cache["prod"]["down"]["cs_" + ax], dtype=float)
        A["entry"] = {"max_abs_diff": float(np.nanmax(np.abs(prod["cs"].astype(np.float32) - cached))),
                      "nan_identical": bool(np.array_equal(np.isnan(prod["cs"]), np.isnan(cached)))}
        # D1 floors
        Fa, FF = span_floor(alpha, straight), span_floor(Fy, straight)
        A["floors"] = {"n": FLOOR_N.tolist(), "slip_span_rad": Fa.tolist(), "fy_span_N": FF.tolist()}
        # D2 V-GRADE
        thin = np.isfinite(prod["cs"]) & (prod["span"] < GRADE_K * floor_at(Fa, np.nan_to_num(prod["n"], nan=10)))
        fin = np.isfinite(prod["cs"]) & br
        A["vgrade"] = {"share_insufficient_in_brackets": float(np.mean(thin[fin])) if fin.any() else float("nan"),
                       "by_phase": phase_shares(ctx, thin, fin)}
        # D3 V-ADAPT
        ad = cs_loop(alpha, Fy, moving, s_m, prm, sr, "adapt", Fa, FF)
        A["vadapt"] = {"coverage_in_brackets": float(np.mean(np.isfinite(ad["cs"][br]))),
                       "coverage_production": float(np.mean(np.isfinite(prod["cs"][br]))),
                       "share_capped_insufficient": float(np.mean(ad["capped"][br & np.isfinite(ad["cs"])])),
                       "median_window_m": float(np.nanmedian(ad["len_m"][br])),
                       "median_window_m_production": float(np.nanmedian(prod["len_m"][br]))}
        g0, g1 = isc.grids(ctx, prod["cs"]), isc.grids(ctx, ad["cs"])
        d0, d1 = isc.decompose(g0), isc.decompose(g1, g0)
        A["vadapt"]["decomposition"] = {"production": d0, "vadapt": d1,
                                        "residual_change": d1["residual_var"] / d0["residual_var"] - 1}
        # D4-new curve-shape bound
        man = ctx.cache["prod"]["manifest"]["axles"]["front" if ax == "f" else "rear"]
        cb = curve_bound(man, man["visited_alpha_p99_deg"])
        cb["C_linear_ref_median"] = float(np.nanmedian(prod["ref"][br]))
        cb["bound_cs_scale_model"] = cb["min_slope_model"] / cb["C_linear_ref_median"]
        cb["bound_cs_scale_visited"] = cb["min_slope_visited"] / cb["C_linear_ref_median"]
        A["curve_bound"] = cb
        # sites: C3 window contents + D2 grade + D5 depth kept
        sites = []
        laps = isc._lap_slices(ctx)
        for d in spike["variants"]["production"][ax]["dips"]:
            pos = d["pos_m"]
            rows = []
            for lap, lm in laps:
                k = lm & np.isfinite(prod["cs"]) & (np.abs(s_m - pos) <= SITE_TOL_M) & ctx.in_br
                if not k.any():
                    continue
                j = np.where(k)[0][np.argmin(prod["cs"][k])]
                n_j = int(prod["n"][j])
                sl = slice(j - n_j, j)
                ta, tb = terms[ax]
                ta_f, tb_f = _butterworth_lowpass(ta, fc, sr)[sl], _butterworth_lowpass(tb, fc, sr)[sl]
                vfy = np.var(ta_f + tb_f)
                fa = floor_at(Fa, n_j)
                rows.append({"lap": lap, "cs": float(prod["cs"][j]), "n": n_j, "len_m": float(prod["len_m"][j]),
                             "slip_span_rad": float(prod["span"][j]), "slip_floor_rad": float(fa),
                             "slip_span_over_floor": float(prod["span"][j] / fa),
                             "fy_span_over_floor": float(prod["fspan"][j] / floor_at(FF, n_j)),
                             "r2": float(prod["r2"][j]),
                             "psidd_share_of_window_fy_var": float(np.var(tb_f) / vfy) if vfy > 0 else float("nan"),
                             "starving": bool(prod["span"][j] < GRADE_K * fa),
                             "vadapt_cs": float(np.nanmin(ad["cs"][k])) if np.isfinite(ad["cs"][k]).any() else float("nan")})
            depth = max(0.0, -d["production"])
            sites.append({"pos_m": pos, "position_locked": d["known_real"], "median_cs": d["production"],
                          "beyond_model_bound": bool(d["production"] < cb["bound_cs_scale_model"]),
                          "beyond_visited_bound": bool(d["production"] < cb["bound_cs_scale_visited"]),
                          "laps": rows,
                          "share_laps_starving": float(np.mean([r["starving"] for r in rows])) if rows else float("nan"),
                          "vadapt_depth_kept": (max(0.0, -np.nanmedian([r["vadapt_cs"] for r in rows])) / depth
                                                if rows and depth > 0 else float("nan"))})
        A["sites"] = sites
        # C1 variance shares and C2 damper witness
        ta, tb = (_butterworth_lowpass(x, fc, sr) for x in terms[ax])
        A["fy_decomposition"] = shares(ta, tb, br, sr)
        A["fy_decomposition"]["sum_check_max_abs_N"] = float(np.max(np.abs(ta + tb - Fy)))
        A["damper_witness"] = damper_witness(st, dam, ax, Fy, br, sr, fc)
        # D5 spike counts (corrected framing)
        A["spikes"] = {"production": spike_counts(ctx, prod["cs"], laps), "vadapt": spike_counts(ctx, ad["cs"], laps)}
        out["axles"][ax] = A
        _log(f"[{key}] {ax}: ENTRY {A['entry']}; insufficient {A['vgrade']['share_insufficient_in_brackets']:.3f} "
             f"by phase {A['vgrade']['by_phase']}; V-ADAPT coverage {A['vadapt']['coverage_in_brackets']:.3f} (prod "
             f"{A['vadapt']['coverage_production']:.3f}) residual {A['vadapt']['decomposition']['residual_change']:+.3f} "
             f"capped {A['vadapt']['share_capped_insufficient']:.3f} window m {A['vadapt']['median_window_m']:.1f} "
             f"(prod {A['vadapt']['median_window_m_production']:.1f}); bound model {cb['bound_model']:+.3f} visited "
             f"{cb['bound_visited']:+.3f} cs-scale {cb['bound_cs_scale_model']:+.3f}/{cb['bound_cs_scale_visited']:+.3f}; "
             f"Fy shares {A['fy_decomposition']}; damper {A['damper_witness']}; spikes {A['spikes']}")
        for s_ in sites:
            _log(f"    site {s_['pos_m']:.0f} m locked {s_['position_locked']} median {s_['median_cs']:+.2f} beyond-bound "
                 f"{s_['beyond_model_bound']}/{s_['beyond_visited_bound']} starving {s_['share_laps_starving']:.2f} "
                 f"V-ADAPT depth kept {s_['vadapt_depth_kept']:.2f}; laps "
                 + "; ".join(f"cs {r['cs']:+.2f} span/floor {r['slip_span_over_floor']:.1f} fy/floor "
                             f"{r['fy_span_over_floor']:.1f} psidd {r['psidd_share_of_window_fy_var']:.2f} "
                             f"{r['len_m']:.0f} m" for r in s_["laps"]))
        out["axles"][ax]["_arrays"] = (prod, ad, thin)
    renders(ctx, out, slip, forces, terms, dam, fc)
    for ax in out["axles"]:
        out["axles"][ax].pop("_arrays")
    _dump(f"audit_{key}.json", out)


def phase_shares(ctx, mark, fin):
    t = ctx.state["time"]
    an = {l["lap_number"] for l in ctx.analysis_laps()}
    res = {}
    half = ctx.params["stability_estimation"]["cs_apex_region_half_length_m"]
    s = ctx.state["s_m"]
    for ph in isc.PHASES:
        m = np.zeros(t.size, dtype=bool)
        for c in ctx.data["corners"]:
            if c["lap_number"] not in an:
                continue
            if ph == "apex_3":
                a = c.get("apex_lap_distance_m")
                t0, t1 = c["segments"]["entry_2_turnin"][0], c["segments"]["exit_5"][1]
                if a is None or a != a:
                    continue
                m |= (t >= t0) & (t <= t1) & (np.abs(s - a) <= half)
            else:
                t0, t1 = c["segments"][ph]
                if t1 > t0:
                    m |= (t >= t0) & (t <= t1)
        k = fin & m
        res[isc.PHASE_WORDS[ph]] = round(float(np.mean(mark[k])), 3) if k.any() else None
    return res


def shares(ta, tb, mask, sr):
    from modules.stability_analysis import _butterworth_lowpass
    out = {}
    for band, (x, y) in (("chain_band", (ta, tb)),
                         ("jitter_band_0.5_2Hz", (ta - _butterworth_lowpass(ta, JITTER_LO_HZ, sr),
                                                  tb - _butterworth_lowpass(tb, JITTER_LO_HZ, sr)))):
        xa, ya = x[mask], y[mask]
        v = np.var(xa + ya)
        out[band] = {"may_share": round(float(np.var(xa) / v), 3), "psidd_share": round(float(np.var(ya) / v), 3),
                     "cov_share": round(float(2 * np.cov(xa, ya)[0, 1] / v), 3)}
    return out


def damper_witness(st, dam, ax, Fy, mask, sr, fc):
    from modules.stability_analysis import _butterworth_lowpass
    l, r = ("fl", "fr") if ax == "f" else ("rl", "rr")
    if dam[l] is None or dam[r] is None:
        return {"status": "damper channel missing"}
    t = st["time"]
    dl = _butterworth_lowpass(np.interp(t, dam[l]["time"], dam[l]["data"]), fc, sr)
    dr = _butterworth_lowpass(np.interp(t, dam[r]["time"], dam[r]["data"]), fc, sr)
    diff = dl - dr
    jb = lambda x: x - _butterworth_lowpass(x, JITTER_LO_HZ, sr)
    a, b = jb(Fy)[mask], jb(diff)[mask]
    rr = float(np.corrcoef(a, b)[0, 1])
    rc = float(np.corrcoef(Fy[mask], diff[mask])[0, 1])
    return {"r_jitter_band": round(rr, 3), "r_chain_band": round(rc, 3)}


def spike_counts(ctx, cs, laps):
    exc = isc.excursions(ctx, cs, laps)
    rec = [r for r in exc[-1.0] if isc._recurs(ctx, cs, r[0], r[1], laps)]
    return {"below_-1": len(exc[-1.0]), "below_-1_position_locked": len(rec),
            "below_-1_non_recurring": len(exc[-1.0]) - len(rec), "below_-2": len(exc[-2.0])}


# ---------------------------------------------------------------- renders

def renders(ctx, out, slip, forces, terms, dam, fc):
    from core.trace_display import kerb_gap_bridge
    from diagnostics.trace_render import cs_panel_range
    plt = lp._plt()
    st = ctx.state
    t, s, kerb = st["time"], st["s_m"], st.get("kerb_mask")
    an = ctx.analysis_laps()
    y_lo, y_hi = cs_panel_range()
    W = max(14, lp._lap_len(ctx) / 400)
    # V-GRADE: production CS with insufficient-signal marks
    fig, axes = plt.subplots(2, 1, figsize=(W, 7.6), sharex=True, constrained_layout=True)
    for axp, a in zip(axes, ("f", "r")):
        prod, ad, thin = out["axles"][a]["_arrays"]
        cs = prod["cs"]
        for b0, b1 in lp._brackets(ctx):
            axp.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        axp.axhline(0, color=REF, lw=0.8)
        axp.axhline(1, color=REF, lw=0.8, ls="--")
        for l in an:
            mm = (t >= l["start_time"]) & (t <= l["end_time"])
            axp.plot(s[mm], cs[mm], color=INK, lw=0.5, alpha=0.55)
            axp.plot(s[mm], kerb_gap_bridge(s[mm], cs[mm], None if kerb is None else kerb[mm]), color=INK, lw=0.5,
                     alpha=0.3, ls="--")
            mk = mm & thin
            axp.plot(s[mk], cs[mk], ".", ms=2.2, color=ACC2, alpha=0.8)
        for site in out["axles"][a]["sites"]:
            if site["position_locked"]:
                axp.axvline(site["pos_m"], color=ACC, lw=0.8, ls=":")
        axp.set_ylim(y_lo, y_hi)
        axp.set_ylabel(("front" if a == "f" else "rear") + " CS ratio", color=INK)
        lp._style(axp)
    axes[1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(lp._wrap(f"{ctx.name} -- production CS ratio with the insufficient-signal grade: orange dots = window "
                          f"slip span below 3x the straight-line noise floor (nothing deleted); dotted blue lines = "
                          f"position-locked dip sites; dashed = kerb gap bridged", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"vgrade_{ctx.key}.png"), dpi=110)
    plt.close(fig)
    # V-ADAPT side by side with production
    p = {a: out["axles"][a]["_arrays"] for a in ("f", "r")}
    isc.render_compare(ctx, os.path.join(OUT_DIR, f"vadapt_compare_{ctx.key}.png"),
                       f"{ctx.name} -- CS ratio, production vs adaptive windows (grow until slip and force variation "
                       f"exceed 3x their noise floors, cap 53 m, nothing deleted)",
                       (p["f"][0]["cs"], p["r"][0]["cs"]), (p["f"][1]["cs"], p["r"][1]["cs"]),
                       "adaptive windows (noise-floor stopping rule)")
    # Curve-shape bound at the sites
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    for axp, a in zip(axes, ("f", "r")):
        A = out["axles"][a]
        cb = A["curve_bound"]
        xs = np.arange(len(A["sites"]))
        for i, site in enumerate(A["sites"]):
            vals = [r["cs"] for r in site["laps"]]
            axp.plot([i] * len(vals), vals, "o", ms=4, color=ACC if site["position_locked"] else INK, alpha=0.6)
            axp.plot(i, site["median_cs"], "_", ms=18, color=INK)
        axp.axhline(cb["bound_cs_scale_model"], color=ACC2, lw=1.4,
                    label=f"deepest CS the fitted curve allows (to 30 deg) {cb['bound_cs_scale_model']:+.2f}")
        axp.axhline(cb["bound_cs_scale_visited"], color=ACC2, lw=1.0, ls="--",
                    label=f"same, within the visited slip range {cb['bound_cs_scale_visited']:+.2f}")
        axp.set_xticks(xs)
        axp.set_xticklabels([f"{s_['pos_m']:.0f} m" for s_ in A["sites"]], fontsize=8)
        axp.set_ylabel("CS ratio at the dip (each lap; bar = cross-lap median)", color=INK)
        axp.set_title(("front" if a == "f" else "rear") + " axle (blue = position-locked site)", fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
    fig.suptitle(lp._wrap(f"{ctx.name} -- deepest CS dips vs the deepest value the session's own fitted tyre curve "
                          f"can produce (curve-shape bound)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"curvebound_{ctx.key}.png"), dpi=110)
    plt.close(fig)
    # Fy composition + damper witness at the deepest position-locked site per axle
    from modules.stability_analysis import _butterworth_lowpass
    sr = st["sample_rate_hz"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7.5), constrained_layout=True)
    for axp, a in zip(axes, ("f", "r")):
        A = out["axles"][a]
        locked = [x for x in A["sites"] if x["position_locked"]] or A["sites"]
        site = min(locked, key=lambda x: x["median_cs"])
        lap = an[len(an) // 2]
        mm = (t >= lap["start_time"]) & (t <= lap["end_time"]) & (np.abs(s - site["pos_m"]) <= 120)
        ta, tb = (_butterworth_lowpass(x, fc, sr) for x in terms[a])
        axp.plot(s[mm], (ta + tb)[mm] / 1000, color=INK, lw=1.4, label="axle lateral force Fy (chain, 2 Hz)")
        axp.plot(s[mm], ta[mm] / 1000, color=ACC, lw=1.0, label="mass x lateral acceleration term")
        axp.plot(s[mm], tb[mm] / 1000, color=ACC2, lw=1.0, label="yaw-acceleration term")
        l_, r_ = ("fl", "fr") if a == "f" else ("rl", "rr")
        if dam[l_] is not None and dam[r_] is not None:
            dd = _butterworth_lowpass(np.interp(t, dam[l_]["time"], dam[l_]["data"]) -
                                      np.interp(t, dam[r_]["time"], dam[r_]["data"]), fc, sr)
            tw = axp.twinx()
            tw.plot(s[mm], dd[mm] / 1000, color=REF, lw=1.0, ls="--", label="damper load left minus right (2 Hz)")
            tw.set_ylabel("damper difference (kN, shape only)", color=REF)
            tw.legend(loc="lower right", frameon=False, fontsize=7)
        axp.axvline(site["pos_m"], color=ACC, ls=":", lw=1)
        axp.set_ylabel("force (kN)", color=INK)
        axp.set_title(f"{'front' if a == 'f' else 'rear'} axle, lap {lap['lap_number']}, around the deepest "
                      f"{'position-locked ' if site['position_locked'] else ''}dip at {site['pos_m']:.0f} m",
                      fontsize=9, loc="left")
        axp.legend(loc="upper left", frameon=False, fontsize=7)
        lp._style(axp)
    axes[1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(lp._wrap(f"{ctx.name} -- what the lateral force is made of at the deepest dip, with the independent "
                          f"damper-load path as a plausibility witness (not a validation)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"fy_witness_{ctx.key}.png"), dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- Part E

def partE(key):
    orig = lp.title_of
    lp.title_of = lambda k: orig(k) + " [post-hoc row: 4 s spectral windows]"
    isc.WELCH_S = 4.0
    isc.OUT_DIR = OUT_DIR
    _log(f"[{key}] PART E (labelled post-hoc): Part 1b with 4 s Welch windows")
    isc.p1b(key)
    src = os.path.join(OUT_DIR, f"p1b_{key}.json")
    dst = os.path.join(OUT_DIR, f"partE_welch4s_{key}.json")
    os.replace(src, dst)
    b = os.path.join(OUT_DIR, f"p1b_bode_{key}.png")
    if os.path.exists(b):
        os.replace(b, os.path.join(OUT_DIR, f"partE_welch4s_bode_{key}.png"))


# ---------------------------------------------------------------- Part F

CHAIR_REF_V = np.linspace(18, 44, 4)
CHAIR_OVERLAP = 0.05
CHAIR_W_MIN = 0.05


def chair_fit(alpha, Fy, v, mask):
    from scipy.optimize import least_squares
    from modules.tyre_model_pacejka import pacejka_lateral_force
    k = 1 / (np.mean(np.diff(CHAIR_REF_V)) * (1 + CHAIR_OVERLAP)) ** 2
    out = []
    for vr in CHAIR_REF_V:
        w = np.clip(1 - k * (v - vr) ** 2, 0, 1)
        m = mask & (w > CHAIR_W_MIN) & np.isfinite(alpha) & np.isfinite(Fy)
        a, f, ww = np.abs(alpha[m]), np.abs(Fy[m]), w[m]
        if a.size < 50:
            out.append({"v_ref": float(vr), "n": int(a.size), "ok": False})
            continue
        res = least_squares(lambda p: ww * (f - pacejka_lateral_force(a, *p)), [10, 1.3, float(np.max(f)), 0.97])
        pred = pacejka_lateral_force(a, *res.x)
        r2 = 1 - np.sum(ww * (f - pred) ** 2) / np.sum(ww * (f - np.average(f, weights=ww)) ** 2)
        B, C, D, E = (float(x) for x in res.x)
        out.append({"v_ref": float(vr), "n": int(a.size), "ok": bool(res.success), "B": B, "C": C, "D": D, "E": E,
                    "r2_weighted": float(r2), "visited_p99_deg": float(np.percentile(np.degrees(a), 99))})
    return out


def partF(key):
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
    from modules.tyre_model_pacejka import pacejka_lateral_force
    ctx = lp.Ctx(key)
    st, prm = ctx.state, ctx.params
    mv = st["moving_mask"] & ~st["kerb_mask"] if st.get("kerb_mask") is not None else st["moving_mask"]
    b0 = estimate_sideslip(st, prm)
    slip = estimate_slip_angles(st, b0, prm)
    fo = estimate_lateral_forces(st, prm)
    slip_e = estimate_slip_angles(st, np.asarray(ctx.cache["prod"]["beta"], dtype=float), prm)
    out = {"key": key, "session": ctx.name, "axles": {}}
    plt = lp._plt()
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.6), constrained_layout=True)
    for axp, ax, a_key, f_key in ((axes[0], "f", "alpha_f_filt", "Fy_f_filt"), (axes[1], "r", "alpha_r_filt", "Fy_r_filt")):
        alpha, Fy = slip[a_key], fo[f_key]
        man = ctx.cache["prod"]["manifest"]["axles"]["front" if ax == "f" else "rear"]
        # CS-scale reference = the production chain's running linear reference, as Part D4-new
        pe = cs_loop(slip_e[a_key], Fy, mv, st["s_m"], prm, st["sample_rate_hz"], "production")
        ref = float(np.nanmedian(pe["ref"][ctx.base & ctx.in_br]))
        ours = curve_bound(man, man["visited_alpha_p99_deg"])
        ours["bound_cs_scale_model"] = ours["min_slope_model"] / ref
        fits = chair_fit(alpha, Fy, st["v_mps"], ctx.base)
        for f in fits:
            if f.get("ok"):
                cb = curve_bound(f, f["visited_p99_deg"])
                f.update({"peak_deg": cb["peak_deg"], "bound_model": cb["bound_model"],
                          "bound_visited": cb["bound_visited"], "bound_cs_scale_model": cb["min_slope_model"] / ref,
                          "origin_slope": cb["S0"]})
        rec_ours = {k: man[k] for k in "BCDE"}
        rec_ours.update({"bound_model": ours["bound_model"], "bound_cs_scale_model": ours["bound_cs_scale_model"],
                         "peak_deg": ours["peak_deg"], "origin_slope": ours["S0"]})
        out["axles"][ax] = {"ours": rec_ours, "chair_bands": fits, "linear_ref": ref,
                            "lineage_note": "front = chair tool scope; rear = labelled extension"}
        m = ctx.base & np.isfinite(alpha)
        a, f_ = np.abs(alpha[m]) * RAD2DEG, np.abs(Fy[m]) / 1000
        if a.size > 40000:
            i = np.random.default_rng(0).choice(a.size, 40000, replace=False)
            a, f_ = a[i], f_[i]
        axp.scatter(a, f_, s=1, color=INK, alpha=0.08, lw=0)
        g = np.linspace(0, float(np.percentile(a, 99.5)) * 1.1, 300)
        axp.plot(g, np.abs(pacejka_lateral_force(np.radians(g), man["B"], man["C"], man["D"], man["E"])) / 1000,
                 color=INK, lw=2.4, label=f"ours: one fit, signed data, Powell (bound {ours['bound_cs_scale_model']:+.2f})")
        for f, c in zip(fits, [ACC, ACC2, "#059669", "#7c3aed"]):
            if f.get("ok"):
                axp.plot(g, pacejka_lateral_force(np.radians(g), f["B"], f["C"], f["D"], f["E"]) / 1000, color=c, lw=1.6,
                         label=f"chair procedure, speed band around {f['v_ref']:.0f} m/s (bound {f['bound_cs_scale_model']:+.2f})")
        axp.set_xlabel("|slip angle| (deg), kinematic seed, 2 Hz chain", color=INK)
        axp.set_ylabel("|axle lateral force| (kN)", color=INK)
        axp.set_title("front axle (the chair tool's own scope)" if ax == "f" else
                      "rear axle (extension, not in the chair tool)", fontsize=9, loc="left")
        axp.legend(frameon=False, fontsize=7)
        lp._style(axp)
        parts = []
        for f in fits:
            if f.get("ok"):
                parts.append(f"{f['v_ref']:.0f} m/s n {f['n']} B {f['B']:.2f} C {f['C']:.3f} D {f['D']:.0f} E {f['E']:.2f} "
                             f"R2w {f['r2_weighted']:.3f} peak {f['peak_deg']:.2f} bound(cs) {f['bound_cs_scale_model']:+.3f}")
            else:
                parts.append(f"{f['v_ref']:.0f} m/s n {f['n']} no fit")
        _log(f"[{key}] PART F {ax}: ours B {man['B']:.2f} C {man['C']:.3f} D {man['D']:.0f} E {man['E']:.2f} peak "
             f"{ours['peak_deg']:.2f} bound(cs) {ours['bound_cs_scale_model']:+.3f}; chair bands " + "; ".join(parts))
    fig.suptitle(lp._wrap(f"{ctx.name} -- tyre curve fitted by our procedure vs the chair's procedure (four speed bands, "
                          f"folded data, weighted least squares; chair smoothing and load-weighted slip angle NOT "
                          f"reproducible from the copy, so omitted); bound = deepest CS each curve can produce", fig),
                 fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"partF_tyre_model_{key}.png"), dpi=110)
    plt.close(fig)
    _dump(f"partF_{key}.json", out)


if __name__ == "__main__":
    {"run": run, "partE": partE, "partF": partF}[sys.argv[1]](sys.argv[2])

