# Q-STIFF-CS -- stiffness identification + CS readability (PLAN.md PHASE 1
# PLAN Q-STIFF-CS; read-only: nothing in modules/ or config/ changes, every
# variant lives in memory).
#
# QUESTIONS: (1) can a second beta-free relation beside the measured
# understeer gradient K2 pin the axle cornering stiffnesses C_f, C_r? (2) what
# makes the CS trace jumpy, and which display / smoothing makes it READABLE?
# Product goal (author, binding): beta trustworthy, CS readable.
#
# RULES, frozen 2026-10-03 BEFORE any run (author rulings 1-4, PLAN.md
# Q-STIFF-CS). Gates order escalation only; every part runs to completion with
# full renders; baselines fresh; the author rules on the renders. Titles in
# plain words.
#
# SESSIONS: Tier 1 = pr_us_v2, v3, nbr_jst_solution (JST 29); all-18 = the
#   serial-221 pool. State, populations, steering offset and K2 exactly as
#   inspect_last_push (Ctx, steering_census, measure_k). Axle stiffness N/rad
#   throughout; m_f = m l_r / L, m_r = m l_f / L; K = m_f / C_f - m_r / C_r.
#
# PART 0 is done in inspect_last_push (render_p1_traces now shares the y-axis);
#   its p1 mode is re-run and the JSONs are compared with the first run.
#
# PART 1a -- steady-state yaw-rate gain per speed band (Rajamani sec. 8.2.3
#   Eq. 8.21: r / delta = v / (L + K v^2)). STATED PRE-RUN: it contains the
#   stiffnesses only through K, so with K2 it forms a RANK-1 system (condition
#   number infinite) -- it is run as an independent K cross-check only.
#   Quasi-steady population, 2 Hz zero-phase signals, steering offset removed;
#   bands of 10 m/s with >= 200 samples; per band G = sum(r delta) /
#   sum(delta^2) (through origin), K_band = (v_med / G - L) / v_med^2;
#   K_gain = weighted least-squares fit of v / (L + K v^2) to the band gains
#   (weights = sample counts); 95 % CIs by corner-traversal cluster bootstrap
#   (B = 200). AGREE iff the K_gain and K2 CIs overlap.
#
# PART 1b -- transient yaw mode (Tier A model: Rajamani sec. 2.3 Eq. 2.31,
#   linear single-track lateral / yaw state space; states beta, r; input
#   road-wheel angle). I_z = config vehicle.yaw_inertia_kgm2 (Level 1, m a b
#   estimate, limitations item 2), rows x0.8 / x1.0 / x1.2.
#   PRIMARY, frequency domain (Tier B: standard spectral system
#   identification): sub-limit samples = racing, moving, v > 20 m/s, |2 Hz
#   a_y| <= 0.5 x racing p95 |a_y|; contiguous runs >= 4 s form segments;
#   per segment raw 100 Hz delta (offset removed) and r, linear detrend; Welch
#   cross / auto spectra, 2 s windows, 50 % overlap, accumulated per speed band
#   (segment median v in [20, 35), [35, 50), [50, 65), [65, 80) m/s; a band
#   needs >= 3 segments). H = S_delta_r / S_delta_delta, coherence gamma^2;
#   points used: 0.5-4 Hz with gamma^2 >= 0.6. Model H(f; C_f, C_r, I_z, v)
#   from Eq. 2.31 at the band's median speed, times exp(-j 2 pi f tau) with a
#   free delay tau in [-0.1, 0.1] s (nuisance: steering and yaw rate come from
#   different front ends). Residual per point: gamma (H_meas - H_model) /
#   |H_meas|. CONSTRAINED fit (primary): C_f = m_f / (K2 + m_r / C_r), free
#   log C_r and tau. UNCONSTRAINED fit (row): free log C_f, log C_r, tau; its
#   implied K is compared with K2.
#   SECONDARY, steering-step events (labelled): inside the sub-limit
#   population, onset where |d delta / dt| (10 Hz low-pass) first exceeds its
#   sub-limit p99; step iff the change over the next 0.3 s is >= 3 x the
#   preceding 0.5 s delta SD and delta over [+0.3, +1.0] s stays within +/-25 %
#   of that change around its new level; events >= 2 s apart, window
#   [-0.5, +1.5] s fully sub-limit. Each event simulated with the measured
#   delta from the steady state of its initial delta (zero-order hold,
#   matrix-exponential discretisation, event mean speed); fit log C_r (K2
#   constraint) and tau to the measured r over all events. Too few: < 5 events
#   -> reported as such.
#   STEP 1 CONDITIONING (decides): condition number of the unconstrained
#   residual Jacobian in (log C_f, log C_r) at its solution; noise_unc = half
#   the 95 % lap-bootstrap (B = 100) interval of the constrained C_r over C_r;
#   iz_unc = max |C_r(I_z x0.8 / x1.2) - C_r| / C_r; total = sqrt(noise^2 +
#   iz^2). THREE-WAY EXIT per session (ruling 3): VIABLE iff total < 25 %;
#   VIABLE-CONDITIONAL-ON-I_z iff noise_unc < 25 % <= total; DEGENERATE AT THIS
#   NOISE iff noise_unc >= 25 % or no band has >= 5 usable points or the fit
#   fails. Route: VIABLE iff >= 2 of 3 sessions VIABLE; CONDITIONAL iff >= 2 of
#   3 are VIABLE or CONDITIONAL; else DEGENERATE.
#   STEP 2 (only if VIABLE or CONDITIONAL): per-session C_f / C_r with CIs,
#   cross-session spread; beta_ss (Rajamani Eq. 8.24, K2) re-rendered with
#   the solved C_r vs the production EKF (shared y); production fits' origin
#   slopes B C D vs the solved C_f / C_r (ratio). Escalation: frequency-domain
#   primary on all 18.
#
# PART 2 -- CS READABILITY on the production chain (cached production EKF
#   beta, entry-checked: fresh CS = cached CS). CS recomputed by the production
#   estimate_cornering_stiffness with the variant parameters.
#   2a DECOMPOSITION per session: each stable corner, 100-point grid between
#     its median bracket start / end (lap distance); per lap CS interpolated
#     with NaN kept (no bridging); SIGNAL = variance over points of the cross-
#     lap median curve; RESIDUAL = mean over points of the across-lap variance
#     (>= 2 laps); pooled over corners weighted by points. Attribution (ordered,
#     not additive): window share = residual reduction of the longest-window
#     row (0.4 s, 0.04 rad); ingredient share = residual reduction of the
#     0.5 Hz ingredient row; both reported against production, plus their
#     combination. Also coverage (finite cells) and retention (correlation of
#     the variant's median curve with production's).
#   2b VARIANTS: windows cs_min_window_s 0.1 / 0.2 / 0.4 s x
#     cs_min_slip_angle_span_rad 0.02 / 0.04 (production = 0.1 / 0.02);
#     ingredients cs_filter_cutoff_hz 2 (production) / 1 / 0.5 Hz (alpha and Fy
#     low-pass, i.e. EKF beta chatter and force noise before CS); displays:
#     PHASE-MEDIAN step / band (summarise_corners' own phase medians per lap,
#     drawn over each phase's lap-distance range; band = p25-p75 across laps,
#     line = cross-lap median), CROSS-LAP BAND (per corner, median and IQR
#     across laps on the 100-point grid, raw laps faint behind); COMBINATION
#     (ruling 4) = the phase-median band on the BEST smoothed-ingredient
#     setting = the 1 / 0.5 Hz row with the largest mean residual reduction
#     among those whose sign pattern holds (below); if none holds, the 1 Hz row.
#   SIGN PATTERN: per (stable corner, phase, axle) the cross-lap median of the
#     phase medians; a variant HOLDS iff <= 10 % of cells finite in both flip
#     sign against production.
#   GATE (escalation only): a variant is PROMISING iff residual -30 % or more
#     on >= 2 of 3 Tier-1 sessions AND sign pattern holds on all 3 AND coverage
#     >= 90 % of production's. Promising -> 2a metrics + comparison renders on
#     all 18.
#   2c RENDERS: production vs each variant side by side (front / rear,
#     kerb-bridged, fixed app y-range), phase-median displays, cross-lap band,
#     decomposition bars.
# SPIKE ADDENDUM (author addition to Part 2, 2026-10-03; definitions as
#   proposed and accepted with "execute"; frozen here before any spike number).
#   Deterministic recomputation of production and the same Part 2 variants
#   (identical parameters, plus the longest-window + 0.5 Hz combination); no
#   new variant. Population: samples inside corner brackets of analysis laps.
#   (a) excursions: runs of consecutive samples below -1.0 (and below -2.0),
#       counted per lap and summed; a NaN or leaving the bracket ends a run.
#   (b) p1 and minimum of CS over the population.
#   (c) five deepest dips: on production's cross-lap median curve (the Part 2
#       corner grids), the five lowest points at least 30 m apart; each
#       variant's value = its own median-curve minimum within +/-10 m of that
#       position. DEPTH = max(0, -CS), i.e. how far beyond the force peak
#       (CS = 0) -- the dips the author weighs; ratio = variant depth /
#       production depth (undefined when production depth is 0).
#   KNOWN-REAL SITE: a production deepest-dip site where at least half of the
#     analysis laps reach, within +/-20 m, a CS at most half the site's median
#     value and below 0 (position-bound recurrence, as the kerb-edge closure
#     characterised apex physics). ERASED iff the variant keeps less than 50 %
#     of the site's depth.
#   NOISE SPIKE: an excursion below -1.0 whose minimum position does NOT recur
#     on at least half of the laps by the same rule (each estimate judged on
#     itself). KILLS NOISE iff the variant's non-recurring count falls by more
#     than 50 % against production.
#   Report per session / axle / variant: variance cut (from Part 2) beside
#   spike survival; flags "erases known-real sites" and "kills noise spikes"
#   kept separate. Spikes are listed first (author: readability weighs spikes
#   first).
# Outputs diagnostics/plots/stiffness_cs/.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_stiffness_cs spike <key>
#   python -m diagnostics.inspect_stiffness_cs p1a|p1b|p2 <key>
#   python -m diagnostics.inspect_stiffness_cs gate
#   python -m diagnostics.inspect_stiffness_cs esc2 <variant> <key>
#   python -m diagnostics.inspect_stiffness_cs step2 <key>

import copy
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/stiffness_cs"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
RAD2DEG = 180.0 / np.pi

BAND_W, BAND_MIN_N, K_BOOT = 10.0, 200, 200
SUB_V_MIN, SEG_MIN_S, WELCH_S = 20.0, 4.0, 2.0
V_BANDS = ((20, 35), (35, 50), (50, 65), (65, 80))
SEG_PER_BAND, F_LO, F_HI, COH_MIN, MIN_POINTS = 3, 0.5, 4.0, 0.6, 5
TAU_MAX = 0.1
IZ_SCALES = (0.8, 1.0, 1.2)
B_BOOT_1B = 100
VIABLE_PCT = 0.25
EV_RATE_Q, EV_RISE_S, EV_PRE_S, EV_HOLD = 99.0, 0.3, 0.5, (0.3, 1.0)
EV_K, EV_TOL, EV_GAP_S, EV_WIN, EV_MIN = 3.0, 0.25, 2.0, (-0.5, 1.5), 5

WINDOWS = ((0.1, 0.02), (0.2, 0.02), (0.4, 0.02), (0.1, 0.04), (0.2, 0.04), (0.4, 0.04))
INGREDIENTS = (2.0, 1.0, 0.5)
N_GRID = 100
RES_GATE, FLIP_MAX, COV_MIN = 0.30, 0.10, 0.90
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
PHASE_WORDS = {"entry_1_brake": "braking", "entry_2_turnin": "turn-in", "apex_3": "apex", "exit_4": "exit",
               "exit_5": "late exit"}

INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2


def _dump(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=lp._jsonable)


def _load(name):
    with open(os.path.join(OUT_DIR, name)) as fh:
        return json.load(fh)


def _log(m):
    print(m, flush=True)


def _plt():
    return lp._plt()


# ---------------------------------------------------------------- P1a

def p1a(key):
    from scipy.optimize import minimize_scalar
    ctx = lp.Ctx(key)
    st, sr = ctx.state, ctx.state["sample_rate_hz"]
    cen = lp.steering_census(ctx)
    off = cen["zero_offset_road_deg"] / RAD2DEG
    k2, stdy = lp.measure_k(ctx, st, off)
    pop = stdy["pop"]
    fc = ctx.params["stability_estimation"]["cs_filter_cutoff_hz"]
    dl = lp._lp(st["delta_f_rad"], fc, sr) - off
    rf, vf = stdy["rf"], lp._lp(st["v_mps"], fc, sr)
    trav = stdy["trav"]
    idx = np.where(pop & np.isfinite(dl) & np.isfinite(rf))[0]
    v_i, d_i, r_i, t_i = vf[idx], dl[idx], rf[idx], trav[idx]
    edges = np.arange(np.floor(v_i.min() / BAND_W) * BAND_W, v_i.max() + BAND_W, BAND_W)

    def bands(sel):
        out = []
        for lo in edges[:-1]:
            m = sel & (v_i >= lo) & (v_i < lo + BAND_W)
            if m.sum() >= BAND_MIN_N:
                G = float(np.sum(r_i[m] * d_i[m]) / np.sum(d_i[m] ** 2))
                vm = float(np.median(v_i[m]))
                out.append((lo, vm, G, int(m.sum())))
        return out

    def fit_k(bs):
        if len(bs) < 1:
            return float("nan")
        v = np.array([b[1] for b in bs]); g = np.array([b[2] for b in bs]); w = np.array([b[3] for b in bs])
        res = minimize_scalar(lambda K: float(np.sum(w * (g - v / (ctx.L + K * v * v)) ** 2)),
                              bounds=(-0.05, 0.05), method="bounded")
        return float(res.x)

    allsel = np.ones(idx.size, dtype=bool)
    b0 = bands(allsel)
    K_gain = fit_k(b0)
    ids = np.unique(t_i)
    rng = np.random.default_rng(20261003)
    ks, kb = [], {b[0]: [] for b in b0}
    by = {i: np.where(t_i == i)[0] for i in ids}
    for _ in range(K_BOOT):
        pick = np.concatenate([by[i] for i in rng.choice(ids, size=ids.size, replace=True)])
        sel = np.zeros(idx.size, dtype=bool)
        cnt = np.bincount(pick, minlength=idx.size)
        sel = cnt > 0
        # duplicated traversals: weight by repeat count via repetition
        rep_idx = np.repeat(np.arange(idx.size), cnt)
        vv, dd, rr = v_i[rep_idx], d_i[rep_idx], r_i[rep_idx]
        bs = []
        for lo in edges[:-1]:
            m = (vv >= lo) & (vv < lo + BAND_W)
            if m.sum() >= BAND_MIN_N:
                G = float(np.sum(rr[m] * dd[m]) / np.sum(dd[m] ** 2))
                vm = float(np.median(vv[m]))
                bs.append((lo, vm, G, int(m.sum())))
                if lo in kb:
                    kb[lo].append((vm / G - ctx.L) / vm ** 2)
        ks.append(fit_k(bs))
    to_dg = RAD2DEG * 9.81
    out = {"key": key, "session": ctx.name, "K2": k2,
           "K_gain_deg_per_g": K_gain * to_dg,
           "K_gain_ci95_deg_per_g": [float(np.nanpercentile(ks, 2.5) * to_dg), float(np.nanpercentile(ks, 97.5) * to_dg)],
           "bands": [{"v_lo": b[0], "v_median": b[1], "gain_per_s": b[2], "n": b[3],
                      "K_band_deg_per_g": (b[1] / b[2] - ctx.L) / b[1] ** 2 * to_dg,
                      "K_band_ci95_deg_per_g": [float(np.percentile(kb[b[0]], 2.5) * to_dg),
                                                float(np.percentile(kb[b[0]], 97.5) * to_dg)] if kb[b[0]] else None}
                     for b in b0],
           "rank_note": "r/delta = v/(L + K v^2) carries the stiffnesses only through K: with K2 the system is rank 1"}
    c1, c2 = out["K_gain_ci95_deg_per_g"], k2["ci95_deg_per_g"]
    out["agree_with_K2"] = bool(c1[0] <= c2[1] and c2[0] <= c1[1])
    _log(f"[{key}] P1a K_gain {out['K_gain_deg_per_g']:+.3f} deg/g CI {[round(x, 3) for x in c1]} vs K2 "
         f"{k2['K_deg_per_g']:+.3f} CI {[round(x, 3) for x in c2]} agree {out['agree_with_K2']}; bands "
         + "; ".join(f"{b['v_lo']:.0f}+ m/s n {b['n']} K {b['K_band_deg_per_g']:+.2f}" for b in out["bands"]))
    render_p1a(ctx, out)
    _dump(f"p1a_{key}.json", out)


def render_p1a(ctx, out):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(8.5, 5), constrained_layout=True)
    k2 = out["K2"]
    ax.axhspan(k2["ci95_deg_per_g"][0], k2["ci95_deg_per_g"][1], color=ACC, alpha=0.15, lw=0,
               label=f"understeer gradient from steering balance (K2) {k2['K_deg_per_g']:+.2f} deg/g, 95 % CI")
    for b in out["bands"]:
        ci = b["K_band_ci95_deg_per_g"]
        ax.errorbar(b["v_median"], b["K_band_deg_per_g"],
                    yerr=None if ci is None else [[b["K_band_deg_per_g"] - ci[0]], [ci[1] - b["K_band_deg_per_g"]]],
                    fmt="o", color=INK, capsize=3)
    ax.axhline(out["K_gain_deg_per_g"], color=ACC2, lw=1.5,
               label=f"from yaw-rate gain over speed {out['K_gain_deg_per_g']:+.2f} deg/g")
    ax.set_xlabel("speed band median (m/s)", color=INK)
    ax.set_ylabel("understeer gradient (deg/g)", color=INK)
    ax.legend(frameon=False, fontsize=8)
    lp._style(ax)
    ax.set_title(lp._wrap(f"{ctx.name} -- understeer gradient from the steady-state yaw-rate gain per speed band "
                          f"(cross-check; same stiffness combination as K2 by construction)", fig), fontsize=9, loc="left")
    fig.savefig(os.path.join(OUT_DIR, f"p1a_yawgain_{ctx.key}.png"), dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- P1b model

def model_matrices(Cf, Cr, m, Iz, a, b, v):
    A = np.array([[-(Cf + Cr) / (m * v), -1.0 + (b * Cr - a * Cf) / (m * v * v)],
                  [(b * Cr - a * Cf) / Iz, -(a * a * Cf + b * b * Cr) / (Iz * v)]])
    B = np.array([Cf / (m * v), a * Cf / Iz])
    return A, B


def h_model(f, Cf, Cr, m, Iz, a, b, v, tau):
    A, B = model_matrices(Cf, Cr, m, Iz, a, b, v)
    out = np.empty(f.size, dtype=complex)
    for i, fi in enumerate(f):
        w = 2 * np.pi * fi
        x = np.linalg.solve(1j * w * np.eye(2) - A, B)
        out[i] = x[1] * np.exp(-1j * w * tau)
    return out


class Spectra:
    """Per speed band: accumulated cross / auto spectra from sub-limit segments."""

    def __init__(self, ctx, off):
        from scipy.signal import csd, welch, detrend
        st, sr = ctx.state, ctx.state["sample_rate_hz"]
        fc = ctx.params["stability_estimation"]["cs_filter_cutoff_hz"]
        ay_lim = 0.5 * float(np.percentile(np.abs(st["ay_mps2"][ctx.racing]), 95))
        sub = ctx.racing & st["moving_mask"] & (st["v_mps"] > SUB_V_MIN) & \
            (np.abs(lp._lp(st["ay_mps2"], fc, sr)) <= ay_lim)
        self.sub, self.sr, self.ay_lim = sub, sr, ay_lim
        e = np.diff(np.concatenate(([0], sub.astype(np.int8), [0])))
        segs = [(a0, a1) for a0, a1 in zip(np.where(e == 1)[0], np.where(e == -1)[0]) if (a1 - a0) / sr >= SEG_MIN_S]
        nper = int(WELCH_S * sr)
        t = st["time"]
        lap_of = np.full(t.size, -1)
        for l in ctx.analysis_laps():
            lap_of[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
        self.segs = []
        for a0, a1 in segs:
            d = detrend(st["delta_f_rad"][a0:a1] - off)
            r = detrend(st["yaw_rate_radps"][a0:a1])
            f, pdd = welch(d, fs=sr, nperseg=nper)
            _, prr = welch(r, fs=sr, nperseg=nper)
            _, pdr = csd(d, r, fs=sr, nperseg=nper)
            nwin = 1 + (a1 - a0 - nper) // (nper // 2)
            self.segs.append({"v": float(np.median(st["v_mps"][a0:a1])), "lap": int(lap_of[a0]), "w": nwin,
                              "pdd": pdd, "prr": prr, "pdr": pdr})
        self.f = f if segs else np.array([])
        self.n_segments = len(self.segs)

    def bands(self, seg_idx=None):
        sel = self.segs if seg_idx is None else [self.segs[i] for i in seg_idx]
        out = []
        for lo, hi in V_BANDS:
            ss_ = [s for s in sel if lo <= s["v"] < hi]
            if len(ss_) < SEG_PER_BAND:
                continue
            w = np.array([s["w"] for s in ss_], dtype=float)
            pdd = sum(s["pdd"] * s["w"] for s in ss_) / w.sum()
            prr = sum(s["prr"] * s["w"] for s in ss_) / w.sum()
            pdr = sum(s["pdr"] * s["w"] for s in ss_) / w.sum()
            H = pdr / pdd
            coh = np.abs(pdr) ** 2 / (pdd * prr)
            m = (self.f >= F_LO) & (self.f <= F_HI) & (coh >= COH_MIN)
            out.append({"band": (lo, hi), "v": float(np.median([s["v"] for s in ss_])), "n_seg": len(ss_),
                        "f": self.f[m], "H": H[m], "coh": coh[m], "f_all": self.f, "H_all": H, "coh_all": coh})
        return out


def fit_freq(bands, ctx, Iz, K=None, x0=None):
    """Constrained (K given) or unconstrained least squares on the coherent
    points; returns the fit dict."""
    from scipy.optimize import least_squares
    usable = [b for b in bands if b["f"].size >= MIN_POINTS]
    if not usable:
        return {"ok": False, "reason": "no band with enough coherent points"}
    m, a, b_ = ctx.m, ctx.lf, ctx.lr

    def unpack(p):
        if K is None:
            return np.exp(p[0]), np.exp(p[1]), p[2]
        Cr = np.exp(p[0])
        den = K + ctx.m_r / Cr
        Cf = ctx.m_f / den if den > 0 else 1e9
        return Cf, Cr, p[1]

    def resid(p):
        Cf, Cr, tau = unpack(p)
        out = []
        for bd in usable:
            Hm = h_model(bd["f"], Cf, Cr, m, Iz, a, b_, bd["v"], tau)
            r = np.sqrt(bd["coh"]) * (bd["H"] - Hm) / np.abs(bd["H"])
            out.append(r.real)
            out.append(r.imag)
        return np.concatenate(out)

    if K is None:
        p0 = x0 if x0 is not None else [np.log(1.5e5), np.log(2.0e5), 0.0]
        lb, ub = [np.log(1e4), np.log(1e4), -TAU_MAX], [np.log(5e6), np.log(5e6), TAU_MAX]
    else:
        p0 = x0 if x0 is not None else [np.log(2.0e5), 0.0]
        lb, ub = [np.log(1e4), -TAU_MAX], [np.log(5e6), TAU_MAX]
    try:
        res = least_squares(resid, p0, bounds=(lb, ub))
    except Exception as e:  # numerical failure counts as degenerate, stated
        return {"ok": False, "reason": f"fit failed: {e}"}
    Cf, Cr, tau = unpack(res.x)
    out = {"ok": bool(res.success), "C_f": float(Cf), "C_r": float(Cr), "tau_s": float(tau),
           "cost": float(res.cost), "n_points": int(sum(bd["f"].size for bd in usable)),
           "bands_used": [bd["band"] for bd in usable], "x": res.x.tolist()}
    if K is None:
        J = res.jac[:, :2]
        sv = np.linalg.svd(J, compute_uv=False)
        out["jacobian_condition"] = float(sv[0] / sv[-1]) if sv[-1] > 0 else float("inf")
        out["K_implied_deg_per_g"] = float((ctx.m_f / Cf - ctx.m_r / Cr) * RAD2DEG * 9.81)
    return out


# ---------------------------------------------------------------- P1b events

def find_events(ctx, spec, off):
    st, sr = ctx.state, spec.sr
    d = st["delta_f_rad"] - off
    dl = lp._lp(d, 10.0, sr)
    rate = np.abs(np.gradient(dl) * sr)
    sub = spec.sub
    thr = float(np.percentile(rate[sub], EV_RATE_Q))
    n_rise, n_pre = int(EV_RISE_S * sr), int(EV_PRE_S * sr)
    h0, h1 = int(EV_HOLD[0] * sr), int(EV_HOLD[1] * sr)
    w0, w1 = int(EV_WIN[0] * sr), int(EV_WIN[1] * sr)
    out, last = [], -10 ** 9
    for i in np.where(sub & (rate > thr))[0]:
        if i - last < EV_GAP_S * sr or i + w0 < 0 or i + w1 >= d.size:
            continue
        if not sub[i + w0:i + w1].all():
            continue
        pre_sd = float(np.std(dl[i - n_pre:i]))
        dd = dl[i + n_rise] - dl[i]
        if abs(dd) < EV_K * max(pre_sd, 1e-6):
            continue
        new = dl[i + n_rise]
        hold = dl[i + h0:i + h1]
        if np.any(np.abs(hold - new) > EV_TOL * abs(dd)):
            continue
        out.append(i)
        last = i
    return out, thr


def sim_r(Cf, Cr, ctx, Iz, v, delta, sr, tau):
    from scipy.linalg import expm
    A, B = model_matrices(Cf, Cr, ctx.m, Iz, ctx.lf, ctx.lr, v)
    dt = 1.0 / sr
    M = expm(np.block([[A, B[:, None]], [np.zeros((1, 3))]]) * dt)
    Ad, Bd = M[:2, :2], M[:2, 2]
    sh = int(round(tau * sr))
    u = np.roll(delta, sh) if sh else delta
    if sh > 0:
        u[:sh] = delta[0]
    elif sh < 0:
        u[sh:] = delta[-1]
    x = -np.linalg.solve(A, B * u[0])
    out = np.empty(u.size)
    for k in range(u.size):
        out[k] = x[1]
        x = Ad @ x + Bd * u[k]
    return out


def fit_events(ctx, events, off, Iz, K):
    from scipy.optimize import least_squares
    st, sr = ctx.state, ctx.state["sample_rate_hz"]
    w0, w1 = int(EV_WIN[0] * sr), int(EV_WIN[1] * sr)
    data = []
    for i in events:
        sl = slice(i + w0, i + w1)
        data.append((float(np.mean(st["v_mps"][sl])), st["delta_f_rad"][sl] - off, st["yaw_rate_radps"][sl]))

    def resid(p):
        Cr = np.exp(p[0])
        den = K + ctx.m_r / Cr
        Cf = ctx.m_f / den if den > 0 else 1e9
        return np.concatenate([sim_r(Cf, Cr, ctx, Iz, v, dl.copy(), sr, p[1]) - r for v, dl, r in data])

    res = least_squares(resid, [np.log(2e5), 0.0], bounds=([np.log(1e4), -TAU_MAX], [np.log(5e6), TAU_MAX]))
    Cr = float(np.exp(res.x[0]))
    return {"C_r": Cr, "C_f": float(ctx.m_f / (K + ctx.m_r / Cr)), "tau_s": float(res.x[1]), "ok": bool(res.success),
            "rms_resid_degps": float(np.sqrt(np.mean(res.fun ** 2)) * RAD2DEG)}


# ---------------------------------------------------------------- P1b

def p1b(key):
    ctx = lp.Ctx(key)
    cen = lp.steering_census(ctx)
    off = cen["zero_offset_road_deg"] / RAD2DEG
    k2, _stdy = lp.measure_k(ctx, ctx.state, off)
    K = k2["K_rad_per_mps2"]
    Iz0 = ctx.params["vehicle"]["yaw_inertia_kgm2"]
    spec = Spectra(ctx, off)
    out = {"key": key, "session": ctx.name, "K2": k2, "Iz_level1": Iz0, "n_segments": spec.n_segments,
           "sub_limit_ay_mps2": spec.ay_lim}
    bands = spec.bands()
    out["bands"] = [{"band": b["band"], "v": b["v"], "n_seg": b["n_seg"], "n_coherent_points": int(b["f"].size)}
                    for b in bands]
    fits = {}
    for s_ in IZ_SCALES:
        fits[s_] = {"constrained": fit_freq(bands, ctx, Iz0 * s_, K=K), "unconstrained": fit_freq(bands, ctx, Iz0 * s_)}
    out["freq"] = {str(s_): v for s_, v in fits.items()}
    c0 = fits[1.0]["constrained"]
    verdict, reason = None, None
    if not c0.get("ok"):
        verdict, reason = "DEGENERATE AT THIS NOISE", c0.get("reason", "fit failed")
    else:
        laps = sorted({s["lap"] for s in spec.segs})
        rng = np.random.default_rng(20261003)
        crs = []
        for _ in range(B_BOOT_1B):
            pick = rng.choice(laps, size=len(laps), replace=True)
            idx = [i for L_ in pick for i, s in enumerate(spec.segs) if s["lap"] == L_]
            fb = fit_freq(spec.bands(idx), ctx, Iz0, K=K, x0=c0["x"])
            if fb.get("ok"):
                crs.append(fb["C_r"])
        out["bootstrap_n_ok"] = len(crs)
        if len(crs) < 0.5 * B_BOOT_1B:
            verdict, reason = "DEGENERATE AT THIS NOISE", f"bootstrap fits succeeded {len(crs)}/{B_BOOT_1B}"
            noise = float("nan")
        else:
            lo_, hi_ = np.percentile(crs, [2.5, 97.5])
            noise = float((hi_ - lo_) / 2 / c0["C_r"])
            out["C_r_ci95"] = [float(lo_), float(hi_)]
        izs = [fits[s_]["constrained"] for s_ in (0.8, 1.2)]
        iz = float(max(abs(f_["C_r"] - c0["C_r"]) for f_ in izs if f_.get("ok")) / c0["C_r"]) \
            if all(f_.get("ok") for f_ in izs) else float("nan")
        tot = float(np.sqrt(noise ** 2 + iz ** 2)) if np.isfinite(noise) and np.isfinite(iz) else float("nan")
        out.update({"noise_unc": noise, "iz_unc": iz, "total_unc": tot})
        if verdict is None:
            if not np.isfinite(noise) or noise >= VIABLE_PCT:
                verdict = "DEGENERATE AT THIS NOISE"
            elif np.isfinite(tot) and tot < VIABLE_PCT:
                verdict = "VIABLE"
            else:
                verdict = "VIABLE-CONDITIONAL-ON-I_z"
    out["verdict"], out["verdict_reason"] = verdict, reason
    # secondary: steering-step events
    ev, thr = find_events(ctx, spec, off)
    out["events"] = {"n": len(ev), "rate_threshold_radps": thr}
    if len(ev) >= EV_MIN:
        out["events"]["fit"] = {str(s_): fit_events(ctx, ev, off, Iz0 * s_, K) for s_ in IZ_SCALES}
    else:
        out["events"]["fit"] = f"too few events ({len(ev)} < {EV_MIN})"
    prod = ctx.cache["prod"]["manifest"]["axles"]
    out["production_origin_slopes"] = {a: prod[a]["B"] * prod[a]["C"] * prod[a]["D"] for a in ("front", "rear")}
    u0 = fits[1.0]["unconstrained"]
    _log(f"[{key}] P1b segments {spec.n_segments}; bands {out['bands']}; constrained C_r "
         f"{c0.get('C_r', float('nan')):.0f} C_f {c0.get('C_f', float('nan')):.0f} tau {c0.get('tau_s', float('nan')):+.3f}; "
         f"unconstrained C_f {u0.get('C_f', float('nan')):.0f} C_r {u0.get('C_r', float('nan')):.0f} cond "
         f"{u0.get('jacobian_condition', float('nan')):.1f} K_impl {u0.get('K_implied_deg_per_g', float('nan')):+.2f}; "
         f"noise {out.get('noise_unc', float('nan')):.3f} iz {out.get('iz_unc', float('nan')):.3f} total "
         f"{out.get('total_unc', float('nan')):.3f} -> {verdict}; events {len(ev)} "
         + (json.dumps(out['events']['fit'], default=float) if isinstance(out['events']['fit'], dict) else out['events']['fit']))
    render_bode(ctx, bands, fits[1.0], Iz0)
    _dump(f"p1b_{key}.json", out)


def render_bode(ctx, bands, fit, Iz):
    plt = _plt()
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True, constrained_layout=True)
    cols = [INK, ACC, ACC2, REF]
    c = fit["constrained"]
    for bd, col in zip(bands, cols):
        lab = f"{bd['band'][0]}-{bd['band'][1]} m/s ({bd['n_seg']} segments)"
        fa = bd["f_all"]
        m = (fa > 0.1) & (fa <= 6)
        axes[0].plot(fa[m], np.abs(bd["H_all"][m]), color=col, lw=0.6, alpha=0.4)
        axes[0].plot(bd["f"], np.abs(bd["H"]), "o", ms=3, color=col, label=lab + ", coherent points")
        axes[1].plot(fa[m], np.degrees(np.angle(bd["H_all"][m])), color=col, lw=0.6, alpha=0.4)
        axes[1].plot(bd["f"], np.degrees(np.angle(bd["H"])), "o", ms=3, color=col)
        axes[2].plot(fa[m], bd["coh_all"][m], color=col, lw=1)
        if c.get("ok"):
            g = np.linspace(0.2, 6, 120)
            Hm = h_model(g, c["C_f"], c["C_r"], ctx.m, Iz, ctx.lf, ctx.lr, bd["v"], c["tau_s"])
            axes[0].plot(g, np.abs(Hm), color=col, lw=1.5)
            axes[1].plot(g, np.degrees(np.angle(Hm)), color=col, lw=1.5)
    axes[2].axhline(COH_MIN, color=REF, ls="--", lw=0.8)
    axes[0].set_ylabel("yaw rate per steer angle (1/s)", color=INK)
    axes[1].set_ylabel("phase (deg)", color=INK)
    axes[2].set_ylabel("coherence", color=INK)
    axes[2].set_xlabel("frequency (Hz)", color=INK)
    for ax in axes:
        ax.set_xscale("log")
        lp._style(ax)
    axes[0].legend(frameon=False, fontsize=7)
    t = (f"{ctx.name} -- yaw-rate response to steering on sub-limit driving, measured (dots = coherent points) vs "
         f"single-track model fitted with the understeer-gradient constraint (lines)")
    if c.get("ok"):
        t += f": rear axle stiffness {c['C_r'] / 1000:.0f} kN/rad, front {c['C_f'] / 1000:.0f} kN/rad"
    fig.suptitle(lp._wrap(t, fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"p1b_bode_{ctx.key}.png"), dpi=110)
    plt.close(fig)


def step2(key):
    """Only if the route is VIABLE or CONDITIONAL: beta_ss with the solved
    stiffness vs the EKF (shared y) and origin-slope comparison."""
    ctx = lp.Ctx(key)
    r = _load(f"p1b_{key}.json")
    c = r["freq"]["1.0"]["constrained"]
    cen = lp.steering_census(ctx)
    off = cen["zero_offset_road_deg"] / RAD2DEG
    K = r["K2"]["K_rad_per_mps2"]
    sr = ctx.state["sample_rate_hz"]
    lo_ = lp._lp(lp.beta_ss(ctx, ctx.state, K, r["C_r_ci95"][0], off), 2.0, sr)
    mid = lp._lp(lp.beta_ss(ctx, ctx.state, K, c["C_r"], off), 2.0, sr)
    hi_ = lp._lp(lp.beta_ss(ctx, ctx.state, K, r["C_r_ci95"][1], off), 2.0, sr)
    bekf = lp._lp(np.asarray(ctx.cache["prod"]["beta"], dtype=float), 2.0, sr)
    from modules.stability_analysis import estimate_sideslip
    b0 = lp._lp(estimate_sideslip(ctx.state, ctx.params), 2.0, sr)
    plt = _plt()
    t, s = ctx.state["time"], ctx.state["s_m"]
    an = ctx.analysis_laps()
    grid = np.arange(0.0, lp._lap_len(ctx), 2.0)

    def med(b):
        rows = []
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"]) & np.isfinite(s)
            o = np.argsort(s[m])
            rows.append(np.interp(grid, s[m][o], b[m][o] * RAD2DEG, left=np.nan, right=np.nan))
        return np.nanmedian(np.array(rows), axis=0)
    fig, ax = plt.subplots(figsize=(max(14, lp._lap_len(ctx) / 400), 4.6), constrained_layout=True)
    for b0_, b1_ in lp._brackets(ctx):
        ax.axvspan(b0_, b1_, color=BAND, lw=0)
    ax.axhline(0, color=REF, lw=0.8)
    l1, l2 = med(lo_), med(hi_)
    ax.fill_between(grid, np.fmin(l1, l2), np.fmax(l1, l2), color=ACC2, alpha=0.2, lw=0,
                    label="steady-state beta, solved rear stiffness 95 % interval")
    ax.plot(grid, med(mid), color=ACC2, lw=1.2, label=f"steady-state beta with SOLVED rear stiffness {c['C_r'] / 1000:.0f} kN/rad")
    ax.plot(grid, med(b0), color=REF, lw=1.1, label="production kinematic seed")
    ax.plot(grid, med(bekf), color=ACC, lw=1.1, label="production EKF")
    ax.set_ylabel("beta (deg), 2 Hz low-pass", color=INK)
    ax.set_xlabel("lap distance (m)", color=INK)
    ax.legend(frameon=False, fontsize=7, ncol=2)
    lp._style(ax)
    fig.suptitle(lp._wrap(f"{ctx.name} -- sideslip beta, cross-lap median: steady state with the identified stiffness vs "
                          f"kinematic seed vs EKF (grey bands = corner brackets)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"step2_beta_{key}.png"), dpi=110)
    plt.close(fig)
    prod = r["production_origin_slopes"]
    out = {"key": key, "C_f": c["C_f"], "C_r": c["C_r"], "C_r_ci95": r["C_r_ci95"],
           "production_over_solved": {"front": prod["front"] / c["C_f"], "rear": prod["rear"] / c["C_r"]}}
    _log(f"[{key}] STEP2 {json.dumps(out, default=float)}")
    _dump(f"step2_{key}.json", out)


# ---------------------------------------------------------------- P2

def cs_run(ctx, beta, pv):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners)
    st = ctx.state
    slip = estimate_slip_angles(st, beta, pv)
    forces = estimate_lateral_forces(st, pv)
    cs = estimate_cornering_stiffness(slip, forces, st, pv)
    stab = estimate_yaw_moment_stability(st, beta, pv, ctx.data.get("laps", []))
    summ = summarise_corners(ctx.data.get("corners", []), cs, stab, st)
    return np.asarray(cs["CS_ratio_f"], dtype=float), np.asarray(cs["CS_ratio_r"], dtype=float), summ


def variant_params(ctx, kind, val):
    pv = copy.deepcopy(ctx.params)
    se = pv["stability_estimation"]
    if kind == "window":
        se["cs_min_window_s"], se["cs_min_slip_angle_span_rad"] = val
    elif kind == "ingredient":
        se["cs_filter_cutoff_hz"] = val
    return pv


def variant_name(kind, val):
    if kind == "window":
        return f"win_{val[0]}s_{val[1]}rad"
    return f"ingr_{val}Hz"


def variant_words(kind, val):
    if kind == "window":
        return f"longer CS window (at least {val[0]} s and {val[1]} rad slip span)"
    return f"CS on smoother ingredients (slip angle and force low-pass {val} Hz instead of 2 Hz)"


def grids(ctx, cs):
    """{sid: lap x 100 array} with NaN kept (gaps stay gaps)."""
    t, s = ctx.state["time"], ctx.state["s_m"]
    br = {}
    for c in ctx.data["corners"]:
        if c.get("stable_corner_id") is not None and c.get("bracket_start_m") is not None:
            br.setdefault(c["stable_corner_id"], []).append((c["bracket_start_m"], c["bracket_end_m"]))
    by = {}
    for sid, lap, sl in ctx.inst:
        by.setdefault(sid, []).append((lap, sl))
    out = {}
    for sid, rows in by.items():
        b0 = float(np.median([x[0] for x in br[sid]]))
        b1 = float(np.median([x[1] for x in br[sid]]))
        g = np.linspace(b0, b1, N_GRID)
        A = []
        for lap, sl in rows:
            ss_, yy = s[sl], cs[sl]
            ok = np.isfinite(ss_)
            if ok.sum() < 5:
                continue
            o = np.argsort(ss_[ok])
            A.append(np.interp(g, ss_[ok][o], yy[ok][o], left=np.nan, right=np.nan))
        if len(A) >= 2:
            out[sid] = (g, np.array(A))
    return out


def decompose(gr, gr_prod=None):
    vs = vr = npts = cells = finite = 0.0
    med_all, medp_all = [], []
    for sid, (g, A) in gr.items():
        cnt = np.sum(np.isfinite(A), axis=0)
        cells += A.size
        finite += np.isfinite(A).sum()
        ok = cnt >= 2
        if ok.sum() < 5:
            continue
        with np.errstate(all="ignore"):
            med = np.nanmedian(A, axis=0)
            var_l = np.nanvar(A, axis=0, ddof=1)
        vs += np.nanvar(med[ok]) * ok.sum()
        vr += np.nanmean(var_l[ok]) * ok.sum()
        npts += ok.sum()
        if gr_prod is not None and sid in gr_prod:
            with np.errstate(all="ignore"):
                mp = np.nanmedian(gr_prod[sid][1], axis=0)
            k = ok & np.isfinite(mp) & np.isfinite(med)
            med_all.append(med[k])
            medp_all.append(mp[k])
    out = {"signal_var": vs / npts if npts else float("nan"), "residual_var": vr / npts if npts else float("nan"),
           "coverage": finite / cells if cells else float("nan")}
    out["signal_share"] = out["signal_var"] / (out["signal_var"] + out["residual_var"])
    if med_all:
        a, b = np.concatenate(med_all), np.concatenate(medp_all)
        out["retention_corr"] = float(np.corrcoef(a, b)[0, 1])
    return out


def phase_table(summ):
    """{(sid, phase, axle): [lap phase medians]} from summarise_corners."""
    tab = {}
    for c in summ:
        sid = c.get("stable_corner_id")
        if sid is None:
            continue
        for ph in PHASES:
            p = c.get("phases", {}).get(ph)
            if not p:
                continue
            for ax, k in (("f", "cs_ratio_f"), ("r", "cs_ratio_r")):
                v = p.get(k, {}).get("median", float("nan"))
                tab.setdefault((sid, ph, ax), []).append((c.get("lap_number"), v))
    return tab


def sign_flip(tab, tab0):
    n = flips = 0
    for k, rows in tab.items():
        if k not in tab0:
            continue
        a = np.nanmedian([v for _l, v in rows]) if any(np.isfinite(v) for _l, v in rows) else np.nan
        b = np.nanmedian([v for _l, v in tab0[k]]) if any(np.isfinite(v) for _l, v in tab0[k]) else np.nan
        if np.isfinite(a) and np.isfinite(b):
            n += 1
            flips += int(np.sign(a) != np.sign(b))
    return {"cells": n, "flips": flips, "flip_share": flips / n if n else float("nan")}


def render_compare(ctx, path, title, cs0, cs1, label1):
    from core.trace_display import kerb_gap_bridge
    from diagnostics.trace_render import cs_panel_range
    plt = _plt()
    y_lo, y_hi = cs_panel_range()
    st = ctx.state
    t, s, kerb = st["time"], st["s_m"], st.get("kerb_mask")
    an = ctx.analysis_laps()
    fig, axes = plt.subplots(4, 1, figsize=(max(14, lp._lap_len(ctx) / 400), 11), sharex=True, constrained_layout=True)
    rows = ((axes[0], cs0[0], "front CS ratio -- production"), (axes[1], cs1[0], f"front CS ratio -- {label1}"),
            (axes[2], cs0[1], "rear CS ratio -- production"), (axes[3], cs1[1], f"rear CS ratio -- {label1}"))
    for ax, arr, lab in rows:
        for b0, b1 in lp._brackets(ctx):
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        ax.axhline(1.0, color=REF, lw=0.8, ls="--")
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s[m], arr[m], color=INK, lw=0.5, alpha=0.6)
            ax.plot(s[m], kerb_gap_bridge(s[m], arr[m], None if kerb is None else kerb[m]), color=INK, lw=0.5,
                    alpha=0.3, ls="--")
        ax.set_ylim(y_lo, y_hi)
        ax.set_title(lab, fontsize=9, loc="left", color=INK)
        lp._style(ax)
    axes[-1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(lp._wrap(f"{title}\nper analysis lap (n={len(an)}); dashed = kerb-excluded stretch bridged for display; "
                          f"0 = force peak, 1 = linear; grey bands = corner brackets", fig), fontsize=10, color=INK)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def phase_ranges(ctx):
    """{(lap, sid, phase): (s0, s1)} on lap distance; apex = apex distance
    +/- cs_apex_region_half_length_m, as summarise_corners reads CS there."""
    t, s = ctx.state["time"], ctx.state["s_m"]
    half = ctx.params["stability_estimation"]["cs_apex_region_half_length_m"]
    out = {}
    an = {l["lap_number"] for l in ctx.analysis_laps()}
    for c in ctx.data["corners"]:
        if c["lap_number"] not in an or c.get("stable_corner_id") is None:
            continue
        for ph in PHASES:
            if ph == "apex_3":
                a = c.get("apex_lap_distance_m")
                if a is not None and a == a:
                    out[(c["lap_number"], c["stable_corner_id"], ph)] = (a - half, a + half)
                continue
            t0, t1 = c["segments"][ph]
            if t1 > t0:
                out[(c["lap_number"], c["stable_corner_id"], ph)] = (float(np.interp(t0, t, s)), float(np.interp(t1, t, s)))
    return out


def render_phase(ctx, path, title, summ, cs_raw):
    plt = _plt()
    rngs = phase_ranges(ctx)
    tab = phase_table(summ)
    st = ctx.state
    t, s = st["time"], st["s_m"]
    an = ctx.analysis_laps()
    fig, axes = plt.subplots(2, 1, figsize=(max(14, lp._lap_len(ctx) / 400), 7.6), sharex=True, constrained_layout=True)
    for ax, axl, raw, lab in ((axes[0], "f", cs_raw[0], "front"), (axes[1], "r", cs_raw[1], "rear")):
        for b0, b1 in lp._brackets(ctx):
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        ax.axhline(1.0, color=REF, lw=0.8, ls="--")
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s[m], raw[m], color=INK, lw=0.4, alpha=0.12)
        for (sid, ph, a_), rows in tab.items():
            if a_ != axl:
                continue
            spans, vals = [], []
            for lap, v in rows:
                r_ = rngs.get((lap, sid, ph))
                if r_ is None or not np.isfinite(v):
                    continue
                spans.append(r_)
                vals.append(v)
                ax.plot(r_, (v, v), color=INK, lw=0.6, alpha=0.5)
            if len(vals) >= 2:
                x0 = float(np.median([r_[0] for r_ in spans])); x1 = float(np.median([r_[1] for r_ in spans]))
                q = np.percentile(vals, [25, 50, 75])
                ax.fill_between((x0, x1), q[0], q[2], color=ACC, alpha=0.25, lw=0)
                ax.plot((x0, x1), (q[1], q[1]), color=ACC, lw=2.2)
        from diagnostics.trace_render import cs_panel_range
        ax.set_ylim(*cs_panel_range())
        ax.set_ylabel(f"{lab} CS ratio", color=INK)
        lp._style(ax)
    axes[1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(lp._wrap(f"{title}\nthick line = cross-lap median of the phase medians (braking / turn-in / apex / exit / "
                          f"late exit), band = their interquartile range across laps, thin steps = each lap's phase "
                          f"median, faint = raw trace", fig), fontsize=10, color=INK)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def render_band(ctx, path, title, gr_f, gr_r, cs_raw):
    plt = _plt()
    st = ctx.state
    t, s = st["time"], st["s_m"]
    an = ctx.analysis_laps()
    fig, axes = plt.subplots(2, 1, figsize=(max(14, lp._lap_len(ctx) / 400), 7.6), sharex=True, constrained_layout=True)
    from diagnostics.trace_render import cs_panel_range
    for ax, gr, raw, lab in ((axes[0], gr_f, cs_raw[0], "front"), (axes[1], gr_r, cs_raw[1], "rear")):
        for b0, b1 in lp._brackets(ctx):
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        ax.axhline(1.0, color=REF, lw=0.8, ls="--")
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s[m], raw[m], color=INK, lw=0.4, alpha=0.12)
        for sid, (g, A) in gr.items():
            with np.errstate(all="ignore"):
                q = np.nanpercentile(A, [25, 50, 75], axis=0)
            ax.fill_between(g, q[0], q[2], color=ACC, alpha=0.25, lw=0)
            ax.plot(g, q[1], color=ACC, lw=1.6)
        ax.set_ylim(*cs_panel_range())
        ax.set_ylabel(f"{lab} CS ratio", color=INK)
        lp._style(ax)
    axes[1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(lp._wrap(f"{title}\nline = cross-lap median per corner position, band = interquartile range across laps, "
                          f"faint = raw per-lap trace", fig), fontsize=10, color=INK)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def p2(key, only=None):
    ctx = lp.Ctx(key)
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    cs_f0, cs_r0, summ0 = cs_run(ctx, beta, ctx.params)
    down = ctx.cache["prod"]["down"]
    out = {"key": key, "session": ctx.name, "entry": {
        "cs_f_max_abs_diff": float(np.nanmax(np.abs(cs_f0.astype(np.float32) - down["cs_f"]))),
        "cs_r_max_abs_diff": float(np.nanmax(np.abs(cs_r0.astype(np.float32) - down["cs_r"]))),
        "nan_pattern_identical": bool(np.array_equal(np.isnan(cs_f0), np.isnan(down["cs_f"])) and
                                      np.array_equal(np.isnan(cs_r0), np.isnan(down["cs_r"])))}}
    _log(f"[{key}] P2 ENTRY {out['entry']}")
    g0 = {"f": grids(ctx, cs_f0), "r": grids(ctx, cs_r0)}
    tab0 = phase_table(summ0)
    out["production"] = {ax: decompose(g0[ax]) for ax in ("f", "r")}
    out["variants"] = {}
    store = {}
    rows = [("window", w) for w in WINDOWS if w != (0.1, 0.02)] + [("ingredient", v) for v in INGREDIENTS if v != 2.0]
    for kind, val in rows:
        name = variant_name(kind, val)
        if only and name not in only:
            continue
        cf, cr, summ = cs_run(ctx, beta, variant_params(ctx, kind, val))
        g = {"f": grids(ctx, cf), "r": grids(ctx, cr)}
        rec = {"words": variant_words(kind, val)}
        for ax in ("f", "r"):
            d = decompose(g[ax], g0[ax])
            d["residual_reduction"] = 1 - d["residual_var"] / out["production"][ax]["residual_var"]
            d["coverage_ratio"] = d["coverage"] / out["production"][ax]["coverage"]
            rec[ax] = d
        rec["sign"] = sign_flip(phase_table(summ), tab0)
        out["variants"][name] = rec
        store[name] = (cf, cr, summ, g)
        render_compare(ctx, os.path.join(OUT_DIR, f"p2_compare_{key}_{name}.png"),
                       f"{ctx.name} -- CS ratio, production vs {variant_words(kind, val)}", (cs_f0, cs_r0), (cf, cr),
                       variant_words(kind, val))
        _log(f"[{key}] P2 {name}: residual -{rec['f']['residual_reduction']:.3f} f / -{rec['r']['residual_reduction']:.3f} r; "
             f"signal share {rec['f']['signal_share']:.3f} / {rec['r']['signal_share']:.3f} (prod "
             f"{out['production']['f']['signal_share']:.3f} / {out['production']['r']['signal_share']:.3f}); coverage ratio "
             f"{rec['f']['coverage_ratio']:.3f} / {rec['r']['coverage_ratio']:.3f}; retention "
             f"{rec['f'].get('retention_corr', float('nan')):.3f} / {rec['r'].get('retention_corr', float('nan')):.3f}; "
             f"sign flips {rec['sign']}")
    # combination row: longest window + heaviest ingredient (attribution overlap)
    pv = variant_params(ctx, "window", (0.4, 0.04))
    pv["stability_estimation"]["cs_filter_cutoff_hz"] = 0.5
    cf, cr, summ = cs_run(ctx, beta, pv)
    g = {"f": grids(ctx, cf), "r": grids(ctx, cr)}
    rec = {"words": "longest window and heaviest ingredient smoothing together (attribution overlap check)"}
    for ax in ("f", "r"):
        d = decompose(g[ax], g0[ax])
        d["residual_reduction"] = 1 - d["residual_var"] / out["production"][ax]["residual_var"]
        d["coverage_ratio"] = d["coverage"] / out["production"][ax]["coverage"]
        rec[ax] = d
    rec["sign"] = sign_flip(phase_table(summ), tab0)
    out["attribution_combined"] = rec
    out["attribution"] = {ax: {"window_share": out["variants"]["win_0.4s_0.04rad"][ax]["residual_reduction"],
                               "ingredient_share": out["variants"]["ingr_0.5Hz"][ax]["residual_reduction"],
                               "combined": rec[ax]["residual_reduction"],
                               "position_locked_signal_share": out["production"][ax]["signal_share"]}
                          for ax in ("f", "r")}
    _log(f"[{key}] P2 attribution {json.dumps(out['attribution'], default=float)}")
    # displays on production
    render_phase(ctx, os.path.join(OUT_DIR, f"p2_phase_{key}_production.png"),
                 f"{ctx.name} -- CS ratio as phase medians per corner (the verdict-level read), production chain",
                 summ0, (cs_f0, cs_r0))
    render_band(ctx, os.path.join(OUT_DIR, f"p2_band_{key}_production.png"),
                f"{ctx.name} -- CS ratio as a cross-lap band per corner, production chain", g0["f"], g0["r"],
                (cs_f0, cs_r0))
    # combination: phase-median band on the best smoothed-ingredient setting
    held = [(n, v) for n, v in out["variants"].items() if n.startswith("ingr_") and v["sign"]["flip_share"] <= FLIP_MAX]
    if held:
        best = max(held, key=lambda x: 0.5 * (x[1]["f"]["residual_reduction"] + x[1]["r"]["residual_reduction"]))[0]
        why = "largest residual reduction with the sign pattern holding (this session)"
    else:
        best, why = "ingr_1.0Hz", "no ingredient row held the sign pattern -> the 1 Hz row (pre-registered fallback)"
    out["combination"] = {"ingredient": best, "why": why}
    cf, cr, summ, g = store[best]
    render_phase(ctx, os.path.join(OUT_DIR, f"p2_phase_{key}_combination.png"),
                 f"{ctx.name} -- CS ratio as phase medians per corner on the smoothest sign-holding ingredient setting "
                 f"({out['variants'][best]['words']})", summ, (cf, cr))
    render_decomp(ctx, out)
    _dump(f"p2_{key}.json", out)


def render_decomp(ctx, out):
    plt = _plt()
    names = ["production"] + list(out["variants"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    for ax, a in zip(axes, ("f", "r")):
        sig = [out["production"][a]["signal_var"]] + [out["variants"][n][a]["signal_var"] for n in names[1:]]
        res = [out["production"][a]["residual_var"]] + [out["variants"][n][a]["residual_var"] for n in names[1:]]
        x = np.arange(len(names))
        ax.bar(x, sig, color=ACC, label="position-locked signal (cross-lap median shape)")
        ax.bar(x, res, bottom=sig, color=REF, label="lap-to-lap residual (jumpiness)")
        ax.set_xticks(x)
        words = ["production"] + [n.replace("win_", "window ").replace("ingr_", "ingredients ").replace("_", " ")
                                  for n in names[1:]]
        ax.set_xticklabels(words, rotation=30, ha="right", fontsize=7)
        ax.set_ylabel("CS ratio variance in corners", color=INK)
        ax.set_title("front axle" if a == "f" else "rear axle", fontsize=9, loc="left")
        lp._style(ax)
    axes[0].legend(frameon=False, fontsize=7)
    fig.suptitle(lp._wrap(f"{ctx.name} -- where the CS jumpiness lives: position-locked signal vs lap-to-lap residual, "
                          f"production and each readability variant", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"p2_decomposition_{ctx.key}.png"), dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- gate

def gate():
    res = {}
    v1 = [_load(f"p1b_{k}.json")["verdict"] for k in TIER1]
    nv = sum(v == "VIABLE" for v in v1)
    nc = sum(v in ("VIABLE", "VIABLE-CONDITIONAL-ON-I_z") for v in v1)
    res["p1b_session_verdicts"] = dict(zip(TIER1, v1))
    res["p1b_route"] = "VIABLE" if nv >= 2 else ("VIABLE-CONDITIONAL-ON-I_z" if nc >= 2 else "DEGENERATE AT THIS NOISE")
    R = {k: _load(f"p2_{k}.json") for k in TIER1}
    res["p2"] = {}
    for name in R[TIER1[0]]["variants"]:
        recs = [R[k]["variants"][name] for k in TIER1]
        red = [max(r["f"]["residual_reduction"], r["r"]["residual_reduction"]) >= RES_GATE for r in recs]
        sign = [r["sign"]["flip_share"] <= FLIP_MAX for r in recs]
        cov = [min(r["f"]["coverage_ratio"], r["r"]["coverage_ratio"]) >= COV_MIN for r in recs]
        res["p2"][name] = {"promising": bool(sum(red) >= 2 and all(sign) and all(cov)), "residual_met": red,
                           "sign_holds": sign, "coverage_ok": cov}
    res["escalate_p2"] = [n for n, e in res["p2"].items() if e["promising"]]
    _dump("gate.json", res)
    _log(json.dumps(res, indent=1))


def esc2(name, key):
    ctx = lp.Ctx(key)
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    kind = "window" if name.startswith("win_") else "ingredient"
    if kind == "window":
        a, b = name[4:].split("s_")
        val = (float(a), float(b[:-3]))
    else:
        val = float(name[5:-2])
    cs_f0, cs_r0, summ0 = cs_run(ctx, beta, ctx.params)
    cf, cr, summ = cs_run(ctx, beta, variant_params(ctx, kind, val))
    g0 = {"f": grids(ctx, cs_f0), "r": grids(ctx, cs_r0)}
    g = {"f": grids(ctx, cf), "r": grids(ctx, cr)}
    rec = {"words": variant_words(kind, val)}
    for ax in ("f", "r"):
        p0 = decompose(g0[ax])
        d = decompose(g[ax], g0[ax])
        d["residual_reduction"] = 1 - d["residual_var"] / p0["residual_var"]
        d["coverage_ratio"] = d["coverage"] / p0["coverage"]
        rec[ax], rec[f"production_{ax}"] = d, p0
    rec["sign"] = sign_flip(phase_table(summ), phase_table(summ0))
    path = os.path.join(OUT_DIR, f"esc_{key}.json")
    prev = json.load(open(path)) if os.path.exists(path) else {}
    prev[name] = rec
    _dump(f"esc_{key}.json", prev)
    render_compare(ctx, os.path.join(OUT_DIR, f"esc_compare_{key}_{name}.png"),
                   f"{ctx.name} -- CS ratio, production vs {variant_words(kind, val)}", (cs_f0, cs_r0), (cf, cr),
                   variant_words(kind, val))
    _log(f"[{key}] ESC {name}: residual -{rec['f']['residual_reduction']:.3f} / -{rec['r']['residual_reduction']:.3f}; "
         f"sign {rec['sign']}; coverage {rec['f']['coverage_ratio']:.3f} / {rec['r']['coverage_ratio']:.3f}")


# ---------------------------------------------------------------- spike addendum

SPIKE_LEVELS = (-1.0, -2.0)
DIP_N, DIP_SEP_M, DIP_TOL_M, SITE_TOL_M = 5, 30.0, 10.0, 20.0


def _lap_slices(ctx):
    t = ctx.state["time"]
    return [(l["lap_number"], (t >= l["start_time"]) & (t <= l["end_time"])) for l in ctx.analysis_laps()]


def _recurs(ctx, cs, pos, value, laps):
    """At least half of the laps reach <= value / 2 (and < 0) within +/-20 m."""
    s = ctx.state["s_m"]
    hits = 0
    for _lap, m in laps:
        k = m & ctx.in_br & np.isfinite(cs) & (np.abs(s - pos) <= SITE_TOL_M)
        if k.any():
            v = float(np.min(cs[k]))
            hits += int(v < 0 and v <= 0.5 * value)
    return hits >= 0.5 * len(laps)


def excursions(ctx, cs, laps):
    s = ctx.state["s_m"]
    out = {}
    for lev in SPIKE_LEVELS:
        runs = []
        for _lap, m in laps:
            below = m & ctx.in_br & np.isfinite(cs) & (cs < lev)
            e = np.diff(np.concatenate(([0], below.astype(np.int8), [0])))
            for a0, a1 in zip(np.where(e == 1)[0], np.where(e == -1)[0]):
                i = a0 + int(np.argmin(cs[a0:a1]))
                runs.append((float(s[i]), float(cs[i])))
        out[lev] = runs
    return out


def deepest_dips(gr):
    pts = []
    for _sid, (g, A) in gr.items():
        with np.errstate(all="ignore"):
            med = np.nanmedian(A, axis=0)
        pts += [(float(x), float(v)) for x, v in zip(g, med) if np.isfinite(v)]
    pts.sort(key=lambda p: p[1])
    chosen = []
    for x, v in pts:
        if all(abs(x - c[0]) >= DIP_SEP_M for c in chosen):
            chosen.append((x, v))
        if len(chosen) == DIP_N:
            break
    return chosen


def _median_min_near(gr, pos):
    best = float("nan")
    for _sid, (g, A) in gr.items():
        k = np.abs(g - pos) <= DIP_TOL_M
        if k.any():
            with np.errstate(all="ignore"):
                v = np.nanmin(np.nanmedian(A[:, k], axis=0))
            if np.isfinite(v) and not (best <= v):
                best = float(v)
    return best


def spike_stats(ctx, cs, gr, laps, dips0, sites_real, base=None):
    pop = ctx.in_br & np.isfinite(cs)
    exc = excursions(ctx, cs, laps)
    noise = [r for r in exc[-1.0] if not _recurs(ctx, cs, r[0], r[1], laps)]
    rec = {"n_below_-1": len(exc[-1.0]), "n_below_-2": len(exc[-2.0]),
           "n_noise_below_-1": len(noise), "n_recurring_below_-1": len(exc[-1.0]) - len(noise),
           "p1": float(np.percentile(cs[pop], 1)) if pop.any() else float("nan"),
           "min": float(np.min(cs[pop])) if pop.any() else float("nan"), "dips": []}
    for (pos, v0), real in zip(dips0, sites_real):
        v = _median_min_near(gr, pos)
        d0, d = max(0.0, -v0), (max(0.0, -v) if np.isfinite(v) else 0.0)
        rec["dips"].append({"pos_m": pos, "production": v0, "variant": v, "known_real": real,
                            "depth_ratio": d / d0 if d0 > 0 else float("nan")})
    if base is not None:
        rec["erases_known_real"] = any(x["known_real"] and np.isfinite(x["depth_ratio"]) and x["depth_ratio"] < 0.5
                                       for x in rec["dips"])
        nb = base["n_noise_below_-1"]
        rec["kills_noise"] = bool(nb > 0 and rec["n_noise_below_-1"] < 0.5 * nb)
        rec["noise_change"] = (rec["n_noise_below_-1"] / nb - 1) if nb else float("nan")
    return rec


def spike(key):
    ctx = lp.Ctx(key)
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    laps = _lap_slices(ctx)
    p2r = _load(f"p2_{key}.json")
    rows = [("production", None, ctx.params)]
    rows += [(variant_name("window", w), ("window", w), variant_params(ctx, "window", w))
             for w in WINDOWS if w != (0.1, 0.02)]
    rows += [(variant_name("ingredient", v), ("ingredient", v), variant_params(ctx, "ingredient", v))
             for v in INGREDIENTS if v != 2.0]
    pc = variant_params(ctx, "window", (0.4, 0.04))
    pc["stability_estimation"]["cs_filter_cutoff_hz"] = 0.5
    rows.append(("combined_win_0.4s_0.04rad_ingr_0.5Hz", None, pc))
    out = {"key": key, "session": ctx.name, "variants": {}}
    base = {}
    for name, _kv, pv in rows:
        cf, cr, _summ = cs_run(ctx, beta, pv)
        rec = {}
        for ax, cs in (("f", cf), ("r", cr)):
            gr = grids(ctx, cs)
            if name == "production":
                dips = deepest_dips(gr)
                real = [_recurs(ctx, cs, x, v, laps) if v < 0 else False for x, v in dips]
                base[ax] = {"dips": dips, "real": real}
                rec[ax] = spike_stats(ctx, cs, gr, laps, dips, real)
                base[ax]["stats"] = rec[ax]
            else:
                rec[ax] = spike_stats(ctx, cs, gr, laps, base[ax]["dips"], base[ax]["real"], base[ax]["stats"])
            if name in p2r["variants"]:
                rec[ax]["variance_cut"] = p2r["variants"][name][ax]["residual_reduction"]
            elif name.startswith("combined"):
                rec[ax]["variance_cut"] = p2r["attribution_combined"][ax]["residual_reduction"]
        out["variants"][name] = rec
        _log(f"[{key}] SPIKE {name}: " + " | ".join(
            f"{ax}: <-1 {rec[ax]['n_below_-1']} (noise {rec[ax]['n_noise_below_-1']}) <-2 {rec[ax]['n_below_-2']} "
            f"p1 {rec[ax]['p1']:+.2f} min {rec[ax]['min']:+.2f} dips "
            + ",".join(f"{d['depth_ratio']:.2f}{'*' if d['known_real'] else ''}" for d in rec[ax]["dips"])
            + (f" erases_real {rec[ax]['erases_known_real']} kills_noise {rec[ax]['kills_noise']} var_cut "
               f"{rec[ax].get('variance_cut', float('nan')):.2f}" if name != "production" else "")
            for ax in ("f", "r")))
    _dump(f"spike_{key}.json", out)
    render_spike(ctx, out)


def render_spike(ctx, out):
    plt = _plt()
    names = list(out["variants"])
    words = ["production"] + [n.replace("combined_", "combined ").replace("win_", "window ").replace("ingr_", "ingredients ")
                              .replace("_", " ") for n in names[1:]]
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    x = np.arange(len(names))
    for j, ax_ in enumerate(("f", "r")):
        a = axes[0, j]
        n1 = [out["variants"][n][ax_]["n_below_-1"] for n in names]
        nn = [out["variants"][n][ax_]["n_noise_below_-1"] for n in names]
        n2 = [out["variants"][n][ax_]["n_below_-2"] for n in names]
        a.bar(x - 0.25, n1, 0.25, color=INK, label="excursions below -1 (all)")
        a.bar(x, nn, 0.25, color=REF, label="of which non-recurring (noise)")
        a.bar(x + 0.25, n2, 0.25, color=ACC2, label="excursions below -2")
        a.set_xticks(x)
        a.set_xticklabels(words, rotation=30, ha="right", fontsize=7)
        a.set_title(("front" if ax_ == "f" else "rear") + " axle: spike counts", fontsize=9, loc="left")
        a.legend(frameon=False, fontsize=7)
        lp._style(a)
        b = axes[1, j]
        for i, n in enumerate(names):
            for d in out["variants"][n][ax_]["dips"]:
                if np.isfinite(d["depth_ratio"]):
                    b.plot(i, d["depth_ratio"], "o" if d["known_real"] else "x", color=ACC if d["known_real"] else INK, ms=6)
        b.axhline(1.0, color=REF, lw=0.8)
        b.axhline(0.5, color=ACC2, lw=0.8, ls="--")
        b.set_xticks(x)
        b.set_xticklabels(words, rotation=30, ha="right", fontsize=7)
        b.set_ylabel("depth kept vs production", color=INK)
        b.set_title(("front" if ax_ == "f" else "rear") + " axle: five deepest production dips\n(o = recurring, "
                    "known-real site; x = not recurring; below dashed line = erased)", fontsize=9, loc="left")
        lp._style(b)
    fig.suptitle(lp._wrap(f"{ctx.name} -- CS spike survival per readability variant: counts of dips beyond the "
                          f"force peak and the fate of the five deepest production dips", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"spike_{ctx.key}.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "spike":
        spike(sys.argv[2])
        sys.exit(0)
    if mode == "spike_render":  # re-render from the saved JSON, no recomputation
        k = sys.argv[2]
        render_spike(lp.Ctx(k), _load(f"spike_{k}.json"))
        sys.exit(0)
    if mode in ("p1a", "p1b", "p2", "step2"):
        globals()[mode](sys.argv[2])
    elif mode == "gate":
        gate()
    elif mode == "esc2":
        esc2(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit(mode)
