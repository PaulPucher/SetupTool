# THE LAST PUSH -- final estimator-side package (PLAN.md PHASE 1 PLAN Q-PUSH;
# read-only: nothing in modules/ or config/ changes, every variant lives in
# memory; production functions are swapped inside modules.tyre_fit_auto only
# for the duration of one chain call and restored in a finally block).
#
# QUESTION: can any genuinely untried route put MORE PHYSICS IN THE PICTURE
# (a visibly cleaner beta, fit cloud or CS trace)? Partial wins count and are
# reported as partial. No curve-truth claims; no courtroom scoring; any
# adoption is its own proposal.
#
# RULES, frozen 2026-10-02 BEFORE any run (author rulings 1-7 of 2026-10-02 in
# PLAN.md Q-PUSH). Standing: gates order ESCALATION only, every part runs to
# completion with full renders, every baseline is measured fresh in this run,
# the author rules on the renders. Render titles in plain words (session,
# quantity, variant).
#
# SESSIONS: Tier 1 = pr_us_v2 (Paul Ricard, MLA outing 32), v3 (Paul Ricard,
#   MLA outing 25), nbr_jst_solution (Nurburgring, JST outing 29). All-18 = the
#   serial-221 pool, inspect_tyre_prior_d2.pool_keys(). State from the (d2)
#   data caches, prepare_vehicle_state with the (d2) params (best available
#   since 2026-10-04: Level-4 manufacturer wheel-angle map, same as the app;
#   runs before that date used cap 1, the constant 15.7).
# ENTRY CHECKS (every session): S0 alpha/Fy/base mask bit-identical to the (d2)
#   fit cache; S0 least-squares refit = cached production manifest (1e-9 rel);
#   P5 scale-1 chain reproduces the cached production gate health and beta.
#
# SHARED DEFINITIONS
#   Populations: racing / S0 straights / corner instances = inspect_imu_offsets
#     .populations; quasi-steady cornering ("mid-corner steady") = the (a2)
#     referee population inspect_imu_offsets._steady (in a corner traversal,
#     |r v| above the session Otsu |a_y| threshold, |psi_dd| < 0.15 rad/s^2,
#     |a_x| < 2 m/s^2 on 2 Hz zero-phase signals, |a_y| < 5 g).
#   Fit population = production _base_mask (moving, off-kerb, analysis laps).
#   M1  Q2d band metric: Fy p95 - p5 and IQR in |alpha| < 0.5 deg, per axle.
#   M1c the same on fit-population samples INSIDE corner brackets only
#       (Q-SEED post-hoc P1, now pre-registered: removes straight dilution).
#       Shrink = 1 - spread_variant / spread_S0 (fresh S0).
#   IQR cloud IQR per alpha bin: fold y = Fy sign(alpha) on |alpha|, 0.5 deg
#       bins on [0, S0 p99 |alpha|], bins with >= 30 samples; report the
#       median over bins and the per-bin list.
#   M3  cross-lap beta coherence as inspect_seed_sweep (100-point corner grid,
#       SD across laps, median over points then corners). Bound = M3c (Q-SEED
#       post-hoc P2, now pre-registered): S0 value + (p97.5 - median) of a
#       200-draw lap bootstrap. DEGRADED iff variant M3 > bound.
#   MECH fit mechanics per axle: Powell converged; start sensitivity = curve
#       RMS between chair-start and prior-start fits of the same loss on
#       [0, p99 |alpha|] as % of D, bar 5.5 % (the derived materiality bar);
#       sanity band (recorded (d) K2): C in (1, 2] and E <= 1 -- screen only.
#   CHATTER RMS of (beta - its 2 Hz zero-phase Butterworth low-pass) on the
#       fit population, deg (limitation 12 quantified); secondary: share of
#       beta PSD power above 2 Hz.
#   GATE HEALTH (chains): nis_gate verdict and health score, fit status,
#       fallback; ONE-WAY -- a worse verdict counts against, better does not
#       count for.
#
# P1 STEERING-DERIVED STEADY-STATE BETA (Tier A: Rajamani 2nd ed. sec. 3.2
#   Eq. 3.23 and sec. 8.2.4 Eqs. 8.20, 8.23, 8.24; verified in the PDF this
#   session).
#   Step 0 census: log_asteer liveness on the racing population, steering zero
#     offset = median road-wheel angle delta_f on S0 straights (removed before
#     use, Tier B), sign check corr(delta_f, r) > 0 on cornering samples.
#   K (understeer gradient, rad per m/s^2):
#     K2 PRIMARY (Level 2, beta-free): OLS of y = lp(delta_f) - offset
#        - L lp(r) / lp(v) on x = lp(a_y) over the quasi-steady population
#        (Eq. 3.23); 95 % CI by corner-traversal cluster bootstrap (B = 200).
#     K1 (Level 1, reported): config's only stiffness source, C_alpha/Fz = 12
#        rad^-1 on both axles (_comment_cs_fallback) -> K1 = m_f / C_f - m_r /
#        C_r = 0: config currently implies a NEUTRAL-STEER car (ruling 2).
#     K_fit (reported): the same expression with the production fits' origin
#        slopes B C D per axle (kinematic-alpha, circular).
#   Rear axle stiffness C_r (ruling 1), two rows, each scaled 0.7 / 1.0 / 1.3:
#     L1 = 12 rad^-1 x m_r g (91.3 kN/rad); L2 = the session's production rear
#     origin slope B C D (Level 2; CIRCULAR -- fitted on kinematic alpha).
#   beta_ss (Eq. 8.24 via 8.20/8.23): 1/R = delta / (L + K v^2);
#     beta_ss = (l_r - m_r v^2 / C_r) / R, m_r = m l_f / L. Reference row
#     (labelled, no steering): Eq. 8.23 with measured curvature 1/R = r / v.
#   Agreement: all estimates low-passed at 2 Hz (the chain's consumption
#     bandwidth); beta_ss - S0 and beta_ss - production EKF (cached production
#     chain beta), median and IQR, in three bins: quasi-steady with |a_y| <=
#     0.5 x racing p95 |a_y| (linear-tyre range, where beta_ss is valid),
#     quasi-steady above it (near the limit: divergence PREDICTED, linear tyre
#     assumption broken), and in-corner non-steady (transients: divergence
#     PREDICTED, steady-state assumption broken).
#   PREDICTION (pre-registered): mid-corner beta_ss ~ l_r r / v - alpha_r,ss is
#     set by the assumed C_r; the +/-30 % rows move it by several degrees,
#     more than the recorded 0.8-1.5 deg beta-candidate separations.
#   ANCHOR EXPERIMENT (ruling 3, run as ordered): S2 segment integration
#     (inspect_seed_sweep.segment_beta: beta = 0 on S0 straights, free
#     integration inside a segment) plus drift-correction knots: one knot at
#     the centre of every quasi-steady run of >= 0.2 s inside the segment,
#     value kappa x mean(beta_ss - integrated beta) over the run; knot 0 at
#     the segment start; end knot = -e at a closed segment end (S2's closure).
#     Correction = linear interpolation through the knots (held flat after the
#     last knot on open segments). Rows: C_r L1 and L2 (K2, scale 1.0) x kappa
#     1.0 and 0.5. Closing-error metric: per-interval |delta correction|
#     between consecutive knots (S2 = one interval per segment, = |e|),
#     median / p90, plus the same per second of interval. "FIXES THE SPREAD"
#     iff p90 falls >= 30 % against fresh S2 on >= 2 of 3 Tier-1 sessions.
#   CIRCULARITY CLAIM (falsifiable, pre-registered): at kappa 1 the anchored
#     population's rear refit origin slope B C D lies closer to the assumed C_r
#     than the S0 refit's does, AND the through-origin slope of Fy_r on
#     alpha_r over the anchor samples is within +/-20 % of C_r -> CONFIRMED on a
#     session; claim CONFIRMED iff on >= 2 of 3 sessions for both C_r rows.
#     K / balance information is P1's primary salvage either way.
#
# P2 DESPIKE + ROBUST FIT (Tier B, standard robust statistics)
#   Despike: Hampel filter on a_y, r and delta_f (and the steering-wheel
#     angle), half-window 0.05 s, outlier iff |x - window median| > 3 x 1.4826
#     x window MAD, replaced by the window median; applied to the state before
#     the seed, so it reaches beta, alpha and Fy (psi_dd from r).
#   Robust losses on the S0 population, Powell from the chair start:
#     Huber, delta = 1.345 x 1.4826 x MAD of the least-squares residuals;
#     Trimmed, least-squares refit after dropping the 10 % largest |residual|,
#     3 iterations, each refit from the chair start.
#   Stated: a robust loss changes the CURVE, not the cloud -- M1/M1c/IQR move
#     only under despike. Residual MAD lower than least squares is favoured by
#     construction, so a robust loss's qualification says "mechanically sound";
#     its real effect shows in the P6 chain.
#
# P3 FIT-POPULATION SWEEP (Tier B), seed S0
#   Rows: all analysis laps (baseline = production fit); FAST = fastest half of
#     the analysis laps by lap time, ceil(n/2), min 2 (primary, ruling 4);
#     FAST101 = laps within 101 % of the session best, run only when >= 2 laps
#     qualify (labelled sensitivity row); FAST + Huber; FAST + Huber + upper
#     envelope = the Q2d construction (fold, 0.2 deg bins, >= 30 samples,
#     q = 0.9) with Pacejka fitted to the bin points under Huber loss.
#   Scope line: Q2d's envelope died on production seed + all laps; this is the
#     untested cell (production seed + fast laps + robust). PREDICTION: the
#     envelope degenerates again unless alpha quality changes (fast laps do
#     not change the seed).
#   One driver per file: a driver-code filter cannot split a session.
#
# P4 SPECTRA-MATCHED INPUT FILTERS (Tier B; evidence Q-RAW)
#   Raw log_acc_y and sclu_acc_y read fresh per session (streaming reader),
#   Welch PSD (4 s segments) over the analysis laps. f_acc = lowest frequency
#   >= 2 Hz from which (log - sclu) PSD exceeds 3 dB on every bin of the next
#   2 Hz. Steering (cached log_asteer, native rate): f_st = lowest frequency
#   >= 1 Hz from which the PSD stays within 3 dB of its top-20 %-band floor
#   for the next 2 Hz. Yaw rate (sclu, filtered on board): no added filter.
#   Filters: zero-phase Butterworth, production order. Lag: sclu_acc_y vs
#   log_acc_y cross-correlation on the 100 Hz grid, +/-0.2 s; aligned row
#   delays log_acc_y by the measured lag (the yaw rate shares the sclu front
#   end).
#   Rows: P4a filters, alpha/Fy low-pass kept at 2 Hz; P4b filters, alpha/Fy
#     low-pass at f_acc instead of the uniform 2 Hz (fit population only, CS
#     chain keeps production filtering, as Q-SEED S4l); P4c = P4a + lag
#     alignment (labelled sub-row, ruling 5).
#   PREDICTION: inert on the beta seed (the integrator is a low-pass already);
#     any effect lands in Fy through psi_dd and in alpha_f through steering.
#
# P5 EKF Q-SWEEP (labelled: SMOOTHNESS, NOT INFORMATION)
#   Production auto chain (fit -> R derivation -> R sweep -> EKF -> NIS gate
#   -> CS), seed S0, with q_beta_var x {0.01, 0.03, 0.1, 0.3, 1, 3} (q_yaw
#   fixed) plus one joint row q_beta and q_yaw x 0.1. Metrics: gate health,
#   chatter, M3 of the chain beta (bound M3c from the scale-1 chain), CS
#   renders. Framing: Q changes how uncertainty is presented, not what the
#   sensors contain -- unless the numbers show otherwise.
#
# ESCALATION GATES (order escalation to the all-18 pool only)
#   Seed / input / population elements (P1 anchored rows, P2 despike, P3 rows,
#     P4 rows): PROMISING iff M1c p95-p5 shrink >= 30 % on at least one axle
#     on >= 2 of 3 Tier-1 sessions AND M3 not degraded on any AND every axle
#     fit converges.
#   Loss elements (P2 Huber / trimmed): PROMISING iff MECH passes (converged,
#     start sensitivity <= 5.5 % of D) on all 6 Tier-1 axle fits AND residual
#     MAD below least squares on both axles on >= 2 of 3 sessions.
#   Envelope row additionally needs the sanity band on all 6 axle fits.
#   Q rows: PROMISING iff chatter falls >= 30 % against the fresh scale-1 chain
#     on >= 2 of 3 sessions AND the gate verdict is not worse on any session
#     AND no fallback.
#   Promising -> the same element on all 18 pool sessions (metrics + renders;
#   Q rows: chains, baseline = the cached production chain, entry-checked on
#   Tier 1).
#
# P6 BEST STACK (ruling 6). Best row per part by its primary metric: highest
#   mean best-axle M1c shrink (P1 anchored, P2 despike, P3 populations, P4),
#   lowest mean residual-MAD ratio (P2 losses), largest mean chatter
#   reduction (P5). QUALIFIED stack = the best PROMISING row of each part
#   (none -> production for that part). EVERYTHING-ON stack = the best row of
#   each part regardless of promise; its population is FAST with the P2 best
#   loss, plus the envelope only if the envelope row passes MECH and the
#   sanity band on all Tier-1 fits. Chain order: despike -> P4 filters/lag ->
#   seed (anchored S2 with beta_ss from the stacked state's own K2) -> fit
#   population -> loss -> EKF Q -> CS. Both stacks run the full chain on
#   Tier 1; CS renders side by side with the fresh production chain.
# RENDERS: CS traces in the production kerb-bridge style (continuous per-lap
#   traces, kerb-excluded gaps bridged dashed and lighter, ruling 7).
# Outputs diagnostics/plots/last_push/.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_last_push p1|p2|p3|p4|p5 <key>
#   python -m diagnostics.inspect_last_push gate
#   python -m diagnostics.inspect_last_push esc <element> <key>
#   python -m diagnostics.inspect_last_push p6 <key>
#   (a "report" mode was listed here at freeze time but never implemented;
#   the summary is in thesis_notes.md -- usage line corrected after the run)

import copy
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_imu_offsets as imu
from diagnostics import inspect_seed_sweep as ss

OUT_DIR = "diagnostics/plots/last_push"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
RAD2DEG = 180.0 / np.pi
G = 9.81

# config states this only in a comment (_comment_cs_fallback, Milliken RCVD
# named there, citation pending), so it cannot be read as a value
C_ALPHA_PER_FZ_L1 = 12.0
STIFF_SCALES = (0.7, 1.0, 1.3)
LINEAR_AY_FRACTION = 0.5
K_BOOT = 200
ANCHOR_MIN_RUN_S = 0.2
KAPPAS = (1.0, 0.5)
CIRC_SLOPE_TOL = 0.20

HAMPEL_HALF_S, HAMPEL_K = 0.05, 3.0
MAD_TO_SD = 1.4826
HUBER_K = 1.345        # Huber's 95 %-efficiency constant
TRIM_FRAC, TRIM_ITERS = 0.10, 3
ENV_BIN_DEG, ENV_MIN_BIN, ENV_Q = 0.2, 30, 0.9   # the Q2d construction
FAST101 = 1.01

PSD_SEG_S = 4.0
PSD_SPLIT_DB, PSD_SUSTAIN_HZ = 3.0, 2.0
LAG_MAX_S = 0.2

Q_ROWS = (("q_beta x0.01", 0.01, 1.0), ("q_beta x0.03", 0.03, 1.0), ("q_beta x0.1", 0.1, 1.0),
          ("q_beta x0.3", 0.3, 1.0), ("production Q", 1.0, 1.0), ("q_beta x3", 3.0, 1.0),
          ("q_beta and q_yaw x0.1", 0.1, 0.1))

IQR_BIN_DEG, IQR_MIN_N = 0.5, 30
SHRINK_GATE, MATERIAL_PCT, CHATTER_GATE = 0.30, 5.5, 0.30
VERDICT_RANK = {"pass": 2, "warn": 1, "fail": 0, None: -1}

INK, BAND, REF, ACC, ACC2, HL = "#1f2937", "#e5e7eb", "#9ca3af", "#2563eb", "#d97706", "#fde68a"
TRACK = {"PRC": "Paul Ricard", "NBR": "Nurburgring"}


def title_of(key):
    s = d2.sessions()[key]
    return f"{TRACK.get(s['track'], s['track'])}, driver {s['driver']}, outing {s['outing']}"


def _dump(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=_jsonable)


def _jsonable(x):
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating, float)):
        return float(x)
    if isinstance(x, np.ndarray):
        return x.tolist()
    return str(x)


def _load(name):
    with open(os.path.join(OUT_DIR, name)) as fh:
        return json.load(fh)


def _log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- context

class Ctx:
    """Everything one session needs, built once per process."""

    def __init__(self, key):
        from modules.stability_analysis import prepare_vehicle_state
        from modules.tyre_fit_auto import _base_mask
        self.key = key
        self.name = title_of(key)
        self.params = d2._params()
        self.data = d2.load_data(key)
        self.state = prepare_vehicle_state(self.data["channels"], self.params)
        self.base = _base_mask(self.state, self.data["laps"])
        # production fit population since the speed window (2026-10-04);
        # self.base stays the metrics population
        from modules.tyre_fit_auto import _speed_window_mask
        self.window, self.window_info = _speed_window_mask(self.state, self.data, self.params["tyre_fit_auto"])
        self.fit_mask = self.base & self.window
        self.racing, self.thr, self.s0, _s1, self.inst = imu.populations(self.state, self.data["laps"],
                                                                         self.data["corners"])
        self.in_br = np.zeros(self.base.size, dtype=bool)
        for _sid, _lap, sl in self.inst:
            self.in_br[sl] = True
        try:
            self.cache = d2.load_fit(key)
        except FileNotFoundError:
            self.cache = None
        vp = self.params["vehicle"]
        self.m = vp["mass_kg"]
        self.lf, self.lr = vp["cog_to_front_axle_m"], vp["cog_to_rear_axle_m"]
        self.L = self.lf + self.lr
        self.m_r = self.m * self.lf / self.L   # Rajamani's rear-axle mass share
        self.m_f = self.m * self.lr / self.L
        self.prior = self.params["tyre_fit_auto"]["tyre_fit_prior"]

    def steady(self, state=None):
        st = self.state if state is None else state
        pseudo = {"sample_rate_hz": st["sample_rate_hz"],
                  "cs_filter_cutoff_hz": self.params["stability_estimation"]["cs_filter_cutoff_hz"],
                  "corners": self.data["corners"], "raw": {}}
        return imu._steady(pseudo, st, self.racing, self.thr, self.inst)

    def analysis_laps(self):
        return [l for l in self.data["laps"] if l.get("is_valid_for_analysis")]

    def lap_mask(self, lap_numbers):
        t = self.state["time"]
        m = np.zeros(t.size, dtype=bool)
        for l in self.analysis_laps():
            if l["lap_number"] in lap_numbers:
                m |= (t >= l["start_time"]) & (t <= l["end_time"])
        return m


def _lp(x, fc, sr):
    from modules.stability_analysis import _butterworth_lowpass
    return _butterworth_lowpass(np.asarray(x, dtype=float), fc, sr)


# ---------------------------------------------------------------- fitting

def _peak(B, C, D, E, p99):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, 89.9, 4000)
    s = pacejka_lateral_stiffness(np.radians(g), B, C, D, E)
    i = np.where(np.diff(np.sign(s)) < 0)[0]
    pk = float(g[i[0]]) if i.size else float("nan")
    return pk, bool(pk == pk and pk <= p99)


def _objective(x, y, loss, delta):
    from modules.tyre_model_pacejka import pacejka_lateral_force

    def j(p):
        r = pacejka_lateral_force(x, *p) - y
        if loss == "huber":
            a = np.abs(r)
            return float(np.sum(np.where(a <= delta, 0.5 * r * r, delta * (a - 0.5 * delta))))
        return float(np.sum(r * r))
    return j


def _powell(x, y, loss, start, delta=None):
    from scipy.optimize import minimize
    res = minimize(_objective(x, y, loss, delta), start, method="Powell")
    return tuple(float(v) for v in res.x), bool(res.success)


def _mad(r):
    return float(MAD_TO_SD * np.median(np.abs(r - np.median(r))))


def fit_axle(alpha, Fy, base_mask, loss="ls", lap_mask=None, envelope=False, start=None):
    """Drop-in for tyre_fit_auto._fit_axle_pacejka (same dict). The plain
    least-squares chair-start call IS the production function, so S0 stays
    bit-identical; the variants change the loss, the start, the lap set or fit
    the Q2d envelope points. Residuals are always on the full population --
    the chain derives R_ay from them."""
    from modules.tyre_fit_auto import _fit_axle_pacejka, PACEJKA_START_GUESS
    from modules.tyre_model_pacejka import pacejka_lateral_force
    m = base_mask if lap_mask is None else (base_mask & lap_mask)
    if loss == "ls" and not envelope and start is None:
        return _fit_axle_pacejka(alpha, Fy, m)
    start = PACEJKA_START_GUESS if start is None else start
    m2 = m & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]
    if a2.size < 50:
        return {"B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
                "powell_converged": False, "fit_n_samples": int(a2.size), "fit_rms_resid_N": float("nan"),
                "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
                "visited_alpha_p99_deg": float("nan"), "sign_ok": False,
                "residuals": np.array([]), "residual_mask": m2, "loss": loss}
    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    if envelope:
        x = np.abs(np.degrees(a2))
        y = f2 * np.sign(a2)
        xs, ys = [], []
        for lo in np.arange(0.0, p99, ENV_BIN_DEG):
            k = (x >= lo) & (x < lo + ENV_BIN_DEG)
            if k.sum() >= ENV_MIN_BIN:
                xs.append(np.radians(lo + ENV_BIN_DEG / 2))
                ys.append(np.quantile(y[k], ENV_Q))
        fx, fy = np.array(xs), np.array(ys)
    else:
        fx, fy = a2, f2
    info = {"loss": loss, "envelope": bool(envelope), "n_fit_points": int(fx.size)}
    if loss == "trim":
        p, ok = _powell(fx, fy, "ls", start)
        keep = np.ones(fx.size, dtype=bool)
        for _ in range(TRIM_ITERS):
            r = np.abs(pacejka_lateral_force(fx, *p) - fy)
            keep = r <= np.quantile(r, 1.0 - TRIM_FRAC)
            p, ok = _powell(fx[keep], fy[keep], "ls", start)
        info["trim_kept"] = int(keep.sum())
    elif loss == "huber":
        p_ls, _ok = _powell(fx, fy, "ls", start)
        delta = HUBER_K * _mad(fy - pacejka_lateral_force(fx, *p_ls))
        p, ok = _powell(fx, fy, "huber", start, delta)
        info["huber_delta_N"] = delta
    else:
        p, ok = _powell(fx, fy, "ls", start)
    B, C, D, E = p
    resid = f2 - pacejka_lateral_force(a2, B, C, D, E)
    pk, inv = _peak(B, C, D, E, p99)
    out = {"B": B, "C": C, "D": D, "E": E, "powell_converged": ok, "fit_n_samples": int(a2.size),
           "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))), "peak_alpha_deg": pk,
           "peak_in_visited_range": inv, "visited_alpha_p99_deg": p99, "sign_ok": D > 0,
           "residuals": resid, "residual_mask": m2}
    out.update(info)
    return out


def make_fitter(loss="ls", lap_mask=None, envelope=False):
    if loss == "ls" and lap_mask is None and not envelope:
        return None
    return lambda a, f, b: fit_axle(a, f, b, loss=loss, lap_mask=lap_mask, envelope=envelope)


def _strip(fit):
    return {k: v for k, v in fit.items() if k not in ("residuals", "residual_mask")}


def mechanics(alpha, Fy, base_mask, fit, ctx, axle, loss="ls", lap_mask=None, envelope=False):
    """Converged, start sensitivity (chair vs prior start, same loss), band."""
    from modules.tyre_model_pacejka import pacejka_lateral_force
    pr = ctx.prior[axle]
    alt = fit_axle(alpha, Fy, base_mask, loss=loss, lap_mask=lap_mask, envelope=envelope,
                   start=(pr["B"], pr["C"], pr["D"], pr["E"]))
    p99 = fit["visited_alpha_p99_deg"]
    out = {"converged": bool(fit["powell_converged"]), "prior_start_converged": bool(alt["powell_converged"]),
           "band_ok": bool(1.0 < fit["C"] <= 2.0 and fit["E"] <= 1.0) if np.isfinite(fit["C"]) else False}
    if np.isfinite(p99) and np.isfinite(fit["D"]) and fit["D"] > 0 and np.isfinite(alt["D"]):
        g = np.radians(np.linspace(0.0, p99, 200))
        f1 = pacejka_lateral_force(g, fit["B"], fit["C"], fit["D"], fit["E"])
        f2 = pacejka_lateral_force(g, alt["B"], alt["C"], alt["D"], alt["E"])
        out["start_sens_pct_D"] = float(100 * np.sqrt(np.mean((f1 - f2) ** 2)) / fit["D"])
    else:
        out["start_sens_pct_D"] = float("nan")
    out["start_ok"] = bool(out["start_sens_pct_D"] <= MATERIAL_PCT) if np.isfinite(out["start_sens_pct_D"]) else False
    rm = fit.get("residuals")
    out["resid_mad_N"] = _mad(rm) if rm is not None and len(rm) else float("nan")
    out["origin_slope_N_per_rad"] = float(fit["B"] * fit["C"] * fit["D"]) if np.isfinite(fit["B"]) else float("nan")
    return out


# ---------------------------------------------------------------- metrics

def m1(alpha, fy, mask):
    return ss.m1(alpha, fy, mask)


def cloud_iqr(alpha, fy, mask, p99_s0):
    m = mask & np.isfinite(alpha) & np.isfinite(fy)
    x = np.abs(alpha[m]) * RAD2DEG
    y = fy[m] * np.sign(alpha[m])
    bins = []
    for lo in np.arange(0.0, p99_s0, IQR_BIN_DEG):
        k = (x >= lo) & (x < lo + IQR_BIN_DEG)
        if k.sum() >= IQR_MIN_N:
            q = np.percentile(y[k], [25, 75])
            bins.append((float(lo + IQR_BIN_DEG / 2), float(q[1] - q[0]), int(k.sum())))
    return {"median_iqr_N": float(np.median([b[1] for b in bins])) if bins else float("nan"), "bins": bins}


def m3_pair(beta, ctx):
    curves = ss.corner_curves(beta, ctx.state, ctx.data, ctx.inst)
    v, n = ss.m3_value(curves)
    return v, n, curves


def m3c_bound(curves):
    v0, _n = ss.m3_value(curves)
    laps = sorted({l for cv in curves.values() for l in cv})
    if len(laps) < 2:
        return float("nan")
    rng = np.random.default_rng(20261002)
    bs = []
    for _ in range(ss.N_BOOT_M3):
        v, _n = ss.m3_value(curves, list(rng.choice(laps, size=len(laps), replace=True)))
        if np.isfinite(v):
            bs.append(v)
    return float(v0 + np.percentile(bs, 97.5) - np.median(bs)) if bs else float("nan")


def chatter(beta, mask, sr):
    from scipy.signal import welch
    b = np.asarray(beta, dtype=float)
    hf = b - _lp(b, 2.0, sr)
    rms = float(np.sqrt(np.mean(hf[mask] ** 2)) * RAD2DEG)
    f, p = welch(b, fs=sr, nperseg=int(4 * sr))
    share = float(p[f > 2.0].sum() / p.sum()) if p.sum() > 0 else float("nan")
    return {"chatter_rms_deg": rms, "psd_share_above_2hz": share}


def evaluate(ctx, tag, label, state, beta, slip_params, fit_mask_laps=None, loss="ls", envelope=False,
             s0_rec=None, render=True, beta_ref=None):
    """Fit-population evaluation of one variant: alpha/Fy from (state, beta),
    M1 / M1c / IQR on the (optionally lap-restricted) fit population, refit
    with the variant's loss, mechanics, M3 of beta, renders."""
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
    slip = estimate_slip_angles(state, beta, slip_params)
    forces = estimate_lateral_forces(state, slip_params)
    pop = ctx.base if fit_mask_laps is None else (ctx.base & fit_mask_laps)
    rec = {"label": label}
    v, n, curves = m3_pair(beta, ctx)
    rec["m3"], rec["m3_n_corners"] = v, n
    if s0_rec is None:
        rec["m3c_bound"] = m3c_bound(curves)
    fits = {}
    for axle, a, f in (("front", slip["alpha_f_filt"], forces["Fy_f_filt"]),
                       ("rear", slip["alpha_r_filt"], forces["Fy_r_filt"])):
        ar = {"m1": m1(a, f, pop), "m1c": m1(a, f, pop & ctx.in_br)}
        p99_s0 = s0_rec[axle]["refit"]["visited_alpha_p99_deg"] if s0_rec else None
        fit = fit_axle(a, f, ctx.base, loss=loss, lap_mask=fit_mask_laps, envelope=envelope)
        p99_grid = p99_s0 if p99_s0 is not None else fit["visited_alpha_p99_deg"]
        ar["iqr"] = cloud_iqr(a, f, pop, p99_grid)
        ar["refit"] = _strip(fit)
        ar["mech"] = mechanics(a, f, ctx.base, fit, ctx, axle, loss=loss, lap_mask=fit_mask_laps, envelope=envelope)
        if s0_rec is not None:
            b0 = s0_rec[axle]
            for k in ("m1", "m1c"):
                ar[f"{k}_shrink"] = 1 - ar[k]["p95_p5_N"] / b0[k]["p95_p5_N"] if "p95_p5_N" in ar[k] else float("nan")
            ar["iqr_ratio"] = ar["iqr"]["median_iqr_N"] / b0["iqr"]["median_iqr_N"]
            ar["curve_vs_s0"] = ss.curve_delta(fit, b0["refit"], b0["refit"]["visited_alpha_p99_deg"])
            ar["resid_mad_ratio"] = ar["mech"]["resid_mad_N"] / b0["mech"]["resid_mad_N"]
        rec[axle] = ar
        fits[axle] = (a, f, fit)
    if s0_rec is not None:
        rec["m3_degraded"] = bool(rec["m3"] > s0_rec["m3c_bound"]) if np.isfinite(s0_rec["m3c_bound"]) else None
        rec["best_axle_m1c_shrink"] = max(rec["front"]["m1c_shrink"], rec["rear"]["m1c_shrink"])
    if render:
        for axle, (a, f, fit) in fits.items():
            curves_ = []
            if s0_rec is not None:
                curves_.append(("production seed, least squares, all laps (fresh)", s0_rec[axle]["refit"], REF, "--"))
            curves_.append((f"{label} refit", fit, ACC, "-"))
            render_cloud(os.path.join(OUT_DIR, f"{tag}_cloud_{ctx.key}_{axle}.png"),
                         f"{ctx.name} -- {axle} tyre-fit cloud, {label}", a, f, pop, curves_, rec[axle])
        rows = [("production kinematic seed (fresh)", beta_ref, REF)] if beta_ref is not None else []
        rows.append((label, beta, INK))
        render_beta(os.path.join(OUT_DIR, f"{tag}_beta_{ctx.key}.png"),
                    f"{ctx.name} -- sideslip beta per analysis lap, {label}", ctx, rows)
    return rec


# ---------------------------------------------------------------- renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _brackets(ctx):
    an = {l["lap_number"] for l in ctx.analysis_laps()}
    return sorted({(c["bracket_start_m"], c["bracket_end_m"]) for c in ctx.data["corners"]
                   if c.get("bracket_start_m") is not None and c["lap_number"] in an})


def _lap_len(ctx):
    l = ctx.analysis_laps()[0]
    t = ctx.state["time"]
    return float(np.nanmax(ctx.state["s_m"][(t >= l["start_time"]) & (t <= l["end_time"])]))


def _wrap(text, fig):
    # long plain-language titles must stay inside the image (render title rule)
    import textwrap
    width = max(60, int(fig.get_figwidth() * 11))
    return "\n".join(textwrap.fill(part, width) for part in text.split("\n"))


def _style(ax):
    ax.grid(axis="y", color=BAND, lw=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


def render_beta(path, title, ctx, rows):
    """rows: (label, beta, colour); one panel per row, all analysis laps."""
    plt = _plt()
    t, s = ctx.state["time"], ctx.state["s_m"]
    an = ctx.analysis_laps()
    win = (t >= an[0]["start_time"]) & (t <= an[-1]["end_time"])
    vals = np.concatenate([np.asarray(b)[win] for _l, b, _c in rows]) * RAD2DEG
    lim = max(2.0, float(np.nanpercentile(np.abs(vals), 99.9)) * 1.1)
    fig, axes = plt.subplots(len(rows), 1, figsize=(max(14, _lap_len(ctx) / 400), 2.9 * len(rows) + 0.6),
                             sharex=True, squeeze=False, constrained_layout=True)
    for ax, (lab, b, col) in zip(axes[:, 0], rows):
        for b0, b1 in _brackets(ctx):
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s[m], np.asarray(b)[m] * RAD2DEG, color=col, lw=0.5, alpha=0.7)
        ax.set_ylim(-lim, lim)
        ax.set_ylabel("beta (deg)", color=INK)
        ax.set_title(lab, fontsize=9, color=INK, loc="left")
        _style(ax)
    axes[-1, 0].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(_wrap(f"{title} (grey bands = corner brackets)", fig), fontsize=10, color=INK)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def render_cloud(path, title, alpha, fy, mask, curves, met):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    plt = _plt()
    m = mask & np.isfinite(alpha) & np.isfinite(fy)
    a, f = alpha[m] * RAD2DEG, fy[m] / 1000.0
    if a.size > 40000:
        i = np.random.default_rng(0).choice(a.size, 40000, replace=False)
        a, f = a[i], f[i]
    fig, ax = plt.subplots(figsize=(8.5, 5.8), constrained_layout=True)
    ax.axvspan(-ss.BAND_DEG, ss.BAND_DEG, color=HL, lw=0, zorder=0, label="near-zero slip band |alpha| < 0.5 deg")
    ax.scatter(a, f, s=1.2, color=INK, alpha=0.12, lw=0, zorder=1)
    lim = float(np.nanpercentile(np.abs(a), 99.5)) * 1.1 if a.size else 5.0
    g = np.linspace(-lim, lim, 400)
    for lab, p, col, ls in curves:
        if p is not None and np.isfinite(p["D"]):
            ax.plot(g, pacejka_lateral_force(np.radians(g), p["B"], p["C"], p["D"], p["E"]) / 1000.0,
                    color=col, lw=2, ls=ls, zorder=3, label=lab)
    ax.set_xlim(-lim, lim)
    ax.set_xlabel("slip angle alpha, filtered (deg)", color=INK)
    ax.set_ylabel("axle lateral force Fy, filtered (kN)", color=INK)
    sub = (f"band spread p95-p5 {met['m1'].get('p95_p5_N', float('nan')) / 1000:.1f} kN (in corners "
           f"{met['m1c'].get('p95_p5_N', float('nan')) / 1000:.1f} kN), median per-bin IQR "
           f"{met['iqr']['median_iqr_N'] / 1000:.2f} kN")
    ax.set_title(_wrap(f"{title}\n{sub}", fig), fontsize=9, color=INK, loc="left")
    ax.grid(color=BAND, lw=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(loc="lower right", frameon=False, fontsize=7)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def render_curves(path, title, alpha, fy, mask, curves):
    met = {"m1": m1(alpha, fy, mask), "m1c": {}, "iqr": {"median_iqr_N": float("nan")}}
    render_cloud(path, title, alpha, fy, mask, curves, met)


def render_cs_bridged(path, title, ctx, state, cs_f, cs_r):
    """CS traces in the production kerb-bridge style: per-lap continuous
    traces, NaN runs that contain kerb samples bridged dashed and lighter
    (core.trace_display.kerb_gap_bridge, the app's own helper)."""
    from core.trace_display import kerb_gap_bridge
    from diagnostics.trace_render import cs_panel_range
    plt = _plt()
    y_lo, y_hi = cs_panel_range()
    t, s, kerb = state["time"], state["s_m"], state.get("kerb_mask")
    an = ctx.analysis_laps()
    fig, axes = plt.subplots(2, 1, figsize=(max(14, _lap_len(ctx) / 400), 7.4), sharex=True, constrained_layout=True)
    for ax, arr, lab in ((axes[0], cs_f, "front CS ratio"), (axes[1], cs_r, "rear CS ratio")):
        arr = np.asarray(arr, dtype=float)
        for b0, b1 in _brackets(ctx):
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        ax.axhline(1.0, color=REF, lw=0.8, ls="--")
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s[m], arr[m], color=INK, lw=0.5, alpha=0.6, zorder=2)
            br = kerb_gap_bridge(s[m], arr[m], None if kerb is None else kerb[m])
            ax.plot(s[m], br, color=INK, lw=0.5, alpha=0.3, ls="--", zorder=2)
        ax.set_ylim(y_lo, y_hi)
        ax.set_ylabel(lab, color=INK)
        _style(ax)
    axes[1].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(_wrap(f"{title}\nper analysis lap (n={len(an)}); dashed = kerb-excluded stretch bridged for display; "
                       f"0 = force peak, 1 = linear; grey bands = corner brackets", fig), color=INK, fontsize=10)
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- chains

def run_chain(ctx, state, seed_beta, fitter=None, q=(1.0, 1.0), slip_params=None):
    """Production auto chain (fit -> R -> EKF -> gate) with in-memory swaps
    inside modules.tyre_fit_auto, then the production downstream to CS."""
    import modules.tyre_fit_auto as tfa
    p = copy.deepcopy(ctx.params)
    p["tyre_fit_auto"]["q_beta_var"] *= q[0]
    p["tyre_fit_auto"]["q_yaw_rate_var"] *= q[1]
    orig = {k: getattr(tfa, k) for k in ("prepare_vehicle_state", "estimate_sideslip", "_fit_axle_pacejka",
                                         "estimate_slip_angles", "estimate_lateral_forces")}
    tfa.prepare_vehicle_state = lambda ch, pr: state
    tfa.estimate_sideslip = lambda st, pr: seed_beta
    if fitter is not None:
        tfa._fit_axle_pacejka = fitter
    if slip_params is not None:
        tfa.estimate_slip_angles = lambda st, b, pr: orig["estimate_slip_angles"](st, b, slip_params)
        tfa.estimate_lateral_forces = lambda st, pr: orig["estimate_lateral_forces"](st, slip_params)
    t0 = time.perf_counter()
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, p, ctx.data, "ekf_auto_pacejka",
                                                            csv_path=d2.sessions()[ctx.key]["path"])
    finally:
        for k, v in orig.items():
            setattr(tfa, k, v)
    dn = d2._downstream(ctx.params, ctx.data, state, beta)
    ax = (man or {}).get("axles", {})
    rec = {"status": (man or {}).get("status"), "gate": (gate or {}).get("verdict"),
           "health": (gate or {}).get("health_score"), "fallback": fb, "fallback_reason": fr,
           "nis": (man or {}).get("nis"), "r_chosen": ((man or {}).get("r_sweep") or {}).get("chosen"),
           "axles": {a: {k: ax[a][k] for k in ("B", "C", "D", "E", "powell_converged")} for a in ax},
           "seconds": round(time.perf_counter() - t0, 1)}
    rec.update(chatter(beta, ctx.base, state["sample_rate_hz"]))
    return np.asarray(beta, dtype=float), rec, dn


def chain_metrics(ctx, beta, rec, bound=None):
    v, n, curves = m3_pair(beta, ctx)
    rec["m3"], rec["m3_n_corners"] = v, n
    if bound is None:
        rec["m3c_bound"] = m3c_bound(curves)
    else:
        rec["m3_degraded"] = bool(v > bound) if np.isfinite(bound) else None
    return rec


# ---------------------------------------------------------------- P1

def steering_census(ctx, state=None):
    st = ctx.state if state is None else state
    sw = ctx.data["channels"]["log_asteer"]
    df = st["delta_f_rad"]
    cor = ctx.racing & (np.abs(st["ay_mps2"]) > ctx.thr)
    return {"channel": "log_asteer", "unit_header": sw.get("unit_raw"), "quality": sw.get("quality"),
            "native_rate_hz": float(1.0 / np.median(np.diff(sw["time"]))),
            "sw_std_racing_deg": float(np.std(st["steer_sw_rad"][ctx.racing]) * RAD2DEG),
            "sw_distinct": int(np.unique(sw["data"]).size),
            "steering_ratio": float(np.median(np.atleast_1d(st["steering_ratio"]))),
            "steering_ratio_level": 1,
            "zero_offset_road_deg": float(np.median(df[ctx.s0]) * RAD2DEG),
            "zero_offset_sw_deg": float(np.median(st["steer_sw_rad"][ctx.s0]) * RAD2DEG),
            "sign_corr_delta_r": float(np.corrcoef(df[cor], st["yaw_rate_radps"][cor])[0, 1])}


def measure_k(ctx, state, offset_rad):
    """Eq. 3.23 regression, beta-free: delta - L/R = K a_y with 1/R = r / v."""
    sr = state["sample_rate_hz"]
    fc = ctx.params["stability_estimation"]["cs_filter_cutoff_hz"]
    stdy = ctx.steady(state)
    pop = stdy["pop"]
    dl = _lp(state["delta_f_rad"], fc, sr) - offset_rad
    rf, vf, af = stdy["rf"], _lp(state["v_mps"], fc, sr), stdy["alog"]
    y = dl - ctx.L * rf / np.where(vf > 1.0, vf, np.nan)
    ok = pop & np.isfinite(y)
    X = np.column_stack([af[ok], np.ones(int(ok.sum()))])
    coef, *_ = np.linalg.lstsq(X, y[ok], rcond=None)
    trav = stdy["trav"][ok]
    ids = np.unique(trav)
    rng = np.random.default_rng(20261002)
    ks = []
    xs, ys = af[ok], y[ok]
    by = {i: np.where(trav == i)[0] for i in ids}
    for _ in range(K_BOOT):
        pick = np.concatenate([by[i] for i in rng.choice(ids, size=ids.size, replace=True)])
        c, *_ = np.linalg.lstsq(np.column_stack([xs[pick], np.ones(pick.size)]), ys[pick], rcond=None)
        ks.append(c[0])
    k = float(coef[0])
    return {"K_rad_per_mps2": k, "K_deg_per_g": k * RAD2DEG * G, "intercept_deg": float(coef[1] * RAD2DEG),
            "ci95_deg_per_g": [float(np.percentile(ks, 2.5) * RAD2DEG * G), float(np.percentile(ks, 97.5) * RAD2DEG * G)],
            "n_samples": int(ok.sum()), "n_traversals": int(ids.size)}, stdy


def beta_ss(ctx, state, K, Cr, offset_rad):
    v = state["v_mps"]
    d = state["delta_f_rad"] - offset_rad
    inv_r = d / (ctx.L + K * v * v)
    b = (ctx.lr - ctx.m_r * v * v / Cr) * inv_r
    return np.where(state["moving_mask"], b, 0.0)


def beta_ss_rform(ctx, state, Cr):
    v = np.where(state["moving_mask"], state["v_mps"], 1.0)
    r = state["yaw_rate_radps"]
    b = ctx.lr * r / v - ctx.m_r * v * r / Cr
    return np.where(state["moving_mask"], b, 0.0)


def anchored_beta(state, s0, anchor_mask, bss, kappa):
    """S2 segment integration plus mid-corner knots (header P1)."""
    v, moving = state["v_mps"], state["moving_mask"]
    sr = state["sample_rate_hz"]
    bd = np.where(moving, state["ay_mps2"] / np.where(moving, v, 1.0) - state["yaw_rate_radps"], 0.0)
    dt = 1.0 / sr
    min_run = int(round(ANCHOR_MIN_RUN_S * sr))
    beta = np.zeros_like(bd)
    seg = moving & ~s0
    idx = np.where(seg)[0]
    deltas, durs = [], []
    n_anch_seg = n_seg = 0
    if idx.size:
        for p in np.split(idx, np.where(np.diff(idx) != 1)[0] + 1):
            n_seg += 1
            inc = bd[p] * dt
            b = np.concatenate(([0.0], np.cumsum(inc[:-1])))
            kx, kv = [0.0], [0.0]
            am = anchor_mask[p]
            if am.any():
                e = np.diff(np.concatenate(([0], am.astype(np.int8), [0])))
                for a0, a1 in zip(np.where(e == 1)[0], np.where(e == -1)[0]):
                    if a1 - a0 >= min_run:
                        kx.append(0.5 * (a0 + a1 - 1))
                        kv.append(kappa * float(np.mean(bss[p[a0:a1]] - b[a0:a1])))
            if len(kx) > 1:
                n_anch_seg += 1
            nxt = p[-1] + 1
            if nxt < s0.size and s0[nxt]:
                kx.append(float(p.size))
                kv.append(-(b[-1] + inc[-1]))
            corr = np.interp(np.arange(p.size, dtype=float), kx, kv)
            beta[p] = b + corr
            for i in range(1, len(kx)):
                deltas.append(abs(kv[i] - kv[i - 1]) * RAD2DEG)
                durs.append((kx[i] - kx[i - 1]) / sr)
    d = np.array(deltas) if deltas else np.array([np.nan])
    rate = d / np.maximum(np.array(durs), 1e-3) if durs else np.array([np.nan])
    info = {"n_segments": n_seg, "n_segments_with_anchor": n_anch_seg, "n_intervals": len(deltas),
            "interval_abs_deg_median": float(np.nanmedian(d)), "interval_abs_deg_p90": float(np.nanpercentile(d, 90)),
            "interval_rate_degps_median": float(np.nanmedian(rate)),
            "interval_rate_degps_p90": float(np.nanpercentile(rate, 90))}
    return np.where(moving, beta, 0.0), info


def s2_intervals(state, s0):
    """S2 fresh through the same interval bookkeeping (no anchors)."""
    return anchored_beta(state, s0, np.zeros(s0.size, dtype=bool), np.zeros(s0.size), 0.0)


def _agree(ctx, est, ref, bins):
    out = {}
    for name, m in bins.items():
        d = (est - ref)[m] * RAD2DEG
        d = d[np.isfinite(d)]
        if d.size >= 20:
            q = np.percentile(d, [25, 50, 75])
            out[name] = {"median_deg": float(q[1]), "iqr_deg": float(q[2] - q[0]), "n": int(d.size)}
        else:
            out[name] = {"n": int(d.size)}
    return out


def p1(key):
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    st, sr = ctx.state, ctx.state["sample_rate_hz"]
    out = {"key": key, "session": ctx.name}
    cen = steering_census(ctx)
    out["census"] = cen
    off = cen["zero_offset_road_deg"] / RAD2DEG
    k2, stdy = measure_k(ctx, st, off)
    out["K2"] = k2
    cr1, cf1 = C_ALPHA_PER_FZ_L1 * ctx.m_r * G, C_ALPHA_PER_FZ_L1 * ctx.m_f * G
    out["K1"] = {"K_rad_per_mps2": ctx.m_f / cf1 - ctx.m_r / cr1, "C_f_N_per_rad": cf1, "C_r_N_per_rad": cr1,
                 "note": "config C_alpha/Fz = 12 rad^-1 on both axles -> neutral steer by construction"}
    man = ctx.cache["prod"]["manifest"]["axles"]
    bcd = {a: man[a]["B"] * man[a]["C"] * man[a]["D"] for a in ("front", "rear")}
    kf = ctx.m_f / bcd["front"] - ctx.m_r / bcd["rear"]
    out["K_fit"] = {"K_rad_per_mps2": kf, "K_deg_per_g": kf * RAD2DEG * G, "origin_slopes_N_per_rad": bcd,
                    "note": "production fits on kinematic alpha (circular)"}
    _log(f"[{key}] P1 census {json.dumps(cen, default=float)}")
    _log(f"[{key}] K2 {k2['K_deg_per_g']:+.3f} deg/g CI {k2['ci95_deg_per_g']}; K1 {out['K1']['K_rad_per_mps2']:+.2e} "
         f"rad/(m/s^2); K_fit {out['K_fit']['K_deg_per_g']:+.3f} deg/g")
    K = k2["K_rad_per_mps2"]
    crows = {"Level-1 stiffness (12 per rad x rear load)": cr1,
             "Level-2 stiffness (production rear origin slope, circular)": bcd["rear"]}
    b0 = estimate_sideslip(st, ctx.params)
    bekf = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    fc = 2.0
    lp = lambda b: _lp(b, fc, sr)
    b0l, bel = lp(b0), lp(bekf)
    pop = stdy["pop"]
    ay_lim = LINEAR_AY_FRACTION * float(np.percentile(np.abs(st["ay_mps2"][ctx.racing]), 95))
    bins = {"steady, linear tyre range": pop & (np.abs(stdy["alog"]) <= ay_lim),
            "steady, near the limit": pop & (np.abs(stdy["alog"]) > ay_lim),
            "in-corner transients": ctx.in_br & ctx.racing & ~pop}
    out["bins_n"] = {k: int(v.sum()) for k, v in bins.items()}
    out["linear_ay_limit_mps2"] = ay_lim
    out["rows"] = {}
    bss_store = {}
    for cname, cr in crows.items():
        for sc in STIFF_SCALES:
            lab = f"{cname} x{sc}"
            b = beta_ss(ctx, st, K, cr * sc, off)
            bl = lp(b)
            out["rows"][lab] = {"C_r_N_per_rad": cr * sc, "vs_S0": _agree(ctx, bl, b0l, bins),
                                "vs_EKF": _agree(ctx, bl, bel, bins)}
            bss_store[(cname, sc)] = b
        br = lp(beta_ss_rform(ctx, st, cr))
        out["rows"][f"{cname}, measured-curvature form (no steering)"] = {
            "C_r_N_per_rad": cr, "vs_S0": _agree(ctx, br, b0l, bins), "vs_EKF": _agree(ctx, br, bel, bins)}
        bss_store[(cname, "r")] = br
    out["rows"]["K1 = 0 (Level-1, neutral), Level-1 stiffness"] = {
        "vs_S0": _agree(ctx, lp(beta_ss(ctx, st, 0.0, cr1, off)), b0l, bins),
        "vs_EKF": _agree(ctx, lp(beta_ss(ctx, st, 0.0, cr1, off)), bel, bins)}
    for lab, r in out["rows"].items():
        a = r["vs_EKF"]
        _log(f"[{key}]   {lab}: vs EKF " + "; ".join(f"{b}: {a[b].get('median_deg', float('nan')):+.2f} "
                                                      f"(IQR {a[b].get('iqr_deg', float('nan')):.2f})" for b in bins))
    render_p1_traces(ctx, b0l, bel, {c: {sc: lp(bss_store[(c, sc)]) for sc in STIFF_SCALES} for c in crows},
                     {c: bss_store[(c, "r")] for c in crows}, pop)
    # ---- anchor experiment
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
    s0_rec = evaluate(ctx, "p1_s0", "production kinematic seed (fresh baseline)", st, b0, ctx.params, render=True)
    out["S0"] = s0_rec
    _entry(ctx, out, b0, s0_rec)
    s2b, s2i = s2_intervals(st, ctx.s0)
    out["S2"] = evaluate(ctx, "p1_s2", "segment integration, straights pinned to zero (fresh S2)", st, s2b,
                         ctx.params, s0_rec=s0_rec, beta_ref=b0)
    out["S2"]["intervals"] = s2i
    out["anchored"] = {}
    for cname, cr in crows.items():
        for kap in KAPPAS:
            lab = f"segment integration + mid-corner steady-state anchor, {cname}, weight {kap}"
            short = f"anch_{'L1' if 'Level-1' in cname else 'L2'}_k{kap}"
            b, info = anchored_beta(st, ctx.s0, pop, bss_store[(cname, 1.0)], kap)
            rec = evaluate(ctx, f"p1_{short}", lab, st, b, ctx.params, s0_rec=s0_rec, beta_ref=b0)
            rec["intervals"] = info
            rec["C_r_assumed"] = cr
            rec["interval_p90_change_vs_S2"] = 1 - info["interval_abs_deg_p90"] / s2i["interval_abs_deg_p90"]
            slip = estimate_slip_angles(st, b, ctx.params)
            fy_r = estimate_lateral_forces(st, ctx.params)["Fy_r_filt"]
            am = pop & ctx.base & np.isfinite(slip["alpha_r_filt"]) & np.isfinite(fy_r)
            ar_, fr_ = slip["alpha_r_filt"][am], fy_r[am]
            slope = float(np.sum(ar_ * fr_) / np.sum(ar_ * ar_)) if ar_.size else float("nan")
            s_s0 = s0_rec["rear"]["mech"]["origin_slope_N_per_rad"]
            s_an = rec["rear"]["mech"]["origin_slope_N_per_rad"]
            rec["circularity"] = {"anchor_sample_slope_N_per_rad": slope, "slope_ratio_to_C_r": slope / cr,
                                  "refit_origin_slope": s_an, "s0_refit_origin_slope": s_s0,
                                  "refit_closer_than_s0": bool(abs(s_an - cr) < abs(s_s0 - cr)),
                                  "confirmed_session": bool(abs(s_an - cr) < abs(s_s0 - cr) and
                                                            abs(slope / cr - 1) <= CIRC_SLOPE_TOL)}
            out["anchored"][short] = rec
            _log(f"[{key}] {short}: intervals p90 {info['interval_abs_deg_p90']:.2f} deg vs S2 "
                 f"{s2i['interval_abs_deg_p90']:.2f} ({rec['interval_p90_change_vs_S2']:+.2f}); anchored segs "
                 f"{info['n_segments_with_anchor']}/{info['n_segments']}; M1c shrink f {rec['front']['m1c_shrink']:+.3f} "
                 f"r {rec['rear']['m1c_shrink']:+.3f}; M3 {rec['m3']:.3f} (bound {s0_rec['m3c_bound']:.3f}); "
                 f"circularity {json.dumps(rec['circularity'], default=float)}")
    _dump(f"p1_{key}.json", out)


def _entry(ctx, out, b0, s0_rec):
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
    if ctx.cache is None:
        out["entry"] = "no cache"
        return
    slip = estimate_slip_angles(ctx.state, b0, ctx.params)
    fo = estimate_lateral_forces(ctx.state, ctx.params)
    c = ctx.cache
    out["entry"] = {"arrays_identical": {k: bool(np.array_equal(a, b)) for k, a, b in (
        ("alpha_f", slip["alpha_f_filt"], c["alpha_f"]), ("alpha_r", slip["alpha_r_filt"], c["alpha_r"]),
        ("fy_f", fo["Fy_f_filt"], c["fy_f"]), ("fy_r", fo["Fy_r_filt"], c["fy_r"]), ("base", ctx.base, c["base_mask"]))},
        "refit_max_rel": {a: max(abs(s0_rec[a]["refit"][k] / c["prod"]["manifest"]["axles"][a][k] - 1) for k in "BCDE")
                          for a in ("front", "rear")}}
    _log(f"[{ctx.key}] ENTRY {json.dumps(out['entry'], default=float)}")


def render_p1_traces(ctx, b0l, bel, bss, brform, pop):
    """Cross-lap median on a 2 m grid per estimate, two panels (stiffness rows)."""
    plt = _plt()
    t, s = ctx.state["time"], ctx.state["s_m"]
    an = ctx.analysis_laps()
    grid = np.arange(0.0, _lap_len(ctx), 2.0)

    def med(b):
        rows = []
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"]) & np.isfinite(s)
            o = np.argsort(s[m])
            rows.append(np.interp(grid, s[m][o], np.asarray(b)[m][o] * RAD2DEG, left=np.nan, right=np.nan))
        return np.nanmedian(np.array(rows), axis=0)

    steady_s = []
    for l in an:
        m = (t >= l["start_time"]) & (t <= l["end_time"]) & pop
        steady_s.append(s[m])
    steady_s = np.concatenate(steady_s) if steady_s else np.array([])
    m0, me = med(b0l), med(bel)
    # shared y: the EKF is one fixed series in every panel, so only a common
    # scale shows that the steady-state estimate is what changes between
    # stiffness rows (Q-STIFF-CS Part 0, author finding 2026-10-03)
    fig, axes = plt.subplots(len(bss), 1, figsize=(max(14, _lap_len(ctx) / 400), 3.6 * len(bss) + 0.8),
                             sharex=True, sharey=True, squeeze=False, constrained_layout=True)
    for ax, (cname, rows) in zip(axes[:, 0], bss.items()):
        for b0_, b1_ in _brackets(ctx):
            ax.axvspan(b0_, b1_, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8)
        lo, mid, hi = med(rows[STIFF_SCALES[0]]), med(rows[1.0]), med(rows[STIFF_SCALES[-1]])
        ax.fill_between(grid, np.fmin(lo, hi), np.fmax(lo, hi), color=ACC2, alpha=0.18, lw=0,
                        label="steady-state beta, rear stiffness +/-30 %")
        ax.plot(grid, mid, color=ACC2, lw=1.1, label="steady-state beta from steering (Rajamani Eq. 8.24)")
        ax.plot(grid, med(_lp(brform[cname], 2.0, ctx.state["sample_rate_hz"])), color=ACC2, lw=0.8, ls=":",
                label="steady-state beta from measured curvature (no steering)")
        ax.plot(grid, m0, color=REF, lw=1.1, label="production kinematic seed")
        ax.plot(grid, me, color=ACC, lw=1.1, label="production EKF")
        ax.set_ylabel("beta (deg), 2 Hz low-pass", color=INK)
        ax.set_title(cname, fontsize=9, color=INK, loc="left")
        ax.legend(loc="upper right", frameon=False, fontsize=7, ncol=2)
        _style(ax)
    if steady_s.size:
        y0 = axes[0, 0].get_ylim()[0]
        for ax in axes[:, 0]:
            ax.plot(steady_s, np.full(steady_s.size, y0), "|", color=INK, ms=4, alpha=0.2)
    axes[-1, 0].set_xlabel("lap distance (m)", color=INK)
    fig.suptitle(_wrap(f"{ctx.name} -- sideslip beta, cross-lap median: steering-derived steady state vs kinematic seed "
                       f"vs EKF (common y-scale in both panels; rug = steady mid-corner samples, grey bands = corner "
                       f"brackets)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"p1_traces_{ctx.key}.png"), dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- P2

def hampel(x, sr):
    from numpy.lib.stride_tricks import sliding_window_view
    h = int(round(HAMPEL_HALF_S * sr))
    xp = np.pad(np.asarray(x, dtype=float), h, mode="edge")
    w = sliding_window_view(xp, 2 * h + 1)
    med = np.median(w, axis=1)
    mad = MAD_TO_SD * np.median(np.abs(w - med[:, None]), axis=1)
    bad = np.abs(x - med) > HAMPEL_K * mad
    return np.where(bad, med, x), float(np.mean(bad))


def despike_state(state):
    st = dict(state)
    sr = state["sample_rate_hz"]
    shares = {}
    for k in ("ay_mps2", "yaw_rate_radps", "delta_f_rad", "steer_sw_rad"):
        st[k], shares[k] = hampel(state[k], sr)
    return st, shares


def p2(key):
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
    ctx = Ctx(key)
    out = {"key": key, "session": ctx.name}
    b0 = estimate_sideslip(ctx.state, ctx.params)
    s0_rec = evaluate(ctx, "p2_s0", "production kinematic seed (fresh baseline)", ctx.state, b0, ctx.params)
    out["S0"] = s0_rec
    _entry(ctx, out, b0, s0_rec)
    st, shares = despike_state(ctx.state)
    out["despike_outlier_share"] = shares
    bd = estimate_sideslip(st, ctx.params)
    out["despike"] = evaluate(ctx, "p2_despike", "inputs despiked (Hampel, 0.05 s, 3 MAD) before the seed",
                              st, bd, ctx.params, s0_rec=s0_rec, beta_ref=b0)
    _log(f"[{key}] P2 despike shares {shares}; M1c shrink f {out['despike']['front']['m1c_shrink']:+.3f} "
         f"r {out['despike']['rear']['m1c_shrink']:+.3f}; M3 {out['despike']['m3']:.3f}")
    slip = estimate_slip_angles(ctx.state, b0, ctx.params)
    fo = estimate_lateral_forces(ctx.state, ctx.params)
    out["loss"] = {}
    for axle, a, f in (("front", slip["alpha_f_filt"], fo["Fy_f_filt"]), ("rear", slip["alpha_r_filt"], fo["Fy_r_filt"])):
        curves = [("least squares (production)", s0_rec[axle]["refit"], REF, "--")]
        for loss, lab, col in (("huber", "Huber loss", ACC), ("trim", "trimmed least squares (10 %)", ACC2)):
            fit = fit_axle(a, f, ctx.base, loss=loss)
            mech = mechanics(a, f, ctx.base, fit, ctx, axle, loss=loss)
            r = {"refit": _strip(fit), "mech": mech,
                 "resid_mad_ratio": mech["resid_mad_N"] / s0_rec[axle]["mech"]["resid_mad_N"],
                 "curve_vs_s0": ss.curve_delta(fit, s0_rec[axle]["refit"], s0_rec[axle]["refit"]["visited_alpha_p99_deg"])}
            out["loss"].setdefault(loss, {})[axle] = r
            curves.append((lab, fit, col, "-"))
            _log(f"[{key}] P2 {loss} {axle}: B {fit['B']:.3f} C {fit['C']:.3f} D {fit['D']:.0f} E {fit['E']:.3f}; "
                 f"conv {mech['converged']}; start sens {mech['start_sens_pct_D']:.2f} % D; MAD ratio "
                 f"{r['resid_mad_ratio']:.3f}; curve vs LS {r['curve_vs_s0']['curve_rms_pct_D']:.2f} % D")
        render_curves(os.path.join(OUT_DIR, f"p2_robust_{key}_{axle}.png"),
                      f"{ctx.name} -- {axle} tyre fit: least squares vs Huber vs trimmed, production seed, all laps",
                      a, f, ctx.base, curves)
    _dump(f"p2_{key}.json", out)


# ---------------------------------------------------------------- P3

def lap_sets(ctx):
    an = sorted(ctx.analysis_laps(), key=lambda l: l["end_time"] - l["start_time"])
    times = {l["lap_number"]: l["end_time"] - l["start_time"] for l in an}
    out = {"lap_times_s": times}
    n = len(an)
    out["fast_half"] = [l["lap_number"] for l in an[:max(2, math.ceil(n / 2))]] if n >= 2 else None
    best = min(times.values()) if times else float("nan")
    f101 = [k for k, v in times.items() if v <= FAST101 * best]
    out["fast101"] = f101 if len(f101) >= 2 else None
    return out


def p3(key):
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    out = {"key": key, "session": ctx.name}
    b0 = estimate_sideslip(ctx.state, ctx.params)
    s0_rec = evaluate(ctx, "p3_s0", "production seed, least squares, all analysis laps (fresh baseline)",
                      ctx.state, b0, ctx.params)
    out["S0"] = s0_rec
    _entry(ctx, out, b0, s0_rec)
    ls = lap_sets(ctx)
    out["lap_sets"] = ls
    rows = (("fast", "fastest half of laps, least squares", "fast_half", "ls", False),
            ("fast101", "laps within 101 % of best (sensitivity), least squares", "fast101", "ls", False),
            ("fast_huber", "fastest half of laps, Huber loss", "fast_half", "huber", False),
            ("fast_huber_env", "fastest half of laps, Huber loss, upper envelope (q 0.9)", "fast_half", "huber", True))
    out["rows"] = {}
    for short, lab, lset, loss, env in rows:
        if ls[lset] is None:
            out["rows"][short] = {"label": lab, "not_run": "lap count does not allow this set"}
            _log(f"[{key}] P3 {short}: not run (lap count)")
            continue
        lm = ctx.lap_mask(set(ls[lset]))
        rec = evaluate(ctx, f"p3_{short}", f"{lab} (laps {ls[lset]})", ctx.state, b0, ctx.params, fit_mask_laps=lm,
                       loss=loss, envelope=env, s0_rec=s0_rec, beta_ref=None)
        rec["laps"] = ls[lset]
        out["rows"][short] = rec
        _log(f"[{key}] P3 {short}: laps {ls[lset]}; M1c shrink f {rec['front']['m1c_shrink']:+.3f} r "
             f"{rec['rear']['m1c_shrink']:+.3f}; " + "; ".join(
                 f"{a} B {rec[a]['refit']['B']:.2f} C {rec[a]['refit']['C']:.3f} D {rec[a]['refit']['D']:.0f} "
                 f"E {rec[a]['refit']['E']:.2f} conv {rec[a]['mech']['converged']} band {rec[a]['mech']['band_ok']} "
                 f"start {rec[a]['mech']['start_sens_pct_D']:.2f}% curve vs all {rec[a]['curve_vs_s0']['curve_rms_pct_D']:.1f}%"
                 for a in ("front", "rear")))
    _dump(f"p3_{key}.json", out)


# ---------------------------------------------------------------- P4

def _psd(t, v, laps):
    from scipy.signal import welch, detrend
    w = np.zeros(t.size, dtype=bool)
    for l in laps:
        if l.get("is_valid_for_analysis"):
            w |= (t >= l["start_time"]) & (t <= l["end_time"])
    rate = float(1.0 / np.median(np.diff(t)))
    f, p = welch(detrend(v[w]), fs=rate, nperseg=int(PSD_SEG_S * rate))
    return f, 10 * np.log10(p + 1e-30), rate


def _first_sustained(f, cond, f_min):
    df = f[1] - f[0]
    n = max(1, int(round(PSD_SUSTAIN_HZ / df)))
    for i in np.where(f >= f_min)[0]:
        if i + n <= f.size and cond[i:i + n].all():
            return float(f[i])
    return float("nan")


def measure_spectra(ctx):
    from diagnostics.inspect_imu_offsets import read_raw_blocks
    raw = read_raw_blocks(d2.sessions()[ctx.key]["path"], ("log_acc_y", "sclu_acc_y"))
    fl, dl, rl = _psd(raw["log_acc_y"][0], raw["log_acc_y"][1], ctx.data["laps"])
    fs_, ds, _rs = _psd(raw["sclu_acc_y"][0], raw["sclu_acc_y"][1], ctx.data["laps"])
    dsi = np.interp(fl, fs_, ds)
    f_acc = _first_sustained(fl, (dl - dsi) > PSD_SPLIT_DB, 2.0)
    sw = ctx.data["channels"]["log_asteer"]
    fst, dst, rst = _psd(sw["time"], sw["data"], ctx.data["laps"])
    floor = float(np.median(dst[fst >= 0.8 * fst[-1]]))
    f_st = _first_sustained(fst, dst <= floor + PSD_SPLIT_DB, 1.0)
    # lag on a common 100 Hz grid over the analysis window
    an = ctx.analysis_laps()
    g = np.arange(an[0]["start_time"], an[-1]["end_time"], 0.01)
    x = np.interp(g, raw["log_acc_y"][0], raw["log_acc_y"][1])
    y = np.interp(g, raw["sclu_acc_y"][0], raw["sclu_acc_y"][1])
    x, y = x - x.mean(), y - y.mean()
    k = int(LAG_MAX_S / 0.01)
    cc = {s: float(np.dot(x[:x.size - s], y[s:])) if s >= 0 else float(np.dot(x[-s:], y[:y.size + s]))
          for s in range(-k, k + 1)}
    lag = max(cc, key=cc.get) * 0.01   # > 0: sclu_acc_y lags log_acc_y
    return {"f_acc_hz": f_acc, "f_steer_hz": f_st, "steer_floor_db": floor, "lag_sclu_behind_log_s": lag,
            "log_rate_hz": rl, "steer_rate_hz": rst,
            "_psd": {"f": fl, "log": dl, "sclu": dsi, "f_st": fst, "steer": dst}}


def filtered_state(ctx, state, spec, lag=False):
    st = dict(state)
    sr = state["sample_rate_hz"]
    ay = state["ay_mps2"]
    if lag:
        ay = np.interp(state["time"] - spec["lag_sclu_behind_log_s"], state["time"], ay)
    if np.isfinite(spec["f_acc_hz"]) and spec["f_acc_hz"] < 0.45 * sr:
        ay = _lp(ay, spec["f_acc_hz"], sr)
    st["ay_mps2"] = ay
    if np.isfinite(spec["f_steer_hz"]) and spec["f_steer_hz"] < 0.45 * sr:
        st["delta_f_rad"] = _lp(state["delta_f_rad"], spec["f_steer_hz"], sr)
        st["steer_sw_rad"] = _lp(state["steer_sw_rad"], spec["f_steer_hz"], sr)
    return st


def render_psd(ctx, spec):
    plt = _plt()
    p = spec["_psd"]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    axes[0].plot(p["f"], p["log"], color=INK, lw=1, label="logger accelerometer log_acc_y (unfiltered)")
    axes[0].plot(p["f"], p["sclu"], color=ACC, lw=1, label="cluster accelerometer sclu_acc_y (filtered on board)")
    axes[0].axvline(spec["f_acc_hz"], color=ACC2, lw=1.2, ls="--", label=f"matched cutoff {spec['f_acc_hz']:.2f} Hz")
    axes[0].set_title("lateral acceleration spectra, analysis laps", fontsize=9, loc="left")
    axes[1].plot(p["f_st"], p["steer"], color=INK, lw=1, label="steering-wheel angle log_asteer")
    axes[1].axhline(spec["steer_floor_db"], color=REF, lw=0.8, ls=":", label="high-band floor")
    axes[1].axvline(spec["f_steer_hz"], color=ACC2, lw=1.2, ls="--", label=f"matched cutoff {spec['f_steer_hz']:.2f} Hz")
    axes[1].set_title("steering spectrum, analysis laps", fontsize=9, loc="left")
    for ax in axes:
        ax.set_xscale("log")
        ax.set_xlabel("frequency (Hz)", color=INK)
        ax.set_ylabel("PSD (dB, header units)", color=INK)
        ax.legend(frameon=False, fontsize=7)
        ax.grid(color=BAND, lw=0.6)
    fig.suptitle(f"{ctx.name} -- input spectra and the spectrum-matched filter cutoffs (lag sclu behind log "
                 f"{spec['lag_sclu_behind_log_s'] * 1000:+.0f} ms)", fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"p4_spectra_{ctx.key}.png"), dpi=110)
    plt.close(fig)


def p4(key):
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    out = {"key": key, "session": ctx.name}
    spec = measure_spectra(ctx)
    render_psd(ctx, spec)
    out["spectra"] = {k: v for k, v in spec.items() if k != "_psd"}
    _log(f"[{key}] P4 spectra {json.dumps(out['spectra'], default=float)}")
    b0 = estimate_sideslip(ctx.state, ctx.params)
    s0_rec = evaluate(ctx, "p4_s0", "production kinematic seed, uniform 2 Hz (fresh baseline)", ctx.state, b0, ctx.params)
    out["S0"] = s0_rec
    _entry(ctx, out, b0, s0_rec)
    pb = copy.deepcopy(ctx.params)
    pb["stability_estimation"]["cs_filter_cutoff_hz"] = float(spec["f_acc_hz"])
    rows = (("p4a", "spectrum-matched input filters, alpha/Fy low-pass kept at 2 Hz", False, ctx.params),
            ("p4b", f"spectrum-matched input filters, alpha/Fy low-pass at {spec['f_acc_hz']:.1f} Hz instead of 2 Hz",
             False, pb),
            ("p4c", "spectrum-matched input filters + accelerometer lag aligned to the cluster, 2 Hz kept", True,
             ctx.params))
    out["rows"] = {}
    for short, lab, lag, pp in rows:
        st = filtered_state(ctx, ctx.state, spec, lag=lag)
        b = estimate_sideslip(st, ctx.params)
        rec = evaluate(ctx, f"p4_{short}", lab, st, b, pp, s0_rec=s0_rec, beta_ref=b0)
        out["rows"][short] = rec
        _log(f"[{key}] P4 {short}: M1c shrink f {rec['front']['m1c_shrink']:+.3f} r {rec['rear']['m1c_shrink']:+.3f}; "
             f"M1 shrink f {rec['front']['m1_shrink']:+.3f} r {rec['rear']['m1_shrink']:+.3f}; IQR ratio f "
             f"{rec['front']['iqr_ratio']:.3f} r {rec['rear']['iqr_ratio']:.3f}; M3 {rec['m3']:.3f}")
    _dump(f"p4_{key}.json", out)


# ---------------------------------------------------------------- P5

def p5(key, rows=Q_ROWS, tag="p5"):
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    path = os.path.join(OUT_DIR, f"{tag}_{key}.json")
    out = json.load(open(path)) if os.path.exists(path) else {"key": key, "session": ctx.name, "rows": {}}
    b0 = estimate_sideslip(ctx.state, ctx.params)
    order = sorted(rows, key=lambda r: r[1:] != (1.0, 1.0))   # production Q first: it carries the bound
    bound = out["rows"].get("production Q", {}).get("m3c_bound")
    for lab, qb, qy in order:
        if lab in out["rows"]:
            continue
        beta, rec, dn = run_chain(ctx, ctx.state, b0, q=(qb, qy))
        is_prod = (qb, qy) == (1.0, 1.0)
        chain_metrics(ctx, beta, rec, bound=None if is_prod else bound)
        if is_prod:
            bound = rec["m3c_bound"]
            if ctx.cache is not None:
                c = ctx.cache["prod"]
                rec["entry"] = {"health_cached": c["gate"]["health_score"], "health_fresh": rec["health"],
                                "beta_max_abs_diff_deg": float(np.max(np.abs(beta - np.asarray(c["beta"], dtype=float)))
                                                               * RAD2DEG)}
        out["rows"][lab] = rec
        render_cs_bridged(os.path.join(OUT_DIR, f"{tag}_cs_{key}_{_slug(lab)}.png"),
                          f"{ctx.name} -- CS ratio, production chain with EKF process noise {lab} "
                          f"(smoothness setting, not information)", ctx, ctx.state, dn["cs_f"], dn["cs_r"])
        render_beta(os.path.join(OUT_DIR, f"{tag}_beta_{key}_{_slug(lab)}.png"),
                    f"{ctx.name} -- EKF sideslip beta, process noise {lab}", ctx,
                    [(f"EKF beta, {lab} (gate {rec['gate']}, chatter {rec['chatter_rms_deg']:.2f} deg rms)", beta, INK)])
        _log(f"[{key}] P5 {lab}: status {rec['status']} gate {rec['gate']} health {rec['health']} fb {rec['fallback']}; "
             f"chatter {rec['chatter_rms_deg']:.3f} deg (>2 Hz share {rec['psd_share_above_2hz']:.4f}); M3 "
             f"{rec['m3']:.3f}; {rec['seconds']} s" + (f"; ENTRY {rec.get('entry')}" if is_prod else ""))
        _dump(f"{tag}_{key}.json", out)
    render_p5_summary(ctx, out)


def _slug(s):
    return "".join(ch if ch.isalnum() else "_" for ch in s).strip("_").replace("__", "_")


def render_p5_summary(ctx, out):
    plt = _plt()
    rows = [(lab, r) for lab, r in out["rows"].items() if "x" in lab or lab == "production Q"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    labs = [l for l, _ in rows]
    axes[0].bar(range(len(rows)), [r["chatter_rms_deg"] for _, r in rows], color=ACC)
    axes[0].set_ylabel("beta chatter, rms above 2 Hz (deg)", color=INK)
    axes[1].bar(range(len(rows)), [r["health"] if r["health"] is not None else np.nan for _, r in rows], color=INK)
    axes[1].set_ylabel("NIS gate health score", color=INK)
    for ax in axes:
        ax.set_xticks(range(len(rows)))
        ax.set_xticklabels(labs, rotation=30, ha="right", fontsize=7)
        _style(ax)
    fig.suptitle(f"{ctx.name} -- EKF process-noise sweep: beta chatter and gate health (smoothness, not information)",
                 fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, f"p5_summary_{ctx.key}.png"), dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- gate

def _frac(vals, need):
    return sum(bool(v) for v in vals) >= need


def gate():
    res = {"elements": {}}
    R = {p: {k: _load(f"{p}_{k}.json") for k in TIER1} for p in ("p1", "p2", "p3", "p4", "p5")}

    def seed_gate(recs):
        ok = [r is not None and "front" in r for r in recs]
        if not all(ok):
            return {"promising": False, "reason": "missing on a Tier-1 session"}
        m1c = [max(r["front"]["m1c_shrink"], r["rear"]["m1c_shrink"]) >= SHRINK_GATE for r in recs]
        m3 = [r.get("m3_degraded") is not True for r in recs]
        conv = [r["front"]["mech"]["converged"] and r["rear"]["mech"]["converged"] for r in recs]
        return {"promising": bool(_frac(m1c, 2) and all(m3) and all(conv)), "m1c_met": m1c, "m3_ok": m3, "conv": conv,
                "primary_mean_best_axle_m1c_shrink": float(np.mean([r["best_axle_m1c_shrink"] for r in recs]))}

    for short in ("anch_L1_k1.0", "anch_L1_k0.5", "anch_L2_k1.0", "anch_L2_k0.5"):
        res["elements"][f"P1 {short}"] = seed_gate([R["p1"][k]["anchored"][short] for k in TIER1])
        res["elements"][f"P1 {short}"]["fixes_spread"] = _frac(
            [R["p1"][k]["anchored"][short]["interval_p90_change_vs_S2"] >= SHRINK_GATE for k in TIER1], 2)
    for cr in ("L1", "L2"):
        res[f"P1 circularity {cr}"] = {k: R["p1"][k]["anchored"][f"anch_{cr}_k1.0"]["circularity"] for k in TIER1}
    res["P1 circularity claim CONFIRMED"] = all(
        _frac([R["p1"][k]["anchored"][f"anch_{cr}_k1.0"]["circularity"]["confirmed_session"] for k in TIER1], 2)
        for cr in ("L1", "L2"))
    res["elements"]["P2 despike"] = seed_gate([R["p2"][k]["despike"] for k in TIER1])
    for loss in ("huber", "trim"):
        recs = [R["p2"][k]["loss"][loss] for k in TIER1]
        mech = [r[a]["mech"]["converged"] and r[a]["mech"]["start_ok"] for r in recs for a in ("front", "rear")]
        mad = [r["front"]["resid_mad_ratio"] < 1 and r["rear"]["resid_mad_ratio"] < 1 for r in recs]
        res["elements"][f"P2 loss {loss}"] = {
            "promising": bool(all(mech) and _frac(mad, 2)), "mech_all": mech, "mad_lower": mad,
            "primary_mean_resid_mad_ratio": float(np.mean([r[a]["resid_mad_ratio"] for r in recs for a in ("front", "rear")]))}
    for short in ("fast", "fast101", "fast_huber", "fast_huber_env"):
        recs = [R["p3"][k]["rows"][short] for k in TIER1]
        if any("not_run" in r for r in recs):
            res["elements"][f"P3 {short}"] = {"promising": False, "reason": "not run on every Tier-1 session (lap count)"}
            continue
        g = seed_gate(recs)
        if short == "fast_huber_env":
            band = [r[a]["mech"]["band_ok"] and r[a]["mech"]["converged"] and r[a]["mech"]["start_ok"]
                    for r in recs for a in ("front", "rear")]
            g["envelope_mech_and_band_all"] = bool(all(band))
            g["promising"] = bool(g["promising"] and all(band))
        res["elements"][f"P3 {short}"] = g
    for short in ("p4a", "p4b", "p4c"):
        res["elements"][f"P4 {short}"] = seed_gate([R["p4"][k]["rows"][short] for k in TIER1])
    for lab, qb, qy in Q_ROWS:
        if (qb, qy) == (1.0, 1.0):
            continue
        recs = [(R["p5"][k]["rows"][lab], R["p5"][k]["rows"]["production Q"]) for k in TIER1]
        red = [1 - r["chatter_rms_deg"] / b["chatter_rms_deg"] for r, b in recs]
        gate_ok = [VERDICT_RANK[r["gate"]] >= VERDICT_RANK[b["gate"]] for r, b in recs]
        fb = [not r["fallback"] for r, _ in recs]
        res["elements"][f"P5 {lab}"] = {"promising": bool(_frac([x >= CHATTER_GATE for x in red], 2) and all(gate_ok)
                                                          and all(fb)),
                                        "chatter_reduction": red, "gate_not_worse": gate_ok, "no_fallback": fb,
                                        "primary_mean_chatter_reduction": float(np.mean(red))}
    E = res["elements"]

    def best(prefix, key, higher=True, only_promising=False):
        c = [(n, e) for n, e in E.items() if n.startswith(prefix) and key in e and (e["promising"] or not only_promising)]
        if not c:
            return None
        return (max if higher else min)(c, key=lambda x: x[1][key])[0]

    stacks = {}
    for which, only in (("qualified", True), ("everything_on", False)):
        s = {"despike": None, "filters": None, "seed": None, "population": None, "loss": None, "envelope": False,
             "q": None}
        if E["P2 despike"]["promising"] or not only:
            s["despike"] = True
        s["filters"] = best("P4 ", "primary_mean_best_axle_m1c_shrink", only_promising=only)
        s["seed"] = best("P1 ", "primary_mean_best_axle_m1c_shrink", only_promising=only)
        if only:
            pop = best("P3 ", "primary_mean_best_axle_m1c_shrink", only_promising=True)
            s["population"] = pop
            s["loss"] = best("P2 loss", "primary_mean_resid_mad_ratio", higher=False, only_promising=True)
            if pop == "P3 fast_huber_env":
                s["envelope"] = True
        else:
            s["population"] = "P3 fast"
            s["loss"] = best("P2 loss", "primary_mean_resid_mad_ratio", higher=False)
            s["envelope"] = bool(E["P3 fast_huber_env"].get("envelope_mech_and_band_all", False))
        s["q"] = best("P5 ", "primary_mean_chatter_reduction", only_promising=only)
        stacks[which] = s
    res["stacks"] = stacks
    res["escalate"] = [n for n, e in E.items() if e.get("promising")]
    _dump("gate.json", res)
    _log(json.dumps(res, indent=1, default=float))


# ---------------------------------------------------------------- escalation (all-18)

def esc(element, key):
    """Promising element on one pool session: metrics + renders (Q rows: chain
    against the cached production chain)."""
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    path = os.path.join(OUT_DIR, f"esc_{key}.json")
    out = json.load(open(path)) if os.path.exists(path) else {"key": key, "session": ctx.name, "elements": {}}
    b0 = estimate_sideslip(ctx.state, ctx.params)
    if element.startswith("P5 "):
        lab = element[3:]
        qb, qy = {r[0]: r[1:] for r in Q_ROWS}[lab]
        beta, rec, dn = run_chain(ctx, ctx.state, b0, q=(qb, qy))
        base = chatter(np.asarray(ctx.cache["prod"]["beta"], dtype=float), ctx.base, ctx.state["sample_rate_hz"])
        rec["chatter_cached_production"] = base
        rec["chatter_reduction"] = 1 - rec["chatter_rms_deg"] / base["chatter_rms_deg"]
        rec["gate_cached_production"] = ctx.cache["prod"]["gate"]
        render_cs_bridged(os.path.join(OUT_DIR, f"esc_cs_{key}_{_slug(lab)}.png"),
                          f"{ctx.name} -- CS ratio, production chain with EKF process noise {lab}", ctx, ctx.state,
                          dn["cs_f"], dn["cs_r"])
        out["elements"][element] = rec
    else:
        s0_rec = out.get("S0") or evaluate(ctx, "esc_s0", "production kinematic seed (fresh baseline)", ctx.state, b0,
                                           ctx.params, render=False)
        out["S0"] = s0_rec
        st, b, pp, lm, loss, env, lab = build_element(ctx, element, b0)
        out["elements"][element] = evaluate(ctx, f"esc_{_slug(element)}", lab, st, b, pp, fit_mask_laps=lm, loss=loss,
                                            envelope=env, s0_rec=s0_rec, beta_ref=b0)
    r = out["elements"][element]
    _log(f"[{key}] ESC {element}: " + (f"chatter red {r['chatter_reduction']:+.3f} gate {r['gate']} fb {r['fallback']}"
                                      if element.startswith("P5 ") else
                                      f"M1c shrink f {r['front']['m1c_shrink']:+.3f} r {r['rear']['m1c_shrink']:+.3f}; "
                                      f"M3 degraded {r.get('m3_degraded')}"))
    _dump(f"esc_{key}.json", out)


def build_element(ctx, element, b0):
    """(state, seed beta, slip params, lap mask, loss, envelope, label) for one
    non-Q element on its own (escalation)."""
    st, b, pp, lm, loss, env = ctx.state, b0, ctx.params, None, "ls", False
    if element == "P2 despike":
        st, _sh = despike_state(ctx.state)
        from modules.stability_analysis import estimate_sideslip
        b = estimate_sideslip(st, ctx.params)
        lab = "inputs despiked before the seed"
    elif element.startswith("P4 "):
        spec = measure_spectra(ctx)
        short = element[3:]
        st = filtered_state(ctx, ctx.state, spec, lag=(short == "p4c"))
        from modules.stability_analysis import estimate_sideslip
        b = estimate_sideslip(st, ctx.params)
        if short == "p4b":
            pp = copy.deepcopy(ctx.params)
            pp["stability_estimation"]["cs_filter_cutoff_hz"] = float(spec["f_acc_hz"])
        lab = f"spectrum-matched input filters ({short})"
    elif element.startswith("P2 loss "):
        # added after the gate, before escalation: loss elements had no
        # branch; same construction as P2 (production seed and population)
        loss = element[8:]
        lab = {"huber": "Huber loss", "trim": "trimmed least squares (10 %)"}[loss] + ", production seed, all laps"
    elif element.startswith("P1 "):
        b, lab = anchored_seed(ctx, st, element[3:])
    elif element.startswith("P3 "):
        short = element[3:]
        ls = lap_sets(ctx)
        lset = "fast101" if short == "fast101" else "fast_half"
        if ls[lset] is None:
            raise RuntimeError(f"{element}: lap count does not allow it on {ctx.key}")
        lm = ctx.lap_mask(set(ls[lset]))
        loss = "huber" if "huber" in short else "ls"
        env = short.endswith("_env")
        lab = f"fit population {short} (laps {ls[lset]})"
    else:
        raise ValueError(element)
    return st, b, pp, lm, loss, env, lab


def anchored_seed(ctx, st, short):
    """Anchored S2 seed on a given state; K2 and offset re-measured on it."""
    cen = steering_census(ctx, st)
    off = cen["zero_offset_road_deg"] / RAD2DEG
    k2, stdy = measure_k(ctx, st, off)
    if "L1" in short:
        cr = C_ALPHA_PER_FZ_L1 * ctx.m_r * G
    else:
        a = ctx.cache["prod"]["manifest"]["axles"]["rear"]
        cr = a["B"] * a["C"] * a["D"]
    kap = float(short.split("_k")[1])
    s0 = ctx.s0
    b, _info = anchored_beta(st, s0, stdy["pop"], beta_ss(ctx, st, k2["K_rad_per_mps2"], cr, off), kap)
    return b, f"segment integration + mid-corner steady-state anchor ({short})"


STEP_WORDS = {
    "despike": "inputs despiked",
    "filters p4a": "spectrum-matched input filters", "filters p4b": "spectrum-matched filters incl. slip/force low-pass",
    "filters p4c": "spectrum-matched input filters with accelerometer lag aligned",
    "seed anch_L1_k1.0": "seed pinned to zero on straights + steady-state anchors (Level-1 stiffness, full weight)",
    "seed anch_L1_k0.5": "seed pinned to zero on straights + steady-state anchors (Level-1 stiffness, half weight)",
    "seed anch_L2_k1.0": "seed pinned to zero on straights + steady-state anchors (Level-2 stiffness, full weight)",
    "seed anch_L2_k0.5": "seed pinned to zero on straights + steady-state anchors (Level-2 stiffness, half weight)",
    "population fast_half": "fastest half of the laps", "population fast101": "laps within 101 % of best",
    "loss trim": "trimmed least-squares tyre fit", "loss huber": "Huber-loss tyre fit",
    "upper envelope": "upper-envelope tyre fit",
}


def steps_in_words(steps):
    # titles in plain words (render title rule); the JSON keeps the codes
    return "; ".join(STEP_WORDS.get(x, x.replace("EKF Q q_beta", "EKF sideslip process noise")) for x in steps)


# ---------------------------------------------------------------- P6

def p6(key):
    from modules.stability_analysis import estimate_sideslip
    ctx = Ctx(key)
    G_ = _load("gate.json")
    p5r = _load(f"p5_{key}.json")["rows"]["production Q"]
    out = {"key": key, "session": ctx.name, "stacks": {}}
    prod_cs = None
    for which in ("qualified", "everything_on"):
        s = G_["stacks"][which]
        st = ctx.state
        steps = []
        if s["despike"]:
            st, _sh = despike_state(st)
            steps.append("despike")
        pp = ctx.params
        if s["filters"]:
            spec = measure_spectra(ctx)
            short = s["filters"][3:]
            st = filtered_state(ctx, st, spec, lag=(short == "p4c"))
            if short == "p4b":
                pp = copy.deepcopy(ctx.params)
                pp["stability_estimation"]["cs_filter_cutoff_hz"] = float(spec["f_acc_hz"])
            steps.append(f"filters {short}")
        if s["seed"]:
            b, _l = anchored_seed(ctx, st, s["seed"][3:])
            steps.append(f"seed {s['seed'][3:]}")
        else:
            b = estimate_sideslip(st, ctx.params)
        lm = None
        if s["population"]:
            ls = lap_sets(ctx)
            lset = "fast101" if s["population"] == "P3 fast101" else "fast_half"
            if ls[lset] is not None:
                lm = ctx.lap_mask(set(ls[lset]))
                steps.append(f"population {lset}")
        loss = s["loss"][8:] if s["loss"] else ("huber" if s["population"] in ("P3 fast_huber", "P3 fast_huber_env")
                                                else "ls")
        if loss != "ls":
            steps.append(f"loss {loss}")
        if s["envelope"]:
            steps.append("upper envelope")
        q = (1.0, 1.0)
        if s["q"]:
            q = {r[0]: r[1:] for r in Q_ROWS}[s["q"][3:]]
            steps.append(f"EKF Q {s['q'][3:]}")
        if not steps:
            out["stacks"][which] = {"steps": [], "note": "no element qualified -> production; see the P5 production-Q chain"}
            _log(f"[{key}] P6 {which}: empty stack = production")
            continue
        fitter = make_fitter(loss=loss, lap_mask=lm, envelope=s["envelope"])
        slip_p = pp if pp is not ctx.params else None
        beta, rec, dn = run_chain(ctx, st, b, fitter=fitter, q=q, slip_params=slip_p)
        chain_metrics(ctx, beta, rec, bound=p5r["m3c_bound"])
        rec["steps"] = steps
        rec["vs_production"] = {"chatter_production": p5r["chatter_rms_deg"], "gate_production": p5r["gate"],
                                "health_production": p5r["health"]}
        out["stacks"][which] = rec
        words = "qualified stack (only elements that passed their gate)" if which == "qualified" else \
            "everything-on stack (best row of every part, regardless of gate)"
        render_cs_bridged(os.path.join(OUT_DIR, f"p6_cs_{key}_{which}.png"),
                          f"{ctx.name} -- CS ratio, {words}: {steps_in_words(steps)}", ctx, st, dn["cs_f"], dn["cs_r"])
        render_beta(os.path.join(OUT_DIR, f"p6_beta_{key}_{which}.png"),
                    f"{ctx.name} -- sideslip beta, {words}", ctx,
                    [(f"stack seed (before the EKF): {steps_in_words(steps)}", b, REF),
                     (f"stack EKF beta (gate {rec['gate']}, chatter {rec['chatter_rms_deg']:.2f} deg rms)", beta, INK)])
        _log(f"[{key}] P6 {which}: steps {steps}; status {rec['status']} gate {rec['gate']} health {rec['health']} "
             f"fb {rec['fallback']}; chatter {rec['chatter_rms_deg']:.3f} (production {p5r['chatter_rms_deg']:.3f}); "
             f"M3 {rec['m3']:.3f} degraded {rec.get('m3_degraded')}; axles {rec['axles']}")
        _dump(f"p6_{key}.json", out)
    _dump(f"p6_{key}.json", out)


# ---------------------------------------------------------------- main

if __name__ == "__main__":
    mode = sys.argv[1]
    if mode in ("p1", "p2", "p3", "p4", "p5", "p6"):
        globals()[mode](sys.argv[2])
    elif mode == "gate":
        gate()
    elif mode == "esc":
        esc(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit(f"unknown mode {mode}")
