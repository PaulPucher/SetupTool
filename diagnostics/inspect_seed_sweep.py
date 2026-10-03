# Seed quality sweep (PLAN.md PHASE 1 PLAN Q-SEED; read-only, nothing in
# modules/ or config/ changes; variants live in memory only).
#
# QUESTION: Q2d located the fit's damage in the kinematic seed (bands of
# +/-10-15 kN Fy at near-zero kinematic alpha). Does seed CONSTRUCTION -- not
# new information -- clean the fit population? No curve-truth claims: this
# attacks seed error, not the curve/beta observability wall.
#
# RULES, frozen 2026-10-02 before any run (author rulings in PLAN.md Q-SEED).
# Identity (Tier A anchor, unchanged): beta_dot = a_y / v - r (Rajamani ch. 2),
#   as estimate_sideslip. Everything varied here is Tier B preprocessing of
#   that identity's input or integration: bias removal, segment-wise
#   integration with boundary conditions, filter cutoffs. No literature claim
#   beyond standard drift closure.
# Sessions: Tier 1 pr_us_v2, v3, nbr_jst_solution (JST 29). All-18 = the
#   serial-221 pool of inspect_tyre_prior_d2.pool_keys(). Inputs: the (d2)
#   data caches, state built by prepare_vehicle_state with the (d2) params
#   (cap 1); fit population = production _base_mask; Fy = estimate_lateral_
#   forces (unchanged by every variant except S4l).
# OFFSETS (measured in-run for every session by the (a) method; ruling 3):
#   yaw bias b_r = median over analysis laps of the closure excess E/T
#     (E = integral of r over the lap minus 360 deg x sign; T = lap time);
#   ay offset, two definitions: S0-median = median a_y on the (a) straight
#     population S0 (racing, outside every corner bracket); referee intercept
#     = intercept of the (a2) OLS a_y = s (r v) + i on the (a2) quasi-steady
#     cornering population (2 Hz zero-phase low-pass, |psi_dd| < 0.15,
#     |a_x| < 2, clip rule), log_acc_y.
#   ENTRY CHECK: on the sessions with a record (report.json, a2_referee.json)
#     the recomputed values must match the record to 1e-6 absolute; a mismatch
#     is reported and the fresh value is used (it is the rule-3 measurement).
# VARIANTS (all from the same state; beta on non-moving samples = 0):
#   S0   production estimate_sideslip (washout high-pass beta_washout_cutoff_hz
#        from config after integration). ENTRY CHECK: alpha_f/r_filt and
#        Fy_f/r_filt bit-identical to the (d2) fit cache; S0 refit B, C, D, E
#        equal the cached production manifest to 1e-9 relative.
#   S1a  yaw bias b_r and ay S0-median removed from r and a_y BEFORE
#        integration, production washout unchanged.
#   S1b  as S1a with the ay referee intercept.
#        The correction enters the integration input only; slip-angle geometry
#        keeps the logged r (the b_r term there is b_r a / v ~ 0.01 deg,
#        negligible) and Fy keeps the logged a_y.
#   S2   segment integration, NO high-pass. Straights = the (a) S0 population;
#        beta = 0 there. A segment = a maximal run of moving non-S0 samples.
#        Inside it beta starts at 0 at the segment entry and integrates
#        beta_dot. Endpoint rule: if the sample after the segment is an S0
#        sample, beta at that sample must be 0 -> the closing error e is
#        removed by a linear ramp e (k+1)/n over the segment's n samples
#        (always applied; every non-zero e is a mismatch). Open-ended segments
#        (no S0 sample after: leaving the racing population) are not detrended.
#   S3a  S1a + S2.   S3b  S1b + S2.
#   S4w  S0 with the washout cutoff x0.5 / x2 (S4w_0.025, S4w_0.1 Hz).
#   S4l  S0 with the alpha/Fy low-pass (cs_filter_cutoff_hz) x0.5 / x2
#        (S4l_1, S4l_4 Hz), applied to the fit population's alpha AND Fy.
#   S4 sub-variants are uncrossed (ruling 1).
# METRICS (alpha-robust, no curve truth; every baseline from S0 in this run):
#   M1 per axle on the fit population with the variant's own alpha: Fy in the
#      bin |alpha| < 0.5 deg; spread p95 - p5 (primary) and IQR; shrink =
#      1 - spread_v / spread_S0.
#   M2 beta on S0 straights: median, |median|, p95 - p5 (deg). S2/S3 are 0
#      there BY CONSTRUCTION -- reported as built in, not as a result.
#   M3 cross-lap coherence: per stable corner, each analysis lap's beta
#      interpolated on 100 points between the corner's median bracket start
#      and end (lap distance); SD across laps per point (>= 2 laps), median
#      over points; session value = median over corners (deg). S0 bound =
#      97.5th percentile of a 200-draw lap bootstrap (laps resampled with
#      replacement, seed fixed). DEGRADED iff variant value > S0 bound.
#   M4 refit: production _fit_axle_pacejka on the variant population; B, C,
#      D, E, convergence, peak, D change and curve RMS vs the S0 refit on 200
#      points of [0, S0 p99 |alpha|] as % of S0 D. Report only.
#   M5 renders (below); the author's eyes are the ruled verification method.
# GATE (escalation only, never cancellation; standing instruction):
#   M1 met on a session iff the p95 - p5 shrink is >= 30 % on at least one
#   axle [op: per-axle reporting, the exhibit bands sit on both axles].
#   PROMISING iff M1 met on >= 2 of 3 Tier-1 sessions AND M3 not degraded on
#   any Tier-1 session AND the render inspection finds no new artefacts (that
#   leg is read by the assistant before escalation, labelled, and the author
#   rules). Promising -> all-18 pass (metrics + beta/cloud renders) and the
#   downstream chain on Tier 1: the variant seed replaces estimate_sideslip
#   inside modules.tyre_fit_auto (fit population, kinematic fallback), the
#   production fit -> EKF -> gate runs unchanged, CS traces rendered. For
#   S4l the variant cutoff applies inside tyre_fit_auto only; the CS chain
#   keeps production filtering. S0 downstream is rendered fresh as baseline.
#   Non-promising variants stop at fit population + refit and keep their
#   beta and cloud renders. HARD STOP after.
# POST-HOC (added after the Tier-1 run, before any escalation; labelled in
#   every output; the pre-registered numbers stand as computed):
#   P1 M1 DILUTION: S2/S3 (and S4w_0.1) set beta to ~0 on straights, so
#      straight samples with Fy ~ 0 enter the |alpha| < 0.5 deg bin (rear bin
#      share 0.28-0.31 -> 0.48-0.56) and shrink p95 - p5 by construction.
#      M1c = M1 on fit-population samples INSIDE corner brackets only.
#   P2 M3 BOUND BIAS: a lap bootstrap with replacement duplicates laps and
#      biases the cross-lap SD low (JST 29: S0 0.401 above its own bound
#      0.388). M3c bound = S0 value + (p97.5 - median) of the same bootstrap.
#   Gate legs re-read with M1c and M3c, beside the pre-registered read.
# Outputs diagnostics/plots/seed_sweep/: offsets_<key>.json, sweep_<key>.json,
#   beta_<key>_<variant>.png, cloud_<key>_<variant>_<axle>.png,
#   cs_<key>_<variant>.png, seed_sweep.txt, seed_sweep.json, run_log.txt.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_seed_sweep sweep <key> [variant,...]
#   python -m diagnostics.inspect_seed_sweep down <key> <variant>
#   python -m diagnostics.inspect_seed_sweep report

import copy
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_imu_offsets as imu

OUT_DIR = "diagnostics/plots/seed_sweep"
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
VARIANTS = ("S0", "S1a", "S1b", "S2", "S3a", "S3b", "S4w_0.025", "S4w_0.1", "S4l_1", "S4l_4")
BAND_DEG = 0.5
SHRINK_GATE = 0.30
N_GRID_M3 = 100
N_BOOT_M3 = 200
ENTRY_TOL_ABS = 1e-6
RAD2DEG = 180.0 / np.pi
INK, BAND, REF, ACC, HL = "#1f2937", "#e5e7eb", "#9ca3af", "#2563eb", "#fde68a"


def _dump(path, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(obj, fh, indent=1, default=float)


# ---------------------------------------------------------------- offsets

def measure_offsets(key, state, data):
    sr = state["sample_rate_hz"]
    t = state["time"]
    racing, thr, s0, _s1, inst = imu.populations(state, data["laps"], data["corners"])
    r_deg = state["yaw_rate_radps"] * RAD2DEG
    per_lap = []
    for l in imu.analysis_laps(data["laps"]):
        m = imu.lap_mask(t, l)
        H = float(np.sum(r_deg[m]) / sr)
        per_lap.append((H - 360.0 * np.sign(H)) / float(m.sum() / sr))
    b_r = float(np.median(per_lap))
    ay_s0 = float(np.median(state["ay_mps2"][s0]))
    pseudo = {"sample_rate_hz": sr,
              "cs_filter_cutoff_hz": d2._params()["stability_estimation"]["cs_filter_cutoff_hz"],
              "corners": data["corners"], "raw": {}}
    stdy = imu._steady(pseudo, state, racing, thr, inst)
    pop = stdy["pop"]
    X = np.column_stack([stdy["rv"][pop], np.ones(int(pop.sum()))])
    coef, *_ = np.linalg.lstsq(X, stdy["alog"][pop], rcond=None)
    out = {"b_r_degps": b_r, "b_r_per_lap": per_lap, "ay_s0_median": ay_s0, "ay_referee_intercept": float(coef[1]),
           "referee_slope": float(coef[0]), "referee_n": int(pop.sum()), "n_s0": int(s0.sum())}
    entry = {}
    try:
        with open(os.path.join(imu.OUT_DIR, "report.json")) as fh:
            rep = json.load(fh)
        with open(os.path.join(imu.OUT_DIR, "a2_referee.json")) as fh:
            ref = json.load(fh)
        if key in rep:
            rec = {"b_r_degps": float(np.median([x["bias_if_no_scale_degps"] for x in rep[key]["p2"]["laps"]])),
                   "ay_s0_median": rep[key]["p1"]["ay"]["S0"]["median"],
                   "ay_referee_intercept": ref[key]["fits"]["log_acc_y"]["all"]["i"]}
            for k, v in rec.items():
                entry[k] = {"record": v, "fresh": out[k], "diff": out[k] - v,
                            "match": bool(abs(out[k] - v) <= ENTRY_TOL_ABS)}
    except FileNotFoundError:
        pass
    out["entry_check"] = entry or "no record for this session"
    return out, s0, inst


# ---------------------------------------------------------------- seeds

def _shift_state(state, b_r_degps, b_ay):
    st = dict(state)
    st["yaw_rate_radps"] = state["yaw_rate_radps"] - b_r_degps / RAD2DEG
    st["ay_mps2"] = state["ay_mps2"] - b_ay
    return st


def segment_beta(state, s0):
    v, moving = state["v_mps"], state["moving_mask"]
    v_safe = np.where(moving, v, 1.0)
    bd = np.where(moving, state["ay_mps2"] / v_safe - state["yaw_rate_radps"], 0.0)
    dt = 1.0 / state["sample_rate_hz"]
    beta = np.zeros_like(bd)
    seg = moving & ~s0
    idx = np.where(seg)[0]
    n_closed = n_open = 0
    closes = []
    if idx.size:
        for p in np.split(idx, np.where(np.diff(idx) != 1)[0] + 1):
            inc = bd[p] * dt
            b = np.concatenate(([0.0], np.cumsum(inc[:-1])))
            nxt = p[-1] + 1
            if nxt < s0.size and s0[nxt]:
                e = b[-1] + inc[-1]
                b = b - e * (np.arange(p.size) + 1) / p.size
                n_closed += 1
                closes.append(e * RAD2DEG)
            else:
                n_open += 1
            beta[p] = b
    ce = np.abs(closes) if closes else np.array([np.nan])
    info = {"n_closed": n_closed, "n_open": n_open, "closing_error_abs_deg_median": float(np.nanmedian(ce)),
            "closing_error_abs_deg_p90": float(np.nanpercentile(ce, 90))}
    return np.where(moving, beta, 0.0), info


def variant_seed(name, state, params, s0, off):
    """beta, params used for alpha/Fy filtering, info."""
    from modules.stability_analysis import estimate_sideslip
    p = params
    info = {}
    if name == "S0":
        beta = estimate_sideslip(state, params)
    elif name in ("S1a", "S1b"):
        b_ay = off["ay_s0_median"] if name == "S1a" else off["ay_referee_intercept"]
        beta = estimate_sideslip(_shift_state(state, off["b_r_degps"], b_ay), params)
        info = {"b_r_degps": off["b_r_degps"], "b_ay": b_ay}
    elif name == "S2":
        beta, info = segment_beta(state, s0)
    elif name in ("S3a", "S3b"):
        b_ay = off["ay_s0_median"] if name == "S3a" else off["ay_referee_intercept"]
        beta, info = segment_beta(_shift_state(state, off["b_r_degps"], b_ay), s0)
        info.update({"b_r_degps": off["b_r_degps"], "b_ay": b_ay})
    elif name.startswith("S4w_"):
        p = copy.deepcopy(params)
        p["stability_estimation"]["beta_washout_cutoff_hz"] = float(name.split("_")[1])
        beta = estimate_sideslip(state, p)
        p = params  # washout only; alpha/Fy filtering stays production
        info = {"beta_washout_cutoff_hz": float(name.split("_")[1])}
    elif name.startswith("S4l_"):
        p = copy.deepcopy(params)
        p["stability_estimation"]["cs_filter_cutoff_hz"] = float(name.split("_")[1])
        beta = estimate_sideslip(state, params)
        info = {"cs_filter_cutoff_hz": float(name.split("_")[1])}
    else:
        raise ValueError(name)
    return beta, p, info


# ---------------------------------------------------------------- metrics

def m1(alpha, fy, base):
    out = {}
    m = base & np.isfinite(alpha) & np.isfinite(fy) & (np.abs(alpha) * RAD2DEG < BAND_DEG)
    y = fy[m]
    out["n"] = int(m.sum())
    out["share_of_pop"] = float(m.sum() / max(1, int((base & np.isfinite(alpha)).sum())))
    if y.size >= 20:
        q = np.percentile(y, [5, 25, 75, 95])
        out.update({"p95_p5_N": float(q[3] - q[0]), "iqr_N": float(q[2] - q[1])})
    return out


def m2(beta, s0):
    b = beta[s0] * RAD2DEG
    q = np.percentile(b, [5, 95])
    return {"median_deg": float(np.median(b)), "abs_median_deg": float(abs(np.median(b))),
            "p95_p5_deg": float(q[1] - q[0])}


def corner_curves(beta, state, data, inst):
    """{sid: {lap: curve on the corner's grid}} in deg."""
    s_m = state["s_m"]
    by = {}
    for sid, lap, sl in inst:
        by.setdefault(sid, []).append((lap, sl))
    br = {}
    for c in data["corners"]:
        if c.get("stable_corner_id") is not None and c.get("bracket_start_m") is not None:
            br.setdefault(c["stable_corner_id"], []).append((c["bracket_start_m"], c["bracket_end_m"]))
    out = {}
    for sid, rows in by.items():
        b0 = float(np.median([x[0] for x in br[sid]]))
        b1 = float(np.median([x[1] for x in br[sid]]))
        g = np.linspace(b0, b1, N_GRID_M3)
        curves = {}
        for lap, sl in rows:
            s, y = s_m[sl], beta[sl] * RAD2DEG
            ok = np.isfinite(s) & np.isfinite(y)
            if ok.sum() < 5:
                continue
            s, y = s[ok], y[ok]
            o = np.argsort(s)
            s, y = s[o], y[o]
            c = np.interp(g, s, y, left=np.nan, right=np.nan)
            curves[lap] = c
        if len(curves) >= 2:
            out[sid] = curves
    return out


def m3_value(curves, laps_pick=None):
    vals = []
    for sid, cv in curves.items():
        keys = list(cv) if laps_pick is None else [l for l in laps_pick if l in cv]
        if len(keys) < 2:
            continue
        A = np.array([cv[l] for l in keys])
        cnt = np.sum(np.isfinite(A), axis=0)
        with np.errstate(all="ignore"):
            sd = np.nanstd(A, axis=0, ddof=1)
        sd = sd[cnt >= 2]
        if sd.size:
            vals.append(float(np.nanmedian(sd)))
    return float(np.median(vals)) if vals else float("nan"), len(vals)


def m3_bound(curves):
    laps = sorted({l for cv in curves.values() for l in cv})
    rng = np.random.default_rng(20261002)
    bs = []
    for _ in range(N_BOOT_M3):
        pick = list(rng.choice(laps, size=len(laps), replace=True))
        v, _n = m3_value(curves, pick)
        if np.isfinite(v):
            bs.append(v)
    if not bs:  # one analysis lap: no cross-lap spread exists
        return float("nan"), float("nan")
    return float(np.percentile(bs, 97.5)), float(np.percentile(bs, 2.5))


def refit(alpha, fy, base):
    from modules.tyre_fit_auto import _fit_axle_pacejka
    r = _fit_axle_pacejka(alpha, fy, base)
    return {k: r[k] for k in ("B", "C", "D", "E", "powell_converged", "fit_n_samples", "fit_rms_resid_N",
                              "peak_alpha_deg", "peak_in_visited_range", "visited_alpha_p99_deg")}


def curve_delta(p, p0, p99_deg):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    g = np.radians(np.linspace(0, p99_deg, 200))
    f = pacejka_lateral_force(g, p["B"], p["C"], p["D"], p["E"])
    f0 = pacejka_lateral_force(g, p0["B"], p0["C"], p0["D"], p0["E"])
    return {"curve_rms_pct_D": float(100 * np.sqrt(np.mean((f - f0) ** 2)) / p0["D"]),
            "dD_pct": float(100 * (p["D"] / p0["D"] - 1))}


# ---------------------------------------------------------------- renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _lap_len(state, laps):
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    m = (state["time"] >= an[0]["start_time"]) & (state["time"] <= an[0]["end_time"])
    return float(np.nanmax(state["s_m"][m]))


def render_beta(path, key, name, state, data, beta, beta0):
    plt = _plt()
    t, s_m = state["time"], state["s_m"]
    an = [l for l in data["laps"] if l.get("is_valid_for_analysis")]
    an_nums = {l["lap_number"] for l in an}
    brackets = sorted({(c["bracket_start_m"], c["bracket_end_m"]) for c in data["corners"]
                       if c.get("bracket_start_m") is not None and c["lap_number"] in an_nums})
    rows = [("S0 (production seed, fresh)", beta0)] + ([] if name == "S0" else [(name, beta)])
    vals = np.concatenate([b[(t >= an[0]["start_time"]) & (t <= an[-1]["end_time"])] for _, b in rows]) * RAD2DEG
    lim = max(2.0, float(np.nanpercentile(np.abs(vals), 99.9)) * 1.1)
    fig, axes = plt.subplots(len(rows), 1, figsize=(max(14, _lap_len(state, data["laps"]) / 400), 3.2 * len(rows)),
                             sharex=True, squeeze=False, constrained_layout=True)
    for ax, (lab, b) in zip(axes[:, 0], rows):
        for b0, b1 in brackets:
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8, zorder=1)
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s_m[m], b[m] * RAD2DEG, color=INK if lab == name else REF if name != "S0" else INK,
                    lw=0.5, alpha=0.7, zorder=2)
        ax.set_ylim(-lim, lim)
        ax.set_ylabel("beta (deg)", color=INK)
        ax.set_title(f"{key}: {lab} -- all analysis laps, grey bands = corner brackets", fontsize=9, color=INK,
                     loc="left")
        ax.grid(axis="y", color=BAND, lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[-1, 0].set_xlabel("lap distance (m)", color=INK)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def render_cloud(path, key, name, axle, alpha, fy, base, p, p0, met):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    plt = _plt()
    m = base & np.isfinite(alpha) & np.isfinite(fy)
    a, f = alpha[m] * RAD2DEG, fy[m] / 1000.0
    rng = np.random.default_rng(0)
    if a.size > 40000:
        i = rng.choice(a.size, 40000, replace=False)
        a, f = a[i], f[i]
    fig, ax = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    ax.axvspan(-BAND_DEG, BAND_DEG, color=HL, lw=0, zorder=0, label=f"M1 bin |alpha| < {BAND_DEG} deg")
    ax.scatter(a, f, s=1.2, color=INK, alpha=0.12, lw=0, zorder=1)
    lim = float(np.nanpercentile(np.abs(a), 99.5)) * 1.1
    g = np.linspace(-lim, lim, 400)
    if p0 is not None and np.isfinite(p0["D"]):
        ax.plot(g, pacejka_lateral_force(np.radians(g), p0["B"], p0["C"], p0["D"], p0["E"]) / 1000.0,
                color=REF, lw=2, ls="--", zorder=2, label="S0 refit")
    if name != "S0" and np.isfinite(p["D"]):
        ax.plot(g, pacejka_lateral_force(np.radians(g), p["B"], p["C"], p["D"], p["E"]) / 1000.0,
                color=ACC, lw=2, zorder=3, label=f"{name} refit")
    ax.set_xlim(-lim, lim)
    ax.set_xlabel(f"alpha_{axle[0]} filtered (deg)", color=INK)
    ax.set_ylabel(f"Fy_{axle[0]} filtered (kN)", color=INK)
    txt = (f"M1 p95-p5 {met.get('p95_p5_N', float('nan')) / 1000:.2f} kN, IQR {met.get('iqr_N', float('nan')) / 1000:.2f}"
           f" kN, n {met['n']}")
    ax.set_title(f"{key} {axle}: {name} fit population -- {txt}", fontsize=9, color=INK, loc="left")
    ax.grid(color=BAND, lw=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(loc="lower right", frameon=False, fontsize=8)
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- modes

def _setup(key):
    from modules.stability_analysis import prepare_vehicle_state
    from modules.tyre_fit_auto import _base_mask
    params = d2._params()
    data = d2.load_data(key)
    state = prepare_vehicle_state(data["channels"], params)
    return params, data, state, _base_mask(state, data["laps"])


def sweep(key, names=VARIANTS):
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
    params, data, state, base = _setup(key)
    off, s0, inst = measure_offsets(key, state, data)
    _dump(os.path.join(OUT_DIR, f"offsets_{key}.json"), off)
    print(f"[{key}] offsets b_r {off['b_r_degps']:+.4f} deg/s, ay S0 {off['ay_s0_median']:+.4f}, "
          f"referee i {off['ay_referee_intercept']:+.4f} m/s^2; entry {json.dumps(off['entry_check'], default=float)}",
          flush=True)
    out = {"key": key, "offsets": off, "variants": {}}
    cache = None
    try:
        cache = d2.load_fit(key)
    except FileNotFoundError:
        pass
    beta0 = None
    base0 = {}
    for name in ("S0",) + tuple(n for n in names if n != "S0"):
        beta, p, info = variant_seed(name, state, params, s0, off)
        slip = estimate_slip_angles(state, beta, p)
        forces = estimate_lateral_forces(state, p)
        rec = {"info": info}
        if name == "S0":
            beta0 = beta
            if cache is not None:
                rec["entry_arrays_identical"] = {k: bool(np.array_equal(a, b)) for k, a, b in (
                    ("alpha_f", slip["alpha_f_filt"], cache["alpha_f"]), ("alpha_r", slip["alpha_r_filt"], cache["alpha_r"]),
                    ("fy_f", forces["Fy_f_filt"], cache["fy_f"]), ("fy_r", forces["Fy_r_filt"], cache["fy_r"]),
                    ("base_mask", base, cache["base_mask"]))}
        rec["m2"] = m2(beta, s0)
        curves = corner_curves(beta, state, data, inst)
        rec["m3"], rec["m3_n_corners"] = m3_value(curves)
        if name == "S0":
            rec["m3_bound_975"], rec["m3_bound_025"] = m3_bound(curves)
            base0["m3_bound"] = rec["m3_bound_975"]
        for axle, a, f in (("front", slip["alpha_f_filt"], forces["Fy_f_filt"]),
                           ("rear", slip["alpha_r_filt"], forces["Fy_r_filt"])):
            met = m1(a, f, base)
            fit_ = refit(a, f, base)
            ar = {"m1": met, "refit": fit_}
            if name == "S0":
                base0[axle] = {"m1": met, "refit": fit_}
                if cache is not None:
                    man = cache["prod"]["manifest"]["axles"][axle]
                    ar["entry_refit_max_rel"] = max(abs(fit_[k] / man[k] - 1) for k in "BCDE")
            else:
                b0 = base0[axle]
                ar["m1_shrink_p95_p5"] = 1 - met["p95_p5_N"] / b0["m1"]["p95_p5_N"]
                ar["m1_shrink_iqr"] = 1 - met["iqr_N"] / b0["m1"]["iqr_N"]
                ar["m4_vs_S0"] = curve_delta(fit_, b0["refit"], b0["refit"]["visited_alpha_p99_deg"])
            rec[axle] = ar
            render_cloud(os.path.join(OUT_DIR, f"cloud_{key}_{name}_{axle}.png"), key, name, axle, a, f, base,
                         fit_, base0[axle]["refit"], met)
        if name != "S0":
            rec["m1_met"] = bool(max(rec["front"]["m1_shrink_p95_p5"], rec["rear"]["m1_shrink_p95_p5"]) >= SHRINK_GATE)
            rec["m3_degraded"] = (bool(rec["m3"] > base0["m3_bound"]) if np.isfinite(base0["m3_bound"])
                                  else None)  # not assessable
        render_beta(os.path.join(OUT_DIR, f"beta_{key}_{name}.png"), key, name, state, data, beta, beta0)
        out["variants"][name] = rec
        print(f"[{key}] {name}: M1 p95-p5 f {rec['front']['m1'].get('p95_p5_N', float('nan')):.0f} "
              f"r {rec['rear']['m1'].get('p95_p5_N', float('nan')):.0f} N; M3 {rec['m3']:.3f} deg; M2 |med| "
              f"{rec['m2']['abs_median_deg']:.3f} spread {rec['m2']['p95_p5_deg']:.3f}; "
              f"refit D f {rec['front']['refit']['D']:.0f} r {rec['rear']['refit']['D']:.0f}"
              + (f"; shrink f {rec['front']['m1_shrink_p95_p5']:+.3f} r {rec['rear']['m1_shrink_p95_p5']:+.3f}; "
                 f"m1_met {rec['m1_met']} m3_degraded {rec['m3_degraded']}" if name != "S0" else
                 f"; entry {rec.get('entry_arrays_identical')} refit rel f {rec['front'].get('entry_refit_max_rel')} "
                 f"r {rec['rear'].get('entry_refit_max_rel')}; M3 bound {rec['m3_bound_975']:.3f}"), flush=True)
    path = os.path.join(OUT_DIR, f"sweep_{key}.json")
    if os.path.exists(path) and tuple(names) != VARIANTS:
        with open(path) as fh:
            prev = json.load(fh)
        prev["variants"].update(out["variants"])
        prev["offsets"] = off
        out = prev
    _dump(path, out)


def down(key, name):
    """Downstream chain on a variant seed: fit -> EKF -> gate -> CS traces."""
    import modules.tyre_fit_auto as tfa
    from diagnostics.trace_render import render_cs_traces
    params, data, state, base = _setup(key)
    off, s0, _inst = measure_offsets(key, state, data)
    beta_seed, p, info = variant_seed(name, state, params, s0, off)
    orig = (tfa.estimate_sideslip, tfa.estimate_slip_angles, tfa.estimate_lateral_forces)
    tfa.estimate_sideslip = lambda st, pr: beta_seed
    if name.startswith("S4l_"):
        tfa.estimate_slip_angles = lambda st, b, pr: orig[1](st, b, p)
        tfa.estimate_lateral_forces = lambda st, pr: orig[2](st, p)
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka",
                                                            csv_path=d2.sessions()[key]["path"])
    finally:
        tfa.estimate_sideslip, tfa.estimate_slip_angles, tfa.estimate_lateral_forces = orig
    dn = d2._downstream(params, data, state, beta)
    clipped = render_cs_traces(os.path.join(OUT_DIR, f"cs_{key}_{name}.png"), state["time"], state["s_m"],
                               dn["cs_f"], dn["cs_r"], data["laps"], data["corners"],
                               f"{key}: CS_ratio, seed {name} -> production fit/EKF/gate")
    ax = (man or {}).get("axles", {})
    rec = {"status": (man or {}).get("status"), "gate": gate.get("verdict") if gate else None,
           "health": gate.get("health_score") if gate else None, "fallback": fb, "fallback_reason": fr,
           "axles": {a: {k: ax[a][k] for k in ("B", "C", "D", "E", "powell_converged")} for a in ax},
           "clipped": clipped}
    path = os.path.join(OUT_DIR, f"down_{key}.json")
    prev = json.load(open(path)) if os.path.exists(path) else {}
    prev[name] = rec
    _dump(path, prev)
    print(f"[{key}] down {name}: {json.dumps(rec, default=float)}", flush=True)


def posthoc(key):
    """P1 / P2 (labelled post-hoc); no refits, alpha only."""
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces
    params, data, state, base = _setup(key)
    off, s0, inst = measure_offsets(key, state, data)
    inb = np.zeros(base.size, dtype=bool)
    for _sid, _lap, sl in inst:
        inb[sl] = True
    out = {}
    for name in VARIANTS:
        beta, p, _info = variant_seed(name, state, params, s0, off)
        slip = estimate_slip_angles(state, beta, p)
        forces = estimate_lateral_forces(state, p)
        rec = {"m1c": {ax: m1(slip[f"alpha_{ax[0]}_filt"], forces[f"Fy_{ax[0]}_filt"], base & inb)
                       for ax in ("front", "rear")}}
        if name == "S0":
            curves = corner_curves(beta, state, data, inst)
            v0, _n = m3_value(curves)
            laps = sorted({l for cv in curves.values() for l in cv})
            rng = np.random.default_rng(20261002)
            bs = []
            for _ in range(N_BOOT_M3):
                v, _n = m3_value(curves, list(rng.choice(laps, size=len(laps), replace=True)))
                if np.isfinite(v):
                    bs.append(v)
            rec["m3c_bound"] = float(v0 + np.percentile(bs, 97.5) - np.median(bs)) if bs else float("nan")
            rec["m3_boot_median"] = float(np.median(bs)) if bs else float("nan")
        out[name] = rec
    for name in VARIANTS[1:]:
        for ax in ("front", "rear"):
            a, b = out[name]["m1c"][ax], out["S0"]["m1c"][ax]
            a["shrink_p95_p5"] = 1 - a["p95_p5_N"] / b["p95_p5_N"]
            a["shrink_iqr"] = 1 - a["iqr_N"] / b["iqr_N"]
    _dump(os.path.join(OUT_DIR, f"posthoc_{key}.json"), out)
    print(f"[{key}] POST-HOC M3c bound {out['S0']['m3c_bound']:.3f} (boot median {out['S0']['m3_boot_median']:.3f})")
    for name, r in out.items():
        print(f"  {name:10s} M1c p95-p5 f {r['m1c']['front'].get('p95_p5_N', float('nan')):7.0f} r "
              f"{r['m1c']['rear'].get('p95_p5_N', float('nan')):7.0f} N; n f {r['m1c']['front']['n']} r "
              f"{r['m1c']['rear']['n']}, share r {r['m1c']['rear']['share_of_pop']:.3f}"
              + (f"; shrink f {r['m1c']['front']['shrink_p95_p5']:+.3f} r {r['m1c']['rear']['shrink_p95_p5']:+.3f}"
                 if name != "S0" else ""), flush=True)


def report():
    lines = []
    allr = {}
    for f in sorted(os.listdir(OUT_DIR)):
        if f.startswith("sweep_") and f.endswith(".json"):
            with open(os.path.join(OUT_DIR, f)) as fh:
                r = json.load(fh)
            allr[r["key"]] = r
    for key, r in allr.items():
        o = r["offsets"]
        lines.append(f"[{key}] b_r {o['b_r_degps']:+.4f} deg/s; ay S0 {o['ay_s0_median']:+.4f}; referee i "
                     f"{o['ay_referee_intercept']:+.4f} m/s^2; entry {json.dumps(o['entry_check'], default=float)}")
        for name, v in r["variants"].items():
            fr, rr = v["front"], v["rear"]
            s = (f"  {name:10s} M1 p95-p5 f {fr['m1'].get('p95_p5_N', float('nan')):7.0f} r "
                 f"{rr['m1'].get('p95_p5_N', float('nan')):7.0f} N (IQR f {fr['m1'].get('iqr_N', float('nan')):6.0f} r "
                 f"{rr['m1'].get('iqr_N', float('nan')):6.0f}; n f {fr['m1']['n']} r {rr['m1']['n']}); M2 med "
                 f"{v['m2']['median_deg']:+.3f} spread {v['m2']['p95_p5_deg']:.3f}; M3 {v['m3']:.3f} "
                 f"({v['m3_n_corners']} c)")
            if name == "S0":
                s += (f"; M3 bound {v['m3_bound_975']:.3f}; entry {v.get('entry_arrays_identical')} refit rel f "
                      f"{fr.get('entry_refit_max_rel')} r {rr.get('entry_refit_max_rel')}")
            else:
                s += (f"; shrink p95-p5 f {fr['m1_shrink_p95_p5']:+.3f} r {rr['m1_shrink_p95_p5']:+.3f}, IQR f "
                      f"{fr['m1_shrink_iqr']:+.3f} r {rr['m1_shrink_iqr']:+.3f}; M4 f dD {fr['m4_vs_S0']['dD_pct']:+.1f} % "
                      f"rms {fr['m4_vs_S0']['curve_rms_pct_D']:.1f} %, r dD {rr['m4_vs_S0']['dD_pct']:+.1f} % rms "
                      f"{rr['m4_vs_S0']['curve_rms_pct_D']:.1f} %; conv f {fr['refit']['powell_converged']} r "
                      f"{rr['refit']['powell_converged']}; M1 met {v['m1_met']}, M3 degraded {v['m3_degraded']}")
            if v.get("info"):
                s += f"; info {json.dumps(v['info'], default=float)}"
            lines.append(s)
    lines.append("\nGATE (numeric legs, Tier 1; render leg read separately):")
    for name in VARIANTS[1:]:
        t1 = [allr[k]["variants"].get(name) for k in TIER1 if k in allr]
        t1 = [x for x in t1 if x]
        if not t1:
            continue
        nm = sum(x["m1_met"] for x in t1)
        nd = sum(x["m3_degraded"] for x in t1)
        lines.append(f"  {name}: M1 met {nm}/{len(t1)}, M3 degraded {nd}/{len(t1)} -> numeric legs "
                     f"{'PASS' if nm >= 2 and nd == 0 else 'FAIL'}")
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT_DIR, "seed_sweep.txt"), "w") as fh:
        fh.write(txt + "\n")
    _dump(os.path.join(OUT_DIR, "seed_sweep.json"), allr)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "sweep":
        sweep(sys.argv[2], tuple(sys.argv[3].split(",")) if len(sys.argv) > 3 else VARIANTS)
    elif mode == "posthoc":
        posthoc(sys.argv[2])
    elif mode == "down":
        down(sys.argv[2], sys.argv[3])
    elif mode == "report":
        report()
    else:
        raise SystemExit(f"unknown mode {mode}")
