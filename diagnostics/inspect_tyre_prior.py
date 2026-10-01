# DIAGNOSTIC, read-only. PLAN.md EXPERIMENT PLAN (d) step 0: one tyre-curve
# prior per axle fitted ONCE on pooled sessions, per-session deviation as a
# single force scale (Option B). Proposal, criteria and rulings:
# thesis_notes.md "(d) tyre-curve baseline prior: proposal, rulings,
# feasibility". Nothing in modules/ or config/ changes; Option B enters the
# production fit chain through an in-memory replacement of
# modules.tyre_fit_auto._fit_axle_pacejka, so steps (c)-(e) (R derivation,
# R sweep, final EKF, gate) run exactly as production runs them.
#
# Tier A (curve family, prior-plus-deviation): anchor PENDING -- nothing
# ships from this script. Tier B: pooling, closed-form LS scale, binning,
# bootstrap.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_tyre_prior compute <key>    production fit + fit arrays
#   python -m diagnostics.inspect_tyre_prior prior            pooled priors (main, LOSO, sensitivity)
#   python -m diagnostics.inspect_tyre_prior bcompute <key>   Option B through the production chain
#   python -m diagnostics.inspect_tyre_prior analyse          EC2, K1-K5
#   python -m diagnostics.inspect_tyre_prior score            K6 via the (b) battery scorer
#   python -m diagnostics.inspect_tyre_prior render           standard trace set, A vs PB
# Outputs in diagnostics/plots/tyre_prior/ (gitignored).
#
# FIXED BEFORE THE FIRST RUN (thresholds from the approved proposal; the
# operational details marked [op] were not spelled out there and are fixed
# here, before any number is seen):
# Fit population: production's own -- kinematic alpha_f/r_filt, Fy_f/r_filt,
#   _base_mask, finiteness as inside _fit_axle_pacejka.
# Prior: _fit_axle_pacejka, unchanged, on the concatenated pool. P221 = the
#   six serial-221 sessions (main). LOSO_<key> = P221 minus one. ALL8 and
#   EW221 (P221, each session weighted 1/n_s; own weighted Powell from the
#   same start guess) are sensitivity rows.
# Option B: Fy = lam * F_prior(alpha), lam = sum(F_prior Fy) / sum(F_prior^2).
# EC2: _fit_axle_pacejka on the cached arrays equals the production manifest's
#   B, C, D, E bit for bit; Dubai and v3 equal their records to the recorded
#   digits; health scores equal the new_laps cache values to 6 decimals; the
#   production beta equals the (b) cache's variant-A beta (float32). STOP on
#   any mismatch.
# K1: grid = 200 points on [0, pooled P221 p95 |alpha|] per axle [op].
#   LOSO prior within 3 % of D_P221 at every grid point; lam_s under LOSO_s
#   within 2 % of lam_s under P221 (relative). Independent-fit spread on the
#   same grid: context only.
# K2: C in (1, 2]; E <= 1; mu = D / mean axle Fz on the fit population in
#   [1.2, 2.0] (Fz measured where available, else the config-resolved
#   source, stated per session); BCD / mean Fz in [6, 24] rad^-1; |lam-1| <=
#   0.15, < 0.05 "not interpretable"; peak in/out of pooled p99.
# K3: region = |alpha_r| > median over the eight sessions of per-session p99
#   |alpha_r|. STABILISES iff prior rear peak <= pooled P221 p99 AND
#   max-min of the six LOSO rear peaks <= 0.5 deg AND no common-sign bias:
#   [op] per pool session the folded region residual mean (Fy - lam F_prior)
#   * sign(alpha) with bootstrap SE; common-sign bias = more than half of the
#   pool sessions beyond 2 SE with the same sign.
# K4: gate verdict, health score, sweep found_in_band, independent vs B;
#   worsening = verdict drops (pass > warn > fail) or found_in_band drops.
# K5 (i) [op] bins: 10 equal-count bins of |alpha| on [0, session p99];
#   folded residual mean per bin; SE from a block bootstrap (200 resamples)
#   over units = corner traversals plus the gaps between them. Pattern = >= 2
#   adjacent bins beyond 2 SE. Triggers if present for B and absent for the
#   independent fit on >= 2 sessions (per axle, either axle counts).
#   (ii) JST 21 vs JST 29, per axle: independent curves on the common grid
#   [0, min of the two sessions' p95 |alpha|], SE per grid point from 30
#   block-bootstrap Powell refits per session [op]; "differ" = max z > 2.
#   B: lam gap z = |dlam| / sqrt(se_a^2 + se_b^2) from the same resamples.
#   Masking iff independent differ and B z <= 2; uninformative if the
#   independent fits do not differ.
#   (iii) Dubai / Le Mans: |lam-1| > 0.15 or pattern (i) for B.
#   (iv) = K4 worsening on any session.
# K6: the (b) survival battery and scorer, unchanged, run on a copy of the
#   (b) caches with variant PB added: battery, A-vs-B resolvability, pooled
#   switch rate and floor on the five sessions that carry F1/F2 (Dubai, JST
#   21, JST 29, US-v2, v3); (b)'s confirm rows on Le Mans, CDR, MLA.
#   Render rule: standard trace set for A and PB on every session, inspected
#   before any score-based exit.

import copy
import gzip
import json
import os
import pickle
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import SESSIONS

OUT_DIR = "diagnostics/plots/tyre_prior"
CACHE = os.path.join(OUT_DIR, "cache")
B_CACHE = os.path.join(CACHE, "b")  # (b)-format caches for the K6 scorer
KEYS = [k for k in SESSIONS if k != "pr_us_v1"]
POOL = ["v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla"]
OUT_OF_POOL = ["dubai", "lmn_mla"]
B_CS_CACHE = "diagnostics/plots/cs_upstream/cache"
NEW_LAPS_CACHE = "diagnostics/plots/new_laps/cache"
AXLES = (("front", "alpha_f", "fy_f", "fz_f"), ("rear", "alpha_r", "fy_r", "fz_r"))

RECORDED = {  # thesis_notes.md, recorded digits
    "dubai": {"front": (10.457, 1.940, 8184.0, 0.091), "rear": (11.759, 1.982, 9213.6, -2.281)},
    "v3": {"front": (8.521, 1.888, 8355.4, -0.518), "rear": (8.371, 2.007, 9486.2, -1.381)},
}

GRID_N = 200
N_BOOT = 200
N_BOOT_REFIT = 30
SEED = 20261001


def _params():
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    return apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))


def _save(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with gzip.open(path, "wb") as fh:
        pickle.dump(obj, fh, protocol=pickle.HIGHEST_PROTOCOL)


def _load(path):
    with gzip.open(path, "rb") as fh:
        return pickle.load(fh)


def load(key):
    return _load(os.path.join(CACHE, f"{key}.pkl.gz"))


# ---------------------------------------------------------------- compute

def compute(key):
    """Production fit (independent, per session) plus the arrays it fits."""
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
        estimate_vertical_loads, load_car_data,
    )
    from modules.tyre_fit_auto import _base_mask, resolve_sideslip_beta
    params = _params()
    data = parse_csv(SESSIONS[key])
    st = prepare_vehicle_state(data["channels"], params)
    laps = data.get("laps", [])
    bm = _base_mask(st, laps)
    slip = estimate_slip_angles(st, estimate_sideslip(st, params), params)
    forces = estimate_lateral_forces(st, params)
    # measured Fz for the mu / BCD-per-load bands, forced as the mu fit forces it
    pm = dict(params)
    pm["stability_estimation"] = dict(params["stability_estimation"])
    pm["stability_estimation"]["vertical_load_source"] = "measured"
    fz = estimate_vertical_loads(st, forces, pm, channels=data["channels"], car_data=load_car_data())
    beta, man, gate, fb, fr = resolve_sideslip_beta(st, params, data, "ekf_auto_pacejka", csv_path=SESSIONS[key])
    bres = _load(os.path.join(B_CS_CACHE, f"{key}.pkl.gz"))
    out = {"key": key, "time": st["time"], "base_mask": bm,
           "alpha_f": slip["alpha_f_filt"], "alpha_r": slip["alpha_r_filt"],
           "fy_f": forces["Fy_f_filt"], "fy_r": forces["Fy_r_filt"],
           "fz_f": fz["fz_f_N"], "fz_r": fz["fz_r_N"], "fz_source": fz.get("vertical_load_source_used"),
           "beta_prod": beta, "manifest": man, "gate": gate, "fallback": fb, "fallback_reason": fr,
           "traversals": bres["traversals"], "laps": bres["laps"]}
    _save(os.path.join(CACHE, f"{key}.pkl.gz"), out)
    print(f"[{key}] status {man.get('status')}, gate {gate.get('verdict') if gate else None} "
          f"{gate.get('health_score') if gate else None}, fz source {out['fz_source']}", flush=True)


# ---------------------------------------------------------------- prior

def _pooled(keys, which):
    a, f, m = [], [], []
    for k in keys:
        r = load(k)
        a.append(r[f"alpha_{which}"]); f.append(r[f"fy_{which}"]); m.append(r["base_mask"])
    return np.concatenate(a), np.concatenate(f), np.concatenate(m)


def _fit_weighted(alpha, fy, w):
    """EW221 sensitivity only: production's SSE with per-sample weights, same
    start guess and optimiser."""
    from scipy.optimize import minimize
    from modules.tyre_fit_auto import PACEJKA_START_GUESS
    from modules.tyre_model_pacejka import pacejka_lateral_force

    def _sse(p):
        return float(np.sum(w * (pacejka_lateral_force(alpha, *p) - fy) ** 2))
    res = minimize(_sse, PACEJKA_START_GUESS, method="Powell")
    return {"B": float(res.x[0]), "C": float(res.x[1]), "D": float(res.x[2]), "E": float(res.x[3]),
            "powell_converged": bool(res.success), "fit_n_samples": int(alpha.size)}


def _peak_deg(B, C, D, E):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, 89.9, 4000)
    s = pacejka_lateral_stiffness(np.radians(g), B, C, D, E)
    i = np.where(np.diff(np.sign(s)) < 0)[0]
    return float(g[i[0]]) if i.size else float("nan")


def prior():
    import time
    from modules.tyre_fit_auto import _fit_axle_pacejka
    pools = {"P221": POOL, "ALL8": KEYS}
    pools.update({f"LOSO_{k}": [x for x in POOL if x != k] for k in POOL})
    out = {}
    for name, keys in list(pools.items()) + [("EW221", POOL)]:
        out[name] = {"keys": keys}
        for ax, which in (("front", "f"), ("rear", "r")):
            t0 = time.perf_counter()
            if name == "EW221":
                aa, ff, w = [], [], []
                for k in keys:
                    r = load(k)
                    m = r["base_mask"] & np.isfinite(r[f"alpha_{which}"]) & np.isfinite(r[f"fy_{which}"])
                    aa.append(r[f"alpha_{which}"][m]); ff.append(r[f"fy_{which}"][m]); w.append(np.full(m.sum(), 1.0 / m.sum()))
                fit = _fit_weighted(np.concatenate(aa), np.concatenate(ff), np.concatenate(w))
            else:
                a, f, m = _pooled(keys, which)
                fit = {k: v for k, v in _fit_axle_pacejka(a, f, m).items() if k not in ("residuals", "residual_mask")}
            fit["peak_alpha_deg"] = _peak_deg(fit["B"], fit["C"], fit["D"], fit["E"])
            out[name][ax] = fit
            print(f"{name:22s} {ax:5s} B={fit['B']:.4f} C={fit['C']:.4f} D={fit['D']:.1f} E={fit['E']:.4f} "
                  f"peak {fit['peak_alpha_deg']:.3f} deg, n={fit['fit_n_samples']}, converged {fit['powell_converged']}, "
                  f"{time.perf_counter() - t0:.0f}s", flush=True)
    with open(os.path.join(OUT_DIR, "priors.json"), "w") as fh:
        json.dump(out, fh, indent=1)


def priors():
    with open(os.path.join(OUT_DIR, "priors.json")) as fh:
        return json.load(fh)


def scale(prior_ax, alpha, fy, mask):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    m = mask & np.isfinite(alpha) & np.isfinite(fy)
    fp = pacejka_lateral_force(alpha[m], prior_ax["B"], prior_ax["C"], prior_ax["D"], prior_ax["E"])
    return float(np.sum(fp * fy[m]) / np.sum(fp ** 2)), m


# ---------------------------------------------------------------- Option B through production

def _option_b_fitter(prior_p):
    """Replacement for _fit_axle_pacejka: same signature and dict, called
    front then rear by fit_session_pacejka."""
    from modules.tyre_model_pacejka import pacejka_lateral_force
    order = iter(("front", "rear"))

    def fit(alpha, Fy, base_mask):
        p = prior_p[next(order)]
        lam, m2 = scale(p, alpha, Fy, base_mask)
        a2, f2 = alpha[m2], Fy[m2]
        D = lam * p["D"]
        resid = f2 - pacejka_lateral_force(a2, p["B"], p["C"], D, p["E"])
        p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
        pk = _peak_deg(p["B"], p["C"], D, p["E"])
        return {"B": p["B"], "C": p["C"], "D": D, "E": p["E"], "lambda": lam,
                "powell_converged": True,  # closed form, no optimiser
                "fit_n_samples": int(a2.size), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
                "peak_alpha_deg": pk, "peak_in_visited_range": bool(pk == pk and pk <= p99),
                "visited_alpha_p99_deg": p99, "sign_ok": D > 0, "residuals": resid, "residual_mask": m2}
    return fit


def bcompute(key):
    import modules.tyre_fit_auto as tfa
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_lateral_forces, estimate_vertical_loads, load_car_data,
        resolve_cs_min_window_samples,
    )
    from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
    from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
    from diagnostics.inspect_cs_upstream_sweep import _chain
    from diagnostics.inspect_new_laps_cs_sweep import _window_termination
    params = _params()
    data = parse_csv(SESSIONS[key])
    ch = data["channels"]
    st = prepare_vehicle_state(ch, params)
    orig = tfa._fit_axle_pacejka
    tfa._fit_axle_pacejka = _option_b_fitter(priors()["P221"])
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, params, data, "ekf_auto_pacejka", csv_path=SESSIONS[key])
    finally:
        tfa._fit_axle_pacejka = orig
    fits_path = os.path.join(CACHE, f"pb_{key}.pkl.gz")
    _save(fits_path, {"beta": beta, "manifest": man, "gate": gate, "fallback": fb, "fallback_reason": fr})
    # downstream chain exactly as (b) builds it, beta injected
    forces = estimate_lateral_forces(st, params)
    fz = estimate_vertical_loads(st, forces, params, channels=ch, car_data=load_car_data())
    ls = estimate_longitudinal_stiffness(estimate_longitudinal_forces(st, ch, params),
                                         estimate_slip_ratio(st, ch, params), st, params)
    _b, slip, cs, summ, _i = _chain(key, params, data, st, forces, fz, ls, "A", base_beta=beta)
    se = params["stability_estimation"]
    res = _load(os.path.join(B_CS_CACHE, f"{key}.pkl.gz"))
    res["variants"] = {v: res["variants"][v] for v in ("A", "F1", "F2") if v in res["variants"]}
    min_window = resolve_cs_min_window_samples(params, st["sample_rate_hz"])
    term = {ax: _window_termination(slip[a], cs[c], res["term_idx"], min_window, se["cs_min_slip_angle_span_rad"],
                                    st["s_m"], se["cs_max_window_m"])["kind"]
            for ax, a, c in (("f", "alpha_f_filt", "CS_ratio_f"), ("r", "alpha_r_filt", "CS_ratio_r"))}
    res["variants"]["PB"] = {"beta": beta.astype(np.float32),
                             "cs": (cs["CS_ratio_f"].astype(np.float32), cs["CS_ratio_r"].astype(np.float32)),
                             "summaries": summ, "term": term,
                             "info": {"gate": str(gate.get("verdict") if isinstance(gate, dict) else gate),
                                      "fallback": fb, "reason": fr}}
    _save(os.path.join(B_CACHE, f"{key}.pkl.gz"), res)
    ax = man.get("axles", {})
    print(f"[{key}] PB status {man.get('status')}, lam f {ax.get('front', {}).get('lambda')}, "
          f"r {ax.get('rear', {}).get('lambda')}, gate {gate.get('verdict') if gate else None} "
          f"{gate.get('health_score') if gate else None}, fallback {fb}", flush=True)


# ---------------------------------------------------------------- analyse helpers

def _units(r):
    """Block-bootstrap unit id per sample: each traversal, and each gap
    between consecutive traversals, is one unit."""
    n = r["time"].size
    uid = np.full(n, -1, dtype=np.int64)
    edges = sorted((tr["lo"], tr["hi"]) for tr in r["traversals"])
    nxt, pos = 0, 0
    for lo, hi in edges:
        if lo > pos:
            uid[pos:lo] = nxt; nxt += 1
        uid[max(lo, pos):hi] = nxt; nxt += 1
        pos = max(pos, hi)
    if pos < n:
        uid[pos:] = nxt
    return uid


def _boot_idx(uid_sel, rng, n_boot):
    """Resampled sample-index sets (indices into the selected population)."""
    units, inv = np.unique(uid_sel, return_inverse=True)
    members = [np.where(inv == i)[0] for i in range(units.size)]
    for _ in range(n_boot):
        pick = rng.integers(0, units.size, units.size)
        yield np.concatenate([members[i] for i in pick])


def _binned(absa, folded, edges):
    idx = np.clip(np.digitize(absa, edges[1:-1]), 0, len(edges) - 2)
    return np.array([folded[idx == b].mean() if np.any(idx == b) else np.nan for b in range(len(edges) - 1)])


def _pattern(mean, se):
    beyond = np.abs(mean) > 2 * se
    return bool(np.any(beyond[:-1] & beyond[1:])), beyond


def _curve(p, grid_rad, D=None):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    return pacejka_lateral_force(grid_rad, p["B"], p["C"], p["D"] if D is None else D, p["E"])


VERDICT_RANK = {"pass": 2, "warn": 1, "fail": 0}


def analyse():
    from modules.tyre_fit_auto import _fit_axle_pacejka
    from modules.tyre_model_pacejka import pacejka_lateral_force
    from modules.stability_analysis import load_parameters
    cfg = load_parameters()["tyre_fit_auto"]
    mu_lo, mu_hi = cfg["mu_plausibility_band_low"], cfg["mu_plausibility_band_high"]
    P = priors()
    R = {k: load(k) for k in KEYS}
    PB = {k: _load(os.path.join(CACHE, f"pb_{k}.pkl.gz")) for k in KEYS}
    rng = np.random.default_rng(SEED)

    print("=== EC2 reproduction")
    ok_all = True
    for k in KEYS:
        r = R[k]
        nl = _load(os.path.join(NEW_LAPS_CACHE, f"{k}.pkl.gz"))
        h_ref = eval(nl["gate_verdict"])["health_score"] if isinstance(nl["gate_verdict"], str) else None
        line = []
        for ax, a, f, _z in AXLES:
            fit = _fit_axle_pacejka(r[a], r[f], r["base_mask"])
            mf = r["manifest"]["axles"][ax]
            same = all(fit[p] == mf[p] for p in "BCDE")
            ok_all &= same
            line.append(f"{ax} bit-exact {same}")
            if k in RECORDED:
                rec = RECORDED[k][ax]
                got = (round(mf["B"], 3), round(mf["C"], 3), round(mf["D"], 1), round(mf["E"], 3))
                match = got == rec
                ok_all &= match
                line.append(f"{ax} record {match} {got}")
        h = r["gate"]["health_score"]
        hm = round(h, 6) == round(h_ref, 6)
        ok_all &= hm
        bA = _load(os.path.join(B_CS_CACHE, f"{k}.pkl.gz"))["variants"]["A"]["beta"]
        db = float(np.max(np.abs(r["beta_prod"].astype(np.float32) - bA)))
        ok_all &= db == 0.0
        print(f"  {k:17s} " + "; ".join(line) + f"; health {h:.6f} vs {h_ref:.6f} {hm}; beta vs (b) A max|d| {db:.2e}")
    print(f"  EC2 {'PASS' if ok_all else 'FAIL -> STOP'}")
    if not ok_all:
        return

    print("\n=== EC3 fit population (n after finiteness) and fit Fz source")
    for k in KEYS:
        r = R[k]
        print(f"  {k:17s} n_f {r['manifest']['axles']['front']['fit_n_samples']}, n_r "
              f"{r['manifest']['axles']['rear']['fit_n_samples']}, Fz source {r['fz_source']}")

    print("\n=== PRIORS")
    for name, pr in P.items():
        if name.startswith("LOSO"):
            continue
        for ax in ("front", "rear"):
            p = pr[ax]
            print(f"  {name:6s} {ax:5s} B={p['B']:.4f} C={p['C']:.4f} D={p['D']:.1f} E={p['E']:.4f} "
                  f"BCD={p['B'] * p['C'] * p['D']:.0f} N/rad peak {p['peak_alpha_deg']:.3f} deg n={p['fit_n_samples']}")

    # visited ranges on the pooled P221 population
    vis = {}
    for ax, a, f, _z in AXLES:
        aa = np.concatenate([np.abs(np.degrees(R[k][a][R[k]["base_mask"] & np.isfinite(R[k][a]) & np.isfinite(R[k][f])]))
                             for k in POOL])
        vis[ax] = {"p95": float(np.percentile(aa, 95)), "p99": float(np.percentile(aa, 99))}

    # lambda per session under every prior
    lam = {name: {k: {ax: scale(pr[ax], R[k][a], R[k][f], R[k]["base_mask"])[0] for ax, a, f, _z in AXLES}
                  for k in KEYS} for name, pr in P.items()}

    print("\n=== K1 stability (grid 0 .. pooled P221 p95 |alpha|)")
    k1_pass = True
    for ax, a, f, _z in AXLES:
        g = np.radians(np.linspace(0, vis[ax]["p95"], GRID_N))
        full = _curve(P["P221"][ax], g)
        D0 = P["P221"][ax]["D"]
        dev = {k: float(np.max(np.abs(_curve(P[f"LOSO_{k}"][ax], g) - full)) / D0) for k in POOL}
        dl = {k: abs(lam[f"LOSO_{k}"][k][ax] / lam["P221"][k][ax] - 1.0) for k in POOL}
        a_ok = all(v <= 0.03 for v in dev.values())
        b_ok = all(v <= 0.02 for v in dl.values())
        k1_pass &= a_ok and b_ok
        print(f"  {ax} (p95 {vis[ax]['p95']:.3f} deg): LOSO max curve dev / D: " +
              ", ".join(f"{k} {100 * v:.2f}%" for k, v in dev.items()) + f" -> {'PASS' if a_ok else 'FAIL'} (<= 3 %)")
        print(f"        lam under LOSO vs P221: " + ", ".join(f"{k} {100 * v:.2f}%" for k, v in dl.items()) +
              f" -> {'PASS' if b_ok else 'FAIL'} (<= 2 %)")
        ind = np.array([_curve(R[k]["manifest"]["axles"][ax], g) for k in KEYS])
        pbc = np.array([lam["P221"][k][ax] * full for k in KEYS])
        sl = slice(GRID_N // 4, None)  # the origin region carries no spread
        print(f"        context, across-session CV on the grid (median over the upper 3/4 of the grid): "
              f"independent {np.median(np.std(ind[:, sl], 0) / np.abs(np.mean(ind[:, sl], 0))):.3f}, "
              f"B {np.median(np.std(pbc[:, sl], 0) / np.abs(np.mean(pbc[:, sl], 0))):.3f}")
        for name in ("ALL8", "EW221"):
            dv = float(np.max(np.abs(_curve(P[name][ax], g) - full)) / D0)
            print(f"        sensitivity {name}: max curve dev from P221 / D = {100 * dv:.2f}%")
    print(f"  K1 {'PASS' if k1_pass else 'FAIL'}")

    print("\n=== K2 plausibility")
    k2 = {}

    def band_row(tag, p, mean_fz, lam_v=None):
        C, E, D, B = p["C"], p["E"], p["D"], p["B"]
        mu = D / mean_fz
        bcd = B * C * D / mean_fz
        flags = []
        if not (1.0 < C <= 2.0): flags.append("C")
        if not (E <= 1.0): flags.append("E")
        if not (mu_lo <= mu <= mu_hi): flags.append("mu")
        if not (6.0 <= bcd <= 24.0): flags.append("BCD/Fz")
        s = f"C={C:.3f} E={E:.3f} mu={mu:.3f} BCD/Fz={bcd:.2f}"
        if lam_v is not None:
            if abs(lam_v - 1) > 0.15: flags.append("lam")
            s += f" lam={lam_v:.4f}{' (<5 %, not interpretable)' if abs(lam_v - 1) < 0.05 else ''}"
        return s, flags
    for k in KEYS:
        r = R[k]
        for ax, a, f, z in AXLES:
            m = r["base_mask"] & np.isfinite(r[a]) & np.isfinite(r[f]) & np.isfinite(r[z])
            mfz = float(np.mean(r[z][m]))
            pb = dict(P["P221"][ax]); pb["D"] = lam["P221"][k][ax] * pb["D"]
            s_b, fl_b = band_row("B", pb, mfz, lam["P221"][k][ax])
            s_i, fl_i = band_row("ind", r["manifest"]["axles"][ax], mfz)
            pk_b = _peak_deg(pb["B"], pb["C"], pb["D"], pb["E"])
            k2[(k, ax)] = fl_b
            print(f"  {k:17s} {ax:5s} meanFz {mfz:.0f} N | B: {s_b} peak {pk_b:.2f} deg "
                  f"({'in' if pk_b <= vis[ax]['p99'] else 'OUT of'} pooled p99 {vis[ax]['p99']:.2f}) flags {fl_b or 'none'}"
                  f" | independent: {s_i} peak {r['manifest']['axles'][ax]['peak_alpha_deg']:.2f} flags {fl_i or 'none'}")

    print("\n=== K3 rear large slip")
    p99s = {k: float(np.percentile(np.abs(np.degrees(R[k]["alpha_r"][R[k]["base_mask"] & np.isfinite(R[k]["alpha_r"])])), 99))
            for k in KEYS}
    thr = float(np.median(list(p99s.values())))
    pr_r = P["P221"]["rear"]
    loso_pk = [P[f"LOSO_{k}"]["rear"]["peak_alpha_deg"] for k in POOL]
    spread = float(np.nanmax(loso_pk) - np.nanmin(loso_pk))
    in_rng = pr_r["peak_alpha_deg"] <= vis["rear"]["p99"]
    signs, n_reg = [], 0
    for k in KEYS:
        r = R[k]
        m = r["base_mask"] & np.isfinite(r["alpha_r"]) & np.isfinite(r["fy_r"])
        ad = np.abs(np.degrees(r["alpha_r"]))
        reg = m & (ad > thr)
        nreg = int(reg.sum())
        if k in POOL:
            n_reg += nreg
        if nreg < 20:
            print(f"  {k:17s} region n {nreg} (too few for a bias estimate)")
            continue
        fp = lam["P221"][k]["rear"] * pacejka_lateral_force(r["alpha_r"][reg], pr_r["B"], pr_r["C"], pr_r["D"], pr_r["E"])
        folded = (r["fy_r"][reg] - fp) * np.sign(r["alpha_r"][reg])
        uid = _units(r)[reg]
        boots = [folded[ix].mean() for ix in _boot_idx(uid, rng, N_BOOT)]
        mean, se = float(folded.mean()), float(np.std(boots))
        sig = abs(mean) > 2 * se
        if k in POOL and sig:
            signs.append(np.sign(mean))
        print(f"  {k:17s} p99 {p99s[k]:.2f} deg, region n {nreg}, folded residual mean {mean:+.0f} N (SE {se:.0f})"
              f"{' beyond 2 SE' if sig else ''}")
    common = max(sum(1 for s in signs if s > 0), sum(1 for s in signs if s < 0)) > len(POOL) / 2
    stab = in_rng and spread <= 0.5 and not common
    print(f"  region |alpha_r| > {thr:.2f} deg; pooled P221 n in region {n_reg}; prior rear peak "
          f"{pr_r['peak_alpha_deg']:.2f} deg vs pooled p99 {vis['rear']['p99']:.2f} ({'in' if in_rng else 'OUT'}); "
          f"LOSO rear peaks {', '.join(f'{x:.2f}' for x in loso_pk)} spread {spread:.2f} deg; common-sign bias {common}")
    print(f"  K3 -> {'STABILISES' if stab else 'IMPOSES'}")

    print("\n=== K4 NIS gate, independent vs B")
    worse = []
    for k in KEYS:
        gi, gb = R[k]["gate"], PB[k]["gate"]
        si = R[k]["manifest"]["r_sweep"]["found_in_band"]
        sb = PB[k]["manifest"].get("r_sweep", {}).get("found_in_band")
        vi = gi["verdict"] if gi else "none"
        vb = gb["verdict"] if gb else "none"
        w = VERDICT_RANK.get(vb, -1) < VERDICT_RANK.get(vi, -1) or (si and not sb)
        if w:
            worse.append(k)
        print(f"  {k:17s} independent {vi} {gi['health_score'] if gi else float('nan'):.4f} sweep-in-band {si} "
              f"status {R[k]['manifest']['status']} | B {vb} {gb['health_score'] if gb else float('nan'):.4f} "
              f"sweep-in-band {sb} status {PB[k]['manifest'].get('status')} fallback {PB[k]['fallback']}"
              f"{'  WORSE' if w else ''}")

    print("\n=== K5 (i) residual structure, 10 equal-count |alpha| bins on [0, p99]")
    pattern_sessions = set()
    for k in KEYS:
        r = R[k]
        uid_all = _units(r)
        for ax, a, f, _z in AXLES:
            m = r["base_mask"] & np.isfinite(r[a]) & np.isfinite(r[f])
            al, fy = r[a][m], r[f][m]
            absa = np.abs(np.degrees(al))
            sel = absa <= np.percentile(absa, 99)
            al, fy, absa, uid = al[sel], fy[sel], absa[sel], uid_all[m][sel]
            edges = np.percentile(absa, np.linspace(0, 100, 11))
            pi = r["manifest"]["axles"][ax]
            pp = P["P221"][ax]
            res_i = (fy - pacejka_lateral_force(al, pi["B"], pi["C"], pi["D"], pi["E"])) * np.sign(al)
            res_b = (fy - lam["P221"][k][ax] * pacejka_lateral_force(al, pp["B"], pp["C"], pp["D"], pp["E"])) * np.sign(al)
            bi, bb = [], []
            for ix in _boot_idx(uid, rng, N_BOOT):
                bi.append(_binned(absa[ix], res_i[ix], edges)); bb.append(_binned(absa[ix], res_b[ix], edges))
            mi, mb = _binned(absa, res_i, edges), _binned(absa, res_b, edges)
            pat_i, _ = _pattern(mi, np.nanstd(bi, 0))
            pat_b, _ = _pattern(mb, np.nanstd(bb, 0))
            if pat_b and not pat_i:
                pattern_sessions.add(k)
            print(f"  {k:17s} {ax:5s} independent pattern {pat_i}, B pattern {pat_b}; bin means N "
                  f"ind [{', '.join(f'{x:+.0f}' for x in mi)}] B [{', '.join(f'{x:+.0f}' for x in mb)}]")
    k5i = len(pattern_sessions) >= 2
    print(f"  sessions with a B-only pattern: {sorted(pattern_sessions) or 'none'} -> (i) {'TRIGGERS' if k5i else 'clear'}")

    print("\n=== K5 (ii) setup pair JST 21 vs JST 29")
    k5ii = False
    pair = ("nbr_jst_problem", "nbr_jst_solution")
    for ax, a, f, _z in AXLES:
        p95 = min(float(np.percentile(np.abs(np.degrees(R[k][a][R[k]["base_mask"] & np.isfinite(R[k][a])])), 95)) for k in pair)
        g = np.radians(np.linspace(0, p95, GRID_N))
        curves, ses, lams, lses = [], [], [], []
        for k in pair:
            r = R[k]
            m = r["base_mask"] & np.isfinite(r[a]) & np.isfinite(r[f])
            idx_m = np.where(m)[0]
            uid = _units(r)[m]
            curves.append(_curve(r["manifest"]["axles"][ax], g))
            bc, bl = [], []
            for ix in _boot_idx(uid, rng, N_BOOT_REFIT):
                sub = np.zeros_like(m)
                sub_idx = idx_m[ix]
                # duplicates matter for the bootstrap: fit on the resampled arrays directly
                aa, ff = r[a][sub_idx], r[f][sub_idx]
                fit = _fit_axle_pacejka(aa, ff, np.ones(aa.size, dtype=bool))
                bc.append(_curve(fit, g))
                bl.append(scale(P["P221"][ax], aa, ff, np.ones(aa.size, dtype=bool))[0])
            ses.append(np.std(bc, 0)); lams.append(lam["P221"][k][ax]); lses.append(float(np.std(bl)))
        z_ind = np.abs(curves[1] - curves[0])[1:] / np.sqrt(ses[0] ** 2 + ses[1] ** 2)[1:]
        z_b = abs(lams[1] - lams[0]) / np.sqrt(lses[0] ** 2 + lses[1] ** 2)
        differ = bool(np.nanmax(z_ind) > 2)
        mask_ax = differ and z_b <= 2
        k5ii |= mask_ax
        print(f"  {ax}: grid to {p95:.2f} deg; independent max z {np.nanmax(z_ind):.2f} "
              f"({'differ' if differ else 'no difference beyond noise -> uninformative'}); "
              f"B lam {lams[0]:.4f} (SE {lses[0]:.4f}) vs {lams[1]:.4f} (SE {lses[1]:.4f}), z {z_b:.2f}"
              f"{' -> MASKING' if mask_ax else ''}")

    print("\n=== K5 (iii) out-of-pool transfer")
    k5iii = False
    for k in OUT_OF_POOL:
        flags = [ax for ax in ("front", "rear") if abs(lam["P221"][k][ax] - 1) > 0.15]
        t = bool(flags) or k in pattern_sessions
        k5iii |= t
        print(f"  {k:17s} lam f {lam['P221'][k]['front']:.4f} r {lam['P221'][k]['rear']:.4f}; "
              f"B-only residual pattern {k in pattern_sessions} -> {'TRIGGERS' if t else 'clear'}")

    k5iv = bool(worse)
    print(f"\n=== K5 summary: (i) {k5i}, (ii) {k5ii}, (iii) {k5iii}, (iv) {k5iv} (K4 worse on {worse or 'none'})"
          f" -> {'MASKING' if (k5i or k5ii or k5iii or k5iv) else 'no masking criterion triggered'}")
    with open(os.path.join(OUT_DIR, "analyse.json"), "w") as fh:
        json.dump({"lam": lam, "vis": vis, "k1": k1_pass, "k3_stabilises": stab, "k4_worse": worse,
                   "k5": [k5i, k5ii, k5iii, k5iv], "k2_flags": {f"{k}|{ax}": v for (k, ax), v in k2.items()}},
                  fh, indent=1)


# ---------------------------------------------------------------- K6

def score():
    # the (b) scorer, unchanged, pointed at a k6/ copy: it reads
    # <OUT_DIR>/cache/<key>.pkl.gz for every key present
    import diagnostics.inspect_cs_upstream_sweep as b
    b.VARIANTS = ["A", "PB", "F1", "F2"]
    b.CANDIDATES = ["PB"]
    b.OUT_DIR = os.path.join(OUT_DIR, "k6")
    os.makedirs(os.path.join(b.OUT_DIR, "cache"), exist_ok=True)
    tier1 = ["dubai", "nbr_jst_problem", "nbr_jst_solution", "pr_us_v2", "v3"]
    for k in KEYS:
        src = os.path.join(B_CACHE, f"{k}.pkl.gz")
        dst = os.path.join(b.OUT_DIR, "cache", f"{k}.pkl.gz")
        if k in tier1:
            with open(src, "rb") as fi, open(dst, "wb") as fo:
                fo.write(fi.read())
        elif os.path.exists(dst):
            os.remove(dst)
    b.score()
    for k in b.TIER2_KEYS:
        with open(os.path.join(B_CACHE, f"{k}.pkl.gz"), "rb") as fi, \
                open(os.path.join(b.OUT_DIR, "cache", f"{k}.pkl.gz"), "wb") as fo:
            fo.write(fi.read())
    b.confirm(["PB"])


def render():
    from diagnostics.trace_render import render_cs_traces
    for k in KEYS:
        r = _load(os.path.join(B_CACHE, f"{k}.pkl.gz"))
        for v in ("A", "PB"):
            cf, cr = (x.astype(float) for x in r["variants"][v]["cs"])
            out = os.path.join(OUT_DIR, f"traces_{k}_{v}.png")
            clipped = render_cs_traces(out, r["time"], r["s_m"], cf, cr, r["laps"], r["corners"], f"{k} {v}")
            print(f"{k} {v}: {out}; below y floor f {clipped['f']}, r {clipped['r']}")


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "compute":
        compute(sys.argv[2])
    elif mode == "bcompute":
        bcompute(sys.argv[2])
    else:
        {"prior": prior, "analyse": analyse, "score": score, "render": render}[mode]()
