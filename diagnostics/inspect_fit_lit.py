# Q-FIT-LIT verification cycle -- the literature-bounded fit L through the
# whole chain (PLAN.md queue entry Q-FIT-LIT; author rulings and EXECUTE
# 2026-10-18). Read-only: the mode is set in a deep-copied params dict, no
# config edit; production default stays per_session.
#
# FROZEN FRAME (written before any number exists)
#
# SESSIONS: NBR-73-010-026-JDR, MLA-SE07_US-v2, NBR JST 29 (paths via
#   tests/conftest.py).
# RUNS (full chain each: fit -> R sweep -> D4 -> NIS gate -> fallback -> CS):
#   L     tyre_fit_mode literature_bounded, inverse-density weights
#   L-nw  the same with weights off (tyre_fit_auto._fit_axle_l wrapped with
#         weighted=False for the run, restored in finally)
# START INDEPENDENCE (fit only, L): starts B 8 and B 25 (mu start by the
#   production rule 8000 N / Fz_ref) vs the production start B 12 ->
#   relative difference of B and mu, and curve difference = RMS over the
#   visited range [p1, p99] of |f_start - f_12| / mu_12.
# LOLO (fit only, L): one refit per analysis lap with that lap's samples
#   removed from the fit population; blockwise jackknife with laps as
#   blocks (Kuensch 1989): SE = sqrt((n - 1) / n * sum (theta_i - mean)^2)
#   for B, mu and K = B C mu; reported with SE / full-fit value and range.
# PHYSICAL CHECKS per axle (L and L-nw): B, mu, K [1/rad], BCD = K Fz_ref
#   [N/rad]; parameter at a bound; peak alpha, p99 |alpha|, peak covered
#   (no -> mu flagged EXTRAPOLATED); peak vs 3-7 deg (Milliken RCVD p. 24,
#   tyre level; reported, not a gate); residual RMS per |alpha| bin 0-1,
#   1-2, 2-4, > 4 deg in N and in Fy/Fz; residual mean in Fy/Fz per corner
#   speed class (samples inside canonical corner brackets on analysis laps)
#   and per speed tercile of the fit population; shifts from the linear
#   region (|alpha| <= 1 deg, OLS f = k alpha + c): Sv = c [Fy/Fz], Sh =
#   -c / k [deg] (reported only, not adopted); validity range [p1, p99] of
#   signed alpha. Fit population, alpha, Fy (front in the tyre frame), Fz:
#   exactly the inputs fit_session_pacejka hands _fit_axle_l.
# CHAIN FUNCTION: fit status (+ reason), D4 result, NIS gate verdict and
#   score, fallback, R pick, hard-bound re-entries (|raw beta| > bound on the
#   base mask, final run), NIS-window divergences, straight-line |beta|
#   median / p95 (base mask outside every canonical corner bracket).
# CS for L and L-nw: distributions and traces, INFORMATION ONLY -- no
#   comparison rule, no production comparison.
# YAW SIGN: one explained line, ours (+Iz psidd / L, yaw rate CCW positive)
#   vs Goblirsch et al. 2025 Eq. 11 (-Izz dr/dt).
# PREDICTIONS (scored CONFIRMED / REFUTED / MIXED):
#   (1) peak beyond p99 on most axles -> mu extrapolated: CONFIRMED if
#       uncovered on >= 4 of 6 axles (L), REFUTED if <= 2, MIXED at 3.
#   (2) D4 g3 at risk on US-v2 rear: CONFIRMED if g3 fails there (L or
#       L-nw); MIXED if it passes with the peak within 1.5 deg of a band
#       edge; REFUTED otherwise.
#   (3) L vs L-nw BCD within 5.5 % (point estimate): CONFIRMED on all 6
#       axles, REFUTED on none, MIXED otherwise.
#   (4) hard-bound re-entries on the NBR sessions rise above production
#       (recorded P: JDR 28, JST 29 10; E0C13 precedent 86 / 52):
#       CONFIRMED if L > P on both, REFUTED if L <= P on both, MIXED
#       otherwise; the level relative to E0C13 is stated beside.
#   (5) start independence: B and mu within 1 % from both starts:
#       CONFIRMED on all 6 axles, REFUTED on none, MIXED otherwise.
# EXIT: one cycle. If L fails the chain function (degenerate, D4 fail, gate
#   fail, fallback) on any session: scoped negative with the failing check
#   and its mechanism, no new variants.
# RENDERS (render title rule): fitcurve_<key>_<axle>.png = slip angle vs
#   Fy/Fz cloud of the fit population with the L and L-nw curves, peak
#   marked, visited range shaded; traces_<key>_L.png = standard CS trace set
#   for L; csdist_<key>.png = CS distribution L and L-nw (information).
# Outputs: diagnostics/plots/fit_lit/ (run.log, per-session JSON).
# Run: python -m diagnostics.inspect_fit_lit
#
# [2026-10-18 REOPEN MODE, declared post-result (author): --reopen-d4 runs L only
#   (weighted, as frozen) with the redesigned D4 of mode literature_bounded
#   (tyre_fit_auto.plausibility_gate_l), reports the D4 checks with values and
#   the chain items, and checks that every fit-level item reproduces the
#   cycle's fit_lit_<key>.json exactly. Output diagnostics/plots/fit_lit/d4_reopen/.
#   The frozen frame above is unchanged.]
# [2026-10-18 C10 MODE (author ruling Q-OBS-REENTRY fix, post-result): --c10 runs
#   L only with C = 1.0 (module constant), reports fit level, start
#   independence, LOLO, D4-L and chain items, and the frozen expectation
#   "in-range curve within 5.5 % of D of the C 1.3 fit": RMS and max over the
#   visited range [p1, p99] of |f_C1.0 - f_C1.3| / mu_C1.3, C 1.3 curves from
#   the cycle's fit_lit_<key>.json. Output diagnostics/plots/fit_lit/c10/.]

import contextlib
import copy
import ctypes
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force  # noqa: E402

OUT_DIR = "diagnostics/plots/fit_lit"
SESSIONS = (
    ("nbr_jdr", "NBR-73-010-026-JDR",
     "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("pr_us_v2", "Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
    ("nbr_jst29", "NBR JST outing 29", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"),
)
P_REENTRIES = {"nbr_jdr": 28, "nbr_jst29": 10}      # Q-FIT-SIMPLE run, production
E0C13_REENTRIES = {"nbr_jdr": 86, "nbr_jst29": 52}  # Q-FIT-SIMPLE run, E0C13
RESID_BINS_DEG = ((0, 1), (1, 2), (2, 4), (4, np.inf))
LINEAR_REGION_DEG = 1.0
BAR = 0.055
PEAK_BAND_DEG = (3.0, 7.0)
WORDS = {"L": "literature-bounded fit, even slip-angle weighting",
         "L-nw": "literature-bounded fit, unweighted"}
STYLE = {"L": ("#2a78d6", "-"), "L-nw": ("#eb6834", "--")}
YAW_LINE = ("YAW SIGN: ours Fy_f = m ay b/L + Iz psidd / L follows from the yaw balance Iz psidd = a Fy_f - b Fy_r "
            "with y left and yaw rate counter-clockwise positive seen from above (z up; checked against the GPS course, "
            "stability_analysis.estimate_sideslip_gps). Goblirsch et al. 2025 Eq. 11 carries -Izz dr/dt: the same balance "
            "gives that sign when r is positive clockwise against a left-positive y (mixed axis set), so the two agree "
            "once r is mapped to -r; the paper's axis definition is not in this repo (reviewer copy) and is to be read "
            "there to close this line.")

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    if _LOG is not None:
        _LOG.write(msg + "\n")
        _LOG.flush()


def keep_awake(on):
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | (0x00000001 if on else 0))


ORIG = {"fsp": tfa.fit_session_pacejka, "ekf": tfa.estimate_sideslip_ekf_pacejka, "fitl": tfa._fit_axle_l}


@contextlib.contextmanager
def capture(holder, weighted):
    def fsp(*a, **kw):
        man = ORIG["fsp"](*a, **kw)
        holder["raw"] = man
        return man

    def ekf(state, params, pass_id=None, **kw):
        out = ORIG["ekf"](state, params, pass_id=pass_id, **kw)
        if pass_id == "_auto_final":
            holder["final"] = out
        return out

    def fitl_nw(alpha, Fy, Fz, mask, bin_rad, weighted=True):
        return ORIG["fitl"](alpha, Fy, Fz, mask, bin_rad, weighted=False)

    tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = fsp, ekf
    if not weighted:
        tfa._fit_axle_l = fitl_nw
    try:
        yield
    finally:
        tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka, tfa._fit_axle_l = (
            ORIG["fsp"], ORIG["ekf"], ORIG["fitl"])


def load_session(key, label, rel):
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_sideslip,
                                            estimate_slip_angles, estimate_lateral_forces)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    path = resolve_data_path(rel)
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))
    params = copy.deepcopy(params)
    params["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
    state = prepare_vehicle_state(data["channels"], params)
    S = {"key": key, "label": label, "path": path, "data": data, "params": params, "state": state}
    base = tfa._base_mask(state, data["laps"])
    window, _ = tfa._speed_window_mask(state, data, params["tyre_fit_auto"])
    S["base"], S["fit_mask"] = base, base & window
    forces = estimate_lateral_forces(state, params)
    slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    li = tfa._axle_load_inputs(state, data, params, forces, S["fit_mask"])
    assert "degenerate_reason" not in li, li
    S["li"] = li
    S["alpha"] = {"front": slip["alpha_f_filt"], "rear": slip["alpha_r_filt"]}
    S["Fy"] = {"front": tfa._front_tyre_frame_force(forces["Fy_f_filt"], li["fx_f_N"], li["delta_f"]),
               "rear": forces["Fy_r_filt"]}
    S["Fz"] = {"front": li["fz_f_N"], "rear": li["fz_r_N"]}
    t, s_m = state["time"], state["s_m"]
    laps = {l["lap_number"]: l for l in data["laps"]}
    inst, in_br = [], np.zeros(t.size, dtype=bool)
    speed_class = np.full(t.size, "", dtype=object)
    for c in data["corners"]:
        lap = laps.get(c.get("lap_number"))
        if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
            continue
        sl = tfa._canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                         c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            inst.append((c["stable_corner_id"], c["lap_number"], sl))
            in_br[sl] = True
            speed_class[sl] = str(c.get("speed_class"))
    S["inst"], S["in_br"], S["speed_class"] = inst, in_br, speed_class
    kerb = state.get("kerb_mask")
    S["verdict_base"] = in_br & state["moving_mask"] & (~kerb if kerb is not None else True)
    S["straight"] = base & ~in_br
    return S


def chain(S, weighted):
    from modules.stability_analysis import estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness
    holder = {}
    t0 = time.perf_counter()
    with capture(holder, weighted):
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(S["state"], S["params"], S["data"], "ekf_auto_pacejka",
                                                             csv_path=S["path"])
    t1 = time.perf_counter()
    slip = estimate_slip_angles(S["state"], beta, S["params"])
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(S["state"], S["params"]), S["state"],
                                      S["params"], S["data"]["laps"])
    return {"beta": beta, "man": man, "gate": gate, "fallback": fb, "fallback_reason": fr, "cs": cs,
            "raw": holder.get("raw"), "final": holder.get("final"), "t_fit": t1 - t0}


def fit_pop(S, axle, extra=None):
    a, f, z = S["alpha"][axle], S["Fy"][axle], S["Fz"][axle]
    m = S["fit_mask"] & np.isfinite(a) & np.isfinite(f) & np.isfinite(z) & (z > 0)
    return m if extra is None else m & extra


def shape(a, B, mu):
    return mu * pacejka_lateral_force(a, B, tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)


def physical(S, fit, axle):
    m = fit_pop(S, axle)
    a, y, z = S["alpha"][axle][m], S["Fy"][axle][m] / S["Fz"][axle][m], S["Fz"][axle][m]
    r_n = y - shape(a, fit["B"], fit["mu"])
    r_N = z * r_n
    ad = np.abs(np.degrees(a))
    bins = {}
    for lo, hi in RESID_BINS_DEG:
        k = (ad >= lo) & (ad < hi)
        bins[f"{lo}-{hi}"] = {"rms_N": float(np.sqrt(np.mean(r_N[k] ** 2))) if k.any() else None,
                              "rms_norm": float(np.sqrt(np.mean(r_n[k] ** 2))) if k.any() else None,
                              "n": int(k.sum())}
    sc = S["speed_class"][m]
    by_class = {c: float(np.mean(r_n[sc == c])) for c in sorted(set(sc) - {""})}
    v = S["state"]["v_mps"][m]
    t1, t2 = np.percentile(v, [100 / 3, 200 / 3])
    terc = {"low": float(np.mean(r_n[v < t1])), "mid": float(np.mean(r_n[(v >= t1) & (v < t2)])),
            "high": float(np.mean(r_n[v >= t2]))}
    lin = ad <= LINEAR_REGION_DEG
    k_, c_ = np.polyfit(a[lin], y[lin], 1)
    p1, p99s = np.percentile(np.degrees(a), [1, 99])
    K = fit["B"] * fit["C"] * fit["mu"]
    peak = fit["peak_alpha_deg"]
    return {
        "B": fit["B"], "mu": fit["mu"], "K_per_rad": K, "BCD_N_per_rad": K * fit["fz_ref_N"], "fz_ref_N": fit["fz_ref_N"],
        "bound_active": fit["bound_active"], "peak_alpha_deg": peak, "p99_abs_deg": fit["visited_alpha_p99_deg"],
        "peak_covered": fit["peak_in_visited_range"],
        "mu_extrapolated": not fit["peak_in_visited_range"],
        "peak_in_3_7_deg": bool(peak == peak and PEAK_BAND_DEG[0] <= peak <= PEAK_BAND_DEG[1]),
        "rms_bins": bins, "mean_resid_norm_speed_class": by_class, "mean_resid_norm_speed_tercile": terc,
        "speed_tercile_edges_kmh": [float(t1 * 3.6), float(t2 * 3.6)],
        "Sv_norm": float(c_), "Sh_deg": float(np.degrees(-c_ / k_)) if k_ else None,
        "linear_slope_per_rad": float(k_), "validity_deg": [float(p1), float(p99s)], "n": int(m.sum()),
    }


def chain_function(S, run):
    man, gate, fin = run["man"], run["gate"], run["final"]
    raw = run["raw"] or {}
    out = {"status": man.get("status"), "reason": man.get("degenerate_reason") or man.get("marginal_reason"),
           "d4": {a: (raw.get("plausibility_gate") or {}).get(a, {}).get("failed") for a in ("front", "rear")},
           "gate": None if gate is None else {"verdict": gate["verdict"], "health_score": gate["health_score"]},
           "fallback": run["fallback"], "fallback_reason": run["fallback_reason"]}
    rs = man.get("r_sweep")
    out["R"] = None if not rs else {"r_ay_scale": rs["chosen"]["r_ay_scale"], "r_yaw_scale": rs["chosen"]["r_yaw_scale"],
                                    "selection": rs["selection"]}
    if fin is not None:
        bflag = np.abs(fin["beta"]) > np.radians(S["params"]["tyre_fit_auto"]["beta_hard_bound_deg"])
        out["hard_bound_reentries"] = int((bflag & S["base"]).sum())
        out["nis_window_divergences"] = int((fin["diverged_mask"] & ~bflag & S["base"]).sum())
    else:
        out["hard_bound_reentries"] = out["nis_window_divergences"] = None
    b = np.abs(np.degrees(run["beta"][S["straight"]]))
    b = b[np.isfinite(b)]
    out["beta_straight_abs_deg"] = {"median": float(np.median(b)), "p95": float(np.percentile(b, 95))}
    out["fails"] = [x for x, bad in (("degenerate", out["status"] == "degenerate"),
                                     ("D4", any(out["d4"][a] for a in ("front", "rear"))),
                                     ("gate fail", out["gate"] is not None and out["gate"]["verdict"] == "fail"),
                                     ("fallback", out["fallback"])) if bad]
    return out


def start_independence(S, axle, ref):
    m = S["fit_mask"]
    bin_rad = S["params"]["tyre_fit_auto"]["fit_density_bin_rad"]
    out = {}
    orig = tfa.L_START_B
    try:
        for b0 in (8.0, 25.0):
            tfa.L_START_B = b0
            f = tfa._fit_axle_l(S["alpha"][axle], S["Fy"][axle], S["Fz"][axle], m, bin_rad)
            pm = fit_pop(S, axle)
            p1, p99 = np.percentile(S["alpha"][axle][pm], [1, 99])
            g = np.linspace(p1, p99, 2000)
            d = np.abs(shape(g, f["B"], f["mu"]) - shape(g, ref["B"], ref["mu"])) / ref["mu"]
            out[f"B0={b0:g}"] = {"B": f["B"], "mu": f["mu"], "dB_rel": f["B"] / ref["B"] - 1,
                                 "dmu_rel": f["mu"] / ref["mu"] - 1, "curve_rms_rel_mu": float(np.sqrt(np.mean(d ** 2)))}
    finally:
        tfa.L_START_B = orig
    return out


def lolo(S, axle, ref):
    t = S["state"]["time"]
    bin_rad = S["params"]["tyre_fit_auto"]["fit_density_bin_rad"]
    vals = []
    for lap in S["data"]["laps"]:
        if not lap.get("is_valid_for_analysis"):
            continue
        keep = ~((t >= lap["start_time"]) & (t <= lap["end_time"]))
        f = tfa._fit_axle_l(S["alpha"][axle], S["Fy"][axle], S["Fz"][axle], S["fit_mask"] & keep, bin_rad)
        vals.append((lap["lap_number"], f["B"], f["mu"], f["B"] * f["C"] * f["mu"]))
    n = len(vals)
    out = {"n_laps": n}
    for j, name in ((1, "B"), (2, "mu"), (3, "K")):
        th = np.array([v[j] for v in vals])
        full = {"B": ref["B"], "mu": ref["mu"], "K": ref["B"] * ref["C"] * ref["mu"]}[name]
        se = float(np.sqrt((n - 1) / n * np.sum((th - th.mean()) ** 2)))
        out[name] = {"full": full, "jackknife_se": se, "se_rel": se / full, "min": float(th.min()),
                     "max": float(th.max())}
    out["per_lap"] = [{"lap": v[0], "B": v[1], "mu": v[2], "K": v[3]} for v in vals]
    return out


def cs_info(S, run):
    cs = run["cs"]
    out = {}
    for ax in ("f", "r"):
        pop = S["verdict_base"] & ~cs[f"near_strike_{ax}"]
        v = cs[f"CS_ratio_{ax}"][pop]
        fin = v[np.isfinite(v)]
        out[ax] = {"finite_share": float(fin.size / max(1, v.size)),
                   "p1_p5_p10_p50": [float(np.percentile(fin, q)) for q in (1, 5, 10, 50)],
                   "share_lt_m1": float(np.mean(fin < -1)), "share_lt_m2": float(np.mean(fin < -2)),
                   "n_no_reference_analysis": cs[f"n_no_reference_analysis_{ax}"]}
    return out


# ------------------------------------------------------------------ renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def render_curve(S, fits, axle, path):
    plt = _plt()
    from matplotlib.colors import LogNorm
    m = fit_pop(S, axle)
    a, y = np.degrees(S["alpha"][axle][m]), S["Fy"][axle][m] / S["Fz"][axle][m]
    p1, p99 = np.percentile(a, [1, 99])
    lim = max(15.0, 1.2 * float(np.max(np.abs([p1, p99]))))
    y_lim = 1.25 * float(np.percentile(np.abs(y), 99.9))
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.axvspan(p1, p99, color="#e5e7eb", alpha=0.6, lw=0, label="visited slip range (1st to 99th percentile)")
    ax.hist2d(a, y, bins=[np.linspace(-lim, lim, 240), np.linspace(-y_lim, y_lim, 160)], cmap="Greys", cmin=1,
              norm=LogNorm())
    g = np.radians(np.linspace(-lim, lim, 1500))
    for code, fit in fits.items():
        col, ls = STYLE[code]
        ax.plot(np.degrees(g), shape(g, fit["B"], fit["mu"]), color=col, ls=ls, lw=2.0, label=WORDS[code])
        pk = fit["peak_alpha_deg"]
        if pk == pk:
            for sgn in (-1, 1):
                ax.plot([sgn * pk], [sgn * shape(np.radians(pk), fit["B"], fit["mu"])], marker="o", ms=8,
                        color=col, mec="white", mew=1.5)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-y_lim, y_lim)
    ax.set_xlabel("axle slip angle from the simple geometric estimate [deg]")
    ax.set_ylabel(f"{axle} axle lateral force / measured axle load [-]")
    ax.set_title(f"{S['label']}, {axle} axle: load-normalised tyre curve, literature-bounded fit\n"
                 f"grey: sample density (log); dots: curve peak; shaded: visited slip range")
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_csdist(S, runs, path):
    plt = _plt()
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), sharey=True)
    for ax, x, name in ((axs[0], "f", "front"), (axs[1], "r", "rear")):
        for code, run in runs.items():
            cs = run["cs"]
            pop = S["verdict_base"] & ~cs[f"near_strike_{x}"]
            v = cs[f"CS_ratio_{x}"][pop]
            v = np.sort(v[np.isfinite(v)])
            col, ls = STYLE[code]
            ax.plot(v, np.arange(1, v.size + 1) / v.size, color=col, ls=ls, lw=1.6, label=WORDS[code])
        ax.set_xlim(-3.5, 1.05)
        ax.set_xlabel(f"{name} cornering stiffness ratio")
        ax.set_title(f"{name} axle")
        ax.grid(alpha=0.25)
    axs[0].set_ylabel("share of samples at or below")
    axs[1].legend(fontsize=8, loc="upper left")
    fig.suptitle(f"{S['label']}: cornering stiffness ratio distribution, literature-bounded fit (information only)\n"
                 f"corner samples on analysis laps, kerb-affected samples excluded")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_traces(S, run, path):
    from diagnostics.trace_render import render_cs_traces
    cs = run["cs"]
    render_cs_traces(path, S["state"]["time"], S["state"]["s_m"], cs["CS_ratio_f"], cs["CS_ratio_r"],
                     S["data"]["laps"], S["data"]["corners"],
                     f"{S['label']}: cornering stiffness ratio, literature-bounded tyre fit",
                     near_strike={"f": cs["near_strike_f"], "r": cs["near_strike_r"]})


# ------------------------------------------------------------------ scoring

def score_counts(n_true, n_total, all_rule=True):
    if n_true == n_total:
        return "CONFIRMED"
    return "REFUTED" if n_true == 0 else "MIXED"


def predictions(R):
    out = {}
    unc = [not R[k]["L"]["physical"][a]["peak_covered"] for k in R for a in ("front", "rear")]
    n = sum(unc)
    out["(1) peak beyond p99 on most axles"] = ("CONFIRMED" if n >= 4 else "REFUTED" if n <= 2 else "MIXED",
                                                f"uncovered on {n} of {len(unc)} axles")
    if "pr_us_v2" in R:
        d4 = [R["pr_us_v2"][c]["chain"]["d4"]["rear"] for c in ("L", "L-nw")]
        pk = [R["pr_us_v2"][c]["physical"]["rear"]["peak_alpha_deg"] for c in ("L", "L-nw")]
        g3 = any(d and "g3 peak" in d for d in d4)
        near = any(p == p and min(abs(p - 3.0), abs(p - 15.0)) <= 1.5 for p in pk)
        out["(2) D4 g3 at risk on US-v2 rear"] = ("CONFIRMED" if g3 else "MIXED" if near else "REFUTED",
                                                  f"D4 rear {d4}, peak {pk}")
    rel = [abs(R[k]["L"]["physical"][a]["BCD_N_per_rad"] / R[k]["L-nw"]["physical"][a]["BCD_N_per_rad"] - 1)
           for k in R for a in ("front", "rear")]
    out["(3) L vs L-nw BCD within 5.5 %"] = (score_counts(sum(r < BAR for r in rel), len(rel)),
                                            f"|rel diff| {[round(r, 4) for r in rel]}")
    nb = [k for k in ("nbr_jdr", "nbr_jst29") if k in R]
    re_ = {k: R[k]["L"]["chain"]["hard_bound_reentries"] for k in nb}
    up = [re_[k] is not None and re_[k] > P_REENTRIES[k] for k in nb]
    out["(4) hard-bound re-entries rise on NBR above production"] = (
        "CONFIRMED" if all(up) else "REFUTED" if not any(up) else "MIXED",
        f"L {re_}, production {P_REENTRIES}, E0C13 precedent {E0C13_REENTRIES}")
    si = [abs(v["dB_rel"]) < 0.01 and abs(v["dmu_rel"]) < 0.01
          for k in R for a in ("front", "rear") for v in R[k]["start_independence"][a].values()]
    out["(5) start independence within 1 %"] = (score_counts(sum(si), len(si)),
                                               f"{sum(si)} of {len(si)} start x axle cases within 1 %")
    return out


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if o != o else float(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    t_start = time.perf_counter()
    try:
        R = {}
        for key, label, rel in SESSIONS:
            log("=" * 100)
            log(f"{label} ({key})")
            S = load_session(key, label, rel)
            log(f"  axle load: lowest level {S['li']['info']['lowest_level']} {S['li']['info']['level_note']}; "
                f"Fx front: {S['li']['info']['fx_front_source']} (level {S['li']['info']['fx_front_level']})")
            runs = {"L": chain(S, True), "L-nw": chain(S, False)}
            rec = {"label": label}
            for code, run in runs.items():
                fits = {a: run["man"]["axles"][a] for a in ("front", "rear")}
                rec[code] = {"physical": {a: physical(S, fits[a], a) for a in ("front", "rear")},
                             "chain": chain_function(S, run), "cs_info": cs_info(S, run),
                             "t_fit_s": run["t_fit"]}
                log(f"  {code}: fit chain {run['t_fit']:.0f} s, status {run['man'].get('status')}, "
                    f"fails {rec[code]['chain']['fails'] or 'none'}")
            fitsL = {a: runs["L"]["man"]["axles"][a] for a in ("front", "rear")}
            rec["start_independence"] = {a: start_independence(S, a, fitsL[a]) for a in ("front", "rear")}
            rec["lolo"] = {a: lolo(S, a, fitsL[a]) for a in ("front", "rear")}
            R[key] = rec
            for a in ("front", "rear"):
                render_curve(S, {c: runs[c]["man"]["axles"][a] for c in runs},
                             a, os.path.join(OUT_DIR, f"fitcurve_{key}_{a}.png"))
            render_traces(S, runs["L"], os.path.join(OUT_DIR, f"traces_{key}_L.png"))
            render_csdist(S, runs, os.path.join(OUT_DIR, f"csdist_{key}.png"))
            with open(os.path.join(OUT_DIR, f"fit_lit_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1)
            log(f"  done ({(time.perf_counter() - t_start) / 60:.1f} min elapsed)")
        pred = predictions(R)
        exit_fail = {k: R[k]["L"]["chain"]["fails"] for k in R if R[k]["L"]["chain"]["fails"]}
        with open(os.path.join(OUT_DIR, "summary.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable({"predictions": pred, "exit_failures_L": exit_fail}), fh, indent=1)
        report(R, pred, exit_fail)
        log(f"\nTOTAL {(time.perf_counter() - t_start) / 60:.1f} min")
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


def report(R, pred, exit_fail):
    f = lambda x, n=3: "nan" if x is None or x != x else f"{x:.{n}f}"  # noqa: E731
    log("\n" + "=" * 100 + "\nREPORT")
    log(YAW_LINE)
    for key, rec in R.items():
        log(f"\n## {rec['label']}")
        for code in ("L", "L-nw"):
            c = rec[code]["chain"]
            log(f"- {code}: status {c['status']} ({c['reason'] or '-'}); D4 {c['d4']}; gate {c['gate']}; fallback "
                f"{c['fallback']}; R {c['R']}; hard-bound re-entries {c['hard_bound_reentries']}; NIS-window "
                f"divergences {c['nis_window_divergences']}; straight |beta| median/p95 "
                f"{f(c['beta_straight_abs_deg']['median'])}/{f(c['beta_straight_abs_deg']['p95'])} deg")
            for a in ("front", "rear"):
                p = rec[code]["physical"][a]
                log(f"    {a}: B {f(p['B'])} mu {f(p['mu'])} K {f(p['K_per_rad'], 2)}/rad BCD {p['BCD_N_per_rad']:.0f} N/rad "
                    f"(Fz_ref {p['fz_ref_N']:.0f} N); bound {p['bound_active']}; peak {f(p['peak_alpha_deg'], 2)} deg, "
                    f"p99 {f(p['p99_abs_deg'], 2)}, covered {p['peak_covered']}"
                    f"{' -> mu EXTRAPOLATED' if p['mu_extrapolated'] else ''}; in 3-7 deg {p['peak_in_3_7_deg']}; "
                    f"validity [{f(p['validity_deg'][0], 2)}, {f(p['validity_deg'][1], 2)}] deg")
                log("       rms per |alpha| bin (N / Fy/Fz / n): " + ", ".join(
                    f"{b} {f(v['rms_N'], 0)} / {f(v['rms_norm'], 3)} / {v['n']}" for b, v in p["rms_bins"].items()))
                log(f"       mean resid Fy/Fz by speed class {({k: round(v, 4) for k, v in p['mean_resid_norm_speed_class'].items()})}; "
                    f"by speed tercile {({k: round(v, 4) for k, v in p['mean_resid_norm_speed_tercile'].items()})} "
                    f"(edges {[round(e) for e in p['speed_tercile_edges_kmh']]} km/h); Sv {f(p['Sv_norm'], 4)}, "
                    f"Sh {f(p['Sh_deg'], 3)} deg")
            ci = rec[code]["cs_info"]
            log(f"    CS (information only): " + "; ".join(
                f"{x} finite {f(ci[x]['finite_share'])} p1/p5/p10/p50 {[round(q, 3) for q in ci[x]['p1_p5_p10_p50']]} "
                f"<-1 {f(ci[x]['share_lt_m1'])} <-2 {f(ci[x]['share_lt_m2'])} no-ref {ci[x]['n_no_reference_analysis']}"
                for x in ("f", "r")))
        for a in ("front", "rear"):
            si = rec["start_independence"][a]
            log(f"  start independence {a}: " + "; ".join(
                f"{k}: B {f(v['B'])} ({v['dB_rel'] * 100:+.3f} %), mu {f(v['mu'])} ({v['dmu_rel'] * 100:+.3f} %), "
                f"curve {v['curve_rms_rel_mu'] * 100:.3f} % of mu" for k, v in si.items()))
            lo = rec["lolo"][a]
            log(f"  LOLO {a} (n {lo['n_laps']} laps): " + "; ".join(
                f"{nm} full {f(lo[nm]['full'])} SE {f(lo[nm]['jackknife_se'], 4)} ({lo[nm]['se_rel'] * 100:.2f} %) "
                f"range [{f(lo[nm]['min'])}, {f(lo[nm]['max'])}]" for nm in ("B", "mu", "K")))
    log("\n## PREDICTIONS")
    for k, (v, why) in pred.items():
        log(f"- {k}: {v} -- {why}")
    log("\n## EXIT")
    log("L fails the chain function on: " + (str(exit_fail) if exit_fail else "no session"))


def _flat(o, p=""):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _flat(v, f"{p}.{k}")
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from _flat(v, f"{p}[{i}]")
    else:
        yield p, o


def main_reopen():
    global _LOG
    out_dir = os.path.join(OUT_DIR, "d4_reopen")
    os.makedirs(out_dir, exist_ok=True)
    _LOG = open(os.path.join(out_dir, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    f = lambda x, n=3: "nan" if x is None or x != x else f"{x:.{n}f}"  # noqa: E731
    try:
        for key, label, rel in SESSIONS:
            S = load_session(key, label, rel)
            run = chain(S, True)
            fits = {a: run["man"]["axles"][a] for a in ("front", "rear")}
            rec = {"label": label, "L": {"physical": {a: physical(S, fits[a], a) for a in ("front", "rear")},
                                         "chain": chain_function(S, run), "cs_info": cs_info(S, run)},
                   "start_independence": {a: start_independence(S, a, fits[a]) for a in ("front", "rear")},
                   "lolo": {a: lolo(S, a, fits[a]) for a in ("front", "rear")},
                   "d4": (run["raw"] or {}).get("plausibility_gate"),
                   "mu_extrapolated": {a: fits[a].get("mu_extrapolated") for a in ("front", "rear")}}
            rec = _jsonable(rec)
            old = json.load(open(os.path.join(OUT_DIR, f"fit_lit_{key}.json"), encoding="utf-8"))
            fo = dict(_flat({"phys": old["L"]["physical"], "si": old["start_independence"], "lolo": old["lolo"]}))
            fn = dict(_flat({"phys": rec["L"]["physical"], "si": rec["start_independence"], "lolo": rec["lolo"]}))
            diffs = [p for p in fo if fo[p] != fn.get(p)]
            with open(os.path.join(out_dir, f"d4_reopen_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(rec, fh, indent=1)
            render_traces(S, run, os.path.join(out_dir, f"traces_{key}_L.png"))
            c = rec["L"]["chain"]
            log("=" * 100)
            log(f"{label}: fit-level reproduction {len(fo) - len(diffs)} / {len(fo)} identical"
                + (f"; DIFFERS {diffs[:5]}" if diffs else ""))
            for a in ("front", "rear"):
                g = rec["d4"][a] if rec["d4"] else {}
                log(f"  D4 {a}: span {f(g.get('alpha_span_deg'), 2)} deg, K {f(g.get('stiffness_norm_per_rad'), 2)} / rad, "
                    f"mu {f(g.get('mu'))} (bound {g.get('mu_bound')}), peak {f(g.get('peak_deg'), 2)} deg covered "
                    f"{g.get('peak_covered')} -> mu extrapolated {g.get('mu_extrapolated')}; failed {g.get('failed') or 'none'}")
            log(f"  chain: status {c['status']} ({c['reason'] or '-'}); gate {c['gate']}; fallback {c['fallback']}; "
                f"R {c['R']}; hard-bound re-entries {c['hard_bound_reentries']}; NIS-window divergences "
                f"{c['nis_window_divergences']}; straight |beta| median/p95 {f(c['beta_straight_abs_deg']['median'])}/"
                f"{f(c['beta_straight_abs_deg']['p95'])} deg")
            ci = rec["L"]["cs_info"]
            log("  CS (information only): " + "; ".join(
                f"{x} finite {f(ci[x]['finite_share'])} p1/p5/p10/p50 {[round(q, 3) for q in ci[x]['p1_p5_p10_p50']]} "
                f"<-1 {f(ci[x]['share_lt_m1'])} <-2 {f(ci[x]['share_lt_m2'])} no-ref {ci[x]['n_no_reference_analysis']}"
                for x in ("f", "r")))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


def render_c_compare(S, new, old, axle, path):
    plt = _plt()
    from matplotlib.colors import LogNorm
    m = fit_pop(S, axle)
    a, y = np.degrees(S["alpha"][axle][m]), S["Fy"][axle][m] / S["Fz"][axle][m]
    p1, p99 = np.percentile(a, [1, 99])
    y_lim = 1.25 * float(np.percentile(np.abs(y), 99.9))
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.axvspan(p1, p99, color="#e5e7eb", alpha=0.6, lw=0, label="visited slip range (1st to 99th percentile)")
    ax.hist2d(a, y, bins=[np.linspace(-25, 25, 300), np.linspace(-y_lim, y_lim, 160)], cmap="Greys", cmin=1,
              norm=LogNorm())
    g = np.radians(np.linspace(-25, 25, 2000))
    ax.plot(np.degrees(g), new["mu"] * pacejka_lateral_force(g, new["B"], 1.0, 1.0, tfa.L_CURVATURE_E),
            color="#2a78d6", lw=2.0, label="literature-bounded fit, shape factor 1.0 (no peak)")
    ax.plot(np.degrees(g), old["mu"] * pacejka_lateral_force(g, old["B"], 1.3, 1.0, tfa.L_CURVATURE_E),
            color="#eb6834", lw=1.6, ls="--", label="literature-bounded fit, shape factor 1.3 (previous)")
    ax.set_xlim(-25, 25)
    ax.set_ylim(-y_lim, y_lim)
    ax.set_xlabel("axle slip angle from the simple geometric estimate [deg]")
    ax.set_ylabel(f"{axle} axle lateral force / measured axle load [-]")
    ax.set_title(f"{S['label']}, {axle} axle: load-normalised tyre curve, shape factor 1.0 against 1.3\n"
                 f"grey: sample density (log); shaded: visited slip range")
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main_c10():
    global _LOG
    out_dir = os.path.join(OUT_DIR, "c10")
    os.makedirs(out_dir, exist_ok=True)
    _LOG = open(os.path.join(out_dir, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    f = lambda x, n=3: "nan" if x is None or x != x else f"{x:.{n}f}"  # noqa: E731
    assert tfa.L_SHAPE_C == 1.0
    try:
        for key, label, rel in SESSIONS:
            S = load_session(key, label, rel)
            run = chain(S, True)
            fits = {a: run["man"]["axles"][a] for a in ("front", "rear")}
            old = json.load(open(os.path.join(OUT_DIR, f"fit_lit_{key}.json"), encoding="utf-8"))
            rec = {"label": label, "physical": {a: physical(S, fits[a], a) for a in ("front", "rear")},
                   "chain": chain_function(S, run), "cs_info": cs_info(S, run),
                   "start_independence": {a: start_independence(S, a, fits[a]) for a in ("front", "rear")},
                   "lolo": {a: lolo(S, a, fits[a]) for a in ("front", "rear")},
                   "d4": (run["raw"] or {}).get("plausibility_gate"), "curve_vs_c13": {}}
            for a in ("front", "rear"):
                o = old["L"]["physical"][a]
                p1, p99 = o["validity_deg"]
                g = np.radians(np.linspace(p1, p99, 2000))
                f_new = fits[a]["mu"] * pacejka_lateral_force(g, fits[a]["B"], 1.0, 1.0, tfa.L_CURVATURE_E)
                f_old = o["mu"] * pacejka_lateral_force(g, o["B"], 1.3, 1.0, tfa.L_CURVATURE_E)
                d = np.abs(f_new - f_old) / o["mu"]
                rec["curve_vs_c13"][a] = {"rms": float(np.sqrt(np.mean(d ** 2))), "max": float(d.max()),
                                          "B_c13": o["B"], "mu_c13": o["mu"]}
                render_c_compare(S, fits[a], o, a, os.path.join(out_dir, f"curve_{key}_{a}.png"))
            render_traces(S, run, os.path.join(out_dir, f"traces_{key}_L.png"))
            rec = _jsonable(rec)
            with open(os.path.join(out_dir, f"c10_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(rec, fh, indent=1)
            log("=" * 100)
            log(label)
            for a in ("front", "rear"):
                p, g4, cv = rec["physical"][a], (rec["d4"] or {}).get(a, {}), rec["curve_vs_c13"][a]
                log(f"  {a}: B {f(p['B'])} mu {f(p['mu'])} K {f(p['K_per_rad'], 2)}/rad BCD {p['BCD_N_per_rad']:.0f} N/rad; "
                    f"bound {p['bound_active']}; peak {g4.get('peak_note') or f(p['peak_alpha_deg'], 2)}; mu extrapolated "
                    f"{g4.get('mu_extrapolated')}; D4 failed {g4.get('failed') or 'none'}")
                log("     rms per |alpha| bin (N / Fy/Fz): " + ", ".join(
                    f"{b} {f(v['rms_N'], 0)} / {f(v['rms_norm'], 3)}" for b, v in p["rms_bins"].items()))
                si = rec["start_independence"][a]
                log("     start independence: " + "; ".join(
                    f"{k}: B {v['dB_rel'] * 100:+.3f} %, mu {v['dmu_rel'] * 100:+.3f} %, curve {v['curve_rms_rel_mu'] * 100:.3f} % of mu"
                    for k, v in si.items()))
                lo = rec["lolo"][a]
                log(f"     LOLO (n {lo['n_laps']}): " + "; ".join(
                    f"{nm} SE {lo[nm]['se_rel'] * 100:.2f} % range [{f(lo[nm]['min'])}, {f(lo[nm]['max'])}]" for nm in ("B", "mu", "K")))
                log(f"     curve vs C 1.3 fit on the visited range: RMS {cv['rms'] * 100:.2f} %, max {cv['max'] * 100:.2f} % of D "
                    f"(C 1.3: B {f(cv['B_c13'])} mu {f(cv['mu_c13'])})")
            c = rec["chain"]
            log(f"  chain: status {c['status']} ({c['reason'] or '-'}); gate {c['gate']}; fallback {c['fallback']}; R {c['R']}; "
                f"hard-bound re-entries {c['hard_bound_reentries']}; NIS-window divergences {c['nis_window_divergences']}; "
                f"straight |beta| {f(c['beta_straight_abs_deg']['median'])}/{f(c['beta_straight_abs_deg']['p95'])} deg")
            ci = rec["cs_info"]
            log("  CS (information only): " + "; ".join(
                f"{x} finite {f(ci[x]['finite_share'])} p1/p5/p10/p50 {[round(q, 3) for q in ci[x]['p1_p5_p10_p50']]} "
                f"<-2 {f(ci[x]['share_lt_m2'])}" for x in ("f", "r")))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    if "--c10" in sys.argv[1:]:
        sys.exit(main_c10())
    sys.exit(main_reopen() if "--reopen-d4" in sys.argv[1:] else main())
