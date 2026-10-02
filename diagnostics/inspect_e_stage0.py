# Experiment (e) Stage 0: rear-axle force-demand occupancy (read-only).
#
# PRE-REGISTRATION (PLAN.md EXPERIMENT PLAN (e), ruled 2026-10-01; P3 dropped,
# "none named"). Fixed before any run; implementation choices the proposal
# left open are marked [impl] and were fixed here, also before any run.
#
# Sessions: dubai, pr_us_v2, v3 (Le Mans banned, NBR out until the damper
# offset is resolved). Inputs through the (d2) harness, per-file units.
#
# Quantity (K&N Eq. 8.22 Kamm limit, RCVD sec. 2.7 friction circle):
#   mu_used,r = sqrt(Fx_r^2 + Fy_r^2) / Fz_r.  The rear is unsteered, so the
#   resultant magnitude does not depend on the tyre-axis rotation: no alpha,
#   no beta, no kappa enters it. Fx_r, Fy_r, Fz_r are production quantities
#   (estimate_longitudinal_forces, estimate_lateral_forces Fy_r_filt,
#   estimate_vertical_loads on the measured cascade).
#   [impl] Fx_r, Fz_r, kappa_r low-passed like the chain: LS _filtered at
#   cs_filter_cutoff_hz (Fy_r_filt already is).
#   Envelope = the session's own p99 of mu_used,r over R (RCVD 9.3/9.4: an
#   empirical envelope, not an assumed mu); occupancy = mu_used,r / envelope.
#   Pure-lateral under-read = |F_r| / |Fy_r| - 1.
#   [impl] under-read read only where |Fy_r| >= 10 % of the session p99 |Fy_r|
#   (the ratio is undefined near Fy = 0); the unguarded median is reported as
#   a labelled sensitivity, never scored.
#   Vehicle g-g occupancy (RCVD 9.1: cannot attribute to an axle) is reported
#   as a cross-check only.
#
# Populations:
#   R: production base mask (moving, analysis laps, off-kerb) AND both rear
#      corners on the Level-4 damper tier (N2).
#   exit: R inside exit_4..exit_5 of analysis-lap corners.
#   combined traction: R & throttle > 50 % & log_pbrake_f <= 5 bar & |a_y| >= 0.8 g
#      (as in the proposal-turn probe). [impl] throttle = RAW ecu_aps: full
#      pedal reads 104-106 %, which the parser's range gate flags; the raw
#      channel is live on all three files (census 2026-10-01).
#   straight: R & |a_y| <= 0.15 g & |yaw rate| <= 3 deg/s & log_pbrake_f <= 5 bar.
#
# Criteria:
#   P1: Spearman rho(Fx_r/Fz_r, kappa_r) on combined traction >= 0.5.
#   P2 (dubai only): on >= 75 % of TC events rear occupancy >= the session's
#       exit p75. [impl] event = contiguous run of ecu_B_tc_act inside R;
#       event occupancy = its maximum.
#   P4: CS and occupancy traces rendered per session and inspected before any
#       exit (diagnostics/plots/e_stage0/).
#   N1: exit-median under-read recomputed with Fx_r from sclu_acc_x instead of
#       log_acc_x; |difference| >= 5 pp -> longitudinal leg unresolvable on
#       that session.
#   N2: scored only on R (both rear gauges on the damper tier).
#   N3: kappa_r "no usable signal" if median(combined) - median(straight)
#       < 3 x straight robust sd (1.4826 MAD), in pp.
#   N4: ecu_slip_act magnitude -- already met in the proposal turn; unused.
# No aggregation rule across P1/P2/P4 was pre-registered: each criterion is
# reported per session and the exit ruling stays with the author.
#
# Q1 (Stage 0 ruling 1, 2026-10-01): mode dubai_l1 -- a LABELLED, NEVER
# SCORED mechanism look at rear occupancy against the TC events on Dubai,
# whose rear-axle Fz is the Level-1 total without aero (RR reconstructed).
# Same quantity and P2 construction, N2 lifted; outputs carry L1_UNSCORED.
#
# Modes: python diagnostics/inspect_e_stage0.py run <key> | dubai_l1 | report

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks

OUT_DIR = "diagnostics/plots/e_stage0"
KEYS = ("dubai", "pr_us_v2", "v3")
TC_KEY = "dubai"
G = 9.81
UR_FY_FLOOR = 0.10      # [impl] under-read guard, fraction of p99 |Fy_r|
P1_BAR, P2_BAR, P2_PCTL = 0.5, 0.75, 75
N1_BAR_PP, N3_FACTOR = 5.0, 3.0
TRAC_APS, TRAC_AY_G, BRAKE_BAR = 50.0, 0.8, 5.0
STR_AY_G, STR_YAW_DEG = 0.15, 3.0
G_FACTOR = {"G": G, "g": G, "m/s/s": 1.0, "m/s^2": 1.0}


def _runs(mask):
    d = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def _raw_on(path, names, t):
    out = {}
    for n, (tt, vv, h) in read_raw_blocks(path, names).items():
        unit = h[h.find("[") + 1:h.find("]")] if "[" in h else ""
        out[n] = (np.interp(t, tt, vv), unit)
    return out


def _exit_mask(t, corners, laps):
    an = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    m = np.zeros(t.size, bool)
    for c in corners:
        if c.get("lap_number") in an:
            t0, t1 = c["segments"]["exit_4"][0], c["segments"]["exit_5"][1]
            m |= (t >= t0) & (t <= t1)
    return m


def _underread(fx, fy, fy_floor):
    with np.errstate(divide="ignore", invalid="ignore"):
        ur = np.hypot(fx, fy) / np.abs(fy) - 1.0
    return ur, np.abs(fy) >= fy_floor


def run(key, unscored_level1=False):
    from modules.stability_analysis import estimate_lateral_forces, estimate_vertical_loads, load_car_data
    from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
    from modules.longitudinal_stiffness import _filtered
    from modules.tyre_fit_auto import _base_mask
    from diagnostics.trace_render import render_cs_traces

    params, data, state = d2._state(key)
    ch, laps, corners = data["channels"], data.get("laps", []), data.get("corners", [])
    t, sr = state["time"], state["sample_rate_hz"]
    fc = params["stability_estimation"]["cs_filter_cutoff_hz"]

    def lp(x):
        y = _filtered(np.asarray(x, float), sr, fc)
        return np.where(np.isfinite(x), y, np.nan)

    forces = estimate_lateral_forces(state, params)
    fz = estimate_vertical_loads(state, forces, params, channels=ch, car_data=load_car_data())
    lf = estimate_longitudinal_forces(state, ch, params)
    kap = estimate_slip_ratio(state, ch, params)
    raw = _raw_on(d2.sessions()[key]["path"], ["ecu_aps", "sclu_acc_x", "ecu_B_tc_act"], t)
    ax_alt = raw["sclu_acc_x"][0] * G_FACTOR[raw["sclu_acc_x"][1]]
    lf_alt = estimate_longitudinal_forces(dict(state, ax_mps2=ax_alt), ch, params)

    moving = state["moving_mask"]
    fx = lp(np.where(moving, lf["fx_r_N"], 0.0))
    fx_alt = lp(np.where(moving, lf_alt["fx_r_N"], 0.0))
    fy = forces["Fy_r_filt"]
    fzr = lp(fz["fz_r_N"])
    kr = lp(kap["kappa_r"]) * 100.0
    src = fz.get("vertical_load_source_per_sample") or {}
    damper_r = np.ones(t.size, bool) if (not src or unscored_level1) else \
        (np.asarray(src["rl"]) == "damper") & (np.asarray(src["rr"]) == "damper")
    tag = "_L1_UNSCORED" if unscored_level1 else ""

    pb = np.interp(t, np.asarray(ch["log_pbrake_f"]["time"], float), np.asarray(ch["log_pbrake_f"]["data"], float))
    aps = raw["ecu_aps"][0]
    tc = raw["ecu_B_tc_act"][0] > 0.5
    ay_g = state["ay_mps2"] / G
    yaw_deg = np.degrees(state["yaw_rate_radps"])

    base = _base_mask(state, laps)
    R = base & damper_r & np.isfinite(fx) & np.isfinite(fy) & np.isfinite(fzr) & (fzr > 0)
    ex = R & _exit_mask(t, corners, laps)
    comb = R & (aps > TRAC_APS) & (pb <= BRAKE_BAR) & (np.abs(ay_g) >= TRAC_AY_G)
    straight = R & (np.abs(ay_g) <= STR_AY_G) & (np.abs(yaw_deg) <= STR_YAW_DEG) & (pb <= BRAKE_BAR)

    mu = np.hypot(fx, fy) / fzr
    env = float(np.nanpercentile(mu[R], 99))
    occ = mu / env
    fy_floor = UR_FY_FLOOR * float(np.nanpercentile(np.abs(fy[R]), 99))
    ur, guard = _underread(fx, fy, fy_floor)
    ur_alt, _ = _underread(fx_alt, fy, fy_floor)
    ax_f, ay_f = lp(state["ax_mps2"]), lp(state["ay_mps2"])
    gg = np.hypot(ax_f, ay_f)
    occ_v = gg / float(np.nanpercentile(gg[R], 99))

    from scipy.stats import spearmanr
    pm = comb & np.isfinite(kr)
    rho = float(spearmanr(fx[pm] / fzr[pm], kr[pm]).correlation) if pm.sum() > 30 else float("nan")

    ks = kr[straight & np.isfinite(kr)]
    sd = float(1.4826 * np.median(np.abs(ks - np.median(ks))))
    exc = float(np.nanmedian(kr[pm]) - np.median(ks))

    exg = ex & guard
    ur_med, ur_alt_med = float(np.nanmedian(ur[exg])), float(np.nanmedian(ur_alt[exg]))
    rec = {
        "key": key, "sample_rate_hz": float(sr),
        "n": {"base": int(base.sum()), "R": int(R.sum()), "exit": int(ex.sum()), "exit_guarded": int(exg.sum()),
              "combined": int(pm.sum()), "straight": int(ks.size)},
        "rear_damper_share_of_base": float(np.mean(damper_r[base])) if base.any() else float("nan"),
        "envelope_mu_p99": env,
        "exit": {"occ_p50": float(np.nanmedian(occ[ex])), "occ_p75": float(np.nanpercentile(occ[ex], 75)),
                 "occ_vehicle_p50": float(np.nanmedian(occ_v[ex])),
                 "fx_share_p50": float(np.nanmedian((fx / np.hypot(fx, fy))[ex])),
                 "underread_p50_pct": 100 * ur_med, "underread_p90_pct": 100 * float(np.nanpercentile(ur[exg], 90)),
                 "underread_p50_unguarded_pct_SENSITIVITY": 100 * float(np.nanmedian(ur[ex]))},
        "P1": {"rho": rho, "bar": P1_BAR, "pass": bool(rho >= P1_BAR)},
        "N1": {"underread_p50_logacc_pct": 100 * ur_med, "underread_p50_scluacc_pct": 100 * ur_alt_med,
               "diff_pp": 100 * abs(ur_med - ur_alt_med), "bar_pp": N1_BAR_PP,
               "longitudinal_leg_unresolvable": bool(100 * abs(ur_med - ur_alt_med) >= N1_BAR_PP)},
        "N3": {"straight_robust_sd_pp": sd, "combined_excursion_pp": exc, "ratio": exc / sd if sd > 0 else float("nan"),
               "no_usable_signal": bool(exc < N3_FACTOR * sd)},
    }
    if key == TC_KEY:
        thr = float(np.nanpercentile(occ[ex], P2_PCTL))
        ev = [(a, b) for a, b in _runs(tc & R)]
        ev_occ = [float(np.nanmax(occ[a:b])) for a, b in ev]
        hits = sum(o >= thr for o in ev_occ)
        rec["P2"] = {"events": len(ev), "event_samples": int((tc & R).sum()), "tc_samples_in_base": int((tc & base).sum()),
                     "threshold_exit_p75": thr, "hits": int(hits),
                     "fraction": hits / len(ev) if ev else float("nan"), "bar": P2_BAR,
                     "pass": bool(ev and hits / len(ev) >= P2_BAR), "event_occ": ev_occ}

    if unscored_level1:
        rec["LABEL"] = ("LEVEL-1 REAR LOADS (no aero), N2 lifted -- mechanism look only, NEVER SCORED "
                        "((e) Stage 0 ruling 1)")
        rec["rear_tiers"] = {c: sorted(set(np.asarray(src[c]).tolist())) for c in ("rl", "rr")} if src else None
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"stage0_{key}{tag}.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    if unscored_level1:
        _render_occ(os.path.join(OUT_DIR, f"occupancy_{key}{tag}.png"), t, state["s_m"], occ,
                    np.where(guard, ur, np.nan), np.where(R, 1, np.nan), laps, corners, tc & R,
                    f"{key} LEVEL-1 rear loads, UNSCORED")
        print(json.dumps({k: v for k, v in rec.items() if k != "P2"} | {"P2": {y: v for y, v in rec["P2"].items()}},
                         indent=1))
        return

    # P4 render: standard CS set on the production chain, plus occupancy
    beta = d2._resolve(key, params, data, state)[0]
    ds = d2._downstream(params, data, state, beta)
    render_cs_traces(os.path.join(OUT_DIR, f"cs_{key}.png"), t, state["s_m"], ds["cs_f"], ds["cs_r"], laps,
                     corners, f"(e) Stage 0 {key}: production CS_ratio")
    _render_occ(os.path.join(OUT_DIR, f"occupancy_{key}.png"), t, state["s_m"], occ, np.where(guard, ur, np.nan),
                np.where(R, 1, np.nan), laps, corners, tc & R if key == TC_KEY else None, key)
    print(json.dumps(rec, indent=1))


def _render_occ(out_path, t, s_m, occ, ur, rmask, laps, corners, tc, key):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND, REF
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    an_nums = {l["lap_number"] for l in an}
    lap_len = np.nanmax(s_m[(t >= an[0]["start_time"]) & (t <= an[0]["end_time"])])
    brackets = sorted({(c["bracket_start_m"], c["bracket_end_m"]) for c in corners
                       if c.get("bracket_start_m") is not None and c["lap_number"] in an_nums})
    fig, axes = plt.subplots(2, 1, figsize=(max(14, lap_len / 400), 7), sharex=True, constrained_layout=True)
    panels = ((axes[0], occ * rmask, "rear occupancy (mu_used / p99)", (0, 1.2)),
              (axes[1], np.clip(ur, 0, 1) * rmask, "pure-lateral under-read |F|/|Fy|-1", (0, 1.0)))
    for ax, arr, lab, (lo, hi) in panels:
        for b0, b1 in brackets:
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(1.0 if hi > 1 else 0.1, color=REF, lw=0.8, ls="--", zorder=1)
        for l in an:
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            ax.plot(s_m[m], arr[m], color=INK, lw=0.5, alpha=0.6, zorder=2)
        ax.set_ylim(lo, hi)
        ax.set_ylabel(lab, color=INK, fontsize=9)
        ax.grid(axis="y", color=BAND, lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    if tc is not None and tc.any():
        axes[0].plot(s_m[tc], np.clip(occ[tc], 0, 1.2), "o", ms=2.2, mew=0, color="#b91c1c", label="TC active")
        axes[0].legend(loc="lower left", frameon=False, fontsize=8)
    axes[1].set_xlabel("lap distance [m]", color=INK)
    fig.suptitle(f"(e) Stage 0 {key}: rear force-demand occupancy and under-read, per analysis lap (n={len(an)}); "
                 f"grey = corner brackets; dashed: occupancy 1 = session p99, under-read 0.1 = K&N ellipse bar",
                 color=INK, fontsize=10)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def report():
    lines = []
    for k in KEYS:
        p = os.path.join(OUT_DIR, f"stage0_{k}.json")
        if os.path.exists(p):
            with open(p) as fh:
                r = json.load(fh)
            lines.append(json.dumps({x: r[x] for x in r if x != "P2"} | ({"P2": {y: v for y, v in r["P2"].items()
                                                                               if y != "event_occ"}} if "P2" in r else {})))
    with open(os.path.join(OUT_DIR, "stage0.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2])
    elif sys.argv[1] == "dubai_l1":
        run("dubai", unscored_level1=True)
    else:
        report()
