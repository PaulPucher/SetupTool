# Werner Eq. 4.3 completion, v1 consumer (read-only diagnostic).
# Per corner-phase on both real sessions: |D_psi * psi_dot| (damping part,
# modules/yaw_damping.py) against |Iz * psi_ddot| (inertial part, Module 5's
# own mz_inertial_Nm, reused, not recomputed). Werner sec. 4.5.2 p.57 found
# the inertial part negligible next to the damping part for his car; this
# checks whether that reproduces here. Negative D_psi (post-peak axle) is
# reported as its own population. Nothing here feeds Module 5, the payload
# or the UI; no config is written.
#
# Production chain as tests/conftest.py runs it: ekf_auto_pacejka via
# resolve_sideslip_beta, cap=1 config defaults (golden convention).
#
# Run from the repo root:  python -m diagnostics.inspect_yaw_damping_eq43

import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, estimate_yaw_moment_stability,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.yaw_damping import estimate_yaw_damping, yaw_damping_moment

SESSIONS = {
    "dubai": "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt",
    "v3": "C:/UNI/Bachelorarbeit/Setuptool_local/GT3_PRC_MLA-v3.txt",
}
OUT_DIR = "diagnostics/plots_yaw_damping"
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
# Reading bars for Werner's claim, fixed before looking at the numbers:
# "dominates" = damping/inertia median ratio > 1, "dwarfs" = > 10
# (his "betraglich zu vernachlaessigen", negligible in magnitude).
DOMINATES, DWARFS = 1.0, 10.0
KNOWN = {"dubai": {"C4 front saturation": (4, "exit_4"), "C3 traction (rear)": (3, "exit_5")}}

COLOR_RATIO, COLOR_NEG, TEXT, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=8)


def run(name):
    data = parse_csv(SESSIONS[name])
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
    state = prepare_vehicle_state(data["channels"], params)
    beta, _, gate, fb, _ = resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka", csv_path=SESSIONS[name])
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    vp, se = params["vehicle"], params["stability_estimation"]
    cw = vp["corner_weights"]
    w_f, w_r = cw["FL_kg"] + cw["FR_kg"], cw["RL_kg"] + cw["RR_kg"]
    front_fraction = w_f / (w_f + w_r)
    # same static split as estimate_lateral_forces: l_f = L * rear share
    l_f = vp["wheelbase_m"] * (1.0 - front_fraction)
    l_r = vp["wheelbase_m"] * front_fraction
    damp = estimate_yaw_damping(cs["C_alpha_f"], cs["C_alpha_r"], l_f, l_r,
                                state["v_mps"], se["moving_speed_min_mps"])
    mz_damp = yaw_damping_moment(damp["D_psi"], state["yaw_rate_radps"])
    mz_inert = stab["mz_inertial_Nm"]

    t = state["time"]
    ok = state["moving_mask"].copy()
    if state.get("kerb_mask") is not None:
        ok &= ~state["kerb_mask"]
    min_n = se["cs_phase_min_valid_samples"]
    valid_laps = {l["lap_number"] for l in data.get("laps", []) if l.get("is_valid_for_analysis")}

    rows = []
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is None or c.get("lap_number") not in valid_laps:
            continue
        for ph in PHASES:
            t0, t1 = c["segments"][ph]
            sl = slice(int(np.searchsorted(t, t0, "left")), int(np.searchsorted(t, t1, "right")))
            m = ok[sl] & np.isfinite(mz_damp[sl]) & np.isfinite(mz_inert[sl])
            if int(m.sum()) < min_n:
                continue  # below 4b's own per-phase floor: no signal
            ad = float(np.median(np.abs(mz_damp[sl][m])))
            ai = float(np.median(np.abs(mz_inert[sl][m])))
            dps = damp["D_psi"][sl][m]
            rows.append({
                "stable_id": sid, "lap": c["lap_number"], "phase": ph, "n": int(m.sum()),
                "median_abs_mz_damping_Nm": ad, "median_abs_mz_inertial_Nm": ai,
                "ratio": ad / ai if ai > 0 else np.nan,
                "median_D_psi": float(np.median(dps)),
                "neg_D_psi_fraction": float(np.mean(dps < 0)),
                "median_D_front": float(np.nanmedian(damp["D_psi_front"][sl][m])),
                "median_D_rear": float(np.nanmedian(damp["D_psi_rear"][sl][m])),
            })
    return {"gate": gate, "fallback": fb, "l_f": l_f, "l_r": l_r, "rows": rows,
            "n_moving": int(ok.sum()), "n_defined": int(np.sum(ok & np.isfinite(damp["D_psi"])))}


def report(name, res):
    rows = res["rows"]
    r = np.array([x["ratio"] for x in rows if np.isfinite(x["ratio"])])
    print(f"\n===== {name}: EKF gate {res['gate']['verdict']} {res['gate']['health_score']:.4f}, "
          f"fallback={res['fallback']}; l_f={res['l_f']:.3f} m, l_r={res['l_r']:.3f} m")
    print(f"  D_psi defined on {res['n_defined']}/{res['n_moving']} moving non-kerb samples "
          f"({100.0 * res['n_defined'] / max(res['n_moving'], 1):.1f} %)")
    print(f"  corner-phase instances with signal: {len(rows)}")
    print(f"  ratio |Mz_damping|/|Mz_inertial| (medians per instance): p10 {np.percentile(r, 10):.2f}, "
          f"p50 {np.median(r):.2f}, p90 {np.percentile(r, 90):.2f}; "
          f"> {DOMINATES:g}: {100 * np.mean(r > DOMINATES):.0f} %, > {DWARFS:g}: {100 * np.mean(r > DWARFS):.0f} %")
    for ph in PHASES:
        rp = np.array([x["ratio"] for x in rows if x["phase"] == ph and np.isfinite(x["ratio"])])
        if rp.size:
            print(f"    {ph:15s} n={rp.size:3d} median ratio {np.median(rp):.2f}")
    neg = [x for x in rows if x["median_D_psi"] < 0]
    print(f"  NEGATIVE D_psi population (instance median < 0): {len(neg)} of {len(rows)}")
    for x in sorted(neg, key=lambda x: (x["stable_id"], x["lap"], x["phase"])):
        axle = "front" if x["median_D_front"] < 0 and x["median_D_rear"] >= 0 else \
               "rear" if x["median_D_rear"] < 0 and x["median_D_front"] >= 0 else "both/mixed"
        print(f"    C{x['stable_id']} L{x['lap']} {x['phase']}: median D_psi {x['median_D_psi']:.0f} N m s/rad, "
              f"negative-axle {axle}, {100 * x['neg_D_psi_fraction']:.0f} % of samples negative")
    for label, (sid, ph) in KNOWN.get(name, {}).items():
        hits = [x for x in rows if x["stable_id"] == sid and x["phase"] == ph]
        negs = [x for x in hits if x["median_D_psi"] < 0]
        print(f"  cross-ref {label} (C{sid} {ph}): {len(negs)} of {len(hits)} laps negative D_psi; "
              f"per-lap median D_psi {[round(x['median_D_psi']) for x in hits]}, "
              f"front/rear contributions {[(round(x['median_D_front']), round(x['median_D_rear'])) for x in hits]}")


def plot(name, res):
    rows = res["rows"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    r = np.array([x["ratio"] for x in rows if np.isfinite(x["ratio"]) and x["ratio"] > 0])
    axes[0].hist(np.log10(r), bins=30, color=COLOR_RATIO)
    for v, lab in ((DOMINATES, "damping = inertia"), (DWARFS, "damping = 10 x inertia")):
        axes[0].axvline(np.log10(v), color=TEXT, linewidth=0.8, linestyle=":")
        axes[0].text(np.log10(v), axes[0].get_ylim()[1] * 0.95, " " + lab, fontsize=7, color=TEXT, va="top")
    axes[0].set_xlabel("log10( median|D_psi*psi_dot| / median|Iz*psi_ddot| ) per corner-phase", fontsize=8)
    axes[0].set_ylabel("corner-phase instances", fontsize=8)
    axes[0].set_title(f"{name}: damping vs inertial yaw moment", fontsize=9, color=TEXT)
    _style(axes[0])
    d = np.array([x["median_D_psi"] for x in rows])
    edges = np.histogram_bin_edges(d, bins=40)  # shared, so the overlay lines up
    axes[1].hist(d[d >= 0], bins=edges, color=COLOR_RATIO, label="D_psi >= 0")
    axes[1].hist(d[d < 0], bins=edges, color=COLOR_NEG, label=f"negative D_psi, post-peak (n={int(np.sum(d < 0))})")
    axes[1].axvline(0.0, color=TEXT, linewidth=0.8)
    axes[1].set_xlabel("median D_psi per corner-phase (N m s/rad)", fontsize=8)
    axes[1].set_title(f"{name}: D_psi distribution", fontsize=9, color=TEXT)
    axes[1].legend(fontsize=7, frameon=False)
    _style(axes[1])
    fig.tight_layout()
    path = os.path.join(OUT_DIR, f"yaw_damping_vs_inertia_{name}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    results_path = os.path.join(OUT_DIR, "yaw_damping_results.json")
    if "--replot" in sys.argv:  # redraw figures from the saved results, no recompute
        with open(results_path) as fh:
            for name, res in json.load(fh).items():
                print(f"figure: {plot(name, res)}")
        return
    out = {}
    for name in SESSIONS:
        res = run(name)
        report(name, res)
        print(f"  figure: {plot(name, res)}")
        out[name] = res
    with open(results_path, "w") as fh:
        json.dump(out, fh, indent=1, default=float)


if __name__ == "__main__":
    main()
