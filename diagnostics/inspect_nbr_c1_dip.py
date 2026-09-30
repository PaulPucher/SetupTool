# DIAGNOSTIC, read-only. Follow-up approved 2026-09-30 ((f) exit ruling 4),
# recorded under PLAN.md PARKED "NBR C1 EKF entry lag". The front CS_ratio
# sample minimum at NBR C1 sits at 612-626 m on every analysis lap of JST 21
# and JST 29 ((f) Part 4). What is physically at that position, which input
# moves first, and does a kerb-mask edge sit there?
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_nbr_c1_dip
# Raw inputs from the files (census 2026-09-30: log_susp_travel_* alive at
# 100 Hz on both, ride height FL failed); CS_ratio, alpha and the CS window
# length from the new_laps cache (production EKF Pacejka path), time-aligned
# with the parsed state (asserted). Figures in diagnostics/plots/nbr_c1_dip/
# (gitignored).
#
# RULES (fixed before the run)
# - Stretch 600-640 m (context 560-700 m in the figure), every analysis lap.
# - Dip = the lap's front CS_ratio sample minimum inside the stretch.
# - "Moves first": per lap and channel, the position of the largest |rate|
#   (per metre, 4 Hz zero-phase low-pass as in (c)) inside 590-640 m, read
#   relative to the dip position; negative = before the dip. A heuristic
#   ordering, not a causal test.
# - Kerb: mask edges (production az rule) anywhere in 560-700 m.
# - CS window at the dip sample: reconstruct_cs_window_start (the live
#   loop's start), window start/length in metres, kerb samples inside it,
#   and the sign of the alpha_f and Fy_f changes across it: a negative
#   CS_ratio needs Fy and alpha to move in opposite directions.
# - Fy_f = m ay a_frac + Iz psidd / L with psidd the raw yaw-rate gradient
#   (estimate_lateral_forces): the psidd term is reported separately.

import gzip
import os
import pickle

import numpy as np

from core.yaw_pooling import lowpass
from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
from diagnostics.gated_kinematic_common import FIXED_CAP

KEYS = {"nbr_jst_problem": "JST 21", "nbr_jst_solution": "JST 29"}
STRETCH = (600.0, 640.0)
RATE_WIN = (590.0, 640.0)
CONTEXT = (560.0, 700.0)
FC_HZ = 4.0
OUT_DIR = "diagnostics/plots/nbr_c1_dip"
DAMPERS = ["log_susp_travel_fl", "log_susp_travel_fr", "log_susp_travel_rl", "log_susp_travel_rr"]


def load(key):
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import estimate_lateral_forces, load_parameters, prepare_vehicle_state
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    data = parse_csv(SESSIONS[key])
    st = prepare_vehicle_state(data["channels"], params)
    forces = estimate_lateral_forces(st, params)
    with gzip.open(os.path.join("diagnostics/plots/new_laps/cache", f"{key}.pkl.gz"), "rb") as fh:
        nl = pickle.load(fh)
    assert nl["time"].tobytes() == st["time"].tobytes(), "cache and parsed state must share the time base"
    t = st["time"]
    damp = {}
    for n in DAMPERS:
        c = data["channels"].get(n)
        if c is not None and c.get("time") is not None and c.get("quality") not in ("missing", "failed"):
            damp[n] = np.interp(t, c["time"], np.asarray(c["data"], float)) * 1000.0   # m -> mm
    vp = params["vehicle"]
    Iz, L = vp["yaw_inertia_kgm2"], vp["wheelbase_m"]
    return {"st": st, "forces": forces, "nl": nl, "damp": damp, "params": params, "Iz": Iz, "L": L}


def per_lap(sess):
    from modules.stability_analysis import reconstruct_cs_window_start
    st, nl, forces, damp, params = sess["st"], sess["nl"], sess["forces"], sess["damp"], sess["params"]
    t, s = st["time"], st["s_m"]
    sr = st["sample_rate_hz"]
    cs_f, cs_r = nl["cs"]["A_baseline"]
    alpha_f = nl["alpha_f"]
    se = params["stability_estimation"]
    kerb = st["kerb_mask"]
    psidd = np.gradient(st["yaw_rate_radps"], t)
    lp = {"ay": lowpass(st["ay_mps2"], FC_HZ, sr), "yaw": lowpass(np.degrees(st["yaw_rate_radps"]), FC_HZ, sr),
          "steer": lowpass(np.degrees(st["steer_sw_rad"]), FC_HZ, sr), "speed": lowpass(st["v_mps"] * 3.6, FC_HZ, sr),
          "az": lowpass(st["az_g"], FC_HZ, sr)}
    lp.update({n.replace("log_susp_travel_", "damp_"): lowpass(v, FC_HZ, sr) for n, v in damp.items()})
    out = []
    for lap in [l for l in nl["laps"] if l.get("is_valid_for_analysis")]:
        m = (t >= lap["start_time"]) & (t <= lap["end_time"]) & np.isfinite(s)
        idx = np.flatnonzero(m)
        ss = s[idx]
        win = idx[(ss >= STRETCH[0]) & (ss <= STRETCH[1])]
        ctx = idx[(ss >= CONTEXT[0]) & (ss <= CONTEXT[1])]
        fin = win[np.isfinite(cs_f[win])]
        r = {"lap": lap["lap_number"], "ctx": ctx, "finite_share": fin.size / max(win.size, 1)}
        if fin.size == 0:
            out.append(r)
            continue
        i = fin[np.argmin(cs_f[fin])]
        r.update({"i": int(i), "dip_s": float(s[i]), "dip": float(cs_f[i]),
                  "cs_r_at": float(cs_r[i]) if np.isfinite(cs_r[i]) else None,
                  "speed": float(st["v_mps"][i] * 3.6), "ay": float(st["ay_mps2"][i]),
                  "yaw": float(np.degrees(st["yaw_rate_radps"][i])), "steer": float(np.degrees(st["steer_sw_rad"][i])),
                  "az": float(st["az_g"][i])})
        rw = idx[(ss >= RATE_WIN[0]) & (ss <= RATE_WIN[1])]
        peaks = {}
        for name, x in lp.items():
            rate = np.abs(np.gradient(x[rw], s[rw]))
            j = int(np.nanargmax(rate))
            peaks[name] = float(s[rw][j] - s[i])
        r["peaks_rel"] = peaks
        kc = kerb[ctx]
        edges = np.flatnonzero(np.diff(kc.astype(np.int8)))
        r["kerb_edges_s"] = [float(s[ctx][e]) for e in edges]
        r["kerb_in_stretch"] = int(kerb[win].sum())
        start = reconstruct_cs_window_start(alpha_f, int(i), nl["min_window"], se["cs_min_slip_angle_span_rad"],
                                            s_m=s, max_window_m=se["cs_max_window_m"])
        wsl = slice(start, int(i) + 1)
        r.update({"win_start_s": float(s[start]), "win_len_m": float(s[i] - s[start]), "win_n": int(i - start),
                  "win_kerb": int(kerb[wsl].sum()),
                  "d_alpha_deg": float(np.degrees(alpha_f[i] - alpha_f[start])),
                  "d_fy_kN": float((forces["Fy_f_filt"][i] - forces["Fy_f_filt"][start]) / 1000.0)})
        # psidd term's share of Fy_f over the window (Iz psidd / L)
        # raw (unfiltered) term; Fy_f_filt low-passes the sum
        term = sess["Iz"] * psidd / sess["L"] / 1000.0
        r["psidd_term_kN_range"] = (float(np.nanmin(term[wsl])), float(np.nanmax(term[wsl])))
        out.append(r)
    return out, lp


def figure(key, sess, rows, lp, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    st, nl = sess["st"], sess["nl"]
    s = st["s_m"]
    cs_f = nl["cs"]["A_baseline"][0]
    panels = [("front CS_ratio", lambda c: cs_f[c]), ("ay (m/s^2)", lambda c: lp["ay"][c]),
              ("yaw rate (deg/s)", lambda c: lp["yaw"][c]),
              ("yaw accel raw (deg/s^2)", lambda c: np.degrees(np.gradient(st["yaw_rate_radps"], st["time"]))[c]),
              ("steering (deg)", lambda c: lp["steer"][c]), ("speed (km/h)", lambda c: lp["speed"][c]),
              ("az (g), kerb samples x", lambda c: st["az_g"][c])]
    dks = [k for k in lp if k.startswith("damp_")]
    panels += [(f"{k} (mm)", (lambda kk: (lambda c: lp[kk][c]))(k)) for k in dks]
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 1.6 * len(panels)), sharex=True)
    for ax, (lab, fn) in zip(axes, panels):
        for r in rows:
            c = r["ctx"]
            line, = ax.plot(s[c], fn(c), lw=0.8, label=f"lap {r['lap']}")
            if lab.startswith("az"):
                kc = c[st["kerb_mask"][c]]
                ax.plot(s[kc], st["az_g"][kc], "x", ms=3, color=line.get_color())
            if "dip_s" in r:
                ax.axvline(r["dip_s"], color=line.get_color(), lw=0.5, ls=":")
        ax.axvspan(*STRETCH, color="#e5e7eb", zorder=0)
        ax.set_ylabel(lab, fontsize=7)
        ax.tick_params(labelsize=7)
    axes[0].set_ylim(-3, 1.2)
    axes[0].legend(fontsize=6, ncol=len(rows), loc="lower left")
    axes[-1].set_xlabel("lap distance (m); grey = 600-640 m; dotted = each lap's front CS minimum", fontsize=8)
    fig.suptitle(f"{KEYS[key]} NBR C1: inputs across the fixed-position CS dip", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for key, label in KEYS.items():
        sess = load(key)
        rows, lp = per_lap(sess)
        print(f"\n== {label}  (dampers alive: {', '.join(sess['damp']) or 'none'}; Iz {sess['Iz']}, L {sess['L']})")
        for r in rows:
            if "dip_s" not in r:
                print(f"  lap {r['lap']}: no finite front CS in 600-640 m")
                continue
            pk = ", ".join(f"{k} {v:+.0f}" for k, v in sorted(r["peaks_rel"].items(), key=lambda kv: kv[1]))
            print(f"  lap {r['lap']}: dip {r['dip']:.2f} at {r['dip_s']:.0f} m (CS_r there "
                  f"{r['cs_r_at'] if r['cs_r_at'] is None else round(r['cs_r_at'], 2)}), finite CS share "
                  f"{r['finite_share']:.2f}; speed {r['speed']:.0f} km/h, ay {r['ay']:.1f}, yaw {r['yaw']:.1f} deg/s, "
                  f"steer {r['steer']:.0f} deg, az {r['az']:.2f} g")
            print(f"    peak-rate positions rel. to dip (m): {pk}")
            print(f"    kerb: {r['kerb_in_stretch']} samples in 600-640 m, mask edges in 560-700 m at "
                  f"{[round(e) for e in r['kerb_edges_s']] or 'none'}")
            ps = r.get("psidd_term_kN_range")
            print(f"    CS window: {r['win_start_s']:.0f}-{r['dip_s']:.0f} m ({r['win_len_m']:.0f} m, {r['win_n']} samples, "
                  f"{r['win_kerb']} kerb samples); d alpha_f {r['d_alpha_deg']:+.2f} deg, d Fy_f {r['d_fy_kN']:+.2f} kN"
                  + f"; raw Iz psidd/L over window {ps[0]:+.1f}..{ps[1]:+.1f} kN")
        figure(key, sess, rows, lp, os.path.join(OUT_DIR, f"nbr_c1_dip_{key}.png"))


if __name__ == "__main__":
    main()
