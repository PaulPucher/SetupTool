# DIAGNOSTIC, read-only, [keep-reproduces]. PLAN.md EXPERIMENT PLAN (c):
# yaw-rate cross-lap pooling. Per stable corner, the analysis laps' yaw rate
# is pooled on a track-position grid; each lap's excursions beyond the
# pooled envelope are flagged as instability CANDIDATES (evidence, never a
# verdict), and each flag is checked for a steering counteraction.
# Nothing in modules/ or config/ changes. Tier B (grid resampling, zero-phase
# low-pass, robust noise floors -- standard preprocessing).
# Since 2026-09-30 the grid, envelope, margin and flags are computed by
# core/yaw_pooling.py (the app's yaw panel, "(c)-display"); this script
# passes its own pre-registered constants below and keeps the counteraction
# test. Outputs byte-identical to the pre-refactor script on all four
# cached sessions.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_yaw_pooling compute <key>
#   python -m diagnostics.inspect_yaw_pooling report
# Sessions: screening JST 29, US-v2, MLA (serial-221 fleet); Dubai only for
# the named C3-LS cross-reference. Outputs in diagnostics/plots/yaw_pooling/
# (gitignored).
#
# PRE-REGISTERED (fixed before the first run):
# - Signal: production yaw rate (deg/s) low-passed zero-phase at 4 Hz
#   (BUTTERWORTH_ORDER) -- removes vibration spikes, keeps 0.1 s-scale
#   rotation; steering wheel angle low-passed at 4 Hz, steering rate its
#   gradient (deg/s).
# - Grid: per corner traversal of an analysis lap, the canonical bracket
#   resampled on a 1 m track-position grid (gaps > 2 m left NaN, as in the
#   value lens); kerb samples excluded before resampling.
# - Pooled median and envelope p10-p90 per grid point over the analysis laps
#   (grid points with fewer than 3 laps carry no envelope).
# - Margin m = 3 x robust std (1.4826 MAD) of the filtered yaw rate's
#   deviation from its own 0.5 Hz trend on straight samples (S0 of
#   inspect_imu_offsets: racing mask outside every corner bracket) -- the
#   noise floor of the plotted signal, not of the raw channel.
# - Flag: a lap's yaw rate beyond the envelope by more than m over >= 5
#   consecutive grid points (5 m). Turn sign = sign of the pooled median at
#   the flag. EXCESS = more rotation in the turn direction than the envelope
#   (the instability-candidate side); DEFICIT = less. Counted per corner-lap.
# - Counteraction: steering-sign convention from the sign of corr(steer,
#   yaw) over racing samples; "unwinding rate" = steering rate against the
#   turn. Corroborated if the unwinding rate exceeds T within 150-300 ms
#   after the flag onset, T = 3 x robust std of steering rate on S0
#   straights. Control: the same test on every OTHER lap at the same grid
#   position (base rate of unwinding there).
# - Cross-reference: JST 29 C1 (the deepest NBR CS readings) and Dubai C3
#   (the C3-LS traction corner).

import gzip
import os
import pickle
import sys

import numpy as np

from core.yaw_pooling import lowpass, pool_yaw_rate, robust_std, session_masks
from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
from diagnostics.gated_kinematic_common import FIXED_CAP

KEYS = ["nbr_jst_solution", "pr_us_v2", "nbr_qp_mla", "dubai"]
SCREEN = ["nbr_jst_solution", "pr_us_v2", "nbr_qp_mla"]
OUT_DIR = "diagnostics/plots/yaw_pooling"
FC_HZ = 4.0
TREND_HZ = 0.5
GRID_M = 1.0
GAP_M = 2.0
MIN_LAPS = 3
MARGIN_SIGMA = 3.0
MIN_RUN_PTS = 5
LAG_WIN_S = (0.15, 0.30)
XREF = {"nbr_jst_solution": [1], "dubai": [3]}


def compute(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    data = parse_csv(SESSIONS[key])
    st = prepare_vehicle_state(data["channels"], params)
    out = {k: st[k] for k in ("time", "s_m", "v_mps", "ay_mps2", "yaw_rate_radps", "steer_sw_rad",
                              "moving_mask", "kerb_mask", "sample_rate_hz")}
    out["laps"] = [{k: l.get(k) for k in ("lap_number", "start_time", "end_time", "is_valid_for_analysis")}
                   for l in data.get("laps", [])]
    out["corners"] = [{k: c.get(k) for k in ("lap_number", "stable_corner_id", "bracket_start_m", "bracket_end_m")}
                      for c in data.get("corners", [])]
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "wb") as fh:
        pickle.dump(out, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"[{key}] cached n={st['time'].size}")


def _load(key):
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def analyse(key):
    # pooling and flags come from core.yaw_pooling (the app panel's code);
    # only the counteraction test, which belongs to the parked evidence
    # proposal, stays here. Refactor checked byte-identical on all four
    # cached sessions (thesis_notes.md "(c)-display implemented").
    d = _load(key)
    sr = d["sample_rate_hz"]
    t = d["time"]
    masks = session_masks(t, d["s_m"], d["moving_mask"], d["kerb_mask"], d["laps"], d["corners"])
    racing, s0 = masks["racing"], masks["straight"]
    pooled = pool_yaw_rate(t, d["s_m"], np.degrees(d["yaw_rate_radps"]), sr, racing, s0, d["kerb_mask"],
                           masks["traversals"], fc_hz=FC_HZ, trend_hz=TREND_HZ, grid_m=GRID_M, gap_m=GAP_M,
                           envelope_percentiles=(10, 90), min_laps=MIN_LAPS, margin_sigma=MARGIN_SIGMA,
                           min_run_pts=MIN_RUN_PTS)
    r4 = pooled["yaw_filt"]
    steer = lowpass(np.degrees(d["steer_sw_rad"]), FC_HZ, sr)
    srate = np.gradient(steer, t)
    T = MARGIN_SIGMA * robust_std(srate[s0])
    k_sign = float(np.sign(np.corrcoef(steer[racing], r4[racing])[0, 1]))
    res = {"key": key, "sigma": pooled["sigma"], "margin": pooled["margin"], "T": T, "k_sign": k_sign,
           "corners": {}}

    def corroborated(t0, turn):
        if not np.isfinite(t0):
            return None
        w = (t >= t0 + LAG_WIN_S[0]) & (t <= t0 + LAG_WIN_S[1])
        if not w.any():
            return None
        unwind = -k_sign * turn * srate[w]
        return bool(np.max(unwind) > T)

    for sid, c in pooled["corners"].items():
        TT, LAPS = c["T"], c["laps"]
        sgn = np.sign(c["med"])
        flags = []
        for f in c["flags"]:
            j, a, b = LAPS.index(f["lap"]), f["i0"], f["i1"]
            turn = float(sgn[a])
            ctrl = [corroborated(TT[jj, a], turn) for jj in range(len(LAPS)) if jj != j]
            ctrl = [x for x in ctrl if x is not None]
            flags.append({"lap": f["lap"], "kind": f["kind"], "s_on": f["s_on"], "len_m": int(b - a),
                          "peak": f["peak"], "corroborated": corroborated(TT[j, a], turn),
                          "control_share": float(np.mean(ctrl)) if ctrl else None})
        res["corners"][sid] = {"grid": c["grid"], "Y": c["Y"], "laps": LAPS, "med": c["med"], "p10": c["p_lo"],
                               "p90": c["p_hi"], "flags": flags, "n_laps": len(LAPS)}
    return res, d, r4, srate


def report():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    summary = {}
    for key in KEYS:
        if not os.path.exists(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz")):
            print(f"[{key}] no cache")
            continue
        res, d, r4, srate = analyse(key)
        role = "screening" if key in SCREEN else "cross-reference only"
        print(f"\n[{key}] ({role}) yaw noise sigma {res['sigma']:.3f} deg/s -> margin {res['margin']:.3f} deg/s; "
              f"steering-rate threshold T {res['T']:.1f} deg/s; steer/yaw sign {res['k_sign']:+.0f}")
        n_cl = sum(c["n_laps"] for c in res["corners"].values())
        exc = [(sid, f) for sid, c in res["corners"].items() for f in c["flags"] if f["kind"] == "EXCESS"]
        dfc = [(sid, f) for sid, c in res["corners"].items() for f in c["flags"] if f["kind"] == "DEFICIT"]
        exc_cl = {(sid, f["lap"]) for sid, f in exc}
        dfc_cl = {(sid, f["lap"]) for sid, f in dfc}
        cor = [f["corroborated"] for _, f in exc if f["corroborated"] is not None]
        ctrl = [f["control_share"] for _, f in exc if f["control_share"] is not None]
        print(f"  corner-laps {n_cl}; EXCESS flags {len(exc)} on {len(exc_cl)} corner-laps; DEFICIT flags {len(dfc)} "
              f"on {len(dfc_cl)} corner-laps")
        print(f"  EXCESS counteraction: corroborated {sum(cor)} / uncorroborated {len(cor) - sum(cor)} "
              f"(share {np.mean(cor) if cor else float('nan'):.2f}); control share at the same positions on other laps "
              f"{np.mean(ctrl) if ctrl else float('nan'):.2f}")
        per_corner = {}
        for sid, f in exc:
            per_corner.setdefault(sid, []).append(f)
        for sid in sorted(per_corner):
            fl = per_corner[sid]
            print(f"    C{sid}: " + "; ".join(
                f"lap {f['lap']} s {f['s_on']:.0f} m, {f['len_m']} m, +{f['peak']:.1f} deg/s, "
                f"{'corr' if f['corroborated'] else 'uncorr' if f['corroborated'] is not None else 'n/a'}" for f in fl))
        for sid in XREF.get(key, []):
            c = res["corners"].get(sid)
            if c is None:
                print(f"  cross-reference C{sid}: not pooled")
                continue
            fl = [f for f in c["flags"]]
            print(f"  cross-reference C{sid}: {c['n_laps']} laps; flags " +
                  (", ".join(f"{f['kind']} lap {f['lap']} +{f['peak']:.1f}" for f in fl) or "none"))
        summary[key] = {"sigma": res["sigma"], "margin": res["margin"], "T": res["T"], "n_corner_laps": n_cl,
                        "excess": len(exc), "excess_corner_laps": len(exc_cl), "deficit": len(dfc),
                        "corroborated": int(sum(cor)), "uncorroborated": int(len(cor) - sum(cor)),
                        "control_share": float(np.mean(ctrl)) if ctrl else None}
        # figure: full lap distance, laps thin, pooled median heavy, envelope, flags; steering-rate sub-panel
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 6.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
        ink, thin, band = "#1f2937", "#9ca3af", "#e5e7eb"
        for sid, c in res["corners"].items():
            g = c["grid"]
            for y in c["Y"]:
                ax1.plot(g, y, color=thin, linewidth=0.5)
            ax1.fill_between(g, c["p10"], c["p90"], color=band, linewidth=0)
            ax1.plot(g, c["med"], color=ink, linewidth=1.6)
            for f in c["flags"]:
                j = c["laps"].index(f["lap"])
                a = int(round((f["s_on"] - g[0]) / GRID_M))
                seg = slice(a, a + f["len_m"])
                ax1.plot(g[seg], c["Y"][j][seg], color="#b91c1c" if f["kind"] == "EXCESS" else "#2563eb", linewidth=1.8)
            ax1.text(g[len(g) // 2], np.nanmax(c["p90"]) + 3, f"C{sid}", ha="center", fontsize=7, color=ink)
        ax1.set_ylim(-80, 80)
        ax1.set_ylabel("yaw rate, 4 Hz (deg/s)", fontsize=8)
        ax1.set_title(f"{key}: yaw rate per analysis lap (thin), pooled median (heavy), p10-p90 envelope (band); "
                      f"flags beyond envelope + {res['margin']:.2f} deg/s: excess red, deficit blue", fontsize=9)
        lap_valid = [l for l in d["laps"] if l["is_valid_for_analysis"]]
        for l in lap_valid:
            m_ = (d["time"] >= l["start_time"]) & (d["time"] < l["end_time"])
            ax2.plot(d["s_m"][m_], srate[m_], color=thin, linewidth=0.4)
        ax2.axhline(res["T"], color=ink, linewidth=0.6, linestyle="--")
        ax2.axhline(-res["T"], color=ink, linewidth=0.6, linestyle="--")
        ax2.set_ylim(-600, 600)
        ax2.set_ylabel("steering rate (deg/s)", fontsize=8)
        ax2.set_xlabel("lap distance (m)", fontsize=8)
        for ax in (ax1, ax2):
            ax.tick_params(labelsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"yaw_pooling_{key}.png"), dpi=110)
        plt.close(fig)
    with open(os.path.join(OUT_DIR, "summary.pkl"), "wb") as fh:
        pickle.dump(summary, fh)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "compute":
        compute(sys.argv[2])
    else:
        {"report": report}[mode]()
