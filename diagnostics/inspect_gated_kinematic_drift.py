# Gated-kinematic diagnostic, step 3 of 4: drift as a function of time.
# Metric 3 of the work order. Two measurements, both CAUSAL:
#  (a) |beta| vs time past each real corner exit (canonical bracket end),
#      on straight-like samples (|ay| below the Otsu gate threshold) up to
#      the next corner entry -- the drift-vs-time criterion that replaced
#      the single-checkpoint check (record: causal 0.02 Hz washout drifted
#      ~0.25 -> ~1.6 deg within 4 s). True straight-line sideslip is small,
#      so |beta| there is read as drift. The gated source is 0 there by
#      construction; its drift lives INSIDE the gate and is measured by (b).
#  (b) gated integrator value at the last gated sample of each run, just
#      before the reset, vs run duration: accumulated in-corner drift plus
#      the true beta at the moment |ay| falls through the threshold.
# Production 0.05 Hz filtfilt is zero-phase/acausal: its (a) numbers are
# printed for reference, marked ACAUSAL, and never plotted as drift.
#
# Run from the repo root:  python -m diagnostics.inspect_gated_kinematic_drift

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from diagnostics.gated_kinematic_common import (
    SESSIONS, OUT_DIR, GATE_FACTORS, load_cache, valid_corner_instances,
)
from diagnostics.gated_kinematic_style import SERIES, TEXT, style_axes

TAU_MAX_S = 4.0
TAU_STEP_S = 0.05
MIN_RUN_S = 0.5
CAUSAL = ["causal_washout_0.05hz", "causal_washout_0.02hz", "ekf_auto_pacejka"] + [f"gated_x{f:g}" for f in GATE_FACTORS]
ACAUSAL = ["kinematic_prod"]


def drift_curves(cache, sources):
    st = cache["state"]
    t, ay, sr = st["time"], st["ay_mps2"], st["sample_rate_hz"]
    racing = cache["racing_mask"]
    thr = cache["thr_otsu"]
    inst = valid_corner_instances({"laps": cache["laps"], "corners": cache["corners"]}, t, st["s_m"])
    starts = np.array([sl.start for _, _, sl in inst])
    taus = np.arange(0.0, TAU_MAX_S + 1e-9, TAU_STEP_S)
    steps = np.round(taus * sr).astype(int)
    out = {}
    for src in sources:
        beta_deg = np.degrees(cache["betas"][src])
        per_tau = [[] for _ in taus]
        for _, _, sl in inst:
            exit_i = sl.stop
            later = starts[starts > exit_i]
            limit = int(later[0]) if later.size else len(t)
            for k, d in enumerate(steps):
                i = exit_i + d
                if i >= limit:
                    break
                if racing[i] and abs(ay[i]) < thr:
                    per_tau[k].append(abs(beta_deg[i]))
        out[src] = {
            "tau_s": taus.tolist(),
            "median_abs_deg": [float(np.median(v)) if v else np.nan for v in per_tau],
            "p90_abs_deg": [float(np.percentile(v, 90)) if v else np.nan for v in per_tau],
            "n": [len(v) for v in per_tau],
        }
    return out, len(inst)


def gate_run_residuals(cache, key, inst):
    st = cache["state"]
    sr = st["sample_rate_hz"]
    racing = cache["racing_mask"]
    g = cache["gates"][key]
    rid = g["run_id"]
    beta = cache["betas"][key]
    yaw = st["yaw_rate_radps"]
    rows = []
    n_runs = int(rid.max()) + 1 if rid.max() >= 0 else 0
    # run boundaries in one pass
    idx = np.where(rid >= 0)[0]
    bounds = np.split(idx, np.where(np.diff(rid[idx]) != 0)[0] + 1) if idx.size else []
    for run in bounds:
        if run.size / sr < MIN_RUN_S or not racing[run].mean() > 0.5:
            continue
        sid, lap, best = None, None, 0
        for s_id, lp, sl in inst:
            ov = min(sl.stop, run[-1] + 1) - max(sl.start, run[0])
            if ov > best:
                sid, lap, best = s_id, lp, ov
        rows.append({
            "stable_id": sid, "lap": lap,
            "duration_s": run.size / sr,
            "residual_deg": float(np.degrees(beta[run[-1]])),
            "heading_change_deg": float(np.degrees(np.sum(yaw[run]) / sr)),
        })
    return rows, n_runs


def plot_drift(name, curves, thr):
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for src in CAUSAL:
        c = curves[src]
        st = SERIES[src]
        ax.plot(c["tau_s"], c["median_abs_deg"], color=st["color"], linestyle=st["ls"],
                linewidth=2, label=st["label"])
    ax.set_xlabel("time past corner exit (s)", fontsize=9)
    ax.set_ylabel("median |beta| on straight-like samples (deg)", fontsize=9)
    ax.set_title(f"{name}: causal drift vs time past corner exit (|ay| < {thr:.2f} m/s^2)\n"
                 "gated sources are 0 here by construction (reset outside the gate)", fontsize=10, color=TEXT)
    style_axes(ax)
    ax.legend(fontsize=7, frameon=False)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, f"drift_vs_time_{name}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_residuals(name, runs_by_key):
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for key, rows in runs_by_key.items():
        st = SERIES[key]
        ax.plot([r["duration_s"] for r in rows], [r["residual_deg"] for r in rows], linestyle="none",
                marker=st["marker"], markersize=5, color=st["color"], alpha=0.8, label=st["label"])
    ax.axhline(0.0, color=TEXT, linewidth=0.8)
    ax.set_xlabel("gate run duration (s)", fontsize=9)
    ax.set_ylabel("gated beta at gate exit, before reset (deg)", fontsize=9)
    ax.set_title(f"{name}: gated integrator value at gate exit vs run length", fontsize=10, color=TEXT)
    style_axes(ax)
    ax.legend(fontsize=7, frameon=False)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, f"gate_exit_residual_{name}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main():
    results = {}
    for name in SESSIONS:
        try:
            cache = load_cache(name)
        except FileNotFoundError:
            print(f"[{name}] no cache")
            continue
        curves, n_exits = drift_curves(cache, CAUSAL + ACAUSAL)
        print(f"\n===== {name}: {n_exits} corner exits (valid laps), Otsu threshold {cache['thr_otsu']:.3f} m/s^2")
        for src in CAUSAL + ACAUSAL:
            c = curves[src]
            tag = "ACAUSAL (reference only)" if src in ACAUSAL else "causal"
            pick = [0, 20, 40, 60, 80]  # tau = 0, 1, 2, 3, 4 s
            vals = ", ".join(f"{c['tau_s'][k]:.0f}s {c['median_abs_deg'][k]:.2f}/{c['p90_abs_deg'][k]:.2f} (n={c['n'][k]})"
                             for k in pick)
            print(f"  {src:22s} [{tag}] median/p90 |beta| deg: {vals}")
        st = cache["state"]
        inst = valid_corner_instances({"laps": cache["laps"], "corners": cache["corners"]}, st["time"], st["s_m"])
        runs = {}
        for f in GATE_FACTORS:
            key = f"gated_x{f:g}"
            rows, n_runs = gate_run_residuals(cache, key, inst)
            runs[key] = rows
            res = np.array([r["residual_deg"] for r in rows])
            dur = np.array([r["duration_s"] for r in rows])
            slope = np.polyfit(dur, np.abs(res), 1)[0] if len(rows) > 2 else np.nan
            print(f"  {key}: {n_runs} gate runs total, {len(rows)} >= {MIN_RUN_S}s on valid laps; "
                  f"|residual| median {np.median(np.abs(res)):.2f} deg, p90 {np.percentile(np.abs(res), 90):.2f} deg, "
                  f"max {np.max(np.abs(res)):.2f} deg; |residual| vs duration slope {slope:.3f} deg/s")
        print(f"  figures: {plot_drift(name, curves, cache['thr_otsu'])}, {plot_residuals(name, runs)}")
        results[name] = {"curves": curves, "gate_runs": runs}
    with open(os.path.join(OUT_DIR, "drift_results.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=float)


if __name__ == "__main__":
    main()
