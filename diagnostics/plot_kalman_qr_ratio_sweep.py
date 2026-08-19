# WP-S5b plots (Open Board item B, sideslip methods comparison):
# ratio-sweep visualization, companion to diagnostics/inspect_kalman_qr_
# ratio_sweep.py. Diagnostics-only, no UI, no production/config change.
#
# Sibling script rather than a plot_sideslip_comparison.py extension:
# that script's structure (one fixed A/C candidate pair, generated
# once) doesn't compose with re-running the observer 7x under
# different Q/R; forcing it in risks destabilizing "keep the existing
# comparison plots working unchanged" (explicit instruction). Reuses
# the same labelled-folder + run_info.txt scheme as plot_sideslip_
# comparison.py, duplicated inline rather than factored into a shared
# helper -- a second consumer alone doesn't justify extraction (same
# reasoning as WP-S3's Metric 5 factoring decision).
#
# Figure A (ratio_sweep_measures.png): seven measures vs Q/R ratio,
# log x-axis, all 7 tested ratios marked as points, chosen ratio
# (0.3162, WP-S5b recommendation) marked with a vertical line in every
# panel.
# Figure B (ratio_sweep_lap_traces.png): the observer's sideslip over
# one representative lap (the first valid lap) at each of the 7 tested
# ratios, shared axes, kinematic estimate as a reference line, corner
# brackets shaded -- so heavy-vs-light smoothing is directly visible.

import datetime
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_sideslip
import diagnostics.sideslip_kalman_observer as sko
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LARGE_EXCURSION_DEG = 10.0
CHOSEN_RATIO = 0.3162
RATIOS = np.logspace(-3, 2, 7)

if len(sys.argv) > 1:
    RUN_LABEL = sys.argv[1]
else:
    RUN_LABEL = datetime.date.today().isoformat()
    print(f"No run label given on the command line -- using today's date as the "
          f"folder name: {RUN_LABEL}")

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots", RUN_LABEL)
os.makedirs(OUTPUT_DIR, exist_ok=True)
written = []


def _git_commit_info():
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True).strip())
        return f"{commit} ({'dirty -- uncommitted changes present' if dirty else 'clean'})"
    except Exception as exc:
        return f"unavailable ({exc})"


BASE_Q_BETA_VAR = sko.Q_BETA_VAR
BASE_Q_YAW_RATE_VAR = sko.Q_YAW_RATE_VAR
BASE_R_YAW_RATE_VAR = sko.R_YAW_RATE_VAR
BASE_R_AY_VAR = sko.R_AY_VAR

run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write(f"run label: {RUN_LABEL}\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {_git_commit_info()}\n")
    f.write("script: diagnostics/plot_kalman_qr_ratio_sweep.py\n")
    f.write(f"ratios swept (Q_scale/R_scale, R held at this run's baseline): "
            f"{[round(float(r), 6) for r in RATIOS]}\n")
    f.write(f"chosen ratio marked in figures: {CHOSEN_RATIO}\n")
    f.write("observer baseline Q/R settings (diagnostics/sideslip_kalman_observer.py) at time of this run:\n")
    f.write(f"  Q_BETA_VAR = {float(BASE_Q_BETA_VAR):.6e} rad^2\n")
    f.write(f"  Q_YAW_RATE_VAR = {float(BASE_Q_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_YAW_RATE_VAR = {float(BASE_R_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_AY_VAR = {float(BASE_R_AY_VAR):.6e} (m/s^2)^2\n")
written.append(run_info_path)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t_ref = state["time"]
s_m = state.get("s_m")
ay = state["ay_mps2"]
ay_g = ay / 9.81
yaw_rate_degps = np.degrees(state["yaw_rate_radps"])
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_laps = [l for l in laps if l.get("is_valid_for_analysis")]
laps_by_number = {l["lap_number"]: l for l in laps}
valid_windows = [(l["start_time"], l["end_time"]) for l in valid_laps]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)

corners = data.get("corners", [])
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)

straight_mask = moving & racing_mask & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)

TRANSIENT_PHASES = ["entry_1_brake", "entry_2_turnin", "exit_4", "exit_5"]


def _phase_slice(start_t, end_t):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t_ref, start_t, side="left"))
    hi = int(np.searchsorted(t_ref, end_t, side="right"))
    return slice(lo, hi)


transient_mask = np.zeros_like(t_ref, dtype=bool)
for c in corners:
    lap = laps_by_number.get(c["lap_number"])
    if lap is None or not lap.get("is_valid_for_analysis"):
        continue
    for phase in TRANSIENT_PHASES:
        s_t, e_t = c["segments"][phase]
        sl = _phase_slice(s_t, e_t)
        if sl.stop > sl.start:
            transient_mask[sl] = True
transient_mask &= moving


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(np.max(lap_s[finite]))
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


def _corner_stats(beta_deg):
    stats = {}
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        lap_medians, pooled_overall, pooled_ay = [], [], []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = moving[sl]
            if not m.any():
                continue
            vals = beta_deg[sl][m]
            lap_medians.append(float(np.median(vals)))
            pooled_overall.append(vals)
            pooled_ay.append(ay[sl][m])
        if not pooled_overall:
            continue
        stats[cid] = {
            "lap_medians": lap_medians,
            "overall_median": float(np.median(np.concatenate(pooled_overall))),
            "median_ay": float(np.median(np.concatenate(pooled_ay))),
            "speed_class": instances[0].get("speed_class"),
        }
    return stats


def _sweep_metrics(beta_deg):
    valid_mask = moving & racing_mask & np.isfinite(beta_deg)
    sl_vals = np.abs(beta_deg[straight_mask & np.isfinite(beta_deg)])
    straight_median = float(np.median(sl_vals))
    straight_p90 = float(np.percentile(sl_vals, 90))

    all_vals = beta_deg[valid_mask]
    max_abs = float(np.max(np.abs(all_vals)))
    n_excursion = int(np.sum(np.abs(all_vals) > LARGE_EXCURSION_DEG))

    stats = _corner_stats(beta_deg)
    cross_lap_stds = [float(np.std(s["lap_medians"])) for s in stats.values() if len(s["lap_medians"]) >= 2]
    overall_medians = [s["overall_median"] for s in stats.values()]
    between_corner_std = float(np.std(overall_medians))

    n_match = n_total = 0
    for cid, s in stats.items():
        if s.get("speed_class") == "low":
            continue
        dir_sign = np.sign(s["median_ay"])
        if dir_sign == 0:
            continue
        n_total += 1
        if np.sign(s["overall_median"]) == -dir_sign:
            n_match += 1

    dbeta_dt = np.gradient(beta_deg, t_ref)
    day_g_dt = np.gradient(ay_g, t_ref)
    tm = transient_mask & np.isfinite(dbeta_dt) & np.isfinite(day_g_dt)
    transient_corr = float(np.corrcoef(dbeta_dt[tm], day_g_dt[tm])[0, 1]) if tm.sum() > 2 else float("nan")

    return {
        "straight_median": straight_median, "straight_p90": straight_p90,
        "cross_lap_std": float(np.median(cross_lap_stds)) if cross_lap_stds else float("nan"),
        "between_corner_std": between_corner_std,
        "max_abs": max_abs, "n_excursion": n_excursion,
        "sign_match": n_match, "sign_total": n_total,
        "transient_corr": transient_corr,
    }


beta_a_deg = np.degrees(estimate_sideslip(state, params))

sweep_beta = {}
sweep_metrics = {}
for ratio in RATIOS:
    sko.Q_BETA_VAR = BASE_Q_BETA_VAR * ratio
    sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR * ratio
    sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR
    sko.R_AY_VAR = BASE_R_AY_VAR
    beta_deg = np.degrees(sko.estimate_sideslip_kalman(state, params))
    sweep_beta[ratio] = beta_deg
    sweep_metrics[ratio] = _sweep_metrics(beta_deg)

sko.Q_BETA_VAR = BASE_Q_BETA_VAR
sko.Q_YAW_RATE_VAR = BASE_Q_YAW_RATE_VAR
sko.R_YAW_RATE_VAR = BASE_R_YAW_RATE_VAR
sko.R_AY_VAR = BASE_R_AY_VAR

# --- Figure A: measures vs ratio --------------------------------------

fig, axes = plt.subplots(4, 2, figsize=(13, 16))
axes = axes.flatten()


def _mark_chosen(ax):
    ax.axvline(CHOSEN_RATIO, color="black", linestyle="--", linewidth=1.0, label=f"chosen ratio={CHOSEN_RATIO}")


ax = axes[0]
ax.plot(RATIOS, [sweep_metrics[r]["straight_median"] for r in RATIOS], "o-", label="median")
ax.plot(RATIOS, [sweep_metrics[r]["straight_p90"] for r in RATIOS], "s-", label="p90")
ax.set_xscale("log")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("straight-line |beta| (deg)")
ax.set_title("Straight-line sideslip vs ratio (target 1)")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[1]
ax.plot(RATIOS, [sweep_metrics[r]["cross_lap_std"] for r in RATIOS], "o-", color="tab:green")
ax.set_xscale("log")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("cross-lap std of median beta (deg)")
ax.set_title("Consistency vs ratio (target 2a)")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[2]
ax.plot(RATIOS, [sweep_metrics[r]["between_corner_std"] for r in RATIOS], "o-", color="tab:orange")
ax.set_xscale("log")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("between-corner std of median beta (deg)")
ax.set_title("Discrimination vs ratio (target 2b)")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[3]
ax.plot(RATIOS, [sweep_metrics[r]["max_abs"] for r in RATIOS], "o-", color="tab:red")
ax.set_xscale("log")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("max |beta| (deg)")
ax.set_title("Max sideslip vs ratio (target 3)")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[4]
ax.plot(RATIOS, [sweep_metrics[r]["n_excursion"] for r in RATIOS], "o-", color="tab:purple")
ax.set_xscale("log")
ax.set_yscale("symlog")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel(f"count |beta| > {LARGE_EXCURSION_DEG} deg")
ax.set_title("Large-excursion count vs ratio (target 3)")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[5]
ax.plot(RATIOS, [sweep_metrics[r]["transient_corr"] for r in RATIOS], "o-", color="tab:brown")
ax.set_xscale("log")
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("corr(d(beta)/dt, d(ay)/dt), entry/exit phases")
ax.set_title("Transient tracking vs ratio")
_mark_chosen(ax)
ax.legend(fontsize=8)

ax = axes[6]
ax.plot(RATIOS, [sweep_metrics[r]["sign_match"] for r in RATIOS], "o-", color="tab:cyan")
ax.set_xscale("log")
ax.set_ylim(0, 12)
ax.set_xlabel("Q/R ratio")
ax.set_ylabel("sign match count (of 11 racing-speed corners)")
ax.set_title("Physical sign match vs ratio (target 4)")
_mark_chosen(ax)
ax.legend(fontsize=8)

axes[7].axis("off")

fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "ratio_sweep_measures.png")
fig.savefig(fname, dpi=120)
plt.close(fig)
written.append(fname)

# --- Figure B: sideslip traces at each ratio, one representative lap ---

rep_lap = valid_laps[0]
lap_no = rep_lap["lap_number"]
lo = int(np.searchsorted(t_ref, rep_lap["start_time"], side="left"))
hi = int(np.searchsorted(t_ref, rep_lap["end_time"], side="right"))
t_rel = t_ref[lo:hi] - rep_lap["start_time"]

fig, ax = plt.subplots(figsize=(13, 6))
# Qualitative, high-contrast palette rather than a continuous colormap:
# most of the swept ratios plateau close together numerically (WP-S5b
# finding), so a sequential gradient (e.g. viridis) bands them into a
# visually indistinguishable cluster in its middle range. Distinct hues
# keep every one of the 7 traces separable even where the underlying
# curves nearly overlap -- itself an accurate depiction of the plateau.
RATIO_COLORS = ["tab:purple", "tab:blue", "tab:cyan", "tab:green", "tab:orange", "tab:red", "tab:brown"]
for i, ratio in enumerate(RATIOS):
    is_chosen = np.isclose(ratio, CHOSEN_RATIO, rtol=1e-3)
    lw = 1.6 if is_chosen else 1.0
    ax.plot(t_rel, sweep_beta[ratio][lo:hi], color=RATIO_COLORS[i], linewidth=lw, alpha=0.85,
            label=f"ratio={ratio:.4g}" + (" (chosen)" if is_chosen else ""))
ax.plot(t_rel, beta_a_deg[lo:hi], color="black", linestyle="--", linewidth=1.4, label="A_kinematic (reference)")

for c in corners:
    if c["lap_number"] != lap_no:
        continue
    seg = c["segments"]
    s_t = seg["entry_1_brake"][0]
    e_t = seg["exit_5"][1]
    if e_t <= s_t:
        continue
    ax.axvspan(s_t - rep_lap["start_time"], e_t - rep_lap["start_time"], color="gray", alpha=0.12, lw=0)

ax.set_xlabel("time since lap start (s)")
ax.set_ylabel("sideslip beta (deg)")
ax.set_title(f"Lap {lap_no}: observer sideslip at each tested ratio (shaded = corner brackets)")
ax.legend(fontsize=7, ncol=2, loc="upper right")
fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "ratio_sweep_lap_traces.png")
fig.savefig(fname, dpi=120)
plt.close(fig)
written.append(fname)

print(f"Output folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")
