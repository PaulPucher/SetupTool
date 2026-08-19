# WP-S6 follow-up (Open Board item B, sideslip methods comparison):
# slip-angle plot script, research use only. Diagnostics-only, no UI,
# no production/config change. Companion to diagnostics/plot_sideslip_
# comparison.py, same labelled-folder + run_info.txt scheme (duplicated
# inline, not factored into a shared helper -- a third consumer of this
# small pattern still doesn't justify extraction on its own, same
# reasoning as WP-S3/S5b).
#
# Purpose: sideslip (beta) plots show the estimator disagreement in the
# state the two methods construct; this script looks one level closer
# to the tyre -- alpha_f/alpha_r, the quantity that actually drives
# Fy in the production Module 4a/4b chain -- and against a known
# approximate tyre-peak slip angle (~8 deg, order-of-magnitude GT3
# slick reference, not sourced to a specific tyre) to see how far each
# method's slip angles reach and whether either method's force-vs-slip
# cloud shows the expected bend near the peak.

import datetime
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
)
from diagnostics.sideslip_kalman_observer import (
    estimate_sideslip_kalman,
    Q_BETA_VAR, Q_YAW_RATE_VAR, R_YAW_RATE_VAR, R_AY_VAR,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TYRE_PEAK_DEG = 8.0

# The observer's tyre model as of this WP (linear, fixed Caf/Car prior --
# see thesis_notes.md "Linear observer saturation-detection failure").
# Every plot this script produces used this model; the run label and the
# manifest both record it explicitly so a future nonlinear-tyre observer's
# plots are never mistaken for these, without needing tyre-model text
# burned into the figures themselves.
TYRE_MODEL_TAG = "linear_tyre"
TYRE_MODEL_DESC = ("linear (fixed stiffness prior, Caf/Car from config/parameters.json "
                    "cs_front/rear_fallback_reference_n_per_rad)")

if len(sys.argv) > 1:
    RUN_LABEL = sys.argv[1]
else:
    RUN_LABEL = datetime.date.today().isoformat()
    print(f"No run label given on the command line -- using today's date as the "
          f"folder name: {RUN_LABEL}")
if not RUN_LABEL.endswith(f"_{TYRE_MODEL_TAG}"):
    RUN_LABEL = f"{RUN_LABEL}_{TYRE_MODEL_TAG}"

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


run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write(f"run label: {RUN_LABEL}\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {_git_commit_info()}\n")
    f.write("script: diagnostics/plot_slip_angle_comparison.py\n")
    f.write(f"tyre model: {TYRE_MODEL_DESC}\n")
    f.write(f"tyre-peak reference line: {TYRE_PEAK_DEG} deg (approximate, not sourced to a specific tyre)\n")
    f.write("observer Q/R settings (diagnostics/sideslip_kalman_observer.py) at time of this run:\n")
    f.write(f"  Q_BETA_VAR = {float(Q_BETA_VAR):.6e} rad^2\n")
    f.write(f"  Q_YAW_RATE_VAR = {float(Q_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_YAW_RATE_VAR = {float(R_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_AY_VAR = {float(R_AY_VAR):.6e} (m/s^2)^2\n")
written.append(run_info_path)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t_ref = state["time"]
s_m = state.get("s_m")
ay = state["ay_mps2"]
ay_g = ay / 9.81
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

beta_a = estimate_sideslip(state, params)
beta_c = estimate_sideslip_kalman(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)

alpha_f_a_deg = np.degrees(slip_a["alpha_f_filt"])
alpha_r_a_deg = np.degrees(slip_a["alpha_r_filt"])
alpha_f_c_deg = np.degrees(slip_c["alpha_f_filt"])
alpha_r_c_deg = np.degrees(slip_c["alpha_r_filt"])
Fy_f = forces["Fy_f_filt"]
Fy_r = forces["Fy_r_filt"]

AXLES = {
    "front": {"a": alpha_f_a_deg, "c": alpha_f_c_deg, "Fy": Fy_f},
    "rear": {"a": alpha_r_a_deg, "c": alpha_r_c_deg, "Fy": Fy_r},
}


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


in_corner_mask = np.zeros_like(t_ref, dtype=bool)
corner_slices_by_id = {}  # cid -> list of (lap_number, slice)
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    slices = []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop <= sl.start:
            continue
        in_corner_mask[sl] = True
        slices.append(sl)
    if slices:
        corner_slices_by_id[cid] = slices

corner_valid_mask = moving & racing_mask & in_corner_mask

# --- Plot 1: per-lap time series, front and rear separately ----------------


def _shade_corners(ax, lap_number, lap_start_t):
    for c in corners:
        if c["lap_number"] != lap_number:
            continue
        seg = c["segments"]
        s_t = seg["entry_1_brake"][0]
        e_t = seg["exit_5"][1]
        if e_t <= s_t:
            continue
        ax.axvspan(s_t - lap_start_t, e_t - lap_start_t, color="gray", alpha=0.15, lw=0)
        sid = c.get("stable_corner_id")
        if sid is not None:
            ymax = ax.get_ylim()[1]
            ax.text((s_t + e_t) / 2 - lap_start_t, ymax * 0.90, f"C{sid}",
                    fontsize=6, ha="center", color="dimgray")


for lap in valid_laps:
    lap_no = lap["lap_number"]
    lo = int(np.searchsorted(t_ref, lap["start_time"], side="left"))
    hi = int(np.searchsorted(t_ref, lap["end_time"], side="right"))
    if hi <= lo:
        continue
    t_rel = t_ref[lo:hi] - lap["start_time"]
    ayg = ay_g[lo:hi]

    for axle_name, axle in AXLES.items():
        fig, ax1 = plt.subplots(figsize=(12, 5))
        ax1.plot(t_rel, axle["a"][lo:hi], label="A_kinematic alpha", color="tab:blue", linewidth=1.0)
        ax1.plot(t_rel, axle["c"][lo:hi], label="C_kalman_observer alpha", color="tab:orange", linewidth=1.0)
        ax1.set_xlabel("time since lap start (s)")
        ax1.set_ylabel(f"{axle_name} slip angle alpha (deg)")
        ax2 = ax1.twinx()
        ax2.plot(t_rel, ayg, color="tab:gray", alpha=0.4, linewidth=0.8, label="ay (g)")
        ax2.set_ylabel("lateral acceleration ay (g)")
        _shade_corners(ax1, lap_no, lap["start_time"])
        l1, la1 = ax1.get_legend_handles_labels()
        l2, la2 = ax2.get_legend_handles_labels()
        ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
        ax1.set_title(f"Lap {lap_no}: {axle_name} slip angle, kinematic vs observer (shaded = corner brackets)")
        fig.tight_layout()
        fname = os.path.join(OUTPUT_DIR, f"lap{lap_no}_slip_{axle_name}.png")
        fig.savefig(fname, dpi=120)
        plt.close(fig)
        written.append(fname)

# --- Plot 2: distribution of |alpha| in corners only, front/rear separate --

corner_stats = {}  # axle -> method -> {"median":..., "p95":..., "max":...}
for axle_name, axle in AXLES.items():
    corner_stats[axle_name] = {}
    fig, ax = plt.subplots(figsize=(9, 5))
    bins = np.linspace(0, 14, 57)
    for method, key, color in (("A_kinematic", "a", "tab:blue"), ("C_kalman_observer", "c", "tab:orange")):
        vals = np.abs(axle[key][corner_valid_mask])
        vals = vals[np.isfinite(vals)]
        ax.hist(vals, bins=bins, alpha=0.55, color=color, label=method, density=True)
        corner_stats[axle_name][method] = {
            "median": float(np.median(vals)), "p95": float(np.percentile(vals, 95)), "max": float(np.max(vals)),
            "n": int(len(vals)),
        }
    ax.axvline(TYRE_PEAK_DEG, color="black", linestyle="--", linewidth=1.0,
               label=f"~{TYRE_PEAK_DEG:.0f} deg (known approximate tyre peak)")
    ax.set_xlabel(f"{axle_name} |slip angle| (deg)")
    ax.set_ylabel("density")
    ax.set_title(f"{axle_name.capitalize()} slip-angle magnitude distribution, corner samples only")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"slip_distribution_{axle_name}.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

# --- Plot 3: per-corner summary, median/p95 |alpha|, front/rear separate ---

for axle_name, axle in AXLES.items():
    cids = sorted(corner_slices_by_id)
    med_a, med_c, p95_a, p95_c = [], [], [], []
    for cid in cids:
        pooled_a, pooled_c = [], []
        for sl in corner_slices_by_id[cid]:
            m = moving[sl]
            if not m.any():
                continue
            pooled_a.append(np.abs(axle["a"][sl][m]))
            pooled_c.append(np.abs(axle["c"][sl][m]))
        if not pooled_a:
            med_a.append(np.nan); med_c.append(np.nan); p95_a.append(np.nan); p95_c.append(np.nan)
            continue
        va, vc = np.concatenate(pooled_a), np.concatenate(pooled_c)
        med_a.append(float(np.median(va))); med_c.append(float(np.median(vc)))
        p95_a.append(float(np.percentile(va, 95))); p95_c.append(float(np.percentile(vc, 95)))

    x = np.arange(len(cids))
    width = 0.35
    fig, (ax_med, ax_p95) = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    ax_med.bar(x - width / 2, med_a, width, label="A_kinematic", color="tab:blue")
    ax_med.bar(x + width / 2, med_c, width, label="C_kalman_observer", color="tab:orange")
    ax_med.axhline(TYRE_PEAK_DEG, color="black", linestyle="--", linewidth=0.8,
                   label=f"~{TYRE_PEAK_DEG:.0f} deg (known approximate tyre peak)")
    ax_med.set_ylabel("median |alpha| (deg)")
    ax_med.set_title(f"{axle_name.capitalize()} per-corner slip-angle magnitude: median (top), p95 (bottom)")
    ax_med.legend(fontsize=8)

    ax_p95.bar(x - width / 2, p95_a, width, color="tab:blue")
    ax_p95.bar(x + width / 2, p95_c, width, color="tab:orange")
    ax_p95.axhline(TYRE_PEAK_DEG, color="black", linestyle="--", linewidth=0.8)
    ax_p95.set_xticks(x)
    ax_p95.set_xticklabels([f"C{c}" for c in cids])
    ax_p95.set_xlabel("stable corner id")
    ax_p95.set_ylabel("p95 |alpha| (deg)")

    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"per_corner_slip_{axle_name}.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

# --- Plot 4: force vs slip angle, one panel per method per axle -----------

for axle_name, axle in AXLES.items():
    fig, (ax_a, ax_c) = plt.subplots(1, 2, figsize=(13, 6), sharey=True)
    for ax, key, method in ((ax_a, "a", "A_kinematic"), (ax_c, "c", "C_kalman_observer")):
        x_vals = axle[key][corner_valid_mask]
        y_vals = axle["Fy"][corner_valid_mask]
        finite = np.isfinite(x_vals) & np.isfinite(y_vals)
        ax.scatter(x_vals[finite], y_vals[finite], s=3, alpha=0.15, color="tab:blue" if method == "A_kinematic" else "tab:orange")
        ax.axvline(TYRE_PEAK_DEG, color="black", linestyle="--", linewidth=0.8)
        ax.axvline(-TYRE_PEAK_DEG, color="black", linestyle="--", linewidth=0.8,
                   label=f"+/-{TYRE_PEAK_DEG:.0f} deg (known approximate tyre peak)")
        ax.set_xlabel(f"{axle_name} slip angle alpha (deg)")
        ax.set_title(method)
        ax.legend(fontsize=7)
    ax_a.set_ylabel(f"{axle_name} lateral force Fy (N)")
    fig.suptitle(f"{axle_name.capitalize()} force vs slip angle, corner samples only")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"force_vs_slip_{axle_name}.png")
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    written.append(fname)

# --- Printed summary ---------------------------------------------------

print(f"Output folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")

print()
print("Slip-angle magnitude over corner samples, per axle per method (deg):")
for axle_name in AXLES:
    for method in ("A_kinematic", "C_kalman_observer"):
        s = corner_stats[axle_name][method]
        print(f"  {axle_name:6s} {method:18s} n={s['n']:6d}  median={s['median']:.3f}  "
              f"p95={s['p95']:.3f}  max={s['max']:.3f}")
