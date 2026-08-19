# WP-S5 (Open Board item B, sideslip methods comparison): research-only
# plot script. Diagnostics-only, no UI, no production/config change.
# Companion to diagnostics/inspect_sideslip_methods_comparison.py's
# numeric Metrics 1-5 -- shows WHERE the kinematic (A) and Kalman-
# observer (C) sideslip estimates differ, not just by how much.
# Writes PNGs to diagnostics/plots/<run label>/ (diagnostics/plots/ is
# gitignored recursively -- reproducible output, not source, never
# enters the repo, subfolders included).
#
# Run label: optional first command-line argument (e.g. `python -m
# diagnostics.plot_sideslip_comparison untuned`), so repeated rounds
# (e.g. before/after Q/R tuning) can be told apart later. No label text
# is ever drawn into the figures themselves -- the folder name is the
# only marker. Defaults to today's date if no label is given.

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
from diagnostics.sideslip_kalman_observer import (
    estimate_sideslip_kalman,
    Q_BETA_VAR, Q_YAW_RATE_VAR, R_YAW_RATE_VAR, R_AY_VAR, P0_BETA_VAR, P0_YAW_RATE_VAR,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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


def _git_commit_info():
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
        ).strip()
        dirty = bool(subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True
        ).strip())
        return f"{commit} ({'dirty -- uncommitted changes present' if dirty else 'clean'})"
    except Exception as exc:
        return f"unavailable ({exc})"

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

beta_a_deg = np.degrees(estimate_sideslip(state, params))
beta_c_deg = np.degrees(estimate_sideslip_kalman(state, params))

laps = data.get("laps", [])
valid_laps = [l for l in laps if l.get("is_valid_for_analysis")]
corners = data.get("corners", [])

os.makedirs(OUTPUT_DIR, exist_ok=True)
written = []

run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write(f"run label: {RUN_LABEL}\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {_git_commit_info()}\n")
    f.write(f"tyre model: {TYRE_MODEL_DESC}\n")
    f.write("observer Q/R settings (diagnostics/sideslip_kalman_observer.py):\n")
    f.write(f"  Q_BETA_VAR = {float(Q_BETA_VAR):.6e} rad^2\n")
    f.write(f"  Q_YAW_RATE_VAR = {float(Q_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_YAW_RATE_VAR = {float(R_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    f.write(f"  R_AY_VAR = {float(R_AY_VAR):.6e} (m/s^2)^2\n")
    f.write(f"  P0_BETA_VAR = {float(P0_BETA_VAR):.6e} rad^2\n")
    f.write(f"  P0_YAW_RATE_VAR = {float(P0_YAW_RATE_VAR):.6e} (rad/s)^2\n")
written.append(run_info_path)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to the helper of the same name used throughout this WP
    # (inspect_c9_negative_cs.py / Metric 5 / WP-S3b/c/S4b).
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


# --- Plots 1 & 2: per-lap time-series overlay + difference -----------------

for lap in valid_laps:
    lap_no = lap["lap_number"]
    lo = int(np.searchsorted(t_ref, lap["start_time"], side="left"))
    hi = int(np.searchsorted(t_ref, lap["end_time"], side="right"))
    if hi <= lo:
        continue
    t_rel = t_ref[lo:hi] - lap["start_time"]
    ba = beta_a_deg[lo:hi]
    bc = beta_c_deg[lo:hi]
    ayg = ay_g[lo:hi]

    fig, ax1 = plt.subplots(figsize=(12, 5))
    ax1.plot(t_rel, ba, label="A_kinematic beta", color="tab:blue", linewidth=1.0)
    ax1.plot(t_rel, bc, label="C_kalman_observer beta", color="tab:orange", linewidth=1.0)
    ax1.set_xlabel("time since lap start (s)")
    ax1.set_ylabel("sideslip beta (deg)")
    ax2 = ax1.twinx()
    ax2.plot(t_rel, ayg, color="tab:gray", alpha=0.4, linewidth=0.8, label="ay (g)")
    ax2.set_ylabel("lateral acceleration ay (g)")
    _shade_corners(ax1, lap_no, lap["start_time"])
    l1, la1 = ax1.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
    ax1.set_title(f"Lap {lap_no}: kinematic vs Kalman-observer sideslip (shaded = corner brackets)")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"lap{lap_no}_timeseries.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

    fig, ax1 = plt.subplots(figsize=(12, 5))
    diff = bc - ba
    ax1.plot(t_rel, diff, color="tab:red", linewidth=1.0, label="observer - kinematic beta")
    ax1.axhline(0, color="black", linewidth=0.5)
    ax1.set_xlabel("time since lap start (s)")
    ax1.set_ylabel("beta difference, observer - kinematic (deg)")
    ax2 = ax1.twinx()
    ax2.plot(t_rel, ayg, color="tab:gray", alpha=0.4, linewidth=0.8, label="ay (g)")
    ax2.set_ylabel("lateral acceleration ay (g)")
    _shade_corners(ax1, lap_no, lap["start_time"])
    l1, la1 = ax1.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
    ax1.set_title(f"Lap {lap_no}: observer minus kinematic sideslip difference (shaded = corner brackets)")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"lap{lap_no}_difference.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

# --- Shared setup for plots 3 & 4: canonical-window pooling + in-corner mask

laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)

in_corner_mask = np.zeros(len(t_ref), dtype=bool)
per_corner_medians = {}  # cid -> (median beta_A deg, median beta_C deg, median ay)

for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    pooled_a, pooled_c, pooled_ay = [], [], []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop <= sl.start:
            continue
        in_corner_mask[sl] = True
        m = moving[sl]
        if m.any():
            pooled_a.append(beta_a_deg[sl][m])
            pooled_c.append(beta_c_deg[sl][m])
            pooled_ay.append(ay[sl][m])
    if pooled_a:
        per_corner_medians[cid] = (
            float(np.median(np.concatenate(pooled_a))),
            float(np.median(np.concatenate(pooled_c))),
            float(np.median(np.concatenate(pooled_ay))),
        )

# --- Plot 3: per-corner summary --------------------------------------------

cids = sorted(per_corner_medians)
med_a = [per_corner_medians[c][0] for c in cids]
med_c = [per_corner_medians[c][1] for c in cids]
med_ay = [per_corner_medians[c][2] for c in cids]

x = np.arange(len(cids))
width = 0.35
fig, ax = plt.subplots(figsize=(14, 6))
ax.bar(x - width / 2, med_a, width, label="A_kinematic", color="tab:blue")
ax.bar(x + width / 2, med_c, width, label="C_kalman_observer", color="tab:orange")
ax.axhline(0, color="black", linewidth=0.5)
ax.set_xticks(x)
ax.set_xticklabels([f"C{c}" for c in cids])
ax.set_xlabel("stable corner id")
ax.set_ylabel("median sideslip beta (deg)")
ax.set_title("Per-corner median sideslip: kinematic vs Kalman observer "
             "(top labels: turn direction by ay sign, not verified L/R)")
ax.legend(fontsize=8)
ymax, ymin = max(med_a + med_c), min(med_a + med_c)
yrange = (ymax - ymin) if ymax > ymin else 1.0
ax.set_ylim(ymin - 0.05 * yrange, ymax + 0.12 * yrange)
for i, ayv in enumerate(med_ay):
    tag = "ay+" if ayv > 0 else "ay-" if ayv < 0 else "0"
    ax.text(x[i], ymax + 0.06 * yrange, tag, ha="center", fontsize=7, color="dimgray")
fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "per_corner_summary.png")
fig.savefig(fname, dpi=120)
plt.close(fig)
written.append(fname)

# --- Plot 4: scatter, kinematic vs observer, coloured by corner/straight ---

valid_windows = [(l["start_time"], l["end_time"]) for l in valid_laps]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)

scatter_mask = moving & racing_mask & np.isfinite(beta_a_deg) & np.isfinite(beta_c_deg)
straight_sel = scatter_mask & ~in_corner_mask
corner_sel = scatter_mask & in_corner_mask

fig, ax = plt.subplots(figsize=(7, 7))
ax.scatter(beta_a_deg[straight_sel], beta_c_deg[straight_sel], s=3, alpha=0.15,
           color="tab:blue", label=f"straight (n={int(straight_sel.sum())})")
ax.scatter(beta_a_deg[corner_sel], beta_c_deg[corner_sel], s=3, alpha=0.15,
           color="tab:orange", label=f"corner bracket (n={int(corner_sel.sum())})")
lims = [
    float(min(beta_a_deg[scatter_mask].min(), beta_c_deg[scatter_mask].min())),
    float(max(beta_a_deg[scatter_mask].max(), beta_c_deg[scatter_mask].max())),
]
ax.plot(lims, lims, color="black", linewidth=0.8, linestyle="--", label="y = x (perfect agreement)")
ax.set_xlabel("A_kinematic sideslip beta (deg)")
ax.set_ylabel("C_kalman_observer sideslip beta (deg)")
ax.set_title("Kinematic vs observer sideslip, all valid-lap moving samples")
ax.legend(fontsize=8, markerscale=3)
fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "scatter_kinematic_vs_observer.png")
fig.savefig(fname, dpi=150)
plt.close(fig)
written.append(fname)

# --- Summary printout --------------------------------------------------

print(f"Output folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")

diff_all = beta_c_deg - beta_a_deg
print()
print(f"Difference (observer - kinematic) stats, all valid-lap moving samples (n={int(scatter_mask.sum())}):")
print(f"  on straights (n={int(straight_sel.sum())}): median|diff|={np.median(np.abs(diff_all[straight_sel])):.3f} deg  "
      f"mean|diff|={np.mean(np.abs(diff_all[straight_sel])):.3f} deg")
print(f"  in corners   (n={int(corner_sel.sum())}): median|diff|={np.median(np.abs(diff_all[corner_sel])):.3f} deg  "
      f"mean|diff|={np.mean(np.abs(diff_all[corner_sel])):.3f} deg")
