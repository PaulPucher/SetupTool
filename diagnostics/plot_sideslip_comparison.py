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
#
# Optional second command-line argument selects the
# observer plotted as method "C" -- "linear" (default, unchanged
# behaviour) or "dugoff_pass0" (nonlinear Dugoff EKF, frozen pass-0
# parameters). dugoff_pass0 plots the RAW EKF beta (pre-fallback, not
# beta_with_fallback) and additionally shades diverged_mask regions in
# red on the per-lap time-series plots -- the 27.8% of samples the
# divergence monitor flags stay visible rather than hidden by a silent
# fallback substitution, matching diagnostics/sideslip_ekf_dugoff.py's
# own "never silently" design.

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
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if len(sys.argv) > 1:
    RUN_LABEL = sys.argv[1]
else:
    RUN_LABEL = datetime.date.today().isoformat()
    print(f"No run label given on the command line -- using today's date as the "
          f"folder name: {RUN_LABEL}")

OBSERVER_MODE = sys.argv[2] if len(sys.argv) > 2 else "linear"
if OBSERVER_MODE not in ("linear", "dugoff_pass0", "dugoff_pass1"):
    raise SystemExit(f"unknown observer mode {OBSERVER_MODE!r}, expected "
                      f"'linear', 'dugoff_pass0', or 'dugoff_pass1'")

# The observer's tyre model. Every plot this script produces used this
# model; the run label and run_info.txt both record it explicitly so
# the observers' plots are never mistaken for each other, without
# needing tyre-model text burned into the figures themselves.
if OBSERVER_MODE == "linear":
    TYRE_MODEL_TAG = "linear_tyre"
    TYRE_MODEL_DESC = ("linear (fixed stiffness prior, Caf/Car from config/parameters.json "
                        "cs_front/rear_fallback_reference_n_per_rad) -- see thesis_notes.md "
                        "'Linear observer saturation-detection failure'")
    METHOD_C_LABEL = "C_kalman_observer"
elif OBSERVER_MODE == "dugoff_pass0":
    TYRE_MODEL_TAG = "dugoff_pass0"
    TYRE_MODEL_DESC = ("nonlinear Dugoff EKF, pass 0 (frozen WP-N1b parameters, "
                        "config/parameters.json tyre_model_ekf.pass_0, no refit) -- "
                        "RAW beta plotted (pre-fallback), diverged_mask shaded red")
    METHOD_C_LABEL = "C_dugoff_ekf_pass0"
else:
    TYRE_MODEL_TAG = "dugoff_pass1"
    TYRE_MODEL_DESC = ("nonlinear Dugoff EKF, pass 1 (frozen pass-0 Dugoff parameters, "
                        "config/parameters.json tyre_model_ekf.pass_1, noise-model-only "
                        "recalibration -- 2-D sweep-refined R_ay/R_yaw_rate) -- RAW beta "
                        "plotted (pre-fallback), diverged_mask shaded red")
    METHOD_C_LABEL = "C_dugoff_ekf_pass1"

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
if OBSERVER_MODE == "linear":
    beta_c_deg = np.degrees(estimate_sideslip_kalman(state, params))
    diverged_mask_full = None
else:
    ekf_pass_id = "pass_0" if OBSERVER_MODE == "dugoff_pass0" else "pass_1"
    ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id=ekf_pass_id)
    beta_c_deg = np.degrees(ekf_result["beta"])  # RAW, pre-fallback -- see header note
    diverged_mask_full = ekf_result["diverged_mask"]

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
    f.write(f"observer mode: {OBSERVER_MODE}\n")
    f.write(f"tyre model: {TYRE_MODEL_DESC}\n")
    if OBSERVER_MODE == "linear":
        f.write("observer Q/R settings (diagnostics/sideslip_kalman_observer.py):\n")
        f.write(f"  Q_BETA_VAR = {float(Q_BETA_VAR):.6e} rad^2\n")
        f.write(f"  Q_YAW_RATE_VAR = {float(Q_YAW_RATE_VAR):.6e} (rad/s)^2\n")
        f.write(f"  R_YAW_RATE_VAR = {float(R_YAW_RATE_VAR):.6e} (rad/s)^2\n")
        f.write(f"  R_AY_VAR = {float(R_AY_VAR):.6e} (m/s^2)^2\n")
        f.write(f"  P0_BETA_VAR = {float(P0_BETA_VAR):.6e} rad^2\n")
        f.write(f"  P0_YAW_RATE_VAR = {float(P0_YAW_RATE_VAR):.6e} (rad/s)^2\n")
    else:
        cfg = params["tyre_model_ekf"][ekf_pass_id]
        f.write("plotted series: RAW EKF beta (pre-fallback, result['beta']), NOT beta_with_fallback\n")
        f.write(f"diverged_mask: shaded red on the per-lap time-series plots below, computed from THIS "
                f"run's own config values -- nis_window_samples={cfg['nis_window_samples']}, "
                f"nis_chi2_bound={cfg['nis_chi2_bound']}, nis_flag_fraction={cfg['nis_flag_fraction']} "
                f"(config tyre_model_ekf.{ekf_pass_id}.nis_*) -- these are PLACEHOLDER defaults, not "
                f"data-derived or validated (nis_tuning_note in config). NOTE: an earlier, separate "
                f"analysis (diagnostics/inspect_nis_short_run_blindspot.py) explored a DIFFERENT, "
                f"proposed-but-never-applied threshold (window=25, flag_fraction=1.0) and found the "
                f"monitor at THAT threshold structurally blind to exceedance runs shorter than the "
                f"window -- roughly half of all NIS-exceeding pass-0 samples, median burst length 9 -- "
                f"but that finding is about the 25/1.0 threshold, NOT the 20/0.5 threshold actually "
                f"driving the shading in this plot; whether the same blind spot holds at 20/0.5, or at "
                f"the calibrated pass-1 setting, has not been re-measured.\n")
        if ekf_pass_id == "pass_1":
            f.write("Dugoff c_alpha/mu_fz: UNCHANGED from pass_0 (noise-model-only recalibration, "
                    "see config tyre_model_ekf.pass_1.changed_from_previous).\n")
        f.write(f"frozen {ekf_pass_id} parameters (config/parameters.json tyre_model_ekf.{ekf_pass_id}):\n")
        f.write(f"  c_alpha_front_n_per_rad = {cfg['c_alpha_front_n_per_rad']:.4f}\n")
        f.write(f"  c_alpha_rear_n_per_rad = {cfg['c_alpha_rear_n_per_rad']:.4f}\n")
        f.write(f"  mu_fz_front_N = {cfg['mu_fz_front_N']:.4f}\n")
        f.write(f"  mu_fz_rear_N = {cfg['mu_fz_rear_N']:.4f}\n")
        f.write(f"  Q_beta_var = {cfg['Q_beta_var']:.6e} rad^2\n")
        f.write(f"  Q_yaw_rate_var = {cfg['Q_yaw_rate_var']:.6e} (rad/s)^2\n")
        f.write(f"  R_yaw_rate_var = {cfg['R_yaw_rate_var']:.6e} (rad/s)^2\n")
        f.write(f"  R_ay_var = {cfg['R_ay_var']:.6e} (m/s^2)^2\n")
        f.write(f"  P0_beta_var = {cfg['P0_beta_var']:.6e} rad^2\n")
        f.write(f"  P0_yaw_rate_var = {cfg['P0_yaw_rate_var']:.6e} (rad/s)^2\n")
        f.write(f"  beta_hard_bound_deg = {cfg['beta_hard_bound_deg']}\n")
        f.write(f"  nis_window_samples = {cfg['nis_window_samples']} (placeholder)\n")
        f.write(f"  nis_chi2_bound = {cfg['nis_chi2_bound']} (placeholder)\n")
        f.write(f"  nis_flag_fraction = {cfg['nis_flag_fraction']} (placeholder)\n")
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


def _shade_diverged(ax, t_rel, diverged_lap):
    # dugoff_pass0 only -- makes the divergence-monitor-flagged samples
    # visible on the trace instead of hiding them behind the silently
    # fallback-corrected series (this script deliberately plots the RAW
    # beta, see header note).
    if diverged_lap is None or not diverged_lap.any():
        return
    idx = np.where(diverged_lap)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    for k, (s_i, e_i) in enumerate(zip(starts, ends)):
        e_i_incl = min(e_i + 1, len(t_rel) - 1)
        ax.axvspan(t_rel[s_i], t_rel[e_i_incl], color="red", alpha=0.20, lw=0, zorder=0,
                   label="diverged_mask (NIS/beta-bound flagged)" if k == 0 else None)


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
    diverged_lap = diverged_mask_full[lo:hi] if diverged_mask_full is not None else None

    fig, ax1 = plt.subplots(figsize=(12, 5))
    ax1.plot(t_rel, ba, label="A_kinematic beta", color="tab:blue", linewidth=1.0)
    ax1.plot(t_rel, bc, label=f"{METHOD_C_LABEL} beta", color="tab:orange", linewidth=1.0)
    ax1.set_xlabel("time since lap start (s)")
    ax1.set_ylabel("sideslip beta (deg)")
    ax2 = ax1.twinx()
    ax2.plot(t_rel, ayg, color="tab:gray", alpha=0.4, linewidth=0.8, label="ay (g)")
    ax2.set_ylabel("lateral acceleration ay (g)")
    _shade_corners(ax1, lap_no, lap["start_time"])
    _shade_diverged(ax1, t_rel, diverged_lap)
    l1, la1 = ax1.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
    ax1.set_title(f"Lap {lap_no}: kinematic vs {METHOD_C_LABEL} sideslip (shaded = corner brackets)")
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
    _shade_diverged(ax1, t_rel, diverged_lap)
    l1, la1 = ax1.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
    ax1.set_title(f"Lap {lap_no}: {METHOD_C_LABEL} minus kinematic sideslip difference (shaded = corner brackets)")
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
ax.bar(x + width / 2, med_c, width, label=METHOD_C_LABEL, color="tab:orange")
ax.axhline(0, color="black", linewidth=0.5)
ax.set_xticks(x)
ax.set_xticklabels([f"C{c}" for c in cids])
ax.set_xlabel("stable corner id")
ax.set_ylabel("median sideslip beta (deg)")
ax.set_title(f"Per-corner median sideslip: kinematic vs {METHOD_C_LABEL} "
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
ax.set_ylabel(f"{METHOD_C_LABEL} sideslip beta (deg)")
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
