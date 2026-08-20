# Plot companion to Phase 1's diagnostics/inspect_washout_cutoff_sweep.py
# (thesis_notes.md "Phase 1: washout cutoff sweep"). Read-only, no
# config/production change. Writes PNGs to diagnostics/plots/
# sideslip_washout_sweep/ (diagnostics/plots/ is gitignored recursively).
#
# NOT a cutoff recommendation -- candidates are shown side by side,
# unranked, for judging robustness across future tracks/drivers. The
# production 0.05 Hz default and pass-1 EKF beta are both included as
# REFERENCE traces (EKF explicitly labelled "reference, not truth" --
# it carries its own documented circularity, thesis_notes.md "WP-N2
# carry-forward decision: pass 1").
#
# Global beta per cutoff: production's own estimate_sideslip formula
# (cumsum(beta_dot)*dt, then _highpass_filter at the swept cutoff),
# identical construction to Phase 1's own sweep script.

import datetime
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, _highpass_filter,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS
from diagnostics._plot_common import (
    git_commit_info, canonical_window_slice, shade_corners_by_distance,
    pick_representative_corners, find_anchor_before, find_anchor_after,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots", "sideslip_washout_sweep")
os.makedirs(OUTPUT_DIR, exist_ok=True)
written = []

CUTOFFS = [0.05, 0.03, 0.02, 0.01]
CUTOFF_COLORS = {0.05: "tab:blue", 0.03: "tab:green", 0.02: "tab:orange", 0.01: "tab:red"}
EKF_COLOR = "black"
DRIFT_BOUND_MEDIAN_DEG = 0.9
DRIFT_BOUND_P90_DEG = 5.8
FORCE_BALANCE_LOW_DEG, FORCE_BALANCE_HIGH_DEG = 0.9, 5.8
CORNER_ZOOM_MARGIN_BEFORE_M = 30.0
CORNER_ZOOM_MARGIN_AFTER_M = 200.0
DRIFT_WINDOW_S = 4.0

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
sr = state["sample_rate_hz"]
dt = 1.0 / sr
s_m = state.get("s_m")
v = state["v_mps"]
ay = state["ay_mps2"]
yaw_rate = state["yaw_rate_radps"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

laps = data.get("laps", [])
valid_laps = [l for l in laps if l.get("is_valid_for_analysis")]
laps_by_number = {l["lap_number"]: l for l in laps}
valid_windows = [(l["start_time"], l["end_time"]) for l in valid_laps]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

corners = data.get("corners", [])
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)
racing_ids = [cid for cid in stable_ids if corners_by_stable_id[cid][0].get("speed_class") != "low"]

apex_half_window = params["stability_estimation"]["apex_half_window_samples"]

beta_dot_raw = np.where(moving_raw, ay / np.where(moving_raw, v, 1.0) - yaw_rate, 0.0)


def global_beta(cutoff):
    if cutoff <= 0.0:
        return np.where(moving_raw, np.cumsum(beta_dot_raw) * dt, 0.0)
    filt = _highpass_filter(np.cumsum(beta_dot_raw) * dt, cutoff, sr)
    return np.where(moving_raw, filt, 0.0)


beta_by_cutoff = {c: global_beta(c) for c in CUTOFFS}
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
beta_ekf = ekf_result["beta"]

# --- representative corners ------------------------------------------------

rep_cids, medians = pick_representative_corners(corners, racing_ids)
print(f"Representative corners chosen (median apex speed, km/h, racing-speed population n={len(racing_ids)}):")
for tag, cid in rep_cids.items():
    print(f"  {tag}: C{cid} ({medians[cid]:.1f} km/h)")

run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write("run label: sideslip_washout_sweep\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {git_commit_info()}\n")
    f.write("script: diagnostics/plot_washout_sweep.py\n")
    f.write(f"cutoffs plotted (beta_washout_cutoff_hz values, config default is 0.05): {CUTOFFS}\n")
    f.write("colors: " + ", ".join(f"{c}={CUTOFF_COLORS[c]}" for c in CUTOFFS) + f", EKF reference={EKF_COLOR} (dashed)\n")
    f.write("NOT a cutoff recommendation -- candidates shown unranked, side by side, for judging "
            "robustness across future tracks/drivers. pass-1 EKF beta is a REFERENCE trace, NOT ground "
            "truth -- it carries its own documented kinematic-fit circularity (thesis_notes.md 'WP-N2 "
            "carry-forward decision: pass 1').\n")
    f.write(f"representative corners (median apex speed, km/h, racing-speed population "
            f"n={len(racing_ids)}, speed_class != 'low'):\n")
    for tag, cid in rep_cids.items():
        f.write(f"  {tag}: C{cid} ({medians[cid]:.1f} km/h) -- chosen as "
                f"{'the minimum' if tag=='slow' else 'the maximum' if tag=='fast' else 'closest to the slow/fast midpoint'} "
                f"median apex speed among racing-speed corners\n")
    f.write(f"pre-registered drift bound (thesis_notes.md Phase 1): median >= {DRIFT_BOUND_MEDIAN_DEG} deg "
            f"OR p90 >= {DRIFT_BOUND_P90_DEG} deg disqualifies a cutoff -- drawn as horizontal reference "
            f"lines on the drift plot.\n")
    f.write(f"force-balance demand band (WP-S3b, all 14 corners, informative reference only -- Cr is "
            f"alpha-derived, not an independent magnitude check): {FORCE_BALANCE_LOW_DEG}-{FORCE_BALANCE_HIGH_DEG} deg\n")
    f.write("pass-1 EKF parameters (config/parameters.json tyre_model_ekf.pass_1), read live:\n")
    cfg1 = params["tyre_model_ekf"]["pass_1"]
    for k in ("c_alpha_front_n_per_rad", "c_alpha_rear_n_per_rad", "mu_fz_front_N", "mu_fz_rear_N",
              "Q_beta_var", "Q_yaw_rate_var", "R_yaw_rate_var", "R_ay_var"):
        f.write(f"  {k} = {cfg1[k]}\n")
    f.write(f"masked population (moving & ~kerb & valid-lap racing time) n={int(base_mask.sum())}\n")
    f.write(f"corner zoom margin: {CORNER_ZOOM_MARGIN_BEFORE_M} m before bracket_start_m, "
            f"{CORNER_ZOOM_MARGIN_AFTER_M} m after bracket_end_m; single representative lap instance per "
            f"corner plotted (first valid-lap instance) for legibility, NOT all-lap overlay.\n")
    f.write("\nSURPRISE, found while building drift_post_corner_straights.png (see chat report for the "
            "full account): Phase 1's own numeric drift metric (thesis_notes.md) checks only ONE instant "
            "(the post-corner straight-line anchor sample). This plot's causal-checkpoint construction "
            "shows that for 0.03/0.02/0.01 Hz, |beta| keeps RISING for several more seconds past that "
            "instant (0.02 Hz: ~0.25 deg at the anchor -> ~1.6 deg by 4s later; 0.01 Hz: ~0.2 deg -> "
            "~2.1 deg), crossing the pre-registered 0.9 deg median bound within roughly 1-1.5s of the "
            "single point Phase 1 actually measured. Production 0.05 Hz is the only cutoff that stays "
            "flat and low across the whole window. Phase 1's disqualifying-bound check therefore likely "
            "UNDERSTATES how much the three lower cutoffs drift on this data -- a limitation of that "
            "metric's single-point design, not of this plot. Not re-litigated here (Phase 1 is closed, "
            "this is a plotting task); flagged for whoever revisits the cutoff decision.\n")
written.append(run_info_path)


def _plot_cutoffs(ax, x, lo, hi, include_ekf=True):
    for c in CUTOFFS:
        ax.plot(x, np.degrees(beta_by_cutoff[c][lo:hi]), color=CUTOFF_COLORS[c], linewidth=1.0, label=f"{c} Hz")
    if include_ekf:
        ax.plot(x, np.degrees(beta_ekf[lo:hi]), color=EKF_COLOR, linewidth=1.1, linestyle="--",
                label="pass-1 EKF (reference, not truth)")


# --- SET 1a: full-lap beta vs distance, one file per lap -------------------

for lap in valid_laps:
    lap_no = lap["lap_number"]
    lo = int(np.searchsorted(t, lap["start_time"], side="left"))
    hi = int(np.searchsorted(t, lap["end_time"], side="right"))
    if hi <= lo or s_m is None:
        continue
    s_lap = s_m[lo:hi]
    finite = np.isfinite(s_lap)
    if not finite.any():
        continue

    fig, ax = plt.subplots(figsize=(14, 5))
    _plot_cutoffs(ax, s_lap, lo, hi)
    ax.set_xlabel("lap distance s (m)")
    ax.set_ylabel("sideslip beta (deg)")
    ax.set_title(f"Lap {lap_no}: washout cutoff sweep, beta vs distance (shaded = corner brackets)")
    shade_corners_by_distance(ax, corners, lap_no)
    ax.legend(fontsize=8, ncol=3, loc="upper right")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"lap{lap_no}_beta_vs_distance.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

# --- SET 1b: corner zooms, 3 representative corners -------------------------

for tag, cid in rep_cids.items():
    instances = [c for c in corners_by_stable_id[cid]
                 if laps_by_number.get(c["lap_number"], {}).get("is_valid_for_analysis")]
    if not instances:
        continue
    c0 = instances[0]
    lap = laps_by_number[c0["lap_number"]]
    bs, be = c0.get("bracket_start_m"), c0.get("bracket_end_m")
    lo = int(np.searchsorted(t, lap["start_time"], side="left"))
    hi = int(np.searchsorted(t, lap["end_time"], side="right"))
    s_lap = s_m[lo:hi]
    zoom_lo_m = bs - CORNER_ZOOM_MARGIN_BEFORE_M
    zoom_hi_m = be + CORNER_ZOOM_MARGIN_AFTER_M
    zoom_lo_local = int(np.searchsorted(s_lap, zoom_lo_m, side="left"))
    zoom_hi_local = int(np.searchsorted(s_lap, zoom_hi_m, side="right"))
    zoom_lo, zoom_hi = lo + zoom_lo_local, lo + zoom_hi_local
    if zoom_hi <= zoom_lo:
        continue
    x = s_m[zoom_lo:zoom_hi]

    fig, ax = plt.subplots(figsize=(12, 5))
    _plot_cutoffs(ax, x, zoom_lo, zoom_hi)
    ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0, label="corner bracket")
    ax.set_xlabel("lap distance s (m)")
    ax.set_ylabel("sideslip beta (deg)")
    ax.set_title(f"C{cid} ({tag}, {medians[cid]:.0f} km/h apex, lap {c0['lap_number']}): entry through exit "
                 f"+ following straight")
    ax.legend(fontsize=8, ncol=3, loc="upper right")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"corner_zoom_{tag}_C{cid}.png")
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    written.append(fname)

# --- SET 1c: drift on post-corner straights ---------------------------------
#
# TWO construction bugs found and fixed while building this plot (see
# the chat report for the full account -- both are real methodological
# traps, not just plotting mistakes):
#
# (1) A first draft used the GLOBAL per-cutoff beta signal (the same
#     one SET 1a/b use) instead of Phase 1's LOCAL RE-ANCHORED
#     construction. The global signal carries whatever drift
#     accumulated since the START OF THE SESSION, not just across one
#     corner, so it showed |beta| sitting well above the pre-
#     registered bound for the whole 4s window at 0.02/0.01 Hz --
#     contradicting Phase 1's own recorded numeric drift (0.02 Hz:
#     median=0.249, p90=0.784 deg).
# (2) Fixing (1) naively (re-anchor at the pre-corner straight, then
#     highpass-filter ONE long local segment spanning the full 4s
#     drift window, plot the whole filtered trace) STILL did not match
#     Phase 1's own t=0 figure: the filtered value AT the post-corner
#     exit sample changed depending on how much data came AFTER it,
#     because _highpass_filter uses scipy.signal.filtfilt --
#     zero-phase, ACAUSAL forward-backward filtering that uses future
#     samples to compute every output value. Phase 1's own metric
#     filters a segment ending exactly ONE sample past the exit anchor
#     (seg_end = i_exit + 1); extending that segment for a multi-second
#     plot changes the filter's own output at the ORIGINAL reference
#     point, not just what comes after it -- a genuine acausal-filter
#     visualization trap, not merely a different population.
# FIX: evaluate at a fixed set of causal checkpoints. At each
# checkpoint time, run the SAME local re-anchored construction with
# seg_end set to that checkpoint (never using data beyond it) -- the
# t=0 checkpoint is then bit-identical to Phase 1's own single-point
# definition, and every later checkpoint reflects only what a
# real-time observer would have known by then, not the future.

ay_g = ay / 9.81
yaw_rate_degps = np.degrees(yaw_rate)
straight_mask = moving_raw & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)
MIN_FILT_LEN = 30  # matches inspect_washout_cutoff_sweep.py's own floor above filtfilt's default padlen
CHECKPOINTS_S = np.arange(0.0, DRIFT_WINDOW_S + 1e-9, 0.25)


def _local_beta_at_checkpoint(i_anchor, i_exit, checkpoint_samples, cutoff):
    seg_end = i_exit + 1 + checkpoint_samples
    local_raw = np.cumsum(beta_dot_raw[i_anchor:seg_end]) * dt
    if cutoff <= 0.0 or (seg_end - i_anchor) < MIN_FILT_LEN:
        beta_local = local_raw
    else:
        try:
            beta_local = _highpass_filter(local_raw, cutoff, sr)
        except ValueError:
            beta_local = local_raw
    return beta_local[i_exit - i_anchor]  # the exit-anchor sample, filtered on data up to THIS checkpoint only


fig, axes = plt.subplots(len(CUTOFFS), 1, figsize=(11, 3.0 * len(CUTOFFS)), sharex=True, sharey=True)
for ax, cutoff in zip(axes, CUTOFFS):
    all_traces = []
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            lap_lo = int(np.searchsorted(t, lap["start_time"], side="left"))
            lap_hi = int(np.searchsorted(t, lap["end_time"], side="right"))
            i_anchor = find_anchor_before(straight_mask, lap_lo, sl.start)
            if i_anchor is None:
                continue
            i_exit = find_anchor_after(straight_mask, sl.stop, lap_hi)
            if i_exit is None:
                continue
            trace = []
            for cp_s in CHECKPOINTS_S:
                cp_samples = int(round(cp_s * sr))
                if i_exit + 1 + cp_samples > lap_hi:
                    break
                val = _local_beta_at_checkpoint(i_anchor, i_exit, cp_samples, cutoff)
                trace.append(abs(float(np.degrees(val))))
            if len(trace) < 2:
                continue
            trace_t = CHECKPOINTS_S[:len(trace)]
            ax.plot(trace_t, trace, color=CUTOFF_COLORS[cutoff], alpha=0.25, linewidth=0.7)
            all_traces.append((trace_t, trace))
    if all_traces:
        common_t = CHECKPOINTS_S
        padded = [np.interp(common_t, tr[0], tr[1], right=np.nan) for tr in all_traces]
        median_trace = np.nanmedian(np.array(padded), axis=0)
        ax.plot(common_t, median_trace, color="black", linewidth=1.8, label="median across corners")
    ax.axhline(DRIFT_BOUND_MEDIAN_DEG, color="tab:purple", linestyle=":", linewidth=1.0,
               label=f"pre-registered median bound ({DRIFT_BOUND_MEDIAN_DEG} deg)")
    ax.axhline(DRIFT_BOUND_P90_DEG, color="tab:purple", linestyle="--", linewidth=1.0,
               label=f"pre-registered p90 bound ({DRIFT_BOUND_P90_DEG} deg)")
    ax.set_ylabel(f"{cutoff} Hz\n|beta| (deg)")
    ax.legend(fontsize=7, loc="upper right")
axes[-1].set_xlabel("time since post-corner straight-line anchor (s), causal checkpoints")
fig.suptitle("Post-corner straight |beta| by cutoff, all corners, thin lines = individual corner instances")
fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "drift_post_corner_straights.png")
fig.savefig(fname, dpi=120)
plt.close(fig)
written.append(fname)

# --- SET 1d: apex-phase |beta| distributions, racing corners ---------------

apex_mask_racing = np.zeros_like(t, dtype=bool)
for c in corners:
    if c.get("stable_corner_id") not in racing_ids:
        continue
    start_t, end_t = c["segments"]["apex_3"]
    if end_t < start_t:
        continue
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window)
        hi = min(len(t), centre + apex_half_window + 1)
    apex_mask_racing[lo:hi] = True
apex_pop_mask = base_mask & apex_mask_racing

fig, ax = plt.subplots(figsize=(10, 6))
box_data = [np.abs(np.degrees(beta_by_cutoff[c][apex_pop_mask])) for c in CUTOFFS]
box_data.append(np.abs(np.degrees(beta_ekf[apex_pop_mask])))
labels = [f"{c} Hz" for c in CUTOFFS] + ["EKF (ref)"]
bp = ax.boxplot(box_data, labels=labels, showfliers=False, patch_artist=True)
for patch, c in zip(bp["boxes"], list(CUTOFF_COLORS.values()) + ["lightgray"]):
    patch.set_facecolor(c)
    patch.set_alpha(0.5)
ax.axhspan(FORCE_BALANCE_LOW_DEG, FORCE_BALANCE_HIGH_DEG, color="tab:purple", alpha=0.12,
           label=f"force-balance demand band ({FORCE_BALANCE_LOW_DEG}-{FORCE_BALANCE_HIGH_DEG} deg, "
                 f"WP-S3b, informative reference only)")
ax.set_ylabel("|beta| (deg)")
ax.set_title(f"Apex-phase |beta| distribution, racing-speed corners (n={len(racing_ids)}), "
             f"n_samples={int(apex_pop_mask.sum())}")
ax.legend(fontsize=8, loc="upper right")
fig.tight_layout()
fname = os.path.join(OUTPUT_DIR, "apex_beta_distribution.png")
fig.savefig(fname, dpi=130)
plt.close(fig)
written.append(fname)

print(f"\nOutput folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")
