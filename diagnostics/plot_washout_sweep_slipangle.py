# Slip-angle companion to diagnostics/plot_washout_sweep.py -- same
# cutoffs, same 3 representative corners, but alpha_f/alpha_r (the
# quantity Module 4b's CS_ratio actually consumes) instead of beta.
# Read-only, no config/production change. Writes PNGs to diagnostics/
# plots/slipangle_washout_sweep/. NOT a cutoff recommendation.

import datetime
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles, _highpass_filter,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from diagnostics._plot_common import git_commit_info, pick_representative_corners

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots", "slipangle_washout_sweep")
os.makedirs(OUTPUT_DIR, exist_ok=True)
written = []

CUTOFFS = [0.05, 0.03, 0.02, 0.01]
CUTOFF_COLORS = {0.05: "tab:blue", 0.03: "tab:green", 0.02: "tab:orange", 0.01: "tab:red"}
EKF_COLOR = "black"
CORNER_ZOOM_MARGIN_BEFORE_M = 30.0
CORNER_ZOOM_MARGIN_AFTER_M = 200.0

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

beta_dot_raw = np.where(moving_raw, ay / np.where(moving_raw, v, 1.0) - yaw_rate, 0.0)


def global_beta(cutoff):
    if cutoff <= 0.0:
        return np.where(moving_raw, np.cumsum(beta_dot_raw) * dt, 0.0)
    filt = _highpass_filter(np.cumsum(beta_dot_raw) * dt, cutoff, sr)
    return np.where(moving_raw, filt, 0.0)


slip_by_cutoff = {c: estimate_slip_angles(state, global_beta(c), params) for c in CUTOFFS}
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
slip_ekf = estimate_slip_angles(state, ekf_result["beta"], params)

cfg1 = params["tyre_model_ekf"]["pass_1"]
onset_f_deg = float(np.degrees(np.arctan(cfg1["mu_fz_front_N"] / (2.0 * cfg1["c_alpha_front_n_per_rad"]))))
onset_r_deg = float(np.degrees(np.arctan(cfg1["mu_fz_rear_N"] / (2.0 * cfg1["c_alpha_rear_n_per_rad"]))))

rep_cids, medians = pick_representative_corners(corners, racing_ids)

run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write("run label: slipangle_washout_sweep\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {git_commit_info()}\n")
    f.write("script: diagnostics/plot_washout_sweep_slipangle.py\n")
    f.write(f"cutoffs plotted: {CUTOFFS}; colors: " + ", ".join(f"{c}={CUTOFF_COLORS[c]}" for c in CUTOFFS)
            + f", EKF reference={EKF_COLOR} (dashed)\n")
    f.write("NOT a cutoff recommendation -- shows what the washout cutoff choice does to the quantity "
            "Module 4b's CS_ratio actually consumes (alpha_f/alpha_r), same 3 representative corners as "
            "diagnostics/plots/sideslip_washout_sweep/. pass-1 EKF alpha is a REFERENCE trace, not truth.\n")
    f.write("same representative-corner selection as sideslip_washout_sweep/run_info.txt:\n")
    for tag, cid in rep_cids.items():
        f.write(f"  {tag}: C{cid} ({medians[cid]:.1f} km/h)\n")
    f.write(f"Dugoff onset boundary (pass-1 frozen curve, lambda=1, linear/saturated regime split): "
            f"front={onset_f_deg:.3f} deg, rear={onset_r_deg:.3f} deg -- drawn as reference lines on the "
            f"distribution plots.\n")
    f.write("slip angles in DEGREES throughout (production Fy-vs-alpha convention, modules/tyre_model.py "
            "header).\n")
written.append(run_info_path)

AXLES = ["front", "rear"]


def _series(source, axle):
    return source["alpha_f_filt"] if axle == "front" else source["alpha_r_filt"]


# --- corner zooms, front/rear, 3 representative corners ---------------------

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
    zoom_lo_local = int(np.searchsorted(s_lap, bs - CORNER_ZOOM_MARGIN_BEFORE_M, side="left"))
    zoom_hi_local = int(np.searchsorted(s_lap, be + CORNER_ZOOM_MARGIN_AFTER_M, side="right"))
    zoom_lo, zoom_hi = lo + zoom_lo_local, lo + zoom_hi_local
    if zoom_hi <= zoom_lo:
        continue
    x = s_m[zoom_lo:zoom_hi]

    for axle in AXLES:
        fig, ax = plt.subplots(figsize=(12, 5))
        for c in CUTOFFS:
            ax.plot(x, np.degrees(_series(slip_by_cutoff[c], axle))[zoom_lo:zoom_hi],
                     color=CUTOFF_COLORS[c], linewidth=1.0, label=f"{c} Hz")
        ax.plot(x, np.degrees(_series(slip_ekf, axle))[zoom_lo:zoom_hi], color=EKF_COLOR,
                 linewidth=1.1, linestyle="--", label="pass-1 EKF (reference, not truth)")
        onset = onset_f_deg if axle == "front" else onset_r_deg
        ax.axhline(onset, color="tab:purple", linestyle=":", linewidth=0.9, label=f"onset (+{onset:.2f} deg)")
        ax.axhline(-onset, color="tab:purple", linestyle=":", linewidth=0.9)
        ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0, label="corner bracket")
        ax.set_xlabel("lap distance s (m)")
        ax.set_ylabel(f"{axle} slip angle alpha (deg)")
        ax.set_title(f"C{cid} ({tag}, {medians[cid]:.0f} km/h apex, lap {c0['lap_number']}): "
                     f"{axle} slip angle, entry through exit + straight")
        ax.legend(fontsize=7, ncol=3, loc="upper right")
        fig.tight_layout()
        fname = os.path.join(OUTPUT_DIR, f"corner_zoom_{tag}_C{cid}_{axle}.png")
        fig.savefig(fname, dpi=130)
        plt.close(fig)
        written.append(fname)

# --- per-axle |alpha| distributions, racing-corner apex phases -------------

apex_half_window = params["stability_estimation"]["apex_half_window_samples"]
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

for axle in AXLES:
    onset = onset_f_deg if axle == "front" else onset_r_deg
    fig, ax = plt.subplots(figsize=(10, 6))
    box_data = [np.abs(np.degrees(_series(slip_by_cutoff[c], axle))[apex_pop_mask]) for c in CUTOFFS]
    box_data.append(np.abs(np.degrees(_series(slip_ekf, axle))[apex_pop_mask]))
    labels = [f"{c} Hz" for c in CUTOFFS] + ["EKF (ref)"]
    bp = ax.boxplot(box_data, labels=labels, showfliers=False, patch_artist=True)
    for patch, c in zip(bp["boxes"], list(CUTOFF_COLORS.values()) + ["lightgray"]):
        patch.set_facecolor(c)
        patch.set_alpha(0.5)
    ax.axhline(onset, color="tab:purple", linestyle=":", linewidth=1.0,
               label=f"pass-1 Dugoff onset boundary ({onset:.2f} deg)")
    ax.set_ylabel(f"{axle} |alpha| (deg)")
    ax.set_title(f"{axle.capitalize()} apex-phase |alpha| distribution, racing-speed corners "
                 f"(n={len(racing_ids)}), n_samples={int(apex_pop_mask.sum())}")
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"apex_alpha_distribution_{axle}.png")
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    written.append(fname)

print(f"\nOutput folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")
