# SET 3: estimator comparison plots (this package's "next turn" plot
# request). Read-only, no config/production change. Usage:
#   python -m diagnostics.plot_estimator_comparison dugoff
#   python -m diagnostics.plot_estimator_comparison pacejka
# Writes to diagnostics/plots/sideslip_dugoff_vs_kin/ or
# diagnostics/plots/sideslip_pacejka_vs_kin/ respectively.
#
# "New kinematic" is not decided (per the request) -- two kinematic
# CANDIDATES are shown (0.03, 0.02 Hz), never a single "new default",
# alongside production 0.05 Hz and the EKF variant. EKF fit parameters
# are read LIVE by calling modules.tyre_fit_auto.fit_session /
# fit_session_pacejka at plot time (Phase 2/3's own reusable chain),
# never hardcoded, so this script always reflects the current fit
# regardless of future config changes.

import datetime
import os
import sys

import numpy as np
from scipy.stats import pearsonr
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, _highpass_filter,
)
from modules.tyre_model import dugoff_lateral_force
from modules.tyre_model_pacejka import pacejka_lateral_force
from modules.tyre_fit_auto import fit_session, fit_session_pacejka
from diagnostics._plot_common import git_commit_info, pick_representative_corners

VARIANT = sys.argv[1] if len(sys.argv) > 1 else None
if VARIANT not in ("dugoff", "pacejka"):
    raise SystemExit("usage: python -m diagnostics.plot_estimator_comparison <dugoff|pacejka>")

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots",
                          f"sideslip_{VARIANT}_vs_kin")
os.makedirs(OUTPUT_DIR, exist_ok=True)
written = []

KIN_CANDIDATES = [0.03, 0.02]
KIN_CANDIDATE_COLORS = {0.03: "tab:green", 0.02: "tab:orange"}
PRODUCTION_CUTOFF = 0.05
PRODUCTION_COLOR = "tab:blue"
EKF_COLOR = "black"
CLOUD_CANDIDATE_CUTOFF = 0.03  # single kinematic candidate used in the 4-way force-vs-slip cloud (e)
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
rep_cids, medians = pick_representative_corners(corners, racing_ids)

# --- kinematic candidates + production, all via the production washout formula --

beta_dot_raw = np.where(moving_raw, ay / np.where(moving_raw, v, 1.0) - yaw_rate, 0.0)


def global_beta(cutoff):
    filt = _highpass_filter(np.cumsum(beta_dot_raw) * dt, cutoff, sr)
    return np.where(moving_raw, filt, 0.0)


beta_production = global_beta(PRODUCTION_CUTOFF)
beta_by_candidate = {c: global_beta(c) for c in KIN_CANDIDATES}
slip_production = estimate_slip_angles(state, beta_production, params)
slip_by_candidate = {c: estimate_slip_angles(state, beta_by_candidate[c], params) for c in KIN_CANDIDATES}
forces = estimate_lateral_forces(state, params)

# --- both EKF fits, read live -- item (e) needs both regardless of VARIANT --

print("Running Dugoff auto-fit chain (live, Phase 2 chain)...")
dugoff_fit = fit_session(data, params, data_file_path=RAW_FILE)
beta_dugoff = dugoff_fit.pop("beta_ekf")
slip_dugoff = estimate_slip_angles(state, beta_dugoff, params)

print("Running Pacejka auto-fit chain (live, Phase 3 chain)...")
pacejka_fit = fit_session_pacejka(data, params, data_file_path=RAW_FILE)
beta_pacejka = pacejka_fit.pop("beta_ekf")
slip_pacejka = estimate_slip_angles(state, beta_pacejka, params)

if VARIANT == "dugoff":
    beta_ekf, slip_ekf, ekf_fit, EKF_LABEL = beta_dugoff, slip_dugoff, dugoff_fit, "Dugoff EKF"
else:
    beta_ekf, slip_ekf, ekf_fit, EKF_LABEL = beta_pacejka, slip_pacejka, pacejka_fit, "Pacejka EKF"

# --- run_info.txt ------------------------------------------------------------

run_info_path = os.path.join(OUTPUT_DIR, "run_info.txt")
with open(run_info_path, "w", encoding="utf-8") as f:
    f.write(f"run label: sideslip_{VARIANT}_vs_kin\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {git_commit_info()}\n")
    f.write("script: diagnostics/plot_estimator_comparison.py\n")
    f.write(f"variant: {VARIANT} ({EKF_LABEL})\n")
    f.write(f"kinematic candidates plotted (NOT a 'new kinematic' decision -- two candidates shown "
            f"side by side): {KIN_CANDIDATES}, colors "
            + ", ".join(f"{c}={KIN_CANDIDATE_COLORS[c]}" for c in KIN_CANDIDATES) + "\n")
    f.write(f"production kinematic: {PRODUCTION_CUTOFF} Hz, color={PRODUCTION_COLOR}\n")
    f.write(f"EKF variant color={EKF_COLOR} (dashed where overlaid with kinematic traces)\n")
    f.write(f"force-vs-slip cloud (e): single kinematic candidate used = {CLOUD_CANDIDATE_CUTOFF} Hz "
            f"(chosen as Phase 1's flagged candidate; NOT re-plotted for both candidates to keep the "
            f"4-panel comparison legible)\n")
    f.write("same representative corners as sideslip_washout_sweep/ and slipangle_washout_sweep/:\n")
    for tag, cid in rep_cids.items():
        f.write(f"  {tag}: C{cid} ({medians[cid]:.1f} km/h)\n")
    f.write(f"\n--- {EKF_LABEL} fit parameters, read LIVE from modules.tyre_fit_auto."
            f"{'fit_session' if VARIANT=='dugoff' else 'fit_session_pacejka'} (this run) ---\n")
    f.write(f"status: {ekf_fit.get('status')}\n")
    for axle in ("front", "rear"):
        f.write(f"  {axle}: {ekf_fit['axles'][axle]}\n")
    f.write(f"final_config: {ekf_fit.get('final_config')}\n")
    f.write(f"nis: {ekf_fit.get('nis')}\n")
    f.write(f"sign_check: {ekf_fit.get('sign_check')}\n")
    f.write(f"\n--- Dugoff fit parameters (for item e's 4-way cloud, always computed) ---\n")
    for axle in ("front", "rear"):
        f.write(f"  {axle}: {dugoff_fit['axles'][axle]}\n")
    f.write(f"\n--- Pacejka fit parameters (for item e's 4-way cloud, always computed) ---\n")
    for axle in ("front", "rear"):
        f.write(f"  {axle}: {pacejka_fit['axles'][axle]}\n")
    f.write(f"\nmasked population (moving & ~kerb & valid-lap racing time) n={int(base_mask.sum())}\n")
written.append(run_info_path)


def _series(source, axle):
    return source["alpha_f_filt"] if axle == "front" else source["alpha_r_filt"]


# --- (a) beta vs distance, per lap -------------------------------------------

for lap in valid_laps:
    lap_no = lap["lap_number"]
    lo = int(np.searchsorted(t, lap["start_time"], side="left"))
    hi = int(np.searchsorted(t, lap["end_time"], side="right"))
    if hi <= lo or s_m is None:
        continue
    s_lap = s_m[lo:hi]
    if not np.isfinite(s_lap).any():
        continue

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(s_lap, np.degrees(beta_production[lo:hi]), color=PRODUCTION_COLOR, linewidth=1.0,
             label=f"kinematic {PRODUCTION_CUTOFF} Hz (production)")
    for c in KIN_CANDIDATES:
        ax.plot(s_lap, np.degrees(beta_by_candidate[c][lo:hi]), color=KIN_CANDIDATE_COLORS[c],
                 linewidth=1.0, label=f"kinematic {c} Hz (candidate)")
    ax.plot(s_lap, np.degrees(beta_ekf[lo:hi]), color=EKF_COLOR, linewidth=1.2, linestyle="--",
             label=f"{EKF_LABEL} (fitted, this run)")
    ax.set_xlabel("lap distance s (m)")
    ax.set_ylabel("sideslip beta (deg)")
    ax.set_title(f"Lap {lap_no}: {EKF_LABEL} vs kinematic candidates, beta vs distance")
    for c in corners:
        if c["lap_number"] != lap_no:
            continue
        bs, be = c.get("bracket_start_m"), c.get("bracket_end_m")
        if bs is None or be is None or be <= bs:
            continue
        ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0)
    ax.legend(fontsize=8, ncol=2, loc="upper right")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"lap{lap_no}_beta_vs_distance.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
    written.append(fname)

# --- (b) + (c) corner zooms: beta, then alpha front/rear -------------------

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

    # (b) beta
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(x, np.degrees(beta_production[zoom_lo:zoom_hi]), color=PRODUCTION_COLOR, linewidth=1.0,
             label=f"kinematic {PRODUCTION_CUTOFF} Hz")
    for c in KIN_CANDIDATES:
        ax.plot(x, np.degrees(beta_by_candidate[c][zoom_lo:zoom_hi]), color=KIN_CANDIDATE_COLORS[c],
                 linewidth=1.0, label=f"kinematic {c} Hz")
    ax.plot(x, np.degrees(beta_ekf[zoom_lo:zoom_hi]), color=EKF_COLOR, linewidth=1.2, linestyle="--",
             label=EKF_LABEL)
    ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0, label="corner bracket")
    ax.set_xlabel("lap distance s (m)")
    ax.set_ylabel("sideslip beta (deg)")
    ax.set_title(f"C{cid} ({tag}, {medians[cid]:.0f} km/h apex): {EKF_LABEL} vs kinematic, beta")
    ax.legend(fontsize=7, ncol=2, loc="upper right")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"corner_zoom_{tag}_C{cid}_beta.png")
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    written.append(fname)

    # (c) alpha front/rear
    for axle in ("front", "rear"):
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(x, np.degrees(_series(slip_production, axle))[zoom_lo:zoom_hi], color=PRODUCTION_COLOR,
                 linewidth=1.0, label=f"kinematic {PRODUCTION_CUTOFF} Hz")
        for c in KIN_CANDIDATES:
            ax.plot(x, np.degrees(_series(slip_by_candidate[c], axle))[zoom_lo:zoom_hi],
                     color=KIN_CANDIDATE_COLORS[c], linewidth=1.0, label=f"kinematic {c} Hz")
        ax.plot(x, np.degrees(_series(slip_ekf, axle))[zoom_lo:zoom_hi], color=EKF_COLOR,
                 linewidth=1.2, linestyle="--", label=EKF_LABEL)
        ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0, label="corner bracket")
        ax.set_xlabel("lap distance s (m)")
        ax.set_ylabel(f"{axle} slip angle alpha (deg)")
        ax.set_title(f"C{cid} ({tag}, {medians[cid]:.0f} km/h apex): {EKF_LABEL} vs kinematic, {axle} alpha")
        ax.legend(fontsize=7, ncol=2, loc="upper right")
        fig.tight_layout()
        fname = os.path.join(OUTPUT_DIR, f"corner_zoom_{tag}_C{cid}_alpha_{axle}.png")
        fig.savefig(fname, dpi=130)
        plt.close(fig)
        written.append(fname)

# --- (d) scatter: EKF beta vs each kinematic candidate ----------------------

scatter_stats = {}
for c in KIN_CANDIDATES:
    x_vals = np.degrees(beta_by_candidate[c][base_mask])
    y_vals = np.degrees(beta_ekf[base_mask])
    corr = float(pearsonr(x_vals, y_vals)[0])
    rms = float(np.sqrt(np.mean((y_vals - x_vals) ** 2)))
    scatter_stats[c] = (corr, rms)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(x_vals, y_vals, s=3, alpha=0.15, color=KIN_CANDIDATE_COLORS[c])
    lims = [float(min(x_vals.min(), y_vals.min())), float(max(x_vals.max(), y_vals.max()))]
    ax.plot(lims, lims, color="black", linewidth=0.8, linestyle="--", label="y = x")
    ax.set_xlabel(f"kinematic {c} Hz beta (deg)")
    ax.set_ylabel(f"{EKF_LABEL} beta (deg)")
    ax.set_title(f"{EKF_LABEL} vs kinematic {c} Hz, n={int(base_mask.sum())}\n"
                 f"corr={corr:.4f}  RMS diff={rms:.3f} deg")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"scatter_ekf_vs_kin{str(c).replace('.', '')}.png")
    fig.savefig(fname, dpi=140)
    plt.close(fig)
    written.append(fname)

# --- (e) force-vs-slip-angle cloud, four ways, per axle --------------------

in_corner_mask = np.zeros_like(t, dtype=bool)
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
        lo = int(np.searchsorted(t, lap["start_time"], side="left"))
        hi = int(np.searchsorted(t, lap["end_time"], side="right"))
        s_lap = s_m[lo:hi]
        finite = np.isfinite(s_lap)
        if not finite.any():
            continue
        lap_s_lo, lap_s_hi = float(np.min(s_lap[finite])), float(np.max(s_lap[finite]))
        s0 = int(np.searchsorted(s_lap, max(lap_s_lo, bracket_start), side="left"))
        s1 = int(np.searchsorted(s_lap, min(lap_s_hi, bracket_end), side="right"))
        if s1 > s0:
            in_corner_mask[lo + s0:lo + s1] = True
corner_valid_mask = base_mask & in_corner_mask

for axle in ("front", "rear"):
    Fy = forces["Fy_f_filt"] if axle == "front" else forces["Fy_r_filt"]
    fit_d = dugoff_fit["axles"][axle]
    fit_p = pacejka_fit["axles"][axle]
    alpha_dugoff_own = _series(slip_dugoff, axle)
    alpha_pacejka_own = _series(slip_pacejka, axle)

    panels = [
        ("production kinematic\n(0.05 Hz)", _series(slip_production, axle), None),
        (f"candidate kinematic\n({CLOUD_CANDIDATE_CUTOFF} Hz)",
         _series(slip_by_candidate[CLOUD_CANDIDATE_CUTOFF], axle), None),
        ("Dugoff EKF\n(own alpha)", alpha_dugoff_own,
         lambda a: dugoff_lateral_force(a, fit_d["c_alpha_n_per_rad"], fit_d["mu_fz_N"])),
        ("Pacejka EKF\n(own alpha)", alpha_pacejka_own,
         lambda a: pacejka_lateral_force(a, fit_p["B"], fit_p["C"], fit_p["D"], fit_p["E"])),
    ]

    fig, axes = plt.subplots(1, 4, figsize=(20, 6), sharey=True)
    for ax, (label, alpha_arr, curve_fn) in zip(axes, panels):
        x_vals = np.degrees(alpha_arr[corner_valid_mask])
        y_vals = Fy[corner_valid_mask]
        finite = np.isfinite(x_vals) & np.isfinite(y_vals)
        ax.scatter(x_vals[finite], y_vals[finite], s=3, alpha=0.12, color="tab:blue")
        if curve_fn is not None:
            a_grid_deg = np.linspace(np.percentile(x_vals[finite], 0.5), np.percentile(x_vals[finite], 99.5), 200)
            fy_grid = curve_fn(np.radians(a_grid_deg))
            ax.plot(a_grid_deg, fy_grid, color="tab:red", linewidth=1.3, label="fitted curve")
            ax.legend(fontsize=7)
        ax.set_xlabel("slip angle alpha (deg)")
        ax.set_title(label, fontsize=9)
    axes[0].set_ylabel(f"{axle} lateral force Fy (N)")
    fig.suptitle(f"{axle.capitalize()} force vs slip angle, four ways, corner samples only "
                 f"(n={int(corner_valid_mask.sum())})")
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"force_vs_slip_{axle}_four_ways.png")
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    written.append(fname)

print(f"\nOutput folder: {OUTPUT_DIR}")
for f in written:
    print(f"  {os.path.relpath(f, os.path.dirname(__file__))}")
print("\nEKF-vs-kinematic-candidate scatter stats:")
for c, (corr, rms) in scatter_stats.items():
    print(f"  vs {c} Hz: corr={corr:.4f}  RMS diff={rms:.3f} deg")
