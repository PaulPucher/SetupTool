# Morning follow-up, Item 2 (updated per the "Closing the reconstruction's
# aero gap" work order): FRONT axle comparison figure -- measured FL vs
# RECONSTRUCTED FR (modules.wheel_loads.reconstruct_missing_corner),
# GT3_PRC_MLA-v3.txt. Companion to diagnostics/inspect_v3_wheel_load_
# comparison_figure.py (rear axle, damper vs static-split) -- that script
# explicitly skipped the front axle because log_dms_dam_fr is corrupted
# for the whole session; this one is what that reconstruction now makes
# possible. FR is now reconstructed against the SESSION-CORRECTED axle-
# total model (modules.wheel_loads.estimate_session_corrected_axle_
# totals), not the plain static one -- the ground-truth diagnostic
# (thesis_notes.md "Reconstruction ground-truth check, lap 8" and
# "Closing the reconstruction's aero gap") found the static model
# understates the true axle total by ~25%, which the earlier version of
# this figure's own sustained-negative-FR observation was a direct
# symptom of. The plain static-split estimate is still shown for
# before/after contrast. Read-only, no config/production changes.

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core import plot_style as ps
from core.figure_render import save_png
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state, estimate_lateral_forces,
    estimate_vertical_loads,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, estimate_session_corrected_axle_totals, reconstruct_missing_corner,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_v3"


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    if state is None or car_data is None:
        print("cannot run -- missing state or car_data.json")
        return

    forces = estimate_lateral_forces(state, params)
    fz_static = estimate_vertical_loads(state, forces, params)
    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    session_corrected = estimate_session_corrected_axle_totals(state, damper_result, params)
    print(f"session-corrected model: mass_kg_session={session_corrected['mass_kg_session']:.1f} kg, "
          f"c_session={session_corrected['c_session_N_per_mps2']:.4f} N/(m/s)^2, "
          f"aero_front_fraction={session_corrected['aero_front_fraction']:.2f}")
    reconstructed = reconstruct_missing_corner(
        damper_result, {"fz_f_N": session_corrected["fz_f_N"], "fz_r_N": session_corrected["fz_r_N"]})

    if not reconstructed["fr"]["reconstructable"].any():
        print("FR is never reconstructable this session (FL also invalid?) -- cannot build this figure")
        return
    print(f"FR reconstructable fraction: {reconstructed['fr']['reconstructable'].mean()*100:.1f}%")

    corners = [c for c in data["corners"] if c.get("stable_corner_id") is not None]
    valid_laps = {lap["lap_number"] for lap in data["laps"] if lap["is_valid_for_analysis"]}
    candidates = [c for c in corners if c.get("lap_number") in valid_laps]
    if not candidates:
        print("no valid-lap corner instances found")
        return

    def bracket_times(c):
        segs = c["segments"]
        return segs["entry_1_brake"][0], segs["exit_5"][1]

    def median_ay(c):
        # Same robust-selection convention as inspect_v3_wheel_load_
        # comparison_figure.py -- median, not max, of |ay|, restricted to
        # a physically plausible 5-22 m/s^2 band to reject spike artifacts.
        t0, t1 = bracket_times(c)
        lo = np.searchsorted(state["time"], t0)
        hi = np.searchsorted(state["time"], t1)
        if hi <= lo:
            return -1.0
        med = float(np.median(np.abs(state["ay_mps2"][lo:hi])))
        return med if 5.0 <= med <= 22.0 else -1.0

    plausible = [c for c in candidates if median_ay(c) > 0]
    chosen = max(plausible or candidates, key=median_ay)
    c_start, c_end = bracket_times(chosen)
    lo = np.searchsorted(state["time"], c_start)
    hi = np.searchsorted(state["time"], c_end)
    sl = slice(max(lo - 20, 0), min(hi + 20, len(state["time"])))
    t = state["time"][sl]
    t0 = t - t[0]

    fig, axes = plt.subplots(2, 1, figsize=(ps.PRINT_WIDTH_CM / 2.54, 14.0 / 2.54),
                              constrained_layout=True, sharex=True)
    theme = ps.PRINT
    fig.patch.set_facecolor(theme["bg"])

    ax = axes[0]
    ax.set_facecolor(theme["bg"])
    ax.plot(t0, damper_result["fl"]["fz_N"][sl], color=ps.LAP_PALETTE[0], linewidth=1.3,
            label="Measured (Level 4)")
    ax.set_ylabel("FL Fz (N)")

    ax = axes[1]
    ax.set_facecolor(theme["bg"])
    ax.plot(t0, fz_static["fz_fr_N"][sl], color=theme["text_muted"], linewidth=1.2,
            linestyle="--", label="Static-split estimate (Level 1)")
    ax.plot(t0, reconstructed["fr"]["fz_N"][sl], color=ps.LAP_PALETTE[1], linewidth=1.3,
            label="Reconstructed (session-corrected axle-total - measured FL)")
    ax.set_ylabel("FR Fz (N)")
    ax.set_xlabel("Time into window (s)")

    for ax in axes:
        ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.85, framealpha=0.85)
        leg.get_frame().set_facecolor(theme["bg"])

    stable_id = chosen.get("stable_corner_id")
    lap_number = chosen.get("lap_number")
    fig.suptitle(f"C{stable_id}, lap {lap_number} -- front axle Fz, measured FL vs reconstructed FR\n"
                 f"(GT3_PRC_MLA-v3, median |ay|={median_ay(chosen):.1f} m/s^2)",
                 color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"wheel_load_reconstruction_C{stable_id}_lap{lap_number}.png")
    save_png(fig, out_path)
    print(f"saved {out_path}")
    print(f"corner stable_id={stable_id} lap={lap_number} median|ay|={median_ay(chosen):.2f} m/s^2, "
          f"window {c_start:.2f}-{c_end:.2f}s")


if __name__ == "__main__":
    main()
