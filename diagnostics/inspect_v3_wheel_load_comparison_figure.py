# Damper package, Phase 3: comparison figure, static-split estimate vs
# damper-derived wheel load, GT3_PRC_MLA-v3.txt. Read-only, no config/
# production changes. Reuses core/plot_style.py's PRINT theme constants
# and core/figure_render.py's save_png (the same PNG-export mechanics the
# app's own "Export figure" buttons use) rather than inventing new style
# constants -- the panel composition itself is new (no existing renderer
# draws Fz vs track position), built directly with matplotlib.
#
# AXLE CHOICE: rear (RL/RR), not front. log_dms_dam_fr is corrupted for
# the entire session (Phase 1/2 finding) -- a front-axle comparison would
# show one real (FL) and one permanently-fallback (FR) trace, which is
# not an illustrative "damper vs static" comparison. Both rear channels
# validate 100% of the session (Phase 2), making the rear pair the
# clearer choice.

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
from modules.wheel_loads import estimate_wheel_loads_from_dampers

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
        # Median, not max, of |ay| over the bracket -- a raw max is prone
        # to a single-sample spike (kerb strike, sensor glitch) picking an
        # unrepresentative "corner" for what is meant to be an
        # illustrative, clean comparison figure. A physically plausible
        # GT3 corner sustains roughly 1-2.5g (~10-25 m/s^2); candidates
        # outside that band are excluded as likely artifacts rather than
        # genuinely the most demanding corner.
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

    for ax, corner, label in ((axes[0], "rl", "RL"), (axes[1], "rr", "RR")):
        ax.set_facecolor(theme["bg"])
        static_key = f"fz_{corner}_N"
        ax.plot(t0, fz_static[static_key][sl], color=theme["text_muted"], linewidth=1.2,
                linestyle="--", label="Static-split estimate (Level 1)")
        ax.plot(t0, damper_result[corner]["fz_N"][sl], color=ps.LAP_PALETTE[0], linewidth=1.3,
                label="Damper-derived (Level 4)")
        ax.set_ylabel(f"{label} Fz (N)")
        ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.85, framealpha=0.85)
        leg.get_frame().set_facecolor(theme["bg"])

    axes[1].set_xlabel("Time into window (s)")
    stable_id = chosen.get("stable_corner_id")
    lap_number = chosen.get("lap_number")
    fig.suptitle(f"C{stable_id}, lap {lap_number} -- rear axle Fz, static-split vs damper-derived\n"
                 f"(GT3_PRC_MLA-v3, median |ay|={median_ay(chosen):.1f} m/s^2)",
                 color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"wheel_load_comparison_C{stable_id}_lap{lap_number}.png")
    save_png(fig, out_path)
    print(f"saved {out_path}")
    print(f"corner stable_id={stable_id} lap={lap_number} median|ay|={median_ay(chosen):.2f} m/s^2, "
          f"window {c_start:.2f}-{c_end:.2f}s")


if __name__ == "__main__":
    main()
