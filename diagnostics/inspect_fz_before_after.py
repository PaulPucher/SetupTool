# Fz-integration Phase 1 (finish), 2026-09-03: static-vs-measured
# before/after, v3 and Dubai. Read-only, no config/production changes.
#
# SCOPE NOTE, load-bearing: this does NOT compare CS_ratio/stability/
# corner-verdict output -- those are proven independent of stability_
# estimation.vertical_load_source (diagnostics/inspect_vertical_load_
# source_cs_independence.py's finding, recorded in thesis_notes.md and
# deleted the same turn): estimate_cornering_stiffness/estimate_yaw_
# moment_stability never receive fz as an argument. What DOES change is
# the fz_*_N/fy_*_norm_N values themselves -- this script reports that
# comparison: per-corner source share (damper/reconstructed/static_
# fallback) and static-vs-measured trace figures at named corners.

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core import plot_style as ps
from core.figure_render import save_png
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_lateral_forces, estimate_vertical_loads,
)
from modules.wheel_loads import CORNERS

OUT_DIR = "diagnostics/plots_fz_integration"


def _source_shares(source_arr, mask):
    m = source_arr[mask]
    n = len(m)
    if n == 0:
        return {"damper": float("nan"), "reconstructed": float("nan"), "static_fallback": float("nan"), "n": 0}
    return {
        "damper": float((m == "damper").mean()),
        "reconstructed": float((m == "reconstructed").mean()),
        "static_fallback": float((m == "static_fallback").mean()),
        "n": int(n),
    }


def _bracket_times(c):
    segs = c["segments"]
    return segs["entry_1_brake"][0], segs["exit_5"][1]


def _median_ay_plausible(state, c):
    t0, t1 = _bracket_times(c)
    lo = np.searchsorted(state["time"], t0)
    hi = np.searchsorted(state["time"], t1)
    if hi <= lo:
        return -1.0
    med = float(np.median(np.abs(state["ay_mps2"][lo:hi])))
    return med if 5.0 <= med <= 22.0 else -1.0


def _plot_corner(state, fz_static, fz_measured, source_per_sample, chosen, session_label, out_dir):
    t0_c, t1_c = _bracket_times(chosen)
    lo = np.searchsorted(state["time"], t0_c)
    hi = np.searchsorted(state["time"], t1_c)
    sl = slice(max(lo - 20, 0), min(hi + 20, len(state["time"])))
    t = state["time"][sl]
    tt = t - t[0]

    fig, axes = plt.subplots(2, 1, figsize=(ps.PRINT_WIDTH_CM / 2.54, 14.0 / 2.54),
                              constrained_layout=True, sharex=True)
    theme = ps.PRINT
    fig.patch.set_facecolor(theme["bg"])

    for ax, axle_key, title in ((axes[0], "fz_f_N", "Front axle Fz (N)"), (axes[1], "fz_r_N", "Rear axle Fz (N)")):
        ax.set_facecolor(theme["bg"])
        ax.plot(tt, fz_static[axle_key][sl], color=theme["text_muted"], linewidth=1.2,
                linestyle="--", label="Static-split (Level 1)")
        ax.plot(tt, fz_measured[axle_key][sl], color=ps.LAP_PALETTE[0], linewidth=1.3,
                label="Measured cascade (damper/reconstructed/static)")
        ax.set_ylabel(title)
        ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.85, framealpha=0.85)
        leg.get_frame().set_facecolor(theme["bg"])
    axes[1].set_xlabel("Time into window (s)")

    corner_mask = np.zeros_like(state["time"], dtype=bool)
    corner_mask[sl] = True
    shares = {c: _source_shares(source_per_sample[c], corner_mask) for c in CORNERS}
    # Only report corners with a non-damper share -- on both real sessions
    # three of four corners are 100% damper throughout (see whole-session
    # table), so naming those explicitly here only pads the subtitle.
    non_damper = [c for c in CORNERS if shares[c]["n"] > 0
                  and (shares[c]["reconstructed"] > 0 or shares[c]["static_fallback"] > 0)]
    share_txt = ("all 4 corners damper-valid" if not non_damper else ", ".join(
        f"{c.upper()} recon={shares[c]['reconstructed']*100:.0f}% static={shares[c]['static_fallback']*100:.0f}%"
        for c in non_damper
    ))

    stable_id = chosen.get("stable_corner_id")
    lap_number = chosen.get("lap_number")
    fig.suptitle(f"{session_label} C{stable_id}, lap {lap_number} -- Fz static vs measured\n"
                 f"median|ay|={_median_ay_plausible(state, chosen):.1f} m/s^2 -- {share_txt}",
                 color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT * 0.9)

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"fz_before_after_{session_label}_C{stable_id}_lap{lap_number}.png")
    save_png(fig, out_path)
    return out_path, shares


def run_for_session(raw_file, session_label, canonical_corner_id, n_extra=2):
    print(f"\n{'='*70}\n{session_label}: {raw_file}\n{'='*70}")
    data = parse_csv(raw_file)
    params = load_parameters()
    car_data = load_car_data()
    if car_data is None:
        print("car_data.json not available -- cannot run")
        return

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        print("prepare_vehicle_state returned None")
        return

    forces = estimate_lateral_forces(state, params)

    params_static = dict(params)
    params_static["stability_estimation"] = dict(params["stability_estimation"])
    params_static["stability_estimation"]["vertical_load_source"] = "static"
    fz_static = estimate_vertical_loads(state, forces, params_static)

    params_measured = dict(params)
    params_measured["stability_estimation"] = dict(params["stability_estimation"])
    params_measured["stability_estimation"]["vertical_load_source"] = "measured"
    fz_measured = estimate_vertical_loads(state, forces, params_measured,
                                           channels=data["channels"], car_data=car_data)
    print(f"vertical_load_source_used: static={fz_static['vertical_load_source_used']!r}, "
          f"measured={fz_measured['vertical_load_source_used']!r}")

    source_per_sample = fz_measured["vertical_load_source_per_sample"]
    if source_per_sample is None:
        print("no damper-valid samples anywhere this session -- measured cascade never differs from static "
              "(provable-by-construction identity, config's own _comment_vertical_load_source)")
        return

    v = state["v_mps"]
    moving = v >= params["stability_estimation"]["moving_speed_min_mps"]
    valid_lap_numbers = {l["lap_number"] for l in data.get("laps", []) if l.get("is_valid_for_analysis")}
    t = state["time"]
    racing = np.zeros_like(t, dtype=bool)
    for lap in data.get("laps", []):
        if lap["lap_number"] in valid_lap_numbers:
            racing |= (t >= lap["start_time"]) & (t <= lap["end_time"])
    racing = racing & moving

    print("\n--- reconstructed share, per corner (source: damper / reconstructed / static_fallback) ---")
    for scope_name, mask in (("whole session, moving", moving), ("valid racing laps only", racing)):
        print(f"  [{scope_name}]")
        for c in CORNERS:
            s = _source_shares(source_per_sample[c], mask)
            if s["n"] == 0:
                print(f"    {c.upper()}: no samples in this scope")
                continue
            print(f"    {c.upper()}: damper={s['damper']*100:5.1f}%  reconstructed={s['reconstructed']*100:5.1f}%  "
                  f"static_fallback={s['static_fallback']*100:5.1f}%  (n={s['n']})")

    # --- corner figure selection: the named canonical corner + n_extra
    # more, auto-picked by highest plausible median|ay| among valid-lap
    # instances (same selection convention as diagnostics/inspect_v3_
    # wheel_load_comparison_figure.py / ..._reconstruction_figure.py).
    corners = [c for c in data.get("corners", []) if c.get("stable_corner_id") is not None
               and c.get("lap_number") in valid_lap_numbers]
    by_id = {}
    for c in corners:
        cid = c["stable_corner_id"]
        med = _median_ay_plausible(state, c)
        if med <= 0:
            continue
        if cid not in by_id or med > by_id[cid][1]:
            by_id[cid] = (c, med)

    chosen_ids = []
    if canonical_corner_id in by_id:
        chosen_ids.append(canonical_corner_id)
    else:
        print(f"\nWARNING: canonical corner C{canonical_corner_id} has no plausible valid-lap instance this session")

    ranked = sorted((cid for cid in by_id if cid != canonical_corner_id),
                     key=lambda cid: by_id[cid][1], reverse=True)
    chosen_ids += ranked[:n_extra]

    print(f"\n--- corner figures: {['C'+str(i) for i in chosen_ids]} ---")
    out_dir = os.path.join(OUT_DIR, session_label)
    for cid in chosen_ids:
        chosen, med = by_id[cid]
        out_path, shares = _plot_corner(state, fz_static, fz_measured, source_per_sample, chosen, session_label, out_dir)
        print(f"  C{cid} lap{chosen['lap_number']} median|ay|={med:.1f} m/s^2 -> {out_path}")


if __name__ == "__main__":
    run_for_session("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt", "dubai", canonical_corner_id=4)
    run_for_session("GT3_PRC_MLA-v3.txt", "v3", canonical_corner_id=12)
