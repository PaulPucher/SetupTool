# Reconstruction ground-truth diagnostic (read-only, no config/production
# changes). Both RL and RR are 100% damper-valid all session on
# GT3_PRC_MLA-v3.txt (Phase 2 finding) -- this lets us drop one of them
# ON PURPOSE, reconstruct it with the EXACT SAME method already shipped
# (modules.wheel_loads.reconstruct_missing_corner), and compare the
# reconstruction against its own real measurement, a genuine ground-truth
# check that the real FR reconstruction (corrupted gauge, no ground truth
# available) can never have.
#
# SCENARIO, matching the real file's own condition: FR is ALREADY invalid
# for the whole session (real corruption, not simulated) -- so this test
# additionally drops RL (using real RR + FL + FR-as-already-reconstructed
# as the available signal set) and separately drops RR (using real RL).
# reconstruct_missing_corner itself only ever needs the axle-total model
# (fz_r_N, independent of ay/roll) and the real axle-mate (RR when
# reconstructing RL, RL when reconstructing RR) -- FL/FR are enumerated
# here only because they are what is ACTUALLY available in this file's
# real condition, not because the rear-axle reconstruction formula uses
# them (it does not; the method is per-axle, front and rear independent).
#
# FILTER INTERPRETATION (stated, not assumed silently): "0.1s low-pass"
# is applied as cutoff_hz=1/0.1=10Hz via the SAME Butterworth low-pass
# already used elsewhere in this project (modules.stability_analysis.
# _butterworth_lowpass, zero-phase filtfilt, order 4) -- applied ONLY to
# the axle-mate measurement (the real, noisy sensor input the formula
# uses), NOT to the axle-total model (a computed ax/ay/v-based quantity,
# not a raw gauge reading).
#
# RE-RUN with the SESSION-CORRECTED axle-total model (modules.wheel_
# loads.estimate_session_corrected_axle_totals, "Closing the
# reconstruction's aero gap" work order) -- the first run of this exact
# script (thesis_notes.md "Reconstruction ground-truth check, lap 8")
# found a ~25% systematic axle-total-model bias using the STATIC (config
# mass + Cl=0) axle-total model; this run uses the session-derived
# mass+aero correction instead, to check whether that bias closes.

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core import plot_style as ps
from core.figure_render import save_png
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state, _butterworth_lowpass,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, estimate_session_corrected_axle_totals,
    reconstruct_missing_corner, CORNERS,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_v3"
LOWPASS_CUTOFF_HZ = 10.0  # 1 / 0.1s, see module docstring


def _drop_and_reconstruct(damper_result, fz_axle_totals, corner_to_drop, mate_fz_override=None):
    """Return the reconstructed fz_N array for corner_to_drop, having
    forced its own 'valid' flag to False. mate_fz_override, if given,
    replaces the axle-mate's fz_N (used for the filtered-input variant)
    without touching its 'valid' flag (still real/valid).
    """
    dr = {c: dict(v) for c, v in damper_result.items()}
    dr[corner_to_drop]["valid"] = np.zeros_like(dr[corner_to_drop]["valid"])
    if mate_fz_override is not None:
        from modules.wheel_loads import AXLE_MATE
        mate = AXLE_MATE[corner_to_drop]
        dr[mate] = dict(dr[mate])
        dr[mate]["fz_N"] = mate_fz_override
    reconstructed = reconstruct_missing_corner(dr, fz_axle_totals)
    return reconstructed[corner_to_drop]["fz_N"]


def _report(label, measured, reconstructed, corner_mask):
    error = reconstructed - measured
    mean_err_N = float(np.mean(error))
    std_err_N = float(np.std(error))
    mean_measured_N = float(np.mean(measured))
    pct_err = mean_err_N / mean_measured_N * 100.0 if mean_measured_N != 0 else float("nan")

    corner_err = error[corner_mask]
    straight_err = error[~corner_mask]
    print(f"\n--- {label} ---")
    print(f"  n={len(measured)}, mean measured={mean_measured_N:.1f} N")
    print(f"  mean error={mean_err_N:+.1f} N ({pct_err:+.2f}%), std error={std_err_N:.1f} N")
    print(f"  corner samples (n={int(corner_mask.sum())}): mean error={float(np.mean(corner_err)):+.1f} N, "
          f"std={float(np.std(corner_err)):.1f} N")
    print(f"  straight samples (n={int((~corner_mask).sum())}): mean error={float(np.mean(straight_err)):+.1f} N, "
          f"std={float(np.std(straight_err)):.1f} N")
    return {"mean_err_N": mean_err_N, "pct_err": pct_err, "std_err_N": std_err_N}


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    if state is None or car_data is None:
        print("cannot run -- missing state or car_data.json")
        return

    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    session_corrected = estimate_session_corrected_axle_totals(state, damper_result, params)
    fz_axle_totals = {"fz_f_N": session_corrected["fz_f_N"], "fz_r_N": session_corrected["fz_r_N"]}
    print(f"session-corrected axle-total model: mass_kg_session={session_corrected['mass_kg_session']:.1f} kg "
          f"(config was {params['vehicle']['mass_kg']:.1f} kg), "
          f"c_session={session_corrected['c_session_N_per_mps2']:.4f} N/(m/s)^2, "
          f"aero_front_fraction={session_corrected['aero_front_fraction']:.2f}")

    lap8 = next((lap for lap in data["laps"] if lap["lap_number"] == 8), None)
    if lap8 is None:
        print("lap 8 not found")
        return
    t = state["time"]
    lo = np.searchsorted(t, lap8["start_time"])
    hi = np.searchsorted(t, lap8["end_time"])
    sl = slice(lo, hi)
    sample_rate_hz = state["sample_rate_hz"]

    ay = state["ay_mps2"]
    ax = state["ax_mps2"]
    corner_mask_full = (np.abs(ay) > 3.0) | (np.abs(ax) > 3.0)  # simple, stated corner/braking-activity proxy

    print("=" * 78)
    print("RECONSTRUCTION GROUND-TRUTH DIAGNOSTIC -- lap 8, RAW inputs")
    print("=" * 78)

    results_raw = {}
    figs_data = {}
    for drop, label in (("rl", "RL (from RR, FL, FR-static)"), ("rr", "RR (from RL, FL, FR-static)")):
        recon_full = _drop_and_reconstruct(damper_result, fz_axle_totals, drop)
        measured = damper_result[drop]["fz_N"][sl]
        recon = recon_full[sl]
        stats = _report(label, measured, recon, corner_mask_full[sl])
        results_raw[drop] = stats
        figs_data[drop] = {"measured": measured, "recon_raw": recon}

    print("\n" + "=" * 78)
    print("RECONSTRUCTION GROUND-TRUTH DIAGNOSTIC -- lap 8, 0.1s LOW-PASS ON THE MATE INPUT")
    print("=" * 78)

    results_filt = {}
    for drop, label in (("rl", "RL (mate RR filtered)"), ("rr", "RR (mate RL filtered)")):
        from modules.wheel_loads import AXLE_MATE
        mate = AXLE_MATE[drop]
        mate_fz_filt = _butterworth_lowpass(damper_result[mate]["fz_N"], LOWPASS_CUTOFF_HZ, sample_rate_hz)
        recon_full = _drop_and_reconstruct(damper_result, fz_axle_totals, drop, mate_fz_override=mate_fz_filt)
        measured = damper_result[drop]["fz_N"][sl]
        recon = recon_full[sl]
        stats = _report(label, measured, recon, corner_mask_full[sl])
        results_filt[drop] = stats
        figs_data[drop]["recon_filt"] = recon

    # --- overlay figure, established PRINT style, one panel per corner ----
    t_lap = t[sl] - t[sl][0]
    fig, axes = plt.subplots(2, 1, figsize=(ps.PRINT_WIDTH_CM / 2.54, 14.0 / 2.54),
                              constrained_layout=True, sharex=True)
    theme = ps.PRINT
    fig.patch.set_facecolor(theme["bg"])
    for ax_, drop, label in ((axes[0], "rl", "RL"), (axes[1], "rr", "RR")):
        ax_.set_facecolor(theme["bg"])
        d = figs_data[drop]
        ax_.plot(t_lap, d["measured"] / 1000.0, color=theme["text"], linewidth=1.1, label="Measured (ground truth)")
        ax_.plot(t_lap, d["recon_raw"] / 1000.0, color=ps.LAP_PALETTE[1], linewidth=1.0, alpha=0.8,
                 label="Reconstructed (raw input)")
        ax_.plot(t_lap, d["recon_filt"] / 1000.0, color=ps.LAP_PALETTE[2], linewidth=1.1, linestyle="--",
                 label="Reconstructed (0.1s low-pass on mate)")
        ax_.set_ylabel(f"{label} Fz (kN)")
        ax_.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
        ax_.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
        for spine in ax_.spines.values():
            spine.set_color(theme["text_muted"])
        leg = ax_.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.75, framealpha=0.85, ncol=1)
        leg.get_frame().set_facecolor(theme["bg"])
    axes[1].set_xlabel("Time into lap 8 (s)")
    fig.suptitle("Reconstruction ground-truth check, lap 8, GT3_PRC_MLA-v3\n"
                 "(RL/RR dropped in turn, reconstructed, compared to their own real measurement)",
                 color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, "wheel_load_reconstruction_ground_truth_lap8.png")
    save_png(fig, out_path)
    print(f"\nsaved {out_path}")

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    for drop in ("rl", "rr"):
        r, f = results_raw[drop], results_filt[drop]
        print(f"  {drop.upper()}: raw mean_err={r['mean_err_N']:+.1f}N ({r['pct_err']:+.2f}%) std={r['std_err_N']:.1f}N "
              f"-> filtered mean_err={f['mean_err_N']:+.1f}N ({f['pct_err']:+.2f}%) std={f['std_err_N']:.1f}N")


if __name__ == "__main__":
    main()
