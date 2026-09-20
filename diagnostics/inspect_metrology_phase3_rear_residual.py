# Metrology overnight package, Phase 3: rear axle-total residual
# decomposition. Read-only, no config/production file touched, no fit of
# any kind -- pure post-hoc regression against already-computed damper-
# derived Fz (modules.wheel_loads, Segers-anchored) and the outing's own
# real static weighing (Deepening Phase 1). No expensive EKF refit is
# needed here (unlike Phase 1's sensitivity sweep): CS/stability/beta are
# not touched, so this script runs in well under a minute.
#
# MOTIVATION: Deepening Phase 2 (FR gauge decoding fix) put all four v3
# corners on real damper measurements for the first time, exposing a
# straight-line residual against the outing's own weighing (thesis_notes.
# md "Damper package"/"Showcase re-rendered": FL +2.1%, FR +2.9%, RL
# +11.5%, RR +22.8%). PLAN.md BACKLOG item B names three still-open
# candidate mechanisms for a rear-axle-total gap of this kind: the aero
# front/rear split placeholder, geometric transfer estimates (roll-centre/
# unsprung-height model), and the motion-ratio table's own calibration in
# whatever travel region a corner actually operates in.
#
# A STRUCTURAL POINT, established by reading modules/wheel_loads.py
# directly before running anything (not assumed): estimate_wheel_loads_
# from_dampers's own arb_N/unsprung_transfer_N/geometric_transfer_N terms
# are built with SIDE_SIGN flipped between a corner and its axle mate, at
# otherwise-equal magnitude -- so for any axle, left_term + right_term = 0
# identically, by construction (verified empirically below, not just
# argued). An axle's L+R TOTAL can therefore never carry a residual from
# these transfer terms, at ANY ay: a geometric/roll-centre modelling error
# could only ever bias the L/R SPLIT, never the total. This directly rules
# out "geometric transfer estimates" as an explanation for a TOTAL residual
# before a single regression is run -- the decomposition below tests this
# prediction rather than assuming it, then discriminates the remaining two
# candidates (real aero vs a corner-specific constant/calibration offset)
# via each one's own distinct signature: aero scales with v^2 and is zero
# at v=0; a motion-ratio-region offset is a real constant, present even at
# v=0/ay=0, and specific to one corner rather than shared by its axle mate.

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state, estimate_vertical_loads,
    estimate_lateral_forces,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle

G = 9.81
PLOTS_DIR = Path("diagnostics/plots_metrology")

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

SESSIONS = {
    "v3": {
        "file": V3_FILE,
        "car_setup": {"corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
                      "corner_weight_rl": 396.2, "corner_weight_rr": 386.5},
    },
    "dubai": {
        "file": DUBAI_FILE,
        "car_setup": {"corner_weight_fl": 302.6, "corner_weight_fr": 301.1,
                      "corner_weight_rl": 392.8, "corner_weight_rr": 389.4},
    },
}


def load_session(name):
    sess = SESSIONS[name]
    params = load_parameters()
    setup_data = {"car": sess["car_setup"]}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    effective_params = apply_resolved_vehicle(params, resolved)
    car_data = load_car_data()
    data = parse_csv(sess["file"])
    state = prepare_vehicle_state(data["channels"], effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    static_fz = estimate_vertical_loads(state, forces, effective_params,
                                         channels=data["channels"], car_data=car_data)
    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], effective_params, car_data)
    static_fallback_fz = {"fl": static_fz["fz_fl_N"], "fr": static_fz["fz_fr_N"],
                           "rl": static_fz["fz_rl_N"], "rr": static_fz["fz_rr_N"]}
    combined = combine_with_static_fallback(damper_result, static_fallback_fz)

    corner_weight_N = {c: effective_params["vehicle"]["corner_weights"][k] * G
                        for c, k in (("fl", "FL_kg"), ("fr", "FR_kg"), ("rl", "RL_kg"), ("rr", "RR_kg"))}

    return {
        "state": state, "damper_result": damper_result, "combined": combined,
        "corner_weight_N": corner_weight_N, "params": effective_params,
    }


def _valid_frac(damper_result):
    return {c: float(damper_result[c]["valid"].mean()) for c in CORNERS}


def _linreg(X_cols, y, mask):
    X = np.column_stack([np.ones(int(mask.sum()))] + [col[mask] for col in X_cols])
    yv = y[mask]
    coeffs, residuals, rank, sv = np.linalg.lstsq(X, yv, rcond=None)
    y_pred = X @ coeffs
    ss_res = float(np.sum((yv - y_pred) ** 2))
    ss_tot = float(np.sum((yv - np.mean(yv)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return coeffs, r2


def analyse_session(name):
    print(f"\n{'='*70}\n{name.upper()}\n{'='*70}")
    sess = load_session(name)
    state, damper_result, combined = sess["state"], sess["damper_result"], sess["combined"]
    corner_weight_N = sess["corner_weight_N"]
    v, ax, ay = state["v_mps"], state["ax_mps2"], state["ay_mps2"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    non_kerb = ~kerb_mask if kerb_mask is not None else np.ones_like(moving, dtype=bool)

    vf = _valid_frac(damper_result)
    print(f"damper valid fraction: {vf}")

    straight = moving & non_kerb & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    print(f"straight-line samples: {int(straight.sum())} / {len(v)}")

    # --- (a) per-corner shares at straight-line, near-zero ay/ax ---
    print("\n--- (a) per-corner straight-line residual (measured - weighing) ---")
    delta_kg = {}
    for c in CORNERS:
        if vf[c] < 0.5:
            print(f"  {c}: SKIPPED, dead/mostly-invalid channel this session (valid_frac={vf[c]:.2f})")
            continue
        measured_kg = float(np.mean(combined[c]["fz_N"][straight])) / G
        weighing_kg = corner_weight_N[c] / G
        d = measured_kg - weighing_kg
        delta_kg[c] = d
        print(f"  {c}: weighing={weighing_kg:.1f} kg, measured={measured_kg:.1f} kg, "
              f"delta={d:+.1f} kg ({d/weighing_kg*100:+.1f}%)")
    if "rl" in delta_kg and "rr" in delta_kg and (delta_kg["rl"] + delta_kg["rr"]) != 0:
        rear_total_delta = delta_kg["rl"] + delta_kg["rr"]
        print(f"  rear total delta={rear_total_delta:+.1f} kg -- RL share="
              f"{delta_kg['rl']/rear_total_delta*100:.0f}%, RR share={delta_kg['rr']/rear_total_delta*100:.0f}%")

    # --- structural check: do geometric/unsprung transfer terms cancel in the axle total? ---
    print("\n--- structural check: L+R transfer-term cancellation (rules out 'geometric transfer' for the TOTAL) ---")
    for axle, (lc, rc) in (("front", ("fl", "fr")), ("rear", ("rl", "rr"))):
        both_valid = damper_result[lc]["valid"] & damper_result[rc]["valid"] & moving
        if both_valid.sum() == 0:
            print(f"  {axle}: no samples with both corners valid, skipped")
            continue
        geo_sum = damper_result[lc]["geometric_transfer_N"][both_valid] + damper_result[rc]["geometric_transfer_N"][both_valid]
        uns_sum = damper_result[lc]["unsprung_transfer_N"][both_valid] + damper_result[rc]["unsprung_transfer_N"][both_valid]
        print(f"  {axle}: mean(geometric_L+geometric_R)={np.nanmean(geo_sum):+.2f} N "
              f"(vs mean|term| {np.nanmean(np.abs(damper_result[lc]['geometric_transfer_N'][both_valid])):.1f} N), "
              f"mean(unsprung_L+unsprung_R)={np.nanmean(uns_sum):+.2f} N "
              f"(vs mean|term| {np.nanmean(np.abs(damper_result[lc]['unsprung_transfer_N'][both_valid])):.1f} N)")

    # --- (b) vs speed: bin the straight (ay~0) population by speed, fit residual vs v^2 ---
    print("\n--- (b) straight-line residual vs speed (aero signature = grows with v^2, ~0 at v=0) ---")
    n_bins = 6
    v_straight = v[straight]
    if len(v_straight) >= n_bins * 20:
        edges = np.quantile(v_straight, np.linspace(0, 1, n_bins + 1))
        for c in CORNERS:
            if vf[c] < 0.5:
                continue
            fz = combined[c]["fz_N"][straight]
            row = []
            for i in range(n_bins):
                lo, hi = edges[i], edges[i + 1]
                sel = (v_straight >= lo) & (v_straight <= hi)
                if sel.sum() < 5:
                    continue
                v_mid_kmh = float(np.mean(v_straight[sel])) * 3.6
                resid_kg = (float(np.mean(fz[sel])) - corner_weight_N[c]) / G
                row.append((v_mid_kmh, resid_kg))
            print(f"  {c}: " + ", ".join(f"{vk:.0f}km/h->{rk:+.1f}kg" for vk, rk in row))
        # linear-in-v^2 fit per corner on the full straight population (not just bin means)
        print("  fit residual_N = a + b*v^2 on straight population (b in N per (m/s)^2, a in N):")
        fit_by_corner = {}
        for c in CORNERS:
            if vf[c] < 0.5:
                continue
            fz = combined[c]["fz_N"][straight]
            resid_N = fz - corner_weight_N[c]
            coeffs, r2 = _linreg([v_straight ** 2], resid_N, np.ones_like(resid_N, dtype=bool))
            fit_by_corner[c] = (coeffs[0], coeffs[1], r2)
            print(f"    {c}: a={coeffs[0]:+7.1f} N, b={coeffs[1]:+8.4f} N/(m/s)^2, R^2={r2:.3f}")

        if all(c in fit_by_corner for c in ("fl", "fr", "rl", "rr")):
            front_b = fit_by_corner["fl"][1] + fit_by_corner["fr"][1]
            rear_b = fit_by_corner["rl"][1] + fit_by_corner["rr"][1]
            total_b = front_b + rear_b
            if total_b != 0:
                print(f"  IMPLIED real aero front/rear split (from b-slopes): "
                      f"front={front_b/total_b*100:.0f}%, rear={rear_b/total_b*100:.0f}% "
                      f"-- vs config wheel_loads.aero_front_fraction=0.40 (40% front/60% rear placeholder)")

        # figure: residual (kg) vs speed, per corner, with the fitted v^2 curve
        PLOTS_DIR.mkdir(parents=True, exist_ok=True)
        fig, ax_fig = plt.subplots(figsize=(8, 5))
        v_plot = np.linspace(0, float(np.max(v_straight)), 100)
        for c, colour in (("fl", "tab:blue"), ("fr", "tab:orange"), ("rl", "tab:green"), ("rr", "tab:red")):
            if c not in fit_by_corner:
                continue
            fz = combined[c]["fz_N"][straight]
            resid_kg = (fz - corner_weight_N[c]) / G
            ax_fig.scatter(v_straight * 3.6, resid_kg, s=4, alpha=0.15, color=colour)
            a_c, b_c, r2_c = fit_by_corner[c]
            ax_fig.plot(v_plot * 3.6, (a_c + b_c * v_plot ** 2) / G, color=colour,
                        label=f"{c.upper()} (R2={r2_c:.2f})", linewidth=2)
        ax_fig.axhline(0, color="grey", linewidth=0.8)
        ax_fig.set_xlabel("speed (km/h)")
        ax_fig.set_ylabel("residual: measured - weighing (kg)")
        ax_fig.set_title(f"{name}: straight-line ($|a_x|,|a_y|$<0.5) Fz residual vs speed")
        ax_fig.legend()
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"{name}_residual_vs_speed.png", dpi=120)
        plt.close(fig)
        print(f"  figure saved: {PLOTS_DIR / f'{name}_residual_vs_speed.png'}")
    else:
        print("  not enough straight-line samples for a speed-binned fit")

    # --- (c) vs lateral acceleration: axle-total residual after removing mass/aero/long-transfer ---
    print("\n--- (c) axle-total residual vs |ay|, after subtracting the fitted v^2/ax terms ---")
    wide = moving & non_kerb & (np.abs(ax) < 2.0)
    for axle, (lc, rc) in (("front", ("fl", "fr")), ("rear", ("rl", "rr"))):
        both_valid = damper_result[lc]["valid"] & damper_result[rc]["valid"] & wide
        if both_valid.sum() < 50:
            print(f"  {axle}: insufficient jointly-valid samples ({int(both_valid.sum())}), skipped")
            continue
        total_fz = combined[lc]["fz_N"] + combined[rc]["fz_N"]
        total_weighing_N = corner_weight_N[lc] + corner_weight_N[rc]
        resid_N = total_fz - total_weighing_N
        coeffs, r2 = _linreg([v[both_valid] ** 2, ax[both_valid]], resid_N[both_valid],
                              np.ones(int(both_valid.sum()), dtype=bool))
        # residual after removing the fitted v^2/ax model, regressed against |ay|
        X = np.column_stack([np.ones(int(both_valid.sum())), v[both_valid] ** 2, ax[both_valid]])
        model_resid = resid_N[both_valid] - X @ coeffs
        ay_abs = np.abs(ay[both_valid])
        if np.std(ay_abs) > 1e-6:
            corr = float(np.corrcoef(model_resid, ay_abs)[0, 1])
        else:
            corr = float("nan")
        coeffs2, r2b = _linreg([ay_abs], model_resid, np.ones_like(model_resid, dtype=bool))
        print(f"  {axle}: v^2/ax fit R^2={r2:.3f}; residual-after-fit vs |ay|: corr={corr:+.3f}, "
              f"slope={coeffs2[1]:+.2f} N per (m/s^2), intercept={coeffs2[0]:+.1f} N")

    # --- (d) lowest-available-speed proxy for 'static' ---
    print("\n--- (d) lowest-available-speed samples vs weighing (best-available static proxy) ---")
    moving_only = moving & non_kerb
    if moving_only.sum() > 0:
        v_moving = v[moving_only]
        low_thresh = np.quantile(v_moving, 0.02)
        low_mask_idx = np.where(moving_only)[0][v[moving_only] <= low_thresh]
        low_mask = np.zeros_like(moving, dtype=bool)
        low_mask[low_mask_idx] = True
        v_lo_kmh = float(np.mean(v[low_mask])) * 3.6
        v_lo_max_kmh = float(np.max(v[low_mask])) * 3.6
        print(f"  lowest 2% of moving-speed samples: mean {v_lo_kmh:.0f} km/h, max {v_lo_max_kmh:.0f} km/h "
              f"(n={int(low_mask.sum())}) -- NOT a true stationary/pit sample, best available in this telemetry")
        for c in CORNERS:
            if vf[c] < 0.5:
                continue
            measured_kg = float(np.mean(combined[c]["fz_N"][low_mask])) / G
            weighing_kg = corner_weight_N[c] / G
            d = measured_kg - weighing_kg
            print(f"  {c}: weighing={weighing_kg:.1f} kg, measured(low-speed)={measured_kg:.1f} kg, "
                  f"delta={d:+.1f} kg ({d/weighing_kg*100:+.1f}%)")

    return sess


def main():
    for name in ("v3", "dubai"):
        analyse_session(name)


if __name__ == "__main__":
    main()
