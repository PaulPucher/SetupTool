# Fz-integration Phase 2 gate resolution, v3 rear divergence dig (2026-
# 09-03, user instruction). Read-only, no config/production changes.
#
# Question: why does v3 rear's cross-check diverge (+20.76%, diagnostics/
# inspect_fz_mu_cross_check.py) while v3 front agrees within 1%, when
# neither axle's Fz rests on a reconstructed proxy (v3's own dead corner
# is FR, front -- rear is RL+RR, both 100% real damper data all session)?
#
# LEGITIMATE LOAD EFFECT signature: the cloud colour-orders by Fz (load
# and operating point genuinely co-vary -- the physical condition under
# which a load-normalised and a free-D fit are EXPECTED to read
# differently, not a contradiction), the mu fit's own residuals are
# roughly uniform across Fz terciles (it is not systematically better/
# worse at one load level than another), and the median/p25/p75 mu-curve
# band brackets the cloud plausibly.
# FIT ARTIFACT signature: residual RMS varies systematically across Fz
# terciles (the mu fit is buying a better score at one load level by
# doing worse at another -- exploiting noise/structure, not tracking a
# real load effect), or the p25/p75 band is implausible (inverted,
# absurdly wide, or misses the cloud).

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
    estimate_sideslip, estimate_slip_angles, estimate_lateral_forces, estimate_vertical_loads,
)
from modules.tyre_fit_auto import _base_mask, _fit_axle_pacejka, _fit_axle_pacejka_mu
from modules.tyre_model_pacejka import pacejka_lateral_force

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_fz_integration/v3"
AXLES = (("front", "FOCUS: control", "f"), ("rear", "FOCUS: divergence under investigation", "r"))


def _load_v3():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    state = prepare_vehicle_state(data["channels"], params)
    base_mask = _base_mask(state, data.get("laps", []))

    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)

    params_measured = dict(params)
    params_measured["stability_estimation"] = dict(params["stability_estimation"])
    params_measured["stability_estimation"]["vertical_load_source"] = "measured"
    fz = estimate_vertical_loads(state, forces, params_measured, channels=data["channels"], car_data=car_data)
    assert fz["vertical_load_source_used"] == "measured"
    return base_mask, slip_kin, forces, fz


def _analyse_axle(axle_name, axle_key, base_mask, slip_kin, forces, fz):
    alpha = slip_kin[f"alpha_{axle_key}_filt"]
    Fy = forces[f"Fy_{axle_key}_filt"]
    Fz = fz[f"fz_{axle_key}_N"]

    free_fit = _fit_axle_pacejka(alpha, Fy, base_mask)
    mu_fit = _fit_axle_pacejka_mu(alpha, Fy, Fz, base_mask)

    m2 = mu_fit["residual_mask"]  # base_mask & finite(alpha,Fy,Fz) -- the fit's own population
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]
    a2_deg = np.degrees(a2)

    corr_fz_abs_alpha = float(np.corrcoef(z2, np.abs(a2))[0, 1])
    corr_fz_abs_fy = float(np.corrcoef(z2, np.abs(f2))[0, 1])

    # Residuals of BOTH fits over the SAME population (m2), not each
    # fit's own slightly different mask -- a fair, directly comparable
    # residual-structure check.
    pred_freeD = pacejka_lateral_force(a2, free_fit["B"], free_fit["C"], free_fit["D"], free_fit["E"])
    resid_freeD = f2 - pred_freeD
    pred_mu = pacejka_lateral_force(a2, mu_fit["B"], mu_fit["C"], mu_fit["mu"] * z2, mu_fit["E"])
    resid_mu = f2 - pred_mu

    p33, p67 = np.percentile(z2, [33.333, 66.667])
    tercile = np.where(z2 <= p33, "low", np.where(z2 <= p67, "mid", "high"))
    tercile_rms = {}
    for band in ("low", "mid", "high"):
        bm = tercile == band
        tercile_rms[band] = {
            "n": int(bm.sum()),
            "fz_range_N": (float(z2[bm].min()), float(z2[bm].max())) if bm.any() else (float("nan"), float("nan")),
            "rms_resid_mu_N": float(np.sqrt(np.mean(resid_mu[bm] ** 2))) if bm.any() else float("nan"),
            "rms_resid_freeD_N": float(np.sqrt(np.mean(resid_freeD[bm] ** 2))) if bm.any() else float("nan"),
        }

    numbers = {
        "axle": axle_name,
        "n_fit_population": int(m2.sum()),
        "corr_fz_abs_alpha": corr_fz_abs_alpha,
        "corr_fz_abs_fy": corr_fz_abs_fy,
        "rms_resid_freeD_N_whole_pop": float(np.sqrt(np.mean(resid_freeD ** 2))),
        "rms_resid_mu_N_whole_pop": float(np.sqrt(np.mean(resid_mu ** 2))),
        "tercile_rms": tercile_rms,
        "free_fit": {k: free_fit[k] for k in ("B", "C", "D", "E")},
        "mu_fit": {k: mu_fit[k] for k in ("B", "C", "D", "E", "mu", "mean_axle_fz_N")},
        "median_fz_N": float(np.median(z2)), "p25_fz_N": float(np.percentile(z2, 25)),
        "p75_fz_N": float(np.percentile(z2, 75)),
    }
    return numbers, a2_deg, f2, z2, free_fit, mu_fit


def _plot_axle(axle_name, focus_label, a2_deg, f2, z2, free_fit, mu_fit, median_fz, p25_fz, p75_fz):
    theme = ps.PRINT
    fig, ax = plt.subplots(figsize=(ps.PRINT_WIDTH_CM / 2.54, 12.0 / 2.54), constrained_layout=True)
    fig.patch.set_facecolor(theme["bg"])
    ax.set_facecolor(theme["bg"])

    sc = ax.scatter(a2_deg, f2, c=z2, cmap="viridis", s=3.0, alpha=0.35, linewidths=0,
                     label="_nolegend_")
    cbar = fig.colorbar(sc, ax=ax, pad=0.01)
    cbar.set_label("Measured Fz (N)", color=theme["text"])
    cbar.ax.tick_params(colors=theme["text"])

    grid_deg = np.linspace(a2_deg.min(), a2_deg.max(), 400)
    grid_rad = np.radians(grid_deg)

    y_freeD = pacejka_lateral_force(grid_rad, free_fit["B"], free_fit["C"], free_fit["D"], free_fit["E"])
    ax.plot(grid_deg, y_freeD, color=theme["text"], linewidth=1.6, linestyle="-",
             label=f"free-D fit (D={free_fit['D']:.0f}N)")

    y_mu_med = pacejka_lateral_force(grid_rad, mu_fit["B"], mu_fit["C"], mu_fit["mu"] * median_fz, mu_fit["E"])
    ax.plot(grid_deg, y_mu_med, color=ps.LAP_PALETTE[3], linewidth=1.6, linestyle="-",
             label=f"mu fit @ median Fz={median_fz:.0f}N (mu={mu_fit['mu']:.3f})")

    y_mu_p25 = pacejka_lateral_force(grid_rad, mu_fit["B"], mu_fit["C"], mu_fit["mu"] * p25_fz, mu_fit["E"])
    y_mu_p75 = pacejka_lateral_force(grid_rad, mu_fit["B"], mu_fit["C"], mu_fit["mu"] * p75_fz, mu_fit["E"])
    ax.plot(grid_deg, y_mu_p25, color=ps.LAP_PALETTE[3], linewidth=1.0, linestyle="--",
             label=f"mu fit @ p25 Fz={p25_fz:.0f}N")
    ax.plot(grid_deg, y_mu_p75, color=ps.LAP_PALETTE[3], linewidth=1.0, linestyle=":",
             label=f"mu fit @ p75 Fz={p75_fz:.0f}N")
    ax.fill_between(grid_deg, y_mu_p25, y_mu_p75, color=ps.LAP_PALETTE[3], alpha=0.12, linewidth=0)

    ax.set_xlabel("Slip angle (deg)")
    ax.set_ylabel("Fy (N)")
    ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
    ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
    for spine in ax.spines.values():
        spine.set_color(theme["text_muted"])
    leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.85, framealpha=0.85, loc="best")
    leg.get_frame().set_facecolor(theme["bg"])
    for text in leg.get_texts():
        text.set_color(theme["text"])

    fig.suptitle(f"v3 {axle_name} tyre cloud, coloured by measured Fz -- {focus_label}",
                 color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"fz_mu_v3_{axle_name}_tyre_cloud.png")
    save_png(fig, out_path)
    return out_path


def main():
    base_mask, slip_kin, forces, fz = _load_v3()

    all_numbers = {}
    for axle_name, focus_label, axle_key in AXLES:
        numbers, a2_deg, f2, z2, free_fit, mu_fit = _analyse_axle(
            axle_name, axle_key, base_mask, slip_kin, forces, fz)
        all_numbers[axle_name] = numbers

        print(f"\n=== v3 {axle_name} ({focus_label}) ===")
        print(f"  n_fit_population = {numbers['n_fit_population']}")
        print(f"  corr(Fz, |alpha|) = {numbers['corr_fz_abs_alpha']:+.4f}")
        print(f"  corr(Fz, |Fy|)    = {numbers['corr_fz_abs_fy']:+.4f}")
        print(f"  rms_resid, whole fit population: free-D={numbers['rms_resid_freeD_N_whole_pop']:.1f}N  "
              f"mu={numbers['rms_resid_mu_N_whole_pop']:.1f}N")
        print(f"  Fz terciles:")
        for band in ("low", "mid", "high"):
            t = numbers["tercile_rms"][band]
            print(f"    {band:>4} (n={t['n']}, Fz=[{t['fz_range_N'][0]:.0f},{t['fz_range_N'][1]:.0f}]N): "
                  f"rms_resid_freeD={t['rms_resid_freeD_N']:.1f}N  rms_resid_mu={t['rms_resid_mu_N']:.1f}N")
        print(f"  mu={numbers['mu_fit']['mu']:.4f}  median_fz={numbers['median_fz_N']:.1f}N  "
              f"p25={numbers['p25_fz_N']:.1f}N  p75={numbers['p75_fz_N']:.1f}N")

        out_path = _plot_axle(axle_name, focus_label, a2_deg, f2, z2, free_fit, mu_fit,
                               numbers["median_fz_N"], numbers["p25_fz_N"], numbers["p75_fz_N"])
        print(f"  saved {out_path}")

    import json
    out_json = "diagnostics/fz_mu_v3_rear_divergence_numbers.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(all_numbers, f, indent=2, default=str)
    print(f"\nsaved {out_json}")


if __name__ == "__main__":
    main()
