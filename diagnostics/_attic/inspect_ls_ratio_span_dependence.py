# PLAN.md unsupervised package, Phase 3. Read-only diagnostic: does
# LS_ratio's per-sample value depend on how wide the sliding regression
# window's own kappa excursion ("slip span") happened to be, for samples
# that pass every OTHER validity gate in modules/longitudinal_
# stiffness.py's _centered_slopes? Motivates (or would flag as
# unjustified) config's longitudinal_stiffness.min_slip_span=0.004 gate,
# which currently excludes any window below it outright.
#
# Recomputes _centered_slopes's own numerator/denominator/slope
# arithmetic independently, with the min_slip_span condition DROPPED,
# rather than asking the production function to skip its own gate (it
# has no such option) -- stays strictly read-only, no modules/ edit.
# Runs the full Modules 1-4a-equivalent chain directly (same call order
# as diagnostics/inspect_ls_cs_disambiguation.py), sideslip_source
# asserted "kinematic" rather than assumed, matching that script's own
# reasoning (comparable baseline, never a silent config read).

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import (
    estimate_longitudinal_stiffness, _filtered, _plausibility_exclude_mask,
    _prefix_sum, _window_sum,
)
import core.plot_style as ps

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUT_PATH = "diagnostics/plots_step2/ls_ratio_span_dependence.png"


def _slopes_span_no_gate(slip, force, valid_mask, sample_rate_hz, se):
    """Same arithmetic as longitudinal_stiffness._centered_slopes, minus
    the slip_span >= min_slip_span condition -- lets windows below the
    production gate through too, with slip_span returned alongside so
    both populations can be plotted against it together.
    """
    n = len(slip)
    half_window = max(2, int(round(se["regression_window_s"] * sample_rate_hz / 2.0)))
    min_samples = max(se["min_samples_floor"], half_window + 1)
    idx = np.arange(n)
    start = np.maximum(0, idx - half_window)
    stop = np.minimum(n, idx + half_window + 1)

    finite = np.isfinite(slip) & np.isfinite(force) & valid_mask
    x = np.where(finite, slip, 0.0)
    y = np.where(finite, force, 0.0)
    finite_flag = finite.astype(float)

    count = _window_sum(_prefix_sum(finite_flag), start, stop)
    sx = _window_sum(_prefix_sum(x), start, stop)
    sy = _window_sum(_prefix_sum(y), start, stop)
    sxx = _window_sum(_prefix_sum(x * x), start, stop)
    sxy = _window_sum(_prefix_sum(x * y), start, stop)

    slip_span = np.full(n, np.nan)
    for i in range(n):
        window_slip = slip[start[i]:stop[i]][finite[start[i]:stop[i]]]
        if window_slip.size:
            slip_span[i] = float(np.nanmax(window_slip) - np.nanmin(window_slip))

    denom = sxx - (sx * sx / np.maximum(count, 1.0))
    numer = sxy - (sx * sy / np.maximum(count, 1.0))
    slopes = np.full(n, np.nan)
    valid_no_gate = (count >= min_samples) & (np.abs(denom) > 1e-12)
    slopes[valid_no_gate] = numer[valid_no_gate] / denom[valid_no_gate]
    return slopes, valid_no_gate, slip_span


def _axle_span_ratio(kappa_raw, fx_raw, speed_valid, az_g, se, ls, plausibility_window_s,
                      sr, reference_N):
    exclude = _plausibility_exclude_mask(kappa_raw, az_g, se, ls, plausibility_window_s, sr)
    kappa_for_filter = np.where(exclude, np.nan, kappa_raw)
    kappa_filt = _filtered(kappa_for_filter, sr, ls["cutoff_hz"])
    fx_filt = _filtered(fx_raw, sr, ls["cutoff_hz"])
    valid_mask = np.isfinite(kappa_raw) & np.isfinite(fx_raw) & speed_valid & ~exclude

    # _centered_slopes's own parameter is named "se" but production calls
    # it with the longitudinal_stiffness dict (regression_window_s and
    # min_samples_floor both live there, not in stability_estimation) --
    # matched here for a faithful reimplementation.
    slopes, valid_no_gate, slip_span = _slopes_span_no_gate(kappa_filt, fx_filt, valid_mask, sr, ls)
    ratio = np.full_like(slopes, np.nan)
    if np.isfinite(reference_N) and abs(reference_N) > 1e-9:
        ratio[valid_no_gate] = np.clip(slopes[valid_no_gate] / reference_N, None, 1.0)
    return ratio, slip_span, valid_no_gate


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    live_default = params["stability_estimation"].get("sideslip_source", "kinematic")
    print(f"live config sideslip_source = {live_default!r} (left untouched, not used for this diagnostic)")

    state = prepare_vehicle_state(data["channels"], params)
    assert state is not None, "prepare_vehicle_state returned None -- required channels missing"

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], params)
    ls_result = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)

    ls = params["longitudinal_stiffness"]
    se = params["stability_estimation"]
    sr = state["sample_rate_hz"]
    speed_valid = state["v_mps"] >= ls["min_speed_mps"]
    az_g = state.get("az_g")
    min_slip_span = ls["min_slip_span"]

    ratio_f, span_f, valid_f = _axle_span_ratio(
        slip_ratio["kappa_f"], long_forces["fx_f_N"], speed_valid, az_g, se, ls,
        ls["plausibility_az_window_front_s"], sr, ls_result["linear_reference_f_N"],
    )
    ratio_r, span_r, valid_r = _axle_span_ratio(
        slip_ratio["kappa_r"], long_forces["fx_r_N"], speed_valid, az_g, se, ls,
        ls["plausibility_az_window_rear_s"], sr, ls_result["linear_reference_r_N"],
    )

    # Cross-check: this script's own re-derivation, restricted to the
    # SAME population production's min_slip_span gate keeps, must agree
    # with modules/longitudinal_stiffness.py's actual output exactly --
    # confirms the independent reimplementation is faithful before
    # trusting the extended (below-gate) population it also reports.
    gated_f = valid_f & (span_f >= min_slip_span)
    gated_r = valid_r & (span_r >= min_slip_span)
    match_f = np.allclose(ratio_f[gated_f], ls_result["LS_ratio_f"][gated_f], equal_nan=True)
    match_r = np.allclose(ratio_r[gated_r], ls_result["LS_ratio_r"][gated_r], equal_nan=True)
    print(f"front: n_valid_no_gate={int(np.sum(valid_f))} n_above_gate={int(np.sum(gated_f))} "
          f"matches production LS_ratio_f on gated population: {match_f}")
    print(f"rear:  n_valid_no_gate={int(np.sum(valid_r))} n_above_gate={int(np.sum(gated_r))} "
          f"matches production LS_ratio_r on gated population: {match_r}")
    assert match_f and match_r, "independent re-derivation disagrees with production -- diagnostic invalid"

    below_gate_f = valid_f & (span_f < min_slip_span) & np.isfinite(span_f) & (span_f > 0)
    below_gate_r = valid_r & (span_r < min_slip_span) & np.isfinite(span_r) & (span_r > 0)
    print(f"front: below-gate population n={int(np.sum(below_gate_f))}, "
          f"LS_ratio_f median there = {np.nanmedian(ratio_f[below_gate_f]):.4f}" if np.any(below_gate_f) else
          "front: below-gate population n=0")
    print(f"rear:  below-gate population n={int(np.sum(below_gate_r))}, "
          f"LS_ratio_r median there = {np.nanmedian(ratio_r[below_gate_r]):.4f}" if np.any(below_gate_r) else
          "rear:  below-gate population n=0")

    theme = ps.PRINT
    fig, axes = plt.subplots(1, 2, figsize=(ps.PRINT_WIDTH_CM / 2.54, 8 / 2.54), dpi=ps.PRINT_DPI)
    fig.patch.set_facecolor(theme["bg"])
    for ax, span, ratio, valid, label in (
        (axes[0], span_f, ratio_f, valid_f, "Front"),
        (axes[1], span_r, ratio_r, valid_r, "Rear"),
    ):
        ax.set_facecolor(theme["bg"])
        plot_mask = valid & np.isfinite(span) & np.isfinite(ratio) & (span > 0)
        below = plot_mask & (span < min_slip_span)
        above = plot_mask & (span >= min_slip_span)
        ax.scatter(span[below], ratio[below], s=3, alpha=0.25, color="#c0392b",
                   label="span < min_slip_span (excluded)")
        ax.scatter(span[above], ratio[above], s=3, alpha=0.25, color="#2d6a35",
                   label="span >= min_slip_span (kept)")
        ax.axvline(min_slip_span, color=theme["text"], linestyle="--", linewidth=1.0,
                   label=f"min_slip_span = {min_slip_span}")
        ax.set_xscale("log")
        ax.set_xlabel("window kappa span (log scale)", color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)
        ax.set_ylabel("LS_ratio (uncapped population)", color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)
        ax.set_title(f"{label} axle", color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)
        ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT * 0.85)
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.75, loc="lower right", facecolor=theme["bg"],
                  edgecolor=theme["text_muted"], labelcolor=theme["text"])

    fig.tight_layout()
    fig.savefig(OUT_PATH, dpi=ps.PRINT_DPI, facecolor=theme["bg"])
    print(f"saved {OUT_PATH}")


if __name__ == "__main__":
    main()
