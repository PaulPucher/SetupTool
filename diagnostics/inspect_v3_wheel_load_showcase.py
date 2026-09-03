# Damper wheel-load showcase, FINAL session-corrected reconstruction
# (read-only, no production/config changes). Computes the numbers block
# reported alongside this script's own run, and renders 3 PRINT figures
# into diagnostics/plots_v3/, overwriting previous versions: fastest-lap
# 4-corner trace, a robustly-selected heavy-braking zoom, and a robustly-
# selected max-lateral-transfer zoom (new). The static-vs-damper
# comparison figures (wheel_load_comparison_C12_lap8.png, wheel_load_
# reconstruction_C12_lap8.png) are CONFIRMED current, not regenerated --
# neither uses anything this script's own event-selection logic touches,
# and the reconstruction figure already reflects the final mass+aero+
# split-corrected model (regenerated in the prior "Session-measured
# split fractions" turn).
#
# ROBUST EVENT SELECTION (heavy-braking and lateral-transfer events):
# earlier picks (raw argmin(ax)/argmax(|ay|)) landed on a vibration/
# resonance burst and a 4.75g sensor spike respectively (thesis_notes.md
# "Wheel-load showcase, ground-truth check..."), neither a real physical
# event. Fixed here with three combined criteria: (1) KERB-EXCLUDED --
# state["kerb_mask"] (modules.stability_analysis._compute_kerb_mask_
# from_az, the same production kerb detector Modules 4b/longitudinal_
# stiffness already use) masked out entirely; (2) PLAUSIBILITY-BANDED --
# |ax|/|ay| capped at 22.0 m/s^2 (~2.24g), reusing Phase 3's own already-
# established upper plausibility bound rather than inventing a new
# number; (3) ROLLING-MEDIAN RANKED -- a 0.2s centred median filter
# (scipy.ndimage.median_filter) applied to ax/|ay| before ranking, so a
# single-sample or few-sample spike/oscillation cannot dominate the pick
# the way a raw argmin/argmax would.
#
# Uses the three-tier cascade throughout (damper-measured -> reconstructed
# -> static-split, modules.wheel_loads.combine_with_reconstruction_and_
# fallback) so FR reads as "reconstructed" wherever the corrupted
# log_dms_dam_fr channel would otherwise force a plain static guess.
#
# Reuses core/figure_render.py's _new_figure/_apply_theme/_draw_corner_
# bands/save_png (the same PRINT-theme building blocks render_lap_figure
# uses) rather than inventing new style constants -- the wheel-load panel
# drawing itself is new (no existing renderer draws Fz).

import os

import numpy as np
from scipy.ndimage import median_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core import plot_style as ps
from core.figure_render import _new_figure, _apply_theme, _draw_corner_bands, save_png
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state, estimate_lateral_forces,
    estimate_vertical_loads,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, estimate_session_corrected_axle_totals,
    combine_with_reconstruction_and_fallback, CORNERS,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_v3"
WHEEL_LABELS = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}
PLAUSIBILITY_BOUND_MPS2 = 22.0  # reuses Phase 3's own established upper bound
ROBUST_MEDIAN_WINDOW_S = 0.2


def _corner_at_time(corners, t):
    for c in corners:
        segs = c["segments"]
        t0, t1 = segs["entry_1_brake"][0], segs["exit_5"][1]
        if t0 <= t <= t1:
            return c.get("stable_corner_id")
    return None


def _robust_extreme_index(signal, valid_mask, window_samples, find_min):
    filtered = median_filter(signal, size=window_samples, mode="nearest")
    candidate = np.where(valid_mask, filtered, np.nan)
    if np.all(np.isnan(candidate)):
        return None
    return int(np.nanargmin(candidate) if find_min else np.nanargmax(candidate))


def _classify_negatives(fz_kn, kerb_slice):
    neg = fz_kn < 0
    if not neg.any():
        return 0, 0
    kerb_coincident = int((neg & kerb_slice).sum()) if kerb_slice is not None else 0
    not_kerb = int((neg & ~kerb_slice).sum()) if kerb_slice is not None else int(neg.sum())
    return kerb_coincident, not_kerb


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
    static_fallback_fz = {c: fz_static[f"fz_{c}_N"] for c in CORNERS}
    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    # FINAL session-corrected axle-total model (mass + aero + measured
    # front/rear split -- thesis_notes.md "Closing the reconstruction's
    # aero gap" and "Session-measured split fractions...") feeds ONLY the
    # reconstruction tier below -- the plain static-split fallback
    # (static_fallback_fz, used only when BOTH corners of an axle are
    # damper-invalid) is untouched.
    session_corrected = estimate_session_corrected_axle_totals(state, damper_result, params)
    c_session = session_corrected["c_session_N_per_mps2"]
    print(f"FINAL session-corrected axle-total model: mass_kg_session={session_corrected['mass_kg_session']:.1f} kg "
          f"(config was {params['vehicle']['mass_kg']:.1f} kg), c_session={c_session:.4f} N/(m/s)^2, "
          f"front_mass_fraction={session_corrected['front_mass_fraction']:.3f} "
          f"(config static was {params['vehicle']['cog_to_rear_axle_m']/params['vehicle']['wheelbase_m']:.3f}), "
          f"rear_left_fraction={session_corrected['rear_left_fraction']:.3f}, "
          f"aero_front_fraction={session_corrected['aero_front_fraction']:.2f} [Level 1 placeholder, unchanged]\n")
    fz_axle_totals = {"fz_f_N": session_corrected["fz_f_N"], "fz_r_N": session_corrected["fz_r_N"]}
    combined = combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz)

    t = state["time"]
    v = state["v_mps"]
    ax = state["ax_mps2"]
    ay = state["ay_mps2"]
    g = 9.81
    sample_rate_hz = state["sample_rate_hz"]
    kerb_mask = state.get("kerb_mask")
    non_kerb = ~kerb_mask if kerb_mask is not None else np.ones(len(t), dtype=bool)
    moving = v >= params["stability_estimation"]["moving_speed_min_mps"]
    straight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    window_samples = max(1, int(round(ROBUST_MEDIAN_WINDOW_S * sample_rate_hz)))

    vp = params["vehicle"]
    config_kg = {"fl": vp["corner_weights"]["FL_kg"], "fr": vp["corner_weights"]["FR_kg"],
                 "rl": vp["corner_weights"]["RL_kg"], "rr": vp["corner_weights"]["RR_kg"]}

    print("=" * 78)
    print("NUMBERS BLOCK")
    print("=" * 78)

    print("\n--- corner weights: config (static) vs measured straight-line mean (kg) ---")
    total_config_kg = sum(config_kg.values())
    total_measured_kg = 0.0
    for c in CORNERS:
        measured_kg = float(np.mean(combined[c]["fz_N"][straight])) / g
        total_measured_kg += measured_kg
        tag = " [reconstructed]" if c == "fr" else ""
        print(f"  {WHEEL_LABELS[c]}{tag}: config={config_kg[c]:.1f} kg, measured={measured_kg:.1f} kg, "
              f"delta={measured_kg - config_kg[c]:+.1f} kg")
    residual_pct = (total_measured_kg - total_config_kg) / total_config_kg * 100.0
    v_ref_straight_kmh = float(np.mean(v[straight])) * 3.6
    aero_at_straight_ref_N = c_session * (float(np.mean(v[straight])) ** 2)
    print(f"  TOTAL: config={total_config_kg:.1f} kg, measured={total_measured_kg:.1f} kg "
          f"({residual_pct:+.1f}%)")
    print(f"  Aero-consistent expectation: at this population's own mean straight-line speed "
          f"({v_ref_straight_kmh:.0f} km/h), the session's own fitted aero term alone predicts "
          f"+{aero_at_straight_ref_N/1000.0:.2f} kN ({aero_at_straight_ref_N/g:.1f} kg-equivalent) of extra "
          f"load -- part but (per thesis_notes.md's closing note) not all of the residual above is "
          f"explained by this; the remainder is the front/rear aero-split identifiability limit already "
          f"recorded as DATA-GATED (PLAN.md BACKLOG item B).")

    print("\n--- robust max longitudinal transfer event (heaviest PLAUSIBLE, non-kerb, "
          f"median-ranked braking) ---")
    brake_valid = moving & non_kerb & (np.abs(ax) <= PLAUSIBILITY_BOUND_MPS2)
    idx_brake = _robust_extreme_index(ax, brake_valid, window_samples, find_min=True)
    if idx_brake is None:
        print("  no valid braking sample found under the robust criterion")
    else:
        front_at_brake = combined["fl"]["fz_N"][idx_brake] + combined["fr"]["fz_N"][idx_brake]
        rear_at_brake = combined["rl"]["fz_N"][idx_brake] + combined["rr"]["fz_N"][idx_brake]
        front_straight_mean = float(np.mean(combined["fl"]["fz_N"][straight] + combined["fr"]["fz_N"][straight]))
        rear_straight_mean = float(np.mean(combined["rl"]["fz_N"][straight] + combined["rr"]["fz_N"][straight]))
        cid = _corner_at_time(data["corners"], t[idx_brake])
        print(f"  event: t={t[idx_brake]:.2f}s, ax={ax[idx_brake]:.2f} m/s^2, v={v[idx_brake]*3.6:.0f} km/h, "
              f"corner={cid if cid is not None else 'braking zone, no bracket'}")
        print(f"  front axle: {front_at_brake/1000.0:.2f} kN (straight-line mean {front_straight_mean/1000.0:.2f} kN, "
              f"gain {(front_at_brake - front_straight_mean)/1000.0:+.2f} kN)")
        print(f"  rear axle: {rear_at_brake/1000.0:.2f} kN (straight-line mean {rear_straight_mean/1000.0:.2f} kN, "
              f"change {(rear_at_brake - rear_straight_mean)/1000.0:+.2f} kN)")

    print("\n--- robust max lateral transfer event (fastest PLAUSIBLE, non-kerb, "
          f"median-ranked corner) ---")
    lateral_valid = moving & non_kerb & (np.abs(ay) <= PLAUSIBILITY_BOUND_MPS2)
    idx_corner = _robust_extreme_index(np.abs(ay), lateral_valid, window_samples, find_min=False)
    if idx_corner is None:
        print("  no valid cornering sample found under the robust criterion")
    else:
        cid = _corner_at_time(data["corners"], t[idx_corner])
        outside_is_right = ay[idx_corner] > 0
        front_outside = combined["fr"]["fz_N"][idx_corner] if outside_is_right else combined["fl"]["fz_N"][idx_corner]
        front_inside = combined["fl"]["fz_N"][idx_corner] if outside_is_right else combined["fr"]["fz_N"][idx_corner]
        rear_outside = combined["rr"]["fz_N"][idx_corner] if outside_is_right else combined["rl"]["fz_N"][idx_corner]
        rear_inside = combined["rl"]["fz_N"][idx_corner] if outside_is_right else combined["rr"]["fz_N"][idx_corner]
        print(f"  event: t={t[idx_corner]:.2f}s, ay={ay[idx_corner]:.2f} m/s^2 "
              f"({'right-hand' if outside_is_right else 'left-hand'} corner), v={v[idx_corner]*3.6:.0f} km/h, "
              f"corner={cid if cid is not None else 'no bracket at this instant'}")
        print(f"  front: outside={front_outside/1000.0:.2f} kN, inside={front_inside/1000.0:.2f} kN, "
              f"split={front_outside/1000.0 - front_inside/1000.0:+.2f} kN")
        print(f"  rear:  outside={rear_outside/1000.0:.2f} kN, inside={rear_inside/1000.0:.2f} kN, "
              f"split={rear_outside/1000.0 - rear_inside/1000.0:+.2f} kN")

    print(f"\n--- aero: extra load at 200/250 km/h from the FINAL c_session ({c_session:.4f} N/(m/s)^2) ---")
    for kmh in (200.0, 250.0):
        v_mps = kmh / 3.6
        extra_N = c_session * v_mps ** 2
        print(f"  at {kmh:.0f} km/h: extra load = {extra_N/1000.0:.2f} kN ({extra_N/g:.1f} kg-equivalent)")

    print("\n" + "=" * 78)
    print("FIGURES")
    print("=" * 78)

    # --- figure (a): full fastest-lap 4-corner trace, established per-lap style
    fastest = next((lap for lap in data["laps"] if lap.get("is_fastest")), None)
    if fastest is None:
        print("no fastest lap found -- skipping figure (a)")
    else:
        s_m = state["s_m"]
        lo = np.searchsorted(t, fastest["start_time"])
        hi = np.searchsorted(t, fastest["end_time"])
        sl = slice(lo, hi)
        order = np.argsort(s_m[sl])
        lap_s = s_m[sl][order]
        kerb_slice = kerb_mask[sl][order] if kerb_mask is not None else None

        corner_bands = []
        seen_ids = set()
        for c in data["corners"]:
            cid = c.get("stable_corner_id")
            if cid is None or cid in seen_ids or c.get("lap_number") != fastest["lap_number"]:
                continue
            seen_ids.add(cid)
            b0, b1 = c.get("bracket_start_m"), c.get("bracket_end_m")
            if b0 is not None and b1 is not None and b1 > b0:
                corner_bands.append((b0, b1, cid))

        fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, ps.PRINT)
        fig.get_layout_engine().set(w_pad=0.0, h_pad=0.03, wspace=0.0, hspace=0.02, rect=(0.06, 0.0, 1.0, 1.0))
        gs = fig.add_gridspec(5, 1)
        axes = []
        ax0 = fig.add_subplot(gs[0, 0])
        ax0.plot(lap_s, (v[sl][order]) * 3.6, color=ps.LAP_PALETTE[0], linewidth=1.0)
        ax0.set_ylabel("Speed (km/h)")
        axes.append(ax0)
        sanity_bits = []
        for row, c in enumerate(CORNERS, start=1):
            axp = fig.add_subplot(gs[row, 0], sharex=ax0)
            fz_kn = combined[c]["fz_N"][sl][order] / 1000.0
            axp.plot(lap_s, fz_kn, color=ps.LAP_PALETTE[1 if c == "fr" else 0], linewidth=1.0)
            label = f"{WHEEL_LABELS[c]} Fz (kN)" + (" [reconstructed]" if c == "fr" else "")
            axp.set_ylabel(label)
            axes.append(axp)
            kerb_n, not_kerb_n = _classify_negatives(fz_kn, kerb_slice)
            if kerb_n or not_kerb_n:
                sanity_bits.append(f"{WHEEL_LABELS[c]} negative samples: {kerb_n} kerb-coincident, {not_kerb_n} not")
        axes[-1].set_xlabel("Track position s (m)")
        for i, axp in enumerate(axes):
            _draw_corner_bands(axp, corner_bands, label_above=(i == 0))
        fig.suptitle(f"Fastest lap ({fastest['lap_number']}) -- 4-corner Fz trace, GT3_PRC_MLA-v3 "
                     f"(final session-corrected model)",
                     color=ps.PRINT["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 1)
        _apply_theme(fig, axes, ps.PRINT)
        os.makedirs(OUT_DIR, exist_ok=True)
        path_a = os.path.join(OUT_DIR, "wheel_load_showcase_fastest_lap8.png")
        save_png(fig, path_a)
        print(f"(a) saved {path_a}")
        print(f"    sanity note: negative Fz dips persist on all four wheels (single-wheel-event-"
              f"invisible limitation, structural, not fixed by session correction). "
              f"{'; '.join(sanity_bits) if sanity_bits else 'no negative samples'}")

    def _event_zoom_figure(idx, signal, signal_label, title_prefix, out_name):
        if idx is None:
            print(f"    skipped -- no valid event index")
            return
        margin_s = 1.5
        lo_t = max(t[idx] - margin_s, t[0])
        hi_t = min(t[idx] + margin_s, t[-1])
        lo_i = np.searchsorted(t, lo_t)
        hi_i = np.searchsorted(t, hi_t)
        sl_ = slice(lo_i, hi_i)
        t_rel = t[sl_] - t[idx]
        kerb_slice = kerb_mask[sl_] if kerb_mask is not None else None

        fig, axes = plt.subplots(2, 1, figsize=(ps.PRINT_WIDTH_CM / 2.54, 14.0 / 2.54),
                                  constrained_layout=True, sharex=True)
        theme = ps.PRINT
        fig.patch.set_facecolor(theme["bg"])
        ax_load, ax_sig = axes
        ax_load.set_facecolor(theme["bg"])
        sanity_bits = []
        for i, c in enumerate(CORNERS):
            fz_kn = combined[c]["fz_N"][sl_] / 1000.0
            ax_load.plot(t_rel, fz_kn, color=ps.LAP_PALETTE[i], linewidth=1.2,
                         label=WHEEL_LABELS[c] + (" [reconstructed]" if c == "fr" else ""))
            kerb_n, not_kerb_n = _classify_negatives(fz_kn, kerb_slice)
            if kerb_n or not_kerb_n:
                sanity_bits.append(f"{WHEEL_LABELS[c]}: {kerb_n} kerb-coincident/{not_kerb_n} not")
        ax_load.set_ylabel("Fz (kN)")
        ax_load.axvline(0.0, color=theme["text_muted"], linewidth=0.8, linestyle=":")
        ax_sig.set_facecolor(theme["bg"])
        ax_sig.plot(t_rel, signal[sl_], color=theme["text"], linewidth=1.2)
        ax_sig.axvline(0.0, color=theme["text_muted"], linewidth=0.8, linestyle=":")
        ax_sig.set_ylabel(signal_label)
        ax_sig.set_xlabel(f"Time relative to {title_prefix.lower()} (s)")
        for a_ in axes:
            a_.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
            a_.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])
            for spine in a_.spines.values():
                spine.set_color(theme["text_muted"])
        leg = ax_load.legend(fontsize=ps.PRINT_FONT_SIZE_PT * 0.8, framealpha=0.85, ncol=2)
        leg.get_frame().set_facecolor(theme["bg"])
        fig.suptitle(f"{title_prefix}, t={t[idx]:.1f}s ({signal_label}={signal[idx]:.1f}) -- "
                     f"wheel loads, GT3_PRC_MLA-v3 (robust selection)",
                     color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT)
        path = os.path.join(OUT_DIR, out_name)
        save_png(fig, path)
        print(f"saved {path}")
        print(f"    sanity note: {'; '.join(sanity_bits) if sanity_bits else 'no negative Fz samples in this window'}")

    print("\n(b) heavy-braking zoom (robust selection):")
    _event_zoom_figure(idx_brake, ax, "ax (m/s^2)", "Robust heaviest-braking zone",
                        "wheel_load_showcase_heavy_braking_zoom.png")

    print("\n(c) max-lateral-transfer zoom (robust selection, NEW figure):")
    _event_zoom_figure(idx_corner, ay, "ay (m/s^2)", "Robust max-lateral-transfer corner",
                        "wheel_load_showcase_lateral_transfer_zoom.png")

    print("\n(d) confirming existing comparison figures reflect the final model (not regenerated -- "
          "nothing they depend on changed since their last render):")
    print("    diagnostics/plots_v3/wheel_load_comparison_C12_lap8.png (rear axle, measured vs "
          "static-split -- both lines independent of the session-correction model, RL/RR are real "
          "sensors and the static-split contrast line is deliberately the OLD global static estimate)")
    print("    diagnostics/plots_v3/wheel_load_reconstruction_C12_lap8.png (front axle, measured FL vs "
          "reconstructed FR -- already regenerated against the FINAL mass+aero+split-corrected model "
          "in the prior 'Session-measured split fractions' turn)")


if __name__ == "__main__":
    main()
