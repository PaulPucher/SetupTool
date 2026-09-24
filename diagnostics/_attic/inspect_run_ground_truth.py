# Ground-truth workup, diagnostics only: for the specific below-zero
# CS_ratio runs already found (thesis_notes.md "Persistence-length
# diagnostic..."), gathers four independent evidence lines per run --
# tyre-curve picture (fold/loop/plateau), combined-slip (LS_ratio/kappa)
# corroboration, driver-input (steering-rate) and stability corroboration
# -- to support a REAL/ARTIFACT/MIXED read per run. No verdict is
# computed here; this script reports numbers and renders only, verdicts
# are synthesised afterward against the rendered figures.
#
# Renderer fix (Part 5, diagnostic-side only -- core/figure_render.py
# itself untouched): corner_by_lap and fitted_line are populated here
# exactly as ui/views/corner_trace_dialog.py's own equivalent builder
# does (per-lap coloured clean/kerb samples; the real fitted Pacejka
# curve from fit_manifest), not the older pooled/no-fit contract
# diagnostics/inspect_step2_chair_plots.py's _build_tyre_curve used.

import os

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, reconstruct_cs_window_start,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.tyre_model_pacejka import pacejka_lateral_force
from modules.geo import project_latlon_to_xy
from core import figure_render, plot_style
from diagnostics.inspect_step2_chair_plots import _canonical_window_slice, _build_track_map

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
OUT_DIR = "diagnostics/plots_ground_truth"

# (stable_corner_id, axle, lap_number) -- the specific runs named in the
# work order: C3 rear all three 100m+ runs; C2 front both 45.2m runs
# (lap2 AND lap3 tied at the same length -- both included rather than
# guessing which single one was meant); C4 all five runs (front lap1/
# 3/4, rear lap3/4).
TARGET_RUNS = [
    (3, "r", 1), (3, "r", 2), (3, "r", 3),
    (2, "f", 2), (2, "f", 3),
    (4, "f", 1), (4, "f", 3), (4, "f", 4),
    (4, "r", 3), (4, "r", 4),
]


def _negative_runs(values, s_m_local, global_offset):
    runs = []
    n = len(values)
    i = 0
    while i < n:
        if values[i] == values[i] and values[i] < 0:
            j = i
            while j + 1 < n and values[j + 1] == values[j + 1] and values[j + 1] < 0:
                j += 1
            runs.append({
                "start_global": i + global_offset, "end_global": j + global_offset,
                "length_samples": j - i + 1, "length_m": float(s_m_local[j] - s_m_local[i]),
                "depth": float(np.median(values[i:j + 1])),
            })
            i = j + 1
        else:
            i += 1
    return runs


def _phase_slices(t, segments, apex_half_window_samples):
    out = {}
    for phase, (start_t, end_t) in segments.items():
        if end_t < start_t:
            out[phase] = slice(0, 0)
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if phase == "apex_3" and hi <= lo:
            centre = lo
            lo = max(0, centre - apex_half_window_samples)
            hi = min(len(t), centre + apex_half_window_samples + 1)
        out[phase] = slice(lo, hi)
    return out


def _build_corner_by_lap(instances, laps_by_number, t, s_m, alpha_arr, Fy_arr, kerb_mask):
    styles = plot_style.lap_styles(c["lap_number"] for c in instances)
    corner_by_lap = []
    pooled_alpha = []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None:
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        lap_alpha, lap_Fy = alpha_arr[sl], Fy_arr[sl]
        lap_kerb = kerb_mask[sl] if kerb_mask is not None else np.zeros(sl.stop - sl.start, dtype=bool)
        valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
        clean = valid & ~lap_kerb
        kerbed = valid & lap_kerb
        if valid.any():
            pooled_alpha.append(lap_alpha[valid])
        corner_by_lap.append({
            "lap_number": c["lap_number"], **styles[c["lap_number"]],
            "clean_xy": (np.degrees(lap_alpha[clean]), lap_Fy[clean]) if clean.any() else None,
            "kerb_xy": (np.degrees(lap_alpha[kerbed]), lap_Fy[kerbed]) if kerbed.any() else None,
        })
    pooled = np.concatenate(pooled_alpha) if pooled_alpha else np.array([])
    max_abs_alpha = float(np.max(np.abs(pooled))) if pooled.size else 0.0
    return corner_by_lap, max_abs_alpha


def _fitted_line(fit_manifest, axle, max_abs_alpha):
    if fit_manifest is None or max_abs_alpha <= 0:
        return None
    axle_fit = fit_manifest.get("axles", {}).get(axle)
    if axle_fit is None:
        return None
    alpha_grid_rad = np.linspace(-max_abs_alpha, max_abs_alpha, 200)
    fy_grid = pacejka_lateral_force(alpha_grid_rad, axle_fit["B"], axle_fit["C"], axle_fit["D"], axle_fit["E"])
    return (np.degrees(alpha_grid_rad), fy_grid, "Fitted tyre model (Pacejka)")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    t = state["time"]
    s_m = state["s_m"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    sample_rate_hz = state["sample_rate_hz"]

    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to run")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], params)
    ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)

    steering_rate = np.gradient(state["delta_f_rad"], t)  # rad/s

    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]
    apex_half_window_samples = se["apex_half_window_samples"]

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}

    corners_by_id = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_id.setdefault(sid, []).append(c)

    gps_lat, gps_lon = state.get("gps_lat"), state.get("gps_lon")
    origin_lat, origin_lon = state.get("gps_origin_lat"), state.get("gps_origin_lon")
    bg_xy = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)

    alpha_arrs = {"f": slip["alpha_f_filt"], "r": slip["alpha_r_filt"]}
    Fy_arrs = {"f": forces["Fy_f_filt"], "r": forces["Fy_r_filt"]}
    cs_ratio_arrs = {"f": cs["CS_ratio_f"], "r": cs["CS_ratio_r"]}
    ls_ratio_arrs = {"f": ls["LS_ratio_f"], "r": ls["LS_ratio_r"]}
    kappa_arrs = {"f": ls["kappa_f_filt"], "r": ls["kappa_r_filt"]}
    axle_name = {"f": "front", "r": "rear"}
    stab_arr = stab["stability_observed_Nm_per_deg"]

    written = []
    for cid, axle_key, lap_number in TARGET_RUNS:
        axle_label = axle_name[axle_key]
        c = next((cc for cc in corners_by_id[cid] if cc["lap_number"] == lap_number), None)
        if c is None:
            print(f"\n-- C{cid} {axle_label} lap{lap_number}: no matching corner instance --")
            continue
        lap = laps_by_number[lap_number]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        cs_ratio_arr = cs_ratio_arrs[axle_key]
        runs = _negative_runs(cs_ratio_arr[sl], s_m[sl], sl.start)
        if not runs:
            print(f"\n-- C{cid} {axle_label} lap{lap_number}: no below-zero run found (unexpected) --")
            continue
        run = max(runs, key=lambda r: r["length_m"])
        rsl = slice(run["start_global"], run["end_global"] + 1)

        print(f"\n{'=' * 90}\nC{cid} {axle_label} lap{lap_number}: run = {run['length_m']:.1f} m "
              f"({run['length_samples']} samples), depth(median)={run['depth']:.3f}, "
              f"s_m=[{s_m[run['start_global']]:.1f}, {s_m[run['end_global']]:.1f}]\n{'=' * 90}")

        # --- item 2: combined-slip corroboration ---
        ls_seg = ls_ratio_arrs[axle_key][rsl]
        kappa_seg = kappa_arrs[axle_key][rsl]
        ls_finite = ls_seg[np.isfinite(ls_seg)]
        kappa_finite = kappa_seg[np.isfinite(kappa_seg)]
        print("  LS_ratio (same axle) over the run span: "
              + (f"n={ls_finite.size}, median={np.median(ls_finite):.3f}, min={ls_finite.min():.3f}"
                 if ls_finite.size else "no finite samples"))
        print("  kappa (slip ratio, same axle) over the run span: "
              + (f"n={kappa_finite.size}, median={np.median(kappa_finite):.4f}, "
                 f"max|kappa|={np.max(np.abs(kappa_finite)):.4f}"
                 if kappa_finite.size else "no finite samples"))

        # --- item 3: driver-input + stability corroboration ---
        lap_sl = slice(int(np.searchsorted(t, lap["start_time"])), int(np.searchsorted(t, lap["end_time"])))
        lap_rate = np.abs(steering_rate[lap_sl])
        lap_rate = lap_rate[np.isfinite(lap_rate)]
        run_rate = np.abs(steering_rate[rsl])
        run_rate = run_rate[np.isfinite(run_rate)]
        p95_lap = np.percentile(lap_rate, 95) if lap_rate.size else float("nan")
        print(f"  steering-rate |rad/s|: lap p95={p95_lap:.3f}, run median={np.median(run_rate):.3f}, "
              f"run max={run_rate.max():.3f} ({'ABOVE' if run_rate.max() > p95_lap else 'below'} lap p95)"
              if run_rate.size else "  steering-rate: no finite samples in run")

        stab_seg = stab_arr[rsl]
        stab_finite = stab_seg[np.isfinite(stab_seg)]
        print("  stability_observed_Nm_per_deg over the run span: "
              + (f"n={stab_finite.size}, median={np.median(stab_finite):.1f}, min={stab_finite.min():.1f}"
                 if stab_finite.size else "no finite samples"))

        # --- item 1: render ---
        instances = [cc for cc in corners_by_id[cid] if cc["lap_number"] in valid_lap_numbers]
        fig_tyre_curves = {}
        for ak in ("f", "r"):
            alpha_arr, Fy_arr = alpha_arrs[ak], Fy_arrs[ak]
            session_valid = moving & np.isfinite(alpha_arr) & np.isfinite(Fy_arr)
            session_xy = (np.degrees(alpha_arr[session_valid]), Fy_arr[session_valid],
                          kerb_mask[session_valid] if kerb_mask is not None else None)
            corner_by_lap, max_abs_alpha = _build_corner_by_lap(
                instances, laps_by_number, t, s_m, alpha_arr, Fy_arr, kerb_mask
            )
            fitted = _fitted_line(fit_manifest, axle_name[ak], max_abs_alpha)
            window_xy = None
            if ak == axle_key:
                ra, rf = alpha_arr[rsl], Fy_arr[rsl]
                rv = np.isfinite(ra) & np.isfinite(rf)
                if rv.any():
                    window_xy = (np.degrees(ra[rv]), rf[rv])
            fig_tyre_curves[axle_name[ak]] = {
                "session_xy": session_xy, "corner_by_lap": corner_by_lap, "window_xy": window_xy,
                "linear_ref_line": None, "fitted_line": fitted, "tangent_line": None,
            }

        v_kmh = state["v_mps"] * 3.6
        fig_laps = []
        for cc in instances:
            lap_l = laps_by_number[cc["lap_number"]]
            lsl = _canonical_window_slice(t, s_m, lap_l["start_time"], lap_l["end_time"],
                                           cc["bracket_start_m"], cc["bracket_end_m"])
            if lsl.stop <= lsl.start:
                continue
            fig_laps.append({
                "lap_number": cc["lap_number"], "selected": cc["lap_number"] == lap_number,
                "s": s_m[lsl], "v_kmh": v_kmh[lsl],
                "cs_f": cs["CS_ratio_f"][lsl], "cs_r": cs["CS_ratio_r"][lsl],
            })
        track_map = _build_track_map(cid, instances, laps_by_number, t, s_m, bg_xy, cs,
                                      alpha_arrs["f"], alpha_arrs["r"], min_window, min_span)
        cls_cfg = params["classification"]
        thresholds = {
            "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
            "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
            "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
        }
        fig = figure_render.render_corner_figure(
            f"C{cid} lap{lap_number} {axle_label} run ({MODE})", fig_laps, thresholds,
            fig_tyre_curves, track_map, theme=plot_style.PRINT,
        )
        out_path = os.path.join(OUT_DIR, f"C{cid:02d}_{axle_key}_lap{lap_number}_run.png")
        figure_render.save_png(fig, out_path)
        written.append(out_path)
        print(f"  wrote {out_path}")

    print(f"\n{len(written)} figures written to {OUT_DIR}")


if __name__ == "__main__":
    main()
