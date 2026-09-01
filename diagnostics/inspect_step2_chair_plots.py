# PLAN.md STEP 2: chair-comparable result plots, kinematic vs ekf_pass_1.
# Purpose (PLAN.md STEP 2): find out whether the strange CS values that
# opened the estimator arc (thesis_notes.md "C9 negative-CS decomposition
# + zero-slip offset finding", 2026-08-17) trace to the kinematic beta
# error or to something else, by putting kinematic- and ekf_pass_1-
# sourced results side by side on the chair's own plot structure
# (docs/literature/plotting_methods.py -- matched in structure, not code:
# that reference is interactive Plotly, this is static matplotlib PNGs).
#
# Read-only diagnostic. Calls production pipeline functions directly
# (modules/stability_analysis.py, modules/geo.py, diagnostics/
# sideslip_ekf_dugoff.py) -- no estimation logic is reimplemented here.
# The only local logic is worst-phase/window-slice bookkeeping below,
# needed because estimate_cornering_stiffness returns only per-sample
# C_alpha values, not the regression window bounds behind each one.
#
# Part B (cleanup/presentation pass) refactor: the actual PANEL/
# COMPOSITION drawing now lives in core/figure_render.py, shared with
# ui/views/corner_trace_dialog.py's "Export figure" button -- this script
# builds the same fig_data structures that button does and calls the
# same render_corner_figure, so a diagnostics PNG and an app export of
# the same corner are visually identical (Part B Q3/point 6). Only the
# WINDOW-FINDING bookkeeping below (worst-phase lookup, canonical-slice
# reconstruction) stays local -- small, already independently verified,
# same acceptable-duplication precedent as _canonical_window_slice always
# having its own copy in both places rather than a shared Qt-free module.
#
# No production file changed. No config value changed. sideslip_source
# is never read from or written to config/parameters.json -- kinematic
# and ekf_pass_1 beta are obtained by calling estimate_sideslip and
# estimate_sideslip_ekf_dugoff directly, bypassing the config-driven
# dispatch entirely.

import os

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    reconstruct_cs_window_start,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.geo import project_latlon_to_xy
from core import figure_render, plot_style

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUT_DIR = "diagnostics/plots_step2"
SOURCES = ("kinematic", "ekf_pass_1")


def _compute_beta(state, params, source):
    if source == "kinematic":
        return estimate_sideslip(state, params)
    if source == "ekf_pass_1":
        return estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")["beta"]
    raise ValueError(source)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Same lap-time -> lap-distance intersection helper as
    # diagnostics/inspect_pass1_final_validation.py's own copy.
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(np.max(lap_s[finite]))
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


def _valid_lap_instances(instances, laps_by_number):
    out = []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is not None and lap.get("is_valid_for_analysis"):
            out.append(c)
    return out


def _find_worst_phase(cs_ratio, c_alpha, instances, laps_by_number, t, s_m):
    # "Worst phase" = the sample with the lowest CS_ratio (most saturated)
    # inside this corner's canonical bracket, pooled across its valid lap
    # instances -- reuses the "worst-phase CS_ratio" concept already
    # established in this project's CS-credibility diagnostics
    # (thesis_notes.md "CS credibility diagnostics: kerb audit + filter
    # sensitivity"), not a new metric invented for this script.
    best = None
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        seg = cs_ratio[sl]
        if not np.isfinite(seg).any():
            continue
        local_idx = int(np.nanargmin(np.where(np.isfinite(seg), seg, np.inf)))
        val = seg[local_idx]
        global_idx = sl.start + local_idx
        if best is None or val < best[0]:
            best = (val, c["lap_number"], global_idx)
    if best is None:
        return None
    _, lap_number, idx = best
    return {"lap_number": lap_number, "index": idx,
            "cs_ratio": float(cs_ratio[idx]), "c_alpha": float(c_alpha[idx])}


# --- fig_data assembly ---------------------------------------------------
# Builds the exact structures core/figure_render.render_corner_figure
# expects -- see ui/views/corner_trace_dialog.py's _build_export_data/
# _build_tyre_curve_export/_render_track_map for the app-side equivalent
# this mirrors (same field names, same "pure display" data going in).

def _build_laps(instances, laps_by_number, t, s_m, v_kmh, cs, thresholds_unused):
    laps = []
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        laps.append({
            "lap_number": c["lap_number"], "selected": True,  # no single "clicked" lap here -- all equal weight
            "s": s_m[sl], "v_kmh": v_kmh[sl],
            "cs_f": cs["CS_ratio_f"][sl], "cs_r": cs["CS_ratio_r"][sl],
        })
    return laps


def _build_track_map(cid, instances, laps_by_number, t, s_m, bg_xy, cs, alpha_f, alpha_r, min_window, min_span):
    rep_lap_number = instances[0]["lap_number"]
    rep_lap = laps_by_number[rep_lap_number]
    rep_sl = slice(int(np.searchsorted(t, rep_lap["start_time"], side="left")),
                    int(np.searchsorted(t, rep_lap["end_time"], side="right")))
    lap_xy = (bg_xy[0][rep_sl], bg_xy[1][rep_sl])

    bracket_sl = _canonical_window_slice(t, s_m, rep_lap["start_time"], rep_lap["end_time"],
                                          instances[0]["bracket_start_m"], instances[0]["bracket_end_m"])
    bracket_xy = (bg_xy[0][bracket_sl], bg_xy[1][bracket_sl]) if bracket_sl.stop > bracket_sl.start else None

    def _window_xy(cs_ratio_arr, alpha_arr):
        wp = _find_worst_phase(cs_ratio_arr, cs_ratio_arr, instances, laps_by_number, t, s_m)
        if wp is None:
            return None
        start = reconstruct_cs_window_start(alpha_arr, wp["index"], min_window, min_span)
        window_sl = slice(start, wp["index"])
        if window_sl.stop <= window_sl.start:
            return None
        return bg_xy[0][window_sl], bg_xy[1][window_sl]

    window_f_xy = _window_xy(cs["CS_ratio_f"], alpha_f)
    window_r_xy = _window_xy(cs["CS_ratio_r"], alpha_r)
    # render_corner_figure's track map (core/figure_render.py's
    # _draw_track_map_panel) expects "brackets_by_lap" (a list, per the
    # multi-lap-colour redesign) -- this script still only ever plots one
    # representative lap's bracket, so it is wrapped as a single-entry
    # list under that lap's own colour rather than the old bare
    # "bracket_xy" key _draw_track_map_panel no longer reads.
    from core.plot_style import lap_styles
    style = lap_styles([rep_lap_number])[rep_lap_number]
    brackets_by_lap = [{"xy": bracket_xy, "color": style["color"], "dash": style["dash"],
                         "lap_number": rep_lap_number}] if bracket_xy is not None else []
    return {"lap_xy": lap_xy, "brackets_by_lap": brackets_by_lap,
            "window_f_xy": window_f_xy, "window_r_xy": window_r_xy}


def _build_tyre_curve(axle, alpha_arr, Fy_arr, ref_arr, cs_ratio_arr, c_alpha_arr, bg_mask,
                       instances, laps_by_number, t, s_m, min_window, min_span, kerb_mask):
    session_valid = bg_mask & np.isfinite(alpha_arr) & np.isfinite(Fy_arr)
    session_xy = (np.degrees(alpha_arr[session_valid]), Fy_arr[session_valid],
                  kerb_mask[session_valid] if kerb_mask is not None else None)

    pooled_alpha, pooled_Fy, pooled_ref = [], [], []
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        lap_alpha, lap_Fy = alpha_arr[sl], Fy_arr[sl]
        valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
        if valid.any():
            pooled_alpha.append(lap_alpha[valid])
            pooled_Fy.append(lap_Fy[valid])
        if ref_arr is not None:
            lap_ref = ref_arr[sl][valid]
            finite_ref = lap_ref[np.isfinite(lap_ref)]
            if finite_ref.size:
                pooled_ref.append(finite_ref)

    pooled_alpha_rad = np.concatenate(pooled_alpha) if pooled_alpha else np.array([])
    pooled_Fy_arr = np.concatenate(pooled_Fy) if pooled_Fy else np.array([])
    corner_xy = (np.degrees(pooled_alpha_rad), pooled_Fy_arr)
    max_abs_alpha = float(np.max(np.abs(pooled_alpha_rad))) if pooled_alpha_rad.size else 0.0

    ref_line = None
    if pooled_ref and max_abs_alpha > 0:
        ref_slope = float(np.median(np.concatenate(pooled_ref)))
        if ref_slope > 0:
            x_line_rad = np.array([-max_abs_alpha, max_abs_alpha])
            ref_line = (np.degrees(x_line_rad), ref_slope * x_line_rad)

    window_xy = None
    tangent_line = None
    wp = _find_worst_phase(cs_ratio_arr, c_alpha_arr, instances, laps_by_number, t, s_m)
    if wp is not None:
        idx = wp["index"]
        start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span)
        window_sl = slice(start, idx)
        if window_sl.stop > window_sl.start:
            window_xy = (np.degrees(alpha_arr[window_sl]), Fy_arr[window_sl])
        cs_n_per_rad = wp["c_alpha"]
        if np.isfinite(cs_n_per_rad):
            x0 = np.degrees(alpha_arr[idx])
            y0 = Fy_arr[idx]
            slope_per_deg = cs_n_per_rad * (np.pi / 180.0)
            span = max(0.5, 0.15 * np.degrees(max_abs_alpha)) if max_abs_alpha > 0 else 1.0
            xs = np.array([x0 - span, x0 + span])
            ys = y0 + slope_per_deg * (xs - x0)
            tangent_line = (xs, ys, f"Tangent CS={cs_n_per_rad:.0f} N/rad")

    return {
        "session_xy": session_xy, "corner_xy": corner_xy, "window_xy": window_xy,
        "linear_ref_line": ref_line, "fitted_line": None, "tangent_line": tangent_line,
    }


def make_corner_figure(cid, source, state, t, s_m, v_kmh, slip, forces, cs, params,
                        laps_by_number, corners_by_stable_id, bg_mask, bg_xy, out_dir):
    instances_all = corners_by_stable_id[cid]
    instances = _valid_lap_instances(instances_all, laps_by_number)
    if not instances:
        return None

    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]
    cls_cfg = params["classification"]
    thresholds = {
        "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
        "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
        "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
    }

    alpha_f, alpha_r = slip["alpha_f_filt"], slip["alpha_r_filt"]
    Fy_f, Fy_r = forces["Fy_f_filt"], forces["Fy_r_filt"]
    kerb_mask = state.get("kerb_mask")

    laps = _build_laps(instances, laps_by_number, t, s_m, v_kmh, cs, thresholds)
    track_map = _build_track_map(cid, instances, laps_by_number, t, s_m, bg_xy, cs,
                                  alpha_f, alpha_r, min_window, min_span)
    tyre_curves = {
        "front": _build_tyre_curve("front", alpha_f, Fy_f, cs.get("C_linear_ref_f"),
                                    cs["CS_ratio_f"], cs["C_alpha_f"], bg_mask,
                                    instances, laps_by_number, t, s_m, min_window, min_span, kerb_mask),
        "rear": _build_tyre_curve("rear", alpha_r, Fy_r, cs.get("C_linear_ref_r"),
                                   cs["CS_ratio_r"], cs["C_alpha_r"], bg_mask,
                                   instances, laps_by_number, t, s_m, min_window, min_span, kerb_mask),
    }

    fig = figure_render.render_corner_figure(
        f"C{cid} ({source})", laps, thresholds, tyre_curves, track_map, theme=plot_style.PRINT,
    )
    out_path = os.path.join(out_dir, f"C{cid:02d}_{source}.png")
    figure_render.save_png(fig, out_path)
    return out_path


# --- main ---------------------------------------------------------------

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    data = parse_csv(RAW_FILE)
    params = load_parameters()
    state = prepare_vehicle_state(data["channels"], params)

    t = state["time"]
    s_m = state["s_m"]
    v_kmh = state["v_mps"] * 3.6
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
    racing_mask = np.zeros_like(t, dtype=bool)
    for s, e in valid_windows:
        racing_mask |= (t >= s) & (t <= e)
    bg_mask = moving & racing_mask

    corners = data.get("corners", [])
    corners_by_stable_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    stable_ids = sorted(corners_by_stable_id)

    gps_lat = state.get("gps_lat")
    gps_lon = state.get("gps_lon")
    origin_lat = state.get("gps_origin_lat")
    origin_lon = state.get("gps_origin_lon")
    if gps_lat is None or gps_lon is None:
        raise RuntimeError("GPS lat/lon unavailable in state -- cannot build track map")
    bg_x, bg_y = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)
    bg_xy = (bg_x, bg_y)

    forces = estimate_lateral_forces(state, params)

    written = []
    for source in SOURCES:
        beta = _compute_beta(state, params, source)
        slip = estimate_slip_angles(state, beta, params)
        cs = estimate_cornering_stiffness(slip, forces, state, params)
        for cid in stable_ids:
            path = make_corner_figure(cid, source, state, t, s_m, v_kmh, slip, forces, cs, params,
                                       laps_by_number, corners_by_stable_id, bg_mask, bg_xy, OUT_DIR)
            if path:
                written.append(path)
                print(f"wrote {path}")

    print(f"\n{len(written)} figures written to {OUT_DIR} "
          f"({len(stable_ids)} corners x {len(SOURCES)} sources)")


if __name__ == "__main__":
    main()
