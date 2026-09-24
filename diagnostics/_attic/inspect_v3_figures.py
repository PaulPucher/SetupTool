# v3 work package, Phase 1: one lap-trace figure and one corner figure
# (PRINT theme) for the presentation, GT3_PRC_MLA-v3.txt, ekf_auto_pacejka
# (live production default). Read-only, no config/production changes.
# Reuses core/figure_render.py's render_lap_figure/render_corner_figure
# unmodified (the same functions ui/views/corner_trace_dialog.py's
# "Export figure" buttons call) and diagnostics/inspect_step2_chair_plots.
# py's corner-figure-building helpers (_build_laps/_build_track_map/
# _build_tyre_curve/_canonical_window_slice/_valid_lap_instances/
# _find_worst_phase) rather than re-deriving them -- [dependency] on that
# script, per diagnostics/README.md convention.

import os

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, estimate_yaw_moment_stability,
    resolve_cs_min_window_samples,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.geo import project_latlon_to_xy
from core import figure_render, plot_style
from ui.views.corner_trace_dialog import _lap_slice
from diagnostics.inspect_step2_chair_plots import (
    _build_laps, _build_track_map, _build_tyre_curve,
    _canonical_window_slice, _valid_lap_instances, _find_worst_phase,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_v3"


def build_lap_trace_figure(lap_number, state, cs, stab, laps_by_number, corners_by_stable_id,
                            worst_colour_by_id, out_dir):
    t = state["time"]
    s_m = state["s_m"]
    lap = laps_by_number[lap_number]
    clipped = _lap_slice(t, s_m, lap["start_time"], lap["end_time"])
    if clipped is None:
        return None
    lo, hi, lap_s, _lo_s, _hi_s = clipped
    sl = slice(lo, hi)
    order = np.argsort(lap_s)

    export_lap = {
        "lap_number": lap_number, "s": lap_s[order],
        "v_kmh": state["v_mps"][sl][order] * 3.6,
        "stab": stab["stability_observed_Nm_per_deg"][sl][order],
        "cs_f": cs["CS_ratio_f"][sl][order],
        "cs_r": cs["CS_ratio_r"][sl][order],
    }

    corner_bands = []
    for cid, instances in sorted(corners_by_stable_id.items()):
        rep = instances[0]
        start_s, end_s = rep.get("bracket_start_m"), rep.get("bracket_end_m")
        if start_s is None or end_s is None or end_s <= start_s:
            continue
        corner_bands.append((start_s, end_s, cid))

    cls_cfg = load_parameters()["classification"]
    thresholds = {
        "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
        "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
        "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
    }

    fig = figure_render.render_lap_figure(
        f"Lap {lap_number} (GT3_PRC_MLA-v3, ekf_auto_pacejka)",
        [export_lap], thresholds, corner_bands, theme=plot_style.PRINT,
    )
    out_path = os.path.join(out_dir, f"lap{lap_number}_trace.png")
    figure_render.save_png(fig, out_path)
    return out_path


def build_corner_figure(cid, state, slip, forces, cs, params, laps_by_number,
                         corners_by_stable_id, bg_mask, bg_xy, out_dir):
    instances_all = corners_by_stable_id[cid]
    instances = _valid_lap_instances(instances_all, laps_by_number)
    if not instances:
        return None

    se = params["stability_estimation"]
    # NOTE (v3 work package finding, 2026-09-02): cs_min_window_samples is
    # a stale key -- the CS validity repair rework (thesis_notes.md) made
    # this rate-derived via resolve_cs_min_window_samples(params, sample_
    # rate_hz), not a flat config literal. diagnostics/inspect_step2_
    # chair_plots.py still reads the old key directly and would raise this
    # exact KeyError if re-run today -- flagged, not fixed here (out of
    # this script's own scope; that script is [keep-reproduces] but has
    # not been re-run since the rework).
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
    min_span = se["cs_min_slip_angle_span_rad"]
    cls_cfg = params["classification"]
    thresholds = {
        "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
        "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
        "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
    }

    t, s_m = state["time"], state["s_m"]
    v_kmh = state["v_mps"] * 3.6
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
        f"C{cid} (GT3_PRC_MLA-v3, ekf_auto_pacejka)", laps, thresholds, tyre_curves, track_map,
        theme=plot_style.PRINT,
    )
    out_path = os.path.join(out_dir, f"C{cid:02d}_v3.png")
    figure_render.save_png(fig, out_path)
    return out_path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    data = parse_csv(RAW_FILE)
    params = load_parameters()
    state = prepare_vehicle_state(data["channels"], params)
    assert state is not None

    live_default = params["stability_estimation"].get("sideslip_source", "kinematic")
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, live_default, csv_path=RAW_FILE
    )
    print(f"fallback_used={fallback_used} fallback_reason={fallback_reason}")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    t = state["time"]
    s_m = state["s_m"]
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

    gps_lat, gps_lon = state.get("gps_lat"), state.get("gps_lon")
    origin_lat, origin_lon = state.get("gps_origin_lat"), state.get("gps_origin_lon")
    if gps_lat is None or gps_lon is None:
        raise RuntimeError("GPS lat/lon unavailable -- cannot build track map for corner figure")
    bg_x, bg_y = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)
    bg_xy = (bg_x, bg_y)

    fastest_lap = next(l["lap_number"] for l in laps if l.get("is_fastest"))
    print(f"fastest valid lap: {fastest_lap}")
    lap_path = build_lap_trace_figure(fastest_lap, state, cs, stab, laps_by_number,
                                       corners_by_stable_id, {}, OUT_DIR)
    print(f"wrote {lap_path}")

    # C13 (moderate oversteer @ apex + strong unstable yaw @ apex, the one
    # "strong"-severity corner this session's verdict summary showed) --
    # the most presentation-worthy single corner: both an axle-verdict
    # AND a yaw-instability finding at once.
    corner_path = build_corner_figure(13, state, slip, forces, cs, params, laps_by_number,
                                       corners_by_stable_id, bg_mask, bg_xy, OUT_DIR)
    print(f"wrote {corner_path}")


if __name__ == "__main__":
    main()
