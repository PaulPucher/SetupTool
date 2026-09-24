# v3 work package: (1) fraction of cornering-window samples failing the
# CS_ratio validity floors (min_span/max_window_m), v3 vs Dubai; (2) C13's
# CS trace re-rendered under the KINEMATIC sideslip source (no Pacejka/EKF
# fit involved at all) -- if the same jagged/oscillating pattern persists
# under kinematic, the cause cannot be the Pacejka fit or the EKF (they
# don't exist in that computation path), narrowing the hypothesis space to
# the CS windowing/estimation pipeline itself or the raw channel data.
# Read-only, no config/production changes. Disposable per CLAUDE.md's
# diagnostics/ rule.

import os

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, resolve_cs_min_window_samples,
)
from modules.geo import project_latlon_to_xy
from core import figure_render, plot_style
from diagnostics.inspect_step2_chair_plots import (
    _build_laps, _build_track_map, _build_tyre_curve, _valid_lap_instances,
)

OUT_DIR = "diagnostics/plots_v3"


def cs_validity_floor_report(label, raw_file, params):
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    assert state is not None

    beta = estimate_sideslip(state, params)  # kinematic -- no fit dependency
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)

    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
    attempted = moving.copy()
    attempted[:min_window] = False  # structural startup gap, not a floor failure

    n_attempted = int(attempted.sum())
    for axle, key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
        cs_ratio = cs[key]
        failed = attempted & ~(cs_ratio == cs_ratio)  # NaN despite being attempted
        frac = float(failed.sum()) / n_attempted if n_attempted else float("nan")
        print(f"[{label}] {axle}: {failed.sum()}/{n_attempted} attempted moving samples "
              f"failed the validity floor ({frac:.4f})")

    return data, state, slip, forces, cs


def render_c13_kinematic(data, state, slip, forces, cs, params, out_dir):
    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    corners = data.get("corners", [])
    corners_by_stable_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)

    cid = 13
    if cid not in corners_by_stable_id:
        print(f"stable_corner_id {cid} not present in this session's corners -- skipping")
        return None
    instances = _valid_lap_instances(corners_by_stable_id[cid], laps_by_number)
    if not instances:
        print(f"C{cid} has no valid-lap instances -- skipping")
        return None

    t, s_m = state["time"], state["s_m"]
    v_kmh = state["v_mps"] * 3.6
    se = params["stability_estimation"]
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
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

    moving = state["moving_mask"]
    racing_mask = moving.copy()
    if kerb_mask is not None:
        racing_mask = racing_mask & ~kerb_mask

    gps_lat, gps_lon = state.get("gps_lat"), state.get("gps_lon")
    origin_lat, origin_lon = state.get("gps_origin_lat"), state.get("gps_origin_lon")
    bg_x, bg_y = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)
    bg_xy = (bg_x, bg_y)

    laps_data = _build_laps(instances, laps_by_number, t, s_m, v_kmh, cs, thresholds)
    track_map = _build_track_map(cid, instances, laps_by_number, t, s_m, bg_xy, cs,
                                  alpha_f, alpha_r, min_window, min_span)
    tyre_curves = {
        "front": _build_tyre_curve("front", alpha_f, Fy_f, cs.get("C_linear_ref_f"),
                                    cs["CS_ratio_f"], cs["C_alpha_f"], racing_mask,
                                    instances, laps_by_number, t, s_m, min_window, min_span, kerb_mask),
        "rear": _build_tyre_curve("rear", alpha_r, Fy_r, cs.get("C_linear_ref_r"),
                                   cs["CS_ratio_r"], cs["C_alpha_r"], racing_mask,
                                   instances, laps_by_number, t, s_m, min_window, min_span, kerb_mask),
    }

    fig = figure_render.render_corner_figure(
        f"C{cid} (GT3_PRC_MLA-v3, KINEMATIC)", laps_data, thresholds, tyre_curves, track_map,
        theme=plot_style.PRINT,
    )
    out_path = os.path.join(out_dir, f"C{cid:02d}_v3_kinematic.png")
    figure_render.save_png(fig, out_path)
    return out_path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    params = load_parameters()

    print("=== CS validity floor failure rate ===")
    _, _, _, _, _ = cs_validity_floor_report(
        "Dubai", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt", params)
    data, state, slip, forces, cs = cs_validity_floor_report(
        "v3", "GT3_PRC_MLA-v3.txt", params)

    print("\n=== C13 CS trace under kinematic (no fit/EKF dependency) ===")
    path = render_c13_kinematic(data, state, slip, forces, cs, params, OUT_DIR)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
