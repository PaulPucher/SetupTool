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
# The only local logic is the window-boundary reconstruction below
# (_reconstruct_cs_window), needed because estimate_cornering_stiffness
# returns only per-sample C_alpha values, not the regression window
# bounds behind each one; same reconstruction approach the (since-
# deleted) kerb-audit diagnostic used, verified there against
# production's own C_window_f/r to 1e-6 relative tolerance.
#
# No production file changed. No config value changed. sideslip_source
# is never read from or written to config/parameters.json -- kinematic
# and ekf_pass_1 beta are obtained by calling estimate_sideslip and
# estimate_sideslip_ekf_dugoff directly, bypassing the config-driven
# dispatch entirely.

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff
from modules.geo import project_latlon_to_xy

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUT_DIR = "diagnostics/plots_step2"
SOURCES = ("kinematic", "ekf_pass_1")
DEG_PER_RAD = 180.0 / np.pi
LAP_COLORS = plt.get_cmap("tab10")


# --- pipeline helpers --------------------------------------------------

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


def _reconstruct_cs_window(alpha, i, min_window, min_span):
    # Mirrors the window-growth loop inside estimate_cornering_stiffness's
    # compute_cs_for_axle (modules/stability_analysis.py) exactly, for one
    # target index -- reconstruction only, not a reimplementation of the
    # estimator (the CS value itself always comes from the production
    # function's own return array).
    start = i - min_window
    while start > 0:
        span = np.max(alpha[start:i]) - np.min(alpha[start:i])
        if span >= min_span:
            break
        start -= 1
    return max(start, 0)


def _valid_lap_instances(instances, laps_by_number):
    out = []
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is not None and lap.get("is_valid_for_analysis"):
            out.append(c)
    return out


def _find_worst_phase(alpha, cs_ratio, c_alpha, instances, laps_by_number, t, s_m):
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


# --- plotting helpers ----------------------------------------------------

def _plot_velocity(ax, instances, laps_by_number, t, s_m, v_kmh):
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        color = LAP_COLORS(c["lap_number"] % 10)
        ax.plot(s_m[sl], v_kmh[sl], color=color, label=f"lap {c['lap_number']}")
    ax.set_ylabel("velocity [km/h]")
    ax.set_xlabel("lap distance s [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=5, loc="lower center")


def _plot_cs(ax, instances, laps_by_number, t, s_m, c_alpha_f, c_alpha_r):
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        color = LAP_COLORS(c["lap_number"] % 10)
        ax.plot(s_m[sl], c_alpha_f[sl], color=color, linestyle="-",
                 label=f"lap {c['lap_number']} front")
        ax.plot(s_m[sl], c_alpha_r[sl], color=color, linestyle="--",
                 label=f"lap {c['lap_number']} rear")
    ax.axhline(0.0, color="grey", linewidth=0.8)
    ax.set_ylabel("instantaneous CS [N/rad]")
    ax.set_xlabel("lap distance s [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=5, loc="lower center")


def _plot_track_map(ax, bg_xy, bracket_xy, window_xy_f, window_xy_r, cid):
    ax.plot(bg_xy[0], bg_xy[1], color="lightgrey", linewidth=1.5, label="lap trace")
    if bracket_xy is not None:
        ax.plot(bracket_xy[0], bracket_xy[1], color="tab:blue", linewidth=3,
                 label=f"C{cid} bracket")
    if window_xy_f is not None:
        ax.plot(window_xy_f[0], window_xy_f[1], color="tab:red", linewidth=4,
                 label="front est. window")
    if window_xy_r is not None:
        ax.plot(window_xy_r[0], window_xy_r[1], color="tab:orange", linewidth=4,
                 linestyle=":", label="rear est. window")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(fontsize=6, loc="upper right")
    ax.set_title("track map")


def _plot_tyre_curve(ax, bg_alpha_deg, bg_Fy, corner_alpha_deg, corner_Fy,
                      window_alpha_deg, window_Fy, worst_phase, axle_label):
    ax.scatter(bg_alpha_deg, bg_Fy, s=2, color="lightgrey", alpha=0.3, label="session scatter")
    ax.scatter(corner_alpha_deg, corner_Fy, s=6, color="tab:blue", alpha=0.6, label="corner samples")
    if window_alpha_deg is not None and len(window_alpha_deg) > 0:
        ax.scatter(window_alpha_deg, window_Fy, s=18, color="tab:red",
                    edgecolor="black", linewidth=0.3, label="est. window")
    if worst_phase is not None:
        x0 = np.degrees(worst_phase["alpha"])
        y0 = worst_phase["Fy"]
        cs_n_per_rad = worst_phase["c_alpha"]
        slope_per_deg = cs_n_per_rad * (np.pi / 180.0)
        span = max(0.5, 1.5 * worst_phase.get("window_span_deg", 1.0))
        xs = np.array([x0 - span, x0 + span])
        ys = y0 + slope_per_deg * (xs - x0)
        ax.plot(xs, ys, color="black", linestyle="--", linewidth=1.5,
                 label=f"tangent CS={cs_n_per_rad:.0f} N/rad")
    ax.axhline(0.0, color="grey", linewidth=0.6)
    ax.axvline(0.0, color="grey", linewidth=0.6)
    ax.set_xlabel("slip angle [deg]")
    ax.set_ylabel("Fy [N]")
    ax.legend(fontsize=6, loc="best")
    ax.set_title(f"{axle_label} tyre curve")


# --- per-corner figure ------------------------------------------------

def make_corner_figure(cid, source, state, t, s_m, v_kmh, slip, forces, cs, params,
                        laps_by_number, corners_by_stable_id, bg_mask, bg_xy, out_dir):
    instances_all = corners_by_stable_id[cid]
    instances = _valid_lap_instances(instances_all, laps_by_number)
    if not instances:
        return None

    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]

    alpha_f = slip["alpha_f_filt"]
    alpha_r = slip["alpha_r_filt"]
    Fy_f = forces["Fy_f_filt"]
    Fy_r = forces["Fy_r_filt"]

    wp_f = _find_worst_phase(alpha_f, cs["CS_ratio_f"], cs["C_alpha_f"], instances, laps_by_number, t, s_m)
    wp_r = _find_worst_phase(alpha_r, cs["CS_ratio_r"], cs["C_alpha_r"], instances, laps_by_number, t, s_m)

    fig = plt.figure(figsize=(18, 14))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.6])
    ax_vel = fig.add_subplot(gs[0, :])
    ax_cs = fig.add_subplot(gs[1, :])
    ax_map = fig.add_subplot(gs[2, 0])
    ax_tyre_f = fig.add_subplot(gs[2, 1])
    ax_tyre_r = fig.add_subplot(gs[2, 2])

    _plot_velocity(ax_vel, instances, laps_by_number, t, s_m, v_kmh)
    _plot_cs(ax_cs, instances, laps_by_number, t, s_m, cs["C_alpha_f"], cs["C_alpha_r"])

    # representative lap for the track-map background: the corner's own
    # first valid instance's lap.
    rep_lap_number = instances[0]["lap_number"]
    rep_lap = laps_by_number[rep_lap_number]
    rep_sl = slice(int(np.searchsorted(t, rep_lap["start_time"], side="left")),
                    int(np.searchsorted(t, rep_lap["end_time"], side="right")))
    rep_xy = bg_xy[0][rep_sl], bg_xy[1][rep_sl]

    bracket_sl = _canonical_window_slice(t, s_m, rep_lap["start_time"], rep_lap["end_time"],
                                          instances[0]["bracket_start_m"], instances[0]["bracket_end_m"])
    bracket_xy = (bg_xy[0][bracket_sl], bg_xy[1][bracket_sl]) if bracket_sl.stop > bracket_sl.start else None

    def _window_xy(wp, alpha_arr):
        if wp is None:
            return None
        lap = laps_by_number[wp["lap_number"]]
        i = wp["index"]
        start = _reconstruct_cs_window(alpha_arr, i, min_window, min_span)
        window_sl = slice(start, i)
        if window_sl.stop <= window_sl.start:
            return None
        return bg_xy[0][window_sl], bg_xy[1][window_sl], window_sl

    win_f = _window_xy(wp_f, alpha_f)
    win_r = _window_xy(wp_r, alpha_r)
    window_xy_f = (win_f[0], win_f[1]) if win_f else None
    window_xy_r = (win_r[0], win_r[1]) if win_r else None

    _plot_track_map(ax_map, rep_xy, bracket_xy, window_xy_f, window_xy_r, cid)

    # corner samples pooled across all valid lap instances, for the tyre-curve highlight
    corner_idx_list = []
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            corner_idx_list.append(sl)

    def _tyre_panel(ax, alpha_arr, Fy_arr, wp, window_slice, axle_label):
        bg_alpha_deg = np.degrees(alpha_arr[bg_mask])
        bg_Fy = Fy_arr[bg_mask]
        corner_alpha = np.concatenate([alpha_arr[sl] for sl in corner_idx_list]) if corner_idx_list else np.array([])
        corner_Fy = np.concatenate([Fy_arr[sl] for sl in corner_idx_list]) if corner_idx_list else np.array([])
        corner_alpha_deg = np.degrees(corner_alpha)

        window_alpha_deg = window_Fy = None
        worst_phase = None
        if wp is not None and window_slice is not None:
            w_alpha = alpha_arr[window_slice]
            w_Fy = Fy_arr[window_slice]
            window_alpha_deg = np.degrees(w_alpha)
            window_Fy = w_Fy
            i = wp["index"]
            span_deg = float(np.degrees(np.max(w_alpha) - np.min(w_alpha))) if len(w_alpha) else 1.0
            worst_phase = {"alpha": alpha_arr[i], "Fy": Fy_arr[i], "c_alpha": wp["c_alpha"],
                           "window_span_deg": span_deg}

        _plot_tyre_curve(ax, bg_alpha_deg, bg_Fy, corner_alpha_deg, corner_Fy,
                          window_alpha_deg, window_Fy, worst_phase, axle_label)

    _tyre_panel(ax_tyre_f, alpha_f, Fy_f, wp_f, win_f[2] if win_f else None, "front")
    _tyre_panel(ax_tyre_r, alpha_r, Fy_r, wp_r, win_r[2] if win_r else None, "rear")

    cs_f_txt = f"{wp_f['c_alpha']:.0f} N/rad" if wp_f else "n/a"
    cs_r_txt = f"{wp_r['c_alpha']:.0f} N/rad" if wp_r else "n/a"
    fig.suptitle(
        f"C{cid} -- sideslip_source={source} -- worst-phase CS front={cs_f_txt}, rear={cs_r_txt}\n"
        f"slip angle axes are in degrees; tangent slope is CS[N/rad] * pi/180 -> N/deg, "
        f"so it stays numerically comparable to the chair's radians plots.",
        fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    out_path = os.path.join(out_dir, f"C{cid:02d}_{source}.png")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
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
