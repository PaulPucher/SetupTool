# Investigation, diagnostics only: why does worst-lap CS_ratio go
# wholesale-negative under ekf_auto_pacejka (12/14 corners CSf-negative,
# contradicting Dubai's own driver ground truth of a balanced car with
# mild understeer)? Working hypothesis under test: window-regression
# artifact from ekf_auto_pacejka's wider alpha range, not real
# saturation. Goal is mechanism identification -- no config or
# production change, no new threshold, no commit. Targets: C1, C2, C3
# (newly extreme under ekf_auto_pacejka), C4 (STEP 2's genuine case),
# C8 (stability sign flip), C9 (STEP 2's old artifact case).
#
# Item 1 (renders): reuses diagnostics/inspect_step2_chair_plots.py's
# renderer (make_corner_figure et al) unmodified -- same production
# figure_render.render_corner_figure call, same fig_data assembly --
# but with beta obtained through the PRODUCTION dispatch (modules.
# tyre_fit_auto.resolve_sideslip_beta), fallback-guarded, instead of
# that script's own direct estimate_sideslip/estimate_sideslip_ekf_
# dugoff calls (kinematic vs ekf_pass_1 is a different comparison to
# this one, kinematic vs ekf_auto_pacejka).
#
# Items 2-4 (numeric mechanism checks) are new, local to this script.

import os

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.geo import project_latlon_to_xy
from diagnostics.inspect_step2_chair_plots import (
    _valid_lap_instances, _find_worst_phase, _canonical_window_slice, make_corner_figure,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
OUT_DIR = "diagnostics/plots_threshold_investigation"
MODES = ("kinematic", "ekf_auto_pacejka")
TARGET_IDS = (1, 2, 3, 4, 8, 9)


def _compute_beta(state, params, data, mode):
    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, mode, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{mode} fell back to kinematic ({fallback_reason}) -- refusing to investigate "
                          "a silently-kinematic distribution")
    return beta


def _sign_stability(c_alpha_arr, idx, moving, radius=10):
    # Sign-flip count of C_alpha in a +-radius neighbourhood around the
    # worst-phase index, over finite/moving samples only.
    lo = max(0, idx - radius)
    hi = min(len(c_alpha_arr), idx + radius + 1)
    signs = []
    for i in range(lo, hi):
        if not moving[i] or not np.isfinite(c_alpha_arr[i]):
            continue
        signs.append(np.sign(c_alpha_arr[i]))
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b and a != 0 and b != 0)
    return {"n_samples": len(signs), "flips": flips, "signs": signs}


def _window_report(axle_label, cs_ratio_arr, c_alpha_arr, c_window_arr, c_section_arr, r2_arr,
                    alpha_arr, instances, laps_by_number, t, s_m, min_window, min_span, moving):
    wp = _find_worst_phase(cs_ratio_arr, c_alpha_arr, instances, laps_by_number, t, s_m)
    if wp is None:
        print(f"    {axle_label}: no finite worst-phase sample found")
        return
    idx = wp["index"]
    start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span)
    window_alpha = alpha_arr[start:idx]
    n_samples = len(window_alpha)
    span_rad = float(np.max(window_alpha) - np.min(window_alpha)) if n_samples else float("nan")
    r2 = float(r2_arr[idx]) if np.isfinite(r2_arr[idx]) else float("nan")
    c_w = float(c_window_arr[idx]) if np.isfinite(c_window_arr[idx]) else float("nan")
    c_s = float(c_section_arr[idx]) if np.isfinite(c_section_arr[idx]) else float("nan")
    stab = _sign_stability(c_alpha_arr, idx, moving)
    print(f"    {axle_label}: lap={wp['lap_number']} idx={idx} CS_ratio={wp['cs_ratio']:.3f} "
          f"C_alpha={wp['c_alpha']:.0f} N/rad")
    print(f"      window: n_samples={n_samples}, span={span_rad:.4f} rad ({np.degrees(span_rad):.2f} deg), R2={r2:.3f}")
    print(f"      C_window (raw regression)={c_w:.0f} N/rad, C_section (monotonic-section)={c_s:.0f} N/rad, "
          f"{'AGREE in sign' if (c_w * c_s > 0) else 'DISAGREE in sign' if np.isfinite(c_w) and np.isfinite(c_s) else 'n/a'}")
    print(f"      sign stability (+-10 samples): {stab['flips']} flip(s) across {stab['n_samples']} samples, "
          f"signs={[int(s) for s in stab['signs']]}")


def _phase_lag_report(axle_label, alpha_arr, Fy_arr, instances, laps_by_number, t, s_m, sample_rate_hz):
    # Representative (first) valid lap instance's own canonical bracket --
    # a single temporally-contiguous slice, not a cross-lap pooled/
    # concatenated series (which would inject artificial discontinuities
    # into a cross-correlation).
    c = instances[0]
    lap = laps_by_number[c["lap_number"]]
    sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                  c["bracket_start_m"], c["bracket_end_m"])
    a = alpha_arr[sl]
    f = Fy_arr[sl]
    valid = np.isfinite(a) & np.isfinite(f)
    a, f = a[valid], f[valid]
    if len(a) < 8:
        print(f"    {axle_label}: too few samples ({len(a)}) in representative bracket for cross-correlation")
        return
    a0 = a - np.mean(a)
    f0 = f - np.mean(f)
    corr = np.correlate(a0, f0, mode="full")
    lag_samples = int(np.argmax(np.abs(corr))) - (len(a0) - 1)
    lag_ms = lag_samples / sample_rate_hz * 1000.0
    peak_r = corr[np.argmax(np.abs(corr))] / (np.linalg.norm(a0) * np.linalg.norm(f0) + 1e-12)
    print(f"    {axle_label}: lap={c['lap_number']} n={len(a)} peak |corr| lag = {lag_samples} samples "
          f"({lag_ms:+.1f} ms), normalised peak r={peak_r:.3f}")


def _min_vs_median_report(axle_label, cs_ratio_arr, instances, laps_by_number, t, s_m):
    pooled = []
    for c in instances:
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        seg = cs_ratio_arr[sl]
        finite = seg[np.isfinite(seg)]
        if finite.size:
            pooled.append(finite)
    if not pooled:
        print(f"    {axle_label}: no finite pooled samples")
        return
    arr = np.concatenate(pooled)
    print(f"    {axle_label}: n={len(arr)} pooled samples across bracket(s) -- "
          f"min={arr.min():.3f}, p10={np.percentile(arr, 10):.3f}, median={np.median(arr):.3f}, "
          f"p90={np.percentile(arr, 90):.3f}, max={arr.max():.3f}")
    frac_negative = float(np.mean(arr < 0))
    print(f"      fraction of pooled samples negative: {frac_negative:.1%}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    data = parse_csv(RAW_FILE)
    params = load_parameters()
    state = prepare_vehicle_state(data["channels"], params)

    t = state["time"]
    s_m = state["s_m"]
    v_kmh = state["v_mps"] * 3.6
    sample_rate_hz = state["sample_rate_hz"]
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
    bg_x, bg_y = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)
    bg_xy = (bg_x, bg_y)

    forces = estimate_lateral_forces(state, params)
    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]

    written = []
    per_mode = {}
    for mode in MODES:
        print(f"\n{'=' * 78}\nMODE: {mode}\n{'=' * 78}")
        beta = _compute_beta(state, params, data, mode)
        slip = estimate_slip_angles(state, beta, params)
        cs = estimate_cornering_stiffness(slip, forces, state, params)
        per_mode[mode] = (slip, cs)

        for cid in TARGET_IDS:
            instances_all = corners_by_stable_id.get(cid, [])
            instances = _valid_lap_instances(instances_all, laps_by_number)
            if not instances:
                print(f"\n-- C{cid}: no valid lap instances --")
                continue

            print(f"\n-- C{cid} ({mode}) --")
            path = make_corner_figure(cid, mode, state, t, s_m, v_kmh, slip, forces, cs, params,
                                       laps_by_number, corners_by_stable_id, bg_mask, bg_xy, OUT_DIR)
            if path:
                written.append(path)
                print(f"  wrote {path}")

            print("  item 2 -- worst-window report:")
            _window_report("front", cs["CS_ratio_f"], cs["C_alpha_f"], cs["C_window_f"], cs["C_section_f"],
                            cs["R2_f"], slip["alpha_f_filt"], instances, laps_by_number, t, s_m,
                            min_window, min_span, moving)
            _window_report("rear", cs["CS_ratio_r"], cs["C_alpha_r"], cs["C_window_r"], cs["C_section_r"],
                            cs["R2_r"], slip["alpha_r_filt"], instances, laps_by_number, t, s_m,
                            min_window, min_span, moving)

            print("  item 3 -- phase-lag report (alpha vs Fy, representative lap bracket):")
            _phase_lag_report("front", slip["alpha_f_filt"], forces["Fy_f_filt"], instances,
                               laps_by_number, t, s_m, sample_rate_hz)
            _phase_lag_report("rear", slip["alpha_r_filt"], forces["Fy_r_filt"], instances,
                               laps_by_number, t, s_m, sample_rate_hz)

            print("  item 4 -- min vs median (pooled per-sample CS_ratio across bracket):")
            _min_vs_median_report("front", cs["CS_ratio_f"], instances, laps_by_number, t, s_m)
            _min_vs_median_report("rear", cs["CS_ratio_r"], instances, laps_by_number, t, s_m)

    print(f"\n{len(written)} figures written to {OUT_DIR} ({len(TARGET_IDS)} corners x {len(MODES)} modes)")


if __name__ == "__main__":
    main()
