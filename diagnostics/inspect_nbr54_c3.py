# NBR 54 corner 3 turn-in, front (author EXECUTE 2026-10-20, read-only): the
# one corner / phase cell that holds normalised CS ratios below the -0.53
# physics floor on 4 of 6 analysis laps. Shipped fit-L chain, CS on Fy / Fz
# with the reference band L_K_EDGE.
#
# Per lap, turn-in phase of corner 3, verdict population (canonical brackets,
# base mask, moving, not kerb, not kerb-affected, not observer-reset-affected):
# samples below -0.53; kerb (kerb mask / smear zone inside the phase,
# kerb-affected samples, distance of the low samples to the nearest kerb
# sample on the lap); front longitudinal utilisation |Fx_f| / Fz_f and the
# braking share (Fx_f < 0) at the low samples; speed, steering-wheel angle,
# ay and front slip angle through the phase; line consistency (GPS lateral
# deviation from the mean line of the six laps at equal lap distance;
# curvature yaw rate / speed); front Fz range inside the low samples' windows.
# Figure: the six laps overlaid through corner 3 -- CS ratio front, steering,
# speed, front longitudinal utilisation -- low samples marked.
# Output diagnostics/plots/cs_norm/ (nbr54_c3_run.log, nbr54_c3.json, PNG).
# Run: python -m diagnostics.inspect_nbr54_c3

import copy
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "cs_norm")
KEY, CORNER, PHASE = "nbr_o54_cdr", "3", "entry_2_turnin"
FLOOR = -0.53
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
LAP_COLOURS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")


def main():
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from diagnostics.inspect_fit_lit import keep_awake, _jsonable
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            reconstruct_cs_window_start, resolve_cs_min_window_samples)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.geo import project_latlon_to_xy
    os.makedirs(OUT_DIR, exist_ok=True)
    log_f = open(os.path.join(OUT_DIR, "nbr54_c3_run.log"), "w", encoding="utf-8")

    def log(msg=""):
        print(msg, flush=True)
        log_f.write(msg + "\n")
        log_f.flush()

    keep_awake(True)
    try:
        _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[KEY]
        path = resolve_data_path(src[1])
        data = parse_csv(path)
        reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
        p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                                 resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
        p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
        st = prepare_vehicle_state(data["channels"], p)
        held = {}
        o_fsp, o_load = tfa.fit_session_pacejka, tfa._axle_load_inputs

        def fsp(*a, **kw):
            held["raw"] = o_fsp(*a, **kw)
            return held["raw"]

        def load(*a, **kw):
            held["load_in"] = o_load(*a, **kw)
            return held["load_in"]

        tfa.fit_session_pacejka, tfa._axle_load_inputs = fsp, load
        try:
            beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, data, "ekf_auto_pacejka", csv_path=path)
        finally:
            tfa.fit_session_pacejka, tfa._axle_load_inputs = o_fsp, o_load
        raw, li = held["raw"], held["load_in"]
        slip = estimate_slip_angles(st, beta, p)
        forces = estimate_lateral_forces(st, p)
        reset = None if fb else raw["observer_reset_mask"]
        cs = estimate_cornering_stiffness(slip, forces, st, p, data["laps"], reset_mask=reset,
                                          axle_loads=raw["cs_axle_loads"], ref_band=tfa.L_K_EDGE)
        t, s_m, sr = st["time"], st["s_m"], st["sample_rate_hz"]
        lap_of = {l["lap_number"]: l for l in data["laps"]}
        an = [l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")]
        in_br = np.zeros(t.size, dtype=bool)
        for c in data["corners"]:
            lap = lap_of.get(c.get("lap_number"))
            if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
                continue
            if c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
                in_br[tfa._canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                                  c["bracket_start_m"], c["bracket_end_m"])] = True
        kerb = st["kerb_mask"]
        kerb_any = kerb | st["kerb_smear_zone"]
        ratio = cs["CS_ratio_f"]
        pop = in_br & raw["base_mask"] & st["moving_mask"] & np.isfinite(ratio) & ~kerb & ~cs["near_strike_f"]
        if reset is not None:
            pop &= ~cs["observer_reset_affected_f"]
        fz, fx = li["fz_f_N"], li["fx_f_N"]
        util = np.abs(fx) / fz
        alpha_deg = np.degrees(slip["alpha_f_filt"])
        sw_deg = np.degrees(st["steer_sw_rad"])
        curv = st["yaw_rate_radps"] / np.maximum(st["v_mps"], 1.0)
        x, y = project_latlon_to_xy(st["gps_lat"], st["gps_lon"], st["gps_origin_lat"], st["gps_origin_lon"])
        mw = resolve_cs_min_window_samples(p, sr)
        se = p["stability_estimation"]
        inst = [c for c in data["corners"] if str(c.get("stable_corner_id")) == CORNER and c["lap_number"] in an]
        inst.sort(key=lambda c: c["lap_number"])
        # common distance range: corner 3 from the earliest entry to the latest exit, +/- 40 m
        spans = [(np.interp(c["segments"]["entry_1_brake"][0], t, s_m), np.interp(c["segments"]["exit_5"][1], t, s_m))
                 for c in inst]
        s_lo, s_hi = min(a for a, _ in spans) - 40.0, max(b for _, b in spans) + 40.0
        grid = np.arange(s_lo, s_hi, 1.0)
        laps_xy, per_lap, traces = {}, [], {}
        for c in inst:
            ln = c["lap_number"]
            lap = lap_of[ln]
            on_lap = (t >= lap["start_time"]) & (t <= lap["end_time"])
            span = on_lap & (s_m >= s_lo) & (s_m <= s_hi)
            idx = np.flatnonzero(span)
            o = np.argsort(s_m[idx])
            idx = idx[o]
            laps_xy[ln] = (np.interp(grid, s_m[idx], x[idx]), np.interp(grid, s_m[idx], y[idx]))
            traces[ln] = {"s": s_m[idx].tolist(), "cs": ratio[idx].tolist(), "sw": sw_deg[idx].tolist(),
                          "v": (st["v_mps"][idx] * 3.6).tolist(), "fxu": (fx[idx] / fz[idx]).tolist(),
                          "low": (pop[idx] & (ratio[idx] < FLOOR)).tolist()}
            a, b = c["segments"][PHASE]
            ph = (t >= a) & (t <= b)
            low = np.flatnonzero(pop & ph & (ratio < FLOOR))
            kerb_idx = np.flatnonzero(on_lap & kerb_any)
            rec = {"lap": ln, "n_phase_verdict": int((pop & ph).sum()), "n_low": int(low.size),
                   "phase_s_m": [float(np.interp(a, t, s_m)), float(np.interp(b, t, s_m))],
                   "kerb_mask_in_phase": int((kerb & ph).sum()), "smear_in_phase": int((st["kerb_smear_zone"] & ph).sum()),
                   "kerb_affected_in_phase": int((cs["near_strike_f"] & ph).sum()),
                   "speed_kmh_min_mean": [float(st["v_mps"][ph].min() * 3.6), float(st["v_mps"][ph].mean() * 3.6)],
                   "steer_sw_deg_max": float(np.abs(sw_deg[ph]).max()), "ay_mps2_max": float(np.abs(st["ay_mps2"][ph]).max()),
                   "alpha_f_deg_max": float(np.abs(alpha_deg[ph]).max()),
                   "curvature_max_per_m": float(np.abs(curv[ph]).max())}
            if low.size:
                d_kerb = (float(np.min(np.abs(s_m[kerb_idx][:, None] - s_m[low][None, :])))
                          if kerb_idx.size else float("nan"))
                fz_rng = []
                for i in low:
                    s0 = reconstruct_cs_window_start(slip["alpha_f_filt"], i, mw, se["cs_min_slip_angle_span_rad"],
                                                     s_m, se["cs_max_window_m"])
                    w = fz[s0:i]
                    fz_rng.append((w.max() - w.min()) / w.mean())
                rec.update({"low_s_m": [float(s_m[low].min()), float(s_m[low].max())],
                            "low_ratio_min": float(ratio[low].min()),
                            "kerb_distance_m": d_kerb, "util_median": float(np.median(util[low])),
                            "util_max": float(util[low].max()), "braking_share": float(np.mean(fx[low] < 0)),
                            "fz_window_range_median": float(np.median(fz_rng)),
                            "alpha_f_deg_at_low_median": float(np.median(np.abs(alpha_deg[low]))),
                            "steer_sw_deg_at_low_median": float(np.median(np.abs(sw_deg[low]))),
                            "speed_kmh_at_low_median": float(np.median(st["v_mps"][low] * 3.6))})
            per_lap.append(rec)
        mx = np.mean([v[0] for v in laps_xy.values()], axis=0)
        my = np.mean([v[1] for v in laps_xy.values()], axis=0)
        for rec in per_lap:
            lx, ly = laps_xy[rec["lap"]]
            a, b = rec["phase_s_m"]
            m = (grid >= a) & (grid <= b)
            dev = np.hypot(lx - mx, ly - my)[m]
            rec["line_dev_m_median_max"] = [float(np.median(dev)), float(dev.max())] if dev.size else None
        log(f"{label}: status {man.get('status')}, fallback {fb}; corner {CORNER} turn-in, front axle, CS on Fy / Fz")
        for rec in per_lap:
            log("-" * 100)
            log(f"lap {rec['lap']}: turn-in {rec['phase_s_m'][0]:.0f}-{rec['phase_s_m'][1]:.0f} m, verdict samples "
                f"{rec['n_phase_verdict']}, below -0.53: {rec['n_low']}")
            log(f"  kerb: mask {rec['kerb_mask_in_phase']}, smear zone {rec['smear_in_phase']}, kerb-affected "
                f"{rec['kerb_affected_in_phase']} samples in the phase")
            log(f"  phase: speed min / mean {rec['speed_kmh_min_mean'][0]:.0f} / {rec['speed_kmh_min_mean'][1]:.0f} km/h, "
                f"|steering wheel| max {rec['steer_sw_deg_max']:.0f} deg, |ay| max {rec['ay_mps2_max']:.1f} m/s^2, "
                f"|alpha_f| max {rec['alpha_f_deg_max']:.2f} deg, curvature max {rec['curvature_max_per_m']:.4f} / m; "
                f"line deviation from the mean line median / max {rec['line_dev_m_median_max']} m")
            if rec["n_low"]:
                log(f"  low samples at {rec['low_s_m'][0]:.0f}-{rec['low_s_m'][1]:.0f} m, min ratio {rec['low_ratio_min']:.2f}; "
                    f"nearest kerb sample {rec['kerb_distance_m']:.0f} m; |Fx_f| / Fz_f median {rec['util_median']:.3f} "
                    f"(max {rec['util_max']:.3f}), braking share {rec['braking_share']:.2f}; front Fz range in the "
                    f"windows median {rec['fz_window_range_median'] * 100:.1f} %; |alpha_f| median "
                    f"{rec['alpha_f_deg_at_low_median']:.2f} deg, |steering wheel| median "
                    f"{rec['steer_sw_deg_at_low_median']:.0f} deg, speed median {rec['speed_kmh_at_low_median']:.0f} km/h")
        with open(os.path.join(OUT_DIR, "nbr54_c3.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable(per_lap), fh, indent=1, default=str)
        log(f"figure: {draw(traces, per_lap, label)}")
        return 0
    finally:
        keep_awake(False)
        log_f.close()


def draw(traces, per_lap, label):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, muted = "#0b0b0b", "#52514e"
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, facecolor="#fcfcfb")
    rows = (("cs", "cornering stiffness ratio, front (-)"), ("sw", "steering-wheel angle (deg)"),
            ("v", "speed (km/h)"), ("fxu", "front Fx / Fz (-), negative = braking"))
    a = np.median([r["phase_s_m"][0] for r in per_lap])
    b = np.median([r["phase_s_m"][1] for r in per_lap])
    for k, (axp, (key, ylab)) in enumerate(zip(axes, rows)):
        axp.set_facecolor("#fcfcfb")
        axp.axvspan(a, b, color="#e6e6e3", lw=0, zorder=0)
        for j, (ln, tr) in enumerate(sorted(traces.items())):
            s, yv = np.asarray(tr["s"]), np.asarray(tr[key], dtype=float)
            col = LAP_COLOURS[j]
            axp.plot(s, yv, color=col, lw=1.3, label=f"lap {ln}", zorder=2)
            fin = np.isfinite(yv)
            if fin.any():
                axp.text(s[fin][-1] + 3, yv[fin][-1], f"lap {ln}", color=muted, fontsize=7, va="center")
            low = np.asarray(tr["low"], dtype=bool)
            if key == "cs" and low.any():
                axp.scatter(s[low], yv[low], s=22, facecolors="none", edgecolors=ink, linewidths=0.9, zorder=3)
        if key == "cs":
            axp.axhline(FLOOR, color=muted, lw=1.0, ls="--")
            axp.text(880, FLOOR - 0.16, "physics floor -0.53; open circles = samples below it (verdict population)",
                     color=muted, fontsize=8)
            axp.set_ylim(-1.6, 1.1)
        axp.set_ylabel(ylab, color=ink, fontsize=9)
        axp.grid(axis="y", color="#e4e4e0", lw=0.6)
        for sp in ("top", "right"):
            axp.spines[sp].set_visible(False)
    axes[0].text(a + 2, -1.45, "turn-in phase (shaded; median of the six laps)", color=muted, fontsize=8, va="bottom")
    axes[-1].set_xlabel("lap distance (m)", color=ink)
    axes[0].legend(loc="lower right", ncol=6, fontsize=8, frameon=False)
    fig.suptitle(f"{label}, corner 3, six analysis laps: front cornering stiffness ratio (load-normalised force) "
                 "with steering, speed and front longitudinal force share", color=ink, fontsize=11)
    fig.tight_layout(rect=(0, 0, 0.97, 0.96))
    out = os.path.join(OUT_DIR, "nbr54_corner3_six_laps_front_cs_steering_speed_fx.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


if __name__ == "__main__":
    sys.exit(main())
