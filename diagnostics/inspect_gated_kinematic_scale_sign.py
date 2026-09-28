# Gated-kinematic diagnostic, step 4 of 4: gyro scale error and sign.
# (a) Gyro scale error, measured on this data: over a closed racing lap the
#     true heading change is exactly +/-360 deg, so the integrated yaw rate's
#     closure residual is sensor error (bias + scale). Bias is taken as the
#     median yaw rate on straight samples and removed; what remains is read
#     as a scale error e. Because gated beta integrates -e*r inside each
#     gate run, the predicted in-corner beta error is -e * (heading change
#     of that run). Compared per corner against the gate-exit residual and
#     its lap-to-lap spread. (Record: ~6 deg/lap scale drift concentrated in
#     cornering, from the shelved GPS-course work.)
# (b) Metric 4: sign agreement and magnitude of mid-corner beta between
#     sources, per corner, highlighting Dubai C6/C10 -- the racing-speed
#     corners where the kinematic estimate and the linear Kalman observer
#     disagreed in sign (WP-S5).
# Read-only. Run from the repo root:
#   python -m diagnostics.inspect_gated_kinematic_scale_sign

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from diagnostics.gated_kinematic_common import (
    SESSIONS, OUT_DIR, load_cache, valid_corner_instances,
)
from diagnostics.inspect_gated_kinematic_drift import gate_run_residuals
from diagnostics.gated_kinematic_style import SERIES, TEXT, style_axes

STRAIGHT_AY_FRACTION = 0.25   # straight = |ay| below this share of the Otsu threshold
SIGN_SOURCES = ("kinematic_prod", "ekf_auto_pacejka", "gated_x1")
MIN_ABS_BETA_DEG = 0.1        # below this a "sign" is noise; excluded from agreement
HISTORIC_DISAGREE = {"dubai": (6, 10)}


def lap_closure(cache):
    st = cache["state"]
    t, r, ay = st["time"], st["yaw_rate_radps"], st["ay_mps2"]
    moving = st["moving_mask"]
    straight = cache["racing_mask"] & (np.abs(ay) < STRAIGHT_AY_FRACTION * cache["thr_otsu"])
    bias = float(np.median(r[straight]))
    sr = st["sample_rate_hz"]
    rows = []
    for lap in cache["laps"]:
        if not lap.get("is_valid_for_analysis"):
            continue
        sl = (t >= lap["start_time"]) & (t < lap["end_time"])
        raw = float(np.degrees(np.sum(r[sl]) / sr))
        corrected = float(np.degrees(np.sum((r[sl] - bias)[moving[sl]]) / sr))
        target = 360.0 * np.sign(corrected)
        rows.append({"lap": lap["lap_number"], "heading_raw_deg": raw,
                     "heading_bias_removed_deg": corrected,
                     "closure_residual_deg": corrected - target,
                     "scale_error": corrected / target - 1.0})
    return bias, rows


def mid_corner_beta(cache, inst):
    """{src: {(sid, lap): median beta over the middle third of the window}}"""
    out = {s: {} for s in SIGN_SOURCES}
    for sid, lap, sl in inst:
        n = sl.stop - sl.start
        mid = slice(sl.start + n // 3, sl.stop - n // 3)
        if mid.stop <= mid.start:
            continue
        for s in SIGN_SOURCES:
            out[s][(sid, lap)] = float(np.degrees(np.median(cache["betas"][s][mid])))
    return out


def main():
    results = {}
    for name in SESSIONS:
        try:
            cache = load_cache(name)
        except FileNotFoundError:
            print(f"[{name}] no cache")
            continue
        st = cache["state"]
        inst = valid_corner_instances({"laps": cache["laps"], "corners": cache["corners"]}, st["time"], st["s_m"])
        bias, laps = lap_closure(cache)
        e = float(np.median([l["scale_error"] for l in laps]))
        print(f"\n===== {name}: yaw-rate bias (straights) {np.degrees(bias):+.4f} deg/s")
        for l in laps:
            print(f"  lap {l['lap']}: heading raw {l['heading_raw_deg']:+.1f} deg, bias-removed "
                  f"{l['heading_bias_removed_deg']:+.1f} deg, closure residual {l['closure_residual_deg']:+.2f} deg, "
                  f"scale error {100 * l['scale_error']:+.2f} %")
        print(f"  session scale error (median of laps) e = {100 * e:+.2f} %")

        rows, _ = gate_run_residuals(cache, "gated_x1", inst)
        per_corner = {}
        for rrow in rows:
            if rrow["stable_id"] is None:
                continue
            per_corner.setdefault(rrow["stable_id"], []).append(rrow)
        print("  per corner (gated_x1): predicted in-gate error -e*dpsi vs observed gate-exit residual")
        corner_rows = []
        for sid in sorted(per_corner):
            rr = per_corner[sid]
            dpsi = np.array([x["heading_change_deg"] for x in rr])
            res = np.array([x["residual_deg"] for x in rr])
            pred = -e * dpsi
            corner_rows.append({"stable_id": sid, "n_runs": len(rr),
                                "heading_change_deg_mean": float(dpsi.mean()),
                                "predicted_error_deg_mean": float(pred.mean()),
                                "residual_deg_mean": float(res.mean()),
                                "residual_deg_std_across_laps": float(res.std(ddof=1)) if len(rr) > 1 else np.nan})
            c = corner_rows[-1]
            print(f"    C{sid}: runs {len(rr)}, dpsi {c['heading_change_deg_mean']:+.1f} deg, predicted scale error "
                  f"{c['predicted_error_deg_mean']:+.2f} deg, residual mean {c['residual_deg_mean']:+.2f} deg, "
                  f"lap spread (std) {c['residual_deg_std_across_laps']:.2f} deg")

        # Does the in-gate drift scale with heading change (a scale-type error
        # of the integrand ay/v - r) or with time (a bias-type error)? One
        # least-squares slope each, through the origin, over all runs.
        dp = np.radians([x["heading_change_deg"] for x in rows])
        du = np.array([x["duration_s"] for x in rows])
        rs = np.radians([x["residual_deg"] for x in rows])
        def _fit(x, y):
            k = float(np.dot(x, y) / np.dot(x, x))
            r2 = 1.0 - float(np.sum((y - k * x) ** 2) / np.sum((y - y.mean()) ** 2))
            return k, r2
        k_psi, r2_psi = _fit(dp, rs)
        k_t, r2_t = _fit(du * np.sign(dp), rs)
        print(f"  in-gate residual vs heading change: slope {k_psi:+.3f} rad/rad (R^2 {r2_psi:.2f}); "
              f"vs signed duration: slope {np.degrees(k_t):+.2f} deg/s (R^2 {r2_t:.2f}); "
              f"gyro-scale prediction would be slope {-e:+.4f} rad/rad")

        mids = mid_corner_beta(cache, inst)
        keys = sorted(mids["ekf_auto_pacejka"])
        def agree(a, b):
            pairs = [(mids[a][k], mids[b][k]) for k in keys
                     if abs(mids[a][k]) >= MIN_ABS_BETA_DEG and abs(mids[b][k]) >= MIN_ABS_BETA_DEG]
            if not pairs:
                return np.nan, 0
            return float(np.mean([np.sign(x) == np.sign(y) for x, y in pairs])), len(pairs)
        sign = {f"{a}_vs_{b}": agree(a, b) for a, b in (("gated_x1", "ekf_auto_pacejka"),
                                                          ("kinematic_prod", "ekf_auto_pacejka"),
                                                          ("gated_x1", "kinematic_prod"))}
        mags = {s: float(np.median([abs(mids[s][k]) for k in keys])) for s in SIGN_SOURCES}
        print(f"  mid-corner sign agreement (|beta| >= {MIN_ABS_BETA_DEG} deg both): " +
              "; ".join(f"{k} {v[0]:.2f} (n={v[1]})" for k, v in sign.items()))
        print("  median |mid-corner beta| deg: " + ", ".join(f"{s} {m:.2f}" for s, m in mags.items()))
        by_corner = {}
        for (sid, lap) in keys:
            by_corner.setdefault(sid, []).append(lap)
        corner_sign = {}
        for sid in sorted(by_corner):
            vals = {s: [mids[s][(sid, lap)] for lap in by_corner[sid]] for s in SIGN_SOURCES}
            corner_sign[sid] = {s: float(np.median(v)) for s, v in vals.items()}
            flag = " <- historic kinematic/observer sign disagreement" if sid in HISTORIC_DISAGREE.get(name, ()) else ""
            print(f"    C{sid}: median mid-corner beta deg " +
                  ", ".join(f"{s} {corner_sign[sid][s]:+.2f}" for s in SIGN_SOURCES) + flag)

        fig, ax = plt.subplots(figsize=(9, 4.6))
        sids = sorted(corner_sign)
        x = np.arange(len(sids))
        for off, s in zip((-0.2, 0.0, 0.2), SIGN_SOURCES):
            stl = SERIES[s]
            ax.plot(x + off, [corner_sign[c][s] for c in sids], linestyle="none", marker=stl["marker"],
                    markersize=6, color=stl["color"], label=stl["label"])
        ax.axhline(0.0, color=TEXT, linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([f"C{c}" for c in sids], fontsize=8)
        ax.set_ylabel("median mid-corner beta (deg)", fontsize=9)
        ax.set_title(f"{name}: mid-corner sideslip by source (median over valid laps)", fontsize=10, color=TEXT)
        style_axes(ax)
        ax.legend(fontsize=7, frameon=False)
        fig.tight_layout()
        path = os.path.join(OUT_DIR, f"mid_corner_beta_{name}.png")
        fig.savefig(path, dpi=130)
        plt.close(fig)
        print(f"  figure: {path}")
        results[name] = {"yaw_bias_degps": float(np.degrees(bias)), "laps": laps, "scale_error_median": e,
                         "corners_scale": corner_rows,
                         "residual_fit": {"slope_vs_heading_rad_per_rad": k_psi, "r2_heading": r2_psi,
                                          "slope_vs_signed_duration_deg_per_s": float(np.degrees(k_t)), "r2_duration": r2_t},
                         "sign_agreement": {k: list(v) for k, v in sign.items()},
                         "median_abs_mid_beta_deg": mags,
                         "corner_mid_beta_deg": {str(k): v for k, v in corner_sign.items()}}
    with open(os.path.join(OUT_DIR, "scale_sign_results.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=float)


if __name__ == "__main__":
    main()
