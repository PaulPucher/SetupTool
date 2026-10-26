# Derivation of tyre_fit_auto.mu_ceiling_edge_u (work order DEV-3 item 1,
# author ruling 2026-10-24): the p95 combined relative uncertainty of
# mu_demo, pooled over the 14 development sessions.
#
# mu_demo is the p99.5 of |Fy / Fz| on an axle's fit population, so its
# error is the relative error of Fy / Fz on the samples at or above that
# percentile. Two input errors, independent, added in quadrature per sample:
#   (a) a_y: the IMU position relative to the CoG is unknown; a longitudinal
#       offset adds dx times the yaw acceleration. Recorded p95: 0.08 g per
#       metre of offset (PRODUCTION.md "IMU position unknown", 2 Hz filtered
#       yaw acceleration, NBR 29 and Le Castellet 25). Fy is the Level-1
#       split of m a_y, so the relative Fy error is 0.08 g * dx / |a_y|.
#   (b) Fz: the event zero entry's per-wheel uncertainty u_wheel_N
#       (channels.json event_zero_entries), per axle sqrt(u_l^2 + u_r^2),
#       relative to the axle load at the sample. Only where the zero entry
#       was applied; a file without one carries no recorded zero
#       uncertainty, so (b) is 0 there (stated, not estimated).
# The production fit is reused, not re-implemented: _fit_axle_l is wrapped
# to capture its inputs and the run stops before the observer.
# Output diagnostics/results_mu_uncertainty_20261024/.
# Run: python -m diagnostics.inspect_mu_uncertainty

import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from diagnostics.inspect_cs_margin_l import CANDIDATES  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_mu_uncertainty_20261024")
AY_ERR_G_PER_M = 0.08    # PRODUCTION.md "IMU position unknown", p95
IMU_OFFSET_M = 1.0       # assumption, author's example value (DEV-3); no recorded mounting position
POOL_PCT = 95.0


class _Stop(Exception):
    pass


def capture(data, params):
    from modules import tyre_fit_auto as tfa
    got = []
    orig = tfa._fit_axle_l

    def wrapped(alpha, Fy, Fz, base_mask, bin_rad, mu_pct, weighted=True):
        r = orig(alpha, Fy, Fz, base_mask, bin_rad, mu_pct, weighted)
        got.append({"Fy": Fy, "Fz": Fz, "mask": r["residual_mask"], "mu": r["mu"]})
        if len(got) == 2:
            raise _Stop
        return r
    tfa._fit_axle_l = wrapped
    try:
        tfa.fit_session_pacejka(data, params)
    except _Stop:
        pass
    finally:
        tfa._fit_axle_l = orig
    return got


def session(path):
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.corner_analysis import reclassify_speed_classes
    from modules.csv_parser import load_channels_config, parse_csv
    from modules.stability_analysis import _butterworth_lowpass, load_parameters, prepare_vehicle_state
    params = load_parameters()
    p = apply_resolved_vehicle(params, resolve_accuracy(params, setup_data=None, cap=None))
    data = parse_csv(path)
    cfg = load_channels_config()
    reclassify_speed_classes(data.get("corners", []), cfg["corner_speed_thresholds"])
    got = capture(data, p)
    state = prepare_vehicle_state(data["channels"], p)
    ay_g = np.abs(_butterworth_lowpass(state["ay_mps2"], p["stability_estimation"]["cs_filter_cutoff_hz"],
                                       state["sample_rate_hz"])) / 9.81
    zero = data.get("event_zero_entry") or {}
    entry = cfg["event_zero_entries"].get(zero.get("event") or "", {}).get("channels", {})
    u = {c: (entry[f"log_dms_dam_{c}"]["u_wheel_N"]
             if (zero.get("channels") or {}).get(f"log_dms_dam_{c}") == "applied" else 0.0)
         for c in ("fl", "fr", "rl", "rr")}
    out = {"event_zero": zero.get("event"), "u_wheel_N": u}
    rel = []
    for (axle, (l, r)), g in zip((("front", ("fl", "fr")), ("rear", ("rl", "rr"))), got):
        y = np.abs(g["Fy"] / g["Fz"])
        hi = g["mask"] & (y >= g["mu"])
        ra = AY_ERR_G_PER_M * IMU_OFFSET_M / ay_g[hi]
        rf = np.hypot(u[l], u[r]) / g["Fz"][hi]
        rc = np.hypot(ra, rf)
        rel.append(rc)
        out[axle] = {"mu_demo": g["mu"], "n_high": int(hi.sum()),
                     "ay_g_median_high": float(np.median(ay_g[hi])),
                     "fz_axle_N_median_high": float(np.median(g["Fz"][hi])),
                     "rel_ay_p50_p95": [float(np.percentile(ra, 50)), float(np.percentile(ra, 95))],
                     "rel_fz_p50_p95": [float(np.percentile(rf, 50)), float(np.percentile(rf, 95))],
                     "rel_combined_p50_p95": [float(np.percentile(rc, 50)), float(np.percentile(rc, 95))]}
    return out, rel


def main():
    from tests.conftest import resolve_data_path
    os.makedirs(OUT, exist_ok=True)
    res, pool, pool_no_b = {}, [], []
    for c in CANDIDATES:
        out, rel = session(resolve_data_path(c[4][1]))
        res[c[0]] = dict(out, label=c[3])
        pool += rel
        print(c[0], json.dumps({k: out[k] for k in ("front", "rear")}), flush=True)
    allv = np.concatenate(pool)
    u_mu = float(np.percentile(allv, POOL_PCT))
    summary = {"u_mu_rel_p95": u_mu, "n_samples": int(allv.size),
               "pool_p50": float(np.percentile(allv, 50)), "pool_p99": float(np.percentile(allv, 99)),
               "inputs": {"ay_err_g_per_m": AY_ERR_G_PER_M, "imu_offset_m": IMU_OFFSET_M, "pool_pct": POOL_PCT},
               "mu_edge_if_relative (2.0 * (1 + u))": 2.0 * (1 + u_mu),
               "mu_edge_if_absolute (2.0 + u)": 2.0 + u_mu,
               "sessions": res}
    with open(os.path.join(OUT, "mu_uncertainty.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != "sessions"}, indent=1))


if __name__ == "__main__":
    main()
