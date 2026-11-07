# ORDER SPEED-1b item 2, read-only: is the R3 straight-line median a sensor
# offset or real curvature? Runs in the R1-only scratchpad copy (R3 off).
# Run: python r3_check.py <copy root> <out json>
import json
import os
import sys

import numpy as np

OUTINGS = {"NBR1": 17, "NBR2": 18, "NBR3": 19, "NBR4": 20, "NBR5": 21,
           "PRC1": 22, "PRC2": 25, "PRC3": 26, "PRC4": 27, "PRC5": 28}
CORNERS = ("fl", "fr", "rl", "rr")
# standstill on wheels: rule of diagnostics/inspect_prc24996_zero.py, unchanged
STANDSTILL_KMH, ON_WHEELS_TRAVEL_MM, MIN_STANDSTILL_S, PIT_KMH = 1.0, 5.0, 1.0, (5.0, 70.0)


def _imports(root):
    sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "tools", "thesis_figures"))
    import common  # noqa
    import modules
    assert os.path.normcase(os.path.dirname(list(modules.__path__)[0])) == os.path.normcase(os.path.normpath(root))
    return common


def stats(x):
    x = x[np.isfinite(x)]
    if x.size == 0:
        return None
    return {"n": int(x.size), "median": float(np.median(x)), "mean": float(np.mean(x)),
            "p10": float(np.percentile(x, 10)), "p90": float(np.percentile(x, 90)),
            "robust_sd": float(1.4826 * np.median(np.abs(x - np.median(x))))}


def session(job):
    root, key = job
    common = _imports(root)
    from modules.loads import _normalize_travel_to_mm
    from modules.observer import steering_straights
    from modules.stability_analysis import prepare_vehicle_state
    sess = common.load_session(OUTINGS[key])
    parsed, p = sess["parsed"], sess["params"]
    assert "sensor_offsets" not in (parsed.get("signal_corrections") or {})
    assert (parsed.get("signal_corrections") or {}).get("speed_reference", {}).get("applied")
    ch = parsed["channels"]
    st = prepare_vehicle_state(ch, p)
    t, sr, v = st["time"], st["sample_rate_hz"], st["v_mps"]
    r_deg = np.degrees(st["yaw_rate_radps"])
    ay = st["ay_mps2"]
    sw = np.degrees(st["steer_sw_rad"])
    out = {"key": key}
    # standstill on wheels
    v_kmh = v * 3.6
    lim = np.interp(t, ch["ecu_B_speedlimit_en"]["time"], ch["ecu_B_speedlimit_en"]["data"])
    pit = (lim > 0.5) & (v_kmh > PIT_KMH[0]) & (v_kmh < PIT_KMH[1])
    trav_ok = all(ch.get(f"log_susp_travel_{c}", {}).get("quality") not in (None, "missing", "failed") for c in CORNERS)
    if trav_ok and pit.any():
        travel = {c: _normalize_travel_to_mm(np.interp(t, ch[f"log_susp_travel_{c}"]["time"],
                                                       ch[f"log_susp_travel_{c}"]["data"]),
                                             ch[f"log_susp_travel_{c}"]["unit_raw"]) for c in CORNERS}
        still = (v_kmh < STANDSTILL_KMH) & np.all(
            [np.abs(travel[c] - np.median(travel[c][pit])) < ON_WHEELS_TRAVEL_MM for c in CORNERS], axis=0)
        out["standstill_rule"] = "on wheels (travel within 5 mm of the pit-lane median, all four corners)"
    else:
        still = v_kmh < STANDSTILL_KMH
        out["standstill_rule"] = "speed only (no travel channels or no pit-lane samples)"
    out["standstill_s"] = float(still.sum() / sr)
    if out["standstill_s"] >= MIN_STANDSTILL_S:
        out["standstill"] = {"yaw_rate_deg_s": stats(r_deg[still]), "ay_mps2": stats(ay[still]),
                             "steer_sw_deg": stats(sw[still])}
    # the chain's straights, as R3 uses them
    sd = steering_straights(st, parsed["laps"], p["tyre_fit_auto"])
    m = sd["mask"]
    vr = v * st["yaw_rate_radps"]
    out["straights"] = {"n": int(m.sum()), "s": float(m.sum() / sr), "centre_deg": sd["centre"],
                        "yaw_rate_deg_s": stats(r_deg[m]), "ay_mps2": stats(ay[m]), "steer_sw_deg": stats(sw[m]),
                        "v_r_mps2": stats(vr[m]), "v_mps": stats(v[m]),
                        "share_abs_r_above_1deg_s": float(np.mean(np.abs(r_deg[m]) > 1.0))}
    # sign convention: in corners a_y and v r share a sign (kinematic identity);
    # steering vs yaw rate from the same quasi-steady corner samples
    corner = st["moving_mask"] & (np.abs(ay) > 5.0)
    out["sign"] = {"frac_ay_vr_same_sign_in_corners": float(np.mean(np.sign(ay[corner]) == np.sign(vr[corner]))),
                   "frac_sw_r_same_sign_in_corners": float(np.mean(np.sign(sw[corner]) == np.sign(r_deg[corner])))}
    # heading change per analysis lap (integral of the yaw rate)
    laps = []
    for l in parsed["laps"]:
        if not l.get("is_valid_for_analysis"):
            continue
        s = (t >= l["start_time"]) & (t <= l["end_time"])
        T = float(s.sum() / sr)
        dpsi = float(np.sum(r_deg[s]) / sr)
        turns = round(dpsi / 360.0)
        laps.append({"lap": l["lap_number"], "T_s": T, "heading_change_deg": dpsi, "full_turns": turns,
                     "residual_deg": dpsi - 360.0 * turns,
                     "implied_mean_offset_deg_s": (dpsi - 360.0 * turns) / T if T > 0 else None})
    out["laps"] = laps
    return out


if __name__ == "__main__":
    root, dst = sys.argv[1], sys.argv[2]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(5) as ex:
        res = list(ex.map(session, [(root, k) for k in OUTINGS]))
    json.dump(res, open(dst, "w", encoding="utf-8"), indent=1)
    print("wrote", dst)
