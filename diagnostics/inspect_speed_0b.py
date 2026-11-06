# ORDER SPEED-0b (review, 2026-11-06): read-only follow-up of stage 0 on the
# working identity. Every chain run imports modules/ and config/ from a
# scratchpad copy of the working repo (root given on the command line) and
# reads the setup sheets from that copy's app DB (read-only), through the
# thesis-figure helpers load_session / run_chain. Nothing in the repo, the
# DB or the goldens is written.
#
# Steps:
#   margin  item 1 -- spike margin of variant R against the root-cause
#           signature |ecu_speed_rr - abs_speed_rr| > 0.5 m/s (stage-0 cache)
#   b2      item 2 -- coasting engine-braking estimate with each session's
#           resolved mass and the working drag (stage-0 cache + copy config)
#   chain   items 2 and 3 -- current / R / R + engine braking chains per
#           final session, and the sensor census on the R chain
# Results: diagnostics/results_speed_0b_20261106/.
#
# Run: python -m diagnostics.inspect_speed_0b margin
#      python -m diagnostics.inspect_speed_0b b2 <copy root>
#      python -m diagnostics.inspect_speed_0b chain <copy root> [workers]

import json
import os
import sys
import time

import numpy as np

from diagnostics import inspect_speed_stage0 as s0

OUT = os.path.join(s0.ROOT, "diagnostics", "results_speed_0b_20261106")
# app outing id of each final session (DB outings table, csv_path checked)
OUTINGS = {"NBR1": 17, "NBR2": 18, "NBR3": 19, "NBR4": 20, "NBR5": 21,
           "PRC1": 22, "PRC2": 25, "PRC3": 26, "PRC4": 27, "PRC5": 28}
FINAL = tuple(OUTINGS)
SIG_DV = 0.5            # root-cause signature, m/s (order)
MARGINS = (1.0, 3.0)    # spike margins under test, m/s
R_MARGIN = 3.0          # margin of the R used in the chain runs (recorded)
NATIVE_HZ = 50.0        # ecu_speed and ecu_speed_* time base
Z_SPIKE = 8.0           # first-difference robust z (order)
STUCK_S = 0.5           # constant run while moving (order)


def _native(key):
    """ecu_speed and the wheel speeds on ecu_speed's own 50 Hz samples, m/s."""
    ch, units, _ = s0.load(key)
    t, v = s0._speed(ch, units, "ecu_speed")
    out = {"t": t, "v": v}
    for n in [f"abs_speed_{w}" for w in s0.WHEELS] + ["ecu_speed_rr"]:
        tt, vv = s0._speed(ch, units, n)
        out[n] = np.interp(t, tt, vv)
    for n in ("log_pbrake_f", "ecu_B_speedlimit_en"):
        out[n] = np.interp(t, *ch[n]) if n in ch else np.zeros_like(t)
    return out


def spike_r(t, v, abs4, margin, half_s=s0.SPIKE_WIN_S, widen_s=s0.SPIKE_WIDEN_S, hz=NATIVE_HZ):
    """Variant R on a native time base: flagged samples, widened mask and
    the replaced speed (max front ABS on the widened mask)."""
    amax = np.max(abs4, axis=0)
    flagged = v > s0._rolling_max(amax, int(round(half_s * hz))) + margin
    wide = s0._widen(flagged, int(round(widen_s * hz)))
    return flagged, wide, np.where(wide, np.maximum(abs4[0], abs4[1]), v)


def margin_session(key):
    n = _native(key)
    abs4 = np.vstack([n[f"abs_speed_{w}"] for w in s0.WHEELS])
    moving = (n["v"] > 5.0) & (n["ecu_B_speedlimit_en"] < 0.5)
    dev = n["ecu_speed_rr"] - n["abs_speed_rr"]
    sig0 = np.abs(dev) > SIG_DV
    # +-1 sample (20 ms): the ECU forms ecu_speed one cycle after its wheel inputs
    sig1 = sig0 | np.r_[sig0[1:], False] | np.r_[False, sig0[:-1]]
    amax = np.max(abs4, axis=0)
    affected = moving & (n["v"] > s0._rolling_max(amax, 5) + SIG_DV)
    res = {"key": key, "moving_samples": int(moving.sum()),
           "signature_samples_moving": int((sig0 & moving).sum()),
           "clean_rr_minus_abs_rr": {"median": float(np.median(dev[moving & ~sig0])),
                                     "p1": float(np.percentile(dev[moving], 1)),
                                     "p99": float(np.percentile(dev[moving], 99))},
           "ecu_speed_affected_samples": int(affected.sum()), "margins": {}}
    for m in MARGINS:
        fl, wide, _ = spike_r(n["t"], n["v"], abs4, m)
        fl &= moving
        res["margins"][str(m)] = {
            "flagged": int(fl.sum()),
            "flagged_events": len(s0._runs(fl)),
            "flagged_not_signature_same_sample": int((fl & ~sig0).sum()),
            "flagged_not_signature_pm1": int((fl & ~sig1).sum()),
            "flagged_under_braking": int((fl & (n["log_pbrake_f"] > s0.BRAKE_FLOOR_BAR)).sum()),
            "widened_samples": int((wide & moving).sum()),
            "undetected_signature": int((sig0 & moving & ~fl).sum()),
            "undetected_signature_outside_widened": int((sig0 & moving & ~wide).sum()),
            "undetected_ecu_speed_affected": int((affected & sig1 & ~wide).sum()),
            "undetected_ecu_speed_affected_max_dv": float(np.max((n["v"] - amax)[affected & sig1 & ~wide],
                                                                 initial=0.0)),
        }
    return res


# --- engine braking (B2) on the working identity ----------------------------

def _copy_imports(root):
    if sys.path[0] != root:
        sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "tools", "thesis_figures"))
    import common   # noqa: F401  (chdir to the copy, DB of the copy)
    import modules
    # modules/ has no __init__: a namespace package over every sys.path entry,
    # searched in order -- the copy must come first
    first = os.path.normcase(os.path.dirname(list(modules.__path__)[0]))
    assert first == os.path.normcase(os.path.normpath(root)), list(modules.__path__)
    import modules.stability_analysis as sa
    assert os.path.normcase(sa.__file__).startswith(os.path.normcase(os.path.normpath(root))), sa.__file__
    return common


def resolved_vehicle(root, outing):
    common = _copy_imports(root)
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.stability_analysis import load_parameters
    row = common.db_outing(outing)
    raw = load_parameters()
    p = apply_resolved_vehicle(raw, resolve_accuracy(raw, common._loads(row["setup_data"]), None))
    v = p["vehicle"]
    return {"mass_kg": v["mass_kg"], "rho": v["aero"]["air_density_kgm3"], "area": v["aero"]["cross_track_area_m2"],
            "cd": p["longitudinal_stiffness"]["drag_coeff"], "rolling": p["longitudinal_stiffness"]["rolling_resistance_coeff"]}


def b2_session(job):
    root, key = job
    veh = resolved_vehicle(root, OUTINGS[key])
    g, ch, units, meta = s0.build_grid(key)
    cands, sp, wide, amax = s0.candidates(g)
    v = cands["R: ecu_speed, spikes -> max front ABS"]
    m, cda = veh["mass_kg"], veh["cd"] * veh["area"]
    moving = (g["ecu_speed"] > 5.0) & (g.get("ecu_B_speedlimit_en", 0 * v) < 0.5)
    fx = g["log_acc_x"] * s0.G
    steer = g["log_asteer"]
    centre = float(np.median(steer[moving & (np.abs(g["log_acc_y"]) < 0.05)]))
    gear = np.round(g["ecu_gear"]).astype(int)
    off = moving & (g["ecu_aps"] < 1.0) & (g["log_pbrake_f"] < 3) & (g["log_pbrake_r"] < 3) & ~wide
    coast = (off & (np.abs(steer - centre) < 3.0) & (np.abs(g["log_acc_y"]) * s0.G < 2.0)
             & (g["ecu_B_cc_eclutch_open"] < 0.5) & (gear >= 1) & (v > 20) & (v < 80))
    keep = np.zeros_like(coast)
    for a, b in s0._runs(coast):           # relaxed rule of stage 0
        if (b - (a + 10)) / s0.FS >= 0.2:
            keep[a + 10:b] = True
    drag = 0.5 * veh["rho"] * cda * v ** 2
    rolling = veh["rolling"] * m * s0.G
    f_e = m * fx + drag + rolling
    dvdt = np.gradient(v, 1.0 / s0.FS)
    lapn = np.round(g["lap_number"]).astype(int)
    biases = [float(np.mean((fx - dvdt)[lapn == ln])) for ln in np.unique(lapn)
              if (lapn == ln).sum() > 60 * s0.FS and moving[lapn == ln].mean() > 0.98]
    bias_n = float(np.median(biases)) * m if biases else 0.0
    res = {"key": key, "vehicle": veh, "cda": cda, "coast_s": float(keep.sum() / s0.FS),
           "imu_bias_N": bias_n, "imu_bias_laps": len(biases)}
    if keep.sum() >= 20:
        res["F_median_N"] = float(np.median(f_e[keep]))
        res["F_bias_corrected_N"] = res["F_median_N"] - bias_n
        res["bands"] = {}
        for lo, hi, name in ((20, 45, "medium 20-45 m/s"), (45, 80, "high 45-80 m/s")):
            s = keep & (v >= lo) & (v < hi)
            if s.sum() >= 10:
                dm = float(np.median(drag[s]))
                res["bands"][name] = {"n": int(s.sum()), "F_median_N": float(np.median(f_e[s])) - bias_n,
                                      "drag_median_N": dm,
                                      "drag_share_of_decel_force": dm / float(np.median(-m * fx[s] + bias_n)),
                                      "drag_span_cda_pm30pct_N": 0.6 * dm}
    brk = moving & (g["log_pbrake_f"] > 3) & ~wide
    fb = -(m * fx + drag)
    res["braking_force_N"] = {"median": float(np.median(fb[brk])), "p95": float(np.percentile(fb[brk], 95))}
    return res


# --- chain runs and sensor census -------------------------------------------

def _install_r(key, margin=R_MARGIN):
    """Variant R inside parse_csv: ecu_speed (raw kph) replaced on the widened
    spikes before lap splitting and corner detection."""
    import modules.csv_parser as cp
    orig = cp._assert_fixed_factor_units
    ch, units, _ = s0.load(key)
    info = {}

    def wrapped(channels):
        sp = channels["ecu_speed"]
        t = sp["time"]
        abs4 = np.vstack([np.interp(t, *s0._speed(ch, units, f"abs_speed_{w}")) for w in s0.WHEELS]) * 3.6
        fl, wide, r = spike_r(t, np.asarray(sp["data"], float), abs4, margin * 3.6)
        sp["data"] = r
        info.update({"flagged": int(fl.sum()), "replaced": int(wide.sum())})
        return orig(channels)

    cp._assert_fixed_factor_units = wrapped
    return info, lambda: setattr(cp, "_assert_fixed_factor_units", orig)


def _evaluate(session, chain):
    from modules import pipeline
    row = session["row"]
    layers = pipeline.recommend(chain["summaries"], setup_data=json.loads(row["setup_data"] or "null"),
                                feedback_data=json.loads(row["feedback_data"] or "null"),
                                corners=chain["corners"], state=chain["state"],
                                channels=session["parsed"].get("channels"), laps=session["parsed"].get("laps"))
    ch = dict(chain, sideslip_source=session["ident"]["sideslip_source"])
    summ = pipeline._session_summary(session["path"], ch, chain["summaries"], layers)
    vote = layers["data_layer"].get("session_recommendation") or {}
    rec = vote.get("recommended")
    summ["session_recommendation_full"] = (
        {"levers": rec["levers"], "direction": rec["direction"], "net": rec["net"]} if rec else
        {"tie": [(o["levers"], o["direction"]) for o in vote.get("equivalent_options") or []]} or None)
    man = chain["fit_manifest"] or {}
    summ["braking_excluded"] = (man.get("axle_load") or {}).get("braking_excluded")
    return summ


def census(session, chain):
    from modules.observer import steering_straights
    st, parsed, p = chain["state"], session["parsed"], session["params"]
    t, sr = st["time"], st["sample_rate_hz"]
    mov = st["moving_mask"]
    out = {"channels": {}}
    for name, c in parsed["channels"].items():
        if c.get("quality") in ("missing", "failed") or c.get("time") is None or len(c["time"]) < 10:
            out["channels"][name] = {"quality": c.get("quality")}
            continue
        tt, d = np.asarray(c["time"], float), np.asarray(c["data"], float)
        mv = np.interp(tt, t, mov.astype(float)) > 0.99
        dt = np.diff(tt)
        nom = float(np.median(dt))
        row = {"quality": c["quality"], "rate_hz": round(1 / nom, 1), "n": int(tt.size),
               "gaps": int(np.sum(dt > 2.5 * nom)), "gap_s": float(np.sum(dt[dt > 2.5 * nom]))}
        d1 = np.diff(d)
        m1 = mv[1:] & mv[:-1]
        if m1.sum() > 100:
            med = np.median(d1[m1])
            mad = 1.4826 * np.median(np.abs(d1[m1] - med))
            if mad > 0:
                z = np.abs(d1 - med) / mad
                hit = m1 & (z > Z_SPIKE)
                row["spike_samples"] = int(hit.sum())
                row["spike_share"] = float(hit.sum() / m1.sum())
                # isolated spikes: an up-step followed within 2 samples by a down-step of the same size
                sgn = np.sign(d1 - med)
                iso = 0
                for i in np.flatnonzero(hit):
                    j = i + 1
                    if j < len(z) and z[j] > Z_SPIKE and sgn[j] == -sgn[i]:
                        iso += 1
                row["isolated_spikes"] = iso
            else:
                row["spike_rule"] = "MAD of the first difference is 0 (quantised or mostly constant)"
            same = np.r_[False, d1 == 0]
            runs = [(a, b) for a, b in s0._runs(same & mv) if tt[min(b, tt.size - 1)] - tt[max(a - 1, 0)] > STUCK_S]
            row["stuck_runs"] = len(runs)
            row["stuck_s"] = float(sum(tt[min(b, tt.size - 1)] - tt[max(a - 1, 0)] for a, b in runs))
        out["channels"][name] = row
    # straight-line medians on the chain's own straights
    sd = steering_straights(st, parsed["laps"], p["tyre_fit_auto"])
    sm = sd["mask"]
    out["straights"] = {"n": int(sm.sum()), "steer_centre_deg": sd["centre"],
                        "yaw_rate_deg_s": float(np.degrees(np.median(st["yaw_rate_radps"][sm]))),
                        "steer_sw_deg": float(np.degrees(np.median(st["steer_sw_rad"][sm]))),
                        "ay_mps2": float(np.median(st["ay_mps2"][sm])),
                        "beta_kin_deg": float(np.degrees(np.median(chain["beta_kin"][sm]))),
                        "beta_chain_deg": float(np.degrees(np.median(chain["beta"][sm])))}
    # a_y against v r on quasi-steady corner samples
    n_avg = max(1, int(0.1 * sr))
    sw = np.degrees(st["steer_sw_rad"])
    rate = np.gradient(np.convolve(sw, np.ones(n_avg) / n_avg, mode="same"), 1.0 / sr)
    vr = st["v_mps"] * st["yaw_rate_radps"]
    kerb = np.zeros_like(mov)
    for k in ("kerb_mask", "kerb_smear_zone"):
        if st.get(k) is not None:
            kerb |= st[k]
    qs = mov & ~kerb & (np.abs(st["ay_mps2"]) > 5.0) & (np.abs(rate) < 10.0) & (np.abs(vr) > 3.0)
    ratio = st["ay_mps2"][qs] / vr[qs]
    out["ay_vs_vr"] = {"rule": "moving, outside kerb mask / smear, |a_y| > 5 m/s^2, |steer rate| < 10 deg/s "
                               "(0.1 s mean), |v r| > 3 m/s^2",
                       "n": int(qs.sum()), "median": float(np.median(ratio)),
                       "p10": float(np.percentile(ratio, 10)), "p90": float(np.percentile(ratio, 90))}
    # time offsets by cross-correlation on moving samples, +-0.3 s
    abs_f = np.maximum(np.interp(t, *_abs_raw(session["key"], "fl")), np.interp(t, *_abs_raw(session["key"], "fr")))
    pairs = {"a_y vs v r (IMU vs ECU speed x gyro)": (st["ay_mps2"], vr),
             "d(ecu_speed)/dt vs d(max front ABS)/dt": (np.gradient(st["v_mps"], 1 / sr), np.gradient(abs_f, 1 / sr)),
             "d(ecu_speed)/dt vs a_x (ECU vs IMU)": (np.gradient(st["v_mps"], 1 / sr), st["ax_mps2"]),
             "d(max front ABS)/dt vs a_x (ABS vs IMU)": (np.gradient(abs_f, 1 / sr), st["ax_mps2"]),
             "yaw rate vs a_y / v (gyro vs IMU)": (st["yaw_rate_radps"], st["ay_mps2"] / np.maximum(st["v_mps"], 5))}
    out["lags_ms"] = {k: _lag_ms(a, b, mov, sr) for k, (a, b) in pairs.items()}
    # yaw-rate spikes and the raw-gradient yaw acceleration
    r = st["yaw_rate_radps"]
    rdot = np.gradient(r, 1.0 / sr)
    yc = parsed["channels"]["sclu_yaw_rate"]
    ty, dy = np.asarray(yc["time"], float), np.asarray(yc["data"], float)
    d1 = np.diff(dy)
    mad = 1.4826 * np.median(np.abs(d1 - np.median(d1)))
    zt = ty[1:][np.abs(d1 - np.median(d1)) / mad > Z_SPIKE] if mad > 0 else np.array([])
    idx = np.argsort(-np.abs(np.where(mov, rdot, 0)))
    top, used = [], np.zeros(t.size, bool)
    for i in idx:
        if len(top) >= 10:
            break
        if used[max(0, i - 50):i + 50].any():
            continue
        used[i] = True
        near = bool(zt.size and np.min(np.abs(zt - t[i])) <= 0.05)
        top.append({"t": float(t[i]), "rdot_radps2": float(rdot[i]), "gyro_spike_within_50ms": near,
                    "kerb": bool(kerb[i]), "v": float(st["v_mps"][i]), "ay": float(st["ay_mps2"][i])})
    iz = p["vehicle"].get("yaw_inertia_kgm2") or p["vehicle"].get("Iz_kgm2")
    out["yaw_accel"] = {"p99_9_abs": float(np.percentile(np.abs(rdot[mov]), 99.9)),
                        "p99_abs": float(np.percentile(np.abs(rdot[mov]), 99)),
                        "gyro_spike_times": int(zt.size), "top10": top, "Iz_kgm2": iz}
    # damper-gauge drift: straight-line axle totals per analysis lap
    fz = chain.get("fz") or {}
    out["damper_drift"] = _damper_drift(st, fz, sm, parsed["laps"], p)
    return out


def _abs_raw(key, w):
    ch, units, _ = s0.load(key)
    return s0._speed(ch, units, f"abs_speed_{w}")


def _lag_ms(a, b, mask, sr, max_s=0.3):
    """Lag of a behind b (ms, positive = a later), parabolic peak refinement."""
    a = np.where(mask, a - np.mean(a[mask]), 0.0)
    b = np.where(mask, b - np.mean(b[mask]), 0.0)
    k = int(max_s * sr)
    lags = np.arange(-k, k + 1)
    c = np.array([np.dot(a[max(0, l):a.size + min(0, l)], b[max(0, -l):b.size - max(0, l)]) for l in lags])
    i = int(np.argmax(c))
    if 0 < i < len(c) - 1:
        den = c[i - 1] - 2 * c[i] + c[i + 1]
        off = 0.5 * (c[i - 1] - c[i + 1]) / den if den != 0 else 0.0
    else:
        off = 0.0
    r = c[i] / np.sqrt(np.dot(a, a) * np.dot(b, b))
    return {"lag_ms": float((lags[i] + off) / sr * 1000), "r": float(r)}


def _damper_drift(st, fz, straight, laps, p):
    if fz.get("vertical_load_source_used") in (None, "static") or "fz_f_N" not in fz:
        return {"note": f"no measured loads ({fz.get('vertical_load_source_used')})"}
    cw = p["vehicle"]["corner_weights"]
    static = {"front": (cw["FL_kg"] + cw["FR_kg"]) * s0.G, "rear": (cw["RL_kg"] + cw["RR_kg"]) * s0.G}
    static["total"] = static["front"] + static["rear"]
    t, v = st["time"], st["v_mps"]
    tot = {"front": fz["fz_f_N"], "rear": fz["fz_r_N"]}
    tot["total"] = tot["front"] + tot["rear"]
    out = {"source": fz.get("vertical_load_source_used")}
    valid = [l for l in laps if l.get("is_valid_for_analysis")]
    for ax, x in tot.items():
        s = straight & np.isfinite(x)
        if s.sum() < 100:
            continue
        # aero load grows with v^2 even on straights: the per-lap means are
        # also given after a session fit x = a + c v^2 on the straight samples
        A = np.vstack([np.ones(s.sum()), v[s] ** 2]).T
        coef = np.linalg.lstsq(A, x[s], rcond=None)[0]
        res = x - (coef[0] + coef[1] * v ** 2)
        per_raw, per_res = [], []
        for l in valid:
            ls = s & (t >= l["start_time"]) & (t <= l["end_time"])
            if ls.sum() > 50:
                per_raw.append(float(np.mean(x[ls])))
                per_res.append(float(np.mean(res[ls])))
        if not per_raw:
            continue
        mu_raw = float(np.mean(per_raw))
        out[ax] = {"laps": len(per_raw),
                   "raw_dev_pct_static": [round(100 * (q - mu_raw) / static[ax], 2) for q in per_raw],
                   "v2_corrected_dev_pct_static": [round(100 * q / static[ax], 2) for q in per_res],
                   "session_mean_over_static": round(mu_raw / static[ax], 3)}
    return out


def chain_session(job):
    root, key, f_pool, f_upper = job
    common = _copy_imports(root)
    t0 = time.time()
    out = {"key": key, "outing": OUTINGS[key], "copy_root": root, "variants": {}}
    # current chain
    sess = common.load_session(OUTINGS[key])
    sess["key"] = key
    ch = common.run_chain(sess)
    out["variants"]["current"] = _evaluate(sess, ch)
    out["identity"] = _identity_check(common, sess, ch, out["variants"]["current"])
    del ch
    # R parse, then R alone and R + engine braking
    info, undo = _install_r(key)
    try:
        sess_r = common.load_session(OUTINGS[key])
    finally:
        undo()
    sess_r["key"] = key
    out["r_replacement"] = info
    for vname, f_e in (("R", None), ("R+EB pooled", f_pool), ("R+EB upper", f_upper)):
        s0.B4_STATS.clear()
        undo_b = s0._install_b4(f_e)
        try:
            ch = common.run_chain(sess_r)
            out["variants"][vname] = _evaluate(sess_r, ch)
            out["variants"][vname]["b4_stats"] = dict(s0.B4_STATS)
            if vname == "R":
                # run_chain does not return the vertical loads; same call as the chain
                from modules.stability_analysis import estimate_vertical_loads, load_car_data
                ch["fz"] = estimate_vertical_loads(ch["state"], ch["forces"], sess_r["params"],
                                                   channels=sess_r["parsed"]["channels"], car_data=load_car_data())
                out["census"] = census(sess_r, ch)
        finally:
            undo_b()
        del ch
    out["runtime_s"] = time.time() - t0
    with open(os.path.join(OUT, f"chain_{key}.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1, default=float)
    return key, out["runtime_s"]


def _identity_check(common, sess, chain, summ):
    payload, why = common.stored_payload(sess)
    if payload is None:
        return {"stored_payload": None, "reason": why}
    from modules.classifier import aggregate_by_corner, classify_corner
    try:
        stored_v = {str(cid): classify_corner(agg)[1] for cid, agg in aggregate_by_corner(payload["summaries"]).items()}
    except Exception as e:   # stored summaries are JSON, not arrays
        stored_v = {"error": repr(e)}
    now_v = {str(c["corner"]): c["verdict"] for c in summ["corner_verdicts"]}
    rec = ((payload.get("setup_engine") or {}).get("data_layer") or {}).get("session_recommendation") or {}
    srec = rec.get("recommended")
    return {"stored_payload": "valid", "schema": payload.get("schema_version"),
            "verdicts_equal": stored_v == now_v, "n_verdicts": len(now_v),
            "verdict_diffs": {k: (stored_v.get(k), now_v.get(k)) for k in set(stored_v) | set(now_v)
                              if stored_v.get(k) != now_v.get(k)},
            "stored_recommendation": (srec["levers"], srec["direction"]) if srec else None,
            "now_recommendation": summ["session_recommendation_full"]}


if __name__ == "__main__":
    cmd = sys.argv[1]
    os.makedirs(OUT, exist_ok=True)
    if cmd == "margin":
        res = [margin_session(k) for k in FINAL]
        with open(os.path.join(OUT, "margin.json"), "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=1, default=float)
        print("wrote margin.json")
    elif cmd == "b2":
        res = [b2_session((sys.argv[2], k)) for k in FINAL]
        with open(os.path.join(OUT, "b2.json"), "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=1, default=float)
        print("wrote b2.json")
    elif cmd == "chain":
        from concurrent.futures import ProcessPoolExecutor, as_completed
        b2 = json.load(open(os.path.join(OUT, "b2.json"), encoding="utf-8"))
        vals = sorted(r["F_bias_corrected_N"] for r in b2 if "F_bias_corrected_N" in r)
        f_pool, f_upper = float(np.median(vals)), float(min(vals))
        print(f"engine braking pooled {f_pool:.0f} N, upper {f_upper:.0f} N from {len(vals)} sessions", flush=True)
        workers = int(sys.argv[3]) if len(sys.argv) > 3 else 3
        with ProcessPoolExecutor(workers) as ex:
            futs = [ex.submit(chain_session, (sys.argv[2], k, f_pool, f_upper)) for k in FINAL]
            for f in as_completed(futs):
                k, dt = f.result()
                print(f"{k} done ({dt:.0f} s)", flush=True)
