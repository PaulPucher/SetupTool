# Speed fix and engine braking, stage 0 (review proposal 2026-11-05, author
# go 2026-11-06). Read-only: nothing in modules/, config/ or the goldens is
# touched; every number is computed from the raw exports.
#
# Step "extract" streams each export once and caches the channels below
# (raw time base, units as logged) to an npz per session in CACHE. The
# chain's own parser keeps only whitelisted channels, and the ABS-domain
# wheel speeds and the GPS quality channels are not whitelisted, so this
# reader is separate from modules/csv_parser.py on purpose.
#
# Step "measure" (A1, A2, A4, B1-B3) reads only that cache; step "b4" is
# the engine-braking counterfactual through the hand-in chain, in memory.
# Results: diagnostics/results_speed_stage0_20261106/ (measure.json,
# b4_<session>.json); the findings are recorded in thesis_notes 'Speed fix
# and engine braking, stage 0'.
#
# Run: python -m diagnostics.inspect_speed_stage0 extract [workers]
#      python -m diagnostics.inspect_speed_stage0 measure [workers]
#      python -m diagnostics.inspect_speed_stage0 b4 <hand-in copy root> [workers]
# (SPEED_STAGE0_CACHE sets the cache folder.)

import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

CACHE = os.environ.get("SPEED_STAGE0_CACHE",
                       os.path.join(os.environ.get("TEMP", ROOT), "speed_stage0_cache"))
OUT = os.path.join(ROOT, "diagnostics", "results_speed_stage0_20261106")

_FINAL = "H:/dani_try 100"
_NBR = "H:/Data_Bachelorthesis/unlabeled/02_NBR_CRV_data"
_DEV = "H:/Data_Bachelorthesis"
# (key, track, set, path). same_as marks a development export of a final
# session (header: same outing, session and logger serial).
SESSIONS = (
    *((f"NBR{i}", "NBR", "final", f"{_FINAL}/NBR{i}.txt") for i in range(1, 6)),
    *((f"PRC{i}", "PRC", "final", f"{_FINAL}/PRC{i}.txt") for i in range(1, 6)),
    ("nbr_o05_mla", "NBR", "dev", f"{_NBR}/01_SE_01/NBR-73-005-001-MLA-26.07.01.0919.txt"),
    ("nbr_o10_jdr", "NBR", "dev", f"{_NBR}/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("nbr_o20_jst", "NBR", "dev", f"{_NBR}/01_SE_02/NBR-73-020-065-JST-26.07.01.1454.txt"),
    ("nbr_o29_jst", "NBR", "dev", f"{_DEV}/NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"),
    ("nbr_o46_jst", "NBR", "dev", f"{_NBR}/01_TE_04/NBR-73-046-162-JST-26.07.02.1411.txt"),
    ("nbr_o53_mla", "NBR", "dev", f"{_NBR}/01_TE_04/NBR-73-053-207-MLA-26.07.02.1707.txt"),
    ("nbr_o54_cdr", "NBR", "dev", f"{_NBR}/01_TE_04/NBR-73-054-214-CDR -26.07.02.1727.txt"),
    ("nbr_o59_mla", "NBR", "dev", f"{_NBR}/New folder/NBR-73-059-232-MLA-26.07.03.1210.txt"),
    ("nbr_o61_jdr", "NBR", "dev", f"{_NBR}/New folder/NBR-73-061-238-JDR-26.07.03.1229.txt"),
    ("nbr_o63_cdr", "NBR", "dev", f"{_NBR}/New folder/NBR-73-063-256-CDR-26.07.03.1309.txt"),
    ("nbr_qp_cdr", "NBR", "dev", f"{_DEV}/NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt"),
    ("nbr_qp_mla", "NBR", "dev", f"{_DEV}/NBR_Qualy_MLA/MLA-QP01-NBR-v2.txt"),
    ("pr_o25_us_v1", "PRC", "dev", f"{_DEV}/Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt"),
    ("pr_o32_us_v2", "PRC", "dev", f"{_DEV}/paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
)
SAME_AS = {"nbr_o29_jst": "NBR1", "nbr_o20_jst": "NBR3", "pr_o32_us_v2": "PRC4"}

WHEELS = ("fl", "fr", "rl", "rr")
WANTED = (
    "ecu_speed", "log_speed", "pcu_ecu_speed", "Corrected Speed", "corr_speed", "DTM_speed_TTL",
    *(f"ecu_speed_{w}" for w in WHEELS), *(f"log_speed_{w}" for w in WHEELS),
    *(f"abs_speed_{w}" for w in WHEELS), *(f"ecu_E_speed_{w}" for w in WHEELS),
    "log_B_wheelspeed_quality", "log_B_wheelspeed_quality_lat",
    "log_gps_speed", "gpsa_speed", "NMEA RX RMC Speed Over Ground", "NMEA RX VTG Speed Ground",
    "MRR_EgoVehSpeed", "MRR_Status_vehSpeedValid",
    "log_gps_quality", "log_gps_sats", "log_gps_hdop", "log_gps_mode", "log_gps_status",
    "log_gps_data_age", "log_gps_lat", "log_gps_lon",
    "ecu_aps", "ecu_gear", "ecu_nmot", "log_pbrake_f", "log_pbrake_r", "log_asteer",
    "log_acc_x", "log_acc_y", "sclu_yaw_rate", "lap_number", "lap_distance",
    "ecu_B_speedlimit_en", "ecu_pclutch", "ecu_B_cc_eclutch_open",
    "ecu_B_EB_act", "ecu_B_EB_enab", "ecu_B_EB_sail_act", "ecu_EB_int_pos", "ecu_EB_tim_pos",
    "log_eb_int_pos", "log_eb_tim_pos",
)


def _name_unit(raw):
    raw = raw.strip()
    if raw.endswith("]") and "[" in raw:
        i = raw.rindex("[")
        return raw[:i], raw[i + 1:-1]
    return raw, ""


def extract_one(job):
    key, path = job
    dst = os.path.join(CACHE, f"{key}.npz")
    if os.path.exists(dst):
        return key, "cached", 0.0
    t0 = time.time()
    wanted = set(WANTED)
    arrays, units, meta = {}, {}, {}
    with open(path, "r", encoding="latin-1") as fh:
        line = fh.readline()
        while line:
            s = line.strip()
            if s == "{OutingInformation}":
                line = fh.readline()
                while line and not line.startswith("{"):
                    p = line.strip().split("\t")
                    if len(p) == 2:
                        meta[p[0]] = p[1]
                    line = fh.readline()
                continue
            if s == "{ChannelBlock}":
                hdr = fh.readline().rstrip("\r\n").split("\t")
                name, unit = _name_unit(hdr[1]) if len(hdr) == 2 else ("", "")
                keep = len(hdr) == 2 and name in wanted
                buf = []
                line = fh.readline()
                while line and not line.startswith("{"):
                    if keep:
                        buf.append(line)
                    line = fh.readline()
                if keep:
                    # same tolerance as parse_csv: decimal comma, NaN dropped
                    txt = "".join(buf).replace(",", ".")
                    vals = np.array([r.split("\t") for r in txt.split("\n") if r.count("\t") == 1],
                                    dtype=object)
                    t = np.array([_f(x) for x in vals[:, 0]]) if len(vals) else np.array([])
                    v = np.array([_f(x) for x in vals[:, 1]]) if len(vals) else np.array([])
                    ok = np.isfinite(t) & np.isfinite(v)
                    arrays[f"t__{name}"] = t[ok]
                    arrays[f"v__{name}"] = v[ok]
                    units[name] = unit
                continue
            line = fh.readline()
    os.makedirs(CACHE, exist_ok=True)
    np.savez_compressed(dst, **arrays, __units=np.array(list(units.items()), dtype=object),
                        __meta=np.array(list(meta.items()), dtype=object))
    return key, f"{len(units)} channels", time.time() - t0


def _f(x):
    try:
        return float(x)
    except ValueError:
        return float("nan")


def load(key):
    z = np.load(os.path.join(CACHE, f"{key}.npz"), allow_pickle=True)
    ch = {}
    for k in z.files:
        if k.startswith("t__"):
            n = k[3:]
            ch[n] = (z[k], z[f"v__{n}"])
    units = dict(z["__units"].tolist())
    meta = dict(z["__meta"].tolist())
    return ch, units, meta


# --- step "measure": A1, A2, A4, B1-B3 from the cache -----------------------
#
# Regime definitions are ad-hoc labels of this check (Tier B, preprocessing
# for the comparison only), not chain definitions:
#   braking   log_pbrake_f > 3 bar (the chain's no-braking floor)
#   traction  ecu_aps > 50 %, brake < 3 bar
#   corner    |a_y| > 5 m/s^2, brake < 3 bar
#   straight  |a_y| < 1.5 m/s^2, brake < 3 bar, ecu_aps > 90 %
# GPS speed lags the vehicle (receiver filtering); the lag is measured per
# session by cross-correlation on moving samples and removed before any
# error is taken. Spike criterion A is the 2026-11-05 one: speed above the
# maximum ABS wheel speed within +-0.1 s by more than 3 m/s, widened by
# 0.5 s for the replacement (variant R, same as the counterfactual).

G = 9.81
FS = 100.0
SPIKE_DV = 3.0
SPIKE_WIN_S = 0.1
SPIKE_WIDEN_S = 0.5
BRAKE_FLOOR_BAR = 3.0
_TO_MPS = {"kph": 1 / 3.6, "km/h": 1 / 3.6, "mph": 0.44704, "m/s": 1.0}


def _speed(ch, units, name):
    t, v = ch[name]
    return t, v * _TO_MPS[units[name].strip().lower()]


def _runs(mask):
    d = np.diff(np.r_[0, mask.astype(np.int8), 0])
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _rolling_max(x, half):
    from scipy.ndimage import maximum_filter1d
    return maximum_filter1d(x, size=2 * half + 1, mode="nearest")


def _widen(mask, half):
    from scipy.ndimage import binary_dilation
    return binary_dilation(mask, structure=np.ones(2 * half + 1, bool)) if mask.any() else mask


def build_grid(key):
    """Every channel on one 100 Hz grid over ecu_speed's time range (linear
    interpolation, as prepare_vehicle_state does)."""
    ch, units, meta = load(key)
    te = ch["ecu_speed"][0]
    t = np.arange(te[0], te[-1], 1.0 / FS)
    g = {"t": t}
    for n in ch:
        tt, vv = ch[n]
        if len(tt) < 2:
            continue
        if units.get(n, "").strip().lower() in _TO_MPS and "gps" not in n.lower() and "speed" in n.lower():
            tt, vv = _speed(ch, units, n)
        g[n] = np.interp(t, tt, vv)
    if "log_gps_speed" in ch:
        g["_gps_t"], g["_gps_v"] = _speed(ch, units, "log_gps_speed")
    return g, ch, units, meta


def spike_mask(v, abs_max):
    half = int(round(SPIKE_WIN_S * FS))
    return v > _rolling_max(abs_max, half) + SPIKE_DV


def candidates(g):
    abs4 = np.vstack([g[f"abs_speed_{w}"] for w in WHEELS])
    abs_max = abs4.max(axis=0)
    front_max = np.maximum(g["abs_speed_fl"], g["abs_speed_fr"])
    front_mean = 0.5 * (g["abs_speed_fl"] + g["abs_speed_fr"])
    sp = spike_mask(g["ecu_speed"], abs_max)
    wide = _widen(sp, int(round(SPIKE_WIDEN_S * FS)))
    r = np.where(wide, front_max, g["ecu_speed"])
    out = {"ecu_speed (current)": g["ecu_speed"], "R: ecu_speed, spikes -> max front ABS": r,
           "B: max front ABS": front_max, "C: mean front ABS": front_mean}
    # D: the ecu_speed formation rule (item 1) applied to the clean ABS family
    ecu_front_max = np.maximum(g["ecu_speed_fl"], g["ecu_speed_fr"])
    out["D: ECU-family max front (rr unused)"] = ecu_front_max
    return out, sp, wide, abs_max


def gps_lag(g, v_ref, moving):
    """Lag (s) maximising the correlation of GPS speed with v_ref(t - lag)."""
    tg, vg = g["_gps_t"], g["_gps_v"]
    sel = (tg > g["t"][0] + 1) & (tg < g["t"][-1] - 1)
    tg, vg = tg[sel], vg[sel]
    mv = np.interp(tg, g["t"], moving.astype(float)) > 0.99
    best = None
    for lag in np.arange(0.0, 0.81, 0.01):
        vr = np.interp(tg - lag, g["t"], v_ref)
        c = np.corrcoef(vr[mv], vg[mv])[0, 1]
        if best is None or c > best[1]:
            best = (lag, c)
    return best


def measure_session(key):
    from modules.stability_analysis import load_parameters
    p = load_parameters()
    m = p["vehicle"]["mass_kg"]
    rho = p["vehicle"]["aero"]["air_density_kgm3"]
    area = p["vehicle"]["aero"]["cross_track_area_m2"]
    cd = p["longitudinal_stiffness"]["drag_coeff"]
    g, ch, units, meta = build_grid(key)
    res = {"key": key, "meta": {k: meta.get(k) for k in ("OutingNumber", "SessionNumber", "TrackName",
                                                         "DriverName")}}
    # A2: channel census on the raw time base
    cen = {}
    for n in WANTED:
        if n not in ch:
            cen[n] = None
            continue
        tt, vv = ch[n]
        if len(tt) < 2:
            cen[n] = {"n": int(len(tt))}
            continue
        dt = np.diff(tt)
        nom = float(np.median(dt))
        cen[n] = {"unit": units.get(n, ""), "rate_hz": round(1 / nom, 2) if nom > 0 else None,
                  "n": int(len(tt)), "gaps_gt_2x": int(np.sum(dt > 2.5 * nom)),
                  "max_gap_s": float(dt.max()), "zero_share": float(np.mean(vv == 0)),
                  "min": float(vv.min()), "max": float(vv.max())}
    res["census"] = cen

    cands, sp, wide, abs_max = candidates(g)
    v_ecu = g["ecu_speed"]
    moving = v_ecu > 5.0
    if "ecu_B_speedlimit_en" in g:
        moving &= g["ecu_B_speedlimit_en"] < 0.5
    # A1: formation of ecu_speed from the ECU wheel set, outside spikes
    clean = moving & ~wide
    ecu4 = {w: g[f"ecu_speed_{w}"] for w in WHEELS}
    rules = {"max front ECU": np.maximum(ecu4["fl"], ecu4["fr"]),
             "mean front ECU": 0.5 * (ecu4["fl"] + ecu4["fr"]),
             "max of four ECU": np.max(np.vstack(list(ecu4.values())), axis=0),
             "mean of four ECU": np.mean(np.vstack(list(ecu4.values())), axis=0),
             "2nd largest of four ECU": np.sort(np.vstack(list(ecu4.values())), axis=0)[-2],
             "mean rear ECU": 0.5 * (ecu4["rl"] + ecu4["rr"]),
             "ecu_speed_rr alone": ecu4["rr"],
             "max front ABS": np.maximum(g["abs_speed_fl"], g["abs_speed_fr"]),
             "log_speed (logger)": g.get("log_speed", np.full_like(v_ecu, np.nan))}
    form = {}
    for name, x in rules.items():
        e = (x - v_ecu)[clean]
        form[name] = {"rms": float(np.sqrt(np.nanmean(e ** 2))), "median": float(np.nanmedian(e)),
                      "p99_abs": float(np.nanpercentile(np.abs(e), 99)),
                      "exact_share_0p05": float(np.nanmean(np.abs(e) < 0.05))}
    # inside spikes: which rule reproduces the spike value
    insp = sp & moving
    form_sp = {}
    for name, x in rules.items():
        e = (x - v_ecu)[insp]
        form_sp[name] = float(np.sqrt(np.nanmean(e ** 2))) if insp.any() else None
    res["formation"] = {"outside_spikes": form, "inside_spikes_rms": form_sp}
    # A1: spike coincidence -- per ecu_speed spike event, is the ECU rr / ABS rr
    # wheel speed also above the ABS maximum of the other three wheels?
    others_abs = np.max(np.vstack([g[f"abs_speed_{w}"] for w in ("fl", "fr", "rl")]), axis=0)
    half = int(round(SPIKE_WIN_S * FS))
    ref = _rolling_max(others_abs, half) + SPIKE_DV
    ev = _runs(sp & moving)
    co = {"events": len(ev), "samples": int((sp & moving).sum())}
    for wname in ("ecu_speed_rr", "abs_speed_rr", "ecu_speed_fl", "ecu_speed_fr", "ecu_speed_rl", "abs_speed_fl"):
        hit = g[wname] > ref
        co[wname] = int(sum(1 for a, b in ev if hit[a:b].any()))
    # error flags / quality flags during spikes
    for fl in ("ecu_E_speed_rr", "ecu_E_speed_fl", "log_B_wheelspeed_quality", "log_B_wheelspeed_quality_lat"):
        if fl in g:
            co[f"{fl}_nonzero_in_spike_events"] = int(sum(1 for a, b in ev if (g[fl][a:b] != 0).any()))
            co[f"{fl}_nonzero_share_moving"] = float(np.mean(g[fl][moving] != 0))
    # braking share of spike events and the spike peak
    co["events_under_braking"] = int(sum(1 for a, b in ev if (g["log_pbrake_f"][a:b] > BRAKE_FLOOR_BAR).any()))
    co["events_decel"] = int(sum(1 for a, b in ev if (g["log_acc_x"][a:b] < -0.2).any()))
    co["peak_mps"] = float(max((v_ecu[a:b].max() for a, b in ev), default=float("nan")))
    res["spikes"] = co

    # A4: candidates vs GPS
    lag, r = gps_lag(g, cands["R: ecu_speed, spikes -> max front ABS"], moving)
    res["gps_lag_s"], res["gps_lag_r"] = float(lag), float(r)
    tg, vg = g["_gps_t"], g["_gps_v"]
    sel = (tg - lag > g["t"][0] + 0.1) & (tg < g["t"][-1] - 0.1)
    tg, vg = tg[sel], vg[sel]

    def on_gps(x):
        return np.interp(tg - lag, g["t"], x)

    mv = on_gps(moving.astype(float)) > 0.99
    pb = on_gps(g["log_pbrake_f"])
    aps = on_gps(g["ecu_aps"])
    ay = np.abs(on_gps(g["log_acc_y"])) * G
    regimes = {"all moving": mv, "straight": mv & (ay < 1.5) & (pb < 3) & (aps > 90),
               "braking": mv & (pb > 3), "traction": mv & (aps > 50) & (pb < 3),
               "corner": mv & (ay > 5) & (pb < 3)}
    # GPS repeated fixes (frozen value, catch-up next) are not speed errors of
    # the candidate: drop GPS samples equal to their predecessor while moving
    rep = np.r_[False, np.diff(vg) == 0]
    regimes = {k: v & ~rep for k, v in regimes.items()}
    res["gps_repeated_fixes"] = int((rep & mv).sum())
    # GPS noise scale on straights: residual of GPS speed around a 1 s
    # centred moving average (10 samples), straight samples only
    k = 11
    sm = np.convolve(vg, np.ones(k) / k, mode="same")
    st = regimes["straight"]
    res["gps_noise_straight_std"] = float(np.std((vg - sm)[st])) if st.sum() > 20 else None
    cmp = {}
    for name, x in cands.items():
        xe = on_gps(x)
        e = xe - vg
        # constant scale (rolling radius) separated from the dynamic error
        k_scale = float(np.sum(vg[mv] * xe[mv]) / np.sum(xe[mv] ** 2))
        e_k = k_scale * xe - vg
        row = {"k_gps_over_cand": k_scale}
        for rn, msk in regimes.items():
            if msk.sum() < 20:
                row[rn] = None
                continue
            row[rn] = {"n": int(msk.sum()), "bias": float(np.mean(e[msk])),
                       "rms": float(np.sqrt(np.mean(e[msk] ** 2))),
                       "p99_abs": float(np.percentile(np.abs(e[msk]), 99)),
                       "rms_scaled": float(np.sqrt(np.mean(e_k[msk] ** 2))),
                       "p99_abs_scaled": float(np.percentile(np.abs(e_k[msk]), 99))}
        # remaining spikes: criterion A against ABS max, and criterion G
        # (candidate above lag-aligned GPS by more than 3 m/s, events)
        row["spike_events_A"] = len(_runs(spike_mask(x, abs_max) & moving))
        gps_on_grid = np.interp(g["t"] + lag, tg, vg)
        row["spike_events_G"] = len(_runs((x * k_scale - gps_on_grid > SPIKE_DV) & moving))
        row["low_events_G"] = len(_runs((gps_on_grid - x * k_scale > SPIKE_DV) & moving))
        # scale relative to current ecu_speed outside spikes
        row["median_ratio_to_ecu_clean"] = float(np.median(x[clean] / v_ecu[clean]))
        cmp[name] = row
    res["candidates"] = cmp
    # GPS quality
    gq = {}
    for n in ("log_gps_quality", "log_gps_sats", "log_gps_hdop", "log_gps_mode", "log_gps_status",
              "log_gps_data_age"):
        if n in ch:
            vv = ch[n][1]
            vals, cnt = np.unique(vv, return_counts=True)
            top = sorted(zip(cnt, vals), reverse=True)[:4]
            gq[n] = {"median": float(np.median(vv)), "min": float(vv.min()), "max": float(vv.max()),
                     "top_values": [(float(b), round(float(a) / len(vv), 4)) for a, b in top]}
    res["gps_quality"] = gq

    # B1-B3: engine braking on coasting samples. Specific force: m f_x is the
    # sum of the tyre and aero forces, so the road gradient drops out;
    # F_x,tyres = m f_x + F_drag (+ rolling, 0 in config). The IMU mounting
    # pitch error does not drop out; it is estimated below from the closed
    # laps (mean of f_x - dv/dt, gradient sums to ~0 over a lap).
    fx_spec = g["log_acc_x"] * G
    v_ref = cands["R: ecu_speed, spikes -> max front ABS"]
    steer = g["log_asteer"]
    centre = float(np.median(steer[moving & (np.abs(g["log_acc_y"]) < 0.05)]))
    straight_s = np.abs(steer - centre) < 3.0
    clutch_closed = g.get("ecu_B_cc_eclutch_open", np.zeros_like(v_ecu)) < 0.5
    gear = np.round(g["ecu_gear"]).astype(int)
    off = moving & (g["ecu_aps"] < 1.0) & (g["log_pbrake_f"] < BRAKE_FLOOR_BAR) & ~wide
    if "log_pbrake_r" in g:
        off &= g["log_pbrake_r"] < BRAKE_FLOOR_BAR
    coast = (off & straight_s & (np.abs(g["log_acc_y"]) * G < 2.0) & clutch_closed & (gear >= 1)
             & (v_ref > 20) & (v_ref < 80))
    b = {"throttle_off_no_brake_s": float(off.sum() / FS),
         "throttle_off_no_brake_runs_median_s": float(np.median([(q - p0) / FS for p0, q in _runs(off)]))
         if off.any() else None,
         "coast_candidate_s": float(coast.sum() / FS), "session_s": float(g["t"][-1] - g["t"][0]),
         "steer_centre_deg": centre}
    # throttle lift and brake release transients: drop the first part of
    # every coasting run, keep runs of a minimum length; strict and relaxed
    rules = {"strict (drop 0.3 s, min 0.5 s)": (0.3, 0.5), "relaxed (drop 0.1 s, min 0.2 s)": (0.1, 0.2)}
    drag = {cda: 0.5 * rho * cda * v_ref ** 2 for cda in (0.7, cd * area, 1.3)}
    f_eng = {cda: m * fx_spec + d for cda, d in drag.items()}
    keeps = {}
    for rname, (drop, dmin) in rules.items():
        keep = np.zeros_like(coast)
        nr = 0
        for a, e in _runs(coast):
            a2 = a + int(drop * FS)
            if (e - a2) / FS >= dmin:
                keep[a2:e] = True
                nr += 1
        keeps[rname] = keep
        row = {"n_samples": int(keep.sum()), "duration_s": float(keep.sum() / FS), "n_runs": nr}
        if keep.sum() > 30:
            row["F_engine_N"] = {f"cda_{cda:.1f}": {"median": float(np.median(f[keep])),
                                                    "p10": float(np.percentile(f[keep], 10)),
                                                    "p90": float(np.percentile(f[keep], 90))}
                                 for cda, f in f_eng.items()}
            base = f_eng[cd * area]
            row["by_speed"] = {}
            for lo, hi in ((20, 35), (35, 50), (50, 80)):
                s_ = keep & (v_ref >= lo) & (v_ref < hi)
                if s_.sum() > 10:
                    row["by_speed"][f"{lo}-{hi}"] = {
                        "n": int(s_.sum()), "F_median": float(np.median(base[s_])),
                        "drag_median": float(np.median(drag[cd * area][s_])),
                        "drag_span_0.7_1.3": float(np.median(drag[1.3][s_] - drag[0.7][s_]))}
            row["by_gear"] = {}
            for gg in np.unique(gear[keep]):
                s_ = keep & (gear == gg)
                if s_.sum() > 10:
                    row["by_gear"][int(gg)] = {
                        "n": int(s_.sum()), "F_median": float(np.median(base[s_])),
                        "F_iqr": float(np.subtract(*np.percentile(base[s_], [75, 25]))),
                        "rpm_median": float(np.median(g["ecu_nmot"][s_])),
                        "v_median": float(np.median(v_ref[s_]))}
            if "ecu_EB_int_pos" in g:
                row["eb_int_pos"] = {str(k2): int(c2) for k2, c2 in
                                     zip(*np.unique(np.round(g["ecu_EB_int_pos"][keep]), return_counts=True))}
            if "ecu_B_EB_act" in g:
                row["eb_act_share"] = float(np.mean(g["ecu_B_EB_act"][keep] > 0.5))
        b[rname] = row
    if "ecu_EB_int_pos" in g:
        b["eb_int_pos_session"] = {str(k2): round(int(c2) / moving.sum(), 3) for k2, c2 in
                                   zip(*np.unique(np.round(g["ecu_EB_int_pos"][moving]), return_counts=True))}
        b["eb_tim_pos_session"] = {str(k2): round(int(c2) / moving.sum(), 3) for k2, c2 in
                                   zip(*np.unique(np.round(g["ecu_EB_tim_pos"][moving]), return_counts=True))}
    if "ecu_B_EB_act" in g:
        b["eb_act_share_moving"] = float(np.mean(g["ecu_B_EB_act"][moving] > 0.5))
        b["eb_act_share_braking"] = float(np.mean(
            g["ecu_B_EB_act"][moving & (g["log_pbrake_f"] > BRAKE_FLOOR_BAR)] > 0.5))
    # IMU f_x bias estimate on full laps (lap_number changes)
    lapn = np.round(g["lap_number"]).astype(int)
    biases = []
    dvdt = np.gradient(v_ref, 1.0 / FS)
    for ln in np.unique(lapn):
        s = (lapn == ln) & moving
        if s.sum() > 60 * FS and s.mean() > 0:
            idx = np.flatnonzero(lapn == ln)
            if moving[idx].mean() > 0.98:
                biases.append(float(np.mean(fx_spec[idx] - dvdt[idx])))
    b["imu_fx_bias_per_lap_mps2"] = biases
    if biases:
        bias = float(np.median(biases))
        b["imu_bias_force_N"] = bias * m
        for rname, keep in keeps.items():
            if keep.sum() > 30:
                b[rname]["F_engine_bias_corrected_median_N"] = float(np.median(f_eng[cd * area][keep]) - bias * m)
    # B3: braking force for scale, R speed, config c_D A
    brk = moving & (g["log_pbrake_f"] > BRAKE_FLOOR_BAR) & ~wide
    fb = -(m * fx_spec + drag[cd * area])
    b["braking_force_N"] = {"median": float(np.median(fb[brk])), "p95": float(np.percentile(fb[brk], 95)),
                            "peak_p99.9": float(np.percentile(fb[brk], 99.9)), "n": int(brk.sum())}
    # throttle-off share of braking samples (where engine braking can act)
    b["braking_with_aps_below_1pct_share"] = float(np.mean(g["ecu_aps"][brk] < 1.0))
    res["engine_braking"] = b
    res["mass_kg"], res["cda_config"] = m, cd * area
    return res


def measure_all(workers=4):
    import json
    from concurrent.futures import ProcessPoolExecutor
    os.makedirs(OUT, exist_ok=True)
    keys = [k for k, _, _, _ in SESSIONS]
    with ProcessPoolExecutor(workers) as ex:
        results = list(ex.map(measure_session, keys))
    with open(os.path.join(OUT, "measure.json"), "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=1, default=float)
    print(f"wrote {OUT}/measure.json")


# --- step "b4": engine-braking counterfactual (read-only, in memory) -------
#
# Under deceleration with the throttle closed (fx_cog < 0 and ecu_aps below
# B4_APS_OFF_PCT) the engine-braking force F_e acts on the driven rear axle;
# only the remainder fx_cog - F_e is split by axle load. If the car
# decelerates less than F_e alone, all of fx_cog goes to the rear. Every
# other sample keeps the shipped split. Implemented by wrapping
# tyre_fit_auto._axle_load_inputs: the raw front change is passed through
# the same 2 Hz zero-phase filter and moved from front to rear, so the
# axle sum is unchanged. Runs on a copy of the hand-in tree (root given on
# the command line) so the identity matches the 2026-11-05 counterfactual.

B4_APS_OFF_PCT = 1.0
B4_VARIANTS = {"base": None, "eb_1.46kN": -1460.0, "eb_2.10kN": -2100.0}
B4_STATS = {}


def _install_b4(f_e):
    import modules.tyre_fit_auto as tfa
    orig = tfa._axle_load_inputs
    cap = {}
    ev, el = tfa.estimate_vertical_loads, tfa.estimate_longitudinal_forces

    def ev_w(*a, **k):
        cap["fz"] = ev(*a, **k)
        return cap["fz"]

    def el_w(*a, **k):
        cap["fx"] = el(*a, **k)
        return cap["fx"]

    def wrapped(state, data, params, forces, fit_mask):
        tfa.estimate_vertical_loads, tfa.estimate_longitudinal_forces = ev_w, el_w
        try:
            out = orig(state, data, params, forces, fit_mask)
        finally:
            tfa.estimate_vertical_loads, tfa.estimate_longitudinal_forces = ev, el
        if "degenerate_reason" in out or f_e is None:
            return out
        fx = cap["fx"]["fx_N"]
        fz = cap["fz"]
        with np.errstate(invalid="ignore", divide="ignore"):
            share_f = fz["fz_f_N"] / (fz["fz_f_N"] + fz["fz_r_N"])
        old_front = np.where(fx < 0.0, fx * share_f, 0.0)
        eb = (fx < 0.0) & (state["throttle_pct"] < B4_APS_OFF_PCT)
        rem = fx - f_e
        new_front = np.where(eb, np.where(rem < 0.0, rem * share_f, 0.0), old_front)
        d = tfa._butterworth_lowpass(new_front - old_front, params["stability_estimation"]["cs_filter_cutoff_hz"],
                                     state["sample_rate_hz"])
        sin_d, cos_d = np.sin(out["delta_f"]), np.cos(out["delta_f"])
        term_old = out["fx_f_N"] * sin_d / cos_d
        out["fx_f_N"] = out["fx_f_N"] + d
        out["fx_r_N"] = out["fx_r_N"] - d
        term_new = out["fx_f_N"] * sin_d / cos_d
        fm = fit_mask & np.isfinite(term_old)
        dt = np.abs(term_new - term_old)[fm]
        fy = np.abs(forces["Fy_f_filt"])[fm]
        B4_STATS.update({
            "eb_share_moving": float(np.mean(eb[state["moving_mask"]])),
            "eb_share_fit": float(np.mean(eb[fit_mask])),
            "front_term_change_N": {"median": float(np.median(dt)), "p99": float(np.percentile(dt, 99)),
                                    "max": float(dt.max())},
            "front_term_change_rel_Fy_median": float(np.median(dt / np.maximum(fy, 1.0))),
            "front_term_change_rel_Fy_p99": float(np.percentile(dt / np.maximum(fy, 1.0), 99)),
            "front_fx_change_fit_N": {"median": float(np.median(np.abs(d[fit_mask]))),
                                      "p99": float(np.percentile(np.abs(d[fit_mask]), 99))},
        })
        return out

    tfa._axle_load_inputs = wrapped
    return lambda: setattr(tfa, "_axle_load_inputs", orig)


def b4_session(job):
    root, key, path = job
    sys.path.insert(0, root)
    from modules.csv_parser import parse_csv
    from modules.accuracy_resolution import resolve_accuracy
    from modules.stability_analysis import load_parameters, summarise_corners
    from modules import pipeline
    t0 = time.time()
    data = parse_csv(path)
    out = {"key": key, "root": root, "variants": {}}
    for vname, f_e in B4_VARIANTS.items():
        B4_STATS.clear()
        undo = _install_b4(f_e)
        try:
            params = load_parameters()
            resolved = resolve_accuracy(params, setup_data=None, cap=None)
            chain = pipeline.run_analysis(data, resolved, csv_path=path)
            summaries = summarise_corners(chain["corners"], chain["cs"], chain["stab"], chain["state"],
                                          fz=chain["fz"], lap_filter=pipeline.analysis_laps(data))
            layers = pipeline.recommend(summaries, corners=chain["corners"], state=chain["state"],
                                        channels=data.get("channels"), laps=data.get("laps"))
            summ = pipeline._session_summary(path, chain, summaries, layers)
        finally:
            undo()
        man = chain["fit_manifest"] or {}
        summ["braking_excluded"] = (man.get("axle_load") or {}).get("braking_excluded")
        summ["fit_n"] = {a: (man.get("axles") or {}).get(a, {}).get("n_fit") for a in ("front", "rear")}
        summ["b4_stats"] = dict(B4_STATS)
        summ["mass_kg"] = params["vehicle"]["mass_kg"]
        out["variants"][vname] = summ
    out["runtime_s"] = time.time() - t0
    return out


def b4_all(root, workers):
    import json
    from concurrent.futures import ProcessPoolExecutor, as_completed
    os.makedirs(OUT, exist_ok=True)
    jobs = [(root, k, p) for k, _, s, p in SESSIONS if s == "final"]
    with ProcessPoolExecutor(workers) as ex:
        futs = {ex.submit(b4_session, j): j[1] for j in jobs}
        for f in as_completed(futs):
            r = f.result()
            with open(os.path.join(OUT, f"b4_{r['key']}.json"), "w", encoding="utf-8") as fh:
                json.dump(r, fh, indent=1, default=float)
            print(f"{r['key']} done ({r['runtime_s']:.0f} s)", flush=True)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "extract"
    if cmd == "extract":
        from concurrent.futures import ProcessPoolExecutor
        workers = int(sys.argv[2]) if len(sys.argv) > 2 else 4
        jobs = [(k, p) for k, _, _, p in SESSIONS]
        print(f"cache {CACHE}", flush=True)
        with ProcessPoolExecutor(workers) as ex:
            for key, msg, dt in ex.map(extract_one, jobs):
                print(f"{key}: {msg} ({dt:.0f} s)", flush=True)
    elif cmd == "measure":
        measure_all(int(sys.argv[2]) if len(sys.argv) > 2 else 4)
    elif cmd == "b4":
        os.environ.setdefault("OMP_NUM_THREADS", "1")
        b4_all(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 5)
    elif cmd == "one":
        import json
        print(json.dumps(measure_session(sys.argv[2]), indent=1, default=float))
