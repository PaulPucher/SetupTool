# Phase 1 Q2c + Q3 (PLAN.md PHASE 1 PLAN; proposal thesis_notes.md "Q2c + Q3
# combined proposal", approved 2026-10-02 with rulings 1-5). Read-only.
#
# RULES, fixed before any run (proposal text + rulings; [impl] = detail the
# proposal left open, fixed here before the run).
# Courtroom (section 1, amended 2026-10-02): (i) fit-internal mechanics only;
#   (ii) rake A->B resolution; (iii) driver statements; (v) cross-lap
#   coherence; (iv) CS bounds = sanity screen. Le Mans never scored; Dubai
#   least weight (reported, never decisive).
# G0 GPS census, 20 usable files: per channel n, rate, distinct, frozen share
#   while moving (ecu_speed > 30 kph), unit. Independence between families
#   (log_gps / NMEA RX / gpsa / IMSA) on speed (10 Hz grid): exact-identity
#   share, corr, rms diff, best lag (+/-2 s); position log_gps vs gpsa
#   (median distance); altitude log_gps_alt vs gpsa_alt (median and rms of the
#   de-meaned difference). Latency vs IMU per speed family: lag maximising
#   corr(d speed/dt, a_x) within +/-1 s on a 100 Hz grid, both low-passed by a
#   0.3 s moving mean [impl].
# G1 = Q2c gradient map: altitude per live family on the analysis laps,
#   5 m lap-distance bins, gradient over a 50 m centred baseline [impl];
#   per position mean and SE = sd / sqrt(n laps) (n >= 3). Pooled per track
#   with lap distance normalised to the track's median lap length [impl].
#   USABLE iff SE <= 1 % on >= 80 % of the lap length; two independent
#   families AGREE iff the median |gradient difference| <= 1 %.
# G2 corner anchor (UNSCORED read; ruling 4: a passing anchor gets its own
#   validation package): course over ground (log_gps_course; NMEA RMC COG as
#   second source), shifted by the G0 latency, unwrapped. Heading = course on
#   straight anchors (|a_y| < 0.1 g, |r| < 2 deg/s, v > 100 kph, runs >= 1 s,
#   beta ~ 0 assumed) and integrated yaw rate between consecutive anchors,
#   linearly drift-closed to the next anchor; segments > 60 s dropped [impl].
#   Yaw-rate sign against course rate from the data [impl]. beta_gps =
#   course - heading. Per corner: mean over analysis laps of the apex-region
#   mean; SE = sd / sqrt(n). BAR SE <= 0.39 deg (half the smallest recorded
#   beta-candidate separation, Door 1 Stage 0). Yield on the 12 new NBR
#   outings: corner-laps with log_gps_data_age < 0.5 s and log_gps_sats >= 6
#   at the apex [impl].
#   [2026-10-02, POST-RESULT REPAIR OF THE TEST INPUT, author ruling after the
#   run: the straight gate on raw 100 Hz a_y never held for 1 s (vibration
#   noise 0.15-0.23 g > 0.1 g). Mode g2r applies the same gate to the G0
#   0.3 s moving mean of a_y and r; bar, thresholds and all else unchanged;
#   output g2_repair.json, g2.json kept as the degenerate record.]
#   [2026-10-02, POST-RESULT REPAIR 2: DELAY SIGN + ISO CONVENTION (author
#   ruling; declared the LAST repair). G0's field lag_s_gps_behind_imu holds
#   the delay with an inverted sign, so g2/g2r shifted the course ~0.5 s the
#   wrong way. Mode g2r2: course shifted EARLIER by the measured +0.30 s
#   (course rate vs yaw rate, corr 0.995, outing B), both course sources;
#   beta_gps reported in the ISO convention (left positive; course over
#   ground is compass-positive clockwise, so beta_iso = heading - course).
#   Smoothed gate as g2r; bar and all else unchanged; output
#   g2_repair2.json, g2.json / g2_repair.json kept as records.]
#   [2026-10-02, DRIFT SCOPE LINE (author ruling: known method limitation,
#   stated before the run, not a defect): heading between straight
#   re-zeros is integrated yaw rate with the drift closed LINEARLY IN TIME,
#   as pre-registered above. The WP5b(c) record (thesis_notes, 2026-07-26)
#   measured a gyro shortfall of ~6 deg per lap (~1.7 % of net rotation)
#   that accumulates with rotation, so time-linear closure under-corrects
#   inside corners on long segments; the rotation-clock alternative is NOT
#   taken (beyond the repair budget). Every anchor number carries this.]
# V1 = prior_anchored at prior_weight 0 (Powell from the 18-session prior,
#   no penalty) vs production (chair start); 18 pool sessions + Dubai. The
#   (d2) chain cache at w 0 IS this variant: ENTRY CHECKS -- the cached prior
#   equals config tyre_fit_auto.tyre_fit_prior; refitting w 0 on the cached
#   arrays reproduces the cached manifest; one fresh chain (JST 21, w 0 and
#   production) reproduces the cached CS arrays. Start-point sensitivity =
#   curve RMS difference on [0, production-population p99 |alpha|], % of
#   production D. MATERIALITY BAR 5.5 % of D (GPS k 1.2 %, a_y scale 4.5 %,
#   mass 3 %, quadrature; a lower bound). Lap bootstrap of the difference,
#   B = 30 [impl]. Material iff max over axles > bar.
# V2 = prior_anchored at w 0.1 / 1 / 10 (cached chains) and the scale-only
#   sub-variant (prior shape frozen, per-session closed-form lambda per axle
#   on the production fit population, EKF on (B, C, lambda D, E); new chains).
# Full chain + courtroom: material sessions PLUS always the rake pair
#   (nbr_jst_problem = A, outing 21; nbr_jst_solution = B, outing 29) and the
#   driver-statement sessions (only A/B carry a recorded label) -- ruling 5.
# (ii) [amended by the author's pre-run ruling, 2026-10-02: DIRECTION-PINNED,
#   PHASE-RESOLVED] the recorded procedure (inspect_cs_readout_evidence):
#   per-lap phase medians with production slicing and gate, apex_3 from the
#   apex region, PROD read-out (worst lap), leave-one-lap-out jackknife SE;
#   RESOLVED iff |B - A| > 2 sqrt(SE_A^2 + SE_B^2) -- now on all five phases.
#   Cell = matched corner x phase. [impl] Category from the contestant's own
#   A point estimate: gap = CS_r - CS_f > 0 -> FRONT-limited, < 0 -> REAR.
#   (a) front-limited cells (primary, as pre-registered): CS_f UP and gap
#       DOWN are expected, each resolved separately; score_a = expected -
#       opposite.
#   (b) rear-limited cells: gap UP (rear limitation reduced) = CONSISTENT,
#       gap DOWN = AGAINST.
#   Decision use [impl]: score_a against production; an AGAINST in (b)
#   counts as an opposite resolution (blocks earning). Symmetric guard on
#   score_a.
# (iii) [per the driver statement anchored 2026-10-02, thesis_notes "Driver
#   statement (iii), NBR outing A"; confidence: author's recollection]:
#   matched PER PHASE on A's point estimates -- entry (entry_1_brake,
#   entry_2_turnin) expected REAR-limited (gap < 0); mid (apex_3) expected
#   FRONT-limited (gap > 0); exit (exit_4, exit_5) expected REAR-limited.
#   [impl] a group is contradicted iff < 50 % of its cells match; (iii) is
#   CONTRADICTED iff >= 2 of 3 groups are. A -> B read: share of cells
#   whose |gap| shrinks ("less extreme in both directions"), descriptive.
# (v) per session median over corner x phase x axle of the lap-to-lap IQR of
#   per-lap phase medians (>= 3 laps); production spread = half the 95 %
#   lap-bootstrap width (B = 50) [impl]; a variant is worse iff its mean
#   excess over its scored sessions exceeds the mean production half-width.
# DECISION (ruled): adoption-proposal-worthy iff (i) passes, (ii) >= prod + 2
#   with zero opposite resolutions, (v) not worse, (iii) not contradicted;
#   SYMMETRIC GUARD: (ii) <= prod - 2 disqualifies; tiebreak simplicity
#   (production = V1 < V2 penalised / scale-only).
# Render rule: CS traces per contestant per scored session; files named in
#   the report.
# Outputs diagnostics/plots/phase1_q3/. Modes: g0 | g1 | g2 | v1 | v2 | q4 | all | g2r | g2r2

import gzip
import json
import os
import pickle
import sys
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks

OUT = "diagnostics/plots/phase1_q3"
CACHE = os.path.join(OUT, "cache")
G = 9.81
A_KEY, B_KEY = "nbr_jst_problem", "nbr_jst_solution"
NEW12 = ("nbr_o005_mla", "nbr_o010_jdr", "nbr_o018_jst", "nbr_o019_jst", "nbr_o020_jst", "nbr_o046_jst",
         "nbr_o053_mla", "nbr_o054_cdr", "nbr_o059_mla", "nbr_o061_jdr", "nbr_o063_cdr", "nbr_o064_jst")
FAMILIES = {
    "log_gps": ["log_gps_lat", "log_gps_lon", "log_gps_alt", "log_gps_speed", "log_gps_course", "log_gps_hdop",
                "log_gps_sats", "log_gps_quality", "log_gps_data_age"],
    "nmea": ["NMEA RX RMC Course Over Ground", "NMEA RX RMC Speed Over Ground", "NMEA RX VTG Speed Ground",
             "NMEA RX RMC Mode"],
    "gpsa": ["gpsa_lat", "gpsa_long", "gpsa_alt", "gpsa_speed", "gpsa_time"],
    "dtm": ["DTM_gps_sats"],
    "imsa": ["IMSA_GPS_vSpeed", "IMSA_GPS_xAlt", "IMSA_GPS_aLat", "IMSA_GPS_aLong", "IMSA_GPS_NSat"],
}
SPEEDS = {"log_gps": "log_gps_speed", "nmea": "NMEA RX RMC Speed Over Ground", "gpsa": "gpsa_speed",
          "imsa": "IMSA_GPS_vSpeed"}
ALTS = {"log_gps": "log_gps_alt", "gpsa": "gpsa_alt", "imsa": "IMSA_GPS_xAlt"}
SPEED_KPH = {"kph": 1.0, "km/h": 1.0, "mph": 1.609344, "m/s": 3.6}
ACC_MS2 = {"G": G, "g": G, "m/s/s": 1.0, "m/s^2": 1.0}
GRAD_SE_BAR, GRAD_LEN_BAR, GRAD_AGREE = 0.01, 0.80, 0.01
ANCHOR_BAR_DEG = 0.39
G2_DELAY_S = 0.30  # measured course delay behind the IMU (repair 2)
MAT_BAR = 5.5
N_BOOT_V1, N_BOOT_COH = 30, 50
RNG = np.random.default_rng(20261002)
WEIGHTS = (0.1, 1.0, 10.0)


def _unit(h):
    return h[h.find("[") + 1:h.find("]")] if "[" in h else ""


def _save(name, obj):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=float)


def _load(name):
    with open(os.path.join(OUT, name)) as fh:
        return json.load(fh)


def _pk(path, obj=None):
    os.makedirs(CACHE, exist_ok=True)
    p = os.path.join(CACHE, path)
    if obj is None:
        with gzip.open(p, "rb") as fh:
            return pickle.load(fh)
    with gzip.open(p, "wb") as fh:
        pickle.dump(obj, fh, protocol=pickle.HIGHEST_PROTOCOL)


# ---------------------------------------------------------------- G0

def _lag(x, y, dt, max_s):
    best = (0, -2.0)
    n = int(max_s / dt)
    for L in range(-n, n + 1):
        a, b = (x[L:], y[:-L]) if L > 0 else ((x[:L], y[-L:]) if L < 0 else (x, y))
        m = np.isfinite(a) & np.isfinite(b)
        if m.sum() > 100:
            c = np.corrcoef(a[m], b[m])[0, 1]
            if c > best[1]:
                best = (L, c)
    return best[0] * dt, best[1]


def _mavg(x, n):
    k = np.ones(n) / n
    return np.convolve(np.nan_to_num(x), k, mode="same")


def g0():
    out = {}
    for key in d2.usable_keys():
        try:
            path = d2.sessions()[key]["path"]
            names = [n for f in FAMILIES.values() for n in f] + ["ecu_speed", "log_acc_x"]
            B = read_raw_blocks(path, names)
            ts, vs, hs = B["ecu_speed"]
            spd = vs * SPEED_KPH.get(_unit(hs), 1.0)
            rec = {"channels": {}, "independence": {}, "latency": {}}
            for fam, chans in FAMILIES.items():
                for n in chans:
                    if n not in B:
                        continue
                    t, v, h = B[n]
                    mv = np.interp(t, ts, spd) > 30 if t.size else np.array([], bool)
                    d = np.diff(v[mv]) if mv.sum() > 2 else np.array([])
                    rec["channels"][n] = {"family": fam, "unit": _unit(h), "n": int(t.size),
                                          "rate_hz": float(1 / np.median(np.diff(t))) if t.size > 2 else float("nan"),
                                          "distinct": int(np.unique(v).size), "min": float(np.min(v)) if v.size else None,
                                          "max": float(np.max(v)) if v.size else None,
                                          "frozen_share_moving": float(np.mean(d == 0)) if d.size else None}
            live_speed = {f: n for f, n in SPEEDS.items()
                          if n in B and rec["channels"][n]["distinct"] > 50}
            grid = np.arange(ts[0], ts[-1], 0.1)
            mv10 = np.interp(grid, ts, spd) > 30
            sp = {f: np.interp(grid, B[n][0], B[n][1] * SPEED_KPH.get(_unit(B[n][2]), 1.0)) for f, n in live_speed.items()}
            fams = sorted(sp)
            for i, a in enumerate(fams):
                for b in fams[i + 1:]:
                    x, y = sp[a][mv10], sp[b][mv10]
                    lag, c = _lag(sp[a], sp[b], 0.1, 2.0)
                    rec["independence"][f"speed {a}~{b}"] = {
                        "identical_share": float(np.mean(np.abs(x - y) < 1e-6)), "corr": float(np.corrcoef(x, y)[0, 1]),
                        "rms_diff_kph": float(np.sqrt(np.mean((x - y) ** 2))), "best_lag_s": lag, "corr_at_lag": c}
            if "log_gps_lat" in B and "gpsa_lat" in B and rec["channels"]["gpsa_lat"]["distinct"] > 50:
                la1 = np.interp(grid, B["log_gps_lat"][0], B["log_gps_lat"][1])
                lo1 = np.interp(grid, B["log_gps_lon"][0], B["log_gps_lon"][1])
                la2 = np.interp(grid, B["gpsa_lat"][0], B["gpsa_lat"][1])
                lo2 = np.interp(grid, B["gpsa_long"][0], B["gpsa_long"][1])
                dist = np.hypot((la1 - la2) * 111320.0, (lo1 - lo2) * 111320.0 * np.cos(np.radians(la1)))
                rec["independence"]["position log_gps~gpsa"] = {"median_m": float(np.median(dist[mv10])),
                                                               "p95_m": float(np.percentile(dist[mv10], 95))}
            alts = {f: n for f, n in ALTS.items() if n in B and rec["channels"][n]["distinct"] > 50}
            if "log_gps" in alts and "gpsa" in alts:
                a1 = np.interp(grid, B["log_gps_alt"][0], B["log_gps_alt"][1])
                a2 = np.interp(grid, B["gpsa_alt"][0], B["gpsa_alt"][1])
                dd = (a1 - a2)[mv10]
                rec["independence"]["altitude log_gps~gpsa"] = {"median_diff_m": float(np.median(dd)),
                                                               "rms_demeaned_m": float(np.std(dd)),
                                                               "identical_share": float(np.mean(np.abs(dd) < 1e-6))}
            tax, vax, hax = B["log_acc_x"]
            g100 = np.arange(ts[0], ts[-1], 0.01)
            ax100 = _mavg(np.interp(g100, tax, vax * ACC_MS2.get(_unit(hax), G)), 30)
            mv100 = np.interp(g100, ts, spd) > 30
            for f, n in live_speed.items():
                v100 = np.interp(g100, B[n][0], B[n][1] * SPEED_KPH.get(_unit(B[n][2]), 1.0)) / 3.6
                dv = _mavg(np.gradient(v100, 0.01), 30)
                lag, c = _lag(np.where(mv100, dv, np.nan), np.where(mv100, ax100, np.nan), 0.01, 1.0)
                rec["latency"][f] = {"lag_s_gps_behind_imu": -lag, "corr": c}
            rec["live_speed_families"] = sorted(live_speed)
            rec["live_alt_families"] = sorted(alts)
            out[key] = rec
            print(f"[g0 {key}] live speed {sorted(live_speed)}, alt {sorted(alts)}; latency "
                  + ", ".join(f"{f} {v['lag_s_gps_behind_imu']:+.2f}s" for f, v in rec["latency"].items()), flush=True)
        except Exception:
            out[key] = {"error": traceback.format_exc()}
            print(f"[g0 {key}] ERROR", flush=True)
    _save("g0.json", out)


# ---------------------------------------------------------------- G1

def _lap_profiles(key, fam_chan):
    params, data, state = d2._state(key)
    B = read_raw_blocks(d2.sessions()[key]["path"], [fam_chan])
    if fam_chan not in B:
        return None
    t, s = state["time"], state["s_m"]
    alt = np.interp(t, B[fam_chan][0], B[fam_chan][1])
    profiles = []
    for l in data["laps"]:
        if not l.get("is_valid_for_analysis"):
            continue
        m = (t >= l["start_time"]) & (t <= l["end_time"]) & np.isfinite(s)
        if m.sum() < 100:
            continue
        ss, aa = s[m], alt[m]
        o = np.argsort(ss)
        profiles.append((ss[o], aa[o], float(np.nanmax(ss))))
    return profiles


def _gradient(ss, aa, grid):
    a = np.interp(grid, ss, aa)
    g = np.full(grid.size, np.nan)
    h = 5  # 5 bins of 5 m = 25 m each side
    g[h:-h] = (a[2 * h:] - a[:-2 * h]) / 50.0
    return g


def g1():
    g0r = _load("g0.json")
    sess = d2.sessions()
    tracks = {}
    for key in d2.usable_keys():
        tracks.setdefault(sess[key]["track"], []).append(key)
    out = {}
    for track, keys in tracks.items():
        rec = {"sessions": keys, "families": {}}
        fam_prof = {}
        for fam, chan in ALTS.items():
            profs = []
            for key in keys:
                if fam not in g0r.get(key, {}).get("live_alt_families", []):
                    continue
                p = _lap_profiles(key, chan)
                if p:
                    profs += p
            if len(profs) >= 3:
                fam_prof[fam] = profs
        for fam, profs in fam_prof.items():
            L = float(np.median([p[2] for p in profs]))
            grid = np.arange(0.0, L, 5.0)
            gs = np.array([_gradient(ss / mx * L, aa, grid) for ss, aa, mx in profs])
            mean = np.nanmean(gs, axis=0)
            n = np.sum(np.isfinite(gs), axis=0)
            se = np.nanstd(gs, axis=0) / np.sqrt(np.maximum(n, 1))
            se[n < 3] = np.nan
            ok = np.isfinite(se)
            rec["families"][fam] = {"n_laps": len(profs), "lap_len_m": L,
                                    "share_se_le_1pct": float(np.mean(se[ok] <= GRAD_SE_BAR)) if ok.any() else None,
                                    "median_se_pct": float(np.nanmedian(se) * 100),
                                    "p95_abs_gradient_pct": float(np.nanpercentile(np.abs(mean), 95) * 100),
                                    "usable": bool(ok.any() and np.mean(se[ok] <= GRAD_SE_BAR) >= GRAD_LEN_BAR)}
            fam_prof[fam] = (grid, mean, se)
        fams = [f for f in fam_prof if isinstance(fam_prof[f], tuple)]
        if len(fams) >= 2:
            (g1_, m1, _), (g2_, m2, _) = fam_prof[fams[0]], fam_prof[fams[1]]
            m2i = np.interp(g1_, g2_, m2)
            rec["agreement"] = {"pair": fams[:2], "median_abs_diff_pct": float(np.nanmedian(np.abs(m1 - m2i)) * 100),
                                "agree": bool(np.nanmedian(np.abs(m1 - m2i)) <= GRAD_AGREE)}
        _render_g1(track, {f: fam_prof[f] for f in fams})
        out[track] = rec
        print(f"[g1 {track}] " + json.dumps(rec["families"]), flush=True)
    _save("g1.json", out)


def _render_g1(track, prof):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND, REF
    if not prof:
        return
    fig, ax = plt.subplots(figsize=(14, 4.5), constrained_layout=True)
    for (fam, (grid, m, se)), ls in zip(prof.items(), ("-", "--", ":")):
        ax.plot(grid, m * 100, ls, color=INK, lw=0.9, label=f"{fam} mean gradient")
        ax.fill_between(grid, (m - 2 * se) * 100, (m + 2 * se) * 100, color=BAND, lw=0)
    ax.axhline(0, color=REF, lw=0.8)
    ax.set_xlabel("lap distance [m] (normalised to the median lap)", color=INK)
    ax.set_ylabel("gradient [%]", color=INK)
    ax.legend(frameon=False, fontsize=8)
    fig.suptitle(f"Q2c/G1 {track}: lap-averaged road gradient from GPS altitude (band = +/-2 SE)", color=INK, fontsize=10)
    fig.savefig(os.path.join(OUT, f"g1_gradient_{track.replace(' ', '_')}.png"), dpi=100)
    plt.close(fig)


# ---------------------------------------------------------------- G2

def _pims(key, params, data, state):
    from diagnostics.inspect_cs_readout_evidence import phase_index_map
    r = {"time": state["time"], "s_m": state["s_m"], "moving": state["moving_mask"], "laps": data["laps"]}
    return phase_index_map(r, {"corners": data["corners"]}, params["stability_estimation"])


def _beta_gps(key, params, data, state, course_chan, lag, smooth=False, iso=False):
    B = read_raw_blocks(d2.sessions()[key]["path"], [course_chan, "log_gps_data_age", "log_gps_sats"])
    if course_chan not in B:
        return None, None
    tc, vc, _ = B[course_chan]
    t = state["time"]
    crs = np.interp(t, tc - lag, np.unwrap(np.radians(vc)))
    r = state["yaw_rate_radps"]
    v = state["v_mps"] * 3.6
    mv = state["moving_mask"] & (v > 30)
    dcrs = np.gradient(crs, t)
    m = mv & (np.abs(np.degrees(r)) > 5)
    sign = float(np.sign(np.polyfit(r[m], dcrs[m], 1)[0])) if m.sum() > 100 else 1.0
    I = np.concatenate(([0.0], np.cumsum(0.5 * (sign * r[1:] + sign * r[:-1]) * np.diff(t))))
    ay_g, r_d = state["ay_mps2"] / G, np.degrees(r)
    if smooth:
        ay_g, r_d = _mavg(ay_g, 30), _mavg(r_d, 30)  # 0.3 s at 100 Hz, as in G0
    straight = mv & (np.abs(ay_g) < 0.1) & (np.abs(r_d) < 2) & (v > 100)
    d = np.diff(np.concatenate(([0], straight.astype(int), [0])))
    runs = [(a, b - 1) for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]) if t[b - 1] - t[a] >= 1.0]
    beta = np.full(t.size, np.nan)
    for (a0, a1), (b0, _b1) in zip(runs[:-1], runs[1:]):
        ta, tb = t[a1], t[b0]
        if tb - ta > 60.0:
            continue
        seg = slice(a1, b0 + 1)
        frac = (t[seg] - ta) / (tb - ta)
        h = crs[a1] + I[seg] - I[a1] + frac * ((crs[b0] - crs[a1]) - (I[b0] - I[a1]))
        beta[seg] = crs[seg] - h
    qual = None
    if "log_gps_data_age" in B and "log_gps_sats" in B:
        age = np.interp(t, B["log_gps_data_age"][0], B["log_gps_data_age"][1])
        sats = np.interp(t, B["log_gps_sats"][0], B["log_gps_sats"][1])
        qual = (age < 0.5) & (sats >= 6)
    if iso:
        beta = -beta  # compass (clockwise) -> ISO (left positive)
    return np.degrees(beta), {"sign": sign, "n_anchor_runs": len(runs), "qual": qual}


def g2(smooth=False, out_name="g2.json", delay=None, iso=False):
    g0r = _load("g0.json")
    out = {}
    for key in d2.usable_keys():
        try:
            params, data, state = d2._state(key)
            pims = _pims(key, params, data, state)
            prod_beta = np.degrees(np.asarray(d2.load_fit(key)["prod"]["beta"], float))
            rec = {}
            for src, chan, fam in (("log_gps_course", "log_gps_course", "log_gps"),
                                   ("nmea_cog", "NMEA RX RMC Course Over Ground", "nmea")):
                lat = g0r.get(key, {}).get("latency", {}).get(fam if fam in SPEEDS else "log_gps", {})
                lag = lat.get("lag_s_gps_behind_imu", 0.0) if lat else 0.0
                if delay is not None:
                    lag = delay  # _beta_gps evaluates the course at tc - lag
                bg, info = _beta_gps(key, params, data, state, chan, lag, smooth, iso)
                if bg is None:
                    continue
                per_c = {}
                ql = []
                for (cid, lap), ph in pims.items():
                    idx = ph["apex_3"]
                    if idx.size == 0:
                        continue
                    vals = bg[idx]
                    good = np.isfinite(vals)
                    if info["qual"] is not None:
                        ql.append(bool(good.any() and info["qual"][idx].all()))
                    if good.sum() >= 3:
                        per_c.setdefault(cid, []).append((float(np.mean(vals[good])), float(np.mean(prod_beta[idx]))))
                corner = {}
                for cid, v in per_c.items():
                    g = np.array([x[0] for x in v])
                    p = np.array([x[1] for x in v])
                    corner[str(cid)] = {"n": len(v), "mean_deg": float(g.mean()),
                                        "se_deg": float(g.std(ddof=1) / np.sqrt(len(v))) if len(v) >= 2 else float("nan"),
                                        "prod_beta_mean_deg_UNSCORED": float(p.mean())}
                ses = [c["se_deg"] for c in corner.values() if np.isfinite(c["se_deg"])]
                rec[src] = {"latency_s": lag, "yaw_sign": info["sign"], "anchor_runs": info["n_anchor_runs"],
                            "corners": corner, "median_se_deg": float(np.median(ses)) if ses else float("nan"),
                            "share_corners_se_le_bar": float(np.mean(np.array(ses) <= ANCHOR_BAR_DEG)) if ses else None,
                            "yield_valid_corner_laps": [int(np.sum(ql)), len(ql)] if ql else None}
            out[key] = rec
            print(f"[g2 {key}] " + "; ".join(f"{s}: median SE {r['median_se_deg']:.2f} deg, share<=bar "
                                             f"{r['share_corners_se_le_bar']}, yield {r['yield_valid_corner_laps']}"
                                             for s, r in rec.items()), flush=True)
        except Exception:
            out[key] = {"error": traceback.format_exc()}
            print(f"[g2 {key}] ERROR", flush=True)
    _save(out_name, out)


# ---------------------------------------------------------------- V1 / V2

def _curve(p, g):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    return pacejka_lateral_force(g, p[0], p[1], p[2], p[3])


def _cdiff(p, p0, p99):
    g = np.radians(np.linspace(0.0, p99, 300))
    return float(np.sqrt(np.mean((_curve(p, g) - _curve(p0, g)) ** 2)) / abs(p0[2]) * 100.0)


def _params_of(ax):
    return (ax["B"], ax["C"], ax["D"], ax["E"])


def _scored_keys():
    return [k for k in d2.pool_keys()] + ["dubai"]


def v1():
    from modules.tyre_fit_auto import _fit_axle_pacejka
    params = d2._params()
    prior = d2.load_prior()
    cfgp = params["tyre_fit_auto"]["tyre_fit_prior"]
    prior_eq = {a: max(abs(prior[a][k] / cfgp[a][k] - 1) for k in "BCDE") for a in ("front", "rear")}
    out = {"entry": {"prior_vs_config_max_rel": prior_eq}, "sessions": {}}
    for key in _scored_keys():
        try:
            F = d2.load_fit(key)
            A0 = d2.load_ach(key, 0)
            base = F["base_mask"]
            laps = [l for l in F["laps"] if l.get("is_valid_for_analysis")]
            rec = {"axles": {}}
            for axle in ("front", "rear"):
                a, fy = F[f"alpha_{axle[0]}"], F[f"fy_{axle[0]}"]
                m0 = base & np.isfinite(a) & np.isfinite(fy)
                p99 = float(np.percentile(np.abs(np.degrees(a[m0])), 99))
                pp = _params_of(F["prod"]["manifest"]["axles"][axle])
                pv = _params_of(A0["manifest"]["axles"][axle])
                refit = d2.anchored_fit(a, fy, base, prior[axle], 0.0)
                entry = max(abs(refit[k] / A0["manifest"]["axles"][axle][k] - 1) for k in "BCDE")
                diff = _cdiff(pv, pp, p99)
                boots = []
                lm = [(F["time"] >= l["start_time"]) & (F["time"] <= l["end_time"]) for l in laps]
                for _ in range(N_BOOT_V1):
                    pick = RNG.integers(0, len(laps), len(laps))
                    w = np.zeros(F["time"].size, int)
                    for i in pick:
                        w += lm[i]
                    idx = np.repeat(np.arange(w.size), w)
                    rp = _fit_axle_pacejka(a[idx], fy[idx], base[idx])
                    rv = d2.anchored_fit(a[idx], fy[idx], base[idx], prior[axle], 0.0)
                    boots.append(_cdiff(_params_of(rv), _params_of(rp), p99))
                rec["axles"][axle] = {"prod": pp, "v1": pv, "entry_refit_max_rel": entry, "curve_diff_pct": diff,
                                      "boot_ci95_pct": [float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))],
                                      "prod_converged": bool(F["prod"]["manifest"]["axles"][axle].get("powell_converged", True)),
                                      "v1_converged": bool(A0["manifest"]["axles"][axle]["powell_converged"])}
            rec["material"] = bool(max(r["curve_diff_pct"] for r in rec["axles"].values()) > MAT_BAR)
            rec["ekf"] = {"prod_gate": F["prod"]["gate"], "prod_fallback": F["prod"]["fallback"],
                          "v1_gate": A0["gate"], "v1_fallback": A0["fallback"]}
            out["sessions"][key] = rec
            print(f"[v1 {key}] diff f {rec['axles']['front']['curve_diff_pct']:.2f} r "
                  f"{rec['axles']['rear']['curve_diff_pct']:.2f} % material {rec['material']}", flush=True)
        except Exception:
            out["sessions"][key] = {"error": traceback.format_exc()}
            print(f"[v1 {key}] ERROR", flush=True)
    # fresh-chain cache validity (one session, w 0 and production)
    try:
        params, data, state = d2._state(A_KEY)
        b0 = d2._resolve(A_KEY, params, data, state, d2._fitter(prior, 0.0))[0]
        cs0 = d2._downstream(params, data, state, b0)
        bp = d2._resolve(A_KEY, params, data, state)[0]
        csp = d2._downstream(params, data, state, bp)
        A0 = d2.load_ach(A_KEY, 0)
        Fp = d2.load_fit(A_KEY)
        out["entry"]["fresh_chain_w0_max_abs_cs"] = float(np.nanmax(np.abs(cs0["cs_f"] - A0["down"]["cs_f"])))
        out["entry"]["fresh_chain_prod_max_abs_cs"] = float(np.nanmax(np.abs(csp["cs_f"] - Fp["prod"]["down"]["cs_f"])))
    except Exception:
        out["entry"]["fresh_chain_error"] = traceback.format_exc()
    _save("v1.json", out)
    print("[v1 entry]", json.dumps(out["entry"]), flush=True)


def _scale_fitter(prior):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    order = iter(("front", "rear"))

    def fit(alpha, Fy, base_mask):
        ax = next(order)
        p = prior[ax]
        m = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
        fp = pacejka_lateral_force(alpha[m], p["B"], p["C"], p["D"], p["E"])
        lam = float(np.sum(fp * Fy[m]) / np.sum(fp * fp))
        D = lam * p["D"]
        resid = Fy[m] - lam * fp
        p99 = float(np.percentile(np.abs(np.degrees(alpha[m])), 99))
        pk = d2._peak_deg(p["B"], p["C"], D, p["E"])
        return {"B": p["B"], "C": p["C"], "D": D, "E": p["E"], "lambda": lam, "powell_converged": True,
                "fit_n_samples": int(m.sum()), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
                "peak_alpha_deg": pk, "peak_in_visited_range": bool(pk == pk and pk <= p99),
                "visited_alpha_p99_deg": p99, "sign_ok": D > 0, "residuals": resid, "residual_mask": m}
    return fit


def v2():
    prior = d2.load_prior()
    v1r = _load("v1.json")
    out = {"sessions": {}}
    for key in _scored_keys():
        try:
            F = d2.load_fit(key)
            base = F["base_mask"]
            rec = {"variants": {}}
            for axle in ("front", "rear"):
                a, fy = F[f"alpha_{axle[0]}"], F[f"fy_{axle[0]}"]
                m0 = base & np.isfinite(a) & np.isfinite(fy)
                p99 = float(np.percentile(np.abs(np.degrees(a[m0])), 99))
                pp = _params_of(F["prod"]["manifest"]["axles"][axle])
                for w in WEIGHTS:
                    Aw = d2.load_ach(key, w)
                    pv = _params_of(Aw["manifest"]["axles"][axle])
                    rec["variants"].setdefault(f"w{w:g}", {})[axle] = {
                        "params": pv, "curve_diff_pct": _cdiff(pv, pp, p99),
                        "converged": bool(Aw["manifest"]["axles"][axle]["powell_converged"])}
                fp = prior[axle]
                from modules.tyre_model_pacejka import pacejka_lateral_force
                f_ = pacejka_lateral_force(a[m0], fp["B"], fp["C"], fp["D"], fp["E"])
                lam = float(np.sum(f_ * fy[m0]) / np.sum(f_ * f_))
                ps = (fp["B"], fp["C"], lam * fp["D"], fp["E"])
                rec["variants"].setdefault("scale", {})[axle] = {"params": ps, "lambda": lam,
                                                               "curve_diff_pct": _cdiff(ps, pp, p99), "converged": True}
            for v, r in rec["variants"].items():
                r["material"] = bool(max(r[ax]["curve_diff_pct"] for ax in ("front", "rear")) > MAT_BAR)
            if rec["variants"]["scale"]["material"] or key in (A_KEY, B_KEY):
                params, data, state = d2._state(key)
                beta, man, gate, fb, fr = d2._resolve(key, params, data, state, _scale_fitter(prior))
                down = d2._downstream(params, data, state, beta)
                _pk(f"scale_{key}.pkl.gz", {"beta": np.asarray(beta, np.float32), "manifest": d2._slim_manifest(man),
                                            "gate": gate, "fallback": fb, "down": down})
                rec["variants"]["scale"]["ekf"] = {"gate": gate, "fallback": fb}
            out["sessions"][key] = rec
            print(f"[v2 {key}] " + "; ".join(f"{v} f {r['front']['curve_diff_pct']:.1f} r {r['rear']['curve_diff_pct']:.1f}"
                                             f"{' M' if r['material'] else ''}" for v, r in rec["variants"].items()), flush=True)
        except Exception:
            out["sessions"][key] = {"error": traceback.format_exc()}
            print(f"[v2 {key}] ERROR", flush=True)
    _save("v2.json", out)


# ---------------------------------------------------------------- Q4 courtroom

def _cs_of(key, contestant):
    if contestant == "prod":
        d = d2.load_fit(key)["prod"]["down"]
    elif contestant == "scale":
        p = os.path.join(CACHE, f"scale_{key}.pkl.gz")
        if not os.path.exists(p):
            return None
        d = _pk(f"scale_{key}.pkl.gz")["down"]
    else:
        d = d2.load_ach(key, float(contestant[1:]))["down"]
    return np.asarray(d["cs_f"], float), np.asarray(d["cs_r"], float)


def _readouts(key, cs, params, data, state, pims, lap_list):
    from diagnostics.inspect_cs_readout_evidence import corner_readouts
    r = {"cs": {"A_baseline": cs}}
    gate = params["stability_estimation"]["cs_phase_min_valid_samples"]
    return corner_readouts(r, pims, lap_list, gate)


def _cells(key, cid, cs, params, data, state, pims, laps):
    """{phase: (f, se_f, gap, se_gap)} -- PROD read-out, jackknife SE."""
    from diagnostics.inspect_cs_readout_evidence import PHASES
    sub = {k: v for k, v in pims.items() if k[0] == cid}

    def stat(ls):
        ro = _readouts(key, cs, params, data, state, sub, ls)
        out = {}
        for ph in PHASES:
            if cid not in ro:
                out[ph] = (float("nan"), float("nan"))
                continue
            f, r = ro[cid][ph]["f"]["PROD"], ro[cid][ph]["r"]["PROD"]
            out[ph] = (f, r - f)
        return out

    def jk(vals):
        v = np.array([x for x in vals if np.isfinite(x)])
        n = v.size
        return float(np.sqrt((n - 1) / n * np.sum((v - v.mean()) ** 2))) if n >= 2 else float("nan")
    th = stat(laps)
    loo = [stat([l for l in laps if l != x]) for x in laps]
    return {ph: (th[ph][0], jk([d[ph][0] for d in loo]), th[ph][1], jk([d[ph][1] for d in loo])) for ph in th}


def _rake(contestant, ctx):
    from diagnostics.inspect_cs_value_lens import _brackets
    csA, csB = _cs_of(A_KEY, contestant), _cs_of(B_KEY, contestant)
    if csA is None or csB is None:
        return None
    (pA, dA, sA, pimA, lA), (pB, dB, sB, pimB, lB) = ctx[A_KEY], ctx[B_KEY]
    ba, bb = _brackets(dA), _brackets(dB)
    pairs = []
    for ca, (a0, a1) in sorted(ba.items()):
        for cb, (b0, b1) in bb.items():
            if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0):
                pairs.append((ca, cb))
                break
    groups = {"entry": ("entry_1_brake", "entry_2_turnin"), "mid": ("apex_3",), "exit": ("exit_4", "exit_5")}
    want = {"entry": -1, "mid": +1, "exit": -1}  # sign of gap the driver statement implies in A
    res = {"a_front": {"expected": 0, "opposite": 0, "cells": 0}, "b_rear": {"consistent": 0, "against": 0, "cells": 0},
           "detail": [], "iii": {g: {"match": 0, "n": 0, "less_extreme": 0, "n_ab": 0} for g in groups}}
    for ca, cb in pairs:
        ca_cells = _cells(A_KEY, ca, csA, pA, dA, sA, pimA, lA)
        cb_cells = _cells(B_KEY, cb, csB, pB, dB, sB, pimB, lB)
        for ph, (fa, sfa, ga, sga) in ca_cells.items():
            fb, sfb, gb, sgb = cb_cells[ph]
            grp = next(g for g, ps in groups.items() if ph in ps)
            if np.isfinite(ga):
                res["iii"][grp]["n"] += 1
                res["iii"][grp]["match"] += int(np.sign(ga) == want[grp])
                if np.isfinite(gb):
                    res["iii"][grp]["n_ab"] += 1
                    res["iii"][grp]["less_extreme"] += int(abs(gb) < abs(ga))
            if not all(np.isfinite([fa, fb, ga, gb, sfa, sfb, sga, sgb])) or ga == 0:
                continue
            gap_res = abs(gb - ga) > 2 * np.sqrt(sga ** 2 + sgb ** 2)
            if ga > 0:
                res["a_front"]["cells"] += 1
                if abs(fb - fa) > 2 * np.sqrt(sfa ** 2 + sfb ** 2):
                    k = "expected" if fb > fa else "opposite"
                    res["a_front"][k] += 1
                    res["detail"].append(f"C{ca} {ph} f {fa:+.2f}->{fb:+.2f} {k}")
                if gap_res:
                    k = "expected" if gb < ga else "opposite"
                    res["a_front"][k] += 1
                    res["detail"].append(f"C{ca} {ph} gap {ga:+.2f}->{gb:+.2f} {k}")
            else:
                res["b_rear"]["cells"] += 1
                if gap_res:
                    k = "consistent" if gb > ga else "against"
                    res["b_rear"][k] += 1
                    res["detail"].append(f"C{ca} {ph} rear gap {ga:+.2f}->{gb:+.2f} {k}")
    res["score"] = res["a_front"]["expected"] - res["a_front"]["opposite"]
    res["opposite"] = res["a_front"]["opposite"] + res["b_rear"]["against"]
    contra = [g for g, v in res["iii"].items() if v["n"] and v["match"] / v["n"] < 0.5]
    res["iii_contradicted_groups"] = contra
    res["iii_contradicted"] = len(contra) >= 2
    return res


def _coherence(key, cs, ctx_k, boot=0):
    from diagnostics.inspect_cs_readout_evidence import PHASES
    params, data, state, pims, laps = ctx_k
    gate = params["stability_estimation"]["cs_phase_min_valid_samples"]

    def metric(lap_set):
        iqrs = []
        for cid in sorted({c for (c, l) in pims}):
            for ph in PHASES:
                for ax_i in (0, 1):
                    vals = []
                    for l in lap_set:
                        idx = pims.get((cid, l), {}).get(ph)
                        if idx is None or idx.size == 0:
                            continue
                        v = cs[ax_i][idx]
                        v = v[np.isfinite(v)]
                        if v.size >= gate:
                            vals.append(np.median(v))
                    if len(vals) >= 3:
                        iqrs.append(np.subtract(*np.percentile(vals, [75, 25])))
        return float(np.median(iqrs)) if iqrs else float("nan")
    m = metric(laps)
    if not boot:
        return m, None
    bs = [metric(list(RNG.choice(laps, len(laps), replace=True))) for _ in range(boot)]
    bs = [b for b in bs if np.isfinite(b)]
    return m, (float(np.percentile(bs, 97.5) - np.percentile(bs, 2.5)) / 2 if bs else float("nan"))


def q4():
    from diagnostics.trace_render import render_cs_traces
    v1r, v2r = _load("v1.json"), _load("v2.json")
    contestants = ["prod", "w0", "w0.1", "w1", "w10", "scale"]
    scored = {"prod": set()}
    for k, r in v1r["sessions"].items():
        if r.get("material"):
            scored.setdefault("w0", set()).add(k)
    for k, r in v2r["sessions"].items():
        for v, vr in r.get("variants", {}).items():
            if vr.get("material"):
                scored.setdefault(v if v == "scale" else v, set()).add(k)
    for c in contestants:
        scored.setdefault(c, set()).update({A_KEY, B_KEY})
    union = set().union(*scored.values())
    scored["prod"] = set(union)  # production is rendered and measured wherever any variant is scored
    ctx = {}
    for k in union:
        params, data, state = d2._state(k)
        laps = [l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")]
        ctx[k] = (params, data, state, _pims(k, params, data, state), laps)
    out = {"scored_sessions": {c: sorted(s) for c, s in scored.items()}, "contestants": {}, "renders": []}
    prod_coh = {}
    for k in union:
        cs = _cs_of(k, "prod")
        prod_coh[k] = _coherence(k, cs, ctx[k], boot=N_BOOT_COH)
    for c in contestants:
        rec = {}
        rake_ctx = {k: (ctx[k][0], ctx[k][1], ctx[k][2], ctx[k][3], ctx[k][4]) for k in (A_KEY, B_KEY)}
        rec["ii_rake"] = _rake(c, rake_ctx)
        rk = rec["ii_rake"]
        rec["iii_per_phase"] = ({"groups": rk["iii"], "contradicted_groups": rk["iii_contradicted_groups"],
                                 "contradicted": rk["iii_contradicted"],
                                 "confidence": "author's recollection of the driver's account, recorded 2026-10-02"}
                                if rk else None)
        coh = {}
        for k in sorted(scored[c]):
            cs = _cs_of(k, c)
            if cs is None:
                continue
            m, _ = _coherence(k, cs, ctx[k])
            coh[k] = {"metric": m, "prod": prod_coh[k][0], "prod_halfwidth": prod_coh[k][1]}
            path = os.path.join(OUT, f"cs_{c}_{k}.png")
            render_cs_traces(path, ctx[k][2]["time"], ctx[k][2]["s_m"], cs[0], cs[1], ctx[k][1]["laps"],
                             ctx[k][1]["corners"], f"Q3/Q4 {c} {k}: CS_ratio")
            out["renders"].append(path)
        ex = [v["metric"] - v["prod"] for v in coh.values() if np.isfinite(v["metric"]) and np.isfinite(v["prod"])]
        hw = [v["prod_halfwidth"] for v in coh.values() if v["prod_halfwidth"] is not None and np.isfinite(v["prod_halfwidth"])]
        rec["v_coherence"] = {"per_session": coh, "mean_excess": float(np.mean(ex)) if ex else float("nan"),
                              "mean_prod_halfwidth": float(np.mean(hw)) if hw else float("nan")}
        rec["v_worse"] = bool(ex and hw and np.mean(ex) > np.mean(hw))
        out["contestants"][c] = rec
        print(f"[q4 {c}] (ii) {rk['score'] if rk else None} opp {rk['opposite'] if rk else None}; "
              f"(iii) {rec['iii_per_phase']}; (v) excess {rec['v_coherence']['mean_excess']:.3f} "
              f"vs hw {rec['v_coherence']['mean_prod_halfwidth']:.3f}", flush=True)
    # decision
    p_score = out["contestants"]["prod"]["ii_rake"]["score"]
    for c, rec in out["contestants"].items():
        if c == "prod":
            rec["decision"] = "BASELINE"
            continue
        rk = rec["ii_rake"]
        if rk is None:
            rec["decision"] = "NOT SCOREABLE (no chain on the rake pair)"
            continue
        mech = _mechanics(c, v1r, v2r)
        if c == "w0":
            # (i) "start-insensitive within the bar or the material difference
            # is explained" -- explanation cannot be automated: material
            # sessions are listed for the author's review, not auto-failed
            mech["start_sensitive_sessions"] = sorted(k for k, r in v1r["sessions"].items() if r.get("material"))
        rec["i_mechanics"] = mech
        if rk["score"] <= p_score - 2:
            rec["decision"] = "DISQUALIFIED (symmetric guard, (ii) <= prod - 2)"
        elif (mech["pass"] and rk["score"] >= p_score + 2 and rk["opposite"] == 0 and not rec["v_worse"]
              and not (rec["iii_per_phase"] or {}).get("contradicted")):
            rec["decision"] = "ADOPTION-PROPOSAL-WORTHY"
        else:
            rec["decision"] = "NOT EARNED"
    _save("q4.json", out)
    for c, rec in out["contestants"].items():
        print(f"[q4 decision] {c}: {rec['decision']}", flush=True)


def _mechanics(c, v1r, v2r):
    conv, fb = [], []
    for k, r in (v1r["sessions"] if c == "w0" else v2r["sessions"]).items():
        if "error" in r:
            continue
        if c == "w0":
            conv += [r["axles"][a]["v1_converged"] for a in ("front", "rear")]
            fb.append(bool(r["ekf"]["v1_fallback"]))
        else:
            vr = r["variants"].get(c)
            if vr:
                conv += [vr[a]["converged"] for a in ("front", "rear")]
                if "ekf" in vr:
                    fb.append(bool(vr["ekf"]["fallback"]))
    return {"converged_share": float(np.mean(conv)) if conv else None, "ekf_fallbacks": int(sum(fb)),
            "pass": bool(conv and all(conv) and not any(fb))}


if __name__ == "__main__":
    mode = sys.argv[1]
    steps = {"g0": g0, "g1": g1, "g2": g2, "v1": v1, "v2": v2, "q4": q4,
             "g2r": lambda: g2(smooth=True, out_name="g2_repair.json"),
             "g2r2": lambda: g2(smooth=True, out_name="g2_repair2.json", delay=G2_DELAY_S, iso=True)}
    for name in (("g0", "g1", "g2", "v1", "v2", "q4") if mode == "all" else (mode,)):
        try:
            steps[name]()
        except Exception:
            print(f"[{name}] STEP ERROR\n{traceback.format_exc()}", flush=True)
