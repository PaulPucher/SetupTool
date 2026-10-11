# Kinematic-integration input audit (author work order 2026-10-07).
# MEASUREMENT ONLY: reports numbers, draws no conclusions, changes nothing
# in the pipeline. Verdicts are the author's.
#
# The kinematic sideslip integrates d beta/dt = ay / vx - r (PRODUCTION.md
# sec. 3, Rajamani ch. 2). A constant error e in that difference grows as
# e * t; a gain error grows with the integrated signal. This script measures
# the error content of the three inputs on one Nurburgring export, using the
# exact arrays the integrator reads (stability_analysis.prepare_vehicle_state:
# ecu_speed, sclu_yaw_rate, log_acc_y on the common grid, no filtering).
#
# 1. Straight-line reference. Straights are defined by STEERING ONLY
#    (log_asteer, steering-wheel angle): |angle - centre| < a_thr, |steering
#    rate| < r_thr, vx >= v_min, duration >= d_min, analysis laps, off
#    kerb-affected samples. Segment selection must be independent of the
#    audited IMU signals (steering is a different sensor), otherwise a biased
#    sensor selects its own reference. Corner gaps are not used: corner
#    boundaries hang on thresholds and are not steering-quiet.
#    The centre is the straight-line steering angle measured on the file (the
#    mode of the angle distribution at speed); the literal zero-centred
#    variant is reported beside it as a sensitivity line.
# 2. Quasi-steady cornering reference: clearly turning (|angle - centre| >=
#    CORNER_ANGLE_FACTOR x a_thr) and steadier than the typical cornering
#    sample of this file: |steering rate| and |dvx/dt| both below their
#    median over the turning population (a race car rarely holds speed in a
#    corner, so an absolute steadiness bar leaves almost nothing), >=
#    STEADY_MIN_S, off kerb-affected samples. Steering and ecu_speed only --
#    the selection never reads yaw rate or ay. In steady state beta_dot = 0,
#    so ay / vx = r; nested models of y = ay / vx against x = r:
#    M0 y = x + c0, M1 y = c1 x + c0, M2 y = c2 x^2 + c1 x + c0.
#    Uncertainty by a segment bootstrap (samples within a segment are not
#    independent at 100 Hz).
# 3. Integration sensitivity: the identity residual e(x) = y - x is exactly
#    the beta rate the integrator adds where the true beta_dot is 0; each
#    fitted term is integrated over 5 s and 10 s.
# 4. Speed sanity: vx (ecu_speed) against the front wheel speeds (undriven).
#
# Kerb-affected = kerb mask or the grade zone (state kerb_smear_zone), the
# one kerb-exclusion definition. Thresholds marked DERIVED are computed from
# the file and printed; the fixed numbers below are method choices.
#
# The steering centre is always measured per session (peak of the steering
# distribution at speed), never hardcoded and never assumed zero (author
# ruling 2026-10-07).
#
# Diagnostic 2 (2026-10-07): --file may repeat (offsets and M1 per export,
# then one cross-session list); --filter-check runs the production washout
# on held-beta steps and compares production beta with a raw integration of
# offset-corrected inputs on the longest sustained-lateral corner of the
# first export. Dubai and Le Mans are out of scope for this matter.
#
# Run: python -m diagnostics.inspect_kinematic_inputs [--file PATH ...] [--png] [--filter-check]

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DEFAULT_FILE = "D:/Data_Bachelorthesis/unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"
OUT_DIR = os.path.join("diagnostics", "output", "kinematic_inputs")
IMU_REPORT = os.path.join("diagnostics", "plots", "imu_offsets", "report.json")

# method choices (not tuned to a result)
HIST_BIN_DEG = 0.2          # steering-angle histogram bin for the centre
SIGMA_FACTOR = 2.0          # a_thr, r_thr = this x robust sigma of the straight peak
RATE_SMOOTH_S = 0.2         # moving average before differentiating steering angle
D_MIN_CANDIDATES_S = (1.0, 2.0, 3.0, 4.0)
D_MIN_SEGMENTS_PER_LAP = 1  # d_min = longest candidate leaving >= this many on every analysis lap
CORNER_ANGLE_FACTOR = 5.0   # quasi-steady: |angle - centre| >= this x a_thr
STEADY_MIN_S = 0.5
STEADY_V_MIN_MPS = 15.0
BOOTSTRAP_N = 500
HORIZONS_S = (5.0, 10.0)
HOLDS_S = (3.0, 6.0, 10.0)          # diagnostic 2: representative held-corner durations
SUSTAINED_AY_MPS2 = 10.0            # diagnostic 2: "sustained lateral" = |ay| >= ~1 g
MPH_TO_MPS = 0.44704


def track_label(path):
    # plain-language track for plot titles, from the export's file name
    name = os.path.basename(path).upper()
    if "NBR" in name:
        return "Nurburgring GP"
    if "PRC" in name or "SE07_US" in name:
        return "Paul Ricard"
    return "Track unknown"


def robust_sigma(x):
    x = np.asarray(x, dtype=float)
    return 1.4826 * float(np.median(np.abs(x - np.median(x))))


def runs(mask):
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def moving_average(x, n):
    return np.convolve(x, np.ones(n) / n, mode="same")


def pct(x, qs=(5, 25, 50, 75, 95)):
    return ", ".join(f"p{q} {v:+.3f}" for q, v in zip(qs, np.percentile(x, qs)))


def load(path):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    parsed = parse_csv(path)
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))
    return parsed, prepare_vehicle_state(parsed["channels"], params)


def channel_facts(ch, names):
    out = []
    for k in names:
        c = ch.get(k)
        if c is None:
            out.append(f"{k}: absent")
            continue
        t = c.get("time")
        rate = (len(t) - 1) / (t[-1] - t[0]) if t is not None and len(t) > 1 else float("nan")
        d = np.asarray(c["data"], dtype=float)
        out.append(f"{k}: unit {c.get('unit_raw')}, quality {c.get('quality')}, {rate:.0f} Hz, "
                   f"std {np.nanstd(d):.3f}, range {np.nanmin(d):.2f}..{np.nanmax(d):.2f}")
    return out


def segment_bootstrap(xs, ys, seg_ids, degree, rng):
    uniq = np.unique(seg_ids)
    by = {s: np.where(seg_ids == s)[0] for s in uniq}
    coefs = []
    for _ in range(BOOTSTRAP_N):
        pick = np.concatenate([by[s] for s in rng.choice(uniq, size=uniq.size, replace=True)])
        if degree == 0:
            coefs.append([np.mean(ys[pick] - xs[pick])])
        else:
            coefs.append(np.polyfit(xs[pick], ys[pick], degree))
    return np.std(np.array(coefs), axis=0)


def steering_straights(st, laps, lap_id, kerb, verbose=False):
    """Steering-defined straights of one session (the definition in the
    header): thresholds derived from steering and speed only, centre measured
    per session. Shared with compare_beta_architectures.py."""
    sr = st["sample_rate_hz"]
    v = st["v_mps"]
    sw = np.degrees(st["steer_sw_rad"])
    an = lap_id >= 0
    v_min = float(np.floor(np.percentile(v[an], 25) / 5.0) * 5.0)
    fast = an & (v >= v_min) & (np.abs(sw) < 20)
    h, e = np.histogram(sw[fast], bins=np.arange(-20, 20 + HIST_BIN_DEG / 2, HIST_BIN_DEG))
    centres = 0.5 * (e[1:] + e[:-1])
    centre = float(centres[np.argmax(h)])
    peak = h >= h.max() / 2
    lo, hi = centres[peak].min(), centres[peak].max()
    a_thr = round(SIGMA_FACTOR * robust_sigma(sw[fast & (sw >= lo) & (sw <= hi)]), 1)
    rate = np.gradient(moving_average(sw, max(1, int(RATE_SMOOTH_S * sr))), 1.0 / sr)
    near = fast & (np.abs(sw - centre) < a_thr)
    r_thr = round(SIGMA_FACTOR * robust_sigma(rate[near]), 1)

    def straights(c0, d_min):
        m = an & ~kerb & (v >= v_min) & (np.abs(sw - c0) < a_thr) & (np.abs(rate) < r_thr)
        return [(a, b) for a, b in runs(m) if (b - a) / sr >= d_min and lap_id[a] == lap_id[b - 1]]

    counts = {}
    for d in D_MIN_CANDIDATES_S:
        segs = straights(centre, d)
        per_lap = [sum(1 for a, _b in segs if lap_id[a] == l["lap_number"]) for l in laps]
        counts[d] = (len(segs), min(per_lap) if per_lap else 0)
    ok = [d for d in D_MIN_CANDIDATES_S if counts[d][1] >= D_MIN_SEGMENTS_PER_LAP]
    d_min = max(ok) if ok else min(D_MIN_CANDIDATES_S)
    d_min_note = "" if ok else " -- NO candidate meets it; shortest used"

    if verbose:
        print("\nTHRESHOLDS (derived from steering and speed, never from the audited IMU signals)")
        print(f"  - v_min = {v_min:.0f} m/s: 25th percentile of analysis-lap speed, floored to 5 m/s")
        print(f"  - steering centre = {centre:+.1f} deg steering-wheel angle: mode of the angle at v >= v_min "
              f"(half-max width {lo:+.1f}..{hi:+.1f} deg)")
        print(f"  - a_thr = {a_thr:.1f} deg: {SIGMA_FACTOR:g} x robust sigma of the angle inside that peak")
        print(f"  - r_thr = {r_thr:.1f} deg/s: {SIGMA_FACTOR:g} x robust sigma of the steering rate "
              f"({RATE_SMOOTH_S:g} s moving average, differentiated) inside |angle - centre| < a_thr")
        print("  - d_min candidates (segments, fewest per analysis lap): "
              + "; ".join(f"{d:g} s -> {counts[d][0]} / {counts[d][1]}" for d in D_MIN_CANDIDATES_S))
        print(f"  - d_min = {d_min:g} s: longest candidate leaving >= {D_MIN_SEGMENTS_PER_LAP} segment(s) on every "
              f"analysis lap{d_min_note}")
    return {"v_min": v_min, "centre": centre, "a_thr": a_thr, "r_thr": r_thr, "d_min": d_min, "rate": rate,
            "straights": straights, "segs": straights(centre, d_min)}


def quasi_steady_segments(st, lap_id, kerb, sd):
    """Quasi-steady cornering segments (measurement 2 criterion): steering and
    ecu_speed only, limits = medians over the session's turning population.
    Shared with compare_beta_architectures.py."""
    sr = st["sample_rate_hz"]
    v = st["v_mps"]
    sw = np.degrees(st["steer_sw_rad"])
    rate = sd["rate"]
    an = lap_id >= 0
    dvdt = np.gradient(moving_average(v, max(1, int(RATE_SMOOTH_S * sr))), 1.0 / sr)
    turning = an & ~kerb & (v >= STEADY_V_MIN_MPS) & (np.abs(sw - sd["centre"]) >= CORNER_ANGLE_FACTOR * sd["a_thr"])
    rate_c = float(np.median(np.abs(rate[turning])))
    dvdt_c = float(np.median(np.abs(dvdt[turning])))
    q = turning & (np.abs(rate) < rate_c) & (np.abs(dvdt) < dvdt_c)
    return {"turning": turning, "rate_c": rate_c, "dvdt_c": dvdt_c,
            "segs": [(a, b) for a, b in runs(q) if (b - a) / sr >= STEADY_MIN_S]}


def audit_file(path, png):
    """Measurements 1-4 on one export; prints the report, returns the
    per-session summary for the cross-session list."""
    parsed, st = load(path)
    ch = parsed["channels"]
    t = st["time"]
    sr = st["sample_rate_hz"]
    v = st["v_mps"]
    r = np.degrees(st["yaw_rate_radps"])          # deg/s
    ay = st["ay_mps2"]                             # m/s^2
    sw = np.degrees(st["steer_sw_rad"])            # steering-wheel angle, deg
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    laps = [l for l in parsed["laps"] if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in laps:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    an = lap_id >= 0

    print("KINEMATIC-INTEGRATION INPUT AUDIT -- measurement only, no conclusions")
    print(f"export: {path}")
    print(f"grid {sr} Hz; laps {len(parsed['laps'])}, analysis laps {[l['lap_number'] for l in laps]}")
    print("\nSTEP 0 -- channels (read from this file)")
    for line in channel_facts(ch, ("log_asteer", "ecu_speed", "sclu_yaw_rate", "log_acc_y",
                                   "log_speed_fl", "log_speed_fr", "log_speed_rl", "log_speed_rr",
                                   "abs_speed_fl", "abs_speed_fr")):
        print("  - " + line)

    sd = steering_straights(st, laps, lap_id, kerb, verbose=True)
    v_min, centre, a_thr, r_thr, d_min, rate = sd["v_min"], sd["centre"], sd["a_thr"], sd["r_thr"], sd["d_min"], sd["rate"]
    straights, segs = sd["straights"], sd["segs"]
    seg_rows = []
    for a, b in segs:
        sl = slice(a, b)
        seg_rows.append({"lap": int(lap_id[a]), "t_mid": float(t[(a + b) // 2]), "dur": (b - a) / sr,
                         "v": float(np.mean(v[sl])), "r_mean": float(np.mean(r[sl])), "r_std": float(np.std(r[sl])),
                         "ay_mean": float(np.mean(ay[sl])), "ay_std": float(np.std(ay[sl]))})
    rm = np.array([s["r_mean"] for s in seg_rows])
    aym = np.array([s["ay_mean"] for s in seg_rows])
    dur = np.array([s["dur"] for s in seg_rows])
    tm = np.array([s["t_mid"] for s in seg_rows])

    print("\nMEASUREMENT 1 -- straight-line reference")
    print(f"  - segments {len(seg_rows)}, total {dur.sum():.1f} s")
    print("  - per lap (segments, seconds, yaw rate mean deg/s, ay mean m/s^2 and g):")
    for l in laps:
        k = np.array([s["lap"] == l["lap_number"] for s in seg_rows], dtype=bool)
        if not k.any():
            print(f"      lap {l['lap_number']}: no segment")
            continue
        w = dur[k]
        rl, al = np.average(rm[k], weights=w), np.average(aym[k], weights=w)
        print(f"      lap {l['lap_number']}: {k.sum()} segments, {w.sum():.1f} s, yaw {rl:+.3f}, "
              f"ay {al:+.3f} ({al / 9.81:+.4f} g)")
    for name, x, unit in (("yaw rate", rm, "deg/s"), ("ay", aym, "m/s^2")):
        slope, intercept = np.polyfit(tm - tm[0], x, 1)
        res = x - (slope * (tm - tm[0]) + intercept)
        se = np.sqrt(np.sum(res ** 2) / max(1, x.size - 2) / np.sum((tm - tm.mean()) ** 2))
        extra = f" ({np.average(x, weights=dur) / 9.81:+.4f} g)" if unit == "m/s^2" else ""
        print(f"  - {name} [{unit}] over segment means: duration-weighted mean {np.average(x, weights=dur):+.3f}{extra}, "
              f"median {np.median(x):+.3f}, std {np.std(x):.3f}; drift {slope * 600:+.3f} per 10 min "
              f"(SE {se * 600:.3f})")
    print(f"  - within-segment std, median over segments: yaw {np.median([s['r_std'] for s in seg_rows]):.3f} deg/s, "
          f"ay {np.median([s['ay_std'] for s in seg_rows]):.3f} m/s^2")
    print(f"  - ay distribution, segment means [m/s^2]: {pct(aym)}")
    smask = np.zeros(t.size, bool)
    for a, b in segs:
        smask[a:b] = True
    print(f"  - ay distribution, all straight samples [m/s^2]: {pct(ay[smask])}")
    lit = straights(0.0, d_min)
    if lit:
        lm = np.array([np.mean(r[a:b]) for a, b in lit])
        lw = np.array([(b - a) / sr for a, b in lit])
        la = np.array([np.mean(ay[a:b]) for a, b in lit])
        print(f"  - sensitivity, zero-centred steering window: {len(lit)} segments, {lw.sum():.1f} s, "
              f"yaw mean {np.average(lm, weights=lw):+.3f} deg/s, ay mean {np.average(la, weights=lw):+.3f} m/s^2")
    else:
        print("  - sensitivity, zero-centred steering window: no segment")
    if os.path.exists(IMU_REPORT):
        rep = json.load(open(IMU_REPORT, encoding="utf-8"))
        print("  - cross-check, 2026-09-30 experiment (a) (read, not re-run; its straights = outside corner brackets):")
        for key, sess in rep.items():
            if not key.startswith("nbr"):
                continue
            p1 = sess.get("p1", {})
            print(f"      {key}: yaw S0 median {p1['r']['S0']['median']:+.3f} deg/s, "
                  f"ay S0 median {p1['ay']['S0']['median']:+.3f} m/s^2")

    # ---- measurement 2: quasi-steady cornering
    qs = quasi_steady_segments(st, lap_id, kerb, sd)
    turning, rate_c, dvdt_c, qsegs = qs["turning"], qs["rate_c"], qs["dvdt_c"], qs["segs"]
    idx = np.concatenate([np.arange(a, b) for a, b in qsegs]) if qsegs else np.array([], int)
    seg_id = np.concatenate([np.full(b - a, i) for i, (a, b) in enumerate(qsegs)]) if qsegs else np.array([], int)
    x = r[idx]
    y = np.degrees(ay[idx] / v[idx])
    print("\nMEASUREMENT 2 -- quasi-steady cornering, y = ay / vx against x = yaw rate (both deg/s)")
    print(f"  - criterion: analysis laps, off kerb-affected samples, vx >= {STEADY_V_MIN_MPS:g} m/s, "
          f"|angle - centre| >= {CORNER_ANGLE_FACTOR:g} x a_thr = {CORNER_ANGLE_FACTOR * a_thr:.1f} deg, "
          f"|steering rate| < {rate_c:.1f} deg/s and |dvx/dt| < {dvdt_c:.2f} m/s^2 (DERIVED: medians over the "
          f"{turning.sum() / sr:.0f} s turning population; speed from ecu_speed), >= {STEADY_MIN_S:g} s")
    print(f"  - {len(qsegs)} segments, {idx.size / sr:.1f} s, {idx.size} samples; |x| median {np.median(np.abs(x)):.2f}, "
          f"p90 {np.percentile(np.abs(x), 90):.2f} deg/s; left (x > 0) {int((x > 0).sum())} / right {int((x < 0).sum())} samples")
    rng = np.random.default_rng(0)
    fits = {}
    for name, deg in (("M0", 0), ("M1", 1), ("M2", 2)):
        if deg == 0:
            coef = np.array([np.mean(y - x)])
            pred = x + coef[0]
        else:
            coef = np.polyfit(x, y, deg)
            pred = np.polyval(coef, x)
        se = segment_bootstrap(x, y, seg_id, deg, rng)
        res = y - pred
        k = deg + 1
        bic = idx.size * np.log(np.mean(res ** 2)) + k * np.log(idx.size)
        fits[name] = (coef, se, res)
        labels = {0: ["c0"], 1: ["c1", "c0"], 2: ["c2", "c1", "c0"]}[deg]
        terms = ", ".join(f"{lab} = {c:+.4f} +/- {s:.4f}" for lab, c, s in zip(labels, coef, se))
        bins = np.percentile(x, np.linspace(0, 100, 11))
        binned = [np.mean(res[(x >= bins[i]) & (x <= bins[i + 1])]) for i in range(10)]
        print(f"  - {name}: {terms} (segment bootstrap SE); residual std {np.std(res):.3f} deg/s; naive BIC {bic:.0f}")
        print(f"      residual mean per yaw-rate decile (low to high x): "
              + ", ".join(f"{b:+.3f}" for b in binned)
              + f"; corr(res, x) {np.corrcoef(res, x)[0, 1]:+.3f}, corr(res, x^2) {np.corrcoef(res, x ** 2)[0, 1]:+.3f}")

    # left / right turns separately (x > 0 = left, counter-clockwise yaw positive)
    lr_fits = {}
    for side, m in (("left", x > 0), ("right", x < 0)):
        segs_side = np.unique(seg_id[m])
        if m.sum() < 50 or segs_side.size < 3:
            print(f"  - M1 {side} turns: too few ({int(m.sum())} samples, {segs_side.size} segments)")
            continue
        coef = np.polyfit(x[m], y[m], 1)
        se_lr = segment_bootstrap(x[m], y[m], seg_id[m], 1, rng)
        lr_fits[side] = (coef, se_lr, int(m.sum()), int(segs_side.size))
        print(f"  - M1 {side} turns ({int(m.sum())} samples, {segs_side.size} segments): "
              f"c1 = {coef[0]:+.4f} +/- {se_lr[0]:.4f}, c0 = {coef[1]:+.4f} +/- {se_lr[1]:.4f}")

    # ---- measurement 3: integration sensitivity
    print("\nMEASUREMENT 3 -- beta error per term, integrated over cornering time (deg)")
    xm, xp = float(np.median(np.abs(x))), float(np.percentile(np.abs(x), 90))
    c0_0 = fits["M0"][0][0]
    c1_1, c0_1 = fits["M1"][0]
    c2_2, c1_2, c0_2 = fits["M2"][0]
    v_corner = float(np.median(v[idx]))
    rb = np.average(rm, weights=dur)
    ab = np.average(aym, weights=dur)
    for T in HORIZONS_S:
        print(f"  - T = {T:g} s:")
        print(f"      M0 offset c0: {c0_0 * T:+.2f}")
        for xr, lab in ((xm, "median"), (xp, "p90")):
            print(f"      M1 at |x| = {xr:.2f} deg/s ({lab}): offset {c0_1 * T:+.2f}, gain (c1 - 1) x: "
                  f"{(c1_1 - 1) * xr * T:+.2f} for x > 0, {(c1_1 - 1) * -xr * T:+.2f} for x < 0")
            print(f"      M2 at |x| = {xr:.2f} deg/s ({lab}): offset {c0_2 * T:+.2f}, gain {(c1_2 - 1) * xr * T:+.2f} "
                  f"(x > 0) / {(c1_2 - 1) * -xr * T:+.2f} (x < 0), square c2 x^2 {c2_2 * xr ** 2 * T:+.2f}")
        print(f"      straight-line offsets: yaw -r_bias {-rb * T:+.2f}; ay_bias / vx at the median cornering "
              f"vx {v_corner:.1f} m/s {np.degrees(ab / v_corner) * T:+.2f}")

    # ---- measurement 4: speed sanity
    print("\nMEASUREMENT 4 -- vx (ecu_speed) against front wheel speeds")
    for fam in ("log_speed", "abs_speed"):
        names = [f"{fam}_fl", f"{fam}_fr"]
        if not all(ch.get(n) is not None and ch[n].get("quality") not in ("missing", "failed") for n in names):
            print(f"  - {fam}: not available")
            continue
        conv = []
        for n in names:
            unit = (ch[n].get("unit_raw") or "").lower()
            f = 1 / 3.6 if unit in ("kph", "km/h") else MPH_TO_MPS if unit == "mph" else None
            if f is None:
                conv = None
                print(f"  - {fam}: unknown unit {unit!r}, skipped")
                break
            conv.append(np.interp(t, ch[n]["time"], ch[n]["data"]) * f)
        if conv is None:
            continue
        wf = 0.5 * (conv[0] + conv[1])
        for lab, m in (("straight segments", smask), ("all analysis-lap samples, vx >= v_min", an & (v >= v_min))):
            dv = (v[m] - wf[m]) * 3.6
            rel = 100 * (v[m] - wf[m]) / wf[m]
            print(f"  - {fam} ({ch[names[0]].get('unit_raw')}), {lab}: vx - wheel [km/h] {pct(dv)}; [%] {pct(rel)}")

    if png:
        plots(path, st, laps, lap_id, segs, x, y, fits, seg_rows, v, sw, centre, a_thr)
    return {
        "name": os.path.basename(path), "centre": centre, "a_thr": a_thr, "r_thr": r_thr, "d_min": d_min,
        "n_straight": len(seg_rows), "straight_s": float(dur.sum()),
        "yaw": float(np.average(rm, weights=dur)), "yaw_sd": float(np.std(rm)),
        "ay": float(np.average(aym, weights=dur)), "ay_sd": float(np.std(aym)),
        "n_corner_seg": len(qsegs), "corner_s": float(idx.size / sr),
        "c1": float(fits["M1"][0][0]), "c1_se": float(fits["M1"][1][0]),
        "c0": float(fits["M1"][0][1]), "c0_se": float(fits["M1"][1][1]),
        "n_left": int((x > 0).sum()), "n_right": int((x < 0).sum()), "lr": lr_fits,
    }


def plots(path, st, laps, lap_id, segs, x, y, fits, seg_rows, v, sw, centre, a_thr):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    name = os.path.basename(path).replace(".txt", "")
    title = f"{track_label(path)}, export {name}"
    rng = np.random.default_rng(1)
    pick = rng.choice(x.size, size=min(x.size, 6000), replace=False)
    grid = np.linspace(x.min(), x.max(), 200)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.scatter(x[pick], y[pick], s=2, alpha=0.3, color="#888888", label="quasi-steady cornering samples")
    ax.plot(grid, grid, color="black", lw=1, label="identity: lateral acc. / speed = yaw rate")
    ax.plot(grid, grid + fits["M0"][0][0], lw=1.2, label="constant offset model")
    ax.plot(grid, np.polyval(fits["M1"][0], grid), lw=1.2, label="gain + offset model")
    ax.plot(grid, np.polyval(fits["M2"][0], grid), lw=1.2, ls="--", label="gain + offset + square model")
    ax.set_xlabel("yaw rate [deg/s]")
    ax.set_ylabel("lateral acceleration / speed [deg/s]")
    ax.set_title(f"{title}\nlateral acceleration over speed vs yaw rate, quasi-steady cornering", fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"identity_fits_{name}.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    bins = np.percentile(x, np.linspace(0, 100, 21))
    mids = 0.5 * (bins[1:] + bins[:-1])
    for label, key in (("constant offset model", "M0"), ("gain + offset model", "M1"),
                       ("gain + offset + square model", "M2")):
        res = fits[key][2]
        ax.plot(mids, [np.mean(res[(x >= bins[i]) & (x <= bins[i + 1])]) for i in range(20)], marker="o", ms=3,
                label=label)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xlabel("yaw rate [deg/s] (bin centres, 20 equal-count bins)")
    ax.set_ylabel("mean residual [deg/s]")
    ax.set_title(f"{title}\nresidual of each error model against yaw rate", fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"residuals_{name}.png"), dpi=120)
    plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(7.5, 6), sharex=True)
    lap_nums = [l["lap_number"] for l in laps]
    for ax, key, lab in ((axs[0], "r_mean", "yaw rate on straights [deg/s]"),
                         (axs[1], "ay_mean", "lateral acceleration on straights [m/s^2]")):
        means = []
        for ln in lap_nums:
            rows = [s for s in seg_rows if s["lap"] == ln]
            means.append(np.average([s[key] for s in rows], weights=[s["dur"] for s in rows]) if rows else np.nan)
            ax.scatter([ln] * len(rows), [s[key] for s in rows], s=8, color="#999999")
        ax.plot(lap_nums, means, marker="o", color="black", label="duration-weighted lap mean")
        ax.axhline(0, color="black", lw=0.6)
        ax.set_ylabel(lab, fontsize=8)
        ax.legend(fontsize=8)
    axs[1].set_xlabel("analysis lap")
    axs[0].set_title(f"{title}\nper-lap offsets on steering-defined straights (grey = single straights)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"lap_offsets_{name}.png"), dpi=120)
    plt.close(fig)

    ln = laps[0]["lap_number"]
    m = lap_id == ln
    s_m = st["s_m"]
    fig, axs = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
    axs[0].plot(s_m[m], v[m] * 3.6, color="black", lw=0.8)
    axs[1].plot(s_m[m], sw[m], color="black", lw=0.8)
    axs[1].axhspan(centre - a_thr, centre + a_thr, color="#cccccc", alpha=0.6, label="straight steering window")
    for a, b in segs:
        if lap_id[a] == ln:
            for ax in axs:
                ax.axvspan(s_m[a], s_m[b - 1], color="#4c9be8", alpha=0.25, lw=0)
    axs[0].set_ylabel("speed [km/h]")
    axs[1].set_ylabel("steering-wheel angle [deg]")
    axs[1].set_ylim(-60, 60)
    axs[1].set_xlabel("lap distance [m]")
    axs[1].legend(fontsize=8, loc="lower right")
    axs[0].set_title(f"{title}, lap {ln}\nstraight segments selected by steering (blue)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"straights_lap{ln}_{name}.png"), dpi=120)
    plt.close(fig)
    print(f"\nplots written to {OUT_DIR}")


def filter_check(path, png, summary):
    """Part B (diagnostic 2): what the production washout does to a held
    beta. Synthetic held steps through the production filter function, then
    one real long corner: production betas against a raw integration of the
    offset-corrected inputs (correction inside this diagnostic only)."""
    from modules.stability_analysis import load_parameters, _highpass_filter, estimate_sideslip
    from modules.tyre_fit_auto import resolve_sideslip_beta
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    raw_params = load_parameters()
    ep = apply_resolved_vehicle(raw_params, resolve_accuracy(raw_params, None, None))
    fc = ep["stability_estimation"]["beta_washout_cutoff_hz"]
    print("\nPART B -- production washout on a held beta")
    print(f"  - filter: modules/stability_analysis._highpass_filter, Butterworth high-pass of order "
          f"BUTTERWORTH_ORDER = 4, applied forward and backward (scipy filtfilt), cutoff {fc:g} Hz "
          f"(1 / (2 pi fc) = {1 / (2 * np.pi * fc):.2f} s)")
    syn_sr = 100
    pad = int(20 * syn_sr)
    steps = {}
    for hold in HOLDS_S:
        n_hold = int(hold * syn_sr)
        tr = np.concatenate([np.zeros(pad), np.ones(n_hold), np.zeros(pad)])
        out = _highpass_filter(tr, fc, syn_sr)
        held = out[pad:pad + n_hold]
        steps[hold] = out
        print(f"  - hold {hold:g} s: filtered / true at end of hold {held[-1]:+.3f}, at mid-hold "
              f"{held[len(held) // 2]:+.3f}, mean over hold {held.mean():+.3f}; undershoot before / after the hold "
              f"{out[:pad].min():+.3f} / {out[pad + n_hold:].min():+.3f}")

    parsed, st = load(path)
    t = st["time"]
    sr = st["sample_rate_hz"]
    v = st["v_mps"]
    ay = st["ay_mps2"]
    r = st["yaw_rate_radps"]
    moving = st["moving_mask"]
    valid = {l["lap_number"] for l in parsed["laps"] if l.get("is_valid_for_analysis")}
    best = None
    for c in parsed["corners"]:
        if c["lap_number"] not in valid:
            continue
        spans = [sp for sp in c["segments"].values() if sp[1] >= sp[0]]
        if not spans:
            continue
        i0 = int(np.searchsorted(t, min(sp[0] for sp in spans)))
        i1 = int(np.searchsorted(t, max(sp[1] for sp in spans)))
        for a_, b_ in runs(np.abs(ay[i0:i1]) >= SUSTAINED_AY_MPS2):
            if best is None or (b_ - a_) > best[3] - best[2]:
                best = (c, i0, i0 + a_, i0 + b_)
    c, i_start, a, b = best
    print(f"\n  - real corner: stable corner C{c.get('stable_corner_id')} (corner number {c['corner_number']}), lap "
          f"{c['lap_number']}: longest run of |ay| >= {SUSTAINED_AY_MPS2:g} m/s^2 inside an analysis-lap corner "
          f"= {(b - a) / sr:.2f} s, mean |ay| {np.mean(np.abs(ay[a:b])):.1f} m/s^2, mean vx {np.mean(v[a:b]):.1f} m/s")
    beta_kin = estimate_sideslip(st, ep)
    src = ep["stability_estimation"]["sideslip_source"]
    beta_prod, _m, _g, fb, fb_reason = resolve_sideslip_beta(st, ep, parsed, src, csv_path=path)
    r_b = np.radians(summary["yaw"])
    ay_b = summary["ay"]
    seg = slice(i_start, b)
    vs = np.maximum(v[seg], 1.0)
    dt = 1.0 / sr
    raw = np.cumsum(np.where(moving[seg], ay[seg] / vs - r[seg], 0.0)) * dt
    raw_corr = np.cumsum(np.where(moving[seg], (ay[seg] - ay_b) / vs - (r[seg] - r_b), 0.0)) * dt
    end = b - i_start - 1
    print(f"    raw integration from the corner instance start (t = {t[i_start]:.2f} s, beta = 0 there, "
          f"{(b - i_start) / sr:.2f} s to the end of the sustained run):")
    print(f"      raw, as logged {np.degrees(raw[end]):+.2f} deg; raw, offsets removed (yaw {summary['yaw']:+.3f} "
          f"deg/s, ay {ay_b:+.3f} m/s^2) {np.degrees(raw_corr[end]):+.2f} deg")
    print(f"      production kinematic sideslip (washout) {np.degrees(beta_kin[b - 1]):+.2f} deg -> ratio to the "
          f"offset-removed raw {beta_kin[b - 1] / raw_corr[end]:+.3f}")
    fb_text = f", FALLBACK: {fb_reason}" if fb else ""
    print(f"      production path ({src}{fb_text}) {np.degrees(beta_prod[b - 1]):+.2f} deg -> ratio "
          f"{beta_prod[b - 1] / raw_corr[end]:+.3f}")
    print(f"      mean over the sustained run: raw offsets removed {np.degrees(np.mean(raw_corr[a - i_start:])):+.2f}, "
          f"kinematic {np.degrees(np.mean(beta_kin[a:b])):+.2f}, production {np.degrees(np.mean(beta_prod[a:b])):+.2f} deg")
    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(OUT_DIR, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for hold, out in steps.items():
            ax.plot((np.arange(out.size) - pad) / syn_sr, out, label=f"held {hold:g} s")
        ax.axhline(1.0, color="black", lw=0.6, ls=":")
        ax.set_xlim(-3, 15)
        ax.set_xlabel("time from the start of the hold [s]")
        ax.set_ylabel("filtered / true sideslip")
        ax.set_title(f"Production sideslip washout ({fc:g} Hz high-pass, forward and backward)\n"
                     "response to a sideslip held constant for 3, 6 and 10 s", fontsize=10)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, "washout_step_response.png"), dpi=120)
        plt.close(fig)
        name = os.path.basename(path).replace(".txt", "")
        tt = t[seg] - t[i_start]
        fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        axs[0].plot(tt, np.degrees(raw_corr), color="black", label="raw integration, offsets removed")
        axs[0].plot(tt, np.degrees(raw), color="#999999", label="raw integration, as logged")
        axs[0].plot(tt, np.degrees(beta_kin[seg]), label="production kinematic sideslip (washout)")
        axs[0].plot(tt, np.degrees(beta_prod[seg]), label="production sideslip (estimator the app uses)")
        axs[0].set_ylabel("sideslip [deg]")
        axs[0].legend(fontsize=8)
        axs[1].plot(tt, ay[seg], color="black")
        axs[1].axvspan(t[a] - t[i_start], t[b - 1] - t[i_start], color="#cccccc", alpha=0.6,
                       label="sustained lateral run")
        axs[1].set_ylabel("lateral acceleration [m/s^2]")
        axs[1].set_xlabel("time from the corner start [s]")
        axs[1].legend(fontsize=8)
        axs[0].set_title(f"{track_label(path)}, export {name}, lap {c['lap_number']}, corner {c.get('stable_corner_id')}\n"
                         "sideslip: production paths vs raw integration of offset-corrected inputs", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"long_corner_{name}.png"), dpi=120)
        plt.close(fig)


def session_plot(summaries):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    names = [s_["name"].replace(".txt", "") for s_ in summaries]
    xs = np.arange(len(names))
    fig, axs = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for ax, key, err, lab in ((axs[0], "yaw", "yaw_sd", "yaw-rate offset on straights [deg/s]"),
                              (axs[1], "ay", "ay_sd", "lateral-acc. offset on straights [m/s^2]"),
                              (axs[2], "c1", "c1_se", "cornering gain (ay/vx per yaw rate)"),
                              (axs[3], "c0", "c0_se", "cornering offset [deg/s]")):
        ax.errorbar(xs, [s_[key] for s_ in summaries], yerr=[s_[err] for s_ in summaries], fmt="o", color="black",
                    capsize=3)
        ax.axhline(1.0 if key == "c1" else 0.0, color="#999999", lw=0.8)
        ax.set_ylabel(lab, fontsize=8)
    axs[3].set_xticks(xs)
    axs[3].set_xticklabels(names, rotation=25, ha="right", fontsize=7)
    axs[0].set_title("Input offsets and cornering gain per export\n(straights: spread across segments; "
                     "cornering: segment-bootstrap error)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "offsets_by_session.png"), dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", action="append", help="export path; repeat for several (default: the NBR audit export)")
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--filter-check", action="store_true",
                    help="diagnostic 2 part B on the first --file: washout step response and one long corner")
    args = ap.parse_args()
    files = args.file or [DEFAULT_FILE]
    summaries = []
    for path in files:
        if len(files) > 1:
            print("\n" + "=" * 100)
        summaries.append(audit_file(path, args.png))
    if args.filter_check:
        filter_check(files[0], args.png, summaries[0])
    if len(summaries) > 1:
        print("\nACROSS SESSIONS (straights: duration-weighted mean +/- spread across segments; "
              "M1: segment bootstrap)")
        for s_ in summaries:
            print(f"  - {s_['name']}: steering centre {s_['centre']:+.1f} deg; yaw {s_['yaw']:+.3f} +/- "
                  f"{s_['yaw_sd']:.3f} deg/s; ay {s_['ay']:+.3f} +/- {s_['ay_sd']:.3f} m/s^2; straights "
                  f"{s_['n_straight']} segments, {s_['straight_s']:.1f} s; M1 c1 {s_['c1']:.4f} +/- {s_['c1_se']:.4f}, "
                  f"c0 {s_['c0']:+.3f} +/- {s_['c0_se']:.3f} deg/s; cornering {s_['n_corner_seg']} segments, "
                  f"{s_['corner_s']:.1f} s; left {s_['n_left']} / right {s_['n_right']} samples")
        if args.png:
            session_plot(summaries)


if __name__ == "__main__":
    main()
