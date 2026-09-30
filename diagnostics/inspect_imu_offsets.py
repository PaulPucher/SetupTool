# DIAGNOSTIC, read-only, [keep-reproduces]. Experiment (a) of the PLAN.md
# EXPERIMENT PLAN: characterise constant/slow offsets of the two IMU signals
# the sideslip chain integrates, yaw rate (sclu_yaw_rate) and lateral
# acceleration (log_acc_y). Nothing in modules/ or config/ changes; any
# correction is applied in memory only. Tier B (signal preprocessing).
#
# Reuses: the session mapping of inspect_new_laps_cs_sweep (US-v1 left out,
# it is the v3 outing re-exported), the racing mask / Otsu gate / corner
# slicing / gated integrator of gated_kinematic_common, the gate-run residual
# of inspect_gated_kinematic_drift and the straight-line |ay| fraction of
# inspect_gated_kinematic_scale_sign. Part 4 reads the gated-kinematic caches
# (Dubai, v3: the sessions of the recorded 2 / 14 deg drift finding).
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_imu_offsets census <key>   (raw header scan)
#   python -m diagnostics.inspect_imu_offsets compute <key>  (parse, slim cache)
#   python -m diagnostics.inspect_imu_offsets report         (Parts 1-3)
#   python -m diagnostics.inspect_imu_offsets consequence    (Part 4)
# Outputs in diagnostics/plots/imu_offsets/ (gitignored).
#
# PRE-REGISTERED DEFINITIONS (fixed before the first result):
# Physical references: on a straight yaw rate r ~ 0, ay ~ 0 and beta_dot ~ 0,
# so the kinematic residual c = ay/v - r ~ 0; c stays ~0 on any steady
# segment (beta constant), so it is the least kink-sensitive of the three.
# Over a closed lap the true heading change is +/-360 deg.
# - Straight population S0 (primary): racing mask (moving, off-kerb,
#   analysis-valid laps) outside every corner-bracket slice of an analysis
#   lap. S1 (robustness): S0 and |ay| < STRAIGHT_AY_FRACTION x Otsu, the
#   scale script's straight definition. S1 truncates ay symmetrically about
#   zero, so the ay offset is read on S0 only.
# - Location statistic: median (primary). Both channels are quantised (step
#   from the census), and a median snaps to a quantum level, so the 10 %
#   trimmed mean is reported alongside.
# - Per lap: statistics over that lap's S0 samples. Session spread: sample
#   IQR, lap-median min..max and std; SE = std(lap medians)/sqrt(n_laps).
# - Typical corner duration T: median duration of gated_x1 gate runs
#   (Otsu x1, >= MIN_RUN_S, majority racing) -- the runs over which the
#   recorded 2 / 14 deg drift was measured. Drift equivalent of an offset:
#   yaw only b_r*T; ay only (b_ay/v_gate)*T with v_gate the median speed
#   inside those runs.
# - Stability within session: analysis laps split into early / late halves
#   (odd count: middle lap left out); d = median(late S0) - median(early S0);
#   SE_d = std(lap medians)*sqrt(1/n_e + 1/n_l). Class: ERRATIC if the lap
#   medians change sign and std(lap medians) > |session median|; else
#   CONSTANT if |d| < 2 SE_d; else SLOWLY MOVING. Fewer than 2 laps: not
#   assessable. Across sessions: CONSISTENT if every session median lies
#   within 2 SE_i of the SE-weighted mean.
# - Heading closure (Part 2): H = integral of r over the lap (all samples).
#   E = H - 360 sign(H). Model E = 360 sign(H) e + b_r T_lap. With the Part-1
#   S0 bias b_r: e = (E - b_r T_lap) / (360 sign(H)). Also reproduced with the
#   recorded definition (bias on racing & |ay| < 0.25 Otsu, moving samples)
#   as a parity check against -1.04 % (Dubai) / -0.32 % (v3).
# - Polynomial gate (Part 3): one point per contiguous S0 segment >= 1.0 s
#   (segment median, time = segment mid-time) -- raw 100 Hz samples are too
#   autocorrelated for an information criterion. Least squares, degree 0, 1,
#   2 in time. BIC = n ln(RSS/n) + (deg+1) ln n. Start at degree 0; degree
#   d+1 is adopted only if BIC(d+1) < BIC(d) - 6 (Kass & Raftery "strong").
#   Applied to r (primary), ay and c.
# - Temperature (Part 3): the census found sclu_temp_rate_z (the sensor
#   cluster's z-rate element temperature) in every file. It is read raw
#   (not whitelisted), interpolated to each segment mid-time, and the same
#   degree 0/1/2 BIC rule is run on segment r against temperature. Time and
#   temperature are expected to be collinear within a session; both fits are
#   reported, neither is preferred by assumption.
# - g-sensitivity (Part 3): per corner traversal, the window is widened from
#   the bracket outward until |ay| < STRAIGHT_AY_FRACTION x Otsu on both
#   sides (not past the neighbouring bracket; otherwise skipped). With beta
#   near zero at both ends the window mean of beta_dot is ~0, so the mean of
#   c (after removing the Part-1 constants) is sensor error. OLS per session:
#   c_k = a0 + a1 mean|ay|_k + a2 mean(r)_k. a1 = even-in-g term (the
#   g-sensitivity question), a2 = odd term (gyro scale, ay scale, roll
#   gravity in ay and speed scale all land here and are not separable).
#   G-DEPENDENT for a session if |t(a1)| > 2; CONSISTENT across sessions if
#   every significant a1 shares one sign.
# - Session verdict: the adopted degree for r, plus "g-dependent" if a1 is.
# - Part 4 (Dubai, v3): the adopted per-session model for r and ay applied in
#   memory; gated_x1 re-integrated with the cached Otsu threshold and gate
#   rule; gate-exit residual as recorded. Attribution: offset share =
#   relative drop of the median |residual|; scale share = -e / slope of the
#   residual vs heading change after correction (the recorded method);
#   unexplained = the rest of that slope. Straight-line gap: kinematic
#   (production washout) and causal 0.05 Hz washout vs the cached
#   ekf_auto_pacejka beta on S0, before and after (EKF inputs unchanged).
# POST-HOC (added after the first Part-4 run, labelled as such in the
# output): the pre-registered S0-median correction made the drift larger,
# and Part 2 showed the S0 yaw median disagreeing with the closure excess
# E/T. Four decomposition rows: r only (S0), ay only (S0), r = median
# closure excess E/T only, and that plus ay (S0).

import gzip
import json
import os
import pickle
import re
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import SESSIONS as SWEEP_SESSIONS
from diagnostics.gated_kinematic_common import (
    FIXED_CAP, racing_mask, otsu_threshold, gated_beta, causal_washout_beta,
    valid_corner_instances, load_cache as load_gk_cache,
)
from diagnostics.inspect_gated_kinematic_drift import gate_run_residuals, MIN_RUN_S
from diagnostics.inspect_gated_kinematic_scale_sign import STRAIGHT_AY_FRACTION

KEYS = [k for k in SWEEP_SESSIONS if k != "pr_us_v1"]
TRACK = {"dubai": "Dubai", "v3": "Paul Ricard", "pr_us_v2": "Paul Ricard",
         "nbr_jst_problem": "NBR GP", "nbr_jst_solution": "NBR GP", "lmn_mla": "Le Mans",
         "nbr_qp_cdr": "NBR GP", "nbr_qp_mla": "NBR GP"}
OUT_DIR = "diagnostics/plots/imu_offsets"
IMU_CHANNELS = ("sclu_yaw_rate", "log_acc_y")
MIN_SEGMENT_S = 1.0
BIC_STRONG = 6.0
TRIM = 0.10
RAD2DEG = 180.0 / np.pi
PART4_KEYS = ("dubai", "v3")
# Census patterns: IMU-related names, and temperature names.
IMU_NAME_RE = re.compile(r"(?i)(sclu|imu|gyro|inertial|yaw|acc_[xyz]|_acc)")
TEMP_NAME_RE = re.compile(r"(?i)(temp|_t_|tmp)")


def _strip_unit(raw_name):
    return raw_name[:raw_name.index("[")].strip() if "[" in raw_name else raw_name.strip()


def trimmed_mean(x, frac=TRIM):
    x = np.sort(np.asarray(x, dtype=float))
    x = x[np.isfinite(x)]
    k = int(frac * x.size)
    return float(np.mean(x[k:x.size - k])) if x.size > 2 * k else float("nan")


# ---------------------------------------------------------------- census

def census(key):
    """Every channel name + unit in the raw file, streamed once."""
    names = []
    with open(SWEEP_SESSIONS[key], "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.startswith("{ChannelBlock}"):
                header = f.readline().rstrip("\n").split("\t")
                if len(header) == 2:
                    names.append(header[1].strip())
            line = f.readline()
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"census_{key}.json"), "w") as fh:
        json.dump(names, fh, indent=0)
    imu = [n for n in names if IMU_NAME_RE.search(_strip_unit(n))]
    temp = [n for n in names if TEMP_NAME_RE.search(_strip_unit(n))]
    print(f"[{key}] {len(names)} channels; IMU-like {len(imu)}; temperature-like {len(temp)}")
    print("  IMU-like: " + "; ".join(imu))
    print("  temperature-like: " + "; ".join(temp))


# ---------------------------------------------------------------- compute

TEMP_CHANNEL = "sclu_temp_rate_z"


def read_raw_block(path, name):
    """(time, value) of one channel block streamed from the raw export;
    Dubai/Le Mans exports use a decimal comma."""
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.startswith("{ChannelBlock}"):
                header = f.readline().rstrip("\n").split("\t")
                if len(header) == 2 and _strip_unit(header[1]) == name:
                    ts, vs = [], []
                    line = f.readline()
                    while line and not line.startswith("{"):
                        parts = line.replace(",", ".").split("\t")
                        if len(parts) == 2:
                            try:
                                ts.append(float(parts[0])); vs.append(float(parts[1]))
                            except ValueError:
                                pass
                        line = f.readline()
                    return np.array(ts), np.array(vs), header[1]
            line = f.readline()
    return None, None, None


def compute(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    data = parse_csv(SWEEP_SESSIONS[key])
    state = prepare_vehicle_state(data["channels"], params)
    native = {}
    for name in IMU_CHANNELS:
        ch = data["channels"][name]
        tt = np.asarray(ch["time"], dtype=float)
        vv = np.asarray(ch["data"], dtype=float)
        d = np.abs(np.diff(vv))
        d = d[d > 0]
        native[name] = {"quality": ch.get("quality"), "unit": ch.get("unit_raw"),
                        "rate_hz": float(1.0 / np.median(np.diff(tt))),
                        "quantum": float(d.min()) if d.size else float("nan"),
                        "quantum_p5": float(np.percentile(d, 5)) if d.size else float("nan")}
    laps = [{k: l.get(k) for k in ("lap_number", "start_time", "end_time", "lap_time",
                                   "is_valid_for_analysis", "is_outlap", "is_inlap")}
            for l in data.get("laps", [])]
    corners = [{k: c.get(k) for k in ("lap_number", "stable_corner_id", "bracket_start_m", "bracket_end_m")}
               for c in data.get("corners", [])]
    tt_, tv_, tunit = read_raw_block(SWEEP_SESSIONS[key], TEMP_CHANNEL)
    if tt_ is not None and tt_.size > 2:
        native[TEMP_CHANNEL] = {"quality": "raw", "unit": tunit, "rate_hz": float(1.0 / np.median(np.diff(tt_))),
                                "quantum": float(np.min(np.abs(np.diff(tv_))[np.abs(np.diff(tv_)) > 0]))
                                if np.any(np.diff(tv_) != 0) else float("nan"),
                                "min": float(tv_.min()), "max": float(tv_.max())}
    out = {"key": key, "native": native, "laps": laps, "corners": corners,
           "temp_t": tt_, "temp_v": tv_,
           "sample_rate_hz": state["sample_rate_hz"],
           "time": state["time"], "s_m": state["s_m"], "v_mps": state["v_mps"],
           "ay_mps2": state["ay_mps2"], "yaw_rate_radps": state["yaw_rate_radps"],
           "moving_mask": state["moving_mask"], "kerb_mask": state["kerb_mask"]}
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "wb") as fh:
        pickle.dump(out, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"[{key}] cached n={state['time'].size}, sr={state['sample_rate_hz']:.1f}; native " +
          "; ".join(f"{n}: {v['rate_hz']:.1f} Hz, {v['unit']}, quantum {v['quantum']:.6g} ({v['quality']})"
                    for n, v in native.items()))


def load(key):
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


# ---------------------------------------------------------------- masks

def populations(st, laps, corners):
    """racing mask, Otsu threshold, S0, S1, corner instances."""
    racing = racing_mask(st, laps)
    thr = otsu_threshold(np.abs(st["ay_mps2"][racing]))
    inst = valid_corner_instances({"laps": laps, "corners": corners}, st["time"], st["s_m"])
    in_corner = np.zeros(st["time"].size, dtype=bool)
    for _, _, sl in inst:
        in_corner[sl] = True
    s0 = racing & ~in_corner
    s1 = s0 & (np.abs(st["ay_mps2"]) < STRAIGHT_AY_FRACTION * thr)
    return racing, thr, s0, s1, inst


def analysis_laps(laps):
    return [l for l in laps if l.get("is_valid_for_analysis")]


def lap_mask(t, lap):
    return (t >= lap["start_time"]) & (t < lap["end_time"])


def signals(st):
    v = st["v_mps"]
    moving = st["moving_mask"]
    v_safe = np.where(moving, v, 1.0)
    r = st["yaw_rate_radps"] * RAD2DEG                       # deg/s
    ay = st["ay_mps2"]                                       # m/s^2
    c = np.where(moving, st["ay_mps2"] / v_safe - st["yaw_rate_radps"], np.nan) * RAD2DEG  # deg/s
    return {"r": r, "ay": ay, "c": c}


def location(x, pop, t, laps):
    xs = x[pop]
    xs = xs[np.isfinite(xs)]
    per_lap = []
    for l in laps:
        m = pop & lap_mask(t, l) & np.isfinite(x)
        per_lap.append({"lap": l["lap_number"], "n": int(m.sum()),
                        "median": float(np.median(x[m])) if m.any() else float("nan"),
                        "tmean": trimmed_mean(x[m]) if m.any() else float("nan")})
    lm = np.array([p["median"] for p in per_lap if np.isfinite(p["median"])])
    return {"n": int(xs.size), "median": float(np.median(xs)), "tmean": trimmed_mean(xs),
            "iqr": [float(np.percentile(xs, 25)), float(np.percentile(xs, 75))],
            "per_lap": per_lap,
            "lap_min": float(lm.min()) if lm.size else float("nan"),
            "lap_max": float(lm.max()) if lm.size else float("nan"),
            "lap_std": float(lm.std(ddof=1)) if lm.size > 1 else float("nan"),
            "se": float(lm.std(ddof=1) / np.sqrt(lm.size)) if lm.size > 1 else float("nan")}


def stability(x, pop, t, laps, loc):
    n = len(laps)
    if n < 2:
        return {"class": "not assessable (1 lap)"}
    h = n // 2
    early, late = laps[:h], laps[n - h:]
    me = pop & np.any([lap_mask(t, l) for l in early], axis=0) & np.isfinite(x)
    ml = pop & np.any([lap_mask(t, l) for l in late], axis=0) & np.isfinite(x)
    d = float(np.median(x[ml]) - np.median(x[me]))
    lm = np.array([p["median"] for p in loc["per_lap"]])
    lm = lm[np.isfinite(lm)]
    s = float(lm.std(ddof=1))
    se_d = s * np.sqrt(1.0 / len(early) + 1.0 / len(late))
    sign_change = np.any(lm > 0) and np.any(lm < 0)
    if sign_change and s > abs(loc["median"]):
        cls = "ERRATIC"
    elif abs(d) < 2 * se_d:
        cls = "CONSTANT"
    else:
        cls = "SLOWLY MOVING"
    return {"class": cls, "d": d, "se_d": float(se_d), "n_early": len(early), "n_late": len(late)}


def gate_runs(st, racing, thr, inst, laps, corners):
    """gated_x1 runs through the recorded residual function."""
    beta, gate, rid = gated_beta(st, thr)
    cache = {"state": st, "racing_mask": racing, "gates": {"g": {"gate": gate, "run_id": rid}},
             "betas": {"g": beta}}
    rows, n_runs = gate_run_residuals(cache, "g", inst)
    return rows, gate, rid


def segments(pop, t, min_s, sr):
    """Contiguous runs of pop, >= min_s long: list of index arrays."""
    idx = np.where(pop)[0]
    if idx.size == 0:
        return []
    parts = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
    return [p for p in parts if p.size / sr >= min_s]


def poly_select(tt, yy):
    fits = {}
    n = tt.size
    for deg in (0, 1, 2):
        coef = np.polyfit(tt, yy, deg)
        rss = float(np.sum((yy - np.polyval(coef, tt)) ** 2))
        fits[deg] = {"coef": coef.tolist(), "rss": rss, "bic": n * np.log(rss / n) + (deg + 1) * np.log(n)}
    chosen = 0
    for deg in (1, 2):
        if fits[deg]["bic"] < fits[chosen]["bic"] - BIC_STRONG and deg == chosen + 1:
            chosen = deg
    return chosen, fits


def widened_windows(st, inst, thr_s, racing):
    ay = np.abs(st["ay_mps2"])
    starts = sorted(sl.start for _, _, sl in inst)
    stops = sorted(sl.stop for _, _, sl in inst)
    out = []
    for sid, lap, sl in inst:
        prev_stop = max([s for s in stops if s <= sl.start], default=0)
        next_start = min([s for s in starts if s >= sl.stop], default=ay.size)
        i0 = sl.start
        while i0 > prev_stop and ay[i0] >= thr_s:
            i0 -= 1
        i1 = sl.stop - 1
        while i1 < next_start - 1 and ay[i1] >= thr_s:
            i1 += 1
        if ay[i0] >= thr_s or ay[i1] >= thr_s:
            continue
        w = slice(i0, i1 + 1)
        if racing[w].mean() < 0.9:
            continue
        out.append((sid, lap, w))
    return out


def ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ beta
    dof = max(1, X.shape[0] - X.shape[1])
    s2 = float(res @ res) / dof
    cov = s2 * np.linalg.inv(X.T @ X)
    se = np.sqrt(np.diag(cov))
    return beta, se


# ---------------------------------------------------------------- report

def analyse(key):
    d = load(key)
    st = {k: d[k] for k in ("time", "s_m", "v_mps", "ay_mps2", "yaw_rate_radps", "moving_mask", "kerb_mask")}
    st["sample_rate_hz"] = d["sample_rate_hz"]
    t, sr = st["time"], st["sample_rate_hz"]
    laps = analysis_laps(d["laps"])
    racing, thr, s0, s1, inst = populations(st, d["laps"], d["corners"])
    sig = signals(st)
    res = {"key": key, "track": TRACK[key], "native": d["native"], "n_laps": len(laps),
           "thr_otsu": thr, "n_racing": int(racing.sum()), "n_s0": int(s0.sum()), "n_s1": int(s1.sum())}

    # Part 1
    p1 = {}
    for name in ("r", "ay", "c"):
        p1[name] = {"S0": location(sig[name], s0, t, laps)}
        if name != "ay":
            p1[name]["S1"] = location(sig[name], s1, t, laps)
        p1[name]["stability"] = stability(sig[name], s0, t, laps, p1[name]["S0"])
    rows, gate, rid = gate_runs(st, racing, thr, inst, d["laps"], d["corners"])
    dur = np.array([r_["duration_s"] for r_ in rows])
    T = float(np.median(dur)) if dur.size else float("nan")
    v_gate = float(np.median(st["v_mps"][gate & racing]))
    b_r, b_ay = p1["r"]["S0"]["median"], p1["ay"]["S0"]["median"]
    p1["corner_T_s"] = T
    p1["v_gate_mps"] = v_gate
    p1["drift_yaw_only_deg"] = -b_r * T
    p1["drift_ay_only_deg"] = (b_ay / v_gate) * T * RAD2DEG
    p1["recorded_gate_residual"] = {"median_abs": float(np.median(np.abs([r_["residual_deg"] for r_ in rows]))),
                                    "max_abs": float(np.max(np.abs([r_["residual_deg"] for r_ in rows]))),
                                    "n_runs": len(rows)}
    res["p1"] = p1

    # Part 2
    moving = st["moving_mask"]
    b_rec = float(np.median(sig["r"][racing & (np.abs(st["ay_mps2"]) < STRAIGHT_AY_FRACTION * thr)]))
    p2 = []
    for l in laps:
        m = lap_mask(t, l)
        H = float(np.sum(sig["r"][m]) / sr)
        Tl = float(m.sum() / sr)
        sgn = float(np.sign(H))
        E = H - 360.0 * sgn
        e = (E - b_r * Tl) / (360.0 * sgn)
        Hrec = float(np.sum((sig["r"][m] - b_rec)[moving[m]]) / sr)
        e_rec = Hrec / (360.0 * np.sign(Hrec)) - 1.0
        rot = float(np.sum(np.abs(sig["r"][m])) / sr)
        p2.append({"lap": l["lap_number"], "H_deg": H, "T_s": Tl, "E_deg": E,
                   "bias_part_deg": b_r * Tl, "e": e, "e_recorded_method": e_rec,
                   "bias_if_no_scale_degps": E / Tl, "abs_rotation_deg": rot})
    res["p2"] = {"laps": p2, "b_recorded_def_degps": b_rec,
                 "e_median": float(np.median([x["e"] for x in p2])) if p2 else float("nan"),
                 "e_rec_median": float(np.median([x["e_recorded_method"] for x in p2])) if p2 else float("nan"),
                 "net_heading_sign": [int(np.sign(x["H_deg"])) for x in p2]}

    # Part 3
    segs = segments(s0, t, MIN_SEGMENT_S, sr)
    t0 = laps[0]["start_time"] if laps else t[0]
    tt = np.array([float(np.mean(t[p])) - t0 for p in segs])
    p3 = {"n_segments": len(segs)}
    for name in ("r", "ay", "c"):
        yy = np.array([float(np.nanmedian(sig[name][p])) for p in segs])
        ok = np.isfinite(yy)
        if ok.sum() >= 4:
            chosen, fits = poly_select(tt[ok], yy[ok])
            p3[name] = {"degree": chosen, "fits": fits, "t0": t0, "t_span_s": float(tt.max() - tt.min())}
        else:
            p3[name] = {"degree": None}
    if d.get("temp_t") is not None and len(segs) >= 4:
        mid_abs = np.array([float(np.mean(t[p])) for p in segs])
        temp_seg = np.interp(mid_abs, d["temp_t"], d["temp_v"])
        yy = np.array([float(np.nanmedian(sig["r"][p])) for p in segs])
        span = float(temp_seg.max() - temp_seg.min())
        in_range = bool(d["temp_t"][0] <= mid_abs.min() and d["temp_t"][-1] >= mid_abs.max())
        if span > 0:
            chosen, fits = poly_select(temp_seg, yy)
            p3["temp"] = {"degree": chosen, "fits": fits, "range": [float(temp_seg.min()), float(temp_seg.max())],
                          "time_overlap": in_range,
                          "corr_temp_time": float(np.corrcoef(temp_seg, mid_abs)[0, 1])}
        else:
            p3["temp"] = {"degree": None, "range": [float(temp_seg.min())] * 2, "time_overlap": in_range}
    wins = widened_windows(st, inst, STRAIGHT_AY_FRACTION * thr, racing)
    ck, A, R = [], [], []
    v_safe = np.where(moving, st["v_mps"], 1.0)
    c_corr = (np.where(moving, (st["ay_mps2"] - b_ay) / v_safe, 0.0)
              - (st["yaw_rate_radps"] - b_r / RAD2DEG)) * RAD2DEG
    for _, _, w in wins:
        mm = moving[w]
        ck.append(float(np.mean(c_corr[w][mm])))
        A.append(float(np.mean(np.abs(st["ay_mps2"][w][mm]))))
        R.append(float(np.mean(sig["r"][w][mm])))
    ck, A, R = map(np.array, (ck, A, R))
    if ck.size >= 6:
        X = np.column_stack([np.ones_like(A), A, R])
        coef, se = ols(X, ck)
        rho = float(np.corrcoef(ck, A)[0, 1])
        p3["g"] = {"n": int(ck.size), "a0": float(coef[0]), "a1": float(coef[1]), "a2": float(coef[2]),
                   "se": se.tolist(), "t_a1": float(coef[1] / se[1]), "corr_c_absay": rho,
                   "g_dependent": bool(abs(coef[1] / se[1]) > 2)}
    else:
        p3["g"] = {"n": int(ck.size)}
    res["p3"] = p3
    res["_plot"] = {"tt": tt, "r_seg": np.array([float(np.nanmedian(sig["r"][p])) for p in segs]),
                    "ck": ck, "A": A, "R": R}
    return res


def _f(x, nd=3):
    return "nan" if x is None or not np.isfinite(x) else f"{x:+.{nd}f}"


def report():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    allres = {}
    for key in KEYS:
        if not os.path.exists(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz")):
            print(f"[{key}] no cache")
            continue
        allres[key] = analyse(key)

    print("\n=== PART 1 -- straight-line offsets (S0 primary; units r, c deg/s; ay m/s^2)")
    for key, r in allres.items():
        nat = r["native"]
        print(f"\n[{key}] {r['track']}: analysis laps {r['n_laps']}, racing {r['n_racing']}, S0 {r['n_s0']} "
              f"({100 * r['n_s0'] / r['n_racing']:.0f} %), S1 {r['n_s1']}; Otsu {r['thr_otsu']:.3f} m/s^2")
        print("  native: " + "; ".join(f"{n} {v['rate_hz']:.0f} Hz, unit {v['unit']}, quantum {v['quantum']:.6g}"
                                       for n, v in nat.items()))
        for name in ("r", "ay", "c"):
            for pop in ("S0", "S1"):
                if pop not in r["p1"][name]:
                    continue
                L = r["p1"][name][pop]
                laps_txt = ", ".join(f"{p['lap']}:{_f(p['median'])}" for p in L["per_lap"])
                print(f"  {name:2s} {pop}: median {_f(L['median'], 4)}, tmean {_f(L['tmean'], 4)}, "
                      f"IQR [{_f(L['iqr'][0])}, {_f(L['iqr'][1])}], lap medians {laps_txt}; "
                      f"lap std {_f(L['lap_std'], 4)}, SE {_f(L['se'], 4)}")
            s = r["p1"][name]["stability"]
            print(f"     stability: {s['class']}" + (f" (late-early {_f(s['d'], 4)}, 2SE {2 * s['se_d']:.4f})"
                                                     if "d" in s else ""))
        p1 = r["p1"]
        print(f"  corner T {p1['corner_T_s']:.2f} s, v_gate {p1['v_gate_mps']:.1f} m/s -> drift equivalent: "
              f"yaw only {_f(p1['drift_yaw_only_deg'])} deg, ay only {_f(p1['drift_ay_only_deg'])} deg; "
              f"gated_x1 gate-exit |residual| median {p1['recorded_gate_residual']['median_abs']:.2f}, "
              f"max {p1['recorded_gate_residual']['max_abs']:.2f} deg ({p1['recorded_gate_residual']['n_runs']} runs)")

    print("\n--- cross-session (S0 medians +/- 2 SE)")
    for name in ("r", "ay", "c"):
        vals = [(k, r["p1"][name]["S0"]["median"], r["p1"][name]["S0"]["se"]) for k, r in allres.items()]
        ok = [(k, m, s) for k, m, s in vals if np.isfinite(s) and s > 0]
        w = np.array([1 / s ** 2 for _, _, s in ok])
        mean = float(np.sum(w * np.array([m for _, m, _ in ok])) / w.sum()) if ok else float("nan")
        inside = [abs(m - mean) <= 2 * s for _, m, s in ok]
        print(f"  {name}: " + "; ".join(f"{k} {_f(m, 4)} ({_f(s, 4)})" for k, m, s in vals) +
              f" | weighted mean {_f(mean, 4)}; within 2 SE: {sum(inside)}/{len(ok)} -> "
              f"{'CONSISTENT' if ok and all(inside) else 'NOT CONSISTENT'}")
    print("  ay per track: " + "; ".join(f"{r['track']} ({k}) {_f(r['p1']['ay']['S0']['median'], 4)}"
                                         for k, r in allres.items()))

    print("\n=== PART 2 -- heading closure per analysis lap")
    for key, r in allres.items():
        p2 = r["p2"]
        print(f"\n[{key}] net heading sign per lap {p2['net_heading_sign']}; S0 bias {_f(r['p1']['r']['S0']['median'], 4)} deg/s, "
              f"recorded-definition bias {_f(p2['b_recorded_def_degps'], 4)} deg/s")
        for x in p2["laps"]:
            print(f"  lap {x['lap']}: H {x['H_deg']:+.2f} deg, T {x['T_s']:.1f} s, E {x['E_deg']:+.2f} deg, "
                  f"bias part {x['bias_part_deg']:+.2f} deg, scale e {100 * x['e']:+.3f} % "
                  f"(recorded method {100 * x['e_recorded_method']:+.3f} %), E/T if no scale "
                  f"{x['bias_if_no_scale_degps']:+.4f} deg/s, |rotation| {x['abs_rotation_deg']:.0f} deg")
        print(f"  session e (median of laps) {100 * p2['e_median']:+.3f} %; recorded method {100 * p2['e_rec_median']:+.3f} %")

    print("\n=== PART 3 -- polynomial gate (BIC, +6 to adopt) and g-sensitivity")
    for key, r in allres.items():
        p3 = r["p3"]
        print(f"\n[{key}] segments {p3['n_segments']}")
        for name in ("r", "ay", "c"):
            q = p3[name]
            if q.get("degree") is None:
                print(f"  {name}: too few segments")
                continue
            b = {dd: q["fits"][dd]["bic"] for dd in (0, 1, 2)}
            c1 = q["fits"][1]["coef"][0]
            print(f"  {name}: BIC d0 {b[0]:.1f}, d1 {b[1]:.1f}, d2 {b[2]:.1f} -> degree {q['degree']}; "
                  f"linear slope {c1 * 60:+.5f} /min over {q['t_span_s'] / 60:.1f} min")
        q = p3.get("temp")
        if q is None:
            print("  temperature: channel absent")
        elif q["degree"] is None:
            print(f"  temperature: constant {q['range'][0]} over the analysis laps (overlap {q['time_overlap']})")
        else:
            b = {dd: q["fits"][dd]["bic"] for dd in (0, 1, 2)}
            print(f"  r vs {TEMP_CHANNEL} ({q['range'][0]:.1f}..{q['range'][1]:.1f}, corr with time "
                  f"{q['corr_temp_time']:+.2f}, overlap {q['time_overlap']}): BIC d0 {b[0]:.1f}, d1 {b[1]:.1f}, "
                  f"d2 {b[2]:.1f} -> degree {q['degree']}; linear slope {q['fits'][1]['coef'][0]:+.5f} deg/s per unit")
        g = p3["g"]
        if "a1" in g:
            print(f"  g: n={g['n']} windows; a0 {g['a0']:+.4f} deg/s, a1 {g['a1']:+.5f} (deg/s)/(m/s^2) "
                  f"t={g['t_a1']:+.2f}, a2 {g['a2']:+.5f} (t={g['a2'] / g['se'][2]:+.2f}); corr(c,|ay|) "
                  f"{g['corr_c_absay']:+.2f} -> {'G-DEPENDENT' if g['g_dependent'] else 'not g-dependent'}")
        else:
            print(f"  g: only {g['n']} windows")
    print("\n--- verdicts")
    for key, r in allres.items():
        deg = r["p3"]["r"].get("degree")
        name = {0: "constant", 1: "linear-in-time", 2: "quadratic-in-time", None: "n/a"}[deg]
        gd = r["p3"]["g"].get("g_dependent")
        print(f"  {key}: yaw {name}{' + g-dependent' if gd else ''}; ay degree {r['p3']['ay'].get('degree')}; "
              f"c degree {r['p3']['c'].get('degree')}")
    sig_a1 = [(k, r["p3"]["g"]["a1"]) for k, r in allres.items() if r["p3"]["g"].get("g_dependent")]
    print(f"  significant a1: {sig_a1} -> "
          f"{'consistent sign' if sig_a1 and len({np.sign(a) for _, a in sig_a1}) == 1 else 'no consistent sign / none'}")

    # figures
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.2))
    keys = list(allres)
    x = np.arange(len(keys))
    for ax, name, unit in zip(axs, ("r", "ay", "c"), ("deg/s", "m/s^2", "deg/s")):
        for i, k in enumerate(keys):
            L = allres[k]["p1"][name]["S0"]
            lm = [p["median"] for p in L["per_lap"]]
            ax.plot([i] * len(lm), lm, "o", color="#9ca3af", markersize=4)
            ax.plot(i, L["median"], "s", color="#1f2937", markersize=7)
        ax.axhline(0, color="#6b7280", linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(keys, rotation=45, ha="right", fontsize=7)
        ax.set_title(f"{name} on straights (S0): lap medians (grey), session (black) [{unit}]", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "offsets_by_session.png"), dpi=120)
    plt.close(fig)
    fig, axs = plt.subplots(2, 4, figsize=(15, 6.5))
    for ax, k in zip(axs.flat, keys):
        pl = allres[k]["_plot"]
        ax.plot(pl["tt"] / 60, pl["r_seg"], ".", color="#1f2937", markersize=4)
        q = allres[k]["p3"]["r"]
        if q.get("degree") is not None:
            tg = np.linspace(pl["tt"].min(), pl["tt"].max(), 50)
            for dd, ls in ((0, "-"), (1, "--"), (2, ":")):
                ax.plot(tg / 60, np.polyval(q["fits"][dd]["coef"], tg), ls, color="#6b7280", linewidth=1)
        ax.set_title(f"{k}: straight-segment yaw rate vs time (deg {q.get('degree')})", fontsize=8)
        ax.set_xlabel("min", fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "yaw_segments_vs_time.png"), dpi=120)
    plt.close(fig)

    for r in allres.values():
        r.pop("_plot", None)
    with open(os.path.join(OUT_DIR, "report.json"), "w") as fh:
        json.dump(allres, fh, indent=1, default=float)


# ---------------------------------------------------------------- Part 4

def consequence():
    from modules.stability_analysis import load_parameters, estimate_sideslip
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    with open(os.path.join(OUT_DIR, "report.json")) as fh:
        rep = json.load(fh)
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    out = {}
    for key in PART4_KEYS:
        cache = load_gk_cache(key)
        st = dict(cache["state"])
        t = st["time"]
        laps, corners = cache["laps"], cache["corners"]
        racing = cache["racing_mask"]
        thr = cache["thr_otsu"]
        _r, _thr, s0, _s1, inst = populations(st, laps, corners)
        r_rep = rep[key]

        def model(name):
            q = r_rep["p3"][name]
            coef = q["fits"][str(q["degree"])]["coef"]   # JSON turns the degree keys into strings
            if q["degree"] == 0:
                # degree 0 = the S0 median (the primary location statistic), not the segment mean
                return np.full(t.size, r_rep["p1"][name]["S0"]["median"]), "constant (S0 median)"
            return np.polyval(coef, t - q["t0"]), f"degree {q['degree']} in time"
        b_r, desc_r = model("r")        # deg/s
        b_ay, desc_ay = model("ay")     # m/s^2
        corr = dict(st)
        corr["yaw_rate_radps"] = st["yaw_rate_radps"] - b_r / RAD2DEG
        corr["ay_mps2"] = st["ay_mps2"] - b_ay

        def gated_summary(s):
            beta, gate, rid = gated_beta(s, thr)
            c = {"state": s, "racing_mask": racing, "gates": {"g": {"gate": gate, "run_id": rid}},
                 "betas": {"g": beta}}
            rows, _n = gate_run_residuals(c, "g", inst)
            res = np.array([x["residual_deg"] for x in rows])
            dpsi = np.radians([x["heading_change_deg"] for x in rows])
            k = float(np.dot(dpsi, np.radians(res)) / np.dot(dpsi, dpsi))
            r2 = 1.0 - float(np.sum((np.radians(res) - k * dpsi) ** 2) / np.sum((np.radians(res) - np.radians(res).mean()) ** 2))
            left = res[dpsi > 0]
            right = res[dpsi < 0]
            return {"n": len(rows), "median_abs": float(np.median(np.abs(res))),
                    "p90_abs": float(np.percentile(np.abs(res), 90)), "max_abs": float(np.max(np.abs(res))),
                    "mean": float(res.mean()), "mean_pos_turn": float(left.mean()) if left.size else float("nan"),
                    "mean_neg_turn": float(right.mean()) if right.size else float("nan"),
                    "slope_rad_per_rad": k, "r2": r2, "rows": rows}
        before = gated_summary(st)
        after = gated_summary(corr)
        e = r_rep["p2"]["e_median"]
        # scale share of the heading-proportional slope, recorded method: -e / slope
        attr = {"offset_share_of_median_abs": 1.0 - after["median_abs"] / before["median_abs"],
                "scale_share_of_slope_before": -e / before["slope_rad_per_rad"],
                "scale_share_of_slope_after": -e / after["slope_rad_per_rad"]}
        attr["unexplained_share_of_slope_after"] = 1.0 - attr["scale_share_of_slope_after"]

        ekf = cache["betas"]["ekf_auto_pacejka"]
        kin_b = cache["betas"]["kinematic_prod"]
        kin_a = estimate_sideslip(corr, params)
        cw_b = cache["betas"]["causal_washout_0.05hz"]
        cw_a = causal_washout_beta(corr, 0.05)

        def gap(b):
            dd = np.degrees(b[s0] - ekf[s0])
            return {"median_signed": float(np.median(dd)), "median_abs": float(np.median(np.abs(dd))),
                    "median_abs_beta": float(np.median(np.abs(np.degrees(b[s0]))))}
        straight = {"ekf_median_abs_beta": float(np.median(np.abs(np.degrees(ekf[s0])))),
                    "kinematic_prod": {"before": gap(kin_b), "after": gap(kin_a)},
                    "causal_washout_0.05hz": {"before": gap(cw_b), "after": gap(cw_a)}}
        out[key] = {"model_r": desc_r, "model_ay": desc_ay, "e": e,
                    "before": {k: v for k, v in before.items() if k != "rows"},
                    "after": {k: v for k, v in after.items() if k != "rows"},
                    "attribution": attr, "straight_gap": straight}
        print(f"\n[{key}] correction r: {desc_r} (S0 {r_rep['p1']['r']['S0']['median']:+.4f} deg/s); "
              f"ay: {desc_ay} (S0 {r_rep['p1']['ay']['S0']['median']:+.4f} m/s^2)")
        for tag, s in (("before", before), ("after", after)):
            print(f"  {tag}: {s['n']} runs, |residual| median {s['median_abs']:.2f} / p90 {s['p90_abs']:.2f} / "
                  f"max {s['max_abs']:.2f} deg; mean {s['mean']:+.2f} deg (positive turns {s['mean_pos_turn']:+.2f}, "
                  f"negative turns {s['mean_neg_turn']:+.2f}); slope vs heading {s['slope_rad_per_rad']:+.4f} "
                  f"rad/rad (R^2 {s['r2']:.2f})")
        print(f"  gyro scale e {100 * e:+.3f} % -> predicted slope {-e:+.4f}; scale share of slope before "
              f"{100 * attr['scale_share_of_slope_before']:.0f} %, after {100 * attr['scale_share_of_slope_after']:.0f} %; "
              f"offset share of median |residual| {100 * attr['offset_share_of_median_abs']:.0f} %; unexplained "
              f"share of slope after {100 * attr['unexplained_share_of_slope_after']:.0f} %")
        print(f"  straights (S0): EKF median |beta| {straight['ekf_median_abs_beta']:.3f} deg")
        for src in ("kinematic_prod", "causal_washout_0.05hz"):
            b_, a_ = straight[src]["before"], straight[src]["after"]
            print(f"    {src}: gap to EKF median {b_['median_signed']:+.3f} -> {a_['median_signed']:+.3f} deg, "
                  f"median |gap| {b_['median_abs']:.3f} -> {a_['median_abs']:.3f} deg; median |beta| "
                  f"{b_['median_abs_beta']:.3f} -> {a_['median_abs_beta']:.3f} deg")
        closure_b = float(np.median([x["bias_if_no_scale_degps"] for x in r_rep["p2"]["laps"]]))
        zero = np.zeros(t.size)
        variants = {"r only (S0)": (b_r, zero), "ay only (S0)": (zero, b_ay),
                    f"r closure excess {closure_b:+.4f} only": (np.full(t.size, closure_b), zero),
                    "r closure excess + ay (S0)": (np.full(t.size, closure_b), b_ay)}
        out[key]["posthoc"] = {}
        print("  POST-HOC decomposition:")
        for vname, (br, bay) in variants.items():
            s = dict(st)
            s["yaw_rate_radps"] = st["yaw_rate_radps"] - br / RAD2DEG
            s["ay_mps2"] = st["ay_mps2"] - bay
            v = gated_summary(s)
            v.pop("rows")
            out[key]["posthoc"][vname] = v
            print(f"    {vname}: |residual| median {v['median_abs']:.2f} / max {v['max_abs']:.2f} deg; mean "
                  f"{v['mean']:+.2f} (pos {v['mean_pos_turn']:+.2f}, neg {v['mean_neg_turn']:+.2f}); slope "
                  f"{v['slope_rad_per_rad']:+.4f} (R^2 {v['r2']:.2f})")
        _plot_consequence(key, before["rows"], after["rows"])
    with open(os.path.join(OUT_DIR, "consequence.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)


def _plot_consequence(key, rb, ra):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    ax.plot([x["heading_change_deg"] for x in rb], [x["residual_deg"] for x in rb], "o", color="#9ca3af",
            markersize=4, label="as logged")
    ax.plot([x["heading_change_deg"] for x in ra], [x["residual_deg"] for x in ra], "o", color="#1f2937",
            markersize=4, label="offset-corrected (in memory)")
    ax.axhline(0, color="#6b7280", linewidth=0.8)
    ax.set_xlabel("heading change of gate run (deg)", fontsize=8)
    ax.set_ylabel("gated_x1 residual at gate exit (deg)", fontsize=8)
    ax.set_title(f"{key}: in-corner drift before / after the offset correction", fontsize=9)
    ax.legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"consequence_{key}.png"), dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------- (a2) pre-check
# Liveness of the two unused second sources found by the census, before an
# (a2) package is designed on them. Pre-registered: over the analysis laps a
# channel is DEAD if it has <= 2 distinct values or zero std, ABSENT if not
# in the file, LIVE if non-constant, inside a plausible range (|yaw| < 300
# deg/s after its unit, |ay| < 5 g) and with non-zero std inside corner
# brackets. Correlation against the production channel over the first
# analysis lap is reported (lag 0 and the best lag within +/-1 s, plus the
# regression slope, which exposes a unit or sign difference); it does not
# decide LIVE/DEAD. Unmapped files: header presence and logger serial only.

# ---------------------------------------------------------------- (a2) turn-proportional error
# Experiment (a2): separate the candidates of the 2-7 % turn-proportional
# error between ay/v and r (gyro scale, ay scale, roll-gravity leakage into
# ay, speed scale). Read-only; corrections in memory only. Modes:
#   a2compute <key>  cache: production state + raw sclu_acc_y, sclu_roll_rate,
#                    a_roll, GPS speed channels, corner speed classes
#   referee          Parts 1-2 (all eight sessions)
#   collapse         Part 3 (Dubai, v3, JST 29)
# PRE-REGISTERED (fixed before the first a2 run):
# PHYSICS. Planar kinematics: ay_CG = v (r + beta_dot) + v_dot beta; in
# quasi-steady cornering ay_CG ~ r v. A body-fixed accelerometer reads
# ay_CG plus: road bank g sin(theta) (1 deg ~ 0.17 m/s^2, track- and
# corner-specific); roll-gravity leakage g sin(phi) for body roll phi
# (outward roll lifts the lateral axis, adding to |ay|; Part 2.3);
# lever arm psi_dd x_s (bounded by the psi_dd ceiling below: 0.15 rad/s^2 x
# ~0.5 m ~ 0.08 m/s^2, under 1 % at the gate) and -r^2 y_s (<= ~0.1 m/s^2,
# one sign in both turn directions, so it lands in the direction-split
# intercepts, not the slope); v_dot beta (bounded by the |ax| ceiling:
# 2 m/s^2 x ~0.05 rad ~ 0.1 m/s^2, about 1 %).
# PART 1 (referee). r, v, ax, log_acc_y and sclu_acc_y low-passed with the
# chain's own zero-phase Butterworth (BUTTERWORTH_ORDER, cs_filter_cutoff_hz
# from config); psi_dd = gradient of filtered r. Steady population: racing
# mask, inside a corner bracket of an analysis lap, |r v| > session Otsu
# gate (selection on the REGRESSOR, which leaves OLS unbiased; selecting on
# ay would truncate the dependent variable), |psi_dd| < 0.15 rad/s^2,
# |ax| < 2.0 m/s^2, raw |log_acc_y| < 5.0 g (the pre-check plausibility
# bound; the excluded share is reported). Per accelerometer: OLS a = s (r v)
# + i with intercept, overall, per turn direction (sign of r) and per corner
# speed class. 95 % CI: bootstrap over corner traversals (1000, seed 0).
# Attenuation check: 1 / slope of the reverse regression reported beside s.
# Referee reading: the accelerometer with the smaller |s - 1| in a stratum
# is "closer"; it is "consistently closer" if so in every experiment session
# and in every direction and speed-class stratum with n >= 500. If both
# slopes exceed 1 (or both fall below) with CIs excluding 1, the shared part
# points at r v (gyro or speed scale).
# PART 2.1. ln s = ln(1+e_a) - ln(1+e_r) - ln(1+e_v). e_v from GPS (Part
# 2.2; ecu speed reading low by k gives e_v = 1/k - 1). e_r range = the
# closure scale of experiment (a) under its two bias definitions (S0,
# recorded). Implied e_a range per accelerometer; the split is DEGENERATE
# where the e_r range is wider than the product error it has to split.
# PART 2.2. GPS speed channels (log_gps_speed, gpsa_speed, IMSA_GPS_vSpeed):
# liveness as in the pre-check; lag = peak of the cross-correlation with
# ecu_speed within +/-1 s; k = origin regression v_gps(t + lag) = k v_ecu
# over racing samples (the recorded 2026-07-26 method, whose script no
# longer exists in the repo).
# PART 2.3. Roll angle: a_roll (suspension route, deg) where live; else, and
# as a cross-check everywhere, sclu_roll_rate integrated from 0 over each
# corner window widened to straight on both sides (IMU route: tilt relative
# to the straight-line attitude, bank change included). Roll gradient
# k_phi = OLS slope of phi (rad) on ay (m/s^2) over the steady population.
# Predicted leakage scale on ay = g |k_phi| (outward roll assumed, as a
# car rolls to the outside; the sign of k_phi only states the channel's
# convention). Against the production product error d = s_log - 1:
# EXCLUDED if opposite sign or |pred| < 0.2 |d|; EXPLAINS if |pred| >= 0.8
# |d|; CONTRIBUTES otherwise. Roll leakage acts on both accelerometers
# alike, so it cannot explain their mutual difference.
# PART 3 (drift collapse). Each candidate's constant comes from its OWN
# independent measurement, never from the referee fit or the drift itself
# (all four act on the same product, so a referee-fitted constant would
# collapse the drift by construction for any of them):
#  C1 ay scale: log_acc_y x s_ls, s_ls = lag-aligned slope of sclu_acc_y on
#     log_acc_y over racing samples (clip rule applied) -- "trust sclu".
#  C2 gyro scale: r / (1 + e_r), two rows: e_r with the recorded bias
#     definition, and with the S0 bias.
#  C3 speed scale: v x k_gps (Part 2.2; the recorded Dubai k 1.01211 where
#     no live GPS, labelled).
#  C4 roll leakage: log_acc_y / (1 + g |k_phi|) (Part 2.3 gradient).
# Drift metric exactly as recorded: gated_x1 (session Otsu threshold held
# fixed), gate-exit residual, slope through the origin vs heading change.
# WINNER: |slope_after| <= 0.5 |slope_before| AND median |residual|_after
# <= 1.02 x before, on all three test sessions. If no single candidate
# wins, pairwise combinations only, labelled.

# POST-HOC (after the first referee run, labelled in the output):
# (1) Direction and speed-class strata of the OLS referee are dominated by
#     errors-in-variables attenuation (the stratum cuts the regressor range;
#     s and 1/s_rev part by up to 0.55), so only the pooled slope reads as a
#     scale. Mode "ratio": median of a / (r v) per stratum with a traversal
#     bootstrap -- a range restriction does not attenuate a ratio.
# (2) a_roll is excluded as a roll source: corr 0.99-1.00 with ay and
#     14.5-15.8 deg/g, an order above the IMU route (0.87-1.49 deg/g) --
#     consistent with a channel computed from ay or an unverified unit, not
#     an independent roll measurement. C4 uses the roll-rate route.
A2_KEYS = KEYS
A2_ROLL_ROUTE = "roll_rate_integral"
A2_COLLAPSE_KEYS = ("dubai", "v3", "nbr_jst_solution")
A2_RAW = ("sclu_acc_y", "sclu_roll_rate", "a_roll", "log_gps_speed", "gpsa_speed", "IMSA_GPS_vSpeed")
PSIDD_MAX = 0.15       # rad/s^2
AX_MAX = 2.0           # m/s^2
CLIP_G = 5.0
N_BOOT = 1000
MIN_STRATUM = 500
K_RECORDED = 1.01211
G = 9.81


def a2compute(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    raw_p = load_parameters()
    params = apply_resolved_vehicle(raw_p, resolve_accuracy(raw_p, setup_data=None, cap=FIXED_CAP))
    data = parse_csv(SWEEP_SESSIONS[key])
    st = prepare_vehicle_state(data["channels"], params)
    raw = read_raw_blocks(SWEEP_SESSIONS[key], A2_RAW)
    out = {"key": key, "sample_rate_hz": st["sample_rate_hz"],
           "cs_filter_cutoff_hz": params["stability_estimation"]["cs_filter_cutoff_hz"],
           "laps": [{k: l.get(k) for k in ("lap_number", "start_time", "end_time", "is_valid_for_analysis")}
                    for l in data.get("laps", [])],
           "corners": [{k: c.get(k) for k in ("lap_number", "stable_corner_id", "bracket_start_m",
                                              "bracket_end_m", "speed_class")} for c in data.get("corners", [])],
           "raw": {n: raw[n] for n in raw}}
    for k in ("time", "s_m", "v_mps", "ay_mps2", "ax_mps2", "yaw_rate_radps", "moving_mask", "kerb_mask"):
        out[k] = st[k]
    with gzip.open(os.path.join(OUT_DIR, "cache", f"a2_{key}.pkl.gz"), "wb") as fh:
        pickle.dump(out, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"[{key}] a2 cache: raw " + "; ".join(
        f"{n} {raw[n][2]} n={raw[n][1].size} distinct={np.unique(raw[n][1]).size}" for n in raw) +
        "; absent: " + ", ".join(n for n in A2_RAW if n not in raw))


def load_a2(key):
    with gzip.open(os.path.join(OUT_DIR, "cache", f"a2_{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def _lp(x, fc, sr):
    from scipy.signal import butter, filtfilt
    from modules.stability_analysis import BUTTERWORTH_ORDER
    b, a = butter(BUTTERWORTH_ORDER, fc / (0.5 * sr), btype="low")
    return filtfilt(b, a, x)


def _raw_on_grid(d, name, conv=1.0):
    if name not in d["raw"]:
        return None
    t, v, _u = d["raw"][name]
    if t.size < 3 or np.unique(v).size <= 2 or np.std(v) == 0:
        return None
    return np.interp(d["time"], t, v) * conv


def _a2_state(d):
    st = {k: d[k] for k in ("time", "s_m", "v_mps", "ay_mps2", "ax_mps2", "yaw_rate_radps",
                            "moving_mask", "kerb_mask")}
    st["sample_rate_hz"] = d["sample_rate_hz"]
    return st


def _steady(d, st, racing, thr, inst):
    sr, fc = d["sample_rate_hz"], d["cs_filter_cutoff_hz"]
    rf = _lp(st["yaw_rate_radps"], fc, sr)
    vf = _lp(st["v_mps"], fc, sr)
    axf = _lp(st["ax_mps2"], fc, sr)
    alog = _lp(st["ay_mps2"], fc, sr)
    sc = _raw_on_grid(d, "sclu_acc_y", G)
    asc = _lp(sc, fc, sr) if sc is not None else None
    psidd = np.gradient(rf, st["time"])
    rv = rf * vf
    clip = np.abs(st["ay_mps2"]) >= CLIP_G * G
    trav = np.full(st["time"].size, -1)
    spd = np.full(st["time"].size, "", dtype=object)
    sc_by = {(c["lap_number"], c["stable_corner_id"]): c.get("speed_class") for c in d["corners"]}
    for j, (sid, lap, sl) in enumerate(inst):
        trav[sl] = j
        spd[sl] = sc_by.get((lap, sid)) or ""
    base = racing & (trav >= 0) & (np.abs(rv) > thr)
    pop = base & (np.abs(psidd) < PSIDD_MAX) & (np.abs(axf) < AX_MAX)
    return {"rv": rv, "rf": rf, "alog": alog, "asc": asc, "pop": pop & ~clip, "trav": trav, "spd": spd,
            "clip_share_racing": float(np.mean(clip[racing])), "clip_share_steady": float(np.mean(clip[pop])),
            "n_base": int(base.sum()), "n_pop_preclip": int(pop.sum())}


def _fit(x, y, clusters, seed=0):
    X = np.column_stack([x, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    rev = np.polyfit(y, x, 1)[0]
    rng = np.random.default_rng(seed)
    uc = np.unique(clusters)
    idx_by = {c: np.where(clusters == c)[0] for c in uc}
    boots = []
    for _ in range(N_BOOT):
        pick = rng.choice(uc, size=uc.size, replace=True)
        ii = np.concatenate([idx_by[c] for c in pick])
        boots.append(np.polyfit(x[ii], y[ii], 1)[0])
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return {"s": float(coef[0]), "i": float(coef[1]), "ci": [float(lo), float(hi)],
            "s_rev": float(1.0 / rev) if rev != 0 else float("nan"), "n": int(x.size), "n_trav": int(uc.size)}


def _gps_k(d, st, racing):
    out = {}
    sr = d["sample_rate_hz"]
    for n in ("log_gps_speed", "gpsa_speed", "IMSA_GPS_vSpeed"):
        if n not in d["raw"]:
            out[n] = {"status": "ABSENT"}
            continue
        t, v, u = d["raw"][n]
        in_r = np.interp(t, st["time"], racing.astype(float)) > 0.5
        if t.size < 3 or np.unique(v[in_r]).size <= 2 or np.std(v[in_r]) == 0:
            out[n] = {"status": "DEAD", "unit": u, "zero_share": float(np.mean(v[in_r] == 0)) if in_r.any() else None}
            continue
        g = np.interp(st["time"], t, v) / 3.6
        e = st["v_mps"]
        kmax = int(1.0 * sr)
        best = (0, -1.0)
        m = racing
        for k in range(-kmax, kmax + 1):
            if k >= 0:
                a, b = g[k:][m[:m.size - k]], e[:e.size - k][m[:m.size - k]]
            else:
                a, b = g[:k][m[-k:]], e[-k:][m[-k:]]
            rr = float(np.corrcoef(a, b)[0, 1])
            if rr > best[1]:
                best = (k, rr)
        k = best[0]
        a, b = (g[k:][m[:m.size - k]], e[:e.size - k][m[:m.size - k]]) if k >= 0 else (g[:k][m[-k:]], e[-k:][m[-k:]])
        kk = float(np.dot(a, b) / np.dot(b, b))
        rate = float(1.0 / np.median(np.diff(t)))
        out[n] = {"status": "LIVE", "unit": u, "rate_hz": rate, "lag_s": k / sr, "r": best[1], "k": kk,
                  "median_diff_mps": float(np.median(a - b))}
    return out


def _roll(d, st, racing, thr, inst, stdy):
    """Roll gradients (rad per m/s^2) on the steady population, per route."""
    res = {}
    ay_f = stdy["alog"]
    pop = stdy["pop"]
    a_roll = _raw_on_grid(d, "a_roll", np.pi / 180.0)
    if a_roll is not None:
        phi = _lp(a_roll, d["cs_filter_cutoff_hz"], d["sample_rate_hz"])
        k = np.polyfit(ay_f[pop], phi[pop], 1)
        res["a_roll"] = {"k_phi_rad_per_mps2": float(k[0]), "intercept_rad": float(k[1]),
                         "deg_per_g": float(np.degrees(k[0]) * G), "leak_scale": float(G * abs(k[0])),
                         "corr": float(np.corrcoef(ay_f[pop], phi[pop])[0, 1])}
    else:
        res["a_roll"] = {"status": "absent or dead"}
    rr = _raw_on_grid(d, "sclu_roll_rate", 6.0 * np.pi / 180.0)   # rpm -> rad/s
    if rr is not None:
        sr = d["sample_rate_hz"]
        wins = widened_windows(st, inst, STRAIGHT_AY_FRACTION * thr, racing)
        phi = np.full(st["time"].size, np.nan)
        for _, _, w in wins:
            phi[w] = np.cumsum(rr[w]) / sr
        phi_f = phi
        ok = pop & np.isfinite(phi_f)
        k = np.polyfit(ay_f[ok], phi_f[ok], 1)
        # closure of the roll integral over each window: how far it drifts back
        ends = [float(np.degrees(phi[w][-1])) for _, _, w in wins]
        res["roll_rate_integral"] = {"k_phi_rad_per_mps2": float(k[0]), "intercept_rad": float(k[1]),
                                     "deg_per_g": float(np.degrees(k[0]) * G), "leak_scale": float(G * abs(k[0])),
                                     "corr": float(np.corrcoef(ay_f[ok], phi_f[ok])[0, 1]), "n": int(ok.sum()),
                                     "window_end_deg_median_abs": float(np.median(np.abs(ends))) if ends else None}
    else:
        res["roll_rate_integral"] = {"status": "absent or dead"}
    return res


def _classify_roll(pred, d):
    if not np.isfinite(pred) or d == 0:
        return "n/a"
    if np.sign(pred) != np.sign(d) or abs(pred) < 0.2 * abs(d):
        return "EXCLUDED"
    if abs(pred) >= 0.8 * abs(d):
        return "EXPLAINS"
    return "CONTRIBUTES"


def _s_ls(d, st, racing):
    """Lag-aligned slope of sclu_acc_y on log_acc_y over racing samples."""
    sc = _raw_on_grid(d, "sclu_acc_y", G)
    if sc is None:
        return None
    ok = racing & (np.abs(st["ay_mps2"]) < CLIP_G * G)
    lg = st["ay_mps2"]
    best = (0, -1.0)
    for k in range(-10, 11):
        if k >= 0:
            a, b, m = sc[k:], lg[:lg.size - k], ok[:ok.size - k]
        else:
            a, b, m = sc[:k], lg[-k:], ok[-k:]
        rr = float(np.corrcoef(a[m], b[m])[0, 1])
        if rr > best[1]:
            best = (k, rr)
    k = best[0]
    a, b, m = (sc[k:], lg[:lg.size - k], ok[:ok.size - k]) if k >= 0 else (sc[:k], lg[-k:], ok[-k:])
    return {"slope": float(np.polyfit(b[m], a[m], 1)[0]), "lag_samples": k, "r": best[1]}


def referee():
    with open(os.path.join(OUT_DIR, "report.json")) as fh:
        rep = json.load(fh)
    allres = {}
    for key in A2_KEYS:
        d = load_a2(key)
        st = _a2_state(d)
        racing, thr, s0, s1, inst = populations(st, d["laps"], d["corners"])
        stdy = _steady(d, st, racing, thr, inst)
        pop = stdy["pop"]
        res = {"serial": "147" if key in ("dubai", "lmn_mla") else "221", "n_pop": int(pop.sum()),
               "n_base": stdy["n_base"], "clip_share_racing": stdy["clip_share_racing"],
               "clip_share_steady": stdy["clip_share_steady"], "fits": {}}
        strata = {"all": pop, "r>0": pop & (stdy["rf"] > 0), "r<0": pop & (stdy["rf"] < 0)}
        for cls in ("low", "medium", "high"):
            strata[cls] = pop & (stdy["spd"] == cls)
        for acc, arr in (("log_acc_y", stdy["alog"]), ("sclu_acc_y", stdy["asc"])):
            if arr is None:
                continue
            res["fits"][acc] = {}
            for sname, m in strata.items():
                if m.sum() < 50:
                    res["fits"][acc][sname] = {"n": int(m.sum())}
                    continue
                res["fits"][acc][sname] = _fit(stdy["rv"][m], arr[m], stdy["trav"][m])
        res["gps"] = _gps_k(d, st, racing)
        res["roll"] = _roll(d, st, racing, thr, inst, stdy)
        res["s_ls"] = _s_ls(d, st, racing)
        # Part 2.1
        p2 = rep[key]["p2"]
        e_r = sorted([p2["e_median"], p2["e_rec_median"]])
        live_k = [g["k"] for g in res["gps"].values() if g.get("status") == "LIVE"]
        k_gps = live_k[0] if live_k else None
        e_v = (1.0 / k_gps - 1.0) if k_gps else None
        split = {}
        for acc in res["fits"]:
            s = res["fits"][acc]["all"]["s"]
            if e_v is None:
                split[acc] = {"s": s, "e_a_range": None}
                continue
            ea = [s * (1 + er) * (1 + e_v) - 1 for er in e_r]
            split[acc] = {"s": s, "e_a_range": sorted(ea), "e_r_range": e_r, "e_v": e_v,
                          "degenerate": bool((e_r[1] - e_r[0]) > abs(s - 1))}
        res["split"] = split
        allres[key] = res

    print("=== PART 1 -- referee: accelerometer vs r*v on quasi-steady cornering (filtered, "
          f"|psi_dd|<{PSIDD_MAX}, |ax|<{AX_MAX}, clip |log_acc_y|>={CLIP_G} g excluded)")
    for key, r in allres.items():
        print(f"\n[{key}] serial {r['serial']}: steady n {r['n_pop']} (of {r['n_base']} above gate in brackets); "
              f"clipped share: racing {100 * r['clip_share_racing']:.3f} %, steady {100 * r['clip_share_steady']:.3f} %")
        for acc, fits in r["fits"].items():
            for sname, f in fits.items():
                if "s" not in f:
                    print(f"  {acc:10s} {sname:6s} n {f['n']} (too few)")
                    continue
                print(f"  {acc:10s} {sname:6s} n {f['n']:6d} ({f['n_trav']:3d} trav): s {f['s']:.4f} "
                      f"[{f['ci'][0]:.4f}, {f['ci'][1]:.4f}], 1/s_rev {f['s_rev']:.4f}, intercept {f['i']:+.3f} m/s^2")
    print("\n--- referee reading (|s-1|, strata n >= %d)" % MIN_STRATUM)
    for key, r in allres.items():
        if "sclu_acc_y" not in r["fits"]:
            continue
        rows = []
        for sname in r["fits"]["log_acc_y"]:
            fl, fs = r["fits"]["log_acc_y"][sname], r["fits"]["sclu_acc_y"][sname]
            if "s" not in fl or fl["n"] < MIN_STRATUM:
                continue
            closer = "sclu" if abs(fs["s"] - 1) < abs(fl["s"] - 1) else "log"
            both = ("both >1" if fl["ci"][0] > 1 and fs["ci"][0] > 1 else
                    "both <1" if fl["ci"][1] < 1 and fs["ci"][1] < 1 else "straddle")
            rows.append(f"{sname}: {closer} ({both})")
        print(f"  {key}: " + "; ".join(rows))

    print("\n=== PART 2.2 -- GPS speed")
    for key, r in allres.items():
        print(f"  [{key}] " + "; ".join(
            f"{n} {g['status']}" + (f" {g['rate_hz']:.0f} Hz, lag {g['lag_s']:+.2f} s, r {g['r']:.4f}, k {g['k']:.5f}"
                                    if g.get("status") == "LIVE" else "") for n, g in r["gps"].items()))
    print("\n=== PART 2.1 -- product split (e_r from closure range, e_v from GPS k)")
    for key, r in allres.items():
        for acc, sp in r["split"].items():
            if sp.get("e_a_range") is None:
                print(f"  [{key}] {acc}: s {sp['s']:.4f}; no live GPS -> not split")
                continue
            print(f"  [{key}] {acc}: s {sp['s']:.4f}, e_r {100 * sp['e_r_range'][0]:+.2f}..{100 * sp['e_r_range'][1]:+.2f} %, "
                  f"e_v {100 * sp['e_v']:+.2f} % -> implied e_a {100 * sp['e_a_range'][0]:+.2f}..{100 * sp['e_a_range'][1]:+.2f} %"
                  f"{' DEGENERATE' if sp['degenerate'] else ''}")
    print("\n=== PART 2.3 -- roll-gravity leakage")
    for key, r in allres.items():
        d_log = r["fits"]["log_acc_y"]["all"]["s"] - 1
        d_sc = r["fits"].get("sclu_acc_y", {}).get("all", {}).get("s", np.nan) - 1
        parts = []
        for route, v in r["roll"].items():
            if "leak_scale" not in v:
                parts.append(f"{route}: {v.get('status')}")
                continue
            parts.append(f"{route}: {v['deg_per_g']:+.3f} deg/g (corr {v['corr']:+.2f}) -> leakage "
                         f"+{100 * v['leak_scale']:.2f} %; vs d_log {100 * d_log:+.2f} % {_classify_roll(v['leak_scale'], d_log)}, "
                         f"vs d_sclu {100 * d_sc:+.2f} % {_classify_roll(v['leak_scale'], d_sc)}"
                         + (f"; window-end |phi| median {v['window_end_deg_median_abs']:.2f} deg"
                            if v.get("window_end_deg_median_abs") is not None else ""))
        print(f"  [{key}] " + " | ".join(parts))
    print("\n--- s_ls (sclu on log, lag-aligned, racing): " + "; ".join(
        f"{k} {r['s_ls']['slope']:.4f}" for k, r in allres.items() if r.get("s_ls")))
    with open(os.path.join(OUT_DIR, "a2_referee.json"), "w") as fh:
        json.dump(allres, fh, indent=1, default=float)


def collapse():
    with open(os.path.join(OUT_DIR, "report.json")) as fh:
        rep = json.load(fh)
    with open(os.path.join(OUT_DIR, "a2_referee.json")) as fh:
        ref = json.load(fh)
    out = {}

    def drift(s, racing, thr, inst):
        beta, gate, rid = gated_beta(s, thr)
        c = {"state": s, "racing_mask": racing, "gates": {"g": {"gate": gate, "run_id": rid}}, "betas": {"g": beta}}
        rows, _n = gate_run_residuals(c, "g", inst)
        res = np.array([x["residual_deg"] for x in rows])
        dpsi = np.radians([x["heading_change_deg"] for x in rows])
        k = float(np.dot(dpsi, np.radians(res)) / np.dot(dpsi, dpsi))
        return {"n": len(rows), "median_abs": float(np.median(np.abs(res))), "max_abs": float(np.max(np.abs(res))),
                "slope": k, "mean_pos": float(res[dpsi > 0].mean()), "mean_neg": float(res[dpsi < 0].mean())}

    for key in A2_COLLAPSE_KEYS:
        d = load_a2(key)
        st = _a2_state(d)
        racing, thr, _s0, _s1, inst = populations(st, d["laps"], d["corners"])
        r = ref[key]
        p2 = rep[key]["p2"]
        live_k = [g["k"] for g in r["gps"].values() if g.get("status") == "LIVE"]
        k_gps, k_lab = (live_k[0], "GPS") if live_k else (K_RECORDED, "recorded Dubai k")
        roll_route = A2_ROLL_ROUTE   # post-hoc (2): a_roll excluded, see header
        leak = r["roll"][roll_route].get("leak_scale", 0.0)
        s_ls = r["s_ls"]["slope"]
        cands = {
            f"C1 ay scale x{s_ls:.4f} (trust sclu)": {"ay": s_ls},
            f"C2a gyro scale e {100 * p2['e_rec_median']:+.2f} % (recorded bias)": {"r": 1 / (1 + p2["e_rec_median"])},
            f"C2b gyro scale e {100 * p2['e_median']:+.2f} % (S0 bias)": {"r": 1 / (1 + p2["e_median"])},
            f"C3 speed scale x{k_gps:.5f} ({k_lab})": {"v": k_gps},
            f"C4 roll leakage /{1 + leak:.4f} ({roll_route})": {"ay": 1 / (1 + leak)},
        }
        base = drift(st, racing, thr, inst)
        rows = {"before": base}
        print(f"\n[{key}] before: {base['n']} runs, median |res| {base['median_abs']:.2f}, max {base['max_abs']:.2f} deg, "
              f"slope {base['slope']:+.4f} rad/rad (pos {base['mean_pos']:+.2f}, neg {base['mean_neg']:+.2f})")
        for name, sc in cands.items():
            s = dict(st)
            s["ay_mps2"] = st["ay_mps2"] * sc.get("ay", 1.0)
            s["yaw_rate_radps"] = st["yaw_rate_radps"] * sc.get("r", 1.0)
            s["v_mps"] = st["v_mps"] * sc.get("v", 1.0)
            dd = drift(s, racing, thr, inst)
            dd["collapse"] = abs(dd["slope"]) <= 0.5 * abs(base["slope"])
            dd["median_ok"] = dd["median_abs"] <= 1.02 * base["median_abs"]
            rows[name] = dd
            print(f"  {name}: median {dd['median_abs']:.2f}, max {dd['max_abs']:.2f}, slope {dd['slope']:+.4f} "
                  f"(pos {dd['mean_pos']:+.2f}, neg {dd['mean_neg']:+.2f}) -> collapse {dd['collapse']}, "
                  f"median ok {dd['median_ok']}")
        out[key] = rows
        # PAIRWISE (pre-registered fallback when no single candidate wins;
        # evaluated for every session, read only if no single winner)
        import itertools
        keys_c = list(cands)
        for a, b in itertools.combinations(keys_c, 2):
            if a.startswith("C2") and b.startswith("C2"):
                continue
            fa, fb = cands[a], cands[b]
            s = dict(st)
            s["ay_mps2"] = st["ay_mps2"] * fa.get("ay", 1.0) * fb.get("ay", 1.0)
            s["yaw_rate_radps"] = st["yaw_rate_radps"] * fa.get("r", 1.0) * fb.get("r", 1.0)
            s["v_mps"] = st["v_mps"] * fa.get("v", 1.0) * fb.get("v", 1.0)
            dd = drift(s, racing, thr, inst)
            dd["collapse"] = abs(dd["slope"]) <= 0.5 * abs(base["slope"])
            dd["median_ok"] = dd["median_abs"] <= 1.02 * base["median_abs"]
            rows["PAIR " + a.split(" ")[0] + "+" + b.split(" ")[0]] = dd
    names = [n for n in out[A2_COLLAPSE_KEYS[0]] if n != "before"]
    print("\n--- winner criterion (all three sessions; PAIR rows = pre-registered pairwise fallback)")
    for i, _ in enumerate(names):
        per = [list(out[k].items())[i + 1][1] for k in A2_COLLAPSE_KEYS]
        win = all(p["collapse"] and p["median_ok"] for p in per)
        print(f"  {names[i].split(' ')[0]}: " + ", ".join(
            f"{k} slope {p['slope']:+.4f} median {p['median_abs']:.2f}" for k, p in zip(A2_COLLAPSE_KEYS, per)) +
            f" -> {'WINS' if win else 'no'}")
    with open(os.path.join(OUT_DIR, "a2_collapse.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)


def ratio():
    """POST-HOC (1): median a / (r v) per stratum, traversal bootstrap."""
    out = {}
    for key in A2_KEYS:
        d = load_a2(key)
        st = _a2_state(d)
        racing, thr, _s0, _s1, inst = populations(st, d["laps"], d["corners"])
        sd = _steady(d, st, racing, thr, inst)
        pop = sd["pop"]
        strata = {"all": pop, "r>0": pop & (sd["rf"] > 0), "r<0": pop & (sd["rf"] < 0)}
        for cls in ("low", "medium", "high"):
            strata[cls] = pop & (sd["spd"] == cls)
        rng = np.random.default_rng(0)
        res = {}
        line = []
        for sname, m in strata.items():
            if m.sum() < MIN_STRATUM:
                continue
            row = {}
            for acc, arr in (("log", sd["alog"]), ("sclu", sd["asc"])):
                q = arr[m] / sd["rv"][m]
                cl = sd["trav"][m]
                uc = np.unique(cl)
                by = {c: q[cl == c] for c in uc}
                boots = [np.median(np.concatenate([by[c] for c in rng.choice(uc, uc.size)])) for _ in range(500)]
                row[acc] = {"median": float(np.median(q)), "ci": [float(x) for x in np.percentile(boots, [2.5, 97.5])]}
            row["n"] = int(m.sum())
            res[sname] = row
            closer = "sclu" if abs(row["sclu"]["median"] - 1) < abs(row["log"]["median"] - 1) else "log"
            line.append(f"{sname} (n {row['n']}): log {row['log']['median']:.4f} [{row['log']['ci'][0]:.4f}, "
                        f"{row['log']['ci'][1]:.4f}], sclu {row['sclu']['median']:.4f} [{row['sclu']['ci'][0]:.4f}, "
                        f"{row['sclu']['ci'][1]:.4f}] -> {closer}")
        out[key] = res
        print(f"[{key}] POST-HOC median ratio a/(r v):\n  " + "\n  ".join(line))
    with open(os.path.join(OUT_DIR, "a2_ratio.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)


PRECHECK_PAIRS = (("MRR_EgoYawRate", "sclu_yaw_rate"), ("sclu_acc_y", "log_acc_y"))
UNMAPPED_GLOB = "D:/Data_Bachelorthesis/**/NBR-*.txt"
LAG_MAX_S = 1.0


def read_raw_blocks(path, names):
    """{name: (time, value, header)} for several blocks in one pass."""
    wanted, out = set(names), {}
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.startswith("{ChannelBlock}"):
                header = f.readline().rstrip("\n").split("\t")
                if len(header) == 2 and _strip_unit(header[1]) in wanted:
                    ts, vs = [], []
                    line = f.readline()
                    while line and not line.startswith("{"):
                        parts = line.replace(",", ".").split("\t")
                        if len(parts) == 2:
                            try:
                                ts.append(float(parts[0])); vs.append(float(parts[1]))
                            except ValueError:
                                pass
                        line = f.readline()
                    out[_strip_unit(header[1])] = (np.array(ts), np.array(vs), header[1].strip())
                    continue
            line = f.readline()
    return out


def header_info(path, names):
    serial, present = None, {}
    with open(path, "r", encoding="latin-1") as f:
        for i, line in enumerate(f):
            if serial is None and line.startswith("SystemDetails"):
                m = re.search(r"Serial Number: (\d+)", line)
                serial = m.group(1) if m else "?"
            if line.startswith("{ChannelBlock}"):
                h = next(f).rstrip("\n").split("\t")
                if len(h) == 2 and _strip_unit(h[1]) in names:
                    present[_strip_unit(h[1])] = h[1].strip()
    return serial, present


def _to_common(name, v, unit):
    """deg/s for yaw sources, g for accelerations; the factor is reported."""
    u = unit.lower()
    if "yaw" in name.lower():
        if "rpm" in u:
            return v * 6.0, "rpm x6"
        if "rad" in u:
            return v * RAD2DEG, "rad/s x57.3"
        return v, "deg/s as logged"
    return v, "as logged"


def _liveness(name, t, v, unit, laps, corner_windows):
    in_laps = np.any([(t >= l["start_time"]) & (t < l["end_time"]) for l in laps], axis=0)
    x = v[in_laps]
    xc, conv = _to_common(name, x, unit)
    dt = np.diff(t)
    rate = float(1.0 / np.median(dt[dt > 0])) if dt.size else float("nan")
    in_c = np.zeros(t.size, dtype=bool)
    for a, b in corner_windows:
        in_c |= (t >= a) & (t < b)
    cstd = float(np.std(v[in_c])) if in_c.any() else float("nan")
    lim = 300.0 if "yaw" in name.lower() else 5.0
    nd = int(np.unique(x).size)
    if x.size == 0:
        status = "ABSENT (no samples in analysis laps)"
    elif nd <= 2 or np.std(x) == 0:
        status = "DEAD"
    elif np.nanmax(np.abs(xc)) < lim and cstd > 0:
        status = "LIVE"
    else:
        status = "IMPLAUSIBLE"
    return {"status": status, "unit": unit, "conv": conv, "rate_hz": rate, "n": int(x.size), "n_distinct": nd,
            "min": float(np.min(xc)) if x.size else None, "max": float(np.max(xc)) if x.size else None,
            "std": float(np.std(xc)) if x.size else None, "corner_std_raw": cstd,
            "zero_share": float(np.mean(x == 0)) if x.size else None}


def _compare(a, b, lap):
    """a: second source, b: production; both (t, v_common). One lap, a
    resampled onto b's time base."""
    tb, vb = b
    ta, va = a
    m = (tb >= lap["start_time"]) & (tb < lap["end_time"])
    tb, vb = tb[m], vb[m]
    ok = (tb >= ta[0]) & (tb <= ta[-1])
    tb, vb = tb[ok], vb[ok]
    va_i = np.interp(tb, ta, va)
    r0 = float(np.corrcoef(va_i, vb)[0, 1])
    dt = float(np.median(np.diff(tb)))
    kmax = int(LAG_MAX_S / dt)
    best = (0, r0)
    for k in range(-kmax, kmax + 1):
        if k >= 0:
            x, y = va_i[k:], vb[:vb.size - k]
        else:
            x, y = va_i[:k], vb[-k:]
        rr = float(np.corrcoef(x, y)[0, 1])
        if abs(rr) > abs(best[1]):
            best = (k, rr)
    slope, icpt = np.polyfit(vb, va_i, 1)
    k = best[0]
    xa, yb = (va_i[k:], vb[:vb.size - k]) if k >= 0 else (va_i[:k], vb[-k:])
    slope_l, icpt_l = np.polyfit(yb, xa, 1)
    return {"r0": r0, "best_lag_s": best[0] * dt, "r_best": best[1], "slope": float(slope),
            "intercept": float(icpt), "slope_at_lag": float(slope_l), "intercept_at_lag": float(icpt_l),
            "std_ratio": float(np.std(xa) / np.std(yb)),
            "n": int(tb.size), "t": tb, "a": va_i, "b": vb}


def precheck():
    import glob
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [n for pair in PRECHECK_PAIRS for n in pair]
    results = {}
    for key in KEYS:
        path = SWEEP_SESSIONS[key]
        d = load(key)
        laps = analysis_laps(d["laps"])
        st = {k: d[k] for k in ("time", "s_m")}
        inst = valid_corner_instances({"laps": d["laps"], "corners": d["corners"]}, st["time"], st["s_m"])
        cw = [(st["time"][sl.start], st["time"][sl.stop - 1]) for _, _, sl in inst]
        serial, _ = header_info_serial_only(path)
        blocks = read_raw_blocks(path, names)
        res = {"serial": serial, "channels": {}, "pairs": {}}
        common = {}
        for n in names:
            if n not in blocks:
                res["channels"][n] = {"status": "ABSENT"}
                continue
            t, v, unit = blocks[n]
            res["channels"][n] = _liveness(n, t, v, unit, laps, cw)
            common[n] = (t, _to_common(n, v, unit)[0])
        print(f"\n[{key}] {TRACK[key]}, logger serial {serial}, analysis laps {len(laps)}")
        for n in names:
            c = res["channels"][n]
            if c["status"] == "ABSENT":
                print(f"  {n:15s} ABSENT")
                continue
            print(f"  {n:15s} {c['status']:11s} {c['rate_hz']:6.1f} Hz, unit {c['unit']} ({c['conv']}), "
                  f"n {c['n']}, distinct {c['n_distinct']}, range [{c['min']:+.3f}, {c['max']:+.3f}], "
                  f"std {c['std']:.3f}, corner std (raw) {c['corner_std_raw']:.3f}, zeros {100 * c['zero_share']:.1f} %")
        for a, b in PRECHECK_PAIRS:
            if a in common and b in common and laps:
                cmp_ = _compare(common[a], common[b], laps[0])
                res["pairs"][a] = {k: v for k, v in cmp_.items() if k not in ("t", "a", "b")}
                print(f"  {a} vs {b}, lap {laps[0]['lap_number']} (n {cmp_['n']}): r(lag 0) {cmp_['r0']:+.3f}, "
                      f"best lag {cmp_['best_lag_s']:+.2f} s (r {cmp_['r_best']:+.3f}), slope {cmp_['slope']:+.3f}, "
                      f"intercept {cmp_['intercept']:+.3f}; at best lag slope {cmp_['slope_at_lag']:+.4f}, "
                      f"intercept {cmp_['intercept_at_lag']:+.4f}, std ratio {cmp_['std_ratio']:.4f}")
                if a == "MRR_EgoYawRate" and res["channels"][a]["status"] == "LIVE":
                    fig, ax = plt.subplots(figsize=(10, 3.8))
                    ax.plot(cmp_["t"] - cmp_["t"][0], cmp_["b"], color="#1f2937", linewidth=0.9,
                            label="sclu_yaw_rate (x6, deg/s)")
                    ax.plot(cmp_["t"] - cmp_["t"][0], cmp_["a"], color="#9ca3af", linewidth=0.9,
                            label=f"MRR_EgoYawRate ({res['channels'][a]['conv']})")
                    ax.set_xlabel("time in lap (s)", fontsize=8)
                    ax.set_ylabel("deg/s", fontsize=8)
                    ax.set_title(f"{key}: lap {laps[0]['lap_number']}, both yaw-rate sources", fontsize=9)
                    ax.legend(fontsize=7, frameon=False)
                    fig.tight_layout()
                    fig.savefig(os.path.join(OUT_DIR, f"precheck_yaw_{key}.png"), dpi=120)
                    plt.close(fig)
        results[key] = res
    mapped = {os.path.normcase(os.path.abspath(p)) for p in SWEEP_SESSIONS.values()}
    print("\n--- unmapped NBR files (presence + serial only)")
    unmapped = {}
    for p in sorted(glob.glob(UNMAPPED_GLOB, recursive=True)):
        if os.path.normcase(os.path.abspath(p)) in mapped:
            continue
        serial, present = header_info(p, set(names))
        unmapped[p] = {"serial": serial, "present": present}
        print(f"  {os.path.relpath(p, 'D:/Data_Bachelorthesis')}: serial {serial}; " +
              ", ".join(f"{n} {'present ' + present[n] if n in present else 'ABSENT'}" for n in names))
    with open(os.path.join(OUT_DIR, "precheck.json"), "w") as fh:
        json.dump({"mapped": results, "unmapped": unmapped}, fh, indent=1, default=float)


def header_info_serial_only(path):
    with open(path, "r", encoding="latin-1") as f:
        for i, line in enumerate(f):
            if line.startswith("SystemDetails"):
                m = re.search(r"Serial Number: (\d+)", line)
                return (m.group(1) if m else "?"), None
            if i > 200:
                break
    return "?", None


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode in ("census", "compute", "a2compute"):
        {"census": census, "compute": compute, "a2compute": a2compute}[mode](sys.argv[2])
    else:
        {"report": report, "consequence": consequence, "precheck": precheck,
         "referee": referee, "collapse": collapse, "ratio": ratio}[mode]()
