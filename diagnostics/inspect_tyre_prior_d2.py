# DIAGNOSTIC, read-only. PLAN.md EXPERIMENT PLAN (d2): (d) reopened by the
# author 2026-10-01 -- a tyre-curve prior per axle fitted once on every
# usable serial-221 session, per-session fits as penalised deviations from
# it, and the fit -> EKF -> refit loop retried with the prior frozen.
# Records: thesis_notes.md "(d2) ..." entries. Nothing in modules/ or
# config/ changes: the anchored fitter enters production's
# fit_session_pacejka through an in-memory replacement of
# modules.tyre_fit_auto._fit_axle_pacejka (the (d) step-0 mechanism), so R
# derivation, R sweep, final EKF and gate run exactly as production runs
# them.
#
# Tier A (prior-plus-deviation tyre fit): anchor PENDING, as in (d) --
# nothing ships from this script. Tier B: pooling, penalised least squares
# (Tikhonov regularisation towards a reference curve), unit conversion,
# census checks.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_tyre_prior_d2 census          header census of every file
#   python -m diagnostics.inspect_tyre_prior_d2 intake <key>    parse once, standing checks, cache
#   python -m diagnostics.inspect_tyre_prior_d2 intake_report   Part 0 inventory
#   python -m diagnostics.inspect_tyre_prior_d2 fit <key>       production fit + downstream chain
#   python -m diagnostics.inspect_tyre_prior_d2 prior           Part 1 pooled prior
#   python -m diagnostics.inspect_tyre_prior_d2 achain <key> <w>  Part 2 anchored fit through production
#   python -m diagnostics.inspect_tyre_prior_d2 loop <key> <w>  Part 3 convergence retry
#   python -m diagnostics.inspect_tyre_prior_d2 report          Parts 1-4 read-out
#   python -m diagnostics.inspect_tyre_prior_d2 render          fit plots + trace set
#   python -m diagnostics.inspect_tyre_prior_d2 drift           addendum: where the loop drift lives
#   python -m diagnostics.inspect_tyre_prior_d2 tloop <key> <w> Door 2: loop with transient-weighted fits
#   python -m diagnostics.inspect_tyre_prior_d2 door2           Door 2 read-out
#   python -m diagnostics.inspect_tyre_prior_d2 d1loop <key>    Door 1: w 0 loop replay, beta kept per pass
#   python -m diagnostics.inspect_tyre_prior_d2 door1           Door 1 Stage 0 read-out
#
# DOOR 1 STAGE 0 (fixed before its runs; author rulings 2026-10-01, thesis_
#   notes.md "Door 1 proposal: longitudinal information ..."). Question: does
#   the tangential-acceleration identity tell the (d2) loop's beta candidates
#   apart on this logging? Identity: d|v|/dt = a_x cos beta + a_y sin beta
#   (Kiencke & Nielsen 9.1.1 Eq. 9.2); d|v|/dt from the NON-DRIVEN (front)
#   wheels, as in their 9.7.1 gradient residual. Solving it for beta is this
#   project's adaptation of an anchored identity (author ruling).
#   Signals: chain grid, all low-passed with the chain's zero-phase
#   cs_filter_cutoff_hz filter. v_f = mean of production's guarded
#   log_speed_fl/fr (km/h -> m/s); vdot = gradient of v_f. a_x at the CG =
#   log_acc_x + X_IMU * r^2 (rigid body: a point X_IMU ahead of the CG reads
#   -r^2 X_IMU less). X_IMU = 0.20 m forward, author estimate 2026-10-01 (SCLU
#   on the steering-wheel line), not config. The a_y lever term psi_dd X_IMU
#   is bounded by the psi_dd gate (<= 0.03 m/s^2) and left out. Assumption
#   stated: the front-wheel-plane / |v| ratio changes slowly in the
#   population, so its derivative is neglected (psi_dd gate).
#   Candidates: kinematic seed (estimate_sideslip), loop passes 1-4 at w 0
#   (d1loop: the PART 3 loop, unchanged, beta stored per pass), beta = 0 as
#   context. Per candidate y = a_x cos b + a_y sin b - vdot, OLS on the
#   nuisance set [1, a_y, v^2] (bias, IMU yaw-mount angle, pitch with aero
#   load) over Q; score = residual RMS. Sensitivity (not scored): r^2 fitted
#   instead of fixed.
#   Q (quasi-steady cornering, (a2) Part 1 population plus): racing, inside a
#   corner instance of an analysis lap, |r v| > session Otsu threshold,
#   |psi_dd| < 0.15 rad/s^2, |a_x| < 2 m/s^2, raw |a_y| < 5 g, not kerb
#   mask, log_pbrake_f < 5 bar (front wheels free-rolling, WP-S1), both
#   front speeds from log_speed (no guard fallback).
#   S (straight floor): racing, outside corner instances, |a_y| < 0.25 x
#   Otsu, same psi_dd / a_x / kerb / brake / source conditions. Floor F =
#   residual RMS of (a_x - vdot) on [1, v^2]; beta-equivalent F / median
#   |a_y| over Q.
#   Entry: E1 flat track -- lap-averaged log_gps_alt profile (5 m bins, each
#   lap's mean removed, 100 m moving average), |gradient| <= 1 % on >= 90 %
#   of Q; E2 -- Q >= 2000 samples and >= 20 traversals; E3 (report only) --
#   log_acc_x - sclu_acc_x regressed on [1, a_y] over Q and S: mutual yaw
#   angle. Sessions: v3, pr_us_v2, dubai (flat-track candidates).
#   Statistic: dRMS = RMS(pass 1) - RMS(pass 4), 95 % CI from 1000 traversal-
#   bootstrap draws (nuisance refitted per draw). D_s = sign(dRMS) where the
#   CI excludes 0, else 0. Fewer than 2 sessions passing E1 and E2 -> N4,
#   CLOSED on inventory. Fewer than 2 sessions with D_s != 0 -> N2, CLOSED
#   (no discrimination). Nonzero D_s not sharing one sign on >= 2 sessions,
#   or the kinematic seed lowest-RMS of {kin, 1..4} on >= 2 sessions -> N3,
#   CLOSED (model error, not information). Otherwise Stage 0 PASS (Stage 1
#   needs the author's look). Reach stated before the run: even full success
#   covers 2 of the 18 pool sessions (v3, US-v2) unless gradient is solved.
#   Figure door1_<key>.png (one analysis lap) inspected before the read-out.
#
# DOOR 2 (fixed before its runs; author work order 2026-10-01). Hypothesis:
#   the curve/beta degeneracy is exact only in quasi-steady cornering;
#   transient samples carry partial separating information, so a fit
#   weighted towards them should drift less under the loop.
#   Weight: alpha_dot = np.gradient(alpha, 1/fs) of the alpha being fitted
#   in that pass (kinematic in pass 1, the filter's own after), per axle;
#   w_i = max(1, |alpha_dot_i| / median |alpha_dot| over that axle's fit
#   population); non-finite alpha_dot -> 1. Data term = sum w r^2 / sum w,
#   so the prior penalty keeps its scale. Yaw acceleration not used
#   (simplest first). Residuals handed to the R derivation unweighted.
#   Loop otherwise identical to PART 3, retry sessions as there, w 0 and 1.
#   Read: inside-p95 curve change, max over both axles per pass (the drift
#   addendum's re-reading metric, same pass-1 p95 split). Scored: the walk
#   after the alpha-source switch, m'(2->3) + m'(3->4) at w = 0. Uniform
#   reference (addendum): JDR 10 3.20 %, v3 5.53 %, JST 19 5.70 %. SIGNAL
#   iff transient-weighted <= half the uniform value on 2 of 3 sessions;
#   else no signal, door closed. Pass 1->2 and total 1->4 context only;
#   w = 1 reported, not scored.
# Outputs in diagnostics/plots/tyre_prior_d2/ (gitignored).
#
# PART 1 (fixed before the prior run): prior = production _fit_axle_pacejka,
#   unchanged (Powell from the chair start guess), on the concatenated pool
#   population (kinematic alpha_f/r_filt, Fy_f/r_filt, _base_mask). An
#   average across sessions; no stability criterion (author).
#
# PART 2 (fixed before the anchored runs):
# - Anchored fit, per axle: minimise
#     J(p) = mean_i (F(alpha_i; p) - Fy_i)^2 + w * mean_g (F(g; p) - F_prior(g))^2
#   over all four (B, C, D, E), Powell started from the prior's parameters.
#   g = 200 points on [0, p99 |alpha|] of the population being fitted (the
#   visited range: where the data can argue against the prior).
# - Why the penalty sits on the curve, not on the parameters: B, C and E
#   trade off along a ridge on which the curve barely moves ((d) step 0:
#   LOSO_cdr front C = 1.26 vs 1.88 at almost the same curve). A parameter
#   penalty would pull along that ridge without constraining the force
#   characteristic; the curve penalty is invariant to it and needs no
#   per-parameter scaling.
# - Reading of w: for a curve linear in its parameters and data spread
#   evenly over the grid, the session's own deviation from the prior shrinks
#   by about 1 / (1 + w). Grid: w = 0 (prior as start only, the unanchored
#   control), 0.1 weak (~9 % shrink), 1 medium (~50 %), 10 strong (~91 %).
# - CHOICE RULE: selected w = the smallest of {0.1, 1, 10} meeting both
#   (G) gate health -- on every usable session no fallback (degenerate fit
#   or gate fail) where production has none, and the count of gate "pass"
#   verdicts not below production's; and (C) convergence -- Part 3 reads
#   CONVERGES at that w on every retry session. Smallest, because the prior
#   should constrain no more than needed. None meets both -> no weight
#   selected; Part 4 then shows w = 1 as reference only, labelled so.
#
# PART 3 (fixed before the loop runs):
# - Retry sessions: the pool session with the most analysis laps (tie: more
#   base_mask samples), v3 (the recorded unanchored divergence, in the pool)
#   and the pool session with the fewest analysis laps (tie: fewer samples).
#   Each at every w of the Part 2 grid; w = 0 replays the unanchored loop
#   under the same harness.
# - Pass 1 = the Part 2 anchored fit (kinematic alpha). Pass p >= 2: the
#   anchored fit on alpha from pass p-1's resolved beta (what production
#   would consume: beta_ekf_with_fallback, kinematic after a fallback), Fy
#   and base_mask unchanged. Prior FROZEN; penalty anchored at the prior and
#   Powell started from the prior on every pass, so only beta carries memory
#   between passes. R derived afresh each pass (production steps (c)-(e)).
#   Maximum 4 passes.
# - Per pass p >= 2: m_p = max over axles of max(|D_p - D_p-1| / D_p-1,
#   max_g |F_p(g) - F_p-1(g)| / D_prior), g = 200 points on [0, pass-1
#   p99 |alpha|] of that axle (fixed across passes).
#   CONVERGES iff m_3 < m_2, m_4 < m_3 and m_4 <= 0.01.
#   DIVERGES iff m_4 > m_3 (growing), or on an axle the sign of D_p - D_p-1
#   alternates over passes 2-4 while |dD| does not shrink (oscillating).
#   Otherwise SHRINKING, NOT CONVERGED within 4 passes.
#   [op] tolerance 1 %: below the (d) K1 band (3 %) and the ~5 % input-scale
#   error budget.
# - Watch items (thesis_notes.md "Refit-loop conclusion: structural
#   non-convergence confirmed on two sessions, two failure directions"):
#   D inflation = D_rear pass 4 / pass 1 - 1 (unanchored record: Dubai
#   9213.6 -> 12402.8 N, +34.6 %; v3 9559.9 -> 14461.1 N, +51.3 %); linear
#   collapse = rear peak running out beyond the visited p99 or vanishing
#   (no stiffness sign change). Gate verdict and health per pass.
#
# DRIFT ADDENDUM (fixed before it ran; curves rebuilt from the cached
#   per-pass parameters, no new EKF run): split points = pass-1 visited
#   p95 / p99 of |alpha| (kinematic, base_mask), fixed across passes;
#   "beyond p99" ends at the largest per-pass visited p99 of that session
#   and axle over all weights (how far the filter's own alpha reaches).
#   Per region RMS and max of the curve change, in N and / D_prior.
#   Verdict on the total drift (pass 4 vs 1): visited-dominant iff RMS
#   inside p95 >= RMS beyond p99, else extrapolated-dominant. Re-reading:
#   the PART 3 rule with m_p = curve change on [0, p95] only (no D term).
#
# PART 4: production vs prior-anchored (selected w) on every usable session:
#   fit plots, gate health, status, worst-lap verdict distribution, the
#   standard CS trace set. Facts only, no scoring (author).
#
# PART 0 DEFINITIONS (fixed before the intake runs):
# - Identity: OutingInformation car, driver, session, outing, first lap,
#   track, logger serial (SystemDetails "Serial Number"). Same car +
#   session + outing + driver + serial as an already mapped file = the same
#   outing; it is a duplicate and enters no pool twice.
# - Rate guard: production _resolve_grid_rate; a ValueError rejects.
# - Liveness: production quality flag (range gate) per chain channel, plus
#   a stuck-value check on the five required channels while moving (share
#   of zero first differences on the native samples); > 0.95 = dead (flat-lined).
# - Units: unit_raw of every chain channel. Channels whose consumer reads
#   unit_raw itself (lap_distance, susp travel, damper force, abs_speed)
#   are converted by production. The others are taken in the config unit by
#   prepare_vehicle_state; a different header unit there is converted here
#   by a known factor (UNIT_FACTORS) or the file is rejected.
# - Clustering sanity: stable corner ids, ids present on every analysis
#   lap, analysis-lap length.
# - Usable = state builds, rate guard passes, >= 1 analysis lap, base_mask
#   non-empty, every chain unit known. Pool = usable, serial 221, not a
#   duplicate, not Dubai / Le Mans (author ruling, different tyres + logger).

import gzip
import json
import os
import pickle
import re
import sys
import time

import numpy as np

DATA_ROOT = "D:/Data_Bachelorthesis"
OUT_DIR = "diagnostics/plots/tyre_prior_d2"
CACHE = os.path.join(OUT_DIR, "cache")

# already mapped (inspect_new_laps_cs_sweep.SESSIONS); keys kept
MAPPED = {
    "dubai": "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt",
    "v3": "GT3_PRC_MLA-v3.txt",
    "pr_us_v1": "D:/Data_Bachelorthesis/Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt",
    "pr_us_v2": "D:/Data_Bachelorthesis/paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt",
    "nbr_jst_problem": "D:/Data_Bachelorthesis/NBR_Sturm_problem/NBR-73-021-071-JST-26.07.01.1509.txt",
    "nbr_jst_solution": "D:/Data_Bachelorthesis/NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt",
    "lmn_mla": "D:/Data_Bachelorthesis/Random Lemans/LMN-#73-P9541-005-240613-10.30-MLA.txt",
    "nbr_qp_cdr": "D:/Data_Bachelorthesis/NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt",
    "nbr_qp_mla": "D:/Data_Bachelorthesis/NBR_Qualy_MLA/MLA-QP01-NBR-v2.txt",
}
EXCLUDED_BY_RULING = ("dubai", "lmn_mla")  # different tyres + logger 147

REQUIRED = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_x"]
# chain channels prepare_vehicle_state takes in the config unit without
# reading unit_raw
UNIT_TRUSTED = REQUIRED + ["log_acc_z", "log_pbrake_f", "log_speed_fl", "log_speed_fr",
                           "log_speed_rl", "log_speed_rr", "log_gps_lat", "log_gps_lon"]
# (channel, header unit) -> factor to the unit the chain expects
UNIT_FACTORS = {
    ("ecu_speed", "mph"): 1.609344,
    ("ecu_speed", "m/s"): 3.6,
    ("log_acc_x", "m/s/s"): 1 / 9.81, ("log_acc_y", "m/s/s"): 1 / 9.81, ("log_acc_z", "m/s/s"): 1 / 9.81,
    ("log_asteer", "rad"): 180 / np.pi,
}
OTHER_CHAIN = ["lap_distance", "log_dms_dam_fl", "log_dms_dam_fr", "log_dms_dam_rl", "log_dms_dam_rr",
               "log_susp_travel_fl", "log_susp_travel_fr", "log_susp_travel_rl", "log_susp_travel_rr",
               "abs_speed_fl", "abs_speed_fr", "abs_speed_rl", "abs_speed_rr", "log_rideheight_fl",
               "log_rideheight_fr", "log_rideheight_rl", "log_rideheight_rr", "log_gps_speed"]
STUCK_DEAD = 0.95
# header spellings of the config unit itself (factor 1)
UNIT_ALIASES = {"km/h": ("kph",), "deg": ("\u00b0",), "g": ("G",)}


def _save(path, obj, level=1):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with gzip.open(path, "wb", compresslevel=level) as fh:
        pickle.dump(obj, fh, protocol=pickle.HIGHEST_PROTOCOL)


def _load(path):
    with gzip.open(path, "rb") as fh:
        return pickle.load(fh)


def _params():
    # Best available, the app's default path: with setup_data=None only the
    # steering conversion differs from cap 1 (Level-4 manufacturer map,
    # corrected 2026-10-04). Fit caches built at cap 1 are archived in
    # cache_cap1/ as the constant-15.7 reference.
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    return apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))


# ---------------------------------------------------------------- census

def _header(path):
    meta = {}
    with open(path, "r", encoding="latin-1") as f:
        in_block = False
        for line in f:
            s = line.strip()
            if s == "{OutingInformation}":
                in_block = True
                continue
            if in_block:
                if s.startswith("{"):
                    break
                parts = s.split("\t")
                if len(parts) == 2:
                    meta[parts[0].strip()] = parts[1].strip()
    m = re.search(r"Serial Number:\s*(\d+)", meta.get("SystemDetails", ""))
    return {"car": meta.get("CarName"), "driver": meta.get("DriverName"), "session": meta.get("SessionNumber"),
            "outing": meta.get("OutingNumber"), "first_lap": meta.get("FirstLapNumber"),
            "track": meta.get("TrackName"), "serial": m.group(1) if m else None,
            "comment": meta.get("ShortComment")}


def _identity(h):
    return (h["car"], h["session"], h["outing"], h["driver"], h["serial"])


def census():
    files = []
    for root, _dirs, names in os.walk(DATA_ROOT):
        for n in names:
            files.append(os.path.join(root, n).replace("\\", "/"))
    non_txt = {}
    for f in files:
        if not f.lower().endswith(".txt"):
            ext = f.rsplit(".", 1)[-1].lower()
            non_txt.setdefault((os.path.dirname(f), ext), 0)
            non_txt[(os.path.dirname(f), ext)] += 1
    txt = sorted(f for f in files if f.lower().endswith(".txt"))
    mapped_by_path = {os.path.normcase(os.path.abspath(p)): k for k, p in MAPPED.items()}
    rows = {}
    for k, p in MAPPED.items():
        if not p.startswith(DATA_ROOT):
            rows[k] = {"path": p, "mapped": True, **_header(p), "size": os.path.getsize(p)}
    for p in txt:
        k = mapped_by_path.get(os.path.normcase(os.path.abspath(p)))
        h = _header(p)
        if k is None:
            k = f"{(h['track'] or 'x').lower()}_o{int(h['outing']):03d}_{(h['driver'] or 'x').lower()}"
        rows[k] = {"path": p, "mapped": k in MAPPED, **h, "size": os.path.getsize(p)}
    # duplicates: same identity as an earlier (mapped first) row
    seen = {}
    order = list(MAPPED)  # mapped order wins: v3 canonical, US-v1 its re-export (record 2026-09-29)
    for k in sorted(rows, key=lambda x: (not rows[x]["mapped"], order.index(x) if x in order else 99, x)):
        ident = _identity(rows[k])
        rows[k]["duplicate_of"] = seen.get(ident)
        seen.setdefault(ident, k)
    print(f"{len(txt)} .txt files under {DATA_ROOT}; non-txt (not parseable):")
    for (d, ext), n in sorted(non_txt.items()):
        print(f"  {d}: {n} x .{ext}")
    for k, r in sorted(rows.items(), key=lambda kv: kv[1]["path"]):
        print(f"  {k:18s} {'mapped' if r['mapped'] else 'NEW   '} car {r['car']} drv {r['driver']} sess {r['session']} "
              f"out {r['outing']} lap0 {r['first_lap']} trk {r['track']} serial {r['serial']} "
              f"{r['size'] / 1e9:.2f} GB{'  DUPLICATE of ' + r['duplicate_of'] if r['duplicate_of'] else ''}"
              f"  [{r['path']}]")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "census.json"), "w") as fh:
        json.dump(rows, fh, indent=1)


def sessions():
    with open(os.path.join(OUT_DIR, "census.json")) as fh:
        return json.load(fh)


# ---------------------------------------------------------------- intake

def convert_units(data, factors):
    """Per-file unit conversion of UNIT_TRUSTED channels, in place on the
    cached copy; factors from intake."""
    for name, fac in factors.items():
        ch = data["channels"][name]
        ch["data"] = ch["data"] * fac
    return data


def intake(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import prepare_vehicle_state, _resolve_grid_rate, _estimate_sample_rate
    from modules.tyre_fit_auto import _base_mask
    row = sessions()[key]
    params = _params()
    t0 = time.perf_counter()
    data = parse_csv(row["path"])
    t_parse = time.perf_counter() - t0
    ch = data["channels"]
    rec = {"key": key, "path": row["path"], "header": {k: row[k] for k in ("car", "driver", "session", "outing",
                                                                          "first_lap", "track", "serial")},
           "duplicate_of": row["duplicate_of"], "parse_s": t_parse, "reject": []}

    rec["units"] = {n: ch[n]["unit_raw"] for n in UNIT_TRUSTED + OTHER_CHAIN if n in ch}
    rec["quality"] = {n: ch[n]["quality"] for n in UNIT_TRUSTED + OTHER_CHAIN if n in ch}
    rec["native_hz"] = {}
    for n in UNIT_TRUSTED + OTHER_CHAIN:
        c = ch.get(n)
        if c is not None and c.get("time") is not None and len(c["time"]) > 2:
            try:
                rec["native_hz"][n] = float(_estimate_sample_rate(c["time"]))
            except ValueError:
                rec["native_hz"][n] = float("nan")

    factors = {}
    for n in UNIT_TRUSTED:
        u = rec["units"].get(n)
        if u is None or ch[n].get("data") is None:
            continue
        exp = ch[n]["unit"]
        if u == exp or u in UNIT_ALIASES.get(exp, ()):
            continue
        if (n, u) in UNIT_FACTORS:
            factors[n] = UNIT_FACTORS[(n, u)]
        else:
            rec["reject"].append(f"unit {n} [{u}] (config [{exp}]) without a known factor")
    rec["unit_factors"] = factors
    convert_units(data, factors)

    try:
        rec["grid_hz"], rec["grid_status"] = _resolve_grid_rate(ch, params)
    except ValueError as e:
        rec["grid_hz"] = None
        rec["reject"].append(f"rate guard: {e}")

    state = prepare_vehicle_state(ch, params)
    if state is None:
        rec["reject"].append("prepare_vehicle_state None (a required channel missing/failed)")
    else:
        mv = state["moving_mask"]
        rec["moving_s"] = float(mv.sum() / state["sample_rate_hz"])
        rec["kerb_frac"] = float(state["kerb_mask"][mv].mean()) if state.get("kerb_mask") is not None else None
        stuck = {}
        tt = state["time"]
        for n in REQUIRED:
            c = ch[n]
            t_n, v_n = np.asarray(c["time"], float), np.asarray(c["data"], float)
            mv_n = np.interp(t_n, tt, mv.astype(float)) > 0.5
            d = np.diff(v_n[mv_n])
            stuck[n] = float(np.mean(d == 0)) if d.size else float("nan")
            if d.size and stuck[n] > STUCK_DEAD:
                rec["reject"].append(f"stuck {n} ({stuck[n]:.2f} zero diffs while moving)")
        rec["stuck_share"] = stuck

    laps = data.get("laps", [])
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    rec["laps_total"] = len(laps)
    rec["laps_analysis"] = [l["lap_number"] for l in an]
    if not an:
        rec["reject"].append("no analysis-valid lap")
    corners = data.get("corners", [])
    sids = sorted({c["stable_corner_id"] for c in corners if c.get("stable_corner_id") is not None})
    an_nums = {l["lap_number"] for l in an}
    per_id = {s: {c["lap_number"] for c in corners if c.get("stable_corner_id") == s} & an_nums for s in sids}
    rec["stable_ids"] = len(sids)
    rec["ids_on_every_lap"] = sum(1 for s in sids if per_id[s] == an_nums)
    if state is not None and an:
        lens = []
        for l in an:
            w = (state["time"] >= l["start_time"]) & (state["time"] <= l["end_time"])
            if np.any(np.isfinite(state["s_m"][w])):
                lens.append(float(np.nanmax(state["s_m"][w])))
        rec["lap_len_m"] = lens
        bm = _base_mask(state, laps)
        rec["base_mask_n"] = int(bm.sum())
        if not bm.any():
            rec["reject"].append("base_mask empty")
    rec["usable"] = not rec["reject"]
    _save(os.path.join(CACHE, f"data_{key}.pkl.gz"), data)
    with open(os.path.join(OUT_DIR, f"intake_{key}.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print(f"[{key}] parse {t_parse:.0f}s, usable {rec['usable']}, reject {rec['reject']}, analysis laps "
          f"{rec['laps_analysis']}, base_mask {rec.get('base_mask_n')}, factors {factors}", flush=True)


def load_data(key):
    return _load(os.path.join(CACHE, f"data_{key}.pkl.gz"))


def intake_rec(key):
    with open(os.path.join(OUT_DIR, f"intake_{key}.json")) as fh:
        return json.load(fh)


# ---------------------------------------------------------------- shared fit helpers

WEIGHTS = (0.0, 0.1, 1.0, 10.0)
W_NAME = {0.0: "none", 0.1: "weak", 1.0: "medium", 10.0: "strong"}
GRID_N = 200
TOL = 0.01
VERDICT_RANK = {"pass": 2, "warn": 1, "fail": 0}


def usable_keys():
    out = []
    for k, r in sessions().items():
        p = os.path.join(OUT_DIR, f"intake_{k}.json")
        if os.path.exists(p) and intake_rec(k)["usable"] and not r["duplicate_of"]:
            out.append(k)
    return sorted(out)


def pool_keys():
    return [k for k in usable_keys() if sessions()[k]["serial"] == "221" and k not in EXCLUDED_BY_RULING]


def _peak_deg(B, C, D, E):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, 89.9, 4000)
    s = pacejka_lateral_stiffness(np.radians(g), B, C, D, E)
    i = np.where(np.diff(np.sign(s)) < 0)[0]
    return float(g[i[0]]) if i.size else float("nan")


def curve(p, grid_rad):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    return pacejka_lateral_force(grid_rad, p["B"], p["C"], p["D"], p["E"])


def anchored_fit(alpha, Fy, base_mask, prior_ax, w):
    """Part 2 anchored fit; same dict as _fit_axle_pacejka plus the penalty
    terms. Header: PART 2."""
    from scipy.optimize import minimize
    from modules.tyre_model_pacejka import pacejka_lateral_force
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]
    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    g = np.radians(np.linspace(0.0, p99, GRID_N))
    fp = curve(prior_ax, g)
    start = (prior_ax["B"], prior_ax["C"], prior_ax["D"], prior_ax["E"])

    def _j(p):
        B, C, D, E = p
        r = pacejka_lateral_force(a2, B, C, D, E) - f2
        j = float(np.mean(r * r))
        if w:
            d = pacejka_lateral_force(g, B, C, D, E) - fp
            j += w * float(np.mean(d * d))
        return j
    res = minimize(_j, start, method="Powell")
    B, C, D, E = (float(x) for x in res.x)
    resid = f2 - pacejka_lateral_force(a2, B, C, D, E)
    pk = _peak_deg(B, C, D, E)
    dev = pacejka_lateral_force(g, B, C, D, E) - fp
    return {"B": B, "C": C, "D": D, "E": E, "w": w, "powell_converged": bool(res.success),
            "fit_n_samples": int(a2.size), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
            "curve_dev_rms_N": float(np.sqrt(np.mean(dev ** 2))), "curve_dev_max_N": float(np.max(np.abs(dev))),
            "peak_alpha_deg": pk, "peak_in_visited_range": bool(pk == pk and pk <= p99),
            "visited_alpha_p99_deg": p99, "sign_ok": D > 0, "residuals": resid, "residual_mask": m2}


def _fitter(prior, w, alpha_override=None):
    """Drop-in for _fit_axle_pacejka, called front then rear by
    fit_session_pacejka. alpha_override: {"front": arr, "rear": arr} -- the
    loop's filter-derived slip angles replace the kinematic ones."""
    order = iter(("front", "rear"))

    def fit(alpha, Fy, base_mask):
        ax = next(order)
        a = alpha if alpha_override is None else alpha_override[ax]
        return anchored_fit(a, Fy, base_mask, prior[ax], w)
    return fit


def _resolve(key, params, data, state, fitter=None):
    import modules.tyre_fit_auto as tfa
    orig = tfa._fit_axle_pacejka
    if fitter is not None:
        tfa._fit_axle_pacejka = fitter
    try:
        return tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka", csv_path=sessions()[key]["path"])
    finally:
        tfa._fit_axle_pacejka = orig


def _downstream(params, data, state, beta):
    """beta -> alpha -> CS -> stability -> summaries, as
    inspect_cs_upstream_sweep._chain builds variant A."""
    from modules.stability_analysis import (
        estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
        estimate_yaw_moment_stability, summarise_corners, estimate_vertical_loads, load_car_data,
    )
    from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
    from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
    from diagnostics.inspect_new_laps_cs_sweep import worst_lap
    ch = data["channels"]
    forces = estimate_lateral_forces(state, params)
    fz = estimate_vertical_loads(state, forces, params, channels=ch, car_data=load_car_data())
    ls = estimate_longitudinal_stiffness(estimate_longitudinal_forces(state, ch, params),
                                         estimate_slip_ratio(state, ch, params), state, params)
    slip = estimate_slip_angles(state, beta, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    summ = summarise_corners(data.get("corners", []), cs, stab, state, fz=fz, ls=ls)
    wl, _neg = worst_lap(summ)
    return {"cs_f": cs["CS_ratio_f"].astype(np.float32), "cs_r": cs["CS_ratio_r"].astype(np.float32),
            "worst_lap": wl}


def _state(key):
    from modules.stability_analysis import prepare_vehicle_state
    params = _params()
    data = load_data(key)
    return params, data, prepare_vehicle_state(data["channels"], params)


def _slim_manifest(man):
    out = {k: v for k, v in (man or {}).items() if k != "r_sweep"}
    if man and "r_sweep" in man:
        out["r_sweep_chosen"] = man["r_sweep"]["chosen"]
        out["r_sweep_found_in_band"] = man["r_sweep"]["found_in_band"]
    return out


def _run_record(beta, man, gate, fb, fr):
    return {"beta": np.asarray(beta, dtype=np.float32), "manifest": _slim_manifest(man),
            "gate": gate, "fallback": fb, "fallback_reason": fr}


# ---------------------------------------------------------------- fit (production)

def fit(key):
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
    from modules.tyre_fit_auto import _base_mask
    t0 = time.perf_counter()
    params, data, state = _state(key)
    beta, man, gate, fb, fr = _resolve(key, params, data, state)
    slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    forces = estimate_lateral_forces(state, params)
    from modules.tyre_fit_auto import _speed_window_mask
    window, window_info = _speed_window_mask(state, data, params["tyre_fit_auto"])
    base = _base_mask(state, data["laps"])
    # fit_mask = the production fit population since the speed window (2026-10-04)
    out = {"key": key, "time": state["time"], "s_m": state["s_m"], "base_mask": base,
           "fit_mask": base & window, "window_info": window_info,
           "alpha_f": slip["alpha_f_filt"], "alpha_r": slip["alpha_r_filt"],
           "fy_f": forces["Fy_f_filt"], "fy_r": forces["Fy_r_filt"],
           "laps": data["laps"],
           "corners": [{k: c.get(k) for k in ("lap_number", "bracket_start_m", "bracket_end_m", "stable_corner_id")}
                       for c in data["corners"]],
           "prod": _run_record(beta, man, gate, fb, fr)}
    out["prod"]["down"] = _downstream(params, data, state, beta)
    _save(os.path.join(CACHE, f"fit_{key}.pkl.gz"), out)
    ax = man.get("axles", {}) if man else {}
    print(f"[{key}] production status {man.get('status') if man else None}, gate "
          f"{gate.get('verdict') if gate else None} {gate.get('health_score') if gate else None}, fallback {fb}; "
          + "; ".join(f"{a} B={ax[a]['B']:.4f} C={ax[a]['C']:.4f} D={ax[a]['D']:.1f} E={ax[a]['E']:.4f}"
                      for a in ("front", "rear") if a in ax) + f"; {time.perf_counter() - t0:.0f}s", flush=True)


def load_fit(key):
    return _load(os.path.join(CACHE, f"fit_{key}.pkl.gz"))


# ---------------------------------------------------------------- Part 1

def prior():
    from modules.tyre_fit_auto import _fit_axle_pacejka
    keys = pool_keys()
    R = {k: load_fit(k) for k in keys}
    out = {"keys": keys, "n_sessions": len(keys)}
    for ax, a, f in (("front", "alpha_f", "fy_f"), ("rear", "alpha_r", "fy_r")):
        t0 = time.perf_counter()
        fit_ = _fit_axle_pacejka(np.concatenate([R[k][a] for k in keys]), np.concatenate([R[k][f] for k in keys]),
                                 np.concatenate([R[k]["base_mask"] for k in keys]))
        out[ax] = {k: v for k, v in fit_.items() if k not in ("residuals", "residual_mask")}
        print(f"P_d2 {ax:5s} B={fit_['B']:.4f} C={fit_['C']:.4f} D={fit_['D']:.1f} E={fit_['E']:.4f} peak "
              f"{fit_['peak_alpha_deg']:.3f} deg (in visited p99 {fit_['visited_alpha_p99_deg']:.3f}: "
              f"{fit_['peak_in_visited_range']}), n={fit_['fit_n_samples']}, rms {fit_['fit_rms_resid_N']:.0f} N, "
              f"converged {fit_['powell_converged']}, {time.perf_counter() - t0:.0f}s", flush=True)
    with open(os.path.join(OUT_DIR, "prior.json"), "w") as fh:
        json.dump(out, fh, indent=1)


def load_prior():
    with open(os.path.join(OUT_DIR, "prior.json")) as fh:
        return json.load(fh)


# ---------------------------------------------------------------- Part 2

def achain(key, w):
    t0 = time.perf_counter()
    params, data, state = _state(key)
    beta, man, gate, fb, fr = _resolve(key, params, data, state, _fitter(load_prior(), w))
    rec = _run_record(beta, man, gate, fb, fr)
    rec["down"] = _downstream(params, data, state, beta)
    _save(os.path.join(CACHE, f"ach_{key}_w{w:g}.pkl.gz"), rec)
    ax = man.get("axles", {}) if man else {}
    print(f"[{key} w={w:g}] status {man.get('status') if man else None}, gate {gate.get('verdict') if gate else None} "
          f"{gate.get('health_score') if gate else None}, fallback {fb}; "
          + "; ".join(f"{a} B={ax[a]['B']:.4f} C={ax[a]['C']:.4f} D={ax[a]['D']:.1f} E={ax[a]['E']:.4f} "
                      f"dev {ax[a]['curve_dev_rms_N']:.0f} N" for a in ("front", "rear") if a in ax)
          + f"; {time.perf_counter() - t0:.0f}s", flush=True)


def load_ach(key, w):
    return _load(os.path.join(CACHE, f"ach_{key}_w{w:g}.pkl.gz"))


# ---------------------------------------------------------------- Part 3

def _pass_row(p, rec):
    man = rec["manifest"]
    ax = man.get("axles", {})
    g = rec["gate"] or {}
    return {"pass": p, "status": man.get("status"), "gate": g.get("verdict"), "health": g.get("health_score"),
            "fallback": rec["fallback"], "nis": man.get("nis"), "r_derivation": man.get("r_derivation"),
            "r_sweep_chosen": man.get("r_sweep_chosen"), "r_sweep_found_in_band": man.get("r_sweep_found_in_band"),
            "axles": {a: {k: ax[a].get(k) for k in ("B", "C", "D", "E", "fit_rms_resid_N", "curve_dev_rms_N",
                                                     "peak_alpha_deg", "peak_in_visited_range",
                                                     "visited_alpha_p99_deg", "powell_converged")}
                      for a in ("front", "rear") if a in ax}}


def loop(key, w):
    from modules.stability_analysis import estimate_slip_angles
    t0 = time.perf_counter()
    params, data, state = _state(key)
    prior_p = load_prior()
    if not os.path.exists(os.path.join(CACHE, f"ach_{key}_w{w:g}.pkl.gz")):
        achain(key, w)
    rec = load_ach(key, w)
    rows = [_pass_row(1, rec)]
    print(f"[{key} w={w:g}] pass 1 {rows[-1]['gate']} {rows[-1]['health']} D_f {rows[-1]['axles']['front']['D']:.1f} "
          f"D_r {rows[-1]['axles']['rear']['D']:.1f}", flush=True)
    beta = rec["beta"].astype(float)
    for p in range(2, 5):
        slip = estimate_slip_angles(state, beta, params)
        ov = {"front": slip["alpha_f_filt"], "rear": slip["alpha_r_filt"]}
        beta_n, man, gate, fb, fr = _resolve(key, params, data, state, _fitter(prior_p, w, ov))
        rec = _run_record(beta_n, man, gate, fb, fr)
        rows.append(_pass_row(p, rec))
        print(f"[{key} w={w:g}] pass {p} {rows[-1]['gate']} {rows[-1]['health']} D_f "
              f"{rows[-1]['axles']['front']['D']:.1f} D_r {rows[-1]['axles']['rear']['D']:.1f} "
              f"{time.perf_counter() - t0:.0f}s", flush=True)
        beta = rec["beta"].astype(float)
    with open(os.path.join(OUT_DIR, f"loop_{key}_w{w:g}.json"), "w") as fh:
        json.dump({"key": key, "w": w, "passes": rows}, fh, indent=1, default=float)


def convergence(rows, grids, D_prior):
    """Header PART 3 test on the per-pass rows."""
    m, dD = [], {a: [] for a in ("front", "rear")}
    for p in range(1, len(rows)):
        mp = 0.0
        for a in ("front", "rear"):
            q0, q1 = rows[p - 1]["axles"][a], rows[p]["axles"][a]
            d_rel = abs(q1["D"] - q0["D"]) / q0["D"]
            dF = float(np.max(np.abs(curve(q1, grids[a]) - curve(q0, grids[a])))) / D_prior[a]
            dD[a].append(q1["D"] - q0["D"])
            mp = max(mp, d_rel, dF)
        m.append(mp)
    if len(m) < 3:
        return m, "INCOMPLETE"
    osc = any(np.all(np.sign(dD[a][1:]) != np.sign(dD[a][:-1]))
              and not (abs(dD[a][2]) < abs(dD[a][1]) < abs(dD[a][0])) for a in dD)
    if m[1] < m[0] and m[2] < m[1] and m[2] <= TOL:
        return m, "CONVERGES"
    if m[2] > m[1] or osc:
        return m, "DIVERGES"
    return m, "SHRINKING, NOT CONVERGED within 4 passes"


# ---------------------------------------------------------------- read-outs

def intake_report():
    S = sessions()
    mapped_units = {}
    for k in S:
        if not os.path.exists(os.path.join(OUT_DIR, f"intake_{k}.json")):
            continue
        r = intake_rec(k)
        if S[k]["mapped"]:
            for n, u in r["units"].items():
                mapped_units.setdefault(n, set()).add(u)
    print("=== Part 0 inventory")
    n_pool, n_old = 0, 0
    for k in sorted(S, key=lambda x: S[x]["path"]):
        p = os.path.join(OUT_DIR, f"intake_{k}.json")
        if not os.path.exists(p):
            print(f"  {k:18s} not parsed ({'duplicate of ' + S[k]['duplicate_of'] if S[k]['duplicate_of'] else 'skipped'})")
            continue
        r = intake_rec(k)
        h = r["header"]
        new_units = {n: u for n, u in r["units"].items() if u not in mapped_units.get(n, set())}
        role = ("DUPLICATE of " + S[k]["duplicate_of"]) if S[k]["duplicate_of"] else (
            "REJECTED" if not r["usable"] else ("out-of-pool (ruling)" if k in EXCLUDED_BY_RULING else
                                               ("POOL" if h["serial"] == "221" else "out-of-pool (serial)")))
        if role == "POOL":
            n_pool += r["base_mask_n"]
            if k in ("v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla"):
                n_old += r["base_mask_n"]
        q_bad = sorted(n for n, q in r["quality"].items() if q != "valid")
        print(f"  {k:18s} {role}: car {h['car']} drv {h['driver']} sess {h['session']} out {h['outing']} trk {h['track']} "
              f"serial {h['serial']}; grid {r.get('grid_status')}; analysis laps {len(r['laps_analysis'])} of "
              f"{r['laps_total']}; base_mask {r.get('base_mask_n')}; lap len m "
              f"{', '.join(f'{x:.0f}' for x in r.get('lap_len_m', []))}; stable ids {r['stable_ids']} "
              f"(on every analysis lap {r['ids_on_every_lap']}); kerb share {r.get('kerb_frac', float('nan')):.3f}; "
              f"stuck max {max(r.get('stuck_share', {'-': 0}).values()):.3f}; unit factors {r['unit_factors'] or 'none'}; "
              f"units not seen on mapped files {new_units or 'none'}; non-valid quality {q_bad or 'none'}"
              f"{'; REJECT ' + str(r['reject']) if r['reject'] else ''}")
    print(f"  header units on mapped files: " + "; ".join(f"{n} {sorted(u)}" for n, u in sorted(mapped_units.items())))
    print(f"  pool fit population (base_mask, upper bound) {n_pool} vs the six (d) sessions {n_old} "
          f"(record 366,989); ratio {n_pool / max(n_old, 1):.2f}")


def _verdict_counts(wl):
    from collections import Counter
    c = Counter()
    for v in wl.values():
        axle = "US" if "understeer" in v["short"] else ("OS" if "oversteer" in v["short"] else "none")
        c[(axle, v["severity"])] += 1
    return c


def _gate_s(rec):
    g = rec["gate"] or {}
    return (f"{rec['manifest'].get('status')}/{g.get('verdict')} {g.get('health_score', float('nan')):.4f}"
            f"{' FALLBACK' if rec['fallback'] else ''}")


def retry_keys():
    pool = pool_keys()
    rec = {k: intake_rec(k) for k in pool}
    most = max((k for k in pool if k != "v3"), key=lambda k: (len(rec[k]["laps_analysis"]), rec[k]["base_mask_n"]))
    few = min((k for k in pool if k not in ("v3", most)),
              key=lambda k: (len(rec[k]["laps_analysis"]), rec[k]["base_mask_n"]))
    return [most, "v3", few]


def report():
    P = load_prior()
    keys = usable_keys()
    pool = set(pool_keys())
    print(f"=== Part 1 prior: {P['n_sessions']} pool sessions: {', '.join(P['keys'])}")
    for ax in ("front", "rear"):
        p = P[ax]
        print(f"  {ax}: B={p['B']:.4f} C={p['C']:.4f} D={p['D']:.1f} E={p['E']:.4f} BCD={p['B'] * p['C'] * p['D']:.0f} "
              f"N/rad, peak {p['peak_alpha_deg']:.3f} deg (pooled p99 {p['visited_alpha_p99_deg']:.3f}), "
              f"n={p['fit_n_samples']}, rms {p['fit_rms_resid_N']:.0f} N")
    try:
        with open("diagnostics/plots/tyre_prior/priors.json") as fh:
            old = json.load(fh)["P221"]
        for ax in ("front", "rear"):
            o = old[ax]
            g = np.radians(np.linspace(0, P[ax]["visited_alpha_p99_deg"], GRID_N))
            dv = np.max(np.abs(curve(P[ax], g) - curve(o, g))) / o["D"]
            print(f"  vs (d) P221 {ax}: B={o['B']:.4f} C={o['C']:.4f} D={o['D']:.1f} E={o['E']:.4f} n={o['fit_n_samples']}; "
                  f"max curve difference / D_P221 on [0, pooled p99] {100 * dv:.2f} %")
    except FileNotFoundError:
        pass

    print("\n=== Part 2 per-session fits (curve deviation from the prior: rms over the session's [0, p99])")
    print("  per session: production (independent, chair start) | w=0 | 0.1 | 1 | 10 -- gate status/verdict health")
    for k in keys:
        f = load_fit(k)
        tag = "pool" if k in pool else "OUT"
        line = [f"prod {_gate_s(f['prod'])}"]
        for w in WEIGHTS:
            pth = os.path.join(CACHE, f"ach_{k}_w{w:g}.pkl.gz")
            line.append(f"w{w:g} {_gate_s(load_ach(k, w))}" if os.path.exists(pth) else f"w{w:g} -")
        print(f"  {k:18s} [{tag}] " + " | ".join(line))
        for ax in ("front", "rear"):
            pr = f["prod"]["manifest"]["axles"][ax]
            g = np.radians(np.linspace(0, pr["visited_alpha_p99_deg"], GRID_N))
            dprod = float(np.sqrt(np.mean((curve(pr, g) - curve(P[ax], g)) ** 2)))
            parts = [f"prod B={pr['B']:.3f} C={pr['C']:.3f} D={pr['D']:.0f} E={pr['E']:.3f} dev {dprod:.0f} N "
                     f"pk {pr['peak_alpha_deg']:.2f}{'' if pr['peak_in_visited_range'] else '(out)'}"]
            for w in WEIGHTS:
                pth = os.path.join(CACHE, f"ach_{k}_w{w:g}.pkl.gz")
                if os.path.exists(pth):
                    a = load_ach(k, w)["manifest"]["axles"][ax]
                    parts.append(f"w{w:g} B={a['B']:.3f} C={a['C']:.3f} D={a['D']:.0f} E={a['E']:.3f} dev "
                                 f"{a['curve_dev_rms_N']:.0f} pk {a['peak_alpha_deg']:.2f}"
                                 f"{'' if a['peak_in_visited_range'] else '(out)'}")
            print(f"      {ax:5s} " + " | ".join(parts))

    print("\n=== Part 2 gate health summary per weight (all usable sessions)")
    prod_fb = {k: load_fit(k)["prod"]["fallback"] for k in keys}
    prod_pass = sum(1 for k in keys if (load_fit(k)["prod"]["gate"] or {}).get("verdict") == "pass")
    gate_ok = {}
    for w in WEIGHTS:
        if not all(os.path.exists(os.path.join(CACHE, f"ach_{k}_w{w:g}.pkl.gz")) for k in keys):
            print(f"  w={w:g}: incomplete")
            continue
        A = {k: load_ach(k, w) for k in keys}
        new_fb = [k for k in keys if A[k]["fallback"] and not prod_fb[k]]
        n_pass = sum(1 for k in keys if (A[k]["gate"] or {}).get("verdict") == "pass")
        worse = [k for k in keys if VERDICT_RANK.get((A[k]["gate"] or {}).get("verdict"), -1)
                 < VERDICT_RANK.get((load_fit(k)["prod"]["gate"] or {}).get("verdict"), -1)]
        better = [k for k in keys if VERDICT_RANK.get((A[k]["gate"] or {}).get("verdict"), -1)
                  > VERDICT_RANK.get((load_fit(k)["prod"]["gate"] or {}).get("verdict"), -1)]
        gate_ok[w] = not new_fb and n_pass >= prod_pass
        print(f"  w={w:g} ({W_NAME[w]}): pass {n_pass} vs production {prod_pass}; new fallbacks {new_fb or 'none'}; "
              f"verdict worse on {worse or 'none'}; better on {better or 'none'} -> (G) {'met' if gate_ok[w] else 'NOT met'}")

    print("\n=== Part 3 convergence retry")
    conv = {}
    for k in retry_keys():
        f = load_fit(k)
        for w in WEIGHTS:
            pth = os.path.join(OUT_DIR, f"loop_{k}_w{w:g}.json")
            if not os.path.exists(pth):
                print(f"  {k} w={w:g}: not run")
                continue
            with open(pth) as fh:
                rows = json.load(fh)["passes"]
            grids = {a: np.radians(np.linspace(0, rows[0]["axles"][a]["visited_alpha_p99_deg"], GRID_N))
                     for a in ("front", "rear")}
            m, verdict = convergence(rows, grids, {a: P[a]["D"] for a in ("front", "rear")})
            conv[(k, w)] = verdict
            print(f"  {k} w={w:g} ({W_NAME[w]}): m_p = {', '.join(f'{100 * x:.2f}%' for x in m)} -> {verdict}")
            for r in rows:
                fa, ra = r["axles"]["front"], r["axles"]["rear"]
                rd = r["r_derivation"] or {}
                ch = r["r_sweep_chosen"] or {}
                print(f"      pass {r['pass']}: {r['status']}/{r['gate']} {r['health']:.4f}"
                      f"{' FALLBACK' if r['fallback'] else ''}; front B={fa['B']:.3f} C={fa['C']:.3f} D={fa['D']:.1f} "
                      f"E={fa['E']:.3f} pk {fa['peak_alpha_deg']:.2f}/p99 {fa['visited_alpha_p99_deg']:.2f} rms "
                      f"{fa['fit_rms_resid_N']:.0f}; rear B={ra['B']:.3f} C={ra['C']:.3f} D={ra['D']:.1f} E={ra['E']:.3f} "
                      f"pk {ra['peak_alpha_deg']:.2f}/p99 {ra['visited_alpha_p99_deg']:.2f} rms {ra['fit_rms_resid_N']:.0f}; "
                      f"R_ay {rd.get('r_ay_var_derived', float('nan')):.3g} R_yaw {rd.get('r_yaw_rate_var_derived', float('nan')):.3g} "
                      f"sweep ({ch.get('r_ay_scale')}, {ch.get('r_yaw_scale')}) in band {r['r_sweep_found_in_band']}; "
                      f"NIS comb {(r['nis'] or {}).get('combined_exceedance', float('nan')):.4f}")
            d1, d4 = rows[0]["axles"]["rear"]["D"], rows[-1]["axles"]["rear"]["D"]
            print(f"      watch: D_rear pass {len(rows)} / pass 1 - 1 = {100 * (d4 / d1 - 1):+.1f} % (unanchored record "
                  f"+34.6 % Dubai, +51.3 % v3); rear peak in visited p99 per pass "
                  f"{[r['axles']['rear']['peak_in_visited_range'] for r in rows]}")
    print("\n=== Choice rule")
    sel = None
    for w in (0.1, 1.0, 10.0):
        c_ok = all(conv.get((k, w)) == "CONVERGES" for k in retry_keys())
        print(f"  w={w:g}: (G) {gate_ok.get(w)}, (C) {c_ok}")
        if gate_ok.get(w) and c_ok and sel is None:
            sel = w
    print(f"  selected w = {sel if sel is not None else 'NONE (w=1 shown in Part 4 as reference only)'}")
    shown = sel if sel is not None else 1.0

    print(f"\n=== Part 4 production vs prior-anchored w={shown:g}: worst-lap verdict distribution per session")
    for k in keys:
        f = load_fit(k)
        a = load_ach(k, shown)
        cp, ca = _verdict_counts(f["prod"]["down"]["worst_lap"]), _verdict_counts(a["down"]["worst_lap"])
        changed = sorted(cid for cid in f["prod"]["down"]["worst_lap"]
                         if cid in a["down"]["worst_lap"] and
                         (f["prod"]["down"]["worst_lap"][cid]["short"], f["prod"]["down"]["worst_lap"][cid]["severity"])
                         != (a["down"]["worst_lap"][cid]["short"], a["down"]["worst_lap"][cid]["severity"]))

        def fmt(c):
            return ", ".join(f"{ax} {sev} {n}" for (ax, sev), n in sorted(c.items()))
        db = np.abs(f["prod"]["beta"].astype(float) - a["beta"].astype(float))
        bm = f["base_mask"]
        print(f"  {k:18s} [{'pool' if k in pool else 'OUT'}] prod {_gate_s(f['prod'])} | anchored {_gate_s(a)}; "
              f"|dbeta| on base_mask p50 {np.degrees(np.median(db[bm])):.2f} p95 {np.degrees(np.percentile(db[bm], 95)):.2f} deg")
        print(f"      prod: {fmt(cp)}\n      anch: {fmt(ca)}\n      corners with a changed verdict: "
              f"{len(changed)} of {len(f['prod']['down']['worst_lap'])} {changed}")
    with open(os.path.join(OUT_DIR, "selection.json"), "w") as fh:
        json.dump({"selected_w": sel, "shown_w": shown, "conv": {f"{k}|{w:g}": v for (k, w), v in conv.items()},
                   "gate_ok": {f"{w:g}": v for w, v in gate_ok.items()}}, fh, indent=1)


def render():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import render_cs_traces
    from ui.style import ACCENT, TEXT_DIM, BAD, OK, TEXT_MUTED, INFO
    P = load_prior()
    with open(os.path.join(OUT_DIR, "selection.json")) as fh:
        shown = json.load(fh)["shown_w"]
    rng = np.random.default_rng(20261001)
    for k in usable_keys():
        f = load_fit(k)
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
        for ax_i, (ax, a, fy) in enumerate((("front", "alpha_f", "fy_f"), ("rear", "alpha_r", "fy_r"))):
            m = f["base_mask"] & np.isfinite(f[a]) & np.isfinite(f[fy])
            idx = np.where(m)[0]
            idx = rng.choice(idx, size=min(6000, idx.size), replace=False)
            pl = axs[ax_i]
            pl.scatter(np.degrees(np.abs(f[a][idx])), f[fy][idx] * np.sign(f[a][idx]), s=1, color=TEXT_MUTED, alpha=0.3,
                       label="data (kinematic alpha)")
            hi = float(np.percentile(np.abs(np.degrees(f[a][m])), 99.5))
            g = np.radians(np.linspace(0, hi, 300))
            pl.plot(np.degrees(g), curve(P[ax], g), color=INFO, lw=2, label="prior")
            pl.plot(np.degrees(g), curve(f["prod"]["manifest"]["axles"][ax], g), color=BAD, lw=1.5, label="production")
            for w, col in ((0.1, OK), (1.0, ACCENT), (10.0, TEXT_DIM)):
                pth = os.path.join(CACHE, f"ach_{k}_w{w:g}.pkl.gz")
                if os.path.exists(pth):
                    pl.plot(np.degrees(g), curve(load_ach(k, w)["manifest"]["axles"][ax], g), color=col, lw=1,
                            label=f"anchored w={w:g}")
            pl.set_title(f"{k} {ax}")
            pl.set_xlabel("|alpha| [deg]")
            pl.set_ylabel("Fy (folded) [N]")
            pl.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"fit_{k}.png"), dpi=110)
        plt.close(fig)
        for tag, rec in (("prod", f["prod"]), (f"w{shown:g}", load_ach(k, shown))):
            out = os.path.join(OUT_DIR, f"traces_{k}_{tag}.png")
            cl = render_cs_traces(out, f["time"], f["s_m"], rec["down"]["cs_f"].astype(float),
                                  rec["down"]["cs_r"].astype(float), f["laps"], f["corners"], f"{k} {tag}")
            print(f"{k} {tag}: {out}; below y floor f {cl['f']}, r {cl['r']}")


def drift():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ui.style import INFO, OK, ACCENT, BAD, TEXT_MUTED
    P = load_prior()
    pass_col = {1: INFO, 2: OK, 3: ACCENT, 4: BAD}
    regions = ("inside p95", "p95-p99", "beyond p99")
    for k in retry_keys():
        f = load_fit(k)
        loops = {}
        for w in WEIGHTS:
            with open(os.path.join(OUT_DIR, f"loop_{k}_w{w:g}.json")) as fh:
                loops[w] = json.load(fh)["passes"]
        fig, axs = plt.subplots(2, len(WEIGHTS), figsize=(16, 7.5), sharey="row")
        for ai, (ax, a, fy) in enumerate((("front", "alpha_f", "fy_f"), ("rear", "alpha_r", "fy_r"))):
            m = f["base_mask"] & np.isfinite(f[a]) & np.isfinite(f[fy])
            absa = np.abs(np.degrees(f[a][m]))
            p95, p99 = float(np.percentile(absa, 95)), float(np.percentile(absa, 99))
            top = max(r["axles"][ax]["visited_alpha_p99_deg"] for w in WEIGHTS for r in loops[w])
            top = max(top, p99 * 1.01)
            g = np.linspace(0.0, top, 600)
            gr = np.radians(g)
            sel = {"inside p95": g <= p95, "p95-p99": (g > p95) & (g <= p99), "beyond p99": g > p99}
            Dp = P[ax]["D"]
            print(f"\n=== {k} {ax}: p95 {p95:.2f} deg, p99 {p99:.2f} deg, beyond p99 up to {top:.2f} deg; "
                  f"D_prior {Dp:.0f} N")
            for wi, w in enumerate(WEIGHTS):
                rows = loops[w]
                cur = [curve(r["axles"][ax], gr) for r in rows]

                def stats(d):
                    return {rg: (float(np.sqrt(np.mean(d[s] ** 2))), float(np.max(np.abs(d[s])))) for rg, s in sel.items()}
                lines = []
                for p in range(1, len(cur)):
                    st = stats(cur[p] - cur[p - 1])
                    lines.append(f"p{p}->p{p + 1} " + "; ".join(
                        f"{rg} rms {v[0]:.0f} N ({100 * v[0] / Dp:.2f}%) max {v[1]:.0f}" for rg, v in st.items()))
                tot = stats(cur[-1] - cur[0])
                verdict = ("VISITED-dominant" if tot["inside p95"][0] >= tot["beyond p99"][0]
                           else "EXTRAPOLATED-dominant")
                m95 = [float(np.max(np.abs(cur[p] - cur[p - 1])[sel["inside p95"]])) / Dp for p in range(1, len(cur))]
                rr = ("CONVERGES" if m95[1] < m95[0] and m95[2] < m95[1] and m95[2] <= TOL else
                      "DIVERGES" if m95[2] > m95[1] else "SHRINKING, NOT CONVERGED")
                print(f"  w={w:g}: TOTAL p1->p4 " + "; ".join(
                    f"{rg} rms {v[0]:.0f} N ({100 * v[0] / Dp:.2f}%) max {v[1]:.0f}" for rg, v in tot.items())
                      + f" -> {verdict}")
                for ln in lines:
                    print(f"      {ln}")
                print(f"      re-reading, inside-p95 curve change m_p = {', '.join(f'{100 * x:.2f}%' for x in m95)} -> {rr}")
                pl = axs[ai, wi]
                for r, c in zip(rows, cur):
                    pl.plot(g, c, color=pass_col[r["pass"]], lw=1.3, label=f"pass {r['pass']}")
                pl.plot(g, curve(P[ax], gr), color=TEXT_MUTED, lw=1, ls=":", label="prior")
                for x, ls in ((p95, "--"), (p99, "-.")):
                    pl.axvline(x, color=TEXT_MUTED, ls=ls, lw=0.9)
                pl.set_title(f"{ax}, w={w:g}  (-- p95, -. p99)", fontsize=9)
                pl.set_xlabel("|alpha| [deg]")
                if wi == 0:
                    pl.set_ylabel("Fy [N]")
                if ai == 0 and wi == 0:
                    pl.legend(fontsize=7)
        fig.suptitle(f"{k}: loop curves per pass, visited-range boundaries (pass-1 kinematic alpha)")
        fig.tight_layout()
        out = os.path.join(OUT_DIR, f"drift_{k}.png")
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f"  figure {out}")


# ---------------------------------------------------------------- Door 2

def anchored_fit_tw(alpha, Fy, base_mask, prior_ax, w, fs):
    """anchored_fit with the Door 2 transient sample weights (header)."""
    from scipy.optimize import minimize
    from modules.tyre_model_pacejka import pacejka_lateral_force
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]
    ad = np.abs(np.gradient(alpha, 1.0 / fs))[m2]
    med = float(np.median(ad[np.isfinite(ad)]))
    sw = np.where(np.isfinite(ad), np.maximum(1.0, ad / med), 1.0)
    sw_sum = float(np.sum(sw))
    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    g = np.radians(np.linspace(0.0, p99, GRID_N))
    fp = curve(prior_ax, g)

    def _j(p):
        B, C, D, E = p
        r = pacejka_lateral_force(a2, B, C, D, E) - f2
        j = float(np.sum(sw * r * r)) / sw_sum
        if w:
            d = pacejka_lateral_force(g, B, C, D, E) - fp
            j += w * float(np.mean(d * d))
        return j
    res = minimize(_j, (prior_ax["B"], prior_ax["C"], prior_ax["D"], prior_ax["E"]), method="Powell")
    B, C, D, E = (float(x) for x in res.x)
    resid = f2 - pacejka_lateral_force(a2, B, C, D, E)
    pk = _peak_deg(B, C, D, E)
    dev = pacejka_lateral_force(g, B, C, D, E) - fp
    return {"B": B, "C": C, "D": D, "E": E, "w": w, "powell_converged": bool(res.success),
            "fit_n_samples": int(a2.size), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
            "curve_dev_rms_N": float(np.sqrt(np.mean(dev ** 2))), "curve_dev_max_N": float(np.max(np.abs(dev))),
            "peak_alpha_deg": pk, "peak_in_visited_range": bool(pk == pk and pk <= p99),
            "visited_alpha_p99_deg": p99, "sign_ok": D > 0, "residuals": resid, "residual_mask": m2,
            "tw_median_alpha_dot": med, "tw_weight_share_above_1": float(np.mean(sw > 1.0)),
            "tw_weight_max": float(np.max(sw))}


def _fitter_tw(prior, w, fs, alpha_override=None):
    order = iter(("front", "rear"))

    def fit(alpha, Fy, base_mask):
        ax = next(order)
        a = alpha if alpha_override is None else alpha_override[ax]
        return anchored_fit_tw(a, Fy, base_mask, prior[ax], w, fs)
    return fit


def tloop(key, w):
    from modules.stability_analysis import estimate_slip_angles
    t0 = time.perf_counter()
    params, data, state = _state(key)
    prior_p = load_prior()
    fs = float(state["sample_rate_hz"])
    rows, ov = [], None
    for p in range(1, 5):
        beta_n, man, gate, fb, fr = _resolve(key, params, data, state, _fitter_tw(prior_p, w, fs, ov))
        rec = _run_record(beta_n, man, gate, fb, fr)
        row = _pass_row(p, rec)
        for a in ("front", "rear"):
            row["axles"][a].update({k: man["axles"][a].get(k) for k in
                                    ("tw_median_alpha_dot", "tw_weight_share_above_1", "tw_weight_max")})
        rows.append(row)
        print(f"[tw {key} w={w:g}] pass {p} {row['gate']} {row['health']} D_f {row['axles']['front']['D']:.1f} "
              f"D_r {row['axles']['rear']['D']:.1f} {time.perf_counter() - t0:.0f}s", flush=True)
        slip = estimate_slip_angles(state, rec["beta"].astype(float), params)
        ov = {"front": slip["alpha_f_filt"], "rear": slip["alpha_r_filt"]}
    with open(os.path.join(OUT_DIR, f"loop_tw_{key}_w{w:g}.json"), "w") as fh:
        json.dump({"key": key, "w": w, "passes": rows}, fh, indent=1, default=float)


def door2():
    P = load_prior()
    ref = {"nbr_o010_jdr": 3.20, "v3": 5.53, "nbr_o019_jst": 5.70}
    hits = 0
    for k in retry_keys():
        f = load_fit(k)
        sp = {}
        for ax, a, fy in (("front", "alpha_f", "fy_f"), ("rear", "alpha_r", "fy_r")):
            m = f["base_mask"] & np.isfinite(f[a]) & np.isfinite(f[fy])
            sp[ax] = float(np.percentile(np.abs(np.degrees(f[a][m])), 95))
        for w in (0.0, 1.0):
            out = {}
            for tag, fn in (("uniform", f"loop_{k}_w{w:g}.json"), ("transient", f"loop_tw_{k}_w{w:g}.json")):
                with open(os.path.join(OUT_DIR, fn)) as fh:
                    rows = json.load(fh)["passes"]
                m, tot = [], {}
                for p in range(1, len(rows)):
                    mp = 0.0
                    for ax in ("front", "rear"):
                        g = np.radians(np.linspace(0.0, sp[ax], 400))
                        d = curve(rows[p]["axles"][ax], g) - curve(rows[p - 1]["axles"][ax], g)
                        mp = max(mp, float(np.max(np.abs(d))) / P[ax]["D"])
                    m.append(mp)
                for ax in ("front", "rear"):
                    g = np.radians(np.linspace(0.0, sp[ax], 400))
                    d = curve(rows[-1]["axles"][ax], g) - curve(rows[0]["axles"][ax], g)
                    tot[ax] = float(np.sqrt(np.mean(d ** 2))) / P[ax]["D"]
                out[tag] = (m, tot, rows)
            mu, mt = out["uniform"][0], out["transient"][0]
            walk_u, walk_t = mu[1] + mu[2], mt[1] + mt[2]
            sig = walk_t <= 0.5 * walk_u
            if w == 0.0 and sig:
                hits += 1
            print(f"{k} w={w:g}{'' if w == 0.0 else ' (not scored)'}: inside-p95 m' per pass uniform "
                  f"{', '.join(f'{100 * x:.2f}%' for x in mu)} | transient {', '.join(f'{100 * x:.2f}%' for x in mt)}; "
                  f"walk 2->4 uniform {100 * walk_u:.2f}% (addendum {ref[k]:.2f}%) vs transient {100 * walk_t:.2f}% "
                  f"-> ratio {walk_t / walk_u:.2f}{' HALVED' if sig else ''}")
            for tag in ("uniform", "transient"):
                _m, tot, rows = out[tag]
                r1, r4 = rows[0]["axles"], rows[-1]["axles"]
                print(f"    {tag:9s} total p1->p4 inside-p95 rms f {100 * tot['front']:.2f}% r {100 * tot['rear']:.2f}%; "
                      f"D_rear p4/p1-1 {100 * (r4['rear']['D'] / r1['rear']['D'] - 1):+.1f}%, D_front "
                      f"{100 * (r4['front']['D'] / r1['front']['D'] - 1):+.1f}%; gates "
                      f"{[(r['gate'], round(r['health'], 4)) for r in rows]}")
            r1t = out["transient"][2][0]["axles"]
            print(f"    transient weights pass 1: median |alpha_dot| f {np.degrees(r1t['front']['tw_median_alpha_dot']):.2f} "
                  f"r {np.degrees(r1t['rear']['tw_median_alpha_dot']):.2f} deg/s; share w>1 "
                  f"{r1t['front']['tw_weight_share_above_1']:.2f}/{r1t['rear']['tw_weight_share_above_1']:.2f}; "
                  f"max w {r1t['front']['tw_weight_max']:.0f}/{r1t['rear']['tw_weight_max']:.0f}")
    print(f"\nDoor 2: sessions with the post-switch walk halved at w = 0: {hits} of 3 -> "
          f"{'SIGNAL' if hits >= 2 else 'NO SIGNAL, door closed'}")


# ---------------------------------------------------------------- Door 1, Stage 0

D1_KEYS = ("v3", "pr_us_v2", "dubai")
X_IMU_M = 0.20          # SCLU ahead of the CG, author estimate 2026-10-01
D1_BRAKE_MAX = 5.0      # bar, WP-S1's braking threshold
D1_GRAD_MAX = 0.01
D1_GRAD_SHARE = 0.90
D1_MIN_N, D1_MIN_TRAV = 2000, 20
D1_ALT_BIN_M, D1_ALT_SMOOTH_M = 5.0, 100.0
D1_BOOT = 1000


def d1loop(key):
    """PART 3 loop at w 0, unchanged, with beta kept per pass."""
    from modules.stability_analysis import estimate_slip_angles
    t0 = time.perf_counter()
    params, data, state = _state(key)
    prior_p = load_prior()
    if not os.path.exists(os.path.join(CACHE, f"ach_{key}_w0.pkl.gz")):
        achain(key, 0.0)
    rec = load_ach(key, 0.0)
    rows, betas = [_pass_row(1, rec)], [rec["beta"].astype(np.float32)]
    beta = rec["beta"].astype(float)
    for p in range(2, 5):
        slip = estimate_slip_angles(state, beta, params)
        ov = {"front": slip["alpha_f_filt"], "rear": slip["alpha_r_filt"]}
        rec = _run_record(*_resolve(key, params, data, state, _fitter(prior_p, 0.0, ov)))
        rows.append(_pass_row(p, rec))
        betas.append(rec["beta"].astype(np.float32))
        beta = rec["beta"].astype(float)
        print(f"[d1 {key}] pass {p} {rows[-1]['gate']} D_r {rows[-1]['axles']['rear']['D']:.1f} "
              f"{time.perf_counter() - t0:.0f}s", flush=True)
    _save(os.path.join(CACHE, f"d1beta_{key}.pkl.gz"), {"betas": betas, "rows": rows})
    with open(os.path.join(OUT_DIR, f"loop_d1_{key}_w0.json"), "w") as fh:
        json.dump({"key": key, "w": 0.0, "passes": rows}, fh, indent=1, default=float)


def _d1_raw(key, name):
    """Raw block on the chain grid via the streaming reader, cached. Unit
    read from this file's header (standing warning)."""
    pth = os.path.join(CACHE, f"d1raw_{key}_{name}.pkl.gz")
    if os.path.exists(pth):
        return _load(pth)
    from diagnostics.inspect_imu_offsets import read_raw_block
    t, v, hdr = read_raw_block(sessions()[key]["path"], name)
    out = None if t is None else {"t": t, "v": v, "unit": hdr[hdr.index("[") + 1:-1] if "[" in hdr else None}
    _save(pth, out)
    return out


def _d1_gradient(key, st, laps):
    """E1: lap-averaged altitude profile -> gradient at every grid sample."""
    raw = _d1_raw(key, "log_gps_alt")
    if raw is None:
        return None, "log_gps_alt absent"
    fac = {"m": 1.0, "ft": 0.3048}.get(raw["unit"])
    if fac is None:
        return None, f"log_gps_alt unit [{raw['unit']}] unknown"
    alt_t, alt = raw["t"], raw["v"] * fac
    s_alt = np.interp(alt_t, st["time"], st["s_m"])
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    smax = max(np.nanmax(st["s_m"][(st["time"] >= l["start_time"]) & (st["time"] <= l["end_time"])]) for l in an)
    nb = int(np.ceil(smax / D1_ALT_BIN_M))
    prof = []
    for l in an:
        m = (alt_t >= l["start_time"]) & (alt_t <= l["end_time"]) & np.isfinite(s_alt)
        b = np.clip((s_alt[m] / D1_ALT_BIN_M).astype(int), 0, nb - 1)
        s_ = np.bincount(b, alt[m], nb)
        c_ = np.bincount(b, None, nb)
        p = np.where(c_ > 0, s_ / np.maximum(c_, 1), np.nan)
        prof.append(p - np.nanmean(p))
    prof = np.nanmean(np.array(prof), axis=0)
    ok = np.isfinite(prof)
    prof = np.interp(np.arange(nb), np.where(ok)[0], prof[ok])
    k = int(round(D1_ALT_SMOOTH_M / D1_ALT_BIN_M))
    sm = np.convolve(np.pad(prof, k, mode="edge"), np.ones(k) / k, mode="same")[k:-k]
    grad = np.gradient(sm, D1_ALT_BIN_M)
    s_c = (np.arange(nb) + 0.5) * D1_ALT_BIN_M
    return np.interp(st["s_m"], s_c, grad), f"alt range of profile {np.ptp(sm):.1f} m"


def _d1_signals(key):
    from modules.stability_analysis import estimate_sideslip
    from modules.longitudinal_forces import _guarded_wheel_speed_kmh
    from diagnostics.gated_kinematic_common import racing_mask, otsu_threshold, valid_corner_instances
    from diagnostics.inspect_imu_offsets import _lp, PSIDD_MAX, AX_MAX, CLIP_G, G
    from diagnostics.inspect_gated_kinematic_scale_sign import STRAIGHT_AY_FRACTION
    params, data, st = _state(key)
    sr, fc = st["sample_rate_hz"], params["stability_estimation"]["cs_filter_cutoff_hz"]
    t, laps = st["time"], data["laps"]
    ch = data["channels"]
    vfl, sfl = _guarded_wheel_speed_kmh(ch, "fl", t, st["moving_mask"], sr, params)
    vfr, sfr = _guarded_wheel_speed_kmh(ch, "fr", t, st["moving_mask"], sr, params)
    src_ok = (sfl == "log_speed") & (sfr == "log_speed")
    v_f = _lp(np.nan_to_num(0.5 * (vfl + vfr) / 3.6), fc, sr)
    vdot = np.gradient(v_f, t)
    r = _lp(st["yaw_rate_radps"], fc, sr)
    ax_imu = _lp(st["ax_mps2"], fc, sr)
    ay = _lp(st["ay_mps2"], fc, sr)
    v = _lp(st["v_mps"], fc, sr)
    psidd = np.gradient(r, t)
    racing = racing_mask(st, laps)
    thr = otsu_threshold(np.abs(st["ay_mps2"][racing]))
    inst = valid_corner_instances(data, t, st["s_m"])
    trav = np.full(t.size, -1)
    for j, (_sid, _lap, sl) in enumerate(inst):
        trav[sl] = j
    brake = st["brake_f_bar"] if st.get("brake_f_bar") is not None else np.zeros(t.size)
    kerb = st["kerb_mask"] if st.get("kerb_mask") is not None else np.zeros(t.size, dtype=bool)
    common = (racing & (np.abs(psidd) < PSIDD_MAX) & (np.abs(ax_imu) < AX_MAX) & ~kerb & (brake < D1_BRAKE_MAX)
              & src_ok & (np.abs(st["ay_mps2"]) < CLIP_G * G))
    Q = common & (trav >= 0) & (np.abs(r * v) > thr)
    S = common & (trav < 0) & (np.abs(st["ay_mps2"]) < STRAIGHT_AY_FRACTION * thr)
    sc = _d1_raw(key, "sclu_acc_x")
    ax_sc = None
    if sc is not None and np.std(sc["v"]) > 0:
        fac = {"G": G, "g": G, "m/s/s": 1.0, "m/s^2": 1.0}.get(sc["unit"])
        if fac is not None:
            ax_sc = _lp(np.interp(t, sc["t"], sc["v"]) * fac, fc, sr)
    grad, grad_note = _d1_gradient(key, st, laps)
    return {"t": t, "laps": laps, "v": v, "r": r, "ay": ay, "ax": ax_imu + X_IMU_M * r ** 2, "ax_imu": ax_imu,
            "vdot": vdot, "Q": Q, "S": S, "trav": trav, "ax_sc": ax_sc, "grad": grad, "grad_note": grad_note,
            "beta_kin": estimate_sideslip(st, params), "thr": thr, "n_inst": len(inst)}


def _d1_ols_rms(y, X, idx):
    c, *_ = np.linalg.lstsq(X[idx], y[idx], rcond=None)
    res = y[idx] - X[idx] @ c
    return float(np.sqrt(np.mean(res * res))), c


def door1():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rng = np.random.default_rng(0)
    lines, verdict = [], {}
    for key in D1_KEYS:
        d = _d1_signals(key)
        bl = _load(os.path.join(CACHE, f"d1beta_{key}.pkl.gz"))
        cands = {"zero": np.zeros_like(d["t"]), "kin": d["beta_kin"]}
        for p, b in enumerate(bl["betas"], start=1):
            cands[f"p{p}"] = b.astype(float)
        Q, S = d["Q"], d["S"]
        qi = np.where(Q & np.all([np.isfinite(c) for c in cands.values()], axis=0))[0]
        trav_q = d["trav"][qi]
        n_trav = np.unique(trav_q).size
        # entry
        if d["grad"] is None:
            e1, e1s = False, float("nan")
        else:
            e1s = float(np.mean(np.abs(d["grad"][qi]) <= D1_GRAD_MAX))
            e1 = e1s >= D1_GRAD_SHARE
        e2 = qi.size >= D1_MIN_N and n_trav >= D1_MIN_TRAV
        lines.append(f"\n[{key}] Q n={qi.size} traversals {n_trav} (instances {d['n_inst']}), S n={int(S.sum())}; "
                     f"E1 share |grad|<=1% {e1s:.3f} ({d['grad_note']}; p95 |grad| "
                     f"{100 * np.nanpercentile(np.abs(d['grad'][qi]), 95) if d['grad'] is not None else float('nan'):.2f}%) "
                     f"-> {'PASS' if e1 else 'FAIL'}; E2 {'PASS' if e2 else 'FAIL'}")
        # E3
        if d["ax_sc"] is not None:
            m3 = np.where(Q | S)[0]
            _r, c3 = _d1_ols_rms(d["ax_imu"] - d["ax_sc"], np.column_stack([np.ones_like(d["t"]), d["ay"]]), m3)
            lines.append(f"  E3 log_acc_x - sclu_acc_x = {c3[0]:+.3f} m/s^2 + {c3[1]:+.4f} a_y -> mutual yaw "
                         f"{np.degrees(np.arctan(c3[1])):+.2f} deg")
        else:
            lines.append("  E3 sclu_acc_x unusable/absent")
        # floor
        si = np.where(S)[0]
        F, _ = _d1_ols_rms(d["ax"] - d["vdot"], np.column_stack([np.ones_like(d["t"]), d["v"] ** 2]), si)
        ay_med = float(np.median(np.abs(d["ay"][qi])))
        lines.append(f"  floor S: RMS {F:.3f} m/s^2; median |a_y| over Q {ay_med:.2f} m/s^2 -> beta-equivalent "
                     f"{np.degrees(F / ay_med):.2f} deg per sample")
        X = np.column_stack([np.ones_like(d["t"]), d["ay"], d["v"] ** 2])
        Xr = np.column_stack([X, d["r"] ** 2])
        ys = {k: d["ax"] * np.cos(b) + d["ay"] * np.sin(b) - d["vdot"] for k, b in cands.items()}
        ys_r = {k: d["ax_imu"] * np.cos(b) + d["ay"] * np.sin(b) - d["vdot"] for k, b in cands.items()}
        rms = {k: _d1_ols_rms(y, X, qi)[0] for k, y in ys.items()}
        rms_r = {k: _d1_ols_rms(y, Xr, qi)[0] for k, y in ys_r.items()}
        # walk signal: nuisance-projected a_y (sin b1 - sin b4)
        dw = d["ay"] * (np.sin(cands["p1"]) - np.sin(cands["p4"]))
        W, _ = _d1_ols_rms(dw, X, qi)
        db = np.degrees(np.abs(cands["p1"] - cands["p4"])[qi])
        lines.append(f"  walk p1 vs p4: |dbeta| over Q median {np.median(db):.2f} p95 {np.percentile(db, 95):.2f} deg; "
                     f"projected signal RMS {W:.3f} m/s^2 ({np.degrees(W / ay_med):.2f} deg-equiv)")
        lines.append("  residual RMS (m/s^2), fixed X_IMU: " + ", ".join(f"{k} {v_:.4f}" for k, v_ in rms.items()))
        lines.append("  sensitivity, r^2 fitted (not scored): " + ", ".join(f"{k} {v_:.4f}" for k, v_ in rms_r.items()))
        # bootstrap dRMS p1 - p4
        ut = np.unique(trav_q)
        by = {u: qi[trav_q == u] for u in ut}
        boots = []
        for _ in range(D1_BOOT):
            ii = np.concatenate([by[u] for u in rng.choice(ut, ut.size, replace=True)])
            boots.append(_d1_ols_rms(ys["p1"], X, ii)[0] - _d1_ols_rms(ys["p4"], X, ii)[0])
        lo, hi = np.percentile(boots, [2.5, 97.5])
        dr = rms["p1"] - rms["p4"]
        ds = 0 if lo <= 0 <= hi else int(np.sign(dr))
        best = min(("kin", "p1", "p2", "p3", "p4"), key=lambda k: rms[k])
        lines.append(f"  dRMS p1-p4 {dr:+.5f} m/s^2, 95% CI [{lo:+.5f}, {hi:+.5f}] -> D_s {ds:+d}; lowest of kin/1-4: {best}")
        verdict[key] = {"entry": e1 and e2, "ds": ds, "best": best}
        # figure: longest analysis lap
        an = [l for l in d["laps"] if l.get("is_valid_for_analysis")]
        lap = an[0]
        w_ = (d["t"] >= lap["start_time"]) & (d["t"] <= lap["end_time"])
        fig, axs = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
        tt = d["t"][w_] - lap["start_time"]
        axs[0].plot(tt, (d["ax"] - d["vdot"])[w_], lw=0.7, label="a_x,CG - vdot_front")
        for k in ("kin", "p1", "p4"):
            axs[0].plot(tt, (-d["ay"] * np.sin(cands[k]))[w_], lw=0.8, label=f"-a_y sin beta {k}")
        axs[0].set_ylabel("m/s^2"); axs[0].legend(fontsize=7, ncol=4)
        for k in ("kin", "p1", "p4"):
            axs[1].plot(tt, np.degrees(cands[k][w_]), lw=0.8, label=k)
        axs[1].set_ylabel("beta deg"); axs[1].legend(fontsize=7)
        axs[2].plot(tt, d["ay"][w_], lw=0.7, label="a_y")
        axs[2].fill_between(tt, -20, 20, where=Q[w_], alpha=0.15, label="Q")
        axs[2].set_ylim(-20, 20); axs[2].set_ylabel("m/s^2"); axs[2].legend(fontsize=7)
        axs[2].set_xlabel(f"time in lap {lap['lap_number']} (s)")
        fig.suptitle(f"Door 1 Stage 0 -- {key}")
        fig.savefig(os.path.join(OUT_DIR, f"door1_{key}.png"), dpi=110, bbox_inches="tight")
        plt.close(fig)
    ok = [k for k in D1_KEYS if verdict[k]["entry"]]
    nz = [verdict[k]["ds"] for k in ok if verdict[k]["ds"] != 0]
    kin_best = sum(1 for k in ok if verdict[k]["best"] == "kin")
    if len(ok) < 2:
        v_ = "N4: fewer than 2 sessions pass entry -> CLOSED on inventory"
    elif len(nz) < 2:
        v_ = "N2: fewer than 2 sessions discriminate -> CLOSED"
    elif max(nz.count(1), nz.count(-1)) < 2 or kin_best >= 2:
        v_ = "N3: inconsistent preference or kinematic seed best -> CLOSED"
    else:
        v_ = "Stage 0 PASS (Stage 1 needs the author's look)"
    lines.append(f"\nentry-passing sessions {ok}; nonzero D_s {nz}; kinematic seed lowest on {kin_best}\nVERDICT: {v_}")
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT_DIR, "door1.txt"), "w") as fh:
        fh.write(txt + "\n")


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode in ("intake", "fit", "d1loop"):
        {"intake": intake, "fit": fit, "d1loop": d1loop}[mode](sys.argv[2])
    elif mode in ("achain", "loop", "tloop"):
        {"achain": achain, "loop": loop, "tloop": tloop}[mode](sys.argv[2], float(sys.argv[3]))
    else:
        {"census": census, "prior": prior, "intake_report": intake_report, "report": report,
         "render": render, "drift": drift, "door2": door2, "door1": door1}[mode]()
