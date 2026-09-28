# DIAGNOSTIC, read-only, [keep-reproduces]. New-session checklist, CS_ratio
# extremity/flapping baselines and the CS smoothing/persistence sweep,
# across every available session (Dubai and v3 as baselines).
#
# Nothing in modules/ or config/ is edited. Variants are built IN MEMORY:
# a deepcopy of the resolved params with one key changed, fed to the
# unchanged production functions, or a post-processing step on their
# output. Tier B throughout (filters, window floors, read-out persistence,
# rolling medians are standard signal techniques) -- measurement only.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_new_laps_cs_sweep channels <key>
#   python -m diagnostics.inspect_new_laps_cs_sweep compute <key>
#   python -m diagnostics.inspect_new_laps_cs_sweep report
# compute writes diagnostics/plots/new_laps/cache/<key>.pkl.gz (gitignored);
# report reads every cache present and prints Parts 1-3.
#
# PRE-REGISTERED DEFINITIONS (fixed before the first run):
# - Corner traversal: one analysis-lap instance of a stable corner, sliced
#   by its canonical bracket (inspect_step2_chair_plots helpers).
# - Leading-axle call per sample: F (front-led, understeer side) when
#   CS_ratio_f < CS_ratio_r, R (rear-led) when CS_ratio_r < CS_ratio_f, no
#   call on a tie (both clipped at 1.0) or when either is NaN. A no-call
#   sample neither counts nor resets anything.
# - Flapping = F<->R transitions of that call inside traversals, pooled per
#   session as transitions per 100 m of called track. Length scale = run
#   lengths in metres between transitions; "sub-resolution" = shorter than
#   the session's own median CS window footprint (the estimator cannot
#   resolve a balance change shorter than the window it regresses over).
# - Phase-level call: same F/R rule on the five phase medians (apex from
#   apex_region). Lap-level: production verdict axle per lap instance
#   (_classify_corner), switches between consecutive analysis laps.
# - D persistence: the reported call may switch only after the new call
#   has held for L metres of track without interruption; a no-call sample
#   or a return to the old call breaks the pending run. Read-out only --
#   CS values, summaries and verdicts are identical to A by construction.
# - E rolling median: centred, samples (5, 11), applied per contiguous
#   finite run so NaN gaps are never filled or bridged.
# - HARD bar: Dubai C4 worst-lap CS_ratio_f < STRONG_CSF (record: -0.134,
#   STRONG) AND the C3 traction signature at its recorded class. The record
#   defines that signature as LS_ratio_r exit_5 negative on 4/4 laps; it is
#   scored as such, and the CS reading (C3 worst-lap CS_f STRONG) is
#   reported alongside as a second, non-disqualifying column.
# - Driver agreement (Dubai, June report "balanced, mild understeer" and
#   feedback C4 exit -4): (i) understeer corners >= oversteer corners,
#   (ii) no severity "strong", (iii) C4 carries a non-ok verdict.

import copy
import gzip
import os
import pickle
import sys
import time

import numpy as np

SESSIONS = {
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
OUT_DIR = "diagnostics/plots/new_laps"
CACHE_DIR = os.path.join(OUT_DIR, "cache")

# Registry levers tc_lat/tc_lon/abs_position/brake_bias: candidate channel
# names found by header scan. Content census only; nothing is wired.
LEVER_CHANNEL_CANDIDATES = [
    "log_tc_lat_pos", "log_tc_lon_pos", "log_tc_int_pos", "log_tc_tim_pos",
    "ecu_TC_int_pos", "ecu_TC_tim_pos", "stw_rt01_tc_tim", "stw_rt03_tc_int",
    "Math_TCInt_Bit", "log_NTCOptionPos", "ecu_B_tc_act", "pcu_ecu_B_tc_act",
    "log_abs_pos", "log_rt_abs_pos", "abs_switch_pos", "Math_D_ABSPos_Bit", "abs_active",
    "brake_balance", "r_brkbias", "log_brkbias", "log_brkbias_dash",
    "Math_Brake_Balance_Fr", "Math_Brake_Bias_Hold", "abs_brk_bal_prop", "log_B_brkbias_change",
]

VARIANTS_CS = {  # variant -> (param overrides) for a real re-estimation
    "B_1.0Hz": {"cs_filter_cutoff_hz": 1.0},
    "B_0.5Hz": {"cs_filter_cutoff_hz": 0.5},
    "C_span_x1.5": {"cs_min_slip_angle_span_rad": "x1.5"},
    "C_span_x2": {"cs_min_slip_angle_span_rad": "x2"},
}
E_WINDOWS = {"E_median_5": 5, "E_median_11": 11}
D_LENGTHS = {"D_persist_10m": 10.0, "D_persist_25m": 25.0}
ROW_ORDER = ["A_baseline", "B_1.0Hz", "B_0.5Hz", "C_span_x1.5", "C_span_x2",
             "D_persist_10m", "D_persist_25m", "E_median_5", "E_median_11", "B_1.0Hz+D_25m"]
PHASES = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _strip_unit(raw_name):
    return raw_name[:raw_name.index("[")].strip() if "[" in raw_name else raw_name.strip()


# ---------------------------------------------------------------- channels

def channel_content_census(key):
    """Streams the raw file once and reads only the candidate blocks --
    parse_csv loads the channels.json whitelist, which these are not on."""
    path = SESSIONS[key]
    wanted = set(LEVER_CHANNEL_CANDIDATES)
    out = {}
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.strip() == "{ChannelBlock}":
                header = f.readline().rstrip("\n").split("\t")
                name = _strip_unit(header[1]) if len(header) == 2 else None
                if name in wanted:
                    ts, vs = [], []
                    line = f.readline()
                    while line and not line.startswith("{"):
                        # Dubai/Le Mans exports use a decimal comma
                        parts = line.replace(",", ".").split("\t")
                        if len(parts) == 2:
                            try:
                                ts.append(float(parts[0])); vs.append(float(parts[1]))
                            except ValueError:
                                pass
                        line = f.readline()
                    t = np.array(ts); v = np.array(vs)
                    rate = float(1.0 / np.median(np.diff(t))) if t.size > 2 else float("nan")
                    uniq = np.unique(v[np.isfinite(v)]) if v.size else np.array([])
                    out[name] = {
                        "unit": header[1], "n": int(v.size), "rate_hz": rate,
                        "n_distinct": int(uniq.size),
                        "min": float(uniq.min()) if uniq.size else None,
                        "max": float(uniq.max()) if uniq.size else None,
                        "values_head": [float(x) for x in uniq[:8]],
                        "n_changes": int(np.sum(np.diff(v) != 0)) if v.size > 1 else 0,
                    }
                    continue
            line = f.readline()
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"lever_channels_{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    for name in LEVER_CHANNEL_CANDIDATES:
        r = out.get(name)
        if r is None:
            print(f"  {name:<22} absent")
        else:
            print(f"  {name:<22} n={r['n']:>8} rate={r['rate_hz']:7.2f}Hz distinct={r['n_distinct']:>5} "
                  f"range=[{r['min']}, {r['max']}] changes={r['n_changes']} head={r['values_head']}")


# ---------------------------------------------------------------- compute

def _rolling_median_finite_runs(arr, window):
    from scipy.ndimage import median_filter
    out = np.full_like(arr, np.nan)
    finite = np.isfinite(arr)
    if not finite.any():
        return out
    edges = np.diff(np.concatenate(([0], finite.astype(np.int8), [0])))
    starts, stops = np.where(edges == 1)[0], np.where(edges == -1)[0]
    for a, b in zip(starts, stops):
        out[a:b] = median_filter(arr[a:b], size=window, mode="nearest")
    return out


def _window_termination(alpha, cs_ratio, idxs, min_window, min_span, s_m, max_window_m):
    """Per corner sample: window length n [samples], footprint [m], and how
    the production widening loop ended -- 'floor' (span already met inside
    the minimum window), 'span' (widened until span met), 'cap' (53 m cap
    hit before span -> NaN). Reconstruction via the production helper."""
    from modules.stability_analysis import reconstruct_cs_window_start
    n_arr = np.zeros(idxs.size, dtype=np.int32)
    fp = np.full(idxs.size, np.nan)
    kind = np.empty(idxs.size, dtype="<U5")
    start_idx = np.zeros(idxs.size, dtype=np.int64)
    for k, i in enumerate(idxs):
        i = int(i)
        st = reconstruct_cs_window_start(alpha, i, min_window, min_span, s_m=s_m, max_window_m=max_window_m)
        start_idx[k] = st
        n_arr[k] = i - st
        if np.isfinite(s_m[i - 1]) and np.isfinite(s_m[st]):
            fp[k] = s_m[i - 1] - s_m[st]
        w = alpha[st:i]
        span = (np.max(w) - np.min(w)) if w.size else 0.0
        if span < min_span:
            kind[k] = "cap"
        elif i - st <= min_window:
            kind[k] = "floor"
        else:
            kind[k] = "span"
    return {"n": n_arr, "footprint_m": fp, "kind": kind, "start": start_idx,
            "cs_finite": np.isfinite(cs_ratio[idxs])}


def compute(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        load_parameters, prepare_vehicle_state, load_car_data, estimate_slip_angles,
        estimate_lateral_forces, estimate_cornering_stiffness, estimate_yaw_moment_stability,
        estimate_vertical_loads, summarise_corners, resolve_cs_min_window_samples, _resolve_grid_rate,
    )
    from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
    from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import resolve_sideslip_beta
    from diagnostics.inspect_step2_chair_plots import _valid_lap_instances, _canonical_window_slice

    path = SESSIONS[key]
    timings = {}
    t0 = time.perf_counter()
    # same call sequence and accuracy cap as inspect_frame_stage2_parity.run_full_pipeline
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))
    data = parse_csv(path)
    timings["parse"] = time.perf_counter() - t0
    channels = data["channels"]

    native = {}
    for name, ch in channels.items():
        tt = ch.get("time")
        if tt is not None and len(tt) > 2:
            d = np.diff(np.asarray(tt, dtype=float)); d = d[d > 0]
            native[name] = (float(1.0 / np.median(d)) if d.size else float("nan"), ch.get("quality"))
        else:
            native[name] = (float("nan"), ch.get("quality"))
    try:
        grid = _resolve_grid_rate(channels, params)
        grid_err = None
    except ValueError as e:
        grid, grid_err = None, str(e)

    state = prepare_vehicle_state(channels, params)
    if state is None:
        raise RuntimeError(f"{key}: prepare_vehicle_state returned None ({grid_err})")
    live = params["stability_estimation"].get("sideslip_source", "kinematic")
    t1 = time.perf_counter()
    beta, _man, gate_verdict, fb_used, fb_reason = resolve_sideslip_beta(state, params, data, live, csv_path=path)
    timings["beta"] = time.perf_counter() - t1
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    t1 = time.perf_counter()
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    timings["cs_baseline"] = time.perf_counter() - t1
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, params, channels=channels, car_data=load_car_data())
    t1 = time.perf_counter()
    ls = estimate_longitudinal_stiffness(estimate_longitudinal_forces(state, channels, params),
                                         estimate_slip_ratio(state, channels, params), state, params)
    timings["ls"] = time.perf_counter() - t1
    corners = data.get("corners", [])
    summaries = {"A_baseline": summarise_corners(corners, cs, stab, state, fz=fz, ls=ls)}
    cs_arrays = {"A_baseline": (cs["CS_ratio_f"].astype(np.float32), cs["CS_ratio_r"].astype(np.float32))}

    se = params["stability_estimation"]
    for vname, overrides in VARIANTS_CS.items():
        p = copy.deepcopy(params)
        for k, v in overrides.items():
            p["stability_estimation"][k] = se[k] * float(v[1:]) if isinstance(v, str) else v
        t1 = time.perf_counter()
        if "cs_filter_cutoff_hz" in overrides:
            cs_v = estimate_cornering_stiffness(estimate_slip_angles(state, beta, p),
                                                estimate_lateral_forces(state, p), state, p)
        else:
            cs_v = estimate_cornering_stiffness(slip, forces, state, p)
        timings[f"cs_{vname}"] = time.perf_counter() - t1
        cs_arrays[vname] = (cs_v["CS_ratio_f"].astype(np.float32), cs_v["CS_ratio_r"].astype(np.float32))
        summaries[vname] = summarise_corners(corners, cs_v, stab, state, fz=fz, ls=ls)
        print(f"[{key}] {vname} done {timings[f'cs_{vname}']:.0f}s", flush=True)
    for vname, w in E_WINDOWS.items():
        cf = _rolling_median_finite_runs(cs["CS_ratio_f"], w)
        cr = _rolling_median_finite_runs(cs["CS_ratio_r"], w)
        cs_arrays[vname] = (cf.astype(np.float32), cr.astype(np.float32))
        summaries[vname] = summarise_corners(corners, {"CS_ratio_f": cf, "CS_ratio_r": cr}, stab, state, fz=fz, ls=ls)

    # baseline window termination on corner samples of analysis laps
    t, s_m = state["time"], state["s_m"]
    laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
    moving = state["moving_mask"].copy()
    if state.get("kerb_mask") is not None:
        moving &= ~state["kerb_mask"]
    traversals = []
    for c in _valid_lap_instances([c for c in corners if c.get("stable_corner_id") is not None
                                   and c.get("bracket_start_m") is not None], laps_by_number):
        lap = laps_by_number[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                     c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            traversals.append({"cid": c["stable_corner_id"], "lap": c["lap_number"],
                               "speed_class": c.get("speed_class"), "lo": sl.start, "hi": sl.stop,
                               "bracket": (c["bracket_start_m"], c["bracket_end_m"])})
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
    idx_all = np.unique(np.concatenate([np.arange(tr["lo"], tr["hi"]) for tr in traversals])) if traversals else np.array([], int)
    idx_all = idx_all[(idx_all >= min_window) & moving[idx_all]]
    t1 = time.perf_counter()
    term = {}
    for axle, a_key, c_key in (("f", "alpha_f_filt", "CS_ratio_f"), ("r", "alpha_r_filt", "CS_ratio_r")):
        term[axle] = _window_termination(slip[a_key], cs[c_key], idx_all, min_window,
                                         se["cs_min_slip_angle_span_rad"], s_m, se["cs_max_window_m"])
    timings["termination"] = time.perf_counter() - t1

    # data-only decision-frame census (no sheet, no feedback, no driver)
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    from modules.decision_frame import (
        aggregate_ls_by_corner, load_decision_frame_config, build_evidence, generate_candidates,
        generate_display_split, resolve_conflicts, group_display_rows, apply_display_top_n,
    )
    from modules.recommendation import load_setup_parameters_registry, _group_by_corner
    df_cfg = load_decision_frame_config()
    registry = load_setup_parameters_registry()
    sm = summaries["A_baseline"]
    evidence = build_evidence(sm, aggregate_ls_by_corner(sm), df_cfg, classify_fn,
                              corners=corners, state=state, channels=channels)
    cands = generate_candidates(evidence, registry, df_cfg, setup_data=None,
                                assessed_corner_ids=set(_group_by_corner(sm).keys()), driving_level=None)
    split = generate_display_split(cands, evidence, None, df_cfg, registry)
    shortlist = split["shortlist"]
    resolve_conflicts(shortlist)
    grouped = group_display_rows(shortlist, registry)
    visible, _tail, _note = apply_display_top_n(grouped, [], df_cfg)

    def _slim(c):
        members = c.get("group_members") or [c]
        return {"id": c.get("id"), "score": c.get("score"), "actions": c.get("actions"),
                "effect_class": c.get("effect_class"), "scenario": c.get("scenario"),
                "corners": sorted({m.get("corner") for m in members if m.get("corner") is not None}),
                "phases": sorted({str(m.get("phase")) for m in members}),
                "evidence": [{"type": e.get("type"), "corner": e.get("corner"), "phase": e.get("phase"),
                              "verdict": e.get("verdict"), "source": e.get("source"),
                              "confidence": e.get("confidence")} for m in members for e in m.get("evidence_refs", [])]}

    lap_len = {}
    ld = channels.get("lap_distance")
    for l in data.get("laps", []):
        if ld is not None and ld.get("time") is not None:
            m = (ld["time"] >= l["start_time"]) & (ld["time"] <= l["end_time"])
            lap_len[l["lap_number"]] = float(np.nanmax(ld["data"][m])) if m.any() else float("nan")

    result = {
        "key": key, "path": path, "size_bytes": os.path.getsize(path),
        "unit_lap_distance": (ld or {}).get("unit_raw"),
        "native_rates": native, "grid": grid, "grid_err": grid_err,
        "sample_rate_hz": state["sample_rate_hz"], "min_window": min_window,
        "gate_verdict": str(gate_verdict), "fallback_used": fb_used, "fallback_reason": fb_reason,
        "laps": [{k: l.get(k) for k in ("lap_number", "start_time", "end_time", "lap_time",
                                         "is_valid_for_analysis", "is_outlap", "is_inlap")} for l in data.get("laps", [])],
        "lap_len_raw": lap_len,
        "corners": [{k: c.get(k) for k in ("lap_number", "corner_number", "stable_corner_id", "speed_class",
                                            "bracket_start_m", "bracket_end_m", "apex_lap_distance_m",
                                            "apex_speed", "warnings")} for c in corners],
        "traversals": traversals,
        "time": t, "s_m": s_m.astype(np.float64), "moving": moving,
        "alpha_f": slip["alpha_f_filt"].astype(np.float32), "alpha_r": slip["alpha_r_filt"].astype(np.float32),
        "cs": cs_arrays, "summaries": summaries, "term": term, "term_idx": idx_all,
        "verdict_fn_version": "outing_form._classify_corner",
        "display_visible": [_slim(c) for c in visible],
        "display_all": [_slim(c) for c in grouped],
        "n_candidates": len(cands),
        "timings": timings,
    }
    os.makedirs(CACHE_DIR, exist_ok=True)
    with gzip.open(os.path.join(CACHE_DIR, f"{key}.pkl.gz"), "wb") as fh:
        pickle.dump(result, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"[{key}] cached; timings " + ", ".join(f"{k}={v:.0f}s" for k, v in timings.items()), flush=True)


# ---------------------------------------------------------------- metrics

def leading_call(csf, csr):
    call = np.zeros(csf.shape, dtype=np.int8)  # +1 = F (front-led), -1 = R, 0 = none
    ok = np.isfinite(csf) & np.isfinite(csr)
    call[ok & (csf < csr)] = 1
    call[ok & (csr < csf)] = -1
    return call


def persist_call(call, s, length_m):
    """D read-out rule on one traversal (see header)."""
    out = np.zeros_like(call)
    reported = 0
    pend, pend_s0 = 0, None
    for k in range(call.size):
        c = call[k]
        if c == 0:
            pend, pend_s0 = 0, None
        elif reported == 0:
            reported = c  # first call of the traversal is taken as it comes
        elif c == reported:
            pend, pend_s0 = 0, None
        else:
            if pend != c:
                pend, pend_s0 = c, s[k]
            if np.isfinite(s[k]) and s[k] - pend_s0 >= length_m:
                reported, pend, pend_s0 = c, 0, None
        out[k] = reported if c != 0 else 0
    return out


def flapping(res, csf, csr, persistence_m=None):
    s_m = res["s_m"]
    moving = res["moving"]
    per_corner = {}
    runs_all = []
    total_trans, total_m = 0, 0.0
    for tr in res["traversals"]:
        sl = slice(tr["lo"], tr["hi"])
        call = leading_call(csf[sl], csr[sl])
        call[~moving[sl]] = 0
        s = s_m[sl]
        if persistence_m is not None:
            call = persist_call(call, s, persistence_m)
        keep = call != 0
        c, sk = call[keep], s[keep]
        if c.size < 2:
            continue
        trans = np.where(c[1:] != c[:-1])[0]
        length = float(np.nanmax(sk) - np.nanmin(sk))
        # run lengths between consecutive transitions (interior runs only; the
        # first/last run is truncated by the bracket and says nothing)
        bounds = sk[trans + 1]
        runs = np.diff(bounds) if bounds.size >= 2 else np.array([])
        runs_all.extend(runs.tolist())
        d = per_corner.setdefault(tr["cid"], {"trans": 0, "m": 0.0, "runs": [], "n_trav": 0})
        d["trans"] += int(trans.size); d["m"] += length; d["runs"].extend(runs.tolist()); d["n_trav"] += 1
        total_trans += int(trans.size); total_m += length
    return {"per_100m": 100.0 * total_trans / total_m if total_m > 0 else float("nan"),
            "total_trans": total_trans, "total_m": total_m, "runs": np.array(runs_all), "per_corner": per_corner}


def _classify(summary):
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    return classify_fn(summary)


def _axle_of(short):
    if "understeer" in short:
        return 1
    if "oversteer" in short:
        return -1
    return 0


def phase_and_lap_flapping(summaries):
    phase_trans, phase_pairs = 0, 0
    lap_calls = {}
    for s in summaries:
        calls = []
        for ph in PHASES:
            if ph == "apex_3" and s.get("apex_region"):
                f, r = s["apex_region"]["cs_ratio_f"]["median"], s["apex_region"]["cs_ratio_r"]["median"]
            else:
                f, r = s["phases"][ph]["cs_ratio_f"]["median"], s["phases"][ph]["cs_ratio_r"]["median"]
            if f == f and r == r and f != r:
                calls.append(1 if f < r else -1)
        if len(calls) >= 2:
            phase_pairs += len(calls) - 1
            phase_trans += sum(1 for a, b in zip(calls, calls[1:]) if a != b)
        if s.get("stable_corner_id") is not None:
            lap_calls.setdefault(s["stable_corner_id"], []).append((s["lap_number"], _axle_of(_classify(s)[1])))
    lap_trans, lap_pairs = 0, 0
    for cid, lst in lap_calls.items():
        seq = [a for _, a in sorted(lst) if a != 0]
        lap_pairs += max(0, len(seq) - 1)
        lap_trans += sum(1 for a, b in zip(seq, seq[1:]) if a != b)
    return {"phase_trans": phase_trans, "phase_pairs": phase_pairs,
            "lap_trans": lap_trans, "lap_pairs": lap_pairs}


def worst_lap(summaries):
    """Per stable corner: production aggregate (worst_lap) + verdict, plus
    per-lap worst-phase values for the negative census."""
    from modules.recommendation import aggregate_by_corner
    agg = aggregate_by_corner(summaries)
    out = {}
    for cid, a in agg.items():
        wf, wr = [], []
        for ph in PHASES:
            src = a["apex_region"] if ph == "apex_3" else a["phases"][ph]
            wf.append(src["cs_ratio_f"]["median"]); wr.append(src["cs_ratio_r"]["median"])
        wf = [v for v in wf if v == v]; wr = [v for v in wr if v == v]
        sev, short, _l, _c = _classify(a)
        out[cid] = {"f": min(wf) if wf else float("nan"), "r": min(wr) if wr else float("nan"),
                    "severity": sev, "short": short, "speed_class": a.get("speed_class"), "n_laps": a["n_laps"]}
    per_lap_neg = {}
    for s in summaries:
        cid = s.get("stable_corner_id")
        if cid is None:
            continue
        vals_f, vals_r = [], []
        for ph in PHASES:
            src = s["apex_region"] if (ph == "apex_3" and s.get("apex_region")) else s["phases"][ph]
            vals_f.append(src["cs_ratio_f"]["median"]); vals_r.append(src["cs_ratio_r"]["median"])
        d = per_lap_neg.setdefault(cid, [0, 0])
        d[0] += int(np.nanmin(vals_f) < 0) if any(v == v for v in vals_f) else 0
        d[1] += int(np.nanmin(vals_r) < 0) if any(v == v for v in vals_r) else 0
    return out, per_lap_neg


def load_all():
    res = {}
    for key in SESSIONS:
        p = os.path.join(CACHE_DIR, f"{key}.pkl.gz")
        if os.path.exists(p):
            with gzip.open(p, "rb") as fh:
                res[key] = pickle.load(fh)
    return res


def _pct(a, q):
    a = np.asarray([v for v in a if v == v], dtype=float)
    return float(np.percentile(a, q)) if a.size else float("nan")


def _f(x, nd=3):
    return "nan" if x is None or x != x else f"{x:.{nd}f}"


# ---------------------------------------------------------------- report

def report():
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_step2_chair_plots import _valid_lap_instances  # noqa: F401 (same env as compute)
    params = load_parameters()
    cls = params["classification"]
    se = params["stability_estimation"]
    S_F, S_R = cls["STRONG_CSF"]["value"], cls["STRONG_CSR"]["value"]
    max_w = se["cs_max_window_m"]
    res = load_all()
    print(f"sessions cached: {list(res)}")

    # ---------------- PART 1
    print("\n" + "=" * 70 + "\nPART 1 -- new-file checklist\n" + "=" * 70)
    for key, r in res.items():
        print(f"\n## {key}  ({r['path']}, {r['size_bytes'] / 1e9:.2f} GB)")
        print(f"  grid: {r['grid']}  err={r['grid_err']}  sample_rate_hz={r['sample_rate_hz']:.2f} min_window={r['min_window']}")
        print(f"  beta: gate={r['gate_verdict']} fallback={r['fallback_used']} {r['fallback_reason'] or ''}")
        chain = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z", "lap_distance"]
        print("  CS-chain native rates: " + ", ".join(f"{c}={_f(r['native_rates'].get(c, (float('nan'),))[0], 1)}Hz/{r['native_rates'].get(c, (None, 'absent'))[1]}" for c in chain))
        bad = [f"{n}={_f(v[0], 1)}/{v[1]}" for n, v in sorted(r["native_rates"].items()) if v[1] != "valid"]
        print(f"  whitelisted channels not 'valid': {bad}")
        laps = r["laps"]
        valid = [l["lap_number"] for l in laps if l["is_valid_for_analysis"]]
        print(f"  laps: {len(laps)} total, analysis laps {valid}; out={[l['lap_number'] for l in laps if l['is_outlap']]} in={[l['lap_number'] for l in laps if l['is_inlap']]}")
        lens = [r["lap_len_raw"].get(n) for n in valid]
        print(f"  lap_distance max per analysis lap (unit {r['unit_lap_distance']}): {[_f(x, 0) for x in lens]}")
        cor = [c for c in r["corners"] if c["lap_number"] in valid]
        per_lap = {n: sum(1 for c in cor if c["lap_number"] == n) for n in valid}
        by_id = {}
        for c in cor:
            by_id.setdefault(c["stable_corner_id"], []).append(c)
        full = sum(1 for m in by_id.values() if len({x['lap_number'] for x in m}) == len(valid))
        single = sum(1 for m in by_id.values() if len(m) == 1)
        print(f"  corners per analysis lap: {per_lap}; stable ids: {len(by_id)} (full={full}, partial={len(by_id) - full - single}, singleton={single})")
        warn = sum(1 for c in cor if c.get("warnings"))
        print(f"  corner instances with warnings: {warn}/{len(cor)}")
        # bracket gaps between consecutive stable corners (canonical brackets)
        br = sorted({(c["stable_corner_id"], c["bracket_start_m"], c["bracket_end_m"]) for c in cor if c["bracket_start_m"] is not None}, key=lambda x: x[1])
        gaps = [(br[k][0], br[k + 1][0], br[k + 1][1] - br[k][2]) for k in range(len(br) - 1)]
        short = [g for g in gaps if g[2] < max_w]
        print(f"  bracket gaps < cs_max_window_m ({max_w} m): {len(short)}/{len(gaps)}; "
              f"gaps m: median={_f(_pct([g[2] for g in gaps], 50), 1)} min={_f(min([g[2] for g in gaps]) if gaps else float('nan'), 1)}")
        # live bridging: CS windows whose start lies before the own bracket start
        # and inside the previous corner's bracket
        for axle in ("f", "r"):
            tm = r["term"][axle]
            idx = r["term_idx"]
            s_start = r["s_m"][tm["start"]]
            s_i = r["s_m"][idx]
            bridged, total = 0, 0
            prev_of = {br[k + 1][0]: br[k] for k in range(len(br) - 1)}
            for tr in r["traversals"]:
                sel = (idx >= tr["lo"]) & (idx < tr["hi"]) & tm["cs_finite"]
                if not sel.any():
                    continue
                total += int(sel.sum())
                pv = prev_of.get(tr["cid"])
                if pv is None:
                    continue
                ss = s_start[sel]
                bridged += int(np.sum((ss < tr["bracket"][0]) & (ss <= pv[2]) & (ss >= pv[1])))
            kinds = {k: float(np.mean(tm["kind"] == k)) for k in ("floor", "span", "cap")}
            fin = tm["cs_finite"]
            print(f"  axle {axle}: finite-CS corner samples {total}; windows reaching into previous corner's bracket: "
                  f"{bridged} ({_f(100 * bridged / total if total else float('nan'), 1)} %); "
                  f"footprint median {_f(np.nanmedian(tm['footprint_m'][fin]), 1)} m, p90 {_f(_pct(tm['footprint_m'][fin], 90), 1)} m")
        p = os.path.join(OUT_DIR, f"lever_channels_{key}.pkl")
        if os.path.exists(p):
            with open(p, "rb") as fh:
                lc = pickle.load(fh)
            print("  lever channel candidates (present, n_distinct, changes, range):")
            for name in LEVER_CHANNEL_CANDIDATES:
                if name in lc:
                    x = lc[name]
                    print(f"    {name}: {x['rate_hz']:.1f} Hz, distinct={x['n_distinct']}, changes={x['n_changes']}, [{x['min']}, {x['max']}] {x['values_head']}")

    # ---------------- PART 2
    print("\n" + "=" * 70 + "\nPART 2 -- extremity and flapping baselines (production A)\n" + "=" * 70)
    base_cache = {}
    for key, r in res.items():
        sm = r["summaries"]["A_baseline"]
        wl, neg = worst_lap(sm)
        base_cache[key] = (wl, neg)
        print(f"\n## {key}: {len(wl)} stable corners")
        for axle, thr in (("f", S_F), ("r", S_R)):
            vals = [v[axle] for v in wl.values()]
            beyond = sum(1 for v in vals if v == v and v < thr)
            nneg = sum(1 for v in vals if v == v and v < 0)
            print(f"  worst-lap CS_{axle}: p5={_f(_pct(vals, 5))} p10={_f(_pct(vals, 10))} p25={_f(_pct(vals, 25))} "
                  f"median={_f(_pct(vals, 50))} p75={_f(_pct(vals, 75))} min={_f(np.nanmin(vals) if vals else float('nan'))}; "
                  f"negative corners {nneg}/{len(vals)}; beyond STRONG ({thr}) {beyond}/{len(vals)} = {_f(100 * beyond / max(len(vals), 1), 0)} %")
        # Lap-count control: worst_lap is a minimum over laps, so its tail
        # deepens with the number of laps alone. Same statistic on every
        # window of 3 consecutive analysis laps holds N fixed.
        an_laps = sorted({s["lap_number"] for s in sm if s.get("stable_corner_id") is not None})
        if len(an_laps) >= 4:
            shares = {"f": [], "r": []}
            meds = {"f": [], "r": []}
            for k in range(len(an_laps) - 2):
                sub = [s for s in sm if s["lap_number"] in an_laps[k:k + 3]]
                wl3, _ = worst_lap(sub)
                for axle, thr in (("f", S_F), ("r", S_R)):
                    v = [x[axle] for x in wl3.values() if x[axle] == x[axle]]
                    shares[axle].append(100 * sum(1 for x in v if x < thr) / max(len(v), 1))
                    meds[axle].append(_pct(v, 50))
            print(f"  lap-count control ({len(an_laps)} laps -> {len(an_laps) - 2} windows of 3): beyond STRONG f "
                  f"{_f(np.mean(shares['f']), 0)} % (range {_f(min(shares['f']), 0)}-{_f(max(shares['f']), 0)}), "
                  f"r {_f(np.mean(shares['r']), 0)} % (range {_f(min(shares['r']), 0)}-{_f(max(shares['r']), 0)}); "
                  f"median worst-lap f {_f(np.mean(meds['f']))} r {_f(np.mean(meds['r']))}")
        negs = {cid: v for cid, v in sorted(neg.items()) if v[0] or v[1]}
        print(f"  negative laps per corner (front,rear) where any: {negs}")
        # mechanism numbers
        tf = r["term"]["f"]; tr_ = r["term"]["r"]
        for axle, tm in (("f", tf), ("r", tr_)):
            fin = tm["cs_finite"]
            print(f"  window termination {axle} (all corner samples, n={tm['kind'].size}): floor={_f(np.mean(tm['kind'] == 'floor'))} "
                  f"span={_f(np.mean(tm['kind'] == 'span'))} cap={_f(np.mean(tm['kind'] == 'cap'))}; among finite CS: "
                  f"floor={_f(np.mean(tm['kind'][fin] == 'floor'))}; n_median={_f(np.median(tm['n'][fin]) if fin.any() else float('nan'), 0)}")
        t = r["time"]
        for axle in ("f", "r"):
            alpha = r[f"alpha_{axle}"].astype(float)
            da = np.abs(np.gradient(alpha, t))
            for sc in ("low", "medium", "high"):
                ii = [np.arange(x["lo"], x["hi"]) for x in r["traversals"] if x["speed_class"] == sc]
                if not ii:
                    continue
                ii = np.concatenate(ii)
                print(f"  |dalpha_{axle}/dt| {sc:<6}: median {_f(np.degrees(np.median(da[ii])), 2)} deg/s p90 {_f(np.degrees(_pct(da[ii], 90)), 2)} (n={ii.size})")
            in_c = np.zeros(t.size, bool)
            for x in r["traversals"]:
                in_c[x["lo"]:x["hi"]] = True
            # same definition as inspect_v3_sawtooth_mechanism.alpha_character_for_corner
            mask = r["moving"] & ~in_c & (np.abs(alpha) < 0.01)
            j = np.diff(alpha[mask])
            print(f"  straight-line alpha_{axle} jitter std: {_f(np.degrees(np.std(j)), 4)} deg/sample (n={mask.sum()})")
        # verdict + candidate census
        from collections import Counter
        vc = Counter((v["severity"], ("US" if "understeer" in v["short"] else "OS" if "oversteer" in v["short"] else "-"),
                      "unstable" in v["short"]) for v in wl.values())
        print(f"  verdicts (severity, axle, unstable-yaw): {dict(vc)}")
        print(f"  candidates: {r['n_candidates']}; visible top rows:")
        for rank, c in enumerate(r["display_visible"], 1):
            act = "; ".join(f"{a.get('parameter')} {a.get('direction') or a.get('target') or a.get('delta')}" for a in (c["actions"] or []))
            ev = Counter((e["type"], e.get("verdict")) for e in c["evidence"])
            print(f"    #{rank} {act} score={_f(c['score'])} class={c['effect_class']} corners={c['corners']} phases={c['phases']} evidence={dict(ev)}")
        top_tc = [i for i, c in enumerate(r["display_all"], 1) if any((a.get("parameter") or "").startswith("tc_") for a in (c["actions"] or []))]
        print(f"  ranks of TC rows among all proposed display rows: {top_tc} (of {len(r['display_all'])})")
        fl = flapping(r, r["cs"]["A_baseline"][0].astype(float), r["cs"]["A_baseline"][1].astype(float))
        fp = np.concatenate([r["term"][a]["footprint_m"][r["term"][a]["cs_finite"]] for a in ("f", "r")])
        res_m = float(np.nanmedian(fp))
        runs = fl["runs"]
        pl = phase_and_lap_flapping(sm)
        print(f"  FLAPPING: {_f(fl['per_100m'], 2)} transitions/100 m ({fl['total_trans']} over {_f(fl['total_m'] / 1000, 1)} km); "
              f"interior runs n={runs.size} median {_f(np.median(runs) if runs.size else float('nan'), 1)} m, "
              f"share < {res_m:.1f} m (own median window footprint) = {_f(np.mean(runs < res_m) if runs.size else float('nan'), 2)}, "
              f"share < 5 m = {_f(np.mean(runs < 5) if runs.size else float('nan'), 2)}")
        print(f"  phase-level: {pl['phase_trans']}/{pl['phase_pairs']} adjacent phase pairs switch axle; "
              f"lap-level verdict axle: {pl['lap_trans']}/{pl['lap_pairs']} consecutive-lap pairs switch")
        worst = sorted(fl["per_corner"].items(), key=lambda kv: -kv[1]["trans"] / max(kv[1]["m"], 1))[:5]
        print("  per-corner transitions/100 m (top 5): " + ", ".join(
            f"C{cid}={_f(100 * d['trans'] / max(d['m'], 1), 2)} (runs med {_f(np.median(d['runs']) if d['runs'] else float('nan'), 1)} m)" for cid, d in worst))
        if key.startswith("nbr"):
            print("  NBR per corner-phase: OS-side at entry/turn-in, US-side at apex (per-lap phase verdicts, k of N laps):")
            by = {}
            for s in sm:
                cid = s.get("stable_corner_id")
                if cid is None:
                    continue
                d = by.setdefault(cid, {"n": 0, "os_e1": 0, "os_e2": 0, "us_apex": 0})
                d["n"] += 1
                for ph, k, want in (("entry_1_brake", "os_e1", -1), ("entry_2_turnin", "os_e2", -1), ("apex_3", "us_apex", 1)):
                    arg = {"phases": {ph: s["phases"][ph]}}
                    if ph == "apex_3" and s.get("apex_region"):
                        arg["apex_region"] = s["apex_region"]
                    if _axle_of(_classify(arg)[1]) == want:
                        d[k] += 1
            tot = {k: sum(d[k] for d in by.values()) for k in ("os_e1", "os_e2", "us_apex", "n")}
            both = sum(1 for d in by.values() if (d["os_e1"] + d["os_e2"]) > 0 and d["us_apex"] > 0)
            print(f"    totals over {tot['n']} corner-laps: OS@entry_1 {tot['os_e1']}, OS@turn-in {tot['os_e2']}, US@apex {tot['us_apex']}; corners with both patterns: {both}/{len(by)}")
            rep = sorted(by.items(), key=lambda kv: -(kv[1]['os_e1'] + kv[1]['os_e2'] + kv[1]['us_apex']))
            print("    " + "; ".join(f"C{cid}: e1 {d['os_e1']}/{d['n']} e2 {d['os_e2']}/{d['n']} apexUS {d['us_apex']}/{d['n']}" for cid, d in rep[:25]))

    # ---------------- PART 3
    print("\n" + "=" * 70 + "\nPART 3 -- sweep (10 rows x all sessions)\n" + "=" * 70)
    table = {}
    for row in ROW_ORDER:
        if row.startswith("D_"):
            cs_key, pers, sm_key = "A_baseline", D_LENGTHS[row], "A_baseline"
        elif row == "B_1.0Hz+D_25m":
            cs_key, pers, sm_key = "B_1.0Hz", 25.0, "B_1.0Hz"
        else:
            cs_key, pers, sm_key = row, None, row
        table[row] = {}
        for key, r in res.items():
            if cs_key not in r["cs"]:
                continue
            csf, csr = (a.astype(float) for a in r["cs"][cs_key])
            fl = flapping(r, csf, csr, persistence_m=pers)
            sm = r["summaries"][sm_key]
            wl, neg = worst_lap(sm)
            idx = r["term_idx"]
            cov = float(np.mean(np.isfinite(csf[idx]) & np.isfinite(csr[idx]))) if idx.size else float("nan")
            vals_f = [v["f"] for v in wl.values()]
            from collections import Counter
            vc = Counter(("US" if "understeer" in v["short"] else "OS" if "oversteer" in v["short"] else "ok") for v in wl.values())
            sev = Counter(v["severity"] for v in wl.values())
            entry = {"flap": fl["per_100m"], "cov": cov, "neg_corners_f": sum(1 for v in vals_f if v == v and v < 0),
                     "neg_corners_r": sum(1 for v in wl.values() if v["r"] == v["r"] and v["r"] < 0),
                     "p10_f": _pct(vals_f, 10), "p10_r": _pct([v["r"] for v in wl.values()], 10),
                     "verdicts": dict(vc), "severity": dict(sev), "wl": wl,
                     "pl": phase_and_lap_flapping(sm)}
            if key == "dubai":
                c4, c3 = wl.get(4), wl.get(3)
                ls_neg = 0
                for s in sm:
                    if s.get("stable_corner_id") == 3:
                        v = s["phases"]["exit_5"].get("ls_ratio_r", {}).get("median", float("nan"))
                        ls_neg += int(v == v and v < 0)
                entry.update({"C4_f": c4["f"], "C4_r": c4["r"], "C3_f": c3["f"], "C3_r": c3["r"], "C3_ls_neg_laps": ls_neg,
                              "C4_short": c4["short"], "C3_short": c3["short"]})
                entry["hard_ok"] = (c4["f"] < S_F) and (ls_neg == 4)
                entry["c3_cs_ok"] = c3["f"] < S_F
                entry["agree"] = (vc.get("US", 0) >= vc.get("OS", 0), sev.get("strong", 0) == 0, not c4["short"].startswith("ok"))
            table[row][key] = entry

    base = table["A_baseline"]
    for row in ROW_ORDER:
        d = table[row].get("dubai", {})
        dq = "" if d.get("hard_ok", True) else "  ** DISQUALIFIED (hard bar) **"
        print(f"\n## {row}{dq}")
        if d:
            print(f"  Dubai C4 worst CS_f {_f(d['C4_f'])} (A {_f(base['dubai']['C4_f'])}), C4 CS_r {_f(d['C4_r'])}; "
                  f"C3 LS_r exit_5 negative laps {d['C3_ls_neg_laps']}/4; C3 CS_f {_f(d['C3_f'])} (A {_f(base['dubai']['C3_f'])}, "
                  f"CS reading {'kept' if d['c3_cs_ok'] else 'LOST'}), C3 CS_r {_f(d['C3_r'])}")
            print(f"  Dubai driver agreement (US>=OS, no strong, C4 flagged): {d['agree']}; C4 '{d['C4_short']}'")
        for key, e in table[row].items():
            b = base.get(key)
            shift = {k: e["verdicts"].get(k, 0) - b["verdicts"].get(k, 0) for k in ("US", "OS", "ok")} if b else {}
            print(f"  {key:<17} flap {_f(e['flap'], 2)}/100m ({_f(e['flap'] / b['flap'] if b and b['flap'] else float('nan'), 2)}x A) "
                  f"phase {e['pl']['phase_trans']}/{e['pl']['phase_pairs']} lap {e['pl']['lap_trans']}/{e['pl']['lap_pairs']}; "
                  f"coverage {_f(e['cov'], 3)}; neg corners f/r {e['neg_corners_f']}/{e['neg_corners_r']}, p10 f/r {_f(e['p10_f'])}/{_f(e['p10_r'])}; "
                  f"verdicts {e['verdicts']} shift {shift}; severity {e['severity']}")

    # flapping-vs-fidelity figure: one ink, direct labels, open marker = disqualified
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6), constrained_layout=True)
    ink, muted = "#1f2937", "#6b7280"
    for ax, which in zip(axes, ("sample", "lap")):
        points = {}
        for row in ROW_ORDER:
            d = table[row].get("dubai")
            if d is None:
                continue
            if which == "sample":
                ks = [k for k in table[row] if base.get(k, {}).get("flap")]
                x = float(np.mean([table[row][k]["flap"] / base[k]["flap"] for k in ks]))
            else:
                # verdict-level: share of consecutive-lap pairs whose verdict axle switches, pooled
                tr = sum(table[row][k]["pl"]["lap_trans"] for k in table[row])
                pr = sum(table[row][k]["pl"]["lap_pairs"] for k in table[row])
                x = tr / pr if pr else float("nan")
            # rows identical by construction (D, E vs A on verdict level) share one point and label
            key = (round(x, 3), round(d["C4_f"], 3), d["hard_ok"])
            points.setdefault(key, []).append(row + ("" if d["hard_ok"] else " (DQ)"))
        for (x, y, filled), _rows in points.items():
            ax.plot([x], [y], "o", ms=9, mfc=ink if filled else "white", mec=ink, mew=2)
        xs = [k[0] for k in points]
        lo, hi = min(xs), max(xs)
        pad = 0.08 * (hi - lo)
        ax.set_xlim(lo - pad, hi + pad)
        # kept rows cluster on one C4 value: labels stacked in the empty band
        # above, each tied to its point by a leader line
        kept = sorted((k for k in points if k[2]), key=lambda k: k[0])
        for j, k in enumerate(kept):
            ax.annotate(", ".join(points[k]), (k[0], k[1]), xytext=(lo + (hi - lo) * 0.25, 0.02 + 0.09 * j),
                        textcoords="data", fontsize=8, color=ink,
                        arrowprops={"arrowstyle": "-", "color": muted, "lw": 0.8})
        for k in points:
            if not k[2]:
                ax.annotate(", ".join(points[k]), (k[0], k[1]), xytext=(8, -3), textcoords="offset points",
                            fontsize=8, color=ink)
        ax.axhline(S_F, color=muted, lw=1, ls="--")
        ax.text(ax.get_xlim()[0], S_F + 0.02, f" STRONG_CSF {S_F}", va="bottom", fontsize=8, color=muted)
        ax.set_xlabel("sample-level flips per 100 m, mean ratio to A over 9 sessions" if which == "sample"
                      else "lap-to-lap verdict-axle switch share, pooled over sessions", color=ink)
        ax.set_ylabel("Dubai C4 worst-lap CS_ratio_f (below dashed = kept)", color=ink)
        ax.grid(color="#e5e7eb", lw=0.8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.suptitle("CS sweep: flapping vs genuine-event fidelity (open marker = hard bar failed)", color=ink)
    os.makedirs(OUT_DIR, exist_ok=True)
    fig_path = os.path.join(OUT_DIR, "flapping_vs_fidelity.png")
    fig.savefig(fig_path, dpi=130)
    print(f"\nfigure: {fig_path}")
    with open(os.path.join(OUT_DIR, "sweep_table.pkl"), "wb") as fh:
        pickle.dump(table, fh)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "channels":
        channel_content_census(sys.argv[2])
    elif mode == "compute":
        compute(sys.argv[2])
    elif mode == "report":
        report()
