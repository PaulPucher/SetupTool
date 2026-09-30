# DIAGNOSTIC, read-only, [keep-reproduces]. PLAN.md EXPERIMENT PLAN (b): CS
# filtering at UPSTREAM insertion points -- what the sideslip estimator and
# the slip-angle construction see -- where the 2026-09-29 sweep filtered only
# the regression inputs (alpha, Fy). Nothing in modules/ or config/ changes;
# every variant is built in memory. Tier B (standard zero-phase low-pass,
# the chain's own filtfilt Butterworth, BUTTERWORTH_ORDER).
#
# Own script, not a mode of inspect_new_laps_cs_sweep: that compute() is one
# monolithic per-session pass, while these variants must feed modified RAW
# channels through the production fit (fit_session_pacejka builds its own
# state from data["channels"], so filtering the prepared state would leave
# the estimator untouched). Reused: the sweep's session mapping, window
# termination and flapping helpers; the read-out, phase map, battery and
# jackknife machinery of inspect_cs_readout_evidence / _decision_table; the
# value-lens roughness helpers.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_cs_upstream_sweep compute <key>
#   python -m diagnostics.inspect_cs_upstream_sweep score
#   python -m diagnostics.inspect_cs_upstream_sweep ecompute <key> <variant>  (E, only if earned)
# Outputs in diagnostics/plots/cs_upstream/ (gitignored).
#
# PRE-REGISTERED (fixed before the first run):
# VARIANTS (raw channels filtered in a copy of the parsed data; corners,
# traversals, forces, LS and vertical loads stay the production ones):
#  A    baseline, production as-is.
#  B4, B2  sclu_yaw_rate and log_acc_y low-passed at 4 / 2 Hz before the
#       sideslip fit and EKF; alpha, Fy and stability keep the production
#       state (the added stage acts on what the estimator sees only).
#  C    ecu_speed low-passed at 2 Hz before the fit/EKF AND in the slip-angle
#       construction. Mechanism: v enters beta_dot = ay/v - r and alpha =
#       beta + l r / v - delta as a divisor, so speed noise is amplified most
#       at low speed; ecu_speed is inertia-limited, so 2 Hz removes noise,
#       not dynamics.
#  D2, D1  the production EKF beta low-passed at 2 / 1 Hz between estimator
#       and alpha (estimator untouched). Production alpha is itself filtered
#       at cs_filter_cutoff_hz downstream, so D2 is a near-cascade.
#  E    the best-scoring single variant + C, computed only if some single
#       variant scores above baseline (Part 2 rule (i) and (ii) below).
#  F1, F2  FLOOR REFERENCES, not candidates: ecu_speed x 1.01 and log_acc_y
#       x 1.02 through the whole chain (state, estimator, alpha, forces) --
#       the (a2) scale-error family (speed 0.5-1.2 %, roll leakage 1.5-2.6 %).
# PART 1 per variant and session: mean |dCS| per sample and median
# decorrelation length (value-lens _acf_length_m) inside traversals; share
# of corner samples whose CS window terminates at the floor; coverage =
# finite CS share of corner samples; EKF gate verdict / fallback.
# PART 2 scoring:
#  1. Survival battery as recorded in the decision table (PROD read-out):
#     C3 LS_r exit_5 negative on 4/4 laps; worst-phase r - f > 0 at Dubai C3,
#     US-v2 C13, NBR B C9; US >= OS corners on Dubai and on NBR A; NBR A->B
#     median front change > 0 and gap change < 0. An element that already
#     fails at baseline cannot disqualify; a variant is DISQUALIFIED if it
#     loses any element the baseline passes (kept in the table).
#  2. NBR A-vs-B resolvability, 2-sigma jackknife (recorded rule), front and
#     gap, against the baseline count.
#  3. Pooled lap-pair verdict switch rate (sweep phase_and_lap_flapping).
#  4. Verdict shift vs baseline per session (changed corner calls; US/OS/ok).
#  5. Floor test: per scoring metric (front resolved, gap resolved, pooled
#     switch rate) the floor = the larger |change| F1 or F2 produce vs
#     baseline; an improvement CLEARS the floor only if larger than it.
#     Effect size alongside: median |change| of per-lap phase medians vs
#     baseline, against the same for F1/F2.
# EXIT (PLAN (b)): a variant proposal only if a variant (i) loses no battery
# element, (ii) beats baseline on resolvability (front or gap) or on the
# pooled switch rate, and (iii) that improvement clears the floor.
# Otherwise (b) closes as tested-upstream-nothing-earned-production.
# TIERING (author amendment, applied mid-run 2026-09-30): Tier 1 = Dubai,
# NBR JST 29 (+ JST 21 for the A-vs-B leg) and US-v2 (the hostile file that
# carries the battery's Paul Ricard C13 gap; already computed); v3 was
# complete before the amendment and is kept. Tier 2 (Le Mans, CDR, MLA) runs
# only the variants that survive Tier 1 undisqualified AND clear the floor,
# with the baseline beside them.
# TIER 2 CONFIRMATION (fixed before the Tier-2 run): per session, baseline vs
# each survivor: lap-pair switch rate WITH its called-pair count, changed
# corner calls and US/OS/ok, coverage, median |phase-median change|. A
# survivor CONFIRMS on a session if its switch rate is below baseline; the
# called-pair count is reported beside it, because a lower rate over fewer
# called pairs is fewer verdicts, not steadier ones.

import copy
import gzip
import os
import pickle
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import (
    SESSIONS, _window_termination, phase_and_lap_flapping,
)

KEYS = [k for k in SESSIONS if k != "pr_us_v1"]
OUT_DIR = "diagnostics/plots/cs_upstream"
VARIANTS = ["A", "B4", "B2", "C", "D2", "D1", "F1", "F2"]
CANDIDATES = ["B4", "B2", "C", "D2", "D1"]
FLOORS = ["F1", "F2"]


def _lp_raw(ch, fc):
    from scipy.signal import butter, filtfilt
    from modules.stability_analysis import BUTTERWORTH_ORDER
    t = np.asarray(ch["time"], dtype=float)
    sr = 1.0 / float(np.median(np.diff(t)))
    b, a = butter(BUTTERWORTH_ORDER, fc / (0.5 * sr), btype="low")
    out = dict(ch)
    out["data"] = filtfilt(b, a, np.asarray(ch["data"], dtype=float))
    return out


def _scale_raw(ch, k):
    out = dict(ch)
    out["data"] = np.asarray(ch["data"], dtype=float) * k
    return out


def _data_with(data, repl):
    d = dict(data)
    d["channels"] = dict(data["channels"])
    d["channels"].update(repl)
    return d


def _lp_arr(x, fc, sr, moving):
    from scipy.signal import butter, filtfilt
    from modules.stability_analysis import BUTTERWORTH_ORDER
    b, a = butter(BUTTERWORTH_ORDER, fc / (0.5 * sr), btype="low")
    return np.where(moving, filtfilt(b, a, x), 0.0)


def _chain(key, params, data, state_prod, forces_prod, fz, ls, variant, base_beta=None):
    """beta -> alpha -> CS -> stability -> summaries for one variant."""
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
    )
    from modules.tyre_fit_auto import resolve_sideslip_beta
    live = params["stability_estimation"].get("sideslip_source", "kinematic")
    path = SESSIONS[key]
    ch = data["channels"]
    info = {}
    st_alpha, st_all = state_prod, state_prod
    forces = forces_prod
    if variant == "A" or variant.startswith("D"):
        if base_beta is None:
            beta, _m, gv, fb, fr = resolve_sideslip_beta(state_prod, params, data, live, csv_path=path)
            info.update(gate=str(gv.get("verdict") if isinstance(gv, dict) else gv), fallback=fb, reason=fr)
        else:
            beta = base_beta
        if variant.startswith("D"):
            beta = _lp_arr(beta, {"D2": 2.0, "D1": 1.0}[variant], state_prod["sample_rate_hz"], state_prod["moving_mask"])
    else:
        if variant in ("B4", "B2"):
            fc = {"B4": 4.0, "B2": 2.0}[variant]
            dv = _data_with(data, {"sclu_yaw_rate": _lp_raw(ch["sclu_yaw_rate"], fc),
                                   "log_acc_y": _lp_raw(ch["log_acc_y"], fc)})
        elif variant == "C":
            dv = _data_with(data, {"ecu_speed": _lp_raw(ch["ecu_speed"], 2.0)})
        elif variant == "F1":
            dv = _data_with(data, {"ecu_speed": _scale_raw(ch["ecu_speed"], 1.01)})
        elif variant == "F2":
            dv = _data_with(data, {"log_acc_y": _scale_raw(ch["log_acc_y"], 1.02)})
        elif variant.startswith("E:"):
            parts = variant[2:].split("+")
            repl = {}
            if "B4" in parts or "B2" in parts:
                fc = 4.0 if "B4" in parts else 2.0
                repl.update({"sclu_yaw_rate": _lp_raw(ch["sclu_yaw_rate"], fc), "log_acc_y": _lp_raw(ch["log_acc_y"], fc)})
            repl["ecu_speed"] = _lp_raw(ch["ecu_speed"], 2.0)
            dv = _data_with(data, repl)
        else:
            raise ValueError(variant)
        sv = prepare_vehicle_state(dv["channels"], params)
        assert sv["time"].size == state_prod["time"].size
        beta, _m, gv, fb, fr = resolve_sideslip_beta(sv, params, dv, live, csv_path=path)
        info.update(gate=str(gv.get("verdict") if isinstance(gv, dict) else gv), fallback=fb, reason=fr)
        if variant == "C" or variant.startswith("E:"):
            st_alpha = sv
        if variant.startswith("E:") and any(p.startswith("D") for p in variant[2:].split("+")):
            dname = [p for p in variant[2:].split("+") if p.startswith("D")][0]
            beta = _lp_arr(beta, {"D2": 2.0, "D1": 1.0}[dname], sv["sample_rate_hz"], sv["moving_mask"])
        if variant in ("F1", "F2"):
            st_alpha = st_all = sv
            forces = estimate_lateral_forces(sv, params)
    slip = estimate_slip_angles(st_alpha, beta, params)
    cs = estimate_cornering_stiffness(slip, forces, st_all, params)
    stab = estimate_yaw_moment_stability(st_all, beta, params, data.get("laps", []))
    summ = summarise_corners(data.get("corners", []), cs, stab, st_all, fz=fz, ls=ls)
    return beta, slip, cs, summ, info


def compute(key, only=None):
    import time
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        load_parameters, prepare_vehicle_state, load_car_data, estimate_lateral_forces,
        estimate_vertical_loads, resolve_cs_min_window_samples,
    )
    from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
    from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from diagnostics.inspect_step2_chair_plots import _valid_lap_instances, _canonical_window_slice
    t0 = time.perf_counter()
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))
    data = parse_csv(SESSIONS[key])
    ch = data["channels"]
    st = prepare_vehicle_state(ch, params)
    forces = estimate_lateral_forces(st, params)
    fz = estimate_vertical_loads(st, forces, params, channels=ch, car_data=load_car_data())
    ls = estimate_longitudinal_stiffness(estimate_longitudinal_forces(st, ch, params),
                                         estimate_slip_ratio(st, ch, params), st, params)
    se = params["stability_estimation"]
    t, s_m = st["time"], st["s_m"]
    moving = st["moving_mask"].copy()
    if st.get("kerb_mask") is not None:
        moving &= ~st["kerb_mask"]
    laps_by = {l["lap_number"]: l for l in data.get("laps", [])}
    corners = data.get("corners", [])
    trav = []
    for c in _valid_lap_instances([c for c in corners if c.get("stable_corner_id") is not None
                                   and c.get("bracket_start_m") is not None], laps_by):
        lap = laps_by[c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            trav.append({"cid": c["stable_corner_id"], "lap": c["lap_number"], "lo": sl.start, "hi": sl.stop})
    min_window = resolve_cs_min_window_samples(params, st["sample_rate_hz"])
    idx_all = np.unique(np.concatenate([np.arange(tr["lo"], tr["hi"]) for tr in trav])) if trav else np.array([], int)
    idx_all = idx_all[(idx_all >= min_window) & moving[idx_all]]
    path = os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz")
    if os.path.exists(path):
        with gzip.open(path, "rb") as fh:
            res = pickle.load(fh)
    else:
        res = {"key": key, "time": t, "s_m": s_m.astype(np.float64), "moving": moving,
               "laps": [{k: l.get(k) for k in ("lap_number", "start_time", "end_time", "is_valid_for_analysis")}
                        for l in data.get("laps", [])],
               "traversals": trav, "term_idx": idx_all, "variants": {},
               "corners": [{k: c.get(k) for k in ("lap_number", "stable_corner_id", "bracket_start_m", "bracket_end_m")}
                           for c in corners]}
    base_beta = None
    todo = only or VARIANTS
    for v in todo:
        tv = time.perf_counter()
        if v.startswith("D") and base_beta is None:
            base_beta = res["variants"].get("A", {}).get("beta")
            base_beta = base_beta.astype(float) if base_beta is not None else None
        beta, slip, cs, summ, info = _chain(key, params, data, st, forces, fz, ls, v, base_beta=base_beta)
        if v == "A":
            base_beta = beta
        term = {ax: _window_termination(slip[a], cs[c], idx_all, min_window, se["cs_min_slip_angle_span_rad"], s_m,
                                        se["cs_max_window_m"])["kind"]
                for ax, a, c in (("f", "alpha_f_filt", "CS_ratio_f"), ("r", "alpha_r_filt", "CS_ratio_r"))}
        res["variants"][v] = {"beta": beta.astype(np.float32),
                              "cs": (cs["CS_ratio_f"].astype(np.float32), cs["CS_ratio_r"].astype(np.float32)),
                              "summaries": summ, "term": term, "info": info}
        print(f"[{key}] {v}: {time.perf_counter() - tv:.0f}s, gate {info.get('gate')}, fallback {info.get('fallback')}", flush=True)
        os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
        with gzip.open(path, "wb") as fh:
            pickle.dump(res, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"[{key}] done {time.perf_counter() - t0:.0f}s", flush=True)


def load(key):
    with gzip.open(os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def _as_r(res, v):
    return {"time": res["time"], "s_m": res["s_m"], "moving": res["moving"], "laps": res["laps"],
            "traversals": res["traversals"], "corners": res["corners"],
            "cs": {"A_baseline": res["variants"][v]["cs"]}, "summaries": {"A_baseline": res["variants"][v]["summaries"]}}


def score():
    from collections import Counter
    from modules.stability_analysis import load_parameters
    from modules.recommendation import aggregate_by_corner
    from diagnostics.inspect_cs_readout_evidence import (
        phase_index_map, load_supp2, corner_readouts, _calls, _worst, _roughness, PHASES,
    )
    from diagnostics.inspect_cs_value_lens import _brackets, _valid_idx
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    allres = {k: load(k) for k in KEYS if os.path.exists(os.path.join(OUT_DIR, "cache", f"{k}.pkl.gz"))}
    variants = [v for v in VARIANTS + sorted({v for r in allres.values() for v in r["variants"] if v.startswith("E:")})
                if all(v in r["variants"] for r in allres.values())]
    pims = {k: phase_index_map(_as_r(r, "A"), load_supp2(k), se) for k, r in allres.items()}
    lapsets = {k: sorted({l for (_c, l) in pims[k]}) for k in allres}
    ro = {v: {k: corner_readouts(_as_r(allres[k], v), pims[k], lapsets[k], gate) for k in allres} for v in variants}

    # parity: baseline phase map reproduces the baseline summaries
    for k, r in allres.items():
        bad = 0
        for s in r["variants"]["A"]["summaries"]:
            m = pims[k].get((s["stable_corner_id"], s["lap_number"]))
            if m is None:
                continue
            for ph in PHASES:
                for ai, ax in ((0, "cs_ratio_f"), (1, "cs_ratio_r")):
                    prod = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                    v_ = r["variants"]["A"]["cs"][ai].astype(float)[m[ph]]
                    v_ = v_[np.isfinite(v_)]
                    mine = float(np.median(v_)) if v_.size >= gate else float("nan")
                    bad += int(not ((prod != prod and mine != mine) or abs(prod - mine) < 1e-5))
        print(f"parity {k}: {bad} baseline phase medians differ")

    print("\n=== PART 1 -- per variant and session: mean |dCS| f/r, median decorrelation length f/r (m), "
          "floor-termination share f/r, coverage f/r, EKF gate / fallback")
    p1 = {}
    for v in variants:
        p1[v] = {}
        for k, r in allres.items():
            rr = _as_r(r, v)
            idx_by = [_valid_idx(rr, tr) for tr in r["traversals"]]
            row = {}
            for ai, ax in ((0, "f"), (1, "r")):
                cs = r["variants"][v]["cs"][ai].astype(float)
                steps, acfs = _roughness(cs, rr, idx_by)
                term = r["variants"][v]["term"][ax]
                cov = float(np.mean(np.isfinite(cs[r["term_idx"]])))
                row[ax] = {"step": float(np.mean(steps)) if steps.size else float("nan"),
                           "acf_m": float(np.median(acfs)) if acfs.size else float("nan"),
                           "floor_share": float(np.mean(term == "floor")), "coverage": cov}
            row["info"] = r["variants"][v]["info"]
            p1[v][k] = row
        print(f"\n[{v}]")
        for k, row in p1[v].items():
            print(f"  {k:17s} step {row['f']['step']:.4f}/{row['r']['step']:.4f}, acf {row['f']['acf_m']:.1f}/"
                  f"{row['r']['acf_m']:.1f} m, floor {100 * row['f']['floor_share']:.1f}/{100 * row['r']['floor_share']:.1f} %, "
                  f"coverage {row['f']['coverage']:.3f}/{row['r']['coverage']:.3f}, gate {row['info'].get('gate')} "
                  f"fallback {row['info'].get('fallback')}")

    print("\n=== PART 2 -- scoring")
    sc = {}
    for v in variants:
        calls = {k: _calls(aggregate_by_corner(allres[k]["variants"][v]["summaries"])) for k in allres}
        b = {}
        r = allres["dubai"]["variants"][v]
        b["C3_LS"] = sum(1 for s in r["summaries"] if s["stable_corner_id"] == 3 and
                         (s["phases"]["exit_5"].get("ls_ratio_r", {}).get("median", float("nan")) or 0) < 0) == 4
        for key, cid in (("dubai", 3), ("pr_us_v2", 13), ("nbr_jst_solution", 9)):
            g = _worst(ro[v][key][cid], "r", "PROD") - _worst(ro[v][key][cid], "f", "PROD")
            b[f"gap_{key}_C{cid}"] = (g > 0, g)
        for key in ("dubai", "nbr_jst_problem"):
            c = Counter(x[0] for x in calls[key].values())
            b[f"US>=OS_{key}"] = (c.get("US", 0) >= c.get("OS", 0), c.get("US", 0), c.get("OS", 0))
        A, B = allres["nbr_jst_problem"], allres["nbr_jst_solution"]
        rA, rB = _as_r(A, v), _as_r(B, v)
        ba, bb = _brackets(rA), _brackets(rB)
        pairs = [(ca, cb) for ca, (a0, a1) in sorted(ba.items()) for cb, (b0, b1) in bb.items()
                 if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0)]
        roA, roB = ro[v]["nbr_jst_problem"], ro[v]["nbr_jst_solution"]
        dfr = [roB[cb]["apex_3"]["f"]["PROD"] - roA[ca]["apex_3"]["f"]["PROD"] for ca, cb in pairs]
        dgp = [(roB[cb]["apex_3"]["r"]["PROD"] - roB[cb]["apex_3"]["f"]["PROD"]) -
               (roA[ca]["apex_3"]["r"]["PROD"] - roA[ca]["apex_3"]["f"]["PROD"]) for ca, cb in pairs]
        dfr = [x for x in dfr if x == x]; dgp = [x for x in dgp if x == x]
        b["AB_direction"] = (np.median(dfr) > 0 and np.median(dgp) < 0, float(np.median(dfr)), float(np.median(dgp)))

        def jk(key, cid, what, rr):
            def stat_of(lset):
                o = corner_readouts(rr, {kk: vv for kk, vv in pims[key].items() if kk[0] == cid}, lset, gate)
                if cid not in o:
                    return float("nan")
                f, r_ = o[cid]["apex_3"]["f"]["PROD"], o[cid]["apex_3"]["r"]["PROD"]
                return f if what == "f" else r_ - f
            laps = lapsets[key]
            th = stat_of(laps)
            loo = np.array([stat_of([l for l in laps if l != x]) for x in laps])
            loo = loo[np.isfinite(loo)]
            n = loo.size
            return th, (float(np.sqrt((n - 1) / n * np.sum((loo - loo.mean()) ** 2))) if n >= 2 else float("nan"))
        res_ = {"f": [0, 0], "gap": [0, 0]}
        for ca, cb in pairs:
            for what in ("f", "gap"):
                ta, sa = jk("nbr_jst_problem", ca, what, rA)
                tb, sb = jk("nbr_jst_solution", cb, what, rB)
                if all(np.isfinite([ta, tb, sa, sb])):
                    res_[what][1] += 1
                    res_[what][0] += int(abs(tb - ta) > 2 * np.sqrt(sa ** 2 + sb ** 2))
        lt = lp = 0
        for k in allres:
            f = phase_and_lap_flapping(allres[k]["variants"][v]["summaries"])
            lt += f["lap_trans"]; lp += f["lap_pairs"]
        shift = {}
        for k in allres:
            c0 = _calls(aggregate_by_corner(allres[k]["variants"]["A"]["summaries"]))
            cv = calls[k]
            cnt = Counter(x[0] for x in cv.values())
            shift[k] = (sum(1 for c in cv if c in c0 and cv[c][0] != c0[c][0]), cnt.get("US", 0), cnt.get("OS", 0), cnt.get("-", 0))
        dmed = []
        for k in allres:
            s0 = {(s["stable_corner_id"], s["lap_number"]): s for s in allres[k]["variants"]["A"]["summaries"]}
            for s in allres[k]["variants"][v]["summaries"]:
                s_b = s0.get((s["stable_corner_id"], s["lap_number"]))
                if s_b is None:
                    continue
                for ph in PHASES:
                    for ax in ("cs_ratio_f", "cs_ratio_r"):
                        a_ = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                        b_ = (s_b["apex_region"] if ph == "apex_3" else s_b["phases"][ph])[ax]["median"]
                        if a_ == a_ and b_ == b_:
                            dmed.append(abs(a_ - b_))
        sc[v] = {"battery": b, "resolved": res_, "switch": (lt, lp, lt / lp if lp else float("nan")),
                 "shift": shift, "effect_median": float(np.median(dmed)) if dmed else float("nan"),
                 "effect_p90": float(np.percentile(dmed, 90)) if dmed else float("nan")}

    base = sc["A"]
    base_ok = {k: (v if isinstance(v, bool) else v[0]) for k, v in base["battery"].items()}
    floor = {"f": max(abs(sc[x]["resolved"]["f"][0] - base["resolved"]["f"][0]) for x in FLOORS),
             "gap": max(abs(sc[x]["resolved"]["gap"][0] - base["resolved"]["gap"][0]) for x in FLOORS),
             "switch": max(abs(sc[x]["switch"][2] - base["switch"][2]) for x in FLOORS),
             "effect": max(sc[x]["effect_median"] for x in FLOORS)}
    print(f"floor (max |change| of F1, F2 vs A): front resolved {floor['f']}, gap resolved {floor['gap']}, "
          f"switch rate {floor['switch']:.3f}, median |phase-median change| {floor['effect']:.4f}")
    exit_rows = {}
    for v in variants:
        s = sc[v]
        lost = [k for k, ok in s["battery"].items() if base_ok[k] and not (ok if isinstance(ok, bool) else ok[0])]
        gained = [k for k, ok in s["battery"].items() if not base_ok[k] and (ok if isinstance(ok, bool) else ok[0])]
        dq = bool(lost)
        imp = {"f": s["resolved"]["f"][0] - base["resolved"]["f"][0],
               "gap": s["resolved"]["gap"][0] - base["resolved"]["gap"][0],
               "switch": base["switch"][2] - s["switch"][2]}
        beats = imp["f"] > 0 or imp["gap"] > 0 or imp["switch"] > 0
        clears = (imp["f"] > floor["f"]) or (imp["gap"] > floor["gap"]) or (imp["switch"] > floor["switch"])
        exit_rows[v] = {"disqualified": dq, "lost": lost, "gained": gained, "improvement": imp,
                        "beats_baseline": beats, "clears_floor": clears,
                        "earns_proposal": (v in CANDIDATES or v.startswith("E:")) and not dq and beats and clears}
        bat = "; ".join(
            f"{k} {'PASS' if (ok if isinstance(ok, bool) else ok[0]) else 'FAIL'}" +
            ("" if isinstance(ok, bool) else " (" + ", ".join(f"{x:+.3f}" if isinstance(x, float) else str(x) for x in ok[1:]) + ")")
            for k, ok in s["battery"].items())
        print(f"\n[{v}]{' FLOOR REFERENCE' if v in FLOORS else ''}")
        print(f"  1 battery: {bat}")
        print(f"    lost vs baseline: {lost or 'none'}; gained: {gained or 'none'} -> {'DISQUALIFIED' if dq else 'survives'}")
        print(f"  2 A-vs-B resolved: front {s['resolved']['f'][0]}/{s['resolved']['f'][1]}, gap "
              f"{s['resolved']['gap'][0]}/{s['resolved']['gap'][1]}")
        print(f"  3 pooled lap-pair switch rate {s['switch'][0]}/{s['switch'][1]} = {s['switch'][2]:.3f}")
        print("  4 verdict shift vs A (changed / US / OS / ok): " + "; ".join(
            f"{k} {x[0]} ({x[1]}/{x[2]}/{x[3]})" for k, x in s["shift"].items()))
        print(f"  5 effect size median |phase-median change| {s['effect_median']:.4f} (p90 {s['effect_p90']:.4f}); "
              f"improvement front {imp['f']:+d}, gap {imp['gap']:+d}, switch {imp['switch']:+.3f}; beats baseline {beats}; "
              f"clears floor {clears}; effect above floor effect {s['effect_median'] > floor['effect']}")
    print("\n--- EXIT (pre-registered): proposal-earning variants: " +
          (", ".join(v for v, e in exit_rows.items() if e["earns_proposal"]) or "none"))
    best = [v for v in CANDIDATES if not exit_rows[v]["disqualified"] and exit_rows[v]["beats_baseline"]]
    print("--- E rule: single variants scoring above baseline (no loss, beats baseline): " + (", ".join(best) or "none"))
    with open(os.path.join(OUT_DIR, "score.pkl"), "wb") as fh:
        pickle.dump({"p1": p1, "sc": sc, "floor": floor, "exit": exit_rows}, fh)


TIER2_KEYS = ["lmn_mla", "nbr_qp_cdr", "nbr_qp_mla"]


def confirm(variants):
    from collections import Counter
    from modules.recommendation import aggregate_by_corner
    from diagnostics.inspect_cs_readout_evidence import _calls, PHASES
    for k in TIER2_KEYS:
        r = load(k)
        a = r["variants"]["A"]
        fa = phase_and_lap_flapping(a["summaries"])
        c0 = _calls(aggregate_by_corner(a["summaries"]))
        n0 = Counter(x[0] for x in c0.values())
        cov0 = [float(np.mean(np.isfinite(a["cs"][i].astype(float)[r["term_idx"]]))) for i in (0, 1)]
        print(f"\n[{k}] A: switch {fa['lap_trans']}/{fa['lap_pairs']} = "
              f"{fa['lap_trans'] / fa['lap_pairs'] if fa['lap_pairs'] else float('nan'):.3f}; "
              f"US/OS/ok {n0.get('US', 0)}/{n0.get('OS', 0)}/{n0.get('-', 0)}; coverage {cov0[0]:.3f}/{cov0[1]:.3f}")
        s0 = {(s["stable_corner_id"], s["lap_number"]): s for s in a["summaries"]}
        for v in variants:
            if v not in r["variants"]:
                print(f"  {v}: not computed")
                continue
            vv = r["variants"][v]
            f = phase_and_lap_flapping(vv["summaries"])
            cv = _calls(aggregate_by_corner(vv["summaries"]))
            nv = Counter(x[0] for x in cv.values())
            changed = sum(1 for c in cv if c in c0 and cv[c][0] != c0[c][0])
            cov = [float(np.mean(np.isfinite(vv["cs"][i].astype(float)[r["term_idx"]]))) for i in (0, 1)]
            d = []
            for s in vv["summaries"]:
                sb = s0.get((s["stable_corner_id"], s["lap_number"]))
                if sb is None:
                    continue
                for ph in PHASES:
                    for ax in ("cs_ratio_f", "cs_ratio_r"):
                        x = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                        y = (sb["apex_region"] if ph == "apex_3" else sb["phases"][ph])[ax]["median"]
                        if x == x and y == y:
                            d.append(abs(x - y))
            rate = f["lap_trans"] / f["lap_pairs"] if f["lap_pairs"] else float("nan")
            rate0 = fa["lap_trans"] / fa["lap_pairs"] if fa["lap_pairs"] else float("nan")
            print(f"  {v}: switch {f['lap_trans']}/{f['lap_pairs']} = {rate:.3f} "
                  f"({'CONFIRMS' if rate < rate0 else 'does not confirm'}); changed calls {changed}; "
                  f"US/OS/ok {nv.get('US', 0)}/{nv.get('OS', 0)}/{nv.get('-', 0)}; coverage {cov[0]:.3f}/{cov[1]:.3f}; "
                  f"median |phase-median change| {np.median(d) if d else float('nan'):.4f}")


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "compute":
        compute(sys.argv[2])
    elif mode == "ecompute":
        compute(sys.argv[2], only=[sys.argv[3]])
    elif mode == "tier2":
        compute(sys.argv[2], only=sys.argv[3].split(","))
    elif mode == "confirm":
        confirm(sys.argv[2].split(","))
    elif mode == "render":
        # retro-renders (2026-09-30 follow-up package): baseline vs D1 vs E on
        # the three screening sessions, via the standard trace-set helper
        from diagnostics.trace_render import render_cs_traces
        for k in ("nbr_jst_solution", "pr_us_v2", "nbr_qp_mla"):
            r = load(k)
            for v in ("A", "D1", "E:C+D1"):
                cf, cr = (a.astype(float) for a in r["variants"][v]["cs"])
                name = v.replace(":", "_").replace("+", "")
                out = os.path.join(OUT_DIR, f"retro_{k}_{name}.png")
                clipped = render_cs_traces(out, r["time"], r["s_m"], cf, cr, r["laps"], r["corners"], f"{k} {v}")
                print(f"{k} {v}: {out}; below y floor f {clipped['f']}, r {clipped['r']}")
    else:
        {"score": score}[mode]()
