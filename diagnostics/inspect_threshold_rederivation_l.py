# Threshold re-derivation on the new default (fit L, schema 16), PROPOSAL
# ONLY (author EXECUTE 2026-10-20, final chain part 1): nothing in config/ or
# modules/ changes; the author rules. Also the floor-event census (CS ratio
# below the -0.53 pure-lateral floor, events not samples).
#
# PROCEDURE, unchanged from inspect_threshold_rederivation.py modes
# compute_fresh / counts_fresh (2026-10-11): per session the production chain
# as the app runs it -- here resolve_sideslip_beta_and_cs_inputs at the
# configured source (fit L by default), CS with the inputs it hands over,
# stability, summarise_corners, aggregate_by_corner over the analysis laps
# (worst lap); corner value = worst of the five phases, apex_3 from
# apex_region (_corner_values, imported unchanged). STRONG = p10, MODERATE =
# p30 of the pool's corner values per axle (np.percentile, linear). Best
# available accuracy (cap None), speed classes reclassified as in the app.
# POOL: the 14 development sessions (Nurburgring + Le Castellet); Dubai
# excluded. Every session must run on EKF beta (no fallback) -- reported.
# SENSITIVITY: the same rule without outings 59 and 70 (g2 front at the edge).
# Beside it: the share of pool corner values below the current and proposed
# thresholds, the limit-event curve floors of the fit-L curves (C = 1), and
# where the proposed STRONG sits relative to -0.53.
# FLOOR EVENTS (modules.limit_events.detect_floor_events, the app's output):
# per session and axle the event count with kerb- / observer-reset-affected
# counts and attribute summaries; per analysis lap the count of clean events
# (neither flag) for the plausibility check, and the corner / phase cells
# holding clean events on >= half the laps.
# Output diagnostics/plots/threshold_rederivation/ (cache_l_20261020/,
# rederivation_l_20261020.txt / .json).
# Run: python -m diagnostics.inspect_threshold_rederivation_l [compute | report]

import copy
import json
import math
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "threshold_rederivation")
# run tag (env THRESH_RUN_TAG): 20261020 = the first fit-L derivation,
# 20261020b = after the R26 source fix; each run keeps its own cache and report
RUN_TAG = os.environ.get("THRESH_RUN_TAG", "20261020")
CACHE = os.path.join(OUT_DIR, f"cache_l_{RUN_TAG}")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
EDGE_SESSIONS = ("nbr_o59_mla", "nbr_qp_mla")   # outings 59 and 70
WORKERS = 3
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


def compute(key):
    from diagnostics.inspect_data_quality import CANDIDATES
    from diagnostics.inspect_threshold_rederivation import _corner_values
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.recommendation import aggregate_by_corner
    from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs
    from modules.limit_events import detect_floor_events, _event_line
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    source = p["stability_estimation"]["sideslip_source"]
    beta, man, gate, fb, fr, cs_in = resolve_sideslip_beta_and_cs_inputs(st, p, data, source, csv_path=path)
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    laps = data.get("laps", [])
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    summ = summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
    corners = _corner_values(aggregate_by_corner([x for x in summ if x["lap_number"] in valid]))
    fe = detect_floor_events(cs, st, data.get("corners", []), laps, p, slip,
                             axle_loads=cs_in["cs_kwargs"].get("axle_loads"), axle_fx=cs_in["axle_fx"])
    line = _event_line(man, gate)
    out = {"key": key, "label": label, "mode": p["tyre_fit_auto"]["tyre_fit_mode"], "source": source,
           "status": man and man.get("status"), "reason": man and (man.get("degenerate_reason")
                                                                     or man.get("marginal_reason")),
           "fallback": fb, "fallback_reason": fr, "gate": gate and gate.get("verdict"),
           "cs_note": cs_in["note"], "analysis_laps": sorted(valid), "corners": corners,
           "floor_events": fe, "limit_event_line": line,
           "n_instances": {str(c["stable_corner_id"]): sum(1 for d in data["corners"]
                                                          if d.get("stable_corner_id") == c["stable_corner_id"]
                                                          and d["lap_number"] in valid)
                           for c in data["corners"] if c.get("stable_corner_id") is not None}}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] {out['mode']}, status {out['status']}, fallback {fb}, gate {out['gate']}; "
          f"{len(corners)} corners, {len(fe['events'])} floor events, {time.perf_counter() - t0:.0f} s", flush=True)
    return key


def _vals(res, keys, ax):
    x = np.array([v[ax] for k in keys for v in res[k]["corners"].values()], float)
    return x[np.isfinite(x)]


def _pct_below(x, thr):
    return float(100.0 * np.mean(x < thr)) if x.size else float("nan")


def report():
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_data_quality import CANDIDATES
    res = {}
    for k in POOL:
        with open(os.path.join(CACHE, f"{k}.pkl"), "rb") as fh:
            res[k] = pickle.load(fh)
    cl = load_parameters()["classification"]
    cur = {"f": (cl["STRONG_CSF"]["value"], cl["MODERATE_CSF"]["value"]),
           "r": (cl["STRONG_CSR"]["value"], cl["MODERATE_CSR"]["value"])}
    L = ["THRESHOLD RE-DERIVATION ON THE NEW DEFAULT (fit L, schema 16) -- PROPOSAL ONLY, nothing adopted",
         "Rule unchanged (2026-10-11 compute_fresh / counts_fresh): STRONG = p10, MODERATE = p30 of the pool's corner "
         "values per axle; corner value = worst lap, worst of five phases, apex_3 from apex_region.", "",
         "POOL COMPOSITION (14 development sessions, Dubai excluded):"]
    J = {"pool": {}, "proposal": {}, "sensitivity_no_59_70": {}, "floor_events": {}}
    for k in POOL:
        r = res[k]
        nf, nr = _vals(res, [k], "f").size, _vals(res, [k], "r").size
        L.append(f"  {r['label']}: mode {r['mode']}, status {r['status']}"
                 + (f" ({r['reason']})" if r["reason"] else "")
                 + f", beta {'KINEMATIC FALLBACK: ' + str(r['fallback_reason']) if r['fallback'] else 'EKF'}"
                 + f", gate {r['gate']}, {len(r['analysis_laps'])} analysis laps, corner values front {nf} / rear {nr}"
                 + (f", CS note: {r['cs_note']}" if r["cs_note"] else ""))
        J["pool"][k] = {"label": r["label"], "status": r["status"], "fallback": r["fallback"], "gate": r["gate"],
                        "n_laps": len(r["analysis_laps"]), "n_f": int(nf), "n_r": int(nr)}
    n_ekf = sum(not res[k]["fallback"] for k in POOL)
    L.append(f"  EKF beta on {n_ekf} of {len(POOL)} sessions")

    no_edge = [k for k in POOL if k not in EDGE_SESSIONS]
    for ax, name, s_key, m_key in (("f", "FRONT", "STRONG_CSF", "MODERATE_CSF"),
                                   ("r", "REAR", "STRONG_CSR", "MODERATE_CSR")):
        v = _vals(res, POOL, ax)
        v2 = _vals(res, no_edge, ax)
        S, M = float(np.percentile(v, 10)), float(np.percentile(v, 30))
        S2, M2 = float(np.percentile(v2, 10)), float(np.percentile(v2, 30))
        L += ["", f"{name} (pool n {v.size}):",
              f"  current  {s_key} {cur[ax][0]:+.3f}, {m_key} {cur[ax][1]:+.3f}  (encode p{_pct_below(v, cur[ax][0]):.1f} "
              f"/ p{_pct_below(v, cur[ax][1]):.1f} of the new pool)",
              f"  proposed {s_key} {S:+.3f}, {m_key} {M:+.3f}  (p10 / p30)",
              f"  sensitivity without outings 59 / 70 (n {v2.size}): {S2:+.3f} / {M2:+.3f}",
              f"  pool p5 / p50 / p90 / min: {np.percentile(v, 5):+.3f} / {np.median(v):+.3f} / "
              f"{np.percentile(v, 90):+.3f} / {v.min():+.3f}; values below -0.53: {int(np.sum(v < -0.53))} "
              f"({_pct_below(v, -0.53):.1f} %)",
              "  per session strong / moderate of n, current | proposed:"]
        J["proposal"][name.lower()] = {"current": cur[ax], "proposed": (S, M), "n": int(v.size),
                                       "current_percentile": (_pct_below(v, cur[ax][0]), _pct_below(v, cur[ax][1])),
                                       "below_floor_053": int(np.sum(v < -0.53))}
        J["sensitivity_no_59_70"][name.lower()] = {"proposed": (S2, M2), "n": int(v2.size)}
        for k in POOL:
            x = _vals(res, [k], ax)
            c_ = (int(np.sum(x < cur[ax][0])), int(np.sum((x >= cur[ax][0]) & (x < cur[ax][1]))))
            p_ = (int(np.sum(x < S)), int(np.sum((x >= S) & (x < M))))
            L.append(f"    {res[k]['label']}: {c_[0]}/{c_[1]} | {p_[0]}/{p_[1]} of {x.size}")

    L += ["", "LIMIT-EVENT LINE of the fit-L curves (min over 0..30 deg of (dFy/dalpha) / (B C D), C = 1):"]
    for k in POOL:
        line = res[k]["limit_event_line"]
        L.append(f"  {res[k]['label']}: " + (line if isinstance(line, str) else
                                               f"front {line['f']:+.4f}, rear {line['r']:+.4f}"))

    L += ["", "FLOOR EVENTS (CS ratio < -0.53, contiguous runs inside corner instances on analysis laps; "
              "clean = neither kerb- nor observer-reset-affected):"]
    tot = {"f": [0, 0], "r": [0, 0]}
    for k in POOL:
        r = res[k]
        evs = r["floor_events"]["events"]
        J["floor_events"][k] = {}
        L.append(f"  {r['label']} ({len(r['analysis_laps'])} analysis laps):")
        for ax, axn in (("f", "front"), ("r", "rear")):
            e_ax = [e for e in evs if e["axle"] == axn]
            clean = [e for e in e_ax if not e["kerb_affected"] and not e["observer_reset_affected"]]
            tot[ax][0] += len(e_ax)
            tot[ax][1] += len(clean)
            rec = {"n": len(e_ax), "n_clean": len(clean),
                   "n_kerb": sum(e["kerb_affected"] for e in e_ax),
                   "n_reset": sum(e["observer_reset_affected"] for e in e_ax)}
            line = (f"    {axn}: {rec['n']} events, {rec['n_clean']} clean, {rec['n_kerb']} kerb-affected, "
                    f"{rec['n_reset']} reset-affected")
            if e_ax:
                def med(f):
                    a = np.array([e[f] for e in e_ax if e[f] is not None], float)
                    return float(np.median(a)) if a.size else float("nan")
                ph = {}
                for e in e_ax:
                    ph[e["phase"]] = ph.get(e["phase"], 0) + 1
                kd = np.array([e["kerb_distance_m"] for e in e_ax if e["kerb_distance_m"] is not None], float)
                rec.update({"duration_s_median": med("duration_s"),
                            "duration_s_max": float(max(e["duration_s"] for e in e_ax)),
                            "n_samples_median": med("n_samples"), "cs_min_median": med("cs_min"),
                            "cs_min_min": float(min(e["cs_min"] for e in e_ax)),
                            "fx_util_max_median": med("fx_util_max"),
                            "braking_events": sum(bool(e["braking"]) for e in e_ax),
                            "fz_window_range_median": med("fz_window_range_median"),
                            "kerb_distance_m_median": float(np.median(kd)) if kd.size else float("nan"),
                            "phases": ph})
                line += (f"; duration median {rec['duration_s_median']:.2f} s (max {rec['duration_s_max']:.2f}), "
                         f"samples median {rec['n_samples_median']:.0f}; min ratio median {rec['cs_min_median']:+.2f} "
                         f"(deepest {rec['cs_min_min']:+.2f}); max |Fx|/Fz median {rec['fx_util_max_median']:.2f}; "
                         f"braking throughout {rec['braking_events']} of {rec['n']}; Fz range in windows median "
                         f"{100 * rec['fz_window_range_median']:.0f} %; kerb distance median "
                         f"{rec['kerb_distance_m_median']:.0f} m; phases "
                         + ", ".join(f"{p_} {n_}" for p_, n_ in sorted(ph.items(), key=lambda x: PHASES.index(x[0]))))
            L.append(line)
            # plausibility: clean events per analysis lap, repeatable cells
            per_lap = {ln: sum(1 for e in clean if e["lap"] == ln) for ln in r["analysis_laps"]}
            cells = {}
            for e in clean:
                cells.setdefault((e["corner"], e["phase"]), set()).add(e["lap"])
            half = math.ceil(len(r["analysis_laps"]) / 2)
            rep_cells = {f"C{c} {ph}": sorted(l) for (c, ph), l in cells.items()
                         if len(l) >= max(2, half)}
            one = sum(1 for l in cells.values() if len(l) == 1)
            rec.update({"clean_per_lap": per_lap, "repeatable_cells": rep_cells, "one_lap_cells": one})
            if clean:
                L.append("      clean events per lap: " + ", ".join(f"{ln} {n}" for ln, n in per_lap.items())
                         + f"; cells on >= half the laps: {rep_cells or 'none'}; cells on one lap: {one}")
            J["floor_events"][k][axn] = rec
    L.append(f"  POOL TOTAL front {tot['f'][0]} events ({tot['f'][1]} clean), rear {tot['r'][0]} ({tot['r'][1]} clean)")
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, f"rederivation_l_{RUN_TAG}.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, f"rederivation_l_{RUN_TAG}.json"), "w") as fh:
        json.dump(J, fh, indent=1, default=lambda o: list(o) if isinstance(o, (set, tuple)) else str(o))
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    from diagnostics.inspect_fit_lit import keep_awake
    keep_awake(True)
    try:
        todo = [k for k in POOL if not os.path.exists(os.path.join(CACHE, f"{k}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, todo):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
