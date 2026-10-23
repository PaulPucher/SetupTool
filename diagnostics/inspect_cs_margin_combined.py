# Combined CS boundary margin (work order 4a, author EXECUTE 2026-10-22).
# Ruling D2 makes "axle limiting" = CS below zero; the boundary needs a margin
# that covers estimator uncertainty. Sensitivity analysis (Tier B), report
# only, no config change -- the margin ruling is the review instance's.
#
# Two parts per corner-phase and axle, on the classifier's own population
# (valid analysis laps, config cs_cross_lap_aggregation via
# aggregate_by_corner, apex_3 from apex_region as _verdict_cs_stat reads it):
#   obs  observer noise: max(|CS(Q x 0.3) - CS|, |CS(R_yaw x 10) - CS|), the
#        two admissible perturbations of the 2026-10-21 robustness check
#        (inspect_observer_robustness.py, script at tag pre-prune-2026-10-22;
#        wrappers restored below unchanged)
#   veh  vehicle inputs: the +/-1 % sweep behind cs_margin
#        (inspect_cs_margin_l.py, results_cs_margin_20261021b), read from its
#        per-run JSON. Two readings are reported because the work order does
#        not fix how the ten runs combine: veh_max (largest single run) and
#        veh_rss (root-sum-square over the five inputs, each input's larger
#        side) -- which one is canonical is an OPEN science item.
#   combined = sqrt(obs^2 + veh^2)
# Known mismatch, stated: the 20261021b JSON aggregate all laps (lap_filter
# None, out- / in-lap included) and store the raw apex_3 slice, not
# apex_region; veh therefore comes from a slightly different population than
# obs. The baseline here is re-run fresh at HEAD and its all-lap aggregate is
# checked against the 20261021b baseline values (identity check).
# Per-lap observer spread (same deltas per lap, not aggregated) is reported
# as information only.
# Worst-lap verdict per axle and corner-phase: limiting if CS < 0 versus
# CS < -margin, margin = per-axle pooled p90 of combined.
# Output diagnostics/results_cs_margin_combined_20261022/ (per-run pickles,
# analysis.txt / analysis.json).
# Run: python -m diagnostics.inspect_cs_margin_combined [report]
# Env CS_COMBINED_OUT overrides the output folder (the run executes from a
# clean HEAD snapshot so concurrent edits cannot reach the workers).

import copy
import json
import os
import pickle
import sys
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from diagnostics.inspect_cs_margin_l import CANDIDATES, POOL, INPUTS, keep_awake  # noqa: E402

OUT = os.environ.get("CS_COMBINED_OUT") or os.path.join(ROOT, "diagnostics", "results_cs_margin_combined_20261022")
VEH_DIR = os.path.join(ROOT, "diagnostics", "results_cs_margin_20261021b")
VARIANTS = ("baseline", "q_x0.3", "ryaw_x10")
WORKERS = 3
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


def _install(variant, tfa):
    # wrappers of inspect_observer_robustness (tag pre-prune-2026-10-22),
    # reduced to the two admissible variants
    if variant == "q_x0.3":
        orig = tfa._q_upper_from_residual

        def q_variant(*a, **kw):
            q = dict(orig(*a, **kw))
            q["q_beta_var"] *= 0.3
            q["q_yaw_rate_var"] *= 0.3
            q["variant"] = variant
            return q
        tfa._q_upper_from_residual = q_variant
    elif variant == "ryaw_x10":
        orig = tfa._measurement_noise_floors

        def floors_variant(*a, **kw):
            fl = copy.deepcopy(orig(*a, **kw))
            fl["yaw_rate"]["var"] *= 10.0
            return fl
        tfa._measurement_noise_floors = floors_variant


def _verdict_value(summary, phase, ax):
    # the statistic the classifier reads (modules.classifier.verdict_cs_stat)
    ar = summary.get("apex_region")
    if phase == "apex_3" and ar is not None:
        return ar[f"cs_ratio_{ax}"]["median"]
    return summary["phases"][phase][f"cs_ratio_{ax}"]["median"]


def run_task(job):
    key, variant = job
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, load_car_data, prepare_vehicle_state,
                                            estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            estimate_vertical_loads, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.corner_aggregation import aggregate_by_corner
    t0 = time.perf_counter()
    _, _track, _outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    params = copy.deepcopy(load_parameters())
    p = apply_resolved_vehicle(params, resolve_accuracy(params, setup_data=None, cap=None))
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    # pool workers are reused across jobs: wrapped functions are restored
    saved = {n: getattr(tfa, n) for n in ("_q_upper_from_residual", "_measurement_noise_floors")}
    try:
        _install(variant, tfa)
        st = prepare_vehicle_state(data["channels"], p)
        beta, man, gate, fb, _fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        for n, f in saved.items():
            setattr(tfa, n, f)
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    fz = estimate_vertical_loads(st, forces, p, channels=data["channels"], car_data=load_car_data())
    summaries = summarise_corners(data.get("corners", []), cs, stab, st, fz=fz, lap_filter=None)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    vsum = [s for s in summaries if s["lap_number"] in valid]
    agg_valid = aggregate_by_corner(vsum)
    agg_all = aggregate_by_corner(summaries)
    out = {
        "key": key, "label": label, "variant": variant, "status": man and man.get("status"),
        "gate": gate and gate.get("verdict"), "fallback": fb, "valid_laps": sorted(valid),
        # classifier population: valid laps, configured aggregation, apex_region at apex_3
        "agg": {f"{cid}|{ph}": {ax: _verdict_value(c, ph, ax) for ax in "fr"}
                for cid, c in agg_valid.items() for ph in PHASES},
        # identity check against 20261021b (all laps, raw phase slice)
        "agg_all_raw": {f"{cid}|{ph}": {ax: c["phases"][ph][f"cs_ratio_{ax}"]["median"] for ax in "fr"}
                        for cid, c in agg_all.items() for ph in c["phases"]},
        "per_lap": {f"{s['lap_number']}|{s['stable_corner_id']}|{ph}": {ax: _verdict_value(s, ph, ax) for ax in "fr"}
                    for s in vsum if s.get("stable_corner_id") is not None for ph in PHASES},
        "elapsed_s": time.perf_counter() - t0,
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{key}__{variant}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{key} {variant}: {out['elapsed_s']:.0f} s, status {out['status']}, gate {out['gate']}, fallback {fb}"


def _fin(x):
    return x is not None and x == x


def _pct(v):
    v = np.asarray([x for x in v if _fin(x)], float)
    if not v.size:
        return {"n": 0, "p50": float("nan"), "p90": float("nan"), "p95": float("nan")}
    return {"n": int(v.size), "p50": float(np.percentile(v, 50)), "p90": float(np.percentile(v, 90)),
            "p95": float(np.percentile(v, 95))}


def _veh_deltas(key):
    # per corner|phase and axle: veh_max over the ten runs, veh_rss over the
    # five inputs (each input's larger side)
    base = json.load(open(os.path.join(VEH_DIR, f"{key}_baseline_base.json"), encoding="utf-8"))
    runs = {}
    for inp in INPUTS:
        for tag in ("plus1", "minus1"):
            runs[(inp, tag)] = json.load(open(os.path.join(VEH_DIR, f"{key}_{inp}_{tag}.json"), encoding="utf-8"))
    out = {}
    for cp, bv in base["cs_values"].items():
        row = {}
        for ax in "fr":
            b = bv.get(f"cs_ratio_{ax}")
            if not _fin(b):
                continue
            per_inp = []
            for inp in INPUTS:
                d = [abs(runs[(inp, t)]["cs_values"].get(cp, {}).get(f"cs_ratio_{ax}") - b)
                     for t in ("plus1", "minus1")
                     if _fin(runs[(inp, t)]["cs_values"].get(cp, {}).get(f"cs_ratio_{ax}"))]
                if d:
                    per_inp.append(max(d))
            if per_inp:
                row[ax] = {"max": max(per_inp), "rss": float(np.sqrt(np.sum(np.square(per_inp))))}
        out[cp] = row
    return base, out


def _load(key, variant):
    fn = os.path.join(OUT, f"{key}__{variant}.pkl")
    return pickle.load(open(fn, "rb")) if os.path.exists(fn) else None


def report():
    L = ["COMBINED CS BOUNDARY MARGIN (work order 4a, 2026-10-22), 14 development sessions",
         "obs = max(|dCS| Q x 0.3, |dCS| R_yaw x 10); veh = +/-1 % vehicle-input sweep (results_cs_margin_20261021b);",
         "combined = sqrt(obs^2 + veh^2); population: valid laps, worst_lap aggregation, apex_3 from apex_region", ""]
    from modules.stability_analysis import load_parameters
    agg_mode = load_parameters()["classification"].get("cs_cross_lap_aggregation", "median")
    L.append(f"classifier aggregation (config cs_cross_lap_aggregation): {agg_mode}; 20261021b used the same "
             f"aggregate_by_corner (same config value at that run) but on ALL laps (lap_filter None) and the raw "
             f"apex_3 slice -- population differs from the classifier's (valid laps, apex_region)")
    L.append("")
    rows = []           # pooled corner-phase records
    lap_obs = {"f": [], "r": []}
    match = []
    per_session = {}
    for key in POOL:
        b, q, ry = _load(key, "baseline"), _load(key, "q_x0.3"), _load(key, "ryaw_x10")
        if b is None or q is None or ry is None:
            L.append(f"{key}: run missing")
            continue
        vbase, veh = _veh_deltas(key)
        # identity check: fresh all-lap aggregate vs the 20261021b baseline
        n_cmp, n_diff, max_abs = 0, 0, 0.0
        for cp, bv in vbase["cs_values"].items():
            for ax in "fr":
                x, y = bv.get(f"cs_ratio_{ax}"), b["agg_all_raw"].get(cp, {}).get(ax)
                if not _fin(x) and not _fin(y):
                    continue
                n_cmp += 1
                if not (_fin(x) and _fin(y)) or x != y:
                    n_diff += 1
                    if _fin(x) and _fin(y):
                        max_abs = max(max_abs, abs(x - y))
        match.append((key, n_cmp, n_diff, max_abs))
        for cp, bv in b["agg"].items():
            for ax in "fr":
                c0 = bv[ax]
                if not _fin(c0):
                    continue
                d = [abs(v["agg"][cp][ax] - c0) for v in (q, ry) if _fin(v["agg"].get(cp, {}).get(ax))]
                obs = max(d) if d else float("nan")
                vd = veh.get(cp, {}).get(ax)
                rows.append({"session": key, "cp": cp, "axle": ax, "cs": c0, "obs": obs,
                             "veh_max": vd["max"] if vd else float("nan"),
                             "veh_rss": vd["rss"] if vd else float("nan")})
        for lk, bv in b["per_lap"].items():
            for ax in "fr":
                c0 = bv[ax]
                if not _fin(c0):
                    continue
                d = [abs(v["per_lap"][lk][ax] - c0) for v in (q, ry)
                     if lk in v["per_lap"] and _fin(v["per_lap"][lk][ax])]
                if d:
                    lap_obs[ax].append((key, max(d)))
        per_session[key] = b["label"]
    for r in rows:
        for vk in ("max", "rss"):
            v = r[f"veh_{vk}"]
            r[f"comb_{vk}"] = float(np.hypot(r["obs"], v)) if _fin(r["obs"]) and _fin(v) else float("nan")

    L.append("BASELINE IDENTITY (fresh HEAD run, all-lap raw aggregate vs 20261021b baseline JSON):")
    for key, n, nd, mx in match:
        L.append(f"  {key}: {n} axle values compared, {nd} differ (max |diff| {mx:.2e})")
    L.append("")
    res = {"aggregation": agg_mode, "identity": match, "axles": {}, "sessions": {}}
    AX = {"f": "front", "r": "rear"}
    for ax in "fr":
        R = [r for r in rows if r["axle"] == ax]
        n_obs = sum(_fin(r["obs"]) for r in R)
        n_veh = sum(_fin(r["veh_max"]) for r in R)
        L.append(f"{AX[ax].upper()} AXLE, pooled: {len(R)} corner-phases with a baseline CS; obs defined {n_obs}, "
                 f"veh defined {n_veh}")
        axr = {}
        for name in ("obs", "veh_max", "veh_rss", "comb_max", "comb_rss"):
            s = _pct([r[name] for r in R])
            axr[name] = s
            L.append(f"  {name:8s} n {s['n']:4d}  p50 {s['p50']:.4f}  p90 {s['p90']:.4f}  p95 {s['p95']:.4f}")
        lo = _pct([d for _k, d in lap_obs[ax]])
        axr["obs_per_lap"] = lo
        L.append(f"  obs per lap (information only) n {lo['n']}  p50 {lo['p50']:.4f}  p90 {lo['p90']:.4f}  "
                 f"p95 {lo['p95']:.4f}")
        for vk in ("max", "rss"):
            m = axr[f"comb_{vk}"]["p90"]
            cs = [r["cs"] for r in R]
            within = sum(abs(c) < m for c in cs)
            lim0 = sum(c < 0 for c in cs)
            limm = sum(c < -m for c in cs)
            axr[f"margin_{vk}"] = {"margin": m, "n": len(cs), "within": within, "lim_cs_lt_0": lim0,
                                   "lim_cs_lt_minus_margin": limm, "changed": lim0 - limm}
            L.append(f"  margin (comb_{vk} p90) {m:.4f}: |CS| < margin {within}/{len(cs)} "
                     f"({100 * within / len(cs):.1f} %); limiting CS<0 {lim0}, CS<-margin {limm}, "
                     f"verdicts changing {lim0 - limm}")
        res["axles"][AX[ax]] = axr
        L.append("")
    L.append("PER SESSION (p50 / p90 / p95; margin = pooled per-axle p90 of comb_max):")
    for key in POOL:
        if key not in per_session:
            continue
        L.append(f"  {per_session[key]}")
        sres = {}
        for ax in "fr":
            R = [r for r in rows if r["axle"] == ax and r["session"] == key]
            m = res["axles"][AX[ax]]["margin_max"]["margin"]
            parts = []
            sr = {}
            for name in ("obs", "veh_max", "comb_max"):
                s = _pct([r[name] for r in R])
                sr[name] = s
                parts.append(f"{name} {s['p50']:.3f}/{s['p90']:.3f}/{s['p95']:.3f}")
            cs = [r["cs"] for r in R]
            within = sum(abs(c) < m for c in cs)
            chg = sum(-m <= c < 0 for c in cs)
            sr.update({"n": len(cs), "within": within, "changed": chg})
            sres[AX[ax]] = sr
            L.append(f"    {AX[ax]:5s} n {len(cs):3d}  " + "  ".join(parts)
                     + f"  within {within} ({100 * within / len(cs) if cs else float('nan'):.0f} %), "
                       f"verdicts changing {chg}")
        res["sessions"][key] = sres
    text = "\n".join(L)
    with open(os.path.join(OUT, "analysis.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "analysis.json"), "w", encoding="utf-8") as fh:
        json.dump({"summary": res, "rows": rows}, fh, indent=1, default=str)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    tasks = [(k, v) for k in POOL for v in VARIANTS
             if not os.path.exists(os.path.join(OUT, f"{k}__{v}.pkl"))]
    keep_awake(True)
    try:
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            futs = {ex.submit(run_task, t): t for t in tasks}
            for i, fut in enumerate(as_completed(futs), 1):
                try:
                    print(f"[{i}/{len(tasks)}] {fut.result()}", flush=True)
                except Exception as e:
                    print(f"[{i}/{len(tasks)}] {futs[fut]} FAILED: {e!r}", flush=True)
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
