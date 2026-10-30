# SENS-1 recount (overnight order 2026-10-29): one counting rule for every
# estimator perturbation, DECISIONS 'Sensitivity counting rules' (author
# ruling 2026-10-27). Sensitivity analysis, Tier B, report only; read-only on
# config and stored payloads, no module, threshold or margin changes.
#
# Unit: corner x phase x {front, rear, stability} of the per-session
# aggregate (aggregate_by_corner over the analysis laps, as the app), on the
# 14 development sessions. The classifier's own phase_symptoms decides: US
# front (moderate / strong), OS rear (strong), INST (dMz/dbeta < 0). A unit
# counts as changed when its symptom appears, disappears or changes severity
# against the baseline. Stability units are not axle units; they are counted
# as their own kind beside front and rear.
# Session recommendation: setup_engine.data_layer as the app calls it
# (tc_activity, logged switch positions), no setup sheet; compared as
# lever + direction of the recommendation, or the set of tied options.
#
# Perturbations, each against the shipped default (in memory, per job):
#   q_x0.3, ryaw_x10   observer Q and yaw-rate R, wrappers of
#                      inspect_cs_margin_combined (unchanged)
#   drag_cda_<x>       c_D A 0 / 0.7 / 1.3 m^2 (shipped 1.0), the variants of
#                      inspect_drag_sensitivity, via drag_coeff at fixed A
#   e_0, e_+1          fit-L curvature E (shipped -1), the E values of the
#                      2026-10-21 C / E grid (inspect_ce_sensitivity_l.py,
#                      script at tag pre-prune-2026-10-22), C left at 1
#   <input>_plus1/minus1  +/-1 % vehicle inputs, _variant_params and the
#                      steering-table scaling of inspect_cs_margin_l
# The CS boundary margins were derived from the Q, R_yaw and vehicle runs
# (p90, inspect_cs_margin_combined), so part of the flips under those
# perturbations is built into the boundary.
# Output diagnostics/results_sens1_recount_20261029/ (per-run pickles,
# analysis.txt / analysis.json).
# Run: python -m diagnostics.inspect_sens1_recount [report]

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

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from diagnostics.inspect_cs_margin_l import (CANDIDATES, POOL, INPUTS, PCT, keep_awake,  # noqa: E402
                                             _variant_params, _scale_wheel_angle_column)
from diagnostics.inspect_cs_margin_combined import _install  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_sens1_recount_20261029")
WORKERS = 6
DRAG_CDA = (0.0, 0.7, 1.3)   # m^2
E_VALUES = (0.0, 1.0)
VARIANTS = (("baseline", "q_x0.3", "ryaw_x10") + tuple(f"drag_cda_{x:.1f}" for x in DRAG_CDA)
            + tuple(f"e_{e:+.0f}" for e in E_VALUES)
            + tuple(f"{i}_{t}" for i in INPUTS for t in ("plus1", "minus1")))
GROUPS = (("observer Q x 0.3", ("q_x0.3",)), ("observer R_yaw x 10", ("ryaw_x10",)),
          *((f"drag c_D A {x:.1f} m^2", (f"drag_cda_{x:.1f}",)) for x in DRAG_CDA),
          *((f"fit-L curvature E = {e:.0f}", (f"e_{e:+.0f}",)) for e in E_VALUES),
          *((f"{i} {s}1 %", (f"{i}_{t}",)) for i in INPUTS for s, t in (("+", "plus1"), ("-", "minus1"))))
KINDS = (("US", "front"), ("OS", "rear"), ("INST", "stability"))


def _params(variant):
    """Resolved parameters for one variant (Level-1 vehicle, no setup sheet,
    as the development set runs) plus the steering-table patch if any."""
    import modules.accuracy_resolution as accres
    from modules.stability_analysis import load_parameters
    raw = copy.deepcopy(load_parameters())
    inp, _, tag = variant.rpartition("_")
    vehicle = inp in INPUTS
    sign = {"plus1": 1, "minus1": -1}.get(tag)
    if vehicle and inp != "steering_wheel_angle":
        raw = _variant_params(raw, inp, sign)
    if vehicle and inp == "steering_wheel_angle":
        patched = _scale_wheel_angle_column(accres.load_car_data(), sign * PCT)
        orig = accres.load_car_data
        accres.load_car_data = lambda: patched
        try:
            resolved = accres.resolve_accuracy(raw, setup_data=None, cap=None)
        finally:
            accres.load_car_data = orig
    else:
        resolved = accres.resolve_accuracy(raw, setup_data=None, cap=None)
    p = copy.deepcopy(accres.apply_resolved_vehicle(raw, resolved))
    if variant.startswith("drag_cda_"):
        area = p["vehicle"]["aero"]["cross_track_area_m2"]
        p["longitudinal_stiffness"]["drag_coeff"] = float(variant[len("drag_cda_"):]) / area
    return p


def _rec_key(rec):
    # session recommendation as lever + direction only (no setup values)
    def opt(o):
        return ("+".join(o["levers"]), o["direction"])
    if rec.get("recommended"):
        return ("recommended", opt(rec["recommended"]))
    if rec.get("equivalent_options"):
        return ("tie", tuple(sorted(opt(o) for o in rec["equivalent_options"])))
    return ("none",)


def run_task(job):
    key, variant = job
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_car_data, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, estimate_vertical_loads,
                                            summarise_corners)
    from modules.corner_analysis import reclassify_speed_classes
    from modules.corner_aggregation import aggregate_by_corner
    from modules.classifier import phase_symptoms
    from modules.setup_engine import data_layer, tc_activity
    from modules.setup_state import logged_setup_values
    t0 = time.perf_counter()
    path = resolve_data_path({c[0]: c for c in CANDIDATES}[key][4][1])
    p = _params(variant)
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    # pool workers are reused across jobs: everything patched is restored
    saved = {n: getattr(tfa, n) for n in ("_q_upper_from_residual", "_measurement_noise_floors", "L_CURVATURE_E")}
    try:
        if variant in ("q_x0.3", "ryaw_x10"):
            _install(variant, tfa)
        elif variant.startswith("e_"):
            tfa.L_CURVATURE_E = float(variant[2:])
        st = prepare_vehicle_state(data["channels"], p)
        beta, man, gate, fb, _fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        for n, f in saved.items():
            setattr(tfa, n, f)
    laps = data.get("laps", [])
    # lap filter as the app's analysis identity: analysis laps, else all laps
    valid = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis")) \
        or sorted(l["lap_number"] for l in laps)
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    fz = estimate_vertical_loads(st, forces, p, channels=data["channels"], car_data=load_car_data())
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, st, fz=fz, lap_filter=valid)
    units = {}
    for cid, agg in aggregate_by_corner(summaries).items():
        found = {(s["phase"], s["symptom"]): s["severity"] for s in phase_symptoms(agg)["symptoms"]}
        for ph in agg["phases"]:
            units[f"{cid}|{ph}"] = {k: found.get((ph, k)) for k, _ in KINDS}
    layer = data_layer(summaries, tc_map=tc_activity(corners, st, data["channels"]),
                       logged=logged_setup_values(data["channels"], laps, {s["lap_number"] for s in summaries}))
    out = {"key": key, "variant": variant, "status": man and man.get("status"),
           "gate": gate and gate.get("verdict"), "fallback": fb,
           # segmentation identity: perturbations must not move corners
           "brackets": sorted((s["lap_number"], s["stable_corner_id"], round(s["bracket_start_m"], 3),
                               round(s["bracket_end_m"], 3)) for s in summaries),
           "units": units, "rec": _rec_key(layer["session_recommendation"]),
           "n_voting_corners": len(layer["corners"]), "elapsed_s": time.perf_counter() - t0}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{key}__{variant}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{key} {variant}: {out['elapsed_s']:.0f} s, status {out['status']}, gate {out['gate']}, fallback {fb}"


def _load(key, variant):
    fn = os.path.join(OUT, f"{key}__{variant}.pkl")
    return pickle.load(open(fn, "rb")) if os.path.exists(fn) else None


def report():
    labels = {c[0]: c[3] for c in CANDIDATES}
    base = {k: _load(k, "baseline") for k in POOL}
    missing = [k for k, b in base.items() if b is None]
    L = ["SENS-1 RECOUNT (2026-10-29), 14 development sessions, one rule for all perturbations",
         "unit = corner x phase x {front US, rear OS, stability INST} of the session aggregate; changed = symptom",
         "appears / disappears / severity differs from baseline (classifier.phase_symptoms, current config)",
         "session recommendation = setup_engine.data_layer, lever + direction (or tied set), no setup sheet",
         "NOTE: the CS boundary margins were derived (p90) from the same Q x 0.3, R_yaw x 10 and +/-1 % vehicle",
         "runs, so part of the flips under those perturbations is built into the boundary.", ""]
    if missing:
        L.append(f"baseline missing: {', '.join(missing)}")
    B = {k: b for k, b in base.items() if b is not None}
    n_units = {k: sum(len(b["units"]) for b in B.values()) for k, _ in KINDS}
    n_base = {k: sum(u[k] is not None for b in B.values() for u in b["units"].values()) for k, _ in KINDS}
    L.append(f"BASELINE: {len(B)} sessions, {n_units['US']} corner-phases -> {3 * n_units['US']} units "
             f"({n_units['US']} per kind)")
    for k, ax in KINDS:
        sev = {}
        for b in B.values():
            for u in b["units"].values():
                if u[k] is not None:
                    sev[u[k]] = sev.get(u[k], 0) + 1
        L.append(f"  {k} ({ax}): {n_base[k]} units with the symptom ({', '.join(f'{s} {n}' for s, n in sorted(sev.items()))})")
    n_rec = {s: sum(b["rec"][0] == s for b in B.values()) for s in ("recommended", "tie", "none")}
    L.append(f"  session recommendations: {n_rec['recommended']} sessions with a recommendation, "
             f"{n_rec['tie']} ties, {n_rec['none']} none")
    L.append("  baseline status per session: " + "; ".join(
        f"{labels[k]} {b['status']}/{b['gate']}{' FALLBACK' if b['fallback'] else ''}" for k, b in B.items()))
    L.append("")
    res = {"baseline": {"units_per_kind": n_units, "symptoms": n_base,
                        "recs": {labels[k]: b["rec"] for k, b in B.items()}}, "variants": {}}
    for gname, (variant,) in GROUPS:
        runs = {k: _load(k, variant) for k in B}
        miss = [k for k, r in runs.items() if r is None]
        tot = {k: 0 for k, _ in KINDS}
        chg = {k: 0 for k, _ in KINDS}
        detail = {k: {"appear": 0, "disappear": 0, "severity": 0} for k, _ in KINDS}
        seg, stat, recs = [], [], []
        for key, r in runs.items():
            if r is None:
                continue
            b = B[key]
            if r["brackets"] != b["brackets"]:
                seg.append(labels[key])
            if (r["status"], r["gate"], r["fallback"]) != (b["status"], b["gate"], b["fallback"]):
                stat.append(f"{labels[key]} {b['status']}/{b['gate']}/{b['fallback']} -> "
                            f"{r['status']}/{r['gate']}/{r['fallback']}")
            for cp in set(b["units"]) | set(r["units"]):
                ub = b["units"].get(cp, {})
                ur = r["units"].get(cp, {})
                for k, _ in KINDS:
                    tot[k] += 1
                    x, y = ub.get(k), ur.get(k)
                    if x != y:
                        chg[k] += 1
                        detail[k]["appear" if x is None else "disappear" if y is None else "severity"] += 1
            if r["rec"] != b["rec"]:
                recs.append(f"{labels[key]}: {b['rec']} -> {r['rec']}")
        T, C = sum(tot.values()), sum(chg.values())
        L.append(f"{gname} ({variant}): {C} / {T} units changed ({100 * C / T if T else float('nan'):.1f} %)"
                 + (f"  [runs missing: {', '.join(miss)}]" if miss else ""))
        for k, ax in KINDS:
            d = detail[k]
            L.append(f"  {k} {ax}: {chg[k]} / {tot[k]} ({100 * chg[k] / tot[k] if tot[k] else float('nan'):.1f} %)"
                     f" -- appear {d['appear']}, disappear {d['disappear']}, severity {d['severity']}")
        L.append(f"  session recommendations changed: {len(recs)} / {len(runs) - len(miss)}")
        L += [f"    {x}" for x in recs]
        if seg:
            L.append(f"  segmentation differs: {', '.join(seg)}")
        if stat:
            L.append("  status / gate / fallback changed: " + "; ".join(stat))
        res["variants"][variant] = {"label": gname, "units": T, "changed": C, "per_kind": {k: [chg[k], tot[k]]
                                    for k, _ in KINDS}, "detail": detail, "rec_changed": recs,
                                    "segmentation_differs": seg, "status_changed": stat, "missing": miss}
    text = "\n".join(L)
    with open(os.path.join(OUT, "analysis.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "analysis.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    only = sys.argv[1].split(",") if len(sys.argv) > 1 else None
    # baselines first, so a partial run can already be reported
    tasks = [(k, v) for v in VARIANTS for k in POOL
             if (only is None or k in only) and not os.path.exists(os.path.join(OUT, f"{k}__{v}.pkl"))]
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
