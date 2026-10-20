# Robustness check of the observer noise on the default path (author EXECUTE
# 2026-10-20, final chain part 2, D). Report only, nothing selected.
# Thresholds fixed at the values adopted 2026-10-20 (read from config).
#
# Variants, applied inside the worker process only (module functions wrapped
# in memory; no module or config change):
#   baseline   Q-UPPER, measured R floors (the shipped default)
#   q_x0.1     both Q variances x 0.1
#   q_x0.3     both Q variances x 0.3
#   qr_hand    Q_beta at the upper bound, Q_r = the old hand-set value
#              (config q_yaw_rate_var, the value the fit-L keys copied) --
#              the yaw-moment equation constrains the force balance again
#   ryaw_x10   R_yaw = 10 x the measured yaw-rate noise floor
# Q enters the chain where Q-UPPER does (interim run, R sweep, final run), so
# each variant runs the whole chain as the default would with that Q / R.
#
# Per session: the app's chain (resolve_sideslip_beta_and_cs_inputs, CS with
# its inputs, stability, summarise_corners, detect_events); corner value =
# worst lap, worst of five phases, apex_3 from apex_region (_corner_values,
# the threshold population); class per axle: strong (< STRONG), moderate
# (< MODERATE), none, or no data. Per variant against the baseline, per
# session and axle: share of corners changing class, by direction (more /
# less severe, to / from no data); floor events appearing / disappearing
# (same lap, corner, axle, overlapping distance span = the same event);
# status, gate verdict / score, fallback, R_ay pick, hard-bound re-entries.
# Pooled flip rate per variant. The baseline is also checked against the
# threshold-derivation cache (cache_l_20261020): identical corner values.
# Output diagnostics/plots/observer_robustness/ (cache/, robustness.txt / .json).
# Run: python -m diagnostics.inspect_observer_robustness [report]

import copy
import json
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "observer_robustness")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
VARIANTS = ("baseline", "q_x0.1", "q_x0.3", "qr_hand", "ryaw_x10")
WORDS = {"baseline": "baseline (Q-UPPER, measured R floors)", "q_x0.1": "Q x 0.1", "q_x0.3": "Q x 0.3",
         "qr_hand": "Q_beta upper bound, Q_r old hand-set value", "ryaw_x10": "R_yaw x 10"}
WORKERS = 3
RANK = {"none": 0, "moderate": 1, "strong": 2}


def _install(variant, tfa, params):
    if variant in ("q_x0.1", "q_x0.3", "qr_hand"):
        orig = tfa._q_upper_from_residual

        def q_variant(*a, **kw):
            q = dict(orig(*a, **kw))
            if variant == "qr_hand":
                q["q_yaw_rate_var"] = params["tyre_fit_auto"]["q_yaw_rate_var"]
            else:
                f = 0.1 if variant == "q_x0.1" else 0.3
                q["q_beta_var"] *= f
                q["q_yaw_rate_var"] *= f
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


def compute(job):
    key, variant = job
    import modules.tyre_fit_auto as tfa
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
    from modules.limit_events import detect_events
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    # pool workers are reused across jobs: every wrapped function is restored
    saved = {n: getattr(tfa, n) for n in ("_q_upper_from_residual", "_measurement_noise_floors",
                                          "fit_session_pacejka")}
    held = {}

    def fsp(*a, **kw):
        held["raw"] = saved["fit_session_pacejka"](*a, **kw)
        return held["raw"]
    try:
        _install(variant, tfa, p)
        tfa.fit_session_pacejka = fsp
        st = prepare_vehicle_state(data["channels"], p)
        beta, man, gate, fb, fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        for n, f in saved.items():
            setattr(tfa, n, f)
    raw = held["raw"]
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(st, p), st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    summ = summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
    corners = _corner_values(aggregate_by_corner([x for x in summ if x["lap_number"] in valid]))
    ev = detect_events(cs, st, data.get("corners", []), laps, man, p, slip=slip, gate_verdict=gate,
                       axle_loads=cs_in["cs_kwargs"].get("axle_loads"), axle_fx=cs_in["axle_fx"])
    hb = raw.get("observer_reset_mask")
    n_reentry = int(np.sum(np.diff(np.concatenate(([0], np.asarray(hb, np.int8)))) == 1)) if hb is not None else None
    chosen = (man.get("r_sweep") or {}).get("chosen") or {}
    out = {"key": key, "label": label, "variant": variant, "status": man.get("status"),
           "reason": man.get("degenerate_reason") or man.get("marginal_reason"), "fallback": fb,
           "fallback_reason": fr, "gate": gate and gate.get("verdict"), "health": gate and gate.get("health_score"),
           "q": {k: (man.get("observer_q") or {}).get(k) for k in ("q_beta_var", "q_yaw_rate_var")},
           "r_ay_var": chosen.get("R_ay_var"), "r_yaw_var": chosen.get("R_yaw_rate_var"),
           "r_selection": (man.get("r_sweep") or {}).get("selection"), "n_hard_bound_reentries": n_reentry,
           "corners": corners,
           "events": [{k: e[k] for k in ("lap", "corner", "axle", "phase", "s_start_m", "s_end_m", "cs_min",
                                         "kerb_affected", "observer_reset_affected")} for e in ev["events"]]}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}__{variant}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key} {variant}] status {out['status']}, gate {out['gate']}, fallback {fb}, re-entries {n_reentry}, "
          f"{len(out['events'])} floor events, {time.perf_counter() - t0:.0f} s", flush=True)
    return job


def _cls(v, S, M):
    if v != v:
        return "no data"
    return "strong" if v < S else ("moderate" if v < M else "none")


def _same(a, b):
    return (a["lap"] == b["lap"] and a["corner"] == b["corner"] and a["axle"] == b["axle"]
            and a["s_start_m"] <= b["s_end_m"] and b["s_start_m"] <= a["s_end_m"])


def report():
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    thr = {"f": (cl["STRONG_CSF"]["value"], cl["MODERATE_CSF"]["value"]),
           "r": (cl["STRONG_CSR"]["value"], cl["MODERATE_CSR"]["value"])}
    res = {}
    for k in POOL:
        for v in VARIANTS:
            with open(os.path.join(CACHE, f"{k}__{v}.pkl"), "rb") as fh:
                res[(k, v)] = pickle.load(fh)
    L = ["OBSERVER-NOISE ROBUSTNESS on the default path -- report only, nothing selected",
         f"Thresholds fixed (adopted 2026-10-20): front {thr['f']}, rear {thr['r']}", ""]
    # baseline reproduces the threshold-derivation cache
    drift = []
    for k in POOL:
        fn = os.path.join(ROOT, "diagnostics", "plots", "threshold_rederivation", "cache_l_20261020", f"{k}.pkl")
        if os.path.exists(fn):
            ref = pickle.load(open(fn, "rb"))["corners"]
            b = res[(k, "baseline")]["corners"]
            same = set(ref) == set(b) and all(
                (ref[c][ax] == b[c][ax]) or (ref[c][ax] != ref[c][ax] and b[c][ax] != b[c][ax])
                for c in ref for ax in "fr")
            if not same:
                drift.append(k)
    L.append("Baseline vs threshold-derivation cache: " + ("identical corner values on all 14 sessions"
                                                            if not drift else f"DIFFERS on {drift}"))
    L.append("")
    L.append("BASELINE per session: status / gate (score) / fallback / R_ay / R_yaw / hard-bound re-entries / "
             "floor events")
    for k in POOL:
        b = res[(k, "baseline")]
        L.append(f"  {b['label']}: {b['status']} / {b['gate']} ({b['health']:.4f}) / {b['fallback']} / "
                 f"{b['r_ay_var']:.3g} / {b['r_yaw_var']:.3g} / {b['n_hard_bound_reentries']} / {len(b['events'])}")
    J = {}
    for v in VARIANTS[1:]:
        L += ["", f"VARIANT {WORDS[v]}:"]
        J[v] = {"sessions": {}}
        tot = {"f": [0, 0], "r": [0, 0]}
        dirs_tot = {"more severe": 0, "less severe": 0, "to no data": 0, "from no data": 0}
        ev_tot = {"base": 0, "appear": 0, "disappear": 0}
        for k in POOL:
            b, x = res[(k, "baseline")], res[(k, v)]
            rec = {}
            parts = []
            for ax, axn in (("f", "front"), ("r", "rear")):
                S, M = thr[ax]
                n = flips = 0
                dirs = {"more severe": 0, "less severe": 0, "to no data": 0, "from no data": 0}
                detail = []
                for c in sorted(set(b["corners"]) | set(x["corners"]), key=str):
                    cb = _cls(b["corners"].get(c, {}).get(ax, float("nan")), S, M)
                    cx = _cls(x["corners"].get(c, {}).get(ax, float("nan")), S, M)
                    if cb == "no data" and cx == "no data":
                        continue
                    n += 1
                    if cb != cx:
                        flips += 1
                        if cx == "no data":
                            d = "to no data"
                        elif cb == "no data":
                            d = "from no data"
                        else:
                            d = "more severe" if RANK[cx] > RANK[cb] else "less severe"
                        dirs[d] += 1
                        detail.append(f"C{c} {cb}->{cx}")
                tot[ax][0] += flips
                tot[ax][1] += n
                for d in dirs:
                    dirs_tot[d] += dirs[d]
                rec[axn] = {"n": n, "flips": flips, "share": flips / n if n else float("nan"), "directions": dirs,
                            "detail": detail}
                parts.append(f"{axn} {flips}/{n}" + (f" ({', '.join(detail)})" if detail else ""))
            app = [e for e in x["events"] if not any(_same(e, o) for o in b["events"])]
            dis = [e for e in b["events"] if not any(_same(e, o) for o in x["events"])]
            ev_tot["base"] += len(b["events"])
            ev_tot["appear"] += len(app)
            ev_tot["disappear"] += len(dis)
            changes = [f"{f_} {b[f_]} -> {x[f_]}" for f_ in ("status", "gate", "fallback") if b[f_] != x[f_]]
            rec.update({"events_base": len(b["events"]), "events_variant": len(x["events"]),
                        "appear": len(app), "disappear": len(dis), "changes": changes,
                        "health": (b["health"], x["health"]),
                        "reentries": (b["n_hard_bound_reentries"], x["n_hard_bound_reentries"]),
                        "r_ay": (b["r_ay_var"], x["r_ay_var"])})
            J[v]["sessions"][k] = rec
            L.append(f"  {b['label']}: class changes {'; '.join(parts)}; floor events {len(b['events'])} -> "
                     f"{len(x['events'])} (+{len(app)} / -{len(dis)}); gate score {b['health']:.4f} -> "
                     f"{x['health']:.4f}; re-entries {b['n_hard_bound_reentries']} -> {x['n_hard_bound_reentries']}; "
                     f"R_ay {b['r_ay_var']:.3g} -> {x['r_ay_var']:.3g}"
                     + (f"; CHANGED: {', '.join(changes)}" if changes else ""))
        pooled = (tot["f"][0] + tot["r"][0]) / max(1, tot["f"][1] + tot["r"][1])
        J[v]["pooled"] = {"front": tot["f"], "rear": tot["r"], "flip_rate": pooled, "directions": dirs_tot,
                          "events": ev_tot}
        L.append(f"  POOLED flip rate {100 * pooled:.1f} % (front {tot['f'][0]}/{tot['f'][1]}, rear "
                 f"{tot['r'][0]}/{tot['r'][1]}); directions " + ", ".join(f"{d} {n_}" for d, n_ in dirs_tot.items())
                 + f"; floor events baseline {ev_tot['base']}, appear {ev_tot['appear']}, disappear "
                   f"{ev_tot['disappear']}")
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "robustness.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, "robustness.json"), "w") as fh:
        json.dump(J, fh, indent=1, default=str)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    from diagnostics.inspect_fit_lit import keep_awake
    keep_awake(True)
    try:
        jobs = [(k, v) for v in VARIANTS for k in POOL
                if not os.path.exists(os.path.join(CACHE, f"{k}__{v}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, jobs):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
