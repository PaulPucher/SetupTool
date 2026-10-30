# Final number check before hand-in (author go 2026-10-30): the sources of
# the thesis numbers that had none on record. Read-only; report only.
#
# stored (evaluation set, stored identity-valid payloads, no re-run):
#   1 floor events of PRC 3 / PRC 5 split by the stored flags kerb_affected
#     and braking (every Fx sample of the event < 0), as tab_observer_health
#     counts them (the stored events list);
#   2 repeated instability: the INST symptoms the data layer counts (corner,
#     phase, laps k of n, cross-lap median dMz/dbeta) and the per-lap INST
#     phases behind them (phase median < stab_neg_thresh);
#   3 corner-phase dMz/dbeta pooled per track, p10 / p90 -- the classifier's
#     population: cross-lap median per corner-phase (aggregate_by_corner over
#     the stored analysis-lap summaries), finite values only;
#   4 the PRC 5 session vote: every ranked option with its net support.
# rerun (14 development sessions, the shipped chain, Level-1 vehicle as the
# development set runs; same harness as inspect_sens1_recount):
#   C / E variants of fit L (C 1.0 with E -1 / 0 / +1, C 1.3 and 1.6 with
#   E -1; the ones Ch. 3 cites), set in memory per job and restored: K and
#   its zone per axle, normalised fit residual, curve peak inside 15 deg,
#   hard-bound re-entries; on the baseline also the median NIS over the fit
#   base mask (final observer run) and the reference candidates rejected by
#   the relative gate (L_REF_RELATIVE) per axle.
# Output diagnostics/results_final_number_check_20261030/.
# Run: python -m diagnostics.inspect_final_number_check stored | rerun | report

import copy
import json
import os
import pickle
import sys
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT = os.path.join(ROOT, "diagnostics", "results_final_number_check_20261030")
WORKERS = 6
EVAL = {"NBR": (17, 18, 19, 20, 21), "PRC": (22, 25, 26, 27, 28)}
VARIANTS = ((1.0, -1.0), (1.0, 0.0), (1.0, 1.0), (1.3, -1.0), (1.6, -1.0))
BASE = (1.0, -1.0)
PEAK_MAX_DEG = 15.0


def _tag(v):
    return f"C{v[0]:.1f}_E{v[1]:+.0f}"


def peak_deg(B, C, E):
    # first sign change of the curve slope in (0, 15] deg; None = no peak
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, PEAK_MAX_DEG, 3000)
    s = pacejka_lateral_stiffness(np.radians(g), B, C, 1.0, E)
    idx = np.where(np.diff(np.sign(s)) < 0)[0]
    return float(g[idx[0]]) if idx.size else None


def stored():
    from tools.thesis_figures import common
    from modules.corner_aggregation import aggregate_by_corner
    from modules.classifier import _thresholds
    stab = _thresholds()["stab"]
    L, J = [], {}
    pooled = {"NBR": [], "PRC": []}
    for track, outings in EVAL.items():
        for o in outings:
            s = common.load_session(o)
            p, why = common.stored_payload(s)
            if p is None:
                raise SystemExit(f"outing {o}: {why}")
            lab = common.session_label(s["row"])
            summ = p["summaries"]
            # 3: classifier population, cross-lap median per corner-phase
            vals = [a["phases"][ph]["stability_observed_Nm_per_deg"]["median"]
                    for a in aggregate_by_corner(summ).values() for ph in a["phases"]]
            vals = [v for v in vals if v == v]
            pooled[track] += vals
            J.setdefault("dmz_session", {})[lab] = {"n": len(vals), "p10": float(np.percentile(vals, 10)),
                                                   "p50": float(np.percentile(vals, 50)),
                                                   "p90": float(np.percentile(vals, 90))}
            # 1: floor events
            ev = (p.get("events") or {}).get("events") or []
            k = sum(bool(e["kerb_affected"]) for e in ev)
            b = sum(e.get("braking") is True for e in ev)
            kb = sum(bool(e["kerb_affected"]) and e.get("braking") is True for e in ev)
            bs = sum((e.get("braking_share") or 0) > 0 for e in ev)
            J.setdefault("floor_events", {})[lab] = {"n": len(ev), "kerb": k, "braking": b, "both": kb,
                                                     "any_braking_sample": bs,
                                                     "axles": sorted({e["axle"] for e in ev})}
            # 2: counted INST and the per-lap phases behind it
            dl = p["setup_engine"]["data_layer"]
            inst = []
            for c in dl["corners"]:
                for sy in c["symptoms"]:
                    if sy["symptom"] != "INST":
                        continue
                    per_lap = {x["lap_number"]: x["phases"][sy["phase"]]["stability_observed_Nm_per_deg"]["median"]
                               for x in summ if x["stable_corner_id"] == c["corner"] and sy["phase"] in x["phases"]}
                    inst.append({"corner": c["corner"], "phase": sy["phase"], "laps": sy["laps"],
                                 "n_laps": c["n_laps"], "lap_numbers": sy.get("lap_numbers"),
                                 "value_cross_lap_median": sy["value"],
                                 "per_lap_inst": sorted(l for l, v in per_lap.items() if v == v and v < stab)})
            J.setdefault("inst_counted", {})[lab] = inst
            # 4: full ranked option list
            rec = dl["session_recommendation"]
            J.setdefault("ranked", {})[lab] = [
                {"option": r["option"], "levers": r["levers"], "direction": r["direction"], "net": r["net"],
                 "advisory": r.get("advisory"),
                 "supporting": [(x["corner"], x["symptom"], x["phase"], x["laps"], x["n_laps"]) for x in r["supporting"]]}
                for r in rec["ranked"]]
            J.setdefault("recommended", {})[lab] = rec["recommended"] and rec["recommended"]["option"]
    J["dmz_pooled"] = {t: {"n": len(v), "p10": float(np.percentile(v, 10)), "p50": float(np.percentile(v, 50)),
                           "p90": float(np.percentile(v, 90))} for t, v in pooled.items()}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "stored.json"), "w") as fh:
        json.dump(J, fh, indent=1, default=str)
    L.append("FLOOR EVENTS (stored events list; kerb = kerb_affected, braking = every Fx sample < 0)")
    for lab, f in J["floor_events"].items():
        L.append(f"  {lab}: {f['n']} events, kerb {f['kerb']}, braking {f['braking']}, both {f['both']} "
                 f"(any braking sample {f['any_braking_sample']}), axles {', '.join(f['axles'])}")
    L.append("COUNTED INST (data layer): corner phase laps k/n, cross-lap median dMz/dbeta, per-lap INST laps")
    for lab, xs in J["inst_counted"].items():
        for x in xs:
            L.append(f"  {lab} C{x['corner']} {x['phase']}: {x['laps']}/{x['n_laps']}, "
                     f"{x['value_cross_lap_median']:.1f} N m/deg, per-lap INST laps {x['per_lap_inst']}")
    L.append("dMz/dbeta CORNER-PHASE (cross-lap median) p10 / p50 / p90, N m/deg")
    for t, d in J["dmz_pooled"].items():
        L.append(f"  {t} pooled: n {d['n']}, {d['p10']:.0f} / {d['p50']:.0f} / {d['p90']:.0f}")
    for lab, d in J["dmz_session"].items():
        L.append(f"    {lab}: n {d['n']}, {d['p10']:.0f} / {d['p50']:.0f} / {d['p90']:.0f}")
    L.append("PRC 5 RANKED OPTIONS (net support)")
    for r in J["ranked"]["PRC 5"]:
        L.append(f"  {r['net']:.2f} {r['option']} ({r['direction']}){' advisory' if r['advisory'] else ''}: "
                 + "; ".join(f"C{c} {sy} {ph} {k}/{n}" for c, sy, ph, k, n in r["supporting"]))
    L.append(f"  recommended: {J['recommended']['PRC 5']}")
    text = "\n".join(L)
    with open(os.path.join(OUT, "stored.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


def run_task(job):
    key, v = job
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_cs_margin_l import CANDIDATES
    from diagnostics.inspect_sens1_recount import _params
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness)
    from modules.corner_analysis import reclassify_speed_classes
    t0 = time.perf_counter()
    path = resolve_data_path({c[0]: c for c in CANDIDATES}[key][4][1])
    p = _params("baseline")
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    saved = {"L_SHAPE_C": tfa.L_SHAPE_C, "L_CURVATURE_E": tfa.L_CURVATURE_E,
             "fit_session_pacejka": tfa.fit_session_pacejka}
    held = {}

    def fsp(*a, **kw):
        held["raw"] = saved["fit_session_pacejka"](*a, **kw)
        return held["raw"]
    try:
        tfa.L_SHAPE_C, tfa.L_CURVATURE_E = v
        tfa.fit_session_pacejka = fsp
        st = prepare_vehicle_state(data["channels"], p)
        beta, man, gate, fb, _fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        for n, x in saved.items():
            setattr(tfa, n, x)
    raw = held.get("raw") or {}
    hb = raw.get("observer_reset_mask")
    out = {"key": key, "variant": v, "status": man and man.get("status"),
           "reason": man and (man.get("degenerate_reason") or man.get("marginal_reason")),
           "gate": gate and gate.get("verdict"), "fallback": fb,
           "reentries": (int(np.sum(np.diff(np.concatenate(([0], np.asarray(hb, np.int8)))) == 1))
                         if hb is not None else None), "axles": {}}
    for name in ("front", "rear"):
        a = (man or {}).get("axles", {}).get(name, {})
        B, C, mu = a.get("B", float("nan")), a.get("C", float("nan")), a.get("mu", float("nan"))
        out["axles"][name] = {"B": B, "C": C, "mu": mu, "K": B * C * mu, "rms_norm": a.get("fit_rms_resid_norm"),
                              "zone": (man or {}).get("k_band_zone", {}).get(name),
                              "peak_deg": peak_deg(B, C, v[1]) if np.isfinite(B) else None}
    if v == BASE:
        nis, base = raw.get("nis_full"), raw.get("base_mask")
        if nis is not None and base is not None:
            x = np.asarray(nis, float)[np.asarray(base, bool) & np.isfinite(nis)]
            out["nis_median"] = float(np.median(x))
            out["nis_mean"] = float(np.mean(x))
        slip = estimate_slip_angles(st, beta, p)
        cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(st, p), st, p, data.get("laps", []),
                                          **cs_in["cs_kwargs"])
        out["ref_rejected"] = {ax: {"low": cs.get(f"n_ref_relative_low_{ax}"),
                                    "high": cs.get(f"n_ref_relative_high_{ax}")} for ax in ("f", "r")}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{key}__{_tag(v)}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{key} {_tag(v)}: {time.perf_counter() - t0:.0f} s, status {out['status']}, fallback {fb}"


def rerun():
    from diagnostics.inspect_cs_margin_l import POOL, keep_awake
    jobs = [(k, v) for v in VARIANTS for k in POOL if not os.path.exists(os.path.join(OUT, f"{k}__{_tag(v)}.pkl"))]
    keep_awake(True)
    try:
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            futs = [ex.submit(run_task, j) for j in jobs]
            for i, f in enumerate(as_completed(futs), 1):
                print(f"[{i}/{len(jobs)}] {f.result()}", flush=True)
    finally:
        keep_awake(False)
    report()


def report():
    from diagnostics.inspect_cs_margin_l import CANDIDATES, POOL
    lab = {c[0]: c[3] for c in CANDIDATES}
    R = {(k, v): pickle.load(open(os.path.join(OUT, f"{k}__{_tag(v)}.pkl"), "rb")) for k in POOL for v in VARIANTS}
    L = ["RE-MEASURE ON THE FINAL CHAIN (14 development sessions)", ""]
    b = {k: R[(k, BASE)] for k in POOL}
    nm = [b[k]["nis_median"] for k in POOL]
    L.append(f"median NIS per session (fit base mask, final observer run): {min(nm):.3f} - {max(nm):.3f}")
    L += [f"  {lab[k]}: median {b[k]['nis_median']:.3f}, mean {b[k]['nis_mean']:.3f}" for k in POOL]
    for ax in ("f", "r"):
        lo = sum(b[k]["ref_rejected"][ax]["low"] or 0 for k in POOL)
        hi = sum(b[k]["ref_rejected"][ax]["high"] or 0 for k in POOL)
        L.append(f"reference candidates rejected by the relative gate, {ax}: {lo} low / {hi} high ("
                 + ", ".join(f"{lab[k]} {b[k]['ref_rejected'][ax]['low']}/{b[k]['ref_rejected'][ax]['high']}"
                             for k in POOL) + ")")
    L.append("")
    for v in VARIANTS:
        rs = [R[(k, v)] for k in POOL]
        L.append(f"VARIANT C {v[0]:.1f}, E {v[1]:+.0f}" + (" (shipped)" if v == BASE else ""))
        for ax in ("front", "rear"):
            z = [r["axles"][ax]["zone"] for r in rs]
            L.append(f"  K zone {ax}: " + ", ".join(f"{s} {z.count(s)}" for s in sorted(set(map(str, z)))))
        sess_zone = ["outside" if "outside" in (r["axles"]["front"]["zone"], r["axles"]["rear"]["zone"])
                     else "edge" if "edge" in (r["axles"]["front"]["zone"], r["axles"]["rear"]["zone"]) else "in band"
                     for r in rs]
        L.append("  sessions (worse axle): " + ", ".join(f"{s} {sess_zone.count(s)}" for s in sorted(set(sess_zone))))
        L.append("  status: " + ", ".join(f"{s} {sum(r['status'] == s for r in rs)}"
                                         for s in sorted({str(r['status']) for r in rs}))
                 + f"; fallback {sum(bool(r['fallback']) for r in rs)}; re-entries sum "
                 f"{sum(r['reentries'] or 0 for r in rs)} (per session {[r['reentries'] for r in rs]})")
        for ax in ("front", "rear"):
            pk = [r["axles"][ax]["peak_deg"] for r in rs if r["axles"][ax]["peak_deg"] is not None]
            if pk:
                L.append(f"  peak {ax} inside {PEAK_MAX_DEG:g} deg on {len(pk)}/14: {min(pk):.1f} - {max(pk):.1f} deg")
        if v != BASE:
            for ax in ("front", "rear"):
                ch = [100 * (R[(k, v)]["axles"][ax]["rms_norm"] / b[k]["axles"][ax]["rms_norm"] - 1) for k in POOL
                      if R[(k, v)]["axles"][ax]["rms_norm"] and b[k]["axles"][ax]["rms_norm"]]
                L.append(f"  normalised residual change vs shipped, {ax}: median {np.median(ch):+.2f} %, "
                         f"min {min(ch):+.2f} %, max {max(ch):+.2f} %")
        L.append("")
    text = "\n".join(L)
    with open(os.path.join(OUT, "rerun.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    {"stored": stored, "rerun": rerun, "report": report}[mode]()
