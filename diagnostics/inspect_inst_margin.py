# INST uncertainty margin (ORDER INST-1, author + review 2026-11-07).
# The open-loop instability verdict (dMz/dbeta < 0, Werner 2021 sec. 2.2.3)
# gets the treatment the CS boundary already has: a margin from the
# estimation uncertainty around the sign criterion. Sensitivity analysis,
# Tier B, report only; nothing is adopted in config here.
#
# Rule, identical to the CS boundary margin (inspect_cs_margin_combined,
# DECISIONS 'Classifier per D2', canonical reading = largest vehicle run):
#   unit  corner x phase of the session aggregate the classifier reads for
#         INST: median over the analysis laps of the per-lap phase median of
#         stability_observed_Nm_per_deg (classifier.aggregate_by_corner)
#   obs   max(|d| Q x 0.3, |d| R_yaw x 10), wrappers of
#         inspect_cs_margin_combined unchanged
#   veh   max |d| over the ten +/-1 % vehicle-input runs
#         (_variant_params / steering-table scaling of inspect_cs_margin_l)
#   comb  sqrt(obs^2 + veh^2); m_INST = pooled p90 over the 14 development
#         sessions, in N m/deg
# Deliberate difference to the recorded CS run: obs and veh come from the
# same population (analysis laps). The CS run took veh from all laps and the
# raw apex_3 slice, a mismatch it stated as a flaw; the all-lap reading is
# reported beside for comparison.
# The vehicle-input band (the cs_margin analog) is reported two ways because
# the recorded flip rule (_is_flip) only knows US <-> OS direction changes:
# the flip distances of units whose aggregate INST appears / disappears
# under a +/-1 % run, and the p90 of veh alone.
#
# Mode "final": the ten final sessions through the thesis-figure helpers
# (load_session / run_chain, stored setup sheets from the DB of the copy it
# runs in); pickles summaries, tc_map and logged values so the report can
# rebuild the data layer at any INST boundary in memory.
# Output diagnostics/results_inst_margin_20261107/ (env INST_MARGIN_OUT
# overrides; the runs execute from a scratchpad copy of the working repo).
# Run: python -m diagnostics.inspect_inst_margin [dev | final | report]

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
from diagnostics.inspect_cs_margin_combined import _install  # noqa: E402
from diagnostics.inspect_sens1_recount import _params  # noqa: E402

OUT = os.environ.get("INST_MARGIN_OUT") or os.path.join(ROOT, "diagnostics", "results_inst_margin_20261107")
WORKERS = int(os.environ.get("INST_MARGIN_WORKERS", "8"))
OBS_VARIANTS = ("q_x0.3", "ryaw_x10")
VEH_VARIANTS = tuple(f"{i}_{t}" for i in INPUTS for t in ("plus1", "minus1"))
VARIANTS = ("baseline",) + OBS_VARIANTS + VEH_VARIANTS
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
# the ten final sessions by app outing id (naming of ORDER SPEED-1)
FINAL = {"NBR1": 17, "NBR2": 18, "NBR3": 19, "NBR4": 20, "NBR5": 21,
         "PRC1": 22, "PRC2": 25, "PRC3": 26, "PRC4": 27, "PRC5": 28}
STAB = "stability_observed_Nm_per_deg"


def _cell_values(s, ph):
    from modules.classifier import verdict_cs_stat
    return {"stab": s["phases"][ph][STAB]["median"],
            "csf": verdict_cs_stat(s, ph, "f")["median"], "csr": verdict_cs_stat(s, ph, "r")["median"]}


def run_dev(job):
    key, variant = job
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_car_data, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, estimate_vertical_loads,
                                            summarise_corners)
    from modules.corner_analysis import reclassify_speed_classes
    from modules.classifier import aggregate_by_corner
    t0 = time.perf_counter()
    path = resolve_data_path({c[0]: c for c in CANDIDATES}[key][4][1])
    p = _params(variant)
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    # pool workers are reused across jobs: everything patched is restored
    saved = {n: getattr(tfa, n) for n in ("_q_upper_from_residual", "_measurement_noise_floors")}
    try:
        if variant in OBS_VARIANTS:
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
    # the app's analysis identity: analysis laps, else all laps
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")} or {l["lap_number"] for l in laps}
    vsum = [s for s in summaries if s["lap_number"] in valid]
    out = {"key": key, "variant": variant, "status": man and man.get("status"),
           "gate": gate and gate.get("verdict"), "fallback": fb, "valid_laps": sorted(valid),
           "brackets": sorted((s["lap_number"], s["stable_corner_id"], round(s["bracket_start_m"], 3),
                               round(s["bracket_end_m"], 3)) for s in vsum),
           "agg": {f"{cid}|{ph}": _cell_values(c, ph) for cid, c in aggregate_by_corner(vsum).items()
                   for ph in PHASES if ph in c["phases"]},
           "agg_all": {f"{cid}|{ph}": _cell_values(c, ph) for cid, c in aggregate_by_corner(summaries).items()
                       for ph in PHASES if ph in c["phases"]},
           "per_lap": {f"{s['lap_number']}|{s['stable_corner_id']}|{ph}": _cell_values(s, ph)
                       for s in vsum if s.get("stable_corner_id") is not None for ph in PHASES
                       if ph in s["phases"]},
           "elapsed_s": time.perf_counter() - t0}
    d = os.path.join(OUT, "dev")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f"{key}__{variant}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{key} {variant}: {out['elapsed_s']:.0f} s, status {out['status']}, gate {out['gate']}, fallback {fb}"


def run_final(name):
    sys.path.insert(0, os.path.join(ROOT, "tools", "thesis_figures"))
    import common  # noqa  (chdir to this repo copy; DB read-only)
    from modules.setup_engine import tc_activity
    from modules.setup_state import logged_setup_values
    t0 = time.perf_counter()
    sess = common.load_session(FINAL[name])
    ch = common.run_chain(sess)
    row, parsed = sess["row"], sess["parsed"]
    summ = ch["summaries"]
    from modules import pipeline
    dl = pipeline.recommend(summ, setup_data=json.loads(row["setup_data"] or "null"), corners=ch["corners"],
                            state=ch["state"], channels=parsed.get("channels"), laps=parsed.get("laps"))["data_layer"]
    out = {"name": name, "outing": FINAL[name], "summaries": summ,
           # identity reference: the pipeline's own data layer at the shipped config
           "pipeline_counted": sorted((c["corner"], s["symptom"], s["phase"], s["laps"]) for c in dl["corners"]
                                      for s in c["symptoms"]),
           "setup_data": json.loads(row["setup_data"] or "null"),
           "tc_map": tc_activity(ch["corners"], ch["state"], parsed.get("channels")),
           "logged": logged_setup_values(parsed.get("channels"), parsed.get("laps"),
                                         {s["lap_number"] for s in summ}),
           "status": (ch["fit_manifest"] or {}).get("status"),
           "gate": (ch["gate_verdict"] or {}).get("verdict"), "fallback": ch["fallback_used"],
           "elapsed_s": time.perf_counter() - t0}
    d = os.path.join(OUT, "final")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f"{name}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{name}: {out['elapsed_s']:.0f} s, status {out['status']}, gate {out['gate']}"


# ---- report ------------------------------------------------------------

def _fin(x):
    return x is not None and x == x


def _pct(v, qs=(50, 90, 99)):
    v = np.asarray([x for x in v if _fin(x)], float)
    return {"n": int(v.size), **{f"p{q}": (float(np.percentile(v, q)) if v.size else float("nan")) for q in qs}}


def _fmt(s):
    return f"n {s['n']:4d}  " + "  ".join(f"{k} {v:8.2f}" for k, v in s.items() if k != "n")


def _load_dev(key, variant):
    fn = os.path.join(OUT, "dev", f"{key}__{variant}.pkl")
    return pickle.load(open(fn, "rb")) if os.path.exists(fn) else None


def _units(pop):
    """Per-unit changes for one population ('agg' or 'agg_all')."""
    rows, seg, stat = [], [], []
    for key in POOL:
        runs = {v: _load_dev(key, v) for v in VARIANTS}
        if any(r is None for r in runs.values()):
            continue
        b = runs["baseline"]
        for v, r in runs.items():
            if r["brackets"] != b["brackets"]:
                seg.append(f"{key} {v}")
            if (r["status"], r["gate"], r["fallback"]) != (b["status"], b["gate"], b["fallback"]):
                stat.append(f"{key} {v}: {b['status']}/{b['gate']}/{b['fallback']} -> "
                            f"{r['status']}/{r['gate']}/{r['fallback']}")
        for cp, bv in b[pop].items():
            x0 = bv["stab"]
            if not _fin(x0):
                continue
            d = {v: (abs(runs[v][pop][cp]["stab"] - x0) if _fin(runs[v][pop].get(cp, {}).get("stab")) else
                     float("nan")) for v in VARIANTS[1:]}
            o = [d[v] for v in OBS_VARIANTS if _fin(d[v])]
            per_inp = {}
            for inp in INPUTS:
                s = [d[f"{inp}_{t}"] for t in ("plus1", "minus1") if _fin(d[f"{inp}_{t}"])]
                if s:
                    per_inp[inp] = max(s)
            obs = max(o) if o else float("nan")
            vmax = max(per_inp.values()) if per_inp else float("nan")
            vrss = float(np.sqrt(np.sum(np.square(list(per_inp.values()))))) if per_inp else float("nan")
            rows.append({"session": key, "cp": cp, "stab": x0, "csr": bv["csr"], "d": d, "obs": obs,
                         "veh_max": vmax, "veh_rss": vrss, "per_inp": per_inp,
                         "comb_max": float(np.hypot(obs, vmax)) if _fin(obs) and _fin(vmax) else float("nan"),
                         "comb_rss": float(np.hypot(obs, vrss)) if _fin(obs) and _fin(vrss) else float("nan"),
                         "var_stab": {v: runs[v][pop].get(cp, {}).get("stab") for v in VEH_VARIANTS},
                         "q_signed": (runs["q_x0.3"][pop].get(cp, {}).get("stab") or float("nan")) - x0})
    return rows, seg, stat


def _q_val(r):
    return r["stab"] + r["q_signed"] if _fin(r.get("q_signed")) else float("nan")


def _flips(rows, boundary):
    # cs_margin analog: units whose aggregate INST (stab < boundary) appears or
    # disappears under a +/-1 % vehicle run; distance = |baseline - boundary|
    out = []
    for r in rows:
        base_inst = r["stab"] < boundary
        changed = [v for v, x in r["var_stab"].items() if _fin(x) and (x < boundary) != base_inst]
        if changed:
            out.append({"session": r["session"], "cp": r["cp"], "stab": r["stab"],
                        "distance": abs(r["stab"] - boundary), "runs": changed})
    return sorted(out, key=lambda f: -f["distance"])


def _cs_check(L):
    # chain identity reading: the CS boundary margins by the recorded rule
    # from these runs, against the adopted 0.063 / 0.077 (derived on the
    # schema-32 chain with veh on all laps / raw apex_3)
    from modules.classifier import aggregate_by_corner  # noqa
    for ax, name, adopted in (("csf", "front", 0.063), ("csr", "rear", 0.077)):
        comb = []
        for key in POOL:
            runs = {v: _load_dev(key, v) for v in VARIANTS}
            if any(r is None for r in runs.values()):
                continue
            for cp, bv in runs["baseline"]["agg"].items():
                c0 = bv[ax]
                if not _fin(c0):
                    continue
                o = [abs(runs[v]["agg"][cp][ax] - c0) for v in OBS_VARIANTS
                     if _fin(runs[v]["agg"].get(cp, {}).get(ax))]
                vb = runs["baseline"]["agg_all"].get(cp, {}).get(ax)
                veh = [abs(runs[v]["agg_all"][cp][ax] - vb) for v in VEH_VARIANTS
                       if _fin(vb) and _fin(runs[v]["agg_all"].get(cp, {}).get(ax))]
                if o and veh:
                    comb.append(float(np.hypot(max(o), max(veh))))
        L.append(f"  CS {name}: p90 combined {np.percentile(comb, 90):.4f} (n {len(comb)}); adopted {adopted}")


def _patch_thr(stab_boundary):
    import modules.classifier as cl
    import modules.setup_engine as se
    orig = cl._thresholds

    def thr():
        t = orig()
        t["stab"] = stab_boundary
        return t
    cl._thresholds, se._thresholds = thr, thr
    return orig


def _restore_thr(orig):
    import modules.classifier as cl
    import modules.setup_engine as se
    cl._thresholds, se._thresholds = orig, orig


def _final_eval(fin, boundary, marginal_band=None):
    """Data layer and verdicts of one final session with the INST boundary at
    `boundary` (in memory). marginal_band = (lo, hi): aggregate INST values in
    [lo, hi) carry an INST [MARGINAL] tag in the verdict text."""
    from modules.classifier import aggregate_by_corner, classify_corner, phase_symptoms
    from modules.setup_engine import data_layer, cell_shares
    orig = _patch_thr(boundary)
    try:
        agg = aggregate_by_corner(fin["summaries"])
        verdicts, inst_units, inst_rear = {}, 0, 0
        for cid, a in agg.items():
            v = classify_corner(a)[1]
            if marginal_band is not None:
                lo, hi = marginal_band
                tag = [ph for ph in PHASES if ph in a["phases"]
                       and _fin(a["phases"][ph][STAB]["median"]) and lo <= a["phases"][ph][STAB]["median"] < hi]
                if tag:
                    v += " | INST [MARGINAL] @ " + ",".join(tag)
            verdicts[cid] = v
            for s in phase_symptoms(a)["symptoms"]:
                if s["symptom"] == "INST":
                    inst_units += 1
        dl = data_layer(fin["summaries"], fin["setup_data"], tc_map=fin["tc_map"], logged=fin["logged"])
        counted = sorted((c["corner"], s["symptom"], s["phase"], s["laps"]) for c in dl["corners"]
                         for s in c["symptoms"])
        # INST cells (per lap and phase) coinciding with rear limiting there
        from modules.classifier import _thresholds
        thr = _thresholds()
        n_cell, n_cell_rear = 0, 0
        for s in fin["summaries"]:
            for sym in phase_symptoms(s, thr)["symptoms"]:
                if sym["symptom"] != "INST":
                    continue
                n_cell += 1
                csr = _cell_values(s, sym["phase"])["csr"]
                n_cell_rear += _fin(csr) and csr < -thr["boundary"]["r"]
        for cid, a in agg.items():
            for sym in phase_symptoms(a, thr)["symptoms"]:
                if sym["symptom"] == "INST":
                    csr = _cell_values(a, sym["phase"])["csr"]
                    inst_rear += _fin(csr) and csr < -thr["boundary"]["r"]
        rec = dl["session_recommendation"]
        r = rec.get("recommended")
        rec_key = ({"levers": r["levers"], "direction": r["direction"], "net": r["net"]} if r else
                   {"tie": [[o["levers"], o["direction"]] for o in rec.get("equivalent_options") or []],
                    "net": (rec.get("equivalent_options") or [{}])[0].get("net")}
                   if rec.get("equivalent_options") else None)
        return {"verdicts": verdicts, "inst_units": inst_units, "inst_units_rear": inst_rear,
                "counted": counted, "cells": cell_shares(fin["summaries"], thr)["INST"],
                "inst_cells": n_cell, "inst_cells_rear": n_cell_rear, "rec": rec_key}
    finally:
        _restore_thr(orig)


def report(m_override=None, band_override=None):
    L = ["INST UNCERTAINTY MARGIN (ORDER INST-1, 2026-11-07)",
         "unit = corner x phase of the session aggregate (median over analysis laps of the per-lap phase median",
         "of dMz/dbeta, N m/deg); obs = max(|d| Q x 0.3, |d| R_yaw x 10); veh = max |d| over the ten +/-1 %",
         "vehicle runs; comb = sqrt(obs^2 + veh^2); m_INST = pooled p90 of comb (14 development sessions)", ""]
    res = {}
    for pop, title in (("agg", "ANALYSIS LAPS (classifier population)"), ("agg_all", "ALL LAPS (recorded CS veh population, comparison)")):
        rows, seg, stat = _units(pop)
        sess = sorted({r["session"] for r in rows})
        L.append(f"{title}: {len(sess)} sessions, {len(rows)} units with a baseline value")
        if pop == "agg":
            L.append(f"  segmentation differs in: {', '.join(seg) if seg else 'none'}")
            L.append(f"  status / gate / fallback changed: {'; '.join(stat) if stat else 'none'}")
        pr = {}
        for name in ("obs", "veh_max", "veh_rss", "comb_max", "comb_rss"):
            s = _pct([r[name] for r in rows])
            pr[name] = s
            L.append(f"  {name:9s} {_fmt(s)}")
        m = pr["comb_max"]["p90"]
        pr["m_inst"] = m
        L.append(f"  m_INST (p90 comb_max) = {m:.2f} N m/deg")
        if pop == "agg":
            L.append("  per perturbation |d| (N m/deg):")
            for v in VARIANTS[1:]:
                s = _pct([r["d"][v] for r in rows])
                pr[v] = s
                L.append(f"    {v:28s} {_fmt(s)}")
            n_both = [r for r in rows if _fin(r["obs"]) and _fin(r["veh_max"])]
            obs_dom = sum(r["obs"] > r["veh_max"] for r in n_both)
            q_dom = sum(r["d"]["q_x0.3"] >= r["d"]["ryaw_x10"] for r in rows
                        if _fin(r["d"]["q_x0.3"]) and _fin(r["d"]["ryaw_x10"]))
            top_inp = {}
            for r in rows:
                if r["per_inp"]:
                    k = max(r["per_inp"], key=r["per_inp"].get)
                    top_inp[k] = top_inp.get(k, 0) + 1
            L.append(f"  dominance: obs > veh in {obs_dom} / {len(n_both)} units; Q x 0.3 >= R_yaw x 10 in "
                     f"{q_dom} units; largest vehicle input per unit: "
                     + ", ".join(f"{k} {n}" for k, n in sorted(top_inp.items(), key=lambda kv: -kv[1])))
            base_inst = [r for r in rows if r["stab"] < 0]
            within = [r for r in base_inst if r["stab"] >= -m]
            L.append(f"  baseline INST units (stab < 0): {len(base_inst)}; within [-m_INST, 0): {len(within)}")
            # cs_margin analog
            vb = pr["veh_max"]["p90"]
            pr["veh_band_p90"] = vb
            L.append(f"  VEHICLE-INPUT BAND (cs_margin analog): p90 veh_max = {vb:.2f} N m/deg")
            for bname, bnd in (("-m_INST", -m), ("0 (current, information)", 0.0)):
                fl = _flips(rows, bnd)
                ds = [f["distance"] for f in fl]
                L.append(f"    flips at boundary {bname}: {len(fl)} units; distances descending: "
                         + ", ".join(f"{x:.2f}" for x in ds[:12]) + (" ..." if len(ds) > 12 else ""))
                L.append(f"      2nd-most-marginal flip distance: {ds[1]:.2f}" if len(ds) > 1 else
                         "      fewer than two flips")
                for f in fl[:6]:
                    L.append(f"      {f['session']} C{f['cp']} stab {f['stab']:+.2f}: {', '.join(f['runs'])}")
                pr[f"flips_{bname}"] = fl
                if bname == "-m_INST":
                    # ruling 2026-11-07: band = 2nd-most-marginal flip distance,
                    # rounded UP to 1 N m/deg (stated choice)
                    pr["band_flip"] = float(np.ceil(ds[1])) if len(ds) > 1 else float("nan")
                    L.append(f"      BAND (headline) = {pr['band_flip']:.0f} N m/deg (2nd distance rounded up)")
            # information: Q x 0.3 acts as a scale on the slope estimate, not as
            # noise around zero, so the pooled p90 is set by the large values;
            # the units near the sign boundary are listed beside it
            L.append("  information, Q x 0.3 as a scale: median of (Q-run value / baseline) over units with "
                     "|baseline| > 50: " + f"{np.median([_q_val(r) / r['stab'] for r in rows if abs(r['stab']) > 50 and _fin(_q_val(r))]):.2f}")
            for lo, hi in ((0, 100), (100, 300), (300, 1e9)):
                sub = [r for r in rows if lo <= abs(r["stab"]) < hi]
                s_ = _pct([r["comb_max"] for r in sub])
                L.append(f"    |baseline| in [{lo:.0f}, {hi:.0f}): comb_max {_fmt(s_)}")
            L.append("  baseline INST units (aggregate < 0): value -> Q x 0.3 value, obs, veh_max:")
            for r in sorted(base_inst, key=lambda r: r["stab"]):
                L.append(f"    {r['session']} C{r['cp']}: {r['stab']:+.1f} -> {_q_val(r):+.1f}, obs {r['obs']:.1f}, "
                         f"veh {r['veh_max']:.1f}")
            L.append("  information: per-session p90 comb_max " + "; ".join(
                f"{k} {_pct([r['comb_max'] for r in rows if r['session'] == k])['p90']:.1f}" for k in sess))
            res["rows"] = rows
        res[pop] = pr
        L.append("")
    L.append("CHAIN CHECK (CS boundary margins by the recorded rule from these runs):")
    _cs_check(L)
    L.append("")
    ma, mb = res["agg"]["m_inst"], res["agg_all"]["m_inst"]
    L.append(f"POPULATION: m_INST analysis laps {ma:.2f} vs all laps {mb:.2f} N m/deg "
             f"({100 * (mb - ma) / ma:+.0f} %)")
    L.append("")
    m = m_override if m_override is not None else ma
    band = band_override if band_override is not None else res["agg"]["band_flip"]
    res["final"] = _report_final(L, m, band, res["agg"]["veh_band_p90"])
    text = "\n".join(L)
    with open(os.path.join(OUT, "analysis.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "analysis.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


def _report_final(L, m, band, band_p90):
    names = [n for n in FINAL if os.path.exists(os.path.join(OUT, "final", f"{n}.pkl"))]
    if not names:
        L.append("FINAL SESSIONS: no runs")
        return {}
    # A (headline, ruling 2026-11-07): INST counts below -m; values within
    #    the band beyond -m are counted and tagged (CS semantics of the code)
    # A_p90: as A with the p90-of-veh band (tags only differ; comparison)
    # B: the order's original wording, band not counted (comparison)
    variants = {"current": (0.0, None), "A": (-m, (-m - band, -m)), "A_p90": (-m, (-m - band_p90, -m)),
                "B": (-m - band, (-m - band, 0.0))}
    L.append(f"FINAL SESSIONS (ten, real config, stored setup sheets), m_INST {m:.2f}, band {band:.0f} "
             f"(flip rule), comparison band {band_p90:.2f} (p90 veh) N m/deg")
    L.append("  A (headline) = INST if dMz/dbeta < -m (counted, votes); [MARGINAL] tag in [-m - band, -m)")
    L.append("  A_p90 = as A, tag band p90 of veh (comparison); B = INST only below -m - band, band no vote (comparison)")
    out = {}
    tot_v = 0
    for n in names:
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        ev = {k: _final_eval(fin, b, mb) for k, (b, mb) in variants.items()}
        out[n] = ev
        cur = ev["current"]
        tot_v += len(cur["verdicts"])
        ident = "identical" if cur["counted"] == fin["pipeline_counted"] else "DIFFERS"
        L.append(f"  {n} (outing {fin['outing']}, {fin['status']}/{fin['gate']}): {len(cur['verdicts'])} verdicts; "
                 f"in-memory current vs pipeline counted symptoms: {ident}")
        for k in ("current", "A", "A_p90", "B"):
            e = ev[k]
            ci = [c for c in e["counted"] if c[1] == "INST"]
            L.append(f"    {k:7s} INST units {e['inst_units']} (with rear limiting {e['inst_units_rear']}); "
                     f"INST cells {e['inst_cells']} (rear limiting {e['inst_cells_rear']}); counted INST "
                     f"{len(ci)} {[(c[0], c[2]) for c in ci]}; rec {e['rec']}")
        for k in ("A", "A_p90", "B"):
            e = ev[k]
            dv = {c: (cur["verdicts"][c], e["verdicts"][c]) for c in cur["verdicts"]
                  if cur["verdicts"][c] != e["verdicts"][c]}
            dc = sorted(set(cur["counted"]) ^ set(e["counted"]))
            L.append(f"    {k}: verdicts changed {len(dv)}; counted symptoms changed {len(dc)}; "
                     f"recommendation {'unchanged' if e['rec'] == cur['rec'] else 'CHANGED'}")
            for c, (x, y) in sorted(dv.items(), key=lambda kv: str(kv[0])):
                L.append(f"      C{c}: {x}  ->  {y}")
            for c in dc:
                L.append(f"      counted {'only before' if c in cur['counted'] else 'only after'}: C{c[0]} {c[1]} "
                         f"{c[2]} ({c[3]} laps)")
    L.append(f"  total verdicts {tot_v}")
    # named cases of the order
    from modules.classifier import aggregate_by_corner
    for n, cid in (("PRC3", 16), ("PRC5", 7), ("NBR1", 8), ("NBR1", 11), ("NBR5", 2), ("NBR5", 9)):
        if n not in out:
            continue
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        a = aggregate_by_corner(fin["summaries"]).get(cid)
        vals = {ph: a["phases"][ph][STAB]["median"] for ph in PHASES if a and ph in a["phases"]}
        neg = {ph: round(v, 1) for ph, v in vals.items() if _fin(v) and v < 0}
        st = {k: [c for c in out[n][k]["counted"] if c[0] == cid and c[1] == "INST"] for k in ("current", "A", "B")}
        L.append(f"  case {n} C{cid}: negative aggregate dMz/dbeta {neg}; counted INST current {len(st['current'])}, "
                 f"A {len(st['A'])}, B {len(st['B'])}")
    return out


def _pool(fn, tasks, workers):
    keep_awake(True)
    try:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            futs = {ex.submit(fn, t): t for t in tasks}
            for i, fut in enumerate(as_completed(futs), 1):
                try:
                    print(f"[{i}/{len(tasks)}] {fut.result()}", flush=True)
                except Exception as e:
                    print(f"[{i}/{len(tasks)}] {futs[fut]} FAILED: {e!r}", flush=True)
    finally:
        keep_awake(False)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    if mode == "dev":
        tasks = [(k, v) for k in POOL for v in VARIANTS
                 if not os.path.exists(os.path.join(OUT, "dev", f"{k}__{v}.pkl"))]
        _pool(run_dev, tasks, WORKERS)
    elif mode == "final":
        tasks = [n for n in FINAL if not os.path.exists(os.path.join(OUT, "final", f"{n}.pkl"))]
        _pool(run_final, tasks, min(WORKERS, len(tasks) or 1))
    else:
        args = [float(x) for x in sys.argv[2:4]]
        report(*args)


if __name__ == "__main__":
    main()
