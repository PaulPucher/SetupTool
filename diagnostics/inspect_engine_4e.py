# Setup engine before / after work order 4e (author EXECUTE 2026-10-23),
# 14 development sessions. Report only, no config change.
#
# Stage "chain" (run BEFORE the 4e engine edits): the frozen default chain
# per session (the golden generator's chain, fit L, best available accuracy,
# no setup sheet, no feedback), then the engine as it stood (margin 0.01,
# effort-first ranking) -> the OLD data layer. The pickle keeps the valid-lap
# summaries and the TC map, so the stage "report" re-runs only the engine:
# the chain is frozen, so old and new see identical summaries.
# Stage "report" (after the edits): NEW data layer, and the new code with
# the old rank key re-installed, which isolates the ranking change from the
# margin change. Also the log_abs_pos census (presence, values, per valid
# lap) and the per-lap session-layer shares.
# Output diagnostics/results_engine_4e_20261023/ (pickles, analysis.txt).
# Run: python -m diagnostics.inspect_engine_4e chain | report

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

from diagnostics.inspect_cs_margin_l import CANDIDATES, keep_awake  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_engine_4e_20261023")
WORKERS = 3


def _abs_census(ch, laps, valid):
    if ch is None:
        return {"present": False}
    t, d = np.asarray(ch["time"]), np.asarray(ch["data"], float)
    fin = np.isfinite(d)
    vals, counts = np.unique(np.round(d[fin]), return_counts=True)
    per_lap = {}
    for lap in laps:
        if lap["lap_number"] not in valid:
            continue
        m = (t >= lap["start_time"]) & (t <= lap["end_time"]) & fin
        u = np.unique(np.round(d[m]))
        per_lap[lap["lap_number"]] = [int(x) for x in u]
    return {"present": True, "quality": ch.get("quality"), "n": int(fin.sum()),
            "values": {int(v): int(c) for v, c in zip(vals, counts)}, "per_valid_lap": per_lap}


def run_chain(key):
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, load_car_data, prepare_vehicle_state,
                                            estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            estimate_vertical_loads, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs
    from modules.setup_engine import tc_activity, data_layer
    t0 = time.perf_counter()
    _, _track, _outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    params = load_parameters()
    p = apply_resolved_vehicle(params, resolve_accuracy(params, setup_data=None, cap=None))
    data = parse_csv(path)
    corners = data.get("corners", [])
    reclassify_speed_classes(corners, load_channels_config()["corner_speed_thresholds"])
    st = prepare_vehicle_state(data["channels"], p)
    beta, man, gate, fb, _fr, cs_in = resolve_sideslip_beta_and_cs_inputs(
        st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    fz = estimate_vertical_loads(st, forces, p, channels=data["channels"], car_data=load_car_data())
    summaries = summarise_corners(corners, cs, stab, st, fz=fz, lap_filter=None)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    vsum = [s for s in summaries if s["lap_number"] in valid]
    tc_map = tc_activity(corners, st, data["channels"])
    out = {
        "key": key, "label": label, "gate": gate and gate.get("verdict"), "fallback": fb,
        "valid_laps": sorted(valid), "summaries": vsum, "tc_map": tc_map,
        "abs": _abs_census(data["channels"].get("log_abs_pos"), laps, valid),
        "old": data_layer(vsum, tc_map=tc_map), "elapsed_s": time.perf_counter() - t0,
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    return f"{key}: {out['elapsed_s']:.0f} s, gate {out['gate']}, fallback {fb}"


def _old_rank_key(c):
    # the pre-4e order: severity -> repetition -> demotion -> effort -> preference -> headroom
    from modules.setup_engine import EFFORT_RANK
    from modules.corner_aggregation import SEVERITY_RANK
    return (-SEVERITY_RANK[c["severity"]], -c["laps"], c["condition_demoted"],
            EFFORT_RANK.get(c["change"]["effort"], len(EFFORT_RANK)), -c["preference"],
            -(c["change"]["headroom"] if c["change"]["headroom"] is not None else 0.0))


def _rec_map(layer):
    out = {}
    for c in layer["corners"]:
        r = c.get("recommended")
        out[c["corner"]] = (None if r is None else
                            (f"{r['symptom']}@{r['phase']}", "+".join(r["levers"]), r["direction"], r["mechanism_id"]))
    return out


def _fmt(r):
    return "none" if r is None else f"{r[1]} {r[2]} ({r[0]}, {r[3]})"


def _diff(a, b):
    lines = []
    for cid in sorted(set(a) | set(b)):
        ra, rb = a.get(cid), b.get(cid)
        if ra != rb:
            lines.append(f"    C{cid}: {_fmt(ra)} -> {_fmt(rb)}")
    return lines


def report():
    import modules.setup_engine as se
    keys = [c[0] for c in CANDIDATES]
    L = ["SETUP ENGINE BEFORE / AFTER WORK ORDER 4e (2026-10-23), 14 development sessions",
         "old = engine before 4e (margin 0.01, effort before preference); new = 4e (margins front / rear "
         "per parameters.json, preference before effort, INST agreement, supported preference cells).",
         "rank-only = new code with the pre-4e rank key: isolates the ranking change.", ""]
    pooled = {"US": [], "OS": [], "INST": [], "counted": []}
    n_total_change = n_rank_change = n_old = n_new = 0
    abs_lines = []
    for key in keys:
        fn = os.path.join(OUT, f"{key}.pkl")
        if not os.path.exists(fn):
            L.append(f"{key}: MISSING")
            continue
        d = pickle.load(open(fn, "rb"))
        new = se.data_layer(d["summaries"], tc_map=d["tc_map"])
        saved = se._data_rank_key
        se._data_rank_key = _old_rank_key
        try:
            rank_old = se.data_layer(d["summaries"], tc_map=d["tc_map"])
        finally:
            se._data_rank_key = saved
        mo, mn, mr = _rec_map(d["old"]), _rec_map(new), _rec_map(rank_old)
        n_old += sum(v is not None for v in mo.values())
        n_new += sum(v is not None for v in mn.values())
        tot, rk = _diff(mo, mn), _diff(mr, mn)
        n_total_change += len(tot)
        n_rank_change += len(rk)
        ses = new["session"]
        cs_ = ses["cells"]
        L.append(f"{d['label']} ({key}), valid laps {d['valid_laps']}, gate {d['gate']}")
        L.append(f"  per-lap cells (no majority rule): front limiting {cs_['US']['k']}/{cs_['US']['n']} "
                 f"= {cs_['US']['share']:.3f}; rear limiting {cs_['OS']['k']}/{cs_['OS']['n']} = "
                 f"{cs_['OS']['share']:.3f}; INST {cs_['INST']['k']}/{cs_['INST']['n']} = {cs_['INST']['share']:.3f}")
        L.append(f"  corners with a counted symptom: {ses['corners_counted']}/{ses['n_corners']} = "
                 f"{ses['counted_share']:.3f}; per symptom " +
                 ", ".join(f"{k} {v['corners']}" for k, v in ses["shares"].items()))
        for k in ("US", "OS", "INST"):
            pooled[k].append(cs_[k]["share"])
        pooled["counted"].append(ses["counted_share"])
        L.append(f"  recommendations old {sum(v is not None for v in mo.values())}, new "
                 f"{sum(v is not None for v in mn.values())}; changed old -> new: {len(tot)}")
        L += tot
        L.append(f"  of which the ranking order alone (rank-only -> new): {len(rk)}")
        L += rk
        a = d["abs"]
        if not a["present"]:
            abs_lines.append(f"  {key}: log_abs_pos ABSENT")
        else:
            changes = [ln for ln, v in a["per_valid_lap"].items() if len(v) != 1]
            abs_lines.append(f"  {key}: present, quality {a['quality']}, values {a['values']}, per valid lap "
                             f"{a['per_valid_lap']}, laps with a change {changes or 'none'}")
        L.append("")
    L.append("POOLED over sessions (per-session shares)")
    for k, v in pooled.items():
        v = np.asarray([x for x in v if x is not None and x == x], float)
        if v.size:
            L.append(f"  {k}: p50 {np.percentile(v, 50):.3f}, p90 {np.percentile(v, 90):.3f} (n sessions {v.size})")
    L.append(f"TOTAL recommendations old {n_old}, new {n_new}; changed old -> new {n_total_change}; "
             f"ranking-order alone {n_rank_change}")
    L.append("")
    L.append("log_abs_pos CENSUS (read from each file by parse_csv)")
    L += abs_lines
    txt = "\n".join(L)
    with open(os.path.join(OUT, "analysis.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    if mode == "report":
        return report()
    keep_awake(True)
    try:
        keys = [c[0] for c in CANDIDATES if not os.path.exists(os.path.join(OUT, f"{c[0]}.pkl"))]
        with ProcessPoolExecutor(WORKERS) as ex:
            for f in as_completed([ex.submit(run_chain, k) for k in keys]):
                print(f.result(), flush=True)
    finally:
        keep_awake(False)


if __name__ == "__main__":
    main()
