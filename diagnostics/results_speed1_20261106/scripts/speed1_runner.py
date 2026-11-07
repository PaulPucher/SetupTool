# ORDER SPEED-1 Parts 2 and 3: chain runs on a scratchpad copy of the working
# repo (its own config variant and DB copy, read-only), through the
# thesis-figure helpers load_session / run_chain, as SPEED-0b did.
# Run: python speed1_runner.py <copy root> <variant> <out dir> [workers]
import json
import os
import sys
import time

import numpy as np

OUTINGS = {"NBR1": 17, "NBR2": 18, "NBR3": 19, "NBR4": 20, "NBR5": 21,
           "PRC1": 22, "PRC2": 25, "PRC3": 26, "PRC4": 27, "PRC5": 28}


def _imports(root):
    if sys.path[0] != root:
        sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "tools", "thesis_figures"))
    import common  # noqa  (chdir to the copy)
    import modules
    first = os.path.normcase(os.path.dirname(list(modules.__path__)[0]))
    assert first == os.path.normcase(os.path.normpath(root)), list(modules.__path__)
    import modules.stability_analysis as sa
    assert os.path.normcase(sa.__file__).startswith(os.path.normcase(os.path.normpath(root)))
    return common


def _rec_of(vote):
    rec = (vote or {}).get("recommended")
    if rec:
        return {"levers": rec["levers"], "direction": rec["direction"], "net": rec["net"]}
    tie = (vote or {}).get("equivalent_options") or []
    if tie:
        return {"tie": [[o["levers"], o["direction"]] for o in tie], "net": tie[0].get("net")}
    return None


def ay_vs_vr(st):
    # SPEED-0b census rule, unchanged
    sr = st["sample_rate_hz"]
    mov = st["moving_mask"]
    n_avg = max(1, int(0.1 * sr))
    sw = np.degrees(st["steer_sw_rad"])
    rate = np.gradient(np.convolve(sw, np.ones(n_avg) / n_avg, mode="same"), 1.0 / sr)
    vr = st["v_mps"] * st["yaw_rate_radps"]
    kerb = np.zeros_like(mov)
    for k in ("kerb_mask", "kerb_smear_zone"):
        if st.get(k) is not None:
            kerb |= st[k]
    qs = mov & ~kerb & (np.abs(st["ay_mps2"]) > 5.0) & (np.abs(rate) < 10.0) & (np.abs(vr) > 3.0)
    ratio = st["ay_mps2"][qs] / vr[qs]
    return {"n": int(qs.sum()), "median": float(np.median(ratio)), "p10": float(np.percentile(ratio, 10)),
            "p90": float(np.percentile(ratio, 90))}


def session(job):
    root, key, out_dir = job
    common = _imports(root)
    from modules import pipeline
    from modules.classifier import aggregate_by_corner, classify_corner
    t0 = time.time()
    sess = common.load_session(OUTINGS[key])
    ch = common.run_chain(sess)
    row = sess["row"]
    layers = pipeline.recommend(ch["summaries"], setup_data=json.loads(row["setup_data"] or "null"),
                                feedback_data=json.loads(row["feedback_data"] or "null"),
                                corners=ch["corners"], state=ch["state"],
                                channels=sess["parsed"].get("channels"), laps=sess["parsed"].get("laps"))
    summ = pipeline._session_summary(sess["path"], dict(ch, sideslip_source=sess["ident"]["sideslip_source"]),
                                     ch["summaries"], layers)
    summ["session_recommendation_full"] = _rec_of(layers["data_layer"].get("session_recommendation"))
    man = ch["fit_manifest"] or {}
    summ["braking_excluded"] = (man.get("axle_load") or {}).get("braking_excluded")
    summ["signal_corrections"] = man.get("signal_corrections")
    summ["parsed_signal_corrections"] = sess["parsed"].get("signal_corrections")
    summ["n_corners_parsed"] = len(sess["parsed"].get("corners") or [])
    summ["ay_vs_vr"] = ay_vs_vr(ch["state"])
    # stored payload (schema 30), read without the app's schema guard
    stored = json.loads(row["analysis_data"] or "null")
    if stored:
        sv = {str(cid): classify_corner(agg)[1] for cid, agg in aggregate_by_corner(stored["summaries"]).items()}
        nv = {str(c["corner"]): c["verdict"] for c in summ["corner_verdicts"]}
        dl = (stored.get("setup_engine") or {}).get("data_layer") or {}
        srec = _rec_of(dl.get("session_recommendation"))
        scount = [{"corner": c["corner"], "symptom": s["symptom"], "phase": s["phase"], "laps": s["laps"]}
                  for c in dl.get("corners") or [] for s in c["symptoms"]]
        summ["stored_identity"] = {
            "schema": stored.get("schema_version"), "n_verdicts": len(nv),
            "verdicts_equal": sv == nv,
            "verdict_diffs": {k: (sv.get(k), nv.get(k)) for k in set(sv) | set(nv) if sv.get(k) != nv.get(k)},
            "recommendation_equal": srec == summ["session_recommendation_full"],
            "stored_recommendation": srec,
            "counted_equal": scount == summ["counted_symptoms"]}
    summ["runtime_s"] = time.time() - t0
    with open(os.path.join(out_dir, f"{key}.json"), "w", encoding="utf-8") as fh:
        json.dump(summ, fh, indent=1, default=float)
    return key, summ["runtime_s"]


if __name__ == "__main__":
    root, variant, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    workers = int(sys.argv[4]) if len(sys.argv) > 4 else 5
    os.makedirs(out_dir, exist_ok=True)
    from concurrent.futures import ProcessPoolExecutor, as_completed
    with ProcessPoolExecutor(workers) as ex:
        futs = [ex.submit(session, (root, k, out_dir)) for k in OUTINGS]
        for f in as_completed(futs):
            k, dt = f.result()
            print(f"{variant} {k} done ({dt:.0f} s)", flush=True)
