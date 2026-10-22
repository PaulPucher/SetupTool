# Close-out checks on the fit-L default (author EXECUTE 2026-10-20):
# B  NIS gate thresholds, the recorded rule (nis_gate.threshold_derived_from,
#    2026-09-03 / 2026-10-11) unchanged: threshold_use_ekf = a round number
#    about 6 % below the lowest real health score of the pool, threshold_warn
#    about an order of magnitude below that lowest score. Pool = the 14
#    development sessions, scores fresh from the app's chain.
# C  R26 remainder, the recorded test (spike follow-up 2026-10-17): per axle
#    the verdict-read CS samples whose reference in force lies below 0.5 x the
#    session median of the set-event references; with the number of such
#    references, their median hold, their values relative to the median, the
#    most negative CS they produce and the classifier phase medians that move
#    when those samples are removed. Set events = the in-band reference
#    candidates; the reference series is forward-filled before change
#    detection (NaN gaps are not set events).
# Verdict-read = what summarise_corners reads: moving, inside a phase of a
#    corner on an analysis lap, finite CS, not kerb-affected, not
#    observer-reset-affected.
# Output diagnostics/plots/closeout_l/ (cache/, closeout.txt / .json).
# Run: python -m diagnostics.inspect_closeout_l [report]

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

# Development-set file list, the 14 entries of the retired hub
# inspect_data_quality.py (inlined in the 2026-10-22 pruning, unchanged):
# (key, track, outing, label, ("rel", path under the data root)).
_NBR = "unlabeled/02_NBR_CRV_data"
CANDIDATES = (
    ("nbr_o05_mla", "NBR", 5, "NBR outing 5 MLA", ("rel", f"{_NBR}/01_SE_01/NBR-73-005-001-MLA-26.07.01.0919.txt")),
    ("nbr_o10_jdr", "NBR", 10, "NBR outing 10 JDR", ("rel", f"{_NBR}/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt")),
    ("nbr_o20_jst", "NBR", 20, "NBR outing 20 JST", ("rel", f"{_NBR}/01_SE_02/NBR-73-020-065-JST-26.07.01.1454.txt")),
    ("nbr_o29_jst", "NBR", 29, "NBR outing 29 JST (NBR_Sturm_Solution copy)",
     ("rel", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")),
    ("nbr_o46_jst", "NBR", 46, "NBR outing 46 JST", ("rel", f"{_NBR}/01_TE_04/NBR-73-046-162-JST-26.07.02.1411.txt")),
    ("nbr_o53_mla", "NBR", 53, "NBR outing 53 MLA", ("rel", f"{_NBR}/01_TE_04/NBR-73-053-207-MLA-26.07.02.1707.txt")),
    ("nbr_o54_cdr", "NBR", 54, "NBR outing 54 CDR", ("rel", f"{_NBR}/01_TE_04/NBR-73-054-214-CDR -26.07.02.1727.txt")),
    ("nbr_o59_mla", "NBR", 59, "NBR outing 59 MLA", ("rel", f"{_NBR}/New folder/NBR-73-059-232-MLA-26.07.03.1210.txt")),
    ("nbr_o61_jdr", "NBR", 61, "NBR outing 61 JDR", ("rel", f"{_NBR}/New folder/NBR-73-061-238-JDR-26.07.03.1229.txt")),
    ("nbr_o63_cdr", "NBR", 63, "NBR outing 63 CDR", ("rel", f"{_NBR}/New folder/NBR-73-063-256-CDR-26.07.03.1309.txt")),
    ("nbr_qp_cdr", "NBR", 69, "NBR qualifying CDR (outing 69)", ("rel", "NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt")),
    ("nbr_qp_mla", "NBR", 70, "NBR qualifying MLA (outing 70)", ("rel", "NBR_Qualy_MLA/MLA-QP01-NBR-v2.txt")),
    ("pr_o25_us_v1", "PR", 25, "Le Castellet outing 25, MLA-SE07_US-v1", ("rel", "Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt")),
    ("pr_o32_us_v2", "PR", 32, "Le Castellet outing 32, MLA-SE07_US-v2", ("rel", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt")),
)


def keep_awake(on):
    # multi-hour runs: stop Windows from sleeping (from the retired hub
    # inspect_fit_lit.py, inlined 2026-10-22)
    if os.name == "nt":
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | (0x00000001 if on else 0))

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "closeout_l")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
WORKERS = 3
SMALL = 0.5   # recorded R26 condition: reference below 0.5 x the session median


def _ffill(x):
    x = np.asarray(x, float).copy()
    idx = np.where(np.isfinite(x), np.arange(x.size), 0)
    np.maximum.accumulate(idx, out=idx)
    y = x[idx]
    y[: np.argmax(np.isfinite(x))] = np.nan if np.isfinite(x).any() else np.nan
    return y


def _phase_medians(summ, valid):
    out = {}
    for s in summ:
        if s["lap_number"] not in valid:
            continue
        for ph, p in s["phases"].items():
            for ax in "fr":
                out[(s["stable_corner_id"], s["lap_number"], ph, ax)] = p[f"cs_ratio_{ax}"]["median"]
        if s.get("apex_region") is not None:
            for ax in "fr":
                out[(s["stable_corner_id"], s["lap_number"], "apex_region", ax)] = s["apex_region"][f"cs_ratio_{ax}"]["median"]
    return out


def compute(key):
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    beta, man, gate, fb, fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
        st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(st, p), st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    t = st["time"]
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    # verdict-read population: moving, inside a phase window of an analysis-lap corner
    in_phase = np.zeros(t.size, dtype=bool)
    for c in data.get("corners", []):
        if c.get("lap_number") in valid and c.get("stable_corner_id") is not None:
            for a, b in c["segments"].values():
                if b >= a:
                    in_phase |= (t >= a) & (t <= b)
    summ = summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
    base_med = _phase_medians(summ, valid)
    out = {"key": key, "label": label, "status": man.get("status"), "fallback": fb,
           "gate": gate and {k: gate[k] for k in ("verdict", "health_score")}, "axles": {}}
    cs_cut = dict(cs)
    for ax in "fr":
        ratio = np.asarray(cs[f"CS_ratio_{ax}"], float)
        ref = _ffill(cs[f"C_linear_ref_{ax}"])
        rc = np.asarray(cs[f"ref_candidates_{ax}"], float)
        lo, hi = tfa.L_K_EDGE
        sets = rc[(rc >= lo) & (rc <= hi)]
        med = float(np.median(sets)) if sets.size else float("nan")
        read = (st["moving_mask"] & in_phase & np.isfinite(ratio) & ~np.asarray(cs[f"near_strike_{ax}"], bool))
        if f"observer_reset_affected_{ax}" in cs:
            read &= ~np.asarray(cs[f"observer_reset_affected_{ax}"], bool)
        small = read & np.isfinite(ref) & (ref < SMALL * med)
        # distinct small references in force at verdict-read samples, and their holds
        change = np.concatenate(([True], ref[1:] != ref[:-1]))
        run_id = np.cumsum(change)
        runs = np.unique(run_id[small])
        holds = [float(t[run_id == r][-1] - t[run_id == r][0]) for r in runs]
        vals = sorted({float(ref[run_id == r][0]) for r in runs})
        c2 = ratio.copy()
        c2[small] = np.nan
        cs_cut[f"CS_ratio_{ax}"] = c2
        out["axles"][ax] = {
            "n_set_events": int(sets.size), "median_ref": med, "band": [lo, hi],
            "n_verdict_read": int(read.sum()), "n_small": int(small.sum()), "n_small_refs": int(len(runs)),
            "small_ref_values": vals, "small_ref_rel": [v / med for v in vals],
            "median_hold_s": float(np.median(holds)) if holds else float("nan"),
            "min_cs_small": float(np.min(ratio[small])) if small.any() else float("nan"),
            "set_rel_p1": float(np.percentile(sets, 1) / med) if sets.size else float("nan"),
            "set_rel_min": float(np.min(sets) / med) if sets.size else float("nan")}
    summ2 = summarise_corners(data.get("corners", []), cs_cut, stab, st, lap_filter=None)
    cut_med = _phase_medians(summ2, valid)
    for ax in "fr":
        moved = sum(1 for k, v in base_med.items() if k[3] == ax and
                    not (v == cut_med.get(k) or (v != v and cut_med.get(k) != cut_med.get(k))))
        out["axles"][ax]["medians_moved"] = moved
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] gate {out['gate']}, small refs f {out['axles']['f']['n_small_refs']} / r "
          f"{out['axles']['r']['n_small_refs']}, {time.perf_counter() - t0:.0f} s", flush=True)
    return key


def report():
    from modules.stability_analysis import load_parameters
    g = load_parameters()["nis_gate"]
    res = {k: pickle.load(open(os.path.join(CACHE, f"{k}.pkl"), "rb")) for k in POOL}
    scores = {k: res[k]["gate"]["health_score"] for k in POOL}
    low_k = min(scores, key=scores.get)
    low = scores[low_k]
    L = ["CLOSE-OUT CHECKS on the fit-L default", "",
         f"B. NIS GATE -- recorded rule unchanged; current threshold_use_ekf {g['threshold_use_ekf']}, "
         f"threshold_warn {g['threshold_warn']}",
         f"  health scores: " + ", ".join(f"{res[k]['label']} {scores[k]:.4f}" for k in POOL),
         f"  lowest: {res[low_k]['label']} {low:.4f}; 6 % below = {0.94 * low:.4f}; one order of magnitude below = "
         f"{low / 10:.4f}", ""]
    L.append("C. R26 REMAINDER (reference in force < 0.5 x session median of set events, verdict-read samples):")
    tot = {"f": 0, "r": 0}
    for k in POOL:
        r = res[k]
        parts = []
        for ax, n in (("f", "front"), ("r", "rear")):
            a = r["axles"][ax]
            tot[ax] += a["n_small"]
            parts.append(f"{n} median ref {a['median_ref']:.2f}/rad, set events {a['n_set_events']}, lowest set "
                         f"{a['set_rel_min']:.2f} x median; small: {a['n_small']} of {a['n_verdict_read']} samples from "
                         f"{a['n_small_refs']} refs" + (f" ({', '.join(f'{v:.2f}' for v in a['small_ref_rel'][:6])} x "
                                                        f"median, hold median {a['median_hold_s']:.2f} s, min CS "
                                                        f"{a['min_cs_small']:+.2f})" if a['n_small'] else "")
                         + f", phase medians moved {a['medians_moved']}")
        L.append(f"  {r['label']} (status {r['status']}, gate {r['gate']['verdict']}): " + "; ".join(parts))
    L.append(f"  TOTAL small-reference verdict-read samples: front {tot['f']}, rear {tot['r']}")
    text = "\n".join(L)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "closeout.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, "closeout.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
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
