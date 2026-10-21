# R26 source fix on the fit-L default (author EXECUTE 2026-10-20): the linear
# reference accepts a band candidate only within L_REF_RELATIVE = [0.5, 2] x
# the session median of that axle's band candidates. Per session one chain
# run; CS computed twice on the same beta, slip and loads -- BEFORE without
# the relative gate (as shipped before the fix), AFTER with it (the app's
# cs_kwargs). Report per session and axle: candidates rejected low / high,
# the R26 recorded test count after the fix (verdict-read samples under a
# reference in force below 0.5 x the median of the band candidates), the
# phase medians that move (analysis laps, apex region included), and the
# deepest corner value (worst lap, worst of five phases) before / after.
# Output diagnostics/plots/r26_fix_l/ (cache/, r26_fix.txt / .json).
# Run: python -m diagnostics.inspect_r26_fix_l [report]

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

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "r26_fix_l")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
WORKERS = 3


def r26_count(cs, ax, read, lo, hi):
    from diagnostics.inspect_closeout_l import _ffill
    rc = np.asarray(cs[f"ref_candidates_{ax}"], float)
    sets = rc[(rc >= lo) & (rc <= hi)]
    med = float(np.median(sets)) if sets.size else float("nan")
    ref = _ffill(cs[f"C_linear_ref_{ax}"])
    return int((read & np.isfinite(ref) & (ref < 0.5 * med)).sum())


def compute(key):
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from diagnostics.inspect_closeout_l import _phase_medians
    from diagnostics.inspect_threshold_rederivation import _corner_values
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.recommendation import aggregate_by_corner
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
    forces = estimate_lateral_forces(st, p)
    kw_after = dict(cs_in["cs_kwargs"])
    kw_before = {k: v for k, v in kw_after.items() if k != "ref_relative"}
    cs_b = estimate_cornering_stiffness(slip, forces, st, p, laps, **kw_before)
    cs_a = estimate_cornering_stiffness(slip, forces, st, p, laps, **kw_after)
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    t = st["time"]
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    in_phase = np.zeros(t.size, dtype=bool)
    for c in data.get("corners", []):
        if c.get("lap_number") in valid and c.get("stable_corner_id") is not None:
            for a, b in c["segments"].values():
                if b >= a:
                    in_phase |= (t >= a) & (t <= b)
    summ_b = summarise_corners(data.get("corners", []), cs_b, stab, st, lap_filter=None)
    summ_a = summarise_corners(data.get("corners", []), cs_a, stab, st, lap_filter=None)
    med_b, med_a = _phase_medians(summ_b, valid), _phase_medians(summ_a, valid)
    cv_b = _corner_values(aggregate_by_corner([x for x in summ_b if x["lap_number"] in valid]))
    cv_a = _corner_values(aggregate_by_corner([x for x in summ_a if x["lap_number"] in valid]))
    lo, hi = kw_after["ref_band"]
    out = {"key": key, "label": label, "status": man.get("status"), "gate": gate and gate.get("verdict"),
           "fallback": fb, "axles": {}}
    for ax in "fr":
        def read(cs):
            r = (st["moving_mask"] & in_phase & np.isfinite(np.asarray(cs[f"CS_ratio_{ax}"], float))
                 & ~np.asarray(cs[f"near_strike_{ax}"], bool))
            if f"observer_reset_affected_{ax}" in cs:
                r &= ~np.asarray(cs[f"observer_reset_affected_{ax}"], bool)
            return r
        moved = sum(1 for k, v in med_b.items() if k[3] == ax and
                    not (v == med_a.get(k) or (v != v and med_a.get(k) != med_a.get(k))))
        vb = np.array([v[ax] for v in cv_b.values()], float)
        va = np.array([v[ax] for v in cv_a.values()], float)
        out["axles"][ax] = {
            "rejected_low": cs_a[f"n_ref_relative_low_{ax}"], "rejected_high": cs_a[f"n_ref_relative_high_{ax}"],
            "median_ref": cs_a[f"ref_relative_median_{ax}"],
            "n_band_candidates": int(((cs_a[f"ref_candidates_{ax}"] >= lo) & (cs_a[f"ref_candidates_{ax}"] <= hi)).sum()),
            "r26_before": r26_count(cs_b, ax, read(cs_b), lo, hi), "r26_after": r26_count(cs_a, ax, read(cs_a), lo, hi),
            "medians_moved": moved,
            "n_medians": sum(1 for k in med_b if k[3] == ax),
            "deepest_before": float(np.nanmin(vb)) if np.isfinite(vb).any() else float("nan"),
            "deepest_after": float(np.nanmin(va)) if np.isfinite(va).any() else float("nan"),
            "no_ref_before": cs_b[f"n_no_reference_analysis_{ax}"], "no_ref_after": cs_a[f"n_no_reference_analysis_{ax}"]}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] {time.perf_counter() - t0:.0f} s", flush=True)
    return key


def report():
    res = {k: pickle.load(open(os.path.join(CACHE, f"{k}.pkl"), "rb")) for k in POOL}
    L = ["R26 SOURCE FIX on the fit-L default: reference candidates gated to [0.5, 2] x the session median", ""]
    tot = {"f": [0, 0, 0, 0], "r": [0, 0, 0, 0]}
    for k in POOL:
        r = res[k]
        parts = []
        for ax, n in (("f", "front"), ("r", "rear")):
            a = r["axles"][ax]
            tot[ax][0] += a["rejected_low"]
            tot[ax][1] += a["rejected_high"]
            tot[ax][2] += a["r26_after"]
            tot[ax][3] += a["medians_moved"]
            parts.append(f"{n}: median ref {a['median_ref']:.2f}/rad, rejected {a['rejected_low']} low / "
                         f"{a['rejected_high']} high of {a['n_band_candidates']} band candidates; R26 count "
                         f"{a['r26_before']} -> {a['r26_after']}; phase medians moved {a['medians_moved']} of "
                         f"{a['n_medians']}; deepest corner value {a['deepest_before']:+.3f} -> "
                         f"{a['deepest_after']:+.3f}; no-reference samples (analysis laps) {a['no_ref_before']} -> "
                         f"{a['no_ref_after']}")
        L.append(f"{r['label']} (status {r['status']}, gate {r['gate']}, fallback {r['fallback']}):")
        L += ["  " + x for x in parts]
    L.append("")
    L.append(f"TOTAL front: rejected {tot['f'][0]} low / {tot['f'][1]} high, R26 count after {tot['f'][2]}, "
             f"phase medians moved {tot['f'][3]}; rear: rejected {tot['r'][0]} / {tot['r'][1]}, R26 after "
             f"{tot['r'][2]}, medians moved {tot['r'][3]}")
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "r26_fix.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, "r26_fix.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=str)
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
