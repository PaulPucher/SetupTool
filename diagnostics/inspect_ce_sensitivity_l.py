# C / E sensitivity of fit L (author EXECUTE 2026-10-21), report only,
# nothing selected. The fit-L shape constants C (L_SHAPE_C) and E
# (L_CURVATURE_E) are set in memory per worker job and restored afterwards;
# B stays the one fitted parameter, mu = mu_demo as shipped; everything
# downstream (observer, CS with the R26 gate, verdicts at the adopted
# thresholds) runs as the app runs it.
# Grid: C {1.0, 1.3, 1.6} x E {-1, 0, 1}; baseline = C 1.0, E -1 (shipped).
# Per variant and session against the baseline: fitted K = B C mu per axle,
# fit residual RMS (N and normalised), D4-L zone, status, gate, fallback,
# hard-bound re-entries, corner-value shift p50 / p90 (worst lap, worst of
# five phases) and verdict-class changes (strong / moderate / none / no data
# at the adopted thresholds). Peak: the slip angle in 0..15 deg where the
# fitted curve's slope changes sign (none if it stays positive) -- computed
# here from B, C, E, because the manifest's peak note is fixed for C = 1.
# Output diagnostics/plots/ce_sensitivity_l/ (cache/, ce_sensitivity.txt /
# .json). Run: python -m diagnostics.inspect_ce_sensitivity_l [report]

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

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "ce_sensitivity_l")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
VARIANTS = [(c, e) for c in (1.0, 1.3, 1.6) for e in (-1.0, 0.0, 1.0)]
BASE = (1.0, -1.0)
WORKERS = 6
RANK = {"none": 0, "moderate": 1, "strong": 2}


def _tag(v):
    return f"C{v[0]:.1f}_E{v[1]:+.0f}"


def peak_deg(B, C, E):
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    g = np.linspace(0.01, 15.0, 3000)
    s = pacejka_lateral_stiffness(np.radians(g), B, C, 1.0, E)
    idx = np.where(np.diff(np.sign(s)) < 0)[0]
    return float(g[idx[0]]) if idx.size else None


def compute(job):
    key, (c_val, e_val) = job
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
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    saved = {"L_SHAPE_C": tfa.L_SHAPE_C, "L_CURVATURE_E": tfa.L_CURVATURE_E,
             "fit_session_pacejka": tfa.fit_session_pacejka}
    held = {}

    def fsp(*a, **kw):
        held["raw"] = saved["fit_session_pacejka"](*a, **kw)
        return held["raw"]
    try:
        tfa.L_SHAPE_C, tfa.L_CURVATURE_E = c_val, e_val
        tfa.fit_session_pacejka = fsp
        st = prepare_vehicle_state(data["channels"], p)
        beta, man, gate, fb, fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        for n, v in saved.items():
            setattr(tfa, n, v)
    raw = held.get("raw", {})
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(st, p), st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    summ = summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
    corners = _corner_values(aggregate_by_corner([x for x in summ if x["lap_number"] in valid]))
    hb = raw.get("observer_reset_mask")
    axles = {}
    for name in ("front", "rear"):
        a = (man or {}).get("axles", {}).get(name, {})
        B, C, mu = a.get("B", float("nan")), a.get("C", float("nan")), a.get("mu", float("nan"))
        axles[name] = {"B": B, "C": C, "E": a.get("E"), "mu": mu, "K": B * C * mu,
                       "rms_N": a.get("fit_rms_resid_N"), "rms_norm": a.get("fit_rms_resid_norm"),
                       "zone": (man or {}).get("k_band_zone", {}).get(name),
                       "peak_deg": peak_deg(B, C, e_val) if np.isfinite(B) else None}
    out = {"key": key, "label": label, "variant": (c_val, e_val), "status": man and man.get("status"),
           "reason": man and (man.get("degenerate_reason") or man.get("marginal_reason")),
           "gate": gate and gate.get("verdict"), "fallback": fb, "fallback_reason": fr,
           "n_hard_bound_reentries": (int(np.sum(np.diff(np.concatenate(([0], np.asarray(hb, np.int8)))) == 1))
                                      if hb is not None else None),
           "axles": axles, "corners": corners}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}__{_tag((c_val, e_val))}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key} {_tag((c_val, e_val))}] status {out['status']}, fallback {fb}, re-entries "
          f"{out['n_hard_bound_reentries']}, {time.perf_counter() - t0:.0f} s", flush=True)
    return job


def _cls(v, S, M):
    if v != v:
        return "no data"
    return "strong" if v < S else ("moderate" if v < M else "none")


def report():
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    thr = {"f": (cl["STRONG_CSF"]["value"], cl["MODERATE_CSF"]["value"]),
           "r": (cl["STRONG_CSR"]["value"], cl["MODERATE_CSR"]["value"])}
    res = {(k, v): pickle.load(open(os.path.join(CACHE, f"{k}__{_tag(v)}.pkl"), "rb")) for k in POOL for v in VARIANTS}
    L = ["C / E SENSITIVITY of fit L (B fitted, mu = mu_demo; observer, CS with the R26 gate, adopted thresholds) -- "
         "report only, nothing selected", ""]
    J = {}
    for v in VARIANTS:
        L.append(f"VARIANT C {v[0]:.1f}, E {v[1]:+.0f}" + ("  (baseline, shipped)" if v == BASE else ""))
        J[_tag(v)] = {}
        shifts = {"f": [], "r": []}
        flips = {"f": [0, 0], "r": [0, 0]}
        dirs = {"more severe": 0, "less severe": 0, "to/from no data": 0}
        for k in POOL:
            r, b = res[(k, v)], res[(k, BASE)]
            parts = []
            for ax, name in (("f", "front"), ("r", "rear")):
                a = r["axles"][name]
                pk = "none" if a["peak_deg"] is None else f"{a['peak_deg']:.1f} deg"
                parts.append(f"{name} K {a['K']:.2f} ({a['zone']}), rms {a['rms_N']:.0f} N / {a['rms_norm']:.4f}, "
                             f"peak {pk}")
                if v != BASE:
                    S, M = thr[ax]
                    for c in set(b["corners"]) | set(r["corners"]):
                        x0 = b["corners"].get(c, {}).get(ax, float("nan"))
                        x1 = r["corners"].get(c, {}).get(ax, float("nan"))
                        if x0 == x0 and x1 == x1:
                            shifts[ax].append(abs(x1 - x0))
                        c0, c1 = _cls(x0, S, M), _cls(x1, S, M)
                        if c0 == "no data" and c1 == "no data":
                            continue
                        flips[ax][1] += 1
                        if c0 != c1:
                            flips[ax][0] += 1
                            if "no data" in (c0, c1):
                                dirs["to/from no data"] += 1
                            else:
                                dirs["more severe" if RANK[c1] > RANK[c0] else "less severe"] += 1
            L.append(f"  {r['label']}: status {r['status']}" + (f" ({r['reason']})" if r["reason"] else "")
                     + f", gate {r['gate']}, fallback {r['fallback']}, re-entries {r['n_hard_bound_reentries']}; "
                     + "; ".join(parts))
            J[_tag(v)][k] = {kk: r[kk] for kk in ("status", "reason", "gate", "fallback", "n_hard_bound_reentries",
                                                  "axles")}
        if v != BASE:
            def q(x, p_):
                return float(np.percentile(x, p_)) if x else float("nan")
            tot = flips["f"][1] + flips["r"][1]
            L.append(f"  vs baseline: corner-value shift p50 / p90 front {q(shifts['f'], 50):.3f} / "
                     f"{q(shifts['f'], 90):.3f}, rear {q(shifts['r'], 50):.3f} / {q(shifts['r'], 90):.3f}; class "
                     f"changes front {flips['f'][0]}/{flips['f'][1]}, rear {flips['r'][0]}/{flips['r'][1]} "
                     f"({100 * (flips['f'][0] + flips['r'][0]) / max(1, tot):.1f} %); "
                     + ", ".join(f"{d} {n}" for d, n in dirs.items()))
            J[_tag(v)]["_vs_baseline"] = {"shift_p50_f": q(shifts["f"], 50), "shift_p90_f": q(shifts["f"], 90),
                                          "shift_p50_r": q(shifts["r"], 50), "shift_p90_r": q(shifts["r"], 90),
                                          "flips": flips, "directions": dirs}
        L.append("")
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "ce_sensitivity.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, "ce_sensitivity.json"), "w") as fh:
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
                if not os.path.exists(os.path.join(CACHE, f"{k}__{_tag(v)}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, jobs):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
