# Drag sensitivity (author EXECUTE 2026-10-21, read-only, before any config
# change). The Level-1 literature estimate c_D * A = 1.0 m^2 (DECISIONS
# 'Drag, Level-1 literature estimate') against the shipped 0, with 0.7 and
# 1.3 m^2 around it. Applied in estimate_longitudinal_forces ONLY: the
# function as tyre_fit_auto imports it is wrapped in memory and its CoG
# force gains + 0.5 rho (c_D A) v^2 (tyre force = m a_x + drag, Fx < 0 under
# braking). Everything else is the default chain (fit L, schema 18): the
# Eq. 9 axle split, the braking filter, the observer's front force frame,
# CS, stability, summarise_corners, floor events.
# Per session and axle: braking-filter excluded share of the fit population,
# fit-L B / K = B C mu / mu_demo, status and gate; floor events (count,
# clean, max |Fx| / Fz median); verdict class changes against c_D A = 0 with
# the adopted thresholds -- per corner-phase by the recorded cs_margin
# method (inspect_metrology_phase1_sensitivity.classify_all) and per corner
# (worst phase, the verdict card), short verdict text without qualifier tags.
# Output diagnostics/plots/drag_sensitivity/ (cache/, drag_sensitivity.txt).
# Run: python -m diagnostics.inspect_drag_sensitivity [report]

import copy
import os
import pickle
import re
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

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


# Per corner-phase verdicts through the app's own classifier; copied
# unchanged from the retired inspect_metrology_phase1_sensitivity.py
# (the recorded cs_margin method, 2026-10-22).
def classify_all(summaries):
    from modules.classifier import classify_corner
    from modules.corner_aggregation import aggregate_by_corner, PHASE_KEYS
    aggregated = aggregate_by_corner(summaries)
    out = {}
    for cid, corner in aggregated.items():
        for phase in PHASE_KEYS:
            if phase not in corner["phases"]:
                continue
            sliced = {"phases": {phase: corner["phases"][phase]}}
            if phase == "apex_3" and corner.get("apex_region") is not None:
                sliced["apex_region"] = corner["apex_region"]
            severity, short, _long, _colour = classify_corner(sliced)
            out[(cid, phase)] = (severity, short)
    return out, aggregated

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "drag_sensitivity")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
CDA = (0.0, 0.7, 1.0, 1.3)   # m^2
WORKERS = 3


def _strip(short):
    return re.sub(r"\s*\[[^\]]*\]", "", short or "").strip()


def compute(task):
    key, cda = task
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.limit_events import detect_floor_events
    from modules.corner_aggregation import aggregate_by_corner
    from modules.classifier import classify_corner
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    rho = p["vehicle"]["aero"]["air_density_kgm3"]
    orig = tfa.estimate_longitudinal_forces

    def with_drag(state, channels, params):
        fx = dict(orig(state, channels, params))
        fx["fx_N"] = fx["fx_N"] + 0.5 * rho * cda * np.maximum(state["v_mps"], 0.0) ** 2
        return fx

    tfa.estimate_longitudinal_forces = with_drag
    try:
        beta, man, gate, fb, fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
            st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    finally:
        tfa.estimate_longitudinal_forces = orig
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(st, p), st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    summ = [s for s in summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
            if s["lap_number"] in valid]
    phase_verdicts, aggregated = classify_all(summ)
    corner_verdicts = {cid: _strip(classify_corner(c)[1]) for cid, c in aggregated.items()}
    events = detect_floor_events(cs, st, data.get("corners", []), laps, p, slip,
                                 axle_loads=cs_in["cs_kwargs"].get("axle_loads"), axle_fx=cs_in["axle_fx"])["events"]
    fit = {}
    for ax in ("front", "rear"):
        a = man["axles"][ax]
        fit[ax] = {"B": a["B"], "mu": a["mu"], "K": a["B"] * a["C"] * a["mu"],
                   "braking_excluded": man["axle_load"]["braking_excluded"][ax]["share"]}
    out = {"key": key, "label": label, "cda": cda, "status": man.get("status"), "fallback": fb,
           "gate": gate and gate.get("health_score"), "fit": fit,
           "phase_verdicts": {k: _strip(v[1]) for k, v in phase_verdicts.items()},
           "corner_verdicts": corner_verdicts,
           "events": [{k: e[k] for k in ("axle", "corner", "lap", "phase", "fx_util_max", "kerb_affected",
                                         "observer_reset_affected")} for e in events]}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}__{cda:.1f}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key} cdA {cda}] status {out['status']}, fallback {fb}, events {len(events)}", flush=True)
    return task


def report():
    res = {(k, c): pickle.load(open(os.path.join(CACHE, f"{k}__{c:.1f}.pkl"), "rb")) for k in POOL for c in CDA}
    L = ["DRAG SENSITIVITY (c_D A applied in estimate_longitudinal_forces only; default chain, schema 18; "
         "thresholds fixed)", ""]
    tot = {c: {"ph": 0, "ph_n": 0, "co": 0, "co_n": 0} for c in CDA}
    ev_tot = {c: {"front": 0, "rear": 0} for c in CDA}
    for k in POOL:
        base = res[(k, 0.0)]
        L.append("=" * 100)
        L.append(f"{base['label']}")
        for c in CDA:
            r = res[(k, c)]
            fx = []
            for ax in ("front", "rear"):
                f = r["fit"][ax]
                evs = [e for e in r["events"] if e["axle"] == ax]
                ev_tot[c][ax] += len(evs)
                clean = sum(1 for e in evs if not e["kerb_affected"] and not e["observer_reset_affected"])
                fxu = [e["fx_util_max"] for e in evs if e["fx_util_max"] is not None]
                fx.append(f"{ax}: braking-excluded {f['braking_excluded'] * 100:.2f} %, B {f['B']:.3f}, K "
                          f"{f['K']:.2f}, mu_demo {f['mu']:.3f}; floor events {len(evs)} ({clean} clean), max "
                          f"|Fx|/Fz median {np.median(fxu) if fxu else float('nan'):.3f}")
            pv = r["phase_verdicts"]
            ph_ch = [kk for kk in base["phase_verdicts"] if pv.get(kk) != base["phase_verdicts"][kk]]
            co_ch = [cid for cid in base["corner_verdicts"] if r["corner_verdicts"].get(cid) != base["corner_verdicts"][cid]]
            tot[c]["ph"] += len(ph_ch); tot[c]["ph_n"] += len(base["phase_verdicts"])
            tot[c]["co"] += len(co_ch); tot[c]["co_n"] += len(base["corner_verdicts"])
            detail = ", ".join(f"C{cid} {base['corner_verdicts'][cid] or 'none'} -> {r['corner_verdicts'].get(cid) or 'none'}"
                               for cid in co_ch[:6])
            L.append(f"  c_D A {c:.1f}: status {r['status']}, fallback {r['fallback']}, gate {r['gate']:.4f}; "
                     + "; ".join(fx))
            if c:
                L.append(f"      verdict changes vs 0: corner-phases {len(ph_ch)} / {len(base['phase_verdicts'])}, "
                         f"corners {len(co_ch)} / {len(base['corner_verdicts'])}" + (f" ({detail})" if detail else ""))
    L.append("=" * 100)
    for c in CDA:
        L.append(f"POOL c_D A {c:.1f}: floor events front {ev_tot[c]['front']} / rear {ev_tot[c]['rear']}"
                 + (f"; verdict changes vs 0: corner-phases {tot[c]['ph']} / {tot[c]['ph_n']} "
                    f"({tot[c]['ph'] / max(1, tot[c]['ph_n']) * 100:.1f} %), corners {tot[c]['co']} / {tot[c]['co_n']} "
                    f"({tot[c]['co'] / max(1, tot[c]['co_n']) * 100:.1f} %)" if c else ""))
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "drag_sensitivity.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    keep_awake(True)
    try:
        todo = [(k, c) for k in POOL for c in CDA if not os.path.exists(os.path.join(CACHE, f"{k}__{c:.1f}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, todo):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
