# cs_margin re-derivation on the new default (author EXECUTE 2026-10-20):
# the recorded +/-1 % verdict-sensitivity sweep of 2026-09-19
# (inspect_metrology_phase1_sensitivity.py + inspect_metrology_phase1_
# analysis.py), run on the fit-L default with the adopted thresholds over the
# 14-session pool. Per-corner-phase verdicts (classify_all), the flip
# definition (_is_flip: the verdict's axle direction changes) and the
# wheel-angle-column steering perturbation are imported unchanged.
#
# ADAPTATIONS, forced, stated:
# - Chain: the new default as the app runs it (speed classes reclassified,
#   resolve_sideslip_beta_and_cs_inputs, CS with the inputs it hands over);
#   the recorded run_one predates fit L and would compute CS on raw Fy.
# - Vehicle inputs: the recorded sweep perturbed Dubai's and v3's
#   setup-sheet corner weights (Level 2, a / b cascaded from them). The 14
#   sessions carry no weighing; the default resolves the vehicle at Level 1,
#   where a / b are stored constants. The same physical perturbations are
#   applied to the Level-1 values, with a / b moved as the cascade would:
#     mass            all corner weights and mass_kg x (1 + p); a, b fixed
#     front_fraction  front share x (1 + p) at fixed total, each axle's L/R
#                     ratio kept; a = wb (1 - ff), b = wb ff
#     wheelbase       wb, a, b x (1 + p) (front share fixed)
#     yaw_inertia     x (1 + p)
#     steering        wheel-angle column of the steering table x (1 + p)
#   so the baseline is the production default itself.
# MARGIN RULE (recorded 2026-09-19): for every flipped corner-phase, the
# baseline distance of its front / rear CS ratio to the nearest threshold of
# that axle (minimum over the axles); cs_margin = the 2nd-most-marginal flip
# distance after different-mechanism cases are read individually (ratio
# discontinuities, front / rear comparison jumps), rounded up to 0.01; with
# no proportional flip the value is retained (2026-10-05 / 2026-10-11
# precedent). Distances and per-flip CS changes are listed so the exclusion
# reading is visible.
# Also: per session, the share of card verdicts (analysis-lap corner
# summaries, as the app shows them) tagged [MARGINAL] at the old and the new
# margin.
# Output diagnostics/results_cs_margin_20261020/ (per-run JSON, summaries of
# the baselines, analysis.txt / .json).
# Run: python -m diagnostics.inspect_cs_margin_l [report [new_margin]]

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
    from ui.views.outing_form import OutingForm
    from modules.recommendation import aggregate_by_corner, PHASE_KEYS
    aggregated = aggregate_by_corner(summaries)
    out = {}
    for cid, corner in aggregated.items():
        for phase in PHASE_KEYS:
            if phase not in corner["phases"]:
                continue
            sliced = {"phases": {phase: corner["phases"][phase]}}
            if phase == "apex_3" and corner.get("apex_region") is not None:
                sliced["apex_region"] = corner["apex_region"]
            severity, short, _long, _colour = OutingForm._classify_corner(None, sliced)
            out[(cid, phase)] = (severity, short)
    return out, aggregated


def _scale_wheel_angle_column(car_data, pct):
    # +/- pct on the steering table's wheel-angle column (from
    # inspect_metrology_phase1_sensitivity.py, unchanged)
    cd = copy.deepcopy(car_data)
    table = cd["steering_ratio_table"]
    wheel_idx = table["columns"].index("wheel_angle_deg")
    table["rows"] = [
        [(v * (1 + pct) if i == wheel_idx else v) for i, v in enumerate(row)]
        for row in table["rows"]
    ]
    return cd

# run tag (env MARGIN_RUN_TAG): 20261020 = first fit-L sweep, 20261020b = after
# the R26 source fix; each run keeps its own folder
OUT = os.path.join(ROOT, "diagnostics", f"results_cs_margin_{os.environ.get('MARGIN_RUN_TAG', '20261020')}")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
INPUTS = ("mass", "front_fraction", "wheelbase", "yaw_inertia", "steering_wheel_angle")
PCT = 0.01
WORKERS = 6
OLD_MARGIN = 0.11
# observer-based uncertainty, robustness run 2026-10-20 (Q x 0.3): median / p90
# of |corner-value change|, front / rear
OBSERVER_Q03 = {"front": (0.017, 0.085), "rear": (0.020, 0.106)}


def _variant_params(p, input_name, sign):
    v = p["vehicle"]
    f = 1.0 + sign * PCT
    cw = v["corner_weights"]
    if input_name == "mass":
        for k in ("FL_kg", "FR_kg", "RL_kg", "RR_kg"):
            cw[k] *= f
        v["mass_kg"] *= f
    elif input_name == "front_fraction":
        tot = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
        ff0 = (cw["FL_kg"] + cw["FR_kg"]) / tot
        ff = ff0 * f
        fl_r = cw["FL_kg"] / (cw["FL_kg"] + cw["FR_kg"])
        rl_r = cw["RL_kg"] / (cw["RL_kg"] + cw["RR_kg"])
        wf, wr = tot * ff, tot * (1.0 - ff)
        cw.update({"FL_kg": wf * fl_r, "FR_kg": wf * (1 - fl_r), "RL_kg": wr * rl_r, "RR_kg": wr * (1 - rl_r)})
        # the cascade's change applied to the stored (3 dp) a / b, so the
        # rounding of the stored values does not enter the 1 % step
        da = v["wheelbase_m"] * (ff0 - ff)
        v["cog_to_front_axle_m"] += da
        v["cog_to_rear_axle_m"] -= da
    elif input_name == "wheelbase":
        for k in ("wheelbase_m", "cog_to_front_axle_m", "cog_to_rear_axle_m"):
            v[k] *= f
    elif input_name == "yaw_inertia":
        v["yaw_inertia_kgm2"] *= f
    return p


def run_task(task):
    key, input_name, sign = task
    import modules.accuracy_resolution as accres
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, load_car_data, prepare_vehicle_state,
                                            estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            estimate_vertical_loads, summarise_corners)
    from modules.corner_analysis import reclassify_speed_classes
    from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    params = copy.deepcopy(load_parameters())
    if input_name not in ("baseline", "steering_wheel_angle"):
        params = _variant_params(params, input_name, sign)
    if input_name == "steering_wheel_angle":
        patched = _scale_wheel_angle_column(load_car_data(), sign * PCT)
        orig = accres.load_car_data
        accres.load_car_data = lambda: patched
        try:
            resolved = accres.resolve_accuracy(params, setup_data=None, cap=None)
        finally:
            accres.load_car_data = orig
    else:
        resolved = accres.resolve_accuracy(params, setup_data=None, cap=None)
    p = accres.apply_resolved_vehicle(params, resolved)
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    st = prepare_vehicle_state(data["channels"], p)
    beta, man, gate, fb, fr, cs_in = resolve_sideslip_beta_and_cs_inputs(
        st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    fz = estimate_vertical_loads(st, forces, p, channels=data["channels"], car_data=load_car_data())
    summaries = summarise_corners(data.get("corners", []), cs, stab, st, fz=fz, lap_filter=None)
    verdicts, aggregated = classify_all(summaries)
    payload = {
        "session": key, "label": label, "input": input_name, "sign": sign, "fallback_used": fb,
        "status": man and man.get("status"), "gate": gate and gate.get("verdict"),
        "vehicle": {k: p["vehicle"][k] for k in ("mass_kg", "wheelbase_m", "yaw_inertia_kgm2",
                                                 "cog_to_front_axle_m", "cog_to_rear_axle_m")},
        "verdicts": {f"{k[0]}|{k[1]}": list(v) for k, v in verdicts.items()},
        "cs_values": {f"{cid}|{ph}": {"cs_ratio_f": c["phases"][ph].get("cs_ratio_f", {}).get("median"),
                                      "cs_ratio_r": c["phases"][ph].get("cs_ratio_r", {}).get("median")}
                      for cid, c in aggregated.items() for ph in c["phases"]},
        "elapsed_s": time.perf_counter() - t0,
    }
    os.makedirs(OUT, exist_ok=True)
    tag = "base" if input_name == "baseline" else ("plus1" if sign > 0 else "minus1")
    with open(os.path.join(OUT, f"{key}_{input_name}_{tag}.json"), "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=1, allow_nan=True)
    if input_name == "baseline":
        valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
        with open(os.path.join(OUT, f"{key}_baseline_summaries.pkl"), "wb") as fh:
            pickle.dump([s for s in summaries if s["lap_number"] in valid], fh)
    return f"{key} {input_name} {tag}: {payload['elapsed_s']:.0f} s, fallback {fb}, status {payload['status']}"


def _load(key, input_name, tag):
    fn = os.path.join(OUT, f"{key}_{input_name}_{tag}.json")
    return json.load(open(fn, encoding="utf-8")) if os.path.exists(fn) else None


def _marginal_shares(key, margin):
    from modules.stability_analysis import load_parameters
    from ui.views.outing_form import OutingForm
    cfg = load_parameters()["classification"]["verdict_stability_margin"]
    keep = cfg["cs_margin"]
    cfg["cs_margin"] = margin   # in-memory only, the cached config dict
    try:
        summ = pickle.load(open(os.path.join(OUT, f"{key}_baseline_summaries.pkl"), "rb"))
        shorts = [OutingForm._classify_corner(None, s)[1] for s in summ]
    finally:
        cfg["cs_margin"] = keep
    tier = [s for s in shorts if "understeer @" in s or "oversteer @" in s]
    return len(tier), sum("[MARGINAL]" in s for s in tier), len(shorts)


def report(new_margin=None):
    from diagnostics.inspect_metrology_phase1_analysis import _is_flip
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    thr = {"f": (cl["STRONG_CSF"]["value"], cl["MODERATE_CSF"]["value"]),
           "r": (cl["STRONG_CSR"]["value"], cl["MODERATE_CSR"]["value"])}
    L = [f"cs_margin RE-DERIVATION on the new default (+/-1 % sweep, 14 sessions); thresholds front {thr['f']}, "
         f"rear {thr['r']}; config cs_margin now {cl['verdict_stability_margin']['cs_margin']}", ""]
    flips, n_runs, n_fb, n_cp = [], 0, 0, 0
    for key in POOL:
        base = _load(key, "baseline", "base")
        if base is None:
            L.append(f"{key}: baseline missing")
            continue
        n_cp += len(base["verdicts"])
        for inp in INPUTS:
            for sign, tag in ((1, "plus1"), (-1, "minus1")):
                var = _load(key, inp, tag)
                if var is None:
                    L.append(f"{key} {inp} {tag}: missing")
                    continue
                n_runs += 1
                n_fb += bool(var["fallback_used"])
                for k, bv in base["verdicts"].items():
                    vv = var["verdicts"].get(k)
                    if vv is None or bv == vv or not _is_flip(bv[1], vv[1]):
                        continue
                    cb, cv = base["cs_values"].get(k, {}), var["cs_values"].get(k, {})
                    d = []
                    for ax in "fr":
                        x = cb.get(f"cs_ratio_{ax}")
                        if x is not None and x == x:
                            d.append(min(abs(x - thr[ax][0]), abs(x - thr[ax][1])))
                    flips.append({"session": key, "label": base["label"], "corner_phase": k, "input": inp,
                                  "sign": sign, "old": bv, "new": vv, "distance": min(d) if d else float("nan"),
                                  "base_f": cb.get("cs_ratio_f"), "base_r": cb.get("cs_ratio_r"),
                                  "var_f": cv.get("cs_ratio_f"), "var_r": cv.get("cs_ratio_r")})
    L.append(f"runs: {n_runs} variants + {len(POOL)} baselines; fallback in {n_fb} variant runs; "
             f"{n_cp} baseline corner-phases")
    L.append(f"FLIPS (axle direction changes, recorded _is_flip): {len(flips)} flips, "
             f"{len({(f['session'], f['corner_phase']) for f in flips})} distinct corner-phases")
    by_cp = {}
    for f in flips:
        by_cp.setdefault((f["session"], f["corner_phase"]), []).append(f)
    ranked = sorted(by_cp.items(), key=lambda kv: -kv[1][0]["distance"])
    for (key, cp), fl in ranked:
        f0 = fl[0]

        def fmt(x):
            return "nan" if x is None or x != x else f"{x:+.3f}"
        changes = "; ".join(f"{f['input']} {f['sign']:+d}%: {f['old'][1]} -> {f['new'][1]} "
                            f"(CSf {fmt(f['base_f'])}->{fmt(f['var_f'])}, CSr {fmt(f['base_r'])}->{fmt(f['var_r'])})"
                            for f in fl)
        L.append(f"  distance {f0['distance']:.3f}  {f0['label']} C{cp}: {len(fl)} input(s): {changes}")
    dists = sorted((fl[0]["distance"] for fl in by_cp.values()), reverse=True)
    L.append("")
    L.append("distinct flipped corner-phase distances, descending: " + ", ".join(f"{d:.3f}" for d in dists))
    L.append(f"observer uncertainty (robustness, Q x 0.3), |corner-value change| median / p90: front "
             f"{OBSERVER_Q03['front'][0]} / {OBSERVER_Q03['front'][1]}, rear {OBSERVER_Q03['rear'][0]} / "
             f"{OBSERVER_Q03['rear'][1]}")
    if new_margin is not None:
        L += ["", f"[MARGINAL] SHARE of card tier verdicts (analysis-lap corner summaries), old {OLD_MARGIN} -> new "
                  f"{new_margin}:"]
        for key in POOL:
            if not os.path.exists(os.path.join(OUT, f"{key}_baseline_summaries.pkl")):
                continue
            n_t, m_o, n_all = _marginal_shares(key, OLD_MARGIN)
            _, m_n, _ = _marginal_shares(key, new_margin)
            lab = _load(key, "baseline", "base")["label"]
            L.append(f"  {lab}: {n_t} tier verdicts of {n_all} cards; MARGINAL {m_o} "
                     f"({100 * m_o / n_t if n_t else float('nan'):.0f} %) -> {m_n} "
                     f"({100 * m_n / n_t if n_t else float('nan'):.0f} %)")
    text = "\n".join(L)
    with open(os.path.join(OUT, "analysis.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "analysis.json"), "w") as fh:
        json.dump({"flips": flips, "distances": dists}, fh, indent=1, default=str)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report(float(sys.argv[2]) if len(sys.argv) > 2 else None)
        return
    tasks = []
    for key in POOL:
        tasks.append((key, "baseline", 0))
        for inp in INPUTS:
            for sign in (1, -1):
                tasks.append((key, inp, sign))
    tasks = [t for t in tasks if not os.path.exists(os.path.join(
        OUT, f"{t[0]}_{t[1]}_{'base' if t[1] == 'baseline' else ('plus1' if t[2] > 0 else 'minus1')}.json"))]
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
