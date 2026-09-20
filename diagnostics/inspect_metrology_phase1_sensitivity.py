# Metrology overnight package, Phase 1: verdict sensitivity map.
# Read-only against config/production -- every perturbation below is
# applied to an in-memory deep copy of setup_data/params/car_data, never
# written to disk or to the real DB.
#
# WHICH INPUTS: enumerated by reading the actual fit/classification chain
# (modules/stability_analysis.py), not assumed. Module 4a (estimate_
# lateral_forces) consumes mass_kg, corner_weights (-> front_fraction),
# yaw_inertia_kgm2, wheelbase_m. Module 4b's slip angles (estimate_slip_
# angles) additionally consume cog_to_front/rear_axle_m (a, b) and
# delta_f_rad, which is steering_ratio-derived. Module 5 (estimate_yaw_
# moment_stability) additionally consumes yaw_inertia_kgm2 and delta_f_rad
# again via calculate_observed_stability. cog_to_front/rear_axle_m is
# itself a pure cascade of corner_weights + wheelbase_m at Level 2 (both
# real sessions, post Deepening-Phase-1) -- perturbing "front_fraction"
# already exercises that cascade, so a and b are not perturbed separately.
# beta (modules.stability_analysis.estimate_sideslip, kinematic) consumes
# only ay/v/yaw_rate -- no vehicle scalar at all, confirmed by reading the
# function; not a sensitivity input.
#
# SKIPPED, citing the Fz-independence precedent: cog_height_m, track_
# width_front_m, track_width_rear_m, and the four aero.* fields feed ONLY
# estimate_vertical_loads (Fz) -- confirmed by reading the function body,
# they never reach estimate_lateral_forces/estimate_slip_angles/estimate_
# yaw_moment_stability. Fz's own verdict-independence was already
# established twice: the Fz-integration Phase 1 census (thesis_notes.md,
# "FR's own corrected channel does not feed CS_ratio at all") and commit
# f33c50c's own message ("vertical_load_source default -> measured
# (verdict independence proven)"). Perturbing an input that structurally
# cannot reach the classifier would only pad the run count, not the
# finding.
#
# TESTED (5 inputs, +/-1% each, both sessions):
#   mass            -- scale all four corner weights by the same factor
#                       (front_fraction unchanged, isolates total mass).
#   front_fraction  -- redistribute front/rear split at fixed total mass,
#                       preserving each axle's own L/R ratio (isolates the
#                       fore-aft split alone).
#   wheelbase       -- perturbed in raw params BEFORE resolve_accuracy, so
#                       the Level-2 cog-position cascade (a = wb*rear_
#                       fraction, b = wb*front_fraction) recomputes
#                       consistently with the new wheelbase, exactly as it
#                       would in production if this measurement changed.
#   yaw_inertia     -- perturbed in raw params (static passthrough, no
#                       per-session resolution logic of its own).
#   steering_ratio  -- both real sessions resolve this at Level 4 (a real,
#                       gitignored car_data.json with a valid steering_
#                       ratio_table exists on this machine, confirmed by
#                       reading _resolve_steering_ratio's own priority
#                       rule) -- perturbing the Level-1 config CONSTANT
#                       would therefore be perturbing a value production
#                       never reads. The table's own ratio column is
#                       scaled instead (a uniform +/-1% digitisation-error
#                       proxy), via a load_car_data monkeypatch scoped to
#                       modules.accuracy_resolution's own reference to it
#                       -- the same in-process monkeypatch technique
#                       already used for channel-whitelist probing
#                       (Deepening Phase 3), never touching the real file.
#
# COMPUTE BUDGET, stated up front: each full pipeline run re-fits the
# production ekf_auto_pacejka curve for real (resolve_sideslip_beta) and
# re-runs the s-anchored windowed regressions (Module 4b/5) -- timed
# directly this session at ~5.7 min/run (v3) and ~10.3 min/run (Dubai),
# dominated by the windowed-regression stage, not the fit itself. 22 runs
# (2 baseline + 5 inputs x 2 directions x 2 sessions) at those real,
# measured costs is ~2.9 hours sequential; run here via a 6-way process
# pool (16 logical cores available) with each worker's BLAS thread count
# pinned to 1 to avoid oversubscription.
#
# MARGIN DEFINITION, stated up front as a real scope limitation, not
# hidden: only ONE magnitude (+/-1%) is tested per input, per the work
# order's own instruction, not a bisection grid (a continuous margin
# search would multiply the run count severalfold, hours each). The
# "stability margin" reported in Phase 1's aggregate is therefore
# CENSORED, not continuous: a corner-phase that flips under some input's
# +/-1% perturbation is reported as "margin <= 1%" (naming the input and
# direction); one that does not flip under ANY tested input is reported
# as "margin > 1% (untested beyond)", not as a precise number -- honest
# interval reporting instead of a false-precision extrapolation past a
# single measured point.

import copy
import json
import os
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

from concurrent.futures import ProcessPoolExecutor, as_completed

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

# Both outings' real, live setup-sheet weighing (Deepening Phase 1 census
# -- Dubai's own real weighing predates this whole project's analysis
# arc; v3's is this package's own Phase 1 update). total_weight/cross_
# percentage omitted (both stored 0.0 = "not entered" on both outings),
# so mass derives from the corner-weight sum on both, same resolver path.
SESSIONS = {
    "dubai": {
        "file": DUBAI_FILE,
        "car_setup": {"corner_weight_fl": 302.6, "corner_weight_fr": 301.1,
                      "corner_weight_rl": 392.8, "corner_weight_rr": 389.4},
    },
    "v3": {
        "file": V3_FILE,
        "car_setup": {"corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
                      "corner_weight_rl": 396.2, "corner_weight_rr": 386.5},
    },
}

PCT = 0.01
INPUTS = ["mass", "front_fraction", "wheelbase", "yaw_inertia", "steering_ratio"]
MAX_WORKERS = 6

RESULTS_DIR = Path("diagnostics/results_metrology")
PLOTS_DIR = Path("diagnostics/plots_metrology")


def _scale_mass(car_setup, pct):
    return {k: v * (1 + pct) for k, v in car_setup.items()}


def _scale_front_fraction(car_setup, pct):
    fl, fr = car_setup["corner_weight_fl"], car_setup["corner_weight_fr"]
    rl, rr = car_setup["corner_weight_rl"], car_setup["corner_weight_rr"]
    w_total = fl + fr + rl + rr
    ff = (fl + fr) / w_total
    ff_new = ff * (1 + pct)
    w_f_new = w_total * ff_new
    w_r_new = w_total - w_f_new
    fl_ratio = fl / (fl + fr)
    rl_ratio = rl / (rl + rr)
    return {
        "corner_weight_fl": w_f_new * fl_ratio,
        "corner_weight_fr": w_f_new * (1 - fl_ratio),
        "corner_weight_rl": w_r_new * rl_ratio,
        "corner_weight_rr": w_r_new * (1 - rl_ratio),
    }


def _scale_steering_ratio_table(car_data, pct):
    cd = copy.deepcopy(car_data)
    table = cd["steering_ratio_table"]
    ratio_idx = table["columns"].index("steering_ratio")
    table["rows"] = [
        [(v * (1 + pct) if i == ratio_idx else v) for i, v in enumerate(row)]
        for row in table["rows"]
    ]
    return cd


def build_variant(car_setup_base, input_name, sign):
    """Returns (car_setup, raw_params_or_None, car_data_patch_or_None, desc)."""
    from modules.stability_analysis import load_parameters, load_car_data
    pct = sign * PCT
    if input_name == "mass":
        return _scale_mass(car_setup_base, pct), None, None, f"mass {sign:+d}%"
    if input_name == "front_fraction":
        return _scale_front_fraction(car_setup_base, pct), None, None, f"front_fraction {sign:+d}%"
    if input_name == "wheelbase":
        p = copy.deepcopy(load_parameters())
        p["vehicle"]["wheelbase_m"] *= (1 + pct)
        return car_setup_base, p, None, f"wheelbase {sign:+d}%"
    if input_name == "yaw_inertia":
        p = copy.deepcopy(load_parameters())
        p["vehicle"]["yaw_inertia_kgm2"] *= (1 + pct)
        return car_setup_base, p, None, f"yaw_inertia {sign:+d}%"
    if input_name == "steering_ratio":
        cd = _scale_steering_ratio_table(load_car_data(), pct)
        return car_setup_base, None, cd, f"steering_ratio {sign:+d}%"
    raise ValueError(input_name)


def run_one(csv_file, car_setup, raw_params=None, car_data_patch=None):
    import modules.accuracy_resolution as accres
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (
        load_parameters, load_car_data, prepare_vehicle_state,
        estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
        estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
    )
    from modules.tyre_fit_auto import resolve_sideslip_beta

    params = raw_params if raw_params is not None else copy.deepcopy(load_parameters())
    setup_data = {"car": car_setup}

    if car_data_patch is not None:
        orig = accres.load_car_data
        accres.load_car_data = lambda: car_data_patch
        try:
            resolved = accres.resolve_accuracy(params, setup_data=setup_data, cap=None)
        finally:
            accres.load_car_data = orig
    else:
        resolved = accres.resolve_accuracy(params, setup_data=setup_data, cap=None)

    effective_params = accres.apply_resolved_vehicle(params, resolved)

    data = parse_csv(csv_file)
    state = prepare_vehicle_state(data["channels"], effective_params)
    if state is None:
        raise RuntimeError(f"{csv_file}: prepare_vehicle_state returned None")
    live_default = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    beta, _fm, _gate, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, effective_params, data, live_default, csv_path=csv_file)
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
    stab = estimate_yaw_moment_stability(state, beta, effective_params, data.get("laps", []))
    fz = estimate_vertical_loads(state, forces, effective_params,
                                  channels=data["channels"], car_data=load_car_data())
    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)

    cw = effective_params["vehicle"]["corner_weights"]
    w_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
    front_fraction = (cw["FL_kg"] + cw["FR_kg"]) / w_total

    return {
        "summaries": summaries,
        "front_fraction": front_fraction,
        "mass_kg": effective_params["vehicle"]["mass_kg"],
        "wheelbase_m": effective_params["vehicle"]["wheelbase_m"],
        "yaw_inertia_kgm2": effective_params["vehicle"]["yaw_inertia_kgm2"],
        "fallback_used": fallback_used,
        "fallback_reason": fallback_reason,
    }


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


def _worker_task(task):
    session_name, input_name, sign = task
    sess = SESSIONS[session_name]
    car_setup_base = sess["car_setup"]
    if input_name == "baseline":
        car_setup, raw_params, car_data_patch, desc = car_setup_base, None, None, "baseline"
    else:
        car_setup, raw_params, car_data_patch, desc = build_variant(car_setup_base, input_name, sign)

    t0 = time.time()
    result = run_one(sess["file"], car_setup, raw_params=raw_params, car_data_patch=car_data_patch)
    verdicts, aggregated = classify_all(result["summaries"])
    elapsed = time.time() - t0

    apex_xy = {}
    for cid, corner in aggregated.items():
        apex_xy[str(cid)] = {"x": corner.get("apex_position_x_m"), "y": corner.get("apex_position_y_m")}

    payload = {
        "session": session_name, "input": input_name, "sign": sign, "desc": desc,
        "front_fraction": result["front_fraction"], "mass_kg": result["mass_kg"],
        "wheelbase_m": result["wheelbase_m"], "yaw_inertia_kgm2": result["yaw_inertia_kgm2"],
        "fallback_used": result["fallback_used"], "fallback_reason": result["fallback_reason"],
        "elapsed_s": elapsed,
        "verdicts": {f"{k[0]}|{k[1]}": list(v) for k, v in verdicts.items()},
        "cs_values": {
            f"{cid}|{phase}": {
                "cs_ratio_f": corner["phases"][phase].get("cs_ratio_f", {}).get("median"),
                "cs_ratio_r": corner["phases"][phase].get("cs_ratio_r", {}).get("median"),
                "stability_observed": corner["phases"][phase].get("stability_observed_Nm_per_deg", {}).get("median"),
            }
            for cid, corner in aggregated.items() for phase in corner["phases"]
        },
        "apex_xy": apex_xy,
    }

    label = f"{session_name}_{input_name}_{'base' if input_name == 'baseline' else ('plus1' if sign > 0 else 'minus1')}"
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = RESULTS_DIR / f"{label}.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, allow_nan=True)

    return label, payload, elapsed


def build_task_list():
    tasks = []
    for session_name in SESSIONS:
        tasks.append((session_name, "baseline", 0))
        for input_name in INPUTS:
            tasks.append((session_name, input_name, +1))
            tasks.append((session_name, input_name, -1))
    return tasks


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    tasks = build_task_list()
    print(f"=== Metrology Phase 1: {len(tasks)} pipeline runs, {MAX_WORKERS} workers ===", flush=True)

    t_start = time.time()
    done = 0
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as pool:
        futures = {pool.submit(_worker_task, task): task for task in tasks}
        for fut in as_completed(futures):
            task = futures[fut]
            try:
                label, payload, elapsed = fut.result()
                done += 1
                print(f"[{done}/{len(tasks)}] {label}: {elapsed:.1f}s "
                      f"(fallback_used={payload['fallback_used']})", flush=True)
            except Exception as exc:
                done += 1
                print(f"[{done}/{len(tasks)}] {task} FAILED: {exc!r}", flush=True)

    print(f"\n=== All runs complete in {time.time()-t_start:.1f}s total wall time ===", flush=True)


if __name__ == "__main__":
    main()
