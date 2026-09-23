# PLAN.md ### NOW item (2): "Pipeline performance: profile first --
# read-only per-module timing on both sessions, machine untouched during
# the run -- then decide. Observed unprofiled: Modules 1-5 832s, fit
# chain 188-383s." Read-only measurement: wraps each pipeline call with
# time.perf_counter() in THIS script only -- nothing in modules/ is
# edited or instrumented. No config/production change, no commit.
#
# Reuses diagnostics/inspect_frame_stage2_parity.py's own DUBAI_FILE/
# V3_FILE/FIXED_CAP constants directly (not re-typed) and its
# run_full_pipeline's exact call shape (production default sideslip
# source, accuracy cap=1, no feedback/setup data) -- but re-issues each
# of that function's own calls ONE AT A TIME, timed, instead of importing
# run_full_pipeline as one opaque blob. That would defeat the point: the
# work order is explicit that Modules 1-5 must be reported individually,
# not lumped.
#
# Live config (read directly before writing this script, never assumed):
# stability_estimation.sideslip_source = "ekf_auto_pacejka",
# vertical_load_source = "measured". The fit chain's own three sub-stages
# (Pacejka fit / EKF run / NIS gate) are NOT three separate top-level
# calls in production -- fit_session_pacejka (modules/tyre_fit_auto.py)
# is one monolithic function that calls _fit_axle_pacejka (the per-axle
# Pacejka fit, x2) and estimate_sideslip_ekf_pacejka (the EKF run, x3:
# interim/sweep-loop/final) internally, with no call boundary this script
# could time from outside without editing modules/ -- forbidden by the
# work order. Resolved by wrapping the single fit_session_pacejka call in
# cProfile (a script-level wrapper only) and bucketing its own per-
# function cumulative-time table by name afterward; NIS gate
# (evaluate_gate) IS already a separate top-level call in
# resolve_sideslip_beta, so it is timed directly like every other stage.
# cProfile's own overhead means the two fit-chain sub-splits are
# approximate; the un-profiled whole-call wall time (fit_total) is
# authoritative and is compared against the bucket sum to show how much
# is left over as "other" (R-derivation, the sweep loop's own bookkeeping,
# sign-check, onset coverage).
#
# NOTE ON "Modules 1-5": no file in this repo defines a numbered 1-5 list
# mapping module ordinal to function (checked directly: no "Module 1"
# self-label anywhere in modules/, no MODULES.md) -- per CLAUDE.md's
# "never guess" rule, results below are reported by exact function name,
# not by an invented ordinal, with an explicit subtotal matching PLAN.md's
# own "Modules 1-5" span (prepare_vehicle_state through estimate_yaw_
# moment_stability, i.e. everything before the Fz/longitudinal/summarise
# side-modules) so the comparison against the 832s figure is still directly
# possible.

import cProfile
import pstats
import time

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, load_car_data,
    estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    estimate_vertical_loads, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
from modules.tyre_fit_auto import fit_session_pacejka
from modules.nis_gate import evaluate_gate
from diagnostics.inspect_frame_stage2_parity import DUBAI_FILE, V3_FILE, FIXED_CAP

def _time(fn, *args, **kwargs):
    t0 = time.perf_counter()
    result = fn(*args, **kwargs)
    return result, time.perf_counter() - t0


def _bucket_profile(profile, name_filters):
    stats = pstats.Stats(profile)
    buckets = {label: 0.0 for label in name_filters}
    for func, (cc, nc, tt, ct, callers) in stats.stats.items():
        funcname = func[2]
        for label, names in name_filters.items():
            if funcname in names:
                # cumulative time (ct), not internal time (tt): each
                # bucket must include time spent in whatever THAT function
                # itself calls (scipy Powell inside the Pacejka fit, the
                # per-sample EKF loop inside the EKF run) -- the two
                # buckets are disjoint branches of fit_session_pacejka's
                # own call tree, so summing ct across them double-counts
                # nothing.
                buckets[label] += ct
    return buckets


def run_timed(raw_file, label):
    print(f"\n{'='*70}\n{label}: {raw_file}\n{'='*70}")
    rows = []

    params = load_parameters()
    resolved, dt = _time(resolve_accuracy, params, setup_data=None, cap=FIXED_CAP)
    effective_params, dt2 = _time(apply_resolved_vehicle, params, resolved)
    rows.append(("accuracy resolution (pre-pipeline)", dt + dt2))

    data, dt = _time(parse_csv, raw_file)
    rows.append(("parse_csv (pre-pipeline)", dt))

    state, dt = _time(prepare_vehicle_state, data["channels"], effective_params)
    rows.append(("prepare_vehicle_state", dt))

    sideslip_source = effective_params["stability_estimation"].get("sideslip_source", "kinematic")
    if sideslip_source == "ekf_auto_pacejka":
        load_normalised = effective_params.get("tyre_fit_auto", {}).get("load_normalised_fit_enabled", False)
        profiler = cProfile.Profile()
        t0 = time.perf_counter()
        profiler.enable()
        raw_fit_manifest = fit_session_pacejka(data, effective_params, data_file_path=raw_file,
                                                load_normalised=load_normalised)
        profiler.disable()
        fit_total = time.perf_counter() - t0
        buckets = _bucket_profile(profiler, {
            "fit chain: Pacejka fit": {"_fit_axle_pacejka", "_fit_axle_pacejka_mu"},
            "fit chain: EKF run": {"estimate_sideslip_ekf_pacejka"},
        })
        accounted = sum(buckets.values())
        rows.append(("fit chain: Pacejka fit", buckets["fit chain: Pacejka fit"]))
        rows.append(("fit chain: EKF run", buckets["fit chain: EKF run"]))
        rows.append(("fit chain: other (R-derivation/sweep/sign-check)", max(0.0, fit_total - accounted)))
        rows.append(("fit chain: TOTAL (un-profiled wall time)", fit_total))

        t0 = time.perf_counter()
        if raw_fit_manifest.get("status") != "degenerate":
            gate_verdict = evaluate_gate(raw_fit_manifest["nis_full"], raw_fit_manifest["base_mask"],
                                          effective_params, state["sample_rate_hz"])
            fallback_fired = gate_verdict["verdict"] == "fail"
            beta = estimate_sideslip(state, effective_params) if fallback_fired \
                else raw_fit_manifest["beta_ekf_with_fallback"]
        else:
            fallback_fired = True
            beta = estimate_sideslip(state, effective_params)
        # If the gate/fit falls back, this row also carries one kinematic
        # estimate_sideslip call's cost (mirrors resolve_sideslip_beta's
        # own real structure exactly -- production pays that same cost at
        # this same point) -- flagged here rather than silently folded in.
        rows.append(("fit chain: NIS gate", time.perf_counter() - t0))
        if fallback_fired:
            print("  NOTE: fit/gate fell back to kinematic beta on this file -- "
                  "'fit chain: NIS gate' row includes one kinematic estimate_sideslip call.")
    else:
        beta, dt = _time(estimate_sideslip, state, effective_params)
        rows.append(("estimate_sideslip (kinematic)", dt))

    slip, dt = _time(estimate_slip_angles, state, beta, effective_params)
    rows.append(("estimate_slip_angles", dt))
    forces, dt = _time(estimate_lateral_forces, state, effective_params)
    rows.append(("estimate_lateral_forces", dt))
    cs, dt = _time(estimate_cornering_stiffness, slip, forces, state, effective_params)
    rows.append(("estimate_cornering_stiffness", dt))
    stab, dt = _time(estimate_yaw_moment_stability, state, beta, effective_params, data.get("laps", []))
    rows.append(("estimate_yaw_moment_stability", dt))
    fz, dt = _time(estimate_vertical_loads, state, forces, effective_params,
                    channels=data["channels"], car_data=load_car_data())
    rows.append(("estimate_vertical_loads (wheel_loads)", dt))
    long_forces, dt = _time(estimate_longitudinal_forces, state, data["channels"], effective_params)
    rows.append(("estimate_longitudinal_forces", dt))
    slip_ratio, dt = _time(estimate_slip_ratio, state, data["channels"], effective_params)
    rows.append(("estimate_slip_ratio", dt))
    ls, dt = _time(estimate_longitudinal_stiffness, long_forces, slip_ratio, state, effective_params)
    rows.append(("estimate_longitudinal_stiffness", dt))
    corners = data.get("corners", [])
    summaries, dt = _time(summarise_corners, corners, cs, stab, state, fz=fz, ls=ls, lap_filter=None)
    rows.append(("summarise_corners", dt))

    grand_total = sum(t for name, t in rows if not name.endswith("(un-profiled wall time)"))
    print(f"\n{'stage':<52}{'seconds':>10}{'% of total':>12}")
    for name, t in sorted(rows, key=lambda x: -x[1]):
        marker = "  (sub-split, not added to total)" if name.endswith("(un-profiled wall time)") else ""
        pct = 0.0 if marker else 100 * t / grand_total
        print(f"{name:<52}{t:>10.2f}{pct:>11.1f}%{marker}")
    print(f"{'GRAND TOTAL (parse_csv through summarise_corners)':<52}{grand_total:>10.2f}{100.0:>11.1f}%")

    modules_1_5_subtotal = sum(t for name, t in rows if name in (
        "prepare_vehicle_state", "estimate_slip_angles", "estimate_lateral_forces",
        "estimate_cornering_stiffness", "estimate_yaw_moment_stability",
    ))
    # Matches PLAN.md's own "fit chain 188-383s" as ONE combined figure --
    # Pacejka fit + EKF run + other bookkeeping + NIS gate, since the gate
    # runs as part of resolving sideslip beta under the auto mode, same
    # as production's own resolve_sideslip_beta does it in one sequence.
    fit_chain_subtotal = sum(t for name, t in rows if name in (
        "fit chain: Pacejka fit", "fit chain: EKF run",
        "fit chain: other (R-derivation/sweep/sign-check)", "fit chain: NIS gate",
    )) or sum(t for name, t in rows if name == "estimate_sideslip (kinematic)")
    print(f"\nModules-1-5 subtotal (excl. fit chain, excl. Fz/long/summarise/parse) = {modules_1_5_subtotal:.2f}s")
    print(f"Fit-chain subtotal = {fit_chain_subtotal:.2f}s")
    return rows, grand_total, modules_1_5_subtotal, fit_chain_subtotal


if __name__ == "__main__":
    results = {}
    for label, path in (("Dubai", DUBAI_FILE), ("v3", V3_FILE)):
        results[label] = run_timed(path, label)

    print(f"\n{'='*70}\nSUMMARY\n{'='*70}")
    for label, (rows, grand_total, m15, fit) in results.items():
        print(f"{label}: grand_total={grand_total:.1f}s  modules_1_5={m15:.1f}s  fit_chain={fit:.1f}s")
