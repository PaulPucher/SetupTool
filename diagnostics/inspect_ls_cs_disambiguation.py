# PLAN.md STEP 3 (LS_ratio), Phase 4: the combined-slip disambiguation
# check. For every corner instance where rear CS_ratio is low (below
# the population p25), report rear LS_ratio alongside and cluster:
#   "low CS + low LS"  -> traction-limited candidates
#   "low CS + high LS" -> cornering-limited candidates
# Read-only diagnostic. No production path touched, nothing whitelisted,
# no config written. Runs the full Modules 1-6 chain directly (same call
# order as tests/conftest.py's pipeline_result fixture and ui/views/
# outing_form.py's StabilityAnalysisThread.run(), sideslip_source
# asserted "kinematic" below rather than assumed -- PLAN.md's own
# constraint that this package must never change the live config
# default, and thesis_notes.md's standing rule that verification must
# never state a config value from memory).
#
# "low LS" / "high LS" split: population median of LS_ratio among the
# SAME low-CS-rear instance population being clustered (a relative,
# data-derived split -- no LS_ratio classification threshold exists in
# config, and per the work order none is introduced here; DISPLAY-ONLY
# clustering for this diagnostic's own read, not a production rule).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def worst_phase_value(summary, key):
    """Worst (lowest) phase median for `key` across a corner instance's
    5 phases -- same construction ui/views/outing_form.py's
    _classify_corner uses for worst_f_val/worst_r_val (recon-confirmed
    this session), reproduced here rather than imported since
    _classify_corner is a QWidget instance method, not a plain function.
    """
    worst = None
    for phase in PHASE_KEYS:
        v = summary["phases"][phase][key]["median"]
        if v != v:  # NaN
            continue
        if worst is None or v < worst:
            worst = v
    return worst


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()

    live_default = params["stability_estimation"].get("sideslip_source", "kinematic")
    print(f"live config sideslip_source = {live_default!r} (left untouched, not used for this diagnostic)")

    state = prepare_vehicle_state(data["channels"], params)
    assert state is not None, "prepare_vehicle_state returned None -- required channels missing"

    # Deliberately kinematic, independent of the live config value --
    # PLAN.md's own hard constraint ("never change sideslip_source's
    # default") does not license reading a non-default value into a
    # disambiguation check meant to be comparable with everything else
    # this session validated under kinematic.
    beta = estimate_sideslip(state, params)
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip_ratio = estimate_slip_ratio(state, data["channels"], params)
    ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)

    corners = data.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, ls=ls, lap_filter=None)

    laps = data.get("laps", [])
    valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    instances = [s for s in summaries if s["lap_number"] in valid_lap_numbers]
    print(f"corner instances (valid laps): n={len(instances)}")

    rows = []
    for s in instances:
        cs_r = worst_phase_value(s, "cs_ratio_r")
        ls_r = worst_phase_value(s, "ls_ratio_r")
        if cs_r is None:
            continue
        rows.append({
            "lap_number": s["lap_number"],
            "corner_number": s["corner_number"],
            "stable_corner_id": s.get("stable_corner_id"),
            "cs_ratio_r": cs_r,
            "ls_ratio_r": ls_r,
        })

    cs_vals = np.array([r["cs_ratio_r"] for r in rows])
    p25 = float(np.percentile(cs_vals, 25))
    print(f"rear CS_ratio (worst-phase-per-instance) population: n={len(rows)} p25={p25:.4f}")

    low_cs = [r for r in rows if r["cs_ratio_r"] < p25]
    print(f"low-rear-CS instances (< p25): n={len(low_cs)}")

    ls_available = [r for r in low_cs if r["ls_ratio_r"] is not None and np.isfinite(r["ls_ratio_r"])]
    print(f"of those, with a finite rear LS_ratio: n={len(ls_available)}")

    if not ls_available:
        print()
        print("RESULT: diagnostic could not run as intended -- zero low-rear-CS "
              "instances have a finite rear LS_ratio. Every low-CS instance is "
              "reported below with LS_ratio=NaN, unclustered.")
        for r in low_cs:
            print(f"  lap={r['lap_number']} corner={r['corner_number']} "
                  f"stable_id={r['stable_corner_id']} CS_r={r['cs_ratio_r']:.3f} LS_r=NaN")
        return

    ls_vals = np.array([r["ls_ratio_r"] for r in ls_available])
    ls_median = float(np.median(ls_vals))
    print(f"LS_ratio median among these instances: {ls_median:.4f}")

    traction_limited = [r for r in ls_available if r["ls_ratio_r"] < ls_median]
    cornering_limited = [r for r in ls_available if r["ls_ratio_r"] >= ls_median]

    print()
    print(f"'low CS + low LS' (traction-limited candidates): n={len(traction_limited)}")
    for r in traction_limited:
        print(f"  lap={r['lap_number']} corner={r['corner_number']} "
              f"stable_id={r['stable_corner_id']} CS_r={r['cs_ratio_r']:.3f} LS_r={r['ls_ratio_r']:.3f}")
    print()
    print(f"'low CS + high LS' (cornering-limited candidates): n={len(cornering_limited)}")
    for r in cornering_limited:
        print(f"  lap={r['lap_number']} corner={r['corner_number']} "
              f"stable_id={r['stable_corner_id']} CS_r={r['cs_ratio_r']:.3f} LS_r={r['ls_ratio_r']:.3f}")


if __name__ == "__main__":
    main()
