# WP-A item 2: filter sensitivity sweep for Module 4b's worst-phase CS
# ratios. Read-only diagnostic, Tier B -- no edits to modules/ or config/.
# The production cs_filter_cutoff_hz (2 Hz, chair-identical) in
# config/parameters.json is never touched; the sweep overrides the value
# only in an in-memory copy of the params dict returned by load_parameters()
# (which does a fresh json.load per call -- nothing here is cached or
# written back). Purpose: separate CS findings that hold up under heavier
# input filtering from ones that are artifacts of the current 2 Hz cutoff,
# before any cutoff change or interpolation/verdict-tier decision is taken.
# No verdict re-classification: config/parameters.json's classification
# thresholds (STRONG_CSF etc.) encode the 2 Hz distribution and are not
# meaningful at other cutoffs, so this script reports raw percentiles and
# instance values only.

import subprocess

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)

DATA_PATH = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
CUTOFFS_HZ = [2.0, 1.5, 1.0, 0.7]
N_REPORT = 10
PERCENTILES = [5, 25, 50, 75, 95]

# Explicitly tracked instances from WP-A item 1's kerb audit, identified by
# (lap_number, stable_corner_id) -- unique across this session's corners
# (verified in item 1: 0 collisions on that key across 56 corners).
NAMED_INSTANCES = {
    "front": [(3, 4, "C4 lap 3 front, -0.552 at 2 Hz")],
    "rear": [(1, 9, "C9 lap 1 rear, -0.721 at 2 Hz, exit_4")],
}


def worst_phase_instances(summaries, stat_key):
    out = []
    for s in summaries:
        worst_val = None
        worst_phase = None
        for phase in PHASE_KEYS:
            v = s["phases"][phase][stat_key]["median"]
            if v == v and (worst_val is None or v < worst_val):
                worst_val = v
                worst_phase = phase
        if worst_val is not None:
            out.append({
                "lap": s["lap_number"], "corner": s["corner_number"],
                "stable_corner_id": s.get("stable_corner_id"),
                "phase": worst_phase, "value": worst_val,
            })
    out.sort(key=lambda r: r["value"])
    return out


def print_percentiles(label, values):
    arr = np.array(values, dtype=float)
    pcts = np.percentile(arr, PERCENTILES)
    print(f"{label} (n={len(arr)}):")
    for p, v in zip(PERCENTILES, pcts):
        tag = "median" if p == 50 else f"p{p}"
        print(f"  {tag:>6} = {v:.3f}")
    print(f"  {'min':>6} = {arr.min():.3f}")
    print(f"  {'max':>6} = {arr.max():.3f}")


def print_top10(label, instances):
    print(f"{label} -- {N_REPORT} most negative worst-phase instances:")
    for rank, r in enumerate(instances[:N_REPORT], 1):
        corner_label = r["corner"] if r["corner"] is not None else "None"
        print(f"  {rank:2d}. lap={r['lap']}  corner={corner_label} (C{r['stable_corner_id']})  "
              f"phase={r['phase']:>15}  value={r['value']:.3f}")


# --- single pipeline run, everything upstream of the CS filter cutoff is
# cutoff-independent (confirmed by reading modules/stability_analysis.py:
# cs_filter_cutoff_hz is read only inside estimate_slip_angles and
# estimate_lateral_forces) and is computed once and reused across the sweep.
data = parse_csv(DATA_PATH)
base_params = load_parameters()
state = prepare_vehicle_state(data["channels"], base_params)
if not state:
    print("State preparation failed - check required channels")
    raise SystemExit

beta = estimate_sideslip(state, base_params)
stab = estimate_yaw_moment_stability(state, beta, base_params, data.get("laps", []))
corners = data.get("corners", [])

print("=" * 78)
print(f"CS filter sensitivity sweep -- {DATA_PATH}")
print(f"Samples: {len(state['time'])} @ {state['sample_rate_hz']:.1f} Hz.  "
      f"Corners detected: {len(corners)}")
print(f"cutoffs swept (in-memory only): {CUTOFFS_HZ} Hz  "
      f"(production config/parameters.json value: "
      f"{base_params['stability_estimation']['cs_filter_cutoff_hz']} Hz)")
print("=" * 78)
print()

per_cutoff = {}

for cutoff in CUTOFFS_HZ:
    params = load_parameters()
    params["stability_estimation"]["cs_filter_cutoff_hz"] = cutoff

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    summaries = summarise_corners(corners, cs, stab, state)

    worst_f = worst_phase_instances(summaries, "cs_ratio_f")
    worst_r = worst_phase_instances(summaries, "cs_ratio_r")
    per_cutoff[cutoff] = {"summaries": summaries, "worst_f": worst_f, "worst_r": worst_r}

    print("-" * 78)
    print(f"CUTOFF = {cutoff} Hz")
    print("-" * 78)
    print(f"corner instances with a valid worst-phase value: "
          f"front={len(worst_f)}  rear={len(worst_r)}  (of {len(summaries)} corners)")
    print()
    print_percentiles("Worst-phase CS_ratio_f percentiles", [r["value"] for r in worst_f])
    print()
    print_percentiles("Worst-phase CS_ratio_r percentiles", [r["value"] for r in worst_r])
    print()
    print_top10("Front axle", worst_f)
    print()
    print_top10("Rear axle", worst_r)
    print()

# --- cross-cutoff comparison ---
print("=" * 78)
print("CROSS-CUTOFF COMPARISON")
print("=" * 78)
print()

for axle, stat_key in [("front", "worst_f"), ("rear", "worst_r")]:
    union_keys = set()
    for cutoff in CUTOFFS_HZ:
        for r in per_cutoff[cutoff][stat_key][:N_REPORT]:
            union_keys.add((r["lap"], r["stable_corner_id"]))
    for lap, stable_id, _note in NAMED_INSTANCES[axle]:
        union_keys.add((lap, stable_id))

    print(f"{axle.upper()} axle -- {len(union_keys)} tracked instances "
          f"(union of each cutoff's top {N_REPORT}, plus named watch items)")
    print()

    for lap, stable_id in sorted(union_keys):
        note = next((n for l, s, n in NAMED_INSTANCES[axle] if l == lap and s == stable_id), None)
        label = f"lap={lap}  C{stable_id}" + (f"  [{note}]" if note else "")
        print(f"  {label}")
        values_by_cutoff = []
        for cutoff in CUTOFFS_HZ:
            match = next((r for r in per_cutoff[cutoff][stat_key]
                          if r["lap"] == lap and r["stable_corner_id"] == stable_id), None)
            if match is None:
                print(f"    {cutoff:>4.1f} Hz: no valid worst-phase value at this cutoff")
                values_by_cutoff.append(None)
            else:
                print(f"    {cutoff:>4.1f} Hz: {match['value']:>7.3f}  (phase={match['phase']})")
                values_by_cutoff.append(match["value"])

        known = [v for v in values_by_cutoff if v is not None]
        if known and all(v < 0 for v in known) and len(known) == len(CUTOFFS_HZ):
            verdict = "FILTER-ROBUST -- stays negative at all four cutoffs"
        elif known and any(v < 0 for v in known):
            verdict = "FILTER-DEPENDENT -- negative at some cutoffs, not others"
        elif known:
            verdict = "never negative in this sweep"
        else:
            verdict = "no valid value at any cutoff"
        print(f"    -> {verdict}")
        print()

# --- plain-language summary ---
print("=" * 78)
print("PLAIN-LANGUAGE SUMMARY (facts only -- no cutoff recommendation, "
      "no config/code change)")
print("=" * 78)

for axle, stat_key in [("front", "worst_f"), ("rear", "worst_r")]:
    p5_by_cutoff = [np.percentile([r["value"] for r in per_cutoff[c][stat_key]], 5) for c in CUTOFFS_HZ]
    n_neg_top10_by_cutoff = [sum(1 for r in per_cutoff[c][stat_key][:N_REPORT] if r["value"] < 0)
                              for c in CUTOFFS_HZ]
    print(f"{axle.upper()}: p5 of the worst-phase distribution across cutoffs "
          f"{CUTOFFS_HZ} Hz = " + ", ".join(f"{v:.3f}" for v in p5_by_cutoff) + ".")
    print(f"  Negative instances within the top {N_REPORT} most extreme, per cutoff: "
          + ", ".join(f"{c}Hz={n}" for c, n in zip(CUTOFFS_HZ, n_neg_top10_by_cutoff)) + ".")
    print()

print("=" * 78)
print("PRODUCTION CONFIG CHECK")
print("=" * 78)
result = subprocess.run(["git", "status", "--porcelain", "config/parameters.json"],
                         capture_output=True, text=True)
if result.stdout.strip():
    print("WARNING -- config/parameters.json shows as modified in git status:")
    print(result.stdout)
else:
    print("git status --porcelain config/parameters.json: clean, no changes detected.")
