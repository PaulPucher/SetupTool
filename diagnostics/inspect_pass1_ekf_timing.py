# WP-N2 Step 1a: wall-clock timing only. Answers whether the pass-1
# EKF (diagnostics/sideslip_ekf_dugoff.py, a per-sample Python loop
# with a 2x2 matrix inversion each step) is fast enough to wire into
# the production pipeline, before any wiring is attempted (PLAN.md
# STEP 1, sub-step 1a). Read-only: no config or production code
# touched, no wiring performed here.
#
# "Full outing analysis" is timed as the same module sequence
# test_stability.py exercises (parse_csv, which includes corner
# detection via modules.corner_analysis.analyse_corners, through
# summarise_corners) -- the established proxy for the pipeline the
# UI's Analyse button runs.
#
# The "with EKF substituted" total is not a second full run: since
# estimate_slip_angles/estimate_lateral_forces/estimate_cornering_
# stiffness/estimate_yaw_moment_stability operate on fixed-size arrays
# regardless of which beta array they are fed, their cost does not
# change with the beta source. The projection is therefore
# production_total - kinematic_sideslip_time + ekf_time, not a
# separately re-timed pipeline.

import platform
import time

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters,
    prepare_vehicle_state,
    estimate_sideslip,
    estimate_slip_angles,
    estimate_lateral_forces,
    estimate_cornering_stiffness,
    estimate_yaw_moment_stability,
    summarise_corners,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
N_REPS = 5


def timed(fn, *args, **kwargs):
    t0 = time.perf_counter()
    result = fn(*args, **kwargs)
    return result, time.perf_counter() - t0


print("=" * 100)
print("MACHINE / ENVIRONMENT")
print("=" * 100)
print(f"  platform: {platform.platform()}")
print(f"  processor: {platform.processor()}")
print(f"  python: {platform.python_version()}")
print(f"  numpy: {np.__version__}")
print()

per_module_reps = {k: [] for k in [
    "parse_csv", "load_parameters", "prepare_vehicle_state",
    "estimate_sideslip_kinematic", "estimate_slip_angles",
    "estimate_lateral_forces", "estimate_cornering_stiffness",
    "estimate_yaw_moment_stability", "summarise_corners",
]}
ekf_reps = []
production_total_reps = []
n_samples = None

for rep in range(N_REPS):
    data, t_parse = timed(parse_csv, RAW_FILE)
    params, t_params = timed(load_parameters)
    state, t_state = timed(prepare_vehicle_state, data["channels"], params)
    n_samples = len(state["time"])

    beta_kin, t_beta_kin = timed(estimate_sideslip, state, params)
    slip, t_slip = timed(estimate_slip_angles, state, beta_kin, params)
    forces, t_forces = timed(estimate_lateral_forces, state, params)
    cs, t_cs = timed(estimate_cornering_stiffness, slip, forces, state, params)
    stab, t_stab = timed(estimate_yaw_moment_stability, state, beta_kin, params, data.get("laps", []))
    corners = data.get("corners", [])
    _, t_summ = timed(summarise_corners, corners, cs, stab, state)

    per_module_reps["parse_csv"].append(t_parse)
    per_module_reps["load_parameters"].append(t_params)
    per_module_reps["prepare_vehicle_state"].append(t_state)
    per_module_reps["estimate_sideslip_kinematic"].append(t_beta_kin)
    per_module_reps["estimate_slip_angles"].append(t_slip)
    per_module_reps["estimate_lateral_forces"].append(t_forces)
    per_module_reps["estimate_cornering_stiffness"].append(t_cs)
    per_module_reps["estimate_yaw_moment_stability"].append(t_stab)
    per_module_reps["summarise_corners"].append(t_summ)

    production_total = (t_parse + t_params + t_state + t_beta_kin + t_slip
                         + t_forces + t_cs + t_stab + t_summ)
    production_total_reps.append(production_total)

    _, t_ekf = timed(estimate_sideslip_ekf_dugoff, state, params, "pass_1")
    ekf_reps.append(t_ekf)

    print(f"rep {rep + 1}/{N_REPS}: production_total={production_total:.4f}s  ekf_pass1={t_ekf:.4f}s")

print()
print("=" * 100)
print(f"DATA: n_samples={n_samples}")
print("=" * 100)
print()

print("=" * 100)
print("PER-MODULE BREAKDOWN (production pipeline, today's kinematic beta) -- mean +/- std over reps, seconds")
print("=" * 100)
for k, vals in per_module_reps.items():
    arr = np.array(vals)
    print(f"  {k:32s}  mean={arr.mean():.4f}  std={arr.std():.4f}  min={arr.min():.4f}  max={arr.max():.4f}")
print()

prod_arr = np.array(production_total_reps)
ekf_arr = np.array(ekf_reps)
kin_arr = np.array(per_module_reps["estimate_sideslip_kinematic"])

print("=" * 100)
print("TOTALS")
print("=" * 100)
print(f"  production full-outing total : mean={prod_arr.mean():.4f}s  std={prod_arr.std():.4f}s  "
      f"min={prod_arr.min():.4f}s  max={prod_arr.max():.4f}s")
print(f"  pass_1 EKF alone              : mean={ekf_arr.mean():.4f}s  std={ekf_arr.std():.4f}s  "
      f"min={ekf_arr.min():.4f}s  max={ekf_arr.max():.4f}s")
print(f"  kinematic estimate_sideslip   : mean={kin_arr.mean():.4f}s  std={kin_arr.std():.4f}s  "
      f"min={kin_arr.min():.4f}s  max={kin_arr.max():.4f}s")

projected = prod_arr.mean() - kin_arr.mean() + ekf_arr.mean()
print()
print(f"  PROJECTED full-outing total with pass_1 EKF substituted for kinematic beta: "
      f"{projected:.4f}s  (= production_total_mean - kinematic_mean + ekf_mean)")
print(f"  per-sample EKF cost: {ekf_arr.mean() / n_samples * 1000:.4f} ms/sample "
      f"({n_samples} samples)")
