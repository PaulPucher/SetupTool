# B2 diagnostic (PLAN.md WP-ALIGN): out/inlap s_m degeneracy check, Module 5
# threshold re-derivation statistics (old estimator vs new production), and a
# CS-side (Module 4b) consistency check confirming Module 4b is untouched.
#
# The "old" estimator is fetched from git history rather than kept as a
# duplicate file in the tree (B1 removed it from modules/ deliberately --
# git history preserves it). OLD_REF must point at the commit whose
# modules/stability_analysis.py still has the pre-B1 (time-anchored OLS)
# estimator; update it once the B1/B2 change is committed.

import subprocess
import types
import json
import numpy as np

REPO_ROOT = "c:/UNI/Bachelorarbeit/Setuptool_local"
# Pre-B1/B2/B3 commit is 4f19bc8200aba6e4628e1ca1dac97211a51a2879 (last
# commit before this WP-ALIGN work started). OLD_REF="HEAD" is only correct
# until this WP's own commit lands -- after that, set OLD_REF to the hash
# above (still the pre-alignment estimator) rather than "HEAD".
OLD_REF = "HEAD"


def _load_old_module():
    src = subprocess.run(
        ["git", "show", f"{OLD_REF}:modules/stability_analysis.py"],
        cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", check=True,
    ).stdout
    mod = types.ModuleType("old_stability_analysis")
    mod.__dict__["__file__"] = "<git:old_stability_analysis>"
    exec(compile(src, "<git:old_stability_analysis>", "exec"), mod.__dict__)
    return mod


def _load_old_parameters():
    src = subprocess.run(
        ["git", "show", f"{OLD_REF}:config/parameters.json"],
        cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", check=True,
    ).stdout
    return json.loads(src)


from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners, _build_inout_lap_mask,
)
from modules.yaw_stability import calculate_filtered_yaw_acceleration, calculate_observed_stability

old_mod = _load_old_module()
old_params = _load_old_parameters()

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
corners = data.get("corners", [])
laps = data.get("laps", [])

stab_old = old_mod.estimate_yaw_moment_stability(state, beta, old_params)
stab_new_prod = estimate_yaw_moment_stability(state, beta, params, laps)

# --- new, without in/out exclusion (for part 1's degeneracy check only) ---
se = params["stability_estimation"]
vp = params["vehicle"]
t = state["time"]
sr = state["sample_rate_hz"]
kerb_mask = state.get("kerb_mask")
moving_noexcl = state["moving_mask"]
if kerb_mask is not None:
    moving_noexcl = moving_noexcl & ~kerb_mask
Iz = vp["yaw_inertia_kgm2"]
yaw_accel_filt = calculate_filtered_yaw_acceleration(state["yaw_rate_radps"], t, sr, se["yaw_stability_accel_window_s"])
Mz_inertial = Iz * yaw_accel_filt
az_mps2 = state["az_g"] * 9.81 if state.get("az_g") is not None else None
stab_obs_noexcl, stab_valid_noexcl, diag_noexcl = calculate_observed_stability(
    s_m=state["s_m"], beta_rad=beta, delta_f_rad=state["delta_f_rad"], v_mps=state["v_mps"],
    ax_mps2=state["ax_mps2"], az_mps2=az_mps2, mz_inertial_Nm=Mz_inertial, valid_mask=moving_noexcl,
    grid_step_m=se["yaw_stability_grid_step_m"], window_m=se["yaw_stability_window_m"],
    min_samples=se["yaw_stability_min_samples"], ridge=se["yaw_stability_ridge"],
    min_beta_std_rad=se["yaw_stability_min_beta_std_rad"],
)

print("=" * 70)
print("PART 1 -- out/inlap s_m degeneracy check")
print("=" * 70)

lap_distance_ch = data["channels"].get("lap_distance")
ld_time, ld_data = lap_distance_ch["time"], lap_distance_ch["data"] * 0.3048
for lap in laps:
    if lap.get("is_outlap") or lap.get("is_inlap"):
        mask = (ld_time >= lap["start_time"]) & (ld_time <= lap["end_time"])
        d = ld_data[mask]
        kind = "outlap" if lap["is_outlap"] else "inlap"
        print(f"  lap {lap['lap_number']} ({kind}): native lap_distance "
              f"min={d.min():.1f}m max={d.max():.1f}m std={d.std():.1f}m n={len(d)}")

# locate the grid point with ~8301-sample window in the no-inout run
s_m = state["s_m"]
coord_all = s_m[np.isfinite(s_m) & moving_noexcl]
# reconstruct the grid the same way calculate_observed_stability does
work_mask = moving_noexcl & np.isfinite(s_m)
coord_sorted = np.sort(s_m[work_mask])
grid_step = se["yaw_stability_grid_step_m"]
window_m = se["yaw_stability_window_m"]
grid = np.arange(coord_sorted.min(), coord_sorted.max() + grid_step, grid_step)
window_counts = np.array([
    np.searchsorted(coord_sorted, c + window_m, side="right") - np.searchsorted(coord_sorted, c - window_m, side="left")
    for c in grid
])
i_max = int(np.argmax(window_counts))
center = grid[i_max]
print(f"\n  Largest-window grid point: s={center:.1f}m, window population={window_counts[i_max]}")

lo = np.searchsorted(coord_sorted, center - window_m, side="left")
hi = np.searchsorted(coord_sorted, center + window_m, side="right")
window_s_values = coord_sorted[lo:hi]
# attribute each windowed s-sample back to a lap via nearest time (moving_noexcl-masked samples, s_m already computed on t_ref)
t_for_mask = t[work_mask]
s_for_mask = s_m[work_mask]
order = np.argsort(s_for_mask)
t_sorted_by_s = t_for_mask[order]
window_times = t_sorted_by_s[lo:hi]
lap_of_time = np.full(len(window_times), -1)
for lap in laps:
    m = (window_times >= lap["start_time"]) & (window_times <= lap["end_time"])
    lap_of_time[m] = lap["lap_number"]
unique, counts = np.unique(lap_of_time, return_counts=True)
print("  Window membership by lap_number:")
for u, c in zip(unique, counts):
    tag = ""
    for lap in laps:
        if lap["lap_number"] == u:
            tag = "outlap" if lap.get("is_outlap") else ("inlap" if lap.get("is_inlap") else "racing")
    print(f"    lap {u} ({tag}): {c} samples")

# count of samples with interpolated stability ~ -97 in the no-inout run
near_m97 = np.abs(stab_obs_noexcl - (-97.2)) < 1.0
print(f"\n  Samples with interpolated stability ~ -97 Nm/deg (no-inout run): {int(near_m97.sum())}")
lap_of_all = np.full(len(t), -1)
for lap in laps:
    m = (t >= lap["start_time"]) & (t <= lap["end_time"])
    lap_of_all[m] = lap["lap_number"]
u2, c2 = np.unique(lap_of_all[near_m97], return_counts=True)
for u, c in zip(u2, c2):
    tag = ""
    for lap in laps:
        if lap["lap_number"] == u:
            tag = "outlap" if lap.get("is_outlap") else ("inlap" if lap.get("is_inlap") else "racing")
    print(f"    lap {u} ({tag}): {c} of these samples")

print("\n" + "=" * 70)
print("PART 2 -- threshold re-derivation statistics (OLD vs NEW production)")
print("=" * 70)

PCT_LIST = [0.5, 1, 2.5, 5, 10, 25, 50]


def fine_pctiles(arr, label):
    valid = arr[~np.isnan(arr)]
    print(f"\n  {label} (n={len(valid)}):")
    print(f"    min: {valid.min():.1f}")
    for q in PCT_LIST:
        print(f"    p{q}: {np.percentile(valid, q):.1f}")


moving_all = state["moving_mask"]
old_valid = stab_old["stability_valid"] & moving_all
new_valid = stab_new_prod["stability_valid"] & moving_all
old_samples = stab_old["stability_observed_Nm_per_deg"][old_valid]
new_samples = stab_new_prod["stability_observed_Nm_per_deg"][new_valid]

print("\nSample-level stability_observed_Nm_per_deg:")
fine_pctiles(old_samples, "OLD")
fine_pctiles(new_samples, "NEW production")

summ_old = summarise_corners(corners, cs, stab_old, state)
summ_new = summarise_corners(corners, cs, stab_new_prod, state)


def worst_stab(summaries):
    out = []
    for s in summaries:
        vals = [p["stability_observed_Nm_per_deg"]["median"] for p in s["phases"].values()]
        vals = [v for v in vals if v == v]
        out.append((min(vals) if vals else float("nan"), s["lap_number"], s["corner_number"], s.get("stable_corner_id")))
    return out


worst_old = worst_stab(summ_old)
worst_new = worst_stab(summ_new)
worst_old_arr = np.array([w[0] for w in worst_old])
worst_new_arr = np.array([w[0] for w in worst_new])

print("\nWorst-phase-per-corner-instance stability (51 instances):")
fine_pctiles(worst_old_arr, "OLD")
fine_pctiles(worst_new_arr, "NEW production")

print("\nFraction of samples below threshold -- OLD:")
for thr in [0, -100, -200, -300, -500]:
    frac = (old_samples < thr).sum() / len(old_samples)
    print(f"    < {thr:5d}: {frac*100:5.1f}%  ({(old_samples < thr).sum()} / {len(old_samples)})")

print("\nFraction of samples below threshold -- NEW production:")
for thr in [0, -25, -50, -75, -100, -150, -200]:
    frac = (new_samples < thr).sum() / len(new_samples)
    print(f"    < {thr:5d}: {frac*100:5.1f}%  ({(new_samples < thr).sum()} / {len(new_samples)})")

print("\nCorner instances (of 51) below threshold -- OLD:")
for thr in [0, -100, -200, -300, -500]:
    n = int((worst_old_arr < thr).sum())
    print(f"    < {thr:5d}: {n} / 51")

print("\nCorner instances (of 51) below threshold -- NEW production:")
for thr in [0, -25, -50, -75, -100, -150, -200]:
    n = int((worst_new_arr < thr).sum())
    print(f"    < {thr:5d}: {n} / 51")

print("\nAll 51 NEW worst-phase-per-corner-instance stability values, sorted ascending:")
for val, lap_n, corner_n, sid in sorted(worst_new, key=lambda w: w[0]):
    print(f"    {val:8.1f} Nm/deg  lap={lap_n} corner={corner_n} stable_id={sid}")

print("\n" + "=" * 70)
print("PART 3 -- CS-side (Module 4b) consistency check")
print("=" * 70)


def worst_cs(summaries, axle_key):
    out = []
    for s in summaries:
        vals = [p[axle_key]["median"] for p in s["phases"].values()]
        vals = [v for v in vals if v == v]
        if vals:
            out.append(min(vals))
    return np.array(out)


csf_worst = worst_cs(summ_new, "cs_ratio_f")
csr_worst = worst_cs(summ_new, "cs_ratio_r")
print(f"\n  Worst-phase CSf (n={len(csf_worst)}): p10={np.percentile(csf_worst,10):.3f}  p50={np.percentile(csf_worst,50):.3f}")
print(f"  Worst-phase CSr (n={len(csr_worst)}): p10={np.percentile(csr_worst,10):.3f}  p50={np.percentile(csr_worst,50):.3f}")
print("  Reference (2026-06-29 thesis_notes.md): front p10=0.049 p50=0.334; rear p10=0.186 p50=0.749")
