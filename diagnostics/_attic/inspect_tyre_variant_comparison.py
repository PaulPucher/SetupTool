# PLAN.md unsupervised package, Phase 3: Dugoff vs Pacejka comparison.
# Read-only, no config/production change. Runs both modules/
# tyre_fit_auto.py fit chains (fit_session / fit_session_pacejka) on
# Dubai and compares fit RMS, onset/peak location, the shared Phase-2
# validation metrics, and self-consistency R^2 at each filter's own
# alpha. Pre-registered prediction (rear-axle identifiability, not
# merely optimizer convergence): thesis_notes.md "Phase 3: Pacejka
# variant -- pre-registration".

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
)
from modules.tyre_model import dugoff_lateral_force
from modules.tyre_model_pacejka import pacejka_lateral_force
from modules.tyre_fit_auto import fit_session, fit_session_pacejka, _base_mask

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
laps = data.get("laps", [])
base_mask = _base_mask(state, laps)
forces = estimate_lateral_forces(state, params)

print("=" * 100)
print("Phase 3 -- Dugoff vs Pacejka fit + EKF comparison on Dubai")
print("=" * 100)

dugoff = fit_session(data, params, data_file_path=RAW_FILE)
beta_dugoff = dugoff.pop("beta_ekf", None)
pacejka = fit_session_pacejka(data, params, data_file_path=RAW_FILE)
beta_pacejka = pacejka.pop("beta_ekf", None)

print(f"Dugoff status: {dugoff.get('status')}   Pacejka status: {pacejka.get('status')}")
print()

print("--- fit RMS residual per axle (N) ---")
for axle in ("front", "rear"):
    print(f"  {axle}: Dugoff={dugoff['axles'][axle]['mu_fz_fit_rms_resid_N']:.1f}  "
          f"Pacejka={pacejka['axles'][axle]['fit_rms_resid_N']:.1f}")
print()

print("--- onset (Dugoff) / peak (Pacejka) location, deg ---")
for axle in ("front", "rear"):
    d_onset = dugoff["onset_coverage"][axle]["onset_deg"]
    d_cov = dugoff["onset_coverage"][axle]["coverage_fraction"]
    p_peak = pacejka["onset_coverage"][axle]["peak_alpha_deg"]
    p_cov = pacejka["onset_coverage"][axle]["coverage_fraction"]
    p_in_range = pacejka["axles"][axle]["peak_in_visited_range"]
    p_p99 = pacejka["axles"][axle]["visited_alpha_p99_deg"]
    print(f"  {axle}: Dugoff onset={d_onset:.3f} deg, coverage={d_cov:.4f}   |   "
          f"Pacejka peak={p_peak:.3f} deg, coverage={p_cov if p_cov==p_cov else float('nan'):.4f}, "
          f"in_visited_range={p_in_range} (visited p99={p_p99:.3f} deg)")
print()

print("--- validation metrics ---")
print(f"  NIS: Dugoff yaw={dugoff['nis']['yaw_rate_exceedance']:.4f} ay={dugoff['nis']['ay_exceedance']:.4f} "
      f"combined={dugoff['nis']['combined_exceedance']:.4f} mean_nis={dugoff['nis']['combined_mean_nis']:.3f}")
print(f"  NIS: Pacejka yaw={pacejka['nis']['yaw_rate_exceedance']:.4f} ay={pacejka['nis']['ay_exceedance']:.4f} "
      f"combined={pacejka['nis']['combined_exceedance']:.4f} mean_nis={pacejka['nis']['combined_mean_nis']:.3f}")
print(f"  sign check: Dugoff median_gate_racing={dugoff['sign_check']['median_gate_racing']} "
      f"per_sample={dugoff['sign_check']['per_sample_fraction_racing']:.4f}")
print(f"  sign check: Pacejka median_gate_racing={pacejka['sign_check']['median_gate_racing']} "
      f"per_sample={pacejka['sign_check']['per_sample_fraction_racing']:.4f}")
print(f"  h2_vs_ay_apex: Dugoff corr={dugoff['h2_vs_ay_apex']['correlation']:.4f}  "
      f"Pacejka corr={pacejka['h2_vs_ay_apex']['correlation']:.4f}")
print()

# --- self-consistency R^2, each filter's own alpha, methodology: inspect_
# pass1_final_validation.py Section 3 (corner-window population) -------

corners = data.get("corners", [])
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)
s_m = state.get("s_m")
t = state["time"]


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(np.max(lap_s[finite]))
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


in_corner_mask = np.zeros_like(t, dtype=bool)
for cid in stable_ids:
    instances = corners_by_stable_id[cid]
    bracket_start = instances[0].get("bracket_start_m")
    bracket_end = instances[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        continue
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
        if sl.stop > sl.start:
            in_corner_mask[sl] = True
corner_valid_mask = base_mask & in_corner_mask

print("--- self-consistency R^2 at each filter's own alpha (corner-window population) ---")
for label, beta_ekf, fit_manifest, fy_func in (
    ("Dugoff", beta_dugoff, dugoff, None), ("Pacejka", beta_pacejka, pacejka, None),
):
    slip_own = estimate_slip_angles(state, beta_ekf, params)
    for axle_name, alpha_c, Fy in (
        ("front", slip_own["alpha_f_filt"], forces["Fy_f_filt"]),
        ("rear", slip_own["alpha_r_filt"], forces["Fy_r_filt"]),
    ):
        alpha_rad = alpha_c[corner_valid_mask]
        fy_meas = Fy[corner_valid_mask]
        finite = np.isfinite(alpha_rad) & np.isfinite(fy_meas)
        alpha_rad, fy_meas = alpha_rad[finite], fy_meas[finite]
        if label == "Dugoff":
            c_a = fit_manifest["axles"][axle_name]["c_alpha_n_per_rad"]
            mu_z = fit_manifest["axles"][axle_name]["mu_fz_N"]
            fy_pred = dugoff_lateral_force(alpha_rad, c_a, mu_z)
        else:
            ax = fit_manifest["axles"][axle_name]
            fy_pred = pacejka_lateral_force(alpha_rad, ax["B"], ax["C"], ax["D"], ax["E"])
        resid = fy_meas - fy_pred
        ss_res = np.sum(resid ** 2)
        ss_tot = np.sum((fy_meas - np.mean(fy_meas)) ** 2)
        r2 = float(1.0 - ss_res / ss_tot)
        rms = float(np.sqrt(np.mean(resid ** 2)))
        print(f"  {label:8s} {axle_name}: n={len(alpha_rad)}  R^2={r2:.4f}  RMS={rms:.0f} N")
print()

print("=" * 100)
print("REAR-AXLE IDENTIFIABILITY CHECK (pre-registered prediction)")
print("=" * 100)
p_rear = pacejka["axles"]["rear"]
print(f"  Pacejka rear: powell_converged={p_rear['powell_converged']}  peak_alpha_deg={p_rear['peak_alpha_deg']:.3f}  "
      f"peak_in_visited_range={p_rear['peak_in_visited_range']}  visited_p99={p_rear['visited_alpha_p99_deg']:.3f} deg  "
      f"fit_rms={p_rear['fit_rms_resid_N']:.1f} N")
d_rear = dugoff["axles"]["rear"]
print(f"  Dugoff rear (for comparison): mu_fz_bound_fraction={d_rear['mu_fz_bound_fraction']:.4f}  "
      f"onset_deg={dugoff['onset_coverage']['rear']['onset_deg']:.3f}  "
      f"fit_rms={d_rear['mu_fz_fit_rms_resid_N']:.1f} N")
if p_rear["powell_converged"] and not p_rear["peak_in_visited_range"]:
    print("  PREDICTION CONFIRMED: Powell reports convergence (no explicit bound hit) but the rear peak "
          "sits beyond the visited alpha range -- an extrapolated, poorly-identified peak, the silent "
          "failure mode predicted (vs Dugoff's explicit bound-hit failure in the historical refit loop).")
elif not p_rear["powell_converged"]:
    print("  Powell itself failed to converge on the rear axle -- a different, more overt failure mode "
          "than predicted, still consistent with the underlying identifiability concern.")
else:
    print("  PREDICTION NOT CONFIRMED as stated: the rear peak sits inside the visited alpha range -- "
          "record this as a failed prediction, not a discarded one.")
