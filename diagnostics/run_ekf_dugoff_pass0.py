# RUN the frozen pass-0 EKF on Dubai and report -- no validation
# comparisons against the rejected observer yet (that is the next work
# package), no NIS thresholds applied, no refit. Read-only: writes
# nothing, changes no config.
#
# Reporting population: the standard masked population used throughout
# WP-N0/N1/N1b (valid-lap, moving, kerb-excluded) -- the filter itself
# still runs over the FULL session array (it is a sequential recursive
# estimator and cannot skip samples without breaking the recursion); the
# mask is applied only when computing the reported statistics below.

import numpy as np
from scipy.stats import chi2, binom

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_slip_angles, estimate_sideslip
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff, slip_angles
from diagnostics.inspect_ekf_dugoff_sanity_checks import check_h2_vs_ay_consistency

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
cfg = params["tyre_model_ekf"]["pass_0"]

t = state["time"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw
laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask
valid_lap_numbers = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis"))

result = estimate_sideslip_ekf_dugoff(state, params)

print("=" * 100)
print("pass-0 EKF run on Dubai")
print("=" * 100)
print(f"file: {RAW_FILE}   laps used: {valid_lap_numbers}")
print(f"reporting population (valid-lap, moving, kerb-excluded): {int(base_mask.sum())} samples")
print()

# --- Section 1: raw beta distribution and hard-bound hits ------------------

beta_hard_bound_deg = cfg["beta_hard_bound_deg"]
beta_hard_bound_rad = np.radians(beta_hard_bound_deg)
beta_raw = result["beta"][base_mask]
beta_deg = np.degrees(beta_raw)

p1, p25, p50, p75, p99 = np.percentile(beta_deg, [1, 25, 50, 75, 99])
max_abs_beta_deg = float(np.max(np.abs(beta_deg)))
n_hard_bound = int((np.abs(beta_raw) > beta_hard_bound_rad).sum())

print("=" * 100)
print("SECTION 1 -- raw EKF beta distribution")
print("=" * 100)
print(f"beta (deg): p1={p1:.3f}  p25={p25:.3f}  median={p50:.3f}  p75={p75:.3f}  p99={p99:.3f}")
print(f"max |beta| = {max_abs_beta_deg:.3f} deg  (hard bound = {beta_hard_bound_deg} deg)")
print(f"samples exceeding the {beta_hard_bound_deg} deg hard bound: {n_hard_bound} / {int(base_mask.sum())} "
      f"({100 * n_hard_bound / base_mask.sum():.2f}%)")
print()

# --- Section 2: per-channel NIS + proposed window/fraction -----------------

nis_combined = result["nis"][base_mask]
innovation = result["innovation"][base_mask]      # columns: yaw_rate, ay
S_diag = result["S_diag"][base_mask]

nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]

chi2_bound_df1 = float(chi2.ppf(0.95, df=1))
chi2_bound_df2 = float(chi2.ppf(0.95, df=2))

print("=" * 100)
print("SECTION 2 -- NIS distribution per measurement channel")
print("=" * 100)
print(f"chi-square 95% bounds used: df=1 (per-channel) = {chi2_bound_df1:.4f}   "
      f"df=2 (combined, config nis_chi2_bound={cfg['nis_chi2_bound']}) = {chi2_bound_df2:.4f}")
print()

for label, nis_ch in (("yaw_rate", nis_yaw), ("ay", nis_ay)):
    p50c, p90c, p99c = np.percentile(nis_ch, [50, 90, 99])
    frac_exceed = float((nis_ch > chi2_bound_df1).mean())
    print(f"  {label:9s}: p50={p50c:.3f}  p90={p90c:.3f}  p99={p99c:.3f}  "
          f"max={np.max(nis_ch):.3f}  fraction > df1 bound = {frac_exceed:.4f}")

p50cb, p90cb, p99cb = np.percentile(nis_combined, [50, 90, 99])
frac_exceed_combined = float((nis_combined > chi2_bound_df2).mean())
print(f"  combined  : p50={p50cb:.3f}  p90={p90cb:.3f}  p99={p99cb:.3f}  "
      f"max={np.max(nis_combined):.3f}  fraction > df2 bound = {frac_exceed_combined:.4f}")
print()

# Proposed window/trigger-fraction: binomial-tail argument against the
# EMPIRICAL combined-NIS exceedance rate (the statistic the implemented
# monitor actually uses) -- for a candidate window width W, find the
# smallest per-window exceedance fraction whose chance of being reached
# by i.i.d. samples AT THE OBSERVED BASELINE RATE is <=1% (binom.ppf
# upper tail). This keeps ordinary noise from tripping the monitor while
# still catching a sustained rate well above baseline.
p_hat = frac_exceed_combined
if p_hat > 0.5:
    print(f"NOTE: baseline p_hat={p_hat:.4f} is far above the 5% a correctly-calibrated filter "
          f"would show (chi-square 95% bound, by definition, is exceeded 5% of the time under a "
          f"consistent filter). This baseline is itself evidence pass 0's Q/R (seeded from the "
          f"rejected LINEAR observer, ratio-invariance explicitly NOT assumed to transfer -- config "
          f"tyre_model_ekf.pass_0.seeded_from) are miscalibrated for THIS nonlinear filter, not proof "
          f"the vehicle is actually diverging that often. The binomial-tail derivation below still "
          f"produces a numerically well-defined answer against this (elevated) baseline, and its "
          f"downstream trigger behaviour (Section 3) remains informative -- but it should be read as a "
          f"placeholder pending the Q/R sensitivity check, not a validated threshold.")
print(f"Proposed window/fraction derivation (binomial tail, target false-alarm <=1%, "
      f"baseline p_hat={p_hat:.4f} from the combined-NIS exceedance rate above):")
candidates = [10, 15, 20, 25, 30, 40, 50]
proposals = []
for W in candidates:
    k = int(binom.ppf(0.99, W, p_hat)) + 1
    k = min(k, W)
    frac = k / W
    proposals.append((W, k, frac))
    print(f"  W={W:3d}: smallest k with P(Binom(W,p_hat)>=k)<=1% -> k={k}  trigger_fraction={frac:.3f}")
print()

# Pick one to propose: W around a typical corner-phase timescale (0.4-0.6s
# at 50Hz = 20-30 samples) balances statistical robustness (longer window,
# tighter false-alarm control) against responsiveness (a real divergence
# episode should be caught within roughly one phase, not span several).
chosen_W, chosen_k, chosen_frac = proposals[3]  # W=25
print(f"PROPOSED (not applied): nis_window_samples={chosen_W}, "
      f"nis_flag_fraction={chosen_frac:.3f} (>= {chosen_k}/{chosen_W} samples over bound in the window). "
      f"Reasoning: {chosen_W} samples = {chosen_W/state['sample_rate_hz']:.2f}s at "
      f"{state['sample_rate_hz']:.0f} Hz, comparable to a single corner phase's own typical duration "
      f"(entry/apex/exit windows in this session run tens of samples each) -- long enough that the "
      f"binomial tail argument controls false alarms from ordinary per-sample chi-square noise at the "
      f"observed baseline rate, short enough that a genuine divergence episode is caught within roughly "
      f"one phase rather than persisting across several before flagging.")
print()

# --- Section 3: divergence monitor output under the proposed thresholds ----

print("=" * 100)
print("SECTION 3 -- divergence monitor output under the proposed thresholds (not applied to config)")
print("=" * 100)

exceed_combined_full = result["nis"] > chi2_bound_df2  # full-length, for correct rolling-window causality
window_trigger_full = np.zeros(len(exceed_combined_full), dtype=bool)
run = []
for i in range(len(exceed_combined_full)):
    if not moving[i]:
        run = []
        continue
    run.append(bool(exceed_combined_full[i]))
    if len(run) > chosen_W:
        run.pop(0)
    if len(run) == chosen_W and (sum(run) / chosen_W) >= chosen_frac:
        window_trigger_full[i] = True

bound_trigger_full = np.abs(result["beta"]) > beta_hard_bound_rad
diverged_proposed_full = window_trigger_full | bound_trigger_full
diverged_proposed = diverged_proposed_full[base_mask]

n_window_trigger = int(window_trigger_full[base_mask].sum())
n_bound_trigger = int(bound_trigger_full[base_mask].sum())
n_total_fallback = int(diverged_proposed.sum())

# contiguous episode count, within the reporting population's own index run
diff = np.diff(diverged_proposed_full.astype(int))
n_episode_starts = int((diff == 1).sum()) + (1 if diverged_proposed_full[0] else 0)

print(f"NIS-window triggers (masked population): {n_window_trigger}")
print(f"hard-|beta|-bound triggers (masked population): {n_bound_trigger}")
print(f"total samples the fallback would replace (union, masked population): {n_total_fallback} "
      f"({100 * n_total_fallback / base_mask.sum():.2f}% of {int(base_mask.sum())})")
print(f"distinct contiguous trigger episodes (full session): {n_episode_starts}")
print()

# Clustering by corner / phase
phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
corner_of_sample = np.full(len(t), -1, dtype=int)
phase_of_sample = np.full(len(t), "", dtype=object)
for c in data.get("corners", []):
    sid = c.get("stable_corner_id")
    if sid is None:
        continue
    for phase in phase_keys:
        start_t, end_t = c["segments"][phase]
        if end_t < start_t:
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if hi <= lo:
            continue
        corner_of_sample[lo:hi] = sid
        phase_of_sample[lo:hi] = phase

flagged_idx = np.where(diverged_proposed_full & base_mask)[0]
if len(flagged_idx) == 0:
    print("No flagged samples in the masked population -- no clustering to report.")
else:
    corners_flagged = corner_of_sample[flagged_idx]
    phases_flagged = phase_of_sample[flagged_idx]
    outside_corner = int((corners_flagged == -1).sum())
    print(f"flagged samples outside any detected corner window: {outside_corner} / {len(flagged_idx)}")
    print("flagged-sample counts by stable_corner_id (corners with >0 only):")
    for sid in sorted(set(corners_flagged) - {-1}):
        print(f"  C{sid}: {int((corners_flagged == sid).sum())}")
    print("flagged-sample counts by phase (samples inside a detected corner only):")
    for phase in phase_keys:
        n_phase = int((phases_flagged == phase).sum())
        if n_phase:
            print(f"  {phase}: {n_phase}")
print()

# --- Section 4: saturation coverage against the frozen curves --------------

c_alpha_f, c_alpha_r = cfg["c_alpha_front_n_per_rad"], cfg["c_alpha_rear_n_per_rad"]
mu_fz_f, mu_fz_r = cfg["mu_fz_front_N"], cfg["mu_fz_rear_N"]

tan_onset_f = mu_fz_f / (2.0 * c_alpha_f)
tan_onset_r = mu_fz_r / (2.0 * c_alpha_r)
onset_f_deg = np.degrees(np.arctan(tan_onset_f))
onset_r_deg = np.degrees(np.arctan(tan_onset_r))

beta_kinematic = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta_kinematic, params)
alpha_f_deg = np.degrees(slip["alpha_f_filt"])[base_mask]
alpha_r_deg = np.degrees(slip["alpha_r_filt"])[base_mask]

print("=" * 100)
print("SECTION 4 -- saturation coverage against the frozen pass-0 Dugoff curves")
print("=" * 100)
print(f"onset (lambda=1) boundary, verified from config: "
      f"front tan(alpha)=mu_fz_f/(2*c_alpha_f)={tan_onset_f:.6f} -> {onset_f_deg:.3f} deg   "
      f"rear tan(alpha)=mu_fz_r/(2*c_alpha_r)={tan_onset_r:.6f} -> {onset_r_deg:.3f} deg")
print()

for label, alpha_deg, onset_deg in (("front", alpha_f_deg, onset_f_deg), ("rear", alpha_r_deg, onset_r_deg)):
    frac_beyond = float((np.abs(alpha_deg) > onset_deg).mean())
    p50a, p90a, p99a = np.percentile(np.abs(alpha_deg), [50, 90, 99])
    print(f"  {label}: fraction |alpha| beyond onset ({onset_deg:.3f} deg) = {frac_beyond:.4f}   "
          f"|alpha| p50={p50a:.3f}  p90={p90a:.3f}  p99={p99a:.3f} deg")
print()

# --- Section 5: sign check on last turn's h2 consistency population --------

print("=" * 100)
print("SECTION 5 -- sign check on the h2 consistency population (reused from last turn)")
print("=" * 100)
h2_pred, ay_meas_pop = check_h2_vs_ay_consistency(state, params, data, verbose=False)

corr = float(np.corrcoef(h2_pred, ay_meas_pop)[0, 1])
slope, intercept = np.polyfit(ay_meas_pop, h2_pred, 1)
print(f"n={len(h2_pred)}  corr(h2_pred, ay_meas) = {corr:.4f}   "
      f"regression slope of h2_pred on ay_meas = {float(slope):.4f}  intercept={float(intercept):.4f}")
