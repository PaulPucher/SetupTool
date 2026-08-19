# Quantify the divergence monitor's structural blind
# spot. nis_flag_fraction=1.0 with a 25-sample window requires EVERY
# sample in the window to exceed the chi-square bound before the
# monitor can trigger -- by construction it cannot flag any contiguous
# exceedance run shorter than 25 samples (0.5s at 50Hz), no matter how
# extreme those samples' own NIS values are. Read-only, no config change.
#
# Convention: runs are found within the masked population's own
# temporal sub-sequence (valid-lap, moving, kerb-excluded, in original
# time order) -- i.e. two masked samples adjacent in the masked
# subsequence are treated as consecutive for run purposes even if
# samples were excluded between them in the full array. This matches
# how the other pass-0 EKF diagnostics in this folder frame "over the masked
# population" and is stated explicitly here since a full-array
# convention (breaking runs at every mask gap too) would give a
# slightly different, but not materially different, picture.

import numpy as np
from scipy.stats import chi2

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
WINDOW = 25  # the proposed (not applied) window width from the previous turn

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

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

result = estimate_sideslip_ekf_dugoff(state, params)
nis_masked = result["nis"][base_mask]

chi2_bound_df2 = float(chi2.ppf(0.95, df=2))
exceed = nis_masked > chi2_bound_df2

# contiguous run-length decomposition of the boolean `exceed` sequence
diff = np.diff(exceed.astype(int))
run_starts = np.where(diff == 1)[0] + 1
run_ends = np.where(diff == -1)[0] + 1
if exceed[0]:
    run_starts = np.concatenate(([0], run_starts))
if exceed[-1]:
    run_ends = np.concatenate((run_ends, [len(exceed)]))
run_lengths = run_ends - run_starts

short_mask = run_lengths < WINDOW
long_mask = ~short_mask

n_short_runs = int(short_mask.sum())
n_short_samples = int(run_lengths[short_mask].sum())
n_long_runs = int(long_mask.sum())
n_long_samples = int(run_lengths[long_mask].sum())

print("=" * 100)
print(f"NIS monitor blind spot (window={WINDOW}, "
      f"combined df=2 95% bound={chi2_bound_df2:.4f})")
print("=" * 100)
print(f"masked population: {len(nis_masked)} samples   total samples exceeding bound: {int(exceed.sum())}")
print(f"total exceedance runs: {len(run_lengths)}")
print()
print(f"runs SHORTER than {WINDOW} samples (structurally invisible to this window/fraction): "
      f"{n_short_runs} runs, {n_short_samples} samples")
if n_short_runs:
    p50, p90 = np.percentile(run_lengths[short_mask], [50, 90])
    print(f"  short-run length distribution: p50={p50:.1f}  p90={p90:.1f}  max={int(run_lengths[short_mask].max())}")
print()
print(f"runs >= {WINDOW} samples (the only ones this window COULD ever flag, subject also to "
      f"nis_flag_fraction=1.0 requiring the full window to exceed): {n_long_runs} runs, {n_long_samples} samples")
if n_long_runs:
    p50l, p90l = np.percentile(run_lengths[long_mask], [50, 90])
    print(f"  long-run length distribution: p50={p50l:.1f}  p90={p90l:.1f}  max={int(run_lengths[long_mask].max())}")
