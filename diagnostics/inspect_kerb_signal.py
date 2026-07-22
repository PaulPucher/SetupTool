from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state
)
import numpy as np

data = parse_csv('C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt')
params = load_parameters()
state = prepare_vehicle_state(data['channels'], params)

if state is None:
    print("State prep failed")
    raise SystemExit

az = state.get("az_g")
moving = state["moving_mask"]
kerb = state.get("kerb_mask")

if az is None:
    print("log_acc_z not loaded -- channel missing or filter rejected it")
    raise SystemExit

az_mov = az[moving]
print(f"Vertical accel statistics (moving samples, {moving.sum()} total):")
print(f"  Min:    {np.min(az_mov):.3f} g")
print(f"  Max:    {np.max(az_mov):.3f} g")
print(f"  Mean:   {np.mean(az_mov):.3f} g")
print(f"  Median: {np.median(az_mov):.3f} g")
print(f"  Std:    {np.std(az_mov):.3f} g")
print(f"  p5:     {np.percentile(az_mov, 5):.3f} g")
print(f"  p95:    {np.percentile(az_mov, 95):.3f} g")
print(f"  p99:    {np.percentile(az_mov, 99):.3f} g")
print(f"  p99.5:  {np.percentile(az_mov, 99.5):.3f} g")
print(f"  p99.9:  {np.percentile(az_mov, 99.9):.3f} g")

print(f"\nDistribution of |az - median|:")
deviation = np.abs(az_mov - np.median(az_mov))
print(f"  Mean dev:   {np.mean(deviation):.3f} g")
print(f"  p95 dev:    {np.percentile(deviation, 95):.3f} g")
print(f"  p99 dev:    {np.percentile(deviation, 99):.3f} g")
print(f"  p99.5 dev:  {np.percentile(deviation, 99.5):.3f} g")
print(f"  p99.9 dev:  {np.percentile(deviation, 99.9):.3f} g")
print(f"  Max dev:    {np.max(deviation):.3f} g")

if kerb is not None:
    kerb_mov = kerb[moving]
    flagged = int(kerb_mov.sum())
    print(f"\nCurrent kerb mask (baseline {params['stability_estimation']['kerb_baseline_g']:.1f}g, "
          f"threshold {params['stability_estimation']['kerb_z_deviation_threshold_g']:.2f}g, "
          f"dilation ±{params['stability_estimation']['kerb_dilation_samples']} samples):")
    print(f"  Flagged moving samples: {flagged} / {moving.sum()} ({flagged/moving.sum()*100:.1f}%)")
else:
    print("\nkerb_mask is None")