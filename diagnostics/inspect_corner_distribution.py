# Inspects the distribution of CS_ratio and stability medians across all corners
# and phases, so we can pick thresholds that flag the genuinely bad ones rather
# than half the lap.

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)
import numpy as np

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
summaries = summarise_corners(data["corners"], cs, stab, state)

worst_csf_per_corner = []
worst_csr_per_corner = []
worst_stab_per_corner = []

for s in summaries:
    csfs, csrs, stabs = [], [], []
    for phase, p in s["phases"].items():
        cs_f = p["cs_ratio_f"]["median"]
        cs_r = p["cs_ratio_r"]["median"]
        sb = p["stability_observed_Nm_per_deg"]["median"]
        if cs_f == cs_f:
            csfs.append(cs_f)
        if cs_r == cs_r:
            csrs.append(cs_r)
        if sb == sb:
            stabs.append(sb)
    if csfs:
        worst_csf_per_corner.append(min(csfs))
    if csrs:
        worst_csr_per_corner.append(min(csrs))
    if stabs:
        worst_stab_per_corner.append(min(stabs))

csf = np.array(worst_csf_per_corner)
csr = np.array(worst_csr_per_corner)
stab = np.array(worst_stab_per_corner)

print(f"Total corners: {len(summaries)}")
print()
print("WORST CSf per corner (front):")
print(f"  min   = {csf.min():.3f}")
print(f"  p10   = {np.percentile(csf, 10):.3f}")
print(f"  p25   = {np.percentile(csf, 25):.3f}")
print(f"  p50   = {np.percentile(csf, 50):.3f}")
print(f"  p75   = {np.percentile(csf, 75):.3f}")
print(f"  p90   = {np.percentile(csf, 90):.3f}")
print(f"  max   = {csf.max():.3f}")
print()
print("WORST CSr per corner (rear):")
print(f"  min   = {csr.min():.3f}")
print(f"  p10   = {np.percentile(csr, 10):.3f}")
print(f"  p25   = {np.percentile(csr, 25):.3f}")
print(f"  p50   = {np.percentile(csr, 50):.3f}")
print(f"  p75   = {np.percentile(csr, 75):.3f}")
print(f"  p90   = {np.percentile(csr, 90):.3f}")
print(f"  max   = {csr.max():.3f}")
print()
print("WORST stability per corner (Nm/deg):")
print(f"  min   = {stab.min():.0f}")
print(f"  p10   = {np.percentile(stab, 10):.0f}")
print(f"  p25   = {np.percentile(stab, 25):.0f}")
print(f"  p50   = {np.percentile(stab, 50):.0f}")
print(f"  p75   = {np.percentile(stab, 75):.0f}")
print(f"  p90   = {np.percentile(stab, 90):.0f}")
print(f"  max   = {stab.max():.0f}")
print()
print("How many corners would each threshold flag?")
print()
print("CSf threshold:")
for t in [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]:
    print(f"  CSf < {t:.2f} : {(csf < t).sum()} / {len(csf)} corners")
print()
print("CSr threshold:")
for t in [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]:
    print(f"  CSr < {t:.2f} : {(csr < t).sum()} / {len(csr)} corners")
print()
print("Stability threshold:")
for t in [0, -100, -200, -500, -1000, -2000]:
    print(f"  Stab < {t:5d} : {(stab < t).sum()} / {len(stab)} corners")