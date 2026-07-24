# B3 verification: full verdict distribution over the 51 corner instances
# under the re-derived config/parameters.json classification thresholds.
# Replicates ui/views/outing_form.py's _classify_corner logic (read-only,
# no PyQt needed) so this can run headless; reads the same config values
# _classify_corner now reads, rather than hardcoding them a second time.

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
summaries = summarise_corners(data["corners"], cs, stab, state)

cls = params["classification"]
STRONG_CSF = cls["STRONG_CSF"]["value"]
STRONG_CSR = cls["STRONG_CSR"]["value"]
MODERATE_CSF = cls["MODERATE_CSF"]["value"]
MODERATE_CSR = cls["MODERATE_CSR"]["value"]
STAB_NEG_THRESH = cls["stab_neg_thresh_Nm_per_deg"]["value"]

print(f"Config thresholds in use: STRONG_CSF={STRONG_CSF} STRONG_CSR={STRONG_CSR} "
      f"MODERATE_CSF={MODERATE_CSF} MODERATE_CSR={MODERATE_CSR} STAB_NEG_THRESH={STAB_NEG_THRESH}")


def classify(summary):
    worst_f_val, worst_r_val, worst_stab_val = 1.0, 1.0, 1e9
    worst_f_phase = worst_r_phase = worst_stab_phase = None
    for phase, p in summary["phases"].items():
        csf = p["cs_ratio_f"]["median"]
        csr = p["cs_ratio_r"]["median"]
        sob = p["stability_observed_Nm_per_deg"]["median"]
        if csf == csf and csf < worst_f_val:
            worst_f_val, worst_f_phase = csf, phase
        if csr == csr and csr < worst_r_val:
            worst_r_val, worst_r_phase = csr, phase
        if sob == sob and sob < worst_stab_val:
            worst_stab_val, worst_stab_phase = sob, phase

    front_strong = worst_f_val < STRONG_CSF
    rear_strong = worst_r_val < STRONG_CSR
    front_moderate = STRONG_CSF <= worst_f_val < MODERATE_CSF
    rear_moderate = STRONG_CSR <= worst_r_val < MODERATE_CSR
    destabilising = worst_stab_val == worst_stab_val and worst_stab_val < STAB_NEG_THRESH
    cs_active = front_strong or rear_strong or front_moderate or rear_moderate

    if (front_strong or rear_strong) and destabilising:
        severity = "strong"
    elif front_strong or rear_strong:
        severity = "moderate"
    elif (front_moderate or rear_moderate) and destabilising:
        severity = "moderate"
    elif destabilising:
        severity = "moderate"
    else:
        severity = "normal"

    if severity == "normal":
        branch = None
    elif cs_active and destabilising:
        branch = "both"
    elif cs_active:
        branch = "CS"
    else:
        branch = "stability"

    return severity, branch, worst_f_val, worst_r_val, worst_stab_val, worst_stab_phase


counts = {"strong": 0, "moderate": 0, "normal": 0}
branch_counts = {"CS": 0, "stability": 0, "both": 0}
non_normal = []

for s in summaries:
    severity, branch, wf, wr, wstab, wphase = classify(s)
    counts[severity] += 1
    if branch:
        branch_counts[branch] += 1
        non_normal.append((s["lap_number"], s["corner_number"], s.get("stable_corner_id"),
                            severity, branch, wf, wr, wstab, wphase))

print(f"\nVerdict distribution (51 instances): "
      f"strong={counts['strong']}  moderate={counts['moderate']}  normal={counts['normal']}")
print(f"Comparison vs 2026-06-29 reference (0 strong / 23 moderate / 49 normal, "
      f"different N -- that reference was over a different lap composition, see B2 report)")

print(f"\nVerdicts driven by branch: CS={branch_counts['CS']}  "
      f"stability-only={branch_counts['stability']}  both={branch_counts['both']}")

print("\nAll non-normal corner instances:")
for lap_n, corner_n, sid, severity, branch, wf, wr, wstab, wphase in non_normal:
    print(f"  lap={lap_n} corner={corner_n} stable_id={sid}: {severity} (branch={branch}) "
          f"worst_CSf={wf:.3f} worst_CSr={wr:.3f} worst_stab={wstab:.1f} ({wphase})")
