# Follow-up: reproduces the ORIGINAL "Mechanism investigation" per-
# SAMPLE pooled median statistic (thesis_notes.md "Mechanism
# investigation: wholesale-negative CS_ratio under ekf_auto_pacejka",
# Finding 1) under the FINAL config, for the same 6 corners, so the
# "pooled medians vs earlier measurements" question has a direct,
# same-statistic answer -- not the worst-lap/worst-instance statistic
# reported throughout Phase 4 (a different quantity).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
TARGET_IDS = (1, 2, 3, 4, 8, 9)

# Original Mechanism investigation's own recorded medians (thesis_notes.md,
# Finding 1) -- for direct side-by-side comparison.
ORIGINAL = {
    "front": {1: 0.614, 2: 0.226, 3: 0.123, 4: 0.412, 8: 0.314, 9: 0.603},
    "rear":  {1: 0.563, 2: 0.246, 3: 0.021, 4: 0.321, 8: 0.242, 9: 0.513},
}


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason})")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)

    t = state["time"]
    laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
    corners_by_id = {}
    for c in data.get("corners", []):
        cid = c.get("stable_corner_id")
        lap = laps_by_number.get(c["lap_number"])
        if cid is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        corners_by_id.setdefault(cid, []).append(c)

    print(f"{'corner':>8} {'axle':>6} {'ORIGINAL (pre-repair)':>24} {'FINAL (this config)':>22} {'n_pooled':>10}")
    for cid in TARGET_IDS:
        insts = corners_by_id.get(cid, [])
        for axle_label, cs_key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
            pooled = []
            for c in insts:
                start_t, _ = c["segments"]["entry_1_brake"]
                _, end_t = c["segments"]["exit_5"]
                lo = int(np.searchsorted(t, start_t, side="left"))
                hi = int(np.searchsorted(t, end_t, side="right"))
                if hi > lo:
                    vals = cs[cs_key][lo:hi]
                    pooled.extend(vals[np.isfinite(vals)])
            median = float(np.median(pooled)) if pooled else float("nan")
            orig = ORIGINAL[axle_label][cid]
            print(f"{'C' + str(cid):>8} {axle_label:>6} {orig:>24.3f} {median:>22.3f} {len(pooled):>10d}")


if __name__ == "__main__":
    main()
