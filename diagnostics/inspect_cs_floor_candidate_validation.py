# WORK PACKAGE: 100 Hz time base + corrected floor derivation. Direct
# REAL-DATA validation of candidate (min_window_s, min_span_rad) floors
# against the actual Phase 4 statistic (worst-lap-per-corner, apex_
# region-substituted CSf/CSr), bypassing the phase-median bootstrap
# entirely for the FINAL choice. Justification: the second pass's
# bootstrap (whole-population sampling) and the third pass's own
# corrected bootstrap (cornering-only population, this same session)
# BOTH show low relative-std ("stable") results for small floors that
# either already failed real Phase 4 validation (second pass) or have
# not yet been checked directly (third pass) -- bootstrap relative std
# measures REPRODUCIBILITY of the resampled median (precision), never
# whether that median's VALUE is close to the true local tangent
# (accuracy): a systematically-biased small window produces a low-
# variance WRONG answer, which no bootstrap-of-the-same-data can ever
# catch. This script checks accuracy directly: does the resulting
# worst-lap distribution actually match the pre-registration?
#
# Parses and fits beta ONCE (the expensive, floor-independent steps),
# then re-runs only estimate_cornering_stiffness/summarise_corners per
# candidate (the cheap-to-vary parts) -- avoids re-paying the parse/fit
# cost for every candidate checked.

import copy
import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

# (min_window_s, min_span_rad) candidates to validate directly against
# real Phase 4 numbers -- spans a range from the third pass's bootstrap-
# favoured small end to a physical scale anchored on the FIRST (pre-
# revision) attempt's own already-Phase-4-validated 2.0 s/0.04 rad
# (100 samples @ 50 Hz), re-expressed at the new 100 Hz grid.
CANDIDATES = [
    (0.10, 0.02), (0.20, 0.02), (0.40, 0.04), (0.60, 0.06), (1.00, 0.04), (2.00, 0.04),
]
FLAGGED_CORNERS = (1, 2, 3, 4, 6)


def _worst_per_instance_apex_region(summaries):
    rows = []
    for s in summaries:
        apex_region = s.get("apex_region")
        csfs, csrs = [], []
        for phase, p in s["phases"].items():
            if phase == "apex_3" and apex_region is not None:
                cs_f = apex_region["cs_ratio_f"]["median"]
                cs_r = apex_region["cs_ratio_r"]["median"]
            else:
                cs_f = p["cs_ratio_f"]["median"]
                cs_r = p["cs_ratio_r"]["median"]
            if cs_f == cs_f:
                csfs.append(cs_f)
            if cs_r == cs_r:
                csrs.append(cs_r)
        rows.append({
            "stable_corner_id": s.get("stable_corner_id"), "lap_number": s.get("lap_number"),
            "worst_csf": min(csfs) if csfs else float("nan"),
            "worst_csr": min(csrs) if csrs else float("nan"),
        })
    return rows


def _worst_lap_per_corner(rows, key):
    by_id = {}
    for r in rows:
        cid, val = r["stable_corner_id"], r[key]
        if cid is None or val != val:
            continue
        by_id.setdefault(cid, []).append(val)
    return {cid: min(vals) for cid, vals in by_id.items()}


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    sample_rate_hz = state["sample_rate_hz"]
    print(f"grid: {state['grid_rate_status']}")

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to validate against it")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    for min_window_s, min_span_rad in CANDIDATES:
        p = copy.deepcopy(params)
        se = p["stability_estimation"]
        se["cs_min_window_s"] = min_window_s
        se["cs_min_slip_angle_span_rad"] = min_span_rad
        n_samples_at_grid = round(min_window_s * sample_rate_hz)

        cs = estimate_cornering_stiffness(slip, forces, state, p)
        summaries = summarise_corners(data["corners"], cs, stab, state)

        rows = _worst_per_instance_apex_region(summaries)
        wl_csf = _worst_lap_per_corner(rows, "worst_csf")
        wl_csr = _worst_lap_per_corner(rows, "worst_csr")
        n_neg_f = sum(1 for v in wl_csf.values() if v < 0)
        n_neg_r = sum(1 for v in wl_csr.values() if v < 0)

        print(f"\n{'=' * 90}\nCANDIDATE: min_window_s={min_window_s} ({n_samples_at_grid} samples @ "
              f"{sample_rate_hz:.0f} Hz), min_span_rad={min_span_rad}\n{'=' * 90}")
        print(f"  worst-lap CSf negative: {n_neg_f}/{len(wl_csf)}  |  worst-lap CSr negative: {n_neg_r}/{len(wl_csr)}")
        print("  CSf:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csf.items(), key=lambda kv: kv[1])])
        print("  CSr:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csr.items(), key=lambda kv: kv[1])])
        print("  flagged corners:", {f"C{cid}": (round(wl_csf.get(cid, float('nan')), 3),
                                                  round(wl_csr.get(cid, float('nan')), 3))
                                      for cid in FLAGGED_CORNERS})


if __name__ == "__main__":
    main()
