# Q-FIT-LIT step B -- axle-load census for the literature-bounded fit L
# (PLAN.md queue entry Q-FIT-LIT, author EXECUTE 2026-10-18). Read-only.
#
# Channel census rule: which load source the measured cascade
# (stability_analysis.estimate_vertical_loads, vertical_load_source
# "measured") actually uses, read from each file, per corner:
#   - share of samples sourced damper / reconstructed / static_fallback on
#     (a) the fit population (tyre_fit_auto base mask x speed window, finite
#     kinematic alpha and Fy) and (b) all moving analysis-lap samples;
#   - log_dms_dam_* channel quality and the event zero entry status;
#   - on MLA-SE07_US-v2: whether the log_dms_dam_fr channel_correction
#     precondition matched (parse once with, once without the correction
#     block; the FR arrays differ iff it was applied).
# STOP RULE (frozen, author): any corner whose non-damper share exceeds 1 %
# of the moving analysis-lap samples on any session -> STOP after the census.
# Run: python -m diagnostics.inspect_fz_census

import copy
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402

OUT_DIR = "diagnostics/output/fz_census"
SESSIONS = (
    ("NBR-73-010-026-JDR", "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt", False),
    ("Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt", True),
    ("NBR JST outing 29", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt", False),
)
CORNERS = ("fl", "fr", "rl", "rr")
SOURCES = ("damper", "reconstructed", "static_fallback")
STOP_SHARE = 0.01
DAMPER_CH = {c: f"log_dms_dam_{c}" for c in CORNERS}

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def fr_correction_applied(path):
    import modules.csv_parser as cp
    with_corr = cp.parse_csv(path)
    orig = cp.load_channels_config

    def no_corr():
        cfg = copy.deepcopy(orig())
        cfg["channel_corrections"] = {}
        return cfg

    cp.load_channels_config = no_corr
    try:
        without = cp.parse_csv(path)
    finally:
        cp.load_channels_config = orig
    a, b = with_corr["channels"]["log_dms_dam_fr"]["data"], without["channels"]["log_dms_dam_fr"]["data"]
    raw_mean = float(np.mean(b)) if b is not None and len(b) else float("nan")
    return (not np.array_equal(a, b)), raw_mean, without["channels"]["log_dms_dam_fr"]["quality"]


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "census.log"), "w", encoding="utf-8")
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, load_car_data, prepare_vehicle_state,
                                            estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
                                            estimate_vertical_loads)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    import modules.tyre_fit_auto as tfa

    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))
    log(f"vertical_load_source (config) = {params['stability_estimation']['vertical_load_source']}")
    stop = []
    for label, rel, check_fr in SESSIONS:
        path = resolve_data_path(rel)
        log("=" * 90)
        log(f"{label}: {path}")
        data = parse_csv(path)
        reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
        state = prepare_vehicle_state(data["channels"], params)
        log(f"  event zero entry: {data.get('event_zero_entry')}")
        for c in CORNERS:
            ch = data["channels"].get(DAMPER_CH[c]) or {}
            log(f"  {DAMPER_CH[c]}: quality {ch.get('quality')}, unit {ch.get('unit_raw')}")
        for ch_name in ("log_pbrake_f", "log_pbrake_r"):
            log(f"  {ch_name}: quality {(data['channels'].get(ch_name) or {}).get('quality')}")
        if check_fr:
            applied, raw_mean, q_without = fr_correction_applied(path)
            log(f"  FR channel_correction precondition matched: {applied} (raw file mean {raw_mean:.1f}, "
                f"quality without the correction: {q_without})")

        forces = estimate_lateral_forces(state, params)
        fz = estimate_vertical_loads(state, forces, params, channels=data["channels"], car_data=load_car_data())
        log(f"  vertical_load_source_used: {fz['vertical_load_source_used']}")
        src = fz["vertical_load_source_per_sample"]
        if src is None:
            log("  NO per-sample source (static path) -> STOP")
            stop.append((label, "static path"))
            continue

        laps = data["laps"]
        racing = np.zeros(state["time"].size, dtype=bool)
        for lap in laps:
            if lap.get("is_valid_for_analysis"):
                racing |= (state["time"] >= lap["start_time"]) & (state["time"] <= lap["end_time"])
        moving_an = state["moving_mask"] & racing
        base = tfa._base_mask(state, laps)
        window, _ = tfa._speed_window_mask(state, data, params["tyre_fit_auto"])
        slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
        fit_pop = (base & window & np.isfinite(slip["alpha_f_filt"]) & np.isfinite(slip["alpha_r_filt"])
                   & np.isfinite(forces["Fy_f_filt"]) & np.isfinite(forces["Fy_r_filt"]))
        for name, pop in (("fit population", fit_pop), ("moving analysis-lap samples", moving_an)):
            log(f"  {name} (n {int(pop.sum())}):")
            for c in CORNERS:
                lab = np.asarray(src[c])[pop]
                shares = {s: float(np.mean(lab == s)) if lab.size else float("nan") for s in SOURCES}
                other = sorted(set(np.unique(lab)) - set(SOURCES))
                log(f"    {c}: " + ", ".join(f"{s} {shares[s] * 100:.3f} %" for s in SOURCES)
                    + (f"; other labels {other}" if other else ""))
                if name == "moving analysis-lap samples" and 1.0 - shares["damper"] > STOP_SHARE:
                    stop.append((label, f"{c} non-damper {(1.0 - shares['damper']) * 100:.2f} %"))
        for ax in ("f", "r"):
            z = fz[f"fz_{ax}_N"][fit_pop]
            log(f"  axle {ax} Fz on the fit population: median {np.median(z):.0f} N, p5 {np.percentile(z, 5):.0f}, "
                f"p95 {np.percentile(z, 95):.0f}, finite {np.isfinite(z).mean() * 100:.2f} %")
    log("=" * 90)
    if stop:
        log("STOP RULE FIRED: " + "; ".join(f"{a}: {b}" for a, b in stop))
    else:
        log("STOP RULE NOT FIRED: every corner >= 99 % damper on moving analysis-lap samples on all sessions.")
    _LOG.close()
    return 1 if stop else 0


if __name__ == "__main__":
    sys.exit(main())
