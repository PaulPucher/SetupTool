# Damper package, Phase 2: validation of modules/wheel_loads.py against
# GT3_PRC_MLA-v3.txt (the only file with damper/suspension-travel channels
# whitelisted so far). Read-only, no config/production-behaviour changes.
# [keep-reproduces] per diagnostics/README.md -- re-run whenever the
# estimator or a new damper-equipped file changes; the empirical sign-
# convention check here is the thing that would need re-confirming.
#
# Checks (bands, not equalities -- anything outside band is REPORTED, never
# "fixed" here):
#   (a) straight-line total load vs config total car weight
#   (b) fuel-drift trend across the session's straight-line stretches
#   (c) transfer signs/magnitudes vs ax/ay, INCLUDING the empirical check
#       of the travel-channel compression-sign convention the ARB term
#       assumes (module docstring's own flagged assumption)
#   (d) front/rear distribution -- reported only, never asserted

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_lateral_forces, estimate_vertical_loads,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
)

RAW_FILE = "GT3_PRC_MLA-v3.txt"


def main():
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    car_data = load_car_data()
    if car_data is None:
        print("car_data.json not available -- cannot run (motion-ratio/ARB tables required)")
        return

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        print("prepare_vehicle_state returned None")
        return
    n = len(state["time"])
    print(f"state: {n} samples at {state['sample_rate_hz']:.2f} Hz")

    forces = estimate_lateral_forces(state, params)
    fz_static = estimate_vertical_loads(state, forces, params)
    static_fallback_fz = {
        "fl": fz_static["fz_fl_N"], "fr": fz_static["fz_fr_N"],
        "rl": fz_static["fz_rl_N"], "rr": fz_static["fz_rr_N"],
    }

    damper_result = estimate_wheel_loads_from_dampers(state, data["channels"], params, car_data)
    combined = combine_with_static_fallback(damper_result, static_fallback_fz)

    print("\n--- per-corner channel validity fraction (damper source usable) ---")
    for c in CORNERS:
        frac = damper_result[c]["valid"].mean()
        print(f"  {c}: {frac*100:.1f}% samples damper-valid")

    ax = state["ax_mps2"]
    ay = state["ay_mps2"]
    v = state["v_mps"]
    total_fz = sum(combined[c]["fz_N"] for c in CORNERS)
    front_fz = combined["fl"]["fz_N"] + combined["fr"]["fz_N"]
    rear_fz = combined["rl"]["fz_N"] + combined["rr"]["fz_N"]

    moving = v >= params["stability_estimation"]["moving_speed_min_mps"]
    straight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    cornering = moving & (np.abs(ay) > 3.0)

    vp = params["vehicle"]
    config_weight_N = vp["mass_kg"] * 9.81
    print(f"\n--- (a) straight-line total load vs config weight ---")
    print(f"config total weight: {config_weight_N:.1f} N ({vp['mass_kg']:.1f} kg)")
    if straight.any():
        mean_total = float(np.mean(total_fz[straight]))
        pct = (mean_total - config_weight_N) / config_weight_N * 100.0
        print(f"straight-line n={straight.sum()}, mean total Fz={mean_total:.1f} N "
              f"({pct:+.2f}% vs config weight, band +/-5%)")
        print("WITHIN BAND" if abs(pct) <= 5.0 else "OUTSIDE BAND -- reported, not corrected")
    else:
        print("no straight-line samples found under the (|ax|<0.5, |ay|<0.5, moving) mask")

    print(f"\n--- (b) fuel-drift trend across the session (straight-line only) ---")
    for lap in data["laps"]:
        if not lap["is_valid_for_analysis"] and lap["lap_number"] != 0:
            continue
        lap_mask = straight & (state["time"] >= lap["start_time"]) & (state["time"] <= lap["end_time"])
        if lap_mask.sum() < 5:
            continue
        print(f"  lap {lap['lap_number']:>3}: n={lap_mask.sum():>5}, "
              f"mean straight-line total Fz={float(np.mean(total_fz[lap_mask])):.1f} N")

    print(f"\n--- (c) transfer signs/magnitudes vs ax/ay ---")
    braking = moving & (ax < -1.0)
    driving = moving & (ax > 1.0)
    if braking.any():
        print(f"braking (ax<-1.0, n={braking.sum()}): mean front Fz={float(np.mean(front_fz[braking])):.1f} N, "
              f"mean rear Fz={float(np.mean(rear_fz[braking])):.1f} N "
              f"(expect front > rear under load transfer)")
    if straight.any():
        print(f"straight/no-transfer (n={straight.sum()}): mean front Fz={float(np.mean(front_fz[straight])):.1f} N, "
              f"mean rear Fz={float(np.mean(rear_fz[straight])):.1f} N")

    if cornering.any():
        corr_ay_fr = np.corrcoef(ay[cornering], combined["fr"]["fz_N"][cornering])[0, 1]
        corr_ay_fl = np.corrcoef(ay[cornering], combined["fl"]["fz_N"][cornering])[0, 1]
        print(f"cornering (|ay|>3.0, n={cornering.sum()}): corr(ay, Fz_fr)={corr_ay_fr:+.3f}, "
              f"corr(ay, Fz_fl)={corr_ay_fl:+.3f} (expect fr positive/fl negative, "
              f"same convention as modules.stability_analysis.estimate_vertical_loads)")

    # Empirical check of the ARB term's travel-compression sign assumption
    # (module docstring's own flagged item): does (travel_fl - travel_fr)
    # correlate POSITIVELY with ay on real cornering data, as the module's
    # "more negative = more compressed" assumption requires for the ARB
    # term to add load to the correct (outside) wheel?
    travel_fl_ch = data["channels"].get("log_susp_travel_fl")
    travel_fr_ch = data["channels"].get("log_susp_travel_fr")
    if (travel_fl_ch and travel_fr_ch and travel_fl_ch["quality"] == "valid"
            and travel_fr_ch["quality"] == "valid"):
        travel_fl = np.interp(state["time"], travel_fl_ch["time"], travel_fl_ch["data"])
        travel_fr = np.interp(state["time"], travel_fr_ch["time"], travel_fr_ch["data"])
        delta = travel_fl - travel_fr
        if cornering.any():
            corr_delta_ay = np.corrcoef(delta[cornering], ay[cornering])[0, 1]
            print(f"\nARB sign-convention check: corr(travel_fl - travel_fr, ay) = {corr_delta_ay:+.3f} "
                  f"over cornering samples.")
            print("Module assumption requires POSITIVE correlation (fl-fr grows as ay grows, i.e. "
                  "fr's travel value falls -- more negative -- faster than fl's under right-hand load, "
                  "matching 'more negative = more compressed'). "
                  + ("CONFIRMED, no code change needed." if corr_delta_ay > 0
                     else "CONTRADICTED -- SIDE_SIGN/ARB sign in modules/wheel_loads.py needs flipping, "
                          "flagged for the report, not changed by this read-only script."))
    else:
        print("\nARB sign-convention check skipped -- fl/fr travel channels not both valid")

    print(f"\n--- (d) front/rear distribution (reported only, never asserted) ---")
    if moving.any():
        print(f"whole session, moving samples: mean front Fz={float(np.mean(front_fz[moving])):.1f} N "
              f"({float(np.mean(front_fz[moving]))/float(np.mean(total_fz[moving]))*100:.1f}%), "
              f"mean rear Fz={float(np.mean(rear_fz[moving])):.1f} N "
              f"({float(np.mean(rear_fz[moving]))/float(np.mean(total_fz[moving]))*100:.1f}%)")


if __name__ == "__main__":
    main()
