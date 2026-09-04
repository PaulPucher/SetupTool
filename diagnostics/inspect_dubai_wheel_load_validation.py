# Fz-integration Phase 1 (2026-09-03): Dubai wheel-load validation chain.
# Read-only, no config/production changes. Mirrors diagnostics/inspect_v3_
# wheel_load_validation.py's checks (a)/(c)/(d) and inspect_v3_aero_load_
# diagnostic.py's v^2 regression, run here for the FIRST time against
# Sample_Dubai.txt -- the premise-correction finding (Dubai actually HAS
# damper/travel channels, contradicting the original work order's "Dubai
# has no channels" assumption) makes this the required plausibility check
# before any consumption, per the user's explicit decision.
#
# Fuel-drift (v3 script's check (b)) is deliberately NOT reproduced here --
# not requested for this check, and Dubai's 4 valid laps carry far less
# fuel burn than v3's own already-marginal signal.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_lateral_forces, estimate_vertical_loads,
)
from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
)

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
WHEEL_LABELS = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}


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

    print("\n=== (0) per-gauge plausibility -- all 8 channels ===")
    for c in CORNERS:
        force_ch = data["channels"].get(f"log_dms_dam_{c}")
        travel_ch = data["channels"].get(f"log_susp_travel_{c}")
        for label, ch in ((f"log_dms_dam_{c}", force_ch), (f"log_susp_travel_{c}", travel_ch)):
            if ch is None:
                print(f"  {label}: MISSING from this file")
                continue
            d = ch["data"]
            print(f"  {label}: quality={ch['quality']!r} unit_raw={ch.get('unit_raw')!r} "
                  f"mean={np.nanmean(d):.4f} std={np.nanstd(d):.4f} "
                  f"range=[{np.nanmin(d):.4f}, {np.nanmax(d):.4f}]")
        dr = damper_result[c]
        verdict = ("REAL" if dr["valid"].any() and not dr["dead_channel"]
                    else "DEAD/FROZEN" if dr["dead_channel"]
                    else "INVALID (quality/missing)")
        print(f"  -> {WHEEL_LABELS[c]} gauge pair verdict: {verdict} "
              f"({dr['valid'].mean()*100:.1f}% damper-valid samples)")

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
    print(f"\n=== (a) straight-line total load vs config weight ===")
    print(f"config total weight: {config_weight_N:.1f} N ({vp['mass_kg']:.1f} kg) -- "
          f"per the user, config mass_kg for THIS car/session IS Dubai's own weighing, "
          f"so this comparison is a direct calibration check, not a loaded-weight-uncertainty band.")
    if straight.any():
        mean_total = float(np.mean(total_fz[straight]))
        pct = (mean_total - config_weight_N) / config_weight_N * 100.0
        print(f"straight-line n={straight.sum()}, mean total Fz={mean_total:.1f} N "
              f"({pct:+.2f}% vs config weight)")
        print(f"  band +/-5% (same reporting band as the v3 validation): "
              + ("WITHIN BAND" if abs(pct) <= 5.0 else "OUTSIDE BAND -- reported, not corrected"))
    else:
        print("no straight-line samples found under the (|ax|<0.5, |ay|<0.5, moving) mask")

    print(f"\n=== (c) transfer signs/magnitudes vs ax/ay ===")
    # AXLE-RELATIVE baseline (Fz-integration Phase 1 wording fix,
    # 2026-09-03): an absolute front-vs-rear comparison ("expect front >
    # rear") only holds for a roughly front/rear-symmetric static split.
    # Dubai's own static split is strongly rear-biased (see (d) below), so
    # the correct braking check is each axle's OWN load CHANGE relative to
    # its own straight-line baseline -- braking must increase the front
    # axle's load and decrease the rear axle's, regardless of which axle
    # carries more load overall. Same fix needed in inspect_v3_wheel_load_
    # validation.py, whose wording this script originally copied verbatim.
    braking = moving & (ax < -1.0)
    if braking.any() and straight.any():
        mean_front_brake = float(np.mean(front_fz[braking]))
        mean_rear_brake = float(np.mean(rear_fz[braking]))
        mean_front_straight = float(np.mean(front_fz[straight]))
        mean_rear_straight = float(np.mean(rear_fz[straight]))
        d_front = mean_front_brake - mean_front_straight
        d_rear = mean_rear_brake - mean_rear_straight
        print(f"braking (ax<-1.0, n={braking.sum()}) vs straight/no-transfer baseline (n={straight.sum()}):")
        print(f"  front: {mean_front_straight:.1f} N -> {mean_front_brake:.1f} N ({d_front:+.1f} N)")
        print(f"  rear:  {mean_rear_straight:.1f} N -> {mean_rear_brake:.1f} N ({d_rear:+.1f} N)")
        print(f"  expect front to INCREASE and rear to DECREASE under braking transfer, "
              f"independent of which axle carries more load at rest: "
              + ("CONFIRMED" if (d_front > 0 and d_rear < 0) else "NOT CONFIRMED -- flagged, not corrected"))
    elif braking.any():
        print(f"braking (ax<-1.0, n={braking.sum()}): mean front Fz={float(np.mean(front_fz[braking])):.1f} N, "
              f"mean rear Fz={float(np.mean(rear_fz[braking])):.1f} N "
              f"(no straight-line baseline available for the axle-relative check)")

    if cornering.any():
        corr_ay_fr = np.corrcoef(ay[cornering], combined["fr"]["fz_N"][cornering])[0, 1]
        corr_ay_fl = np.corrcoef(ay[cornering], combined["fl"]["fz_N"][cornering])[0, 1]
        print(f"cornering (|ay|>3.0, n={cornering.sum()}): corr(ay, Fz_fr)={corr_ay_fr:+.3f}, "
              f"corr(ay, Fz_fl)={corr_ay_fl:+.3f} (expect fr positive/fl negative)")
        # Rear axle's ARB-relevant pair (rl/rr) -- rr is expected dead per
        # the premise-correction finding, reported for completeness anyway.
        corr_ay_rr = np.corrcoef(ay[cornering], combined["rr"]["fz_N"][cornering])[0, 1]
        corr_ay_rl = np.corrcoef(ay[cornering], combined["rl"]["fz_N"][cornering])[0, 1]
        print(f"cornering (|ay|>3.0, n={cornering.sum()}): corr(ay, Fz_rr)={corr_ay_rr:+.3f}, "
              f"corr(ay, Fz_rl)={corr_ay_rl:+.3f} (expect rr positive/rl negative)")

    travel_fl_ch = data["channels"].get("log_susp_travel_fl")
    travel_fr_ch = data["channels"].get("log_susp_travel_fr")
    if (travel_fl_ch and travel_fr_ch and travel_fl_ch["quality"] == "valid"
            and travel_fr_ch["quality"] == "valid"):
        travel_fl = np.interp(state["time"], travel_fl_ch["time"], travel_fl_ch["data"])
        travel_fr = np.interp(state["time"], travel_fr_ch["time"], travel_fr_ch["data"])
        delta = travel_fl - travel_fr
        if cornering.any():
            corr_delta_ay = np.corrcoef(delta[cornering], ay[cornering])[0, 1]
            print(f"\nARB sign-convention check (front, fl-fr vs ay): corr = {corr_delta_ay:+.3f} "
                  + ("CONFIRMED (matches v3's own convention)." if corr_delta_ay > 0
                     else "CONTRADICTED -- flagged, not changed by this read-only script."))
    else:
        print("\nARB sign-convention check (front) skipped -- fl/fr travel channels not both valid")

    print(f"\n=== (aero) total Fz regressed against v^2, Fz_total = a + b*ax + c*v^2 ===")
    mask = moving & (np.abs(ay) < 1.5)
    nfit = int(mask.sum())
    print(f"regression population: n={nfit} (moving, |ay|<1.5 m/s^2)")
    if nfit >= 20:
        X = np.column_stack([np.ones(nfit), ax[mask], v[mask] ** 2])
        y = total_fz[mask]
        coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
        a, b, c_coef = coeffs
        y_pred = X @ coeffs
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        resid = y - y_pred
        print(f"  a={a:.1f} N  b={b:.2f} N/(m/s^2)  c={c_coef:.4f} N/(m/s)^2  "
              f"{'(downforce-consistent, c>0)' if c_coef > 0 else '(lift-consistent or noise, c<=0)'}  R^2={r2:.4f}")
        print(f"  residual: mean={resid.mean():.1f} N, std={resid.std():.1f} N, "
              f"|resid| p90={np.percentile(np.abs(resid), 90):.1f} N")
        v_kmh_range = (float(np.min(v[mask])) * 3.6, float(np.max(v[mask])) * 3.6)
        print(f"  speed range in regression population: {v_kmh_range[0]:.0f}-{v_kmh_range[1]:.0f} km/h")
    else:
        print("  too few samples for a meaningful fit")

    print(f"\n=== (d) front/rear distribution (reported only, never asserted) ===")
    if moving.any():
        mean_front = float(np.mean(front_fz[moving]))
        mean_rear = float(np.mean(rear_fz[moving]))
        mean_total_mv = float(np.mean(total_fz[moving]))
        print(f"whole session, moving samples: mean front Fz={mean_front:.1f} N ({mean_front/mean_total_mv*100:.1f}%), "
              f"mean rear Fz={mean_rear:.1f} N ({mean_rear/mean_total_mv*100:.1f}%)")


if __name__ == "__main__":
    main()
