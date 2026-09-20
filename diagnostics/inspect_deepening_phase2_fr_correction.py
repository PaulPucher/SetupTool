# Deepening work package, Phase 2: FR gauge decimal-correction test, v3.
# Read-only against config/production; the correction is tested in a
# local, deep-copied channels dict only, never written to config until/
# unless the acceptance chain below passes.
#
# Phase 0's own forensics (thesis_notes.md "Frame-Stage-2 Phase 0")
# already ruled out pure rescaling: log_dms_dam_fr's raw fluctuation
# (whole-session std=1864.2N, per-window 1500-2250N) ALREADY sits at the
# same physical scale as the three healthy corners' own raw pushrod-force
# std (FL 1925.9N, RL 2082.5N, RR 2007.3N) -- a x1e-3 or x1e-6 rescale
# would shrink that fluctuation to an unphysically small value, not
# recover it. The only decoding family consistent with that finding is a
# pure ADDITIVE offset (raw + K), which is what this phase tests, plus an
# offset-then-scale variant for completeness per the work order's own
# instruction (expected to fail the same way, verified not assumed).
#
# K is picked empirically, not guessed: this session's own real, valid
# corners provide a cross-check the earlier forensics phase did not have
# (no reliable outing weighing existed yet) -- FL is FR's own axle mate,
# nearly equal static weight after Phase 1's real weighing (FL 300.3kg vs
# FR 298.9kg, 0.47% apart), same motion-ratio table (front, axle-shared).
# K is chosen so FR's own straight-line raw mean, scaled by the tiny
# static-weight ratio, matches FL's own straight-line raw mean.

import copy

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, load_car_data, prepare_vehicle_state
from modules.wheel_loads import estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS
from modules.stability_analysis import estimate_lateral_forces, estimate_vertical_loads
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle

V3_FILE = "GT3_PRC_MLA-v3.txt"

# Phase 1's own real v3 weighing (this session, stored in the outing).
V3_CORNER_WEIGHTS_KG = {"fl": 300.3, "fr": 298.9, "rl": 396.2, "rr": 386.5}


def main():
    params = load_parameters()
    setup_data = {"car": {
        "corner_weight_fl": V3_CORNER_WEIGHTS_KG["fl"], "corner_weight_fr": V3_CORNER_WEIGHTS_KG["fr"],
        "corner_weight_rl": V3_CORNER_WEIGHTS_KG["rl"], "corner_weight_rr": V3_CORNER_WEIGHTS_KG["rr"],
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    effective_params = apply_resolved_vehicle(params, resolved)
    car_data = load_car_data()

    data = parse_csv(V3_FILE)
    state = prepare_vehicle_state(data["channels"], effective_params)
    if state is None:
        print("prepare_vehicle_state returned None -- aborting")
        return
    n = len(state["time"])
    moving = state["moving_mask"]
    ax, ay = state["ax_mps2"], state["ay_mps2"]
    straight = moving & (np.abs(ax) < 0.5) & (np.abs(ay) < 0.5)
    print(f"state: {n} samples, straight-line n={straight.sum()}")

    fr_ch = data["channels"]["log_dms_dam_fr"]
    fl_ch = data["channels"]["log_dms_dam_fl"]
    t_fr, v_fr = fr_ch["time"], fr_ch["data"]
    t_fl, v_fl = fl_ch["time"], fl_ch["data"]
    fr_on_ref = np.interp(state["time"], t_fr, v_fr)
    fl_on_ref = np.interp(state["time"], t_fl, v_fl)

    fr_raw_straight_mean = fr_on_ref[straight].mean()
    fl_raw_straight_mean = fl_on_ref[straight].mean()
    weight_ratio = V3_CORNER_WEIGHTS_KG["fr"] / V3_CORNER_WEIGHTS_KG["fl"]
    fr_target_mean = fl_raw_straight_mean * weight_ratio
    K = fr_target_mean - fr_raw_straight_mean
    print(f"\nFL raw straight-line mean: {fl_raw_straight_mean:.2f}")
    print(f"FR raw straight-line mean (uncorrected): {fr_raw_straight_mean:.2f}")
    print(f"FR target mean (FL * weight_ratio {weight_ratio:.5f}): {fr_target_mean:.2f}")
    print(f"CANDIDATE OFFSET K (pure additive, empirically anchored to FL): {K:.2f}")

    print(f"\n--- decoding candidates tested ---")
    print(f"(b) offset-then-scale variants, per the work order's own instruction -- expected to fail the")
    print(f"    same way Phase 0 already found, since the raw fluctuation is ALREADY at the right scale:")
    for scale in (1e-3, 1e-6):
        v_scaled = (v_fr + K) * scale
        print(f"    (raw+K)*{scale}: std={v_scaled.std():.6g} (healthy-corner reference std~1900-2100N) -- "
              f"{'REJECTED, wrong physical scale' if v_scaled.std() < 100 else 'check'}")

    corrected_fr = v_fr + K
    print(f"\ncorrected FR (raw+K only, no scale): mean={corrected_fr.mean():.2f} std={corrected_fr.std():.2f} "
          f"min={corrected_fr.min():.2f} max={corrected_fr.max():.2f}")

    # Inject the corrected channel: same array, quality forced to "valid"
    # (channels.json's own range gate would now pass it anyway, since the
    # corrected value sits inside [-5000,20000]N -- forced explicitly here
    # so this test does not depend on re-deriving the gate's own pass/fail
    # logic a second time).
    channels_corrected = copy.deepcopy(data["channels"])
    channels_corrected["log_dms_dam_fr"] = dict(fr_ch)
    channels_corrected["log_dms_dam_fr"]["data"] = corrected_fr
    channels_corrected["log_dms_dam_fr"]["quality"] = "valid"
    in_band = (corrected_fr >= -5000) & (corrected_fr <= 20000)
    print(f"corrected FR in-band fraction (channels.json range [-5000,20000]N): {in_band.mean()*100:.2f}%")

    damper_result = estimate_wheel_loads_from_dampers(state, channels_corrected, effective_params, car_data)
    print(f"\nFR corner: valid={damper_result['fr']['valid'].mean()*100:.1f}% dead_channel={damper_result['fr']['dead_channel']}")
    for c in CORNERS:
        print(f"  {c}: valid={damper_result[c]['valid'].mean()*100:.1f}% dead_channel={damper_result[c]['dead_channel']}")

    forces = estimate_lateral_forces(state, effective_params)
    fz_static = estimate_vertical_loads(state, forces, effective_params)
    static_fallback_fz = {c: fz_static[f"fz_{c}_N"] for c in CORNERS}
    combined = combine_with_static_fallback(damper_result, static_fallback_fz)

    total_fz = sum(combined[c]["fz_N"] for c in CORNERS)
    front_fz = combined["fl"]["fz_N"] + combined["fr"]["fz_N"]

    vp = effective_params["vehicle"]
    config_weight_N = vp["mass_kg"] * 9.81
    print(f"\n=== ACCEPTANCE CHECK (a): straight-line total load vs config/session weight ===")
    print(f"session weight (Phase 1 weighing): {config_weight_N:.1f} N ({vp['mass_kg']:.1f} kg)")
    if straight.any():
        mean_total = float(np.mean(total_fz[straight]))
        pct = (mean_total - config_weight_N) / config_weight_N * 100.0
        print(f"straight-line n={straight.sum()}, mean total Fz={mean_total:.1f} N ({pct:+.2f}% vs session weight)")
        print(f"  (reference band from the damper package's own healthy validation: +/-5%% 'in band', "
              f"else must be aero-explicable -- v3's own prior all-static-fallback run found +11.64%%, "
              f"attributed to real aero downforce, corroborated by an independent v^2 regression)")

    print(f"\n=== ACCEPTANCE CHECK (b): transfer correlations ===")
    cornering = moving & (np.abs(ay) > 3.0)
    corr_fr_ay = np.corrcoef(combined["fr"]["fz_N"][cornering], ay[cornering])[0, 1]
    corr_fl_ay = np.corrcoef(combined["fl"]["fz_N"][cornering], ay[cornering])[0, 1]
    print(f"corr(ay, Fz_fr) = {corr_fr_ay:+.4f}  (target ~0.9+, reference healthy corners this project has "
          f"recorded: +0.966 to +0.979)")
    print(f"corr(ay, Fz_fl) = {corr_fl_ay:+.4f}  (reference: -0.893 to -0.895, opposite sign, same axle)")

    print(f"\n=== ACCEPTANCE CHECK (c): front L/R consistent with Phase 1 outing weights ===")
    fr_static_mean = float(np.mean(combined["fr"]["fz_N"][straight]))
    fl_static_mean = float(np.mean(combined["fl"]["fz_N"][straight]))
    fr_expected_N = V3_CORNER_WEIGHTS_KG["fr"] * 9.81
    fl_expected_N = V3_CORNER_WEIGHTS_KG["fl"] * 9.81
    print(f"FR straight-line mean Fz: {fr_static_mean:.1f} N  (session-weighed static: {fr_expected_N:.1f} N, "
          f"delta {fr_static_mean-fr_expected_N:+.1f} N, {(fr_static_mean-fr_expected_N)/fr_expected_N*100:+.2f}%)")
    print(f"FL straight-line mean Fz: {fl_static_mean:.1f} N  (session-weighed static: {fl_expected_N:.1f} N, "
          f"delta {fl_static_mean-fl_expected_N:+.1f} N, {(fl_static_mean-fl_expected_N)/fl_expected_N*100:+.2f}%)")
    print(f"FL/FR ratio, corrected: {fl_static_mean/fr_static_mean:.4f}  vs session-weighed ratio: "
          f"{fl_expected_N/fr_expected_N:.4f}")

    print(f"\n=== ACCEPTANCE CHECK (d): fuel drift sanity (front total across laps) ===")
    lap_numbers = sorted({lap["lap_number"] for lap in data.get("laps", []) if lap.get("is_valid_for_analysis")})
    for lap in data.get("laps", []):
        if not lap.get("is_valid_for_analysis"):
            continue
        lap_mask = straight & (state["time"] >= lap["start_time"]) & (state["time"] <= lap["end_time"])
        if lap_mask.sum() < 5:
            continue
        print(f"  lap {lap['lap_number']}: n={lap_mask.sum()} mean total Fz={np.mean(total_fz[lap_mask]):.1f} N")


if __name__ == "__main__":
    main()
