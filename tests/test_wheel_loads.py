# Damper package Phase 1 -- unit tests on modules/wheel_loads.py, synthetic
# fixtures only (same discipline as tests/test_longitudinal_stiffness.py's
# own hand-computed-ramp tests and tests/test_csv_parser_formats.py's
# "synthetic fixtures only" precedent -- pure-function unit tests are not
# the "no synthetic test data" analysis/validation rule these fixtures
# never claim to be real telemetry).

import numpy as np

from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
    reconstruct_missing_corner, combine_with_reconstruction_and_fallback,
    estimate_session_corrected_axle_totals,
)


def _car_data():
    # Small hand-crafted tables, NOT the real digitised car_data.json --
    # shape-identical (same keys, same "points"/"positions" structure) so
    # the module's own lookup code is exercised exactly as it runs in
    # production, with numbers chosen to make expected values easy to
    # hand-verify.
    return {
        "motion_ratio_vs_wheel_travel": {
            "front": {"points": [[-50, 0.60], [0, 0.65], [50, 0.70]]},
            "rear": {"points": [[-50, 0.75], [0, 0.78], [50, 0.76]]},
        },
        "arb": {
            "front": {"positions": {str(i): float(i) * 2.0 for i in range(1, 8)}, "ratio_to_wheel": 1.0},
            "rear": {"positions": {str(i): float(i) * 3.0 for i in range(1, 8)}, "ratio_to_wheel": 1.0},
        },
    }


def _params():
    return {
        "vehicle": {
            "track_width_front_m": 1.6,
            "track_width_rear_m": 1.6,
            "wheelbase_m": 2.5,
            "cog_to_front_axle_m": 1.3,
            "cog_to_rear_axle_m": 1.2,
            "cog_height_m": 0.3,
            "corner_weights": {"FL_kg": 300.0, "FR_kg": 300.0, "RL_kg": 400.0, "RR_kg": 400.0},
        },
        "wheel_loads": {
            "pushrod_offset_fl_N": 0.0, "pushrod_offset_fr_N": 0.0,
            "pushrod_offset_rl_N": 0.0, "pushrod_offset_rr_N": 0.0,
            "unsprung_mass_front_kg": 40.0, "unsprung_mass_rear_kg": 45.0,
            "tyre_dynamic_radius_front_m": 0.34, "tyre_dynamic_radius_rear_m": 0.35,
            "roll_centre_front_m": 0.02, "roll_centre_rear_m": 0.05,
            "arb_position_fallback": 4,
            "aero_front_fraction": 0.40,
        },
        "stability_estimation": {"moving_speed_min_mps": 5.0},
    }


def _channel(time, data, quality="valid"):
    return {"time": time, "data": data, "quality": quality}


def _all_corner_channels(n, time, force_by_corner, travel_by_corner, quality_by_corner=None):
    quality_by_corner = quality_by_corner or {c: "valid" for c in CORNERS}
    channels = {}
    for c in CORNERS:
        channels[f"log_dms_dam_{c}"] = _channel(time, force_by_corner[c], quality_by_corner.get(c, "valid"))
        channels[f"log_susp_travel_{c}"] = _channel(time, travel_by_corner[c], quality_by_corner.get(c, "valid"))
    return channels


def _state(n, ay=0.0):
    return {"time": np.arange(n, dtype=float), "ay_mps2": np.full(n, ay, dtype=float)}


# --- known pushrod force + MR -> known wheel load ---------------------------

def test_known_pushrod_force_and_mr_gives_known_sprung_load():
    n = 4
    state = _state(n, ay=0.0)
    time = state["time"]
    # Travel held exactly at a digitised table point (0mm) on every corner
    # so motion_ratio is exact, not interpolated: front 0.65, rear 0.78.
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {"fl": np.full(n, 1000.0), "fr": np.full(n, 1000.0),
             "rl": np.full(n, 1000.0), "rr": np.full(n, 1000.0)}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)
    # ARB delta is zero (symmetric travel) so fz_N reduces to sprung_N plus
    # the (zero, symmetric) unsprung/geometric transfer terms at ay=0.
    assert np.allclose(result["fl"]["sprung_N"], 1000.0 * 0.65)
    assert np.allclose(result["rl"]["sprung_N"], 1000.0 * 0.78)
    assert np.allclose(result["fl"]["fz_N"], 1000.0 * 0.65, atol=1e-9)
    assert np.allclose(result["rr"]["fz_N"], 1000.0 * 0.78, atol=1e-9)


def test_pushrod_offset_is_subtracted_before_motion_ratio():
    n = 2
    state = _state(n)
    time = state["time"]
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {c: np.full(n, 1200.0) for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()
    params["wheel_loads"]["pushrod_offset_fl_N"] = 200.0

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)
    assert np.allclose(result["fl"]["sprung_N"], (1200.0 - 200.0) * 0.65)
    assert np.allclose(result["fr"]["sprung_N"], 1200.0 * 0.65)


# --- ARB delta sign ----------------------------------------------------------

def test_arb_force_sign_and_magnitude_from_travel_delta():
    n = 2
    state = _state(n)
    time = state["time"]
    # Front: fl travel higher than fr by 10mm, both at MR=0.65 (table
    # point 0mm plus a small offset kept inside the table's flat-enough
    # region -- exact MR value at 0mm used for a clean hand check).
    travel = {"fl": np.full(n, 5.0), "fr": np.full(n, -5.0), "rl": np.zeros(n), "rr": np.zeros(n)}
    force = {c: np.zeros(n) for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()
    # position 4 -> front rate = 4*2.0 = 8.0 N/mm (ratio_to_wheel=1.0)
    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=4)

    delta_mm = 10.0
    rate = 8.0
    avg_mr = 0.65  # both corners land close enough to 0.65 given the small +/-5mm travel
    expected_half = 0.5 * delta_mm * rate / avg_mr
    assert np.allclose(result["fl"]["arb_N"], -expected_half, rtol=1e-2)
    assert np.allclose(result["fr"]["arb_N"], expected_half, rtol=1e-2)
    # Bar reaction is equal and opposite across the axle by construction.
    assert np.allclose(result["fl"]["arb_N"] + result["fr"]["arb_N"], 0.0, atol=1e-6)


# --- missing-channel fallback to static split --------------------------------

def test_missing_damper_channel_falls_back_per_corner():
    n = 3
    state = _state(n)
    time = state["time"]
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {c: np.full(n, 900.0) for c in CORNERS}
    quality = {"fl": "valid", "fr": "failed", "rl": "valid", "rr": "valid"}
    channels = _all_corner_channels(n, time, force, travel, quality_by_corner=quality)
    car_data = _car_data()
    params = _params()

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)
    assert result["fl"]["valid"].all()
    assert not result["fr"]["valid"].any()
    assert np.isnan(result["fr"]["fz_N"]).all()

    static_fallback_fz = {"fl": np.full(n, 1.0), "fr": np.full(n, 2.0),
                           "rl": np.full(n, 3.0), "rr": np.full(n, 4.0)}
    combined = combine_with_static_fallback(result, static_fallback_fz)
    assert np.allclose(combined["fr"]["fz_N"], 2.0)
    assert (combined["fr"]["source"] == "static_fallback").all()
    assert (combined["fl"]["source"] == "damper").all()
    assert not np.allclose(combined["fl"]["fz_N"], 1.0)


def test_missing_travel_channel_also_triggers_fallback():
    n = 2
    state = _state(n)
    time = state["time"]
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {c: np.full(n, 900.0) for c in CORNERS}
    quality = {"fl": "valid", "fr": "valid", "rl": "valid", "rr": "missing"}
    channels = _all_corner_channels(n, time, force, travel, quality_by_corner=quality)
    channels["log_susp_travel_rr"]["time"] = None
    car_data = _car_data()
    params = _params()

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)
    assert not result["rr"]["valid"].any()


# --- unsprung / geometric transfer sign matches the shared convention -------

def test_lateral_transfer_sign_matches_left_negative_right_positive_convention():
    n = 2
    state = _state(n, ay=5.0)
    time = state["time"]
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {c: np.zeros(n) for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)
    # Positive ay -> right-side wheels (fr/rr) gain load, left-side (fl/rl)
    # lose it, for BOTH the unsprung and geometric transfer terms --
    # matches modules.stability_analysis.estimate_vertical_loads's own
    # fz_fl_N = fz_f/2 - lateral_transfer/2, fz_fr_N = fz_f/2 + lateral_
    # transfer/2 sign convention exactly.
    assert result["fl"]["unsprung_transfer_N"][0] < 0
    assert result["fr"]["unsprung_transfer_N"][0] > 0
    assert result["fl"]["geometric_transfer_N"][0] < 0
    assert result["fr"]["geometric_transfer_N"][0] > 0
    assert np.isclose(result["fl"]["unsprung_transfer_N"][0], -result["fr"]["unsprung_transfer_N"][0])
    assert np.isclose(result["rl"]["geometric_transfer_N"][0], -result["rr"]["geometric_transfer_N"][0])


# --- FR reconstruction (morning follow-up, Item 2) ---------------------------

def _synthetic_damper_result(n, fz_by_corner, valid_by_corner):
    return {
        c: {"fz_N": np.full(n, fz_by_corner[c], dtype=float), "valid": np.full(n, valid_by_corner[c])}
        for c in CORNERS
    }


def test_reconstruct_missing_corner_recovers_asymmetric_roll_split():
    # Axle total is fixed (6000N front), but the true split is asymmetric
    # (2500/3500, NOT 50/50) -- the reconstruction must recover the exact
    # missing value from the axle total minus the real measured mate, not
    # from an assumed symmetric split.
    n = 3
    damper_result = _synthetic_damper_result(
        n,
        {"fl": 2500.0, "fr": 0.0, "rl": 4000.0, "rr": 4200.0},
        {"fl": True, "fr": False, "rl": True, "rr": True},
    )
    fz_axle_totals = {"fz_f_N": np.full(n, 6000.0), "fz_r_N": np.full(n, 8200.0)}

    reconstructed = reconstruct_missing_corner(damper_result, fz_axle_totals)
    assert np.allclose(reconstructed["fr"]["fz_N"], 3500.0)
    assert reconstructed["fr"]["reconstructable"].all()
    # Valid corners are never marked reconstructable, even though they
    # technically satisfy "axle total minus mate" too.
    assert not reconstructed["fl"]["reconstructable"].any()
    assert not reconstructed["rl"]["reconstructable"].any()
    assert not reconstructed["rr"]["reconstructable"].any()


def test_reconstruct_missing_corner_both_axle_corners_invalid_not_reconstructable():
    n = 2
    damper_result = _synthetic_damper_result(
        n,
        {"fl": 0.0, "fr": 0.0, "rl": 4000.0, "rr": 4200.0},
        {"fl": False, "fr": False, "rl": True, "rr": True},
    )
    fz_axle_totals = {"fz_f_N": np.full(n, 6000.0), "fz_r_N": np.full(n, 8200.0)}

    reconstructed = reconstruct_missing_corner(damper_result, fz_axle_totals)
    assert not reconstructed["fl"]["reconstructable"].any()
    assert not reconstructed["fr"]["reconstructable"].any()
    assert np.isnan(reconstructed["fl"]["fz_N"]).all()
    assert np.isnan(reconstructed["fr"]["fz_N"]).all()


def test_estimate_session_corrected_axle_totals_recovers_noiseless_fit():
    # ax=ay=0 everywhere (every sample satisfies both the tight and wide
    # straight-line masks) -- FL/RL/RR constructed so total_fz_for_fit
    # (FL used twice, as its own FR proxy, plus RL plus RR) equals
    # A + C*v^2 EXACTLY at every sample, letting the regression recover
    # C exactly (noiseless) and letting the returned fz_f_N+fz_r_N sum be
    # checked directly against the same closed form (long-transfer term
    # is exactly zero here since ax=0 throughout).
    n = 50
    v = np.linspace(20.0, 80.0, n)
    A_true = 12000.0
    C_true = 0.8
    total_true = A_true + C_true * v ** 2
    quarter = total_true / 4.0  # FL=RL=RR=quarter, fr_proxy=FL=quarter -> sum = 4*quarter = total_true
    state = {"time": np.arange(n, dtype=float), "v_mps": v,
             "ax_mps2": np.zeros(n), "ay_mps2": np.zeros(n)}
    damper_result = {
        "fl": {"fz_N": quarter.copy()}, "rl": {"fz_N": quarter.copy()}, "rr": {"fz_N": quarter.copy()},
        "fr": {"fz_N": np.full(n, np.nan)},  # never read (fr_proxy uses fl instead)
    }
    params = _params()

    result = estimate_session_corrected_axle_totals(state, damper_result, params)
    assert np.isclose(result["c_session_N_per_mps2"], C_true, rtol=1e-6)
    assert np.isclose(result["mass_kg_session"], np.mean(total_true) / 9.81, rtol=1e-6)
    assert result["aero_front_fraction"] == 0.40
    # FL=RL=RR symmetric in this fixture -> both session-measured split
    # fractions must land exactly on 0.5 (front==rear total, RL==RR).
    assert np.isclose(result["front_mass_fraction"], 0.5, rtol=1e-9)
    assert np.isclose(result["rear_left_fraction"], 0.5, rtol=1e-9)

    total_model = result["fz_f_N"] + result["fz_r_N"]
    expected_total = result["mass_kg_session"] * 9.81 + C_true * v ** 2
    assert np.allclose(total_model, expected_total, rtol=1e-6)
    # Front/rear split of the aero term matches aero_front_fraction exactly
    # (mass term uses the session-measured front_mass_fraction, 0.5 here).
    aero_f = result["fz_f_N"] - result["mass_kg_session"] * 9.81 * result["front_mass_fraction"]
    assert np.allclose(aero_f, 0.40 * C_true * v ** 2, rtol=1e-6)


def test_estimate_session_corrected_axle_totals_recovers_asymmetric_split_fractions():
    # Deliberately asymmetric front/rear AND left/right rear totals --
    # front_mass_fraction and rear_left_fraction must recover the exact
    # known ratios, not silently default to 0.5 or to the static config
    # geometric fraction.
    n = 20
    v = np.full(n, 40.0)
    state = {"time": np.arange(n, dtype=float), "v_mps": v,
             "ax_mps2": np.zeros(n), "ay_mps2": np.zeros(n)}
    fl = np.full(n, 1000.0)   # front total = 2*fl = 2000 (fr proxied by fl)
    rl = np.full(n, 1800.0)
    rr = np.full(n, 1200.0)   # rear total = 3000 -> front fraction = 2000/5000 = 0.4
    damper_result = {
        "fl": {"fz_N": fl}, "rl": {"fz_N": rl}, "rr": {"fz_N": rr},
        "fr": {"fz_N": np.full(n, np.nan)},
    }
    params = _params()

    result = estimate_session_corrected_axle_totals(state, damper_result, params)
    assert np.isclose(result["front_mass_fraction"], 0.4, rtol=1e-9)
    assert np.isclose(result["rear_left_fraction"], 1800.0 / 3000.0, rtol=1e-9)


def test_combine_with_reconstruction_and_fallback_three_tier_source_labels():
    n = 2
    damper_result = _synthetic_damper_result(
        n,
        {"fl": 2500.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},
        {"fl": True, "fr": False, "rl": False, "rr": False},
    )
    fz_axle_totals = {"fz_f_N": np.full(n, 6000.0), "fz_r_N": np.full(n, 8200.0)}
    static_fallback_fz = {"fl": np.full(n, 1.0), "fr": np.full(n, 2.0),
                           "rl": np.full(n, 3.0), "rr": np.full(n, 4.0)}

    combined = combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz)
    assert (combined["fl"]["source"] == "damper").all()
    assert np.allclose(combined["fl"]["fz_N"], 2500.0)
    assert (combined["fr"]["source"] == "reconstructed").all()
    assert np.allclose(combined["fr"]["fz_N"], 3500.0)
    # Both rear corners invalid at once -> no reconstruction possible ->
    # falls all the way through to the plain static-split fallback.
    assert (combined["rl"]["source"] == "static_fallback").all()
    assert np.allclose(combined["rl"]["fz_N"], 3.0)
    assert (combined["rr"]["source"] == "static_fallback").all()
    assert np.allclose(combined["rr"]["fz_N"], 4.0)
