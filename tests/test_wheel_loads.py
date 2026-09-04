# Damper package Phase 1 -- unit tests on modules/wheel_loads.py, synthetic
# fixtures only (same discipline as tests/test_longitudinal_stiffness.py's
# own hand-computed-ramp tests and tests/test_csv_parser_formats.py's
# "synthetic fixtures only" precedent -- pure-function unit tests are not
# the "no synthetic test data" analysis/validation rule these fixtures
# never claim to be real telemetry).

import numpy as np
import pytest

from modules.wheel_loads import (
    estimate_wheel_loads_from_dampers, combine_with_static_fallback, CORNERS,
    reconstruct_missing_corner, combine_with_reconstruction_and_fallback,
    estimate_session_corrected_axle_totals, _normalize_travel_to_mm, _channel_is_dead,
    _axle_total_with_proxy,
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
            # 0.0 here (never below the real threshold) keeps every formula-
            # correctness test below unaffected by the dead-channel guard --
            # most of them deliberately use constant/zero synthetic data
            # (std=0) to make expected values hand-verifiable, which is
            # exactly what the guard is designed to catch on real telemetry.
            # test_dead_channel_guard_* below overrides this to a realistic
            # value to exercise the guard itself.
            "dead_channel_std_max_travel_mm": 0.0,
            "dead_channel_std_max_force_N": 0.0,
        },
        "stability_estimation": {"moving_speed_min_mps": 5.0},
    }


def _channel(time, data, quality="valid", unit_raw="N"):
    return {"time": time, "data": data, "quality": quality, "unit_raw": unit_raw}


def _all_corner_channels(n, time, force_by_corner, travel_by_corner, quality_by_corner=None,
                          travel_unit_raw="mm"):
    quality_by_corner = quality_by_corner or {c: "valid" for c in CORNERS}
    channels = {}
    for c in CORNERS:
        channels[f"log_dms_dam_{c}"] = _channel(time, force_by_corner[c], quality_by_corner.get(c, "valid"),
                                                  unit_raw="N")
        channels[f"log_susp_travel_{c}"] = _channel(time, travel_by_corner[c], quality_by_corner.get(c, "valid"),
                                                      unit_raw=travel_unit_raw)
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
    valid_true = np.full(n, True)
    damper_result = {
        "fl": {"fz_N": quarter.copy(), "valid": valid_true},
        "rl": {"fz_N": quarter.copy(), "valid": valid_true},
        "rr": {"fz_N": quarter.copy(), "valid": valid_true},
        "fr": {"fz_N": np.full(n, np.nan), "valid": np.full(n, False)},  # proxied from fl, ratio 1.0 (equal weights)
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
    valid_true = np.full(n, True)
    damper_result = {
        "fl": {"fz_N": fl, "valid": valid_true}, "rl": {"fz_N": rl, "valid": valid_true},
        "rr": {"fz_N": rr, "valid": valid_true},
        "fr": {"fz_N": np.full(n, np.nan), "valid": np.full(n, False)},
    }
    params = _params()

    result = estimate_session_corrected_axle_totals(state, damper_result, params)
    assert np.isclose(result["front_mass_fraction"], 0.4, rtol=1e-9)
    assert np.isclose(result["rear_left_fraction"], 1800.0 / 3000.0, rtol=1e-9)


# --- travel unit normalisation (Fz-integration Phase 1) ---------------------

def test_normalize_travel_to_mm_identity_for_mm():
    data = np.array([1.0, -2.5, 30.0])
    assert np.array_equal(_normalize_travel_to_mm(data, "mm"), data)


def test_normalize_travel_to_mm_scales_metres():
    data = np.array([0.001, -0.0025, 0.030])
    assert np.allclose(_normalize_travel_to_mm(data, "m"), [1.0, -2.5, 30.0])


def test_normalize_travel_to_mm_raises_on_unknown_unit():
    with pytest.raises(ValueError):
        _normalize_travel_to_mm(np.zeros(3), "cm")


def test_wheel_loads_from_dampers_gives_same_result_for_m_and_mm_inputs():
    # The same physical travel (5mm at every sample) logged two different
    # ways must produce IDENTICAL sprung_N/arb_N -- proves the conversion
    # is wired into estimate_wheel_loads_from_dampers itself, not just
    # unit-tested in isolation.
    n = 2
    state = _state(n)
    time = state["time"]
    travel_mm_native = {"fl": np.full(n, 5.0), "fr": np.full(n, -5.0), "rl": np.zeros(n), "rr": np.zeros(n)}
    travel_m_native = {c: v / 1000.0 for c, v in travel_mm_native.items()}
    force = {c: np.full(n, 1000.0) for c in CORNERS}
    car_data = _car_data()
    params = _params()

    channels_mm = _all_corner_channels(n, time, force, travel_mm_native, travel_unit_raw="mm")
    channels_m = _all_corner_channels(n, time, force, travel_m_native, travel_unit_raw="m")

    result_mm = estimate_wheel_loads_from_dampers(state, channels_mm, params, car_data, arb_position=4)
    result_m = estimate_wheel_loads_from_dampers(state, channels_m, params, car_data, arb_position=4)
    for c in CORNERS:
        assert np.allclose(result_mm[c]["fz_N"], result_m[c]["fz_N"])
        assert np.allclose(result_mm[c]["arb_N"], result_m[c]["arb_N"], equal_nan=True)


def test_wheel_loads_from_dampers_raises_on_unrecognised_force_unit():
    n = 2
    state = _state(n)
    time = state["time"]
    travel = {c: np.zeros(n) for c in CORNERS}
    force = {c: np.full(n, 1000.0) for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    channels["log_dms_dam_fl"]["unit_raw"] = "bar"  # not N -- must refuse, not silently misread
    car_data = _car_data()
    params = _params()
    with pytest.raises(ValueError):
        estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=0)


# --- dead-channel (frozen, plausible-value) guard ----------------------------

def test_channel_is_dead_detects_near_zero_variance():
    assert _channel_is_dead(np.full(50, 12.3), std_max=1.0) is True
    assert _channel_is_dead(np.linspace(0.0, 40.0, 50), std_max=1.0) is False


def test_dead_channel_guard_flags_frozen_travel_and_demotes_whole_corner():
    # RR's travel channel is frozen (constant, a plausible-looking value,
    # same shape as the real Sample_Dubai.txt log_susp_travel_rr finding)
    # while its own force channel is real (varying) -- the corner must
    # still be demoted to invalid, since motion ratio genuinely depends on
    # travel: a dead travel channel corrupts the sprung-force term too,
    # not only ARB.
    n = 20
    state = _state(n)
    time = state["time"]
    rng_force = np.linspace(900.0, 1100.0, n)  # real variation
    travel = {"fl": np.linspace(-5.0, 5.0, n), "fr": np.linspace(5.0, -5.0, n),
              "rl": np.linspace(-3.0, 3.0, n), "rr": np.full(n, 2.0)}  # rr frozen
    force = {c: rng_force.copy() for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()
    params["wheel_loads"]["dead_channel_std_max_travel_mm"] = 1.0
    params["wheel_loads"]["dead_channel_std_max_force_N"] = 50.0

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=4)
    assert result["rr"]["dead_channel"] is True
    assert not result["rr"]["valid"].any()
    assert np.isnan(result["rr"]["fz_N"]).all()
    # Untouched corners keep their own quality-based validity.
    assert result["fl"]["dead_channel"] is False
    assert result["fl"]["valid"].all()


def test_dead_channel_on_one_corner_invalidates_only_that_axles_arb_term():
    # Same fixture as above: rr frozen, rl/fl/fr real. Front axle's ARB
    # (fl<->fr, both real) must stay valid; rear axle's ARB (rl<->rr, one
    # dead) must not -- but rl's OWN fz_N (sprung + unsprung + geometric,
    # ARB just contributes 0) must still be a real, finite number, not NaN
    # -- the explicit-degrade requirement: a missing ARB term must not
    # invalidate the whole corner it belongs to.
    n = 20
    state = _state(n, ay=3.0)
    time = state["time"]
    rng_force = np.linspace(900.0, 1100.0, n)
    travel = {"fl": np.linspace(-5.0, 5.0, n), "fr": np.linspace(5.0, -5.0, n),
              "rl": np.linspace(-3.0, 3.0, n), "rr": np.full(n, 2.0)}
    force = {c: rng_force.copy() for c in CORNERS}
    channels = _all_corner_channels(n, time, force, travel)
    car_data = _car_data()
    params = _params()
    params["wheel_loads"]["dead_channel_std_max_travel_mm"] = 1.0
    params["wheel_loads"]["dead_channel_std_max_force_N"] = 50.0

    result = estimate_wheel_loads_from_dampers(state, channels, params, car_data, arb_position=4)
    assert result["fl"]["arb_valid"].all()
    assert result["fr"]["arb_valid"].all()
    assert not result["rl"]["arb_valid"].any()
    assert not result["rr"]["arb_valid"].any()
    # rl stays individually valid and produces a real Fz despite the
    # missing ARB term on its own axle.
    assert result["rl"]["valid"].all()
    assert np.all(np.isfinite(result["rl"]["fz_N"]))
    assert np.allclose(result["rl"]["arb_N"], 0.0, equal_nan=True) or np.isnan(result["rl"]["arb_N"]).all()


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


# --- generalised per-axle mate-proxy (Fz-integration Phase 1 bug fix) -------
# Original bug: estimate_session_corrected_axle_totals hardcoded FR as the
# only proxyable corner (v3's own failure pattern) and summed rl+rr assuming
# both real -- silently NaN on Dubai, whose dead corner is RR instead. Fixed
# by generalising to a per-axle real-or-ratio-proxied rule (see
# _axle_total_with_proxy's own docstring for the ratio-vs-equality
# reasoning). Tests below cover: the v3 pattern reproduces byte-identically
# (ratio 1.0, equal config weights), the Dubai pattern now resolves to a
# finite total (ratio != 1.0, asymmetric config weights), and the genuinely
# unrecoverable both-corners-dead case degrades explicitly, not silently.

def test_axle_total_with_proxy_one_corner_dead_equal_weights_matches_old_fr_proxy_behaviour():
    # v3 pattern: front axle, FR dead, FL real, FL_kg==FR_kg (ratio 1.0) --
    # must reproduce the original hardcoded "front_total = 2*FL" formula.
    n = 4
    fl_fz = np.array([1000.0, 1100.0, 900.0, 1050.0])
    damper_result = _synthetic_damper_result(
        n, {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},  # placeholder, overwritten below
        {"fl": True, "fr": False, "rl": True, "rr": True},
    )
    damper_result["fl"]["fz_N"] = fl_fz
    weights = {"fl": 290.0, "fr": 290.0, "rl": 395.0, "rr": 381.0}

    total, degraded, reason = _axle_total_with_proxy(damper_result, weights, "fl", "fr")
    assert np.allclose(total, 2.0 * fl_fz)
    assert degraded is False
    assert reason is None


def test_axle_total_with_proxy_dead_corner_uses_static_ratio_not_equality():
    # Dubai pattern: rear axle, RR dead, RL real, RL_kg != RR_kg (config's
    # real asymmetric values) -- proxy must scale by RR_kg/RL_kg, not just
    # copy RL's value, and the total must be FINITE (the original bug's
    # exact failure mode: this used to be rl_fz + nan = nan, silently).
    n = 3
    rl_fz = np.array([4000.0, 4200.0, 3900.0])
    damper_result = _synthetic_damper_result(
        n, {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},
        {"fl": True, "fr": True, "rl": True, "rr": False},
    )
    damper_result["rl"]["fz_N"] = rl_fz
    weights = {"fl": 290.0, "fr": 290.0, "rl": 395.0, "rr": 381.0}

    total, degraded, reason = _axle_total_with_proxy(damper_result, weights, "rl", "rr")
    expected_rr_proxy = rl_fz * (381.0 / 395.0)
    assert np.allclose(total, rl_fz + expected_rr_proxy)
    assert np.all(np.isfinite(total))
    assert not np.allclose(total, 2.0 * rl_fz)  # would be wrong -- that's the equality mistake, not the ratio
    assert degraded is False
    assert reason is None


def test_axle_total_with_proxy_both_corners_dead_degrades_explicitly():
    n = 5
    damper_result = _synthetic_damper_result(
        n, {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},
        {"fl": True, "fr": True, "rl": False, "rr": False},
    )
    weights = {"fl": 290.0, "fr": 290.0, "rl": 395.0, "rr": 381.0}

    total, degraded, reason = _axle_total_with_proxy(damper_result, weights, "rl", "rr")
    assert np.isnan(total).all()
    assert degraded is True
    assert "rl/rr" in reason
    assert "100.0%" in reason


def test_estimate_session_corrected_axle_totals_rear_dead_dubai_pattern_gives_finite_total():
    # Integration-level regression test for the bug itself: FL/FR/RL real
    # (Dubai's actual front-axle-both-valid, rear-left-valid pattern), RR
    # dead all session -- fz_r_N must be finite everywhere, not NaN, and
    # rear_correction_degraded must be False (exactly one corner missing,
    # not both).
    n = 30
    v = np.full(n, 40.0)
    state = {"time": np.arange(n, dtype=float), "v_mps": v,
             "ax_mps2": np.zeros(n), "ay_mps2": np.zeros(n)}
    fl = np.full(n, 1500.0)
    fr = np.full(n, 1500.0)
    rl = np.full(n, 2000.0)
    damper_result = _synthetic_damper_result(
        n, {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},
        {"fl": True, "fr": True, "rl": True, "rr": False},
    )
    damper_result["fl"]["fz_N"] = fl
    damper_result["fr"]["fz_N"] = fr
    damper_result["rl"]["fz_N"] = rl
    params = _params()
    params["vehicle"]["corner_weights"] = {"FL_kg": 290.0, "FR_kg": 290.0, "RL_kg": 395.0, "RR_kg": 381.0}

    result = estimate_session_corrected_axle_totals(state, damper_result, params)
    assert np.all(np.isfinite(result["fz_r_N"]))
    assert result["rear_correction_degraded"] is False
    assert result["rear_correction_degraded_reason"] is None
    assert result["front_correction_degraded"] is False


def test_estimate_session_corrected_axle_totals_both_rear_corners_dead_flags_degraded():
    n = 10
    v = np.full(n, 40.0)
    state = {"time": np.arange(n, dtype=float), "v_mps": v,
             "ax_mps2": np.zeros(n), "ay_mps2": np.zeros(n)}
    fl = np.full(n, 1500.0)
    fr = np.full(n, 1500.0)
    # rl/rr fz_N left as NaN -- matches real production semantics (an
    # invalid corner's fz_N is NaN, per estimate_wheel_loads_from_dampers's
    # own "not ok" branch), not a placeholder 0.0 (which would make
    # rear_left_fraction's own 0/0 raise ZeroDivisionError below, unrelated
    # to this test's own subject).
    damper_result = _synthetic_damper_result(
        n, {"fl": 0.0, "fr": 0.0, "rl": np.nan, "rr": np.nan},
        {"fl": True, "fr": True, "rl": False, "rr": False},
    )
    damper_result["fl"]["fz_N"] = fl
    damper_result["fr"]["fz_N"] = fr
    params = _params()

    result = estimate_session_corrected_axle_totals(state, damper_result, params)
    assert np.isnan(result["fz_r_N"]).all()
    assert result["rear_correction_degraded"] is True
    assert "rl/rr" in result["rear_correction_degraded_reason"]
