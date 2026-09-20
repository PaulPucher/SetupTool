# Deepening work package, Phase 1: targeted tests for modules/
# accuracy_resolution.py's per-outing corner-weight resolution
# (_resolve_corner_weights/_resolve_mass/_resolve_cog_position). This
# mechanism predates this package (commit 82bc49c, "accuracy level
# selector") and is already wired into production (ui/views/outing_form.
# py calls resolve_accuracy with the outing's real setup_data) -- but had
# NO dedicated test file: every existing call site in tests/ (conftest.py,
# test_auto_fit_wiring.py, test_golden_auto_modes.py) passes setup_data=
# None deliberately, for golden-file reproducibility, so the Level-2
# session-measurement branch itself was never exercised by the suite.
# Pure-function tests -- no real pipeline run needed, resolve_accuracy/
# apply_resolved_vehicle take plain dicts.

import pytest

from modules.stability_analysis import load_parameters
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle


@pytest.fixture
def params():
    return load_parameters()


def _config_corner_weights(params):
    cw = params["vehicle"]["corner_weights"]
    return {k: cw[k] for k in ("FL_kg", "FR_kg", "RL_kg", "RR_kg")}


# --- Fallback both ways ----------------------------------------------

def test_no_setup_data_falls_back_to_level_1(params):
    resolved = resolve_accuracy(params, setup_data=None, cap=None)
    assert resolved["levels"]["corner_weights"] == 1
    assert resolved["levels"]["mass"] == 1
    assert resolved["levels"]["cog_position"] == 1
    cw = _config_corner_weights(params)
    assert resolved["values"]["corner_weights"] == {
        "FL_kg": cw["FL_kg"], "FR_kg": cw["FR_kg"], "RL_kg": cw["RL_kg"], "RR_kg": cw["RR_kg"],
    }
    assert resolved["values"]["mass_kg"] == params["vehicle"]["mass_kg"]
    assert resolved["clipped"] is False


def test_empty_car_dict_falls_back_to_level_1(params):
    resolved = resolve_accuracy(params, setup_data={"car": {}}, cap=None)
    assert resolved["levels"]["corner_weights"] == 1
    assert resolved["levels"]["mass"] == 1


def test_zero_valued_fields_treated_as_not_entered(params):
    # 0.0 is the setup_data JSON blob's own unfilled default for a numeric
    # spinbox -- a real car's corner load can never be 0 kg, so this is the
    # documented "not entered" sentinel, not a real measurement.
    setup_data = {"car": {
        "corner_weight_fl": 0.0, "corner_weight_fr": 0.0,
        "corner_weight_rl": 0.0, "corner_weight_rr": 0.0,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["levels"]["corner_weights"] == 1


def test_partial_fill_never_mixes_measured_and_defaulted(params):
    # Only 3 of 4 corners entered -- must fall back entirely to Level 1,
    # never silently mix one measured corner with the config default for
    # the missing one.
    setup_data = {"car": {
        "corner_weight_fl": 300.0, "corner_weight_fr": 298.0,
        "corner_weight_rl": 396.0,
        # corner_weight_rr missing
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["levels"]["corner_weights"] == 1


def test_full_session_weights_resolve_to_level_2(params):
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["levels"]["corner_weights"] == 2
    assert resolved["values"]["corner_weights"] == {
        "FL_kg": 300.3, "FR_kg": 298.9, "RL_kg": 396.2, "RR_kg": 386.5,
    }


def test_cap_1_forces_level_1_even_with_full_session_weights(params):
    # The exact mechanism that makes cap=1 the suite-wide reproducibility
    # convention (tests/conftest.py FIXED_CAP) -- verified directly here,
    # not just asserted in a comment.
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=1)
    assert resolved["levels"]["corner_weights"] == 1
    assert resolved["values"]["corner_weights"] == _config_corner_weights(params)
    # clipped=True: session data WAS available (best_available_level=2) but
    # the cap held it back -- distinct from "nothing to clip".
    assert resolved["clipped"] is True


# --- Derived fractions --------------------------------------------------

def test_mass_derives_from_corner_weight_sum_when_total_weight_absent(params):
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["levels"]["mass"] == 2
    assert resolved["values"]["mass_kg"] == pytest.approx(300.3 + 298.9 + 396.2 + 386.5)


def test_explicit_total_weight_wins_over_corner_sum(params):
    # Priority rule (accuracy_resolution.py's own docstring): explicit
    # setup_data.total_weight beats the derived corner-weight sum when
    # both are available -- never blended.
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
        "total_weight": 1400.0,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["values"]["mass_kg"] == 1400.0
    assert resolved["values"]["mass_kg"] != pytest.approx(300.3 + 298.9 + 396.2 + 386.5)


def test_mass_inconsistency_warns_but_does_not_block(params):
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
        "total_weight": 2000.0,  # wildly inconsistent with the corner sum (~1382)
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    assert resolved["warnings"], "expected a mass-inconsistency warning"
    assert resolved["values"]["mass_kg"] == 2000.0  # still uses the explicit value, never silently corrected


def test_cog_position_derived_from_corner_weights_matches_static_balance(params):
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    wheelbase = params["vehicle"]["wheelbase_m"]
    front_fraction = (300.3 + 298.9) / (300.3 + 298.9 + 396.2 + 386.5)
    rear_fraction = 1.0 - front_fraction
    # Static 2-mass balance: F_front = m*g*cog_to_rear_axle/wheelbase, so
    # cog_to_rear_axle = wheelbase * front_fraction (same relation
    # config/parameters.json's own placeholder constants were derived
    # from -- verified by hand against the live config before this test
    # was written, not assumed).
    assert resolved["values"]["cog_to_rear_axle_m"] == pytest.approx(wheelbase * front_fraction)
    assert resolved["values"]["cog_to_front_axle_m"] == pytest.approx(wheelbase * rear_fraction)
    assert resolved["levels"]["cog_position"] == 2


def test_cog_position_level_1_reads_config_constants_unrounded(params):
    # At Level 1 the config's own stored constants are used directly
    # (not recomputed from config's own corner_weights) -- deliberately,
    # per accuracy_resolution.py's own comment, to avoid sub-millimetre
    # float drift against byte-identical baselines.
    resolved = resolve_accuracy(params, setup_data=None, cap=None)
    assert resolved["values"]["cog_to_front_axle_m"] == params["vehicle"]["cog_to_front_axle_m"]
    assert resolved["values"]["cog_to_rear_axle_m"] == params["vehicle"]["cog_to_rear_axle_m"]


# --- Synthetic outing with weights: full apply_resolved_vehicle path ---

def test_apply_resolved_vehicle_overrides_effective_params(params):
    setup_data = {"car": {
        "corner_weight_fl": 300.3, "corner_weight_fr": 298.9,
        "corner_weight_rl": 396.2, "corner_weight_rr": 386.5,
    }}
    resolved = resolve_accuracy(params, setup_data=setup_data, cap=None)
    effective = apply_resolved_vehicle(params, resolved)

    assert effective["vehicle"]["mass_kg"] == pytest.approx(1381.9)
    assert effective["vehicle"]["corner_weights"] == {
        "FL_kg": 300.3, "FR_kg": 298.9, "RL_kg": 396.2, "RR_kg": 386.5,
    }
    # Everything else in the vehicle block is untouched (deepcopy, not a
    # partial merge) -- wheelbase is never a setup_data-resolved field.
    assert effective["vehicle"]["wheelbase_m"] == params["vehicle"]["wheelbase_m"]
    # The original params dict itself must be unmutated.
    assert params["vehicle"]["corner_weights"]["FL_kg"] == 290.0


def test_apply_resolved_vehicle_without_session_data_is_byte_identical_to_config(params):
    resolved = resolve_accuracy(params, setup_data=None, cap=None)
    effective = apply_resolved_vehicle(params, resolved)
    assert effective["vehicle"]["mass_kg"] == params["vehicle"]["mass_kg"]
    assert effective["vehicle"]["corner_weights"] == _config_corner_weights(params)
    assert effective["vehicle"]["cog_to_front_axle_m"] == params["vehicle"]["cog_to_front_axle_m"]
    assert effective["vehicle"]["cog_to_rear_axle_m"] == params["vehicle"]["cog_to_rear_axle_m"]
