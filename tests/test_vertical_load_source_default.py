# Fz-integration package close-out (2026-09-04): stability_estimation.
# vertical_load_source default flip ("static" -> "measured"), authorised
# because verdict independence is PROVEN (CS_ratio/stability never
# receive fz at all -- thesis_notes.md "Fz-integration Phase 1 (finish):
# CS_ratio independence..."), not because the numeric behaviour is
# expected to be identical -- it is not, by design (that is the whole
# point of Fz-integration). This test covers the one thing that DOES
# matter for the default itself: does modules.stability_analysis.
# estimate_vertical_loads resolve correctly under the LIVE config
# default, both with real damper channels present and with none at all
# (the "static remains the automatic path where channels are absent"
# guarantee the config's own derived_from comment states).
#
# Synthetic fixtures only (state/forces/channels/car_data all hand-
# built) -- same category as tests/test_wheel_loads.py's own fixtures,
# reusing its car_data/channel-construction shape for consistency.

import numpy as np

from modules.stability_analysis import load_parameters, estimate_vertical_loads


def _car_data():
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


def _channel(time, data, quality="valid", unit_raw="N"):
    return {"time": time, "data": data, "quality": quality, "unit_raw": unit_raw}


def _state_and_forces(n=30):
    t = np.arange(n, dtype=float)
    # Half straight-line (|ax|,|ay| small -- needed so modules.wheel_loads.
    # estimate_session_corrected_axle_totals's own straight-line masks are
    # non-empty, avoiding a benign-but-noisy "mean of empty slice" warning
    # a first version of this fixture (constant ay=3.0, never "straight")
    # produced), half cornering, so the per-wheel lateral-transfer split
    # is exercised too.
    ay = np.where(np.arange(n) < n // 2, 0.1, 3.0)
    state = {
        "time": t, "v_mps": np.full(n, 40.0), "ax_mps2": np.zeros(n), "ay_mps2": ay,
    }
    forces = {"Fy_f_filt": np.full(n, 1500.0), "Fy_r_filt": np.full(n, 1800.0)}
    return state, forces


def test_live_default_is_measured():
    params = load_parameters()
    assert params["stability_estimation"]["vertical_load_source"] == "measured", (
        "Fz-integration package close-out flipped this default deliberately -- "
        "if this now fails, the config default was changed back; update this "
        "assertion only if that is an intended, recorded decision"
    )


def test_default_resolves_to_measured_cascade_with_real_channels():
    params = load_parameters()
    state, forces = _state_and_forces()
    n = len(state["time"])
    # Real variation, not flat -- a perfectly constant channel correctly
    # trips the dead-channel guard (modules.wheel_loads._channel_is_dead,
    # Fz-integration Phase 1), which a first version of this fixture found
    # out the hard way (every corner fell to static_fallback, not damper).
    wobble = 5.0 * np.sin(np.linspace(0, 6.0, n))
    travel = {c: wobble.copy() for c in ("fl", "fr", "rl", "rr")}
    force = {"fl": 3000.0 + 200.0 * np.sin(np.linspace(0, 4.0, n)),
             "fr": 3000.0 + 200.0 * np.sin(np.linspace(0, 4.0, n)),
             "rl": 4000.0 + 200.0 * np.sin(np.linspace(0, 4.0, n)),
             "rr": 4000.0 + 200.0 * np.sin(np.linspace(0, 4.0, n))}
    channels = {}
    for c in ("fl", "fr", "rl", "rr"):
        channels[f"log_dms_dam_{c}"] = _channel(state["time"], force[c], unit_raw="N")
        channels[f"log_susp_travel_{c}"] = _channel(state["time"], travel[c], unit_raw="mm")

    fz = estimate_vertical_loads(state, forces, params, channels=channels, car_data=_car_data())

    assert fz["vertical_load_source_used"] == "measured"
    assert fz["vertical_load_source_per_sample"] is not None
    for c in ("fl", "fr", "rl", "rr"):
        assert (fz["vertical_load_source_per_sample"][c] == "damper").all(), (
            f"{c}: all four corners have real, valid damper channels in this fixture -- "
            "every sample must resolve to the damper tier, not reconstructed/static_fallback"
        )
    # fz_f_N/fz_r_N must stay internally consistent with the per-wheel split
    # regardless of which tier produced each corner (same invariant the
    # production code's own comment states).
    np.testing.assert_allclose(fz["fz_f_N"], fz["fz_fl_N"] + fz["fz_fr_N"])
    np.testing.assert_allclose(fz["fz_r_N"], fz["fz_rl_N"] + fz["fz_rr_N"])


def test_default_resolves_to_static_automatically_without_channels():
    params = load_parameters()
    state, forces = _state_and_forces()

    fz_measured_path = estimate_vertical_loads(state, forces, params, channels=None, car_data=None)
    fz_static_explicit = estimate_vertical_loads(state, forces, {
        **params,
        "stability_estimation": {**params["stability_estimation"], "vertical_load_source": "static"},
    })

    assert fz_measured_path["vertical_load_source_used"] == "static", (
        "no channels/car_data supplied -- must fall back to the static path automatically, "
        "per config's own vertical_load_source_derived_from guarantee"
    )
    assert fz_measured_path["vertical_load_source_per_sample"] is None
    # Numerically identical to an explicit static run -- the provable-by-
    # construction identity the config comment states, checked directly.
    for key in ("fz_f_N", "fz_r_N", "fz_fl_N", "fz_fr_N", "fz_rl_N", "fz_rr_N"):
        np.testing.assert_allclose(fz_measured_path[key], fz_static_explicit[key])
