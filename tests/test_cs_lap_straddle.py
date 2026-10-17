# R11 guard (2026-10-17): a CS regression window that would straddle a lap
# boundary of the lap table yields no value and is counted. Real Dubai data,
# kinematic sideslip as in test_kerb_smear_grade (the guard does not depend
# on the beta source).

import numpy as np

from modules.stability_analysis import (
    estimate_cornering_stiffness, estimate_lateral_forces, estimate_sideslip, estimate_slip_angles,
    reconstruct_cs_window_start, resolve_cs_min_window_samples,
)


def test_no_value_from_a_window_over_a_lap_boundary(state, effective_params, parsed_data):
    se = effective_params["stability_estimation"]
    laps = parsed_data["laps"]
    beta = estimate_sideslip(state, effective_params)
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params, laps)
    no_laps = estimate_cornering_stiffness(slip, forces, state, effective_params, [])
    t = state["time"]
    lap_id = np.full(t.size, -1)
    for lap in laps:
        lap_id[(t >= lap["start_time"]) & (t <= lap["end_time"])] = lap["lap_number"]
    min_window = resolve_cs_min_window_samples(effective_params, state["sample_rate_hz"])
    for ax in "fr":
        alpha = slip[f"alpha_{ax}_filt"]
        removed = np.isfinite(no_laps[f"C_alpha_{ax}"]) & ~np.isfinite(cs[f"C_alpha_{ax}"])
        # every removed sample is a straddling window, and the count says so
        assert int(removed.sum()) == cs[f"n_lap_straddle_{ax}"] > 0
        for i in np.flatnonzero(np.isfinite(cs[f"C_alpha_{ax}"]) | removed):
            start = reconstruct_cs_window_start(alpha, int(i), min_window, se["cs_min_slip_angle_span_rad"],
                                                state["s_m"], se["cs_max_window_m"])
            assert removed[i] == (lap_id[start] != lap_id[i - 1]), f"axle {ax} sample {i}"
        assert no_laps[f"n_lap_straddle_{ax}"] == 0
