# Kerb-gap bridge in the trace windows is display-only: it must draw across
# kerb holes without ever reaching an analysis array. Real Dubai data.

import copy
import os
import re

import numpy as np

from core.trace_display import kerb_gap_bridge


def _nan_equal(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return a.shape == b.shape and np.array_equal(np.isnan(a), np.isnan(b)) and \
        np.array_equal(a[~np.isnan(a)], b[~np.isnan(b)])


def test_bridge_spans_kerb_gaps_only_and_leaves_input_untouched(pipeline_result, parsed_data, state):
    cs = pipeline_result["cs"]
    t, s_m, kerb = state["time"], state["s_m"], state["kerb_mask"]
    bridged_runs = 0
    for lap in parsed_data["laps"]:
        if not lap.get("is_valid_for_analysis"):
            continue
        m = (t >= lap["start_time"]) & (t < lap["end_time"]) & np.isfinite(s_m)
        x, k = s_m[m], kerb[m]
        for key in ("CS_ratio_f", "CS_ratio_r"):
            y = cs[key][m]
            before = y.copy()
            yb = kerb_gap_bridge(x, y, k)
            assert _nan_equal(y, before), "the bridge must not modify its input"
            assert yb is not y
            fin = np.isfinite(yb)
            # every bridged sample is either a finite anchor equal to y, or a
            # NaN sample of y inside a run that holds a kerb sample
            assert np.allclose(yb[fin & np.isfinite(y)], y[fin & np.isfinite(y)])
            gap = fin & ~np.isfinite(y)
            edges = np.diff(np.concatenate(([0], gap.astype(np.int8), [0])))
            for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
                assert k[a:b].any(), "a bridged gap must contain a kerb sample"
                bridged_runs += 1
            # a NaN run without kerb samples is never bridged
            bad = ~np.isfinite(y)
            e2 = np.diff(np.concatenate(([0], bad.astype(np.int8), [0])))
            for a, b in zip(np.where(e2 == 1)[0], np.where(e2 == -1)[0]):
                if not k[a:b].any():
                    assert not np.isfinite(yb[a:b]).any()
    assert bridged_runs > 0, "Dubai has kerb strikes inside analysis laps; expected at least one bridge"


def test_analysis_output_identical_with_bridge_flag_on_and_off(parsed_data, effective_params):
    from modules.stability_analysis import (
        prepare_vehicle_state, estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
    )

    def run(flag):
        p = copy.deepcopy(effective_params)
        p["corner_trace_display"]["kerb_gap_bridge"] = flag
        st = prepare_vehicle_state(parsed_data["channels"], p)
        beta = estimate_sideslip(st, p)
        slip = estimate_slip_angles(st, beta, p)
        forces = estimate_lateral_forces(st, p)
        cs = estimate_cornering_stiffness(slip, forces, st, p)
        stab = estimate_yaw_moment_stability(st, beta, p, parsed_data.get("laps", []))
        summ = summarise_corners(parsed_data.get("corners", []), cs, stab, st)
        return st, beta, cs, stab, summ

    on, off = run(True), run(False)
    for k in ("yaw_rate_radps", "ay_mps2", "v_mps", "kerb_mask"):
        assert np.asarray(on[0][k]).tobytes() == np.asarray(off[0][k]).tobytes()
    assert on[1].tobytes() == off[1].tobytes()
    for k in ("CS_ratio_f", "CS_ratio_r"):
        assert on[2][k].tobytes() == off[2][k].tobytes()
    assert on[3]["stability_observed_Nm_per_deg"].tobytes() == off[3]["stability_observed_Nm_per_deg"].tobytes()
    assert repr(on[4]) == repr(off[4])


def test_no_analysis_module_reads_the_bridge():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pat = re.compile(r"kerb_gap_bridge|trace_display")
    for d in ("modules", "core"):
        for name in os.listdir(os.path.join(root, d)):
            if not name.endswith(".py") or (d == "core" and name == "trace_display.py"):
                continue
            with open(os.path.join(root, d, name), encoding="utf-8") as fh:
                assert not pat.search(fh.read()), f"{d}/{name} references the display-only bridge"
