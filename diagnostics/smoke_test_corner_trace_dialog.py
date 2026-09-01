# One-off headless smoke test for the UI cleanup package's corner_trace_
# dialog.py changes (legend wiring, fitted-curve overlay) -- NOT part of
# the regression suite, run manually. Uses Qt's offscreen platform
# plugin so it can execute without a real display. Constructs a real
# QApplication + CornerTraceDialog/LapTraceDialog and drives show_corner/
# show_lap with a REAL analysis result (live config sideslip_source) to
# catch any runtime error (e.g. a pyqtgraph API misuse) that a pure
# syntax check cannot -- this project's own convention leaves interactive
# UI testing to the user, but a headless construction+render pass is
# worth the few seconds given the legend/overlay code was written without
# being able to visually verify it.

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

app = QApplication([])

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    estimate_vertical_loads, summarise_corners, estimate_slip_angles,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
# Cleanup/reliability pass: exercise the LS panel (added since this
# script was last touched) and the lap_filter-respecting checkbox
# default, not just cs/stab/speed.
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

params = load_parameters()
sideslip_source = params["stability_estimation"]["sideslip_source"]
print(f"live sideslip_source = {sideslip_source!r}")

data = parse_csv(RAW_FILE)
state = prepare_vehicle_state(data["channels"], params)
beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
    state, params, data, sideslip_source, csv_path=RAW_FILE
)
print(f"fallback_used={fallback_used}  gate_verdict={gate_verdict['verdict'] if gate_verdict else None}")

slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
fz = estimate_vertical_loads(state, forces, params)
long_forces = estimate_longitudinal_forces(state, data["channels"], params)
slip_ratio = estimate_slip_ratio(state, data["channels"], params)
ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)
corners = data.get("corners", [])
summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls, lap_filter=None)

stability_result = {
    "state": state, "cs": cs, "stab": stab, "fz": fz, "ls": ls, "slip": slip, "forces": forces,
    "corners": corners, "summaries": summaries, "sideslip_source": sideslip_source,
    "fit_manifest": fit_manifest, "gate_verdict": gate_verdict,
    "lap_filter": [1],  # exercise the single-lap-selected checkbox default too
}

from ui.views.corner_trace_dialog import CornerTraceDialog, LapTraceDialog

print("\n--- CornerTraceDialog: constructing ---")
ctd = CornerTraceDialog()
print("constructed OK")

# Drive show_corner for several corners, including C5 (this session's
# own investigation target) and whichever corner sorts first per lap.
target_summaries = [s for s in summaries if s.get("stable_corner_id") in (1, 5, 8)]
for s in target_summaries:
    print(f"show_corner: lap={s['lap_number']} C{s['stable_corner_id']}")
    ctd.show_corner(s, stability_result, data)
print("show_corner calls completed OK")
assert ctd.lap_visible.get(1) is True, "lap_filter=[1] should default only lap 1 checked"
assert not any(v for ln, v in ctd.lap_visible.items() if ln != 1), \
    "lap_filter=[1] should leave every other lap unchecked by default"
print("lap_filter single-lap default-checked behaviour OK")

# Re-render with lap_filter reset to None (the "All laps" case) to
# confirm the old all-checked default still holds -- both branches of
# the Phase 1 lap-selection fix exercised in one script.
stability_result_all_laps = dict(stability_result, lap_filter=None)
ctd.show_corner(target_summaries[0], stability_result_all_laps, data)
assert all(ctd.lap_visible.values()), "lap_filter=None ('All laps') should default every lap checked"
print("lap_filter=None ('All laps') default-checked behaviour OK")

print("\n--- LapTraceDialog: constructing ---")


def _classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


ltd = LapTraceDialog()
print("constructed OK")
valid_laps = sorted({l["lap_number"] for l in data.get("laps", []) if l.get("is_valid_for_analysis")})
for ln in valid_laps[:2]:
    print(f"show_lap: {ln}")
    ltd.show_lap(ln, stability_result, data, _classify_fn)
print("show_lap calls completed OK")

print("\nALL SMOKE TESTS PASSED -- no exception raised during construction or render.")
