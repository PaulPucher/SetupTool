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
#
# Cleanup/reliability/presentation pass, Part A (2026-09-01): extended
# with checked-set correctness assertions (A1), an LS display-mask check
# (A4), and a double-click-only guard check (A3) -- A2 (legend icon fix)
# has no headless-observable effect (it only changes which PlotDataItem
# carries a pyqtgraph legend's name= registration) and is left to the
# user's own visual check, per the work order.

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

app = QApplication([])

import numpy as np
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

# A1: unchecking a lap must remove it from EVERY panel and from the
# export cache -- not just hide it, and not leave a stale aggregate
# (pooled reference line / worst-phase window) computed over it.
print("\n--- A1: checked-set correctness on toggle ---")
c5_summaries = [s for s in summaries if s.get("stable_corner_id") == 5]
ctd.show_corner(c5_summaries[0], stability_result_all_laps, data)
all_checked = sorted(ln for ln, v in ctd.lap_visible.items() if v)
assert len(all_checked) >= 2, "need at least 2 checked laps for this corner to test toggling"
before_export_laps = sorted(lap["lap_number"] for lap in ctd._export_data["laps"])
assert before_export_laps == all_checked, \
    f"export laps {before_export_laps} should exactly match the checked set {all_checked} before any toggle"

drop_lap = all_checked[0]
ctd._on_lap_visibility_toggled(drop_lap, False)
assert ctd.lap_visible[drop_lap] is False
# Part B: unchecked laps are not merely hidden any more -- a checkbox
# toggle triggers a full re-render (_rerender_preserving_checked), and
# only CHECKED laps ever get a curve item at all.
assert drop_lap not in ctd.lap_curve_items, f"lap {drop_lap} should have no curve item at all once unchecked"
after_export_laps = sorted(lap["lap_number"] for lap in ctd._export_data["laps"])
assert drop_lap not in after_export_laps, "export cache should drop the unchecked lap immediately"
assert after_export_laps == [ln for ln in all_checked if ln != drop_lap]
print(f"toggling lap {drop_lap} off: curve removed, export cache updated OK")

ctd._on_lap_visibility_toggled(drop_lap, True)
assert ctd.lap_visible[drop_lap] is True
assert drop_lap in ctd.lap_curve_items, f"lap {drop_lap} should have a curve item again once re-checked"
for item in ctd.lap_curve_items[drop_lap]:
    assert item.isVisible(), f"lap {drop_lap}'s Traces-tab curve should be visible once re-checked"
restored_export_laps = sorted(lap["lap_number"] for lap in ctd._export_data["laps"])
assert restored_export_laps == all_checked, "re-checking should restore the export cache to the full checked set"
print(f"toggling lap {drop_lap} back on: curve restored, export cache restored OK")

# A1: unchecking every lap should still leave the track outline (no crash,
# geometry present) with both worst-phase windows gone.
for ln in all_checked:
    ctd._on_lap_visibility_toggled(ln, False)
assert not any(ctd.lap_visible.values()), "every lap should now be unchecked"
# Follow-up item 2: render_corner_figure has the track map back (a
# narrow row, restored after round 3 item 2 had dropped it) -- back to
# asserting against _export_data["track_map"]'s own geometry.
if ctd._export_data is not None and ctd._export_data.get("track_map") is not None:
    tm = ctd._export_data["track_map"]
    assert tm["lap_xy"] is not None, "track outline should remain with zero laps checked"
    assert tm["window_f_xy"] is None and tm["window_r_xy"] is None, \
        "no checked lap should mean no worst-phase window on either axle"
print("zero laps checked: track outline remains, both axle windows absent OK")
for ln in all_checked:
    ctd._on_lap_visibility_toggled(ln, True)  # restore before the next section

# Part B: colour is a function of ascending POSITION in the checked set,
# not of the lap number -- unchecking the lowest-numbered checked lap
# should shift every remaining lap's colour up by one palette slot.
print("\n--- Part B: dynamic per-checked-lap colour assignment ---")
from core import plot_style

before_colors = {ln: ctd._current_styles[ln]["color"] for ln in all_checked}
second_lowest = all_checked[1]
assert before_colors[all_checked[0]] == plot_style.LAP_PALETTE[0]
assert before_colors[second_lowest] == plot_style.LAP_PALETTE[1]
ctd._on_lap_visibility_toggled(all_checked[0], False)
after_colors = {ln: ctd._current_styles[ln]["color"] for ln in all_checked if ln != all_checked[0]}
assert after_colors[second_lowest] == plot_style.LAP_PALETTE[0], \
    "the new lowest-numbered checked lap should take palette slot 0 once lap {all_checked[0]} is unchecked"
print(f"unchecking lap {all_checked[0]}: lap {second_lowest} correctly shifted from palette slot 1 to slot 0")
ctd._on_lap_visibility_toggled(all_checked[0], True)  # restore before the next section

# A4: LS display mask -- config value is read, and at least one sample in
# this corner's own window should have been masked to NaN by it (Dubai's
# own |ax| distribution, see config/parameters.json's derivation note,
# says only ~10% of racing time sits at or below 1.0 m/s^2, so a masked
# sample should exist somewhere in a multi-lap corner window).
print("\n--- A4: LS display mask ---")
ls_min_ax = params["corner_trace_display"]["ls_display_min_ax_mps2"]
print(f"config ls_display_min_ax_mps2 = {ls_min_ax}")
any_masked = any(
    lap["ls_f"] is not None and np.isnan(lap["ls_f"]).any()
    for lap in ctd._export_data["laps"]
)
print(f"any LS_f sample masked to NaN by the |ax| gate in this corner's export: {any_masked}")

print("\n--- LapTraceDialog: constructing ---")


def _classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


ltd = LapTraceDialog()
print("constructed OK")
assert "ls_f" not in ltd.plots and "ls_r" not in ltd.plots, "A4: the full-lap trace should have no LS panel at all"
assert len(ltd.PANEL_TITLES) == 4, \
    "Part B: full-lap trace scaffold should be exactly 4 panels (speed/stab/cs_f/cs_r)"
print("A4: full-lap trace has no LS panel OK")

valid_laps = sorted({l["lap_number"] for l in data.get("laps", []) if l.get("is_valid_for_analysis")})
for ln in valid_laps[:2]:
    print(f"show_lap: {ln}")
    ltd.show_lap(ln, stability_result, data, _classify_fn)
print("show_lap calls completed OK")

# A3: a plain click on a corner band must NOT open CornerTraceDialog --
# only a double-click (event.double()) may.
print("\n--- A3: double-click-only guard on the full-lap trace ---")
from PyQt6.QtCore import QPointF

click_log = []
ltd._on_corner_click = lambda summary: click_log.append(summary["stable_corner_id"])


class _FakeClickEvent:
    def __init__(self, pos, double):
        self._pos = pos
        self._double = double

    def scenePos(self):
        return self._pos

    def double(self):
        return self._double


assert ltd._corner_bands, "expected at least one corner band after show_lap"
start_s, end_s, cid = ltd._corner_bands[0]
top_plot = ltd.plots["stab"]
scene_pos = top_plot.getViewBox().mapViewToScene(QPointF((start_s + end_s) / 2.0, 0))

ltd._on_scene_clicked(_FakeClickEvent(scene_pos, double=False))
assert click_log == [], "a single (non-double) click on a corner band must NOT open the corner trace"
print("single click on a corner band: correctly ignored")

ltd._on_scene_clicked(_FakeClickEvent(scene_pos, double=True))
assert click_log == [cid], f"a double-click on corner band C{cid} should open its corner trace exactly once"
print(f"double-click on corner band C{cid}: correctly opened")

# Addendum item 4's own explicit requirement: verify against a SYNTHETIC
# 20-lap result (this real session only has ~5 valid laps) that the
# default-N-fastest, all-20-checkable, and export-exactly-checked
# behaviours all scale past the real dataset's own lap count. Built by
# cloning corner 5's own real lap-1 instance/lap/summary under 20 new,
# clearly-synthetic lap numbers (1000..1019) with distinct fabricated lap
# times (so "fastest N" has a real ordering to pick from) -- CLAUDE.md's
# "real data only" rule governs analysis/validation, not this headless
# UI-plumbing scale test, which is exactly what the addendum's own work
# order asks for here.
print("\n--- Addendum: synthetic 20-lap default/all-checked/export scale test ---")
SYNTHETIC_CORNER_ID = 999  # a stable_corner_id no real corner uses, so this
                           # test's instances list can never pick up C5's own
                           # real lap 1-4 instances alongside the synthetic 20
base_corner = next(c for c in corners if c.get("stable_corner_id") == 5 and c["lap_number"] == 1)
base_lap = next(l for l in data["laps"] if l["lap_number"] == 1)
base_summary = next(s for s in summaries if s.get("stable_corner_id") == 5 and s["lap_number"] == 1)
base_lap_time = base_lap.get("lap_time_precise") or base_lap.get("lap_time") or 100.0

synthetic_lap_numbers = [1000 + i for i in range(20)]
synthetic_laps = [
    dict(base_lap, lap_number=ln, is_valid_for_analysis=True,
         lap_time=base_lap_time + i * 0.01, lap_time_precise=base_lap_time + i * 0.01)
    for i, ln in enumerate(synthetic_lap_numbers)
]
synthetic_corners = [dict(base_corner, lap_number=ln, stable_corner_id=SYNTHETIC_CORNER_ID)
                      for ln in synthetic_lap_numbers]
synthetic_summaries = [dict(base_summary, lap_number=ln, stable_corner_id=SYNTHETIC_CORNER_ID)
                        for ln in synthetic_lap_numbers]

synthetic_parsed_data = dict(data, laps=data["laps"] + synthetic_laps)
synthetic_stability_result = dict(
    stability_result, corners=corners + synthetic_corners,
    summaries=summaries + synthetic_summaries, lap_filter=None,
)

ctd.show_corner(synthetic_summaries[0], synthetic_stability_result, synthetic_parsed_data)
checked_now = sorted(ln for ln, v in ctd.lap_visible.items() if v)
n_default = params["corner_trace_display"]["default_laps_shown"]
assert len(checked_now) == n_default, f"expected {n_default} laps checked by default, got {len(checked_now)}"
assert set(checked_now).issubset(set(synthetic_lap_numbers)), "default-checked laps should be among the 20 synthetic ones"
print(f"20-lap corner opened: {len(checked_now)} laps checked by default (config default_laps_shown={n_default}) OK")

for ln in synthetic_lap_numbers:
    ctd._on_lap_visibility_toggled(ln, True)
all_checked_20 = sorted(ln for ln, v in ctd.lap_visible.items() if v)
assert all_checked_20 == synthetic_lap_numbers, f"expected all 20 synthetic laps checked, got {all_checked_20}"
export_laps_20 = sorted(lap["lap_number"] for lap in ctd._export_data["laps"])
assert export_laps_20 == all_checked_20, "export should contain exactly the 20 checked laps, no more, no fewer"
print(f"all 20 laps checked: export contains exactly {len(export_laps_20)} laps, no more no fewer OK")

print("\nALL SMOKE TESTS PASSED -- no exception raised during construction or render.")
