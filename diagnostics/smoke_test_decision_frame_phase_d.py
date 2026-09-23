# Headless smoke test for Phase D's own top-line/tail rendering
# (DECISION LAYER SPEC, ui/views/outing_form.py's Decision Frame section,
# 2026-09-22). Same offscreen-Qt technique as smoke_test_decision_frame_
# widget.py; that script checks the widget BINDING (toggle, enable/
# disable, "generation does not raise"), this one checks the actual
# CONTENT the D6 STOP made load-bearing: a top line never carries a
# corner id or stray prose, and a candidate with no real delta (the
# matrix-bridge-sourced, direction-only class) never renders a digit it
# does not have. [keep-reproduces] per diagnostics/README.md.
#
# Runs the real pipeline against Sample_Dubai.txt (kinematic sideslip,
# same fast/no-config-mutation choice smoke_test_decision_frame_widget.py
# already made), once through a real OutingForm (catches Qt integration
# errors) and once by calling modules.decision_frame.generate_candidates/
# generate_display_split directly with the form's own _classify_corner
# (catches content regressions render_top_line/render_tail_line cannot
# hide behind a widget).

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import re

from PyQt6.QtWidgets import QApplication, QLabel, QPushButton

app = QApplication([])

import models.driver, models.outing, models.raceweekend
from models.base import Session
from models.outing import Outing
from models.raceweekend import RaceWeekend
from ui.views.outing_form import OutingForm

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.decision_frame import (
    build_evidence, aggregate_ls_by_corner, load_decision_frame_config,
    generate_candidates, generate_display_split, resolve_conflicts, group_display_rows,
    render_top_line, render_tail_line, render_action_line, STATUS_NO_TRIGGER, STATUS_PROPOSED,
)
from modules.recommendation import load_setup_parameters_registry, _group_by_corner

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
CORNER_ID_RE = re.compile(r"\bC\d+\b")
DIGIT_RE = re.compile(r"\d")

print("--- real pipeline (kinematic, fast) on Sample_Dubai.txt ---")
data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
long_forces = estimate_longitudinal_forces(state, data["channels"], params)
slip_ratio = estimate_slip_ratio(state, data["channels"], params)
ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, params)
summaries = summarise_corners(data.get("corners", []), cs, stab, state, ls=ls, lap_filter=None)

s = Session()
weekend = s.get(RaceWeekend, 1)
outing = s.get(Outing, 1)
form = OutingForm(weekend, lambda: None, outing=outing)

print("\n--- content check: rebuild shortlist/tail directly, same call the form makes ---")
config = load_decision_frame_config()
registry = load_setup_parameters_registry()
ls_stats = aggregate_ls_by_corner(summaries)
evidence = build_evidence(
    summaries, ls_stats, config, form._classify_corner,
    corners=data.get("corners"), state=state, channels=data["channels"],
)
assessed_corner_ids = set(_group_by_corner(summaries).keys())
candidates = generate_candidates(evidence, registry, config, setup_data=None,
                                  assessed_corner_ids=assessed_corner_ids)
split = generate_display_split(candidates, evidence, None, config, registry)
shortlist, tail = split["shortlist"], split["tail"]
resolve_conflicts(shortlist)
ungrouped_shortlist_count, ungrouped_tail_count = len(shortlist), len(tail)
shortlist = group_display_rows(shortlist, registry)
tail = group_display_rows(tail, registry)
print(f"shortlist={len(shortlist)} (ungrouped {ungrouped_shortlist_count}) "
      f"tail={len(tail)} (ungrouped {ungrouped_tail_count})")
assert shortlist or tail, "expected at least one inventory entry (42-lever registry never empty)"

print("\n--- Phase D feedback round ITEM 1: identical rows grouped for display ---")
shortlist_lines = [render_top_line(c, registry) for c in shortlist]
assert len(shortlist_lines) == len(set(shortlist_lines)), \
    f"two visible shortlist rows render identical top-line strings: {shortlist_lines}"
print(f"OK -- {len(shortlist)} distinct shortlist top lines, no duplicates")
grouped_shortlist = [c for c in shortlist if c.get("group_members")]
if grouped_shortlist:
    for g in grouped_shortlist:
        assert g["score"] == max(m.get("score", float("-inf")) for m in g["group_members"]), \
            "group score must be the MAX member score, never summed"
        print(f"  OK group {render_top_line(g, registry)!r}: "
              f"{len(g['group_members'])} members, score={g['score']:.2f} (max, not summed)")
else:
    print("  (no groups formed on this run's own data -- see synthetic check below)")

print("\n--- D1: shortlist top lines carry no corner id ---")
for c in shortlist:
    line = render_top_line(c, registry)
    assert not CORNER_ID_RE.search(line), f"corner id leaked into top line: {line!r}"
    print(f"  OK {line!r}")

print("\n--- D6: direction-only candidates (no real delta) render no digit ---")
checked_direction_only = 0
for c in shortlist + [e for e in tail if e.get("status") != STATUS_NO_TRIGGER]:
    has_real_delta = any(
        a.get("delta") and registry.get(a["parameter"], {}).get("value_space", {}).get("type") in ("int", "float")
        and registry.get(a["parameter"], {}).get("value_space", {}).get("unit")
        for a in c.get("actions", [])
    )
    if c.get("actions") and not has_real_delta:
        line = render_top_line(c, registry)
        assert not DIGIT_RE.search(line), f"direction-only candidate rendered a digit: {line!r}"
        checked_direction_only += 1
        print(f"  OK direction-only {line!r}")
print(f"checked {checked_direction_only} real direction-only candidate(s) on this run's own data")

# The kinematic-sideslip fast path this smoke test uses (same choice
# smoke_test_decision_frame_widget.py already made, for speed) does not
# always happen to fire a matrix_bridge-sourced (no-delta) or enum-lever
# candidate on this file -- which class fires depends on the sideslip
# source's own verdicts, not on Phase D's rendering code. D6's own rule is
# about the RENDERING function, not about which candidates today's data
# happens to produce, so it is checked directly here against synthetic
# actions shaped exactly like the two real no-coverage classes found
# during the D6 STOP (a matrix_bridge action with no delta key at all, and
# an enum-lever action carrying the routing-sign delta some generators
# attach): never renders a magnitude that is not really there.
print("\n--- D6 (synthetic, guaranteed coverage): no-delta and enum-lever actions ---")
no_delta_action = {"parameter": "tc_lon", "direction": "decrease"}  # matrix_bridge shape: no "delta" key
enum_symbolic_action = {"parameter": "wing_position", "direction": "increase", "delta": 1}  # routing sign, not mm/deg
for action, expect_phrase in ((no_delta_action, "less intervention"), (enum_symbolic_action, "higher position")):
    line = render_action_line(action, registry)
    assert not DIGIT_RE.search(line), f"D6 violated -- rendered a digit with no real magnitude: {line!r}"
    assert expect_phrase in line, f"expected direction-only phrase {expect_phrase!r} in {line!r}"
    print(f"  OK {action['parameter']}/{action['direction']} -> {line!r}")

print("\n--- ITEM 1 (synthetic, guaranteed coverage): grouping collapses duplicates ---")
c1 = {"id": "a", "corner": 4, "phase": "exit_4", "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
      "score": 0.30, "evidence_refs": [], "status": STATUS_PROPOSED}
c2 = {"id": "b", "corner": 9, "phase": "exit_4", "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
      "score": 0.55, "evidence_refs": [], "status": STATUS_PROPOSED}
c3 = {"id": "c", "corner": 6, "phase": "exit_4", "actions": [{"parameter": "diff_position", "direction": "increase", "delta": 1}],
      "score": 0.40, "evidence_refs": [], "status": STATUS_PROPOSED}
grouped = group_display_rows([c1, c2, c3], registry)
assert len(grouped) == 2, f"expected 2 display rows (2 tc_lon collapse, diff_position stays separate), got {len(grouped)}"
tc_group = next(g for g in grouped if g["id"] in ("a", "b"))
assert tc_group["group_members"] == [c1, c2]
assert tc_group["score"] == 0.55, "group score must be MAX (0.55), never summed (0.85) or first-member (0.30)"
assert tc_group["corner"] == 9, "representative fields must come from the max-score member (c2, corner 9)"
print(f"  OK synthetic: 2 identical tc_lon candidates (scores 0.30/0.55) -> "
      f"1 row, score={tc_group['score']}, corner={tc_group['corner']} (from the max-score member)")

no_trigger_a = {"status": STATUS_NO_TRIGGER, "lever": "camber_fl"}
no_trigger_b = {"status": STATUS_NO_TRIGGER, "lever": "camber_fr"}
grouped_nt = group_display_rows([no_trigger_a, no_trigger_b], registry)
assert len(grouped_nt) == 2, "two different levers' no_trigger rows must never collapse into one"
print("  OK synthetic: two different levers' no_trigger rows stay separate")

print("\n--- D2: tail ordering -- real candidates (by score desc) before no_trigger rows ---")
seen_no_trigger = False
last_score = None
for entry in tail:
    if entry.get("status") == STATUS_NO_TRIGGER:
        seen_no_trigger = True
        continue
    assert not seen_no_trigger, "a real tail candidate appeared after a no_trigger row"
    assert entry.get("status") != STATUS_PROPOSED or entry.get("score", 0) < config["display_score_threshold"]["value"], \
        "a proposed tail entry scores at/above threshold -- should be in shortlist"
    if last_score is not None:
        assert entry.get("score", 0) <= last_score, "tail real candidates not ranked by descending score"
    last_score = entry.get("score", 0)
    line = render_tail_line(entry, registry)
    print(f"  OK tail: {line!r}")
print(f"tail has {sum(1 for e in tail if e.get('status') == STATUS_NO_TRIGGER)} no_trigger row(s)")

print("\n--- widget construction: OutingForm builds real rows without raising ---")
# Uses the REAL setup_data this fixture outing's own _collect_setup_data()
# returns (blank sheet), unlike the content check above (setup_data=None,
# which skips the window-edge check entirely) -- Stage 3's own "unfilled
# sheet -> not_assessable" rule means this can legitimately move every
# setup-sheet-backed candidate into the tail on a blank test outing. That
# is correct pre-existing B5 behaviour, not a Phase D regression, so this
# section checks the widget's own rendered content on its own terms
# rather than asserting it matches the setup_data=None reconstruction.
form.stability_result = {"summaries": summaries, "corners": data.get("corners"), "state": state}
form.parsed_data = data
form.btn_generate_decision_frame.setEnabled(True)
form._generate_decision_frame()
print(f"summary label: {form.decision_frame_summary_label.text()!r}")

shortlist_labels = [
    w.text() for w in form.decision_frame_host.findChildren(QLabel)
    if "font-weight: 600; padding: 3px 8px" in w.styleSheet()
]
print(f"rendered {len(shortlist_labels)} shortlist badge label(s): {shortlist_labels}")
for text in shortlist_labels:
    assert not CORNER_ID_RE.search(text), f"widget-rendered top line leaked a corner id: {text!r}"
print("OK -- no corner id in any widget-rendered shortlist badge")
assert len(shortlist_labels) == len(set(shortlist_labels)), \
    f"ITEM 1 violated -- widget rendered two visible rows with identical top-line text: {shortlist_labels}"
print("OK -- no two widget-rendered shortlist badges are identical (ITEM 1)")

tail_toggles = [
    w for w in form.decision_frame_host.findChildren(QPushButton)
    if w.text().startswith(("> assessed, not proposed", "v assessed, not proposed"))
]
reasoning_buttons = [
    w for w in form.decision_frame_host.findChildren(QPushButton)
    if w.text() in ("> reasoning", "v reasoning")
]
assert reasoning_buttons, "no '> reasoning' expand buttons found -- dropdown content missing entirely"
if tail_toggles:
    assert tail_toggles[0].isChecked() is False, "tail must be collapsed by default (D2)"
    print(f"OK tail toggle present and collapsed: {tail_toggles[0].text()!r}")
    tail_toggles[0].setChecked(True)
    assert tail_toggles[0].text().startswith("v assessed, not proposed")
    print("OK tail toggle expands on click")

print("\n--- dropdown content presence: expand the first row's reasoning ---")
reasoning_buttons[0].setChecked(True)
assert reasoning_buttons[0].text() == "v reasoning"
print(f"OK {len(reasoning_buttons)} reasoning dropdown(s) present and toggle correctly")

form._clear_decision_frame_rows()
s.close()

print("\nALL PHASE D SMOKE TESTS PASSED")
