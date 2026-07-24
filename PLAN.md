## STATUS (update at every work stop)
Current WP: WP2b-1 complete (2026-07-23), WP2b-2 next. Last commit:
8ec11c5 (WP2b-1: parameter registry, manufacturer reference data,
car.json + setup UI extensions) -- committed and pushed.
WP2b-1 that session: config/car_data.json (manufacturer reference
data digitised from docs/car_data/, gitignored/local-only, consumer-
scoped -- only tables with a named registry/WP5b/module consumer are
digitised, everything else stays in the docs/car_data/ image archive
undigitised); config/setup_parameters.json (46-entry tunable
registry, direction_semantics cross-checked against car_data.json
per entry -- caught and fixed a real tc_lat/tc_lon inversion and
rewrote abs_position as categorical, not monotonic; gained a
`value_source` field ("setup_sheet" default vs "logged_data",
independent of recommendation_target) after tc_lat/tc_lon/
abs_position/brake_bias were identified as driver-adjusted
mid-session from the wheel, not setup-sheet targets -- their
maps_to is null pending a channel-name identification pass, see the
new "NEW DATA FILE — DIAGNOSTIC CHECKLIST" section above WP1);
car.json gained arb_front_mount and
differential_locking_torque_measured only (car dict, no new
front_axle tier -- schema decision); outing_form.py renders those
two new fields (QComboBox support added to the generic
_collect_inputs/_load_inputs for arb_front_mount; flat-key<->nested
reshape helpers for the diff torque table, round-trip verified,
including an offscreen QComboBox collect/load smoke test). PLAN.md
WP5b gained item (f) steering-ratio lookup as steering_ratio_table's
registry consumer.
Open threads: WP2b-2 (rewrite recommendations.json rules against the
new registry keys); the new-data-file channel scan for
tc_lat/tc_lon/abs_position/brake_bias (checklist above WP1); UI
verification checklist (arb_front_mount dropdown + 5 diff-torque
cells render in setup+setdown, save/reopen round-trip, setdown
mirror, PDF export, test_stability) -- user to run manually per
CLAUDE.md UI-change rule; track_check naming verdict (T5-T6 region,
user-confirmed two real corners = one load event); second-track
clustering validation when new data arrives (also tests the
overlap-fraction gap-vs-overlap assumption); wing_position
spinbox-vs-enum UI polish noted under WP4.

Cross-cutting session (2026-07-24, not tied to a WP number, currently
uncommitted): the Werner (2021) attribution pass never got committed
on its own before the foundations-audit/cleanup pass started, so both
now sit together in the working tree and will land as one combined
commit. (1) Werner (2021) attribution pass after a primary-source
verification -- full citation closed out
in thesis_notes.md §6; framing corrected project-wide from "follows/
matches Werner" to "adopts Werner's framework as-is, adapts only the
effective-stiffness estimation (no validated tyre model), extends via
the WP5b D_psi/wheel-load completion of his Eq. 4.3"; touched
thesis_notes.md, PROJECT.md, modules/stability_analysis.py docstrings.
(2) Two new CLAUDE.md standing rules: scientific grounding (Tier A/B/C
per technical decision, Tier A needs a literature anchor, Tier B needs
config-driven data-derived parameters, calibration tunables vs
method-defining named constants) and comment style (WHY not WHAT,
ASCII in .py files, no boilerplate/filler). (3) Read-only foundations
audit against both new rules across stability_analysis.py,
corner_analysis.py, csv_parser.py, recommendation.py, geo.py --
reported per-function tier/anchor/config-driven status plus comment-
style violations; user decided per item. (4) Applied those decisions:
new docstring citations (estimate_sideslip: Mitschke/Wallentowitz,
page TBD pending verify; estimate_slip_angles: Werner S2.1.1/Milliken);
removed the unverified "Suzuka convention" label everywhere (thesis_
notes.md struck through with a dated correction, PROJECT.md rewritten)
and replaced with Werner (2021) S2.2.3's actual sign convention;
corrected corner_analysis.py's speed-fallback header ("prominence
threshold" -> the valley-depth check actually implemented) and gave it
an explicit Tier B thesis_notes.md entry; moved 15 previously-hardcoded
Tier B calibration values into config (parameters.json,
channels.json x2 groups, recommendations.json) with zero behavioural
change; named the method-defining constants that deliberately stayed
in code (Butterworth order, CS blend exponents, source-balance
normaliser, feedback scale) with one-line justifications; ASCII-fixed
comments/docstrings in stability_analysis.py, corner_analysis.py,
csv_parser.py (user-facing warning strings left untouched by design).
test_stability.py run after every file in this pass (6 runs); output
byte-identical to the pre-session baseline throughout -- confirms the
whole pass was attribution/documentation/config-location only, no
logic changed.

WP-ALIGN session (2026-07-24, B1-B3, currently uncommitted, lands as
one combined commit with the two sessions above): chair-basis rebuild
of Module 5 (yaw moment stability). New modules/yaw_stability.py
(after the chair performance_analysis tooling, internal): rolling-mean
yaw acceleration replaces the 5 Hz Butterworth filter; the stability
regression is now s-anchored (lap_distance grid, Gaussian-weighted
local ridge, pools samples across laps at the same track position)
replacing the old time-anchored single-lap OLS. Three call-site
adaptations layered on top of the chair-identical estimator: raw-yaw-
rate path (forced -- chair's pre-smoothed-input filter list is out of
reference scope), moving/kerb/in-out-lap masking wiring (neutral
engineering), and structural in/out-lap exclusion (production, WP6-
independent of the UI lap_filter) -- the exclusion's rationale turned
out to be two separate legs on inspection (B2 diagnostic): the outlap
leg is forced in character (lap_distance channel is frozen at s~0 for
the whole outlap, a literal coordinate degeneracy, not a judgement
call), the inlap leg is the original cold-tyre/stationarity domain
improvement. Classification thresholds re-derived against the new
estimator's output distribution and moved into config/parameters.json
(new "classification" block, each entry with its own derived_from
note): CS thresholds unchanged (still valid, different signal); the
stability threshold moved from -500 to -50 Nm/deg (gap-selected
between -99.2 and -18.5 in the new worst-phase-per-corner
distribution), which now correctly flags all 4 laps of the one
physical corner (stable_id 8) that -500 only caught on 1 lap under the
old estimator. Verdict distribution on Dubai: 1 strong / 16 moderate /
34 normal (51 instances) -- proportionally close to the 2026-06-29
reference (0/23/49 over 72 instances, different lap composition) even
though the corner population, the estimator, and the threshold all
changed. A new CLAUDE.md/thesis_notes.md deviation taxonomy (FORCED
ADAPTATION / DOMAIN IMPROVEMENT / NEUTRAL ENGINEERING, plus a "vehicle
parameterization is not a deviation" clarification) now labels every
chair-comparison deviation, present and future. ui/views/outing_form.py's
_stability_colour also made config-derived (BAD boundary = the new
classification threshold, WARN boundary = a named 0.4 ratio inherited
from the original -200/-500 design) so detail-cell colours can't drift
out of sync with the verdict threshold again. test_stability.py run at
every step; Modules 1-4b confirmed byte-identical throughout B1-B3 --
only Module 5's numbers and the classification/colour thresholds
changed, exactly as intended.
Open threads: CS-threshold re-derivation deferred to the vehicle-model-
upgrade WP (bundled with the Fy yaw-term fix + WP5b a/b -- maps onto
WP5b(e), re-run diagnostic + re-derive thresholds after a Level 2+ Fy
split lands, since CS thresholds are tuned to the current static-
weight-split Fy signal); valid_fraction_stab replacement metric
deferred post-B3 (the field is non-discriminating under the new
s-anchored estimator -- see thesis_notes.md -- but no replacement
transparency metric has been designed yet, kerb_fraction covers the
kerb case only); _stability_colour is now config-derived (done this
session, listed here for visibility, not an open thread).

Vehicle-model-upgrade WP (2026-07-24, reduced scope, currently
uncommitted): Fy yaw-moment term landed in Module 4a (exact 2-DOF
planar force/moment balance, Fy_f = m*ay*front_fraction +
Iz*psidd/wheelbase, Fy_r = m*ay - Fy_f; raw np.gradient psidd,
deliberately not Module 5's filtered signal; Tier A, Milliken & Milliken
RCVD, p. TBD verify; chair-identical construction, no deviation).
WP5b(a)/(b) untangled: the original "roll stiffness needed for Fy" framing
was a conflation -- Fy needs none of it; axle-Fz-for-CS-normalization and
the per-tire/roll-stiffness-apportionment work (DOMAIN IMPROVEMENT vs the
chair's simpler independent-per-axle split) both moved into WP5b(b),
where they have an actual consumer. Housekeeping: OLD_REF hash fixed in
inspect_yaw_stability_b2.py, load_parameters() now lru_cache'd (neutral
engineering, not a chair-comparison item). Verification: Modules 1-3 and
5 byte-identical throughout (confirmed twice); CS thresholds re-checked
against the new distribution and kept unchanged (max flag-count shift
2/51 instances; n=51 resolution argument in thesis_notes.md); a genuine,
not-engineered-for CSr-tail repair observed (p10 -0.052 -> +0.082);
verdict distribution moved from 1 strong/16 moderate/34 normal (post-B3)
to 0/14/37, entirely via CS-branch movement -- Module 5's stability
branch contribution unchanged (byte-identical output).

WP5 (result persistence) + WP6 (lap-filter/pipeline cache) + wing_position
rider (2026-07-25, currently uncommitted): analysis_data column added to
Outing (models/outing.py) with an idempotent PRAGMA/ALTER migration in
init_db (models/base.py) -- verified additive and idempotent against the
real local DB (2 outings, backed up first, row data confirmed untouched).
Cache-hit rendering on reopen ("cached (laps X-Y) - re-run Analyse to
refresh"), write triggers on analysis completion (existing outings) and
via _save_outing (new outings / untouched cache-hit re-saves). Guard A:
verdicts are never persisted -- classification always runs live from
current config at render time, now through a single shared
_render_stability_summaries call site; verified a config threshold edit
changes a cached corner's verdict only after a fresh process (lru_cache
semantics), confirmed and config restored byte-identical afterward. Guard
B: ANALYSIS_SCHEMA_VERSION=1 (modules/stability_analysis.py) with a
bump-rule comment; version or csv_path mismatch (normalised via
os.path.normcase/normpath) treated as no cache, both verified. WP6:
_pipeline_cache stores {csv_path, corners, state, cs, stab} per
loaded_csv_path -- a lap-filter-only re-Analyse reuses cached Modules 1-5
output object-for-object (verified by identity, not just value equality),
no corner-detection or Modules-1-5 recompute; invalidated in
_on_csv_loaded's existing reset block; no cross-population between the
WP5 DB cache and the WP6 in-memory cache. Rider: wing_position is now a
QComboBox (P8/P9/P10), same pattern as arb_front_mount, in both setup and
setdown forms. Surprise, corrected in code comment: a legacy stored value
outside {P8,P9,P10} doesn't load as "nothing selected" -- QComboBox.
setCurrentText() no-ops on unmatched text, so it silently shows P8;
accepted by user decision (see UI polish note above). test_stability.py
byte-identical (structural -- this WP touches no code test_stability.py
imports).

# SetupTool — Work Plan (Phase 6)
Written: 2026-07-22. Point-by-point, no timeline. Execute work packages in order
unless noted otherwise. Each package is self-contained for a smaller model.

---

## SESSION PREAMBLE — paste this at the start of every new work session

You are working on SetupTool, a PyQt6/SQLAlchemy desktop app for Porsche 992 GT3R
race engineering (bachelor thesis, TUM / Proton Competition). Project root:
`C:\UNI\Bachelorarbeit\Setuptool_local`, Windows, VS Code terminal (PowerShell),
venv activated with `.\.venv\Scripts\Activate.ps1`.
Sample data: `C:\UNI\Bachelorarbeit\Data\Sample\Sample_Dubai.txt`
(7 laps: 0=outlap, 1-5=valid, 6=inlap; ~14-15 corners per lap detected).

WORKING RULES (mandatory):
1. One step at a time. Propose, wait for my confirmation, then give code.
2. Science before code: explain the method and its assumptions first.
3. Name every file explicitly. For edits give a unique find-anchor and the
   replacement. For new files, first give the PowerShell command
   `New-Item -Path <name> -ItemType File`, then the content.
4. Never guess at code you haven't seen. If a file's current content is needed
   and not in context, ASK me to paste it. Do not reconstruct from memory.
5. No PyQt6 imports in modules/ or core/. No business logic in ui/.
6. All tunable numbers go in config JSON files, never hardcoded.
7. Colour literals only from ui/style.py constants (OK, WARN, BAD, NEUTRAL,
   ACCENT, PANEL, PANEL_ALT, BORDER, TEXT, TEXT_MUTED, TEXT_DIM).
8. Short comment blocks. Real data only, no synthetic test data.
9. After each change: I run the app or `python test_stability.py` and paste
   results before we continue.
10. If anything in the work order is ambiguous or conflicts with the actual
    code, STOP and ask me instead of choosing silently.

Accuracy-level system: every physical quantity is Level 1 (config default),
2 (session measurement), 3 (logged sensor), or 4 (lookup table), upgradeable
independently.

Current state in one line: Modules 1-6 stability pipeline works end-to-end
(CS_ratio + dMz/dbeta per corner per phase, kerb exclusion via log_acc_z),
UI shows per-lap corner grid with severity colours; corner numbering is NOT
yet consistent across laps — that is the first work package.

---

## NEW DATA FILE — DIAGNOSTIC CHECKLIST (run whenever a new log file arrives)
- Second-track clustering validation: rerun stable-corner-id clustering,
  check the overlap-fraction gap-vs-overlap assumption holds (see WP3b).
- Channel scan for the four `value_source: "logged_data"` registry
  parameters (config/setup_parameters.json: tc_lat, tc_lon, abs_position,
  brake_bias) — these are driver-adjustable mid-session from the steering
  wheel / brake balance bar, so their truth is a logged channel, not a
  setup-sheet value, but the exact channel names are not yet known.
  Identify and record: TC LAT switch-position channel, TC LON
  switch-position channel, ABS switch-position channel, brake bias
  channel. Once found, add to config/channels.json and update the four
  registry entries' `notes` (channel name TBD -> confirmed).
- Multi-stint in/out/stop-lap classification (see WP7 IDEAS item 9) if the
  file spans more than one stint.

---

## WP1 — Cross-lap corner identity + detection robustness  [BLOCKER — do first]

### Problem
`modules/corner_analysis.py` segments corners per lap independently by steering
threshold. Borderline corners split/merge differently per lap → one lap has 14
corners, the next 15. Corner 7 in Lap 1 is then a different physical corner
than Corner 7 in Lap 2. This breaks the UI grid alignment, makes driver
feedback un-mappable, and blocks the recommendation engine.

### Method (decided — do not redesign)
Use the `lap_distance` channel (present in file, unit ft, already parsed and
quality-checked) as the spatial anchor. A physical corner sits at a fixed
distance along the lap regardless of lap time. Steps:

A) Bracket-merge improvement in `_bracket_corners_by_steering`:
   merge two adjacent brackets when the gap between them is shorter than a new
   config value `bracket_merge_gap_s` (default 0.6) — stabilises chicanes that
   sometimes read as one corner, sometimes two. Add the key to
   `config/channels.json` under `corner_detection`.

B) New post-processing function in `modules/corner_analysis.py`:
   `assign_stable_corner_ids(corners, channels)`:
   - For each corner, interpolate `lap_distance` at `apex_time`
     (np.interp on the lap_distance channel's time/data), convert ft→m
     (×0.3048), store as `apex_lap_distance_m` on the corner dict.
   - Collect all apex distances from all laps, sort ascending.
   - 1D-cluster: start a new cluster wherever the gap to the previous apex
     distance exceeds `corner_match_tolerance_m` (new config key in
     `corner_detection`, default 50).
   - Number clusters 1..N by ascending distance → write into each corner's
     existing `stable_corner_id` field (currently a None placeholder).
   - A lap missing a corner simply has no member in that cluster. A lap with
     an extra detection gets its own (possibly single-member) cluster — that is
     acceptable and visible.
   - Call this at the end of `analyse_corners` before returning.

C) UI grid alignment in `ui/views/outing_form.py`:
   - `_on_stability_done` / `_build_lap_row`: columns are now keyed by
     `stable_corner_id`, not per-lap corner_number. Determine the full set of
     stable ids across the analysed laps; every lap row has one slot per stable
     id in ascending order; a lap without that corner shows a dim placeholder
     cell (NEUTRAL colour, text "—", not clickable).
   - Cell label shows `C{stable_corner_id}`.
   - Module 6 (`summarise_corners` in stability_analysis.py) must pass
     `stable_corner_id` through into the summary dict (read it from the corner
     dict; it currently writes the placeholder None — change to
     `c.get("stable_corner_id")`).

### Acceptance criteria
- On Dubai data with all 5 valid laps analysed: every lap row shows the same
  columns; total unique stable corners ≈ 14-16; missing corners appear as
  gaps, not shifted numbering.
- Console print (temporary, remove after check): per lap, the list of
  (stable_corner_id, apex_lap_distance_m) to visually verify clustering.
- `python test_stability.py` still runs without errors.

### Stop-and-ask triggers
- If `lap_distance` quality is not "valid" on the sample file.
- If clustering produces > 20 or < 10 stable corners on Dubai (tolerance wrong).

---

## WP2 — Recommendation engine framework  [core deliverable]

### Goal
A rule-based skeleton that turns (analysis verdicts + driver feedback + current
setup) into ranked setup suggestions with an evidence trail. Framework first —
the user tunes weights and real rules later. Requires WP1 (stable corner ids).

### Architecture (decided)
- New file `modules/recommendation.py` (pure Python, no Qt).
- New file `config/recommendations.json` holding ALL rules and weights.
- New collapsible UI section "Recommendations" in outing_form.py, below the
  Stability Analysis section.

### config/recommendations.json schema
```json
{
  "settings": {
    "agreement_bonus": 1.5,
    "conflict_penalty": 0.5,
    "min_score_to_show": 0.5,
    "max_recommendations": 8
  },
  "rules": [
    {
      "id": "us_entry_arb",
      "name": "Entry understeer -> soften front ARB",
      "phases": ["entry_2_turnin"],
      "condition": {
        "verdict": "understeer",
        "min_severity": "moderate",
        "feedback_sign": "negative",
        "min_feedback_abs": 1
      },
      "suggestion": {
        "parameter": "front_arb",
        "direction": "soften",
        "weight": 1.0
      },
      "rationale": "Front axle saturating at turn-in; reducing front roll stiffness shifts lateral load transfer rearward."
    }
  ]
}
```
Seed with 4-6 placeholder rules covering: entry understeer, exit oversteer
(traction), apex understeer, unstable yaw on entry. The user replaces the
engineering content later; the pipe must work.

### modules/recommendation.py
`generate_recommendations(summaries, classify_fn, feedback_data, setup_data, config) -> list`
- Aggregate summaries across laps per stable_corner_id (median of phase
  medians) so each physical corner has one aggregate verdict per phase.
  Use the same classification thresholds as the UI — to avoid duplication,
  accept the classifier as a callable argument (`classify_fn`) supplied by the
  caller; in the UI thread that is `self._classify_corner`. NOTE: this keeps
  business logic out of ui/ in a later refactor step — flag it as tech debt in
  a comment; do not refactor now.
- Map feedback rows to stable_corner_id by index (feedback row 1 ↔ stable id 1).
- For each rule: find corners where condition matches; per match compute
  score = rule.weight × severity_factor (moderate=1, strong=2) ×
  (agreement_bonus if feedback sign matches verdict direction,
   conflict_penalty if it contradicts, 1.0 if feedback is 0/absent).
- Sum scores per (parameter, direction); output sorted list of dicts:
  {parameter, direction, score, corners: [stable ids], rules_fired: [ids],
   conflicts: [stable ids where driver disagreed], rationale}.

### UI section
- Collapsible "Recommendations" toggle, same pattern as Stability Analysis.
- Button "Generate" (enabled after an analysis exists). Runs synchronously
  (fast — no heavy math).
- Each recommendation: one row — parameter + direction badge (ACCENT), score,
  affected corners ("C3, C7, C11"), expandable rationale + conflict note in
  WARN colour if conflicts non-empty.

### Acceptance criteria
- Generate on Dubai + hand-entered dummy feedback produces a ranked,
  explainable list; changing a weight in the JSON changes the ranking without
  code edits; empty feedback still produces (data-only) recommendations.

---

## WP2b-1 — Setup-parameter registry [promoted from WP2 config note] [COMPLETE 2026-07-23]
`config/recommendations.json` rule `suggestion.parameter` values (`front_arb`,
`rear_arb`) are provisional labels, not yet tied to real setup-sheet fields.
New file `config/setup_parameters.json`: one entry per tunable, defining --
identity mapping to the real `car.json` field, its value space (range/units),
direction semantics (what "soften"/"stiffen"/etc. means for that parameter),
a one-sentence physical mechanism, and an optional phase-affinity hint (which
corner phases that parameter plausibly affects). Prerequisite for WP2b-2 --
rules cannot cite real parameters until the registry defining them exists.
Do after WP2 lands; not a blocker for the WP2 framework itself.

Populated from a team-knowledge session: 46-entry registry (37
mechanical / 3 electronic / 6 context), reference tables digitised
from manufacturer data (docs/car_data/) into config/car_data.json as
config lookups, consumer-scoped (only tables with a named registry/
WP5b/module consumer got digitised). Registry gained a `value_source`
field ("setup_sheet" vs "logged_data", independent of
recommendation_target) -- tc_lat/tc_lon/abs_position/brake_bias are
driver-adjusted mid-session from the wheel, so their truth is a
logged channel, not a setup-sheet value; maps_to is null for these
four pending a channel-name identification pass (see the new-data-
file diagnostic checklist above WP1). Only arb_front_mount and
differential_locking_torque_measured were added to car.json (car
dict) with matching outing_form.py UI rows. WP2b-2 remains --
rewrite recommendations.json rules against these registry keys.

## WP2b-2 — Rule engineering against the registry
Rewrite `config/recommendations.json` rules to reference `config/
setup_parameters.json` keys instead of provisional strings; replace the seed
weights/conditions with tuned, engineering-reviewed values; promote each
rule's `status` from `"seed"` to `"reviewed"` as it is validated. Depends on
WP2b-1.

Optional scope addition: per-driver feedback weighting (NOT "driver level")
— an optional field on the `Driver` model letting a driver's reported
feedback carry more or less weight in `source_balance` resolution.
`modules/recommendation.py` already isolates this behind a single helper,
`_resolve_source_balance(config, outing)`, which today just returns
`config["settings"]["source_balance"]`; extend it to resolve in order
feedback-weighting override (driver) > outing override > global default
rather than reading `settings["source_balance"]` inline anywhere else.

---

## WP3 — Driver feedback ↔ analysis comparison view

### Goal
Per stable corner: what the driver said vs what the data says, side by side.
Requires WP1. Feeds credibility into WP2 (already handled there via
agreement/conflict factors); this package is the *visual* comparison.

### Implementation
- In the corner details panel (`_build_corner_details`), add a "Driver" column
  to the per-phase table: the feedback value for that stable corner and phase
  (e1..x5 map to the five phase keys in order), colour-coded: negative =
  understeer side, positive = oversteer side, 0 = TEXT_DIM.
- Mapping: feedback row index == stable_corner_id (row 1 ↔ C1). If feedback
  has fewer rows than stable corners, show "—".
- Add an agreement marker per phase: ✓ (OK) when sign of feedback matches the
  data verdict direction, ✗ (WARN) when it contradicts, blank when feedback 0.
  Data verdict direction per phase: front-led issue = understeer (negative),
  rear-led = oversteer (positive); derive from which axle's CS median is
  proportionally worse against its own thresholds in that phase.

### Acceptance criteria
- Entering feedback, saving, reopening, analysing shows the Driver column
  aligned to the right corners; agreement markers behave correctly on at
  least three hand-checked corners.

---

- Level 3 sideslip: log_gps_course (velocity-vector direction, 10 Hz) vs
  chassis heading -> direct beta estimate, replaces kinematic integration.
- Speed validation: log_gps_speed vs ecu_speed agreement check; potential
  Level 3 upgrade for the speed signal feeding Modules 2-5.

## WP3b — Track template + corner naming map [promoted from ideas]
Two-layer corner identity: stable_corner_id = analysis key (all detected
load events, incl. high-speed kinks); display label = official corner
name from a per-track template (config/tracks.json: track name, official
corner count, corner names, GPS positions once available).
- Map view: plotted GPS trace (log_gps_lat/lon), detected stable corners
  marked; engineer assigns/confirms official labels once per track;
  merge/split correction lives on the same screen; template persists.
- Feedback binds to official labels, not raw indices. Kinks get optional
  own rows or fold into the neighbouring corner.
- Until WP3b lands, WP3's index-based feedback mapping is interim.
- Compound corners (bracket length > `compound_corner_min_length_m`, flagged
  `"compound_corner"` in warnings): candidate for a dual-apex phase structure,
  splitting at ay local minima -- for PHASE segmentation display only, never
  for bracket detection or stable_corner_id clustering. Deferred until a
  compound corner actually needs distinct entry/apex/exit phases shown
  separately; the single-bracket-per-compound-corner behaviour from WP1 stays
  the analysis ground truth either way.
Depends on: WP1 complete, GPS channels confirmed (Task 1 scan).

### Corner map v1 landed [2026-07-22]
The GPS outline + severity-coloured stable-corner markers described above
now exist as a static widget
(`OutingForm._build_corner_map` / `_update_corner_map_trace` /
`_update_corner_map_markers` in `ui/views/outing_form.py`; position
computed from parsed data alone via `modules.corner_analysis.
compute_stable_corner_positions`, projection shared via `modules/geo.py`).
Not yet built: click-to-highlight-row (click a marker -> scroll/highlight
its feedback table row, and vice versa), and everything else in this WP3b
section above (official-name template, engineer merge/split correction
on the same screen) -- v1 is read-only, detected-corner-id labels only.

Placement [2026-07-22, revised]: lives as its own labelled section
directly above Stability Analysis, NOT inside Driver Feedback. Rationale:
it is the legend for the ANALYSIS layer (stable_corner_id, matching the
stability grid and recommendation engine's corner chips) -- the Driver
Feedback section keeps its existing separate image-loader track map,
since drivers work in official corner nomenclature (the HUMAN layer).
Two-layer corner identity (see thesis_notes.md, "Analysis layer vs human
layer for corner identity") reflected directly in where each map lives,
not just in the data model.

## WP4 — Data lifecycle

### Scope
1. "Clear Data" button next to Analyse: resets parsed_data, loaded_csv_path,
   lap table, plots, stability section, recommendations section back to the
   no-file state. Confirm dialog not needed (nothing destructive persisted).
2. Loading a different CSV while one is loaded: must fully reset all derived
   state first (same reset routine), then load. Factor the reset into one
   method `_reset_data_state()` used by both paths.
3. Verify (manual test, no code expected): open existing outing → auto-load →
   change lap selection → re-Analyse → save → reopen. Fix what breaks.

### Acceptance criteria
- After Clear, the form looks exactly like a fresh outing's data section.
- Loading file B after file A shows only B's laps/corners; no stale widgets.

### Naming note [2026-07-22]
The "Exclude In/Out Laps" toggle actually filters on `is_valid_for_analysis`
(`_get_lap_filter_from_selector`, `_populate_lap_table`), which on Dubai
happens to coincide with in/out laps but is not the same thing in general --
a mid-session lap with a `warnings` entry (e.g. lap_distance/lap_time
disagreement) or one outside the 110%-of-fastest window is also excluded by
this toggle, silently, with a label that only mentions in/out. Decide:
rename the toggle/label to something accurate ("Exclude Invalid Laps"), or
split the concept into two independent filters (in/out vs validity).

### UI polish note [2026-07-23]
`outing_form.py`'s `wing_position` field renders as a `NoScrollSpinBox`
(numeric), but WP2b-1's registry defines it as an enum -- legal set
P8|P9|P10 only (config/car_data.json wing_position_table, GT3 R 2026
column). A spinbox permits illegal intermediate values. Low priority,
batch with the next UI pass (would follow the arb_front_mount QComboBox
pattern added in WP2b-1 TASK 3).

### wing_position legacy-value decision [2026-07-24]
wing_position combo defaults to P8; legacy pre-enum stored values
render as P8 (setCurrentText no-op, verified 2026-07-24) -- accepted
by user decision 2026-07-24, P8 is the standard position; old numeric
values are not treated as trusted setup data.

---

## WP5 — Result persistence

### Scope
- Add column `analysis_data = Column(String, nullable=True)` to models/outing.py.
  SQLite migration: simplest safe path — `ALTER TABLE outings ADD COLUMN
  analysis_data TEXT` executed once in init_db if the column is missing
  (inspect via PRAGMA table_info). Do NOT drop/recreate the table.
- On analysis completion, serialise {csv_path, lap_filter, summaries,
  generated_at} to JSON into the outing (only when editing an existing outing;
  for a new outing it is picked up by the normal save).
- On opening an outing with analysis_data whose csv_path matches: render the
  stability section from the cached summaries immediately, label it
  "cached — re-run Analyse to refresh". Any new Analyse overwrites the cache.

### Acceptance criteria
- Reopening an analysed outing shows results without recomputation; changing
  the CSV invalidates the cache.

---

## WP5b — Analysis accuracy upgrades (stability estimation depth)
Deepens the physics of Modules 1-5. Do after WP2/WP3 structure exists;
before heavy thesis validation runs, since these change the numbers.

a) ~~Level 2 Fy split: replace static weight split with dynamic axle load
   transfer from roll stiffness balance (setup_data has spring/ARB
   values -> roll stiffness fraction; lateral load transfer per axle
   from ay, track width, CoG height). Config: cog_height_m,
   track_width_f/r_m, roll stiffness inputs. Expect front CS values to
   shift; re-tune classify thresholds against new distribution.~~
   [SUPERSEDED 2026-07-24, vehicle-model-upgrade WP -- this conflated
   two separable things. DONE: Fy_f/Fy_r now use the exact 2-DOF
   planar force/moment balance (Fy_f = m*ay*front_fraction +
   Iz*psidd/wheelbase, Fy_r = m*ay - Fy_f, modules/stability_analysis.py
   estimate_lateral_forces) -- this needs NO roll stiffness, no track
   width, no CoG height at all; those were never actually required for
   Fy, only for a per-TIRE Fz split (see thesis_notes.md, dated entry).
   REMAINING (axle-level Fz for CS normalization: static + aero +
   longitudinal transfer, needs cog_height_m; and the per-tire Fz/
   roll-stiffness-apportionment work) moves into (b) below, since its
   only real consumer is the wheel-load work there -- building it
   speculatively now with no reader would violate the project's own
   scope-discipline principle (config/car_data.json's "what reads
   this, not is it true" rule).]
b) Level 4 Fy/Fz split: wheel loads from damper forces
   (log_dms_dam_fl/fr/rl/rr, 100 Hz, confirmed logged). Motion-ratio
   lookup needed (already digitised: config/car_data.json
   motion_ratio_vs_wheel_travel). Scope now also includes, folded in
   from (a) above (2026-07-24):
   - Axle-level Fz_f/Fz_r (static split + aero + longitudinal load
     transfer via ax/cog_height_m/wheelbase_m -- same construction as
     the chair's own fz_f_N/fz_r_N) as the direct CS-normalization
     consumer (fy_f_N/fz_f_N, fy_r_N/fz_r_N) -- Module 4b doesn't
     compute this yet. New config: cog_height_m (Level 1, vehicle
     description -- source not yet confirmed, check docs/car_data/ or
     ask for a measured/homologated figure before implementing).
     Aero term (air_density, lift_coeff/Cl*A, CoP-to-CoG distance) is
     a further open question -- no evidence yet that GT3R aero
     coefficients are digitised anywhere; if unavailable, launch
     axle-Fz without the aero term as a documented Level 1 limitation
     rather than block on it. [2026-07-24] Aero from damper-load v^2
     regression on straights (Level 3, sensor-funded extension)
     supersedes the no-aero-data limitation above as the preferred
     path once WP5b(b)'s damper-force channels are in use -- fit
     downforce-vs-speed-squared directly from logged damper load on a
     straight, rather than sourcing a separate homologation Cl*A
     figure. Setup-specific aero (ride-height coupling to downforce)
     and the motion-ratio + spring/damper load path both need
     verifying against the actual sensor installation before this is
     trusted quantitatively.
   - Per-tire Fz_fl/fr/rl/rr (needs track_width_front/rear_m, Level 1,
     source tbc): DOMAIN IMPROVEMENT proposed over the chair's own
     construction here -- the chair applies m*ay*h_cog/track_width
     independently per axle (no true front/rear apportionment by roll
     balance); SetupTool should instead derive a genuine roll-
     stiffness fraction from setup spring/ARB values (ARB already
     digitised: car_data.json arb.front/rear.positions x
     ratio_to_wheel; spring-value units need confirming first) and
     split total lateral transfer by that fraction -- Tier A, Milliken
     & Milliken RCVD roll-stiffness-from-wheel-rate chapter, page TBD
     verify. Justification: a GT3 team actively tunes this balance:
     the chair's simplification is not wrong for its own context, but
     a race-engineering tool should reflect what springs/ARB actually
     do to load distribution. Real damper-force data in this same
     sub-step gives a natural cross-check against the estimate.
   D_psi completion of Werner Eq. 4.3 (thesis_notes.md "Completing
   Werner Eq. 4.3") is gated by this sub-step's wheel loads and stays
   sequenced after it, same WP.
c) Level 3 sideslip: beta from log_gps_course (velocity vector) minus
   chassis heading estimate; replaces kinematic integration + washout.
   Validate against Module 2 output before switching default.
d) Speed validation: log_gps_speed vs ecu_speed agreement report;
   promote speed source if GPS proves cleaner.
e) After any of a-d lands: re-run the corner-distribution diagnostic
   and re-derive classification thresholds — they were tuned on
   Level 1 numbers and are not portable across accuracy levels.
f) Level 4 steering ratio lookup — replace the constant steering_ratio
   (Level 1, thesis limitation #3) with the digitised wheel-travel/
   stroke/ratio table (config/car_data.json: steering_ratio_table,
   source Steering.png). Consumer: prepare_vehicle_state's delta_f
   computation.

## WP6 — Performance (do opportunistically, after WP1)

Highest-value, lowest-risk item: **lap-filter changes must not recompute
Modules 1-5.** The lap_filter only affects `summarise_corners` (Module 6).
- Cache `(state, beta, slip, forces, cs, stab)` on the form keyed by
  loaded_csv_path after the first Analyse. Subsequent Analyse clicks with the
  same file run only `summarise_corners` + classification (fast, no thread
  needed — but keep the thread for uniformity).
- Invalidate the cache in `_reset_data_state()` (WP4).
Defer Module 5 stride-subsampling unless full analysis still feels too slow
after caching (measure first, ask the user).

---

## WP7 — Housekeeping

- Replace `generate_handover.py` with the walk-everything version (already
  supplied in chat on 2026-07-22; it writes a file inventory header). Verify
  the inventory lists csv_parser.py, all root scripts, all configs.
- `project.md` refresh after WP1-WP3 land.
- Keep test_stability.py; extend it with a stable-corner-id printout after WP1.

---

## IDEAS (optional — not work orders; pick up only on explicit request)

1. **Confidence score per verdict.** Combine kerb_fraction, valid_fraction_stab
   and n_samples into a 0-100% confidence shown small in each grid cell.
   Low-confidence cells render hatched/dimmed instead of asserting a verdict.
2. **Corner aggregate row.** Above the per-lap rows, one "All laps" row per
   stable corner (median across laps) — the engineer's first glance; per-lap
   rows become the drill-down.
3. **Outing-to-outing delta view.** Same weekend, two outings: show setup
   parameter diffs next to verdict changes per corner ("softened front ARB →
   C7 entry understeer moderate → normal"). This is the germ of a learning
   database of what changes fixed what.
4. **Session story line.** One auto-generated sentence per outing in the
   outings list ("balanced, mild entry understeer C3/C7, no instability").
5. **Analysis PDF export.** Reuse pdf_export.py patterns: one A4 page with the
   corner grid + top recommendations, printable for the driver briefing.
6. **Feedback auto-prefill hint.** After analysis, pre-highlight (not pre-fill)
   the feedback cells where data expects the driver to have felt something —
   speeds up the debrief conversation.
7. **Per-track parameter overrides.** Extend parameters.json with an optional
   per-track section (kerb threshold, corner tolerance) keyed by weekend.track.
8. **Track map from lap_distance + heading** (later, when GPS-equipped data
   arrives): dead-reckoning becomes unnecessary if gps lat/lon are logged —
   the file already contains log_gps_lat/lon at 10 Hz; a plotted outline with
   stable-corner markers is then mostly a drawing task.
9. **Multi-stint stint-aware in/out/stop-lap classification** (later, when
   multi-stint race data arrives): `_merge_trailing_pit_fragment` in
   `modules/csv_parser.py` only handles the session-trailing fragment (the
   final `lap_number` segment after the pit-in beacon, as on the Dubai
   sample). A multi-stint file has no such fragment mid-race — the stop lap
   runs line-to-line through the pit box as a single `lap_number`, so a
   stint's true in-lap/out-lap/stop-lap boundaries need classification via
   MID-lap `ecu_B_speedlimit_en` engagement (already whitelisted in
   `config/channels.json` for this reason) rather than a trailing-fragment
   merge. Deferred until such a file is available to validate against.

---

## ORDER OF EXECUTION

WP1 → WP2 → WP3 → WP4 → WP5 → WP6 → WP7. WP4 can be interleaved anytime.
Ideas only on explicit request. New data files from the user slot in as
validation passes for WP1 (rerun clustering on a second track before trusting
the tolerance default).