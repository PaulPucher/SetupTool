## STATUS (update at every work stop)
Current WP: baseline cleanup done, WP2 next. Last commit: <hash/date>.
Next commit: baseline cleanup (diagnostics/ folder, dead-code removal,
project.md/thesis_notes.md refresh) on top of WP1 close (cross-lap
corner identity + detection robustness -- dual-criterion detection,
compound-corner flag, lap selector toggle/clear, interval-overlap
clustering with seeded splitting). Open threads: track_check naming
verdict (T5-T6 region, user-confirmed two real corners = one load
event); second-track clustering validation when new data arrives
(also tests the overlap-fraction gap-vs-overlap assumption).

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

## WP2b-1 — Setup-parameter registry [promoted from WP2 config note]
`config/recommendations.json` rule `suggestion.parameter` values (`front_arb`,
`rear_arb`) are provisional labels, not yet tied to real setup-sheet fields.
New file `config/setup_parameters.json`: one entry per tunable, defining --
identity mapping to the real `car.json` field, its value space (range/units),
direction semantics (what "soften"/"stiffen"/etc. means for that parameter),
a one-sentence physical mechanism, and an optional phase-affinity hint (which
corner phases that parameter plausibly affects). Prerequisite for WP2b-2 --
rules cannot cite real parameters until the registry defining them exists.
Do after WP2 lands; not a blocker for the WP2 framework itself.

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

a) Level 2 Fy split: replace static weight split with dynamic axle load
   transfer from roll stiffness balance (setup_data has spring/ARB
   values -> roll stiffness fraction; lateral load transfer per axle
   from ay, track width, CoG height). Config: cog_height_m,
   track_width_f/r_m, roll stiffness inputs. Expect front CS values to
   shift; re-tune classify thresholds against new distribution.
b) Level 4 Fy split (superset of a): wheel loads from damper forces
   (log_dms_dam_fl/fr/rl/rr, 100 Hz, confirmed logged). Motion-ratio
   lookup needed (config placeholder until real table available).
c) Level 3 sideslip: beta from log_gps_course (velocity vector) minus
   chassis heading estimate; replaces kinematic integration + washout.
   Validate against Module 2 output before switching default.
d) Speed validation: log_gps_speed vs ecu_speed agreement report;
   promote speed source if GPS proves cleaner.
e) After any of a-d lands: re-run the corner-distribution diagnostic
   and re-derive classification thresholds — they were tuned on
   Level 1 numbers and are not portable across accuracy levels.

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