## STATUS - rewritten at every work stop, never appended

### NOW

WHERE THE PROJECT STANDS

- The production tool works and is unchanged except for one fix:
  modules/corner_analysis.py's entry_1_brake phase boundary, which
  previously spanned 85% of the dataset and now spans 5.5%,
  verified against brake pressure. Committed.
- The sideslip estimator arc is CLOSED. A linear-tyre Kalman
  observer was built and rejected (could not represent saturation).
  A nonlinear single-track EKF with a Dugoff tyre model replaced
  it. Its carried-forward configuration is PASS 1, with a frozen
  validation baseline (diagnostics/inspect_pass1_final_validation.py
  plus its manifest and notes entry). That baseline is the
  reference any future estimator work is compared against.
- The refit loop (passes 2-4) was STOPPED as non-converging on
  pre-registered criteria. The rear axle degenerated: each refit
  pushed the Dugoff onset boundary outward until zero of 24,183
  samples sat beyond it and the rear curve collapsed to
  pure-linear. Passes 2-4 are NOT carried forward.
- Pass 1's curve was fitted from KINEMATIC slip angles, which
  under-read. The refit loop existed to break that dependency and
  did not succeed. The carried-forward estimator therefore retains
  a known, documented circularity. This is a stated limitation of
  the method, not a resolved issue, and must appear as such in the
  write-up.
- A real regression test suite now exists (tests/, 95 tests -- 49 at
  the suite's own start, +10 Phase-3 Pacejka tests (WP-N3 package),
  +35 from the fresh-session auto-fit/NIS-gate wiring package below
  (20 gate unit tests, 7 wiring-validation tests, 8 new golden-file
  tests for the two auto modes) -- full suite now ~26 min, driven
  almost entirely by the wiring/golden tests' real fit-chain runs, not
  the original pipeline), alongside test_stability.py (unchanged,
  still a zero-assertion smoke test).
  REGRESSION not correctness -- pins current behaviour, does not
  validate it. 5 phases: golden-value pipeline/recommendation
  snapshots (cap=1, sideslip_source=kinematic -- see tests/conftest.py
  for why not "Best available"), phase-boundary invariants (all passed
  against current, post-fix behaviour -- no violation found), unit
  tests on pure functions (hand-derived slip-angle/Dugoff/EKF-
  Jacobian/Pacejka checks), config/schema integrity, NaN/empty-path
  coverage. Full record: thesis_notes.md "Regression test suite
  established". No production file touched building it; pytest
  installed into .venv only (not requirements.txt).
- UNSUPERVISED PACKAGE COMPLETED (2026-08-20, per-session-fittable
  self-checking sideslip, all 5 phases -- full record: thesis_notes.md
  "WP-N3 (per-session-fittable, self-checking sideslip): unsupervised
  package" and its Phase 1-5 entries). Entirely additive/read-only:
  no existing production file (modules/, ui/, core/) touched; the only
  config change is the new additive tyre_fit_auto namespace;
  sideslip_source unchanged at "kinematic"; no commit made. New files:
  modules/tyre_fit_auto.py (reusable one-shot per-session Dugoff/
  Pacejka fit + EKF validation chain, NOT wired to production),
  modules/tyre_model_pacejka.py, diagnostics/sideslip_ekf_pacejka.py,
  diagnostics/inspect_washout_cutoff_sweep.py, diagnostics/inspect_
  tyre_fit_auto_acceptance.py, diagnostics/inspect_tyre_variant_
  comparison.py, diagnostics/inspect_nis_tyre_mismatch_gate.py,
  tests/test_pacejka_model.py. Headline findings: (1) washout cutoffs
  0.03-0.005 Hz all beat the production 0.05 Hz default on mid-corner
  recovery without crossing the pre-registered drift-disqualification
  bound; (2) the automated fit chain reproduces the recorded pass-0/
  pass-1 procedure exactly on its scripted half, with a small (~0.37%)
  gap traced to an inconsistency in the archived, never-scripted R
  derivation itself, not to this package's automation; (3) Pacejka
  validates marginally better than Dugoff in aggregate but both axles
  (not just the pre-registered rear) show an extrapolated, poorly-
  identified peak on this session's data -- no winner declared; (4) a
  prototype NIS-based tyre-mismatch health score cleanly separates
  healthy from synthetically-mismatched runs, though both of its own
  pre-registered numeric predictions failed (recorded as failed, causes
  traced) and its proposed thresholds rest on five data points from one
  session only. Five decisions now waiting for the user (washout-cutoff
  decision -- BLOCKED, see below, not a live candidate list; fit-variant
  choice; NIS-gate thresholds; a small speed-classification cleanup
  found independently in two phases; and the standing pre-package
  carry-forward items below) -- see thesis_notes.md Phase 5 entry for
  the full list with reasoning.
- WASHOUT-CUTOFF DECISION BLOCKED (2026-08-2X, thesis_notes.md "Drift
  re-examined over time: single-checkpoint verdict superseded",
  under the Phase 1 washout entry) -- SUPERSEDES finding (1) two
  bullets above. Phase 1's drift-disqualification check evaluated
  only ONE causal checkpoint per corner exit; re-plotting drift as a
  function of time past that checkpoint shows 0.03/0.02/0.01 Hz all
  keep drifting for several more seconds (0.02 Hz: ~0.25 -> ~1.6 deg
  within 4s, crossing the 0.9 deg bound at ~1.5s) -- production 0.05
  Hz alone stays flat. The checkpoint was the single most favourable
  instant for every lower cutoff, not a representative one. DO NOT
  present 0.03/0.02 Hz as candidates without this caveat. The trade
  is corner-spacing dependent (a cutoff survivable on Dubai's spacing
  may not survive tighter corner sequences elsewhere) -- a single
  track-independent cutoff may not exist. Blocked pending EITHER
  multi-track data or a proper drift-vs-time acceptance criterion
  (not a checkpoint). Strengthens the auto-fit EKF (modules/tyre_
  fit_auto.py) as the PRIMARY sideslip-source candidate over a
  retuned kinematic cutoff, kinematic retained as fallback. General
  method note also recorded there: _highpass_filter (scipy filtfilt)
  is zero-phase/acausal -- any future drift or boundary claim about
  it must use causal checkpoints, not arbitrary segment lengths.
- FRESH-SESSION WORK PACKAGE COMPLETED (2026-08-2X: per-session tyre
  auto-fit + NIS gate wired into production -- full record: thesis_
  notes.md "4. Fresh-session work package: per-session tyre auto-fit +
  NIS gate wired into production" and its Phase 1-4 entries). Two new
  sideslip_source values now live: "ekf_auto_dugoff"/"ekf_auto_pacejka"
  run modules/tyre_fit_auto.py's fit chain on the SESSION'S OWN data
  (not a frozen prior curve), gated by new modules/nis_gate.py (ported
  from the WP-N3 prototype, thresholds still PROVISIONAL) -- gate
  fail/fit degenerate falls back to kinematic beta, recorded in the
  analysis payload and shown in the UI/PDF, never silently. Existing
  modes' behaviour perfectly preserved (regression suite green in
  kinematic/ekf_pass_1 throughout, confirmed by the same golden files
  unchanged); default stays "kinematic", nothing auto-enabled; no
  commit. ANALYSIS_SCHEMA_VERSION 5->6 (payload gained fit_manifest/
  gate_verdict/fallback_used/fallback_reason, null outside the two
  auto modes). New production-adjacent files: modules/nis_gate.py;
  new UI: sideslip_mode_combo (Data section, writes config directly --
  same restart-persistent pattern as ui/views/settings_view.py, chosen
  over accuracy_cap_combo's pattern because that one does NOT persist
  across restarts, verified before choosing) and estimator_status_label
  (which estimator actually produced beta, loud KINEMATIC-fallback
  wording); PDF export (core/weekend_pdf_export.py) carries the same
  status line -- FLAGGED: this file was not in Phase 3's own stated
  permitted-file list, edited anyway because sub-item (d) could not be
  satisfied otherwise; see thesis_notes.md for the full reasoning, this
  is worth a second look. TWO REAL BUGS found and fixed during this
  package, before shipping: fit_session/fit_session_pacejka were
  exposing the raw pre-fallback EKF beta instead of beta_with_fallback
  (would have violated the same "never feed a silently-diverged state
  downstream" rule the existing ekf_pass_1 path already followed);
  _format_estimator_status's self-attribute access broke the None-self
  reuse convention core/weekend_pdf_export.py's own PDF code depends on
  (would have crashed real PDF generation on any fallback render).
  Architecture note: the dispatch logic was extracted into modules.
  tyre_fit_auto.resolve_sideslip_beta (not left inline in the QThread)
  specifically so Phase 4's wiring validation could run without a Qt
  event loop -- StabilityAnalysisThread.run() is now a thin caller.
  Decisions now live for the user: banner/status-line wording (still
  functional-but-plain, not polished copy); NIS-gate threshold
  maturation (still five-data-points-one-session provisional, same
  caveat as WP-N3); whether to also authorise deleting core/weekend_
  pdf_export.py from Phase 3's permitted-file list retroactively (or
  flag the edit for reversion) given the file-list gap found above;
  the same standing washout-cutoff/fit-variant/speed-classification
  decisions carried over from the prior package, now joined by these.
- STEP 3 (LS_ratio) UNSUPERVISED PACKAGE COMPLETED (2026-08-30, all 5
  phases -- full record: thesis_notes.md "PLAN.md STEP 3 (LS_ratio):
  unsupervised package" Phase 1-4 entries; STEP 3 itself marked DONE
  below with the same summary). Entirely additive/read-only outside
  the three named production files: no existing estimator's numeric
  output changed (test_config_schema_integrity.py's own
  ANALYSIS_SCHEMA_VERSION assertion literal was also updated, FLAGGED
  as outside this phase's stated file list, same precedent as the
  fresh-session package's core/weekend_pdf_export.py edit above); the
  only config changes are the new additive longitudinal_stiffness
  namespace and the log_speed_fl/fr/rl/rr whitelist addition;
  sideslip_source unchanged at the user's own live value throughout
  (temporarily flipped to "kinematic" for the final regression-suite
  verification run only, restored exactly afterward, verified by
  diff); no commit made. New files: modules/longitudinal_forces.py,
  modules/longitudinal_stiffness.py, tests/test_longitudinal_
  stiffness.py, diagnostics/inspect_ls_cs_disambiguation.py. Headline
  finding: LS_ratio is fully implemented and wired end-to-end
  (pipeline, WP5/WP6 caches, corner-detail card, both trace dialogs)
  but produces all-NaN output on this car's real 50 Hz log under the
  chair's own unmodified defaults -- a proven structural config/
  sample-rate mismatch (max window 23 samples vs min_samples=25),
  not a data problem or implementation bug. Phase 4's disambiguation
  check was attempted per its own pre-registration and correctly
  returned zero clusterable instances as a result, not a shortcut.
  Decision now live for the user: the min_samples/regression_window_s
  re-derivation for 50 Hz (STEP 3's own close-out note below) -- until
  it lands, this is real, tested, dormant infrastructure, not a
  finished result.
- STEP 3 FOLLOW-UP: 50 HZ ADAPTATION AND REAL PHASE 4 COMPLETED
  (2026-08-30, same day, user decision -- full record: thesis_notes.md
  "PLAN.md STEP 3: 50 Hz min_samples adaptation", "...Phase 2, re-run
  against the adapted estimator", "...Phase 4, run for real"). The
  min_samples decision above landed: min_samples is now derived at
  runtime (max(min_samples_floor, half_window+1)) instead of
  transplanting the chair's literal 25, keeping the chair's own 0.45 s
  physical window unchanged -- config/parameters.json's min_samples
  key REMOVED, min_samples_floor=15 added (a real, load-bearing floor
  at 50 Hz: derived value would be 12, floor lifts it to 15). LS_ratio
  now produces real output (front n_valid=11299/24183, rear n_valid=
  18450/24183). tests/test_longitudinal_stiffness.py: one test
  RETIRED (test_real_config_at_50hz_never_reaches_min_samples, pinned
  now-obsolete pre-adaptation behaviour) and REPLACED (test_real_
  config_at_50hz_now_validates_with_rate_derived_min_samples); one
  test renamed and reworked to provoke the same structural guarantee
  via a low sample rate instead of a literal min_samples value (that
  lever no longer exists); every other test's helper call updated
  min_samples= to min_samples_floor=; all 8 pass. Phase 2's own
  update-rate prediction, re-checked against real output: FAILED both
  axles (measured 36.47% front/40.85% rear vs predicted ~87%/~61%
  +/-15pp) -- the raw-kappa-threshold basis didn't model the window-
  validity/stiffness-sign requirements, recorded as a miss. Phase 4
  run for real: 14/14 low-rear-CS instances now have a finite
  LS_ratio (0/14 before); split 7 traction-limited/7 cornering-
  limited -- the pre-registered "majority cornering-limited, 1-4
  traction-limited, C4/C14 candidates" prediction FAILED on every
  clause. Genuine new finding: C3 (stable_corner_id 3) is traction-
  limited on all 4 of its valid laps, a new candidate no prior
  evidence surfaced -- the combined-slip disambiguation's first real,
  working empirical result, even though the specific pre-registered
  guesses were wrong. Full regression suite CONFIRMED green (102
  passed, 1 xfailed, 0 failed, 0 errors -- identical count to the
  prior turn's own confirmation, run under a temporarily-flipped
  kinematic config restored exactly afterward, verified byte-
  identical to git HEAD via diff); git status clean; protected set
  empty; sideslip_source restored to the user's own live value; no
  commit.
- STEP 3 FOLLOW-UP 2: C3 VERIFIED CLEAN, LS PLAUSIBILITY GUARD BUILT,
  MASK WIDENING QUANTIFIED (2026-08-30, same day -- full record:
  thesis_notes.md "PLAN.md STEP 3 follow-up: C3 verified clean, LS
  plausibility guard implemented, mask widening quantified"). (1) C3,
  checked first, read-only: zero leaked-window contamination across
  all 4 laps' worst phases (exit_4/exit_5 both times) -- the headline
  finding needs no asterisk. (2) LS plausibility guard built
  (modules/longitudinal_stiffness.py, config additive): excludes a
  sample from the LS regression windows only when BOTH an implausible
  kappa (>12%, same gap-selected bound as the kerb investigation) AND
  a recent az kerb-like disturbance (axle-specific trailing window,
  150ms front/600ms rear, sized from the measured ringdown) hold --
  never on kappa alone, the load-bearing design constraint. A real
  bug caught and fixed during implementation: guarding only the post-
  filter window sums left the Butterworth-filtered signal itself
  still corrupted near an excluded sample (filtfilt smears an
  outlier's energy into neighbours regardless of later masking) --
  fixed by NaN-ing the excluded raw sample BEFORE filtering too. 8
  new unit tests, all pass. (3) Re-run with the guard active: C3
  unchanged (byte-identical LS_r values), the 14-instance 7/7 split
  unchanged, only one value anywhere shifted (4th decimal place).
  Whole-session guard footprint quantified: front axle 0 samples
  excluded, rear axle 2 -- narrow but real (up to 3827% relative
  slope change at the windows it does touch). The 5 originally-
  flagged contaminated windows individually show zero change under
  the guard -- traced and explained (most don't have BOTH implausible
  kappa AND az coincidence at the exact same sample), not a defect.
  (4) Mask widening quantified, NOT applied: current racing
  population n=24183; widening to 150ms costs -1.33%; widening to
  500-600ms (needed to properly cover rear ringdown) costs -7.7% to
  -9.4% of the ENTIRE population, since kerb_mask is one shared, non-
  axle-specific mask. Frozen-baseline statistics that would need
  re-deriving if applied: the WP-N2 pass-1 validation baseline, every
  percentile figure this session recorded against n=24183, the golden
  test files, and the classification thresholds. Full regression suite
  CONFIRMED green (110 passed, 1 xfailed, 0 failed, 0 errors --
  exactly the prior 102 plus the 8 new plausibility-guard tests, run
  under a temporarily-flipped kinematic config restored exactly
  afterward, verified byte-identical to git HEAD via diff); git status
  clean; protected set empty; sideslip_source restored to the user's
  own live value; no commit.
- DOCUMENTATION/COMMENT POLISH PASS DONE (2026-08-30, same day,
  unsupervised package, all 5 phases -- full record: thesis_notes.md
  "6. Documentation/comment polish pass"). Text-only sweep across
  every .py file in modules/, ui/, core/, diagnostics/, tests/, plus
  test_stability.py: comments, docstrings, and (ui/core only)
  user-facing strings rewritten or removed against an AI-slop removal
  list; zero numbers/thresholds/config values/control-flow/signatures
  changed. 41 non-ASCII characters normalized in ui/views/outing_
  form.py alone, dozens more repo-wide; the four already-decided
  protected texts ([UNCAL] marker, both calibration banners,
  _format_estimator_status templates) verified untouched. diagnostics/
  and tests/ (89 files) needed zero edits -- already at the target
  standard. Regression suite CONFIRMED green (110 passed, 1 xfailed,
  0 failed, 0 errors -- byte-identical to the recorded baseline), run
  under the same temporarily-flipped-kinematic-then-restored procedure
  as every prior full-suite run this project has done; git status
  clean; protected set empty; sideslip_source restored to the user's
  own live value; no commit. Items surfaced, not acted on (report
  only): a possible aero downforce sign-convention question in
  estimate_vertical_loads; the pre-existing core/pdf_export.py vs
  outing_form.py CORNER_LABELS hand-copy duplication (text
  reconciled, structural duplication remains); one dead local in
  diagnostics/inspect_abs_slip_channels.py; four live citation
  page-TBD placeholders needing the physical text to resolve.
- AERO DOWNFORCE SIGN CONVENTION VERIFIED, NOT A BUG (2026-08-30,
  same day -- full record: thesis_notes.md "Aero downforce sign
  convention verified computationally"). The polish pass's report-
  only flag on estimate_vertical_loads's fz_aero_total_N formula is
  RESOLVED: computational test against the real config/live code
  confirms cl < 0 correctly increases both axle loads (downforce),
  cl > 0 decreases them (lift) -- exactly as config's own pre-existing
  lift_coeff_note had already worked out from the formula alone, now
  confirmed rather than inferred. Added config/parameters.json's
  lift_coeff_sign_convention (one line, unmissable at the key) so a
  future real Cl entry can't get the sign backwards silently. No code
  changed, no commit.
- PDF LAYOUT REWORK DONE (2026-08-30, same day, propose-then-implement,
  approved with 3 clarifying decisions -- full record: thesis_notes.md
  "7. PDF layout rework: shared strip renderer"). Both the single-
  session setup/setdown sheet (core/pdf_export.py) and the weekend
  PDF's setup-sheet content (core/weekend_pdf_export.py) now share one
  renderer (build_session_strip), landscape A4, four strips/page on
  the weekend document (a chronological Setup/Setdown pair per outing,
  page break every 2 outings), monochrome, every value bordered,
  front-up 2x2 wheel orientation, two-layer corner boxes (readable
  core row + abbreviated dense damper/advanced row), marked-position
  schematics (custom reportlab Flowable, no new dependency) for
  wing_position/arb_front_mount only -- splitter_offset stays numeric,
  no diffuser field exists so none was invented. Canonical field set
  decided as the single-outing sheet's fuller 17-fields/corner set
  (the weekend PDF's narrower set, missing damper fields and the
  corner-weight grid, was drift, not a decision) -- the weekend PDF
  gains those fields plus Setdown strips it never printed before, and
  its old embedded per-outing "Setup Sheet" table was removed (not
  duplicated) now that the dedicated strips section covers it.
  arb_front_mount now prints for the first time (previously collected,
  never shown); the diff locking-torque table joined the PDF in the
  2026-08-30 ship-readiness cleanup package's Phase 4 (thesis_notes.md
  "9. Ship-readiness cleanup"), closing this open item. Two real bugs
  found and fixed before delivery,
  from visually inspecting sample PDFs (no golden PDF test exists,
  confirmed by grep): the team logo was sized proportional to full
  strip height, blowing up to page-width at full-page scale; and the
  car-parameter value column was oversized (55/45 split) for single-
  digit values. No new dependency in the shipped code; pymupdf was
  installed only transiently for this session's own visual QA and
  uninstalled immediately after. Regression suite CONFIRMED green
  (110 passed, 1 xfailed, byte-identical), same temporarily-flipped-
  kinematic-then-restored procedure as every prior run; sideslip_
  source restored to the user's own live value; git status clean
  otherwise; protected set empty; no commit. Sample PDFs generated
  against the real Dubai weekend, not synthetic data.
- SPLITTER/DIFFUSER MEASUREMENT POINTS DONE (2026-08-30, same day, all
  5 phases -- full record: thesis_notes.md "8. Splitter/diffuser
  measurement points"). New feature: 5 nullable floor-referenced mm
  check points each for splitter and diffuser, additive to and
  distinct from the existing splitter_offset SETTING (untouched).
  Phase 1: investigated setup_data/setdown_data's persistence pattern
  first -- no schema version exists on that JSON blob at all; the
  established migration pattern is purely additive (_load_inputs
  skips unknown keys). Followed the pre-existing differential_
  locking_torque_measured reshape precedent exactly: new core/
  setup_data_points.py (pure functions, no Qt) folds flat widget keys
  (splitter_point_1.._5 / diffuser_point_1.._5) to/from car[
  "splitter_points"]/car["diffuser_points"] arrays on save/load.
  Phase 2: new ui/views/measurement_points_widget.py, a custom-painted
  QWidget (rounded blade outline for splitter, rectangle for diffuser)
  with 5 real QLineEdit boxes positioned at physical coordinates,
  front-up orientation; wired into outing_form.py's existing car
  section with no change to the generic widget-collect/load dispatch.
  No source paper-sheet image exists for either shape (checked) --
  point layout is this session's own placement, flagged for the
  user's visual confirmation. Tooltips distinguish "setting, vs car"
  (splitter_offset) from "measured, vs floor" (the new points), per
  the work order's visibility requirement. Screenshotted headlessly
  against the real Dubai outing: old data loads with all 20 point
  boxes empty, setup/setdown each get independent widget instances.
  Phase 3: core/pdf_export.py's shared strip renderer gained
  MeasurementDiagram (outline + value boxes, same mechanism as the
  existing PositionSchematic) at both scales. Rendered two weekend-
  scale legibility candidates and picked by looking, as required: the
  outline+boxes design (kept) stayed legible and bounded; a values-
  only-row alternative (rejected, removed from the code entirely)
  overflowed its column and displaced the neighbouring Wing/ARB
  schematic. Phase 4: 9 new targeted tests (tests/test_setup_data_
  points.py, persistence round-trip with/without points, old-outing
  load, a hand-shortened-array defensive case, a cross-file position-
  count contract check) plus a new Qt-based diagnostics smoke test
  (diagnostics/smoke_test_measurement_points_widget.py, "form
  binding" -- drives the real widget/collect/save/reload path end to
  end) -- no golden regeneration. Testing policy for this package:
  no full-suite/golden runs during iteration (render-and-look plus
  targeted tests only), full suite exactly once at the end. Final run:
  119 passed (110 baseline + 9 new), 1 xfailed, byte-identical
  otherwise -- same temporarily-flipped-kinematic-then-restored
  procedure; sideslip_source restored to the user's own live value;
  protected set empty; no commit. 4 new files, all uncommitted.
- SHIP-READINESS CLEANUP DONE (2026-08-30, same day, unsupervised
  package, all 6 phases -- full record: thesis_notes.md "9. Ship-
  readiness cleanup"). Deleted the 29-file dead-diagnostics candidate
  list plus 2 of 3 orphaned manifest JSONs (the third, fit_dugoff_
  pass3_refit_manifest.json, kept -- live provenance pointer for the
  still-kept pass_3 config block, a deliberate deviation from the
  literal list); fixed every dangling reference the deletion left
  behind. Placeholder/TODO/TBD sweep: zero stale markers found, every
  live one already tracked to a real open item; one non-ASCII
  character fixed (config/parameters.json rad^-1). Second-round
  comment tightening pass (modules/, ui/, core/): 9 edits across 2
  files, the rest already at standard from the prior polish pass.
  Diff locking-torque table (differential_locking_torque_measured,
  5 points, collected since the arb_front_mount/diff-torque schema
  addition but never printed) now renders in core/pdf_export.py's
  shared strip renderer's car column at both scales -- render-and-
  look confirmed it fits cleanly, no degradation needed; this
  package's one authorised functional change. Ship-readiness audit
  (report only): all 4 sideslip_source modes complete a full analyze-
  and-export cycle with no traceback; all 14 dialog/window classes
  instantiate cleanly headless; several raw-exception-string and
  silent-failure UI risks catalogued for a future pass (thesis_notes.md
  Phase 5 entry has the full list with file:line pointers); config-key
  comment audit found essentially zero gaps, already closed by prior
  polish passes.
  OUT-OF-BAND FIX, user-authorised mid-package (thesis_notes.md
  "Out-of-scope emergency fix"): core/config_loader.py had a one-
  character SyntaxError that broke the entire application's startup
  -- an uncommitted working-tree corruption that appeared DURING this
  session (git HEAD was always clean; an earlier note here and to the
  user misattributed it to a commit, corrected same day in thesis_
  notes.md). Fixed on explicit user approval, the one deviation from
  this package's own "no functional changes outside Phase 4" rule.
  Full regression suite run exactly once at the end: 119 passed, 1
  xfailed -- the pre-registered baseline, confirmed. Protected set
  empty, no commit, sideslip_source restored to the user's own live
  value (ekf_auto_pacejka).
- SECOND DIAGNOSTICS SWEEP DONE (2026-08-30, same day, full-inventory
  reclassification -- full record: thesis_notes.md "10. Second
  diagnostics sweep: full-inventory classification"). All 54 files in
  diagnostics/ reclassified (not just the prior candidate list): 23
  more scripts deleted (each finding already traced to a real thesis_
  notes.md record), plus 4 gitignored stale artifacts and the orphaned
  diagnostics/plots/ directory (198 PNGs from 5 now-deleted plot
  scripts). One near-miss caught before it broke anything: inspect_
  wheel_speed_sources.py was initially deleted, then restored after a
  follow-up internal-import grep found inspect_washout_cutoff_sweep.py
  depends on it -- reclassified [dependency] rather than [keep-
  referenced]. New CLAUDE.md standing rule added (diagnostics/
  disposal rule: Referenced/Reproduces/Dependency categories, dispose
  at commit time, not in a later sweep); diagnostics/README.md
  rewritten to state every survivor's specific keep-reason (previously
  covered only 2 of the then-many files, silently stale since 2026-07).
  26 scripts + 4 non-py artifacts + README.md survive. Targeted pytest
  run on the 5 tests touching diagnostics/ imports/citations: 65
  passed, 1 xfailed, 2 errors traced to a conftest.py fixture guard
  unrelated to the sweep (confirmed via a temporary kinematic flip,
  both pass, then restored); sideslip_source restored to the user's
  own live value, byte-identical to the pre-sweep state via git diff.
  No full suite run (not required by this package).
- CHANNEL-REQUIREMENTS GENERATOR ADDED (2026-08-30, same day --
  diagnostics/generate_channel_requirements.py, [keep-reproduces] per
  diagnostics/README.md). Regenerates two committed deliverables,
  docs/channel_requirements.md (the telemetry-export checklist for a
  new event, with per-channel WHY) and docs/channel_list.txt (the same
  channels, bare one-per-line, for a literal tick-off), from config/
  channels.json plus real read-site greps of modules/core/ui and the
  repo-root channel_list.txt (the real Dubai channel inventory). Both
  outputs come from one run, so they can never disagree with each
  other; re-run whenever channels.json or a channel-consuming module
  changes. A reusable generator, kept by design, not a one-off
  investigation.
- GT3 PAUL RICARD EXPORT PARSED (2026-08-31 -- full record: thesis_
  notes.md "11. GT3 Paul Ricard export: diagnosis and fix"). GT3_
  PRC_MLA.txt (534 MB, team telemetry, found untracked at repo root)
  diagnosed and fixed: modules/csv_parser.py now branches on header
  shape to also parse Pi Toolbox's WIDE-TABLE export layout (Paul
  Ricard) alongside the existing NARROW layout (Dubai, byte-identical/
  unaffected). A second, more dangerous problem found in the same
  diagnosis: lap_distance was unconditionally assumed to be in feet in
  two places (modules/stability_analysis.py, modules/corner_
  analysis.py) -- fixed with a new unit-aware _normalize_lap_distance_
  to_metres, sourced from each channel's own captured unit_raw string;
  both call sites lost their independent hardcoded *0.3048. Latin-1
  encoding confirmed and documented (both real exports -- Dubai and
  Paul Ricard -- are single-byte Pi Toolbox text). 14 new tests (tests/
  test_csv_parser_formats.py, synthetic fixtures only; the real file
  deliberately excluded from any test/validation per the user's own
  instruction). Full regression suite: 133 passed, 1 xfailed, 0
  failed, 0 errors (35m32s) -- the prior 119 plus these 14, zero
  regressions. GT3_PRC_MLA.txt remains gitignored (/*.txt pattern) and
  untracked, parser-test-only -- never an analysis or validation
  target, per the user's own explicit instruction (not this project's
  native 50 Hz rate, a partial session).
- GT3 PAUL RICARD SAMPLE-RATE GUARD ADDED, CENSUS CONFIRMS NO FASTER
  CHANNEL (2026-08-31, same package). config/parameters.json gained
  stability_estimation.expected_sample_rate_hz=50 (provenance comment
  naming every 50-Hz-calibrated consumer); modules/csv_parser.py now
  measures and exposes measured_sample_rate_hz; prepare_vehicle_state
  raises immediately, naming both rates, on any mismatch (Paul
  Ricard's real 20 Hz vs this project's 50 Hz calibration). KNOWN
  RESIDUAL GAP, not fixed: corner detection runs inside parse_csv,
  before the guard's own check point, so a non-50-Hz file's corner
  markers could still render with mis-scaled smoothing even though the
  full Analyse pipeline is correctly refused -- a deliberate scope
  decision (parse_csv measures/exposes, prepare_vehicle_state
  refuses), not an oversight. Follow-up per-channel census (diagnostics/
  inspect_prc_sample_rates.py, read-only, streamed three file windows
  rather than loading the 534 MB file) RE-RUN this turn to verify
  before writing this status, rather than trusting the prior session's
  summary blindly: the row-grid rate is consistently 20.000 Hz across
  start/middle/end windows (no cross-window disagreement); of 267
  channel-window combinations checked, only one channel differs from
  the grid rate by more than 1 Hz -- Math_Wheel_LockRL, a sparse
  wheel-lock EVENT marker (n=3-77 non-missing samples in a ~2000-row
  window), not a continuous sensor; no whitelisted analysis channel or
  damper/wheel-speed-family channel runs faster than the 20 Hz grid
  anywhere in the file. DECISION: keep the hard rate guard as
  implemented, no adaptive-rate work; a 50 Hz re-export has been
  requested from the team. Census script deleted this turn (diagnostics/
  inspect_prc_sample_rates.py) now that its one decision has landed --
  ITS FINDING IS RECORDED HERE ONLY, NOT in thesis_notes.md (this
  status-rewrite turn was explicitly text-only/PLAN.md-only and
  instructed not to touch thesis_notes.md; flagged for a future thesis_
  notes.md entry if this census result is wanted there for the write-up).
- STEP 2 (chair-comparable result plots) DONE (2026-08-31 -- full
  record: thesis_notes.md "12. PLAN.md STEP 2: chair-comparable result
  plots, kinematic vs ekf_pass_1"). diagnostics/inspect_step2_chair_
  plots.py ([keep-reproduces] in diagnostics/README.md, thesis-figure
  source), 28 PNGs (14 stable corners x kinematic/ekf_pass_1 sources)
  in diagnostics/plots_step2/ (gitignored). HEADLINE FINDING: the rear
  CS extremes that opened the estimator arc are largely BETA
  ARTIFACTS -- C9 rear worst-phase CS improves from -362508 to -74581
  N/rad and C6 rear from -311382 to -13064 N/rad under ekf_pass_1 vs
  kinematic, both with visibly cleaner near-monotonic tyre curves under
  ekf_pass_1. C4's front saturation is NOT a beta artifact -- CS stays
  large and negative under both sources (-98372 kinematic / -95027
  ekf_pass_1) with the same peak-and-fold shape at a similar slip angle
  both times, read as genuine saturation, not estimator noise. No
  production/config file changed; sideslip_source never read from or
  written to config throughout.
  PROJECT DIRECTION (decided 2026-08-31): estimator/method work is
  FROZEN for now. STEP 4 (decision-matrix cleanup) and the STEP 3/4
  threshold-re-derivation prerequisites below remain open but are NOT
  the next work. Next priorities, in order: (1) commit the large
  uncommitted working tree (many sessions' worth -- see git status),
  (2) reliability passes (the ship-readiness audit's catalogued raw-
  exception-string/silent-failure UI risks, thesis_notes.md "9. Ship-
  readiness cleanup" Phase 5, still uncatalogued-into-fixes), (3)
  thesis writing. STEP 4 and further method work resume only when the
  user reopens it -- the prerequisites list below is preserved exactly
  as scoped, not abandoned.

WHAT CHANGED IN UNDERSTANDING (2026-08-20) -- corrections that
must not be lost

- CS_ratio IS the utilisation measure. The chair's own docstring
  defines it as the axle's SATURATION LEVEL: 1 = linear region,
  0 = at the peak, below 0 = beyond the peak. It needs NO friction
  coefficient and NO vertical load -- saturation is read off the
  shape of the measured curve. An earlier framing in this session
  that utilisation requires sqrt(Fx^2+Fy^2)/(mu*Fz), and that a
  downforce figure was therefore a blocker, is WITHDRAWN. Cl stays
  a Level-1 placeholder like every other unsourced quantity, to be
  upgraded when damper-derived load arrives.
- The chair's PRIMARY cornering-stiffness method is MODEL-FREE:
  sliding-window least squares over measured slip angle and
  measured lateral force, no tyre model at all. Its Pacejka fit
  appears only inside an evaluation plot as a reference overlay.
- This project's Dugoff curve occupies a THIRD role, distinct from
  both: it is the internal model of the EKF, the thing that lets
  the filter produce beta. Pacejka and Dugoff are not competing
  choices for the same slot.
- The chair receives lateral velocity (or beta) as an INPUT
  CHANNEL and computes beta as arctan2(vy, vx). They never
  estimate it. The GT3R has no such sensor. This is precisely the
  documented adaptation in the method lineage: adopt the chair's
  model-free CS estimation as-is, adapt only where the sensor
  situation forces it.
- Pure-lateral CS_ratio is NOT blind to combined slip. Module 4b
  measures a slope from data, and a tyre spending grip
  longitudinally genuinely has a flatter lateral curve. What it
  cannot do is say WHY the slope dropped. That is what a
  longitudinal ratio adds: disambiguation, not new sensitivity.

PLAN, in order

STEP 1 -- Wire the pass-1 EKF beta into the pipeline so results
are visible in the app.
  1a. DONE (2026-08-20). TIME IT FIRST, before any wiring. Result:
      pass_1 EKF alone 4.66s (0.114 ms/sample, n=40800); production
      full-outing total 123.27s; projected total with EKF substituted
      127.93s (+3.8%). Gate NOT TRIGGERED -- user confirmed, proceed.
      Unplanned finding: estimate_cornering_stiffness (106.22s, 86% of
      total), not the EKF, is the pipeline's actual cost driver; not
      acted on, out of this sub-step's scope. Full record: thesis_
      notes.md "WP-N2 Step 1a: pass-1 EKF wall-clock timing".
  1b. DONE (2026-08-20), SHIPPED DEFAULTED OFF. Wired behind config/
      parameters.json stability_estimation.sideslip_source
      ("kinematic" default / "ekf_pass_1"), ANALYSIS_SCHEMA_VERSION
      4->5 (both cache layers carry sideslip_source in their identity
      check), verified against the frozen pass-1 baseline under both
      cap=None and cap=1 -- all residual drift from the frozen
      manifest traced to two already-approved prior changes (WP-B
      steering-ratio L4 lookup; the entry_1_brake phase-boundary fix
      postdating the manifest's freeze commit), not to the wiring.
      Switch left at "kinematic" after verification -- turning it on
      is a separate decision. Full record: thesis_notes.md "WP-N2 Step
      1b: wiring proposal, approval, and implementation".
  1c. MECHANISM IMPLEMENTED (2026-08-20), PLACEHOLDER WORDING. A
      config-driven flag (classification.thresholds_calibrated_for_
      sideslip_source, "kinematic") gates a persistent banner in both
      the stability and recommendations panels plus a per-verdict "
      [UNCAL]" marker (_classify_corner, inherited by the PDF export
      unmodified) whenever the active sideslip_source doesn't match
      it. Wording is placeholder pending visual review -- CRITICAL
      SPLIT still to state once reviewed: with new beta, the TRACES
      (CS_ratio through a corner, the tyre curve, stability) are
      immediately meaningful, because they do not depend on
      thresholds. The VERDICTS and RECOMMENDATIONS are NOT, because
      those thresholds were derived against the kinematic CS_ratio
      distribution. Until thresholds are re-derived, read the traces
      and ignore the verdict colours.

STEP 2 -- DONE (2026-08-31, standalone diagnostic, single session --
  full record: thesis_notes.md "12. PLAN.md STEP 2: chair-comparable
  result plots, kinematic vs ekf_pass_1"; summary: NOW above).
  diagnostics/inspect_step2_chair_plots.py, 28 PNGs in diagnostics/
  plots_step2/ (gitignored). ANSWERED the purpose question below:
  the rear CS extremes (C6, C9) that opened the estimator arc are
  largely beta artifacts (both improve sharply under ekf_pass_1 vs
  kinematic); C4's front saturation is NOT a beta artifact (large and
  negative under both beta sources, same fold shape both times) --
  read as genuine saturation. No production/config file changed.
  Verify the result plots against the chair's output.
  Axis decisions, fixed: tyre-curve slip angle in DEGREES
  (readability), lateral force in N, cornering stiffness reported
  in N/rad (standard unit), all labels explicit.
  TRAP: with slip angle in degrees the visual slope is N/deg, not
  N/rad. A tangent drawn from an N/rad value on a degrees axis is
  wrong by 180/pi ~ 57.3. Convert: slope_per_deg = CS_N_per_rad *
  pi/180. The chair plots radians; state how numerical
  comparability is preserved despite the axis difference.
  Chair's plot structure to match, for comparability: velocity vs
  distance; instantaneous CS in N/rad vs distance with both the
  online estimate and a reference-model derivative; track map with
  the current corner and estimation window highlighted; and the
  tyre curve (slip angle vs lateral force) with the lap scatter,
  the current corner, the estimation window, and a tangent whose
  slope is the local CS estimate.
  The purpose of this step is to find out whether the strange
  CS values that started this whole arc were caused by the beta
  error or by something else.

STEP 3 -- DONE (2026-08-30, unsupervised package, all 5 phases, plus a
2026-08-30 follow-up turn that unblocked and completed Phase 4 for
real -- full record: thesis_notes.md "PLAN.md STEP 3 (LS_ratio):
unsupervised package" Phase 1-4 entries, "PLAN.md STEP 3: 50 Hz
min_samples adaptation", "PLAN.md STEP 3 Phase 2, re-run against the
adapted estimator", "PLAN.md STEP 3 Phase 4, run for real"). LS_ratio
implemented and wired into production DISPLAY ONLY (no verdict/
classifier reads it) -- modules/longitudinal_forces.py (axle Fx + slip
ratio kappa, externally validated against diagnostics/inspect_
combined_slip_premise.py's already-recorded figures, exact digit
match) and modules/longitudinal_stiffness.py (the chair's windowed
dFx/dkappa estimator).
RESOLVED FINDING: the chair's literal min_samples=25 with a 0.45 s
window was structurally incompatible with this session's 50 Hz
Cosworth log (proven: max window 23 samples). DECIDED AND ADAPTED
(user decision, follow-up turn): min_samples is now derived at
runtime as max(min_samples_floor, half_window+1), keeping the chair's
own PHYSICAL window (0.45 s, unchanged) and deriving the count from
the actual log rate instead of transplanting it -- FORCED ADAPTATION
under the deviation taxonomy, config/parameters.json's min_samples_
floor=15 replaces the removed min_samples=25 key. LS_ratio now
produces real, non-NaN output (front n_valid=11299/24183, rear
n_valid=18450/24183, base population). The Phase 2 pre-registered
update-rate prediction (~61% rear/~87% front) FAILED against the now-
real output (measured 40.85% rear/36.47% front) -- the raw-kappa-
threshold basis for that prediction did not model the window-validity
and stiffness-sign requirements, recorded honestly as a miss, not
adjusted after the fact.
PHASE 4 RUN FOR REAL: 14 low-rear-CS-p25 corner instances, all 14 now
with a finite LS_ratio (0/14 before the adaptation). Split 7
traction-limited / 7 cornering-limited -- the pre-registered
"majority cornering-limited, 1-4 traction-limited, C4/C14 as likely
candidates" prediction FAILED on every clause (even split, not a
majority; C4 absent from the population entirely; C14 landed
cornering-limited, the opposite of predicted). GENUINE NEW FINDING:
stable_corner_id 3 (C3) is traction-limited on ALL FOUR of its valid
laps, a new candidate the prior EKF-context attribution history never
surfaced -- the first concrete, repeatable empirical example of the
combined-slip disambiguation working as intended, even though the
pre-registered numeric guesses were wrong. The combined-slip premise
itself (some low-CS corners are traction-limited, some are not) is
upheld on this one session; the specific predictions about WHICH ones
and how many were not.

STEP 4 -- Decision-matrix cleanup (the recommendation rules).
  Prerequisites that must land first, and why:
   - threshold re-derivation against the new CS_ratio
     distribution, per the standing rule that thresholds encode
     the current estimator's distribution and are re-derived after
     any estimator change, never carried over;
   - the CS_ratio cross-lap aggregation problem (see PARKED): at
     the aggregate level CS_ratio is pinned at ~1.000 for every
     corner, because the metric is clipped at 1.0 and
     median-across-four-laps discards single-lap excursions. A
     measured example: corner 6 lap 1 collapsed to 0.219 and was
     washed out entirely. Re-deriving thresholds against a metric
     that does not vary would be pointless, so this must be
     settled inside the threshold step.
   - 15 of 39 rules key on entry_1_brake, whose statistics changed
     substantially with the phase fix; and 22 of 56 corner
     instances (39.3%) now have a zero-length braking phase, where
     a corner with no computable signal is currently classified
     identically to a corner that is genuinely fine (safe, no
     crash, but silent).
   - NEW (STEP 3 close-out): whether LS_ratio enters the
     recommendation rules at all is UNDECIDED -- STEP 3 shipped it
     DISPLAY ONLY, deliberately, per its own work order. If/when it
     does, threshold re-derivation above must cover BOTH ratios
     together (CS_ratio and LS_ratio), not CS_ratio alone as
     originally scoped, since any rule combining them needs
     thresholds derived from the same estimator run. The min_samples/
     50 Hz gate that used to block this is LIFTED (2026-08-30 follow-
     up turn, min_samples now rate-derived) -- LS_ratio has a real
     distribution now (front n_valid=11299/24183, rear n_valid=
     18450/24183, base population; see thesis_notes.md "PLAN.md STEP
     3: 50 Hz min_samples adaptation"). Nothing else about this
     prerequisite has changed: still gated on the same threshold-
     re-derivation step as CS_ratio, still undecided whether LS_ratio
     should be a recommendation input at all.

STANDING WARNINGS -- carry these into every future session

- NEVER state a config value from memory or from an instruction.
  Read it from config and quote it. Two errors this week came from
  exactly that.
- test_stability.py has ZERO assertions. It is a smoke test that
  confirms the pipeline does not crash. It passed cleanly through
  two separate broken phase-boundary fixes. Phase-boundary and
  numerical correctness have no automated coverage.
- For boundary and numerical work, verify against an EXTERNAL
  physical reference channel. Distributional plausibility is not
  verification: both broken fixes produced healthy-looking
  durations and population shares, and only the brake-pressure
  cross-check caught them.
- Findings go into thesis_notes.md in the SAME turn they are
  produced. Chat reports are not the record.
- Protected set, never committed: docs/literature/, docs/car_data/,
  config/car_data.json, HANDOVER.md, docs/study/. Verify with
  git ls-files returning empty.

### BACKLOG (ordered)
A - Numbers correct: nonlinear single-track Kalman filter with a
    data-identified tyre curve (linear observer rejected for
    production on saturation-detection failure, see NOW above --
    this replaces the prior "observer tuning; comparison report +
    decision" item, does not close it); vehicle-parameter provenance
    (wheelbase, Iz, cog height, track widths, Cl reviewer
    placeholders) + team figures, unrelated to the estimator
    question. PARTIAL OUTCOME (see NOW, carry-forward decision): the
    data-identified-curve refit sub-attempt (passes 2-4) is REJECTED
    as non-converging; pass 1's configuration (kinematic-sourced
    curve, calibrated noise model) is carried forward instead, with
    the kinematic circularity explicitly unresolved. Done when:
    sideslip has a defensible source and verdicts are recomputed on
    it -- still open, gated on R re-derivation against pass 1 (NOW)
    and the threshold re-derivation (PARKED).
C - Decision-matrix depth: elicitation question set (own file, to
    be created in a later session, sorted by who answers - user or
    engineer); matrix expansion incl. the beyond-peak gap; cost
    function with elicited weights; feasibility/conflict logic.
    Gated on elicitation answers.
B - Forces: damper-derived Level-4 wheel loads; roll-stiffness
    apportionment; Fy split upgrade -> CS threshold re-derivation.
    Gated on damper data arriving (no date). Science prep done
    externally (Segers Ch. 9/10 anchors, method fixed; open items:
    roll center heights, gauge calibration check) -- still gated on
    50 Hz damper data arriving.
D - Output artifacts: weekend PDF expansion; per-corner CS overlay
    and g-g plots.
E - Cleanup (after track A closes): diagnostics inventory + README,
    archive dead one-offs; HANDOVER regeneration incl. diagnostics;
    protected-set audit and push readiness.
F - (optional thesis figure, not a work item) Rear Dugoff curve
    plotted at pass 0, 2, 3 and 4 on shared axes, showing the
    pass-4 rear curve as a straight line. A single figure
    communicating the refit loop's identifiability failure -- the
    onset boundary moving outward until it reaches 88.4 deg and
    coverage falls to exactly zero of 24,183 samples. Not a result
    plot; the passes are not carried forward. Purely a figure for
    the write-up.

### PARKED (decided, not forgotten)
Beyond-peak verdict tier - shelved, reopens only with a validated
sideslip estimate. Kerb-gap interpolation - optional cleanup.
Compound-corner curvature detection - not worth destabilizing the
realization. Worst-phase sentinel NaN quirk and valid_fraction_stab
replacement metric - cosmetic, left alone. k=1.01211 application -
gated on second-track data. New-data-file checklist - runs when a
log arrives.
Combined-slip tyre model: pure-lateral Dugoff cannot represent
rear exit-traction or front entry-braking limitation, producing
false negatives in both. Rajamani Ch. 13.10's own formulation is
already combined-slip, so no new anchor is needed; log_speed_* is
the designated wheel-speed source (WP-S1, not yet whitelisted).
Gated on the EKF arc closing. Full reasoning and evidence:
thesis_notes.md WP-N2 combined-slip subsection.
Classification-threshold re-derivation against the EKF's CS_ratio
distribution: deliberately deferred until the estimator is
finalised (refit passes complete, combined-slip comparison done),
because re-derivation is the step that commits to the EKF as the
production sideslip source. Pass_1 flagged counts are NOT
comparable to production verdicts in the meantime. Reasoning:
thesis_notes.md "Threshold re-derivation deliberately deferred".
entry_1_brake bounded-backward-search hardening: the 2026-08-20 fix
(off_throttle[0] -> [-1], modules/corner_analysis.py) corrects
which off-throttle sample is picked but the backward search itself
is still unbounded -- for a corner taken nearly flat-out with only
a brief lift, brake_start_t could still reach back further than
intended if no clear off-throttle sample sits close to turn-in.
Bounding the lookback (e.g. not searching past the previous
corner's own bracket) is a reasonable defensive measure but is a
second change with its own behaviour; bundling it with the index
fix would have made verification ambiguous, so it was deliberately
left out of that fix. Reopens if a spot-check or production use
surfaces an implausibly long entry_1_brake window post-fix.
CS_ratio aggregation-sensitivity: aggregate_by_corner's median-of-
medians across four laps washes out a single lap's real signal on a
ceiling-pinned metric (found 2026-08-20 comparing entry_1_brake
before/after its phase-boundary fix -- CS_ratio stayed pinned at
~1.000 at the aggregate level for every corner checked despite a
dramatic single-lap collapse at one). Most of the 15 braking-matrix
rules key on CS_ratio, so they carry near-zero aggregate sensitivity
on this dataset independent of any phase-boundary correctness.
Candidate directions (percentile instead of median, worst-lap,
reporting lap-to-lap spread) not evaluated. Gated on the same
decision point as the deferred classification-threshold
re-derivation above -- changing the aggregation is itself a
production behaviour change affecting verdicts. Reasoning:
thesis_notes.md "Production impact of the fix, and a structural
finding about CS_ratio aggregation".

### PROCESS RULES
- One step at a time; proposal and implementation never combined;
  every turn ends at a stop point; user runs git.
- Every WP names its artifact up front (changed number, UI element,
  config, or documented decision). Knowledge-only work is a
  diagnostic, capped at one per question, and must state which
  decision it unblocks.
- Estimator-input changes trigger threshold re-derivation.
- Tier A methods need a verified anchor; full citations live in
  thesis_notes.md, code carries a pointer only.
- STATUS is rewritten at every work stop; history goes below.

### ANCHORS (verified)
Rajamani, Vehicle Dynamics and Control, 2nd ed. 2012 - Ch. 2,
sec. 2.3 bicycle model p. 27, sec. 2.6 yaw-rate/slip-angle model
p. 37; Ch. 14 Kalman application. Kiencke & Nielsen, Automotive
Control Systems, 2nd ed. 2005 - "Vehicle Body Side Slip Angle
Observer" section. Both confirmed visually by the user. Lecture
anchor: dropped by decision. Open: Mitschke/Wallentowitz "p. TBD"
in estimate_sideslip to be REPLACED by Rajamani 2.6/2.3, not
deleted (not yet done).

## STATUS HISTORY (superseded, newest last)
Its "still-uncommitted" claims are OUTDATED -- that work was committed in 82bc49c, f37053f, e6ef209, 0bdff87, 5f44688, b96d59a, 2d3346f; read it as historical narrative only, never as current repo state.

## STATUS (update at every work stop)
Current WP: WP-S4b (observer self-consistency check) DONE (2026-08-19,
this session -- see thesis_notes.md "WP-S4b: observer self-consistency
and the Cr_A inflation finding" for full detail), closing out this
session's Open Board item B (sideslip methods comparison) sequence:
WP-S1 (wheel-speed source characterization), WP-S2 (comparison harness,
Metrics 1-4), WP-S3 (Metric 5, zero-slip Fy offset + direction-match),
WP-S3b (chain decomposition: geometric-cancellation reframing,
force-balance steady-state gap, IMU-lever-arm/weight-split mechanisms
rejected), WP-S3c (washout-mechanism ablation, uninformative -- own
drift too large to trust), WP-S4 (linear Kalman sideslip observer
registered as third diagnostics-only candidate C_kalman_observer;
confirms real steady-state slip where the kinematic candidate reads
~0, at every corner, in sign), WP-S4b (this turn). OUTCOME: the 2-3x
alpha_r_ss overshoot reported at WP-S4 traced to an inflated reference
stiffness (Cr_A, fitted against the washout-suppressed kinematic
alpha) rather than a flaw in the observer -- worst at exactly the
corners (C3/C11/C13) that showed the worst apparent overshoot.
Separately, Cr_A's own ~4x corner-to-corner spread (79k-337k N/rad) is
new evidence the kinematic alpha's error reaches the PRODUCTION
cornering-stiffness estimate itself (Module 4b), not only beta -- a
live finding, not yet acted on; the standing CS threshold re-derivation
rule applies once/if a beta fix is wired to production, not before.
Entire arc is diagnostics-only: no production/UI/pipeline wiring
changed; the only config changes are two re-introduced fallback-
stiffness keys and one new Iz placeholder, both diagnostics-only
consumers, test_stability.py confirmed byte-identical after every
step. Next steps: Q/R tuning for the Kalman observer (currently hand-
tuned initial placeholders with no ground truth to tune against), then
the WP-S6 comparison report closing out Open Board item B. User has
not committed yet, stopped before commit every turn this session per
instruction -- this WP-S1..S4b arc lands on top of the prior sessions'
still-uncommitted work described below (WP1 consolidation, WP2b-2 +
matrix v2 review, WP5b(b)/(c)/(d), the Accuracy-registry arc), unless
those have since been committed separately (this block was stale
relative to git log before this update; check `git log` before relying
on "still-uncommitted" claims below for anything predating 2026-08-19).

Queue once this lands, confirmed still accurate: the new-data-file
diagnostic checklist (runs automatically when a new log arrives -- also
the reproduction check WP5b(d)'s k-application decision is gated on);
the WP2b-2 engineer follow-up questions (headroom-ranking, rake-package
magnitude, TC-LAT-escalation throttle scoping -- see WP2b-2 section) PLUS
the matrix v2 tick-through (dagger-marked/project-lead-reviewed cells,
ABS direction semantics, the three situational OS-APX cells -- see
"Matrix v2 review round" section); WP5b(b) phase 2 (damper-derived Level 4
wheel loads + the roll-stiffness DOMAIN IMPROVEMENT split); sourcing the
three reviewer-placeholder figures (cog_height_m, track_width_front/
rear_m, lift_coeff/Cl) from the team; the WP5b(d) k-application decision
itself (apply the measured k=1.01211 rolling-radius correction to
ecu_speed, or not) -- gated on the second-track reproduction check above,
its own re-derivation stop, not taken this session. New this close-out:
WP1's two open watch items (C10 corner_radius_filtered overlap 0.60 vs.
the 0.67 pre-WP1 baseline; C9's inter-lap agreement dip + the C9-lap1
CS_r=-0.721 flag, both tracing to C9's own start boundary never having
been independently examined) -- see thesis_notes.md "WP1 open watch
items, carried forward".
Last commit:
113c7d9 (clean up).
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
~~Open threads: corner_analysis.py:359 lap_distance interpolation lacks
the lap-boundary reset guard that stability_analysis has (found
2026-07-25 during study doc stage 2). Fix requires WP1-freeze proof:
before/after identical stable_corner_id assignment and corner count on
Dubai. Small targeted WP after study document.~~ [DONE 2026-07-26,
small-decisions sweep item 3: corner_analysis.py's apex_lap_distance_m
now routes through the shared _interp_lap_distance_guarded helper
(modules/stability_analysis.py, imported directly, no duplication).
WP1-freeze proof run: per-lap corner counts, every corner's
apex_lap_distance_m (10 decimal places), and every stable_corner_id
byte-identical before/after on Dubai (diagnostics/inspect_wp1_reset_
guard_freeze_proof.py) -- confirms the fix is a no-op on this file, as
expected (no apex sits near a lap-boundary reset). bracket_start_m/
bracket_end_m stay unguarded, out of scope for this fix.
thesis_notes.md: "corner_analysis.py:359 reset-guard fix (small-
decisions sweep)".]
~~cs_front/rear_fallback_reference config keys defined+commented but
consumed nowhere (found 2026-07-26). Decision needed: wire per comment
(estimator change, triggers re-derivation -- bundle with WP5b(b)) or
remove. Do not wire casually.~~ [DONE 2026-07-26, small-decisions
sweep item 1: DELETED (cs_front_fallback_reference_n_per_rad,
cs_rear_fallback_reference_n_per_rad, and their explanatory comment,
config/parameters.json) -- confirmed consumed nowhere in modules/.
Rationale: the no-linear-reference case has never occurred on real
data; if it ever does, the corner reports invalid, more honest than
filling from an unvalidated constant. thesis_notes.md: "cs fallback
reference constants deleted (small-decisions sweep)".]
~~Chair estimator has a time_s-anchored fallback mode (window/grid
scaled /50) when s_m is unusable; SetupTool short-circuits to
all-invalid instead (study doc §8c). Decide: port the fallback tier or
document the short-circuit as deliberate. Moot for current data.~~
[DONE 2026-07-26, small-decisions sweep item 2: documented as
deliberate, not ported -- one sentence added to estimate_yaw_moment_
stability's docstring (modules/stability_analysis.py): the fallback is
a differently-behaving estimator (time-local, no cross-lap pooling)
whose output the s-grid-derived thresholds could not classify
meaningfully; no verdict is more honest than a silently degraded one.
thesis_notes.md: "s_m short-circuit documented as deliberate
(small-decisions sweep)".]
~~Accuracy-level registry consolidation (rider WP): single source in
config, wire the inline dict + the hardcoded duplicate, add beta +
wheelbase entries, verify and apply the weakest-link semantics, re-tag
accordingly; found 2026-07-26, study doc §11.~~ [DONE 2026-07-26, see
the Accuracy-registry arc session below.]
Repo handover procedure (decided 2026-07-26): docs/car_data images
exist in main's HISTORY (untracked from the index 2026-07-26, prior
commits unaffected). Therefore the showing branch MUST be an orphan
branch (no parent history) pushed to a separate submission remote;
never grant access to the working remote. No history rewrite on main
- keeps OLD_REF and all recorded commit hashes valid. Protected-set
verification from now on: git ls-files on all protected paths must
return empty, check-ignore alone is insufficient.

Accuracy-registry arc (2026-07-26, WP-A through marker cleanup,
currently uncommitted, lands as one combined commit): five pieces in
sequence. (1) WP-A: config/parameters.json's accuracy_levels block
consolidated to the single source for every per-quantity tag --
extended to eleven then twelve nodes, each {level, source, capped_by};
prepare_vehicle_state's inline accuracy dict and estimate_lateral_
forces's hardcoded literal deleted, both now read the registry;
resolved the yaw_rate=3-vs-steering_angle=1 weakest-link question with
two named mechanisms (chained-constant vs provenance-assumption, not
one); removed six stale VBOX_*/gpsa_* channels.json entries verified
absent from the real Dubai file. Zero behaviour change throughout.
(2) WP-C: new modules/accuracy_resolution.py -- per-session resolver
for mass/corner_weights (leaf nodes, highest-available-wins, never
blended) and cog_position (pure cascade); global accuracy-cap dropdown
next to Analyse, plumbed as a plain value like lap_filter, never read
from UI state inside modules/; both WP5 (Outing.analysis_data) and WP6
(_pipeline_cache) identities extended with accuracy_cap + a resolved_
vehicle_snapshot, ANALYSIS_SCHEMA_VERSION bumped 1->2; resolved-level
footer + clipped-only "comparison run" tag with the threshold caveat
text. Five-cap byte-identity regression on Dubai confirmed; synthetic
acceptance proof (corner weights shifted +2.06pp front) confirmed
Fy_f/CS_ratio_f respond correctly, CS_ratio_f's flat median explained
by the pre-existing ceiling-clipping effect, not a defect.
(3) Small WP: explicit Save button (Tier C) next to Back, factored
_save_outing into a shared _persist_outing() core; first Save in
new-outing mode now sets self.outing so the form becomes edit mode
(no duplicate row on a later Back) -- found and fixed a real
DetachedInstanceError (session.commit() expires attributes by default;
session.refresh() before close fixes it) via a synthetic offscreen-Qt
test against an isolated throwaway DB, never data/setuptool.db.
Post-save WARN hint reuses the existing stability_status_label, no
auto-rerun. One later hardening fix: the bare except TypeError around
setup_data parsing now logs before returning, no behaviour change on
the happy path.
(4) WP-B: steering_ratio Level 1->4, config/car_data.json's
steering_ratio_table (21 rows, monotonic, np.interp default clamp,
+/-291.5 deg domain vs log_asteer's actual -130.7/+113.9 deg on Dubai
-- never clamps on this data). New leaf node (car_data.json presence-
gated, no per-outing setup_data involvement) plus steering_angle as a
fourth pure cascade. Graceful L1 fallback verified with car_data.json
genuinely renamed away in a fresh process; test_stability.py's raw,
un-resolved call path confirmed byte-identical. Diagnostic found and
corrected the proposal's own claim: Module 5's stability regressand is
NOT byte-identical (delta_f_rad is one of its five ridge regressors) --
front-axle-only scoping verified directly (beta/Fy_f/Fy_r/alpha_r/
CS_ratio_r byte-identical, alpha_f/CS_ratio_f/stability move). Sign
dispute resolved by the diagnostic, not argument: the effect is signed
with the steering direction (increases alpha_f one way, decreases the
other), this session's data skews toward the decreasing side. Location
prediction confirmed (low-speed corners shift most) with a nuance
(medium/low separation weaker than pre-registered). Zero verdict flips
under unchanged thresholds; all five classification thresholds
re-confirmed unchanged after review (config/parameters.json
derived_from fields, thesis_notes.md).
(5) Full channel census (2622 channels, re-scanned with correct
cp1252 decoding -- the existing scan_channels.py's utf-8/errors=
"replace" read mangles every degree-sign unit) + targeted
verification: no lateral-velocity/sideslip/optical-sensor channel
exists anywhere in the log; the log_a_car heading hypothesis tested
and REFUTED (r=-0.001 vs yaw rate); corner_radius confirmed as a live
logged curvature channel (r=0.87 vs ay/v^2); TO_VBOX_01-05 confirmed
constant/inert. Chair-context marker cleanup: all four "[to verify
with chair]"/"[context claim, to verify with chair]" markers in
thesis_notes.md removed, each surrounding claim reworded to rest on
SetupTool's own post-session nature and documented physics/coordinate
evidence rather than an unverified assumption about the chair
pipeline's operating context; DOMAIN IMPROVEMENT taxonomy labels
unchanged throughout (the classification never depended on the
removed wording).
Open threads: worst-phase selector sentinel -- all-phases-at-ceiling
reports NaN in diagnostics while classifying as normal (found
explaining the WP-B n=49->48 count; the sentinel starts at the ceiling
and only updates on a strictly-lower value, so an instance with every
phase exactly at 1.0 has no phase that reads as "the worst"); reconcile
the representation someday, not urgent, no diagnostic or verdict was
wrong because of it. Everything still open from before this arc stays
open, unchanged by it: the new-data-file diagnostic checklist (tc_lat/
tc_lon/abs_position/brake_bias channel-name scan) above WP1; WP2b-2
(rule engineering against the setup_parameters registry); WP5b(d)
(speed cross-validation) -- log_gps_speed confirmed present in the
real file but still not whitelisted, deliberately not added ahead of
this consumer; the small-decisions list -- cs_front/rear_fallback_
reference wiring-or-removal decision, the chair's time_s-anchored
Module 5 fallback tier (port or document-as-deliberate), and the
corner_analysis.py:359 lap_distance reset-guard fix (needs a WP1-freeze
before/after proof first). [UPDATE 2026-07-26: WP5b(c) (GPS-course
beta) is no longer open -- SHELVED after two implementation iterations,
see WP5b section (c) and thesis_notes.md for the full validation
record and reopen condition (denser anchor data). log_gps_course IS
now whitelisted (config/channels.json), unlike log_gps_speed above,
which stays deliberately unwhitelisted pending WP5b(d). [FURTHER
UPDATE 2026-07-26: WP5b(d) itself is now DONE too -- log_gps_speed IS
now whitelisted, see WP5b section (d) for the verdict (b) record.]

WP5b(b) phase 1 + WP5b(c) session (2026-07-26, currently uncommitted,
lands with the Accuracy-registry arc above as one combined commit):
two work packages, both chair-parity/validation work on Modules 4b/6.
TURN (a) -- Fz compute function: new modules/stability_analysis.py
estimate_vertical_loads(state, forces, params), chair-identical axle
Fz (static + aero + longitudinal transfer) and per-wheel split
(independent-per-axle lateral transfer, NOT the roll-stiffness
DOMAIN IMPROVEMENT, which stays a later sub-step) plus fy_f_norm_N/
fy_r_norm_N (Fy_filt/fz axle), Tier A (Milliken RCVD, p. TBD verify),
docstring states no deviation from the chair construction. New config
(vehicle.cog_height_m=0.30, track_width_front/rear_m=1.66/1.64, all
reviewer placeholders "NOT sourced, replace with team figure";
aero.air_density_kgm3=1.225 L1-by-convention; aero.lift_coeff=0.0,
documented zero-meaning, sign convention inferred from the chair's own
formula+comment -- not yet empirically confirmed, config note flags
the first real Cl entry must validate Fz rising with v^2). Two new
accuracy_levels registry nodes (vertical_load_split,
per_wheel_load_split), both Level 1. Isolated: zero call sites,
test_stability.py byte-identical. Sign-convention verification against
real Dubai data (not assumed): braking loads the front (ax negative
under braking, matching the chair's formula) and a GPS-derived
right-hander loads the left (outside) tire -- both MATCH, no flip
needed. TURN (b) -- consumer wiring: estimate_vertical_loads joins
StabilityAnalysisThread's pipeline (after estimate_lateral_forces) and
the WP6 _pipeline_cache identity (new "fz" key); summarise_corners
gained an optional fz= parameter adding fz_f_N/fz_r_N/fy_f_norm_N/
fy_r_norm_N per-phase stat blocks (additive only, old call sites
unaffected); UI gained one Fzf/Fzr median column pair in the corner-
details phase table (fy_norm computed but not displayed, doesn't fit
the panel width cleanly, deferred); nothing feeds _classify_corner.
ANALYSIS_SCHEMA_VERSION 2->3 (payload shape change, comment explains
why). Verified additive-only: a before/after summary-dict diff showed
zero pre-existing keys changed, only the new keys added; three sample
Fz values reported physically plausible (5-8 kN/axle, static-dominated
at low ax/ay, matching the static-split calculation almost exactly).
WP5b(c) -- GPS-course beta, two iterations, SHELVED: see the WP5b
section (c) above for the work-package-level summary and
thesis_notes.md for the full validation record (rotation-convention
and latency findings, iteration 1's numbers and diagnosed root cause,
iteration 2's two fixes and the falsifiable lever-arm check that
passed without moving the decision-criteria metrics). config/
parameters.json's accuracy_levels.sideslip_angle gained an inactive
registry note recording the outcome; production beta untouched
throughout. Closeout pass (this session): registry note text finalised
(above), this WP5b(b)/(c) STATUS paragraph and the WP5b section (b)/(c)
markers added, Open threads paragraph corrected (WP5b(c) removed from
the open/gated list). thesis_notes.md completeness checked against
today's work -- see the session's own report for the verdict; nothing
added beyond what was explicitly flagged.

WP-N2 pass 0 build session (2026-08-19, superseded by the pass 1-4
arc and the carry-forward decision above -- kept for the development
narrative): pass 0 (nonlinear single-track EKF, Dugoff tyre model,
frozen WP-N1b curve) built -- diagnostics/sideslip_ekf_dugoff.py
(states [beta, yaw_rate], Vx/delta_f scheduled, Dugoff forces +
analytic dugoff_lateral_stiffness for both Jacobians, nonlinear state
propagation with covariance-only Ad, windowed-NIS + hard-|beta|-bound
divergence monitor, fixed fallback to kinematic beta/measured yaw
rate/P0); diagnostics/inspect_ekf_dugoff_sanity_checks.py
(Jacobian-collapse check against the rejected linear filter's own A/C
-- exact match at alpha=0, small expected deviation at alpha=0.02 rad
from Dugoff's tan(alpha) nonlinearity; h2-vs-ay consistency check,
explicitly labelled NOT validation and PARTLY CIRCULAR). config/
parameters.json gained tyre_model_ekf.pass_0 (additive, tyre_model_fit
untouched): frozen Dugoff parameters + frozen_from pointer, Q/R/P0
seeded from the tuned linear observer (QR_RATIO=0.3162), beta_hard_
bound_deg=15.0 (physically anchored), NIS window/bound/fraction
(20/5.99/0.5, placeholder pending validation), Iz_provenance,
fy_axle_dependency_note.
KEY FINDING from this build (verified numerically on Dubai data):
Module 4a's Fy_f/Fy_r satisfy a*Fy_f - b*Fy_r == Iz*psidd_raw
IDENTICALLY (max deviation 7.3e-12 Nm using live a/b) -- the two axle
forces carry exactly TWO independent measured quantities (ay, psidd)
between them, not four independent numbers; any per-axle fit against
both must be read with this coupling in mind. Iz choice: vehicle.
yaw_inertia_kgm2 (2082.0), not yaw_inertia_kalman_kgm2 (1800.0) --
consistency with the training-data forces, not a better-sourced claim.
DOCUMENTATION FIX, same session: the Ulsoy, Peng, Cakmakci citation
corrected (confirmed by two independent readings) to anchor the
nonlinear single-track vehicle model (sec. 14.3) and sideslip's
operational significance (sec. 14.1), not observer structure; Eq. 14.8
confirmed a term-by-term match (two documented simplifications: no
roll DOF, pure-lateral Dugoff vs. combined-slip Magic Formula).
Open design decisions this build carried forward (all since resolved
or superseded by the pass 1-4 arc and carry-forward decision above):
sign correctness at racing-speed corners; the saturation/circularity
check reframed for a nonlinear model; steady-state magnitude check;
NIS/bound placeholder tuning; the Q/R sensitivity check under this
filter's state-dependent Jacobian; how the fitted curve's valid slip
range gets reported as a production-facing mechanism (still not
designed -- unchanged open item, now inherited by whichever future WP
revisits production wiring).

### NOW section archived [superseded 2026-08-20, replaced by the
entry_1_brake production-fix / method-lineage-correction / plan
rewrite below the current STATUS block]
Track A (numbers correct): sideslip methods-comparison arc CONTINUES --
NOT closed. Linear-tyre Kalman observer (WP-S4/S5/S5b/S6) REJECTED for
production (saturation-detection failure). Kinematic estimate remains
production. WP-N0/N1/N1b DONE (Dugoff model chosen, c_alpha refit from
Module 4b, mu_fz interior optimum both axles): frozen pass-0/pass-1
Dugoff parameters c_alpha_front=132798, c_alpha_rear=174217 N/rad,
mu_fz_front=10653, mu_fz_rear=15819 N (read exact values from config --
do not trust these).

WP-N2, nonlinear single-track EKF, Dugoff tyre model. Pass 0 built and
run (frozen WP-N1b curve, Q/R/P0 seeded from the tuned linear observer):
NIS baseline poor (93.4% combined exceedance -- expected, since R
assumed sensor-only noise while the curve's own fit residuals are two
orders of magnitude larger), but three convergent lines of evidence for
kinematic slip-angle under-read. Pass 1: noise-model recalibration only
(curve unchanged), R redefined as total innovation uncertainty, 2-D
sweep found one interior grid point inside the pre-registered NIS band
-- NIS/sign/C2-excursion gates all PASSED. Pass 1's flagged CS_ratio
counts jumped sharply (front 32/56, rear 27/56 vs kinematic 11/9);
investigated and found NOT YET INTERPRETABLE -- thresholds are
kinematic-fitted and the whole pass_1 distribution shifted across every
percentile band, not just the tail. THRESHOLD RE-DERIVATION
DELIBERATELY DEFERRED (see PARKED) until the estimator is finalised.

Passes 2-4: refit loop, attempting to break the kinematic-sourced
curve's circularity by refitting c_alpha/mu_fz from the EKF's own
converged slip angles each pass, Q/R/P0 held fixed to isolate the
curve's effect, predictions pre-registered before every pass (thesis_
notes.md WP-N2 pass 2/3/4 entries). OUTCOME: NON-CONVERGED, STOPPED at
pass 4 (one short of the pre-registered 4-pass cap) on the
pre-registered failure criteria, not the cap. FRONT axle oscillated
with GROWING (not shrinking) magnitude at pass 4 -- failure mode 1.
REAR axle's mu_fz fit diverged to its search bracket's ceiling at pass
4 (8.48e6 N, effective mu 1102.5, onset 88.4 deg, coverage exactly
0.0000) -- failure mode 3, the curve degenerating to pure-linear, the
same structural blind spot that condemned the linear observer, arrived
at here by fit failure rather than design. MECHANISM: a self-starving
positive feedback (lower c_alpha -> onset moves outward -> fewer
saturating samples -> mu_fz less identifiable -> drifts up -> onset
moves out further), pre-registered as a risk at pass 0 (rear only 6.95%
of samples past onset) and confirmed as the cause. SECOND READING,
EXPECTED not established: the rear axle of this RWD car may saturate
principally under longitudinal traction on exit, not laterally, so a
pure-lateral model has genuinely little rear saturation to identify in
this data -- connects directly to the PARKED combined-slip item. Full
trajectory, mechanism and scorecard: thesis_notes.md "WP-N2 refit loop:
NON-CONVERGENCE, rear degeneracy to a pure-linear curve, and the
identifiability limit".

CARRY-FORWARD DECISION (thesis_notes.md "WP-N2 carry-forward decision:
pass 1"): the estimator carried forward is PASS 1's configuration
(pass_0's Dugoff curve, pass_1's calibrated noise model) -- chosen by a
PROVENANCE RULE stated before any outcome comparison (pass 1 is the
last configuration not produced by the non-converging refit loop), not
by comparing which pass scored best, which would be retrofitting.
CARRIED-FORWARD LIMITATION, not resolved: pass 1's curve is still
fitted from KINEMATIC slip angles -- the exact circularity the refit
loop existed to break, and the loop failed to break it. Stated
limitation of the carried-forward method, not a solved problem.

R RE-DERIVATION DECIDED NOT NEEDED (2026-08-20, thesis_notes.md "WP-N2
pass 1: final validation baseline"): pass 1's R was derived from and
NIS-gated against this exact curve and filter configuration -- the
acceptance figures (yaw_rate 10.01%, ay 9.18%, combined mean NIS
2.907) are already a direct measurement of the carried-forward
estimator, not an inference from provenance. The staleness found in
passes 2-4 was specific to refitted curves, none of which are carried
forward. No change made. Two refinement opportunities remain open,
not defects: Q was never swept, and the accepted R is one interior
point on a coarse 5x5 grid with no finer search run around it.

NEXT STEP: the classification-threshold re-derivation (deferred, see
PARKED), and the combined-slip comparison the rear degeneracy now
motivates more strongly than before.

Last commit: 4b1b548 (Sideslip observer arc: comparison harness, Kalman
candidate, tuning, report). Local main up to date with origin/main.
Uncommitted: everything from the WP-N0/N1/N1b turn plus the full WP-N2
arc this session -- pass 0/1/2/3/4 filter runs and their config blocks
(tyre_model_ekf.pass_0 through pass_4), diagnostics/sideslip_ekf_
dugoff.py and every inspect_ekf_*/fit_dugoff_pass*_refit.py/*_manifest.
json script and file this arc produced, the carry-forward decision, and
every corresponding thesis_notes.md entry (see thesis_notes.md for the
full, dated file-by-file record of each pass). This PLAN.md rewrite.

## SUPERSEDED NOW-BLOCK CLOSING TEXT (2026-08-31 STATUS rewrite)
The following closing paragraph stood at the end of ### NOW's "WHERE
THE PROJECT STANDS" bullet list from the 2026-08-30 ship-readiness
cleanup entry until the 2026-08-31 STATUS rewrite replaced it with the
PROJECT DIRECTION paragraph (method work frozen; commit/reliability/
thesis-writing priorities) now in ### NOW. Preserved here verbatim,
not because its content is wrong -- STEP 4's prerequisites listed below
are still exactly as scoped -- but because "next substantive work" is
no longer an accurate framing of what happens next:
  NEXT: STEP 4 below (decision-matrix cleanup) is the next substantive
  work -- its prerequisites are unchanged by this cleanup package and
  already listed in full under STEP 4's own entry (threshold re-
  derivation for CS_ratio AND LS_ratio together, the CS_ratio cross-
  lap aggregation problem under PARKED, the entry_1_brake rule-base
  implications, and whether LS_ratio enters the recommendation rules
  at all).

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
- Re-attempt GPS-course sideslip (shelved, thesis_notes.md WP5b(c)) as an
  independent arbiter at C6/C10 (the two racing-speed corners where the
  kinematic estimate and the Kalman observer disagree in sign, WP-S5) --
  only meaningful once the denser anchor data / longer session reopen
  condition is met, see thesis_notes.md "GPS-course sideslip as a
  potential arbiter" entry.

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

## WP1 consolidation — canonical corner realization

### Turn 1 [COMPLETE 2026-07-26]
Two-pass split (pass 1 unchanged, pass 2 new -- confident-member canonical-
window reassignment for `straddles_adjacent_corners`-tagged brackets) +
canonical bracket/phase-boundary derivation (median per boundary, reset-
guarded, closes the bracket-edge guard gap WP1 left open) + re-realization
over every valid lap (`canonical_quiet` tagging for previously-missing
laps) + canonical `speed_class` once per stable corner. Full detail,
including a real edge-effect bug caught and fixed during implementation
and the C10/C11 before/after result, in thesis_notes.md "WP1
consolidation, Turn 1" [2026-07-26]. `modules/corner_analysis.py` only;
detection code (`_analyse_lap`/`_bracket_corners_by_steering`/
`_bracket_corners_by_speed`/`_build_corner`) untouched, verified by
source hash. `test_stability.py` clean; Modules 1-5 sample-level outputs
verified numerically unchanged.

### Turn 2 — validation + re-derivation inputs [COMPLETE 2026-07-26]
Read-only diagnostics, no thresholds/code changed. Added a canonical
overlap matrix (new): C9<->C10 overlap 88% of the smaller window, 99.4%
sample-sharing every lap, traced to one pass-1 connected component (lap 2
uniquely contributes 2 brackets there); C11<->C12 overlap 32%.
corner_radius_filtered overlap regressed for C10 (0.67 pre-WP1 baseline ->
0.55) -- a third independent method converging on the same pair.
Inter-lap stability agreement tightened 10-100x for 13/14 corners
(confirms the pooled-grid mechanism). Full detail, thesis_notes.md "WP1
consolidation, Turn 2". Handed the C9/C10 merge-vs-partition question to
the reviewer rather than resolving it -- resolved in Turn 3 below.

### Turn 3 — canonical boundary resolution [COMPLETE 2026-07-26, reviewer decision: partition not merge]
New post-pass `_resolve_canonical_overlaps` (modules/corner_analysis.py):
any canonical-window pair overlapping more than `canonical_overlap_max`
(new config key, 0.10) is truncated to a shared boundary at the pooled
(cross-lap median) |ay| minimum between the two apex positions -- the
"split a compound at an ay minimum" idea named as an open refinement in
thesis_notes.md's original 2026-07-22 compound-corner finding, implemented
six months later for canonical windows specifically. Phase boundaries
re-clamped into the truncated range; an absent phase (its defining event
outside the sub-window, e.g. C10 has no brake phase now) collapses to a
degenerate zero-length phase -- summarise_corners/_classify_corner already
read that as "no signal" with no code change needed to either. Result:
zero overlap, zero sample-sharing for both pairs; C11 reclassifies from
"medium" to "high" (152-162 km/h) -- it was genuinely mis-classified
before, not just noisy; C10's corner_radius overlap improves 0.55->0.60
(short of the original 0.67 baseline); C9's inter-lap agreement worsens
slightly (std 6.7->12.0, still small absolute) -- reported plainly, not
spun. C8/C3's six-most-negative-stability finding is now IDENTICAL to the
original pre-WP1 result -- the most reassuring cross-check this session,
since nothing about canonical realization should touch C8 or C3 at all.
Full detail, thesis_notes.md "WP1 consolidation, Turn 3".

### Turn 4 — threshold re-derivation [COMPLETE 2026-07-27: RE-CONFIRMED UNCHANGED, user's own call]
Decision: keep all five values (STRONG_CSF/STRONG_CSR/MODERATE_CSF/
MODERATE_CSR/stab_neg_thresh_Nm_per_deg unchanged) -- `derived_from`
strings in config/parameters.json each gained a dated append recording
the argument (stability's six-most-negative distribution and its
exceedance counts are tri-state-invariant across pre-WP1/post-Turn-1/
post-Turn-3; CS-side moderate-count movement traces to repaired
realization defects, C11's misclassification and window jitter, not
estimator drift). No value changes. Full detail and the verdict-
distribution re-check (0/15/41 of 56, 26.8% flagged, vs. the historical
~33% and ~27.5% points and the June driver report) in thesis_notes.md
"Threshold re-confirmation after WP1 consolidation" and "Verdict-
distribution re-check after WP1 consolidation". Blocking gate from
CLAUDE.md's grounding rule is now closed for this arc.

### WP1 open watch items (carried forward, not blocking)
Two threads deliberately left open rather than chased further this
session, both tracing to the same unexamined variable (C9's own start
boundary, never touched by the Turn 3 partition and never independently
re-examined the way the shared C9/C10 boundary was): (1) C10's
corner_radius_filtered overlap (0.60 post-partition) hasn't fully
recovered to the original pre-WP1 baseline (0.67); (2) C9's inter-lap
stability agreement worsened slightly post-partition (std 6.7->12.0) and
its canonical_quiet lap-1 instance's CS_r=-0.721 flag persists unresolved.
Neither blocks anything currently planned; revisit if C9/C10 become
load-bearing for a recommendation-engine rule. See thesis_notes.md "WP1
open watch items, carried forward".

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

## WP2b-2 — Rule engineering against the registry [COMPLETE 2026-07-26]
Rewrote `config/recommendations.json` against an external engineer decision
matrix (scenario x speed-class grid, supplied as an authoritative input, not
derived in-repo): 7 WP2 ARB-only seeds retired in place (status field, kept
for history); 26 rules elicited 1:1 from the matrix's live cells; 3 held
escalation rules (fully specified, zero-firing, pending an applied-
recommendations history log to activate); 2 dropped cells (OS-BRK-low,
INST-ENT) recorded for traceability with no action, per the matrix's own
"none"/scope-drop. Every rule carries a `cell_id` back-reference.

`modules/recommendation.py` gained: speed_class matrix gate (modal
aggregation across a corner's laps, config/channels.json thresholds);
per-lap consistency gate (verdict must repeat on >= min_repeat_laps AND >=
min_repeat_fraction of analysed laps, both config); a `suggestion` schema
that accepts a package/axle-symmetric-pair action list, not just one
parameter (registry has no combined axle-level ARB/camber/damper key);
escalation_tier ranking (new setup_parameters.json field, deliberately
separate from change_effort -- see thesis_notes.md); a parameter_conflict
post-pass (two rules recommending opposite directions/targets for the same
registry parameter surface to the engineer, never netted); a feasibility
post-pass (current setup-sheet value + action delta against the registry's
range, "at_limit"/"unchecked"/"ok", `setup_data` now actually read); and an
action_class split (advisory vs. recommended, config-driven boundary --
see thesis_notes.md "ACTION-CLASS SPLIT"). `config/setup_parameters.json`
gained `escalation_tier` on every recommendation_target entry and a `step`
field (0.3 deg) on the four camber entries. UI (`ui/views/outing_form.py`
`_build_recommendation_row`) renders cell_id(s), the ADVISORY/SELECTED
tag, AT LIMIT / limit-not-checked markers, the new parameter_conflict
badge (BAD, distinct from the existing driver/data WARN conflict), and
advisory buckets' observation lines.

Verified via a synthetic script (not yet a committed test file -- see
follow-up list below): all 31 matrix cells + the 1 dropped combined-entry
cell round-trip 1:1 into the rule file; a verdict repeating on only 1 of 5
laps produces zero recommendations for that corner; two corners with
opposite-direction rear-ARB matrix cells both surface `parameter_conflict`
and neither gets auto-selected; ranking is identical across repeat calls
with identical input; an infeasible toe change renders AT LIMIT and is
excluded from the change budget, an unchecked tc_lon change is not;
a moderate data-only match renders advisory with zero budget selections,
the same corner with corroborating driver feedback renders recommended and
selected. `test_stability.py` (Modules 1-5, does not call the
recommendation engine) unaffected, clean run.

Optional scope addition NOT done this session: per-driver feedback
weighting (NOT "driver level") -- an optional field on the `Driver` model
letting a driver's reported feedback carry more or less weight in
`source_balance` resolution. `modules/recommendation.py` still isolates
this behind `_resolve_source_balance(config, outing)`, which today just
returns `config["settings"]["source_balance"]`; extend it to resolve in
order feedback-weighting override (driver) > outing override > global
default rather than reading `settings["source_balance"]` inline anywhere
else.

### Engineer follow-up questions (answers needed before further WP2b work)
- [2026-07-26, WP2b-2 amendment 6] Should a recommended change's remaining
  headroom against its registry limit (not just a binary in-range/at-limit)
  influence ranking -- e.g. rank a change with lots of room below a change
  that's nearly at its limit, all else equal? Not elicited from the
  decision matrix; deliberately NOT implemented this session (no
  comfort-zone/headroom ranking) pending your answer.
- The "rake forward"/"rake reduction" ride_height_front/rear package
  magnitude was set to the matrix's stated 1-2mm window's minimum (1mm) as
  a conservative first-change default, since the matrix gave a range, not
  a single value -- confirm 1mm is the right choice, or state a different
  default.
- The held US-APX-med-esc (TC LAT) escalation's scoping caveat from
  thesis_notes.md ("valid for understeer WITH throttle involvement... not
  off-throttle push") is not yet encoded as a phase/throttle condition on
  the rule itself -- needed before promoting that rule out of "held", not
  before (it never fires while held).
- [2026-07-26, WP2b-2 amendment: elicitation_provenance, RESOLVED same day]
  Full markup supplied and applied to `config/recommendations.json`: 12
  engineer-verbatim cells (US-BRK-med, US-TIN-low/med, US-APX-med,
  US-EXIT-low/med/high, OS-BRK-med, OS-EXIT-low/high, INST-BRK-med,
  US-BRK-high's base package) + 2 held escalations inheriting it
  (US-BRK-med-esc, US-APX-med-esc); 8 project-default (asterisked) cells
  (US-BRK-low, US-TIN-high, US-APX-low, US-APX-high, OS-BRK-high,
  OS-EXIT-med -- the sign-corrected "S8-Med" case, INST-BRK-low/high) + 1
  held escalation overridden to project-default (US-BRK-high-esc, bump-HS
  direction defaulted, NOT inherited from its verbatim base US-BRK-high);
  6 mirror-derived cells (OS-TIN-low/med/high, OS-APX-low/med/high,
  unchanged from the prior pass). Verified: 12/8/6 split on the 26
  elicited-status rules, dropped/retired rules null. Confirmation list is
  exactly the 8 project-default + 6 mirror-derived cells (all capped to
  ADVISORY under `settings.action_class.cap_non_verbatim_to_advisory`
  until individually promoted to status "reviewed") -- these 14 cells are
  the concrete engineer follow-up: OS-APX-low (front ARB +1 vs. rear ARB
  -1, see the prior question below, still open), and the other 13:
  US-BRK-low (ABS map -- which literal position was meant), US-TIN-high
  (wing direction), US-APX-low (which axle "ARB" meant), US-APX-high
  (camber step magnitude), OS-BRK-high (rear ARB direction), OS-EXIT-med
  (whether the sign correction from the engineer's stated "+1" to the
  ruleset's soften/-1 is right), INST-BRK-low (ABS map), INST-BRK-high
  (front vs. rear axle), OS-TIN-low/med/high and OS-APX-med/high (whether
  the US-side mirror actually holds for the OS-side scenario), and
  US-BRK-high-esc (bump-HS direction, held -- moot until promoted out of
  "held" anyway).
- [2026-07-26, matrix v2 review round, PARTIALLY RESOLVES the above] All
  8 project-default cells and 3 of the 6 mirror-derived cells (OS-TIN-
  low/med/high) confirmed by project-lead review and promoted to a new
  grade, `project-lead-reviewed` (action-eligible, between engineer-
  verbatim and the capped grades) -- see thesis_notes.md "Matrix v2
  review round". US-BRK-low and INST-BRK-low's ABS-map answers, decoded
  as garbled-then-defaulted, are REWRITTEN (front toe / try-and-error
  ABS nudge, no more literal position target) rather than simply
  confirmed. Camber step corrected 0.3->0.1deg. OS-APX-low/med/high
  instead got a `situational: true` flag (permanently advisory,
  alternatives listed verbatim in rationale) rather than a provenance
  promotion -- the matrix's own review concluded these three
  specifically don't have one confirmable answer (axle-grip load-
  sensitivity, see thesis_notes.md), so "confirm which lever" is the
  wrong question for them; nothing further to tick off there.
  Tick-through interpretation (flagging where I inferred rather than
  was told directly -- confirm or correct): "the dagger-marked cells"
  is read as the 11 newly project-lead-reviewed cells (this session
  introduced no other new markup tier); "the three remaining verbatim-
  pending items" is read as OS-APX-low/med/high, the only cells left
  short of engineer-verbatim after this round (now situational rather
  than pending, per the point above -- if you meant three different
  cells, say which). Remaining open, unambiguous: US-BRK-high-esc
  (bump-HS direction, held, project-default, untouched by this round)
  and the original OS-APX-low front-ARB-vs-rear-ARB question above,
  which matrix v2 answered by making it situational (both levers
  listed) rather than picking one -- arguably resolved, but flagged in
  case you intended a single answer instead.
- [2026-07-27, PART A driver-level feedback weighting] The
  `driver_level_weighting` table (config/recommendations.json
  settings, driving_level 1-10 -> feedback_weight 0.6-1.5, neutral at
  level 5, default 1.0) is project-lead-elicited, not derived from any
  session data -- confirm the curve (linear, 0.1 per level, neutral
  midpoint) is the right shape, or state a different one.
- [2026-07-27, consistency-gate feedback override] Two elicited
  thresholds (config/recommendations.json settings.consistency_gate.
  feedback_override): `feedback_override_raw_min` = 4 and
  `feedback_override_scaled_min` = 4.0. Both project-lead-elicited,
  not data-derived -- confirm both numbers, and the driver
  feedback-scale semantics they rest on (+-2..3 = "clearly felt",
  +-4..5 = "approaching undrivable", recorded in the config comment
  and thesis_notes.md) are the intended reading before this override
  is exercised on a real (non-synthetic) recommendation.
- [2026-07-27, repair turn] Two follow-ups from the feedback-encoding
  repair:
  - The driver feedback-table caption (ui/views/outing_form.py,
    `scale_desc` label, "Scale: -5 undrivable understeer ... +5
    undrivable oversteer") still ends with "Placeholder — full
    description to be added per value." This is the canonical scale
    recording every rule's sign convention now formally rests on
    (`modules/recommendation.py` `VERDICT_EXPECTED_FEEDBACK_SIGN`,
    config/recommendations.json's `_comment_feedback_encoding`) --
    worth finishing properly (a one-line meaning for each of -5/-3/-1/
    0/+1/+3/+5, not just the current endpoints-and-midpoint shorthand)
    now that code/config both cite it as the source of truth. Tier C
    UI polish, not urgent, batch with the legend-readability pass.
  - `condition.min_feedback_abs` is 1 for every matrix rule (config/
    recommendations.json) -- verified during the repair turn that this
    means a magnitude-exactly-1 ("slight" on the recorded scale)
    complaint DOES fully corroborate a matching data verdict today
    (`_feedback_modulation`'s `abs(fb_value) < min_abs` is a strict
    `<`, so 1 clears a floor of 1). If the intent is that only >=2
    ("clearly felt" or stronger) should corroborate, this floor needs
    raising -- not changed here, since it's a calibration decision,
    not a bug fix; confirm the intended floor.

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

   [PHASE 1 DONE 2026-07-26: chair-identical axle Fz_f/Fz_r (static +
   aero + longitudinal transfer, estimate_vertical_loads) and the
   chair's own independent-per-axle wheel split (fz_fl/fr/rl/rr) --
   NOT the roll-stiffness-apportionment DOMAIN IMPROVEMENT above, which
   stays open, damper-validated, its own later sub-step. fy_f_norm_N/
   fy_r_norm_N (Fy_filt/fz axle) wired as the named consumer, surfaced
   read-only in Module 6 + the corner-details UI table (Fzf/Fzr median
   columns; fy_norm computed but not yet displayed, deferred, doesn't
   fit the panel width cleanly); nothing feeds _classify_corner.
   cog_height_m/track_width_front_m/track_width_rear_m are reviewer-
   supplied order-of-magnitude PLACEHOLDERS (0.30m, 1.66m, 1.64m,
   explicitly "NOT sourced, replace with team figure"); air_density is
   an L1-by-convention physical constant; lift_coeff=0.0 (aero term
   inert, documented zero-meaning, Cl sign convention inferred from the
   chair's own formula+comment, not yet empirically confirmed -- first
   real Cl entry must validate Fz rising with v^2 before trust).
   PHASE 2 (damper-derived Level 4 wheel loads, the roll-stiffness
   DOMAIN IMPROVEMENT split, aero-from-damper-v^2 regression, D_psi
   completion) remains open, unchanged by phase 1.]
c) ~~Level 3 sideslip: beta from log_gps_course (velocity vector) minus
   chassis heading estimate; replaces kinematic integration + washout.
   Validate against Module 2 output before switching default.~~
   [SHELVED 2026-07-26, two iterations, see thesis_notes.md: beta_gps
   (GPS-course minus gyro-integrated, drift-anchored heading) built and
   validated as estimate_sideslip_gps (validation-only, never wired to
   any consumer). Iteration 1 (time-linear drift allocation) gave a
   poorly-correlated, oversized result (r=-0.12, 51% per-corner sign
   agreement, a physically-impossible 9.34m implied antenna offset);
   root-cause diagnosed as a ~6deg/lap gyro scale-drift concentrated
   during cornering, mismatched by time-linear correction allocation.
   Iteration 2 (rotation-proportional allocation + measured +0.32s
   latency correction) passed its own falsifiable check on that
   diagnosis (implied antenna offset shrank 86% to 1.325m, physically
   plausible) but the decision-criteria metrics did not materially
   improve (r=-0.24, 52.2% sign agreement) -- NOT MET either iteration.
   REOPEN CONDITION: denser anchor data (a longer session and/or more
   straight-line sections than this single 4-lap file offers) --
   6 anchors across ~530s was diagnosed as the likely remaining limit,
   not the allocation scheme or latency, both already fixed. Kinematic
   beta (estimate_sideslip) remains production, untouched throughout.
   accuracy_levels.sideslip_angle carries this outcome as an inactive
   registry note (config/parameters.json).]
d) ~~Speed validation: log_gps_speed vs ecu_speed agreement report;
   promote speed source if GPS proves cleaner. log_gps_speed confirmed
   present in the real file but still not in channels.json -- lands
   with this consumer, not whitelisted speculatively ahead of it.~~
   [DONE 2026-07-26: log_gps_speed whitelisted (config/channels.json),
   isolated channels-direct comparison (diagnostics/inspect_gps_speed_
   validation.py), ecu_speed untouched as production source and
   pipeline time-anchor throughout. VERDICT (b), not a source switch:
   GPS speed retained as a permanent cross-check; k=1.01211 measured
   (origin-regression scale factor, tight across speed classes, k
   range 0.0057; residual spread collapses post-correction, raw median
   +0.506 m/s -> post-k +0.002 m/s; no slip sign-flip under braking or
   traction, arguing for a constant calibration factor over a slip
   artifact) as a CANDIDATE rolling-radius correction to ecu_speed's
   own conversion -- measuring k is not an estimator-input change,
   APPLYING it would be (v feeds Modules 2-5 + the WP5b(b) aero-Fz
   term). Application deferred pending a second-track reproduction
   check (k must hold up on a new data file, not just Dubai) plus its
   own re-derivation stop, both not yet done. accuracy_levels.speed
   carries this outcome as an inactive registry note (config/
   parameters.json). thesis_notes.md: "WP5b(d): GPS speed
   cross-validation (validation only)".]
e) After any of a-d lands: re-run the corner-distribution diagnostic
   and re-derive classification thresholds — they were tuned on
   Level 1 numbers and are not portable across accuracy levels.
~~f) Level 4 steering ratio lookup — replace the constant steering_ratio
   (Level 1, thesis limitation #3) with the digitised wheel-travel/
   stroke/ratio table (config/car_data.json: steering_ratio_table,
   source Steering.png). Consumer: prepare_vehicle_state's delta_f
   computation.~~ [DONE 2026-07-26, WP-B: modules/accuracy_resolution.py
   resolves steering_ratio to Level 4 from this table when
   config/car_data.json is present and the cap allows it, graceful L1
   fallback otherwise; diagnostic run, CS/stability thresholds
   re-confirmed unchanged (config/parameters.json derived_from fields),
   full session record in thesis_notes.md.]

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
10. **Aggregate force-balance Mz cross-check** is computable from
    existing axle Fy (chair's aggregate tier needs no wheel forces) —
    candidate diagnostic, not method. Found 2026-07-26 (study document
    §8): the chair's own force-balance Mz function has an aggregate
    fallback tier, `mz = lf*Fy_f - lr*Fy_r`, needing only axle-level
    lateral forces and CoG-to-axle distances — SetupTool's Module 4a
    already computes `Fy_f`/`Fy_r`. Unlike the chair's full per-wheel
    tier (needs per-wheel tyre forces this project doesn't have), this
    aggregate cross-check could be wired without new sensor data.

---

## ORDER OF EXECUTION

WP1 → WP2 → WP3 → WP4 → WP5 → WP6 → WP7. WP4 can be interleaved anytime.
Ideas only on explicit request. New data files from the user slot in as
validation passes for WP1 (rerun clustering on a second track before trusting
the tolerance default).