# Diagnostics

One-off read-only scripts used during development to produce methodology
evidence for the thesis. Not part of the app; run manually from the project
root (`python diagnostics/<script>.py`) against the sample data.

REWRITTEN FROM SCRATCH 2026-09-24 (WP-CLEAN Phase 1, hard pass). Every
script below cleared one of seven load-bearing tests (K1-K7), verified
against the actual current state of config/modules/docs/thesis_material_index.md
-- not against this file's own prior claims. 37 scripts that do not clear
any of K1-K7 moved to `diagnostics/_attic/` (git mv, reversible, nothing
deleted) even where their finding is recorded in thesis_notes.md -- the
record is the deliverable, the script was scaffolding.
`diagnostics/_attic/` awaits a final user decision (keep/delete) at
orphan-branch time; do not delete from it without that explicit decision.

Plot directories: five are gitignored (`plots/`, `plots_step2/`,
`plots_decision_layer/`, `plots_gated_kinematic/`, `plots_yaw_damping/`), seven are tracked (`plots_deepening/`,
`plots_fz_integration/`, `plots_ground_truth/`, `plots_ls_evidence/`,
`plots_metrology/`, `plots_threshold_investigation/`, `plots_v3/`) --
a historical accident, deliberately left as-is (WP-CLEAN Phase 2,
2026-09-24). The orphan branch curates what ships.

Categories:
- **K1** config-cited provenance -- filename (or verified descriptive
  citation) appears in a live config `derived_from`/`_comment` backing a
  CURRENT value. A stale or superseded citation does not qualify.
- **K2** golden-value generator or the frozen pass-1 validation baseline.
- **K3** live figure source -- generates a figure the thesis or
  supervisor materials actually use (verified either by exact filename in
  docs/thesis_material_index.md, or by reading the script's own output
  path against a PNG the index lists).
- **K4** working smoke test (`smoke_test_*.py`) -- regression tooling,
  not investigation.
- **K5** named census/measurement tooling still in active use.
- **K6** imported by another K1-K5 keeper (verified against the actual
  `from diagnostics.X import` graph, not assumed).
- **K7** regenerates committed repo content, or is the sole tool for a
  recurring PLAN.md checklist task (new-data-file channel discovery).

RELOCATED (2026-09-24, WP-CLEAN relocation mini-package):
sideslip_ekf_dugoff.py and sideslip_ekf_pacejka.py -- both were
production dependencies of modules/tyre_fit_auto.py living here by
historical accident (their own module docstrings claimed "diagnostics-
only, no modules/ consumer", which was already stale). Moved to
modules/sideslip_ekf_dugoff.py and modules/sideslip_ekf_pacejka.py;
every import and reference across the repo updated in the same commit.
No longer part of this directory's own inventory -- 90 scripts remain
here (53 keepers + 37 in diagnostics/_attic/). [Count as of 2026-09-24;
the per-section lists below are the current inventory.]

REMOVED (2026-10-17, Class B rebuild, author ruling "delete including
evidence"): the Dugoff path. modules/sideslip_ekf_dugoff.py and
modules/tyre_model.py are gone; with them fit_dugoff_first_pass.py,
fit_dugoff_pass4_refit.py, inspect_ekf_pass1_rQ_sweep.py,
inspect_nis_tyre_mismatch_gate.py (K1 entries below struck), the _attic
scripts inspect_ekf_dugoff_sanity_checks.py,
inspect_tyre_fit_auto_acceptance.py, inspect_tyre_variant_comparison.py,
and the local manifests fit_dugoff_first_pass_manifest.json /
fit_dugoff_pass3_refit_manifest.json. The config provenance strings that
cited them now say so (git history keeps the code).

## K1 -- config-cited provenance (26)

- ~~**fit_dugoff_first_pass.py**~~ -- REMOVED 2026-10-17 (Dugoff path).
- ~~**fit_dugoff_pass4_refit.py**~~ -- REMOVED 2026-10-17 (Dugoff path).
- **inspect_beta_gps_validation.py** -- cited in config/parameters.json's
  gps_course_latency_s_derived_from.
- **inspect_c3_leaked_windows.py** -- cited in config/parameters.json's
  kerb_investigation_reference.
- **inspect_corner_distribution.py** -- cited in config/parameters.json's
  STRONG/MODERATE_CSF/CSR derived_from fields. Also K3.
- **inspect_cs_floor_candidate_validation.py** -- cited in
  config/parameters.json's cs_min_window_s/cs_max_window_m provenance.
- **inspect_cs_max_window_locality_sizing.py** -- cited in
  config/parameters.json's `_comment_cs_max_window_m`.
- **inspect_cs_window_floor_derivation.py** -- cited in
  config/parameters.json's cs_linear_slip_threshold_rad_derived_from.
- **inspect_damper_motion_sign_and_threshold.py** -- cited in
  config/decision_frame.json's damper_motion.rate_threshold_mm_s_derived_from
  and min_valid_fraction_derived_from.
- **inspect_deepening_phase2_fr_correction.py** -- cited in
  config/channels.json's log_dms_dam_fr channel_corrections derived_from.
- ~~**inspect_ekf_pass1_rQ_sweep.py**~~ -- REMOVED 2026-10-17 (Dugoff
  path); tyre_fit_auto._comment names it as the origin of the sweep grid.
- **inspect_kerb_wheel_speed_spikes.py** -- cited in
  config/parameters.json's kerb_investigation_reference.
- **inspect_kerb_severity_census.py** -- cited in
  config/decision_frame.json's kerb_blowoff_evidence.derived_from (WP-
  ELICIT Phase C3, 2026-09-24). Previously undocumented gap, closed here.
- **inspect_ls_max_window_locality_sizing.py** -- cited in
  config/parameters.json's max_window_m_derived_from.
- **inspect_ls_window_floor_derivation.py** -- cited in
  config/parameters.json's min_slip_span_derived_from (current 0.016
  value, supersedes the old 0.004).
- **inspect_metrology_phase1_sensitivity.py** -- cited in
  config/parameters.json's verdict_stability_margin derived_from.
  Adapted 2026-10-05 (author ruling): steering input = +/-1 % on the
  table's wheel-angle column (the ratio column is no longer read since the
  steering fix); results to diagnostics/results_metrology_20261005/.
  inspect_metrology_phase1_analysis.py reads the same folder.
  [2026-10-11: METROLOGY_RUN_TAG selects the run folder (default 20261005); the cs_margin re-measure on the corrected estimator wrote results_metrology_20261011/ and plots_metrology_20261011/ (analysis via inspect_metrology_phase1_analysis.py with the same tag).]
- **inspect_native_channel_rates.py** -- cited in config/parameters.json's
  `_comment_grid_rate`.
- ~~**inspect_nis_tyre_mismatch_gate.py**~~ -- REMOVED 2026-10-17 (Dugoff
  path); nis_gate._comment records it as the gate's prototype.
- **inspect_tpms_pressure_cornering_phase.py** -- cited in
  config/channels.json's tpms_press_fl note and config/decision_frame.
  json's tyre_pressure_target derived_from (WP-ELICIT Phase B2,
  2026-09-24). Previously undocumented gap, closed here.
- **inspect_saturation_coverage.py** -- keep-reason void (tyre_model_fit
  removed 2026-10-17), cleanup candidate. Was cited by description in
  config/parameters.json's tyre_model_fit._comment: "the WP-N0
  saturation-coverage diagnostic's own candidate-threshold list."
- **inspect_v3_brake_phase_stability.py** -- cited in
  config/parameters.json's `_comment_stab_phase_no_braking_floor_bar`
  (stab_phase_no_braking_floor_bar=3.0). Previously undocumented gap,
  closed this pass.
- **inspect_v3_nis_gate_failure.py** -- cited in config/parameters.json's
  nis_gate threshold_derived_from.
- **inspect_v3_wheel_speed_census.py** -- cited in
  config/parameters.json's ratio_max_deviation_derived_from.
- **inspect_wheel_speed_guard_before_after.py** -- cited in
  config/parameters.json's window_s_note/std_min_kmh_note.
- **inspect_yaw_stability_b2.py** -- cited in config/parameters.json's
  stab_neg_thresh_Nm_per_deg derived_from.
- **sideslip_kalman_observer.py** -- cited in config/parameters.json's
  yaw_inertia_kalman note and accuracy_levels node.

## K2 -- golden generator / frozen pass-1 baseline (1)

- **inspect_pass1_final_validation.py** -- THE frozen pass-1 EKF
  validation baseline; do not delete without a deliberate decision to
  retire it. Also K3. [2026-10-17: its Dugoff imports are wrapped -- it
  exits with an explanatory line, since the observer it validates was
  removed; the record is pass1_final_validation_manifest.json.]

## K3 -- live figure sources (18 not already listed above)

- **capture_wp_perf_reference.py** -- WP-PERF byte-identity baseline
  capture, cited docs/thesis_material_index.md Ch.6. Also K5, K6 (imports
  inspect_frame_stage2_parity.py).
- **compare_wp_perf_reference.py** -- byte-identity comparison companion
  to the above, same Ch.6 citation. Also K5.
- **generate_decision_layer_figure.py** (keep-reproduces) -- generates
  diagnostics/plots_decision_layer/six_stage_flow and lever_coverage_table
  (PNG + PDF each) from live config, cited docs/thesis_material_index.md
  Ch.5. Fixed 2026-09-29 to compute rather than state (rank-based
  display_top_n, per-class change_time, held-secondary routes,
  feedback-only entry count, tyre bands, speed_qualified_effect cells);
  last regenerated 2026-09-29, the 2026-09-23 set kept as *_20260923
  (thesis_notes.md "[2026-09-29, figures regenerated]"). Run:
  `python -m diagnostics.generate_decision_layer_figure`.
- **inspect_dubai_wheel_load_validation.py** -- named in
  docs/thesis_material_index.md Ch.4 as underlying the wheel-load
  figures.
- **inspect_frame_candidate_census.py** -- cited
  docs/thesis_material_index.md Ch.5, re-run at every decision-layer
  phase boundary. Also K5, K6 (imports inspect_frame_stage2_parity.py).
- **inspect_fz_before_after.py** -- generates
  diagnostics/plots_fz_integration/, cited
  docs/thesis_material_index.md Ch.4.
- **inspect_metrology_phase1_analysis.py** -- code-verified: writes
  `{session}_margin_distribution.png`, exact match to the Ch.3 figure
  docs/thesis_material_index.md cites (script not named in index prose).
- **inspect_metrology_phase1_corner_map.py** -- code-verified: writes
  `{session}_corner_sequence_marginal.png`, exact match to the Ch.3
  figure the index cites.
- **inspect_metrology_phase3_rear_residual.py** -- code-verified: writes
  `{name}_residual_vs_speed.png`, exact match to the Ch.4 figure the
  index cites.
- **inspect_pipeline_wall_times.py** -- cited
  docs/thesis_material_index.md Ch.5/6. Also K5, K6 (imports
  inspect_frame_stage2_parity.py).
- **inspect_prc_v3_sample_rates.py** -- cited
  docs/thesis_material_index.md Ch.3 (damper-channel-capability finding).
- **inspect_step2_chair_plots.py** -- generates the 28 chair-comparable
  PNGs in diagnostics/plots_step2/, cited
  docs/thesis_material_index.md Ch.2/3. [2026-10-17: its ekf_pass_1
  (Dugoff) source was removed, it renders the kinematic set only; kept
  because other scripts import its helpers.]
- **inspect_v3_fr_gauge_forensics.py** -- cited
  docs/thesis_material_index.md Ch.3.
- **inspect_v3_reconstruction_ground_truth.py** -- named in
  docs/thesis_material_index.md Ch.4 as underlying the wheel-load
  figures; also cited directly in modules/wheel_loads.py:479's own
  docstring.
- **inspect_v3_sawtooth_mechanism.py** -- cited
  docs/thesis_material_index.md Ch.2. Also K6 (imports
  inspect_step2_chair_plots.py).
- **inspect_v3_wheel_load_comparison_figure.py** -- code-verified: writes
  wheel_load_comparison_C12_lap8.png, exact match to the Ch.4 figure the
  index cites (script not named in index prose).
- **inspect_v3_wheel_load_reconstruction_figure.py** -- code-verified:
  writes wheel_load_reconstruction_C12_lap8.png, exact match to the Ch.4
  figure the index cites (script not named in index prose).
- **inspect_v3_wheel_load_showcase.py** -- generates
  wheel_load_showcase_fastest_lap8.png and heavy_braking_zoom.png, cited
  docs/thesis_material_index.md Ch.4.

## K4 -- working smoke tests (6)

- **smoke_test_cache_sidecar_analyse_contract.py** -- WP-CACHE
  Analyse-button contract, end to end on real data.
- **smoke_test_corner_trace_dialog.py** -- CornerTraceDialog/
  LapTraceDialog headless smoke test.
- **smoke_test_decision_frame_phase_d.py** -- D1/D2/D6 top-line/tail
  rendering rules against real and synthetic candidates.
- **smoke_test_decision_frame_widget.py** -- Decision Frame section
  widget binding.
- **smoke_test_measurement_points_widget.py** -- splitter/diffuser
  measurement-points form widget.
- **smoke_test_settings_view.py** -- SettingsView Section 4 cost-function
  weights, restart-persistence check.
- **smoke_test_kerb_bridge.py** (2026-09-30) -- kerb-gap display bridge in
  CornerTraceDialog/LapTraceDialog on one cached outing (sidecar read
  read-only): bridges only with the flag on, analysis arrays
  byte-identical before and after rendering.

## K5 -- named census/measurement tooling (7 not already listed above)

- **inspect_brake_bias_channel_identity.py** -- WP-ELICIT HANDOFF item 5
  (2026-09-25, Phase D brake-bias channel identity): recomputes percent-
  front from raw brake pressure during real braking events and
  correlates it against every Frame-Stage-2 Phase 2 candidate channel,
  both real sessions. Re-run whenever a new session or a resolved
  candidate channel warrants re-checking the identity.
- **inspect_pipeline_sidecar_size.py** -- WP-CACHE sidecar size/load-time
  measurement, both real sessions against the acceptance gate.
- **inspect_gated_kinematic_run.py**, **inspect_gated_kinematic_cs.py**,
  **inspect_gated_kinematic_drift.py**, **inspect_gated_kinematic_scale_sign.py**
  (2026-09-28, keep-reproduces) -- gated-kinematic sideslip evidence
  (run/cache, CS comparison, causal drift-vs-time, gyro-scale + sign),
  both real sessions. Keep-reason: provenance for the 2026-09-29
  gated-kinematic fail-ruling (thesis_notes.md "Gated-kinematic sideslip
  diagnostic"; ekf_auto_pacejka stays production). Plots in
  `plots_gated_kinematic/` (gitignored).
- **inspect_new_laps_cs_sweep.py** (2026-09-29, keep-reproduces) -- new-
  session checklist (rate guard, corner clustering, bracket gaps vs
  cs_max_window_m, lever-channel content census), CS_ratio extremity/
  flapping baselines and the ten-row CS smoothing/persistence sweep over
  all nine sessions. Keep-reason (updated 2026-09-30): provenance of the
  closed CS diagnostic arc; its caches and flapping metrics are the base
  for PLAN.md EXPERIMENT PLAN (b), the upstream-filter sweep; also the
  reusable tool for the PLAN.md new-data-file checklist on the next file.
  Modes: channels <key>, compute <key>, report. Records: thesis_notes.md
  "New-session checklist", "CS_ratio extremity and flapping baselines",
  "CS smoothing/persistence sweep" (2026-09-29). Caches, logs, figure in
  `plots/new_laps/` (gitignored).
- **inspect_cs_value_lens.py** (2026-09-29, keep-reproduces) -- value-
  level CS_ratio lens on the sweep caches: NBR outing mapping (headers +
  logged rake), full-lap traces, within-corner roughness/autocorrelation
  vs window length, lap-to-lap reproduction, demand-gated band shares and
  axle gap, NBR A (outing 21) vs B (outing 29), fold/loop on worst
  windows. Imports inspect_new_laps_cs_sweep (caches, session list).
  Keep-reason (updated 2026-09-30): provenance of the closed CS diagnostic
  arc and the A-vs-B known-answer check, which EXPERIMENT PLAN (b)'s
  battery reuses. Modes: mapping; traces (full-lap front/rear CS_ratio vs
  lap distance per session, one line per analysis lap ->
  `plots/cs_value_lens/trace_<key>.png`); supplement <key>; smooth, util,
  ab, foldloop (worked_corner_dubai.png, nbr_a_vs_b.png, fold_loop.png).
  Records: thesis_notes.md "NBR outing mapping and full-lap CS traces" and
  "CS_ratio value lens" (2026-09-29). Outputs in `plots/cs_value_lens/`
  (gitignored).
- **inspect_cs_readout_evidence.py** (2026-09-29, keep-reproduces) --
  evidence and prototype for the CS read-out decision: slip-vs-force lag
  (both beta sources, lag-only synthetic tyre), run-scale fold/loop with
  lag correction and healthy-corner specificity control, sawtooth
  window-start repair (diagnostic copy of compute_cs_for_axle with parity
  check against production), four read-outs (worst lap, repeat floor,
  pooled median, pooled p25) scored on the pre-registered criteria, and a
  census with the pooled read-out fed to the frame in memory plus TC score
  decomposition. Imports inspect_new_laps_cs_sweep and
  inspect_cs_value_lens. Keep-reason (updated 2026-09-30): provenance of
  the closed CS diagnostic arc (read-out decision taken: variant A); its
  phase slicing and read-outs are imported by inspect_cs_decision_table,
  which WP-HARDEN re-runs. Modes: supp2 <key>, sawtooth <key>, sawrender
  <key> (2026-09-30: production vs repair-active side-by-side trace sets
  with window-start jumps marked and the deepest per-lap spikes listed,
  JST 29 and US-v2; decision input for the sawtooth ruling of
  2026-09-30, repair stays PARKED); lag,
  foldrun, sawreport, parity, readout, census. Records: thesis_notes.md
  "Slip-vs-force lag check",
  "Fold/loop at run scale", "Sawtooth window-start repair", "Pooled
  read-out prototype", "Synthesis for the CS read-out decision"
  (2026-09-29), "Sawtooth repair side-by-side render" (2026-09-30).
  Outputs in `plots/cs_readout_evidence/` (gitignored).
- **inspect_cs_decision_table.py** (2026-09-29, keep-reproduces) -- final
  CS read-out round: pooled-p25 thresholds by the recorded procedure
  (rules checked against the recorded Dubai derivation), cs_margin
  analogue from a re-run of the recorded +/-1 % sensitivity set (reuses
  inspect_metrology_phase1_sensitivity's SESSIONS/build_variant and
  run_one's call sequence), display-layer hardening prototype (evidence
  floor sweep, session-level lever-conflict cards), and the A0 / A / B
  decision table with the sawtooth sensitivity line. Imports
  inspect_new_laps_cs_sweep and inspect_cs_readout_evidence. Keep-reason
  (updated 2026-09-30): provenance of the closed CS diagnostic arc, and
  live tooling -- WP-HARDEN (variant A, approved, deferred) re-measures
  its census with part2 / table at implementation; the battery is reused
  by EXPERIMENT PLAN (b). Modes: sens, thresholds, part2, margin, table,
  render, hardened. hardened (2026-09-30, WP-HARDEN implementation): the
  census on the production display path (modules/decision_frame.
  harden_display_rows, per-member floor, action-key conflicts), before
  vs after per session; record thesis_notes.md "WP-HARDEN: CS display
  hardening implemented". part2 / table stay as the prototype's frozen
  per-card, increase/decrease-only record. The single render mode draws, per session, the plan-B trace
  overlays (per-lap CS_ratio thin, pooled p25 per phase heavy, fixed
  -3.5..1.2 axes -> `render/planb_trace_<key>.png`) and worst-lap vs
  pooled-p25 per corner (`render/worstlap_vs_p25_<key>.png`), plus the
  references-only C4 provenance sheet (`render/C4_provenance.txt`); value
  level only, no thresholds or verdicts. Records: thesis_notes.md
  "Pooled-p25 thresholds by the recorded procedure", "Hardened
  production, display layer", "Three-way decision table", "Decision
  synthesis" and the render addendum (2026-09-29). Outputs in
  `plots/cs_decision_table/` (gitignored).
- **inspect_imu_offsets.py** (2026-09-30, keep-reproduces) -- PLAN.md
  EXPERIMENT PLAN (a): straight-line offsets of yaw rate, ay and the
  kinematic residual ay/v - r per lap and session, heading closure per lap
  (bias vs scale), degree 0/1/2 BIC gate against time and against
  sclu_temp_rate_z, g-sensitivity on widened corner windows, and the
  in-memory correction's effect on the gated-kinematic drift and the
  straight-line kinematic-vs-EKF gap. Eight sessions (sweep mapping minus
  US-v1); Part 4 on Dubai and v3 from the gated-kinematic caches. Imports
  inspect_new_laps_cs_sweep (sessions), gated_kinematic_common,
  inspect_gated_kinematic_drift and inspect_gated_kinematic_scale_sign.
  Modes: census <key> (raw channel-name scan), compute <key> (slim cache
  incl. the raw temperature block), report (Parts 1-3), consequence (Part
  4), precheck (liveness of MRR_EgoYawRate and sclu_acc_y against the
  production channels, one-lap overlays `precheck_yaw_<key>.png`, header
  presence in the unmapped NBR files); (a2): a2compute <key> (production
  state + raw sclu_acc_y, sclu_roll_rate, a_roll, GPS speed channels,
  corner speed classes), referee (steady-state accelerometer vs r v, GPS
  speed k, gyro/speed split, roll leakage), ratio (post-hoc median-ratio
  referee per stratum), collapse (single and pairwise candidate
  corrections on the gated-kinematic drift, Dubai / v3 / JST 29);
  rollcheck / rollcheck2 (2026-09-30, the (a3) gate: roll-integral closure
  decomposed into constant bias, per-corner reproducibility and, post-hoc,
  yaw cross-coupling; leakage constant recomputed; figure
  roll_closure.png); attcompute <key> / attitude (2026-09-30, sensor tilt
  vs static attitude: ride-height/rake channels read from all eight files
  with header-unit conversion, matched-speed attitude, yaw-leak angle for
  all eight, ay offsets, per-logger models; figure tilt_vs_attitude.png).
  Keep-reason: evidence for the IMU-offset and
  turn-proportional-error decisions and for the (a3) gate answer; any
  correction package re-runs it. Records: thesis_notes.md "IMU offsets,
  Part 1" to "Part 5", "Second IMU sources, liveness pre-check", "(a2)
  Part 1" to "Part 4", "Roll closure check: the roll-rate channel
  carries yaw cross-coupling" and "Sensor tilt vs static attitude, eight
  sessions" (2026-09-30). Outputs in `plots/imu_offsets/`
  (gitignored).
- **inspect_cs_upstream_sweep.py** (2026-09-30, keep-reproduces) -- PLAN.md
  EXPERIMENT PLAN (b): CS filtering at upstream insertion points (yaw rate
  + ay before the sideslip fit, speed before fit and alpha, EKF beta
  before alpha) plus the (a2)-floor reference rows (speed x1.01, ay x1.02)
  through the production chain in memory; survival battery, NBR A-vs-B
  jackknife resolvability, pooled lap-pair switch rate, verdict shift,
  floor test. Own script because the variants must feed modified RAW
  channels through fit_session_pacejka. Imports inspect_new_laps_cs_sweep,
  inspect_cs_readout_evidence, inspect_cs_value_lens. Modes: compute <key>,
  ecompute <key> <variant>, tier2 <key> <variants>, score, confirm
  <variants>, render (baseline vs D1 vs E standard trace sets on JST 29,
  US-v2, MLA via trace_render.py; thesis_notes.md "(c) Part 2"). Tiered per the author's amendment (Tier 1 Dubai, JST 21/29,
  US-v2, v3 kept; Tier 2 Le Mans, CDR, MLA for survivors only).
  Keep-reason: evidence for the (b) exit ruling; any filter proposal
  re-runs it. Records: thesis_notes.md "(b) Part 1" to "Part 3"
  (2026-09-30). Outputs in `plots/cs_upstream/` (gitignored).
- **inspect_yaw_pooling.py** (2026-09-30, keep-reproduces) -- PLAN.md
  EXPERIMENT PLAN (c): yaw rate pooled per corner on a 1 m track grid over
  the analysis laps, p10-p90 envelope, excess/deficit flags beyond a
  noise-derived margin, steering-unwinding check 150-300 ms after each
  flag with a same-position control. Screening JST 29, US-v2, MLA; Dubai
  for the C3 cross-reference. Imports inspect_new_laps_cs_sweep,
  gated_kinematic_common and core/yaw_pooling. Refactored 2026-09-30
  ("(c)-display"): grid, envelope, margin and flags now come from
  core/yaw_pooling.py, the app's yaw panel code, called with the script's
  own pre-registered constants; only the counteraction test stays local.
  Outputs checked byte-identical to the pre-refactor script on all four
  cached sessions. Modes: compute <key>, report. Keep-reason: evidence for
  the (c) ruling, and the reference the app panel was lifted from. Records:
  thesis_notes.md "(c) Part 1", "(c)-display implemented". Outputs in `plots/yaw_pooling/`
  (gitignored). The upstream-sweep script's `render` mode (Part 2 retro
  renders) is listed under inspect_cs_upstream_sweep.py.
- **inspect_event_coincidence.py** (2026-09-30, keep-reproduces) -- PLAN.md
  EXPERIMENT PLAN (f): per corner-lap events in three channels (yaw
  excess/deficit from core/yaw_pooling via inspect_yaw_pooling.analyse; CS
  worst-phase median below the session p10, front/rear; stability
  worst-phase median below the session p10), coincidence counts against a
  within-corner lap-shuffle null (1000 shuffles, fixed seed), steering
  reaction as an attribute, the carried spike/NBR C1/Dubai C3-C4
  questions. Reads the new_laps and yaw_pooling caches only; imports
  inspect_yaw_pooling and inspect_frame_stage2_parity (classifier entry
  check). Mode: report. Keep-reason: evidence for the (f) exit ruling.
  Records: thesis_notes.md "(f) Part 1" to "(f) Part 5". Outputs in
  `plots/event_coincidence/` (gitignored).
- **inspect_nbr_c1_dip.py** (2026-09-30, referenced) -- follow-up to (f):
  raw inputs (ay, yaw rate, psidd, steering, speed, az with kerb samples,
  four damper travels) and the CS window at each lap's front CS minimum
  across 600-640 m at NBR C1, JST 21 and JST 29. Keep-reason: cited by
  PLAN.md PARKED "NBR C1 EKF entry lag"; any proposal on kerb samples
  inside CS windows re-runs it. Records: thesis_notes.md "NBR C1
  fixed-position CS dip: kerb strike at the apex". Outputs in
  `plots/nbr_c1_dip/` (gitignored).
- **inspect_cs_kerb_edge.py** (2026-09-30, keep-reproduces) -- share of
  per-corner-lap CS_ratio minima within 3 m of a kerb-mask edge vs a
  uniform-position chance rate, eight census sessions, front/rear, all
  minima and deep tail; plus whether each minimum's CS window holds kerb
  samples (fix coverage). Reads the new_laps and imu_offsets caches.
  Keep-reason: sizes PLAN.md EXPERIMENT PLAN (g) (kerb samples inside CS
  windows); re-run at its counterfactual step. Records: thesis_notes.md
  "CS minima at kerb-mask edges, eight sessions". Outputs in
  `plots/cs_kerb_edge/` (gitignored).
- **inspect_cs_kerb_window.py** (2026-09-30, keep-reproduces) -- PLAN.md
  EXPERIMENT PLAN (g) step 0: in-memory CS re-estimation, BASE / K (kerb
  samples excluded inside windows) / KS (K + sawtooth start repair), one
  pass per axle from the new_laps, value-lens supp and imu_offsets caches;
  exit legs (near-edge enrichment, new deep dips, kerb-adjacent battery)
  and the stacked standard trace sets. Imports inspect_cs_kerb_edge,
  inspect_cs_readout_evidence, inspect_new_laps_cs_sweep, inspect_cs_value_lens,
  trace_render. Modes: compute <key>, score tier1|all. Keep-reason:
  evidence for the (g) exit ruling. Records: thesis_notes.md "(g) step 0:
  kerb samples excluded inside CS windows". Outputs in
  `plots/cs_kerb_window/` (gitignored).
- **inspect_kerb_mask_variants.py** (2026-10-01, keep-reproduces) --
  kerb-detection variants for the PARKED open limitation "CS minima
  cluster at kerb-mask edges": Part 1 strike disturbance envelope (ay,
  yaw rate, az past the az-threshold mask, cubic-trend residual, null
  control), variant masks A/B/C/D, K re-estimation under each and the
  own-edge enrichment legs, stacked standard trace sets. Parses the raw
  files (slim cache); imports inspect_cs_kerb_edge (its corner_laps gained
  an optional kerb= override for this), inspect_cs_kerb_window,
  inspect_cs_readout_evidence, trace_render. Modes: compute <key>, part1,
  masks <key>, score tier1|all; window-reach check: reach tier1, trunc
  <key> (no-crossing windows T), score_t tier1. Keep-reason: evidence for
  the closure of the kerb-edge limitation, cited by the PARKED limitation
  entry. Records: thesis_notes.md "Kerb-detection variants, Part 1" to
  "Part 4", "Window-reach check: CS windows across masked kerb gaps",
  "Kerb-edge association: closed as characterised".
  Outputs in `plots/kerb_mask_variants/` (gitignored).
- **inspect_tyre_prior.py** (2026-10-01, keep-reproduces) -- EXPERIMENT
  PLAN (d) step 0: production per-session Pacejka fits plus the arrays they
  fit (EC2 bit-exact reproduction), pooled priors (P221, LOSO, ALL8,
  EW221), Option B (one closed-form force scale per session and axle)
  through the production fit chain via an in-memory _fit_axle_pacejka
  replacement, criteria K1-K5, K6 via the unchanged (b) scorer
  (inspect_cs_upstream_sweep score/confirm pointed at a copy), standard
  trace set. Modes: compute <key>, prior, bcompute <key>, analyse, score,
  render. Keep-reason: evidence cited by the (d) closing ruling (author,
  2026-10-01) and by the estimator-chapter finding "Per-session tyre-curve
  fitting, experimentally vindicated against a pooled baseline". Records:
  thesis_notes.md "(d) tyre-curve baseline prior: proposal, rulings,
  feasibility", "(d) step 0: pooled prior with per-session scale", "(d)
  closure".
  Outputs in `plots/tyre_prior/` (gitignored).
- **inspect_rear_curve_bias.py** (2026-10-02, keep-reproduces, pending the
  author's Q2d ruling; Q3 may reuse it) -- PHASE 1 PLAN Q2d: production /
  low-kappa / q90 Pacejka fits per axle on the (d2) production fit cache,
  curve deltas with lap bootstrap, the (e) Option-2 residual-vs-kappa
  signature, curve and residual renders. Imports the (d2) harness. Outputs
  in `plots/rear_curve_bias/`. Env Q2D_BOOT overrides the bootstrap count
  for smoke runs only.
- **inspect_phase1_q3.py** (2026-10-02, keep-reproduces, CONFIRMED by the
  author 2026-10-02: backs the Q3/Q4 and G2 closures) -- PHASE 1 PLAN Q2c + Q3: G0 GPS census, G1
  gradient map, G2 corner anchor (unscored), V1 / V2 tailoring variants and
  the Q4 courtroom ((ii) rake, (iii) per-phase driver statement, (v)
  coherence) with CS renders; rules frozen in its header. Imports the (d2)
  harness and its caches. Outputs in `plots/phase1_q3/`; modes g0 | g1 |
  g2 | v1 | v2 | q4 | all.
- **inspect_rake_ab_anchor.py** (2026-10-02, keep-reproduces, CONFIRMED by
  the author 2026-10-02: backs the A/B anchor characterisation) -- PHASE 1 PLAN Q3
  ruling 3 (reframed): NBR A (outing 21) vs B (outing 29) change census
  over all channel blocks, C1 / C4 / C10 corner traces (speed, a_y, steer,
  yaw, brake, GPS line offset, alpha_f / peak, CS), consistency and
  phase-flipping sign tests. Imports the (d2) harness. Outputs in
  `plots/rake_ab_anchor/`; modes census | corners | coherence | all.
- **inspect_g2_anchor_validation.py** (2026-10-02, keep-reproduces,
  CONFIRMED by the author 2026-10-02: backs the GPS-anchor closure) -- PHASE 1 PLAN Q3 extended
  package: Gate A on g2_repair2.json, validation battery (sign/phase,
  latency +/-0.05 s, SE vs speed, kerb, lap-count, cross-session), Gate B,
  gated first external beta read (Part 3). Imports inspect_phase1_q3 and
  the (d2) harness. Outputs in `plots/g2_anchor_validation/`.
- **inspect_seed_sweep.py** (2026-10-02, keep-reproduces, Q-SEED CLOSED
  2026-10-02: backs the closure and the supervisor answer) -- PHASE 1 PLAN Q-SEED: seed-construction
  variants S0 / S1a / S1b / S2 / S3a / S3b / S4w / S4l, per-session offsets
  by the (a) method, metrics M1-M4 (plus the labelled post-hoc M1c / M3c),
  beta / fit-cloud renders, downstream CS chain for escalated variants.
  Backs the supervisor's corner-switching question (S2/S3). Imports the
  (d2) harness and inspect_imu_offsets. Modes sweep | posthoc | down |
  report. Outputs in `plots/seed_sweep/`.
- **inspect_last_push.py** (2026-10-02, keep-reproduces, CONFIRMED by the
  author 2026-10-03: backs the Q-PUSH closure and the K2 exhibit) -- PHASE 1
  PLAN Q-PUSH "THE LAST PUSH": P1
  steering-derived steady-state beta + understeer gradient + anchored seed,
  P2 despike + robust losses, P3 fit-population sweep, P4 spectrum-matched
  filters, P5 EKF Q-sweep, P6 stacked chains, gate and all-18 escalation.
  Imports the (d2) harness, inspect_imu_offsets and inspect_seed_sweep
  (which therefore also has a dependency reason). Modes p1-p6 | gate | esc.
  Outputs in `plots/last_push/`.
- **inspect_recover_rr.py** (2026-10-03, keep-reproduces, CONFIRMED by the
  author 2026-10-03: the reference for Q-CAL item 2 -- production's voltage
  path must reproduce its four offsets) -- PHASE 1 PLAN Q-RECOVER-2: NBR damper-gauge offsets
  recovered through the raw amplifier voltage referenced to the same car's
  Paul Ricard zero (R3), the characterised standing-rolling correction with
  leave-one-out (R1), the drift-as-weight decomposition (T2) and the
  reference checks (T3). Reads Q-SHARED-OFFSET fit.json, Q2 outputs and the
  v4_windows inventory maps. Modes run | render. Outputs in
  `plots/recover_rr/`.
- **inspect_v4_windows.py** (2026-10-03, keep-reproduces, CONFIRMED by the
  author 2026-10-03: backs the raw-gauge-voltage finding and the window
  verdicts) -- PHASE 1 PLAN Q-V4-WINDOWS (standing windows
  behind the two single-file V4 failures, exits CONTAMINATED WINDOWS / REAL
  GAP / EVIDENCE SPLIT) and Q-CHANNEL-INVENTORY (every channel in all 20
  files, liveness, consumed, load-adjacent pair test). Modes windows |
  inventory <key> | families. Outputs in `plots/v4_windows/`.
- **inspect_shared_offset.py** (2026-10-03, keep-reproduces, CONFIRMED by the
  author 2026-10-03: reproduces the NBR front-right recovery Q2 now builds on) -- PHASE 1 PLAN Q-SHARED-OFFSET: one
  shared damper-gauge offset per corner across the 16 NBR files (pooled
  straight-line fit with one aero term per corner, constant and season-trend
  variants), file-bootstrap uncertainty, shape-validity checks V1-V4, exit,
  renders vs the static fallback. Reads the Q2 cache / JSON. Modes
  prep <key> | fit | renders. Outputs in `plots/shared_offset/`.
- **inspect_wheelload.py** (2026-10-03, keep-reproduces, CONFIRMED by the author
  2026-10-03: backs the Q-WHEELLOAD record and the rewritten engineer question) -- PHASE 1 PLAN Q-WHEELLOAD: damper-gauge decode
  check (units, scale, shared constant, sign), per-corner wheel-load renders
  for all 20 files (NBR L4-rel with the Q2 offsets), fuel-drift overlay,
  setup-reference test, cross-file offsets. Reads the Q2 outputs
  (inspect_damper_offset_q2.py partB JSON) and the (d2) harness. Modes
  part0 | file <key> | summary. Outputs in `plots/wheelload/`.
- **inspect_pairing.py** (2026-10-03, keep-reproduces, CONFIRMED by the author
  2026-10-03: backs the Q-PAIRING verdicts and the position-distinctive exhibit) -- PHASE 1 PLAN Q-PAIRING: per-site window contents at
  the 22 position-locked CS dips (alpha-Fy lag, re-pairing, EKF-vs-kinematic
  slip gap, kerb / az / damper disturbance), global in-corner lag
  distribution, synthetic lag-vs-depth test, per-site A / B / C verdicts.
  Imports inspect_last_push, inspect_stiffness_cs and inspect_chair_audit.
  Modes sites | synth. Outputs in `plots/pairing/`.
- **inspect_kerb_filter_decay.py** (2026-10-03, keep-reproduces, CONFIRMED by
  the author 2026-10-03: derives kerb_smear_grade_s; referenced from PLAN.md
  Q-CAL item 1) -- Q-CAL-1
  measurement: kerb-strike smear through the CS chain's 2 Hz filtfilt past
  the kerb-mask edge (line-bridged counterfactual, null-control noise band,
  per-event decay, CS windows touched), and the spike-coincidence count
  (mode `coincidence`, reads only its own records and the (d2) fit caches),
  the position-locked site check (mode `sites`) and the options 4 + 0
  verification on the Tier-1 sessions (mode `verify`).
  Its decay records feed the grade-zone width of the option-4 package.
  Imports inspect_last_push and the (d2) harness. Outputs in
  `plots/kerb_filter_decay/`.
- **inspect_nbr_zero_entry_verify.py** (2026-10-03, keep-reproduces, pending
  the author's confirmation; referenced from PLAN.md Q-CAL item 2) -- Q-CAL-2
  acceptance: the 16 NBR files through the production parser with the
  event zero entries -- entry status, damper share, gauge-vs-voltage map
  re-fit on the corrected channels against the production-parsed Paul
  Ricard reference, V1-V4 on the production loads. Imports
  inspect_recover_rr (frozen reference, read only), inspect_v4_windows
  (_scan) and inspect_shared_offset. Outputs in `plots/nbr_zero_entry_verify/`.
- **inspect_fit_ma_calibration.py** (2026-10-03, keep-reproduces, ruled by
  the author 2026-10-03: the record of the rejection) -- Q-CAL-3 calibration
  pass, the chair's 0.15 s moving average on the tyre-fit inputs vs
  production's 2 Hz inputs on US-v2, v3, JST 29 (frozen frame in its
  header; verdict HARM, MA rejected). Mode `metrics` re-renders metrics.png
  from results.json without recomputation. Imports inspect_last_push,
  inspect_seed_sweep and the (d2) harness; its harness pattern is reused by
  inspect_fit_k2_calibration.py. Outputs in `plots/fit_ma_calibration/`.
- **inspect_fit_decisive.py** (2026-10-04, keep-reproduces, pending the
  author's ruling; referenced from PLAN.md Q-FIT-DECISIVE) -- the decisive
  fit-population pass: speed window, smear zone, large-Fx tails and
  survivor combinations vs production on US-v2 / v3 / JST 29, frame frozen
  in its header (nothing earned, production stays). Imports
  inspect_fit_k2_calibration (entry), inspect_fit_ma_calibration and
  inspect_last_push (both therefore also have a dependency reason).
  Outputs in `plots/fit_decisive/`.
- **inspect_combined_loads_feasibility.py** (2026-10-04, keep-reproduces,
  CONFIRMED by the author 2026-10-04: backs the metric-decision closure) -- part (c):
  production chain on the 16 NBR files, in-corner CS binned by measured
  (damper-tier, 2 Hz) axle load, medians and counts only, no fit, no
  verdict. Modes `compute <key>` and `report`. Imports inspect_imu_offsets
  and inspect_nbr_zero_entry_verify (_path). Outputs in
  `plots/combined_loads_feasibility/`.
- **inspect_fit_k2_calibration.py** (2026-10-04, keep-reproduces, CONFIRMED by
  the author 2026-10-04: the record of the HARM verdict; referenced from PLAN.md Q-CAL item 3) -- Q-CAL-3
  K2-constrained calibration pass: joint front/rear tyre fit with the
  balance m_f/C_f - m_r/C_r pinned to Q-PUSH P1's diagnostic K2, vs the
  production fit, US-v2 / v3 / JST 29, frame frozen in its header (verdict
  HARM, informational for Q-K2). Mode `metrics` re-renders from
  results.json. Imports inspect_fit_ma_calibration (which therefore also
  has a dependency reason), inspect_last_push and the (d2) harness; reads
  plots/last_push/p1_<key>.json. Outputs in `plots/fit_k2_calibration/`.
- **inspect_threshold_rederivation.py** (2026-10-04, keep-reproduces,
  CONFIRMED by the author 2026-10-04: the method dry-run for the threshold
  re-derivation that runs last in the queue, after the metric decision) -- post-levers
  threshold re-derivation diagnostic: classifier input per corner on the
  final estimator (eight standard sessions, production chain, cap 1), the
  recorded STRONG / MODERATE rules (imported from
  inspect_cs_decision_table), the percentile each threshold encodes, the
  JST 29 front tail check. Modes `compute <key>` and `report`. If the
  author derives new config values from it, it becomes their
  derived_from. Outputs in `plots/threshold_rederivation/`.
  [2026-10-11: modes `compute_fresh <key>` / `counts_fresh` -- the same
  clean rule on the corrected estimator, every session through the fresh
  chain into cache_clean_20261011; derived_from of the 2026-10-11
  STRONG / MODERATE values.]
- **inspect_iteration.py** (2026-10-05, keep-reproduces, CONFIRMED by the author 2026-10-05 --
  documents the Step 4 verdicts) -- Step 4 iteration experiment: refit on the
  EKF-refined slip angles, up to 3 rounds, starting from the session's own
  fit and from the pooled curve (Tier 1), frozen frame and literature
  citations in the header, the decisive frame for the final round.
  Imports inspect_pool_fits (pool curves, fitter hook) -> also a dependency
  reason for that script. Modes run | report | plots. Outputs in
  `plots/iteration/`.
- **inspect_pool_fits.py** (2026-10-05, keep-reproduces, CONFIRMED by the author 2026-10-05 --
  documents the Step 3 pool verdicts) -- Step 3 pooled fits: cache rebuild with the
  speed window (entry check per member), one pooled Pacejka fit per axle for
  Pool T (18 files, same tyre spec) and Pool D (7 JST files), each member's
  chain with the pooled curve behind the run_chain fitter hook, frozen
  verdict frame in the header, plus the C+ plots (pooled vs member curves,
  Tier-1 full-lap CS, Tier-1 CS histograms). Modes rebuild | pool | eval |
  report | plots | csplots (2026-10-05, kerb display package: the Tier-1
  CS renders with near-strike samples greyed; the grade mark is recomputed
  from the cached beta and must reproduce the cached CS first). Outputs in
  `plots/pool_fits/`.
- **inspect_mu_trace.py** (2026-10-05, keep status PROPOSED: keep-reproduces if
  the author ships a mu view or metric from it, else disposable at the commit) --
  Q-MU: axle utilisation trace mu = |Fy_axle| / Fz_axle on the 16 NBR files,
  Level-1 Fy over MEASURED Fz (fresh parse with the Q-CAL-2 zero entries; the
  cached d2 data predate them), both through the production 2 Hz low-pass;
  entry checks E1-E4 in the header; in-corner clean / near-strike-grey
  statistics, JST 29 position-locked sites, verdict-class sign test with the
  R = R_load x R_yaw decomposition, JST 21 phase frame. Imports
  inspect_tyre_prior_d2, inspect_last_push, inspect_pool_fits. Modes run |
  report | plots. Outputs in `plots/mu_trace/`.
- **inspect_mass.py** (2026-10-05, keep status PROPOSED: keep-reproduces as the
  record of the Q-MASS FAIL verdict, else disposable at the commit) -- Q-MASS
  step 1: production session mass fit (total Fz = m g + c v^2 on straights) on
  16 NBR + 2 Paul Ricard files, split-half stability, ECU fuel burn beside;
  frozen pass rule in the header. Modes step1 | report1. Outputs in
  `plots/mass/`.
- **inspect_curve_floor.py** (2026-10-05, keep-reproduces, CONFIRMED by the author 2026-10-05 --
  the basis of the STRONG-verdict meaning; re-runs with every threshold re-derivation) -- per pool session and axle, the
  production curve's own minimum cornering-stiffness-ratio analogue on the visited
  slip range vs the classification thresholds. Output `plots/curve_floor/`.
  [2026-10-06: the floor computation is promoted to modules/limit_events.curve_floor
  (the limit-event line); this script now calls it and reproduces floors.json
  byte-identically.]
- **inspect_spike_audit.py** (2026-10-05, keep-reproduces, CONFIRMED by the author 2026-10-05 -- as
  the per-event record feeding Q-SITES step 2) -- Q-SPIKE-AUDIT: every CS < -0.5
  event of NBR JST outing 29 classified (a) kerb detected / (b) kerb missed / (c)
  estimator artefact / (d) car-on-track event / (e) unexplained, frozen rules in the
  header; overview and deepest-event renders. Imports inspect_tyre_prior_d2,
  inspect_last_push, inspect_chair_audit (V-GRADE floor), gated_kinematic_common.
  Modes run | plots | repass (specificity re-pass, matched-window p95) |
  repass2 (final ledger, kerb-missed at union-5 %). Outputs in `plots/spike_audit/`.
- **inspect_speed_bands.py** (2026-10-04, keep-reproduces, CONFIRMED 2026-10-05) -- Step 2:
  our speed distribution against the chair's four speed bands, data
  quality per region, candidate windows SW / CU / BI; the derivation
  evidence behind config tyre_fit_auto.fit_speed_window_pct (cited in its
  provenance string -> also Referenced). Outputs in `plots/speed_bands/`.
- **inspect_steering_fix_impact.py** (2026-10-04, keep-reproduces, CONFIRMED 2026-10-05)
  -- Step 1-fix impact assessment: faulty lookup vs constant 15.7 vs
  corrected wheel-angle map on Tier 1 (beta, alpha, CS, fits), and the
  beta-spike localisation (mode spikes: EKF fallback-switch timing; BACKLOG
  -- PERMANENT item 11 evidence). Reads the archived caches
  plots/tyre_prior_d2/cache_cap1/ (15.7, pre-Q-CAL-1 mask) and
  cache_steerfix_prewindow/. Outputs in `plots/steering_fix_impact/`.
  [2026-10-17: its own Pacejka EKF config now carries nis_window_s and
  q_reference_rate_hz (it broke after R2 Class A).]
- RECORD-ONLY since 2026-10-04 (speed window in production): inspect_fit_
  decisive.py (its fitter hook stops on the mask check), inspect_fit_ma_
  calibration.py and inspect_fit_k2_calibration.py (their hooks do NOT
  check the mask -- a re-run would silently fit the windowed population);
  re-running any of them as recorded needs fit_speed_window_pct null.
- **inspect_kerb_cs_movement.py** (2026-10-03, keep-reproduces, CONFIRMED by
  the author 2026-10-03: documents lever 1's effect on CS) -- Q-CAL item 1
  closing check: CS before (+/-50 ms kerb mask) vs after (+/-100 ms) on
  US-v2, v3, JST 29 and Dubai with the cached production beta (entry check
  against the cached CS), distribution and tail shares, the 22
  position-locked site dips, one trace exhibit. Imports inspect_last_push
  and the (d2) harness. Outputs in `plots/kerb_cs_movement/`.
- **inspect_chair_audit.py** (2026-10-03, keep-reproduces, CONFIRMED by the
  author 2026-10-03: backs the Q-CHAIR-AUDIT closure and the curve-bound exhibit) -- PHASE 1 PLAN Q-CHAIR-AUDIT: Fy
  composition and damper witness, insufficient-signal grade (V-GRADE),
  adaptive windows (V-ADAPT, re-implemented production window loop with an
  exact entry check), curve-shape bound at position-locked dip sites, the
  post-hoc 4 s Welch Part-1b row, Part F tyre-model comparison (the chair's
  fit procedure on our population). Imports inspect_last_push and
  inspect_stiffness_cs (both therefore also have a dependency reason). Modes
  run | partE | partF. Outputs in `plots/chair_audit/`.
- **inspect_stiffness_cs.py** (2026-10-03, keep-reproduces, pending the
  author's ruling on Q-STIFF-CS) -- PHASE 1 PLAN Q-STIFF-CS: Part 1a yaw-rate
  gain per speed band (K cross-check), Part 1b transient yaw-mode stiffness
  identification (frequency domain + steering-step events, three-way exit),
  Part 2 CS readability (variance decomposition, window / ingredient
  variants, phase-median and cross-lap-band displays), gate, escalation.
  Imports inspect_last_push (which therefore also has a dependency reason)
  and the (d2) harness. Modes p1a | p1b | p2 | gate | esc2 | step2 | spike |
  spike_render (re-render from saved JSON). Outputs
  in `plots/stiffness_cs/`.
- **inspect_raw_channels.py** (2026-10-02, keep-reproduces: the evidence
  behind docs/team_questions_raw_channels.md, which the author reviews) -- PHASE 1 PLAN
  Q-RAW: native-sample rate, quantisation, hold, limits, spectra and lags
  of the sclu / log / ADR2 / MRR inertial channels on four files; the
  evidence behind the team case file. Imports the (d2) harness and
  inspect_imu_offsets.read_raw_blocks. Outputs in `plots/raw_channels/`.
- **inspect_damper_offset_q2.py** (2026-10-02, keep-reproduces, pending the
  author's Q2 exit ruling) -- PHASE 1 PLAN Q2: Part A limitation-11
  load-source check (tiers + static-path rear Fz), Part B NBR damper-offset
  census (burn-corrected drift, offset family), Part C anchors C1/C2/C3 via
  the linear-in-offset wheel_loads chain, physics validation, L4-rel load
  renders. Imports the (d2) harness and inspect_imu_offsets.read_raw_blocks.
  Modes: partA, partB <key>, partB_all, report. Outputs in
  `plots/damper_offset_q2/`. Delete at the commit closing Q2 unless the
  ruling keeps it as reference.
- **inspect_fuel_channels.py** (2026-10-02, keep-reproduces, pending the
  author's ruling on Q2b) -- PHASE 1 PLAN Q2b: fuel-used families,
  vol_fuel_rem_telem and the level-named channels on all usable files
  (consumption agreement, lap-sum plausibility, near-full share, co-movement
  with burn, plateau and slosh checks). Imports the (d2) harness and
  inspect_imu_offsets.read_raw_blocks. Outputs in `plots/fuel_q2b/`. It is
  the reference if a per-sample mass correction is proposed; otherwise
  delete at the commit closing Q2b.
- **inspect_e_stage0.py** (2026-10-01, keep-reproduces, pending the author's
  (e) exit ruling) -- EXPERIMENT PLAN (e) Stage 0: rear-axle force-demand
  occupancy sqrt(Fx_r^2 + Fy_r^2)/Fz_r on Level-4 rear loads, criteria
  P1/P2/N1-N3 as pre-registered in its header; imports the (d2) harness
  (inspect_tyre_prior_d2) and inspect_imu_offsets.read_raw_blocks. Modes:
  run <key>, dubai_l1 (Q1: labelled Level-1 read, never scored), report.
  Outputs in `plots/e_stage0/`. Disposal decided with
  the (e) exit ruling (adoption -> it is the reference; closure -> delete
  in that commit).
- **inspect_tyre_prior_d2.py** (2026-10-01, keep-reproduces) -- EXPERIMENT
  PLAN (d2): header census of every file on D:/Data_Bachelorthesis,
  per-file intake with the standing checks (rate guard, liveness, unit
  aliases/factors, clustering) and a cached parse, production fits (bit-
  exact vs the (d) cache), the pooled 18-session prior, curve-penalised
  anchored fits (w grid) through the production chain via an in-memory
  _fit_axle_pacejka replacement, the fit -> EKF -> refit convergence loop
  under the frozen prior, read-out and render. Modes: census, intake <key>,
  intake_report, fit <key>, prior, achain <key> <w>, loop <key> <w>,
  report, render, drift (where the loop drift lives, visited vs
  extrapolated), tloop <key> <w> and door2 (Door 2, transient-weighted
  loop, no signal), d1loop <key> and door1 (Door 1 Stage 0, tangential-
  acceleration identity vs the loop's per-pass beta; closed N4). Keep-reason: REFERENCED -- config/parameters.json
  tyre_fit_auto.tyre_fit_prior.derived_from names it as the prior's
  source; also the evidence for the (d2) closure and the estimator-chapter
  finding "The refit loop's drift decomposed". The anchored fitter itself
  now lives in modules/tyre_fit_auto.py (_fit_axle_pacejka_anchored).
  Records: thesis_notes.md "(d2) reopened ..." to "(d2) closure".
  Outputs in `plots/tyre_prior_d2/` (gitignored).
- **inspect_yaw_damping_eq43.py** (2026-09-29, keep-reproduces) -- the
  v1 consumer of modules/yaw_damping.py (Werner Eq. 4.3/4.4 completed
  without Fz): damping vs inertial yaw moment per corner-phase and the
  negative-D_psi population, both real sessions. Reproduces the
  thesis_notes.md "Werner Eq. 4.3 v1" figures; `--replot` redraws from the
  saved JSON. Plots in `plots_yaw_damping/` (gitignored).

- **inspect_kinematic_inputs.py** (2026-10-07, keep: thesis evidence: beta investigation rounds 1-3, reproduces the numbers in the beta chapter) -- kinematic-integration input audit, work
  order in PLAN.md STATUS 2026-10-07: steering-defined straights (data-derived
  thresholds), straight-line yaw-rate / ay offsets per segment / lap /
  session, quasi-steady ay / vx vs yaw-rate fits M0 / M1 / M2 with segment
  bootstrap, beta-error propagation per term, vx vs front wheel speeds.
  Reads diagnostics/plots/imu_offsets/report.json for the cross-check only.
  Record: thesis_notes.md "Kinematic-integration input audit ...". Outputs
  diagnostics/output/kinematic_inputs/ (--png, gitignored). Diagnostic 2
  (2026-10-07): --file repeatable (cross-session list), --filter-check
  (washout step response, one long corner); record "Diagnostic 2: what the
  beta washout does ...".
- **compare_beta_architectures.py** (2026-10-07, keep: thesis evidence: beta investigation rounds 1-3, reproduces the numbers in the beta chapter) -- offline beta architecture comparison A / B / B2 / C on
  NBR-73-010-026-JDR and MLA-SE07_US-v2 (calibration / validation straights,
  drift, five longest corners, entry / exit cleanliness, slip-angle span).
  Imports steering_straights, load, track_label from inspect_kinematic_inputs
  (Dependency). Record: thesis_notes.md "Beta architecture comparison ...".
  Round 2 (default; --round 1 reproduces round 1): gain + offset correction
  D / D2 / CG and the app's estimator E; record "Beta architectures, round 2
  ...". Also imports quasi_steady_segments.
- **seed_sensitivity_fit.py** (2026-10-07, keep: thesis evidence: beta investigation rounds 1-3, reproduces the numbers in the beta chapter) -- round 3: seed sensitivity of the session Pacejka fit
  and of E. Seeds A / D2 / B2 substituted for the kinematic beta inside
  tyre_fit_auto (route a, fit data) or also inside sideslip_ekf_pacejka
  (route b, EKF fallback / reset), in-process only; per run fit parameters,
  implied B*C*D stiffness, divergence counts, R choice, gate verdict, E on
  corners / straights. Imports from compare_beta_architectures and
  inspect_kinematic_inputs (Dependency). Record: thesis_notes.md "Beta
  architectures, round 3 ...".
- **check_estimator_hardening.py** (2026-10-08, keep: thesis evidence: beta investigation; validates the D3/D4 estimator change old-vs-new (phase 1) and carries the phase 2 step 1 analysis) -- phase-1 validation of the estimator hardening: D4
  plausibility-gate dry run on the round-3 fits (seeds A / D2 / B2) and the
  old vs new divergence re-entry on seed A (corners, straights, divergence
  counts split hard-bound / NIS-window, mid-corner zero resets, R choice).
  OLD = HEAD copy of sideslip_ekf_pacejka.py in diagnostics/output/
  estimator_hardening/ (gitignored, swapped in-process). Imports from
  compare_beta_architectures and inspect_kinematic_inputs (Dependency).
  Record: thesis_notes.md "Estimator hardening D3 + D4, phase 1 ...".
  Phase 2 modes (2026-10-11): p2s1 / p2s1b (Dubai, NBR, PR acceptance facts,
  per-corner sign check, pinned R), p2s2grid (full R grid), p2s2verify (the
  redesigned R selection), p2s2c (stability surge mechanism), p2s2robust /
  p2s2robust_report (R-robust / R-sensitive split, acceptance metrics --
  derived_from of the 2026-10-11 NIS-gate thresholds). Also imports
  inspect_tyre_prior_d2, inspect_new_laps_cs_sweep and
  inspect_threshold_rederivation (Dependency).
- **check_class_a.py** (2026-10-16, keep: thesis evidence + acceptance tool of the R2 Class A rebuild; reproduces the recorded old-vs-new acceptance numbers) -- OLD = the modules at
  279f4ee loaded from git in-process, NEW = working tree; Dubai, NBR-73-010-
  026-JDR, MLA-SE07_US-v2 (paths via tests/conftest resolve_data_path). Full
  chain (status, sign check, R pick, NIS gate score + skipped windows,
  divergences, straights |beta|), #26 and #32 isolated on the new beta, #12
  apex-on-grid count. Imports inspect_kinematic_inputs and
  inspect_tyre_prior_d2 (Dependency). Output diagnostics/output/class_a/
  (gitignored). Record: thesis_notes.md "Class A estimator corrections,
  rebuild". [2026-10-17, R3: runnable again -- injects the removed Dugoff
  modules from 279f4ee for the old tyre_fit_auto, restores
  nis_window_samples = 20 for the old run, passes laps to the new CS;
  optional label-prefix arguments. Its new side now carries Fix 0, so the
  #26-isolated lines reproduce the R2 record only where Fix 0 moves
  nothing (Dubai); for the full record run it at e04bdd5.]
- **check_class_b.py** (2026-10-17, keep: thesis evidence + acceptance tool
  of the R3 Class B rebuild; reproduces the recorded old-vs-new acceptance
  numbers) -- OLD = the modules at 85b9293 loaded from git in-process (the
  removed Dugoff modules injected for the old tyre_fit_auto), NEW = working
  tree; same three exports. Parse / state / fit manifest / gate / beta /
  stability bit-identity, Fix 0 CS deltas and no-reference counts, PR C16
  worst CS per lap, #12 apex cells, h2 and TPMS windows. Output
  diagnostics/output/class_b/ (gitignored). Record: thesis_notes.md
  "Class B, rebuild".

## K6 -- imported by a K1-K5 keeper (3 not already listed above)

- **gated_kinematic_common.py**, **gated_kinematic_style.py**
  (keep-reproduces) -- shared helpers and fixed series styling imported
  by the four inspect_gated_kinematic_* scripts (K5); same keep-reason:
  provenance for the 2026-09-29 gated-kinematic fail-ruling.
- **inspect_frame_stage2_parity.py** -- imported by
  capture_wp_perf_reference.py, inspect_damper_motion_sign_and_threshold.py,
  inspect_frame_candidate_census.py, inspect_pipeline_wall_times.py (all
  keepers above). The standard byte-stability tool for decision-layer
  work; old-vs-new engine parity check.

- **trace_render.py** (2026-09-30) -- shared plotting helper, not a
  diagnostic: the standard trace set (front/rear CS_ratio vs lap
  distance, thin line per analysis lap, corner-bracket bands, y fixed to
  config corner_trace_display) factored from the value-lens layout.
  Imported by inspect_cs_upstream_sweep.py (render) and
  inspect_cs_readout_evidence.py (sawrender; optional markers overlay
  added 2026-09-30). New packages call it;
  existing scripts keep their own copies (not refactored).

## K7 -- regenerates committed content / recurring checklist tool (2)

- **scan_channels.py** -- the tool that found this project's real GPS
  channel names, driving the WP1 channels.json whitelist fix. Sole tool
  for the recurring PLAN.md new-data-file channel-discovery checklist
  item; re-run whenever a new telemetry file needs its own channel
  census.
- **generate_channel_requirements.py** -- regenerates the two committed
  deliverables docs/channel_requirements.md and docs/channel_list.txt
  from config/channels.json and real read-site greps, in one run so the
  two can never disagree. Re-run whenever channels.json or a
  channel-consuming module changes.
- **inspect_channel_census.py** -- REPRODUCES the definitive channel census
  of Q-CHANNEL-AUDIT (2026-10-06): every channel of every Pi Toolbox text
  export on disk (value fingerprints), the channel names of every raw .pds,
  the candidate sweep and the content test (modes exports / pds / report /
  content; output diagnostics/channel_census/). Cited by thesis_notes
  "Q-CHANNEL-AUDIT ..." as the census the methods chapter refers to.
- **inspect_lap_census.py** -- REPRODUCES the lap census behind the
  double-outlap defect (2026-10-06): the unchanged production lap
  classifier on every unique export beside raw lap facts (duration, pit
  limiter at start / end, distance, speed). Output diagnostics/lap_census/.
  Re-run after the leading-fragment fix to show the analysis-lap sets
  unchanged.
  Blind spot (recorded 2026-10-06): its double-out-lap flag cannot see a
  fragment mislabelled as an in lap -- compare lap lists, not only out-lap counts.
- **smoke_test_analysis_lock.py** -- REPRODUCES (headless Qt smoke test)
  defect (b) of the 2026-10-06 app check: the driver-feedback corner count
  typed 10 -> 15 during a real Dubai analysis; checks the lock refuses it,
  the run completes, and the corner / lap trace windows open without an
  exception (optional screenshots via SMOKE_SHOT_DIR). Throwaway weekend row,
  nothing persisted.
- **inspect_fit_simplification.py** (2026-10-18) -- REFERENCED (PLAN.md queue
  entry Q-FIT-SIMPLE) and REPRODUCES the Q-FIT-SIMPLE frozen frame: tyre-fit
  simplification variants swapped into tyre_fit_auto for one chain run each
  and restored, entry check bit-identical to unswapped production. Imports
  inspect_stiffness_cs (grids / decompose, the cross-lap residual). Output
  diagnostics/plots/fit_simplification/.
- **inspect_fz_census.py** (2026-10-18) -- REFERENCED (PLAN.md Q-FIT-LIT
  step B, thesis_notes "Q-FIT-LIT: census and Step 1"): per-corner load
  source shares (damper / reconstructed / static) of the measured cascade on
  the fit population and the moving analysis-lap samples, damper channel
  quality, and whether the US-v2 FR channel correction applied; frozen 1 %
  stop rule. Output diagnostics/output/fz_census/.
- **check_fit_lit_inactive.py** (2026-10-18) -- REFERENCED (PLAN.md Q-FIT-LIT
  step C): the bit-identity proof that the inactive fit L leaves the
  production path unchanged -- tyre_fit_auto and sideslip_ekf_pacejka at
  fd40c78 vs the working tree on three sessions. Imports check_class_b.diff.
  Output diagnostics/output/fit_lit/. Since fit L became the default
  (2026-10-20) it selects per_session explicitly: the old-path bit-identity
  check.
- **inspect_fit_lit.py** (2026-10-18) -- REFERENCED (PLAN.md Q-FIT-LIT
  verification cycle) and REPRODUCES its frozen frame: fit L and L-nw
  through the whole chain (mode set in memory), start independence,
  leave-one-lap-out blockwise jackknife, physical checks, five frozen
  predictions; mode --reopen-d4 = the declared post-result D4 reopen (L
  only). Output diagnostics/plots/fit_lit/.
- **inspect_fit_lit_mechanism.py** (2026-10-18) -- REFERENCED (thesis_notes
  "Q-FIT-LIT: verification run -- chain results VOID"): post-hoc attribution
  of the L exit -- observer re-run with L's curves removing Fx, Fz variation
  and the d_is_mu path one at a time; located the measurement-update defect.
  Imports inspect_fit_lit. Output diagnostics/output/fit_lit/mechanism.log.
- **inspect_pr_anomaly.py** (2026-10-18) -- REFERENCED (PLAN.md
  Q-PR-ANOMALY, thesis_notes "Q-PR-ANOMALY") and REPRODUCES its frozen
  frame: per-corner residual of fit L against load, force and slip-side
  factors, linear-region K per session and lap. Imports inspect_fit_lit;
  reads diagnostics/plots/fit_lit/fit_lit_<key>.json. Output
  diagnostics/plots/pr_anomaly/.
- **inspect_obs_reentry.py** (2026-10-18) -- REFERENCED (PLAN.md
  Q-OBS-REENTRY, thesis_notes "Q-OBS-REENTRY") and REPRODUCES its frozen
  frame: per hard-bound re-entry of the observer under fit L (and
  production for context) the slip, curve slope, conditions and cause
  categories, and the share of verdict CS samples whose window it touches.
  Imports inspect_fit_lit. Output diagnostics/plots/obs_reentry/.
- **inspect_capacity.py** (2026-10-19) -- REFERENCED (PLAN.md Q-CAPACITY,
  thesis_notes "Q-CAPACITY") and REPRODUCES its frozen frame: measured
  force demand against the fitted grip, H-CAP at the observer re-entries,
  and the conditional demonstrated-utilisation run through the swap
  harness. Imports inspect_fit_lit and inspect_obs_reentry. Output
  diagnostics/plots/capacity/.
- **inspect_data_quality.py** (2026-10-19) -- REFERENCED (PLAN.md
  Q-DATA-QUALITY, thesis_notes "Q-DATA-QUALITY") and REPRODUCES the uniform
  file screen (Q1 force ceiling, Q2 load calibration with pooled NBR c, Q3
  load completeness, Q4 analysis laps) over every NBR and Le Castellet
  export. Output diagnostics/output/data_quality/.
- **inspect_pr_tail.py** (2026-10-19) -- REFERENCED (PLAN.md Q-PR-TAIL,
  thesis_notes "Q-PR-TAIL") and REPRODUCES its frozen frame: demand tail and
  observer re-entries of US-v2 vs the NBR controls under the
  demonstrated-mu rule. Imports inspect_fit_lit, inspect_obs_reentry,
  inspect_capacity. Output diagnostics/plots/pr_tail/.
- **inspect_fit_lit_final.py** (2026-10-19) -- REFERENCED (PLAN.md Q-FIT-LIT
  final implementation): the run of the implemented fit L (demonstrated mu,
  braking filter, observer-reset mark) on three sessions. Imports
  inspect_fit_lit and inspect_capacity. Output diagnostics/plots/fit_lit/final/.
- **inspect_nis_reset_effect.py** (2026-10-19) -- REFERENCED (PLAN.md
  Q-NIS-RESET): windowed CS slope with and without the NIS-window
  covariance reset (EKF flag nis_window_reset=False) on verdict windows
  marked only by NIS-window resets, against the declared 0.10 p95 tolerance.
  Also Q-NIS-CONTROL (the same metric on windows with no reset) and
  --populations (verdict CS samples per corner and phase after kerb and
  after kerb + reset exclusion). Imports inspect_fit_lit. Output
  diagnostics/plots/fit_lit/nis_reset/.
- **inspect_q_phys.py** (2026-10-19) -- REFERENCED (PLAN.md Q-OBS-DERIVE,
  step Q-PHYS): physical process noise for the fit-L observer from the
  fit-L force residual (SD, autocorrelation time, per-step Q under two
  discretisation assumptions) on the 14-session analysis set. Imports
  inspect_fit_lit, inspect_data_quality, tests.conftest. Output
  diagnostics/plots/q_phys/.
- **derive_cd_threshold.py** (2026-10-19) -- REFERENCED (PLAN.md h
  derivation; DECISIONS.md 'h derivation frame'): grid over the
  change-detector threshold h on the derive split, per-session cache
  (diagnostics/output/cd_threshold_cache/, gitignored), 3 worker
  processes. Also the cache provider for inspect_cs_dips.py. Output
  diagnostics/plots/cd_threshold/.
- **inspect_cs_dips.py** (2026-10-19) -- REFERENCED (PLAN.md Q-DIPS):
  Magic Formula post-peak slope floor, CS dip census with cause flags,
  our CS estimator vs the chair's with a counterfactual dip count.
  Imports derive_cd_threshold (cache). Output diagnostics/plots/cs_dips/.
- **inspect_q_upper.py** (2026-10-19) -- REFERENCED (PLAN.md Q-UPPER;
  DECISIONS.md 'Q-UPPER frame'): the fit-L observer with Q at the Q-PHYS
  upper bound against the divergence trigger, full chain per point on
  the derive split, 3 workers. Reads diagnostics/plots/q_phys/. Output
  diagnostics/plots/q_upper/.
- **inspect_fallback_59_70.py** (2026-10-19) -- REFERENCED (PLAN.md
  Q-UPPER, fallback check): D4-L inputs and values per analysis session
  and the weather / track-state channels of each export. Reads the
  derive_cd_threshold caches. Output diagnostics/plots/fallback_59_70/.
- **run_fit_l_observer_final.py** (2026-10-19) -- REFERENCED (PLAN.md
  observer finalisation fit L): the shipped fit-L chain (Q-UPPER in the
  chain, hard bound only, R_ay noise floor) on the 14-session analysis
  set, 3 workers. Output diagnostics/plots/fit_l_observer_final/.
- **inspect_cs_norm.py** (2026-10-20) -- REFERENCED (PLAN.md D4-L K band
  at source precision + CS on Fy / Fz): raw vs load-normalised CS on the
  14-session analysis set, reference band exclusion, lap-to-lap
  plausibility of samples below the -0.53 floor, one NBR 29 figure.
  Output diagnostics/plots/cs_norm/.
- **inspect_nbr54_c3.py** (2026-10-20) -- REFERENCED (PLAN.md reference
  band at source precision + NBR 54 C3 check): read-only check of the one
  repeatable cell of normalised CS below -0.53 (NBR 54 corner 3 turn-in,
  front), with the six-lap overlay figure. Output diagnostics/plots/cs_norm/.
- **inspect_threshold_rederivation_l.py** (2026-10-20) -- REFERENCED
  (PLAN.md final chain part 1): threshold re-derivation on the new default
  (fit L, schema 16) as a proposal, the recorded p10 / p30 rule unchanged
  (imports inspect_threshold_rederivation._corner_values) on the 14
  development sessions, sensitivity without outings 59 / 70, plus the
  floor-event census (events below -0.53, per lap and per cell). Output
  diagnostics/plots/threshold_rederivation/ (cache_l_20261020/,
  rederivation_l_20261020.txt / .json).
- **inspect_observer_robustness.py** (2026-10-20) -- REFERENCED (PLAN.md
  final chain part 2, D): robustness of the default path to the observer
  noise -- baseline (Q-UPPER, measured R floors), Q x 0.1, Q x 0.3, Q_beta
  upper bound with the old hand-set Q_r, R_yaw x 10 -- on the 14
  development sessions with the adopted thresholds fixed; verdict-class
  flips, floor events appearing / disappearing, gate and status changes.
  Variants are wrapped in memory per worker job, nothing in modules/ or
  config/ changes. Imports inspect_threshold_rederivation._corner_values.
  Output diagnostics/plots/observer_robustness/.
- **inspect_cs_margin_l.py** (2026-10-20) -- REFERENCED (config
  classification.verdict_stability_margin derived_from; PLAN.md cs_margin
  re-derivation): the recorded +/-1 % verdict-sensitivity sweep on the
  fit-L default, 14 sessions, adopted thresholds; [MARGINAL] shares at the
  old and new margin. Imports inspect_metrology_phase1_sensitivity
  (_scale_wheel_angle_column, classify_all) and
  inspect_metrology_phase1_analysis (_is_flip), which thereby qualify as
  Dependency. Output diagnostics/results_cs_margin_20261020/.
- **inspect_closeout_l.py** (2026-10-20) -- REFERENCED (config
  nis_gate.threshold_derived_from; PLAN.md close-out corrections): on the
  fit-L default over the 14 development sessions, the NIS gate health
  scores for the recorded threshold rule and the R26 recorded test
  (verdict-read CS samples under a reference below 0.5 x the session median
  of set events). Output diagnostics/plots/closeout_l/.
- **inspect_r26_fix_l.py** (2026-10-20) -- REFERENCED (PLAN.md R26 source
  fix): CS before / after the relative reference gate (L_REF_RELATIVE) on
  the 14 development sessions -- rejected candidates, R26 test count, moved
  phase medians, deepest corner value. Imports inspect_closeout_l (_ffill,
  _phase_medians) and inspect_threshold_rederivation (_corner_values).
  Output diagnostics/plots/r26_fix_l/. Run tags (2026-10-20): the threshold
  and cs_margin re-runs after the fix use THRESH_RUN_TAG / MARGIN_RUN_TAG =
  20261020b in inspect_threshold_rederivation_l.py / inspect_cs_margin_l.py.
- **inspect_ce_sensitivity_l.py** (2026-10-21) -- REFERENCED (PLAN.md
  documentation fixes + C / E sensitivity): fit-L shape C {1.0, 1.3, 1.6} x
  E {-1, 0, 1} set in memory per worker job, B fitted, mu = mu_demo; K,
  residual RMS, D4-L zone, status, re-entries, peak in 0-15 deg, corner-value
  shift and verdict-class changes against the shipped C 1.0 / E -1. Report
  only. Output diagnostics/plots/ce_sensitivity_l/. (inspect_observer_
  robustness.py gains ROBUST_RUN_TAG = 20261021 for the re-run on the final
  chain.)
- **inspect_utilisation.py** (2026-10-21) -- REFERENCED (PLAN.md STATUS
  NEXT 1, Q-UTIL decision pending): lateral and combined utilisation
  (|Fy| / Fz and sqrt(Fy^2 + Fx^2) / Fz over the fit-L mu_demo), secant
  ratio (|Fy| / Fz) / (|alpha| K) at |alpha| >= 0.01 rad and the vehicle
  mu_eff, after the chair's mue_effective / lateral_slip_efficiency_norm,
  on the CS path's 2 Hz signals of the default chain; coverage per phase,
  apex axle leader, distributions, rank correlation with the CS ratio, NBR
  54 corner 3 events, u leader against the CS verdict. Read-only, report
  only. Output diagnostics/plots/utilisation/.
- **inspect_phase_segmentation.py** (2026-10-21) -- REFERENCED (PLAN.md
  STATUS NEXT 1, segmentation and throttle-gate rulings pending): ecu_aps
  census and lift-off reach per session; the chair's rCosPhi phases
  (corner_characterisation thresholds verbatim, declared corner-extent
  simplifications) against our five phases on NBR 29 and NBR 10. Read-only.
  Output diagnostics/plots/phase_segmentation/.
- **inspect_washout_final.py** (2026-10-21) -- REFERENCED (thesis_notes
  "Yaw-stability median, washout scope, Fx drag"; PRODUCTION.md 12 washout
  limitation): the Q-PR-ANOMALY H-alpha test re-run on the shipped fit L
  (fit inputs captured from tyre_fit_auto._fit_axle_l), 14 development
  sessions, both axles. Read-only. Output diagnostics/plots/washout_final/.
- **inspect_drag_sensitivity.py** (2026-10-21) -- REFERENCED (PLAN.md
  STATUS NEXT 0b; PRODUCTION.md 12 drag sensitivity once C lands): c_D A in
  {0, 0.7, 1.0, 1.3} m^2 added to the CoG longitudinal force in memory
  (estimate_longitudinal_forces as tyre_fit_auto imports it), 14
  development sessions; braking-filter share, fit-L B / K / mu_demo, floor
  events, verdict class changes. Imports inspect_metrology_phase1_
  sensitivity.classify_all. Read-only. Output
  diagnostics/plots/drag_sensitivity/.
