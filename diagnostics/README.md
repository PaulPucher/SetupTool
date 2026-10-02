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
here (53 keepers + 37 in diagnostics/_attic/).

## K1 -- config-cited provenance (26)

- **fit_dugoff_first_pass.py** -- config/parameters.json's tyre_model_fit
  and tyre_model_ekf._comment blocks cite it as the WP-N1 c_alpha/mu_fz
  first-pass fit source (pass_0 numbers copied from its output).
- **fit_dugoff_pass4_refit.py** -- config/parameters.json's
  `_comment_pass_4_removed` explicitly states it is "intentionally KEPT
  for now" and still reads the live pass_3 block as its own EKF source.
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
- **inspect_ekf_pass1_rQ_sweep.py** -- cited in config/parameters.json's
  r_q_sweep_note.
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
- **inspect_native_channel_rates.py** -- cited in config/parameters.json's
  `_comment_grid_rate`.
- **inspect_nis_tyre_mismatch_gate.py** -- cited in
  config/parameters.json's nis_gate._comment.
- **inspect_tpms_pressure_cornering_phase.py** -- cited in
  config/channels.json's tpms_press_fl note and config/decision_frame.
  json's tyre_pressure_target derived_from (WP-ELICIT Phase B2,
  2026-09-24). Previously undocumented gap, closed here.
- **inspect_saturation_coverage.py** -- cited by description (not exact
  filename), config/parameters.json's tyre_model_fit._comment: "the
  WP-N0 saturation-coverage diagnostic's own candidate-threshold list."
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
  retire it. Also K3.

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
  docs/thesis_material_index.md Ch.2/3. Imports
  modules/sideslip_ekf_dugoff.py (relocated 2026-09-24, an ordinary
  modules/ import now, not a diagnostics-internal K6 relationship).
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
- **inspect_phase1_q3.py** (2026-10-02, keep-reproduces, pending the
  author's Q3 exit ruling) -- PHASE 1 PLAN Q2c + Q3: G0 GPS census, G1
  gradient map, G2 corner anchor (unscored), V1 / V2 tailoring variants and
  the Q4 courtroom ((ii) rake, (iii) per-phase driver statement, (v)
  coherence) with CS renders; rules frozen in its header. Imports the (d2)
  harness and its caches. Outputs in `plots/phase1_q3/`; modes g0 | g1 |
  g2 | v1 | v2 | q4 | all.
- **inspect_rake_ab_anchor.py** (2026-10-02, keep-reproduces, pending the
  author's ruling on the A/B anchor characterisation) -- PHASE 1 PLAN Q3
  ruling 3 (reframed): NBR A (outing 21) vs B (outing 29) change census
  over all channel blocks, C1 / C4 / C10 corner traces (speed, a_y, steer,
  yaw, brake, GPS line offset, alpha_f / peak, CS), consistency and
  phase-flipping sign tests. Imports the (d2) harness. Outputs in
  `plots/rake_ab_anchor/`; modes census | corners | coherence | all.
- **inspect_g2_anchor_validation.py** (2026-10-02, keep-reproduces, pending
  the author's look at the GPS-anchor closure) -- PHASE 1 PLAN Q3 extended
  package: Gate A on g2_repair2.json, validation battery (sign/phase,
  latency +/-0.05 s, SE vs speed, kerb, lap-count, cross-session), Gate B,
  gated first external beta read (Part 3). Imports inspect_phase1_q3 and
  the (d2) harness. Outputs in `plots/g2_anchor_validation/`.
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
