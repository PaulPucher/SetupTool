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

Plot directories: three are gitignored (`plots/`, `plots_step2/`,
`plots_decision_layer/`), seven are tracked (`plots_deepening/`,
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
- **generate_decision_layer_figure.py** -- generates
  diagnostics/plots_decision_layer/six_stage_flow.png and
  lever_coverage_table.png, cited docs/thesis_material_index.md Ch.5.
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

## K5 -- named census/measurement tooling (2 not already listed above)

- **inspect_brake_bias_channel_identity.py** -- WP-ELICIT HANDOFF item 5
  (2026-09-25, Phase D brake-bias channel identity): recomputes percent-
  front from raw brake pressure during real braking events and
  correlates it against every Frame-Stage-2 Phase 2 candidate channel,
  both real sessions. Re-run whenever a new session or a resolved
  candidate channel warrants re-checking the identity.
- **inspect_pipeline_sidecar_size.py** -- WP-CACHE sidecar size/load-time
  measurement, both real sessions against the acceptance gate.

## K6 -- imported by a K1-K5 keeper (1 not already listed above)

- **inspect_frame_stage2_parity.py** -- imported by
  capture_wp_perf_reference.py, inspect_damper_motion_sign_and_threshold.py,
  inspect_frame_candidate_census.py, inspect_pipeline_wall_times.py (all
  keepers above). The standard byte-stability tool for decision-layer
  work; old-vs-new engine parity check.

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
