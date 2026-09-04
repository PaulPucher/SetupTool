# Diagnostics

One-off read-only scripts used during development to produce methodology
evidence for the thesis. Not part of the app; run manually from the project
root (`python diagnostics/<script>.py`) against the sample data.

STANDING RULE (2026-08-30, CLAUDE.md): a diagnostic script is disposable
by default. Once its finding is recorded in thesis_notes.md and its work
package commits, it gets deleted in that same commit unless it is still
referenced from outside its own file (a docstring pointer, a config
`derived_from`/provenance string, the frozen pass-1 validation baseline,
or a PLAN.md open item) or is live tooling still needed to reproduce
something (a golden-value generator, the frozen-baseline script, a smoke
test, a production import source). This file lists every surviving
script's reason to exist -- if a script is here without one, that is a
bug in this README, not license to leave a future one uncommented.

Every entry below survived the 2026-08-30 full-inventory sweep
(thesis_notes.md "10. Second diagnostics sweep: full-inventory
classification") or the 2026-09-02 threshold-anchoring/arc-closure
sweep (thesis_notes.md "Threshold anchoring + arc closure, Phase 6:
diagnostics classification") -- the latter classified every script
from the CS validity repair investigation arc plus one pre-existing
gap (inspect_tyre_variant_comparison.py, unrelated to that arc, never
previously listed here). Category in brackets: **[keep-referenced]** = cited
from outside diagnostics/ (config provenance, PLAN.md, or a docstring/
test pointer); **[keep-reproduces]** = live tooling (frozen baseline,
smoke test, or an actual production import source); **[dependency]** =
kept because a surviving script imports it, not for its own findings.

- **scan_channels.py** `[keep-referenced]` — inventories every channel
  actually present in a Cosworth Pi Toolbox file, regardless of the
  `channels.json` whitelist. Used to discover that the file's real GPS
  position channels are `log_gps_lat`/`log_gps_lon`, not
  `gpsa_lat`/`gpsa_long`/`VBOX_*` — evidence for the WP1 GPS channel scan
  and whitelist decision (those six placeholder entries were removed
  from channels.json in the WP-A registry-consolidation rider,
  2026-07-26). Cited by name in PLAN.md (cp1252-decoding precedent).
- **inspect_corner_distribution.py** `[keep-referenced]` — prints
  per-corner worst-phase CS_ratio and stability-margin percentiles, and
  how many corners each candidate threshold would flag. Basis for the
  live `classification.STRONG_CSF/STRONG_CSR/MODERATE_CSF/MODERATE_CSR`
  thresholds in config/parameters.json, cited there by exact filename in
  every `derived_from` field. Load-bearing threshold provenance --
  deleting this would leave those thresholds unsourced.
- **inspect_yaw_stability_b2.py** `[keep-referenced]` — same role as
  above for `classification.stab_neg_thresh_Nm_per_deg`, cited by exact
  filename in its `derived_from` field.
- **inspect_beta_gps_validation.py** `[keep-referenced]` — cited by
  filename in modules/stability_analysis.py's own docstring
  (estimate_sideslip_gps's per-lap check).
- **sideslip_kalman_observer.py** `[keep-referenced]` — the diagnostics-
  only linear Kalman sideslip observer candidate. Cited by exact
  filename twice in config/parameters.json (vehicle.yaw_inertia_kalman_
  note and its accuracy_levels.yaw_inertia_kalman node) as the consumer
  of those two placeholder values.
- **fit_dugoff_first_pass.py** `[keep-referenced]` — WP-N1b Dugoff
  c_alpha/mu_fz first-pass fit. Cited by exact filename in config/
  parameters.json (tyre_model_fit._comment, tyre_model_ekf.pass_0.
  frozen_from) and in modules/tyre_fit_auto.py's own docstring
  (explains what the automated fit chain does and does not reproduce
  from this script).
- **fit_dugoff_first_pass_manifest.json** `[keep-referenced]` —
  fit_dugoff_first_pass.py's own output, cited by exact path in config/
  parameters.json's tyre_model_ekf.pass_0.frozen_from as the source of
  the frozen c_alpha/mu_fz numbers. Gitignored (regenerable), kept on
  disk so the citation is directly verifiable without re-running.
- **inspect_ekf_pass1_rQ_sweep.py** `[keep-referenced]` — the pass-1 R
  noise-model 2-D grid sweep. Cited by exact filename in config/
  parameters.json (tyre_model_ekf.pass_1.r_q_sweep_note) and modules/
  tyre_fit_auto.py's own docstring/comments (its band/grid values are
  reproduced there).
- **inspect_ekf_dugoff_sanity_checks.py** `[keep-referenced]` — cited by
  filename in PLAN.md (WP-N1 sanity-check record).
- **inspect_combined_slip_premise.py** `[keep-referenced]` — cited by
  filename in modules/longitudinal_forces.py's own docstring (the
  external validation this module's Fx/kappa output was checked
  against, exact digit match recorded in PLAN.md/thesis_notes.md).
- **inspect_tyre_fit_auto_acceptance.py** `[keep-referenced]` — cited by
  filename in tests/test_auto_fit_wiring.py's own docstring (states
  what that test does NOT re-cover, already covered here).
- **inspect_washout_cutoff_sweep.py** `[keep-referenced]` — cited by
  filename in PLAN.md (Phase 1 washout-cutoff sweep, the entry that
  blocked the washout-cutoff decision). Also **[dependency]**: imports
  diagnostics/sideslip_ekf_dugoff.py and diagnostics/inspect_wheel_
  speed_sources.py (AY_STRAIGHT_MAX_G/YAW_STRAIGHT_MAX_DEGPS).
- **inspect_ls_cs_disambiguation.py** `[keep-referenced]` — cited by
  filename in PLAN.md (STEP 3 LS_ratio disambiguation record).
- **inspect_ls_ratio_span_dependence.py** `[keep-referenced]` — cited by
  filename in thesis_notes.md ("PLAN.md unsupervised package, Phase 3:
  LS_ratio span-dependence" entry), which also cites the PNG it
  produces (diagnostics/plots_step2/ls_ratio_span_dependence.png) as
  supporting evidence for the longitudinal_stiffness.min_slip_span=
  0.004 config gate.
- **inspect_nis_tyre_mismatch_gate.py** `[keep-referenced]` — the WP-N3
  NIS mismatch-gate prototype. Cited by filename in modules/nis_gate.py's
  own docstring (states it ports this prototype) and tests/
  test_nis_gate.py's docstring.
- **inspect_saturation_coverage.py** `[keep-referenced]` — the WP-N0
  saturation-coverage diagnostic. Cited by description (its own
  candidate-threshold list) in config/parameters.json's tyre_model_fit
  comment as the source of ay_linear_threshold_g=0.3's default.
- **inspect_kerb_wheel_speed_spikes.py** `[keep-referenced]` — cited by
  exact filename in config/parameters.json's longitudinal_stiffness.
  kerb_investigation_reference.
- **inspect_c3_leaked_windows.py** `[keep-referenced]` — same
  config/parameters.json citation as above.
- **sideslip_ekf_dugoff.py** `[keep-reproduces]` — NOT diagnostics-only
  despite its location: `modules/tyre_fit_auto.py` imports
  `estimate_sideslip_ekf_dugoff` from this file directly, as does
  tests/test_pure_functions.py and tests/test_nis_gate.py. A real
  production dependency living in diagnostics/ by historical accident,
  not by design -- flagged, not relocated, out of this sweep's scope.
  Also `[dependency]`-consumed from inside diagnostics/ itself:
  inspect_step2_chair_plots.py (PLAN.md STEP 2) imports
  `estimate_sideslip_ekf_dugoff` from here directly for its ekf_pass_1
  side-by-side plots -- one more reason this file cannot be deleted or
  relocated without updating a second caller.
- **sideslip_ekf_pacejka.py** `[keep-reproduces]` — same as above,
  imported by modules/tyre_fit_auto.py.
- **fit_dugoff_pass3_refit_manifest.json** `[keep-referenced]` —
  config/parameters.json's tyre_model_ekf.pass_3.frozen_from cites this
  exact path. The pass_3 config block is itself live (fit_dugoff_
  pass4_refit.py below still reads it), so this manifest stays even
  though its own producing script (fit_dugoff_pass3_refit.py) was
  deleted in the first diagnostics sweep (2026-08-20).
- **inspect_pass1_final_validation.py** `[keep-reproduces]` — THE
  frozen pass-1 EKF validation baseline. Cited throughout PLAN.md/
  thesis_notes.md as "the reference any future estimator work is
  compared against." Do not delete without a deliberate, explicit
  decision to retire the pass-1 baseline itself.
- **pass1_final_validation_manifest.json** `[keep-reproduces]` — the
  above script's own frozen numeric output. Gitignored, kept on disk as
  the actual evidence backing the baseline claim.
- **inspect_combined_slip_premise.py, inspect_slip_channel_sweep.py** —
  see individual entries; inspect_slip_channel_sweep.py kept as a
  BORDERLINE case (2026-08-30 sweep): its raw-channel keyword-scan
  finding may already be folded into the WP2b-1 full channel census
  (thesis_notes.md "Full channel census + targeted verification"), but
  this was not confirmed with full certainty -- kept per "when unsure,
  keep, list" rather than risk losing an unrecorded finding.
- **fit_dugoff_pass4_refit.py** `[keep-reproduces]` — reads the still-
  live pass_3 config block as its own EKF source. Kept explicitly in
  the first diagnostics sweep (2026-08-20) specifically so pass_3
  wouldn't be orphaned; that dependency is unchanged.
- **smoke_test_corner_trace_dialog.py** `[keep-reproduces]` — reusable
  headless Qt smoke test for CornerTraceDialog/LapTraceDialog (offscreen
  platform, real analysis result, catches runtime errors a syntax check
  cannot). Not part of the regression suite, run manually.
- **inspect_ls_cs_disambiguation.py, inspect_kerb_wheel_speed_spikes.py,
  inspect_c3_leaked_windows.py** — see individual entries above.
- **smoke_test_measurement_points_widget.py** `[keep-reproduces]` —
  reusable headless Qt smoke test for the measurement-points form widget
  (splitter/diffuser points), same pattern as smoke_test_corner_trace_
  dialog.py. Cited by filename in PLAN.md and tests/test_setup_data_
  points.py's own comment.
- **inspect_wheel_speed_sources.py** `[dependency]` — kept solely because
  inspect_washout_cutoff_sweep.py imports AY_STRAIGHT_MAX_G/
  YAW_STRAIGHT_MAX_DEGPS from it. Misclassified for deletion in the
  2026-08-30 sweep's first pass, caught by the post-deletion compile
  check, restored. No independent external reference of its own.
- **inspect_step2_chair_plots.py** `[keep-reproduces]` — PLAN.md STEP 2
  is now marked DONE there, citing this script by filename, with the
  headline finding (rear CS extremes at C6/C9 are largely beta
  artifacts; C4's front saturation is not). Findings recorded in
  thesis_notes.md "12. PLAN.md STEP 2: chair-comparable result plots,
  kinematic vs ekf_pass_1". Kept as the reusable generator behind the
  session's 28 PNGs -- the thesis figure source for this finding, not a
  one-off investigation script; re-run whenever the sideslip source or
  the CS estimator changes and the figures need regenerating. Output
  diagnostics/plots_step2/ (gitignored). Imports `estimate_sideslip_ekf_
  dugoff` from sideslip_ekf_dugoff.py directly (see that entry above).
- **inspect_cs_window_floor_derivation.py** `[keep-referenced]` — the CS
  validity repair's first-attempt window-floor bootstrap (superseded as a
  DECIDING criterion, thesis_notes.md "CS validity repair, part A, Phase
  1: window-floor re-derivation" and its own REVISION entry) -- but its
  linear_region_end finding is still the LIVE source config/parameters.
  json's cs_linear_slip_threshold_rad_derived_from cites directly by exact
  filename (that specific finding was never superseded, only the window-
  floor N/span choice was). Deleting this would leave that citation
  dangling.
- **inspect_cs_duration_only_comparison.py** `[keep-referenced]` — the
  sign-off clarification round's 0.1s-vs-0.2s duration-only bootstrap
  (thesis_notes.md "CS validity repair, sign-off clarification round").
  Cited by exact filename in config/parameters.json's cs_min_window_s_
  derived_from as joint provenance (with inspect_cs_floor_candidate_
  validation.py) for the live 0.1s floor.
- **inspect_cs_max_window_locality_sizing.py** `[keep-referenced]` —
  measures the natural (uncapped) CS window's own metre extent. Cited by
  exact filename in config/parameters.json's _comment_cs_max_window_m as
  the source of the live cs_max_window_m=53.0 value; re-run at least
  twice across this arc's own floor revisions (thesis_notes.md "100 Hz
  time-base work package, Phase 4" also cites its re-run numbers).
- **inspect_native_channel_rates.py** `[keep-referenced]` — per-channel
  native sample-rate census (ecu_speed 50 Hz vs the other five CS-chain
  channels at 100 Hz). Cited by exact filename in config/parameters.
  json's _comment_grid_rate as the evidence behind the adaptive 50-100 Hz
  grid design.
- **inspect_corner_bracket_geometry.py** `[keep-referenced]` — corner
  bracket length vs corner-to-corner gap geometry (thesis_notes.md "CS
  validity repair, limitation: cs_max_window_m does not guarantee
  locality..."). Cited by exact filename in PLAN.md's PARKED new-data-
  file checklist ("re-run diagnostics/inspect_corner_bracket_geometry.py
  ... on the new track's own corner sequence") -- a live, forward-looking
  reference for whenever a second track's data arrives, not just a past
  finding.
- **inspect_cs_floor_candidate_validation.py** `[keep-reproduces]` — the
  CS validity repair's direct real-data floor validation (thesis_notes.md
  "100 Hz time-base work package, Phase 1 FINAL" and "CS validity repair,
  sign-off clarification round"). THE generator behind diagnostics/
  threshold_anchoring_input.md (cited there by exact filename as its own
  source) and the threshold-anchoring Phase 1/2 work (thesis_notes.md
  "Threshold anchoring, Phase 1/2") -- also jointly cited in config/
  parameters.json's cs_min_window_s_derived_from and _comment_cs_max_
  window_m. Load-bearing provenance for the live STRONG/MODERATE_CS*
  thresholds; re-run whenever the CS window floor or the anchoring
  population needs re-deriving.
- **inspect_cs_phase_median_floor_derivation_v2.py** `[keep-reproduces]`
  — the CS window floor's final bootstrap methodology (cornering-only
  population; thesis_notes.md "100 Hz time-base work package, Phase 1:
  floor derivation, third pass"), the version that led directly to the
  direct-real-data-validation pivot above. Kept as the final methodology
  version of this investigation branch, superseding v1 (inspect_cs_
  phase_median_floor_derivation.py, deleted 2026-09-02 -- its own
  earlier finding is fully recorded in thesis_notes.md "CS validity
  repair, part A, Phase 1 REVISION").
- **inspect_run_ground_truth.py** `[keep-reproduces]` — per-run ground-
  truth verdicts (fold/loop tyre-curve pictures + LS_ratio/kappa +
  steering-rate/stability evidence) for the 10 named C2/C3/C4 runs
  (thesis_notes.md "Ground-truth workup: per-run verdicts for the long-
  run corners..."). The single strongest evidence line in this entire
  investigation arc (C4 confirmed REAL via actual fold pictures, C2
  front/C3 rear confirmed ARTIFACT via loop pictures) and the basis for
  which corners are excluded from the live STRONG_CSF/CSR noise-margin
  population (config/parameters.json derived_from citations). Imports
  `_canonical_window_slice`/`_build_track_map` from inspect_step2_
  chair_plots.py (already `[keep-reproduces]` below).
- **inspect_tyre_variant_comparison.py** `[keep-referenced]` — WP-N3
  Phase 3's Dugoff-vs-Pacejka fit comparison (thesis_notes.md "3.
  WP-N3..., Phase 3: Pacejka variant"). Backs the still-open "fit-variant
  choice" decision listed in PLAN.md's own carry-forward items --
  unrelated to the CS validity repair/threshold anchoring arc, out of
  that arc's own disposal sweep, kept here since it had no README entry
  at all (a gap this sweep also closes). NOTE: thesis_notes.md's own
  "10. Second diagnostics sweep" entry (2026-08-30) lists this exact
  filename as DELETED that day -- `git log --follow` on this path shows
  only its original 2026-08-20 add, never a deletion, so that historical
  record entry appears to be in error (flagged, not corrected -- CLAUDE.md
  forbids rewriting past thesis_notes.md entries).
- **generate_channel_requirements.py** `[keep-reproduces]` — regenerates
  the two committed deliverables docs/channel_requirements.md (the
  telemetry-export checklist for a new event, with per-channel WHY) and
  docs/channel_list.txt (the same channels, bare one-per-line, for a
  literal tick-off) from config/channels.json, real read-site greps of
  modules/core/ui, and channel_list.txt (repo root, the real Dubai
  channel inventory -- not to be confused with the generated docs/
  channel_list.txt). Both outputs come from one run, so they can never
  disagree with each other. Re-run whenever channels.json or a channel-
  consuming module changes; the two docs/ files are the deliverables,
  this script is what keeps them from drifting. Not a one-off
  investigation -- a reusable generator, kept by design, not by the
  disposal-rule
  exceptions above.
- **inspect_prc_v3_sample_rates.py** `[keep-reproduces]` — read-only
  per-channel-block sample-rate/layout census (streamed, never loads a
  full multi-GB file into memory). Written for GT3_PRC_MLA-v3.txt (2026-
  09-02, thesis_notes.md "GT3_PRC_MLA-v3 census: per-channel-block layout,
  100 Hz dampers"), the first real damper-channel-bearing telemetry file
  this project has seen -- kept as a reusable data-provenance check for
  future telemetry files, not a one-off finding: any new export can be
  re-run through this script to confirm layout (wide-table vs per-channel
  block), per-channel-family rates, and TC LAT/TC LON/ABS/brake-bias
  header candidates before it is trusted as an analysis input.
- **smoke_test_decision_frame_widget.py** `[keep-reproduces]` — reusable
  headless Qt smoke test for the "Decision Frame (preview)" section
  (decision-matrix frame, Stage 1, 2026-09-02), same technique and role as
  smoke_test_measurement_points_widget.py: verifies the widget binding
  (toggle show/hide, button enable/disable, row rendering against a real
  constructed OutingForm) that tests/test_decision_frame.py's pytest suite
  cannot reach (conftest.py deliberately keeps PyQt6 out of pytest).
- **inspect_v3_wheel_load_validation.py** `[keep-reproduces]` — damper
  package (thesis_notes.md "Damper package: wheel loads from pushrod/
  suspension-travel channels, Phases 1-6", 2026-09-03) Phase 2 validation
  of modules/wheel_loads.py against real damper/suspension-travel data:
  straight-line total load vs config weight, fuel-drift trend, transfer
  signs/magnitudes vs ax/ay, and the ARB sign-convention empirical check
  the module's own docstring flags as needing re-confirmation. Re-run
  whenever the estimator changes or a new damper-equipped session arrives.
- **inspect_v3_wheel_load_comparison_figure.py** `[keep-reproduces]` — the
  same package's Phase 3 static-split-vs-damper-derived Fz comparison
  figure generator (rear axle, chosen over front because log_dms_dam_fr
  is corrupted for the whole GT3_PRC_MLA-v3.txt session). Reusable figure
  source, not a one-off -- re-run for a new session or corner choice.
- **inspect_v3_tc_eb_abs_channels.py** `[keep-reproduces]` — the same
  package's Phase 4 channel survey (traction control, engine braking, ABS
  activity-vs-position, brake bias), extending the v3 census's own tc_lat/
  tc_lon/abs/brake_bias search with tract/asr/eb/ebrake/engine_brake/map
  token-matched terms and per-candidate rate/range/changes-during-session
  detail. Identification evidence only, no mapping conclusion. Reusable
  for any future telemetry file, same role as inspect_prc_v3_sample_
  rates.py for layout/rate census.
- **inspect_v3_sawtooth_mechanism.py** `[keep-reproduces]` — damper
  package Phase 7 (thesis_notes.md "v3 sawtooth mechanism investigation:
  corner selection, window stats, floor-fraction and alpha-character
  comparison vs Dubai", 2026-09-02), read-only. Diagnosed v3's CS_ratio
  sawtooth artifact (C13/C12/C5) as CORNER CHARACTER, not a floor
  miscalibration: v3 and Dubai resolve to numerically identical CS
  window floors at 100Hz, but v3's own alpha signal is ~31% faster and
  ~1.8x noisier at matched corner speed, so the same window-growth floor
  is hit sooner and more often. Open thread (PLAN.md STATUS), will need
  re-running once CS window floors or thresholds are revisited.
- **inspect_v3_fuel_drift_recheck.py** `[keep-reproduces]` — follow-up to
  the damper package's own Phase 2(b) inconclusive fuel-drift finding
  (thesis_notes.md "Session-measured split fractions..."): normalises
  each lap's straight-line total by the session-fit aero coefficient
  (c_session, relative to the session's own reference speed) to remove
  the speed confound that made the raw per-lap totals uninterpretable.
  Re-run whenever the session-correction model or a new session changes.
- **inspect_v3_wheel_load_reconstruction_figure.py** `[keep-reproduces]`
  — morning follow-up Item 2 (thesis_notes.md "Morning follow-up to the
  damper package...", 2026-09-03): front-axle comparison figure, measured
  FL vs RECONSTRUCTED FR (modules.wheel_loads.reconstruct_missing_
  corner), companion to inspect_v3_wheel_load_comparison_figure.py's
  rear-axle figure. Reusable figure source, not a one-off.
- **inspect_v3_abs_consistency_check.py** `[keep-reproduces]` — morning
  follow-up Item 3: read-only ABS switch-position/activity consistency
  check on GT3_PRC_MLA-v3.txt (streams 8 named channels directly, no
  channels.json change). No mapping conclusion -- identification/
  consistency evidence only. Re-run for any future ABS-question or new
  damper-equipped session.
- **inspect_v3_nis_gate_failure.py** `[keep-referenced]` — pre-existing
  script (v3 work package), gap in this README until now. Cited by
  filename in PLAN.md's NIS gate redesign proposal (damper package Phase
  8, 2026-09-03) as the source of the live Dubai-vs-v3 health-score/
  rate-correction numbers that proposal's PROBLEM section quotes exactly
  (Dubai 0.1417->0.1351, v3 0.1163->0.0849 under a rate-corrected
  window). Re-run whenever the NIS gate window/thresholds change --
  the proposal's own numbers would need re-confirming.
- **inspect_v3_aero_load_diagnostic.py** `[keep-reproduces]` — the same
  package's Phase 5 aero diagnostic: damper-derived total Fz regressed
  against v^2 (with an ax term to remove longitudinal-transfer
  contamination) on straight-line stretches. Trusts nothing downstream
  (a pure top-level Fz regression, feeds no estimator); does not write
  back to config (vehicle.aero.lift_coeff/cross_track_area_m2 stay at
  their 0.0 placeholders -- only their product is identifiable from a
  constant-speed regression). Re-run whenever a new damper-equipped
  session arrives or the wheel-load estimator changes.
- **inspect_dubai_wheel_load_validation.py** `[keep-reproduces]` —
  Fz-integration Phase 1 (2026-09-03): the Dubai counterpart to inspect_v3_
  wheel_load_validation.py/inspect_v3_aero_load_diagnostic.py, run after
  the premise-correction finding that Sample_Dubai.txt actually has real
  damper/travel channels (thesis_notes.md "Fz-integration Phase 1: premise
  correction..."). Per-gauge plausibility, straight-line-vs-config-weight,
  transfer sign/magnitude, ARB sign-convention, and aero v^2 checks for a
  SECOND real session -- re-run whenever the wheel-load estimator, the
  dead-channel guard, or Dubai's own file changes.
- **inspect_fz_before_after.py** `[keep-reproduces]` — Fz-integration
  Phase 1 finish (2026-09-03): static-vs-measured before/after for
  stability_estimation.vertical_load_source on both real sessions
  (thesis_notes.md "Fz-integration Phase 1 (finish)..."). Reports what
  actually changes under the flag (fz_*_N/fy_norm values, per-corner
  damper/reconstructed/static_fallback share) -- explicitly NOT CS_ratio/
  stability/verdicts, which are proven independent of this flag (same
  entry). Generates the front/rear-axle static-vs-measured trace figures
  at named + auto-picked corners. Re-run whenever the wheel-load
  estimator changes or a third damper-equipped session arrives.
- **inspect_fz_mu_tyre_fit.py** `[keep-reproduces]` — Fz-integration
  Phase 2 (2026-09-03): free-D vs load-normalised (mu) Pacejka fit on
  both real sessions (thesis_notes.md "Fz-integration Phase 2: load-
  normalised (mu) Pacejka tyre fit..."). Reports B/C/D/E (or mu),
  residuals, and the resulting EKF's NIS/gate numbers per axle; flags a
  fitted mu outside config tyre_fit_auto.mu_plausibility_band_low/high.
  Writes diagnostics/fz_mu_tyre_fit_results.json (gitignored). Re-run
  whenever the mu fit, the measured-Fz cascade, or the Pacejka model
  changes, or a third damper-equipped session arrives.
- **inspect_fz_mu_cross_check.py** `[keep-reproduces]` — Fz-integration
  Phase 2 gate resolution (2026-09-03): the decisive cross-check
  (thesis_notes.md "Fz-integration Phase 2 gate resolution") comparing
  each axle/session's joint-fit mu against (free-D fit's own D) /
  (median measured Fz in that axle's fit population). Reuses
  diagnostics/fz_mu_tyre_fit_results.json, only recomputes the median
  Fz (cheap). Re-run whenever the mu fit or the measured-Fz cascade
  changes.
- **inspect_fz_mu_v3_rear_divergence.py** `[keep-reproduces]` — Fz-
  integration Phase 2 gate resolution, v3 rear divergence dig (2026-09-
  03, thesis_notes.md "v3 rear divergence dig..."). v3 front (control)
  and rear (the +20.76% cross-check divergence) tyre-cloud figures
  coloured by measured Fz with free-D/mu-median/mu-p25-p75-band curves
  overlaid, plus correlation and Fz-tercile residual numbers. Concluded
  LEGITIMATE LOAD EFFECT, not a fit artifact. Re-run whenever the mu fit
  or the measured-Fz cascade changes.
- **inspect_fz_mu_refit_evaluation.py** `[keep-reproduces]` — Fz-
  integration Phase 3 (2026-09-03, thesis_notes.md "Fz-integration
  Phase 3: bounded refit loop under mu..."). Mirrors inspect_v3_
  pacejka_refit_evaluation.py's exact 4-iteration refit chain, one
  substitution (_fit_axle_pacejka_mu, D=mu*Fz, in place of the free-D
  axle fit). Classifies BOUNDED/CREEPING/WANDERING per axle per the
  amended per-axle +/-15%-of-iteration-1 mu growth band. Result: both
  sessions non-convergent (Dubai CREEPING, v3 WANDERING). Re-run
  whenever the mu fit, the EKF Pacejka path, or the measured-Fz cascade
  changes.
- **inspect_v3_wheel_speed_census.py** `[keep-reproduces]` — Fz-
  integration Phase 5 pre-implementation census (2026-09-03, thesis_
  notes.md "Fz-integration Phase 5: wheel-speed plausibility guard +
  ABS-domain fallback"). Raw-channel-name scan for ABS-domain wheel-
  SPEED alternatives (found abs_speed_fl/fr/rl/rr, 100Hz, both
  sessions) plus log_speed_rr dropout/stuck/spike diagnosis vs its own
  mates and ecu_speed. Re-run whenever a third session arrives or the
  guard's own thresholds need re-deriving.
- **inspect_wheel_speed_guard_before_after.py** `[keep-reproduces]` —
  Fz-integration Phase 5 (2026-09-03). Loads the pre-Phase-5 module
  straight from git HEAD (via a system temp file, not a tracked copy)
  and compares it against the current guarded behaviour on both real
  sessions: per-corner wheel_speed_source share, LS_ratio_r no-signal
  fraction before/after. The tool that caught BOTH real calibration
  bugs before shipping (std_min_kmh initially 10x too high; the mate-
  ratio check initially penalised v3's healthy log_speed_rl almost as
  often as its actually-faulty mate log_speed_rr). Re-run whenever the
  guard's thresholds or the wheel-speed channels change.
- **inspect_v3_pit_limiter_lap_census.py** `[keep-reproduces]` — Fz-
  integration Phase 4 (2026-09-03, thesis_notes.md "Fz-integration Phase
  4: pit-limiter-based out/in-lap classification"). Read-only census of
  lap_number/ecu_B_speedlimit_en/lap_distance boundaries on both real
  sessions before/after the fix -- the tool that found v3's real bug
  (last lap wrongly is_valid_for_analysis=True despite its final ~22s
  running under the pit limiter). Re-run whenever the lap-splitting
  logic changes or a third session arrives.
- **inspect_v3_pacejka_refit_evaluation.py** `[keep-reproduces]` — corner
  canonicalisation + refit evaluation work order (2026-09-03), Phase 2 and
  its same-day extension: runs the Pacejka B/C/D/E refit chain (up to 4
  iterations, each seeded from the previous iteration's own EKF beta
  instead of kinematic beta) on BOTH Dubai (confirmation) and v3, reusing
  modules.tyre_fit_auto.fit_session_pacejka/_fit_axle_pacejka and
  diagnostics.sideslip_ekf_pacejka directly. THE load-bearing provenance
  for PLAN.md BACKLOG A's "data-identified tyre curve" sub-item closure
  (thesis_notes.md "Refit-loop conclusion: structural non-convergence
  confirmed on two sessions, two failure directions") -- the run that
  found D grows without plateauing on both files across 4 iterations
  (Dugoff's own historical failure mode instead collapsed D). Re-run if
  this closure is ever revisited, or a further-iteration/production-
  adoption question is reopened.
