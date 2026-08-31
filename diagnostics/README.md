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
classification"). Category in brackets: **[keep-referenced]** = cited
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
