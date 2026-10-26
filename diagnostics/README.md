# Diagnostics

Read-only scripts beside the app; not imported by it. Run from the project
root, e.g. `python -m diagnostics.inspect_closeout_l`.

Rewritten 2026-10-22 (pruning step P1c). [2026-10-22, recommendation
overhaul: generate_decision_layer_figure.py and smoke_test_settings_view.py
deleted with the decision frame and its Settings section; the rendered
plots_decision_layer/ figures stay as record.] Every script, result and record
from before the pruning is preserved at git tag `pre-prune-2026-10-22`
(`git show pre-prune-2026-10-22:diagnostics/<script>.py`). References to
deleted scripts in config, PLAN.md and thesis_notes.md carry the note
"(script at tag pre-prune-2026-10-22)".

## Keep rule (author, 2026-10-22)

A script stays only if it runs on the current code AND is needed for one of:
(a) re-deriving a current config value under the standing re-derivation rule;
(b) producing a thesis figure (the K3 figure sources);
(c) tooling the app or the tests use.
Everything else is deleted at commit time (disposal rule, CLAUDE.md).
`results_*` folders, plot folders and plots_INDEX.md are outputs, not
scripts, and are not covered by this rule.

## Kept scripts

(a) Re-derivation of current config values, fit-L chain, 14-session
development set (session list inlined in each script):
- inspect_threshold_rederivation_l.py -- CS STRONG / MODERATE thresholds (p10 / p30) and the floor-event census.
- inspect_cs_margin_l.py -- cs_margin (the +/-1 % verdict-sensitivity sweep); imports _is_flip from inspect_metrology_phase1_analysis.py.
- inspect_closeout_l.py -- NIS gate thresholds (threshold_use_ekf / threshold_warn).
- inspect_q_phys.py -- observer Q (held-over-step upper bound); also (c): tests/test_tyre_fit_literature.py reads its NBR 29 output.
- inspect_nis_window_share.py -- NIS window-share census behind the gate (author-named, 2026-10-22).
- inspect_drag_sensitivity.py -- drag c_D A sensitivity behind drag_coeff.
- inspect_cs_margin_combined.py -- combined observer + vehicle-input CS boundary margin behind classification.cs_boundary_margin_front / _rear (work order 4a, 2026-10-22; values ruled 2026-10-22, 4e); reuses CANDIDATES / keep_awake from inspect_cs_margin_l.py.
- inspect_engine_4e.py -- setup engine before / after work order 4e on the 14 sessions (recommendation changes, ranking-only effect, session-layer cell shares, log_abs_pos census), results results_engine_4e_20261023/. Referenced by thesis_notes 'Work order 4e' (CLAUDE.md Referenced class); it re-derives no config value; KEPT by author ruling 2026-10-23 (4g item 6: referenced in thesis_notes), an exception to the (a)-(c) rule above. Reuses CANDIDATES / keep_awake from inspect_cs_margin_l.py.
- inspect_engine_4f.py -- setup engine before / after work order 4f on the 14 sessions (snapshot / compare of the recommended change per corner on the 4e summaries, lever coverage per symptom x phase x speed), results results_engine_4f_20261023/. Referenced by thesis_notes 'Work order 4f'; imports CANDIDATES from inspect_cs_margin_l.py and reads results_engine_4e_20261023/ (so inspect_engine_4e's results are its input). Re-derives no config value; KEPT by author ruling 2026-10-23 (4g item 6: referenced in thesis_notes). Also mode list <tag>: setup recommendation and cockpit line per corner (4g); mode session <tag>: session setup recommendation per session with net support, supporting / opposing corners and alternatives (4h, results session_post4h.txt, referenced by thesis_notes 'Work order 4h').

(b) Thesis figure sources (K3):
- inspect_corner_distribution.py -- CS ratio / stability distributions per corner.
- inspect_step2_chair_plots.py -- chair-comparable CS trace plots; the style source core/figure_render.py and the trace dialog cite.
- inspect_v3_sawtooth_mechanism.py -- CS sawtooth mechanism figure (uses inspect_step2_chair_plots.py).
- inspect_metrology_phase1_analysis.py -- metrology sensitivity figures from results_metrology_*.
- inspect_metrology_phase1_corner_map.py -- metrology corner-map figure.
- inspect_fz_before_after.py -- static vs measured wheel-load figure.
- inspect_v3_wheel_load_showcase.py -- damper wheel-load showcase figures.
- inspect_v3_wheel_load_comparison_figure.py -- static-split vs damper wheel-load figure.
- inspect_v3_wheel_load_reconstruction_figure.py -- reconstructed-corner figure.
- inspect_v3_reconstruction_ground_truth.py -- reconstruction ground truth (cited in modules/wheel_loads.py).
- inspect_v3_fr_gauge_forensics.py -- FR damper-gauge forensics figure.
- inspect_prc_v3_sample_rates.py -- v3 channel sample-rate census.

(c) Tooling:
- inspect_kinematic_inputs.py -- tests/test_tyre_fit_literature.py imports steering_straights from it (straights definition).
- scan_channels.py -- channel discovery for a new data file.
- inspect_channel_census.py -- channel census (channel census rule).
- inspect_lap_census.py -- lap census; imports read_channels from inspect_channel_census.py.
- inspect_native_channel_rates.py -- native channel rates behind the common grid rate (config _comment_grid_rate).
- generate_channel_requirements.py -- regenerates docs/channel_requirements.md from the code and the channel export list. Also imported by export_labeled_dev.py (CHANNEL_NOTES).
- inspect_mu_uncertainty.py -- work order DEV-3 (2026-10-24): re-derives config tyre_fit_auto.mu_ceiling_edge_u (p95 combined relative uncertainty of mu_demo, 14 dev sessions), results results_mu_uncertainty_20261024/. Keep: Reproduces (re-derives a current config value; named in its derived_from).
- inspect_wheel_load_plausibility.py -- work order DEV-4 (2026-10-24): census behind config wheel_loads.plausibility_min_static_fraction (straight-line damper total / static weight, 14 dev + 8 labelled), results results_wheel_load_plausibility_20261024/. Keep: Reproduces (named in that key's derived_from).
- export_labeled_dev.py -- work order DEV-1 (2026-10-24): exports the labelled development set (weekends Labeled PRC / Labeled NBR, 8 outings) via the app's cache identity or the app chain, results results_labeled_dev_20261024/. Referenced by PLAN.md STATUS (DEV-1). Keep: Reproduces -- it exports the evaluation package (author ruling 2026-10-24, DEV-4 item 5); output folder as first argument (results_labeled_dev_20261024, _schema26, _schema27).
- smoke_test_analysis_lock.py -- headless Qt smoke test, analysis lock.
- smoke_test_measurement_points_widget.py -- headless Qt smoke test, splitter / diffuser widget.
