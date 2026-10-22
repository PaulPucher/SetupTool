# Diagnostics

Read-only scripts beside the app; not imported by it. Run from the project
root, e.g. `python -m diagnostics.inspect_closeout_l`.

Rewritten 2026-10-22 (pruning step P1c). Every script, result and record
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
- generate_decision_layer_figure.py -- decision-layer flow and lever coverage figures (layer retires with D1).

(c) Tooling:
- inspect_kinematic_inputs.py -- tests/test_tyre_fit_literature.py imports steering_straights from it (straights definition).
- scan_channels.py -- channel discovery for a new data file.
- inspect_channel_census.py -- channel census (channel census rule).
- inspect_lap_census.py -- lap census; imports read_channels from inspect_channel_census.py.
- inspect_native_channel_rates.py -- native channel rates behind the common grid rate (config _comment_grid_rate).
- generate_channel_requirements.py -- regenerates docs/channel_requirements.md from the code and the channel export list.
- smoke_test_analysis_lock.py -- headless Qt smoke test, analysis lock.
- smoke_test_measurement_points_widget.py -- headless Qt smoke test, splitter / diffuser widget.
- smoke_test_settings_view.py -- headless Qt smoke test, Settings cost-function section.
