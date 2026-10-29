# Thesis figure export

Small scripts that write one CSV set (`<dir>/data/`) and one pgfplots
`.tex` (`<dir>/plots/`) per thesis figure or table, so a re-run updates the
thesis directly. `<dir>` comes from `config/thesis_figures.json`
(`thesis_figures_dir`); every script takes `--out DIR` to override it.
Separate from `diagnostics/` and its disposal rule; whether the folder
ships with the submission is open.

Read-only on the app: the DB is opened read-only, no cache, sidecar or
config is written. Sessions are app outing IDs. Output names are stable
(the chapter's `\input` never changes); outing, lap and corner go into the
CSV header only.

Figures:

- `fig_example_fit.py` (fig:example_fit) -- one outing; partial read-only
  re-run of fit L (stops before the observer).
- `fig_k_sessions.py` (fig:k_sessions) -- several outings; stored,
  identity-valid payloads only.
- `fig_example_corner.py` (fig:example_corner) -- one outing, lap, corner;
  full read-only re-run, five panels.
- `fig_cs_distribution.py` (fig:cs_distribution) -- one or two session
  groups; stored payloads only.
- `fig_example_lap.py` (fig:example_lap) -- consecutive corners of the
  fastest analysis lap; read-only re-run; dMz/dbeta is the session profile.
- `fig_verdict_repeatability.py` (fig:verdict_repeatability) -- one track;
  corners matched across sessions by lap distance; stored payloads only.
- `fig_balance_speed_class.py` (fig:balance_speed_class) -- (before, after)
  pairs; engine cell shares per speed class; stored payloads only.
- `fig_session_overview.py` (fig:session_overview) -- per session the
  per-lap cell shares US / OS / INST with the counted symptoms; stored
  payloads only.
- `fig_symptom_phase.py` (fig:symptom_phase) -- per track and phase the
  per-lap cell shares; stored payloads only.
- `fig_corner_phases.py` (fig:corner_phases) -- one corner of one lap:
  throttle, |steering| and |a_y| with the bracket thresholds, own event
  lines and the stored phase bounds; refuses a corner whose own events
  differ from the stored bounds by more than 2 m. No analysis re-run.
- `fig_cs_reading.py` (fig:cs_reading) -- Fy/Fz over alpha per axle for one
  corner, the window closest to the phase median, k_tilde / k_lin / k_win
  through the window start; each axle on its own lap and phase; full read-only
  re-run.
- `fig_filter_chain.py` (fig:filter_chain) -- one corner on a time axis: a_y
  as logged vs low-passed as in the force estimate; yaw acceleration as raw
  gradient, as in the force estimate and as in the yaw-moment regression.
  No analysis re-run.
- `fig_kerb_hit.py` (fig:kerb_hit) -- one kerb strike: a_z with the
  criterion, strike samples, widened mask and margin; a_y below. No re-run.
- `fig_cs_kerb.py` (fig:cs_kerb) -- CS over one kerb corner as computed and
  a figure-only counterfactual without the kerb mask; full read-only re-run.
- `fig_beta_washout.py` (fig:beta_washout) -- one lap: plain integral of the
  kinematic sideslip rate vs the washed-out estimate. No re-run.
- `fig_dmz_reading.py` (fig:dmz_reading) -- partial-regression plots of
  dMz/dbeta for one stable and one unstable corner phase; checks its
  grid-point math against the module profile; full read-only re-run.
- `fig_nis_health.py` (fig:nis_health) -- NIS window share over one lap
  with the gate band; checks the session health score; full read-only re-run.

Render-only: tab_sessions, tab_example_verdicts, tab_observer_health,
fig_example_corner, fig_example_fit, fig_cs_distribution,
fig_session_overview, fig_symptom_phase, fig_corner_phases,
fig_cs_reading and the process figures of Ch. 3 (fig_filter_chain,
fig_kerb_hit, fig_cs_kerb, fig_beta_washout, fig_dmz_reading,
fig_nis_health) take `--render-only` and write
the `.tex` from the CSVs already in `<dir>/data/`, without loading a
session (layout iterations); a full run writes the CSVs and then renders
the same way.

Tables (CSV plus a booktabs environment -- tabularx, or a plain tabular
with fixed columns in \small; the chapter holds the table float, caption
and label):

- `tab_sessions.py` (tab:sessions), `tab_example_verdicts.py`
  (tab:example_verdicts), `tab_observer_health.py` (tab:observer_health;
  the reset count needs a read-only re-run per session).

Helpers: `find_example_corner.py` lists candidate corners for
fig:example_corner (the author picks); `common.py` holds outing
resolution, the app's cache guards, the read-only chain, thresholds from
config, the phase labels, the CS symbol, the name guard and the neutral
labels.

Anonymisation (hard rule): figures and tables carry neutral labels -- the
outing's entry in `local/labels.json` `_outing_labels` (the author's
numbering), else `<code> <n>` (weekend code, DB outing number) -- and
driver codes D1-Dn (by first appearance within `_driver_code_weekends`
when set). `local/` is git-ignored: `labels.json` (author-maintained:
weekend name -> code, `_outing_labels`, `_driver_code_weekends`,
`_guard_tokens`), `overlap.json` (tab_sessions `--overlap`) and the label
maps `label_map_<tag>.txt`. Every CSV and .tex passes the name guard (driver and
outing names from the app DB) before it is written.

Commands:

    python -m tools.thesis_figures.fig_example_fit --outing 12 --tag r2
    python -m tools.thesis_figures.fig_k_sessions --outings 11,12,15,16,13,14 --separator-after 16 --tag r2
    python -m tools.thesis_figures.fig_example_corner --outing 14 --lap 53 --corner 1 --tag r2
    python -m tools.thesis_figures.fig_cs_distribution --group "NBR=11,12,15,16" --group "PRC=13,14" --tag r2
    python -m tools.thesis_figures.fig_example_lap --outing 12 --s-start 3200 --s-end 4000 --tag r2
    python -m tools.thesis_figures.fig_verdict_repeatability --outings 11,12,15,16 --tag r2
    python -m tools.thesis_figures.fig_balance_speed_class --pair 13,14 --pair 11,12 --tag r2
    python -m tools.thesis_figures.tab_sessions --outings 9,10,11,12,13,14,15,16 --tag r2
    python -m tools.thesis_figures.tab_example_verdicts --outing 12 --tag r2
    python -m tools.thesis_figures.tab_observer_health --outings 11,12,13,14,15,16 --tag r2
    python -m tools.thesis_figures.fig_corner_phases --outing 26 --lap 16 --corner 8 --tag final
    python -m tools.thesis_figures.fig_cs_reading --outing 26 --corner 17 --front 18:exit_4 --rear 15:exit_5 --tag final
    python -m tools.thesis_figures.fig_filter_chain --outing 26 --lap 15 --corner 17 --tag final
    python -m tools.thesis_figures.fig_kerb_hit --outing 26 --lap 15 --tag final
    python -m tools.thesis_figures.fig_cs_kerb --outing 26 --lap 15 --corner 1 --tag final
    python -m tools.thesis_figures.fig_beta_washout --outing 26 --lap 15 --tag final
    python -m tools.thesis_figures.fig_dmz_reading --outing 17 --unstable 11:exit_4 --tag final
    python -m tools.thesis_figures.fig_example_lap --outing 26 --lap 15 --tag final
    python -m tools.thesis_figures.fig_nis_health --outing 26 --lap 15 --tag final
    python -m tools.thesis_figures.find_example_corner --outings 11,12,13,14,15,16 --top 3

Refusals: an outing whose damper path is rejected (excluded from the thesis
evaluation) needs `--allow-excluded`; a degenerate fit or
a kinematic fallback needs `--allow-status`; both then plot the session
marked. Scripts on stored results refuse a stale payload and name the
outing: re-analyse it in the app first. tab_sessions lists excluded and
not-analysable sessions with their reason.

The `.tex` files hold only the tikzpicture (or the tabularx environment) and read
their CSVs relative to the thesis root (`figures/data/...`) with
`comment chars={\#}`.
