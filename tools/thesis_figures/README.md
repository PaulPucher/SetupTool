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
  groups, one panel each with the front (solid) and rear (dashed) curve in
  the axle colours (author 2026-10-30); stored payloads only.
- `fig_example_lap.py` (fig:example_lap) -- consecutive corners of the
  fastest analysis lap; read-only re-run; dMz/dbeta is the session profile.
  Panel heights 0.18 / 0.32 / 0.22 text width, CS axis stopped just above
  the cap of 1 (author 2026-10-30).
- `fig_verdict_repeatability.py` (fig:verdict_repeatability) -- one track;
  corners matched across sessions by lap distance; stored payloads only.
- `fig_balance_speed_class.py` (fig:balance_speed_class) -- (before, after)
  pairs; engine cell shares per speed class; stored payloads only.
  Neither this nor fig_verdict_repeatability is used in the thesis; their
  outputs (old session labels) were deleted from the figure folder
  (author 2026-10-30).
- `fig_session_overview.py` (fig:session_overview) -- per session the
  per-lap cell shares US / OS / INST with the counted symptoms (a small gray
  0 on a bar with cells but no counted symptom); a fourth tick-label line
  "(dev.)" marks sessions whose data overlap the development data
  (tab_sessions dev_overlap; replaces the dagger, author 2026-10-30; not
  the threshold pool, which is tab_sessions in_sample); stored payloads only.
- `fig_symptom_phase.py` (fig:symptom_phase) -- per track and phase the
  per-lap cell shares; stored payloads only.
- `fig_corner_phases.py` (fig:corner_phases) -- one corner of one lap:
  throttle, |steering| and |a_y| with the bracket thresholds, own event
  lines and the stored phase bounds; refuses a corner whose own events
  differ from the stored bounds by more than 2 m. No analysis re-run.
- `fig_cs_reading.py` (fig:cs_reading) -- Fy/Fz over alpha per axle for one
  corner, the window closest to the phase median, k_tilde / k_lin / k_win
  through the window start; each axle on its own lap and phase; full read-only
  re-run. Bracket samples thinned for display to every 4th sample
  (BRACKET_EVERY_N, stated in the CSV header); window samples complete.
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
- `fig_comparison_matrix.py` (fig:comparison_matrix) -- per session the
  outcome of the driver-statement and the setup-change comparison as one
  symbol each (shape carries the outcome) and the tire age; the outcomes are
  the author's assignments in tab:feedback / tab:setup_comparison, held in the
  script; tire age and session order from fig_session_overview.csv.
- `fig_inst_tire_age.py` (fig:inst_tire_age) -- per session the INST share of
  the per-lap cells over the tire age, NBR / PRC markers; read from
  fig_session_overview.csv (same population). Descriptive only. Zero-line
  labels hang below the line in a tick-free strip (ZERO_STRIP), so no label
  meets a marker, another label or the axis.

Render-only: tab_sessions, tab_example_verdicts, tab_observer_health,
fig_example_corner, fig_example_fit, fig_cs_distribution,
fig_session_overview, fig_symptom_phase, fig_corner_phases, fig_comparison_matrix,
fig_inst_tire_age,
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
  (tab:example_verdicts; with `--appendix <ids>` one table per session,
  tab_verdicts_<label>.tex, each with a line below it: the data-layer session
  recommendation as lever and direction with net support, else "none", and
  the alternatives, in the thesis wording (anti-roll bar, low-speed,
  higher / lower; author 2026-10-30); no setup values. A value that rounds to zero prints
  0.00, never -0.00), `tab_observer_health.py` (tab:observer_health;
  the reset count needs a read-only re-run per session).

Appendix screenshots: `app_screenshots.py --outing 26` writes
feedback_input.png and recommendation_output.png to `<dir>/screenshots/`
from the app's own OutingForm widgets with the stored layers (no outing
opened, nothing saved); setup from -> to values and step sizes print as
"[hidden]". Keep reason: reproduces the appendix screenshots (author
ruling 2026-10-30). Needs a Windows desktop session (native Qt platform for
the system fonts). `--blank` instead renders the feedback input of a new,
empty form (6 corners, defaults, speed class "-", no outing or weekend row)
cropped to the corner table and the scale legend at 2x, and overwrites
feedback_input.png (author 2026-10-30; the app has a dark theme only).

Outside this folder: `tools/pure_lateral_floor.py` reproduces the
pure-lateral CS floor (-0.53, limit_events.PURE_LATERAL_FLOOR) by
minimising the normalised Magic Formula slope at C = 3 over E in [-1, 1]
(step 0.01).

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
    python -m tools.thesis_figures.tab_example_verdicts --appendix 17,18,19,20,21,22,25,26,27,28 --tag final
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
    python -m tools.thesis_figures.fig_comparison_matrix --tag final
    python -m tools.thesis_figures.fig_inst_tire_age --tag final
    python -m tools.thesis_figures.find_example_corner --outings 11,12,13,14,15,16 --top 3

Refusals: an outing whose damper path is rejected (excluded from the thesis
evaluation) needs `--allow-excluded`; a degenerate fit or
a kinematic fallback needs `--allow-status`; both then plot the session
marked. Scripts on stored results refuse a stale payload and name the
outing: re-analyse it in the app first. tab_sessions lists excluded and
not-analysable sessions with their reason.

Colours: the scripts name only the semantic colours defined once in the
thesis (`source/definitions/figure_colors.tex`, TUM core palette; the
anniversary colours are not used, the class defines them in percent under
optCMYK): axleFront / axleRear (front solid, rear dashed), sigRaw / sigProc
(logged vs processed signal: filter_chain, kerb_hit, beta_washout),
phaseB / phaseT / phaseA / phaseE1 / phaseE2 (`common.PHASE_FILL`:
corner_phases, example_corner), symUS / symOS / symINST
(`common.SYMPTOM_FILL`: session_overview, symptom_phase), exclusion with
exclMargin / exclStrike (kerb mask: kerb_hit, cs_kerb), okBand / edgeBand
(k_sessions bands; nis_health uses edgeBand and its own stronger in-band tint TUMGreen!25,
set in the script, author 2026-10-30), boundary (decision
boundaries and thresholds), trackNBR / trackPRC (inst_tire_age; PRC dark
gray dashed, so green marks only INST and the acceptable bands).
example_corner draws the fit-sample range limits thin and dotted in the axle colours. Line styles and markers repeat the meaning for black and
white print.

The `.tex` files hold only the tikzpicture (or the tabularx environment) and read
their CSVs relative to the thesis root (`figures/data/...`) with
`comment chars={\#}`.
