# Thesis figure export

Small scripts that write one CSV set (`<dir>/data/`) and one pgfplots
`.tex` (`<dir>/plots/`) per thesis figure or table, so a re-run updates the
thesis directly. `<dir>` comes from `config/thesis_figures.json`
(`thesis_figures_dir`); every script takes `--out DIR` to override it.
Separate from `diagnostics/` and its disposal rule (author ruling
2026-10-24); whether the folder ships with the submission is open.

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

Tables (CSV plus a booktabs body for tabularx):

- `tab_sessions.py` (tab:sessions), `tab_example_verdicts.py`
  (tab:example_verdicts), `tab_observer_health.py` (tab:observer_health;
  the reset count needs a read-only re-run per session).

Helpers: `find_example_corner.py` lists candidate corners for
fig:example_corner (the author picks); `common.py` holds outing
resolution, the app's cache guards, the read-only chain, thresholds from
config, the phase labels, the CS symbol, the name guard and the neutral
labels.

Anonymisation (hard rule): figures and tables carry neutral labels
`<code> <n>` (weekend code from `local/labels.json`, DB outing number) and
driver codes D1-Dn. `local/` is git-ignored: `labels.json` (author-
maintained: weekend name -> code, `_guard_tokens`) and the label maps
`label_map_<tag>.txt`. Every CSV and .tex passes the name guard (driver and
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
    python -m tools.thesis_figures.find_example_corner --outings 11,12,13,14,15,16 --top 3

Refusals: an outing whose damper path is rejected (excluded from the thesis
evaluation, ruling 2026-10-24) needs `--allow-excluded`; a degenerate fit or
a kinematic fallback needs `--allow-status`; both then plot the session
marked. Scripts on stored results refuse a stale payload and name the
outing: re-analyse it in the app first. tab_sessions lists excluded and
not-analysable sessions with their reason.

The `.tex` files hold only the tikzpicture (or the table body) and read
their CSVs relative to the thesis root (`figures/data/...`) with
`comment chars={\#}`.
