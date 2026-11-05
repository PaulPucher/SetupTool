# SetupTool

SetupTool analyses the lateral handling balance of a GT3 race car from
standard logged telemetry and turns repeated handling symptoms into one setup
recommendation per session. It is a desktop application (PyQt6) with a
SQLite store; the analysis chain also runs without the user interface.

## 1. What the tool does

For one exported session the analysis chain runs these steps in order:

1. **Load and parse** the Pi Toolbox ASCII export (narrow or wide layout).
   Only the channels listed in `config/channels.json` are read; each channel
   passes a range gate. Laps are split and classified (out lap, in lap,
   analysis lap).
2. **Corner detection**: corner brackets from steering and lateral
   acceleration, the apex, a speed class and five phases per corner (brake,
   turn-in, apex, early exit, late exit). Corners are matched across laps so
   every lap of the same corner shares one corner number.
3. **Vehicle state**: all channels resampled onto one time base, filtered,
   with a kerb mask from vertical acceleration.
4. **Vertical loads**: per-wheel loads from the damper force and travel
   channels through the motion-ratio table; if these channels are missing or
   implausible, the static weight split is used instead.
5. **Tyre fit and observer**: a reduced Magic Formula curve is fitted per
   axle for this session; a single-track extended Kalman filter estimates the
   sideslip angle with that curve. A consistency check on the filter
   innovations (NIS) decides whether the observer's sideslip is used; if the
   fit or the check fails, a kinematic sideslip estimate is used.
6. **Stiffness ratio and yaw stability**: slip angles and axle lateral
   forces give the effective cornering stiffness of each axle, expressed as
   a ratio to its linear value (CS ratio); the yaw-moment derivative
   dMz/dbeta measures open-loop yaw stability.
7. **Corner summaries and verdicts**: per corner, lap and phase the
   statistics of these quantities; an axle past the peak of its force curve
   is limiting (front: understeer, rear: oversteer), a negative dMz/dbeta is
   open-loop unstable. Floor events (CS ratio below the pure-lateral floor)
   are recorded separately.
8. **Recommendation**: a symptom counts only if it repeats on most analysis
   laps of a corner. Counted symptoms vote for setup levers through a
   mechanism table (`config/mechanisms.json`); the lever with the most net
   support is the session recommendation. Driver feedback, when entered, is
   shown as a separate layer and never changes the data-based result.

Vehicle quantities carry an accuracy level: 1 = configuration default,
2 = session measurement (setup sheet), 3 = logged sensor, 4 = lookup table.

## 2. Installation

Python 3.13 (tested). On Windows:

    py -3.13 -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt

On Linux or macOS use `python3.13 -m venv .venv` and
`source .venv/bin/activate`.

## 3. Use

Start the application from any working directory:

    python main.py

1. **Weekends**: "+ New" creates a race weekend or test (track, series, car
   number and date are required).
2. Open the weekend and create an **outing** with "+ New"; fill in the
   session data and, if available, the setup sheet (corner weights on the
   sheet replace the configured vehicle mass split).
3. **Load Outing** selects the exported session file.
4. **Analyse** runs the chain. The status lines show which sideslip
   estimate and which load path were used; the corner table shows the
   verdicts; the trace windows show the signals per corner and lap.
5. **Setup Recommendation -> Generate** builds the recommendation from the
   current analysis, setup sheet and driver feedback.
6. **Save** stores the outing.

Results are stored in `data/setuptool.db` (outing data and the analysis
summary) and `data/analysis_cache/` (the full analysis arrays, so a stored
outing reopens without re-running). PDF setup sheets and PNG figures are
written to the location chosen in the save dialog. Changes on the Settings
page are written into `config/parameters.json` and `config/channels.json`.

Headless analysis of one session file, run from the tool folder (the folder
that holds `main.py`):

    python -m modules.pipeline <session file> -o summary.json

writes a JSON summary: fit status, normalised stiffness K and friction
coefficient mu per axle, NIS health score, per-corner verdicts, counted
symptoms and the session recommendation (lever and direction). Without `-o`
the summary is printed. A file without a mandatory channel is refused with a
message naming the channel.

`config/data_roots.json` may list folders where session files are stored on
different machines; a saved file path is then found under any of them.

## 4. File overview

In the order of the analysis chain.

`config/`
- `channels.json`: channel whitelist, units, range gates, corner detection
  and speed-class thresholds, logger zero entries.
- `parameters.json`: vehicle description, estimator and classification
  parameters, accuracy levels.
- `car_data.json`: motion-ratio, anti-roll-bar and steering tables.
- `mechanisms.json`: symptom-to-lever mechanism table.
- `setup_parameters.json`: setup levers, value ranges and direction words.
- `setup_windows.json`: typical window of each lever.
- `data_roots.json`: optional folders that hold the session files.

`main.py`: start-up (database, main window).

`models/`
- `db.py`: SQLite engine and the tables for weekends, outings and drivers.

`modules/`
- `csv_parser.py`: reads the export, applies channel gates and zero
  entries, splits and classifies laps.
- `corner_analysis.py`: corner brackets, apex, speed class, phases, corner
  identity across laps, GPS projection.
- `accuracy_resolution.py`: resolves vehicle values and their accuracy level.
- `stability_analysis.py`: vehicle state, slip angles, axle forces, vertical
  loads, CS ratio and the corner summaries.
- `loads.py`: per-wheel loads from damper channels; longitudinal force.
- `yaw_stability.py`: yaw-moment derivative dMz/dbeta.
- `tyre_model_pacejka.py`: reduced Magic Formula tyre model.
- `tyre_fit_auto.py`: per-session tyre fit, plausibility checks, sideslip
  source selection.
- `observer.py`: extended Kalman filter, straight-line noise measurement,
  NIS check.
- `classifier.py`: cross-lap aggregation and the per-corner verdict.
- `limit_events.py`: floor events.
- `setup_state.py`: current setup values, legal steps, windows; setup-sheet
  measurement points.
- `setup_engine.py`: counted symptoms, mechanism votes, session
  recommendation and its text.
- `pipeline.py`: the analysis chain without the user interface; headless
  entry point.
- `storage.py`: repository paths, data roots and the analysis cache.

`core/`
- `display.py`: error text and drawing helpers for the trace windows.
- `figure_render.py`: plot styles and PNG export of the traces.
- `pdf_export.py`: setup and setdown sheet PDF.

`ui/`
- `main_window.py`: main window and navigation.
- `style.py`: colour palette and stylesheet.
- `icons/`: navigation icons.
- `views/weekends.py`: weekend list and weekend dialog.
- `views/outings.py`: outing list of one weekend.
- `views/outing_form.py`: outing form: setup sheet, analysis, verdicts,
  recommendation, driver feedback.
- `views/corner_trace_dialog.py`: corner and lap trace windows.
- `views/measurement_points_widget.py`: splitter and diffuser point input.
- `views/drivers.py`: driver list and driver dialog.
- `views/settings_view.py`: Settings page.
