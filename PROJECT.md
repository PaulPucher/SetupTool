# SetupTool — Project Documentation

## What this is
A desktop application for racing engineers working with the Porsche 992 GT3R.
It combines logged vehicle data from the Cosworth datalogger with driver feedback
and session parameters to generate explainable setup recommendations.

## Background
The tool is designed to bridge the gap between raw data and actionable setup decisions,
in a format that makes sense in a real race weekend environment.
This is a bachelor thesis project at TUM, developed in cooperation with Proton Competition.

## Tech stack
- Python 3.10
- PyQt6 — desktop UI framework
- SQLAlchemy 2.0 + SQLite — database layer
- pandas — CSV parsing and data processing
- numpy — numerical processing
- scipy — signal filtering (Butterworth, filtfilt)
- pyqtgraph — data plotting
- reportlab + Pillow — PDF setup sheet generation
- PyInstaller — packaging to Windows executable (planned)
- Git + GitHub — version control

## Architecture principles
- UI and core logic are strictly separated — no business logic in UI files, no PyQt6 imports in core/ or modules/
- Config files are the source for car definition, channel mappings, and detection/estimation parameters
- Database stores only runtime data — outings, feedback, recommendations
- Data files (Cosworth .txt) stay on disk, outing stores the path only, re-parses on open
- Parser only loads channels listed in channels.json — never loads the full file
- Missing or faulty channels degrade gracefully — never crash the app
- Every derived quantity has a defined accuracy level (1-4), upgradeable independently
- Colour literals live in ui/style.py only — both stylesheet and widget code import from there

## Accuracy level system
Every physical quantity in the stability estimation has a cascade:
- Level 1: config default (always available, used now)
- Level 2: session measurement (setup sheet values per outing)
- Level 3: logged sensor data (direct measurement)
- Level 4: lookup table interpolation (kinematic corrections)
Each node upgrades independently without restructuring the pipeline.

## Folder structure
main.py — entry point
ui/ — all visual components, no business logic
core/ — pure Python logic, no Qt imports
models/ — SQLAlchemy database models
modules/ — domain specific: csv_parser, corner_analysis, stability_analysis
config/ — JSON config files and images

## Data model
Outings are the core unit. Each outing contains:
- car setup state before session (setup_data JSON)
- car setup state after session (setdown_data JSON)
- reference to Cosworth TXT file (csv_path) — re-parsed on open
- driver feedback (feedback_data JSON) — corner table, -5 to +5 per phase
- session parameters, comments
Multiple outings per weekend stored independently.

## Config files
config/car.json — Porsche 992 GT3R setup sheet template
config/channels.json — channel definitions, corner detection parameters
config/parameters.json — vehicle constants and stability estimation parameters
config/images/ — logos and car photo

## Vehicle constants (config/parameters.json)
- Wheelbase: 2.505 m (nominal, changes with kinematics — Level 4 refinement)
- Mass: 1356 kg (measured with setup adapters, 75 kg driver, 35 kg fuel)
- Corner weights: FL=290, FR=290, RL=395, RR=381 kg
- Front fraction: 42.8%, Rear fraction: 57.2%
- CoG to front axle (a): 1.433 m, CoG to rear axle (b): 1.072 m
- Yaw inertia Iz: 2082 kg·m² (estimated: m×a×b, ~10-20% error, Level 1)
- Steering ratio: 15.7 (read from lookup table at centre position, constant at Level 1)
- Yaw rate conversion: 0.10472 rad/s per rpm

## Stability estimation parameters (config/parameters.json)
- cs_filter_cutoff_hz = 2.0 (Butterworth lowpass on slip angles and Fy)
- cs_min_slip_angle_span_rad = 0.02 (window growth target)
- cs_linear_slip_threshold_rad = 0.021 (linear-zone gate)
- beta_washout_cutoff_hz = 0.05 (β drift removal)
- yaw_accel_filter_cutoff_hz = 5.0 (post-differentiation lowpass)
- stability_min_beta_span_rad = 0.01 (Module 5 validity gate)
- stability_regression_window_s = 2.0 (centred regression window)
- yaw_rate_to_radps = 0.10472

## CSV / Data file format
Pi Toolbox ASCII format (.txt). Key properties:
- One channel per {ChannelBlock}, own time vector
- European decimal notation (comma → period)
- Channel names include unit suffix — parser strips before matching
- Variable sample rates per channel
- Currently 26 channels configured, including sclu_yaw_rate and the real GPS
  position channels (log_gps_lat/log_gps_lon); gpsa_lat/gpsa_long/VBOX_* are
  configured but absent from the current sample file

## Data pipeline
1. Parser reads file, loads only channels in channels.json
2. Quality flags per channel (valid/partial/failed/missing)
3. Laps split by lap_number, verified against lap_time and lap_distance
4. Laps marked: is_fastest, is_valid_for_analysis
5. Corner segmentation (corner_analysis.py) on valid laps
6. Stability analysis (stability_analysis.py) — separate from parser, triggered from UI

## Corner segmentation
Dual-criterion bracketing with hysteresis: enter on steering angle (25°) OR
lateral G (0.6g); exit only when both drop below their thresholds (15°,
0.35g). Steering alone is systematically marginal in fast corners (delta
scales inversely with radius); lateral G answers "is the car cornering"
directly. Same-direction adjacent brackets separated by a short time gap are
merged (chicanes, which reverse direction, are left as separate brackets).
Brackets longer than 300 m are flagged "compound_corner" (sustained-G
double-apex corner, not two events linked by a straight).
Lateral G apex detection within each bracket, speed fallback if no steering
channel, five-phase segment boundaries. All parameters config-driven in
channels.json corner_detection block.
Cross-lap corner identity (assign_stable_corner_ids in corner_analysis.py):
corners are linked across laps by bracket-span overlap fraction along
lap_distance, not apex position (a single peak-G point moves within a
bracket lap-to-lap in a way the bracket boundaries don't). Connected
components are candidate clusters; same-lap exclusivity is a hard
constraint enforced by a deterministic seeded split where one lap's bracket
straddles what other laps detect as two distinct corners
("straddles_adjacent_corners" warning marks this). stable_corner_id is
consistent across laps for the same physical corner.

## Driver Feedback
Corner-by-corner table, configurable 1-30 corners.
Five phases per corner: Entry 1, Entry 2, Apex 3, Exit 4, Exit 5 (-5 to +5 integer).
-5 = undrivable understeer, 0 = neutral, +5 = undrivable oversteer.
Entry 1 corresponds to braking phase in data pipeline.
Saved as feedback_data JSON on Outing model.

## Stability estimation
Module lives in modules/stability_analysis.py.
Pure Python/numpy/scipy, no Qt imports.
Scientific basis: kinematic derivation from logged signals, no tyre model required.
Framework after Werner (2021) / Milliken, adapted for measurement-
direct use on the 992 GT3R: effective cornering stiffness estimated
from logged data in place of Werner's tyre-model evaluation (no
validated Pacejka set available); yaw-damping completion via wheel
loads planned (WP5b).

### Signal chain
ecu_speed + sclu_yaw_rate + log_asteer → slip angles α_f, α_r
log_acc_y + mass + weight distribution → lateral forces Fy_f, Fy_r
dFy/dα windowed OLS + section-blend → cornering stiffness Cα → CS ratio
sclu_yaw_rate differentiated + Iz → Mz_inertial → local OLS → dMz/dβ stability

### Modules built and verified
Module 1 — prepare_vehicle_state(): unit conversions, common 50 Hz time base
Module 2 — estimate_sideslip(): kinematic integration, washout 0.05 Hz
Module 3 — estimate_slip_angles(): full arctan bicycle model, 2 Hz Butterworth
Module 4a — estimate_lateral_forces(): Fy = m×ay, static weight split (Level 1)
Module 4b — estimate_cornering_stiffness(): Werner MA, full documented method
  - Window OLS slope C_window with R²
  - Monotonic-section OLS slopes C_section, weighted by α-span
  - R²-weighted blend: C_α = w·C_window + (1-w)·C_section
  - Linear reference held unless window entirely inside ±0.021 rad
  - No undocumented gates, no EMA — the estimation machinery itself is
    this project's adaptation of Werner's approach, not a tyre-model
    evaluation (see thesis_notes.md §1)
Module 5 — estimate_yaw_moment_stability(): Iz·ψ̈ → Mz_inertial,
  local centred 2 s OLS over [1, β, δ_f, v, ax], yaw rate excluded
  for multicollinearity with β via the kinematic identity β̇ = ay/v − ψ̇.
  Sign convention per Werner (2021) §2.2.3: positive dMz/dbeta =
  restoring = stable.
Module 6 — summarise_corners(): per-corner per-phase median + IQR aggregation,
  lap_filter argument (UI selector translates to lap numbers), apex_3 window
  expansion ±5 samples. apex_position_x_m/y_m filled from log_gps_lat/lon via
  a local equirectangular projection (origin = first GPS sample, fine at
  track scale); stable_corner_id passed through from the cross-lap
  clustering in corner_analysis.py.

### Verified output on Dubai sample (5 valid laps, 50 Hz, 40 800 samples)
- β: −4.29° to +2.91°, mean abs 0.87°, std 1.11°
- α_f: ±6.1°, mean abs 1.64° (front > rear = understeer signature ✓)
- α_r: ±4.4°, mean abs 1.00°
- Fy_f: ±9.6 kN
- Cα front mean 114 k N/rad, Cα rear mean 176 k N/rad (rear stiffer ✓ for Porsche load)
- CS ratio front mean 0.765, rear mean 0.851 (rear more stable than front ✓ understeer)
- Implied C_linear_ref front 158 k / rear 190 k N/rad (within expected 80-180 k band)
- Yaw acceleration ±5.5 rad/s², Mz_inertial ±11 kNm
- Stability observed median 2547 Nm/deg, 93 % positive (stabilising), 7 % negative
- 64 corners detected across 5 laps (dual-criterion detection + bracket-merge),
  clustering into 15 stable cross-lap corner identities (11 full 5-lap
  clusters + 3 compound-straddle partial + 1 genuine singleton)

### Documented Level 1 limitations (for thesis)
1. Static weight split for Fy (overstates rear on roll-stiff GT3)
2. Iz from m·a·b bicycle estimate (~10–20 % error, all Mz scales linearly)
3. Steering ratio constant (±25 % real variation)
4. β kinematic integration with washout drift correction
5. Accelerometer assumed at CoG (mounting offset neglected)
6. No yaw damping term in Mz (Werner himself drops this for the same reason)
7. Closed-loop derivative — driver in the loop affects c_β
8. Bicycle model: identical slip per axle

### Upgrade paths defined
- Level 3: GPS heading for β (VBOX_Heading — channel in config, not in current file)
- Level 2: load transfer correction for Fy split (next refinement)
- Level 4: steering ratio lookup table (2D: wheel travel × steering stroke)
- Level 4: damper force for actual wheel loads (channels expected in future data)

## UI integration of stability analysis
ui/views/outing_form.py:
- "Analyse" button in the Data section (enabled once a CSV is loaded)
- StabilityAnalysisThread runs Modules 1–6 in the background
- Lap selector: single-select table with an explicit "All laps" row;
  click-to-select, click-again-to-deselect (toggles back to "All laps",
  highlight moves visibly rather than leaving no row selected), plus a
  "Clear Selection" button; + Exclude In/Out toggle. Both translate to a
  lap_filter list.
- Collapsible "Stability Analysis" section between Setdown and Driver Feedback
- Grid keyed by stable_corner_id, not per-lap corner_number: one column per
  stable id found across the analysed laps, ascending; a lap missing that
  corner shows a dim NEUTRAL placeholder cell ("—", not clickable) instead
  of a gap or shifted numbering. Cell label reads "C{stable_corner_id}".
- Each cell expands inline to a full per-phase table (CSf, CSr, Stab
  medians + IQR)
- Click "→ plot" jumps the channel plot to the corner's apex time

## UI style
Dark theme. Accent: amber #C0A060. Background: #141414.
PDF sheets: light theme, green accent #2d6a35.
ui/style.py exports colour constants (BG, PANEL, ACCENT, OK, WARN, BAD, NEUTRAL, etc.)
used both by the global stylesheet and by widget code for status colouring.

## Naming conventions
- outing not session
- SetupTool not RacingSetupTool
- table names plural snake_case

## Hard rules
- no PyQt6 in core/ or modules/
- no business logic in ui/
- all models import Base/Session from models/base.py only
- parser only loads channels in channels.json
- missing channels degrade gracefully
- data files stay on disk, outing stores path only
- colour literals only in ui/style.py
- HANDOVER.md and generate_handover.py local only, never pushed

## File responsibilities
- main.py — entry point
- ui/style.py — stylesheet + colour constants
- ui/main_window.py — window, topbar, sidebar, navigation
- ui/views/weekends.py — race weekends list
- ui/views/weekend_dialog.py — create/edit weekend
- ui/views/outings.py — outings list per weekend
- ui/views/outing_form.py — full outing form with data, plot, setup, stability, feedback
- ui/views/drivers.py — drivers list
- ui/views/driver_dialog.py — create/edit driver
- models/base.py — engine, Session, Base, init_db
- models/driver.py — Driver model
- models/raceweekend.py — RaceWeekend model
- models/outing.py — Outing model (includes csv_path)
- modules/__init__.py — empty package marker
- modules/csv_parser.py — Pi Toolbox ASCII parser, lap splitting, verification
- modules/corner_analysis.py — corner segmentation, five-phase boundaries
- modules/stability_analysis.py — full Modules 1–6 stability pipeline
- core/config_loader.py — reads car.json
- core/pdf_export.py — A4 setup/setdown PDF
- config/car.json — GT3R setup template
- config/channels.json — channel definitions + corner detection config
- config/parameters.json — vehicle constants + stability estimation parameters

## Current status
WP1 complete (cross-lap corner identity + detection robustness), baseline
repo cleanup done. Modules 1-6, corner segmentation, and the UI grid all
reflect the current dual-criterion, interval-overlap-clustered state.
WP2 (recommendation engine) is next — see PLAN.md for the full work plan.

### Done
- Full PyQt6 app: dark theme, topbar logos, sidebar navigation
- Race Weekends, Drivers: CRUD, sortable lists
- Outing form: session, setup (3-col, per-corner dampers, mirror), setdown, feedback, comments
- PDF export: A4, light theme, print setup/setdown
- CSV parser: Pi Toolbox ASCII, selective loading, quality flags, lap splitting/verification
- pyqtgraph plot: stacked traces, crosshair, lap selector (toggle/clear), auto-range, in/out exclusion
- Corner segmentation: dual-criterion (steering OR ay) bracketing, same-
  direction bracket-merge, compound-corner flagging, five phases
- Cross-lap corner identity: interval-overlap + connected-components
  clustering with same-lap-exclusive seeded splitting; stable_corner_id +
  GPS-derived apex position
- Stability Modules 1-6: full documented Werner method, yaw stability,
  kerb/jump exclusion, data-driven AND-logic severity classification,
  per-corner aggregation
- UI integration: Analyse button, background thread, lap-filter-aware,
  collapsible section, stable_corner_id grid with placeholder cells,
  expand-for-details
- Shared colour-constant module in ui/style.py

### To-do (next session, in priority order)
1. WP2 — Recommendation engine framework (modules/recommendation.py,
   config/recommendations.json, rule-based ranked suggestions with an
   evidence trail). Full spec in PLAN.md.
2. WP3 — Driver feedback vs. analysis comparison view.
3. WP3b — Track template + corner naming map (official turn labels over
   stable_corner_id; GPS-based track map).
4. WP4 — Data lifecycle (Clear Data, reset-on-reload, verify edit/reopen cycle).
5. WP5 — Result persistence (cache analysis on the Outing model).
6. WP6 — Performance (cache Modules 1-5 output across lap-filter changes).
7. Level 2 Fy split — dynamic load transfer from roll stiffness balance.
8. Settings UI (after engine parameters stable).
9. PyInstaller packaging.

## Known gaps
- Math channels pending (full Cosworth licence)
- GPS channels (log_gps_lat/log_gps_lon) present, whitelisted, and used for
  apex_position_x_m/y_m; a full track-map view (plotting the trace, WP3b)
  is still future work
- test_stability.py in project root — keep for now, useful for non-UI module testing