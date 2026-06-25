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
- Currently 23 channels configured including sclu_yaw_rate and GPS/VBOX placeholders

## Data pipeline
1. Parser reads file, loads only channels in channels.json
2. Quality flags per channel (valid/partial/failed/missing)
3. Laps split by lap_number, verified against lap_time and lap_distance
4. Laps marked: is_fastest, is_valid_for_analysis
5. Corner segmentation (corner_analysis.py) on valid laps
6. Stability analysis (stability_analysis.py) — separate from parser, triggered from UI

## Corner segmentation
Steering angle threshold bracketing with hysteresis (entry 25°, exit 15°).
Lateral G apex detection, speed fallback, five-phase segment boundaries.
All parameters config-driven in channels.json corner_detection block.
Known limitation: corner count may vary 13-15 per lap on 16-corner layout.
Track-anchored detection identified as future improvement.

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
Follows Werner MA pipeline for cornering stiffness estimation.

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
  - No undocumented gates, no EMA — matches the Werner reference exactly
Module 5 — estimate_yaw_moment_stability(): Iz·ψ̈ → Mz_inertial,
  local centred 2 s OLS over [1, β, δ_f, v, ax], yaw rate excluded
  for multicollinearity with β via the kinematic identity β̇ = ay/v − ψ̇.
  Positive c_β = stabilising (Suzuka convention).
Module 6 — summarise_corners(): per-corner per-phase median + IQR aggregation,
  lap_filter argument (UI selector translates to lap numbers), apex_3 window
  expansion ±5 samples, placeholders for apex_position_x_m, apex_position_y_m,
  stable_corner_id (track-map module fills these later).

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
- 72 corners detected across 5 laps, per-corner output captures understeer signature

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
- Lap selector + Exclude In/Out toggle translate to a lap_filter list
- Collapsible "Stability Analysis" section between Setdown and Driver Feedback
- Per-corner cards with severity-based plain-English verdict
  ("strong understeer at turn-in", "destabilising yaw moment at apex", etc.)
- Cards grouped by severity: strong (red) → moderate (amber) → divider → normal (green)
- Each card expands to full per-phase table (CSf, CSr, Stab medians + IQR)
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
Phase 5 (stability analysis Level 1) functionally complete.
End-to-end pipeline: load CSV → Analyse → verdict-coloured per-corner cards in UI.

### Done
- Full PyQt6 app: dark theme, topbar logos, sidebar navigation
- Race Weekends, Drivers: CRUD, sortable lists
- Outing form: session, setup (3-col, per-corner dampers, mirror), setdown, feedback, comments
- PDF export: A4, light theme, print setup/setdown
- CSV parser: Pi Toolbox ASCII, selective loading, quality flags, lap splitting/verification
- pyqtgraph plot: stacked traces, crosshair, lap selector, auto-range, in/out exclusion
- Corner segmentation: steering threshold + lateral G apex + five phases
- Stability Modules 1-6: full documented Werner method, yaw stability, per-corner aggregation
- UI integration: Analyse button, background thread, lap-filter-aware,
  collapsible section, verdict-based corner cards with expand-for-details
- Shared colour-constant module in ui/style.py

### To-do (next session, in priority order)
1. Tighten verdict thresholds — 10/14 corners flagged "strong" on Dubai is too eager.
   CS thresholds need tightening, and "strong" should likely require combined conditions
   (CS drop AND negative stability) rather than either alone.
2. UI layout rework — corner grid: laps stacked vertically, corners horizontal within
   each lap and ordered by number, severity shown by cell colour.
3. Curb/jump exclusion — vertical-G or wheel-speed-deviation gate to flag samples
   affected by kerbs and exclude them from CS regression and stability windows.
4. Data lifecycle — Clear loaded data button, behaviour on loading a different CSV,
   verify edit+reload+reanalyse for existing outings.
5. Performance — UI lag with 72 cards; grid layout may help, profile before assuming.
   Module 5 subsampling + caching also deferred from this session.
6. Track-map module — dead-reckon v + ψ̇ with lap closure correction; cross-lap
   corner ID via spatial clustering of apex positions; numbered colour-coded
   corners as primary stability interface.
7. Level 2 Fy split — dynamic load transfer from roll stiffness balance.
8. Result persistence — cache analysis result on Outing model so re-opening
   doesn't recompute.
9. Settings UI (after engine parameters stable).
10. PyInstaller packaging.

## Known gaps
- Math channels pending (full Cosworth licence)
- GPS channels configured but not present in current sample file
- Track-anchored corner detection future work (needs track map module first)
- test_stability.py in project root — keep for now, useful for non-UI module testing