# SetupTool — Project Documentation

## What this is
A desktop application for racing engineers working with the Porsche 992 GT3R.
It combines logged vehicle data from the Cosworth datalogger with driver feedback
and session parameters to generate explainable setup recommendations.

## Background
The tool is designed to bridge the gap between raw data and actionable setup decisions,
in a format that makes sense in a real race weekend environment.

## Tech stack
- Python 3.10
- PyQt6 — desktop UI framework
- SQLAlchemy 2.0 + SQLite — database layer
- pandas — CSV parsing and data processing
- numpy — numerical processing for corner detection
- pyqtgraph — data plotting (installed, not yet wired into UI)
- reportlab + Pillow — PDF setup sheet generation
- PyInstaller — packaging to Windows executable
- Git + GitHub — version control

## Architecture principles
- UI and core logic are strictly separated — no business logic in UI files, no PyQt6 imports in core/
- Config files are the source for car definition and parameters
- Database stores only runtime data — outings, feedback, recommendations
- Everything outside the exe is editable — config via settings menu in UI, data via file pickers,
  images (car photo, team logo) via swapping files in config/images/

## What lives where
- exe — application logic and UI only
- config/ — JSON files for car definition, channel mappings, recommendation weights; images/ subfolder
  for the car photo and team logo used on setup sheets
- data/ — SQLite database, session history, logged data references
- modules/ — domain specific pieces like the CSV parser and recommendation engine

## Folder structure
main.py is the entry point, everything else is split by responsibility.

ui/ handles all visual components — nothing in here touches business logic.
core/ is pure Python logic — no Qt imports in core.
models/ contains the SQLAlchemy database models.
modules/ holds domain specific pieces like the CSV parser and recommendation engine.
config/ holds the default JSON config files and images that ship with the application.
data/ is git ignored — the SQLite database and session files live here at runtime.

## Data model
The database stores runtime data only — nothing that belongs in config.

Outings are the core unit. Each outing contains:
- the car setup state at the time of the outing (setup_data, JSON)
- the car setup state after the outing (setdown_data, JSON) — optional, added when the car comes back in
- a reference to the Cosworth CSV/TXT file (csv_path)
- driver feedback (feedback_data, JSON) — corner table, track map path, -5 to +5 scale per phase
- generated recommendations (not yet built)
Multiple outings per weekend are stored independently and can be compared against each other.

## Config files
config/car.json — car definition: per corner toe, camber, ride height FIA/aero, ARB, springs,
  damper bump LS/HS, blowoff, rebound LS/HS, packer, preload, total travel, free length, static droop,
  gap on GND; car-level corner weights, total weight, cross %, diff preload/position, wing position,
  splitter offset, notes
config/channels.json — maps Cosworth channel names to readable labels, units, valid ranges,
  and required flag. Parser loads only channels listed here. Math channels to be added when
  full Cosworth licence is available. Currently 16 channels configured.
config/parameters.json — recommendation engine weights and thresholds (not yet built)
config/images/car_default.jpg — top-down car photo shown in the outing form and on setup sheets
config/images/team_logo.png — team logo shown in the setup sheet PDF header
config/images/logo_left.png — Proton Competition logo shown in app topbar (left slot)
config/images/logo_right.jpg — TUM logo shown in app topbar (right slot)

Config files are editable through the settings menu in the UI (not yet built).
They can also be loaded from anywhere on the machine via file picker (not yet built).
Defaults ship with the application and are used as fallback if a file is missing or corrupt.
Images are swapped by replacing the file directly — no code changes needed to rebrand.

## Data hierarchy

Driver — defined at season level, independent of any weekend or outing.
Has a name and a driving level 1-10.
Drivers are saved permanently and selectable across all weekends and seasons.

Race Weekend — defines the track and the event (track, series, car number, year, date, type).
All outings belong to a weekend.

Outing — belongs to a weekend. Auto-numbered per weekend, optional custom name.
Contains:
- driver: selected from the driver list, nullable
- environment: air temp, track temp, track condition — carried over from previous outing
- car state: fuel level, tyre age, tyre type, tyre name — tyre type and name carry over, fuel and age do not
- setup: full car setup state before the session, carried over from previous outing
- setdown: full car setup state after the session, pre-filled from setup, independently editable and printable
- feedback: corner-by-corner driver feedback table, track map image path, -5 to +5 scale per phase
- session type, comments
- data reference: path to Cosworth TXT file, lap selection (lap list working, plot not yet built)

Setup preset — a named setup saved independently of any outing (not yet built).

Rule: driver, tyre type, tyre name, and setup carry over from the previous outing as default.
Tyre age, fuel load, and date/time are always fresh. Every field is always editable. No field is required to save.

## CSV / Data file format
Pi Toolbox ASCII format (.txt extension). Key properties:
- One channel per {ChannelBlock}, each with its own time vector
- European decimal notation (comma as separator: 314,124 = 314.124)
- Channel names include unit suffix in file (e.g. ecu_speed[kph]) — parser strips suffix before matching
- Variable sample rates per channel (1Hz to 100Hz+)
- Lap splitting via lap_number channel
- Speed channel: ecu_speed (km/h)
- Math channels pending — full list available once Cosworth licence is accessible

## Corner segmentation
Corner detection uses local speed minima on the ecu_speed channel.
Each minimum is an apex; entry/exit boundaries are the surrounding speed peaks.
Speed classification thresholds (configurable in channels.json):
- Low speed: apex < 80 km/h
- Medium speed: apex 80–150 km/h
- High speed: apex > 150 km/h
Corner data is produced per lap and will feed the recommendation engine.

## Driver Feedback
Corner-by-corner feedback table in the outing form.
Configurable corner count (1–30). Per corner: Worst Corner checkbox, five phase inputs
(Entry 1, Entry 2, Apex 3, Exit 4, Exit 5) on a -5 to +5 integer scale.
-5 = undrivable understeer, 0 = neutral, +5 = undrivable oversteer.
Track map image loadable from file, stored as path per outing.
Scale description placeholder at bottom — full text to be added per value.
Saved as feedback_data JSON on the Outing model.

## UI style
Dark theme, clean and modern. No gradients, no clutter.

Accent color: amber #C0A060 — used for active navigation, primary buttons, highlights.
Background: #141414 main, #1a1a1a sidebar and topbar, #1e1e1e hover states.
Borders: #2a2a2a primary, #222 subtle separators.
Text: #e0e0e0 primary, #888 secondary, #555 muted.

PDF setup sheets use a separate light theme — white background, muted green accent #2d6a35,
minimal ink, designed to fit one A4 page per sheet.

Layout:
- top bar with two logo slots (logo_left.png, logo_right.jpg) and centered app title
- narrow icon sidebar for navigation (Race Weekends, Drivers, Settings)
- main content area with header and list/form view
- weekends and outings views use an internal QStackedWidget for in-place navigation
  (list ↔ outings ↔ outing form), with a Back button that saves and returns

## Naming conventions
- outing not session — session is SQLAlchemy's internal word for database transactions
- SetupTool not RacingSetupTool
- table names are plural snake_case — race_weekends, outings, drivers

## Hard rules
- no PyQt6 imports in core/ or modules/
- no business logic in ui/
- all models import Base and Session from models/base.py only, never create a second one
- init_db() is called once at startup in main.py
- config files and images are never bundled into the exe — they live next to it at runtime
- HANDOVER.md and generate_handover.py are local only, never pushed to GitHub
- spin boxes use NoScrollSpinBox (QDoubleSpinBox) or NoScrollIntSpinBox (QSpinBox),
  both defined in outing_form.py, to prevent accidental value changes when scrolling the page
- parser only loads channels listed in channels.json — never loads the full file
- missing or faulty channels degrade gracefully — never crash the app

## File responsibilities
- main.py — entry point, starts app, applies stylesheet, calls init_db
- ui/style.py — global dark theme stylesheet, all colors defined here
- ui/main_window.py — window layout, topbar (with logos), sidebar, page stack navigation
- ui/views/weekends.py — race weekends & tests list, internal stack to outings view
- ui/views/weekend_dialog.py — create/edit weekend dialog
- ui/views/outings.py — outings list per weekend, internal stack to outing form
- ui/views/outing_form.py — full outing form: session, data (CSV load + lap list),
  car setup (setup + collapsible setdown), driver feedback (corner table + track map),
  comments, PDF print buttons
- ui/views/drivers.py — drivers list
- ui/views/driver_dialog.py — create/edit driver dialog
- models/base.py — engine, Session factory, Base class, init_db function
- models/driver.py — Driver model (name, driving_level 1-10)
- models/raceweekend.py — RaceWeekend model (track, series, car_number, year, date, type)
- models/outing.py — Outing model (date_time, name, number, driver, environment, car state,
  setup_data, setdown_data, feedback_data, session_type, comments, csv reference)
- modules/__init__.py — empty, makes modules/ a package
- modules/csv_parser.py — Pi Toolbox ASCII parser: metadata extraction, selective channel loading,
  European decimal handling, lap splitting, corner detection with speed classification,
  quality flagging (valid/partial/failed/missing)
- core/config_loader.py — reads config/car.json, returns setup parameter structure
- core/pdf_export.py — generates the A4 setup/setdown PDF sheet
- config/car.json — Porsche 992 GT3R setup sheet template, all parameters null by default
- config/channels.json — channel definitions: name, label, unit, range, required flag
- config/images/car_default.jpg — swappable car photo
- config/images/team_logo.png — swappable team logo (PDF header)
- config/images/logo_left.png — Proton Competition logo (topbar)
- config/images/logo_right.jpg — TUM logo (topbar)

## Current status
Phase 4 in progress — CSV parser complete, lap list working, pyqtgraph plot next.

Done:
- virtual environment, folder structure, Git/GitHub, requirements.txt
- PyQt6 dark theme, topbar with logos, icon sidebar, page stack navigation
- Race Weekends list — sortable, create/edit/delete via pre-filled dialog
- Drivers list — sortable, create/edit/delete via pre-filled dialog
- Outings view per weekend — internal stack navigation, back button refreshes, edit weekend
  button in header, sort-safe row lookup via stored outing id, global focus frame removed
- Outing form session section — date/time, name, driver, session type, tyre type/name/age,
  fuel load, air/track temp, track condition, all with scroll-safe spin boxes
- Carry-over logic for new outings, full pre-fill for editing existing outings
- Car Setup section — three column layout, per-corner damper grouping with mirror-to-opposite-side
  button (fixed: captures correct inputs dict at build time), collapsible Damper Advanced,
  swappable car image, Weights and Car parameter groups, full width Setup Notes
- Setdown section — collapsible, reuses setup UI logic, pre-fills from setup or existing setdown data
- PDF export — Print Setup and Print Setdown buttons, swappable team logo, real-aspect-ratio
  scaled images, 2x2 corner weight grid matching physical car layout, fits one A4 page.
  Zero values now display correctly. Overwrite dialog implemented (custom, suppresses Windows native).
- Rebound labelling consistent throughout UI and PDF (rebound_ls / rebound_hs keys)
- Driver Feedback section — numbered corner table (1-30 configurable), Worst Corner checkbox,
  five phase spin boxes (-5 to +5), track map image loader, scale description placeholder,
  saves/loads as feedback_data JSON
- Topbar logos — Proton Competition (left) and TUM (right), loaded from config/images/
- Data section — Load Outing button, Pi Toolbox ASCII parser (modules/csv_parser.py),
  selective channel loading from channels.json, European decimal handling, lap splitting,
  corner detection with speed classification, quality flagging, lap list table with
  fastest lap highlighted in amber

Next, in planned order:
1. Progress dialog — background QThread for file loading, busy indicator UI
2. Quick fixes — button label "Load Outing", file filter includes .txt
3. Lap selector — clicking lap row scopes all subsequent data views to that lap
4. pyqtgraph plot panel — stacked channel traces (speed, throttle, brake, RPM, steering, gear),
   shared X axis (time), lap-scoped, below lap table in Data section
5. Recommendation engine (core/) — takes outing data, setup/setdown, driver feedback,
   produces explainable setup recommendations. Corner speed classification feeds this.
6. Weekend-level report — compiled PDF summary across all outings in a weekend, likely landscape.
7. Outing comparison — side-by-side view of two or more outings.
8. Automatic tyre lap counter — increments tyre_age automatically per completed outing.
9. Settings view — load and manage custom config files via UI file picker.
10. PyInstaller packaging — build the distributable Windows executable.

## Pending / known gaps
- Math channels not yet in channels.json — awaiting full Cosworth licence access
- pyqtgraph not yet wired into UI
- Lap distance in file is in feet (lap_distance[ft]) — may need unit conversion
- Corner segmentation parameters (min_prominence=15, min_distance=20) need validation
  against real lap data once plot is working