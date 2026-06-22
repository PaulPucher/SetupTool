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
- pyqtgraph — data plotting
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

## Folder structure
main.py is the entry point, everything else is split by responsibility.

ui/ handles all visual components — nothing in here touches business logic.
core/ is pure Python logic — no Qt imports in core.
models/ contains the SQLAlchemy database models.
modules/ holds domain specific pieces like the CSV parser and recommendation engine (not yet built).
config/ holds the default JSON config files and images that ship with the application.
data/ is git ignored — the SQLite database and session files live here at runtime.

## Data model
The database stores runtime data only — nothing that belongs in config.

Outings are the core unit. Each outing contains:
- the car setup state at the time of the outing (setup_data, JSON)
- the car setup state after the outing (setdown_data, JSON) — optional, added when the car comes back in
- a reference to the Cosworth CSV file (not yet wired up)
- driver feedback (not yet built)
- generated recommendations (not yet built)
Multiple outings per weekend are stored independently and can be compared against each other.

## Config files
config/car.json — car definition: per corner toe, camber, ride height FIA/aero, ARB, springs,
  damper bump LS/HS, blowoff, rebound LS/HS, packer, preload, total travel, free length, static droop,
  gap on GND; car-level corner weights, total weight, cross %, diff preload/position, wing position,
  splitter offset, notes
config/channels.json — maps Cosworth channel names to physical measurements (not yet built)
config/parameters.json — recommendation engine weights and thresholds (not yet built)
config/images/car_default.jpg — top-down car photo shown in the outing form and on setup sheets
config/images/team_logo.png — team logo shown in the setup sheet PDF header

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
- session type, comments
- data reference: path to Cosworth CSV file, lap selection (not yet wired up)

Setup preset — a named setup saved independently of any outing (not yet built).

Rule: driver, tyre type, tyre name, and setup carry over from the previous outing as default.
Tyre age, fuel load, and date/time are always fresh. Every field is always editable. No field is required to save.

## UI style
Dark theme, clean and modern. No gradients, no clutter.

Accent color: amber #C0A060 — used for active navigation, primary buttons, highlights.
Background: #141414 main, #1a1a1a sidebar and topbar, #1e1e1e hover states.
Borders: #2a2a2a primary, #222 subtle separators.
Text: #e0e0e0 primary, #888 secondary, #555 muted.

PDF setup sheets use a separate light theme — white background, muted green accent #2d6a35,
minimal ink, designed to fit one A4 page per sheet.

Layout:
- top bar with two logo slots and centered app title
- narrow icon sidebar for navigation (Race Weekends, Drivers, Settings)
- main content area with header and list/form view
- weekends and outings views use an internal QStackedWidget for in-place navigation
  (list ↔ outings ↔ outing form), with a Back button that saves and returns

## Naming conventions
- outing not session — session is SQLAlchemy's internal word for database transactions
- SetupTool not RacingSetupTool
- table names are plural snake_case — race_weekends, outings, drivers

## Hard rules
- no PyQt6 imports in core/
- no business logic in ui/
- all models import Base and Session from models/base.py only, never create a second one
- init_db() is called once at startup in main.py
- config files and images are never bundled into the exe — they live next to it at runtime
- HANDOVER.md and generate_handover.py are local only, never pushed to GitHub
- spin boxes use NoScrollSpinBox (defined in outing_form.py) to prevent accidental value changes
  when scrolling the page

## File responsibilities
- main.py — entry point, starts app, applies stylesheet, calls init_db
- ui/style.py — global dark theme stylesheet, all colors defined here
- ui/main_window.py — window layout, topbar, sidebar, page stack navigation
- ui/views/weekends.py — race weekends & tests list, internal stack to outings view
- ui/views/weekend_dialog.py — create/edit weekend dialog
- ui/views/outings.py — outings list per weekend, internal stack to outing form
- ui/views/outing_form.py — full outing form: session, car setup (setup + collapsible setdown),
  driver feedback placeholder, comments, PDF print buttons
- ui/views/drivers.py — drivers list
- ui/views/driver_dialog.py — create/edit driver dialog
- models/base.py — engine, Session factory, Base class, init_db function
- models/driver.py — Driver model (name, driving_level 1-10)
- models/raceweekend.py — RaceWeekend model (track, series, car_number, year, date, type)
- models/outing.py — Outing model (date_time, name, number, driver, environment, car state,
  setup_data, setdown_data, session_type, comments, csv reference)
- core/config_loader.py — reads config/car.json, returns setup parameter structure
- core/pdf_export.py — generates the A4 setup/setdown PDF sheet
- config/car.json — Porsche 992 GT3R setup sheet template, all parameters null by default
- config/images/car_default.jpg — swappable car photo
- config/images/team_logo.png — swappable team logo

## Current status
Phase 3 in progress — outing form core is complete, moving into driver feedback next.

Done:
- virtual environment, folder structure, Git/GitHub, requirements.txt
- PyQt6 dark theme, topbar, icon sidebar, page stack navigation
- Race Weekends list — sortable, create/edit/delete via pre-filled dialog
- Drivers list — sortable, create/edit/delete via pre-filled dialog
- Outings view per weekend — internal stack navigation, back button refreshes, edit weekend
  button in header, sort-safe row lookup via stored outing id, global focus frame removed
- Outing form session section — date/time, name, driver, session type, tyre type/name/age,
  fuel load, air/track temp, track condition, all with scroll-safe spin boxes
- Carry-over logic for new outings, full pre-fill for editing existing outings
- Car Setup section — three column layout, per-corner damper grouping with mirror-to-opposite-side
  button, collapsible Damper Advanced, swappable car image, Weights and Car parameter groups,
  full width Setup Notes
- Setdown section — collapsible, reuses setup UI logic, pre-fills from setup or existing setdown data
- PDF export — Print Setup and Print Setdown buttons, swappable team logo, real-aspect-ratio
  scaled images, 2x2 corner weight grid matching physical car layout, fits one A4 page
- Bug fixes: non-numeric car number no longer crashes the app, outings can be saved without
  a driver selected, scroll wheel no longer accidentally changes spin box values

Next, in planned order:
1. Driver Feedback section — numbered track map image with a corner-by-corner table,
   four dropdowns per corner (Braking, Entry, Apex, Exit). Still to decide: how track images
   are sourced and numbered per circuit, and the dropdown option values.
2. Data handling — load Cosworth CSV with pandas, display lap list with fastest lap highlighted,
   select single lap or full outing, channel preview/plotting with pyqtgraph.
3. Recommendation engine (core/) — the central thesis deliverable. Takes outing data, setup/setdown
   state, and driver feedback, and produces explainable setup recommendations. To be designed
   around supervisor input on deriving stability insights from setup data.
4. Weekend-level report — compiled PDF summary across all outings in a weekend, likely landscape.
5. Outing comparison — side-by-side view of two or more outings.
6. Automatic tyre lap counter — increments tyre_age automatically per completed outing on that
   tyre set, replacing manual entry.
7. Settings view — load and manage custom config files via UI file picker.
8. PyInstaller packaging — build the distributable Windows executable.