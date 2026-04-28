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
- PyInstaller — packaging to Windows executable
- Git + GitHub — version control

## Architecture principles
- UI and core logic are strictly separated — no business logic in UI files, no PyQt6 imports in core/
- Config files are the source for car definition and parameters
- Database stores only runtime data — sessions, feedback, recommendations
- Everything outside the exe is editable — config via settings menu in UI, data via file pickers

## What lives where
- exe — application logic and UI only
- config/ — JSON files for car definition, channel mappings, recommendation weights
- data/ — SQLite database, session history, logged data references

## Folder structure
main.py is the entry point, everything else is split by responsibility.

ui/ handles all visual components — nothing in here touches business logic.
core/ is pure Python logic — no Qt imports in core.
models/ contains the SQLAlchemy database models.
modules/ holds domain specific pieces like the CSV parser and recommendation engine.
config/ holds the default JSON config files that ship with the application.
data/ is git ignored — the SQLite database and session files live here at runtime.

## Data model
The database stores runtime data only — nothing that belongs in config.

Sessions are the core unit. Each session contains:
- the car setup state at the time of the session
- a reference to the Cosworth CSV file
- driver feedback
- generated recommendations
Multiple sessions per weekend are stored independently and can be compared against each other.

## Config files
config/car.json — car definition, suspension geometry, physical parameter limits
config/channels.json — maps Cosworth channel names to physical measurements
config/parameters.json — recommendation engine weights and thresholds

Config files are editable through the settings menu in the UI.
They can also be loaded from anywhere on the machine via file picker.
Defaults ship with the application and are used as fallback if a file is missing or corrupt.


## Data hierarchy

Driver — defined at season level, independent of any weekend or outing.
Has a name and a driving level/style profile.
Drivers are saved permanently and selectable across all weekends and seasons.

Race Weekend — defines the track and the event (year, round or name).
All outings belong to a weekend. The weekend has no date itself.

Outing — belongs to a weekend, each outing stores its own date and time independently.
Contains:
- driver: selected from the driver list
- environment: air temp, track temp, track condition — carried over from previous outing, always editable
- car state: fuel level, tyre age, tyre compound (dry or wet) — carried over from previous outing, always editable
- setup: full car setup state — carried over from previous outing, always editable
- data reference: path to Cosworth CSV file, lap selection (single lap or all laps)

Setup preset — a named setup saved independently of any outing.
Can be loaded as a starting point for any outing setup.

Rule: everything carries over from the previous outing as default, every field is always editable.

## Current status
Phase 1 complete.

Done:
- virtual environment set up
- folder structure created
- PyQt6 installed, first window running
- SQLAlchemy installed
- Git initialised, connected to GitHub
- PROJECT.md started
- models created: Driver, RaceWeekend, Outing
- database initialises correctly, tables created in data/setuptool.db

Next:
- create default config JSON files in config/
- build main window structure with basic navigation
- build first UI forms for creating a driver and a race weekend