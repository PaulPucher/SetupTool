# SetupTool Channel Requirements

Generated from the code, not written by hand. This is the checklist for
whoever configures a telemetry export for a new event: everything
SetupTool actually reads today, plus the damper and TPMS channels the
tool is built to accept the moment they're logged. An incomplete export
(e.g. the 11-channel Paul Ricard snippet) silently produces a degraded or
empty analysis with no error -- ticking every channel below prevents that.

Regenerate this file with `python diagnostics/generate_channel_requirements.py`
whenever config/channels.json or a channel-consuming module changes; do
not hand-edit the channel lists below without re-running the generator
first, or this document will drift from the code again.

## 1. REQUIRED

Every channel config/channels.json whitelists AND that real code (grepped
across modules/, core/, ui/) actually reads. Cross-checked against real
read sites, not assumed from the whitelist alone -- 1 whitelisted
channel(s) (log_gps_speed) were found to have zero consumers
anywhere in the codebase and are dropped from this list entirely (see
"Dropped as dead" at the end of this section). 4 more
(log_rideheight_fl, log_rideheight_fr, log_rideheight_rl, log_rideheight_rr) also have zero consumers but are NOT dead
config entries -- see "Ride-height sensors" under DAMPER CHANNELS below.

### 1a. Hard-required -- missing any one of these, the analysis does not
degrade, it does not run at all

`prepare_vehicle_state` (modules/stability_analysis.py) returns `None`
immediately if any of these is missing, wrong quality, or untimed.
Nothing downstream computes.

- **ecu_speed** (Speed, km/h) -- the pipeline's own time reference (t_ref); every other channel is interpolated onto this one's timeline. Corner-speed classification, moving-sample mask.
- **sclu_yaw_rate** (Yaw Rate, rpm) -- core input to sideslip estimation and Module 5 (yaw moment stability).
- **log_asteer** (Steering Angle, deg) -- front wheel angle (delta_f), corner entry/exit detection thresholds.
- **log_acc_y** (Lateral G, g) -- corner apex detection, Module 4a/4b lateral force and cornering stiffness.
- **log_acc_x** (Longitudinal G, g) -- Module 4a longitudinal force balance, Fy split.

### 1b. Lap source -- missing this, lap splitting produces ZERO laps and
nothing downstream exists (no corners, no analysis, no PDF)

- **lap_number** -- `modules/csv_parser.py`'s `_split_laps` reads
  `channels.get("lap_number")` directly (confirmed against the current
  source); if missing, it returns an empty lap list immediately. This is a
  logged channel, not derived -- there is no fallback path. **This is the
  single most critical channel in the whole list.**

### 1c. Distance source -- missing this, cross-lap corner identity and
Module 5's stability regression both go to nothing (not a crash, but a
silent, near-total loss of analysis depth)

- **lap_distance** (ft in the file, converted to metres internally) --
  read directly as a logged channel (confirmed against the current
  source, both modules/stability_analysis.py and modules/corner_analysis.py)
  -- **not derived from speed integration**. Without it: Module 5's
  s-anchored yaw-stability regression returns all-NaN for every sample
  (prepare_vehicle_state's s_m stays None); cross-lap corner clustering
  (assign_stable_corner_ids) returns immediately with no clustering at
  all, so every corner's stable_corner_id stays None.

### 1d. Required, degrades gracefully if missing (real features lost,
pipeline still runs)

- **ecu_B_speedlimit_en** -- trailing pit-fragment lap merge (csv_parser.py); has a duration-based fallback if absent, so this only sharpens in/out-lap classification.
- **ecu_aps** -- corner-phase detection (brake/throttle overlap), pipeline state.
- **ecu_gear** -- pipeline state; currently display-only downstream, no estimator consumes it yet.
- **ecu_nmot** -- UI trace display only (ui/views/outing_form.py), not read by any estimator module.
- **lap_time** -- precision lap-timing and a lap-time consistency cross-check; falls back to a computed duration if missing.
- **log_acc_z** -- Tier B (signal/data engineering exclusion mask, CLAUDE.md scientific-grounding rule) -- kerb-strike detection only (_compute_kerb_mask_from_az, modules/stability_analysis.py). **Missing this silently disables kerb exclusion -- treat as required in practice**: kerb-contaminated samples are never masked out, and nothing else flags that they weren't.
- **log_gps_lat** -- LIVE, named consumers (not the shelved GPS-heading arc -- see log_gps_course's own note): ui/views/outing_form.py:3342 `_update_corner_map_trace` (the Track Map tab -- "Load a CSV to see the track map" / "No GPS data in this file."); modules/corner_analysis.py:867 `compute_stable_corner_positions` (renders corner-map markers immediately after parsing, before Analyse has run); modules/stability_analysis.py:297 `prepare_vehicle_state` feeds summarise_corners' apex_position_x_m/y_m (computed, but not currently read back by ui/ or core/ -- a smaller, separate dead sub-path within otherwise-live code, noted not acted on). Degrades the track-map visualisation only if missing.
- **log_gps_lon** -- see log_gps_lat -- read together at every one of the same call sites, for the same track-map/geo projection.
- **log_pbrake_f** -- state channel (stability_analysis.py), front-axle input to modules/longitudinal_forces.py's Fx/slip-ratio split.
- **log_pbrake_r** -- rear-axle input to the same Fx/slip-ratio split. **Both front AND rear are needed** -- the Paul Ricard snippet included only the front; rear-axle longitudinal force/kappa silently loses accuracy without the rear channel, with no error shown.
- **log_speed_fl** -- per-wheel speed (km/h) -- modules/longitudinal_forces.py's per-wheel slip ratio (kappa), feeding the LS_ratio (longitudinal stiffness) display estimator.
- **log_speed_fr** -- see log_speed_fl.
- **log_speed_rl** -- see log_speed_fl.
- **log_speed_rr** -- see log_speed_fl.

### Excluded: consumed only by a confirmed dead/shelved code path

Not the same as "Dropped as dead" below -- this channel's name IS a
real string literal in production code (a mechanical consumer check
alone would call it "used"), but tracing that consumer shows the code
path itself is dead. Reported, not deleted.

- **log_gps_course** -- read only by `estimate_sideslip_gps` (modules/stability_analysis.py:384), a GPS-course sideslip validation function. Confirmed dead: never called from any modules/core/ui file (only caller anywhere is diagnostics/inspect_beta_gps_validation.py, a diagnostic script); that script's own header states "estimate_sideslip_gps is not called from any pipeline/UI path; production" outright; sideslip_comparison_report.md labels it "GPS-course (shelved, negative control)"; thesis_notes.md WP5b(c) records it as investigated and shelved. The code is not deleted -- reported here, not acted on -- so collecting log_gps_course costs nothing if that path is ever revisited, but it is not currently needed for anything, live or shelved-but-checked.

### Dropped as dead (whitelisted in config/channels.json, zero real
consumers found anywhere in modules/, core/, or ui/)

log_gps_speed. Not worth asking for -- and config/channels.json
itself could be trimmed of these in a future pass (out of scope here,
noted, not acted on).

## 2. DAMPER CHANNELS

Not read by any code today -- secured now because this export is the
last practical chance before the Fz-upgrade work needs them (damper-
derived Level-4 wheel loads, PLAN.md backlog item B). Nothing in the
current pipeline breaks or degrades without these; they are pure
future-proofing.

**Selection rule for the exporter: tick every channel matching
`damper*` or `susp*travel*` (case-insensitive) on your system** -- exact
names vary by logger/car configuration, so match by pattern, not by the
literal names below. **Important: on this car "damper" does not appear
in any channel name at all** -- searching only "damper" finds nothing
here; search "susp" too before concluding there is nothing to export.

In this car's actual Cosworth export (channel_list.txt, the real
2622-channel Dubai inventory), the matching channels are:

- log_susp_travel_fl[m]
- log_susp_travel_fr[m]
- log_susp_travel_rl[m]
- log_susp_travel_rr[m]
- log_susp_travel_fl_diag, log_susp_travel_fr_diag, log_susp_travel_rl_diag, log_susp_travel_rr_diag -- diagnostic companions to the four above;
  harmless to include, not analytically needed

No damper velocity or force math channel exists in that same export --
only position/travel is logged. Damper speed is derivable from position
at analysis time and does not need to be logged separately.

### Ride-height sensors

log_rideheight_fl, log_rideheight_fr, log_rideheight_rl, log_rideheight_rr -- possibly not fitted at Dubai; secured
for the Fz upgrade path. If live, they provide direct measured
aero-load evidence (dynamic vs static ride height) and bear on the
unresolved rolling-radius question. The car HAS ride-height sensors;
whether they were installed for this specific session is unknown --
the Dubai export whitelisted these channels but carried zero
consumed data for any of them, which is consistent with either
"not fitted that weekend" or "fitted, never logged". Their nature
stays unknown until a session with live data on them arrives; do not
read the current all-missing state as evidence they don't exist.

### Optional math channels (conveniences, not dependencies -- nothing
breaks without them; include if present, do not chase them if absent)

- **Any existing damper velocity or force math channel**, if the logger
  computes one on-device. SetupTool would derive velocity from position
  itself if needed; a logged version just saves that step.

- **Math_Corner Radius** -- this car's own Dubai export names it
  corner_radius and corner_radius_filtered; a different export (the Paul Ricard
  snippet) showed the same channel as "Math_Corner Radius". **NAME MAPPING
  REQUIRED**: the tool matches by exact name only, so ticking just one
  spelling risks silently missing it on a system using the other --
  map the two names together, or include both spellings in the export.
  A convenience cross-check channel (thesis_notes.md), not consumed by
  any estimator, marked optional as decided.

No other math channels are wanted: the pipeline computes its own
derived quantities from raw sensors by design (see CLAUDE.md's
scientific-grounding rule) -- pre-computed math channels beyond the two
above are neither required nor useful here.

## 3. TPMS CHANNELS

Also not read by any code today, also secured now as future-proofing --
tyre pressure and temperature per corner, for future tyre-state work.
Nothing in the current pipeline breaks or degrades without these.

**Selection: exactly these 8 channels -- no wildcard needed.** Confirmed
against this car's actual Cosworth export (channel_list.txt):

- tpms_press_fl / tpms_press_fr / tpms_press_rl / tpms_press_rr [bar]
  -- tyre pressure per corner
- tpms_temp_fl / tpms_temp_fr / tpms_temp_rl / tpms_temp_rr [°C] --
  tyre temperature per corner

Note, not an instruction: this car's TPMS system exposes many more
internal/diagnostic channels under the same `tpms` prefix -- per-corner
IR zone temperatures, pressure/temperature error and warning flags,
internal ECU diagnostics -- **371 channels total match `tpms*`** (counted
directly, channel_list.txt). None of the other 363 are needed; the 8 above are the full
selection, not a starting point.

## Export conditions

- **Full session, not excerpts.** Every lap, outlap through inlap --
  the pipeline's own lap/corner logic depends on seeing the complete
  session, not a trimmed window.
- **Native sample rate, no resampling on export.** This car's Cosworth
  log runs at 50 Hz; SetupTool's rate-dependent estimators (LS_ratio's
  window sizing, in particular) are derived from the actual logged
  rate at runtime, not hardcoded -- resampling on export would silently
  change what those estimators compute.
- **File format:** the parser (`modules/csv_parser.py`) reads exactly
  the `PiToolboxVersionedASCIIDataSet` tab-delimited export (confirmed
  against the Dubai sample file's own header) -- `{OutingInformation}`
  and `{ChannelBlock}` sections, European decimal notation (comma as
  decimal separator). This is the only format the parser accepts.
- **Both brake pressure channels.** log_pbrake_f AND log_pbrake_r --
  the Paul Ricard snippet contained only the front. Verify both are
  ticked before sending.
- **Channel names must match the Dubai export exactly.** The tool
  matches by name, not by physical meaning -- a renamed channel is
  indistinguishable from a missing one, and will silently degrade
  analysis with no error. If any channel name differs on your logger
  configuration, report the name mapping before the event so
  config/channels.json can be updated to match.
- **Save the selection as a named preset** on your export tool once
  configured, so future exports reuse it instead of re-ticking by
  hand -- the whole point of this document is to make an incomplete
  export impossible to send by accident, and a saved preset is the
  actual mechanism that achieves that, not just the checklist.

When in doubt, include it: a ticked channel costs kilobytes in the
export file; a missing one costs the session.
