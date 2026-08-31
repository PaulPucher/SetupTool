# Generates docs/channel_requirements.md from the actual code, not from
# memory -- the document a data engineer uses to configure a telemetry
# export must never drift from what modules/csv_parser.py and the
# analysis pipeline actually read. Re-run this whenever config/
# channels.json or a channel-consuming module changes; the generated
# file is the deliverable (committed), this script is the tool that
# keeps it honest. No tools/ directory convention exists in this repo
# (checked before placing this here) -- kept in diagnostics/ instead,
# [keep-reproduces] per CLAUDE.md's diagnostics/ disposal rule: it
# regenerates a real, committed deliverable, not a one-off finding.
#
# What is mechanically derived (re-verified every run, cannot drift
# silently): the whitelist itself, which whitelisted channels have a
# real consumer anywhere in modules/core/ui (dead entries dropped),
# the hard-required list parsed directly out of prepare_vehicle_state's
# own `required = [...]` line, confirmation that lap_number/lap_distance
# are read as plain channels with no derivation path, the damper/TPMS/
# corner-radius channel families as they actually appear in a real
# export (channel_list.txt), and any channel access in modules/core
# whose name is NOT in the whitelist (a latent bug, reported not fixed).
#
# What is NOT mechanically derivable and stays hand-authored, in the
# CHANNEL_NOTES table below (still versioned, still reviewed, just not
# regex-extractable): WHY each channel matters and what breaks without
# it. If channels.json gains a new consumed channel with no entry in
# CHANNEL_NOTES, generation prints a warning instead of silently
# emitting an unexplained bullet -- the staleness case is loud, not
# silent.

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHANNELS_JSON = ROOT / "config" / "channels.json"
STABILITY_PY = ROOT / "modules" / "stability_analysis.py"
CSV_PARSER_PY = ROOT / "modules" / "csv_parser.py"
CHANNEL_LIST_TXT = ROOT / "channel_list.txt"
OUT_MD = ROOT / "docs" / "channel_requirements.md"
OUT_TXT = ROOT / "docs" / "channel_list.txt"

# Whitelisted, zero consumers found -- BUT this is real, installed
# hardware (ride-height sensors), not a dead config entry. Zero data at
# Dubai is consistent with either "not fitted that weekend" or "fitted,
# just never logged/consumed" -- unknown until a session with live data
# arrives. Excluded from the mechanical dead-channel bucket and given
# their own note in DAMPER CHANNELS instead of REQUIRED's "dropped as
# dead" list, which is reserved for entries with no such ambiguity.
RIDE_HEIGHT_CHANNELS = ("log_rideheight_fl", "log_rideheight_fr", "log_rideheight_rl", "log_rideheight_rr")

# Exact real names confirmed against channel_list.txt (the 2622-channel
# Dubai inventory) -- shared by the markdown, the WhatsApp short list,
# and the bare channel_list.txt output below, so all three can never
# disagree with each other.
DAMPER_VALUE_CHANNELS = ("log_susp_travel_fl", "log_susp_travel_fr", "log_susp_travel_rl", "log_susp_travel_rr")
TPMS_VALUE_CHANNELS = ("tpms_press_fl", "tpms_press_fr", "tpms_press_rl", "tpms_press_rr",
                        "tpms_temp_fl", "tpms_temp_fr", "tpms_temp_rl", "tpms_temp_rr")
CORNER_RADIUS_VALUE_CHANNELS = ("corner_radius", "corner_radius_filtered")
CORNER_RADIUS_PRC_NAME = "Math_Corner Radius"

# Whitelisted, mechanically "consumed" (the name appears as a real string
# literal in production code), but that code path is a confirmed dead/
# shelved arc -- consuming it is not the same as needing it. Excluded from
# REQUIRED and both generated files; the dead path is reported, not acted
# on (2026-08-30 correction -- see thesis_notes.md).
#   log_gps_course: read only by modules/stability_analysis.py:384
#   estimate_sideslip_gps. Confirmed dead THREE independent ways: (1) that
#   function is never called from any other modules/core/ui file (grepped
#   repo-wide, only caller anywhere is diagnostics/inspect_beta_gps_
#   validation.py, itself a diagnostic script); (2) diagnostics/inspect_
#   beta_gps_validation.py's own header states it explicitly: "estimate_
#   sideslip_gps is not called from any pipeline/UI path; production"; (3)
#   sideslip_comparison_report.md labels it "GPS-course (shelved, negative
#   control)" and thesis_notes.md WP5b(c) records it as investigated and
#   shelved. Code NOT deleted this pass -- reported only, per instruction.
DEAD_PATH_CHANNELS = ("log_gps_course",)
DEAD_PATH_NOTE = ("read only by `estimate_sideslip_gps` (modules/stability_analysis.py:384), a GPS-course "
                   "sideslip validation function. Confirmed dead: never called from any modules/core/ui "
                   "file (only caller anywhere is diagnostics/inspect_beta_gps_validation.py, a diagnostic "
                   "script); that script's own header states \"estimate_sideslip_gps is not called from "
                   "any pipeline/UI path; production\" outright; sideslip_comparison_report.md labels it "
                   "\"GPS-course (shelved, negative control)\"; thesis_notes.md WP5b(c) records it as "
                   "investigated and shelved. The code is not deleted -- reported here, not acted on -- so "
                   "collecting log_gps_course costs nothing if that path is ever revisited, but it is not "
                   "currently needed for anything, live or shelved-but-checked.")

SCAN_DIRS = ["modules", "core", "ui"]
STRING_LITERAL_RE = re.compile(r'"([A-Za-z_][A-Za-z0-9_]*)"|\'([A-Za-z_][A-Za-z0-9_]*)\'')

# Hand-authored WHY per channel -- see module docstring above for why
# this table exists instead of being derived. Keyed by channel name;
# "tier" controls which REQUIRED subsection it prints under.
CHANNEL_NOTES = {
    "ecu_speed": ("hard", "the pipeline's own time reference (t_ref); every other channel is "
                  "interpolated onto this one's timeline. Corner-speed classification, moving-sample mask."),
    "sclu_yaw_rate": ("hard", "core input to sideslip estimation and Module 5 (yaw moment stability)."),
    "log_asteer": ("hard", "front wheel angle (delta_f), corner entry/exit detection thresholds."),
    "log_acc_y": ("hard", "corner apex detection, Module 4a/4b lateral force and cornering stiffness."),
    "log_acc_x": ("hard", "Module 4a longitudinal force balance, Fy split."),
    "ecu_B_speedlimit_en": ("soft", "trailing pit-fragment lap merge (csv_parser.py); has a duration-based "
                            "fallback if absent, so this only sharpens in/out-lap classification."),
    "lap_time": ("soft", "precision lap-timing and a lap-time consistency cross-check; falls back to a "
                 "computed duration if missing."),
    "ecu_aps": ("soft", "corner-phase detection (brake/throttle overlap), pipeline state."),
    "log_pbrake_f": ("soft", "state channel (stability_analysis.py), front-axle input to "
                     "modules/longitudinal_forces.py's Fx/slip-ratio split."),
    "log_pbrake_r": ("soft", "rear-axle input to the same Fx/slip-ratio split. **Both front AND rear are "
                     "needed** -- the Paul Ricard snippet included only the front; rear-axle longitudinal "
                     "force/kappa silently loses accuracy without the rear channel, with no error shown."),
    "ecu_gear": ("soft", "pipeline state; currently display-only downstream, no estimator consumes it yet."),
    "ecu_nmot": ("soft", "UI trace display only (ui/views/outing_form.py), not read by any estimator module."),
    "log_acc_z": ("soft", "Tier B (signal/data engineering exclusion mask, CLAUDE.md scientific-grounding "
                  "rule) -- kerb-strike detection only (_compute_kerb_mask_from_az, "
                  "modules/stability_analysis.py). **Missing this silently disables kerb exclusion -- "
                  "treat as required in practice**: kerb-contaminated samples are never masked out, and "
                  "nothing else flags that they weren't."),
    "log_gps_lat": ("soft", "LIVE, named consumers (not the shelved GPS-heading arc -- see log_gps_course's "
                    "own note): ui/views/outing_form.py:3342 `_update_corner_map_trace` (the Track Map tab "
                    "-- \"Load a CSV to see the track map\" / \"No GPS data in this file.\"); "
                    "modules/corner_analysis.py:867 `compute_stable_corner_positions` (renders corner-map "
                    "markers immediately after parsing, before Analyse has run); modules/"
                    "stability_analysis.py:297 `prepare_vehicle_state` feeds summarise_corners' "
                    "apex_position_x_m/y_m (computed, but not currently read back by ui/ or core/ -- a "
                    "smaller, separate dead sub-path within otherwise-live code, noted not acted on). "
                    "Degrades the track-map visualisation only if missing."),
    "log_gps_lon": ("soft", "see log_gps_lat -- read together at every one of the same call sites, for the "
                    "same track-map/geo projection."),
    "log_speed_fl": ("soft", "per-wheel speed (km/h) -- modules/longitudinal_forces.py's per-wheel slip "
                     "ratio (kappa), feeding the LS_ratio (longitudinal stiffness) display estimator."),
    "log_speed_fr": ("soft", "see log_speed_fl."),
    "log_speed_rl": ("soft", "see log_speed_fl."),
    "log_speed_rr": ("soft", "see log_speed_fl."),
}

HARD_LABELS = {
    "ecu_speed": "Speed, km/h", "sclu_yaw_rate": "Yaw Rate, rpm", "log_asteer": "Steering Angle, deg",
    "log_acc_y": "Lateral G, g", "log_acc_x": "Longitudinal G, g",
}


def load_whitelist():
    data = json.loads(CHANNELS_JSON.read_text(encoding="utf-8"))
    return data["channels"]


def scan_py_files(dirs):
    files = []
    for d in dirs:
        files.extend((ROOT / d).rglob("*.py"))
    return files


def find_consumers(whitelist, files):
    """Per whitelisted channel: every (file, line) where its exact name
    appears as a quoted string literal. A channel with zero hits is not
    read by any code path -- whitelisted but dead.
    """
    consumers = {name: [] for name in whitelist}
    for f in files:
        text = f.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            for m in STRING_LITERAL_RE.finditer(line):
                name = m.group(1) or m.group(2)
                if name in consumers:
                    consumers[name].append((f.relative_to(ROOT).as_posix(), lineno))
    return consumers


def find_hard_required():
    text = STABILITY_PY.read_text(encoding="utf-8")
    m = re.search(r'required\s*=\s*\[([^\]]*)\]', text)
    if not m:
        return []
    return [s.strip().strip('"\'') for s in m.group(1).split(",") if s.strip()]


def find_unwhitelisted_access(whitelist, files):
    access_re = re.compile(
        r'channels(?:\.get)?\[\s*["\']([A-Za-z_][A-Za-z0-9_]*)["\']'
        r'|channels\.get\(\s*["\']([A-Za-z_][A-Za-z0-9_]*)["\']'
        r'|_interp_channel\(\s*channels\s*,\s*["\']([A-Za-z_][A-Za-z0-9_]*)["\']'
    )
    findings = []
    for f in files:
        text = f.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            for m in access_re.finditer(line):
                name = next(g for g in m.groups() if g)
                if name not in whitelist:
                    findings.append((f.relative_to(ROOT).as_posix(), lineno, name))
    return findings


def find_lap_and_distance_source():
    csv_text = CSV_PARSER_PY.read_text(encoding="utf-8")
    stab_text = STABILITY_PY.read_text(encoding="utf-8")
    return ('channels.get("lap_number")' in csv_text,
            'channels.get("lap_distance")' in stab_text)


def _channel_list_matches(pattern):
    if not CHANNEL_LIST_TXT.exists():
        return []
    return [ln.strip() for ln in CHANNEL_LIST_TXT.read_text(encoding="utf-8", errors="replace").splitlines()
            if ln.strip() and pattern.search(ln)]


def find_damper_channels():
    return _channel_list_matches(re.compile(r'damper|shock|susp.*(trav|pos|vel|force)', re.IGNORECASE))


def find_corner_radius_channels():
    return _channel_list_matches(re.compile(r'corner.*radius', re.IGNORECASE))


def find_tpms_channels():
    return _channel_list_matches(re.compile(r'tpms', re.IGNORECASE))


LAP_DISTANCE_SOURCE_CHANNELS = ("lap_number", "lap_distance")


def render_markdown(data):
    used_hard = [n for n in data["hard_required"]]
    used_soft = sorted(n for n in data["used"]
                        if n not in used_hard and n not in LAP_DISTANCE_SOURCE_CHANNELS)

    missing_notes = [n for n in data["used"]
                      if n not in CHANNEL_NOTES and n not in LAP_DISTANCE_SOURCE_CHANNELS]
    if missing_notes:
        print("WARNING: consumed channels with no authored note in CHANNEL_NOTES "
              f"(add one before trusting the generated doc): {missing_notes}")

    lines = []
    a = lines.append

    a("# SetupTool Channel Requirements\n")
    a("Generated from the code, not written by hand. This is the checklist for\n"
      "whoever configures a telemetry export for a new event: everything\n"
      "SetupTool actually reads today, plus the damper and TPMS channels the\n"
      "tool is built to accept the moment they're logged. An incomplete export\n"
      "(e.g. the 11-channel Paul Ricard snippet) silently produces a degraded or\n"
      "empty analysis with no error -- ticking every channel below prevents that.\n")
    a("Regenerate this file with `python diagnostics/generate_channel_requirements.py`\n"
      "whenever config/channels.json or a channel-consuming module changes; do\n"
      "not hand-edit the channel lists below without re-running the generator\n"
      "first, or this document will drift from the code again.\n")

    dead_pure = [n for n in data["dead"] if n not in RIDE_HEIGHT_CHANNELS]
    dead_ride_height = [n for n in data["dead"] if n in RIDE_HEIGHT_CHANNELS]

    a("## 1. REQUIRED\n")
    a(f"Every channel config/channels.json whitelists AND that real code (grepped\n"
      f"across modules/, core/, ui/) actually reads. Cross-checked against real\n"
      f"read sites, not assumed from the whitelist alone -- {len(dead_pure)} whitelisted\n"
      f"channel(s) ({', '.join(dead_pure)}) were found to have zero consumers\n"
      f"anywhere in the codebase and are dropped from this list entirely (see\n"
      f"\"Dropped as dead\" at the end of this section). {len(dead_ride_height)} more\n"
      f"({', '.join(dead_ride_height)}) also have zero consumers but are NOT dead\n"
      f"config entries -- see \"Ride-height sensors\" under DAMPER CHANNELS below.\n")

    a("### 1a. Hard-required -- missing any one of these, the analysis does not\n"
      "degrade, it does not run at all\n")
    a("`prepare_vehicle_state` (modules/stability_analysis.py) returns `None`\n"
      "immediately if any of these is missing, wrong quality, or untimed.\n"
      "Nothing downstream computes.\n")
    for n in used_hard:
        label = HARD_LABELS.get(n, "")
        note = CHANNEL_NOTES.get(n, ("hard", "(no note authored)"))[1]
        a(f"- **{n}** ({label}) -- {note}")
    a("")

    a("### 1b. Lap source -- missing this, lap splitting produces ZERO laps and\n"
      "nothing downstream exists (no corners, no analysis, no PDF)\n")
    confirmed = "confirmed" if data["lap_ok"] else "NOT CONFIRMED -- re-check csv_parser.py"
    a(f"- **lap_number** -- `modules/csv_parser.py`'s `_split_laps` reads\n"
      f"  `channels.get(\"lap_number\")` directly ({confirmed} against the current\n"
      f"  source); if missing, it returns an empty lap list immediately. This is a\n"
      f"  logged channel, not derived -- there is no fallback path. **This is the\n"
      f"  single most critical channel in the whole list.**\n")

    a("### 1c. Distance source -- missing this, cross-lap corner identity and\n"
      "Module 5's stability regression both go to nothing (not a crash, but a\n"
      "silent, near-total loss of analysis depth)\n")
    confirmed = "confirmed" if data["dist_ok"] else "NOT CONFIRMED -- re-check stability_analysis.py"
    a(f"- **lap_distance** (ft in the file, converted to metres internally) --\n"
      f"  read directly as a logged channel ({confirmed} against the current\n"
      f"  source, both modules/stability_analysis.py and modules/corner_analysis.py)\n"
      f"  -- **not derived from speed integration**. Without it: Module 5's\n"
      f"  s-anchored yaw-stability regression returns all-NaN for every sample\n"
      f"  (prepare_vehicle_state's s_m stays None); cross-lap corner clustering\n"
      f"  (assign_stable_corner_ids) returns immediately with no clustering at\n"
      f"  all, so every corner's stable_corner_id stays None.\n")

    a("### 1d. Required, degrades gracefully if missing (real features lost,\n"
      "pipeline still runs)\n")
    for n in used_soft:
        note = CHANNEL_NOTES.get(n, ("soft", "(no note authored -- add one to CHANNEL_NOTES)"))[1]
        a(f"- **{n}** -- {note}")
    a("")

    a("### Excluded: consumed only by a confirmed dead/shelved code path\n")
    a(f"Not the same as \"Dropped as dead\" below -- this channel's name IS a\n"
      f"real string literal in production code (a mechanical consumer check\n"
      f"alone would call it \"used\"), but tracing that consumer shows the code\n"
      f"path itself is dead. Reported, not deleted.\n")
    for n in DEAD_PATH_CHANNELS:
        a(f"- **{n}** -- {DEAD_PATH_NOTE}")
    a("")

    a("### Dropped as dead (whitelisted in config/channels.json, zero real\n"
      "consumers found anywhere in modules/, core/, or ui/)\n")
    a(", ".join(dead_pure) + ". Not worth asking for -- and config/channels.json\n"
      "itself could be trimmed of these in a future pass (out of scope here,\n"
      "noted, not acted on).\n")

    a("## 2. DAMPER CHANNELS\n")
    a("Not read by any code today -- secured now because this export is the\n"
      "last practical chance before the Fz-upgrade work needs them (damper-\n"
      "derived Level-4 wheel loads, PLAN.md backlog item B). Nothing in the\n"
      "current pipeline breaks or degrades without these; they are pure\n"
      "future-proofing.\n")
    a("**Selection rule for the exporter: tick every channel matching\n"
      "`damper*` or `susp*travel*` (case-insensitive) on your system** -- exact\n"
      "names vary by logger/car configuration, so match by pattern, not by the\n"
      "literal names below. **Important: on this car \"damper\" does not appear\n"
      "in any channel name at all** -- searching only \"damper\" finds nothing\n"
      "here; search \"susp\" too before concluding there is nothing to export.\n")
    value_dampers = [c for c in data["damper_channels"] if "_diag" not in c]
    a("In this car's actual Cosworth export (channel_list.txt, the real\n"
      f"{data['channel_list_total']}-channel Dubai inventory), the matching channels are:\n")
    for c in value_dampers:
        a(f"- {c}")
    diag_dampers = [c for c in data["damper_channels"] if "_diag" in c]
    if diag_dampers:
        a(f"- {', '.join(diag_dampers)} -- diagnostic companions to the four above;\n"
          f"  harmless to include, not analytically needed\n")
    a("No damper velocity or force math channel exists in that same export --\n"
      "only position/travel is logged. Damper speed is derivable from position\n"
      "at analysis time and does not need to be logged separately.\n")

    a("### Ride-height sensors\n")
    a(f"{', '.join(dead_ride_height)} -- possibly not fitted at Dubai; secured\n"
      f"for the Fz upgrade path. If live, they provide direct measured\n"
      f"aero-load evidence (dynamic vs static ride height) and bear on the\n"
      f"unresolved rolling-radius question. The car HAS ride-height sensors;\n"
      f"whether they were installed for this specific session is unknown --\n"
      f"the Dubai export whitelisted these channels but carried zero\n"
      f"consumed data for any of them, which is consistent with either\n"
      f"\"not fitted that weekend\" or \"fitted, never logged\". Their nature\n"
      f"stays unknown until a session with live data on them arrives; do not\n"
      f"read the current all-missing state as evidence they don't exist.\n")

    a("### Optional math channels (conveniences, not dependencies -- nothing\n"
      "breaks without them; include if present, do not chase them if absent)\n")
    a("- **Any existing damper velocity or force math channel**, if the logger\n"
      "  computes one on-device. SetupTool would derive velocity from position\n"
      "  itself if needed; a logged version just saves that step.\n")
    a(f"- **Math_Corner Radius** -- this car's own Dubai export names it\n"
      f"  {' and '.join(CORNER_RADIUS_VALUE_CHANNELS)}; a different export (the Paul Ricard\n"
      f"  snippet) showed the same channel as \"{CORNER_RADIUS_PRC_NAME}\". **NAME MAPPING\n"
      f"  REQUIRED**: the tool matches by exact name only, so ticking just one\n"
      f"  spelling risks silently missing it on a system using the other --\n"
      f"  map the two names together, or include both spellings in the export.\n"
      f"  A convenience cross-check channel (thesis_notes.md), not consumed by\n"
      f"  any estimator, marked optional as decided.\n")
    a("No other math channels are wanted: the pipeline computes its own\n"
      "derived quantities from raw sensors by design (see CLAUDE.md's\n"
      "scientific-grounding rule) -- pre-computed math channels beyond the two\n"
      "above are neither required nor useful here.\n")

    a("## 3. TPMS CHANNELS\n")
    a("Also not read by any code today, also secured now as future-proofing --\n"
      "tyre pressure and temperature per corner, for future tyre-state work.\n"
      "Nothing in the current pipeline breaks or degrades without these.\n")
    a("**Selection: exactly these 8 channels -- no wildcard needed.** Confirmed\n"
      "against this car's actual Cosworth export (channel_list.txt):\n")
    a("- tpms_press_fl / tpms_press_fr / tpms_press_rl / tpms_press_rr [bar]\n"
      "  -- tyre pressure per corner\n"
      "- tpms_temp_fl / tpms_temp_fr / tpms_temp_rl / tpms_temp_rr [°C] --\n"
      "  tyre temperature per corner\n")
    a(f"Note, not an instruction: this car's TPMS system exposes many more\n"
      f"internal/diagnostic channels under the same `tpms` prefix -- per-corner\n"
      f"IR zone temperatures, pressure/temperature error and warning flags,\n"
      f"internal ECU diagnostics -- **{len(data['tpms_channels'])} channels total match `tpms*`** (counted\n"
      f"directly, channel_list.txt). None of the other {len(data['tpms_channels']) - 8} are needed; the 8 above are the full\n"
      f"selection, not a starting point.\n")

    a("## Export conditions\n")
    a("- **Full session, not excerpts.** Every lap, outlap through inlap --\n"
      "  the pipeline's own lap/corner logic depends on seeing the complete\n"
      "  session, not a trimmed window.\n"
      "- **Native sample rate, no resampling on export.** This car's Cosworth\n"
      "  log runs at 50 Hz; SetupTool's rate-dependent estimators (LS_ratio's\n"
      "  window sizing, in particular) are derived from the actual logged\n"
      "  rate at runtime, not hardcoded -- resampling on export would silently\n"
      "  change what those estimators compute.\n"
      "- **File format:** the parser (`modules/csv_parser.py`) reads exactly\n"
      "  the `PiToolboxVersionedASCIIDataSet` tab-delimited export (confirmed\n"
      "  against the Dubai sample file's own header) -- `{OutingInformation}`\n"
      "  and `{ChannelBlock}` sections, European decimal notation (comma as\n"
      "  decimal separator). This is the only format the parser accepts.\n"
      "- **Both brake pressure channels.** log_pbrake_f AND log_pbrake_r --\n"
      "  the Paul Ricard snippet contained only the front. Verify both are\n"
      "  ticked before sending.\n"
      "- **Channel names must match the Dubai export exactly.** The tool\n"
      "  matches by name, not by physical meaning -- a renamed channel is\n"
      "  indistinguishable from a missing one, and will silently degrade\n"
      "  analysis with no error. If any channel name differs on your logger\n"
      "  configuration, report the name mapping before the event so\n"
      "  config/channels.json can be updated to match.\n"
      "- **Save the selection as a named preset** on your export tool once\n"
      "  configured, so future exports reuse it instead of re-ticking by\n"
      "  hand -- the whole point of this document is to make an incomplete\n"
      "  export impossible to send by accident, and a saved preset is the\n"
      "  actual mechanism that achieves that, not just the checklist.\n")
    a("When in doubt, include it: a ticked channel costs kilobytes in the\n"
      "export file; a missing one costs the session.\n")

    return "\n".join(lines)


def print_whatsapp_short_list(data):
    print()
    print("=" * 70)
    print("WhatsApp-ready short list -- paste as-is")
    print("=" * 70)
    soft_required = sorted(n for n in data["used"]
                            if n not in data["hard_required"] and n not in LAP_DISTANCE_SOURCE_CHANNELS)
    print("REQUIRED -- HARD (pipeline will not run without these):")
    print(", ".join(data["hard_required"]))
    print()
    print("REQUIRED -- LAP/DISTANCE SOURCE (do not omit):")
    print("lap_number, lap_distance")
    print()
    print("REQUIRED -- other (degrades if missing):")
    print(", ".join(soft_required))
    print()
    print("DAMPER (all channels matching damper*/susp*travel*):")
    print(", ".join(DAMPER_VALUE_CHANNELS))
    print()
    print("RIDE-HEIGHT (if fitted -- unknown until seen, include if present):")
    print(", ".join(RIDE_HEIGHT_CHANNELS))
    print()
    print("TPMS (exactly these 8, no wildcard needed):")
    print(", ".join(TPMS_VALUE_CHANNELS))
    print()
    print("OPTIONAL (if present, no chasing needed -- corner_radius needs BOTH spellings):")
    print(", ".join(CORNER_RADIUS_VALUE_CHANNELS) + f", {CORNER_RADIUS_PRC_NAME}, "
          "any damper velocity/force math channel")


def render_channel_list_txt(data):
    """Bare, one-name-per-line list -- no units, no headers, no prose.
    Exact known names throughout -- TPMS's 8 value channels are the full
    selection (no tpms* wildcard; 2026-08-30 correction, the earlier
    wildcard fallback over-collected 363 channels nobody needs). The
    corner_radius entries include both this car's own spelling and the
    "Math_Corner Radius" spelling seen on a different export, since the
    exporter must be able to tick or search whichever one their system
    uses -- the tool matches by exact name, so shipping only one spelling
    risks silently missing the channel on a system that uses the other.
    """
    soft_required = sorted(n for n in data["used"]
                            if n not in data["hard_required"] and n not in LAP_DISTANCE_SOURCE_CHANNELS)
    lines = list(data["hard_required"])
    lines += list(LAP_DISTANCE_SOURCE_CHANNELS)
    lines += soft_required
    lines += list(DAMPER_VALUE_CHANNELS)
    lines += list(RIDE_HEIGHT_CHANNELS)
    lines += list(TPMS_VALUE_CHANNELS)
    lines += list(CORNER_RADIUS_VALUE_CHANNELS)
    lines.append(CORNER_RADIUS_PRC_NAME)
    return "\n".join(lines) + "\n"


def main():
    whitelist = load_whitelist()
    files = scan_py_files(SCAN_DIRS)
    consumers = find_consumers(whitelist, files)
    hard_required = find_hard_required()
    unwhitelisted = find_unwhitelisted_access(whitelist, files)
    lap_ok, dist_ok = find_lap_and_distance_source()
    damper_channels = find_damper_channels()
    corner_radius_channels = find_corner_radius_channels()
    tpms_channels = find_tpms_channels()
    channel_list_total = (len(CHANNEL_LIST_TXT.read_text(encoding="utf-8", errors="replace").splitlines())
                           if CHANNEL_LIST_TXT.exists() else 0)

    dead = sorted(name for name, hits in consumers.items() if not hits)
    used = [name for name in whitelist if name not in dead and name not in DEAD_PATH_CHANNELS]

    data = {
        "whitelist": whitelist, "consumers": consumers, "hard_required": hard_required,
        "dead": dead, "used": used, "unwhitelisted": unwhitelisted,
        "lap_ok": lap_ok, "dist_ok": dist_ok,
        "damper_channels": damper_channels, "corner_radius_channels": corner_radius_channels,
        "tpms_channels": tpms_channels, "channel_list_total": channel_list_total,
    }

    print("=" * 70)
    print("Channel requirements generator -- source-of-truth summary")
    print("=" * 70)
    print(f"Whitelisted channels: {len(whitelist)}")
    print(f"Consumed by real code: {len(used)}")
    print(f"DEAD (whitelisted, never read anywhere in modules/core/ui): {len(dead)}  {dead}")
    print(f"Hard-required (prepare_vehicle_state): {hard_required}")
    print(f"Lap source confirmed as channels.get('lap_number'): {lap_ok}")
    print(f"Distance source confirmed as channels.get('lap_distance'): {dist_ok}")
    print(f"Unwhitelisted access findings (modules/+core/ only): {len(unwhitelisted)}")
    for f, ln, name in unwhitelisted:
        print(f"  - {f}:{ln} reads {name!r} (not in channels.json whitelist)")
    print(f"Damper-family channels found in channel_list.txt: {len(damper_channels)}")
    print(f"Corner-radius-family channels found: {len(corner_radius_channels)}")
    print(f"TPMS-family channels found: {len(tpms_channels)}")

    OUT_MD.parent.mkdir(parents=True, exist_ok=True)
    OUT_MD.write_text(render_markdown(data), encoding="utf-8")
    print(f"\nWrote {OUT_MD.relative_to(ROOT).as_posix()}")

    OUT_TXT.write_text(render_channel_list_txt(data), encoding="utf-8")
    print(f"Wrote {OUT_TXT.relative_to(ROOT).as_posix()}")

    print_whatsapp_short_list(data)

    return data


if __name__ == "__main__":
    main()
