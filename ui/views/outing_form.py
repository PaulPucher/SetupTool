# Outing form -- full form for creating a new outing.

import collections
import os
import traceback
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QScrollArea, QPushButton,
    QLineEdit, QComboBox, QTextEdit,
    QDateTimeEdit, QDoubleSpinBox,
    QGroupBox, QFrame, QSpinBox,
    QTableWidget, QTableWidgetItem, QCheckBox,
    QHeaderView, QAbstractSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, QDateTime, QThread, pyqtSignal, QTimer
from models.base import Session
from models.driver import Driver
from models.outing import Outing
from core.config_loader import get_setup_parameters
from ui.style import ACCENT, OK, WARN, BAD, NEUTRAL, TEXT, TEXT_MUTED, TEXT_DIM, PANEL, PANEL_ALT, BORDER
from ui.views.measurement_points_widget import MeasurementPointsWidget
from core.error_text import friendly_error_text

# WARN boundary as a fraction of the BAD (stab_neg_thresh) boundary -- ratio
# inherited from the original -200/-500 design so detail colours track the
# verdict threshold automatically. [neutral engineering]
STAB_COLOUR_WARN_FRACTION = 0.4

# Corner-map marker click hit-test radius, px -- matches the marker dot's
# own on-screen size (size=26 in _update_corner_map_markers) so the click
# target feels like "the dot", not a much larger or smaller invisible zone.
# Converted to view (data) coordinates at click time via viewPixelSize().
CORNER_MARKER_CLICK_RADIUS_PX = 26


def _norm_path(path):
    # Shared csv_path comparison for both the WP5 DB cache and the WP6
    # in-memory pipeline cache -- normalises case and separators so the
    # same file picked via different casing/slashes still matches.
    if not path:
        return path
    return os.path.normcase(os.path.normpath(path))


def invalidate_all_pipeline_caches():
    # PART B amendment: called by the settings page after a section-1 save.
    # Redundant safety net, not the primary defense (resolved_vehicle_
    # snapshot already carries the section-1 constants, so a genuine
    # physics edit is caught structurally by the identity check below
    # regardless). FIX 2 moved the pipeline cache from per-OutingForm-
    # instance to this module-level singleton, so clearing it here is the
    # only thing this function needs to do now -- no per-instance loop.
    _pipeline_cache_store.clear()


# FIX 2 (session-persistent pipeline cache, 2026-07-28): previously an
# OutingForm INSTANCE attribute, reset to None on every fresh CSV load --
# closing an outing (discarding its OutingForm) and reopening it, or
# opening a second outing that points at the SAME csv_path, always forced
# a full Modules-1-5 recompute even though the exact same file had already
# been analysed once this session. Keyed by the same normalised csv_path
# _norm_path already produces; capped at _PIPELINE_CACHE_MAX_ENTRIES,
# OLDEST-ACCESSED entry evicted first (collections.OrderedDict.move_to_end
# on every hit, so "oldest" tracks recency of USE, not just of insertion)
# -- bounded memory footprint regardless of how many different files get
# analysed in one session. All existing identity fields (accuracy_cap,
# resolved_vehicle_snapshot) are unchanged and still checked at read time
# (_pipeline_cache_get's caller) -- only WHERE the entry lives changed,
# not what makes an entry valid.
_PIPELINE_CACHE_MAX_ENTRIES = 2
_pipeline_cache_store = collections.OrderedDict()


def _pipeline_cache_get(csv_path):
    key = _norm_path(csv_path)
    entry = _pipeline_cache_store.get(key)
    if entry is not None:
        _pipeline_cache_store.move_to_end(key)
    return entry


def _pipeline_cache_put(csv_path, entry):
    key = _norm_path(csv_path)
    _pipeline_cache_store[key] = entry
    _pipeline_cache_store.move_to_end(key)
    while len(_pipeline_cache_store) > _PIPELINE_CACHE_MAX_ENTRIES:
        _pipeline_cache_store.popitem(last=False)


class NoScrollSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class NoScrollIntSpinBox(QSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class CsvLoaderThread(QThread):
    finished = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self, path):
        super().__init__()
        self.path = path

    def run(self):
        try:
            from modules.csv_parser import parse_csv
            result = parse_csv(self.path)
            self.finished.emit(result)
        except Exception as e:
            # Reliability pass: full traceback to the console/log (for
            # diagnosis), a friendly one-line message to the UI.
            print(traceback.format_exc())
            self.error.emit(friendly_error_text(e))


class StabilityAnalysisThread(QThread):
    finished = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self, parsed_data, lap_filter, pipeline_cache=None,
                 cap=None, resolved_accuracy=None, csv_path=None):
        super().__init__()
        self.parsed_data = parsed_data
        self.lap_filter = lap_filter
        # Fresh-session work package (auto-fit modes): passed through to
        # modules.tyre_fit_auto's fit_session(s) as data_file_path, purely
        # for manifest traceability (git hash/timestamp are the real
        # reproducibility anchors) -- never read for control flow.
        self.csv_path = csv_path
        # WP6: {corners, state, cs, stab, accuracy_cap, resolved_vehicle_
        # snapshot} from a prior full run on this same csv_path AND the same
        # cap/resolved-vehicle-snapshot (matched by the caller before
        # constructing this thread -- a cap or resolved-value change behaves
        # like a csv_path change, full Modules-1-5 recompute, not a
        # lap-filter-only Module-6 recompute). When present, Modules 1-5 and
        # corner detection are NOT re-run -- only summarise_corners
        # (Module 6) re-executes for the new lap_filter. None means run the
        # full pipeline as before.
        self.pipeline_cache = pipeline_cache
        # WP-C: the global accuracy-level cap (None = "best available", or
        # int 1-4) and the already-resolved per-session accuracy (modules.
        # accuracy_resolution.resolve_accuracy's output), both computed once
        # by the caller so the same resolution backs both the pipeline-cache
        # identity check and the actual computation below.
        self.cap = cap
        self.resolved_accuracy = resolved_accuracy

    def run(self):
        # TEMPORARY perf instrumentation (WP6 timing verification) -- one
        # manual timing run, then keep or remove per user decision.
        import time
        t0 = time.perf_counter()
        try:
            from modules.stability_analysis import (
                load_parameters, prepare_vehicle_state,
                estimate_slip_angles, estimate_lateral_forces,
                estimate_cornering_stiffness, estimate_yaw_moment_stability,
                estimate_vertical_loads, summarise_corners,
            )
            from modules.accuracy_resolution import apply_resolved_vehicle
            # PLAN.md STEP 3 (LS_ratio) Phase 3: longitudinal counterpart to
            # estimate_lateral_forces/estimate_cornering_stiffness above,
            # same "read-only diagnostic, feeds Module 6/UI only" status Fz
            # had at its own Phase-3-equivalent turn -- no classify_fn input.
            from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
            from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
            # WP-N2 Step 1b / fresh-session work package: which beta a given
            # sideslip_source produces (kinematic, ekf_pass_1, or the two
            # auto-fit modes) is modules.tyre_fit_auto.resolve_sideslip_
            # beta's job now, called below -- estimate_sideslip and the
            # Dugoff EKF are imported there, not here.
            pipeline_cache_hit = self.pipeline_cache is not None
            if self.pipeline_cache is not None:
                corners = self.pipeline_cache["corners"]
                state = self.pipeline_cache["state"]
                cs = self.pipeline_cache["cs"]
                stab = self.pipeline_cache["stab"]
                fz = self.pipeline_cache["fz"]
                # PLAN.md STEP 3 Phase 3: .get() so a pipeline-cache entry
                # written by a pre-this-package session (no "ls" key)
                # degrades to None instead of KeyError -- same convention
                # as slip/forces below.
                ls = self.pipeline_cache.get("ls")
                # WP-A item 3: slip/forces (alpha_*_filt/Fy_*_filt) join the
                # cache alongside cs/stab/fz -- same precedent as fz's own
                # WP5b(b) addition. Only the corner-trace dialog's tyre-curve
                # tab reads these; .get() so an entry cached by an older
                # session (before this key existed) degrades to None there
                # instead of KeyError.
                slip = self.pipeline_cache.get("slip")
                forces = self.pipeline_cache.get("forces")
                sideslip_source = self.pipeline_cache.get("sideslip_source", "kinematic")
                # Fresh-session work package: fit_manifest/gate_verdict/
                # fallback_used/fallback_reason join the cache alongside
                # slip/forces above, same reasoning -- a lap-filter-only
                # re-Analyse under an auto mode must not lose the estimator-
                # status line just because Modules 1-5 were reused rather
                # than recomputed. .get() with None/False defaults so a
                # pre-this-package cached entry degrades to None/False
                # instead of KeyError.
                fit_manifest = self.pipeline_cache.get("fit_manifest")
                gate_verdict = self.pipeline_cache.get("gate_verdict")
                fallback_used = self.pipeline_cache.get("fallback_used", False)
                fallback_reason = self.pipeline_cache.get("fallback_reason")
            else:
                params = load_parameters()
                # WP-C: substitute the resolved (and cap-clipped) mass/
                # corner_weights/cog values wherever Modules 1-5 read
                # params["vehicle"] -- neither function's own body changes,
                # they read the same keys as always, just off this
                # deep-copied effective dict instead of the shared
                # lru_cache'd one.
                effective_params = apply_resolved_vehicle(params, self.resolved_accuracy)
                state = prepare_vehicle_state(self.parsed_data["channels"], effective_params)
                if state is None:
                    self.error.emit("Required channels missing or failed")
                    return
                sideslip_source = effective_params["stability_estimation"].get(
                    "sideslip_source", "kinematic"
                )
                # Fresh-session work package: dispatch (ekf_pass_1 / the two
                # auto-fit modes / kinematic) lives in modules/tyre_fit_auto.
                # resolve_sideslip_beta, not inline here -- keeps this QThread
                # a thin caller and makes the dispatch logic directly
                # testable without Qt (tests/test_auto_fit_wiring.py calls
                # the exact same function). Timed separately from the rest
                # of Modules 1-5 so the fit chain's own wall-clock is visible
                # regardless of which mode is active (near-zero for
                # kinematic/ekf_pass_1, the actual fit+sweep cost for the
                # two auto modes).
                from modules.tyre_fit_auto import resolve_sideslip_beta
                t_fit0 = time.perf_counter()
                beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
                    state, effective_params, self.parsed_data, sideslip_source, csv_path=self.csv_path
                )
                t_fit1 = time.perf_counter()
                fit_time_s = t_fit1 - t_fit0
                if sideslip_source in ("ekf_auto_dugoff", "ekf_auto_pacejka"):
                    print(f"[PERF] {sideslip_source} fit chain: {fit_time_s:.3f}s")
                    if fit_time_s > 30.0:
                        print(f"[PERF] *** WARNING: {sideslip_source} fit chain took {fit_time_s:.3f}s, "
                              f"exceeds the 30s budget (production performance NOT optimised for this "
                              f"per the work order -- reported, not fixed) ***")
                slip = estimate_slip_angles(state, beta, effective_params)
                forces = estimate_lateral_forces(state, effective_params)
                cs = estimate_cornering_stiffness(slip, forces, state, effective_params)
                stab = estimate_yaw_moment_stability(state, beta, effective_params, self.parsed_data.get("laps", []))
                # WP5b(b) phase 1 turn (b): read-only Fz/fy_norm diagnostic,
                # feeds Module 6/UI only -- no classify_fn input.
                fz = estimate_vertical_loads(state, forces, effective_params)
                # PLAN.md STEP 3 Phase 3: same read-only-diagnostic status.
                long_forces = estimate_longitudinal_forces(state, self.parsed_data["channels"], effective_params)
                slip_ratio = estimate_slip_ratio(state, self.parsed_data["channels"], effective_params)
                ls = estimate_longitudinal_stiffness(long_forces, slip_ratio, state, effective_params)
                corners = self.parsed_data.get("corners", [])
            t_modules = time.perf_counter()
            print(f"[PERF] Modules 1-5: {t_modules - t0:.3f}s  pipeline_cache_hit={pipeline_cache_hit}")
            summaries = summarise_corners(corners, cs, stab, state, fz=fz, ls=ls,
                                          lap_filter=self.lap_filter)
            t_summarise = time.perf_counter()
            print(f"[PERF] summarise_corners: {t_summarise - t_modules:.3f}s")
            self.finished.emit({
                "summaries": summaries,
                "state": state,
                "cs": cs,
                "stab": stab,
                "fz": fz,
                "ls": ls,
                "slip": slip,
                "forces": forces,
                "corners": corners,
                "cap": self.cap,
                # Cleanup pass, Phase 1: lets CornerTraceDialog respect the
                # analysis's own lap selection (single lap chosen -> traces
                # default to that lap only) instead of always defaulting
                # every valid lap to checked regardless of what was
                # actually analysed.
                "lap_filter": self.lap_filter,
                "resolved_accuracy": self.resolved_accuracy,
                "sideslip_source": sideslip_source,
                "fit_manifest": fit_manifest,
                "gate_verdict": gate_verdict,
                "fallback_used": fallback_used,
                "fallback_reason": fallback_reason,
            })
            t_total = time.perf_counter()
            print(f"[PERF] thread total: {t_total - t0:.3f}s  pipeline_cache_hit={pipeline_cache_hit}")
        except Exception as e:
            # Reliability pass: same convention as CsvLoaderThread.run()
            # above -- full traceback to the console/log, a one-line
            # message to the UI's status label.
            print(traceback.format_exc())
            self.error.emit(friendly_error_text(e))


class OutingForm(QWidget):
    def __init__(self, weekend, on_back, outing=None):
        super().__init__()
        self.weekend = weekend
        self.on_back = on_back
        self.outing = outing
        self.setup_inputs = {}
        self.setdown_inputs = {}
        self.feedback_map_path = None
        self.corner_rows = []
        self.parsed_data = None
        self.loaded_csv_path = None
        self.stability_result = None
        self.corner_positions_cache = None
        self.corner_map_trace_xy = None
        # PART C: lazily-created, reused per-corner trace window (see
        # ui/views/corner_trace_dialog.py) -- None until first opened.
        self._corner_trace_dialog = None
        # Lap-trace-view work package: same lazy/reused convention as
        # _corner_trace_dialog, a separate window/instance (LapTraceDialog
        # shares CornerTraceDialog's base class but is its own dialog, not
        # a mode switch on the same one) -- None until first opened.
        self._lap_trace_dialog = None
        # WP6: {csv_path, corners, state, cs, stab} from the last full
        # WP5: JSON string mirroring whatever analysis_data should be
        # persisted on next save (fresh analysis result, or an untouched
        # cache-hit's raw string), or None. FIX 2: the WP6 Modules-1-5
        # pipeline cache itself is no longer instance state -- see the
        # module-level _pipeline_cache_store/_pipeline_cache_get/_put
        # near the top of this file.
        self._analysis_data_json = None
        # WP-small: the resolved_vehicle_snapshot (modules.accuracy_
        # resolution.resolve_accuracy's "values") behind whatever is
        # currently rendered in the stability section, or None if nothing
        # is rendered -- lets an explicit Save compare newly-saved setup
        # data against what the displayed analysis actually used.
        self._displayed_resolved_vehicle_snapshot = None
        # Fix turn: (stored_version, current_version) when
        # _try_render_cached_analysis rejects a persisted cache purely for
        # a schema_version mismatch -- lets _generate_recommendations (and
        # any other stability_result consumer) tell that case apart from
        # "never analysed" and say so instead of rendering nothing.
        # Cleared in _render_stability_summaries, the single shared render
        # call site both a fresh Analyse and a successful cache-hit go
        # through.
        self._cached_schema_mismatch = None

        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        outer_layout.addWidget(self._build_header())

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        content = QWidget()
        self.content_layout = QVBoxLayout(content)
        self.content_layout.setContentsMargins(24, 24, 24, 24)
        self.content_layout.setSpacing(32)

        self.content_layout.addWidget(self._build_session_section())
        self.content_layout.addWidget(self._build_data_section())
        self.content_layout.addWidget(self._build_setup_section("setup"))
        self.content_layout.addWidget(self._build_setdown_toggle())
        self.content_layout.addWidget(self._build_corner_map())
        self.content_layout.addWidget(self._build_stability_toggle())
        self.content_layout.addWidget(self._build_recommendations_toggle())
        self.content_layout.addWidget(self._build_decision_frame_toggle())
        self.content_layout.addWidget(self._build_feedback_section())
        self.content_layout.addWidget(self._build_comments_section())
        self.content_layout.addStretch()

        if self.outing:
            self._prefill()
        else:
            self._carryon_from_last()

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

    def _sideslip_source_calibrated(self):
        # WP-N2 Step 1b: single source of truth for the traces-vs-verdicts
        # gate, shared by the stability banner, the recommendations banner,
        # and (indirectly, via _classify_corner) every per-verdict marker
        # and the PDF export. Pure config comparison -- independent of which
        # data is currently rendered, since the WP5/WP6 cache identity
        # checks already guarantee rendered data matches the live config's
        # sideslip_source.
        from modules.stability_analysis import load_parameters
        params = load_parameters()
        active = params["stability_estimation"].get("sideslip_source", "kinematic")
        calibrated_for = params["classification"].get(
            "thresholds_calibrated_for_sideslip_source", "kinematic"
        )
        return active == calibrated_for

    def _stability_colour(self, kind, value, axle="f"):
        # Align with _classify_corner thresholds so details colours match verdicts.
        # CS thresholds differ front vs rear because rear normally stays stiffer.
        if value is None or (isinstance(value, float) and value != value):
            return NEUTRAL
        if kind == "cs":
            if axle == "r":
                if value >= 0.35:
                    return OK
                if value >= 0.20:
                    return WARN
                return BAD
            else:
                if value >= 0.25:
                    return OK
                if value >= 0.10:
                    return WARN
                return BAD
        if kind == "stab":
            from modules.stability_analysis import load_parameters
            bad_thresh = load_parameters()["classification"]["stab_neg_thresh_Nm_per_deg"]["value"]
            warn_thresh = bad_thresh * STAB_COLOUR_WARN_FRACTION
            if value > warn_thresh:
                return OK
            if value > bad_thresh:
                return WARN
            return BAD
        return TEXT_MUTED

    def _classify_corner(self, summary):
        # Returns (severity, short_verdict, long_verdict, colour).
        # Thresholds are config-driven (config/parameters.json classification
        # block); each carries its own derived_from note there. Values only,
        # not the derivation history -- see thesis_notes.md for that.
        from modules.stability_analysis import load_parameters
        params = load_parameters()
        cls_cfg = params["classification"]
        # WP-N2 Step 1b: thresholds below are only known-valid for the
        # sideslip source named in thresholds_calibrated_for_sideslip_source
        # (re-derived only at Step 4, deliberately deferred -- PLAN.md
        # PARKED). A mismatch means the CS_ratio/stability distribution
        # feeding this classification has shifted out from under thresholds
        # fitted to a different source; the verdict below is not wrong, but
        # its severity boundaries are unvalidated for this data. Placeholder
        # marker text -- wording to be finalised after visual review.
        active_sideslip_source = params["stability_estimation"].get(
            "sideslip_source", "kinematic"
        )
        calibrated_for = cls_cfg.get("thresholds_calibrated_for_sideslip_source", "kinematic")
        uncalibrated_marker = "" if active_sideslip_source == calibrated_for else " [UNCAL]"
        # Threshold anchoring, Phase 2 (2026-09-02): stab_neg_thresh itself
        # was NOT re-derived for ekf_auto_pacejka (no negative population to
        # anchor a margin against, thesis_notes.md "Threshold anchoring,
        # Phase 2") and stays on its kinematic-era gap-selected value --
        # a second, narrower marker than [UNCAL] above (which now reports
        # CS-threshold calibration only), appended only to a firing
        # unstable-yaw verdict specifically, since that is the only verdict
        # this legacy value governs.
        stab_calibrated_for = cls_cfg.get("stab_thresh_calibrated_for_sideslip_source", "kinematic")
        stab_legacy_marker = "" if active_sideslip_source == stab_calibrated_for else " [stab thresh: kinematic-era, not re-derived]"
        STRONG_CSF = cls_cfg["STRONG_CSF"]["value"]
        STRONG_CSR = cls_cfg["STRONG_CSR"]["value"]
        MODERATE_CSF = cls_cfg["MODERATE_CSF"]["value"]
        MODERATE_CSR = cls_cfg["MODERATE_CSR"]["value"]
        STAB_NEG_THRESH = cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"]

        worst_f_phase = None
        worst_f_val = 1.0
        worst_r_phase = None
        worst_r_val = 1.0
        worst_stab_phase = None
        worst_stab_val = 1e9

        phase_labels_short = {
            "entry_1_brake": "brake",
            "entry_2_turnin": "turn-in",
            "apex_3": "apex",
            "exit_4": "exit",
            "exit_5": "exit",
        }
        phase_labels_long = {
            "entry_1_brake": "brake",
            "entry_2_turnin": "turn-in",
            "apex_3": "apex",
            "exit_4": "early exit",
            "exit_5": "late exit",
        }

        # CS validity repair part A, Phase 3: apex_3's CS reads come from
        # apex_region (a distance-based window around the apex, config
        # cs_apex_region_half_length_m) instead of apex_3's own structurally
        # fixed 11-sample slice (thesis_notes.md "apex_3 structural
        # finding") -- apex_3 still supplies its own stability median and
        # every other phase's CS reads are unaffected.
        apex_region = summary.get("apex_region")
        for phase, p in summary["phases"].items():
            if phase == "apex_3" and apex_region is not None:
                csf = apex_region["cs_ratio_f"]["median"]
                csr = apex_region["cs_ratio_r"]["median"]
            else:
                csf = p["cs_ratio_f"]["median"]
                csr = p["cs_ratio_r"]["median"]
            sob = p["stability_observed_Nm_per_deg"]["median"]
            if csf == csf and csf < worst_f_val:
                worst_f_val = csf
                worst_f_phase = phase
            if csr == csr and csr < worst_r_val:
                worst_r_val = csr
                worst_r_phase = phase
            if sob == sob and sob < worst_stab_val:
                worst_stab_val = sob
                worst_stab_phase = phase

        front_strong_cs = worst_f_val < STRONG_CSF
        rear_strong_cs = worst_r_val < STRONG_CSR
        front_moderate_cs = STRONG_CSF <= worst_f_val < MODERATE_CSF
        rear_moderate_cs = STRONG_CSR <= worst_r_val < MODERATE_CSR
        destabilising = (worst_stab_val == worst_stab_val
                         and worst_stab_val < STAB_NEG_THRESH)

        # Vocabulary intentionally limited to: understeer / oversteer / unstable yaw.
        # We pick the dominant axle behaviour and the phase where it's worst.
        # Front issue -> understeer. Rear issue -> oversteer. Both -> the worse one.
        short_parts = []
        long_parts = []
        severity = "normal"

        # Decide which axle leads the verdict
        # (rear collapse is rarer and more consequential, so it wins ties)
        front_active = front_strong_cs or front_moderate_cs
        rear_active = rear_strong_cs or rear_moderate_cs

        primary = None  # "understeer" or "oversteer" or None
        primary_phase = None
        primary_val = None
        primary_axle = None

        if rear_strong_cs:
            primary, primary_phase, primary_val, primary_axle = (
                "oversteer", worst_r_phase, worst_r_val, "r")
        elif front_strong_cs:
            primary, primary_phase, primary_val, primary_axle = (
                "understeer", worst_f_phase, worst_f_val, "f")
        elif rear_moderate_cs:
            primary, primary_phase, primary_val, primary_axle = (
                "oversteer", worst_r_phase, worst_r_val, "r")
        elif front_moderate_cs:
            primary, primary_phase, primary_val, primary_axle = (
                "understeer", worst_f_phase, worst_f_val, "f")

        if (front_strong_cs or rear_strong_cs) and destabilising:
            severity = "strong"
        elif front_strong_cs or rear_strong_cs:
            severity = "moderate"
        elif (front_moderate_cs or rear_moderate_cs) and destabilising:
            severity = "moderate"
        elif destabilising:
            severity = "moderate"
        # else stays "normal"

        if primary is not None:
            short_parts.append(f"{primary} @ {phase_labels_short[primary_phase]}")
            cs_label = "CSf" if primary_axle == "f" else "CSr"
            long_parts.append(
                f"{primary} at {phase_labels_long[primary_phase]} "
                f"({cs_label} {primary_val:.2f})"
            )

        if destabilising:
            short_parts.append(f"unstable yaw @ {phase_labels_short[worst_stab_phase]}{stab_legacy_marker}")
            long_parts.append(
                f"unstable yaw at {phase_labels_long[worst_stab_phase]} "
                f"({worst_stab_val:.0f} Nm/deg){stab_legacy_marker}"
            )

        if not short_parts:
            short_parts.append("ok")
        if not long_parts:
            long_parts.append("within normal range")

        colour_map = {"strong": BAD, "moderate": WARN, "normal": OK}
        return (
            severity,
            " | ".join(short_parts) + uncalibrated_marker,
            " | ".join(long_parts) + uncalibrated_marker,
            colour_map[severity],
        )

    def _build_header(self):
        header = QWidget()
        header.setFixedHeight(52)
        header.setStyleSheet("border-bottom: 1px solid #222;")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(20, 0, 20, 0)

        btn_back = QPushButton("< Back")
        btn_back.setFixedWidth(80)
        btn_back.setStyleSheet("background-color: #252525; color: #888;")
        btn_back.clicked.connect(self._save_outing)

        # Small WP: explicit Save, enabled in both new-outing and edit
        # modes -- persists via the same _persist_outing() core Back uses,
        # but stays on the page (see _on_save_clicked).
        self.btn_save = QPushButton("Save")
        self.btn_save.setFixedWidth(80)
        self.btn_save.setStyleSheet("background-color: #252525; color: #888;")
        self.btn_save.clicked.connect(self._on_save_clicked)

        title = QLabel(f"New Outing - {self.weekend.track}")
        title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        layout.addWidget(btn_back)
        layout.addSpacing(8)
        layout.addWidget(self.btn_save)
        layout.addSpacing(16)
        layout.addWidget(title)
        layout.addStretch()

        if self.outing:
            btn_delete = QPushButton("Delete")
            btn_delete.setFixedWidth(80)
            btn_delete.setStyleSheet("background-color: #252525; color: #c0392b;")
            btn_delete.clicked.connect(self._delete_outing)
            layout.addWidget(btn_delete)

        return header

    def _section_label(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size: 13px; font-weight: 600; color: #C0A060; margin-bottom: 8px;")
        return label

    def _row(self, label_text, widget):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(label_text)
        label.setFixedWidth(120)
        label.setStyleSheet("color: #888;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        return row

    def _setup_row(self, label_text, widget):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(8)
        label = QLabel(label_text)
        label.setFixedWidth(100)
        label.setStyleSheet("color: #888; font-size: 11px;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        return row

    def _build_session_section(self):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._section_label("Session"))

        self.datetime_edit = QDateTimeEdit()
        self.datetime_edit.setDateTime(QDateTime.currentDateTime())
        self.datetime_edit.setCalendarPopup(True)

        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Optional name for this outing")

        self.driver_combo = QComboBox()
        self._load_drivers()

        self.session_type_combo = QComboBox()
        self.session_type_combo.addItems(["Practice", "Qualifying", "Race", "Warmup"])

        self.tyre_type_combo = QComboBox()
        self.tyre_type_combo.addItems(["Dry", "Wet"])

        self.tyre_name_input = QLineEdit()
        self.tyre_name_input.setPlaceholderText("e.g. Prc Set 1")

        self.tyre_age_input = NoScrollSpinBox()
        self.tyre_age_input.setSuffix(" km")
        self.tyre_age_input.setDecimals(1)
        self.tyre_age_input.setRange(0, 9999)

        self.fuel_load_input = NoScrollSpinBox()
        self.fuel_load_input.setSuffix(" L")
        self.fuel_load_input.setDecimals(1)
        self.fuel_load_input.setRange(0, 200)

        self.air_temp_input = NoScrollSpinBox()
        self.air_temp_input.setSuffix(" degC")
        self.air_temp_input.setRange(-20, 80)
        self.air_temp_input.setDecimals(1)

        self.track_temp_input = NoScrollSpinBox()
        self.track_temp_input.setSuffix(" degC")
        self.track_temp_input.setRange(-20, 80)
        self.track_temp_input.setDecimals(1)

        self.track_condition_combo = QComboBox()
        self.track_condition_combo.addItems(["Dry", "Damp", "Wet"])

        layout.addWidget(self._row("Date & Time", self.datetime_edit))
        layout.addWidget(self._row("Name", self.name_input))
        layout.addWidget(self._row("Driver", self.driver_combo))
        layout.addWidget(self._row("Session Type", self.session_type_combo))
        layout.addWidget(self._row("Tyre Type", self.tyre_type_combo))
        layout.addWidget(self._row("Tyre Name", self.tyre_name_input))
        layout.addWidget(self._row("Tyre Age", self.tyre_age_input))
        layout.addWidget(self._row("Fuel Load", self.fuel_load_input))
        layout.addWidget(self._row("Air Temp", self.air_temp_input))
        layout.addWidget(self._row("Track Temp", self.track_temp_input))
        layout.addWidget(self._row("Track Condition", self.track_condition_combo))

        return section

    def _build_data_section(self):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._section_label("Data"))

        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(10)

        btn_load = QPushButton("Load Outing")
        btn_load.setFixedWidth(120)
        btn_load.clicked.connect(self._load_csv)

        self.btn_clear_data = QPushButton("Clear Data")
        self.btn_clear_data.setFixedWidth(100)
        self.btn_clear_data.clicked.connect(self._on_clear_data_clicked)

        self.btn_analyse = QPushButton("Analyse")
        self.btn_analyse.setFixedWidth(100)
        self.btn_analyse.clicked.connect(self._run_stability_analysis)
        self.btn_analyse.setEnabled(False)

        # Lap-trace-view work package: enabled alongside Generate
        # (_render_stability_summaries, the single shared render call site
        # for both a fresh Analyse and a cache-hit) -- "an analysis
        # exists" is the same precondition both features need.
        self.btn_lap_traces = QPushButton("Lap traces")
        self.btn_lap_traces.setFixedWidth(100)
        self.btn_lap_traces.clicked.connect(self._open_lap_trace)
        self.btn_lap_traces.setEnabled(False)

        # WP-C: global accuracy-level cap. A plain UI-selected value threaded
        # into the analysis call like lap_filter -- modules/ never reads this
        # combo box directly, only the int (or None for "best available")
        # _get_accuracy_cap_from_selector() returns.
        self.accuracy_cap_combo = QComboBox()
        self.accuracy_cap_combo.addItems(
            ["Best available", "Level 1", "Level 2", "Level 3", "Level 4"]
        )
        self.accuracy_cap_combo.setFixedWidth(130)

        # Fresh-session work package, Phase 3a: sideslip-estimator mode
        # selector. Replaces config-file-only switching -- selecting an
        # item WRITES config/parameters.json's stability_estimation.
        # sideslip_source directly (imitates ui/views/settings_view.py's
        # own _on_save_clicked persistence pattern: full read-modify-
        # write of the JSON file, then load_parameters.cache_clear() +
        # invalidate_all_pipeline_caches() -- investigated first, see
        # thesis_notes.md for why this pattern was chosen over an
        # ephemeral per-session QComboBox value like accuracy_cap_combo:
        # accuracy_cap_combo has NO cross-restart persistence at all
        # (verified by reading _prefill/_carryon_from_last, neither
        # touches it), which fails this phase's explicit "persists across
        # restarts" requirement, whereas config-file persistence already
        # exists for this exact field and is restart-persistent by
        # construction). Participates in cache identity exactly as the
        # config key does today because it IS the config key -- no new
        # identity field needed anywhere.
        # UI cleanup package: "EKF (frozen Dubai fit)" / "ekf_pass_1"
        # REMOVED from the SELECTABLE items -- it is the validated
        # baseline (the frozen pass-1 manifest tests/diagnostics compare
        # against) and stays fully reachable by editing config/parameters.
        # json's stability_estimation.sideslip_source directly, which is
        # why it must not be deleted from _ESTIMATOR_LABELS above (that
        # mapping is still used for the status line/PDF whenever ekf_
        # pass_1 IS the active mode, config-selected). Only the DROPDOWN's
        # own selectable set shrinks to the three modes users are meant
        # to choose day to day.
        # ekf_auto_dugoff marked experimental in the dropdown (CS validity
        # repair, Phase 4, user decision): its rear-axle fit degenerates on
        # this car's data under the final CS window floor and falls back to
        # kinematic on every run -- kept selectable (a designed, loud
        # fallback, not a crash) rather than removed, but the label must
        # not imply it is as trustworthy as the other two modes.
        self._SIDESLIP_MODE_DISPLAY_TO_VALUE = {
            "Kinematic": "kinematic",
            "EKF auto Dugoff (experimental)": "ekf_auto_dugoff",
            "EKF auto Pacejka": "ekf_auto_pacejka",
        }
        self._SIDESLIP_MODE_VALUE_TO_DISPLAY = {
            v: k for k, v in self._SIDESLIP_MODE_DISPLAY_TO_VALUE.items()
        }
        self.sideslip_mode_combo = QComboBox()
        self.sideslip_mode_combo.addItems(list(self._SIDESLIP_MODE_DISPLAY_TO_VALUE.keys()))
        self.sideslip_mode_combo.setFixedWidth(230)
        from modules.stability_analysis import load_parameters as _load_params_for_init
        _current_mode = _load_params_for_init()["stability_estimation"].get(
            "sideslip_source", "kinematic"
        )
        # If config is currently "ekf_pass_1" (config-level selection,
        # not reachable from this dropdown any more), the lookup below
        # misses and falls back to "Kinematic" -- the combo cannot
        # display a value outside its own item list (same accepted,
        # already-shipped QComboBox.setCurrentText() no-op-on-unmatched-
        # value behaviour as wing_position's own legacy-value handling).
        # This does NOT affect what actually runs: Analyse always reads
        # sideslip_source from config, never from the combo's current
        # display text, and estimator_status_label (post-analysis)
        # correctly names "EKF (frozen pass-1 Dugoff fit)" regardless of
        # what the dropdown shows. Documented, not silently accepted.
        self.sideslip_mode_combo.setCurrentText(
            self._SIDESLIP_MODE_VALUE_TO_DISPLAY.get(_current_mode, "Kinematic")
        )
        # Connected AFTER the initial setCurrentText above, so populating
        # the widget at construction time never triggers a config write.
        self.sideslip_mode_combo.currentTextChanged.connect(self._on_sideslip_mode_changed)

        self.csv_status_label = QLabel("No file loaded")
        self.csv_status_label.setStyleSheet("color: #555; font-size: 12px;")

        self.stability_status_label = QLabel("")
        self.stability_status_label.setStyleSheet("color: #555; font-size: 12px;")

        # Fresh-session work package, Phase 3b: which estimator actually
        # produced beta (auto modes can fall back!), fit status, gate
        # verdict, fallback reason -- see _format_estimator_status.
        self.estimator_status_label = QLabel("")
        self.estimator_status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self.estimator_status_label.setWordWrap(True)
        self.estimator_status_label.setVisible(False)

        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(self.btn_clear_data)
        btn_layout.addWidget(self.btn_analyse)
        btn_layout.addWidget(self.btn_lap_traces)
        btn_layout.addWidget(self.accuracy_cap_combo)
        btn_layout.addWidget(self.sideslip_mode_combo)
        btn_layout.addWidget(self.csv_status_label)
        btn_layout.addStretch()
        layout.addWidget(btn_row)

        status_row = QWidget()
        status_layout = QHBoxLayout(status_row)
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.addWidget(self.stability_status_label)
        status_layout.addStretch()
        layout.addWidget(status_row)

        estimator_status_row = QWidget()
        estimator_status_layout = QHBoxLayout(estimator_status_row)
        estimator_status_layout.setContentsMargins(0, 0, 0, 0)
        estimator_status_layout.addWidget(self.estimator_status_label)
        estimator_status_layout.addStretch()
        layout.addWidget(estimator_status_row)

        self.btn_clear_lap_selection = QPushButton("Clear Selection")
        self.btn_clear_lap_selection.setFixedWidth(120)
        self.btn_clear_lap_selection.setStyleSheet("""
            QPushButton {
                background-color: #1a1a1a;
                color: #888;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                padding: 4px 10px;
                font-size: 11px;
            }
            QPushButton:hover {
                color: #C0A060;
                border-color: #C0A060;
            }
        """)
        self.btn_clear_lap_selection.clicked.connect(self._clear_lap_selection)
        self.btn_clear_lap_selection.setVisible(False)

        lap_controls_row = QWidget()
        lap_controls_layout = QHBoxLayout(lap_controls_row)
        lap_controls_layout.setContentsMargins(0, 0, 0, 0)
        lap_controls_layout.setSpacing(8)
        lap_controls_layout.addWidget(self.btn_clear_lap_selection)
        lap_controls_layout.addStretch()
        layout.addWidget(lap_controls_row)

        # Tracks the lap_table's effective selection so a repeat click on the
        # same lap can toggle it off (Qt's SingleSelection alone can't tell
        # "clicked the already-selected row" from a fresh selection).
        self._selected_lap_value = None

        self.lap_table = QTableWidget()
        self.lap_table.setColumnCount(3)
        self.lap_table.setHorizontalHeaderLabels(["Lap", "Lap Time", ""])
        self.lap_table.verticalHeader().setVisible(False)
        self.lap_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.lap_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.lap_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.lap_table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.lap_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.lap_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.lap_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        self.lap_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        self.lap_table.setColumnWidth(0, 50)
        self.lap_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        self.lap_table.setColumnWidth(1, 100)
        self.lap_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.lap_table.setStyleSheet("""
            QTableWidget { background-color: #141414; border: none; gridline-color: #1e1e1e; outline: 0; }
            QTableWidget::item { padding: 4px; border-bottom: 1px solid #1e1e1e; color: #d0d0d0; }
            QTableWidget::item:selected { background-color: #252525; color: #C0A060; }
            QHeaderView::section { background-color: #1a1a1a; color: #555; font-size: 10px;
                padding: 6px 4px; border: none; border-bottom: 1px solid #222; }
        """)
        self.lap_table.cellClicked.connect(self._on_lap_selected)
        self.lap_table.setVisible(False)
        layout.addWidget(self.lap_table)

        self.plot_container = self._build_plot_widget()
        self.plot_container.setVisible(False)
        layout.addWidget(self.plot_container)

        return section

    def _build_plot_widget(self):
        import pyqtgraph as pg
        pg.setConfigOption('background', '#141414')
        pg.setConfigOption('foreground', '#888888')
        pg.setConfigOptions(antialias=True)

        class TimeAxisItem(pg.AxisItem):
            def tickStrings(self, values, scale, spacing):
                result = []
                for v in values:
                    mins = int(abs(v) // 60)
                    secs = abs(v) % 60
                    result.append(f"{mins}:{secs:04.1f}")
                return result

        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 8, 0, 0)
        container_layout.setSpacing(0)

        self.plot_channels = [
            {"key": "ecu_speed",    "label": "Speed (km/h)", "color": "#C0A060"},
            {"key": "ecu_aps",      "label": "Throttle (%)", "color": "#4CAF50"},
            {"key": "log_pbrake_f", "label": "Brake (bar)",  "color": "#e74c3c"},
            {"key": "ecu_nmot",     "label": "RPM",          "color": "#00bcd4"},
            {"key": "ecu_gear",     "label": "Gear",         "color": "#f1c40f"},
            {"key": "log_asteer",   "label": "Steer (deg)",    "color": "#9b59b6"},
        ]

        self.pg_layout = pg.GraphicsLayoutWidget()
        self.pg_layout.setFixedHeight(500)
        self.plot_items = {}
        self.plot_curves = {}
        self.crosshair_lines = {}
        first_plot = None

        for i, ch in enumerate(self.plot_channels):
            is_last = (i == len(self.plot_channels) - 1)
            axis_items = {'bottom': TimeAxisItem(orientation='bottom')} if is_last else {}
            plot = self.pg_layout.addPlot(axisItems=axis_items)
            plot.setLabel('left', ch["label"], color='#888', size='8pt')
            plot.showGrid(x=True, y=True, alpha=0.15)
            plot.setMaximumHeight(80)
            plot.getAxis('left').setWidth(70)
            plot.getViewBox().setMouseMode(pg.ViewBox.PanMode)
            plot.getViewBox().setMouseEnabled(x=True, y=False)
            plot.getViewBox().wheelEvent = lambda event, axis=None: None

            if not is_last:
                plot.getAxis('bottom').setStyle(showValues=False)
                plot.getAxis('bottom').setHeight(0)
            else:
                plot.setMaximumHeight(100)
                plot.getAxis('bottom').setLabel('Time (m:ss)', color='#888', size='8pt')

            if first_plot is None:
                first_plot = plot
            else:
                plot.setXLink(first_plot)

            curve = plot.plot(pen=pg.mkPen(color=ch["color"], width=1.5))
            cross = pg.InfiniteLine(
                angle=90, movable=False,
                pen=pg.mkPen(color='#444444', width=1,
                             style=Qt.PenStyle.DashLine)
            )
            plot.addItem(cross, ignoreBounds=True)

            self.plot_items[ch["key"]] = plot
            self.plot_curves[ch["key"]] = curve
            self.crosshair_lines[ch["key"]] = cross
            self.pg_layout.nextRow()

        self.pg_layout.scene().sigMouseMoved.connect(self._on_mouse_moved)
        container_layout.addWidget(self.pg_layout)
        return container

    def _on_mouse_moved(self, pos):
        if not self.plot_items:
            return
        for ch in self.plot_channels:
            plot = self.plot_items.get(ch["key"])
            if plot and plot.sceneBoundingRect().contains(pos):
                mouse_point = plot.vb.mapSceneToView(pos)
                x = mouse_point.x()
                for line in self.crosshair_lines.values():
                    line.setPos(x)
                break

    def _on_lap_selected(self, row, col):
        lap_item = self.lap_table.item(row, 0)
        if not lap_item:
            return
        value = lap_item.data(Qt.ItemDataRole.UserRole)

        if value != "all" and value == self._selected_lap_value:
            # Repeat click on the already-selected lap -- toggle off.
            self._clear_lap_selection()
            return

        self._selected_lap_value = value
        if value == "all":
            self._update_plots(None)
        else:
            self._update_plots(value)

    def _all_laps_row(self):
        for row in range(self.lap_table.rowCount()):
            item = self.lap_table.item(row, 0)
            if item is not None and item.data(Qt.ItemDataRole.UserRole) == "all":
                return row
        return None

    def _clear_lap_selection(self):
        # Move the highlight to "All laps" rather than leaving no row
        # selected -- the effective scope must stay visible, not just correct.
        all_row = self._all_laps_row()
        if all_row is not None:
            self.lap_table.selectRow(all_row)
        else:
            self.lap_table.clearSelection()
            self.lap_table.setCurrentCell(-1, -1)
        self._selected_lap_value = "all"
        self._update_plots(None)

    def _update_plots(self, lap_number):
        if not self.parsed_data:
            return

        channels = self.parsed_data["channels"]
        start_t = None
        end_t = None

        if lap_number is not None:
            lap = next(
                (l for l in self.parsed_data["laps"]
                 if l["lap_number"] == lap_number),
                None
            )
            if not lap:
                return
            start_t = lap["start_time"]
            end_t = lap["end_time"]

        for ch_config in self.plot_channels:
            key = ch_config["key"]
            if key not in channels:
                self.plot_curves[key].setData([], [])
                continue
            ch = channels[key]
            if ch["quality"] in ("missing", "failed") or ch["time"] is None:
                self.plot_curves[key].setData([], [])
                continue

            time_arr = ch["time"]
            data_arr = ch["data"]

            if start_t is not None:
                mask = (time_arr >= start_t) & (time_arr <= end_t)
                plot_time = time_arr[mask] - start_t
                plot_data = data_arr[mask]
            else:
                plot_time = time_arr - time_arr[0] if len(time_arr) > 0 else time_arr
                plot_data = data_arr

            self.plot_curves[key].setData(
                plot_time.tolist(), plot_data.tolist()
            )
            first_key = self.plot_channels[0]["key"]
        if first_key in self.plot_items:
            self.plot_items[first_key].setXRange(
                0, (end_t - start_t) if start_t is not None else
                max((ch["time"][-1] - ch["time"][0])
                    for ch in channels.values()
                    if ch["time"] is not None and len(ch["time"]) > 0),
                padding=0.02
            )
        for ch_config in self.plot_channels:
            if ch_config["key"] in self.plot_items:
                self.plot_items[ch_config["key"]].enableAutoRange(axis='y')

        self.plot_container.setVisible(True)

    def _load_csv(self):
        from PyQt6.QtWidgets import QFileDialog, QProgressDialog

        path, _ = QFileDialog.getOpenFileName(
            self, "Load Outing Data", "", "Pi Toolbox Files (*.txt *.csv);;All Files (*)"
        )
        if not path:
            return

        self.progress = QProgressDialog("Loading outing data...", None, 0, 0, self)
        self.progress.setWindowTitle("Loading")
        self.progress.setWindowModality(Qt.WindowModality.WindowModal)
        self.progress.setMinimumDuration(0)
        self.progress.setStyleSheet("""
            QProgressDialog { background-color: #1a1a1a; color: #e0e0e0; }
            QLabel { color: #e0e0e0; font-size: 12px; }
            QProgressBar {
                background-color: #141414;
                border: 1px solid #2a2a2a;
                border-radius: 3px;
                height: 6px;
            }
            QProgressBar::chunk { background-color: #C0A060; border-radius: 3px; }
        """)
        self.progress.show()

        self.loader_thread = CsvLoaderThread(path)
        self.loader_thread.finished.connect(self._on_csv_loaded)
        self.loader_thread.error.connect(self._on_csv_error)
        self.loader_thread.start()

    def _on_csv_loaded(self, result):
        import os
        from modules.csv_parser import get_available_channels
        self.progress.close()
        self.parsed_data = result
        self.loaded_csv_path = self.loader_thread.path
        # A previous file's analysis/marker cache must never leak into a
        # newly loaded file -- same bug class as stale UI widgets (WP4).
        # FIX 2: the WP6 pipeline cache itself is deliberately NOT reset
        # here any more -- it is module-level and keyed by csv_path, so
        # loading a different file simply looks up a different key (no
        # leakage risk), and reloading THIS SAME file is exactly the case
        # this fix exists to make fast.
        self.stability_result = None
        self.corner_positions_cache = None
        self._analysis_data_json = None
        self._displayed_resolved_vehicle_snapshot = None
        if self._corner_trace_dialog is not None:
            self._corner_trace_dialog.hide()
        if self._lap_trace_dialog is not None:
            self._lap_trace_dialog.hide()
        filename = os.path.basename(self.loader_thread.path)
        laps = self.parsed_data.get("laps", [])
        available = get_available_channels(self.parsed_data)
        self.csv_status_label.setText(
            f"{filename} - {len(laps)} laps, {len(available)} channels"
        )
        self.csv_status_label.setStyleSheet("color: #888; font-size: 12px;")
        self._populate_lap_table(laps)
        self._update_corner_map_trace()
        self._update_corner_map_markers()
        self.btn_analyse.setEnabled(True)
        # WP5: render immediately from a matching persisted cache, if any --
        # no lap-selector reconstruction, just the summaries as last analysed.
        self._try_render_cached_analysis()

    def _on_csv_error(self, error_msg):
        self.progress.close()
        self.csv_status_label.setText(f"Error loading file: {error_msg}")
        self.csv_status_label.setStyleSheet("color: #c0392b; font-size: 12px;")

    def _on_clear_data_clicked(self):
        # Decisions batch (Phase 2d): nothing to clear yet -- avoid a
        # confirmation dialog over an already-empty Data section.
        if not self.parsed_data and not self.loaded_csv_path:
            return
        from PyQt6.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self, "Clear data",
            "Remove the loaded data file and analysis from this outing? "
            "This cannot be undone once you Save.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._reset_data_state()

    def _reset_data_state(self):
        # Data lifecycle (Phase 2d): undo everything _on_csv_loaded and a
        # completed Analyse populate, back to the pre-load state. In-memory/
        # widget state only -- self.loaded_csv_path=None is what _save_
        # outing later persists as csv_path="" (same "stage in memory, Save
        # writes it" convention every other form field already follows), so
        # this needs no DB write of its own and a Back-without-Save discards
        # the clear exactly like it discards any other unsaved edit.
        self.parsed_data = None
        self.loaded_csv_path = None
        self.stability_result = None
        self.corner_positions_cache = None
        self._analysis_data_json = None
        self._displayed_resolved_vehicle_snapshot = None
        self._cached_schema_mismatch = None
        if self._corner_trace_dialog is not None:
            self._corner_trace_dialog.hide()
        if self._lap_trace_dialog is not None:
            self._lap_trace_dialog.hide()

        self.csv_status_label.setText("No file loaded")
        self.csv_status_label.setStyleSheet("color: #555; font-size: 12px;")
        self.stability_status_label.setText("")
        self.stability_status_label.setStyleSheet("color: #555; font-size: 12px;")
        self.estimator_status_label.setText("")
        self.estimator_status_label.setVisible(False)
        self.calibration_banner_label.setText("")
        self.calibration_banner_label.setVisible(False)
        self.accuracy_footer_label.setText("")
        self.recommendations_calibration_banner_label.setText("")
        self.recommendations_calibration_banner_label.setVisible(False)

        self.lap_table.setRowCount(0)
        self.lap_table.setVisible(False)
        self.plot_container.setVisible(False)

        self._clear_cards()
        self.stability_summary_label.setText(
            "Click Analyse in the Data section to populate results."
        )
        self.recommendations_summary_label.setText(
            "Run Analyse in the Data section, then Generate."
        )
        self.recommendations_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")

        self.btn_analyse.setEnabled(False)
        self.btn_lap_traces.setEnabled(False)
        self.btn_generate_recommendations.setEnabled(False)
        self.btn_generate_decision_frame.setEnabled(False)

        self._update_corner_map_trace()

    def _get_lap_filter_from_selector(self):
        # Decisions batch (Phase 2b): the lap_table selection only scopes the
        # raw-channel plot view below (_on_lap_selected/_update_plots) -- it
        # no longer has any say over which laps get analysed. Analysis
        # always covers every is_valid_for_analysis lap; a file with none
        # (e.g. every lap flagged in/out) falls back to every lap in the
        # file rather than analysing nothing.
        if not self.parsed_data:
            return None
        all_laps = sorted({l["lap_number"] for l in self.parsed_data.get("laps", [])})
        valid_laps = sorted(l["lap_number"] for l in self.parsed_data.get("laps", [])
                             if l.get("is_valid_for_analysis", False))
        return valid_laps if valid_laps else all_laps

    def _get_accuracy_cap_from_selector(self):
        # WP-C: None means "Best available" -- no ceiling. Otherwise the
        # plain int (1-4) crossing the UI/modules boundary like lap_filter.
        text = self.accuracy_cap_combo.currentText()
        if text == "Best available":
            return None
        return int(text.replace("Level ", ""))

    def _on_sideslip_mode_changed(self, display_text):
        # Fresh-session work package, Phase 3a: writes config/parameters.
        # json directly, same read-modify-write + cache-clear + pipeline-
        # invalidate pattern as ui/views/settings_view.py's _on_save_
        # clicked -- see the sideslip_mode_combo construction comment for
        # why this pattern (not an ephemeral UI value) was chosen.
        new_value = self._SIDESLIP_MODE_DISPLAY_TO_VALUE.get(display_text)
        if new_value is None:
            return
        import json
        from modules.stability_analysis import PARAMETERS_PATH, load_parameters
        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            params = json.load(f)
        if params["stability_estimation"].get("sideslip_source", "kinematic") == new_value:
            return
        params["stability_estimation"]["sideslip_source"] = new_value
        # ensure_ascii=False: found during this package's own verification
        # (thesis_notes.md) -- json.dump's default (True) silently escapes
        # every non-ASCII character anywhere else in the file (e.g. this
        # very file's own "x"/"^-1" in an unrelated comment)
        # into \uXXXX sequences on every save. Same latent behaviour exists
        # in ui/views/settings_view.py's _on_save_clicked (not fixed here,
        # out of this phase's permitted files) -- fixed here since this is
        # this package's own new code, not inherited silently.
        with open(PARAMETERS_PATH, "w", encoding="utf-8", newline="") as f:
            json.dump(params, f, indent=2, ensure_ascii=False)
            f.write("\n")
        load_parameters.cache_clear()
        invalidate_all_pipeline_caches()
        self.stability_status_label.setText(
            f"Estimator mode changed to {display_text} -- re-run Analyse to apply."
        )
        self.stability_status_label.setStyleSheet(f"color: {WARN}; font-size: 12px;")

    def _get_setup_data_dict(self):
        # WP-C resolver input: the PERSISTED setup_data for this outing, not
        # any unsaved live form edits -- same convention core.pdf_export
        # already uses. A brand-new unsaved outing (self.outing is None) or
        # an outing with no setup_data yet resolves at Level 1 everywhere,
        # same as today.
        if not self.outing or not self.outing.setup_data:
            return None
        import json
        try:
            return json.loads(self.outing.setup_data)
        except (json.JSONDecodeError, TypeError):
            return None

    def _run_stability_analysis(self):
        if not self.parsed_data:
            return
        # TEMPORARY perf instrumentation (WP6 timing verification) -- one
        # manual timing run, then keep or remove per user decision.
        import time
        self._analyse_click_time = time.perf_counter()
        lap_filter = self._get_lap_filter_from_selector()
        all_lap_nums = sorted({l["lap_number"] for l in self.parsed_data.get("laps", [])})
        print(f"[ANALYSE] all_laps={all_lap_nums}  lap_filter={lap_filter}")
        self.btn_analyse.setEnabled(False)
        self.stability_status_label.setText(
            f"Analysing laps {lap_filter}..."
        )
        self.stability_status_label.setStyleSheet("color: #C0A060; font-size: 12px;")

        # WP-C: resolve accuracy once per Analyse click -- backs both the
        # pipeline-cache identity check below and the thread's own
        # computation, so the cache decision and the computation can never
        # disagree about which values were used.
        from modules.stability_analysis import load_parameters, _resolve_grid_rate
        from modules.accuracy_resolution import resolve_accuracy
        cap = self._get_accuracy_cap_from_selector()
        setup_data = self._get_setup_data_dict()
        params = load_parameters()
        resolved_accuracy = resolve_accuracy(params, setup_data, cap)
        # WP-N2 Step 1b: the config switch (not per-click UI state, but still
        # part of the run's identity -- a config flip + app restart between
        # sessions must not let a stale in-memory entry serve a different
        # estimator's numbers).
        sideslip_source = params["stability_estimation"].get("sideslip_source", "kinematic")
        # 100 Hz time-base work package: the grid rate is a property of
        # THIS FILE's own channels (target_sample_rate_hz/min_sample_
        # rate_hz config aside), not a per-click UI choice -- but a config
        # edit to either of those between an earlier cached run and this
        # one must still invalidate the cache, same reasoning as
        # sideslip_source above. Cheap to recompute (channel header timing
        # only, no Modules-1-5 work).
        grid_rate_hz, _grid_status = _resolve_grid_rate(self.parsed_data["channels"], params)

        # WP6: reuse the last full Modules-1-5 run if it's for this same
        # file AND the same cap/resolved-vehicle-snapshot -- a cap change or
        # a setup_data edit invalidates this exactly like a csv_path change
        # would (full Modules-1-5 recompute), not like a lap-filter-only
        # change (Module-6-only recompute). StabilityAnalysisThread then
        # only re-runs summarise_corners on a genuine hit. FIX 2: the cache
        # is now a module-level, session-persistent singleton keyed by
        # csv_path (_pipeline_cache_get) -- so this hit-check can succeed
        # even for an outing this OutingForm instance never analysed
        # itself, as long as some instance analysed this same file earlier
        # in the session. The cap/resolved_vehicle_snapshot identity check
        # is unchanged; sideslip_source joins it (WP-N2 Step 1b), same
        # pattern as accuracy_cap.
        pipeline_cache = None
        cached_entry = _pipeline_cache_get(self.loaded_csv_path)
        if (cached_entry is not None
                and cached_entry.get("accuracy_cap") == cap
                and cached_entry.get("resolved_vehicle_snapshot") == resolved_accuracy["values"]
                and cached_entry.get("sideslip_source") == sideslip_source
                and cached_entry.get("grid_rate_hz") == grid_rate_hz):
            pipeline_cache = cached_entry
        self.stab_thread = StabilityAnalysisThread(
            self.parsed_data, lap_filter, pipeline_cache=pipeline_cache,
            cap=cap, resolved_accuracy=resolved_accuracy, csv_path=self.loaded_csv_path,
        )
        self.stab_thread.finished.connect(self._on_stability_done)
        self.stab_thread.error.connect(self._on_stability_error)
        self.stab_thread.start()

    def _on_stability_done(self, result):
        self.stability_result = result
        # WP6: cache Modules 1-5's output for this file, self-contained
        # (corners included -- fast path must never re-run corner detection).
        # WP-C: accuracy_cap + resolved_vehicle_snapshot join this identity --
        # see the hit-check in _run_stability_analysis. FIX 2: stored in the
        # module-level singleton (_pipeline_cache_put), not this instance --
        # outliving this OutingForm so a later reopen of this same file, in
        # this same session, from any OutingForm instance, still hits.
        _pipeline_cache_put(self.loaded_csv_path, {
            "csv_path": _norm_path(self.loaded_csv_path),
            "corners": result["corners"],
            "state": result["state"],
            "cs": result["cs"],
            "stab": result["stab"],
            # WP5b(b) phase 1 turn (b): fz (estimate_vertical_loads output)
            # joins the WP6 in-memory cache identity alongside state/cs/stab
            # -- a lap-filter-only re-Analyse must reuse it, not recompute
            # it, same as the other Modules-1-5 outputs it's cached with.
            "fz": result["fz"],
            # PLAN.md STEP 3 Phase 3: ls (estimate_longitudinal_stiffness
            # output) joins the WP6 in-memory cache identity alongside fz --
            # same lap-filter-only-reuse reasoning.
            "ls": result["ls"],
            # WP-A item 3: slip/forces (alpha_*_filt/Fy_*_filt), same
            # cache-reuse reasoning as fz above -- the corner-trace dialog's
            # tyre-curve tab needs them and they must survive a lap-filter-
            # only re-Analyse exactly like every other Modules-1-5 output.
            "slip": result["slip"],
            "forces": result["forces"],
            "accuracy_cap": result["cap"],
            "resolved_vehicle_snapshot": result["resolved_accuracy"]["values"],
            # WP-N2 Step 1b: joins the WP6 identity alongside accuracy_cap --
            # see the hit-check in _run_stability_analysis.
            "sideslip_source": result["sideslip_source"],
            # 100 Hz time-base work package: joins the identity alongside
            # sideslip_source -- see the hit-check in _run_stability_analysis.
            "grid_rate_hz": result["state"]["sample_rate_hz"],
            # Fresh-session work package: NOT new identity fields (sideslip_
            # source alone already differentiates auto modes from each other
            # and from kinematic/ekf_pass_1) -- cached alongside slip/forces
            # above purely so a lap-filter-only re-Analyse doesn't lose the
            # estimator-status line.
            "fit_manifest": result["fit_manifest"],
            "gate_verdict": result["gate_verdict"],
            "fallback_used": result["fallback_used"],
            "fallback_reason": result["fallback_reason"],
        })
        # WP5: build (not yet write) the cache payload for this analysis;
        # _save_outing uses whatever this holds, so a save after a cache-hit
        # render (no fresh Analyse this session) still persists correctly.
        lap_filter = self.stab_thread.lap_filter
        self._analysis_data_json = self._build_analysis_data_json(
            result["summaries"], lap_filter, result["cap"], result["resolved_accuracy"],
            result["sideslip_source"], fit_manifest=result["fit_manifest"],
            gate_verdict=result["gate_verdict"], fallback_used=result["fallback_used"],
            fallback_reason=result["fallback_reason"],
            grid_rate_hz=result["state"]["sample_rate_hz"],
        )
        if self.outing:
            self._persist_analysis_cache()
        # TEMPORARY perf instrumentation (WP6 timing verification).
        import time
        t_render0 = time.perf_counter()
        self._render_stability_summaries(
            result["summaries"], cached=False,
            cap=result["cap"], resolved_accuracy=result["resolved_accuracy"],
            sideslip_source=result["sideslip_source"], fit_manifest=result["fit_manifest"],
            gate_verdict=result["gate_verdict"], fallback_used=result["fallback_used"],
            fallback_reason=result["fallback_reason"],
            grid_rate_hz=result["state"]["sample_rate_hz"],
        )
        t_render1 = time.perf_counter()
        print(f"[PERF] render: {t_render1 - t_render0:.3f}s")
        if hasattr(self, "_analyse_click_time"):
            print(f"[PERF] total (click-to-rendered): {t_render1 - self._analyse_click_time:.3f}s")

    def _format_lap_filter_label(self, lap_filter):
        if not lap_filter:
            return "no laps"
        laps = sorted(lap_filter)
        if len(laps) == 1:
            return f"lap {laps[0]}"
        if laps == list(range(laps[0], laps[-1] + 1)):
            return f"laps {laps[0]}-{laps[-1]}"
        return "laps " + ",".join(str(l) for l in laps)

    def _build_analysis_data_json(self, summaries, lap_filter, cap, resolved_accuracy,
                                   sideslip_source="kinematic", fit_manifest=None,
                                   gate_verdict=None, fallback_used=False, fallback_reason=None,
                                   grid_rate_hz=None):
        import json
        import datetime
        from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
        payload = {
            "csv_path": _norm_path(self.loaded_csv_path),
            "lap_filter": lap_filter,
            "summaries": summaries,
            "generated_at": datetime.datetime.now().isoformat(),
            "schema_version": ANALYSIS_SCHEMA_VERSION,
            # WP-C: the cap this run was generated under, the resolved
            # per-node levels (footer display), the resolved vehicle
            # snapshot (WP5/WP6 cache identity -- level alone is not a
            # sufficient identity token, two different real corner-weight
            # measurements could both resolve to L2 with different numbers),
            # whether the cap actually clipped anything, and any resolver
            # warnings (e.g. the mass/corner-sum consistency check).
            "accuracy_cap": cap,
            "resolved_levels": resolved_accuracy["levels"],
            "resolved_vehicle_snapshot": resolved_accuracy["values"],
            "resolved_clipped": resolved_accuracy["clipped"],
            "resolved_warnings": resolved_accuracy["warnings"],
            # WP-N2 Step 1b: which beta this run used -- schema v5 identity
            # field, see ANALYSIS_SCHEMA_VERSION's own bump comment.
            "sideslip_source": sideslip_source,
            # Fresh-session work package: schema v6. fit_manifest/gate_
            # verdict are None for kinematic/ekf_pass_1 (no fit chain runs
            # for those modes); fallback_used/fallback_reason are always
            # present (False/None outside the two auto modes) -- "a saved
            # outing carries the curve it was analysed under", including
            # the never-silent fallback record when the gate didn't pass.
            "fit_manifest": fit_manifest,
            "gate_verdict": gate_verdict,
            "fallback_used": fallback_used,
            "fallback_reason": fallback_reason,
            # 100 Hz time-base work package: the grid rate this run's
            # Modules 1-5 actually ran at (thesis_notes.md "PHASE 0") --
            # cache identity field, checked in _try_render_cached_analysis.
            # v8 payload shape extension, not a new schema_version bump
            # (still within this same, still-uncommitted v7->8 package).
            "grid_rate_hz": grid_rate_hz,
        }
        return json.dumps(payload)

    def _persist_analysis_cache(self):
        # WP5 write trigger for an EXISTING outing: on analysis completion,
        # independent of the Back-button save (_save_outing has its own
        # symmetry addition for the new-outing / re-save cases).
        if not self.outing or self._analysis_data_json is None:
            return
        from sqlalchemy import update
        session = Session()
        session.execute(
            update(Outing).where(Outing.id == self.outing.id).values(
                analysis_data=self._analysis_data_json
            )
        )
        session.commit()
        session.close()
        self.outing.analysis_data = self._analysis_data_json

    def _try_render_cached_analysis(self):
        # WP5 cache-hit path, called from _on_csv_loaded. Guards: existing
        # outing, parseable JSON, matching schema_version (guard B -- a
        # mismatch, e.g. from the pre-B1 estimator, is treated as no cache
        # at all), matching csv_path (normalised), and (WP-C) a matching
        # accuracy_cap + a freshly-recomputed resolved_vehicle_snapshot --
        # this is what catches "setup_data was edited since this cache was
        # written" without needing a separate content hash: recomputing the
        # resolution is cheap (plain field reads/compares), not a Modules-1-5
        # recompute. Verdicts are never part of the stored payload --
        # _render_stability_summaries always classifies live from current
        # config (guard A).
        # TEMPORARY perf instrumentation (WP6 timing verification).
        import time
        t0 = time.perf_counter()
        if not self.outing or not self.outing.analysis_data:
            return False
        import json
        from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION, load_parameters, _resolve_grid_rate
        from modules.accuracy_resolution import resolve_accuracy
        try:
            cached = json.loads(self.outing.analysis_data)
        except (json.JSONDecodeError, TypeError):
            return False
        if cached.get("schema_version") != ANALYSIS_SCHEMA_VERSION:
            stored_v = cached.get("schema_version")
            self._cached_schema_mismatch = (stored_v, ANALYSIS_SCHEMA_VERSION)
            outdated_msg = (
                f"analysis outdated (v{stored_v} vs v{ANALYSIS_SCHEMA_VERSION}) "
                f"- re-run Analyse first"
            )
            self.stability_status_label.setText(outdated_msg)
            self.stability_status_label.setStyleSheet(f"color: {WARN}; font-size: 12px;")
            self.recommendations_summary_label.setText(outdated_msg)
            self.recommendations_summary_label.setStyleSheet(f"color: {WARN}; font-size: 11px;")
            return False
        if _norm_path(cached.get("csv_path")) != _norm_path(self.loaded_csv_path):
            return False
        cap = self._get_accuracy_cap_from_selector()
        if cached.get("accuracy_cap") != cap:
            return False
        current_resolved = resolve_accuracy(load_parameters(), self._get_setup_data_dict(), cap)
        if current_resolved["values"] != cached.get("resolved_vehicle_snapshot"):
            return False
        # WP-N2 Step 1b: same guard family as accuracy_cap/resolved_vehicle_
        # snapshot above -- a config-switch flip since this payload was
        # written must not silently render numbers from the other estimator.
        current_sideslip_source = load_parameters()["stability_estimation"].get(
            "sideslip_source", "kinematic"
        )
        if cached.get("sideslip_source") != current_sideslip_source:
            return False
        # 100 Hz time-base work package: same reasoning as sideslip_source
        # above -- a target_sample_rate_hz/min_sample_rate_hz config edit
        # (or this file's own channel timing somehow differing) since the
        # cache was written must not silently render a payload computed at
        # a different grid rate.
        current_grid_rate, _grid_status = _resolve_grid_rate(self.parsed_data["channels"], load_parameters())
        if cached.get("grid_rate_hz") != current_grid_rate:
            return False
        summaries = cached.get("summaries")
        if not summaries:
            return False
        lap_filter = cached.get("lap_filter")
        # Decisions batch (Phase 2b): analysis now always covers every
        # is_valid_for_analysis lap (or every lap if none are valid) -- a
        # cached payload written under the old exclude-toggle/single-lap
        # selector can carry a different lap_filter than that policy would
        # produce today (e.g. one lap only, or all laps including in/out).
        # Such a payload no longer reflects current policy and must be
        # treated as a cache miss, not rendered as if it were current.
        if sorted(lap_filter or []) != sorted(self._get_lap_filter_from_selector() or []):
            return False
        self.stability_result = {"summaries": summaries}
        self._analysis_data_json = self.outing.analysis_data
        cached_resolved_accuracy = {
            "levels": cached.get("resolved_levels"),
            "values": cached.get("resolved_vehicle_snapshot"),
            "clipped": cached.get("resolved_clipped"),
            "warnings": cached.get("resolved_warnings") or [],
        }
        self._render_stability_summaries(
            summaries, cached=True, lap_filter=lap_filter,
            cap=cap, resolved_accuracy=cached_resolved_accuracy,
            sideslip_source=current_sideslip_source,
            fit_manifest=cached.get("fit_manifest"),
            gate_verdict=cached.get("gate_verdict"),
            fallback_used=cached.get("fallback_used", False),
            grid_rate_hz=cached.get("grid_rate_hz"),
            fallback_reason=cached.get("fallback_reason"),
        )
        t1 = time.perf_counter()
        print(f"[PERF] db_cache_hit=True  render+sync total: {t1 - t0:.3f}s")
        return True

    # WP-C: short display labels for the resolved-level footer -- not every
    # registry node name is worth spelling out in a compact one-line strip.
    _ACCURACY_FOOTER_LABELS = [
        ("mass", "mass"),
        ("corner_weights", "corners"),
        ("cog_position", "cog"),
        ("yaw_inertia", "Iz"),
        ("steering_ratio", "steer_ratio"),
        ("lateral_force_split", "Fy_split"),
        ("sideslip_angle", "beta"),
        ("speed", "speed"),
        ("yaw_rate", "yaw_rate"),
        ("steering_angle", "steer_ang"),
        ("lateral_acc", "ay"),
        ("wheelbase_m", "wheelbase"),
    ]

    def _format_accuracy_footer(self, levels):
        if not levels:
            return ""
        parts = [
            f"{label} L{levels[node]}"
            for node, label in self._ACCURACY_FOOTER_LABELS
            if node in levels
        ]
        return " | ".join(parts)

    # Fresh-session work package, Phase 3b: which estimator actually
    # produced beta, plus fit/gate status -- separate from the [UNCAL]
    # calibration banner (that answers "are verdict thresholds valid for
    # this estimator", this answers "what estimator, and did the auto
    # chain fall back"). Pure formatting, no state -- testable without Qt.
    _ESTIMATOR_LABELS = {
        "kinematic": "kinematic (production default)",
        "ekf_pass_1": "EKF (frozen pass-1 Dugoff fit)",
        "ekf_auto_dugoff": "EKF auto-fit (Dugoff, this session)",
        "ekf_auto_pacejka": "EKF auto-fit (Pacejka, this session)",
    }

    def _format_estimator_status(self, sideslip_source, fit_manifest, gate_verdict,
                                  fallback_used, fallback_reason):
        # Class attribute accessed via the class, not self -- self is None
        # under the same reuse convention _classify_corner/core/weekend_
        # pdf_export.py's _estimator_status_text already rely on (found by
        # tests/test_auto_fit_wiring.py: self._ESTIMATOR_LABELS raised
        # AttributeError on None, which would have crashed real PDF
        # generation on any fallback render, not just this test).
        label = OutingForm._ESTIMATOR_LABELS.get(sideslip_source, sideslip_source)
        if fallback_used:
            # Deliberately loud and impossible to mistake for a real EKF
            # render: the requested mode is named, but the word KINEMATIC
            # (capitalised, WARN colour) is what actually produced beta.
            text = (
                f"Estimator: KINEMATIC (fallback -- requested {label} could not be trusted: "
                f"{fallback_reason})"
            )
            return text, WARN
        if sideslip_source in ("ekf_auto_dugoff", "ekf_auto_pacejka"):
            fit_status = fit_manifest.get("status") if fit_manifest else "?"
            if gate_verdict:
                gate_text = (
                    f"gate={gate_verdict['verdict']} "
                    f"(score={gate_verdict['health_score']:.4f}, provisional threshold)"
                )
            else:
                gate_text = "gate=?"
            return f"Estimator: {label} -- fit={fit_status}, {gate_text}", TEXT_MUTED
        return f"Estimator: {label}", TEXT_MUTED

    def _render_stability_summaries(self, summaries, cached=False, lap_filter=None,
                                     cap=None, resolved_accuracy=None, sideslip_source=None,
                                     fit_manifest=None, gate_verdict=None,
                                     fallback_used=False, fallback_reason=None,
                                     grid_rate_hz=None):
        # Shared by the live analysis-finished path and the WP5 cache-hit
        # path -- the ONLY place that builds cards/classifies from a
        # summaries list, so a threshold re-derivation always shows up here
        # on next render regardless of which path produced the summaries.
        # Reaching this call means a valid (version-matched) render is
        # about to happen, so any earlier outdated-schema flag no longer
        # applies.
        self._cached_schema_mismatch = None
        #
        # WP-C comparison-run tag: fires only when the cap actually clipped
        # a dynamically-resolved node below its own best-available level for
        # this setup_data -- selecting a non-default cap that happens not to
        # bind on today's data must not read as a comparison run. Thresholds
        # themselves are never re-derived for a capped run (see thesis_notes.
        # md "Accuracy cap is a viewing choice, not a reference-configuration
        # change") -- the caveat that verdicts still come from the reference-
        # configuration thresholds surfaces here, next to the same label a
        # capped run's own numbers are shown under.
        comparison_tag = ""
        if resolved_accuracy and resolved_accuracy.get("clipped"):
            cap_label = f"Level<={cap}" if cap is not None else "Level<=?"
            comparison_tag = (
                f" -- COMPARISON RUN ({cap_label}): verdicts use thresholds "
                f"derived at the reference configuration; not directly "
                f"comparable to a production run."
            )
        if cached:
            self.stability_status_label.setText(
                f"cached ({self._format_lap_filter_label(lap_filter)}) - "
                f"re-run Analyse to refresh{comparison_tag}"
            )
        else:
            self.stability_status_label.setText(
                f"Analysed {len(summaries)} corners. See Stability Analysis "
                f"section.{comparison_tag}"
            )
        self.stability_status_label.setStyleSheet(
            f"color: {WARN if comparison_tag else TEXT_MUTED}; font-size: 12px;"
        )
        self.accuracy_footer_label.setText(
            self._format_accuracy_footer(resolved_accuracy.get("levels")) if resolved_accuracy else ""
        )
        # WP-N2 Step 1b: placeholder wording, pending review.
        if self._sideslip_source_calibrated():
            self.calibration_banner_label.setVisible(False)
            self.calibration_banner_label.setText("")
        else:
            self.calibration_banner_label.setVisible(True)
            self.calibration_banner_label.setText(
                "Sideslip estimator changed; verdict thresholds not re-derived -- "
                "read traces, not verdict colours."
            )
        self._displayed_resolved_vehicle_snapshot = (
            resolved_accuracy.get("values") if resolved_accuracy else None
        )
        # Fresh-session work package, Phase 3b: estimator/fit/gate/fallback
        # status line -- always rendered (even for kinematic/ekf_pass_1,
        # where it just names the estimator) so "which estimator produced
        # this" is never ambiguous regardless of mode.
        if sideslip_source is not None:
            status_text, status_color = self._format_estimator_status(
                sideslip_source, fit_manifest, gate_verdict, fallback_used, fallback_reason
            )
            # 100 Hz time-base work package: the one status line the work
            # order asks for, appended rather than folded into _format_
            # estimator_status's own tested/PDF-shared text -- keeps that
            # function's existing contract untouched. target_sample_rate_hz
            # read fresh (not cached) so a config edit shows immediately.
            if grid_rate_hz is not None:
                from modules.stability_analysis import load_parameters as _load_params_for_grid
                target = _load_params_for_grid()["stability_estimation"]["target_sample_rate_hz"]
                grid_text = (f"{grid_rate_hz:.0f} Hz" if grid_rate_hz >= target
                             else f"{grid_rate_hz:.0f} Hz (channel-limited)")
                status_text = f"{status_text} | time base: {grid_text}"
            self.estimator_status_label.setText(status_text)
            self.estimator_status_label.setStyleSheet(
                f"color: {status_color}; font-size: 11px;"
                + ("font-weight: bold;" if fallback_used else "")
            )
            self.estimator_status_label.setVisible(True)
        else:
            self.estimator_status_label.setVisible(False)
        self.btn_analyse.setEnabled(True)
        self.btn_generate_recommendations.setEnabled(True)
        self.btn_generate_decision_frame.setEnabled(True)
        self.btn_lap_traces.setEnabled(True)
        self._update_corner_map_markers()

        self._clear_cards()
        if not summaries:
            self.stability_summary_label.setText("No corners in selected laps.")
            return

        by_lap = {}
        n_strong = n_moderate = n_normal = 0
        for s in summaries:
            severity, short, long_v, colour = self._classify_corner(s)
            entry = {
                "summary": s,
                "severity": severity,
                "short": short,
                "long": long_v,
                "colour": colour,
            }
            by_lap.setdefault(s["lap_number"], []).append(entry)
            if severity == "strong":
                n_strong += 1
            elif severity == "moderate":
                n_moderate += 1
            else:
                n_normal += 1

        self.stability_summary_label.setText(
            f"{len(summaries)} corners | "
            f"<span style='color:{BAD};'>{n_strong} strong</span> | "
            f"<span style='color:{WARN};'>{n_moderate} moderate</span> | "
            f"<span style='color:{OK};'>{n_normal} normal</span>"
        )
        self.stability_summary_label.setTextFormat(Qt.TextFormat.RichText)
        self.stability_summary_label.setStyleSheet("font-size: 11px;")

        # Columns are keyed by stable_corner_id: the full set across all
        # analysed laps, ascending, so every lap row has the same slots.
        all_stable_ids = sorted({
            s["stable_corner_id"] for s in summaries
            if s.get("stable_corner_id") is not None
        })

        for lap_num in by_lap:
            by_lap[lap_num] = {
                e["summary"]["stable_corner_id"]: e for e in by_lap[lap_num]
            }

        insert_pos = self.cards_host_layout.count() - 1
        for lap_num in sorted(by_lap.keys()):
            lap_row = self._build_lap_row(lap_num, by_lap[lap_num], all_stable_ids)
            self.cards_host_layout.insertWidget(insert_pos, lap_row)
            insert_pos += 1

    def _build_lap_row(self, lap_num, entries_by_id, all_stable_ids):
        # Container with the lap header row, the corner cells row, and a
        # placeholder for the inline details panel that expands below.
        wrapper = QWidget()
        w_layout = QVBoxLayout(wrapper)
        w_layout.setContentsMargins(0, 0, 0, 0)
        w_layout.setSpacing(0)

        # Header + cells row
        row = QWidget()
        row.setStyleSheet(f"background-color: {PANEL}; border: 1px solid {BORDER};")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(8, 6, 8, 6)
        row_layout.setSpacing(6)

        lap_label = QLabel(f"Lap {lap_num}")
        lap_label.setFixedWidth(60)
        lap_label.setStyleSheet(
            f"color: {ACCENT}; font-size: 12px; font-weight: 600; background: transparent; border: none;"
        )
        row_layout.addWidget(lap_label)

        details_host = QWidget()
        details_layout = QVBoxLayout(details_host)
        details_layout.setContentsMargins(0, 4, 0, 0)
        details_layout.setSpacing(0)
        details_host.setVisible(False)

        # Track which cell is currently expanded for this lap
        state = {"active_corner": None, "details_widget": None}

        def show_details(entry):
            while details_layout.count() > 0:
                item = details_layout.takeAt(0)
                w = item.widget()
                if w is not None:
                    w.deleteLater()
            new_details = self._build_corner_details(entry["summary"])
            details_layout.addWidget(new_details)
            details_host.setVisible(True)
            state["active_corner"] = entry["summary"]["stable_corner_id"]
            state["details_widget"] = new_details

        def hide_details():
            while details_layout.count() > 0:
                item = details_layout.takeAt(0)
                w = item.widget()
                if w is not None:
                    w.deleteLater()
            details_host.setVisible(False)
            state["active_corner"] = None
            state["details_widget"] = None

        for stable_id in all_stable_ids:
            entry = entries_by_id.get(stable_id)
            if entry is not None:
                cell = self._build_corner_cell(entry, show_details, hide_details, state)
            else:
                cell = self._build_placeholder_cell()
            row_layout.addWidget(cell)

        row_layout.addStretch()

        w_layout.addWidget(row)
        w_layout.addWidget(details_host)
        return wrapper

    def _build_corner_cell(self, entry, show_details, hide_details, state):
        # Compact horizontal cell for one corner inside its lap row.
        s = entry["summary"]
        stable_id = s["stable_corner_id"]
        colour = entry["colour"]
        short = entry["short"]

        cell = QPushButton()
        cell.setCheckable(False)
        cell.setCursor(Qt.CursorShape.PointingHandCursor)
        cell.setText(f"C{stable_id}\n{short}")
        cell.setStyleSheet(
            f"QPushButton {{"
            f" background-color: {colour}; color: #111; "
            f" border: none; border-radius: 3px; "
            f" padding: 4px 8px; font-size: 10px; font-weight: 600; "
            f" text-align: center; "
            f"}}"
            f"QPushButton:hover {{ background-color: {colour}; opacity: 0.85; }}"
        )
        cell.setMinimumWidth(110)
        cell.setMinimumHeight(38)

        def on_clicked():
            if state["active_corner"] == stable_id:
                hide_details()
            else:
                show_details(entry)

        cell.clicked.connect(on_clicked)
        return cell

    def _build_placeholder_cell(self):
        # Dim, non-interactive cell for a lap with no corner at this stable id.
        cell = QPushButton("-")
        cell.setEnabled(False)
        cell.setStyleSheet(
            f"QPushButton {{"
            f" background-color: {NEUTRAL}; color: {TEXT_DIM}; "
            f" border: none; border-radius: 3px; "
            f" padding: 4px 8px; font-size: 10px; font-weight: 600; "
            f" text-align: center; "
            f"}}"
        )
        cell.setMinimumWidth(110)
        cell.setMinimumHeight(38)
        return cell

    def _build_corner_details(self, summary):
        # Inline details panel: long verdict, the per-phase table, and plot jump.
        severity, _short, long_v, colour = self._classify_corner(summary)

        panel = QWidget()
        panel.setStyleSheet(
            f"background-color: {PANEL_ALT}; border: 1px solid {BORDER}; border-radius: 3px;"
        )
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        # Header line: corner identifier + verdict + plot jump
        header = QWidget()
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(0, 0, 0, 0)
        h_layout.setSpacing(10)

        title = QLabel(
            f"Lap {summary['lap_number']} | C{summary['stable_corner_id']}  "
            f"<span style='color:{TEXT_DIM};'>({summary['speed_class']}, "
            f"{summary['apex_speed']:.0f} km/h, t={summary['apex_time']:.1f}s)</span>"
        )
        title.setStyleSheet(f"color: {TEXT}; font-size: 12px; background: transparent; border: none;")
        title.setTextFormat(Qt.TextFormat.RichText)

        verdict_badge = QLabel(long_v)
        verdict_badge.setStyleSheet(
            f"background-color: {colour}; color: #111; "
            "padding: 3px 10px; border-radius: 3px; font-size: 11px; font-weight: 600;"
        )

        btn_trace = QPushButton("trace")
        btn_trace.setFixedWidth(70)
        btn_trace.setStyleSheet(
            f"background-color: {PANEL}; color: {TEXT_MUTED}; "
            "font-size: 10px; padding: 2px 6px;"
        )
        btn_trace.clicked.connect(lambda _, s=summary: self._open_corner_trace(s))

        h_layout.addWidget(title)
        h_layout.addWidget(verdict_badge)
        h_layout.addStretch()
        h_layout.addWidget(btn_trace)
        layout.addWidget(header)

        # Per-phase table
        phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
        phase_labels = {
            "entry_1_brake": "Brake",
            "entry_2_turnin": "Turn-in",
            "apex_3": "Apex",
            "exit_4": "Exit 4",
            "exit_5": "Exit 5",
        }

        # Fzf/Fzr columns: read-only diagnostic (WP5b(b) phase 1 turn (b)),
        # no severity colour -- nothing here feeds _classify_corner. Shown
        # in kN for table-width readability. fy_f_norm_N/fy_r_norm_N are
        # computed (summarise_corners) but NOT shown here yet -- a further
        # two-column pair does not fit this panel's width cleanly alongside
        # CSf/CSr/Stab; deferred to a later UI pass rather than cramped in.
        # LSf/LSr columns: PLAN.md STEP 3 Phase 3, DISPLAY ONLY -- formatted
        # identically to CSf/CSr (median [p25..p75], same 2-decimal
        # precision, same scale) but rendered with the Fz columns' neutral
        # TEXT_MUTED colour, not _stability_colour -- no CS-style severity
        # thresholds exist for LS_ratio in this package, and none should be
        # implied by colour-coding it as if they did.
        rows_html = (
            f"<table cellpadding='2' style='font-size:10px;'>"
            f"<tr>"
            f"<th align='left' style='color:{TEXT_DIM};'>phase</th>"
            f"<th style='color:{TEXT_DIM};'>n</th>"
            f"<th style='color:{TEXT_DIM};'>valid</th>"
            f"<th style='color:{TEXT_DIM};'>CSf med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>CSr med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>Stab med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>Fzf med kN</th>"
            f"<th style='color:{TEXT_DIM};'>Fzr med kN</th>"
            f"<th style='color:{TEXT_DIM};'>LSf med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>LSr med [p25..p75]</th>"
            f"</tr>"
        )
        for phase in phase_keys:
            p = summary["phases"][phase]
            csf = p["cs_ratio_f"]
            csr = p["cs_ratio_r"]
            sob = p["stability_observed_Nm_per_deg"]
            csf_colour = self._stability_colour("cs", csf["median"], axle="f")
            csr_colour = self._stability_colour("cs", csr["median"], axle="r")
            sob_colour = self._stability_colour("stab", sob["median"])
            csf_str = (f"{csf['median']:.2f} [{csf['p25']:.2f}..{csf['p75']:.2f}]"
                       if csf["n"] > 0 else "-")
            csr_str = (f"{csr['median']:.2f} [{csr['p25']:.2f}..{csr['p75']:.2f}]"
                       if csr["n"] > 0 else "-")
            sob_str = (f"{sob['median']:.0f} [{sob['p25']:.0f}..{sob['p75']:.0f}]"
                       if sob["n"] > 0 else "-")
            fzf = p.get("fz_f_N")
            fzr = p.get("fz_r_N")
            fzf_str = f"{fzf['median']/1000:.1f}" if fzf and fzf["n"] > 0 else "-"
            fzr_str = f"{fzr['median']/1000:.1f}" if fzr and fzr["n"] > 0 else "-"
            lsf = p.get("ls_ratio_f")
            lsr = p.get("ls_ratio_r")
            lsf_str = (f"{lsf['median']:.2f} [{lsf['p25']:.2f}..{lsf['p75']:.2f}]"
                       if lsf and lsf["n"] > 0 else "-")
            lsr_str = (f"{lsr['median']:.2f} [{lsr['p25']:.2f}..{lsr['p75']:.2f}]"
                       if lsr and lsr["n"] > 0 else "-")
            rows_html += (
                f"<tr>"
                f"<td style='color:{ACCENT}; width:80px;'>{phase_labels[phase]}</td>"
                f"<td style='color:{TEXT_MUTED}; width:40px;'>{p['n_samples']}</td>"
                f"<td style='color:{TEXT_MUTED}; width:50px;'>{p['valid_fraction_stab']*100:.0f}%</td>"
                f"<td style='color:{csf_colour}; width:160px;'>{csf_str}</td>"
                f"<td style='color:{csr_colour}; width:160px;'>{csr_str}</td>"
                f"<td style='color:{sob_colour}; width:180px;'>{sob_str}</td>"
                f"<td style='color:{TEXT_MUTED}; width:70px;'>{fzf_str}</td>"
                f"<td style='color:{TEXT_MUTED}; width:70px;'>{fzr_str}</td>"
                f"<td style='color:{TEXT_MUTED}; width:160px;'>{lsf_str}</td>"
                f"<td style='color:{TEXT_MUTED}; width:160px;'>{lsr_str}</td>"
                f"</tr>"
            )
        rows_html += "</table>"

        body = QLabel(rows_html)
        body.setTextFormat(Qt.TextFormat.RichText)
        body.setStyleSheet("background: transparent; border: none;")
        layout.addWidget(body)

        return panel

    def _build_stability_toggle(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("> Stability Analysis")
        btn_toggle.setStyleSheet(
            f"background-color: {PANEL}; color: {TEXT_MUTED}; font-size: 12px; "
            "padding: 8px 14px; text-align: left;"
        )
        btn_toggle.setCheckable(True)
        btn_toggle.setChecked(False)
        layout.addWidget(btn_toggle)

        self.stability_panel = QWidget()
        panel_layout = QVBoxLayout(self.stability_panel)
        panel_layout.setContentsMargins(0, 8, 0, 0)
        panel_layout.setSpacing(8)

        self.stability_summary_label = QLabel(
            "Click Analyse in the Data section to populate results."
        )
        self.stability_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        panel_layout.addWidget(self.stability_summary_label)

        # WP-N2 Step 1b: persistent, does-not-scroll-away caveat -- shown
        # whenever config/parameters.json stability_estimation.sideslip_
        # source doesn't match classification.thresholds_calibrated_for_
        # sideslip_source (_sideslip_source_calibrated below). Complements
        # the per-verdict "[UNCAL]" marker _classify_corner appends -- that
        # marker can scroll out of view on a long corner grid, this banner
        # cannot. Hidden (empty text) when calibrated. Placeholder wording.
        self.calibration_banner_label = QLabel("")
        self.calibration_banner_label.setStyleSheet(f"color: {WARN}; font-size: 11px; font-weight: bold;")
        self.calibration_banner_label.setWordWrap(True)
        self.calibration_banner_label.setVisible(False)
        panel_layout.addWidget(self.calibration_banner_label)

        # WP-C: compact per-node resolved-accuracy footer, always rendered
        # alongside the summary line above (live analysis or cache-hit).
        self.accuracy_footer_label = QLabel("")
        self.accuracy_footer_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        self.accuracy_footer_label.setWordWrap(True)
        panel_layout.addWidget(self.accuracy_footer_label)

        self.cards_scroll = QScrollArea()
        self.cards_scroll.setWidgetResizable(True)
        self.cards_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        self.cards_scroll.setMinimumHeight(400)

        self.cards_host = QWidget()
        self.cards_host_layout = QVBoxLayout(self.cards_host)
        self.cards_host_layout.setContentsMargins(0, 0, 0, 0)
        self.cards_host_layout.setSpacing(6)
        self.cards_host_layout.addStretch()

        self.cards_scroll.setWidget(self.cards_host)
        panel_layout.addWidget(self.cards_scroll)

        self.stability_panel.setVisible(False)
        layout.addWidget(self.stability_panel)

        btn_toggle.toggled.connect(lambda checked, btn=btn_toggle: (
            self.stability_panel.setVisible(checked),
            btn.setText("v Stability Analysis" if checked else "> Stability Analysis")
        ))

        return container

    def _clear_cards(self):
        while self.cards_host_layout.count() > 1:
            item = self.cards_host_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _build_recommendations_toggle(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("> Recommendations")
        btn_toggle.setStyleSheet(
            f"background-color: {PANEL}; color: {TEXT_MUTED}; font-size: 12px; "
            "padding: 8px 14px; text-align: left;"
        )
        btn_toggle.setCheckable(True)
        btn_toggle.setChecked(False)
        layout.addWidget(btn_toggle)

        self.recommendations_panel = QWidget()
        panel_layout = QVBoxLayout(self.recommendations_panel)
        panel_layout.setContentsMargins(0, 8, 0, 0)
        panel_layout.setSpacing(8)

        gen_row = QWidget()
        gen_row_layout = QHBoxLayout(gen_row)
        gen_row_layout.setContentsMargins(0, 0, 0, 0)
        self.btn_generate_recommendations = QPushButton("Generate")
        self.btn_generate_recommendations.setFixedWidth(100)
        self.btn_generate_recommendations.setEnabled(False)
        self.btn_generate_recommendations.clicked.connect(self._generate_recommendations)
        gen_row_layout.addWidget(self.btn_generate_recommendations)
        gen_row_layout.addStretch()
        panel_layout.addWidget(gen_row)

        # WP-N2 Step 1b: same mechanism as calibration_banner_label in the
        # stability panel -- rules key on _classify_corner's verdict text
        # (which already carries the "[UNCAL]" marker), so a top-of-panel
        # caveat here covers the whole recommendations list at a glance.
        self.recommendations_calibration_banner_label = QLabel("")
        self.recommendations_calibration_banner_label.setStyleSheet(
            f"color: {WARN}; font-size: 11px; font-weight: bold;"
        )
        self.recommendations_calibration_banner_label.setWordWrap(True)
        self.recommendations_calibration_banner_label.setVisible(False)
        panel_layout.addWidget(self.recommendations_calibration_banner_label)

        self.recommendations_summary_label = QLabel(
            "Run Analyse in the Data section, then Generate."
        )
        self.recommendations_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        panel_layout.addWidget(self.recommendations_summary_label)

        self.recommendations_host = QWidget()
        self.recommendations_host_layout = QVBoxLayout(self.recommendations_host)
        self.recommendations_host_layout.setContentsMargins(0, 0, 0, 0)
        self.recommendations_host_layout.setSpacing(6)
        self.recommendations_host_layout.addStretch()
        panel_layout.addWidget(self.recommendations_host)

        self.recommendations_panel.setVisible(False)
        layout.addWidget(self.recommendations_panel)

        btn_toggle.toggled.connect(lambda checked, btn=btn_toggle: (
            self.recommendations_panel.setVisible(checked),
            btn.setText("v Recommendations" if checked else "> Recommendations")
        ))

        return container

    def _clear_recommendation_rows(self):
        while self.recommendations_host_layout.count() > 1:
            item = self.recommendations_host_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _generate_recommendations(self):
        # Synchronous: aggregation + rule matching over ~15 corners and a
        # handful of rules is fast enough not to need a worker thread.
        # WP-N2 Step 1b: set regardless of the early-return paths below --
        # rules key on _classify_corner's verdict text (which already
        # carries the "[UNCAL]" marker), so this banner must reflect the
        # live config every time this method runs, not only on a full
        # regeneration. Placeholder wording, pending review.
        if self._sideslip_source_calibrated():
            self.recommendations_calibration_banner_label.setVisible(False)
            self.recommendations_calibration_banner_label.setText("")
        else:
            self.recommendations_calibration_banner_label.setVisible(True)
            self.recommendations_calibration_banner_label.setText(
                "Sideslip estimator changed; recommendation thresholds not "
                "re-derived -- treat as indicative only."
            )
        if not self.stability_result:
            # Fix turn: this must never render as silent emptiness. The
            # button is normally disabled in this state (only enabled by
            # _render_stability_summaries), but a stale schema-mismatch
            # flag from a rejected cache-hit is the one case that can
            # still reach here, so it gets its own explicit message; any
            # other reason falls back to the panel's own default prompt.
            if self._cached_schema_mismatch:
                stored_v, current_v = self._cached_schema_mismatch
                self.recommendations_summary_label.setText(
                    f"analysis outdated (v{stored_v} vs v{current_v}) - re-run Analyse first"
                )
                self.recommendations_summary_label.setStyleSheet(f"color: {WARN}; font-size: 11px;")
            else:
                self.recommendations_summary_label.setText(
                    "Run Analyse in the Data section, then Generate."
                )
                self.recommendations_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
            return
        import json
        from modules.recommendation import generate_recommendations, load_recommendations_config

        summaries = self.stability_result["summaries"]
        feedback_data = json.loads(self._collect_feedback_data())
        setup_data = json.loads(self._collect_setup_data())
        config = load_recommendations_config()
        driving_level = self._resolve_current_driving_level()

        results = generate_recommendations(
            summaries, self._classify_corner, feedback_data, setup_data, config,
            outing=self.outing, driving_level=driving_level,
        )

        rule_status = {r["id"]: r.get("status", "seed") for r in config["rules"]}
        analysed_lap_count = len({s["lap_number"] for s in summaries})

        self._clear_recommendation_rows()

        if not results:
            self.recommendations_summary_label.setText(
                "No recommendations at current thresholds."
            )
            return

        self.recommendations_summary_label.setText(f"{len(results)} recommendation(s).")

        insert_pos = self.recommendations_host_layout.count() - 1
        for r in results:
            row = self._build_recommendation_row(r, rule_status, analysed_lap_count, driving_level)
            self.recommendations_host_layout.insertWidget(insert_pos, row)
            insert_pos += 1

    def _resolve_current_driving_level(self):
        # PART A: resolved from the currently selected driver in the combo,
        # not self.outing.driver_id -- a new, not-yet-saved outing has no
        # outing.driver_id yet even though a driver may already be selected
        # here. Same read _save_outing itself uses for driver_id (below).
        driver_id = self.driver_combo.currentData()
        if driver_id is None:
            return None
        session = Session()
        driver = session.get(Driver, driver_id)
        level = driver.driving_level if driver else None
        session.close()
        return level

    def _build_recommendation_row(self, r, rule_status, analysed_lap_count, driving_level=None):
        card = QWidget()
        card.setStyleSheet(f"background-color: {PANEL}; border: 1px solid {BORDER};")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(10, 8, 10, 8)
        card_layout.setSpacing(6)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)

        # WP2b-2: a package/axle-symmetric-pair suggestion has no single
        # parameter/direction -- badge falls back to listing every action.
        # FIX 1: a synthetic urgent row (action_class "urgent_gap") has no
        # setup-parameter action at all -- the badge names the corner and
        # verdict instead of a lever, since there isn't one to name.
        if r["action_class"] == "urgent_gap":
            c0 = r["corners"][0] if r["corners"] else None
            badge_text = f"C{c0['stable_corner_id']}: {c0['short_verdict']}" if c0 else "engineer attention"
        elif r["parameter"] is not None:
            badge_text = f"{r['parameter']} | {r['direction']}"
        else:
            badge_text = " + ".join(
                f"{a['parameter']} -> {a['target']}" if "target" in a
                else f"{a['parameter']} {a['direction']}"
                for a in r["actions"]
            )
        badge = QLabel(badge_text)
        badge.setStyleSheet(
            f"background-color: {ACCENT}; color: #111; font-size: 11px; "
            "font-weight: 600; padding: 3px 8px; border-radius: 3px;"
        )
        header_layout.addWidget(badge)

        # FIX 1: synthetic rows carry no engine score (nothing was ranked
        # against anything) -- the URGENT tag below is the signal instead.
        if r["score"] is not None:
            score_label = QLabel(f"score {r['score']:.2f}")
            score_label.setStyleSheet(f"color: {TEXT}; font-size: 11px;")
            header_layout.addWidget(score_label)

        # FIX 1 (undrivable-feedback tier, design ruling 2026-07-28): a
        # raw|feedback|>=4 corner must never render as silent emptiness --
        # this tag is the one thing every one of the tier's three outcomes
        # (pierced bucket, synthetic gap row, synthetic contradiction row)
        # has in common, so it is checked once, ahead of the normal
        # ADVISORY/SELECTED branch below (an urgent row can still also be
        # "recommended"/"selected" -- the tag is additive, not exclusive).
        if r.get("urgent"):
            urgent_label = QLabel(r.get("urgent_tag") or "URGENT")
            urgent_label.setStyleSheet(
                f"background-color: {BAD}; color: #fff; font-size: 10px; "
                "font-weight: 700; padding: 3px 8px; border-radius: 3px;"
            )
            header_layout.addWidget(urgent_label)

        trigger_label = QLabel(" / ".join(r["trigger_source"]))
        trigger_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
        header_layout.addWidget(trigger_label)

        if r["cell_ids"]:
            cell_label = QLabel(" / ".join(r["cell_ids"]))
            cell_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            header_layout.addWidget(cell_label)

        # WP2b-2 amendment 7: advisory matches are an observation, never a
        # mandate -- mild understeer is this car's deliberate stable
        # baseline. Visually distinct from a budget-eligible recommendation.
        if r["action_class"] == "advisory":
            class_label = QLabel("ADVISORY")
            class_label.setStyleSheet(
                f"color: {TEXT_MUTED}; font-size: 10px; font-weight: 600;"
            )
            header_layout.addWidget(class_label)
        elif r["selected"]:
            selected_label = QLabel("SELECTED (within budget)")
            selected_label.setStyleSheet(
                f"color: {ACCENT}; font-size: 10px; font-weight: 600;"
            )
            header_layout.addWidget(selected_label)

        # WP2b-2 amendment 6: feasibility against the outing's current
        # setup sheet, checked at generate time -- never silently applied
        # past a registry limit, never silently hidden when unchecked.
        if r["limit_status"] == "at_limit":
            limit_label = QLabel(f"AT LIMIT ({', '.join(r['at_limit_parameters'])})")
            limit_label.setStyleSheet(f"color: {BAD}; font-size: 10px; font-weight: 600;")
            header_layout.addWidget(limit_label)
        elif r["limit_status"] == "unchecked":
            limit_label = QLabel("limit not checked (setup sheet unfilled)")
            limit_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
            header_layout.addWidget(limit_label)

        # Mandatory per WP2: a placeholder rule must never look like
        # engineering truth. Shown whenever ANY contributing rule is still
        # status:"seed" (retired in WP2b-2 -- dead code path kept in case a
        # future experimental rule ever ships as "seed" again).
        has_seed = any(
            rule_status.get(rule_id, "seed") == "seed" for rule_id in r["rules_fired"]
        )
        if has_seed:
            seed_label = QLabel("unvalidated rule")
            seed_label.setStyleSheet(
                f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;"
            )
            header_layout.addWidget(seed_label)

        header_layout.addStretch()
        card_layout.addWidget(header)

        chips_row = QWidget()
        chips_layout = QHBoxLayout(chips_row)
        chips_layout.setContentsMargins(0, 0, 0, 0)
        chips_layout.setSpacing(6)
        for c in r["corners"]:
            text = f"C{c['stable_corner_id']}"
            if c["n_laps"] < analysed_lap_count:
                text += f" ({c['n_laps']} lap{'s' if c['n_laps'] != 1 else ''})"
            is_worst = c.get("worst_corner", False)
            if is_worst:
                text = f"! {text}"
            chip = QLabel(text)
            if is_worst:
                # Driver flagged this corner "worst" -- the score boost
                # (worst_corner_multiplier) must be visible, not silent.
                chip.setStyleSheet(
                    f"background-color: {PANEL_ALT}; color: {TEXT}; font-size: 10px; "
                    f"font-weight: 600; padding: 2px 6px; border-radius: 3px; "
                    f"border: 1px solid {ACCENT};"
                )
            else:
                chip.setStyleSheet(
                    f"background-color: {PANEL_ALT}; color: {TEXT_MUTED}; font-size: 10px; "
                    "padding: 2px 6px; border-radius: 3px;"
                )
            chips_layout.addWidget(chip)
        chips_layout.addStretch()
        card_layout.addWidget(chips_row)

        if r["conflicts"]:
            conflict_ids = ", ".join(f"C{c['stable_corner_id']}" for c in r["conflicts"])
            # PART A: level context is one value for the whole outing (one
            # driver), so it's appended once to the label rather than
            # repeated per corner or threaded back through the engine's
            # per-corner conflicts list.
            level_note = f" (driver level {driving_level}/10)" if driving_level is not None else ""
            conflict_label = QLabel(f"driver and data disagree at {conflict_ids}{level_note}")
            conflict_label.setStyleSheet(f"color: {WARN}; font-size: 10px;")
            card_layout.addWidget(conflict_label)

        # WP2b-2: a DIFFERENT conflict from the one above -- two rules
        # recommend opposite directions for the same registry parameter.
        # Never netted/averaged; BAD (not WARN) since this is a harder stop
        # than a driver/data disagreement on a single rule.
        if r["parameter_conflict"]:
            pc_label = QLabel(
                f"CONFLICT with another recommendation on: {', '.join(r['conflict_parameters'])}"
            )
            pc_label.setStyleSheet(f"color: {BAD}; font-size: 10px; font-weight: 600;")
            card_layout.addWidget(pc_label)

        if r["observation_lines"]:
            obs_host = QWidget()
            obs_layout = QVBoxLayout(obs_host)
            obs_layout.setContentsMargins(0, 0, 0, 0)
            obs_layout.setSpacing(2)
            for line in r["observation_lines"]:
                obs_label = QLabel(line)
                obs_label.setWordWrap(True)
                obs_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
                obs_layout.addWidget(obs_label)
            card_layout.addWidget(obs_host)

        # Task 4 (second-choice visibility): a held escalation is context,
        # not a recommendation -- it never fires, this is display only.
        for note in r["escalation_notes"]:
            esc_label = QLabel(note)
            esc_label.setWordWrap(True)
            esc_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
            card_layout.addWidget(esc_label)

        btn_expand = QPushButton("> rationale")
        btn_expand.setCheckable(True)
        btn_expand.setChecked(False)
        btn_expand.setStyleSheet(
            f"background-color: transparent; color: {TEXT_MUTED}; font-size: 10px; "
            "text-align: left; border: none; padding: 2px 0;"
        )
        card_layout.addWidget(btn_expand)

        rationale_host = QWidget()
        rationale_layout = QVBoxLayout(rationale_host)
        rationale_layout.setContentsMargins(12, 2, 0, 0)
        rationale_layout.setSpacing(2)
        for rat in r["rationale"]:
            # Fix turn: no [cell_id]/[rule_id] prefix here -- the cell_id(s)
            # already show as their own header badge (chips_layout above).
            line = QLabel(rat["rationale"])
            line.setWordWrap(True)
            line.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
            rationale_layout.addWidget(line)
        rationale_host.setVisible(False)
        card_layout.addWidget(rationale_host)

        def toggle_rationale(checked):
            rationale_host.setVisible(checked)
            btn_expand.setText("v rationale" if checked else "> rationale")
        btn_expand.toggled.connect(toggle_rationale)

        return card

    def _build_decision_frame_toggle(self):
        # Decision-matrix frame, Stage 1 (2026-09-02): the three-layer
        # evidence/candidate/scoring frame (modules/decision_frame.py),
        # additive and parallel to the Recommendations section above --
        # that section's 39-rule engine is untouched by this one. Same
        # collapsible-toggle/card visual language as _build_recommendations_
        # toggle, deliberately, so the two sections read as one family.
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("> Decision Frame (preview)")
        btn_toggle.setStyleSheet(
            f"background-color: {PANEL}; color: {TEXT_MUTED}; font-size: 12px; "
            "padding: 8px 14px; text-align: left;"
        )
        btn_toggle.setCheckable(True)
        btn_toggle.setChecked(False)
        layout.addWidget(btn_toggle)

        self.decision_frame_panel = QWidget()
        panel_layout = QVBoxLayout(self.decision_frame_panel)
        panel_layout.setContentsMargins(0, 8, 0, 0)
        panel_layout.setSpacing(8)

        gen_row = QWidget()
        gen_row_layout = QHBoxLayout(gen_row)
        gen_row_layout.setContentsMargins(0, 0, 0, 0)
        self.btn_generate_decision_frame = QPushButton("Generate")
        self.btn_generate_decision_frame.setFixedWidth(100)
        self.btn_generate_decision_frame.setEnabled(False)
        self.btn_generate_decision_frame.clicked.connect(self._generate_decision_frame)
        gen_row_layout.addWidget(self.btn_generate_decision_frame)
        gen_row_layout.addStretch()
        panel_layout.addWidget(gen_row)

        note_label = QLabel(
            "Preview: Stage 1 scope only (exit oversteer, LS-disambiguated, plus the "
            "brake-balance plausibility check). Everything else falls back to the "
            "Recommendations section above, which this does not replace."
        )
        note_label.setWordWrap(True)
        note_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
        panel_layout.addWidget(note_label)

        self.decision_frame_summary_label = QLabel(
            "Run Analyse in the Data section, then Generate."
        )
        self.decision_frame_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        panel_layout.addWidget(self.decision_frame_summary_label)

        self.decision_frame_host = QWidget()
        self.decision_frame_host_layout = QVBoxLayout(self.decision_frame_host)
        self.decision_frame_host_layout.setContentsMargins(0, 0, 0, 0)
        self.decision_frame_host_layout.setSpacing(6)
        self.decision_frame_host_layout.addStretch()
        panel_layout.addWidget(self.decision_frame_host)

        self.decision_frame_panel.setVisible(False)
        layout.addWidget(self.decision_frame_panel)

        btn_toggle.toggled.connect(lambda checked, btn=btn_toggle: (
            self.decision_frame_panel.setVisible(checked),
            btn.setText("v Decision Frame (preview)" if checked else "> Decision Frame (preview)")
        ))

        return container

    def _clear_decision_frame_rows(self):
        while self.decision_frame_host_layout.count() > 1:
            item = self.decision_frame_host_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _generate_decision_frame(self):
        # Synchronous, same rationale as _generate_recommendations: a
        # handful of corners through three light layers is fast enough not
        # to need a worker thread.
        if not self.stability_result:
            self.decision_frame_summary_label.setText(
                "Run Analyse in the Data section, then Generate."
            )
            self.decision_frame_summary_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
            return
        import json
        from modules.decision_frame import (
            build_evidence, aggregate_ls_by_corner, load_decision_frame_config,
            generate_candidates, generate_shortlist,
        )
        from modules.recommendation import load_setup_parameters_registry

        summaries = self.stability_result["summaries"]
        config = load_decision_frame_config()
        registry = load_setup_parameters_registry()
        setup_data = json.loads(self._collect_setup_data())

        ls_stats = aggregate_ls_by_corner(summaries)
        evidence = build_evidence(summaries, ls_stats, config, self._classify_corner)
        candidates = generate_candidates(evidence, registry, config)
        shortlist = generate_shortlist(candidates, evidence, setup_data, config)

        self._clear_decision_frame_rows()

        if not shortlist:
            self.decision_frame_summary_label.setText(
                f"{len(evidence)} evidence item(s), no candidates at this stage's scope "
                "(Stage 1 only covers exit oversteer and brake-balance plausibility)."
            )
            return

        self.decision_frame_summary_label.setText(
            f"{len(evidence)} evidence item(s), {len(shortlist)} candidate(s)."
        )

        insert_pos = self.decision_frame_host_layout.count() - 1
        for c in shortlist:
            row = self._build_decision_frame_row(c)
            self.decision_frame_host_layout.insertWidget(insert_pos, row)
            insert_pos += 1

    def _build_decision_frame_row(self, c):
        # Same visual language as _build_recommendation_row: PANEL/BORDER
        # card, ACCENT action badge, muted chips, expandable detail via a
        # checkable "> ..." button -- deliberately, so the two sections
        # read as one family (see _build_decision_frame_toggle's comment).
        card = QWidget()
        card.setStyleSheet(f"background-color: {PANEL}; border: 1px solid {BORDER};")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(10, 8, 10, 8)
        card_layout.setSpacing(6)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)

        if c["actions"]:
            badge_text = " + ".join(
                f"{a['parameter']} -> {a['target']}" if "target" in a
                else f"{a['parameter']} {a['direction']}"
                for a in c["actions"]
            )
        else:
            badge_text = f"C{c['corner']}: engineer attention (no routed action)"
        badge = QLabel(badge_text)
        badge.setStyleSheet(
            f"background-color: {ACCENT}; color: #111; font-size: 11px; "
            "font-weight: 600; padding: 3px 8px; border-radius: 3px;"
        )
        header_layout.addWidget(badge)

        score_label = QLabel(f"score {c['score']:.2f}")
        score_label.setStyleSheet(f"color: {TEXT}; font-size: 11px;")
        header_layout.addWidget(score_label)

        # 'proposed'-grade candidates are advisory-capped, same policy as
        # modules.recommendation._match_is_recommended's provenance cap --
        # visually distinct from a matrix-backed candidate, same ADVISORY
        # wording the Recommendations section already uses for the same
        # concept.
        if c["grade"] == "proposed":
            grade_label = QLabel("ADVISORY (proposed, not matrix-reviewed)")
            grade_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px; font-weight: 600;")
        else:
            grade_label = QLabel("derived-from-matrix")
            grade_label.setStyleSheet(f"color: {ACCENT}; font-size: 10px; font-weight: 600;")
        header_layout.addWidget(grade_label)

        if c.get("cell_id"):
            cell_label = QLabel(c["cell_id"])
            cell_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            header_layout.addWidget(cell_label)

        header_layout.addStretch()
        card_layout.addWidget(header)

        evidence_line = QLabel(
            f"C{c['corner']} {c['phase']} -- {len(c['evidence_refs'])} evidence item(s), "
            f"effort={c['effort_class']}, effect={c['effect_class']}"
        )
        evidence_line.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
        card_layout.addWidget(evidence_line)

        btn_expand = QPushButton("> reasoning")
        btn_expand.setCheckable(True)
        btn_expand.setChecked(False)
        btn_expand.setStyleSheet(
            f"background-color: transparent; color: {TEXT_MUTED}; font-size: 10px; "
            "text-align: left; border: none; padding: 2px 0;"
        )
        card_layout.addWidget(btn_expand)

        detail_host = QWidget()
        detail_layout = QVBoxLayout(detail_host)
        detail_layout.setContentsMargins(12, 2, 0, 0)
        detail_layout.setSpacing(2)

        rationale_line = QLabel(c["rationale"])
        rationale_line.setWordWrap(True)
        rationale_line.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
        detail_layout.addWidget(rationale_line)

        components_text = ", ".join(f"{k}={v:+.3f}" for k, v in c["score_components"].items())
        components_line = QLabel(f"score breakdown: {components_text}")
        components_line.setWordWrap(True)
        components_line.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        detail_layout.addWidget(components_line)

        for note in c.get("score_interaction_notes", []):
            note_line = QLabel(f"interaction: {note}")
            note_line.setWordWrap(True)
            note_line.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            detail_layout.addWidget(note_line)

        for flag in c.get("score_flags", []):
            flag_line = QLabel(flag)
            flag_line.setWordWrap(True)
            flag_line.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
            detail_layout.addWidget(flag_line)

        for e in c["evidence_refs"]:
            source_line = QLabel(f"evidence ({e['type']}): {e['source']}")
            source_line.setWordWrap(True)
            source_line.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            detail_layout.addWidget(source_line)

        detail_host.setVisible(False)
        card_layout.addWidget(detail_host)

        def toggle_detail(checked):
            detail_host.setVisible(checked)
            btn_expand.setText("v reasoning" if checked else "> reasoning")
        btn_expand.toggled.connect(toggle_detail)

        return card

    def _open_corner_trace(self, summary):
        # PART C: reused, non-modal per-corner trace window (ui/views/
        # corner_trace_dialog.py) -- created lazily, replotted in place on
        # every click rather than spawned per corner.
        from ui.views.corner_trace_dialog import CornerTraceDialog
        if self._corner_trace_dialog is None:
            self._corner_trace_dialog = CornerTraceDialog(self)
        self._corner_trace_dialog.show_corner(
            summary, self.stability_result or {}, self.parsed_data or {}
        )

    def _open_lap_trace(self):
        # Lap-trace-view work package: reused, non-modal full-lap trace
        # window -- same lazy-create/replot-in-place convention as
        # _open_corner_trace, a separate instance/window (LapTraceDialog),
        # not the corner dialog reused in a different mode. on_corner_click
        # is bound to _open_corner_trace directly -- a band click reuses
        # the exact same open path a corner card's own "trace" button does,
        # nothing duplicated.
        if not self.stability_result:
            return
        from ui.views.corner_trace_dialog import LapTraceDialog

        lap_number = self._selected_lap_value
        if not isinstance(lap_number, int):
            valid_laps = sorted(
                l["lap_number"] for l in (self.parsed_data or {}).get("laps", [])
                if l.get("is_valid_for_analysis")
            )
            if not valid_laps:
                return
            lap_number = valid_laps[0]

        if self._lap_trace_dialog is None:
            self._lap_trace_dialog = LapTraceDialog(self, on_corner_click=self._open_corner_trace)
        self._lap_trace_dialog.show_lap(
            lap_number, self.stability_result or {}, self.parsed_data or {}, self._classify_corner,
        )

    def _on_stability_error(self, msg):
        self.stability_status_label.setText(f"Analysis failed: {msg}")
        self.stability_status_label.setStyleSheet("color: #c0392b; font-size: 12px;")
        self.btn_analyse.setEnabled(True)

    def _populate_lap_table(self, laps):
        from PyQt6.QtGui import QColor
        self.lap_table.setRowCount(0)

        all_row = self.lap_table.rowCount()
        self.lap_table.insertRow(all_row)
        self.lap_table.setRowHeight(all_row, 28)
        all_item = QTableWidgetItem("All")
        all_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        all_item.setForeground(QColor("#888"))
        all_item.setData(Qt.ItemDataRole.UserRole, "all")
        self.lap_table.setItem(all_row, 0, all_item)
        self.lap_table.setItem(all_row, 1, QTableWidgetItem(""))
        full_item = QTableWidgetItem("Full Outing")
        full_item.setForeground(QColor("#555"))
        self.lap_table.setItem(all_row, 2, full_item)

        for lap in laps:
            row = self.lap_table.rowCount()
            self.lap_table.insertRow(row)
            self.lap_table.setRowHeight(row, 28)

            is_outlap = lap.get("is_outlap", False)
            is_inlap = lap.get("is_inlap", False)
            display_text = "Out" if is_outlap else ("In" if is_inlap else str(lap["lap_number"]))
            lap_item = QTableWidgetItem(display_text)
            lap_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            lap_item.setData(Qt.ItemDataRole.UserRole, lap["lap_number"])

            # Show only as much precision as is actually held: the precise
            # channel value (hundredths) when the parser adopted it, the
            # computed 0.2s-grid value (tenths) otherwise -- never claim
            # more precision than the underlying number has.
            precise = lap.get("lap_time_precise")
            use_precise = precise is not None
            display_time = precise if use_precise else lap["lap_time"]
            mins = int(display_time // 60)
            secs = display_time % 60
            time_str = f"{mins}:{secs:05.2f}" if use_precise else f"{mins}:{secs:04.1f}"
            time_item = QTableWidgetItem(time_str)
            time_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            if is_outlap:
                badge_text = "OUT LAP"
            elif is_inlap:
                badge_text = "IN LAP"
            elif lap["is_fastest"]:
                badge_text = "FASTEST"
            else:
                badge_text = ""
            badge_item = QTableWidgetItem(badge_text)
            badge_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            if lap["is_fastest"] and not is_outlap and not is_inlap:
                for item in [lap_item, time_item, badge_item]:
                    item.setForeground(QColor("#C0A060"))
            elif is_outlap or is_inlap:
                for item in [lap_item, time_item, badge_item]:
                    item.setForeground(QColor("#555555"))

            self.lap_table.setItem(row, 0, lap_item)
            self.lap_table.setItem(row, 1, time_item)
            self.lap_table.setItem(row, 2, badge_item)

        if self.lap_table.rowCount() > 0:
            header_h = self.lap_table.horizontalHeader().height()
            total_row_h = sum(
                self.lap_table.rowHeight(i)
                for i in range(self.lap_table.rowCount())
            )
            self.lap_table.setFixedHeight(header_h + total_row_h + 4)
            self.lap_table.setVisible(True)

        for row in range(self.lap_table.rowCount()):
            badge = self.lap_table.item(row, 2)
            if badge and badge.text() == "FASTEST":
                self.lap_table.selectRow(row)
                fastest_num = self.lap_table.item(row, 0).data(
                    Qt.ItemDataRole.UserRole
                )
                self._selected_lap_value = fastest_num
                self._update_plots(fastest_num)
                break

        if self.lap_table.rowCount() > 0:
            header_h = self.lap_table.horizontalHeader().height()
            total_row_h = sum(
                self.lap_table.rowHeight(i)
                for i in range(self.lap_table.rowCount())
            )
            self.lap_table.setFixedHeight(header_h + total_row_h + 4)
            self.lap_table.setVisible(True)

    def _build_setup_section(self, prefix="setup"):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        if prefix == "setup":
            layout.addWidget(self._section_label("Car Setup"))
            self.setup_inputs = {}
            self.setup_inputs["car"] = {}
            self._active_inputs = self.setup_inputs
        else:
            layout.addWidget(self._section_label("Setdown"))
            self.setdown_inputs = {}
            self.setdown_inputs["car"] = {}
            self._active_inputs = self.setdown_inputs

        params = get_setup_parameters()

        columns = QWidget()
        columns_layout = QHBoxLayout(columns)
        columns_layout.setContentsMargins(0, 0, 0, 0)
        columns_layout.setSpacing(16)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(16)
        left_layout.addWidget(self._build_corner_block("FL", params.get("front_left", {})))
        left_layout.addWidget(self._build_corner_block("RL", params.get("rear_left", {})))

        center = self._build_car_center(params.get("car", {}))

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(16)
        right_layout.addWidget(self._build_corner_block("FR", params.get("front_right", {})))
        right_layout.addWidget(self._build_corner_block("RR", params.get("rear_right", {})))

        columns_layout.addWidget(left)
        columns_layout.addWidget(center)
        columns_layout.addWidget(right)

        layout.addWidget(columns)

        notes_label = QLabel("Setup Notes")
        notes_label.setStyleSheet("color: #888; font-size: 11px; margin-top: 8px;")
        notes_widget = QTextEdit()
        notes_widget.setMinimumHeight(100)
        notes_widget.setPlaceholderText("Kinematic info, special configurations, general setup notes...")
        notes_widget.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a1a;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                padding: 8px;
                color: #e0e0e0;
                font-size: 12px;
            }
        """)
        self._active_inputs["car"]["notes"] = notes_widget
        layout.addWidget(notes_label)
        layout.addWidget(notes_widget)

        if prefix == "setup":
            btn_print = QPushButton("Print Setup")
            btn_print.setFixedWidth(140)
            btn_print.clicked.connect(lambda: self._print_sheet("setup"))
            layout.addWidget(btn_print)
        else:
            btn_print = QPushButton("Print Setdown")
            btn_print.setFixedWidth(140)
            btn_print.clicked.connect(lambda: self._print_sheet("setdown"))
            layout.addWidget(btn_print)

        return section

    def _build_setdown_toggle(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("> Add Setdown")
        btn_toggle.setStyleSheet("background-color: #1a1a1a; color: #888; font-size: 12px; padding: 8px 14px; text-align: left;")
        btn_toggle.setCheckable(True)
        btn_toggle.setChecked(False)
        layout.addWidget(btn_toggle)

        self.setdown_widget = self._build_setup_section("setdown")
        self.setdown_widget.setVisible(False)
        layout.addWidget(self.setdown_widget)

        btn_toggle.toggled.connect(lambda checked, btn=btn_toggle: (
            self.setdown_widget.setVisible(checked),
            btn.setText("v Add Setdown" if checked else "> Add Setdown"),
            self._prefill_setdown() if checked else None
        ))

        return container

    def _build_corner_block(self, corner_label, params):
        corner_key = {"FL": "front_left", "FR": "front_right",
                      "RL": "rear_left", "RR": "rear_right"}[corner_label]

        if corner_key not in self._active_inputs:
            self._active_inputs[corner_key] = {}

        group = QGroupBox(corner_label)
        group.setStyleSheet("""
            QGroupBox {
                color: #C0A060;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
        """)

        layout = QVBoxLayout(group)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 12, 8, 8)

        always_visible = ["toe", "camber", "ride_height_fia", "ride_height_aero", "arb", "springs"]
        advanced_fields = ["packer", "preload", "total_travel", "free_length", "static_droop", "gap_on_gnd"]

        labels = {
            "toe": "Toe (mm)", "camber": "Camber (deg)",
            "ride_height_fia": "Ride Ht. FIA (mm)", "ride_height_aero": "Ride Ht. Aero (mm)",
            "arb": "ARB (pos.)", "springs": "Springs (N/mm)",
            "bump_ls": "Bump LS", "bump_hs": "Bump HS",
            "blowoff": "Blowoff",
            "rebound_ls": "Rebound LS", "rebound_hs": "Rebound HS",
            "packer": "Packer (mm)", "preload": "Preload (mm)",
            "total_travel": "Total Travel (mm)", "free_length": "Free Length (mm)",
            "static_droop": "Static Droop (mm)", "gap_on_gnd": "Gap on GND (mm)"
        }

        for param in always_visible:
            widget = NoScrollSpinBox()
            widget.setRange(-9999, 9999)
            widget.setDecimals(2)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs[corner_key][param] = widget
            layout.addWidget(self._setup_row(labels[param], widget))

        damper_label = QLabel("Damper")
        damper_label.setStyleSheet("color: #555; font-size: 10px; font-weight: 500; margin-top: 6px;")
        layout.addWidget(damper_label)

        bump_row = QWidget()
        bump_layout = QHBoxLayout(bump_row)
        bump_layout.setContentsMargins(0, 0, 0, 0)
        bump_layout.setSpacing(8)
        for param, label_text in [("bump_ls", "Bump LS"), ("bump_hs", "Bump HS")]:
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2)
            lbl = QLabel(label_text)
            lbl.setStyleSheet("color: #555; font-size: 10px;")
            widget = QLineEdit()
            self._active_inputs[corner_key][param] = widget
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(widget)
            bump_layout.addWidget(cell)
        layout.addWidget(bump_row)

        blowoff_cell = QWidget()
        blowoff_cell_layout = QVBoxLayout(blowoff_cell)
        blowoff_cell_layout.setContentsMargins(0, 0, 0, 0)
        blowoff_cell_layout.setSpacing(2)
        blowoff_lbl = QLabel("Blowoff")
        blowoff_lbl.setStyleSheet("color: #555; font-size: 10px;")
        blowoff_widget = QLineEdit()
        self._active_inputs[corner_key]["blowoff"] = blowoff_widget
        blowoff_cell_layout.addWidget(blowoff_lbl)
        blowoff_cell_layout.addWidget(blowoff_widget)
        layout.addWidget(blowoff_cell)

        reb_row = QWidget()
        reb_layout = QHBoxLayout(reb_row)
        reb_layout.setContentsMargins(0, 0, 0, 0)
        reb_layout.setSpacing(8)
        for param, label_text in [("rebound_ls", "Rebound LS"), ("rebound_hs", "Rebound HS")]:
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2)
            lbl = QLabel(label_text)
            lbl.setStyleSheet("color: #555; font-size: 10px;")
            widget = QLineEdit()
            self._active_inputs[corner_key][param] = widget
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(widget)
            reb_layout.addWidget(cell)
        layout.addWidget(reb_row)

        if corner_label in ("FL", "RL"):
            mirror_target = "FR" if corner_label == "FL" else "RR"
            btn_mirror = QPushButton(f"<-> mirror damper to {mirror_target}")
            btn_mirror.setStyleSheet("background-color: #1e1e1e; color: #888; font-size: 10px; padding: 3px 8px;")
            btn_mirror.clicked.connect(lambda checked, cl=corner_label, inp=self._active_inputs: self._mirror_damper(cl, inp))
            layout.addWidget(btn_mirror)

        btn_advanced = QPushButton("> Damper Advanced")
        btn_advanced.setStyleSheet("background-color: #1a1a1a; color: #555; font-size: 10px; padding: 3px 8px; text-align: left;")
        btn_advanced.setCheckable(True)
        btn_advanced.setChecked(False)
        layout.addWidget(btn_advanced)

        advanced_widget = QWidget()
        advanced_layout = QVBoxLayout(advanced_widget)
        advanced_layout.setContentsMargins(0, 4, 0, 0)
        advanced_layout.setSpacing(4)
        advanced_widget.setVisible(False)

        for param in advanced_fields:
            widget = NoScrollSpinBox()
            widget.setRange(-9999, 9999)
            widget.setDecimals(2)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs[corner_key][param] = widget
            advanced_layout.addWidget(self._setup_row(labels[param], widget))

        layout.addWidget(advanced_widget)

        btn_advanced.toggled.connect(lambda checked, aw=advanced_widget, btn=btn_advanced: (
            aw.setVisible(checked),
            btn.setText("v Damper Advanced" if checked else "> Damper Advanced")
        ))

        return group

    def _build_car_center(self, params):
        center = QWidget()
        layout = QVBoxLayout(center)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        center.setFixedWidth(350)

        img_label = QLabel()
        img_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        img_label.setStyleSheet("border: 1px solid #2a2a2a; border-radius: 4px;")

        from PyQt6.QtGui import QPixmap
        pixmap = QPixmap("config/images/car_default.jpg")
        if not pixmap.isNull():
            pixmap = pixmap.scaledToHeight(750, Qt.TransformationMode.SmoothTransformation)
            img_label.setPixmap(pixmap)
        else:
            img_label.setText("[ car image ]")
            img_label.setStyleSheet("color: #333; border: 1px solid #2a2a2a; border-radius: 4px;")

        layout.addWidget(img_label)

        weights_group = QGroupBox("Weights")
        weights_group.setStyleSheet("""
            QGroupBox {
                color: #C0A060;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
        """)
        weights_layout = QVBoxLayout(weights_group)
        weights_layout.setSpacing(4)
        weights_layout.setContentsMargins(8, 12, 8, 8)

        weight_grid = QWidget()
        weight_grid_layout = QHBoxLayout(weight_grid)
        weight_grid_layout.setContentsMargins(0, 0, 0, 0)
        weight_grid_layout.setSpacing(4)

        left_weights = QWidget()
        left_weights_layout = QVBoxLayout(left_weights)
        left_weights_layout.setContentsMargins(0, 0, 0, 0)
        left_weights_layout.setSpacing(4)

        right_weights = QWidget()
        right_weights_layout = QVBoxLayout(right_weights)
        right_weights_layout.setContentsMargins(0, 0, 0, 0)
        right_weights_layout.setSpacing(4)

        for param, label_text in [("corner_weight_fl", "FL (kg)"), ("corner_weight_rl", "RL (kg)")]:
            widget = NoScrollSpinBox()
            widget.setRange(0, 9999)
            widget.setDecimals(1)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs["car"][param] = widget
            left_weights_layout.addWidget(self._setup_row(label_text, widget))

        for param, label_text in [("corner_weight_fr", "FR (kg)"), ("corner_weight_rr", "RR (kg)")]:
            widget = NoScrollSpinBox()
            widget.setRange(0, 9999)
            widget.setDecimals(1)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs["car"][param] = widget
            right_weights_layout.addWidget(self._setup_row(label_text, widget))

        weight_grid_layout.addWidget(left_weights)
        weight_grid_layout.addWidget(right_weights)
        weights_layout.addWidget(weight_grid)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setStyleSheet("color: #2a2a2a;")
        weights_layout.addWidget(separator)

        for param, label_text in [("total_weight", "Total (kg)"), ("cross_percentage", "Cross %")]:
            widget = NoScrollSpinBox()
            widget.setRange(0, 9999)
            widget.setDecimals(1)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs["car"][param] = widget
            weights_layout.addWidget(self._setup_row(label_text, widget))

        layout.addWidget(weights_group)

        car_group = QGroupBox("Car")
        car_group.setStyleSheet("""
            QGroupBox {
                color: #C0A060;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
        """)
        car_layout = QVBoxLayout(car_group)
        car_layout.setSpacing(4)
        car_layout.setContentsMargins(8, 12, 8, 8)

        car_labels = {
            "differential_preload": "Diff Preload",
            "differential_position": "Diff Position",
            "splitter_offset": "Splitter",
        }

        for param, label_text in car_labels.items():
            widget = NoScrollSpinBox()
            widget.setRange(-9999, 9999)
            widget.setDecimals(2)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            if param == "splitter_offset":
                # Distinguishes the SETTING (this field, car-referenced) from
                # the floor-referenced CHECK points added below -- both
                # coexist, neither replaces the other.
                widget.setToolTip("Splitter offset -- setting, vs car")
            self._active_inputs["car"][param] = widget
            car_layout.addWidget(self._setup_row(label_text, widget))

        splitter_label = QLabel("Splitter Points (measured, vs floor, mm)")
        splitter_label.setStyleSheet("color: #555; font-size: 10px; font-weight: 500; margin-top: 6px;")
        splitter_label.setToolTip("Floor-referenced check measurements -- distinct from the Splitter setting above.")
        car_layout.addWidget(splitter_label)
        splitter_widget = MeasurementPointsWidget("splitter")
        for i, edit in enumerate(splitter_widget.point_widgets, start=1):
            self._active_inputs["car"][f"splitter_point_{i}"] = edit
        car_layout.addWidget(splitter_widget)

        diffuser_label = QLabel("Diffuser Points (measured, vs floor, mm)")
        diffuser_label.setStyleSheet("color: #555; font-size: 10px; font-weight: 500; margin-top: 6px;")
        diffuser_label.setToolTip("Floor-referenced check measurements.")
        car_layout.addWidget(diffuser_label)
        diffuser_widget = MeasurementPointsWidget("diffuser")
        for i, edit in enumerate(diffuser_widget.point_widgets, start=1):
            self._active_inputs["car"][f"diffuser_point_{i}"] = edit
        car_layout.addWidget(diffuser_widget)

        arb_mount_combo = QComboBox()
        arb_mount_combo.addItems(["P0", "P1", "P2"])
        self._active_inputs["car"]["arb_front_mount"] = arb_mount_combo
        car_layout.addWidget(self._setup_row("ARB Front Mount", arb_mount_combo))

        # Legal set is P8/P9/P10 only (config/setup_parameters.json
        # wing_position registry entry, cross-checked against car_data.json
        # wing_position_table's GT3 R 2026 column) -- was a free-range
        # spinbox that permitted illegal intermediate values (WP4 UI-polish
        # note). A pre-existing outing whose stored value isn't one of these
        # three (from before this fix) silently falls back to this combo's
        # first item (P8) on load -- QComboBox.setCurrentText() on a
        # non-editable combo is a no-op for unmatched text, verified
        # empirically, not "nothing selected" as originally assumed.
        wing_position_combo = QComboBox()
        wing_position_combo.addItems(["P8", "P9", "P10"])
        self._active_inputs["car"]["wing_position"] = wing_position_combo
        car_layout.addWidget(self._setup_row("Wing Pos.", wing_position_combo))

        diff_torque_label = QLabel("Diff Locking Torque (measured, Nm)")
        diff_torque_label.setStyleSheet("color: #555; font-size: 10px; font-weight: 500; margin-top: 6px;")
        car_layout.addWidget(diff_torque_label)

        diff_torque_row = QWidget()
        diff_torque_layout = QHBoxLayout(diff_torque_row)
        diff_torque_layout.setContentsMargins(0, 0, 0, 0)
        diff_torque_layout.setSpacing(4)
        for pos in range(1, 6):
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2)
            lbl = QLabel(str(pos))
            lbl.setStyleSheet("color: #555; font-size: 10px;")
            widget = NoScrollSpinBox()
            widget.setRange(0, 9999)
            widget.setDecimals(0)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs["car"][f"differential_locking_torque_measured_{pos}"] = widget
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(widget)
            diff_torque_layout.addWidget(cell)
        car_layout.addWidget(diff_torque_row)

        layout.addWidget(car_group)
        layout.addStretch()
        return center

    def _mirror_damper(self, from_corner, inputs):
        mapping = {"FL": "FR", "RL": "RR"}
        from_key = {"FL": "front_left", "RL": "rear_left"}[from_corner]
        to_key = {"FR": "front_right", "RR": "rear_right"}[mapping[from_corner]]

        for param in ["bump_ls", "bump_hs", "blowoff", "rebound_ls", "rebound_hs"]:
            inputs[to_key][param].setText(
                inputs[from_key][param].text()
            )

    def _collect_inputs(self, inputs):
        import json
        data = {}
        for corner_key, fields in inputs.items():
            data[corner_key] = {}
            for param, widget in fields.items():
                if isinstance(widget, QDoubleSpinBox):
                    data[corner_key][param] = widget.value()
                elif isinstance(widget, QComboBox):
                    data[corner_key][param] = widget.currentText()
                elif isinstance(widget, QLineEdit):
                    data[corner_key][param] = widget.text().strip()
                elif isinstance(widget, QTextEdit):
                    data[corner_key][param] = widget.toPlainText().strip()
        return json.dumps(data)

    def _reshape_diff_torque_out(self, json_string):
        import json
        data = json.loads(json_string)
        car = data.get("car")
        if isinstance(car, dict):
            torque = {}
            for pos in range(1, 6):
                key = f"differential_locking_torque_measured_{pos}"
                if key in car:
                    torque[str(pos)] = car.pop(key)
            if torque:
                car["differential_locking_torque_measured"] = torque
        return json.dumps(data)

    def _reshape_diff_torque_in(self, json_string):
        import json
        if not json_string:
            return json_string
        try:
            data = json.loads(json_string)
        except (json.JSONDecodeError, TypeError):
            return json_string
        car = data.get("car")
        if isinstance(car, dict):
            torque = car.pop("differential_locking_torque_measured", None)
            if isinstance(torque, dict):
                for pos in range(1, 6):
                    key = str(pos)
                    if key in torque:
                        car[f"differential_locking_torque_measured_{pos}"] = torque[key]
        return json.dumps(data)

    # Splitter/diffuser measurement points (mm, floor-referenced -- distinct
    # from the existing splitter_offset SETTING, which is car-referenced
    # and untouched). Reshape logic lives in core/setup_data_points.py (pure
    # JSON transform, no Qt) rather than here, so it's testable without
    # importing this PyQt6 module -- same reason tests/conftest.py's own
    # pipeline_result fixture keeps this file out of the regression suite.
    # Same pop-based mechanism as the diff-torque reshape above; widgets
    # bind to flat splitter_point_1.._5 / diffuser_point_1.._5 keys
    # (ui/views/measurement_points_widget.py), folded to/from a plain array
    # under car[...] on save/load so a missing array (any outing saved
    # before this feature) leaves the flat keys entirely absent -- the
    # normal _load_inputs "skip unknown param" path then leaves those
    # widgets at their default empty state, no explicit migration needed.
    def _reshape_points_out(self, json_string):
        from core.setup_data_points import reshape_points_out
        return reshape_points_out(json_string)

    def _reshape_points_in(self, json_string):
        from core.setup_data_points import reshape_points_in
        return reshape_points_in(json_string)

    def _collect_setup_data(self):
        return self._reshape_points_out(self._reshape_diff_torque_out(self._collect_inputs(self.setup_inputs)))

    def _collect_setdown_data(self):
        return self._reshape_points_out(self._reshape_diff_torque_out(self._collect_inputs(self.setdown_inputs)))

    def _collect_feedback_data(self):
        import json
        corners = []
        for row_data in self.corner_rows:
            corners.append({
                "worst": row_data["worst"].isChecked(),
                "e1": row_data["e1"].value(),
                "e2": row_data["e2"].value(),
                "a3": row_data["a3"].value(),
                "x4": row_data["x4"].value(),
                "x5": row_data["x5"].value(),
            })
        return json.dumps({
            "corner_count": self.corner_count_spin.value(),
            "corners": corners,
            "map_path": self.feedback_map_path or ""
        })

    def _load_feedback_data(self, json_string):
        import json
        if not json_string:
            return
        try:
            data = json.loads(json_string)
        except (json.JSONDecodeError, TypeError):
            return

        count = data.get("corner_count", 10)
        self.corner_count_spin.setValue(count)

        for i, row_data in enumerate(self.corner_rows):
            if i >= len(data.get("corners", [])):
                break
            c = data["corners"][i]
            row_data["worst"].setChecked(c.get("worst", False))
            row_data["e1"].setValue(c.get("e1", 0))
            row_data["e2"].setValue(c.get("e2", 0))
            row_data["a3"].setValue(c.get("a3", 0))
            row_data["x4"].setValue(c.get("x4", 0))
            row_data["x5"].setValue(c.get("x5", 0))

        map_path = data.get("map_path", "")
        if map_path:
            self.feedback_map_path = map_path
            self._display_track_map(map_path)

    def _load_inputs(self, inputs_dict, json_string):
        import json
        if not json_string:
            return
        try:
            data = json.loads(json_string)
        except (json.JSONDecodeError, TypeError):
            return

        for corner_key, fields in data.items():
            if corner_key not in inputs_dict:
                continue
            for param, value in fields.items():
                if param not in inputs_dict[corner_key]:
                    continue
                widget = inputs_dict[corner_key][param]
                if isinstance(widget, QDoubleSpinBox):
                    try:
                        widget.setValue(float(value) if value else 0.0)
                    except (ValueError, TypeError):
                        pass
                elif isinstance(widget, QComboBox):
                    if value:
                        widget.setCurrentText(str(value))
                elif isinstance(widget, QLineEdit):
                    widget.setText(str(value) if value else "")
                elif isinstance(widget, QTextEdit):
                    widget.setPlainText(str(value) if value else "")

    def _load_setup_data(self, json_string):
        self._load_inputs(self.setup_inputs, self._reshape_points_in(self._reshape_diff_torque_in(json_string)))

    def _load_setdown_data(self, json_string):
        self._load_inputs(self.setdown_inputs, self._reshape_points_in(self._reshape_diff_torque_in(json_string)))

    def _prefill_setdown(self):
        if self.outing and self.outing.setdown_data:
            self._load_inputs(self.setdown_inputs,
                               self._reshape_points_in(self._reshape_diff_torque_in(self.outing.setdown_data)))
        else:
            self._load_inputs(self.setdown_inputs,
                               self._reshape_points_in(self._reshape_diff_torque_in(self._collect_setup_data())))

    def _print_sheet(self, sheet_type):
        from PyQt6.QtWidgets import QFileDialog, QMessageBox
        from core.pdf_export import generate_setup_pdf
        import os

        label = "Setup" if sheet_type == "setup" else "Setdown"
        default_name = f"{self.weekend.track}_Outing{self.outing.number if self.outing else 'new'}_{self.session_type_combo.currentText()}_{label}.pdf"

        path, _ = QFileDialog.getSaveFileName(
            self, f"Save {label} PDF", default_name, "PDF Files (*.pdf)",
            options=QFileDialog.Option.DontConfirmOverwrite
        )

        if not path:
            return

        if not path.endswith(".pdf"):
            path += ".pdf"

        if os.path.exists(path):
            reply = QMessageBox.question(
                self, "File exists",
                f"{os.path.basename(path)} already exists. Do you want to replace it?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.No:
                base = path[:-4]
                counter = 2
                while os.path.exists(f"{base}_{counter}.pdf"):
                    counter += 1
                path = f"{base}_{counter}.pdf"

        class TempOuting:
            pass

        temp = TempOuting()
        temp.setup_data = self._collect_setup_data() if sheet_type == "setup" else self._collect_setdown_data()
        temp.date_time = self.datetime_edit.dateTime().toPyDateTime()
        temp.number = self.outing.number if self.outing else "new"
        temp.name = self.name_input.text().strip()
        temp.session_type = self.session_type_combo.currentText()
        temp.driver_name = self.driver_combo.currentText()

        try:
            generate_setup_pdf(temp, self.weekend, path, sheet_type=label)
        except PermissionError:
            QMessageBox.warning(
                self, "Save failed",
                f"Could not save {os.path.basename(path)}.\nThe file may be open in another program."
            )
        except Exception as e:
            # Reliability pass: PermissionError was the only exception
            # this ever caught -- anything else (e.g. a KeyError from
            # malformed setup data) propagated unhandled out of this Qt
            # slot with no dialog telling the user the export failed.
            # Full traceback to the console/log, a friendly message here.
            print(traceback.format_exc())
            QMessageBox.warning(
                self, "Save failed",
                f"Could not save {os.path.basename(path)}: {friendly_error_text(e)}"
            )

    def _build_corner_map(self):
        # WP3b interim: GPS outline of the reference lap + one marker per
        # stable_corner_id, as the visual legend for the feedback table's
        # row numbers below. Static v1 -- no click interaction; that's the
        # WP3b follow-up (PLAN.md). Sits above Stability Analysis, not in
        # Driver Feedback: this is the legend for the ANALYSIS layer
        # (stable_corner_id, matching the grid/recommendations), not the
        # human/official-name layer the driver feedback table and its
        # separate image-loader track map use -- the two-layer corner
        # identity design (thesis_notes.md) reflected directly in layout.
        import pyqtgraph as pg

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self._section_label("Corner Map"))

        self.corner_map_plot = pg.PlotWidget()
        self.corner_map_plot.setBackground(PANEL)
        self.corner_map_plot.setMinimumHeight(280)
        self.corner_map_plot.setAspectLocked(True)
        self.corner_map_plot.hideAxis('left')
        self.corner_map_plot.hideAxis('bottom')
        self.corner_map_plot.getViewBox().setMouseEnabled(x=False, y=False)
        self.corner_map_plot.getViewBox().wheelEvent = lambda event: None
        # Corner click-through: one shared scene-click handler, same pattern
        # as LapTraceDialog._on_scene_clicked (corner_trace_dialog.py) --
        # hit-tests against stored marker positions rather than per-item
        # signals, so a click on either the dot or its text label counts.
        self.corner_map_plot.scene().sigMouseClicked.connect(self._on_corner_map_clicked)
        layout.addWidget(self.corner_map_plot)

        self.corner_map_hint_label = QLabel("")
        self.corner_map_hint_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        layout.addWidget(self.corner_map_hint_label)

        self.corner_map_trace_curve = None
        self.corner_map_trace_xy = None
        self.corner_map_markers = {}
        self.corner_map_marker_xy = {}  # stable_corner_id -> (x_m, y_m), for click hit-testing
        self._show_corner_map_placeholder("Load a CSV to see the track map.")

        return container

    def _show_corner_map_placeholder(self, text):
        import pyqtgraph as pg
        self.corner_map_plot.clear()
        self.corner_map_trace_curve = None
        self.corner_map_trace_xy = None
        self.corner_map_markers = {}
        self.corner_map_marker_xy = {}
        self.corner_map_hint_label.setText("")
        placeholder = pg.TextItem(text, color=TEXT_DIM, anchor=(0.5, 0.5))
        self.corner_map_plot.addItem(placeholder)
        self.corner_map_plot.setRange(xRange=(-1, 1), yRange=(-1, 1))

    def _snap_to_trace(self, x, y):
        # Cross-lap median apex position vs a single reference lap's drawn
        # trace can float off the line (worst in compound corners, where
        # the apex position itself is unstable lap-to-lap). Position
        # estimate stays the cross-lap median; only the DISPLAYED point is
        # snapped to the nearest vertex on the drawn polyline, so markers
        # always sit on the line the driver/engineer is actually reading.
        import numpy as np
        if self.corner_map_trace_xy is None:
            return x, y
        tx, ty = self.corner_map_trace_xy
        if len(tx) == 0:
            return x, y
        d2 = (tx - x) ** 2 + (ty - y) ** 2
        idx = int(np.argmin(d2))
        return float(tx[idx]), float(ty[idx])

    def _update_corner_map_trace(self):
        import numpy as np
        import pyqtgraph as pg
        from modules.geo import compute_gps_origin, project_latlon_to_xy

        if not self.parsed_data:
            self._show_corner_map_placeholder("Load a CSV to see the track map.")
            return

        channels = self.parsed_data.get("channels", {})
        gps_lat_ch = channels.get("log_gps_lat")
        gps_lon_ch = channels.get("log_gps_lon")
        origin_lat, origin_lon = compute_gps_origin(gps_lat_ch, gps_lon_ch)
        if origin_lat is None:
            self._show_corner_map_placeholder("No GPS data in this file.")
            return

        laps = self.parsed_data.get("laps", [])
        valid_laps = [l for l in laps if l.get("is_valid_for_analysis")]
        target_lap = next((l for l in valid_laps if l.get("is_fastest")), None)
        if target_lap is None and valid_laps:
            target_lap = valid_laps[0]
        if target_lap is None:
            self._show_corner_map_placeholder("No valid lap to plot.")
            return

        t = gps_lat_ch["time"]
        lat_d, lon_d = gps_lat_ch["data"], gps_lon_ch["data"]
        mask = (t >= target_lap["start_time"]) & (t <= target_lap["end_time"])
        if not mask.any():
            self._show_corner_map_placeholder("No GPS samples in the reference lap.")
            return

        x, y = project_latlon_to_xy(lat_d[mask], lon_d[mask], origin_lat, origin_lon)

        self.corner_map_plot.clear()
        self.corner_map_markers = {}
        self.corner_map_marker_xy = {}
        self.corner_map_hint_label.setText("")
        self.corner_map_trace_xy = (np.asarray(x), np.asarray(y))
        self.corner_map_trace_curve = self.corner_map_plot.plot(
            x, y, pen=pg.mkPen(color=TEXT_MUTED, width=2)
        )
        self.corner_map_plot.enableAutoRange()

    def _update_corner_map_markers(self):
        import pyqtgraph as pg
        from modules.corner_analysis import compute_stable_corner_positions

        if not self.parsed_data or self.corner_map_trace_curve is None:
            return  # no trace drawn -- no GPS, or nothing loaded yet

        if self.corner_positions_cache is None:
            corners = self.parsed_data.get("corners", [])
            channels = self.parsed_data.get("channels", {})
            self.corner_positions_cache = compute_stable_corner_positions(corners, channels)

        positions = self.corner_positions_cache
        if not positions:
            return

        colour_by_id = {}
        if self.stability_result:
            from modules.recommendation import aggregate_by_corner
            aggregated = aggregate_by_corner(self.stability_result["summaries"])
            for cid, agg in aggregated.items():
                _severity, _short, _long, colour = self._classify_corner(agg)
                colour_by_id[cid] = colour

        # Drop markers for corners that no longer exist (new file loaded).
        for cid in list(self.corner_map_markers.keys()):
            if cid not in positions:
                scatter, text = self.corner_map_markers.pop(cid)
                self.corner_map_plot.removeItem(scatter)
                self.corner_map_plot.removeItem(text)
                self.corner_map_marker_xy.pop(cid, None)

        for cid, pos in positions.items():
            colour = colour_by_id.get(cid, NEUTRAL)
            if cid in self.corner_map_markers:
                scatter, _text = self.corner_map_markers[cid]
                scatter.setBrush(pg.mkBrush(colour))
            else:
                snap_x, snap_y = self._snap_to_trace(pos["x_m"], pos["y_m"])
                # Marker dot size, px -- CORNER_MARKER_CLICK_RADIUS_PX (top
                # of file, used by _on_corner_map_clicked) matches this
                # value so the click target tracks the dot's own footprint.
                scatter = pg.ScatterPlotItem(
                    [snap_x], [snap_y], size=26,
                    brush=pg.mkBrush(colour), pen=pg.mkPen(None)
                )
                text = pg.TextItem(
                    html=f'<b style="font-size: 13pt; color: #111111;">{cid}</b>',
                    anchor=(0.5, 0.5)
                )
                text.setPos(snap_x, snap_y)
                self.corner_map_plot.addItem(scatter)
                self.corner_map_plot.addItem(text)
                self.corner_map_markers[cid] = (scatter, text)
                self.corner_map_marker_xy[cid] = (snap_x, snap_y)

    def _on_corner_map_clicked(self, event):
        # Same scene-click + hit-test pattern as LapTraceDialog.
        # _on_scene_clicked (ui/views/corner_trace_dialog.py) -- one shared
        # handler against stored positions, not a signal per marker item
        # (pg.TextItem has no native click signal, so a per-item-signal
        # approach would miss clicks on the label half of each marker).
        pos = event.scenePos()
        if not self.corner_map_plot.sceneBoundingRect().contains(pos):
            return
        if not self.corner_map_marker_xy:
            return

        view_box = self.corner_map_plot.getViewBox()
        view_pos = view_box.mapSceneToView(pos)
        x, y = view_pos.x(), view_pos.y()

        px_w, px_h = view_box.viewPixelSize()
        radius = CORNER_MARKER_CLICK_RADIUS_PX * max(px_w, px_h)
        radius_sq = radius * radius

        best_cid, best_d2 = None, None
        for cid, (mx, my) in self.corner_map_marker_xy.items():
            d2 = (mx - x) ** 2 + (my - y) ** 2
            if d2 <= radius_sq and (best_d2 is None or d2 < best_d2):
                best_cid, best_d2 = cid, d2

        if best_cid is None:
            return
        self._open_corner_trace_from_map(best_cid)

    def _resolve_worst_lap_summary_for_corner(self, stable_corner_id):
        # Mirrors ui/views/corner_trace_dialog.py's _aggregate_worst_severity
        # (the logic that already decides this same marker's colour) but
        # returns the SUMMARY that produced the worst rank, not just the
        # colour -- the map marker is per-stable_corner_id, but
        # CornerTraceDialog.show_corner needs one specific lap's instance.
        if not self.stability_result:
            return None
        summaries = self.stability_result.get("summaries")
        if not summaries:
            return None
        laps_by_number = {l["lap_number"]: l for l in (self.parsed_data or {}).get("laps", [])}
        candidates = [
            s for s in summaries
            if s["stable_corner_id"] == stable_corner_id
            and laps_by_number.get(s["lap_number"], {}).get("is_valid_for_analysis")
        ]
        if not candidates:
            return None

        from modules.recommendation import SEVERITY_RANK
        ranked = [(SEVERITY_RANK[self._classify_corner(s)[0]], s) for s in candidates]
        max_rank = max(rank for rank, _s in ranked)
        worst = [s for rank, s in ranked if rank == max_rank]

        # A single genuine worst-severity instance wins outright. A tie
        # (>1 candidate sharing the worst rank) or no spread at all (every
        # lap "normal" -- the same case the marker's own colour shows as
        # NEUTRAL) falls back to the fastest valid lap, same is_fastest
        # flag _update_corner_map_trace already uses for the map's own
        # reference lap.
        if len(worst) == 1 and max_rank > SEVERITY_RANK["normal"]:
            return worst[0]

        fastest = next((s for s in candidates
                         if laps_by_number.get(s["lap_number"], {}).get("is_fastest")), None)
        return fastest or candidates[0]

    def _open_corner_trace_from_map(self, stable_corner_id):
        summary = self._resolve_worst_lap_summary_for_corner(stable_corner_id)
        if summary is None:
            self.corner_map_hint_label.setText("Run Analyse to open corner details")
            return
        self.corner_map_hint_label.setText("")
        self._open_corner_trace(summary)

    def _build_feedback_section(self):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self._section_label("Driver Feedback"))

        count_row = QWidget()
        count_layout = QHBoxLayout(count_row)
        count_layout.setContentsMargins(0, 0, 0, 0)
        count_label = QLabel("Corners")
        count_label.setStyleSheet("color: #888;")
        self.corner_count_spin = NoScrollIntSpinBox()
        self.corner_count_spin.setRange(1, 30)
        self.corner_count_spin.setValue(10)
        self.corner_count_spin.setFixedWidth(60)
        self.corner_count_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.corner_count_spin.setStyleSheet("""
            QSpinBox {
                background-color: #1a1a1a;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                color: #e0e0e0;
                padding: 4px 8px;
                font-size: 12px;
            }
        """)
        count_layout.addWidget(count_label)
        count_layout.addSpacing(8)
        count_layout.addWidget(self.corner_count_spin)
        count_layout.addStretch()
        layout.addWidget(count_row)

        split = QWidget()
        split_layout = QHBoxLayout(split)
        split_layout.setContentsMargins(0, 0, 0, 0)
        split_layout.setSpacing(16)

        self.feedback_table = QTableWidget()
        self.feedback_table.setColumnCount(7)
        self.feedback_table.setHorizontalHeaderLabels(
            ["No.", "Worst", "Entry 1", "Entry 2", "Apex 3", "Exit 4", "Exit 5"]
        )
        self.feedback_table.verticalHeader().setVisible(False)
        self.feedback_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.feedback_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.feedback_table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.feedback_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.feedback_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.feedback_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        self.feedback_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        self.feedback_table.setColumnWidth(0, 36)
        self.feedback_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        self.feedback_table.setColumnWidth(1, 52)
        for col in range(2, 7):
            self.feedback_table.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeMode.Stretch)
        self.feedback_table.setStyleSheet("""
            QTableWidget { background-color: #141414; border: none; gridline-color: #1e1e1e; outline: 0; }
            QTableWidget::item { padding: 2px; border-bottom: 1px solid #1e1e1e; }
            QTableWidget::item:selected { background-color: #141414; }
            QHeaderView::section { background-color: #1a1a1a; color: #555; font-size: 10px;
                padding: 6px 4px; border: none; border-bottom: 1px solid #222; }
        """)

        self._rebuild_corner_rows(self.corner_count_spin.value())
        self.corner_count_spin.valueChanged.connect(self._rebuild_corner_rows)

        split_layout.addWidget(self.feedback_table, 3)

        map_panel = QWidget()
        map_layout = QVBoxLayout(map_panel)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.setSpacing(8)

        map_title = QLabel("Track Map")
        map_title.setStyleSheet("color: #888; font-size: 11px;")
        map_layout.addWidget(map_title)

        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_label.setMinimumHeight(200)
        self.map_label.setStyleSheet("""
            QLabel {
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                color: #333;
                font-size: 11px;
                background-color: #1a1a1a;
            }
        """)
        self.map_label.setText("No track map loaded")
        map_layout.addWidget(self.map_label, 1)

        btn_load_map = QPushButton("Load Image")
        btn_load_map.setFixedWidth(120)
        btn_load_map.clicked.connect(self._load_track_map)
        map_layout.addWidget(btn_load_map)

        self.map_filename_label = QLabel("")
        self.map_filename_label.setStyleSheet("color: #555; font-size: 10px;")
        map_layout.addWidget(self.map_filename_label)
        map_layout.addStretch()

        split_layout.addWidget(map_panel, 2)
        layout.addWidget(split)

        scale_desc = QLabel(
            "Scale: -5 undrivable understeer | -3 strong understeer | -1 slight understeer | "
            "0 neutral | +1 slight oversteer | +3 strong oversteer | +5 undrivable oversteer\n"
            "Placeholder -- full description to be added per value."
        )
        scale_desc.setStyleSheet("color: #444; font-size: 10px; margin-top: 4px;")
        scale_desc.setWordWrap(True)
        layout.addWidget(scale_desc)

        return section

    def _rebuild_corner_rows(self, count):
        existing = []
        for row_data in self.corner_rows:
            existing.append({
                "worst": row_data["worst"].isChecked(),
                "values": [row_data[k].value() for k in ["e1", "e2", "a3", "x4", "x5"]]
            })

        self.corner_rows = []
        self.feedback_table.setRowCount(0)

        for i in range(count):
            row = self.feedback_table.rowCount()
            self.feedback_table.insertRow(row)
            self.feedback_table.setRowHeight(row, 28)

            num_label = QLabel(str(i + 1))
            num_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            num_label.setStyleSheet("color: #C0A060; font-weight: 600; font-size: 11px; background: transparent;")
            self.feedback_table.setCellWidget(row, 0, num_label)

            check_container = QWidget()
            check_container.setStyleSheet("background: transparent;")
            check_layout = QHBoxLayout(check_container)
            check_layout.setContentsMargins(0, 0, 0, 0)
            check_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            checkbox = QCheckBox()
            checkbox.setStyleSheet("""
                QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #2a2a2a; border-radius: 2px; background: #1a1a1a; }
                QCheckBox::indicator:checked { background-color: #C0A060; border-color: #C0A060; }
            """)
            check_layout.addWidget(checkbox)
            self.feedback_table.setCellWidget(row, 1, check_container)

            prev = existing[i] if i < len(existing) else None
            spins = {}
            for col_idx, key in enumerate(["e1", "e2", "a3", "x4", "x5"]):
                spin = NoScrollIntSpinBox()
                spin.setRange(-5, 5)
                spin.setValue(prev["values"][col_idx] if prev else 0)
                spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
                spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
                spin.setStyleSheet("""
                    QSpinBox {
                        background-color: #1a1a1a;
                        border: 1px solid #2a2a2a;
                        color: #e0e0e0;
                        padding: 2px;
                        font-size: 12px;
                    }
                """)
                self.feedback_table.setCellWidget(row, col_idx + 2, spin)
                spins[key] = spin

            if prev:
                checkbox.setChecked(prev["worst"])

            self.corner_rows.append({"worst": checkbox, **spins})

        header_h = self.feedback_table.horizontalHeader().height()
        total_row_h = sum(self.feedback_table.rowHeight(i) for i in range(count))
        self.feedback_table.setFixedHeight(header_h + total_row_h + 4)

    def _load_track_map(self):
        from PyQt6.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Track Map", "", "Images (*.png *.jpg *.jpeg *.bmp)"
        )
        if path:
            self.feedback_map_path = path
            self._display_track_map(path)

    def _display_track_map(self, path):
        from PyQt6.QtGui import QPixmap
        import os
        pixmap = QPixmap(path)
        if not pixmap.isNull():
            w = self.map_label.width() or 300
            scaled = pixmap.scaledToWidth(w, Qt.TransformationMode.SmoothTransformation)
            self.map_label.setPixmap(scaled)
            self.map_filename_label.setText(os.path.basename(path))
        else:
            self.map_label.setText("Could not load image")

    def _build_comments_section(self):
        section = QWidget()
        section.setStyleSheet("background-color: #1a1a1a; border-radius: 4px;")
        layout = QVBoxLayout(section)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)
        layout.addWidget(self._section_label("Comments"))

        self.comments_input = QTextEdit()
        self.comments_input.setMinimumHeight(160)
        self.comments_input.setPlaceholderText("General notes about this outing...")
        self.comments_input.setStyleSheet("""
            QTextEdit {
                background-color: #141414;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                padding: 8px;
                color: #e0e0e0;
                font-size: 13px;
            }
        """)
        layout.addWidget(self.comments_input)

        return section

    def _load_drivers(self):
        session = Session()
        drivers = session.query(Driver).order_by(Driver.name).all()
        for driver in drivers:
            self.driver_combo.addItem(driver.name, userData=driver.id)
        session.close()

    def _prefill(self):
        if not self.outing:
            return
        self.datetime_edit.setDateTime(QDateTime.fromString(
            self.outing.date_time.strftime("%Y-%m-%d %H:%M:%S"), "yyyy-MM-dd HH:mm:ss"))
        self.name_input.setText(self.outing.name or "")
        if self.outing.driver_id:
            index = self.driver_combo.findData(self.outing.driver_id)
            if index >= 0:
                self.driver_combo.setCurrentIndex(index)
        if self.outing.session_type:
            self.session_type_combo.setCurrentText(self.outing.session_type)
        if self.outing.tyre_type:
            self.tyre_type_combo.setCurrentText(self.outing.tyre_type)
        self.tyre_name_input.setText(self.outing.tyre_name or "")
        if self.outing.tyre_age:
            self.tyre_age_input.setValue(self.outing.tyre_age)
        if self.outing.fuel_level:
            self.fuel_load_input.setValue(self.outing.fuel_level)
        if self.outing.air_temp:
            self.air_temp_input.setValue(self.outing.air_temp)
        if self.outing.track_temp:
            self.track_temp_input.setValue(self.outing.track_temp)
        if self.outing.track_condition:
            self.track_condition_combo.setCurrentText(self.outing.track_condition)
        self.comments_input.setPlainText(self.outing.comments or "")
        self._load_setup_data(self.outing.setup_data)
        self._load_feedback_data(self.outing.feedback_data)
        if self.outing.setdown_data:
            self._load_setdown_data(self.outing.setdown_data)
        if self.outing.csv_path:
            QTimer.singleShot(100, lambda: self._auto_load_csv(self.outing.csv_path))

    def _auto_load_csv(self, path):
        import os
        if not os.path.exists(path):
            self.csv_status_label.setText("Data file not found at saved path")
            self.csv_status_label.setStyleSheet("color: #555; font-size: 12px;")
            return
        self.progress = __import__('PyQt6.QtWidgets', fromlist=['QProgressDialog']).QProgressDialog(
            "Loading outing data...", None, 0, 0, self)
        self.progress.setWindowTitle("Loading")
        self.progress.setWindowModality(Qt.WindowModality.WindowModal)
        self.progress.setMinimumDuration(0)
        self.progress.show()
        self.loader_thread = CsvLoaderThread(path)
        self.loader_thread.finished.connect(self._on_csv_loaded)
        self.loader_thread.error.connect(self._on_csv_error)
        self.loader_thread.start()

    def _carryon_from_last(self):
        session = Session()
        last_outing = (
            session.query(Outing)
            .filter(Outing.race_weekend_id == self.weekend.id)
            .order_by(Outing.date_time.desc())
            .first()
        )
        session.close()

        if not last_outing:
            return
        if last_outing.driver_id:
            index = self.driver_combo.findData(last_outing.driver_id)
            if index >= 0:
                self.driver_combo.setCurrentIndex(index)
        if last_outing.tyre_type:
            self.tyre_type_combo.setCurrentText(last_outing.tyre_type)
        if last_outing.tyre_name:
            self.tyre_name_input.setText(last_outing.tyre_name)
        if last_outing.air_temp:
            self.air_temp_input.setValue(last_outing.air_temp)
        if last_outing.track_temp:
            self.track_temp_input.setValue(last_outing.track_temp)
        if last_outing.track_condition:
            self.track_condition_combo.setCurrentText(last_outing.track_condition)
        self._load_setup_data(last_outing.setup_data)

    def _delete_outing(self):
        if self.outing:
            from sqlalchemy import delete
            session = Session()
            session.execute(delete(Outing).where(Outing.id == self.outing.id))
            session.commit()
            session.close()
        self.on_back()

    def _persist_outing(self):
        # Shared persistence core for both Back (_save_outing) and the
        # explicit Save button (_on_save_clicked). Creates the row on the
        # first call in new-outing mode and sets self.outing to it -- every
        # later call (another Save, or Back) then takes the update path,
        # so a new outing can never be inserted twice. Returns the just-
        # written setup_data JSON string so a caller can react to it
        # (e.g. the post-save stale-analysis check) without re-reading
        # the DB.
        driver_id = self.driver_combo.currentData()
        setup_data_json = self._collect_setup_data()
        field_values = dict(
            date_time=self.datetime_edit.dateTime().toPyDateTime(),
            name=self.name_input.text().strip(),
            driver_id=driver_id,
            session_type=self.session_type_combo.currentText(),
            tyre_type=self.tyre_type_combo.currentText(),
            tyre_name=self.tyre_name_input.text().strip(),
            tyre_age=self.tyre_age_input.value(),
            fuel_level=self.fuel_load_input.value(),
            air_temp=self.air_temp_input.value(),
            track_temp=self.track_temp_input.value(),
            track_condition=self.track_condition_combo.currentText(),
            comments=self.comments_input.toPlainText().strip(),
            setup_data=setup_data_json,
            setdown_data=self._collect_setdown_data(),
            feedback_data=self._collect_feedback_data(),
            csv_path=self.loaded_csv_path or "",
            analysis_data=self._analysis_data_json,
        )

        session = Session()
        if self.outing:
            from sqlalchemy import update
            session.execute(
                update(Outing).where(Outing.id == self.outing.id).values(**field_values)
            )
            outing = None
        else:
            outing_count = session.query(Outing).filter(
                Outing.race_weekend_id == self.weekend.id).count()
            outing = Outing(
                race_weekend_id=self.weekend.id,
                number=outing_count + 1,
                **field_values,
            )
            session.add(outing)
        session.commit()
        if outing is not None:
            # session.commit() expires every attribute on the object by
            # default; refresh while the session is still open so id and
            # every column are safely cached before the session closes --
            # reading an expired attribute on an already-detached instance
            # raises DetachedInstanceError (hit by the synthetic test this
            # WP added, not by any prior code path, since every existing
            # post-close use of self.outing only ever WRITES to it).
            session.refresh(outing)
        session.close()

        if outing is not None:
            # First save in new-outing mode: the row now exists -- from
            # here on this form behaves exactly like edit mode, so a
            # later Save or Back updates this same row instead of
            # inserting a second one.
            self.outing = outing
        else:
            for key, value in field_values.items():
                setattr(self.outing, key, value)

        return setup_data_json

    def _save_outing(self):
        self._persist_outing()
        self.on_back()

    def _on_save_clicked(self):
        setup_data_json = self._persist_outing()
        self.btn_save.setText("Saved")
        QTimer.singleShot(1500, lambda: self.btn_save.setText("Save"))
        self._warn_if_setup_data_changed_since_analysis(setup_data_json)

    def _warn_if_setup_data_changed_since_analysis(self, setup_data_json):
        # Post-save hint (no auto-rerun): only meaningful if a stability
        # analysis is currently rendered at all -- _displayed_resolved_
        # vehicle_snapshot is set exactly there (_render_stability_
        # summaries), for both the live-run and cache-hit paths.
        if self._displayed_resolved_vehicle_snapshot is None:
            return
        import json
        from modules.stability_analysis import load_parameters
        from modules.accuracy_resolution import resolve_accuracy
        try:
            setup_data = json.loads(setup_data_json)
        except (json.JSONDecodeError, TypeError) as e:
            print(f"[WARN] setup-data-changed check skipped, could not parse setup_data: {e!r}")
            return
        cap = self._get_accuracy_cap_from_selector()
        current_resolved = resolve_accuracy(load_parameters(), setup_data, cap)
        if current_resolved["values"] != self._displayed_resolved_vehicle_snapshot:
            self.stability_status_label.setText(
                "setup data changed - re-run Analyse to use it"
            )
            self.stability_status_label.setStyleSheet(f"color: {WARN}; font-size: 12px;")