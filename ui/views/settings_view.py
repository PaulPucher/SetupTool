# Settings page: vehicle constants, analysis tunables, setup-engine
# repetition rule and thresholds (parameters.json), speed-class thresholds
# (channels.json). Only reads/writes the JSON and clears the loader caches.

import copy
import datetime
import json
import os

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QScrollArea,
    QLabel, QPushButton,
)
from PyQt6.QtCore import Qt

from ui.style import ACCENT, WARN, TEXT, TEXT_MUTED, TEXT_DIM
from ui.views.outing_form import NoScrollSpinBox, NoScrollIntSpinBox, latest_pipeline_entry
from core.threshold_overrides import apply_threshold_edit, is_manual
from modules.wheel_loads import effective_cl_area

PARAMETERS_PATH = "config/parameters.json"
CHANNELS_PATH = "config/channels.json"

# left label column; labels wrap inside it instead of clipping
LABEL_WIDTH = 260
# placeholder marker bar, drawn inside the row's own left margin; the label
# column shrinks by the same indent so the value column stays aligned
MARKER_BAR_WIDTH = 2
MARKER_INDENT = 8

_AERO_INACTIVE = ("Inactive while Cl*A = 0. Live aero is fitted per session (level 2), "
                  "which outranks a config value (level 1).")

# Section 1: vehicle constants. corner_weights and cog_to_*_axle_m left
# out -- they have per-session resolution (accuracy_resolution).
SECTION1_FIELDS = [
    {"path": ("vehicle", "mass_kg"), "label": "Mass", "unit": "kg",
     "decimals": 1, "min": 500.0, "max": 2000.0,
     "note_path": ("vehicle", "mass_note"), "accuracy_key": "mass",
     "short_note": "Car mass with driver and fuel. A filled corner-weight sheet replaces it per session."},
    {"path": ("vehicle", "cog_height_m"), "label": "CoG height", "unit": "m",
     "decimals": 3, "min": 0.05, "max": 1.0,
     "note_path": ("vehicle", "cog_height_note"), "accuracy_key": None,
     "short_note": "Sets longitudinal and lateral load transfer."},
    {"path": ("vehicle", "track_width_front_m"), "label": "Track width front", "unit": "m",
     "decimals": 3, "min": 0.8, "max": 2.2,
     "note_path": ("vehicle", "track_width_note"), "accuracy_key": None,
     "short_note": "Sets lateral load transfer on the front axle."},
    {"path": ("vehicle", "track_width_rear_m"), "label": "Track width rear", "unit": "m",
     "decimals": 3, "min": 0.8, "max": 2.2,
     "note_path": ("vehicle", "track_width_note"), "accuracy_key": None,
     "short_note": "Sets lateral load transfer on the rear axle."},
    {"path": ("vehicle", "wheelbase_m"), "label": "Wheelbase", "unit": "m",
     "decimals": 3, "min": 1.5, "max": 3.5,
     "note_path": None, "accuracy_key": "wheelbase_m",
     "short_note": "Axle distance; sets the static front/rear split and the yaw lever arms."},
    {"path": ("vehicle", "yaw_inertia_kgm2"), "label": "Yaw inertia (Iz)", "unit": "kg*m^2",
     "decimals": 1, "min": 500.0, "max": 5000.0,
     "note_path": ("vehicle", "yaw_inertia_note"), "accuracy_key": "yaw_inertia",
     "short_note": "Yaw inertia; used by the sideslip filters and the yaw-stability estimate."},
    {"path": ("vehicle", "steering_ratio"), "label": "Steering ratio (constant)", "unit": "",
     "decimals": 2, "min": 5.0, "max": 25.0,
     "note_path": ("vehicle", "steering_ratio_note"), "accuracy_key": "steering_ratio",
     "short_note": "Steering-wheel to road-wheel angle; used when no ratio table is available."},
    {"path": ("vehicle", "aero", "air_density_kgm3"), "label": "Air density", "unit": "kg/m^3",
     "decimals": 3, "min": 1.0, "max": 1.5,
     "note_path": ("vehicle", "aero", "air_density_note"), "accuracy_key": None,
     "short_note": "Air density for the config aero and drag terms. " + _AERO_INACTIVE},
    {"path": ("vehicle", "aero", "lift_coeff"), "label": "Lift coefficient (Cl)", "unit": "",
     "decimals": 3, "min": -3.0, "max": 3.0,
     "note_path": ("vehicle", "aero", "lift_coeff_note"), "accuracy_key": None,
     "short_note": "Config lift coefficient, negative = downforce. " + _AERO_INACTIVE},
    {"path": ("vehicle", "aero", "cross_track_area_m2"), "label": "Frontal area (cross-track)", "unit": "m^2",
     "decimals": 3, "min": 0.0, "max": 5.0,
     "note_path": ("vehicle", "aero", "cross_track_area_note"), "accuracy_key": None,
     "short_note": "Config aero reference area. " + _AERO_INACTIVE},
    {"path": ("vehicle", "aero", "diff_cog_x_m"), "label": "Aero CoP-CoG offset (x)", "unit": "m",
     "decimals": 3, "min": -2.0, "max": 2.0,
     "note_path": ("vehicle", "aero", "diff_cog_x_note"), "accuracy_key": None,
     "short_note": "Shifts config aero load between the axles. " + _AERO_INACTIVE},
]

# Section 2: analysis tunables. Corner-detection tunables left out --
# riskier than tuning analysis of an already-detected corner.
SECTION2_PARAMS_FIELDS = [
    {"path": ("stability_estimation", "moving_speed_min_mps"), "label": "Moving speed min", "unit": "m/s",
     "decimals": 2, "min": 0.0, "max": 20.0,
     "short_note": "Samples below this speed are left out of the analysis."},
    {"path": ("stability_estimation", "kerb_z_deviation_threshold_g"), "label": "Kerb z-deviation threshold", "unit": "g",
     "decimals": 2, "min": 0.0, "max": 5.0,
     "short_note": "Vertical acceleration offset from the baseline that marks a kerb hit."},
    {"path": ("stability_estimation", "kerb_baseline_g"), "label": "Kerb baseline", "unit": "g",
     "decimals": 2, "min": -2.0, "max": 2.0,
     "short_note": "Vertical acceleration of the car at rest; kerb offsets are measured from it."},
    # in seconds, not samples, so the value survives a grid-rate change
    {"path": ("stability_estimation", "kerb_dilation_s"), "label": "Kerb dilation", "unit": "s",
     "decimals": 2, "min": 0.0, "max": 0.5,
     "short_note": "Time masked either side of a kerb hit, to cover the ringdown."},
]
# setup-engine repetition rule: not an estimator input, so outside the
# cache identity (resolve_estimator_settings); a change takes effect on the
# next Generate
SECTION2_ENGINE_FLOAT_FIELDS = [
    {"path": ("setup_engine", "min_repeat_fraction"), "label": "Repeat rule: lap share", "unit": "",
     "decimals": 2, "min": 0.0, "max": 1.0,
     "short_note": "A corner-phase symptom counts when it appears on more than this share of valid laps."},
]
SECTION2_ENGINE_INT_FIELDS = [
    {"path": ("setup_engine", "min_repeat_laps"), "label": "Repeat rule: minimum laps", "unit": "",
     "min": 1, "max": 20,
     "short_note": "... and on at least this many laps."},
]
SECTION2_CHANNELS_INT_FIELDS = [
    {"path": ("corner_speed_thresholds", "low_max"), "label": "Low/medium speed boundary", "unit": "km/h",
     "min": 0, "max": 350,
     "short_note": "Apex speed separating low- from medium-speed corners."},
    {"path": ("corner_speed_thresholds", "medium_max"), "label": "Medium/high speed boundary", "unit": "km/h",
     "min": 0, "max": 350,
     "short_note": "Apex speed separating medium- from high-speed corners."},
]
# Section 3: classification thresholds, editable; an edit is recorded as a
# manual override (core/threshold_overrides.py)
SECTION3_FIELDS = [
    {"key": "STRONG_CSF", "label": "Strong front CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0,
     "short_note": "A limiting front axle (CS below zero) below this = strong understeer."},
    {"key": "STRONG_CSR", "label": "Strong rear CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0,
     "short_note": "Rear grading tier; a limiting rear axle is always strong, so it does not set the verdict."},
    {"key": "MODERATE_CSF", "label": "Moderate front CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0,
     "short_note": "Front grading tier; the understeer verdict starts at the zero boundary."},
    {"key": "MODERATE_CSR", "label": "Moderate rear CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0,
     "short_note": "Rear CS above zero but below this = rear near its peak, shown as information."},
    {"key": "stab_neg_thresh_Nm_per_deg", "label": "Stability (destabilising) threshold",
     "unit": "Nm/deg", "decimals": 1, "min": -5000.0, "max": 5000.0,
     "short_note": "Yaw stability below this = open-loop unstable."},
]
STAB_KEY = "stab_neg_thresh_Nm_per_deg"
STAB_KINEMATIC_ERA_NOTE = ("Derived default is kinematic-era, not re-derived for the current "
                           "sideslip estimator.")


def _get_path(data, path):
    node = data
    for key in path:
        node = node[key]
    return node


def _set_path(data, path, value):
    node = data
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value


def _is_placeholder_note(note):
    if not note:
        return False
    lowered = note.lower()
    return "not sourced" in lowered or "placeholder" in lowered


class SettingsView(QWidget):
    def __init__(self):
        super().__init__()
        self.section1_widgets = {}
        self.section2_widgets = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        layout.addWidget(self._build_header())

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        content = QWidget()
        self.content_layout = QVBoxLayout(content)
        self.content_layout.setContentsMargins(24, 20, 24, 24)
        self.content_layout.setSpacing(24)

        self.warning_label = QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet(f"color: {WARN}; font-size: 12px; font-weight: 600;")
        self.warning_label.setVisible(False)
        self.content_layout.addWidget(self.warning_label)

        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            initial_params = json.load(f)

        self.content_layout.addWidget(self._build_section1(initial_params))
        self.content_layout.addWidget(self._build_section2())
        self.content_layout.addWidget(self._build_section3())
        self.content_layout.addStretch()

        scroll.setWidget(content)
        layout.addWidget(scroll)

        self._load_from_disk()

    def _build_header(self):
        header = QWidget()
        header.setFixedHeight(52)
        header.setStyleSheet("border-bottom: 1px solid #222;")
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(20, 0, 20, 0)

        title = QLabel("Settings")
        title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        self.status_label = QLabel("")
        self.status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")

        btn_save = QPushButton("Save")
        btn_save.setFixedWidth(80)
        btn_save.clicked.connect(self._on_save_clicked)

        h_layout.addWidget(title)
        h_layout.addSpacing(16)
        h_layout.addWidget(self.status_label)
        h_layout.addStretch()
        h_layout.addWidget(btn_save)
        return header

    def _section_label(self, text):
        label = QLabel(text)
        label.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {ACCENT}; margin-bottom: 4px;")
        return label

    def _field_row(self, spec, widget, note_text=None, short_note=None, accuracy_text=None,
                   accuracy_tooltip=None, extra_widgets=()):
        # short_note visible (what the value does), the config note with its
        # provenance in the tooltip. Neither is written back on Save.
        is_placeholder = _is_placeholder_note(note_text)
        indent = MARKER_INDENT if is_placeholder else 0

        row = QWidget()
        row_layout = QVBoxLayout(row)
        row_layout.setContentsMargins(indent, 0, 0, 0)
        row_layout.setSpacing(2)

        top = QWidget()
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(0, 0, 0, 0)
        label_text = spec["label"] + (f" ({spec['unit']})" if spec.get("unit") else "")
        label = QLabel(label_text)
        label.setFixedWidth(LABEL_WIDTH - indent)
        label.setWordWrap(True)
        label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        top_layout.addWidget(label)
        top_layout.addWidget(widget, 0, Qt.AlignmentFlag.AlignTop)
        if accuracy_text:
            acc_label = QLabel(accuracy_text)
            acc_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; margin-left: 8px;")
            if accuracy_tooltip:
                acc_label.setToolTip(accuracy_tooltip)
            top_layout.addWidget(acc_label)
        for extra in extra_widgets:
            top_layout.addWidget(extra)
        top_layout.addStretch()
        row_layout.addWidget(top)

        if short_note:
            note_label = QLabel(short_note)
            note_label.setWordWrap(True)
            note_colour = WARN if is_placeholder else TEXT_DIM
            note_label.setStyleSheet(
                f"color: {note_colour}; font-size: 10px; margin-left: {LABEL_WIDTH - indent}px;")
            row_layout.addWidget(note_label)
            row.note_label = note_label
            if note_text:
                note_label.setToolTip(note_text)

        if note_text:
            label.setToolTip(note_text)
            widget.setToolTip(note_text)

        if is_placeholder:
            # selector-scoped: an unscoped rule cascades to every child, and each
            # label then draws its own border inside its fixed width (clipping)
            row.setObjectName("placeholderRow")
            row.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
            row.setStyleSheet(
                f"QWidget#placeholderRow {{ border-left: {MARKER_BAR_WIDTH}px solid {WARN}; }}")

        return row

    def _build_section1(self, params):
        # labels built once from file content -- Save only rewrites numeric leaves
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Vehicle Physics Constants"))

        accuracy_levels = params.get("accuracy_levels", {})
        for spec in SECTION1_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section1_widgets[spec["path"]] = widget

            note_text = _get_path(params, spec["note_path"]) if spec["note_path"] else None
            # visible tag just "L1"/"L2"/...; source / capped_by go to the tooltip
            accuracy_text = None
            accuracy_tooltip = None
            if spec["accuracy_key"]:
                entry = accuracy_levels.get(spec["accuracy_key"])
                if entry:
                    accuracy_text = f"L{entry['level']}"
                    accuracy_tooltip = entry.get("capped_by") or entry.get("source") or None

            layout.addWidget(self._field_row(
                spec, widget, note_text=note_text,
                short_note=spec.get("short_note"), accuracy_text=accuracy_text,
                accuracy_tooltip=accuracy_tooltip,
            ))

        # live aero: the session fit of the most recently analysed outing
        fitted_row = QWidget()
        fitted_layout = QHBoxLayout(fitted_row)
        fitted_layout.setContentsMargins(0, 0, 0, 0)
        fitted_label = QLabel("Session-fitted aero (read-only)")
        fitted_label.setFixedWidth(LABEL_WIDTH)
        fitted_label.setWordWrap(True)
        fitted_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self.fitted_aero_label = QLabel("")
        self.fitted_aero_label.setWordWrap(True)
        self.fitted_aero_label.setStyleSheet(f"color: {TEXT}; font-size: 11px;")
        self.fitted_aero_label.setToolTip(
            "c from total Fz = static + c*v^2 on straights (level 2); "
            "Cl*A = -2c/rho in the config aero model's sign convention.")
        fitted_layout.addWidget(fitted_label)
        fitted_layout.addWidget(self.fitted_aero_label, 1)
        layout.addWidget(fitted_row)

        return container

    def _refresh_fitted_aero(self):
        entry = latest_pipeline_entry()
        if entry is None:
            self.fitted_aero_label.setText("No outing analysed in this window yet.")
            return
        fz = entry.get("fz") or {}
        name = os.path.basename(entry.get("csv_path") or "")
        if "c_session_N_per_mps2" not in fz:
            self.fitted_aero_label.setText(f"{name}: loaded result predates this field -- re-run Analyse.")
            return
        c = fz["c_session_N_per_mps2"]
        if c is None:
            self.fitted_aero_label.setText(f"{name}: no damper data, no fit (static loads).")
            return
        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            rho = json.load(f)["vehicle"]["aero"]["air_density_kgm3"]
        self.fitted_aero_label.setText(
            f"{name}: c = {c:.3f} N/(m/s)^2, Cl*A = {effective_cl_area(c, rho):.2f} m^2")

    def showEvent(self, event):
        # the loaded outing can change while Settings is hidden
        self._refresh_fitted_aero()
        super().showEvent(event)

    def _build_section2(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Analysis Tunables"))

        for spec in SECTION2_PARAMS_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("parameters",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        for spec in SECTION2_ENGINE_FLOAT_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("parameters",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        for spec in SECTION2_ENGINE_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("parameters",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        for spec in SECTION2_CHANNELS_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("channels",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        return container

    def _build_section3(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Classification Thresholds"))

        self.section3_widgets = {}
        self.section3_manual_labels = {}
        self.section3_restore_buttons = {}
        self.section3_note_labels = {}
        for spec in SECTION3_FIELDS:
            key = spec["key"]
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)

            manual = QLabel("manual")
            manual.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; margin-left: 8px;")
            restore = QPushButton("restore derived")
            restore.setStyleSheet(
                f"background-color: transparent; color: {TEXT_MUTED}; font-size: 10px; "
                "border: none; padding: 0 4px;")
            restore.clicked.connect(lambda _checked=False, k=key: self._restore_threshold(k))

            row = self._field_row(spec, widget, short_note=spec["short_note"],
                                  extra_widgets=(manual, restore))
            self.section3_widgets[key] = widget
            self.section3_manual_labels[key] = manual
            self.section3_restore_buttons[key] = restore
            self.section3_note_labels[key] = row.note_label
            layout.addWidget(row)

        return container

    def _restore_threshold(self, key):
        # pending until Save, like every other edit on this page; Save sees the
        # derived default and records a restore
        default = self._section3_derived_defaults.get(key)
        if default is not None:
            self.section3_widgets[key].setValue(float(default))

    def _edited(self, key, widget):
        # A user edit is a widget that no longer shows what it showed after
        # loading. Comparing with the config value instead fails: the spinbox
        # rounds to its decimals (Qt rounds 0.125 to 0.13, Python's round()
        # gives 0.12), so an untouched field would count as edited and be
        # written back rounded.
        return abs(widget.value() - self._shown_on_load[key]) > 1e-9

    def _load_from_disk(self):
        # fresh file read, not the cached loaders -- show what's on disk
        # widget values right after loading, keyed like the widget dicts
        self._shown_on_load = {}
        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            params = json.load(f)
        with open(CHANNELS_PATH, encoding="utf-8") as f:
            channels = json.load(f)

        for spec in SECTION1_FIELDS:
            widget = self.section1_widgets[spec["path"]]
            widget.blockSignals(True)
            widget.setValue(float(_get_path(params, spec["path"])))
            widget.blockSignals(False)
            self._shown_on_load[spec["path"]] = widget.value()

        sources = {"parameters": params, "channels": channels}
        for key, widget in self.section2_widgets.items():
            file_key, path = key[0], key[1:]
            value = _get_path(sources[file_key], path)
            widget.blockSignals(True)
            if isinstance(widget, NoScrollIntSpinBox):
                widget.setValue(int(value))
            else:
                widget.setValue(float(value))
            widget.blockSignals(False)
            self._shown_on_load[key] = widget.value()

        cls_cfg = params["classification"]
        active_source = params["stability_estimation"].get("sideslip_source", "kinematic")
        stab_source = cls_cfg.get("stab_thresh_calibrated_for_sideslip_source", "kinematic")
        self._section3_derived_defaults = {}
        for spec in SECTION3_FIELDS:
            key = spec["key"]
            entry = cls_cfg[key]
            widget = self.section3_widgets[key]
            widget.blockSignals(True)
            widget.setValue(float(entry["value"]))
            widget.blockSignals(False)
            self._shown_on_load[key] = widget.value()
            widget.setToolTip(entry.get("derived_from", ""))
            manual = is_manual(entry)
            self._section3_derived_defaults[key] = entry.get("derived_default")
            self.section3_manual_labels[key].setVisible(manual)
            self.section3_manual_labels[key].setToolTip(
                f"Data-derived default: {entry['derived_default']}" if manual else "")
            self.section3_restore_buttons[key].setVisible(manual)
            # the stab default was derived on the kinematic estimator; the
            # verdict marker says so on firing verdicts, the row says it here
            note = spec["short_note"]
            if key == STAB_KEY and active_source != stab_source:
                note += " " + STAB_KINEMATIC_ERA_NOTE
            self.section3_note_labels[key].setText(note)

        self.warning_label.setVisible(False)
        self.status_label.setText("")

    def _on_save_clicked(self):
        # a config write while an analysis runs would race that run's cache
        # entry -> refused until the run ends (inputs are locked during a run)
        from ui.views.outing_form import analysis_running
        if analysis_running():
            self.warning_label.setText("Not saved: an analysis is running. Save again when it has finished.")
            self.warning_label.setVisible(True)
            self.status_label.setText("Not saved.")
            return
        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            params = json.load(f)
        with open(CHANNELS_PATH, encoding="utf-8") as f:
            channels = json.load(f)
        params_on_disk, channels_on_disk = copy.deepcopy(params), copy.deepcopy(channels)

        # A field is written only when the user changed it: an untouched field
        # reads back rounded to its decimals, and writing that back would edit
        # an estimator input nobody touched (_edited).
        section1_changed = False
        for spec in SECTION1_FIELDS:
            widget = self.section1_widgets[spec["path"]]
            if not self._edited(spec["path"], widget):
                continue
            section1_changed = True
            _set_path(params, spec["path"], widget.value())

        from modules.accuracy_resolution import resolve_estimator_settings
        estimator_before = resolve_estimator_settings(params, channels)
        sources = {"parameters": params, "channels": channels}
        for key, widget in self.section2_widgets.items():
            if not self._edited(key, widget):
                continue
            value = widget.value()
            if isinstance(widget, NoScrollIntSpinBox):
                value = int(value)
            _set_path(sources[key[0]], key[1:], value)
        # these inputs are part of every cache identity; a change makes the
        # stored results stale until the next Analyse
        estimator_changed = resolve_estimator_settings(params, channels) != estimator_before

        today = datetime.date.today().isoformat()
        thresholds_changed = False
        for spec in SECTION3_FIELDS:
            entry = params["classification"][spec["key"]]
            widget = self.section3_widgets[spec["key"]]
            # an untouched row must not turn a finer-grained derived value
            # into a manual override
            if not self._edited(spec["key"], widget):
                continue
            outcome = apply_threshold_edit(entry, widget.value(), today)
            thresholds_changed |= outcome != "unchanged"

        # A file is rewritten only when a value in it changed: json.dump
        # reflows the hand-kept layout (0.10 -> 0.1, inline arrays split), which
        # changes nothing the app reads but every byte hash of the file (the
        # thesis-figure config_hash).
        # newline="" -- text mode on Windows would turn every LF into CRLF
        written = []
        for path, data, on_disk in ((PARAMETERS_PATH, params, params_on_disk),
                                    (CHANNELS_PATH, channels, channels_on_disk)):
            if data == on_disk:
                continue
            with open(path, "w", encoding="utf-8", newline="") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
            written.append(path)
        if not written:
            self._load_from_disk()
            self.status_label.setText("Nothing changed, nothing saved.")
            return

        from modules.stability_analysis import load_parameters, load_car_data
        load_parameters.cache_clear()
        load_car_data.cache_clear()

        # persisted stores are covered by their cache identities; this drops the
        # in-memory pipeline cache of any open OutingForm that predates the save
        from ui.views.outing_form import invalidate_all_pipeline_caches
        invalidate_all_pipeline_caches()

        messages = []
        if section1_changed:
            messages.append("Physics constants changed - results will differ. Re-run Analyse.")
        if estimator_changed:
            messages.append("Estimator inputs changed (moving speed, kerb detection or corner speed "
                            "boundaries) - results will differ. Re-run Analyse.")
        if thresholds_changed:
            # verdicts are classified live from config, never stored
            messages.append("Thresholds changed - verdicts use them on the next render or Generate.")
        self.warning_label.setText(" ".join(messages))
        self.warning_label.setVisible(bool(messages))

        warning_text, warning_visible = self.warning_label.text(), self.warning_label.isVisible()
        self._load_from_disk()
        self.warning_label.setText(warning_text)
        self.warning_label.setVisible(warning_visible)
        self.status_label.setText("Saved.")
