# Settings page: vehicle constants, analysis tunables and thresholds
# (parameters.json), speed-class thresholds (channels.json), rule-engine
# settings (recommendations.json), cost weights (decision_frame.json).
# Only reads/writes the JSON and clears the loader caches.

import datetime
import json
import os

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QScrollArea,
    QLabel, QPushButton, QFrame,
)
from PyQt6.QtCore import Qt

from ui.style import ACCENT, WARN, TEXT, TEXT_MUTED, TEXT_DIM, PANEL, PANEL_ALT, BORDER
from ui.views.outing_form import NoScrollSpinBox, NoScrollIntSpinBox, latest_pipeline_entry
from core.threshold_overrides import apply_threshold_edit, is_manual
from modules.wheel_loads import effective_cl_area
from modules.decision_frame import TPMS_COMPOUND_CAVEAT

PARAMETERS_PATH = "config/parameters.json"
CHANNELS_PATH = "config/channels.json"
RECOMMENDATIONS_PATH = "config/recommendations.json"
DECISION_FRAME_PATH = "config/decision_frame.json"

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
]
SECTION2_PARAMS_INT_FIELDS = [
    {"path": ("stability_estimation", "kerb_dilation_samples"), "label": "Kerb dilation", "unit": "samples",
     "min": 0, "max": 50,
     "short_note": "Samples masked either side of a kerb hit, to cover the ringdown."},
]
SECTION2_CHANNELS_INT_FIELDS = [
    {"path": ("corner_speed_thresholds", "low_max"), "label": "Low/medium speed boundary", "unit": "km/h",
     "min": 0, "max": 350,
     "short_note": "Apex speed separating low- from medium-speed corners."},
    {"path": ("corner_speed_thresholds", "medium_max"), "label": "Medium/high speed boundary", "unit": "km/h",
     "min": 0, "max": 350,
     "short_note": "Apex speed separating medium- from high-speed corners."},
]
SECTION2_RECS_INT_FIELDS = [
    {"path": ("settings", "consistency_gate", "min_repeat_laps"), "label": "Consistency gate: min repeat laps", "unit": "",
     "min": 0, "max": 10,
     "short_note": "Weekend report: laps a verdict must repeat on before a rule fires."},
    {"path": ("settings", "change_budget", "default_max"), "label": "Change budget: default max", "unit": "",
     "min": 0, "max": 10,
     "short_note": "Weekend report: number of changes marked as selected."},
    {"path": ("settings", "change_budget", "absolute_cap"), "label": "Change budget: absolute cap", "unit": "",
     "min": 0, "max": 10,
     "short_note": "Weekend report: upper limit reserved for a manual override."},
]
SECTION2_RECS_FLOAT_FIELDS = [
    {"path": ("settings", "consistency_gate", "min_repeat_fraction"), "label": "Consistency gate: min repeat fraction", "unit": "",
     "decimals": 2, "min": 0.0, "max": 1.0,
     "short_note": "Weekend report: share of laps a verdict must repeat on."},
    {"path": ("settings", "driver_level_weighting", "default_weight"), "label": "Driver weighting: default weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 3.0,
     "short_note": "Weekend report: feedback weight for a driver level missing from the table."},
]
DRIVER_WEIGHT_LEVELS = [str(i) for i in range(1, 11)]

# Section 4: decision-frame cost weights, saved with the others. Ranking
# only -- generate_candidates never reads cost_function, only score() does
# (tested).
SECTION4_FIELDS = [
    {"path": ("cost_function", "severity"), "label": "Severity weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "derived_from"),
     "short_note": "How much problem severity (and confidence) drives ranking."},
    {"path": ("cost_function", "change_time", "seconds"), "label": "Change time: seconds", "unit": "",
     "decimals": 4, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "change_time_derived_from"),
     "short_note": "Score for a change done in seconds (in-car electronics or a quick click)."},
    {"path": ("cost_function", "change_time", "minutes"), "label": "Change time: minutes", "unit": "",
     "decimals": 4, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "change_time_derived_from"),
     "short_note": "Score for a change made in the pit box in minutes."},
    {"path": ("cost_function", "change_time", "half_hour"), "label": "Change time: half hour", "unit": "",
     "decimals": 4, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "change_time_derived_from"),
     "short_note": "Score for a change taking about half an hour."},
    {"path": ("cost_function", "change_time", "garage_hours"), "label": "Change time: garage hours", "unit": "",
     "decimals": 4, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "change_time_derived_from"),
     "short_note": "Score for a change that needs the car in the garage."},
    {"path": ("cost_function", "breadth"), "label": "Breadth weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "breadth_derived_from"),
     "short_note": "Penalises a global lever that helps only some assessed corners (0 = annotation only)."},
    {"path": ("cost_function", "headroom"), "label": "Headroom weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "headroom_derived_from"),
     "short_note": "How much settings-window distance from nominal matters."},
    {"path": ("cost_function", "interaction"), "label": "Interaction weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "interaction_derived_from"),
     "short_note": "Penalises adverse side-effects on other active problems at the same corner."},
    {"path": ("cost_function", "effect_class", "primary"), "label": "Effect class: primary", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "effect_class_derived_from"),
     "short_note": "Multiplier for a primary (matrix-exact) candidate."},
    {"path": ("cost_function", "effect_class", "secondary"), "label": "Effect class: secondary", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "effect_class_derived_from"),
     "short_note": "Multiplier for a secondary (side-effect) candidate."},
]


# Section 5: tyre-pressure target bands (check only, never recommended).
# The config keeps one band per wheel; the elicited band is per axle, so
# each row writes both wheels of its axle.
SECTION5_FIELDS = [
    {"label": "Front band min", "unit": "bar", "key": "min_bar", "wheels": ("fl", "fr")},
    {"label": "Front band max", "unit": "bar", "key": "max_bar", "wheels": ("fl", "fr")},
    {"label": "Rear band min", "unit": "bar", "key": "min_bar", "wheels": ("rl", "rr")},
    {"label": "Rear band max", "unit": "bar", "key": "max_bar", "wheels": ("rl", "rr")},
]


def _inverted_tyre_bands(values):
    # values: {(wheels, "min_bar"|"max_bar"): bar} -> axle names whose min > max
    axles = {("fl", "fr"): "front", ("rl", "rr"): "rear"}
    return [name for wheels, name in axles.items()
            if values.get((wheels, "min_bar"), 0.0) > values.get((wheels, "max_bar"), float("inf"))]


# Section 3: classification thresholds, editable; an edit is recorded as a
# manual override (core/threshold_overrides.py)
SECTION3_FIELDS = [
    {"key": "STRONG_CSF", "label": "Strong front CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0, "short_note": "Front CS ratio below this = strong understeer."},
    {"key": "STRONG_CSR", "label": "Strong rear CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0, "short_note": "Rear CS ratio below this = strong oversteer."},
    {"key": "MODERATE_CSF", "label": "Moderate front CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0, "short_note": "Front CS ratio below this = moderate understeer."},
    {"key": "MODERATE_CSR", "label": "Moderate rear CS threshold", "unit": "", "decimals": 3,
     "min": -2.0, "max": 2.0, "short_note": "Rear CS ratio below this = moderate oversteer."},
    {"key": "stab_neg_thresh_Nm_per_deg", "label": "Stability (destabilising) threshold",
     "unit": "Nm/deg", "decimals": 1, "min": -5000.0, "max": 5000.0,
     "short_note": "Yaw stability below this = unstable yaw."},
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
        self.driver_weight_widgets = {}
        self.section4_widgets = {}

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

        with open(DECISION_FRAME_PATH, encoding="utf-8") as f:
            initial_decision_frame = json.load(f)

        self.content_layout.addWidget(self._build_section1(initial_params))
        self.content_layout.addWidget(self._build_section2())
        self.content_layout.addWidget(self._build_section3())
        self.content_layout.addWidget(self._build_section4(initial_decision_frame))
        self.content_layout.addWidget(self._build_section5(initial_decision_frame))
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

    def _divider(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(f"color: {BORDER};")
        return line

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

        for spec in SECTION2_PARAMS_INT_FIELDS:
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

        for spec in SECTION2_RECS_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("recommendations",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        for spec in SECTION2_RECS_FLOAT_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("recommendations",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget, short_note=spec.get("short_note")))

        layout.addWidget(self._build_driver_weight_table())

        return container

    def _build_driver_weight_table(self):
        row = QWidget()
        row_layout = QVBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(2)

        label = QLabel("Driver-level feedback weight table (level 1-10)")
        label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        row_layout.addWidget(label)

        grid = QWidget()
        grid_layout = QHBoxLayout(grid)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setSpacing(6)
        for level in DRIVER_WEIGHT_LEVELS:
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(1)
            level_label = QLabel(level)
            level_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            level_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 9px;")
            spin = NoScrollSpinBox()
            spin.setDecimals(2)
            spin.setRange(0.0, 3.0)
            spin.setFixedWidth(56)
            self.driver_weight_widgets[level] = spin
            cell_layout.addWidget(level_label)
            cell_layout.addWidget(spin)
            grid_layout.addWidget(cell)
        grid_layout.addStretch()
        row_layout.addWidget(grid)

        note = QLabel("Weekend report: how much a driver's feedback counts, by driving level.")
        note.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; margin-left: {LABEL_WIDTH}px;")
        note.setWordWrap(True)
        note.setToolTip("Elicited from the project lead -- not yet validated.")
        row_layout.addWidget(note)

        return row

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

    def _build_section4(self, decision_frame):
        # notes baked in once, as in _build_section1; only numbers are edited
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Decision-Frame Scoring Weights"))

        rule_note = QLabel(
            "Re-rank the shortlist only -- never touch verdicts, evidence, or which "
            "candidates are generated."
        )
        rule_note.setWordWrap(True)
        rule_note.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
        layout.addWidget(rule_note)

        for spec in SECTION4_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section4_widgets[spec["path"]] = widget

            note_text = _get_path(decision_frame, spec["note_path"]) if spec.get("note_path") else None
            layout.addWidget(self._field_row(spec, widget, note_text=note_text,
                                              short_note=spec.get("short_note")))

        return container

    def _build_section5(self, decision_frame):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Tyre Pressure Target (check only)"))

        caveat = QLabel(TPMS_COMPOUND_CAVEAT[0].upper() + TPMS_COMPOUND_CAVEAT[1:] + ".")
        caveat.setWordWrap(True)
        caveat.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
        layout.addWidget(caveat)

        full_note = decision_frame.get("tyre_pressure_target", {}).get("compound_note")
        # the sensor's plausible range, the same bounds the flag uses for glitches
        with open(CHANNELS_PATH, encoding="utf-8") as f:
            lo_bar, hi_bar = json.load(f)["channels"]["tpms_press_fl"]["range"]
        self.section5_widgets = []
        for spec in SECTION5_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(2)
            widget.setRange(lo_bar, hi_bar)
            widget.setFixedWidth(120)
            self.section5_widgets.append((spec, widget))
            layout.addWidget(self._field_row(spec, widget, note_text=full_note))
        return container

    def _load_from_disk(self):
        # fresh file read, not the cached loaders -- show what's on disk
        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            params = json.load(f)
        with open(CHANNELS_PATH, encoding="utf-8") as f:
            channels = json.load(f)
        with open(RECOMMENDATIONS_PATH, encoding="utf-8") as f:
            recs = json.load(f)
        with open(DECISION_FRAME_PATH, encoding="utf-8") as f:
            decision_frame = json.load(f)

        for spec in SECTION1_FIELDS:
            widget = self.section1_widgets[spec["path"]]
            widget.blockSignals(True)
            widget.setValue(float(_get_path(params, spec["path"])))
            widget.blockSignals(False)

        for spec in SECTION4_FIELDS:
            widget = self.section4_widgets[spec["path"]]
            widget.blockSignals(True)
            widget.setValue(float(_get_path(decision_frame, spec["path"])))
            widget.blockSignals(False)

        sources = {"parameters": params, "channels": channels, "recommendations": recs}
        for key, widget in self.section2_widgets.items():
            file_key, path = key[0], key[1:]
            value = _get_path(sources[file_key], path)
            widget.blockSignals(True)
            if isinstance(widget, NoScrollIntSpinBox):
                widget.setValue(int(value))
            else:
                widget.setValue(float(value))
            widget.blockSignals(False)

        targets = decision_frame["tyre_pressure_target"]
        for spec, widget in self.section5_widgets:
            widget.blockSignals(True)
            widget.setValue(float(targets[spec["wheels"][0]][spec["key"]]))
            widget.blockSignals(False)

        weights = recs["settings"]["driver_level_weighting"]["weights"]
        for level, widget in self.driver_weight_widgets.items():
            widget.blockSignals(True)
            widget.setValue(float(weights[level]))
            widget.blockSignals(False)

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
        # an inverted band would mark every wheel off-band; refuse the whole
        # save so nothing half-persists
        inverted = _inverted_tyre_bands(
            {(spec["wheels"], spec["key"]): widget.value() for spec, widget in self.section5_widgets})
        if inverted:
            self.warning_label.setText(
                f"Not saved: tyre band min above max ({', '.join(inverted)}). Fix and save again.")
            self.warning_label.setVisible(True)
            self.status_label.setText("Not saved.")
            return

        with open(PARAMETERS_PATH, encoding="utf-8") as f:
            params = json.load(f)
        with open(CHANNELS_PATH, encoding="utf-8") as f:
            channels = json.load(f)
        with open(RECOMMENDATIONS_PATH, encoding="utf-8") as f:
            recs = json.load(f)
        with open(DECISION_FRAME_PATH, encoding="utf-8") as f:
            decision_frame = json.load(f)

        section1_changed = False
        for spec in SECTION1_FIELDS:
            widget = self.section1_widgets[spec["path"]]
            old_value = _get_path(params, spec["path"])
            new_value = widget.value()
            if abs(float(old_value) - new_value) > 1e-9:
                section1_changed = True
            _set_path(params, spec["path"], new_value)

        sources = {"parameters": params, "channels": channels, "recommendations": recs}
        for key, widget in self.section2_widgets.items():
            file_key, path = key[0], key[1:]
            value = widget.value()
            if isinstance(widget, NoScrollIntSpinBox):
                value = int(value)
            _set_path(sources[file_key], path, value)

        for level, widget in self.driver_weight_widgets.items():
            recs["settings"]["driver_level_weighting"]["weights"][level] = widget.value()

        for spec in SECTION4_FIELDS:
            widget = self.section4_widgets[spec["path"]]
            _set_path(decision_frame, spec["path"], widget.value())

        for spec, widget in self.section5_widgets:
            for wheel in spec["wheels"]:
                decision_frame["tyre_pressure_target"][wheel][spec["key"]] = round(widget.value(), 2)

        today = datetime.date.today().isoformat()
        thresholds_changed = False
        for spec in SECTION3_FIELDS:
            entry = params["classification"][spec["key"]]
            new_value = self.section3_widgets[spec["key"]].value()
            # the spinbox rounds to its decimals; an untouched row must not
            # turn a finer-grained derived value into a manual override
            if abs(new_value - round(float(entry["value"]), spec["decimals"])) < 1e-9:
                continue
            outcome = apply_threshold_edit(entry, new_value, today)
            thresholds_changed |= outcome != "unchanged"

        # newline="" -- text mode on Windows would turn every LF into CRLF
        with open(PARAMETERS_PATH, "w", encoding="utf-8", newline="") as f:
            json.dump(params, f, indent=2)
            f.write("\n")
        with open(CHANNELS_PATH, "w", encoding="utf-8", newline="") as f:
            json.dump(channels, f, indent=2)
            f.write("\n")
        with open(RECOMMENDATIONS_PATH, "w", encoding="utf-8", newline="") as f:
            json.dump(recs, f, indent=2)
            f.write("\n")
        with open(DECISION_FRAME_PATH, "w", encoding="utf-8", newline="") as f:
            json.dump(decision_frame, f, indent=2)
            f.write("\n")

        from modules.stability_analysis import load_parameters, load_car_data
        load_parameters.cache_clear()
        load_car_data.cache_clear()
        # load_decision_frame_config isn't cached -> nothing to clear

        # safety net for an open OutingForm whose in-memory pipeline cache predates
        # this save (the snapshot comparison covers everything else)
        from ui.views.outing_form import invalidate_all_pipeline_caches
        invalidate_all_pipeline_caches()

        messages = []
        if section1_changed:
            messages.append("Physics constants changed - results will differ. Re-run Analyse.")
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
