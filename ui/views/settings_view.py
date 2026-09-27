# Settings page: vehicle constants, analysis tunables and thresholds
# (parameters.json), speed-class thresholds (channels.json), rule-engine
# settings (recommendations.json), cost weights (decision_frame.json).
# Only reads/writes the JSON and clears the loader caches.

import json
import re

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QScrollArea,
    QLabel, QPushButton, QFrame,
)
from PyQt6.QtCore import Qt

from ui.style import ACCENT, WARN, TEXT, TEXT_MUTED, TEXT_DIM, PANEL, PANEL_ALT, BORDER
from ui.views.outing_form import NoScrollSpinBox, NoScrollIntSpinBox

PARAMETERS_PATH = "config/parameters.json"
CHANNELS_PATH = "config/channels.json"
RECOMMENDATIONS_PATH = "config/recommendations.json"
DECISION_FRAME_PATH = "config/decision_frame.json"

# Section 1: vehicle constants. corner_weights and cog_to_*_axle_m left
# out -- they have per-session resolution (accuracy_resolution).
SECTION1_FIELDS = [
    {"path": ("vehicle", "mass_kg"), "label": "Mass", "unit": "kg",
     "decimals": 1, "min": 500.0, "max": 2000.0,
     "note_path": ("vehicle", "mass_note"), "accuracy_key": "mass",
     "short_note": "Car weight, with driver and fuel reference weights."},
    {"path": ("vehicle", "cog_height_m"), "label": "CoG height", "unit": "m",
     "decimals": 3, "min": 0.05, "max": 1.0,
     "note_path": ("vehicle", "cog_height_note"), "accuracy_key": None,
     "short_note": "Centre of gravity height. Estimate."},
    {"path": ("vehicle", "track_width_front_m"), "label": "Track width front", "unit": "m",
     "decimals": 3, "min": 0.8, "max": 2.2,
     "note_path": ("vehicle", "track_width_note"), "accuracy_key": None,
     "short_note": "Front track width. Estimate."},
    {"path": ("vehicle", "track_width_rear_m"), "label": "Track width rear", "unit": "m",
     "decimals": 3, "min": 0.8, "max": 2.2,
     "note_path": ("vehicle", "track_width_note"), "accuracy_key": None,
     "short_note": "Rear track width. Estimate."},
    {"path": ("vehicle", "wheelbase_m"), "label": "Wheelbase", "unit": "m",
     "decimals": 3, "min": 1.5, "max": 3.5,
     "note_path": None, "accuracy_key": "wheelbase_m"},
    {"path": ("vehicle", "yaw_inertia_kgm2"), "label": "Yaw inertia (Iz)", "unit": "kg*m^2",
     "decimals": 1, "min": 500.0, "max": 5000.0,
     "note_path": ("vehicle", "yaw_inertia_note"), "accuracy_key": "yaw_inertia",
     "short_note": "Rotational inertia about the vertical axis. Estimate."},
    {"path": ("vehicle", "steering_ratio"), "label": "Steering ratio (constant)", "unit": "",
     "decimals": 2, "min": 5.0, "max": 25.0,
     "note_path": ("vehicle", "steering_ratio_note"), "accuracy_key": "steering_ratio",
     "short_note": "Steering wheel to road wheel ratio, held constant."},
    {"path": ("vehicle", "aero", "air_density_kgm3"), "label": "Air density", "unit": "kg/m^3",
     "decimals": 3, "min": 1.0, "max": 1.5,
     "note_path": ("vehicle", "aero", "air_density_note"), "accuracy_key": None,
     "short_note": "Standard sea-level air density."},
    {"path": ("vehicle", "aero", "lift_coeff"), "label": "Lift coefficient (Cl)", "unit": "",
     "decimals": 3, "min": -3.0, "max": 3.0,
     "note_path": ("vehicle", "aero", "lift_coeff_note"), "accuracy_key": None,
     "short_note": "Aero lift coefficient. Estimate."},
    {"path": ("vehicle", "aero", "cross_track_area_m2"), "label": "Frontal area (cross-track)", "unit": "m^2",
     "decimals": 3, "min": 0.0, "max": 5.0,
     "note_path": ("vehicle", "aero", "cross_track_area_note"), "accuracy_key": None,
     "short_note": "Frontal area. Estimate."},
    {"path": ("vehicle", "aero", "diff_cog_x_m"), "label": "Aero CoP-CoG offset (x)", "unit": "m",
     "decimals": 3, "min": -2.0, "max": 2.0,
     "note_path": ("vehicle", "aero", "diff_cog_x_note"), "accuracy_key": None,
     "short_note": "Aero centre-of-pressure offset from CoG. Estimate."},
]

# Section 2: analysis tunables. Corner-detection tunables left out --
# riskier than tuning analysis of an already-detected corner.
SECTION2_PARAMS_FIELDS = [
    {"path": ("stability_estimation", "moving_speed_min_mps"), "label": "Moving speed min", "unit": "m/s",
     "decimals": 2, "min": 0.0, "max": 20.0},
    {"path": ("stability_estimation", "kerb_z_deviation_threshold_g"), "label": "Kerb z-deviation threshold", "unit": "g",
     "decimals": 2, "min": 0.0, "max": 5.0},
    {"path": ("stability_estimation", "kerb_baseline_g"), "label": "Kerb baseline", "unit": "g",
     "decimals": 2, "min": -2.0, "max": 2.0},
]
SECTION2_PARAMS_INT_FIELDS = [
    {"path": ("stability_estimation", "kerb_dilation_samples"), "label": "Kerb dilation", "unit": "samples",
     "min": 0, "max": 50},
]
SECTION2_CHANNELS_INT_FIELDS = [
    {"path": ("corner_speed_thresholds", "low_max"), "label": "Low/medium speed boundary", "unit": "km/h",
     "min": 0, "max": 350},
    {"path": ("corner_speed_thresholds", "medium_max"), "label": "Medium/high speed boundary", "unit": "km/h",
     "min": 0, "max": 350},
]
SECTION2_RECS_INT_FIELDS = [
    {"path": ("settings", "consistency_gate", "min_repeat_laps"), "label": "Consistency gate: min repeat laps", "unit": "",
     "min": 0, "max": 10},
    {"path": ("settings", "change_budget", "default_max"), "label": "Change budget: default max", "unit": "",
     "min": 0, "max": 10},
    {"path": ("settings", "change_budget", "absolute_cap"), "label": "Change budget: absolute cap", "unit": "",
     "min": 0, "max": 10},
    {"path": ("settings", "driver_level_weighting", "neutral_level"), "label": "Driver weighting: neutral level", "unit": "",
     "min": 1, "max": 10},
]
SECTION2_RECS_FLOAT_FIELDS = [
    {"path": ("settings", "consistency_gate", "min_repeat_fraction"), "label": "Consistency gate: min repeat fraction", "unit": "",
     "decimals": 2, "min": 0.0, "max": 1.0},
    {"path": ("settings", "driver_level_weighting", "default_weight"), "label": "Driver weighting: default weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 3.0},
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
    {"path": ("cost_function", "change_time"), "label": "Change-time weight", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "derived_from"),
     "short_note": "Cheaper/faster changes rank higher (inverse effort)."},
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
     "short_note": "Penalises adverse side-effects on other active problems (interaction_table)."},
    {"path": ("cost_function", "effect_class", "primary"), "label": "Effect class: primary", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "effect_class_derived_from"),
     "short_note": "Multiplier for a primary (matrix-exact) candidate."},
    {"path": ("cost_function", "effect_class", "secondary"), "label": "Effect class: secondary", "unit": "",
     "decimals": 2, "min": 0.0, "max": 10.0,
     "note_path": ("cost_function", "effect_class_derived_from"),
     "short_note": "Multiplier for a secondary (side-effect) candidate."},
    {"path": ("display_score_threshold", "value"), "label": "Display cutoff score", "unit": "",
     "decimals": 2, "min": 0.0, "max": 20.0,
     "note_path": ("display_score_threshold", "derived_from"),
     "short_note": "Below this score, a candidate moves from the visible shortlist into the collapsed tail."},
]


# Section 3: classification thresholds, read-only
SECTION3_FIELDS = [
    ("STRONG_CSF", "Strong front CS threshold"),
    ("STRONG_CSR", "Strong rear CS threshold"),
    ("MODERATE_CSF", "Moderate front CS threshold"),
    ("MODERATE_CSR", "Moderate rear CS threshold"),
    ("stab_neg_thresh_Nm_per_deg", "Stability (destabilising) threshold"),
]


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


_DATE_RE = re.compile(r"\d{4}-\d{2}-\d{2}")


def _short_derived_from(text):
    # derived_from strings are full audit trails -> tooltip; visible line
    # keeps only the latest date (strings are append-only)
    dates = _DATE_RE.findall(text or "")
    if dates:
        return f"Derived from data, last confirmed {max(dates)} - read-only."
    return "Derived from data - read-only."


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
                   accuracy_tooltip=None):
        # full note_text in the tooltip, short_note visible (what, unit, "estimate
        # - replace with team figure"). Neither is written back on Save.
        row = QWidget()
        row_layout = QVBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(2)

        top = QWidget()
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(0, 0, 0, 0)
        label_text = spec["label"] + (f" ({spec['unit']})" if spec.get("unit") else "")
        label = QLabel(label_text)
        label.setFixedWidth(240)
        label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        top_layout.addWidget(label)
        top_layout.addWidget(widget)
        if accuracy_text:
            acc_label = QLabel(accuracy_text)
            acc_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; margin-left: 8px;")
            if accuracy_tooltip:
                acc_label.setToolTip(accuracy_tooltip)
            top_layout.addWidget(acc_label)
        top_layout.addStretch()
        row_layout.addWidget(top)

        is_placeholder = _is_placeholder_note(note_text)
        if short_note:
            note_label = QLabel(short_note)
            note_label.setWordWrap(True)
            note_colour = WARN if is_placeholder else TEXT_DIM
            note_label.setStyleSheet(f"color: {note_colour}; font-size: 10px; margin-left: 240px;")
            row_layout.addWidget(note_label)
            if note_text:
                note_label.setToolTip(note_text)

        if note_text:
            label.setToolTip(note_text)
            widget.setToolTip(note_text)

        if is_placeholder:
            row.setStyleSheet(f"border-left: 2px solid {WARN};")

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

        return container

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
            layout.addWidget(self._field_row(spec, widget))

        for spec in SECTION2_PARAMS_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("parameters",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget))

        for spec in SECTION2_CHANNELS_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("channels",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget))

        for spec in SECTION2_RECS_INT_FIELDS:
            widget = NoScrollIntSpinBox()
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("recommendations",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget))

        for spec in SECTION2_RECS_FLOAT_FIELDS:
            widget = NoScrollSpinBox()
            widget.setDecimals(spec["decimals"])
            widget.setRange(spec["min"], spec["max"])
            widget.setFixedWidth(120)
            self.section2_widgets[("recommendations",) + spec["path"]] = widget
            layout.addWidget(self._field_row(spec, widget))

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

        note = QLabel("How much a driver's feedback counts, by experience level -- not yet validated.")
        note.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        note.setToolTip("Elicited from the project lead -- not yet validated.")
        row_layout.addWidget(note)

        return row

    def _build_section3(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._section_label("Classification Thresholds (read-only)"))

        rule_note = QLabel(
            "Thresholds are always re-derived from this car's own data, never "
            "hand-edited -- read-only here by design."
        )
        rule_note.setWordWrap(True)
        rule_note.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-style: italic;")
        layout.addWidget(rule_note)

        self.section3_value_labels = {}
        self.section3_derived_labels = {}
        for key, label_text in SECTION3_FIELDS:
            row = QWidget()
            row_layout = QVBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(2)

            top = QWidget()
            top_layout = QHBoxLayout(top)
            top_layout.setContentsMargins(0, 0, 0, 0)
            name_label = QLabel(label_text)
            name_label.setFixedWidth(240)
            name_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
            value_label = QLabel("")
            value_label.setStyleSheet(f"color: {TEXT}; font-size: 12px; font-weight: 600;")
            top_layout.addWidget(name_label)
            top_layout.addWidget(value_label)
            top_layout.addStretch()
            row_layout.addWidget(top)

            derived_label = QLabel("")
            derived_label.setWordWrap(True)
            derived_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; margin-left: 240px;")
            row_layout.addWidget(derived_label)

            self.section3_value_labels[key] = value_label
            self.section3_derived_labels[key] = derived_label
            layout.addWidget(row)

        return container

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

        weights = recs["settings"]["driver_level_weighting"]["weights"]
        for level, widget in self.driver_weight_widgets.items():
            widget.blockSignals(True)
            widget.setValue(float(weights[level]))
            widget.blockSignals(False)

        cls_cfg = params["classification"]
        for key, _label in SECTION3_FIELDS:
            entry = cls_cfg[key]
            self.section3_value_labels[key].setText(f"{entry['value']}")
            self.section3_derived_labels[key].setText(_short_derived_from(entry["derived_from"]))
            self.section3_derived_labels[key].setToolTip(entry["derived_from"])

        self.warning_label.setVisible(False)
        self.status_label.setText("")

    def _on_save_clicked(self):
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

        if section1_changed:
            self.warning_label.setText(
                "Physics constants changed - results will differ. Re-run Analyse."
            )
            self.warning_label.setVisible(True)
        else:
            self.warning_label.setVisible(False)

        self._load_from_disk()
        self.status_label.setText("Saved.")
