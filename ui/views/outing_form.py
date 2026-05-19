# Outing form — full form for creating a new outing.

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QScrollArea, QPushButton,
    QLineEdit, QComboBox, QTextEdit,
    QDateTimeEdit, QDoubleSpinBox,
    QGroupBox, QFrame
)
from PyQt6.QtCore import Qt, QDateTime
from models.base import Session
from models.driver import Driver
from models.outing import Outing
from core.config_loader import get_setup_parameters

class NoScrollSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class OutingForm(QWidget):
    def __init__(self, weekend, on_back, outing=None):
        super().__init__()
        self.weekend = weekend
        self.on_back = on_back
        self.outing = outing

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
        self.content_layout.addWidget(self._build_setup_section())
        self.content_layout.addWidget(self._build_feedback_section())
        self.content_layout.addWidget(self._build_comments_section())
        self.content_layout.addStretch()

        if self.outing:
            self._prefill()
        else:
            self._carryon_from_last()

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

    def _build_header(self):
        header = QWidget()
        header.setFixedHeight(52)
        header.setStyleSheet("border-bottom: 1px solid #222;")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(20, 0, 20, 0)

        btn_back = QPushButton("← Back")
        btn_back.setFixedWidth(80)
        btn_back.setStyleSheet("background-color: #252525; color: #888;")
        btn_back.clicked.connect(self._save_outing)

        title = QLabel(f"New Outing — {self.weekend.track}")
        title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        layout.addWidget(btn_back)
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
        self.air_temp_input.setSuffix(" °C")
        self.air_temp_input.setRange(-20, 80)
        self.air_temp_input.setDecimals(1)

        self.track_temp_input = NoScrollSpinBox()
        self.track_temp_input.setSuffix(" °C")
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

        btn_load = QPushButton("Load CSV")
        btn_load.setFixedWidth(120)

        placeholder = QLabel("No file loaded")
        placeholder.setStyleSheet("color: #555; font-size: 12px;")

        layout.addWidget(btn_load)
        layout.addWidget(placeholder)

        return section

    def _build_setup_section(self):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)
        layout.addWidget(self._section_label("Car Setup"))

        params = get_setup_parameters()
        self.setup_inputs = {}

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

        # full width notes
        notes_label = QLabel("Setup Notes")
        notes_label.setStyleSheet("color: #888; font-size: 11px; margin-top: 8px;")
        self.setup_inputs["car"]["notes"] = QTextEdit()
        self.setup_inputs["car"]["notes"].setMinimumHeight(100)
        self.setup_inputs["car"]["notes"].setPlaceholderText("Kinematic info, special configurations, general setup notes...")
        self.setup_inputs["car"]["notes"].setStyleSheet("""
            QTextEdit {
                background-color: #1a1a1a;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                padding: 8px;
                color: #e0e0e0;
                font-size: 12px;
            }
        """)
        layout.addWidget(notes_label)
        layout.addWidget(self.setup_inputs["car"]["notes"])

        return section

        

    def _build_corner_block(self, corner_label, params):
        corner_key = {"FL": "front_left", "FR": "front_right",
                      "RL": "rear_left", "RR": "rear_right"}[corner_label]

        if corner_key not in self.setup_inputs:
            self.setup_inputs[corner_key] = {}

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
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
            }
        """)

        layout = QVBoxLayout(group)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 12, 8, 8)

        always_visible = ["toe", "camber", "ride_height_fia", "ride_height_aero", "arb", "springs"]
        advanced_fields = ["packer", "preload", "total_travel", "free_length", "static_droop", "gap_on_gnd"]

        labels = {
            "toe": "Toe (mm)", "camber": "Camber (°)",
            "ride_height_fia": "Ride Ht. FIA (mm)", "ride_height_aero": "Ride Ht. Aero (mm)",
            "arb": "ARB (pos.)", "springs": "Springs (N/mm)",
            "bump_ls": "Bump LS", "bump_hs": "Bump HS",
            "blowoff": "Blowoff",
            "rebound_ls": "Reb LS", "rebound_hs": "Reb HS",
            "packer": "Packer (mm)", "preload": "Preload (mm)",
            "total_travel": "Total Travel (mm)", "free_length": "Free Length (mm)",
            "static_droop": "Static Droop (mm)", "gap_on_gnd": "Gap on GND (mm)"
        }

        for param in always_visible:
            widget = NoScrollSpinBox()
            widget.setRange(-9999, 9999)
            widget.setDecimals(2)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self.setup_inputs[corner_key][param] = widget
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
            self.setup_inputs[corner_key][param] = widget
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
        self.setup_inputs[corner_key]["blowoff"] = blowoff_widget
        blowoff_cell_layout.addWidget(blowoff_lbl)
        blowoff_cell_layout.addWidget(blowoff_widget)
        layout.addWidget(blowoff_cell)

        reb_row = QWidget()
        reb_layout = QHBoxLayout(reb_row)
        reb_layout.setContentsMargins(0, 0, 0, 0)
        reb_layout.setSpacing(8)
        for param, label_text in [("rebound_ls", "Reb LS"), ("rebound_hs", "Reb HS")]:
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2)
            lbl = QLabel(label_text)
            lbl.setStyleSheet("color: #555; font-size: 10px;")
            widget = QLineEdit()
            self.setup_inputs[corner_key][param] = widget
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(widget)
            reb_layout.addWidget(cell)
        layout.addWidget(reb_row)

        if corner_label in ("FL", "RL"):
            mirror_target = "FR" if corner_label == "FL" else "RR"
            btn_mirror = QPushButton(f"↔ mirror damper to {mirror_target}")
            btn_mirror.setStyleSheet("background-color: #1e1e1e; color: #888; font-size: 10px; padding: 3px 8px;")
            btn_mirror.clicked.connect(lambda checked, cl=corner_label: self._mirror_damper(cl))
            layout.addWidget(btn_mirror)

        btn_advanced = QPushButton("▶ Damper Advanced")
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
            self.setup_inputs[corner_key][param] = widget
            advanced_layout.addWidget(self._setup_row(labels[param], widget))

        layout.addWidget(advanced_widget)

        btn_advanced.toggled.connect(lambda checked, aw=advanced_widget, btn=btn_advanced: (
            aw.setVisible(checked),
            btn.setText("▼ Damper Advanced" if checked else "▶ Damper Advanced")
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
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
            }
        """)
        weights_layout = QVBoxLayout(weights_group)
        weights_layout.setSpacing(4)
        weights_layout.setContentsMargins(8, 12, 8, 8)

        self.setup_inputs["car"] = {}

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
            self.setup_inputs["car"][param] = widget
            left_weights_layout.addWidget(self._setup_row(label_text, widget))

        for param, label_text in [("corner_weight_fr", "FR (kg)"), ("corner_weight_rr", "RR (kg)")]:
            widget = NoScrollSpinBox()
            widget.setRange(0, 9999)
            widget.setDecimals(1)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self.setup_inputs["car"][param] = widget
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
            self.setup_inputs["car"][param] = widget
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
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
            }
        """)
        car_layout = QVBoxLayout(car_group)
        car_layout.setSpacing(4)
        car_layout.setContentsMargins(8, 12, 8, 8)

        car_labels = {
            "differential_preload": "Diff Preload",
            "differential_position": "Diff Position",
            "wing_position": "Wing Pos.",
            "splitter_offset": "Splitter",
            
        }

        for param, label_text in car_labels.items():
            if param == "notes":
                widget = QLineEdit()
            else:
                widget = NoScrollSpinBox()
                widget.setRange(-9999, 9999)
                widget.setDecimals(2)
                widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self.setup_inputs["car"][param] = widget
            car_layout.addWidget(self._setup_row(label_text, widget))

        layout.addWidget(car_group)
        layout.addStretch()
        return center

    def _mirror_damper(self, from_corner):
        mapping = {"FL": "FR", "RL": "RR"}
        from_key = {"FL": "front_left", "RL": "rear_left"}[from_corner]
        to_key = {"FR": "front_right", "RR": "rear_right"}[mapping[from_corner]]

        for param in ["bump_ls", "bump_hs", "blowoff", "rebound_ls", "rebound_hs"]:
            self.setup_inputs[to_key][param].setText(
                self.setup_inputs[from_key][param].text()
            )

    def _collect_setup_data(self):
        import json
        data = {}
        for corner_key, fields in self.setup_inputs.items():
            data[corner_key] = {}
            for param, widget in fields.items():
                if isinstance(widget, QDoubleSpinBox):
                    data[corner_key][param] = widget.value()
                elif isinstance(widget, QLineEdit):
                    data[corner_key][param] = widget.text().strip()
                elif isinstance(widget, QTextEdit):
                    data[corner_key][param] = widget.toPlainText().strip()
        return json.dumps(data)
    
    def _load_setup_data(self, json_string):
        import json
        if not json_string:
            return
        try:
            data = json.loads(json_string)
        except (json.JSONDecodeError, TypeError):
            return

        for corner_key, fields in data.items():
            if corner_key not in self.setup_inputs:
                continue
            for param, value in fields.items():
                if param not in self.setup_inputs[corner_key]:
                    continue
                widget = self.setup_inputs[corner_key][param]
                if isinstance(widget, QDoubleSpinBox):
                    try:
                        widget.setValue(float(value) if value else 0.0)
                    except (ValueError, TypeError):
                        pass
                elif isinstance(widget, QLineEdit):
                    widget.setText(str(value) if value else "")
                elif isinstance(widget, QTextEdit):
                    widget.setPlainText(str(value) if value else "")

    def _build_feedback_section(self):
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._section_label("Driver Feedback"))

        placeholder = QLabel("Track map and corner feedback will go here")
        placeholder.setStyleSheet("color: #555; font-size: 12px;")
        layout.addWidget(placeholder)

        return section
    
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

    def _save_outing(self):
        driver_id = self.driver_combo.currentData()

        session = Session()
        if self.outing:
            from sqlalchemy import update
            session.execute(
                update(Outing).where(Outing.id == self.outing.id).values(
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
                    setup_data=self._collect_setup_data()
                )
            )
        else:
            outing_count = session.query(Outing).filter(
                Outing.race_weekend_id == self.weekend.id).count()
            outing = Outing(
                race_weekend_id=self.weekend.id,
                date_time=self.datetime_edit.dateTime().toPyDateTime(),
                number=outing_count + 1,
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
                setup_data=self._collect_setup_data()
            )
            session.add(outing)
        session.commit()
        session.close()
        self.on_back()