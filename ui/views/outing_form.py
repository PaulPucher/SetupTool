# Outing form — full form for creating a new outing.

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QScrollArea, QPushButton,
    QLineEdit, QComboBox, QTextEdit,
    QDateTimeEdit, QDoubleSpinBox,
    QGroupBox, QFrame, QSpinBox,
    QTableWidget, QTableWidgetItem, QCheckBox,
    QHeaderView, QAbstractSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, QDateTime
from models.base import Session
from models.driver import Driver
from models.outing import Outing
from core.config_loader import get_setup_parameters
from PyQt6.QtCore import Qt, QDateTime, QThread, pyqtSignal, QTimer

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
            self.error.emit(str(e))

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

        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(10)

        btn_load = QPushButton("Load Outing")
        btn_load.setFixedWidth(120)
        btn_load.clicked.connect(self._load_csv)

        self.csv_status_label = QLabel("No file loaded")
        self.csv_status_label.setStyleSheet("color: #555; font-size: 12px;")

        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(self.csv_status_label)
        btn_layout.addStretch()
        layout.addWidget(btn_row)

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
        self.lap_table.setVisible(False)
        layout.addWidget(self.lap_table)

        return section

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
            QProgressDialog {
                background-color: #1a1a1a;
                color: #e0e0e0;
            }
            QLabel {
                color: #e0e0e0;
                font-size: 12px;
            }
            QProgressBar {
                background-color: #141414;
                border: 1px solid #2a2a2a;
                border-radius: 3px;
                height: 6px;
            }
            QProgressBar::chunk {
                background-color: #C0A060;
                border-radius: 3px;
            }
        """)
        self.progress.show()

        self.loader_thread = CsvLoaderThread(path)
        self.loader_thread.finished.connect(self._on_csv_loaded)
        self.loader_thread.error.connect(self._on_csv_error)
        self.loader_thread.start()

    def _on_csv_loaded(self, result):
        import os
        from modules.csv_parser import get_lap_summary, get_available_channels
        self.progress.close()
        self.parsed_data = result
        self.loaded_csv_path = self.loader_thread.path
        filename = os.path.basename(self.loader_thread.path)
        laps = get_lap_summary(self.parsed_data)
        available = get_available_channels(self.parsed_data)
        self.csv_status_label.setText(
            f"{filename} — {len(laps)} laps, {len(available)} channels"
        )
        self.csv_status_label.setStyleSheet("color: #888; font-size: 12px;")
        self._populate_lap_table(laps)

    def _on_csv_error(self, error_msg):
        self.progress.close()
        self.csv_status_label.setText(f"Error loading file: {error_msg}")
        self.csv_status_label.setStyleSheet("color: #c0392b; font-size: 12px;")

    def _populate_lap_table(self, laps):
        self.lap_table.setRowCount(0)

        for lap in laps:
            row = self.lap_table.rowCount()
            self.lap_table.insertRow(row)
            self.lap_table.setRowHeight(row, 28)

            lap_item = QTableWidgetItem(str(lap["lap_number"]))
            lap_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            mins = int(lap["lap_time"] // 60)
            secs = lap["lap_time"] % 60
            time_str = f"{mins}:{secs:06.3f}"
            time_item = QTableWidgetItem(time_str)
            time_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            badge_item = QTableWidgetItem("FASTEST" if lap["is_fastest"] else "")
            badge_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            if lap["is_fastest"]:
                for item in [lap_item, time_item, badge_item]:
                    item.setForeground(__import__('PyQt6.QtGui', fromlist=['QColor']).QColor("#C0A060"))

            self.lap_table.setItem(row, 0, lap_item)
            self.lap_table.setItem(row, 1, time_item)
            self.lap_table.setItem(row, 2, badge_item)

        if self.lap_table.rowCount() > 0:
            header_h = self.lap_table.horizontalHeader().height()
            total_row_h = sum(self.lap_table.rowHeight(i) for i in range(self.lap_table.rowCount()))
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
            btn_print = QPushButton("⎙ Print Setup")
            btn_print.setFixedWidth(140)
            btn_print.clicked.connect(lambda: self._print_sheet("setup"))
            layout.addWidget(btn_print)
        else:
            btn_print = QPushButton("⎙ Print Setdown")
            btn_print.setFixedWidth(140)
            btn_print.clicked.connect(lambda: self._print_sheet("setdown"))
            layout.addWidget(btn_print)

        return section

    def _build_setdown_toggle(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("▶ Add Setdown")
        btn_toggle.setStyleSheet("background-color: #1a1a1a; color: #888; font-size: 12px; padding: 8px 14px; text-align: left;")
        btn_toggle.setCheckable(True)
        btn_toggle.setChecked(False)
        layout.addWidget(btn_toggle)

        self.setdown_widget = self._build_setup_section("setdown")
        self.setdown_widget.setVisible(False)
        layout.addWidget(self.setdown_widget)

        btn_toggle.toggled.connect(lambda checked, btn=btn_toggle: (
            self.setdown_widget.setVisible(checked),
            btn.setText("▼ Add Setdown" if checked else "▶ Add Setdown"),
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
            btn_mirror = QPushButton(f"↔ mirror damper to {mirror_target}")
            btn_mirror.setStyleSheet("background-color: #1e1e1e; color: #888; font-size: 10px; padding: 3px 8px;")
            btn_mirror.clicked.connect(lambda checked, cl=corner_label, inp=self._active_inputs: self._mirror_damper(cl, inp))
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
            self._active_inputs[corner_key][param] = widget
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
            widget = NoScrollSpinBox()
            widget.setRange(-9999, 9999)
            widget.setDecimals(2)
            widget.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            self._active_inputs["car"][param] = widget
            car_layout.addWidget(self._setup_row(label_text, widget))

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
                elif isinstance(widget, QLineEdit):
                    data[corner_key][param] = widget.text().strip()
                elif isinstance(widget, QTextEdit):
                    data[corner_key][param] = widget.toPlainText().strip()
        return json.dumps(data)

    def _collect_setup_data(self):
        return self._collect_inputs(self.setup_inputs)

    def _collect_setdown_data(self):
        return self._collect_inputs(self.setdown_inputs)

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
                elif isinstance(widget, QLineEdit):
                    widget.setText(str(value) if value else "")
                elif isinstance(widget, QTextEdit):
                    widget.setPlainText(str(value) if value else "")

    def _load_setup_data(self, json_string):
        self._load_inputs(self.setup_inputs, json_string)

    def _load_setdown_data(self, json_string):
        self._load_inputs(self.setdown_inputs, json_string)

    def _prefill_setdown(self):
        if self.outing and self.outing.setdown_data:
            self._load_inputs(self.setdown_inputs, self.outing.setdown_data)
        else:
            self._load_inputs(self.setdown_inputs, self._collect_setup_data())

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
            "Scale: −5 undrivable understeer · −3 strong understeer · −1 slight understeer · "
            "0 neutral · +1 slight oversteer · +3 strong oversteer · +5 undrivable oversteer\n"
            "Placeholder — full description to be added per value."
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
                    setup_data=self._collect_setup_data(),
                    setdown_data=self._collect_setdown_data(),
                    feedback_data=self._collect_feedback_data(),
                    csv_path=self.loaded_csv_path or "",
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
                setup_data=self._collect_setup_data(),
                setdown_data=self._collect_setdown_data(),
                feedback_data=self._collect_feedback_data(),
                csv_path=self.loaded_csv_path or "",
            )
            session.add(outing)
        session.commit()
        session.close()
        self.on_back()