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
from PyQt6.QtCore import Qt, QDateTime, QThread, pyqtSignal, QTimer
from models.base import Session
from models.driver import Driver
from models.outing import Outing
from core.config_loader import get_setup_parameters
from ui.style import ACCENT, OK, WARN, BAD, NEUTRAL, TEXT, TEXT_MUTED, TEXT_DIM, PANEL, PANEL_ALT, BORDER


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


class StabilityAnalysisThread(QThread):
    finished = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self, parsed_data, lap_filter):
        super().__init__()
        self.parsed_data = parsed_data
        self.lap_filter = lap_filter

    def run(self):
        try:
            from modules.stability_analysis import (
                load_parameters, prepare_vehicle_state, estimate_sideslip,
                estimate_slip_angles, estimate_lateral_forces,
                estimate_cornering_stiffness, estimate_yaw_moment_stability,
                summarise_corners,
            )
            params = load_parameters()
            state = prepare_vehicle_state(self.parsed_data["channels"], params)
            if state is None:
                self.error.emit("Required channels missing or failed")
                return
            beta = estimate_sideslip(state, params)
            slip = estimate_slip_angles(state, beta, params)
            forces = estimate_lateral_forces(state, params)
            cs = estimate_cornering_stiffness(slip, forces, state, params)
            stab = estimate_yaw_moment_stability(state, beta, params)
            corners = self.parsed_data.get("corners", [])
            summaries = summarise_corners(corners, cs, stab, state,
                                          lap_filter=self.lap_filter)
            self.finished.emit({
                "summaries": summaries,
                "state": state,
                "cs": cs,
                "stab": stab,
            })
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
        self.content_layout.addWidget(self._build_stability_toggle())
        self.content_layout.addWidget(self._build_feedback_section())
        self.content_layout.addWidget(self._build_comments_section())
        self.content_layout.addStretch()

        if self.outing:
            self._prefill()
        else:
            self._carryon_from_last()

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

    def _stability_colour(self, kind, value):
        if value is None or (isinstance(value, float) and value != value):
            return NEUTRAL
        if kind == "cs":
            if value >= 0.7:
                return OK
            if value >= 0.4:
                return WARN
            return BAD
        if kind == "stab":
            if value > 500:
                return OK
            if value > -200:
                return WARN
            return BAD
        return TEXT_MUTED

    def _classify_corner(self, summary):
        STRONG_CS = 0.40
        MODERATE_CS = 0.55
        STAB_NEG_THRESH = 0.0

        worst_f_phase = None
        worst_f_val = 1.0
        worst_r_phase = None
        worst_r_val = 1.0
        worst_stab_phase = None
        worst_stab_val = 1e9

        phase_labels = {
            "entry_1_brake": "brake",
            "entry_2_turnin": "turn-in",
            "apex_3": "apex",
            "exit_4": "early exit",
            "exit_5": "late exit",
        }

        for phase, p in summary["phases"].items():
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

        verdicts = []
        severity = "normal"

        front_strong = worst_f_val < STRONG_CS
        rear_strong = worst_r_val < STRONG_CS
        front_moderate = STRONG_CS <= worst_f_val < MODERATE_CS
        rear_moderate = STRONG_CS <= worst_r_val < MODERATE_CS

        if front_strong and rear_strong:
            verdicts.append(
                f"both axles saturated at {phase_labels[worst_f_phase]}"
            )
            severity = "strong"
        elif front_strong:
            verdicts.append(
                f"strong understeer at {phase_labels[worst_f_phase]} "
                f"(front {worst_f_val:.2f})"
            )
            severity = "strong"
        elif rear_strong:
            verdicts.append(
                f"strong oversteer at {phase_labels[worst_r_phase]} "
                f"(rear {worst_r_val:.2f})"
            )
            severity = "strong"
        elif front_moderate and rear_moderate:
            verdicts.append(
                f"both axles working hard at {phase_labels[worst_f_phase]}"
            )
            severity = "moderate"
        elif front_moderate:
            verdicts.append(
                f"mild understeer at {phase_labels[worst_f_phase]} "
                f"(front {worst_f_val:.2f})"
            )
            severity = "moderate"
        elif rear_moderate:
            verdicts.append(
                f"mild oversteer at {phase_labels[worst_r_phase]} "
                f"(rear {worst_r_val:.2f})"
            )
            severity = "moderate"

        if worst_stab_val == worst_stab_val and worst_stab_val < STAB_NEG_THRESH:
            verdicts.append(
                f"destabilising yaw moment at {phase_labels[worst_stab_phase]} "
                f"({worst_stab_val:.0f} Nm/deg)"
            )
            if severity == "normal":
                severity = "moderate"
            elif severity == "moderate":
                severity = "strong"

        if not verdicts:
            verdicts.append("within normal range")

        colour_map = {"strong": BAD, "moderate": WARN, "normal": OK}
        return severity, " · ".join(verdicts), colour_map[severity]

    def _build_corner_card(self, summary):
        card = QWidget()
        card.setStyleSheet(
            f"background-color: {PANEL}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(10, 8, 10, 8)
        card_layout.setSpacing(6)

        severity, verdict, colour = self._classify_corner(summary)

        header = QWidget()
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(0, 0, 0, 0)
        h_layout.setSpacing(10)

        title = QLabel(
            f"Lap {summary['lap_number']} · Corner {summary['corner_number']}  "
            f"<span style='color:{TEXT_DIM};'>({summary['speed_class']}, "
            f"{summary['apex_speed']:.0f} km/h, t={summary['apex_time']:.1f}s)</span>"
        )
        title.setStyleSheet(f"color: {TEXT}; font-size: 12px;")
        title.setTextFormat(Qt.TextFormat.RichText)

        verdict_badge = QLabel(verdict)
        verdict_badge.setStyleSheet(
            f"background-color: {colour}; color: #111; "
            "padding: 3px 10px; border-radius: 3px; font-size: 11px; font-weight: 600;"
        )

        btn_jump = QPushButton("→ plot")
        btn_jump.setFixedWidth(70)
        btn_jump.setStyleSheet(
            f"background-color: {PANEL_ALT}; color: {TEXT_MUTED}; "
            "font-size: 10px; padding: 2px 6px;"
        )
        btn_jump.clicked.connect(lambda _, t=summary["apex_time"]:
                                 self._jump_plot_to_time(t))

        btn_expand = QPushButton("▶ details")
        btn_expand.setCheckable(True)
        btn_expand.setFixedWidth(80)
        btn_expand.setStyleSheet(
            f"background-color: {PANEL_ALT}; color: {TEXT_MUTED}; "
            "font-size: 10px; padding: 2px 6px;"
        )

        h_layout.addWidget(title)
        h_layout.addWidget(verdict_badge)
        h_layout.addStretch()
        h_layout.addWidget(btn_jump)
        h_layout.addWidget(btn_expand)
        card_layout.addWidget(header)

        details = QWidget()
        d_layout = QVBoxLayout(details)
        d_layout.setContentsMargins(0, 6, 0, 0)
        d_layout.setSpacing(2)

        phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
        phase_labels = {
            "entry_1_brake": "Brake",
            "entry_2_turnin": "Turn-in",
            "apex_3": "Apex",
            "exit_4": "Exit 4",
            "exit_5": "Exit 5",
        }

        rows_html = (
            f"<table cellpadding='2' style='font-size:10px;'>"
            f"<tr>"
            f"<th align='left' style='color:{TEXT_DIM};'>phase</th>"
            f"<th style='color:{TEXT_DIM};'>n</th>"
            f"<th style='color:{TEXT_DIM};'>valid</th>"
            f"<th style='color:{TEXT_DIM};'>CSf med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>CSr med [p25..p75]</th>"
            f"<th style='color:{TEXT_DIM};'>Stab med [p25..p75]</th>"
            f"</tr>"
        )
        for phase in phase_keys:
            p = summary["phases"][phase]
            csf = p["cs_ratio_f"]
            csr = p["cs_ratio_r"]
            sob = p["stability_observed_Nm_per_deg"]
            csf_colour = self._stability_colour("cs", csf["median"])
            csr_colour = self._stability_colour("cs", csr["median"])
            sob_colour = self._stability_colour("stab", sob["median"])
            csf_str = (f"{csf['median']:.2f} [{csf['p25']:.2f}..{csf['p75']:.2f}]"
                       if csf["n"] > 0 else "—")
            csr_str = (f"{csr['median']:.2f} [{csr['p25']:.2f}..{csr['p75']:.2f}]"
                       if csr["n"] > 0 else "—")
            sob_str = (f"{sob['median']:.0f} [{sob['p25']:.0f}..{sob['p75']:.0f}]"
                       if sob["n"] > 0 else "—")
            rows_html += (
                f"<tr>"
                f"<td style='color:{ACCENT}; width:80px;'>{phase_labels[phase]}</td>"
                f"<td style='color:{TEXT_MUTED}; width:40px;'>{p['n_samples']}</td>"
                f"<td style='color:{TEXT_MUTED}; width:50px;'>{p['valid_fraction_stab']*100:.0f}%</td>"
                f"<td style='color:{csf_colour}; width:160px;'>{csf_str}</td>"
                f"<td style='color:{csr_colour}; width:160px;'>{csr_str}</td>"
                f"<td style='color:{sob_colour}; width:180px;'>{sob_str}</td>"
                f"</tr>"
            )
        rows_html += "</table>"

        body = QLabel(rows_html)
        body.setTextFormat(Qt.TextFormat.RichText)
        d_layout.addWidget(body)

        details.setVisible(False)
        card_layout.addWidget(details)

        btn_expand.toggled.connect(lambda checked, w=details, b=btn_expand: (
            w.setVisible(checked),
            b.setText("▼ details" if checked else "▶ details")
        ))

        return card, severity

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

        self.btn_analyse = QPushButton("Analyse")
        self.btn_analyse.setFixedWidth(100)
        self.btn_analyse.clicked.connect(self._run_stability_analysis)
        self.btn_analyse.setEnabled(False)

        self.csv_status_label = QLabel("No file loaded")
        self.csv_status_label.setStyleSheet("color: #555; font-size: 12px;")

        self.stability_status_label = QLabel("")
        self.stability_status_label.setStyleSheet("color: #555; font-size: 12px;")

        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(self.btn_analyse)
        btn_layout.addWidget(self.csv_status_label)
        btn_layout.addStretch()
        layout.addWidget(btn_row)

        status_row = QWidget()
        status_layout = QHBoxLayout(status_row)
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.addWidget(self.stability_status_label)
        status_layout.addStretch()
        layout.addWidget(status_row)

        self.exclude_inout_btn = QPushButton("Exclude In/Out Laps")
        self.exclude_inout_btn.setCheckable(True)
        self.exclude_inout_btn.setChecked(False)
        self.exclude_inout_btn.setFixedWidth(160)
        self.exclude_inout_btn.setStyleSheet("""
            QPushButton {
                background-color: #1a1a1a;
                color: #888;
                border: 1px solid #2a2a2a;
                border-radius: 4px;
                padding: 4px 10px;
                font-size: 11px;
            }
            QPushButton:checked {
                background-color: #252525;
                color: #C0A060;
                border-color: #C0A060;
            }
        """)
        self.exclude_inout_btn.toggled.connect(self._on_exclude_toggled)
        self.exclude_inout_btn.setVisible(False)
        layout.addWidget(self.exclude_inout_btn)

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

    def _on_exclude_toggled(self, checked):
        if not self.parsed_data:
            return
        from modules.csv_parser import get_lap_summary
        prev_value = None
        cur_row = self.lap_table.currentRow()
        if cur_row >= 0:
            item = self.lap_table.item(cur_row, 0)
            if item is not None:
                prev_value = item.data(Qt.ItemDataRole.UserRole)
        laps = get_lap_summary(self.parsed_data)
        self._populate_lap_table(laps)
        if prev_value is not None:
            for r in range(self.lap_table.rowCount()):
                it = self.lap_table.item(r, 0)
                if it is not None and it.data(Qt.ItemDataRole.UserRole) == prev_value:
                    self.lap_table.selectRow(r)
                    break

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
            {"key": "log_asteer",   "label": "Steer (°)",    "color": "#9b59b6"},
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
        if value == "all":
            self._update_plots(None)
        else:
            self._update_plots(value)

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
        self.btn_analyse.setEnabled(True)

    def _on_csv_error(self, error_msg):
        self.progress.close()
        self.csv_status_label.setText(f"Error loading file: {error_msg}")
        self.csv_status_label.setStyleSheet("color: #c0392b; font-size: 12px;")

    def _get_lap_filter_from_selector(self):
        if not self.parsed_data:
            return None
        all_laps = sorted({l["lap_number"] for l in self.parsed_data.get("laps", [])})
        exclude_inout = self.exclude_inout_btn.isChecked()
        valid_laps = [l["lap_number"] for l in self.parsed_data.get("laps", [])
                      if l.get("is_valid_for_analysis", False)]
        current_row = self.lap_table.currentRow()
        selected_value = None
        if current_row >= 0:
            lap_item = self.lap_table.item(current_row, 0)
            if lap_item is not None:
                selected_value = lap_item.data(Qt.ItemDataRole.UserRole)
        if selected_value is None or selected_value == "all":
            if exclude_inout:
                return valid_laps if valid_laps else all_laps
            return all_laps
        return [int(selected_value)]

    def _run_stability_analysis(self):
        if not self.parsed_data:
            return
        lap_filter = self._get_lap_filter_from_selector()
        all_lap_nums = sorted({l["lap_number"] for l in self.parsed_data.get("laps", [])})
        exclude_state = self.exclude_inout_btn.isChecked()
        sel_row = self.lap_table.currentRow()
        sel_value = None
        if sel_row >= 0:
            item = self.lap_table.item(sel_row, 0)
            if item is not None:
                sel_value = item.data(Qt.ItemDataRole.UserRole)
        print(f"[ANALYSE] selector_row={sel_row} selector_value={sel_value!r}  "
              f"exclude_inout={exclude_state}  all_laps={all_lap_nums}  "
              f"lap_filter={lap_filter}")
        self.btn_analyse.setEnabled(False)
        self.stability_status_label.setText(
            f"Analysing laps {lap_filter}..."
        )
        self.stability_status_label.setStyleSheet("color: #C0A060; font-size: 12px;")
        self.stab_thread = StabilityAnalysisThread(self.parsed_data, lap_filter)
        self.stab_thread.finished.connect(self._on_stability_done)
        self.stab_thread.error.connect(self._on_stability_error)
        self.stab_thread.start()

    def _on_stability_done(self, result):
        self.stability_result = result
        summaries = result["summaries"]
        self.stability_status_label.setText(
            f"Analysed {len(summaries)} corners. See Stability Analysis section."
        )
        self.stability_status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        self.btn_analyse.setEnabled(True)

        self._clear_cards()
        if not summaries:
            self.stability_summary_label.setText("No corners in selected laps.")
            return

        strong_cards = []
        moderate_cards = []
        normal_cards = []
        for s in summaries:
            card, severity = self._build_corner_card(s)
            if severity == "strong":
                strong_cards.append(card)
            elif severity == "moderate":
                moderate_cards.append(card)
            else:
                normal_cards.append(card)

        n_strong = len(strong_cards)
        n_moderate = len(moderate_cards)
        n_normal = len(normal_cards)

        self.stability_summary_label.setText(
            f"{len(summaries)} corners · "
            f"<span style='color:{BAD};'>{n_strong} strong</span> · "
            f"<span style='color:{WARN};'>{n_moderate} moderate</span> · "
            f"<span style='color:{OK};'>{n_normal} normal</span>"
        )
        self.stability_summary_label.setTextFormat(Qt.TextFormat.RichText)
        self.stability_summary_label.setStyleSheet("font-size: 11px;")

        insert_pos = self.cards_host_layout.count() - 1
        for card in strong_cards + moderate_cards:
            self.cards_host_layout.insertWidget(insert_pos, card)
            insert_pos += 1

        if normal_cards:
            divider = QLabel(f"— {n_normal} corners within normal range —")
            divider.setStyleSheet(
                f"color: {TEXT_DIM}; font-size: 10px; padding: 8px 0;"
            )
            divider.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.cards_host_layout.insertWidget(insert_pos, divider)
            insert_pos += 1
            for card in normal_cards:
                self.cards_host_layout.insertWidget(insert_pos, card)
                insert_pos += 1

    def _build_stability_toggle(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        btn_toggle = QPushButton("▶ Stability Analysis")
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
            btn.setText("▼ Stability Analysis" if checked else "▶ Stability Analysis")
        ))

        return container

    def _clear_cards(self):
        while self.cards_host_layout.count() > 1:
            item = self.cards_host_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _jump_plot_to_time(self, apex_t):
        if not self.parsed_data:
            return
        for lap in self.parsed_data.get("laps", []):
            if lap["start_time"] <= apex_t <= lap["end_time"]:
                rel_t = apex_t - lap["start_time"]
                first_key = self.plot_channels[0]["key"]
                if first_key in self.plot_items:
                    self._update_plots(lap["lap_number"])
                    span = 6.0
                    self.plot_items[first_key].setXRange(
                        max(0, rel_t - span / 2),
                        rel_t + span / 2,
                        padding=0
                    )
                break

    def _on_stability_error(self, msg):
        self.stability_status_label.setText(f"Analysis failed: {msg}")
        self.stability_status_label.setStyleSheet("color: #c0392b; font-size: 12px;")
        self.btn_analyse.setEnabled(True)

    def _populate_lap_table(self, laps):
        from PyQt6.QtGui import QColor
        exclude = getattr(self, 'exclude_inout_btn', None) and self.exclude_inout_btn.isChecked()
        if exclude:
            laps = [l for l in laps if l.get("is_valid_for_analysis", False)]
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

            is_outlap = lap["lap_number"] == 0
            display_text = "Out" if is_outlap else str(lap["lap_number"])
            lap_item = QTableWidgetItem(display_text)
            lap_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            lap_item.setData(Qt.ItemDataRole.UserRole, lap["lap_number"])

            mins = int(lap["lap_time"] // 60)
            secs = lap["lap_time"] % 60
            time_str = f"{mins}:{secs:06.3f}"
            time_item = QTableWidgetItem(time_str)
            time_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            if is_outlap:
                badge_text = "OUT LAP"
            elif lap["is_fastest"]:
                badge_text = "FASTEST"
            else:
                badge_text = ""
            badge_item = QTableWidgetItem(badge_text)
            badge_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

            if lap["is_fastest"] and not is_outlap:
                for item in [lap_item, time_item, badge_item]:
                    item.setForeground(QColor("#C0A060"))
            elif is_outlap:
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
            self.exclude_inout_btn.setVisible(True)

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
            QGroupBox::title { subcontrol-origin: margin; left: 8px; }
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