# Outing form — full form for creating a new outing.


from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QScrollArea, QPushButton,
    QLineEdit, QComboBox, QTextEdit,
    QDateTimeEdit, QDoubleSpinBox
)
from PyQt6.QtCore import Qt, QDateTime
from models.base import Session
from models.driver import Driver
from models.outing import Outing


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

        # scrollable content area
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
            btn_delete.setStyleSheet("background-color: #252525; color: #C0392b;")
            btn_delete.clicked.connect(self._delete_outing)
            layout.addWidget(btn_delete)



        return header

    def _section_label(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size: 13px; font-weight: 600; color: #C0A060; margin-bottom: 8px;")
        return label

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
        self.name_input.setPlaceholderText("Optional name for this outing (e.g. 'Jörg quali sim fail')")

        self.driver_combo = QComboBox()
        self._load_drivers()

        self.session_type_combo = QComboBox()
        self.session_type_combo.addItems(["Practice", "Qualifying", "Race", "Warmup"])

        self.tyre_type_combo = QComboBox()
        self.tyre_type_combo.addItems(["Dry", "Wet"])

        self.tyre_name_input = QLineEdit()
        self.tyre_name_input.setPlaceholderText("e.g. Prc Set 1")

        self.tyre_age_input = QDoubleSpinBox()
        self.tyre_age_input.setSuffix(" Km")
        self.tyre_age_input.setDecimals(1)
        self.tyre_age_input.setRange(0, 9999)

        self.fuel_load_input = QDoubleSpinBox()
        self.fuel_load_input.setSuffix(" Liters")
        self.fuel_load_input.setDecimals(1)
        self.fuel_load_input.setRange(0, 200)

        self.air_temp_input = QDoubleSpinBox()
        self.air_temp_input.setSuffix(" °C")
        self.air_temp_input.setRange(-20, 80)
        self.air_temp_input.setDecimals(1)

        self.track_temp_input = QDoubleSpinBox()
        self.track_temp_input.setSuffix(" °C")
        self.track_temp_input.setRange(-20, 80)
        self.track_temp_input.setDecimals(1)

        self.track_condition_combo = QComboBox()
        self.track_condition_combo.addItems(["Dry", "Damp", "Wet"])


        layout.addWidget(self._row("Date & Time", self.datetime_edit))
        layout.addWidget(self._row("name", self.name_input))
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
        layout.setSpacing(10)
        layout.addWidget(self._section_label("Car Setup"))

        placeholder = QLabel("Setup fields will go here")
        placeholder.setStyleSheet("color: #555; font-size: 12px;")
        layout.addWidget(placeholder)

        return section

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
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._section_label("Comments"))

        self.comments_input = QTextEdit()
        self.comments_input.setFixedHeight(120)
        self.comments_input.setPlaceholderText("General notes about this outing...")
        layout.addWidget(self.comments_input)

        return section

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

    def _load_drivers(self):
        session = Session()
        drivers = session.query(Driver).order_by(Driver.name).all()
        for driver in drivers:
            self.driver_combo.addItem(driver.name, userData=driver.id)
        session.close()

    def _prefill(self):
        if not self.outing:
            return
        
        self.datetime_edit.setDateTime(QDateTime.fromString(self.outing.date_time.strftime("%Y-%m-%d %H:%M:%S"), "yyyy-MM-dd HH:mm:ss"))
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
        if not driver_id:
            self.on_back()
            return
        
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
                    comments=self.comments_input.toPlainText().strip()
                )
            )
        else:
            outing_count = session.query(Outing).filter(Outing.race_weekend_id == self.weekend.id).count()
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
                comments=self.comments_input.toPlainText().strip()
            )
            session.add(outing)
        session.commit()
        session.close()
        self.on_back()