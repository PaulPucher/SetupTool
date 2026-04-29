# Dialog for creating a new race weekend or test.
# Opens when the user clicks + New in the weekends view.

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QComboBox,
    QPushButton, QDateEdit, QWidget
)
from PyQt6.QtCore import Qt, QDate
from models.base import Session
from models.raceweekend import RaceWeekend


class WeekendDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("New Race Weekend / Test")
        self.setFixedWidth(400)
        self.setModal(True)

        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        self.track_input = QLineEdit()
        self.series_input = QLineEdit()
        self.car_number_input = QLineEdit()

        self.type_combo = QComboBox()
        self.type_combo.addItems(["Race Weekend", "Test"])

        self.date_edit = QDateEdit()
        self.date_edit.setCalendarPopup(True)
        self.date_edit.setDate(QDate.currentDate())

        layout.addWidget(self._row("Track", self.track_input))
        layout.addWidget(self._row("Series", self.series_input))
        layout.addWidget(self._row("Car number", self.car_number_input))
        layout.addWidget(self._row("Type", self.type_combo))
        layout.addWidget(self._row("Date", self.date_edit))

        layout.addStretch()
        layout.addWidget(self._build_buttons())

    def _row(self, label_text, widget):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(label_text)
        label.setFixedWidth(100)
        label.setStyleSheet("color: #888;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        return row

    def _build_buttons(self):
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.setStyleSheet("background-color: #252525; color: #888;")
        btn_cancel.clicked.connect(self.reject)

        btn_save = QPushButton("Save")
        btn_save.clicked.connect(self._save)

        layout.addWidget(btn_cancel)
        layout.addWidget(btn_save)
        return row

    def _save(self):
        track = self.track_input.text().strip()
        series = self.series_input.text().strip()
        car_number = self.car_number_input.text().strip()
        date = self.date_edit.date().toPyDate()

        if not track or not series or not car_number:
            return

        session = Session()
        weekend = RaceWeekend(
            track=track,
            series=series,
            car_number=int(car_number),
            year=date.year,
            date=date
        )
        session.add(weekend)
        session.commit()
        session.close()
        self.accept()