# Race Weekends & Tests view: the main list of all events, sortable by
# column, and the create / edit / delete dialog for one weekend or test.

from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QComboBox, QDateEdit, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView, QStackedWidget, QMessageBox
)
from PyQt6.QtCore import Qt, QDate
from models.db import Session, Outing, RaceWeekend
from ui.style import BAD, PANEL_ALT, TEXT_MUTED


class WeekendsView(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.stack = QStackedWidget()

        self.list_page = QWidget()
        list_layout = QVBoxLayout(self.list_page)
        list_layout.setContentsMargins(0, 0, 0, 0)
        list_layout.setSpacing(0)
        list_layout.addWidget(self._build_header())
        list_layout.addWidget(self._build_table())
        
        self.stack.addWidget(self.list_page)
        layout.addWidget(self.stack)
        self.load_data()

    def _build_header(self):
        header = QWidget()
        header.setFixedHeight(52)
        header.setStyleSheet("border-bottom: 1px solid #222;")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(20, 0, 20, 0)

        title = QLabel("Race Weekends & Tests")
        title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        btn_new = QPushButton("+ New")
        btn_new.setFixedWidth(80)
        btn_new.clicked.connect(self._open_new_dialog)

        layout.addWidget(title)
        layout.addStretch()
        layout.addWidget(btn_new)

        return header

    def _open_new_dialog(self):
        dialog = WeekendDialog(self)
        if dialog.exec():
            self.load_data()

    def _open_weekend(self, row, column):
        # the row's own id: after a header sort the row index no longer
        # matches the query order
        session = Session()
        weekend = session.get(RaceWeekend, self.table.item(row, 0).data(Qt.ItemDataRole.UserRole))
        session.close()

        # imported here: outings imports this module for WeekendDialog
        from ui.views.outings import OutingsView
        outings_view = OutingsView(weekend, on_back=self._show_list)
        self.stack.addWidget(outings_view)
        self.stack.setCurrentWidget(outings_view)

    def _show_list(self):
        self.load_data()
        self.stack.setCurrentWidget(self.list_page)

    def _build_table(self):
        table = QTableWidget()
        table.setColumnCount(5)
        table.setHorizontalHeaderLabels(["Track", "Series", "Car No.", "Date", "Type"])
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        table.setColumnWidth(4, 120)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSortingEnabled(True)
        table.cellDoubleClicked.connect(self._open_weekend)

        self.table = table
        return table
    
    def load_data(self):
        session = Session()
        weekends = session.query(RaceWeekend).order_by(RaceWeekend.date.desc()).all()
        
        # sorting off while filling: with it on, every setItem re-sorts and the
        # rest of the row lands on another row
        self.table.setSortingEnabled(False)
        self.table.setRowCount(0)
        
        for weekend in weekends:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(weekend.track))
            self.table.setItem(row, 1, QTableWidgetItem(weekend.series))
            self.table.setItem(row, 2, QTableWidgetItem(str(weekend.car_number)))
            date_str = weekend.date.strftime("%d.%m.%Y") if weekend.date else ""
            self.table.setItem(row, 3, QTableWidgetItem(date_str))
            self.table.setItem(row, 4, QTableWidgetItem(weekend.type or ""))
            self.table.item(row, 0).setData(Qt.ItemDataRole.UserRole, weekend.id)

        self.table.setSortingEnabled(True)
        session.close()


class WeekendDialog(QDialog):
    def __init__(self, parent=None, weekend=None):
        super().__init__(parent)
        self.weekend = weekend
        self.setWindowTitle("New Race Weekend / Test")
        self.setFixedWidth(400)
        self.setModal(True)
        # native min/max buttons on every dialog
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowMinMaxButtonsHint)

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

        if weekend:
            self.setWindowTitle("Edit Race Weekend / Test")
            self.track_input.setText(weekend.track)
            self.series_input.setText(weekend.series)
            self.car_number_input.setText(str(weekend.car_number))
            self.type_combo.setCurrentText(weekend.type or "Race Weekend")
            if weekend.date:
                self.date_edit.setDate(QDate(weekend.date.year, weekend.date.month, weekend.date.day))

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
        label.setStyleSheet(f"color: {TEXT_MUTED};")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        return row

    def _build_buttons(self):
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.setStyleSheet(f"background-color: {PANEL_ALT}; color: {TEXT_MUTED};")
        btn_cancel.clicked.connect(self.reject)

        btn_save = QPushButton("Save")
        btn_save.clicked.connect(self._save)

        if self.weekend:
            btn_delete = QPushButton("Delete")
            btn_delete.setStyleSheet(f"background-color: {PANEL_ALT}; color: {BAD};")
            btn_delete.clicked.connect(self._delete)
            layout.addWidget(btn_delete)

        layout.addStretch()
        layout.addWidget(btn_cancel)
        layout.addWidget(btn_save)
        return row

    def _save(self):
        track = self.track_input.text().strip()
        series = self.series_input.text().strip()
        car_number = self.car_number_input.text().strip()
        date = self.date_edit.date().toPyDate()
        event_type = self.type_combo.currentText()
       

        if not track or not series or not car_number:
            QMessageBox.warning(self, "Missing field", "Track, series and car number are required.")
            return

        try:
            car_number_int = int(car_number)
        except ValueError:
            QMessageBox.warning(self, "Invalid car number", "The car number must be a whole number.")
            return

        session = Session()
        if self.weekend:
            from sqlalchemy import update
            session.execute(
                update(RaceWeekend).where(RaceWeekend.id == self.weekend.id).values(
                    track=track,
                    series=series,
                    car_number=car_number_int,
                    year=date.year,
                    date=date,
                    type=event_type
                )
            )
        else:
            weekend = RaceWeekend(
                track=track,
                series=series,
                car_number=car_number_int,
                year=date.year,
                date=date,
                type=event_type
            )
            session.add(weekend)
        session.commit()
        session.close()
        self.accept()
    
    def _delete(self):
        # A weekend with outings is refused, not deleted together with them:
        # the outings carry stored analyses and setup sheets that one click
        # must not wipe. Single outings are deleted from the outing form. An
        # empty one still asks first, like the outing delete.
        session = Session()
        n_outings = session.query(Outing).filter(Outing.race_weekend_id == self.weekend.id).count()
        if n_outings:
            session.close()
            QMessageBox.warning(
                self, "Delete race weekend / test",
                f"This race weekend / test still has {n_outings} outing{'s' if n_outings != 1 else ''}. "
                "Delete them first; nothing was deleted.",
            )
            return
        reply = QMessageBox.question(
            self, "Delete race weekend / test",
            "Delete this race weekend / test? It has no outings. This cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            session.close()
            return
        weekend = session.get(RaceWeekend, self.weekend.id)
        session.delete(weekend)
        session.commit()
        session.close()
        self.accept()