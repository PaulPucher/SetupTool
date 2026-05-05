# Race Weekends & Tests view — main list showing all events, sortable by column.

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView, QStackedWidget
)
from PyQt6.QtCore import Qt
from ui.views.weekend_dialog import WeekendDialog
from ui.views.outings import OutingsView
from models.base import Session
from models.raceweekend import RaceWeekend


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
        session = Session()
        weekends = session.query(RaceWeekend).order_by(RaceWeekend.date.desc()).all()
        weekend = weekends[row] 
        session.close()

        outings_view = OutingsView(weekend, on_back=self._show_list)
        self.stack.addWidget(outings_view)
        self.stack.setCurrentWidget(outings_view)

    def _show_list(self):
        self.load_data()
        self.stack.setCurrentWidget(self.list_page)

    def _open_edit_dialog(self, row, column):
        session = Session()
        weekends = session.query(RaceWeekend).order_by(RaceWeekend.date.desc()).all()
        weekend = weekends[row]
        session.close()
        dialog = WeekendDialog(self, weekend=weekend)
        if dialog.exec():
            self.load_data()

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

        session.close()