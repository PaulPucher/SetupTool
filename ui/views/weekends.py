# Race Weekends & Tests view — main list showing all events, sortable by column.

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView
)
from PyQt6.QtCore import Qt
from ui.views.weekend_dialog import WeekendDialog
from models.base import Session
from models.raceweekend import RaceWeekend


class WeekendsView(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        layout.addWidget(self._build_header())
        layout.addWidget(self._build_table())
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

        
    def _build_table(self):
        table = QTableWidget()
        table.setColumnCount(5)
        table.setHorizontalHeaderLabels(["Track", "Series", "Car No.", "Date", "Type"])
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSortingEnabled(True)

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
            self.table.setItem(row, 3, QTableWidgetItem(str(weekend.date) if weekend.date else ""))
            self.table.setItem(row, 4, QTableWidgetItem(""))
        
        session.close()