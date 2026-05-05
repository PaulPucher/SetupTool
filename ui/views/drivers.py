# Drivers view — list of all drivers with name and driving level.

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView
)
from PyQt6.QtCore import Qt
from models.base import Session
from models.driver import Driver
from ui.views.driver_dialog import DriverDialog


class DriversView(QWidget):
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

        title = QLabel("Drivers")
        title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        btn_new = QPushButton("+ New")
        btn_new.setFixedWidth(80)
        btn_new.clicked.connect(self._open_new_dialog)

        layout.addWidget(title)
        layout.addStretch()
        layout.addWidget(btn_new)

        return header

    def _open_new_dialog(self):
        dialog = DriverDialog(self)
        if dialog.exec():
            self.load_data()

    def _open_edit_dialog(self, row, column):
        session = Session()
        drivers = session.query(Driver).order_by(Driver.name).all()
        driver = drivers[row]
        session.close()
        dialog = DriverDialog(self, driver=driver)
        if dialog.exec():
            self.load_data()

    def _build_table(self):
        table = QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(["Name", "Level"])
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        table.setColumnWidth(1, 80)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSortingEnabled(True)
        table.cellDoubleClicked.connect(self._open_edit_dialog)

        self.table = table
        return table

    def load_data(self):
        session = Session()
        drivers = session.query(Driver).order_by(Driver.name).all()

        self.table.setRowCount(0)

        for driver in drivers:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(driver.name))
            self.table.setItem(row, 1, QTableWidgetItem(str(driver.driving_level) if driver.driving_level else ""))

        session.close()