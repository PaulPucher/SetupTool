# Drivers view: the list of all drivers with name and driving level, and the
# dialog for creating or editing a driver.

from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QComboBox, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView
)
from PyQt6.QtCore import Qt
from models.db import Session, Driver
from ui.style import BAD, PANEL_ALT, TEXT_DIM, TEXT_MUTED


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
        # the row's own id: after a header sort the row index no longer
        # matches the query order
        session = Session()
        driver = session.get(Driver, self.table.item(row, 0).data(Qt.ItemDataRole.UserRole))
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

        # sorting off while filling, as in the weekend list
        self.table.setSortingEnabled(False)
        self.table.setRowCount(0)

        for driver in drivers:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(driver.name))
            self.table.setItem(row, 1, QTableWidgetItem(str(driver.driving_level) if driver.driving_level else ""))
            self.table.item(row, 0).setData(Qt.ItemDataRole.UserRole, driver.id)

        self.table.setSortingEnabled(True)
        session.close()


class DriverDialog(QDialog):
    def __init__(self, parent=None, driver=None):
        super().__init__(parent)
        self.setWindowTitle("New Driver")
        self.setFixedWidth(400)
        self.setModal(True)
        # native min/max buttons on every dialog
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowMinMaxButtonsHint)

        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        self.driver = driver

        self.name_input = QLineEdit()

        self.level_combo = QComboBox()
        self.level_combo.addItems([str(i) for i in range(1, 11)])

        if driver:
            self.setWindowTitle("Edit Driver")
            self.name_input.setText(driver.name)
            self.level_combo.setCurrentText(str(driver.driving_level))

        layout.addWidget(self._row("Name", self.name_input))
        layout.addWidget(self._row_with_hint("Level", self.level_combo, "1 = lowest, 10 = highest"))

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

    def _row_with_hint(self, label_text, widget, hint_text):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(label_text)
        label.setFixedWidth(100)
        label.setStyleSheet(f"color: {TEXT_MUTED};")
        hint = QLabel(hint_text)
        hint.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        row_layout.addWidget(hint)
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

        if self.driver:
            btn_delete = QPushButton("Delete")
            btn_delete.setStyleSheet(f"background-color: {PANEL_ALT}; color: {BAD};")
            btn_delete.clicked.connect(self._delete)
            layout.addWidget(btn_delete)

        layout.addStretch()
        layout.addWidget(btn_cancel)
        layout.addWidget(btn_save)
        return row

    def _save(self):
        name = self.name_input.text().strip()
        level = int(self.level_combo.currentText())

        if not name:
            return

        session = Session()
        if self.driver:
            from sqlalchemy import update
            session.execute(
                update(Driver).
                where(Driver.id == self.driver.id).
                values(name=name, driving_level=level)  
            )
        else:
            driver = Driver(name=name, driving_level=level)
            session.add(driver)
        session.commit()
        session.close()
        self.accept()

    def _delete(self):
        session = Session()
        driver = session.get(Driver, self.driver.id)
        session.delete(driver)
        session.commit()
        session.close()
        self.accept()