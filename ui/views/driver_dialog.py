# Dialog for creating or editing a driver.

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QComboBox,
    QPushButton, QWidget
)
from PyQt6.QtCore import Qt
from models.base import Session
from models.driver import Driver


class DriverDialog(QDialog):
    def __init__(self, parent=None, driver=None):
        super().__init__(parent)
        self.setWindowTitle("New Driver")
        self.setFixedWidth(400)
        self.setModal(True)

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
        label.setStyleSheet("color: #888;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        return row

    def _row_with_hint(self, label_text, widget, hint_text):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(label_text)
        label.setFixedWidth(100)
        label.setStyleSheet("color: #888;")
        hint = QLabel(hint_text)
        hint.setStyleSheet("color: #555; font-size: 11px;")
        row_layout.addWidget(label)
        row_layout.addWidget(widget)
        row_layout.addWidget(hint)
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

        if self.driver:
            btn_delete = QPushButton("Delete")
            btn_delete.setStyleSheet("background-color: #252525; color: #c0392b;")
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