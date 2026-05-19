# Outings view — list of all outings for a specific race weekend.


from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem,
    QPushButton, QHeaderView, QStackedWidget
)
from PyQt6.QtCore import Qt
from models.base import Session
from models.outing import Outing
from models.raceweekend import RaceWeekend
from ui.views.weekend_dialog import WeekendDialog
from ui.views.outing_form import OutingForm


class OutingsView(QWidget):
    def __init__(self, weekend, on_back):
        super().__init__()
        self.weekend = weekend
        self.on_back = on_back

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

        btn_back = QPushButton("← Back")
        btn_back.setFixedWidth(80)
        btn_back.setStyleSheet("background-color: #252525; color: #888;")
        btn_back.clicked.connect(self.on_back)

        self.title = QLabel(f"{self.weekend.track} — {self.weekend.series} {self.weekend.year}")
        self.title.setStyleSheet("font-size: 15px; font-weight: 500; color: #e0e0e0;")

        btn_edit = QPushButton("Edit")
        btn_edit.setFixedWidth(80)
        btn_edit.setStyleSheet("background-color: #252525; color: #888;")
        btn_edit.clicked.connect(self._open_edit_dialog)
        
        btn_new = QPushButton("+ New")
        btn_new.setFixedWidth(80)
        btn_new.clicked.connect(self._open_new_outing)

        layout.addWidget(btn_back)
        layout.addSpacing(16)
        layout.addWidget(self.title)
        layout.addStretch()
        layout.addWidget(btn_edit)
        layout.addSpacing(8)
        layout.addWidget(btn_new)

        return header

    def _build_table(self):
        table = QTableWidget()
        table.setColumnCount(6)
        table.setHorizontalHeaderLabels(["No.", "Name", "Driver", "Session Type", "Tyre Age", "Date & Time"])
        table.setColumnWidth(0, 40)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        table.setColumnWidth(2, 150)
        table.setColumnWidth(3, 120)
        table.setColumnWidth(4, 80)
        table.setColumnWidth(5, 160)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSortingEnabled(True)
        table.cellDoubleClicked.connect(self._open_edit_outing)

        self.table = table
        return table

    def load_data(self):
        session = Session()
        outings = (
            session.query(Outing)
            .filter(Outing.race_weekend_id == self.weekend.id)
            .order_by(Outing.date_time.desc())
            .all()
        )

        self.table.setRowCount(0)

        for outing in outings:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(str(outing.number or "")))
            self.table.setItem(row, 1, QTableWidgetItem(outing.name or ""))
            self.table.setItem(row, 2, QTableWidgetItem(outing.driver.name if outing.driver else ""))
            self.table.setItem(row, 3, QTableWidgetItem(outing.session_type or ""))
            self.table.setItem(row, 4, QTableWidgetItem(f"{outing.tyre_age} km" if outing.tyre_age else "New"))
            self.table.setItem(row, 5, QTableWidgetItem(outing.date_time.strftime("%d.%m.%Y %H:%M")))
            self.table.item(row, 0).setData(Qt.ItemDataRole.UserRole, outing.id)

        session.close()
    
    def _open_edit_dialog(self):
        dialog = WeekendDialog(self, weekend=self.weekend)
        if dialog.exec():
            self.weekend = Session().get(RaceWeekend, self.weekend.id)
            self.title.setText(f"{self.weekend.track} — {self.weekend.series} {self.weekend.year}")

    def _open_new_outing(self):
        form = OutingForm(self.weekend, on_back=self._show_list)
        self.stack.addWidget(form)
        self.stack.setCurrentWidget(form)

    def _open_edit_outing(self, row, column):
        outing_id = self.table.item(row, 0).data(Qt.ItemDataRole.UserRole)
        session = Session()
        outing = session.get(Outing, outing_id)
        session.close()
        form = OutingForm(self.weekend, on_back=self._show_list, outing=outing)
        self.stack.addWidget(form)
        self.stack.setCurrentWidget(form)
    
    def _show_list(self):
        self.load_data()
        self.stack.setCurrentWidget(self.list_page)
                               
