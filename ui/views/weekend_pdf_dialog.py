# Weekend PDF export -- outing-selection dialog. Tier C UI, no business
# logic beyond the "which outings" selection and the file-save dialog;
# document generation itself lives entirely in core/weekend_pdf_export.py.

import os
from datetime import datetime, timedelta

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox,
    QPushButton, QScrollArea, QWidget, QFileDialog, QMessageBox,
)

from models.base import Session
from models.outing import Outing
from ui.style import TEXT_MUTED

RECENT_DAYS = 7


class WeekendPdfDialog(QDialog):
    def __init__(self, parent, weekend):
        super().__init__(parent)
        self.weekend = weekend
        self.setWindowTitle("Export Weekend PDF")
        self.setModal(True)
        self.resize(480, 440)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        title = QLabel(f"Select outings to include — {weekend.track} {weekend.year}")
        layout.addWidget(title)

        hint = QLabel(f"Outings from the last {RECENT_DAYS} days are pre-checked; adjust freely.")
        hint.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        layout.addWidget(hint)

        session = Session()
        outings = (
            session.query(Outing)
            .filter(Outing.race_weekend_id == weekend.id)
            .order_by(Outing.date_time.desc())
            .all()
        )
        # Scalar columns are already loaded by the query above; only the
        # (unused here) relationships would need the session kept open.
        self._outing_rows = [
            (o.id, o.number, o.name, o.date_time) for o in outings
        ]
        session.close()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(4)

        self.checkboxes = {}
        cutoff = datetime.now() - timedelta(days=RECENT_DAYS)
        for outing_id, number, name, date_time in self._outing_rows:
            date_str = date_time.strftime("%d.%m.%Y %H:%M") if date_time else "—"
            label = f"#{number or '-'}  {name or '(unnamed)'}  —  {date_str}"
            cb = QCheckBox(label)
            cb.setChecked(bool(date_time and date_time >= cutoff))
            self.checkboxes[outing_id] = cb
            content_layout.addWidget(cb)
        content_layout.addStretch()

        if not self._outing_rows:
            content_layout.addWidget(QLabel("No outings in this weekend."))

        scroll.setWidget(content)
        layout.addWidget(scroll)

        btn_row = QHBoxLayout()
        btn_cancel = QPushButton("Cancel")
        btn_cancel.setStyleSheet("background-color: #252525; color: #888;")
        btn_cancel.clicked.connect(self.reject)
        btn_export = QPushButton("Export…")
        btn_export.clicked.connect(self._on_export)
        btn_row.addStretch()
        btn_row.addWidget(btn_cancel)
        btn_row.addWidget(btn_export)
        layout.addLayout(btn_row)

    def _on_export(self):
        selected_ids = [oid for oid, cb in self.checkboxes.items() if cb.isChecked()]
        if not selected_ids:
            QMessageBox.information(self, "No outings selected",
                                     "Select at least one outing to export.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"{self.weekend.track}_{self.weekend.year}_Weekend_{timestamp}.pdf"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Weekend PDF", default_name, "PDF Files (*.pdf)",
            options=QFileDialog.Option.DontConfirmOverwrite,
        )
        if not path:
            return
        if not path.endswith(".pdf"):
            path += ".pdf"
        if os.path.exists(path):
            reply = QMessageBox.question(
                self, "File exists",
                f"{os.path.basename(path)} already exists. Do you want to replace it?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if reply == QMessageBox.StandardButton.No:
                base = path[:-4]
                counter = 2
                while os.path.exists(f"{base}_{counter}.pdf"):
                    counter += 1
                path = f"{base}_{counter}.pdf"

        session = Session()
        selected_outings = (
            session.query(Outing)
            .filter(Outing.id.in_(selected_ids))
            .all()
        )
        # Same pattern as OutingsView._open_edit_outing: read the ORM rows
        # while the session is open, close it, then hand the (already
        # scalar-loaded) objects to code that runs after close -- no
        # relationship access happens post-close anywhere downstream
        # (generate_weekend_pdf resolves driver name/level via its own
        # fresh Session, never outing.driver).
        session.close()

        from core.weekend_pdf_export import generate_weekend_pdf
        try:
            generate_weekend_pdf(self.weekend, selected_outings, path)
        except PermissionError:
            QMessageBox.warning(
                self, "Save failed",
                f"Could not save {os.path.basename(path)}.\nThe file may be open in another program.",
            )
            return
        except Exception as e:
            QMessageBox.critical(
                self, "Export failed",
                f"Could not build the PDF: {e!r}",
            )
            return

        QMessageBox.information(self, "Export complete", f"Saved {os.path.basename(path)}.")
        self.accept()
