# Splitter/diffuser measurement-point diagrams: fixed outline with numbered
# input boxes at the measurement positions, front up (as the wheel grid).
# Positions from core/setup_data_points.py.

from PyQt6.QtWidgets import QWidget, QLineEdit
from PyQt6.QtGui import QPainter, QPen, QColor, QPainterPath
from PyQt6.QtCore import Qt, QRectF

from ui.style import TEXT_MUTED, BORDER, PANEL_ALT
from core.setup_data_points import SPLITTER_POINT_POSITIONS, DIFFUSER_POINT_POSITIONS

BOX_W = 34
BOX_H = 20
# >= max(BOX_W, BOX_H) / 2 -- some points sit on the outline edge
MARGIN = 20
# where the splitter's straight sides end (fraction of height from the rear)
SPLITTER_SIDE_FRACTION = 0.45


class MeasurementPointsWidget(QWidget):
    """outline: 'splitter' or 'diffuser' (shape only). point_widgets are the
    QLineEdits in point order; the caller registers them in the form's input
    dict like any other field.
    """

    def __init__(self, outline, parent=None):
        super().__init__(parent)
        self.outline = outline
        self.positions = SPLITTER_POINT_POSITIONS if outline == "splitter" else DIFFUSER_POINT_POSITIONS
        self.point_widgets = []
        for i in range(len(self.positions)):
            edit = QLineEdit(self)
            edit.setFixedSize(BOX_W, BOX_H)
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setPlaceholderText(str(i + 1))
            edit.setStyleSheet("font-size: 10px; padding: 0px;")
            edit.setToolTip(f"Point {i + 1} -- measured, vs floor (mm)")
            self.point_widgets.append(edit)
        self.setMinimumSize(220, 150 if outline == "splitter" else 170)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        w = max(self.width() - 2 * MARGIN, 1)
        h = max(self.height() - 2 * MARGIN, 1)
        for (fx, fy), edit in zip(self.positions, self.point_widgets):
            x = MARGIN + fx * w - BOX_W / 2
            y = MARGIN + fy * h - BOX_H / 2
            edit.move(int(x), int(y))

    def _splitter_path(self, rect):
        """Plan view: straight rear and sides, one bezier arc across the front."""
        side_y = rect.bottom() - rect.height() * SPLITTER_SIDE_FRACTION
        path = QPainterPath()
        path.moveTo(rect.left(), rect.bottom())
        path.lineTo(rect.right(), rect.bottom())
        path.lineTo(rect.right(), side_y)
        path.cubicTo(rect.right(), rect.top(), rect.left(), rect.top(), rect.left(), side_y)
        path.closeSubpath()
        return path

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(QColor(BORDER))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(QColor(PANEL_ALT))

        w = self.width() - 2 * MARGIN
        h = self.height() - 2 * MARGIN
        rect = QRectF(MARGIN, MARGIN, w, h)

        if self.outline == "splitter":
            painter.drawPath(self._splitter_path(rect))
        else:
            painter.drawRect(rect)

        # front-up cue as an arrow, not text (drawText unreliable offscreen)
        painter.setPen(QPen(QColor(TEXT_MUTED), 1.5))
        cx = rect.center().x()
        top = rect.top() - 4
        painter.drawLine(int(cx), int(top), int(cx), int(top - 8))
        painter.drawLine(int(cx), int(top - 8), int(cx - 4), int(top - 3))
        painter.drawLine(int(cx), int(top - 8), int(cx + 4), int(top - 3))
        painter.end()
