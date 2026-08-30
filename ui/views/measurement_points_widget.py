# Splitter/diffuser floor-referenced measurement point diagrams.
# Draws a fixed outline (curved-leading-edge blade for the splitter,
# rectangle for the diffuser) with small numbered input boxes at the
# physical measurement positions -- front-up orientation (top of the
# widget = front of the car), matching the 2x2 wheel-grid convention used
# everywhere else in this form.
# Point positions are the single shared table in core/setup_data_points.py
# (also used by core/pdf_export.py) -- extracted programmatically from the
# user's own annotated reference screenshot (2026-08-30, pure-green marker
# detection), not estimated. See thesis_notes.md "8. Splitter/diffuser
# measurement points, position re-extraction" for the numbering rationale
# and the pixel-space extraction this was computed from.

from PyQt6.QtWidgets import QWidget, QLineEdit
from PyQt6.QtGui import QPainter, QPen, QColor, QPainterPath
from PyQt6.QtCore import Qt, QRectF

from ui.style import TEXT_MUTED, BORDER, PANEL_ALT
from core.setup_data_points import SPLITTER_POINT_POSITIONS, DIFFUSER_POINT_POSITIONS

BOX_W = 34
BOX_H = 20
# Must be >= max(BOX_W, BOX_H) / 2 -- some points sit ON the outline's own
# edge (splitter point 5 at fy=0, diffuser's bottom row near fy=0.96), so
# a box centred there needs this margin to avoid clipping the widget's
# own bounds, not just cosmetic breathing room.
MARGIN = 20
# Fraction of the splitter shape's height, measured from the rear (bottom),
# where the straight sides end and the curved leading edge begins.
SPLITTER_SIDE_FRACTION = 0.45


class MeasurementPointsWidget(QWidget):
    """outline: 'splitter' or 'diffuser', selects the drawn shape only --
    both use the same positioned-QLineEdit mechanism. point_widgets (in
    point-number order) are the actual QLineEdit instances; the caller
    registers them into the form's own input dict, same as any other
    field, so the existing generic collect/load dispatch handles them
    with no special case.
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
        """Wide, shallow plan-view with a curved leading edge (front) --
        straight rear edge and sides, a single cubic-bezier arc sweeping
        across the front instead of the four-corner rounded-rect used
        before this pass (that read as a generic rounded blob, not a
        splitter silhouette).
        """
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

        # Front-up orientation cue: a short arrow at the top edge, matching
        # the wheel-grid convention (front axle drawn at the top) without
        # relying on drawText -- text rendering was unreliable under the
        # offscreen Qt platform used for this session's own screenshots.
        painter.setPen(QPen(QColor(TEXT_MUTED), 1.5))
        cx = rect.center().x()
        top = rect.top() - 4
        painter.drawLine(int(cx), int(top), int(cx), int(top - 8))
        painter.drawLine(int(cx), int(top - 8), int(cx - 4), int(top - 3))
        painter.drawLine(int(cx), int(top - 8), int(cx + 4), int(top - 3))
        painter.end()
