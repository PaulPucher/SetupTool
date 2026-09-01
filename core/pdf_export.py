# PDF export for the car setup/setdown sheet.
# Landscape A4, monochrome. One shared strip renderer (build_session_strip)
# is reused at two scales: "large" fills a whole page here (single-session
# print); "small" is reused by core/weekend_pdf_export.py to pack four
# strips (two outings' Setup/Setdown pairs) onto one landscape page. Keeping
# one renderer for both is the point -- see thesis_notes.md "PDF layout
# rework: shared strip renderer".
# Swap config/images/team_logo.png to rebrand - no code changes needed.

import json
import os
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer, Image,
    Flowable, KeepInFrame,
)
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT
from PIL import Image as PILImage


PAGE_W, PAGE_H = landscape(A4)
MARGIN = 10 * mm

TEXT_HEX = "#111111"
MUTED_HEX = "#555555"

TEXT = colors.HexColor(TEXT_HEX)
MUTED = colors.HexColor(MUTED_HEX)
LIGHT_GRAY = colors.HexColor("#f5f5f5")
MID_GRAY = colors.HexColor("#999999")
WHITE = colors.white
BLACK = colors.black

LOGO_PATH = "config/images/team_logo.png"

# Core row: readable words, one bordered cell per value.
CORNER_LABELS = {
    "toe": "Toe (mm)", "camber": "Camber (deg)",
    "ride_height_fia": "Ride Ht. FIA", "ride_height_aero": "Ride Ht. Aero",
    "arb": "ARB", "springs": "Springs (N/mm)",
}
# Damper block: a real LS/HS x Bump/Reb table, not label-per-cell.
DAMPER_ROWS = [("Bump", "bump_ls", "bump_hs"), ("Reb", "rebound_ls", "rebound_hs")]
BLOWOFF_KEY = "blowoff"
# Advanced fields: label/value pairs, full words, never abbreviated.
ADVANCED_LABELS = {
    "packer": "Packer", "preload": "Preload",
    "total_travel": "Total Travel", "free_length": "Free Length",
    "static_droop": "Static Droop", "gap_on_gnd": "Gap on GND",
}
CAR_LABELS = {
    "differential_preload": "Diff Preload", "differential_position": "Diff Position",
    "splitter_offset": "Splitter",
}
DIFF_TORQUE_LABEL = "Diff Locking Torque (measured, Nm)"
DIFF_TORQUE_POSITIONS = ["1", "2", "3", "4", "5"]
WEIGHT_TOTALS_LABELS = {
    "total_weight": "Total (kg)", "cross_percentage": "Cross %",
}
# Marked-position schematics -- splitter_offset stays a plain numeric cell
# in CAR_LABELS above; wing_position/arb_front_mount are the only fields
# with a real legal-position set to draw. Sourced from config/setup_
# parameters.json's own registry entries / car_data.json's
# wing_position_table, not invented here.
WING_POSITIONS = ["P8", "P9", "P10"]
ARB_MOUNT_POSITIONS = ["P0", "P1", "P2"]

# Splitter/diffuser floor-referenced measurement points (thesis_notes.md
# "8. Splitter/diffuser measurement points"). Positions come from the one
# shared table in core/setup_data_points.py -- both this file and ui/
# views/measurement_points_widget.py import from there now (both live in
# a package core/ can reach; the earlier hand-duplicated copy was only
# needed because core/ cannot import the PyQt6 widget module directly).
from core.setup_data_points import SPLITTER_POINT_POSITIONS, DIFFUSER_POINT_POSITIONS

# Fraction of the splitter shape's height, from the rear, where the
# straight sides end and the curved leading edge begins -- must match
# ui/views/measurement_points_widget.py's SPLITTER_SIDE_FRACTION so the
# form and the PDF draw the same silhouette.
SPLITTER_SIDE_FRACTION = 0.45


def _fmt(value):
    if value is None or value == "":
        return "-"
    if isinstance(value, float):
        if value == int(value):
            return str(int(value))
        return f"{value:.2f}".rstrip("0").rstrip(".")
    return str(value)


def _strip_styles(size):
    """Two font-size presets sharing one layout -- 'large' fills a whole
    landscape page (single-session print) at type sized to use most of
    the page, 'small' is one of four strips on a weekend page. Values
    only differ in point size, never in what's shown.
    """
    # A3 revision: every numeric value cell (core row, damper, advanced
    # list, diff torque, weight grid, car params) now shares ONE fixed
    # "value" font size per scale, sized so a 6-digit value ("888888" /
    # "-88888") fits every cell at its ACTUAL width with no shrinking --
    # verified against every value-cell width formula in this file (see
    # thesis_notes.md). No dynamic per-value autofit: a cell that can't
    # fit at this size is a layout bug, fixed by widening the cell (see
    # the DAMPER_W_FRAC/DAMPER_LABEL_FRAC/ADVANCED_VAL_FRAC/CAR_PARAM_
    # VAL_FRAC constants and the diff-torque row's 3-across layout below),
    # never by shrinking the number.
    if size == "large":
        f = dict(header=16, corner_title=16, core_label=12.5, value=9,
                  table_label=11.5, car_label=11,
                  section_title=14, schematic=9, notes=10)
        pad = 6.5
    else:
        f = dict(header=7.5, corner_title=7, core_label=5.6, value=6.5,
                  table_label=5, car_label=5.4,
                  section_title=6, schematic=4.6, notes=5.2)
        pad = 1.1
    # Cleanup pass, Phase 3: several inter-block Spacer heights below used
    # to be hardcoded in millimetres regardless of `size` -- fine at
    # "large" scale, but at "small" scale (STRIP_H=44mm, four strips per
    # weekend page) those same fixed gaps ate a disproportionate share of
    # an already tight height budget relative to the font sizes actually
    # shrinking around them. That mismatch was the real driver behind
    # KeepInFrame's shrink search collapsing each strip -- including the
    # Car column -- to roughly a quarter of its intended width (confirmed
    # by rendering the real Dubai weekend sheet: content stopped at ~30mm
    # of a ~269mm-wide strip). Scaled to the same large/small ratio as
    # `pad` above so gaps shrink in proportion to everything around them.
    gap_lg = 2.5 * mm if size == "large" else 0.6 * mm
    gap_sm = 1.5 * mm if size == "large" else 0.4 * mm
    # Cleanup pass, Phase 3, same finding as gap_lg/gap_sm above but for
    # the splitter/diffuser MeasurementDiagram flowables specifically:
    # _measurement_diagram_boxes sizes them from the car column's own
    # WIDTH alone (car_col_w is nearly the same in mm at both scales --
    # only STRIP_H shrinks, from ~190mm large to 44mm small), so the two
    # diagrams alone rendered at ~33mm+36mm tall regardless of `size`,
    # already exceeding the entire "small" strip height budget before
    # counting anything else in the car column. diagram_scale shrinks
    # their footprint directly instead of relying on KeepInFrame's own
    # (until now catastrophic) uniform shrink to compensate.
    diagram_scale = 1.0 if size == "large" else 0.4
    # Cleanup pass, Phase 3: ParagraphStyle's own `leading` (line-box
    # height) defaults to a FLAT 12pt regardless of fontSize when left
    # unset -- every style below left it unset. At "large" scale
    # (fontSize up to 16) that under-sizes the box relative to the glyph
    # ink, so the next stacked Paragraph (e.g. the header's own muted
    # subtitle line, or a section_title sitting right above its table)
    # visually collides with the one above it. At "small" scale (fontSize
    # down to 4.6) the same flat 12pt is far MORE than needed, over-
    # sizing every label/value pair's box -- inflating the whole strip's
    # natural height well past what 4-per-page (STRIP_H, core/weekend_
    # pdf_export.py) budgets for, which is what forced KeepInFrame's
    # shrink search to collapse the entire strip (car column included)
    # down to roughly a quarter of its intended width, confirmed by
    # rendering the real Dubai weekend sheet before this fix. Standard
    # typographic ratio (1.2x fontSize), applied uniformly instead of
    # reportlab's flat constant, fixes both symptoms with one change.
    def _lead(size):
        return size * 1.2

    header_muted_size = f["header"] * 0.6
    styles = {
        "header": ParagraphStyle("header", fontSize=f["header"], leading=_lead(f["header"]),
                                  fontName="Helvetica-Bold", textColor=TEXT),
        "header_muted": ParagraphStyle("header_muted", fontSize=header_muted_size,
                                        leading=_lead(header_muted_size),
                                        fontName="Helvetica", textColor=MUTED),
        "sheet_label": ParagraphStyle("sheet_label", fontSize=f["header"], leading=_lead(f["header"]),
                                       fontName="Helvetica-Bold", textColor=TEXT, alignment=TA_RIGHT),
        "corner_title": ParagraphStyle("corner_title", fontSize=f["corner_title"],
                                        leading=_lead(f["corner_title"]),
                                        fontName="Helvetica-Bold", textColor=TEXT),
        "core_label": ParagraphStyle("core_label", fontSize=f["core_label"], leading=_lead(f["core_label"]),
                                      fontName="Helvetica", textColor=MUTED, wordWrap=None),
        "value": ParagraphStyle("value", fontSize=f["value"], leading=_lead(f["value"]),
                                 fontName="Helvetica-Bold", textColor=TEXT, alignment=TA_CENTER),
        "table_label": ParagraphStyle("table_label", fontSize=f["table_label"], leading=_lead(f["table_label"]),
                                       fontName="Helvetica", textColor=MUTED),
        "table_head": ParagraphStyle("table_head", fontSize=f["table_label"], leading=_lead(f["table_label"]),
                                      fontName="Helvetica-Bold", textColor=TEXT, alignment=TA_CENTER),
        "car_label": ParagraphStyle("car_label", fontSize=f["car_label"], leading=_lead(f["car_label"]),
                                     fontName="Helvetica", textColor=MUTED),
        "section_title": ParagraphStyle("section_title", fontSize=f["section_title"],
                                         leading=_lead(f["section_title"]),
                                         fontName="Helvetica-Bold", textColor=TEXT,
                                         alignment=TA_LEFT),
        "schematic_label": ParagraphStyle("schematic_label", fontSize=f["schematic"],
                                           leading=_lead(f["schematic"]),
                                           fontName="Helvetica", textColor=MUTED),
        "schematic_box_font": f["schematic"],
        "notes": ParagraphStyle("notes", fontSize=f["notes"], leading=_lead(f["notes"]),
                                 fontName="Helvetica", textColor=TEXT),
        "_pad": pad,
        "_fontsizes": f,
        "_gap_lg": gap_lg,
        "_gap_sm": gap_sm,
        "_diagram_scale": diagram_scale,
    }
    return styles


def _bordered_table(rows, col_widths, styles, row_heights=None, pad_scale=1.0):
    table = Table(rows, colWidths=col_widths, rowHeights=row_heights)
    pad = styles["_pad"] * pad_scale
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), WHITE),
        ("TOPPADDING", (0, 0), (-1, -1), pad),
        ("BOTTOMPADDING", (0, 0), (-1, -1), pad),
        ("LEFTPADDING", (0, 0), (-1, -1), pad * 1.3),
        ("RIGHTPADDING", (0, 0), (-1, -1), pad * 1.3),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.4, MID_GRAY),
    ]))
    return table


class PositionSchematic(Flowable):
    """Marked-position schematic: a row of small tick-boxes, one per legal
    position, the active position filled solid black with its label
    reversed out in white -- garage-sheet size, not a banner. Plain
    reportlab canvas primitives, no new dependency.
    """

    def __init__(self, options, active, box_w, box_h, gap, font_size):
        super().__init__()
        self.options = options
        self.active = active
        self.box_w = box_w
        self.box_h = box_h
        self.gap = gap
        self.font_size = font_size
        self.width = len(options) * box_w + (len(options) - 1) * gap
        self.height = box_h

    def wrap(self, availWidth, availHeight):
        return self.width, self.height

    def draw(self):
        c = self.canv
        c.setLineWidth(0.5)
        for i, opt in enumerate(self.options):
            x = i * (self.box_w + self.gap)
            is_active = (opt == self.active)
            c.setStrokeColor(BLACK)
            c.setFillColor(BLACK if is_active else WHITE)
            c.rect(x, 0, self.box_w, self.box_h, stroke=1, fill=1)
            c.setFillColor(WHITE if is_active else TEXT)
            c.setFont("Helvetica-Bold" if is_active else "Helvetica", self.font_size)
            c.drawCentredString(x + self.box_w / 2, self.box_h / 2 - self.font_size * 0.35, opt)


class MeasurementDiagram(Flowable):
    """Splitter/diffuser floor-referenced check points, drawn as the
    outline shape (curved-leading-edge blade / rectangle) with each
    point's value in a small bordered box at its physical position --
    monochrome, matching the rest of the sheet. Front-up: fy=0 (in
    `positions`) is the top of the drawn shape, i.e. the front of the
    car, same convention as the 2x2 wheel grid and the UI widget this
    mirrors. reportlab's canvas is y-up (y=0 at the bottom) -- the
    opposite of Qt's y-down widget frame -- so "front" here is the TOP
    of the local height, matching measurement_points_widget.py's
    _splitter_path by construction, not by coincidence.
    """

    def __init__(self, outline, positions, values, width, height, box_w, box_h, font_size):
        super().__init__()
        self.outline = outline
        self.positions = positions
        self.values = values
        self.width = width
        self.height = height
        self.box_w = box_w
        self.box_h = box_h
        self.font_size = font_size
        # Some points sit exactly on (or, per the reference annotation,
        # just past) the outline's own edge -- e.g. splitter point 5 at
        # fy=0, or diffuser's bottom row at fy~0.96. Reserve half a box on
        # every side so those boxes never clip the flowable's own bounds;
        # positions stay normalised to the OUTLINE's box (self.width/
        # self.height), not the padded total this Flowable occupies.
        self.margin_x = box_w / 2
        self.margin_y = box_h / 2

    def wrap(self, availWidth, availHeight):
        return self.width + 2 * self.margin_x, self.height + 2 * self.margin_y

    def _splitter_path(self):
        """Wide, shallow plan-view with a curved leading edge (front) --
        straight rear edge and sides, one cubic-bezier arc across the
        front. Mirrors ui/views/measurement_points_widget.py's
        _splitter_path in reportlab's y-up frame (front = y=height here,
        vs. Qt's y=0 there). Coordinates are local to the outline's own
        box; draw() translates by (margin_x, margin_y) before painting.
        """
        side_y = self.height * SPLITTER_SIDE_FRACTION
        p = self.canv.beginPath()
        p.moveTo(0, 0)
        p.lineTo(self.width, 0)
        p.lineTo(self.width, side_y)
        p.curveTo(self.width, self.height, 0, self.height, 0, side_y)
        p.close()
        return p

    def draw(self):
        c = self.canv
        c.saveState()
        c.translate(self.margin_x, self.margin_y)
        c.setStrokeColor(BLACK)
        c.setFillColor(WHITE)
        c.setLineWidth(0.75)
        if self.outline == "splitter":
            c.drawPath(self._splitter_path(), stroke=1, fill=1)
        else:
            c.rect(0, 0, self.width, self.height, stroke=1, fill=1)

        for (fx, fy), value in zip(self.positions, self.values):
            # canvas y grows upward; fy=0 (front) is the TOP of the shape.
            cx = fx * self.width
            cy = (1 - fy) * self.height
            x0, y0 = cx - self.box_w / 2, cy - self.box_h / 2
            c.setStrokeColor(BLACK)
            c.setFillColor(WHITE)
            c.rect(x0, y0, self.box_w, self.box_h, stroke=1, fill=1)
            c.setFillColor(TEXT)
            c.setFont("Helvetica-Bold", self.font_size)
            c.drawCentredString(cx, cy - self.font_size * 0.35, _fmt(value))
        c.restoreState()


# A3 revision: structural width fractions chosen (see thesis_notes.md)
# so the uniform "value" font (f["value"], _strip_styles) fits a 6-digit
# value in every one of these cells with margin, at both scales -- solved
# once against the tightest cell (damper LS/HS, a 2-way split within an
# already-narrow sub-table) and re-checked against every other cell
# formula below. Changing any of these without re-checking that margin
# (core/pdf_export.py's own verification script, not committed) reopens
# the overflow bug this constant set closes.
DAMPER_W_FRAC = 0.54        # of corner_w, was 0.42
DAMPER_LABEL_FRAC = 0.33    # of damper_w, was 0.4
ADVANCED_VAL_FRAC = 0.42    # of advanced_w, was 0.38
CAR_PARAM_VAL_FRAC = 0.26   # of car_col_w, was 0.22
DIFF_TORQUE_COLS = 3        # was 5-across (one row); car_col_w/5 cannot
                             # hold a 6-digit value at any legible size


def _damper_table(data, styles, width):
    """Bump/Reb x LS/HS as a real small table -- each label written once,
    not repeated per cell. Blowoff is its own row, value spanning both
    data columns since it has no LS/HS split.
    """
    label_w = width * DAMPER_LABEL_FRAC
    val_w = (width - label_w) / 2
    rows = [
        [Paragraph("", styles["table_head"]), Paragraph("LS", styles["table_head"]),
         Paragraph("HS", styles["table_head"])],
    ]
    for row_label, k_ls, k_hs in DAMPER_ROWS:
        rows.append([
            Paragraph(row_label, styles["table_label"]),
            Paragraph(_fmt(data.get(k_ls, "")), styles["value"]),
            Paragraph(_fmt(data.get(k_hs, "")), styles["value"]),
        ])
    rows.append([
        Paragraph("Blowoff", styles["table_label"]),
        Paragraph(_fmt(data.get(BLOWOFF_KEY, "")), styles["value"]), "",
    ])
    t = _bordered_table(rows, [label_w, val_w, val_w], styles)
    t.setStyle(TableStyle([("SPAN", (1, 3), (2, 3))]))
    return t


def _advanced_list(data, styles, width):
    val_w = width * ADVANCED_VAL_FRAC
    label_w = width - val_w
    rows = [[Paragraph(label, styles["table_label"]), Paragraph(_fmt(data.get(key, "")), styles["value"])]
            for key, label in ADVANCED_LABELS.items()]
    return _bordered_table(rows, [label_w, val_w], styles)


def _corner_box(label, data, styles, width):
    title_gap = 2.5 * mm if styles["_fontsizes"]["corner_title"] >= 10 else 0.5 * mm
    elements = [Paragraph(label, styles["corner_title"]), Spacer(1, title_gap)]

    lw = width * 0.30
    vw = width * 0.20
    core_pairs = list(CORNER_LABELS.items())
    core_rows = []
    for i in range(0, len(core_pairs), 2):
        (k1, l1), (k2, l2) = core_pairs[i], core_pairs[i + 1]
        core_rows.append([
            Paragraph(l1, styles["core_label"]), Paragraph(_fmt(data.get(k1, "")), styles["value"]),
            Paragraph(l2, styles["core_label"]), Paragraph(_fmt(data.get(k2, "")), styles["value"]),
        ])
    elements.append(_bordered_table(core_rows, [lw, vw, lw, vw], styles))
    elements.append(Spacer(1, styles["_gap_lg"]))

    damper_w = width * DAMPER_W_FRAC
    advanced_w = width - damper_w - 2 * mm
    lower = Table(
        [[_damper_table(data, styles, damper_w), _advanced_list(data, styles, advanced_w)]],
        colWidths=[damper_w, advanced_w],
    )
    lower.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 0),
    ]))
    elements.append(lower)
    return elements


def _diff_torque_row(car, styles, width):
    """One bordered cell per measured locking-torque position (position
    number over value, same stacked-cell convention as before), wrapped
    at DIFF_TORQUE_COLS per row instead of one 5-across row -- five equal
    columns across the full car-column width left too little room for a
    6-digit value at the shared "value" font size (car_col_w/5 vs the
    2-column damper split's own already-tight car_col_w/2-ish budget);
    wrapping to rows of DIFF_TORQUE_COLS gives each cell car_col_w/
    DIFF_TORQUE_COLS instead, verified against the "value" font's needed
    width in the same check as every other cell in this file.
    """
    torque = car.get("differential_locking_torque_measured") or {}
    cell_w = width / DIFF_TORQUE_COLS
    positions = DIFF_TORQUE_POSITIONS
    rows = []
    for i in range(0, len(positions), DIFF_TORQUE_COLS):
        chunk = positions[i:i + DIFF_TORQUE_COLS]
        row = [[Paragraph(pos, styles["table_head"]), Paragraph(_fmt(torque.get(pos, "")), styles["value"])]
               for pos in chunk]
        row += [""] * (DIFF_TORQUE_COLS - len(chunk))
        rows.append(row)
    return _bordered_table(rows, [cell_w] * DIFF_TORQUE_COLS, styles)


def _weight_grid(car, styles, width):
    lw = width * 0.2
    vw = width * 0.3
    rows = [
        [Paragraph("FL", styles["car_label"]), Paragraph(_fmt(car.get("corner_weight_fl", "")), styles["value"]),
         Paragraph("FR", styles["car_label"]), Paragraph(_fmt(car.get("corner_weight_fr", "")), styles["value"])],
        [Paragraph("RL", styles["car_label"]), Paragraph(_fmt(car.get("corner_weight_rl", "")), styles["value"]),
         Paragraph("RR", styles["car_label"]), Paragraph(_fmt(car.get("corner_weight_rr", "")), styles["value"])],
    ]
    return _bordered_table(rows, [lw, vw, lw, vw], styles)


def _schematic_row(label, options, active, styles, width):
    box_w = 8 * mm
    box_h = 5 * mm
    gap = 1 * mm
    row = [
        Paragraph(label, styles["schematic_label"]),
        PositionSchematic(options, active, box_w, box_h, gap, styles["schematic_box_font"]),
    ]
    t = Table([row], colWidths=[width - (len(options) * box_w + (len(options) - 1) * gap) - 2 * mm, None])
    t.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 0),
        ("TOPPADDING", (0, 0), (-1, -1), 1),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1),
    ]))
    return t


def _measurement_diagram_boxes(title, outline, positions, points, styles, width):
    """Outline shape with a small value box at each physical position --
    same visual language as the setup-form widget, used at both scales.
    A values-only-row alternative (no outline, point-number header +
    value cells) was built and rendered at weekend scale for comparison
    and rejected: it overflowed its column width and displaced the
    neighbouring Wing/ARB schematic rendering, while this outline stayed
    legible and correctly bounded. See thesis_notes.md "8. Splitter/
    diffuser measurement points, Phase 3".
    """
    # box_frac sized so diagram_w + box_w (the value box's full width,
    # which MeasurementDiagram reserves as margin on each side for a
    # point centred right at fx=0/1) sums to exactly `width` -- some
    # points sit ON the outline's own edge (splitter point 5 at fy=0,
    # diffuser's bottom row near fy=0.96), so this margin isn't optional
    # headroom, it is where those boxes actually live.
    box_frac = 0.14
    diagram_w = width / (1 + box_frac) * styles["_diagram_scale"]
    aspect = 0.55 if outline == "splitter" else 0.6
    diagram_h = diagram_w * aspect
    box_w = diagram_w * box_frac
    box_h = box_w * 0.6
    font_size = max(styles["_fontsizes"]["value"] * 0.8, 4)
    return [
        Paragraph(title, styles["car_label"]),
        MeasurementDiagram(outline, positions, points, diagram_w, diagram_h, box_w, box_h, font_size),
    ]


def _car_column(car, styles, width):
    """Narrow right-hand column (Decision: car-level block is ONE column,
    sized to content, not a co-equal quadrant). Numeric cells are sized to
    their value, not stretched; schematics are one compact line each.
    Splitter/diffuser points always render as outline+value-boxes at both
    scales -- a values-only row alternative was built and rendered at
    weekend (small) scale for comparison and rejected: it overflowed its
    column width and even displaced the Wing/ARB schematic rendering,
    while the outline stayed legible and correctly bounded. See thesis_
    notes.md "8. Splitter/diffuser measurement points, Phase 3".
    """
    title_gap = 2.5 * mm if styles["_fontsizes"]["section_title"] >= 10 else 0.8 * mm
    elements = [Paragraph("Car", styles["section_title"]), Spacer(1, title_gap)]

    val_w = width * CAR_PARAM_VAL_FRAC
    label_w = width - val_w
    param_rows = [[Paragraph(label, styles["car_label"]),
                    Paragraph(_fmt(car.get(key, "")), styles["value"])]
                  for key, label in CAR_LABELS.items()]
    elements.append(_bordered_table(param_rows, [label_w, val_w], styles))
    elements.append(Spacer(1, styles["_gap_sm"]))

    elements.append(Paragraph(DIFF_TORQUE_LABEL, styles["car_label"]))
    elements.append(_diff_torque_row(car, styles, width))
    elements.append(Spacer(1, styles["_gap_sm"]))

    splitter_points = car.get("splitter_points") or [None] * len(SPLITTER_POINT_POSITIONS)
    diffuser_points = car.get("diffuser_points") or [None] * len(DIFFUSER_POINT_POSITIONS)
    elements.extend(_measurement_diagram_boxes("Splitter Pts (mm, vs floor)", "splitter",
                                                 SPLITTER_POINT_POSITIONS, splitter_points, styles, width))
    elements.append(Spacer(1, styles["_gap_sm"]))

    elements.append(_schematic_row("Wing", WING_POSITIONS, car.get("wing_position"), styles, width))
    elements.append(_schematic_row("ARB Fr.", ARB_MOUNT_POSITIONS, car.get("arb_front_mount"), styles, width))
    elements.append(Spacer(1, styles["_gap_sm"]))

    elements.extend(_measurement_diagram_boxes("Diffuser Pts (mm, vs floor)", "diffuser",
                                                 DIFFUSER_POINT_POSITIONS, diffuser_points, styles, width))
    elements.append(Spacer(1, styles["_gap_sm"]))

    elements.append(Paragraph("Weights (kg)", styles["section_title"]))
    elements.append(Spacer(1, title_gap))
    elements.append(_weight_grid(car, styles, width))
    totals_rows = [[Paragraph(label, styles["car_label"]),
                     Paragraph(_fmt(car.get(key, "")), styles["value"])]
                   for key, label in WEIGHT_TOTALS_LABELS.items()]
    elements.append(_bordered_table(totals_rows, [label_w, val_w], styles))
    return elements


def _scaled_image(path, target_height):
    with PILImage.open(path) as im:
        iw, ih = im.size
    width = target_height * (iw / ih)
    return Image(path, width=width, height=target_height)


def build_session_strip(meta, data, size, strip_w, strip_h):
    """One full Setup or Setdown sheet, at 'large' (single full page) or
    'small' (one of four weekend strips) scale. `data` is the parsed
    setup_data/setdown_data dict (front_left/front_right/rear_left/
    rear_right/car); `meta` carries the header text
    (number/name/session_type/date_str/driver_name/sheet_label).

    Layout: the 2x2 wheel grid (FL/FR top, RL/RR bottom -- front-up) is
    its own clean Table, no column spanning, so it can never clip against
    a neighbour; the car-level block is a single narrow column beside it.
    Wrapped in KeepInFrame(mode='shrink') so a strip that runs long
    shrinks to fit its allotted box rather than overflowing the page grid.
    """
    styles = _strip_styles(size)
    fl = data.get("front_left", {}) or {}
    fr = data.get("front_right", {}) or {}
    rl = data.get("rear_left", {}) or {}
    rr = data.get("rear_right", {}) or {}
    car = data.get("car", {}) or {}

    header_bits = [b for b in [meta.get("session_type"), meta.get("date_str"), meta.get("driver_name")] if b]
    left_header = [
        Paragraph(f"{meta.get('name') or ''} #{meta.get('number', '')}".strip(), styles["header"]),
        Paragraph("  |  ".join(header_bits), styles["header_muted"]),
    ]
    right_header = [Paragraph(meta.get("sheet_label", ""), styles["sheet_label"])]
    if os.path.exists(LOGO_PATH):
        try:
            logo_h = 6 * mm if size == "large" else 3 * mm
            right_header.insert(0, _scaled_image(LOGO_PATH, logo_h))
        except Exception:
            pass
    header_pad = 3 * mm if size == "large" else 1.5
    header_table = Table([[left_header, right_header]], colWidths=[strip_w * 0.75, strip_w * 0.25])
    header_table.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (1, 0), (1, 0), "RIGHT"),
        ("LINEBELOW", (0, 0), (-1, 0), 0.75, BLACK),
        ("TOPPADDING", (0, 0), (-1, -1), header_pad),
        ("BOTTOMPADDING", (0, 0), (-1, -1), header_pad),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 0),
    ]))

    wheel_grid_w = strip_w * 0.73
    car_col_w = strip_w - wheel_grid_w - 4 * mm
    corner_w = wheel_grid_w / 2

    wheel_grid = Table(
        [[_corner_box("FL", fl, styles, corner_w), _corner_box("FR", fr, styles, corner_w)],
         [_corner_box("RL", rl, styles, corner_w), _corner_box("RR", rr, styles, corner_w)]],
        colWidths=[corner_w, corner_w],
    )
    row_gap = 6 * mm if size == "large" else 1.5 * mm
    wheel_grid.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2),
        ("TOPPADDING", (0, 0), (-1, -1), 1),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1),
        ("BOTTOMPADDING", (0, 0), (-1, 0), row_gap),
        ("TOPPADDING", (0, 1), (-1, 1), row_gap),
    ]))

    body = Table([[wheel_grid, _car_column(car, styles, car_col_w)]],
                 colWidths=[wheel_grid_w, car_col_w])
    body.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 0),
    ]))

    header_gap = 4 * mm if size == "large" else 1 * mm
    content = [header_table, Spacer(1, header_gap), body]

    notes = car.get("notes", "")
    if notes:
        content.append(Spacer(1, 1 * mm))
        content.append(Paragraph(f"Notes: {notes}", styles["notes"]))

    return KeepInFrame(strip_w, strip_h, content, mode="shrink")


def generate_setup_pdf(outing, weekend, output_path, sheet_type="Setup"):
    """Single-session sheet -- one call, one sheet_type, one landscape
    page, the shared strip renderer at 'large' scale. Reads outing.setup_
    data regardless of sheet_type (the caller, ui/views/outing_form.py's
    _print_sheet, already puts whichever data applies under that one
    attribute name) -- unchanged calling contract, layout only.
    """
    doc = SimpleDocTemplate(
        output_path, pagesize=(PAGE_W, PAGE_H),
        leftMargin=MARGIN, rightMargin=MARGIN, topMargin=MARGIN, bottomMargin=MARGIN
    )

    setup = {}
    parse_error = None
    if outing.setup_data:
        try:
            setup = json.loads(outing.setup_data)
        except Exception as e:
            # Reliability pass: this used to swallow the error and render
            # a fully blank-but-otherwise-normal-looking sheet -- zero
            # indication anywhere that the stored setup_data was corrupt
            # rather than genuinely empty. A visible note in the PDF
            # itself, same "surface it in the document, don't abort the
            # export" convention core/weekend_pdf_export.py's own per-
            # outing error handling already uses.
            from core.error_text import friendly_error_text
            parse_error = friendly_error_text(e)

    meta = {
        "number": getattr(outing, "number", ""),
        "name": getattr(outing, "name", "") or "",
        "session_type": getattr(outing, "session_type", "") or "",
        "date_str": outing.date_time.strftime("%d.%m.%Y %H:%M") if getattr(outing, "date_time", None) else "",
        "driver_name": getattr(outing, "driver_name", "") or "",
        "sheet_label": f"{weekend.track} - {sheet_type.upper()}",
    }

    strip_w = PAGE_W - 2 * MARGIN
    strip_h = PAGE_H - 2 * MARGIN
    story = []
    if parse_error is not None:
        # Reserve real space for the banner instead of stacking it in
        # front of a KeepInFrame already sized to the FULL page -- that
        # would push the frame past the bottom margin and overflow the
        # page, trading one Phase 3 overlap bug for a new one.
        warn_h = 8 * mm
        warn_style = ParagraphStyle("setup_data_error", fontSize=11, fontName="Helvetica-Bold",
                                     textColor=colors.HexColor("#c0392b"))
        story.append(Paragraph(
            f"Setup data could not be read ({parse_error}) -- sheet below is blank.", warn_style))
        story.append(Spacer(1, 3 * mm))
        strip_h -= warn_h
    story.append(build_session_strip(meta, setup, "large", strip_w, strip_h))
    doc.build(story)
