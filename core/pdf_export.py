# Setup/setdown sheet PDF, landscape A4, monochrome. One strip renderer
# (build_session_strip) at two scales: "large" = one sheet per page,
# "small" = four strips per weekend page (weekend_pdf_export).
# Rebrand by swapping config/images/team_logo.png.

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
from reportlab.pdfbase.pdfmetrics import stringWidth
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
    "wing_position": "Wing", "arb_front_mount": "ARB Fr.",
    "splitter_offset": "Splitter",
}
DIFF_TORQUE_LABEL = "Diff Locking Torque (measured, Nm)"
DIFF_TORQUE_POSITIONS = ["1", "2", "3", "4", "5"]
WEIGHT_TOTALS_LABELS = {
    "total_weight": "Total (kg)", "cross_percentage": "Cross %",
}

# splitter/diffuser point positions from core/setup_data_points.py (shared
# with the form widget)
from core.setup_data_points import SPLITTER_POINT_POSITIONS, DIFFUSER_POINT_POSITIONS

# where the splitter's straight sides end (fraction of height from the
# rear); must match the widget's SPLITTER_SIDE_FRACTION
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
    """Font presets for "large" and "small"; same layout, only sizes differ."""
    # One "value" size per scale, sized so a 6-digit value ("888888" /
    # "-88888") fits every value cell at its real width. No autofit: a cell
    # that doesn't fit gets wider (the *_FRAC constants), the number never
    # shrinks.
    if size == "large":
        f = dict(header=16, corner_title=16, core_label=12.5, value=9,
                  table_label=11.5,
                  section_title=14, notes=10)
        pad = 6.5
    else:
        f = dict(header=7.5, corner_title=7, core_label=5.6, value=6.5,
                  table_label=5,
                  section_title=6, notes=5.2)
        pad = 1.1
    # car_label uses the value size -- one number, can't drift
    f["car_label"] = f["value"]
    # spacer gaps scale with size -- fixed mm gaps at small scale made
    # KeepInFrame shrink the whole strip to ~1/4 width
    gap_lg = 2.5 * mm if size == "large" else 0.6 * mm
    gap_sm = 1.5 * mm if size == "large" else 0.4 * mm
    # Diagram height factor: only the height shrinks at small scale; the
    # diagram always spans the car-column width. Value box width comes from
    # the font, not the diagram, so boxes don't scale with the outline.
    # Checked: no two close-in-x boxes collide at the flatter height.
    diagram_height_scale = 1.0 if size == "large" else 0.36
    # leading = 1.2 x fontSize -- reportlab's flat 12 pt default collided
    # lines at large scale and bloated the strip at small scale
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
        "notes": ParagraphStyle("notes", fontSize=f["notes"], leading=_lead(f["notes"]),
                                 fontName="Helvetica", textColor=TEXT),
        "_pad": pad,
        "_fontsizes": f,
        "_gap_lg": gap_lg,
        "_gap_sm": gap_sm,
        "_diagram_height_scale": diagram_height_scale,
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


class MeasurementDiagram(Flowable):
    """Splitter/diffuser outline with each point's value in a bordered box at
    its position. Front up: fy = 0 is the top (front of car), as in the wheel
    grid and the form widget. reportlab is y-up (Qt is y-down) -> front =
    top of the local height.
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
        # half a box of margin on every side -- some points sit on or just past
        # the outline edge. Positions stay relative to the outline box.
        self.margin_x = box_w / 2
        self.margin_y = box_h / 2

    def wrap(self, availWidth, availHeight):
        return self.width + 2 * self.margin_x, self.height + 2 * self.margin_y

    def _splitter_path(self):
        """Plan view: straight rear and sides, one bezier arc across the front. Same
        shape as the widget's _splitter_path, in reportlab's y-up frame. Local to
        the outline box; draw() translates by the margins.
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
            # y up: fy = 0 (front) at the top
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


# Width fractions sized so a 6-digit "value" fits every cell at both
# scales (tightest: damper LS/HS). Changing one -> re-check all cells.
DAMPER_W_FRAC = 0.54  # of corner_w
DAMPER_LABEL_FRAC = 0.33  # of damper_w
ADVANCED_VAL_FRAC = 0.42  # of advanced_w
CAR_PARAM_VAL_FRAC = 0.26  # of car_col_w
DIFF_TORQUE_COLS = 3  # 5 across can't fit a 6-digit value


def _damper_table(data, styles, width):
    """Bump/Reb x LS/HS table, each label once. Blowoff row spans both columns
    (no LS/HS split).
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
    """One cell per locking-torque position (number over value), DIFF_TORQUE_COLS
    per row -- five across left no room for a 6-digit value.
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


# worst-case point value: signed, 3 digits + 1 decimal (mm offsets)
POINT_VALUE_WORST_CASE = "-999.9"


def _measurement_diagram_boxes(title, outline, positions, points, styles, width):
    """Outline with a value box at each position, both scales. (A values-only
    row overflowed its column at small scale -- rejected.)
    diagram_w = full car-column width; box_w from POINT_VALUE_WORST_CASE at
    the actual font size (same size as the table values). Only diagram_h
    shrinks for the small strip; checked box pairs don't touch.
    """
    font_size = styles["_fontsizes"]["value"]
    # 1.2x text width -- a box flush with the glyphs looks cramped in print
    box_w = stringWidth(POINT_VALUE_WORST_CASE, "Helvetica-Bold", font_size) * 1.2
    box_h = box_w * 0.6
    # diagram_w + box_w = width: the margin is where edge points' boxes live
    diagram_w = width - box_w
    aspect = 0.55 if outline == "splitter" else 0.6
    diagram_h = diagram_w * aspect * styles["_diagram_height_scale"]
    return [
        Paragraph(title, styles["car_label"]),
        MeasurementDiagram(outline, positions, points, diagram_w, diagram_h, box_w, box_h, font_size),
    ]


def _car_column(car, styles, width):
    """Narrow right column, cells sized to content. Splitter/diffuser as outline
    + value boxes at both scales. Wing position and ARB front mount are plain
    rows -- position codes ("P8"), a schematic added nothing.
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
    """One Setup or Setdown sheet at "large" (full page) or "small" (weekend
    strip) scale. data = setup/setdown dict (front_left .. rear_right, car);
    meta = header text fields.
    2x2 wheel grid (front up) as its own table, no spanning; car block as a
    narrow column beside it. KeepInFrame(shrink) keeps a long strip in its box.
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
    """Single-session sheet, one landscape page, "large" scale. Reads
    outing.setup_data for either sheet_type -- the caller puts the right
    data there.
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
            # corrupt setup_data -> visible note in the PDF, not a blank-looking sheet
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
        # reserve space for the banner -- stacking it on a full-page KeepInFrame
        # overflows the page
        warn_h = 8 * mm
        warn_style = ParagraphStyle("setup_data_error", fontSize=11, fontName="Helvetica-Bold",
                                     textColor=colors.HexColor("#c0392b"))
        story.append(Paragraph(
            f"Setup data could not be read ({parse_error}) -- sheet below is blank.", warn_style))
        story.append(Spacer(1, 3 * mm))
        strip_h -= warn_h
    story.append(build_session_strip(meta, setup, "large", strip_w, strip_h))
    doc.build(story)
