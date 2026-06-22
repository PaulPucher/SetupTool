# PDF export for the car setup sheet.
# Generates a printable A4 sheet from outing data, sized to fit one page.
# Light theme, minimal ink for printing.
# Swap config/images/team_logo.png or config/images/car_default.jpg to rebrand —
# no code changes needed.

import json
import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer, Image
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT
from PIL import Image as PILImage


PAGE_W, PAGE_H = A4
MARGIN = 10 * mm

ACCENT_HEX = "#2d6a35"
TEXT_HEX = "#111111"
MUTED_HEX = "#555555"

ACCENT = colors.HexColor(ACCENT_HEX)
TEXT = colors.HexColor(TEXT_HEX)
MUTED = colors.HexColor(MUTED_HEX)
LIGHT_GRAY = colors.HexColor("#f5f5f5")
MID_GRAY = colors.HexColor("#cccccc")
WHITE = colors.white

CAR_IMAGE_PATH = "config/images/car_default.jpg"
LOGO_PATH = "config/images/team_logo.png"

CORNER_LABELS = {
    "toe": "Toe (mm)", "camber": "Camber (°)",
    "ride_height_fia": "Ride Ht. FIA", "ride_height_aero": "Ride Ht. Aero",
    "arb": "ARB", "springs": "Springs (N/mm)",
}
ADVANCED_LABELS = {
    "packer": "Packer", "preload": "Preload",
    "total_travel": "Total Travel", "free_length": "Free Length",
    "static_droop": "Static Droop", "gap_on_gnd": "Gap on GND",
}
CAR_LABELS = {
    "differential_preload": "Diff Preload", "differential_position": "Diff Position",
    "wing_position": "Wing Pos.", "splitter_offset": "Splitter",
}
WEIGHT_TOTALS_LABELS = {
    "total_weight": "Total (kg)", "cross_percentage": "Cross %",
}


def _styles():
    return {
        "event": ParagraphStyle("event", fontSize=12, fontName="Helvetica-Bold",
                                 textColor=TEXT, alignment=TA_CENTER),
        "sheet_label": ParagraphStyle("sheet_label", fontSize=10, fontName="Helvetica-Bold",
                                       textColor=ACCENT, alignment=TA_CENTER, spaceBefore=1),
        "subtitle": ParagraphStyle("subtitle", fontSize=6.5, fontName="Helvetica",
                                    textColor=MUTED, alignment=TA_CENTER, spaceBefore=2),
        "left_name": ParagraphStyle("left_name", fontSize=8.5, fontName="Helvetica-Bold",
                                     textColor=TEXT, alignment=TA_LEFT),
        "left_info": ParagraphStyle("left_info", fontSize=6.5, fontName="Helvetica",
                                     textColor=MUTED, alignment=TA_LEFT),
        "corner_title": ParagraphStyle("corner_title", fontSize=7.5, fontName="Helvetica-Bold",
                                        textColor=ACCENT),
        "section_title": ParagraphStyle("section_title", fontSize=7, fontName="Helvetica-Bold",
                                         textColor=ACCENT),
        "label": ParagraphStyle("label", fontSize=6, fontName="Helvetica", textColor=MUTED),
        "value": ParagraphStyle("value", fontSize=6, fontName="Helvetica-Bold", textColor=TEXT),
        "damper_group": ParagraphStyle("damper_group", fontSize=5, fontName="Helvetica",
                                        textColor=MUTED),
        "notes_title": ParagraphStyle("notes_title", fontSize=7, fontName="Helvetica-Bold",
                                       textColor=ACCENT),
        "notes": ParagraphStyle("notes", fontSize=6.5, fontName="Helvetica", textColor=TEXT),
    }


def _fmt(value):
    if value is None or value == "":
        return "—"
    if isinstance(value, float) and value == 0.0:
        return "—"
    return str(value)


def _param_rows(data, label_map, styles):
    rows = []
    for key, label in label_map.items():
        rows.append([
            Paragraph(label, styles["label"]),
            Paragraph(_fmt(data.get(key, "")), styles["value"]),
        ])
    return rows


def _simple_table(rows, col_widths, styles):
    t = Table(rows, colWidths=col_widths)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), WHITE),
        ("ROWBACKGROUNDS", (0, 0), (-1, -1), [WHITE, LIGHT_GRAY]),
        ("TOPPADDING", (0, 0), (-1, -1), 1.2),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1.2),
        ("LEFTPADDING", (0, 0), (-1, -1), 3),
        ("RIGHTPADDING", (0, 0), (-1, -1), 3),
        ("GRID", (0, 0), (-1, -1), 0.25, MID_GRAY),
    ]))
    return t


def _corner_flowables(label, data, styles, col_w):
    inner_w = col_w * 0.94
    elements = [Paragraph(label, styles["corner_title"])]
    elements.append(_simple_table(_param_rows(data, CORNER_LABELS, styles),
                                   [inner_w * 0.6, inner_w * 0.4], styles))

    elements.append(Paragraph("DAMPER", styles["damper_group"]))
    half_w = inner_w / 2
    bump_rows = [[
        Paragraph("Bump LS", styles["label"]), Paragraph(_fmt(data.get("bump_ls", "")), styles["value"]),
        Paragraph("Bump HS", styles["label"]), Paragraph(_fmt(data.get("bump_hs", "")), styles["value"]),
    ]]
    elements.append(_simple_table(bump_rows, [half_w*0.55, half_w*0.45, half_w*0.55, half_w*0.45], styles))

    blowoff_rows = [[Paragraph("Blowoff", styles["label"]), Paragraph(_fmt(data.get("blowoff", "")), styles["value"])]]
    elements.append(_simple_table(blowoff_rows, [inner_w * 0.6, inner_w * 0.4], styles))

    reb_rows = [[
        Paragraph("Reb LS", styles["label"]), Paragraph(_fmt(data.get("rebound_ls", "")), styles["value"]),
        Paragraph("Reb HS", styles["label"]), Paragraph(_fmt(data.get("rebound_hs", "")), styles["value"]),
    ]]
    elements.append(_simple_table(reb_rows, [half_w*0.55, half_w*0.45, half_w*0.55, half_w*0.45], styles))

    elements.append(_simple_table(_param_rows(data, ADVANCED_LABELS, styles),
                                   [inner_w * 0.6, inner_w * 0.4], styles))
    return elements


def _scaled_image(path, target_width):
    with PILImage.open(path) as im:
        iw, ih = im.size
    height = target_width * (ih / iw)
    return Image(path, width=target_width, height=height)

def _weight_grid(car, styles, width):
    cell_style = ParagraphStyle("weight_cell", fontSize=6.5, fontName="Helvetica",
                                 textColor=TEXT, alignment=TA_CENTER, leading=9)

    def cell(label, value):
        return Paragraph(
            f'<font color="{MUTED_HEX}" size="5.5">{label}</font><br/>'
            f'<font color="{TEXT_HEX}" size="8"><b>{value}</b></font>',
            cell_style
        )

    rows = [
        [cell("FL", _fmt(car.get("corner_weight_fl", ""))), cell("FR", _fmt(car.get("corner_weight_fr", "")))],
        [cell("RL", _fmt(car.get("corner_weight_rl", ""))), cell("RR", _fmt(car.get("corner_weight_rr", "")))],
    ]
    grid = Table(rows, colWidths=[width / 2, width / 2])
    grid.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, MID_GRAY),
        ("BACKGROUND", (0, 0), (-1, -1), WHITE),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    return grid


def generate_setup_pdf(outing, weekend, output_path, sheet_type="Setup"):
    doc = SimpleDocTemplate(
        output_path, pagesize=A4,
        leftMargin=MARGIN, rightMargin=MARGIN, topMargin=MARGIN, bottomMargin=MARGIN
    )
    styles = _styles()
    story = []

    outing_name = getattr(outing, "name", "") or ""
    session_type = getattr(outing, "session_type", "") or ""
    driver_name = getattr(outing, "driver_name", "") or ""
    outing_number = getattr(outing, "number", "")

    left_cell = [
        Paragraph(outing_name or "—", styles["left_name"]),
        Paragraph(f"Car #{weekend.car_number}", styles["left_info"]),
        Paragraph(f"Outing {outing_number}", styles["left_info"]),
    ]

    sub_parts = [p for p in [session_type, outing.date_time.strftime('%d.%m.%Y %H:%M'), driver_name] if p]
    center_cell = [
        Paragraph(f"{weekend.track} — {weekend.series} {weekend.year}", styles["event"]),
        Paragraph(sheet_type.upper(), styles["sheet_label"]),
        Paragraph(" &nbsp;|&nbsp; ".join(sub_parts), styles["subtitle"]),
    ]

    right_cell = []
    if os.path.exists(LOGO_PATH):
        right_cell.append(_scaled_image(LOGO_PATH, 28 * mm))

    header_w = PAGE_W - 2 * MARGIN
    header_table = Table(
        [[left_cell, center_cell, right_cell]],
        colWidths=[header_w * 0.22, header_w * 0.56, header_w * 0.22]
    )
    header_table.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (2, 0), (2, 0), "RIGHT"),
        ("LINEBELOW", (0, 0), (-1, 0), 1.2, ACCENT),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.append(header_table)
    story.append(Spacer(1, 3 * mm))

    setup = {}
    if outing.setup_data:
        try:
            setup = json.loads(outing.setup_data)
        except Exception:
            pass

    fl = setup.get("front_left", {})
    fr = setup.get("front_right", {})
    rl = setup.get("rear_left", {})
    rr = setup.get("rear_right", {})
    car = setup.get("car", {})

    content_w = PAGE_W - 2 * MARGIN
    col_w = content_w * 0.30
    center_w = content_w - 2 * col_w

    left_col = []
    left_col.extend(_corner_flowables("FL — Front Left", fl, styles, col_w))
    left_col.append(Spacer(1, 2 * mm))
    left_col.extend(_corner_flowables("RL — Rear Left", rl, styles, col_w))

    right_col = []
    right_col.extend(_corner_flowables("FR — Front Right", fr, styles, col_w))
    right_col.append(Spacer(1, 2 * mm))
    right_col.extend(_corner_flowables("RR — Rear Right", rr, styles, col_w))

    center_col = []
    if os.path.exists(CAR_IMAGE_PATH):
        center_col.append(_scaled_image(CAR_IMAGE_PATH, center_w * 0.9))

    main_table = Table(
        [[left_col, center_col, right_col]],
        colWidths=[col_w, center_w, col_w]
    )
    main_table.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("ALIGN", (1, 0), (1, 0), "CENTER"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2),
    ]))
    story.append(main_table)
    story.append(Spacer(1, 3 * mm))

    bottom_w = content_w / 2
    inner_bottom_w = bottom_w * 0.9

    weights_col = [Paragraph("Weights", styles["section_title"])]
    weights_col.append(_weight_grid(car, styles, inner_bottom_w))
    weights_col.append(Spacer(1, 1.5 * mm))
    weights_col.append(_simple_table(_param_rows(car, WEIGHT_TOTALS_LABELS, styles),
                                      [inner_bottom_w * 0.55, inner_bottom_w * 0.45], styles))

    car_col = [Paragraph("Car", styles["section_title"])]
    car_col.append(_simple_table(_param_rows(car, CAR_LABELS, styles),
                                  [inner_bottom_w * 0.55, inner_bottom_w * 0.45], styles))

    bottom_table = Table([[weights_col, car_col]], colWidths=[bottom_w, bottom_w])
    bottom_table.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LINEABOVE", (0, 0), (-1, 0), 1, ACCENT),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 2),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2),
    ]))
    story.append(bottom_table)

    notes = car.get("notes", "")
    if notes:
        story.append(Spacer(1, 3 * mm))
        story.append(Paragraph("Setup Notes", styles["notes_title"]))
        story.append(Paragraph(notes, styles["notes"]))

    doc.build(story)