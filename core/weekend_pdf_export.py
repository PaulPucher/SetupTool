# Weekend PDF export (Tier C UI feature, approved proposal + three
# corrections). One multi-page document covering a user-selected subset of
# a race weekend's outings: a cover summary page, then one section per
# outing (setup sheet, resolved accuracy footer, verdict summary,
# recommendations, driver feedback). reportlab tables throughout -- no
# corner-grid card layout (that style is core/pdf_export.py's single-
# outing setup sheet, a different document with a different purpose;
# CORNER_LABELS/ADVANCED_LABELS/CAR_LABELS/WEIGHT_TOTALS_LABELS/_fmt are
# reused from there so the parameter-label mapping has one source).
#
# Verdict trust rule (Guard-A/B consistency): a verdict is only ever
# printed for an outing whose cached analysis_data.schema_version matches
# the CURRENT modules.stability_analysis.ANALYSIS_SCHEMA_VERSION, and even
# then it is classified LIVE from current config (OutingForm._classify_
# corner) against the stored summaries -- never a value read out of the
# cache. This mirrors exactly how ui/views/outing_form.py's own render
# path works (Guard A: verdicts never persisted, always classified live;
# Guard B: a schema-version mismatch is treated as no cache at all). A
# stale-schema or missing analysis prints "not analysed under current
# version - re-run Analyse" instead of any verdict/recommendation section.

import json
from xml.sax.saxutils import escape

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer, PageBreak
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER

from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
from modules.recommendation import (
    aggregate_by_corner, generate_recommendations, load_recommendations_config,
)
from models.base import Session
from models.driver import Driver

PAGE_W, PAGE_H = A4
MARGIN = 14 * mm
CONTENT_W = PAGE_W - 2 * MARGIN

ACCENT_HEX = "#2d6a35"
ACCENT = colors.HexColor(ACCENT_HEX)
TEXT = colors.HexColor("#111111")
MUTED = colors.HexColor("#555555")
BAD = colors.HexColor("#c0392b")
WARN_COLOR = colors.HexColor("#9a6b00")
HEADER_BG = colors.HexColor("#e8f0e9")
LIGHT_GRAY = colors.HexColor("#f5f5f5")
MID_GRAY = colors.HexColor("#cccccc")
WHITE = colors.white


def _styles():
    return {
        "title": ParagraphStyle("title", fontSize=16, fontName="Helvetica-Bold",
                                 textColor=TEXT, alignment=TA_LEFT, spaceAfter=2),
        "subtitle": ParagraphStyle("subtitle", fontSize=10, fontName="Helvetica",
                                    textColor=MUTED, alignment=TA_LEFT),
        "h1": ParagraphStyle("h1", fontSize=13, fontName="Helvetica-Bold",
                              textColor=ACCENT, alignment=TA_LEFT, spaceBefore=4, spaceAfter=2),
        "h2": ParagraphStyle("h2", fontSize=10, fontName="Helvetica-Bold",
                              textColor=ACCENT, alignment=TA_LEFT, spaceBefore=6, spaceAfter=2),
        "body": ParagraphStyle("body", fontSize=8.5, fontName="Helvetica", textColor=TEXT),
        "muted": ParagraphStyle("muted", fontSize=8, fontName="Helvetica", textColor=MUTED),
        "warn": ParagraphStyle("warn", fontSize=9, fontName="Helvetica-Bold", textColor=WARN_COLOR),
        "error": ParagraphStyle("error", fontSize=9, fontName="Helvetica-Bold", textColor=BAD),
        "cell": ParagraphStyle("cell", fontSize=7.5, fontName="Helvetica", textColor=TEXT, leading=9),
        "cell_head": ParagraphStyle("cell_head", fontSize=7.5, fontName="Helvetica-Bold",
                                     textColor=TEXT, leading=9),
    }


def _p(value, style):
    if value is None:
        value = "—"
    return Paragraph(escape(str(value)), style)


def _table(rows, col_widths, styles, header=True):
    # header row bold/highlighted, plain grid otherwise -- "tables-only",
    # no corner-grid/card layout for this document.
    para_rows = []
    for i, row in enumerate(rows):
        style = styles["cell_head"] if (header and i == 0) else styles["cell"]
        para_rows.append([_p(cell, style) for cell in row])
    t = Table(para_rows, colWidths=col_widths, repeatRows=1 if header else 0)
    style_cmds = [
        ("GRID", (0, 0), (-1, -1), 0.4, MID_GRAY),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 2.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
    ]
    if header:
        style_cmds.append(("BACKGROUND", (0, 0), (-1, 0), HEADER_BG))
        style_cmds.append(("ROWBACKGROUNDS", (0, 1), (-1, -1), [WHITE, LIGHT_GRAY]))
    else:
        style_cmds.append(("ROWBACKGROUNDS", (0, 0), (-1, -1), [WHITE, LIGHT_GRAY]))
    t.setStyle(TableStyle(style_cmds))
    return t


def _classify_corner(summary):
    # Reuses the UI's own classifier unmodified -- OutingForm._classify_
    # corner never touches self (only load_parameters() + the summary
    # dict), the same None-self call already used by this session's
    # diagnostics/*.py scripts. Guarantees a PDF verdict can never disagree
    # with what the live app would show for the same summary.
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def _accuracy_footer_text(levels):
    from ui.views.outing_form import OutingForm
    if not levels:
        return ""
    parts = [f"{label} L{levels[node]}" for node, label in OutingForm._ACCURACY_FOOTER_LABELS
             if node in levels]
    return " | ".join(parts)


def _driver_name_and_level(driver_id):
    if driver_id is None:
        return None, None
    session = Session()
    driver = session.get(Driver, driver_id)
    name = driver.name if driver else None
    level = driver.driving_level if driver else None
    session.close()
    return name, level


def _load_json(raw):
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return {}


def analysis_status(outing):
    """'current' | 'stale' | 'absent'. The ONLY gate for whether a verdict/
    recommendation section may be printed at all -- see module docstring.
    """
    if not outing.analysis_data:
        return "absent"
    parsed = _load_json(outing.analysis_data)
    if parsed.get("schema_version") != ANALYSIS_SCHEMA_VERSION:
        return "stale"
    return "current"


_STATUS_LABELS = {
    "current": "Analysed (current)",
    "stale": "Stale (re-run Analyse)",
    "absent": "Not analysed",
}


def _setup_sheet_flowables(outing, styles):
    from core.pdf_export import CORNER_LABELS, ADVANCED_LABELS, CAR_LABELS, WEIGHT_TOTALS_LABELS, _fmt

    setup = _load_json(outing.setup_data)
    corners = {
        "FL": setup.get("front_left", {}) or {}, "FR": setup.get("front_right", {}) or {},
        "RL": setup.get("rear_left", {}) or {}, "RR": setup.get("rear_right", {}) or {},
    }
    rows = [["Parameter", "FL", "FR", "RL", "RR"]]
    for key, label in list(CORNER_LABELS.items()) + list(ADVANCED_LABELS.items()):
        rows.append([label] + [_fmt(corners[c].get(key, "")) for c in ("FL", "FR", "RL", "RR")])
    w = CONTENT_W
    flow = [_table(rows, [w * 0.34, w * 0.165, w * 0.165, w * 0.165, w * 0.165], styles)]

    car = setup.get("car", {}) or {}
    if car:
        car_rows = [["Car parameter", "Value"]]
        for key, label in list(CAR_LABELS.items()) + list(WEIGHT_TOTALS_LABELS.items()):
            car_rows.append([label, _fmt(car.get(key, ""))])
        flow.append(Spacer(1, 2 * mm))
        flow.append(_table(car_rows, [w * 0.6, w * 0.4], styles))
    return flow


def _verdict_flowables(summaries, styles):
    aggregated = aggregate_by_corner(summaries)
    rows = [["Corner", "Speed class", "Severity", "Verdict"]]
    for cid in sorted(aggregated.keys()):
        agg = aggregated[cid]
        severity, _short, long_v, _colour = _classify_corner(agg)
        rows.append([f"C{cid}", agg.get("speed_class") or "—", severity, long_v])
    w = CONTENT_W
    return [_table(rows, [w * 0.12, w * 0.16, w * 0.16, w * 0.56], styles)]


def _recommendations_flowables(outing, summaries, driving_level, styles):
    config = load_recommendations_config()
    feedback_data = _load_json(outing.feedback_data)
    setup_data = _load_json(outing.setup_data)

    results = generate_recommendations(
        summaries, _classify_corner, feedback_data, setup_data, config,
        driving_level=driving_level,
    )
    if not results:
        return [Paragraph("No recommendations at current thresholds.", styles["muted"])]

    # situational/provenance are per-RULE, not carried on the bucket result
    # directly -- cross-referenced by rules_fired, same pattern the UI uses
    # for its "unvalidated rule" (seed-status) check.
    rule_situational = {r["id"]: bool(r.get("situational")) for r in config["rules"]}
    rule_provenance = {r["id"]: r.get("elicitation_provenance") for r in config["rules"]}

    rows = [["Action", "Score", "Trigger", "Cell ID(s)", "Provenance", "Situational",
             "Class", "Selected", "Conflicts / limits"]]
    for r in results:
        if r["parameter"] is not None:
            action_text = f"{r['parameter']} {r['direction']}"
        else:
            action_text = " + ".join(
                f"{a['parameter']} -> {a['target']}" if "target" in a
                else f"{a['parameter']} {a['direction']}"
                for a in r["actions"]
            )
        provenances = sorted({(rule_provenance.get(rid) or "—") for rid in r["rules_fired"]})
        situational = any(rule_situational.get(rid) for rid in r["rules_fired"])
        action_class_text = "ADVISORY" if r["action_class"] == "advisory" else "RECOMMENDED"

        notes = []
        if r["conflicts"]:
            notes.append("driver/data disagree: " + ", ".join(
                f"C{c['stable_corner_id']}" for c in r["conflicts"]))
        if r["parameter_conflict"]:
            notes.append("param conflict: " + ", ".join(r["conflict_parameters"]))
        if r["limit_status"] == "at_limit":
            notes.append("AT LIMIT: " + ", ".join(r["at_limit_parameters"]))
        elif r["limit_status"] == "unchecked":
            notes.append("limit unchecked")

        rows.append([
            action_text, f"{r['score']:.2f}", "/".join(r["trigger_source"]),
            "/".join(r["cell_ids"]) or "—", "/".join(provenances),
            "yes" if situational else "no",
            ("SELECTED" if r["selected"] else action_class_text),
            "yes" if r["selected"] else "no",
            "; ".join(notes) or "—",
        ])
    w = CONTENT_W
    return [_table(rows, [w * 0.18, w * 0.06, w * 0.07, w * 0.09, w * 0.11,
                           w * 0.08, w * 0.11, w * 0.07, w * 0.23], styles)]


def _feedback_flowables(outing, styles):
    feedback_data = _load_json(outing.feedback_data)
    corners = feedback_data.get("corners", [])
    if not corners:
        return [Paragraph("No feedback entered.", styles["muted"])]
    any_nonzero = any(
        c.get("worst") or any(c.get(k, 0) for k in ("e1", "e2", "a3", "x4", "x5"))
        for c in corners
    )
    if not any_nonzero:
        return [Paragraph("No feedback entered (all zero).", styles["muted"])]
    rows = [["Corner", "Brake", "Turn-in", "Apex", "Exit 4", "Exit 5", "Worst"]]
    for i, c in enumerate(corners, 1):
        vals = [c.get(k, 0) for k in ("e1", "e2", "a3", "x4", "x5")]
        rows.append([f"C{i}"] + [str(v) for v in vals] + ["worst" if c.get("worst") else ""])
    w = CONTENT_W
    return [_table(rows, [w * 0.12] + [w * 0.15] * 5 + [w * 0.13], styles)]


def _outing_meta_line(outing, driver_name):
    bits = [b for b in [
        outing.date_time.strftime("%d.%m.%Y %H:%M") if outing.date_time else None,
        outing.session_type, driver_name,
        f"tyre {outing.tyre_type} ({outing.tyre_age} km)" if outing.tyre_type else None,
    ] if b]
    return " | ".join(bits)


def _build_outing_section(outing, styles):
    flow = []
    driver_name, driving_level = _driver_name_and_level(outing.driver_id)
    title = f"Outing {outing.number or outing.id}" + (f" — {outing.name}" if outing.name else "")
    flow.append(Paragraph(escape(title), styles["h1"]))
    flow.append(Paragraph(escape(_outing_meta_line(outing, driver_name)), styles["muted"]))
    flow.append(Spacer(1, 2 * mm))

    flow.append(Paragraph("Setup Sheet", styles["h2"]))
    flow.extend(_setup_sheet_flowables(outing, styles))
    flow.append(Spacer(1, 3 * mm))

    status = analysis_status(outing)
    flow.append(Paragraph("Analysis", styles["h2"]))
    if status != "current":
        flow.append(Paragraph("not analysed under current version - re-run Analyse",
                               styles["warn"]))
    else:
        parsed = _load_json(outing.analysis_data)
        summaries = parsed.get("summaries", [])
        footer = _accuracy_footer_text(parsed.get("resolved_levels"))
        if footer:
            flow.append(Paragraph("Resolved accuracy: " + escape(footer), styles["muted"]))
        flow.append(Spacer(1, 2 * mm))

        flow.append(Paragraph("Verdict Summary", styles["h2"]))
        flow.extend(_verdict_flowables(summaries, styles))
        flow.append(Spacer(1, 3 * mm))

        flow.append(Paragraph("Recommendations", styles["h2"]))
        flow.extend(_recommendations_flowables(outing, summaries, driving_level, styles))

    flow.append(Spacer(1, 3 * mm))
    flow.append(Paragraph("Driver Feedback", styles["h2"]))
    flow.extend(_feedback_flowables(outing, styles))
    return flow


def _cover_page_flowables(weekend, outings, styles):
    flow = []
    header_bits = [weekend.track, f"{weekend.series} {weekend.year}"]
    flow.append(Paragraph(escape(" — ".join(b for b in header_bits if b)), styles["title"]))
    sub_bits = [f"Car #{weekend.car_number}"]
    if weekend.type:
        sub_bits.append(weekend.type)
    if weekend.date:
        sub_bits.append(weekend.date.strftime("%d.%m.%Y"))
    flow.append(Paragraph(escape(" · ".join(sub_bits)), styles["subtitle"]))
    flow.append(Spacer(1, 4 * mm))

    driver_names = []
    seen = set()
    for o in outings:
        if o.driver_id and o.driver_id not in seen:
            seen.add(o.driver_id)
            name, _level = _driver_name_and_level(o.driver_id)
            if name:
                driver_names.append(name)
    if driver_names:
        flow.append(Paragraph("Drivers: " + escape(", ".join(driver_names)), styles["body"]))
        flow.append(Spacer(1, 4 * mm))

    rows = [["#", "Name", "Date", "Driver", "Session", "Status"]]
    for o in outings:
        name, _level = _driver_name_and_level(o.driver_id)
        rows.append([
            str(o.number or "—"), o.name or "(unnamed)",
            o.date_time.strftime("%d.%m.%Y %H:%M") if o.date_time else "—",
            name or "—", o.session_type or "—",
            _STATUS_LABELS[analysis_status(o)],
        ])
    w = CONTENT_W
    flow.append(_table(rows, [w * 0.08, w * 0.22, w * 0.18, w * 0.18, w * 0.14, w * 0.20], styles))
    return flow


def generate_weekend_pdf(weekend, outings, output_path):
    """Build the multi-outing weekend PDF at output_path.

    `outings` is the user's selected subset (Outing ORM rows, any order --
    re-sorted here by date_time). Each outing's section is built inside its
    own try/except: one outing's malformed data (bad JSON, a corrupted
    summaries payload) renders as an inline error note for that outing only
    and the rest of the document still builds -- a single bad outing must
    never abort the whole export. An empty or all-stale/absent selection
    still produces a valid PDF (cover page + per-outing "not analysed"
    sections), since nothing here assumes at least one outing has a
    current analysis.
    """
    styles = _styles()
    ordered = sorted(outings, key=lambda o: o.date_time or o.id)

    doc = SimpleDocTemplate(
        output_path, pagesize=A4,
        leftMargin=MARGIN, rightMargin=MARGIN, topMargin=MARGIN, bottomMargin=MARGIN,
    )
    story = []
    story.extend(_cover_page_flowables(weekend, ordered, styles))
    story.append(PageBreak())

    for outing in ordered:
        try:
            story.extend(_build_outing_section(outing, styles))
        except Exception as e:
            label = f"Outing {outing.number or outing.id}"
            story.append(Paragraph(escape(f"{label}: ERROR building this section -- {e!r}"),
                                    styles["error"]))
        story.append(PageBreak())

    if story and isinstance(story[-1], PageBreak):
        story.pop()

    doc.build(story)
