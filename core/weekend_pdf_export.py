# Weekend PDF for a selected set of outings: cover summary, Setup/Setdown
# strips (build_session_strip "small", one outing's pair per page), then
# per outing: accuracy footer, verdicts, recommendations, driver feedback.
# Labels and _fmt shared with core/pdf_export.py.
#
# Verdicts only for outings whose cached schema_version matches the current
# ANALYSIS_SCHEMA_VERSION, and then classified live from current config --
# never read from the cache (same as the app). Stale or missing ->
# "Not analysed under current version -- re-run Analyse."

import json
from xml.sax.saxutils import escape

from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer, PageBreak
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER

from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
from modules.recommendation import (
    aggregate_by_corner, generate_recommendations, load_recommendations_config,
)
from models.driver import driver_name_and_level

PAGE_W, PAGE_H = landscape(A4)
MARGIN = 14 * mm
CONTENT_W = PAGE_W - 2 * MARGIN
# two strips per page -- at four, the wheel-grid boxes needed ~77 mm of a
# 44 mm strip
STRIPS_PER_PAGE = 2
STRIP_GAP = 2 * mm
STRIP_H = (PAGE_H - 2 * MARGIN - (STRIPS_PER_PAGE - 1) * STRIP_GAP) / STRIPS_PER_PAGE

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
        value = "-"
    return Paragraph(escape(str(value)), style)


def _table(rows, col_widths, styles, header=True):
    # bold header row, plain grid -- tables only
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
    # the app's own classifier, called with self=None (doesn't use self) ->
    # PDF can't disagree with the app
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def _sideslip_source_calibrated():
    # same two-key comparison as OutingForm._sideslip_source_calibrated,
    # duplicated to keep a QWidget method out of PDF code
    from modules.stability_analysis import load_parameters
    params = load_parameters()
    active = params["stability_estimation"].get("sideslip_source", "kinematic")
    calibrated_for = params["classification"].get(
        "thresholds_calibrated_for_sideslip_source", "kinematic"
    )
    return active == calibrated_for


def _accuracy_footer_text(levels):
    from ui.views.outing_form import OutingForm
    if not levels:
        return ""
    parts = [f"{label} L{levels[node]}" for node, label in OutingForm._ACCURACY_FOOTER_LABELS
             if node in levels]
    return " | ".join(parts)


def _load_json(raw):
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return {}


def analysis_status(outing):
    """'current' | 'stale' | 'absent' -- the only gate for printing verdicts."""
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


def _session_meta(outing, driver_name, sheet_label, weekend):
    return {
        "number": outing.number or outing.id,
        "name": outing.name or "",
        "session_type": outing.session_type or "",
        "date_str": outing.date_time.strftime("%d.%m.%Y %H:%M") if outing.date_time else "",
        "driver_name": driver_name or "",
        "sheet_label": f"{weekend.track} - {sheet_label}",
    }


def _build_setup_sheets_section(weekend, outings, styles):
    """Setup/Setdown strips, "small" scale, one outing's pair per landscape page,
    chronological. Page break before each outing after the first -> Setdown
    always under its Setup.
    """
    from core.pdf_export import build_session_strip

    flow = []
    for i, outing in enumerate(outings):
        if i > 0:
            flow.append(PageBreak())
        driver_name, _level = driver_name_and_level(outing.driver_id)
        setup = _load_json(outing.setup_data)
        setdown = _load_json(outing.setdown_data)
        strip_w = CONTENT_W
        flow.append(build_session_strip(
            _session_meta(outing, driver_name, "SETUP", weekend), setup, "small", strip_w, STRIP_H))
        flow.append(Spacer(1, STRIP_GAP))
        flow.append(build_session_strip(
            _session_meta(outing, driver_name, "SETDOWN", weekend), setdown, "small", strip_w, STRIP_H))
    return flow


def _estimator_status_text(sideslip_source, fit_manifest, gate_verdict, fallback_used, fallback_reason):
    # text from OutingForm's formatter (self=None) so wording matches the app;
    # colour chosen here from fallback_used
    if sideslip_source is None:
        return None
    from ui.views.outing_form import OutingForm
    text, _colour = OutingForm._format_estimator_status(
        None, sideslip_source, fit_manifest, gate_verdict, fallback_used, fallback_reason
    )
    return text


def _verdict_flowables(summaries, styles, sideslip_source=None, fit_manifest=None,
                        gate_verdict=None, fallback_used=False, fallback_reason=None):
    aggregated = aggregate_by_corner(summaries)
    rows = [["Corner", "Speed class", "Severity", "Verdict"]]
    for cid in sorted(aggregated.keys()):
        agg = aggregated[cid]
        severity, _short, long_v, _colour = _classify_corner(agg)
        rows.append([f"C{cid}", agg.get("speed_class") or "-", severity, long_v])
    w = CONTENT_W
    flow = []
    # estimator/fit/gate/fallback line, above the calibration banner: "what
    # produced this" before "are the thresholds valid for it"
    status_text = _estimator_status_text(
        sideslip_source, fit_manifest, gate_verdict, fallback_used, fallback_reason
    )
    if status_text:
        flow.append(Paragraph(status_text, styles["warn"] if fallback_used else styles["muted"]))
        flow.append(Spacer(1, 1.5 * mm))
    # page-level caveat to the per-cell "[UNCAL]" markers; placeholder wording
    if not _sideslip_source_calibrated():
        flow.append(Paragraph(
            "PLACEHOLDER: sideslip estimator changed, verdict thresholds not "
            "re-derived -- read traces, not verdict colours.",
            styles["warn"],
        ))
        flow.append(Spacer(1, 1.5 * mm))
    flow.append(_table(rows, [w * 0.12, w * 0.16, w * 0.16, w * 0.56], styles))
    return flow


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

    # situational/provenance are per rule -> looked up via rules_fired, as in the UI
    rule_situational = {r["id"]: bool(r.get("situational")) for r in config["rules"]}
    rule_provenance = {r["id"]: r.get("elicitation_provenance") for r in config["rules"]}

    rows = [["Action", "Score", "Trigger", "Cell ID(s)", "Provenance", "Situational",
             "Class", "Selected", "Conflicts / limits"]]
    for r in results:
        # urgent_gap rows have no action or score -> corner + verdict, like the
        # UI badge
        if r["action_class"] == "urgent_gap":
            c0 = r["corners"][0] if r["corners"] else None
            action_text = f"C{c0['stable_corner_id']}: {c0['short_verdict']}" if c0 else "engineer attention"
            score_text = "-"
        else:
            if r["parameter"] is not None:
                action_text = f"{r['parameter']} {r['direction']}"
            else:
                action_text = " + ".join(
                    f"{a['parameter']} -> {a['target']}" if "target" in a
                    else f"{a['parameter']} {a['direction']}"
                    for a in r["actions"]
                )
            score_text = f"{r['score']:.2f}"
        provenances = sorted({(rule_provenance.get(rid) or "-") for rid in r["rules_fired"]})
        situational = any(rule_situational.get(rid) for rid in r["rules_fired"])
        if r["action_class"] == "urgent_gap":
            action_class_text = "URGENT"
        elif r["action_class"] == "advisory":
            action_class_text = "ADVISORY"
        else:
            action_class_text = "RECOMMENDED"

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
            action_text, score_text, "/".join(r["trigger_source"]),
            "/".join(r["cell_ids"]) or "-", "/".join(provenances),
            "yes" if situational else "no",
            ("SELECTED" if r["selected"] else action_class_text),
            "yes" if r["selected"] else "no",
            "; ".join(notes) or "-",
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
    driver_name, driving_level = driver_name_and_level(outing.driver_id)
    title = f"Outing {outing.number or outing.id}" + (f" -- {outing.name}" if outing.name else "")
    flow.append(Paragraph(escape(title), styles["h1"]))
    flow.append(Paragraph(escape(_outing_meta_line(outing, driver_name)), styles["muted"]))
    flow.append(Spacer(1, 2 * mm))

    status = analysis_status(outing)
    flow.append(Paragraph("Analysis", styles["h2"]))
    if status != "current":
        flow.append(Paragraph("Not analysed under current version -- re-run Analyse.",
                               styles["warn"]))
    else:
        parsed = _load_json(outing.analysis_data)
        summaries = parsed.get("summaries", [])
        footer = _accuracy_footer_text(parsed.get("resolved_levels"))
        if footer:
            flow.append(Paragraph("Resolved accuracy: " + escape(footer), styles["muted"]))
        flow.append(Spacer(1, 2 * mm))

        flow.append(Paragraph("Verdict Summary", styles["h2"]))
        flow.extend(_verdict_flowables(
            summaries, styles,
            sideslip_source=parsed.get("sideslip_source"),
            fit_manifest=parsed.get("fit_manifest"),
            gate_verdict=parsed.get("gate_verdict"),
            fallback_used=parsed.get("fallback_used", False),
            fallback_reason=parsed.get("fallback_reason"),
        ))
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
    flow.append(Paragraph(escape(" -- ".join(b for b in header_bits if b)), styles["title"]))
    sub_bits = [f"Car #{weekend.car_number}"]
    if weekend.type:
        sub_bits.append(weekend.type)
    if weekend.date:
        sub_bits.append(weekend.date.strftime("%d.%m.%Y"))
    flow.append(Paragraph(escape(" | ".join(sub_bits)), styles["subtitle"]))
    flow.append(Spacer(1, 4 * mm))

    driver_names = []
    seen = set()
    for o in outings:
        if o.driver_id and o.driver_id not in seen:
            seen.add(o.driver_id)
            name, _level = driver_name_and_level(o.driver_id)
            if name:
                driver_names.append(name)
    if driver_names:
        flow.append(Paragraph("Drivers: " + escape(", ".join(driver_names)), styles["body"]))
        flow.append(Spacer(1, 4 * mm))

    rows = [["#", "Name", "Date", "Driver", "Session", "Status"]]
    for o in outings:
        name, _level = driver_name_and_level(o.driver_id)
        rows.append([
            str(o.number or "-"), o.name or "(unnamed)",
            o.date_time.strftime("%d.%m.%Y %H:%M") if o.date_time else "-",
            name or "-", o.session_type or "-",
            _STATUS_LABELS[analysis_status(o)],
        ])
    w = CONTENT_W
    flow.append(_table(rows, [w * 0.08, w * 0.22, w * 0.18, w * 0.18, w * 0.14, w * 0.20], styles))
    return flow


def generate_weekend_pdf(weekend, outings, output_path):
    """Weekend PDF at output_path. outings = selected Outing rows, sorted here
    by date_time. Cover, setup strips, one page per outing.
    Each outing's analysis section in its own try/except -> one bad outing
    becomes an inline note, never aborts the export. Works with no current
    analysis at all.
    """
    styles = _styles()
    ordered = sorted(outings, key=lambda o: o.date_time or o.id)

    doc = SimpleDocTemplate(
        output_path, pagesize=(PAGE_W, PAGE_H),
        leftMargin=MARGIN, rightMargin=MARGIN, topMargin=MARGIN, bottomMargin=MARGIN,
    )
    story = []
    story.extend(_cover_page_flowables(weekend, ordered, styles))
    story.append(PageBreak())

    if ordered:
        story.extend(_build_setup_sheets_section(weekend, ordered, styles))
        story.append(PageBreak())

    for outing in ordered:
        try:
            story.extend(_build_outing_section(outing, styles))
        except Exception as e:
            # readable message, not a raw repr()
            from core.error_text import friendly_error_text
            label = f"Outing {outing.number or outing.id}"
            story.append(Paragraph(escape(f"{label}: ERROR building this section -- {friendly_error_text(e)}"),
                                    styles["error"]))
        story.append(PageBreak())

    if story and isinstance(story[-1], PageBreak):
        story.pop()

    doc.build(story)
