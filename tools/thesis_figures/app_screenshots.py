# Appendix screenshots of the app: the driver-feedback input and the setup
# recommendation output, taken from the real OutingForm widgets with one
# session's stored results. The form is opened without an outing, so nothing
# is loaded from or saved to the outing row; the stored payload passes the
# app's cache guards (common.stored_payload) before it is shown.
# Setup values never reach the image: from -> to values and step sizes
# (author ruling 2026-10-30: step sizes count as setup values) are replaced
# by "[hidden]" in the rendered text. The feedback table shows the stored
# values; none are entered here (real-data rule).
# Native Qt platform on purpose: the offscreen platform renders without the
# system fonts. The window is placed off the visible desktop.
# --blank: the feedback input of a new, empty form instead (no outing, no
# session; author ruling 2026-10-30): BLANK_CORNERS rows at their default,
# speed class "-", cropped to the corner table and the scale legend, at 2x.
# It overwrites feedback_input.png. The app has a dark theme only.
# Run: python -m tools.thesis_figures.app_screenshots --outing 26 [--out DIR]
#      python -m tools.thesis_figures.app_screenshots --blank [--out DIR]

import argparse
import copy
import json
import os
import re

from tools.thesis_figures import common

FEEDBACK_PNG = "feedback_input.png"
RECOMMENDATION_PNG = "recommendation_output.png"
HIDDEN = "[hidden]"
BLANK_CORNERS = 6
BLANK_SCALE = 2   # device pixel ratio of the blank-form shot
GAP_PX = 12       # logical px between table and scale legend; layout only
# "(1 step)", "(2 steps)" and signed step changes such as "TC lon +1"
_STEP_COUNT = re.compile(r"\(\d+(?:\.\d+)? steps?\)|\(\d+(?:\.\d+)?\)")
_SIGNED_STEP = re.compile(r"(?<=\s)[+-]\d+(?:\.\d+)?(?=\s|$|\|)")


def hide_setup_values(node):
    # from -> to of every change, in a display copy of the stored layers
    if isinstance(node, dict):
        ch = node.get("change")
        if isinstance(ch, dict):
            for k in ch.get("keys") or []:
                for f in ("from", "to"):
                    if k.get(f) is not None:
                        k[f] = HIDDEN
        for v in node.values():
            hide_setup_values(v)
    elif isinstance(node, list):
        for v in node:
            hide_setup_values(v)


def hide_steps(text):
    return _SIGNED_STEP.sub(HIDDEN, _STEP_COUNT.sub(f"({HIDDEN})", text))


def shoot(outing, out=None):
    # common sets offscreen for the figure scripts; this one needs the fonts
    os.environ["QT_QPA_PLATFORM"] = "windows"
    from PyQt6.QtWidgets import QApplication, QLabel
    app = QApplication.instance() or QApplication([])
    from ui.style import STYLESHEET
    app.setStyleSheet(STYLESHEET)
    import models.driver, models.outing, models.raceweekend  # noqa: F401,E401  (mapper registry)
    from models.base import Session
    from models.raceweekend import RaceWeekend
    from ui.views.outing_form import OutingForm

    session = common.load_session(outing)
    payload, why = common.stored_payload(session)
    if payload is None:
        raise common.Refused(f"outing {outing}: {why}")
    _data, plots_dir, _prefix = common.out_dirs(out)
    shots_dir = os.path.join(os.path.dirname(os.path.normpath(plots_dir)), "screenshots")
    os.makedirs(shots_dir, exist_ok=True)

    def pump():
        for _ in range(20):
            app.processEvents()

    db = Session()
    try:
        weekend = db.get(RaceWeekend, int(session["row"]["race_weekend_id"]))
        form = OutingForm(weekend, lambda: None, outing=None)
        form.resize(1200, 900)
        form.move(-4000, -4000)
        form.show()

        # one feedback row per detected corner, no track map (an image the
        # team supplied may carry branding)
        fb = json.loads(session["row"]["feedback_data"] or "{}")
        fb.update(corner_count=len({s["stable_corner_id"] for s in payload["summaries"]}), map_path="")
        form.stability_result = {"summaries": payload["summaries"], "corners": session["parsed"]["corners"]}
        form._load_feedback_data(json.dumps(fb))
        form._refresh_feedback_speed_labels()
        pump()
        form._section_feedback.grab().save(os.path.join(shots_dir, FEEDBACK_PNG))

        layers = copy.deepcopy(payload["setup_engine"])
        hide_setup_values(layers)
        form.setup_panel.setVisible(True)
        form._render_setup_layers(layers)
        for lab in form.setup_panel.findChildren(QLabel):
            lab.setText(hide_steps(lab.text()))
        pump()
        form.setup_panel.grab().save(os.path.join(shots_dir, RECOMMENDATION_PNG))
        form.close()
    finally:
        db.close()
    entered = any(v for r in fb.get("corners", []) for v in r.values())
    return shots_dir, entered


def shoot_blank(out=None):
    # a new form as the app opens it for a new outing; the weekend is a
    # stand-in (the form reads only its track name for the title), so no
    # weekend or outing row is touched
    import types
    os.environ["QT_QPA_PLATFORM"] = "windows"
    os.environ["QT_SCALE_FACTOR"] = str(BLANK_SCALE)
    from PyQt6.QtCore import QPoint, QRect
    from PyQt6.QtGui import QColor, QPainter, QPixmap
    from PyQt6.QtWidgets import QApplication, QLabel
    app = QApplication.instance() or QApplication([])
    from ui.style import BACKGROUND, STYLESHEET
    app.setStyleSheet(STYLESHEET)
    import models.driver, models.outing, models.raceweekend  # noqa: F401,E401  (mapper registry, driver list)
    from ui.views.outing_form import OutingForm

    _data, plots_dir, _prefix = common.out_dirs(out)
    shots_dir = os.path.join(os.path.dirname(os.path.normpath(plots_dir)), "screenshots")
    os.makedirs(shots_dir, exist_ok=True)
    form = OutingForm(types.SimpleNamespace(track="", id=None), lambda: None, outing=None)
    form.resize(1200, 900)
    form.move(-4000, -4000)
    form.show()
    form.corner_count_spin.setValue(BLANK_CORNERS)
    for _ in range(20):
        app.processEvents()

    table = form.feedback_table.grab()
    scale_label = next(lab for lab in form._section_feedback.findChildren(QLabel) if lab.text().startswith("Scale:"))
    # the legend label spans the section; crop it to its text
    hint = scale_label.sizeHint()
    scale = scale_label.grab(QRect(0, 0, min(hint.width(), scale_label.width()), scale_label.height()))
    # the grabs carry the device pixel ratio; the canvas takes the same one
    # and is laid out in logical px, so nothing is rescaled
    dpr = table.devicePixelRatio()
    tw, th = table.width() // dpr, table.height() // dpr
    sh = scale.height() // dpr
    w, h = int(tw * dpr), int((th + GAP_PX + sh) * dpr)
    canvas = QPixmap(w, h)
    canvas.setDevicePixelRatio(dpr)
    canvas.fill(QColor(BACKGROUND))
    p = QPainter(canvas)
    p.drawPixmap(QPoint(0, 0), table)
    p.drawPixmap(QPoint(0, int(th + GAP_PX)), scale)
    p.end()
    path = os.path.join(shots_dir, FEEDBACK_PNG)
    canvas.save(path)
    form.close()
    return path, (w, h), dpr


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--blank", action="store_true", help="blank feedback form, no outing")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    if a.blank:
        path, (w, h), dpr = shoot_blank(a.out)
        print(f"{path}: {w} x {h} px at {dpr:g}x")
        return
    if a.outing is None:
        ap.error("--outing is required unless --blank")
    shots_dir, entered = shoot(a.outing, a.out)
    print(f"{FEEDBACK_PNG}, {RECOMMENDATION_PNG} -> {shots_dir}; feedback entered: {entered}")


if __name__ == "__main__":
    main()
