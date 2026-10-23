# CS phase colours follow the classifier's own rule (author ruling
# 2026-10-20: colour and verdict cannot disagree). Since ruling D2
# (2026-10-22): an axle is limiting below -cs_boundary_margin_front / _rear (per axle, 2026-10-22); front limiting
# -> WARN, in the strong tier -> BAD; rear limiting -> BAD; rear above the
# boundary but below its moderate tier -> WARN (information only); else OK.
# Real data: the corner summaries of the NBR JST outing 29 default-path
# golden (tests/golden/default_nbr_jst29.json).

import json
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt6.QtWidgets")

from modules.stability_analysis import load_parameters  # noqa: E402

GOLDEN = "tests/golden/default_nbr_jst29.json"
OLD_REAR_WARN_LITERAL = 0.20   # the retired literal: a rear value below it was BAD


@pytest.fixture(scope="module")
def phase_values():
    with open(GOLDEN, encoding="utf-8") as f:
        summaries = json.load(f)["summaries"]
    vals = {"f": [], "r": []}
    for s in summaries:
        for p in s["phases"].values():
            for ax in "fr":
                v = p[f"cs_ratio_{ax}"]["median"]
                if v is not None and v == v:
                    vals[ax].append(v)
    return vals


def test_colour_is_the_d2_class(phase_values):
    from ui.style import BAD, OK, WARN
    from ui.views.outing_form import OutingForm
    cl = load_parameters()["classification"]
    mf, mr = cl["cs_boundary_margin_front"]["value"], cl["cs_boundary_margin_rear"]["value"]
    for v in phase_values["f"]:
        expected = (BAD if v < cl["STRONG_CSF"]["value"] else WARN) if v < -mf else OK
        assert OutingForm._stability_colour(None, "cs", v, axle="f") == expected
    for v in phase_values["r"]:
        expected = BAD if v < -mr else (WARN if v < cl["MODERATE_CSR"]["value"] else OK)
        assert OutingForm._stability_colour(None, "cs", v, axle="r") == expected


def test_rear_value_between_moderate_and_old_literal_is_not_bad(phase_values):
    from ui.style import BAD, OK
    from ui.views.outing_form import OutingForm
    moderate = load_parameters()["classification"]["MODERATE_CSR"]["value"]
    between = [v for v in phase_values["r"] if moderate <= v < OLD_REAR_WARN_LITERAL]
    assert between, "no real rear phase value between MODERATE_CSR and the old literal in this golden"
    for v in between:
        assert OutingForm._stability_colour(None, "cs", v, axle="r") == OK != BAD
