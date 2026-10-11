# Product text names no sources and no models (author app-check ruling
# 2026-10-06: "the product describes what, not after whom"). Anchors live in
# code comments, docstrings, config provenance, PRODUCTION.md and the thesis.

import ast
import os
import re

NAMES = re.compile(r"Werner|Segers|Rajamani|Milliken|Pacejka|Dugoff|Hoffman|Kiencke|Bechtloff|Ulsoy")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _string_literals(path):
    tree = ast.parse(open(path, encoding="utf-8").read())
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node, clean=False)
            if doc is not None:
                docstrings.add(doc)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and node.value not in docstrings:
            yield node.lineno, node.value


def test_no_source_or_model_names_in_ui_strings():
    hits = []
    for dp, _dns, fns in os.walk(os.path.join(ROOT, "ui")):
        for fn in fns:
            if fn.endswith(".py"):
                path = os.path.join(dp, fn)
                hits += [f"{os.path.relpath(path, ROOT)}:{ln}: {s[:80]!r}"
                         for ln, s in _string_literals(path) if NAMES.search(s)]
    assert not hits, "source / model names in UI strings:\n" + "\n".join(hits)


def test_unstable_verdict_names_no_source():
    from ui.views.outing_form import OutingForm
    phase = {"cs_ratio_f": {"median": 1.0}, "cs_ratio_r": {"median": 1.0},
             "stability_observed_Nm_per_deg": {"median": 500.0}}
    phases = {p: dict(phase) for p in ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")}
    phases["exit_4"] = dict(phase, stability_observed_Nm_per_deg={"median": -40.0})
    _sev, short, long_, _c = OutingForm._classify_corner(None, {"phases": phases})
    assert "open-loop unstable" in short          # the parsers' token is kept
    assert long_ == "Open-loop unstable, early exit, -40 Nm/deg"
    assert not NAMES.search(short + long_)


def test_estimator_status_names_no_model():
    from ui.views.outing_form import OutingForm
    for src in ("kinematic", "ekf_pass_1", "ekf_auto_dugoff", "ekf_auto_pacejka"):
        text, _c = OutingForm._format_estimator_status(None, src, {"status": "ok"},
                                                        {"verdict": "pass", "health_score": 0.1}, False, None)
        assert not NAMES.search(text), text
