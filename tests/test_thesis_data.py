# Thesis data reader: a sidecar written exactly as the app writes it (real
# Dubai pipeline result) reads back to the same CS arrays, the one-concept
# kerb-affected mark and the parser's lap ids. No Qt in the reader.

import os
import re

import numpy as np

from modules import pipeline_sidecar
from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION


def test_sidecar_round_trip(tmp_path, monkeypatch, pipeline_result, parsed_data, state, raw_params):
    from core.thesis_data import read_sidecar
    from modules.accuracy_resolution import resolve_estimator_settings
    monkeypatch.setattr(pipeline_sidecar, "SIDECAR_DIR", str(tmp_path))
    analysis_laps = [l["lap_number"] for l in parsed_data["laps"] if l.get("is_valid_for_analysis")]
    # a round trip carries the complete identity (R24 refusal of incomplete
    # ones: test_stale_sidecar_is_refused)
    identity = pipeline_sidecar.build_identity(ANALYSIS_SCHEMA_VERSION, "dubai.txt", 1, {}, "ekf_auto_pacejka",
                                               state["sample_rate_hz"], analysis_laps,
                                               estimator_settings=resolve_estimator_settings(raw_params))
    payload = {"state": state, "cs": pipeline_result["cs"], "corners": pipeline_result["corners"]}
    assert pipeline_sidecar.write_sidecar(99, identity, payload)

    got = read_sidecar(os.path.join(str(tmp_path), "outing_99.pkl.gz"), laps=parsed_data["laps"])
    cs = pipeline_result["cs"]
    assert got["analysis_laps"] == sorted(analysis_laps)
    assert np.array_equal(got["cs_f"], cs["CS_ratio_f"], equal_nan=True)
    assert np.array_equal(got["cs_r"], cs["CS_ratio_r"], equal_nan=True)
    assert np.array_equal(got["distance_m"], state["s_m"], equal_nan=True)
    for ax in "fr":
        assert np.array_equal(got[f"kerb_affected_{ax}"], state["kerb_mask"] | cs[f"near_strike_{ax}"])
    assert got["kerb_affected_f"].any()
    t = state["time"]
    for lap in parsed_data["laps"]:
        inside = (t >= lap["start_time"]) & (t <= lap["end_time"])
        assert np.all(got["lap"][inside] == lap["lap_number"])


def test_reader_has_no_ui_import():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, "core", "thesis_data.py"), encoding="utf-8") as fh:
        src = fh.read()
    assert not re.search(r"^\s*(from|import)\s+(ui|PyQt6)\b", src, re.M)


def test_stale_sidecar_is_refused(tmp_path, monkeypatch, pipeline_result, parsed_data, state, raw_params):
    """R24: the reader refuses a sidecar from another schema or analysis
    identity with a clear error instead of reading it silently."""
    import pytest
    from core.thesis_data import StaleSidecarError, read_sidecar
    from modules.accuracy_resolution import resolve_estimator_settings
    monkeypatch.setattr(pipeline_sidecar, "SIDECAR_DIR", str(tmp_path))
    laps = [l["lap_number"] for l in parsed_data["laps"] if l.get("is_valid_for_analysis")]
    settings = resolve_estimator_settings(raw_params)
    payload = {"state": state, "cs": pipeline_result["cs"], "corners": pipeline_result["corners"]}

    def write(oid, schema=ANALYSIS_SCHEMA_VERSION, est=settings, source="ekf_auto_pacejka"):
        ident = pipeline_sidecar.build_identity(schema, "dubai.txt", 1, {}, source, state["sample_rate_hz"], laps,
                                                estimator_settings=est)
        assert pipeline_sidecar.write_sidecar(oid, ident, payload)
        return os.path.join(str(tmp_path), f"outing_{oid}.pkl.gz"), ident

    good, ident = write(1)
    assert read_sidecar(good, laps=parsed_data["laps"], expected_identity=ident)["analysis_laps"] == sorted(laps)
    for oid, kw, field in ((2, {"schema": ANALYSIS_SCHEMA_VERSION - 1}, "schema_version"),
                           (3, {"est": None}, "estimator_settings"),
                           (4, {"est": dict(settings, kerb_baseline_g=settings["kerb_baseline_g"] + 0.1)},
                            "estimator_settings"),
                           (5, {"source": "kinematic"}, "sideslip_source")):
        path, _ = write(oid, **kw)
        with pytest.raises(StaleSidecarError, match=field):
            read_sidecar(path, laps=parsed_data["laps"])
    # outing-dependent fields only against an expected identity
    with pytest.raises(StaleSidecarError, match="accuracy_cap"):
        read_sidecar(good, laps=parsed_data["laps"], expected_identity=dict(ident, accuracy_cap=2))
