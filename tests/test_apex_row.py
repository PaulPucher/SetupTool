# Per-phase table: the Apex row shows the apex-region CS statistic the
# verdict reads, never the fixed apex slice (author ruling 2026-10-07).
# Display layer only; real Dubai summary, values planted so the two slices
# differ.

import copy

import pytest


def test_apex_row_shows_the_value_the_verdict_reads(pipeline_result):
    pytest.importorskip("PyQt6.QtWidgets")
    from ui.views.outing_form import OutingForm
    summary = copy.deepcopy(next(s for s in pipeline_result["summaries"] if s.get("apex_region")))
    # every phase clean except the apex REGION (front -1.00, strong); the fixed
    # apex slice carries a different value that must not reach the display
    for p in summary["phases"].values():
        for ax in ("cs_ratio_f", "cs_ratio_r"):
            p[ax] = {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 20, "n_kerb_excluded": 0}
        p["stability_observed_Nm_per_deg"] = {"median": 500.0, "p25": 500.0, "p75": 500.0, "n": 20}
    summary["phases"]["apex_3"]["cs_ratio_f"]["median"] = 0.93
    summary["apex_region"]["cs_ratio_f"] = {"median": -1.0, "p25": -1.1, "p75": -0.9, "n": 30, "n_kerb_excluded": 0}
    summary["apex_region"]["cs_ratio_r"] = {"median": 1.0, "p25": 1.0, "p75": 1.0, "n": 30, "n_kerb_excluded": 0}

    _sev, _short, long_text, _c = OutingForm._classify_corner(None, summary)
    assert "front CS -1.00" in long_text
    rows = OutingForm._phase_table_rows(summary, 25.0, 5)
    region = [r for r in rows if r["label"] == "Apex region (+/-25 m)"]
    assert len(region) == 1 and region[0]["csf"].startswith("-1.00")
    assert region[0]["csf_median"] == OutingForm._verdict_cs_stat(summary, "apex_3", "f")["median"]
    # the fixed-slice CS is shown nowhere
    assert not any("0.93" in r["csf"] for r in rows)
    assert [r["label"] for r in rows] == ["Brake", "Turn-in", "Apex region (+/-25 m)",
                                         "Apex (+/-5 samples)", "Exit 4", "Exit 5"]
