# Smoke test of the thesis figure export (tools/thesis_figures/, 2026-10-24):
# the helper's rules and each script's command line, without a chain run.
# The binning / thinning checks use small hand-made arrays (pure-function
# tests, same category as tests/test_wheel_loads.py); the stale-payload check
# reads the real app DB read-only.

import numpy as np
import pytest

from tools.thesis_figures import common


def test_phase_labels_cover_the_app_phases_once():
    from modules.classifier import PHASE_KEYS
    assert list(common.PHASE_LABELS) == PHASE_KEYS
    assert common.PHASE_LABELS["exit_5"] == "exit completion"


def test_lines_come_from_config_and_the_gate():
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    lin = common.lines()
    assert lin["boundary_f"] == -cl["cs_boundary_margin_front"]["value"]
    assert lin["boundary_r"] == -cl["cs_boundary_margin_rear"]["value"]
    assert lin["strong_f"] == cl["STRONG_CSF"]["value"]
    # the rear information tier is drawn since ruling B10 (2026-10-27): a
    # note, no symptom, same line set as the app's trace views
    assert lin["info_r"] == cl["MODERATE_CSR"]["value"]
    assert lin["mu_edge"] == pytest.approx(lin["mu_ceiling"] * (1 + load_parameters()["tyre_fit_auto"]["mu_ceiling_edge_u"]))


def test_bin_with_an_excluded_sample_is_nan():
    s = np.arange(0.0, 10.0, 0.5)
    x = np.ones_like(s)
    excl = np.zeros(s.shape, bool)
    excl[5] = True                       # s = 2.5 m -> bin [2, 4)
    centres, out = common.bin_trace(s, x, excl, 0.0, 9.5)
    assert np.isnan(out[1]) and np.all(out[[0, 2, 3]] == 1.0)


def test_thinning_caps_and_keeps_the_sparse_tail():
    rng = np.random.default_rng(1)
    a = np.concatenate([rng.normal(0, 0.01, 20000), np.linspace(0.1, 0.15, 30)])
    keep = common.thin_stratified(a, a, cap=3000)
    assert keep.size <= 3000
    assert set(range(20000, 20030)) <= set(keep.tolist())


def test_stale_payload_is_refused_with_its_reason():
    row = common.db_outing(12)
    # schema is the first guard; the other identity fields only need to exist
    ident = {"cap": None, "resolved": {"values": None, "estimator_settings": None}, "sideslip_source": None,
             "grid_rate_hz": None, "lap_filter": []}
    session = {"row": row, "path": row["csv_path"], "ident": ident}
    payload, why = common.stored_payload(session)
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
    import json
    if json.loads(row["analysis_data"]).get("schema_version") == ANALYSIS_SCHEMA_VERSION:
        pytest.skip("outing 12 already re-analysed at the current schema")
    assert payload is None and "stale stored analysis (schema_version" in why


@pytest.mark.parametrize("mod", ["fig_example_fit", "fig_example_corner", "fig_k_sessions", "fig_cs_distribution",
                                 "find_example_corner", "tab_sessions", "tab_example_verdicts", "fig_example_lap",
                                 "tab_observer_health", "fig_verdict_repeatability", "fig_balance_speed_class",
                                 "fig_session_overview", "fig_symptom_phase", "app_screenshots"])
def test_command_line_builds(mod):
    import importlib
    m = importlib.import_module(f"tools.thesis_figures.{mod}")
    with pytest.raises(SystemExit) as e:
        m.main(["--help"])
    assert e.value.code == 0


def test_corner_figure_has_four_panels_and_one_legend_below():
    # layout order 2026-10-27: a_y, beta, alpha, CS (no steer panel), CS
    # symbol from the helper, one named legend placed below the last panel,
    # phase marks abbreviated, fit-sample range per axle on the alpha panel
    # as thin lines without a fill (revision 2026-10-29)
    from tools.thesis_figures import fig_example_corner as c
    n = 5
    cols = {k: np.linspace(0.1, 1.0, n) for k in ("s_m", "delta_f_deg", "ay_filt_g", "beta_obs_deg", "beta_kin_deg",
                                                  "alpha_f_deg", "alpha_r_deg", "cs_f", "cs_r")}
    phases = {"phase_key": ["exit_4"], "phase_label": ["early exit"], "s_start_m": [0.0], "s_end_m": [10.0]}
    fit_range = {"f": (-6.0, 6.5), "r": (-4.5, 4.8)}
    tex = c._tex("b", "figures/data/", cols, phases, common.lines(), 0.0, 10.0, [], fit_range)
    assert tex.count("\\nextgroupplot") == 4 and "group size=1 by 4" in tex
    assert "y=ay_filt_g" in tex and "y=delta_f_deg" not in tex and f"ylabel={{{common.CS_AXIS}}}" in tex
    assert tex.count("legend style") == 1 and "\\pgfplotslegendfromname{legExampleCorner}" in tex
    assert "{E1}" in tex and "early exit}" not in tex and "clip=false" in tex
    # two limit lines per axle plus one legend image, no band fill
    assert tex.count("fit-sample range") == 1 and "okBand" not in tex
    assert sum(tex.count(f"\\addplot[{c.BAND_EDGE[a]}, forget plot]") for a in ("f", "r")) == 4
    # CS lower limit -0.2 unless the corner's CS goes lower
    assert "ymin=-0.2, ymax=1.05" in tex


def test_name_guard_refuses_a_driver_name(tmp_path):
    import sqlite3
    con = sqlite3.connect(f"file:{common.DB}?mode=ro", uri=True)
    name = con.execute("SELECT name FROM drivers LIMIT 1").fetchone()[0]
    con.close()
    surname = name.split()[-1]
    with pytest.raises(SystemExit):
        common.write_csv(str(tmp_path / "x.csv"), {"label": [f"run by {surname.upper()}"]}, {})
    assert not (tmp_path / "x.csv").exists()
    common.write_csv(str(tmp_path / "y.csv"), {"label": ["NBR 2"], "cs_f": [0.5]}, {})
    assert (tmp_path / "y.csv").exists()


def test_session_label_from_local_codes():
    row = common.db_outing(12)
    code = common.load_labels().get(row["weekend"])
    if not code:
        pytest.skip("no code for this weekend in local/labels.json")
    assert common.session_label(row) == f"{code} {row['number']}"
    with pytest.raises(SystemExit):
        common.session_label(dict(row, weekend="weekend without a code"))


def test_corner_matching_recovers_the_shifted_ids():
    # real stored payloads: NBR outing 15 detects one extra corner, so its C2
    # is the others' C1 (thesis_notes 'stable_corner_id is per session')
    import json
    ref = common.corner_brackets(json.loads(common.db_outing(11)["analysis_data"])["summaries"])
    oth = common.corner_brackets(json.loads(common.db_outing(15)["analysis_data"])["summaries"])
    m = common.match_corners(ref, oth)
    assert m[2][0] == 1 and m[16][0] == 15 and 1 not in m


def test_verdict_codes():
    import json
    from modules.classifier import aggregate_by_corner
    agg = aggregate_by_corner(json.loads(common.db_outing(12)["analysis_data"])["summaries"])
    codes = {common.verdict(a)[1] for a in agg.values()}
    assert codes <= {"-", "n", "U", "U*", "O", "O*", "I", "UI", "U*I", "OI", "O*I"} and codes


def test_distribution_drops_phases_that_do_not_occur():
    # ruling 2026-10-24: a phase without any sample on any lap is not a cell;
    # read-only on the stored payload of outing 14
    import json
    from modules.classifier import _group_by_corner
    from tools.thesis_figures.fig_cs_distribution import cells
    payload = json.loads(common.db_outing(14)["analysis_data"])
    got, dropped = cells(payload)
    laps = _group_by_corner(payload["summaries"])
    for cid, ph, _ax, v, cause in got:
        assert sum((s["phases"].get(ph) or {}).get("n_samples", 0) for s in laps[cid]) > 0
        assert (cause is None) == bool(np.isfinite(v)) and cause in (None, "b", "c")
    assert dropped >= 0 and len(got) % 2 == 0


def test_verdict_table_prints_no_negative_zero():
    from tools.thesis_figures.tab_example_verdicts import _fmt
    assert _fmt(-0.00249, ".2f") == "0.00" and _fmt(-0.3, ".0f") == "0"
    assert _fmt(-0.006, ".2f") == "-0.01"


def test_appendix_recommendation_line_has_no_setup_values():
    from tools.thesis_figures.tab_example_verdicts import appendix_name, recommendation_line
    o = {"levers": ["arb_rl", "arb_rr"], "direction": "softer", "net": 2.0,
         "change": {"step": 1, "keys": [{"from": 5, "to": 4}]}}
    head, alts = recommendation_line({"recommended": o, "equivalent_options": [], "alternatives": []})
    assert head == "rear anti-roll bar softer (net support 2.00)" and alts == "none"
    tie = recommendation_line({"recommended": None, "equivalent_options": [o, o], "alternatives": []})[0]
    assert tie.startswith("none (tie: ")
    assert appendix_name("PRC 3") == "tab_verdicts_prc3"


def test_appendix_line_uses_the_thesis_wording():
    from tools.thesis_figures.tab_example_verdicts import _option
    assert _option({"levers": ["damper_bump_ls_rl", "damper_bump_ls_rr"], "direction": "stiffer"}) == \
        "rear low-speed bump stiffer"
    assert _option({"levers": ["ride_height_rear"], "direction": "decrease"}) == "rear ride height lower"
    assert _option({"levers": ["diff_position"], "direction": "increase"}) == "differential locking higher"


def test_screenshot_text_hides_step_sizes():
    from tools.thesis_figures.app_screenshots import hide_steps
    assert hide_steps("Rear LS bump stiffer (1 step) | [hidden] -> [hidden]") == \
        "Rear LS bump stiffer ([hidden]) | [hidden] -> [hidden]"
    assert hide_steps("driver: TC lon +1") == "driver: TC lon [hidden]"
    assert hide_steps("C15 - medium - unstable yaw - exit - 5/5") == "C15 - medium - unstable yaw - exit - 5/5"
