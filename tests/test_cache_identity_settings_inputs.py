# R2 (+R23), 2026-10-17: the Settings Section-2 estimator inputs (moving
# speed, kerb threshold / baseline / dilation, both corner speed boundaries)
# are part of every cache identity, built once in
# accuracy_resolution.resolve_estimator_settings. Real Dubai data where data
# are needed.

import copy

import pytest

from modules import pipeline_sidecar
from modules.accuracy_resolution import resolve_accuracy, resolve_estimator_settings
from modules.corner_analysis import reclassify_speed_classes
from modules.csv_parser import load_channels_config
from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION

# (file, path, identity key) per Settings field
INPUTS = [
    ("parameters", ("stability_estimation", "moving_speed_min_mps"), "moving_speed_min_mps"),
    ("parameters", ("stability_estimation", "kerb_z_deviation_threshold_g"), "kerb_z_deviation_threshold_g"),
    ("parameters", ("stability_estimation", "kerb_baseline_g"), "kerb_baseline_g"),
    ("parameters", ("stability_estimation", "kerb_dilation_s"), "kerb_dilation_s"),
    ("channels", ("corner_speed_thresholds", "low_max"), "corner_speed_low_max"),
    ("channels", ("corner_speed_thresholds", "medium_max"), "corner_speed_medium_max"),
]


def _bumped(raw_params, file_key, path):
    params, channels = copy.deepcopy(raw_params), copy.deepcopy(load_channels_config())
    node = params if file_key == "parameters" else channels
    for k in path[:-1]:
        node = node[k]
    node[path[-1]] = node[path[-1]] + 1
    return params, channels


def test_settings_inputs_cover_every_section2_estimator_field():
    from ui.views.settings_view import SECTION2_CHANNELS_INT_FIELDS, SECTION2_PARAMS_FIELDS
    paths = {spec["path"] for spec in SECTION2_PARAMS_FIELDS + SECTION2_CHANNELS_INT_FIELDS}
    assert paths == {path for _f, path, _k in INPUTS}


@pytest.mark.parametrize("file_key,path,key", INPUTS)
def test_each_input_changes_the_identity(raw_params, file_key, path, key):
    base = resolve_estimator_settings(raw_params, load_channels_config())
    params, channels = _bumped(raw_params, file_key, path)
    moved = resolve_estimator_settings(params, channels)
    assert moved != base
    assert [k for k in base if base[k] != moved[k]] == [key]


def test_resolve_accuracy_carries_the_settings_inputs(raw_params):
    resolved = resolve_accuracy(raw_params, setup_data=None, cap=None)
    assert resolved["estimator_settings"] == resolve_estimator_settings(raw_params)


@pytest.mark.parametrize("file_key,path,key", INPUTS)
def test_sidecar_check_names_the_mismatch(tmp_path, monkeypatch, raw_params, file_key, path, key):
    monkeypatch.setattr(pipeline_sidecar, "SIDECAR_DIR", str(tmp_path))

    def identity(settings):
        return pipeline_sidecar.build_identity(ANALYSIS_SCHEMA_VERSION, "dubai.txt", None, {"mass_kg": 1.0},
                                               "ekf_auto_pacejka", 100.0, [1, 2], estimator_settings=settings)

    stored = resolve_estimator_settings(raw_params, load_channels_config())
    assert pipeline_sidecar.write_sidecar(7, identity(stored), {"x": 1})
    assert pipeline_sidecar.load_sidecar(7, identity(stored)) == ({"x": 1}, None)
    params, channels = _bumped(raw_params, file_key, path)
    payload, reason = pipeline_sidecar.load_sidecar(7, identity(resolve_estimator_settings(params, channels)))
    assert payload is None and reason == f"estimator_settings.{key}"
    # a sidecar written before the field existed misses on it
    assert pipeline_sidecar.write_sidecar(8, identity(None), {"x": 1})
    assert pipeline_sidecar.load_sidecar(8, identity(stored)) == (None, "estimator_settings")


def test_every_cache_check_compares_the_settings_inputs():
    """Source scan (same rationale as test_config_schema_integrity): the
    in-process cache entry and its hit check, the fast path, the saved payload
    and its hit check, and both sidecar identities carry estimator_settings."""
    with open("ui/views/outing_form.py", encoding="utf-8") as f:
        src = f.read()

    def body(start_marker, end_marker):
        a = src.index(start_marker)
        return src[a:src.index(end_marker, a)]

    put = body("pipeline_cache_entry = {", "_pipeline_cache_put(self.loaded_csv_path, pipeline_cache_entry)")
    hit = body("cached_entry = _pipeline_cache_get(self.loaded_csv_path)\n        if (cached_entry",
               "pipeline_cache = cached_entry")
    fast = body("def _run_stability_analysis", "def _on_recompute_clicked")
    payload = body("def _build_analysis_data_json", "\n    def ")
    db_hit = body("def _try_render_cached_analysis", "\n    def ")
    done = body("def _on_stability_done", "\n    def ")
    for name, text in (("cache entry", put), ("in-process hit check", hit), ("fast path", fast),
                       ("payload", payload), ("DB hit check", db_hit)):
        assert "estimator_settings" in text, name
    assert "_SETTINGS_GENERATION" in fast
    for name, text in (("sidecar write", done), ("sidecar read", db_hit)):
        call = text[text.index("build_identity("):]
        assert "estimator_settings=" in call[:call.index(")\n")], name


def test_speed_classes_follow_the_current_boundaries(parsed_data):
    """Unchanged boundaries reproduce the parse-time classes exactly; moved
    boundaries reclassify (the fit's sign check reads speed_class, R23)."""
    corners = copy.deepcopy(parsed_data["corners"])
    thresholds = load_channels_config()["corner_speed_thresholds"]
    before = [c["speed_class"] for c in corners]
    reclassify_speed_classes(corners, thresholds)
    assert [c["speed_class"] for c in corners] == before
    reclassify_speed_classes(corners, {"low_max": 0, "medium_max": 0})
    assert {c["speed_class"] for c in corners} == {"high"}
    # one class per stable corner, as at detection
    by_cid = {}
    for c in corners:
        by_cid.setdefault(c["stable_corner_id"], set()).add(c["speed_class"])
    assert all(len(v) == 1 for v in by_cid.values())
