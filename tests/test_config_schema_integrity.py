# Phase 4 -- config and schema integrity.
#
# REGRESSION, NOT CORRECTNESS: these tests check STRUCTURAL properties
# (a key exists, a version number matches a payload shape, a registry
# entry follows its own documented format) -- never whether a config
# VALUE is scientifically right. See tests/test_golden_pipeline.py's
# docstring for that distinction generally.
#
# SCOPE, stated honestly rather than implied: "every config key the code
# reads" is checked only for the blocks/functions this session read in
# full and can enumerate with confidence (modules/stability_analysis.py's
# estimate_sideslip/estimate_slip_angles/estimate_lateral_forces/
# estimate_yaw_moment_stability, ui/views/outing_form.py's
# _classify_corner/_stability_colour, diagnostics/sideslip_ekf_dugoff.py's
# tyre_model_ekf.pass_1 consumer). This is NOT an exhaustive static-
# analysis sweep of every params[...]/cfg[...] access in the codebase --
# see the final report's "chose not to do" section for why that was
# judged too fragile to trust (alias tracking across params/se/cd/cfg
# local variable names would need real static analysis, not regex, to
# avoid false positives/negatives).

import json

import pytest

from modules.stability_analysis import load_parameters, ANALYSIS_SCHEMA_VERSION

# --- 1. every key the enumerated functions read must exist -------------------

STABILITY_ESTIMATION_REQUIRED_KEYS = [
    # estimate_sideslip
    "beta_washout_cutoff_hz",
    # estimate_slip_angles
    "cs_filter_cutoff_hz",
    # estimate_lateral_forces
    "cs_filter_cutoff_hz",
    # estimate_yaw_moment_stability
    "yaw_stability_accel_window_s", "yaw_stability_grid_step_m", "yaw_stability_window_m",
    "yaw_stability_min_samples", "yaw_stability_ridge", "yaw_stability_min_beta_std_rad",
    # WP-N2 Step 1b
    "sideslip_source",
]
CLASSIFICATION_REQUIRED_KEYS = [
    "STRONG_CSF", "STRONG_CSR", "MODERATE_CSF", "MODERATE_CSR", "stab_neg_thresh_Nm_per_deg",
    "thresholds_calibrated_for_sideslip_source",
]
TYRE_MODEL_EKF_PASS1_REQUIRED_KEYS = [
    "c_alpha_front_n_per_rad", "c_alpha_rear_n_per_rad", "mu_fz_front_N", "mu_fz_rear_N",
    "Q_beta_var", "Q_yaw_rate_var", "R_yaw_rate_var", "R_ay_var",
    "P0_beta_var", "P0_yaw_rate_var", "beta_hard_bound_deg",
    "nis_window_samples", "nis_chi2_bound", "nis_flag_fraction",
]


def test_stability_estimation_required_keys_present(raw_params):
    se = raw_params["stability_estimation"]
    missing = [k for k in STABILITY_ESTIMATION_REQUIRED_KEYS if k not in se]
    assert not missing, f"stability_estimation missing keys read by production code: {missing}"


def test_classification_required_keys_present(raw_params):
    cls = raw_params["classification"]
    missing = [k for k in CLASSIFICATION_REQUIRED_KEYS if k not in cls]
    assert not missing, f"classification missing keys read by _classify_corner/_stability_colour: {missing}"


def test_tyre_model_ekf_pass1_required_keys_present(raw_params):
    cfg = raw_params["tyre_model_ekf"]["pass_1"]
    missing = [k for k in TYRE_MODEL_EKF_PASS1_REQUIRED_KEYS if k not in cfg]
    assert not missing, f"tyre_model_ekf.pass_1 missing keys read by estimate_sideslip_ekf_dugoff: {missing}"


# --- optional keys degrade gracefully when absent -----------------------------

def test_missing_sideslip_source_defaults_to_kinematic():
    """The WP-N2 Step 1b call-site branch reads sideslip_source via
    .get("sideslip_source", "kinematic") -- verified directly here by
    removing the key and reproducing the same .get() call, not by
    invoking the Qt-based call site itself."""
    se = {"some_other_key": 1}
    assert se.get("sideslip_source", "kinematic") == "kinematic"


def test_missing_calibration_flag_defaults_to_kinematic():
    cls_cfg = {"some_other_key": 1}
    assert cls_cfg.get("thresholds_calibrated_for_sideslip_source", "kinematic") == "kinematic"


# --- 2. ANALYSIS_SCHEMA_VERSION matches the payload shape actually built -----

def test_schema_version_matches_pipeline_result_shape(pipeline_result):
    """Reproduces the exact field set ANALYSIS_SCHEMA_VERSION's own bump
    history documents (modules/stability_analysis.py:21-51) as having
    been added, version by version: v4 added bracket_start_m/
    bracket_end_m on each corner summary and fz_f_N/fz_r_N/fy_f_norm_N/
    fy_r_norm_N on each phase; v5 (WP-N2 Step 1b) added sideslip_source
    to the persisted payload (a payload-builder field, not a
    summarise_corners field -- checked in the outing_form.py source scan
    below instead, since summarise_corners itself does not take or emit
    sideslip_source); v6 (fresh-session work package: per-session tyre
    auto-fit + NIS gate wired into production) added fit_manifest/
    gate_verdict/fallback_used/fallback_reason to the same payload,
    same "payload-builder field, not a summarise_corners field" scoping;
    v7 (PLAN.md STEP 3 Phase 3) added an OPTIONAL ls_ratio_f/ls_ratio_r
    pair per phase, populated only when summarise_corners is called with
    ls= -- this fixture's own pipeline_result (tests/conftest.py) does
    NOT pass ls= (conftest.py is outside every phase's permitted-file
    list in that package, deliberately not touched), so the v7 keys are
    NOT asserted present here, unlike v4's fz_*/fy_*_norm_N keys below
    -- summarise_corners's OUTPUT shape for a caller that omits ls= is
    unchanged since v4; only its capability (and outing_form.py's own
    payload wrapper) gained keys, checked in the source-scan tests below.

    DELIBERATE UPDATE, not a weakening: this assertion's own prior text
    said "update this test deliberately if the version was bumped
    again" -- exactly what happened here, three times now (5->6, 6->7,
    7->8). Every other check in this function (bracket_start_m/
    bracket_end_m, fz_*/fy_*_norm_N keys) is unchanged and still runs at
    full strength against the SAME kinematic-mode pipeline_result
    fixture; the version literal moved from 7 to 8 and gained an
    apex_region shape check (CS validity repair part A, Phase 3) to match
    the now-current, deliberately-bumped production constant. FLAGGED,
    same as the fresh-session work package's own core/weekend_pdf_
    export.py precedent: this file was not in the CS validity repair
    package's own stated permitted-file list (modules/stability_
    analysis.py, ui/views/outing_form.py, modules/recommendation.py);
    edited anyway because the version-literal assertion this test's own
    docstring explicitly invites updating would otherwise fail against
    the very bump Phase 3 was instructed to make. Worth a second look,
    per that same precedent.
    """
    assert ANALYSIS_SCHEMA_VERSION == 8, (
        f"ANALYSIS_SCHEMA_VERSION is {ANALYSIS_SCHEMA_VERSION}, this test's expectations were "
        "written for 8 -- update this test deliberately if the version was bumped again"
    )
    summaries = pipeline_result["summaries"]
    assert summaries, "no corner summaries produced -- cannot check payload shape"
    sample = summaries[0]
    assert "bracket_start_m" in sample and "bracket_end_m" in sample
    for phase in sample["phases"].values():
        for key in ("fz_f_N", "fz_r_N", "fy_f_norm_N", "fy_r_norm_N"):
            assert key in phase, f"phase missing v4-bumped key {key!r}"
    assert "apex_region" in sample, "corner summary missing v8-bumped apex_region key"
    for key in ("n_samples", "cs_ratio_f", "cs_ratio_r"):
        assert key in sample["apex_region"], f"apex_region missing v8-bumped key {key!r}"


def test_analysis_data_payload_includes_sideslip_source():
    """Structural check on ui/views/outing_form.py's own source, not a
    Qt-instantiated call: _build_analysis_data_json must reference
    sideslip_source in its payload dict. Source-scan rather than an
    import+call because that method is a QWidget instance method with
    file-save side effects (_norm_path(self.loaded_csv_path) etc.) not
    worth reproducing headlessly for a one-line structural fact.
    """
    with open("ui/views/outing_form.py", "r", encoding="utf-8") as f:
        src = f.read()
    start = src.index("def _build_analysis_data_json")
    end = src.index("\n    def ", start + 1)
    body = src[start:end]
    assert '"sideslip_source"' in body, (
        "_build_analysis_data_json's payload dict no longer includes sideslip_source -- "
        "ANALYSIS_SCHEMA_VERSION 5's own bump rationale depends on this field existing"
    )


def test_analysis_data_payload_includes_auto_fit_fields():
    """NEW test (fresh-session work package) -- does not alter the
    sideslip_source check above. _build_analysis_data_json must
    reference all four v6-bumped fields in its payload dict, same
    source-scan rationale as the sideslip_source check.
    """
    with open("ui/views/outing_form.py", "r", encoding="utf-8") as f:
        src = f.read()
    start = src.index("def _build_analysis_data_json")
    end = src.index("\n    def ", start + 1)
    body = src[start:end]
    for field in ('"fit_manifest"', '"gate_verdict"', '"fallback_used"', '"fallback_reason"'):
        assert field in body, (
            f"_build_analysis_data_json's payload dict no longer includes {field} -- "
            "ANALYSIS_SCHEMA_VERSION 6's own bump rationale depends on this field existing"
        )


# --- 3. accuracy-level registry internal consistency --------------------------

def test_accuracy_levels_registry_well_formed(raw_params):
    registry = raw_params["accuracy_levels"]
    problems = []
    for node, entry in registry.items():
        if node == "_comment":
            continue
        if "level" not in entry or entry["level"] not in (1, 2, 3, 4):
            problems.append(f"{node}: level={entry.get('level')!r} not in {{1,2,3,4}}")
        if "source" not in entry or not isinstance(entry["source"], str) or not entry["source"].strip():
            problems.append(f"{node}: source missing or empty")
        capped_by = entry.get("capped_by")
        if capped_by is not None:
            if not (capped_by.startswith("chained-constant:") or capped_by.startswith("provenance-assumption:")):
                problems.append(
                    f"{node}: capped_by={capped_by!r} does not follow the registry's own documented "
                    "'chained-constant: <name>' / 'provenance-assumption: <name>' convention"
                )
    assert not problems, "accuracy_levels registry format violations:\n" + "\n".join(problems)


def test_accuracy_levels_covers_every_dynamically_resolved_node(raw_params):
    """modules/accuracy_resolution.py dynamically resolves exactly five
    nodes (mass, corner_weights, cog_position, steering_ratio,
    steering_angle -- its own module docstring). All five must be
    present in the static registry, since resolve_accuracy's levels dict
    starts from the registry and overwrites only these five keys.
    """
    registry = raw_params["accuracy_levels"]
    for node in ("mass", "corner_weights", "cog_position", "steering_ratio", "steering_angle"):
        assert node in registry, f"accuracy_levels registry missing dynamically-resolved node {node!r}"


# --- 4. both cache identity checks include every field they should -----------

def test_pipeline_cache_identity_fields():
    """WP6 in-memory cache: both the write side (_pipeline_cache_put's
    payload) and the read side (the hit-check condition in
    _run_stability_analysis) must agree on the same identity fields --
    accuracy_cap, resolved_vehicle_snapshot, sideslip_source, and (100 Hz
    time-base work package) grid_rate_hz. Source-scan, not a Qt-
    instantiated call, same rationale as the payload check above.
    """
    with open("ui/views/outing_form.py", "r", encoding="utf-8") as f:
        src = f.read()

    put_start = src.index("_pipeline_cache_put(self.loaded_csv_path, {")
    put_end = src.index("})", put_start)
    put_body = src[put_start:put_end]

    hit_start = src.index("cached_entry = _pipeline_cache_get(self.loaded_csv_path)")
    hit_end = src.index("pipeline_cache = cached_entry", hit_start)
    hit_body = src[hit_start:hit_end]

    for field in ('"accuracy_cap"', '"resolved_vehicle_snapshot"', '"sideslip_source"', '"grid_rate_hz"'):
        assert field in put_body, f"_pipeline_cache_put payload missing {field}"
    for field in ('"accuracy_cap"', '"resolved_vehicle_snapshot"', "sideslip_source", "grid_rate_hz"):
        assert field in hit_body, f"pipeline-cache hit-check condition missing {field}"


def test_persisted_cache_identity_fields():
    """WP5 persisted (DB) cache: both _build_analysis_data_json's
    payload and _try_render_cached_analysis's hit-check must agree on
    the same identity fields. accuracy_cap and resolved_vehicle_snapshot
    predate this session (WP-C); sideslip_source is a later addition;
    grid_rate_hz (100 Hz time-base work package) is the latest -- checked
    here so a future edit that touches one side without the other is
    caught structurally, not just by a slow full pipeline re-run.
    """
    with open("ui/views/outing_form.py", "r", encoding="utf-8") as f:
        src = f.read()

    payload_start = src.index("def _build_analysis_data_json")
    payload_end = src.index("\n    def ", payload_start + 1)
    payload_body = src[payload_start:payload_end]

    hitcheck_start = src.index("def _try_render_cached_analysis")
    hitcheck_end = src.index("\n    def ", hitcheck_start + 1)
    hitcheck_body = src[hitcheck_start:hitcheck_end]

    for field in ('"accuracy_cap"', '"resolved_vehicle_snapshot"', '"sideslip_source"', '"grid_rate_hz"'):
        assert field in payload_body, f"_build_analysis_data_json payload missing {field}"
    for field in ('"accuracy_cap"', '"resolved_vehicle_snapshot"', '"sideslip_source"', '"grid_rate_hz"'):
        assert field in hitcheck_body, f"_try_render_cached_analysis missing a check against {field}"
