# WP-CACHE Phase 1: targeted unit tests for modules/pipeline_sidecar.py.
# Pure filesystem/pickle module, no Qt/DB/pipeline dependency -- every test
# redirects SIDECAR_DIR into pytest's own tmp_path so the real data/
# analysis_cache/ is never touched. "Restart-simulate" (the work order's
# own phrase) needs no special fixture: load_sidecar reads only from disk,
# with no process-local state to reset between a write and a later load,
# so a plain write-then-load call pair across two identity objects already
# is the restart case.

import gzip
import pickle

import numpy as np
import pytest

from modules import pipeline_sidecar as sidecar


@pytest.fixture(autouse=True)
def _isolated_sidecar_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(sidecar, "SIDECAR_DIR", str(tmp_path / "analysis_cache"))


def _sample_identity(**overrides):
    base = dict(
        schema_version=8,
        csv_path="C:/data/session.txt",
        accuracy_cap=None,
        resolved_vehicle_snapshot={"mass_kg": 1381.9, "cog_x_m": 1.5},
        sideslip_source="ekf_auto_pacejka",
        grid_rate_hz=100,
        lap_filter=[3, 1, 2],
    )
    base.update(overrides)
    return sidecar.build_identity(**base)


def _sample_payload():
    return {
        "csv_path": "C:/data/session.txt",
        "corners": [{"id": 1, "name": "T1"}, {"id": 2, "name": "T2"}],
        "state": {"time": np.linspace(0, 10, 1000), "sample_rate_hz": 100},
        "cs": {"CS_ratio": np.full(1000, 0.5)},
        "stab": {"margin": np.full(1000, 1.2)},
        "fz": {"fz_fl_N": np.full(1000, 3500.0)},
        "ls": {"LS_ratio": np.full(1000, -0.2)},
        "slip": {"alpha_f_filt": np.full(1000, 0.01)},
        "forces": {"Fy_f_filt": np.full(1000, 2000.0)},
        "accuracy_cap": None,
        "resolved_vehicle_snapshot": {"mass_kg": 1381.9, "cog_x_m": 1.5},
        "sideslip_source": "ekf_auto_pacejka",
        "grid_rate_hz": 100,
        "fit_manifest": {"iterations": 2},
        "gate_verdict": "pass",
        "fallback_used": False,
        "fallback_reason": None,
    }


def test_round_trip_write_then_load_matches(tmp_path):
    identity = _sample_identity()
    payload = _sample_payload()
    assert sidecar.write_sidecar(1, identity, payload) is True

    loaded_payload, reason = sidecar.load_sidecar(1, identity)
    assert reason is None
    assert loaded_payload["corners"] == payload["corners"]
    assert loaded_payload["fit_manifest"] == payload["fit_manifest"]
    np.testing.assert_array_equal(loaded_payload["state"]["time"], payload["state"]["time"])
    np.testing.assert_array_equal(loaded_payload["cs"]["CS_ratio"], payload["cs"]["CS_ratio"])


def test_load_missing_file_is_absent_not_an_error(tmp_path):
    payload, reason = sidecar.load_sidecar(999, _sample_identity())
    assert payload is None
    assert reason == "no sidecar file"


@pytest.mark.parametrize("field,override", [
    ("schema_version", {"schema_version": 7}),
    ("csv_path", {"csv_path": "C:/data/other_session.txt"}),
    ("accuracy_cap", {"accuracy_cap": 2}),
    ("resolved_vehicle_snapshot", {"resolved_vehicle_snapshot": {"mass_kg": 1400.0}}),
    ("sideslip_source", {"sideslip_source": "kinematic"}),
    ("grid_rate_hz", {"grid_rate_hz": 50}),
    ("lap_filter", {"lap_filter": [4, 5]}),
])
def test_each_identity_field_mismatch_falls_back(field, override):
    written_identity = _sample_identity()
    assert sidecar.write_sidecar(2, written_identity, _sample_payload()) is True

    current_identity = _sample_identity(**override)
    payload, reason = sidecar.load_sidecar(2, current_identity)
    assert payload is None
    assert reason == field


def test_sidecar_format_version_mismatch_falls_back(monkeypatch):
    identity = _sample_identity()
    assert sidecar.write_sidecar(3, identity, _sample_payload()) is True

    monkeypatch.setattr(sidecar, "SIDECAR_FORMAT_VERSION", 2)
    # build_identity stamps whatever SIDECAR_FORMAT_VERSION is current when
    # called -- the "current" side of the comparison now expects v2 while
    # the stored file is still v1, the exact real-world case (a future
    # code change bumps the format).
    current_identity = sidecar.build_identity(
        schema_version=identity["schema_version"], csv_path=identity["csv_path"],
        accuracy_cap=identity["accuracy_cap"],
        resolved_vehicle_snapshot=identity["resolved_vehicle_snapshot"],
        sideslip_source=identity["sideslip_source"], grid_rate_hz=identity["grid_rate_hz"],
        lap_filter=identity["lap_filter"],
    )
    payload, reason = sidecar.load_sidecar(3, current_identity)
    assert payload is None
    assert reason == "sidecar_format_version"


def test_garbage_file_is_corrupt_not_an_exception(tmp_path):
    sidecar_path = sidecar._sidecar_path(4)
    import os
    os.makedirs(sidecar.SIDECAR_DIR, exist_ok=True)
    with open(sidecar_path, "wb") as f:
        f.write(b"not a gzip file at all")

    payload, reason = sidecar.load_sidecar(4, _sample_identity())
    assert payload is None
    assert reason == "unreadable/corrupt sidecar"


def test_truncated_mid_payload_stream_falls_back(tmp_path):
    # Amendment 1: a real write, then the file truncated partway through
    # the SECOND pickled object (the payload) -- the identity header must
    # still decode cleanly and match, so this exercises the payload
    # pickle.load's own EOFError path specifically, not just "any corrupt
    # file" (test_garbage_file_is_corrupt_not_an_exception above already
    # covers a file that fails at the very first read).
    identity = _sample_identity()
    assert sidecar.write_sidecar(5, identity, _sample_payload()) is True

    sidecar_path = sidecar._sidecar_path(5)
    with open(sidecar_path, "rb") as f:
        full_bytes = f.read()
    # Decompress, cut the decompressed stream partway through (well past
    # the small identity object, into the large payload), then re-compress
    # so the file is a well-formed gzip stream that is simply short --
    # exactly what an interrupted write (pre-atomic-rename) would look like
    # if a caller ever read a .tmp file directly.
    raw = gzip.decompress(full_bytes)
    truncated_raw = raw[: len(raw) // 2]
    with open(sidecar_path, "wb") as f:
        f.write(gzip.compress(truncated_raw))

    payload, reason = sidecar.load_sidecar(5, identity)
    assert payload is None
    assert reason == "unreadable/corrupt sidecar"


def test_write_is_atomic_no_tmp_file_left_after_success(tmp_path):
    identity = _sample_identity()
    assert sidecar.write_sidecar(6, identity, _sample_payload()) is True

    import os
    assert os.path.exists(sidecar._sidecar_path(6))
    assert not os.path.exists(sidecar._sidecar_path(6) + ".tmp")


def test_write_failure_never_produces_a_partial_final_file(monkeypatch):
    call_count = {"n": 0}
    real_dump = pickle.dump

    def flaky_dump(obj, f, protocol=None):
        call_count["n"] += 1
        if call_count["n"] == 2:  # the payload dump, after identity already wrote fine
            raise RuntimeError("simulated failure mid-write")
        return real_dump(obj, f, protocol=protocol)

    monkeypatch.setattr(sidecar.pickle, "dump", flaky_dump)

    result = sidecar.write_sidecar(7, _sample_identity(), _sample_payload())
    assert result is False

    import os
    assert not os.path.exists(sidecar._sidecar_path(7))


def test_write_creates_sidecar_dir_if_missing(tmp_path):
    import os
    assert not os.path.isdir(sidecar.SIDECAR_DIR)
    assert sidecar.write_sidecar(8, _sample_identity(), _sample_payload()) is True
    assert os.path.isdir(sidecar.SIDECAR_DIR)
