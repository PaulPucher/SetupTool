# Saved data paths across machines (UI-1 item 1): an outing saved on the
# laptop (D:/Data_Bachelorthesis) must reload its file and hit its caches on
# the temporary machine (H:/Data_Bachelorthesis), and the other way round.
# Real export, two spellings of its path.

import os

import pytest

from modules.data_paths import load_roots, path_key, resolve_saved_path, split_root
from modules.pipeline_sidecar import _first_mismatch, build_identity
from tests.conftest import find_data_file

NBR29_REL = "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"
OUTING_ROOTS = ("D:/Data_Bachelorthesis", "H:/Data_Bachelorthesis")


@pytest.fixture(scope="module")
def spellings():
    here = find_data_file(NBR29_REL)
    if here is None:
        pytest.skip("NBR JST 29 not on this machine")
    root, _rel = split_root(here)
    other = next(r for r in OUTING_ROOTS if os.path.normcase(r) != os.path.normcase(root))
    return here, f"{other}/{NBR29_REL}"


def test_config_roots_match_the_test_roots():
    from tests.conftest import DATA_ROOTS
    assert tuple(load_roots()) == DATA_ROOTS


def test_other_machine_path_resolves_to_the_local_file(spellings):
    here, other = spellings
    assert not os.path.exists(other)
    assert os.path.normcase(resolve_saved_path(other)) == os.path.normcase(os.path.normpath(here))
    assert resolve_saved_path(here) == here


def test_cache_key_ignores_the_machine_root(spellings):
    here, other = spellings
    assert path_key(here) == path_key(other)
    assert path_key(here) != path_key(here.replace("JST-26.07.01.1734", "JST-26.07.01.1509"))


def test_sidecar_identity_matches_across_machines(spellings):
    here, other = spellings
    common = dict(schema_version=25, accuracy_cap=None, resolved_vehicle_snapshot={"m": 1},
                  sideslip_source="ekf_auto_pacejka", grid_rate_hz=100.0, lap_filter=[1, 2])
    stored = build_identity(csv_path=os.path.normcase(os.path.normpath(other)), **common)
    expected = build_identity(csv_path=os.path.normcase(os.path.normpath(here)), **common)
    assert _first_mismatch(stored, expected) is None
    moved = build_identity(csv_path=os.path.normcase(os.path.normpath(here + ".x")), **common)
    assert _first_mismatch(moved, expected) == "csv_path"


def test_path_outside_the_roots_is_unchanged():
    p = "Z:/elsewhere/file.txt"
    assert split_root(p) is None and resolve_saved_path(p) is None
    assert path_key(p) == os.path.normcase(os.path.normpath(p))
