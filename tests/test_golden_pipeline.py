# Golden-value regression on the DEFAULT path (fit L) on two
# thesis sessions, NBR JST outing 29 and Le Castellet outing 32 (author
# ruling 2026-10-20: the Dubai goldens are retired -- Dubai is outside the
# thesis data and its goldens were stale since acf7dbf).
#
# THESE ARE REGRESSION TESTS, NOT CORRECTNESS TESTS. They pin CURRENT
# output (tests/golden/default_*.json, written by tests/generate_golden.py)
# so a change that alters the numbers is VISIBLE; a pass means "nothing
# changed unintentionally", never "the numbers are right".
#
# Pinned per session: fit manifest (volatile fields skipped), gate verdict,
# fallback, corner summaries, the event layer (floor events), the corner
# verdicts at the adopted thresholds and the recommendations. One chain
# function for generator and test (generate_golden.run_default_chain).
#
# APEX FRAGILITY (2026-10-11): apex_3 is one instant +/- 5 samples; its
# medians flip on small beta changes, so a diff concentrated in apex_3 is
# expected after any estimator change.
#
# Float tolerance and NaN handling: tests/_json_utils.py (rtol 1e-6, atol
# 1e-9, NaN == NaN).

import json
import os

import pytest

from tests._json_utils import diff_json
from tests.conftest import find_data_file
from tests.generate_golden import GOLDEN_SESSIONS, golden_path, run_default_chain

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PARTS = ("fit_manifest", "gate_verdict", "summaries", "events", "verdicts", "recommendations")


def _load_golden(key):
    try:
        with open(golden_path(key), "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        pytest.fail(f"{golden_path(key)} not found -- run `python -m tests.generate_golden` deliberately")


@pytest.fixture(scope="module", params=sorted(GOLDEN_SESSIONS))
def run(request):
    key = request.param
    rel = GOLDEN_SESSIONS[key][1]
    if find_data_file(rel) is None:
        pytest.skip(f"{GOLDEN_SESSIONS[key][0]} export not on this machine")
    return key, run_default_chain(rel), _load_golden(key)


def test_golden_metadata_and_mode(run):
    key, got, golden = run
    meta = golden["_meta"]
    for k in ("git_commit_hash", "generated_at_utc", "data_file", "session", "accuracy_cap"):
        assert meta.get(k) not in (None, ""), f"{golden_path(key)} _meta missing {k!r}"
    assert golden["tyre_fit_mode"] == got["tyre_fit_mode"] == "literature_bounded"
    assert golden["sideslip_source"] == got["sideslip_source"] == "ekf_auto_pacejka"


def test_default_did_not_fall_back(run):
    _key, got, _golden = run
    assert not got["fallback_used"], f"the default fell back to kinematic ({got['fallback_reason']})"


@pytest.mark.parametrize("part", PARTS)
def test_output_matches_golden(run, part):
    key, got, golden = run
    diffs = diff_json({part: got[part]}, {part: golden[part]})
    assert not diffs, (
        f"{len(diffs)} field(s) of {part} differ from {golden_path(key)} (golden generated at commit "
        f"{golden['_meta']['git_commit_hash']}, {golden['_meta']['generated_at_utc']}):\n"
        + "\n".join(f"  {p}: got={a!r} expected={b!r}" for p, a, b in diffs[:30])
        + (f"\n  ... and {len(diffs) - 30} more" if len(diffs) > 30 else "")
    )
