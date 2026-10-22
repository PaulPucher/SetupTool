# Quick end-to-end check of the DEFAULT analysis path (fit L, schema 20)
# against the golden snapshot, without pytest. Runs the app's chain function
# for function (tests/generate_golden.run_default_chain) on NBR JST outing 29
# and compares fit manifest, NIS gate, corner summaries, floor events and the
# classifier verdicts with tests/golden/default_nbr_jst29.json.
# The recommendations part is left out: both recommendation engines are being
# replaced (DECISIONS 'Recommendation overhaul rulings', 2026-10-22).
# Regression, not correctness: a pass means nothing changed unintentionally.
#
# Run: python test_stability.py [nbr_jst29 | pr_lc32]   (exit 1 on a diff)

import json
import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tests._json_utils import diff_json
from tests.generate_golden import GOLDEN_SESSIONS, golden_path, run_default_chain

PARTS = ("fit_manifest", "gate_verdict", "summaries", "events", "verdicts")


def _max_numeric_gap(diffs):
    # largest absolute and relative gap among the numeric mismatches
    gaps = [(abs(a - b), abs(a - b) / abs(b) if b else math.inf)
            for _p, a, b in diffs
            if isinstance(a, (int, float)) and isinstance(b, (int, float))
            and not isinstance(a, bool) and not isinstance(b, bool)
            and math.isfinite(a) and math.isfinite(b)]
    if not gaps:
        return None
    return max(g[0] for g in gaps), max(g[1] for g in gaps), len(gaps)


def main():
    key = sys.argv[1] if len(sys.argv) > 1 else "nbr_jst29"
    label, rel = GOLDEN_SESSIONS[key]
    with open(golden_path(key), "r", encoding="utf-8") as f:
        golden = json.load(f)
    got = run_default_chain(rel)

    man, gate = got["fit_manifest"], got["gate_verdict"] or {}
    print(f"{label}: tyre_fit_mode {got['tyre_fit_mode']}, sideslip {got['sideslip_source']}, "
          f"fit status {man.get('status')}, gate {gate.get('verdict')} {gate.get('health_score')}, "
          f"fallback {got['fallback_used']}")
    print(f"  corner summaries {len(got['summaries'])}, floor events {len(got['events']['events'])}, "
          f"non-ok verdicts {sum(1 for v in got['verdicts'] if v['short'] != 'ok')}")
    print(f"  golden: commit {golden['_meta']['git_commit_hash']}, generated {golden['_meta']['generated_at_utc']}")

    failed = got["fallback_used"]
    if failed:
        print(f"  FAIL: default path fell back to kinematic ({got['fallback_reason']})")
    for part in PARTS:
        diffs = diff_json({part: got[part]}, {part: golden[part]})
        if not diffs:
            print(f"  {part}: match")
            continue
        failed = True
        gap = _max_numeric_gap(diffs)
        gap_txt = (f", {gap[2]} numeric, max abs {gap[0]:.3g}, max rel {gap[1]:.3g}" if gap else "")
        print(f"  {part}: {len(diffs)} field(s) differ{gap_txt}")
        for p, a, b in diffs[:5]:
            print(f"    {p}: got={a!r} golden={b!r}")
    print("RESULT: " + ("DIFF against the golden" if failed else "matches the golden"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
