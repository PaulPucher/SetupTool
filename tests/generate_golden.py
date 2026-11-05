# One-off generator for tests/golden/default_*.json -- NOT a test itself
# (pytest does not collect this file). Run manually and deliberately, only
# after an intentional behaviour change:
#   python -m tests.generate_golden
#
# RETARGETED 2026-10-20 (author ruling, final chain part 2): the Dubai
# goldens are retired (Dubai is outside the thesis data, stale since
# acf7dbf); the goldens pin the DEFAULT path -- fit L (tyre_fit_mode
# literature_bounded), CS on Fy / Fz, floor events, the thresholds adopted
# on 2026-10-20 -- on two thesis sessions, NBR JST outing 29 and Le
# Castellet outing 32.
#
# Will not overwrite a golden file that already exists -- delete it first to
# regenerate, so a routine re-run never resets a verified golden's
# provenance (git_commit_hash / generated_at_utc).
#
# REGRESSION, NOT CORRECTNESS: this snapshots CURRENT output -- see
# tests/test_golden_pipeline.py.
#
# Fixed configuration (run_default_chain, shared with the test so the two
# cannot diverge): the app's chain itself (modules.pipeline.run_analysis,
# the function the analysis thread calls), then summarise_corners over all
# laps, at best available accuracy (cap None, as the threshold
# derivation), setup_data None, feedback {}.
# 2026-10-22 (recommendation overhaul): verdicts from modules/classifier.py
# (ruling D2), recommendations = setup engine data layer. Best
# available and the fit-L axle loads both read the gitignored
# config/car_data.json, so the goldens hold on a machine that has it.

import datetime
import json
import os
import subprocess

import numpy as np

GOLDEN_SESSIONS = {
    "nbr_jst29": ("NBR JST outing 29", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"),
    "pr_lc32": ("Le Castellet outing 32", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
}
ACCURACY_CAP = None   # best available
# volatile or machine-specific manifest fields, not behaviour
MANIFEST_SKIP = ("timestamp", "git_commit_hash", "data_file")


def golden_path(key):
    return f"tests/golden/default_{key}.json"


def _git_hash():
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception as e:
        return f"UNAVAILABLE ({e})"


def json_safe(obj):
    """Round-trip through json so tuples, numpy scalars and NaN compare the
    same way freshly computed and loaded from file."""
    def conv(o):
        if isinstance(o, dict):
            return {str(k): conv(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [conv(v) for v in o]
        if isinstance(o, np.floating):
            return float(o)
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.bool_):
            return bool(o)
        return o
    return json.loads(json.dumps(conv(obj), allow_nan=True))


def run_default_chain(rel_path):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, summarise_corners
    from modules.accuracy_resolution import resolve_accuracy
    from modules.pipeline import recommend, run_analysis
    from tests.conftest import resolve_data_path

    path = resolve_data_path(rel_path)
    params = load_parameters()
    data = parse_csv(path)
    chain = run_analysis(data, resolve_accuracy(params, setup_data=None, cap=ACCURACY_CAP), csv_path=path)
    laps = data.get("laps", [])
    man = chain["fit_manifest"]
    summaries = summarise_corners(chain["corners"], chain["cs"], chain["stab"], chain["state"], fz=chain["fz"],
                                  lap_filter=None)
    out = {
        "tyre_fit_mode": params["tyre_fit_auto"]["tyre_fit_mode"], "sideslip_source": chain["sideslip_source"],
        "fallback_used": chain["fallback_used"], "fallback_reason": chain["fallback_reason"],
        "gate_verdict": chain["gate_verdict"],
        "fit_manifest": {k: v for k, v in (man or {}).items() if k not in MANIFEST_SKIP},
        "summaries": summaries, "events": chain["events"],
    }
    # verdicts through the app's classifier (modules/classifier.py) on every
    # summary; recommendations = the setup engine's data layer on the valid
    # analysis laps (no setup sheet, no feedback, no logged channels)
    from modules.classifier import classify_corner

    out["verdicts"] = [{"stable_corner_id": s.get("stable_corner_id"), "lap_number": s.get("lap_number"),
                        "severity": v[0], "short": v[1]} for s, v in ((s, classify_corner(s)) for s in summaries)]
    valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
    out["recommendations"] = recommend([s for s in summaries if s["lap_number"] in valid])["data_layer"]
    return json_safe(out)


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    for key, (label, rel) in GOLDEN_SESSIONS.items():
        target = golden_path(key)
        if os.path.exists(target):
            print(f"skipping {target} (already exists) -- delete it first to regenerate deliberately")
            continue
        res = run_default_chain(rel)
        if res["fallback_used"]:
            raise SystemExit(f"refusing to write {target}: the default fell back to kinematic "
                             f"({res['fallback_reason']})")
        from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
        res["_meta"] = {
            # the schema from the code, never a literal (the label once stayed at
            # 16 through three bumps)
            "purpose": f"REGRESSION baseline of the default path (fit L, schema {ANALYSIS_SCHEMA_VERSION}), "
                       "not a correctness claim -- see tests/test_golden_pipeline.py",
            "session": label, "data_file": rel,
            "git_commit_hash": _git_hash(),
            "generated_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "accuracy_cap": "best available", "setup_data": None,
        }
        with open(target, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1, allow_nan=True)
        print(f"wrote {target} ({len(res['summaries'])} corner summaries, {len(res['events']['events'])} "
              f"events, {len(res['recommendations']['corners'])} corner recommendation(s))")


if __name__ == "__main__":
    main()
