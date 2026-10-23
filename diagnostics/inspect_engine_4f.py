# Setup engine before / after work order 4f (lever pass-over, author EXECUTE
# 2026-10-23), 14 development sessions. Report only.
#
# Reuses the frozen-chain summaries of diagnostics/results_engine_4e_20261023
# (the chain is unchanged since schema 19), so only the engine re-runs:
#   snapshot <tag>  data layer per session with the current config -> <tag>.pkl
#   logged          logged switch positions per session (parse_csv only, the
#                   channels setup_state reads) -> logged.pkl
#   compare <a> <b> recommended change per corner, a -> b, plus the lever
#                   coverage per symptom x phase group x speed class
#   list <tag>      setup recommendation and cockpit line per corner (4g
#                   split: mechanical levers vs driver-adjustable levers)
# Output diagnostics/results_engine_4f_20261023/.
# Run: python -m diagnostics.inspect_engine_4f snapshot pre4f | logged |
#      snapshot post4f | compare pre4f post4f

import os
import pickle
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from diagnostics.inspect_cs_margin_l import CANDIDATES  # noqa: E402

SRC = os.path.join(ROOT, "diagnostics", "results_engine_4e_20261023")
OUT = os.path.join(ROOT, "diagnostics", "results_engine_4f_20261023")


def _logged():
    fn = os.path.join(OUT, "logged.pkl")
    return pickle.load(open(fn, "rb")) if os.path.exists(fn) else {}


def snapshot(tag):
    from modules.setup_engine import data_layer
    logged = _logged()
    res = {}
    for c in CANDIDATES:
        d = pickle.load(open(os.path.join(SRC, f"{c[0]}.pkl"), "rb"))
        res[c[0]] = {"label": d["label"],
                     "layer": data_layer(d["summaries"], tc_map=d["tc_map"], logged=logged.get(c[0]))}
    os.makedirs(OUT, exist_ok=True)
    pickle.dump(res, open(os.path.join(OUT, f"{tag}.pkl"), "wb"))
    print(f"snapshot {tag}: {sum(1 for v in res.values() for x in v['layer']['corners'] if x['recommended'])} "
          f"corner recommendations, logged values {'used' if logged else 'none'}")


def logged():
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv
    from modules.setup_state import logged_setup_values
    out = {}
    for c in CANDIDATES:
        data = parse_csv(resolve_data_path(c[4][1]))
        valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
        out[c[0]] = logged_setup_values(data["channels"], data["laps"], valid)
        print(c[0], out[c[0]], flush=True)
    os.makedirs(OUT, exist_ok=True)
    pickle.dump(out, open(os.path.join(OUT, "logged.pkl"), "wb"))


def _rec(c):
    r = c.get("recommended")
    if r is None:
        return None
    ch = r["change"]
    vals = ", ".join(f"{k['from']} -> {k['to']}" for k in ch["keys"] if k["from"] is not None)
    return (f"{'+'.join(r['levers'])} {r['direction']} ({r['symptom']} @ {r['phase']}, {r['mechanism_id']}"
            + (f", {vals}" if vals else "") + f", {ch['feasibility']})")


def coverage():
    from modules.setup_engine import _matching_rows, load_mechanisms
    mech = load_mechanisms()
    L = ["LEVER COVERAGE (lever groups per symptom x phase group x speed class)"]
    for sym in ("US", "OS", "INST"):
        for g, members in mech["phase_groups"].items():
            row = []
            for sp in ("low", "medium", "high"):
                rows = _matching_rows(sym, members[0], sp, mech)
                row.append(f"{sp} {len({tuple(r['levers']) for r in rows})}")
            L.append(f"  {sym} {g}: " + ", ".join(row))
    return L


def _line(r):
    if r is None:
        return "none"
    ch = r["change"]
    vals = ", ".join(f"{k['from']} -> {k['to']}" for k in ch["keys"] if k["from"] is not None)
    return (f"{'+'.join(r['levers'])} {r['direction']} ({r['symptom']} @ {r['phase']}, {r['mechanism_id']}, "
            f"{ch['effort']}" + (f", {vals}" if vals else "") + ")")


def listing(tag):
    T = pickle.load(open(os.path.join(OUT, f"{tag}.pkl"), "rb"))
    L = [f"SETUP / COCKPIT PER CORNER ({tag}), 14 development sessions", ""]
    for key, v in T.items():
        corners = v["layer"]["corners"]
        L.append(f"{v['label']} ({key}): {len(corners)} corner(s)")
        for c in corners:
            L.append(f"    C{c['corner']} ({c['speed_class']}): setup {_line(c.get('recommended'))}")
            L.append(f"        driver can also: {_line(c.get('cockpit'))}")
    txt = "\n".join(L)
    with open(os.path.join(OUT, f"list_{tag}.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)


def compare(a, b):
    A = pickle.load(open(os.path.join(OUT, f"{a}.pkl"), "rb"))
    B = pickle.load(open(os.path.join(OUT, f"{b}.pkl"), "rb"))
    L = [f"SETUP ENGINE {a} -> {b} (work order 4f, 2026-10-23), 14 development sessions", ""]
    n_a = n_b = n_ch = 0
    for key in A:
        ra = {c["corner"]: _rec(c) for c in A[key]["layer"]["corners"]}
        rb = {c["corner"]: _rec(c) for c in B[key]["layer"]["corners"]}
        n_a += sum(v is not None for v in ra.values())
        n_b += sum(v is not None for v in rb.values())
        lines = [f"    C{cid}: {ra.get(cid)} -> {rb.get(cid)}" for cid in sorted(set(ra) | set(rb))
                 if ra.get(cid) != rb.get(cid)]
        n_ch += len(lines)
        L.append(f"{B[key]['label']} ({key}): {len(lines)} changed")
        L += lines
    L += ["", f"TOTAL recommendations {a} {n_a}, {b} {n_b}; changed {n_ch}", ""] + coverage()
    txt = "\n".join(L)
    with open(os.path.join(OUT, f"compare_{a}_{b}.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "snapshot":
        snapshot(sys.argv[2])
    elif mode == "logged":
        logged()
    elif mode == "list":
        listing(sys.argv[2])
    else:
        compare(sys.argv[2], sys.argv[3])
