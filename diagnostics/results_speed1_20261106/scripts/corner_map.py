# ORDER SPEED-1c item 6: stable corner ID map, pre-speed-fix chain (all
# switches off) -> new default, by lap-distance overlap of each stable
# corner's median bracket (analysis laps). Run in a fresh process per copy:
#   python corner_map.py dump <copy root> <out json>
#   python corner_map.py map <old json> <new json> <out json>
import json
import os
import sys

import numpy as np

OUTINGS = {"NBR1": 17, "PRC3": 26}


def dump(root, dst):
    sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "tools", "thesis_figures"))
    import common  # noqa
    import modules
    assert os.path.normcase(os.path.dirname(list(modules.__path__)[0])) == os.path.normcase(os.path.normpath(root))
    out = {}
    for key, oid in OUTINGS.items():
        sess = common.load_session(oid)
        valid = set(sess["ident"]["lap_filter"])
        by = {}
        for c in sess["parsed"]["corners"]:
            cid = c.get("stable_corner_id")
            if cid is None or c.get("lap_number") not in valid or c.get("bracket_start_m") is None:
                continue
            by.setdefault(cid, []).append((c["bracket_start_m"], c["bracket_end_m"]))
        out[key] = {str(cid): {"start_m": float(np.median([a for a, _ in v])), "end_m": float(np.median([b for _, b in v])),
                               "instances": len(v)} for cid, v in sorted(by.items())}
        out[key + "_instances_total"] = len(sess["parsed"]["corners"])
    json.dump(out, open(dst, "w", encoding="utf-8"), indent=1)


def mapping(old_p, new_p, dst):
    old, new = json.load(open(old_p)), json.load(open(new_p))
    res = {}
    for key in OUTINGS:
        o, n = old[key], new[key]
        rows = []
        for oid, a in o.items():
            best = []
            for nid, b in n.items():
                ov = min(a["end_m"], b["end_m"]) - max(a["start_m"], b["start_m"])
                if ov > 0:
                    best.append((ov / min(a["end_m"] - a["start_m"], b["end_m"] - b["start_m"]), nid))
            best.sort(reverse=True)
            rows.append({"old": oid, "old_range_m": [round(a["start_m"]), round(a["end_m"])],
                         "new": [nid for _, nid in best], "overlap": [round(f, 2) for f, _ in best],
                         "new_range_m": [[round(n[nid]["start_m"]), round(n[nid]["end_m"])] for _, nid in best]})
        hit = {nid for r in rows for nid in r["new"]}
        res[key] = {"rows": rows, "new_without_old": [{"new": nid, "range_m": [round(b["start_m"]), round(b["end_m"])]}
                                                      for nid, b in n.items() if nid not in hit],
                    "instances_old": old[key + "_instances_total"], "instances_new": new[key + "_instances_total"]}
    json.dump(res, open(dst, "w", encoding="utf-8"), indent=1)
    for key, r in res.items():
        print(f"== {key}: instances {r['instances_old']} -> {r['instances_new']}")
        for row in r["rows"]:
            m = "; ".join(f"C{nid} ({f:.2f}, {rng[0]}-{rng[1]} m)" for nid, f, rng in
                          zip(row["new"], row["overlap"], row["new_range_m"])) or "no new corner"
            print(f"  C{row['old']} {row['old_range_m'][0]}-{row['old_range_m'][1]} m -> {m}")
        for x in r["new_without_old"]:
            print(f"  new C{x['new']} {x['range_m'][0]}-{x['range_m'][1]} m has no old counterpart")


if __name__ == "__main__":
    if sys.argv[1] == "dump":
        dump(sys.argv[2], sys.argv[3])
    else:
        mapping(sys.argv[2], sys.argv[3], sys.argv[4])
