import json
import os
REPO = r"C:\Paul\UNI\Bachelorarbeit\Setuptool_local"
here = os.path.dirname(os.path.abspath(__file__))
old = json.load(open(os.path.join(REPO, "diagnostics", "results_sens1_recount_20261029", "analysis.json")))
new = json.load(open(os.path.join(here, "copies", "sens", "diagnostics", "results_sens1_recount_20261029", "analysis.json")))


def pct(a, b):
    return f"{a} / {b} ({100.0 * a / b:.1f} %)"


for lab, d in (("OLD (2026-10-29, schema 30)", old), ("NEW (2026-11-06, schema 33)", new)):
    b = d["baseline"]
    recs = b["recs"]
    n_rec = sum(1 for v in recs.values() if v[0] == "recommended")
    n_tie = sum(1 for v in recs.values() if v[0] == "tie")
    print(f"{lab} baseline: units {sum(b['units_per_kind'].values())} ({b['units_per_kind']['US']} per kind); "
          f"symptoms US {b['symptoms']['US']}, OS {b['symptoms']['OS']}, INST {b['symptoms']['INST']}; "
          f"recommendations {n_rec}, ties {n_tie}, none {len(recs) - n_rec - n_tie}")
print()
for k in old["variants"]:
    o, n = old["variants"][k], new["variants"].get(k)
    print(f"- {o['label']} ({k}):")
    for lab, v in (("old", o), ("new", n)):
        if v is None:
            print(f"    {lab}: missing")
            continue
        pk = v["per_kind"]
        print(f"    {lab}: {pct(v['changed'], v['units'])}; front US {pct(*pk['US'])}, rear OS {pct(*pk['OS'])}, "
              f"INST {pct(*pk['INST'])}; recommendations changed {len(v['rec_changed'])} / 14"
              + (f"; segmentation differs {v['segmentation_differs']}" if v.get("segmentation_differs") else "")
              + (f"; missing {v['missing']}" if v.get("missing") else ""))
    if n:
        for r in n["rec_changed"]:
            print(f"      new rec change: {r}")
