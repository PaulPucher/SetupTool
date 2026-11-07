# ORDER INST-2 addendum (author, 2026-11-07): INST tier variant T, comparison
# only. Post-processing of the stored INST-1 runs, no chain run.
# Item 4: development units with dMz/dbeta below -band_INST -> count,
# p10 / p50 / p90, bootstrap 90 % interval of the p50 (units resampled with
# replacement; units are clustered by session, so the interval is optimistic).
# Item 5: T = INST moderate in (-strong, -band), strong below -strong,
# strong = that p50; INST counts only below -band. Severity is set in memory
# by wrapping phase_symptoms; the session vote weights are the configured
# severity_weight (moderate 1, strong 2).
# Run from the repo root: python <this file>
import json
import os
import pickle
import sys

import numpy as np

ROOT = os.getcwd()
sys.path.insert(0, ROOT)
from diagnostics import inspect_inst_margin as im  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_inst_margin_20261107")
im.OUT = OUT
N_BOOT = 10000
SEED = 20261107


def _patch_tiers(strong):
    import modules.classifier as cl
    import modules.setup_engine as se
    orig = cl.phase_symptoms

    def ps(summary, thr=None):
        res = orig(summary, thr)
        for s in res["symptoms"]:
            if s["symptom"] == "INST":
                s["severity"] = "strong" if s["value"] < strong else "moderate"
        return res
    cl.phase_symptoms, se.phase_symptoms = ps, ps
    return orig


def _restore(orig):
    import modules.classifier as cl
    import modules.setup_engine as se
    cl.phase_symptoms, se.phase_symptoms = orig, orig


def _inst_tiers(fin):
    from modules.classifier import aggregate_by_corner, phase_symptoms
    out = {"moderate": [], "strong": []}
    for cid, a in aggregate_by_corner(fin["summaries"]).items():
        for s in phase_symptoms(a)["symptoms"]:
            if s["symptom"] == "INST":
                out[s["severity"]].append((cid, s["phase"], round(s["value"], 1)))
    return out


def _severity(fin):
    from modules.classifier import aggregate_by_corner, classify_corner
    return {cid: classify_corner(a)[0] for cid, a in aggregate_by_corner(fin["summaries"]).items()}


def main():
    band = json.load(open(os.path.join(OUT, "inst2_band.json"), encoding="utf-8"))["band"]
    rows, _seg, _stat = im._units("agg")
    v = np.array(sorted(r["stab"] for r in rows if r["stab"] < -band))
    per_sess = {}
    for r in rows:
        if r["stab"] < -band:
            per_sess[r["session"]] = per_sess.get(r["session"], 0) + 1
    rng = np.random.default_rng(SEED)
    boot = np.median(rng.choice(v, size=(N_BOOT, v.size), replace=True), axis=1)
    p10, p50, p90 = np.percentile(v, [10, 50, 90])
    lo, hi = np.percentile(boot, [5, 95])
    strong = float(p50)
    L = [f"INST-2 ADDENDUM (2026-11-07), band_INST {band:.4f} N m/deg (INST-2)",
         f"ITEM 4: development units below -band: {v.size} (of {len(rows)}), sessions "
         f"{len({r['session'] for r in rows if r['stab'] < -band})}",
         f"  p10 {p10:.2f}  p50 {p50:.2f}  p90 {p90:.2f} N m/deg",
         f"  bootstrap 90 % interval of the p50 ({N_BOOT} resamples of the units, seed {SEED}): [{lo:.2f}, {hi:.2f}]",
         "  units per session: " + ", ".join(f"{k} {n}" for k, n in sorted(per_sess.items(), key=lambda kv: -kv[1]))
         + " (clustered by session, so the unit bootstrap interval is optimistic)",
         "  values: " + ", ".join(f"{x:.1f}" for x in v),
         "",
         f"ITEM 5: variant T: INST moderate if {strong:.2f} < dMz/dbeta < {-band:.2f}, strong if below "
         f"{strong:.2f} N m/deg; vote weights moderate 1, strong 2 (configured severity_weight)"]
    res = {"band": band, "units_below_band": v.tolist(), "p10": p10, "p50": p50, "p90": p90,
           "boot90": [lo, hi], "final": {}}
    tv = ts = tc = tr = 0
    tot = 0
    for n in im.FINAL:
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        cur = im._final_eval(fin, 0.0)
        assert cur["counted"] == fin["pipeline_counted"], n
        sev_cur = _severity(fin)
        orig_thr = im._patch_thr(-band)
        orig_ps = _patch_tiers(strong)
        try:
            tiers = _inst_tiers(fin)
            sev_t = _severity(fin)
        finally:
            _restore(orig_ps)
            im._restore_thr(orig_thr)
        orig_ps = _patch_tiers(strong)
        try:
            t = im._final_eval(fin, -band)
        finally:
            _restore(orig_ps)
        tot += len(cur["verdicts"])
        dv = {c: (cur["verdicts"][c], t["verdicts"][c]) for c in cur["verdicts"] if cur["verdicts"][c] != t["verdicts"][c]}
        ds = {c: (sev_cur[c], sev_t[c]) for c in sev_cur if sev_cur[c] != sev_t[c]}
        # counted symptoms compared with their severity
        from modules.setup_engine import counted_symptoms
        from modules.stability_analysis import load_parameters
        cfg = load_parameters()["setup_engine"]
        c0 = {(cid, s["symptom"], s["phase"]): s["severity"] for cid, c in counted_symptoms(fin["summaries"], cfg).items()
              for s in c["symptoms"]}
        orig_thr = im._patch_thr(-band)
        orig_ps = _patch_tiers(strong)
        try:
            c1 = {(cid, s["symptom"], s["phase"]): s["severity"]
                  for cid, c in counted_symptoms(fin["summaries"], cfg).items() for s in c["symptoms"]}
        finally:
            _restore(orig_ps)
            im._restore_thr(orig_thr)
        dc = {k: (c0.get(k), c1.get(k)) for k in set(c0) | set(c1) if c0.get(k) != c1.get(k)}
        rc = cur["rec"] != t["rec"]
        tv += len(dv)
        ts += len(ds)
        tc += len(dc)
        tr += rc
        L.append(f"  {n} (outing {fin['outing']}): INST units moderate {len(tiers['moderate'])} {tiers['moderate']}, "
                 f"strong {len(tiers['strong'])} {tiers['strong']}")
        for c, (x, y) in sorted(dv.items(), key=lambda kv: str(kv[0])):
            L.append(f"    verdict C{c}: {x}  ->  {y}")
        for c, (x, y) in sorted(ds.items(), key=lambda kv: str(kv[0])):
            L.append(f"    corner severity C{c}: {x} -> {y}")
        for k, (x, y) in sorted(dc.items(), key=lambda kv: str(kv[0])):
            L.append(f"    counted C{k[0]} {k[1]} {k[2]}: {x} -> {y}")
        L.append(f"    recommendation: {cur['rec']}" + (f"  ->  {t['rec']}" if rc else " (unchanged)"))
        res["final"][n] = {"tiers": tiers, "verdicts_changed": dv, "severity_changed": ds,
                           "counted_changed": {str(k): v for k, v in dc.items()}, "rec_before": cur["rec"],
                           "rec_after": t["rec"]}
    L.append(f"  totals vs schema-33 default: verdict text changed {tv} of {tot}; corner severity changed {ts}; "
             f"counted symptoms changed (presence or severity) {tc}; recommendations changed {tr}")
    text = "\n".join(L)
    with open(os.path.join(OUT, "inst2_tiers.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "inst2_tiers.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


if __name__ == "__main__":
    main()
