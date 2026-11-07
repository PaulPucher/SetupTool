# ORDER INST-2 addendum W (author, 2026-11-07; replaces the tier addendum T):
# band as INST-2 item 1 plus INST votes with weight 1 (as a moderate CS
# symptom) instead of 2. Post-processing of the stored INST-1 runs, no chain
# run. The vote weight is changed in memory by wrapping
# setup_engine.symptom_votes (INST severity and verdicts untouched).
# "Comes from INST": the share of the recommended option's support weight
# that INST votes supply -- reported as all / majority / part.
# Run from the repo root: python <this file>
import json
import os
import pickle
import sys
from fractions import Fraction

ROOT = os.getcwd()
sys.path.insert(0, ROOT)
from diagnostics import inspect_inst_margin as im  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_inst_margin_20261107")
im.OUT = OUT


def _patch_w():
    import modules.setup_engine as se
    orig = se.symptom_votes

    def sv(*a, **kw):
        votes = orig(*a, **kw)
        for v in votes:
            if v["symptom"] == "INST" and not v["driver_only"]:
                v["severity_weight"] = 1
                v["weight"] = v["k"] / v["n"]
        return votes
    se.symptom_votes = sv
    return orig


def _layer(fin, boundary, w):
    import modules.setup_engine as se
    orig_thr = im._patch_thr(boundary)
    orig_sv = _patch_w() if w else None
    try:
        return se.data_layer(fin["summaries"], fin["setup_data"], tc_map=fin["tc_map"], logged=fin["logged"])
    finally:
        if orig_sv is not None:
            se.symptom_votes = orig_sv
        im._restore_thr(orig_thr)


def _rec(dl):
    rec = dl["session_recommendation"]
    tops = [rec["recommended"]] if rec.get("recommended") else rec.get("equivalent_options") or []
    if not tops:
        return None
    o = tops[0]
    inst = sum(Fraction(v["weight"]).limit_denominator(1000) for v in o["supporting"] if v["symptom"] == "INST")
    tot = sum(Fraction(v["weight"]).limit_denominator(1000) for v in o["supporting"])
    src = "all" if tot and inst == tot else "majority" if 2 * inst > tot else "part" if inst else "none"
    return {"option": "+".join(o["levers"]) + " " + o["direction"], "tie": len(tops) > 1, "net": o["net"],
            "support": o["support"], "opposition": o["opposition"], "inst_support": float(inst),
            "other_support": float(tot - inst), "inst_source": src,
            "inst_votes": sorted((v["corner"], v["phase"], round(v["weight"], 3)) for v in o["supporting"]
                                 if v["symptom"] == "INST")}


def _fmt(r):
    if r is None:
        return "none"
    t = " (tie)" if r["tie"] else ""
    return (f"{r['option']}{t}, net {r['net']:.2f} (support {r['support']:.2f}: INST {r['inst_support']:.2f}, "
            f"other {r['other_support']:.2f}; opposition {r['opposition']:.2f}) -> INST share {r['inst_source']}")


def main():
    band = json.load(open(os.path.join(OUT, "inst2_band.json"), encoding="utf-8"))["band"]
    L = [f"INST-2 ADDENDUM W (2026-11-07): band_INST {band:.4f} N m/deg; INST votes weight 1 instead of 2",
         "per session: default (schema 33) | band alone | W; INST share of the recommended option's support", ""]
    res = {"band": band, "sessions": {}}
    n_changed = 0
    src = {"band": {}, "W": {}}
    for n in im.FINAL:
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        d0 = _layer(fin, 0.0, False)
        assert sorted((c["corner"], s["symptom"], s["phase"], s["laps"]) for c in d0["corners"]
                      for s in c["symptoms"]) == fin["pipeline_counted"], n
        r0, rb, rw = _rec(d0), _rec(_layer(fin, -band, False)), _rec(_layer(fin, -band, True))
        ch = (rw or {}).get("option") != (rb or {}).get("option") or (rw or {}).get("net") != (rb or {}).get("net")
        n_changed += ch
        for k, r in (("band", rb), ("W", rw)):
            if r is not None:
                src[k][r["inst_source"]] = src[k].get(r["inst_source"], 0) + 1
        L.append(f"  {n} (outing {fin['outing']}):")
        L.append(f"    default : {_fmt(r0)}")
        L.append(f"    band    : {_fmt(rb)}")
        L.append(f"    W       : {_fmt(rw)}" + ("   [differs from band alone]" if ch else ""))
        if rw and rw["inst_votes"]:
            L.append(f"    W INST votes (corner, phase, weight): {rw['inst_votes']}")
        res["sessions"][n] = {"default": r0, "band": rb, "W": rw, "W_differs_from_band": ch}
    n_b = sum(src["band"].values())
    n_w = sum(src["W"].values())
    L.append("")
    L.append(f"  session recommendations: band alone {n_b}, W {n_w}; W differs from the band alone in {n_changed}")
    for k in ("band", "W"):
        s = src[k]
        L.append(f"  {k}: recommendations from INST -- all {s.get('all', 0)}, majority {s.get('majority', 0)}, "
                 f"part {s.get('part', 0)}, none {s.get('none', 0)}")
    res["source_counts"] = src
    text = "\n".join(L)
    with open(os.path.join(OUT, "inst2_w.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "inst2_w.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


if __name__ == "__main__":
    main()
