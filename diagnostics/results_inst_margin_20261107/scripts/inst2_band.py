# ORDER INST-2 (author + review, 2026-11-07): post-processing of the stored
# INST-1 runs (diagnostics/results_inst_margin_20261107/), no chain run.
# Band from measurement and vehicle-input uncertainty only: per unit
# combined = sqrt(r_yaw^2 + veh^2), r_yaw = |d| under R_yaw x 10, veh = largest
# |d| of the ten +/-1 % vehicle runs; band_INST = pooled p90, unrounded. Q x 0.3
# is a model setting for dMz/dbeta (systematic shift, INST-1), reported as a
# sensitivity only. Counterfactual: INST counts below -band_INST; [-band, 0)
# gives no INST (reading A of INST-1 without a tag band).
# Run from the repo root: python <this file>
import json
import os
import sys

import numpy as np

ROOT = os.getcwd()
sys.path.insert(0, ROOT)
from diagnostics import inspect_inst_margin as im  # noqa: E402

OUT = os.path.join(ROOT, "diagnostics", "results_inst_margin_20261107")
im.OUT = OUT


def main():
    rows, seg, stat = im._units("agg")
    sessions = sorted({r["session"] for r in rows})
    comb = [float(np.hypot(r["d"]["ryaw_x10"], r["veh_max"])) for r in rows
            if im._fin(r["d"]["ryaw_x10"]) and im._fin(r["veh_max"])]
    s = im._pct(comb)
    band = s["p90"]
    L = ["INST-2 BAND (2026-11-07): combined = sqrt(r_yaw^2 + veh^2), pooled p90, 14 development sessions",
         f"sessions ({len(sessions)}): {', '.join(sessions)}",
         f"units {len(rows)}, combined defined {s['n']}",
         f"combined p50 {s['p50']:.4f}  p90 {s['p90']:.4f}  p99 {s['p99']:.4f} N m/deg",
         f"band_INST = {band:.4f} N m/deg (unrounded)"]
    for name, vals in (("r_yaw", [r["d"]["ryaw_x10"] for r in rows]), ("veh", [r["veh_max"] for r in rows]),
                       ("Q x 0.3 (sensitivity, not in the band)", [r["d"]["q_x0.3"] for r in rows])):
        p = im._pct(vals)
        L.append(f"  {name}: p50 {p['p50']:.2f}  p90 {p['p90']:.2f}  p99 {p['p99']:.2f}")
    r_dom = sum(r["d"]["ryaw_x10"] > r["veh_max"] for r in rows
                if im._fin(r["d"]["ryaw_x10"]) and im._fin(r["veh_max"]))
    L.append(f"  r_yaw > veh in {r_dom} / {s['n']} units")
    base = [r for r in rows if r["stab"] < 0]
    L.append(f"  development INST units {len(base)}, of which in [-band, 0): "
             f"{sum(r['stab'] >= -band for r in base)}")
    L.append("")
    L.append(f"FINAL SESSIONS, INST only if dMz/dbeta < -{band:.4f}:")
    names = [n for n in im.FINAL if os.path.exists(os.path.join(OUT, "final", f"{n}.pkl"))]
    import pickle
    from modules.classifier import aggregate_by_corner
    res = {"band": band, "combined": s, "sessions": sessions, "final": {}}
    tv = tc = tr = 0
    tot_v = 0
    for n in names:
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        cur = im._final_eval(fin, 0.0)
        new = im._final_eval(fin, -band)
        tot_v += len(cur["verdicts"])
        assert cur["counted"] == fin["pipeline_counted"], n
        ci = [c for c in cur["counted"] if c[1] == "INST"]
        ni = [c for c in new["counted"] if c[1] == "INST"]
        dv = {c: (cur["verdicts"][c], new["verdicts"][c]) for c in cur["verdicts"]
              if cur["verdicts"][c] != new["verdicts"][c]}
        dc = sorted(set(cur["counted"]) ^ set(new["counted"]))
        rc = cur["rec"] != new["rec"]
        tv += len(dv)
        tc += len(dc)
        tr += rc
        L.append(f"  {n} (outing {fin['outing']}): INST units {cur['inst_units']} -> {new['inst_units']}; counted "
                 f"INST {len(ci)} -> {len(ni)} {[(c[0], c[2]) for c in ni]}")
        for c, (x, y) in sorted(dv.items(), key=lambda kv: str(kv[0])):
            L.append(f"    verdict C{c}: {x}  ->  {y}")
        for c in dc:
            L.append(f"    counted {'only before' if c in cur['counted'] else 'only after'}: C{c[0]} {c[1]} {c[2]} "
                     f"({c[3]} laps)")
        L.append(f"    recommendation: {cur['rec']}" + (f"  ->  {new['rec']}" if rc else " (unchanged)"))
        res["final"][n] = {"current": cur, "band": new}
    L.append(f"  totals: verdicts changed {tv} of {tot_v}; counted symptoms changed {tc}; recommendations changed {tr}")
    L.append("")
    L.append("NAMED CASES (aggregate dMz/dbeta, N m/deg; survives = counted INST after the band):")
    for n, cid in (("PRC5", 7), ("NBR1", 8), ("NBR1", 11), ("NBR5", 2), ("NBR5", 9), ("PRC2", 11)):
        fin = pickle.load(open(os.path.join(OUT, "final", f"{n}.pkl"), "rb"))
        a = aggregate_by_corner(fin["summaries"])[cid]
        neg = {ph: round(v[im.STAB]["median"], 1) for ph, v in a["phases"].items()
               if im._fin(v[im.STAB]["median"]) and v[im.STAB]["median"] < 0}
        after = [c[2] for c in res["final"][n]["band"]["counted"] if c[0] == cid and c[1] == "INST"]
        before = [c[2] for c in res["final"][n]["current"]["counted"] if c[0] == cid and c[1] == "INST"]
        L.append(f"  {n} C{cid}: {neg}; counted INST before {before}, after {after} -> "
                 f"{'survives' if after == before else ('partly' if after else 'removed')}")
    text = "\n".join(L)
    with open(os.path.join(OUT, "inst2_band.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "inst2_band.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, default=str)
    print(text)


if __name__ == "__main__":
    main()
