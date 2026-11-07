import json
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
from speed1_compare import KEYS, cdiff, rec, vdiff, verdicts

R = {r: {k: json.load(open(f"runs/{r}/{k}.json")) for k in KEYS} for r in ("off", "r1r2", "all")}
n = 0
bad = []
for k in KEYS:
    i = R["off"][k]["stored_identity"]
    n += i["n_verdicts"]
    if not (i["verdicts_equal"] and i["recommendation_equal"] and i["counted_equal"]):
        bad.append(k)
print(f"PART 2 off vs stored payloads: {n} verdicts, differing sessions: {bad or 'none'}")
S, B = R["all"], R["r1r2"]
print("fallback/degenerate run (3):", [k for k in KEYS if S[k]["fallback_used"] or S[k]["fit_status"] == "degenerate"] or "none")
print("verdicts total (2)", sum(len(verdicts(B[k])) for k in KEYS), "(3)", sum(len(verdicts(S[k])) for k in KEYS))
for base in ("r1r2", "off"):
    print(f"(3) vs {base}: verdicts {sum(len(vdiff(R[base][k], S[k])) for k in KEYS)}, counted "
          f"{sum(len(cdiff(R[base][k], S[k])) for k in KEYS)}, recommendation changed {[k for k in KEYS if rec(R[base][k]) != rec(S[k])]}")
print("(2) vs off: verdicts", sum(len(vdiff(R['off'][k], B[k])) for k in KEYS))


def rng(v):
    return f"{min(v):+.1f} to {max(v):+.1f}"


for ax in ("front", "rear"):
    print(ax, "K (3) vs (2)", rng([100 * (S[k]["axles"][ax]["K_per_rad"] / B[k]["axles"][ax]["K_per_rad"] - 1) for k in KEYS]),
          "%, mu", rng([100 * (S[k]["axles"][ax]["mu"] / B[k]["axles"][ax]["mu"] - 1) for k in KEYS]), "%")
print("health (3)-(2) x1e-3", rng([1000 * (S[k]["health_score"] - B[k]["health_score"]) for k in KEYS]))
for k in KEYS:
    s = S[k]
    o = s["signal_corrections"]["sensor_offsets"]
    offs = "; ".join(f"{nm} {c['offset_physical']:+.3f} {c['unit_physical']} ({c['source']}, n {c['n']})" if c.get("applied")
                     else f"{nm} not corrected ({c.get('reason')})" for nm, c in o["channels"].items())
    print(f"- {k}: corners (0) {R['off'][k]['n_corners_parsed']} / (2) {B[k]['n_corners_parsed']} / (3) {s['n_corners_parsed']}; "
          f"stable verdicts {len(verdicts(s))}; standstill {o['standstill_s']:.1f} s; {offs}")
    print(f"    rec (2): {rec(B[k])} | (3): {rec(s)}; verdict changes vs (2): {len(vdiff(B[k], s))}; "
          f"health {s['health_score']:.3f}; K f/r {s['axles']['front']['K_per_rad']:.2f}/{s['axles']['rear']['K_per_rad']:.2f}; "
          f"a_y/(v r) {s['ay_vs_vr']['median']:.3f}")
    for c, a, b in vdiff(B[k], s):
        print(f"      C{c}: '{a}' -> '{b}'")
    for c, a, b in cdiff(B[k], s):
        print(f"      counted C{c[0]} {c[1]} {c[2]}: {a} -> {b}")
