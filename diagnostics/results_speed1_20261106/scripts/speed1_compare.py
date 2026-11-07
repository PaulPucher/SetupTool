# ORDER SPEED-1 Part 3: decomposition summary from speed1_runner outputs.
# Run: python speed1_compare.py <runs dir> > part3.txt
import json
import os
import sys

RUNS = ("off", "r1", "r1r2", "all")
NAMES = {"off": "(0) all off", "r1": "(1) R1", "r1r2": "(2) R1+R2", "all": "(3) R1+R2+R3"}
KEYS = ("NBR1", "NBR2", "NBR3", "NBR4", "NBR5", "PRC1", "PRC2", "PRC3", "PRC4", "PRC5")


def load(d):
    out = {}
    for r in RUNS:
        out[r] = {}
        for k in KEYS:
            p = os.path.join(d, r, f"{k}.json")
            if os.path.exists(p):
                out[r][k] = json.load(open(p, encoding="utf-8"))
    return out


def verdicts(s):
    return {str(c["corner"]): c["verdict"] for c in s["corner_verdicts"]}


def counted(s):
    return {(str(c["corner"]), c["symptom"], c["phase"]): c["laps"] for c in s["counted_symptoms"]}


def vdiff(a, b):
    va, vb = verdicts(a), verdicts(b)
    return [(k, va.get(k), vb.get(k)) for k in sorted(set(va) | set(vb), key=lambda x: (len(x), x))
            if va.get(k) != vb.get(k)]


def cdiff(a, b):
    ca, cb = counted(a), counted(b)
    return [(k, ca.get(k), cb.get(k)) for k in sorted(set(ca) | set(cb)) if ca.get(k) != cb.get(k)]


def rec(s):
    r = s.get("session_recommendation_full")
    if not r:
        return "none"
    if "tie" in r:
        return "tie (" + " / ".join(f"{'+'.join(l)} {d}" for l, d in r["tie"]) + f"), net {r['net']}"
    return f"{'+'.join(r['levers'])} {r['direction']}, net {r['net']:.2f}"


def fmt(x, n=3):
    return "-" if x is None else f"{x:.{n}f}"


def main():
    runs = load(sys.argv[1])
    prev = None
    for r in RUNS:
        S = runs[r]
        print(f"=== {NAMES[r]}  ({len(S)} sessions)")
        nv = sum(len(verdicts(s)) for s in S.values())
        fb = [k for k, s in S.items() if s["fallback_used"] or s["fit_status"] == "degenerate"]
        print(f"POOLED: verdicts {nv}; kinematic fallback / degenerate: {fb or 'none'}")
        if r != "off":
            d0 = sum(len(vdiff(runs['off'][k], S[k])) for k in S)
            dp = sum(len(vdiff(runs[prev][k], S[k])) for k in S)
            c0 = sum(len(cdiff(runs['off'][k], S[k])) for k in S)
            cp = sum(len(cdiff(runs[prev][k], S[k])) for k in S)
            rc0 = [k for k in S if rec(runs['off'][k]) != rec(S[k])]
            print(f"  changed verdicts vs (0): {d0} of {nv}; vs {NAMES[prev]}: {dp}")
            print(f"  changed counted symptoms vs (0): {c0}; vs {NAMES[prev]}: {cp}")
            print(f"  sessions whose recommendation differs from (0): {rc0 or 'none'}")
        else:
            ids = [k for k, s in S.items() if not (s.get('stored_identity') or {}).get('verdicts_equal')
                   or not (s.get('stored_identity') or {}).get('recommendation_equal')
                   or not (s.get('stored_identity') or {}).get('counted_equal')]
            print(f"  stored-payload identity (verdicts, recommendation, counted symptoms): "
                  f"{'all ten equal' if not ids else 'DIFFERS ' + str(ids)}")
        for k in KEYS:
            if k not in S:
                print(f"- {k}: missing")
                continue
            s = S[k]
            ax = s["axles"]
            line = (f"- {k}: fit {s['fit_status']}, gate {s['gate_verdict']} health {fmt(s['health_score'])}, "
                    f"fallback {s['fallback_used']}; K f/r {fmt(ax['front']['K_per_rad'], 2)} / "
                    f"{fmt(ax['rear']['K_per_rad'], 2)}, mu f/r {fmt(ax['front']['mu'])} / {fmt(ax['rear']['mu'])}; "
                    f"braking excl f/r {fmt(s['braking_excluded']['front']['share'])} / "
                    f"{fmt(s['braking_excluded']['rear']['share'])}")
            print(line)
            sc = s.get("signal_corrections") or {}
            if "speed_reference" in sc:
                q = sc["speed_reference"]
                print(f"    R1: {q['status']}, flagged {q.get('flagged_samples')} samples "
                      f"({q.get('flagged_events')} events), replaced {q.get('replaced_samples')}, "
                      f"max excess {fmt(q.get('max_excess_mps'), 1)} m/s")
            if "engine_braking" in sc:
                q = sc["engine_braking"]
                print(f"    R2: {q['status']}, share of moving samples {fmt(q.get('share_moving'))}, "
                      f"of the fit population {fmt(q.get('share_fit_population'))}")
            if "sensor_offsets" in sc:
                q = sc["sensor_offsets"]
                parts = [f"{n} {fmt(c.get('offset'), 4)} {c.get('unit', '')}" if c.get("applied") else
                         f"{n} not applied ({c.get('reason')})" for n, c in q["channels"].items()]
                print(f"    R3: {q['status']}, straights {fmt(q.get('straight_s'), 1)} s; " + "; ".join(parts))
            print(f"    corners parsed {s['n_corners_parsed']}, verdicts {len(verdicts(s))}; "
                  f"recommendation: {rec(s)}")
            if r != "off":
                for lab, base in (("vs (0)", runs["off"][k]), (f"vs {NAMES[prev]}", runs[prev][k])):
                    dv = vdiff(base, s)
                    if dv:
                        print(f"    verdicts {lab}: " + "; ".join(f"C{c} '{a}' -> '{b}'" for c, a, b in dv))
                    dc = cdiff(base, s)
                    if dc:
                        print(f"    counted {lab}: " + "; ".join(f"C{c[0]} {c[1]} {c[2]} laps {a} -> {b}"
                                                             for c, a, b in dc))
                    if rec(base) != rec(s):
                        print(f"    recommendation {lab}: {rec(base)} -> {rec(s)}")
            if r == "all":
                a = s["ay_vs_vr"]
                print(f"    a_y / (v r): median {a['median']:.3f}, p10 {a['p10']:.3f}, p90 {a['p90']:.3f} "
                      f"(n {a['n']})")
        prev = r
        print()


if __name__ == "__main__":
    main()
