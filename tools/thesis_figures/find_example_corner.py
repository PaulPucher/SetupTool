# Candidates for fig:example_corner:
# corner instances on the analysis laps of the given outings that pass
#   1. kerb_fraction 0 in every phase of that lap,
#   2. a CS value on both axles in turn in, at the apex and in at least one
#      exit phase,
#   3. braking, turn in, early exit and exit completion at least one 2 m bin
#      long (the apex is one instant by definition),
#   4. observer slip angles inside the fitted range: per axle the largest
#      observer |alpha| over the plotted, non-excluded samples <= the stored
#      axles.visited_alpha_p99_deg of the session (p99 of |alpha| in the fit
#      population, kinematic slip angle -- the fit's own input),
# ranked: a US or INST symptom on that lap that is also counted for the
# corner first, then fewest empty CS bins, then fewest empty bins of any
# trace -- the bins the figure draws. Lists only; the author picks. Also
# reports, over ALL corner instances, how many exceed the fitted range.
# Needs an identity-valid stored
# payload (for the fitted range) and a read-only re-run (for the traces).
# Run: python -m tools.thesis_figures.find_example_corner --outings 11,12,13,14,15,16 [--top 3]

import argparse

import numpy as np

from tools.thesis_figures import common
from tools.thesis_figures.fig_example_corner import corner_traces

LENGTH_PHASES = ("entry_1_brake", "entry_2_turnin", "exit_4", "exit_5")
FUNNEL = ("all", "1 kerb-free", "2 CS present", "3 phase lengths", "4 inside fitted range")


def candidates(outing):
    from modules.classifier import phase_symptoms, verdict_cs_stat
    from modules.setup_engine import counted_symptoms
    from modules.stability_analysis import load_parameters
    session = common.load_session(outing)
    payload, why = common.stored_payload(session)
    if payload is None:
        raise common.Refused(f"outing {outing}: {why}")
    # gate first: a degenerate fit may carry no axles to read the range from
    common.gate(outing, payload["fit_manifest"], payload["fallback_used"], False, False)
    fit_range = {ax: payload["fit_manifest"]["axles"][name]["visited_alpha_p99_deg"]
                 for ax, name in (("f", "front"), ("r", "rear"))}
    res = common.run_chain(session)
    common.gate(outing, res["fit_manifest"], res["fallback_used"], False, False)
    counted = counted_symptoms(res["summaries"], load_parameters()["setup_engine"])
    funnel = dict.fromkeys(FUNNEL, 0)
    exceed = {"n": 0, "f": 0, "r": 0, "either": 0}
    out = []
    for s in res["summaries"]:
        lap, cid = s["lap_number"], s.get("stable_corner_id")
        funnel["all"] += 1
        cols, phases, _lo, _hi, _summ, amax = corner_traces(session, res, lap, cid)
        # exceedance over every instance, independent of the other criteria
        if all(np.isfinite(amax[a]) for a in ("f", "r")):
            over = {a: amax[a] > fit_range[a] for a in ("f", "r")}
            exceed["n"] += 1
            exceed["f"] += over["f"]
            exceed["r"] += over["r"]
            exceed["either"] += over["f"] or over["r"]
        if any(s["phases"].get(ph) is None or s["phases"][ph]["kerb_fraction"] > 0 for ph in common.PHASE_LABELS):
            continue
        funnel["1 kerb-free"] += 1

        def cs_ok(ph):
            return all(np.isfinite(verdict_cs_stat(s, ph, ax)["median"]) for ax in ("f", "r"))
        if not (cs_ok("entry_2_turnin") and cs_ok("apex_3") and (cs_ok("exit_4") or cs_ok("exit_5"))):
            continue
        funnel["2 CS present"] += 1
        length = dict(zip(phases["phase_key"], np.subtract(phases["s_end_m"], phases["s_start_m"])))
        if any(length.get(ph, 0.0) < common.TRACE_BIN_M for ph in LENGTH_PHASES):
            continue
        funnel["3 phase lengths"] += 1
        if not all(amax[a] <= fit_range[a] for a in ("f", "r")):
            continue
        funnel["4 inside fitted range"] += 1
        lap_sym = {(x["symptom"], x["phase"]) for x in phase_symptoms(s)["symptoms"]}
        corner_sym = {(x["symptom"], x["phase"]) for x in counted.get(cid, {}).get("symptoms", [])}
        preferred = sorted(f"{sy} @ {common.PHASE_LABELS[ph]}" for sy, ph in lap_sym & corner_sym if sy in ("US", "INST"))
        n_cs = int(np.isnan(cols["cs_f"]).sum() + np.isnan(cols["cs_r"]).sum())
        n_any = int(np.isnan(np.column_stack([v for k, v in cols.items() if k != "s_m"])).any(axis=1).sum())
        out.append({"outing": outing, "label": common.session_label(session["row"]), "lap": lap, "corner": cid,
                    "speed_class": s.get("speed_class"), "symptom": ", ".join(preferred) or "none counted on this lap",
                    "preferred": bool(preferred), "n_bins": len(cols["s_m"]), "empty_cs_bins": n_cs,
                    "empty_any_bins": n_any, "max_alpha_deg": {a: round(amax[a], 2) for a in amax}})
    return out, {"label": common.session_label(session["row"]), "fit_range_p99_deg": fit_range,
                 "funnel": funnel, "exceed": exceed}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", required=True)
    ap.add_argument("--top", type=int, default=3)
    a = ap.parse_args(argv)
    allc, stats = [], {}
    for o in (int(x) for x in a.outings.split(",")):
        c, st = candidates(o)
        stats[o] = st
        print(f"outing {o} ({st['label']}): fitted range p99 f {st['fit_range_p99_deg']['f']:.2f} / "
              f"r {st['fit_range_p99_deg']['r']:.2f} deg; funnel {st['funnel']}; instances above the fitted range "
              f"{st['exceed']}", flush=True)
        allc += c
    allc.sort(key=lambda c: (not c["preferred"], c["empty_cs_bins"], c["empty_any_bins"]))
    print("top:" if allc else "no corner instance passes all four criteria")
    for c in allc[:a.top]:
        print(c)
    return allc, stats


if __name__ == "__main__":
    main()
