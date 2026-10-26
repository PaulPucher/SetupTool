# tab:observer_health -- per session: NIS gate health score and verdict
# (stored gate_verdict), observer resets (hard-bound re-entries: not stored,
# so counted as the rising edges of the shipped observer's reset mask in a
# read-only re-run of the chain), floor events (stored events list) and the
# estimation path (stored fit record). Stale stored payloads refuse the
# table and are named.
# Run: python -m tools.thesis_figures.tab_observer_health --outings 11,12,13,14,15,16 [--tag r2]
#      [--allow-status] [--allow-excluded] [--out DIR]

import argparse

from tools.thesis_figures import common
from tools.thesis_figures.fig_k_sessions import collect

SCRIPT = "tab_observer_health"
COLUMNS = ("label", "health_score", "gate_verdict", "n_observer_resets", "n_floor_events", "estimation_path", "fit_status")


def build(outings, tag="dev", allow_status=False, allow_excluded=False, out=None):
    rows = collect(outings, allow_status, allow_excluded)
    common.write_label_map(tag, outings)
    cols = {k: [] for k in COLUMNS}
    for o, session, _man, marks in rows:
        payload, _why = common.stored_payload(session)
        gate = payload.get("gate_verdict") or {}
        path, reason = common.estimation_path(payload["fit_manifest"], payload["fallback_used"],
                                              payload["fallback_reason"])
        status, detail = common.fit_status(payload["fit_manifest"])
        res = common.run_chain(session)
        resets = common.count_rising(res["reset_mask"])
        for k, v in (("label", common.session_label(session["row"])),
                     ("health_score", float(gate["health_score"]) if gate.get("health_score") is not None
                      else float("nan")),
                     ("gate_verdict", gate.get("verdict") or "-"),
                     ("n_observer_resets", resets if resets is not None else "-"),
                     ("n_floor_events", len((payload.get("events") or {}).get("events") or [])),
                     ("estimation_path", (path + (f" ({reason})" if reason else "")).replace(",", ";")),
                     ("fit_status", (status + (f" ({detail})" if detail else "")).replace(",", ";"))):
            cols[k].append(v)
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, outings, tag,
                                 sources="health score and verdict: stored gate_verdict; resets: rising edges of the "
                                         "observer reset mask, read-only re-run; floor events: stored events; path: "
                                         "stored fit record"))
    header = ["Session", "Health score", "Gate", "Observer resets", "Floor events", "Estimation", "Tyre fit"]
    body = [[common.tex_escape(cols["label"][i]),
             f"{cols['health_score'][i]:.2f}" if cols["health_score"][i] == cols["health_score"][i] else "--",
             common.tex_escape(cols["gate_verdict"][i]), str(cols["n_observer_resets"][i]),
             str(cols["n_floor_events"][i]), common.tex_escape(cols["estimation_path"][i]),
             common.tex_escape(cols["fit_status"][i].replace(";", ","))]
            for i in range(len(cols["label"]))]
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, header, body, "l c c c c X X")
    return cols


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", required=True)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    print(build([int(x) for x in a.outings.split(",")], a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
