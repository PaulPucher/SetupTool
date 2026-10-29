# tab:observer_health -- per session: NIS gate health score and verdict
# (stored gate_verdict), observer resets (hard-bound re-entries: not stored,
# so counted as the rising edges of the shipped observer's reset mask in a
# read-only re-run of the chain), floor events (stored events list) and the
# estimation path (stored fit record); from the same re-run, the median
# |beta| of the observer on the steering-defined straights (the fit-L noise
# straights, modules/straights.py), and per axle the share of corner
# instances on the analysis laps whose observer slip angle leaves the
# slip-angle range of the session's fit samples (p0.5-p99.5 of the fit
# population, common.fit_sample_range; partial fit re-run) on either side.
# Stale stored payloads refuse the table and are named.
# Run: python -m tools.thesis_figures.tab_observer_health --outings 11,12,13,14,15,16 [--tag r2]
#      [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common
from tools.thesis_figures.fig_example_corner import corner_traces
from tools.thesis_figures.fig_k_sessions import collect

SCRIPT = "tab_observer_health"
COLUMNS = ("label", "health_score", "gate_verdict", "n_observer_resets", "n_floor_events", "estimation_path", "fit_status",
           "median_abs_beta_straights_deg", "n_instances", "beyond_fit_range_f", "beyond_fit_range_r")


def beyond_fit_range(session, res, fit_range):
    """(n instances, share front, share rear): an instance counts on an axle
    when its observer slip angle, over the samples the corner figure plots
    (kerb, reset and standstill excluded), leaves [p0.5, p99.5]."""
    n, over = 0, {"f": 0, "r": 0}
    for s in res["summaries"]:
        *_x, amax = corner_traces(session, res, s["lap_number"], s.get("stable_corner_id"))
        if not all(np.isfinite(amax[f"{a}_lo"]) for a in ("f", "r")):
            continue
        n += 1
        for a in ("f", "r"):
            over[a] += amax[f"{a}_lo"] < fit_range[a][0] or amax[f"{a}_hi"] > fit_range[a][1]
    return n, (over["f"] / n if n else float("nan")), (over["r"] / n if n else float("nan"))


def median_abs_beta_straights(session, res):
    from modules.straights import steering_straights
    mask = steering_straights(res["state"], session["parsed"]["laps"], session["params"]["tyre_fit_auto"])["mask"]
    b = np.degrees(np.abs(np.asarray(res["beta"])[mask]))
    b = b[np.isfinite(b)]
    return float(np.median(b)) if b.size else float("nan")


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
        n_inst, share_f, share_r = beyond_fit_range(session, res, common.fit_sample_range(session))
        for k, v in (("label", common.session_label(session["row"])),
                     ("health_score", float(gate["health_score"]) if gate.get("health_score") is not None
                      else float("nan")),
                     ("gate_verdict", gate.get("verdict") or "-"),
                     ("n_observer_resets", resets if resets is not None else "-"),
                     ("n_floor_events", len((payload.get("events") or {}).get("events") or [])),
                     ("estimation_path", (path + (f" ({reason})" if reason else "")).replace(",", ";")),
                     ("fit_status", (status + (f" ({detail})" if detail else "")).replace(",", ";")),
                     ("median_abs_beta_straights_deg", median_abs_beta_straights(session, res)),
                     ("n_instances", n_inst), ("beyond_fit_range_f", share_f), ("beyond_fit_range_r", share_r)):
            cols[k].append(v)
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, outings, tag,
                                 sources="health score and verdict: stored gate_verdict; resets: rising edges of the "
                                         "observer reset mask, read-only re-run; floor events: stored events; path: "
                                         "stored fit record; |beta| on straights and slip range: same re-run, "
                                         "fit range p0.5-p99.5 of the fit population (partial fit re-run)"))
    render(out)
    return cols


# fixed widths, right-aligned: the headers wrap inside their column
_NUM_COLS = (("Health", "1.3cm"), ("Resets", "1.3cm"), ("Floor events", "1.4cm"),
             ("Median $|\\beta|$ on straights in deg", "2.6cm"), ("Beyond fit range f\\,/\\,r in \\%", "2.6cm"))


def render(out=None):
    """The .tex from the stored CSV only (no session is loaded). Gate (all
    pass), estimation path and tire fit (both in tab:sessions) stay in the
    CSV only."""
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    _meta, c = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    body = []
    for i, label in enumerate(c["label"]):
        h, b = common.num(c["health_score"][i]), common.num(c["median_abs_beta_straights_deg"][i])
        f, r = common.num(c["beyond_fit_range_f"][i]), common.num(c["beyond_fit_range_r"][i])
        body.append([common.tex_escape(label), f"{h:.2f}" if h == h else "--", c["n_observer_resets"][i],
                     c["n_floor_events"][i], f"{b:.2f}" if b == b else "--",
                     f"{100 * f:.0f} / {100 * r:.0f}" if f == f and r == r else "--"])
    colspec = "l " + " ".join(f">{{\\raggedleft\\arraybackslash}}p{{{w}}}" for _h, w in _NUM_COLS)
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, ["Session"] + [h for h, _w in _NUM_COLS], body,
                       colspec, env="tabular", small=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSV only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if not a.outings:
        ap.error("--outings is required unless --render-only")
    print(build([int(x) for x in a.outings.split(",")], a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
