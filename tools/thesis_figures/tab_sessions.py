# tab:sessions -- one row per session: neutral label, track, session type,
# driver code, analysis / total laps, corners, estimation path with its
# reason, health score. No dates, no event names, no driver names (the
# label map stays local). Stored, identity-valid payloads; a session the
# app cannot analyse at all gets its row from a read-only chain attempt
# (path "not analysable" and the missing required channels). A stale
# payload refuses the table and names the outing.
# Run: python -m tools.thesis_figures.tab_sessions --outings 9,10,11,12,13,14,15,16 [--tag r2] [--out DIR]

import argparse

from tools.thesis_figures import common

SCRIPT = "tab_sessions"
COLUMNS = ("label", "track", "session_type", "driver_code", "analysis_laps", "total_laps", "n_corners",
           "estimation_path", "path_reason", "fit_status", "health_score")


def _not_analysable(session):
    # the chain's own entry check: prepare_vehicle_state's required channels
    from modules.stability_analysis import prepare_vehicle_state
    if prepare_vehicle_state(session["parsed"]["channels"], session["params"]) is not None:
        return None
    req = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_x"]
    ch = session["parsed"]["channels"]
    missing = [c for c in req if ch.get(c) is None or ch[c].get("quality") in ("missing", "failed")
               or ch[c].get("time") is None]
    return "required channel absent: " + ", ".join(missing)


def collect_rows(outings):
    codes = common.driver_codes()
    rows, problems = [], []
    for o in outings:
        try:
            session = common.load_session(o, allow_no_grid=True)
        except common.Refused as e:
            problems.append(str(e))
            continue
        row, parsed = session["row"], session["parsed"]
        base = {"label": common.session_label(row), "track": parsed["metadata"].get("TrackName") or "-",
                "session_type": row["session_type"] or "-", "driver_code": codes.get(row["driver_id"], "-"),
                "total_laps": len(parsed["laps"])}
        # no grid rate -> the app refuses the file before any estimator runs
        grid_error = session["ident"]["grid_error"]
        payload, why = (None, None) if grid_error else common.stored_payload(session)
        if payload is None:
            # ";" for the message's commas: the CSV writer does not quote
            na = (("no common grid rate: " + grid_error.split(" -- ")[0]).replace(",", ";") if grid_error
                  else _not_analysable(session))
            if na is None:
                problems.append(f"outing {o}: {why}")
                continue
            base.update(analysis_laps=len(session["ident"]["lap_filter"]),
                        n_corners=len({c.get("stable_corner_id") for c in parsed.get("corners", [])}),
                        estimation_path="not analysable", path_reason=na, fit_status="-",
                        health_score=float("nan"))
        else:
            path, reason = common.estimation_path(payload["fit_manifest"], payload["fallback_used"],
                                                  payload["fallback_reason"])
            status, detail = common.fit_status(payload["fit_manifest"])
            gate = payload.get("gate_verdict") or {}
            base.update(analysis_laps=len(payload["lap_filter"]),
                        n_corners=len({s["stable_corner_id"] for s in payload["summaries"]}),
                        estimation_path=path, path_reason=reason or "-",
                        # ";" inside the field: the CSV writer does not quote
                        fit_status=status + (f" ({detail.replace(',', ';')})" if detail else ""),
                        health_score=float(gate["health_score"]) if gate.get("health_score") is not None
                        else float("nan"))
        rows.append((o, base))
    if problems:
        raise common.Refused("refused:\n  " + "\n  ".join(problems))
    return rows


def build(outings, tag="dev", out=None):
    rows = collect_rows(outings)
    common.write_label_map(tag, outings)
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    cols = {k: [b[k] for _o, b in rows] for k in COLUMNS}
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, outings, tag, health_score="NIS gate health score (fraction of windows "
                                 "in band); empty without an observer"))
    header = ["Session", "Track", "Type", "Driver", "Laps", "Corners", "Estimation", "Tyre fit", "Health"]
    body = []
    for _o, b in rows:
        path = b["estimation_path"] + (f" ({b['path_reason']})" if b["path_reason"] != "-" else "")
        health = f"{b['health_score']:.2f}" if b["health_score"] == b["health_score"] else "--"
        body.append([common.tex_escape(x) for x in (b["label"], b["track"], b["session_type"], b["driver_code"])]
                    + [f"{b['analysis_laps']}/{b['total_laps']}", str(b["n_corners"]), common.tex_escape(path),
                       common.tex_escape(b["fit_status"].replace(";", ",")), health])
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, header, body, "l l l l c c X X c")
    return cols


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", required=True, help="comma-separated app outing IDs, in table order")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    build([int(x) for x in a.outings.split(",")], a.tag, a.out)


if __name__ == "__main__":
    main()
