# tab:sessions -- one row per session: neutral label, track, session type,
# driver code, analysis / total laps, corners, estimation path with its
# reason, health score, damper-load path, membership in the 14-session
# threshold pool, overlap with the development data and the evaluation
# status. No dates, no event names, no driver names (the label map stays
# local). Pool and overlap come from a local, git-ignored JSON (--overlap:
# outing ID -> {"in_sample", "overlap"}), the result of a content-hash
# check outside this script. A session that is not in the app at all is a
# fixed row (--excluded-row). Stored, identity-valid payloads; a session the
# app cannot analyse at all gets its row from a read-only chain attempt
# (path "not analysable" and the missing required channels). A stale
# payload refuses the table and names the outing.
# Run: python -m tools.thesis_figures.tab_sessions --outings 9,10,11,12,13,14,15,16 [--tag r2] [--out DIR]
#      [--overlap tools/thesis_figures/local/overlap.json] [--excluded-row "LABEL;TRACK;REASON"]

import argparse
import json

from tools.thesis_figures import common

SCRIPT = "tab_sessions"
COLUMNS = ("label", "track", "session_type", "driver_code", "analysis_laps", "total_laps", "n_corners",
           "estimation_path", "path_reason", "fit_status", "health_score", "damper_path", "in_sample",
           "dev_overlap", "status")


def damper_path(fit_manifest):
    """Measured (damper loads) / measured with event zero entry / static
    split, with the gate reporting of the event-entry ruling: on a source
    session of the entry the 0.8 gate is "not applicable" and the
    offset-free validation decides."""
    al = (fit_manifest or {}).get("axle_load") or {}
    plaus = al.get("damper_plausibility") or {}
    if plaus and not plaus.get("passed", True):
        return "static split (damper loads rejected)"
    if al.get("vertical_load_source_used") != "measured":
        return al.get("vertical_load_source_used") or "-"
    path = "measured with event zero entry" if "event zero entry" in (al.get("level_note") or "") else "measured"
    if str(plaus.get("reporting", "")).startswith("not applicable"):
        val = plaus.get("validation") or {}
        path += ("; gate not applicable; offset-free validation "
                 + ("passed" if val.get("passed") else "failed" if val else "missing"))
    return path


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


def collect_rows(outings, overlap=None):
    codes = common.driver_codes()
    overlap = overlap or {}
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
                "total_laps": len(parsed["laps"]),
                "in_sample": (overlap.get(str(o)) or {}).get("in_sample", "-"),
                "dev_overlap": (overlap.get(str(o)) or {}).get("overlap", "-")}
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
                        health_score=float("nan"), damper_path="-", status="excluded: " + na)
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
                        else float("nan"), damper_path=damper_path(payload["fit_manifest"]).replace(",", ";"))
            # exclusion rules (ruling 2026-10-27): rejected damper path
            # without a valid zero entry, kinematic fallback
            excl = [r for r, bad in (("damper loads rejected", base["damper_path"].startswith("static")),
                                     ("observer fallback", path == "kinematic fallback")) if bad]
            base["status"] = "excluded: " + "; ".join(excl) if excl else "evaluated"
        rows.append((o, base))
    if problems:
        raise common.Refused("refused:\n  " + "\n  ".join(problems))
    return rows


def build(outings, tag="dev", out=None, overlap=None, excluded_rows=()):
    rows = collect_rows(outings, overlap)
    for spec in excluded_rows:
        label, track, reason = (x.strip() for x in spec.split(";", 2))
        rows.append((None, {"label": label, "track": track, "session_type": "-", "driver_code": "-",
                            "analysis_laps": "-", "total_laps": "-", "n_corners": "-", "estimation_path": "-",
                            "path_reason": "-", "fit_status": "-", "health_score": float("nan"),
                            "damper_path": "-", "in_sample": "no", "dev_overlap": "-",
                            "status": reason.replace(",", ";")}))
    common.write_label_map(tag, outings)
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    cols = {k: [b[k] for _o, b in rows] for k in COLUMNS}
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, outings, tag, health_score="NIS gate health score (fraction of windows "
                                 "in band); empty without an observer"))
    render(out)
    return cols


def _zero_entry(damper):
    # yes* = offset correction checked by the offset-free validation, the
    # weight gate not applicable (event-entry reporting rule)
    if not damper.startswith("measured"):
        return "--"
    if "gate not applicable" in damper:
        return "yes*"
    return "yes" if "event zero entry" in damper else "no"


_OVERLAP_WORDS = {"same file as dev or golden outing": "same file", "same session, different export": "same session"}


def render(out=None):
    """The .tex from the stored CSV only (no session is loaded). Excluded
    rows stay in the CSV; the table lists the evaluated sessions (the
    exclusion is stated once in the text, author ruling 2026-10-27).
    Track (in the label), estimation path (all observer) and status (all
    evaluated) are in the CSV only."""
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    _meta, c = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    body = []
    for i, label in enumerate(c["label"]):
        if c["status"][i].startswith("excluded"):
            continue
        health = common.num(c["health_score"][i])
        body.append([common.tex_escape(label), common.tex_escape(c["driver_code"][i]),
                     f"{c['analysis_laps'][i]}/{c['total_laps'][i]}", c["n_corners"][i],
                     f"{health:.2f}" if health == health else "--",
                     common.tex_escape(c["fit_status"][i].split(" ")[0]),
                     _zero_entry(c["damper_path"][i]), common.tex_escape(c["in_sample"][i]),
                     common.tex_escape(_OVERLAP_WORDS.get(c["dev_overlap"][i], c["dev_overlap"][i]))])
    header = ["Session", "Driver", "Laps", "Corners", "Health", "Fit", "Zero entry", "Pool", "Overlap"]
    note = ("\\parbox{\\linewidth}{\\small $^*$\\,Offset correction checked by the offset-free validation; "
            "the plausibility gate against the static weight is not applicable.}")
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, header, body, "l l r r r l l l l",
                       env="tabular", small=True, footnote=note)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", help="comma-separated app outing IDs, in table order")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--out")
    ap.add_argument("--overlap", help="local JSON: outing ID -> {in_sample, overlap}")
    ap.add_argument("--excluded-row", action="append", default=[],
                    help="'LABEL;TRACK;REASON' for a session that is not in the app")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSV only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if not a.outings:
        ap.error("--outings is required unless --render-only")
    overlap = None
    if a.overlap:
        with open(a.overlap, "r", encoding="utf-8") as f:
            overlap = json.load(f)
    build([int(x) for x in a.outings.split(",")], a.tag, a.out, overlap, a.excluded_row)


if __name__ == "__main__":
    main()
