# fig:verdict_repeatability -- one track, rows = corners, columns =
# sessions, cell = the classifier's corner verdict as a code with a grey
# level (black-and-white readable). stable_corner_id is per session (a
# session may detect one corner more and shift every ID), so corners are
# matched by lap distance first (common.match_corners: brackets overlapping
# by at least half the shorter one, one to one, greedy by overlap) to the
# reference session (the first in label order); a corner without a match
# gets its own row. Codes: U understeer, O oversteer, I open-loop unstable
# (combined, e.g. UI), * = MARGINAL on the axle verdict, - ok, n no CS data.
# Grey: strong dark, moderate medium, ok white, no data light. Stored,
# identity-valid payloads only.
# Run: python -m tools.thesis_figures.fig_verdict_repeatability --outings 11,12,15,16 [--tag r2]
#      [--allow-status] [--allow-excluded] [--out DIR]

import argparse

from tools.thesis_figures import common
from tools.thesis_figures.fig_k_sessions import collect

SCRIPT = "fig_verdict_repeatability"
GREY = {"strong": "black!45", "moderate": "black!20", "normal": "white", "no_data": "black!6"}


def build(outings, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.corner_aggregation import aggregate_by_corner
    rows = collect(outings, allow_status, allow_excluded)
    rows.sort(key=lambda r: (r[1]["row"]["race_weekend_id"], r[1]["row"]["number"]))
    tracks = {r[1]["parsed"]["metadata"].get("TrackName") for r in rows}
    if len(tracks) != 1:
        raise common.Refused(f"one track only; the selection spans {sorted(map(str, tracks))}")
    common.write_label_map(tag, outings)
    sessions = []
    for o, session, _man, marks in rows:
        payload, _why = common.stored_payload(session)
        sessions.append({"outing": o, "label": common.session_label(session["row"]), "marks": marks,
                         "agg": aggregate_by_corner(payload["summaries"]),
                         "brackets": common.corner_brackets(payload["summaries"])})
    ref = sessions[0]
    row_of = {(ref["outing"], cid): i for i, cid in enumerate(sorted(ref["brackets"], key=lambda c: ref["brackets"][c][0]))}
    row_label = {i: f"C{cid}" for (o, cid), i in row_of.items()}
    row_s = {i: ref["brackets"][cid][0] for (o, cid), i in row_of.items()}
    audit = {k: [] for k in ("session_label", "session_corner_id", "matched_reference_corner", "overlap")}
    for sess in sessions[1:]:
        match = common.match_corners(ref["brackets"], sess["brackets"])
        for cid in sorted(sess["brackets"], key=lambda c: sess["brackets"][c][0]):
            if cid in match:
                rc, ov = match[cid]
                row_of[(sess["outing"], cid)] = row_of[(ref["outing"], rc)]
            else:
                rc, ov = "-", 0.0
                i = len(row_label)
                row_of[(sess["outing"], cid)] = i
                row_label[i] = f"{sess['label']} C{cid}"
                row_s[i] = sess["brackets"][cid][0]
            for k, v in zip(audit, (sess["label"], cid, rc, ov)):
                audit[k].append(v)
    # rows in lap-distance order, reference corners and unmatched ones alike
    order = {i: n for n, i in enumerate(sorted(row_label, key=lambda i: row_s[i]), start=1)}
    cells = {k: [] for k in ("row", "col", "row_label", "col_label", "code", "severity", "session_corner_id",
                             "s_start_m", "s_end_m")}
    for col, sess in enumerate(sessions, start=1):
        for cid, agg in sess["agg"].items():
            _text, code, state = common.verdict(agg)
            i = row_of[(sess["outing"], cid)]
            b = sess["brackets"][cid]
            for k, v in zip(cells, (order[i], col, row_label[i], sess["label"], code, state, cid, b[0], b[1])):
                cells[k].append(v)
    data_dir, plots_dir, prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, outings, tag, [f"{s['label']}: {k}" for s in sessions for k in s["marks"]],
                    reference=ref["label"], matching=f"bracket overlap >= {common.MATCH_MIN_OVERLAP} of the shorter, "
                                                     "one to one, greedy by overlap",
                    codes="U understeer, O oversteer, I open-loop unstable, * MARGINAL, - ok, n no CS data")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cells, m)
    common.write_csv(f"{data_dir}/{SCRIPT}_matching.csv", audit, m)
    # one table per grey level: pgfplots prints a symbolic point meta (the
    # code) as the cell text, so the grey has to come from the series
    present = []
    for sev in GREY:
        idx = [j for j, v in enumerate(cells["severity"]) if v == sev]
        if idx:
            present.append(sev)
            common.write_csv(f"{data_dir}/{SCRIPT}_{sev}.csv",
                             {k: [cells[k][j] for j in idx] for k in ("col", "row", "code")}, m)
    labels_y = [row_label[i] for i in sorted(row_label, key=lambda i: order[i])]
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(prefix, [s["label"] for s in sessions], labels_y, present))
    return {"rows": len(labels_y), "unmatched": [l for l in labels_y if " C" in l]}


LEGEND = {"strong": "strong", "moderate": "moderate", "normal": "ok", "no_data": "no data"}
MACRO = {"strong": "A", "moderate": "B", "normal": "C", "no_data": "D"}   # TeX names take letters only


def _tex(prefix, col_labels, row_labels, present):
    nx, ny = len(col_labels), len(row_labels)
    t = ["% generated by tools/thesis_figures/fig_verdict_repeatability.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    for sev in present:
        t.append(common.table_read(prefix, f"{SCRIPT}_{sev}.csv", f"tblRep{MACRO[sev]}"))
    t.append("\\begin{axis}[x=1.5cm, y=0.48cm, xmin=0.5, xmax=" + f"{nx + 0.5}, ymin=0.5, ymax={ny + 0.5},\n"
             "  y dir=reverse, axis line style={draw=none}, tick style={draw=none},\n"
             f"  xtick={{1,...,{nx}}}, xticklabels={{{','.join(common.tex_escape(c) for c in col_labels)}}},\n"
             "  xticklabel pos=top,\n"
             f"  ytick={{1,...,{ny}}}, yticklabels={{{','.join(common.tex_escape(r) for r in row_labels)}}},\n"
             "  legend style={font=\\small, at={(0.5,-0.04)}, anchor=north, legend columns=4, draw=none}]\n")
    for sev in present:
        t.append(f"\\addplot[only marks, mark=square*, mark size=0.21cm, mark options={{fill={GREY[sev]}, draw=black!60}},\n"
                 "  point meta=explicit symbolic, nodes near coords,\n"
                 "  every node near coord/.append style={anchor=center, font=\\small, text=black}]\n"
                 f"  table[x=col, y=row, meta=code]{{\\tblRep{MACRO[sev]}}};\n"
                 f"\\addlegendentry{{{LEGEND[sev]}}}\n")
    t.append("\\end{axis}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", required=True, help="outings of one track")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    print(build([int(x) for x in a.outings.split(",")], a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
