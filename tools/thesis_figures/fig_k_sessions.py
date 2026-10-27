# fig:k_sessions -- normalised origin slope K_i = B C mu and demonstrated mu_i
# per session (front circles, rear squares) against the plausibility-gate
# bands of fit L: K band
# with its edge zones, mu ceiling with its edge zone. Stored results only:
# every outing needs an identity-valid payload of the current schema, else
# the script refuses and names the outing and the reason.
# Tick labels are the neutral '<code> <n>' (local/labels.json + DB outing
# number); the mapping goes to local/label_map_<tag>.txt only.
# Run: python -m tools.thesis_figures.fig_k_sessions --outings 11,12,15,16,13,14
#      [--separator-after 16] [--tag r2] [--allow-status] [--allow-excluded] [--out DIR]

import argparse

from tools.thesis_figures import common

SCRIPT = "fig_k_sessions"


def collect(outings, allow_status=False, allow_excluded=False):
    """Stored manifests of all outings, or Refused listing every stale or
    refused outing at once (so one run tells the author what to re-analyse)."""
    rows, problems = [], []
    for o in outings:
        try:
            session = common.load_session(o)
            payload, why = common.stored_payload(session)
            if payload is None:
                problems.append(f"outing {o}: {why}")
                continue
            man = payload["fit_manifest"] or {}
            marks = common.gate(o, man, payload["fallback_used"], allow_status, allow_excluded)
            # --allow-status lets a degenerate fit through the gate, but one
            # that stopped before the curve fit has no K or mu to plot
            missing = [k for k in ("axles", "k_band_zone", "mu_ceiling_zone") if not man.get(k)]
            if missing:
                problems.append(f"outing {o}: fit status {man.get('status')!r} has no fitted curve to plot "
                                f"(no {', '.join(missing)})")
                continue
            rows.append((o, session, man, marks))
        except common.Refused as e:
            problems.append(str(e))
    if problems:
        raise common.Refused("refused:\n  " + "\n  ".join(problems))
    return rows


def build(outings, labels=None, separator_after=None, tag="dev", allow_status=False, allow_excluded=False,
          out=None):
    # argument errors before any session is loaded or file written
    if labels is not None and len(labels) != len(outings):
        raise common.Refused(f"--labels gives {len(labels)} labels for {len(outings)} outings")
    if separator_after is not None and separator_after not in outings:
        raise common.Refused(f"--separator-after {separator_after} is not one of --outings")
    rows = collect(outings, allow_status, allow_excluded)
    lin = common.lines()
    # neutral labels '<code> <n>' (anonymisation rule); a label given on the
    # command line still passes the name guard on writing
    labels = labels or [common.session_label(s["row"]) for _o, s, _m, _k in rows]
    common.write_label_map(tag, outings)
    cols = {k: [] for k in ("x", "outing_id", "label", "K_f_per_rad", "K_r_per_rad", "mu_f", "mu_r", "k_zone_f",
                            "k_zone_r", "mu_zone_f", "mu_zone_r", "fit_status", "marks")}
    for x, ((o, _s, man, marks), lab) in enumerate(zip(rows, labels), start=1):
        ax = man["axles"]
        for k, v in (("x", x), ("outing_id", o), ("label", lab.replace(",", " ")),
                     # K is not persisted; the gate's own formula on the persisted fit
                     ("K_f_per_rad", ax["front"]["B"] * ax["front"]["C"] * ax["front"]["mu"]),
                     ("K_r_per_rad", ax["rear"]["B"] * ax["rear"]["C"] * ax["rear"]["mu"]),
                     ("mu_f", ax["front"]["mu"]), ("mu_r", ax["rear"]["mu"]),
                     ("k_zone_f", man["k_band_zone"]["front"]), ("k_zone_r", man["k_band_zone"]["rear"]),
                     ("mu_zone_f", man["mu_ceiling_zone"]["front"]), ("mu_zone_r", man["mu_ceiling_zone"]["rear"]),
                     ("fit_status", man.get("status")), ("marks", "; ".join(marks) or "-")):
            cols[k].append(v)
    data_dir, plots_dir, prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, outings, tag, k_band=lin["k_band"], k_edge=lin["k_edge"],
                                 mu_ceiling=lin["mu_ceiling"], mu_edge=round(lin["mu_edge"], 6),
                                 source="stored payloads (identity-valid)"))
    sep = None
    if separator_after is not None:
        sep = outings.index(separator_after) + 1.5
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(prefix, len(rows), lin, cols, sep))
    return cols


def _band(x0, x1, y0, y1, fill):
    return (f"\\addplot[draw=none, fill={fill}, forget plot] coordinates "
            f"{{({x0},{y0:.4g}) ({x1},{y0:.4g}) ({x1},{y1:.4g}) ({x0},{y1:.4g})}} -- cycle;\n")


def _tex(prefix, n, lin, cols, sep):
    x0, x1 = 0.5, n + 0.5
    k_hi = max(lin["k_edge"][1], max(cols["K_f_per_rad"] + cols["K_r_per_rad"])) * 1.08
    mu_lo = min(cols["mu_f"] + cols["mu_r"]) - 0.1
    mu_hi = max(lin["mu_edge"], max(cols["mu_f"] + cols["mu_r"])) + 0.08
    t = ["% generated by tools/thesis_figures/fig_k_sessions.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblK")]
    t.append("\\begin{groupplot}[group style={group size=1 by 2, vertical sep=0.4cm, xticklabels at=edge bottom},\n"
             f"  thesis axis, width=\\textwidth, height=0.3\\textwidth, xmin={x0}, xmax={x1},\n"
             # explicit ticks: xtick=data would read the band polygon's x
             f"  xtick={{1,...,{n}}}, xticklabels from table={{\\tblK}}{{label}}]\n")
    t.append(f"\\nextgroupplot[ylabel={{$K_i$ in 1/rad}}, ymin=0, ymax={k_hi:.3g}]\n")
    t.append(_band(x0, x1, lin["k_edge"][0], lin["k_band"][0], "auxGray!8"))
    t.append(_band(x0, x1, lin["k_band"][0], lin["k_band"][1], "auxGray!20"))
    t.append(_band(x0, x1, lin["k_band"][1], lin["k_edge"][1], "auxGray!8"))
    t += _points("K_f_per_rad", "K_r_per_rad", legend=False)
    if sep is not None:
        t.append(f"\\addplot[black, densely dashed, forget plot] coordinates {{({sep},0) ({sep},{k_hi:.3g})}};\n")
    # one legend for both panels, below the last one
    t.append(f"\\nextgroupplot[ylabel={{$\\mu_i$}}, ymin={mu_lo:.3g}, ymax={mu_hi:.3g},\n"
             "  legend style={font=\\small, at={(0.5,-0.22)}, anchor=north, legend columns=2}]\n")
    t.append(_band(x0, x1, mu_lo, lin["mu_ceiling"], "auxGray!20"))
    t.append(_band(x0, x1, lin["mu_ceiling"], lin["mu_edge"], "auxGray!8"))
    t += _points("mu_f", "mu_r", legend=True)
    if sep is not None:
        t.append(f"\\addplot[black, densely dashed, forget plot] coordinates {{({sep},{mu_lo:.3g}) ({sep},{mu_hi:.3g})}};\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def _points(col_f, col_r, legend):
    fp = "" if legend else ", forget plot"
    t = [f"\\addplot[only marks, mark=*, color=axleFront{fp}] table[x=x, y={col_f}]{{\\tblK}};\n",
         f"\\addplot[only marks, mark=square*, color=axleRear{fp}] table[x=x, y={col_r}]{{\\tblK}};\n"]
    if legend:
        t.insert(1, "\\addlegendentry{front axle}\n")
        t.append("\\addlegendentry{rear axle}\n")
    return t


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outings", required=True, help="comma-separated app outing IDs, in plot order")
    ap.add_argument("--labels", help="comma-separated tick labels, same order (default: neutral '<code> <n>')")
    ap.add_argument("--separator-after", type=int, help="outing ID after which a group separator is drawn")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    outings = [int(x) for x in a.outings.split(",")]
    labels = a.labels.split(",") if a.labels else None
    build(outings, labels, a.separator_after, a.tag, a.allow_status, a.allow_excluded, a.out)


if __name__ == "__main__":
    main()
