# fig:cs_distribution -- empirical cumulative distribution of CS, one panel
# per session group, front and rear curve in the axle styles of the other
# figures (front solid, rear dashed; author 2026-10-30).
# Population, as the classifier evaluates it: one value per session x
# corner x phase x axle, the worst lap over the session's analysis laps --
# the minimum of the per-lap phase medians of CS (apex_3: the apex-region
# median), kerb-affected and observer-reset samples excluded before each
# median -- i.e. corner_aggregation.aggregate_by_corner read through
# classifier.verdict_cs_stat. A phase that does not occur in a corner (no
# sample on any lap, mostly a missing braking phase) is not a cell and
# leaves the population; the remaining cells without a value are counted
# in the header by cause: (b) every CS sample kerb- or reset-excluded,
# (c) samples present but no CS value (span below the floor, no clean
# reference yet, window across a lap boundary). Lines: limiting boundary
# per axle and the front grading tier; the rear has no grading tier, and
# its information band is display-only. Stored results only.
# Run: python -m tools.thesis_figures.fig_cs_distribution --group "NBR=11,12,15,16"
#      --group "PRC=13,14" [--tag r1] [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common
from tools.thesis_figures.fig_example_lap import BOUNDARY
from tools.thesis_figures.fig_k_sessions import collect

SCRIPT = "fig_cs_distribution"
AXLE_STYLE = {"f": "axleFront, solid", "r": "axleRear, dashed"}
# the front strong tier beside the two boundary styles of fig_example_lap
STRONG_STYLE = "boundary, thin, dashdotted"
X_TICKS = (-0.4, 0.0, 0.4, 0.8)


def cells(payload):
    """(corner, phase, axle, value, cause) per population cell, cause None
    for a cell with a value, "b" or "c" without one (module comment); and
    the number of corner-phases dropped as not occurring (per axle)."""
    from modules.classifier import verdict_cs_stat
    from modules.corner_aggregation import PHASE_KEYS, _group_by_corner, aggregate_by_corner
    laps_by_corner = _group_by_corner(payload["summaries"])
    out, dropped = [], 0
    for cid, agg in sorted(aggregate_by_corner(payload["summaries"]).items(), key=lambda kv: kv[0]):
        laps = laps_by_corner[cid]
        for ph in PHASE_KEYS:
            if sum((s["phases"].get(ph) or {}).get("n_samples", 0) for s in laps) == 0:
                dropped += 1
                continue
            for ax in ("f", "r"):
                v = verdict_cs_stat(agg, ph, ax)["median"]
                cause = None
                if not np.isfinite(v):
                    st = [verdict_cs_stat(s, ph, ax) for s in laps if s["phases"].get(ph)]
                    n_val = sum(x.get("n", 0) for x in st)
                    n_excl = sum(x.get("n_kerb_excluded", 0) + x.get("n_reset_excluded", 0) for x in st)
                    cause = "b" if n_val == 0 and n_excl > 0 else "c"
                out.append((cid, ph, ax, v, cause))
    return out, dropped


def build(groups, tag="dev", allow_status=False, allow_excluded=False, out=None):
    if not 1 <= len(groups) <= 2:
        raise common.Refused("one or two session groups")
    all_outings = [o for _n, os_ in groups for o in os_]
    rows = {o: (s, k) for o, s, _m, k in collect(all_outings, allow_status, allow_excluded)}
    lin = common.lines()
    common.write_label_map(tag, all_outings)
    data_dir, plots_dir, prefix = common.out_dirs(out)
    audit = {k: [] for k in ("group", "outing_id", "stable_corner_id", "phase_key", "axle", "cs", "no_value_cause")}
    curves = {}
    for name, os_ in groups:
        vals = {"f": [], "r": []}
        causes = {"f": [], "r": []}
        n_dropped = 0
        for o in os_:
            payload, _why = common.stored_payload(rows[o][0])
            got, dropped = cells(payload)
            n_dropped += dropped
            for cid, ph, ax, v, cause in got:
                for k, x in zip(audit, (name, o, cid, ph, ax, v, cause or "-")):
                    audit[k].append(x)
                vals[ax].append(v)
                causes[ax].append(cause)
        for ax, v in vals.items():
            v = np.asarray(v, float)
            n_pop, v = v.size, np.sort(v[np.isfinite(v)])
            ecdf = np.arange(1, v.size + 1) / v.size
            fn = f"{SCRIPT}_{name}_{ax}.csv"
            marks = [f"outing {o}: {k}" for o in os_ for k in rows[o][1]]
            labels = ", ".join(common.session_label(rows[o][0]["row"]) for o in os_)
            common.write_csv(f"{data_dir}/{fn}", {f"cs_{ax}": v, "ecdf": ecdf},
                             common.meta(SCRIPT, os_, tag, marks, group=name, session_labels=labels, axle=ax,
                                         n_population_cells=n_pop,
                                         n_cells_with_value=v.size,
                                         n_no_value_kerb_reset_excluded=causes[ax].count("b"),
                                         n_no_value_no_cs_value=causes[ax].count("c"),
                                         n_at_cap_1=int((v >= 1.0).sum()),
                                         n_dropped_phase_not_occurring=n_dropped,
                                         population="worst lap per corner x phase (aggregate_by_corner, "
                                                    "verdict_cs_stat), phases that do not occur dropped, "
                                                    "stored payloads"))
            curves[(name, ax)] = fn
    common.write_csv(f"{data_dir}/{SCRIPT}_cells.csv", audit, common.meta(SCRIPT, all_outings, tag))
    render(out)
    return {k: v for k, v in curves.items()}


def render(out=None):
    """The .tex from the stored CSVs only (no session is loaded): the group
    names in their order of appearance in the cells CSV, lines from config."""
    data_dir, plots_dir, prefix = common.out_dirs(out)
    _meta, cells_ = common.read_csv(f"{data_dir}/{SCRIPT}_cells.csv")
    names = list(dict.fromkeys(cells_["group"]))
    curves = {(n, ax): f"{SCRIPT}_{n}_{ax}.csv" for n in names for ax in ("f", "r")}
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(prefix, [(n, None) for n in names], curves, common.lines()))


def _tex(prefix, groups, curves, lin):
    # one legend below the right panel: the axles, then the decision lines;
    # no rear grading tier and no rear information band (display-only)
    t = ["% generated by tools/thesis_figures/fig_cs_distribution.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    for (name, ax), fn in curves.items():
        t.append(common.table_read(prefix, fn, f"tblCS{name.replace(' ', '')}{ax.upper()}"))
    t.append("\\begin{groupplot}[group style={group size=2 by 1, horizontal sep=1.2cm},\n"
             "  thesis axis, width=0.5\\textwidth, height=0.42\\textwidth, ymin=0, ymax=1,\n"
             # few fixed ticks in one fixed-point format, so no label collides
             f"  xtick={{{','.join(f'{x:g}' for x in X_TICKS)}}},\n"
             "  xticklabel style={/pgf/number format/fixed, /pgf/number format/precision=1},\n"
             "  yticklabel style={/pgf/number format/fixed, /pgf/number format/precision=1},\n"
             f"  xlabel={{{common.CS_AXIS}}}]\n")
    last = len(groups) - 1
    for gi, (name, _os) in enumerate(groups):
        opts = f"title={{{name}}}, ylabel={{{'cumulative share' if gi == 0 else ''}}}"
        if gi == last:
            opts += (", legend style={font=\\small, cells={anchor=west}, at={(1,-0.22)}, anchor=north east, "
                     "legend columns=3}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        if gi == last:
            t.append(f"\\addlegendimage{{{AXLE_STYLE['f']}, thick}}\\addlegendentry{{front}}\n")
            t.append(f"\\addlegendimage{{{AXLE_STYLE['r']}, thick}}\\addlegendentry{{rear}}\n")
            t.append(f"\\addlegendimage{{{STRONG_STYLE}}}\\addlegendentry{{front strong tier}}\n")
            t.append(f"\\addlegendimage{{{BOUNDARY['f']}}}\\addlegendentry{{limiting boundary f}}\n")
            t.append(f"\\addlegendimage{{{BOUNDARY['r']}}}\\addlegendentry{{limiting boundary r}}\n")
        for ax in ("f", "r"):
            t.append(f"\\addplot[{AXLE_STYLE[ax]}, thick, const plot, forget plot] "
                     f"table[x=cs_{ax}, y=ecdf]"
                     f"{{\\tblCS{name.replace(' ', '')}{ax.upper()}}};\n")
        for val, style in ((lin["boundary_f"], BOUNDARY["f"]), (lin["strong_f"], STRONG_STYLE),
                           (lin["boundary_r"], BOUNDARY["r"])):
            t.append(f"\\addplot[{style}, forget plot] coordinates {{({val:.4g},0) ({val:.4g},1)}};\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--group", action="append", help='NAME=id,id,... (one or two groups)')
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSVs only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if not a.group:
        ap.error("--group is required unless --render-only")
    groups = [(g.split("=")[0], [int(x) for x in g.split("=")[1].split(",")]) for g in a.group]
    build(groups, a.tag, a.allow_status, a.allow_excluded, a.out)


if __name__ == "__main__":
    main()
