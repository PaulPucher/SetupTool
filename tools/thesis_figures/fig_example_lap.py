# fig:example_lap -- a stretch of consecutive corners on one lap over lap
# distance, three panels: (1) speed in km/h, (2) CS_f, CS_r with the
# per-axle limiting boundaries, (3) dMz/dbeta with the zero line; corner
# brackets shaded and labelled with that session's corner IDs. Lap rule:
# the fastest analysis lap (a neutral rule, no hand-picking), or --lap
# for continuity with another figure of the same session.
# dMz/dbeta is the session profile: the local ridge regression over lap
# distance pooled across all analysis laps (yaw_stability), one profile
# per session, not specific to the plotted lap -- said in the header and
# the legend. Read-only full re-run of the app's chain.
# Without --s-start / --s-end the whole lap, bins widened to stay within
# the point cap.
# Run: python -m tools.thesis_figures.fig_example_lap --outing 12 [--s-start 3200 --s-end 4000]
#      [--lap N] [--tag r2] [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_example_lap"


def build(outing, s_start, s_end, tag="dev", allow_status=False, allow_excluded=False, out=None, lap_number=None):
    from modules.csv_parser import _effective_lap_time
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    if lap_number is None:
        lap = common.fastest_lap(session["parsed"]["laps"], session["ident"]["lap_filter"])
        lap_rule = "fastest analysis lap"
    else:
        if lap_number not in session["ident"]["lap_filter"]:
            raise common.Refused(f"lap {lap_number}: not an analysis lap")
        lap = [l for l in session["parsed"]["laps"] if l["lap_number"] == lap_number][0]
        lap_rule = "chosen analysis lap"
    st, cs, stab = res["state"], res["cs"], res["stab"]
    t, s = st["time"], st["s_m"]
    on_lap = (t >= lap["start_time"]) & (t <= lap["end_time"])
    # lap_distance can reset a few samples before the lap table's end: those
    # samples belong to the next lap's start and are left out
    idx = np.flatnonzero(on_lap)
    fin = np.isfinite(s[idx])
    after_reset = idx[fin & (s[idx] < np.fmax.accumulate(np.where(fin, s[idx], -np.inf)) - 1.0)]
    on_lap[after_reset] = False
    if s_start is None:
        # whole lap; the bins widen so no series exceeds the point cap
        s_start, s_end = 0.0, float(np.nanmax(s[on_lap]))
    step = max(common.TRACE_BIN_M, float(np.ceil((s_end - s_start) / common.SCATTER_CAP)))
    sel = on_lap & (s >= s_start) & (s <= s_end)
    if not sel.any():
        raise common.Refused(f"lap {lap['lap_number']}: no samples between {s_start} and {s_end} m")

    reset = np.zeros(t.shape, bool)
    for ax in ("f", "r"):
        if f"observer_reset_affected_{ax}" in cs:
            reset |= np.asarray(cs[f"observer_reset_affected_{ax}"], bool)
    excl = np.asarray(st["kerb_mask"], bool) | reset | ~np.asarray(st["moving_mask"], bool)
    traces = {
        "v_kmh": (st["v_mps"] * 3.6, excl),
        "cs_f": (np.asarray(cs["CS_ratio_f"]), excl | np.asarray(cs["near_strike_f"], bool)),
        "cs_r": (np.asarray(cs["CS_ratio_r"]), excl | np.asarray(cs["near_strike_r"], bool)),
        "dmz_dbeta_Nm_per_deg": (np.asarray(stab["stability_observed_Nm_per_deg"]), excl),
    }
    cols = {}
    for name, (x, ex) in traces.items():
        centres, cols[name] = common.bin_trace(s[sel], np.asarray(x)[sel], ex[sel], s_start, s_end, step)
    cols = {"s_m": centres, **cols}
    corners = {"corner_id": [], "s_start_m": [], "s_end_m": []}
    for sm in sorted((x for x in res["summaries"] if x["lap_number"] == lap["lap_number"]),
                     key=lambda x: x["bracket_start_m"]):
        if sm["bracket_end_m"] > s_start and sm["bracket_start_m"] < s_end:
            corners["corner_id"].append(sm["stable_corner_id"])
            corners["s_start_m"].append(max(s_start, sm["bracket_start_m"]))
            corners["s_end_m"].append(min(s_end, sm["bracket_end_m"]))

    lin = common.lines()
    data_dir, plots_dir, prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, lap=lap["lap_number"],
                    lap_rule=f"{lap_rule} ({_effective_lap_time(lap):.2f} s)",
                    s_range_m=f"{s_start:.1f}-{s_end:.1f}", bin_m=f"{step:g}",
                    dmz="session profile: local ridge regression over lap distance pooled across all analysis "
                        "laps; not specific to the plotted lap",
                    corner_ids="per session (stable_corner_id is not matched across sessions)")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols, m)
    common.write_csv(f"{data_dir}/{SCRIPT}_corners.csv", corners, m)
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(prefix, cols, corners, lin, s_start, s_end, marks))
    return {"lap": lap["lap_number"], "n_bins": len(centres), "corners": corners["corner_id"], "marks": marks}


def _range(a, pad=0.08, include=()):
    v = np.concatenate([a[np.isfinite(a)], np.asarray(include, float)])
    lo, hi = float(v.min()), float(v.max())
    d = (hi - lo) * pad or 1.0
    return lo - d, hi + d


# speed and dMz/dbeta are both solid black: they sit in separate panels, and
# a broken dMz line hid detail of the profile (author 2026-10-31). The
# boundaries keep their own patterns so they read in black and white.
SPEED = "black, solid"
DMZ = "black, solid"
BOUNDARY = {"f": "boundary, thin, densely dotted", "r": "boundary, thin, loosely dashed"}
# corner labels in \small (10 pt, unscaled at 0.95 textwidth), like the tick
# and axis labels; \scriptsize was 8 pt, below the 9 pt print minimum
LABEL_FONT = "\\small"
LABEL_W = 0.055      # corner label width incl. gap, share of the x span ("C18" in \small)
LABEL_ROW = 1.1      # em between label rows in the strip above the speed panel
# three equal panels, total 0.72 of the text width (author 2026-10-31)
PANEL_HEIGHT = (0.24, 0.24, 0.24)
# fixed CS axis wider than the data (capped at 1 in stability_analysis), so
# the curves are compressed vertically and read less steep (author 2026-10-31)
CS_YLIM = (-0.4, 1.3)
CS_TICKS = "0,0.5,1"


def _label_rows(xs, width):
    # greedy rows: each label in the lowest row whose last label it clears
    ends, rows = [], []
    for x in xs:
        r = next((i for i, e in enumerate(ends) if x - width / 2 > e), len(ends))
        if r == len(ends):
            ends.append(0.0)
        ends[r] = x + width / 2
        rows.append(r)
    return rows


def _tex(prefix, cols, corners, lin, lo, hi, marks):
    panels = [("$v$ in km/h", [("v_kmh", SPEED, "$v$")], ()),
              (common.CS_AXIS, [("cs_f", "axleFront, solid", common.CS_F), ("cs_r", "axleRear, dashed", common.CS_R)],
               (lin["boundary_f"], lin["boundary_r"])),
              ("$\\mathrm{d}M_z/\\mathrm{d}\\beta$ in N\\,m/deg",
               [("dmz_dbeta_Nm_per_deg", DMZ, "$\\mathrm{d}M_z/\\mathrm{d}\\beta$ (session profile)")], (0.0,))]
    note = f" ({'; '.join(marks)})" if marks else ""
    mids = [0.5 * (s0 + s1) for s0, s1 in zip(corners["s_start_m"], corners["s_end_m"])]
    rows = _label_rows(mids, LABEL_W * (hi - lo))
    n_rows = max(rows, default=-1) + 1
    t = ["% generated by tools/thesis_figures/fig_example_lap.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblLap")]
    # set layers: the bracket shading goes on the background layer, under the
    # axis frame and grid, so it cannot cover the axis lines
    t.append("\\begin{groupplot}[group style={group size=1 by 3, vertical sep=0.3cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom}, set layers,\n"
             # 0.95: the y labels sit outside the axis width
             "  thesis axis, width=0.95\\textwidth, unbounded coords=jump,\n"
             f"  xmin={lo:.1f}, xmax={hi:.1f}, xlabel={{$s$ in m}}, xtick distance={100 if hi - lo < 1500 else 500},\n"
             "  /pgf/number format/1000 sep={}]\n")
    last = len(panels) - 1
    for k, (ylabel, series, include) in enumerate(panels):
        y0, y1 = _range(np.concatenate([cols[c] for c, _s, _l in series]), include=include)
        if k == 1:
            # refuse rather than silently clip a sample at the fixed range
            v = np.concatenate([cols[c] for c, _s, _l in series] + [np.asarray(include, float)])
            v = v[np.isfinite(v)]
            if v.min() <= CS_YLIM[0] or v.max() >= CS_YLIM[1]:
                raise common.Refused(f"CS {v.min():.3g}..{v.max():.3g} outside the fixed axis {CS_YLIM}")
            y0, y1 = CS_YLIM
        opts = f"height={PANEL_HEIGHT[k]}\\textwidth, ylabel={{{ylabel}}}, ymin={y0:.4g}, ymax={y1:.4g}"
        if k == 1:
            opts += f", ytick={{{CS_TICKS}}}"
        if k == 0:
            # the corner labels sit in a strip above the axis box
            opts += ", clip=false"
            if note:
                opts += f", title={{{note.strip()}}}, title style={{yshift={n_rows * LABEL_ROW:.1f}em}}"
        if k == last:
            opts += (", legend style={font=\\small, cells={anchor=west}, at={(0.5,-0.4)}, anchor=north, "
                     "legend columns=3}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        for j, (cid, s0, s1) in enumerate(zip(corners["corner_id"], corners["s_start_m"], corners["s_end_m"])):
            shade = "auxGray!14" if j % 2 == 0 else "auxGray!6"
            t.append(f"\\addplot[draw=none, fill={shade}, on layer=axis background, forget plot] coordinates "
                     f"{{({s0:.2f},{y0:.4g}) ({s1:.2f},{y0:.4g}) ({s1:.2f},{y1:.4g}) ({s0:.2f},{y1:.4g})}} -- cycle;\n")
            if k == 0:
                t.append(f"\\node[anchor=south, font={LABEL_FONT}, inner sep=1pt, "
                         f"yshift={0.15 + rows[j] * LABEL_ROW:.2f}em] at "
                         f"({{axis cs:{mids[j]:.2f},0}} |- {{rel axis cs:0,1}}) {{C{cid}}};\n")
        if k == last:
            for _yl, ser, _inc in panels[:-1]:
                for _c, style, leg in ser:
                    t.append(f"\\addlegendimage{{{style}, thick}}\\addlegendentry{{{leg}}}\n")
            t.append(f"\\addlegendimage{{{BOUNDARY['f']}}}\\addlegendentry{{limiting boundary f}}\n")
            t.append(f"\\addlegendimage{{{BOUNDARY['r']}}}\\addlegendentry{{limiting boundary r}}\n")
        for col, style, leg in series:
            if k == last:
                t.append(f"\\addplot[{style}, thick] table[x=s_m, y={col}]{{\\tblLap}};\n\\addlegendentry{{{leg}}}\n")
            else:
                t.append(f"\\addplot[{style}, thick, forget plot] table[x=s_m, y={col}]{{\\tblLap}};\n")
        if k == 1:
            for val, style in ((lin["boundary_f"], BOUNDARY["f"]), (lin["boundary_r"], BOUNDARY["r"])):
                t.append(f"\\addplot[{style}, forget plot] coordinates {{({lo:.1f},{val:.4g}) ({hi:.1f},{val:.4g})}};\n")
        if k == last:
            # zero line: a plain reference, not one of the legend styles
            t.append(f"\\addplot[auxGray, thin, forget plot] coordinates {{({lo:.1f},0) ({hi:.1f},0)}};\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int, required=True)
    ap.add_argument("--s-start", type=float, help="lap distance in m; both omitted = whole lap")
    ap.add_argument("--s-end", type=float, help="lap distance in m")
    ap.add_argument("--lap", type=int, help="analysis lap; default the fastest")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    if (a.s_start is None) != (a.s_end is None):
        ap.error("--s-start and --s-end go together (both omitted = whole lap)")
    print(build(a.outing, a.s_start, a.s_end, a.tag, a.allow_status, a.allow_excluded, a.out, a.lap))


if __name__ == "__main__":
    main()
