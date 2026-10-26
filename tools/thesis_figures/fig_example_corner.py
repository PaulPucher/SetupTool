# fig:example_corner -- one corner of one lap over lap distance, five
# stacked panels with the five phases shaded: (1) road-wheel angle, (2) the
# filtered a_y in g, (3) observer beta vs kinematic beta, (4) alpha_f,
# alpha_r, (5) CS_f, CS_r with the per-axle decision boundaries and the
# front grading tier (the rear has no grading tier; its information band
# is display-only and not drawn). Read-only full re-run of the app's chain
# (common.run_chain); one outing, lap and corner. Stable output names: the
# session, lap and corner go into the CSV header only.
# Run: python -m tools.thesis_figures.fig_example_corner --outing 14 --lap 53
#      --corner 1 [--tag r2] [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_example_corner"
G = 9.81   # m/s^2 per g, as in the app's load code
NARROW_PHASE_FRACTION = 0.12   # of the plotted distance range; layout only
PANEL_HEIGHT = "0.2\\textwidth"


def _instance(res, lap, corner):
    c = [c for c in res["corners"] if c.get("lap_number") == lap and c.get("stable_corner_id") == corner]
    s = [s for s in res["summaries"] if s["lap_number"] == lap and s.get("stable_corner_id") == corner]
    if not c or not s:
        raise common.Refused(f"lap {lap} corner {corner}: no such corner instance on the analysis laps")
    return c[0], s[0]


def corner_traces(session, res, lap, corner):
    """The figure's data for one corner instance: 2 m bins over the corner
    and the phase bounds in lap distance; plus, for find_example_corner, the
    largest observer |alpha| per axle over the plotted, non-excluded samples."""
    from modules.stability_analysis import _butterworth_lowpass
    inst, summ = _instance(res, lap, corner)
    st, cs, slip = res["state"], res["cs"], res["slip"]
    lin = common.lines()
    lap_row = [l for l in session["parsed"]["laps"] if l["lap_number"] == lap][0]
    t, s = st["time"], st["s_m"]
    on_lap = (t >= lap_row["start_time"]) & (t <= lap_row["end_time"])

    # phase bounds in lap distance; braking can start before the summary's
    # distance bracket, so the plotted range is the union of both
    phases = {"phase_key": [], "phase_label": [], "s_start_m": [], "s_end_m": []}
    for key, label in common.PHASE_LABELS.items():
        seg = (inst.get("segments") or {}).get(key)
        if seg is None or seg[1] < seg[0]:
            continue
        s0, s1 = np.interp(seg, t[on_lap], s[on_lap])
        phases["phase_key"].append(key)
        phases["phase_label"].append(label)
        phases["s_start_m"].append(float(s0))
        phases["s_end_m"].append(float(s1))
    lo = min([summ["bracket_start_m"]] + phases["s_start_m"])
    hi = max([summ["bracket_end_m"]] + phases["s_end_m"])
    sel = on_lap & (s >= lo) & (s <= hi)

    # one exclusion for every trace: kerb mask, observer reset (either
    # axle), not moving; CS additionally drops its axle's grade mark, the
    # samples the verdict statistics drop
    reset = np.zeros(t.shape, bool)
    for ax in ("f", "r"):
        if f"observer_reset_affected_{ax}" in cs:
            reset |= np.asarray(cs[f"observer_reset_affected_{ax}"], bool)
    excl = np.asarray(st["kerb_mask"], bool) | reset | ~np.asarray(st["moving_mask"], bool)
    # a_y as it enters the force estimate: masked to moving, same Butterworth
    ay_filt = _butterworth_lowpass(np.where(st["moving_mask"], st["ay_mps2"], 0.0),
                                   lin["cs_filter_cutoff_hz"], st["sample_rate_hz"])
    alpha = {"f": np.degrees(slip["alpha_f_filt"]), "r": np.degrees(slip["alpha_r_filt"])}
    traces = {
        "delta_f_deg": (np.degrees(st["delta_f_rad"]), excl),
        "ay_filt_g": (ay_filt / G, excl),
        "beta_obs_deg": (np.degrees(res["beta"]), excl),
        "beta_kin_deg": (np.degrees(res["beta_kin"]), excl),
        "alpha_f_deg": (alpha["f"], excl),
        "alpha_r_deg": (alpha["r"], excl),
        "cs_f": (np.asarray(cs["CS_ratio_f"]), excl | np.asarray(cs["near_strike_f"], bool)),
        "cs_r": (np.asarray(cs["CS_ratio_r"]), excl | np.asarray(cs["near_strike_r"], bool)),
    }
    cols = {}
    for name, (x, ex) in traces.items():
        centres, cols[name] = common.bin_trace(s[sel], np.asarray(x)[sel], ex[sel], lo, hi)
    cols = {"s_m": centres, **cols}
    ok = sel & ~excl
    max_alpha = {ax: float(np.nanmax(np.abs(alpha[ax][ok]))) if ok.any() else float("nan") for ax in ("f", "r")}
    return cols, phases, lo, hi, summ, max_alpha


def build(outing, lap, corner, tag="dev", allow_status=False, allow_excluded=False, out=None):
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    cols, phases, lo, hi, _summ, _amax = corner_traces(session, res, lap, corner)
    lin = common.lines()
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])

    data_dir, plots_dir, prefix = common.out_dirs(out)
    base = SCRIPT
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, lap=lap, corner=corner,
                    bin_m=common.TRACE_BIN_M, fit_status=(res["fit_manifest"] or {}).get("status"),
                    fallback=res["fallback_used"],
                    ay_filter=f"Butterworth {lin['cs_filter_cutoff_hz']:g} Hz zero-phase, as the force estimate; "
                              f"in g (/{G})",
                    lines=f"boundary f {lin['boundary_f']:g} r {lin['boundary_r']:g}; strong f {lin['strong_f']:g}")
    common.write_csv(f"{data_dir}/{base}.csv", cols, m)
    common.write_csv(f"{data_dir}/{base}_phases.csv", phases, m)
    common.write_tex(f"{plots_dir}/{base}.tex", _tex(base, prefix, cols, phases, lin, lo, hi, marks))
    return {"summaries": res["summaries"], "fit_manifest": res["fit_manifest"], "marks": marks, "files": base,
            "n_bins": len(cols["s_m"]), "n_nan_cs_f": int(np.isnan(cols["cs_f"]).sum()),
            "n_nan_cs_r": int(np.isnan(cols["cs_r"]).sum())}


def _range(*arrs, pad=0.08):
    v = np.concatenate([a[np.isfinite(a)] for a in arrs])
    lo, hi = float(v.min()), float(v.max())
    d = (hi - lo) * pad or 1.0
    return lo - d, hi + d


def _tex(base, prefix, cols, phases, lin, lo, hi, marks):
    panels = [
        ("$\\delta_\\mathrm{f}$ in deg", [("delta_f_deg", "black, solid", "$\\delta_\\mathrm{f}$")]),
        ("$a_y$ (filtered) in g", [("ay_filt_g", "black, solid", "$a_y$ (filtered)")]),
        ("$\\beta$ in deg", [("beta_obs_deg", "black, solid", "$\\beta$ observer"),
                             ("beta_kin_deg", "auxGray, dashdotted", "$\\beta$ kinematic")]),
        ("$\\alpha$ in deg", [("alpha_f_deg", "axleFront, solid", "$\\alpha_\\mathrm{f}$"),
                              ("alpha_r_deg", "axleRear, dashed", "$\\alpha_\\mathrm{r}$")]),
        (common.CS_AXIS, [("cs_f", "axleFront, solid", common.CS_F), ("cs_r", "axleRear, dashed", common.CS_R)]),
    ]
    cs_lines = [(lin["boundary_f"], "axleFront, thin, solid", "decision boundary f"),
                (lin["strong_f"], "axleFront, thin, dashdotted", "grading tier f (strong)"),
                (lin["boundary_r"], "axleRear, thin, solid", "decision boundary r")]
    ranges = [_range(*[cols[c] for c, _s, _l in p[1]]) for p in panels]
    cs_lo, cs_hi = ranges[-1]
    ranges[-1] = (min(cs_lo, lin["strong_f"] - 0.05), cs_hi)
    note = f" ({'; '.join(marks)})" if marks else ""
    last = len(panels) - 1
    t = ["% generated by tools/thesis_figures/fig_example_corner.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{base}.csv", "tblCorner")]
    t.append("\\begin{groupplot}[group style={group size=1 by 5, vertical sep=0.3cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom},\n"
             f"  thesis axis, width=\\textwidth, height={PANEL_HEIGHT}, unbounded coords=jump,\n"
             f"  xmin={lo:.1f}, xmax={hi:.1f}, xlabel={{$s$ in m}}]\n")
    for k, (ylabel, series) in enumerate(panels):
        y0, y1 = ranges[k]
        opts = f"ylabel={{{ylabel}}}, ymin={y0:.4g}, ymax={y1:.4g}"
        if k == 0 and note:
            opts += f", title={{{note.strip()}}}"
        if k == last:
            # one legend for all panels, below the last one
            opts += (", legend style={font=\\small, cells={anchor=west}, at={(0.5,-0.8)}, anchor=north, "
                     "legend columns=4}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        # phases: alternating light fills, labels in the top panel only; a
        # phase shorter than one bin (the apex is one instant) is a thin line
        for j, (s0, s1, lab) in enumerate(zip(phases["s_start_m"], phases["s_end_m"], phases["phase_label"])):
            if s1 - s0 < common.TRACE_BIN_M:
                t.append(f"\\addplot[auxGray, densely dotted, forget plot] coordinates "
                         f"{{({s0:.2f},{y0:.4g}) ({s0:.2f},{y1:.4g})}};\n")
                if k == 0:
                    t.append(f"\\node[rotate=90, anchor=east, font=\\small] at (axis cs:{s0:.2f},{y1:.4g}) {{{lab}}};\n")
                continue
            shade = "auxGray!12" if j % 2 == 0 else "auxGray!4"
            t.append(f"\\addplot[draw=none, fill={shade}, forget plot] coordinates "
                     f"{{({s0:.2f},{y0:.4g}) ({s1:.2f},{y0:.4g}) ({s1:.2f},{y1:.4g}) ({s0:.2f},{y1:.4g})}} -- cycle;\n")
            if k == 0:
                narrow = (s1 - s0) < NARROW_PHASE_FRACTION * (hi - lo)
                place = "rotate=90, anchor=east" if narrow else "anchor=north"
                t.append(f"\\node[{place}, font=\\small] at (axis cs:{0.5 * (s0 + s1):.2f},{y1:.4g}) {{{lab}}};\n")
        if k == last:
            for _yl, ser in panels[:-1]:
                for _col, style, leg in ser:
                    t.append(f"\\addlegendimage{{{style}, thick}}\\addlegendentry{{{leg}}}\n")
        for col, style, leg in series:
            # only the last panel feeds the shared legend directly
            if k == last:
                t.append(f"\\addplot[{style}, thick] table[x=s_m, y={col}]{{\\tblCorner}};\n\\addlegendentry{{{leg}}}\n")
            else:
                t.append(f"\\addplot[{style}, thick, forget plot] table[x=s_m, y={col}]{{\\tblCorner}};\n")
        if k == last:
            for val, style, leg in cs_lines:
                t.append(f"\\addplot[{style}] coordinates {{({lo:.1f},{val:.4g}) ({hi:.1f},{val:.4g})}};\n"
                         f"\\addlegendentry{{{leg}}}\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int, required=True)
    ap.add_argument("--lap", type=int, required=True)
    ap.add_argument("--corner", type=int, required=True, help="stable corner id of that session")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    res = build(a.outing, a.lap, a.corner, a.tag, a.allow_status, a.allow_excluded, a.out)
    print({k: v for k, v in res.items() if k not in ("summaries", "fit_manifest")})


if __name__ == "__main__":
    main()
