# fig:example_corner -- one corner of one lap over lap distance, four
# stacked panels with the phases shaded: (1) the filtered a_y in g, (2)
# observer beta vs kinematic beta, (3) alpha_f, alpha_r, (4) CS_f, CS_r with
# the lines a verdict rule acts on (front decision boundary and strong
# tier, rear decision boundary; the app screen also draws the rear
# information tier). Panel 3 carries the slip-angle range of the session's
# fit samples (p0.5-p99.5 of the fit population per axle) as a light band:
# where the observer's slip angles leave it, the tire curve is
# extrapolated. The steer angle stays in the CSV only. Read-only
# full re-run of the app's chain (common.run_chain); one outing, lap and
# corner. Stable output names: the session, lap and corner go into the
# CSV header only.
# Run: python -m tools.thesis_figures.fig_example_corner --outing 14 --lap 53
#      --corner 1 [--tag r2] [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_example_corner"
G = 9.81   # m/s^2 per g, as in the app's load code
PANEL_HEIGHT = "0.15\\textwidth"   # axis box; with the panel gap about 0.18 \textwidth per panel
AXIS_WIDTH = "0.86\\textwidth"     # leaves room for the y labels inside \textwidth


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
    # signed extremes, for the comparison with the fit samples' p0.5-p99.5 range
    for ax in ("f", "r"):
        max_alpha[f"{ax}_lo"] = float(np.nanmin(alpha[ax][ok])) if ok.any() else float("nan")
        max_alpha[f"{ax}_hi"] = float(np.nanmax(alpha[ax][ok])) if ok.any() else float("nan")
    return cols, phases, lo, hi, summ, max_alpha


def build(outing, lap, corner, tag="dev", allow_status=False, allow_excluded=False, out=None):
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    cols, phases, lo, hi, _summ, _amax = corner_traces(session, res, lap, corner)
    fit_range = common.fit_sample_range(session)
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
                    lines=f"boundary f {lin['boundary_f']:g} r {lin['boundary_r']:g}; strong f {lin['strong_f']:g}; "
                          f"info r {lin['info_r']:g}",
                    fit_sample_range_deg=f"p{common.FIT_RANGE_PCT[0]:g}-p{common.FIT_RANGE_PCT[1]:g} of the fit "
                                         f"population: f {fit_range['f'][0]:.3f} to {fit_range['f'][1]:.3f} "
                                         f"(n {fit_range['n_f']}); r {fit_range['r'][0]:.3f} to "
                                         f"{fit_range['r'][1]:.3f} (n {fit_range['n_r']})")
    common.write_csv(f"{data_dir}/{base}.csv", cols, m)
    common.write_csv(f"{data_dir}/{base}_phases.csv", phases, m)
    render(out)
    return {"summaries": res["summaries"], "fit_manifest": res["fit_manifest"], "marks": marks, "files": base,
            "n_bins": len(cols["s_m"]), "n_nan_cs_f": int(np.isnan(cols["cs_f"]).sum()),
            "n_nan_cs_r": int(np.isnan(cols["cs_r"]).sum()), "fit_range": fit_range}


def _range(*arrs, pad=0.08):
    v = np.concatenate([a[np.isfinite(a)] for a in arrs])
    lo, hi = float(v.min()), float(v.max())
    d = (hi - lo) * pad or 1.0
    return lo - d, hi + d


def render(out=None):
    """The .tex from the stored CSVs only (no session is loaded): traces and
    phase bounds from the two CSVs, the fit-sample range from the CSV
    header, the CS lines from config."""
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, raw = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    _m, ph = common.read_csv(f"{data_dir}/{SCRIPT}_phases.csv")
    cols = {k: np.array([common.num(x) for x in v]) for k, v in raw.items()}
    phases = {"phase_label": ph["phase_label"], "s_start_m": [float(x) for x in ph["s_start_m"]],
              "s_end_m": [float(x) for x in ph["s_end_m"]]}
    rng = re.search(r"f (\S+) to (\S+) \(n \d+\); r (\S+) to (\S+) \(n", meta["fit_sample_range_deg"])
    fit_range = {"f": (float(rng.group(1)), float(rng.group(2))), "r": (float(rng.group(3)), float(rng.group(4)))}
    # bin centres sit half a bin inside the plotted range
    lo = min([cols["s_m"][0] - common.TRACE_BIN_M / 2] + phases["s_start_m"])
    hi = max([cols["s_m"][-1] + common.TRACE_BIN_M / 2] + phases["s_end_m"])
    marks = [m for m in (meta.get("marks") or "").split("; ") if m]
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex",
                     _tex(SCRIPT, prefix, cols, phases, common.lines(), lo, hi, marks, fit_range))


ALPHA_PANEL = 2
BAND_STYLE = {"f": "axleFront", "r": "axleRear"}
BAND_OPACITY = 0.15
CS_RANGE = (-0.2, 1.05)          # widened downwards only if the corner's CS goes lower
YLABEL_SHIFT = "3.4em"           # one fixed y-label position for all panels
# front = axle colour, solid, circles; rear = axle colour, dashed, squares
FRONT = "axleFront, solid, mark=*, mark size=1pt, mark repeat=4"
REAR = "axleRear, dashed, mark=square*, mark size=1pt, mark repeat=4, mark options={solid}"


def _tex(base, prefix, cols, phases, lin, lo, hi, marks, fit_range):
    panels = [
        ("$a_y$ in g", [("ay_filt_g", "black, solid")]),
        ("$\\beta$ in deg", [("beta_obs_deg", "black, solid"), ("beta_kin_deg", "auxGray, dashed")]),
        ("$\\alpha$ in deg", [("alpha_f_deg", FRONT), ("alpha_r_deg", REAR)]),
        (common.CS_AXIS, [("cs_f", FRONT), ("cs_r", REAR)]),
    ]
    # thesis figure: only the lines a verdict rule acts on (B10 amended
    # 2026-10-27); the app screen keeps the rear information tier as well
    cs_lines = [(lin["boundary_f"], "axleFront, thin, dashdotted"),
                (lin["strong_f"], "axleFront, thin, densely dotted"),
                (lin["boundary_r"], "axleRear, thin, dashdotdotted")]
    ranges = [_range(*[cols[c] for c, _s in p[1]]) for p in panels]
    a_lo, a_hi = ranges[ALPHA_PANEL]
    ranges[ALPHA_PANEL] = (min(a_lo, fit_range["f"][0], fit_range["r"][0]) - 0.3,
                           max(a_hi, fit_range["f"][1], fit_range["r"][1]) + 0.3)
    cs_min = float(np.nanmin(np.concatenate([cols["cs_f"], cols["cs_r"]])))
    ranges[-1] = (min(CS_RANGE[0], round(cs_min - 0.05, 2)), CS_RANGE[1])
    note = f" ({'; '.join(marks)})" if marks else ""
    last = len(panels) - 1
    t = ["% generated by tools/thesis_figures/fig_example_corner.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{base}.csv", "tblCorner")]
    # scale only axis: the panel height is the axis box itself, so the tick
    # and label allowance pgfplots subtracts otherwise cannot squash the panels
    t.append(f"\\begin{{groupplot}}[group style={{group size=1 by {len(panels)}, vertical sep=0.3cm, "
             "xlabels at=edge bottom,\n    xticklabels at=edge bottom},\n"
             f"  thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT}, unbounded coords=jump,\n"
             f"  xmin={lo:.1f}, xmax={hi:.1f}, xlabel={{$s$ in m}},\n"
             "  xticklabel style={/pgf/number format/.cd, fixed, set thousands separator={}},\n"
             "  yticklabel style={/pgf/number format/fixed},\n"
             # every y label at the same distance from its axis, whatever the tick width
             # (.style replaces the default anchor=near ticklabel, which follows the tick width)
             f"  every axis y label/.style={{at={{(rel axis cs:0,0.5)}}, xshift=-{YLABEL_SHIFT}, rotate=90, "
             "anchor=center}]\n")
    for k, (ylabel, series) in enumerate(panels):
        y0, y1 = ranges[k]
        opts = f"ylabel={{{ylabel}}}, ymin={y0:.4g}, ymax={y1:.4g}"
        if k == 0:
            # phase labels sit above the top panel, outside the axis box
            opts += ", clip=false" + (f", title={{{note.strip()}}}, title style={{yshift=1.2em}}" if note else "")
        if k == last:
            opts += (", legend to name=legExampleCorner, legend style={cells={anchor=west}, legend columns=5, "
                     "/tikz/every even column/.append style={column sep=0.6em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        # phases: light alternating bands; the apex (one instant) a thin dotted line
        j = 0
        for s0, s1, lab in zip(phases["s_start_m"], phases["s_end_m"], phases["phase_label"]):
            abbr = common.PHASE_ABBR.get(lab, lab)
            if s1 - s0 < common.TRACE_BIN_M / 2:
                t.append(f"\\addplot[black, thin, densely dotted, forget plot] coordinates "
                         f"{{({s0:.2f},{y0:.4g}) ({s0:.2f},{y1:.4g})}};\n")
                x_lab = s0
            else:
                shade = "auxGray!14" if j % 2 == 0 else "auxGray!5"
                j += 1
                t.append(f"\\addplot[draw=none, fill={shade}, forget plot] coordinates "
                         f"{{({s0:.2f},{y0:.4g}) ({s1:.2f},{y0:.4g}) ({s1:.2f},{y1:.4g}) ({s0:.2f},{y1:.4g})}} -- cycle;\n")
                x_lab = 0.5 * (s0 + s1)
            if k == 0:
                t.append(f"\\node[anchor=south, font=\\small, inner sep=1pt] at (axis cs:{x_lab:.2f},{y1:.4g}) {{{abbr}}};\n")
        if k == ALPHA_PANEL:
            for ax, (b0, b1) in ((a, fit_range[a]) for a in ("f", "r")):
                t.append(f"\\addplot[draw=none, fill={BAND_STYLE[ax]}, fill opacity={BAND_OPACITY}, forget plot] "
                         f"coordinates {{({lo:.1f},{b0:.4g}) ({hi:.1f},{b0:.4g}) ({hi:.1f},{b1:.4g}) "
                         f"({lo:.1f},{b1:.4g})}} -- cycle;\n")
        if k == last:
            t.append(f"\\addlegendimage{{{FRONT}, thick}}\\addlegendentry{{front axle}}\n"
                     f"\\addlegendimage{{{REAR}, thick}}\\addlegendentry{{rear axle}}\n"
                     "\\addlegendimage{black, solid, thick}\\addlegendentry{observer}\n"
                     "\\addlegendimage{auxGray, dashed, thick}\\addlegendentry{kinematic}\n"
                     f"\\addlegendimage{{area legend, draw=none, fill=auxGray, fill opacity={2 * BAND_OPACITY}}}"
                     "\\addlegendentry{fit-sample range}\n")
            for val, style in cs_lines:
                t.append(f"\\addplot[{style}, forget plot] coordinates {{({lo:.1f},{val:.4g}) ({hi:.1f},{val:.4g})}};\n")
        for col, style in series:
            t.append(f"\\addplot[{style}, thick, forget plot] table[x=s_m, y={col}]{{\\tblCorner}};\n")
    t.append("\\end{groupplot}\n")
    t.append(f"\\node[anchor=north, below=2pt] at (group c1r{len(panels)}.outer south) "
             "{\\pgfplotslegendfromname{legExampleCorner}};\n")
    t.append("\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--corner", type=int, help="stable corner id of that session")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSVs only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap, a.corner):
        ap.error("--outing, --lap and --corner are required unless --render-only")
    res = build(a.outing, a.lap, a.corner, a.tag, a.allow_status, a.allow_excluded, a.out)
    print({k: v for k, v in res.items() if k not in ("summaries", "fit_manifest")})


if __name__ == "__main__":
    main()
