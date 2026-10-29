# fig:cs_reading -- how the cornering stiffness ratio is read from the
# load-normalised axle force and the slip angle. Two panels, front | rear:
# Fy_i / Fz_i over alpha_i for the samples of one corner bracket (thin gray),
# the samples of ONE regression window as axle-colour markers, and three
# slopes through the window's start sample: the blended k_tilde (C_alpha),
# the plain window slope k_win (C_window) and the linear reference k_lin
# (C_linear_ref), each extended SLOPE_EXTEND of the window's slip-angle span
# beyond either side so the slopes stay readable over the markers. The
# render reads the slopes from the CSV header (per rad, 4 decimals) and the
# start point from the window CSV. Each axle shows the lap and phase behind
# its tab:example_verdicts value (worst lap per phase), and the window whose
# CS lies closest to that phase median over the clean samples the verdict
# statistics read (summarise_corners: phase slice, moving, kerb grade mark
# and observer-reset mark excluded). Force and slip angle are multiplied by
# the corner direction (sign of the median Fy over the bracket), which
# leaves every slope unchanged. Read-only full re-run (common.run_chain).
# Run: python -m tools.thesis_figures.fig_cs_reading --outing 26 --corner 17
#      --front 18:exit_4 --rear 15:exit_5 [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_cs_reading"
AXLES = ("f", "r")
AXLE_WORD = {"f": "front", "r": "rear"}
DEG = np.pi / 180.0
MEDIAN_TOL = 1e-9   # the recomputed phase median must be the stored one
PANEL_WIDTH = "0.4\\textwidth"
PANEL_HEIGHT = "0.34\\textwidth"
# window samples: markers only, front circle, rear square
STYLE = {"f": "axleFront, only marks, mark=*, mark size=1.8pt",
         "r": "axleRear, only marks, mark=square*, mark size=1.8pt"}
SLOPES = (("k_tilde", "black, solid, thick", "$\\tilde{k}$"), ("k_lin", "black, dashed", "$k_\\mathrm{lin}$"),
          ("k_win", "black, densely dotted, thick", "$k_\\mathrm{win}$"))
SLOPE_EXTEND = 0.5   # of the window's alpha span, on each side
BRACKET_STYLE = "auxGray!55, thin"


def axle_panel(session, res, corner, axle, lap, phase):
    from modules.classifier import verdict_cs_stat
    from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples
    st, cs, slip = res["state"], res["cs"], res["slip"]
    t, s = st["time"], st["s_m"]
    se = session["params"]["stability_estimation"]
    inst = [c for c in res["corners"] if c["lap_number"] == lap and c.get("stable_corner_id") == corner]
    summ = [x for x in res["summaries"] if x["lap_number"] == lap and x.get("stable_corner_id") == corner]
    if not inst or not summ:
        raise common.Refused(f"lap {lap} corner {corner}: no such corner instance on the analysis laps")
    inst, summ = inst[0], summ[0]

    # the verdict population of this phase, as summarise_corners forms it
    a, b = inst["segments"][phase]
    idx = np.arange(np.searchsorted(t, a, "left"), np.searchsorted(t, b, "right"))
    idx = idx[np.asarray(st["moving_mask"], bool)[idx]]
    raw = np.asarray(cs[f"CS_ratio_{axle}"], float)
    excl = np.asarray(cs[f"near_strike_{axle}"], bool).copy()
    if f"observer_reset_affected_{axle}" in cs:
        excl |= np.asarray(cs[f"observer_reset_affected_{axle}"], bool)
    clean = np.where(excl, np.nan, raw)
    ok = idx[np.isfinite(clean[idx])]
    med = float(np.median(clean[ok]))
    stored = verdict_cs_stat(summ, phase, axle)["median"]
    if abs(med - stored) > MEDIAN_TOL:
        raise common.Refused(f"{AXLE_WORD[axle]} lap {lap} {phase}: recomputed median {med:.6f} != stored {stored:.6f}")
    i = int(ok[np.argmin(np.abs(clean[ok] - med))])
    alpha = np.asarray(slip[f"alpha_{axle}_filt"], float)
    fz = res["axle_loads"][f"fz_{axle}_N"]
    y = np.asarray(res["forces"][f"Fy_{axle}_filt"], float) / fz
    start = reconstruct_cs_window_start(alpha, i, resolve_cs_min_window_samples(session["params"], st["sample_rate_hz"]),
                                        se["cs_min_slip_angle_span_rad"], s, se["cs_max_window_m"])
    k = {name: float(cs[f"{key}_{axle}"][i]) for name, key in
         (("k_win", "C_window"), ("k_sec", "C_section"), ("k_tilde", "C_alpha"), ("k_lin", "C_linear_ref"))}
    r2 = float(cs[f"R2_{axle}"][i])
    # where k_lin was set: the last sample up to i whose own C_alpha became it
    ca, clr = np.asarray(cs[f"C_alpha_{axle}"]), np.asarray(cs[f"C_linear_ref_{axle}"])
    j = int(np.flatnonzero((np.arange(len(t)) <= i) & (ca == clr[i]) & (clr == clr[i]))[-1])
    lap_of_j = [l["lap_number"] for l in session["parsed"]["laps"] if l["start_time"] <= t[j] <= l["end_time"]]

    t0, t1 = inst["segments"]["entry_2_turnin"][0], inst["segments"]["exit_5"][1]
    br = np.arange(np.searchsorted(t, t0, "left"), np.searchsorted(t, t1, "right"))
    sign = float(np.sign(np.nanmedian(y[br]))) or 1.0
    br_excl = excl[br] | np.asarray(st["kerb_mask"], bool)[br] | ~np.asarray(st["moving_mask"], bool)[br]
    br = br[::max(1, int(np.ceil(br.size / common.SCATTER_CAP)))]
    xb = np.where(excl[br] | np.asarray(st["kerb_mask"], bool)[br], np.nan, sign * np.degrees(alpha[br]))
    win = np.arange(start, i)
    xw, yw = sign * np.degrees(alpha[win]), sign * y[win]
    info = {"lap": lap, "phase": phase, "phase_median": med, "cs_window": float(raw[i]),
            "n_window": int(i - start), "window_s": float(t[i - 1] - t[start] + 1.0 / st["sample_rate_hz"]),
            "window_m": (float(s[start]), float(s[i - 1])), "r2": r2, **k,
            "k_lin_ref": {"lap": lap_of_j[0] if lap_of_j else None, "sample": j,
                          "s_m": float(s[j - 1])}, "n_bracket_excluded": int(br_excl.sum()), "sign": sign}
    return {"bracket": {"alpha_deg": xb, "fy_fz": sign * y[br]}, "window": {"alpha_deg": xw, "fy_fz": yw},
            "info": info}


def build(outing, corner, picks, tag="dev", allow_status=False, allow_excluded=False, out=None):
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    if res["axle_loads"] is None:
        raise common.Refused(f"outing {outing}: no measured axle loads (not the fit-L CS input)")
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    panels = {ax: axle_panel(session, res, corner, ax, *picks[ax]) for ax in AXLES}
    extra = {}
    for ax in AXLES:
        p = panels[ax]["info"]
        extra[f"{ax}_window"] = (f"lap {p['lap']} {common.PHASE_LABELS[p['phase']]}; phase median {p['phase_median']:.6f}; "
                                 f"window CS {p['cs_window']:.6f}; {p['n_window']} samples {p['window_s']:.3f} s "
                                 f"s {p['window_m'][0]:.1f}-{p['window_m'][1]:.1f} m; k_win {p['k_win']:.4f} "
                                 f"R2 {p['r2']:.4f} k_sec {p['k_sec']:.4f} k_tilde {p['k_tilde']:.4f} "
                                 f"k_lin {p['k_lin']:.4f} (set on lap {p['k_lin_ref']['lap']} at s "
                                 f"{p['k_lin_ref']['s_m']:.1f} m)")
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, corner=corner,
                    regressand="Fy / Fz per axle (measured axle loads, 2 Hz zero-phase), x corner direction",
                    slopes="per rad; drawn through the first window sample", **extra)
    for ax in AXLES:
        common.write_csv(f"{data_dir}/{SCRIPT}_{ax}.csv", panels[ax]["bracket"], m)
        common.write_csv(f"{data_dir}/{SCRIPT}_{ax}_window.csv", panels[ax]["window"], m)
    render(out)
    return {ax: panels[ax]["info"] for ax in AXLES}


def render(out=None):
    """The .tex from the stored CSVs only: lap, phase, CS and the three
    slopes from the header, the window start point and span from the
    window CSV. A missing value refuses the render."""
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, _c = common.read_csv(f"{data_dir}/{SCRIPT}_f.csv")
    panels = {}
    for ax in AXLES:
        hdr = meta.get(f"{ax}_window") or ""
        g = re.match(r"lap (\d+) (.+?); phase median \S+; window CS (\S+);", hdr)
        k = {name: re.search(rf"\b{name} (-?\d+(?:\.\d+)?)", hdr) for name, _s, _l in SLOPES}
        if g is None or None in k.values():
            raise common.Refused(f"{SCRIPT}: {ax} header lacks lap / CS / slopes -- run the full build")
        _m, w = common.read_csv(f"{data_dir}/{SCRIPT}_{ax}_window.csv")
        xw = np.array([common.num(x) for x in w["alpha_deg"]])
        yw = np.array([common.num(x) for x in w["fy_fz"]])
        x0, y0 = float(xw[0]), float(yw[0])
        span = float(xw.max() - xw.min())
        xa, xz = float(xw.min()) - SLOPE_EXTEND * span, float(xw.max()) + SLOPE_EXTEND * span
        # slopes per rad, the alpha axis in deg
        segs = [(name, xa, y0 + float(k[name].group(1)) * (xa - x0) * DEG,
                 xz, y0 + float(k[name].group(1)) * (xz - x0) * DEG) for name, _s, _l in SLOPES]
        panels[ax] = {"lap": g.group(1), "phase": g.group(2), "cs": float(g.group(3)), "segs": segs}
    marks = [m for m in (meta.get("marks") or "").split("; ") if m]
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(prefix, panels, marks))


def _tex(prefix, panels, marks):
    t = ["% generated by tools/thesis_figures/fig_cs_reading.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    for ax in AXLES:
        t.append(common.table_read(prefix, f"{SCRIPT}_{ax}.csv", f"tblRead{ax.upper()}"))
        t.append(common.table_read(prefix, f"{SCRIPT}_{ax}_window.csv", f"tblWin{ax.upper()}"))
    t.append("\\begin{groupplot}[group style={group size=2 by 1, horizontal sep=1.6cm},\n"
             f"  thesis axis, scale only axis, width={PANEL_WIDTH}, height={PANEL_HEIGHT}, unbounded coords=jump,\n"
             "  xlabel={$\\alpha$ in deg}, yticklabel style={/pgf/number format/fixed}]\n")
    sym = {"f": common.CS_F, "r": common.CS_R}
    ylab = {"f": "$F_{y,\\mathrm{f}}/F_{z,\\mathrm{f}}$", "r": "$F_{y,\\mathrm{r}}/F_{z,\\mathrm{r}}$"}
    style = {name: s for name, s, _l in SLOPES}
    for n, ax in enumerate(AXLES):
        p = panels[ax]
        opts = f"ylabel={{{ylab[ax]}}}, title={{{AXLE_WORD[ax]} axle}}"
        if n == 1:
            opts += (", legend to name=legCsReading, legend style={cells={anchor=west}, legend columns=3, "
                     "/tikz/every even column/.append style={column sep=0.6em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        t.append(f"\\addplot[{BRACKET_STYLE}, forget plot] table[x=alpha_deg, y=fy_fz]{{\\tblRead{ax.upper()}}};\n")
        # markers only: the window samples need not be monotonic in alpha
        t.append(f"\\addplot[{STYLE[ax]}, forget plot] table[x=alpha_deg, y=fy_fz]{{\\tblWin{ax.upper()}}};\n")
        for name, x0, y0, x1, y1 in p["segs"]:
            t.append(f"\\addplot[{style[name]}, forget plot] coordinates {{({x0:.4f},{y0:.5f}) ({x1:.4f},{y1:.5f})}};\n")
        t.append(f"\\node[anchor=north west, align=left, font=\\small, fill=white, inner sep=2pt] at "
                 f"(rel axis cs:0.03,0.97) {{lap {p['lap']}, {p['phase']}\\\\ {sym[ax]} $= {p['cs']:.3f}$}};\n")
        if n == 1:
            t.append(f"\\addlegendimage{{{BRACKET_STYLE}}}\\addlegendentry{{bracket samples}}\n"
                     f"\\addlegendimage{{{STYLE['f']}}}\\addlegendentry{{window samples, front}}\n"
                     f"\\addlegendimage{{{STYLE['r']}}}\\addlegendentry{{window samples, rear}}\n"
                     + "".join(f"\\addlegendimage{{{s}}}\\addlegendentry{{{lab}}}\n" for _n, s, lab in SLOPES))
    t.append("\\end{groupplot}\n")
    note = f" ({'; '.join(marks)})" if marks else ""
    t.append("\\node[anchor=north, below=2pt] at ($(group c1r1.outer south)!0.5!(group c2r1.outer south)$) "
             "{\\pgfplotslegendfromname{legCsReading}" + note + "};\n")
    t.append("\\end{tikzpicture}\n")
    return "".join(t)


def _pick(text):
    lap, phase = text.split(":")
    if phase not in common.PHASE_LABELS:
        raise argparse.ArgumentTypeError(f"phase {phase!r}: one of {', '.join(common.PHASE_LABELS)}")
    return int(lap), phase


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--corner", type=int, help="stable corner id of that session")
    ap.add_argument("--front", type=_pick, help="lap:phase_key of the front panel")
    ap.add_argument("--rear", type=_pick, help="lap:phase_key of the rear panel")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSVs only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.corner, a.front, a.rear):
        ap.error("--outing, --corner, --front and --rear are required unless --render-only")
    print(build(a.outing, a.corner, {"f": a.front, "r": a.rear}, a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
