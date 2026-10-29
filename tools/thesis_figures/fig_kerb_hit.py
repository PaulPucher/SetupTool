# fig:kerb_hit -- one kerb strike as the kerb mask sees it (time axis).
# Panel 1: vertical acceleration a_z with the strike criterion
# |a_z - kerb_baseline_g| > kerb_z_deviation_threshold_g (both limits drawn),
# the samples above it, the mask widened by kerb_dilation_s on both sides
# (_compute_kerb_mask_from_az) and the margin kerb_smear_grade_s around the
# mask (_kerb_smear_zone: near-strike grade mark, no CS reference) shaded.
# Panel 2: a_y as logged and low-passed as the force estimate uses it
# (cs_filter_cutoff_hz, zero-phase) -- the filter spreads the strike into
# the margin. Strike choice (rule fixed before drawing): on the given lap,
# the strike sample inside a corner bracket with the largest deviation.
# No analysis re-run: only prepare_vehicle_state.
# Run: python -m tools.thesis_figures.fig_kerb_hit --outing 26 --lap 15 [--half-window-s 0.9]
#      [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_kerb_hit"
G = 9.81
AXIS_WIDTH = "0.86\\textwidth"
PANEL_HEIGHT = "0.2\\textwidth"


def _runs(mask):
    d = np.diff(np.concatenate(([0], np.asarray(mask, int), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def build(outing, lap, half_window_s=0.9, tag="dev", out=None):
    from modules.stability_analysis import _butterworth_lowpass, prepare_vehicle_state
    session = common.load_session(outing)
    p = session["params"]
    se = p["stability_estimation"]
    st = prepare_vehicle_state(session["parsed"]["channels"], p)
    if st.get("az_g") is None:
        raise common.Refused(f"outing {outing}: no usable log_acc_z")
    t, sr = st["time"], st["sample_rate_hz"]
    base, thr = se["kerb_baseline_g"], se["kerb_z_deviation_threshold_g"]
    dev = np.abs(st["az_g"] - base)
    raw = dev > thr
    inbr = np.zeros(t.shape, bool)
    for c in session["parsed"]["corners"]:
        if c["lap_number"] == lap:
            inbr[np.searchsorted(t, c["segments"]["entry_2_turnin"][0]):
                 np.searchsorted(t, c["segments"]["exit_5"][1], "right")] = True
    cand = np.flatnonzero(raw & inbr)
    if not cand.size:
        raise common.Refused(f"lap {lap}: no strike inside a corner bracket")
    k = int(cand[np.argmax(dev[cand])])
    corner = next(c["stable_corner_id"] for c in session["parsed"]["corners"] if c["lap_number"] == lap
                  and c["segments"]["entry_2_turnin"][0] <= t[k] <= c["segments"]["exit_5"][1])
    sel = np.flatnonzero(np.abs(t - t[k]) <= half_window_s)
    ay_used = _butterworth_lowpass(np.where(st["moving_mask"], st["ay_mps2"], 0.0), se["cs_filter_cutoff_hz"], sr)
    tk = t[k]
    cols = {"t_s": t[sel] - tk, "az_g": st["az_g"][sel], "ay_raw_g": st["ay_mps2"][sel] / G,
            "ay_used_g": ay_used[sel] / G}
    regions = {"kind": [], "t0_s": [], "t1_s": []}
    half = 0.5 / sr   # a sample covers +/- half a grid step
    for kind, mask in (("margin", st["kerb_smear_zone"]), ("mask", st["kerb_mask"]), ("strike", raw)):
        for a, b in _runs(np.asarray(mask, bool)[sel]):
            regions["kind"].append(kind)
            regions["t0_s"].append(float(t[sel][a] - tk - half))
            regions["t1_s"].append(float(t[sel][b] - tk + half))
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, session_label=label, lap=lap, corner=corner,
                    strike_s_m=f"{st['s_m'][k]:.1f}", peak_deviation_g=f"{dev[k]:.3f}",
                    time_zero="strike sample with the largest |a_z - baseline| inside a corner bracket of this lap",
                    criterion=f"|a_z - {base:g} g| > {thr:g} g (kerb_baseline_g, kerb_z_deviation_threshold_g)",
                    widening=f"{se['kerb_dilation_s']:g} s each side (kerb_dilation_s)",
                    margin=f"{se['kerb_smear_grade_s']:g} s each side of the mask (kerb_smear_grade_s)",
                    ay_filter=f"{se['cs_filter_cutoff_hz']:g} Hz Butterworth zero-phase (cs_filter_cutoff_hz)",
                    grid_rate_hz=f"{sr:.1f}")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols, m)
    common.write_csv(f"{data_dir}/{SCRIPT}_regions.csv", regions, m)
    render(out)
    return {"corner": corner, "s_m": float(st["s_m"][k]), "peak_dev_g": float(dev[k]), "n": len(sel),
            "regions": list(zip(regions["kind"], np.round(regions["t0_s"], 3), np.round(regions["t1_s"], 3)))}


def render(out=None):
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, raw = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    _m, rg = common.read_csv(f"{data_dir}/{SCRIPT}_regions.csv")
    tt = [common.num(x) for x in raw["t_s"]]
    base, thr = (float(x) for x in re.findall(r"(\d+(?:\.\d+)?) g", meta["criterion"])[:2])
    cut = re.search(r"\d+(?:\.\d+)? Hz", meta["ay_filter"]).group(0)
    shade = {"margin": "auxGray!12", "mask": "auxGray!30", "strike": "auxGray!60"}
    regions = list(zip(rg["kind"], (float(x) for x in rg["t0_s"]), (float(x) for x in rg["t1_s"])))
    t = ["% generated by tools/thesis_figures/fig_kerb_hit.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblKerb")]
    t.append("\\begin{groupplot}[group style={group size=1 by 2, vertical sep=0.3cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom},\n"
             f"  thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT},\n"
             f"  xmin={tt[0]:.2f}, xmax={tt[-1]:.2f}, xlabel={{$t$ in s (0 = largest deviation)}},\n"
             "  every axis y label/.style={at={(rel axis cs:0,0.5)}, xshift=-3.4em, rotate=90, anchor=center}]\n")
    panels = (("$a_z$ in g", [("az_g", "black, thick", "$a_z$")]),
              ("$a_y$ in g", [("ay_raw_g", "auxGray!70, thin", "$a_y$ as logged"),
                              ("ay_used_g", "black, densely dashed, thick", f"$a_y$ low-pass {cut}, zero-phase")]))
    for k, (ylabel, series) in enumerate(panels):
        opts = f"ylabel={{{ylabel}}}"
        if k == 1:
            # one legend below both panels, so no entry covers the trace
            opts += (", legend to name=legKerb, legend style={cells={anchor=west}, legend columns=3, "
                     "/tikz/every even column/.append style={column sep=0.8em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        # margin first, mask over it, the strike samples on top
        for kind, a, b in regions:
            t.append(f"\\fill[{shade[kind]}] ({{axis cs:{a:.4f},0}} |- {{rel axis cs:0,0}}) rectangle "
                     f"({{axis cs:{b:.4f},0}} |- {{rel axis cs:0,1}});\n")
        if k == 0:
            for v in (base + thr, base - thr):
                t.append(f"\\addplot[black, thin, dashed, forget plot] coordinates {{({tt[0]:.2f},{v:g}) ({tt[-1]:.2f},{v:g})}};\n")
            t.append(f"\\addplot[black, thin, densely dotted, forget plot] coordinates {{({tt[0]:.2f},{base:g}) ({tt[-1]:.2f},{base:g})}};\n")
        for col, style, _leg in series:
            t.append(f"\\addplot[{style}, forget plot] table[x=t_s, y={col}]{{\\tblKerb}};\n")
        if k == 1:
            for kind, word in (("strike", "above the criterion"), ("mask", "kerb mask (widened)"),
                               ("margin", "margin (near-strike mark)")):
                t.append(f"\\addlegendimage{{area legend, draw=none, fill={shade[kind]}}}\\addlegendentry{{{word}}}\n")
            t.append("\\addlegendimage{black, thin, dashed}\\addlegendentry{criterion $1 \\pm " f"{thr:g}" "$ g}\n")
            for _y, ser in panels:
                for _c, style, leg in ser:
                    t.append(f"\\addlegendimage{{{style}}}\\addlegendentry{{{leg}}}\n")
    t.append("\\end{groupplot}\n")
    t.append("\\node[anchor=north, below=2pt] at (group c1r2.outer south) {\\pgfplotslegendfromname{legKerb}};\n")
    t.append("\\end{tikzpicture}\n")
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", "".join(t))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--half-window-s", type=float, default=0.9)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap):
        ap.error("--outing and --lap are required unless --render-only")
    print(build(a.outing, a.lap, a.half_window_s, a.tag, a.out))


if __name__ == "__main__":
    main()
