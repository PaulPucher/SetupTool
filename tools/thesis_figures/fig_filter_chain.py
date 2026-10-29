# fig:filter_chain -- how the chain conditions its signals, over one corner
# of one lap (time axis, a few seconds). Panel 1: a_y as logged vs as it
# enters the force estimate (moving-masked, Butterworth low-pass of order
# BUTTERWORTH_ORDER at stability_estimation.cs_filter_cutoff_hz, zero-phase
# filtfilt; the filter acts on F_y, which is linear in a_y, so this is the
# a_y content of F_y). The yaw rate itself is used unfiltered and is not
# drawn. Panel 2: yaw acceleration as the raw gradient of the yaw rate, as
# it enters the force estimate (same low-pass on F_y), and as the yaw-moment
# regression uses it (centred rolling mean over
# stability_estimation.yaw_stability_accel_window_s,
# yaw_stability.calculate_filtered_yaw_acceleration). Values read from
# config at build time and stored in the CSV header. No analysis re-run:
# only prepare_vehicle_state.
# Run: python -m tools.thesis_figures.fig_filter_chain --outing 26 --lap 15 --corner 17
#      [--pad-s 1.0] [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_filter_chain"
G = 9.81
AXIS_WIDTH = "0.86\\textwidth"
PANEL_HEIGHT = "0.2\\textwidth"


def build(outing, lap, corner, pad_s=1.0, tag="dev", out=None):
    from modules.stability_analysis import BUTTERWORTH_ORDER, _butterworth_lowpass, prepare_vehicle_state
    from modules.yaw_stability import calculate_filtered_yaw_acceleration
    session = common.load_session(outing)
    p = session["params"]
    se = p["stability_estimation"]
    st = prepare_vehicle_state(session["parsed"]["channels"], p)
    inst = [c for c in session["parsed"]["corners"] if c["lap_number"] == lap and c.get("stable_corner_id") == corner]
    if not inst:
        raise common.Refused(f"lap {lap} corner {corner}: no such corner instance")
    seg = inst[0]["segments"]
    t, sr, mv = st["time"], st["sample_rate_hz"], st["moving_mask"]
    cutoff = se["cs_filter_cutoff_hz"]
    # filters run over the whole session, as in the chain; the window is cut afterwards
    ay_used = _butterworth_lowpass(np.where(mv, st["ay_mps2"], 0.0), cutoff, sr)
    psidd_raw = np.gradient(st["yaw_rate_radps"], t)
    psidd_force = _butterworth_lowpass(np.where(mv, psidd_raw, 0.0), cutoff, sr)
    psidd_m5 = calculate_filtered_yaw_acceleration(st["yaw_rate_radps"], t, sr, se["yaw_stability_accel_window_s"])
    t_ref = seg["entry_2_turnin"][0]
    sel = np.flatnonzero((t >= seg["entry_1_brake"][0] - pad_s) & (t <= seg["exit_5"][1] + pad_s))
    sel = sel[::max(1, int(np.ceil(sel.size / common.SCATTER_CAP)))]
    cols = {"t_s": t[sel] - t_ref, "ay_raw_g": st["ay_mps2"][sel] / G, "ay_used_g": ay_used[sel] / G,
            "psidd_raw_degps2": np.degrees(psidd_raw[sel]), "psidd_force_degps2": np.degrees(psidd_force[sel]),
            "psidd_m5_degps2": np.degrees(psidd_m5[sel])}
    marks = {k: float(seg[k][i] - t_ref) for k, i in (("entry_2_turnin", 0), ("apex_3", 0), ("exit_5", 1))}
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, session_label=label, lap=lap, corner=corner,
                    time_zero="turn-in (bracket opening) of this corner instance",
                    turn_in_apex_end_s=f"{marks['entry_2_turnin']:.3f} {marks['apex_3']:.3f} {marks['exit_5']:.3f}",
                    low_pass=f"Butterworth order {BUTTERWORTH_ORDER}, {cutoff:g} Hz (stability_estimation."
                             "cs_filter_cutoff_hz), filtfilt zero-phase, on F_y in the force estimate",
                    yaw_rate="sclu_yaw_rate used unfiltered (interpolated onto the analysis grid)",
                    yaw_accel_m5=f"np.gradient then centred rolling mean over "
                                 f"{se['yaw_stability_accel_window_s']:g} s (stability_estimation."
                                 "yaw_stability_accel_window_s)",
                    grid_rate_hz=f"{sr:.1f}")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols, m)
    render(out)
    return {"n": len(sel), "sample_rate_hz": sr, "window_s": float(cols["t_s"][-1] - cols["t_s"][0])}


def render(out=None):
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, raw = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    tt = [common.num(x) for x in raw["t_s"]]
    ti, ta, te = (float(x) for x in meta["turn_in_apex_end_s"].split())
    import re
    cut = re.search(r"\d+(?:\.\d+)? Hz", meta["low_pass"]).group(0)
    win = meta["yaw_accel_m5"].split("over ")[1].split(" (")[0]
    panels = [("$a_y$ in g", [("ay_raw_g", "auxGray!70, thin", "as logged"),
                              ("ay_used_g", "black, thick", f"low-pass {cut}, zero-phase (force estimate)")]),
              ("$\\ddot\\psi$ in deg/s$^2$", [("psidd_raw_degps2", "auxGray!70, thin", "raw gradient"),
                                             ("psidd_force_degps2", "black, thick", f"low-pass {cut} (force estimate)"),
                                             ("psidd_m5_degps2", "black, dashed", f"mean over {win} (yaw-moment regression)")])]
    t = ["% generated by tools/thesis_figures/fig_filter_chain.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblFilt")]
    t.append("\\begin{groupplot}[group style={group size=1 by 2, vertical sep=0.3cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom},\n"
             f"  thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT},\n"
             f"  xmin={tt[0]:.2f}, xmax={tt[-1]:.2f}, xlabel={{$t$ in s (0 = turn-in)}},\n"
             "  every axis y label/.style={at={(rel axis cs:0,0.5)}, xshift=-3.4em, rotate=90, anchor=center}]\n")
    last = len(panels) - 1
    for k, (ylabel, series) in enumerate(panels):
        opts = f"ylabel={{{ylabel}}}" + (", clip=false" if k == 0 else "")
        if k == last:
            # one legend below both panels, so no entry covers a trace
            opts += (", legend to name=legFilter, legend style={cells={anchor=west}, legend columns=2, "
                     "/tikz/every even column/.append style={column sep=0.8em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        for x in (ti, ta, te):
            t.append(f"\\draw[auxGray, thin, densely dashed] ({{axis cs:{x:.3f},0}} |- {{rel axis cs:0,0}}) -- "
                     f"({{axis cs:{x:.3f},0}} |- {{rel axis cs:0,1}});\n")
        if k == 0:
            for x, lab in ((ti, "T"), (ta, "A"), (te, "E")):
                t.append(f"\\node[anchor=south, font=\\small, inner sep=1pt] at ({{axis cs:{x:.3f},0}} |- "
                         f"{{rel axis cs:0,1}}) {{{lab}}};\n")
        for col, style, _leg in series:
            t.append(f"\\addplot[{style}, forget plot] table[x=t_s, y={col}]{{\\tblFilt}};\n")
        if k == last:
            for i, (_y, ser) in enumerate(panels):
                for _c, style, leg in ser:
                    word = ("$a_y$ " if i == 0 else "$\\ddot\\psi$ ") + leg
                    t.append(f"\\addlegendimage{{{style}}}\\addlegendentry{{{word}}}\n")
    t.append("\\end{groupplot}\n")
    t.append("\\node[anchor=north, below=2pt] at (group c1r2.outer south) {\\pgfplotslegendfromname{legFilter}};\n")
    t.append("\\end{tikzpicture}\n")
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", "".join(t))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--corner", type=int)
    ap.add_argument("--pad-s", type=float, default=1.0, help="seconds before braking start and after corner end")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap, a.corner):
        ap.error("--outing, --lap and --corner are required unless --render-only")
    print(build(a.outing, a.lap, a.corner, a.pad_s, a.tag, a.out))


if __name__ == "__main__":
    main()
