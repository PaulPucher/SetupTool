# fig:cs_kerb -- what the kerb exclusion does to the cornering stiffness
# ratio over one kerb corner of one lap (lap distance, 2 m bins). Panel 1:
# CS_f, CS_r as the chain computes them, with the samples the verdict
# statistics drop (kerb mask, near-strike grade mark, observer reset,
# standstill) left out. Panel 2: a FIGURE-ONLY counterfactual: the same CS
# step (estimate_cornering_stiffness, same slip angles, forces and keyword
# arguments) re-run on a copy of the vehicle state whose kerb mask and
# margin are all false; only observer reset and standstill are left out.
# Nothing is stored; the chain itself never runs without the mask.
# Kerb-mask samples of the lap are shaded in both panels. Read-only full
# re-run (common.run_chain). Thesis corner (review rule 2026-10-28, fixed
# before drawing): on the given lap, the kerb-touching corner instance with
# the largest drop of its CS minimum (front or rear) without the mask.
# Run: python -m tools.thesis_figures.fig_cs_kerb --outing 26 --lap 15 --corner 13
#      [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_cs_kerb"
AXIS_WIDTH = "0.86\\textwidth"
PANEL_HEIGHT = "0.2\\textwidth"
FRONT = "axleFront, solid, mark=*, mark size=1pt, mark repeat=4"
REAR = "axleRear, dashed, mark=square*, mark size=1pt, mark repeat=4, mark options={solid}"


def _runs(mask):
    d = np.diff(np.concatenate(([0], np.asarray(mask, int), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def build(outing, lap, corner, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.stability_analysis import estimate_cornering_stiffness
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    st, cs = res["state"], res["cs"]
    t, s = st["time"], st["s_m"]
    laps = session["parsed"]["laps"]
    st_cf = dict(st, kerb_mask=np.zeros_like(st["kerb_mask"]), kerb_smear_zone=np.zeros_like(st["kerb_smear_zone"]))
    cs_cf = estimate_cornering_stiffness(res["slip"], res["forces"], st_cf, session["params"], laps, **res["cs_kwargs"])

    inst = [c for c in res["corners"] if c["lap_number"] == lap and c.get("stable_corner_id") == corner]
    if not inst:
        raise common.Refused(f"lap {lap} corner {corner}: no such corner instance")
    seg = inst[0]["segments"]
    lap_row = [l for l in laps if l["lap_number"] == lap][0]
    on = (t >= lap_row["start_time"]) & (t <= lap_row["end_time"])
    lo, hi = (float(x) for x in np.interp([seg["entry_2_turnin"][0], seg["exit_5"][1]], t[on], s[on]))
    sel = on & (s >= lo) & (s <= hi)
    reset = np.zeros(t.shape, bool)
    for ax in ("f", "r"):
        if f"observer_reset_affected_{ax}" in cs:
            reset |= np.asarray(cs[f"observer_reset_affected_{ax}"], bool)
    still = ~np.asarray(st["moving_mask"], bool)
    kerb = np.asarray(st["kerb_mask"], bool)
    cols, minima = {}, {}
    for name, src, excl in (("chain", cs, lambda ax: kerb | reset | still | np.asarray(cs[f"near_strike_{ax}"], bool)),
                            ("nokerb", cs_cf, lambda ax: reset | still)):
        for ax in ("f", "r"):
            x = np.asarray(src[f"CS_ratio_{ax}"], float)
            centres, cols[f"cs_{ax}_{name}"] = common.bin_trace(s[sel], x[sel], excl(ax)[sel], lo, hi)
            v = x[sel & ~excl(ax)]
            minima[f"{ax}_{name}"] = float(np.nanmin(v)) if np.isfinite(v).any() else float("nan")
    cols = {"s_m": centres, **cols}
    regions = {"s0_m": [], "s1_m": []}
    idx = np.flatnonzero(sel)
    for a, b in _runs(kerb[idx]):
        regions["s0_m"].append(float(s[idx][a]))
        regions["s1_m"].append(float(s[idx][b]))
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, lap=lap, corner=corner,
                    bin_m=common.TRACE_BIN_M, range_m=f"{lo:.1f} to {hi:.1f} (turn-in to corner end)",
                    minima=" ".join(f"{k} {v:.4f}" for k, v in minima.items()),
                    counterfactual="figure only: estimate_cornering_stiffness re-run with kerb_mask and "
                                   "kerb_smear_zone all false; same slip angles, forces and arguments; not stored")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols, m)
    common.write_csv(f"{data_dir}/{SCRIPT}_kerb.csv", regions, m)
    render(out)
    return {"minima": minima, "range_m": (lo, hi), "n_bins": len(centres), "kerb_runs": len(regions["s0_m"]),
            "reproduces_chain": bool(np.allclose(
                estimate_cornering_stiffness(res["slip"], res["forces"], st, session["params"], laps,
                                             **res["cs_kwargs"])["CS_ratio_f"], cs["CS_ratio_f"], equal_nan=True))}


def render(out=None):
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, raw = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    _m, rg = common.read_csv(f"{data_dir}/{SCRIPT}_kerb.csv")
    lo, hi = (float(x) for x in meta["range_m"].split(" (")[0].split(" to "))
    vals = np.array([common.num(x) for k, v in raw.items() if k != "s_m" for x in v])
    y0 = min(-0.2, float(np.nanmin(vals)) - 0.05)
    t = ["% generated by tools/thesis_figures/fig_cs_kerb.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblCsKerb")]
    t.append("\\begin{groupplot}[group style={group size=1 by 2, vertical sep=0.9cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom},\n"
             f"  thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT}, unbounded coords=jump,\n"
             f"  xmin={lo:.1f}, xmax={hi:.1f}, ymin={y0:.2f}, ymax=1.05, xlabel={{$s$ in m}},\n"
             "  xticklabel style={/pgf/number format/.cd, fixed, set thousands separator={}},\n"
             f"  ylabel={{{common.CS_AXIS}}},\n"
             "  every axis y label/.style={at={(rel axis cs:0,0.5)}, xshift=-3.4em, rotate=90, anchor=center},\n"
             "  title style={font=\\small, yshift=-0.5ex}]\n")
    for k, (name, title) in enumerate((("chain", "as computed (kerb samples excluded)"),
                                       ("nokerb", "without kerb exclusion (illustration)"))):
        opts = f"title={{{title}}}"
        if k == 1:
            opts += (", legend to name=legCsKerb, legend style={cells={anchor=west}, legend columns=3, "
                     "/tikz/every even column/.append style={column sep=0.6em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        for a, b in zip(rg["s0_m"], rg["s1_m"]):
            t.append(f"\\fill[auxGray!25] ({{axis cs:{float(a):.2f},0}} |- {{rel axis cs:0,0}}) rectangle "
                     f"({{axis cs:{float(b):.2f},0}} |- {{rel axis cs:0,1}});\n")
        t.append(f"\\addplot[black, thin, densely dotted, forget plot] coordinates {{({lo:.1f},0) ({hi:.1f},0)}};\n")
        t.append(f"\\addplot[{FRONT}, thick, forget plot] table[x=s_m, y=cs_f_{name}]{{\\tblCsKerb}};\n")
        t.append(f"\\addplot[{REAR}, thick, forget plot] table[x=s_m, y=cs_r_{name}]{{\\tblCsKerb}};\n")
        if k == 1:
            t.append(f"\\addlegendimage{{{FRONT}, thick}}\\addlegendentry{{front axle}}\n"
                     f"\\addlegendimage{{{REAR}, thick}}\\addlegendentry{{rear axle}}\n"
                     "\\addlegendimage{area legend, draw=none, fill=auxGray!25}\\addlegendentry{kerb mask}\n")
    t.append("\\end{groupplot}\n")
    t.append("\\node[anchor=north, below=2pt] at (group c1r2.outer south) {\\pgfplotslegendfromname{legCsKerb}};\n")
    t.append("\\end{tikzpicture}\n")
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", "".join(t))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--corner", type=int)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap, a.corner):
        ap.error("--outing, --lap and --corner are required unless --render-only")
    print(build(a.outing, a.lap, a.corner, a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
