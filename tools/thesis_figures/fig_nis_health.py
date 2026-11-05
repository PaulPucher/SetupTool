# fig:nis_health -- the observer self-check over one lap: per sample, the
# share of the trailing window (nis_gate.nis_window_s) whose NIS exceeds the
# chi-square 95 % bound (2 degrees of freedom), with the in-band region
# [nis_gate.nis_band_low, nis_gate.nis_band_high] shaded. The session health
# score is the share of gate-mask samples whose window lies in the band;
# build recomputes it from the same per-sample shares and refuses unless it
# equals the gate's own score. Per-sample shares are the
# nis_gate._score_and_skips arithmetic, repeated here on the gate's inputs,
# which a read-only re-run captures (tyre_fit_auto.evaluate_gate wrapped for
# this call only). Windows less than half covered are blank, as in the gate.
# A side bar stacks the lap's shares below / inside / above the band.
# Run: python -m tools.thesis_figures.fig_nis_health --outing 26 --lap 15 [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_nis_health"
AXIS_WIDTH = "0.7\\textwidth"
PANEL_HEIGHT = "0.26\\textwidth"
BAR_WIDTH = "0.05\\textwidth"   # side bar: this lap's share below / inside / above the band
# bottom to top as on the share axis; below and above the band gray, inside the band tint
# in-band tint stronger than okBand (k_sessions): one thin band over a long
# trace is otherwise lost at print size
IN_BAND = "TUMGreen!25"
BAR_PARTS = (("below", "edgeBand"), ("inside", IN_BAND), ("above", "auxGray!45"))


def window_shares(nis, window, min_valid_fraction, bound):
    finite = np.isfinite(nis)
    ex = (finite & (nis > bound)).astype(float)
    cw = np.cumsum(np.insert(ex, 0, 0.0))
    cv = np.cumsum(np.insert(finite.astype(float), 0, 0.0))
    ends = np.arange(window - 1, len(nis))
    ne, nv = cw[ends + 1] - cw[ends + 1 - window], cv[ends + 1] - cv[ends + 1 - window]
    cov = nv >= min_valid_fraction * window
    frac = np.full(len(nis), np.nan)
    frac[ends[cov]] = ne[cov] / nv[cov]
    return frac


def build(outing, lap, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules import observer as nis_gate
    from modules import tyre_fit_auto as tfa
    session = common.load_session(outing)
    got = {}
    orig = tfa.evaluate_gate

    def gate(nis, mask, params, sr):
        got.update(nis=np.asarray(nis, float), mask=np.asarray(mask, bool))
        got["verdict"] = orig(nis, mask, params, sr)
        return got["verdict"]
    tfa.evaluate_gate = gate
    try:
        res = common.run_chain(session)
    finally:
        tfa.evaluate_gate = orig
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    if "nis" not in got:
        raise common.Refused(f"outing {outing}: no NIS gate ran (kinematic source)")
    p = session["params"]["nis_gate"]
    st = res["state"]
    w = nis_gate.resolve_nis_window_samples(session["params"], st["sample_rate_hz"])
    frac = window_shares(got["nis"], w, nis_gate.MIN_WINDOW_VALID_FRACTION, nis_gate._CHI2_DF2_95)
    lo, hi = p["nis_band_low"], p["nis_band_high"]
    v = frac[got["mask"] & np.isfinite(frac)]
    health = float(np.mean((v >= lo) & (v <= hi)))
    if abs(health - got["verdict"]["health_score"]) > 1e-12:
        raise common.Refused(f"outing {outing}: recomputed health {health} != gate {got['verdict']['health_score']}")
    t, s = st["time"], st["s_m"]
    lap_row = [l for l in session["parsed"]["laps"] if l["lap_number"] == lap][0]
    on = np.flatnonzero((t >= lap_row["start_time"]) & (t <= lap_row["end_time"]))
    shown = np.where(got["mask"][on], frac[on], np.nan)
    keep = np.arange(on.size)[::max(1, int(np.ceil(on.size / common.SCATTER_CAP)))]
    lv = shown[np.isfinite(shown)]
    lap_share = {"below": float(np.mean(lv < lo)), "inside": float(np.mean((lv >= lo) & (lv <= hi))),
                 "above": float(np.mean(lv > hi))}
    cols = {"s_m": s[on][keep], "share_pct": 100.0 * shown[keep]}
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, lap=lap,
                    window=f"{p['nis_window_s']:g} s = {w} samples (nis_gate.nis_window_s), trailing",
                    band=f"{100 * lo:g} to {100 * hi:g} % (nis_gate.nis_band_low / nis_band_high)",
                    bound="chi-square 95 % quantile, 2 degrees of freedom",
                    session_health=f"{health:.4f} (equals the gate's health_score)",
                    lap_shares="below {below:.3f} inside {inside:.3f} above {above:.3f}".format(**lap_share),
                    thinning=f"every {keep[1] - keep[0]}th sample; outside the gate mask blank")
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols, m)
    render(out)
    return {"health": health, "window_samples": w, "lap_shares": lap_share, "n": int(keep.size)}


def render(out=None):
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, raw = common.read_csv(f"{data_dir}/{SCRIPT}.csv")
    s = [common.num(x) for x in raw["s_m"]]
    lo, hi = (float(x) for x in re.findall(r"(\d+(?:\.\d+)?)", meta["band"])[:2])
    win = meta["window"].split(" = ")[0]
    share = dict(zip(*[iter(meta["lap_shares"].split())] * 2))
    t = ["% generated by tools/thesis_figures/fig_nis_health.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}.csv", "tblNis"),
         f"\\begin{{axis}}[name=nis, thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT}, "
         "unbounded coords=jump,\n"
         f"  xmin={s[0]:.0f}, xmax={s[-1]:.0f}, ymin=0, xlabel={{$s$ in m}},\n"
         f"  ylabel={{NIS exceedance in \\% ({win} window)}},\n"
         "  xticklabel style={/pgf/number format/.cd, fixed, set thousands separator={}},\n"
         "  legend style={font=\\small, cells={anchor=west}, at={(0.99,0.97)}, anchor=north east}]\n",
         f"\\fill[{IN_BAND}] ({{axis cs:{s[0]:.0f},{lo:g}}}) rectangle ({{axis cs:{s[-1]:.0f},{hi:g}}});\n",
         f"\\addlegendimage{{area legend, draw=none, fill={IN_BAND}}}"
         f"\\addlegendentry{{band {lo:g} to {hi:g}\\,\\%}}\n",
         "\\addplot[black, thin] table[x=s_m, y=share_pct]{\\tblNis};\n\\addlegendentry{window share}\n",
         "\\end{axis}\n",
         # one stacked bar: share of this lap's gate-mask windows below, inside
         # and above the band (CSV header lap_shares)
         f"\\begin{{axis}}[at={{(nis.east)}}, anchor=west, xshift=0.5cm, scale only axis, width={BAR_WIDTH}, "
         f"height={PANEL_HEIGHT},\n  xmin=0, xmax=1, ymin=0, ymax=100, xtick=\\empty, ytick=\\empty, "
         "clip=false, title={this lap}]\n"]
    y = 0.0
    for part, fill in BAR_PARTS:
        h = 100.0 * float(share[part])
        t.append(f"\\fill[{fill}, draw=black, thin] (axis cs:0,{y:.2f}) rectangle (axis cs:1,{y + h:.2f});\n"
                 f"\\node[anchor=west, font=\\small] at (axis cs:1,{y + h / 2:.2f}) {{{part} {h:.1f}\\,\\%}};\n")
        y += h
    t.append("\\end{axis}\n\\end{tikzpicture}\n")
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", "".join(t))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap):
        ap.error("--outing and --lap are required unless --render-only")
    print(build(a.outing, a.lap, a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
