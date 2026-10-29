# fig:corner_phases -- how corner_analysis locates bracket, turn-in, apex,
# unwind point and the five phases in the logged signals, for one corner of
# one lap. Three stacked panels over lap distance: throttle with the
# lift-off level, |steering-wheel angle| and |a_y| with the bracket's
# opening and closing thresholds, both smoothed as the bracket logic does
# (moving average over smoothing_window_samples of the lap slice, a_y first
# interpolated onto the steering grid). Event lines L / T / A / U / E are the
# lap's own detection (_analyse_lap); the shading is the stored phase
# bounds (canonical medians, _realize_canonical_corners). The figure is
# only built for a corner where the two agree within MATCH_TOL_M, so lines
# and shading show the same thing. The apex phase is drawn with the width
# the statistics read (apex_instant_slice, +/- apex_half_window_samples on
# the analysis grid). Values from config/channels.json corner_detection.
# Run: python -m tools.thesis_figures.fig_corner_phases --outing 26 --lap 16
#      --corner 8 [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_corner_phases"
MATCH_TOL_M = 2.0       # selection rule (review ruling 2026-10-28): own events vs stored bounds
X_PAD_M = 20.0          # x range: braking start - 20 m to bracket end + 20 m
PANEL_HEIGHT = "0.17\\textwidth"
AXIS_WIDTH = "0.86\\textwidth"
YLABEL_SHIFT = "3.4em"
EVENTS = ("L", "T", "A", "U", "E")
_BOUND_KEYS = (("entry_1_brake", 0), ("entry_2_turnin", 0), ("apex_3", 0), ("exit_4", 1), ("exit_5", 1))


def _thin(n, cap=common.SCATTER_CAP):
    # every k-th sample, so a series stays under the pdflatex point cap
    return np.arange(0, n, max(1, int(np.ceil(n / cap))))


def corner_data(session, lap, corner):
    """Signals, own events, stored bounds and the bracket diagnostics of
    one corner instance; distances via the guarded lap-distance map."""
    from modules import corner_analysis as ca
    from modules.stability_analysis import (_interp_lap_distance_guarded, _normalize_lap_distance_to_metres,
                                            apex_instant_slice, prepare_vehicle_state)
    ch = session["parsed"]["channels"]
    cfg = ca._load_config()
    cd = cfg["corner_detection"]
    ld = ch["lap_distance"]
    ldm = _normalize_lap_distance_to_metres(ld["data"], ld.get("unit_raw"))

    def S(tt):
        return _interp_lap_distance_guarded(np.asarray(tt, float), ld["time"], ldm)

    inst = [c for c in session["parsed"]["corners"] if c["lap_number"] == lap and c.get("stable_corner_id") == corner]
    if not inst:
        raise common.Refused(f"lap {lap} corner {corner}: no such corner instance")
    inst = inst[0]
    lap_row = [l for l in session["parsed"]["laps"] if l["lap_number"] == lap][0]
    own = {c["corner_number"]: c for c in ca._analyse_lap(lap_row, ch, cd, cfg["corner_speed_thresholds"])}
    own = own.get(inst.get("corner_number"))
    if own is None:
        raise common.Refused(f"lap {lap} corner {corner}: no own bracket on this lap (canonical quiet instance)")
    stored = [float(S(inst["segments"][k][i])) for k, i in _BOUND_KEYS]
    events = [float(S(own["segments"][k][i])) for k, i in _BOUND_KEYS]
    mismatch = [e - b for e, b in zip(events, stored)]
    if max(abs(m) for m in mismatch) > MATCH_TOL_M:
        raise common.Refused(f"lap {lap} corner {corner}: own events differ from the stored bounds by up to "
                             f"{max(abs(m) for m in mismatch):.1f} m (> {MATCH_TOL_M:g} m)")

    sw = cd["smoothing_window_samples"]
    t0, t1 = lap_row["start_time"], lap_row["end_time"]
    steer = ca._slice_channel(ch["log_asteer"], t0, t1)
    lat = ca._slice_channel(ch["log_acc_y"], t0, t1)
    thr = ca._slice_channel(ch["ecu_aps"], t0, t1)
    st_sm = ca._smooth(steer["data"], sw)
    ay_sm = np.abs(ca._smooth(np.interp(steer["time"], lat["time"], lat["data"]), sw))

    # which signal opened the bracket: the bracket logic's own opening sample
    brackets = ca._bracket_corners_by_steering(steer, cd, lat)
    t_open = own["segments"]["entry_2_turnin"][0] - steer["abs_start"]
    b0, b1 = min(brackets, key=lambda b: abs(steer["time"][b[0]] - t_open))
    opened = [name for name, ok in (("steering", abs(st_sm[b0]) > cd["steering_entry_threshold_deg"]),
                                    ("a_y", ay_sm[b0] > cd["ay_entry_threshold_g"])) if ok]
    # post-apex steering peak exactly as _build_corner: the bracket slice
    # smoothed on its own, half_idx 0 = the early-exit fallback (B19)
    bracket_steer = np.abs(ca._smooth(steer["data"][b0:b1 + 1], sw))
    post = steer["time"][b0:b1 + 1] > own["apex_time"] - steer["abs_start"]
    peak = float(np.max(bracket_steer[post]))
    fallback = bool(np.argmax(bracket_steer[post] <= peak * cd["steering_unwind_fraction"]) == 0)

    # apex phase as the statistics read it, on the analysis grid
    state = prepare_vehicle_state(ch, session["params"])
    sl = apex_instant_slice(state["time"], inst["segments"]["apex_3"][0],
                            session["params"]["stability_estimation"]["apex_half_window_samples"])
    apex_band = (float(S(state["time"][sl.start])), float(S(state["time"][sl.stop - 1])))

    lo, hi = stored[0] - X_PAD_M, stored[4] + X_PAD_M
    s_st = S(steer["time"] + steer["abs_start"])
    s_th = S(thr["time"] + thr["abs_start"])
    m_st = np.flatnonzero((s_st >= lo) & (s_st <= hi))
    m_th = np.flatnonzero((s_th >= lo) & (s_th <= hi))
    m_st, m_th = m_st[_thin(m_st.size)], m_th[_thin(m_th.size)]
    seg_t = {k: inst["segments"][k] for k in common.PHASE_LABELS}
    return {
        "steer": {"s_m": s_st[m_st], "steer_abs_deg": np.abs(st_sm[m_st]), "ay_abs_g": ay_sm[m_st]},
        "throttle": {"s_m": s_th[m_th], "throttle_pct": thr["data"][m_th]},
        "events": {"event": list(EVENTS), "s_m": events, "stored_bound_m": stored, "mismatch_m": mismatch},
        "phases": {"phase_key": list(common.PHASE_LABELS), "phase_label": list(common.PHASE_LABELS.values()),
                   "s_start_m": [stored[0], stored[1], apex_band[0], stored[2], stored[3]],
                   "s_end_m": [stored[1], stored[2], apex_band[1], stored[3], stored[4]],
                   # the apex instant has no duration; its phase is the slice width
                   "t_s": [(seg_t[k][1] - seg_t[k][0]) if k != "apex_3" else None for k in seg_t]},
        "lo": lo, "hi": hi, "opened_by": " and ".join(opened) or "-", "peak_post_deg": peak,
        "early_exit_fallback": fallback, "apex_ay_g": inst.get("apex_lateral_g"), "cd": cd,
        "apex_band_s": float(state["time"][sl.stop - 1] - state["time"][sl.start]),
    }


def build(outing, lap, corner, tag="dev", out=None):
    session = common.load_session(outing)
    d = corner_data(session, lap, corner)
    d["phases"]["t_s"] = [d["apex_band_s"] if v is None else v for v in d["phases"]["t_s"]]
    cd = d["cd"]
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    m = common.meta(SCRIPT, [outing], tag, session_label=label, lap=lap, corner=corner,
                    x_range_m=f"{d['lo']:.1f} to {d['hi']:.1f}",
                    smoothing=f"moving average over {cd['smoothing_window_samples']} logged samples of the lap "
                              "slice; a_y interpolated onto the steering grid first (bracket logic)",
                    thresholds=f"open {cd['steering_entry_threshold_deg']:g} deg / {cd['ay_entry_threshold_g']:g} g; "
                               f"close {cd['steering_exit_threshold_deg']:g} deg / {cd['ay_exit_threshold_g']:g} g; "
                               f"lift-off {cd['brake_throttle_max_pct']:g} %; unwind fraction "
                               f"{cd['steering_unwind_fraction']:g}",
                    opened_by=d["opened_by"], peak_post_apex_steer_deg=f"{d['peak_post_deg']:.2f}",
                    half_peak_deg=f"{d['peak_post_deg'] * cd['steering_unwind_fraction']:.2f}",
                    apex_ay_g=f"{d['apex_ay_g']:.3f}", early_exit_fallback=d["early_exit_fallback"],
                    events="own detection of this lap; phases = stored bounds (match within "
                           f"{MATCH_TOL_M:g} m); apex phase +/- apex_half_window_samples on the analysis grid")
    common.write_csv(f"{data_dir}/{SCRIPT}_steer.csv", d["steer"], m)
    common.write_csv(f"{data_dir}/{SCRIPT}_throttle.csv", d["throttle"], m)
    common.write_csv(f"{data_dir}/{SCRIPT}_events.csv", d["events"], m)
    common.write_csv(f"{data_dir}/{SCRIPT}_phases.csv", d["phases"], m)
    render(out)
    return {k: v for k, v in d.items() if k not in ("steer", "throttle", "cd")}


def render(out=None):
    """The .tex from the stored CSVs only; threshold levels from the CSV
    header, so a render never mixes a new config with old traces."""
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, _s = common.read_csv(f"{data_dir}/{SCRIPT}_steer.csv")
    _m, ev = common.read_csv(f"{data_dir}/{SCRIPT}_events.csv")
    _m, ph = common.read_csv(f"{data_dir}/{SCRIPT}_phases.csv")
    th = [float(x) for x in re.findall(r"\d+(?:\.\d+)?", meta["thresholds"])]
    lo, hi = (float(x) for x in meta["x_range_m"].split(" to "))
    events = dict(zip(ev["event"], (float(x) for x in ev["s_m"])))
    phases = list(zip(ph["phase_label"], (float(x) for x in ph["s_start_m"]), (float(x) for x in ph["s_end_m"])))
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex",
                     _tex(prefix, lo, hi, th, float(meta["half_peak_deg"]), events, phases))


def _label_nudge(xs, min_gap):
    # anchors for labels closer than min_gap: the left one leans left, the
    # right one right, so two marks a few metres apart stay readable
    anchors = ["south"] * len(xs)
    order = np.argsort(xs)
    for a, b in zip(order[:-1], order[1:]):
        if xs[b] - xs[a] < min_gap:
            anchors[a], anchors[b] = "south east", "south west"
    return anchors


def _tex(prefix, lo, hi, th, half_peak, events, phases):
    open_deg, open_g, close_deg, close_g, lift = th[:5]
    min_gap = 0.03 * (hi - lo)
    panels = [("throttle in \\%", "tblThr", "throttle_pct", (-5, 105), [(lift, "auxGray, thin")]),
              ("$|\\delta_\\mathrm{SW}|$ in deg", "tblSteer", "steer_abs_deg", None,
               [(open_deg, "auxGray, thin"), (close_deg, "auxGray, thin, dashed")]),
              ("$|a_y|$ in g", "tblSteer", "ay_abs_g", None, [(open_g, "auxGray, thin"), (close_g, "auxGray, thin, dashed")])]
    t = ["% generated by tools/thesis_figures/fig_corner_phases.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n", common.table_read(prefix, f"{SCRIPT}_throttle.csv", "tblThr"),
         common.table_read(prefix, f"{SCRIPT}_steer.csv", "tblSteer")]
    t.append("\\begin{groupplot}[group style={group size=1 by 3, vertical sep=0.3cm, xlabels at=edge bottom,\n"
             "    xticklabels at=edge bottom},\n"
             f"  thesis axis, scale only axis, width={AXIS_WIDTH}, height={PANEL_HEIGHT}, unbounded coords=jump,\n"
             f"  xmin={lo:.1f}, xmax={hi:.1f}, xlabel={{$s$ in m}},\n"
             "  xticklabel style={/pgf/number format/.cd, fixed, set thousands separator={}},\n"
             "  yticklabel style={/pgf/number format/fixed},\n"
             f"  every axis y label/.style={{at={{(rel axis cs:0,0.5)}}, xshift=-{YLABEL_SHIFT}, rotate=90, "
             "anchor=center}]\n")
    def vline(s):
        # from the bottom to the top of the axis box, whatever the y range
        return f"({{axis cs:{s:.2f},0}} |- {{rel axis cs:0,0}}) -- ({{axis cs:{s:.2f},0}} |- {{rel axis cs:0,1}})"

    for k, (ylabel, tbl, col, yr, hl) in enumerate(panels):
        rng = f", ymin={yr[0]}, ymax={yr[1]}" if yr else ", ymin=0, enlarge y limits={upper, value=0.08}"
        t.append(f"\\nextgroupplot[ylabel={{{ylabel}}}{rng}" + (", clip=false" if k == 0 else "") + "]\n")
        # phase bands first (behind the traces); the apex band over T / E1
        for j, (lab, s0, s1) in enumerate(phases):
            shade = "auxGray!30" if lab == "apex" else ("auxGray!14" if j % 2 == 0 else "auxGray!5")
            t.append(f"\\fill[{shade}] ({{axis cs:{s0:.2f},0}} |- {{rel axis cs:0,0}}) rectangle "
                     f"({{axis cs:{s1:.2f},0}} |- {{rel axis cs:0,1}});\n")
        for val, style in hl:
            t.append(f"\\addplot[{style}, forget plot] coordinates {{({lo:.1f},{val:g}) ({hi:.1f},{val:g})}};\n")
        if col == "steer_abs_deg":
            u = events["U"]
            w = 0.04 * (hi - lo)
            t.append(f"\\addplot[black, thick, forget plot] coordinates {{({u - w:.2f},{half_peak:.3f}) "
                     f"({u + w:.2f},{half_peak:.3f})}};\n")
        t.append(f"\\addplot[black, thick, forget plot] table[x=s_m, y={col}]{{\\{tbl}}};\n")
        for s in events.values():
            t.append(f"\\draw[black, thin, densely dashed] {vline(s)};\n")
        if k == 0:
            names = list(events)
            xs = [events[n] for n in names]
            for n, x, a in zip(names, xs, _label_nudge(xs, min_gap)):
                t.append(f"\\node[anchor={a}, font=\\small, inner sep=1pt, yshift=1.3em] at "
                         f"({{axis cs:{x:.2f},0}} |- {{rel axis cs:0,1}}) {{{n}}};\n")
            labs = [(common.PHASE_ABBR[lab], 0.5 * (s0 + s1)) for lab, s0, s1 in phases]
            for (abbr, x), a in zip(labs, _label_nudge([x for _a, x in labs], min_gap)):
                t.append(f"\\node[anchor={a}, font=\\scriptsize, text=auxGray!80!black, inner sep=1pt] at "
                         f"({{axis cs:{x:.2f},0}} |- {{rel axis cs:0,1}}) {{{abbr}}};\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--lap", type=int)
    ap.add_argument("--corner", type=int, help="stable corner id of that session")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSVs only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.lap, a.corner):
        ap.error("--outing, --lap and --corner are required unless --render-only")
    print(build(a.outing, a.lap, a.corner, a.tag, a.out))


if __name__ == "__main__":
    main()
