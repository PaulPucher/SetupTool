# Matplotlib figure rendering for exports -- display only, draws the arrays
# it's given. Callers: the trace dialog's export buttons and
# diagnostics/inspect_step2_chair_plots.py, so both produce identical
# figures. render_* is the public API; _draw_* are building blocks.
# Lap = colour (plot_style.lap_styles); one quantity per panel, front and
# rear always separate (laps carry cs_f/cs_r/ls_f/ls_r).

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from core import plot_style as ps

CM_PER_INCH = 2.54


def _apply_theme(fig, axes, theme):
    fig.patch.set_facecolor(theme["bg"])
    for ax in axes:
        ax.set_facecolor(theme["bg"])
        ax.tick_params(colors=theme["text"], labelsize=ps.PRINT_FONT_SIZE_PT)
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        ax.xaxis.label.set_color(theme["text"])
        ax.yaxis.label.set_color(theme["text"])
        ax.title.set_color(theme["text"])
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])


def _legend(ax, theme, font_scale=0.85, handles=None, labels=None, **kwargs):
    if handles is not None:
        leg = ax.legend(handles, labels, fontsize=ps.PRINT_FONT_SIZE_PT * font_scale, framealpha=0.85,
                         handlelength=1.3, labelspacing=0.3, borderpad=0.35, **kwargs)
    else:
        leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * font_scale, framealpha=0.85,
                         handlelength=1.3, labelspacing=0.3, borderpad=0.35, **kwargs)
    if leg is not None:
        leg.get_frame().set_facecolor(theme["bg"])
        leg.get_frame().set_edgecolor(theme["text_muted"])
        for text in leg.get_texts():
            text.set_color(theme["text"])
    return leg


def _side_legend_axes(fig, gs_cell, width_ratios=(5, 1.5)):
    """Split gs_cell into data axes (left) and a legend axes (right) -- legends
    never inside the data. A bbox_to_anchor legend can collapse sibling axes
    under constrained_layout (see _tyre_curve_axes).
    """
    sub = gs_cell.subgridspec(1, 2, width_ratios=width_ratios, wspace=0.06)
    ax = fig.add_subplot(sub[0, 0])
    legend_ax = fig.add_subplot(sub[0, 1])
    return ax, legend_ax


def _draw_side_legend(ax, legend_ax, theme, font_scale=0.7):
    """Fill legend_ax from ax's handles; ax itself never gets a legend."""
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.set_facecolor(theme["bg"])
    legend_ax.axis("off")
    if handles:
        _legend(legend_ax, theme, handles=handles, labels=labels, loc="center",
                ncol=1, font_scale=font_scale, frameon=False)


def _tyre_curve_axes(fig, gs_row):
    """Legend row under each of the two tyre-curve axes in gs_row.

    A bbox_to_anchor legend hanging below the axes made constrained_layout's
    column solve infeasible (row-spanning axes above) -> tyre panels collapsed
    to ~0 width with only a warning. A real GridSpec cell behaves like any
    other axes. Wide legends (ncol=6) in both columns caused the same
    collapse -> low ncol and tighter padding in render_corner_figure.
    """
    sub = gs_row.subgridspec(2, 2, height_ratios=[6, 1.6], hspace=0.03, wspace=0.03)
    ax_f = fig.add_subplot(sub[0, 0])
    ax_r = fig.add_subplot(sub[0, 1])
    legend_f = fig.add_subplot(sub[1, 0])
    legend_r = fig.add_subplot(sub[1, 1])
    return ax_f, ax_r, legend_f, legend_r


def _lap_pen_kwargs(lap, styles):
    # colour from the lap's style, one width for all laps; dash only past 10 laps
    style = styles[lap["lap_number"]]
    linestyle = "--" if style["dash"] == "dash" else "-"
    return dict(color=style["color"], linewidth=ps.PRINT_LAP_WIDTH, linestyle=linestyle)


def _add_threshold(ax, value, kind, label):
    # thresholds grey, by style: strong dashed, moderate dotted, unstable dash-dot
    style_map = {"strong": "--", "moderate": ":", "unstable": "-."}
    ax.axhline(value, color=ps.THRESHOLD_GREY, linestyle=style_map[kind], linewidth=1.0, label=label)


def _draw_speed_panel(ax, laps, styles, theme, legend_ax):
    for lap in laps:
        ax.plot(lap["s"], lap["v_kmh"], label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Speed (km/h)")
    _draw_side_legend(ax, legend_ax, theme)


def _draw_stability_panel(ax, laps, styles, thresholds, theme, legend_ax):
    for lap in laps:
        ax.plot(lap["s"], lap["stab"], label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
    _add_threshold(ax, thresholds["stab"], "unstable", "Unstable below this")
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Stability (Nm/deg)")
    _draw_side_legend(ax, legend_ax, theme)


def _draw_ratio_panel(ax, laps, styles, theme, value_key, ylabel, legend_ax,
                       strong=None, moderate=None, zero_line=False):
    # Front/rear CS and LS panels, one quantity each. LS: no thresholds
    # (strong/moderate None) but a dotted y = 0 line -- zero = no grip on the
    # LS scale.
    for lap in laps:
        val = lap.get(value_key)
        if val is None:
            continue
        ax.plot(lap["s"], val, label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
    if strong is not None:
        _add_threshold(ax, strong, "strong", "Strong")
    if moderate is not None:
        _add_threshold(ax, moderate, "moderate", "Moderate")
    if zero_line:
        ax.axhline(0.0, color=ps.THRESHOLD_GREY, linestyle=":", linewidth=1.0, label="Zero")
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel(ylabel)
    _draw_side_legend(ax, legend_ax, theme)


def _draw_track_map_panel(ax, track_map, corner_label, theme, compact=False):
    # compact (inset): no legend -- lap colours already in the other panels,
    # ring meaning in the caption
    if track_map is None:
        # no GPS -> geometry None -> placeholder
        ax.text(0.5, 0.5, "No GPS data", ha="center", va="center",
                color=theme["text_muted"], transform=ax.transAxes,
                fontsize=ps.PRINT_FONT_SIZE_PT * (0.6 if compact else 1.0))
        ax.set_xticks([])
        ax.set_yticks([])
        if not compact:
            ax.set_title("Track map")
        return
    lap_x, lap_y = track_map["lap_xy"]
    ax.plot(lap_x, lap_y, color=ps.TRACK_BG_COLOR, linewidth=1.0, label="Lap trace")
    # one bracket polyline per checked lap in its colour (each lap drives its
    # own line); grey outline = analysed lap, context only
    for entry in track_map.get("brackets_by_lap", []):
        bx, by = entry["xy"]
        linestyle = "--" if entry["dash"] == "dash" else "-"
        ax.plot(bx, by, color=entry["color"], linewidth=(1.2 if compact else 2.0), linestyle=linestyle,
                label=f"Lap {entry['lap_number']} bracket")
    # window rings: hollow black, front solid, rear dotted
    ring_lw = 0.9 if compact else 1.2
    if track_map.get("window_f_xy") is not None:
        wx, wy = track_map["window_f_xy"]
        ax.plot(wx, wy, marker='o', markersize=(4 if compact else 6), markerfacecolor='none',
                markeredgecolor=ps.WINDOW_RING_COLOR, markeredgewidth=ring_lw,
                linestyle='-', color=ps.WINDOW_RING_COLOR, linewidth=1.0, label="Front window")
    if track_map.get("window_r_xy") is not None:
        wx, wy = track_map["window_r_xy"]
        ax.plot(wx, wy, marker='o', markersize=(4 if compact else 6), markerfacecolor='none',
                markeredgecolor=ps.WINDOW_RING_COLOR, markeredgewidth=ring_lw,
                linestyle=':', color=ps.WINDOW_RING_COLOR, linewidth=1.0, label="Rear window")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    if not compact:
        ax.set_title("Track map")
        _legend(ax, theme, loc="upper right", ncol=1, font_scale=0.7)


def _corner_sample_bounds(curve, margin=0.15):
    """Bounding box of this corner's samples (clean + kerb, all checked laps)
    + margin -- the session cloud doesn't drive the zoom.
    Returns (xlim, ylim), or (None, None) -> matplotlib autoscale.
    """
    xs, ys = [], []
    for entry in curve.get("corner_by_lap", []):
        for key in ("clean_xy", "kerb_xy"):
            xy = entry.get(key)
            if xy is not None:
                xs.append(xy[0])
                ys.append(xy[1])
    if not xs:
        return None, None
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    x = x[np.isfinite(x)]
    y = y[np.isfinite(y)]
    if x.size == 0 or y.size == 0:
        return None, None
    x_lo, x_hi = float(x.min()), float(x.max())
    y_lo, y_hi = float(y.min()), float(y.max())
    x_pad = (x_hi - x_lo) * margin or 1.0
    y_pad = (y_hi - y_lo) * margin or 1.0
    return (x_lo - x_pad, x_hi + x_pad), (y_lo - y_pad, y_hi + y_pad)


def _draw_tyre_curve_panel(ax, axle_label, curve, theme, legend_ax):
    # draw order: session cloud (light, first), lap samples, window rings
    # (last, on top); thin fitted/tangent/reference lines; legend in its own
    # cell below (_tyre_curve_axes)
    sx, sy, skerb = curve["session_xy"]
    if skerb is not None:
        clean = ~skerb
        ax.scatter(sx[clean], sy[clean], s=ps.SESSION_CLOUD_SIZE_PRINT, color=ps.SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Session", zorder=1)
    else:
        ax.scatter(sx, sy, s=ps.SESSION_CLOUD_SIZE_PRINT, color=ps.SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Session", zorder=1)

    # lap colour: filled = clean, hollow = kerb-flagged
    for entry in curve.get("corner_by_lap", []):
        color = entry["color"]
        if entry.get("clean_xy") is not None:
            cx, cy = entry["clean_xy"]
            ax.scatter(cx, cy, s=ps.LAP_SAMPLE_SIZE_PRINT, color=color, alpha=ps.LAP_SAMPLE_ALPHA,
                       label=f"Lap {entry['lap_number']}", zorder=2)
        if entry.get("kerb_xy") is not None:
            kx, ky = entry["kerb_xy"]
            ax.scatter(kx, ky, s=ps.LAP_SAMPLE_SIZE_PRINT, facecolors='none', edgecolors=color,
                       linewidths=1.0, alpha=ps.LAP_SAMPLE_ALPHA, zorder=2)

    if curve.get("linear_ref_line") is not None:
        lx, ly = curve["linear_ref_line"]
        ax.plot(lx, ly, color=theme["text_muted"], linewidth=ps.TYRE_LINE_WIDTH_PRINT,
                linestyle="-", label="Linear reference", zorder=3)

    if curve.get("fitted_line") is not None:
        fx, fy, flabel = curve["fitted_line"]
        ax.plot(fx, fy, color=ps.FITTED_CURVE_COLOR, linewidth=ps.TYRE_LINE_WIDTH_PRINT, label=flabel, zorder=3)

    if curve.get("tangent_line") is not None:
        tx, ty, tlabel = curve["tangent_line"]
        # theme text colour, readable on both themes
        ax.plot(tx, ty, color=theme["text"], linewidth=ps.TYRE_LINE_WIDTH_PRINT,
                linestyle="--", label=tlabel, zorder=3)

    if curve.get("window_xy") is not None:
        wx, wy = curve["window_xy"]
        ax.scatter(wx, wy, s=ps.WINDOW_RING_SIZE_PRINT, facecolors='none',
                   edgecolors=ps.WINDOW_RING_COLOR, linewidths=1.2, label="Estimation window", zorder=5)

    ax.axhline(0.0, color=theme["text_muted"], linewidth=0.6)
    ax.axvline(0.0, color=theme["text_muted"], linewidth=0.6)

    # zoom to the corner samples (cloud just clipped); set after all artists
    # so autoscale doesn't override
    xlim, ylim = _corner_sample_bounds(curve)
    if xlim is not None:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
    # square box, not equal data scale (reference slope would be unreadable)
    ax.set_box_aspect(1.0)

    ax.set_xlabel("Slip angle (deg)", fontsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    # kN: 6-digit N tick labels ate the width the panel needs (>= 7 cm)
    import matplotlib.ticker as mticker
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, pos: f"{v / 1000:.0f}"))
    ax.set_ylabel("Fy (kN)", fontsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    ax.tick_params(labelsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    ax.set_title(f"{axle_label} tyre curve", fontsize=ps.PRINT_FONT_SIZE_PT)

    # ncol 2 -- wider legends in both columns broke the layout solve
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.set_facecolor(theme["bg"])
    legend_ax.axis("off")
    if handles:
        _legend(legend_ax, theme, handles=handles, labels=labels, loc="center",
                ncol=2, font_scale=0.55, frameon=False)


def _new_figure(width_cm, height_cm, theme):
    plt.rcParams["font.family"] = ps.PRINT_FONT_FAMILY
    plt.rcParams["font.size"] = ps.PRINT_FONT_SIZE_PT
    # constrained_layout: fixed hspace/wspace couldn't fit the long axis
    # labels at 16 cm
    fig = plt.figure(figsize=(width_cm / CM_PER_INCH, height_cm / CM_PER_INCH),
                      constrained_layout=True)
    return fig


def render_corner_figure(corner_label, laps, thresholds, tyre_curves, track_map, theme=ps.PRINT):
    """Chair-style composition: speed, front CS, rear CS vs s (with thresholds),
    a narrow track-map row (compact: no title/legend), front/rear tyre curves
    (linear reference, fitted model, tangent). Stability/LS are in
    render_verdict_traces_figure.
    Side legend strips per trace panel; tyre curves keep theirs below.
    laps = exactly the checked set; colours assigned here via lap_styles,
    same as the verdict figure.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_CORNER, theme)
    # Small h_pad: it applies at every nested boundary and shrank the tyre
    # panels badly. Outer margins via rect instead, once: bottom 0.036 (~8 mm),
    # left 0.035 (~5.6 mm) so the y label isn't clipped.
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.04, wspace=0.0, hspace=0.02, rect=(0.07, 0.036, 1.0, 1.0))
    gs = fig.add_gridspec(5, 2, height_ratios=[2.4, 2.4, 2.4, 0.9, 5.4])
    ax_speed, legend_speed = _side_legend_axes(fig, gs[0, :])
    ax_csf, legend_csf = _side_legend_axes(fig, gs[1, :])
    ax_csr, legend_csr = _side_legend_axes(fig, gs[2, :])
    ax_map = fig.add_subplot(gs[3, :])
    ax_tyre_f, ax_tyre_r, legend_f, legend_r = _tyre_curve_axes(fig, gs[4, :])

    _draw_speed_panel(ax_speed, laps, styles, theme, legend_speed)
    _draw_ratio_panel(ax_csf, laps, styles, theme, "cs_f", "Front CS ratio", legend_csf,
                       strong=thresholds["strong_csf"], moderate=thresholds["moderate_csf"])
    _draw_ratio_panel(ax_csr, laps, styles, theme, "cs_r", "Rear CS ratio", legend_csr,
                       strong=thresholds["strong_csr"], moderate=thresholds["moderate_csr"])
    _draw_track_map_panel(ax_map, track_map, corner_label, theme, compact=True)
    _draw_tyre_curve_panel(ax_tyre_f, "Front", tyre_curves["front"], theme, legend_f)
    _draw_tyre_curve_panel(ax_tyre_r, "Rear", tyre_curves["rear"], theme, legend_r)

    # shared x for speed/CS; track map is plan view, excluded
    xlim = ax_speed.get_xlim()
    for ax in (ax_csf, ax_csr):
        ax.set_xlim(xlim)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, [ax_speed, ax_csf, ax_csr, ax_map, ax_tyre_f, ax_tyre_r], theme)
    # _apply_theme reset the smaller tyre-panel tick font -> restore after it
    for ax in (ax_tyre_f, ax_tyre_r):
        ax.tick_params(labelsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    return fig


def _verdict_panel_set(fig, gs, panel_specs):
    axes = []
    for row, (_key, draw) in enumerate(panel_specs):
        ax, legend_ax = _side_legend_axes(fig, gs[row, 0])
        draw(ax, legend_ax)
        axes.append(ax)
    xlim = axes[0].get_xlim()
    for ax in axes[1:]:
        ax.set_xlim(xlim)
    return axes


def render_verdict_traces_figure(corner_label, laps, thresholds, theme=ps.PRINT):
    """Extension stack, one quantity per panel, shared x: speed, stability,
    front CS, rear CS, front LS, rear LS. Side legend per panel.
    Returns a list of figures -- currently one; six panels fit in 24 cm.
    Should they ever not: two _verdict_panel_set calls (speed/stab/CSf/CSr,
    speed/LSf/LSr), each its own _new_figure.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, theme)
    # left margin 0.035 (~5.6 mm) so the y label isn't clipped
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.03, wspace=0.0, hspace=0.02, rect=(0.035, 0.0, 1.0, 1.0))
    gs = fig.add_gridspec(6, 1)
    panel_specs = [
        ("speed", lambda ax, lax: _draw_speed_panel(ax, laps, styles, theme, lax)),
        ("stab", lambda ax, lax: _draw_stability_panel(ax, laps, styles, thresholds, theme, lax)),
        ("cs_f", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_f", "Front CS ratio", lax,
                                                    strong=thresholds["strong_csf"], moderate=thresholds["moderate_csf"])),
        ("cs_r", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_r", "Rear CS ratio", lax,
                                                    strong=thresholds["strong_csr"], moderate=thresholds["moderate_csr"])),
        ("ls_f", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "ls_f", "Front LS ratio", lax,
                                                    zero_line=True)),
        ("ls_r", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "ls_r", "Rear LS ratio", lax,
                                                    zero_line=True)),
    ]
    # autoscale ignores NaN -> masked LS panels came out narrower; x anchored
    # on the speed panel
    axes = _verdict_panel_set(fig, gs, panel_specs)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, axes, theme)
    return [fig]


def _draw_corner_bands(ax, corner_bands, label_above=False):
    # neutral grey bands in print -- verdict colours read as a traffic light on
    # paper; the corner label carries identity
    for start_s, end_s, cid in corner_bands:
        ax.axvspan(start_s, end_s, color="#999999", alpha=0.15, zorder=0)
        if label_above:
            ax.annotate(f"C{cid}", xy=((start_s + end_s) / 2.0, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(0, 2), textcoords="offset points", ha="center", va="bottom",
                        fontsize=ps.PRINT_FONT_SIZE_PT * 0.7, color="#666666")


def render_lap_figure(lap_label, laps, thresholds, corner_bands, theme=ps.PRINT):
    """Whole-lap export: speed, stability, front CS, rear CS vs s (no LS -- no
    lap-level reading). Corner bands in light grey, ids above the top panel.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, theme)
    # same left margin as the verdict figure
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.03, wspace=0.0, hspace=0.02, rect=(0.035, 0.0, 1.0, 1.0))
    gs = fig.add_gridspec(4, 1)
    panel_specs = [
        ("speed", lambda ax, lax: _draw_speed_panel(ax, laps, styles, theme, lax)),
        ("stab", lambda ax, lax: _draw_stability_panel(ax, laps, styles, thresholds, theme, lax)),
        ("cs_f", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_f", "Front CS ratio", lax,
                                                    strong=thresholds["strong_csf"], moderate=thresholds["moderate_csf"])),
        ("cs_r", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_r", "Rear CS ratio", lax,
                                                    strong=thresholds["strong_csr"], moderate=thresholds["moderate_csr"])),
    ]
    axes = _verdict_panel_set(fig, gs, panel_specs)
    for i, ax in enumerate(axes):
        _draw_corner_bands(ax, corner_bands, label_above=(i == 0))

    fig.suptitle(lap_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, axes, theme)
    return fig


def save_png(fig, path):
    """Fixed dpi -> same result, same bytes. Closes the figure."""
    fig.savefig(path, dpi=ps.PRINT_DPI, facecolor=fig.get_facecolor())
    plt.close(fig)
