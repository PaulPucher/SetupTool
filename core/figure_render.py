# Shared matplotlib figure renderer -- Tier C (UI/product), no Qt, no
# estimation logic. Pure display: every function here draws arrays it is
# handed, computing nothing about corners, brackets, windows, or slopes
# itself (that domain logic stays in the caller -- ui/views/corner_trace_
# dialog.py's export handlers, or diagnostics/inspect_step2_chair_plots.py
# -- same "pure display" convention corner_trace_dialog.py's own module
# docstring already states for the interactive pyqtgraph dialog).
#
# Two callers share every function below: the app's "Export figure"/
# "Export verdict traces" buttons (PRINT theme, one corner at a time) and
# diagnostics/inspect_step2_chair_plots.py's batch export (also PRINT
# theme, looped over corners/sources) -- so an app export and a
# diagnostics-script export of the same corner produce visually identical
# figures. Composition functions (render_*) are the only public API;
# _draw_* panel functions are composition building blocks, not meant to
# be called directly by either caller.
#
# Part B redesign (2026-09-01, trace-dialog work package + addendum): lap
# identity is carried by COLOUR (core/plot_style.lap_styles, assigned to
# the exported/checked set in ascending lap order). Each panel plots
# exactly ONE quantity (front and rear CS/LS ratio are separate panels,
# never one panel with two overlaid axle curves) -- "laps" dicts passed
# in carry per-axle arrays under distinct keys (cs_f/cs_r/ls_f/ls_r).
#
# Corrections batch (same day, after visual review): the bold/faint
# ("emphasised lap") line-width distinction is removed entirely -- every
# lap draws at the same width now, colour is the only identity carried by
# a trace line. render_corner_figure's layout was reworked (three-column
# squeeze made the tyre-curve panels unreadable) into a stacked layout
# with the tyre curves given a full-width row; the track map moved out of
# that row into either an inset-in-the-speed-panel or its own narrow row
# (track_map_style="inset"|"row" -- see that function's own docstring for
# which this session picked and why).

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
    """Split gs_cell into a data axes (left) and a dedicated legend axes
    (right) -- corrections round 3, item 1: "every trace panel gets its
    legend in a dedicated strip to the RIGHT of the axes", never drawn
    inside/over the data. Same dedicated-GridSpec-cell precedent as
    _tyre_curve_axes below (that one places the strip under the panel,
    this one beside it) and for the same reason: an inside-axes or
    bbox_to_anchor legend reads as "inside the axes" regardless of
    whether it happens to sit over empty space, and can collapse sibling
    axes under constrained_layout (see _tyre_curve_axes's own docstring).
    """
    sub = gs_cell.subgridspec(1, 2, width_ratios=width_ratios, wspace=0.06)
    ax = fig.add_subplot(sub[0, 0])
    legend_ax = fig.add_subplot(sub[0, 1])
    return ax, legend_ax


def _draw_side_legend(ax, legend_ax, theme, font_scale=0.7):
    """Populate legend_ax (from _side_legend_axes) from ax's own handles/
    labels -- ax itself never calls .legend(), so nothing is ever drawn
    inside its own bounds.
    """
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.set_facecolor(theme["bg"])
    legend_ax.axis("off")
    if handles:
        _legend(legend_ax, theme, handles=handles, labels=labels, loc="center",
                ncol=1, font_scale=font_scale, frameon=False)


def _tyre_curve_axes(fig, gs_row):
    """Nest a dedicated legend row under each of the two tyre-curve axes
    within `gs_row` (a 2-column GridSpec slice, e.g. gs[4, :]).

    Corrections batch, item 5: "legend outside the axes (below the
    panel), never over data" was first tried as ax.legend(loc="upper
    center", bbox_to_anchor=(0.5, -0.14)) -- a legend positioned outside
    its own axes' bounds via bbox_to_anchor. Rendering the real Dubai
    corner exposed a genuine matplotlib/constrained_layout interaction
    bug: a bbox_to_anchor legend hanging outside its axes makes the
    layout engine's per-column width solve infeasible when that axes
    shares its row with OTHER full-width axes above it (row-spanning
    axes force each column to reconcile widths across rows) -- the
    engine "solved" this by collapsing the tyre-curve axes to near-zero
    width (measured 0.03 of figure width, not the required 7cm) while
    silently emitting a "collapsed to zero" warning. A real, dedicated
    GridSpec cell for the legend (this function) is a normal part of the
    layout instead of an out-of-bounds decoration, and constrained_layout
    handles it exactly like any other axes.

    A second, smaller instance of the SAME failure mode reappeared once
    the legend axes existed but were fed a wide (ncol=6) legend on TWO
    side-by-side columns at once -- reproduced in isolation (both legend
    axes populated at once collapses; either alone does not), fixed by
    keeping ncol low (see _draw_tyre_curve_panel) and tightening this
    GridSpec's own padding directly (fig.get_layout_engine().set(...) in
    render_corner_figure) rather than leaving matplotlib's default
    spacing to compete with the legend's own width demand.
    """
    sub = gs_row.subgridspec(2, 2, height_ratios=[6, 1.6], hspace=0.03, wspace=0.03)
    ax_f = fig.add_subplot(sub[0, 0])
    ax_r = fig.add_subplot(sub[0, 1])
    legend_f = fig.add_subplot(sub[1, 0])
    legend_r = fig.add_subplot(sub[1, 1])
    return ax_f, ax_r, legend_f, legend_r


def _lap_pen_kwargs(lap, styles):
    # Corrections batch: colour comes from the lap's assigned style
    # (ascending-order palette position within the checked/exported set);
    # every lap draws at the SAME width now (no bold/faint distinction --
    # see plot_style.PRINT_LAP_WIDTH's own comment). Dash pattern only
    # differs once more than len(LAP_PALETTE) laps are checked at once.
    style = styles[lap["lap_number"]]
    linestyle = "--" if style["dash"] == "dash" else "-"
    return dict(color=style["color"], linewidth=ps.PRINT_LAP_WIDTH, linestyle=linestyle)


def _add_threshold(ax, value, kind, label):
    # Part B: every threshold line is the same neutral grey, distinguished
    # by STYLE only -- strong=dashed, moderate=dotted, unstable=dash-dot.
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
    # Shared by all four axle-ratio panels (Front/Rear CS, Front/Rear LS)
    # -- ONE quantity per panel, no solid/dashed axle encoding anywhere
    # (corrections batch item 3) -- front and rear are always separate
    # panels, matching the interactive dialog exactly. strong/moderate are
    # None for LS panels (LS_ratio has no classification thresholds,
    # PLAN.md STEP 3 -- display only). zero_line (corrections round 3,
    # item 4): LS panels only -- a dotted neutral-grey y=0 reference, LS_
    # ratio's own scale runs positive (linear) through zero (no grip) to
    # negative (already reported as a ratio, not raw stiffness, so zero
    # itself is a meaningful landmark CS's strong/moderate lines already
    # give CS panels but LS has no classification thresholds to supply).
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
    # compact=True (render_corner_figure's "inset" layout variant): the
    # panel is small (an inset inside the speed panel), so the legend is
    # dropped entirely rather than shrunk further -- lap colour is already
    # established by the speed/CS panels' own legends, and axle-window
    # identity (front solid / rear dotted ring) is stated in the
    # composition's own caption, not repeated here.
    if track_map is None:
        # No GPS channel on this outing -- corner_trace_dialog.py's
        # _render_track_map returns None in that case rather than an
        # empty geometry dict; render a plain placeholder instead of
        # crashing on a missing "lap_xy" key.
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
    # One bracket polyline PER CHECKED LAP, in that lap's own colour --
    # each lap drives a slightly different physical line through the
    # corner, so the bracket is genuinely that lap's own data (unlike the
    # grey background outline above, which is context only and stays
    # fixed to the analysed lap regardless of checked state, see
    # ui/views/corner_trace_dialog.py's _render_track_map docstring).
    for entry in track_map.get("brackets_by_lap", []):
        bx, by = entry["xy"]
        linestyle = "--" if entry["dash"] == "dash" else "-"
        ax.plot(bx, by, color=entry["color"], linewidth=(1.2 if compact else 2.0), linestyle=linestyle,
                label=f"Lap {entry['lap_number']} bracket")
    # Windows: hollow black rings -- front solid, rear dotted, so the two
    # remain distinguishable from each other without a second colour.
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
    """Bounding box of the CORNER samples only (clean + kerb-flagged,
    every checked lap), +margin fraction each side -- corrections round
    3, item 3: the tyre-curve axis range is fitted to what this corner's
    checked laps actually show, not to the much wider session cloud
    (context, drawn but no longer allowed to drive the zoom level).
    Returns (xlim, ylim), or (None, None) if no corner sample exists at
    all (falls back to matplotlib's own autoscale over whatever IS
    plotted, i.e. the session cloud, rather than crashing on empty data).
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
    # Corrections batch, item 5: session cloud very light/thin and drawn
    # FIRST (background); lap samples medium filled at partial alpha;
    # estimation-window rings largest, drawn LAST (on top). Fitted/
    # tangent/linear-reference lines are thin (TYRE_LINE_WIDTH_PRINT).
    # Legend moves OUTSIDE the axes, below the panel -- `legend_ax` is a
    # real, dedicated GridSpec cell for it (see _tyre_curve_axes's own
    # docstring for why, not a bbox_to_anchor overhang off `ax` itself).
    sx, sy, skerb = curve["session_xy"]
    if skerb is not None:
        clean = ~skerb
        ax.scatter(sx[clean], sy[clean], s=ps.SESSION_CLOUD_SIZE_PRINT, color=ps.SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Session", zorder=1)
    else:
        ax.scatter(sx, sy, s=ps.SESSION_CLOUD_SIZE_PRINT, color=ps.SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Session", zorder=1)

    # Corner samples in the LAP'S OWN colour (filled), kerb-flagged
    # samples of that same lap as a HOLLOW marker in the same colour --
    # ties the kerb-affected points back to which lap they came from
    # instead of one shared neutral "Kerb-flagged" cloud.
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
        # Bug fix, Part B: theme["text"] (black on PRINT, light on
        # INTERACTIVE), not a fixed constant -- see plot_style.py's own
        # note on the retired TANGENT_COLOR for why.
        ax.plot(tx, ty, color=theme["text"], linewidth=ps.TYRE_LINE_WIDTH_PRINT,
                linestyle="--", label=tlabel, zorder=3)

    if curve.get("window_xy") is not None:
        wx, wy = curve["window_xy"]
        ax.scatter(wx, wy, s=ps.WINDOW_RING_SIZE_PRINT, facecolors='none',
                   edgecolors=ps.WINDOW_RING_COLOR, linewidths=1.2, label="Estimation window", zorder=5)

    ax.axhline(0.0, color=theme["text_muted"], linewidth=0.6)
    ax.axvline(0.0, color=theme["text_muted"], linewidth=0.6)

    # Corrections round 3, item 3: zoom to the corner samples, not the
    # session cloud -- the cloud is context only, drawn above but simply
    # clipped at these axes bounds (its own array is untouched, this is
    # a view-range choice like every other display-only clip in this
    # file). set AFTER every artist is added so the explicit limits win
    # over matplotlib's own autoscale-to-everything-plotted default.
    xlim, ylim = _corner_sample_bounds(curve)
    if xlim is not None:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
    # Square panel, independent of the (very different) data ranges on
    # each axis -- box_aspect fixes the drawn BOX shape, not a 1:1 data
    # scale (which would make the reference slope unreadable, see the
    # "Equal-axis scaling deliberately OFF" note this panel's interactive
    # twin carries).
    ax.set_box_aspect(1.0)

    ax.set_xlabel("Slip angle (deg)", fontsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    # kN, not N: real Fy magnitudes here reach 5-6 digits ("-100000"),
    # whose tick-label width was the single largest remaining cost against
    # the "at least 7cm wide" floor once every other margin had already
    # been minimised (verified by rendering) -- kN tick labels stay 2-3
    # digits for the same data, a display-only axis choice, no numeric
    # value anywhere is altered.
    import matplotlib.ticker as mticker
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, pos: f"{v / 1000:.0f}"))
    ax.set_ylabel("Fy (kN)", fontsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    ax.tick_params(labelsize=ps.PRINT_FONT_SIZE_PT * 0.75)
    ax.set_title(f"{axle_label} tyre curve", fontsize=ps.PRINT_FONT_SIZE_PT)

    # ncol capped at 2 (not the "however many entries" ncol=6 originally
    # tried): with front AND rear both needing their own legend side by
    # side, a wide (ncol=6) legend on EITHER one was found, by rendering,
    # to make the whole figure's constrained_layout solve infeasible (see
    # _tyre_curve_axes's own docstring) -- ncol=2 reproduced no such
    # failure in the same isolation test and keeps the legend readable as
    # a short multi-row block rather than one wide row.
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.set_facecolor(theme["bg"])
    legend_ax.axis("off")
    if handles:
        _legend(legend_ax, theme, handles=handles, labels=labels, loc="center",
                ncol=2, font_scale=0.55, frameon=False)


def _new_figure(width_cm, height_cm, theme):
    plt.rcParams["font.family"] = ps.PRINT_FONT_FAMILY
    plt.rcParams["font.size"] = ps.PRINT_FONT_SIZE_PT
    # constrained_layout, not manual hspace/wspace/tight_layout: at a
    # fixed 16cm print width the y-axis label + tick text (e.g. "Lateral
    # force Fy (N)" next to "-30000") needs real per-figure space that a
    # single fixed hspace/wspace guess got wrong -- constrained_layout
    # solves that per-axes automatically, including reserving room for
    # legends placed with bbox_to_anchor (verified by rendering: an
    # outside-axes legend does not get clipped).
    fig = plt.figure(figsize=(width_cm / CM_PER_INCH, height_cm / CM_PER_INCH),
                      constrained_layout=True)
    return fig


def render_corner_figure(corner_label, laps, thresholds, tyre_curves, track_map, theme=ps.PRINT):
    """The chair-style composition: speed vs s, front CS ratio vs s, rear
    CS ratio vs s (each with its own thresholds), the track map, and
    front/rear tyre curves (linear reference, fitted model, tangent).
    Stability/LS are deliberately NOT here -- SetupTool's own extension
    panels, kept out of this composition so exported thesis figures stay
    uncrowded (see render_verdict_traces_figure for those).

    Follow-up (round 3 corrections, item 2): the track map -- dropped
    from this export by round 3's own item 2 to make room for the new
    side-legend columns -- is back, as a narrow full-width row between
    the rear-CS panel and the tyre curves. `compact=True` (see
    _draw_track_map_panel): no title, no legend, smaller markers -- lap
    colour is already established by the speed/CS panels' own legends,
    and this row's whole point is staying narrow.

    Corrections round 3, item 1: every trace panel's legend is a
    dedicated strip to the RIGHT of its axes (_side_legend_axes), never
    drawn inside/over the data. The tyre-curve panels keep their
    existing below-panel legend strip (item 1's own second sentence) --
    see _tyre_curve_axes.

    `laps` renders EXACTLY the checked set (addendum item 4) -- colour is
    assigned here, once, from that exact set via core.plot_style.
    lap_styles, so a lap's colour in this figure always matches its
    colour in render_verdict_traces_figure for the same checked set.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_CORNER, theme)
    # h_pad stays small (item 1's bottom-margin fix does NOT come from
    # here): h_pad applies at EVERY nested axes boundary in this
    # composition (4 outer rows, each row itself split for its side
    # legend, plus the tyre-curve row's own 2x2 data/legend subgrid) --
    # raising it enough for an 8mm outer margin cost the tyre panels over
    # half their own size by rendering (6.49cm -> 3.89cm square, measured
    # this session). `rect` reserves an outer margin ONCE, as a single
    # figure-fraction constraint, instead of compounding across every
    # nested subgridspec -- 0.036 of the 24cm figure height = 8.6mm,
    # measured at 8.1mm actual clearance after the legend axes' own
    # internal padding, still clearing item 1's 8mm floor, at no cost to
    # the tyre-curve panel's own square size. Fixes round: rect's LEFT
    # edge was 0.0 -- no reserved margin at all, so the leftmost panel's
    # y-axis label sat flush against the figure edge and got clipped on
    # export. 0.035 of the 16cm figure width = 5.6mm, measured (see this
    # session's report) at >=5mm actual clearance after axis-label text.
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

    # Shared x across the two track-position panels (speed/CS) -- same
    # fix as render_verdict_traces_figure's own. The track map is a
    # plan-view position plot, not a track-position trace -- excluded.
    xlim = ax_speed.get_xlim()
    for ax in (ax_csf, ax_csr):
        ax.set_xlim(xlim)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, [ax_speed, ax_csf, ax_csr, ax_map, ax_tyre_f, ax_tyre_r], theme)
    # _apply_theme's own tick_params(labelsize=PRINT_FONT_SIZE_PT) call
    # above resets the smaller tick font _draw_tyre_curve_panel already
    # set on these two axes specifically (needed to keep the panel over
    # 7cm wide, see _tyre_curve_axes's own docstring) -- restore it after,
    # not before, _apply_theme runs.
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
    """SetupTool's own extension stack, one quantity per panel, shared x
    (track position): Speed, Stability, Front CS, Rear CS, Front LS, Rear
    LS -- no solid/dashed axle encoding anywhere, front and rear are
    always separate panels, matching the interactive dialog's own Traces
    tab exactly. Every panel gets its own side legend column (corrections
    round 3, item 1).

    Returns a LIST of one or two figures. Corrections round 3, item 2:
    six stacked panels at a readable height (checked by rendering) fit
    comfortably under the 24cm cap even with the new side legend columns
    (a side column costs WIDTH, not height, so six-panels-tall was never
    actually at risk here) -- a single figure is returned, not split.
    The two-figure split this round's own instruction pre-specified
    (speed/stability/front CS/rear CS, then speed/front LS/rear LS) is
    documented here in case a future change (e.g. a taller per-panel
    minimum) makes six panels genuinely too cramped: reintroduce it by
    building two _verdict_panel_set calls instead of one, each its own
    _new_figure, speed repeated in both for track-position context.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, theme)
    # Fixes round: rect's left edge defaulted to 0.0 (no explicit rect at
    # all) -- the leftmost panel's y-axis label sat flush against the
    # figure edge and clipped on export. 0.035 of the 16cm figure width
    # = 5.6mm, measured at >=5mm actual clearance.
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
    # Bug fix, Part B: matplotlib's autoscale excludes NaN from a panel's
    # own x-range -- the LS panels' A4 display mask NaNs out most of a
    # lap's own samples, so without a shared xlim they autoscale to a
    # visibly NARROWER x-range than speed/stability/CS. _verdict_panel_set
    # anchors every panel on the first (speed, never NaN-masked).
    axes = _verdict_panel_set(fig, gs, panel_specs)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, axes, theme)
    return [fig]


def _draw_corner_bands(ax, corner_bands, label_above=False):
    # Corrections round 3, LAP TRACE EXPORT item: light grey spans, NOT
    # the interactive dialog's per-corner worst-verdict colour -- a
    # green/gold/red band reads as a traffic-light verdict on paper the
    # way it does not need to be read on screen (where the checkbox
    # legend/header text sits right next to it); print composition, so
    # keep it neutral and let the corner id label carry the identity.
    for start_s, end_s, cid in corner_bands:
        ax.axvspan(start_s, end_s, color="#999999", alpha=0.15, zorder=0)
        if label_above:
            ax.annotate(f"C{cid}", xy=((start_s + end_s) / 2.0, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(0, 2), textcoords="offset points", ha="center", va="bottom",
                        fontsize=ps.PRINT_FONT_SIZE_PT * 0.7, color="#666666")


def render_lap_figure(lap_label, laps, thresholds, corner_bands, theme=ps.PRINT):
    """Full-lap problem-map export: speed, stability, front CS ratio, rear
    CS ratio vs track position over the WHOLE lap (unwindowed, unlike
    render_corner_figure/render_verdict_traces_figure), matching
    LapTraceDialog's own 4-panel scaffold (no LS panel -- LS_ratio has no
    lap-level reading). Corner bands are light grey spans (see
    _draw_corner_bands), corner id labelled above the TOP panel only --
    verdict colours are deliberately not used in print.
    """
    from core import plot_style
    styles = plot_style.lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, theme)
    # Fixes round: same left-margin fix as render_verdict_traces_figure.
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
    """Fixed dpi regardless of theme -- reproducibility (same analysis
    result -> same bytes) depends on dpi/size/font never drifting with
    window state, screen, or caller. Closes the figure after saving.
    """
    fig.savefig(path, dpi=ps.PRINT_DPI, facecolor=fig.get_facecolor())
    plt.close(fig)
