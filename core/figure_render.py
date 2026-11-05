# Matplotlib figure rendering for exports -- display only, draws the arrays
# it's given. The trace dialog's export buttons and the diagnostics share
# this code, so their figures are identical. render_* is the public API;
# _draw_* are building blocks (some diagnostics import them directly).
# Lap = colour (lap_styles); one quantity per panel, front and rear always
# separate (laps carry cs_f/cs_r). The style constants below are shared
# with the trace dialog (pyqtgraph), so a quantity has one colour on screen
# and in print.

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ui.style import NEUTRAL, TEXT_MUTED

# Shared plot styling for the trace dialog (pyqtgraph) and exported figures
# (matplotlib): a quantity has the same colour on screen and in print.
# Plain constants, no plotting imports.
# Lap identity = colour (lap_styles); quantity/axle identity = panel.

# lap palette = matplotlib tab10, hardcoded (no matplotlib import here);
# mid lightness, readable on both themes
LAP_PALETTE = (
    "#1f77b4",  # blue
    "#ff7f0e",  # orange
    "#2ca02c",  # green
    "#d62728",  # red
    "#9467bd",  # purple
    "#8c564b",  # brown
    "#e377c2",  # pink
    "#7f7f7f",  # grey
    "#bcbd22",  # olive
    "#17becf",  # cyan
)

# past 10 laps colours repeat with alternating dash, so no two laps share
# both. "dash" is a style key; callers map it to Qt / matplotlib.
LAP_DASH_PATTERNS = ("solid", "dash")


def lap_styles(lap_numbers):
    """{"color", "dash"} per lap, assigned in ascending lap order. Depends only
    on the checked set -- colour is not a fixed property of a lap number.
    """
    ordered = sorted(lap_numbers)
    n = len(LAP_PALETTE)
    styles = {}
    for i, lap in enumerate(ordered):
        styles[lap] = {
            "color": LAP_PALETTE[i % n],
            "dash": LAP_DASH_PATTERNS[(i // n) % len(LAP_DASH_PATTERNS)],
        }
    return styles


# fixed hues, not lap-identified
FITTED_CURVE_COLOR = "#E040FB"   # EKF auto-fit model curve (Pacejka)

# line widths: screen in px, print in pt. One width for all laps.
SCREEN_LAP_WIDTH = 1.5
PRINT_LAP_WIDTH = 1.0

# kerb-gap bridge in print: same lightness as the dialog's BRIDGE_ALPHA
# (110 of 255), so an interpolated stretch never reads as measured
KERB_BRIDGE_ALPHA_PRINT = 110 / 255

# tyre-curve markers: session cloud light and first, lap samples medium,
# window rings largest and last. Screen px and matplotlib `s` (pt^2)
# scale differently -> set separately.
SESSION_CLOUD_COLOR_SCREEN = "#3A3A3A"
SESSION_CLOUD_COLOR_PRINT = "#DDDDDD"
SESSION_CLOUD_SIZE_SCREEN = 1
SESSION_CLOUD_SIZE_PRINT = 2
# one px step up from 1.5 so laps separate in the dialog; still below the
# window rings
LAP_SAMPLE_SIZE_SCREEN = 2.5
# print only: smaller again so rings and tangent stay readable over the
# marker cloud (screen can zoom)
LAP_SAMPLE_SIZE_PRINT = 3.3
LAP_SAMPLE_ALPHA = 0.4  # 0..1, pyqtgraph scales by 255
WINDOW_RING_SIZE_SCREEN = 3.5
WINDOW_RING_SIZE_PRINT = 12
# fitted / tangent / linear-reference widths, print only
TYRE_LINE_WIDTH_PRINT = 0.8

LEGEND_FONT_PT = "11pt"   # pyqtgraph legend text (dialog only)

# print theme of the exported figures; data hues are the screen ones, only
# background/text/grid change
PRINT = {
    "name": "print",
    "bg": "#ffffff",
    "text": "#000000",
    "text_muted": "#444444",
    "grid": "#999999",
    "grid_alpha": 0.3,
}

# all threshold lines one mid grey (readable on both themes), told apart
# by style: strong dashed, moderate dotted, unstable dash-dot
THRESHOLD_GREY = "#8A8A8A"

# near-strike grade overlay: the muted text grey, solid --
# reads as lower-quality data, not as a threshold line
NEAR_STRIKE_COLOR = TEXT_MUTED

# Track-map-specific annotation colours (not lap-identified data).
TRACK_BG_COLOR = NEUTRAL  # faint whole-lap context outline
WINDOW_RING_COLOR = "#000000"  # estimation-window ring, fixed black

# fixed print geometry -> exports reproducible regardless of window state
PRINT_DPI = 300
PRINT_WIDTH_CM = 16.0          # A4 text width
# white border around the tight export crop (inches); keeps legend text and
# tick labels off the file edge
PRINT_PAD_INCHES = 0.05
# both 24 cm: stacked panels with side legends stay readable
PRINT_HEIGHT_CM_CORNER = 24.0  # 5 rows: speed / CSf / CSr / track map / tyre curves
PRINT_HEIGHT_CM_VERDICT = 24.0  # verdict and lap stacks: speed / stability / CSf / CSr
PRINT_FONT_FAMILY = "DejaVu Sans"  # matplotlib default, always installed
PRINT_FONT_SIZE_PT = 8

CM_PER_INCH = 2.54


def _apply_theme(fig, axes, theme):
    fig.patch.set_facecolor(theme["bg"])
    for ax in axes:
        ax.set_facecolor(theme["bg"])
        ax.tick_params(colors=theme["text"], labelsize=PRINT_FONT_SIZE_PT)
        for spine in ax.spines.values():
            spine.set_color(theme["text_muted"])
        ax.xaxis.label.set_color(theme["text"])
        ax.yaxis.label.set_color(theme["text"])
        ax.title.set_color(theme["text"])
        ax.grid(True, alpha=theme["grid_alpha"], color=theme["grid"])


def _legend(ax, theme, font_scale=0.85, handles=None, labels=None, **kwargs):
    if handles is not None:
        leg = ax.legend(handles, labels, fontsize=PRINT_FONT_SIZE_PT * font_scale, framealpha=0.85,
                         handlelength=1.3, labelspacing=0.3, borderpad=0.35, **kwargs)
    else:
        leg = ax.legend(fontsize=PRINT_FONT_SIZE_PT * font_scale, framealpha=0.85,
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

    The legend gets a real GridSpec cell: a bbox_to_anchor legend below the
    axes, or a wide one, makes constrained_layout's column solve infeasible
    and collapses the tyre panels to ~0 width (only a warning is raised).
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
    return dict(color=style["color"], linewidth=PRINT_LAP_WIDTH, linestyle=linestyle)


def _add_threshold(ax, value, kind, label):
    # thresholds grey, by style as on screen (core.display.CS_LINE_SPECS):
    # boundary solid, strong tier dash-dot, rear info tier dotted and fainter
    # (a note, not a symptom), unstable dash-dot on its own panel
    linestyle, width, alpha = {"boundary": ("-", 1.0, 1.0), "strong": ("-.", 1.0, 1.0),
                               "info": (":", 0.8, 0.6), "unstable": ("-.", 1.0, 1.0)}[kind]
    ax.axhline(value, color=THRESHOLD_GREY, linestyle=linestyle, linewidth=width, alpha=alpha, label=label)


def _draw_kerb_bridges(ax, laps, styles, value_key):
    # Dashed, lighter bridge across kerb gaps, in the lap's colour -- the
    # dialog's convention. Drawn only for laps that carry a "kerb" mask.
    from core.display import kerb_gap_bridge
    drawn = False
    for lap in laps:
        kerb, y = lap.get("kerb"), lap.get(value_key)
        if kerb is None or y is None:
            continue
        yb = kerb_gap_bridge(lap["s"], y, kerb)
        if np.isfinite(yb).any():
            ax.plot(lap["s"], yb, color=styles[lap["lap_number"]]["color"], linewidth=PRINT_LAP_WIDTH,
                    linestyle="--", alpha=KERB_BRIDGE_ALPHA_PRINT, label="_nolegend_")
            drawn = True
    if drawn:
        # one legend line per panel, not one per lap
        ax.plot([], [], color=THRESHOLD_GREY, linewidth=PRINT_LAP_WIDTH, linestyle="--",
                alpha=KERB_BRIDGE_ALPHA_PRINT, label="Kerb bridge (not measured)")


def _draw_speed_panel(ax, laps, styles, theme, legend_ax):
    for lap in laps:
        ax.plot(lap["s"], lap["v_kmh"], label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Speed (km/h)")
    _draw_side_legend(ax, legend_ax, theme)


def _draw_stability_panel(ax, laps, styles, thresholds, theme, legend_ax):
    for lap in laps:
        ax.plot(lap["s"], lap["stab"], label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
    _draw_kerb_bridges(ax, laps, styles, "stab")
    _add_threshold(ax, thresholds["stab"], "unstable", "Unstable below this")
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Stability (Nm/deg)")
    _draw_side_legend(ax, legend_ax, theme)


def _draw_ratio_panel(ax, laps, styles, theme, value_key, ylabel, legend_ax,
                       thresholds=None, near_strike=None):
    # Front or rear CS panel, one quantity each. near_strike = {"mode"}
    # (kerb-exclusion display switch); thresholds carries the
    # classifier.cs_decision_lines values.
    from core.display import near_strike_split
    mode = (near_strike or {}).get("mode", "off") if value_key in ("cs_f", "cs_r") else "off"
    grey_drawn = False
    for lap in laps:
        val = lap.get(value_key)
        if val is None:
            continue
        main, overlay = near_strike_split(val, lap.get("near_strike_" + value_key[-1]), mode)
        ax.plot(lap["s"], main, label=f"Lap {lap['lap_number']}", **_lap_pen_kwargs(lap, styles))
        if overlay is not None:
            ax.plot(lap["s"], overlay, color=NEAR_STRIKE_COLOR, linewidth=PRINT_LAP_WIDTH, label="_nolegend_")
            grey_drawn = True
    if grey_drawn:
        ax.plot([], [], color=NEAR_STRIKE_COLOR, linewidth=PRINT_LAP_WIDTH,
                label="Kerb-affected (greyed)")
    if value_key in ("cs_f", "cs_r"):
        _draw_kerb_bridges(ax, laps, styles, value_key)
    if thresholds is not None:
        from core.display import cs_lines
        for value, kind, label in cs_lines(value_key, thresholds):
            _add_threshold(ax, value, kind, label)
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel(ylabel)
    _draw_side_legend(ax, legend_ax, theme)


def _draw_track_map_panel(ax, track_map, theme):
    # Compact inset without title or legend: the lap colours are already in
    # the other panels, the ring meaning goes in the caption.
    if track_map is None:
        ax.text(0.5, 0.5, "No GPS data", ha="center", va="center",
                color=theme["text_muted"], transform=ax.transAxes,
                fontsize=PRINT_FONT_SIZE_PT * 0.6)
        ax.set_xticks([])
        ax.set_yticks([])
        return
    lap_x, lap_y = track_map["lap_xy"]
    ax.plot(lap_x, lap_y, color=TRACK_BG_COLOR, linewidth=1.0, label="Lap trace")
    # one bracket polyline per checked lap in its colour (each lap drives its
    # own line); grey outline = analysed lap, context only
    for entry in track_map.get("brackets_by_lap", []):
        bx, by = entry["xy"]
        linestyle = "--" if entry["dash"] == "dash" else "-"
        ax.plot(bx, by, color=entry["color"], linewidth=1.2, linestyle=linestyle,
                label=f"Lap {entry['lap_number']} bracket")
    # window rings: hollow black, front solid, rear dotted
    ring_lw = 0.9
    if track_map.get("window_f_xy") is not None:
        wx, wy = track_map["window_f_xy"]
        ax.plot(wx, wy, marker='o', markersize=4, markerfacecolor='none',
                markeredgecolor=WINDOW_RING_COLOR, markeredgewidth=ring_lw,
                linestyle='-', color=WINDOW_RING_COLOR, linewidth=1.0, label="Front window")
    if track_map.get("window_r_xy") is not None:
        wx, wy = track_map["window_r_xy"]
        ax.plot(wx, wy, marker='o', markersize=4, markerfacecolor='none',
                markeredgecolor=WINDOW_RING_COLOR, markeredgewidth=ring_lw,
                linestyle=':', color=WINDOW_RING_COLOR, linewidth=1.0, label="Rear window")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])


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
        ax.scatter(sx[clean], sy[clean], s=SESSION_CLOUD_SIZE_PRINT, color=SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Checked laps, whole lap", zorder=1)
    else:
        ax.scatter(sx, sy, s=SESSION_CLOUD_SIZE_PRINT, color=SESSION_CLOUD_COLOR_PRINT,
                   alpha=0.6, label="Checked laps, whole lap", zorder=1)

    # lap colour: filled = clean, hollow = kerb-affected
    for entry in curve.get("corner_by_lap", []):
        color = entry["color"]
        if entry.get("clean_xy") is not None:
            cx, cy = entry["clean_xy"]
            ax.scatter(cx, cy, s=LAP_SAMPLE_SIZE_PRINT, color=color, alpha=LAP_SAMPLE_ALPHA,
                       label=f"Lap {entry['lap_number']}", zorder=2)
        if entry.get("kerb_xy") is not None:
            kx, ky = entry["kerb_xy"]
            ax.scatter(kx, ky, s=LAP_SAMPLE_SIZE_PRINT, facecolors='none', edgecolors=color,
                       linewidths=1.0, alpha=LAP_SAMPLE_ALPHA, zorder=2)

    if curve.get("linear_ref_line") is not None:
        lx, ly = curve["linear_ref_line"]
        ax.plot(lx, ly, color=theme["text_muted"], linewidth=TYRE_LINE_WIDTH_PRINT,
                linestyle="-", label="Linear reference", zorder=3)

    if curve.get("fitted_line") is not None:
        fx, fy, flabel = curve["fitted_line"]
        ax.plot(fx, fy, color=FITTED_CURVE_COLOR, linewidth=TYRE_LINE_WIDTH_PRINT, label=flabel, zorder=3)

    if curve.get("tangent_line") is not None:
        tx, ty, tlabel = curve["tangent_line"]
        # theme text colour, readable on both themes
        ax.plot(tx, ty, color=theme["text"], linewidth=TYRE_LINE_WIDTH_PRINT,
                linestyle="--", label=tlabel, zorder=3)

    if curve.get("window_xy") is not None:
        wx, wy = curve["window_xy"]
        ax.scatter(wx, wy, s=WINDOW_RING_SIZE_PRINT, facecolors='none',
                   edgecolors=WINDOW_RING_COLOR, linewidths=1.2, label="Estimation window", zorder=5)

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

    ax.set_xlabel("Slip angle (deg)", fontsize=PRINT_FONT_SIZE_PT * 0.75)
    # the force the CS was estimated on: Fy / Fz under fit L, else Fy, shown
    # in kN (6-digit N tick labels ate the width the panel needs, >= 7 cm)
    if curve.get("normalised"):
        ax.set_ylabel(curve.get("y_label", "Fy / Fz (-)"), fontsize=PRINT_FONT_SIZE_PT * 0.75)
    else:
        import matplotlib.ticker as mticker
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, pos: f"{v / 1000:.0f}"))
        ax.set_ylabel("Fy (kN)", fontsize=PRINT_FONT_SIZE_PT * 0.75)
    ax.tick_params(labelsize=PRINT_FONT_SIZE_PT * 0.75)
    ax.set_title(f"{axle_label} tyre curve", fontsize=PRINT_FONT_SIZE_PT)

    # ncol 2 -- wider legends in both columns broke the layout solve
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.set_facecolor(theme["bg"])
    legend_ax.axis("off")
    if handles:
        _legend(legend_ax, theme, handles=handles, labels=labels, loc="center",
                ncol=2, font_scale=0.55, frameon=False)


def _new_figure(width_cm, height_cm, theme):
    plt.rcParams["font.family"] = PRINT_FONT_FAMILY
    plt.rcParams["font.size"] = PRINT_FONT_SIZE_PT
    # constrained_layout: fixed hspace/wspace cannot fit the long axis labels
    # at 16 cm width
    fig = plt.figure(figsize=(width_cm / CM_PER_INCH, height_cm / CM_PER_INCH),
                      constrained_layout=True)
    return fig


def render_corner_figure(corner_label, laps, thresholds, tyre_curves, track_map, theme=PRINT, near_strike=None):
    """Chair-style composition: speed, front CS, rear CS vs s (with thresholds),
    a narrow track-map row (compact: no title/legend), front/rear tyre curves
    (linear reference, fitted model, tangent). Stability is in
    render_verdict_traces_figure.
    Side legend strips per trace panel; tyre curves keep theirs below.
    laps = exactly the checked set; colours assigned here via lap_styles,
    same as the verdict figure.
    """
    styles = lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(PRINT_WIDTH_CM, PRINT_HEIGHT_CM_CORNER, theme)
    # Small h_pad: it applies at every nested boundary and would shrink the
    # tyre panels. Outer margins once via rect: left 0.07 (~11 mm) so the y
    # label isn't clipped, bottom 0.036 (~8.6 mm), top 0.96 so the suptitle
    # stays above the speed panel instead of inside it.
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.04, wspace=0.0, hspace=0.02, rect=(0.07, 0.036, 1.0, 0.96))
    gs = fig.add_gridspec(5, 2, height_ratios=[2.4, 2.4, 2.4, 0.9, 5.4])
    ax_speed, legend_speed = _side_legend_axes(fig, gs[0, :])
    ax_csf, legend_csf = _side_legend_axes(fig, gs[1, :])
    ax_csr, legend_csr = _side_legend_axes(fig, gs[2, :])
    ax_map = fig.add_subplot(gs[3, :])
    ax_tyre_f, ax_tyre_r, legend_f, legend_r = _tyre_curve_axes(fig, gs[4, :])

    _draw_speed_panel(ax_speed, laps, styles, theme, legend_speed)
    _draw_ratio_panel(ax_csf, laps, styles, theme, "cs_f", "Front CS ratio", legend_csf,
                       thresholds=thresholds, near_strike=near_strike)
    _draw_ratio_panel(ax_csr, laps, styles, theme, "cs_r", "Rear CS ratio", legend_csr,
                       thresholds=thresholds, near_strike=near_strike)
    _draw_track_map_panel(ax_map, track_map, theme)
    _draw_tyre_curve_panel(ax_tyre_f, "Front", tyre_curves["front"], theme, legend_f)
    _draw_tyre_curve_panel(ax_tyre_r, "Rear", tyre_curves["rear"], theme, legend_r)

    # shared x for speed/CS; track map is plan view, excluded
    xlim = ax_speed.get_xlim()
    for ax in (ax_csf, ax_csr):
        ax.set_xlim(xlim)

    fig.suptitle(corner_label, color=theme["text"], fontsize=PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, [ax_speed, ax_csf, ax_csr, ax_map, ax_tyre_f, ax_tyre_r], theme)
    # _apply_theme reset the smaller tyre-panel tick font -> restore after it
    for ax in (ax_tyre_f, ax_tyre_r):
        ax.tick_params(labelsize=PRINT_FONT_SIZE_PT * 0.75)
    _near_strike_caption(fig, laps, near_strike, theme)
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


def render_verdict_traces_figure(corner_label, laps, thresholds, theme=PRINT, near_strike=None):
    """Extension stack, one quantity per panel, shared x: speed, stability,
    front CS, rear CS. Side legend per panel. Returns a list holding one
    figure; the dialog's export saves every figure in it.
    """
    styles = lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(PRINT_WIDTH_CM, PRINT_HEIGHT_CM_VERDICT, theme)
    # left margin 0.035 (~5.6 mm) so the y label isn't clipped
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.03, wspace=0.0, hspace=0.02, rect=(0.035, 0.0, 1.0, 1.0))
    gs = fig.add_gridspec(4, 1)
    panel_specs = [
        ("speed", lambda ax, lax: _draw_speed_panel(ax, laps, styles, theme, lax)),
        ("stab", lambda ax, lax: _draw_stability_panel(ax, laps, styles, thresholds, theme, lax)),
        ("cs_f", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_f", "Front CS ratio", lax,
                                                    thresholds=thresholds,
                                                    near_strike=near_strike)),
        ("cs_r", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_r", "Rear CS ratio", lax,
                                                    thresholds=thresholds,
                                                    near_strike=near_strike)),
    ]
    axes = _verdict_panel_set(fig, gs, panel_specs)

    fig.suptitle(corner_label, color=theme["text"], fontsize=PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, axes, theme)
    _near_strike_caption(fig, laps, near_strike, theme)
    return [fig]


def _draw_corner_bands(ax, corner_bands, label_above=False):
    # neutral grey bands in print -- verdict colours read as a traffic light on
    # paper; the corner label carries identity
    for start_s, end_s, cid in corner_bands:
        ax.axvspan(start_s, end_s, color="#999999", alpha=0.15, zorder=0)
        if label_above:
            ax.annotate(f"C{cid}", xy=((start_s + end_s) / 2.0, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(0, 2), textcoords="offset points", ha="center", va="bottom",
                        fontsize=PRINT_FONT_SIZE_PT * 0.7, color="#666666")


def render_lap_figure(lap_label, laps, thresholds, corner_bands, theme=PRINT, near_strike=None):
    """Whole-lap export: speed, stability, front CS, rear CS vs s. Corner
    bands in light grey, ids above the top panel.
    """
    styles = lap_styles(lap["lap_number"] for lap in laps)

    fig = _new_figure(PRINT_WIDTH_CM, PRINT_HEIGHT_CM_VERDICT, theme)
    fig.get_layout_engine().set(w_pad=0.0, h_pad=0.03, wspace=0.0, hspace=0.02, rect=(0.035, 0.0, 1.0, 1.0))
    gs = fig.add_gridspec(4, 1)
    panel_specs = [
        ("speed", lambda ax, lax: _draw_speed_panel(ax, laps, styles, theme, lax)),
        ("stab", lambda ax, lax: _draw_stability_panel(ax, laps, styles, thresholds, theme, lax)),
        ("cs_f", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_f", "Front CS ratio", lax,
                                                    thresholds=thresholds,
                                                    near_strike=near_strike)),
        ("cs_r", lambda ax, lax: _draw_ratio_panel(ax, laps, styles, theme, "cs_r", "Rear CS ratio", lax,
                                                    thresholds=thresholds,
                                                    near_strike=near_strike)),
    ]
    axes = _verdict_panel_set(fig, gs, panel_specs)
    for i, ax in enumerate(axes):
        _draw_corner_bands(ax, corner_bands, label_above=(i == 0))

    fig.suptitle(lap_label, color=theme["text"], fontsize=PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, axes, theme)
    _near_strike_caption(fig, laps, near_strike, theme)
    return fig


def _near_strike_caption(fig, laps, near_strike, theme):
    # one caption sentence under the figure; counts over the drawn CS panels
    from core.display import kerb_exclusion_caption
    if not near_strike:
        return
    n_marked = n_finite = 0
    for lap in laps:
        for ax in "fr":
            val, mark = lap.get(f"cs_{ax}"), lap.get(f"near_strike_{ax}")
            if val is None or mark is None:
                continue
            fin = np.isfinite(np.asarray(val, dtype=float))
            n_finite += int(fin.sum())
            n_marked += int((np.asarray(mark, dtype=bool) & fin).sum())
    if n_finite == 0:
        return
    text = kerb_exclusion_caption(near_strike["mode"], n_marked, n_finite)
    engine = fig.get_layout_engine()
    rect = list(engine.get()["rect"])
    if rect[1] < 0.035:
        # make room for the caption line under the lowest x label
        rect[3] = rect[3] - (0.035 - rect[1])
        rect[1] = 0.035
        engine.set(rect=tuple(rect))
    fig.text(0.5, 0.008, text, ha="center", va="bottom", wrap=True, color=theme["text"],
             fontsize=PRINT_FONT_SIZE_PT * 0.8)


def save_png(fig, path):
    """Fixed dpi -> same result, same bytes. Closes the figure.

    Tight crop: constrained_layout leaves side-legend text and the
    fixed-aspect map/tyre axes partly outside the nominal 16 cm canvas, so
    the saved canvas grows to every drawn artist and can come out a little
    wider than PRINT_WIDTH_CM. The layout itself is unchanged.
    """
    fig.savefig(path, dpi=PRINT_DPI, facecolor=fig.get_facecolor(), bbox_inches="tight",
                pad_inches=PRINT_PAD_INCHES)
    plt.close(fig)
