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


def _legend(ax, theme, font_scale=0.85, **kwargs):
    leg = ax.legend(fontsize=ps.PRINT_FONT_SIZE_PT * font_scale, framealpha=0.85,
                     handlelength=1.3, labelspacing=0.3, borderpad=0.35, **kwargs)
    if leg is not None:
        leg.get_frame().set_facecolor(theme["bg"])
        leg.get_frame().set_edgecolor(theme["text_muted"])
        for text in leg.get_texts():
            text.set_color(theme["text"])
    return leg


def _lap_style(lap):
    selected = lap.get("selected", True)
    return dict(
        linewidth=ps.SELECTED_WIDTH if selected else ps.NORMAL_WIDTH,
        alpha=(ps.ALPHA_SELECTED if selected else ps.ALPHA_FAINT) / 255.0,
    )


def _draw_speed_panel(ax, laps, theme, color=ps.SPEED_COLOR):
    for i, lap in enumerate(laps):
        ax.plot(lap["s"], lap["v_kmh"], color=color,
                label="Speed" if i == 0 else None, **_lap_style(lap))
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Speed (km/h)")
    _legend(ax, theme, loc="lower center", ncol=1)


def _draw_cs_ratio_panel(ax, laps, thresholds, theme):
    for i, lap in enumerate(laps):
        ax.plot(lap["s"], lap["cs_f"], color=ps.CSF_COLOR,
                label="Front CS" if i == 0 else None, **_lap_style(lap))
        ax.plot(lap["s"], lap["cs_r"], color=ps.CSR_COLOR,
                label="Rear CS" if i == 0 else None, **_lap_style(lap))
    ax.axhline(thresholds["strong_csf"], color=ps.THRESHOLD_CSF_COLOR, linestyle="--",
               linewidth=1.0, label="Front CS strong")
    ax.axhline(thresholds["moderate_csf"], color=ps.THRESHOLD_CSF_COLOR, linestyle=":",
               linewidth=1.0, label="Front CS moderate")
    ax.axhline(thresholds["strong_csr"], color=ps.THRESHOLD_CSR_COLOR, linestyle="--",
               linewidth=1.0, label="Rear CS strong")
    ax.axhline(thresholds["moderate_csr"], color=ps.THRESHOLD_CSR_COLOR, linestyle=":",
               linewidth=1.0, label="Rear CS moderate")
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("CS ratio")
    _legend(ax, theme, loc="lower center", ncol=3)


def _draw_stability_panel(ax, laps, thresholds, theme):
    for i, lap in enumerate(laps):
        ax.plot(lap["s"], lap["stab"], color=ps.STAB_COLOR,
                label="Stability" if i == 0 else None, **_lap_style(lap))
    ax.axhline(thresholds["stab"], color=ps.THRESHOLD_STAB_COLOR, linestyle="--",
               linewidth=1.0, label="Unstable below this")
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("Stability (Nm/deg)")
    _legend(ax, theme, loc="lower center", ncol=1)


def _draw_ls_panel(ax, laps, theme):
    for i, lap in enumerate(laps):
        if lap.get("ls_f") is None or lap.get("ls_r") is None:
            continue
        ax.plot(lap["s"], lap["ls_f"], color=ps.LSF_COLOR,
                label="Front LS" if i == 0 else None, **_lap_style(lap))
        ax.plot(lap["s"], lap["ls_r"], color=ps.LSR_COLOR,
                label="Rear LS" if i == 0 else None, **_lap_style(lap))
    ax.set_xlabel("Track position s (m)")
    ax.set_ylabel("LS ratio")
    _legend(ax, theme, loc="lower center", ncol=1)


def _draw_track_map_panel(ax, track_map, corner_label, theme):
    if track_map is None:
        # No GPS channel on this outing -- corner_trace_dialog.py's
        # _render_track_map returns None in that case rather than an
        # empty geometry dict; render a plain placeholder instead of
        # crashing on a missing "lap_xy" key.
        ax.text(0.5, 0.5, "No GPS data", ha="center", va="center",
                color=theme["text_muted"], transform=ax.transAxes)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("Track map")
        return
    lap_x, lap_y = track_map["lap_xy"]
    ax.plot(lap_x, lap_y, color=ps.TRACK_BG_COLOR, linewidth=1.0, label="Lap trace")
    if track_map.get("bracket_xy") is not None:
        bx, by = track_map["bracket_xy"]
        ax.plot(bx, by, color=ps.CORNER_BRACKET_COLOR, linewidth=3.0, label=f"{corner_label} bracket")
    if track_map.get("window_f_xy") is not None:
        wx, wy = track_map["window_f_xy"]
        ax.plot(wx, wy, color=ps.WINDOW_F_COLOR, linewidth=4.0, label="Front window")
    if track_map.get("window_r_xy") is not None:
        wx, wy = track_map["window_r_xy"]
        ax.plot(wx, wy, color=ps.WINDOW_R_COLOR, linewidth=4.0, linestyle=":", label="Rear window")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Track map")
    _legend(ax, theme, loc="upper right", ncol=1)


def _draw_tyre_curve_panel(ax, axle_label, curve, theme):
    sx, sy, skerb = curve["session_xy"]
    if skerb is not None:
        clean = ~skerb
        ax.scatter(sx[clean], sy[clean], s=ps.MARKER_SIZE_SESSION, color=theme["text_muted"],
                   alpha=0.35, label="Session")
        ax.scatter(sx[skerb], sy[skerb], s=ps.MARKER_SIZE_SESSION * 2, marker="x",
                   color=ps.TRACK_BG_COLOR, alpha=0.6, label="Kerb-flagged")
    else:
        ax.scatter(sx, sy, s=ps.MARKER_SIZE_SESSION, color=theme["text_muted"], alpha=0.35, label="Session")

    # theme["text"] (not ACCENT) -- ACCENT is already the linear-reference
    # line's colour a few lines below; reusing it here would make "corner
    # samples" and "linear reference" indistinguishable in the legend.
    cx, cy = curve["corner_xy"]
    ax.scatter(cx, cy, s=ps.MARKER_SIZE_CORNER, color=theme["text"], alpha=0.7, label="Corner samples")

    if curve.get("window_xy") is not None:
        wx, wy = curve["window_xy"]
        ax.scatter(wx, wy, s=ps.MARKER_SIZE_WINDOW, color=ps.THRESHOLD_STAB_COLOR,
                   edgecolor=theme["text"], linewidth=0.3, label="Estimation window")

    if curve.get("linear_ref_line") is not None:
        lx, ly = curve["linear_ref_line"]
        ax.plot(lx, ly, color=ps.CORNER_BRACKET_COLOR, linewidth=2.0, label="Linear reference")

    if curve.get("fitted_line") is not None:
        fx, fy, flabel = curve["fitted_line"]
        ax.plot(fx, fy, color=ps.FITTED_CURVE_COLOR, linewidth=2.0, label=flabel)

    if curve.get("tangent_line") is not None:
        tx, ty, tlabel = curve["tangent_line"]
        ax.plot(tx, ty, color=ps.TANGENT_COLOR, linewidth=ps.TANGENT_WIDTH,
                linestyle="--", label=tlabel)

    ax.axhline(0.0, color=theme["text_muted"], linewidth=0.6)
    ax.axvline(0.0, color=theme["text_muted"], linewidth=0.6)
    ax.set_xlabel("Slip angle (deg)")
    ax.set_ylabel("Lateral force Fy (N)")
    ax.set_title(f"{axle_label} tyre curve")
    # Smaller font than other panels: up to 7 entries, some with a long
    # dynamic label (model name, tangent CS value) -- these panels are
    # the narrowest third of the figure width, verified by rendering
    # (default font_scale clipped the legend text past the figure edge
    # on the rightmost panel).
    _legend(ax, theme, loc="best", ncol=1, font_scale=0.62)


def _new_figure(width_cm, height_cm, theme):
    plt.rcParams["font.family"] = ps.PRINT_FONT_FAMILY
    plt.rcParams["font.size"] = ps.PRINT_FONT_SIZE_PT
    # constrained_layout, not manual hspace/wspace/tight_layout: at a
    # fixed 16cm print width the y-axis label + tick text (e.g. "Lateral
    # force Fy (N)" next to "-30000") needs real per-figure space that a
    # single fixed hspace/wspace guess got wrong -- constrained_layout
    # solves that per-axes automatically, including reserving room for
    # legends placed with bbox_to_anchor.
    fig = plt.figure(figsize=(width_cm / CM_PER_INCH, height_cm / CM_PER_INCH),
                      constrained_layout=True)
    return fig


def render_corner_figure(corner_label, laps, thresholds, track_map, tyre_curves, theme=ps.PRINT):
    """The full chair-style composition: velocity vs s, CS ratio vs s
    (with thresholds), track map (corner bracket + both axle windows),
    front and rear tyre curves (linear reference, fitted model, tangent).
    Stability/LS are deliberately NOT here -- SetupTool's own extension
    panels, kept out of this composition so exported thesis figures stay
    uncrowded (see render_verdict_traces_figure for those).
    """
    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_CORNER, theme)
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.8])
    ax_speed = fig.add_subplot(gs[0, :])
    ax_cs = fig.add_subplot(gs[1, :])
    ax_map = fig.add_subplot(gs[2, 0])
    ax_tyre_f = fig.add_subplot(gs[2, 1])
    ax_tyre_r = fig.add_subplot(gs[2, 2])

    _draw_speed_panel(ax_speed, laps, theme)
    _draw_cs_ratio_panel(ax_cs, laps, thresholds, theme)
    _draw_track_map_panel(ax_map, track_map, corner_label, theme)
    _draw_tyre_curve_panel(ax_tyre_f, "Front", tyre_curves["front"], theme)
    _draw_tyre_curve_panel(ax_tyre_r, "Rear", tyre_curves["rear"], theme)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, [ax_speed, ax_cs, ax_map, ax_tyre_f, ax_tyre_r], theme)
    return fig


def render_verdict_traces_figure(corner_label, laps, thresholds, theme=ps.PRINT):
    """SetupTool's own extension stack: stability / CS ratio / LS ratio /
    speed vs s, the same four panels as the dialog's interactive Traces
    tab -- kept separate from render_corner_figure so a thesis reader
    sees the chair-comparable result on its own, uncluttered figure.
    """
    fig = _new_figure(ps.PRINT_WIDTH_CM, ps.PRINT_HEIGHT_CM_VERDICT, theme)
    gs = fig.add_gridspec(4, 1)
    ax_stab = fig.add_subplot(gs[0, 0])
    ax_cs = fig.add_subplot(gs[1, 0])
    ax_ls = fig.add_subplot(gs[2, 0])
    ax_speed = fig.add_subplot(gs[3, 0])

    _draw_stability_panel(ax_stab, laps, thresholds, theme)
    _draw_cs_ratio_panel(ax_cs, laps, thresholds, theme)
    _draw_ls_panel(ax_ls, laps, theme)
    _draw_speed_panel(ax_speed, laps, theme)

    fig.suptitle(corner_label, color=theme["text"], fontsize=ps.PRINT_FONT_SIZE_PT + 2)
    _apply_theme(fig, [ax_stab, ax_cs, ax_ls, ax_speed], theme)
    return fig


def save_png(fig, path):
    """Fixed dpi regardless of theme -- reproducibility (same analysis
    result -> same bytes) depends on dpi/size/font never drifting with
    window state, screen, or caller. Closes the figure after saving.
    """
    fig.savefig(path, dpi=ps.PRINT_DPI, facecolor=fig.get_facecolor())
    plt.close(fig)
