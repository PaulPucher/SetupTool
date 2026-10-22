# Corner and full-lap trace windows: stability, CS_ratio f/r and speed over
# track position s. Corner window = one stable corner's
# bracket + config margin; lap window = whole lap with a tinted band per
# corner. Display only -- plots arrays from the cached pipeline result.

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QTabWidget, QWidget,
    QPushButton, QFileDialog, QMessageBox,
)

from ui.style import BAD, BORDER, NEUTRAL, PANEL, PANEL_ALT, TEXT, TEXT_DIM, TEXT_MUTED

# Colours/widths from core/plot_style.py, shared with the exports
# (core/figure_render.py, diagnostics/inspect_step2_chair_plots.py).
# Lap = colour (lap_styles over the checked set); quantity/axle = panel.
# All laps same width.
from core import plot_style
from core.trace_display import kerb_exclusion_caption
from modules.limit_events import legend_text as event_legend_text
from core.plot_style import (
    FITTED_CURVE_COLOR, TRACK_BG_COLOR, WINDOW_RING_COLOR, THRESHOLD_GREY, NEAR_STRIKE_COLOR,
    SCREEN_LAP_WIDTH, LEGEND_FONT_PT,
    SESSION_CLOUD_COLOR_SCREEN, SESSION_CLOUD_SIZE_SCREEN,
    LAP_SAMPLE_SIZE_SCREEN, LAP_SAMPLE_ALPHA, WINDOW_RING_SIZE_SCREEN,
)

# One legend style for every plot, font above pyqtgraph's default.
# addLegend() once per plot at construction -- pyqtgraph updates it on
# clear()/plot(name=); calling it per render stacks duplicate boxes.
LEGEND_INSET = (-8, 8)  # px from the top-right corner


def _style_new_legend(plot):
    """One styled legend per plot, call once. Solid background. Pinned top-right
    (anchor), not top-left via offset -- traces start at the left edge.
    """
    import pyqtgraph as pg
    legend = plot.addLegend(labelTextSize=LEGEND_FONT_PT)
    legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=LEGEND_INSET)
    legend.setBrush(pg.mkBrush(PANEL_ALT))
    legend.setLabelTextColor(TEXT_MUTED)
    return legend


# short footer; the in-plot legends name every curve and threshold
BASE_LEGEND_TEXT = (
    "Colour = lap (see each panel's own legend). "
    "Unchecked laps are not drawn anywhere in this window."
)

# corner-band sentence; colour words match ui.style BAD/WARN/OK
LAP_BAND_LEGEND_TEXT = (
    "Corner bands: colour = worst verdict across all laps (red/gold/green). "
    "Click a band to open that corner's trace."
)

PHASE_ORDER = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]

# kerb-gap bridge: lighter than a measured trace (config corner_trace_display
# kerb_gap_bridge switches it)
BRIDGE_ALPHA = 110
KERB_BRIDGE_LEGEND_TEXT = (
    "Dashed, lighter stretch across a kerb band: straight-line bridge for "
    "continuity only -- not measured, not used by any verdict."
)
# event-strip legend: modules.limit_events.legend_text (floor events at -0.53)
EVENT_STRIP_WIDTH = 5  # px; a line, not a filled band (shading budget)
EXCLUSION_LEGEND_TEXT = (
    "Grey band: left out of the analysis here -- kerb-affected (the analysis fills the span from "
    "clean data nearby) or the car not up to speed (out lap or pit lane)."
)

# tyre-curve tab legend. Plot design after the chair performance_analysis
# tooling (internal); no code copied.
TYRE_CURVE_LEGEND_TEXT = (
    "Slip angle vs lateral force, this corner's window. Filled = lap "
    "sample, hollow = kerb-affected, black ring = estimation window."
)


def _tyre_curve_basis(cs, cs_axle_loads):
    """Which force the tyre-curve tab plots, so points, reference, tangent
    and fitted curve share the unit the CS was estimated in. Fit L estimates
    the CS on Fy / Fz with its 2 Hz axle loads (the CS output then carries
    the relative reference gate keys): plot Fy / Fz, slopes in 1/rad. CS on
    Fy (kinematic source, or fit L degenerate before the loads existed): plot
    Fy in N, slopes in N/rad. Returns {"normalised", "loads", "y_label",
    "slope_unit"}, or None when the CS is normalised but this result holds
    no loads (an older cached result) -- the tab then says so instead of
    mixing units."""
    normalised = cs is not None and "ref_relative_median_f" in cs
    if normalised and cs_axle_loads is None:
        return None
    if normalised:
        return {"normalised": True, "loads": cs_axle_loads, "y_label": "Fy / Fz (-)", "slope_unit": "1/rad"}
    return {"normalised": False, "loads": None, "y_label": "Lateral force Fy (N)", "slope_unit": "N/rad"}


def _tyre_curve_force(basis, axle, Fy_arr):
    # the plotted force, same normalisation as the CS estimate
    if Fy_arr is None or not basis["normalised"]:
        return Fy_arr
    return Fy_arr / basis["loads"]["fz_f_N" if axle == "front" else "fz_r_N"]


def _fitted_curve_d(basis, axle_fit):
    # fit L stores D = mu * fz_ref; on the Fy / Fz axis the curve's peak
    # parameter is mu = D / fz_ref
    if basis["normalised"]:
        return axle_fit["D"] / axle_fit["fz_ref_N"]
    return axle_fit["D"]


def _lap_slice(t, s_m, lap_start_t, lap_end_t):
    """Clip to [lap_start_t, lap_end_t) and trim trailing already-reset samples:
    lap_end_t (lap_number) and the lap_distance reset don't line up exactly,
    and those near-zero samples broke the s clamp and searchsorted's sort
    order (seen on Dubai). Trim at the first drop below the running max.
    Returns (lo, hi, lap_s, lap_s_lo, lap_s_hi) or None if nothing finite.
    """
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return None
    lap_s = s_m[lo:hi]

    RESET_DROP_M = 50.0
    finite = np.isfinite(lap_s)
    running_max = np.maximum.accumulate(np.where(finite, lap_s, -np.inf))
    reset_mask = finite & (running_max - lap_s > RESET_DROP_M)
    if reset_mask.any():
        cut = int(np.argmax(reset_mask))
        lap_s = lap_s[:cut]
        hi = lo + cut

    finite_idx = np.flatnonzero(np.isfinite(lap_s))
    if len(finite_idx) == 0:
        return None
    # s_m is NaN next to a reset -> clamp on first/last finite; min(nan, x)
    # returns nan when nan comes first
    lap_s_lo = float(lap_s[finite_idx[0]])
    lap_s_hi = float(lap_s[finite_idx[-1]])
    return lo, hi, lap_s, lap_s_lo, lap_s_hi


def _extend_slice_with_margin(t, s_m, lap_start_t, lap_end_t,
                               bracket_start_m, bracket_end_m,
                               margin_before_m, margin_after_m):
    """Canonical bracket (bracket_start_m/end_m, same for every lap) widened by
    the config margin, clamped to this lap's s range (s_m resets per lap).
    Not anchored on per-lap phase times -- a truncated entry_1_brake made the
    window open deep inside the corner.
    Returns (slice, start_s, end_s) or (slice(0, 0), None, None).
    """
    clipped = _lap_slice(t, s_m, lap_start_t, lap_end_t)
    if clipped is None:
        return slice(0, 0), None, None
    lo, hi, lap_s, lap_s_lo, lap_s_hi = clipped
    target_start_s = max(lap_s_lo, bracket_start_m - margin_before_m)
    target_end_s = min(lap_s_hi, bracket_end_m + margin_after_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local), target_start_s, target_end_s


def _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m, bracket_start_m, bracket_end_m):
    """Lowest-CS_ratio sample in the canonical bracket over all valid instances
    (same idea as inspect_step2_chair_plots' _find_worst_phase), for the
    estimation-window highlight. Returns {"index"} or None.
    """
    best = None
    for c in instances:
        lap = laps_by_number.get(c["lap_number"])
        if lap is None:
            continue
        sl, _start_s, _end_s = _extend_slice_with_margin(
            t, s_m, lap["start_time"], lap["end_time"], bracket_start_m, bracket_end_m, 0.0, 0.0,
        )
        if sl.stop <= sl.start:
            continue
        seg = cs_ratio_arr[sl]
        if not np.isfinite(seg).any():
            continue
        local_idx = int(np.nanargmin(np.where(np.isfinite(seg), seg, np.inf)))
        val = seg[local_idx]
        if best is None or val < best[0]:
            best = (val, sl.start + local_idx)
    if best is None:
        return None
    return {"index": best[1], "cs_ratio": best[0]}


def _worst_stab_phase(summary):
    # duplicates _classify_corner's stability argmin so the tinted band matches
    # the verdict badge -- change both together
    worst_phase, worst_val = None, float("inf")
    for phase, phase_data in summary["phases"].items():
        val = phase_data["stability_observed_Nm_per_deg"]["median"]
        if val == val and val < worst_val:  # NaN-safe
            worst_val = val
            worst_phase = phase
    return worst_phase


def _phase_bands_for_lap(corner, t, s_m):
    # phase boundaries of the representative lap, in s. apex_3 is one instant
    # -> apex line instead of a band. Bands cover the five phases only; the
    # margin stays unshaded context.
    bands = []
    for phase in PHASE_ORDER:
        start_t, end_t = corner["segments"][phase]
        if end_t <= start_t:
            continue
        s_start = float(np.interp(start_t, t, s_m))
        s_end = float(np.interp(end_t, t, s_m))
        bands.append((phase, s_start, s_end))
    return bands


def _contiguous_runs(x, mask):
    runs = []
    in_run = False
    start_idx = None
    for i, flag in enumerate(mask):
        if flag and not in_run:
            in_run, start_idx = True, i
        elif not flag and in_run:
            in_run = False
            runs.append((x[start_idx], x[i - 1]))
    if in_run:
        runs.append((x[start_idx], x[-1]))
    return runs


def _aggregate_worst_severity(corner_summaries, classify_fn):
    # lap view = problem map: band tint = worst severity over ALL valid
    # instances, via the injected classify_fn (same as everywhere else)
    from modules.recommendation import SEVERITY_RANK

    worst_rank = -1
    worst_colour = None
    for corner_summary in corner_summaries:
        severity, _short, _long, colour = classify_fn(corner_summary)
        rank = SEVERITY_RANK[severity]
        if rank > worst_rank:
            worst_rank = rank
            worst_colour = colour
    return worst_colour


def _fastest_lap(lap_numbers, laps_by_number):
    # fastest = lap_time_precise if present, else lap_time (as csv_parser);
    # lowest lap number only if neither exists
    times = {}
    for ln in lap_numbers:
        lap = laps_by_number.get(ln, {})
        lt = lap.get("lap_time_precise")
        if lt is None:
            lt = lap.get("lap_time")
        if lt is not None:
            times[ln] = lt
    if times:
        return min(times, key=times.get)
    return min(lap_numbers)


def _fastest_n_laps(lap_numbers, laps_by_number, n):
    # default checked set = fastest N valid laps (default_laps_shown); laps
    # without a time sort last, never dropped
    def _time(ln):
        lap = laps_by_number.get(ln, {})
        lt = lap.get("lap_time_precise")
        if lt is None:
            lt = lap.get("lap_time")
        return lt if lt is not None else float("inf")

    return set(sorted(lap_numbers, key=_time)[:n])


class _TraceDialogBase(QDialog):
    """Shared scaffold for both windows: pyqtgraph panels, per-lap checkboxes,
    legend, pen/threshold/masked-span helpers. Subclasses add show_corner or
    show_lap.
    Colour depends on the whole checked set -> a checkbox toggle re-renders
    in full (_rerender_preserving_checked). Only checked laps are plotted,
    so every named legend item is visible.
    """

    WINDOW_TITLE = "Trace"
    WINDOW_SIZE = (880, 680)

    # panel order: speed, stability, then one axle per panel
    PANEL_TITLES = [
        ("Speed (km/h)", "speed"), ("Stability (Nm/deg)", "stab"),
        ("Front CS ratio", "cs_f"), ("Rear CS ratio", "cs_r"),
    ]
    # panels on the CS ratio scale -> fixed y-range
    RATIO_PANEL_KEYS = ("cs_f", "cs_r")
    # lighter vertical stretch, by key (speed is the first row)
    CONTEXT_PANEL_KEYS = ("speed",)

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg
        from modules.stability_analysis import load_parameters

        self.panel_titles = list(self.PANEL_TITLES)

        self.setWindowTitle(self.WINDOW_TITLE)
        width, height = self.WINDOW_SIZE
        self.resize(width, height)
        self.setModal(False)
        # native min/max buttons -- data-dense windows
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowMinMaxButtonsHint)
        pg.setConfigOptions(antialias=True)

        self.lap_curve_items = {}  # lap_number -> [curve items], checked laps only
        # lap_number -> [kerb-gap bridge items]; drawn only, never analysed
        self.lap_bridge_items = {}
        # quiet-lap dash (corner window only), ORed with the palette's past-10 dash
        self._lap_line_style = {}
        # lap_number -> {"color", "dash"} for the current checked set
        self._current_styles = {}
        self.lap_visible = {}
        # replayed on every checkbox toggle; set at the top of show_corner/show_lap
        self._last_show_args = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        self.header_label = QLabel("")
        self.header_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        layout.addWidget(self.header_label)

        # per-lap checkboxes, rebuilt per render, outside the plot area
        self.lap_checkbox_container = self._build_lap_checkbox_container()
        layout.addWidget(self.lap_checkbox_container)

        self.pg_layout = pg.GraphicsLayoutWidget()
        self.pg_layout.setBackground(PANEL)
        layout.addWidget(self.pg_layout)

        # placeholder; rebuilt per render. Caption says why (threshold meaning,
        # fold-back), legends say which line is which.
        self.legend_label = QLabel(BASE_LEGEND_TEXT)
        self.legend_label.setWordWrap(True)
        self.legend_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        layout.addWidget(self.legend_label)

        _disp = load_parameters().get("corner_trace_display", {})
        self._ratio_y_range = (_disp.get("cs_ls_panel_y_min", -3.5), _disp.get("cs_ls_panel_y_max", 1.2))
        self._exclusion_spans = []
        self.plots = {}
        self.legends = {}
        first_plot = None
        for i, (label, key) in enumerate(self.panel_titles):
            is_last = (i == len(self.panel_titles) - 1)
            plot = self.pg_layout.addPlot()
            plot.setLabel('left', label, color='#888', size='8pt')
            plot.showGrid(x=True, y=True, alpha=0.15)
            plot.getViewBox().setMouseMode(pg.ViewBox.PanMode)
            if first_plot is None:
                first_plot = plot
            else:
                plot.setXLink(first_plot)
            if is_last:
                plot.getAxis('bottom').setLabel('Track position s (m)', color='#888', size='8pt')
            else:
                plot.getAxis('bottom').setStyle(showValues=False)
            # one legend per plot (_style_new_legend)
            self.legends[key] = _style_new_legend(plot)
            self.plots[key] = plot
            self.pg_layout.nextRow()

        # speed = context -> less stretch (ratio, survives resizing); the rest
        # full stretch
        for i, (_label, key) in enumerate(self.panel_titles):
            context = key in self.CONTEXT_PANEL_KEYS
            self.pg_layout.ci.layout.setRowStretchFactor(i, 1 if context else 3)

    # None = every panel visible (lap window); the corner window narrows it
    DEFAULT_PANELS = None

    def _layout_panels(self, visible):
        # re-lays the existing plot items; hidden ones stay alive (and keep
        # receiving data), they only leave the layout
        ci = self.pg_layout.ci
        ci.clear()
        keys = [k for _label, k in self.panel_titles if k in visible]
        for i, key in enumerate(keys):
            plot = self.plots[key]
            ci.addItem(plot, row=i, col=0)
            last = (i == len(keys) - 1)
            axis = plot.getAxis('bottom')
            axis.setStyle(showValues=last)
            axis.setLabel('Track position s (m)', color='#888', size='8pt')
            axis.showLabel(last)
            context = key in self.CONTEXT_PANEL_KEYS
            ci.layout.setRowStretchFactor(i, 1 if context else 3)
        for j in range(len(keys), len(self.panel_titles)):
            ci.layout.setRowStretchFactor(j, 0)
        # x-link to the first VISIBLE panel: pyqtgraph aligns linked views by
        # screen geometry, and a panel outside the scene has none
        if keys:
            anchor = self.plots[keys[0]]
            anchor.setXLink(None)
            for _label, key in self.panel_titles:
                plot = self.plots[key]
                if plot is not anchor:
                    plot.setXLink(anchor if key in keys else None)

    def _build_lap_checkbox_container(self):
        from PyQt6.QtWidgets import QWidget
        container = QWidget()
        row_layout = QHBoxLayout(container)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(10)
        self.lap_checkbox_layout = row_layout
        return container

    def closeEvent(self, event):
        # hide, don't destroy -- no scene rebuild on reopen; the two windows are
        # independent
        event.ignore()
        self.hide()

    def _rebuild_lap_checkboxes(self, instances, selected_lap, default_checked_laps=None, fastest_lap=None,
                                 preserve_visible=None):
        # default_checked_laps: laps checked initially, None = all.
        # fastest_lap: lap to label "(fastest)", or None.
        # preserve_visible: exact {lap: bool} from a toggle re-render, used as is.
        while self.lap_checkbox_layout.count():
            item = self.lap_checkbox_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self.lap_visible = {}
        for c in instances:
            lap_num = c["lap_number"]
            is_quiet = "canonical_quiet" in c.get("warnings", [])
            label = f"Lap {lap_num}" + (" (no signal - quiet)" if is_quiet else "")
            if lap_num == fastest_lap:
                label += " (fastest)"
            if lap_num == selected_lap:
                label += " (selected)"
            cb = QCheckBox(label)
            if preserve_visible is not None:
                is_checked = preserve_visible.get(lap_num, False)
            else:
                is_checked = True if default_checked_laps is None else (lap_num in default_checked_laps)
            cb.setChecked(is_checked)
            cb.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
            cb.toggled.connect(lambda checked, ln=lap_num: self._on_lap_visibility_toggled(ln, checked))
            self.lap_checkbox_layout.addWidget(cb)
            self.lap_visible[lap_num] = is_checked
        self.lap_checkbox_layout.addStretch()

    def _on_lap_visibility_toggled(self, lap_number, checked):
        # colour and legends depend on the whole checked set -> full re-render
        # (_rerender_preserving_checked) instead of patching items
        self.lap_visible[lap_number] = checked
        self._rerender_preserving_checked()

    def _rerender_preserving_checked(self):
        raise NotImplementedError

    def _restyle_lap_curves(self, lap_number):
        # same width and full opacity for every lap; only colour changes
        items = self.lap_curve_items.get(lap_number)
        style = self._current_styles.get(lap_number)
        if not items or style is None:
            return
        # quiet-lap dash OR palette dash -> dashed
        is_dashed = (self._lap_line_style.get(lap_number) == Qt.PenStyle.DashLine) or (style["dash"] == "dash")
        qt_style = Qt.PenStyle.DashLine if is_dashed else Qt.PenStyle.SolidLine
        for item in items:
            item.setPen(self._pen(style["color"], SCREEN_LAP_WIDTH, qt_style))
        # bridges: lap colour, always dashed and lighter, so an interpolated
        # stretch never reads as measured
        for item in self.lap_bridge_items.get(lap_number, []):
            item.setPen(self._pen(style["color"], SCREEN_LAP_WIDTH, Qt.PenStyle.DashLine, alpha=BRIDGE_ALPHA))

    def _plot_kerb_bridges(self, lap_number, x, kerb, series):
        """series: [(panel_key, y)]. Returns True if anything was bridged."""
        from core.trace_display import kerb_gap_bridge
        items = []
        for key, y in series:
            yb = kerb_gap_bridge(x, y, kerb)
            if np.isfinite(yb).any():
                items.append(self.plots[key].plot(
                    x, yb, connect="finite",
                    pen=self._pen(THRESHOLD_GREY, SCREEN_LAP_WIDTH, Qt.PenStyle.DashLine, alpha=BRIDGE_ALPHA)))
        if items:
            # extend: one lap can carry bridges on several panels and corners
            self.lap_bridge_items.setdefault(lap_number, []).extend(items)
        return bool(items)

    def _plot_cs_graded(self, key, x, y, mark, mode, pen, name, counts):
        """CS panel trace under the kerb-exclusion switch: the lap line without
        the kerb-affected samples (restyled with the lap), those samples as a
        grey overlay (grey mode). counts = [n_affected, n_finite], updated.
        Drawing only -- the samples stay in every computed result."""
        from core.trace_display import near_strike_split
        main, overlay = near_strike_split(y, mark, mode)
        if mark is not None:
            fin = np.isfinite(np.asarray(y, dtype=float))
            counts[0] += int((np.asarray(mark, dtype=bool) & fin).sum())
            counts[1] += int(fin.sum())
        item = self.plots[key].plot(x, main, connect="finite", pen=pen, name=name)
        if overlay is not None:
            self.plots[key].plot(x, overlay, connect="finite",
                                 pen=self._pen(NEAR_STRIKE_COLOR, SCREEN_LAP_WIDTH, Qt.PenStyle.SolidLine))
        return item

    def _restyle_all_laps(self):
        # colour assignment from the current checked set; once per render
        checked = sorted(ln for ln, v in self.lap_visible.items() if v)
        self._current_styles = plot_style.lap_styles(checked)
        for ln in self.lap_curve_items:
            self._restyle_lap_curves(ln)

    # Shading budget (author ruling 2026-10-06), per panel: filled area only for
    # the background tint family (phase / corner-severity context) and ONE
    # exclusion grey (kerb strike or car not up to speed); never stacked --
    # the grey is drawn once as the union over the checked laps and the tint
    # is clipped around it. Events and envelopes are lines or strips.

    def _collect_exclusion(self, x, mask):
        self._exclusion_spans.extend(_contiguous_runs(x, mask))

    def _draw_tint_and_exclusion(self, tint_specs):
        """tint_specs: [(s_start, s_end, colour, alpha)], drawn on every panel
        except where an exclusion span lies."""
        import pyqtgraph as pg
        from core.trace_display import interval_subtract, interval_union

        holes = interval_union(self._exclusion_spans)
        grey = pg.mkColor(NEUTRAL)
        grey.setAlpha(90)
        for s_start, s_end in holes:
            for plot in self.plots.values():
                region = pg.LinearRegionItem(values=(s_start, s_end), brush=pg.mkBrush(grey), movable=False)
                region.setZValue(-5)
                for line in region.lines:
                    line.setPen(pg.mkPen(color=TEXT_DIM, width=1, style=Qt.PenStyle.DotLine))
                plot.addItem(region)
        for s_start, s_end, colour, alpha in tint_specs:
            color = pg.mkColor(colour)
            color.setAlpha(alpha)
            for lo, hi in interval_subtract((s_start, s_end), holes):
                for plot in self.plots.values():
                    region = pg.LinearRegionItem(values=(lo, hi), brush=pg.mkBrush(color), movable=False)
                    region.setZValue(-10)
                    for line in region.lines:
                        line.setPen(pg.mkPen(None))
                    plot.addItem(region)
        return bool(holes)

    def _add_event_bands(self, events, lap_number, corner_id=None):
        # limit-event spans of one lap as a thin strip along the bottom of the
        # axle's CS panel (a line, not a filled band); True if any was drawn
        import pyqtgraph as pg

        if not events or events.get("status") != "computed":
            return False
        y_min, y_max = self._ratio_y_range
        y_strip = y_min + 0.03 * (y_max - y_min)
        drawn = False
        for e in events["events"]:
            if e["lap"] != lap_number or (corner_id is not None and e["corner"] != corner_id):
                continue
            plot = self.plots.get("cs_f" if e["axle"] == "front" else "cs_r")
            if plot is None:
                continue
            item = plot.plot([e["s_start_m"], e["s_end_m"]], [y_strip, y_strip],
                             pen=pg.mkPen(color=TEXT_MUTED, width=EVENT_STRIP_WIDTH))
            item.setZValue(4)
            drawn = True
        return drawn

    def _pen(self, color, width, style, alpha=255):
        import pyqtgraph as pg
        qcolor = pg.mkColor(color)
        qcolor.setAlpha(alpha)
        return pg.mkPen(color=qcolor, width=width, style=style)

    def _add_threshold_line(self, panel_key, value, color, style=Qt.PenStyle.DashLine, name=None):
        # Threshold meaning goes into the legend (name=None -> no entry).
        # InfiniteLine has no .opts, so LegendItem's swatch paint fails silently
        # -> register a zero-point PlotDataItem with the same pen instead.
        import pyqtgraph as pg
        pen = pg.mkPen(color=color, width=1, style=style)
        self.plots[panel_key].addLine(y=value, pen=pen)
        if name is not None:
            # not in lap_curve_items (would get restyled as a lap); plot.clear()
            # removes it
            self.plots[panel_key].plot([], [], pen=pen, name=name)

    def _apply_ratio_y_range(self):
        # CS panels share a fixed y-range (config corner_trace_display): one
        # 1/0/negative scale, deep outliers would squash the 0..1 region. View
        # clip only, arrays untouched.
        from modules.stability_analysis import load_parameters
        margin_cfg = load_parameters().get("corner_trace_display", {})
        y_min = margin_cfg.get("cs_ls_panel_y_min", -3.5)
        y_max = margin_cfg.get("cs_ls_panel_y_max", 1.2)
        for key in self.RATIO_PANEL_KEYS:
            if key in self.plots:
                self.plots[key].setYRange(y_min, y_max, padding=0)


class CornerTraceDialog(_TraceDialogBase):
    """Non-modal per-corner trace window, one instance per form; replots in
    place for another corner.
    """

    WINDOW_TITLE = "Corner Trace"
    # the three verdict-carrying signals; everything else opt-in (author ruling
    # 2026-10-06)
    DEFAULT_PANELS = ("stab", "cs_f", "cs_r")

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg

        # existing trace scaffold becomes one tab, tyre curves a second; tyre plots
        # kept in self.tyre_plots so the s-axis helpers never see them
        outer_layout = self.layout()
        pg_index = outer_layout.indexOf(self.pg_layout)
        outer_layout.removeWidget(self.pg_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.pg_layout, "Traces")

        tyre_tab = QWidget()
        tyre_layout = QVBoxLayout(tyre_tab)
        tyre_layout.setContentsMargins(0, 0, 0, 0)
        tyre_layout.setSpacing(4)

        self.tyre_legend_label = QLabel(TYRE_CURVE_LEGEND_TEXT)
        self.tyre_legend_label.setWordWrap(True)
        self.tyre_legend_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        tyre_layout.addWidget(self.tyre_legend_label)

        self.tyre_pg_layout = pg.GraphicsLayoutWidget()
        self.tyre_pg_layout.setBackground(PANEL)
        tyre_layout.addWidget(self.tyre_pg_layout)

        # track map above the tyre curves, full width, equal aspect (plan view)
        self.track_map_plot = self.tyre_pg_layout.addPlot(row=0, col=0, colspan=2)
        self.track_map_plot.setTitle("Track map", color=TEXT_MUTED, size='9pt')
        self.track_map_plot.setAspectLocked(True)
        self.track_map_plot.hideAxis('left')
        self.track_map_plot.hideAxis('bottom')
        self.track_map_legend = _style_new_legend(self.track_map_plot)
        self.tyre_pg_layout.nextRow()

        self.tyre_plots = {}
        for col, (axle, title) in enumerate((("front", "Front axle"), ("rear", "Rear axle"))):
            plot = self.tyre_pg_layout.addPlot(row=1, col=col)
            plot.setTitle(title, color=TEXT_MUTED, size='9pt')
            # left label set per render: Fy / Fz or Fy in N (_tyre_curve_basis)
            plot.setLabel('bottom', 'Slip angle (deg)', color='#888', size='8pt')
            plot.showGrid(x=True, y=True, alpha=0.15)
            # no equal-axis scaling. Legend outside the plot: standalone
            # LegendItem in its own row, filled by addItem()
            self.tyre_plots[axle] = plot
        self.tyre_pg_layout.nextRow()

        self.tyre_legends = {}
        for col, axle in enumerate(("front", "rear")):
            legend = pg.LegendItem(labelTextSize=LEGEND_FONT_PT)
            legend.setBrush(pg.mkBrush(PANEL_ALT))
            legend.setLabelTextColor(TEXT_MUTED)
            self.tyre_pg_layout.addItem(legend, row=2, col=col)
            self.tyre_legends[axle] = legend
        # short legend row
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(0, 3)
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(1, 6)
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(2, 1)

        self.tabs.addTab(tyre_tab, "Tyre Curves")

        # "Export figure" and "Export verdict traces" both read the corner on
        # screen, from one _export_data cache filled in show_corner
        self._export_data = None
        # checked-set-dependent views all rebuilt from this context
        self._render_ctx = None
        export_row = QHBoxLayout()
        self.export_figure_btn = QPushButton("Export figure")
        self.export_figure_btn.clicked.connect(self._on_export_figure_clicked)
        self.export_verdict_btn = QPushButton("Export verdict traces")
        self.export_verdict_btn.clicked.connect(self._on_export_verdict_clicked)
        export_row.addWidget(self.export_figure_btn)
        export_row.addWidget(self.export_verdict_btn)
        export_row.addStretch(1)

        # opt-in panels: one checkbox per non-default panel, choice kept for
        # the life of the window
        self._visible_panels = set(self.DEFAULT_PANELS)
        panel_row = QHBoxLayout()
        more_label = QLabel("More panels:")
        more_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
        panel_row.addWidget(more_label)
        for label, key in self.panel_titles:
            if key in self.DEFAULT_PANELS:
                continue
            cb = QCheckBox(label)
            cb.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
            cb.toggled.connect(lambda checked, k=key: self._on_panel_toggled(k, checked))
            panel_row.addWidget(cb)
        panel_row.addStretch(1)
        self._layout_panels(self._visible_panels)

        outer_layout.insertWidget(pg_index, self.tabs)
        outer_layout.insertLayout(pg_index, panel_row)
        outer_layout.insertLayout(pg_index + 2, export_row)

    def _on_panel_toggled(self, key, checked):
        if checked:
            self._visible_panels.add(key)
        else:
            self._visible_panels.discard(key)
        self._layout_panels(self._visible_panels)

    def _add_phase_bands(self, corner, t, s_m, worst_phase):
        # the tint family of this window: returns the phase tints as specs (drawn
        # with the exclusion grey, _draw_tint_and_exclusion); the apex line now
        tint_specs = [(s_start, s_end, BAD if phase == worst_phase else BORDER,
                       70 if phase == worst_phase else 30)
                      for phase, s_start, s_end in _phase_bands_for_lap(corner, t, s_m)]
        import pyqtgraph as pg

        apex_s = corner.get("apex_lap_distance_m")
        if apex_s is not None:
            for plot in self.plots.values():
                apex_line = pg.InfiniteLine(
                    pos=apex_s, angle=90,
                    pen=pg.mkPen(color=TEXT_DIM, width=1, style=Qt.PenStyle.DotLine),
                )
                plot.addItem(apex_line)
        return tint_specs

    def _plot_cross_lap_envelopes(self, instances, laps_by_number, t, s_m, bracket_start_m, bracket_end_m,
                                  margin_before_m, margin_after_m, series, cfg):
        # p10 / p90 over ALL analysis laps of this corner as two thin lines per
        # panel (shading budget: no fill); the checked laps draw on top
        from core.trace_display import cross_lap_envelope
        if not cfg:
            return
        slices = []
        for c in instances:
            lap = laps_by_number[c["lap_number"]]
            sl, _a, _b = _extend_slice_with_margin(t, s_m, lap["start_time"], lap["end_time"],
                                                   bracket_start_m, bracket_end_m, margin_before_m, margin_after_m)
            if sl.stop > sl.start:
                slices.append(sl)
        pen = self._pen(TEXT_DIM, 1, Qt.PenStyle.DashLine)
        lo_pct, hi_pct = cfg["percentiles"]
        for key, arr in series.items():
            env = cross_lap_envelope([(s_m[sl], arr[sl]) for sl in slices], cfg)
            if env is None:
                continue
            grid, p_lo, p_hi = env
            self.plots[key].plot(grid, p_lo, connect="finite", pen=pen,
                                 name=f"All analysis laps, p{lo_pct} / p{hi_pct}").setZValue(-1)
            self.plots[key].plot(grid, p_hi, connect="finite", pen=pen).setZValue(-1)

    def _clear_tyre_curves(self):
        for plot in self.tyre_plots.values():
            plot.clear()
        for legend in self.tyre_legends.values():
            legend.clear()

    def _render_tyre_curves(self, instances, laps_by_number, bracket_start_m, bracket_end_m,
                             t, s_m, slip, forces, cs, kerb_mask, params, session_mask,
                             sideslip_source=None, fit_manifest=None, sample_rate_hz=None,
                             kerb_mode="grey", cs_axle_loads=None):
        """Tyre Curves tab: alpha vs the force the CS was estimated on (Fy / Fz
        under fit L, else Fy; _tyre_curve_basis) per axle over the canonical
        window (no margin). Plot design after the chair performance_analysis
        tooling (internal); no code copied.

        Checked laps only. Lap colour, filled = clean, hollow = kerb-flagged.
        Beyond the peak a lap's cloud folds back over the reference line.
        Reference line = one slope per axle, median C_linear_ref over valid
        samples (already NaN on kerb / not moving).
        Faint cloud of the checked laps (whole lap, moving, no kerb) first;
        estimation window (hollow black ring) + tangent on top.
        Auto-fit mode: the fitted Pacejka curve over the visited alpha
        range -- shows the saturation the linear reference can't.
        """
        import pyqtgraph as pg
        from core.trace_display import kerb_scatter_split
        from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples

        self._clear_tyre_curves()
        if slip is None or forces is None:
            return

        # unchecked laps contribute nothing, incl. to the pooled reference/window
        instances = self._checked_instances(instances)
        styles = plot_style.lap_styles(c["lap_number"] for c in instances)
        basis = _tyre_curve_basis(cs, cs_axle_loads)
        if basis is None:
            self.tyre_legend_label.setText(
                "Tyre curves need the axle loads of a fresh analysis -- re-run Analyse.")
            return
        self.tyre_legend_label.setText(TYRE_CURVE_LEGEND_TEXT)
        for plot in self.tyre_plots.values():
            plot.setLabel('left', basis["y_label"], color='#888', size='8pt')

        def _alpha_brush(color, alpha_frac):
            qcolor = pg.mkColor(color)
            qcolor.setAlpha(int(round(alpha_frac * 255)))
            return pg.mkBrush(qcolor)

        def _alpha_pen(color, alpha_frac, width):
            qcolor = pg.mkColor(color)
            qcolor.setAlpha(int(round(alpha_frac * 255)))
            return pg.mkPen(qcolor, width=width)

        axle_specs = (
            ("front", slip.get("alpha_f_filt"), _tyre_curve_force(basis, "front", forces.get("Fy_f_filt")),
             cs.get("C_linear_ref_f") if cs is not None else None,
             cs.get("CS_ratio_f") if cs is not None else None,
             cs.get("C_alpha_f") if cs is not None else None),
            ("rear", slip.get("alpha_r_filt"), _tyre_curve_force(basis, "rear", forces.get("Fy_r_filt")),
             cs.get("C_linear_ref_r") if cs is not None else None,
             cs.get("CS_ratio_r") if cs is not None else None,
             cs.get("C_alpha_r") if cs is not None else None),
        )

        for axle, alpha_arr, Fy_arr, ref_arr, cs_ratio_arr, c_alpha_arr in axle_specs:
            mark_arr = cs.get("near_strike_" + axle[0]) if cs is not None else None
            plot = self.tyre_plots[axle]
            legend = self.tyre_legends[axle]
            if alpha_arr is None or Fy_arr is None:
                continue

            # session cloud first, underneath
            if session_mask is not None:
                session_valid = session_mask & np.isfinite(alpha_arr) & np.isfinite(Fy_arr)
                if session_valid.any():
                    session_item = plot.plot(
                        np.degrees(alpha_arr[session_valid]), Fy_arr[session_valid],
                        pen=None, symbol='o', symbolSize=SESSION_CLOUD_SIZE_SCREEN,
                        symbolBrush=pg.mkBrush(SESSION_CLOUD_COLOR_SCREEN), symbolPen=None,
                    )
                    session_item.setZValue(-1)
                    legend.addItem(session_item, "Checked laps, whole lap")

            pooled_alpha_rad = []
            pooled_ref = []

            for c in instances:
                lap = laps_by_number.get(c["lap_number"])
                if lap is None:
                    continue
                sl, _start_s, _end_s = _extend_slice_with_margin(
                    t, s_m, lap["start_time"], lap["end_time"],
                    bracket_start_m, bracket_end_m, 0.0, 0.0,
                )
                if sl.stop <= sl.start:
                    continue

                lap_alpha = alpha_arr[sl]
                lap_Fy = Fy_arr[sl]
                valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
                clean, kerbed = kerb_scatter_split(
                    valid, kerb_mask[sl] if kerb_mask is not None else None,
                    mark_arr[sl] if mark_arr is not None else None, kerb_mode)
                if not valid.any():
                    continue
                pooled_alpha_rad.append(lap_alpha[valid])

                color = styles[c["lap_number"]]["color"]
                if clean.any():
                    item = plot.plot(np.degrees(lap_alpha[clean]), lap_Fy[clean], pen=None, symbol='o',
                                      symbolSize=LAP_SAMPLE_SIZE_SCREEN, symbolBrush=_alpha_brush(color, LAP_SAMPLE_ALPHA),
                                      symbolPen=None)
                    legend.addItem(item, f"Lap {c['lap_number']}")
                if kerbed.any():
                    plot.plot(np.degrees(lap_alpha[kerbed]), lap_Fy[kerbed], pen=None, symbol='o',
                              symbolSize=LAP_SAMPLE_SIZE_SCREEN, symbolBrush=None,
                              symbolPen=_alpha_pen(color, LAP_SAMPLE_ALPHA, 1.2))

                if ref_arr is not None:
                    lap_ref = ref_arr[sl][valid]
                    finite_ref = lap_ref[np.isfinite(lap_ref)]
                    if finite_ref.size:
                        pooled_ref.append(finite_ref)

            max_abs_alpha = (float(np.max(np.abs(np.concatenate(pooled_alpha_rad))))
                              if pooled_alpha_rad else 0.0)

            if pooled_ref and max_abs_alpha > 0:
                ref_slope = float(np.median(np.concatenate(pooled_ref)))
                if ref_slope > 0:
                    x_line_rad = np.array([-max_abs_alpha, max_abs_alpha])
                    y_line = ref_slope * x_line_rad
                    item = plot.plot(
                        np.degrees(x_line_rad), y_line,
                        pen=pg.mkPen(color=THRESHOLD_GREY, width=2, style=Qt.PenStyle.SolidLine),
                    )
                    legend.addItem(item, "Linear reference")

            # fitted curve evaluated in rad, x shown in deg (same as the reference
            # line) -- avoids the per-rad vs deg tangent trap
            if (sideslip_source == "ekf_auto_pacejka"
                    and fit_manifest is not None and max_abs_alpha > 0):
                axle_fit = fit_manifest.get("axles", {}).get(axle)
                if axle_fit is not None:
                    alpha_grid_rad = np.linspace(-max_abs_alpha, max_abs_alpha, 200)
                    from modules.tyre_model_pacejka import pacejka_lateral_force
                    fy_grid = pacejka_lateral_force(
                        alpha_grid_rad, axle_fit["B"], axle_fit["C"], _fitted_curve_d(basis, axle_fit), axle_fit["E"]
                    )
                    item = plot.plot(
                        np.degrees(alpha_grid_rad), fy_grid,
                        pen=pg.mkPen(color=FITTED_CURVE_COLOR, width=2, style=Qt.PenStyle.SolidLine),
                    )
                    legend.addItem(item, "Fitted tyre curve")

            # estimation window + tangent, same computation as the export
            if cs_ratio_arr is not None:
                wp = _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m,
                                      bracket_start_m, bracket_end_m)
                if wp is not None:
                    min_window = resolve_cs_min_window_samples(params, sample_rate_hz)
                    min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]
                    max_window_m = params["stability_estimation"]["cs_max_window_m"]
                    idx = wp["index"]
                    start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span,
                                                         s_m=s_m, max_window_m=max_window_m)
                    window_sl = slice(start, idx)
                    if window_sl.stop > window_sl.start:
                        window_item = plot.plot(np.degrees(alpha_arr[window_sl]), Fy_arr[window_sl], pen=None,
                                                 symbol='o', symbolSize=WINDOW_RING_SIZE_SCREEN, symbolBrush=None,
                                                 symbolPen=pg.mkPen(WINDOW_RING_COLOR, width=1.4))
                        window_item.setZValue(10)
                        legend.addItem(window_item, "Estimation window")
                    cs_slope = float(c_alpha_arr[idx]) if c_alpha_arr is not None else None
                    if cs_slope is not None and np.isfinite(cs_slope) and max_abs_alpha > 0:
                        x0 = np.degrees(alpha_arr[idx])
                        y0 = Fy_arr[idx]
                        slope_per_deg = cs_slope * (np.pi / 180.0)
                        span = max(0.5, 0.15 * max_abs_alpha * (180.0 / np.pi))
                        xs = np.array([x0 - span, x0 + span])
                        ys = y0 + slope_per_deg * (xs - x0)
                        tangent_item = plot.plot(xs, ys, pen=pg.mkPen(color=TEXT, width=1, style=Qt.PenStyle.DashLine))
                        fmt = ".1f" if basis["normalised"] else ".0f"
                        legend.addItem(tangent_item, f"Tangent CS={cs_slope:{fmt}} {basis['slope_unit']}")

    def _render_track_map(self, representative, bracket_start_m, bracket_end_m, t, s_m, state,
                           cs, slip, params, instances, laps_by_number):
        """Track map: representative lap's GPS trace as grey context, one bracket
        polyline per checked lap in its colour, each axle's worst-phase window as
        a hollow black ring (front solid, rear dotted). Windows located with
        reconstruct_cs_window_start, pooled over checked laps only; the outline
        stays from the representative lap regardless.
        Returns the track_map geometry figure_render expects, or None without GPS.
        """
        import pyqtgraph as pg
        from modules.geo import project_latlon_to_xy
        from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples

        self.track_map_plot.clear()
        gps_lat = state.get("gps_lat")
        gps_lon = state.get("gps_lon")
        origin_lat = state.get("gps_origin_lat")
        origin_lon = state.get("gps_origin_lon")
        if gps_lat is None or gps_lon is None:
            return None

        x, y = project_latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon)

        lap = laps_by_number[representative["lap_number"]]
        lap_sl = slice(int(np.searchsorted(t, lap["start_time"], side="left")),
                        int(np.searchsorted(t, lap["end_time"], side="right")))
        lap_xy = (x[lap_sl], y[lap_sl])

        checked_instances = self._checked_instances(instances)
        styles = plot_style.lap_styles(c["lap_number"] for c in checked_instances)

        brackets_by_lap = []
        for c in checked_instances:
            c_lap = laps_by_number.get(c["lap_number"])
            if c_lap is None:
                continue
            bracket_sl, _start_s, _end_s = _extend_slice_with_margin(
                t, s_m, c_lap["start_time"], c_lap["end_time"], bracket_start_m, bracket_end_m, 0.0, 0.0,
            )
            if bracket_sl.stop <= bracket_sl.start:
                continue
            brackets_by_lap.append({
                "lap_number": c["lap_number"], "xy": (x[bracket_sl], y[bracket_sl]),
                **styles[c["lap_number"]],
            })

        min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
        min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]
        max_window_m = params["stability_estimation"]["cs_max_window_m"]

        def _window_xy(cs_ratio_arr, alpha_arr):
            if cs_ratio_arr is None or alpha_arr is None:
                return None
            wp = _worst_cs_phase(cs_ratio_arr, checked_instances, laps_by_number, t, s_m,
                                  bracket_start_m, bracket_end_m)
            if wp is None:
                return None
            start = reconstruct_cs_window_start(alpha_arr, wp["index"], min_window, min_span,
                                                 s_m=s_m, max_window_m=max_window_m)
            window_sl = slice(start, wp["index"])
            if window_sl.stop <= window_sl.start:
                return None
            return x[window_sl], y[window_sl]

        window_f_xy = _window_xy(cs.get("CS_ratio_f") if cs else None, slip.get("alpha_f_filt") if slip else None)
        window_r_xy = _window_xy(cs.get("CS_ratio_r") if cs else None, slip.get("alpha_r_filt") if slip else None)

        self.track_map_plot.plot(lap_xy[0], lap_xy[1], pen=pg.mkPen(TRACK_BG_COLOR, width=1), name="Lap trace")
        for entry in brackets_by_lap:
            bx, by = entry["xy"]
            qt_style = Qt.PenStyle.DashLine if entry["dash"] == "dash" else Qt.PenStyle.SolidLine
            self.track_map_plot.plot(bx, by, pen=pg.mkPen(entry["color"], width=3, style=qt_style),
                                      name=f"Lap {entry['lap_number']} bracket")
        if window_f_xy is not None:
            self.track_map_plot.plot(window_f_xy[0], window_f_xy[1], pen=None, symbol='o', symbolSize=10,
                                      symbolBrush=None, symbolPen=pg.mkPen(WINDOW_RING_COLOR, width=1.6),
                                      name="Front window")
        if window_r_xy is not None:
            self.track_map_plot.plot(window_r_xy[0], window_r_xy[1], pen=None, symbol='o', symbolSize=10,
                                      symbolBrush=None, symbolPen=pg.mkPen(WINDOW_RING_COLOR, width=1.6,
                                                                            style=Qt.PenStyle.DotLine),
                                      name="Rear window")

        return {"lap_xy": lap_xy, "brackets_by_lap": brackets_by_lap,
                "window_f_xy": window_f_xy, "window_r_xy": window_r_xy}

    def _build_tyre_curve_export(self, axle, alpha_arr, Fy_arr, ref_arr, cs_ratio_arr, c_alpha_arr,
                                  kerb_mask, session_mask, instances, laps_by_number,
                                  bracket_start_m, bracket_end_m, t, s_m, params,
                                  sideslip_source, fit_manifest, sample_rate_hz,
                                  mark_arr=None, kerb_mode="grey", basis=None):
        """One axle's tyre_curves entry for figure_render: session scatter, corner
        scatter per checked lap (filled/hollow), worst-phase window, linear
        reference, auto-fit curve, and a tangent through the worst-phase sample
        at slope CS * pi/180 per deg. Fy_arr is already on the basis' force
        (Fy / Fz or Fy); the entry carries its axis label.
        """
        from core.trace_display import kerb_scatter_split
        session_valid = session_mask & np.isfinite(alpha_arr) & np.isfinite(Fy_arr)
        session_xy = (np.degrees(alpha_arr[session_valid]), Fy_arr[session_valid],
                      kerb_mask[session_valid] if kerb_mask is not None else None)

        styles = plot_style.lap_styles(c["lap_number"] for c in instances)
        pooled_alpha = []
        pooled_ref = []
        corner_by_lap = []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None:
                continue
            sl, _s, _e = _extend_slice_with_margin(
                t, s_m, lap["start_time"], lap["end_time"], bracket_start_m, bracket_end_m, 0.0, 0.0,
            )
            if sl.stop <= sl.start:
                continue
            lap_alpha, lap_Fy = alpha_arr[sl], Fy_arr[sl]
            valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
            clean, kerbed = kerb_scatter_split(
                valid, kerb_mask[sl] if kerb_mask is not None else None,
                mark_arr[sl] if mark_arr is not None else None, kerb_mode)
            if valid.any():
                pooled_alpha.append(lap_alpha[valid])
            corner_by_lap.append({
                "lap_number": c["lap_number"], **styles[c["lap_number"]],
                "clean_xy": (np.degrees(lap_alpha[clean]), lap_Fy[clean]) if clean.any() else None,
                "kerb_xy": (np.degrees(lap_alpha[kerbed]), lap_Fy[kerbed]) if kerbed.any() else None,
            })
            if ref_arr is not None:
                lap_ref = ref_arr[sl][valid]
                finite_ref = lap_ref[np.isfinite(lap_ref)]
                if finite_ref.size:
                    pooled_ref.append(finite_ref)

        pooled_alpha_rad = np.concatenate(pooled_alpha) if pooled_alpha else np.array([])
        max_abs_alpha = float(np.max(np.abs(pooled_alpha_rad))) if pooled_alpha_rad.size else 0.0

        ref_line = None
        if pooled_ref and max_abs_alpha > 0:
            ref_slope = float(np.median(np.concatenate(pooled_ref)))
            if ref_slope > 0:
                x_line_rad = np.array([-max_abs_alpha, max_abs_alpha])
                ref_line = (np.degrees(x_line_rad), ref_slope * x_line_rad)

        fitted_line = None
        if (sideslip_source == "ekf_auto_pacejka"
                and fit_manifest is not None and max_abs_alpha > 0):
            axle_fit = fit_manifest.get("axles", {}).get(axle)
            if axle_fit is not None:
                alpha_grid_rad = np.linspace(-max_abs_alpha, max_abs_alpha, 200)
                from modules.tyre_model_pacejka import pacejka_lateral_force
                fy_grid = pacejka_lateral_force(alpha_grid_rad, axle_fit["B"], axle_fit["C"],
                                                 _fitted_curve_d(basis, axle_fit), axle_fit["E"])
                fitted_line = (np.degrees(alpha_grid_rad), fy_grid, "Fitted tyre curve")

        window_xy = None
        tangent_line = None
        if cs_ratio_arr is not None:
            from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples
            wp = _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m,
                                  bracket_start_m, bracket_end_m)
            if wp is not None:
                min_window = resolve_cs_min_window_samples(params, sample_rate_hz)
                min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]
                max_window_m = params["stability_estimation"]["cs_max_window_m"]
                idx = wp["index"]
                start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span,
                                                     s_m=s_m, max_window_m=max_window_m)
                window_sl = slice(start, idx)
                if window_sl.stop > window_sl.start:
                    window_xy = (np.degrees(alpha_arr[window_sl]), Fy_arr[window_sl])
                cs_slope = float(c_alpha_arr[idx]) if c_alpha_arr is not None else None
                if cs_slope is not None and np.isfinite(cs_slope):
                    x0 = np.degrees(alpha_arr[idx])
                    y0 = Fy_arr[idx]
                    slope_per_deg = cs_slope * (np.pi / 180.0)
                    span = max(0.5, 0.15 * max_abs_alpha * (180.0 / np.pi)) if max_abs_alpha > 0 else 1.0
                    xs = np.array([x0 - span, x0 + span])
                    ys = y0 + slope_per_deg * (xs - x0)
                    fmt = ".1f" if basis["normalised"] else ".0f"
                    tangent_line = (xs, ys, f"Tangent CS={cs_slope:{fmt}} {basis['slope_unit']}")

        return {
            "session_xy": session_xy, "corner_by_lap": corner_by_lap, "window_xy": window_xy,
            "linear_ref_line": ref_line, "fitted_line": fitted_line, "tangent_line": tangent_line,
            "y_label": basis["y_label"], "normalised": basis["normalised"],
        }

    def _on_export_figure_clicked(self):
        self._export("corner")

    def _on_export_verdict_clicked(self):
        self._export("verdict")

    def _export(self, kind):
        if self._export_data is None:
            QMessageBox.information(self, "Export figure", "Analyse a corner first -- nothing to export yet.")
            return
        from core import figure_render, plot_style

        data = self._export_data
        default_name = f"{data['corner_label']}_{'figure' if kind == 'corner' else 'verdict_traces'}.png"
        path, _ = QFileDialog.getSaveFileName(self, "Export figure", default_name, "PNG image (*.png)")
        if not path:
            return
        try:
            if kind == "corner":
                fig = figure_render.render_corner_figure(
                    data["corner_label"], data["laps"], data["thresholds"],
                    data["tyre_curves"], data["track_map"], theme=plot_style.PRINT,
                    near_strike=data.get("near_strike"),
                )
                figure_render.save_png(fig, path)
                saved_paths = [path]
            else:
                # returns a list of figures -> save all
                figs = figure_render.render_verdict_traces_figure(
                    data["corner_label"], data["laps"], data["thresholds"], theme=plot_style.PRINT,
                    near_strike=data.get("near_strike"),
                )
                saved_paths = []
                base, ext = path.rsplit(".", 1) if "." in path else (path, "png")
                for i, fig in enumerate(figs, start=1):
                    fig_path = path if len(figs) == 1 else f"{base}_{i}.{ext}"
                    figure_render.save_png(fig, fig_path)
                    saved_paths.append(fig_path)
        except Exception as e:
            from core.error_text import friendly_error_text
            QMessageBox.warning(self, "Export figure", f"Could not export figure ({friendly_error_text(e)}).")
            return
        QMessageBox.information(self, "Export figure", "Saved to " + ", ".join(saved_paths))

    def _checked_instances(self, instances):
        # the one place that decides which instances count (checked laps)
        return [c for c in instances if self.lap_visible.get(c["lap_number"], True)]

    def _refresh_checked_dependent_views(self):
        """Rebuild everything pooled over the checked set: tyre-curve reference,
        window highlights, export cache. Runs on the first render and on every
        toggle -- one code path.
        """
        ctx = self._render_ctx
        if ctx is None:
            return
        track_map = None
        if ctx["bracket_start_m"] is not None and ctx["bracket_end_m"] is not None:
            self._render_tyre_curves(
                ctx["instances"], ctx["laps_by_number"], ctx["bracket_start_m"], ctx["bracket_end_m"],
                ctx["t"], ctx["s_m"], ctx["slip"], ctx["forces"], ctx["cs"], ctx["kerb_mask"], ctx["params"],
                ctx["session_mask"],
                sideslip_source=ctx["sideslip_source"], fit_manifest=ctx["fit_manifest"],
                sample_rate_hz=ctx["state"]["sample_rate_hz"], kerb_mode=ctx["near_strike"]["mode"],
                cs_axle_loads=ctx["cs_axle_loads"],
            )
            # track map geometry also needed by the export
            track_map = self._render_track_map(
                ctx["representative"], ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"],
                ctx["state"], ctx["cs"], ctx["slip"], ctx["params"], ctx["instances"], ctx["laps_by_number"],
            )
        self._build_export_data(track_map)

    def _build_export_data(self, track_map):
        # export = exactly the checked set, tyre geometry rebuilt from it
        ctx = self._render_ctx
        basis = _tyre_curve_basis(ctx["cs"], ctx["cs_axle_loads"]) if ctx is not None else None
        if (ctx is None or ctx["bracket_start_m"] is None or ctx["bracket_end_m"] is None
                or ctx["slip"] is None or ctx["forces"] is None or basis is None):
            self._export_data = None
            return

        checked_instances = self._checked_instances(ctx["instances"])
        checked_lap_numbers = {c["lap_number"] for c in checked_instances}
        laps = [lap for lap in ctx["all_export_laps"] if lap["lap_number"] in checked_lap_numbers]

        slip, forces, cs = ctx["slip"], ctx["forces"], ctx["cs"]
        tyre_curves = {
            "front": self._build_tyre_curve_export(
                "front", slip.get("alpha_f_filt"), _tyre_curve_force(basis, "front", forces.get("Fy_f_filt")),
                cs.get("C_linear_ref_f") if cs is not None else None,
                cs.get("CS_ratio_f") if cs is not None else None,
                cs.get("C_alpha_f") if cs is not None else None,
                ctx["kerb_mask"], ctx["session_mask"], checked_instances, ctx["laps_by_number"],
                ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"], ctx["params"],
                ctx["sideslip_source"], ctx["fit_manifest"], ctx["state"]["sample_rate_hz"],
                mark_arr=cs.get("near_strike_f") if cs is not None else None,
                kerb_mode=ctx["near_strike"]["mode"], basis=basis,
            ),
            "rear": self._build_tyre_curve_export(
                "rear", slip.get("alpha_r_filt"), _tyre_curve_force(basis, "rear", forces.get("Fy_r_filt")),
                cs.get("C_linear_ref_r") if cs is not None else None,
                cs.get("CS_ratio_r") if cs is not None else None,
                cs.get("C_alpha_r") if cs is not None else None,
                ctx["kerb_mask"], ctx["session_mask"], checked_instances, ctx["laps_by_number"],
                ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"], ctx["params"],
                ctx["sideslip_source"], ctx["fit_manifest"], ctx["state"]["sample_rate_hz"],
                mark_arr=cs.get("near_strike_r") if cs is not None else None,
                kerb_mode=ctx["near_strike"]["mode"], basis=basis,
            ),
        }
        self._export_data = {
            "corner_label": ctx["corner_label"], "laps": laps, "thresholds": ctx["thresholds"],
            "tyre_curves": tyre_curves, "track_map": track_map, "near_strike": ctx["near_strike"],
        }

    def _rerender_preserving_checked(self):
        # full re-render with the current checkbox state
        if self._last_show_args is not None:
            self.show_corner(*self._last_show_args, preserve_visible=dict(self.lap_visible))

    def show_corner(self, summary, stability_result, parsed_data, preserve_visible=None):
        """Repopulate for summary's stable_corner_id. summary = the clicked lap's
        corner summary: its lap is the representative lap (phase bands, map
        outline, initial x range) and its phase medians pick the tinted phase.
        """
        from modules.stability_analysis import load_parameters

        self._last_show_args = (summary, stability_result, parsed_data)

        for plot in self.plots.values():
            plot.clear()
        self._clear_tyre_curves()
        self.lap_curve_items = {}
        self.lap_bridge_items = {}
        self._render_ctx = None
        self._exclusion_spans = []

        stable_corner_id = summary["stable_corner_id"]
        state = stability_result.get("state")
        cs = stability_result.get("cs")
        stab = stability_result.get("stab")
        corners = stability_result.get("corners")
        # slip/forces: tyre tab only; missing on older cached results -> empty tab
        slip = stability_result.get("slip")
        forces = stability_result.get("forces")
        if state is None or cs is None or stab is None or corners is None:
            self.header_label.setText(
                f"C{stable_corner_id}: raw sample arrays aren't available for this render "
                f"(cached summaries only) -- re-run Analyse to enable the trace view."
            )
            self._rebuild_lap_checkboxes([], None)
            self.show()
            self.raise_()
            return

        t = state["time"]
        s_m = state.get("s_m")
        instances = [c for c in corners if c.get("stable_corner_id") == stable_corner_id]
        laps_by_number = {l["lap_number"]: l for l in parsed_data.get("laps", [])}
        instances = [c for c in instances
                     if laps_by_number.get(c["lap_number"], {}).get("is_valid_for_analysis")]
        instances.sort(key=lambda c: c["lap_number"])

        if s_m is None or not instances:
            self.header_label.setText(
                f"C{stable_corner_id}: no lap_distance channel, or no valid-lap instances "
                f"of this corner -- nothing to trace."
            )
            self._rebuild_lap_checkboxes([], None)
            self.show()
            self.raise_()
            return

        params = load_parameters()
        margin_cfg = params.get("corner_trace_display", {})
        margin_before_m = margin_cfg.get("margin_before_m", 100.0)
        margin_after_m = margin_cfg.get("margin_after_m", 50.0)
        bridge_on = margin_cfg.get("kerb_gap_bridge", True)
        # kerb exclusion (display only): the same switch drives the export
        ns_mode = margin_cfg.get("kerb_exclusion_display", "grey")
        ns_f, ns_r = cs.get("near_strike_f"), cs.get("near_strike_r")
        ns_counts = [0, 0]
        any_bridge = False

        v_kmh = state["v_mps"] * 3.6
        cs_f = cs["CS_ratio_f"]
        cs_r = cs["CS_ratio_r"]
        stab_obs = stab["stability_observed_Nm_per_deg"]
        kerb_mask = state.get("kerb_mask")
        moving_mask = state.get("moving_mask")

        selected_lap = summary["lap_number"]
        representative = next((c for c in instances if c["lap_number"] == selected_lap), instances[0])
        phase_tints = self._add_phase_bands(representative, t, s_m, worst_phase=_worst_stab_phase(summary))
        # default view (author ruling 2026-10-06): the selected lap full-strength
        # over the cross-lap envelope of all analysis laps; other laps opt-in
        default_checked_laps = {selected_lap}
        self._rebuild_lap_checkboxes(instances, selected_lap, default_checked_laps=default_checked_laps,
                                      preserve_visible=preserve_visible)

        # canonical bracket from the summary, raw corner dict as fallback (same value)
        bracket_start_m = summary.get("bracket_start_m")
        bracket_end_m = summary.get("bracket_end_m")
        if bracket_start_m is None or bracket_end_m is None:
            bracket_start_m = representative.get("bracket_start_m")
            bracket_end_m = representative.get("bracket_end_m")

        any_kerb = False
        any_not_moving = False
        any_event = False
        rep_start_s, rep_end_s = None, None
        # checked laps only
        checked_instances = self._checked_instances(instances)
        # export copy of exactly what's on screen
        export_laps = []
        self._lap_line_style = {}
        for c in checked_instances:
            is_selected = (c["lap_number"] == selected_lap)  # x-range target only
            is_quiet = "canonical_quiet" in c.get("warnings", [])
            self._lap_line_style[c["lap_number"]] = Qt.PenStyle.DashLine if is_quiet else Qt.PenStyle.SolidLine

            lap = laps_by_number[c["lap_number"]]
            sl, start_s, end_s = _extend_slice_with_margin(
                t, s_m, lap["start_time"], lap["end_time"],
                bracket_start_m, bracket_end_m,
                margin_before_m, margin_after_m,
            )
            if sl.stop <= sl.start:
                continue
            if is_selected:
                rep_start_s, rep_end_s = start_s, end_s

            x = s_m[sl]
            order = np.argsort(x)  # monotonic within a lap; guard anyway
            x = x[order]

            if kerb_mask is not None:
                lap_kerb = kerb_mask[sl][order]
                if lap_kerb.any():
                    any_kerb = True
                    self._collect_exclusion(x, lap_kerb)
            any_event |= self._add_event_bands(stability_result.get("events"), c["lap_number"],
                                               corner_id=stable_corner_id)

            if moving_mask is not None:
                lap_not_moving = ~moving_mask[sl][order]
                if lap_not_moving.any():
                    any_not_moving = True
                    self._collect_exclusion(x, lap_not_moving)

            # placeholder pen; _restyle_all_laps sets colour. name on the real item --
            # only checked laps are plotted
            style = self._lap_line_style[c["lap_number"]]
            lap_name = f"Lap {c['lap_number']}"
            placeholder_pen_kwargs = dict(color=THRESHOLD_GREY, width=SCREEN_LAP_WIDTH, style=style)
            curve_items = [
                self.plots["speed"].plot(x, v_kmh[sl][order], connect="finite",
                                          pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["stab"].plot(x, stab_obs[sl][order], connect="finite",
                                         pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self._plot_cs_graded("cs_f", x, cs_f[sl][order],
                                     ns_f[sl][order] if ns_f is not None else None, ns_mode,
                                     self._pen(**placeholder_pen_kwargs), lap_name, ns_counts),
                self._plot_cs_graded("cs_r", x, cs_r[sl][order],
                                     ns_r[sl][order] if ns_r is not None else None, ns_mode,
                                     self._pen(**placeholder_pen_kwargs), lap_name, ns_counts),
            ]
            self.lap_curve_items[c["lap_number"]] = curve_items
            if bridge_on and kerb_mask is not None:
                any_bridge |= self._plot_kerb_bridges(
                    c["lap_number"], x, kerb_mask[sl][order],
                    [("stab", stab_obs[sl][order]), ("cs_f", cs_f[sl][order]), ("cs_r", cs_r[sl][order])])
            export_laps.append({
                "lap_number": c["lap_number"], "selected": is_selected,
                "s": x, "v_kmh": v_kmh[sl][order],
                "cs_f": cs_f[sl][order], "cs_r": cs_r[sl][order],
                "stab": stab_obs[sl][order],
                # export draws kerb bridges only when this is set (same flag)
                "kerb": kerb_mask[sl][order] if (bridge_on and kerb_mask is not None) else None,
                "near_strike_f": ns_f[sl][order] if ns_f is not None else None,
                "near_strike_r": ns_r[sl][order] if ns_r is not None else None,
            })

        self._draw_tint_and_exclusion(phase_tints)
        self._plot_cross_lap_envelopes(
            instances, laps_by_number, t, s_m, bracket_start_m, bracket_end_m,
            margin_before_m, margin_after_m,
            {"cs_f": cs_f, "cs_r": cs_r, "stab": stab_obs}, margin_cfg.get("cs_envelope"))

        # legend = checked laps + threshold lines
        self._laps_by_number = laps_by_number
        self._restyle_all_laps()

        cls_cfg = params["classification"]

        # background cloud (screen and export) = the whole lap of every CHECKED
        # lap: one lap-selection state, so an unchecked lap leaves no dot here
        # either (author defect 2026-10-07: its kerb samples stayed visible)
        session_mask = moving_mask if moving_mask is not None else np.ones_like(t, dtype=bool)
        if kerb_mask is not None:
            session_mask = session_mask & ~kerb_mask
        racing_mask = np.zeros_like(t, dtype=bool)
        for c in checked_instances:
            lap = laps_by_number[c["lap_number"]]
            racing_mask |= (t >= lap["start_time"]) & (t <= lap["end_time"])
        session_mask = session_mask & racing_mask

        # checked-set-dependent views via _refresh_checked_dependent_views
        self._render_ctx = {
            "corner_label": f"C{stable_corner_id}",
            "instances": instances, "laps_by_number": laps_by_number,
            "bracket_start_m": bracket_start_m, "bracket_end_m": bracket_end_m,
            "t": t, "s_m": s_m, "slip": slip, "forces": forces, "cs": cs,
            # the CS normalisation loads (fit L), for the tyre-curve unit
            "cs_axle_loads": stability_result.get("cs_axle_loads"),
            "kerb_mask": kerb_mask, "state": state, "params": params,
            "sideslip_source": stability_result.get("sideslip_source"),
            "fit_manifest": stability_result.get("fit_manifest"),
            "representative": representative, "all_export_laps": export_laps,
            "near_strike": {"mode": ns_mode},
            "session_mask": session_mask,
            "thresholds": {
                "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
                "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
                "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
            },
        }
        self._refresh_checked_dependent_views()
        # thresholds grey; strong dashed, moderate dotted, unstable dash-dot
        self._add_threshold_line("stab", cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"], THRESHOLD_GREY,
                                  Qt.PenStyle.DashDotLine, name="Unstable below this")
        self._add_threshold_line("cs_f", cls_cfg["STRONG_CSF"]["value"], THRESHOLD_GREY, Qt.PenStyle.DashLine,
                                  name="Strong")
        self._add_threshold_line("cs_f", cls_cfg["MODERATE_CSF"]["value"], THRESHOLD_GREY, Qt.PenStyle.DotLine,
                                  name="Moderate")
        self._add_threshold_line("cs_r", cls_cfg["STRONG_CSR"]["value"], THRESHOLD_GREY, Qt.PenStyle.DashLine,
                                  name="Strong")
        self._add_threshold_line("cs_r", cls_cfg["MODERATE_CSR"]["value"], THRESHOLD_GREY, Qt.PenStyle.DotLine,
                                  name="Moderate")

        if rep_start_s is not None:
            x_lo, x_hi = rep_start_s, rep_end_s
        else:
            x_lo, x_hi = representative["bracket_start_m"], representative["bracket_end_m"]
        # X range on every panel -- setXLink alone let panels auto-range and push
        # a wrong range back
        for plot in self.plots.values():
            plot.setXRange(x_lo, x_hi, padding=0.02)
        # fixed y for ratio panels; stab/speed auto-range
        for key in ("stab", "speed"):
            self.plots[key].enableAutoRange(axis='y')
        self._apply_ratio_y_range()

        n_laps = len(instances)
        n_quiet = sum(1 for c in instances if "canonical_quiet" in c.get("warnings", []))
        # readable text, not the "canonical_quiet" code
        quiet_note = f", {n_quiet} lap(s) not natively detected here (dashed)" if n_quiet else ""
        # colour is the only difference between laps
        self.header_label.setText(
            f"C{stable_corner_id} -- {n_laps} valid lap(s) available{quiet_note}, unchecked laps hidden -- "
            f"context margin -{margin_before_m:g}m/+{margin_after_m:g}m outside the bracket"
        )

        legend_parts = [BASE_LEGEND_TEXT]
        if any_kerb or any_not_moving:
            legend_parts.append(EXCLUSION_LEGEND_TEXT)
        if any_bridge:
            legend_parts.append(KERB_BRIDGE_LEGEND_TEXT)
        legend_parts.append(kerb_exclusion_caption(ns_mode, ns_counts[0], ns_counts[1]))
        if any_event:
            legend_parts.append(event_legend_text(stability_result.get("events")))
        self.legend_label.setText(" ".join(legend_parts))
        self.legend_label.setVisible(True)

        self.show()
        self.raise_()
        self.activateWindow()


class LapTraceDialog(_TraceDialogBase):
    """Non-modal full-lap trace window = the outing's problem map. Same
    scaffold, whole lap, one band per stable corner tinted by its worst
    verdict over all valid instances. Double-click a band -> corner window.
    """

    WINDOW_TITLE = "Lap Trace"
    WINDOW_SIZE = (1100, 680)

    def __init__(self, parent=None, on_corner_click=None):
        super().__init__(parent)
        self._on_corner_click = on_corner_click
        self._corner_bands = []  # [(start_s, end_s, id)] for hit-testing
        self._corner_summary_by_id = {}  # id -> displayed lap's summary
        self._laps_by_number = {}  # lap_number -> lap dict
        self.pg_layout.scene().sigMouseClicked.connect(self._on_scene_clicked)

        # export cache, filled at the end of show_lap
        self._export_data = None
        export_row = QHBoxLayout()
        self.export_figure_btn = QPushButton("Export figure")
        self.export_figure_btn.clicked.connect(self._on_export_figure_clicked)
        export_row.addWidget(self.export_figure_btn)
        export_row.addStretch(1)
        self.layout().addLayout(export_row)

    def _on_export_figure_clicked(self):
        if self._export_data is None:
            QMessageBox.information(self, "Export figure", "Show a lap first -- nothing to export yet.")
            return
        from core import figure_render, plot_style

        data = self._export_data
        default_name = f"{data['lap_label']}_figure.png"
        path, _ = QFileDialog.getSaveFileName(self, "Export figure", default_name, "PNG image (*.png)")
        if not path:
            return
        try:
            fig = figure_render.render_lap_figure(
                data["lap_label"], data["laps"], data["thresholds"], data["corner_bands"],
                theme=plot_style.PRINT, near_strike=data.get("near_strike"),
            )
            figure_render.save_png(fig, path)
        except Exception as e:
            from core.error_text import friendly_error_text
            QMessageBox.warning(self, "Export figure", f"Could not export figure ({friendly_error_text(e)}).")
            return
        QMessageBox.information(self, "Export figure", f"Saved to {path}")

    # restyle/toggle helpers inherited

    def _rerender_preserving_checked(self):
        # full re-render, checked set kept
        if self._last_show_args is not None:
            lap_number, stability_result, parsed_data, classify_fn = self._last_show_args
            self.show_lap(lap_number, stability_result, parsed_data, classify_fn,
                           preserve_visible=dict(self.lap_visible))

    def _load_lap_data(self, lap_number, stability_result, parsed_data):
        """state/cs/stab sliced to the whole lap via _lap_slice (no margin).
        Arrays ready to plot, or None.
        """
        state = stability_result.get("state")
        cs = stability_result.get("cs")
        stab = stability_result.get("stab")
        laps_by_number = {l["lap_number"]: l for l in parsed_data.get("laps", [])}
        lap = laps_by_number.get(lap_number)
        if state is None or cs is None or stab is None or lap is None:
            return None
        t = state["time"]
        s_m = state.get("s_m")
        if s_m is None:
            return None
        clipped = _lap_slice(t, s_m, lap["start_time"], lap["end_time"])
        if clipped is None:
            return None
        lo, hi, lap_s, _lo_s, _hi_s = clipped
        sl = slice(lo, hi)
        order = np.argsort(lap_s)  # monotonic within a lap; guard anyway
        kerb_mask = state.get("kerb_mask")
        moving_mask = state.get("moving_mask")
        return {
            "x": lap_s[order],
            "stab": stab["stability_observed_Nm_per_deg"][sl][order],
            "csf": cs["CS_ratio_f"][sl][order],
            "csr": cs["CS_ratio_r"][sl][order],
            "nsf": cs["near_strike_f"][sl][order] if cs.get("near_strike_f") is not None else None,
            "nsr": cs["near_strike_r"][sl][order] if cs.get("near_strike_r") is not None else None,
            "speed": state["v_mps"][sl][order] * 3.6,
            "kerb": kerb_mask[sl][order] if kerb_mask is not None else None,
            "not_moving": ~moving_mask[sl][order] if moving_mask is not None else None,
        }

    def _add_corner_bands(self, corners_by_id, worst_colour_by_id):
        import pyqtgraph as pg

        top_plot = self.plots["stab"]
        top_plot.getViewBox().autoRange()
        y_top = top_plot.getViewBox().viewRange()[1][1]
        tint_specs = []

        for cid, instances in sorted(corners_by_id.items()):
            rep = instances[0]
            start_s = rep.get("bracket_start_m")
            end_s = rep.get("bracket_end_m")
            colour = worst_colour_by_id.get(cid)
            if start_s is None or end_s is None or end_s <= start_s or colour is None:
                continue
            tint_specs.append((start_s, end_s, colour, 55))
            label = pg.TextItem(text=f"C{cid}", color=TEXT_MUTED, anchor=(0.5, 1.0))
            label.setPos((start_s + end_s) / 2.0, y_top)
            top_plot.addItem(label)
            self._corner_bands.append((start_s, end_s, cid))
        # the tint family of this window, drawn with the exclusion grey
        self._draw_tint_and_exclusion(tint_specs)

    def _on_scene_clicked(self, event):
        # double-click only -- sigMouseClicked also fires at the end of a pan,
        # which opened the corner window by accident
        if not event.double():
            return
        pos = event.scenePos()
        top_plot = self.plots["stab"]
        # scene -> view coordinates, as in outing_form's crosshair
        if not top_plot.sceneBoundingRect().contains(pos):
            return
        view_pos = top_plot.getViewBox().mapSceneToView(pos)
        x = view_pos.x()
        for start_s, end_s, cid in self._corner_bands:
            if start_s <= x <= end_s:
                summary = self._corner_summary_by_id.get(cid)
                if summary is not None and self._on_corner_click is not None:
                    self._on_corner_click(summary)
                return

    def show_lap(self, lap_number, stability_result, parsed_data, classify_fn, on_corner_click=None,
                 preserve_visible=None):
        """Whole-lap trace of lap_number. Checkbox per valid lap. Bands tinted by
        each corner's worst severity over all valid instances (classify_fn).
        """
        from modules.stability_analysis import load_parameters

        if on_corner_click is not None:
            self._on_corner_click = on_corner_click
        self._last_show_args = (lap_number, stability_result, parsed_data, classify_fn)

        for plot in self.plots.values():
            plot.clear()
        self.lap_curve_items = {}
        self.lap_bridge_items = {}
        self._corner_bands = []
        self._exclusion_spans = []
        self._corner_summary_by_id = {}

        state = stability_result.get("state")
        cs = stability_result.get("cs")
        stab = stability_result.get("stab")
        corners = stability_result.get("corners")
        summaries = stability_result.get("summaries")
        if state is None or cs is None or stab is None or corners is None or summaries is None:
            self.header_label.setText(
                f"Lap {lap_number}: raw sample arrays aren't available for this render "
                f"(cached summaries only) -- re-run Analyse to enable the trace view."
            )
            self._rebuild_lap_checkboxes([], None)
            self.show()
            self.raise_()
            return

        laps_by_number = {l["lap_number"]: l for l in parsed_data.get("laps", [])}
        valid_lap_numbers = {ln for ln, l in laps_by_number.items() if l.get("is_valid_for_analysis")}
        self._laps_by_number = laps_by_number

        if state.get("s_m") is None or lap_number not in valid_lap_numbers:
            self.header_label.setText(
                f"Lap {lap_number}: no lap_distance channel, or not a valid analysed lap -- "
                f"nothing to trace."
            )
            self._rebuild_lap_checkboxes([], None)
            self.show()
            self.raise_()
            return

        valid_summaries = [s for s in summaries if s["lap_number"] in valid_lap_numbers]
        corners_by_id = {}
        for c in corners:
            cid = c.get("stable_corner_id")
            if cid is not None and c["lap_number"] in valid_lap_numbers:
                corners_by_id.setdefault(cid, []).append(c)
        summaries_by_id = {}
        for s in valid_summaries:
            summaries_by_id.setdefault(s["stable_corner_id"], []).append(s)

        worst_colour_by_id = {
            cid: _aggregate_worst_severity(insts, classify_fn)
            for cid, insts in summaries_by_id.items()
        }
        for s in valid_summaries:
            if s["lap_number"] == lap_number:
                self._corner_summary_by_id[s["stable_corner_id"]] = s

        fastest_overall = _fastest_lap(sorted(valid_lap_numbers), laps_by_number)
        checkbox_instances = [{"lap_number": ln, "warnings": []} for ln in sorted(valid_lap_numbers)]
        # default checked = fastest N, plus the viewed lap if it isn't among them
        margin_cfg = load_parameters().get("corner_trace_display", {})
        n_default = margin_cfg.get("default_laps_shown", 5)
        default_checked_laps = _fastest_n_laps(
            sorted(valid_lap_numbers), laps_by_number, n_default) | {lap_number}
        self._rebuild_lap_checkboxes(
            checkbox_instances, lap_number,
            default_checked_laps=default_checked_laps, fastest_lap=fastest_overall,
            preserve_visible=preserve_visible,
        )

        # checked laps only
        checked_lap_numbers = sorted(ln for ln, v in self.lap_visible.items() if v)

        any_kerb = False
        any_event = False
        any_not_moving = False
        bridge_on = margin_cfg.get("kerb_gap_bridge", True)
        # kerb exclusion (display only): the same switch drives the export
        ns_mode = margin_cfg.get("kerb_exclusion_display", "grey")
        ns_counts = [0, 0]
        any_bridge = False
        export_laps = []
        for ln in checked_lap_numbers:
            data = self._load_lap_data(ln, stability_result, parsed_data)
            if data is None:
                continue

            if data["kerb"] is not None and data["kerb"].any():
                any_kerb = True
                self._collect_exclusion(data["x"], data["kerb"])
            any_event |= self._add_event_bands(stability_result.get("events"), ln)
            if data["not_moving"] is not None and data["not_moving"].any():
                any_not_moving = True
                self._collect_exclusion(data["x"], data["not_moving"])

            # placeholder pen, restyled below
            lap_name = f"Lap {ln}"
            placeholder_pen_kwargs = dict(color=THRESHOLD_GREY, width=SCREEN_LAP_WIDTH,
                                           style=Qt.PenStyle.SolidLine)
            curve_items = [
                self.plots["speed"].plot(data["x"], data["speed"], connect="finite",
                                          pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["stab"].plot(data["x"], data["stab"], connect="finite",
                                         pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self._plot_cs_graded("cs_f", data["x"], data["csf"], data["nsf"], ns_mode,
                                     self._pen(**placeholder_pen_kwargs), lap_name, ns_counts),
                self._plot_cs_graded("cs_r", data["x"], data["csr"], data["nsr"], ns_mode,
                                     self._pen(**placeholder_pen_kwargs), lap_name, ns_counts),
            ]
            self.lap_curve_items[ln] = curve_items
            if bridge_on and data["kerb"] is not None:
                any_bridge |= self._plot_kerb_bridges(
                    ln, data["x"], data["kerb"],
                    [("stab", data["stab"]), ("cs_f", data["csf"]), ("cs_r", data["csr"])])
            # figure_render key shape (s/v_kmh/stab/cs_f/cs_r), from the plotted slices
            export_laps.append({
                "lap_number": ln, "s": data["x"], "v_kmh": data["speed"],
                "stab": data["stab"], "cs_f": data["csf"], "cs_r": data["csr"],
                "kerb": data["kerb"] if bridge_on else None,
                "near_strike_f": data["nsf"], "near_strike_r": data["nsr"],
            })

        # colour assignment, same rule as the toggle re-render
        self._restyle_all_laps()

        cls_cfg = load_parameters()["classification"]
        # thresholds grey, by style
        self._add_threshold_line("stab", cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"], THRESHOLD_GREY,
                                  Qt.PenStyle.DashDotLine, name="Unstable below this")
        self._add_threshold_line("cs_f", cls_cfg["STRONG_CSF"]["value"], THRESHOLD_GREY, Qt.PenStyle.DashLine,
                                  name="Strong")
        self._add_threshold_line("cs_f", cls_cfg["MODERATE_CSF"]["value"], THRESHOLD_GREY, Qt.PenStyle.DotLine,
                                  name="Moderate")
        self._add_threshold_line("cs_r", cls_cfg["STRONG_CSR"]["value"], THRESHOLD_GREY, Qt.PenStyle.DashLine,
                                  name="Strong")
        self._add_threshold_line("cs_r", cls_cfg["MODERATE_CSR"]["value"], THRESHOLD_GREY, Qt.PenStyle.DotLine,
                                  name="Moderate")

        for plot in self.plots.values():
            plot.enableAutoRange(axis='x')
        # fixed y for ratio panels; stab/speed auto-range
        for key in ("stab", "speed"):
            self.plots[key].enableAutoRange(axis='y')
        self._apply_ratio_y_range()

        self._add_corner_bands(corners_by_id, worst_colour_by_id)

        # export cache
        self._export_data = {
            "lap_label": f"Lap {lap_number}", "laps": export_laps,
            "thresholds": {
                "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
                "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
                "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
            },
            "corner_bands": list(self._corner_bands),
            "near_strike": {"mode": ns_mode},
        }

        self.header_label.setText(
            f"Lap {lap_number} -- full-lap trace, {len(valid_lap_numbers)} valid lap(s) "
            f"available, fastest {n_default} (incl. lap {lap_number}) shown by default -- "
            f"check other laps to overlay."
        )

        legend_parts = [BASE_LEGEND_TEXT, LAP_BAND_LEGEND_TEXT]
        if any_kerb or any_not_moving:
            legend_parts.append(EXCLUSION_LEGEND_TEXT)
        if any_bridge:
            legend_parts.append(KERB_BRIDGE_LEGEND_TEXT)
        legend_parts.append(kerb_exclusion_caption(ns_mode, ns_counts[0], ns_counts[1]))
        if any_event:
            legend_parts.append(event_legend_text(stability_result.get("events")))
        self.legend_label.setText(" ".join(legend_parts))
        self.legend_label.setVisible(True)

        self.show()
        self.raise_()
        self.activateWindow()
