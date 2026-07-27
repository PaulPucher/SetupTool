# Per-corner and full-lap trace windows (PLAN.md, Tier C UI work package
# "PART C" + usability follow-up "Task 2", extended by the lap-trace-view
# work package). Both plot stability_observed / CS_ratio_f,r / speed
# against track position (s_m); the corner window covers one stable
# corner's phase bracket plus a config-resident approach/coast-out margin,
# the lap window covers a full lap's own 0..s_max range with a labeled,
# severity-tinted band per stable corner. Pure display: every array plotted
# here already exists in state/cs/stab (Modules 1-5 output cached on the
# form after a live Analyse) -- neither window performs estimation,
# threshold derivation, or masking of its own.

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox

from ui.style import BAD, BORDER, NEUTRAL, PANEL, TEXT_DIM, TEXT_MUTED

# Per-signal curve colours, fixed across laps (a lap is distinguished by
# line width/style/opacity, not colour) -- literal hex, same convention as
# the existing channel-strip plot's plot_channels list (outing_form.py
# _build_plot_widget), not ui.style: those constants are reserved for
# verdict/chrome colouring, not continuous data-curve identity.
CSF_COLOR = "#4FC3F7"
CSR_COLOR = "#FFB74D"
STAB_COLOR = "#B39DDB"
SPEED_COLOR = "#C0A060"  # matches ecu_speed's colour in the channel-strip plot

SELECTED_WIDTH = 2.5
NORMAL_WIDTH = 1.5
ALPHA_SELECTED = 255
ALPHA_FAINT = 90  # non-selected, still-visible laps -- out of 255

# Fix turn: plain-English legend, no internal field/code names (CSf/CSr,
# etc.) -- every curve, threshold line and shaded band gets a wording an
# engineer who has never opened this codebase can read unaided.
BASE_LEGEND_TEXT = (
    "Top panel: yaw-moment stability (purple); red dashed line = "
    "destabilising-yaw threshold, below it the car is flagged unstable. "
    "Middle panel: front cornering stiffness (blue) and rear cornering "
    "stiffness (orange); dashed lines = strong-collapse threshold, "
    "dotted lines = moderate-collapse threshold, same colour per axle. "
    "Bottom panel: speed (tan), shown for context only."
)

# Lap-trace-view work package: corner-band legend sentence, appended to
# BASE_LEGEND_TEXT. Colour words describe ui.style's actual BAD/WARN/OK
# hexes (a saturated red, a gold/tan, a green) -- not a separate palette.
LAP_BAND_LEGEND_TEXT = (
    "Background bands, labeled by corner number: colour is that corner's "
    "worst verdict across all its laps (red = strong, gold = moderate, "
    "green = normal) -- click a band to open its own per-corner trace."
)

PHASE_ORDER = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _phase_slice(t, start_t, end_t):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    return slice(lo, hi)


def _lap_slice(t, s_m, lap_start_t, lap_end_t):
    """Clip to this lap's own [lap_start_t, lap_end_t) time window and trim
    any trailing already-reset samples (see the reset-guard note below) --
    the reusable primitive both the corner window's margin-extension and
    the full-lap view's unmargined range are built on.

    lap_end_t (from parsed_data["laps"], the lap_number channel's own
    transition point) does not always land exactly on lap_distance's own
    reset instant -- the two are independent channels/sample timings.
    Found on real Dubai data: 3 of one corner's 4 lap windows carried a
    handful of already-reset (near-zero) trailing samples inside their
    nominal [lap_start_t, lap_end_t) window, which corrupted BOTH the s-
    bound clamp below (their tiny values are finite, not NaN, so an
    earlier finite-only guard didn't catch them) and searchsorted's own
    sortedness precondition (a late drop back to ~0 breaks monotonicity).
    Trim to the last index before any such drop -- a genuine same-lap
    sample never falls this far below its own running maximum.

    Returns (lo, hi, lap_s, lap_s_lo, lap_s_hi) -- lap_s is the (possibly
    trimmed) s_m sub-array for [lo:hi], lap_s_lo/hi its first/last FINITE
    value -- or None if this lap contributes no (or no finite) samples.
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
    # The reset-guarded s_m is also deliberately NaN at samples immediately
    # adjacent to a lap-boundary reset (modules/stability_analysis.py
    # _interp_lap_distance_guarded) -- clamp against the first/last FINITE
    # sample, not lap_s[0]/lap_s[-1] directly: Python's own min(nan, x)/
    # max(nan, x) silently returns nan when the NaN operand comes first.
    lap_s_lo = float(lap_s[finite_idx[0]])
    lap_s_hi = float(lap_s[finite_idx[-1]])
    return lo, hi, lap_s, lap_s_lo, lap_s_hi


def _extend_slice_with_margin(t, s_m, lap_start_t, lap_end_t,
                               bracket_start_m, bracket_end_m,
                               margin_before_m, margin_after_m):
    """Widen the corner's canonical bracket (bracket_start_m/bracket_end_m --
    the WP1 Turn 3 post-partition window, one fixed pair per stable_corner_id,
    persisted on the summary since ANALYSIS_SCHEMA_VERSION 4) by the
    configured margin on each side, clamped to this lap's own s_m extent
    (never reaches into a neighbouring lap -- s_m resets each lap, so
    clamping at this lap's own first/last sample is the correct bound, not
    an arbitrary safety pad).

    Fix turn: previously anchored on this lap's own entry_1_brake/exit_5
    PHASE times, interpolated to s per lap -- broke for any corner whose
    entry_1_brake phase was truncated to a degenerate (near-)zero-length
    boundary by the Turn 3 canonical-overlap partition (see corner_analysis.
    py's _resolve_canonical_overlaps): the margin then measured "100 m
    before" from a point already deep inside the corner, not the true
    approach, so the trace opened hundreds of metres late. bracket_start_m/
    end_m is the corner's own whole-window canonical bound, identical for
    every lap and already correctly resolved by that same partition step --
    using it directly needs no per-lap time interpolation at all for this
    part, and is the same value regardless of which lap's card was clicked.

    Returns (slice, start_s, end_s) or (slice(0, 0), None, None) if this
    lap contributes no samples at all.
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


def _worst_stab_phase(summary):
    # Duplicates OutingForm._classify_corner's stability-only argmin loop
    # (ui/views/outing_form.py, the "sob" branch inside its per-phase for
    # loop) rather than returning it from the classifier -- same precedent
    # as _stability_colour independently recomputing its own threshold
    # comparison there. This keeps the tinted band matching the phase
    # named in that same corner's verdict badge; if _classify_corner's
    # argmin logic ever changes (e.g. a tie-break rule), update this
    # function to match, or the tint and the badge will silently drift
    # apart.
    worst_phase, worst_val = None, float("inf")
    for phase, p in summary["phases"].items():
        val = p["stability_observed_Nm_per_deg"]["median"]
        if val == val and val < worst_val:  # NaN-safe (NaN != NaN)
            worst_val = val
            worst_phase = phase
    return worst_phase


def _phase_bands_for_lap(corner, t, s_m):
    # Phase boundaries for ONE representative lap (the lap the trace was
    # opened from), converted to s via that same lap's own t/s_m arrays.
    # apex_3 is a single instant (start_t == end_t, corner_analysis.py's
    # canonical realization) -- degenerate, skipped as a band; marked
    # instead by the apex position line in _add_phase_bands. Bands cover
    # ONLY the corner's own 5 phases (entry_1_brake..exit_5) -- the Task 2
    # context margin extends the PLOTTED DATA outside this range, never
    # the banding, so the margin reads as unshaded approach/coast-out
    # context, not part of "the bracket".
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
    for i, m in enumerate(mask):
        if m and not in_run:
            in_run, start_idx = True, i
        elif not m and in_run:
            in_run = False
            runs.append((x[start_idx], x[i - 1]))
    if in_run:
        runs.append((x[start_idx], x[-1]))
    return runs


def _aggregate_worst_severity(corner_summaries, classify_fn):
    # Lap-trace-view work package, amendment 1: the lap view is the
    # outing's problem map, not a per-lap snapshot -- a corner band's tint
    # is the WORST severity across ALL its valid-lap instances, not just
    # the instance on the lap currently displayed. classify_fn is the
    # caller-supplied classifier (in the UI thread, OutingForm.
    # _classify_corner) -- the same dependency-injection convention
    # modules/recommendation.py's generate_recommendations already uses,
    # so this tint can never disagree with the verdict shown elsewhere for
    # the same corner, and no severity logic is duplicated here.
    from modules.recommendation import SEVERITY_RANK

    worst_rank = -1
    worst_colour = None
    for s in corner_summaries:
        severity, _short, _long, colour = classify_fn(s)
        rank = SEVERITY_RANK[severity]
        if rank > worst_rank:
            worst_rank = rank
            worst_colour = colour
    return worst_colour


def _fastest_lap(lap_numbers, laps_by_number):
    # Lap-view emphasis fix: same "fastest" concept modules/csv_parser.py's
    # own outlier gate uses (lap_time_precise when the file's own channel
    # confirms it, else the computed lap_time duration -- see
    # csv_parser.py's _effective_lap_time/_attach_precise_lap_time) --
    # read inline here rather than importing that module-private helper,
    # matching outing_form.py's own existing inline read of the same two
    # fields (_build_lap_row's display_time). Falls back to the lowest lap
    # number only if NEITHER field is available for any candidate lap.
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


class _TraceDialogBase(QDialog):
    """Shared scaffold for the per-corner and full-lap trace windows: the
    3-panel pyqtgraph layout, per-lap visibility checkboxes, legend, and
    the pen/threshold-line/masked-span drawing helpers. Subclasses add
    only what differs -- show_corner (a windowed single-corner view) or
    show_lap (an unwindowed full-lap view with corner bands) -- neither
    subclass repeats this scaffold.
    """

    WINDOW_TITLE = "Trace"
    WINDOW_SIZE = (880, 680)

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg

        self.setWindowTitle(self.WINDOW_TITLE)
        self.resize(*self.WINDOW_SIZE)
        self.setModal(False)
        pg.setConfigOptions(antialias=True)

        self.lap_curve_items = {}  # lap_number -> [curve items], for visibility toggling
        self.lap_visible = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        self.header_label = QLabel("")
        self.header_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        layout.addWidget(self.header_label)

        # Task 2b: per-lap visibility checkboxes, built fresh per render
        # (the valid-lap set differs corner to corner / outing to outing)
        # -- outside the plot area, not a floating in-plot legend.
        self.lap_checkbox_container = self._build_lap_checkbox_container()
        layout.addWidget(self.lap_checkbox_container)

        self.pg_layout = pg.GraphicsLayoutWidget()
        self.pg_layout.setBackground(PANEL)
        layout.addWidget(self.pg_layout)

        # Rebuilt in full by show_corner()/show_lap() every call (only
        # mentions kerb/not-moving bands when the plotted range actually
        # has one) -- this is just the pre-first-click placeholder.
        self.legend_label = QLabel(BASE_LEGEND_TEXT)
        self.legend_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        layout.addWidget(self.legend_label)

        titles = [("Stability (Nm/deg)", "stab"), ("CS ratio", "cs"), ("Speed (km/h)", "speed")]
        self.plots = {}
        first_plot = None
        for i, (label, key) in enumerate(titles):
            is_last = (i == len(titles) - 1)
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
            self.plots[key] = plot
            self.pg_layout.nextRow()

        # Task 2b: panel 3 (speed, context only) gets less vertical room
        # than panels 1-2 (the two panels the verdict/threshold lines
        # actually live in) -- row stretch factors, not a fixed pixel cap,
        # so the ratio holds as the (resizable) window is resized.
        self.pg_layout.ci.layout.setRowStretchFactor(0, 3)
        self.pg_layout.ci.layout.setRowStretchFactor(1, 3)
        self.pg_layout.ci.layout.setRowStretchFactor(2, 1)

    def _build_lap_checkbox_container(self):
        from PyQt6.QtWidgets import QWidget
        container = QWidget()
        row_layout = QHBoxLayout(container)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(10)
        self.lap_checkbox_layout = row_layout
        return container

    def closeEvent(self, event):
        # Hide, don't destroy -- avoids rebuilding the pyqtgraph scene
        # graph on every reopen (same convention as the toggle-visible
        # sections elsewhere in outing_form.py). Both trace windows are
        # non-modal and share this override, so closing one (e.g. the
        # corner drill-down opened from a lap-band click) never touches
        # the other, still-open window.
        event.ignore()
        self.hide()

    def _rebuild_lap_checkboxes(self, instances, selected_lap, default_checked_laps=None, fastest_lap=None):
        # default_checked_laps: set of lap_numbers to start checked, or
        # None for "all checked" (CornerTraceDialog's existing behaviour,
        # unchanged -- every call site before the lap-trace-view work
        # package omitted this argument). LapTraceDialog passes
        # {selected_lap} so only the lap being viewed starts visible.
        # fastest_lap: lap_number to mark "(fastest)", or None for no
        # marking (CornerTraceDialog's existing behaviour, unchanged --
        # its own call site omits this argument too). Lap-view emphasis
        # fix: LapTraceDialog passes the session's fastest valid lap so
        # the label stays legible even when the emphasis rule (dynamic,
        # see LapTraceDialog._recompute_emphasis) bolds a different lap.
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
            is_checked = True if default_checked_laps is None else (lap_num in default_checked_laps)
            cb.setChecked(is_checked)
            cb.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
            cb.toggled.connect(lambda checked, ln=lap_num: self._on_lap_visibility_toggled(ln, checked))
            self.lap_checkbox_layout.addWidget(cb)
            self.lap_visible[lap_num] = is_checked
        self.lap_checkbox_layout.addStretch()

    def _on_lap_visibility_toggled(self, lap_number, checked):
        self.lap_visible[lap_number] = checked
        for item in self.lap_curve_items.get(lap_number, []):
            item.setVisible(checked)

    def _add_notmoving_bands(self, x, mask):
        # Q2 follow-up: moving_mask=False spans (car below moving_speed_
        # min_mps -- outlap/pit-lane/stationary portions the analysis
        # excludes) get the SAME grey-band treatment as kerb spans (the
        # existing pattern, reused rather than inventing a second visual
        # language) with a distinguishing dash-dot edge and a separate
        # legend line, so the two exclusion reasons read as different
        # things, not one unexplained grey blob.
        import pyqtgraph as pg

        color = pg.mkColor(NEUTRAL)
        color.setAlpha(60)
        for s_start, s_end in _contiguous_runs(x, mask):
            for plot in self.plots.values():
                region = pg.LinearRegionItem(values=(s_start, s_end), brush=pg.mkBrush(color), movable=False)
                region.setZValue(-6)
                for line in region.lines:
                    line.setPen(pg.mkPen(color=TEXT_DIM, width=1, style=Qt.PenStyle.DashDotLine))
                plot.addItem(region)

    def _add_kerb_bands(self, x, mask):
        import pyqtgraph as pg

        color = pg.mkColor(NEUTRAL)
        color.setAlpha(90)
        for s_start, s_end in _contiguous_runs(x, mask):
            for plot in self.plots.values():
                region = pg.LinearRegionItem(values=(s_start, s_end), brush=pg.mkBrush(color), movable=False)
                region.setZValue(-5)
                for line in region.lines:
                    line.setPen(pg.mkPen(color=TEXT_DIM, width=1, style=Qt.PenStyle.DotLine))
                plot.addItem(region)

    def _pen(self, color, width, style, alpha=255):
        import pyqtgraph as pg
        qcolor = pg.mkColor(color)
        qcolor.setAlpha(alpha)
        return pg.mkPen(color=qcolor, width=width, style=style)

    def _add_threshold_line(self, panel_key, value, color, style=Qt.PenStyle.DashLine):
        import pyqtgraph as pg
        self.plots[panel_key].addLine(y=value, pen=pg.mkPen(color=color, width=1, style=style))


class CornerTraceDialog(_TraceDialogBase):
    """Reusable, non-modal per-corner trace window. One instance lives on
    the form (created lazily); opening it for a different corner replots
    in place rather than creating a new window, same pattern as the
    corner map's clear()-and-redraw (outing_form.py
    _update_corner_map_trace).
    """

    WINDOW_TITLE = "Corner Trace"

    def _add_phase_bands(self, corner, t, s_m, worst_phase):
        import pyqtgraph as pg

        for phase, s_start, s_end in _phase_bands_for_lap(corner, t, s_m):
            is_worst = (phase == worst_phase)
            color = pg.mkColor(BAD if is_worst else BORDER)
            color.setAlpha(70 if is_worst else 30)
            for plot in self.plots.values():
                region = pg.LinearRegionItem(values=(s_start, s_end), brush=pg.mkBrush(color), movable=False)
                region.setZValue(-10)
                for line in region.lines:
                    line.setPen(pg.mkPen(None))
                plot.addItem(region)

        apex_s = corner.get("apex_lap_distance_m")
        if apex_s is not None:
            for plot in self.plots.values():
                apex_line = pg.InfiniteLine(
                    pos=apex_s, angle=90,
                    pen=pg.mkPen(color=TEXT_DIM, width=1, style=Qt.PenStyle.DotLine),
                )
                plot.addItem(apex_line)

    def show_corner(self, summary, stability_result, parsed_data):
        """Repopulate in place for `summary`'s stable_corner_id. `summary`
        is the single lap's corner-detail summary the trace button was
        clicked from -- its own lap_number is the "selected" (emphasised)
        lap and the representative lap for phase-band positions; its
        phase medians decide which phase's band is tinted (matching the
        verdict badge already shown for that same card).
        """
        from modules.stability_analysis import load_parameters

        for plot in self.plots.values():
            plot.clear()
        self.lap_curve_items = {}

        stable_corner_id = summary["stable_corner_id"]
        state = stability_result.get("state")
        cs = stability_result.get("cs")
        stab = stability_result.get("stab")
        corners = stability_result.get("corners")
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

        margin_cfg = load_parameters().get("corner_trace_display", {})
        margin_before_m = margin_cfg.get("margin_before_m", 100.0)
        margin_after_m = margin_cfg.get("margin_after_m", 50.0)

        v_kmh = state["v_mps"] * 3.6
        cs_f = cs["CS_ratio_f"]
        cs_r = cs["CS_ratio_r"]
        stab_obs = stab["stability_observed_Nm_per_deg"]
        kerb_mask = state.get("kerb_mask")
        moving_mask = state.get("moving_mask")

        selected_lap = summary["lap_number"]
        representative = next((c for c in instances if c["lap_number"] == selected_lap), instances[0])
        self._add_phase_bands(representative, t, s_m, worst_phase=_worst_stab_phase(summary))
        self._rebuild_lap_checkboxes(instances, selected_lap)

        # Fix turn: read the canonical bracket off the persisted summary
        # (ANALYSIS_SCHEMA_VERSION 4) first, falling back to the raw corner
        # dict only if an older in-memory summary somehow lacks it -- either
        # source gives the same value, since both are written by the same
        # WP1 Turn 3 canonical realization pass. One fixed pair for every
        # lap of this stable corner, not re-derived per lap.
        bracket_start_m = summary.get("bracket_start_m")
        bracket_end_m = summary.get("bracket_end_m")
        if bracket_start_m is None or bracket_end_m is None:
            bracket_start_m = representative.get("bracket_start_m")
            bracket_end_m = representative.get("bracket_end_m")

        any_kerb = False
        any_not_moving = False
        rep_start_s, rep_end_s = None, None
        for c in instances:
            is_selected = (c["lap_number"] == selected_lap)
            is_quiet = "canonical_quiet" in c.get("warnings", [])
            width = SELECTED_WIDTH if is_selected else NORMAL_WIDTH
            style = Qt.PenStyle.DashLine if is_quiet else Qt.PenStyle.SolidLine
            alpha = ALPHA_SELECTED if is_selected else ALPHA_FAINT

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
            order = np.argsort(x)  # s_m is monotonic within a lap by construction; guard anyway
            x = x[order]

            if kerb_mask is not None:
                lap_kerb = kerb_mask[sl][order]
                if lap_kerb.any():
                    any_kerb = True
                    self._add_kerb_bands(x, lap_kerb)

            if moving_mask is not None:
                lap_not_moving = ~moving_mask[sl][order]
                if lap_not_moving.any():
                    any_not_moving = True
                    self._add_notmoving_bands(x, lap_not_moving)

            curve_items = [
                self.plots["stab"].plot(
                    x, stab_obs[sl][order], connect="finite",
                    pen=self._pen(STAB_COLOR, width, style, alpha),
                ),
                self.plots["cs"].plot(
                    x, cs_f[sl][order], connect="finite",
                    pen=self._pen(CSF_COLOR, width, style, alpha),
                ),
                self.plots["cs"].plot(
                    x, cs_r[sl][order], connect="finite",
                    pen=self._pen(CSR_COLOR, width, style, alpha),
                ),
                self.plots["speed"].plot(
                    x, v_kmh[sl][order], connect="finite",
                    pen=self._pen(SPEED_COLOR, width, style, alpha),
                ),
            ]
            self.lap_curve_items[c["lap_number"]] = curve_items

        cls_cfg = load_parameters()["classification"]
        self._add_threshold_line("stab", cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"], BAD)
        self._add_threshold_line("cs", cls_cfg["STRONG_CSF"]["value"], CSF_COLOR, Qt.PenStyle.DashLine)
        self._add_threshold_line("cs", cls_cfg["MODERATE_CSF"]["value"], CSF_COLOR, Qt.PenStyle.DotLine)
        self._add_threshold_line("cs", cls_cfg["STRONG_CSR"]["value"], CSR_COLOR, Qt.PenStyle.DashLine)
        self._add_threshold_line("cs", cls_cfg["MODERATE_CSR"]["value"], CSR_COLOR, Qt.PenStyle.DotLine)

        if rep_start_s is not None:
            x_lo, x_hi = rep_start_s, rep_end_s
        else:
            x_lo, x_hi = representative["bracket_start_m"], representative["bracket_end_m"]
        # Set X range explicitly on EVERY panel, not just "stab" -- relying
        # on setXLink alone to propagate it left "cs"/"speed" free to
        # auto-range on their own newly-added curves and (being linked)
        # push a stale/wrong range back onto "stab"; pinning all three
        # directly removes that race regardless of link-callback order.
        for plot in self.plots.values():
            plot.enableAutoRange(axis='y')
            plot.setXRange(x_lo, x_hi, padding=0.02)

        n_laps = len(instances)
        n_quiet = sum(1 for c in instances if "canonical_quiet" in c.get("warnings", []))
        # Human-readable, not the internal "canonical_quiet" warning code --
        # this label is user-facing (Q2 follow-up).
        quiet_note = f", {n_quiet} lap(s) not natively detected here (dashed)" if n_quiet else ""
        self.header_label.setText(
            f"C{stable_corner_id} -- {n_laps} valid lap(s) shown{quiet_note}, "
            f"lap {selected_lap} emphasised (bold; others faint) -- "
            f"context margin -{margin_before_m:g}m/+{margin_after_m:g}m outside the bracket"
        )

        legend_parts = [BASE_LEGEND_TEXT]
        if any_kerb:
            legend_parts.append(
                "Grey band, dotted edge: kerb strike -- the analysis fills this span in from clean data nearby."
            )
        if any_not_moving:
            legend_parts.append(
                "Grey band, dash-dot edge: car not up to speed here (out lap or pit lane)."
            )
        self.legend_label.setText(" ".join(legend_parts))
        self.legend_label.setVisible(True)

        self.show()
        self.raise_()
        self.activateWindow()


class LapTraceDialog(_TraceDialogBase):
    """Reusable, non-modal full-lap trace window: the outing's problem
    map. Same 3-panel scaffold as CornerTraceDialog (shared base class),
    unwindowed over the shown lap's own 0..s_max range, with one labeled
    background band per stable corner tinted by that corner's WORST
    verdict across all its valid-lap instances (not just this lap's own
    instance -- see _aggregate_worst_severity). Clicking a band opens the
    existing CornerTraceDialog for that corner via on_corner_click.
    """

    WINDOW_TITLE = "Lap Trace"
    WINDOW_SIZE = (1100, 680)

    # Order matches the curve_items list built in show_lap (stab/csf/csr/
    # speed) -- used by _restyle_lap_curves to rebuild each item's pen
    # without needing to remember its colour separately.
    _CURVE_COLOR_ORDER = (STAB_COLOR, CSF_COLOR, CSR_COLOR, SPEED_COLOR)

    def __init__(self, parent=None, on_corner_click=None):
        super().__init__(parent)
        self._on_corner_click = on_corner_click
        self._corner_bands = []          # [(start_s, end_s, stable_corner_id), ...] for click hit-testing
        self._corner_summary_by_id = {}  # stable_corner_id -> this render's displayed-lap summary
        self._laps_by_number = {}        # lap_number -> lap dict, for _fastest_lap's lap_time lookup
        self.pg_layout.scene().sigMouseClicked.connect(self._on_scene_clicked)

    def _on_lap_visibility_toggled(self, lap_number, checked):
        # Lap-view emphasis fix: visibility toggling itself is unchanged
        # (super()); only which lap renders bold is re-decided every time
        # the visible set changes -- see _recompute_emphasis.
        super()._on_lap_visibility_toggled(lap_number, checked)
        self._recompute_emphasis()

    def _restyle_lap_curves(self, lap_number, is_emphasized):
        items = self.lap_curve_items.get(lap_number)
        if not items:
            return
        width = SELECTED_WIDTH if is_emphasized else NORMAL_WIDTH
        alpha = ALPHA_SELECTED if is_emphasized else ALPHA_FAINT
        for item, color in zip(items, self._CURVE_COLOR_ORDER):
            item.setPen(self._pen(color, width, Qt.PenStyle.SolidLine, alpha))

    def _recompute_emphasis(self):
        # Lap-view emphasis fix (replaces the old clicked-lap special-
        # casing entirely, not a special case on top of it): exactly one
        # visible lap -> that lap is emphasised regardless of identity;
        # more than one -> the fastest AMONG THE CURRENTLY VISIBLE laps is
        # emphasised (_fastest_lap, same lap_time_precise/lap_time reading
        # as the checkbox's static "(fastest)" label, but recomputed here
        # over only the visible subset, which can differ from the
        # session-wide fastest lap the label marks). Zero laps visible:
        # nothing to restyle.
        visible = [ln for ln, v in self.lap_visible.items() if v]
        if not visible:
            return
        emphasized = visible[0] if len(visible) == 1 else _fastest_lap(visible, self._laps_by_number)
        for ln in self.lap_curve_items:
            self._restyle_lap_curves(ln, ln == emphasized)

    def _load_lap_data(self, lap_number, stability_result, parsed_data):
        """Slice state/cs/stab to lap_number's full valid-lap range via
        _lap_slice (the same clamp+reset-guard-trim primitive the corner
        window's margin extension uses, just without a margin). Returns a
        dict of ready-to-plot arrays, or None if this lap contributes no
        usable samples.
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
        order = np.argsort(lap_s)  # s_m is monotonic within a lap by construction; guard anyway
        kerb_mask = state.get("kerb_mask")
        moving_mask = state.get("moving_mask")
        return {
            "x": lap_s[order],
            "stab": stab["stability_observed_Nm_per_deg"][sl][order],
            "csf": cs["CS_ratio_f"][sl][order],
            "csr": cs["CS_ratio_r"][sl][order],
            "speed": state["v_mps"][sl][order] * 3.6,
            "kerb": kerb_mask[sl][order] if kerb_mask is not None else None,
            "not_moving": ~moving_mask[sl][order] if moving_mask is not None else None,
        }

    def _add_corner_bands(self, corners_by_id, worst_colour_by_id):
        import pyqtgraph as pg

        top_plot = self.plots["stab"]
        top_plot.getViewBox().autoRange()
        y_top = top_plot.getViewBox().viewRange()[1][1]

        for cid, instances in sorted(corners_by_id.items()):
            rep = instances[0]
            start_s = rep.get("bracket_start_m")
            end_s = rep.get("bracket_end_m")
            colour = worst_colour_by_id.get(cid)
            if start_s is None or end_s is None or end_s <= start_s or colour is None:
                continue
            color = pg.mkColor(colour)
            color.setAlpha(55)
            for plot in self.plots.values():
                region = pg.LinearRegionItem(values=(start_s, end_s), brush=pg.mkBrush(color), movable=False)
                region.setZValue(-20)
                for line in region.lines:
                    line.setPen(pg.mkPen(None))
                plot.addItem(region)
            label = pg.TextItem(text=f"C{cid}", color=TEXT_MUTED, anchor=(0.5, 1.0))
            label.setPos((start_s + end_s) / 2.0, y_top)
            top_plot.addItem(label)
            self._corner_bands.append((start_s, end_s, cid))

    def _on_scene_clicked(self, event):
        pos = event.scenePos()
        top_plot = self.plots["stab"]
        # Same scene-position-to-view-coordinate pattern outing_form.py's
        # channel-strip plot already uses for its crosshair
        # (_on_mouse_moved) -- reused, not reinvented, just for a click
        # instead of a hover.
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

    def show_lap(self, lap_number, stability_result, parsed_data, classify_fn, on_corner_click=None):
        """Repopulate in place for the full 0..s_max trace of `lap_number`.
        All valid laps get a visibility checkbox (reused mechanism), but
        only `lap_number` starts checked -- default display is the
        selected lap only, others available to overlay. Corner bands are
        tinted by each stable corner's AGGREGATED verdict (worst severity
        across ALL its valid-lap instances via `classify_fn`, the same
        caller-supplied classifier generate_recommendations/the stability
        grid use) -- this window is the outing's problem map, not a
        per-lap snapshot.
        """
        from modules.stability_analysis import load_parameters

        if on_corner_click is not None:
            self._on_corner_click = on_corner_click

        for plot in self.plots.values():
            plot.clear()
        self.lap_curve_items = {}
        self._corner_bands = []
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
        self._rebuild_lap_checkboxes(
            checkbox_instances, lap_number,
            default_checked_laps={lap_number}, fastest_lap=fastest_overall,
        )

        any_kerb = False
        any_not_moving = False
        for ln in sorted(valid_lap_numbers):
            data = self._load_lap_data(ln, stability_result, parsed_data)
            if data is None:
                continue
            # Style is a placeholder here -- _recompute_emphasis (called
            # once every lap's curves exist, below) sets the real width/
            # alpha for every lap in one pass. No is_selected special case
            # for lap_number: emphasis is decided purely by the dynamic
            # rule (lap-view emphasis fix).
            visible = self.lap_visible[ln]

            if data["kerb"] is not None and data["kerb"].any():
                any_kerb = True
                self._add_kerb_bands(data["x"], data["kerb"])
            if data["not_moving"] is not None and data["not_moving"].any():
                any_not_moving = True
                self._add_notmoving_bands(data["x"], data["not_moving"])

            curve_items = [
                self.plots["stab"].plot(
                    data["x"], data["stab"], connect="finite",
                    pen=self._pen(STAB_COLOR, NORMAL_WIDTH, Qt.PenStyle.SolidLine, ALPHA_FAINT),
                ),
                self.plots["cs"].plot(
                    data["x"], data["csf"], connect="finite",
                    pen=self._pen(CSF_COLOR, NORMAL_WIDTH, Qt.PenStyle.SolidLine, ALPHA_FAINT),
                ),
                self.plots["cs"].plot(
                    data["x"], data["csr"], connect="finite",
                    pen=self._pen(CSR_COLOR, NORMAL_WIDTH, Qt.PenStyle.SolidLine, ALPHA_FAINT),
                ),
                self.plots["speed"].plot(
                    data["x"], data["speed"], connect="finite",
                    pen=self._pen(SPEED_COLOR, NORMAL_WIDTH, Qt.PenStyle.SolidLine, ALPHA_FAINT),
                ),
            ]
            for item in curve_items:
                item.setVisible(visible)
            self.lap_curve_items[ln] = curve_items

        # Lap-view emphasis fix: one pass over every lap just built,
        # applying the dynamic one-visible/fastest-of-visible rule -- the
        # same recompute the checkbox-toggle handler calls, so the initial
        # render and every subsequent toggle go through one rule, not two.
        self._recompute_emphasis()

        cls_cfg = load_parameters()["classification"]
        self._add_threshold_line("stab", cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"], BAD)
        self._add_threshold_line("cs", cls_cfg["STRONG_CSF"]["value"], CSF_COLOR, Qt.PenStyle.DashLine)
        self._add_threshold_line("cs", cls_cfg["MODERATE_CSF"]["value"], CSF_COLOR, Qt.PenStyle.DotLine)
        self._add_threshold_line("cs", cls_cfg["STRONG_CSR"]["value"], CSR_COLOR, Qt.PenStyle.DashLine)
        self._add_threshold_line("cs", cls_cfg["MODERATE_CSR"]["value"], CSR_COLOR, Qt.PenStyle.DotLine)

        for plot in self.plots.values():
            plot.enableAutoRange(axis='y')
            plot.enableAutoRange(axis='x')

        self._add_corner_bands(corners_by_id, worst_colour_by_id)

        self.header_label.setText(
            f"Lap {lap_number} -- full-lap trace, {len(valid_lap_numbers)} valid lap(s) "
            f"available, lap {lap_number} shown by default -- check other laps to overlay; "
            f"the fastest checked lap is always emphasised (bold)."
        )

        legend_parts = [BASE_LEGEND_TEXT, LAP_BAND_LEGEND_TEXT]
        if any_kerb:
            legend_parts.append(
                "Grey band, dotted edge: kerb strike -- the analysis fills this span in from clean data nearby."
            )
        if any_not_moving:
            legend_parts.append(
                "Grey band, dash-dot edge: car not up to speed here (out lap or pit lane)."
            )
        self.legend_label.setText(" ".join(legend_parts))
        self.legend_label.setVisible(True)

        self.show()
        self.raise_()
        self.activateWindow()
