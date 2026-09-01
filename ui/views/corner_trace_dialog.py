# Per-corner and full-lap trace windows (PLAN.md, Tier C UI work package
# "PART C" + usability follow-up "Task 2", extended by the lap-trace-view
# work package). Both plot stability_observed / CS_ratio_f,r / LS_ratio_f,r
# (PLAN.md STEP 3 Phase 3) / speed against track position (s_m); the
# corner window covers one stable
# corner's phase bracket plus a config-resident approach/coast-out margin,
# the lap window covers a full lap's own 0..s_max range with a labeled,
# severity-tinted band per stable corner. Pure display: every array plotted
# here already exists in state/cs/stab (Modules 1-5 output cached on the
# form after a live Analyse) -- neither window performs estimation,
# threshold derivation, or masking of its own.

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QTabWidget, QWidget,
    QPushButton, QFileDialog, QMessageBox,
)

from ui.style import BAD, BORDER, NEUTRAL, PANEL, PANEL_ALT, TEXT, TEXT_DIM, TEXT_MUTED

# Cleanup pass, Part B: per-quantity colour/width/font constants now live
# in core/plot_style.py, shared with the matplotlib export path (core/
# figure_render.py) and diagnostics/inspect_step2_chair_plots.py's batch
# export -- one source of truth so a curve means the same colour whether
# it's on screen here or in an exported thesis figure.
#
# Part B redesign (2026-09-01, trace-dialog work package + addendum): lap
# identity is now carried by COLOUR (plot_style.lap_styles, assigned to
# the checked set in ascending order), not a fixed per-axle hue -- CSF_
# COLOR/CSR_COLOR/STAB_COLOR/etc. are retired. Axle/quantity identity
# moves to which PANEL a curve is on instead (see PANEL_TITLES).
#
# Corrections batch (same day, after visual review): the bold/faint
# ("emphasised lap") distinction is removed entirely -- SCREEN_LAP_WIDTH
# is the only width, every checked lap draws identically. Tyre-curve
# marker/line constants re-tuned for readability (session cloud very
# light/thin, lap samples medium filled at partial alpha, estimation-
# window rings largest and on top).
from core import plot_style
from core.plot_style import (
    FITTED_CURVE_COLOR, TRACK_BG_COLOR, WINDOW_RING_COLOR, THRESHOLD_GREY,
    SCREEN_LAP_WIDTH, LEGEND_FONT_PT,
    SESSION_CLOUD_COLOR_SCREEN, SESSION_CLOUD_SIZE_SCREEN,
    LAP_SAMPLE_SIZE_SCREEN, LAP_SAMPLE_ALPHA, WINDOW_RING_SIZE_SCREEN,
)

# UI cleanup package: ONE consistent in-plot legend style for every plot
# this dialog family builds (item 3 -- "apply one consistent style,
# don't restyle per-plot ad hoc"). Font bumped visibly above pyqtgraph's
# own default ('9pt', itself already larger than this file's 8pt axis-
# label convention) so a first-time user does not have to lean in to
# read it. Legends are built ONCE per plot at construction time (see
# _TraceDialogBase.__init__ and CornerTraceDialog.__init__) and updated
# automatically thereafter by pyqtgraph itself whenever a named curve is
# added/removed via plot.clear()/plot.plot(..., name=...) -- calling
# addLegend() again on every show_corner()/show_lap() render would stack
# duplicate legend boxes, so it must stay a one-time construction call.
LEGEND_INSET = (-8, 8)  # px, pulled in from the plot's actual top-right corner


def _style_new_legend(plot):
    """Attach ONE styled legend to `plot`, call exactly once per plot
    object (see LEGEND_FONT_PT's own comment for why). Solid background
    so it stays fully readable over a busy trace.

    Cleanup fix: addLegend(offset=...) alone anchors the legend's
    top-LEFT corner near the viewbox's top-left, offset by that amount --
    on every panel here, traces start at the left edge of the window
    (track position s), so that placement sat directly on top of the
    data. legend.anchor(itemPos=(1,0), parentPos=(1,0), ...) explicitly
    pins the legend's own top-right corner to the viewbox's top-right
    corner instead, which is empty on every panel in this dialog family
    (curves only reach their highest values well after the left edge).
    """
    import pyqtgraph as pg
    legend = plot.addLegend(labelTextSize=LEGEND_FONT_PT)
    legend.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=LEGEND_INSET)
    legend.setBrush(pg.mkBrush(PANEL_ALT))
    legend.setLabelTextColor(TEXT_MUTED)
    return legend


# Cleanup pass, Phase 1: short footer, not a per-panel/per-threshold
# explainer -- each panel's own in-plot legend (see _style_new_legend/
# _add_threshold_line) now names every curve and threshold line directly.
BASE_LEGEND_TEXT = (
    "Colour = lap (see each panel's own legend). "
    "Unchecked laps are not drawn anywhere in this window."
)

# A4: split out of BASE_LEGEND_TEXT (2026-09-01 cleanup/reliability pass)
# -- LapTraceDialog no longer has an LS panel at all (LS_ratio has no
# lap-level reading, see that class's own docstring), so this sentence is
# now CornerTraceDialog-only, appended when the ls panel actually has data
# this render (same optional-degradation guard show_corner already uses).
LS_PANEL_LEGEND_TEXT = "LS ratio is shown for context only -- no verdict depends on it."

# Lap-trace-view work package: corner-band legend sentence, appended to
# BASE_LEGEND_TEXT. Colour words describe ui.style's actual BAD/WARN/OK
# hexes (a saturated red, a gold/tan, a green) -- not a separate palette.
LAP_BAND_LEGEND_TEXT = (
    "Corner bands: colour = worst verdict across all laps (red/gold/green). "
    "Click a band to open that corner's trace."
)

PHASE_ORDER = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]

# WP-A item 3 (CS credibility bundle): tyre-curve scatter tab legend. Plot
# design after chair performance_analysis tooling (internal, its
# create_evaluation_plot scatter panel and create_scatter grouping-axis
# pattern); no code copied.
TYRE_CURVE_LEGEND_TEXT = (
    "Slip angle vs lateral force, this corner's window. Filled = lap "
    "sample, hollow = kerb-flagged, black ring = estimation window."
)


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


def _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m, bracket_start_m, bracket_end_m):
    """The single sample with the lowest CS_ratio (most saturated) inside
    this corner's own canonical bracket, pooled across every valid lap
    instance -- same "worst phase" concept diagnostics/inspect_step2_
    chair_plots.py's own _find_worst_phase already established, reused
    here (not reimplemented) for the corner-trace track map's and export
    figure's estimation-window highlight. Returns {"index": global array
    index} or None if no instance contributes a finite sample.
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
    for phase, phase_data in summary["phases"].items():
        val = phase_data["stability_observed_Nm_per_deg"]["median"]
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
    for corner_summary in corner_summaries:
        severity, _short, _long, colour = classify_fn(corner_summary)
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


def _fastest_n_laps(lap_numbers, laps_by_number, n):
    # Addendum item 2: default checked set on a corner opened from an "All
    # laps" analysis is the fastest N valid laps of that corner, N =
    # corner_trace_display.default_laps_shown. Same lap_time_precise/
    # lap_time reading convention as _fastest_lap, generalised to top-N --
    # a lap with neither field sorts LAST (float("inf")) rather than being
    # silently dropped, so at least min(n, len(lap_numbers)) laps are
    # always selected even if no lap has a recorded time.
    def _time(ln):
        lap = laps_by_number.get(ln, {})
        lt = lap.get("lap_time_precise")
        if lt is None:
            lt = lap.get("lap_time")
        return lt if lt is not None else float("inf")

    return set(sorted(lap_numbers, key=_time)[:n])


class _TraceDialogBase(QDialog):
    """Shared scaffold for the per-corner and full-lap trace windows: the
    pyqtgraph panel layout, per-lap visibility checkboxes, legend, and the
    pen/threshold-line/masked-span drawing helpers. Subclasses add only
    what differs -- show_corner (a windowed single-corner view) or
    show_lap (an unwindowed full-lap view with corner bands) -- neither
    subclass repeats this scaffold.

    Part B redesign (2026-09-01, trace-dialog work package + addendum):
    lap identity is now carried by COLOUR (core.plot_style.lap_styles,
    reassigned to the checked set in ascending order on every render), not
    a fixed per-quantity hue -- so a checkbox toggle now triggers a FULL
    re-render (_rerender_preserving_checked, implemented per subclass)
    instead of incremental item show/hide: colour assignment and each
    panel's per-lap legend both depend on the WHOLE checked set, not just
    the toggled lap, so patching individual items in place would have to
    reproduce that same whole-set logic anyway. A consequence, and the
    real fix underneath Part A's A1/A2 items: only CHECKED laps are ever
    plotted at all (no hidden-but-present item exists for an unchecked
    lap), which is also why no dedicated "legend swatch" trick is needed
    any more -- every item carrying a legend name is, by construction,
    always visible.
    """

    WINDOW_TITLE = "Trace"
    WINDOW_SIZE = (880, 680)

    # Panel scaffold, overridable per subclass -- LapTraceDialog drops the
    # ls_f/ls_r pair entirely (LS_ratio has no lap-level reading, see its
    # own class docstring). Part B layout order (work order's own list):
    # Speed first (context/overview), then Stability, then CS/LS split
    # into one axle per panel -- axle identity now lives in the PANEL,
    # freeing colour entirely for lap identity.
    PANEL_TITLES = [
        ("Speed (km/h)", "speed"), ("Stability (Nm/deg)", "stab"),
        ("Front CS ratio", "cs_f"), ("Rear CS ratio", "cs_r"),
        ("Front LS ratio", "ls_f"), ("Rear LS ratio", "ls_r"),
    ]
    # Panels sharing the CS_ratio/LS_ratio 1/0/negative scale and its
    # fixed y-range clip (_apply_ratio_y_range) -- overridable per
    # subclass (LapTraceDialog drops the ls_f/ls_r pair).
    RATIO_PANEL_KEYS = ("cs_f", "cs_r", "ls_f", "ls_r")
    # Panel(s) that get the lighter "context" vertical stretch, by KEY not
    # row position -- Part B moved "speed" to the FIRST row, so a
    # position-based rule ("last row is lighter") no longer identifies it.
    CONTEXT_PANEL_KEYS = ("speed",)

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg

        self.setWindowTitle(self.WINDOW_TITLE)
        self.resize(*self.WINDOW_SIZE)
        self.setModal(False)
        # Follow-up item 1: a QDialog's default window flags omit the
        # native minimise/maximise buttons on Windows -- these trace
        # windows are data-dense and benefit from being maximised with
        # one click rather than dragged to size.
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowMinMaxButtonsHint)
        pg.setConfigOptions(antialias=True)

        self.lap_curve_items = {}   # lap_number -> [curve items] -- CHECKED laps only, see class docstring
        # Quiet-lap dash style is a per-lap PRESENTATION choice
        # (CornerTraceDialog only, see the "canonical_quiet" warning below)
        # -- independent of colour/emphasis, _restyle_lap_curves ORs it
        # with the palette's own beyond-10-laps dash cycling.
        self._lap_line_style = {}
        # lap_number -> {"color": hex, "dash": "solid"|"dash"} for the
        # CURRENT checked set -- recomputed by _restyle_all_laps every
        # render (colour is a function of the checked set, not the lap
        # number, see plot_style.lap_styles).
        self._current_styles = {}
        self.lap_visible = {}
        # Replayed by _rerender_preserving_checked on every checkbox
        # toggle -- set at the top of show_corner/show_lap.
        self._last_show_args = None

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
        # UI cleanup package: 10px -> 12px. This prose caption explains WHY
        # (threshold meaning, beyond-peak fold-back, etc) -- content an
        # in-plot legend can't carry -- so it stays, just larger; WHICH
        # LINE IS WHICH is now the in-plot legends' job (item 3).
        self.legend_label = QLabel(BASE_LEGEND_TEXT)
        self.legend_label.setWordWrap(True)
        self.legend_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        layout.addWidget(self.legend_label)

        # PLAN.md STEP 3 Phase 3: "ls" panel added alongside "cs", same
        # scaffold, same per-plot legend treatment -- DISPLAY ONLY (no
        # threshold lines are drawn on it, see _add_threshold_line call
        # sites in show_corner/show_lap: none target "ls").
        self.plots = {}
        self.legends = {}
        first_plot = None
        for i, (label, key) in enumerate(self.PANEL_TITLES):
            is_last = (i == len(self.PANEL_TITLES) - 1)
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
            # UI cleanup package: one legend per plot, styled consistently
            # -- see _style_new_legend/LEGEND_FONT_PT. "stab"/"speed" plot
            # only one named series each today (still worth a legend: it
            # states the line's name instead of requiring the prose
            # caption below to be read first); "cs" plots two.
            self.legends[key] = _style_new_legend(plot)
            self.plots[key] = plot
            self.pg_layout.nextRow()

        # Task 2b: the speed panel (context only) gets less vertical room
        # than the other panels -- row stretch factors, not a fixed pixel
        # cap, so the ratio holds as the (resizable) window is resized.
        # LS panels get the same stretch as stability/CS -- primary data
        # panels, not context-only ones, even though DISPLAY ONLY (no
        # threshold lines). By KEY (CONTEXT_PANEL_KEYS), not row position
        # -- Part B put "speed" first, not last.
        for i, (_label, key) in enumerate(self.PANEL_TITLES):
            self.pg_layout.ci.layout.setRowStretchFactor(i, 1 if key in self.CONTEXT_PANEL_KEYS else 3)

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

    def _rebuild_lap_checkboxes(self, instances, selected_lap, default_checked_laps=None, fastest_lap=None,
                                 preserve_visible=None):
        # default_checked_laps: set of lap_numbers to start checked, or
        # None for "all checked" (CornerTraceDialog's existing behaviour,
        # unchanged -- every call site before the lap-trace-view work
        # package omitted this argument). LapTraceDialog passes
        # {selected_lap} so only the lap being viewed starts visible.
        # fastest_lap: lap_number to mark "(fastest)", or None for no
        # marking (CornerTraceDialog's existing behaviour, unchanged --
        # its own call site omits this argument too). LapTraceDialog
        # passes the session's fastest valid lap so the checkbox label
        # states it directly (no visual bold/faint distinction exists any
        # more, see plot_style.SCREEN_LAP_WIDTH's own comment).
        #
        # preserve_visible (Part B): an exact {lap_number: bool} map to use
        # VERBATIM instead of computing a fresh default -- passed by
        # _rerender_preserving_checked on every checkbox-toggle-triggered
        # full re-render, so the user's own checked/unchecked choices
        # survive the rebuild that toggle itself causes.
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
        # Part B: colour (ascending palette position within the checked
        # set) and each panel's per-lap legend both depend on the WHOLE
        # checked set, not just the toggled lap -- re-running this
        # dialog's own render entry point from scratch, with the checked
        # set preserved, is simpler and less bug-prone than patching every
        # affected item's colour/legend row in place one by one. See
        # _rerender_preserving_checked (implemented per subclass).
        self.lap_visible[lap_number] = checked
        self._rerender_preserving_checked()

    def _rerender_preserving_checked(self):
        raise NotImplementedError

    def _restyle_lap_curves(self, lap_number):
        # Corrections batch: no bold/faint distinction any more -- every
        # checked lap draws at the same SCREEN_LAP_WIDTH and full opacity
        # (_pen's own alpha=255 default), colour is the only thing that
        # ever changes here.
        items = self.lap_curve_items.get(lap_number)
        style = self._current_styles.get(lap_number)
        if not items or style is None:
            return
        # A "quiet" lap's dash overlay (CornerTraceDialog only, see
        # _lap_line_style's own comment) and the palette's own beyond-10-
        # laps dash cycling (plot_style.lap_styles) are ORed together --
        # either condition renders dashed, never a distinct third style;
        # both applying at once (rare -- needs a quiet lap AND >10 laps
        # checked) still just reads as dashed.
        is_dashed = (self._lap_line_style.get(lap_number) == Qt.PenStyle.DashLine) or (style["dash"] == "dash")
        qt_style = Qt.PenStyle.DashLine if is_dashed else Qt.PenStyle.SolidLine
        for item in items:
            item.setPen(self._pen(style["color"], SCREEN_LAP_WIDTH, qt_style))

    def _restyle_all_laps(self):
        # Part B: recomputes colour assignment from the current checked
        # set (ascending order -- plot_style.lap_styles). Called once at
        # the end of every full render (initial open or checkbox-
        # triggered re-render) -- the single place it can change.
        checked = sorted(ln for ln, v in self.lap_visible.items() if v)
        self._current_styles = plot_style.lap_styles(checked)
        for ln in self.lap_curve_items:
            self._restyle_lap_curves(ln)

    def _add_notmoving_bands(self, x, mask):
        # Q2 follow-up: moving_mask=False spans (car below moving_speed_
        # min_mps -- outlap/pit-lane/stationary portions the analysis
        # excludes) get the SAME grey-band treatment as kerb spans (the
        # existing pattern, reused rather than inventing a second visual
        # language) with a distinguishing dash-dot edge and a separate
        # legend line, so the two exclusion reasons read as different
        # things, not one unexplained grey blob. Only ever called for a
        # CHECKED lap (Part B: checked-only plotting, see class docstring)
        # -- no separate visibility tracking needed, a checkbox toggle
        # clears and rebuilds the whole plot.
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

    def _add_threshold_line(self, panel_key, value, color, style=Qt.PenStyle.DashLine, name=None):
        # Footer-text cleanup: threshold-line MEANING now lives here, as a
        # named legend entry, instead of in the long prose caption below
        # the plots (self.legend_label) -- name=None keeps the pre-cleanup
        # behaviour (no legend entry) for any future caller that doesn't
        # pass one.
        #
        # Bug fix: pyqtgraph's LegendItem.addItem(item, name) always builds
        # an ItemSample that reads item.opts['pen'] to draw the swatch --
        # InfiniteLine (what addLine returns) has no .opts dict at all, so
        # registering the line object itself raises AttributeError inside
        # ItemSample.paint() on every repaint (silently, Qt just skips the
        # failed paint -- the legend ROW still appeared, with a name, just
        # with no swatch ever drawn next to it). A zero-point PlotDataItem
        # with the same pen is invisible on the plot itself but IS a real
        # ItemSample-compatible item, giving the swatch the InfiniteLine
        # never could.
        import pyqtgraph as pg
        pen = pg.mkPen(color=color, width=1, style=style)
        self.plots[panel_key].addLine(y=value, pen=pen)
        if name is not None:
            # Zero-point PlotDataItem, not tracked in lap_curve_items (that
            # dict is keyed by lap_number and iterated by _recompute_
            # emphasis/_restyle_lap_curves for per-lap bold/faint styling
            # -- a threshold-line entry there would get recoloured on the
            # next emphasis pass). plot.clear() at the top of every show_
            # corner/show_lap call removes it along with everything else,
            # same as the InfiniteLine itself -- no separate cleanup needed.
            self.plots[panel_key].plot([], [], pen=pen, name=name)

    def _apply_ratio_y_range(self):
        # Cleanup pass, Phase 1 (generalised, Part B): the CS/LS ratio
        # panels share one fixed y-range instead of each auto-ranging
        # independently. CS_ratio and LS_ratio are the SAME 1/0/negative
        # scale (1 = linear region, 0 = at the peak, below 0 = beyond it --
        # both modules/stability_analysis.py's estimate_cornering_
        # stiffness and modules/longitudinal_stiffness.py's estimate_
        # longitudinal_stiffness clip only the +1.0 ceiling, never a
        # floor), but LS_ratio's occasional numerically-unstable windows
        # (Phase 0 finding, this package: a kappa span barely above
        # min_slip_span's own validity gate blows the OLS slope up, values
        # as extreme as -31.8 seen on Dubai) would otherwise force auto-
        # range so wide the meaningful 0..1 region on every ratio panel
        # collapses to an unreadable sliver. Config-driven, corner_trace_
        # display namespace (same block as margin_before_m/after_m);
        # default bound is data-derived from CS_ratio's own observed range
        # on Dubai at the production kinematic config (min -3.065 rear /
        # -1.872 front) plus headroom. Out-of-range samples are simply not
        # drawn past the panel edge -- pyqtgraph's own viewbox clipping,
        # the underlying array is untouched. Pending Phase 0's own open
        # decision (a proper numerical-stability fix to LS_ratio itself),
        # which may supersede this display-only clip.
        from modules.stability_analysis import load_parameters
        margin_cfg = load_parameters().get("corner_trace_display", {})
        y_min = margin_cfg.get("cs_ls_panel_y_min", -3.5)
        y_max = margin_cfg.get("cs_ls_panel_y_max", 1.2)
        # ls_f/ls_r are absent from LapTraceDialog's RATIO_PANEL_KEYS (A4:
        # no lap-level LS reading) -- guard each key's presence rather
        # than assuming all four exist, since this method is shared.
        for key in self.RATIO_PANEL_KEYS:
            if key in self.plots:
                self.plots[key].setYRange(y_min, y_max, padding=0)


class CornerTraceDialog(_TraceDialogBase):
    """Reusable, non-modal per-corner trace window. One instance lives on
    the form (created lazily); opening it for a different corner replots
    in place rather than creating a new window, same pattern as the
    corner map's clear()-and-redraw (outing_form.py
    _update_corner_map_trace).
    """

    WINDOW_TITLE = "Corner Trace"

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg

        # WP-A item 3: wrap the existing s_m-trace scaffold (self.pg_layout,
        # built by the base class) as one tab, add a second tab for the
        # tyre-curve scatter -- reparented in place rather than restructuring
        # _TraceDialogBase, so LapTraceDialog (the other subclass) is
        # untouched. self.plots (stab/cs/speed) stays exactly what
        # _add_kerb_bands/_add_notmoving_bands/_add_threshold_line iterate
        # over; the tyre-curve panels live in their own self.tyre_plots so
        # those s_m-axis-shaped helpers never see them.
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
        self.tyre_legend_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")  # UI cleanup: 10px -> 12px
        tyre_layout.addWidget(self.tyre_legend_label)

        self.tyre_pg_layout = pg.GraphicsLayoutWidget()
        self.tyre_pg_layout.setBackground(PANEL)
        tyre_layout.addWidget(self.tyre_pg_layout)

        # Part B, Q2: track map goes above the front/rear tyre curves,
        # full width -- spatially tied to the axle-window context those
        # panels already show, rather than the Traces tab (Q1 kept that
        # tab's own 4-panel stack unchanged). Equal-aspect (unlike the
        # tyre-curve panels below it) since this is a literal plan-view
        # position plot, not a slope comparison.
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
            plot.setLabel('left', 'Lateral force Fy (N)', color='#888', size='8pt')
            plot.setLabel('bottom', 'Slip angle (deg)', color='#888', size='8pt')
            plot.showGrid(x=True, y=True, alpha=0.15)
            # Equal-axis scaling deliberately OFF (WP-A item 3 spec) -- alpha
            # is a few degrees' range, Fy a few thousand newtons; forcing a
            # 1:1 pixel scale would make the reference slope unreadable.
            # Corrections batch, item 5: "legend outside the axes (below
            # the panel), never over data" -- no addLegend() on the plot
            # itself any more (that anchors INSIDE the viewbox); each
            # axle's legend is now a standalone LegendItem placed in its
            # own layout row below (see tyre_legends below), populated by
            # explicit addItem() calls in _render_tyre_curves rather than
            # pyqtgraph's name= auto-registration (which only targets a
            # plot's own attached legend).
            self.tyre_plots[axle] = plot
        self.tyre_pg_layout.nextRow()

        self.tyre_legends = {}
        for col, axle in enumerate(("front", "rear")):
            legend = pg.LegendItem(labelTextSize=LEGEND_FONT_PT)
            legend.setBrush(pg.mkBrush(PANEL_ALT))
            legend.setLabelTextColor(TEXT_MUTED)
            self.tyre_pg_layout.addItem(legend, row=2, col=col)
            self.tyre_legends[axle] = legend
        # Legend row stays short -- most of the tab's height goes to the
        # track map and the tyre-curve scatter panels themselves.
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(0, 3)
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(1, 6)
        self.tyre_pg_layout.ci.layout.setRowStretchFactor(2, 1)

        self.tabs.addTab(tyre_tab, "Tyre Curves")

        # Part B, Q3: one "Export figure" (full chair-style composition,
        # this corner only) and one "Export verdict traces" (SetupTool's
        # own stability/CS/LS/speed stack) -- both read whatever corner is
        # CURRENTLY displayed regardless of which tab is active, so a
        # single _export_data cache (populated at the end of show_corner)
        # backs both buttons rather than each button re-deriving data from
        # whichever tab happens to be open.
        self._export_data = None
        # A1: everything checked-set-dependent (tyre curves, track map,
        # export) is derived from this context on every render AND every
        # checkbox toggle -- see _refresh_checked_dependent_views.
        self._render_ctx = None
        export_row = QHBoxLayout()
        self.export_figure_btn = QPushButton("Export figure")
        self.export_figure_btn.clicked.connect(self._on_export_figure_clicked)
        self.export_verdict_btn = QPushButton("Export verdict traces")
        self.export_verdict_btn.clicked.connect(self._on_export_verdict_clicked)
        export_row.addWidget(self.export_figure_btn)
        export_row.addWidget(self.export_verdict_btn)
        export_row.addStretch(1)

        outer_layout.insertWidget(pg_index, self.tabs)
        outer_layout.insertLayout(pg_index + 1, export_row)

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

    def _clear_tyre_curves(self):
        for plot in self.tyre_plots.values():
            plot.clear()
        for legend in self.tyre_legends.values():
            legend.clear()

    def _render_tyre_curves(self, instances, laps_by_number, bracket_start_m, bracket_end_m,
                             t, s_m, slip, forces, cs, kerb_mask, params, session_mask,
                             sideslip_source=None, fit_manifest=None):
        """Tyre Curves tab: per-axle slip-angle-vs-lateral-force scatter for
        this corner's own canonical window (bracket_start_m/end_m, no
        approach/coast-out margin -- unlike the Traces tab). Plot design
        after chair performance_analysis tooling (internal, its
        create_evaluation_plot scatter panel and create_scatter grouping-
        axis pattern); no code copied.

        Part B redesign: corner samples plot in the LAP'S OWN colour
        (filled circle, plot_style.lap_styles); a kerb-flagged sample of
        that SAME lap is the hollow (unfilled) version of the same
        colour/marker, so a kerb correction stays visibly tied to which
        lap it came from instead of one shared neutral cloud. Beyond-peak
        (throwaway) tyre operation shows as a lap's own cloud folding BACK
        over the reference slope line -- Fy falling while |slip angle|
        keeps growing -- rather than scattering around it.

        The reference line uses ONE value per axle (median of
        C_linear_ref_f/r over every valid sample pooled across the
        checked laps' canonical windows), not a per-lap or per-sample
        line. C_linear_ref is already NaN at any kerb-flagged or
        non-moving sample (estimate_cornering_stiffness only writes it
        past that gate), so no separate kerb exclusion is needed for this
        median.

        Part B also brings the estimation-window highlight and its
        tangent line INTO this interactive tab (previously export-only,
        see _build_tyre_curve_export) -- hollow black ring, fixed colour
        (not lap-identified: only one window is ever highlighted per
        axle, pooled across the checked laps' own worst phase), on top of
        everything else.

        Corrections batch (post-visual-review): a very light/thin whole-
        SESSION background cloud (session_mask -- moving & not kerb-
        flagged, the same population the PRINT export's own session
        cloud already used) is now drawn first, underneath everything, so
        this corner's own samples read as a highlighted subset of the
        car's overall tyre behaviour rather than floating in isolation.
        Each axle's legend is a standalone LegendItem placed OUTSIDE that
        axle's own plot (see __init__) -- populated by explicit addItem()
        calls below, not name= auto-registration.

        UI cleanup package: when `sideslip_source` is one of the two
        auto-fit modes and `fit_manifest` carries that session's own
        fitted curve parameters, the FITTED model curve (Dugoff or
        Pacejka, whichever ran) is drawn over the same cloud, evaluated
        across this corner's own visited slip-angle range -- unlike the
        reference line (a single linear slope through the origin, the
        LINEAR-REGIME estimate only), the fitted curve is the actual
        nonlinear model the EKF used, and can show the saturation/peak
        the linear reference cannot. sideslip_source/fit_manifest are
        None for kinematic/ekf_pass_1 (and for kinematic, fit_manifest
        genuinely does not exist -- no fit chain ran) -- current
        (reference-line-only) behaviour is unchanged in that case.

        A1 fix: `instances` is filtered to the currently CHECKED laps only
        (self._checked_instances) before anything else -- an unchecked lap
        contributes no scatter points and no share of the pooled linear-
        reference slope or worst-phase window. Called again in full on
        every checkbox toggle (see _refresh_checked_dependent_views), not
        just hidden/shown.
        """
        import pyqtgraph as pg
        from modules.stability_analysis import reconstruct_cs_window_start

        self._clear_tyre_curves()
        if slip is None or forces is None:
            return

        # A1: an unchecked lap contributes NOTHING here -- neither its own
        # scatter cloud nor its samples' share of the pooled linear-
        # reference slope/worst-phase window below. Filtered once, up
        # front, rather than gating visibility per item after the fact.
        instances = self._checked_instances(instances)
        styles = plot_style.lap_styles(c["lap_number"] for c in instances)

        def _alpha_brush(color, alpha_frac):
            qcolor = pg.mkColor(color)
            qcolor.setAlpha(int(round(alpha_frac * 255)))
            return pg.mkBrush(qcolor)

        def _alpha_pen(color, alpha_frac, width):
            qcolor = pg.mkColor(color)
            qcolor.setAlpha(int(round(alpha_frac * 255)))
            return pg.mkPen(qcolor, width=width)

        axle_specs = (
            ("front", slip.get("alpha_f_filt"), forces.get("Fy_f_filt"),
             cs.get("C_linear_ref_f") if cs is not None else None,
             cs.get("CS_ratio_f") if cs is not None else None,
             cs.get("C_alpha_f") if cs is not None else None),
            ("rear", slip.get("alpha_r_filt"), forces.get("Fy_r_filt"),
             cs.get("C_linear_ref_r") if cs is not None else None,
             cs.get("CS_ratio_r") if cs is not None else None,
             cs.get("C_alpha_r") if cs is not None else None),
        )

        for axle, alpha_arr, Fy_arr, ref_arr, cs_ratio_arr, c_alpha_arr in axle_specs:
            plot = self.tyre_plots[axle]
            legend = self.tyre_legends[axle]
            if alpha_arr is None or Fy_arr is None:
                continue

            # Corrections batch: whole-session background cloud, very
            # light/thin, drawn FIRST (underneath the corner's own
            # samples) -- same moving-and-not-kerb-flagged population the
            # PRINT export's own session cloud already used.
            if session_mask is not None:
                session_valid = session_mask & np.isfinite(alpha_arr) & np.isfinite(Fy_arr)
                if session_valid.any():
                    session_item = plot.plot(
                        np.degrees(alpha_arr[session_valid]), Fy_arr[session_valid],
                        pen=None, symbol='o', symbolSize=SESSION_CLOUD_SIZE_SCREEN,
                        symbolBrush=pg.mkBrush(SESSION_CLOUD_COLOR_SCREEN), symbolPen=None,
                    )
                    session_item.setZValue(-1)
                    legend.addItem(session_item, "Session")

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
                lap_kerb = kerb_mask[sl] if kerb_mask is not None else np.zeros(sl.stop - sl.start, dtype=bool)

                valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
                clean = valid & ~lap_kerb
                kerbed = valid & lap_kerb
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

            # UI cleanup package: fitted model curve, auto modes only.
            # Evaluated in RADIANS (the model's own natural unit, same as
            # dugoff_lateral_force/pacejka_lateral_force's own contract)
            # over a fine grid spanning this corner's own visited alpha
            # range, THEN converted to degrees for the x-axis -- the same
            # radians-in/degrees-for-display pattern the reference line
            # above already uses (x_line_rad computed in radians, only
            # np.degrees() at the plot() call) -- avoids the N/rad-vs-
            # degrees tangent trap on record (thesis_notes.md): the
            # model's Fy values themselves need no unit conversion at
            # all (N stays N regardless of how alpha is displayed), only
            # the x-axis representation changes.
            if (sideslip_source in ("ekf_auto_dugoff", "ekf_auto_pacejka")
                    and fit_manifest is not None and max_abs_alpha > 0):
                axle_fit = fit_manifest.get("axles", {}).get(axle)
                if axle_fit is not None:
                    alpha_grid_rad = np.linspace(-max_abs_alpha, max_abs_alpha, 200)
                    if sideslip_source == "ekf_auto_dugoff":
                        from modules.tyre_model import dugoff_lateral_force
                        fy_grid = dugoff_lateral_force(
                            alpha_grid_rad, axle_fit["c_alpha_n_per_rad"], axle_fit["mu_fz_N"]
                        )
                        model_name = "Dugoff"
                    else:
                        from modules.tyre_model_pacejka import pacejka_lateral_force
                        fy_grid = pacejka_lateral_force(
                            alpha_grid_rad, axle_fit["B"], axle_fit["C"], axle_fit["D"], axle_fit["E"]
                        )
                        model_name = "Pacejka"
                    item = plot.plot(
                        np.degrees(alpha_grid_rad), fy_grid,
                        pen=pg.mkPen(color=FITTED_CURVE_COLOR, width=2, style=Qt.PenStyle.SolidLine),
                    )
                    legend.addItem(item, f"Fitted tyre model ({model_name})")

            # Part B: estimation window (hollow black ring, on top) and its
            # tangent line, now live in this interactive tab too -- same
            # worst-phase computation _build_tyre_curve_export uses for
            # the PRINT export, so the two always show identical geometry.
            if cs_ratio_arr is not None:
                wp = _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m,
                                      bracket_start_m, bracket_end_m)
                if wp is not None:
                    min_window = params["stability_estimation"]["cs_min_window_samples"]
                    min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]
                    idx = wp["index"]
                    start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span)
                    window_sl = slice(start, idx)
                    if window_sl.stop > window_sl.start:
                        window_item = plot.plot(np.degrees(alpha_arr[window_sl]), Fy_arr[window_sl], pen=None,
                                                 symbol='o', symbolSize=WINDOW_RING_SIZE_SCREEN, symbolBrush=None,
                                                 symbolPen=pg.mkPen(WINDOW_RING_COLOR, width=1.4))
                        window_item.setZValue(10)
                        legend.addItem(window_item, "Estimation window")
                    cs_n_per_rad = float(c_alpha_arr[idx]) if c_alpha_arr is not None else None
                    if cs_n_per_rad is not None and np.isfinite(cs_n_per_rad) and max_abs_alpha > 0:
                        x0 = np.degrees(alpha_arr[idx])
                        y0 = Fy_arr[idx]
                        slope_per_deg = cs_n_per_rad * (np.pi / 180.0)
                        span = max(0.5, 0.15 * max_abs_alpha * (180.0 / np.pi))
                        xs = np.array([x0 - span, x0 + span])
                        ys = y0 + slope_per_deg * (xs - x0)
                        tangent_item = plot.plot(xs, ys, pen=pg.mkPen(color=TEXT, width=1, style=Qt.PenStyle.DashLine))
                        legend.addItem(tangent_item, f"Tangent CS={cs_n_per_rad:.0f} N/rad")

    def _render_track_map(self, representative, bracket_start_m, bracket_end_m, t, s_m, state,
                           cs, slip, params, instances, laps_by_number):
        """Track map (Tyre Curves tab, Part B Q2): the representative lap's
        own GPS trace (grey, context outline), one bracket polyline PER
        CHECKED LAP in that lap's own colour (Part B: each lap drives a
        slightly different physical line through the corner, so the
        bracket really is that lap's own data, unlike the fixed bracket_
        start_m/end_m s-range), and each axle's worst-phase estimation
        window as a hollow black ring (front solid, rear dotted -- same
        "hollow ring" marker language as the tyre-curve panels, reused
        here since only one window per axle is ever highlighted so no
        second colour vocabulary is needed to tell them apart from each
        other). Reuses modules.stability_analysis.reconstruct_cs_
        window_start (Tier B, verified against production) to locate each
        window's raw samples -- computes no CS value of its own, pure
        display. Returns the same geometry dict core/figure_render.py's
        track_map argument expects, so show_corner can hand it straight to
        the export button without a second computation; None if this
        outing has no GPS channel to project.

        A1 fix: the front/rear window highlight is pooled only over
        CHECKED instances (self._checked_instances) -- an unchecked lap
        can never surface (or keep) a window highlight on the map. The
        background lap outline is NOT lap-specific measured data (it is
        context/orientation only) and stays drawn from the analysed
        `representative` lap regardless of checked state -- explicitly
        confirmed by the work order's own "with no lap checked, panels are
        empty except threshold lines and the track outline" rule.
        """
        import pyqtgraph as pg
        from modules.geo import project_latlon_to_xy
        from modules.stability_analysis import reconstruct_cs_window_start

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

        min_window = params["stability_estimation"]["cs_min_window_samples"]
        min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]

        def _window_xy(cs_ratio_arr, alpha_arr):
            if cs_ratio_arr is None or alpha_arr is None:
                return None
            wp = _worst_cs_phase(cs_ratio_arr, checked_instances, laps_by_number, t, s_m,
                                  bracket_start_m, bracket_end_m)
            if wp is None:
                return None
            start = reconstruct_cs_window_start(alpha_arr, wp["index"], min_window, min_span)
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
                                  sideslip_source, fit_manifest):
        """Assemble one axle's core/figure_render.py tyre_curves[axle]
        entry: session scatter (whole-session background, NEW for export
        -- the interactive tab only ever shows this corner's own window,
        never the full session), corner scatter PER CHECKED LAP (Part B:
        filled = clean sample in that lap's own colour, hollow = kerb-
        flagged sample of that same lap -- same convention _render_tyre_
        curves now uses live), the worst-phase estimation window, the
        linear-reference line (same median-of-C_linear_ref computation
        _render_tyre_curves uses), the auto-fit model curve (same Dugoff/
        Pacejka evaluation), and a tangent line through the worst-phase
        sample at slope CS[N/rad] * pi/180 -- the chair-comparable
        composition's own element, not otherwise shown in this dialog.
        """
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
            lap_kerb = kerb_mask[sl] if kerb_mask is not None else np.zeros(sl.stop - sl.start, dtype=bool)
            valid = np.isfinite(lap_alpha) & np.isfinite(lap_Fy)
            clean = valid & ~lap_kerb
            kerbed = valid & lap_kerb
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
        if (sideslip_source in ("ekf_auto_dugoff", "ekf_auto_pacejka")
                and fit_manifest is not None and max_abs_alpha > 0):
            axle_fit = fit_manifest.get("axles", {}).get(axle)
            if axle_fit is not None:
                alpha_grid_rad = np.linspace(-max_abs_alpha, max_abs_alpha, 200)
                if sideslip_source == "ekf_auto_dugoff":
                    from modules.tyre_model import dugoff_lateral_force
                    fy_grid = dugoff_lateral_force(alpha_grid_rad, axle_fit["c_alpha_n_per_rad"], axle_fit["mu_fz_N"])
                    model_name = "Dugoff"
                else:
                    from modules.tyre_model_pacejka import pacejka_lateral_force
                    fy_grid = pacejka_lateral_force(alpha_grid_rad, axle_fit["B"], axle_fit["C"],
                                                     axle_fit["D"], axle_fit["E"])
                    model_name = "Pacejka"
                fitted_line = (np.degrees(alpha_grid_rad), fy_grid, f"Fitted tyre model ({model_name})")

        window_xy = None
        tangent_line = None
        if cs_ratio_arr is not None:
            from modules.stability_analysis import reconstruct_cs_window_start
            wp = _worst_cs_phase(cs_ratio_arr, instances, laps_by_number, t, s_m,
                                  bracket_start_m, bracket_end_m)
            if wp is not None:
                min_window = params["stability_estimation"]["cs_min_window_samples"]
                min_span = params["stability_estimation"]["cs_min_slip_angle_span_rad"]
                idx = wp["index"]
                start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span)
                window_sl = slice(start, idx)
                if window_sl.stop > window_sl.start:
                    window_xy = (np.degrees(alpha_arr[window_sl]), Fy_arr[window_sl])
                cs_n_per_rad = float(c_alpha_arr[idx]) if c_alpha_arr is not None else None
                if cs_n_per_rad is not None and np.isfinite(cs_n_per_rad):
                    x0 = np.degrees(alpha_arr[idx])
                    y0 = Fy_arr[idx]
                    slope_per_deg = cs_n_per_rad * (np.pi / 180.0)
                    span = max(0.5, 0.15 * max_abs_alpha * (180.0 / np.pi)) if max_abs_alpha > 0 else 1.0
                    xs = np.array([x0 - span, x0 + span])
                    ys = y0 + slope_per_deg * (xs - x0)
                    tangent_line = (xs, ys, f"Tangent CS={cs_n_per_rad:.0f} N/rad")

        return {
            "session_xy": session_xy, "corner_by_lap": corner_by_lap, "window_xy": window_xy,
            "linear_ref_line": ref_line, "fitted_line": fitted_line, "tangent_line": tangent_line,
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
                )
                figure_render.save_png(fig, path)
                saved_paths = [path]
            else:
                # render_verdict_traces_figure returns a LIST -- normally
                # one figure, but a genuinely too-cramped six-panel stack
                # would return two (corrections round 3, item 2); saving
                # every entry the composition returns means this call site
                # never has to know which case it got.
                figs = figure_render.render_verdict_traces_figure(
                    data["corner_label"], data["laps"], data["thresholds"], theme=plot_style.PRINT,
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
        # A1: single source of truth for "which of this corner's instances
        # currently count" -- every checked-set-dependent computation
        # (tyre-curve scatter/pooled reference, track-map windows, export)
        # filters through this rather than re-deriving lap_visible lookups
        # independently.
        return [c for c in instances if self.lap_visible.get(c["lap_number"], True)]

    def _refresh_checked_dependent_views(self):
        """Recompute everything that POOLS or AGGREGATES across the
        checked set: the tyre-curve linear-reference slope, the front/rear
        worst-phase window highlight, and the export cache. These are not
        a single lap's own togglable curve -- toggling one checkbox can
        change what the aggregate itself is, so each must be rebuilt in
        full on every toggle (see _on_lap_visibility_toggled), not merely
        hidden/shown like a per-lap trace curve. Called once at the end of
        the initial show_corner render too, so there is exactly one code
        path for "what the checked set currently implies" regardless of
        whether this is the first render or a toggle-triggered one.
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
            )
            # Follow-up item 2: render_corner_figure has the track map
            # back (narrow row) -- _render_track_map still draws directly
            # onto self.track_map_plot for the interactive tab as a side
            # effect, but its return value is captured again now that the
            # export composition needs the same geometry too.
            track_map = self._render_track_map(
                ctx["representative"], ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"],
                ctx["state"], ctx["cs"], ctx["slip"], ctx["params"], ctx["instances"], ctx["laps_by_number"],
            )
        self._build_export_data(track_map)

    def _build_export_data(self, track_map):
        # Part B, Q3 / addendum item 4: "export renders EXACTLY the
        # checked set" -- filters ctx["all_export_laps"] (built for every
        # instance, once, in show_corner) down to the currently-checked
        # lap numbers, and rebuilds both axles' tyre-curve export geometry
        # from checked_instances only, so a toggle never leaves the export
        # cache describing a stale checked set.
        ctx = self._render_ctx
        if (ctx is None or ctx["bracket_start_m"] is None or ctx["bracket_end_m"] is None
                or ctx["slip"] is None or ctx["forces"] is None):
            self._export_data = None
            return

        checked_instances = self._checked_instances(ctx["instances"])
        checked_lap_numbers = {c["lap_number"] for c in checked_instances}
        laps = [lap for lap in ctx["all_export_laps"] if lap["lap_number"] in checked_lap_numbers]

        slip, forces, cs = ctx["slip"], ctx["forces"], ctx["cs"]
        tyre_curves = {
            "front": self._build_tyre_curve_export(
                "front", slip.get("alpha_f_filt"), forces.get("Fy_f_filt"),
                cs.get("C_linear_ref_f") if cs is not None else None,
                cs.get("CS_ratio_f") if cs is not None else None,
                cs.get("C_alpha_f") if cs is not None else None,
                ctx["kerb_mask"], ctx["session_mask"], checked_instances, ctx["laps_by_number"],
                ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"], ctx["params"],
                ctx["sideslip_source"], ctx["fit_manifest"],
            ),
            "rear": self._build_tyre_curve_export(
                "rear", slip.get("alpha_r_filt"), forces.get("Fy_r_filt"),
                cs.get("C_linear_ref_r") if cs is not None else None,
                cs.get("CS_ratio_r") if cs is not None else None,
                cs.get("C_alpha_r") if cs is not None else None,
                ctx["kerb_mask"], ctx["session_mask"], checked_instances, ctx["laps_by_number"],
                ctx["bracket_start_m"], ctx["bracket_end_m"], ctx["t"], ctx["s_m"], ctx["params"],
                ctx["sideslip_source"], ctx["fit_manifest"],
            ),
        }
        self._export_data = {
            "corner_label": ctx["corner_label"], "laps": laps, "thresholds": ctx["thresholds"],
            "tyre_curves": tyre_curves, "track_map": track_map,
        }

    def _rerender_preserving_checked(self):
        # Part B: colour and each panel's per-lap legend depend on the
        # WHOLE checked set -- re-run show_corner from scratch (same
        # summary/stability_result/parsed_data as the last real open),
        # handing the checkboxes' CURRENT state back in via
        # preserve_visible so the user's own choices survive the rebuild.
        if self._last_show_args is not None:
            self.show_corner(*self._last_show_args, preserve_visible=dict(self.lap_visible))

    def show_corner(self, summary, stability_result, parsed_data, preserve_visible=None):
        """Repopulate in place for `summary`'s stable_corner_id. `summary`
        is the single lap's corner-detail summary the trace button was
        clicked from -- its own lap_number is the representative lap for
        phase-band/track-map-background/initial-X-range positions; its
        phase medians decide which phase's band is tinted (matching the
        verdict badge already shown for that same card). No lap is drawn
        differently from any other (corrections batch: the bold/faint
        distinction is gone, see plot_style.SCREEN_LAP_WIDTH).
        """
        from modules.stability_analysis import load_parameters

        self._last_show_args = (summary, stability_result, parsed_data)

        for plot in self.plots.values():
            plot.clear()
        self._clear_tyre_curves()
        self.lap_curve_items = {}
        self._render_ctx = None

        stable_corner_id = summary["stable_corner_id"]
        state = stability_result.get("state")
        cs = stability_result.get("cs")
        stab = stability_result.get("stab")
        corners = stability_result.get("corners")
        # WP-A item 3: slip/forces (alpha_*_filt/Fy_*_filt) -- Tyre Curves
        # tab only; None on any render path that predates their addition to
        # the pipeline cache/result dict (ui/views/outing_form.py), handled
        # by _render_tyre_curves degrading to an empty tab, same as the
        # existing state/cs/stab-missing guard just above does for the
        # whole dialog.
        slip = stability_result.get("slip")
        forces = stability_result.get("forces")
        # PLAN.md STEP 3 Phase 3: ls (estimate_longitudinal_stiffness
        # output) -- same optional-graceful-degradation pattern as slip/
        # forces above, not part of the state/cs/stab/corners guard below
        # (a render predating this package, or an in-memory pipeline-cache
        # entry from before this session's own reload, simply shows no LS
        # curves rather than erroring).
        ls = stability_result.get("ls")
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
        # A4: display-only mask -- the LS panel only draws where |ax_mps2|
        # exceeds this bound (see config/parameters.json's own note for the
        # Dubai-distribution derivation). Never touches LS_ratio's own
        # numeric output, only what gets plotted/exported here.
        ls_display_min_ax = margin_cfg.get("ls_display_min_ax_mps2", 1.0)

        v_kmh = state["v_mps"] * 3.6
        cs_f = cs["CS_ratio_f"]
        cs_r = cs["CS_ratio_r"]
        ls_f = ls["LS_ratio_f"] if ls is not None else None
        ls_r = ls["LS_ratio_r"] if ls is not None else None
        ax_mps2 = state.get("ax_mps2")
        stab_obs = stab["stability_observed_Nm_per_deg"]
        kerb_mask = state.get("kerb_mask")
        moving_mask = state.get("moving_mask")

        selected_lap = summary["lap_number"]
        representative = next((c for c in instances if c["lap_number"] == selected_lap), instances[0])
        self._add_phase_bands(representative, t, s_m, worst_phase=_worst_stab_phase(summary))
        # Cleanup pass, Phase 1: when the analysis itself was run for one
        # specific lap (lap_filter is a single-element list), the trace
        # should default to showing only that lap too, not every valid
        # lap regardless of what was analysed. Addendum item 2: an "All
        # laps" analysis (lap_filter has more than one entry, or is
        # unavailable on an older cached render) now defaults to the
        # FASTEST N valid laps of this corner (N = corner_trace_display.
        # default_laps_shown), not every one of them -- avoids a wall of
        # overlapping colours by default on a corner with many laps, while
        # every other valid lap stays available to check.
        lap_filter = stability_result.get("lap_filter")
        if lap_filter and len(lap_filter) == 1:
            default_checked_laps = set(lap_filter)
        else:
            n_default = margin_cfg.get("default_laps_shown", 5)
            default_checked_laps = _fastest_n_laps(
                [c["lap_number"] for c in instances], laps_by_number, n_default)
        self._rebuild_lap_checkboxes(instances, selected_lap, default_checked_laps=default_checked_laps,
                                      preserve_visible=preserve_visible)

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
        # Part B: only CHECKED laps are ever plotted in the Traces tab now
        # (class docstring) -- an unchecked lap contributes no item at
        # all, not a hidden one, and colour is a function of exactly this
        # set (plot_style.lap_styles via _restyle_all_laps below).
        checked_instances = self._checked_instances(instances)
        # Part B, Q3: captured alongside the on-screen curves below (same
        # x/sl/order, not re-sliced) so "Export figure"/"Export verdict
        # traces" render exactly what's on screen -- see _build_export_data.
        export_laps = []
        self._lap_line_style = {}
        for c in checked_instances:
            is_selected = (c["lap_number"] == selected_lap)  # X-range fit target only -- NOT bold/faint, see _restyle_all_laps below
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

            # A4: LS display mask -- NaN out samples where |ax_mps2| falls
            # at or below the config bound, on both the on-screen curve and
            # the export copy, so screen and PRINT export always agree.
            ls_f_lap = ls_f[sl][order] if ls_f is not None else None
            ls_r_lap = ls_r[sl][order] if ls_r is not None else None
            if ls_f_lap is not None and ax_mps2 is not None:
                ax_gate = np.abs(ax_mps2[sl][order]) > ls_display_min_ax
                ls_f_lap = np.where(ax_gate, ls_f_lap, np.nan)
                ls_r_lap = np.where(ax_gate, ls_r_lap, np.nan)

            # Part B: every curve is plotted with a PLACEHOLDER pen here
            # (quiet-lap dash style only) -- _restyle_all_laps (called once
            # below, and again on every full checkbox-toggle re-render)
            # sets the real per-lap colour/width from the current checked
            # set. name=f"Lap N" is attached directly to each real item
            # here, not a dedicated dummy item, because ONLY checked laps
            # are ever plotted at all (class docstring) -- every named
            # item is, by construction, always visible, so the crossed-eye
            # legend risk Part A's A2 fixed cannot recur under this model.
            style = self._lap_line_style[c["lap_number"]]
            lap_name = f"Lap {c['lap_number']}"
            placeholder_pen_kwargs = dict(color=THRESHOLD_GREY, width=SCREEN_LAP_WIDTH, style=style)
            curve_items = [
                self.plots["speed"].plot(x, v_kmh[sl][order], connect="finite",
                                          pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["stab"].plot(x, stab_obs[sl][order], connect="finite",
                                         pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["cs_f"].plot(x, cs_f[sl][order], connect="finite",
                                        pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["cs_r"].plot(x, cs_r[sl][order], connect="finite",
                                        pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
            ]
            if ls_f_lap is not None:
                curve_items.append(self.plots["ls_f"].plot(
                    x, ls_f_lap, connect="finite", pen=self._pen(**placeholder_pen_kwargs), name=lap_name))
                curve_items.append(self.plots["ls_r"].plot(
                    x, ls_r_lap, connect="finite", pen=self._pen(**placeholder_pen_kwargs), name=lap_name))
            self.lap_curve_items[c["lap_number"]] = curve_items
            export_laps.append({
                "lap_number": c["lap_number"], "selected": is_selected,
                "s": x, "v_kmh": v_kmh[sl][order],
                "cs_f": cs_f[sl][order], "cs_r": cs_r[sl][order],
                "stab": stab_obs[sl][order],
                "ls_f": ls_f_lap, "ls_r": ls_r_lap,
            })

        # Part B: "Legend per panel: lap entries only, plus the fixed
        # lines" -- each panel's legend now lists the checked laps by
        # name (set directly on their own real curve above) plus whatever
        # threshold lines get added to that panel below; no separate
        # dedicated-entry step is needed any more (see class docstring).
        self._laps_by_number = laps_by_number
        self._restyle_all_laps()

        cls_cfg = params["classification"]

        # A1: build the session/racing mask once here (used by both axles'
        # tyre-curve export background) -- previously computed only inside
        # the export-data block below, now needed unconditionally since
        # _refresh_checked_dependent_views/_build_export_data run through
        # _render_ctx instead of this method's own local scope.
        valid_laps_by_number = {l["lap_number"]: l for l in parsed_data.get("laps", []) if l.get("is_valid_for_analysis")}
        session_mask = moving_mask if moving_mask is not None else np.ones_like(t, dtype=bool)
        if kerb_mask is not None:
            session_mask = session_mask & ~kerb_mask
        racing_mask = np.zeros_like(t, dtype=bool)
        for lap in valid_laps_by_number.values():
            racing_mask |= (t >= lap["start_time"]) & (t <= lap["end_time"])
        session_mask = session_mask & racing_mask

        # A1: everything checked-set-dependent (tyre curves, track map,
        # export) now runs through one shared context + one shared refresh
        # method -- see _refresh_checked_dependent_views's own docstring.
        self._render_ctx = {
            "corner_label": f"C{stable_corner_id}",
            "instances": instances, "laps_by_number": laps_by_number,
            "bracket_start_m": bracket_start_m, "bracket_end_m": bracket_end_m,
            "t": t, "s_m": s_m, "slip": slip, "forces": forces, "cs": cs,
            "kerb_mask": kerb_mask, "state": state, "params": params,
            "sideslip_source": stability_result.get("sideslip_source"),
            "fit_manifest": stability_result.get("fit_manifest"),
            "representative": representative, "all_export_laps": export_laps,
            "session_mask": session_mask,
            "thresholds": {
                "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
                "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
                "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
            },
        }
        self._refresh_checked_dependent_views()
        # Part B: threshold lines are ALL neutral grey now, differentiated
        # by style only -- strong=dashed, moderate=dotted, unstable=dash-
        # dot (work order's own spec); front/rear CS now live on separate
        # panels so no colour clash between the two axles' thresholds can
        # happen any more.
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
        # Corrections round 3, item 4: LS panels have no classification
        # thresholds (PLAN.md STEP 3 -- display only), so a dotted zero
        # line is the one reference line they get, matching the export
        # side (core/figure_render.py's _draw_ratio_panel zero_line=True).
        self._add_threshold_line("ls_f", 0.0, THRESHOLD_GREY, Qt.PenStyle.DotLine, name="Zero")
        self._add_threshold_line("ls_r", 0.0, THRESHOLD_GREY, Qt.PenStyle.DotLine, name="Zero")

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
            plot.setXRange(x_lo, x_hi, padding=0.02)
        # Ratio panels get a fixed y-range (see _apply_ratio_y_range);
        # only "stab"/"speed" still auto-range.
        for key in ("stab", "speed"):
            self.plots[key].enableAutoRange(axis='y')
        self._apply_ratio_y_range()

        n_laps = len(instances)
        n_quiet = sum(1 for c in instances if "canonical_quiet" in c.get("warnings", []))
        # Human-readable, not the internal "canonical_quiet" warning code --
        # this label is user-facing (Q2 follow-up).
        quiet_note = f", {n_quiet} lap(s) not natively detected here (dashed)" if n_quiet else ""
        # Corrections batch: no lap is drawn differently from any other
        # any more -- colour is the only thing that distinguishes them.
        self.header_label.setText(
            f"C{stable_corner_id} -- {n_laps} valid lap(s) available{quiet_note}, unchecked laps hidden -- "
            f"context margin -{margin_before_m:g}m/+{margin_after_m:g}m outside the bracket"
        )

        legend_parts = [BASE_LEGEND_TEXT]
        if ls_f is not None and ls_r is not None:
            legend_parts.append(LS_PANEL_LEGEND_TEXT)
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
    map. Same 4-panel scaffold as CornerTraceDialog (shared base class),
    unwindowed over the shown lap's own 0..s_max range, with one labeled
    background band per stable corner tinted by that corner's WORST
    verdict across all its valid-lap instances (not just this lap's own
    instance -- see _aggregate_worst_severity). Clicking a band opens the
    existing CornerTraceDialog for that corner via on_corner_click.
    """

    WINDOW_TITLE = "Lap Trace"
    WINDOW_SIZE = (1100, 680)

    # A4: no ls_f/ls_r panels here at all (LS_ratio has no lap-level
    # reading, see this class's own docstring).
    PANEL_TITLES = [
        ("Speed (km/h)", "speed"), ("Stability (Nm/deg)", "stab"),
        ("Front CS ratio", "cs_f"), ("Rear CS ratio", "cs_r"),
    ]
    RATIO_PANEL_KEYS = ("cs_f", "cs_r")

    def __init__(self, parent=None, on_corner_click=None):
        super().__init__(parent)
        self._on_corner_click = on_corner_click
        self._corner_bands = []          # [(start_s, end_s, stable_corner_id), ...] for click hit-testing
        self._corner_summary_by_id = {}  # stable_corner_id -> this render's displayed-lap summary
        self._laps_by_number = {}        # lap_number -> lap dict, for _fastest_lap's lap_time lookup
        self.pg_layout.scene().sigMouseClicked.connect(self._on_scene_clicked)

        # LAP TRACE EXPORT item: same mechanism as CornerTraceDialog's own
        # "Export figure" button -- populated at the end of every show_lap.
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
                theme=plot_style.PRINT,
            )
            figure_render.save_png(fig, path)
        except Exception as e:
            from core.error_text import friendly_error_text
            QMessageBox.warning(self, "Export figure", f"Could not export figure ({friendly_error_text(e)}).")
            return
        QMessageBox.information(self, "Export figure", f"Saved to {path}")

    # _restyle_lap_curves/_restyle_all_laps/_on_lap_visibility_toggled:
    # all inherited unchanged from _TraceDialogBase.

    def _rerender_preserving_checked(self):
        # Part B: same "re-run the last real render, checked set
        # preserved" pattern as CornerTraceDialog -- see that class's own
        # _rerender_preserving_checked for why a full re-render, not
        # incremental item patching.
        if self._last_show_args is not None:
            lap_number, stability_result, parsed_data, classify_fn = self._last_show_args
            self.show_lap(lap_number, stability_result, parsed_data, classify_fn,
                           preserve_visible=dict(self.lap_visible))

    def _load_lap_data(self, lap_number, stability_result, parsed_data):
        """Slice state/cs/stab to lap_number's full valid-lap range via
        _lap_slice (the same clamp+reset-guard-trim primitive the corner
        window's margin extension uses, just without a margin). Returns a
        dict of ready-to-plot arrays, or None if this lap contributes no
        usable samples.
        """
        # A4: no "ls" read here at all -- LS_ratio has no lap-level
        # reading (see this class's own docstring), the full-lap trace
        # never had one.
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
        # A3 fix: double-click only. pyqtgraph's GraphicsScene emits
        # sigMouseClicked on every ordinary click too (including the first
        # click of a pan/drag sequence -- the scene queues a MouseClickEvent
        # on press and sends it on release regardless of how far the mouse
        # travelled in between in the common case), so a plain pan across
        # a corner band was opening CornerTraceDialog by accident.
        # event.double() is True only for the second click of an actual
        # double-click (pyqtgraph.GraphicsScene.mouseDoubleClickEvent
        # queues a second MouseClickEvent(double=True) on top of the first
        # click/release pair) -- gating on it makes a single click, and a
        # drag that ends in a single release, inert.
        if not event.double():
            return
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

    def show_lap(self, lap_number, stability_result, parsed_data, classify_fn, on_corner_click=None,
                 preserve_visible=None):
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
        self._last_show_args = (lap_number, stability_result, parsed_data, classify_fn)

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
        # Corrections round 3, item 5: default checked = fastest N valid
        # laps (matching CornerTraceDialog's own addendum default), not
        # just the single lap this view was opened for -- the viewed lap
        # is still guaranteed checked (it is what the user explicitly
        # asked to see, and _rebuild_lap_checkboxes already labels it
        # "(selected)"), unioned in rather than swapped out if it falls
        # outside the fastest-N set.
        margin_cfg = load_parameters().get("corner_trace_display", {})
        n_default = margin_cfg.get("default_laps_shown", 5)
        default_checked_laps = _fastest_n_laps(
            sorted(valid_lap_numbers), laps_by_number, n_default) | {lap_number}
        self._rebuild_lap_checkboxes(
            checkbox_instances, lap_number,
            default_checked_laps=default_checked_laps, fastest_lap=fastest_overall,
            preserve_visible=preserve_visible,
        )

        # Part B: only CHECKED laps are ever plotted (class docstring) --
        # colour comes from plot_style.lap_styles over exactly this set,
        # assigned below by _restyle_all_laps.
        checked_lap_numbers = sorted(ln for ln, v in self.lap_visible.items() if v)

        any_kerb = False
        any_not_moving = False
        export_laps = []
        for ln in checked_lap_numbers:
            data = self._load_lap_data(ln, stability_result, parsed_data)
            if data is None:
                continue

            if data["kerb"] is not None and data["kerb"].any():
                any_kerb = True
                self._add_kerb_bands(data["x"], data["kerb"])
            if data["not_moving"] is not None and data["not_moving"].any():
                any_not_moving = True
                self._add_notmoving_bands(data["x"], data["not_moving"])

            # Part B: PLACEHOLDER pen here (colour/width are set for real
            # by _restyle_all_laps, called once below) -- name=f"Lap N" is
            # attached directly since only checked laps are ever plotted
            # (see class docstring), so the item is always visible.
            lap_name = f"Lap {ln}"
            placeholder_pen_kwargs = dict(color=THRESHOLD_GREY, width=SCREEN_LAP_WIDTH,
                                           style=Qt.PenStyle.SolidLine)
            curve_items = [
                self.plots["speed"].plot(data["x"], data["speed"], connect="finite",
                                          pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["stab"].plot(data["x"], data["stab"], connect="finite",
                                         pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["cs_f"].plot(data["x"], data["csf"], connect="finite",
                                        pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
                self.plots["cs_r"].plot(data["x"], data["csr"], connect="finite",
                                        pen=self._pen(**placeholder_pen_kwargs), name=lap_name),
            ]
            self.lap_curve_items[ln] = curve_items
            # LAP TRACE EXPORT item: figure_render's panel functions expect
            # this exact key shape (s/v_kmh/stab/cs_f/cs_r), matching
            # CornerTraceDialog's own all_export_laps entries -- built here
            # from the same arrays already sliced for the interactive plot,
            # not re-sliced.
            export_laps.append({
                "lap_number": ln, "s": data["x"], "v_kmh": data["speed"],
                "stab": data["stab"], "cs_f": data["csf"], "cs_r": data["csr"],
            })

        # Part B: recomputes the per-lap colour assignment from the
        # current checked set -- the same recompute the checkbox-toggle
        # handler triggers via a full re-render, so the initial render and
        # every subsequent toggle go through one rule.
        self._restyle_all_laps()

        cls_cfg = load_parameters()["classification"]
        # Part B: threshold lines are neutral grey, style-differentiated
        # -- see CornerTraceDialog.show_corner's own comment for the same
        # change.
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
        # Ratio panels get a fixed y-range (see _apply_ratio_y_range);
        # only "stab"/"speed" still auto-range.
        for key in ("stab", "speed"):
            self.plots[key].enableAutoRange(axis='y')
        self._apply_ratio_y_range()

        self._add_corner_bands(corners_by_id, worst_colour_by_id)

        # LAP TRACE EXPORT item: cache for _on_export_lap_figure_clicked,
        # same "populated at the end of every render" convention as
        # CornerTraceDialog's own self._export_data.
        self._export_data = {
            "lap_label": f"Lap {lap_number}", "laps": export_laps,
            "thresholds": {
                "stab": cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"],
                "strong_csf": cls_cfg["STRONG_CSF"]["value"], "moderate_csf": cls_cfg["MODERATE_CSF"]["value"],
                "strong_csr": cls_cfg["STRONG_CSR"]["value"], "moderate_csr": cls_cfg["MODERATE_CSR"]["value"],
            },
            "corner_bands": list(self._corner_bands),
        }

        self.header_label.setText(
            f"Lap {lap_number} -- full-lap trace, {len(valid_lap_numbers)} valid lap(s) "
            f"available, fastest {n_default} (incl. lap {lap_number}) shown by default -- "
            f"check other laps to overlay."
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
