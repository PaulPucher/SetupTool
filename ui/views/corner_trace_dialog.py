# Per-corner trace window (PLAN.md, Tier C UI work package "PART C" +
# usability follow-up "Task 2"). Plots stability_observed / CS_ratio_f,r /
# speed against track position (s_m) over one stable corner's phase
# bracket plus a config-resident approach/coast-out margin, one curve per
# valid lap. Pure display: every array plotted here already exists in
# state/cs/stab (Modules 1-5 output cached on the form after a live
# Analyse) -- this window performs no estimation, threshold derivation,
# or masking of its own.

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

PHASE_ORDER = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _phase_slice(t, start_t, end_t):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    return slice(lo, hi)


def _extend_slice_with_margin(t, s_m, lap_start_t, lap_end_t,
                               brake_start_t, exit_end_t,
                               margin_before_m, margin_after_m):
    """Widen the entry_1_brake-to-exit_5 slice by the configured margin on
    each side, clamped to this lap's own s_m extent (never reaches into a
    neighbouring lap -- s_m resets each lap, so clamping at this lap's own
    first/last sample is the correct bound, not an arbitrary safety pad).
    All interpolation stays lap-local (t/s_m sub-arrays only) -- the same
    discipline as the reset-guarded s_m interpolation elsewhere in this
    codebase, never interpolating across a lap-boundary discontinuity.
    Returns (slice, start_s, end_s) or (slice(0, 0), None, None) if this
    lap contributes no samples at all.
    """
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0), None, None
    lap_t = t[lo:hi]
    lap_s = s_m[lo:hi]

    # lap_end_t (from parsed_data["laps"], the lap_number channel's own
    # transition point) does not always land exactly on lap_distance's own
    # reset instant -- the two are independent channels/sample timings.
    # Found on real Dubai data: 3 of this corner's 4 lap windows carried a
    # handful of already-reset (near-zero) trailing samples inside their
    # nominal [lap_start_t, lap_end_t) window, which corrupted BOTH the s-
    # bound clamp below (their tiny values are finite, not NaN, so an
    # earlier finite-only guard didn't catch them) and searchsorted's own
    # sortedness precondition (a late drop back to ~0 breaks monotonicity).
    # Trim to the last index before any such drop -- a genuine same-lap
    # sample never falls this far below its own running maximum.
    RESET_DROP_M = 50.0
    finite = np.isfinite(lap_s)
    running_max = np.maximum.accumulate(np.where(finite, lap_s, -np.inf))
    reset_mask = finite & (running_max - lap_s > RESET_DROP_M)
    if reset_mask.any():
        cut = int(np.argmax(reset_mask))
        lap_t = lap_t[:cut]
        lap_s = lap_s[:cut]
        hi = lo + cut

    finite_idx = np.flatnonzero(np.isfinite(lap_s))
    if len(finite_idx) == 0:
        return slice(0, 0), None, None
    # The reset-guarded s_m is also deliberately NaN at samples immediately
    # adjacent to a lap-boundary reset (modules/stability_analysis.py
    # _interp_lap_distance_guarded) -- clamp against the first/last FINITE
    # sample, not lap_s[0]/lap_s[-1] directly: Python's own min(nan, x)/
    # max(nan, x) silently returns nan when the NaN operand comes first.
    lap_s_lo = float(lap_s[finite_idx[0]])
    lap_s_hi = float(lap_s[finite_idx[-1]])
    brake_start_s = float(np.interp(brake_start_t, lap_t, lap_s))
    exit_end_s = float(np.interp(exit_end_t, lap_t, lap_s))
    target_start_s = max(lap_s_lo, brake_start_s - margin_before_m)
    target_end_s = min(lap_s_hi, exit_end_s + margin_after_m)
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


class CornerTraceDialog(QDialog):
    """Reusable, non-modal per-corner trace window. One instance lives on
    the form (created lazily); opening it for a different corner replots
    in place rather than creating a new window, same pattern as the
    corner map's clear()-and-redraw (outing_form.py
    _update_corner_map_trace).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph as pg

        self.setWindowTitle("Corner Trace")
        self.resize(880, 680)
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

        # Task 2b: per-lap visibility checkboxes, built fresh per corner
        # (the valid-lap set differs corner to corner) -- outside the plot
        # area, not a floating in-plot legend.
        self.lap_checkbox_container = self._build_lap_checkbox_container()
        layout.addWidget(self.lap_checkbox_container)

        self.pg_layout = pg.GraphicsLayoutWidget()
        self.pg_layout.setBackground(PANEL)
        layout.addWidget(self.pg_layout)

        # Rebuilt in full by show_corner() every call (only mentions kerb/
        # not-moving bands when this corner's plotted range actually has
        # one) -- this is just the pre-first-click placeholder.
        self.legend_label = QLabel(f"CSf ({CSF_COLOR}) / CSr ({CSR_COLOR}).")
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
        # sections elsewhere in outing_form.py).
        event.ignore()
        self.hide()

    def _rebuild_lap_checkboxes(self, instances, selected_lap):
        while self.lap_checkbox_layout.count():
            item = self.lap_checkbox_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self.lap_visible = {}
        for c in instances:
            lap_num = c["lap_number"]
            is_quiet = "canonical_quiet" in c.get("warnings", [])
            label = f"Lap {lap_num}" + (" (quiet)" if is_quiet else "")
            if lap_num == selected_lap:
                label += " *"
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
            cb.toggled.connect(lambda checked, ln=lap_num: self._on_lap_visibility_toggled(ln, checked))
            self.lap_checkbox_layout.addWidget(cb)
            self.lap_visible[lap_num] = True
        self.lap_checkbox_layout.addStretch()

    def _on_lap_visibility_toggled(self, lap_number, checked):
        self.lap_visible[lap_number] = checked
        for item in self.lap_curve_items.get(lap_number, []):
            item.setVisible(checked)

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
            seg = c["segments"]
            sl, start_s, end_s = _extend_slice_with_margin(
                t, s_m, lap["start_time"], lap["end_time"],
                seg["entry_1_brake"][0], seg["exit_5"][1],
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

        legend_parts = [f"CSf ({CSF_COLOR}) / CSr ({CSR_COLOR})."]
        if any_kerb:
            legend_parts.append(
                "Grey band, dotted edge: kerb-masked span -- estimator infills from clean neighbourhood."
            )
        if any_not_moving:
            legend_parts.append(
                "Grey band, dash-dot edge: no valid signal -- car not at speed (outlap/pit portion)."
            )
        self.legend_label.setText(" ".join(legend_parts))
        self.legend_label.setVisible(True)

        self.show()
        self.raise_()
        self.activateWindow()

    def _pen(self, color, width, style, alpha=255):
        import pyqtgraph as pg
        qcolor = pg.mkColor(color)
        qcolor.setAlpha(alpha)
        return pg.mkPen(color=qcolor, width=width, style=style)

    def _add_threshold_line(self, panel_key, value, color, style=Qt.PenStyle.DashLine):
        import pyqtgraph as pg
        self.plots[panel_key].addLine(y=value, pen=pg.mkPen(color=color, width=1, style=style))
