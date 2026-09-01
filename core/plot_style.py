# Shared plot styling for the corner trace dialog (pyqtgraph, interactive)
# and exported/diagnostic figures (matplotlib, static) -- one source of
# truth for per-quantity colour, so a curve means the same colour whether
# it is on screen or in a thesis figure. Tier C (UI/product), no science
# content: nothing here is a measurement, threshold, or method choice.
#
# No PyQt6/pyqtgraph/matplotlib import -- this module is plain constants
# and small dicts, safe to import from core/, ui/, and a headless
# diagnostics script alike (verified: ui/style.py itself has zero imports,
# so importing colours from it here carries no Qt dependency forward).

from ui.style import ACCENT, BAD, NEUTRAL, TEXT, TEXT_MUTED

# Per-signal curve colours, fixed regardless of theme -- a lap is
# distinguished by line width/style/opacity, not colour (see WIDTH/ALPHA
# below). Literal hex, same convention ui/views/corner_trace_dialog.py
# already used before this module existed: reserved for continuous data-
# curve identity, distinct from ui.style's verdict/chrome palette.
CSF_COLOR = "#4FC3F7"
CSR_COLOR = "#FFB74D"
STAB_COLOR = "#B39DDB"
SPEED_COLOR = "#C0A060"
LSF_COLOR = "#81C784"
LSR_COLOR = "#F06292"
FITTED_CURVE_COLOR = "#E040FB"

# Track-map window highlights reuse the axle's own curve colour (CSF/CSR)
# so the map keys into the tyre-curve panels beside it without a third
# colour vocabulary. Corner bracket and tangent/reference lines get their
# own neutral, theme-aware colours (see THEME below) since they are
# annotation, not axle-identified data.
WINDOW_F_COLOR = CSF_COLOR
WINDOW_R_COLOR = CSR_COLOR

# Line widths / marker sizes / alpha, shared by both the interactive
# dialog and the exported figures -- one visual weight per role. Alpha is
# 0..255 for pyqtgraph, translated to 0..1 (divide by 255) for matplotlib.
SELECTED_WIDTH = 2.5
NORMAL_WIDTH = 1.5
TANGENT_WIDTH = 1.5
ALPHA_SELECTED = 255
ALPHA_FAINT = 90          # non-selected, still-visible laps -- out of 255
MARKER_SIZE_SESSION = 2   # background session scatter, tyre-curve/export
MARKER_SIZE_CORNER = 5
MARKER_SIZE_WINDOW = 7
LEGEND_FONT_PT = "11pt"   # pyqtgraph legend text (dialog only)

# Two themes: INTERACTIVE (this dialog's existing dark look) and PRINT
# (light background, black text -- thesis/export use). Data-curve hues
# above are identical in both; only background/text/muted/gridline swap.
# Matplotlib reads hex strings directly; pyqtgraph reads the same strings
# via pg.mkColor/pg.mkBrush, already how ui.style's constants are used
# elsewhere in this dialog.
INTERACTIVE = {
    "name": "interactive",
    "bg": "#1e1e1e",       # PANEL_ALT-adjacent, matches the dialog's own plot background
    "text": TEXT,
    "text_muted": TEXT_MUTED,
    "grid": TEXT_MUTED,
    "grid_alpha": 0.15,
}

PRINT = {
    "name": "print",
    "bg": "#ffffff",
    "text": "#000000",
    "text_muted": "#444444",
    "grid": "#999999",
    "grid_alpha": 0.3,
}

# Threshold-line colours (classification config values plot in these) --
# stability's single threshold is BAD (same red used for "unstable" the
# rest of the app), CS thresholds use the axle's own curve colour with
# dashed=strong / dotted=moderate, same convention the dialog already had.
THRESHOLD_STAB_COLOR = BAD
THRESHOLD_CSF_COLOR = CSF_COLOR
THRESHOLD_CSR_COLOR = CSR_COLOR

# Track-map-specific annotation colours (not axle-identified data).
TRACK_BG_COLOR = NEUTRAL       # faint whole-lap trace
CORNER_BRACKET_COLOR = ACCENT  # this corner's own phase bracket
TANGENT_COLOR = TEXT           # worst-phase tangent line, tyre-curve panel

# PRINT-only export geometry -- fixed so every exported figure is
# reproducible byte-for-byte from the same analysis result: same size,
# same dpi, same font, regardless of screen/window state at export time.
PRINT_DPI = 300
PRINT_WIDTH_CM = 16.0          # A4 text width
PRINT_HEIGHT_CM_CORNER = 20.0  # render_corner_figure (3-row chair composition)
PRINT_HEIGHT_CM_VERDICT = 14.0  # render_verdict_traces_figure (4-row stack)
PRINT_FONT_FAMILY = "DejaVu Sans"  # matplotlib's own default -- always present, no font-install dependency
PRINT_FONT_SIZE_PT = 8
