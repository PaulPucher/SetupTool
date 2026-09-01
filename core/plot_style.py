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
#
# Part B redesign (2026-09-01, trace-dialog work package + addendum):
# lap identity is now carried by COLOUR (lap_styles below), assigned
# dynamically to the CHECKED set in ascending lap order -- axle/quantity
# identity is instead carried by PANEL (one quantity per panel; see
# ui/views/corner_trace_dialog.py's PANEL_TITLES and core/figure_render.py's
# per-axle panel functions). CSF_COLOR/CSR_COLOR/etc. below are RETIRED as
# curve colours but kept as WINDOW_F_COLOR/WINDOW_R_COLOR's old value is
# gone too -- axle identity on the track map and tyre-curve panels is now
# conveyed by which panel/title, not a dedicated hue.

from ui.style import NEUTRAL, TEXT, TEXT_MUTED

# Lap colour palette -- addendum item 3: "10 vibrant distinguishable
# colours; beyond 10, cycle through the palette with an alternating dash
# pattern". This is matplotlib's own built-in "tab10" categorical palette
# (Vega/D3 category10 lineage) verbatim -- a long-established, widely
# recognised qualitative set, not an invented one; using it needs no new
# dependency (hardcoded here since this module must stay import-free of
# matplotlib/pyqtgraph, see the module docstring) and reads reasonably on
# both the dark INTERACTIVE and light PRINT backgrounds below (mid
# lightness throughout, no near-white/near-black entries).
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

# Addendum item 3: beyond len(LAP_PALETTE) checked laps, colours repeat
# and the dash pattern alternates so no two laps ever share BOTH colour
# and style. "dash" here is a style KEY, not a Qt/matplotlib enum value --
# each caller translates it to its own pen/linestyle vocabulary (see
# ui/views/corner_trace_dialog.py's _qt_dash_style / core/figure_render.py's
# _mpl_linestyle).
LAP_DASH_PATTERNS = ("solid", "dash")


def lap_styles(lap_numbers):
    """Assign a fixed {"color": hex, "dash": "solid"|"dash"} to each of
    `lap_numbers` in ASCENDING order. Pure function of the (unordered) set
    passed in -- the same checked set always yields the same colours,
    calling code re-invokes this every time the checked set changes rather
    than caching a lap-number-to-colour mapping across renders (addendum
    item 1: "colour is not a fixed property of the lap number").
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


# Reference/fitted/annotation colours -- NOT lap-identified, so they keep
# their own fixed hues regardless of the lap-colour redesign above.
FITTED_CURVE_COLOR = "#E040FB"   # EKF auto-fit model curve (Dugoff/Pacejka)

# Line widths -- screen (pyqtgraph, pixels) and print (matplotlib, points)
# are DIFFERENT units on DIFFERENT physical media (a monitor vs a 300 DPI
# page). Corrections batch (2026-09-01, post-Part-B visual review): the
# emphasised/faint (bold-lap) distinction is REMOVED entirely -- every
# checked lap's trace line is now the SAME width, distinguished only by
# colour (and, past 10 checked laps, dash pattern). One width per medium.
SCREEN_LAP_WIDTH = 1.5
PRINT_LAP_WIDTH = 1.0

# Tyre-curve panel readability re-tune (corrections batch, item 5) --
# session cloud very light/thin and drawn first (background), lap samples
# medium filled at partial alpha, estimation-window rings largest and
# drawn last (on top). Screen (pyqtgraph symbolSize, px diameter) and
# print (matplotlib scatter `s`, points^2 area) are set independently --
# the two APIs scale markers differently, so a literal px number does not
# translate 1:1 into an `s` value.
SESSION_CLOUD_COLOR_SCREEN = "#3A3A3A"
SESSION_CLOUD_COLOR_PRINT = "#DDDDDD"
SESSION_CLOUD_SIZE_SCREEN = 1
SESSION_CLOUD_SIZE_PRINT = 2
# Corrections round 3, item 3: "Markers 1.5 px, alpha 0.5" -- re-tuned again
# now that the tyre-curve axis range fits the corner samples themselves
# (render_corner_figure) rather than the much wider session cloud, so the
# same marker reads noticeably larger on screen at the new tighter zoom;
# shrunk to match. PRINT value keeps the prior ~3.5x screen-px-to-
# matplotlib-s scale factor (2px->7 before this round) so the two media
# stay visually proportionate to each other.
LAP_SAMPLE_SIZE_SCREEN = 1.5
LAP_SAMPLE_SIZE_PRINT = 5
LAP_SAMPLE_ALPHA = 0.5           # 0..1 -- pyqtgraph callers scale by 255
WINDOW_RING_SIZE_SCREEN = 3.5
WINDOW_RING_SIZE_PRINT = 12
# Fitted model / tangent / linear-reference line width, PRINT only (the
# corrections batch specified these three in points explicitly -- no
# screen-width instruction was given, so the interactive tab's own
# existing widths for these three lines are left as they were; flagged in
# the session report as an open item).
TYRE_LINE_WIDTH_PRINT = 0.8

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

# Threshold-line colour -- Part B redesign: ALL threshold lines (stability,
# front CS, rear CS) are now one shared neutral grey, distinguished from
# each other by LINE STYLE alone (strong=dashed, moderate=dotted,
# unstable=dash-dot), not colour -- axle identity no longer needs a
# threshold colour of its own now that front/rear CS are separate panels
# (see PANEL_TITLES), so a colour clash between an axle's threshold and
# another axle's lap-coloured curve can no longer happen. One mid-grey
# value (not theme["text_muted"]) chosen for legibility against BOTH the
# dark INTERACTIVE (#1e1e1e) and light PRINT (#ffffff) backgrounds above.
THRESHOLD_GREY = "#8A8A8A"

# Track-map-specific annotation colours (not lap-identified data).
TRACK_BG_COLOR = NEUTRAL       # faint whole-lap trace (context outline, not lap-coloured -- see A1)
WINDOW_RING_COLOR = "#000000"  # estimation-window hollow ring -- fixed black, not lap-identified (Part B spec)

# Bug fix, Part B: the tangent line ("tangent black dashed thin" per the
# work order) is now drawn with the ACTIVE THEME's own text colour
# (theme["text"]) at the call site, not a fixed constant -- the previous
# TANGENT_COLOR = TEXT (ui.style's light INTERACTIVE-theme text, "#e0e0e0")
# was used unconditionally in BOTH themes, so a PRINT (white background)
# export's tangent line rendered as low-contrast light grey on white
# instead of the intended black. theme["text"] already resolves to
# "#000000" for PRINT and TEXT for INTERACTIVE, matching "black on print,
# legible-on-dark on screen" with no separate constant needed.

# PRINT-only export geometry -- fixed so every exported figure is
# reproducible byte-for-byte from the same analysis result: same size,
# same dpi, same font, regardless of screen/window state at export time.
PRINT_DPI = 300
PRINT_WIDTH_CM = 16.0          # A4 text width
# Corrections round 3, item 2: both export figures share one 24cm cap now
# (verdict traces raised from its prior 20cm) -- verified by rendering
# that six stacked panels, each now with its own side legend column
# rather than an inside-axes one, stay comfortably readable at 24cm; see
# render_verdict_traces_figure's own docstring for the pre-specified
# two-figure split kept on file in case a future change needs it.
PRINT_HEIGHT_CM_CORNER = 24.0  # render_corner_figure (4-row composition: speed/CSf/CSr/tyre curves)
PRINT_HEIGHT_CM_VERDICT = 24.0  # render_verdict_traces_figure (6-row stack, Part B)
PRINT_FONT_FAMILY = "DejaVu Sans"  # matplotlib's own default -- always present, no font-install dependency
PRINT_FONT_SIZE_PT = 8
