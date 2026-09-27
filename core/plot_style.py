# Shared plot styling for the trace dialog (pyqtgraph) and exported figures
# (matplotlib): a quantity has the same colour on screen and in print.
# Plain constants, no plotting imports.
# Lap identity = colour (lap_styles); quantity/axle identity = panel.

from ui.style import NEUTRAL, TEXT, TEXT_MUTED

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
FITTED_CURVE_COLOR = "#E040FB"   # EKF auto-fit model curve (Dugoff/Pacejka)

# line widths: screen in px, print in pt. One width for all laps.
SCREEN_LAP_WIDTH = 1.5
PRINT_LAP_WIDTH = 1.0

# tyre-curve markers: session cloud light and first, lap samples medium,
# window rings largest and last. Screen px and matplotlib `s` (pt^2)
# scale differently -> set separately.
SESSION_CLOUD_COLOR_SCREEN = "#3A3A3A"
SESSION_CLOUD_COLOR_PRINT = "#DDDDDD"
SESSION_CLOUD_SIZE_SCREEN = 1
SESSION_CLOUD_SIZE_PRINT = 2
# smaller now that the axis fits the corner samples; print ~3.5x screen px
LAP_SAMPLE_SIZE_SCREEN = 1.5
# print only: smaller again so rings and tangent stay readable over the
# marker cloud (screen can zoom)
LAP_SAMPLE_SIZE_PRINT = 3.3
LAP_SAMPLE_ALPHA = 0.4  # 0..1, pyqtgraph scales by 255
WINDOW_RING_SIZE_SCREEN = 3.5
WINDOW_RING_SIZE_PRINT = 12
# fitted / tangent / linear-reference widths, print only
TYRE_LINE_WIDTH_PRINT = 0.8

LEGEND_FONT_PT = "11pt"   # pyqtgraph legend text (dialog only)

# INTERACTIVE (dark) and PRINT (light) themes; data hues identical, only
# background/text/grid swap
INTERACTIVE = {
    "name": "interactive",
    "bg": "#1e1e1e",  # dialog plot background
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

# all threshold lines one mid grey (readable on both themes), told apart
# by style: strong dashed, moderate dotted, unstable dash-dot
THRESHOLD_GREY = "#8A8A8A"

# Track-map-specific annotation colours (not lap-identified data).
TRACK_BG_COLOR = NEUTRAL  # faint whole-lap context outline
WINDOW_RING_COLOR = "#000000"  # estimation-window ring, fixed black

# tangent line uses theme["text"] at the call site -- a fixed light TEXT
# colour vanished on the white print background

# fixed print geometry -> exports reproducible regardless of window state
PRINT_DPI = 300
PRINT_WIDTH_CM = 16.0          # A4 text width
# both figures 24 cm (six stacked panels with side legends still readable)
PRINT_HEIGHT_CM_CORNER = 24.0  # 4 rows: speed / CSf / CSr / tyre curves
PRINT_HEIGHT_CM_VERDICT = 24.0  # 6-row stack
PRINT_FONT_FAMILY = "DejaVu Sans"  # matplotlib default, always installed
PRINT_FONT_SIZE_PT = 8
