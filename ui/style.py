# Global stylesheet, applied once at startup.

# All colour literals live here.

# Base
BACKGROUND  = "#141414"
PANEL       = "#1a1a1a"
PANEL_ALT   = "#252525"
BORDER      = "#2a2a2a"

# Text. Every text colour reaches the WCAG AA ratio 4.5:1 on all four
# backgrounds above (UI-1 item 3, author 2026-10-24; checked in
# tests/test_style_contrast.py): TEXT_MUTED >= 7.5:1, TEXT_DIM >= 5.1:1.
# The earlier #888 / #555 fell to 4.3:1 / 2.5:1 on the darker panels.
TEXT        = "#e0e0e0"
TEXT_MUTED  = "#adadad"
TEXT_DIM    = "#959595"

# Accent
ACCENT      = "#C0A060"

# status colours (stability cards, validity flags)
OK          = "#4CAF50"   # healthy / stabilising
WARN        = "#C0A060"  # same hue as ACCENT on purpose
BAD         = "#c0392b"   # saturated / destabilising
NEUTRAL     = "#444"      # no data / NaN
INFO        = "#4A90D9"   # opposite-side marker to BAD (yaw panel: deficit flag)

# text on a filled cell or badge: dark on the light fills (green, amber),
# light on the dark ones (red, grey); TEXT itself reaches only 4.4:1 on BAD
TEXT_ON_LIGHT_FILL = "#111111"
TEXT_ON_DARK_FILL  = "#f2f2f2"
FILL_TEXT = {OK: TEXT_ON_LIGHT_FILL, WARN: TEXT_ON_LIGHT_FILL, BAD: TEXT_ON_DARK_FILL, NEUTRAL: TEXT_ON_DARK_FILL}

# speed class tints: neutral blue-greys, light -> dark with rising speed, so
# they never read as a verdict colour (author ruling 2026-10-23, 4g item 5)
SPEED_LOW    = "#B4C0CA"
SPEED_MEDIUM = "#7E8C98"
SPEED_HIGH   = "#4B5864"
SPEED_TINT = {"low": SPEED_LOW, "medium": SPEED_MEDIUM, "high": SPEED_HIGH}
# label text on the tint: dark on the light two, light on the dark one
SPEED_TEXT = {"low": PANEL, "medium": PANEL, "high": TEXT}
SPEED_LETTER = {"low": "L", "medium": "M", "high": "H"}

# pyqtgraph axes: tick labels and axis titles
PLOT_FOREGROUND = TEXT_MUTED

STYLESHEET = f"""
    QMainWindow, QWidget {{
        background-color: {BACKGROUND};
        color: {TEXT};
        font-family: system-ui;
        font-size: 13px;
    }}

    QLabel {{
        color: {TEXT};
    }}

    QCheckBox, QRadioButton {{
        color: {TEXT};
    }}

    QTabBar::tab {{
        background-color: {PANEL};
        color: {TEXT_MUTED};
        padding: 6px 14px;
        border: 1px solid {BORDER};
        border-bottom: none;
    }}

    QTabBar::tab:selected {{
        background-color: {PANEL_ALT};
        color: {TEXT};
    }}

    QPushButton {{
        background-color: {ACCENT};
        color: #1a1200;
        border: none;
        border-radius: 6px;
        padding: 6px 14px;
        font-weight: 600;
        font-size: 12px;
    }}

    QPushButton:hover {{
        background-color: #d4b472;
    }}

    QPushButton:pressed {{
        background-color: #a88c50;
    }}

    QListWidget {{
        background-color: {PANEL};
        border: none;
        border-right: 1px solid {BORDER};
        outline: 0;
        selection-background-color: {PANEL_ALT};
        selection-color: {ACCENT};
    }}

    QListWidget::item {{
        padding: 10px 8px;
        border-bottom: 1px solid #1e1e1e;
        color: {TEXT_MUTED};
    }}

    QListWidget::item:selected {{
        background-color: {PANEL_ALT};
        border-left: 2px solid {ACCENT};
    }}

    QListWidget::item:hover {{
        background-color: #1e1e1e;
        color: {TEXT_MUTED};
    }}

    QTableWidget {{
        background-color: {BACKGROUND};
        border: none;
        gridline-color: #1e1e1e;
    }}

    QTableWidget::item {{
        padding: 8px;
        border-bottom: 1px solid #1e1e1e;
        color: #d0d0d0;
    }}

    QTableWidget::item:selected {{
        background-color: #1e1e1e;
        color: {TEXT};
    }}

    QTableWidget::item:focus {{
        outline: 0;
        border: 0px;
        background-color: #1e1e1e;
        }}

    QTableWidget:focus {{
        outline: 0;
        border: none;
    }}

    QHeaderView::section {{
        background-color: {PANEL};
        color: {TEXT_DIM};
        font-size: 11px;
        padding: 8px;
        border: none;
        border-bottom: 1px solid #222;
        text-transform: uppercase;
    }}

    QScrollBar:vertical {{
        background: {BACKGROUND};
        width: 6px;
    }}

    QScrollBar::handle:vertical {{
        background: {BORDER};
        border-radius: 3px;
    }}
"""
