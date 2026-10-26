# Text contrast of the theme (UI-1 item 3): every text colour reaches the
# WCAG AA ratio 4.5:1 on every background it is drawn on. Relative luminance
# and ratio as defined by WCAG 2.x.

import pytest

from ui import style

AA = 4.5


def _luminance(hex_colour):
    h = hex_colour.lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    rgb = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    lin = [c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb]
    return 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2]


def contrast(a, b):
    hi, lo = sorted((_luminance(a), _luminance(b)), reverse=True)
    return (hi + 0.05) / (lo + 0.05)


BACKGROUNDS = (style.BACKGROUND, style.PANEL, style.PANEL_ALT, style.BORDER)


@pytest.mark.parametrize("text", [style.TEXT, style.TEXT_MUTED, style.TEXT_DIM, style.PLOT_FOREGROUND])
def test_text_on_every_background(text):
    for bg in BACKGROUNDS:
        assert contrast(text, bg) >= AA, (text, bg, round(contrast(text, bg), 2))


def test_text_on_verdict_fills():
    for fill, text in style.FILL_TEXT.items():
        assert contrast(fill, text) >= AA, (fill, text, round(contrast(fill, text), 2))


def test_speed_badges():
    for cls, tint in style.SPEED_TINT.items():
        assert contrast(tint, style.SPEED_TEXT[cls]) >= AA, cls
