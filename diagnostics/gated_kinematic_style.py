# Fixed series identity for every gated-kinematic figure: one colour + one
# secondary encoding (linestyle/marker) per source, same in every plot, so a
# source never changes colour between figures and never relies on colour
# alone. Colours are the dataviz reference palette's categorical slots 1-7
# in their fixed order.

SERIES = {
    "kinematic_prod":        {"color": "#2a78d6", "ls": "-",  "marker": "o", "label": "kinematic, 0.05 Hz filtfilt (production)"},
    "ekf_auto_pacejka":      {"color": "#eb6834", "ls": "--", "marker": "s", "label": "ekf_auto_pacejka"},
    "gated_x1":              {"color": "#1baf7a", "ls": "-",  "marker": "D", "label": "gated, Otsu threshold"},
    "gated_x0.75":           {"color": "#eda100", "ls": ":",  "marker": "v", "label": "gated, 0.75 x Otsu"},
    "gated_x1.5":            {"color": "#e87ba4", "ls": "-.", "marker": "^", "label": "gated, 1.5 x Otsu"},
    "causal_washout_0.05hz": {"color": "#008300", "ls": "--", "marker": "x", "label": "causal washout 0.05 Hz"},
    "causal_washout_0.02hz": {"color": "#4a3aa7", "ls": ":",  "marker": "+", "label": "causal washout 0.02 Hz"},
}
TEXT = "#0b0b0b"
TEXT_MUTED = "#52514e"
GRID = "#e4e3df"


def style_axes(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(TEXT_MUTED)
    ax.tick_params(colors=TEXT_MUTED, labelsize=8)
