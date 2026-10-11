# Standard trace set for diagnostics figures: front and rear CS_ratio vs lap
# distance, one thin line per analysis lap, grey corner-bracket bands, y fixed
# to the app's own corner-trace CS panel range (config corner_trace_display).
# Factored from the value-lens trace layout (inspect_cs_value_lens.traces) so
# new packages call it instead of rebuilding; existing scripts keep their own
# copies. Plot-only: takes arrays, computes nothing.

import numpy as np

INK, BAND, REF = "#1f2937", "#e5e7eb", "#6b7280"


def cs_panel_range():
    from modules.stability_analysis import load_parameters
    cfg = load_parameters()["corner_trace_display"]
    return cfg["cs_ls_panel_y_min"], cfg["cs_ls_panel_y_max"]


def render_cs_traces(out_path, time, s_m, cs_f, cs_r, laps, corners, title, y_range=None, markers=None,
                     near_strike=None, near_strike_mode=None):
    """laps: dicts with lap_number/start_time/end_time/is_valid_for_analysis;
    corners: dicts with lap_number/bracket_start_m/bracket_end_m. markers:
    optional {"f"/"r": {"s": [...], "y": [...], "label": str, "color": str,
    "notes": [(s, y, text), ...]}} overlaid on the traces. near_strike:
    optional {"f"/"r": bool array} grade mark, drawn per near_strike_mode
    (default config corner_trace_display.kerb_exclusion_display) with one
    caption sentence. Returns the count of samples below the fixed y floor
    per axle (clipped from view)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from core.plot_style import NEAR_STRIKE_COLOR
    from core.trace_display import near_strike_split, kerb_exclusion_caption
    from modules.stability_analysis import load_parameters
    if near_strike is not None and near_strike_mode is None:
        near_strike_mode = load_parameters()["corner_trace_display"]["kerb_exclusion_display"]
    n_mark = n_fin = 0
    y_lo, y_hi = y_range or cs_panel_range()
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    an_nums = {l["lap_number"] for l in an}
    lap_len = np.nanmax(s_m[(time >= an[0]["start_time"]) & (time <= an[0]["end_time"])]) if an else 1.0
    brackets = sorted({(c["bracket_start_m"], c["bracket_end_m"]) for c in corners
                       if c.get("bracket_start_m") is not None and c["lap_number"] in an_nums})
    fig, axes = plt.subplots(2, 1, figsize=(max(14, lap_len / 400), 7), sharex=True, constrained_layout=True)
    clipped = {}
    for ax, arr, lab, key in ((axes[0], cs_f, "cornering stiffness ratio, front", "f"),
                              (axes[1], cs_r, "cornering stiffness ratio, rear", "r")):
        arr = np.asarray(arr, dtype=float)
        for b0, b1 in brackets:
            ax.axvspan(b0, b1, color=BAND, lw=0, zorder=0)
        ax.axhline(0.0, color=REF, lw=0.8, zorder=1)
        ax.axhline(1.0, color=REF, lw=0.8, ls="--", zorder=1)
        clipped[key] = 0
        for l in an:
            m = (time >= l["start_time"]) & (time <= l["end_time"])
            y = arr[m]
            clipped[key] += int(np.sum(np.isfinite(y) & (y < y_lo)))
            mark = None if near_strike is None else np.asarray(near_strike[key], bool)[m]
            main, overlay = near_strike_split(y, mark, near_strike_mode or "off")
            if mark is not None:
                n_fin += int(np.isfinite(y).sum())
                n_mark += int((mark & np.isfinite(y)).sum())
            ax.plot(s_m[m], main, color=INK, lw=0.5, alpha=0.6, zorder=2)
            if overlay is not None:
                ax.plot(s_m[m], overlay, color=NEAR_STRIKE_COLOR, lw=0.5, alpha=0.9, zorder=2)
        mk = (markers or {}).get(key)
        if mk:
            ax.plot(mk["s"], np.clip(mk["y"], y_lo, y_hi), "o", ms=2.2, mew=0, color=mk["color"], alpha=0.8,
                    zorder=3, label=mk["label"])
            for xs, ys, text in mk.get("notes", []):
                ax.annotate(text, (xs, max(ys, y_lo)), xytext=(0, -14), textcoords="offset points", ha="center",
                            fontsize=8, color=mk["color"], fontweight="bold", zorder=4)
            ax.legend(loc="lower left", frameon=False, fontsize=8)
        ax.set_ylim(y_lo, y_hi)
        ax.set_ylabel(lab, color=INK)
        ax.grid(axis="y", color=BAND, lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[1].set_xlabel("lap distance [m]", color=INK)
    fig.suptitle(f"{title} -- per analysis lap (n={len(an)}), grey bands = corner brackets, 0 = force peak, "
                 f"1 = linear; y fixed {y_lo}..{y_hi}", color=INK, fontsize=10)
    cap = kerb_exclusion_caption(near_strike_mode or "off", n_mark, n_fin) if near_strike else None
    if cap:
        fig.text(0.5, 0.002, cap, ha="center", va="bottom", fontsize=8, color=INK)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return clipped
