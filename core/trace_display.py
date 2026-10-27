# Display-only helpers for the trace windows. Nothing here is imported by
# modules/ or by any analysis path; the arrays it returns exist only to be
# drawn.

import numpy as np


def kerb_gap_bridge(x, y, kerb):
    """Linear bridge across kerb gaps, for drawing only.

    A gap is a run of non-finite y bounded by finite samples on both sides
    that contains at least one kerb-flagged sample -- the estimators mask
    kerb samples out, so these are the holes a kerb strike leaves. Returns a
    NEW array, NaN everywhere except the bridged stretch and its two finite
    anchors (so the segment joins the plotted trace); y itself is never
    modified. Gaps without a kerb sample (window cap, not moving) stay gaps:
    those are missing estimates, not masked ones.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    out = np.full(y.shape, np.nan)
    if kerb is None or y.size < 3:
        return out
    kerb = np.asarray(kerb, dtype=bool)
    bad = ~np.isfinite(y)
    edges = np.diff(np.concatenate(([0], bad.astype(np.int8), [0])))
    for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        if a == 0 or b >= y.size or not kerb[a:b].any():
            continue
        lo, hi = a - 1, b
        if not (np.isfinite(x[lo]) and np.isfinite(x[hi])) or x[hi] == x[lo]:
            continue
        seg = slice(lo, hi + 1)
        out[seg] = np.interp(x[seg], [x[lo], x[hi]], [y[lo], y[hi]])
    return out


# Kerb exclusion display. The kerb-affected mark is
# estimate_cornering_stiffness' near_strike_f / near_strike_r (internal name:
# CS samples whose regression window holds a kerb-mask or smear-zone sample).
# This module only draws; the verdict statistics drop marked samples in
# summarise_corners, and the CS arrays keep every sample.
NEAR_STRIKE_MODES = ("grey", "exclude", "off")


def near_strike_split(y, mark, mode):
    """(main, overlay) arrays for drawing a CS trace under the display mode.

    grey: main = y without the marked samples, overlay = the marked samples
    plus their finite neighbours (so each grey stretch joins the trace and a
    lone marked sample still draws as a segment); exclude: main as for grey,
    overlay None; off or no mark: (y, None). y itself is never modified.
    """
    y = np.asarray(y, dtype=float)
    if mode == "off" or mark is None:
        return y, None
    if mode not in NEAR_STRIKE_MODES:
        raise ValueError(f"kerb_exclusion_display {mode!r} not one of {NEAR_STRIKE_MODES}")
    mark = np.asarray(mark, dtype=bool) & np.isfinite(y)
    main = np.where(mark, np.nan, y)
    if mode == "exclude" or not mark.any():
        return main, None
    grow = mark.copy()
    grow[1:] |= mark[:-1]
    grow[:-1] |= mark[1:]
    overlay = np.where(grow & np.isfinite(y), y, np.nan)
    return main, overlay


# Interval helpers for the dialog's shading: per panel only one tint family
# and one exclusion grey, never stacked, so the grey is drawn once as the
# union over the checked laps and the tint is clipped where it lies.

def interval_union(intervals):
    """Sorted, merged (lo, hi) intervals."""
    out = []
    for lo, hi in sorted((min(a, b), max(a, b)) for a, b in intervals):
        if out and lo <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return out


def interval_subtract(interval, holes):
    """Parts of interval (lo, hi) not covered by the merged holes."""
    lo, hi = interval
    parts, cur = [], lo
    for a, b in holes:
        if b <= cur or a >= hi:
            continue
        if a > cur:
            parts.append((cur, a))
        cur = max(cur, b)
    if cur < hi:
        parts.append((cur, hi))
    return parts


def cross_lap_envelope(laps_xy, cfg):
    """(grid, p_lo, p_hi) of several laps' (s, y) traces on a common s-grid,
    NaN where fewer than cfg["min_laps"] laps have a value, or None when no
    lap has data. cfg = corner_trace_display.cs_envelope. Drawing only."""
    grid_m = cfg["grid_m"]
    pct_lo, pct_hi = cfg["percentiles"]
    rows = []
    for x, y in laps_xy:
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        ok = np.isfinite(x) & np.isfinite(y)
        if ok.sum() >= 2:
            order = np.argsort(x[ok])
            rows.append((x[ok][order], y[ok][order]))
    if not rows:
        return None
    lo = min(xs[0] for xs, _ys in rows)
    hi = max(xs[-1] for xs, _ys in rows)
    if hi <= lo:
        return None
    grid = np.arange(lo, hi + grid_m, grid_m)
    m = np.full((len(rows), grid.size), np.nan)
    for k, (xs, ys) in enumerate(rows):
        row = np.interp(grid, xs, ys, left=np.nan, right=np.nan)
        # no bridging across holes (kerb, not moving): the two samples around a
        # grid point must lie within two grid steps of each other
        j = np.clip(np.searchsorted(xs, grid), 1, xs.size - 1)
        row[(xs[j] - xs[j - 1]) > 2 * grid_m] = np.nan
        m[k] = row
    n = np.sum(np.isfinite(m), axis=0)
    p_lo = np.full(grid.size, np.nan)
    p_hi = np.full(grid.size, np.nan)
    enough = n >= cfg["min_laps"]
    if enough.any():
        p_lo[enough] = np.nanpercentile(m[:, enough], pct_lo, axis=0)
        p_hi[enough] = np.nanpercentile(m[:, enough], pct_hi, axis=0)
    return grid, p_lo, p_hi


def kerb_scatter_split(valid, kerb, mark, mode):
    """(filled, hollow) sample selections for a tyre-curve scatter under the
    kerb-exclusion switch. Kerb-affected = inside the kerb mask or carrying the
    axle's grade mark -- the same definition the CS traces grey; grey draws them
    hollow, exclude leaves them out, off draws them like every other sample."""
    valid = np.asarray(valid, dtype=bool)
    affected = np.zeros_like(valid)
    for m in (kerb, mark):
        if m is not None:
            affected |= np.asarray(m, dtype=bool)
    if mode == "off":
        return valid, np.zeros_like(valid)
    if mode not in NEAR_STRIKE_MODES:
        raise ValueError(f"kerb_exclusion_display {mode!r} not one of {NEAR_STRIKE_MODES}")
    hollow = valid & affected if mode == "grey" else np.zeros_like(valid)
    return valid & ~affected, hollow


KERB_CAPTION = "kerb-affected samples excluded from CS verdict statistics, retained in CS arrays"


def kerb_exclusion_caption(mode, n_affected, n_finite):
    """The one kerb-exclusion caption, prefixed by
    what this plot does with the samples and always carrying the count; the
    0.2 s reach of the mark is methods-text detail, not caption."""
    share = 100.0 * n_affected / max(1, n_finite)
    prefix = {"grey": "Grey", "exclude": "Not drawn", "off": "Drawn normally"}[mode]
    return f"{prefix}: {KERB_CAPTION} (n = {n_affected}, {share:.1f} %)."


# CS lines per ratio panel, shared by the trace window and its export so the
# two draw the same set. Values from modules.classifier.cs_decision_lines;
# the kind picks the line style: boundary solid (decides the symptom),
# strong dash-dot (grades front depth), info dotted and weaker (rear note
# only, no symptom). Same set as the thesis figure fig_example_corner.
CS_LINE_SPECS = {
    "cs_f": (("boundary_f", "boundary", "Decision boundary"),
             ("strong_f", "strong", "Strong tier (front)")),
    "cs_r": (("boundary_r", "boundary", "Decision boundary"),
             ("info_r", "info", "Rear info tier (no symptom)")),
}


def cs_lines(panel, values):
    """[(value, kind, label)] of one ratio panel; values as returned by
    classifier.cs_decision_lines (or the export's thresholds dict)."""
    return [(values[key], kind, label) for key, kind, label in CS_LINE_SPECS[panel]]
