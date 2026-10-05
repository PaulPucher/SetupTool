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


# Near-strike grading (kerb display package, 2026-10-05). The mark comes from
# estimate_cornering_stiffness (near_strike_f / near_strike_r: CS samples whose
# regression window reaches the kerb smear zone). Drawing only -- the samples
# stay in every computed result.
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
        raise ValueError(f"near_strike_display {mode!r} not one of {NEAR_STRIKE_MODES}")
    mark = np.asarray(mark, dtype=bool) & np.isfinite(y)
    main = np.where(mark, np.nan, y)
    if mode == "exclude" or not mark.any():
        return main, None
    grow = mark.copy()
    grow[1:] |= mark[:-1]
    grow[:-1] |= mark[1:]
    overlay = np.where(grow & np.isfinite(y), y, np.nan)
    return main, overlay


def near_strike_caption(mode, n_marked, n_finite, grade_s):
    """One caption sentence for a CS plot, or None when nothing is graded."""
    if mode == "off" or n_marked == 0:
        return None
    width = f"{grade_s:g} s"
    if mode == "grey":
        return (f"Grey: samples whose estimation window lies within {width} of a masked kerb strike "
                "(strike energy smeared by the 2 Hz filter); marked lower-quality, retained in all "
                "computed results.")
    share = 100.0 * n_marked / max(1, n_finite)
    return (f"Samples whose estimation window lies within {width} of a masked kerb strike (strike energy "
            f"smeared by the 2 Hz filter) are omitted from this plot (n = {n_marked}, {share:.1f} %); "
            "retained in all computed results.")
