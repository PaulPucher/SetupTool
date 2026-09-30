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
