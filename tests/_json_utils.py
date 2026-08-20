# Shared helpers for the golden-file regression tests (Phase 1). Not a
# test module itself -- imported by tests/test_golden_pipeline.py.
#
# REGRESSION, NOT CORRECTNESS: comparing against a golden snapshot only
# tells you whether output CHANGED since the snapshot was taken, never
# whether it is right. See tests/test_golden_pipeline.py's own docstring.

import math

import numpy as np

# Tolerance rationale: the pipeline's own numeric range spans small slip
# angles (~1e-3 to 1e-1 rad) up to large forces/moments (~1e3-1e4 N/Nm),
# so a single absolute tolerance would be either too loose for small
# values or too tight for large ones. Using the same combined
# relative+absolute form as numpy.isclose/math.isclose:
#   abs(a - b) <= atol + rtol * abs(b)
# rtol=1e-6 is loose enough to absorb floating-point reassociation from a
# future numpy/scipy version bump (not a logic change), but far tighter
# than any real logic change in this codebase has ever produced (the
# smallest genuine estimator change on record, WP-B's steering-ratio
# upgrade, moved downstream statistics by 1e-3 to 1e-2 relative -- see
# thesis_notes.md "WP-N2 Step 1b" verification section). atol=1e-9
# handles comparisons near zero, where rtol alone would demand
# unreasonable precision.
RTOL = 1e-6
ATOL = 1e-9


def _is_nan(x):
    return isinstance(x, float) and math.isnan(x)


def floats_close(a, b, rtol=RTOL, atol=ATOL):
    """NaN == NaN passes (both are the pipeline's "not computed" sentinel,
    not an error state); NaN vs a real number fails; everything else uses
    the combined relative+absolute tolerance above."""
    if _is_nan(a) and _is_nan(b):
        return True
    if _is_nan(a) or _is_nan(b):
        return False
    return abs(a - b) <= atol + rtol * abs(b)


def diff_json(a, b, path=""):
    """Recursively compare two JSON-shaped structures (dict/list/str/int/
    float/bool/None -- the shape summarise_corners and generate_
    recommendations both produce). Returns a list of (path, got, expected)
    mismatches; empty list means "matches within tolerance". Numpy scalar
    types are coerced to native Python via float()/int() before
    comparison so a golden file loaded from JSON (always native types)
    compares cleanly against freshly-computed numpy-derived values.
    """
    diffs = []

    if isinstance(a, (np.floating,)):
        a = float(a)
    if isinstance(b, (np.floating,)):
        b = float(b)
    if isinstance(a, (np.integer,)):
        a = int(a)
    if isinstance(b, (np.integer,)):
        b = int(b)

    if isinstance(a, dict) and isinstance(b, dict):
        keys = set(a.keys()) | set(b.keys())
        for k in sorted(keys, key=str):
            if k not in a:
                diffs.append((f"{path}.{k}", "<missing>", b[k]))
            elif k not in b:
                diffs.append((f"{path}.{k}", a[k], "<missing>"))
            else:
                diffs.extend(diff_json(a[k], b[k], f"{path}.{k}"))
        return diffs

    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            diffs.append((f"{path}[len]", len(a), len(b)))
            return diffs
        for i, (av, bv) in enumerate(zip(a, b)):
            diffs.extend(diff_json(av, bv, f"{path}[{i}]"))
        return diffs

    if isinstance(a, bool) or isinstance(b, bool):
        # bool is a subclass of int in Python -- must be checked before
        # the float branch below, or True/False would compare as 1/0.
        if a != b:
            diffs.append((path, a, b))
        return diffs

    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if not floats_close(float(a), float(b)):
            diffs.append((path, a, b))
        return diffs

    if a != b:
        diffs.append((path, a, b))
    return diffs
