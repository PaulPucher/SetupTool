# Reproduces the pure-lateral floor of the CS ratio (limit_events
# PURE_LATERAL_FLOOR = -0.53): the steepest post-peak slope of the Magic
# Formula relative to its slope at the origin, at the largest shape factor
# searched by Goblirsch et al. (2025), C = 3, over E in [-1, 1].
# The normalised slope depends only on B * alpha, C and E (D and B scale
# numerator and denominator alike), so B = D = 1 and the abscissa is
# x = B * alpha. Per E: dense grid in x, then a bounded refinement around the
# grid minimum. E steps of 0.01 as stated in the thesis (Ch. 3).
# Run: python -m tools.pure_lateral_floor

import numpy as np
from scipy.optimize import minimize_scalar

from modules.tyre_model_pacejka import pacejka_lateral_stiffness

C = 3.0
E_GRID = np.round(np.arange(-1.0, 1.0 + 1e-9, 0.01), 2)
X_MAX = 50.0      # B * alpha; the minimum lies well inside (checked: argmin < 5)
X_STEP = 1e-3


def normalised_slope(x, e):
    # slope at the origin is B * C * D = C for B = D = 1
    return pacejka_lateral_stiffness(x, 1.0, C, 1.0, e) / C


def minimum_for(e):
    x = np.arange(0.0, X_MAX, X_STEP)
    s = normalised_slope(x, e)
    i = int(np.argmin(s))
    r = minimize_scalar(lambda v: float(normalised_slope(v, e)),
                        bounds=(max(0.0, x[i] - X_STEP), x[i] + X_STEP), method="bounded",
                        options={"xatol": 1e-10})
    return float(r.fun), float(r.x)


def main():
    rows = [(e, *minimum_for(e)) for e in E_GRID]
    e_min, s_min, x_min = min(rows, key=lambda r: r[1])
    worst_other = max(r[1] for r in rows)
    print(f"C = {C:g}, E in [{E_GRID[0]:g}, {E_GRID[-1]:g}] step 0.01 ({len(E_GRID)} values)")
    print(f"steepest normalised post-peak slope: {s_min:.5f} at E = {e_min:+.2f}, B*alpha = {x_min:.4f}")
    print(f"least steep of the per-E minima: {worst_other:.5f}")
    for e in (-1.0, 0.0, 1.0):
        s, x = next((r[1], r[2]) for r in rows if r[0] == e)
        print(f"  E = {e:+.0f}: {s:.5f} at B*alpha = {x:.4f}")
    print(f"rounded floor: {round(s_min, 2):.2f} (limit_events.PURE_LATERAL_FLOOR)")


if __name__ == "__main__":
    main()
