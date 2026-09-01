# Yaw-moment-stability estimator: raw yaw acceleration and the local
# s-anchored ridge regression for dMz/dbeta.
#
# Concept and target relation (Mz = Iz*psidd + D_psi*psid, dMz/dbeta sign
# convention): method anchor recorded in thesis_notes.md, "Yaw moment
# stability dMz/dbeta" entry. The estimator construction below (centred
# rolling-mean yaw acceleration; s-anchored Gaussian-weighted local ridge regression
# pooling samples across laps at the same track position) is after the
# chair performance_analysis tooling (internal), not part of Werner's
# method -- the reference implementation is read-only in
# docs/literature/, never imported from.

import numpy as np
import pandas as pd

# Gaussian kernel sigma = window_m / GAUSS_SIGMA_DIVISOR. Fixes the
# weighting SHAPE (how fast influence decays inside the window), not a
# per-track calibration, so it stays a named constant (CLAUDE.md
# method-defining-constant rule) rather than a config entry.
GAUSS_SIGMA_DIVISOR = 2.5

_PREDICTOR_COLUMNS = ("beta_rad", "delta_f_rad", "v_mps", "ax_mps2", "az_mps2")


def calculate_filtered_yaw_acceleration(yaw_rate_radps, time_s, sample_rate_hz, window_s):
    """Differentiate yaw rate, then a centred rolling mean (chair-exact
    window/min_periods construction, raw-yaw-rate path only -- the
    chair's function also accepts a pre-smoothed yaw-rate input from a
    chair-external filter list that is outside this reference's scope,
    so only the raw-signal differentiation path is reproduced here).
    [forced adaptation]
    """
    window = max(5, int(round(window_s * sample_rate_hz)))
    if window % 2 == 0:
        window += 1
    min_periods = max(2, window // 3)

    yaw_accel_raw = np.gradient(yaw_rate_radps, time_s)
    yaw_accel = (
        pd.Series(yaw_accel_raw)
        .rolling(window=window, center=True, min_periods=min_periods)
        .mean()
        .bfill()
        .ffill()
        .to_numpy(dtype=float)
    )
    return yaw_accel


def calculate_observed_stability(
    s_m, beta_rad, delta_f_rad, v_mps, ax_mps2, az_mps2, mz_inertial_Nm,
    valid_mask, grid_step_m, window_m, min_samples, ridge, min_beta_std_rad,
):
    """Compute local weighted-ridge dMz/dbeta [Nm/deg] in track-distance space
    (after the chair performance_analysis tooling, internal).

    s_m is SetupTool's lap_distance channel converted to metres --
    within-lap track position, resetting every lap. Sorting samples by
    s_m interleaves every lap's pass through the same corner, so the
    local window at a given s pools samples across laps rather than
    depending on one lap's own excitation (this is the chair's own
    semantics for s_m; SetupTool interpolates it onto a common sample
    timeline where the chair receives it natively -- see
    stability_analysis.py for the lap-reset interpolation guard this
    requires, a channel-alignment necessity, not a method change).
    [neutral engineering]

    az_mps2 is an OPTIONAL regressor (chair-identical): pass None to
    drop it from the set (beta + delta_f + v + ax used instead) rather
    than invalidating the estimate when the channel is unavailable.

    valid_mask marks samples to KEEP; excluded samples (moving/kerb/
    in-out-lap gates -- SetupTool call-site adaptations, see
    stability_analysis.py) are NaN'd before the internal dropna step,
    identical to how the chair estimator ignores missing data. The
    estimator itself carries no knowledge of what was excluded or why.

    Returns (stability_Nm_per_deg, valid, diagnostics) -- diagnostics is
    reporting-only (method string, grid coverage, per-grid-point skip
    reasons, per-window sample counts), not consumed by the production
    pipeline.
    """
    predictor_arrays = {
        "beta_rad": np.asarray(beta_rad, dtype=float),
        "delta_f_rad": np.asarray(delta_f_rad, dtype=float),
        "v_mps": np.asarray(v_mps, dtype=float),
        "ax_mps2": np.asarray(ax_mps2, dtype=float),
    }
    if az_mps2 is not None:
        predictor_arrays["az_mps2"] = np.asarray(az_mps2, dtype=float)
    predictors = [c for c in _PREDICTOR_COLUMNS if c in predictor_arrays]
    if len(predictors) < 3:
        raise ValueError(
            "Need beta_rad plus at least two of delta_f_rad, v_mps, ax_mps2, "
            "az_mps2 for observed stability."
        )

    regression_text = " + ".join(
        c.replace("_rad", "").replace("_mps2", "").replace("_mps", "") for c in predictors
    )
    method = f"local weighted ridge regression in s +/- {window_m:.0f} m: mz_inertial ~ {regression_text}"

    n = len(s_m)
    s_m = np.asarray(s_m, dtype=float)
    keep = np.asarray(valid_mask, dtype=bool)

    work = pd.DataFrame({"s_m": s_m, **predictor_arrays, "mz_inertial_Nm": np.asarray(mz_inertial_Nm, dtype=float)})
    work.loc[~keep, :] = np.nan
    work = work.replace([np.inf, -np.inf], np.nan).dropna()

    result = np.full(n, np.nan)
    diagnostics = {
        "method": method,
        "predictors": predictors,
        "n_input_valid": int(len(work)),
        "n_grid_points": 0,
        "n_grid_valid": 0,
        "skip_counts": {"min_samples": 0, "beta_std": 0, "linalg_error": 0},
        "window_sample_counts": [],
    }
    if len(work) < min_samples:
        return pd.Series(result).to_numpy(), np.zeros(n, dtype=bool), diagnostics

    work = work.sort_values("s_m")
    coord_all = work["s_m"].to_numpy(dtype=float)
    x_all = work[predictors].to_numpy(dtype=float)
    y_all = work["mz_inertial_Nm"].to_numpy(dtype=float)

    coord_min = float(coord_all.min())
    coord_max = float(coord_all.max())
    grid = np.arange(coord_min, coord_max + grid_step_m, grid_step_m)
    sigma = max(window_m / GAUSS_SIGMA_DIVISOR, 1e-9)

    grid_values = np.full(len(grid), np.nan)
    skip_counts = {"min_samples": 0, "beta_std": 0, "linalg_error": 0}
    window_sample_counts = []

    for i, center in enumerate(grid):
        lo = np.searchsorted(coord_all, center - window_m, side="left")
        hi = np.searchsorted(coord_all, center + window_m, side="right")
        if hi - lo < min_samples:
            skip_counts["min_samples"] += 1
            continue
        window_sample_counts.append(int(hi - lo))

        x = x_all[lo:hi]
        y = y_all[lo:hi]
        d = np.abs(coord_all[lo:hi] - center)
        weights = np.exp(-0.5 * (d / sigma) ** 2)
        weight_sum = float(weights.sum())
        if weight_sum <= 0:
            continue

        x_mean = (x * weights[:, None]).sum(axis=0) / weight_sum
        y_mean = float((y * weights).sum() / weight_sum)
        x_centered = x - x_mean
        y_centered = y - y_mean
        variance = (weights[:, None] * (x_centered ** 2)).sum(axis=0) / weight_sum
        scale = np.sqrt(np.maximum(variance, 1e-12))
        if scale[0] < min_beta_std_rad:
            skip_counts["beta_std"] += 1
            continue

        x_scaled = x_centered / scale
        sqrt_weights = np.sqrt(weights)
        a = x_scaled * sqrt_weights[:, None]
        b = y_centered * sqrt_weights
        try:
            coef = np.linalg.solve(a.T @ a + ridge * np.eye(x_scaled.shape[1]), a.T @ b)
        except np.linalg.LinAlgError:
            skip_counts["linalg_error"] += 1
            continue

        beta_slope_nm_per_rad = coef[0] / scale[0]
        grid_values[i] = beta_slope_nm_per_rad * np.pi / 180.0

    valid_grid = np.isfinite(grid_values)
    valid_query = np.isfinite(s_m)
    if valid_grid.sum() >= 2:
        result[valid_query] = np.interp(
            s_m[valid_query], grid[valid_grid], grid_values[valid_grid],
            left=np.nan, right=np.nan,
        )
    valid = np.isfinite(result)

    diagnostics.update({
        "n_grid_points": int(len(grid)),
        "n_grid_valid": int(valid_grid.sum()),
        "skip_counts": skip_counts,
        "window_sample_counts": window_sample_counts,
    })
    return result, valid, diagnostics
