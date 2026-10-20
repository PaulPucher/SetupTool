# Per-session Pacejka tyre-curve fit + EKF validation, and
# resolve_sideslip_beta -- the live beta dispatcher for every sideslip_source.
#
# Steps: (a)+(b) axle curve fit, (c) R derivation from the fit residuals,
# (d) EKF run (modules/sideslip_ekf_pacejka.py), (e) validation (NIS,
# sign check, onset coverage, h2). The chain automates the frozen pass-0/1
# Dugoff workflow, removed 2026-10-17 (Class B rebuild; git history).
#
# Lateral-only axle fit (Pacejka 2nd ed.): pure side slip is the base curve,
# Eq. 4.49 / 4.E19 with zero shifts and free D (sec. 4.3.1; regression per
# sec. 1.2.1); an effective axle curve absorbing load transfer is valid for
# quasi-steady turning, peak may vanish (sec. 1.2.2). Combined slip only scales
# it by G_yk = 1 at zero slip ratio (Eqs. 4.64-4.69): moderate Fx is harmless,
# the large-Fx tails (front limit seen under braking) are the known bias.

import subprocess
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import minimize
from scipy.stats import chi2

from modules.stability_analysis import (
    prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_vertical_loads, load_car_data, apex_instant_slice,
    _butterworth_lowpass,
)
from modules.longitudinal_forces import estimate_longitudinal_forces
from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness
from modules.nis_gate import evaluate_gate
from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka

PACEJKA_START_GUESS = (12.0, 1.9, 8000.0, 0.97)  # chair start guess (B, C, D, E)

# Fit L (tyre_fit_mode "literature_bounded", Q-FIT-LIT 2026-10-18). These
# values define the method, they are not tuned: with an estimated sideslip
# and no sliding data the curve shape is not identified (Q-FIT-SIMPLE), so
# shape and curvature are fixed at literature values, mu is the demonstrated
# utilisation (config mu_demo_percentile) and B is the one fitted parameter.
# Deviation class FORCED ADAPTATION -- the chair fits all four on measured
# beta.
# C = 1.0: with E = -1 the curve rises monotonically to the asymptote D, so
# no post-peak drop is modelled where the data never show one (Voser et al.
# 2010; Rao et al. 2006; axle peak may vanish, Pacejka 2006 sec. 1.2.2; C
# lower bound, Goblirsch et al. 2025 Table I). C = 1.3 put an extrapolated
# falling branch in the observer's reach, which drove it to the hard bound
# (Q-OBS-REENTRY, 2026-10-18).
L_SHAPE_C = 1.0
L_CURVATURE_E = -1.0     # fixed E "according to the literature", Goblirsch et al. 2025; Pacejka Tables 4.1 / 4.2
# C = 1 with E in [-1, 1] has a positive slope everywhere, so there is no
# peak to locate; stated in the manifest instead of a NaN peak position
L_PEAK_NOTE = "none (C = 1, monotonic)"
L_B_BOUNDS = (5.0, 40.0)   # Goblirsch et al. 2025 Table I
# physical ceiling of a racing tyre's friction coefficient; a demonstrated
# utilisation above it points at the input data, not at the tyre (Pacejka
# 2006 sec. 1.2.1; Goblirsch et al. 2025 Table I upper bound of D / Fz)
L_MU_CEILING = 2.0
L_START_B = 12.0           # the chair's B start, inside the bounds
# D4 under L: cornering stiffness "ranges from about 6 to about 30 times the
# vertical wheel load ... The lower value holds for the older bias-ply tyre
# construction and the larger value for modern racing tyres" (Pacejka 2006
# sec. 1.2.1). An order of magnitude given to one significant figure, so it
# is read at that precision (author ruling 2026-10-20): inside the band ok,
# within the rounding margin of either "about" marginal (the EKF ships,
# flagged at the edge of the literature range), beyond it degenerate.
L_K_BAND = (6.0, 30.0)
L_K_EDGE = (5.5, 35.0)

TYRE_FIT_MODES = ("per_session", "prior_anchored", "literature_bounded")
SIDESLIP_SOURCES = ("kinematic", "ekf_auto_pacejka")
# Grid resolution of the prior-curve penalty; part of the method definition
# (what "mean distance over the visited range" means), not a tunable.
PRIOR_PENALTY_GRID_N = 200

CHI2_DF1_95 = float(chi2.ppf(0.95, df=1))
CHI2_DF2_95 = float(chi2.ppf(0.95, df=2))
NEAR_ZERO_SLIP_DEG = 0.2  # same near-zero alpha_r population as earlier validation


def _base_mask(state, laps):
    t = state["time"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    # R6: the smear zone goes with the mask -- samples a strike disturbs must
    # not shape the tyre estimate. This one mask feeds the fit, the R
    # derivation, the NIS sweep, the gate population and the sign check.
    smear_zone = state.get("kerb_smear_zone")
    if smear_zone is not None:
        moving = moving & ~smear_zone
    valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
    racing_mask = np.zeros_like(t, dtype=bool)
    for s, e in valid_windows:
        racing_mask |= (t >= s) & (t <= e)
    return moving & racing_mask


def _r_from_residuals(resid_front_full, resid_rear_full, common_mask, mass_kg):
    """Step (c), Method A: R_ay from fit residuals, inter-axle correlation
    measured on this session. Force residual / m -> ay residual (h = (Fy_f +
    Fy_r)/m). Inputs full length, zero outside each axle mask; common_mask =
    where both are defined.
    """
    rf = resid_front_full[common_mask]
    rr = resid_rear_full[common_mask]
    std_f = float(np.sqrt(np.mean(rf ** 2))) / mass_kg
    std_r = float(np.sqrt(np.mean(rr ** 2))) / mass_kg
    rho = float(np.corrcoef(rf, rr)[0, 1]) if len(rf) > 2 else 0.0
    r_ay_var = std_f ** 2 + std_r ** 2 + 2.0 * rho * std_f * std_r
    return r_ay_var, std_f, std_r, rho


def _sign_check(t, s_m, beta, ay, moving, corners_by_stable_id, laps_by_number, racing_ids):
    beta_deg = np.degrees(beta)
    n_match_median = n_total = 0
    n_match_median_racing = n_racing = 0
    per_num = per_den = 0
    for cid, instances in corners_by_stable_id.items():
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled_ay, pooled_beta = [], []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = moving[sl]
            if not m.any():
                continue
            pooled_ay.append(ay[sl][m])
            pooled_beta.append(beta_deg[sl][m])
        if not pooled_ay:
            continue
        ay_cat = np.concatenate(pooled_ay)
        beta_cat = np.concatenate(pooled_beta)
        med_ay = float(np.median(ay_cat))
        med_beta = float(np.median(beta_cat))
        dir_sign = np.sign(med_ay)
        median_match = (np.sign(med_beta) == -dir_sign) if dir_sign != 0 else None
        per_sample_match = (np.sign(beta_cat) == -dir_sign) if dir_sign != 0 else np.zeros_like(beta_cat, dtype=bool)
        n_total += 1
        n_match_median += int(bool(median_match))
        if cid in racing_ids:
            n_racing += 1
            n_match_median_racing += int(bool(median_match))
            per_num += int(np.sum(per_sample_match))
            per_den += len(per_sample_match)
    return {
        "median_gate_all": f"{n_match_median}/{n_total}",
        "median_gate_racing": f"{n_match_median_racing}/{n_racing}",
        "median_gate_racing_fraction": (n_match_median_racing / n_racing) if n_racing else float("nan"),
        "per_sample_fraction_racing": (per_num / per_den) if per_den else float("nan"),
    }


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo or s_m is None:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    # searchsorted needs ascending distances: search only up to the lap's
    # maximum, so samples after it (a reset or a backwards step at the lap
    # end) cannot misplace the slice (#23)
    k_max = int(np.flatnonzero(finite)[np.argmax(lap_s[finite])])
    lap_s = lap_s[:k_max + 1]
    finite = finite[:k_max + 1]
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(lap_s[k_max])
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


def _speed_window_mask(state, data, cfg):
    """Speed window on the Pacejka fit population, after the chair
    performance_analysis tooling (internal), tyre-fit speed bands: their band
    union drops the fastest, mostly straight-line stretches. One window instead
    of four band curves because the EKF consumes one signed curve per axle;
    the edges come from this session's own corner speeds (vehicle
    parameterisation), so the window adapts to the track.

    Percentile population: speed of moving samples inside corner instances on
    analysis laps (kerb samples included). Returns (mask, info); without corner
    instances, or with the config value null, the mask keeps everything and
    info says why.
    """
    n = state["time"].size
    pct = cfg.get("fit_speed_window_pct")
    if pct is None:
        return np.ones(n, dtype=bool), {"status": "off (fit_speed_window_pct is null)"}

    t, s_m = state["time"], state["s_m"]
    laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
    in_corner = np.zeros(n, dtype=bool)
    if s_m is not None:
        for c in data.get("corners", []):
            lap = laps_by_number.get(c.get("lap_number"))
            if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
                continue
            if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
                continue
            in_corner[_canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                              c["bracket_start_m"], c["bracket_end_m"])] = True
    pop = in_corner & state["moving_mask"]
    if not pop.any():
        return np.ones(n, dtype=bool), {"status": "not applied: no corner instances on analysis laps "
                                                  "(lap distance missing or no corners detected)"}

    v = state["v_mps"]
    lo, hi = (float(x) for x in np.percentile(v[pop], pct))
    return (v >= lo) & (v <= hi), {
        "status": "applied", "pct": [float(pct[0]), float(pct[1])],
        "lo_mps": lo, "hi_mps": hi, "lo_kmh": lo * 3.6, "hi_kmh": hi * 3.6,
        "n_incorner_samples": int(pop.sum()),
    }


def _fit_axle_pacejka(alpha, Fy, base_mask):
    """Steps (a)+(b), Pacejka: joint (B, C, D, E) Powell fit from the chair
    start guess, same base_mask. No bracket to hit -- Powell convergence is
    the nearest analogue, not a reliable degeneracy signal.
    """
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]

    if len(a2) == 0:
        # empty population (e.g. no valid laps on v3) -> np.percentile
        # would raise deep in numpy. Return no-signal; the caller turns it
        # into status="degenerate".
        return {
            "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
            "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
            "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
            "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }

    def _sse(p):
        B, C, D, E = p
        pred = pacejka_lateral_force(a2, B, C, D, E)
        return float(np.sum((pred - f2) ** 2))

    res = minimize(_sse, PACEJKA_START_GUESS, method="Powell")
    B, C, D, E = (float(x) for x in res.x)
    pred = pacejka_lateral_force(a2, B, C, D, E)
    resid = f2 - pred
    rms = float(np.sqrt(np.mean(resid ** 2)))

    # peak: grid over visited alpha range out to +/-90 deg, refined by
    # bisection on the stiffness sign change. Peak outside the data is
    # reported as such, not clamped.
    grid_deg = np.linspace(0.01, 89.9, 4000)
    grid_rad = np.radians(grid_deg)
    stiffness_grid = pacejka_lateral_stiffness(grid_rad, B, C, D, E)
    sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
    if len(sign_changes) > 0:
        i = sign_changes[0]
        peak_deg = float(grid_deg[i])
        peak_in_visited_range = peak_deg <= float(np.percentile(np.abs(np.degrees(a2)), 99))
    else:
        peak_deg = float("nan")
        peak_in_visited_range = False

    return {
        "B": B, "C": C, "D": D, "E": E,
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": rms,
        "peak_alpha_deg": peak_deg, "peak_in_visited_range": peak_in_visited_range,
        "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a2)), 99)),
        "sign_ok": D > 0,
        "residuals": resid, "residual_mask": m2,
    }


def _fit_axle_pacejka_anchored(alpha, Fy, base_mask, prior_axle, weight):
    """tyre_fit_mode 'prior_anchored' (EXPERIMENTAL, Tier A anchor pending):
    all four parameters fitted, Powell started from the prior, minimising
        mean_i (F(alpha_i) - Fy_i)^2 + weight * mean_g (F(g) - F_prior(g))^2
    with g on [0, p99 |alpha|] of this population. The penalty sits on the
    curve, not on (B, C, D, E): B, C and E trade off along a ridge on which
    the curve barely moves, so a parameter penalty would not constrain the
    force characteristic. Same dict as _fit_axle_pacejka plus the penalty
    terms. Moved unchanged from diagnostics/inspect_tyre_prior_d2.py.
    """
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
    a2, f2 = alpha[m2], Fy[m2]

    if len(a2) == 0:
        # empty population guard, as in _fit_axle_pacejka
        return {
            "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
            "prior_weight": weight, "curve_dev_rms_N": float("nan"),
            "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
            "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
            "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }

    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))
    g = np.radians(np.linspace(0.0, p99, PRIOR_PENALTY_GRID_N))
    pB, pC, pD, pE = (prior_axle[k] for k in "BCDE")
    f_prior = pacejka_lateral_force(g, pB, pC, pD, pE)

    def _j(p):
        B, C, D, E = p
        r = pacejka_lateral_force(a2, B, C, D, E) - f2
        j = float(np.mean(r * r))
        if weight:
            d = pacejka_lateral_force(g, B, C, D, E) - f_prior
            j += weight * float(np.mean(d * d))
        return j

    res = minimize(_j, (pB, pC, pD, pE), method="Powell")
    B, C, D, E = (float(x) for x in res.x)
    resid = f2 - pacejka_lateral_force(a2, B, C, D, E)
    dev = pacejka_lateral_force(g, B, C, D, E) - f_prior

    grid_deg = np.linspace(0.01, 89.9, 4000)
    stiffness_grid = pacejka_lateral_stiffness(np.radians(grid_deg), B, C, D, E)
    sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
    peak_deg = float(grid_deg[sign_changes[0]]) if len(sign_changes) > 0 else float("nan")

    return {
        "B": B, "C": C, "D": D, "E": E,
        "prior_weight": weight, "curve_dev_rms_N": float(np.sqrt(np.mean(dev ** 2))),
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
        "peak_alpha_deg": peak_deg, "peak_in_visited_range": bool(peak_deg == peak_deg and peak_deg <= p99),
        "visited_alpha_p99_deg": p99,
        "sign_ok": D > 0,
        "residuals": resid, "residual_mask": m2,
    }


def _fit_axle_pacejka_mu(alpha, Fy, Fz, base_mask):
    """(B, C, mu, E) with D = mu * Fz per sample (Milliken RCVD ch. 2 (sec. 2.1)).
    Same dict as _fit_axle_pacejka; D = mu * mean(Fz) as representative
    value for the constant-D consumers, plus "mu" and "mean_axle_fz_N".
    """
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz)
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]

    if len(a2) == 0:
        # empty population guard, as in _fit_axle_pacejka
        return {
            "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
            "mu": float("nan"), "mean_axle_fz_N": float("nan"),
            "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
            "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
            "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }

    mean_fz = float(np.mean(z2))
    # start mu = chair start peak force (8000 N) / this axle's mean Fz
    mu_start = PACEJKA_START_GUESS[2] / mean_fz if mean_fz else 1.5

    def _sse(p):
        B, C, mu, E = p
        pred = pacejka_lateral_force(a2, B, C, mu * z2, E)
        return float(np.sum((pred - f2) ** 2))

    start = (PACEJKA_START_GUESS[0], PACEJKA_START_GUESS[1], mu_start, PACEJKA_START_GUESS[3])
    res = minimize(_sse, start, method="Powell")
    B, C, mu, E = (float(x) for x in res.x)
    pred = pacejka_lateral_force(a2, B, C, mu * z2, E)
    resid = f2 - pred
    rms = float(np.sqrt(np.mean(resid ** 2)))

    D_repr = mu * mean_fz
    grid_deg = np.linspace(0.01, 89.9, 4000)
    grid_rad = np.radians(grid_deg)
    stiffness_grid = pacejka_lateral_stiffness(grid_rad, B, C, D_repr, E)
    sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
    if len(sign_changes) > 0:
        i = sign_changes[0]
        peak_deg = float(grid_deg[i])
        peak_in_visited_range = peak_deg <= float(np.percentile(np.abs(np.degrees(a2)), 99))
    else:
        peak_deg = float("nan")
        peak_in_visited_range = False

    return {
        "B": B, "C": C, "D": D_repr, "E": E, "mu": mu, "mean_axle_fz_N": mean_fz,
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": rms,
        "peak_alpha_deg": peak_deg, "peak_in_visited_range": peak_in_visited_range,
        "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a2)), 99)),
        "sign_ok": mu > 0,
        "residuals": resid, "residual_mask": m2,
    }


def _front_tyre_frame_force(fy_body, fx, delta):
    """Front axle lateral force in the wheel frame from the body-frame value
    and the axle Fx (Goblirsch et al. 2025 Eq. 13)."""
    return (fy_body - fx * np.sin(delta)) / np.cos(delta)


def _front_body_force(fy_tyre, fx, delta):
    """Inverse of _front_tyre_frame_force: body-lateral share of the steered
    wheel's tyre-frame lateral force and its Fx."""
    return fy_tyre * np.cos(delta) + fx * np.sin(delta)


def _analysis_lap_mask(t, laps):
    out = np.zeros_like(t, dtype=bool)
    for lap in laps:
        if lap.get("is_valid_for_analysis"):
            out |= (t >= lap["start_time"]) & (t <= lap["end_time"])
    return out


def _axle_load_inputs(state, data, params, forces, fit_mask):
    """Inputs of fit L besides alpha and Fy: axle Fz from the vertical-load
    cascade as configured (measured -> reconstructed -> static, no refusal),
    front Fx split by axle load (below), and the steer angle, each
    through the same 2 Hz order-4 zero-phase filter as Fy and alpha so the
    pairing stays consistent (Tier B; the fitted curve is a quasi-steady
    effective axle curve, Pacejka sec. 1.2.2). Also records which load source
    each corner used and the lowest accuracy level among them."""
    se = params["stability_estimation"]
    sr = state["sample_rate_hz"]
    cutoff = se["cs_filter_cutoff_hz"]
    fz = estimate_vertical_loads(state, forces, params, channels=data["channels"], car_data=load_car_data())
    # Front Fx after Goblirsch et al. 2025 Eq. 9: the CoG longitudinal force
    # (m ax + drag + rolling, the terms of estimate_longitudinal_forces) goes
    # to the front in proportion to the measured axle load under braking,
    # none under drive (rear drive). Not the brake-pressure split of
    # estimate_longitudinal_forces: a pressure ratio is a force ratio only
    # with known per-axle brake gains, and pad friction varies with
    # temperature (author ruling 2026-10-18). That split stays as is for its
    # own reader, the longitudinal-stiffness display.
    fx = estimate_longitudinal_forces(state, data["channels"], params)
    fx_cog = fx["fx_N"]
    with np.errstate(invalid="ignore", divide="ignore"):
        fx_front = np.where(fx_cog < 0.0, fx_cog * fz["fz_f_N"] / (fz["fz_f_N"] + fz["fz_r_N"]), 0.0)
    # rear: its load share of braking plus all of the drive
    raw = {"fz_f_N": fz["fz_f_N"], "fz_r_N": fz["fz_r_N"], "fx_f_N": fx_front, "fx_r_N": fx_cog - fx_front,
           "delta_f": state["delta_f_rad"]}
    bad = [k for k, v in raw.items() if not np.all(np.isfinite(v))]
    if bad:
        return {"degenerate_reason": f"fit L: non-finite input ({', '.join(bad)}) -- the zero-phase filter would spread it"}
    out = {k: _butterworth_lowpass(np.asarray(v, dtype=float), cutoff, sr) for k, v in raw.items()}

    levels = params["accuracy_levels"]
    level_of = {"damper": levels["wheel_load_damper"]["level"],
                "reconstructed": levels["wheel_load_damper_reconstructed"]["level"],
                "static_fallback": levels["per_wheel_load_split"]["level"]}
    moving_an = state["moving_mask"] & _analysis_lap_mask(state["time"], data.get("laps", []))
    src = fz["vertical_load_source_per_sample"]
    shares, used = {}, set()
    if src is None:
        used.add("static_fallback")
        shares = {"note": "static path (no damper data or no car_data) -- every sample static"}
    else:
        for c, lab in src.items():
            lab = np.asarray(lab)
            shares[c] = {pop_name: {s: float(np.mean(lab[pop] == s)) if pop.any() else float("nan")
                                    for s in level_of}
                         for pop_name, pop in (("fit_population", fit_mask), ("moving_analysis_laps", moving_an))}
            used |= set(np.unique(lab[moving_an])) & set(level_of)
    zero = data.get("event_zero_entry") or {}
    zero_applied = any(v == "applied" for v in (zero.get("channels") or {}).values())
    lowest = min(level_of[s] for s in used) if used else levels["vertical_load_split"]["level"]
    out["info"] = {
        "vertical_load_source_used": fz["vertical_load_source_used"],
        "source_shares": shares,
        "lowest_level": lowest,
        "level_note": ("damper loads carry an event zero entry (L4 with an L2 zero, Q-CAL-2)"
                       if zero_applied and "damper" in used else ""),
        "fx_front_source": "Goblirsch et al. 2025 Eq. 9: m ax + drag + rolling, braking split by the "
                           "measured axle loads, drive rear only",
        "fx_front_level": min(fx["accuracy_level"], lowest),
        "filter_cutoff_hz": cutoff,
    }
    return out


def _fit_axle_l(alpha, Fy, Fz, base_mask, bin_rad, mu_pct, weighted=True):
    """Fit L on one axle (Q-FIT-LIT): the load-normalised force f = Fy / Fz
    against slip angle with C and E fixed. mu is not fitted: it is the
    demonstrated utilisation, the mu_pct percentile of |Fy / Fz| on this
    population -- a lower bound of the true grip, because a fitted asymptote
    below forces the car demonstrably produced contradicts the measurement
    (Q-CAPACITY). B is then the one fitted parameter (bounded Powell,
    optional inverse-density weights on the signed slip angle), so it sets
    the origin slope K = B C mu. Fitting the ratio gives every sample the
    same weight whatever its load. Same dict as _fit_axle_pacejka, D = mu *
    Fz_ref as the representative peak force for the constant-D readers;
    residuals in N (Fy - Fz f), in the frame of Fy."""
    m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy) & np.isfinite(Fz) & (Fz > 0)
    a2, f2, z2 = alpha[m2], Fy[m2], Fz[m2]
    if len(a2) == 0:
        return {
            "B": float("nan"), "C": L_SHAPE_C, "D": float("nan"), "E": L_CURVATURE_E,
            "mu": float("nan"), "fz_ref_N": float("nan"), "weighted": weighted,
            "mu_source": f"demonstrated utilisation p{mu_pct:g} (lower bound)",
            "bound_active": {}, "powell_converged": False,
            "fit_n_samples": 0, "fit_rms_resid_N": float("nan"), "fit_rms_resid_norm": float("nan"),
            "peak": L_PEAK_NOTE, "visited_alpha_p99_deg": float("nan"),
            "sign_ok": False,
            "residuals": np.array([]), "residual_mask": m2,
        }
    y = f2 / z2
    fz_ref = float(np.median(z2))
    mu = float(np.percentile(np.abs(y), mu_pct))
    if weighted:
        _, inv, cnt = np.unique(np.floor(a2 / bin_rad).astype(np.int64), return_inverse=True, return_counts=True)
        w = 1.0 / cnt[inv]
    else:
        w = None

    def _j(p):
        r2 = (pacejka_lateral_force(a2, p[0], L_SHAPE_C, mu, L_CURVATURE_E) - y) ** 2
        return float(np.sum(r2 if w is None else w * r2))

    res = minimize(_j, (L_START_B,), method="Powell", bounds=[L_B_BOUNDS])
    B = float(res.x[0])
    shape = pacejka_lateral_force(a2, B, L_SHAPE_C, 1.0, L_CURVATURE_E)
    resid = f2 - mu * z2 * shape

    lo, hi = L_B_BOUNDS
    tol = 1e-4 * (hi - lo)
    bound_active = {"B": "lower" if B - lo <= tol else ("upper" if hi - B <= tol else None), "mu": None}

    p99 = float(np.percentile(np.abs(np.degrees(a2)), 99))

    return {
        "B": B, "C": L_SHAPE_C, "D": mu * fz_ref, "E": L_CURVATURE_E,
        "mu": mu, "fz_ref_N": fz_ref, "weighted": weighted, "bound_active": bound_active,
        "mu_source": f"demonstrated utilisation p{mu_pct:g} (lower bound)",
        "powell_converged": bool(res.success),
        "fit_n_samples": int(len(a2)), "fit_rms_resid_N": float(np.sqrt(np.mean(resid ** 2))),
        "fit_rms_resid_norm": float(np.sqrt(np.mean((y - mu * shape) ** 2))),
        "peak": L_PEAK_NOTE, "visited_alpha_p99_deg": p99,
        "sign_ok": mu > 0,
        "residuals": resid, "residual_mask": m2,
    }


# Fit L: how far the R_ay grid may be continued below its configured end
# before reaching the session's ay noise floor -- a guard against a missing
# or nonsensical floor, not a tunable (8 half-decade steps = 4 decades).
L_R_AY_MAX_EXTRA_STEPS = 8


def _q_upper_from_residual(front_fit, rear_fit, load_in, state, params):
    """Fit-L process noise per session (Q-UPPER, author ruling 2026-10-19,
    Level 2): the force residual of both axle fits, on the samples both fit
    populations share, is the error of the observer's force model; through
    the single-track equations it becomes an error of beta_dot,
    (dFy_f + dFy_r) / (m Vx), and of r_dot, (a dFy_f - b dFy_r) / Iz, taken
    sample-wise so the front/rear correlation stays in. Held over one step,
    the per-step variance is SD^2 dt_ref^2 at the q_reference_rate_hz grid
    (the convention the observer rescales to the session rate). An upper
    bound: the residual is taken at the kinematic slip angle and carries its
    error too. Diagonal, after Villano et al. 2021 (dynamic filter);
    physically meaningful variances after Bechtloff (ch. 5)."""
    vp, se, cfg = params["vehicle"], params["stability_estimation"], params["tyre_fit_auto"]
    n = state["v_mps"].size
    res = {}
    for name, fit in (("front", front_fit), ("rear", rear_fit)):
        full = np.full(n, np.nan)
        full[fit["residual_mask"]] = fit["residuals"]
        res[name] = full
    # front residual is in the tyre frame; the Fx sin(delta) term is on both
    # sides of it, so only cos(delta) carries over to the body frame
    dfy_f = res["front"] * np.cos(load_in["delta_f"])
    dfy_r = res["rear"]
    joint = front_fit["residual_mask"] & rear_fit["residual_mask"]
    vx = np.maximum(state["v_mps"], se["moving_speed_min_mps"])
    e_beta = (dfy_f + dfy_r) / (vp["mass_kg"] * vx)
    e_r = (vp["cog_to_front_axle_m"] * dfy_f - vp["cog_to_rear_axle_m"] * dfy_r) / vp["yaw_inertia_kgm2"]
    dt_ref = 1.0 / cfg["q_reference_rate_hz"]
    sd_beta, sd_r = float(e_beta[joint].std()), float(e_r[joint].std())
    return {"q_beta_var": sd_beta ** 2 * dt_ref ** 2, "q_yaw_rate_var": sd_r ** 2 * dt_ref ** 2,
            "sd_beta_dot": sd_beta, "sd_r_dot": sd_r, "n_joint": int(joint.sum()),
            "method": "held-over-step upper bound from the fit-L force residual (Level 2)"}


def _measurement_noise_floors(state, laps, params):
    """Measurement-noise variances of the session (author rulings
    2026-10-19, Level 2, Tier B): each observer measurement minus its
    zero-phase low-pass at the CS filter cutoff (2 Hz, the same cutoff as the
    CS and Fz filters), on the steering-defined straights. The single-track
    model has no vertical or vibration dynamics, so what the signal carries
    above the cutoff there is noise for it -- R covers what the model cannot
    predict. Taken on exactly the arrays the measurement update reads:
    state["ay_mps2"] (log_acc_y x 9.81) and state["yaw_rate_radps"]
    (sclu_yaw_rate x yaw_rate_to_radps), both linearly interpolated onto the
    grid and otherwise unfiltered. ay is the floor for R_ay; yaw rate gives
    R_yaw directly."""
    from modules.straights import steering_straights
    sd = steering_straights(state, laps, params["tyre_fit_auto"])
    cutoff, sr = params["stability_estimation"]["cs_filter_cutoff_hz"], state["sample_rate_hz"]
    out = {"n_segments": len(sd["segs"])}
    for name, key in (("ay", "ay_mps2"), ("yaw_rate", "yaw_rate_radps")):
        x = state[key]
        hp = x - _butterworth_lowpass(x, cutoff, sr)
        m = sd["mask"] & np.isfinite(hp)
        out[name] = {"var": float(np.var(hp[m])) if m.any() else float("nan"), "n_samples": int(m.sum()),
                     "straight_s": float(m.sum() / sr)}
    return out


def plausibility_gate_l(alpha, fit_mask, axle_fit, cfg):
    """D4 for fit L (Q-FIT-LIT reopen, 2026-10-18, declared post-result).
    Under L the curve peak lies beyond the visited slip range wherever the
    data do not reach it, so it is an extrapolation (Goblirsch et al. 2025)
    and cannot be gated on; what the data do identify is the origin slope.
      g1 excitation: unchanged (p1..p99 span of the fit's slip angles)
      g2 normalised origin slope K = B C mu inside L_K_BAND; the axle ratio
         is the sum of tyre stiffnesses over the sum of tyre loads, so the
         tyre band carries over to the axle; inside L_K_EDGE but outside
         the band it only flags (k_zone "edge", status marginal), beyond
         L_K_EDGE it fails (2026-10-20)
      demonstrated mu above L_MU_CEILING: more grip than a racing tyre has
         points at the force or load inputs, not at the tyre (2026-10-19;
         replaces the mu-at-bound check of the fitted-mu version)
    No peak to report: with C = 1 the curve is monotonic, and mu is the
    demonstrated utilisation, not an asymptote read off the curve (wording
    ruling 2026-10-20)."""
    a = np.degrees(alpha[fit_mask & np.isfinite(alpha)])
    span = float(np.percentile(a, 99) - np.percentile(a, 1)) if a.size else float("nan")
    K = float(axle_fit["B"] * axle_fit["C"] * axle_fit["mu"])
    failed = []
    if not span >= cfg["min_alpha_span_deg"]:
        failed.append("g1 excitation")
    if L_K_BAND[0] <= K <= L_K_BAND[1]:
        k_zone = "in band"
    elif L_K_EDGE[0] <= K <= L_K_EDGE[1]:
        k_zone = "edge"
    else:
        k_zone = "outside"
        failed.append("g2 normalised stiffness")
    if not axle_fit["mu"] <= L_MU_CEILING:
        failed.append("mu_demo above the physical ceiling 2.0")
    return {"alpha_span_deg": span, "stiffness_norm_per_rad": K, "k_zone": k_zone,
            "mu": axle_fit["mu"], "mu_ceiling": L_MU_CEILING, "mu_source": axle_fit["mu_source"],
            "peak": axle_fit["peak"], "failed": failed}


def plausibility_gate(alpha, fit_mask, axle_fit, cfg):
    """D4 garbage catch on one fitted axle curve (2026-10-07). Bands are
    deliberately wide: they reject curves no GT3 axle can have, they do not
    adjudicate along the B/C/E ridge.
      g1 excitation: p1..p99 span of the fit's slip-angle data
      g2 linear stiffness at zero slip, dFy/dalpha(0) = B*C*D
      g3 curve peak (numerical argmax, positive slip side); no peak inside
         the grid counts as a fail
    """
    a = np.degrees(alpha[fit_mask & np.isfinite(alpha)])
    span = float(np.percentile(a, 99) - np.percentile(a, 1)) if a.size else float("nan")
    B, C, D, E = (axle_fit[k] for k in "BCDE")
    stiffness = float(B * C * D)
    grid_deg = np.linspace(0.01, 89.9, 4000)
    i = int(np.argmax(pacejka_lateral_force(np.radians(grid_deg), B, C, D, E)))
    peak = float(grid_deg[i]) if 0 < i < grid_deg.size - 1 else float("nan")
    failed = []
    if not span >= cfg["min_alpha_span_deg"]:
        failed.append("g1 excitation")
    if not cfg["stiffness_min_npr"] <= stiffness <= cfg["stiffness_max_npr"]:
        failed.append("g2 stiffness")
    if not cfg["peak_min_deg"] <= peak <= cfg["peak_max_deg"]:
        failed.append("g3 peak")
    return {"alpha_span_deg": span, "stiffness_npr": stiffness, "peak_deg": peak, "failed": failed}


def fit_session_pacejka(data, params, data_file_path=None, load_normalised=False, tyre_fit_mode="per_session"):
    """Per-session fit with the reduced Magic Formula, steps (a)-(e). R
    selection of the sweep (2026-10-11): ay exceedance only, band centre,
    sign-check guard, config-default R as fallback (status at most
    'marginal', reason in marginal_reason) -- see the comment at the sweep.

    load_normalised=True: D = mu * Fz with measured Fz (Milliken RCVD ch. 2
    (sec. 2.1)). No measured Fz -> degenerate manifest, no silent free-D
    fallback. Default False = free-D.

    tyre_fit_mode: 'per_session' (default) or 'prior_anchored' (experimental,
    see _fit_axle_pacejka_anchored; prior and weight from config). The
    prior is a free-D curve, so prior_anchored with load_normalised is
    refused rather than mixed.

    tyre_fit_mode 'literature_bounded': fit L (Q-FIT-LIT, inactive by
    default) -- _fit_axle_l on the load-normalised force, front in the tyre
    frame; the observer gets mu and the per-sample axle loads (d_is_mu). The
    manifest adds axle_load (source shares, lowest level), front_force_frame
    and fit_density_bin_rad; axles carry mu, fz_ref_N, bound_active.
    """
    if tyre_fit_mode not in TYRE_FIT_MODES:
        raise ValueError(f"tyre_fit_mode {tyre_fit_mode!r} not recognised (expected one of {TYRE_FIT_MODES})")
    cfg = params["tyre_fit_auto"]
    vp = params["vehicle"]
    mass_kg = vp["mass_kg"]

    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        return {"status": "degenerate", "degenerate_reason": "prepare_vehicle_state returned None "
                "-- required channels missing from this session", "data_file": data_file_path}

    laps = data.get("laps", [])
    base_mask = _base_mask(state, laps)
    # the window narrows the curve fit only; NIS sweep, gate and sign check
    # keep base_mask (the observer must stay valid on every racing sample)
    window, window_info = _speed_window_mask(state, data, cfg)
    fit_mask = base_mask & window
    if window_info["status"] == "applied":
        window_info["base_retained_fraction"] = float(fit_mask.sum() / max(1, base_mask.sum()))

    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)

    if tyre_fit_mode == "prior_anchored" and load_normalised:
        return {"status": "degenerate", "degenerate_reason": "tyre_fit_mode 'prior_anchored' requires the free-D "
                "fit (load_normalised False) -- the prior is a free-D curve", "data_file": data_file_path}
    if tyre_fit_mode == "literature_bounded" and load_normalised:
        return {"status": "degenerate", "degenerate_reason": "tyre_fit_mode 'literature_bounded' is load-normalised "
                "by construction -- load_normalised True would mix two mu fits", "data_file": data_file_path}

    lit = tyre_fit_mode == "literature_bounded"
    if lit:
        load_in = _axle_load_inputs(state, data, params, forces, fit_mask)
        if "degenerate_reason" in load_in:
            return {"status": "degenerate", "degenerate_reason": load_in["degenerate_reason"],
                    "data_file": data_file_path}
        fy_f_tyre = _front_tyre_frame_force(forces["Fy_f_filt"], load_in["fx_f_N"], load_in["delta_f"])
        bin_rad, mu_pct = cfg["fit_density_bin_rad"], cfg["mu_demo_percentile"]
        # heavy braking leaves the pure-slip fit: beyond fit_long_util_max the
        # friction circle takes more than a few percent of lateral capacity
        # (config derived_from); the observer still sees every sample.
        # Braking side only (2026-10-19): Eq. 9 puts all drive on the rear,
        # so traction is the rear's normal operating state through every
        # corner exit, not an outlier; removing it took over half the rear
        # population. Its combined-slip cost stays, sized by the friction
        # circle.
        axle_masks = {}
        for axle, fx_k, fz_k in (("front", "fx_f_N", "fz_f_N"), ("rear", "fx_r_N", "fz_r_N")):
            with np.errstate(invalid="ignore", divide="ignore"):
                util = np.abs(load_in[fx_k]) / load_in[fz_k]
            keep = ~((load_in[fx_k] < 0) & (util > cfg["fit_long_util_max"]))
            axle_masks[axle] = fit_mask & keep
            load_in["info"].setdefault("braking_excluded", {})[axle] = {
                "n": int((fit_mask & ~keep).sum()), "share": float((fit_mask & ~keep).sum() / max(1, fit_mask.sum()))}
        front_fit = _fit_axle_l(slip_kin["alpha_f_filt"], fy_f_tyre, load_in["fz_f_N"], axle_masks["front"], bin_rad,
                                mu_pct)
        rear_fit = _fit_axle_l(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], load_in["fz_r_N"], axle_masks["rear"],
                               bin_rad, mu_pct)
    elif load_normalised:
        car_data = load_car_data()
        if car_data is None:
            return {"status": "degenerate", "degenerate_reason": "load_normalised=True requires "
                    "car_data.json, not available", "data_file": data_file_path}
        # force measured Fz for this call -- the global flag defaults to
        # "static" and the mu fit would silently never run
        params_measured_fz = dict(params)
        params_measured_fz["stability_estimation"] = dict(params["stability_estimation"])
        params_measured_fz["stability_estimation"]["vertical_load_source"] = "measured"
        fz = estimate_vertical_loads(state, forces, params_measured_fz,
                                      channels=data["channels"], car_data=car_data)
        if fz.get("vertical_load_source_used") != "measured":
            return {"status": "degenerate", "degenerate_reason": "load_normalised=True requires measured "
                    "Fz -- no damper-valid samples this session (vertical_load_source resolved to 'static')",
                    "data_file": data_file_path}
        front_fit = _fit_axle_pacejka_mu(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], fz["fz_f_N"], fit_mask)
        rear_fit = _fit_axle_pacejka_mu(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], fz["fz_r_N"], fit_mask)
    elif tyre_fit_mode == "prior_anchored":
        prior, weight = cfg["tyre_fit_prior"], cfg["prior_weight"]
        front_fit = _fit_axle_pacejka_anchored(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], fit_mask,
                                               prior["front"], weight)
        rear_fit = _fit_axle_pacejka_anchored(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], fit_mask,
                                              prior["rear"], weight)
    else:
        front_fit = _fit_axle_pacejka(slip_kin["alpha_f_filt"], forces["Fy_f_filt"], fit_mask)
        rear_fit = _fit_axle_pacejka(slip_kin["alpha_r_filt"], forces["Fy_r_filt"], fit_mask)

    manifest = {
        "data_file": data_file_path,
        "load_normalised": load_normalised,
        "tyre_fit_mode": tyre_fit_mode,
        "speed_window": window_info,
        "laps_used": sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis")),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "axles": {"front": {k: v for k, v in front_fit.items() if k not in ("residuals", "residual_mask")},
                  "rear": {k: v for k, v in rear_fit.items() if k not in ("residuals", "residual_mask")}},
    }
    if tyre_fit_mode == "prior_anchored":
        manifest["prior_weight"] = cfg["prior_weight"]
        manifest["prior_derived_from"] = cfg["tyre_fit_prior"]["derived_from"]
    if lit:
        manifest["axle_load"] = load_in["info"]
        manifest["front_force_frame"] = "tyre frame (Goblirsch et al. 2025 Eq. 13), fit and observer"
        manifest["fit_density_bin_rad"] = bin_rad
        # measured axle loads (2 Hz zero-phase) for the load-normalised CS
        # (author ruling 2026-10-19) and the Eq. 9 axle Fx for the floor-event
        # attributes (2026-10-20); set before any degenerate return, since
        # neither depends on the fit. In memory only, like nis_full
        manifest["cs_axle_loads"] = {"fz_f_N": load_in["fz_f_N"], "fz_r_N": load_in["fz_r_N"]}
        manifest["axle_fx"] = {"fx_f_N": load_in["fx_f_N"], "fx_r_N": load_in["fx_r_N"]}
    try:
        git_hash = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    except Exception as e:
        git_hash = f"UNAVAILABLE ({e})"
    manifest["git_commit_hash"] = git_hash

    if not (front_fit["sign_ok"] and front_fit["powell_converged"] and
            rear_fit["sign_ok"] and rear_fit["powell_converged"]):
        manifest["status"] = "degenerate"
        manifest["degenerate_reason"] = "Powell did not converge or D<=0 (sign check failed) on at least one axle"
        return manifest

    common_mask = front_fit["residual_mask"] & rear_fit["residual_mask"]
    resid_front_full = np.zeros_like(base_mask, dtype=float)
    resid_rear_full = np.zeros_like(base_mask, dtype=float)
    if lit:
        # R is a lateral-acceleration noise: the front residual goes back to
        # the body frame (the Fx term cancels in the difference)
        resid_front_full[front_fit["residual_mask"]] = (
            front_fit["residuals"] * np.cos(load_in["delta_f"][front_fit["residual_mask"]]))
    else:
        resid_front_full[front_fit["residual_mask"]] = front_fit["residuals"]
    resid_rear_full[rear_fit["residual_mask"]] = rear_fit["residuals"]
    r_ay_var_derived, std_f, std_r, rho = _r_from_residuals(resid_front_full, resid_rear_full, common_mask, mass_kg)

    # fit L hands the observer mu and the per-sample loads; the shared state
    # is copied, not extended, so no other reader sees the extra keys
    d_front, d_rear = (front_fit["mu"], rear_fit["mu"]) if lit else (front_fit["D"], rear_fit["D"])
    ekf_state = state
    if lit:
        ekf_state = dict(state)
        ekf_state["fz_axle_f_N"] = load_in["fz_f_N"]
        ekf_state["fz_axle_r_N"] = load_in["fz_r_N"]
        ekf_state["fx_axle_f_N"] = load_in["fx_f_N"]
    interim_cfg = {
        "b_front": front_fit["B"], "c_front": front_fit["C"], "d_front": d_front, "e_front": front_fit["E"],
        "b_rear": rear_fit["B"], "c_rear": rear_fit["C"], "d_rear": d_rear, "e_rear": rear_fit["E"],
        "Q_beta_var": cfg["q_beta_var"], "Q_yaw_rate_var": cfg["q_yaw_rate_var"],
        "q_reference_rate_hz": cfg["q_reference_rate_hz"],
        "P0_beta_var": cfg["p0_beta_var"], "P0_yaw_rate_var": cfg["p0_yaw_rate_var"],
        "R_ay_var": r_ay_var_derived, "R_yaw_rate_var": cfg["r_yaw_rate_interim_seed_var"],
        "beta_hard_bound_deg": cfg["beta_hard_bound_deg"],
        "nis_window_s": cfg["nis_window_s"], "nis_chi2_bound": cfg["nis_chi2_bound"],
        "nis_flag_fraction": cfg["nis_flag_fraction"],
    }
    if lit:
        interim_cfg["d_is_mu"] = True
        # fit L: Q from this session's force residual (Q-UPPER, 2026-10-19),
        # trigger from its own key (default hard bound only), so nothing here
        # can move the production observer
        q_l = _q_upper_from_residual(front_fit, rear_fit, load_in, state, params)
        manifest["observer_q"] = q_l
        interim_cfg["Q_beta_var"] = q_l["q_beta_var"]
        interim_cfg["Q_yaw_rate_var"] = q_l["q_yaw_rate_var"]
        interim_cfg["divergence_trigger"] = cfg["l_divergence_trigger"]
        interim_cfg["cd_drift"] = cfg["l_cd_drift"]
        interim_cfg["cd_threshold"] = cfg["l_cd_threshold_h"]
    params_interim = dict(params)
    params_interim["tyre_model_ekf_pacejka"] = {"_auto_interim": interim_cfg}
    interim_result = estimate_sideslip_ekf_pacejka(ekf_state, params_interim, pass_id="_auto_interim")
    yaw_innov_interim = interim_result["innovation"][base_mask][:, 0]
    r_yaw_rate_var_derived = float(np.mean(yaw_innov_interim ** 2))

    manifest["r_derivation"] = {
        "std_front_mps2": std_f, "std_rear_mps2": std_r, "inter_axle_correlation": rho,
        "r_ay_var_derived": r_ay_var_derived, "r_yaw_rate_var_derived": r_yaw_rate_var_derived,
    }

    corners = data.get("corners", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    corners_by_stable_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    racing_ids = [cid for cid, insts in corners_by_stable_id.items() if insts[0].get("speed_class") != "low"]
    t = state["time"]
    s_m = state.get("s_m")
    ay = state["ay_mps2"]

    # R selection (author ruling 2026-10-11): on the ay exceedance only, in
    # [r_sweep_ay_band_low, r_sweep_ay_band_high], nearest the band centre,
    # guarded by the sign check of that grid point's own run (>= the marginal
    # limit). Under the corrected re-entry the yaw exceedance stays far above
    # any NIS band on every grid point -- the yaw innovation is dominated by
    # single-track model mismatch, not sensor noise -- so it is logged per
    # point, not selected on. No guarded in-band point -> the config default R,
    # never an arbitrary "nearest overall" pick (that pick inverted the
    # physics on Paul Ricard). Tier B: a data-derived calibration rule on the
    # existing chi-square exceedance statistic, not a new method.
    band_lo, band_hi = cfg["r_sweep_ay_band_low"], cfg["r_sweep_ay_band_high"]
    centre = (band_lo + band_hi) / 2.0

    def _sweep_point(r_ay_scale, r_yaw_scale):
        sweep_cfg = dict(interim_cfg)
        sweep_cfg["R_ay_var"] = r_ay_var_derived * r_ay_scale
        sweep_cfg["R_yaw_rate_var"] = r_yaw_rate_var_derived * r_yaw_scale
        params_sweep = dict(params)
        params_sweep["tyre_model_ekf_pacejka"] = {"_auto_sweep": sweep_cfg}
        result = estimate_sideslip_ekf_pacejka(ekf_state, params_sweep, pass_id="_auto_sweep")
        innovation = result["innovation"][base_mask]
        S_diag = result["S_diag"][base_mask]
        f_yaw = float((innovation[:, 0] ** 2 / S_diag[:, 0] > CHI2_DF1_95).mean())
        f_ay = float((innovation[:, 1] ** 2 / S_diag[:, 1] > CHI2_DF1_95).mean())
        # guard judges the beta this grid point would ship (#20)
        sign = _sign_check(t, s_m, result["beta_with_fallback"], ay, base_mask, corners_by_stable_id,
                           laps_by_number, racing_ids)
        return {"r_ay_scale": r_ay_scale, "r_yaw_scale": r_yaw_scale,
                "R_ay_var": sweep_cfg["R_ay_var"], "R_yaw_rate_var": sweep_cfg["R_yaw_rate_var"],
                "f_yaw": f_yaw, "f_ay": f_ay, "ay_in_band": band_lo <= f_ay <= band_hi,
                "sign_fraction": sign["median_gate_racing_fraction"]}

    ay_scales = list(cfg["r_sweep_ay_scales"])
    yaw_scales = list(cfg["r_sweep_yaw_scales"])
    floor_scale = None
    if lit:
        # measured noise floors (author rulings 2026-10-19), in scale units:
        # R_yaw is the yaw-rate floor itself, not swept -- with Q_r at the
        # upper bound the yaw state follows the gyro and the sweep only ran
        # into its grid ends. R_ay keeps the sweep with the ay floor as its
        # lowest candidate; the half-decade grid (1, 0.3, 0.1, ...) is
        # continued downward until one value lies below it, so a sweep that
        # prefers lower shows as floor-binding instead of as the grid edge.
        floors = _measurement_noise_floors(state, laps, params)
        floor_scale = floors["ay"]["var"] / r_ay_var_derived
        yaw_floor_scale = floors["yaw_rate"]["var"] / r_yaw_rate_var_derived
        floors["ay"]["r_ay_scale"], floors["yaw_rate"]["r_yaw_scale"] = floor_scale, yaw_floor_scale
        manifest["noise_floors"] = floors
        if yaw_floor_scale == yaw_floor_scale:
            yaw_scales = [yaw_floor_scale]
        if floor_scale == floor_scale:
            lo = min(ay_scales)
            for _ in range(L_R_AY_MAX_EXTRA_STEPS):
                if lo < floor_scale:
                    break
                mant = lo / 10.0 ** np.floor(np.log10(lo))
                lo = float(f"{lo * (0.3 if abs(mant - 1.0) < 1e-6 else 1.0 / 3.0):.3g}")
                ay_scales.append(lo)
            ay_scales.append(floor_scale)
        else:
            floor_scale = None
    sweep_results = [_sweep_point(a_, y_) for a_ in ay_scales for y_ in yaw_scales]

    def _pick(admissible):
        cands = sorted((r for r in sweep_results if r["ay_in_band"] and admissible(r)),
                       key=lambda r: abs(r["f_ay"] - centre))
        tr = []
        for r in cands:
            passed = r["sign_fraction"] >= cfg["sign_check_marginal_fraction"]
            tr.append({"r_ay_scale": r["r_ay_scale"], "r_yaw_scale": r["r_yaw_scale"], "f_ay": r["f_ay"],
                       "sign_fraction": r["sign_fraction"], "guard_pass": passed})
            if passed:
                return r, cands, tr
        return None, cands, tr

    if floor_scale is None:
        chosen, candidates, trace = _pick(lambda r: True)
    else:
        chosen, candidates, trace = _pick(lambda r: r["r_ay_scale"] >= floor_scale)
        free, _, _ = _pick(lambda r: True)
        if chosen is None:
            loc = None
        elif chosen["r_ay_scale"] == floor_scale:
            loc = "floor"
        elif chosen["r_ay_scale"] == max(ay_scales):
            loc = "top edge"
        else:
            loc = "interior"
        manifest["noise_floors"]["ay"]["pick_location"] = loc
        manifest["noise_floors"]["ay"]["floor_binding"] = bool(free is not None and free["r_ay_scale"] < floor_scale)
        manifest["noise_floors"]["ay"]["pick_without_floor"] = free and {k: free[k] for k in ("r_ay_scale",
                                                                                              "r_yaw_scale")}
    if chosen is not None:
        selection = "in_band"
    else:
        selection = "r_sweep_guard_fail" if candidates else "r_sweep_no_in_band"
        d_ay, d_yaw = cfg["r_sweep_default_ay_scale"], cfg["r_sweep_default_yaw_scale"]
        if lit:
            # fit L: the default never undercuts the measured floors
            d_yaw = yaw_scales[0] if len(yaw_scales) == 1 else d_yaw
            d_ay = max(d_ay, floor_scale) if floor_scale is not None else d_ay
        chosen = next((r for r in sweep_results if r["r_ay_scale"] == d_ay and r["r_yaw_scale"] == d_yaw), None)
        if chosen is None:
            chosen = _sweep_point(d_ay, d_yaw)
    sweep_found_in_band = selection == "in_band"
    manifest["r_sweep"] = {"grid_points": sweep_results, "chosen": chosen, "found_in_band": sweep_found_in_band,
                           "selection": selection, "guard_trace": trace}

    final_cfg = dict(interim_cfg)
    final_cfg["R_ay_var"] = chosen["R_ay_var"]
    final_cfg["R_yaw_rate_var"] = chosen["R_yaw_rate_var"]
    params_final = dict(params)
    params_final["tyre_model_ekf_pacejka"] = {"_auto_final": final_cfg}
    final_result = estimate_sideslip_ekf_pacejka(ekf_state, params_final, pass_id="_auto_final")
    beta_ekf = final_result["beta"]
    # Sign check, coverage and h2 validate the beta that ships, not the raw
    # update (#20): the two differ at hard-bound re-entries, where the raw
    # value sits beyond the bound and the shipped one is the reset state.
    beta_shipped = final_result["beta_with_fallback"]
    manifest["final_config"] = final_cfg

    innovation = final_result["innovation"][base_mask]
    nis_combined = final_result["nis"][base_mask]
    S_diag = final_result["S_diag"][base_mask]
    nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
    nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]
    f_yaw = float((nis_yaw > CHI2_DF1_95).mean())
    f_ay = float((nis_ay > CHI2_DF1_95).mean())
    f_comb = float((nis_combined > CHI2_DF2_95).mean())
    manifest["nis"] = {"yaw_rate_exceedance": f_yaw, "ay_exceedance": f_ay,
                        "combined_exceedance": f_comb, "combined_mean_nis": float(np.mean(nis_combined))}

    manifest["sign_check"] = _sign_check(t, s_m, beta_shipped, ay, base_mask, corners_by_stable_id, laps_by_number,
                                         racing_ids)

    slip_ekf = estimate_slip_angles(state, beta_shipped, params)
    if not lit:
        # share of samples past the curve peak; fit L has no peak (C = 1)
        coverage = {}
        for axle_name, alpha_c, fit in (("front", slip_ekf["alpha_f_filt"], front_fit),
                                        ("rear", slip_ekf["alpha_r_filt"], rear_fit)):
            alpha_deg = np.degrees(alpha_c)[base_mask]
            if np.isfinite(fit["peak_alpha_deg"]):
                frac_beyond = float((np.abs(alpha_deg) > fit["peak_alpha_deg"]).mean())
            else:
                frac_beyond = float("nan")
            coverage[axle_name] = {"peak_alpha_deg": fit["peak_alpha_deg"], "coverage_fraction": frac_beyond}
        manifest["onset_coverage"] = coverage

    apex_half = params["stability_estimation"]["apex_half_window_samples"]
    apex_mask = np.zeros_like(t, dtype=bool)
    for c in corners:
        start_t, end_t = c["segments"]["apex_3"]
        if end_t < start_t:
            continue
        apex_mask[apex_instant_slice(t, start_t, apex_half)] = True
    apex_pop = base_mask & apex_mask
    idx = np.where(apex_pop)[0]
    if lit:
        # same force model as the observer: D = mu Fz(t), front back to the body frame
        f_tyre = pacejka_lateral_force(slip_ekf["alpha_f_filt"][idx], front_fit["B"], front_fit["C"],
                                       front_fit["mu"] * load_in["fz_f_N"][idx], front_fit["E"])
        h2_pred = (_front_body_force(f_tyre, load_in["fx_f_N"][idx], load_in["delta_f"][idx]) +
                   pacejka_lateral_force(slip_ekf["alpha_r_filt"][idx], rear_fit["B"], rear_fit["C"],
                                         rear_fit["mu"] * load_in["fz_r_N"][idx], rear_fit["E"])) / mass_kg
    else:
        h2_pred = (pacejka_lateral_force(slip_ekf["alpha_f_filt"][idx], front_fit["B"], front_fit["C"], front_fit["D"], front_fit["E"]) +
                   pacejka_lateral_force(slip_ekf["alpha_r_filt"][idx], rear_fit["B"], rear_fit["C"], rear_fit["D"], rear_fit["E"])) / mass_kg
    r_h2 = float(np.corrcoef(h2_pred, ay[idx])[0, 1]) if len(idx) > 2 else float("nan")
    manifest["h2_vs_ay_apex"] = {"n": int(len(idx)), "correlation": r_h2}

    # The gross-miscalibration trigger reads the ay exceedance only: under the
    # corrected re-entry the yaw exceedance measures single-track model
    # mismatch, not noise calibration (ruling 2026-10-11, Tier B), so yaw can
    # only cap the status through the soft yaw_mismatch_ceiling.
    gate_frac = manifest["sign_check"]["median_gate_racing_fraction"]
    gross = cfg["nis_gross_miscalibration_fraction"]
    # No racing corner to check (NaN fraction) used to pass every comparison
    # silently; an unvalidated fit is degenerate, not ok (#22).
    sign_nan = gate_frac != gate_frac
    if sign_nan or gate_frac < cfg["sign_check_degenerate_fraction"] or f_ay > gross:
        manifest["status"] = "degenerate"
        parts = []
        if sign_nan:
            parts.append("sign check not computable")
        elif gate_frac < cfg["sign_check_degenerate_fraction"]:
            parts.append(f"sign check: median gate racing fraction {gate_frac:.3f} < "
                         f"{cfg['sign_check_degenerate_fraction']:g}")
        if f_ay > gross:
            parts.append(f"NIS gross miscalibration (ay): exceedance {f_ay:.3f} > {gross:g}")
        manifest["degenerate_reason"] = "; ".join(parts)
    else:
        reasons = [] if sweep_found_in_band else [manifest["r_sweep"]["selection"]]
        if gate_frac < cfg["sign_check_marginal_fraction"]:
            reasons.append(f"sign check: median gate racing fraction {gate_frac:.3f} < "
                           f"{cfg['sign_check_marginal_fraction']:g}")
        if f_yaw > cfg["yaw_mismatch_ceiling"]:
            reasons.append("yaw_mismatch_high")
        if reasons:
            manifest["status"] = "marginal"
            manifest["marginal_reason"] = "; ".join(reasons)
        else:
            manifest["status"] = "ok"

    # D4: an implausible curve takes the existing degenerate route (kinematic
    # fallback in resolve_sideslip_beta). The verdict lives in the in-memory
    # manifest only -- resolve_sideslip_beta strips it, the failed checks
    # reach the persisted payload through degenerate_reason / fallback_reason.
    gate_fn = plausibility_gate_l if lit else plausibility_gate
    gate = {name: gate_fn(slip_kin[key], fit_mask, fit, cfg)
            for name, key, fit in (("front", "alpha_f_filt", front_fit), ("rear", "alpha_r_filt", rear_fit))}
    manifest["plausibility_gate"] = gate
    gate_failures = [f"{name}: {', '.join(g['failed'])}" for name, g in gate.items() if g["failed"]]
    if gate_failures:
        reason = "tyre curve plausibility gate failed -- " + "; ".join(gate_failures)
        if manifest["status"] == "degenerate":
            reason = manifest["degenerate_reason"] + "; " + reason
        manifest["status"] = "degenerate"
        manifest["degenerate_reason"] = reason
    if lit:
        # g2 at the edge of the literature range: the observer still ships,
        # the session is flagged (author ruling 2026-10-20)
        edge = [f"{name}: g2 at the edge of the literature range (K {g['stiffness_norm_per_rad']:.2f} / rad)"
                for name, g in gate.items() if g["k_zone"] == "edge"]
        manifest["k_band_zone"] = {name: g["k_zone"] for name, g in gate.items()}
        if edge and manifest["status"] != "degenerate":
            prev = manifest.get("marginal_reason")
            manifest["status"] = "marginal"
            manifest["marginal_reason"] = "; ".join(([prev] if prev else []) + edge)

    if load_normalised:
        # mu outside plausibility band -> reported, not dropped
        mu_lo, mu_hi = cfg["mu_plausibility_band_low"], cfg["mu_plausibility_band_high"]
        manifest["mu_plausibility"] = {
            "mu_front": front_fit["mu"], "mu_rear": rear_fit["mu"],
            "band_low": mu_lo, "band_high": mu_hi,
            "front_plausible": bool(mu_lo <= front_fit["mu"] <= mu_hi),
            "rear_plausible": bool(mu_lo <= rear_fit["mu"] <= mu_hi),
        }

    if lit:
        # hard-bound re-entries only, for the CS reset mark (author ruling
        # 2026-10-19: the CS sensitivity to the reset policy is general, so a
        # covariance-only reset no longer marks windows); in memory only, like
        # nis_full
        manifest["observer_reset_mask"] = final_result["hard_bound_mask"]
    manifest["beta_ekf"] = beta_ekf
    manifest["beta_ekf_with_fallback"] = final_result["beta_with_fallback"]  # production must use this, not beta_ekf
    manifest["nis_full"] = final_result["nis"]  # full-length, for modules.nis_gate.evaluate_gate
    manifest["base_mask"] = base_mask
    return manifest


def resolve_sideslip_beta(state, params, data, sideslip_source, csv_path=None):
    """Which beta a given sideslip_source produces. Lives here, not in the
    QThread, so it is testable without Qt (tests/test_auto_fit_wiring.py).

    Returns (beta, fit_manifest, gate_verdict, fallback_used, fallback_reason).
    fit_manifest: JSON-safe manifest, auto modes only. gate_verdict: from
    nis_gate.evaluate_gate, None outside auto modes or after a degenerate fit.
    Degenerate fit or gate "fail" -> kinematic beta, reason as text.
    An unknown sideslip_source is refused: it used to run kinematic
    silently, so a typo in the config shipped the fallback beta as if
    chosen. Never mutates params.
    """
    return resolve_sideslip_beta_and_cs_inputs(state, params, data, sideslip_source, csv_path)[:5]


def resolve_sideslip_beta_and_cs_inputs(state, params, data, sideslip_source, csv_path=None):
    """resolve_sideslip_beta plus what the CS estimate needs from fit L
    (default since 2026-10-20). Sixth element cs_inputs:
    {"cs_kwargs": keyword arguments for estimate_cornering_stiffness,
     "axle_fx": {"fx_f_N", "fx_r_N"} or None, "note": text or None}.
    Outside fit L both are empty / None, so the old path calls the CS
    estimate exactly as before. In fit L: CS on Fy / Fz with the reference
    band L_K_EDGE, and the observer-reset mark from hard-bound re-entries --
    dropped when the session ships kinematic beta, since then no observer
    state reached the slip angles."""
    beta, fit_manifest, gate_verdict, fallback_used, fallback_reason, raw = _resolve(
        state, params, data, sideslip_source, csv_path)
    cs_inputs = {"cs_kwargs": {}, "axle_fx": None, "note": None}
    # from the config, not the manifest: the early degenerate returns carry no mode
    if raw is not None and params.get("tyre_fit_auto", {}).get("tyre_fit_mode") == "literature_bounded":
        loads = raw.get("cs_axle_loads")
        if loads is None:
            # only a non-finite load input stops fit L before the loads exist;
            # the CS then stays on Fy, its ratio is unit-free either way
            cs_inputs["note"] = "CS on Fy (axle loads unavailable: " + str(raw.get("degenerate_reason")) + ")"
        else:
            cs_inputs["cs_kwargs"] = {"axle_loads": loads, "ref_band": L_K_EDGE}
            cs_inputs["axle_fx"] = raw.get("axle_fx")
        if not fallback_used and raw.get("observer_reset_mask") is not None:
            cs_inputs["cs_kwargs"]["reset_mask"] = raw["observer_reset_mask"]
    return beta, fit_manifest, gate_verdict, fallback_used, fallback_reason, cs_inputs


def _resolve(state, params, data, sideslip_source, csv_path):
    if sideslip_source not in SIDESLIP_SOURCES:
        raise ValueError(
            f"sideslip_source {sideslip_source!r} not recognised (expected one of {SIDESLIP_SOURCES}; "
            "the Dugoff sources ekf_pass_1 / ekf_auto_dugoff were removed 2026-10-17)"
        )

    if sideslip_source == "ekf_auto_pacejka":
        # both config-gated, defaults unchanged (free D, per-session)
        load_normalised = params.get("tyre_fit_auto", {}).get("load_normalised_fit_enabled", False)
        tyre_fit_mode = params.get("tyre_fit_auto", {}).get("tyre_fit_mode", "per_session")
        raw_fit_manifest = fit_session_pacejka(data, params, data_file_path=csv_path,
                                                load_normalised=load_normalised, tyre_fit_mode=tyre_fit_mode)
        fit_status = raw_fit_manifest.get("status")
        fallback_used = False
        fallback_reason = None
        gate_verdict = None
        if fit_status == "degenerate":
            fallback_used = True
            fallback_reason = f"fit status 'degenerate': {raw_fit_manifest.get('degenerate_reason')}"
            beta = estimate_sideslip(state, params)
        else:
            gate_verdict = evaluate_gate(
                raw_fit_manifest["nis_full"], raw_fit_manifest["base_mask"], params, state["sample_rate_hz"])
            if gate_verdict["verdict"] == "fail":
                fallback_used = True
                fallback_reason = (
                    f"NIS gate verdict 'fail' (health_score={gate_verdict['health_score']!r}, "
                    f"threshold_warn={gate_verdict['threshold_warn']})"
                )
                beta = estimate_sideslip(state, params)
            else:
                beta = raw_fit_manifest["beta_ekf_with_fallback"]
        fit_manifest = {
            k: v for k, v in raw_fit_manifest.items()
            if k not in ("beta_ekf", "beta_ekf_with_fallback", "nis_full", "base_mask", "plausibility_gate",
                         "observer_reset_mask", "cs_axle_loads", "axle_fx")
        }
        return beta, fit_manifest, gate_verdict, fallback_used, fallback_reason, raw_fit_manifest

    return estimate_sideslip(state, params), None, None, False, None, None
