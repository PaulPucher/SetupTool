# Stability analysis module for SetupTool.
# Pure Python/numpy/scipy. No Qt imports.
# Units: SI throughout (m, s, rad, N, Nm, kg).
# Cornering-stiffness (Module 4b) and yaw-moment-stability (Module 5)
# estimation after Werner (2021) S2.2.2-2.2.3 / S4.5.2 Eq. 4.3-4.4;
# effective-stiffness estimation adapted, see thesis_notes.md.

import numpy as np
from scipy.signal import butter, filtfilt
import json
from modules.geo import project_latlon_to_xy

PARAMETERS_PATH = "config/parameters.json"

# Method-defining constants (CLAUDE.md grounding rule): these fix what the
# estimator IS, not how it is tuned to this car/track, so they stay as named
# constants rather than config entries.
BUTTERWORTH_ORDER = 4  # standard 4th-order digital filter; defines roll-off shape, not a physical threshold
SPAN_WEIGHT_EXPONENT = 4  # steep smooth-step so a section only counts once its alpha span nears cs_min_slip_angle_span_rad
R2_WEIGHT_EXPONENT = 1  # linear R^2 blend between window- and section-slope estimates, no extra shaping


def load_parameters():
    with open(PARAMETERS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _butterworth_lowpass(data, cutoff_hz, sample_rate_hz, order=BUTTERWORTH_ORDER):
    nyq = 0.5 * sample_rate_hz
    normal_cutoff = cutoff_hz / nyq
    if normal_cutoff >= 1.0:
        return data
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)


def _highpass_filter(data, cutoff_hz, sample_rate_hz, order=BUTTERWORTH_ORDER):
    nyq = 0.5 * sample_rate_hz
    normal_cutoff = cutoff_hz / nyq
    if normal_cutoff >= 1.0:
        return data
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)


def _estimate_sample_rate(time_arr):
    dt = np.diff(time_arr)
    dt_median = np.median(dt)
    if dt_median <= 0:
        raise ValueError("Time array has non-positive intervals")
    return 1.0 / dt_median


# Tier B signal conditioning for the Module 4b CS_alpha blend (see
# thesis_notes.md "CS_ratio (cornering stiffness ratio)"): smooth-step
# weighting, monotonic-section splitting, and per-section OLS slopes are
# preprocessing on noisy measured data, not part of Werner's method itself.
def _smooth_weight(value, lower, upper, order):
    v = np.clip(value, lower, upper)
    rng = upper - lower
    if rng <= 0:
        return 0.0
    mid = (lower + upper) / 2.0
    if v <= mid:
        return 0.5 * (2.0 * (v - lower) / rng) ** order
    else:
        return 1.0 - 0.5 * (2.0 * (upper - v) / rng) ** order


def _find_monotonic_sections(alpha_filt):
    n = len(alpha_filt)
    if n < 2:
        return [(0, n)], np.zeros(n, dtype=int)
    d = np.diff(alpha_filt)
    sign = np.sign(d)
    for i in range(1, len(sign)):
        if sign[i] == 0:
            sign[i] = sign[i - 1]
    splits = np.where((sign[1:] != sign[:-1]) & (sign[1:] != 0) & (sign[:-1] != 0))[0] + 1
    section_starts = [0] + (splits + 1).tolist()
    section_ends = (splits + 1).tolist() + [n]
    sections = list(zip(section_starts, section_ends))
    section_id = np.zeros(n, dtype=int)
    for k, (s, e) in enumerate(sections):
        section_id[s:e] = k
    return sections, section_id


def _section_slopes(alpha, Fy, sections):
    n_sec = len(sections)
    slopes = np.full(n_sec, np.nan)
    spans = np.zeros(n_sec)
    for k, (s, e) in enumerate(sections):
        if e - s < 2:
            continue
        a = alpha[s:e]
        f = Fy[s:e]
        a_mean = np.mean(a)
        f_mean = np.mean(f)
        denom = np.sum((a - a_mean) ** 2)
        if denom < 1e-10:
            continue
        slopes[k] = np.sum((a - a_mean) * (f - f_mean)) / denom
        spans[k] = np.max(a) - np.min(a)
    return slopes, spans


def _compute_kerb_mask_from_az(az_g, threshold_g, baseline_g, dilation_samples):
    # Flag samples where vertical accel deviates more than threshold_g from baseline_g,
    # then dilate the mask by dilation_samples on each side to catch ringdown.
    if az_g is None:
        return None
    raw = np.abs(az_g - baseline_g) > threshold_g
    if dilation_samples <= 0:
        return raw
    # Simple symmetric dilation using a rolling-OR
    n = len(raw)
    out = raw.copy()
    for shift in range(1, dilation_samples + 1):
        out[shift:] |= raw[:-shift]
        out[:-shift] |= raw[shift:]
    return out


def prepare_vehicle_state(channels, params):
    vp = params["vehicle"]
    se = params["stability_estimation"]

    required = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_x"]
    for ch_name in required:
        ch = channels.get(ch_name)
        if ch is None or ch["quality"] in ("missing", "failed") or ch["time"] is None:
            return None

    t_ref = channels["ecu_speed"]["time"]
    sr = _estimate_sample_rate(t_ref)

    def interp_channel(ch_name):
        ch = channels.get(ch_name)
        if ch is None or ch["quality"] in ("missing", "failed") or ch["time"] is None:
            return None
        return np.interp(t_ref, ch["time"], ch["data"])

    v_kmh = interp_channel("ecu_speed")
    v_mps = v_kmh / 3.6

    yaw_rate_rpm = interp_channel("sclu_yaw_rate")
    yaw_rate_radps = yaw_rate_rpm * se["yaw_rate_to_radps"]

    steer_sw_deg = interp_channel("log_asteer")
    steer_sw_rad = steer_sw_deg * np.pi / 180.0
    i_s = vp["steering_ratio"]
    delta_f_rad = steer_sw_rad / i_s

    ay_mps2 = interp_channel("log_acc_y") * 9.81
    ax_mps2 = interp_channel("log_acc_x") * 9.81

    # Vertical accel (g) for kerb detection. Optional channel -- graceful degradation.
    az_g = None
    az_ch = channels.get("log_acc_z")
    if az_ch is not None and az_ch.get("quality") not in ("missing", "failed") and az_ch.get("time") is not None:
        az_g = np.interp(t_ref, az_ch["time"], az_ch["data"])

    kerb_mask = _compute_kerb_mask_from_az(
        az_g,
        threshold_g=se["kerb_z_deviation_threshold_g"],
        baseline_g=se["kerb_baseline_g"],
        dilation_samples=int(se["kerb_dilation_samples"]),
    )

    throttle = interp_channel("ecu_aps")
    brake_f = interp_channel("log_pbrake_f")
    gear = interp_channel("ecu_gear")

    moving_mask = v_mps > se["moving_speed_min_mps"]

    # GPS position (Level 3, optional). Used for apex_position_x/y_m via a
    # local equirectangular projection anchored at the first sample -- fine
    # at track scale, no need for a proper geodesic projection.
    gps_lat = interp_channel("log_gps_lat")
    gps_lon = interp_channel("log_gps_lon")
    gps_origin_lat = None
    gps_origin_lon = None
    if gps_lat is not None and gps_lon is not None:
        gps_origin_lat = float(gps_lat[0])
        gps_origin_lon = float(gps_lon[0])
    else:
        gps_lat = None
        gps_lon = None

    return {
        "time": t_ref,
        "sample_rate_hz": sr,
        "v_mps": v_mps,
        "yaw_rate_radps": yaw_rate_radps,
        "delta_f_rad": delta_f_rad,
        "steer_sw_rad": steer_sw_rad,
        "ay_mps2": ay_mps2,
        "ax_mps2": ax_mps2,
        "throttle_pct": throttle,
        "brake_f_bar": brake_f,
        "gear": gear,
        "moving_mask": moving_mask,
        "kerb_mask": kerb_mask,
        "az_g": az_g,
        "gps_lat": gps_lat,
        "gps_lon": gps_lon,
        "gps_origin_lat": gps_origin_lat,
        "gps_origin_lon": gps_origin_lon,
        "steering_ratio": i_s,
        "accuracy_level": {
            "speed": 1,
            "yaw_rate": 3,
            "steering_angle": 1,
            "lateral_acc": 1,
        }
    }


def estimate_sideslip(state, params):
    """Kinematic identity ay = v*(beta_dot + psi_dot) after
    Mitschke/Wallentowitz, Dynamik der Kraftfahrzeuge (single-track
    lateral kinematics), p. TBD, verify. Washout integration below is
    Tier B signal conditioning (drift correction), not part of the cited
    identity itself.
    """
    se = params["stability_estimation"]
    v = state["v_mps"]
    ay = state["ay_mps2"]
    yaw_rate = state["yaw_rate_radps"]
    sr = state["sample_rate_hz"]
    moving = state["moving_mask"]

    v_safe = np.where(moving, v, 1.0)
    beta_dot = np.where(moving, ay / v_safe - yaw_rate, 0.0)

    dt = 1.0 / sr
    beta_raw = np.cumsum(beta_dot) * dt

    beta = _highpass_filter(beta_raw, se["beta_washout_cutoff_hz"], sr)
    beta = np.where(moving, beta, 0.0)
    return beta


def estimate_slip_angles(state, beta, params):
    """Single-track slip-angle relations after Werner (2021) S2.1.1 /
    Milliken RCVD.
    """
    vp = params["vehicle"]
    se = params["stability_estimation"]

    v = state["v_mps"]
    yaw_rate = state["yaw_rate_radps"]
    delta_f = state["delta_f_rad"]
    moving = state["moving_mask"]
    sr = state["sample_rate_hz"]

    a = vp["cog_to_front_axle_m"]
    b = vp["cog_to_rear_axle_m"]

    v_x = v * np.cos(beta)
    v_y = v * np.sin(beta)
    v_x_safe = np.where(moving, v_x, 1.0)

    alpha_f = delta_f - np.arctan((v_y + a * yaw_rate) / v_x_safe)
    alpha_r = -np.arctan((v_y - b * yaw_rate) / v_x_safe)

    alpha_f = np.where(moving, alpha_f, 0.0)
    alpha_r = np.where(moving, alpha_r, 0.0)

    cutoff = se["cs_filter_cutoff_hz"]
    alpha_f_filt = _butterworth_lowpass(alpha_f, cutoff, sr)
    alpha_r_filt = _butterworth_lowpass(alpha_r, cutoff, sr)

    return {
        "alpha_f_raw": alpha_f,
        "alpha_r_raw": alpha_r,
        "alpha_f_filt": alpha_f_filt,
        "alpha_r_filt": alpha_r_filt,
    }


def estimate_lateral_forces(state, params):
    vp = params["vehicle"]
    se = params["stability_estimation"]
    sr = state["sample_rate_hz"]
    moving = state["moving_mask"]

    m = vp["mass_kg"]
    cw = vp["corner_weights"]
    W_total = cw["FL_kg"] + cw["FR_kg"] + cw["RL_kg"] + cw["RR_kg"]
    W_f = cw["FL_kg"] + cw["FR_kg"]
    W_r = cw["RL_kg"] + cw["RR_kg"]
    front_fraction = W_f / W_total
    rear_fraction = W_r / W_total

    Fy_total = m * state["ay_mps2"]
    Fy_f = np.where(moving, Fy_total * front_fraction, 0.0)
    Fy_r = np.where(moving, Fy_total * rear_fraction, 0.0)

    cutoff = se["cs_filter_cutoff_hz"]
    Fy_f_filt = _butterworth_lowpass(Fy_f, cutoff, sr)
    Fy_r_filt = _butterworth_lowpass(Fy_r, cutoff, sr)

    return {
        "Fy_f_raw": Fy_f,
        "Fy_r_raw": Fy_r,
        "Fy_f_filt": Fy_f_filt,
        "Fy_r_filt": Fy_r_filt,
        "front_fraction": front_fraction,
        "rear_fraction": rear_fraction,
        "accuracy_level": 1
    }


def estimate_cornering_stiffness(slip, forces, state, params):
    """Module 4b: effective cornering stiffness / CS ratio.

    After Werner (2021) S2.2.2-2.2.3. Effective-stiffness estimation is
    adapted (windowed regression from logged Fy/alpha in place of
    Werner's Pacejka-model evaluation) -- see thesis_notes.md.
    """
    se = params["stability_estimation"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask

    alpha_f = slip["alpha_f_filt"]
    alpha_r = slip["alpha_r_filt"]
    Fy_f = forces["Fy_f_filt"]
    Fy_r = forces["Fy_r_filt"]

    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    min_window = se["cs_min_window_samples"]

    def compute_cs_for_axle(alpha, Fy):
        n = len(alpha)
        C_window = np.full(n, np.nan)
        C_section = np.full(n, np.nan)
        C_alpha = np.full(n, np.nan)
        R2 = np.full(n, np.nan)
        CS_ratio = np.full(n, np.nan)
        C_linear_ref = np.nan

        sections, section_id = _find_monotonic_sections(alpha)
        sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)

        for i in range(min_window, n):
            if not moving[i]:
                continue

            start = i - min_window
            while start > 0:
                span = np.max(alpha[start:i]) - np.min(alpha[start:i])
                if span >= min_span:
                    break
                start -= 1

            window_alpha = alpha[start:i]
            window_Fy = Fy[start:i]
            if len(window_alpha) < min_window:
                continue

            alpha_mean = np.mean(window_alpha)
            Fy_mean = np.mean(window_Fy)
            denom = np.sum((window_alpha - alpha_mean) ** 2)
            if denom < 1e-10:
                continue

            C_w = np.sum((window_alpha - alpha_mean) * (window_Fy - Fy_mean)) / denom
            Fy_hat = C_w * window_alpha + (Fy_mean - C_w * alpha_mean)
            ss_res = np.sum((window_Fy - Fy_hat) ** 2)
            ss_tot = np.sum((window_Fy - Fy_mean) ** 2)
            R2_i = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0

            C_window[i] = C_w
            R2[i] = R2_i

            sec_ids_in_window = np.unique(section_id[start:i])
            weights = []
            slopes = []
            for k in sec_ids_in_window:
                slope_k = sec_slopes[k]
                span_k = sec_spans[k]
                if np.isnan(slope_k):
                    continue
                w_k = _smooth_weight(span_k, 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
                if w_k <= 0:
                    continue
                weights.append(w_k)
                slopes.append(slope_k)

            if weights:
                w_arr = np.array(weights)
                s_arr = np.array(slopes)
                C_s = float(np.sum(w_arr * s_arr) / np.sum(w_arr))
                C_section[i] = C_s
            else:
                C_s = np.nan

            if not np.isnan(C_s):
                w_r2 = _smooth_weight(R2_i, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
                C_alpha[i] = w_r2 * C_w + (1.0 - w_r2) * C_s
            else:
                C_alpha[i] = C_w

            window_max_abs_alpha = np.max(np.abs(window_alpha))
            if window_max_abs_alpha < linear_thresh:
                C_linear_ref = C_alpha[i]

            if not np.isnan(C_linear_ref) and C_linear_ref > 0:
                CS_ratio[i] = min(C_alpha[i] / C_linear_ref, 1.0)

        return C_alpha, C_window, C_section, R2, CS_ratio

    C_f, Cw_f, Cs_f, R2_f, CS_ratio_f = compute_cs_for_axle(alpha_f, Fy_f)
    C_r, Cw_r, Cs_r, R2_r, CS_ratio_r = compute_cs_for_axle(alpha_r, Fy_r)

    return {
        "C_alpha_f": C_f,
        "C_alpha_r": C_r,
        "C_window_f": Cw_f,
        "C_window_r": Cw_r,
        "C_section_f": Cs_f,
        "C_section_r": Cs_r,
        "R2_f": R2_f,
        "R2_r": R2_r,
        "CS_ratio_f": CS_ratio_f,
        "CS_ratio_r": CS_ratio_r,
    }


def estimate_yaw_moment_stability(state, beta, params):
    """Module 5: yaw moment stability dMz/dbeta.

    After Werner (2021) S4.5.2 Eq. 4.3/4.4 (Mz = Iz*psidd + D_psi*psid).
    D_psi term not yet computed (no wheel-load sensor); see
    thesis_notes.md "Completing Werner Eq. 4.3" and WP5b.
    """
    vp = params["vehicle"]
    se = params["stability_estimation"]

    t = state["time"]
    sr = state["sample_rate_hz"]
    v = state["v_mps"]
    yaw_rate = state["yaw_rate_radps"]
    delta_f = state["delta_f_rad"]
    ax = state["ax_mps2"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask

    Iz = vp["yaw_inertia_kgm2"]
    cutoff_hz = se["yaw_accel_filter_cutoff_hz"]
    window_s = se["stability_regression_window_s"]
    beta_span_min = se["stability_min_beta_span_rad"]

    n = len(t)

    psi_ddot_raw = np.gradient(yaw_rate, t)
    psi_ddot_filt = _butterworth_lowpass(psi_ddot_raw, cutoff_hz, sr)
    Mz_inertial = Iz * psi_ddot_filt

    half_window = int(round(window_s * sr / 2))
    min_samples = se["stability_min_window_samples"]

    stability_radps = np.full(n, np.nan)
    stability_valid = np.zeros(n, dtype=bool)

    for i in range(n):
        if not moving[i]:
            continue

        lo = max(0, i - half_window)
        hi = min(n, i + half_window + 1)
        win_mask = moving[lo:hi]
        if win_mask.sum() < min_samples:
            continue

        b_win = beta[lo:hi][win_mask]
        d_win = delta_f[lo:hi][win_mask]
        v_win = v[lo:hi][win_mask]
        ax_win = ax[lo:hi][win_mask]
        m_win = Mz_inertial[lo:hi][win_mask]

        if (b_win.max() - b_win.min()) < beta_span_min:
            continue

        X = np.column_stack([
            np.ones_like(b_win),
            b_win,
            d_win,
            v_win,
            ax_win,
        ])

        try:
            coeffs, *_ = np.linalg.lstsq(X, m_win, rcond=None)
        except np.linalg.LinAlgError:
            continue

        c_beta = coeffs[1]
        stability_radps[i] = c_beta
        stability_valid[i] = True

    stability_observed = stability_radps * (np.pi / 180.0)

    return {
        "yaw_accel_filtered_radps2": psi_ddot_filt,
        "mz_inertial_Nm": Mz_inertial,
        "stability_observed_Nm_per_deg": stability_observed,
        "stability_observed_radps": stability_radps,
        "stability_valid": stability_valid,
        "iz_used_kgm2": Iz,
    }


def summarise_corners(corners, cs, stab, state, lap_filter=None, apex_half_window_samples=None):
    if apex_half_window_samples is None:
        apex_half_window_samples = load_parameters()["stability_estimation"]["apex_half_window_samples"]
    t = state["time"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")

    cs_f = cs["CS_ratio_f"]
    cs_r = cs["CS_ratio_r"]
    stab_obs = stab["stability_observed_Nm_per_deg"]
    stab_valid = stab["stability_valid"]

    phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]

    def _stats(arr):
        valid = arr[~np.isnan(arr)]
        n = len(valid)
        if n == 0:
            return {"median": float("nan"), "p25": float("nan"),
                    "p75": float("nan"), "n": 0}
        return {
            "median": float(np.median(valid)),
            "p25": float(np.percentile(valid, 25)),
            "p75": float(np.percentile(valid, 75)),
            "n": int(n),
        }

    def _phase_slice(start_t, end_t, is_apex=False):
        if end_t < start_t:
            return slice(0, 0)
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if is_apex and hi <= lo:
            # Apex is a single instant -- expand to +/- N samples
            centre = lo
            lo = max(0, centre - apex_half_window_samples)
            hi = min(len(t), centre + apex_half_window_samples + 1)
        return slice(lo, hi)

    gps_lat = state.get("gps_lat")
    gps_lon = state.get("gps_lon")
    gps_origin_lat = state.get("gps_origin_lat")
    gps_origin_lon = state.get("gps_origin_lon")

    out = []
    for c in corners:
        if lap_filter is not None and c["lap_number"] not in lap_filter:
            continue

        apex_x = None
        apex_y = None
        if gps_lat is not None:
            apex_idx = int(np.searchsorted(t, c["apex_time"]))
            apex_idx = min(max(apex_idx, 0), len(t) - 1)
            apex_x, apex_y = project_latlon_to_xy(
                gps_lat[apex_idx], gps_lon[apex_idx], gps_origin_lat, gps_origin_lon
            )
            apex_x = float(apex_x)
            apex_y = float(apex_y)

        corner_summary = {
            "lap_number": c["lap_number"],
            "corner_number": c["corner_number"],
            "speed_class": c["speed_class"],
            "apex_time": c["apex_time"],
            "apex_speed": c["apex_speed"],
            "apex_lateral_g": c.get("apex_lateral_g"),
            "method": c.get("method"),
            "warnings": c.get("warnings", []),
            "apex_position_x_m": apex_x,
            "apex_position_y_m": apex_y,
            "stable_corner_id": c.get("stable_corner_id"),
            "phases": {},
        }

        for phase in phase_keys:
            start_t, end_t = c["segments"][phase]
            sl = _phase_slice(start_t, end_t, is_apex=(phase == "apex_3"))

            if sl.stop > sl.start:
                phase_moving = moving[sl]
                idx = np.where(phase_moving)[0] + sl.start
                # Kerb fraction: of the moving samples in this phase,
                # how many were flagged as kerb-affected
                if kerb_mask is not None:
                    n_phase_moving = int(phase_moving.sum())
                    if n_phase_moving > 0:
                        kerb_in_phase = int(kerb_mask[sl][phase_moving].sum())
                        kerb_fraction = float(kerb_in_phase / n_phase_moving)
                    else:
                        kerb_fraction = 0.0
                else:
                    kerb_fraction = 0.0
            else:
                idx = np.array([], dtype=int)
                kerb_fraction = 0.0

            n_samples = len(idx)
            if n_samples == 0:
                corner_summary["phases"][phase] = {
                    "n_samples": 0,
                    "valid_fraction_stab": 0.0,
                    "kerb_fraction": kerb_fraction,
                    "cs_ratio_f": _stats(np.array([])),
                    "cs_ratio_r": _stats(np.array([])),
                    "stability_observed_Nm_per_deg": _stats(np.array([])),
                }
                continue

            stab_valid_phase = stab_valid[idx]
            valid_fraction_stab = float(stab_valid_phase.sum() / n_samples)

            corner_summary["phases"][phase] = {
                "n_samples": int(n_samples),
                "valid_fraction_stab": valid_fraction_stab,
                "kerb_fraction": kerb_fraction,
                "cs_ratio_f": _stats(cs_f[idx]),
                "cs_ratio_r": _stats(cs_r[idx]),
                "stability_observed_Nm_per_deg": _stats(stab_obs[idx]),
            }

        out.append(corner_summary)

    return out