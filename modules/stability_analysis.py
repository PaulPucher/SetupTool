# Vehicle state, sideslip, slip angles, axle forces, Fz, cornering
# stiffness (Module 4b) and yaw-moment stability (Module 5). SI units.
# CS_ratio framework: Werner 2021; stiffness estimation adapted (windowed
# regression on logged Fy/alpha). Module 5 estimator (yaw_stability.py)
# after the chair performance_analysis tooling (internal).

import functools
import numpy as np
from scipy.signal import butter, filtfilt
import json
from modules.geo import project_latlon_to_xy
from modules.yaw_stability import calculate_filtered_yaw_acceleration, calculate_observed_stability

PARAMETERS_PATH = "config/parameters.json"
CAR_DATA_PATH = "config/car_data.json"

# Cache version for the persisted analysis payload. Bump when
# summarise_corners' numbers or the payload shape change for the same
# input; not for read/render-only changes. Mismatch = no cache.
#   2: accuracy_cap / resolved_* fields
#   3: fz / fy_norm stat blocks per phase
#   4: bracket_start_m / bracket_end_m per corner
#   5: sideslip_source
#   6: auto-fit modes, fit_manifest, gate_verdict, fallback_*
#   7: ls_ratio_f/r stat blocks (display only)
#   8: apex_region per corner; grid_rate_hz in the payload
ANALYSIS_SCHEMA_VERSION = 8

# method-defining, not tunables
BUTTERWORTH_ORDER = 4  # roll-off shape
SPAN_WEIGHT_EXPONENT = 4  # steep smooth-step: a section counts once its span nears cs_min_slip_angle_span_rad
R2_WEIGHT_EXPONENT = 1  # plain linear R^2 blend


@functools.lru_cache(maxsize=1)  # re-read only after restart
def load_parameters():
    with open(PARAMETERS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


@functools.lru_cache(maxsize=1)  # same as load_parameters
def load_car_data():
    # car_data.json is local-only (gitignored) -> missing/bad = None.
    # Never print its contents.
    try:
        with open(CAR_DATA_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, UnicodeDecodeError):
        return None


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


# channels whose native rate limits the grid rate. ecu_speed excluded --
# it may be slower and gets upsampled (see prepare_vehicle_state).
CS_CHAIN_FAST_CHANNELS = ["sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z", "lap_distance"]


def _resolve_grid_rate(channels, params):
    """Grid rate = min(target_sample_rate_hz, slowest native rate among
    CS_CHAIN_FAST_CHANNELS). Refuses below min_sample_rate_hz, naming the
    binding channel. An absent channel can't bind the rate.
    """
    se = params["stability_estimation"]
    target_rate = se["target_sample_rate_hz"]
    min_rate = se["min_sample_rate_hz"]

    native_rates = {}
    for ch_name in CS_CHAIN_FAST_CHANNELS:
        ch = channels.get(ch_name)
        if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
            continue
        native_rates[ch_name] = _estimate_sample_rate(ch["time"])

    if not native_rates:
        raise ValueError(
            "No CS-chain fast channel (sclu_yaw_rate, log_asteer, log_acc_y, log_acc_z, "
            "lap_distance) has usable timing data -- cannot determine a common grid rate."
        )

    binding_channel = min(native_rates, key=native_rates.get)
    cs_chain_capability = native_rates[binding_channel]

    if cs_chain_capability < min_rate:
        raise ValueError(
            f"Sample rate too low: {binding_channel} measured {cs_chain_capability:.1f} Hz, "
            f"below the {min_rate:.0f} Hz floor. Every estimator window in this pipeline was "
            f"validated at {min_rate:.0f}-{target_rate:.0f} Hz only -- analysis is refused rather "
            "than silently run at the wrong scale. See config/parameters.json "
            "stability_estimation.min_sample_rate_hz."
        )

    grid_rate = min(target_rate, cs_chain_capability)
    status = f"{grid_rate:.0f} Hz" if grid_rate >= target_rate else f"{grid_rate:.0f} Hz (channel-limited, {binding_channel})"
    return grid_rate, status


# CS_alpha blend preprocessing (smooth-step weights, monotonic sections,
# section OLS) -- signal conditioning, not part of Werner's method
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


def _normalize_lap_distance_to_metres(data, unit_raw):
    # lap_distance unit differs per export (Dubai ft, Paul Ricard m). A wrong
    # guess scales every distance quantity by 3.28 -> check unit_raw.
    if unit_raw == "ft":
        return data * 0.3048
    if unit_raw == "m":
        return data
    raise ValueError(
        f"lap_distance unit {unit_raw!r} not recognised (expected 'ft' or 'm') -- "
        "add explicit handling before trusting this export's distance values"
    )


def _interp_lap_distance_guarded(t_ref, ld_time, ld_data_m):
    # lap_distance resets each lap -> interpolating across the reset invents a
    # position; those samples go NaN. Needed only because we resample s_m.
    # [neutral engineering]
    # ld_data_m already in metres.
    s_m = np.interp(t_ref, ld_time, ld_data_m)

    reset_after = np.zeros(len(ld_time), dtype=bool)
    reset_after[:-1] = np.diff(ld_data_m) < 0
    bracket_lo = np.clip(np.searchsorted(ld_time, t_ref, side="right") - 1, 0, len(ld_time) - 1)
    return np.where(reset_after[bracket_lo], np.nan, s_m)


def _build_inout_lap_mask(t_ref, laps):
    # Module 5 excludes in/out laps regardless of the UI lap_filter: cold
    # tyres break the cross-lap stationarity the regression assumes (same
    # idea as the kerb mask). [domain improvement]
    mask = np.zeros(len(t_ref), dtype=bool)
    for lap in laps or []:
        if lap.get("is_outlap") or lap.get("is_inlap"):
            mask |= (t_ref >= lap["start_time"]) & (t_ref <= lap["end_time"])
    return mask


def _compute_kerb_mask_from_az(az_g, threshold_g, baseline_g, dilation_samples):
    # threshold catches the hit, dilation the ringdown around it
    if az_g is None:
        return None
    raw = np.abs(az_g - baseline_g) > threshold_g
    if dilation_samples <= 0:
        return raw
    # OR with itself shifted +/- 1..dilation_samples
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

    # Synthetic even grid at the resolved rate over ecu_speed's time span.
    # ecu_speed is 50 Hz, the CS chain 100 Hz
    # (diagnostics/inspect_native_channel_rates.py) -- resampling onto
    # ecu_speed threw half of it away. ecu_speed upsampled linearly: speed
    # is inertia-limited, can't jump between samples.
    sr, grid_rate_status = _resolve_grid_rate(channels, params)
    ecu_speed_t = channels["ecu_speed"]["time"]
    # linspace, not arange -- arange dropped the last sample
    n_grid = int(round((ecu_speed_t[-1] - ecu_speed_t[0]) * sr)) + 1
    t_ref = np.linspace(ecu_speed_t[0], ecu_speed_t[-1], n_grid)

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
    # steering_ratio_table only present after accuracy resolution at L4;
    # otherwise the plain constant. np.interp clamps outside the table
    # (+/-291 deg, beyond anything driven).
    steering_ratio_table = vp.get("steering_ratio_table")
    if steering_ratio_table is not None:
        i_s = np.interp(steer_sw_deg, steering_ratio_table["angle_deg"], steering_ratio_table["ratio"])
    else:
        i_s = vp["steering_ratio"]
    delta_f_rad = steer_sw_rad / i_s

    ay_mps2 = interp_channel("log_acc_y") * 9.81
    ax_mps2 = interp_channel("log_acc_x") * 9.81

    # az [g] for kerb detection; None if the channel is unusable
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

    # GPS (L3, optional), local projection anchored at the first sample
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

    # track distance for Module 5; None if lap_distance unusable
    s_m = None
    ld_ch = channels.get("lap_distance")
    if ld_ch is not None and ld_ch.get("quality") not in ("missing", "failed") and ld_ch.get("time") is not None:
        ld_data_m = _normalize_lap_distance_to_metres(ld_ch["data"], ld_ch.get("unit_raw"))
        s_m = _interp_lap_distance_guarded(t_ref, ld_ch["time"], ld_data_m)

    return {
        "time": t_ref,
        "s_m": s_m,
        "sample_rate_hz": sr,
        "grid_rate_status": grid_rate_status,
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
            "speed": params["accuracy_levels"]["speed"]["level"],
            "yaw_rate": params["accuracy_levels"]["yaw_rate"]["level"],
            "steering_angle": params["accuracy_levels"]["steering_angle"]["level"],
            "lateral_acc": params["accuracy_levels"]["lateral_acc"]["level"],
        }
    }


def estimate_sideslip(state, params):
    """Kinematic identity ay = v*(beta_dot + psi_dot) (Rajamani ch. 2).
    Washout integration = drift correction, signal conditioning only.
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


def _interp_circular_deg(t_query, t_src, deg_src):
    """Angle interpolation via sin/cos + atan2 -- linear breaks at the 0/360
    wrap. Returns rad in (-pi, pi].
    """
    rad_src = np.radians(deg_src)
    sin_i = np.interp(t_query, t_src, np.sin(rad_src))
    cos_i = np.interp(t_query, t_src, np.cos(rad_src))
    return np.arctan2(sin_i, cos_i)


def estimate_sideslip_gps(state, channels, params):
    """beta_gps = GPS course over ground - heading. Validation only, not used
    in the pipeline (diagnostics/inspect_beta_gps_validation.py).
    Heading isn't logged -> integrated yaw rate, re-anchored to course at
    low-slip samples.

    Rotation: course is compass (clockwise from North); corr(d course/dt,
    -yaw_rate) = +0.95 -> yaw_rate is counter-clockwise positive. Checked,
    not assumed.
    GPS latency: course lags yaw rate ~0.32 s -> course read
    gps_course_latency_s ahead.
    Anchors: smoothed |ay| < gps_course_anchor_max_ay_g for at least
    gps_course_anchor_min_duration_s; one anchor per run (midpoint, median
    offset). Raw per-sample gate gave only sub-0.1 s runs.
    Drift allocated by accumulated |yaw rate| between anchors, not time --
    the gyro shortfall scales with rotation, i.e. with corners.
    """
    se = params["stability_estimation"]
    t_ref = state["time"]
    sr = state["sample_rate_hz"]
    moving = state["moving_mask"]

    course_ch = channels.get("log_gps_course")
    if course_ch is None or course_ch.get("quality") in ("missing", "failed") or course_ch.get("time") is None:
        return np.full_like(t_ref, np.nan)

    latency_s = se.get("gps_course_latency_s", 0.0)
    course_rad = _interp_circular_deg(t_ref + latency_s, course_ch["time"], course_ch["data"])

    # open-loop heading, sign per the rotation finding above
    psi_gyro_dot = -state["yaw_rate_radps"]
    dt = 1.0 / sr
    psi_gyro = np.cumsum(psi_gyro_dot) * dt

    ay_g = np.abs(state["ay_mps2"]) / 9.81
    smooth_win = max(1, int(round(se["gps_course_anchor_smooth_window_s"] * sr)))
    if smooth_win > 1:
        kernel = np.ones(smooth_win) / smooth_win
        ay_g_smooth = np.convolve(ay_g, kernel, mode="same")
    else:
        ay_g_smooth = ay_g
    candidate = moving & (ay_g_smooth < se["gps_course_anchor_max_ay_g"])

    d = np.diff(candidate.astype(int))
    starts = list(np.where(d == 1)[0] + 1)
    ends = list(np.where(d == -1)[0] + 1)
    if candidate[0]:
        starts = [0] + starts
    if candidate[-1]:
        ends = ends + [len(candidate)]

    min_run_samples = se["gps_course_anchor_min_duration_s"] * sr
    raw_offset = np.arctan2(np.sin(course_rad - psi_gyro), np.cos(course_rad - psi_gyro))

    anchor_times = []
    anchor_offsets = []
    for s_idx, e_idx in zip(starts, ends):
        if (e_idx - s_idx) < min_run_samples:
            continue
        anchor_times.append(float(t_ref[(s_idx + e_idx) // 2]))
        anchor_offsets.append(float(np.median(np.unwrap(raw_offset[s_idx:e_idx]))))

    if len(anchor_times) == 0:
        # no anchor window at all -> drift unresolved, beta_gps unusable
        return np.full_like(t_ref, np.nan)

    anchor_offsets_unwrapped = np.unwrap(np.array(anchor_offsets))
    # rotation clock (cumulative |yaw rate|) instead of time; np.interp holds
    # end values outside the anchored range
    yaw_abs_cum = np.cumsum(np.abs(state["yaw_rate_radps"])) * dt
    anchor_rotation = np.interp(anchor_times, t_ref, yaw_abs_cum)
    drift_offset = np.interp(yaw_abs_cum, anchor_rotation, anchor_offsets_unwrapped)

    psi_hat = psi_gyro + drift_offset
    beta_gps = np.arctan2(np.sin(course_rad - psi_hat), np.cos(course_rad - psi_hat))
    beta_gps = np.where(moving, beta_gps, np.nan)
    return beta_gps


def estimate_slip_angles(state, beta, params):
    """Single-track slip angles, Werner S2.2.3 convention."""
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
    """Module 4a: 2-DOF planar balance (Milliken RCVD), as in the chair
    performance_analysis tooling (internal):
        Fy_f = m*ay*front_fraction + Iz*psidd/wheelbase,  Fy_r = m*ay - Fy_f
    psidd = raw gradient of yaw rate, not Module 5's smoothed one -- the CS
    Butterworth downstream would otherwise double-filter.
    Still Level 1: Iz and the static fractions are L1.
    """
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

    Iz = vp["yaw_inertia_kgm2"]
    wheelbase = vp["wheelbase_m"]
    psidd_raw = np.gradient(state["yaw_rate_radps"], state["time"])

    Fy_total = m * state["ay_mps2"]
    Fy_f_full = Fy_total * front_fraction + Iz * psidd_raw / wheelbase
    Fy_r_full = Fy_total - Fy_f_full
    Fy_f = np.where(moving, Fy_f_full, 0.0)
    Fy_r = np.where(moving, Fy_r_full, 0.0)

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
        "accuracy_level": params["accuracy_levels"]["lateral_force_split"]["level"]
    }


def estimate_vertical_loads(state, forces, params, channels=None, car_data=None):
    """Axle and per-wheel Fz + diagnostic fy_*_norm_N = Fy_filt / Fz.
    As in the chair performance_analysis tooling (internal): per-axle
    lateral-transfer split, no roll-stiffness apportionment.
    fy_*_norm_N: display only, not a classifier input.
    Level 1 (cog height, track widths, aero are placeholders).

    vertical_load_source == "measured" with channels + car_data -> damper
    cascade (wheel_loads: measured -> reconstructed -> this static model).
    Axle totals then re-summed from the wheels. "static" never touches
    wheel_loads.
    """
    vp = params["vehicle"]
    aero = vp["aero"]

    m = vp["mass_kg"]
    g = 9.81
    wb = vp["wheelbase_m"]
    l_f_cog = vp["cog_to_front_axle_m"]
    l_r_cog = vp["cog_to_rear_axle_m"]
    h_cog = vp["cog_height_m"]

    # 1. static distribution (positive down)
    fz_static_f_N = m * g * l_r_cog / wb
    fz_static_r_N = m * g * l_f_cog / wb

    # 2. aero (positive = downforce)
    rho = aero["air_density_kgm3"]
    cl = aero["lift_coeff"]
    a_aero = aero["cross_track_area_m2"]
    x_cp_cog = aero["diff_cog_x_m"]
    fz_aero_total_N = -0.5 * rho * state["v_mps"] ** 2 * a_aero * cl
    dfz_aero_f_N = fz_aero_total_N * (l_r_cog - x_cp_cog) / wb
    dfz_aero_r_N = fz_aero_total_N * (l_f_cog + x_cp_cog) / wb

    # 3. longitudinal transfer
    dfz_long_transfer_N = m * state["ax_mps2"] * h_cog / wb

    fz_f_N = fz_static_f_N + dfz_aero_f_N - dfz_long_transfer_N
    fz_r_N = fz_static_r_N + dfz_aero_r_N + dfz_long_transfer_N

    # 4. per-wheel: independent per-axle lateral transfer
    front_track = vp["track_width_front_m"]
    rear_track = vp["track_width_rear_m"]
    lateral_transfer_front = m * state["ay_mps2"] * h_cog / front_track
    lateral_transfer_rear = m * state["ay_mps2"] * h_cog / rear_track

    fz_fl_N = fz_f_N / 2 - lateral_transfer_front / 2
    fz_fr_N = fz_f_N / 2 + lateral_transfer_front / 2
    fz_rl_N = fz_r_N / 2 - lateral_transfer_rear / 2
    fz_rr_N = fz_r_N / 2 + lateral_transfer_rear / 2

    # damper cascade, static model as innermost fallback
    vertical_load_source = params["stability_estimation"].get("vertical_load_source", "static")
    fz_source_per_sample = None
    c_session = None  # session-fitted aero coefficient, level 2; None on the static path
    if vertical_load_source == "measured" and channels is not None and car_data is not None:
        from modules.wheel_loads import (
            estimate_wheel_loads_from_dampers, estimate_session_corrected_axle_totals,
            combine_with_reconstruction_and_fallback, CORNERS as WHEEL_CORNERS,
        )
        static_fallback_fz = {"fl": fz_fl_N, "fr": fz_fr_N, "rl": fz_rl_N, "rr": fz_rr_N}
        damper_result = estimate_wheel_loads_from_dampers(state, channels, params, car_data)
        any_damper_valid = any(np.any(damper_result[c]["valid"]) for c in WHEEL_CORNERS)
        if any_damper_valid:
            session_corrected = estimate_session_corrected_axle_totals(state, damper_result, params)
            fz_axle_totals = {"fz_f_N": session_corrected["fz_f_N"], "fz_r_N": session_corrected["fz_r_N"]}
            c_session = session_corrected["c_session_N_per_mps2"]
        else:
            # no real damper sample anywhere -> skip the fit, it would average
            # nothing and nothing could be reconstructed anyway (both real
            # sessions have damper data; Dubai only lacks RR)
            fz_axle_totals = {"fz_f_N": fz_f_N, "fz_r_N": fz_r_N}
        combined = combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz)
        fz_fl_N = combined["fl"]["fz_N"]
        fz_fr_N = combined["fr"]["fz_N"]
        fz_rl_N = combined["rl"]["fz_N"]
        fz_rr_N = combined["rr"]["fz_N"]
        # axle total = sum of the cascade wheels, so axle and wheel values always agree
        fz_f_N = fz_fl_N + fz_fr_N
        fz_r_N = fz_rl_N + fz_rr_N
        fz_source_per_sample = {c: combined[c]["source"] for c in WHEEL_CORNERS}
        vertical_load_source = "measured"
    else:
        vertical_load_source = "static"

    # 5. normalised force diagnostic
    fy_f_norm_N = forces["Fy_f_filt"] / fz_f_N
    fy_r_norm_N = forces["Fy_r_filt"] / fz_r_N

    return {
        "fz_f_N": fz_f_N,
        "fz_r_N": fz_r_N,
        "fz_fl_N": fz_fl_N,
        "fz_fr_N": fz_fr_N,
        "fz_rl_N": fz_rl_N,
        "fz_rr_N": fz_rr_N,
        "fy_f_norm_N": fy_f_norm_N,
        "fy_r_norm_N": fy_r_norm_N,
        "accuracy_level_axle": params["accuracy_levels"]["vertical_load_split"]["level"],
        "accuracy_level_wheel": params["accuracy_levels"]["per_wheel_load_split"]["level"],
        "vertical_load_source_used": vertical_load_source,
        "vertical_load_source_per_sample": fz_source_per_sample,
        "c_session_N_per_mps2": c_session,
    }


def resolve_cs_min_window_samples(params, sample_rate_hz):
    """cs_min_window_s [s] -> samples at this file's rate, floored. The chair's
    10-sample default was implicitly 100 Hz. Shared by the estimator and all
    window reconstructions.
    """
    se = params["stability_estimation"]
    return max(se["cs_min_window_samples_floor"], int(round(se["cs_min_window_s"] * sample_rate_hz)))


def reconstruct_cs_window_start(alpha, i, min_window, min_span, s_m=None, max_window_m=None):
    """Start index of the window compute_cs_for_axle used at index i -- for
    trace-window highlights and diagnostics. Reconstruction only, never
    recomputes CS. Matched the live loop to 1e-6.

    min_window = sample count (resolve_cs_min_window_samples).
    s_m/max_window_m cap growth by track distance, so the cap means the same
    in slow and fast corners. Either missing, or NaN s_m at the boundary ->
    no cap; only safe on indices with a finite CS_ratio.
    Same incremental running max/min as compute_cs_for_axle -- change both
    together.
    """
    start = i - min_window
    s_i = s_m[i - 1] if (s_m is not None and max_window_m is not None) else None
    if s_i is not None and not np.isfinite(s_i):
        s_i = None
    if start > 0:
        window_max = np.max(alpha[start:i])
        window_min = np.min(alpha[start:i])
    while start > 0:
        span = window_max - window_min
        if span >= min_span:
            break
        if s_i is not None:
            s_start = s_m[start]
            if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                break
        start -= 1
        window_max = np.maximum(window_max, alpha[start])
        window_min = np.minimum(window_min, alpha[start])
    return max(start, 0)


def estimate_cornering_stiffness(slip, forces, state, params):
    """Module 4b: effective cornering stiffness and CS_ratio (Werner 2021).
    Stiffness by windowed regression on logged Fy/alpha instead of Werner's
    Pacejka evaluation.
    """
    se = params["stability_estimation"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    s_m = state.get("s_m")

    alpha_f = slip["alpha_f_filt"]
    alpha_r = slip["alpha_r_filt"]
    Fy_f = forces["Fy_f_filt"]
    Fy_r = forces["Fy_r_filt"]

    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
    max_window_m = se["cs_max_window_m"]

    def compute_cs_for_axle(alpha, Fy):
        n = len(alpha)
        C_window = np.full(n, np.nan)
        C_section = np.full(n, np.nan)
        C_alpha = np.full(n, np.nan)
        R2 = np.full(n, np.nan)
        CS_ratio = np.full(n, np.nan)
        C_linear_ref = np.nan
        # linear reference in effect per sample (CS_ratio denominator), for the
        # tyre-curve audit plot
        C_linear_ref_arr = np.full(n, np.nan)

        sections, section_id = _find_monotonic_sections(alpha)
        sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)

        for i in range(min_window, n):
            if not moving[i]:
                continue

            # widen until both floors clear, capped at max_window_m of track -- a
            # flat-alpha stretch can't pull in unrelated sections.
            # Same loop as reconstruct_cs_window_start; keep in sync.
            start = i - min_window
            s_i = s_m[i - 1] if s_m is not None else None
            if s_i is not None and not np.isfinite(s_i):
                s_i = None
            if start > 0:
                window_max = np.max(alpha[start:i])
                window_min = np.min(alpha[start:i])
            while start > 0:
                span = window_max - window_min
                if span >= min_span:
                    break
                if s_i is not None:
                    s_start = s_m[start]
                    if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                        break
                start -= 1
                window_max = np.maximum(window_max, alpha[start])
                window_min = np.minimum(window_min, alpha[start])

            window_alpha = alpha[start:i]
            window_Fy = Fy[start:i]
            achieved_span = np.max(window_alpha) - np.min(window_alpha)
            if achieved_span < min_span:
                continue  # span floor not reached -> no signal

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

            C_linear_ref_arr[i] = C_linear_ref

        return C_alpha, C_window, C_section, R2, CS_ratio, C_linear_ref_arr

    C_f, Cw_f, Cs_f, R2_f, CS_ratio_f, Clr_f = compute_cs_for_axle(alpha_f, Fy_f)
    C_r, Cw_r, Cs_r, R2_r, CS_ratio_r, Clr_r = compute_cs_for_axle(alpha_r, Fy_r)

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
        "C_linear_ref_f": Clr_f,
        "C_linear_ref_r": Clr_r,
    }


def estimate_yaw_moment_stability(state, beta, params, laps=None):
    """Module 5: dMz/dbeta. Target relation Mz = Iz*psidd + D_psi*psid (Werner
    S4.5.2 Eq. 4.3); D_psi not computed yet. Estimator in yaw_stability.py,
    after the chair performance_analysis tooling (internal).
    Front vs rear saturation = controllability vs stability loss (Hoffman et
    al. 2008, sec. 2); saddle-node framing as motivation only (Ono et al.
    1998), no bifurcation analysis.

    Exclusions (moving, kerb, in/out lap) applied here by NaN-ing samples;
    the estimator itself runs unmasked. [neutral engineering]
    In/out laps always excluded, independent of the UI lap_filter -- cold
    tyres corrupt the cross-lap pooling. [domain improvement]
    The chair's time-anchored fallback (no s_m) not ported: different
    estimator, the s-grid thresholds don't apply -> no verdict instead.
    """
    vp = params["vehicle"]
    se = params["stability_estimation"]

    t = state["time"]
    sr = state["sample_rate_hz"]
    v = state["v_mps"]
    yaw_rate = state["yaw_rate_radps"]
    delta_f = state["delta_f_rad"]
    ax = state["ax_mps2"]
    az_g = state.get("az_g")
    s_m = state.get("s_m")
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    moving = moving & ~_build_inout_lap_mask(t, laps)

    Iz = vp["yaw_inertia_kgm2"]

    yaw_accel_filt = calculate_filtered_yaw_acceleration(
        yaw_rate, t, sr, se["yaw_stability_accel_window_s"]
    )
    Mz_inertial = Iz * yaw_accel_filt

    n = len(t)
    if s_m is None:
        stability_observed = np.full(n, np.nan)
        stability_valid = np.zeros(n, dtype=bool)
    else:
        az_mps2 = az_g * 9.81 if az_g is not None else None
        stability_observed, stability_valid, _diagnostics = calculate_observed_stability(
            s_m=s_m,
            beta_rad=beta,
            delta_f_rad=delta_f,
            v_mps=v,
            ax_mps2=ax,
            az_mps2=az_mps2,
            mz_inertial_Nm=Mz_inertial,
            valid_mask=moving,
            grid_step_m=se["yaw_stability_grid_step_m"],
            window_m=se["yaw_stability_window_m"],
            min_samples=se["yaw_stability_min_samples"],
            ridge=se["yaw_stability_ridge"],
            min_beta_std_rad=se["yaw_stability_min_beta_std_rad"],
        )

    return {
        "yaw_accel_filtered_radps2": yaw_accel_filt,
        "mz_inertial_Nm": Mz_inertial,
        "stability_observed_Nm_per_deg": stability_observed,
        "stability_valid": stability_valid,
        "iz_used_kgm2": Iz,
    }


def summarise_corners(corners, cs, stab, state, fz=None, ls=None, lap_filter=None,
                       apex_half_window_samples=None, cs_phase_min_valid_samples=None,
                       cs_apex_region_half_length_m=None, stab_phase_no_braking_floor_bar=None,
                       ls_phase_min_valid_samples=None):
    # fz and ls optional: each adds its stat blocks per phase, omitted ->
    # older summary shape. stability_observed goes through _gate_stab_stat
    # (sample-count gate + no-braking check for entry_1_brake).
    if (apex_half_window_samples is None or cs_phase_min_valid_samples is None
            or cs_apex_region_half_length_m is None or stab_phase_no_braking_floor_bar is None
            or ls_phase_min_valid_samples is None):
        se_defaults = load_parameters()["stability_estimation"]
        if apex_half_window_samples is None:
            apex_half_window_samples = se_defaults["apex_half_window_samples"]
        if cs_phase_min_valid_samples is None:
            cs_phase_min_valid_samples = se_defaults["cs_phase_min_valid_samples"]
        if cs_apex_region_half_length_m is None:
            cs_apex_region_half_length_m = se_defaults["cs_apex_region_half_length_m"]
        if stab_phase_no_braking_floor_bar is None:
            stab_phase_no_braking_floor_bar = se_defaults["stab_phase_no_braking_floor_bar"]
        if ls_phase_min_valid_samples is None:
            ls_phase_min_valid_samples = se_defaults["ls_phase_min_valid_samples"]
    t = state["time"]
    s_m = state.get("s_m")
    brake_f_bar = state.get("brake_f_bar")
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")

    cs_f = cs["CS_ratio_f"]
    cs_r = cs["CS_ratio_r"]
    stab_obs = stab["stability_observed_Nm_per_deg"]
    stab_valid = stab["stability_valid"]
    fz_f = fz["fz_f_N"] if fz is not None else None
    fz_r = fz["fz_r_N"] if fz is not None else None
    fy_f_norm = fz["fy_f_norm_N"] if fz is not None else None
    fy_r_norm = fz["fy_r_norm_N"] if fz is not None else None
    ls_f = ls["LS_ratio_f"] if ls is not None else None
    ls_r = ls["LS_ratio_r"] if ls is not None else None

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

    def _gate_cs_stat(stat):
        # too few finite samples -> NaN, not a median of one or two outliers
        if stat["n"] < cs_phase_min_valid_samples:
            return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        return stat

    def _gate_ls_stat(stat):
        # same gate for LS, own floor (ls_phase_min_valid_samples) -- different
        # noise than CS
        if stat["n"] < ls_phase_min_valid_samples:
            return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        return stat

    def _gate_stab_stat(stat, phase, brake_vals):
        # stability: same sample-count gate as CS (cs floor reused), plus
        # entry_1_brake with no real braking (max pressure under the floor) ->
        # no signal (v3 C7/C15)
        if stat["n"] < cs_phase_min_valid_samples:
            return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        if phase == "entry_1_brake" and brake_f_bar is not None:
            if len(brake_vals) == 0 or float(np.nanmax(brake_vals)) < stab_phase_no_braking_floor_bar:
                return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        return stat

    def _apex_region_idx(c):
        # distance band around the apex instead of the fixed 11-sample apex_3
        # slice (CS reads). Limited to this instance's own time span first --
        # s_m resets every lap, a pure distance band would grab other laps.
        if s_m is None:
            return np.array([], dtype=int)
        apex_s = c.get("apex_lap_distance_m")
        if apex_s is None or apex_s != apex_s:
            return np.array([], dtype=int)
        valid_segs = [seg for seg in c["segments"].values() if seg[1] >= seg[0]]
        if not valid_segs:
            return np.array([], dtype=int)
        lo = int(np.searchsorted(t, min(seg[0] for seg in valid_segs), side="left"))
        hi = int(np.searchsorted(t, max(seg[1] for seg in valid_segs), side="right"))
        if hi <= lo:
            return np.array([], dtype=int)
        within = moving[lo:hi] & (np.abs(s_m[lo:hi] - apex_s) <= cs_apex_region_half_length_m)
        return np.where(within)[0] + lo

    def _phase_slice(start_t, end_t, is_apex=False):
        if end_t < start_t:
            return slice(0, 0)
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if is_apex and hi <= lo:
            # apex is one instant -> +/- N samples
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
            "bracket_start_m": c.get("bracket_start_m"),
            "bracket_end_m": c.get("bracket_end_m"),
            "phases": {},
        }

        for phase in phase_keys:
            start_t, end_t = c["segments"][phase]
            sl = _phase_slice(start_t, end_t, is_apex=(phase == "apex_3"))

            if sl.stop > sl.start:
                phase_moving = moving[sl]
                idx = np.where(phase_moving)[0] + sl.start
                # share of moving samples flagged as kerb
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
                if fz is not None:
                    corner_summary["phases"][phase]["fz_f_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fz_r_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fy_f_norm_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fy_r_norm_N"] = _stats(np.array([]))
                if ls is not None:
                    corner_summary["phases"][phase]["ls_ratio_f"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["ls_ratio_r"] = _stats(np.array([]))
                continue

            stab_valid_phase = stab_valid[idx]
            valid_fraction_stab = float(stab_valid_phase.sum() / n_samples)

            corner_summary["phases"][phase] = {
                "n_samples": int(n_samples),
                "valid_fraction_stab": valid_fraction_stab,
                "kerb_fraction": kerb_fraction,
                "cs_ratio_f": _gate_cs_stat(_stats(cs_f[idx])),
                "cs_ratio_r": _gate_cs_stat(_stats(cs_r[idx])),
                "stability_observed_Nm_per_deg": _gate_stab_stat(
                    _stats(stab_obs[idx]), phase,
                    brake_f_bar[idx] if brake_f_bar is not None else np.array([])
                ),
            }
            if fz is not None:
                corner_summary["phases"][phase]["fz_f_N"] = _stats(fz_f[idx])
                corner_summary["phases"][phase]["fz_r_N"] = _stats(fz_r[idx])
                corner_summary["phases"][phase]["fy_f_norm_N"] = _stats(fy_f_norm[idx])
                corner_summary["phases"][phase]["fy_r_norm_N"] = _stats(fy_r_norm[idx])
            if ls is not None:
                corner_summary["phases"][phase]["ls_ratio_f"] = _gate_ls_stat(_stats(ls_f[idx]))
                corner_summary["phases"][phase]["ls_ratio_r"] = _gate_ls_stat(_stats(ls_r[idx]))

        apex_idx = _apex_region_idx(c)
        corner_summary["apex_region"] = {
            "n_samples": int(apex_idx.size),
            "cs_ratio_f": _gate_cs_stat(_stats(cs_f[apex_idx])),
            "cs_ratio_r": _gate_cs_stat(_stats(cs_r[apex_idx])),
        }

        out.append(corner_summary)

    return out