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
#   9: no shape change -- estimator inputs changed: Q-CAL-1 mask, Q-CAL-2
#      NBR zero entries (2026-10-03). Standing rule: any estimator-input
#      change made outside Settings bumps this, since the caches do not
#      see config edits.
#  10: no shape change -- estimator input changed (2026-10-04): the Level-4
#      steering conversion reads the wheel angle from the manufacturer table
#      (mean of both front wheels) instead of dividing by its local ratio.
#  11: estimator input changed (2026-10-04): speed window on the Pacejka fit
#      population (tyre_fit_auto.fit_speed_window_pct); fit manifest gains
#      speed_window.
#  12: (2026-10-05) balance key added - measured understeer gradient,
#      Level 2; and summarise_corners' CS statistics now skip kerb-affected
#      (near-strike) samples, the population the thresholds derive on.
#  13: (2026-10-06) events key added - limit events (modules/limit_events.py),
#      display / record only; no estimator input and no summary changed.
#  14: (2026-10-07) CS stat blocks gain n_kerb_excluded (finite samples left
#      out as kerb-affected), so the per-phase table can say "no clean data"
#      instead of a blank; display only, no value changes.
#  15: (2026-10-11) estimator change: Pacejka EKF divergence re-entry without
#      kinematic injection (NIS window -> keep state, P0; hard bound -> beta 0),
#      tyre-curve plausibility gate, R selected on the ay exceedance with a
#      sign-check guard and default-R fallback, gross trigger on ay only, soft
#      yaw ceiling. Payload: fit_manifest gains marginal_reason and
#      r_sweep.selection / guard_trace (grid points carry ay_in_band and
#      sign_fraction instead of both_in_band); degenerate_reason / fallback_reason
#      texts changed; every degenerate status now carries a reason.
#  owed (final re-derivation chain, registry R3): the 2026-10-16/17 rebuild
#      changed results under 15; payload drops "balance" (K readout removed,
#      so the 12 key is no longer written) and resolved_levels.
#      understeer_gradient, gains "estimator_settings" and the NIS gate's
#      n_windows_skipped_low_coverage.
#  16: (2026-10-20) default switch: fit L (tyre_fit_mode literature_bounded)
#      is the default path -- tyre fit, observer, CS on Fy / Fz with the
#      [5.5, 35] per rad reference band and the hard-bound reset mark all
#      change values; thresholds re-derived on it. Payload: under fit L the
#      events record holds the floor events (CS below the -0.53 pure-lateral
#      floor, measured attributes, "line" key) instead of the curve-floor
#      limit events; fit_manifest carries the fit-L fields (peak "none",
#      mu_source; no mu_extrapolated / onset_coverage). Migration: older payloads are recomputed on
#      open (schema mismatch), nothing is converted in place. The 15 owed
#      note above is settled by this bump too.
#  17: (2026-10-20) estimator inputs changed after 16 was committed (schema
#      bump rule): NIS gate thresholds re-derived on fit L (0.104 / 0.011,
#      config), CS linear reference gated to [0.5, 2] x the session median
#      (R26 source fix, code), cs_margin 0.01 and thresholds re-confirmed. No
#      payload shape change; stored payloads under 16 are recomputed on open.
#  18: (2026-10-21) estimator input changed (schema bump rule): the throttle
#      range gate widens to [0, 110] % (the pedal reads 104-106 % at full
#      throttle); on 7 of 14 development sessions the channel had been graded
#      failed, so every entry_1_brake phase was zero-length -- those phases
#      now hold samples. Corner warnings carry channel-missing warnings into
#      the canonical instances. No payload shape change.
#  19: (2026-10-21) estimator input changed (schema bump rule): aerodynamic
#      drag enters the CoG longitudinal force (drag_coeff 0.50 x frontal area
#      2.0 m^2, Level 1, literature) -- fit-L braking filter, Eq. 9 axle split
#      and the observer's front force frame move. No payload shape change.
#  20: (2026-10-22) payload shape change from the pruning package (P1b), no
#      value change: the longitudinal-stiffness blocks are gone, fit_manifest
#      loses seven legacy config echoes (load_normalised, nis_window_s,
#      nis_chi2_bound, nis_flag_fraction, divergence_trigger, cd_drift,
#      cd_threshold), and fit L is the only fit mode, so the in-memory fit
#      result always carries cs_axle_loads. Goldens differ only in _meta;
#      no threshold re-derivation (values bit-identical).
#  21: (2026-10-22) payload shape change, recommendation overhaul: the
#      stored analysis carries the setup engine's data / engineer / session
#      layers ("setup_engine"); the classifier follows ruling D2 (limiting =
#      CS < -cs_boundary_margin). Chain values (fit, gate, summaries, floor
#      events) unchanged; verdicts and recommendations change by design.
#  22: (2026-10-23) work order 4e, config edits outside Settings: the
#      boundary margin is per axle (cs_boundary_margin_front 0.062 / _rear
#      0.076, was 0.01); the setup engine ranks preference before effort,
#      ignores preference cells the mechanism table does not support,
#      counts INST as agreeing with oversteer feedback, reports per-lap cell
#      shares in the session layer and reads the ABS map from log_abs_pos.
#      Chain values unchanged; verdicts and recommendations change.
#  23: (2026-10-23) work order 4f, config edits outside Settings: ride
#      heights become M4 levers (high speed), OS under braking takes the
#      braking-instability rows, bias OS also at turn-in, rear camber APX
#      names INST, camber step 0.2 deg, damper effort minutes, TC stepped
#      only inside 2-11. Chain values unchanged; recommendations change.
#  24: (2026-10-23) work order 4g: payload shape change -- each corner of the
#      setup layers carries a mechanical setup recommendation and a separate
#      cockpit option (cockpit, cockpit_ranked, cockpit_conflicts,
#      cockpit_at_limit); the front ride height has a floor only. Chain values
#      unchanged; recommendations change.
#  25: (2026-10-23) work order 4h: payload shape change and config edits
#      outside Settings -- each setup layer carries a session setup
#      recommendation from the corner vote (session_recommendation, config
#      setup_engine.session_vote), mechanisms.json rows gain an axle field and
#      the limiting-end class replaces the row order as the last per-corner
#      rank key. Chain values unchanged; recommendations change.
ANALYSIS_SCHEMA_VERSION = 25

# method-defining, not tunables
BUTTERWORTH_ORDER = 4  # roll-off shape
SPAN_WEIGHT_EXPONENT = 4  # steep smooth-step: a section counts once its span nears cs_min_slip_angle_span_rad
R2_WEIGHT_EXPONENT = 1  # plain linear R^2 blend


@functools.lru_cache(maxsize=1)  # re-read only after restart
def load_parameters():
    with open(PARAMETERS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def thresholds_calibrated(params):
    """True when the classification thresholds were derived for the active
    estimator path: the configured sideslip source and, for the auto fit,
    the configured tyre_fit_mode (thresholds encode the estimator; author
    ruling 2026-10-20). Config comparison only -- a run-time kinematic
    fallback does not change it; the status line shows that."""
    cl = params["classification"]
    source = params["stability_estimation"].get("sideslip_source", "kinematic")
    if source != cl.get("thresholds_calibrated_for_sideslip_source", "kinematic"):
        return False
    if source == "ekf_auto_pacejka":
        mode = params.get("tyre_fit_auto", {}).get("tyre_fit_mode", "per_session")
        return mode == cl.get("thresholds_calibrated_for_tyre_fit_mode", "per_session")
    return True


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


# A line with intercept is only over-determined from 3 points on; the chair
# estimator uses the same floor (end - start > 1, both ends inclusive).
SECTION_MIN_SAMPLES = 3


def _section_slope_and_span(a, f):
    # OLS slope and slip-angle span of one (possibly clipped) section;
    # (nan, 0) when it carries no slope information.
    if len(a) < SECTION_MIN_SAMPLES:
        return np.nan, 0.0
    a_mean = np.mean(a)
    f_mean = np.mean(f)
    denom = np.sum((a - a_mean) ** 2)
    if denom < 1e-10:
        return np.nan, 0.0
    return np.sum((a - a_mean) * (f - f_mean)) / denom, np.max(a) - np.min(a)


def _section_slopes(alpha, Fy, sections):
    # Whole-section values; compute_cs_for_axle reuses them only for sections
    # lying completely inside its window and re-fits the clipped edge ones.
    n_sec = len(sections)
    slopes = np.full(n_sec, np.nan)
    spans = np.zeros(n_sec)
    for k, (s, e) in enumerate(sections):
        slopes[k], spans[k] = _section_slope_and_span(alpha[s:e], Fy[s:e])
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


def _build_non_analysis_lap_mask(t_ref, laps):
    # Module 5 pools analysis laps only (finding #32): beyond the in/out laps
    # the slow laps (past valid_lap_max_ratio, e.g. traffic or a cool-down
    # lap) also break the cross-lap stationarity the regression assumes.
    # Same lap set as the CS summaries and the tyre fit. No lap list -> no
    # lap exclusion, as before. [domain improvement]
    mask = np.zeros(len(t_ref), dtype=bool)
    if not laps:
        return mask
    mask[:] = True
    for lap in laps:
        if lap.get("is_valid_for_analysis"):
            mask &= ~((t_ref >= lap["start_time"]) & (t_ref <= lap["end_time"]))
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


def _seconds_to_samples(duration_s, sample_rate_hz):
    return max(0, int(round(duration_s * sample_rate_hz)))


def _kerb_smear_zone(kerb_mask, grade_samples):
    # The 2 Hz zero-phase CS filter runs over the strike before the mask is
    # applied, so the strike leaks into unmasked neighbours on both sides
    # (Q-CAL-1, diagnostics/inspect_kerb_filter_decay.py (script at tag pre-prune-2026-10-22)). Zone = unmasked
    # samples within grade_samples of a mask sample. It feeds the
    # near_strike grade mark (together with the mask, R1), bars a window from
    # setting the CS linear reference and leaves the tyre-fit base mask with
    # the kerb mask (R6). The CS regression itself still runs over it.
    # [domain improvement: the chair tooling has no kerb treatment]
    if kerb_mask is None:
        return None
    n = len(kerb_mask)
    c = np.concatenate(([0], np.cumsum(kerb_mask, dtype=np.int64)))
    idx = np.arange(n)
    near = (c[np.minimum(idx + grade_samples + 1, n)] - c[np.maximum(idx - grade_samples, 0)]) > 0
    return near & ~kerb_mask


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
    # L4 (table present after accuracy resolution): the wheel angle comes
    # straight from the manufacturer map, no ratio division -- the table's
    # ratio column is the local slope, not sw / wheel. np.interp clamps at
    # the rack end stop (+/-291 deg). L1: the plain constant.
    steering_wheel_table = vp.get("steering_wheel_table")
    if steering_wheel_table is not None:
        delta_f_deg = np.interp(steer_sw_deg, steering_wheel_table["sw_deg"], steering_wheel_table["wheel_deg"])
        delta_f_rad = np.radians(delta_f_deg)
        # overall ratio kept for read-outs; undefined at centre -> constant
        near_centre = np.abs(delta_f_deg) < 1e-3
        i_s = np.where(near_centre, vp["steering_ratio"],
                       steer_sw_deg / np.where(near_centre, 1.0, delta_f_deg))
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
        dilation_samples=_seconds_to_samples(se["kerb_dilation_s"], sr),
    )
    kerb_smear_zone = _kerb_smear_zone(kerb_mask, _seconds_to_samples(se["kerb_smear_grade_s"], sr))

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
        "kerb_smear_zone": kerb_smear_zone,
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
    """Module 4a: 2-DOF planar balance (Milliken RCVD ch. 5 (sec. 5.7);
    Rajamani sec. 2.3 (Eqs. 2.21/2.22)), as in the chair
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


def apex_instant_slice(t, apex_t, half_window_samples):
    """The apex_3 window: the apex is one instant, read as the first grid
    sample at or after it +/- half_window_samples (11 samples at N = 5).
    Always this width -- the old phase slice took [lo, hi) first and only
    widened when it was empty, so an apex landing exactly on a grid sample
    got a 1-sample window (finding #12). Shared by summarise_corners, the
    tyre-fit h2 check and the TPMS apex read-out.
    """
    centre = int(np.searchsorted(t, apex_t, side="left"))
    return slice(max(0, centre - half_window_samples), min(len(t), centre + half_window_samples + 1))


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


def estimate_cornering_stiffness(slip, forces, state, params, laps, reset_mask=None, axle_loads=None,
                                 ref_band=None, ref_relative=None):
    """Module 4b: effective cornering stiffness and CS_ratio (Werner 2021).
    Stiffness by windowed regression on logged Fy/alpha instead of Werner's
    Pacejka evaluation.

    Linear reference (CS_ratio denominator): set only by a clean window --
    no sample of the regression window [start, i) in the kerb mask or the
    smear zone, none off an analysis lap -- whose stiffness lies inside the
    tyre-fit stiffness band (R26/R27). A disqualified window leaves the
    previous reference in force. Before, the moving/kerb test looked at the
    newest sample only, so a window reaching back over a strike could set
    the reference and every CS after it divided by a kerb-inflated value
    (PR C16 lap 51). Until the first clean window no CS is computed; those
    samples are counted (n_no_reference_*), never zero-filled. laps = the
    parsed lap list (analysis-lap test and the R11 lap-boundary guard);
    empty -> neither, as in Module 5. [domain improvement: the chair tooling
    has no kerb treatment]

    reset_mask (fit L, Q-FIT-LIT 2026-10-19): samples where the shipped
    observer reset its state at the hard bound (beta set to zero). A window
    [start, i) holding one is marked observer_reset_affected_*: the reset
    breaks the state continuity the windowed regression assumes, the same
    reasoning as the R11 lap-boundary guard. Covariance-only resets do not
    mark (author ruling 2026-10-19: the CS sensitivity to the reset policy
    is general, not specific to reset windows). Marked only, never deleted;
    None (the per_session path, or a session that fell back to kinematic
    beta) adds no key.

    axle_loads / ref_band (fit L, author ruling 2026-10-19, implemented
    2026-10-20): with axle_loads ({"fz_f_N", "fz_r_N"}, the measured axle
    loads after the fit-L 2 Hz zero-phase filter) the force input is Fy / Fz
    per axle, sample by sample -- cornering stiffness is defined at constant
    vertical load (Milliken, load-normalised lateral force), and the window
    regression otherwise reads a load change inside the window as a slope.
    The Fy definition is unchanged (only divided); filter, window,
    segmentation and R^2 blend are unchanged, and the reference is
    normalised the same way, so the ratio stays C / C_ref. ref_band replaces
    the reference plausibility band (fit L: tyre_fit_auto.L_K_EDGE, 5.5-35
    per rad -- the "about 6 to about 30" of Pacejka sec. 1.2.1 read at its
    stated precision, the same reading as the D4-L g2 check, 2026-10-20). Either given -> the output gains the reference-candidate slopes
    and counts (ref_candidates_*, n_ref_candidates_*, n_ref_band_excluded_*).
    [declared adaptation beyond the chair estimator, which uses Fy in N;
    enabled by the measured axle loads]

    ref_relative (fit L, R26 ruling 2026-10-20; tyre_fit_auto.L_REF_RELATIVE):
    (lo, hi) -- a band candidate sets the reference only within lo..hi x the
    median of ALL band candidates of that axle in the session. The median is
    taken in a first pass over the whole session, so it does not depend on
    the order of the samples; the reference is then set in a second pass. A
    rejected candidate leaves the previous reference in force (no CS value
    is deleted). The output gains n_ref_relative_low_* / _high_* and
    ref_relative_median_*. None: the gate is off (the per_session path).
    """
    se = params["stability_estimation"]
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    s_m = state.get("s_m")
    # The regression itself still runs over the smear zone (the chair
    # window, unchanged); the zone only marks near_strike and disqualifies
    # a window from setting the reference.
    smear_zone = state.get("kerb_smear_zone")
    # R1: the grade mark covers the mask as well as the zone -- a window lying
    # entirely inside a kerb run (i the first sample after it) holds no zone
    # sample and would otherwise pass as clean
    affected = None
    if kerb_mask is not None or smear_zone is not None:
        affected = np.zeros(len(state["moving_mask"]), dtype=bool)
        if kerb_mask is not None:
            affected |= kerb_mask
        if smear_zone is not None:
            affected |= smear_zone
    affected_cum = None if affected is None else np.concatenate(([0], np.cumsum(affected, dtype=np.int64)))
    reset_cum = None if reset_mask is None else np.concatenate(([0], np.cumsum(reset_mask, dtype=np.int64)))
    n = len(state["moving_mask"])
    # R11: lap id per sample from the lap table (-1 outside every lap); the s_m
    # cap stops widening at a distance reset, but the minimum window does not,
    # and s_m need not reset exactly at the lap table's boundary
    lap_id = np.full(n, -1, dtype=np.int64)
    for lap in laps or []:
        lap_id[(state["time"] >= lap["start_time"]) & (state["time"] <= lap["end_time"])] = lap["lap_number"]
    off_analysis = _build_non_analysis_lap_mask(state["time"], laps) if laps else np.zeros(n, dtype=bool)
    ref_blocked = off_analysis.copy()
    if kerb_mask is not None:
        ref_blocked |= kerb_mask
    if smear_zone is not None:
        ref_blocked |= smear_zone
    ref_blocked_cum = np.concatenate(([0], np.cumsum(ref_blocked, dtype=np.int64)))

    alpha_f = slip["alpha_f_filt"]
    alpha_r = slip["alpha_r_filt"]
    Fy_f = forces["Fy_f_filt"]
    Fy_r = forces["Fy_r_filt"]
    if axle_loads is not None:
        Fy_f = Fy_f / axle_loads["fz_f_N"]
        Fy_r = Fy_r / axle_loads["fz_r_N"]

    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    min_window = resolve_cs_min_window_samples(params, state["sample_rate_hz"])
    max_window_m = se["cs_max_window_m"]
    # Band rule (R26/R27, author ruling 2026-10-17): the reference is the
    # linear axle stiffness, the same quantity the D4 plausibility gate bounds
    # on the fitted curve (B*C*D, Bechtloff Fig. 7.2 magnitudes), so the same
    # config band is read here rather than duplicated. Needed because beta is
    # estimated: a low-excitation window's slope is ill-determined (Bechtloff,
    # identifiability) and can set a reference far below the session level,
    # or zero / negative. The chair does not bound its reference because it
    # works with measured beta. [domain improvement]
    ref_min = params["tyre_fit_auto"]["stiffness_min_npr"]
    ref_max = params["tyre_fit_auto"]["stiffness_max_npr"]
    if ref_band is not None:
        ref_min, ref_max = ref_band
    report_ref = axle_loads is not None or ref_band is not None

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
        near_strike = np.zeros(n, dtype=bool)
        reset_mark = np.zeros(n, dtype=bool)
        # stiffness computed, but no clean reference set yet -> no CS
        no_reference = np.zeros(n, dtype=bool)
        n_straddle = 0
        ref_candidates = []
        # first pass: per computed sample its window start and whether it is
        # a reference candidate; the reference itself is set in the second
        # pass, after the session median of the band candidates is known
        computed = np.zeros(n, dtype=bool)
        win_start = np.zeros(n, dtype=np.int64)
        candidate = np.zeros(n, dtype=bool)

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
            # R11: a window over a lap boundary mixes two traversals -> no
            # value, counted (a guard, not an assert)
            if lap_id[start] != lap_id[i - 1]:
                n_straddle += 1
                continue

            C_w = np.sum((window_alpha - alpha_mean) * (window_Fy - Fy_mean)) / denom
            Fy_hat = C_w * window_alpha + (Fy_mean - C_w * alpha_mean)
            ss_res = np.sum((window_Fy - Fy_hat) ** 2)
            ss_tot = np.sum((window_Fy - Fy_mean) ** 2)
            R2_i = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0

            C_window[i] = C_w
            R2[i] = R2_i

            # Sections clipped to the window, as in the chair estimator: a
            # section reaching past either window edge is re-fitted on its
            # in-window part only, and its weight comes from that part's span.
            # Before, whole-session sections leaked data from outside the
            # window into the section estimate (finding #26).
            weights = []
            slopes = []
            for k in range(section_id[start], section_id[i - 1] + 1):
                s, e = sections[k]
                lo, hi = max(s, start), min(e, i)
                if lo == s and hi == e:
                    slope_k, span_k = sec_slopes[k], sec_spans[k]
                else:
                    slope_k, span_k = _section_slope_and_span(alpha[lo:hi], Fy[lo:hi])
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

            computed[i] = True
            win_start[i] = start
            window_max_abs_alpha = np.max(np.abs(window_alpha))
            if window_max_abs_alpha < linear_thresh and ref_blocked_cum[i] == ref_blocked_cum[start]:
                candidate[i] = True
                if report_ref:
                    ref_candidates.append(C_alpha[i])

        # outside the band the previous reference is held, as for a
        # disqualified window (Fix 0); NaN compares False and never sets
        with np.errstate(invalid="ignore"):
            accept = candidate & (C_alpha >= ref_min) & (C_alpha <= ref_max)
        rel = {"low": 0, "high": 0, "median": float("nan")}
        if ref_relative is not None and accept.any():
            med = float(np.median(C_alpha[accept]))
            low = accept & (C_alpha < ref_relative[0] * med)
            high = accept & (C_alpha > ref_relative[1] * med)
            rel = {"low": int(low.sum()), "high": int(high.sum()), "median": med}
            accept &= ~(low | high)

        # second pass, in sample order: set the reference, then the ratio
        for i in np.flatnonzero(computed):
            start = win_start[i]
            if accept[i]:
                C_linear_ref = C_alpha[i]
            if np.isnan(C_linear_ref):
                no_reference[i] = True
            elif C_linear_ref > 0:
                CS_ratio[i] = min(C_alpha[i] / C_linear_ref, 1.0)
                # same [start, i) window the regression used
                if affected_cum is not None:
                    near_strike[i] = affected_cum[i] > affected_cum[start]
                if reset_cum is not None:
                    reset_mark[i] = reset_cum[i] > reset_cum[start]
            C_linear_ref_arr[i] = C_linear_ref

        return (C_alpha, C_window, C_section, R2, CS_ratio, C_linear_ref_arr, near_strike, no_reference,
                n_straddle, reset_mark, np.asarray(ref_candidates), rel)

    C_f, Cw_f, Cs_f, R2_f, CS_ratio_f, Clr_f, ns_f, nr_f, nx_f, rm_f, rc_f, rel_f = compute_cs_for_axle(alpha_f, Fy_f)
    C_r, Cw_r, Cs_r, R2_r, CS_ratio_r, Clr_r, ns_r, nr_r, nx_r, rm_r, rc_r, rel_r = compute_cs_for_axle(alpha_r, Fy_r)

    out = {
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
        # Q-CAL-1 grade mark (window holds a kerb-mask or smear-zone sample,
        # R1), in memory only (no schema change). Read by summarise_corners,
        # which drops marked samples from the verdict statistics, by the
        # limit-event kerb label, the trace display and the thesis reader
        "near_strike_f": ns_f,
        "near_strike_r": ns_r,
        # Fix 0 report, in memory only: samples with a stiffness but no clean
        # reference yet, in total and on analysis laps
        "no_reference_f": nr_f,
        "no_reference_r": nr_r,
        "n_no_reference_f": int(nr_f.sum()),
        "n_no_reference_r": int(nr_r.sum()),
        "n_no_reference_analysis_f": int((nr_f & ~off_analysis).sum()),
        "n_no_reference_analysis_r": int((nr_r & ~off_analysis).sum()),
        # R11 guard count, in memory only: windows refused for straddling a lap boundary
        "n_lap_straddle_f": nx_f,
        "n_lap_straddle_r": nx_r,
    }
    if reset_mask is not None:
        out["observer_reset_affected_f"] = rm_f
        out["observer_reset_affected_r"] = rm_r
    if report_ref:
        for ax, rc in (("f", rc_f), ("r", rc_r)):
            out[f"ref_candidates_{ax}"] = rc
            out[f"n_ref_candidates_{ax}"] = int(rc.size)
            out[f"n_ref_band_excluded_{ax}"] = int(((rc < ref_min) | (rc > ref_max)).sum())
    if ref_relative is not None:
        for ax, rel in (("f", rel_f), ("r", rel_r)):
            out[f"n_ref_relative_low_{ax}"] = rel["low"]
            out[f"n_ref_relative_high_{ax}"] = rel["high"]
            out[f"ref_relative_median_{ax}"] = rel["median"]
    return out


def estimate_yaw_moment_stability(state, beta, params, laps=None):
    """Module 5: dMz/dbeta. Target relation Mz = Iz*psidd + D_psi*psid (Werner
    S4.5.2 Eq. 4.3); D_psi not computed yet. Estimator in yaw_stability.py,
    after the chair performance_analysis tooling (internal).
    Front vs rear saturation = controllability vs stability loss (Hoffman et
    al. 2008, sec. 2); saddle-node framing as motivation only (Ono et al.
    1998), no bifurcation analysis.

    Exclusions (moving, kerb, non-analysis lap) applied here by NaN-ing
    samples; the estimator itself runs unmasked. [neutral engineering]
    Only analysis laps are pooled, independent of the UI lap_filter -- in/out
    laps (cold tyres) and slow laps corrupt the cross-lap pooling. [domain
    improvement]
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
    moving = moving & ~_build_non_analysis_lap_mask(t, laps)

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


def summarise_corners(corners, cs, stab, state, fz=None, lap_filter=None,
                       apex_half_window_samples=None, cs_phase_min_valid_samples=None,
                       cs_apex_region_half_length_m=None, stab_phase_no_braking_floor_bar=None):
    # fz optional: adds its stat blocks per phase, omitted -> older summary
    # shape. stability_observed goes through _gate_stab_stat (sample-count
    # gate + no-braking check for entry_1_brake).
    if (apex_half_window_samples is None or cs_phase_min_valid_samples is None
            or cs_apex_region_half_length_m is None or stab_phase_no_braking_floor_bar is None):
        se_defaults = load_parameters()["stability_estimation"]
        if apex_half_window_samples is None:
            apex_half_window_samples = se_defaults["apex_half_window_samples"]
        if cs_phase_min_valid_samples is None:
            cs_phase_min_valid_samples = se_defaults["cs_phase_min_valid_samples"]
        if cs_apex_region_half_length_m is None:
            cs_apex_region_half_length_m = se_defaults["cs_apex_region_half_length_m"]
        if stab_phase_no_braking_floor_bar is None:
            stab_phase_no_braking_floor_bar = se_defaults["stab_phase_no_braking_floor_bar"]
    t = state["time"]
    s_m = state.get("s_m")
    brake_f_bar = state.get("brake_f_bar")
    moving = state["moving_mask"]
    kerb_mask = state.get("kerb_mask")

    # Classification reads the CLEAN population: kerb-affected (near-strike)
    # samples are left out of the CS statistics, the population the
    # thresholds were derived on (2026-10-05). The CS arrays themselves keep
    # every sample; only the summary skips them.
    raw_f, raw_r = cs["CS_ratio_f"], cs["CS_ratio_r"]
    mark_f = np.broadcast_to(np.asarray(cs.get("near_strike_f", False), dtype=bool), raw_f.shape)
    mark_r = np.broadcast_to(np.asarray(cs.get("near_strike_r", False), dtype=bool), raw_r.shape)
    cs_f = np.where(mark_f, np.nan, raw_f)
    cs_r = np.where(mark_r, np.nan, raw_r)
    # fit L only: windows holding an observer reset leave the verdict
    # statistics like kerb-affected ones; absent key -> nothing changes
    reset_f = reset_r = None
    if "observer_reset_affected_f" in cs:
        reset_f = np.asarray(cs["observer_reset_affected_f"], dtype=bool)
        reset_r = np.asarray(cs["observer_reset_affected_r"], dtype=bool)
        cs_f = np.where(reset_f, np.nan, cs_f)
        cs_r = np.where(reset_r, np.nan, cs_r)
    stab_obs = stab["stability_observed_Nm_per_deg"]
    stab_valid = stab["stability_valid"]
    fz_f = fz["fz_f_N"] if fz is not None else None
    fz_r = fz["fz_r_N"] if fz is not None else None
    fy_f_norm = fz["fy_f_norm_N"] if fz is not None else None
    fy_r_norm = fz["fy_r_norm_N"] if fz is not None else None

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

    def _cs_stat(clean, raw, mark, idx, reset=None):
        # n_kerb_excluded: finite samples left out as kerb-affected, so the
        # display can say why a phase has no clean value (display only)
        stat = dict(_gate_cs_stat(_stats(clean[idx])))
        stat["n_kerb_excluded"] = int((mark[idx] & np.isfinite(raw[idx])).sum())
        if reset is not None:
            stat["n_reset_excluded"] = int((reset[idx] & ~mark[idx] & np.isfinite(raw[idx])).sum())
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
        if is_apex:
            return apex_instant_slice(t, start_t, apex_half_window_samples)
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
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
                    "cs_ratio_f": _cs_stat(cs_f, raw_f, mark_f, idx, reset_f),
                    "cs_ratio_r": _cs_stat(cs_r, raw_r, mark_r, idx, reset_r),
                    "stability_observed_Nm_per_deg": _stats(np.array([])),
                }
                if fz is not None:
                    corner_summary["phases"][phase]["fz_f_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fz_r_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fy_f_norm_N"] = _stats(np.array([]))
                    corner_summary["phases"][phase]["fy_r_norm_N"] = _stats(np.array([]))
                continue

            stab_valid_phase = stab_valid[idx]
            valid_fraction_stab = float(stab_valid_phase.sum() / n_samples)

            corner_summary["phases"][phase] = {
                "n_samples": int(n_samples),
                "valid_fraction_stab": valid_fraction_stab,
                "kerb_fraction": kerb_fraction,
                "cs_ratio_f": _cs_stat(cs_f, raw_f, mark_f, idx, reset_f),
                "cs_ratio_r": _cs_stat(cs_r, raw_r, mark_r, idx, reset_r),
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

        apex_idx = _apex_region_idx(c)
        corner_summary["apex_region"] = {
            "n_samples": int(apex_idx.size),
            "cs_ratio_f": _cs_stat(cs_f, raw_f, mark_f, apex_idx, reset_f),
            "cs_ratio_r": _cs_stat(cs_r, raw_r, mark_r, apex_idx, reset_r),
        }

        out.append(corner_summary)

    return out