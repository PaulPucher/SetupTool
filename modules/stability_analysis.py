# Stability analysis module for SetupTool.
# Pure Python/numpy/scipy. No Qt imports.
# Units: SI throughout (m, s, rad, N, Nm, kg).
# Cornering-stiffness (Module 4b) target relation and cross-lap yaw-moment-
# stability target relation (Module 5): method anchors recorded in
# thesis_notes.md, "CS_ratio (cornering stiffness ratio) -- Werner MA
# method" and "Yaw moment stability dMz/dbeta" entries. Effective-
# stiffness estimation (Module 4b) is adapted; Module 5's estimator
# (modules/yaw_stability.py) is after the chair performance_analysis
# tooling (internal), not Werner's own construction. See thesis_notes.md
# for both attribution splits.

import functools
import numpy as np
from scipy.signal import butter, filtfilt
import json
from modules.geo import project_latlon_to_xy
from modules.yaw_stability import calculate_filtered_yaw_acceleration, calculate_observed_stability

PARAMETERS_PATH = "config/parameters.json"
CAR_DATA_PATH = "config/car_data.json"

# WP5 persisted-analysis-cache version tag (models/outing.py analysis_data).
# Bump whenever a change to Modules 1-6 would alter summarise_corners()'s
# stored numeric output for the same input file (an estimator rebuild, a
# Fy/Fz formula change, a new regressor) -- NOT for changes that only affect
# how summaries are read or rendered (config-driven thresholds, UI, caching)
# -- OR whenever the analysis_data payload's own SHAPE changes (new fields
# the cache-hit check now requires), since an older payload has nothing to
# compare against for those fields either. A stored value that doesn't
# match this one is treated as no cache at all (see ui/views/outing_form.py's
# cache-hit check). Bumped 1->2 (WP-C): payload gained accuracy_cap/
# resolved_levels/resolved_vehicle_snapshot/resolved_clipped/resolved_
# warnings; a pre-WP-C payload has none of these and must not be read as a
# hit against the new cap/snapshot check. Bumped 2->3 (WP5b(b) phase 1 turn
# (b)): each phase dict inside summaries now carries fz_f_N/fz_r_N/
# fy_f_norm_N/fy_r_norm_N stat blocks; a pre-turn-(b) payload has none of
# these and must fall to no-cache, not render a details panel expecting
# keys that aren't there. Bumped 3->4 (fix turn): each corner summary now
# carries bracket_start_m/bracket_end_m (the canonical, post-WP1-Turn-3-
# partition corner window already computed in modules/corner_analysis.py,
# previously only on the raw corner dicts, not the persisted summary) --
# a pre-bump payload has neither key, so the trace window's margin
# computation must not read them off a stale cache. Bumped 4->5 (WP-N2 Step
# 1b): payload gained sideslip_source (which beta the analysis ran under --
# "kinematic" or "ekf_pass_1", config/parameters.json stability_estimation.
# sideslip_source), so a persisted run is never silently re-read under a
# different estimator after a config-switch flip and app restart. Bumped
# regardless of the default ("kinematic") producing byte-identical numbers --
# the payload SHAPE changed, which this version tag also covers per the rule
# above. Bumped 5->6 (fresh-session work package: per-session tyre auto-fit
# + NIS gate wired into production): sideslip_source gained two new values
# ("ekf_auto_dugoff", "ekf_auto_pacejka"); the payload gained fit_manifest
# (a stripped/JSON-safe summary of modules.tyre_fit_auto's fit result --
# axle parameters, R-sweep choice, validation numbers -- present only for
# the two auto modes, null otherwise), gate_verdict (modules.nis_gate's
# verdict dict, same null-elsewhere rule), and fallback_used/fallback_reason
# (whether the auto mode's gate failed/fit degenerated and kinematic beta
# was substituted, and why -- False/None for every other mode). A pre-bump
# payload has none of these keys; falling to no-cache on a version mismatch
# is the existing rule (ui/views/outing_form.py's cache-hit check), not a
# new one. kinematic and ekf_pass_1 modes' own OWN numeric content
# (summaries, sideslip_source, every pre-existing key) is byte-identical --
# only the payload's shape gained new, always-present-but-often-null keys.
# Bumped 6->7 (PLAN.md STEP 3, Phase 3): summarise_corners's optional ls=
# argument, when passed, adds ls_ratio_f/ls_ratio_r stat blocks to each
# phase dict (same _stats() shape as cs_ratio_f/cs_ratio_r) -- a
# pre-bump payload has neither key, so a persisted result predating this
# change must fall to no-cache, not render an LS trace/detail-card
# column against data that was never computed. DISPLAY ONLY: no
# classification/recommendation logic reads ls_ratio_f/r.
# Bumped 7->8 (CS validity repair part A, Phase 3): each corner summary
# gains a top-level apex_region dict (n_samples, cs_ratio_f, cs_ratio_r --
# same _stats() shape as a phase's own cs_ratio_f/r), a DISTANCE-based
# statistic around the apex replacing apex_3's structurally fixed
# 11-sample slice wherever an apex_3-keyed CS read feeds classification
# (_classify_corner/_phase_verdict). A pre-bump payload has no
# apex_region key, so a persisted result predating this change must fall
# to no-cache rather than have those call sites read a missing key.
# Existing phase dicts' own cs_ratio_f/r may also now report NaN in a few
# more cases than before (cs_phase_min_valid_samples gate) -- same
# pre-existing NaN-safe consumption, no new shape for that part.
# v8 EXTENDED, same package, no further bump (100 Hz time-base work
# order): ui/views/outing_form.py's _build_analysis_data_json payload
# (not summarise_corners's own output -- a payload-builder field, same
# scoping as sideslip_source/fit_manifest above) gained grid_rate_hz, the
# common-grid rate this run actually used (thesis_notes.md "PHASE 0").
# Not bumped to 9: this whole v7->8 package is still uncommitted, so v8's
# own documented shape is extended rather than versioned again for a
# change nothing has yet observed as "8" externally.
ANALYSIS_SCHEMA_VERSION = 8  # unchanged literal -- extension noted above, not a new bump

# Method-defining constants (CLAUDE.md grounding rule): these fix what the
# estimator IS, not how it is tuned to this car/track, so they stay as named
# constants rather than config entries.
BUTTERWORTH_ORDER = 4  # standard 4th-order digital filter; defines roll-off shape, not a physical threshold
SPAN_WEIGHT_EXPONENT = 4  # steep smooth-step so a section only counts once its alpha span nears cs_min_slip_angle_span_rad
R2_WEIGHT_EXPONENT = 1  # linear R^2 blend between window- and section-slope estimates, no extra shaping


@functools.lru_cache(maxsize=1)  # config only re-read after an app restart
def load_parameters():
    with open(PARAMETERS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


@functools.lru_cache(maxsize=1)  # same re-read-after-restart convention as load_parameters
def load_car_data():
    # config/car_data.json is gitignored/local-only digitised manufacturer
    # reference data (WP2b-1) -- not guaranteed to exist on every machine
    # this tool runs on, so a missing or malformed file degrades to None
    # rather than raising, mirroring core.config_loader.load_car_config's
    # existing convention for car.json. Never log this file's contents --
    # local-only means local-only, including in stdout/print debugging.
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


# 100 Hz time-base work package: the channels whose OWN native rate
# determines how fast the common grid can genuinely run (method-defining
# -- which channels constitute "the CS chain" is a fact about this
# estimator, not a per-car tunable). ecu_speed is deliberately excluded:
# it is always allowed to be the slower channel and gets upsampled onto
# whatever grid the OTHER five support (see prepare_vehicle_state's own
# comment on why that upsampling is safe).
CS_CHAIN_FAST_CHANNELS = ["sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z", "lap_distance"]


def _resolve_grid_rate(channels, params):
    """100 Hz time-base work package (thesis_notes.md 'PHASE 0'). Picks
    the common time grid's own rate: min(target_sample_rate_hz, the
    slowest CS_CHAIN_FAST_CHANNELS channel's own native rate) -- a file
    whose fast channels only support 50 Hz gets a 50 Hz grid (channel-
    limited, not refused); one supporting 100 Hz+ gets the 100 Hz target.
    Refuses only below min_sample_rate_hz (the GT3 Paul Ricard 20 Hz
    case), naming the binding (slowest present) channel. A fast channel
    that is entirely absent cannot bind the rate (nothing to measure) --
    its own absence is a separate, pre-existing degrade-to-None concern
    handled elsewhere (kerb_mask/s_m already tolerate a missing log_acc_z/
    lap_distance), not this guard's job.
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


def _normalize_lap_distance_to_metres(data, unit_raw):
    # lap_distance's own [unit] varies by export -- Dubai logs feet,
    # a real Paul Ricard export logs metres already (2026-08-31
    # investigation). The parser never validates a channel's actual file
    # unit against what code assumes (config's "unit" field is a display
    # label only, never checked against the file) -- this is the one
    # place a wrong assumption would silently scale every distance-
    # derived quantity (corner brackets, apex positions, Module 5's
    # s-anchored regression) by ~3.28x, so it is the one place that must
    # check the file's own claim before converting.
    if unit_raw == "ft":
        return data * 0.3048
    if unit_raw == "m":
        return data
    raise ValueError(
        f"lap_distance unit {unit_raw!r} not recognised (expected 'ft' or 'm') -- "
        "add explicit handling before trusting this export's distance values"
    )


def _interp_lap_distance_guarded(t_ref, ld_time, ld_data_m):
    # lap_distance resets to ~0 at every lap boundary. Linearly interpolating
    # across that boundary sample pair (as plain np.interp would) fabricates
    # a mid-range s value corresponding to no real track position, so any
    # t_ref sample whose bracketing native-sample pair straddles a reset is
    # set NaN instead. SetupTool-specific channel-alignment guard (Tier B):
    # the chair receives s_m natively at its own timeline and never needs
    # this interpolation step. [neutral engineering]
    # ld_data_m is already in metres -- callers normalise via
    # _normalize_lap_distance_to_metres before reaching here (2026-08-31,
    # the unit is a per-file fact, not something this shared helper can
    # know on its own).
    s_m = np.interp(t_ref, ld_time, ld_data_m)

    reset_after = np.zeros(len(ld_time), dtype=bool)
    reset_after[:-1] = np.diff(ld_data_m) < 0
    bracket_lo = np.clip(np.searchsorted(ld_time, t_ref, side="right") - 1, 0, len(ld_time) - 1)
    return np.where(reset_after[bracket_lo], np.nan, s_m)


def _build_inout_lap_mask(t_ref, laps):
    # Module 5 production exclusion (independent of the UI's is_valid_for_
    # analysis / lap_filter display toggle, per WP6/PLAN.md): cold-tyre in-
    # and out-lap samples violate the local-regression assumption that the
    # underlying vehicle condition is stationary across laps at the same
    # track position (see thesis_notes.md). Same epistemic category as the
    # kerb mask -- both exclude samples not representative of the racing
    # condition being modelled. [domain improvement]
    mask = np.zeros(len(t_ref), dtype=bool)
    for lap in laps or []:
        if lap.get("is_outlap") or lap.get("is_inlap"):
            mask |= (t_ref >= lap["start_time"]) & (t_ref <= lap["end_time"])
    return mask


def _compute_kerb_mask_from_az(az_g, threshold_g, baseline_g, dilation_samples):
    # Threshold flags the impact itself; dilating the mask catches the
    # ringdown oscillation on either side that the raw threshold misses.
    if az_g is None:
        return None
    raw = np.abs(az_g - baseline_g) > threshold_g
    if dilation_samples <= 0:
        return raw
    # Symmetric dilation: OR the mask with itself shifted +/-1..dilation_samples.
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

    # 100 Hz time-base work package (thesis_notes.md "PHASE 0"): the
    # common grid is no longer ecu_speed's own raw timestamps -- ecu_speed
    # is natively 50 Hz on this car (confirmed, diagnostics/inspect_
    # native_channel_rates.py), while the other five CS-chain channels
    # (CS_CHAIN_FAST_CHANNELS) are natively 100 Hz, and downsampling them
    # onto ecu_speed's own coarser grid was silently discarding half their
    # real resolution. _resolve_grid_rate picks the grid's own rate
    # (target, or slower if the fast channels can't support it -- refusing
    # only below the hard floor) and a synthetic, evenly-spaced grid at
    # that rate is built here, spanning ecu_speed's own observed time
    # range (still the anchor -- ecu_speed remains a required channel).
    # ecu_speed itself is then upsampled onto this grid via the SAME
    # np.interp every other channel already uses below: safe because
    # vehicle speed is an inertia-limited signal (cannot jump between
    # real samples), so linear interpolation between genuine readings
    # invents no meaningfully wrong information, unlike upsampling a
    # fast-changing quantity would.
    sr, grid_rate_status = _resolve_grid_rate(channels, params)
    ecu_speed_t = channels["ecu_speed"]["time"]
    # np.arange's own stop-EXCLUSIVE semantics silently dropped the file's
    # final sample here (invisible on a real ~80k-sample file, glaring on
    # a small fixture) -- np.linspace with an explicit, rounded sample
    # count is endpoint-inclusive and immune to step-accumulation drift.
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
    # steering_ratio_table (WP-B, Level 4): present only when modules.
    # accuracy_resolution resolved it there (car_data.json's manufacturer
    # steering_ratio_table available and cap allows it) -- absent on any
    # raw, un-resolved params dict (e.g. test_stability.py's direct call),
    # which keeps the plain constant division below byte-identical to
    # before this WP. np.interp clamps outside the table's own domain by
    # default -- the deliberate choice (config/car_data.json's table spans
    # +/-291 deg full-lock, well beyond any steering angle actually seen).
    steering_ratio_table = vp.get("steering_ratio_table")
    if steering_ratio_table is not None:
        i_s = np.interp(steer_sw_deg, steering_ratio_table["angle_deg"], steering_ratio_table["ratio"])
    else:
        i_s = vp["steering_ratio"]
    delta_f_rad = steer_sw_rad / i_s

    ay_mps2 = interp_channel("log_acc_y") * 9.81
    ax_mps2 = interp_channel("log_acc_x") * 9.81

    # Vertical accel (g) for kerb detection; optional -- stays None if the channel is missing/failed/untimed.
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

    # Track-distance coordinate for Module 5's s-anchored regression (see
    # modules/yaw_stability.py). Optional -- None if lap_distance is missing
    # or invalid, same missing-channel-degrades-to-None pattern as az_g/GPS above.
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
    """Kinematic identity ay = v*(beta_dot + psi_dot). Method anchor
    recorded in thesis_notes.md, WP-S4 entry. Washout integration below
    is Tier B signal conditioning (drift correction), not part of the
    cited identity itself.
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
    """Circular-safe interpolation of an angle in degrees onto a new time
    base. Naive linear interpolation across the 0/360 wrap corrupts values
    near the wrap point (e.g. 359 -> 1 deg reads as a -358 deg jump); the
    sin/cos components are interpolated separately and the angle recovered
    via atan2 instead -- standard technique for circular quantities.
    Returns radians, wrapped to (-pi, pi].
    """
    rad_src = np.radians(deg_src)
    sin_i = np.interp(t_query, t_src, np.sin(rad_src))
    cos_i = np.interp(t_query, t_src, np.cos(rad_src))
    return np.arctan2(sin_i, cos_i)


def estimate_sideslip_gps(state, channels, params):
    """WP5b(c): GPS-course-based sideslip estimate (beta_gps), a Level-3
    validation candidate. VALIDATION ONLY in this phase -- not called from
    any pipeline or UI path; production beta stays estimate_sideslip's
    kinematic integration + washout, unchanged. See diagnostics/inspect_
    beta_gps_validation.py for the comparison report and thesis_notes.md
    for the full write-up.

    Concept, Tier A: beta = course-over-ground minus vehicle heading, the
    GPS-aided kinematic sideslip estimation family (thesis_notes.md
    limitations register item 4; primary source to verify before citing).
    Heading is not logged (log_a_car refuted as a heading channel,
    thesis_notes.md channel census); reconstructed here by integrating
    yaw_rate_radps and periodically re-anchoring the drift to
    log_gps_course at trustworthy low-slip samples -- the same "linear
    reference held while inside a small window" pattern already used in
    estimate_cornering_stiffness's cs_linear_slip_threshold_rad. Tier B,
    standard drift-correction/bias-resync technique.

    ROTATION CONVENTION (empirical finding, diagnostics/inspect_beta_gps_
    validation.py, WP5b(c), laps 1-4 (valid-for-analysis), n=5014 moving
    samples): log_gps_course is a compass bearing (0-360 deg, clockwise-
    positive from North -- standard GPS/NMEA course-over-ground
    convention). Correlating wrap-safe d(course)/dt against +yaw_rate_radps
    gives r=-0.9548; against -yaw_rate_radps gives r=+0.9548.
    -yaw_rate_radps is adopted below (the strong positive correlation) --
    i.e. yaw_rate_radps in this project's convention is counter-clockwise-
    positive, the OPPOSITE rotational sense from the compass bearing. This
    does not follow from the z-down accelerometer convention already
    established for kerb detection (thesis_notes.md) -- tested
    independently here, not assumed from that precedent.

    GPS LATENCY (same diagnostic, cross-correlation over lags -1..+1 s):
    peak r=0.9898 at lag=+0.32 s -- course-derived rate lags yaw_rate by
    ~0.32 s, a real, measurable GPS pipeline delay (zero-lag r was only
    0.9575). CORRECTED as of iteration 2 (WP5b(c)): course is sampled
    gps_course_latency_s seconds ahead of each query time (config,
    derived_from the cross-correlation evidence above) before anchoring or
    subtraction -- a stale course reading recorded at time t describes the
    vehicle's true state at t - latency, so querying it at t + latency
    recovers the value that actually corresponds to "now".

    Anchor gate (Tier B, data-derived, diagnostics/inspect_beta_gps_
    validation.py): a sample qualifies once gps_course_anchor_smooth_
    window_s-smoothed |ay| stays below gps_course_anchor_max_ay_g for a
    contiguous run of at least gps_course_anchor_min_duration_s (smoothing
    was necessary -- the raw per-sample gate produced only sub-0.1 s runs
    on Dubai, too brittle to anchor anything). Each qualifying run
    contributes one anchor point (its time midpoint, its median course-
    minus-gyro offset, unwrapped within the run first) -- 6 anchors,
    20.4 s total anchor time, found on the Dubai sample.

    DRIFT ALLOCATION (iteration 2, WP5b(c)): iteration 1 interpolated the
    anchor offset linearly in ELAPSED TIME between anchors and produced a
    large, poorly-correlated beta_gps (see thesis_notes.md). A closed-loop
    per-lap check (diagnostics/inspect_beta_gps_validation.py Section 1b)
    diagnosed why: the ~6 deg/lap gyro-integration shortfall is a
    scale-type error that accumulates in proportion to ROTATION
    (concentrated in the laps' ~15 corners), not in proportion to clock
    time, so a time-linear correction under-corrected exactly where
    cornering (and beta) happens. Fixed here: the offset is interpolated
    in proportion to accumulated |yaw_rate| integral between the anchor
    pair instead -- a monotonic "rotation clock" replaces the time axis
    for this one interpolation, still via np.interp, no new machinery.
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

    # Open-loop heading from yaw rate, sign per the rotation-convention
    # finding above (matches course's clockwise-from-North sense).
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
        # No qualifying straight-line window in this file -- drift is
        # unresolved, beta_gps is not trustworthy anywhere. Degenerate
        # case, not exercised by the Dubai sample (6 anchors found there).
        return np.full_like(t_ref, np.nan)

    anchor_offsets_unwrapped = np.unwrap(np.array(anchor_offsets))
    # Iteration 2: allocate the correction by accumulated |rotation|, not
    # elapsed time -- a monotonic "rotation clock" (cumulative |yaw_rate|
    # integral) replaces the time axis for this one interpolation, still
    # via np.interp (holds the boundary value outside the anchored range,
    # same as the time-linear version did).
    yaw_abs_cum = np.cumsum(np.abs(state["yaw_rate_radps"])) * dt
    anchor_rotation = np.interp(anchor_times, t_ref, yaw_abs_cum)
    drift_offset = np.interp(yaw_abs_cum, anchor_rotation, anchor_offsets_unwrapped)

    psi_hat = psi_gyro + drift_offset
    beta_gps = np.arctan2(np.sin(course_rad - psi_hat), np.cos(course_rad - psi_hat))
    beta_gps = np.where(moving, beta_gps, np.nan)
    return beta_gps


def estimate_slip_angles(state, beta, params):
    """Single-track slip-angle relations; method anchor recorded in
    thesis_notes.md, "CS_ratio (cornering stiffness ratio) -- Werner
    MA method" entry.
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
    """Module 4a: axle lateral forces via 2-DOF planar force/moment
    balance -- Fy_f = m*ay*front_fraction + Iz*psidd/wheelbase,
    Fy_r = m*ay - Fy_f. Method anchor recorded in thesis_notes.md,
    "Fy yaw-moment term (Module 4a)" entry. Same construction as the
    chair performance_analysis tooling's own fy_f_N/fy_r_N (internal);
    no deviation, this is adopted as-is.

    psidd is the RAW yaw acceleration (np.gradient of yaw_rate_radps),
    computed here independently of Module 5's 0.15 s rolling-mean-
    filtered signal (modules/yaw_stability.py) -- the chair keeps these
    separate too: raw for this instantaneous per-sample force balance,
    filtered only for the windowed stability regression. Pre-smoothing
    psidd here with a different time constant before Module 4b's own
    downstream Butterworth filter (cs_filter_cutoff_hz) would
    double-filter with inconsistent time constants.

    Method upgrade only, not an accuracy-level upgrade:
    accuracy_levels.lateral_force_split stays 1 -- Iz and the static
    corner-weight fractions are still Level 1, so the new yaw term
    inherits their ~10-20% uncertainty rather than adding a
    better-characterised signal.
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
    """WP5b(b) phase 1: axle and per-wheel vertical tyre loads (Fz), plus
    the normalised-force diagnostic fy_f_norm_N/fy_r_norm_N.

    Method anchor recorded in thesis_notes.md, "WP5b(b) phase 1:
    chair-parity vertical loads (Fz)" entry. Same construction as the chair
    performance_analysis tooling's own fz_f_N/fz_r_N/fz_fl_N/fz_fr_N/
    fz_rl_N/fz_rr_N and fy_f_norm_N/fy_r_norm_N
    (docs/literature/data_handler.py:1548-1621, internal) -- adopted as-is,
    no deviation. The per-wheel split is the chair's own independent-
    per-axle lateral-transfer split, NOT a roll-stiffness apportionment
    (that stays a documented later DOMAIN IMPROVEMENT, damper-validated,
    PLAN.md WP5b(b)).

    fy_f_norm_N/fy_r_norm_N = Fy_f_filt/fz_f_N, Fy_r_filt/fz_r_N -- a
    diagnostic only in phase 1 turn (b): read-only, surfaced in Module 6/
    the UI details panel, feeds no classification (_classify_corner is
    untouched). It is a separate quantity from CS_ratio (Module 4b's
    Calpha-ratio metric), not a replacement.

    accuracy_levels.vertical_load_split / per_wheel_load_split stay at
    Level 1: cog_height_m, track_width_front/rear_m and the aero
    coefficients are all unsourced placeholders (config/parameters.json
    notes).

    Fz-integration Phase 1 (2026-09-03, Tier B: a consumption switch, not
    a new vehicle-dynamics method -- modules.wheel_loads's own Segers
    anchor is unchanged and does all the physics here): when config
    stability_estimation.vertical_load_source is "measured" and both
    channels/car_data are supplied, the static model above is computed
    exactly as before but used only as the CASCADE'S OWN innermost
    fallback (modules.wheel_loads.combine_with_reconstruction_and_
    fallback) instead of as the final answer -- fz_fl_N/fz_fr_N/fz_rl_N/
    fz_rr_N (and therefore fz_f_N/fz_r_N, resummed from them so the two
    always agree) become damper-measured where valid, axle-total-
    reconstructed where exactly one corner of an axle is invalid, static
    otherwise. channels/car_data default to None so every existing call
    site is unaffected; "static" (the config default) never touches
    modules.wheel_loads at all.
    """
    vp = params["vehicle"]
    aero = vp["aero"]

    m = vp["mass_kg"]
    g = 9.81
    wb = vp["wheelbase_m"]
    l_f_cog = vp["cog_to_front_axle_m"]
    l_r_cog = vp["cog_to_rear_axle_m"]
    h_cog = vp["cog_height_m"]

    # --- 1. Static load distribution (positive downwards) ---
    fz_static_f_N = m * g * l_r_cog / wb
    fz_static_r_N = m * g * l_f_cog / wb

    # --- 2. Aerodynamic load component (positive for downforce) ---
    rho = aero["air_density_kgm3"]
    cl = aero["lift_coeff"]
    a_aero = aero["cross_track_area_m2"]
    x_cp_cog = aero["diff_cog_x_m"]
    fz_aero_total_N = -0.5 * rho * state["v_mps"] ** 2 * a_aero * cl
    dfz_aero_f_N = fz_aero_total_N * (l_r_cog - x_cp_cog) / wb
    dfz_aero_r_N = fz_aero_total_N * (l_f_cog + x_cp_cog) / wb

    # --- 3. Longitudinal load transfer component ---
    dfz_long_transfer_N = m * state["ax_mps2"] * h_cog / wb

    fz_f_N = fz_static_f_N + dfz_aero_f_N - dfz_long_transfer_N
    fz_r_N = fz_static_r_N + dfz_aero_r_N + dfz_long_transfer_N

    # --- 4. Per-wheel split: independent per-axle lateral-transfer, chair-identical ---
    front_track = vp["track_width_front_m"]
    rear_track = vp["track_width_rear_m"]
    lateral_transfer_front = m * state["ay_mps2"] * h_cog / front_track
    lateral_transfer_rear = m * state["ay_mps2"] * h_cog / rear_track

    fz_fl_N = fz_f_N / 2 - lateral_transfer_front / 2
    fz_fr_N = fz_f_N / 2 + lateral_transfer_front / 2
    fz_rl_N = fz_r_N / 2 - lateral_transfer_rear / 2
    fz_rr_N = fz_r_N / 2 + lateral_transfer_rear / 2

    # --- Fz-integration Phase 1: swap in the damper cascade, static model
    # stays available as its own innermost fallback (see docstring). ---
    vertical_load_source = params["stability_estimation"].get("vertical_load_source", "static")
    fz_source_per_sample = None
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
        else:
            # No corner has ANY real damper sample this session (e.g. Dubai
            # -- no damper channels at all): estimate_session_corrected_
            # axle_totals's own straight-line means would average nothing
            # (NaN), and reconstruct_missing_corner never selects this value
            # anyway once every corner is invalid (an invalid corner's own
            # axle-mate is also always invalid, so reconstructable is False
            # everywhere) -- skip the unused fit rather than compute NaN.
            fz_axle_totals = {"fz_f_N": fz_f_N, "fz_r_N": fz_r_N}
        combined = combine_with_reconstruction_and_fallback(damper_result, fz_axle_totals, static_fallback_fz)
        fz_fl_N = combined["fl"]["fz_N"]
        fz_fr_N = combined["fr"]["fz_N"]
        fz_rl_N = combined["rl"]["fz_N"]
        fz_rr_N = combined["rr"]["fz_N"]
        # Axle total = sum of the cascade's own per-wheel outputs, not the
        # static axle formula above or the session-corrected model directly
        # -- keeps fz_f_N/fz_r_N always consistent with fz_fl_N+fz_fr_N /
        # fz_rl_N+fz_rr_N regardless of which tier produced each sample.
        fz_f_N = fz_fl_N + fz_fr_N
        fz_r_N = fz_rl_N + fz_rr_N
        fz_source_per_sample = {c: combined[c]["source"] for c in WHEEL_CORNERS}
        vertical_load_source = "measured"
    else:
        vertical_load_source = "static"

    # --- 5. Normalised-force diagnostic, chair's own fy_*_norm_N construction ---
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
    }


def resolve_cs_min_window_samples(params, sample_rate_hz):
    """CS validity repair part A, Phase 1 (rate-corrected): cs_min_window_s
    is a PHYSICAL window duration, not a sample count -- the chair's own
    literal default (10 samples) was always a 100 Hz-calibrated value
    (10/100 = 0.1 s), silently treated as rate-independent until this
    correction. Converts to samples at THIS file's own measured rate,
    same pattern as modules.longitudinal_stiffness's own 50 Hz min_samples
    adaptation (regression_window_s * sample_rate_hz, floored). Shared by
    estimate_cornering_stiffness and every UI/diagnostics caller that
    reconstructs a window, so the derivation can never drift between them.
    """
    se = params["stability_estimation"]
    return max(se["cs_min_window_samples_floor"], int(round(se["cs_min_window_s"] * sample_rate_hz)))


def reconstruct_cs_window_start(alpha, i, min_window, min_span, s_m=None, max_window_m=None):
    """Reconstruct the sliding window's own start index for target index
    `i` -- mirrors compute_cs_for_axle's internal growth loop below
    exactly, for callers that only have the per-sample CS_ratio/C_alpha
    output and need to know which raw samples produced one particular
    estimate (the corner-trace track map's front/rear "estimation window"
    highlight, and various diagnostics scripts' tyre-curve window
    scatter). Reconstruction only, not a second implementation of the
    estimator -- the CS value itself always comes from this function's
    own returned arrays, never recomputed here. Verified against a
    captured C_window_f/r trace to 1e-6 relative tolerance before this
    was factored out of the (then diagnostics-only) copy of this loop.

    min_window is already a resolved SAMPLE COUNT here (see
    resolve_cs_min_window_samples) -- this function has no opinion on
    physical units, only the caller does.

    s_m/max_window_m (CS validity repair part A, Phase 2, DISTANCE-based
    per the locality-bound revision): caps how far the window may grow by
    real track distance travelled, not a converted sample count -- a
    corner's own physical scale is a distance, not a duration, so the cap
    must not silently change meaning between a slow and a fast corner at
    the same sample count. Omitting either (or an unreadable s_m at the
    boundary -- NaN across a lap-distance reset, or a lap-boundary
    crossing) falls back to NO distance cap for that reconstruction --
    safe ONLY when called on an index already known to carry a finite
    CS_ratio (compute_cs_for_axle itself enforces the cap when producing
    that value; an index with no finite CS_ratio never had a qualifying
    window to reconstruct in the first place).
    """
    start = i - min_window
    s_i = s_m[i - 1] if (s_m is not None and max_window_m is not None) else None
    if s_i is not None and not np.isfinite(s_i):
        s_i = None
    while start > 0:
        span = np.max(alpha[start:i]) - np.min(alpha[start:i])
        if span >= min_span:
            break
        if s_i is not None:
            s_start = s_m[start]
            if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                break
        start -= 1
    return max(start, 0)


def estimate_cornering_stiffness(slip, forces, state, params):
    """Module 4b: effective cornering stiffness / CS ratio.

    Method anchor recorded in thesis_notes.md, "CS_ratio (cornering
    stiffness ratio) -- Werner MA method" entry. Effective-stiffness
    estimation is adapted (windowed regression from logged Fy/alpha in
    place of Werner's Pacejka-model evaluation) -- see thesis_notes.md.
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
        # Per-sample record of the linear-region reference slope in effect
        # at each index (CS_ratio's denominator) -- exposed for the
        # tyre-curve audit plot (WP-A item 3), not used elsewhere.
        C_linear_ref_arr = np.full(n, np.nan)

        sections, section_id = _find_monotonic_sections(alpha)
        sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)

        for i in range(min_window, n):
            if not moving[i]:
                continue

            # Adaptive widening (CS validity repair part A, Phase 2): grow the
            # window until it clears BOTH floors, capped at max_window_m (a
            # real TRACK DISTANCE, not a sample count -- Phase 1 REVISION's
            # locality bound) so a near-flat-alpha stretch (a straight, a
            # slow lift) cannot chase min_span arbitrarily far back and
            # blend in unrelated track sections -- see cs_max_window_m's own
            # config comment. Mirrors reconstruct_cs_window_start exactly;
            # keep both in sync.
            start = i - min_window
            s_i = s_m[i - 1] if s_m is not None else None
            if s_i is not None and not np.isfinite(s_i):
                s_i = None
            while start > 0:
                span = np.max(alpha[start:i]) - np.min(alpha[start:i])
                if span >= min_span:
                    break
                if s_i is not None:
                    s_start = s_m[start]
                    if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                        break
                start -= 1

            window_alpha = alpha[start:i]
            window_Fy = Fy[start:i]
            achieved_span = np.max(window_alpha) - np.min(window_alpha)
            if achieved_span < min_span:
                continue  # widening could not clear the span floor within the cap -- no signal

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
    """Module 5: yaw moment stability dMz/dbeta.

    Target relation method anchor recorded in thesis_notes.md, "Yaw
    moment stability dMz/dbeta" entry (Mz = Iz*psidd + D_psi*psid);
    D_psi term not yet computed (no wheel-load sensor); see
    thesis_notes.md "Completing Werner Eq. 4.3" and WP5b. The estimator
    itself (yaw-accel rolling mean, s-anchored Gaussian-weighted local
    ridge regression) is modules.yaw_stability, after the chair
    performance_analysis tooling (internal) -- see thesis_notes.md for
    the attribution split and the call-site sample-exclusion adaptation
    notes below.

    Front/rear saturation as controllability-loss vs stability-loss,
    and the saddle-node framing (motivation only, no bifurcation
    analysis implemented): method anchors recorded in thesis_notes.md,
    "Front/rear saturation and saddle-node concept anchors closed"
    entry.

    Sample exclusions (moving mask, kerb mask, structural in/out-lap
    exclusion) are all applied HERE, at the call site, by NaN-ing
    excluded samples before handing arrays to the chair-derived
    estimator; the estimator itself runs unmasked on whatever it is
    given, exactly as the chair's own tooling does on a full session.
    [neutral engineering]
    In/out-lap exclusion is production behaviour, independent of the
    UI's display lap_filter (WP6): cold tyres change stiffness, which
    would corrupt the cross-lap pooling this estimator relies on.
    [domain improvement]

    The chair estimator carries a time-anchored fallback mode when s_m
    is unusable; SetupTool deliberately does not port it: the fallback
    is a differently-behaving estimator (time-local, no cross-lap
    pooling) whose output the s-grid-derived thresholds could not
    classify meaningfully -- no stability verdict is more honest than
    a silently degraded one.
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
                       cs_apex_region_half_length_m=None, stab_phase_no_braking_floor_bar=None):
    # fz (modules.stability_analysis.estimate_vertical_loads's output) is
    # optional and additive only: passing it adds fz_f_N/fz_r_N/
    # fy_f_norm_N/fy_r_norm_N stat blocks per phase; omitting it (older
    # diagnostics/*.py call sites predating WP5b(b)) reproduces the exact
    # pre-turn-(b) summary shape, no behaviour change for those callers.
    # ls (modules.longitudinal_stiffness.estimate_longitudinal_stiffness's
    # output, PLAN.md STEP 3 Phase 3) is the same additive-optional
    # pattern: passing it adds ls_ratio_f/ls_ratio_r stat blocks per
    # phase, same _stats() treatment as cs_ratio_f/cs_ratio_r; omitting
    # it reproduces the exact pre-Phase-3 summary shape.
    # v3 diagnostics Part C2 (2026-09-03): stability_observed_Nm_per_deg
    # now goes through _gate_stab_stat, same no-signal-on-too-few-samples
    # gate CS_ratio already had (_gate_cs_stat) plus a brake-specific
    # no-actual-braking check for entry_1_brake -- see that helper and
    # stab_phase_no_braking_floor_bar's own config comment.
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
        # CS validity repair part A, Phase 2: a CS_ratio stat block backed
        # by too few finite samples reports NaN (no signal) rather than a
        # median that is really just one or two extreme readings -- see
        # cs_phase_min_valid_samples's own config comment.
        if stat["n"] < cs_phase_min_valid_samples:
            return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        return stat

    def _gate_stab_stat(stat, phase, brake_vals):
        # v3 diagnostics Part B2/C2 (2026-09-03): stability_observed_Nm_
        # per_deg gets the SAME sample-count gate CS_ratio already has
        # (cs_phase_min_valid_samples reused, not re-derived -- see
        # stab_phase_no_braking_floor_bar's own config comment) plus a
        # second, brake-specific check found evidenced on GT3_PRC_MLA-v3's
        # C7/C15: an entry_1_brake phase with no real braking (max brake
        # pressure never clears the floor) reports no-signal instead of a
        # median computed from an essentially-unloaded phase.
        if stat["n"] < cs_phase_min_valid_samples:
            return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        if phase == "entry_1_brake" and brake_f_bar is not None:
            if len(brake_vals) == 0 or float(np.nanmax(brake_vals)) < stab_phase_no_braking_floor_bar:
                return {"median": float("nan"), "p25": float("nan"), "p75": float("nan"), "n": stat["n"]}
        return stat

    def _apex_region_idx(c):
        # CS validity repair part A, Phase 3: a DISTANCE-based (not sample-
        # count) window around the apex, replacing apex_3's structurally
        # fixed 11-sample slice for CS reads (thesis_notes.md "apex_3
        # structural finding"). Bounded in TIME to this corner's own
        # instance first (union of its own 5 phase segments) before
        # applying the distance band -- s_m resets every lap, so a pure
        # distance-band search would otherwise pull in every other lap's
        # samples passing the same track distance.
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
                corner_summary["phases"][phase]["ls_ratio_f"] = _stats(ls_f[idx])
                corner_summary["phases"][phase]["ls_ratio_r"] = _stats(ls_r[idx])

        apex_idx = _apex_region_idx(c)
        corner_summary["apex_region"] = {
            "n_samples": int(apex_idx.size),
            "cs_ratio_f": _gate_cs_stat(_stats(cs_f[apex_idx])),
            "cs_ratio_r": _gate_cs_stat(_stats(cs_r[apex_idx])),
        }

        out.append(corner_summary)

    return out