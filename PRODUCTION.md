# PRODUCTION -- the SetupTool signal chain as it runs today

Dated: 2026-10-07. Describes the code on branch new-laps: commit 9427b6a
plus the packages of 2026-10-06 / 07 (event layer, product text, analysis
lock, shading budget and corner-trace default, leading pit-fragment merge,
UI cleanup round), ANALYSIS_SCHEMA_VERSION 14. Updated on request only;
every update replaces the content and re-dates this header.

This file states what is in production, from the raw logger file to the
setup recommendation, one section per stage of the signal chain. It is the
skeleton of the methods chapter. It describes what IS; how the design got
here lives in thesis_notes.md, the standing rulings in DECISIONS.md.


## 0. How to read this file

Every number below carries its HOME:

- CONFIG = config/parameters.json, path given as block.key. Other config
  files are named explicitly (channels.json, decision_frame.json,
  recommendations.json). A config value is a calibration value: tuned to
  data or hardware, changeable without changing what the method is.
- CODE = a named constant in the module named. A code constant is
  method-defining: it states what the method is (filter order, kernel
  shape, blend exponent), not how it is tuned.
- HOME FLAG = a number whose home breaks that split. Flagged where it
  occurs; nothing has been moved.

Vehicle description (mass, corner weights, yaw inertia, CoG position,
wheelbase, track widths, CoG height, steering map, motion ratios, ARB
rates, roll centres, tyre radii, unsprung masses) is parameterisation, not
a tunable. It lives in the vehicle block of config/parameters.json and in
the local, untracked config/car_data.json, and is not reproduced here.
Its provenance is the accuracy level of each quantity (section 17).

Accuracy levels: 1 = config default, 2 = session measurement, 3 = logged
sensor, 4 = lookup table (manufacturer data).

Tiers: A = vehicle-dynamics method, needs a literature anchor; B = signal
or data engineering (filters, masks, segmentation, thresholds), standard
technique, labelled preprocessing; C = UI / product.

Chair comparison: the reference is the chair performance_analysis tooling
(internal). Per stage it is stated which parts are the chair's method
as-is, which are adapted (FORCED ADAPTATION, DOMAIN IMPROVEMENT or
NEUTRAL ENGINEERING, per CLAUDE.md), and which are SetupTool's own.
Vehicle parameterisation is never a deviation.

Symbols used throughout: t time [s]; v vehicle speed [m/s]; r yaw rate
[rad/s]; a_x, a_y, a_z longitudinal, lateral, vertical acceleration
[m/s^2, or g where stated]; delta_f road-wheel steer angle [rad]; beta
vehicle sideslip angle at the CoG [rad]; alpha_f, alpha_r front and rear
axle slip angles [rad]; Fy_f, Fy_r axle lateral forces [N]; Fz vertical
load [N]; m vehicle mass [kg]; I_z yaw inertia [kg m^2]; a, b CoG-to-front
and CoG-to-rear axle distances [m]; L = a + b wheelbase [m]; s lap
distance [m].


## 1. Channels in: the Cosworth logger export

What comes in. One Pi Toolbox ASCII export of a Cosworth logger outing
(modules/csv_parser.py). Two block layouts are read (one channel per
block, or a wide time-plus-all-channels block); comma decimals are
accepted; the file is decoded as latin-1. Only channels whitelisted in
config/channels.json are kept. Every channel keeps its own native time
base and the unit string the FILE declares (unit_raw); conversions
downstream read unit_raw per file and refuse unknown units rather than
guess (lap distance ft or m, suspension travel mm or m, damper force N
only, ABS wheel speed kph or mph).

Channels the analysis chain uses: ecu_speed (km/h), sclu_yaw_rate (rpm),
log_asteer (steering-wheel angle, deg), log_acc_x / log_acc_y /
log_acc_z (g), lap_number, lap_time, lap_distance, ecu_B_speedlimit_en
(pit limiter), ecu_aps (throttle, %), log_pbrake_f / log_pbrake_r (bar),
ecu_gear, log_gps_lat / log_gps_lon, log_speed_* and abs_speed_* (wheel
speeds), log_dms_dam_* (damper pushrod force, N), log_susp_travel_*
(suspension travel), log_abs_pos and abs_active (ABS), tpms_press_*
(tyre pressures). prepare_vehicle_state requires ecu_speed, sclu_yaw_rate, log_asteer, log_acc_y and
log_acc_x, else the analysis is refused.

Order of operations per channel, in the parser:
1. Decoding correction (channels.json channel_corrections): a known
   corrupt additive offset is removed only if the file's raw mean lies
   inside the stated precondition range. One entry exists:
   log_dms_dam_fr offset +15954360.4 N, precondition mean range
   [-16050000, -15850000] N (a decoding fault on one Paul Ricard file).
2. Event zero entry (channels.json event_zero_entries): see section 9.1.
3. Quality gate (channels.json channel_quality_gates): share of samples
   inside the channel's plausibility range; below failed_below = 0.5 the
   channel is "failed", below partial_below = 0.95 "partial", else
   "valid". Ranges per channel live in channels.json channels.<name>.range.

Lap structure (modules/csv_parser.py, channels.json lap_splitting):
- Laps are split on lap_number. Lap 0 is the outlap by position.
- A leading pit fragment is merged into the next lap, which keeps the
  outlap flag: the first lap merges when the pit limiter is on at its END
  (Level 3; the lap counter ticked while the car was still in the pit
  lane), or, without that channel, when it is shorter than
  pit_fragment_max_duration_s = 20 s (Level 1). It removed a 16-18 s
  pit-lane fragment from 8 of 24 exports; no analysis lap changed.
- A trailing pit fragment is merged into the previous lap, which becomes
  the inlap: by the pit limiter (on at the fragment's first sample, Level
  3) or, without that channel, by duration < pit_fragment_max_duration_s
  = 20 s (Level 1).
- Pit limiter on at a lap's start marks an outlap, on at its end an inlap
  (flags are only added, never cleared).
- Lap time: the computed duration, replaced by the lap_time channel's
  finer value when the two agree within lap_time_precise_max_delta_s =
  1.0 s.
- Lap warnings: lap_time channel disagrees with the duration by more than
  lap_time_disagreement_max_s = 2.0 s; lap distance rises by less than
  lap_distance_min_travelled_m = 1000 on a lap longer than
  lap_distance_check_min_duration_s = 30 s.
- ANALYSIS LAP (is_valid_for_analysis): not out, not in, no warning,
  lap time > lap_time_min_s = 10 s and <= valid_lap_max_ratio = 1.1 x
  the fastest such lap. Every verdict, fit and threshold in this file is
  built on analysis laps only.
- HOME FLAG (minor): the lap_time disagreement check is skipped unless
  the channel's maximum exceeds a literal 5 s in code (csv_parser.py
  _verify_laps); a guard value with no config key.

Tier B / neutral engineering throughout. No chair counterpart (the chair
tooling does not parse this logger's export).


## 2. Preprocessing and conditioning

### 2.1 Common time grid

Why: the chain mixes channels logged at different rates; every estimator
window was validated at 50-100 Hz.
- Grid rate = min(target_sample_rate_hz, slowest native rate among the
  fast CS-chain channels sclu_yaw_rate, log_asteer, log_acc_y, log_acc_z,
  lap_distance). CONFIG stability_estimation.target_sample_rate_hz = 100.
  Below min_sample_rate_hz = 50 the analysis is refused, naming the
  binding channel.
- The grid spans ecu_speed's time range (np.linspace, inclusive). Every
  channel is linearly interpolated onto it; ecu_speed (50 Hz on the
  sessions checked) is upsampled, which is safe because speed is
  inertia-limited.
- Unit conversions: v = ecu_speed / 3.6; r = sclu_yaw_rate x
  stability_estimation.yaw_rate_to_radps (= 0.10472, the rpm-to-rad/s
  factor held in config); a_x, a_y = channel x 9.81; a_z stays in g for
  the kerb detector.
- Lap distance is converted to metres from the file's unit and
  interpolated with a reset guard: grid samples that straddle the
  lap-counter reset get NaN instead of an invented mid-lap position.
- Accuracy: speed Level 1 (ecu_speed, opaque ECU calibration), yaw rate
  Level 3, lateral acceleration Level 1 (IMU not at the CoG), steering
  angle Level 1 or 4 (2.2).
- Known limit, not a home violation: several windows are still sample
  counts and therefore halve their physical duration at 100 Hz:
  tyre_fit_auto.nis_window_samples = 20 (0.2 s at 100 Hz, the EKF
  divergence monitor, section 5) and stability_estimation.
  apex_half_window_samples = 5 (12.2). The config comment
  _comment_grid_rate records this.

Neutral engineering. The chair's 10-sample window default was implicitly
a 100 Hz figure; the time-based floors in this file restore that
meaning on any grid.

### 2.2 Steering: steering-wheel angle to road-wheel angle

Why: the bicycle model needs ONE front road-wheel angle delta_f; the
logger records the steering-wheel angle.
- Level 4 (manufacturer table in car_data.json present and the accuracy
  cap allows it, modules/accuracy_resolution.py): the table's wheel-angle
  column is read directly against its steering-wheel-angle column; the
  table's ratio column is the LOCAL ratio d(sw)/d(wheel) and is not used
  for the conversion. The table gives one wheel over the full stroke; the
  + and - branches differ at large lock (Ackermann). The bicycle-model
  angle is the mean of |wheel| on both branches at the same |sw|, signed
  like sw (Rajamani sec. 2.2, Ackermann turning geometry, Eq. 2.17: the
  single-track steer angle is the average of the inner and outer front
  wheel angles). The result is odd in sw. delta_f = interp(sw) and is
  clamped at the table's ends (end stop).
- The table is rejected (fall back to Level 1) unless it is finite,
  strictly increasing in both columns, sign-preserving and spans both
  signs.
- Level 1 (no table): delta_f = sw / vehicle.steering_ratio (constant).
- Tier A (Rajamani sec. 2.2). Vehicle parameterisation, not a chair
  deviation.

### 2.3 Moving gate

moving = v > stability_estimation.moving_speed_min_mps = 5.0 m/s
(CONFIG). Outside it: kinematic beta, slip angles and axle forces are set
to 0 before filtering, no CS is computed, the EKF resets, and no
stability regression sample is taken. The wheel-load straight-line fits
(9.4) use v >= 5.0 m/s. Tier B.

### 2.4 Zero-phase low-pass (the 2 Hz conditioning)

Slip angles, axle lateral forces and the understeer-gradient signals are
low-passed with a Butterworth filter run forward and backward (filtfilt,
zero phase). Cutoff CONFIG stability_estimation.cs_filter_cutoff_hz =
2.0 Hz; order CODE stability_analysis.BUTTERWORTH_ORDER = 4
(method-defining roll-off). The same 4th-order design is used for the
washout high-pass (3). Chair's method as-is (2 Hz, 4th order). Tier B.

### 2.5 Kerb exclusion (one concept)

Why: a kerb strike is a vertical impulse that the planar single-track
model cannot represent; samples it disturbs must not shape the tyre
estimate or the verdict. Kerb exclusion is ONE concept with three
internal workings.

- Detection (the mask): kerb = |a_z - kerb_baseline_g| >
  kerb_z_deviation_threshold_g, with CONFIG stability_estimation.
  kerb_baseline_g = 1.0 g and kerb_z_deviation_threshold_g = 1.2 g. The
  config carries no derived_from for the 1.2 g value.
- Dilation: the raw mask is widened by +/- kerb_dilation_s = 0.10 s
  (CONFIG), converted to samples at the grid rate (+/-10 at 100 Hz), to
  cover the ringdown around the hit.
- Filter-smear reach (the grade mark): the 2 Hz zero-phase filter (2.4)
  runs over the strike BEFORE the mask acts, so a strike leaks into
  unmasked neighbours on both sides. The grade zone is every unmasked
  sample within kerb_smear_grade_s = 0.20 s (CONFIG) of a masked sample.
  A cornering-stiffness sample is graded kerb-affected (internal name
  near_strike) when its regression window (8.1) contains a grade-zone
  sample. User-facing, mask and grade mark are one thing: KERB-AFFECTED =
  inside the mask or carrying the grade mark. The 0.20 s is the measured
  force smear past a strike (p75 0.25 s past a +/-0.05 s mask edge, 15
  Nurburgring and 2 Paul Ricard sessions), expressed from the +/-0.10 s
  edge of the dilated mask.

Where it acts:
- ESTIMATION (masked samples): no CS value at masked samples (8.1);
  masked samples are outside the Pacejka fit population, the R
  derivation, the NIS gate population and the sign check (4, 5); outside
  the stability regression population (10); outside the
  understeer-gradient population (11). The kinematic integration (3)
  and the EKF (5) run through masked samples.
- JUDGEMENT (graded samples): the per-phase CS statistics that feed
  classification skip the graded samples (12.2). The thresholds were
  derived on exactly this population (15). The CS arrays themselves keep
  every sample.
- DISPLAY: ONE switch, CONFIG corner_trace_display.kerb_exclusion_display
  = "grey" (alternatives "exclude" and "off"), for the trace windows, the
  figure exports and the diagnostics renders: CS traces draw kerb-affected
  samples grey (or leave them out, or draw them normally); the tyre-curve
  scatter draws them hollow (or leaves them out, or filled) on the same
  definition (core/trace_display.kerb_scatter_split). Every view carries
  ONE caption (core/trace_display.kerb_exclusion_caption): a plot-mode
  prefix (Grey / Not drawn / Drawn normally), then "kerb-affected samples
  excluded from verdict statistics, retained in CS arrays", then the
  count (n = X, Y %). Masked spans
  show as the exclusion grey band. corner_trace_display.kerb_gap_bridge =
  true draws a dashed bridge across kerb holes in the CS and stability
  traces (display only).
- Not applied: the wheel-load straight-line fits (9.4) and the static load
  model; the LS chain uses its own az-coincidence guard (14.2).

Chair: DOMAIN IMPROVEMENT -- the chair tooling has no kerb treatment.
Tier B. The kerb detector was validated on Nurburgring JST outing 29 (it
catches the strikes that drive the deep CS tail).

### 2.6 In-lap and out-lap exclusion

The stability regression always excludes out- and in-laps, independent
of any lap selection: cold tyres break the cross-lap stationarity the
regression assumes. The Pacejka fit, the NIS gate and the understeer
gradient use analysis laps only (1). DOMAIN IMPROVEMENT (stability).


## 3. Kinematic sideslip (the seed and the fallback)

Method. Kinematic identity a_y = v (d beta/dt + r) (Rajamani ch. 2, as
cited in the module docstring), integrated:
    d beta/dt = a_y / v - r          (moving samples, else 0)
    beta_raw  = cumulative sum of d beta/dt x (1 / grid rate)
    beta_kin  = high-pass(beta_raw), set to 0 where not moving
The high-pass is a washout that removes integration drift: CONFIG
stability_estimation.beta_washout_cutoff_hz = 0.05 Hz, 4th-order
zero-phase Butterworth (2.4). The washout is signal conditioning (Tier
B); the identity is Tier A.

Role in production: (1) the seed of the per-session tyre fit (4); (2)
the per-sample replacement whenever the EKF diverges (5); (3) the
session-wide fallback when the fit is degenerate or the NIS gate fails.

Accuracy: sideslip_angle Level 1.

Chair: FORCED ADAPTATION -- the chair tooling reads a beta channel that is
measured, state-estimated or logged depending on the source format; no
sideslip measurement is available on the GT3R, so beta is estimated.


## 4. Per-session Pacejka tyre fit

Method anchor (verbatim, modules/tyre_fit_auto.py header):
"Lateral-only axle fit (Pacejka 2nd ed.): pure side slip is the base
curve, Eq. 4.49 / 4.E19 with zero shifts and free D (sec. 4.3.1;
regression per sec. 1.2.1); an effective axle curve absorbing load
transfer is valid for quasi-steady turning, peak may vanish (sec. 1.2.2).
Combined slip only scales it by G_yk = 1 at zero slip ratio (Eqs.
4.64-4.69): moderate Fx is harmless, the large-Fx tails (front limit seen
under braking) are the known bias."

Model, per axle (modules/tyre_model_pacejka.py; general form also
Rajamani ch. 13):
    Fy(alpha) = D sin(C arctan(B alpha - E (B alpha - arctan(B alpha))))
B stiffness factor, C shape factor, D peak force [N], E curvature
factor. Odd in alpha. The analytic slope dFy/dalpha is used by the EKF.

Fit, once per session (tyre_fit_auto.fit_session_pacejka, mode
per_session):
- Data: alpha_f, alpha_r from the slip-angle definition (6) evaluated on
  the KINEMATIC beta (3); Fy_f, Fy_r from the force split (7); both after
  the 2 Hz conditioning.
- Fit population = base mask AND speed window:
  - base mask: moving, outside the kerb mask, on analysis laps;
  - speed window: keep samples whose speed lies inside [p1, p99] of the
    speeds of moving samples inside corner instances on analysis laps,
    per session. CONFIG tyre_fit_auto.fit_speed_window_pct = [1, 99]. It
    drops the fastest, mostly straight-line, near-zero-slip stretches.
    The window narrows the curve fit only; the R derivation, the NIS
    gate and the sign check keep the base mask. The derived km/h edges
    are written to the fit manifest.
- Estimator: joint least squares of (B, C, D, E) per axle, Powell
  minimisation from the chair's start guess CODE tyre_fit_auto.
  PACEJKA_START_GUESS = (12.0, 1.9, 8000.0, 0.97). Free D (CONFIG
  tyre_fit_auto.load_normalised_fit_enabled = false; tyre_fit_mode =
  "per_session").
- Degenerate (session falls back to kinematic beta): Powell not
  converged or D <= 0 on either axle.
- Reported, not acted on: residual RMS, the curve's peak slip angle (sign
  change of the slope on a 0.01-89.9 deg grid) and whether it lies inside
  the visited range (p99 |alpha|).
- Plausibility gate (tyre_fit_auto.plausibility_gate, after the status
  verdict), per axle on the final curve: g1 the p1..p99 span of the fit's
  slip-angle data >= min_alpha_span_deg = 3.0; g2 the linear stiffness at
  zero slip B*C*D inside [stiffness_min_npr, stiffness_max_npr] = [30000,
  600000] N/rad; g3 the curve's maximum (numerical argmax, positive slip
  side) inside [peak_min_deg, peak_max_deg] = [3.0, 15.0] deg, no peak in
  the grid fails (CONFIG tyre_fit_auto, Level 1 calibration tunables). Any
  failure makes the fit degenerate (kinematic fallback) with the failed
  checks in degenerate_reason. A garbage catch, not ridge adjudication;
  magnitudes after Bechtloff (single-track reference stiffnesses) and a team
  rule of thumb (front peak about 8 deg). The per-axle verdict stays in the
  in-memory manifest only.

Noise model for the EKF, derived per session (same function):
- R_ay (lateral-acceleration measurement variance): from the fit
  residuals, converted to acceleration by dividing by m, combining both
  axles with their measured inter-axle correlation rho:
  R_ay = s_f^2 + s_r^2 + 2 rho s_f s_r.
- R_r (yaw-rate measurement variance): mean square of the yaw-rate
  innovation of an interim EKF run with R_ay fixed and R_r seeded from
  CONFIG tyre_fit_auto.r_yaw_rate_interim_seed_var = 3.046e-06.
- 2-D sweep around both: R_ay x r_sweep_ay_scales = [1.0, 0.3, 0.1,
  0.03, 0.01], R_r x r_sweep_yaw_scales = [1.0, 2.0, 4.0, 8.0, 16.0]
  (CONFIG). For each of the 25 points the per-channel NIS exceedance (NIS
  above the chi-square 95 % bound, df 1) is measured on the base mask, and
  the sign check is run on that point's own beta.
- Selection (2026-10-11): on the ay exceedance only. Candidates are the
  points with f_ay inside [r_sweep_ay_band_low, r_sweep_ay_band_high] =
  [0.03, 0.15], ordered by distance to the band centre; the first whose own
  sign-check fraction reaches sign_check_marginal_fraction = 0.7 is
  chosen. None in band, or none passing the guard -> the config default R
  (r_sweep_default_ay_scale = 0.1, r_sweep_default_yaw_scale = 16.0) and the
  status at most marginal (marginal_reason r_sweep_no_in_band /
  r_sweep_guard_fail). The yaw exceedance is logged per point but not
  selected on: under the corrected re-entry its floor over the grid is
  0.21-0.39 on the reference exports -- single-track model mismatch, not
  sensor noise. The former "nearest overall" pick is gone (it chose
  ay x1 / yaw x1 on Paul Ricard, where the sign check fell to 5/15). The
  manifest records the selection and the guard trace. CONFIG Level 1.
- Fit status (CONFIG tyre_fit_auto): degenerate if the sign-check
  fraction < sign_check_degenerate_fraction = 0.5 or the ay NIS
  exceedance > nis_gross_miscalibration_fraction = 0.5 (ay only, 2026-10-11);
  marginal if the sweep fell back, the sign-check fraction <
  sign_check_marginal_fraction = 0.7, or the yaw exceedance >
  yaw_mismatch_ceiling = 0.75 (soft ceiling, never degenerate; Level 1);
  else ok. Every degenerate status carries degenerate_reason, every
  marginal one marginal_reason. The sign check counts, over racing corners
  (speed class not low), how often the median EKF beta has the sign
  opposite to the median a_y. The same configured value 0.7 is both the
  sweep guard and the marginal limit.
- Q and P0 (process and initial covariance): CONFIG tyre_fit_auto.
  q_beta_var = 9.632e-07, q_yaw_rate_var = 2.408e-07, p0_beta_var =
  p0_yaw_rate_var = 3.046e-04; beta_hard_bound_deg = 15.0;
  nis_window_samples = 20, nis_chi2_bound = 5.99, nis_flag_fraction =
  0.5 (section 5).

Accuracy: the fitted curve is a per-session model fit; sideslip stays
Level 1.

Chair: the axle-level Magic Formula fit on vehicle data and its start
guess are the chair's method. FORCED ADAPTATION: the slip angles come
from the kinematic seed (slip source forces it -- no measured beta); one
speed window instead of the chair's four speed-band curves, because the
EKF consumes one signed curve per axle (window edges are vehicle
parameterisation). SetupTool's own: the per-session R derivation and
sweep and the fit status rules. Tier A (fit, Pacejka), Tier B (speed
window, sweep, status).


## 5. EKF sideslip observer, NIS gate, fallback

### 5.1 Observer

Method (modules/sideslip_ekf_pacejka.py, whose header states it; moved
there from the Dugoff observer when that was removed, 2026-10-17):
single-track model, Rajamani sec.
2.3 / 2.6 and Ulsoy/Peng/Cakmakci sec. 14.1 / 14.3; extended Kalman
filter, Rajamani ch. 14 (chapter confirmed for the 2nd ed.; edition and
page still to verify).
- State x = [beta, r]; input delta_f; measurement z = [r_meas, a_y,meas].
  Longitudinal speed scheduled per sample, V_x = max(v, 5.0 m/s).
- Process: d beta/dt = (Fy_f + Fy_r) / (m V_x) - r;
  dr/dt = (a Fy_f - b Fy_r) / I_z, with Fy from the session's Pacejka
  curves at the small-angle slip angles alpha_f = delta_f - beta - a r /
  V_x, alpha_r = -beta + b r / V_x. Explicit Euler at the grid step.
  Jacobian F evaluated at x(k|k), measurement Jacobian H at x(k+1|k),
  tyre slopes from the analytic Magic Formula derivative.
- Measurement model: h = [r, (Fy_f + Fy_r) / m].
- Divergence, per sample: the share of the last tyre_fit_auto.nis_window_s
  = 0.4 s of samples (converted at the grid rate) whose combined NIS exceeds nis_chi2_bound = 5.99 (chi-square
  95 %, df 2) is above nis_flag_fraction = 0.5 (NIS-window divergence), OR
  |beta| > beta_hard_bound_deg = 15.0 deg (hard-bound divergence). The 15
  deg bound is physical (a diverged filter exceeds it by orders of
  magnitude), not taken from the kinematic beta's range.
- Re-entry (2026-10-07/08), the kinematic beta never enters the state or
  the output: NIS-window divergence keeps the filter's own state and
  resets P to P0 (after Goblirsch et al. 2024, sec. III-D, and Rajamani
  sec. 14.4.2); hard-bound divergence resets beta to 0, the yaw-rate state
  to the measurement and P to P0 (reset to zero, never to another
  estimate, after Villano et al. 2021, sec. 3.1). Hard bound takes
  precedence. Keeping an out-of-range state ran away in validation, hence
  the zero reset. Before 2026-10-07 both cases wrote the kinematic beta
  into the state and the output.
- Not moving: state and P reset, beta = 0.
- Output used downstream: beta_with_fallback -- the post-re-entry state
  (the name is kept for the callers; there is no per-sample fallback any
  more).
- Vehicle quantities: m, a, b, I_z (Level 1, or Level 2 for m, a, b with
  a session weighing).
- [2026-10-16/17: the former sample-count window nis_window_samples
  (0.2 s at 100 Hz) is now the duration nis_window_s (Class A fix 3); the
  key was removed with the Dugoff mirror.]

### 5.2 Session gate (modules/nis_gate.py)

Why: the fitted curve may not represent the session's tyre; then the
EKF beta must not be trusted.
- Health score = share of base-mask samples whose trailing window of
  nis_gate.nis_window_s = 0.4 s (CONFIG, converted to samples at the grid
  rate) has a combined-NIS exceedance share inside [nis_gate.nis_band_low,
  nis_band_high] = [0.03, 0.15].
- Verdict: pass if score >= threshold_use_ekf = 0.08; warn if >=
  threshold_warn = 0.01; fail otherwise or if the score is NaN (CONFIG
  nis_gate). The gate catches divergence, not fit quality; thresholds are
  labelled provisional in config (two real sessions).

### 5.3 Dispatch (tyre_fit_auto.resolve_sideslip_beta)

CONFIG stability_estimation.sideslip_source = "ekf_auto_pacejka".
- Fit degenerate -> kinematic beta for the whole session.
- Gate fail -> kinematic beta for the whole session.
- Gate pass or warn (fit ok or marginal) -> EKF beta_with_fallback.
- Degenerate includes a failed tyre-curve plausibility gate (4) and the
  ay-only gross-miscalibration trigger; the reason text says which.
The fallback and its reason are recorded in the analysis payload and
shown in the estimator status line, never silent. The same beta feeds
the slip angles (6) and the stability regression (10).

Accuracy: sideslip_angle Level 1 (a per-session model fit, not a
measured, logged or tabulated source).

Chair: SetupTool's own observer, FORCED because no sideslip measurement is
available on the GT3R.
Tier A (model, EKF), Tier B (gate, sweep).


## 6. Slip angles

Method (stability_analysis.estimate_slip_angles; single-track slip
angles, Werner 2021 sec. 2.2.3 convention):
    v_x = v cos(beta),  v_y = v sin(beta)
    alpha_f = delta_f - arctan((v_y + a r) / v_x)
    alpha_r = -arctan((v_y - b r) / v_x)
set to 0 where not moving, then 2 Hz zero-phase low-pass (2.4). The
EKF's internal model (5.1) uses the small-angle form of the same
definitions.

Accuracy: inherits sideslip (Level 1), steering (Level 1 or 4), speed
(Level 1), yaw rate (Level 3), CoG position (Level 1 or 2).

Chair: the chair's definition as-is; FORCED ADAPTATION in the beta it is
fed (estimated, not measured). Tier A.


## 7. Axle lateral forces (Fy split)

Method (stability_analysis.estimate_lateral_forces; 2-DOF planar
balance, Milliken RCVD ch. 5 (sec. 5.7), Rajamani sec. 2.3 (Eqs. 2.21 /
2.22)):
    Fy_f = m a_y w_f + I_z (dr/dt) / L,    Fy_r = m a_y - Fy_f
w_f = front static weight fraction from the corner weights; dr/dt = raw
numerical gradient of r (not the smoothed Module-5 yaw acceleration, so
the 2 Hz filter downstream does not double-filter). Set to 0 where not
moving, then 2 Hz zero-phase low-pass.

Fy_f and Fy_r together carry only two measured quantities (a_y and
dr/dt); any fit against both axles inherits that coupling.

Accuracy: lateral_force_split Level 1 (capped by I_z and the corner
weights).

Chair: construction identical to the chair tooling's. In the deviation
taxonomy the Level-1 Fy split is a FORCED ADAPTATION (no measured axle
forces). Tier A.


## 8. Cornering stiffness ratio (CS) regression

### 8.1 Method

Framework: Werner 2021, cornering stiffness ratio. Werner evaluates
effective stiffness from a tyre model; here the effective stiffness is
estimated directly from logged Fy and alpha by windowed regression
(stability_analysis.estimate_cornering_stiffness). Per axle, per sample
i (moving and outside the kerb mask):
1. Window [start, i): start = i - n_min, n_min = max(cs_min_window_
   samples_floor, round(cs_min_window_s x grid rate)) = max(5, round(0.1
   x rate)) = 10 at 100 Hz. The window widens backward until the
   alpha span in it reaches cs_min_slip_angle_span_rad = 0.02 rad, or
   until its track length reaches cs_max_window_m = 53.0 m (or lap
   distance is NaN / resets). Span not reached -> no value.
2. C_window = OLS slope of Fy on alpha over the window, with its R^2.
3. C_section = weighted mean of the OLS slopes of the monotonic alpha
   sections inside the window; a section's weight is a smooth step of its
   span on [0, 0.02 rad] with exponent CODE SPAN_WEIGHT_EXPONENT = 4.
4. C_alpha = w C_window + (1 - w) C_section, w = smooth step of R^2 on
   [0, 1] with exponent CODE R2_WEIGHT_EXPONENT = 1 (that is, w = R^2).
5. Linear reference C_lin: whenever every |alpha| in the window is below
   cs_linear_slip_threshold_rad = 0.03 rad, C_lin is set to the current
   C_alpha; otherwise the last value is held (a running hold through the
   session).
6. CS = min(C_alpha / C_lin, 1.0), only where C_lin > 0.
Reading: 1 = linear regime; 0 = the axle's force peak; negative = past
the peak (force falling with slip).
All CONFIG values here sit in stability_estimation.

Near-strike grade mark: per 2.5, set on sample i when the window [start,
i) contains a grade-zone sample. In memory only (not persisted).

### 8.2 Exclusions at this stage

Moving gate (2.3) and kerb mask (2.5) decide which samples get a CS
value. A window ending outside the mask may span masked samples; the
grade mark is what catches that. Out- and in-laps get CS values too;
only analysis-lap corners are ever read (12).

### 8.3 Accuracy, chair, tier

Accuracy: CS is a ratio of two slopes of the same Level-1 force signal
against the same Level-1 slip angle; constant scale errors in m largely
cancel in the ratio.

Chair: the chair's method as-is in the 2 Hz 4th-order input filter, the
0.02 rad span, the 0.1 s minimum window, the weighting exponents 1 and 4
and the ratio construction. Adapted: cs_linear_slip_threshold_rad = 0.03
rad is re-derived on this car's own Fy-alpha cloud (the chair's value is
0.021 rad); cs_max_window_m = 53.0 m is SetupTool's own locality cap in
track metres; the sample-rate-resolved minimum window and the floor of 5
samples are NEUTRAL ENGINEERING; kerb mask and grade mark are DOMAIN
IMPROVEMENT. Relative to Werner: windowed regression on logged data
instead of a tyre-model evaluation (no validated tyre model for this
car). Tier A (framework), Tier B (windowing, blend, hold).


## 9. Vertical loads (side branch, not a verdict input)

Why it is in production: per-axle normalised force fy_norm = Fy_filt /
Fz (display, and the thesis g-g figure) and the reconstruction cascade's
session-fitted aero coefficient. CS and stability never receive Fz;
the verdict chain is load-independent by construction.
CONFIG stability_estimation.vertical_load_source = "measured".

### 9.1 Voltage path and zeroing (parser, before the quality gate)

At one event (Nurburgring, logger serial 221) the damper gauges were
zeroed against a different reference than at every other event of this
car, so the raw force sits tens of kN off and the range gate would fail
the channels.
- channels.json event_zero_entries.NBR_221: per-corner zero_N
  (log_dms_dam_fl 58639.9 N, fr 59466.3 N, rl 58419.7 N, rr 58906.6 N),
  with file spread and wheel-level uncertainty u_wheel_N (176.9, 332.2,
  389.2, 140.3 N).
- Derivation (voltage path): per file and corner, the offset of the
  gauge-versus-raw-input-voltage map at this event minus the same car's
  Paul Ricard map offset; median over the event's 16 files.
- Applied only if (a) the header's TrackName and logger serial match and
  (b) the channel's raw mean minus zero_N lies inside the channel's own
  range gate (guard against a re-zeroed gauge; no new tunable).
- Accuracy: "L4 with an L2 zero" (L4-rel): the logged sensor and the
  motion-ratio table are Level 4, the zero is an event measurement
  (Level 2).
- Separate and still 0: CONFIG wheel_loads.pushrod_offset_{fl,fr,rl,rr}_N
  = 0.0, the static gauge-check calibration (Level 1).

### 9.2 Damper-derived wheel load (Level 4), per corner

Method (modules/wheel_loads.py; Segers ch. 9, pushrod force to wheel via
motion ratio; ch. 10, geometric versus elastic load transfer). Four
additive terms:
1. sprung: (pushrod force - offset) x MR(travel), MR from the
   manufacturer motion-ratio table (clipped at its ends);
2. anti-roll bar: half the left-right travel difference x ARB rate / mean
   MR, opposite signs left and right (ARB position from the setup, else
   CONFIG wheel_loads.arb_position_fallback = 4);
3. unsprung lateral transfer: m_unsprung a_y h_u / track (bypasses the
   gauge);
4. geometric (roll-centre) transfer: m_sprung,axle a_y z_rc / track (the
   elastic share is already in term 1).
Bump rubbers are not modelled (under-reads at extreme compression).
A corner is valid when both its force and travel channels are "valid",
units are recognised, and neither is dead: CONFIG wheel_loads.
dead_channel_std_max_force_N = 50.0, dead_channel_std_max_travel_mm =
1.0 (std over the session).

### 9.3 Reconstruction cascade, per corner and sample

damper (Level 4) -> reconstructed = axle total - measured axle mate
(Level 1; heave and pitch only, single-wheel events and warp are
unobservable) -> static split (Level 1). Axle totals are then re-summed
from the four wheels. The source tier is recorded per sample.

### 9.4 Session-fitted aero coefficient c_session (Level 2)

Used only to build the axle totals from which a single missing corner is
reconstructed (9.3); never written to config.
- Joint least squares on straight-line samples: total measured load(v) =
  S + c_session v^2, so the intercept carries the static weight
  (mass_session = S / g) and all speed dependence goes to c_session.
- A missing corner in the fit data is proxied by its mate x the static
  corner-weight ratio; only real damper data feeds the fit.
- Front static share from straight-line means; aero split from CONFIG
  wheel_loads.aero_front_fraction = 0.25 (Level 1, straights carry no
  signal for it).
- HOME FLAG: the straight-line gates are literals in
  wheel_loads.estimate_session_corrected_axle_totals: fit population
  |a_y| < 1.5 m/s^2, front-share population |a_x| < 0.5 and |a_y| < 0.5
  m/s^2. They are calibration gates and have no config key.

### 9.5 Static model (Level 1, innermost fallback)

stability_analysis.estimate_vertical_loads: static axle split + aero
(inert: lift coefficient 0, Level 1 placeholder) + longitudinal transfer
m a_x h / L; per wheel +/- m a_y h / track / 2 per axle (independent
per-axle lateral transfer, no roll-stiffness apportionment). CoG height
and track widths are unsourced placeholders.

Accuracy: per sample (9.3); vertical_load_split and per_wheel_load_split
Level 1 on the static path.

Chair: static model chair-identical. Damper path, zeroing, cascade and
c_session are SetupTool's own (no chair counterpart). Tier A (Segers),
Tier B (guards, gates, fit).


## 10. Yaw-moment stability (Module 5)

Method. Concept and sign: Werner 2021 sec. 2.2.3 -- the stability moment
derivative dMz/dbeta is the measure of open-loop stability, positive =
restoring. Target relation Mz = I_z (d^2 psi/dt^2) + D_psi (d psi/dt)
(Werner sec. 4.5.2, Eq. 4.3); the damping term D_psi is not computed
(inertial regressand only). Front versus rear saturation as
controllability versus stability loss: Hoffman et al. 2008, sec. 2;
saddle-node framing as motivation only (Ono et al. 1998), no bifurcation
analysis. Estimator after the chair tooling
(modules/yaw_stability.py):
1. Yaw acceleration: numerical gradient of r, then a centred rolling
   mean over max(5, round(yaw_stability_accel_window_s x rate)) samples,
   made odd (15 at 100 Hz), min_periods = window // 3.
   Mz_inertial = I_z x that yaw acceleration.
2. Local regression over track distance, all laps pooled: grid every
   yaw_stability_grid_step_m = 2.0 m of lap distance; per grid point all
   samples within +/- yaw_stability_window_m = 55.0 m, Gaussian weights
   with sigma = window / CODE GAUSS_SIGMA_DIVISOR = 2.5 (22 m).
3. Regression Mz_inertial ~ beta + delta_f + v + a_x + a_z (a_z dropped
   if unusable), predictors centred and scaled by their weighted std,
   ridge yaw_stability_ridge = 1.0. Skipped if fewer than
   yaw_stability_min_samples = 90 samples or weighted std(beta) <
   yaw_stability_min_beta_std_rad = 0.001 rad.
4. dMz/dbeta = beta coefficient / beta scale [Nm/rad] x pi/180 ->
   Nm/deg; grid values linearly interpolated onto every sample's lap
   distance.
All six values: CONFIG stability_estimation.yaw_stability_*, the chair's
defaults; changing any of them means re-deriving thresholds.

Exclusions (regression population only, by NaN-ing samples): not moving,
kerb mask, out- and in-laps. The result is a field over track position:
every sample with a finite lap distance receives the value at its
position, masked samples included. No lap distance -> no stability
value.

Accuracy: I_z Level 1 (m a b estimate, method ceiling 1); beta Level 1;
the stability value inherits the weakest input.

Chair: estimator and its six values as-is. FORCED ADAPTATION:
raw-yaw-rate path (the chair's pre-smoothed input is not available).
DOMAIN IMPROVEMENT: kerb and in/out-lap exclusion. Not ported: the
chair's time-anchored fallback without lap distance (no verdict
instead). The measure is chair-conform (inertial regressand, no damping
term). Tier A.


## 11. Understeer gradient (session balance line, beta-free)

Method (modules/balance_gradient.py; Rajamani sec. 3.2, Eq. 3.23:
delta = L / R + K a_y with 1 / R = r / v). K is the OLS slope of
    y = LP(delta_f) - delta_0 - L LP(r) / LP(v)     on     x = LP(a_y)
LP = 2 Hz zero-phase low-pass (2.4); delta_0 = median delta_f on racing
samples outside corner brackets (steering zero); L = a + b. Reported in
deg/g.

Quasi-steady population (CONFIG understeer_gradient):
- racing: moving, outside the kerb mask, on analysis laps;
- inside a canonical corner bracket (12.1);
- |LP(r) LP(v)| above the session's Otsu split of |a_y| over racing
  samples, otsu_bins = 256 (standard between-class-variance threshold,
  no fixed g value);
- |d LP(r) / dt| < psidd_max_radps2 = 0.15 rad/s^2;
- |LP(a_x)| < ax_max_mps2 = 2.0 m/s^2;
- |a_y| < ay_clip_g = 5.0 g.

Range: 95 % corner-traversal cluster bootstrap (traversals, not samples,
are resampled, because samples within one pass are not independent),
bootstrap_draws = 200, bootstrap_seed = 20261002. The reading word ("the
car understeers" / "oversteers") appears only when the range excludes
zero, else "balance not resolved from neutral".

What it is and is not: the car's steady-state balance over the
quasi-steady population; a corner verdict (13) is the local saturation of
one axle in one phase -- the two measure different things and both
stand. It is a readout, never a constraint on the fit. Shown as one UI
line beside the verdicts.

Accuracy: understeer_gradient Level 2 (session measurement); wheelbase
the only vehicle constant in it; steering via the steering_angle node
(Level 4 with the table).

Chair: SetupTool's own (no chair counterpart). Tier A (Rajamani), Tier B
(population gates).


## 12. Phases and aggregation

### 12.1 Corners, phases, cross-lap identity

modules/corner_analysis.py, thresholds in channels.json corner_detection
(Tier B):
- Bracket: enter when the smoothed |steering-wheel angle| >
  steering_entry_threshold_deg = 25 OR smoothed |a_y| > ay_entry_
  threshold_g = 0.6; exit when |sw| < steering_exit_threshold_deg = 15
  AND |a_y| < ay_exit_threshold_g = 0.35; duration >= min_corner_
  duration_s = 0.8; moving-average smoothing over smoothing_window_
  samples = 10 native samples; same-direction brackets closer than
  bracket_merge_gap_s = 0.6 merged (chicanes stay separate).
- Apex = peak |a_y| in the bracket, rejected below lateral_g_apex_
  threshold = 0.3 g. Speed class from the apex speed: low < low_max = 80,
  medium < medium_max = 150 km/h, else high (channels.json
  corner_speed_thresholds).
- Phases: entry_1_brake from the last sample at throttle >=
  brake_throttle_max_pct = 95 % before turn-in to turn-in; entry_2_turnin
  to the apex; apex_3 = the apex instant; exit_4 from the apex until the
  steering unwinds to steering_unwind_fraction = 0.5 of its post-apex
  peak; exit_5 to bracket end.
- Cross-lap identity: brackets mapped to lap distance; linked when they
  overlap >= bracket_overlap_min_fraction = 0.3 of the shorter; same-lap
  collisions split from the lap with the most brackets, two passes.
  Only laps within lap_time_representative_factor = 1.05 x the fastest
  can seed or shape a canonical corner.
- Canonical corner: each boundary is the median over representative
  laps, then realised on EVERY analysis lap by inverting its lap
  distance; windows overlapping by more than canonical_overlap_max = 0.1
  are partitioned at the minimum of the cross-lap median |a_y| profile
  (grid canonical_boundary_grid_step_m = 2.0 m). Brackets longer than
  compound_corner_min_length_m = 300 m are tagged compound. Speed class
  per stable corner from the median apex speed.

### 12.2 Per-phase statistics (stability_analysis.summarise_corners)

Per corner, per analysis lap, per phase:
- Samples: moving samples inside the phase's time window; apex_3 (an
  instant) uses +/- apex_half_window_samples = 5 grid samples (CONFIG
  stability_estimation; a sample count, see 2.1).
- CS statistics (median, p25, p75, n) on the CLEAN population:
  graded kerb-affected samples set to NaN; masked samples are already NaN
  (8). Gate: fewer than cs_phase_min_valid_samples = 5 finite samples ->
  NaN. Each CS block also carries n_kerb_excluded (finite samples left
  out as kerb-affected; display only), so the per-phase table states
  "no clean data (kerb-affected excluded, n = K)", "too few clean
  samples (n = c; kerb-affected excluded, n = K)" or "too few samples
  (n = c)" instead of an empty cell. The table's CS cells read the same
  statistic as the classifier (one accessor, OutingForm._verdict_cs_stat):
  the apex shows as "Apex region (+/-25 m)" (the CS the verdict reads) and
  "Apex (+/-5 samples)" (stability, loads, LS of the apex instant; its own
  CS slice is not displayed).
- Apex region for CS: moving samples within +/- cs_apex_region_half_
  length_m = 25.0 m of the canonical apex lap distance, limited to the
  instance's own time span, same clean population and gate. Classification
  reads apex_3 CS from here.
- Stability statistics: same count gate (cs_phase_min_valid_samples
  reused); entry_1_brake additionally NaN when the phase's peak front
  brake pressure stays below stab_phase_no_braking_floor_bar = 3.0 bar
  (no real braking).
- Kerb fraction per phase is reported (share of moving samples in the
  mask).
- Load and LS statistics are added for display and the decision frame;
  LS has its own gate ls_phase_min_valid_samples = 5.
All CONFIG values here sit in stability_estimation.

### 12.3 Cross-lap aggregation (recommendation.aggregate_by_corner)

Per stable corner and phase: CS across laps combined by CONFIG
classification.cs_cross_lap_aggregation = "worst_lap" (the minimum of
the per-lap phase medians; the apex region likewise); stability across
laps by the median; speed class modal (ties toward medium, then lower).
Composed with the classifier's minimum over phases, the CS corner value
is the minimum over (lap, phase).

Where each level is used: the verdict cards classify each corner on each
lap (one card per corner-lap, worst phase within that lap); the track-map
markers and the decision frame classify the worst-lap aggregate. The
thresholds were derived on worst-lap corner values (15).

Chair: SetupTool's own (segmentation, phases, apex region, aggregation).
Tier B.


## 13. Classification

### 13.1 Rule (ui/views/outing_form.py _classify_corner)

Inputs per corner: the five phases' CS medians (apex_3 from the apex
region) and stability medians.
- Worst values: worst_f, worst_r = the lowest front / rear CS median over
  the phases; worst_stab = the lowest stability median.
- Axle tiers (CONFIG classification):
  front STRONG if worst_f < STRONG_CSF = -0.646;
  front MODERATE if -0.646 <= worst_f < MODERATE_CSF = -0.285;
  rear STRONG if worst_r < STRONG_CSR = -0.404;
  rear MODERATE if -0.404 <= worst_r < MODERATE_CSR = -0.131.
- Open-loop unstable if worst_stab < stab_neg_thresh_Nm_per_deg = 0.0
  (Werner sec. 2.2.3 sign criterion; explicitly not a criticality
  threshold -- skilled drivers operate there).
- Verdict word: front -> "understeer", rear -> "oversteer", at the
  phase of that axle's worst value. Precedence when both fire: rear
  STRONG, front STRONG, rear MODERATE, front MODERATE (rear wins ties:
  rarer and more consequential). "open-loop unstable" is appended at its
  own phase.
- Severity (two-signal logic):
  strong = any STRONG CS AND open-loop unstable;
  moderate = any STRONG CS alone, or any MODERATE CS AND open-loop
  unstable, or open-loop unstable alone;
  normal = everything else (including a MODERATE CS alone, which still
  names its verdict word).
- MARGINAL: the verdict-driving axle's worst value lies within cs_margin
  = 0.11 (CONFIG classification.verdict_stability_margin.cs_margin) of
  the nearer of that axle's two thresholds; the tooltip states that the
  verdict can flip under a realistic ~1 % input change. A qualifier, not
  a tier.
- Text: the SHORT verdict string (e.g. "oversteer @ ... [MARGINAL]") is
  the machine interface the decision frame and the rule engine parse; the
  card shows condition, phase, number, severity ("Oversteer, early exit,
  rear CS -0.52, strong"; "Open-loop unstable, early exit, -40 Nm/deg");
  MARGINAL / UNCAL qualifiers sit in the tooltip.
- Calibration markers: "[UNCAL]" when stability_estimation.
  sideslip_source differs from classification.thresholds_calibrated_
  for_sideslip_source; a stability-threshold marker when it differs from
  stab_thresh_calibrated_for_sideslip_source. Both are
  "ekf_auto_pacejka" today, so neither fires. The comparison is on
  config only: a session that fell back to kinematic beta at run time
  (5.3) is classified against the same thresholds without a marker; the
  fallback shows in the estimator status line.
- Colours: severity strong / moderate / normal -> BAD / WARN / OK
  (ui/style.py).

The PDF export calls this same classifier, so the two cannot disagree.

### 13.2 Phase-detail colours (display)

ui/views/outing_form.py _stability_colour colours individual phase
values:
- HOME FLAG and stale values: CS colour bands are literals in code --
  front OK >= 0.25, WARN >= 0.10, else BAD; rear OK >= 0.35, WARN >=
  0.20, else BAD. They are positive values from an earlier threshold set
  and do not follow the CONFIG thresholds of 13.1: a phase value of, for
  example, +0.05 shows BAD while the verdict reads normal.
- Stability: WARN boundary = stab_neg_thresh x CODE
  STAB_COLOUR_WARN_FRACTION = 0.4. With stab_neg_thresh = 0 the WARN band
  has zero width (OK above 0, BAD at or below 0).

### 13.3 Exclusions active at judgement

Analysis laps only; clean CS population (kerb-affected out); phase count
gates (12.2); the no-braking gate on entry_1_brake stability.

Accuracy: verdicts inherit CS and stability (Level 1 chain); MARGINAL
expresses the measured sensitivity (+/-1 % input errors move 90 % of
corner values by less than 0.075; no verdict flips proportionally at
the derived thresholds).

Chair: SetupTool's own. Classification thresholds differ from any chair
value by rule. Tier A anchor for the stability boundary (Werner sec.
2.2.3); the CS thresholds are Tier B (data-derived, section 15).

### 13.4 Limit events (display and record only)

modules/limit_events.py. Single in-corner moments where an axle's CS
falls below the floor of the session's own fitted Pacejka curve: force
loss the quasi-steady characteristic cannot produce anywhere. A third
output class beside verdicts and recommendations; it feeds no verdict and
no recommendation (tested: summaries, verdicts and recommendations
byte-identical with the layer on and off).
- Line: per session and axle, min over slip angle 0..30 deg (CODE
  EVENT_FLOOR_MAX_DEG, grid FLOOR_GRID_N = 4000) of (dFy/dalpha) / (B C D).
  The 30 deg range extrapolates the curve beyond the visited range on
  purpose; the B C D normalisation sits shallower than a CS-scale floor
  would, so the line errs toward fewer events. No line when the fit is
  degenerate or absent.
- Events: in-corner stretches (canonical brackets, analysis laps, full CS
  arrays) below the line, merged across gaps < CONFIG limit_events.
  bridge_gap_s = 0.5 s within one corner instance; events shorter than
  min_duration_s = 0.1 s (one CS regression window, cs_min_window_s) are
  dropped.
- Labels: kerb-affected (a graded sample in the event or a mask sample in
  its span), recurring (same axle and corner on another analysis lap,
  minimum within recurrence_radius_m = 10 m), else single.
- Display: one line on the verdict card, a session list per corner and
  phase, a thin strip along the bottom of the CS panel in the traces.
- Counts at 0.1 s (best available): Dubai 36 events, NBR JST 29 71.
Anchor: Segers secs. 7.4-7.6 (moment-level reading of logged traces),
Pacejka secs. 1.2.2 / 4.3.1 (the curve); the event channel itself is
SetupTool's construction, Tier B segmentation on a Tier A-derived line.
Chair: none.
Layering note: the classifier lives in ui/ (a known Q-SLIM finding),
called with self = None by the decision frame and the PDF export.


## 14. Recommendation (decision frame)

### 14.1 Decision frame

The production recommendation path is modules/decision_frame.py, the
carrier of the 39 rules of config/recommendations.json as candidate
bridges, run on the Generate button after Analyse. It never applies a
change; it ranks proposals for the engineer. Tier C in structure, with
Tier B gates and engineer-elicited rule content; Segers ch. 5 anchors the
brake-bias bridge.

Order as executed (ui/views/outing_form.py):
1. Evidence (build_evidence), every item with a confidence in [0, 1]:
   - corner_verdict: the classifier (13.1) per phase on the worst-lap
     aggregate; confidence = (laps repeating the verdict at that severity
     / laps) x (laps with signal in that phase / laps); a MARGINAL verdict
     is capped at decision_frame.json intervention_evidence.abs.
     confidence = 0.8;
   - matrix_verdict: the classifier over each rule's phase group;
   - LS disambiguation of exit oversteer (traction- vs cornering-limited)
     and LS threshold evidence (14.2);
   - brake_balance_signature: front at least MODERATE while the rear
     stays normal in entry_1_brake (classifier labels, no duplicated
     numbers);
   - ABS intervention (trusted channel log_abs_pos; masking flag when
     abs_active duty cycle in entry_1_brake > heavy_duty_cycle_threshold
     = 0.5); TC evidence disabled;
   - damper motion (rate_threshold_mm_s = 1.0, min_valid_fraction =
     0.7), kerb blow-off (severity_threshold_g = 2.9571, repeat_min_laps
     = 2), driver feedback (confidence bands 0.1 / 0.75 / 1.0 for |fb| up
     to 1 / 3 / 5).
2. Candidates (generate_candidates): exit-oversteer bridge, brake-balance
   and brake-bias bridges, the matrix-rule bridges, lever bridges; then
   the eligibility gate for heavy correctors (springs, camber, toe: need
   STRONG evidence on >= 2 corners in the same axle family and direction,
   or |feedback| >= 4, camber never by feedback; a moderate rating (1 <
   |fb| <= 3, the mid feedback band) from a driver at or above
   eligibility_classes.driver_level_threshold = 5 vetoes the
   multi-corner unlock); feedback-only candidates
   (|feedback| >= 2); feedback corroboration; driver-versus-data
   disagreement (display); window-edge status from the setup sheet;
   breadth; TC safety note; speed-qualified effect class.
3. Score (six-term additive cost function, decision_frame.json
   cost_function): problem weight = severity (1.0) x severity rank (0 /
   1 / 2) x phase importance (entry_1 0.8, entry_2 0.9, apex 1.0, exits
   1.2) x confidence (the minimum over the candidate's evidence);
   change time (seconds 1.875, minutes 1.25, half hour 0.8333, garage
   hours 0.625); breadth (weight 0.0, annotation only); headroom (0.1 x
   (1 - normalised distance of the new setting from the window's
   nominal), worst action of a package); interaction (-0.1 if any
   adverse side effect fires); effect class (primary 1.0, secondary 0.6).
   Headroom and interaction together (0.2) stay below the smallest
   change-time class step (0.2083). Ties ordered shallow-first (ARB
   before diff).
4. Shortlist = proposed candidates; every other status goes to the tail
   with its reason. Per-corner conflicts resolved (platform calming,
   else higher phase importance); identical rows grouped.
5. Display hardening: session-wide lever conflicts merged into one card;
   evidence floor display_evidence_floor = 0.5 on the problem weight (a
   row passes with feedback |fb| >= 2 or >= 2 corners); then
   display_top_n = 3 proposals visible, the rest in the tail, nothing
   dropped.
Accuracy: inherits the verdicts; feedback is the driver's rating.

HOME FLAGS (decision_frame.py): the heavy-corrector feedback bypass
"|feedback| >= 4" is a literal in _apply_eligibility_gate (it mirrors
recommendations.json consistency_gate.feedback_override.
feedback_override_raw_min = 4 without reading it), and the feedback
generation floor FEEDBACK_MIN_MAGNITUDE = 2 is a code constant although
it is a calibration value.

### 14.2 LS ratio (decision-frame evidence only)

Longitudinal analogue of CS (modules/longitudinal_forces.py,
modules/longitudinal_stiffness.py; after the chair tooling): axle Fx and
slip ratio kappa, windowed dFx/dkappa over a low-slip reference, clipped
at 1. CONFIG longitudinal_stiffness: cutoff_hz = 8.0,
linear_slip_threshold = 0.015, min_speed_mps = 5.0 (chair defaults);
min_window_s = 0.4, min_window_samples_floor = 8, min_slip_span = 0.016,
max_window_m = 900.0 (re-derived on this car); rear_rolling_radius_offset
= 0.0141; a sample is excluded only when |kappa| > plausibility_kappa_
bound = 0.12 AND a kerb-like a_z disturbance occurred within the
trailing plausibility_az_window_front_s = 0.15 / rear_s = 0.6. Wheel
speeds pass a plausibility guard (CONFIG wheel_speed_guard). LS is never
a verdict tier; its thresholds live in decision_frame.json (15).
Accuracy: longitudinal_force_split Level 1.


## 15. THRESHOLD INVENTORY

### 15.1 Derived on the current estimator (2026-10-11)

CS thresholds, CONFIG classification, percentile rule, author ruling
2026-10-05: STRONG = p10, MODERATE = p30 per axle. Re-derived 2026-10-11 on
the corrected estimator (re-entry without kinematic injection, plausibility
gate, ay-only R selection with sign-check guard, ay-only gross trigger).
- STRONG_CSF = -0.784 (front p10, n 106; was -0.646)
- MODERATE_CSF = -0.265 (front p30; was -0.285)
- STRONG_CSR = -0.677 (rear p10, n 104; was -0.404)
- MODERATE_CSR = -0.323 (rear p30; was -0.131)
- Pool: seven sessions -- Dubai, Paul Ricard MLA outings 25 and 32,
  Nurburgring JST outings 21 and 29, Nurburgring qualifying CDR and MLA;
  Le Mans informational only (different tyres), Dubai least
  representative.
- Corner value: worst lap, worst of the five phases, apex_3 from the
  apex region; np.percentile, linear interpolation.
- Single population: since schema 14 the per-phase CS statistics skip
  kerb-affected samples themselves, so the former clean / full split is
  vacuous; the classifier reads the same population.
- Chain: best available -- ekf_auto_pacejka, Level-4 steering table, fit
  speed window, every session through the fresh chain. Recorded as
  classification.thresholds_calibrated_for_sideslip_source =
  "ekf_auto_pacejka".
- Meaning check (section 16): the fitted curves do not change with the
  estimator change (the fit precedes every EKF call), so the curve floors
  stand; both new STRONG values lie further below them.
- Derivation tool: diagnostics/inspect_threshold_rederivation.py,
  modes compute_fresh / counts_fresh (cache_clean_20261011).

Stability threshold, CONFIG classification:
- stab_neg_thresh_Nm_per_deg = 0.0 -- anchored, not data-derived: Werner
  sec. 2.2.3 sign criterion (dMz/dbeta < 0 = open-loop unstable),
  explicitly not criticality; author ruling 2026-10-05, retained
  2026-10-11; recorded for stab_thresh_calibrated_for_sideslip_source =
  "ekf_auto_pacejka". Pool worst-phase distribution on the corrected
  estimator: n 109, 34 negative (was 9); p5 -457, p10 -389, p15 -270
  Nm/deg. Of the 34, 23 are negative also at the fallback R 0.1/16
  (R-robust), 11 only at the ruled pick (R-sensitive). The beta-slope
  uncertainty from the ridge regression understates the true uncertainty
  because beta is itself an estimated input.

### 15.2 Retained values and severity logic

- cs_margin = 0.11 (CONFIG classification.verdict_stability_margin):
  derived 2026-09-19 from a +/-1 % sensitivity sweep on two sessions
  (Dubai, Paul Ricard v3) on the chain of that date; RE-MEASURED
  2026-10-05 on the corrected chain (steering input perturbed on the
  table's wheel-angle column): no proportional flip within +/-1 %, both
  observed flips are different-mechanism jumps (a ratio discontinuity; a
  front/rear relative comparison); value retained by author ruling.
  Its numeric value still originates from the 2026-09-19 chain.
- classification.cs_cross_lap_aggregation = "worst_lap": a method
  choice the thresholds are bound to.
- Severity logic (CODE, outing_form._classify_corner): the two-signal
  severity table and the rear-before-front precedence (13.1);
  recommendation.SEVERITY_RANK = normal 0 / moderate 1 / strong 2
  (ordinal enum).
- Severity-dependent rule settings (recommendations.json settings):
  severity_factors normal 0.0 / moderate 1.0 / strong 2.0;
  action_class.data_trigger_recommended_severities = ["strong"];
  consistency_gate min_repeat_laps = 2, min_repeat_fraction = 0.4;
  feedback_override feedback_override_raw_min = 4,
  feedback_override_scaled_min = 4.0.

### 15.3 Other decision thresholds and gates in production

- LS thresholds (decision_frame.json ls_threshold_evidence):
  STRONG_LSF = -0.79, STRONG_LSR = -0.6 (pooled p10, n = 31 corners, two
  sessions, 2026-09-19); exit_phase_confidence_discount = 0.5.
- NIS gate (CONFIG nis_gate): threshold_use_ekf = 0.023, threshold_warn =
  0.0025 (re-derived 2026-10-11 with the recorded rule on eight real
  sessions, lowest health score 0.0245; was 0.08 / 0.01).
- Tyre-fit status (CONFIG tyre_fit_auto): sign_check_degenerate_fraction
  = 0.5, sign_check_marginal_fraction = 0.7 (also the R-sweep guard),
  nis_gross_miscalibration_fraction = 0.5 on the ay exceedance only,
  yaw_mismatch_ceiling = 0.75 (soft, marginal only). Checked 2026-10-11 and
  retained: the sign-check fraction is 1.000 on all eight reference
  sessions at their picks, ay exceedance 0.05-0.12, yaw exceedance
  0.18-0.63.
- Tyre-curve plausibility gate (CONFIG tyre_fit_auto, 4):
  min_alpha_span_deg = 3.0, stiffness 30000-600000 N/rad, peak 3-15 deg.
- Kerb: kerb_z_deviation_threshold_g = 1.2 (no derived_from in config),
  kerb_dilation_s = 0.10 (2026-10-03), kerb_smear_grade_s = 0.20
  (2026-10-03); kerb blow-off severity_threshold_g = 2.9571 (two
  sessions).
- Decision frame (decision_frame.json): display_evidence_floor = 0.5,
  display_top_n = 3, ABS confidence 0.8 and heavy_duty_cycle_threshold =
  0.5 (round number), driver-feedback bands 0.1 / 0.75 / 1.0
  (provisional), conditions.not_evaluable_confidence_cap = 0.5,
  damper_motion rate_threshold_mm_s = 1.0 and min_valid_fraction = 0.7.
- Code constants acting as thresholds: FEEDBACK_MIN_MAGNITUDE = 2 and
  the literal "|feedback| >= 4" (decision_frame.py, HOME FLAG, 14.1);
  STAB_COLOUR_WARN_FRACTION = 0.4 and the CS colour literals (13.2).

### 15.4 Values still carrying a pre-current-estimator derivation

"Current estimator" = the schema-12 chain (Level-4 steering by
wheel-angle column since 2026-10-04, fit speed window since 2026-10-04,
clean classification population since 2026-10-05). The standing
re-derivation rule needs this list:
- cs_margin = 0.11 -- numeric origin 2026-09-19; re-measured and
  retained 2026-10-05 (15.2).
- CS colour bands in _stability_colour (0.25 / 0.10 front, 0.35 / 0.20
  rear) -- from an earlier, positive threshold set; inconsistent with the
  current thresholds (13.2).
- STAB_COLOUR_WARN_FRACTION = 0.4 -- sized for a negative stability
  threshold; degenerate at 0 (13.2).
- LS thresholds STRONG_LSF / STRONG_LSR (2026-09-19). The LS chain does
  not consume sideslip or steering, so the estimator changes since do
  not reach it in principle; not re-verified.
- NIS gate thresholds (2026-09-03) and tyre-fit status boundaries.
- CS estimator calibration derived 2026-09-02, before the steering
  conversion change: cs_linear_slip_threshold_rad = 0.03, cs_max_window_m
  = 53.0; cs_min_window_s = 0.1 and cs_min_slip_angle_span_rad = 0.02 are
  the chair's values, validated on the same date.
- cs_apex_region_half_length_m = 25.0 (sized on the extent of one corner's CS dip, C4) and
  stab_phase_no_braking_floor_bar = 3.0 (sized on one Paul Ricard file).
- Kerb smear reach kerb_smear_grade_s = 0.20 (measured 2026-10-03, on
  the chain before the steering change).
- Display-only: corner_trace_display.cs_ls_panel_y_min / max = -3.5 /
  1.2 (sized on Dubai at the kinematic configuration, 2026-09-01).
- Decision-frame calibrations sized on the two early sessions (kerb
  blow-off 2.9571 g, ABS duty cycle 0.5, damper motion 1.0 mm/s and 0.7).


## 16. STRONG VERDICT MEANING

On every session of the threshold pool and on both axles, the STRONG
threshold (re-derived 2026-10-11: -0.784 front, -0.677 rear) lies below
the floor of that session's own fitted Pacejka curve (the deepest slope ratio the curve produces over the visited slip
range, and also out to 30 deg; pool floors -0.325 front, -0.233 rear),
so a corner value at STRONG cannot be produced by the session's
quasi-steady tyre characteristic. STRONG therefore means "anomalous
force loss beyond the quasi-steady characteristic (kerb-shown majority
per audit; mechanism open otherwise)".


## 17. ACCURACY-LEVEL REGISTRY

As held in CONFIG accuracy_levels (static default tier) and resolved per
session by modules/accuracy_resolution.py. Default cap = best available.
Only mass, corner_weights and steering_ratio resolve live; cog_position
and steering_angle cascade from them; every other node stays at its
registry level.

- mass: Level 1, config default. Resolves to Level 2 from the outing's
  setup sheet (explicit total weight, else the sum of four entered corner
  weights); a mismatch above 1 % between the two raises a warning (HOME
  FLAG: MASS_CORNER_SUM_TOLERANCE = 0.01 is a code constant that its own
  comment calls a calibration tunable).
- corner_weights: Level 1, config default. Level 2 when all four corners
  are entered on the setup sheet (no mixing).
- cog_position: Level 1, derived from the static weight split; cascades
  to Level 2 with a session weighing (a = L x rear share, b = L x front
  share).
- yaw_inertia: Level 1, m a b bicycle-model estimate; method ceiling 1,
  never cascades. Upgrade path: a homologation value (vehicle note); a
  mass-derived I_z is not shipped.
- yaw_inertia_kalman: Level 1, reviewer placeholder; diagnostics only, no
  production consumer.
- steering_ratio: Level 1 constant; resolves to Level 4 when the
  manufacturer table (car_data.json) is present and usable (wheel angle
  read from the table, two-wheel mean).
- steering_angle: pure cascade of steering_ratio (Level 1 or 4).
- lateral_force_split: Level 1 (2-DOF balance; capped by I_z and corner
  weights).
- longitudinal_force_split: Level 1 (m a_x plus measured brake-pressure
  split; drag and rolling-resistance coefficients are 0.0 placeholders).
- sideslip_angle: Level 1 (per-session EKF/Pacejka fit with gate and
  kinematic fallback). The GPS-course Level-3 path was investigated and
  is not viable on this data; the registry defines no other upgrade (a
  reference sideslip sensor is an outlook item only).
- speed: Level 1, ecu_speed (opaque ECU calibration). GPS cross-check
  measured a scale k = 1.012; applying it would be an estimator change
  and is not done.
- yaw_rate: Level 3, logged sensor, exact unit conversion.
- lateral_acc: Level 1, logged sensor, provenance assumption IMU not at
  the CoG.
- wheelbase_m: Level 1 by convention (fixed chassis dimension).
- vertical_load_split: Level 1, static + aero + longitudinal-transfer
  decomposition (chair-identical construction).
- per_wheel_load_split: Level 1 on the static path; with measured loads
  the level is per sample -- damper Level 4, on Nurburgring "L4 with an
  L2 zero" (L4-rel).
- wheel_load_damper: Level 4, Segers ch. 9/10 decomposition, per corner
  where force and travel channels are valid.
- wheel_load_damper_reconstructed: Level 1 (axle total minus measured
  mate; capped by the Level-1 axle-total placeholders).
- yaw_damping: Level 1, diagnostic consumer only (not in Module 5, the
  payload or the UI).
- understeer_gradient: Level 2, session measurement, beta-free.
- Not a registry node: c_session (session-fitted aero coefficient),
  Level 2 per its module docstring; aero_front_fraction Level 1.


## 18. SCHEMA HISTORY

ANALYSIS_SCHEMA_VERSION (modules/stability_analysis.py) versions the
persisted analysis payload; a mismatch means no cache. Standing rule:
any estimator-input change made outside Settings bumps it.
- 1: the original payload (the code's list starts at 2).
- 2: accuracy_cap and resolved_* fields.
- 3: Fz and fy_norm statistic blocks per phase.
- 4: bracket_start_m / bracket_end_m per corner.
- 5: sideslip_source.
- 6: auto-fit modes, fit_manifest, gate_verdict, fallback_*.
- 7: ls_ratio_f / ls_ratio_r statistic blocks (display only).
- 8: apex_region per corner; grid_rate_hz in the payload.
- 9: estimator inputs changed (2026-10-03): kerb-mask time-based
  dilation, Nurburgring event zero entries.
- 10: estimator input changed (2026-10-04): Level-4 steering reads the
  wheel angle from the manufacturer table (mean of both front wheels).
- 11: estimator input changed (2026-10-04): speed window on the Pacejka
  fit population; fit manifest gains speed_window.
- 12: (2026-10-05) balance key added -- measured understeer gradient,
  Level 2; the classification CS statistics skip kerb-affected samples.
- 13: (2026-10-06) events key added -- limit events (13.4), display and
  record only; no estimator input and no summary value changed.
- 14: (2026-10-07) every CS statistic block gains n_kerb_excluded
  (display only, no value changes).
- 15: (2026-10-11) estimator change: divergence re-entry without kinematic
  injection, tyre-curve plausibility gate, R selected on the ay exceedance
  with sign-check guard and default-R fallback, ay-only gross trigger, soft
  yaw ceiling (4, 5). fit_manifest gains marginal_reason and
  r_sweep.selection / guard_trace; sweep grid points carry ay_in_band and
  sign_fraction; degenerate / fallback reason texts changed.


## 19. PRODUCT SURFACE (what the user sees and can change)

- Minimal flow: load an outing, see the laps (out / in marked), the
  recorded channels below; Analyse runs the best-available chain. No
  estimator selector (sideslip_source is config only).
- No source or model names in product text ("fitted tyre curve",
  "open-loop unstable"); anchors live in code comments, this file and the
  thesis.
- Analysis lock: every mutable input of the outing form (session, setup,
  setdown, driver feedback incl. corner count, comments, file load /
  clear, lap table, Generate, Back / Save / Delete) is disabled while an
  analysis runs; Settings refuses a save meanwhile; a result whose file
  changed during the run is discarded.
- Driver-feedback table: corner number and the five phase ratings
  (-5..+5); no worst-corner column (removed 2026-10-07; it fed only the
  legacy rule engine behind the weekend PDF export).
- Trace windows: ONE lap-selection state -- traces, kerb grey, exclusion
  band, kerb bridges, event strips, yaw rows and flags, tyre-curve lap
  samples and the tyre-curve background cloud ("Checked laps, whole lap")
  follow the checked laps, on screen and in the export. The cross-lap
  envelopes (CS p10 / p90 lines, CONFIG corner_trace_display.cs_envelope;
  yaw envelope and median) and the lap window's corner tints pool all
  analysis laps. Corner window default: CS front, CS rear, stability; the
  selected lap over the p10 / p90 lines; speed, LS, yaw and steering rate
  opt-in. Shading budget per panel: one tint family and one exclusion grey
  filled, everything else lines or strips.
- Thesis data reader: core/thesis_data.read_sidecar reads an analysis
  sidecar (data/analysis_cache) into CS front / rear, kerb-affected per
  axle, kerb mask, lap distance, per-sample lap id and the analysis laps,
  for thesis scripts; no Qt.
