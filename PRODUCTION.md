# PRODUCTION -- the SetupTool analysis method as it runs today

Dated 2026-10-21. Describes the default analysis path of branch new-laps:
tyre fit L (config tyre_fit_auto.tyre_fit_mode = "literature_bounded"),
ANALYSIS_SCHEMA_VERSION 19. [2026-10-22: sections 9 (classification rules)
and 10 (recommendation) rewritten for the recommendation overhaul, schema
21; the chain sections 1-8 are unchanged in value since schema 19.] It states what the method IS, in the order the
signal flows, so that the thesis method chapter can be written from it. How
each rule came about is recorded in DECISIONS.md (standing rulings) and
thesis_notes.md (dated findings); this file points there only where a rule
was reopened. Updated on request only; an update replaces the content.

Citations use the keys of the thesis bibliography (literature.bib), e.g.
[Pacejka.2006, p. 6]. "page to verify" marks a reference whose page is not
recorded as checked. The chair basis is the chair performance_analysis tooling
(internal).

Notation: t time; v speed; r yaw rate; a_x, a_y accelerations; delta_f
road-wheel steer angle; beta sideslip at the CoG; alpha_f, alpha_r axle slip
angles; Fy, Fx, Fz axle lateral, longitudinal and vertical force; m mass;
I_z yaw inertia; a, b CoG to front / rear axle; L = a + b. Per-axle values
carry the index f / r. Every parameter is given as value, accuracy level
(L1-L4) and tier (A/B/C); CONFIG = config/parameters.json unless named,
CODE = a method-defining constant in the module named.


## 1. Scope and data basis

Purpose. From one logged outing of a Porsche 992 GT3R, estimate per corner
and phase how close each axle runs to its lateral limit, and turn that into
understeer / oversteer verdicts and setup suggestions. No sideslip sensor,
no tyre model of the car and no measured axle forces exist; every quantity
is reconstructed from the standard logger channels plus the damper strain
gauges.

Data. One Cosworth logger export per outing (Pi Toolbox ASCII). Required
channels: ecu_speed, sclu_yaw_rate, log_asteer, log_acc_x, log_acc_y;
used besides: log_acc_z (kerb detection), lap channels, brake pressures,
throttle, damper forces and travels (vertical loads), wheel speeds, TPMS.
Channel presence, units and rates are read from each file, never assumed.
- Common grid: every channel is interpolated onto one time grid at
  min(100 Hz, slowest fast-chain channel); below 50 Hz the analysis is
  refused (CONFIG stability_estimation.target_sample_rate_hz = 100,
  min_sample_rate_hz = 50; L1, B). Windows are durations converted at the
  grid rate.
- Analysis laps: not out- or in-lap, no lap warning, lap time within 1.1 x
  the fastest valid lap (channels.json lap_splitting; B). Fits, references,
  verdicts and thresholds use analysis laps only.
- Moving gate: v > 5.0 m/s (stability_estimation.moving_speed_min_mps; L1,
  B).
- Conditioning: slip angles, axle forces and axle loads pass the same 2 Hz
  zero-phase Butterworth (order 4) low-pass (stability_estimation.
  cs_filter_cutoff_hz = 2.0, CODE BUTTERWORTH_ORDER = 4; chair as-is; B).
- Kerb mask: |a_z - 1 g| > 1.2 g, dilated by +/- 0.10 s; a CS regression
  window touching the mask or the 0.20 s filter-smear zone beyond it is
  "kerb-affected" (kerb_z_deviation_threshold_g, kerb_dilation_s,
  kerb_smear_grade_s; L1, B; DOMAIN IMPROVEMENT, the chair has no kerb
  treatment). Kerb-affected samples stay in the arrays and leave the
  verdict statistics. The 1.2 g threshold is a physical criterion on the
  vertical acceleration; the 3 % flagged share seen when it was set on the
  Dubai sample (2026-06-29) was a plausibility band, not a target. A fixed
  physical threshold gives a track-dependent flagged share: on the
  development set 1.9-8.1 % of moving analysis-lap samples after dilation,
  4.2-15.8 % with the smear zone (Le Castellet highest). Validated on NBR
  outing 29 (2026-10-05: it catches the strikes that drive the deep CS
  tail, 25 events including all of the deepest; 4 of 58 demonstrable
  misses); not separately validated on Le Castellet (12).

Corner brackets and phases (modules/corner_analysis.py; channels.json
corner_detection; B unless stated).
- Bracket: entered when |steering-wheel angle| > 25 deg OR |a_y| > 0.6 g,
  left when |steering| < 15 deg AND |a_y| < 0.35 g; same-direction
  brackets less than 0.6 s apart merge; a bracket lasts at least 0.8 s
  and its apex reaches |a_y| >= 0.3 g; brackets longer than 300 m are
  flagged compound. The dual criterion is argued (2026-07-22: the steering
  needed scales with 1 / R, so a fixed steering threshold is marginal in
  fast corners, and a_y is the vehicle response); the values themselves
  are not derived (12); a Paul Ricard check (2026-09-03) found no
  re-derivation needed. Cross-lap identity by lap distance: canonical
  corners take the median boundaries of the representative laps and are
  realised on every analysis lap.
- Phases, defined by driver actions: entry_1_brake from throttle lift-off
  (the last sample with throttle >= 95 % before turn-in, searched back no
  further than the previous corner's end) to the bracket start;
  entry_2_turnin from the bracket start to the apex (peak smoothed
  |a_y|); apex_3 the apex (the verdict reads the apex region +/- 25 m);
  exit_4 from the apex until the steering has unwound to half its
  post-apex peak; exit_5 to the bracket end. A corner taken without a
  full-throttle sample before turn-in keeps a zero-length entry_1_brake
  by construction (6-24 per development session). The lift-off
  construction was verified against brake pressure (2026-08-20, Dubai:
  pressure rises a median 0.13 s after the lift-off point, never before);
  the values 95 % and 0.5 are not derived (12). The throttle channel reads
  up to 106 % at full throttle; its range gate is [0, 110] % (until
  2026-10-21 a [0, 100] gate discarded it on 7 of 14 sessions and emptied
  every braking phase there).
- Declared adaptation of the chair basis: the chair splits a corner by
  the direction of the acceleration vector (rCosPhi = a_x / |a|: Braking,
  Entry, Mid, Exit). SetupTool keeps driver-input phases because the five
  phases must match the resolution and definition of the team's driver
  feedback form (e1..x5, a frozen team instrument that defines phases by
  driver actions), so measurement and feedback stay comparable.
  Principle: under- and oversteer have different causes per phase and
  need an individual solution per phase, and the driver reports symptoms
  per corner phase, all five phases for every corner [Putz.2022, secs.
  2.2.1-2.2.2 and 9.1, page to verify]; Puetz & Serne define the phases by
  roll state, the exact boundaries here follow the feedback form.
  Comparison on NBR 29 and NBR 10 (diagnostics/inspect_phase_segmentation.
  py (script at tag pre-prune-2026-10-22)): the schemes differ mainly around the apex (the chair's Mid starts
  17-129 m before our apex); the chair's Braking phase (a_x / |a| < -0.97)
  is empty on 64 of 90 and 101 of 180 corners, since trail braking turns
  the acceleration vector early.

Development set vs thesis package. All method choices, thresholds and the
robustness evidence in this file come from the DEVELOPMENT SET of 14
sessions: Nurburgring outings 5, 10, 20, 29, 46, 53, 54, 59, 61, 63,
qualifying 69 and 70; Le Castellet outings 25 and 32. The thesis results
are produced on a separate, clean data package (pending), on which no
choice is tuned. The Dubai sample is outside the thesis data.

Accuracy levels (every physical quantity states one): L1 config default,
L2 session measurement, L3 logged sensor, L4 lookup table (manufacturer
data). A derived quantity inherits the weakest input. Vehicle description
(mass, inertia, geometry, motion ratios, steering map) is
parameterisation, not a tunable: it lives in the vehicle config and the
local car_data.json and enters through these levels.

Grounding tiers: A vehicle-dynamics method, needs a literature anchor; B
signal / data engineering (filters, masks, segmentation, thresholds),
standard technique, labelled preprocessing, parameters config-driven and
data-derived; C product / UI. Deviations from the chair basis carry one
label each: FORCED ADAPTATION (the GT3R data leave no alternative), DOMAIN
IMPROVEMENT (the chair version is right for its context, ours differs),
NEUTRAL ENGINEERING.


## 2. Vertical loads

Why: fit L normalises every lateral force by the axle load, so the axle
load is a direct input of the tyre fit, the observer and the cornering
stiffness. It is measured, not modelled: each wheel carries a strain gauge
on the damper pushrod.

Per wheel (modules/wheel_loads.py, after [Segers.2014, ch. 9 / 10, page to
verify]):

    Fz_wheel = (F_damper - F_offset) MR(travel)          sprung, elastic
             + 0.5 (s_left - s_right) k_ARB / MR_mean     anti-roll bar
             + m_u a_y r_dyn / track                      unsprung transfer
             + m_s,axle a_y z_rc / track                  roll-centre transfer

- F_damper: logged pushrod force (L3); MR(travel): manufacturer
  motion-ratio table, clipped at its ends (L4); ARB rate table (L4), blade
  position from the setup, else CONFIG wheel_loads.arb_position_fallback =
  4 (L1); m_u = 40 / 45 kg, r_dyn = 0.3406 / 0.3518 m, z_rc = 0.014 / 0.044
  m front / rear (CONFIG wheel_loads; L1).
- Event zero (channels.json event_zero_entries.NBR_221): at the Nurburgring
  event the gauges were zeroed against a different reference, so the raw
  force sits about 58-59 kN off. Per corner zero_N FL 58639.9, FR 59466.3,
  RL 58419.7, RR 58906.6 N, derived as the median over
  the event's 16 files of the gauge-versus-input-voltage offset minus the
  same car's Le Castellet offset; wheel-level uncertainty u = 177 / 332 /
  389 / 140 N. Applied only on a match of track name and logger serial and
  when the corrected mean passes the channel range gate. Level: L3 sensor
  with an L2 zero.
- Decoding correction (channels.json channel_corrections): one Le
  Castellet file carries a corrupt additive offset on log_dms_dam_fr,
  removed (+15954360.4 N) only if the raw mean lies in [-16.05e6, -15.85e6]
  N. Cross-checked against the axle mate and a_y (corr +0.893).
- Cascade per corner and sample: damper (above) -> reconstructed from the
  axle total and the measured mate (L1) -> static split (L1); the source is
  recorded per sample. On every development session the fit population is
  100 % damper-sourced.
- Axle load = sum of the two wheels, then the 2 Hz zero-phase low-pass
  (same filter as Fy and alpha, so force and load stay paired).

Tier A (load decomposition, Segers), B (zero entry, guards, cascade). No
chair counterpart. Open items: section 12.


## 3. Axle lateral forces

Why: no axle force is measured; the planar force and moment balance of the
single-track model gives both from two measured quantities
([Milliken.1995, sec. 5.7, page to verify]; [Rajamani.2012, Eqs. 2.21 /
2.22, page to verify]):

    Fy_f = m a_y (b / L) + I_z (dr/dt) / L,      Fy_r = m a_y - Fy_f

with the static front weight fraction b / L from the corner weights and the
raw numerical gradient dr/dt (filtered once, downstream). Set to 0 when not
moving, then 2 Hz low-pass. Fy_f and Fy_r share two measured inputs (a_y,
dr/dt), so any two-axle fit inherits that coupling. Level: L1
(lateral_force_split, capped by I_z and the corner weights). Tier A;
chair-identical construction, FORCED ADAPTATION (no measured axle forces).

Longitudinal axle force (input of the fit and the observer, not of the
verdict): the CoG force Fx_cog = m a_x + 0.5 rho c_D A v^2 (tyre force =
inertial force plus aerodynamic drag). c_D A = 1.0 m^2, Level 1 from the
literature (no team value): c_D = 0.50, production-based GT race cars
[Katz.1995, Table 2.3, p. 50], A = 2.0 m^2, a generic race car with
touring-car dimensions [Trzesniowski.2019b, Abb. 5.89, p. 257], cross-
checked against a 911 Carrera road car, 1.78 m^2 [Katz.1995, Appendix 1,
p. 264] (CONFIG longitudinal_stiffness.drag_coeff, vehicle.aero.
cross_track_area_m2; L1, A). Rolling resistance 0 (documented
approximation). The frontal area is shared with the lift model, which
stays at zero because lift_coeff = 0. It is split to the front in proportion to the measured axle loads
under braking and is zero on the front under drive (rear-wheel drive)
[Goblirsch.2025, Eq. 9]:

    Fx_f = Fx_cog Fz_f / (Fz_f + Fz_r)  if Fx_cog < 0, else 0;   Fx_r = Fx_cog - Fx_f

A brake-pressure split is not used: pressure ratios become force ratios
only with known per-axle brake gains. Tier A.


## 4. Kinematic seed beta

The kinematic identity a_y = v (d beta/dt + r) is integrated
([Rajamani.2012, ch. 2, page to verify]):

    beta_raw = sum (a_y / v - r) dt          (moving samples)
    beta_kin = high-pass(beta_raw)

The 0.05 Hz zero-phase washout (stability_estimation.
beta_washout_cutoff_hz; L1, B) removes the integration drift of the offset
IMU signals. The cost of the washout is a limitation of the seed (12).

Role: the kinematic slip angles alpha_f, alpha_r computed from beta_kin
(section 7 definition) are the abscissa of the tyre fit (5), and beta_kin
is the session-wide fallback when the fit or the gate rejects the observer
(6). It never enters the observer state. Level L1. FORCED ADAPTATION (the
chair reads a sideslip channel; the GT3R has none). Tier A (identity), B
(washout).


## 5. Tyre fit L and the D4-L gate

Why fit L. With an estimated sideslip and almost no sliding data, the
shape of the axle curve is not identified by the data; a four-parameter fit
on such data moves C and E freely and invents a peak (thesis_notes
Q-FIT-SIMPLE, Q-FIT-LIT). Fit L therefore fixes everything the data cannot
identify at literature values and fits only the origin slope.

Model, per axle, on the load-normalised force f = Fy / Fz against alpha
([Pacejka.2006, Eq. 4.49, page to verify]; axle-level effective curve
[Pacejka.2006, p. 14]):

    f(alpha) = mu sin(C arctan(B alpha - E (B alpha - arctan(B alpha))))

- C = 1.0 (CODE L_SHAPE_C): with E = -1 the curve rises monotonically to
  mu, so no post-peak drop is modelled where the data never show one; C =
  1 is the lower bound of [Goblirsch.2025, Table I]; D is the peak only for
  C >= 1 [Pacejka.2006, p. 174]; the axle peak may vanish [Pacejka.2006,
  p. 14]; practice of fixing C at 1 where no post-peak drop is seen
  [Rao.2006, pp. 41-42], [Voser.2010, p. 322]. Tier A, method-defining.
- E = -1 (CODE L_CURVATURE_E), fixed "according to the literature"
  [Goblirsch.2025, Table I]. Tier A.
- mu = demonstrated utilisation: the 99.5th percentile of |Fy / Fz| on the
  axle's fit population (CONFIG tyre_fit_auto.mu_demo_percentile = 99.5;
  L2 session measurement; B). Not fitted: a fitted asymptote below forces
  the car demonstrably produced contradicts the measurement. It is a LOWER
  bound of the true friction (12); peak bounded by the maximum acquired
  forces after [Farroni.2018, p. 5]; conservative
  grip estimate [Acosta.2018, p. 1904];
  chair basis: mue_effective.
- B is the one fitted parameter: weighted least squares of f, bounded
  Powell on [5, 40] from the chair's start 12.0 (CODE L_B_BOUNDS
  [Goblirsch.2025, Table I], L_START_B). It sets the normalised origin
  slope K = B C mu [1/rad].
- Weights: inverse density on the signed slip angle, w = 1 / (samples in
  the sample's 0.01 rad bin) (CONFIG fit_density_bin_rad = 0.01; L1, B),
  so the dense near-zero region does not dominate the even data
  distribution that [Goblirsch.2025, page to verify] assumes.
- Front force frame: the front curve is fitted in the tyre frame, Fy_f,tyre
  = (Fy_f - Fx_f sin delta_f) / cos delta_f [Goblirsch.2025, Eq. 13]; the
  observer maps back with the inverse. Fz, Fx and delta_f pass the same 2
  Hz filter as Fy.
- Fit population: moving, outside the kerb mask and its smear zone, on
  analysis laps (the base mask), inside the session's speed window [p1,
  p99] of in-corner speeds (CONFIG fit_speed_window_pct = [1, 99]; L1, B),
  and without heavy braking: a sample leaves the axle's population when Fx
  < 0 and |Fx| / Fz > 0.3 (CONFIG fit_long_util_max = 0.3; L1, B). Sized
  by the friction circle [Pacejka.2006, p. 5]: at 0.3 the lateral capacity
  loss 1 - sqrt(1 - (0.3 / mu)^2) is 2-2.5 % for mu 1.4-1.6. Traction is
  kept (the driven rear operates under drive through every exit; removing
  it took over half the rear population); its cost is a limitation (12).
  Pure-slip fitting practice: [Farroni.2016,
  footnote 9, p. 824].
- Residual (for Q, section 6): Fy - mu Fz f(alpha) in N, per sample.

D4-L plausibility gate, per axle on the final curve (tyre_fit_auto.
plausibility_gate_l):
- g1 excitation: the p1..p99 span of the fit's slip angles >= 3.0 deg
  (CONFIG min_alpha_span_deg; L1, B).
- g2 normalised stiffness K = B C mu against the source, read at its own
  precision [Pacejka.2006, p. 6]: "ranges from about 6 to about 30 times
  the vertical wheel load ... The lower value holds for the older bias-ply
  tyre construction and the larger value for modern racing tyres." The
  axle ratio is the sum of tyre stiffnesses over the sum of tyre loads, so
  the tyre band carries over to the axle. Zones (CODE L_K_BAND, L_K_EDGE):
  K in [6, 30] ok; K in [5.5, 6) or (30, 35] marginal ("g2 at the edge of
  the literature range", the observer still ships); otherwise the fit is
  degenerate. Tier A.
- mu ceiling: demonstrated mu above 2.0 points at the force or load inputs,
  not at the tyre -> degenerate (CODE L_MU_CEILING [Pacejka.2006, p. 6];
  [Goblirsch.2025, Table I]).
- Reported, not gated: the curve has no peak ("none (C = 1, monotonic)");
  mu is stated as "demonstrated utilisation p99.5 (lower bound)".
Development set: front K 21.98-30.34, rear 15.46-29.17 per rad; outings 59
and 70 front 30.34 = edge, marginal (12).

Chair: the axle-level Magic Formula fit on vehicle data and the B start are
the chair's; fixing C, E and mu is a FORCED ADAPTATION (the chair fits all
four on a measured sideslip); the speed window is a FORCED ADAPTATION (one
signed curve per axle for the observer).


## 6. EKF observer

Why: the kinematic seed drifts and is washed out; a model-based observer
using the session's own tyre curves estimates beta through the corner.

Model ([Rajamani.2012, secs. 2.3 / 2.6, page to verify]; [Ulsoy.2012,
sec. 14.3, p. 263]; extended Kalman filter [Rajamani.2012, ch. 14, page to
verify]; modules/sideslip_ekf_pacejka.py):
- State x = [beta, r]; input delta_f; measurements z = [r, a_y]; V_x =
  max(v, 5 m/s) scheduled per sample.
- Process: d beta/dt = (Fy_f + Fy_r) / (m V_x) - r,
  dr/dt = (a Fy_f - b Fy_r) / I_z; explicit Euler at the grid step.
- Forces from fit L with the measured loads: rear Fy_r = mu_r Fz_r(t)
  f_r(alpha_r); front body force Fy_f = mu_f Fz_f(t) f_f(alpha_f) cos
  delta_f + Fx_f sin delta_f [Goblirsch.2025, Eq. 13]; small-angle slip
  angles alpha_f = delta_f - beta - a r / V_x, alpha_r = -beta + b r / V_x;
  analytic slopes for the Jacobians.
- Measurement model h = [r, (Fy_f + Fy_r) / m].

Process noise Q-UPPER (per session, L2; DECISIONS "Observer finalisation"):
Q cannot be identified without a reference sideslip (12), so its upper
bound is used. The fit-L force residual of both axles on their shared
population, propagated through the single-track equations sample by
sample (front/rear correlation kept), gives the error of the state
derivatives:

    e_beta = (dFy_f + dFy_r) / (m V_x),     e_r = (a dFy_f - b dFy_r) / I_z
    Q = diag(SD(e_beta)^2, SD(e_r)^2) dt_ref^2,   dt_ref = 1 / 50 s

held over one step (rescaled by 50 Hz / grid rate). An upper bound because
the residual is taken at the kinematic slip angle and carries its error
too. Diagonal form after [Villano.2021, Eqs. 28-29, p. 2661]; physically
meaningful variances after [Bechtloff.2018, Eq. 5.16, p. 81]. With Q_r at
this bound the yaw state follows the gyro (yaw rate as input of a sideslip
observer: [Villano.2021, sec. 3.1, p. 2659]); the cost is the weakened
yaw-moment constraint (12). Tier B.

Measurement noise (L2, B), from the session's own noise floors: the
variance of each measured signal minus its 2 Hz zero-phase low-pass, on
steering-defined straights (modules/straights.py: speed >= p25 of the
session, steering angle and steering rate within 2 robust sigma, outside
the kerb mask; CONFIG tyre_fit_auto.l_straight_*). Above 2 Hz the
single-track model has no dynamics, so that content is noise for it.
- R_yaw = the yaw-rate floor itself (6.4e-6 to 2.8e-5 (rad/s)^2 on the
  development set).
- R_ay = selected by the existing sweep, never below the ay floor (0.38-
  0.88 (m/s^2)^2): the half-decade grid around the residual-derived R_ay
  is continued downward until one value lies below the floor, and the
  floor is added; candidates are points with ay NIS exceedance in [0.03,
  0.15], nearest the centre, whose own sign check passes 0.7 (CONFIG
  r_sweep_*, sign_check_marginal_fraction; L1, B). No admissible point ->
  the default scale, never below the floor, status marginal.

Divergence and reset: only a hard bound |beta| > 15 deg triggers a reset
(CONFIG beta_hard_bound_deg = 15.0, l_divergence_trigger =
"hard_bound_only"; L1, B). The reset sets beta = 0, r = the measurement and
P = P0 (CONFIG p0_beta_var = p0_yaw_rate_var = 3.046e-4), never another estimate (reset to
zero after [Villano.2021, sec. 3.1, p. 2659]). With a realistic noise
model no covariance-only reset is needed; the reset samples form the
observer-reset mask used by the CS (7). Not moving: state reset, beta = 0.

Status of the session fit (CONFIG tyre_fit_auto; L1, B): degenerate if the
D4-L gate fails, the sign-check fraction (median beta opposite in sign to
median a_y on racing corners) is < 0.5, or the ay NIS exceedance > 0.5;
marginal if the K zone is "edge", the R sweep fell back, the sign check <
0.7 or the yaw exceedance > 0.75; else ok.

Session gate (modules/nis_gate.py; B): health score = share of base-mask
samples whose trailing 0.4 s window has a combined-NIS exceedance share in
[0.03, 0.15]; pass >= 0.104, warn >= 0.011, else fail (CONFIG nis_gate;
L1). Recorded rule, applied on fit L over the 14 development sessions
(2026-10-20): pass = a round number about 6 % below the lowest real score
(Le Castellet 25, 0.1107), warn = an order of magnitude below it, reserved
for a collapsed fit. The gate catches divergence, not fit quality.

Fallback (tyre_fit_auto.resolve_sideslip_beta): fit degenerate or gate
fail -> kinematic beta for the whole session, with the reason in the
payload and the status line; otherwise the observer beta. Level: sideslip
L1 (a per-session model estimate). Tier A (model, EKF), B (noise, gate,
status). SetupTool's own observer, FORCED (no sideslip measurement).


## 7. Cornering stiffness

Why: the cornering-stiffness ratio CS measures how far the local slope of
the axle's force-slip relation has fallen from its linear value -- 1 in
the linear range, towards 0 near the limit -- without assuming a tyre
model (framework: [Werner.2021, sec. 2.2.3, p. 16]). Fit L normalises the
force by the axle load, so the slope is a normalised stiffness [1/rad] and
load transfer does not move it ([Milliken.1995, pp. 26-27]).

Per axle, per sample i (moving, outside the kerb mask;
stability_analysis.estimate_cornering_stiffness, CONFIG
stability_estimation):
1. Slip angles from the shipped beta (single-track definitions):
   alpha_f = delta_f - arctan((v sin beta + a r) / (v cos beta)),
   alpha_r = -arctan((v sin beta - b r) / (v cos beta)); 2 Hz low-pass.
2. Window [start, i): at least 0.1 s (min 5 samples), widened backward
   until the alpha span reaches 0.02 rad, capped at 53 m of track; span not
   reached -> no value; a window straddling two laps -> no value
   (cs_min_window_s = 0.1, cs_min_window_samples_floor = 5,
   cs_min_slip_angle_span_rad = 0.02, cs_max_window_m = 53.0; L1, B).
3. Regressand y = Fy / Fz (axle load, section 2). C_window = OLS slope of y
   on alpha with R^2; C_section = span-weighted mean of the slopes of the
   window's monotonic alpha sections (weight = smooth step of the span,
   CODE SPAN_WEIGHT_EXPONENT = 4); C_alpha = R^2 C_window + (1 - R^2)
   C_section (CODE R2_WEIGHT_EXPONENT = 1).
4. Linear reference C_lin: a window sets it to its C_alpha only if every
   |alpha| < 0.03 rad (cs_linear_slip_threshold_rad, re-derived on this
   car's Fy-alpha cloud; L1, B), the window is clean (no kerb, smear-zone
   or off-analysis-lap sample) and C_alpha lies in [5.5, 35] per rad --
   the same source-precision reading of [Pacejka.2006, p. 6] as g2 (CODE
   L_K_EDGE), and within [0.5, 2] x the session median of that axle's
   band candidates (CODE tyre_fit_auto.L_REF_RELATIVE; B). The median is
   taken over all band candidates of the session in a first pass, so it
   does not depend on sample order; the reference is set in a second pass.
   Basis: the slope at zero slip "is not or hardly affected by the level
   of speed and by the condition wet or dry", and its load sensitivity is
   far below a factor 2 [Pacejka.2006, p. 6]; a candidate outside the
   window is an ill-determined low-excitation slope (registry R26).
   Otherwise the previous reference holds; no CS value is deleted.
   Development set: reference medians front 17.8-24.4, rear 17.9-30.6 per
   rad; the band excludes 0-0.6 % (front) and 0-5.9 % (rear) of the
   candidates; the relative gate rejects 69 front candidates (all low, on
   NBR 10, 46, 54, 63, 69) and no rear one.
5. CS = min(C_alpha / C_lin, 1). No reference yet -> NaN (counted).
Reading: 1 = linear; towards 0 = the axle approaching its force limit;
negative = axle force falling while slip grows (a measured local slope; the
fitted curve, monotonic since C = 1, has no peak).

Marks (arrays keep every sample; marked samples leave the verdict
statistics): kerb-affected (the window holds a kerb or smear-zone sample)
and observer-reset-affected (the window holds a hard-bound reset sample,
section 6 -- the slip angles there jump with the state).

Segmentation: corner brackets and the five phases as in section 1. Per corner, analysis lap and phase the
CS median over the clean population (at least 5 samples); apex_3 reads the
apex region +/- 25 m.

Chair: as-is in the 2 Hz input filter, 0.02 rad span, 0.1 s minimum
window, weighting exponents 1 and 4 and the ratio construction. Adapted:
the 0.03 rad linear limit (chair 0.021) re-derived on this car; the 53 m
locality cap (NEUTRAL ENGINEERING); kerb marks, clean-window reference and
the reference band (DOMAIN IMPROVEMENT); Fy / Fz instead of Fy (DOMAIN
IMPROVEMENT: load transfer otherwise moves the slope). Tier A (framework),
B (windowing, blend, hold, reference rules).


## 8. Floor events

Why: a CS ratio can fall below anything a pure-slip tyre curve produces.
That is information about the axle (combined slip, transient load), not
noise to delete; it is labelled, counted and described.

Floor: the steepest post-peak slope of the pure-lateral Magic Formula
relative to its initial slope B C D, minimised over E in [-1, 1], at C =
3.0 (the upper bound of [Goblirsch.2025, Table I]; [Pacejka.2006, p.
175]); independent of B and D. Result -0.53 (CODE limit_events.
PURE_LATERAL_FLOOR; A). A ratio below it is "not explainable by pure
lateral saturation".

Event = one contiguous run of CS < -0.53 inside a corner instance on an
analysis lap, per axle, on the full CS arrays, without bridging gaps
(limit_events.detect_floor_events). Neighbouring CS samples share most of
their regression window, so a slope held over a flat-alpha stretch is one
observation: plausibility and floor statistics count events, not samples.
Attributes (measured, no classification): lap, corner, phase at the
minimum and phases spanned, start / end / minimum position and time,
duration, sample count, minimum ratio, distance to the nearest kerb sample,
kerb-affected and reset-affected flags, maximum |Fx| / Fz, braking share
and flag (Fx < 0 on every sample), Fz range over mean inside the event
samples' regression windows (median, max), recurrence across laps
(+/- 10 m). Under fit L these events are the app's event layer (display and
record only, never a verdict input).

Worked example (NBR outing 54, corner 3, turn-in, front, laps 215-218):
one or two events per lap at 725-770 m, braking throughout, max |Fx| / Fz
0.24-0.35, front Fz range 24-36 % inside the windows, no kerb within 36 m.
Reading: the front at its limit in trail braking -- combined slip (about 2
% capacity loss by the friction circle [Pacejka.2006, p. 5]) plus a fast
load increase, which the force follows with a lag (relaxation length
[Pacejka.2006, p. 22]; cornering on changing wheel load [Pacejka.2006,
pp. 366-368, Fig. 8.3]); real behaviour, not an artefact.


## 9. Classification

Per-phase statistics (stability_analysis.summarise_corners; CONFIG
stability_estimation; L1, B): per corner, analysis lap and phase the median
of the clean CS population and of the stability value; fewer than
cs_phase_min_valid_samples = 5 finite samples -> NaN. After that gate the
entry_1_brake stability value is set to NaN when the maximum front brake
pressure (log_pbrake_f) in the phase stays below
stab_phase_no_braking_floor_bar = 3.0 bar -- no real braking, so the
"braking" phase holds no braking data (config derivation, sized on one Le
Castellet file: at throttle > 90 % the pressure is 0 bar at p99 with one
6.03 bar transient, real braking phases reach >= 13.6 bar; 3.0 bar lies
between).

Cross-lap aggregation (modules/corner_aggregation.py aggregate_by_corner;
the same function feeds the map, the classifier and the setup engine):
per stable corner and phase, CS = the worst lap, i.e. the minimum over
laps of the phase medians (CONFIG classification.cs_cross_lap_aggregation
= "worst_lap"; the apex region likewise); stability = the median over laps
of the phase medians (rationale recorded 2026-10-21, after the fact:
the dMz/dbeta regression is pooled over laps by track distance, so the
per-lap values are samples of one pooled curve that differ only through
sample positions and masks; the median is their robust central value,
while a minimum -- right for CS, which is estimated per lap and locally
-- would select mask or position artefacts, not a different lap's
behaviour). The classifier then takes the minimum over phases,
so the CS corner value is the minimum over (lap, phase) -- the population
the thresholds were derived on. The verdict cards classify each corner-lap
(worst phase within that lap); the map and the setup engine classify the
aggregate (modules/classifier.py classify_corner).

Speed class (low < 80 <= medium < 150 km/h <= high, channels.json
corner_speed_thresholds, Settings-editable; B): a canonical corner takes
the class of the median apex speed over its instances
(modules/corner_analysis.py); the aggregate takes the modal class across
laps, ties towards medium, then the lower class (corner_aggregation.
_aggregate_speed_class) -- this only resolves per-lap corners whose class
differs between laps.

Thresholds (CONFIG classification; L1, B; derived 2026-10-20 on fit L, 14
development sessions, all on observer beta; re-derived after the R26
reference gate and again on schema 18 (populated braking phases,
2026-10-21) with identical p10 / p30; re-derived on schema 19 (drag,
2026-10-21) and adopted):
- STRONG = p10, MODERATE = p30 of the pool's corner values per axle:
  front STRONG_CSF = -0.137, MODERATE_CSF = -0.023 (n 220);
  rear STRONG_CSR = -0.002, MODERATE_CSR = +0.098 (n 208).
- Tier semantics: relative ranks in the pool, no physical claim. A positive
  rear MODERATE is accepted: about 0.1 is a rear stiffness at 10 % of linear,
  near saturation, and rear saturation erodes the stability margin
  [Hoffman.2008, sec. 2, p. 136].
- Physical statements are separate labels: ratio < 0 = "axle force falling
  while slip grows"; ratio < -0.53 = floor event (8).
- Axle limiting (ruling D2, 2026-10-22): an axle is limiting when its CS
  ratio is below zero, i.e. past the peak of its Fy / Fz curve; the test is
  CS < -margin per axle (CONFIG classification.cs_boundary_margin_front =
  0.062, cs_boundary_margin_rear = 0.076; L1, B; author ruling 2026-10-22):
  the p90 of the combined CS change under the two admissible observer
  perturbations (Q x 0.3, R_yaw x 10) and the +/-1 % vehicle-input sweep
  (largest run) over the 14 development sessions
  (diagnostics/inspect_cs_margin_combined.py). The observer term dominates;
  the vehicle part comes from all laps and the raw apex_3 slice, the
  observer part from valid laps and the apex region (small population
  mismatch, stated). The percentile tiers grade depth only.
- Symptoms per phase (modules/classifier.py phase_symptoms): front
  limiting -> understeer (US), moderate, strong when the value also lies
  below STRONG_CSF; rear limiting -> oversteer (OS), always strong -- a
  rear axle past its peak removes the restoring yaw moment, a loss of
  stability [Hoffman.2008, sec. 2, p. 136]; rear between the boundary and
  MODERATE_CSR -> information only, no symptom; open-loop unstable yaw
  (below) -> INST in whichever phase it occurs, strong. STRONG_CSR and
  MODERATE_CSF set no verdict any more (recorded tiers, Settings-editable).
- Verdict word: rear limiting first (oversteer), then front (understeer),
  each at its worst phase; open-loop unstable added at its worst phase.
- Low priority: a high-speed corner whose only symptom is front limiting
  is flagged low priority -- mild high-speed understeer is the deliberate
  safe balance [Milliken.1995, p. 396 item 21, p. 401 item 1F].

Yaw stability (second signal; modules/yaw_stability.py, chair estimator
and its six values as-is; CONFIG stability_estimation.yaw_stability_*; L1,
A): the yaw-moment derivative dMz/dbeta.
- Target: the inertial yaw moment I_z psi_dd, psi_dd = numerical gradient
  of the yaw rate followed by a centred rolling mean over 0.15 s (15
  samples at 100 Hz); the damping term of the full yaw balance is not
  included (inertial regressand only).
- Regressors: beta, delta_f, v, a_x, a_z (a_z dropped if unusable); the yaw
  rate is not a regressor. Predictors centred and scaled by their weighted
  standard deviation.
- Local weighted ridge regression over lap distance s, all analysis laps
  pooled: grid every 2 m, samples within +/- 55 m, Gaussian weights with
  sigma = window / 2.5 = 22 m (CODE GAUSS_SIGMA_DIVISOR), ridge 1.0. A grid
  point is skipped with fewer than 90 samples or a weighted beta standard
  deviation below 0.001 rad. dMz/dbeta = the beta coefficient over the beta
  scale, in Nm/deg, interpolated onto every sample's lap distance.
- Exclusions (regression population): not moving, kerb mask, non-analysis
  laps (DOMAIN IMPROVEMENT; the raw-yaw-rate path is a FORCED ADAPTATION).
- Sign: positive = restoring. "Open-loop unstable" when the worst phase
  value is < 0 Nm/deg -- the sign criterion, anchored for the thesis on
  [Guiggiani.2023, pp. 306, 308] (the code docstring's Werner reference,
  sec. 2.2.3, is provenance only); explicitly not a criticality threshold.
Severity (D2): the highest of the corner's symptoms -- strong for rear
limiting, front limiting in the strong tier, or open-loop unstable;
moderate for front limiting alone; normal otherwise.

[MARGINAL]: the verdict-driving value lies within cs_margin = 0.01 of a
boundary that decides its class -- the axle's own limiting boundary, and for the front
also the strong tier (CONFIG classification.verdict_stability_margin; L1, B).
Derived by the recorded +/-1 % sweep of mass, front weight fraction,
wheelbase, yaw inertia and steering map on the 14 sessions (schema 19):
one proportional verdict flip at threshold distance 0.000, rounded up;
two discontinuity flips excluded by the recorded rule (a ratio
discontinuity at 0.046, and a worst-lap apex-region median of a spread
small sample that jumps +0.063 -> +0.410 under a 1 % input). It
expresses sensitivity to the vehicle parameters ONLY; the observer
uncertainty (12) is larger and is not expressed by it. Share of tier
verdicts tagged [MARGINAL]: 0-50 % per session.

Display: the CS phase values are coloured by the class they would set
(classifier.cs_value_state): front limiting WARN, in the strong tier BAD;
rear limiting BAD; rear information band WARN; else OK. [UNCAL]
marks every verdict when the active estimator path differs from the one
the thresholds were derived on -- sideslip source and, for the auto fit,
tyre_fit_mode (classification.thresholds_calibrated_for_sideslip_source /
_for_tyre_fit_mode = ekf_auto_pacejka / literature_bounded).

Tier B (thresholds, aggregation), A (stability sign criterion; fit L does
not touch it). The thresholds differ from any chair value by rule.


## 10. Setup recommendation (setup engine)

modules/setup_engine.py with config/mechanisms.json (recommendation
overhaul D1-D7, live 2026-10-22; rulings of work orders 4e, 4f, 4g and 4h, schema 25). One small engine; the earlier
rule engine and decision frame are removed. It proposes, it never applies a
change. Tier A in the mechanism table (anchors below), B in the repetition
rule, C in the display.

Symptom -> mechanism -> lever:
- Symptoms come from the classifier per lap and phase (section 9): US, OS,
  INST.
- Repetition (D5; CONFIG setup_engine.min_repeat_fraction = 0.5,
  min_repeat_laps = 2; L1, B, author ruling 2026-10-22): a corner-phase
  symptom counts only when it appears on more than half of the corner's
  valid laps and on at least two. Its severity is read from the cross-lap
  aggregate, the population the tiers were derived on.
- Mechanism table (config/mechanisms.json, 46 rows; D3, lever pass-over 4f): each row ties a
  lever group, a direction, symptoms, phase groups (BRK = entry_1_brake,
  TIN = entry_2_turnin, APX = apex_3, EXIT = exit_4 + exit_5) and speed
  classes to its source. M1 roll stiffness distribution (ARBs, springs; all
  phases and speeds) [Milliken.1995, pp. 409-410]; [TE Bd. 5, Tab. 5.2
  p. 210, Tab. 5.11-5.13 pp. 293-295]. M2 transient load transfer
  (low-speed dampers; BRK, TIN, EXIT, never APX) [TE Bd. 5, Tab. 5.9,
  p. 254]; [Segers.2014, pp. 259-263]; under braking the damper rows serve
  OS and INST alike (braking instability, [TE Bd. 5, Tab. 5.9 p. 254,
  Tab. 5.13 p. 295]). M4 aero balance (wing, splitter, ride heights; high
  speed only) [Milliken.1995, p. 396, p. 572]; [TE Bd. 5, Tab. 5.11
  p. 293]. Ride heights (4f): nose down moves downforce forward -- US: front
  down / rear up, OS and INST: front up / rear down -- [Katz, pp. 170-172,
  Fig. 5.26]; [Milliken.1995, p. 396 item 20, p. 573]; valid outside the
  ground-effect stall regime, which the engine does not detect, so the line
  states it for the engineer (the "front up against sudden push" of [TE
  Bd. 5, Tab. 5.11] is the stall case, Katz Fig. 5.26; Katz edition to
  verify). M5 tyre working point (camber, advisory -- needs tyre
  temperatures [Segers.2014, pp. 194-196]; front toe at low-speed turn-in
  only, a response lever; rear toe; rear camber more negative also for OS
  under braking and for INST at the apex) [TE Bd. 5, p. 213, Tab. 5.12 p. 294,
  Tab. 5.14 p. 297]; [Milliken.1995, p. 394, p. 726]. M6 drivetrain (diff,
  TC long / lat; a traction-limited exit -- TC flag ecu_B_tc_act active on
  the majority of the symptom laps -- ranks TC long above the diff)
  [Milliken.1995, pp. 739-740, p. 391]; [Segers.2014, p. 89]; TC is
  stepped only inside positions 2-11, where the manufacturer table is
  monotonic (a decrease from 2 is at limit). M7 brake balance (bias; OS
  more front under braking and at turn-in -- too much rear bias makes the
  car loose coming into the turn [Milliken.1995, p. 393 item 6]; ABS map
  toward the rear-axle-stable map within the current grip bracket, for OS
  and INST under braking) [Milliken.1995, p. 393, p. 398]; [Segers.2014,
  p. 114]. The differential is not a lever for braking instability:
  coast-side locking of the fitted differential package is not documented
  (thesis_notes, 4f). INST takes a lever's OS direction unless a row names INST for
  that lever and phase (rear saturation = stability loss
  [Hoffman.2008, sec. 2, p. 136]). Left and right always move together
  [Segers.2014, p. 263]. Not levers: high-speed dampers, blow-off, front
  ARB mount. TE Bd. 5 = Trzesniowski /
  Eder 2019 (bib Trzesniowski.2019); the Milliken and Segers bib keys are
  still to verify; pages as given in the overhaul work order.
- Side effects as text (TE Bd. 5, Tab. 5.14 pp. 296-297): camber more
  negative -- higher straight-line rolling resistance, less longitudinal
  force, more tyre deformation; front toe more toe-out -- higher tyre
  temperature and wear; differential locking increase -- disturbing yaw
  moment in corners at low lateral acceleration. Rows in the opposite
  direction carry no text (none ruled).
- Direction and step: the direction word is converted to a signed step
  through the registry's direction_semantics (setup_parameters.json; damper
  clicks: increasing = softer); step = the registry adjustment_step
  (dampers: "fine" = 1 click, CONFIG setup_engine.damper_step; team
  practice for race-weekend finetuning, the larger steps of [TE Bd. 5,
  p. 292] are test-day advice; camber 0.2 deg = two 0.1 deg clicks, team
  practice; ride heights 1 mm), else value_space.step, else one unit / one
  enum option. Change effort per lever from the registry; dampers are
  "minutes" (< 5 min in the pitlane, Setup_change_overview).
- Current value and feasibility (modules/setup_state.py): from -> to from
  the setup sheet; a step past the registry value space is "at limit" and
  skipped; past the typical window (config/setup_windows.json) is noted
  "engineer may exceed"; window headroom breaks ties. The front ride height
  has a floor only (standard - 2 mm, diffuser floor limit) and no upper
  limit. The ABS map has no
  sheet field: its current position is the session mode of the logged
  channel log_abs_pos (L3), and a map change within the session adds "map
  changed in session (last valid lap: N)", N = the position on the last
  valid lap; the map is categorical, so the target stays the engineer's
  choice. TC and brake bias have neither a sheet field nor a trusted
  channel and are "not assessable" (the files carry a TC "int" / "tim"
  position pair, but which one is LAT and which LON is not documented, 4f).

Two outputs per corner (4g): the setup recommendation is chosen among the
mechanical levers only (ARB, springs, LS dampers, wing, splitter, ride
heights, camber, toe, diff); the best driver-adjustable lever (registry
escalation_tier "cockpit": brake bias, ABS map, TC lat / lon) is shown as a
separate "driver can also" line, from the same table and the same rank
key. Team practice: cockpit values are changed by the drivers during an
outing and are not part of the engineer's setup progression
(Setup_change_overview). The thesis comparison with the engineer's changes
reads the setup recommendation only (D8 amendment).

Ranking and output (D4, order ruled in 4e): severity -> repetition count
-> TC-condition demotion -> engineer preference -> change effort (seconds
< minutes < half hour < garage hours [Milliken.1995, p. 391]; shown on
every line) -> headroom; high-speed front-only corners sort after the
others of the same severity. Per corner one recommended change and up to
two alternatives -- one change at a time, so its effect stays attributable
[Milliken.1995, p. 391]; [TE Bd. 5, p. 292]. The same lever change across
phases is merged; an opposite direction of an already chosen lever is
listed as a conflict. The limiting-end class (4h) ranks right after the
repetition count and the TC demotion, ahead of preference: limiting axle
US -> front, OS / INST -> rear; a row on that axle before a global row
before a row on the other axle [Milliken.1995, p. 409] (improve the
limiting end first; physics before engineer preference). Each row carries its axle (mechanisms.json "axle"):
ARB, springs, LS dampers, camber, toe, ride heights by their axle; wing
rear; splitter front; diff, TC, brake bias, ABS map global. It replaces the
row order of mechanisms.json, which decided silently before. Engineer preference: the elicited cells of the
retired engineer matrix (moved into mechanisms.json) give preference 1 to a
candidate only when the cell names its lever with the table's direction
for that symptom, phase group and speed class; a weight only, never a new
lever or a direction. Four cells the table does not support (a
contradicting direction or a lever without a row there) are ignored and
listed in mechanisms.json: US-BRK-low, OS-APX-high, INST-BRK-med,
INST-BRK-high; in US-BRK-high only the wing entry is ignored (its ride-height
entries match M4 since 4f).

Session setup recommendation (4h): the setup is changed for the whole
lap, so the per-corner output is a diagnosis and the setup decision is ONE
mechanical change per session, with up to two alternatives
[Milliken.1995, p. 572 item 4, p. 391]; [TE Bd. 5, p. 254].
- Vote (CONFIG setup_engine.session_vote; L1, B, author ruling
  2026-10-23): every counted corner symptom votes for each mechanical lever
  group of its candidate rows, signed by the registry direction, with
  weight severity rank (moderate 1, strong 2) x k / n.
- Opposition: the weight of every vote for the same lever group in the
  opposite direction is subtracted; net support = support - opposition;
  net <= 0 is never recommended. Support, opposition and the supporting /
  opposing corners are shown.
- Rank: net support -> limiting-end class (the option's limiting axle is
  the one its supporting votes report; equal front and rear weight ranks
  as global) -> preference (any supporting candidate preferred) -> change
  effort -> headroom. An exact tie after all keys has no winner and is
  listed as equivalent options.
- At-limit lever groups with positive net support are listed, not
  recommended.
- Not voting (listed with the reason): a diff increase whose condition
  not_traction_limited fails (TC active on most symptom laps,
  [Milliken.1995, p. 740]) and low-priority corners (high speed, front
  limiting only, [Milliken.1995, p. 396 item 21, p. 401 item 1F]), which
  stay in the per-corner diagnosis.
- Engineer layer: the same vote plus the driver-only corners (|fb| >= 4,
  no data support) with weight 1 x 1, marked; each supporting corner shows
  agree / neutral / conflict. Feedback weights nothing.
- The thesis comparison with the engineer's change reads the session setup
  recommendation of the data layer (D8 amendment, mechanism level); the
  per-corner diagnosis is supporting evidence.
- Not weighted: corner importance (fastest corners, corners before long
  straights, [TE Bd. 5, p. 254]) -- a stated limitation.

Two layers (D6) and the session layer (D7):
- Data layer: computed without driver feedback and stored unchanged (DB
  payload and sidecar, key "setup_engine"); the thesis comparison reads it.
- Engineer layer: the data layer re-ranked with feedback -- per
  corner-phase, feedback sign (- understeer, + oversteer, |fb| >= 1) against
  the symptom gives agree / neutral / conflict, ranked right after
  severity; INST is read like OS -- rear saturation is a loss of
  stability [Hoffman.2008, sec. 2], which the driver feels as a loose
  rear, so oversteer feedback agrees and understeer feedback conflicts;
  |fb| >= 4 with no counted data symptom adds a driver-only row from the
  same table, labelled "no data support". Driver level is not used.
- Session layer: per symptom the share of corners with a counted symptom,
  the share of corners with any counted symptom, and the single-lap cell
  shares without the repetition rule (front limiting and rear limiting
  over the cells with a CS value on that axle, INST over the cells with a
  stability value; each with its n). Computed, stored and reported beside
  the session setup recommendation; the share threshold itself stays unset
  (setup_engine.session_share_threshold = null, author ruling 2026-10-22).

On the development set (2026-10-23, schema 22): single-lap axle limiting
beyond the margin is rare (pooled p50 / p90 of the per-session shares:
front 0.018 / 0.039, rear 0.000 / 0.006) and almost never repeats; INST
(0.012 / 0.047) repeats -- 11 of the 12 counted corners over the 14
sessions are INST. 12 corner recommendations in total; NBR JST 29 gives two
(C8, C11, both INST), Le Castellet 32 none. [Schema 24, 4g: still 12
corners; every INST setup line is front ARB stiffer, the cockpit line brake
bias more front (turn-in), tc_lat increase (apex) or tc_lon increase (exit);
NBR JST 29: C8 front ARB stiffer / brake bias more front, C11 front ARB
stiffer / tc_lat increase.] [Schema 25, 4h: the limiting-end key turns
every INST corner line to rear ARB softer. Session recommendation: rear ARB
softer in NBR 29 (net 8.00), 46, 53, 54, 63; equivalent options without a
winner in NBR 20, NBR qualifying CDR 69 and Le Castellet 25 (no setup
sheet, so no headroom key); none in the six sessions without a counted
symptom, Le Castellet 32 among them.]


## 11. Validation evidence (development set, 14 sessions)

- Observer health: no fallback; NIS gate pass on all 14 (health 0.111-
  0.243); status 12 ok, outings 59 and 70 marginal (K edge); hard-bound
  re-entries 0 on 12 sessions, 2 (Le Castellet 25) and 1 (Le Castellet 32)
  -- against 27-246 per derive session under the earlier hand-set Q. NIS
  median 0.49-0.75, below the chi-square (df 2) median 1.39: the filter is
  mildly conservative, consistent with Q being an upper bound. Sign check
  0.94-1.0; apex force consistency h2 0.987-0.996.
- Straight-line beta: on the steering-defined straights the median |beta|
  is 0.045-0.080 deg (acceptance < 1 deg; earlier observer settings 0.15-
  0.96 deg).
- Noise floors binding: R_ay is the ay floor on 13 sessions (Le Castellet
  25 interior at scale 0.01 against floor 0.0071); the sweep would go lower
  on NBR 5, 59, 61 and 70.
- Robustness (final chain, thresholds fixed; variants of the observer noise
  against the baseline, 14 sessions, 2026-10-21): Q x 0.3 -> corner values
  shift by a median of 0.017 / 0.020 (p90 0.085 / 0.106) front / rear, 15.6
  % of corner verdicts change class; Q x 0.1 -> median 0.049 / 0.054 (p90
  0.21), 25.5 %; R_yaw x 10 -> 0.0 %, no value moves. Q_beta at the bound
  with the old hand-set Q_r = 2.408e-7 fails the observer-health criterion
  (237-783 hard-bound re-entries per session, floor events 59 -> 2671,
  median value shift 0.40 / 0.43): not admissible, and it confirms Q-UPPER
  for r. Reading: CS values are robust to a few hundredths; the STRONG /
  MODERATE split (0.10-0.12 apart) is finer than the observer uncertainty.
- Plausibility of the normalised CS (criterion: deep negatives rare and
  repeatable): 54 front floor events (39 clean) and 3 rear (all under
  traction) in the whole pool (schema 19); no reset-affected event; durations median
  0.05-0.18 s. Repeatable cells only NBR 54 corner 3 turn-in (4 of 6 laps,
  the worked example of 8) and NBR 46 corner 2 turn-in (2 of 4); all other
  events single-lap. Against raw Fy the share of samples below -0.53 falls
  from 0.13-1.63 % to 0-1.14 % (front). No corner value below -0.53 (pool
  minimum front -0.354, rear -0.207).
- Reference plausibility (registry R26 test: verdict-read CS samples under a
  reference below 0.5 x the session median): 0 on every session and axle
  after the relative gate (7), from 1461 front samples on NBR 46, 54, 63
  and 69 before it; 15 front phase medians moved, the deepest corner value
  only on NBR 69 (-0.471 -> -0.361). The thresholds re-derived after the
  gate are identical at full precision (9).


## 12. Limitations

- No peak coverage. The data do not reach the axle peak (before C was
  fixed, the fitted peaks of 6.4-15.8 deg lay beyond the visited p99 of
  4.0-7.1 deg on every axle); a peak would be an extrapolation. Fit L
  models none (C = 1), so nothing past the visited range is claimed.
- mu is a lower bound. The demonstrated utilisation p99.5 is what the car
  produced, not what the tyre can (front 1.48-1.86, rear 1.45-1.62 on the
  three derive sessions NBR JDR, Le Castellet 32 and NBR 29). Where the
  driver did not reach the limit, mu and K = B C mu read low.
- Traction combined slip stays in the rear fit. Under an isotropic friction
  circle [Pacejka.2006, p. 5] the available lateral fraction of the rear
  population is 0.97-0.98 at the median and 0.87-0.88 at p95 of the
  longitudinal utilisation: about 2-3 % lateral capacity is lost at the
  median, 12-13 % at p95. An estimate, not a measurement; the front under
  braking is filtered at |Fx| / Fz 0.3 (2-2.5 % loss).
- Washout cost of the seed. The 0.05 Hz washout bleeds a held sideslip
  (time constant about 3.2 s): the per-corner residual of the shipped fit
  at the seed slip angle grows with corner duration -- Spearman rho +0.51
  to +0.80 on every Nurburgring session and axle, +0.29 to +0.41 on Le
  Castellet (14 development sessions, 51-174 corner instances per axle;
  p < 0.01 except three Le Castellet 25 values; diagnostics/
  inspect_washout_final.py (script at tag pre-prune-2026-10-22), 2026-10-21). The weaker effect on Le Castellet
  is recorded as observed; its causes were not examined. The fit's
  abscissa carries this error; the observer does not (its state never takes
  the seed).
- Q is not identifiable without a reference sideslip. The force residuals
  of the two axles are correlated (0.79-0.92) with correlation times of 1.2-
  2.1 s, so a white, diagonal Q cannot be derived from them; the held-over-
  step upper bound is used instead (6). A reference measurement (optical
  sideslip sensor) would be needed [Farroni.2016,
  p. 811 -- page to verify].
- Correlated process and measurement noise. The force-model error enters
  both the beta equation and the a_y measurement, and R_ay starts from the
  same residual; the filter assumes them independent and white. The
  cross-covariance is not estimable from these data (outlook: [Bechtloff.
  2018, Eqs. 5.15-5.16, page to verify]).
- Yaw-moment equation. With Q_r at its upper bound the yaw state follows
  the gyro, and the yaw-moment equation contributes little to the
  observability of beta. The front / rear split of the axle forces
  (measured a_y and yaw acceleration, 3) and of the slip-angle differences
  (measured steering, yaw rate and speed) does not depend on it. Its effect
  on CS cannot be isolated: no admissible configuration with an active
  yaw-moment constraint exists -- Q_r at the old hand-set value (about 6000
  x smaller) fails the observer-health criterion (11).
- Observer uncertainty vs tier spacing. Within the admissible Q range the
  corner values move by a median of 0.02-0.05 (p90 0.09-0.21), the tiers
  lie 0.10-0.12 apart, so 15-26 % of verdicts change class under Q
  variations; [MARGINAL] (0.01) does not express this.
- Mid-corner coverage. A CS needs excitation: the window must sweep 0.02
  rad of slip. Steady mid-corner slip and kerb-masked stretches give no
  value. The same principle holds for slip-slope friction estimation: with
  very small slip the data lie around the origin of the force-slip curve,
  where the estimate is stochastically uncertain, so no update is made
  without sufficient excitation [Rajamani.2012, sec. 14.4.3, p. 412];
  persistent excitation also [Sierra.2006, sec. 3, page to verify]. "no CS data"
  occurs on 9 front and 21 rear corner values of the pool.
- Le Castellet load uncertainty. The straight-line load ratio reads 0.950-
  0.955 on both Le Castellet outings against about 1.0 at the Nurburgring
  -- a session mass or aero offset of about 5 %, which enters mu directly;
  one front-right damper channel there needed a decoding correction (2).
- Fresh-tyre K edge. Outings 59 and 70 read front K 30.34 per rad (+1.1 %
  above the band, next highest 28.42), consistent with fresh racing tyres
  (a tyre run and qualifying); marginal, observer shipped; decided in the
  clean package.
- Bump rubbers and ARB blade. Bump rubbers are not modelled (the load
  under-reads at extreme compression; their position relative to the gauge
  is to be verified at the car); the ARB blade position should come from
  the setup sheet (L2) instead of the fallback 4. No magnitudes recorded.
- The relative reference gate trusts the session median. It rejects slopes
  far from the session's typical linear stiffness; a session whose
  low-excitation windows dominated the band candidates would carry a low
  median itself. On the development set the median sits at 17.8-24.4
  (front) and 17.9-30.6 (rear) per rad, inside the literature range of 6-30
  and close to the fitted K (5).
- Kerb threshold not validated on Le Castellet. The 1.2 g mask was
  validated on NBR outing 29 only; on Le Castellet, where it flags the
  largest share (8.1 / 7.1 % after dilation), no separate check of what it
  catches and misses exists.
- Corner phase values not derived. steering_unwind_fraction = 0.5 (the
  end of exit_4 at half the post-apex steering unwind) is Tier B, the
  midpoint of the unwinding, not derived; brake_throttle_max_pct = 95 is
  Tier B, its lift-off construction verified against brake pressure
  (2026-08-20), the value not derived. The bracket thresholds (25 / 15
  deg, 0.6 / 0.35 g, 0.6 s merge, 0.8 s, 0.3 g apex) are Tier B, not
  derived (1).
- Drag from literature. c_D A = 1.0 m^2 is a Level-1 value from sources of
  about 1990; a modern GT3 carries more downforce and likely more drag, and
  the Katz edition (PDF labelled 2003, bib 1995) is to be verified.
  Sensitivity (diagnostics/inspect_drag_sensitivity.py, 14 sessions,
  thresholds fixed): at 1.0 m^2 against no drag, 2.2 % of corner verdicts
  (0.7 % of corner-phases) change, 3 of 57 front floor events disappear,
  the braking-filter share falls 0.4-1.6 points, rear K shifts < 0.5 /
  rad, mu_demo within 0.002, no status or fallback change; at 1.3 m^2 3.1
  % of corner verdicts -- ~~below the admissible observer-noise effect (15.6
  % at Q x 0.3)~~ [2026-10-27, SENS-1: comparison superseded -- the two
  numbers use different counting rules (corner verdict string vs per-axle
  tier class), both on the pre-D2 classifier; recount under one rule
  pending]. Rolling resistance stays 0. Outlook: a Level-2 session
  estimate from the power balance over full-throttle straights on a flat
  track [Trzesniowski.2019b, sec. 5.4, Eq. 5.13 and Tab. 5.5 (drivetrain efficiency), p. 238].
- IMU position unknown. The mounting position of the accelerometer
  relative to the CoG is not recorded; a_y is taken as the CoG value. A
  longitudinal offset dx adds dx times the yaw acceleration: with the 2 Hz
  filtered yaw acceleration that reaches the force split, |psi_dd| p95 /
  p99 = 0.79 / 1.60 rad/s^2 (NBR 29) and 0.77 / 1.50 rad/s^2 (Le Castellet
  25), i.e. about 0.08 g (p95) and 0.16 g (p99) of a_y error per metre of
  offset (against |a_y| p50 about 0.96 g).
- Fy / Fz is first-order only. Normalising removes the proportional load
  dependence; the degressive part remains -- the friction coefficient falls
  and cornering stiffness grows less than proportionally with load
  ([Milliken.1995, pp. 26-27]; [Pacejka.2006, sec. 1.2.1, page to verify]).
  No fit-L number for the residual load dependence is recorded.
