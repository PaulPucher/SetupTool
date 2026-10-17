# Dataflow registry -- every seam of every verdict-relevant quantity

Built 2026-10-17 (R4, rebuild of the first-pass registry), base acf7dbf on
branch new-laps. Read-only on code, config and tests: this file records what
the code does, it changes nothing. Line numbers refer to acf7dbf.

Why this file exists: the CS reference bug fixed in R3 (Fix 0) lived in a
cross-stage seam -- a regression window carried a reference value into later
CS samples, and the kerb rule applied to the newest sample only. Auditing
each stage on its own missed it. This registry lists, for every quantity
that reaches a verdict, a displayed statistic or the saved payload, the
state it carries between samples, windows, corners and laps, and which
exclusion rules guard each edge.

How to read an entry (five fixed parts):
1. COMPUTED IN: file and function.
2. DIRECT INPUTS: per-sample inputs with unit and rate at that point.
3. CARRIED STATE: everything that persists between samples, windows,
   corners or laps and influences the value; where set, where read, lifetime.
4. POPULATION RULES: which masks gate the inputs and the carried state, and
   at which edge -- the INPUT edge (samples entering the computation), the
   CARRIED-STATE edge (samples allowed to shape a value that is carried
   forward), or both. An edge where a rule does not act is listed as
   "UNPROTECTED EDGE" with its finding number.
5. DOWNSTREAM CONSUMERS: who reads the result.
Every entry ends with a STATUS line: "fully protected" or "edges open: ...".

Findings are numbered R1 to R26 (R26 added 2026-10-17 after the spike trace) and listed at the end with their class:
F1 defect, F2 claim-code mismatch, F3 unjustified choice or limitation.

Counts: the shipped pipeline (best available, cap None, sideslip source
ekf_auto_pacejka, the app default path) run fresh at acf7dbf on the three
validation exports -- Dubai sample export, NBR-73-010-026-JDR, Paul Ricard
MLA-SE07_US-v2 (PR). The saved R3 arrays were not used: diagnostics/output/
class_b/ holds logs only, no arrays.

"Verdict-read" sample (used throughout): a sample inside a statistic block
the classifier reads -- for CS the four time phases entry_1_brake,
entry_2_turnin, exit_4, exit_5 plus the apex region (the classifier's apex_3
CS), for stability all five phases -- whose block passes its count gate
(median finite), with a finite value in the population that block uses.

NOT IN SCOPE (stated by the work order): the other decision-frame evidence --
ABS, TC, TPMS, LS thresholds, brake balance -- and the LS and Fz statistic
blocks of the per-phase table (display and decision-frame inputs only, no
verdict). They share sections S1-S6 but are not registered here.


## Shared sections (S1-S6)

### S1 Time grid

- Set: stability_analysis.prepare_vehicle_state (stability_analysis.py:317-321).
  Grid rate = min(target_sample_rate_hz = 100, slowest native rate of
  sclu_yaw_rate, log_asteer, log_acc_y, log_acc_z, lap_distance)
  (_resolve_grid_rate, :109-145); 100 Hz on all three exports. Grid spans
  ecu_speed's time range (np.linspace); every channel linearly interpolated
  onto it (ecu_speed 50 Hz upsampled).
- Lap distance s_m [m]: unit-converted (_normalize_lap_distance_to_metres,
  :210) and interpolated with the reset guard (:223-233): grid samples
  bracketing a lap-counter reset get NaN (4 / 8 / 8 samples on analysis laps,
  Dubai / NBR / PR).
- Durations converted to samples once per session at the grid rate (carried,
  session lifetime): kerb_dilation_s (:366), kerb_smear_grade_s (:368),
  cs_min_window_s (resolve_cs_min_window_samples, :740), EKF and gate NIS
  windows nis_window_s (sideslip_ekf_pacejka.py:105, nis_gate.py:283),
  yaw_stability_accel_window_s (yaw_stability.py:20). Still a sample count:
  apex_half_window_samples = 5 (+/-0.05 s at 100 Hz, PRODUCTION.md 2.1).
- Lap windows are native lap_number time spans (csv_parser._split_laps): the
  grid samples between one lap's last and the next lap's first native sample
  belong to no lap (116 / 248 / 208 samples) and count as off an analysis lap.
- Consumers: every quantity below.

### S2 Moving mask

- Set: prepare_vehicle_state, moving = v > moving_speed_min_mps = 5.0 m/s
  (stability_analysis.py:374). Settings-editable (settings_view.py:89).
  Lifetime: session, carried inside state (and every cache holding state).
- Acts on: kinematic beta (beta_dot = 0, output 0), slip angles and axle
  forces (0 before the 2 Hz filter), the CS loop (no value), the EKF (state
  and P reset), stability (regression population), tyre-fit base mask,
  phase sample sets, balance population, speed-window percentile population.
- On analysis laps the gate never closes on the three exports (0 non-moving
  samples), so its edges have no effect there.
- Cache identity: NOT in any (finding R2).

### S3 Kerb mask and smear zone

- Mask set: _compute_kerb_mask_from_az (stability_analysis.py:265-278):
  |a_z - kerb_baseline_g| > kerb_z_deviation_threshold_g, dilated by
  +/- round(kerb_dilation_s x rate) samples. Config 1.0 g / 1.2 g / 0.10 s;
  all three Settings-editable (settings_view.py:92-101).
- Smear zone set: _kerb_smear_zone (:285-299): unmasked samples within
  round(kerb_smear_grade_s x rate) = 20 samples of a mask sample.
- Lifetime: session, carried inside state.
- Where each rule acts (edge by consumer):
  - CS value: mask at the INPUT edge for sample i only (stability_analysis.py:
    800-802, :847); window samples [start, i) may be masked.
  - CS reference: mask + smear + off-analysis at the CARRIED-STATE edge (Fix 0,
    :811-816, :930).
  - CS grade mark (near_strike): smear only, over [start, i) (:938-939).
    UNPROTECTED EDGE: a window holding mask samples but no smear sample
    (R1).
  - Kinematic beta, EKF state: neither rule (R5).
  - Tyre fit, R derivation, NIS sweep, NIS gate, sign check, h2: mask at the
    input edge (tyre_fit_auto._base_mask, :44-54); smear UNPROTECTED (R6).
  - Speed-window percentile population: mask not applied, documented in
    the _speed_window_mask docstring ("kerb samples included").
  - Stability regression: mask at the input edge (:1004-1007); smear
    UNPROTECTED (R10); yaw-acceleration filter runs over mask samples (R10).
  - Stability phase statistics: neither rule (R8).
  - Balance: mask at the input edge (balance_gradient.py:212-214); smear
    UNPROTECTED (R7).
  - Limit events: full CS arrays; label = grade mark in the hits or mask in
    the span (limit_events.py:154-155); inherits R1.
  - Decision-frame kerb flags: the mask is the evidence window itself.
  - Cross-lap envelopes in the trace window: neither rule (R25).
- Cache identity: threshold, baseline, dilation NOT in any (R2).

### S4 Analysis-lap mask

- Set: csv_parser._split_laps (csv_parser.py:500-521): is_valid_for_analysis
  = not out-lap, not in-lap, effective lap time <= fastest x
  valid_lap_max_ratio and > lap_time_min_s, no lap warnings. Lifetime:
  session, carried in parsed_data["laps"].
- Representations: corners exist only on analysis laps
  (corner_analysis.analyse_corners, :70-72, and _realize_canonical_corners);
  _build_non_analysis_lap_mask (stability_analysis.py:249-262, sample mask,
  CS reference and stability); tyre_fit_auto._base_mask (racing windows);
  balance_gradient on_lap; limit_events and sign check via laps_by_number.
- Not applied: kinematic beta integrator and washout, EKF state (both run
  through out-laps and in-laps, R5 bucket), the 2 Hz filters (S5), the
  stability field's interpolation (it fills every sample with a lap
  distance, out-laps included -- documented PRODUCTION.md 10).

### S5 The 2 Hz zero-phase filter

- Where: _butterworth_lowpass (stability_analysis.py:78-84), 4th order,
  filtfilt over the WHOLE session array: slip angles (:563-564), axle forces
  (:607-608), the balance signals (balance_gradient.py:241-251). The beta
  washout high-pass (:444) uses the same design at 0.05 Hz.
- Carried state: the filter's impulse response, both directions in time,
  across every mask: kerb strikes leak into neighbours (the reason for the
  smear zone, S3), lap boundaries, the moving gate (inputs zeroed below
  5 m/s are filtered with the rest -- no effect on analysis laps, S2).
- Population rules: none at this stage by design (filter first, mask after);
  the downstream smear zone is the only compensation, and it covers the CS
  grade mark and the CS reference only (R6, R7, R10 list the rest).

### S6 Caches

Every store that can render a result without recomputing it:
- load_parameters / load_car_data (stability_analysis.py:61-75):
  functools.lru_cache, process lifetime. Cleared by a Settings save
  (settings_view.py:800-801); a hand edit of config is seen only after a
  restart.
- In-memory pipeline cache (outing_form.py:57-87): LRU of 2, keyed by csv
  path; holds corners, state, cs, stab, fz, ls, slip, forces, fit manifest,
  gate, fallback, balance, events. Identity checked by the caller
  (:1380-1387): accuracy cap, resolved vehicle snapshot, sideslip_source,
  grid rate. Cleared by a Settings save (invalidate_all_pipeline_caches,
  :51-54, called at settings_view.py:806-807).
- In-form fast path (outing_form.py:1322-1346): _last_render_kwargs +
  stability_result; identity cap, snapshot, sideslip_source, grid rate,
  lap_filter. NOT cleared by a Settings save.
- parsed_data (form lifetime): laps and corners, including speed_class, are
  computed at parse time from channels.json (corner_speed_thresholds,
  corner_detection); a re-Analyse never re-parses.
- DB payload outings.analysis_data (outing_form.py:1497-1541, read at
  :1543-1637): identity schema_version, csv_path, cap, snapshot,
  sideslip_source, grid rate, lap_filter. Verdicts are classified live from
  config at render, never stored.
- Sidecar data/analysis_cache/outing_<id>.pkl.gz (pipeline_sidecar.py):
  same identity + sidecar_format_version; hit warms the in-memory cache.
- Readers without an identity check: core/thesis_data.read_sidecar (reports
  schema_version, does not refuse); core/weekend_pdf_export checks
  schema_version only (:140).
- Display caches keyed to the state object: corner_trace_dialog _yaw_cache
  (:542-547), outing_form corner_positions_cache.
- Identity coverage: nothing in any identity sees the Settings Section-2
  fields (moving speed, kerb threshold, kerb baseline, kerb dilation, corner
  speed boundaries) -- R2; a code change without a schema bump is invisible
  to the persisted stores -- R3.


## Quantities

### Q1 Kinematic beta (seed and fallback)

1. COMPUTED IN: stability_analysis.estimate_sideslip (:427-446).
2. DIRECT INPUTS: v [m/s], a_y [m/s^2], r [rad/s], all on the grid (100 Hz);
   moving mask.
3. CARRIED STATE:
   - Integrator beta_raw = cumsum(beta_dot) dt (:442). Set every sample,
     read every later sample; lifetime SESSION, never reset (a non-moving
     sample adds 0 but keeps the sum).
   - Washout high-pass, 0.05 Hz zero-phase (:444): every sample depends on
     samples seconds before AND after it; lifetime session.
4. POPULATION RULES:
   - Moving: input edge (beta_dot = 0) and output (beta = 0) (:438-445).
   - Kerb mask: UNPROTECTED EDGE, input and carried state (R5).
   - Analysis laps: UNPROTECTED EDGE, out-lap and in-lap samples feed the
     integrator and washout carried into analysis laps (R5 bucket).
   - Smear zone: not applicable upstream of the 2 Hz filter.
5. DOWNSTREAM CONSUMERS: tyre fit seed (tyre_fit_auto.py:430-431) and the
   plausibility gate population; session-wide fallback beta
   (tyre_fit_auto.py:735, :745) -> Q3, Q7, Q8.
STATUS: edges open: kerb mask (R5), analysis laps (R5).

### Q2 EKF beta / shipping beta

1. COMPUTED IN: sideslip_ekf_pacejka.estimate_sideslip_ekf_pacejka
   (:50-211), run by tyre_fit_auto.fit_session_pacejka (interim :511, 25
   sweep points :551, final :592); dispatched by
   tyre_fit_auto.resolve_sideslip_beta (:704-754).
2. DIRECT INPUTS: v (Vx floored at 5 m/s) [m/s], delta_f [rad], measured r
   [rad/s] and a_y [m/s^2], grid rate; moving mask.
3. CARRIED STATE:
   - State x = [beta, r] and covariance P: set every moving sample, read by
     the next; lifetime until the next non-moving sample (reset to 0 / P0,
     :124-131) or a divergence (P -> P0; hard bound also beta -> 0, r ->
     measured, :191-199). In practice SESSION-long across lap boundaries.
   - nis_recent, trailing 0.4 s of NIS exceedance flags (:177-181): window
     lifetime, cleared on reset/divergence.
   - Session constants: Pacejka B, C, D, E per axle from the fit (Q7), R
     from the sweep pick, Q scaled by q_reference_rate_hz / rate, P0.
   - Dispatch decision (resolve_sideslip_beta): fit degenerate or gate
     'fail' -> kinematic beta (Q1) for the WHOLE session.
4. POPULATION RULES:
   - Moving: input and carried state (reset).
   - Kerb mask: UNPROTECTED EDGE, the filter updates through kerb samples
     and carries that state on (R5).
   - Analysis laps: UNPROTECTED EDGE, state carried from an out-lap into
     lap 1 (R5 bucket).
   - Smear: n/a (raw measurements, no 2 Hz filter here).
5. DOWNSTREAM CONSUMERS: slip angles (Q3), stability regression (Q8), the
   fit's own sign check / coverage / h2 (Q7), payload sideslip_source,
   fit_manifest, gate_verdict, fallback_*.
STATUS: edges open: kerb mask (R5), analysis laps (R5).

### Q3 Slip angles (and the axle forces they are regressed against)

1. COMPUTED IN: stability_analysis.estimate_slip_angles (:538-571);
   axle forces estimate_lateral_forces (:574-618).
2. DIRECT INPUTS: beta (Q1 or Q2) [rad], delta_f [rad], v [m/s], r [rad/s],
   a_y [m/s^2], raw d r/dt (np.gradient) [rad/s^2], grid rate. Config a, b,
   m, I_z, corner weights (Level 1/2, accuracy resolution).
3. CARRIED STATE:
   - Everything beta carries (Q1/Q2).
   - The 2 Hz zero-phase filter (S5), both directions in time, session-wide.
4. POPULATION RULES:
   - Moving: input edge (0 before filtering).
   - Kerb mask, smear, analysis laps: none at this stage by design (filter
     first); the smear reach is what S3's zone models.
5. DOWNSTREAM CONSUMERS: CS regression (Q4), tyre fit (kinematic version,
   Q7), EKF coverage check (Q7), limit-event window reconstruction (Q12),
   tyre-curve display, sidecar.
STATUS: edges open: kerb mask and analysis laps through beta (R5); kerb
smear by construction (S5, handled downstream by S3 where it is handled).

### Q4 Cornering stiffness ratio CS, front and rear (values)

1. COMPUTED IN: stability_analysis.estimate_cornering_stiffness (:782-973),
   inner compute_cs_for_axle (:828-943).
2. DIRECT INPUTS: alpha_f/r_filt [rad], Fy_f/r_filt [N] (Q3), s_m [m],
   moving, kerb mask, smear zone, off-analysis mask (S2-S4), all at the
   grid rate; config cs_min_window_s (-> 10 samples), cs_min_slip_angle_
   span_rad 0.02, cs_max_window_m 53.0, cs_linear_slip_threshold_rad 0.03.
3. CARRIED STATE:
   - Regression window [start, i): grows backward from 10 samples until the
     alpha span reaches 0.02 rad, capped at 53 m of track or a lap-distance
     reset (:853-870). Window lifetime; R11 (lap boundary).
   - Monotonic sections and their whole-section slopes (:843-844): one
     partition of the whole session; clipped and re-fitted per window
     (#26, :898-913), so no out-of-window data enters.
   - LINEAR REFERENCE C_linear_ref (CS denominator): SET at :930-931 by a
     window whose max |alpha| < 0.03 rad and which holds no kerb-mask, smear
     or off-analysis sample (Fix 0); READ at :933-936 by every later sample;
     LIFETIME until the next clean set -- no age limit, crosses lap
     boundaries (R4). Stored per sample as C_linear_ref_f/r (NaN wherever
     the loop skipped -- a value reappearing after such a gap is NOT a new
     set event).
   - no_reference flag (:933-934): stiffness computed, no reference yet ->
     CS stays NaN (R12).
4. POPULATION RULES:
   - Moving and kerb mask at sample i: input edge (:800-802, :847).
   - Kerb mask inside the window [start, i): UNPROTECTED at the input edge
     (window may hold masked samples); caught by the grade mark only via the
     smear zone (R1).
   - Kerb mask, smear zone, analysis laps, linear limit at the
     carried-state edge (the reference): enforced (Fix 0). Verified below.
   - Analysis laps at the input edge: CS is computed on every lap; only
     analysis-lap corners are read (Q5). Window can straddle a lap boundary
     (R11, latent).
   - Reference age: UNPROTECTED (R4).
   - Reference value: UNPROTECTED, no bound (R26, added 2026-10-17).
5. DOWNSTREAM CONSUMERS: per-phase statistics (Q5), limit events (Q12),
   trace windows and cross-lap envelopes (R25), thesis_data.read_sidecar,
   tyre-curve audit plot (C_linear_ref_f/r), sidecar and in-memory cache.
   n_no_reference_* are in memory only (R12).
FIX 0 VERIFICATION (every reference SET event, detected as C_linear_ref == C_alpha and independently by re-evaluating the set condition on the reconstructed window; both detections agree sample for sample): front / rear Dubai 2640 / 3104, NBR 15456 / 14980, PR 4585 / 4836 -- all equal to the first pass. Windows holding a kerb-mask or smear sample: 0. Windows reaching off an analysis lap: 0. Windows above the linear slip-angle limit: 0. Pitfall confirmed: reference values reappearing after a NaN gap without a set event, front / rear Dubai 139 / 150, NBR 232 / 237, PR 380 / 346 -- none counted as set events.
STATUS: edges open: kerb mask inside the window (R1), reference age across
laps (R4), reference value unbounded (R26), beta's kerb carry (R5), lap boundary inside the window (R11,
latent), no-reference visibility (R12).

### Q5 CS per-phase statistics (and the near-strike grade mark they read)

1. COMPUTED IN: stability_analysis.summarise_corners (:1047-1277); CS blocks
   via _cs_stat (:1113-1118); grade mark from Q4 (:938-939).
2. DIRECT INPUTS: CS_ratio_f/r and near_strike_f/r (Q4), moving mask, phase
   time windows and canonical apex distance (Q10), s_m; config
   apex_half_window_samples 5, cs_phase_min_valid_samples 5,
   cs_apex_region_half_length_m 25.0.
3. CARRIED STATE:
   - Corner phase boundaries (Q10, session-canonical).
   - Grade mark: a per-sample flag derived from the sample's own window
     (window lifetime), no carry beyond it.
   - Through the CS values: the reference (Q4) and beta (Q1/Q2).
4. POPULATION RULES:
   - Analysis laps: input edge (corners exist only there).
   - Moving: input edge (phase index sets).
   - Kerb mask: input edge indirectly (CS is NaN at mask samples).
   - Smear zone: input edge via the grade mark (samples set NaN, :1081-1082).
   - Window entirely inside a kerb run: UNPROTECTED (R1).
   - Count gate: < 5 finite clean samples -> NaN (:1107-1111).
   - No-reference samples: silently absent, no count in the block (R12).
5. DOWNSTREAM CONSUMERS: classifier _classify_corner (per corner-lap card,
   via _verdict_cs_stat), aggregation (Q6), per-phase table, payload
   summaries (persisted), PDF export.
STATUS: edges open: R1, R12; carried: R4, R5 through CS.

### Q6 Worst-lap CS values and cross-lap aggregation

1. COMPUTED IN: recommendation.aggregate_by_corner (:96-145), combiner from
   config classification.cs_cross_lap_aggregation = "worst_lap" (min of
   per-lap phase medians, apex region likewise); stability by median.
2. DIRECT INPUTS: the per-corner-lap summaries (Q5, Q9).
3. CARRIED STATE: none of its own; the min over laps lets ONE lap's block
   decide the corner, so every per-lap seam (R1, R4, R12) propagates
   unattenuated into the aggregate.
4. POPULATION RULES: inherited from Q5/Q9; NaN blocks dropped
   (_nanmin_or_nan); a corner with no finite block -> NaN (R13 path).
5. DOWNSTREAM CONSUMERS: decision frame (corner_verdict, matrix_verdict
   evidence), track-map markers, recommendation rule engine.
STATUS: edges open: inherited R1, R4, R12, R13.

### Q7 Tyre fit and fit acceptance (status, sign check, NIS gate score, R pick)

1. COMPUTED IN: tyre_fit_auto.fit_session_pacejka (:395-701); gate
   nis_gate.evaluate_gate (:290-323) called from resolve_sideslip_beta
   (:737-738); plausibility_gate (:369-392).
2. DIRECT INPUTS: kinematic-seed slip angles and axle forces (Q1, Q3) after
   2 Hz, v (speed window), measured r and a_y (EKF), corners and laps.
3. CARRIED STATE:
   - Fit parameters B, C, D, E per axle: SESSION, set once, read by every
     EKF run, the gate, the event line (Q12), the tyre-curve display.
   - R_ay from the fit residuals, R_r from the interim EKF, the 25-point
     sweep and its pick: session.
   - EKF state per run (Q2).
   - NIS gate trailing window 0.4 s over the full-length NIS, looking back
     across mask gaps (documented, nis_gate.py:260-266).
   - Sign check: per stable corner, beta and a_y pooled over all analysis-lap
     instances inside the canonical bracket (:72-116); racing corners =
     speed_class != low (:527), speed_class from Q10.
4. POPULATION RULES:
   - Fit population = base mask (moving, outside the kerb mask, analysis
     laps) AND speed window [p1, p99] (:422-426): mask and analysis laps
     enforced at the input edge.
   - Smear zone: UNPROTECTED EDGE for the fit, R derivation, sweep, gate and
     sign check (R6).
   - Speed-window percentile population: kerb samples included (documented).
   - Gate window: reaches across kerb and off-analysis samples (documented).
   - Event line from a gate-failed fit: R14.
5. DOWNSTREAM CONSUMERS: dispatch (Q2), event line (Q12), payload
   fit_manifest / gate_verdict / fallback_*, estimator status line,
   tyre-curve display.
STATUS: edges open: smear (R6), speed classes from un-cached settings (R2, R23),
gate-failed line (R14).

### Q8 Observed stability (regression inputs and output)

1. COMPUTED IN: stability_analysis.estimate_yaw_moment_stability
   (:976-1044); yaw_stability.calculate_filtered_yaw_acceleration (:15-34)
   and calculate_observed_stability (:37-161).
2. DIRECT INPUTS: beta (Q1/Q2), delta_f, v, a_x [m/s^2], a_z [m/s^2],
   Mz_inertial = I_z x filtered yaw acceleration [Nm], s_m [m]; grid rate.
3. CARRIED STATE:
   - Yaw acceleration: np.gradient then centred rolling mean of 15 samples
     (0.15 s), bfill/ffill at the ends (:20-33): window lifetime, runs over
     kerb samples (R10).
   - Cross-lap pooling: all analysis-lap samples sorted by lap distance, one
     regression per 2 m grid point over +/-55 m with Gaussian weights:
     SESSION lifetime; every lap's samples shape every other lap's value at
     the same track position.
   - Grid field: interpolated linearly onto every sample's lap distance,
     ACROSS grid points where the regression refused (min_samples /
     beta_std / linalg) (:146-153) -- R9.
   - Config I_z and the six yaw_stability_* values (session).
4. POPULATION RULES:
   - Moving, kerb mask, analysis laps: input edge of the regression
     (valid_mask, :1003-1007, NaN-ing at yaw_stability.py:77).
   - Smear zone: UNPROTECTED (R10).
   - Kerb mask through the yaw-acceleration filter and through beta's state:
     UNPROTECTED (R10, R5).
   - Output edge: the field is written to every sample with a finite lap
     distance, kerb samples and out-laps included (documented PRODUCTION.md
     10); its readers must re-apply the masks (R8).
5. DOWNSTREAM CONSUMERS: stability phase statistics (Q9), trace windows
   and the cross-lap envelope (R25), sidecar.
STATUS: edges open: R5, R9, R10; output re-masking left to readers (R8).

### Q9 Stability per-phase statistics

1. COMPUTED IN: summarise_corners (:1245-1258), _gate_stab_stat (:1127-1136).
2. DIRECT INPUTS: stability_observed_Nm_per_deg (Q8), moving mask, phase
   windows (Q10), brake_f_bar for the no-braking gate.
3. CARRIED STATE: everything Q8 carries (cross-lap pooling, the
   interpolation across refused grid points); phase windows.
4. POPULATION RULES:
   - Analysis laps, moving: input edge.
   - Kerb mask: UNPROTECTED -- the phase set uses the moving mask only
     (:1071, :1209-1210), so the field is read at kerb positions (R8).
   - Smear: not applied (R10).
   - Count gate (cs_phase_min_valid_samples reused) and entry_1_brake
     no-braking gate (3.0 bar).
5. DOWNSTREAM CONSUMERS: classifier (open-loop unstable, severity),
   aggregation (median across laps, Q6), per-phase table, payload.
STATUS: edges open: kerb mask (R8), smear (R10), interpolation (R9),
beta's kerb carry (R5).

### Q10 Corner phases

1. COMPUTED IN: corner_analysis.analyse_corners (:56-80), called by
   csv_parser.parse_csv; canonical realisation _realize_canonical_corners
   (:587-745).
2. DIRECT INPUTS: native-rate log_asteer, ecu_speed, log_acc_y, ecu_aps,
   lap_distance (not the grid); laps; channels.json corner_detection and
   corner_speed_thresholds.
3. CARRIED STATE:
   - Canonical boundaries per stable corner: median over representative laps
     (<= 1.05 x fastest), SESSION lifetime, applied to every analysis lap.
   - Stable corner ids (cross-lap clustering), session.
   - speed_class from the median apex speed vs low_max / medium_max
     (Settings-editable, NOT re-applied without a re-parse -- R2).
   - Lifetime beyond the session: parsed_data in the form; corners in the
     in-memory cache and sidecar.
4. POPULATION RULES: analysis laps at the input edge; no kerb or moving rule
   (geometry only).
5. DOWNSTREAM CONSUMERS: summaries (Q5, Q9), tyre fit speed window, sign
   check and h2 (Q7), balance brackets (Q13, removed 2026-10-17), events (Q12), decision frame
   (kerb flags Q14, TPMS, ABS), track map.
STATUS: edges open: speed class staleness after a Settings edit (R2).

### Q11 Classification and headline numbers

1. COMPUTED IN: ui/views/outing_form.OutingForm._classify_corner (:375-514),
   accessor _verdict_cs_stat (:517-524); also called (self = None) by the
   decision frame and the PDF export. Thresholds read live from
   config/parameters.json classification at every call.
2. DIRECT INPUTS: per corner-lap (cards) or per corner aggregate (decision
   frame, track map): the five phases' CS medians (apex_3 from the apex
   region) and stability medians (Q5, Q6, Q9).
3. CARRIED STATE: none of its own; thresholds and cs_margin from config
   (never cached: classified live on every render). Calibration markers
   compare config strings only (documented PRODUCTION.md 13.1).
4. POPULATION RULES: inherited. A corner-lap with no finite CS on either
   axle starts from worst = 1.0 and reads "ok / Within normal range" unless
   stability fires (:403-408, :503-506) -- R13.
   Headline numbers on the card (CS value, Nm/deg) are the worst-phase
   medians themselves, same population.
5. DOWNSTREAM CONSUMERS: verdict cards, per-phase table colours, decision
   frame (corner_verdict, matrix_verdict evidence), rule engine, PDF export.
STATUS: edges open: R13; inherited R1, R4, R8, R12.

### Q12 Limit events

1. COMPUTED IN: limit_events.detect_events (:89-175); line _event_line
   (:53-66), curve_floor (:41-50).
2. DIRECT INPUTS: CS_ratio_f/r FULL arrays and near_strike_f/r (Q4), kerb
   mask, s_m, corners and analysis laps, fit_manifest (Q7), slip angles (for
   the window start).
3. CARRIED STATE:
   - Event line per axle: curve floor of the session's fitted curve,
     SESSION; refused only for a degenerate or absent fit (R14).
   - Run merging across gaps < 0.5 s within one corner instance.
   - Recurrence across analysis laps (+/-10 m), session.
4. POPULATION RULES:
   - Analysis laps, canonical bracket: input edge.
   - Kerb: kept and labelled (documented); label misses a window lying
     entirely inside a kerb run (R1 consequence).
   - No-reference samples: NaN, invisible (R12 family).
5. DOWNSTREAM CONSUMERS: verdict-card line, session list, trace strips,
   payload "events" (persisted). Feeds no verdict (tested, PRODUCTION.md
   13.4).
STATUS: edges open: R1 (label), R14 (latent on these exports).

### Q13 Balance line (understeer gradient) -- REMOVED 2026-10-17, K readout deleted (R4b); entry kept as the record

1. COMPUTED IN: balance_gradient.estimate_understeer_gradient (:195-269);
   reading balance_reading (:272-279).
2. DIRECT INPUTS: delta_f [rad], r [rad/s], v [m/s], a_x and a_y [m/s^2],
   2 Hz-filtered over the whole session (S5); corners, laps, s_m.
3. CARRIED STATE:
   - Otsu |a_y| split on the racing population, SESSION.
   - Steering zero offset = median delta_f on racing samples outside
     corners, SESSION.
   - The 2 Hz filter state (S5); bootstrap seed from config.
4. POPULATION RULES:
   - Moving, kerb mask, analysis laps: input edge (:212-221).
   - Smear zone: UNPROTECTED (R7).
   - Quasi-steady gates |psidd|, |a_x|, a_y clip.
5. DOWNSTREAM CONSUMERS: the UI balance line, payload "balance".
STATUS: edges open: smear (R7).

### Q14 Decision-frame kerb flags

1. COMPUTED IN: decision_frame._build_kerb_blowoff_evidence (:598-651),
   kerb_event_flags (:659-668), axle attribution _kerb_axle_attribution
   (:562-595).
2. DIRECT INPUTS: a_z [g] on the grid, kerb mask (S3), corner overall time
   windows (Q10), damper travel channels (native, interpolated).
3. CARRIED STATE: config severity_threshold_g (pooled p75 of two sessions)
   and repeat_min_laps 2 (session constants); the kerb mask carried in the
   cached state (S6).
4. POPULATION RULES: corners on analysis laps only (input edge); the mask is
   the evidence selector. Kerb parameters edited in Settings do not reach a
   cached state (R2).
5. DOWNSTREAM CONSUMERS: decision-frame flag lines and tooltip
   (kerb_event_population_note); never a candidate.
STATUS: edges open: R2 (stale mask after a Settings edit).


## Status lines (one per quantity)

- Q1 kinematic beta: edges open: kerb mask, analysis laps (R5).
- Q2 EKF / shipping beta: edges open: kerb mask, analysis laps (R5).
- Q3 slip angles and axle forces: edges open: R5 through beta; filter smear by construction (S5).
- Q4 CS values: edges open: R1, R4, R5, R11 (latent), R12, R26. Reference edge (Fix 0) protected and verified.
- Q5 CS per-phase statistics: edges open: R1, R12; carried R4, R5.
- Q6 worst-lap values and aggregation: edges open: inherited R1, R4, R12; R13 path.
- Q7 tyre fit and acceptance: edges open: R6, R2 / R23, R14 (latent).
- Q8 observed stability: edges open: R5, R9 (latent), R10; output re-masking left to readers (R8).
- Q9 stability per-phase statistics: edges open: R8, R9 (latent), R10, R5.
- Q10 corner phases: edges open: R2.
- Q11 classification and headline numbers: edges open: R13; inherited R1, R4, R8, R12.
- Q12 limit events: edges open: R1 (label, 0 cases here), R14 (latent).
- Q13 balance line: removed 2026-10-17, K readout deleted.
- Q14 decision-frame kerb flags: edges open: R2.
- S6 caches (cross-cutting): edges open: R2, R3, R24.
No quantity is fully protected; the CS reference edge is the one seam
verified closed.


## Findings

Classes: F1 defect, F2 claim-code mismatch, F3 unjustified choice or
limitation. Numbers front / rear; sessions Dubai / NBR / PR.

- R1 F1 -- stability_analysis.py:938-939: the CS grade mark tests the smear zone only, not the kerb mask, so a window [start, i) lying entirely inside a kerb run (i the first sample after it) is neither masked nor graded. Finite unmarked CS with a mask sample in the window: Dubai 28 / 23, NBR 28 / 24, PR 111 / 88, every one entirely inside the run. Verdict-read (block membership, as the first pass counted): Dubai 19 / 15, NBR 23 / 21, PR 75 / 54 (count gate applied: 15 / 9, 23 / 21, 70 / 46). Treating them as kerb-affected moves classifier-read medians, front / rear: Dubai 20 (+2 to NaN) / 11 (+1), max 0.018 / 0.010; NBR 23 / 17, max 0.026 / 0.031; PR 67 (+1) / 35, max 0.079 / 0.078. The same gap sits in the display and thesis definition "mask or grade mark" (core/thesis_data.py:44, trace grey) and in the limit-event kerb label (0 events affected on these exports). STATUS FIXED 2026-10-17 (R5): the grade mark covers kerb mask and smear zone; newly excluded verdict-read 19 / 15, 23 / 21, 75 / 54 (as recorded); no CS value changes.
- R2 F1 -- outing_form.py:1244-1256, :1380-1387, :1560-1610; pipeline_sidecar.py:21-24: the Settings estimator inputs (moving speed, kerb threshold, kerb baseline, kerb dilation, corner speed boundaries; settings_view.py:89-111) are in no cache identity. After an edit the DB payload and the sidecar re-render stale results as current; the in-form fast path (:1322-1346) is not cleared by the Settings save and reports "results current (cached)"; parsed_data keeps the old corner speed classes. Only the in-memory pipeline cache is cleared (settings_view.py:806-807). STATUS FIXED 2026-10-17 (R5): accuracy_resolution.resolve_estimator_settings joins every cache identity (in-process cache, fast path, payload, sidecar); a Settings save clears the in-form fast path; speed classes recomputed before every full run; Settings asks for a re-run.
- R3 F1, deferred to the schema bump -- stability_analysis.py:53: R2 and R3 changed estimator results under ANALYSIS_SCHEMA_VERSION 15, so a payload or sidecar saved at schema 15 before the fixes renders as current. This machine's DB holds schema 8 (5 outings) and 14 (1), no 15: latent here; the laptop's state is unknown.
- R4 F3 -- stability_analysis.py:930-936: the linear reference carries across lap boundaries with no age limit. Verdict-read samples dividing by the previous lap's reference: Dubai 0 / 0, NBR 0 / 0, PR 562 / 410 (block membership, as the first pass counted; 562 / 409 with the count gate), reference age 9.2-16.1 s front, 9.2-12.5 s rear. Including graded samples: 863 / 969. STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), thesis_notes R5 entry.
- R5 F3 -- stability_analysis.py:427-446, sideslip_ekf_pacejka.py:123-200: the kinematic integrator and washout and the EKF state run through kerb samples (and through out-laps) and carry that state on. Verdict-read CS samples with a mask sample in the previous 1 s: Dubai 4.11 / 3.39 %, NBR 1.98 / 1.94 %, PR 9.99 / 10.29 %. Stability: of the regression population (the samples that shape the field) 10.48 / 4.99 / 16.19 % (= the first pass's 5-16 %); of the verdict-read stability samples 15.86 / 8.41 / 27.46 % (20.68 % on PR when the kerb positions of R8 are left out). STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), thesis_notes R5 entry.
- R6 F2 -- tyre_fit_auto.py:44-54, :422-426: the tyre-fit population (and the R derivation, NIS sweep, gate and sign check on the base mask) excludes the kerb mask but not the smear zone, contradicting PRODUCTION.md 2.5 ("samples it disturbs must not shape the tyre estimate"). Smear share of the base mask: 4.52 / 2.32 / 7.44 % (= the first-pass figures); of the fit population after the speed window: 4.89 / 2.34 / 7.70 %. STATUS FIXED 2026-10-17 (R5): the tyre-fit base mask excludes the smear zone with the kerb mask; first-pass numbers reproduced.
- R7 F3 [removed 2026-10-17, K readout deleted] -- balance_gradient.py:212-221, :247-248: the same smear gap in the balance regression population: 125 of 3206 (3.90 %), 177 of 11459 (1.54 %), 356 of 4424 (8.05 %).
- R8 F2 -- stability_analysis.py:1071, :1209-1210, :1245-1258: stability phase statistics read the field at kerb positions (the phase set is the moving mask only), against PRODUCTION.md 2.5 ("must not shape ... the verdict") and 13.3. Samples, per block occurrence: Dubai 2005, NBR 2333, PR 6769 (unique 1855 / 2222 / 6401). Excluding them changes 80 / 68 / 223 phase medians (66 moved + 14 to NaN, 58 + 10, 190 + 33). STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), chair-faithful, thesis_notes R5 entry.
- R9 F3 -- yaw_stability.py:146-153: the stability field is interpolated linearly across grid points where the regression refused (chair code does the same). Refused interior grid points: Dubai 0, NBR 8, PR 0; verdict-read stability samples bridged over a refused gap: 0 on all three. Latent here. STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), chair-faithful, thesis_notes R5 entry.
- R10 F3 -- yaw_stability.py:25-33, stability_analysis.py:1004-1007: the 15-sample yaw-acceleration window runs over kerb samples (regression samples whose window reaches a mask sample: 1.71 / 0.91 / 2.82 %); stability has no smear-zone treatment (regression samples in the smear zone: 4.52 / 2.32 / 7.44 %). STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), chair-faithful, thesis_notes R5 entry.
- R11 F3, latent -- stability_analysis.py:853-870: a CS window can straddle a lap boundary (the s_m cap stops growth at a reset, the 10-sample minimum window does not). Finite-CS windows straddling a boundary: Dubai 0 / 0, NBR 2 / 2, PR 1 / 1; verdict-read: 0 on all three. STATUS FIXED 2026-10-17 (R5): a window over a lap-table boundary yields no value and is counted (n_lap_straddle_*); removed 99 / 2 / 1 samples per axle, 0 verdict-read; all at lap-table boundaries (first-pass 598 / 1438 / 1162 was another counting definition, author ruling).
- R12 F2 -- stability_analysis.py:933-934, :965-972: samples with a stiffness but no reference are NaN and appear nowhere in the summaries, the payload or the per-phase table (n_no_reference_* is in memory only; the table's "too few samples" text does not name them). Inside classifier-read blocks: Dubai 0 / 0, NBR 0 / 0, PR 327 / 184 (on analysis laps 81 / 81, 0 / 0, 419 / 231). STATUS FIXED 2026-10-17 (R5), in-memory: the analysis-lap no-reference count is on the summary line; per-phase counts deferred to the schema bump (payload shape).
- R13 F2 -- outing_form.py:403-408, :503-506: a corner-lap with no finite CS on either axle starts from worst = 1.0 and shows "ok / Within normal range". Dubai 2 of 56, NBR 13 of 180, PR 3 of 153 corner-laps, all shown "ok". STATUS FIXED 2026-10-17 (R5): 'no CS data', neutral colour, own headline count, severity stays normal; 2 / 13 / 3 corner-laps (unchanged by the band rule).
- R14 F2 -- limit_events.py:53-66: the event line is drawn from any non-degenerate fit, including one whose NIS gate failed (that session ships kinematic beta). Gate pass on all three exports: no effect here. STATUS FIXED 2026-10-17 (R5): no event line when the NIS gate failed (gate pass on all three: no effect here).
- R15 F2 -- stability_analysis.py:961-962: comment "read by the tyre fit and the trace display, never by the classifier" -- the tyre fit does not read near_strike; summarise_corners, the classifier's input, does (:1079-1082).
- R16 F2 -- config/parameters.json:96 (kerb_smear_grade_s_derived_from): "Flag only -- no exclusion, no down-rank, no verdict change" -- since schema 12 the grade mark removes samples from the verdict statistics, and since Fix 0 the zone bars windows from setting the reference.
- R17 F2 -- PRODUCTION.md 8.1 step 5 (:535-540, "a running hold through the session"; any window under 0.03 rad sets the reference), 8.2 (:549-554, "the grade mark is what catches" masked spans), 12.2 (:791-793, clean population = graded set to NaN, masked already NaN): the pre-Fix-0 reference rule, and silent on R1. Not resolved by R3 (R3 edited 5.1 only).
- R18 F2 -- PRODUCTION.md 2.1 (:151-156) and 4 (:383) still state nis_window_samples = 20 (key deleted in R3); 4 lists Q without its dt-scaling (q_reference_rate_hz). 5.1: RESOLVED by R3's edit for the NIS window (0.4 s stated); 5.1 still does not mention the Q scaling.
- R19 F2 -- PRODUCTION.md 2.6 (:258-263) and 10 (:695, :705): "out- and in-laps" excluded from stability; since #32 (R2) every non-analysis lap is (slow laps too). Not resolved by R3.
- R20 F2 -- PRODUCTION.md 18 (:1232-1234: "a mismatch means no cache", bump rule for edits "outside Settings"), settings_view.py:805 ("the snapshot comparison covers everything else"), outing_form.py:51-53 ("safety net only"): imply Settings edits are covered by the cache identities; R2 shows the Section-2 fields are not.
- R21 F2, new -- nis_gate.py:4-5: header names diagnostics/inspect_nis_tyre_mismatch_gate.py as the prototype; the script was deleted in R3.
- R22 F2, new -- PRODUCTION.md 3 (:279-280): role "(2) the per-sample replacement whenever the EKF diverges" -- since 2026-10-07 the kinematic beta never enters the EKF output (5.1 says so); 3 contradicts 5.1.
- R23 F2, new -- tyre_fit_auto.py:527: the fit's sign check picks racing corners by speed_class, set at parse time from the Settings corner speed boundaries, so the fit status depends on a Settings field no identity or warning covers (R2 at the fit-status level); PRODUCTION.md 4 states the rule without the dependency. STATUS DOCUMENTED DEPENDENCY 2026-10-17 (R5): covered by R2 (speed classes recomputed from the current boundaries before the fit; the boundaries are in the cache identity).
- R24 F3, new -- core/thesis_data.py:14-45: the thesis data reader has no identity or schema check (reports schema_version, never refuses); a thesis figure can be drawn from a stale sidecar (R3 family; the local sidecars are schema 8 and 14). STATUS FIXED 2026-10-17 (R5): core/thesis_data.read_sidecar refuses a stale sidecar (schema, format, sideslip source, Settings estimator inputs; the outing-dependent fields against an optional expected identity) with StaleSidecarError.
- R25 F3, new -- corner_trace_dialog.py:836-859, core/trace_display.py:100-130: the cross-lap p10 / p90 envelopes (a displayed statistic in the default corner view) pool the full CS arrays (graded samples included) and the stability field at kerb positions; kerb exclusion governs the drawn samples, not the envelope lines. STATUS DOCUMENTED LIMITATION 2026-10-17 (R5): display only -- cross_lap_envelope and the yaw-pooling envelope are drawn by the trace dialog only; no module reads them, no verdict, severity or recommendation.

- R26 F3, added 2026-10-17 (spike trace), STATUS see end of line -- stability_analysis.py:930-931 (Q4): the linear reference has no bound on its value; a clean low-excitation window can set a reference far below the session level (JST 29 rear 8595 N/rad = 0.05 x median); verdict-read; inherited from the chair's calculate_cornering_stiffness_ratio (docs/literature/cornering_stiffness_estimator.py, no bound there either). STATUS FIXED 2026-10-17 (R5): band rule -- a window may set the reference only inside tyre_fit_auto.stiffness_min_npr / stiffness_max_npr; outside, the previous reference is held. Remainder (references between the floor and ~0.5 x median) documented, thesis_notes R5 entry.
- R27 F1, added 2026-10-17 (R5), STATUS FIXED 2026-10-17 by the R26 band rule -- stability_analysis.py estimate_cornering_stiffness: a clean window could set a zero or negative reference; production then left CS NaN without a no-reference flag (set events <= 0: JST 29 0 / 29, NBR 10 / 0, PR 361 / 306; finite CS regained by the band rule PR +361 / +306).

First-pass mapping: R1-R20 as in the work order (R15-R20 = the six claim mismatches, R18 partly resolved by R3). Deviations from the recorded numbers, with causes:
- None in value. Every recorded number reproduced once the first pass's population definitions were identified: R1 and R4 count block membership without the count gate (with the gate: R1 15 / 9, 23 / 21, 70 / 46; R4 PR rear 409); R5 stability is the regression population, not the verdict-read set; R6 is the base mask, not the speed-windowed fit population.
- R11: the first pass recorded "0 cases"; 0 holds for verdict-read samples, but finite-CS windows straddling a lap boundary exist (NBR 2 / 2, PR 1 / 1), unread.
- R5 CS shares are 1.94-10.31 % (recorded "2-10 %", rounding). STATUS DOCUMENTED LIMITATION 2026-10-17 (R5), thesis_notes R5 entry.
