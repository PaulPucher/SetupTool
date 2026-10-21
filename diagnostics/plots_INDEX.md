# diagnostics/plots/ index

One line per plot folder of diagnostics/plots/: the arc it belongs to, the
script that writes it, and the thesis_notes.md entry that reports it. The
plots folder is gitignored; this index is committed and updated at every
close-out (author ruling 2026-10-02). Created 2026-10-02.

CODE LEGEND (standing rule 2026-10-02: every render carries a plain-language
title; this index explains the internal codes that appear in file names).

Session keys (all folders):
- dubai -- Dubai, sample session (logger 147, different tyres; least weight).
- lmn_mla -- Le Mans, driver MLA (logger 147, different tyres; banned from scoring).
- v3 -- Paul Ricard, driver MLA, session 7 outing 25 (the "v3" export).
- pr_us_v1 -- Paul Ricard, re-export of the v3 outing (duplicate, not pooled).
- pr_us_v2 -- Paul Ricard, driver MLA, the "US solved" outing.
- nbr_jst_problem -- Nurburgring GP, driver JST, outing 21 ("A", before the rake + springs change).
- nbr_jst_solution -- Nurburgring GP, driver JST, outing 29 ("B", after the change); "JST 29" in the records.
- nbr_qp_cdr / nbr_qp_mla -- Nurburgring GP qualifying practice, drivers CDR / MLA.
- nbr_o<NNN>_<drv> -- Nurburgring GP, outing NNN, driver code drv (the 12 long sessions added 2026-10-01).

seed_sweep/ (Q-SEED, how the kinematic sideslip seed is built):
- S0 -- production seed: integrate a_y/v - r, then 0.05 Hz high-pass (washout).
- S1a / S1b -- yaw-gyro bias and lateral-accelerometer offset removed BEFORE
  integration; a = offset from the straight-line median, b = offset from the
  in-corner referee intercept.
- S2 -- segment integration: beta held at 0 on straights, integrated freely
  inside each corner segment, linear correction so it returns to 0 at the exit.
- S3a / S3b -- S1a / S1b combined with S2.
- S4w_0.025 / S4w_0.1 -- S0 with the washout at 0.025 Hz (slower) / 0.1 Hz (faster).
- S4l_1 / S4l_4 -- S0 with the slip-angle and force low-pass at 1 Hz / 4 Hz instead of 2 Hz.
- File types: beta_ = sideslip angle along the lap; cloud_ = tyre-fit
  population (slip angle vs lateral force) with fitted curves; cs_ = cornering
  stiffness ratio after the full fit / EKF chain; M1 = force spread near zero
  slip angle, M3 = lap-to-lap scatter of sideslip at the same track position.

raw_channels/ (Q-RAW): psd_ = frequency content of the inertial channels;
sclu = sensor-cluster bus signals, log = logger accelerometers, ADR2 = third
accelerometer set (10 Hz), MRR = radar unit's copy of yaw rate.

last_push/ (Q-PUSH "THE LAST PUSH", final estimator-side package; script
inspect_last_push.py; thesis_notes "Q-PUSH 'THE LAST PUSH' ..." 2026-10-02):
- p1_ -- steering-derived steady-state sideslip (Rajamani Eq. 8.24) and the
  understeer gradient. p1_traces_ = cross-lap median beta: steady-state from
  steering vs kinematic seed vs EKF, top panel Level-1 stiffness (12 per rad
  x load), bottom Level-2 (production rear origin slope, circular).
  p1_s0_ = fresh production seed; p1_s2_ = segment integration (straights
  pinned to 0); p1_anch_L1 / L2 _k1.0 / _k0.5 = S2 plus mid-corner anchors
  to beta_ss with Level-1 / Level-2 stiffness at weight 1 / 0.5.
- p2_ -- despike = Hampel outlier replacement on a_y, yaw rate, steering
  before the seed; robust_ = least squares vs Huber vs trimmed tyre fits.
- p3_ -- fit population: fast = fastest half of the analysis laps; fast101 =
  laps within 101 % of the best; fast_huber = fast + Huber loss;
  fast_huber_env = fast + Huber + upper envelope (90 % quantile per 0.2 deg
  bin, the Q2d construction).
- p4_ -- spectrum-matched input filters: p4a = filters, 2 Hz slip/force
  low-pass kept; p4b = filters, slip/force low-pass at the matched
  accelerometer cutoff; p4c = p4a + accelerometer lag aligned to the cluster;
  spectra_ = the PSDs and chosen cutoffs.
- p5_ -- EKF process-noise (Q) sweep through the full production chain:
  cs_ = CS traces (kerb gaps bridged dashed), beta_ = EKF beta, summary_ =
  chatter and gate health per Q row.
- p6_ -- stacked chains: qualified = only elements that passed their gate;
  everything_on = best row of every part regardless of the gate.
- esc_ -- all-18 escalation of the gate-passing elements (Huber, trimmed).
- File types: _beta_ = sideslip along the lap; _cloud_ = tyre-fit population
  with fitted curves; _cs_ = CS ratio after the full chain; JSON per part
  and session; run_*.txt logs.

stiffness_cs/ (Q-STIFF-CS, stiffness identification + CS readability; script
inspect_stiffness_cs.py; thesis_notes "Q-STIFF-CS: ..." 2026-10-03):
- p1a_yawgain_ -- understeer gradient from the steady-state yaw-rate gain per
  speed band vs the steering-balance value K2.
- p1b_bode_ -- yaw-rate response to steering on sub-limit driving: gain,
  phase, coherence per speed band (model lines only where a fit ran).
- p2_compare_<key>_<variant> -- production CS vs one readability variant,
  front and rear: win_<T>s_<S>rad = CS window at least T seconds and S rad
  slip span (production 0.1 s / 0.02 rad); ingr_<F>Hz = slip-angle and force
  low-pass F Hz instead of 2 Hz.
- p2_phase_<key>_production / _combination -- CS drawn as per-phase medians
  (braking / turn-in / apex / exit / late exit) with cross-lap band;
  combination = the same on the best smoothed-ingredient setting.
- p2_band_ -- per-corner cross-lap median and interquartile band of CS.
- p2_decomposition_ -- position-locked signal vs lap-to-lap residual variance
  per variant.
- spike_ -- spike survival per variant: counts of CS dips below -1 / -2 (all and
  non-recurring) and the depth kept at production's five deepest dips (o =
  recurring, known-real site; x = not recurring).
- JSON per part and session, gate.json, run_*.txt logs.
- Part 0's re-rendered figure lives in last_push/ (p1_traces_<key>.png).

chair_audit/ (Q-CHAIR-AUDIT, chair pipeline audit + CS foundation; script
inspect_chair_audit.py; thesis_notes "Q-CHAIR-AUDIT: ..." 2026-10-03):
- vgrade_ -- production CS with orange "insufficient signal" marks (window
  slip span below 3x the straight-line noise floor); dotted lines =
  position-locked dip sites.
- vadapt_compare_ -- production CS vs adaptive windows (grow until slip and
  force variation exceed 3x their noise floors, cap 53 m).
- curvebound_ -- CS at the deepest dips (each lap) vs the deepest value the
  session's own fitted tyre curve can produce.
- fy_witness_ -- lateral force and its two terms at the deepest dip, with the
  damper left-minus-right load as a plausibility witness.
- partE_welch4s_bode_ -- post-hoc row: yaw-rate-per-steer response with 4 s
  spectral windows.
- partF_tyre_model_ -- our one-fit tyre curve vs the chair's four speed-band
  fits on the same population (chair smoothing and load-weighted slip omitted:
  not reproducible from the copy); 'bound +nan' = curve has no downslope.
- audit_<key>.json, partE_welch4s_<key>.json, partF_<key>.json, run_<key>.txt,
  run_partF_<key>.txt.

pairing/ (Q-PAIRING, what the position-locked deep CS dips are; script
inspect_pairing.py; thesis_notes "Q-PAIRING: ..." 2026-10-03):
- lags_ -- timing between slip angle and lateral force: every corner
  (histogram) vs each lap at the position-locked dip sites (lines).
- site_kerb_ / site_nokerb_ -- the deepest kerb-disturbed site and the
  deepest site without kerb disturbance: slip angle (EKF and kinematic),
  lateral force, CS ratio with the curve bound, vertical acceleration with
  the kerb mask; all laps.
- synth_lag_ -- CS on clean corner stretches with a known delay imposed
  between slip angle and force (lag-vs-depth curve) vs the median dip depth.
- Verdict codes in the JSON: A = slip-side error, B = strike-disturbed
  channels, C = slip-force timing mismatch.
- sites_<key>.json, synth_<key>.json, run_*.txt.

wheelload/ (Q-WHEELLOAD, per-corner wheel loads + damper-gauge decode check;
script inspect_wheelload.py; thesis_notes "Q-WHEELLOAD: ..." 2026-10-03):
- loads_<key> -- wheel load per corner (FL / FR / RL / RR, never summed),
  every analysis lap, static corner weight dashed; NBR files with the Q2
  gauge offset removed (label L4-rel), other files as the chain gives them.
- fueldrift_<key> -- within-run change of the straight-line wheel load per
  corner vs the change fuel burn alone would cause.
- offsets_crossfile -- the NBR gauge offset per corner across the 16 outings
  in outing order, with Q2's uncertainty.
- offsets_vs_rideheight -- the NBR gauge offset per corner vs that corner's
  ride height (setup-reference test).
- part0.json (decode check), file_<key>.json, summary.json, run_*.txt.

shared_offset/ (Q-SHARED-OFFSET, one shared damper-gauge offset per corner
across the 16 NBR files; script inspect_shared_offset.py; thesis_notes
"Q-SHARED-OFFSET: ..." 2026-10-03):
- shared_offsets -- per-file offsets (one aero term per corner) in outing
  order vs the shared offset (solid) and the shared offset with a season
  trend (dashed); band = the 0.7 kN load bar in gauge units.
- recovered_<role>_<key> -- wheel load per corner recovered with the shared
  offset vs the production static fallback model; roles: earliest outing,
  median-residual file, latest outing, worst file.
- fit.json, run_*.txt; cache/ holds the per-file burn traces.

v4_windows/ (Q-V4-WINDOWS + Q-CHANNEL-INVENTORY; script inspect_v4_windows.py;
thesis_notes "Q-V4-WINDOWS: ..." and "Q-CHANNEL-INVENTORY: ..." 2026-10-03):
- window_<key> -- the standing windows behind a failed static check (V4):
  speed and pit limiter, gauge offset error per corner vs the file's rolling
  value (700 N band), suspension travel, brake pressure; +/-20 s context.
- windows.json -- per-window evidence E1-E7 and the pre-registered exit.
- inventory/inv_<key>.json -- every channel of one file: unit, rate,
  liveness, consumed, relevance, raw-vs-processed pair test.
- inventory/families.txt / .json -- the channel list per file family
  (identical channel sets) as a plain list.
- run_*.txt -- run logs.

recover_rr/ (Q-RECOVER-2, NBR loads recovered through the raw-voltage
reference; script inspect_recover_rr.py; thesis_notes "Q-RECOVER-2: ..."
2026-10-03):
- voltage_reference -- the gauge offset per corner over the 16 NBR outings
  from three routes: the car at rest (config weights), the rolling fit, and
  the raw amplifier voltage referenced to the same car's Paul Ricard zero.
- gap_correction -- the gap between the offset at rest and the shared
  rolling offset, per corner, with the characterised correction and the
  700 N bar around it.
- drift_patterns -- the rear-right rolling offset as a corner weight over
  the season, split into weight patterns (common, axle, diagonal,
  left-right).
- recover.json, run.txt, run_render.txt.

Other folders: legend to be added at each folder's next close-out (their
codes are defined in the scripts' headers and the thesis_notes entries
named below).

- chair_audit/ -- PHASE 1 Q-CHAIR-AUDIT, chair pipeline audit + CS foundation (inspect_chair_audit.py) -- "Q-CHAIR-AUDIT: chair pipeline audit + CS foundation" (2026-10-03); legend above.
- cs_decision_table/ -- CS diagnostic arc, A0 / A / B decision table (inspect_cs_decision_table.py) -- "Three-way decision table: A0 / A / B" (2026-09-29), render addendum.
- cs_kerb_edge/ -- kerb-edge question, CS minima at kerb-mask edges (inspect_cs_kerb_edge.py) -- "CS minima at kerb-mask edges, eight sessions" (2026-09-30).
- cs_kerb_window/ -- kerb-edge question, window-reach check (inspect_cs_kerb_window.py) -- "Window-reach check: CS windows across masked kerb gaps" (2026-10-01).
- cs_readout_evidence/ -- CS diagnostic arc, read-out evidence and sawtooth repair side-by-side (inspect_cs_readout_evidence.py) -- read-out evidence entries of 2026-09-29 ("Sawtooth repair side-by-side render").
- cs_upstream/ -- experiment (b), upstream-insertion CS filter sweep (inspect_cs_upstream_sweep.py) -- "(b) Part 1: upstream insertion-point sweep" to "(b) Part 3" (2026-09-30).
- cs_value_lens/ -- CS diagnostic arc, value lens traces (inspect_cs_value_lens.py) -- "CS_ratio value lens: smoothness, reproduction, demand bands, A vs B, fold/loop" (2026-09-29).
- damper_offset_q2/ -- PHASE 1 Q2, NBR damper-offset feasibility (inspect_damper_offset_q2.py) -- "Q2 run: NBR damper offset NOT FEASIBLE per outing ..." (2026-10-02).
- e_stage0/ -- experiment (e) Stage 0 and Q1, rear force-demand occupancy (inspect_e_stage0.py) -- "(e) Stage 0: rear-axle force-demand occupancy ..." (2026-10-01) and "Q1: labelled Dubai P2 read" (2026-10-02).
- event_coincidence/ -- experiment (f), event coincidence (inspect_event_coincidence.py) -- "(f) Part 1: event lists, three channels" to "(f) Part 5" (2026-09-30).
- fuel_q2b/ -- PHASE 1 Q2b, fuel-channel census (inspect_fuel_channels.py) -- "Q2b: fuel-channel content check ..." (2026-10-02).
- g2_anchor_validation/ -- PHASE 1 Q3 extended, GPS corner-anchor validation (inspect_g2_anchor_validation.py) -- "G2 GPS corner anchor: repair 2, Gate A passed, validation FAILED ..." (2026-10-02).
- imu_offsets/ -- experiments (a), (a2), roll closure and tilt checks (inspect_imu_offsets.py) -- "IMU offsets, Part 1" to "Part 5", "(a2) Part 1" to "Part 4", "Roll closure check ...", "Sensor tilt vs static attitude ..." (2026-09-30).
- last_push/ -- PHASE 1 Q-PUSH "THE LAST PUSH", P1-P6 + gate + all-18 escalation (inspect_last_push.py) -- "Q-PUSH 'THE LAST PUSH': final estimator-side package" (2026-10-02); legend above. CLOSED 2026-10-03; p1_traces_<key>.png are the figures of the section-1 exhibit "The car's measured balance ...".
- kerb_mask_variants/ -- kerb-edge question, kerb-detection variants (inspect_kerb_mask_variants.py) -- "Kerb-detection variants, Part 1" to "Part 4" (2026-10-01).
- fit_ma_calibration/ -- Q-CAL-3 calibration pass, the chair's 0.15 s moving average on the tyre-fit inputs vs production's 2 Hz inputs (inspect_fit_ma_calibration.py) -- "Q-CAL-3 calibration pass: ..." (2026-10-03). fit_<key>.png = fit population and both fitted curves per axle; beta_<key>.png = EKF sideslip on the median-time lap, production fit vs moving-average fit; metrics.png = the scored metrics per session and axle, production vs moving average (x labels fixed 2026-10-03: re-rendered from results.json only, labels now "track, outing N / axle"); summary.txt; results.json; run_log.txt.
- combined_loads_feasibility/ -- combined-loads feasibility part (c), in-corner CS binned by measured axle load, 16 NBR files, no fit, no verdict (inspect_combined_loads_feasibility.py) -- "Combined-loads feasibility (c): ..." (2026-10-04). cs_vs_fz_front_axle.png / cs_vs_fz_rear_axle.png = pooled bin median (line), bin 25th-75th percentile (grey bar), per-file bin medians (dots), samples and files per bin; summary.txt = per-file coverage and every bin with counts; results.json; cache/<key>.npz = per-file CS / Fz samples; run_log.txt. CS = cornering-stiffness ratio.
- fit_decisive/ -- Q-FIT-DECISIVE, tyre-fit population levers vs production: speed window, smear zone removed, large-Fx tails removed (inspect_fit_decisive.py) -- "Q-FIT-DECISIVE: ..." (2026-10-04). fit_<key>.png = fit population and the curve of every variant per axle, origin slopes in the legend, derived bounds in the title; metrics.png = scored metrics per session and axle, one colour per variant; summary.txt = bounds, per-variant metrics, verdicts, shipping rule; results.json; run_log.txt. Codes in the text files: P production fit, SW speed window, Z smear zone removed, LX large-Fx tails removed. Mode lt (2026-10-04): fit_lt_<key>.png, metrics_lt.png, summary_lt.txt, results_lt.json, run_log_lt.txt -- LT = traction tail removed (a_x above p95), heavy braking kept.
- fit_k2_calibration/ -- Q-CAL-3 K2-constrained calibration pass, tyre fit with the front/rear balance pinned to the measured understeer gradient vs the production fit (inspect_fit_k2_calibration.py) -- "Q-CAL-3 K2-constrained calibration pass: ..." (2026-10-04). fit_<key>.png = fit population and both curves per axle, origin slopes in the legend; beta_<key>.png = EKF sideslip on the median-time lap, production vs balance-pinned fit; metrics.png = scored metrics per session and axle; summary.txt; results.json; run_log.txt. No codes beyond the session keys (P = production fit, K = balance-pinned fit in the text files).
- threshold_rederivation/ -- post-levers threshold re-derivation diagnostic, proposals only (inspect_threshold_rederivation.py) -- "Threshold re-derivation diagnostic on the final estimator: proposals only" (2026-10-04). corner_population_thresholds.png = classifier input per corner (worst phase, worst lap) per session, front and rear, current thresholds dashed vs re-derived solid; summary.txt = pools, per-session values, percentiles encoded, JST 29 front tail check; results.json; cache/<key>.pkl = per-session corner populations; counts.txt / counts.json = corner counts per session at STRONG p10 / p15 and MODERATE p30 / p40 (direction ruling 2, 2026-10-04); run_log.txt. CSF / CSR = front / rear cornering-stiffness ratio thresholds; STRONG / MODERATE = verdict tiers.
- threshold_rederivation/ (2026-10-20 additions, inspect_threshold_rederivation_l.py) -- "Threshold re-derivation on the new default (literature-bounded fit), proposal only". rederivation_l_20261020.txt / .json = pool composition (14 development sessions, Dubai excluded), current vs proposed STRONG / MODERATE per axle, sensitivity without outings 59 and 70, limit-event curve floors of the literature-bounded curves, and the floor-event census (CS ratio below the -0.53 pure-lateral floor counted as events, not samples; clean = neither kerb- nor observer-reset-affected); cache_l_20261020/<key>.pkl = per-session corner values and floor events. No figure.
- observer_robustness/ -- robustness of the default path (literature-bounded fit) to the observer noise, report only (inspect_observer_robustness.py) -- "Robustness check of the observer noise" (2026-10-20). robustness.txt / .json = per variant against the baseline, per session and axle: corner verdict class changes (strong / moderate / none / no data, with direction), floor events appearing / disappearing, gate / status / fallback, hard-bound re-entries, chosen R_ay; pooled flip rate. Variant codes in cache/<session key>__<variant>.pkl: baseline = Q upper bound with measured R floors; q_x0.1 / q_x0.3 = both process-noise variances times 0.1 / 0.3; qr_hand = Q_beta at the upper bound with the yaw-rate process noise at the old hand-set value; ryaw_x10 = yaw-rate measurement noise ten times the measured floor. No figure.
- closeout_l/ -- close-out checks on the default path (literature-bounded fit), report only (inspect_closeout_l.py) -- "NIS gate re-derivation and R26 check on the default path" (2026-10-20). closeout.txt / .json = per session the NIS gate health score and verdict, the recorded threshold rule's inputs (lowest score, 6 % below, one order of magnitude below), and per axle the R26 test: median set-event reference, verdict-read samples whose reference in force lies below half that median, their references, hold and most negative CS, and the phase medians that move without them; cache/<session key>.pkl. R26 = the registry finding "linear reference far below the session level". No figure.
- r26_fix_l/ -- the R26 source fix on the default path, before / after (inspect_r26_fix_l.py) -- "Relative reference plausibility: effect on the development set" (2026-10-20). r26_fix.txt / .json = per session and axle the band candidates rejected low / high by the [0.5, 2] x median gate, the R26 test count before -> after, phase medians moved, the deepest corner value before -> after, no-reference counts; cache/<session key>.pkl. threshold_rederivation/ gains rederivation_l_20261020b.* and cache_l_20261020b/ (the re-derivation after the fix; tag b = after the R26 fix). No figure.
- ce_sensitivity_l/ -- shape sensitivity of the literature-bounded fit, report only (inspect_ce_sensitivity_l.py) -- "C / E sensitivity of the literature-bounded fit" (2026-10-21). ce_sensitivity.txt / .json = per shape variant (C = Magic Formula shape factor, E = curvature factor) and session: fitted normalised stiffness K, residual RMS, plausibility zone, status, hard-bound re-entries, peak slip angle inside 0-15 deg, and against the shipped C 1.0 / E -1 the corner-value shift and verdict-class changes; cache/<session key>__C<c>_E<e>.pkl. observer_robustness/ gains robustness_20261021.* and cache_20261021/ (the re-run on the final chain with the R26 reference gate). No figure.
- utilisation/ -- Q-UTIL, utilisation measures beside the cornering stiffness ratio on the default path, report only (inspect_utilisation.py) -- PLAN.md STATUS NEXT 1 (2026-10-21). nbr29_lap<N>_cs_secant_utilisation_front_rear.png = NBR outing 29, the fastest analysis lap: cornering stiffness ratio, secant ratio (lateral force over axle load divided by slip angle times the fitted normalised stiffness; 1 = linear) and lateral utilisation (lateral force over axle load divided by the demonstrated friction utilisation), front and rear, over lap distance, corner phases shaded, kerb-affected samples not drawn. utilisation.txt = per session (NBR 29, 54, 10, Le Castellet 25) coverage per phase, apex axle leader per corner, distributions, rank correlations, NBR 54 corner 3 events, agreement with the verdict; cache/<session key>.pkl. Codes in the text: u = lateral utilisation, u_comb = combined (lateral and longitudinal) utilisation, s = secant ratio, mu_eff = vehicle-level effective friction, C<n> = corner n.
- phase_segmentation/ (text only) -- our corner phases against the chair's acceleration-direction phases, and the throttle channel census behind the empty braking phases (inspect_phase_segmentation.py) -- thesis_notes "Phase segmentation against the chair basis" (2026-10-21). phase_segmentation.txt = per session the throttle channel (unit, range, p99, quality grade, whether it reaches the lift-off threshold before turn-in) and, for NBR 29 and NBR 10, per corner the phase lengths of both schemes and the boundary shifts. Codes: rCosPhi = longitudinal over total acceleration; chair phases Braking / Entry / Mid / Exit; ours brake / turn-in / apex region / exit_4 (apex to half steering unwind) / exit_5 (to corner end); C<n> = corner n.
- washout_final/ (text only) -- the cost of the sideslip washout on the shipped literature-bounded tyre fit: per session and axle, rank correlation of the corner-by-corner tyre-curve residual with corner duration and with time since the last straight (inspect_washout_final.py) -- thesis_notes "Yaw-stability median, washout scope, Fx drag" (2026-10-21). washout_final.txt = the numbers; run.log; cache/<session key>.pkl. Codes: rho = Spearman rank correlation, n = corner instances.
- steering_fix_impact/ -- Step 1-fix, steering conversion faulty lookup vs constant ratio 15.7 vs corrected wheel-angle map, Tier 1 (inspect_steering_fix_impact.py) -- "Steering-fix impact (d) ...", "Beta spikes between faulty and corrected steering ..." (2026-10-04). summary.txt / results.json = per path fits, beta, alpha, CS shares, rule lines (the (ii) -> (iii) lines are VOID: stale 15.7 caches); spikes.txt = per-event beta-difference list; cache/ = faulty-path chains; run_log.txt. No renders (author: skipped). Codes: faulty = total angle divided by the local ratio column, const = constant 15.7, fixed = corrected map.
- speed_bands/ -- Step 2, our speed distribution against the chair's four speed bands (inspect_speed_bands.py) -- "Step 2: our speed distribution against the chair's four speed bands" (2026-10-04). speed_bands.png = per Tier-1 session the speed histogram with the candidate windows and the chair band centres, and the slip-angle magnitude per speed bin; summary.txt; results.json. Codes: SW = our in-corner p1-p99 window (shipped), CU = chair band union transplanted, BI = band-informed construction on our corner speeds.
- pool_fits/ -- Step 3 pooled fits, Pool T (same tyre spec, 18 files) and Pool D (driver JST, 7 files) (inspect_pool_fits.py) -- "Pooled fits (Step 3) ..." (2026-10-05). curves_poolT_front/rear.png, curves_poolD_front/rear.png = pooled curve over each member's own production curve, fit population as faint cloud; cs_lap_<key>.png = Tier-1 production cornering stiffness ratio front / rear over the median-time lap, kerb strikes shaded, near-strike samples greyed with the caption sentence (re-rendered 2026-10-05, kerb display package, mode csplots); cs_hist_<key>.png = Tier-1 in-corner histograms (log counts, negative tail visible), near-strike samples as a stacked grey segment, shares in the titles; summary.txt / results.json = pooled parameters, per-member metrics, verdicts; pool_fits.json; rebuild_log.txt; eval/ = per-member pooled-curve chains; run_log.txt. Production state = corrected steering + speed window. Codes: P = member's own production fit, pooled = the pool curve.
- iteration/ -- Step 4 iteration experiment, tyre curve refitted on the refined slip angles, starting from the session's own fit and from the pooled curve, Tier 1 (inspect_iteration.py) -- "Iteration experiment (Step 4): ..." (2026-10-05). curves_<key>_front/rear.png = tyre curve after each refit, both starting points side by side; sideslip_<key>.png = estimated sideslip on the median-time lap, production vs each starting point's final refit; summary.txt / results.json = steps per round, stops, frame metrics, verdicts; rounds_<key>.json; rounds/<key>_<start>_r<N>.npz = per-round beta, CS, slip angles; run_log.txt. File-name codes: own = starting from the session's own fit, pool = starting from the pooled curve, rN = after refit N.
- mu_trace/ -- Q-MU, lateral force over measured vertical load (Fy / Fz) per axle inside corners, 16 Nurburgring outings (inspect_mu_trace.py) -- "Q-MU: ..." (2026-10-05). mu_lap_front/rear_nbr_jst_solution.png = JST outing 29, Fy / Fz over the median-time lap, inside corners only, kerb strikes shaded, samples within 0.2 s of a masked strike greyed with the caption sentence; mu_hist_front/rear_nbr16.png = pooled in-corner histograms, clean + grey stacked, clean 50th / 90th / 99th percentile lines; report.txt = distributions, sites, verdict classes, phases; result_<key>.json, arrays_<key>.npz, trace_nbr_jst_solution.npz; run_log.txt. Codes: mu = Fy / Fz of one axle (estimated lateral force over measured load); D = front minus rear; R = front over rear; R_load = the part of R the static force split and the measured load distribution give; R_yaw = the yaw-acceleration remainder.
- mass/ -- Q-MASS step 1, measured car mass per outing from the damper loads, 16 Nurburgring + 2 Paul Ricard files (inspect_mass.py) -- "Q-MASS step 1: ..." (2026-10-05). step1_<key>.json = mass, aero coefficient, front mass share, first / second half masses, fuel burn; report_step1.txt = per-file lines and the pass / fail verdict; run_log.txt. No plots. Codes: delta = first-half minus second-half mass.
- k2_line/ -- understeer gradient per session on the corrected steering chain, and the verdict-card mock (inspect_k2_line.py, deleted 2026-10-17 with the K readout; folder kept as record) -- "Understeer gradient on the corrected chain ..." (2026-10-05). mock_verdict_card_nbr_jst_solution.png = Nurburgring JST outing 29 verdict card with the added measured-balance line (mock, not app code); k2_<key>.json, report.txt, run_log.txt. Codes: K2 = understeer gradient measured from steering (deg per g).
- spike_audit/ -- every deep cornering-stiffness-ratio dip (below -0.5) of Nurburgring JST outing 29 named one by one (inspect_spike_audit.py) -- "Q-SPIKE-AUDIT ..." (2026-10-05). overview_f.png / overview_r.png = front / rear ratio in corners, all analysis laps, each dip marked with its cause letter, kerb-affected samples greyed; deep_<rank>_<id>.png = the five deepest dips zoomed (ratio, slip angle, axle lateral force, vertical acceleration, damper velocity per wheel, kerb mask), shaded = what the estimator saw; report.txt, events.json, arrays.npz; repass_report.txt / repass.json = the specificity re-pass labels (2026-10-05). repass2_report.txt / repass2.json = the FINAL frozen ledger, kerb-missed criterion tightened to the union-5 % level (2026-10-05). Codes: F / R = front / rear dip number; a = kerb detected, b = kerb missed, c = estimator artefact, d = car-on-track event, e = unexplained; in the re-pass, r = no specific mechanism shown.
- curve_floor/ -- the fitted tyre curve's own deepest cornering stiffness ratio per pool session and axle vs the classification thresholds (inspect_curve_floor.py) -- "Curve floor vs thresholds ..." (2026-10-05). report.txt, floors.json; no plots. Codes: floor = minimum of local slope over origin slope on the visited slip range (0 to the 99th percentile).
- plots_metrology_20261005/ (diagnostics/ level) -- cs_margin re-measure, +/-1 % verdict-sensitivity sweep on the current chain, steering input as a wheel-angle-column perturbation (inspect_metrology_phase1_sensitivity.py / _analysis.py) -- "cs_margin re-measure ..." (2026-10-05). dubai_margin_distribution.png, v3_margin_distribution.png = cornering stiffness ratio per corner-phase against the thresholds, flips marked; raw per-run results in diagnostics/results_metrology_20261005/.
- kerb_cs_movement/ -- Q-CAL item 1, how the kerb-mask repair moved CS (inspect_kerb_cs_movement.py) -- "Q-CAL-1 CS-movement check: ..." (2026-10-03). cs_distribution_<key>.png = CS histogram before (+/-50 ms mask) vs after (+/-100 ms), front and rear, log counts with the negative tail visible; site_dips_before_after.png = the 22 position-locked sites, median dip before vs after (diamonds = sites carrying the near-strike mark); trace_pr_us_v2_front_2893m_lap52.png = CS over distance around that site, before vs after, old mask / added mask / smear zone shaded; summary.txt = percentiles, tail shares and the per-lap site list; run_log.txt.
- kerb_filter_decay/ -- Q-CAL item 1, kerb-strike smear through the 2 Hz filter past the mask edge (inspect_kerb_filter_decay.py) -- "Q-CAL-1 measurement: kerb strikes leak past the mask edge through the 2 Hz filter" (2026-10-03). filter_smear_front_axle.png / filter_smear_rear_axle.png = worst and median case per axle (rows: lateral force, slip angle, smear relative to the clean-stretch noise band; grey = kerb mask, yellow = smear zone); summary.txt = per-session and pooled decay times and CS-window shares; coincidence.txt = spike-coincidence count (ruling 5); site_check.txt = the 22 position-locked dip sites vs the 0.25 s grade zone (per site and lap: Z dip in zone, w window only, - neither); verify.txt = options 4 + 0 check on the three Tier-1 sessions (stage A = mark alone reproduces the cached CS; stage B = shipped config: per-site near_strike count and every site-lap whose dip changed under the restored +/-100 ms dilation, before -> after); decay_records.json = per-event records; run_log.txt. No codes beyond the session keys.
- nbr_zero_entry_verify/ -- Q-CAL item 2 acceptance, NBR wheel loads through the production parser with the event zero entries (inspect_nbr_zero_entry_verify.py) -- "Q-CAL-2 implemented: ..." (2026-10-03). verify.txt = per file: entry status, damper share per corner, map |r| and implied zero minus entry, V1-V4; offset reproduction per corner; exit. verify.json = the same as data; run_log.txt. No plots.
- nbr_c1_dip/ -- (f) follow-up, NBR C1 fixed-position CS dip (inspect_nbr_c1_dip.py) -- "NBR C1 fixed-position CS dip: kerb strike at the apex" (2026-09-30).
- new_laps/ -- NBR outing mapping and full-lap CS traces, shared caches (inspect_new_laps_cs_sweep.py) -- "NBR outing mapping and full-lap CS traces" (2026-09-29).
- phase1_q3/ -- PHASE 1 Q2c + Q3 + Q4, GPS census, gradient map, tailoring variants, courtroom (inspect_phase1_q3.py) -- "Q2c + Q3 combined run ..." (2026-10-02).
- pairing/ -- PHASE 1 Q-PAIRING, slip side / strike / timing at the position-locked CS dips (inspect_pairing.py) -- "Q-PAIRING: what the position-locked deep CS dips are ..." (2026-10-03); legend above.
- rake_ab_anchor/ -- PHASE 1 Q3 ruling 3, NBR A -> B anchor examination (inspect_rake_ab_anchor.py) -- "NBR A -> B anchor examination, reframed ..." (2026-10-02).
- raw_channels/ -- PHASE 1 Q-RAW, raw-vs-processed inertial channel check for the team case file (inspect_raw_channels.py) -- "Q-RAW: raw-channel case file ..." (2026-10-02); note docs/team_questions_raw_channels.md.
- rear_curve_bias/ -- PHASE 1 Q2d, rear-curve bias diagnostic and the kinematic-seed exhibit (inspect_rear_curve_bias.py) -- "Q2d: rear-curve bias diagnostic ..." (2026-10-02).
- seed_sweep/ -- PHASE 1 Q-SEED, seed quality sweep S0-S4 (inspect_seed_sweep.py) -- "Q-SEED: seed quality sweep ..." (2026-10-02).
- recover_rr/ -- PHASE 1 Q-RECOVER-2, NBR wheel loads recovered via the raw-voltage reference (inspect_recover_rr.py) -- "Q-RECOVER-2: ..." (2026-10-03); legend above.
- v4_windows/ -- PHASE 1 Q-V4-WINDOWS (standing windows behind failed static checks) and Q-CHANNEL-INVENTORY (all channels, raw gauge voltages) (inspect_v4_windows.py) -- 2026-10-03; legend above.
- shared_offset/ -- PHASE 1 Q-SHARED-OFFSET, one shared NBR damper-gauge offset per corner (inspect_shared_offset.py) -- "Q-SHARED-OFFSET: ..." (2026-10-03); legend above.
- stiffness_cs/ -- PHASE 1 Q-STIFF-CS, stiffness identification + CS readability (inspect_stiffness_cs.py) -- "Q-STIFF-CS: stiffness identification + CS readability" (2026-10-03); legend above.
- tyre_prior/ -- experiment (d) step 0, pooled prior with per-session scale (inspect_tyre_prior.py) -- "(d) step 0: pooled prior with per-session scale" (2026-10-01).
- tyre_prior_d2/ -- experiment (d2), Door 1, Door 2; holds the shared data/fit caches of the 20-file pool (inspect_tyre_prior_d2.py) -- "(d2) Part 0" to "(d2) closure and Part 5 build", "Door 2 ...", Door 1 Stage 0 (2026-10-01).
- wheelload/ -- PHASE 1 Q-WHEELLOAD, per-corner wheel loads, damper-gauge decode check, setup-reference test (inspect_wheelload.py) -- "Q-WHEELLOAD: per-corner wheel-load renders + damper-gauge decode check" (2026-10-03); legend above.
- yaw_pooling/ -- experiment (c), yaw-rate cross-lap pooling (inspect_yaw_pooling.py) -- "(c) Part 1: yaw-rate cross-lap pooling ..." (2026-09-30).
- fit_simplification/ -- Q-FIT-SIMPLE, tyre-fit simplification variants through the whole chain (inspect_fit_simplification.py) -- thesis_notes "Q-FIT-SIMPLE: frozen frame" and the run entry (2026-10-18). fit_<key>_<axle>.png = sample density of the fit population (slip angle from the simple geometric estimate vs axle lateral force, log grey) with the fitted tyre curve of every variant, dotted lines at the 99 % slip-angle extent; csdist_<key>.png = cumulative distribution of the cornering stiffness ratio per variant, front and rear (corner samples on analysis laps, kerb-affected excluded); traces_<key>_<code>.png = standard CS trace set per variant (diagnostics/trace_render.py). fit_simplification_<key>.json, summary.json (beats-production rule, prediction scores), run.log. Session keys: nbr_jdr = NBR-73-010-026-JDR, pr_us_v2 = Paul Ricard MLA-SE07_US-v2, nbr_jst29 = NBR JST outing 29. Variant codes (file names only; plots carry words): P = production fit, all four Magic Formula parameters free; P13 = all four free from the chair tyre-fit start (shape factor C 1.3); E0 = curvature factor E fixed at 0, shape factor C free in 1-3; E0C19 / E0C13 = E fixed at 0, C fixed at 1.9 / 1.3; E97C13 = E fixed at 0.97, C fixed at 1.3; Pw / E0w = P / E0 with even slip-angle weighting (inverse sample density, dashed in the plots).
- fit_lit/ -- Q-FIT-LIT verification, the literature-bounded tyre fit through the whole chain (inspect_fit_lit.py) -- thesis_notes "Q-FIT-LIT: verification cycle" (2026-10-18). fitcurve_<key>_<axle>.png = slip angle from the simple geometric estimate vs lateral force divided by the measured axle load (log grey density of the fit population), with the weighted and unweighted fitted curves, dots at the curve peak, the visited slip range (1st-99th percentile) shaded; traces_<key>_L.png = standard CS trace set for the weighted fit; csdist_<key>.png = cumulative CS distribution of both fits, information only. fit_lit_<key>.json, summary.json (prediction scores, exit), run.log. Session keys as in fit_simplification/. Codes (file names and logs only): L = literature-bounded fit (shape factor 1.3 and curvature factor -1 fixed, B and friction coefficient bounded) with even slip-angle weighting; L-nw = the same unweighted. void_run/ = the first run's outputs (void, implementation defect, kept for the reproduction check). d4_reopen/ = the declared post-result reopen with the redesigned plausibility gate for this fit (inspect_fit_lit.py --reopen-d4): d4_reopen_<key>.json, traces_<key>_L.png, run.log (2026-10-18). c10/ = the same fit with shape factor 1.0 (no peak; inspect_fit_lit.py --c10, post-result ruling): curve_<key>_<axle>.png = shape factor 1.0 against the previous 1.3 curve on the sample density, visited range shaded; traces_<key>_L.png; c10_<key>.json; run.log.
- pr_anomaly/ -- Q-PR-ANOMALY, why the Paul Ricard US-v2 rear axle shows a low normalised origin slope (inspect_pr_anomaly.py) -- thesis_notes "Q-PR-ANOMALY" (2026-10-18). factors_<key>.png = per corner and lap, the tyre-curve residual (measured minus literature-bounded curve, in force / axle load, positive = more force than the curve) against six candidate factors: measured / quasi-static axle load, yaw-acceleration share of the axle force, corner duration, time since the last straight, mean longitudinal acceleration, kerb-affected share; usv2_rear_cloud_by_speed.png = US-v2 rear slip angle vs force / axle load coloured by corner speed class, with the fitted curve. pr_anomaly.json, run.log. Session keys as in fit_simplification/. Hypothesis codes in the log: H-Fz = load side, H-Fy = force side, H-alpha = slip side.
- obs_reentry/ -- Q-OBS-REENTRY, where and why the sideslip observer hits its 15 deg bound under the literature-bounded tyre fit (inspect_obs_reentry.py) -- thesis_notes "Q-OBS-REENTRY" (2026-10-18). timeline_<key>.png = re-entries over lap distance and lap, literature-bounded fit (dots) and production fit (crosses); slip_<key>.png = observer slip angle just before each re-entry on the fitted curve, front and rear, visited slip range shaded, colour = first matching cause category (slip beyond the visited range or near the curve peak / kerb strike just before / low speed or straight / none). obs_reentry.json, run.log. Session keys as in fit_simplification/. Hypothesis codes in the log: H-OBS observability, H-KERB kerb, H-LOWV low speed or straight, H-LOC position-locked corner. Top level = the run with shape factor 1.0; c13_run/ = the earlier run with shape factor 1.3.
- capacity/ -- Q-CAPACITY, measured lateral force demand against the fitted grip of the literature-bounded fit (inspect_capacity.py) -- thesis_notes "Q-CAPACITY" (2026-10-19). demand_<key>.png = distribution of lateral force / measured axle load per axle (log count) with the fitted friction coefficient and the demonstrated utilisation (99.5th percentile); events_nbr_jdr.png = measured demand in the half second before each observer hard-bound re-entry on the fitted curve, fit population in grey. capacity.json, run.log. Session keys as in fit_simplification/. Code in the log: H-CAP = re-entries where demand reaches the fitted grip.
- pr_tail/ -- Q-PR-TAIL, Paul Ricard US-v2 observer re-entries under the demonstrated-grip rule (inspect_pr_tail.py) -- thesis_notes "Q-PR-TAIL" (2026-10-19). usv2_demand_track.png = lateral force / measured axle load over lap distance, front and rear, all analysis laps overlaid, tail samples (above the 99.5th percentile) and observer hard-bound re-entries marked; usv2_events_curve.png = measured demand in the half second before each re-entry on the fitted curve. pr_tail.json, run.log. Hypothesis codes in the log: H-FORCE yaw / Fx term share, H-KERB-MISS vertical-acceleration spikes below the kerb threshold, H-LOC position-locked corners, H-TAIL-SHAPE tail heaviness.
- fit_lit/nis_reset/ (text only) -- Q-NIS-RESET, does the observer's NIS-window covariance reset change the cornering stiffness slope (inspect_nis_reset_effect.py) -- thesis_notes "Q-NIS-RESET" (2026-10-19). run.log = per session and axle the relative slope difference with and without that reset; nis_reset_<key>.json = the same numbers; run A = shipped observer, run B = same observer with the NIS-window reset switched off. run_control.log / nis_control_<key>.json = the same comparison on windows holding no reset (U) beside the reset windows (M), with sideslip differences and slip-angle spans; populations.log / populations_<key>.json = verdict cornering stiffness ratio samples per corner and phase before and after the kerb and observer-reset exclusions.
- q_phys/ (text only) -- Q-PHYS, process noise of the sideslip observer derived from the literature-bounded tyre fit's force error (inspect_q_phys.py) -- thesis_notes "Q-OBS-DERIVE" (2026-10-19). run.log = per session the tyre force residual per axle, its spread and correlation time, the resulting sideslip-rate and yaw-acceleration errors and the per-step process noise under two assumptions next to the current values; q_phys_<key>.json = the same numbers.
- cd_threshold/ (text only) -- h derivation, the alarm threshold of the change detector that resets the sideslip observer's uncertainty (derive_cd_threshold.py) -- thesis_notes "h derivation for the change-detector trigger" (2026-10-19). run.log = per derive session and threshold value the hard-bound re-entries, alarm count, mean time between alarms on the validation straights and the NIS gate, next to the shipped NIS-window trigger; results.json = the same numbers and the selection.
- cs_dips/ (text only) -- Q-DIPS, where deep negative cornering stiffness ratios come from (inspect_cs_dips.py) -- PLAN.md "Q-DIPS" (2026-10-19, report only). run.log = the Magic Formula post-peak slope floor, per session and axle the share of negative ratios with cause flags, and the dip counts under the current, the chair-style, the load-normalised and the combined estimate; results.json = the same numbers.
- q_upper/ (text only) -- Q-UPPER, the sideslip observer with its process noise at the upper bound from the tyre-force error, against the alarm threshold of the uncertainty reset (inspect_q_upper.py) -- thesis_notes "Q-UPPER" (2026-10-19). run.log = per session and trigger setting the hard-bound re-entries, alarms, mean time between alarms on straights, NIS, noise picks and all observer gates; results.json = the same numbers and the selection.
- fallback_59_70/ (text only) -- why NBR outings 59 and 70 fall back to the simple sideslip estimate under the literature-bounded tyre fit (inspect_fallback_59_70.py) -- thesis_notes "Q-UPPER" (2026-10-19). run.log = per session and axle the tyre-curve plausibility values against their limits, and the ambient temperature, wiper, wet-tyre and track-condition channels; results.json = the same numbers.
- fit_l_observer_final/ (text only) -- the sideslip observer of the literature-bounded tyre fit after its finalisation (process noise from the session's tyre-force error, no soft reset, measurement-noise floor for lateral acceleration) on all 14 analysis sessions (run_fit_l_observer_final.py) -- thesis_notes "Observer finalisation for fit L" (2026-10-19). run.log = per session status, process noise, noise picks and where they land, hard-bound re-entries, NIS, straight-line sideslip and every observer gate; results.json = the same numbers.
- cs_norm/ -- cornering stiffness ratio from the raw axle force against the load-normalised axle force (lateral force divided by the measured axle load) under the literature-bounded tyre fit (inspect_cs_norm.py) -- thesis_notes "D4-L K band at source precision, and CS on load-normalised axle force" (2026-10-20). nbr29_lap<N>_cs_ratio_raw_vs_normalised.png = NBR outing 29, the analysis lap of median lap time, ratio over lap distance for front and rear axle, both estimates, corner phases shaded grey, the -0.53 physics floor dashed; run.log / results.json = per session the reference-band exclusion, the share of negative ratios for both estimates and the lap-to-lap repetition of samples below -0.53. nbr54_corner3_six_laps_front_cs_steering_speed_fx.png = NBR outing 54, corner 3, the six analysis laps overlaid: front cornering stiffness ratio from the load-normalised force (samples below -0.53 circled), steering-wheel angle, speed and the front longitudinal force share, turn-in phase shaded (inspect_nbr54_c3.py; nbr54_c3_run.log / nbr54_c3.json = the per-lap numbers). run_band_6_30.log = the first run with the 6-30 per rad reference band.
- predates_harden.diff / .stat (files, not a folder) -- backlog-sweep diff saved for the WP-HARDEN commit split (PLAN.md STATUS, 2026-09-30).
- ../channel_census/ (beside plots/, not under it) -- Q-CHANNEL-AUDIT, full channel census of every export and .pds on disk (inspect_channel_census.py) -- "Q-CHANNEL-AUDIT ..." (2026-10-06). content_nbr_jst29.png / content_pr_v3.png: one corner-dense lap, the candidate channels against the kinematic sideslip and lateral acceleration (titles in plain words; "kinematic" = the production seed sideslip from integrating a_y / v - r).
- ../lap_census/ (beside plots/, text only) -- double-outlap lap census (inspect_lap_census.py) -- "Lap census: the double outlap is a leading pit-exit fragment ..." (2026-10-06). lim = pit limiter at lap start / end; out / in / valid = the production lap flags.
- ../output/kinematic_inputs/ (beside plots/) -- kinematic-integration input audit (inspect_kinematic_inputs.py --png) -- "Kinematic-integration input audit ..." (2026-10-07). identity_fits_<export>.png = lateral acceleration over speed vs yaw rate in quasi-steady cornering with the identity line and the three error models (constant offset; gain + offset; gain + offset + square); residuals_<export>.png = mean residual of each model in 20 equal-count yaw-rate bins; lap_offsets_<export>.png = yaw-rate and lateral-acceleration offsets per analysis lap on steering-defined straights; straights_lap<N>_<export>.png = speed and steering-wheel angle over one lap, selected straights shaded. <export> = the logger export's file name. Codes in the text report: M0 / M1 / M2 = the three error models in that order; S0 = the 2026-09-30 experiment's straight population (outside corner brackets).
  [2026-10-07, diagnostic 2 additions to ../output/kinematic_inputs/ (gitignored): the per-export files above for five more exports; washout_step_response.png = production sideslip washout response to a sideslip held 3, 6 and 10 s; long_corner_<export>.png = one long corner, production sideslip paths against raw integration of offset-corrected inputs, lateral acceleration below; offsets_by_session.png = straight-line yaw-rate and lateral-acceleration offsets, cornering gain and offset per export.]
  [2026-10-07, architecture comparison (compare_beta_architectures.py --png) in ../output/kinematic_inputs/: beta_arch_corners_<export>.png = sideslip of the four architectures on the five longest sustained corners; beta_arch_validation_<export>.png = residual sideslip at held-out straight midpoints per architecture; beta_arch_drift_<export>.png = that residual over session time with linear trends. Codes: A = production kinematic path (whole file + 0.05 Hz washout), B = offsets removed without washout, B2 = B with a 0.005 Hz washout, C = offsets removed, reset on calibration straights.]
  [2026-10-07, round 2 (compare_beta_architectures.py --png): beta_arch2_corners_<export>.png = six architectures on the five longest sustained corners; beta_arch2_since_reset_<export>.png = anchored gain-corrected sideslip at held-out straights against time since its last reset; beta_arch2_drift_<export>.png = residual on held-out straights over session time, all six, linear trends. Codes: A production kinematic, B2 offsets only + 0.005 Hz washout, D gain + offset corrected without washout, D2 = D + 0.005 Hz washout, CG gain + offset corrected with resets on calibration straights, E = the app's estimator sideslip.]
  [2026-10-07, round 3 (seed_sensitivity_fit.py --png) in ../output/kinematic_inputs/: seed_sens_corner_<export>.png = the app's estimator sideslip on the longest reference corner under the three tyre-fit seeds and both substitution routes; seed_sens_curve_front_<export>.png / seed_sens_curve_rear_<export>.png = the fitted axle tyre curves of the same five runs. Codes: seed A = production kinematic, D2 = gain + offset corrected + 0.005 Hz washout, B2 = offsets only + 0.005 Hz washout; route (a) = seed replaces the kinematic beta in the fit data only (dashed), route (b) = also in the estimator's divergence fallback and reset (solid).]
  [2026-10-08, estimator hardening (check_estimator_hardening.py --png) in ../output/kinematic_inputs/: hardening_corner_<export>.png = the app's estimator sideslip on the longest reference corner before (divergence resets write the kinematic sideslip into the estimator) and after (state kept with reset covariance, or reset to zero beyond the 15 deg bound) the divergence re-entry change.]
- ../output/estimator_hardening/ (beside plots/, gitignored) -- phase 2 of the estimator hardening (check_estimator_hardening.py) -- thesis_notes "Phase 2 step 2 ..." (2026-10-11). dubai_corner0_reentry.png = Dubai lap 1 corner 1, estimator sideslip before and after the divergence re-entry change with lateral acceleration; sign_check_corners_<export>.png = median estimator sideslip per corner before and after (sign check input); pr_pinned_noise_sign_check.png = Paul Ricard per-corner median sideslip, before / after with the swept noise / after with the noise pinned to the old choice; stability_dubai_C5_r_sensitive.png, stability_dubai_C9_both.png, stability_nbr_jst21_C14.png = estimator sideslip on all analysis laps through the phase with the lowest observed stability (crosses: covariance reset on measurement disagreement). robustness/ holds pickles only. Codes: 0.03/1, 0.1/16 = measurement-noise grid point (ay scale / yaw scale).
- plots_metrology_20261011/ -- cs_margin perturbation sweep on the corrected estimator (inspect_metrology_phase1_sensitivity.py + inspect_metrology_phase1_analysis.py with METROLOGY_RUN_TAG=20261011) -- thesis_notes "Phase 2 step 2 close-out" (2026-10-11). <session>_margin_distribution.png = baseline corner-phase CS values against the STRONG / MODERATE lines. Same layout as plots_metrology_20261005/.
