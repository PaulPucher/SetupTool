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
- steering_fix_impact/ -- Step 1-fix, steering conversion faulty lookup vs constant ratio 15.7 vs corrected wheel-angle map, Tier 1 (inspect_steering_fix_impact.py) -- "Steering-fix impact (d) ...", "Beta spikes between faulty and corrected steering ..." (2026-10-04). summary.txt / results.json = per path fits, beta, alpha, CS shares, rule lines (the (ii) -> (iii) lines are VOID: stale 15.7 caches); spikes.txt = per-event beta-difference list; cache/ = faulty-path chains; run_log.txt. No renders (author: skipped). Codes: faulty = total angle divided by the local ratio column, const = constant 15.7, fixed = corrected map.
- speed_bands/ -- Step 2, our speed distribution against the chair's four speed bands (inspect_speed_bands.py) -- "Step 2: our speed distribution against the chair's four speed bands" (2026-10-04). speed_bands.png = per Tier-1 session the speed histogram with the candidate windows and the chair band centres, and the slip-angle magnitude per speed bin; summary.txt; results.json. Codes: SW = our in-corner p1-p99 window (shipped), CU = chair band union transplanted, BI = band-informed construction on our corner speeds.
- pool_fits/ -- Step 3 pooled fits, Pool T (same tyre spec, 18 files) and Pool D (driver JST, 7 files) (inspect_pool_fits.py) -- "Pooled fits (Step 3) ..." (2026-10-05). curves_poolT_front/rear.png, curves_poolD_front/rear.png = pooled curve over each member's own production curve, fit population as faint cloud; cs_lap_<key>.png = Tier-1 production cornering stiffness ratio front / rear over the median-time lap, kerb strikes shaded, near-strike samples greyed with the caption sentence (re-rendered 2026-10-05, kerb display package, mode csplots); cs_hist_<key>.png = Tier-1 in-corner histograms (log counts, negative tail visible), near-strike samples as a stacked grey segment, shares in the titles; summary.txt / results.json = pooled parameters, per-member metrics, verdicts; pool_fits.json; rebuild_log.txt; eval/ = per-member pooled-curve chains; run_log.txt. Production state = corrected steering + speed window. Codes: P = member's own production fit, pooled = the pool curve.
- iteration/ -- Step 4 iteration experiment, tyre curve refitted on the refined slip angles, starting from the session's own fit and from the pooled curve, Tier 1 (inspect_iteration.py) -- "Iteration experiment (Step 4): ..." (2026-10-05). curves_<key>_front/rear.png = tyre curve after each refit, both starting points side by side; sideslip_<key>.png = estimated sideslip on the median-time lap, production vs each starting point's final refit; summary.txt / results.json = steps per round, stops, frame metrics, verdicts; rounds_<key>.json; rounds/<key>_<start>_r<N>.npz = per-round beta, CS, slip angles; run_log.txt. File-name codes: own = starting from the session's own fit, pool = starting from the pooled curve, rN = after refit N.
- mu_trace/ -- Q-MU, lateral force over measured vertical load (Fy / Fz) per axle inside corners, 16 Nurburgring outings (inspect_mu_trace.py) -- "Q-MU: ..." (2026-10-05). mu_lap_front/rear_nbr_jst_solution.png = JST outing 29, Fy / Fz over the median-time lap, inside corners only, kerb strikes shaded, samples within 0.2 s of a masked strike greyed with the caption sentence; mu_hist_front/rear_nbr16.png = pooled in-corner histograms, clean + grey stacked, clean 50th / 90th / 99th percentile lines; report.txt = distributions, sites, verdict classes, phases; result_<key>.json, arrays_<key>.npz, trace_nbr_jst_solution.npz; run_log.txt. Codes: mu = Fy / Fz of one axle (estimated lateral force over measured load); D = front minus rear; R = front over rear; R_load = the part of R the static force split and the measured load distribution give; R_yaw = the yaw-acceleration remainder.
- mass/ -- Q-MASS step 1, measured car mass per outing from the damper loads, 16 Nurburgring + 2 Paul Ricard files (inspect_mass.py) -- "Q-MASS step 1: ..." (2026-10-05). step1_<key>.json = mass, aero coefficient, front mass share, first / second half masses, fuel burn; report_step1.txt = per-file lines and the pass / fail verdict; run_log.txt. No plots. Codes: delta = first-half minus second-half mass.
- k2_line/ -- understeer gradient per session on the corrected steering chain, and the verdict-card mock (inspect_k2_line.py) -- "Understeer gradient on the corrected chain ..." (2026-10-05). mock_verdict_card_nbr_jst_solution.png = Nurburgring JST outing 29 verdict card with the added measured-balance line (mock, not app code); k2_<key>.json, report.txt, run_log.txt. Codes: K2 = understeer gradient measured from steering (deg per g).
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
- predates_harden.diff / .stat (files, not a folder) -- backlog-sweep diff saved for the WP-HARDEN commit split (PLAN.md STATUS, 2026-09-30).
- ../channel_census/ (beside plots/, not under it) -- Q-CHANNEL-AUDIT, full channel census of every export and .pds on disk (inspect_channel_census.py) -- "Q-CHANNEL-AUDIT ..." (2026-10-06). content_nbr_jst29.png / content_pr_v3.png: one corner-dense lap, the candidate channels against the kinematic sideslip and lateral acceleration (titles in plain words; "kinematic" = the production seed sideslip from integrating a_y / v - r).
- ../lap_census/ (beside plots/, text only) -- double-outlap lap census (inspect_lap_census.py) -- "Lap census: the double outlap is a leading pit-exit fragment ..." (2026-10-06). lim = pit limiter at lap start / end; out / in / valid = the production lap flags.
- ../output/kinematic_inputs/ (beside plots/) -- kinematic-integration input audit (inspect_kinematic_inputs.py --png) -- "Kinematic-integration input audit ..." (2026-10-07). identity_fits_<export>.png = lateral acceleration over speed vs yaw rate in quasi-steady cornering with the identity line and the three error models (constant offset; gain + offset; gain + offset + square); residuals_<export>.png = mean residual of each model in 20 equal-count yaw-rate bins; lap_offsets_<export>.png = yaw-rate and lateral-acceleration offsets per analysis lap on steering-defined straights; straights_lap<N>_<export>.png = speed and steering-wheel angle over one lap, selected straights shaded. <export> = the logger export's file name. Codes in the text report: M0 / M1 / M2 = the three error models in that order; S0 = the 2026-09-30 experiment's straight population (outside corner brackets).
  [2026-10-07, diagnostic 2 additions to ../output/kinematic_inputs/ (gitignored): the per-export files above for five more exports; washout_step_response.png = production sideslip washout response to a sideslip held 3, 6 and 10 s; long_corner_<export>.png = one long corner, production sideslip paths against raw integration of offset-corrected inputs, lateral acceleration below; offsets_by_session.png = straight-line yaw-rate and lateral-acceleration offsets, cornering gain and offset per export.]
  [2026-10-07, architecture comparison (compare_beta_architectures.py --png) in ../output/kinematic_inputs/: beta_arch_corners_<export>.png = sideslip of the four architectures on the five longest sustained corners; beta_arch_validation_<export>.png = residual sideslip at held-out straight midpoints per architecture; beta_arch_drift_<export>.png = that residual over session time with linear trends. Codes: A = production kinematic path (whole file + 0.05 Hz washout), B = offsets removed without washout, B2 = B with a 0.005 Hz washout, C = offsets removed, reset on calibration straights.]
  [2026-10-07, round 2 (compare_beta_architectures.py --png): beta_arch2_corners_<export>.png = six architectures on the five longest sustained corners; beta_arch2_since_reset_<export>.png = anchored gain-corrected sideslip at held-out straights against time since its last reset; beta_arch2_drift_<export>.png = residual on held-out straights over session time, all six, linear trends. Codes: A production kinematic, B2 offsets only + 0.005 Hz washout, D gain + offset corrected without washout, D2 = D + 0.005 Hz washout, CG gain + offset corrected with resets on calibration straights, E = the app's estimator sideslip.]
  [2026-10-07, round 3 (seed_sensitivity_fit.py --png) in ../output/kinematic_inputs/: seed_sens_corner_<export>.png = the app's estimator sideslip on the longest reference corner under the three tyre-fit seeds and both substitution routes; seed_sens_curve_front_<export>.png / seed_sens_curve_rear_<export>.png = the fitted axle tyre curves of the same five runs. Codes: seed A = production kinematic, D2 = gain + offset corrected + 0.005 Hz washout, B2 = offsets only + 0.005 Hz washout; route (a) = seed replaces the kinematic beta in the fit data only (dashed), route (b) = also in the estimator's divergence fallback and reset (solid).]
  [2026-10-08, estimator hardening (check_estimator_hardening.py --png) in ../output/kinematic_inputs/: hardening_corner_<export>.png = the app's estimator sideslip on the longest reference corner before (divergence resets write the kinematic sideslip into the estimator) and after (state kept with reset covariance, or reset to zero beyond the 15 deg bound) the divergence re-entry change.]
