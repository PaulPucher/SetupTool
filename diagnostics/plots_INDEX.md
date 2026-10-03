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
