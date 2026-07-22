# Thesis Notes — SetupTool
Raw collection for the written thesis. Not polished prose — arguments,
derivations, findings, numbers, and limitations captured as they emerge.
Lives in the project root, travels with git and every handover.

HOW TO USE:
- Claude Code: append under the matching section when a finding is
  thesis-worthy (see CLAUDE.md rule). Never delete existing entries;
  strike through with a note if superseded.
- Chat sessions: paste this file when asking for thesis-writing help.
- Each entry gets a date so the development narrative stays reconstructable.

---

## 1. Core methods and their justification

### CS_ratio (cornering stiffness ratio) — Werner MA method [2026-06]
- Slip angles from full arctan bicycle model; lateral forces from m*ay with
  static weight split (Level 1); C_alpha = windowed OLS slope of Fy vs alpha,
  blended with monotonic-section slopes, R^2-weighted.
- CS_ratio = C_alpha(current) / C_alpha(linear reference). Linear reference
  only updated when the whole window sits inside +/-0.021 rad slip.
- Interpretation: 1.0 = linear regime; ~0.5 = tyre working hard; <0.2 =
  saturated. Computed per axle -> identifies WHICH axle limits and WHEN
  (phase-resolved).
- No tyre model required — everything from logged signals. This is the
  central methodological claim: tyre-state estimation without tyre data.

### Yaw moment stability dMz/dbeta [2026-06]
- Mz_inertial = Iz * psi_ddot (yaw accel from differentiated, 5 Hz filtered
  yaw rate). Local centred 2 s OLS of Mz over [1, beta, delta_f, v, ax].
- c_beta > 0 stabilising, < 0 destabilising (Suzuka convention).
- KEY DERIVATION for thesis: yaw rate EXCLUDED from the regressor set
  because of structural multicollinearity with beta via the kinematic
  identity beta_dot = ay/v - psi_dot. Including both makes the OLS
  ill-conditioned and the coefficients uninterpretable.
- Catches a different failure mode than CS_ratio: tyre can be within grip
  while vehicle dynamics are unstable, and vice versa.

### Two-signal AND-logic for severity classification [2026-06-29]
- A corner is flagged "strong" only when CS collapse AND destabilising yaw
  coincide. Either alone = one-eyed view. Rationale: engineer needs the 2-3
  corners that are BROKEN, not every corner where the tyre works hard.
- Empirical result: single-signal thresholds flagged 47/72 corners on a
  balanced car (useless); AND-logic + distribution-tuned thresholds gave
  0 strong / 23 moderate / 49 normal — matching driver's report of a
  balanced car with mild understeer tendency.
- Threshold derivation was DATA-DRIVEN: percentile analysis of worst-phase
  CS per corner (front p10=0.049, median=0.334; rear p10=0.186,
  median=0.749) -> STRONG_CSF=0.10, STRONG_CSR=0.20, MODERATE_CSF=0.25,
  MODERATE_CSR=0.35, STAB_NEG_THRESH=-500 Nm/deg. Asymmetric front/rear
  thresholds because rear CS stays structurally higher on this car
  (57.2% rear weight) — when rear drops it means more.

### Kerb/jump exclusion [2026-06-29]
- Vertical accel (log_acc_z) deviation-from-baseline gate: |az - 1.0g| >
  1.2g, dilated +/-5 samples (0.1 s ringdown). Baseline is +1.0g:
  Cosworth/SCLU convention is z-down (gravity positive) — discovered
  empirically (initial -1g assumption flagged 100% of samples).
- Threshold tuned to flag 3.0% of moving samples on Dubai (target band
  0.5-3%, plausible for moderate kerb usage).
- Effect on results: stability valid samples 30813->29550, median
  2547->2676 Nm/deg (kerbs were biasing stability DOWN), CS_ratio means up
  ~0.006. Per-corner: one apex phase dropped from 100% to 18% valid —
  kerb transparency at exactly the right place.
- LIMITATION (Level 1): static deviation threshold; sustained aero load
  (1.5g at 250 km/h) approaches the threshold. Rate-of-change (daz/dt)
  detection is the documented upgrade path.

### Dual-criterion corner detection [2026-07-22]
- Original steering-threshold detection (25 deg entry / 15 deg exit
  hysteresis) failed in fast sections. PHYSICS: required steering angle
  scales inversely with corner radius (delta ~ L/R + understeer term), so
  a fixed degree threshold is systematically marginal in fast corners —
  bulletproof in hairpins, borderline in sweepers. Track-independent
  argument: recurs at every fast circuit.
- CONCEPTUAL argument: steering is the DRIVER INPUT channel (contains line
  variance, corrections, counter-steer); lateral G is the VEHICLE RESPONSE
  and answers the only relevant question — is the car cornering?
- Fix: ENTER on (|steer| > 25 deg) OR (|ay| > 0.6 g); EXIT on
  (|steer| < 15 deg) AND (|ay| < 0.35 g). Graceful degradation: without
  ay channel, ay term is all zeros and behaviour is bit-identical to old.
- Diagnostic evidence (case classification): lap 1 five sub-threshold
  peaks 18.7-23.0 deg where other laps hit 27-43 deg in the same 80 m —
  same physical corner, threshold-borderline. Reversal-shape analysis
  showed identical gross steering shape on all 5 laps (rules out line
  variance as cause).
- Result: change was surgically confined — 69->64 corners, exactly -1 per
  lap, all at the target section; nothing else on track changed.

### Compound-corner finding [2026-07-22]
- AND-exit revealed a ~430 m double-apex complex (~1805-2237 m, T3-T7
  region of Dubai GP): sustained 0.53-0.71 g between the two former
  "corners" on every lap. The old steering-only detector split it at the
  driver's wheel-opening point between apexes — a driver-input artifact.
- Thesis point: 0.5-0.7 g is continuous cornering, not a link section
  (a genuine gap drops below ~0.3 g). The tyres never unload -> one load
  event. Detection now flags brackets > 300 m as "compound_corner".
- Open refinement: phase segmentation of compounds (dual-apex structure,
  ay-minima splitting for PHASE purposes only, never for detection).

### Cross-lap corner identity via lap-distance clustering [2026-07-22]
- Problem: independent per-lap detection gave varying counts
  (13/15/14/15/15) -> corner N not comparable across laps, feedback
  un-mappable.
- Method: apex position via np.interp of lap_distance at apex_time
  (ft->m); 1D gap clustering across all laps (new cluster when gap >
  50 m); clusters numbered by ascending distance = stable_corner_id.
- Bracket-merge preprocessing (gap < 0.6 s, same direction) collapsed
  chicane split artifacts: counts converged 13/14/14/14/14; the three
  merges hit exactly the pre-identified split pairs and left
  opposite-direction chicanes untouched (direction criterion validated).
- Known behaviour: single-linkage chaining can join >tolerance pairs
  through intermediate laps' apexes; same-lap-duplicate warning is the
  guard signal.
  [SUPERSEDED by the two entries below -- point-based clustering and its
  gap-tolerance sweep successor both replaced by overlap-fraction +
  connected components; kept for the development narrative.]

### Overlap-fraction join criterion [2026-07-22]
- Replaces a fixed-metre gap tolerance: two brackets (different laps)
  link iff overlap_length >= bracket_overlap_min_fraction (0.3) *
  min(bracket_length_a, bracket_length_b). Proportional, not additive --
  a coincidental few-metre overlap between two genuinely different
  corners can no longer masquerade as a real link regardless of bracket
  size.
- ASSUMPTION documented, not hidden: this criterion assumes a genuine
  same-corner pair always overlaps rather than leaving a small gap.
  True with 100+ m margin on every clean Dubai cluster. Extreme
  lap-to-lap brake-point variance on a future track could in principle
  produce a small gap for a real same corner instead of an overlap --
  a documented stop-and-ask case, deliberately not pre-patched with a
  tolerance pad. Second-track validation will test this assumption
  directly.

### Connected components + seeded splitting [2026-07-22]
- Replaces a greedy left-to-right interval sweep that failed on real
  data: an ordinary 195 m corner detected only by one lap was
  permanently absorbed into an unrelated neighbouring cluster via a
  coincidental ~2 m (1% fraction) bracket overlap with whichever cluster
  happened to still be open -- confirmed by a full manual trace of the
  sweep before the fix.
- Method: build the link graph (overlap-fraction criterion above); take
  connected components as candidate clusters; where same-lap
  exclusivity is violated (a lap contributes >1 bracket to one
  component -- the compound-straddle case), split deterministically:
  seed from the lap with the most brackets in that component, assign
  every other bracket to its best-overlap-fraction seed, tag
  "straddles_adjacent_corners" when a second-best seed also clears the
  threshold.
- Revealed MORE structure than apex-point clustering could see BY
  CONSTRUCTION (it only ever compares single points, never bracket
  geometry): two additional compound-straddle regions beyond the one
  already documented, plus one containment case (one lap's compound
  bracket entirely contains a different lap's separate short corner,
  100% overlap fraction on the smaller bracket). Pre-stated acceptance
  predictions ("three singletons," "straddle tags in one region only")
  were revised on this traced evidence -- every reassignment reduces to
  a specific, checkable overlap-fraction number, nothing is asserted
  without it.
- Final structure: 11 full (5-lap) clusters + 3 partial (compound-
  straddle splits) + 1 true singleton = 15 stable corners. Same-lap
  exclusivity holds everywhere; no residual violation after splitting.
  [SUPERSEDED 2026-07-22 by the limiter-based inlap reclassification
  below -- lap 5 was contaminated by pit-entry driving and is now
  excluded from is_valid_for_analysis, so the valid-lap count dropped
  from 5 to 4: 11 full (now 4-lap) + 3 partial + 0 singleton = 14
  stable corners. The clustering METHOD documented above is unchanged;
  only the input lap set changed.]

### Recommendation engine: median-of-medians aggregation + classifier reuse [2026-07-22]
- Corner-level recommendation evidence aggregates each stable corner's
  per-lap phase medians via a further median across laps (median-of-
  medians), not a mean and not single-lap values. This deliberately
  privileges REPEATABLE behaviour: a single-lap anomaly (traffic, a
  missed apex, a gust) washes out of the aggregate and cannot by
  itself drive a setup suggestion -- a recommendation only fires when
  the pattern holds across the analysed laps. n_laps (cluster member
  count contributing to that corner) is carried in the evidence trail
  so a partial cluster is visibly thinner evidence than a full one,
  without any confidence weighting suppressing it.
- Rules never reimplement the CS/stability thresholds: they call the
  identical classify_fn (self._classify_corner) the stability grid
  already uses, sliced to the rule's own phases. This is a structural
  guarantee, not a convention that could drift -- a recommendation for
  a corner can never disagree with the verdict the engineer already
  sees in the grid for the same corner and phase, because both read
  the same classifier.

### Limiter-based inlap reclassification [2026-07-22]
- Duration-window lap validity (`is_valid_for_analysis`: lap_time <=
  1.10x fastest) had ACCEPTED the pre-merge "lap 5" (129.2s, well
  inside the 110% window) as a normal valid lap -- but its final ~13s
  were already pit-entry driving under the pit speed limiter
  (`ecu_B_speedlimit_en` engages at t=1108.78s, 13.14s before the
  lap_number transition; `ecu_speed` decays from 140.7 km/h to
  pit-limiter speed ~55-60 km/h entirely within that window, confirmed
  by three independent limiter-state channels agreeing on the same
  timing). A duration check cannot see this -- the contamination is in
  WHAT was driven, not how long it took.
- Fix: Level 3 limiter-channel detection (`ecu_B_speedlimit_en` engaged
  at the trailing fragment's first sample; Level 1 fallback: fragment
  shorter than a config duration threshold when the channel is
  missing) merges the session-trailing fragment into the preceding
  lap and flags it `is_inlap`, excluded from `is_valid_for_analysis`
  like the outlap. Merged duration 137.4s.
- Effect: stable corner count 15->14 (see superseded note above); the
  one true singleton (lap-5-only) disappeared entirely once lap 5
  stopped contributing corner candidates -- exactly the predicted
  failure mode, not a coincidence.
- THESIS POINT: qualitatively different from earlier accuracy-level
  upgrades (Level 1->3 for beta, kerb detection, etc.), which refine a
  NUMBER within an already-correct classification. Here a Level 3
  signal corrected a STRUCTURAL misclassification a duration window
  could not detect in principle -- the accuracy-level cascade's value
  is not limited to numerical precision.

## 2. Design principles (architecture chapter material)

### Accuracy-level cascade [project-wide]
- Every physical quantity: Level 1 config default -> 2 session
  measurement -> 3 logged sensor -> 4 lookup table. Nodes upgrade
  independently without pipeline restructuring. This is the answer to
  "how do you ship a useful tool with imperfect data TODAY".
- [2026-07-22] The cascade applies to lap timing itself, not just
  vehicle dynamics: boundary-sample durations (Level 1, +/-0.2s grid,
  from the lap_number channel's own sample interval) are retained for
  slicing; the logger's dedicated lap-timer channel (Level 3, ms
  resolution) is adopted for display and fastest/validity decisions --
  after a real tie at grid resolution (laps 3 and 4 both quantised to
  125.19999999999993s) demonstrated the need, not as a speculative
  upgrade.

### Analysis layer vs human layer for corner identity [2026-07-22]
- The tool detects LOAD EVENTS (anything stressing tyres, incl. flat-out
  kinks); humans think in NAMED corners (the track map in every driver's
  head). The two may disagree on count — the mapping between them must be
  explicit, not forced. stable_corner_id = analysis key; display label
  from per-track template = human key; feedback binds to the HUMAN layer.
- Track-robustness architecture (three layers): (1) expected corner count
  per track displayed against detected count; (2) engineer-in-the-loop
  merge/split correction, persisted per weekend; (3) GPS-based track
  template making the correction permanent per circuit. Matches how race
  engineering works: tools propose, engineers decide.

### Transparency over suppression
- kerb_fraction reported per phase instead of silently filtering;
  cell-level colour shows per-phase tyre state honestly while card-level
  severity answers "is this corner broken" — two different questions,
  deliberately allowed to differ.
- Missing corner in a lap = empty grid cell, not renumbering. A missing
  detection is itself information (wider line, lift, traffic).

### Data and driver as co-equal evidence sources [2026-07-22]
- The recommendation engine's rule trigger types (data / driver / both)
  treat classifier output and driver feedback symmetrically: either
  can independently raise a hypothesis (a "data" rule fires from the
  classifier and lets feedback modulate it; a "driver" rule fires from
  feedback and lets the classifier modulate it instead), and each
  modulates the other's score up (agreement) or down (conflict) rather
  than one silently overriding the other. A global source_balance
  setting can additionally weigh how much a data- vs driver-raised
  hypothesis counts (neutral by default) without touching that
  agreement/conflict modulation -- balance decides who may raise a
  hypothesis, modulation decides what the other source says about it.
- Conflicts are SURFACED, not suppressed: a rule that fires with a
  contradicting signal from the other source is flagged in a
  "conflicts" list carried through to the UI, not discarded or
  averaged away. Continues the transparency-over-suppression principle
  above -- a driver feeling something the data doesn't show (or vice
  versa) is a debrief item the engineer should see, not noise to be
  filtered out.
- The engine's three scoring signals mirror a real debrief's
  information flow, and are kept deliberately orthogonal rather than
  collapsed into one tunable: source_balance decides WHO may raise a
  hypothesis (data-driven or driver-driven); agreement/conflict
  modulation decides WHAT the other source says about that specific
  hypothesis once raised; the worst-corner flag is the driver's own
  PRIORITISATION, scaling every finding on that corner regardless of
  which source raised it. Three separate questions an engineer asks in
  a debrief, kept as three separate multipliers rather than one
  conflated weight -- so each can be reasoned about and defended on
  its own.

### Verdict vocabulary
- Deliberately limited to understeer / oversteer / unstable yaw / ok.
  Engineering jargon ("collapse", "saturated", "loaded") tested and
  rejected — the tool speaks the language of the debrief.

## 3. Validation results (Dubai sample, 992 GT3R, 5 valid laps, 50 Hz)

- beta: -4.29 to +2.91 deg, mean abs 0.87 deg — plausible GT magnitude.
- alpha_f mean abs 1.64 deg > alpha_r 1.00 deg — understeer signature,
  consistent with driver report.
- C_alpha front mean ~115k, rear ~178k N/rad; implied linear refs 161k /
  190k N/rad (expected band 80-180k) — rear stiffer matches 57.2% rear
  weight.
- Yaw accel +/-5.5 rad/s^2; Mz_inertial +/-11 kNm; stability median
  ~2.7 kNm/deg, 93% of samples stabilising.
- GPS: log_gps_lat 25.047-25.053, lon 55.232-55.245 (Dubai Autodrome
  confirmed), 10 Hz, 11-12 sats — live and usable.
- Iz = 2082 kg m^2 from m*a*b estimate.

## 4. Level 1 limitations register (limitations chapter)

1. Static weight split for Fy (overstates rear on roll-stiff GT3).
2. Iz from m*a*b bicycle estimate (~10-20% error; all Mz scales linearly).
3. Steering ratio constant (+/-25% real variation over travel).
4. beta from kinematic integration + washout (drift-corrected, not
   measured). Upgrade path identified: log_gps_course (velocity-vector
   direction, 10 Hz) vs chassis heading -> direct beta (Level 3).
5. Accelerometer assumed at CoG.
6. No yaw damping term in Mz (Werner drops it too).
7. Closed-loop derivative — driver in the loop affects c_beta.
8. Bicycle model: identical slip per axle.
9. Kerb detection: static threshold (rate-of-change upgrade documented).
10. ecu_speed is Level 1; log_gps_speed available for cross-validation.

## 5. Development-process notes (optional methodology section)

- Threshold tuning workflow: hypothesis -> distribution diagnostic script
  -> percentile-based threshold choice -> re-validate on data. (Scripts:
  inspect_kerb_signal.py, inspect_corner_distribution.py.)
- Detection-change validation pattern: pre-simulate on existing data
  before any code change (clustering simulated on 72 corners before
  implementation); classify failure cases (a/b/c) with a read-only
  diagnostic before choosing a fix.
- AI-assisted development with human verification gates: propose-approve-
  implement-test loop; every change reviewed as diff; science decisions
  made separately from implementation. (Include/exclude in thesis at
  author's discretion.)

## 6. Open questions / to verify before writing

- Name the official turns spanned by the compound complex (GPS
  coordinates of bracket start/peak/end vs track map — pending).
- Second-track validation of clustering tolerance (50 m) once new data
  arrives.
- Werner MA full citation + exact method name for the CS reference.
- Suzuka convention citation for c_beta sign.
- Confirm 992 GT3R official corner-weight/mass provenance for the
  constants table.

- Werner method-delta comparison: three-column table (adopted as-is /
  deliberately different + why / not implemented + upgrade path),
  built against the original paper re-uploaded in a dedicated session.
  Include exact parameter values where we diverge. Best timed after
  WP5b, so the "not implemented" column reflects final state.

  [2026-07-22] Official corner count for Dubai GP is itself inconsistent
across sources (Wikipedia: 17 turns; multiple track databases: 16).
Detector found 17 stable corners. Direct real-world evidence for the
two-layer corner identity design: corner counting is a human convention,
not a physical fact — load-event detection and human naming must be
separate layers with an explicit mapping.

[2026-07-22] Corner clustering evolved point-based (apex distance, 50 m
gap) -> interval-based (bracket span overlap + same-lap exclusivity),
forced by a real failure: in compound/double-apex corners the peak-G
point jumps 80-190 m between apexes lap-to-lap, fragmenting one
physical corner into complementary clusters (diagnosed via
complementary lap-membership signature). Bracket boundaries are stable
where apex position is not. Same-lap exclusivity converts the duplicate
warning into a structural guarantee: no cluster can claim two events
from one lap, protecting genuine per-lap extra detections from being
merged away.

[2026-07-22] Segmentation acceptance criteria (frozen for thesis scope):
structural same-lap exclusivity; physically explainable memberships;
ambiguity marked not hidden (straddle/compound warnings,
placeholders); stable cross-lap identity for feedback mapping. NOT
required: complete per-lap coverage or agreement with official turn
counts (themselves inconsistent, 16 vs 17 for Dubai GP). Remaining
imperfection is bounded by the engineer merge/split override (WP3b).
Rationale: segmentation is a means; the deliverable is per-corner
stability insight. Driving ambiguity (one lap taking two corners as
one event) is irreducible in the data, not an algorithm defect.

[2026-07] Inlap reclassification cascade: limiter-based inlap
detection (Level 3, ecu_B_speedlimit_en) revealed that
duration-window validation had accepted the true inlap (lap 5) as a
valid race lap — its final sector is pit-entry driving. Merging the
8 s pit-lane fragment into it and excluding it by default removed
singleton cluster C14, whose sole supporting corner was detected on
that lap: what looked like a genuine one-lap extra corner was
pit-approach driving. Two lessons: (1) the accuracy-level cascade can
correct structural misclassifications, not just refine numbers;
(2) data-quality fixes propagate — a lap-model correction changed the
corner-identity result without touching the clustering algorithm.