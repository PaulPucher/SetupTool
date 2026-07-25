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
- ~~No tyre model required — everything from logged signals. This is
  the central methodological claim: tyre-state estimation without tyre
  data.~~
- [FRAMING CORRECTION 2026-07-24, primary source verified] The project
  ADOPTS Werner's framework as-is — derivative definitions incl.
  dMz/dbeta stability (his §2.2.3 pp.15-16, after Milliken),
  measurement-side Mz = Iz*psidd + D_psi*psid (his §4.5.2 Eq. 4.3/4.4),
  linear-reference Calpha and effective-stiffness concepts. ONE
  component is necessarily adapted, not adopted: Werner evaluates
  effective stiffnesses from a Pacejka tyre model; no validated tyre
  model exists for the 992 GT3R, so effective Calpha is estimated
  directly from logged Fy/alpha by windowed local regression (section
  blend, R^2 weighting, linear-reference hold, 0.021 rad gate — the
  adaptation layer, specific to this project's sensor situation, not
  present in Werner. Practical consequence of the adaptation: the
  pipeline runs on logged signals alone, making it applicable for
  customer teams without access to manufacturer tyre data). The
  estimator's internal machinery (windowing, section blending, R^2
  weighting, hold rule) is signal-conditioning for noisy measured data
  — engineering robustness choices, explicitly NOT claimed as
  methodological novelty.

### Fy yaw-moment term (Module 4a) [2026-07-24]
- Replaces the pure static weight split (Fy_f = m*ay*front_fraction,
  Fy_r = m*ay*rear_fraction) with the exact 2-DOF planar force/moment
  balance: Fy_f = m*ay*front_fraction + Iz*psidd/wheelbase,
  Fy_r = m*ay - Fy_f. Tier A: Milliken & Milliken, RCVD, 2-DOF planar
  force/moment balance, p. TBD verify (same pending-citation pattern as
  estimate_sideslip). Same construction as the chair performance_
  analysis tooling's own fy_f_N/fy_r_N (internal) -- adopted as-is, no
  deviation.
- psidd is the RAW np.gradient(yaw_rate_radps, time), computed fresh in
  Module 4a -- deliberately NOT Module 5's 0.15 s rolling-mean-filtered
  signal. The chair itself keeps these separate (raw for the
  instantaneous per-sample force balance, filtered only for the
  windowed stability regression); reusing Module 5's filtered signal
  here would double-filter with an inconsistent time constant ahead of
  Module 4b's own downstream Butterworth stage.
- Method upgrade only, not an accuracy-level upgrade: Iz and the static
  corner-weight fractions are still Level 1, so the new term inherits
  their ~10-20% uncertainty; accuracy_levels.lateral_force_split stays
  1.
- OBSERVED (Dubai, 51 corner instances, diagnostics/
  inspect_vehicle_model_upgrade.py and inspect_corner_distribution.py):
  - RMS(Iz*psidd/wheelbase) / RMS(m*ay) = 5.3% -- a modest, plausible
    correction, not dominating the static term.
  - Steady-state reproduction (smallest 10% |psidd| bucket, n=3939):
    median relative |Fy_f_new-Fy_f_old|/|Fy_f_old| = 2.06%, confirming
    the new formula collapses back to the old static split when the
    car isn't rotating, as it must. (The bucket's MAX relative diff is
    a large-looking 2399% -- this is a denominator artifact from a
    sample where Fy_f_old is itself near zero, not a real error; the
    median is the meaningful statistic here.)
  - Worst-phase-per-corner CS percentiles moved as expected: CSf p50
    0.337->0.367, p10 0.082->0.030; CSr p50 0.616->0.704, p10
    -0.052->0.082 (rear no longer dips negative at p10 -- a plausible
    improvement, not engineered for).
  - PREDICTED-VS-OBSERVED MISMATCH, worth stating plainly: the
    pre-registered expectation was "CS medians shift in entry_1/
    entry_2, negligible in apex/exit" (largest |psidd| is at turn-in).
    Observed is the opposite for the per-phase MEDIAN-across-51-
    instances statistic: entry_1_brake and entry_2_turnin show exactly
    zero median shift (both pinned at the 1.0 ceiling in old and new),
    while apex_3 shows the largest shift (CSf 0.632->0.547) and exit_4
    a small one. Explanation: CS_ratio is clipped at 1.0
    (estimate_cornering_stiffness), and most entry-phase instances sit
    AT that ceiling in both old and new (strong, clean alpha excitation
    under braking/turn-in keeps C_alpha near the linear reference) --
    the ceiling saturates the median regardless of the underlying Fy
    perturbation. apex_3 (few samples, deep in the nonlinear regime,
    rarely clipped) is where the ratio metric is actually sensitive to
    the change. Individual corner instances DO shift at entry (e.g.
    corner 3's entry_2_turnin CSf: 0.52->0.42) -- it is the
    aggregate-median statistic specifically that hides this, not the
    underlying signal. The worst-phase-per-corner percentiles above are
    the more informative distribution for threshold re-derivation
    purposes precisely because they are not this kind of per-phase
    aggregate.
  - Modules 1-3 and Module 5 confirmed byte-identical in test_stability.py
    before/after (diffed line-by-line) -- only Module 4a/4b numbers
    moved, as intended. Stability threshold distribution (worst-phase
    percentiles) also confirmed unchanged, as expected (Module 5
    consumes neither Fy nor Fz).
- CLASSIFICATION thresholds NOT changed by this entry -- re-derivation
  is the user's own decision from the reported percentiles (same
  standing rule as B1-B3): I derive nothing, only report.

### CS threshold re-confirmation after Fy yaw term [2026-07-24]
- After the Fy yaw-moment term landed (see above), CS thresholds
  (STRONG_CSF/CSR, MODERATE_CSF/CSR) were re-checked against the new
  worst-phase-per-corner distribution rather than left stale. Decision:
  KEEP the 2026-06-29 values unchanged -- the shift was real but small
  enough that the existing thresholds still land in a sensible place.
- Exceedance-rate comparison (51 instances, same diagnostics/
  inspect_corner_distribution.py method as 2026-06-29): CSf<0.10 7->6,
  CSf<0.25 18->16; CSr<0.20 7->6, CSr<0.35 12/51 (worst-phase CSr below
  MODERATE_CSR -- not separately tracked before this check). Maximum
  movement in any single flag-count bucket: 2 instances out of 51.
- N=51 RESOLUTION ARGUMENT: at this sample size, "p10" corresponds to
  roughly the 5th-ranked value out of 51 -- a percentile figure moving
  by a few hundredths (e.g. CSf p10: 0.082->0.030 across the Fy
  change) can be one or two instances re-ordering near a boundary,
  not a meaningful distribution shift. Exceedance COUNTS (how many
  corners actually cross a fixed threshold) are the more trustworthy
  statistic to act on at this N than fine percentile arithmetic --
  which is why the re-confirmation above is framed as flag-count
  deltas, not as a fresh percentile-anchored derivation from scratch.
- CSr-tail repair, worth noting as a genuine (not engineered-for)
  improvement: worst-phase CSr p10 was -0.052 pre-Fy-fix (a negative
  cornering-stiffness-ratio at the 10th percentile is physically
  awkward -- it means the windowed OLS slope went the wrong sign for
  at least ~5 of 51 instances); post-fix it is +0.082, no longer
  negative at p10. The 2-DOF Fy correction was not tuned toward this
  outcome -- it applies equally in both directions depending on
  rotation sense -- so a directional improvement specifically in the
  metric's most awkward tail is a plausibility point in favour of the
  formula being applied correctly, not proof by itself.
- VERDICT-DISTRIBUTION CONFIRMATION (config thresholds unchanged,
  diagnostics/inspect_b3_verdict_distribution.py): 0 strong / 14
  moderate / 37 normal (was 1/16/34 just after B3, before the Fy fix).
  Two instances changed branch/severity without leaving the flagged
  set: lap 1 corner 8 and lap 2 corner 8 (both stable_id 8) dropped
  their CS-branch trigger as front CS improved, leaving each on
  stability-only (lap 2's dropped from strong to moderate accordingly,
  since strong required a concurrent CS trigger). Four instances left
  the flagged set entirely (lap 2 corner 7 id 7, lap 2 corner 13 id 14,
  lap 3 corner 2 id 2, lap 4 corner 12 id 14) as their worst-phase CS
  improved past threshold. One new instance entered (lap 3 corner 4
  id 4, CS branch, worst_CSf now -0.454) -- a reminder that the yaw
  term is signed and can make a corner's CS reading worse as easily as
  better, depending on rotation direction; it is not a one-directional
  correction. Net: 17 non-normal instances -> 14, driven entirely by
  CS-branch movement (stability-only/both counts for stable_id 8 are
  internally consistent with the byte-identical Module 5 output).

### Ground-truth alignment improved under the corrected Fy model [2026-07-24]
- After the Fy yaw-moment term, the session's only "strong" verdict
  disappeared (verdict distribution 1/16/34 -> 0/14/37). Not a
  sensitivity loss: the corrected front Fy softens stable_corner_8's
  apparent CS collapse, while the re-derived stability branch still
  flags that corner on all 4 laps (3x stability-only, 1x both).
- Against the June driver report ("balanced, mild understeer, no
  instability"), 0 strong / 14 moderate is the more consistent
  picture than the pre-correction 1 strong. Model upgrade moved the
  tool TOWARD the driver's ground truth - validation argument for
  the thesis, not just a numbers shift.
- Caveat for the write-up: single session, single driver report,
  n=51; alignment evidence, not proof.

### Yaw moment stability dMz/dbeta [2026-06]
- ~~Mz_inertial = Iz * psi_ddot (yaw accel from differentiated, 5 Hz
  filtered yaw rate). Local centred 2 s OLS of Mz over
  [1, beta, delta_f, v, ax].~~ [SUPERSEDED 2026-07-24 -- see "Module 5
  chair-basis alignment" below. The 5 Hz Butterworth yaw-accel filter and
  the time-anchored single-lap 2 s OLS are both replaced; this entry kept
  for the development narrative.]
- ~~c_beta > 0 stabilising, < 0 destabilising (Suzuka convention).~~
  [CORRECTED 2026-07-24, primary source verified] "Suzuka convention"
  was an informal label with no literature basis found in the
  primary-source check; replaced. Sign convention per Werner (2021)
  S2.2.3: positive dMz/dbeta = restoring = stable.
- KEY DERIVATION for thesis: yaw rate EXCLUDED from the regressor set
  because of structural multicollinearity with beta via the kinematic
  identity beta_dot = ay/v - psi_dot. Including both makes the OLS
  ill-conditioned and the coefficients uninterpretable. Still holds for
  the chair-basis regressor set below (beta, delta_f, v, ax, az) -- the
  identity doesn't depend on which estimator evaluates it.
- Catches a different failure mode than CS_ratio: tyre can be within grip
  while vehicle dynamics are unstable, and vice versa.

### Completing Werner Eq. 4.3 — damping term via wheel loads [2026-07-24]
- Werner could not evaluate the damping term D_psi (no wheel-load
  sensors, his §4.5.2) and stopped measurement-side Mz analysis there.
- This car logs damper forces (log_dms_dam_*) -> the WP5b wheel-load
  upgrade funds computing D_psi and completing Eq. 4.3 with both
  terms — the project's direct sensor-enabled extension of the
  chair's prior work.
- Until then, Iz*psidd-only is a shared, documented approximation
  (limitation #6, same order of magnitude as Mz_inertial at race
  speed).

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
  MODERATE_CSR=0.35, ~~STAB_NEG_THRESH=-500 Nm/deg.~~ [SUPERSEDED
  2026-07-24 -- see "Stability-threshold re-derivation for the
  chair-basis estimator" below. CS thresholds unchanged and still
  valid -- only the stability threshold moved, and only because the
  estimator producing that signal changed (B1).] Asymmetric front/rear
  thresholds because rear CS stays structurally higher on this car
  (57.2% rear weight) — when rear drops it means more.

### Stability-threshold re-derivation for the chair-basis estimator [2026-07-24]
- STAB_NEG_THRESH moves from -500 to -50 Nm/deg. Not a re-tightening of
  the same gate -- the estimator underneath it changed (B1), so the old
  value's meaning doesn't carry over; this is a fresh derivation against
  the new distribution, same METHOD class as 2026-06-29 (percentile-
  anchored, diagnostic-script-driven, re-validated against ground truth),
  different number because the input distribution is different.
- OLD (-500, time-anchored OLS estimator): sat close to the sample-level
  p2.5 percentile of that estimator's output (p2.5 = -554.1); flagged
  1 of 51 corner instances (stable_corner_id 8, lap 3 only).
- NEW (-50, chair-basis s-anchored ridge estimator): gap-selected, not
  percentile-anchored to a fixed rank -- the worst-phase-per-corner-
  instance distribution (diagnostics/inspect_yaw_stability_b2.py) has a
  clean, wide gap between -99.2 Nm/deg (stable_corner_id 8, all 4 laps)
  and -18.5 Nm/deg (next-worst corner, a different physical corner).
  -50.0 sits in that gap: it flags exactly the same single physical
  corner as before, now correctly across all 4 of its laps instead of
  1 (the old threshold only just crossed lap 3's value, missing the
  other 3 laps of the same corner that were arguably worse). Sample-level
  exceedance is 3.0% (new) vs 2.7% (old) -- comparable order of
  magnitude despite the ~10x smaller absolute threshold, because the
  new estimator's whole distribution is compressed by roughly the same
  factor (see B1 report).
- Ground-truth check: ground truth for Dubai is a balanced car with mild
  understeer tendency and no reported yaw instability. A single
  consistently-destabilising physical corner across all 4 laps, and
  nothing else crossing the gate, is consistent with that -- the
  re-derivation does not manufacture new instability findings, it just
  restores the gate's sensitivity to the corner it was always meant to
  catch, at the new estimator's scale.
- Verification distribution (51 corner instances, new estimator +
  re-derived thresholds): see B3 verdict-distribution report in the
  session log / PLAN.md STATUS.

### Module 5 chair-basis alignment: s-anchored ridge regression [2026-07-24]
- Replaces both pieces of the old estimator: yaw acceleration now a
  centred rolling mean (0.15 s window) over differentiated yaw rate,
  not a 5 Hz Butterworth low-pass; the stability regression is now
  s-anchored (grid in lap_distance, 2 m step, 55 m Gaussian half-width)
  rather than time-anchored (2 s window, single lap). Regressors
  beta, delta_f, v, ax, az (az optional, dropped cleanly if
  log_acc_z is unavailable); ridge solve in per-regressor-standardised
  space instead of plain OLS.
- ATTRIBUTION SPLIT: the target relation (Mz = Iz*psidd + D_psi*psid,
  dMz/dbeta sign convention) stays Werner (2021) S2.2.3/S4.5.2 Eq. 4.3
  -- unchanged, see above. The ESTIMATOR construction (rolling-mean
  filter, s-anchored grid, Gaussian weights, standardised ridge) is
  after the chair performance_analysis tooling (internal); this is a
  new adaptation layer, analogous to Module 4b's effective-Calpha
  estimator being an adaptation of Werner's own tyre-model evaluation.
- WHY s-anchoring changes the numbers, not just the noise floor: a
  single lap's ~2 s window around a corner apex often has weak beta
  excitation (closed-loop derivative limitation #7) -- the OLS
  coefficient on beta is then poorly conditioned even though the
  window "runs". Sorting all samples by lap_distance interleaves every
  lap's pass through the same corner, so the local Gaussian window at
  a given s pools 4 laps' worth of excitation instead of 1. This is a
  genuine methodological improvement in conditioning, not a fitting
  trick -- but it also means results are no longer directly comparable
  to the old per-lap numbers; B1's before/after report in PLAN.md
  quantifies the shift.
- THREE SetupTool-specific adaptations, all at the call site, estimator
  itself untouched (mirrors the CS_ratio adaptation-layer pattern):
  (a) yaw acceleration is differentiated from the raw yaw-rate channel
  only -- the chair function also accepts a pre-smoothed yaw-rate input
  from a chair-external filter list that is outside this project's
  reference scope, so only the raw-signal path is reproduced.
  (b) the chair invokes its estimator unmasked on a full session,
  relying on its own dropna handling for missing data; SetupTool feeds
  it the same arrays with three sample-exclusion masks pre-applied as
  NaN (moving, kerb, structural in/out-lap) -- the estimator's own
  missing-data handling is what actually enforces the exclusion, no new
  masking logic was added inside it.
  (c) structural in/out-lap exclusion is the new one, added as a
  PRODUCTION exclusion (not a report-only diagnostic): the local
  s-window regression implicitly assumes the same underlying vehicle
  condition recurs at a given track position across laps it pools.
  ~~Cold tyres (in/out laps) violate that -- tyre stiffness is
  temperature-dependent -- so an in/out-lap sample at the same s as a
  hot-tyre racing lap is not "the same corner" in the sense the
  regression needs.~~ [SUPERSEDED 2026-07-24, B2 diagnostic evidence --
  see "In/out-lap exclusion: two-leg rationale" below. Cold-tyre
  stationarity is still one real reason (the inlap leg), but B2 showed
  a second, independent, and quantitatively dominant reason specific to
  the outlap: not kept here because it understated the outlap case.]
  Same epistemic category as the kerb mask (both exclude samples
  unrepresentative of the condition being modelled), and deliberately
  independent of the UI's is_valid_for_analysis display filter (WP6:
  Module 5 must not depend on what the user is currently looking at).

### In/out-lap exclusion: two-leg rationale [2026-07-24, B2 evidence]
- The single "cold tyres" rationale above was correct but incomplete --
  B2's out/inlap s_m degeneracy check (diagnostics/inspect_yaw_
  stability_b2.py) found a second, separate, and larger effect specific
  to the outlap. The exclusion is really two legs, not one:
  - OUTLAP leg (data-validity necessity, FORCED in character): the
    outlap's native lap_distance channel is frozen at s~0 for its
    entire duration (min=0.0m, max=0.5m, std=0.0m over 17601 samples --
    the channel simply hasn't started counting yet). Every outlap
    sample therefore masquerades as being at track position s~0-50m
    regardless of where the car actually is. Of the no-inout-exclusion
    run's ~-97 Nm/deg artifact samples (9126 total), 8801 (96%) were
    outlap samples piled onto one grid point (s=54m, window population
    8301 of which 7563 were outlap). This is not a judgement call about
    representativeness -- the s-coordinate itself is invalid for these
    samples, so including them isn't "including a different vehicle
    condition", it's regressing against a fabricated track position.
    Excluding the outlap here is closer to FORCED ADAPTATION in
    character: the s-anchored method cannot be applied validly to
    samples whose s-coordinate doesn't mean anything.
  - INLAP leg (stationarity assumption, the original DOMAIN IMPROVEMENT
    reasoning): the inlap (lap 5, limiter-merged fragment) does NOT
    have a degenerate lap_distance -- it spans nearly the full lap
    (min=0.2m, max=5080.2m, std=1602.6m), so it contaminates every
    corner a little rather than one grid point a lot. Here the cold-
    tyre/stationarity argument is the only reason to exclude it, and it
    remains a DOMAIN IMPROVEMENT: post-session analysis lets us identify
    and remove a lap we know carries a different (cold) tyre condition,
    which a continuous online tool [context claim, to verify with
    chair] would have no equivalent opportunity to do.
- CLASSIFICATION (updated): the exclusion as a whole stays DOMAIN
  IMPROVEMENT (it is still a context-driven choice available to a
  post-session tool), but the outlap leg specifically is noted as
  FORCED in character -- excluding it isn't optional once the
  coordinate degeneracy is known, whereas the inlap leg is a genuine
  domain judgement call.

### Pooled grid makes stability a per-corner, not per-lap-instance, property [2026-07-24]
- Structural consequence of s-anchoring (B1): because the local
  Gaussian window at a given grid point in s pools samples from every
  lap that passes through it, the fitted slope at that grid point is
  already a cross-lap quantity before it is ever assigned back to an
  individual lap's corner instance. Interpolating the grid back onto
  each sample's own timeline (B1) means two different laps' passes
  through the same physical corner draw their stability value from
  the SAME underlying grid points whenever their s-ranges for that
  phase overlap -- which they do almost everywhere (grid coverage
  99.7% in production, per B1).
- Consequence for reading the per-lap corner grid in the UI: apparent
  per-lap differences in a corner's stability value are therefore not
  telling you the vehicle behaved differently lap to lap at that
  corner -- under this estimator they can only arise from which grid
  points a given lap's phase-boundary window happens to cover (phase
  segmentation timing varies slightly lap to lap, per WP1), not from
  a materially different regression result. This is different from
  CS_ratio (Module 4b), which is still a genuinely per-sample,
  per-lap quantity (windowed OLS on that lap's own alpha/Fy trace).
- Why this is worth stating plainly rather than leaving implicit: a
  reader comparing the per-lap stability column across laps for one
  corner could otherwise mistake grid-coverage noise for a real
  lap-to-lap driving difference. The median-of-medians aggregation the
  recommendation engine already uses (see "Recommendation engine:
  median-of-medians aggregation" above) is unaffected by this -- it
  was designed to be robust to single-lap anomalies regardless of
  their source, and this is simply a new, understood source of
  small per-lap stability variance to be aware of, not a new failure
  mode to guard against.
- s_m channel-alignment note: lap_distance is SetupTool's own proxy for
  the chair's native s_m, interpolated onto the common sample timeline;
  guarded against fabricating a value across a lap-boundary reset
  (linear interpolation between the last high-distance sample and the
  first near-zero sample of the next lap would otherwise synthesise a
  midpoint corresponding to no real track position). Tier B
  channel-alignment necessity, not a method change.
- CLASSIFICATION (deviation taxonomy, CLAUDE.md) [2026-07-24]: (a) the
  raw-yaw-rate-only path = FORCED ADAPTATION (the chair's pre-smoothed-
  input filter list is outside this reference's scope; no alternative
  input is available to us). (b) the NaN-then-dropna masking wiring
  itself = NEUTRAL ENGINEERING (no science content, just how the three
  exclusions are fed into the chair's own missing-data handling). (c)
  structural in/out-lap exclusion = DOMAIN IMPROVEMENT overall
  [UPDATED 2026-07-24: see "In/out-lap exclusion: two-leg rationale"
  above -- the outlap leg is FORCED in character (s-coordinate
  degeneracy), the inlap leg is the original DOMAIN IMPROVEMENT
  (cold-tyre stationarity, chair presumed continuous/online [context
  claim, to verify with chair] vs. SetupTool's post-session analysis)].
  The s_m reset-guard interpolation above = NEUTRAL ENGINEERING.

### Moving-speed mask: domain-improvement classification [2026-07-24]
- `moving_speed_min_mps` (state["moving_mask"] = v_mps > threshold)
  excludes stationary/pit-lane samples from both Module 4b and Module
  5. The chair's reference estimator has no equivalent explicit speed
  gate; it relies on its own dropna handling for whatever is missing,
  not on excluding low-speed driving.
- CLASSIFICATION: DOMAIN IMPROVEMENT, same reasoning as the in/out-lap
  exclusion above: the chair pipeline is presumed to run online/
  continuously [context claim, to verify with chair], where a
  low-speed sample is just the current state, not something to
  discard; SetupTool's post-session analysis can identify and remove
  standing/pit-lane samples that carry no cornering information and
  would otherwise dilute the regression. Based on their version, which
  is not wrong for their context -- this is a decision available to us
  because of what SetupTool is (post-session), not a correction of an
  error in theirs.

### Kerb/jump exclusion [2026-06-29]
- Vertical accel (log_acc_z) deviation-from-baseline gate: |az - 1.0g| >
  1.2g, dilated +/-5 samples (0.1 s ringdown). Baseline is +1.0g:
  Cosworth/SCLU convention is z-down (gravity positive) — discovered
  empirically (initial -1g assumption flagged 100% of samples).
- Threshold tuned to flag 3.0% of moving samples on Dubai (target band
  0.5-3%, plausible for moderate kerb usage).
- Effect on results: stability valid samples 30813->29550, median
  2547->2676 Nm/deg (kerbs were biasing stability DOWN), CS_ratio means up
  ~0.006. ~~Per-corner: one apex phase dropped from 100% to 18% valid —
  kerb transparency at exactly the right place.~~ [SUPERSEDED 2026-07-24,
  B1 finding -- true of the time-windowed OLS estimator this was
  measured against, not of the current one. The s-anchored estimator
  (B1) computes validity at the GRID level (2 m step, 55 m window) and
  interpolates back onto every sample; a phase with locally low sample
  density or a kerb-affected pocket gets infilled from its clean local
  neighbourhood in s by construction, because the grid point nearest
  that phase can still draw on samples from every other lap at the same
  track position. Re-measured on the new estimator: this same corner's
  apex phase now reads valid_stab=100% (see B1 report), and the same is
  true almost everywhere -- grid coverage was 99.7% in production.
  valid_fraction_stab is therefore NON-DISCRIMINATING under the new
  estimator and should not be read as a kerb-transparency signal any
  more; kerb_fraction (a separate, untouched field, still computed
  per-phase from the same kerb mask) remains the correct transparency
  signal for "was this phase affected by a kerb". This is a documented
  side-effect of the s-anchored/grid-based validity mechanism, not a
  regression to fix -- see B1 report.]
- LIMITATION (Level 1): static deviation threshold; sustained aero load
  (1.5g at 250 km/h) approaches the threshold. Rate-of-change (daz/dt)
  detection is the documented upgrade path.
- CLASSIFICATION (deviation taxonomy, CLAUDE.md) [2026-07-24]: DOMAIN
  IMPROVEMENT. The chair pipeline serves a vehicle class not expected
  to ride kerbs regularly [context claim, to verify with chair]; a
  GT3 car uses kerbs every lap as part of the racing line, so an
  unmasked kerb transient corrupts exactly the samples (apex,
  exit) that matter most to the regression. Based on their version,
  which is not wrong for a vehicle class that doesn't need it -- the
  threshold-gate MECHANISM itself (deviation-from-baseline gate,
  dilation for ringdown) is standard Tier B practice, not the
  contribution; the contribution being claimed is the domain analysis
  (GT3 kerb usage pattern) and the decision to exclude, not the filter
  construction.

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

### No-steering-channel fallback: Tier B heuristic, not part of the method [2026-07-24]
- `_bracket_corners_by_speed` (used only when the steering channel is
  missing) finds corners from speed minima, keeping one only if the
  rise back to the surrounding local peak clears
  `min_apex_speed_drop_kmh`. This is a valley-depth check, not a formal
  peak-prominence algorithm -- the module header previously overclaimed
  "prominence threshold"; corrected. Config-driven, data-derived
  threshold, explicitly a Tier B fallback: never the primary detection
  method above, which is the dual-criterion steering/ay logic.

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

### Deviation taxonomy for chair-comparison [2026-07-24]
- Every place SetupTool's estimators differ from the chair
  performance_analysis tooling (internal reference, docs/literature/,
  read-only) carries exactly one of three class labels, defined in
  CLAUDE.md: FORCED ADAPTATION (no alternative given the GT3R
  sensor/data situation -- same method, different available inputs),
  DOMAIN IMPROVEMENT (their version is correct for their context; ours
  differs and we improve on it for ours -- "based on their version,
  which is not wrong"), NEUTRAL ENGINEERING (no science content:
  channel-alignment guards, config key naming, module boundaries).
- Why a taxonomy and not just prose: an examiner's first question
  about any difference from a cited reference is "why did you change
  it, and does that weaken the reference anchor?" The three-way split
  answers that up front and consistently -- forced vs chosen vs
  cosmetic -- rather than requiring the same argument to be
  reconstructed ad hoc for every deviation. Every current deviation
  (kerb mask, moving mask, in/out-lap exclusion, the raw-yaw-rate path,
  s_m interpolation) is labelled at its own thesis_notes.md entry; every
  future one gets exactly one label when it's introduced, not
  retrofitted later.

### Vehicle parameterization is not a deviation [2026-07-24]
- The chair tooling is vehicle-agnostic; every physical vehicle
  quantity enters through config, not through the algorithm. SetupTool
  parameterizes the identical algorithms for the Porsche 992 GT3R,
  whose properties differ fundamentally from the chair's reference
  vehicle (different vehicle class) -- "chair-identical" always means
  the algorithm, never the vehicle numbers, so this is never a
  deviation-taxonomy entry on its own. Three parameter categories, each
  with its own provenance rule: (1) vehicle description (mass, Iz,
  wheelbase, track widths, ...) differs from the chair BY NECESSITY --
  provenance is the Level 1-4 accuracy system, not chair comparison.
  (2) method calibration tunables (the six yaw_stability_* values,
  cs_* values, ...) match the chair BY CHOICE -- they are the chair's
  own dataclass defaults, adopted deliberately; changing any is an
  estimator change and re-triggers threshold re-derivation. (3)
  classification thresholds differ from any chair values BY RULE --
  always re-derived from this car's own output distribution (see the
  B1/B2 threshold-re-derivation workflow), never carried over from the
  chair or from a prior estimator's distribution.

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

### Setup-parameter registry design [2026-07-23]
- The parameter registry (config/setup_parameters.json, WP2b-1)
  separates five concerns per tunable that a recommendation engine
  would otherwise conflate into one opaque string: identity (mapping
  to the real car.json field), value space (range/units), direction
  semantics (what "increasing this value" physically means -- not
  assumable from the field name alone, see below), physical mechanism
  (one sentence, defensible on its own), and change_effort. Keeping
  these as separate, independently-inspectable fields is what let a
  systematic per-entry audit catch a real direction error (see next
  point) instead of a plausible-sounding but wrong rule silently
  shipping.
- change_effort (seconds -> minutes -> garage_hours) creates a
  natural recommendation hierarchy for free: electronic changes
  (tc_lat/tc_lon/abs_position, seconds, driver-adjustable mid-session)
  are cheaper to try than mechanical clicks (dampers/ARB, seconds-
  minutes) which are cheaper than garage-level changes (springs,
  diff package, hours). A recommendation engine can prefer the
  cheapest lever that explains the evidence without any separate
  cost model -- the ordering falls out of the registry's own field.
- Registry entries were populated from a team-knowledge session;
  reference tables (ARB stiffness curves, damper matrix, diff
  locking-torque chart, wing legal set, ABS/TC per-position tables)
  were digitised from manufacturer/team documents into
  config/car_data.json and cited by key from the registry, rather
  than re-transcribed inline -- one lookup table, multiple registry
  entries can cite it (e.g. all four ARB positions cite the same
  arb.front/arb.rear stiffness curve).
- METHODOLOGICAL POINT: direction semantics were verified against the
  digitised source table for every entry that had one, not assumed
  from the parameter's plain-English description. This caught a real
  inversion -- tc_lat/tc_lon's own settings-overview table shows
  increasing position means LESS permitted rotation/wheel spin (more
  TC intervention), the opposite of a naive reading -- and separately
  showed abs_position is categorical (two dry-grip brackets plus an
  ascending wet-severity bracket), not the single monotonic soft-hard
  axis every other electronic parameter in the registry is. Both
  would have been highest-damage errors specifically because every
  future rule written against this registry inherits its direction
  claim silently; the audit step is what makes the registry a source
  of truth rather than a plausible-sounding guess.
- SCOPE DISCIPLINE: config/car_data.json was deliberately restricted
  to data with a named consumer (a registry entry, a WP5b work-plan
  item, or a module) after an initial pass over-digitised several
  smooth, unlabelled kinematic curves (toe/camber/antidive/roll-
  centre vs. wheel travel) that shared a source image with data that
  WAS needed, but had no consumer of their own. The manufacturer
  images remain the permanent archive (docs/car_data/, gitignored);
  the JSON only mirrors what the tool actually reads. One digitised
  table (the steering-ratio lookup) was pruned and then restored with
  a durable reason once a real consumer was named (WP5b(f), replacing
  the Level 1 constant steering_ratio) -- the deciding question for
  whether reference data belongs in a config file is "what reads
  this", not "is it true" or "was it asked for".
- ADDENDUM [2026-07-23, same session]: added a `value_source` field
  ("setup_sheet" vs "logged_data") as a SIXTH, independent axis after
  first modelling tc_lat/tc_lon/abs_position as ordinary setup-sheet
  targets alongside ARB/dampers/springs -- wrong, because all three
  (like brake_bias, already correctly modelled this way) are changed
  by the driver mid-session from the steering wheel on feel, tyre
  life and fuel load; the setup sheet never holds their true value,
  a logged channel would. This is the same distinction the
  recommendation engine already draws between data and driver
  evidence sources (see "Data and driver as co-equal evidence
  sources" above) applied one level down, to the PARAMETERS
  themselves rather than the evidence about them: `value_source`
  answers "where does this parameter's truth live" and is
  deliberately orthogonal to `recommendation_target` ("should the
  engine suggest changing it") -- a parameter can be driver-owned
  and still worth recommending (tc_lat/tc_lon/abs_position all keep
  recommendation_target=true), because the two questions have
  different answers for different reasons: ownership is about WHO
  sets the value, recommendability is about whether the engine has
  anything useful to say about it.

### TC LAT for power-induced understeer - engineer rationale resolved [2026-07-25]
- Questionnaire S3-Med escalation "TC LAT +1" for apex understeer
  initially flagged as contradicting the stated convention (higher =
  less rotation). Resolved via load-transfer mechanism (engineer +
  project lead): earlier lateral-TC intervention cuts torque sooner,
  reducing acceleration squat and keeping load on the front axle -
  "more reserves on the front". The convention text describes the
  anti-rotation effect on an oversteering car; this use case is a
  pushing car under power.
- Rule-implementation caveat: valid for understeer WITH throttle
  involvement (apex-on-power, exit), not off-throttle push - phase
  detection can scope this via throttle/ax.

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
- ~~Werner MA full citation + exact method name for the CS reference.~~
  [RESOLVED 2026-07-24, primary source verified] Werner, F.: "Analyse
  des Fahrverhaltens eines autonomen Rennfahrzeugs anhand eines
  Giermomentdiagramms und echtzeitfähige Adaption an reale
  Fahrzeugsensordaten", MA thesis, Hochschule München, 2021
  (supervisors P. Pfeffer, HM; L. Hermansdorfer, TUM FTM), 122 pp.
- ~~Suzuka convention citation for c_beta sign.~~ [RESOLVED 2026-07-24]
  No citation existed to find -- the label itself was wrong; see the
  dated correction under "Yaw moment stability dMz/dbeta" in section 1.
- Confirm 992 GT3R official corner-weight/mass provenance for the
  constants table.
- [ADDED 2026-07-24] Verify page numbers for the estimate_sideslip
  citation (Mitschke/Wallentowitz, Dynamik der Kraftfahrzeuge,
  single-track lateral kinematics) -- currently cited as "p. TBD,
  verify" in the docstring pending access to the primary source.

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