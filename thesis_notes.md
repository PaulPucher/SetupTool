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

### Module 4b vs. the chair estimator: method-identical, not implementation-identical [2026-07-26]
- Line-by-line verification against `docs/literature/cornering_
  stiffness_estimator.py` (study document §7) found all method-
  defining parameters and mathematical operations equal (2 Hz BW4
  input filter, 0.02 rad adaptive window span, 0.021 rad linear-hold
  threshold, weighting orders 1 overall / 4 section-wise, the ratio
  construction itself) but the implementation independent: eight
  structural differences (windowing algorithm, concurrency, slope/R2
  computation path, filter-application module boundary, and four
  others, see study doc §7). Precise framing going forward: METHOD-
  IDENTICAL, not "source-verified identical" -- the latter overclaims
  what line-by-line code comparison actually supports.
- Two of the eight differences can alter outputs in edge regimes, not
  just code shape: the minimum-window-sample floor
  (`cs_min_window_samples=10`, config-documented Tier B hardening,
  no chair-side equivalent beyond the mathematical >1-sample minimum)
  and the `_smooth_weight` input-clipping SetupTool added on top of an
  otherwise identical weighting formula. Both classed NEUTRAL
  ENGINEERING / Tier B hardening, not method changes -- they change
  robustness at the margins, not what the method computes in the
  normal operating range.
- Also found during the same pass, unrelated to the identical-vs-
  independent framing: `cs_front_fallback_reference_n_per_rad`/
  `cs_rear_fallback_reference_n_per_rad` (config/parameters.json,
  commented as Milliken & Milliken RCVD GT3-slick fallback values for
  when OLS cannot resolve linear stiffness from data) are defined but
  consumed nowhere in `modules/` -- see PLAN.md open thread.

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

### Front/rear saturation and saddle-node concept anchors closed [2026-07-26]
- Hoffman, R.C., Stein, J.L., Louca, L.S., Huh, K. (2008), "Using the
  Milliken Moment Method and dynamic simulation to evaluate vehicle
  stability and controllability," Int. J. Vehicle Design, Vol. 48,
  Nos. 1/2, pp. 132-148 -- front-axle saturation as loss of
  directional control vs. rear-axle saturation as loss of stability,
  and the front/rear -> controllability/stability pairing, at p. 136,
  Section 2. Verified against the primary source by the reviewer,
  2026-07-26; added to the Module 5 docstring
  (`estimate_yaw_moment_stability`, `modules/stability_analysis.py`).
- Saddle-node bifurcation framing: Ono et al. (1998), cited after
  Hoffman et al. (2008, p. 136) until the primary Ono source is
  obtained directly. Cited as motivation for the stability-derivative
  framing only -- SetupTool implements no bifurcation analysis, no
  phase-plane computation, nowhere in `modules/`.

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
  magnitude despite the ~10x smaller absolute threshold, ~~because the
  new estimator's whole distribution is compressed by roughly the same
  factor~~ [SUPERSEDED 2026-07-26, study document §8d: compression is
  QUANTILE-DEPENDENT, not a single uniform factor applied to the whole
  distribution -- ~5x at the median (2676->561 Nm/deg) vs ~10x at p2.5
  (-554.1->-50); the tails compress more than the centre. The p2.5/
  threshold-level ~10x figure above is still correct as far as it
  goes; "the whole distribution... by roughly the same factor" is the
  part that overstated it] (see B1 report).
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
  (a) ~~yaw acceleration is differentiated from the raw yaw-rate channel
  only -- the chair function also accepts a pre-smoothed yaw-rate input
  from a chair-external filter list that is outside this project's
  reference scope, so only the raw-signal path is reproduced.~~
  [SUPERSEDED 2026-07-26, study document §8b, direct read of the chair
  file: the chair's function is not a raw-path-vs-one-pre-smoothed-
  alternative choice but a FOUR-TIER fallback chain (pre-smoothed
  accel column, pre-smoothed rate column, raw-but-precomputed accel
  column, then raw yaw-rate+time as the last resort). SetupTool's path
  is confirmed to be exactly the chair's own last-resort tier, not a
  simplification invented for this project -- see study doc §8b for
  the full chain and file:line citations.]
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
    remains a DOMAIN IMPROVEMENT: analysing a complete recorded session
    after the fact lets SetupTool identify and remove a lap known to
    carry a different (cold) tyre condition at the same track position
    other laps pool from -- a choice this post-session analysis has
    the opportunity to make, not a claim about what any other tool's
    own operating context does or doesn't allow.
- CLASSIFICATION (updated): the exclusion as a whole stays DOMAIN
  IMPROVEMENT (it is still a context-driven choice available to a
  post-session tool), but the outlap leg specifically is noted as
  FORCED in character -- excluding it isn't optional once the
  coordinate degeneracy is known, whereas the inlap leg is a genuine
  domain judgement call.

### B1 exclusion ablation numbers (from the B1 diagnostic report, 2026-07-24)
- With in/out-lap exclusion applied: 94.8% of valid samples stabilising.
  Without exclusion: 73.1%.
- CAVEAT, load-bearing: B2 (see "In/out-lap exclusion: two-leg
  rationale" above) attributed the without-exclusion run's negative
  tail overwhelmingly to the outlap coordinate artifact (the -97
  Nm/deg plateau, 96% outlap samples at one grid point) -- not to a
  genuine cold-tyre destabilising signal. This ablation therefore
  quantifies the DATA-VALIDITY leg of the exclusion rationale (the
  outlap's fabricated s-coordinate corrupting the local regression),
  not the cold-tyre/stationarity leg (the inlap's genuine but
  unquantified-by-this-number representativeness argument). Do not
  cite 73.1% as evidence that cold tyres are destabilising -- B2
  already showed the shortfall is dominated by a coordinate artifact,
  not a physical effect.

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
  raw-yaw-rate-only path = FORCED ADAPTATION ~~(the chair's pre-smoothed-
  input filter list is outside this reference's scope; no alternative
  input is available to us)~~ [SUPERSEDED 2026-07-26, see the dated
  correction above and study document §8b -- the classification itself
  (FORCED ADAPTATION) is unchanged; only the "filter list"/single-
  alternative parenthetical is imprecise, since the chair path is a
  four-tier fallback chain, not one alternative]. (b) the NaN-then-dropna masking wiring
  itself = NEUTRAL ENGINEERING (no science content, just how the three
  exclusions are fed into the chair's own missing-data handling). (c)
  structural in/out-lap exclusion = DOMAIN IMPROVEMENT overall
  [UPDATED 2026-07-24: see "In/out-lap exclusion: two-leg rationale"
  above -- the outlap leg is FORCED in character (s-coordinate
  degeneracy), the inlap leg is the original DOMAIN IMPROVEMENT
  (cold-tyre stationarity: a post-session tool can identify and
  discard a lap known to carry a different, cold-tyre condition at
  the same track position other laps pool from -- available because
  of what SetupTool is, post-session, not a claim about the chair
  pipeline's own operating context)].
  The s_m reset-guard interpolation above = NEUTRAL ENGINEERING.

### Moving-speed mask: domain-improvement classification [2026-07-24]
- `moving_speed_min_mps` (state["moving_mask"] = v_mps > threshold)
  excludes stationary/pit-lane samples from both Module 4b and Module
  5. The chair's reference estimator has no equivalent explicit speed
  gate; it relies on its own dropna handling for whatever is missing,
  not on excluding low-speed driving.
- CLASSIFICATION: DOMAIN IMPROVEMENT, same reasoning as the in/out-lap
  exclusion above: SetupTool's post-session analysis can identify and
  remove standing/pit-lane samples that carry no cornering information
  and would otherwise dilute the regression -- a decision available
  because of what SetupTool is (post-session, analysing a complete
  recorded file), not a correction of an error in the chair's own
  tooling, and not a claim about what the chair pipeline's own
  operating context does or doesn't require.

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
  IMPROVEMENT. A GT3 car rides kerbs every lap as part of the racing
  line; kerb transients violate the local regression's stationarity
  assumption, and the unmasked transient corrupts exactly the samples
  (apex, exit) that matter most to the regression -- exclusion is
  justified by our own data and physics rationale regardless of the
  chair pipeline's original context. The threshold-gate MECHANISM
  itself (deviation-from-baseline gate, dilation for ringdown) is
  standard Tier B practice, not the contribution; the contribution
  being claimed is the domain analysis (GT3 kerb usage pattern) and
  the decision to exclude, not the filter construction.

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

### Consistency-gate feedback override [2026-07-27]
- The per-lap consistency gate (`_consistency_gate_ok`, modules/
  recommendation.py) normally requires a "data"/"both"-trigger verdict
  to repeat on >= min_repeat_laps (2) AND >= min_repeat_fraction (0.4)
  of a corner's analysed laps before it can fire at all -- a genuine
  single-lap data flag, however severe, was structurally unable to
  produce a recommendation on its own, even when the driver's own
  feedback strongly corroborated it (see the PART A driver-level
  feedback-weighting work: scaling fb_value only ever modulates score/
  corroboration for an ALREADY-firing match, it cannot substitute for
  a verdict that never clears the gate in the first place -- found
  during the recommendation eligibility trace against the real Dubai
  outing's own driver feedback, where a single-lap understeer verdict
  at C9 was blocked purely by this gate, feedback aside).
- Override (project-lead-elicited 2026-07-27): a candidate passes the
  gate with an effective min_repeat_laps of 1 -- bypassing BOTH the
  laps floor AND the fraction check, not just the former, since a
  0.4-of-4-laps fraction would still reject a single repeat -- when
  BOTH raw |feedback| >= feedback_override_raw_min (4) AND scaled
  |feedback| >= feedback_override_scaled_min (4.0) (scaled = raw x the
  driver_level_weighting multiplier, PART A). Two independent floors,
  not one: the raw floor keys directly off the elicited driver
  feedback-scale semantics -- +-2..3 is "clearly felt", +-4..5 is
  "approaching undrivable" -- so raw>=4 specifically means the driver
  is reporting the car is nearly undrivable at that phase, not merely
  noticeable; the scaled floor exists independently to stop a LOW
  driving_level's raw complaint from piercing the gate on weighting
  alone (a raw-4 complaint from a level-1 driver, weight 0.6, scales
  to 2.4, well under 4.0, and is correctly refused the override).
- Rationale: a strong, unprompted driver complaint on a corner already
  showing a moderate+ data verdict is itself consistency evidence -- a
  capable driver will not provoke the same imbalance repeatedly across
  laps just to make the data repeat, so demanding multi-lap repetition
  on top of a near-undrivable complaint asks the data to double-
  confirm what the driver has already reported directly.
- Verified via synthetic checks (raw/level combinations against a
  controlled single-lap-repeat corner): raw 4 / level 10 (scaled 6.0)
  -> RECOMMENDED; raw 4 / level 2 (scaled 2.8, below the scaled floor)
  -> no override, normal gate correctly rejects; raw 2 / level 10
  (scaled 3.0, raw itself below the raw floor regardless of scaling)
  -> no override. Also confirmed against the real, persisted Dubai
  outing: the override does not change that outing's actual result
  (still zero recommendations) -- none of its data-flagged corners
  combine a qualifying feedback magnitude with a verdict that clears
  the earlier severity/verdict gates the consistency check sits
  behind.

### Repair turn: feedback encoding unification + severity floor [2026-07-27]
- CANONICAL DRIVER-FEEDBACK ENCODING, formalised (project-lead + reviewer
  decision, same day): signed-bipolar per the recorded scale already shown
  to the driver (ui/views/outing_form.py's feedback-table caption) --
  negative=understeer, positive=oversteer, magnitude bands roughly |2..3|
  "clearly felt", |4..5| "approaching undrivable"/"undrivable". Audited the
  full ruleset against this map (`modules/recommendation.py`
  `VERDICT_EXPECTED_FEEDBACK_SIGN`): all 26 non-retired rules and the 7
  retired seeds already had condition.verdict <-> condition.feedback_sign
  pairs agreeing with it -- no rule needed changing. The encoding itself
  was correct everywhere it had been applied; what needed fixing was one
  place it had NOT been applied at all (below).
- BUG FOUND AND FIXED: the consistency-gate feedback override (added the
  previous session) checked feedback MAGNITUDE only (`abs(raw_fb_value) >=
  feedback_override_raw_min`), never direction. A strong complaint in the
  WRONG direction -- e.g. +5 (oversteer-direction) on an understeer-verdict
  rule, or -5 on an oversteer-verdict rule -- would have cleared the
  override's magnitude floor and incorrectly bypassed the consistency gate
  regardless of whether the driver actually agreed with that rule's
  verdict. Fixed via `_override_direction_ok(verdict, raw_fb_value,
  raw_min)`: understeer needs `raw <= -raw_min`, oversteer needs
  `raw >= +raw_min`, any other verdict (unstable_yaw included -- no
  feedback-sign axis at all, same as `_feedback_modulation`) never
  qualifies. Verified directly: a synthetic oversteer-flagged corner with
  +5 feedback now fires via the override (RECOMMENDED, single-lap repeat
  that would otherwise fail the 2-lap floor); the same corner with -5
  (magnitude equally >=4, wrong direction) correctly does NOT fire --
  before this fix it would have.
- SEVERITY FLOOR FIX: the four held escalation rules under a CS-only
  (understeer) verdict -- US-BRK-low-esc, US-BRK-med-esc, US-BRK-high-esc,
  US-APX-med-esc -- carried `min_severity: "strong"`, structurally
  unreachable from their own single-phase evaluation: `_classify_corner`'s
  "strong" branch requires strong-CS AND destabilising yaw on the SAME
  restricted phase these rules test, but these rules exist purely to
  escalate a WORSENING CS-only reading, unrelated to yaw instability.
  Corrected to `min_severity: "moderate"` -- the advisory/recommended
  split and provenance caps (settings.action_class) are the intended
  proportionality mechanism, not an accidentally-unreachable severity
  gate. No behavioural change today (all four are status="held", which
  already excludes them from firing) -- this closes a landmine that would
  otherwise have made all four permanently inert even after a future WP
  promotes them out of "held".
- VERIFIED END-TO-END, the user's exact scenario: C12, driving_level=10,
  real persisted feedback (-5 on every phase) -- `generate_recommendations`
  still returns zero results for this outing, confirmed against the real
  data, not assumed. The reason is unchanged by either fix above: C12's
  aggregate severity is "normal" on every phase (moderate-band front CS at
  apex, CSf=0.122, without accompanying destabilising yaw -- the AND-logic
  never promotes past "normal" on CS-alone), so every candidate rule fails
  at `severity_ok` before the code ever reaches the consistency gate where
  the (now-corrected) override lives. Neither fix could have changed this
  outcome; the blocker is upstream of both.
- FINDING, verified and stated (not silently changed): the assumption that
  the standard corroboration floor (`condition.min_feedback_abs`) sits at
  >=2, keeping the "benign"/"slight" |1| zone from ever corroborating, is
  WRONG for the current config -- every matrix rule's `min_feedback_abs`
  is 1, and `_feedback_modulation`'s check is `abs(fb_value) < min_abs`
  (strict `<`), so a magnitude-exactly-1 complaint ("slight" on the
  recorded scale) DOES count as full corroboration today (confirmed:
  `abs(-1)=1 >= 1` clears the floor, `agreement_bonus` applied). This is
  independent of the override (which has its own, much higher floor of 4
  and is unaffected) -- it is the ORIGINAL WP2b-2 corroboration path. Not
  changed here: raising `min_feedback_abs` is a calibration decision, not
  made unilaterally without being asked. Flagged for a decision.

### Undrivable tier: lap-level cell matching [2026-07-28]
- BUG FOUND: the undrivable-feedback tier (`_apply_undrivable_escalation`,
  design ruling 2026-07-28) checked the AGGREGATED (median-of-medians)
  corner verdict at only the single phase the driver's worst-magnitude
  feedback happened to name, then looked for a pre-built bucket agreeing
  with it. For the real persisted Dubai outing's C12 (`x4=-5`, feedback
  named exit_4) this produced a spurious "no elicited rule covers this
  case" gap row, even though C12 shows a genuine, repeating moderate-
  understeer pattern (2 of 4 laps, lap1/lap4) at apex_3 -- a DIFFERENT
  phase, where recorded feedback is 0. Two separate effects compounded:
  (1) the aggregate at apex_3 also dilutes that 2-of-4-laps pattern down
  to "normal" by itself (the median-of-medians is doing exactly its
  designed job of privileging repeatable behaviour -- it is simply the
  wrong statistic for the undrivable tier's own "never render silent
  emptiness" requirement); (2) the check never even looked at apex_3 at
  all, being pinned to exit_4 by construction. Verified read-only before
  any code changed (diagnostics/inspect_urgent_tier_lap_level_verify.py):
  per-lap classify_fn verdicts printed for all 5 phases, confirming
  exit_4 never reaches moderate severity on ANY lap or in aggregate (so
  even a same-phase lap-level check would not have found it), while
  apex_3 does, on exactly 2 of 4 laps.
- RULING (project-lead + reviewer, same session): cell matching for this
  tier now runs against LAP-LEVEL verdict instances, searched across
  EVERY non-retired data/both rule whose condition could plausibly cover
  the corner's implied direction and speed_class (`_candidate_rules_for_
  verdict`), not just the phase the feedback named. For each candidate,
  `_qualifying_laps_for_rule` checks every analysed lap against that
  rule's OWN min_severity/verdict on its OWN phases. If a qualifying lap
  exists and the rule already produced a real match against the
  aggregate, that bucket is pierced (unchanged behaviour, generalised to
  search every rule rather than one phase). If the rule did NOT produce a
  real match (the aggregate diluted it, as at C12/apex_3), a second
  architecture question followed: how does the resulting URGENT-
  RECOMMENDED row get a real setup action rather than a placeholder? The
  ruling: RE-EVALUATE the rule through the exact same `_evaluate_rule`/
  bucket-construction path every other rule uses (`_add_rule_matches_to_
  buckets`, factored out of the main loop for this reuse), substituting
  the qualifying lap's own real (unaggregated) phase data for the rule's
  phases in place of the aggregate -- `by_corner_laps` itself stays
  untouched, so the rule's own consistency gate still checks genuine
  repetition (or the feedback-override path, on its own real per-phase
  feedback value) rather than being bypassed. This means exactly one code
  path builds every action row in the engine; the undrivable tier is a
  different ENTRY CONDITION into that path (lap-level trigger instead of
  the aggregate), not a parallel renderer. The scaled-feedback floor
  (`feedback_override_scaled_min`) still gates whether a found match gets
  pierced to URGENT, same double-floor discipline as before; contradiction
  detection (opposite-axle lap-level evidence, no matching-direction
  evidence anywhere) and the true-gap fallback (no evidence in either
  direction anywhere) both moved to the same lap-level, all-phase search,
  keeping the three-way (pierce/contradiction/gap) outcome structure
  unchanged.
- VERIFIED against the real, persisted Dubai outing after implementing
  (diagnostics/inspect_urgent_tier_lap_level_fix_check.py): three corners
  carry feedback clearing the tier's raw_min floor (C8, C9, C12). C12 now
  renders `action_class=recommended`, `urgent=True`, actions = soften
  front ARB (both sides, US-APX-low), rationale including "C12: 2 of 4
  laps show understeer @ apex -- driver reports near-undrivable
  (understeer)" -- matching the ruling's own worked example verbatim,
  confirming the fix against real data, not a synthetic scenario. C9 (a
  second real corner with qualifying feedback, unrelated to this fix's
  design) now also fires two urgent-recommended rows: one via genuine
  2-of-4-lap repetition (TC LON increase), one via only 1 of 4 laps,
  reachable only because that rule's OWN phase (apex_3) carries real
  feedback (+5) clearing the consistency-gate's feedback-override floor
  directly -- confirming the substituted-corner re-fire correctly
  inherits the existing override mechanism rather than needing a second,
  parallel one. The OS-APX-low situational cell (permanently-advisory by
  design, matrix v2 review) correctly renders RECOMMENDED here, since
  piercing bypasses the situational cap exactly as it already did for
  normal pierces -- no special-casing needed. C8 -- genuinely no lap-
  level evidence in either direction, anywhere, for its speed_class --
  still renders the unchanged "no elicited rule covers this case" gap
  row, confirming the true-gap path is untouched. `test_stability.py`
  unaffected (this fix touches only modules/recommendation.py and
  config/recommendations.json, neither of which Modules 1-6 read).
- Config: no new tunables -- reuses settings.consistency_gate.
  feedback_override's existing raw_min/scaled_min pair, documented in
  config/recommendations.json's `_comment_escalation_enabled`.

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

### WP5b(b) phase 1: chair-parity vertical loads (Fz) [2026-07-26]
- Construction, chair-identical (docs/literature/data_handler.py:1548-
  1621, internal, adopted as-is, no deviation): axle-level Fz_f/Fz_r
  from a static weight split plus an aero term plus longitudinal load
  transfer (m*ax*h_cog/wheelbase); per-wheel Fz_fl/fr/rl/rr from an
  INDEPENDENT-per-axle lateral-transfer split (m*ay*h_cog/track_width,
  applied separately front and rear). Roll-stiffness apportionment
  (splitting total lateral transfer by actual front/rear roll
  stiffness rather than independently per axle) is EXPLICITLY DEFERRED
  to phase 2, damper-validated -- not built speculatively here. Tier A:
  Milliken & Milliken, RCVD, load-transfer chapter, p. TBD verify (same
  pending-citation pattern as the Fy yaw-moment term and
  estimate_sideslip).
- EMPIRICAL SIGN-CONVENTION CHECKS, both run against real Dubai data,
  neither assumed -- the same discipline the kerb-detection az
  discovery established (a wrong sign assumption there flagged 100% of
  samples before the real z-down convention was found empirically):
  - ax (longitudinal-transfer direction): heavy-braking samples
    (log_pbrake_f > p90) give median ax_mps2 = -11.79; heavy-
    acceleration samples give +3.97. The chair's formula
    (dfz_long_transfer_N = m*ax*h_cog/wb, fz_f_N = static_f + aero_f -
    transfer) needs ax negative under braking to load the front --
    MATCH, no sign flip needed.
  - ay (left/right loading): the corner with the largest |apex_
    lateral_g| in the file was independently verified as a genuine
    right-hander via a GPS-trace heading-change check (net -101.9 deg,
    a ground-truth measure from the vehicle's own path, independent of
    any onboard sensor's sign convention) -- mean ay_mps2 over that
    corner's window was -6.75, which per the chair's fz_fl/fz_fr
    formula loads the LEFT (physically-correct outside) tire in a
    right turn -- MATCH, no sign flip needed.
  - Both checks could have failed (as the az kerb-baseline assumption
    once did); they didn't, but the result is evidence, not something
    assumed correct because the formula "looked right."
- PLACEHOLDER PROVENANCE, every one explicitly reviewer-supplied and
  NOT sourced from a team figure: cog_height_m=0.30, track_width_
  front_m=1.66, track_width_rear_m=1.64 (config/parameters.json notes
  read "reviewer-supplied order-of-magnitude placeholder, NOT sourced,
  replace with team figure"); aero.lift_coeff (Cl) = 0.0, with a
  documented physical zero-meaning (the aero term evaluates to exactly
  zero at every speed -- Fz reduces to the static+longitudinal-only
  estimate, a known underestimate of axle load at speed, not a
  front/rear balance error). The Cl-NEGATIVE-for-downforce sign
  convention (inferred from the chair's own formula+comment pairing,
  fz_aero_total_N = -0.5*rho*v^2*A*Cl commented "positive for
  downforce") is flagged in config as UNCONFIRMED -- no numeric Cl
  default exists in the shared reference files to check against
  directly, so the config note requires the first real Cl entry to be
  validated empirically (Fz rising with v^2, not falling) before it is
  trusted, the same "verify, don't assume" standard as the ax/ay checks
  above.
- TURN (b), consumer wiring: estimate_vertical_loads joins
  StabilityAnalysisThread's pipeline and the WP6 pipeline-cache
  identity; summarise_corners gained an additive-only fz= parameter
  (fz_f_N/fz_r_N/fy_f_norm_N/fy_r_norm_N per-phase stat blocks).
  fy_f_norm_N/fy_r_norm_N = Fy_filt/fz axle is the chair's OWN
  normalised-force construction (data_handler.py:1619-1621) -- the
  actual named consumer Fz was built for, distinct from and not
  replacing CS_ratio (Module 4b's Calpha-ratio metric). ADDITIVE-ONLY
  PROOF: a before/after diff of one corner's summary dict showed
  exactly 20 new keys (4 fields x 5 phases) added and ZERO pre-existing
  values changed. ANALYSIS_SCHEMA_VERSION bumped 2->3 for the payload-
  shape change. Nothing feeds _classify_corner -- read-only diagnostic
  throughout, per-phase Fz medians surfaced in the corner-details UI
  table only (fy_norm computed but not yet displayed, deferred, does
  not fit the panel width cleanly alongside CSf/CSr/Stab).

### GPS-course sideslip (beta_gps, WP5b(c)) attempted and shelved [2026-07-26]
- Goal: a Level-3 sideslip candidate, beta = course-over-ground minus
  vehicle heading (GPS-aided kinematic sideslip estimation family;
  primary source still to verify), reconstructing heading by
  integrating yaw_rate_radps and periodically re-anchoring the drift
  to log_gps_course at trustworthy low-slip (near-straight) samples.
  Whitelisted log_gps_course (config/channels.json, 10 Hz, confirmed
  live on Dubai); new isolated sibling function
  `estimate_sideslip_gps` (modules/stability_analysis.py) -- VALIDATION
  ONLY throughout both iterations, never called from any pipeline/UI
  path, zero effect on production beta (`estimate_sideslip`, kinematic
  integration + washout) or on `test_stability.py` (byte-identical
  after every step, both iterations).
- Rotation-convention finding (empirical, not assumed): log_gps_course
  is a compass bearing (clockwise-positive from North); correlating
  wrap-safe d(course)/dt against yaw_rate_radps over racing laps gives
  r=-0.9548 for +yaw_rate and r=+0.9548 for -yaw_rate, so
  -yaw_rate_radps was adopted. This does NOT follow from the z-down
  accelerometer convention already established for kerb detection --
  tested independently, a different sensor axis can have a different
  convention. Same check doubled as a latency probe: cross-correlation
  peaks at r=0.9898 at lag=+0.32s (course lags yaw_rate), vs r=0.9575
  at zero lag -- a real, measurable GPS pipeline delay.
- ITERATION 1 (time-linear drift allocation, latency uncorrected):
  correlation r(beta_gps, beta_kinematic)=-0.12; per-corner sign
  agreement 130/255=51.0% (barely above chance, though still better in
  absolute count than the earlier log_a_car probe's 1-of-3); straight-
  line near-zero check: kinematic 0.40deg vs gps 1.47deg; the
  Amendment-2 lever-arm probe (regressing beta_gps-beta_kinematic
  against yaw_rate/v) returned an antenna-offset slope of 9.34m --
  physically impossible for a GT3 car, signalling a construction
  problem rather than a real finding.
- ROOT-CAUSE DIAGNOSIS (falsifiable, not just descriptive): a
  closed-loop per-lap check (net gyro-integrated heading change vs net
  GPS-course change over one lap) found a consistent ~6deg/lap
  shortfall (~1.7% of the ~354-360deg net rotation) across all 4 laps
  -- a small, systematic gyro-integration scale-type error, not
  random noise. Because this drift accumulates in proportion to
  ROTATION (concentrated in the ~15 corners/lap) rather than elapsed
  time, iteration 1's time-linear interpolation of the anchor
  correction under-corrected exactly during cornering (where beta is
  measured) and over-corrected on the straights between anchors --
  diagnosed as the specific cause of the large, poorly-correlated
  errors above.
- ITERATION 2 (two targeted fixes, same anchors, no new machinery):
  (1) the drift correction is now interpolated in proportion to
  accumulated |yaw_rate| integral between anchors (a monotonic
  "rotation clock" substituted for the time axis in the same
  np.interp call) instead of elapsed time; (2) course is now sampled
  gps_course_latency_s=0.32s (config, derived_from the cross-
  correlation evidence above) ahead of each query time before
  anchoring/subtraction, correcting the measured GPS latency.
  FALSIFIABLE CHECK ON THE DIAGNOSIS: the lever-arm probe was
  re-run unchanged and the phantom antenna offset shrank from 9.34m
  to 1.325m (an 86% reduction, now in a physically plausible range)
  -- the diagnosis's own prediction held. The long-corner washout
  probe also improved qualitatively: beta_gps now decays toward zero
  through a long corner (as beta_kinematic's washout also does),
  where iteration 1 showed it growing instead.
  DECISION-CRITERIA METRICS DID NOT MATERIALLY IMPROVE, however:
  correlation r=-0.24 (iteration 1: -0.12, no better); per-corner sign
  agreement 133/255=52.2% (iteration 1: 51.0%, essentially flat);
  straight-line bias unchanged in character (gps still ~4x worse than
  kinematic).
- VERDICT: STILL NOT MET. This line is SHELVED -- both iterations
  documented here. beta_gps as constructed is not a usable Level-3
  candidate; production beta stays kinematic, no consumer touched at
  any point. The diagnosis is corroborated (a passed falsifiable
  check is stronger evidence than a plausible-sounding explanation
  alone would have been) but the identified drift mechanism was not
  the dominant error source for the headline metrics -- a further
  attempt would need to address something beyond allocation scheme
  and latency, most plausibly the anchor count itself (6 total across
  one 4-lap session is a data-availability limit of this specific
  file, not obviously fixable by construction alone). Full validation
  reports for both iterations: diagnostics/inspect_beta_gps_
  validation.py (script always reflects the current/iteration-2
  construction; iteration-1 numbers are quoted here and in the
  script's own before/after comparison, not separately reproducible
  without reverting the two fixes).
- RE-BASELINE NOTE [2026-08-18, WP-S2 sanity-gate re-run]: the
  per-corner sign-agreement figure above (133/255=52.2%) was frozen
  BEFORE commit 0bdff87 ("lap segmentation big improve", 2026-07-26)
  landed WP1's canonical corner realization, which moved corner-phase
  boundaries. Re-running the unchanged original
  inspect_beta_gps_validation.py script against the current
  realization now prints 127/257=49.4% -- confirmed by direct
  re-execution, not recomputed differently. The old number was
  correct FOR ITS OWN realization at the time it was recorded; it is
  not wrong, only realization-dependent, and is superseded here by
  the current canonical realization's figure rather than struck. r
  is unchanged (-0.24, matches within rounding on re-run). Shelving
  VERDICT is unaffected: both 52.2% and 49.4% are barely-above-chance
  per-corner sign agreement, and the diagnosis (weak/near-chance
  cross-method agreement, not a usable Level-3 candidate) does not
  depend on which of the two numbers is read. diagnostics/
  inspect_sideslip_methods_comparison.py's Metric 2 sanity gate now
  checks against 127/257=49.4%, with the same re-baseline reasoning
  in its own comment.

### Small-decisions sweep [2026-07-26]
- cs fallback reference constants deleted (small-decisions sweep):
  `cs_front_fallback_reference_n_per_rad`/`cs_rear_fallback_reference_
  n_per_rad` removed from config/parameters.json -- defined, commented,
  consumed nowhere, verified 2026-07-26. The no-linear-reference case
  has never occurred on real data; if it ever does, the corner reports
  invalid, more honest than silently filling from an unvalidated
  constant.
- [2026-08-18] Clarifying note on the deletion above: the sideslip
  methods-comparison observer framework (Open Board item B,
  circularity option 2 -- beta-independent tyre-stiffness prior, WP-S1)
  will RE-introduce `cs_front_fallback_reference_n_per_rad`/
  `cs_rear_fallback_reference_n_per_rad` into config/parameters.json,
  this time with a named consumer (the prior itself). This supersedes
  the deletion's rationale going forward without contradicting it --
  the constants were correctly removed on 2026-07-26 for the reason
  given then ("consumed nowhere"); a new consumer arriving later is a
  new decision, not evidence the deletion was wrong. Re-introduction
  happens under item B's own review and re-derivation stop, not taken
  here -- no config change made by this note.
- s_m short-circuit documented as deliberate (small-decisions sweep):
  the chair's time-anchored fallback mode for when s_m is unusable is
  NOT ported -- `estimate_yaw_moment_stability`'s docstring now states
  why: the fallback is a differently-behaving estimator (time-local,
  no cross-lap pooling) whose output the s-grid-derived classification
  thresholds could not classify meaningfully. No stability verdict is
  more honest than a silently degraded one.
- corner_analysis.py:359 reset-guard fix (small-decisions sweep):
  apex_lap_distance_m now routes through the shared
  `_interp_lap_distance_guarded` helper (modules/stability_analysis.py)
  instead of plain `np.interp`, closing the lap-boundary-reset gap
  found 2026-07-25. WP1-freeze proof (diagnostics/inspect_wp1_reset_
  guard_freeze_proof.py): per-lap corner counts, every apex_lap_
  distance_m (10 dp), and every stable_corner_id byte-identical
  before/after on Dubai -- confirms the fix is a genuine no-op on this
  file, as expected (no apex sits near a reset). `bracket_start_m`/
  `bracket_end_m` stay unguarded, explicitly out of scope.

### WP5b(d): GPS speed cross-validation (validation only) [2026-07-26]
- Whitelisted `log_gps_speed` (config/channels.json, 10 Hz, confirmed
  live on Dubai, kph). Kept channels-direct and isolated
  (diagnostics/inspect_gps_speed_validation.py reads the raw channel
  itself, same pattern as `estimate_sideslip_gps`, WP5b(c)) -- no new
  field on `prepare_vehicle_state`'s shared state dict, since this
  comparison has exactly one reader. `ecu_speed` stays the production
  speed source and the pipeline's own time-anchor (`t_ref`) throughout;
  zero call sites into any Module 2-5 function, verified by grep;
  `test_stability.py` byte-identical.
- LATENCY: cross-correlating GPS speed against ecu_speed (racing laps,
  moving samples) peaks at lag=+0.320s, r=0.9997 (r=0.9926 at zero
  lag) -- within 2 samples of `gps_course_latency_s` (0.32s, WP5b(c)).
  Reused that key rather than adding a new one -- the "same GPS
  pipeline, same latency" plausibility argument from the proposal held
  up empirically, not just assumed.
- SCALE FACTOR: origin-regression k (v_gps = k*v_ecu) = 1.01211 --
  ecu_speed reads ~1.2% low vs GPS. Consistent across speed classes
  (k=1.0156 low / 1.0099 medium / 1.0128 high, range 0.0057, whole-
  session k=1.01211) -- a tight, condition-independent ratio, the
  signature of a constant rolling-radius/calibration offset rather
  than a speed-dependent effect. Residual spread tightens sharply once
  k is applied: raw (v_gps-v_ecu) median +0.506 m/s, std 0.383, MAD
  0.272; post-k median +0.002 m/s (near-zero), std 0.319, MAD 0.186 --
  the scale correction removes almost the entire systematic offset.
- SLIP PREDICTION, more carefully than the diagnostic script's own
  crude zero-relative CONFIRMED/REFUTED labels suggest: read against
  the whole-session baseline (+0.506 m/s), not against zero. Heavy-
  braking median +0.417 m/s is SMALLER than baseline (closer to
  agreement, opposite of the wheel-lock-makes-it-worse expectation);
  heavy-traction median +0.706 m/s is LARGER than baseline, but in the
  SAME direction (ecu still reads low) rather than flipping negative
  as wheelspin would predict (wheelspin should make ecu read HIGH vs
  GPS). Neither regime shows the classic sign-flip signature of wheel-
  speed slip corruption; kerb samples show no meaningful difference
  either (on-kerb median +0.494 vs off-kerb +0.506 m/s). Most
  consistent explanation: this GT3 car's traction/ABS systems keep
  slip small enough on this session that it isn't the dominant driver
  of the ecu/GPS gap -- strengthening, not weakening, the case that k
  is a genuine constant calibration factor rather than a slip
  artifact.
- DECISION: (b) -- keep GPS speed as a permanent cross-check; k=1.012
  is a strong, well-evidenced candidate rolling-radius correction to
  `ecu_speed`'s own conversion, IMPROVING THE EXISTING L1 CONSTANT IN
  PLACE rather than switching sources. Not (a): the prior from WP5b(c)
  (beta_gps shelved on this exact GPS receiver/logger) was not
  contradicted here, but GPS speed itself looks materially cleaner
  than GPS course did (r=0.9997 vs beta_gps's r=-0.24) -- speed is a
  scalar, geometrically much simpler than a drift-integrated heading,
  so this is not a surprise, just a difference worth naming. Not (c):
  nothing here looks troubled enough to shelve.
  CORRECTION carried through from the proposal review: measuring k is
  NOT an estimator-input change (pure diagnostic); APPLYING it to
  ecu_speed's conversion WOULD be one -- v_mps feeds Modules 2-5 and
  the WP5b(b) aero-Fz term, so applying the correction triggers the
  standing re-derivation stop and is a separate, not-yet-taken future
  decision. accuracy_levels.speed's registry note (config/
  parameters.json) states this distinction explicitly.

### Wheel-speed source characterization (WP-S1) [2026-08-18]
- Report-only channel-quality diagnostic (diagnostics/inspect_wheel_
  speed_sources.py), Tier B, standard signal-QA technique (NaN/frozen/
  dropout fractions, straight-line agreement vs production ecu_speed,
  L/R consistency). Nothing whitelisted, nothing consumed by any
  Module 2-5 path; ecu_speed stays the production speed source.
  Motivation: the raw Dubai log carries four redundant wheel-speed
  channel families, none whitelisted -- worth characterizing once
  before any of them becomes a consumer, same "establish the ground
  truth before building on it" pattern as WP5b(d)'s GPS-speed
  cross-validation above.
- TWO INDEPENDENT FAMILIES, not four: log_speed_* and ecu_speed_
  wheels_* are byte-identical (same time base, max abs diff = 0.0,
  Section 2). abs_speed_* (a fixed-ratio mph rescale of Team_nWheel*,
  confirmed by matching relative-deviation and L/R-spread numbers
  across the two Section 2/3) is the second family. Two channel
  families on the wire, not four independently-sourced ones.
- FRONT/REAR SPLIT, not a uniform scale offset: on straight-line
  samples (moving & valid-lap & |ay|<=0.15g & |yaw rate|<=3.0 deg/s,
  n=5171), front axle (log_speed_fl/fr) tracks production ecu_speed
  at median -0.03% (essentially exact agreement); rear axle
  (log_speed_rl/rr) reads +1.41% high. The rear offset is CONSTANT
  and throttle-independent: median +1.44% under throttle-on
  (ecu_aps>20%) vs +1.41% across all straight-line samples -- a
  driven-axle traction-slip signature would shrink or reverse off-
  throttle; it does not move. Diagnosis: a rolling-radius difference
  (rear tyre circumference vs the ECU's per-wheel radius constant),
  not traction slip.
- BRAKING-ONLY FRONT EFFECT, separate from the above: under braking
  (log_pbrake_f>5bar, n=643) front reads -1.38% (vs -0.03% off-
  braking) while rear stays flat (-0.09%, matching its all-condition
  baseline). This is the classic signature of front-wheel slip under
  braking (this car carries more front brake bias), layered on top of
  the constant rear rolling-radius offset -- two distinct effects on
  two distinct channels, not one confound.
- CROSS-REFERENCE: the rolling-radius-difference reading here is
  corroborated, not duplicated, by WP5b(d)'s GPS-speed finding above
  (k=1.01211, ecu_speed reads ~1.2% low vs GPS, condition-independent
  across speed classes) -- both point independently to a rolling-
  radius/calibration-scale story rather than a slip artifact, from two
  unrelated measurement chains (wheel-speed cross-family comparison
  here vs GPS-speed comparison there).
- DESIGNATION: log_speed_* is the designated candidate family for any
  future wheel-speed consumer (byte-identical to ecu_speed_wheels_*,
  the cleaner of the two families on median deviation and L/R std).
  No config whitelist change made here -- per the project's no-
  consumer convention, whitelisting is deferred until a concrete
  consumer exists; this entry documents the characterization work so
  that future consumer does not have to repeat it.

### Zero-slip offset: chain decomposition + mechanism search (WP-S3b/S3c) [2026-08-18]
- Follow-up to Metric 5's direction-locked zero-slip Fy offset (WP-S3,
  diagnostics/inspect_sideslip_methods_comparison.py), which persists at
  comparable magnitude under both the kinematic and GPS-course beta
  candidates despite the two being nearly uncorrelated (r=-0.24). WP-S3b/
  S3c (diagnostics/inspect_offset_chain_decomposition.py, diagnostics/
  inspect_washout_mechanism.py) traced the effect to its construction and
  tested candidate mechanisms.
- REFRAMING: the near-zero-alpha condition at high ay is a GEOMETRIC
  CANCELLATION in the slip-angle construction itself, not evidence of an
  Fy-side offset per se. At the rear, alpha_r's small-angle form is
  yaw_geom_r - beta; near-zero alpha_r means beta and the yaw-rate
  geometry term (lr*psidot/v) are nearly equal (confirmed: mean |median
  beta|=0.625 deg vs mean |median yaw_geom_r|=0.612 deg across 14
  corners, ratio 1.02). At the front, alpha_f's small-angle form is
  delta_f - (beta + yaw_geom_f); near-zero alpha_f means steering is
  matched by the sum of the other two terms (mean |median delta_f|=1.560
  deg vs mean |median (beta+yaw_geom_f)|=1.559 deg across 13 corners).
  Both hold while ay stays large throughout (mean |ay|~10 m/s^2) -- so
  the correct framing is that this is an ALPHA-ERROR proxy (the
  estimated slip angle reads near zero when it should not), not
  primarily an Fy-construction artifact; Fy_f/Fy_r (estimate_lateral_
  forces) do not depend on beta at all, so the Fy side was never the
  natural place to look for the root cause.
- FORCE-BALANCE CHECK quantifies the same alpha-error directly, in slip-
  angle terms rather than force terms: alpha_r_ss = Fy_r_needed / Cr,
  Fy_r_needed = m*ay*lf/L (steady-state 2-DOF moment balance, Milliken &
  Milliken RCVD, diagnostic use only, not implemented), Cr = the current
  linear-reference cornering stiffness (C_linear_ref_r) at the same
  samples. Cr's own numerator/denominator are alpha_r/Fy_r-derived --
  circular for an independent magnitude claim, though the underlying OLS
  slope naturally demeans a roughly-constant per-window Fy offset, which
  limits (does not remove) that circularity; treated as informative for
  order-of-magnitude and sign only. Result: every one of 14 corners
  demands 0.9-5.8 deg of steady-state rear slip to support the observed
  ay via Cr, while the estimated alpha_r reads ~0 deg at those same
  samples -- a large, systematic under-read, always signed with ay (same
  direction-lock Metric 5 found on the Fy side).
- MECHANISMS TESTED:
  - IMU longitudinal lever-arm (ay_imu = ay_cog + x_imu*psiddot, standard
    rigid-body kinematics, diagnostic fit only): position is UNKNOWN
    in-repo (config/parameters.json's accuracy_levels.lateral_acc
    capped_by note and limitations register item 5 record only the
    assumption "accelerometer at CoG", no measured or estimated offset
    anywhere). A diagnostic (non-calibration) least-squares fit of the
    near-zero-slip Fy offset against mass*fraction*psidd returned
    x_imu=-8.42 m (front) / -8.71 m (rear), R^2=0.087-0.123 -- over 3x
    this car's 2.505 m wheelbase and a weak fit. REJECTED as the
    dominant mechanism.
  - Static front/rear weight-split mis-apportionment (Fy_f = m*ay*
    front_fraction + ..., Level 1 static split, thesis_notes.md
    limitations register item 1): REJECTED on physics grounds (reviewer
    decision, 2026-08-18) -- an incorrect static/dynamic split
    apportions a given total Fy_total=m*ay between axles differently,
    but cannot make either axle's OWN Fy read near zero while that
    axle is genuinely carrying several kN of load; mis-apportionment
    changes the SPLIT, not whether an axle's force is offset from zero
    at its own zero-slip point. Not tested computationally here --
    excluded on the force-balance argument alone.
  - Washout ablation (per-corner beta re-anchored to zero at the last
    straight-line sample before corner entry, raw beta_dot = ay/v -
    yaw_rate integrated with the 0.05 Hz high-pass removed): UNINFORMATIVE,
    neither confirms nor refutes. Median beta shifted substantially per
    corner (1-6.5 deg) when the washout was removed, and the near-zero-
    alpha_r sample count thinned in aggregate (1965 -> 1299), but not
    consistently per corner (several corners' counts grew instead), and
    the direction-locked Fy offset did not shrink toward zero under
    re-anchoring -- it flipped sign and stayed comparably large (+6197 N
    -> -4684 N globally). The re-anchored construction's own empirical
    residual drift (measured at the first straight-line sample after
    corner exit, where true beta should read ~0: median 5.7 deg, mean
    6.1 deg, p90 10.8 deg, max 14.3 deg over a single corner's span) is
    comparable to or larger than the effect under test, so its near-
    zero-slip selection cannot be trusted as clean ground truth. This
    result is also consistent with (not merely compatible with) Metric
    5's cross-beta-source finding: GPS-course beta uses a completely
    different, non-washout drift-correction mechanism (periodic
    re-anchoring to log_gps_course) and still shows a comparable
    direction-locked offset, so a washout-specific explanation was
    already in tension with that result before this ablation ran.
- STANDING SYNTHESIS: the kinematic sideslip estimate lacks any low-
  frequency truth source for steady-state sideslip -- the washout
  high-pass strips steady-state content by construction (that is its
  job, correcting integration drift), and the GPS-course candidate's
  anchor density was too sparse to supply it instead (its own shelving
  record, WP5b(c) above: 6 anchors across one 4-lap session). This is
  consistent with the offset persisting under both candidates (Metric
  5) rather than being specific to either beta construction. WP-S4's
  observer target is therefore explicit, not open-ended: recover
  steady-state sideslip via a model-based (force-balance) correction
  path, and its success should be judged primarily against Metric 5 (the
  direction-locked Fy offset) and this entry's Section-2 steady-state
  gap, not against correlation with the kinematic candidate alone (which
  shares the same blind spot).
- SEPARATE DATA POINT, not folded into the mechanism verdicts above: the
  raw-integration residual-drift finding (median 5.7 deg over a single
  corner's span, from beta_dot = ay/v - yaw_rate integrated with no
  high-pass) shows beta_dot itself carries a systematic bias large
  enough to matter at single-corner timescales. Worth revisiting once
  the WP5b(d) GPS-speed scale question (k=1.01211) and the ay-provenance
  question (WP-S3b's IMU-lever-arm/weight-split threads) settle, since
  either an uncorrected speed scale or an ay bias would feed directly
  into beta_dot's own construction.

### WP-S4: Kalman sideslip observer (diagnostics candidate) [2026-08-19]
- METHOD: discrete-time linear Kalman filter on the linear single-track
  (bicycle) model in yaw-rate/slip-angle state-space form. States
  x = [beta, yaw_rate]. Input u = delta_f (front steering angle, existing
  Level-4 steering_ratio path). Measurements z = [yaw_rate, ay]
  (sclu_yaw_rate, log_acc_y). Speed v enters as a time-varying model
  parameter (production ecu_speed), not a state. Standard time-update/
  measurement-update recursion; first-order (Euler) discretization of the
  continuous-time A/B matrices at each sample's own Vx (Tier B numerical
  simplification, not a modelling claim). Diagnostics-only: diagnostics/
  sideslip_kalman_observer.py, pure function, no Qt, no production wiring,
  registered as the third CANDIDATES entry (C_kalman_observer) in
  diagnostics/inspect_sideslip_methods_comparison.py.
- Q/R: named constants in the observer script itself (Q_BETA_VAR,
  Q_YAW_RATE_VAR, R_YAW_RATE_VAR, R_AY_VAR, plus P0_BETA_VAR/
  P0_YAW_RATE_VAR for the post-stationary reset covariance), each with a
  one-line comment stating they are hand-tuned initial values -- no
  ground-truth beta, yaw-rate-error, or ay-error signal exists on this
  dataset to tune against.
- Iz PLACEHOLDER: vehicle.yaw_inertia_kalman_kgm2 = 1800.0 kg m^2,
  config/parameters.json, "NOT sourced, reviewer placeholder, replace
  with team figure" (same convention as cog_height_m). LOCATION NOTE:
  the original work order specified config/car.json's vehicle section;
  car.json has no vehicle section (it is per-session setup data only,
  loaded via core.config_loader, structurally distinct from parameters.
  json's vehicle-constants section that already holds cog_height_m and
  the production yaw_inertia_kgm2). Resolved (user decision, 2026-08-19)
  to use parameters.json's existing vehicle section instead, under a
  distinct key (yaw_inertia_kalman_kgm2, not yaw_inertia_kgm2) so this
  diagnostic's placeholder cannot silently collide with or shadow the
  production Iz value (2082.0, Modules 4a/5's m*a*b estimate). A matching
  accuracy_levels registry node (yaw_inertia_kalman, Level 1) was added
  alongside it, not chained to the production yaw_inertia node.
- CIRCULARITY OPTION 2 (beta-independent tyre-stiffness prior): Caf/Car
  use config/parameters.json stability_estimation.cs_front/rear_
  fallback_reference_n_per_rad -- fixed reference values (axle normal
  load x C_alpha/Fz = 12 rad^-1, Milliken & Milliken RCVD GT3-slick),
  RE-INTRODUCED verbatim (same values 68268/91343, same original
  comment) after their 2026-07-26 deletion (thesis_notes.md "Small-
  decisions sweep": deleted then as consumed nowhere in modules/). This
  observer is the first real consumer; still not consumed by any
  modules/ production path, diagnostics-only. Chosen specifically
  because production's own C_linear_ref (windowed OLS on alpha/Fy) is
  itself alpha-derived -- using it as this observer's tyre-stiffness
  prior would make the observer circular against the very kinematic-
  beta-alpha construction it exists to cross-check.
- ANCHORS: vehicle model = Rajamani, Vehicle Dynamics and Control, 2nd
  ed., Springer 2012, ISBN 978-1-4614-1432-2, Ch. 2 "Lateral Vehicle
  Dynamics", section 2.6 "Dynamic Model in Terms of Yaw Rate and Slip
  Angle", p. 37 (also 2.3 Bicycle Model, p. 27) -- section titles/pages
  read from the book's contents listing, user to confirm visually at the
  page. Kalman algorithm = [user's lecture, course/semester/chapter TO
  VERIFY by user], with Rajamani Ch. 14 (accelerometer/GPS Kalman
  fusion, KF recursion equations around p. 413-414) as a worked
  automotive application. Observer-based sideslip estimation as
  established practice = Kiencke & Nielsen, Automotive Control Systems,
  2nd ed., Springer 2005, ISBN 3-540-23139-0, section "Vehicle Body Side
  Slip Angle Observer" (subsections: Basic Theory of a Nonlinear
  Observer; Observer Design; Validation), page TO VERIFY by user.
- EXPLICITLY NOT CLAIMED: no source prescribes this exact linear single-
  track Kalman sideslip estimator; the model comes from Rajamani, the
  estimator (KF recursion) from the lecture, composed here for this
  project. K&N's own sideslip estimator is a nonlinear two-track
  observer -- named here as the future nonlinear ablation, not
  implemented. K&N's Kalman-filter material in the vehicle chapter
  concerns velocity estimation, not sideslip.
- [2026-08-19] RULE CHANGE, citation location: from this WP onward, full
  Tier A citations (author, title, edition, ISBN, section, page) live in
  thesis_notes.md only; production/diagnostic code docstrings carry a
  single pointer line ("method anchors recorded in thesis_notes.md,
  WP-<id> entry") rather than repeating author/title/page inline.
  Applied here first (diagnostics/sideslip_kalman_observer.py's
  docstring carries the model/state-space definition and the pointer
  line only, no author/title/page).
- [2026-08-19] CORRECTION, citation set: the lecture anchor listed above
  as "TO VERIFY by user" for the Kalman algorithm has been DROPPED by
  user decision and is no longer part of this entry's citation set. The
  Kalman algorithm now rests instead on Rajamani, Vehicle Dynamics and
  Control, 2nd ed., Springer 2012, Ch. 14 (worked automotive Kalman
  filter application) -- already named above as a supporting anchor,
  now the SOLE algorithm anchor rather than a secondary one alongside
  the lecture. The vehicle model anchor is unchanged: Rajamani sec. 2.6
  (p. 37) and 2.3 (p. 27), same book. Kiencke & Nielsen's "Vehicle Body
  Side Slip Angle Observer" section stands unchanged as textbook support
  for observer-based sideslip estimation being established practice, not
  as a source for this specific estimator. The EXPLICITLY NOT CLAIMED
  paragraph above stands unchanged by this correction: no source
  prescribes this exact linear single-track Kalman sideslip estimator;
  model and estimator are composed here.

### WP-S4b: observer self-consistency and the Cr_A inflation finding [2026-08-19]
- The 2-3x alpha_r_ss overshoot reported in WP-S4 traced to the
  REFERENCE, not the observer: Cr_A (C_linear_ref_r, the windowed-OLS
  linear-reference stiffness fitted against the washout-suppressed
  kinematic alpha) is inflated, most severely at C3/C11/C13 (Cr_C/Cr_A =
  0.264/0.526/0.384) -- the same three corners that showed the worst
  apparent overshoot in WP-S4. Recomputed with Cr_C (the identical
  windowed-OLS logic, fed the observer's own alpha instead), alpha_r_ss
  matches the observer's own alpha_r within a few percent at every
  corner except thin-sample C4 (n=3). diagnostics/inspect_observer_
  self_consistency.py, new sibling script (not a harness extension --
  a one-off cross-check, not a metric every future candidate needs,
  same reasoning as WP-S3b/S3c).
- SECOND FINDING, recorded in its own right, independent of the
  overshoot resolution above: Cr_A spans roughly 79k-337k N/rad across
  the 14 stable corners (a factor of ~4 for one tyre on one car), while
  Cr_C's spread over the same samples is much tighter (~97k-101k N/rad).
  A four-fold corner-to-corner swing in rear cornering stiffness is not
  physically plausible -- this is evidence that the kinematic alpha's
  error propagates into the PRODUCTION cornering-stiffness estimate
  itself (Module 4b, CS_ratio's own C_linear_ref/C_alpha machinery), not
  only into beta as previously framed. IMPLICATION, not yet acted on: a
  future beta fix (whichever form the observer or a successor takes) is
  expected to move CS_ratio values and therefore verdicts -- the
  standing threshold re-derivation rule (CLAUDE.md deviation taxonomy)
  applies at that point, not before; nothing re-derived here, this
  entry only records that the trigger condition now has direct evidence
  behind it.
- EPISTEMIC LIMITS, stated explicitly rather than left implicit:
  self-consistency is NOT accuracy -- Cr_C is regressed FROM alpha_r_C
  (the same windowed-OLS logic, just fed the observer's own alpha), so
  the near-1.0 agreement between alpha_r_ss(Cr_C) and alpha_r_C is close
  to circular wherever that regression is well-conditioned; a wrong-but-
  smoothly-varying alpha estimate would pass the same test. The
  Cr_C-vs-RCVD-fallback-prior agreement (median Cr_C within 9% of
  Car_prior=91343 N/rad) is a NUMERICAL SANITY CHECK on the Kalman
  filter's own implementation (confirms the discretization/recursion/
  measurement-matrix construction behaves as designed, no coding bug
  driving wild divergence from the intended physics) -- not independent
  physical confirmation that 91343 N/rad is this car's true rear
  stiffness, since the observer's own dynamics are built around that
  same prior and would structurally tend to reproduce it regardless. No
  ground truth for sideslip exists anywhere in this log; every check in
  this WP-S3/S4 arc is internal-consistency or cross-candidate
  agreement, never an independent measurement.

### Sideslip sign check: physical validation of the observer [2026-08-19]
- TEST: in steady cornering at racing speed, vehicle sideslip signs
  OPPOSITE the turn direction (the rear of the car points to the
  outside of the corner) -- standard bicycle-model result, same
  Rajamani sec. 2.3/2.6 anchor as the observer's own vehicle model.
  Turn direction taken from the sign of median lateral acceleration in
  each corner's canonical window (this codebase's own established
  convention). This is an EXTERNAL physical check -- not a comparison
  against another estimate (WP-S3's Metric 2/5) and not a self-
  consistency check (WP-S4b) -- the first such check in this arc.
  diagnostics/inspect_sideslip_sign_check.py, new sibling script.
- RESULT: the observer's sign matches the physical expectation at all
  14 corners. The kinematic estimate matches at 8, is clearly wrong at
  5 (C6, C7, C9, C10, C12), and is a near-zero borderline case at one
  (C11, +0.022 deg -- too small to read as a decisive sign either way).
- NUANCE, recorded honestly rather than left as a clean 14-vs-8
  scoreline: sideslip sign genuinely reverses at low speed (the lr/R
  kinematic term dominates over the speed-scaled term), and three of
  the five kinematic mismatches (C7, C9, C12) are this dataset's only
  low-speed-class corners -- at those three, the kinematic sign is
  actually CONSISTENT with the low-speed-reversed expectation (same
  sign as ay), not simply wrong. Neither method is demonstrably wrong
  at those three corners with this check alone. The defensible claim is
  therefore that the observer is correct at the eleven racing-speed
  corners while the kinematic estimate is wrong at two of them -- C6
  (130.6 km/h) and C10 (150.5 km/h), both well above the reversal
  threshold (config/channels.json corner_speed_thresholds.low_max=80
  km/h, reused rather than a new threshold invented for this check), no
  low-speed explanation available for either.
- VISUAL COMPARISON EVIDENCE, also recorded here (diagnostics/
  plot_sideslip_comparison.py): the two methods agree closely on
  straights (median |difference|=0.59 deg, n=8627) and diverge sharply
  in corners (median 4.16 deg, n=15556 -- roughly sevenfold); the
  kinematic-vs-observer scatter shows the kinematic estimate compressed
  within about +/-3 deg even inside corner brackets, while the observer
  spreads to about +/-8 deg, visually the clearest single piece of
  evidence in this whole comparison thread that the kinematic estimate
  stays compressed in corners in a way the observer does not.
- UNEXPLAINED, noted as an observation only: the five sign-mismatch
  corners cluster positionally in the C6-C12 middle stretch of the lap
  numbering, while C1-C5 and C13-C14 are clean matches for both
  methods. No mechanism established for this clustering and no further
  diagnostic planned against it this session.

### WP-S5b: Kalman observer tuning outcome [2026-08-19]
- RATIO INVARIANCE CONFIRMED: a discrete linear Kalman filter's gain
  K = P_pred @ C.T @ inv(C@P_pred@C.T + R) is invariant under uniform
  rescaling (Q,R,P) -> (lambda*Q, lambda*R, lambda*P) -- verified both
  algebraically (the lambda cancels in K's own construction) and
  empirically (the original 3x3 (Q_scale, R_scale) grid, diagnostics/
  inspect_kalman_qr_sweep.py, produced byte-identical summary
  statistics for every pair sharing a ratio, e.g. Q=0.1/R=1.0 exactly
  equalled Q=1.0/R=10.0, both ratio=0.1). PRACTICAL CONSEQUENCE: that
  9-point grid tested only 3 distinct settings; the absolute Q/R values
  were never separately meaningful, only their ratio was. Corrected
  sweep: diagnostics/inspect_kalman_qr_ratio_sweep.py, a single ratio
  parameter over np.logspace(-3, 2, 7) with R held fixed.
- INTERIOR TURNING POINTS, not edge-monotonic trends: straight-line
  sideslip (target 1) has a genuine minimum at ratio~0.007-0.05, NOT at
  either grid extreme -- pushing further toward ratio->0 makes it worse
  again (median 0.242 deg at ratio=0.001 vs 0.192 deg at the interior
  minimum). Large-excursion count/max|beta| (target 3) shows the same
  interior-minimum shape, and the low-ratio extreme is markedly worse
  (n=544 samples beyond 10 deg at ratio=0.001, vs n=0-1 near the
  interior minimum) -- a LAG-INDUCED artefact (the heaviest-smoothing
  setting has enough delay relative to genuinely fast corner-entry/exit
  dynamics to overshoot on transients, not a noise effect). Between-
  corner spread (target 2b) appears to improve monotonically toward the
  same low-ratio extreme (std 6.584 deg at ratio=0.001, vs a plateau of
  ~4.25 deg from ratio~2 onward) -- FLAGGED EXPLICITLY as the warning
  sign the work order asked to watch for, not treated as evidence for
  that extreme: this apparent gain is confounded with the same lag
  artefact producing target 3's failure at that identical point (same
  setting simultaneously worst on straight-line noise, excursion count,
  AND transient tracking below), not genuine preserved corner-to-corner
  physical signal. The measure never collapses toward zero anywhere in
  the tested range, so target 2's "must not collapse" failure mode did
  not occur regardless of ratio.
- TRANSIENT-TRACKING TENSION, the new check this round added
  specifically to catch smoothing winning by default: corr(d(beta)/dt,
  d(ay)/dt) during corner entry/exit phases (n=21859 transient samples)
  rises from -0.38 (ratio=0.001) to a genuine asymptotic plateau near
  -0.998 from ratio~2 onward (a well-behaved bounded approach, not a
  runaway-extreme warning sign, since a correlation cannot exceed
  magnitude 1). At the zone that looked steady-state-optimal in
  isolation (ratio 0.007-0.05), this correlation is only -0.70 to
  -0.91 -- a real, measurable loss of transient responsiveness that the
  steady-state measures alone would not have revealed.
- WHY RATIO 0.3162 OVER THE STEADY-STATE-OPTIMAL ZONE: ratio=0.3162
  achieves transient-tracking correlation -0.9896, within ~1 percentage
  point of the asymptotic ceiling, while giving up only a small, real
  amount of steady-state polish relative to the ratio~0.007-0.05 zone
  (straight-line median 0.259 deg vs 0.192-0.215 deg there; excursion
  count n=4 vs n=0-1). Chosen specifically to avoid trading genuine
  corner-entry/exit responsiveness for a marginal steady-state gain --
  consistent with the agreed target 2 framing ("over-smoothing is a
  failure, not a success"). Applied: diagnostics/sideslip_kalman_
  observer.py's Q_BETA_VAR/Q_YAW_RATE_VAR scaled by QR_RATIO=0.3162; R
  left unchanged (the sweep held R fixed throughout, consistent with
  the ratio-invariance finding above). Re-ran the comparison harness
  (diagnostics/inspect_sideslip_methods_comparison.py) once after
  applying the tuned values: both mandatory sanity/regression gates
  (Metric 2 A-vs-B, Metric 5 A regression check) still PASSED,
  unaffected by C's retune as expected (neither gate depends on C).
- SIGN RESULT ROBUST AT EVERY TESTED SETTING: the WP-S5 physical sign
  check (observer sign matches the racing-speed bicycle-model
  expectation) held at 11/11 racing-speed corners at all 7 ratios in
  this sweep, including the chosen 0.3162 and re-confirmed directly
  with the tuned observer wired in (diagnostics/inspect_sideslip_sign_
  check.py, re-run after the code change: 14/14 overall, unchanged).
  Sign robustness was never in tension with the tuning trade-offs
  above -- only the steady-state-vs-transient trade-off was real.
- Full numeric sweep, both the invariance-confirming grid and the
  corrected ratio sweep, plus their visual companions (measures-vs-
  ratio panel and the multi-ratio lap-trace overlay showing the
  heavy/light smoothing contrast directly): diagnostics/inspect_kalman_
  qr_sweep.py (superseded methodology, kept for the record), inspect_
  kalman_qr_ratio_sweep.py, plot_kalman_qr_ratio_sweep.py (diagnostics/
  plots/qr_ratio_sweep/, gitignored). Full candidate comparison closing
  out this arc: sideslip_comparison_report.md (repo root -- see its own
  header note on why not docs/), the WP-S6 deliverable for the win/
  no-win decision.

### GPS-course sideslip as a potential arbiter -- not usable yet [2026-08-19]
- The GPS-course method (B, shelved) is recorded here as a POTENTIAL
  independent arbiter between the kinematic estimate and the Kalman
  observer -- but NOT usable as such on the current data. Its shelving
  record (r=-0.24 with the kinematic estimate, sign agreement at
  chance level, thesis_notes.md WP5b(c)) means it cannot distinguish
  itself from noise on this session, so agreement with either
  candidate would carry no evidential weight either way.
- Its existing reopen condition (denser straight-line anchor data, a
  longer session) is therefore also the condition under which it could
  serve as an arbiter -- specifically at the two racing-speed corners
  where the kinematic estimate and the observer disagree in sign (C6,
  C10, WP-S5 sign check) and neither has independent confirmation.
  Added to the new-data-file diagnostic checklist (PLAN.md) as an item
  to attempt when a longer log arrives.

### Observer saturation-detection failure: the decisive finding [2026-08-19]
- TEST: cornering-stiffness ratios (CS_ratio_f/CS_ratio_r) recomputed
  from the observer's slip angles, compared against the production
  kinematic path -- diagnostics/inspect_observer_slip_angle_
  circularity.py, raised against the rear force-vs-slip plot
  (diagnostics/plot_slip_angle_comparison.py) showing a suspiciously
  clean, near-linear cloud.
- RESULT: observer slip angle explains 99.8% (rear, R^2=0.9979) and
  99.7% (front, R^2=0.9971) of lateral force variance as a straight
  line, with best-fit slopes within 11-12% of the fixed stiffness
  priors (rear: fitted 101055 N/rad vs Car_prior=91343; front: fitted
  60070 N/rad vs Caf_prior=68268) and residual scatter under 6% of the
  force's own spread (rear 4.6%, front 5.4%). The CS_ratio distribution
  compresses toward 1 at both axles (rear p5 0.211 -> 0.716; front p5
  0.107 -> 0.587, corner samples). Under current thresholds, worst-
  phase-per-corner-instance flagged counts fall from 7 strong + 4
  moderate (front) and 5 strong + 4 moderate (rear), out of 56
  instances, to ZERO at both axles using observer-derived CS_ratio.
- MECHANISM: the filter's measurement equation ties lateral
  acceleration to sideslip through the fixed cornering-stiffness prior
  (ay = -(Caf+Car)/m*beta - (Caf*lf-Car*lr)/(m*Vx)*yaw_rate + Caf/m*
  delta_f, thesis_notes.md WP-S4 entry), and lateral acceleration is
  one of only two correcting measurements (the other is yaw_rate
  directly) -- so the Kalman gain pulls the state estimate onto the
  assumed linear tyre relationship regardless of what the tyre is
  actually doing. The measured steering angle (delta_f) enters as a
  control input (via B*u), not as a correcting measurement, which is
  why the expected front-axle independence did not materialise: the
  a priori structural argument (WP-S6's circularity check write-up)
  predicted the front would retain more independent content than the
  rear; the measured R^2 (0.9971 front vs 0.9979 rear) shows this
  essentially did not happen -- worth recording as a case where a
  reasonable-sounding structural argument was tested and refuted by
  the numbers, not assumed correct.
- THE GENERAL STATEMENT: a state observer built on a LINEAR tyre model
  cannot detect departure from tyre linearity. Saturation does not
  exist in its model, so it cannot appear in its output. This is not a
  tuning defect (WP-S5b's Q/R sweep already searched a wide range
  without touching this) or an implementation bug -- it is a structural
  consequence of circularity option 2's own choice (WP-S4: a FIXED
  Caf/Car prior, chosen specifically to avoid the alpha-derived
  circularity in the production C_linear_ref) that was flagged at the
  time as trading one circularity problem for a different limitation,
  whose full downstream consequence (saturation detection specifically)
  is only established with this check.
- WHAT THIS DOES NOT INVALIDATE: the diagnosis of the kinematic
  estimate's own failures (steady-state suppression traced to the
  washout filter, wrong sign at C6/C10, the implausible ~4x corner-to-
  corner cornering-stiffness spread traced to the kinematic alpha's own
  error, WP-S3/S3b/S3c/S4b) and the observer's own validated properties
  (physical sign correctness at all 14/11 corners, WP-S5; order-of-
  magnitude steady-state recovery against the force-balance expectation,
  WP-S4/S4b). Those tested different properties -- direction and
  magnitude of a steady-state quantity -- that do not depend on local
  linearity being violated, unlike CS_ratio, which specifically measures
  deviation from local linearity.
- DECISION RECORDED: the observer is NOT adopted into production. It
  remains a documented diagnostic instrument (diagnostics/sideslip_
  kalman_observer.py), validated on sign, useful for the specific
  diagnostic purpose that motivated building it (locating WHERE and
  roughly how large the kinematic estimate's steady-state suppression
  is), but not usable as a CS_ratio/saturation-detection input as
  currently constructed.
- FUTURE WORK, named honestly, not pursued here: a nonlinear-tyre
  observer (in the spirit of Kiencke & Nielsen's own nonlinear two-
  track construction, already named in WP-S4's thesis_notes.md entry as
  the future nonlinear ablation) or an adaptively-estimated rather than
  fixed stiffness prior could in principle preserve saturation
  detection. Both carry a circularity problem of their own -- slip
  angles are needed to fit the tyre curve that in turn produces slip
  angles -- that would need an explicit resolution before either is
  attempted. Out of scope here; not started.
- [2026-08-19] CORRECTION, status: the "DECISION RECORDED" bullet above
  ("the arc is closed") is SUPERSEDED, not struck -- it correctly
  recorded the linear observer's own rejection, but the arc itself
  continues: the supervisor's own direction is to pursue the nonlinear
  observer named as future work above, not to stop here. See the new
  entry immediately below ("Linear observer saturation-detection
  failure: why the tyre model must be nonlinear") for the corrected
  status and next work package. PLAN.md's STATUS block is rewritten in
  place, not superseded, so it always carries the current state --
  check there for the live status rather than this entry going forward.

### Linear observer saturation-detection failure: why the tyre model must be nonlinear [2026-08-19]
- TEST: cornering-stiffness ratios recomputed from the linear
  observer's slip angles, compared against the production kinematic
  path -- same test and script as the entry above (diagnostics/
  inspect_observer_slip_angle_circularity.py); this entry restates the
  result under the corrected forward-looking framing established after
  the entry above was written.
- RESULT: the observer's slip angle explains 99.8% (rear) and 99.7%
  (front) of lateral force variance as a straight line; best-fit
  slopes within 11-12% of the fixed stiffness priors; residual scatter
  under 6% of the force's own spread. CS_ratio distributions compress
  toward 1 at both axles (rear p5 0.211 -> 0.716, front p5 0.107 ->
  0.587). Under current thresholds, flagged instances fall from 7
  strong + 4 moderate (front) and 5 strong + 4 moderate (rear) to ZERO
  at both axles.
- MECHANISM: the filter's measurement equation ties lateral
  acceleration to sideslip through the FIXED cornering-stiffness prior,
  and lateral acceleration is one of only two correcting measurements,
  so the state estimate is pulled onto the assumed linear tyre
  relationship. Steering angle enters as a control input, not a
  correcting measurement, which is why the expected front-axle
  independence did not appear.
- GENERAL STATEMENT: a state observer built on a linear tyre model
  cannot detect departure from tyre linearity -- saturation does not
  exist in its model, so it cannot appear in its output.
- WHAT THIS DOES NOT INVALIDATE: the diagnosis of the kinematic
  estimate's failures (steady-state suppression, wrong sign at C6/C10,
  implausible fourfold stiffness spread) and the observer's validated
  properties (physical sign correctness at all racing-speed corners,
  order-of-magnitude steady-state recovery). Those test properties that
  do not depend on local linearity.
- STATUS (corrects the prior entry's "arc closed" framing): the linear
  observer is NOT adopted into production, and the arc is NOT closed.
  It establishes the requirement for the next step: a NONLINEAR
  single-track Kalman filter, per the supervisor's own suggestion, with
  the tyre curve IDENTIFIED FROM THIS CAR'S OWN DATA rather than taken
  from a published curve -- no published curve exists for these tyres,
  and the car may run different compounds between events, so a
  published reference would not necessarily be valid anyway.
- CIRCULARITY PROBLEM AND INTENDED RESOLUTION, recorded before any
  implementation starts: slip angles are needed to fit the tyre curve
  that in turn produces slip angles. Intended approach -- fit the curve
  initially from low-slip samples only, where the tyre is genuinely
  near-linear and the CURRENT (linear-observer) estimate is least
  wrong; run the observer with that fitted curve; refit the curve from
  the resulting improved slip angles; iterate to convergence. KNOWN
  LIMITATION, recorded in advance: a fitted curve is only valid over
  the slip range the data actually visits -- extrapolation beyond the
  visited range is not supported by this method and must not be
  presented as such.
- INTENDED BY-PRODUCT: a measured tyre curve for this car, plottable as
  a deliverable in its own right, independent of whatever the
  nonlinear-observer iteration ultimately concludes about production
  adoption.

### WP-N1: Dugoff tyre model chosen + first-pass fit, identifiability finding [2026-08-19]
- MODEL CHOICE: Dugoff pure-lateral tyre model (Rajamani, Vehicle Dynamics
  and Control, 2nd ed., Ch. 13.10, eqs. 13.72-13.76, page TBD verify --
  chapter/eq. numbers only, not yet checked against the printed edition).
  Two-parameter form (c_alpha, mu_fz), analytic in both force and
  dFy/dalpha (modules/tyre_model.py) -- the EKF the next WP builds needs
  the derivative in closed form, not just the force curve.
- ALTERNATIVES CONSIDERED AND REJECTED: a simple ad-hoc saturating form
  (e.g. a raw arctan or polynomial saturation with no friction-circle
  meaning) was rejected in favour of Dugoff -- same parameter count, but
  Dugoff has both a literature anchor and a physical interpretation for
  each parameter (c_alpha = linear cornering stiffness, mu_fz = friction
  force ceiling), where an unanchored ad-hoc form would have neither.
  Magic Formula (Pacejka) recorded as the fallback if Dugoff's two-
  parameter form proves insufficient once the EKF is running (more
  parameters, better shape fidelity far from the fit population, but no
  literature anchor exists yet for THIS car's tyres regardless of which
  form is used -- see identification-loop rationale below).
- SIGN CONVENTION: the literature Dugoff formula is Fy =
  -c_alpha*tan(alpha)*f(lambda) (force opposes slip, standard SAE
  framing). This codebase's own slip-angle definitions already produce a
  POSITIVE Fy-vs-alpha slope (empirically confirmed on Dubai data both
  axles: corr(alpha, Fy_filt) = +0.85 front, +0.59 rear, matching Module
  4b's own positive C_alpha throughout) -- modules/tyre_model.py drops
  the literature minus sign to match this pipeline's established
  convention (Werner (2021) S2.2.3, same convention already used
  throughout stability_analysis.py). Shape unaffected, sign only.
~~- OBSERVER-LINE ANCHORS (for the next WP, recorded now so the EKF starts
  with citations already in hand): Ulsoy, Peng, Cakmakci, "Automotive
  Control Systems" (observer input/measurement structure) -- CHAPTER NOT
  VERIFIED: nobody has opened this book and confirmed which chapter
  covers this; "Ch. 14" is an unconfirmed guess carried from a prior
  reference, not a checked citation, and must not be recorded as one
  until someone actually opens the source. Rajamani Ch. 14 (Kalman
  filter treatment, page TBD verify) -- chapter-level anchor already
  confirmed visually by the user, PLAN.md ANCHORS ("Ch. 14 Kalman
  application"), only the exact page within it is unverified. Kiencke &
  Nielsen, "Automotive Control Systems", 2nd ed., "Vehicle Body Side
  Slip Angle Observer" section (observer-based sideslip estimation as
  established practice) -- section TOPIC/title already confirmed
  visually by the user, PLAN.md ANCHORS; only the numeric section
  reference is unverified. See section 6 open-questions entry.~~
  [CORRECTED 2026-08-19: the "Ulsoy... CHAPTER NOT VERIFIED" framing above
  was itself wrong, not merely cautious -- the citation is now confirmed
  by TWO independent readings of the primary text. Confirmed content:
  Ulsoy, Peng, Cakmakci, "Automotive Control Systems" anchors the
  NONLINEAR SINGLE-TRACK VEHICLE MODEL (sec. 14.3, p. 263 -- section
  header reads "14.3 Nonlinear Vehicle Model", read directly) and the
  OPERATIONAL SIGNIFICANCE OF SIDESLIP (sec. 14.1, p. 258ff, also read
  directly) -- it does NOT cover observer input/measurement structure,
  so it does not belong in an "observer-line anchors" bullet at all;
  moved to the WP-N2 EKF-proposal entry's model-equations anchors
  instead, alongside Rajamani sec. 2.3/2.6. FURTHER CONFIRMED (WP-N2
  implementation turn): Ulsoy Eq. 14.8 is a term-by-term match for this
  EKF's force and moment balances (beta_dot, r_dot equations, WP-N2
  entry below), with two documented simplifications on this project's
  side: (a) the roll degree of freedom is dropped -- Ulsoy's mRh*p_dot
  and Ixz*p_dot terms have no counterpart here, this project's
  single-track model carries no roll state; (b) pure-lateral Dugoff
  (modules/tyre_model.py) is used where Ulsoy uses a combined-slip Magic
  Formula tyre model. Remaining observer-line anchors, unchanged:
  Rajamani Ch. 14 (Kalman filter treatment, page TBD verify -- chapter
  itself already confirmed, WP-S4/PLAN.md ANCHORS) and Kiencke &
  Nielsen's "Vehicle Body Side Slip Angle Observer" section (section
  number TBD verify, and likely to STAY TBD: the source PDF's body text
  does not survive text extraction -- broken font encoding produces
  systematically substituted glyphs, not the actual characters -- so its
  exact section/page numbers can only ever be confirmed from the printed
  copy, not by searching the digitised file).]
- IDENTIFICATION-LOOP DESIGN (own project design, not from a cited
  source): no published tyre curve exists for these tyres, and the car
  may run different compounds between events, so a published curve would
  not necessarily apply even if one existed. Planned iteration: fit the
  curve from low-slip samples first (near-linear regime, where the
  current kinematic estimate is least wrong), run the observer, refit
  from the resulting slip angles, repeat to convergence. Rationale
  recorded in the prior entry ("Linear observer saturation-detection
  failure"); this WP is the first concrete step (the curve-fitting
  machinery), not the loop itself.
~~- FIRST-PASS FIT (diagnostics/fit_dugoff_first_pass.py, Dubai, laps 1-4,
  24183 masked samples): c_alpha_front=18034 N/rad, c_alpha_rear=9206
  N/rad (OLS slope through the origin, |ay|<0.3g population only, n=7240
  each). Both values are roughly 6-10x BELOW the production Module 4b
  C_alpha figures (test_stability.py: mean 114617 N/rad front, 190532
  N/rad rear) and below the config fallback references (68268/91343
  N/rad). Diagnosed cause, checked directly: the low-|ay| population has
  low alpha/Fy correlation (r=0.37 front, r=0.13 rear) -- near zero slip,
  both alpha and Fy are small and noise-dominated, and this fit (a single
  direct OLS pass) has none of Module 4b's safeguards against exactly
  this (minimum window span, R^2-weighted section blending). Forcing the
  regression through the origin was checked and ruled out as the cause
  (allowing a free intercept gives slope within 0.5% of the through-
  origin value, intercept near zero).
- CEILING FIT DID NOT CONVERGE TO AN INTERIOR OPTIMUM: mu_fz_front and
  mu_fz_rear both landed at exactly 100.0% of their search bracket's
  upper bound (bracket = [1, 5x max observed |Fy|] N) -- the objective is
  monotonically improving toward larger mu_fz given this fit's low
  c_alpha, so the "fit" mu_fz values (48905 N front, 67878 N rear;
  implied effective mu ~8.7-8.8, physically implausible for a race
  slick) are a boundary artifact, not an identified parameter. This is
  the WP-N0 gating question answered directly: with THIS c_alpha, the
  ceiling is not identifiable from this data -- not because the data
  never approaches the friction limit (WP-N0 found |Fy|/Fz up to 2.25 at
  p99=1.73, well above what a mu~1-2 slick should sustain, which is
  itself informative), but because the low-slip stiffness fit that feeds
  the ceiling fit is itself unreliable. The two problems compound: a
  understated c_alpha forces an overstated mu_fz to span the same
  observed force range. FIX IS NOT ATTEMPTED THIS SESSION (out of WP-N1
  scope, "no filter, no UI changes this turn") -- candidates for the next
  pass: reuse Module 4b's own windowed/R^2-weighted stiffness estimate
  as c_alpha's source instead of a raw low-ay OLS pass, or widen/
  reweight the linear-regime population.~~ [SUPERSEDED 2026-08-19,
  WP-N1b: the diagnosis above was correct (errors-in-variables
  attenuation, confirmed) but the fix was not yet applied when this
  paragraph was written. See "WP-N1b: c_alpha refit from Module 4b"
  entry below for the corrected fit -- interior-optimum mu_fz, physically
  plausible effective mu.]
- SCOPE: modules/tyre_model.py has zero call sites in modules/ or ui/
  this session -- diagnostics-only, no production wiring, no accuracy-
  level registry entry (nothing to register: not consumed).
  config/parameters.json gained one new block, tyre_model_fit
  (ay_linear_threshold_g=0.3, calibration tunable, WP-N0's candidate-
  threshold list carried forward as the default, not yet re-derived from
  a gap-selection pass of its own). test_stability.py confirmed
  byte-identical (no modules/stability_analysis.py change this session).

### WP-N1b: c_alpha refit from Module 4b [2026-08-19]
- DIAGNOSIS (why the WP-N1 fit failed): the low-|ay| OLS slope is a
  textbook errors-in-variables attenuation case. Near zero slip, both
  the regressor (kinematic alpha) and the response (Fy) are dominated by
  their own measurement/estimation noise rather than by genuine tyre
  signal -- alpha near zero is itself the hardest regime for the
  kinematic sideslip estimate (small denominators, integrated-drift
  sensitivity) and Fy near zero is likewise close to the force balance's
  own noise floor. Classical errors-in-variables theory: OLS slope bias
  is attenuated toward zero in proportion to the regressor's noise-to-
  signal ratio, which is exactly worst where |alpha| is smallest --
  matching the observed 6-10x understatement and the low correlation
  (r=0.37 front, r=0.13 rear) at that population. This is a property of
  the ESTIMATOR (single unweighted OLS pass on a noise-dominated
  regime), not of the tyre.
- DECISION: source c_alpha from Module 4b's own per-sample effective
  stiffness (estimate_cornering_stiffness) instead of refitting a
  better-behaved OLS. Module 4b's window-growth-until-sufficient-span
  and R^2-weighted section-blending machinery exists precisely to resist
  this same attenuation risk across the whole session, not just at low
  slip -- reusing it is not a new estimator, it is deferring to the one
  already trusted throughout the rest of this pipeline (CS_ratio,
  classification thresholds, all of it). c_alpha taken as the median of
  C_alpha_f/C_alpha_r restricted to CS_ratio==1.0 (Module 4b's own
  operational linear-regime indicator: window stiffness at or above the
  currently-known linear reference) within this script's base mask
  (valid-lap, moving, kerb-excluded).
- RESULT: c_alpha_front=132798 N/rad (n=13408), c_alpha_rear=174217
  N/rad (n=16688) -- both now close to Module 4b's own session-wide
  means (114617/190532 N/rad, test_stability.py; different population,
  median-over-linear-regime vs mean-over-all-valid, so not expected to
  match exactly, only to be in the same neighbourhood -- confirmed). The
  mu_fz ceiling refit now converges to an INTERIOR optimum for both
  axles: mu_fz_front=10653 N (21.8% of a widened [1,48905] N bracket),
  mu_fz_rear=15819 N (23.3% of [1,67878] N) -- neither hit the bound.
  Implied effective mu: 1.90 front, 2.06 rear -- physically plausible
  for a GT3 race slick (vs. WP-N1's implausible ~8.7-8.8). RMS residual
  on the full masked population: 2753 N front, 5793 N rear (both fits
  use n=24183, the same full session population as WP-N1, not just the
  linear-regime subset).
- CIRCULARITY STATUS (recorded now, before the EKF is built): both this
  WP's c_alpha and, through it, Module 4b's own C_alpha are KNOWINGLY
  INITIALIZED from the kinematic slip-angle
  estimate, which is already documented elsewhere (thesis_notes.md,
  "Linear observer" entries) to under-read mid-corner. This first-pass
  Dugoff fit therefore inherits that same known weakness -- it is a
  first-round parameter set, not an independently-validated one. The
  PLANNED loop (kinematic-seeded fit -> nonlinear observer -> refined
  slip angles -> refit -> repeat, see "Linear observer saturation-
  detection failure" entry) is INTENDED to break this dependence once it
  runs. As of this session THE OBSERVER DOES NOT YET EXIST -- nothing
  has been fused, nothing has been refined, and no iteration has
  happened. Whether the loop actually converges, and whether convergence
  removes the kinematic estimate's bias rather than just relocating it,
  WILL BE VERIFIED EMPIRICALLY when the observer is built and run (per
  open design decision (2) in PLAN.md) -- it is not verified now and
  must not be described as verified or as already broken.
- SCOPE: diagnostics/fit_dugoff_first_pass.py only, modified in place
  (no new file). No modules/ or config/ change this turn beyond what
  WP-N1 already introduced. test_stability.py confirmed byte-identical
  (Module 4b's estimate_cornering_stiffness is now also CALLED by this
  diagnostic script, but the function itself is untouched -- same
  computation test_stability.py already exercises, reused not modified).

### WP-N2: nonlinear Dugoff EKF, pass 0 (frozen parameters, no refit) [2026-08-19]
- MODEL: states x=[beta, yaw_rate], input u=delta_f, Vx scheduled
  (production ecu_speed, floored). Slip angles use this codebase's own
  small-angle definitions (modules/stability_analysis.py estimate_slip_
  angles): alpha_f = delta_f - beta - a*r/Vx, alpha_r = -beta + b*r/Vx.
  Tyre forces from modules/tyre_model.py's Dugoff model, frozen WP-N1b
  parameters. Dynamics: beta_dot = (Fy_f+Fy_r)/(m*Vx) - r, r_dot =
  (a*Fy_f - b*Fy_r)/Iz -- a convention-agnostic Newtonian force/moment
  balance (holds regardless of the Fy-vs-alpha sign convention in use,
  as long as Fy_f/Fy_r are the actual signed physical force). Anchors:
  Rajamani sec. 2.3/2.6 (same equations already anchoring the kinematic
  estimator) and Ulsoy, Peng, Cakmakci sec. 14.1/14.3 -- see the
  corrected WP-N1 entry bullet above for the citation-correction record
  and the Eq. 14.8 term-by-term match (two documented simplifications:
  no roll DOF, pure-lateral Dugoff vs. combined-slip Magic Formula).
  Measurement set identical to the rejected linear observer (yaw rate +
  ay) -- h1=r (direct), h2=(Fy_f+Fy_r)/m (nonlinear in x through the
  tyre law, same structure the rejected filter used with Caf/Car in
  place of the Dugoff terms).
- JACOBIANS: F=df/dx and H=dh/dx, both built exclusively from modules/
  tyre_model.py's analytic dugoff_lateral_stiffness (Cf_eff, Cr_eff) --
  no numerical differencing. F evaluated at the prior state estimate
  (predict step), H at the predicted state (update step) -- standard EKF
  convention. State propagation integrates the true nonlinear f(x,u)
  directly (explicit Euler); F/Ad are used ONLY to propagate the
  covariance P, never to propagate the state itself -- the mechanical
  difference from the rejected filter, which was fully linear so state
  and covariance propagation used the same Ad@x operation throughout.
- JACOBIAN-COLLAPSE SANITY CHECK (diagnostics/inspect_ekf_dugoff_sanity_
  checks.py, section 1): at alpha=0 exactly, F/H reduce to the rejected
  filter's own A/C matrices (diagnostics/sideslip_kalman_observer.py)
  bit-for-bit when the same fixed Caf/Car are substituted in (max|F-A|=
  max|H-C|=0.0 exactly) -- confirms the hand-derived Jacobian formulas
  are algebraically correct. At a small nonzero alpha (0.02 rad, ~1.15
  deg, within the visited kinematic range) the match degrades slightly
  (max|F-A|=1.8e-3, max|H-C|=4.7e-2) because Dugoff's Fy=C*tan(alpha)*
  f(lambda) is not linear in alpha even before saturation (tan(alpha) !=
  alpha) -- exact collapse holds only in the alpha->0 limit, as expected
  and as the check's own docstring states in advance.
- Fy-AXLE DEPENDENCY IDENTITY (amendment, verified numerically on Dubai
  data, section 3 of the same sanity-check script): a*Fy_f - b*Fy_r ==
  Iz*psidd_raw IDENTICALLY, given a/b computed live from wheelbase*
  corner-weight-fraction (max abs deviation 7.3e-12 Nm across 39464
  moving samples -- floating-point noise only, not approximation). Using
  config's own STORED (3-decimal-rounded) cog_to_front/rear_axle_m
  instead gives a small nonzero residual (max 21.1 Nm, mean 5.4 Nm,
  median relative deviation 0.43% against a 645 Nm floor) fully
  explained by that 0.54mm rounding gap, not by any additional
  independent measurement. CONSEQUENCE, stated plainly because it
  changes how every downstream result must be read: Fy_f and Fy_r
  (modules/stability_analysis.py estimate_lateral_forces, the forces
  WP-N1b's frozen c_alpha/mu_fz were fit against) carry exactly TWO
  independent measured quantities between them -- ay and yaw-rate-
  derivative (psidd) -- not four independent per-axle numbers. A fit
  against "both axle forces" is therefore a fit against a fixed linear
  transform of two raw signals, not two independently-corroborating
  channels; recorded in config/parameters.json tyre_model_ekf.pass_0.
  fy_axle_dependency_note for permanence.
- Fy_f/Fy_r AND MEASURED ay -- CONFIRMED FROM CODE, QUOTED (modules/
  stability_analysis.py estimate_lateral_forces): `Fy_total = m *
  state["ay_mps2"]`, then `Fy_f_full = Fy_total * front_fraction + Iz *
  psidd_raw / wheelbase`, `Fy_r_full = Fy_total - Fy_f_full`. Both axle
  forces are built directly from MEASURED ay (via Fy_total) plus
  measured yaw acceleration -- there is no independent per-axle force
  measurement anywhere in this pipeline. This is exactly what the
  Fy-axle dependency identity above proves algebraically: two measured
  scalars in, two "independent-looking" axle forces out.
- h2-vs-ay SIGN/UNIT CONSISTENCY CHECK, NOT VALIDATION (same script,
  section 2, explicitly labelled as such in its own docstring because it
  is PARTLY CIRCULAR: the axle forces used to fit c_alpha/mu_fz, WP-N1b,
  themselves derive from measured ay, so a close match here partly
  reflects that shared ancestry, not independent confirmation -- must
  never be cited later as evidence the model is correct). Population:
  471 samples, each corner's apex_3 phase (expanded +/- apex_half_
  window_samples the same way summarise_corners does, since apex_3 is a
  zero-width instant otherwise), valid-lap/moving/kerb-excluded.
  Residual (h2_pred - ay_meas): p10=-9.55, median=+1.50, p90=+8.53 m/s^2
  against an ay_meas range of -19.7 to +23.1 m/s^2 at these samples --
  real, non-trivial scatter (roughly consistent with WP-N1b's own fit
  RMS residuals, ~2-4 m/s^2-equivalent per axle), not a tight match;
  reported plainly, not spun, per the check's own stated purpose (catch
  a gross sign/unit error, nothing stronger).
- Iz CHOICE: vehicle.yaw_inertia_kgm2 (2082.0), NOT vehicle.yaw_inertia_
  kalman_kgm2 (1800.0) -- estimate_lateral_forces built the training-data
  forces using 2082.0, so using 1800.0 in this filter's r_dot would make
  its own moment balance inconsistent with its training data's Iz by
  ~14%. Both remain Level-1 estimates in their own right (2082.0: m*a*b
  bicycle-model approximation, ~10-20% error; 1800.0: unsourced reviewer
  placeholder) -- 2082.0 is chosen for training-data self-consistency,
  not claimed as the more accurate of the two. Recorded in config/
  parameters.json tyre_model_ekf.pass_0.Iz_provenance.
- DIVERGENCE MONITORING: windowed NIS against a chi-square bound
  (df=2, two measurements) plus a hard 15 deg |beta| ceiling -- the
  ceiling is physically anchored, deliberately NOT derived from the
  kinematic estimate's own observed beta range (that estimate under-
  reads mid-corner, so its range would clip the very signal this filter
  exists to recover). NIS window width, chi2 bound and flag fraction
  (20 samples, 5.99, 0.5) are PLACEHOLDER defaults, not yet data-derived
  -- explicitly deferred to the validation work package, not this pass
  (config note: nis_tuning_note). Fallback on either trigger is FIXED,
  not optional: beta -> kinematic estimate at that instant, yaw-rate
  state -> measured yaw rate at that instant, P -> P0. Raw (pre-
  fallback) EKF output is still returned alongside the fallback-
  corrected series and the diverged_mask flag -- never a silent
  substitution.
- SCOPE: diagnostics/sideslip_ekf_dugoff.py (new) and diagnostics/
  inspect_ekf_dugoff_sanity_checks.py (new) this turn; config/
  parameters.json gained one new additive block, tyre_model_ekf.pass_0
  (frozen Dugoff parameters + Q/R/P0 seeded from the tuned linear
  observer + divergence tunables, all with provenance notes) --
  tyre_model_fit untouched. No modules/ or ui/ file changed; the new
  filter imports modules/tyre_model.py and modules/stability_analysis.py
  estimate_sideslip, nothing more. The filter has NOT been run on real
  data this turn, and the validation script (sign-check equivalent,
  reframed saturation/circularity check, steady-state magnitude check,
  divergence-monitor summary) is explicitly the NEXT work package, not
  this one. test_stability.py confirmed byte-identical.

### WP-N2 pass-0 run: NIS baseline, saturation coverage, and three convergent lines of evidence for slip-angle under-read [2026-08-19]

ESTABLISHED (measured this run, diagnostics/run_ekf_dugoff_pass0.py,
24,183 masked samples -- valid-lap, moving, kerb-excluded):
- Raw EKF beta: p1=-5.15, p25=-1.01, median=+0.06, p75=+1.38,
  p99=+5.17 deg; max |beta| 14.12 deg; ZERO samples hit the 15 deg
  hard bound -- the filter does not physically diverge.
- NIS exceedance against the 95% chi-square bound: yaw-rate channel
  85.8%, ay channel 71.9%, combined 93.4%. Percentiles: yaw rate
  p50=156.2 / p90=2108.1 / p99=7206.0; ay p50=15.0 / p90=181.7 /
  p99=1850.5.
- Divergence-monitor clustering under the placeholder thresholds
  (nis_window_samples=25, nis_flag_fraction=1.0): 6,730 of 24,183
  samples flagged, 240 contiguous episodes, 92% of flagged samples
  in C14 and 86% in the entry_1_brake phase.
- Dugoff adhesion/sliding onset from the frozen pass-0 parameters,
  tan(alpha)=mu_fz/(2*c_alpha): front 2.297 deg, rear 2.599 deg.
  Coverage against kinematic |alpha|: front 34.0% of samples past
  onset (p50 1.47, p90 3.84, p99 5.06 deg); rear 6.95% past onset
  (p50 0.81, p90 2.37, p99 3.36 deg).
- h2-vs-ay: corr(h2_pred, ay_meas) = +0.887, regression slope
  0.582, intercept 0.438, n=471. Sign convention confirmed correct;
  magnitude systematically damped by roughly 40%.

REASONING (analysis of the above, not separate measurement):
- The NIS baseline is dominated by MODEL error, not tuning. R for
  the ay channel assumes a measurement standard deviation of
  0.05 m/s^2 while the fit's own RMS residuals are several m/s^2
  (WP-N1b: 2753 N front / 5793 N rear over 1356 kg = 2.0 and
  4.3 m/s^2). A residual two orders of magnitude above the assumed
  sensor noise produces NIS in the thousands by itself. Inflating R
  would conceal this rather than correct it.
- Consequence, stated as the reason this is not simply a bad
  result: large, structured innovations are the raw material the
  planned refit iteration needs. A near-perfect NIS would mean the
  innovations carry no information about the tyre parameters and
  the loop would have nothing to converge on. Note also that the
  rejected linear observer was never NIS-checked, so it may well
  have carried the same disagreement unmeasured.
- Rear mu_fz identifiability is now a measured concern, not a
  worry: under 7% of rear samples reach the saturating branch and
  even p99 sits barely past onset. If the rear refit fails to
  settle across passes, this is the first explanation to test.
- The entry_1_brake / C14 concentration of divergence flags is
  CONSISTENT WITH the documented pure-lateral Dugoff simplification
  (no combined-slip coupling, where Ulsoy's own reference model
  uses combined-slip Magic Formula), since heavy braking is
  precisely where longitudinal-lateral coupling matters. NOT
  isolated from the Q/R miscalibration above; both may contribute.
  CAVEAT on the phase attribution itself: entry_1_brake's start
  (modules/corner_analysis.py _build_corner, brake_start_t) is found
  by an off-throttle lookback that searches back across the entire
  prior time history for the last sample below brake_throttle_max_pct
  -- this window can extend up the preceding straight and overlap
  neighbouring corners' own brackets (first found doing WP-N0's
  own per-corner masking, diagnostics/inspect_saturation_coverage.py).
  The 86% entry_1_brake figure above is therefore INDICATIVE of where
  divergence concentrates, not an exact phase attribution.

EXPECTED, CONDITIONAL, NOT YET CONFIRMED:
- If the h2 check used the pipeline's KINEMATIC slip angles (open
  question, to be answered from the code next turn), then the 0.582
  slope is consistent with the kinematic estimate's known
  mid-corner under-read propagating through: suppressed sideslip ->
  slip angles too small -> Dugoff evaluated at too-small angles
  predicts too little force -> h2 under-swings ay. If instead it
  used the EKF's own slip angles, this interpretation does not
  hold and the slope indicates a genuine curve-shape deficiency.
  Record both branches; do not assert either.
- Falsifiable prediction for the refit passes: if the EKF recovers
  the missing steady-state sideslip, slip angles should grow and
  the h2 regression slope should move toward 1.0. If they do not
  move, the under-read explanation is wrong.

UNVERIFIED, recorded as a lead not a fact:
- Team hearsay (source: told to the author, no datasheet, not
  checked): these tyres peak near 8 deg slip angle. Peak slip angle
  varies with compound, temperature and load, so this is indicative
  at best. IF approximately right, the session's measured maxima
  (4.8 deg front, 3.4 deg rear kinematic) are roughly half what a
  GT3 driven at the limit should reach, which is a third
  independent line pointing at the same slip-angle under-read as
  the force-balance gap (WP-S3c: 0.9-5.8 deg rear slip demanded
  where the estimate reads ~0) and the h2 slope above. Mark
  TODO-verify.
- Model-shape limitation worth stating alongside it: the Dugoff
  form has NO peak. It rises and asymptotes toward a ceiling and
  never falls. A real tyre stays linear longer, peaks, then loses
  force. So Dugoff necessarily bends earlier and more gently than
  an 8-deg-peak tyre would. This does not invalidate pass 0, but if
  the fitted curve keeps disagreeing with the data in that specific
  direction, the Magic Formula fallback (already recorded in WP-N1)
  is the indicated next step.

OPEN ITEM for PLAN.md's PARKED section (not added there this turn --
recorded here only, per instruction):
- beta_washout_cutoff_hz has never been swept. Production sits at
  0.05 Hz (~3 s time constant) against 2-5 s corners, which is why
  steady-state sideslip is suppressed. WP-S3c established that
  removing the washout entirely gives 5.7 deg median residual drift
  over a single corner (p90 10.8, max 14.3), i.e. larger than the
  0.9-5.8 deg signal being sought -- so "no filter" is not viable.
  But the intermediate range (e.g. 0.02, 0.01 Hz) was never tested.
  Cheap to run, judged against the existing sign check and
  force-balance expectation, and could improve the PRODUCTION
  estimate independently of whether the EKF survives. Sequenced
  after the EKF so the EKF's own beta can serve as the yardstick.

#### Circularity check: pass-0 EKF vs the rejected linear observer [2026-08-19]

ESTABLISHED (diagnostics/inspect_observer_slip_angle_circularity.py,
same script and quantities used to condemn the linear observer):
- R^2 of alpha vs Fy against a straight line: front 0.9739, rear
  0.9710. Linear observer reference: 0.9971 / 0.9979.
- Best-fit slope vs the frozen pass-0 prior: front 86,013 vs
  132,798 N/rad (ratio 0.648); rear 163,367 vs 174,217 (ratio
  0.938). Linear observer reference: within 11-12% of its own
  fixed priors at both axles.
- Residual scatter as a fraction of Fy's own spread: front 16.1%,
  rear 17.0%. Linear observer reference: under 6% both axles.
- Worst-phase-per-corner-instance flagged counts, out of 56, under
  current thresholds: front 0 strong + 10 moderate, rear 4 strong
  + 29 moderate. Linear observer reference: ZERO at both axles.
  Production kinematic baseline: 7 strong + 4 moderate front,
  5 + 4 rear.
- CS_ratio medians: front 0.525 (EKF) vs 0.757 (kinematic); rear
  0.518 (EKF) vs 1.000 (kinematic).
- New check, frozen pass-0 curve evaluated at the EKF's OWN slip
  angles vs measured Fy: front R^2 0.9872, RMS residual 750 N
  (p10 -934, p50 +239, p90 +992); rear R^2 0.9916, RMS 812 N
  (p10 -1100, p50 -231, p90 +981). Residuals are roughly 9-12% of
  each axle's Fy standard deviation (6,600-8,900 N).
- Onset coverage using the EKF's own alpha: front 53.7% past
  2.297 deg (kinematic: 34.0%), p50 2.638 / p90 5.823 / p99 7.632
  deg; rear 36.8% past 2.599 deg (kinematic: 6.95%), p50 1.877 /
  p90 4.017 / p99 5.826 deg.

ASSESSMENT:
- The structural failure that condemned the linear observer is
  ABSENT here. That observer could not represent saturation and
  flagged zero instances at either axle; this one flags 4 strong +
  29 moderate rear and 10 moderate front. Saturation exists in the
  model and appears in the output.
- Independence is PARTIAL and FRONT-DOMINANT, not established. The
  front slope ratio of 0.648 means the EKF's own slip angles imply
  a stiffness only 65% of the prior they were given -- a
  substantial departure from restatement (the linear observer sat
  at 0.88-0.89 of its priors). The rear ratio of 0.938 is much
  closer to a restatement.
- That asymmetry is COHERENT rather than random, which is itself
  evidence the mechanism is real: the front covers the saturating
  branch (34% past onset kinematically) while the rear barely does
  (6.95%). Where the data carries information about the curve, the
  estimate moves away from the prior; where it does not, the
  estimate echoes the prior.
- The frozen-curve check does NOT read as clearly non-circular.
  R^2 of 0.987-0.992 means most of the variance is the model
  recovering its own assumed shape. The 9-12% residual is real
  departure and is not negligible, but the headline number leans
  toward the EKF's alpha being close to a deterministic inverse of
  the assumed curve.
- CAUTION on the rear coverage jump (6.95% -> 36.8%): the rear is
  simultaneously the axle whose slip angles are most tied to their
  prior. The most likely reading is that rear slip angles grew
  largely because the assumed curve implies they should have, not
  because the data demanded it. Do not cite the rear coverage
  improvement as independent evidence.

STANDING SUMMARY going into the tuning package: partial
independence, front-dominant, rear weak. Not the linear observer's
failure mode; not yet a demonstrated saturation detector.

#### Combined-slip limitation: rear exit-traction and front entry-braking false negatives [2026-08-19]

PHYSICS (Tier A, anchor already in hand):
- A tyre has one friction budget shared between longitudinal and
  lateral use. Under a friction-circle construction, a tyre using
  a fraction x of its capacity longitudinally retains
  sqrt(1 - x^2) laterally: 13% lateral loss at x=0.5, 29% at
  x=0.7, 56% at x=0.9.
- Consequence for this car: on corner exit the rear axle of a
  rear-engined RWD car spends heavily on traction, so its
  REMAINING lateral capacity can be roughly half its
  neutral-throttle value -- meaning the rear reaches its limit at a
  much SMALLER slip angle than a pure-lateral model implies. A rear
  reading 2 deg of slip is not necessarily inside the linear range.
- Mirror case on entry: front brake bias means the fronts spend
  longitudinal capacity under braking, cutting lateral capacity
  exactly when turn-in demands it.
- LIMITATION STATED PLAINLY: modules/tyre_model.py implements the
  PURE-LATERAL Dugoff reduction. It therefore cannot represent
  either case, and will produce FALSE NEGATIVES -- reporting an
  axle as unsaturated when it is at its combined limit -- in
  precisely the two situations a race engineer cares most about
  (power-down oversteer on exit; entry limitation under braking).

ANCHOR (no new citation needed):
- Rajamani Ch. 13.10's Dugoff formulation is ALREADY combined-slip:
  its lambda term includes both the longitudinal slip ratio and the
  slip angle, and is exactly the friction-circle construction above.
  Extending modules/tyre_model.py to combined slip means using LESS
  of that section's simplification, not adopting a new source. The
  pure-lateral reduction is documented as this project's own choice
  in the WP-N1 entry.

SUPPORTING EVIDENCE ALREADY ON RECORD (cross-reference, not new
measurement):
- WP-S1 (wheel-speed source characterization) designated log_speed_*
  as the candidate wheel-speed family, byte-identical to
  ecu_speed_wheels_*, deliberately not whitelisted because no
  consumer existed. A combined-slip model would be that consumer.
- WP-S1 found a BRAKING-SPECIFIC front effect: front wheel speed
  reads -1.38% under braking (log_pbrake_f > 5 bar) versus -0.03%
  off-braking, while the rear stays flat at -0.09% -- the signature
  of front-wheel slip under braking on a front-brake-biased car.
- WP-S1 also established the rear's constant +1.41% offset as a
  rolling-radius difference, NOT traction slip (throttle-independent:
  +1.44% under throttle vs +1.41% overall). A constant offset is
  correctable; a slip-dependent one would not be. This is what makes
  a wheel-speed-derived slip ratio plausibly usable here.
- WP-N2 pass-0's divergence flags concentrate 86% in the
  entry_1_brake phase (indicative, see the phase-overlap caveat
  already recorded) -- consistent with, though not isolated from,
  missing longitudinal-lateral coupling.

STATUS: EXPECTED, not established. Two specific unknowns before any
implementation: (1) whether the rear actually reaches meaningful
longitudinal utilisation on this session's data, which has not been
measured; (2) whether a wheel-speed-derived slip ratio is clean
enough to use once the rolling-radius offset is corrected.
Deliberately NOT started now -- the EKF arc has three open problems
ahead of it (R calibration, divergence-monitor design, refit
passes), and layering a combined-slip model onto a miscalibrated
filter would entangle two investigations.

#### Max-|beta| excursion and the divergence monitor's short-run blind spot [2026-08-19]

ESTABLISHED -- the C2 excursion:
- The session's largest raw EKF sideslip at pass 0, -14.119 deg,
  sits at lap 4, t=884.224s, s_m=868.0m, inside C2's canonical
  bracket -- NOT in the C14 divergence cluster.
- The excursion spans roughly 12 consecutive samples (~0.24s,
  t=884.204-884.324) with beta_raw between -11 and -14 deg,
  coincident with ay of 24.9-26.0 m/s^2 (2.5-2.65 g) and yaw rate
  of 18-25 deg/s.
- Steering (delta_f) stays small and smooth throughout at
  ~1.7-2.0 deg -- no correction input of the kind a genuine
  -14 deg slide at 153 km/h would normally produce.
- At t=884.344s beta_raw moves from -7.76 to +3.06 deg in a single
  20 ms sample: a 10.8 deg sign-flipping discontinuity.
- nis_ay across the peak window: 22,060 / 27,239 / 3,826, against
  a df=1 95% bound of 3.84 -- four to five orders of magnitude over.
- diverged_mask is FALSE at the peak sample despite that NIS, and
  only trips a few samples later.

ASSESSMENT, with its own uncertainty stated:
- The single-sample 10.8 deg sign flip falsifies a genuine
  coherent slide: vehicle sideslip cannot change by that much in
  20 ms. Combined with the near-flat steering trace, the evidence
  points to a numerical excursion.
- Not fully clean, and recorded as such: the underlying ay
  measurement does show a real, unusually large spike (2.65 g)
  plausibly reflecting a physical event -- kerb strike, bump, or
  extreme load moment. Best reading is a real but extreme
  measurement input to which a poorly-calibrated filter,
  consistent with pass 0's 93.4% NIS baseline, responds with an
  unstable, self-inconsistent state estimate. It is not a clean
  instrumentation glitch with no physical basis.
- SUPERSEDED IN OUTCOME by the pass 1 recalibration (see that
  entry): at the calibrated setting the max single-step jump in
  the same window falls to 1.913 deg and stays same-sign, and max
  |beta| in-window falls from 14.119 to 4.327 deg. The pathology
  does not survive calibration. The characterisation above is kept
  because it is what identified the problem and set the pass 1
  acceptance gate.

ESTABLISHED -- blind-spot quantification, WITH ITS THRESHOLD
PROVENANCE STATED:
- Measured against a PROPOSED threshold pair that was never
  implemented: nis_window_samples=25 with nis_flag_fraction=1.0.
- Over pass 0's 24,183-sample masked population: 22,590 samples
  (93.4%) exceed the combined df=2 95% bound (5.99), forming 1,303
  contiguous runs. 1,148 runs / 11,568 samples fall in runs
  SHORTER than 25 samples (p50=9, p90=19, max=24) and are
  therefore invisible to that proposed rule at any severity. 155
  runs / 11,022 samples are >= 25 samples (p50=58, p90=148.6,
  max=232).

CORRECTION, same date, recorded rather than silently fixed:
- The thresholds actually in config/parameters.json, for both
  pass_0 and pass_1, are nis_window_samples=20,
  nis_chi2_bound=5.99, nis_flag_fraction=0.5. Those are what drive
  diverged_mask everywhere it appears, including the shading on
  the plot outputs.
- The blind-spot figures above therefore describe the PROPOSED
  25/1.0 rule, not the implemented 20/0.5 one. Expected direction
  without claiming the number: a 0.5 flag fraction requires only
  half the window above the bound, so the implemented rule should
  be LESS blind than those figures suggest. By how much has NOT
  been measured.
- The STRUCTURAL finding is unaffected by which pair is in use: a
  purely windowed rule cannot detect bursts shorter than its
  window at any severity, so any usable monitor needs a per-sample
  severity trigger alongside the windowed one.
- Provenance of the error, recorded because it is a process
  lesson: the proposed threshold values were carried forward in a
  review instruction as though they had been implemented. It was
  caught when a plot run_info.txt was written against config
  directly rather than against the instruction. Chat reports are
  not the record; config and this notebook are.

### WP-N2 pass 1: noise-model recalibration, derivation and 2-D sweep [2026-08-19]

DECISION RATIFIED, recorded as a deliberate modelling choice:
R for the measurement channels represents TOTAL INNOVATION
UNCERTAINTY -- sensor noise plus model error -- not sensor noise
alone. Rationale: the pass-0 value assumed 0.05 m/s^2 for ay, a
fair claim about the accelerometer, but the filter's ay prediction
also carries the frozen Dugoff curve's own error, measured at
2.0 m/s^2 front and 4.3 m/s^2 rear RMS (WP-N1b). Telling the filter
to trust its prediction to sensor precision while the model is
wrong by two orders of magnitude more is what produced the 93.4%
NIS baseline and the unstable state response at the C2 excursion.
Setting R to total innovation uncertainty is standard Kalman
practice; it is recorded here because it is a chosen modelling
position, not a knob turned until diagnostics improved.

ESTABLISHED -- derivation:
- Front/rear Dugoff fit residuals are strongly correlated,
  rho = 0.8999 over WP-N1b's own fit population (n=24,183). The
  independence assumption was CHECKED AND REJECTED, not flagged:
  independent-assumption combined std 4.7197 m/s^2 (R_ay_var
  22.28) vs correlation-corrected 6.1516 m/s^2 (R_ay_var 37.84).
  This correlation is the expected consequence of the identity
  already recorded (a*Fy_f - b*Fy_r == Iz*psidd): both axle forces
  are a deterministic transform of the same two measured signals.
- The knock-on from ay miscalibration into the yaw-rate channel is
  real and partial, now measured rather than assumed: yaw-rate
  innovation std 3.1709 deg/s at pass 0, falling to 2.2278 deg/s
  after fixing R_ay alone. Roughly half the yaw-rate inflation was
  downstream of the ay problem; a substantial independent residual
  remains. This is why the two R values were derived sequentially
  rather than simultaneously.
- The single-shot derivation FAILED its own pre-registered gate in
  opposite directions per channel: ay over-corrected to 0.04%
  exceedance with the K_ay gain collapsed ~180x, yaw rate still
  under-corrected at 30.1%. A scalar rescale of R could not fix
  both; the channels' relative weighting had to move.

ESTABLISHED -- 2-D sweep:
- Parameterisation: R_ay_scale and R_yaw_rate_scale as independent
  multipliers on the single-shot values (37.8418 and 0.001511889),
  anchored at those values rather than restarted. Q held FIXED --
  nothing in the pass_1 evaluation implicated Q (both the NIS
  improvement and the C2 fix came from R alone), so a third free
  dimension would have widened the search without a diagnosed
  reason. Q's own dimension is explicitly OPEN, not dropped.
- Grid 5x5 = 25 points, R_ay_scale in {1.0, 0.3, 0.1, 0.03, 0.01},
  R_yaw_rate_scale in {1.0, 2.0, 4.0, 8.0, 16.0}. Behaviour
  monotonic in both directions as expected.
- Exactly one grid point put BOTH channels inside the
  pre-registered 3-15% band: R_ay_scale 0.1 (R_ay_var 3.7842,
  9.18% exceedance) and R_yaw_rate_scale 4.0 (R_yaw_rate_var
  0.0060476, 10.01%). The point sits in the grid INTERIOR, not at
  an edge -- a mild indication it is not a knife-edge artifact,
  though only this coarse grid was tested, not a finer one around
  it.
- CROSS-CHECK worth recording honestly: R_ay_var 3.7842 lands
  within 9% of Method B's empirical estimate (3.475), which had
  been demoted for circularity because it was measured from a
  filter running with wrong R. The demotion remains methodologically
  correct -- a filter's settings cannot be derived from its own
  miscalibrated output -- but the contamination did not, in this
  instance, distort the number much.

ESTABLISHED -- acceptance criteria at the recommended setting
(pass 0 -> pass 1 sweep-refined):
- NIS exceedance, GATE: yaw rate 85.8% -> 10.01%, ay 71.9% ->
  9.18%. Both PASS. Combined mean NIS 903.6 -> 2.907 against a
  theoretical 2.
- C2 excursion, GATE: max |beta| in the t=883-885.5s window
  14.119 -> 4.327 deg; max single-step jump 10.826 -> 1.913 deg.
  PASS. Note the interpretation explicitly: the over-corrected
  single-shot derivation gave a SMALLER jump (0.553 deg) but for
  the wrong reason -- a filter ignoring its accelerometer looks
  smooth. The gate was never "small jumps" but "no sign-flipping
  discontinuity": pass 0's jump flipped sign (-7.76 -> +3.06 deg),
  this one stays same-sign (+2.05 -> +0.14 deg).
- Sign check, median, GATE: 14/14 all corners, 13/13 racing-speed,
  unchanged across all passes. PASS. Caveat retained: the median
  is robust to local instability, so this gate measures central
  tendency rather than filter health -- pass 0 scored 13/13 while
  containing the C2 pathology.
- Sign check, per-sample pooled, REPORTED: 98.93% -> 99.63%
  (14460/14513) at racing-speed corners. This is the
  pre-registered prediction that FAILED at the over-corrected
  setting (98.37%, a slight regression) and now holds. Recorded as
  both a failure and a subsequent success, not only the latter.
- Circularity, REPORTED not gated: front slope ratio 0.648 ->
  0.557, rear 0.938 -> 0.771 -- both continuing toward independence
  from the prior. Flagged counts front 0+10 -> 21+11 (32/56), rear
  4+29 -> 20+7 (27/56).
- K_ay gain magnitude: median -4.416e-3 -> -1.933e-4, a ~23x
  reduction, far more moderate than the single-shot derivation's
  ~180x collapse, consistent with the NIS band being satisfied
  rather than overshot.
- h2-vs-ay correlation, population-matched at last: 0.9808 over
  the full masked population (n=24,183) and 0.9682 over the same
  n=471 apex-phase subset that produced the 0.887 kinematic
  reference. At IDENTICAL samples, the EKF's own slip angles
  explain measured lateral acceleration better than the production
  kinematic estimate's do.

OPEN, explicitly not resolved by this work:
- The flagged-count increase (front 32/56, rear 27/56) is not
  attributed. It could be genuine saturation detection or noise;
  this turn's evidence does not distinguish them.
- The circularity improvement was measured at a setting that has
  since changed twice. It should be re-read once the configuration
  is stable, not treated as settled.
- Q was never swept. Its dimension is open, deliberately deferred
  for want of a diagnosed reason to move it, not because it was
  judged unimportant.
- The 3-15% band was satisfied by exactly one point on a coarse
  5x5 grid. No finer search was run around it.

#### Circularity and flag attribution at the calibrated setting [2026-08-19]

ESTABLISHED -- circularity, four-way comparison:
- R^2 of alpha vs Fy against a straight line: front 0.9707, rear
  0.9812. References: linear observer 0.9971/0.9979; pass_0
  0.9739/0.9710.
- Best-fit slope as a ratio of the frozen prior: front 0.557, rear
  0.771. References: linear observer ~0.88-0.89 of its own fixed
  priors; pass_0 0.648/0.938; pass_1 mid-sweep 0.570/0.795.
- Residual scatter as a fraction of Fy's own spread: front 17.1%,
  rear 13.7%. Linear observer reference: under 6% at both axles.
- CS_ratio percentiles, EKF: front p5 -0.208, p25 0.288, median
  0.597, p75 0.953; rear p5 0.002, p25 0.409, median 0.697, p75
  1.000.
- Flagged worst-phase-per-instance counts out of 56: front 21
  strong + 11 moderate, rear 20 strong + 7 moderate. Linear
  observer reference: ZERO at both axles.

ESTABLISHED -- frozen-curve check, and why it is the decisive
number:
- Frozen pass-0 Dugoff curve evaluated at pass_1's own slip angles
  vs measured Fy: front R^2 0.9526, RMS residual 1443 N (p10
  -1908, p50 +429, p90 +1886); rear R^2 0.9822, RMS 1185 N (p10
  -1575, p50 +139, p90 +1582).
- Against pass_0's readings (front 0.9872 / 750 N, rear 0.9916 /
  812 N), the CALIBRATED filter's slip angles depart FURTHER from
  the assumed curve than the uncalibrated filter's did, at both
  axles: lower R^2, roughly doubled front RMS residual.
- REASONING, stated because this is the load-bearing inference of
  the arc: if the filter were substantially inverting its own
  assumed curve, improving its calibration would tighten that
  inversion and drive R^2 toward 1. The opposite happened.
  Increasing the weight the filter places on its measurements
  pushed the estimate AWAY from the model rather than into it,
  which is the signature of genuine measurement information
  entering the estimate. This is the opposite of the behaviour
  that condemned the linear observer, whose slip angles collapsed
  onto its own fixed stiffness prior.
- Corroborating: front slope ratio 0.557 means the stiffness
  implied by the filter's own slip angles is barely half the prior
  it was given; front residual scatter at 17.1% against the linear
  observer's under 6%.

ESTABLISHED -- onset coverage using pass_1's own alpha:
- front 59.04% beyond the 2.297 deg onset, |alpha| p50 3.194 /
  p90 6.611 / p99 8.153 deg.
- rear 48.84% beyond 2.599 deg, p50 2.525 / p90 4.869 / p99
  5.622 deg.
- Monotone across every setting measured: kinematic 34.0%/6.95%
  -> pass_0 53.7%/36.8% -> pass_1 calibrated 59.04%/48.84%. The
  rear-identifiability concern recorded at pass 0 (under 7% of
  rear samples past onset) is resolved at this setting.
- WATCH ITEM, recorded rather than dismissed: front p99 of
  8.153 deg sits essentially at the unverified ~8 deg tyre-peak
  figure. Plausible, but not comfortably inside it. If later
  passes push slip angles further, over-growth becomes a live
  concern rather than a hypothetical.

ESTABLISHED -- flag attribution (attribution only; physical
correctness is NOT assessed here and requires the driver report):
- Front, 32 instances across 13 distinct stable_corner_ids: C1(3),
  C2(2), C3(2), C4(4), C5(3), C6(1), C7(2), C8(2), C9(3), C10(1),
  C12(2), C13(3), C14(4).
- Rear, 27 instances across 11 ids: C1(2), C2(2), C3(3), C4(4),
  C5(1), C8(3), C9(3), C10(1), C12(1), C13(3), C14(4).
- C4 and C14 are flagged on ALL FOUR laps at BOTH axles.
- Worst phase is overwhelmingly apex_3 or exit_4 (roughly 14 and
  13 of the 32 front instances); exit_5 occasional (4);
  entry_1_brake and entry_2_turnin never appear as the worst phase
  at the front. Rear pattern is similar.
- Speed classes span low, medium and high with no concentration.
- CLUSTER, not scatter, at both axles: 13 corners carry 32 front
  instances (average 2.46 each), 11 carry 27 rear instances
  (average 2.45). This is not a many-corners-flagged-once pattern.

REASONING on the two phase signatures, recorded because they were
at risk of being conflated:
- The NIS-divergence flags concentrate in entry_1_brake (86%,
  indicative). The CS_ratio saturation flags concentrate in
  apex_3/exit_4 with entry phases essentially absent. Two
  different checks producing two different phase signatures
  suggests two different underlying causes -- filter instability
  under braking, tyre saturation at apex and exit -- rather than
  one shared artifact. Not proven; recorded as the more likely
  reading, to be revisited if either signature moves.
- The entry_1_brake overlap caveat (off-throttle lookback,
  modules/corner_analysis.py _build_corner) does not materially
  affect the CS_ratio attribution above, since entry_1_brake
  barely appears in that flagged set.

OPEN:
- Whether the flagged corners are the CORRECT ones is not
  addressed. The specific testable claim is C4 and C14 flagged
  4/4 laps at both axles: either genuine problem corners or an
  artifact concentrated there. Requires the June driver report.

### WP-N2 pass 1: CS_ratio interpretability, linear-reference
staleness hypothesis DISPROVED, and the WP-S4b reference-spread
improvement [2026-08-20]

CONTEXT: pass_1's flagged worst-phase-per-instance counts rose
sharply (front 32/56, rear 27/56, against the kinematic path's 11
and 9). This entry records what was checked to decide whether
those flags are interpretable, and the answer is not yet.

ESTABLISHED -- flag interpretability:
- Demand ranking of all 14 stable corners by median |ay| over
  valid laps: C12 1.377g, C3 1.368g, C13 1.281g, C1 1.279g,
  C5 1.241g, C6 1.205g, C8 1.185g, C14 1.182g, C2 1.124g,
  C4 1.109g, C9 1.087g, C7 1.009g, C11 0.981g, C10 0.487g.
  corner_radius_filtered was unavailable, so radius fell back to
  kinematic v^2/ay.
- CAVEAT on that ranking, recorded so it is not over-read: median
  lateral g is a weak proxy for tyre demand. Aerodynamic
  downforce means a fast corner generates more grip as well as
  more load, so a slower corner at lower g can sit closer to the
  tyre's limit than a faster one above it. Statements about
  "the most demanding corners" below are correspondingly weak.
- Flag distribution is near-universal: front 32 instances across
  13 of 14 corners (only C11 clean), rear 27 across 11 of 14
  (C6, C7, C11 clean). With flagging this widespread, demand rank
  cannot discriminate.
- C4 and C14 are the only corners flagged 4/4 laps at both axles,
  yet rank #10 and #8 of 14 by median |ay|. The two top-ranked
  corners are only partially flagged (C12 front 2/4, rear 1/4;
  C3 front 2/4, rear 3/4). C10, the clear low-demand outlier at
  0.487g, is flagged weakly (one moderate front, one moderate
  rear, never strong); C11 is flagged nowhere.
- Threshold comparability: the classification thresholds
  (STRONG_CSF 0.10, STRONG_CSR 0.20, MODERATE_CSF 0.25,
  MODERATE_CSR 0.35) were derived and every subsequent
  re-confirmation performed against KINEMATIC CS_ratio
  distributions, never against an EKF distribution.
  Worst-phase-per-instance percentiles (p5/10/25/50/75/90/95):
    front kinematic -0.041/0.064/0.268/0.389/0.647/0.915/1.000
    front pass_1    -0.818/-0.677/-0.159/0.188/0.475/0.689/0.750
    rear  kinematic  0.188/0.312/0.395/0.757/1.000/1.000/1.000
    rear  pass_1    -0.270/-0.136/0.080/0.366/0.631/0.857/0.951
  Every band moved, not only the tail: front median roughly
  halved, rear median roughly halved, and the kinematic path's
  ceiling-clipped p90-p95 (pinned near 1.0 at both axles) opened
  up under pass_1.
- Flagged-set overlap: front kinematic 11 -> pass_1 32 (10 shared,
  1 lost, 22 new); rear 9 -> 27 (7 shared, 2 lost, 20 new).

REASONING: because the shift spans the whole distribution rather
than concentrating near the old boundary, the count jump is
substantially shift-driven. This diagnostic cannot separate
"distribution moved because the thresholds were fitted to a
different signal" from "pass_1 detects genuine additional
saturation". No threshold was re-derived or applied -- that is a
separate stop under the standing rule.

STANDING CONCLUSION: pass_1's CS_ratio flags are NOT YET
INTERPRETABLE. Deliberately weaker than "artifact": the metric has
not been shown wrong, it has been shown unreadable against
thresholds fitted to a different distribution.

ESTABLISHED -- staleness hypothesis, DISPROVED:
- Hypothesis under test: Module 4b updates C_linear_ref only when
  the entire regression window sits inside
  cs_linear_slip_threshold_rad (0.021 rad, ~1.2 deg, read from
  config). Pass_1's slip angles run far larger than the kinematic
  path's, so fewer windows qualify, the held reference goes stale,
  and CS_ratio is divided by a reference no longer describing the
  current linear stiffness.
- PREMISE CONFIRMED: update rate front 29.21% (kinematic) ->
  25.27% (pass_1); rear 45.96% -> 25.79%, nearly halved. Updates
  are bursty rather than evenly spaced (p50 and p90 gap = 1 sample
  in all four combinations: long runs of qualifying windows on
  straights, droughts through corners). Worst-case drought: front
  788 -> 779 samples (essentially unchanged); rear 490 -> 755
  samples, 50% longer under pass_1.
- CONSEQUENCE CONTRADICTED, by every value-distribution measure:
  pass_1's held reference is MORE stable, not less.
    global p95/p5 spread: front 6.39 -> 3.11; rear kinematic p5 is
      NEGATIVE (-17,265 N/rad, a physically nonsensical stiffness)
      while pass_1's p5 stays positive at 66,695
    per-corner median spread: front 4.00 -> 2.44; rear 6.27 -> 2.12
- WP-S4b REPRODUCTION, methodology verified rather than asserted:
  over the identical near-zero-alpha_r(A) sample set WP-S4b used,
  the kinematic path reproduces its recorded finding almost
  exactly (79,523-337,111 N/rad, ratio 4.24, against the recorded
  79k-337k). Pass_1 over those same samples: 78,117-177,550 N/rad,
  ratio 2.27.
- SIGNIFICANCE, stated because it reaches beyond this entry:
  WP-S4b recorded that fourfold rear reference swing as evidence
  the kinematic slip-angle error propagates into the PRODUCTION
  cornering-stiffness estimate itself, not only into beta. Pass_1
  roughly halves that swing and removes the negative-stiffness
  tail. The EKF measurably improves a production metric that has
  been on record as defective since 2026-08-19.

ESTABLISHED -- what does explain the sign instability:
- Four phase-median sign flips exist across C4 and C14: C4 front
  lap4, C4 rear lap3, C14 front lap1, C14 rear lap3.
- Three of the four coincide with a near-degenerate regression
  window at the phase's own worst sample: R^2 = 0.147, 0.035 and
  0.005 respectively, against 0.77-1.00 on the same corner's
  non-flipping laps. The mechanism is poor conditioning of the
  NUMERATOR (C_alpha) at individual instants, not staleness of
  the denominator.
- Staleness does NOT track with which laps flip: flip and
  non-flip laps for the same corner and axle show comparable
  elapsed samples since the last reference update (C4 front
  63-77 samples across all four laps; C4 rear 53-65).
- The fourth flip, C4 rear lap 3, resists that explanation: its
  worst single sample is well-conditioned (R^2 0.935) and
  negative, while the phase median came out positive, so enough
  of the rest of the phase is positive to pull the median past
  zero. INCONCLUSIVE -- this check cannot distinguish genuine
  within-phase boundary-crossing from noise spread across many
  samples. Recorded as unresolved in either direction.
- FRAMING, recorded to prevent a later misreading: the car is
  driven by a human over four laps, so magnitude variation lap to
  lap is EXPECTED and is not itself evidence of a defective
  metric -- a driver explores line, entry speed and brake release,
  and identical values every lap would be the suspicious outcome.
  What driver variation does not explain is a SIGN change in
  CS_ratio, which is a categorical claim (past the tyre's lateral
  peak or not) rather than a matter of degree. That is why the
  sign flips specifically, not the scatter, were investigated.

METHODOLOGICAL FINDING, useful beyond this entry: window |alpha|
span is structurally uninformative as a conditioning measure. It
clusters at 1.15-1.29 deg in every row, both axles, both paths,
all laps, because estimate_cornering_stiffness's window-growth
loop stops as soon as span first clears
cs_min_slip_angle_span_rad. Sample count (13-152) and window R^2
are the variables that actually move; span is a constant by
construction and must not be read as a quality proxy.

**Threshold re-derivation deliberately deferred [2026-08-20]**
- DECISION: classification thresholds are NOT re-derived against
  the pass_1 distribution at this time. This is a considered
  deferral, not an oversight.
- REASONING: re-deriving the thresholds is the step that commits
  to the EKF as the production sideslip source, since the tool's
  verdicts change the moment it happens. That commitment should
  follow, not precede, (a) finishing the estimator -- the refit
  passes have not run, so the tyre curve is still the one fitted
  from kinematic slip angles -- and (b) the planned comparison
  against a combined-slip formulation. Committing now would fix
  the thresholds to an intermediate estimator.
- CONSEQUENCE, stated so it cannot be misread later: any flagged
  count computed from pass_1 output is NOT comparable to the
  production verdict distribution (historically 0 strong / 15
  moderate / 41 normal over 56 instances) and must not be cited
  as evidence the tool is finding more or fewer problems. The
  distribution shifted across every percentile band; the counts
  are measured on a scale that no longer matches the thresholds.
- SEPARATELY, and importantly: the EKF's own quality does NOT
  depend on this. The three results establishing it -- that
  calibration moved its slip angles further from its own assumed
  curve rather than closer; that its slip angles explain measured
  ay better than the kinematic estimate's at identical samples
  (0.968 vs 0.887, n=471); and that it roughly halves the
  reference-stiffness swing WP-S4b recorded as defective, while
  removing a negative-stiffness tail -- are all measured upstream
  of CS_ratio and are unaffected by which thresholds are in use.

Diagnostics: inspect_corner_demand_ranking.py,
inspect_pass1_flagged_attribution.py,
inspect_threshold_comparability.py,
inspect_cs_linear_ref_staleness.py (all read-only, in
diagnostics/).

### WP-N2 pass 2: EKF-sourced Dugoff refit -- proposal and
pre-registered predictions [2026-08-20]

PURPOSE: pass 0/pass_1 froze a Dugoff curve fitted from KINEMATIC
slip angles (WP-N1b) -- the documented circularity named when this
arc began. Pass 2 is the first pass that refits the curve from the
EKF's OWN slip angles (pass_1's, the calibrated configuration),
then reruns the filter with the refitted curve. This entry records
the design and the predictions BEFORE any pass-2 number exists.

DESIGN, approved with two changes from the original proposal:

- SLIP SOURCE: the EKF's own alpha_f/alpha_r from pass_1 (not
  pass_0, which is known-miscalibrated in R). Same base_mask as
  every fit in this arc (valid-lap, moving, kerb-excluded,
  n=24183).
- FIT STRUCTURE: WP-N1b's two-step shape retained (median c_alpha
  over a linear-regime subset, then bounded 1-D least-squares
  mu_fz over the full population with c_alpha fixed). The
  linear-regime indicator is NOT reused from the kinematic path
  (CS_ratio==1.0 there is itself a kinematic-alpha-derived flag,
  and reusing it would launder the kinematic under-read into the
  refit's own sample selection). Replacement: estimate_cornering_
  stiffness recomputed with the EKF's OWN alpha/Fy (identical
  function, different input array, already exercised this session
  in inspect_threshold_comparability.py and inspect_cs_linear_ref_
  staleness.py) -- its own CS_ratio==1.0 flag is the linear-regime
  indicator, introducing zero kinematic dependency.
- Q/R/P0 HELD at pass_1's values. Re-deriving R against the new
  curve's residuals in the same pass would change curve and noise
  model together, making pass 2's behaviour impossible to attribute
  to either alone -- the same one-variable-at-a-time discipline
  pass_1 itself used ("noise model only"). The resulting formal
  mis-specification (R_ay_var=3.78418 was derived from the residuals
  of the curve pass 2 replaces) is recorded, not hidden.
  Re-derivation is deferred to a later pass.
- CONVERGENCE CRITERION: relative change |theta_N - theta_(N-1)| /
  theta_(N-1) for each of the four curve parameters (c_alpha_front,
  c_alpha_rear, mu_fz_front, mu_fz_rear), converged when ALL FOUR
  fall under 5% for TWO CONSECUTIVE passes (guards against a
  coincidental single-pass near-match). Corroborating check: the
  EKF's own |alpha| distribution pass-over-pass.
- UPPER BOUND ON ITERATION [added at approval, deliberately
  arbitrary]: MAXIMUM FOUR REFIT PASSES, pass 2 through pass 5. An
  open-ended "small step, one more pass" rule has no natural stop if
  the sequence drifts slowly rather than genuinely converging. If
  the criterion above is not met by pass 5, that is recorded as a
  NON-CONVERGENCE FINDING in its own dated entry naming which of the
  three failure modes below occurred, and iteration stops there.
  This cap bounds the exercise; it is not derived from anything
  about the estimator or the data.
- FAILURE MODES: (1) OSCILLATION -- a parameter alternates between
  distinct value sets pass-over-pass (relative-change sign flips).
  (2) MONOTONIC DRIFT WITHOUT SETTLING -- same direction every pass,
  relative-change magnitude not shrinking. (3) PHYSICALLY
  IMPLAUSIBLE VALUES -- effective mu moving away from the already-
  flagged 1.90/2.06 range with no plausibility anchor, or the onset
  boundary crossing outside the range bounded by observed slip
  angles and the ~8 deg hearsay peak figure (TODO-verify, used only
  as an order-of-magnitude fence). On any of these: stop, do not
  keep iterating hoping it self-resolves, record which mode
  occurred. A non-converging loop is itself a legitimate finding
  about whether this iteration can break its own circularity.

PRE-REGISTERED PREDICTIONS, recorded before any pass-2 number
exists, so convergence and correctness are demonstrated rather than
narrated afterward. Any prediction that fails is recorded as a
failed prediction, not quietly dropped.

1. c_alpha per axle FALLS at both axles. Rough magnitude anchor:
   pass_1's own circularity check already measured the EKF alpha's
   best-fit slope as a fraction of the frozen prior -- front 0.557x,
   rear 0.771x (132798 -> ~74k N/rad front, 174217 -> ~134k N/rad
   rear). Expect the refit's linear-regime-median c_alpha to move in
   the same direction and rough order of magnitude, not to land on
   these exact numbers (a linear-regime-restricted median is a
   different statistic than a whole-population best-fit slope).
2. mu_fz / effective mu -- FALSIFIABLE BAND [added at approval]:
   HOLDS if mu_fz stays within +/-25% of pass_0's values at BOTH
   axles (front 10653.12 N: band [7989.84, 13316.40]; rear
   15818.77 N: band [11864.08, 19773.46]). FAILS outside that band
   at either axle. Rationale for the band's centre: mu_fz sets the
   asymptotic force ceiling, set by the data's peak Fy level, which
   this refit does not touch (only the alpha-Fy pairing moves, not
   the underlying Fz/Fy data). If this prediction fails, that
   reasoning -- not the band width -- is what gets revisited.
3. Onset boundary tan(alpha)=mu_fz/(2*c_alpha) MOVES OUTWARD
   (increases) at both axles, following from (1) with (2) roughly
   flat. First-order estimate from the 0.557/0.771 ratios: front
   toward roughly 3.4-4.1 deg, rear toward roughly 3.1-3.5 deg --
   explicitly derived from already-measured ratios, contingent on
   (2) holding.
4. Onset coverage FALLS from 59.04% front / 48.84% rear (pass_1's
   own alpha against the frozen boundary). Coherent outcome, not a
   regression: the boundary moving out is expected to dominate over
   any second-order shift in the alpha distribution itself.
5. Frozen-curve R^2 self-consistency -- the REFIT curve evaluated at
   PASS 2's OWN resulting alpha (from running the EKF with the new
   curve, not the alpha it was fit from): expect a RISE from
   pass_1's 0.9526 front / 0.9822 rear, and this rise is EXPECTED,
   not itself a warning sign (a curve fit to describe its own
   generating alpha-Fy relationship should fit it better than an
   increasingly stale frozen curve did). WARNING SIGNATURE, the
   CONJUNCTION specifically, not a rise alone: R^2 approaching the
   linear observer's ~0.997 level WHILE c_alpha simultaneously snaps
   back toward the pass_0/pass_1 prior (132798/174217) -- that
   combination would mean the loop re-derived its own starting
   point. A moderate rise (toward 0.97-0.98) alongside a c_alpha
   genuinely below the prior is the healthy, expected outcome and
   must not be misread as the warning sign.
6. h2-vs-ay regression slope: NOT expected to move much from pass_1's
   own already-high 0.9682 (measured at n=471 identical apex
   samples, superseding the stale 0.582 kinematic-alpha figure as
   the relevant baseline). h2's ay-match is driven largely by the
   filter's own ay measurement-update pulling the estimate toward
   agreement -- a property of the filter structure, not primarily
   the curve shape -- so this is predicted to be an insensitive
   discriminator of refit quality, not a strong directional call.

SCOPE OF THIS ENTRY: predictions and design only, recorded before
the fit runs. Results, HELD/FAILED verdicts and convergence status
follow in a separate dated entry once the fit has been run.

### WP-N2 pass 2: refit results, prediction verdicts, convergence
status [2026-08-20]

REFITTED PARAMETERS (diagnostics/fit_dugoff_pass2_refit.py,
diagnostics/fit_dugoff_pass2_refit_manifest.json, timestamp
2026-08-20T07:53:48Z; config/parameters.json tyre_model_ekf.pass_2):
c_alpha_front=66647.5 N/rad (-49.81% vs pass_0/1's 132797.9),
c_alpha_rear=118993.1 N/rad (-31.70% vs 174217.3), mu_fz_front=
13577.4 N (+27.45% vs 10653.1), mu_fz_rear=19924.5 N (+25.95% vs
15818.8). Effective mu: 2.42 front, 2.59 rear (up from 1.90/2.06 --
moving FURTHER from plausible, not toward it).

PREDICTION VERDICTS, against the pre-registration above:

1. c_alpha falls at both axles -- HELD. Actual ratios: front 0.502x
   prior (predicted ~0.557x), rear 0.683x prior (predicted ~0.771x).
   Same direction and same rough order of magnitude at both axles;
   both landed somewhat more aggressive (further from the prior)
   than the circularity-check proxy suggested.
2. mu_fz within +/-25% of pass_0 at both axles -- FAILED at BOTH
   axles (front +27.45%, rear +25.95%, both just outside the band).
   REVISITING THE REASONING as required by the pre-registration: the
   stated assumption ("mu_fz is set by the data's peak Fy level,
   which the refit does not touch") treated mu_fz as fit
   independently of c_alpha. It is not -- WP-N1b's two-step
   procedure fixes the ORDER (c_alpha first) but the mu_fz
   least-squares step still runs over the SAME Fy/Fz data using the
   NEW, much smaller c_alpha. Since Fy=c_alpha*tan(alpha)*f(lambda)
   and lambda=mu_fz/(2*c_alpha*|tan(alpha)|), shrinking c_alpha
   shifts lambda at fixed alpha, and the least-squares fit
   compensates by raising mu_fz to keep predicted Fy matched to the
   (Fz/Fy data-derived, unchanged) measured Fy at the alpha values
   the EKF now visits. The two parameters are coupled through the
   shared fitting objective, not independently identified by the
   two-step ordering -- the original reasoning conflated "sequential
   fitting" with "independent identification."
3. Onset boundary moves outward at both axles -- HELD on direction
   (front 2.297 -> 5.816 deg, rear 2.599 -> 4.786 deg, both clearly
   increased), but the stated magnitude estimate (front ~3.4-4.1,
   rear ~3.1-3.5 deg) was EXCEEDED at both axles. This was flagged
   as contingent on prediction 2 holding; prediction 2 failed (mu_fz
   rose rather than staying flat), which pushes the boundary out
   further than a c_alpha-only shift would -- the magnitude miss is
   a direct, expected consequence of prediction 2's failure, not an
   independent surprise.
4. Onset coverage falls from 59.04%/48.84% -- HELD clearly. Under
   pass_2's own boundary and own alpha (diagnostics/inspect_ekf_
   dugoff_circularity.py pass_2, Section 6): front 24.60%, rear
   16.40% -- a large drop in the predicted direction.
5. Self-consistency R^2 (refit curve at pass_2's OWN resulting
   alpha, from running the EKF with the new curve) -- HELD. Front
   0.9526 -> 0.9704 (a moderate rise, as predicted and expected);
   rear 0.9822 -> 0.9824 (essentially flat). The CONJUNCTION warning
   signature is CLEARLY ABSENT: R^2 sits nowhere near the linear
   observer's ~0.997 (front 0.9704, rear 0.9824), and c_alpha moved
   SHARPLY AWAY from the pass_0/1 prior (49.81%/31.70% below it),
   not back toward it. This is the healthy, expected outcome the
   prediction described.
6. h2-vs-ay slope/correlation -- HELD, precisely. Full masked
   population (n=24183): corr=+0.9809 (pass_1's reference: 0.9808,
   a 0.0001 difference). Apex_3 population (n=471): corr=+0.9686
   (pass_1's reference: 0.9682). CORRECTION to the pre-registration's
   own wording, recorded rather than silently fixed: 0.9682/0.9808
   were CORRELATION values, not a "regression slope" as prediction 6
   labelled them -- diagnostics/inspect_ekf_pass2_evaluation.py
   Section 3 reports both explicitly (apex-population regression
   slope of h2_pred on ay_meas = 0.9018, intercept -0.5102) so this
   imprecision is not repeated. The metric is confirmed insensitive
   to the curve refit either way it is read.

ADDITIONAL FINDING, not among the six pre-registered predictions,
recorded because it bears directly on the "flags not yet
interpretable" standing conclusion above: worst-phase-per-corner-
instance flagged counts under CURRENT (kinematic-derived)
thresholds, out of 56, sample-level CS_ratio distribution (diagnostics/
inspect_ekf_dugoff_circularity.py pass_2, Sections 3-4) -- front
strong 8 + moderate 8 = 16 (pass_1: 21+11=32; kinematic: 7+4=11);
rear strong 7 + moderate 2 = 9 (pass_1: 20+7=27; kinematic: 5+4=9,
now EQUAL). Pass 2's flagged counts sit BETWEEN kinematic and
pass_1 at the front and land exactly on the kinematic count at the
rear -- the flagged-count inflation identified in "Threshold
re-derivation deliberately deferred" above has substantially
receded under this refit, without any threshold change. This is
read as corroborating, not conclusive: the deferred re-derivation
decision stands unchanged (thresholds are still kinematic-fitted,
and this is a different sample-level statistic than that entry's
worst-phase-per-instance percentiles, not a direct re-check of it),
but it is a positive sign the pass-1 flag inflation was at least
partly a distribution-shift artifact of an unrefit curve rather than
a stable property of the EKF approach.

OTHER MEASURED QUANTITIES:
- NIS exceedance (diagnostics/inspect_ekf_pass2_evaluation.py,
  R KNOWINGLY MIS-SPECIFIED -- held at pass_1's value, derived from
  the residuals of the curve pass 2 replaces): yaw_rate 2.31% (just
  under the 3-15% band), ay 5.65%, combined 5.37% (both inside),
  combined mean NIS 1.624 (target ~2). Direction is coherent with
  the known mis-specification: pass_2's own fit residuals (RMS
  1078.7 N front / 1346.2 N rear) are roughly 2.5-4x smaller than
  the residuals R_ay was derived from (2752.7/5793.2 N), so R is now
  too generous for this curve -- concrete evidence, not just
  expectation, that R re-derivation (deferred to a later pass) would
  tighten these figures further.
- Sign check: median gate 14/14 all corners, 13/13 racing-speed
  (unchanged from every prior pass). Per-sample pooled fraction
  99.77% (14480/14513) -- a further improvement on pass_1's 99.63%.

CONVERGENCE STATUS after this pass: NOT CONVERGED, as expected for a
first refit iteration -- none of the four relative changes (49.81%,
31.70%, 27.45%, 25.95%) are remotely close to the 5% gate, and the
gate requires two consecutive passes under it regardless. 1 of the
maximum 4 refit passes (pass 2-5) used; 3 remain. Only one data
point exists so oscillation vs. monotonic drift cannot yet be
distinguished -- that needs pass 3. EARLY WATCH ITEM for pass 3,
not a verdict: effective mu moved AWAY from the already-flagged-as-
high 1.90/2.06 toward 2.42/2.59, the wrong direction for failure
mode 3 (physically implausible values) if it continues.

Diagnostics: fit_dugoff_pass2_refit.py, inspect_ekf_pass2_
evaluation.py (both new, this pass), inspect_ekf_dugoff_
circularity.py (existing, parameterised by pass_id, run with
"pass_2"). test_stability.py confirmed unaffected (tyre_model_ekf
has no production consumer).

### WP-N2 pass 3: pre-registered predictions, carrying forward the
c_alpha/mu_fz coupling finding [2026-08-20]

CARRIED FORWARD FROM PASS 2's FAILED PREDICTION: c_alpha and mu_fz
are COUPLED through the fit, not independently identified by the
two-step ORDER. lambda = mu_fz/(2*c_alpha*|tan(alpha)|) -- a change
in c_alpha shifts lambda at fixed alpha, and the mu_fz least-squares
step (run over the same Fy/Fz data, c_alpha fixed from step 1)
compensates by moving mu_fz to keep predicted Fy matched. Fitting
c_alpha before mu_fz sequences the computation; it does not make
mu_fz's optimum insensitive to c_alpha's value. This is why pass 2's
mu_fz prediction failed (+27.45% front / +25.95% rear against a
+/-25% band) even though the stated mechanism (data's peak Fy level
unchanged) is true in itself -- the fit's SENSITIVITY to c_alpha was
the missing piece, not the Fy data changing.

RIDGE CHECK, added this pass rather than assumed: watching the four
parameters independently may not detect a loop where c_alpha and
mu_fz slide together along a ridge of near-equivalent fits (many
(c_alpha, mu_fz) pairs could give similar predicted Fy if they move
together in the right proportion) -- the parameter-wise 5% rule
would keep reporting "not converged" indefinitely in that case even
if the FIT ITSELF has stabilised. Tracked via mu_fz/c_alpha per axle
at pass_0, pass_2 and pass_3 (pass_1 carries pass_0's curve
unchanged, so it contributes no new ratio point). CHECKED, NOT YET
PRESENT as of pass_0->pass_2: front ratio 0.08022 (pass_0) ->
0.20372 (pass_2), a 2.540x change; rear 0.09080 -> 0.16744, a 1.844x
change. The ratio moved substantially at both axles, not less than
the individual parameters -- pass_0->pass_2 shows no sign of
ridge-sliding (a stabilising ratio while the raw parameters keep
moving); it shows a genuine, non-stabilised shift in both the
parameters AND their ratio. Pass 3 is the first point that can test
whether the ratio's OWN step size is shrinking (ridge convergence,
even if the raw parameters keep moving) or continuing to move by a
similar or larger amount (no ridge convergence either).

PRE-REGISTERED PREDICTIONS, before any pass-3 number exists:

1. c_alpha per axle: CONTINUES TO FALL at both axles (same direction
   as pass_0->pass_2, since the EKF's own alpha distribution grew
   larger under pass_2's curve -- pass_2's own resulting |alpha| p99
   is 8.582 deg front / 5.880 deg rear, larger than pass_1's alpha
   the pass-2 fit was trained on). FALSIFIABLE, tied to the
   convergence question directly: the MAGNITUDE of the pass2->pass3
   relative change should be SMALLER than pass2's own step (49.81%
   front / 31.70% rear) if this is genuinely damping. A step equal
   to or larger than pass 2's is recorded as evidence of non-decaying
   drift (failure mode 2), not quietly noted as "still moving."
2. mu_fz per axle: CONTINUES TO RISE (informed by the coupling --
   c_alpha predicted to keep falling, which the fit compensates for
   by raising mu_fz). Same shrinking-step falsifiability as (1): the
   pass2->pass3 percentage step should be SMALLER than pass 2's own
   (+27.45% front / +25.95% rear). Equal or larger is evidence
   against damping.
3. Effective mu per axle: CONTINUES TO RISE from pass_2's 2.42
   front / 2.59 rear. Band, held with LOWER confidence than the
   others (extrapolating a two-point trend): [2.3, 3.2] at both
   axles. Two live explanations, stated explicitly per instruction:
   (a) Level-1 Fz with Cl=0 omits downforce entirely, a documented
   conditional since WP-N1b that would underestimate axle Fz at
   speed and inflate effective mu as a roughly CONSTANT bias across
   every pass; (b) the c_alpha/mu_fz coupling letting the pair drift
   along the ridge, which would produce a bias that GROWS pass over
   pass as the pair keeps sliding. EXPECTED: (b) dominates, because
   Fz estimation has been unchanged pass_0->pass_2 while effective mu
   still grew substantially (1.90/2.06 -> 2.42/2.59) -- a constant
   Fz-driven bias cannot by itself explain a pass-to-pass increase
   when the thing it depends on (Fz) did not change. If effective mu
   keeps climbing at pass 3, that is further evidence for (b); if it
   plateaus while c_alpha/mu_fz still move (a stabilising ratio),
   that would point toward the ridge settling with the Fz bias as
   the remaining, roughly-constant residual offset.
4. Onset boundary and coverage per axle: boundary CONTINUES TO MOVE
   OUTWARD at both axles from 5.816/4.786 deg, following from (1)+(2)
   both continuing in the same direction. Coverage CONTINUES TO FALL
   from 24.60%/16.40% (pass_2's own boundary and alpha), same
   reasoning as pass 2's coverage prediction -- the boundary moving
   out is expected to dominate any shift in the alpha distribution
   itself.
5. Self-consistency R^2 (refit curve at pass_3's OWN resulting
   alpha): front CONTINUES A MODEST RISE toward the 0.97-0.98
   neighbourhood (from 0.9704); rear STAYS ROUGHLY FLAT near 0.98
   (from 0.9824), consistent with pass_1->pass_2's own pattern.
   CONJUNCTION WARNING SIGNATURE RESTATED: would require R^2
   approaching the linear observer's ~0.997 level WHILE c_alpha
   snaps back toward the pass_0/pass_1 prior (132798/174217) --
   predicted ABSENT again, since prediction 1 has c_alpha continuing
   to fall AWAY from that prior, not toward it.
6. Per-sample sign fraction: CONTINUES TO HOLD OR RISE from pass_2's
   99.77% (pass_1: 99.63%, pass_2: 99.77% -- a two-point rising
   trend). Falsifiable floor: predicted to stay at or above 99.5% at
   racing-speed corners; a drop below that would be a genuine
   reversal of the trend, not noise.
7. NIS per channel, R held at pass_1's value throughout (mis-
   specified for whichever curve is current): predicted to stay in
   a similar low range to pass_2's (yaw_rate 2.31%, ay 5.65%,
   combined 5.37%), since R has not moved and the curve keeps
   adapting toward smaller residuals each pass. Band: yaw_rate under
   5%, ay under 10%, combined under 10%. A jump outside this band
   would mean the curve's own residual behaviour changed character
   pass2->pass3, not just magnitude.

Any prediction that fails is recorded as a failed prediction, not
quietly dropped, same standing rule as pass 2.

### WP-N2 pass 3: refit results, prediction verdicts, ridge check,
convergence status [2026-08-20]

REFITTED PARAMETERS (diagnostics/fit_dugoff_pass3_refit.py,
diagnostics/fit_dugoff_pass3_refit_manifest.json, timestamp
2026-08-20T08:10:25Z; config/parameters.json tyre_model_ekf.pass_3):
c_alpha_front=72905.3 N/rad, c_alpha_rear=118729.5 N/rad,
mu_fz_front=12069.8 N, mu_fz_rear=19383.4 N. Effective mu: 2.15
front (down from pass_2's 2.42), 2.52 rear (down from 2.59) --
REVERSED toward plausibility this pass, not away from it.

RELATIVE CHANGE, pass2->pass3, vs the pass0->pass2 step (diagnostics/
inspect_ekf_pass3_evaluation.py Section 0, shrinking-step check):
- c_alpha_front: -49.81% (pass0->2) -> +9.39% (pass2->3) --
  OSCILLATION (sign flip), magnitude shrank (49.81 -> 9.39).
- c_alpha_rear: -31.70% -> -0.22% -- SAME DIRECTION, magnitude
  collapsed to near zero. The smallest step either axle has taken.
- mu_fz_front: +27.45% -> -11.10% -- OSCILLATION (sign flip),
  magnitude shrank (27.45 -> 11.10).
- mu_fz_rear: +25.95% -> -2.72% -- OSCILLATION (sign flip), but the
  magnitude is now small (under the 5% gate).

RATIO TREND (mu_fz/c_alpha, ridge check, Section 0b): front 0.08022
(pass_0) -> 0.20372 (pass_2, +153.95%) -> 0.16555 (pass_3, -18.73%);
rear 0.09080 -> 0.16744 (+84.41%) -> 0.16326 (-2.50%). PATTERN
CHECK, stated plainly per instruction: ridge-sliding (a stabilising
ratio while the raw parameters keep moving) is NOT observed at
either axle as of pass_3. Front's ratio is STILL moving by a large
amount (-18.73%), in step with its still-large raw-parameter moves
-- no dissociation. Rear's ratio moved only slightly (-2.50%), but
so did its raw parameters (-0.22%/-2.72%) -- again no dissociation,
just everything becoming small together, which reads as ordinary
settling rather than the specific ridge-sliding failure mode.

PREDICTION VERDICTS:

1. c_alpha per axle, direction+magnitude -- MIXED, reported
   precisely rather than forced. Front FAILED the stated
   falsifiability (predicted the pass2->3 step would be SMALLER in
   magnitude than pass0->2's AND continue falling; it continued
   falling in the sense that... no, it ROSE, i.e. the DIRECTION
   itself reversed, which the prediction did not anticipate --
   FAILED). Rear HELD on the shrinking-step falsifiability (-0.22%
   is smaller in magnitude than -31.70%, same direction) though the
   step is now near-zero rather than merely smaller.
2. mu_fz per axle, direction+band informed by coupling -- FAILED at
   both axles on direction (predicted continued rise; both fell).
   Magnitude did shrink at both (27.45->11.10 front, 25.95->2.72
   rear), satisfying the secondary shrinking-step criterion even
   though the primary direction call was wrong.
3. Effective mu, band [2.3, 3.2] -- FAILED at both axles: front
   2.15 and rear 2.52 both fell BELOW the band's own floor of 2.3,
   the opposite of the predicted continued climb. REVISITING per
   the pre-registration's own instruction: the prediction favoured
   explanation (b) (c_alpha/mu_fz ridge drift) over (a) (Level-1 Fz
   omission) BECAUSE effective mu had only ever risen so far
   (1.90/2.06 -> 2.42/2.59); a reversal is more evidence for (b),
   not against it -- a roughly-constant Fz-driven bias (a) cannot
   produce a DROP any more than it could have produced the earlier
   rise on its own, since Fz estimation is unchanged throughout.
   The reversal is consistent with (b): the SAME oscillating
   c_alpha/mu_fz pair that flipped direction on its own raw
   parameters flipped effective mu's direction too, for the same
   reason (mu_fz fell while mean Fz over the fit population is
   unchanged pass-to-pass). The band width, not the mechanism
   argument, is what failed here -- (b) predicted correctly that
   mu_fz's own movement would dominate; the two-point-trend
   extrapolation that set the band's location was too confident
   given only one prior data point.
4. Onset boundary (outward) and coverage (falling) -- FAILED at
   both axles, both quantities, as a DIRECT, EXPECTED consequence of
   predictions 1+2 failing (this prediction was explicitly stated as
   contingent on those continuing in the same direction). Boundary:
   front 5.816 -> 4.732 deg (moved INWARD), rear 4.786 -> 4.667 deg
   (also inward, smaller move). Coverage: front 24.60% -> 41.49%
   (ROSE), rear 16.40% -> 26.17% (ROSE) -- both the shrinking
   boundary and continued alpha growth (front |alpha| p99 8.582 ->
   8.509 deg, roughly flat; p50 3.510 -> 3.817 deg, up) combine to
   raise coverage sharply. A clean, well-explained failure cascade,
   not a surprise once 1+2's reversal is known.
5. Self-consistency R^2 with the conjunction warning -- HELD on the
   primary claim (rise continues, conjunction absent), magnitude
   miss on both axles (predicted "modest"/"roughly flat", actual
   rose more): front 0.9704 -> 0.9833 (predicted toward 0.97-0.98,
   landed just above that range); rear 0.9822 -> 0.9885 (predicted
   roughly flat, actual +0.6pp, a real rise not flatness).
   CONJUNCTION CHECK, restated and confirmed absent: R^2 (0.9833/
   0.9885) sits well below the linear observer's ~0.997, AND c_alpha
   is nowhere near snapping back to the pass_0/1 prior -- front sits
   at 54.9% of the original prior (72905/132798), rear at 68.2%
   (118729/174217), both far from the ~100% a "restated prior" would
   show. Rising R^2 alongside genuinely-displaced c_alpha remains the
   healthy pattern the prediction described.
6. Per-sample sign fraction, floor >=99.5% -- HELD on the floor
   (99.72%, comfortably above it) though the DIRECTION reversed
   rather than continued rising: 99.63% (pass_1) -> 99.77% (pass_2)
   -> 99.72% (pass_3), a small dip. Read as noise around a
   consistently-high plateau, not a trend break -- the floor is what
   was pre-registered as falsifiable, and it held.
7. NIS per channel, band yaw<5%/ay<10%/combined<10% -- HELD clearly.
   yaw_rate 1.58% (pass_2: 2.31%), ay 3.93% (5.65%), combined 3.26%
   (5.37%), mean NIS 1.315 (target ~2, moving further below it as R
   grows more stale relative to the shrinking fit residuals -- pass_3
   fit RMS 812.1 N front / 1042.2 N rear, smaller again than pass_2's
   1078.7/1346.2 N). R re-derivation remains deferred; this is
   further, larger evidence it would matter when taken.

SCORECARD: 2 of 7 predictions HELD cleanly (5 with a magnitude
caveat, 7 cleanly), 1 HELD on its falsifiable floor despite a
direction miss (6), 1 mixed (1, front failed / rear held), 3 FAILED
outright (2, 3, 4) -- with 3 and 4 both traceable to the SAME
upstream cause (the front-axle oscillation flipping mu_fz's
direction), not three independent surprises. The exercise did what
pre-registration is for: several genuinely falsifiable claims broke,
and the reason each broke is now on record rather than narrated
after the fact.

FLAGGED COUNTS, reported for continuity only -- NOT comparable to
production verdicts (thresholds remain kinematic-fitted, re-
derivation still deliberately deferred): front strong 8 + moderate 4
= 12 (pass_2: 16; kinematic: 11); rear strong 3 + moderate 4 = 7
(pass_2: 9; kinematic: 9 -- pass_3 now sits BELOW the kinematic
count at the rear). Continuing pass_2's pattern of moving toward,
and now past, the kinematic baseline without any threshold change.

CONVERGENCE STATUS after this pass: NOT CONVERGED. No parameter is
under the 5% gate for two consecutive passes (rear's two parameters
ARE under 5% this single pass -- c_alpha -0.22%, mu_fz -2.72% -- but
that is one data point, not two consecutive ones, and front remains
far over the gate on both parameters). 2 of the maximum 4 refit
passes (pass 2-5) used; 2 remain.
- FRONT: one sign flip observed on both parameters, WITH SHRINKING
  MAGNITUDE (c_alpha 49.81% -> 9.39%, mu_fz 27.45% -> 11.10%). Too
  early to call this failure mode 1 (oscillation as defined requires
  a repeating, NON-decaying pattern) -- a single decaying flip is
  also consistent with a converging oscillatory fixed-point
  iteration, a legitimate numerical behaviour, not itself a failure.
  Pass 4 is the discriminating test: a further flip with continued
  shrinking magnitude supports convergence-via-oscillation; a further
  flip with similar or larger magnitude confirms failure mode 1;
  continuing pass_3's direction with growing magnitude would instead
  be failure mode 2 (monotonic drift).
- REAR: c_alpha held direction with a collapsing step; mu_fz flipped
  sign but at small (under-5%) magnitude. Reads as settling, the most
  convergence-consistent behaviour seen so far in this arc, though
  still only one small step, not two.
- Ridge check: NOT PRESENT at either axle, as stated above -- no
  dissociation between ratio stability and raw-parameter movement.

Diagnostics: fit_dugoff_pass3_refit.py, inspect_ekf_pass3_
evaluation.py (both new, this pass), inspect_ekf_dugoff_circularity.py
(existing, run with "pass_3"). test_stability.py confirmed exit 0,
unaffected (tyre_model_ekf has no production consumer).

### WP-N2 pass 4: pre-registered predictions, the front-axle
discriminating test [2026-08-20]

RULE CLARIFICATION, decided BEFORE pass 4 runs: the convergence
criterion stands exactly as originally written -- ALL FOUR
parameters (c_alpha_front/rear, mu_fz_front/rear) under 5% relative
change for TWO CONSECUTIVE passes. PER-AXLE convergence is not
convergence. If rear satisfies the gate while front does not, that
is reported as a distinct finding (one axle settling, the other
not), and the loop is NOT declared converged on that basis. This
closes off a way the criterion could otherwise be read generously
after the fact, now that rear's pass_3 step (c_alpha -0.22%, mu_fz
-2.72%) already sits under 5% while front's does not.

PASS 4 IS THE DISCRIMINATING TEST FOR THE FRONT AXLE. Front c_alpha
and mu_fz both flipped sign at pass 3 with sharply shrinking
magnitude (c_alpha 49.81% -> 9.39%; mu_fz 27.45% -> 11.10%). Decision
rule fixed in advance, before pass 4's numbers exist:
- DAMPING (converging oscillation) if pass 4 flips sign again with
  magnitude continuing to fall substantially -- roughly 2-4% or
  lower.
- FAILURE MODE 1 (oscillation) if the sign flips again while
  magnitude stops shrinking or grows.
- AMBIGUOUS MIDDLE, a legitimate third outcome, not forced into
  either: sign flip with magnitude falling only slightly (say
  6-8%).
EXPECTED: DAMPING, held as the primary prediction rather than a
neutral toss-up -- a >5x shrink in one step (49.81% -> 9.39% for
c_alpha) is the textbook signature of a converging oscillatory
fixed-point iteration, not typically what a genuinely non-convergent
bounce looks like at its first reversal. This is a real, checkable
call, not a hedge: front c_alpha and mu_fz are both predicted to flip
sign again at pass 4, with magnitude in the 2-4%-or-lower range.

PRE-REGISTERED PREDICTIONS. Where pass 3 already failed by
over-extrapolating a single prior data point (predictions 2/3/4
there), this entry predicts DIRECTION with magnitude stated as
unpredicted wherever only one relevant data point exists, and only
gives a numeric band where a multi-pass trend actually supports one.

1. c_alpha per axle: FRONT -- flips sign again (falls), magnitude
   2-4% or lower under the damping reading (see discriminating test
   above; this IS the falsifiable claim, not a separate one). REAR --
   continues its established two-step pattern (two consecutive
   same-direction, shrinking steps: -31.70% then -0.22%): direction
   uncertain at this magnitude (could tip either way near a settling
   point) but MAGNITUDE predicted small, under 5%, continuing the
   shrinking trend rather than reopening it.
2. mu_fz per axle: FRONT -- flips sign again (rises), magnitude
   2-4% or lower, via the same coupling mechanism established at
   pass 2/3 (c_alpha and mu_fz move oppositely as the fit
   compensates). REAR -- magnitude predicted small (under 5%,
   informed by pass_3's already-small -2.72% step and c_alpha_rear's
   own near-flat behaviour), direction unpredicted with confidence.
3. Effective mu per axle: trajectory front 1.90 -> 2.42 -> 2.15,
   rear 2.06 -> 2.59 -> 2.52. PREDICT CONTINUED MOVEMENT TOWARD THE
   PASS_0 RANGE (front further down from 2.15, rear roughly flat to
   slightly down from 2.52), NOT a further reversal upward. Reasoning
   stated plainly: this is the same mechanism as prediction 1/2's
   damping call -- if front's c_alpha/mu_fz oscillation is genuinely
   damping, mu_fz_front's swing amplitude shrinks each pass, so any
   pass-4 overshoot past pass_0's value should be smaller than
   pass_2's overshoot was, pulling effective mu back toward (not
   further from) the pass_0 anchor even without landing on it exactly.
   No numeric band given -- only the trajectory's two prior points
   inform this, same caution as pass 3's over-confident band that
   failed.
4. mu_fz/c_alpha ratio, ridge check continued: FRONT predicted to
   move by LESS than pass_2->3's -18.73% step, consistent with the
   damping call above, still with no ridge-sliding dissociation
   expected (ratio and raw parameters should keep moving together,
   not one stabilising while the other doesn't). REAR predicted to
   again move very little (under 5%), continuing the "everything
   small together" pattern rather than a distinctive ridge signature.
5. Onset boundary and coverage per axle: FRONT -- if prediction 1/2's
   damping call holds, onset REVERSES AGAIN, moving back outward from
   4.732 deg (partial retrace toward, not necessarily reaching,
   5.816 deg), and coverage falls back from 41.49% toward a lower
   value. REAR -- boundary and coverage both predicted to stay close
   to pass_3's 4.667 deg / 26.17%, small movement only, tracking
   rear's own near-flat raw parameters.
6. Self-consistency R^2 with the conjunction signature restated:
   R^2 has risen EVERY pass so far regardless of which direction the
   raw parameters moved (front 0.9526 -> 0.9704 -> 0.9833; rear
   0.9822 -> 0.9824 -> 0.9885) -- predict this monotonic rise
   CONTINUES: front toward roughly 0.985-0.99, rear toward roughly
   0.988-0.992, both still comfortably below the linear observer's
   ~0.997. CONJUNCTION CHECK RESTATED: would require R^2 approaching
   ~0.997 WHILE c_alpha snaps back toward the pass_0/1 prior
   (132798/174217) -- c_alpha is currently at 54.9% front / 68.2%
   rear of that prior; predicted ABSENT again, since prediction 1
   has c_alpha_front falling FURTHER from the prior this pass (a
   damping flip means falling again from 72905, moving further below
   both pass_2's 66647 and further still below the prior), not toward
   it.
7. Per-sample sign fraction: has oscillated in a narrow high band
   (99.63 -> 99.77 -> 99.72%). Predict it stays in that same
   99.5-99.9% neighbourhood -- same falsifiable floor as before
   (>=99.5%), direction not predicted with confidence given the last
   two passes already moved in opposite directions by similar small
   amounts.
8. NIS per channel: UNLIKE predictions 1-5 above, this one rests on
   a genuine multi-pass trend, not a single point -- fit RMS
   residuals have shrunk EVERY pass regardless of the c_alpha/mu_fz
   oscillation direction (WP-N1b's original 2752.7/5793.2 N -> pass
   2's 1078.7/1346.2 N -> pass 3's 812.1/1042.2 N, monotonic both
   axles across three measurements). R remains held at pass_1's
   value throughout. Predict NIS exceedance continues LOWER than
   pass_3's already-low readings (yaw_rate 1.58%, ay 3.93%, combined
   3.26%): band yaw_rate<3%, ay<7%, combined<7% -- tighter than pass
   3's own pre-registered band, because this is extrapolating an
   established trend rather than a single point.

Any prediction that fails is recorded as a failed prediction, same
standing rule as every prior pass.

### WP-N2 pass 4: rear mu_fz fit failure (failure mode 3), front
oscillation verdict, arc STOPPED [2026-08-20]

REFITTED PARAMETERS (diagnostics/fit_dugoff_pass4_refit.py,
diagnostics/fit_dugoff_pass4_refit_manifest.json, timestamp
2026-08-20T08:34:44Z; config/parameters.json tyre_model_ekf.pass_4):
c_alpha_front=65134.4 N/rad, c_alpha_rear=114873.3 N/rad,
mu_fz_front=13141.8 N. mu_fz_rear=8,484,797.3 N -- effective mu
1102.5. THIS IS NOT A NORMAL RESULT, recorded verbatim per the
project's own "a numbered pass records what the fit actually
produced" principle, not corrected or discarded.

REAR mu_fz FIT FAILURE (failure mode 3, physically implausible
values, exactly as pre-registered as a possible outcome when this
arc began): the bounded least-squares search hit its own widened
bracket ceiling after 4 widen attempts (mu_fz_bound_fraction=
0.99999998, "STILL HIT BOUND, not an interior optimum" per the
refit script's own printed diagnostic). DIAGNOSIS: the mu_fz
objective is only constrained by samples where the Dugoff model
actually saturates (lambda=mu_fz/(2*c_alpha*|tan(alpha)|) < 1).
c_alpha_rear this pass (114873.3) is close to pass_3's (118729.5,
essentially unchanged, -3.25%), but this pass's own EKF alpha
population apparently leaves too few or no samples demanding
saturation at any mu_fz the search bracket could represent before
widening ran out -- the optimizer drove mu_fz to the bracket edge
instead of finding a minimum. CONFIRMED, not just inferred, by
Section 6 of diagnostics/inspect_ekf_dugoff_circularity.py pass_4:
onset boundary = 88.449 deg (a physically meaningless angle -- no
real tyre slip range approaches this) and coverage = 0.0000 --
LITERALLY ZERO of 24183 masked samples exceed this pass's rear
onset. The rear Dugoff curve this pass is PURE LINEAR
(Fy_r=c_alpha_rear*tan(alpha_r), no representable saturation
anywhere in the visited data) -- structurally the SAME failure mode
that condemned the linear Kalman observer at the start of this arc
(WP-S4/S5/S5b/S6, "a state observer built on a linear tyre model
cannot detect departure from tyre linearity"), arrived at here via a
degenerate fit rather than a deliberate linear model choice. The
irony is exact: the refit loop, designed specifically to escape the
linear observer's structural blind spot, produced a rear axle that
is once again linear, this time by construction failure rather than
design.

FRONT-AXLE DISCRIMINATING TEST VERDICT, against the three-way rule
fixed in advance: c_alpha_front flipped sign again (pass2->3 +9.39%,
pass3->4 -10.66%) with magnitude GROWING (9.39 -> 10.66), not
shrinking -- by the pre-registered rule this is unambiguously
FAILURE MODE 1 (oscillation), not damping. mu_fz_front also flipped
sign (pass2->3 -11.10%, pass3->4 +8.88%) with magnitude shrinking
somewhat (11.10 -> 8.88) but not into the damping range (2-4%) and
just outside the pre-registered ambiguous-middle band (6-8%) --
closer to ambiguous than to clean failure-mode-1, but not a match
for either boundary. VERDICT, not forced into false unanimity: the
two coupled parameters give SLIGHTLY DIFFERENT readings this pass
(c_alpha clearly failure-mode-1, mu_fz borderline-ambiguous), and
c_alpha's reading is treated as the more decisive of the two (a
clean, unambiguous non-shrink against the pre-registered numeric
bands, vs mu_fz's own reading which sits in a genuinely grey zone).
Overall front-axle reading: FAILURE MODE 1, or at best an unresolved
muddle -- NOT the damping outcome predicted as primary.

RATIO TREND (ridge check, four points now): front 0.08022 -> 0.20372
-> 0.16555 -> 0.20177 (step pass3->4 = +21.87%, still moving by a
large amount, no ridge-sliding dissociation from the raw parameters,
which also moved by a large amount). Rear 0.09080 -> 0.16744 ->
0.16326 -> 73.86225 (step pass3->4 = +45142.87%) -- the ratio's own
blowup, mechanically identical to and fully explained by the rear
mu_fz fit failure above, not a separate finding.

PREDICTION VERDICTS (8 pre-registered):

1. c_alpha direction/magnitude -- FRONT FAILED (predicted damping,
   2-4% magnitude; got failure-mode-1, magnitude grew to 10.66%).
   REAR the falsifiable claim (small magnitude, under 5%) HELD
   (-3.25%), though direction classification is GROWING not
   shrinking relative to pass_2->3's own -0.22% step -- a small but
   real departure from rear's prior settling pattern, plausibly
   contaminated by the rear mu_fz blowup changing which samples
   qualify as rear's own linear-regime population this pass
   (c_alpha_source_mask_n jumped to 13066 from pass_3's 10517).
2. mu_fz direction/magnitude -- FRONT FAILED on the damping claim
   (see discriminating-test verdict above). REAR CATASTROPHICALLY
   FAILED -- predicted small movement under 5%; actual step
   +43673.42%, the fit failure itself.
3. Effective mu trajectory -- FRONT HELD on direction (2.15 -> 2.34,
   continued movement, though UP not further down as predicted --
   partial miss: predicted "further down from 2.15" specifically,
   actual rose to 2.34, still well inside the historical 1.90-2.59
   range and consistent with the mechanism argument (b) even though
   the specific direction call was wrong this pass, since the front
   oscillation's own sign flip naturally moves effective mu with it).
   REAR MOOT/FAILED -- 1102.5 is not a value the prediction's
   trajectory-continuation framing anticipated or could have; this
   is the fit failure, not an effective-mu finding in its own right.
4. mu_fz/c_alpha ratio -- FRONT the "moves by less than -18.73%"
   claim FAILED (actual +21.87%, comparable magnitude, opposite
   sign -- consistent with, not independent of, prediction 1/2's
   front failure). REAR FAILED entirely (the fit-failure blowup).
5. Onset/coverage -- FRONT HELD, precisely: predicted onset
   "reverses again, moving back outward... partial retrace toward,
   not necessarily reaching, 5.816 deg" -- actual landed at 5.761
   deg, almost exactly at that ceiling. Coverage predicted to fall
   back from 41.49% toward a lower value -- actual 27.08%, HELD on
   direction. REAR NOT MEANINGFULLY EVALUABLE -- onset 88.449 deg /
   coverage 0.0000 are the fit failure's signature, not a
   coverage-trend result.
6. Self-consistency R^2 with conjunction -- THE MONOTONIC-RISE
   CLAIM FAILED at both axles: front 0.9704 -> 0.9833 -> 0.9712 (the
   pass_2->3 rise did NOT continue, it reversed back down to
   essentially pass_2's level); rear 0.9822 -> 0.9885 -> 0.9824
   (also reversed down). CONJUNCTION CHECK ITSELF HELD (the more
   important half of this prediction): R^2 stays well below the
   linear observer's ~0.997 at both axles (0.9712/0.9824), and
   c_alpha_front is now FURTHER from the original prior (49.05% of
   132798, vs pass_3's 54.9%) not closer -- the danger signature
   remains absent. NOTE on rear's interpretability: rear's curve is
   now the degenerate pure-linear fit, so its R^2 this pass measures
   a straight-line fit's quality, not a saturating Dugoff curve's --
   not directly comparable to prior passes' rear R^2 in kind, only
   in the narrow sense of "still not approaching 0.997."
7. Per-sample sign fraction, floor >=99.5% -- HELD (99.76%,
   14478/14513), continuing the narrow 99.5-99.9% band every pass
   has shown.
8. NIS per channel, band yaw<3%/ay<7%/combined<7% -- HELD clearly:
   2.02%/5.67%/5.09%, mean NIS 1.583. Worth noting explicitly: this
   was the one prediction built on a genuine multi-pass trend rather
   than a single point, and it is also the one prediction category
   that held cleanly through a pass where three raw-parameter
   predictions failed outright -- the state-estimation/NIS behaviour
   is evidently more robust to the curve's own instability than the
   curve's own parameters are to each other.

SCORECARD: 1 held cleanly (8), 1 held on its floor (7), 2 partially
held (5 held cleanly at front/moot at rear; 6's conjunction-absence
half held, monotonic-rise half failed), 4 failed outright or were
rendered moot by the rear fit failure (1 mixed-toward-fail, 2, 3
mixed-toward-fail, 4). The predictions that survived are exactly
the ones NOT mechanically downstream of the c_alpha/mu_fz pair's own
behaviour (NIS, sign fraction, the conjunction-absence check) --
everything directly reading the pair's own trajectory failed or
degenerated this pass, which is itself informative: the coupling
identified after pass 2 is not a minor wrinkle, it is now the
dominant source of this arc's instability.

CONVERGENCE STATUS: NOT CONVERGED. Per the rule clarification fixed
before this pass ran, per-axle convergence does not count -- but no
axle converges anyway this pass: front fails the 5% gate on both
parameters (10.66%, 8.88%) and rear's mu_fz is not merely over the
gate but not a real fitted value at all. 3 of the maximum 4 refit
passes (pass 2-5) used; 1 (pass 5) remains under the numeric cap.

RECOMMENDATION, not a unilateral decision: STOP HERE, do not run
pass 5. Two independent grounds, either alone sufficient under the
standing rule ("on any of these: stop, do not keep iterating hoping
it self-resolves"): (a) failure mode 3 has unambiguously triggered
at the rear -- refitting pass 5 from a curve whose rear axle
represents zero saturation anywhere in the data has no principled
reason to self-correct, and continuing would mean feeding a known-
degenerate alpha population into another refit; (b) the front axle's
own discriminating test came back FAILURE MODE 1 (or at best
unresolved), not the damping outcome that would have justified
continuing to look for a settling point. This is recorded as a
recommendation for the next turn to act on or override, not as an
autonomous decision to skip pass 5 -- the pass-count cap itself
still permits one more pass; the failure-mode criteria are the
reason to stop short of it.

EFFECTIVE-MU TRAJECTORY: front 1.90 -> 2.42 -> 2.15 -> 2.34 (bouncing
within a band, not trending toward either extreme); rear 2.06 ->
2.59 -> 2.52 -> 1102.5 (the fit failure, not a trajectory point).
Failure mode 3 is not "receding" as hoped after pass 3 -- it has now
OCCURRED, at the rear, decisively. Front alone shows no clear drift
toward implausibility (bouncing in the 2.15-2.42 neighbourhood, well
short of anything alarming on its own).

FLAGGED COUNTS, continuity only, NOT comparable to production
verdicts: front strong 10 + moderate 5 = 15 (pass_3: 12; pass_2: 16;
kinematic: 11); rear strong 4 + moderate 3 = 7 (pass_3: 7, unchanged;
pass_2: 9; kinematic: 9). Rear's count is computed from the
degenerate linear curve this pass -- reported for continuity, not
read as a meaningful saturation-detection result.

Diagnostics: fit_dugoff_pass4_refit.py, inspect_ekf_pass4_
evaluation.py (both new, this pass), inspect_ekf_dugoff_
circularity.py (existing, run with "pass_4"). test_stability.py
confirmed exit 0, unaffected (tyre_model_ekf has no production
consumer).

### WP-N2 refit loop: NON-CONVERGENCE, rear degeneracy to a
pure-linear curve, and the identifiability limit [2026-08-20]

DECISION: the refit iteration is STOPPED at pass 4, one pass short
of the arbitrary four-pass cap. Stopped on the pre-registered
failure criteria, not on the cap. Both grounds triggered
independently:
- Rear: failure mode 3. The bounded mu_fz search hit its widened
  bracket ceiling after 4 widen attempts, landing at mu_fz_rear =
  8,484,797 N, effective mu 1102.5. Confirmed directly rather than
  inferred: the onset boundary is 88.449 deg and coverage is
  exactly 0.0000 -- not one of 24,183 samples exceeds it. The rear
  Dugoff curve has degenerated to pure-linear
  (Fy_r = c_alpha_rear * tan(alpha_r)) with no representable
  saturation anywhere in the data.
- Front: failure mode 1. Against the three-way rule fixed in
  advance, c_alpha_front flipped sign with magnitude GROWING
  (9.39% -> 10.66%), which the rule designates as oscillation, not
  damping. mu_fz_front shrank only slightly (11.10% -> 8.88%),
  outside the ambiguous-middle band and closer to ambiguous than
  to either boundary. Weighting c_alpha's cleaner signal: front is
  failure mode 1, or at best unresolved.

ESTABLISHED -- parameter trajectory across the loop:
- c_alpha_front (N/rad): 132797.9 -> 66647.5 -> 72905.3 ->
  65134.4. Relative steps: -49.81%, +9.39%, -10.66%.
- c_alpha_rear: 174217.3 -> 118993.1 -> 118729.5 -> 114873.3.
  Steps: -31.70%, -0.22%, -3.25%.
- mu_fz_front (N): 10653.1 -> 13577.4 -> 12069.8 -> 13141.8.
  Steps: +27.45%, -11.10%, +8.88%.
- mu_fz_rear: 15818.8 -> 19924.5 -> 19383.4 -> 8,484,797.3.
- mu_fz/c_alpha ratio, front: 0.08022 -> 0.20372 -> 0.16555 ->
  0.20177. Ridge-sliding was checked at every pass and was NOT
  present -- the ratio kept moving as much as the raw parameters,
  so the parameters were not dissociating along a ridge of
  near-equivalent fits.
- Effective mu, front: 1.90 -> 2.42 -> 2.15 -> 2.34, bouncing
  within a band rather than drifting. Rear: 2.06 -> 2.59 -> 2.52
  -> 1102.5, the failure rather than a trajectory point.
- Rear onset coverage across the arc: kinematic 6.95%, pass_0
  36.8%, pass_1 48.84%, pass_2 16.40%, pass_4 0.0000%.

MECHANISM -- self-starvation, and it was pre-registered as a risk:
- Each refit lowers c_alpha, which pushes the onset boundary
  tan(alpha) = mu_fz/(2*c_alpha) OUTWARD, which leaves a smaller
  fraction of samples in the saturating region, which weakens
  mu_fz's identifiability, which allows mu_fz to drift upward,
  which pushes onset further out. Positive feedback terminating in
  degeneracy.
- The risk was recorded at pass 0, before any refit ran: the
  saturation-coverage check found only 6.95% of rear samples past
  onset and flagged rear mu_fz identifiability as a measured
  concern, naming it as the first explanation to test if the rear
  refit failed to settle across passes. It failed to settle, and
  this is why. A pre-registered risk that materialised, not a
  surprise.

SIGNIFICANCE -- what the rear degeneracy says:
- The rear curve collapsed to pure-linear: structurally the SAME
  blind spot that condemned the linear Kalman observer at the
  start of this arc, arrived at here by fit degeneracy rather than
  by deliberate model choice.
- SECOND READING, recorded as the more likely explanation and
  directly connected to the parked combined-slip item: the rear
  axle of a rear-engined RWD car does not principally saturate
  LATERALLY. It saturates under traction on corner exit. Its
  pure-lateral slip angle therefore stays modest precisely because
  its limit is being reached longitudinally. A pure-lateral model
  searching for rear saturation in slip angle finds little to
  identify -- not because the car never reaches the limit, but
  because the model is looking in the wrong dimension. On this
  reading the rear fit did not fail from a poor search; it failed
  because there is genuinely little pure-lateral rear saturation
  in this data. Marked EXPECTED, not established -- it is
  consistent with the coverage trajectory and with the WP-S1
  wheel-speed evidence already recorded, but has not been tested.

WHAT IS NOT INVALIDATED, stated plainly so the scope of the
failure is not overread:
- This is a finding about PARAMETER IDENTIFIABILITY, not about
  whether the filter works. Every result established upstream of
  the curve stands: calibration moved the filter's slip angles
  FURTHER from its own assumed curve rather than closer; its slip
  angles explain measured ay better than the kinematic estimate's
  at identical samples (0.968 vs 0.887, n=471); and it roughly
  halves the reference-stiffness swing WP-S4b recorded as
  defective (rear per-corner spread 6.27 -> 2.12; the WP-S4b
  sample set 4.24x -> 2.27x) while removing a negative-stiffness
  tail (kinematic rear p5 -17,265 N/rad, pass_1 +66,695).
- CORROBORATING PATTERN worth recording: at pass 4, every
  prediction that survived (per-sample sign fraction 99.76%, NIS
  2.02%/5.67%/5.09%, and the conjunction-absence half of the R^2
  check) is one that does NOT mechanically depend on the
  c_alpha/mu_fz pair. Every prediction reading that pair directly
  failed or degenerated. The instability is localised to curve
  identification.

OPEN, not decided here:
- Which pass, if any, is carried forward as the estimator. That
  choice must NOT be made by selecting whichever pass looks best
  in hindsight -- it requires a criterion stated before the
  candidates are compared. Its own decision.
- Whether the combined-slip formulation (PARKED, see its own
  entry) resolves the rear identifiability limit. The mechanism
  above is the strongest argument yet for attempting it.
- R re-derivation, deliberately deferred through the refit loop,
  is now measurably stale: pass 4's fit residuals are far below
  the 2752.7 / 5793.2 N the current R was derived from, and NIS
  has drifted under its band accordingly.

### WP-N2 carry-forward decision: pass 1 [2026-08-20]

DECISION: the estimator carried forward is PASS 1's configuration --
the pass_0 Dugoff curve (c_alpha_front 132797.9, c_alpha_rear
174217.3, mu_fz_front 10653.1, mu_fz_rear 15818.8 N/rad and N
respectively; read exact values from config/parameters.json
tyre_model_ekf.pass_1, not from this prose) with pass 1's calibrated
noise model (R redefined as total innovation uncertainty, 2-D
sweep-refined R_ay_var/R_yaw_rate_var).

RULE, stated before any comparison, and why it is not
hindsight-selected: pass 1 is the LAST configuration whose
parameters were NOT produced by the refit iteration. Passes 2, 3 and
4 are all outputs of a loop that has since been established as
non-converging (thesis_notes.md "WP-N2 refit loop: NON-CONVERGENCE,
rear degeneracy to a pure-linear curve, and the identifiability
limit"), and pass 4's rear curve is degenerate outright (mu_fz_rear
8.48e6 N, onset 88.449 deg, coverage exactly zero). The rule turns on
the PROVENANCE of the parameters, not on how any pass scored -- it
would select pass 1 regardless of which pass happened to produce the
best numbers on any given metric. This is recorded explicitly because
the alternative -- choosing by comparing outcomes now that all four
passes are known -- would be retrofitting, which the numbered-pass
design (each pass fully reproducible from its own recorded
parameters, changed_from_previous stating exactly what moved) exists
to prevent. A provenance rule fixed before comparison is the same
discipline this whole arc has applied to predictions: state the
criterion first, then let the data satisfy or fail it, never the
reverse.

CARRIED-FORWARD LIMITATION, recorded alongside the decision rather
than buried in a later entry: pass 1's curve was fitted from
KINEMATIC slip angles (WP-N1b), which are documented elsewhere in
this notebook to under-read mid-corner. That circularity is exactly
what the refit loop (passes 2-4) was built to break, and it was NOT
broken -- the loop failed on its own pre-registered failure criteria
before producing a curve independent of the kinematic estimate. The
carried-forward estimator therefore retains a known, documented
dependency on the very estimate it was meant to improve upon. This is
a STATED LIMITATION of the method going forward, not a resolved
issue, and must be presented as such wherever pass 1's configuration
is described or used -- including in the thesis write-up, where it
is the honest boundary of what this arc achieved: a better-calibrated
noise model on a still-kinematic-sourced curve, with the
saturation-detection improvement over the linear observer intact
(pass 1 flags real instances where the linear observer flagged none)
but the curve-identification half of the original ambition unmet.

### WP-N2 pass 1: final validation baseline for combined-slip
comparison [2026-08-20]

PURPOSE: consolidates checks already established at various points
this session into ONE run, ONE timestamp, ONE manifest, for the
carried-forward estimator (pass 1's configuration -- pass_0's Dugoff
curve, pass_1's calibrated noise model). Introduces NO new findings;
every section below states where its result was first established.
Diagnostics/inspect_pass1_final_validation.py (new, read-only),
diagnostics/pass1_final_validation_manifest.json (new, machine-
readable copy of every number below). Run against commit
76fc57673f4c2c618363809ba7c09aca226be4ba, 2026-08-20T09:12:21Z.

R RE-DERIVATION: decided NOT NEEDED (see PLAN.md NOW for the full
reasoning). Pass 1's R was derived from and NIS-gated against this
exact curve and configuration -- Section 1 below reproduces those
same acceptance figures directly, confirming the empirical grounds
rather than merely restating the provenance argument. Two refinement
opportunities remain open, recorded here so they are not silently
forgotten: Q was never swept (nothing has diagnosed a reason to move
it), and the accepted R is one interior point on a coarse 5x5 grid
with no finer search run around it. Neither is a defect -- pass 1's R
already satisfies its own pre-registered gate at the values in hand.

0. CONFIG, read live: c_alpha_front=132797.90, c_alpha_rear=
   174217.33, mu_fz_front=10653.12, mu_fz_rear=15818.77;
   Q_beta_var=9.632e-7, Q_yaw_rate_var=2.408e-7; R_ay_var=3.78418,
   R_yaw_rate_var=0.0060476.
1. NIS PER CHANNEL (first established: WP-N2 pass 1 "acceptance
   criteria" entry). yaw_rate exceedance=10.01%, ay=9.18%, combined=
   13.77%, combined mean NIS=2.907 (target ~2). All inside the
   pre-registered 3-15% band; the combined exceedance figure (13.77%)
   was not separately quoted before, recorded here for completeness.
2. SIGN CHECK (first established: same entry). Median gate 14/14 all
   corners, 13/13 racing-speed. Per-sample pooled fraction 99.63%
   (14460/14513), reproduces exactly.
3. SELF-CONSISTENCY R^2, simplified conjunction framing (first
   established: "Circularity and flag attribution at the calibrated
   setting"). Front R^2=0.9526, RMS=1443 N; rear R^2=0.9822, RMS=
   1185 N -- both well below the linear observer's ~0.997. SIMPLIFIED
   FRAMING, stated in the script's own output: pass 1 never refit
   c_alpha, so the two-part conjunction signature used in the pass
   2-4 refit-loop entries (R^2 near 0.997 AND c_alpha snapping back
   to a prior) does not apply -- there is no prior/posterior
   distinction to snap back to. The single relevant comparison is R^2
   against ~0.997, and pass_1 sits well clear of it -- the original
   load-bearing evidence that calibration moved the filter's slip
   angles away from restating its own assumed curve, not toward it.
4. ONSET AND COVERAGE (first established: "Circularity check: pass-0
   EKF vs the rejected linear observer" / calibrated-setting
   follow-up). Front onset=2.297 deg, coverage=59.04%; rear onset=
   2.599 deg, coverage=48.84%. Kinematic reference: front 34.0%,
   rear 6.95%.
5. h2-VS-ay, APEX_3 POPULATION (first established: WP-N2 pass-0-run
   entry, kinematic reference 0.887; re-measured with pass_1's own
   alpha in the pass 1 acceptance entry as 0.9682). This run: n=471,
   corr=+0.9679 -- a 0.0003 reproduction variance against the prior
   0.9682 figure, not a discrepancy of substance; recorded plainly
   rather than silently rounded to match.
6. WP-S4b REFERENCE-SPREAD COMPARISON (first established: "WP-S4b:
   observer self-consistency and the Cr_A inflation finding";
   re-measured for pass_1 specifically in "WP-N2 pass 2: proposal and
   pre-registered predictions"). Kinematic: 79,523-337,111 N/rad
   (ratio 4.24x). Pass_1: 78,117-177,550 N/rad (ratio 2.27x).
   Reproduces exactly.
7. FILTER STABILITY (new consolidation -- these were pre-registered
   gates at pass 1 and the baseline was incomplete without them).
   C2 excursion window, t=883.0-885.5s: pass_0 reference max|beta|=
   14.119 deg, max single-step=10.826 deg, SIGN-FLIPPING; pass_1
   (this run) max|beta|=4.327 deg, max single-step=1.913 deg, at
   t=883.004s (+2.053 -> +0.140 deg) -- SAME-SIGN, reproduces the
   pass_1 acceptance entry's C2 gate exactly. NUANCE, checked and
   reported precisely rather than left ambiguous: a separate,
   smaller single-step sign crossing (1 flip) exists elsewhere in
   the 2.5s window -- NOT at the max-step location, and NOT itself
   flagged as anomalous by any check (the max-step statistic already
   captures the window's largest discontinuity, and that one is
   same-sign). Consistent with ordinary beta oscillation across a
   corner sequence rather than a second discontinuity; the exact
   sample was not printed by this run and is not otherwise
   characterised here. diverged_mask fraction over the full masked
   population: 0.79% (192/24183) -- a new figure, not previously
   recorded as a direct pass_1 statistic. THRESHOLDS, read live:
   nis_window_samples=20, nis_chi2_bound=5.99, nis_flag_fraction=0.5
   -- CAVEAT, stated in the script's own output: these remain the
   ORIGINAL PLACEHOLDER defaults, never validated against a real run.
   The short-run blind-spot quantification already on record
   (thesis_notes.md "blind-spot quantification") was measured
   against a DIFFERENT, never-implemented threshold pair
   (nis_window_samples=25, nis_flag_fraction=1.0) and does NOT
   describe the monitor actually running here -- that validation
   remains undone.
8. DESCRIPTIVES AND PROVENANCE (new consolidation, so this baseline
   is interpretable without reconstructing the session). Beta (deg,
   masked population): p1=-4.747, p25=-1.593, p50=+0.249, p75=+2.080,
   p99=+5.090, max|beta|=6.843. Masked population n=24183 (moving &
   ~kerb & valid-lap racing time). Git commit at run time:
   76fc57673f4c2c618363809ba7c09aca226be4ba. Run timestamp:
   2026-08-20T09:12:21Z.

STANDING NOTE: this entry is the citable reference point for
combined-slip comparison work. It freezes already-established facts;
it does not supersede or reinterpret any of them.

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

### Accuracy-level registry consolidated [2026-07-26]
- config/parameters.json's accuracy_levels block is now the single
  source for every per-quantity accuracy tag -- extended from six
  entries to the full eleven-node set (mass, cog_position,
  yaw_inertia, steering_ratio, lateral_force_split, sideslip_angle,
  speed, yaw_rate, steering_angle, lateral_acc, wheelbase_m), each
  {level, source, capped_by}. prepare_vehicle_state's inline
  {speed:1, yaw_rate:3, steering_angle:1, lateral_acc:1} dict and
  estimate_lateral_forces's hardcoded accuracy_level:1 are deleted;
  both now read the registry at call time. Zero behaviour change --
  every level value is unchanged, only its source moved from a
  hardcoded literal to config.
- Two distinct capped_by mechanisms, not one, resolving the study
  document's open weakest-link question with code evidence rather
  than speculation: "chained-constant: <name>" when a Level-1 config
  constant sits in the node's own derivation (steering_angle <-
  steering_ratio: delta_f_rad = steer_sw_rad / steering_ratio, a
  config constant explicitly noted "treated as constant at Level 1";
  lateral_force_split <- yaw_inertia_kgm2 + corner_weights, both
  Level 1); "provenance-assumption: <name>" when no such constant
  exists in the code but a documented, unverifiable assumption about
  the channel's own measurement caps it anyway (lateral_acc <-
  IMU-not-at-CoG, limitations register item 5; speed <- opaque
  ECU-internal wheel-speed calibration, limitations register item 10
  -- neither ay_mps2 nor v_mps passes through any local Level-1
  constant, both are exact unit conversions only).
- yaw_rate=3 vs steering_angle=1, resolved: yaw_rate_radps is an
  exact rpm-to-radps unit conversion with no config constant in its
  chain and no documented weakening assumption -- neither mechanism
  applies, so it alone keeps the raw logged-sensor tier. steering_
  angle is chain-limited by steering_ratio. Confirms the weakest-link
  hypothesis holds, but only in this two-mechanism form -- the single
  "chained through a constant" reading alone does not explain
  lateral_acc or speed, which needed the second, provenance-assumption
  mechanism instead.
- Whitelist hygiene, same session: config/channels.json's VBOX_
  Lateral_acc/VBOX_Velocity/VBOX_Heading/gpsa_lat/gpsa_long/VBOX_
  Longitudinal_acc entries removed -- verified absent from every
  channel actually present in the real Dubai file (grep of the raw
  log's Time/<channel> blocks). The real file's own log_gps_course
  and log_gps_speed, which WP5b(c)/(d) actually need, are deliberately
  NOT added here -- they land together with their consumers in those
  work packages, not speculatively ahead of them.

### Per-session accuracy resolution + global level cap [2026-07-26]
- modules/accuracy_resolution.py resolves the registry's static mass/
  corner_weights/cog_position nodes against per-outing setup_data
  (Outing.setup_data.car.total_weight and .corner_weight_fl/fr/rl/rr),
  plus an optional global cap (int 1-4, or None for "best available").
  Highest-available-wins, never blended: mass has three source tiers
  (config default; sum of resolved-L2 corner weights, "sum(corner_
  weights)"; explicit setup_data.total_weight, priority above the
  derived sum) and corner_weights has two (config default; all-four-
  present session measurement, never a partial 3-of-4 promotion, to
  avoid mixing a measured corner with a defaulted one in the same
  front/rear fraction split). cog_position is a pure cascade from
  corner_weights' own resolved level, not an independent source list.
- Two node kinds this makes explicit: PURE cascades (cog_position --
  the derived value is nothing but a geometric consequence of the
  input, no ceiling of its own) vs METHOD-CEILINGED cascades
  (yaw_inertia, method_ceiling=1 in the registry -- the m*a*b estimate
  is itself an approximation whose error doesn't shrink just because
  mass/cog_position get better-measured). lateral_force_split inherits
  yaw_inertia's ceiling transitively, which is precisely why session-
  measuring corner weights alone can never lift it past Level 1 --
  Iz remains the binding constraint, not the input data.
- Consistency check (not a blocking gate): when both an explicit
  setup_data.total_weight and all four corner weights are available
  and disagree by >1% relative, a footer warning fires ("session mass
  inconsistent: total X vs corner sum Y") but the explicit total still
  wins per the source-priority order -- no averaging, no refusal.
- ACCURACY CAP IS A VIEWING CHOICE, NOT A REFERENCE-CONFIGURATION
  CHANGE -- the load-bearing distinction for threshold interaction.
  The reference configuration for classification-threshold derivation
  is cap=Best-available with all currently-wired production sources
  active (not every node forced to a theoretical Level 4 -- some,
  e.g. yaw_inertia, may never move past their own method ceiling).
  Selecting a lower cap to compare against a historical run never
  touches what best-available itself yields, so it never triggers the
  standing re-derivation rule (CLAUDE.md, config/parameters.json's own
  classification._comment); only a change to the reference
  configuration itself does -- a new source wired (WP5b(c)/(d)/(f)),
  or real corner weights entered for the first time at an already-
  wired source. On today's real outings (both all-zero setup_data),
  every node resolves identically regardless of cap, so this
  distinction has no live consequence yet -- it is recorded now so the
  first setup_data fill or the first GPS-source WP doesn't have to
  re-derive this reasoning from scratch.
- Both cache identities (WP5 Outing.analysis_data, WP6 in-memory
  _pipeline_cache) gained accuracy_cap + a resolved_vehicle_snapshot
  (resolved values, not just levels -- two different real corner-
  weight measurements could both resolve to Level 2 with different
  numbers, so level alone is not a sufficient identity token). A cap
  change or a setup_data edit since the cache was written invalidates
  the entire Modules-1-5 cache, the same way a csv_path change already
  does -- not a lap-filter-only Module-6 recompute, since mass/corner_
  weights/cog_position feed Module 1 and Module 4a directly.
  ANALYSIS_SCHEMA_VERSION bumped 1->2 for this payload-shape change; a
  pre-WP-C cached payload has none of the new fields and is treated as
  no cache at all on first open, same as any other schema-version
  mismatch.

### WP-C resolver end-to-end acceptance proof [2026-07-26]
- Synthetic outing, real Dubai file, full production path (resolve_
  accuracy -> apply_resolved_vehicle -> Modules 1-4b), three runs:
  baseline (no setup_data), cap=1 with synthetic corner weights, cap=
  Best-available with the same synthetic corner weights. Corner
  weights shifted +2.06 percentage points front of the config split,
  summed to exactly the config mass (1356.0 kg) so the test isolates
  the front/rear split from any mass-magnitude effect.
- cap=1 output is exactly equal to the no-setup-data baseline: Fy_f
  median -128.9731 N in both runs (float-identical) -- confirms a
  capped run genuinely discards the synthetic data rather than just
  relabelling it.
- Best-available picks up the measured split: Fy_f median -135.5190 N,
  delta -6.5458 N from the cap=1/baseline value -- confirms the
  resolved corner-weight split actually reaches estimate_lateral_
  forces through the effective-params override.
- Whole-session CS_ratio_f median stayed flat at 1.000000 across all
  three runs -- not a resolver defect, the known ceiling-clipping
  insensitivity of this exact statistic (CS_ratio is capped at 1.0 and
  most moving samples across a whole session sit at that ceiling
  regardless of the underlying Fy perturbation, same mechanism already
  documented for the Fy yaw-term work and for WP-B's steering-ratio
  upgrade -- a worst-phase-per-corner or per-phase statistic, not a
  flat whole-session median, is where this kind of perturbation
  actually shows up).

### Full channel census + targeted verification (2622 channels) [2026-07-26]
- Complete channel inventory of the raw Dubai log, re-scanned with correct
  cp1252 decoding (the existing diagnostics/scan_channels.py's utf-8/
  errors="replace" read mangles every degree-sign unit into U+FFFD).
  RESULT: no lateral-velocity, sideslip, or optical-sensor channel exists
  anywhere in the log -- vy/v_lat/lateral_vel/vel_y/beta/sideslip/drift/
  attitude/heading/yaw_angle/Correvit/Kistler/optical/math/derived all
  return zero matches. Closes the "is there a hidden measured beta"
  question with direct evidence rather than absence-of-mention.
- log_a_car heading hypothesis tested and REFUTED: d(log_a_car)/dt
  (unwrapped, rad/s) vs sclu_yaw_rate-derived yaw_rate_radps over 3 racing
  laps gives r=-0.001 (median ratio -0.008, should be ~1 if it were
  heading); log_gps_course - log_a_car (angle-wrapped) spreads +-135 deg
  even on straights (|ay|<0.1g) and matches our kinematic beta's sign in
  only 1 of 3 known corners. Channel unidentified, deprioritized -- not a
  usable heading/attitude source.
- corner_radius confirmed as a live, already-logged curvature math
  channel: correlation of 1/corner_radius against ay/v^2 = 0.87, median
  ratio 0.95 (kinematic identity ay=v^2/R essentially confirmed
  end-to-end from the logger's own onboard computation). Raw channel
  stays live everywhere (including straights, where it blows up to
  near-infinite magnitude); corner_radius_filtered is gated OFF entirely
  on straights (zero samples present, not just smoothed) and stays
  physically sane (tens-hundreds of metres) through corners. Both noted
  as future cross-validation candidates for WP1/chair-style corner
  detection -- not wired into anything yet.
- TO_VBOX_01-05: constant 1.0 for the entire session, zero variance,
  inert -- confirms the WP-A whitelist-removal finding rather than hiding
  a disguised computed channel.
- Per-wheel speeds (log_speed_fl/fr/rl/rr, 50 Hz, same rate as ecu_speed)
  and abs_Slip_FL/FR/RL/RR (100 Hz) are both live and dynamic this
  session -- per-wheel speeds differ from ecu_speed by tens of km/h
  through a high-speed corner, abs_Slip shows hundreds of distinct
  values -- but neither is consumed anywhere in modules/.

### Steering ratio Level 1 -> Level 4 lookup (WP-B) [2026-07-26]
- prepare_vehicle_state's delta_f_rad computation now sources
  steering_ratio from config/car_data.json's manufacturer steering_ratio_
  table (21 rows, steering_wheel_angle_deg strictly monotonic, linear
  np.interp with default clamp outside +/-291.5 deg -- never triggered on
  Dubai, log_asteer stays within -130.7/+113.9 deg all session) when that
  local-only file is present and the accuracy cap allows it (Level 4);
  falls back to the 15.7 deg/deg constant (Level 1) otherwise --
  modules/accuracy_resolution.py's third dynamically-wired leaf node
  (mass, corner_weights, steering_ratio), with steering_angle added as a
  fourth pure cascade (cog_position, steering_angle) alongside the
  existing method-ceilinged ones. Graceful degradation verified directly:
  a fresh process with config/car_data.json renamed away resolves
  steering_ratio to Level 1 with the exact config constant, cap<=3 forces
  Level 1 even with the file present (a free ablation lever), and
  test_stability.py's direct, un-resolved call path (raw params, no
  steering_ratio_table key) is byte-identical before/after this WP.
- PARAMETERIZATION, NOT DEVIATION: the 15.7 constant was never a chair
  scientific position -- it is this car's own mechanical steering
  geometry, and the table is manufacturer-digitised data for the same
  car, not an adopted/adapted chair method. No CLAUDE.md deviation-
  taxonomy entry applies, same as mass/corner_weights/wheelbase.
- MODULE 5 REGRESSOR CORRECTION, found verifying the proposal's own claim
  against the code rather than trusting it: the proposal stated "Modules
  1-2 and 5 must remain byte-identical (steering feeds slip angles/4b,
  not beta or the stability regressand)". Half right. delta_f_rad IS one
  of Module 5's five regressors (yaw_stability.py _PREDICTOR_COLUMNS =
  beta_rad, delta_f_rad, v_mps, ax_mps2, az_mps2; stability_analysis.py's
  estimate_yaw_moment_stability passes delta_f=state["delta_f_rad"]
  straight into the ridge fit) -- changing delta_f_rad's values changes
  the fitted dMz/dbeta coefficient even though beta itself never enters
  the computation, because a multi-regressor ridge solve's fitted
  coefficient on one variable depends on every co-regressor's values.
  Verified directly: beta, Fy_f_filt, Fy_r_filt, alpha_r_filt, and
  CS_ratio_r are BYTE-IDENTICAL old (cap=1, L1 constant) vs new (Best
  available, L4 table) -- confirms Module 2, Module 4a, and the REAR
  half of Modules 3/4b are genuinely untouched. delta_f_rad, alpha_f
  (front only), CS_ratio_f (front only), and stability_observed_Nm_per_
  deg all move measurably -- the front-only scoping is a real
  refinement over "Module 4b changes", and the stability regressand
  moving is a real correction over the proposal's own stated
  expectation, not a defect.
- SIGN DISPUTE, resolved by the diagnostic rather than by argument, per
  instruction: neither the original proposal's "reduces alpha_f" nor the
  amendment's "increases alpha_f" is a complete, sign-independent
  statement -- both were oversimplified. The actual mechanism: a lower
  table ratio at high |steering angle| increases delta_f_rad's
  MAGNITUDE in the SAME sign as the steering input (confirmed: raw
  delta_f_rad's tails widen both ways, p5 -0.0870->-0.0889,
  p95 0.0646->0.0651), so alpha_f = delta_f - arctan(...) shifts in that
  same signed direction -- INCREASING at positive-steer samples,
  DECREASING at negative-steer samples, exactly as the amendment argued
  for one sign and the original proposal argued for the other, each
  correct for half the steering range. On this specific session, high-
  steer samples (|steer_sw_deg|>80, n=2643) skew toward the negative
  side (log_asteer's own observed range is -130.7/+113.9 deg, more
  negative-side magnitude available) -- so the AGGREGATE median shift is
  negative: median(alpha_f_new-alpha_f_old) = -0.0028 rad, only 32% of
  high-steer samples show an increase. CS_ratio_f's aggregate median
  shift at the same samples is also negative (-0.0020, 51.9% of samples
  decrease) -- a near coin-flip per-sample, but net downward, consistent
  with the amendment's predicted CONSEQUENCE (CS_ratio_f pushed down)
  even though its stated MECHANISM ("increases alpha_f") was only half
  the story.
- LOCATION PREDICTION (pre-registered, stood as proposed): CONFIRMED.
  Speed-class shift in worst-phase CS_ratio_f median: low corners
  (n=10) -0.0556, medium (n=30) -0.0454, high (n=11) -0.0011 -- low
  shifts most, high shifts least, exactly as predicted. NUANCE: the
  ordering (low > medium > high) is confirmed, but the low/medium
  separation is weaker than the pre-registered prediction implied --
  -0.0556 vs -0.0454 is a modest gap, not the clean step the phase-shift
  finding shows (apex_3 vs entry/exit). Medium-speed corners carry a
  real part of this effect too, not just low; the prediction was
  directionally right but the speed-class boundary is softer than
  "low only" would suggest. Phase shift:
  apex_3's aggregate CS_ratio_f median moves -0.0907 (the largest of any
  phase), exit_4 -0.0120, while entry_1_brake/entry_2_turnin/exit_5 show
  exactly zero aggregate shift -- the same ceiling-clipping mechanism
  already documented for the Fy yaw-term work (most entry/exit-phase
  instances sit pinned at the CS_ratio=1.0 ceiling in both old and new,
  so an aggregate median can't show movement there regardless of the
  underlying perturbation; apex_3, rarely clipped, is where the ratio
  metric is actually sensitive). worst_f_phase itself stays exit_4/
  apex_3-dominated in both old and new (21/15 of 51 each), consistent
  with a real but second-order shift, not a restructuring of which
  phase is worst.
- VERDICT FLIPS under UNCHANGED thresholds: zero, across all 51
  instances (37 normal->normal, 14 moderate->moderate). The underlying
  distributions did shift measurably (see percentiles below), but no
  shift in this specific session crosses a classification threshold
  boundary. Byte-identity of the rear axle and Module 2 (above) means
  the flip-free result is not from lack of a real effect -- the front-
  axle effect is real and measured, it simply doesn't happen to cross a
  boundary in today's data.
- RE-DERIVATION INPUT (percentiles, p5/10/25/50/75/90/95, worst-phase-
  per-corner-instance, the exact statistic thresholds were derived
  against -- reviewer re-derives, nothing re-derived here):
  worst-phase CS_ratio_f OLD (n=49): p5=-0.2526 p10=-0.0162 p25=0.2232
  p50=0.3452 p75=0.5756 p90=0.8342 p95=0.9230. NEW (n=48): p5=-0.2340
  p10=-0.1639 p25=0.2119 p50=0.2900 p75=0.5234 p90=0.8171 p95=0.8842.
  worst-phase stability_observed OLD (n=51): p5=-231.18 p10=7.92
  p25=195.54 p50=431.64 p75=586.95 p90=631.21 p95=706.70. NEW (n=51):
  p5=-231.39 p10=7.90 p25=213.09 p50=428.39 p75=587.55 p90=631.50
  p95=709.03. p10 of worst-phase CS_ratio_f moves the most of any
  percentile (-0.0162->-0.1639) while p5 moves the opposite way
  (-0.2526->-0.2340) -- non-monotonic across percentile levels, expected
  at n=51 (same N=51 resolution argument already documented for the CS-
  threshold re-confirmation after the Fy yaw-term work: individual
  corner instances re-ordering near a rank boundary can move one
  percentile without a real distribution-wide shift). No threshold
  value changed by this entry -- re-derivation is the standing separate
  stop, same rule as every prior estimator-input change.

### CS/stability thresholds re-confirmed after steering-ratio L4 upgrade [2026-07-26]
- The re-derivation stop closed: all five classification thresholds
  checked against the WP-B old-vs-new distribution, none changed.
- STRONG_CSF/MODERATE_CSF: +2/51 exceedance at both boundaries (0.10:
  6->8; 0.25: 16->18) -- within the standing N=51 resolution argument
  (a couple of instances re-ordering near a rank/count boundary is not
  a distribution-wide shift at this sample size). SCALE-VS-INPUT-
  ACCURACY ARGUMENT for keeping the values unchanged rather than
  re-deriving fresh ones: this upgrade improves an INPUT's accuracy
  (steering ratio, Level 1->4) inside an otherwise-unchanged estimator,
  it does not rescale the estimator's own output distribution the way
  the B1 estimator rebuild did (that moved the stability threshold from
  -500 to -50 because the ESTIMATOR changed). A threshold surviving a
  better-measured input with only marginal exceedance movement means
  those +2 flagged instances are the improved input surfacing real
  signal that a slightly-wrong constant was previously masking, not
  drift the threshold needs to chase.
- STRONG_CSR/MODERATE_CSR: rear byte-identical under the steering
  upgrade (verified directly, see the WP-B entry's byte-identity
  proof) -- no re-check needed, there is nothing for the rear-axle
  threshold to have moved against.
- stab_neg_thresh_Nm_per_deg: exceedances identical at all three
  levels checked (< 0: 5/5, < -50: 4/4, < -100: 3/3). The id-8 cluster
  (all 4 laps still occupy the 4 most-negative worst-phase-stability
  slots) and the gap the threshold sits in (-99.3 to -18.8, essentially
  unchanged from the original -99.2/-18.5 B1 derivation) are both
  confirmed intact under the new steering-ratio resolution.
- ONE CEILING-TIED INSTANCE MIGRATION, found while explaining the
  n=49->48 worst-phase-CS_ratio_f count from the WP-B diagnostic:
  lap 3/corner 1 (stable_id 1, medium speed) moved from a valid worst
  value (0.9937, at exit_4) under the old constant to NaN under the
  new table -- not a data loss. exit_4's median CS_ratio_f rounds up
  to exactly 1.0 under the new delta_f, so all 5 phases now tie at the
  ceiling and the worst-phase selector's strict "<" comparison (mirrors
  _classify_corner exactly: starts from a ceiling sentinel, only
  updates on a strictly-lower value) can no longer identify any phase
  as "the worst" -- the same sentinel-artifact mechanism as the two
  pre-existing all-ceiling instances (lap 1/corner 7, lap 2/corner 10,
  unaffected by this upgrade, NaN in both old and new). NaN here means
  "every phase tied at the ceiling", not "missing data" -- see the new
  PLAN.md open-thread note on this representation gap.

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

### Reference-grid choice reviewed against the chair's fixed-rate resampling [2026-07-26]
- `ecu_speed`'s time base as the common resampling grid (`prepare_
  vehicle_state`) was a pragmatic early-development choice with no
  documented reasoning; reviewed against the chair's fixed 100 Hz
  resampling grid and deliberately retained, since `ecu_speed` is
  required and measured near-uniform on Dubai (median dt 20.0000 ms,
  p95 jitter 0.0000 ms, ~50.000 Hz over 40800 samples), making it
  functionally equivalent to a fixed grid without re-triggering
  threshold re-derivation project-wide.
- The measured 50 Hz differs from the chair's 100 Hz default; this is
  immaterial -- upsampling native 50 Hz data adds no information, and
  every downstream filter parameter derives from the measured rate,
  not a hardcoded one -- the chair-exact yaw-accel window formula
  yields a 9-sample (~0.18 s) window at this rate, identical to what
  the chair code would produce on the same data.

### Observer/Kalman-filter beta estimation reviewed and not adopted [2026-07-26]
- No observer/Kalman estimator was considered at implementation time
  (early AI-assisted choice); reviewed 2026-07-26 and deliberately
  retained: the kinematic route is the minimal-assumption method using
  only directly-logged IMU channels, consistent with the project's
  measurement-side no-vehicle-model principle -- an observer would
  require exactly the tyre/vehicle model parameters this project
  deliberately does not claim. Observer-based estimation is future
  work conditional on a validated vehicle model.

### WP2b-2: decision-matrix rule engineering [2026-07-26]
- The 7 WP2 seed rules (ARB-only, provisional parameter labels) were
  retired in place (status field, never deleted -- kept for history)
  and superseded by 26 rules elicited from an external engineer
  decision matrix (scenario x speed-class grid, maintained outside
  the repo, supplied as an authoritative input rather than derived
  here). Every matrix cell -- including the two the matrix defines no
  action for, and the escalation cells not yet automated -- got
  exactly one traceable rule entry (`cell_id` field, document leads
  code), verified by a synthetic round-trip check against all 31
  scenario x speed-class cells plus the one dropped combined-entry
  cell.
- SEVERITY-CLASSIFICATION NUANCE (caught while writing synthetic test
  data, not previously documented): `_classify_corner`'s severity
  logic does NOT reach "moderate" from moderate-band CS collapse
  alone. Re-reading the actual branch order: `"moderate"` requires
  EITHER a strong CS collapse alone (front or rear, independent of
  yaw stability), OR a moderate CS collapse combined with a
  destabilising yaw moment, OR a destabilising yaw moment alone --
  moderate CS collapse with a stable yaw moment stays "normal" and
  never fires any severity-gated rule. This is existing, unchanged
  classifier behaviour (not touched by WP2b-2), but it was easy to
  misread from the threshold names alone; worth stating explicitly
  since every matrix rule's `min_severity: "moderate"` gate depends on
  it.
- ESCALATION-ORDER TIER is a new, deliberately separate axis from
  `change_effort` in the registry: change_effort measures literal
  time-to-change (seconds/minutes/garage_hours), while the matrix's
  cockpit -> pitlane -> garage escalation order measures who decides
  and how disruptive the change is -- they disagree for real
  parameters (`diff_position` is change_effort "seconds" but matrix-
  "garage", a driver-preference domain change; `camber_*` is
  change_effort "minutes" but also matrix-"garage"). Collapsing them
  into one field would have silently misranked recommendations by the
  wrong notion of "cheap".
- ACTION-CLASS SPLIT (advisory vs. recommended, WP2b-2 amendment 7):
  a "data"-trigger match at moderate severity with no corroborating
  driver feedback on the same phases renders as an ADVISORY
  observation (non-imperative rationale, never budget-eligible)
  rather than a RECOMMENDED, budget-eligible suggestion. Grounding:
  mild understeer is this car's deliberate stable baseline (June
  driver-report precedent, predating this session) and the matrix
  elicitation's own bias is against unnecessary changes when the
  driver is inconsistent with the data -- an uncorroborated moderate
  verdict is diagnosis, not mandate. The class boundary (which
  severity/trigger combinations are action-eligible) is a config
  value (`recommendations.json settings.action_class`), not a
  hardcoded threshold, consistent with the Tier B config-driven
  principle. This is a product/process design choice (Tier C).
- CONSISTENCY GATE (Tier B, config-documented): no recommendation
  fires unless its triggering verdict repeats on both an absolute
  floor (`min_repeat_laps`) AND a fraction (`min_repeat_fraction`) of
  that corner's analysed laps, evaluated per-lap rather than only on
  the existing median-of-medians aggregate. Standard exclusion-mask
  practice, not a novel method; parameters are config values, not
  named constants, since they tune to outing length rather than
  defining what the aggregation method is.
- FEASIBILITY LIMITATION (WP2b-2 amendment 6, worth stating plainly
  for the limitations register): checking a recommended change against
  the outing's current setup sheet requires knowing the current value,
  but numeric setup-sheet fields default to 0 in the UI until filled,
  and several registry parameters (damper clicks, min=0) treat 0 as a
  legitimate real setting. There is no way to distinguish a genuine
  0 from an unfilled default without an explicit "was this field ever
  touched" flag, which does not exist. The engine accepts this
  ambiguity rather than guessing: any stored 0 is treated as "current
  value unknown" uniformly, for every parameter, and the affected
  recommendation renders "limit not checked" rather than silently
  asserting feasibility. A rare genuine 0 setting on a zero-legitimate
  parameter is the one case this reads as unknown when it could have
  been checked -- accepted, not fixed, since fixing it needs new data
  the setup sheet doesn't currently capture.

### WP1 consolidation, Turn 1: canonical corner realization [2026-07-26]
- Replaces per-lap-only corner/phase realization with a post-pass over
  the existing (unchanged) detection + connected-components clustering:
  one canonical bracket + set of phase boundaries per stable_corner_id
  (median per boundary across cluster members, s_m-anchored, reset-
  guarded), re-realized onto every valid lap by inverting that lap's own
  guarded s_m(t) -- including laps that detected no bracket there at all
  (tagged `canonical_quiet`, real telemetry, informative not an error).
  Closes the bracket-edge reset-guard gap explicitly left open in the
  WP1-freeze-proof entry above (`bracket_start_m`/`end_m` now use the
  same `_interp_lap_distance_guarded` helper apex distance already did).
- TWO-PASS SPLIT: pass 1 (seeded split, unchanged) still assigns
  straggler brackets to their best-overlap SEED LAP. Pass 2 (new)
  re-checks only pass-1's `straddles_adjacent_corners`-tagged brackets
  against each candidate cluster's CONFIDENT-member canonical window
  instead -- a once-per-corner-pair decision instead of a per-lap race.
  On Dubai, pass 2 found 0 reassignments: the confident-member windows
  it compared against happened to agree with pass 1's own seed-based
  assignment for every straddling bracket in this file. The stability
  improvement documented below therefore came from the canonical-bracket-
  median step (item 1), not from pass 2 changing membership -- both
  mechanisms ran, only one changed the outcome on this data; a second
  track could exercise pass 2 differently.
- REAL BUG CAUGHT AND FIXED during implementation, worth recording as a
  methods note: an early version sliced each lap's speed/lat_g channel
  down to the bracket window BEFORE smoothing, instead of smoothing the
  full lap first (as `_build_corner` already did) and slicing after.
  Convolution's "same"-mode edge behaviour on a short array attenuates
  values toward the boundary (kernel taps with nothing to multiply
  against still count toward the fixed-length normaliser), which biased
  every corner's re-realized apex_speed low and, since severity/
  speed_class both key off it, would have silently corrupted every
  downstream verdict. Caught by comparing before/after apex speeds on
  real Dubai data (C3/C4 read "high" ~160-185 km/h before, implausibly
  "medium" ~88-98 km/h after) rather than trusting the diff looked
  reasonable -- the fix hoists the full-lap smooth outside the per-
  corner loop, both correcting and de-duplicating the computation.
- RESULT on the two corners this session's diagnostics flagged as
  unstable (C10/C11, bracket-width bimodality -- a lap-2 short/high-
  speed bracket vs. a lap-1 wide/low-speed compound bracket for the same
  cluster): both now realize consistently across all 4 valid laps at
  ~80-91 km/h, classified "medium" uniformly -- markedly tighter than
  the pre-canonical 75.8-151.7 km/h (C10) / 76.6-162.9 km/h (C11)
  swings. This DIFFERS from the proposal's own prediction (that the
  low-speed ~76-79 km/h compound realization would likely win a vote
  across instances) -- the canonical bracket is a median over ALL final
  members' geometry, not a vote over classified instances, and a median
  window between a wide-slow and a narrow-fast bracket produces a
  moderate reading distinct from either extreme. Recorded as the actual
  outcome, not retrofitted to match the prediction.
- Per-lap corner counts are now equal across every stable corner id
  (14 stable corners x 4 valid laps = 56 instances, 5 of them
  `canonical_quiet` -- previously-missing laps that now materialize with
  real, quiet telemetry, e.g. C9's lap 1). C10's canonical bracket
  (175m) no longer clears the compound_corner threshold (300m) that its
  widest per-lap realization used to; C11's (380m) still does.
- Modules 1-5 sample-level outputs are unaffected by construction (they
  read `channels`/`params`/`laps` only, never `corners`/segments) --
  verified numerically identical pre/post this turn on Dubai (state,
  beta, CS_ratio_f, stability_observed_Nm_per_deg all unchanged).
  `test_stability.py` clean.
- STANDING GATE, not yet done: per CLAUDE.md's grounding rule, this is
  an estimator-INPUT change (the phase-window distribution feeding
  classify_fn's thresholds has moved) even though estimator code hasn't.
  WP1 Turn 2 (validation + re-derivation inputs) is the next step, not
  yet started this turn -- current classify_fn verdicts on canonical
  data should be read with that pending, not as newly-validated.

### WP1 consolidation, Turn 2: validation findings that motivated Turn 3 [2026-07-26]
- Read-only diagnostics (no thresholds/code changed) traced the C9/C10
  pairing to its root: pass 1's own union-find puts both in ONE
  connected component (5 brackets across laps [1,2,2,3,4] -- lap 2
  uniquely contributes two separate brackets, which is what triggers
  the seeded split at all). C9<->C10 canonical windows overlapped 88%
  of the smaller window; 99.4% of C10's samples were also inside C9's
  window, every lap. `corner_radius_filtered` overlap (an independent
  channel) regressed for C10 specifically (0.67 pre-WP1 baseline ->
  0.55) -- a third, independent method converging on the same pair.
  True local apex positions (peak |lat_g|, independent of the canonical
  median) were nonetheless 58m apart with different speed signatures
  (C9 ~76 km/h, C10 ~87-90 km/h) -- evidence pointed both ways, handed
  to the reviewer rather than resolved here.
- Inter-lap stability agreement (the "pooled grid" mechanism's own
  predicted effect) tightened dramatically for 13 of 14 corners after
  Turn 1's canonical phasing, several by 10-100x (C8: std 124.4->0.6;
  C10: 192.0->0.8; C11: 209.5->2.2) -- strong independent confirmation
  the canonical-phase mechanism works as designed, C9/C10/C11/C12
  aside.

### WP1 consolidation, Turn 3: canonical boundary resolution [2026-07-26, reviewer decision: partition not merge]
- New post-pass (`_resolve_canonical_overlaps`, `modules/corner_
  analysis.py`): any pair of canonical windows overlapping more than
  `canonical_overlap_max` (config, 0.10) is truncated to a shared
  boundary placed at the |ay| minimum of the pooled (cross-lap median,
  s-anchored) lateral-g profile between the two apex positions --
  Tier B geometric post-pass, config-documented (`canonical_overlap_
  max`, `canonical_boundary_grid_step_m`, both new config/channels.json
  corner_detection keys). This is the same "split a compound at an ay
  minimum" idea already named as an open, undone refinement in this
  file's original compound-corner finding (2026-07-22) -- implemented
  here for canonical WINDOW resolution, not per-lap phase display, six
  months after being first named.
- Phase boundaries (brake/turnin/half -- apex is never touched, the
  search region is defined between the two apex positions by
  construction) are re-clamped into each truncated window afterward. A
  boundary whose defining event no longer sits inside its sub-window
  collapses to a degenerate (zero-length) phase there --
  `summarise_corners`/`_classify_corner` already read that as "no
  signal" (empty slice -> n_samples=0 -> NaN stats -> skipped by the
  worst-value search); NEITHER function needed a code change for this
  to be handled honestly, confirming the existing degenerate-phase
  convention (already used by `apex_3` and by the throttle-missing
  brake-phase fallback) generalises cleanly to a case it wasn't
  originally written for.
- RESULT on Dubai: C9<->C10 and C11<->C12 both resolve to exactly 0.0%
  overlap and 0.0% sample-sharing (was 88%/99.4% for C9/C10). C11's
  canonical realization changes qualitatively, not just numerically:
  its apex speed jumps from ~81-85 km/h ("medium") to 152-162 km/h
  ("high") once no longer contaminated by neighbouring C12's slow
  apex in its own min-speed search -- C11 was genuinely mis-classified
  before this turn, not just noisily classified. `corner_radius_
  filtered` overlap for C10 improves from 0.55 to 0.60 (mean 0.78->
  0.84) -- better, but still short of the original pre-WP1 0.67
  baseline; C11 reaches a perfect 1.000 on every lap (was 1.000
  already, unaffected before, now confirmed unaffected after too since
  its OWN window only lost contaminated territory, not real signal).
- NOT uniformly positive, reported plainly: C9's inter-lap stability
  agreement WORSENED slightly post-partition (std 6.7 -> 12.0 Nm/deg --
  still small in absolute terms next to corners like C10's pre-
  canonical 192.0, but a real, honest non-improvement, not spun as
  one). C9-L1's CS_r=-0.721 flag (Turn 2) persists unchanged post-
  partition -- C9's start boundary wasn't touched by this turn, so this
  finding is independent of the C9/C10 question and remains open.
- The six-most-negative-stability / C8-cluster finding is now IDENTICAL
  to the original pre-WP1 (pre-canonical) result: C8 (4 of 6, ~-398
  Nm/deg) + C3 (2 of 6, now positive, +24.7/+27.1) -- the single most
  load-bearing distribution fact in this diagnostic (which corner
  triggers the strong destabilising-yaw threshold) is unchanged across
  all three states measured this session (before WP1, after Turn 1,
  after Turn 3) -- the most reassuring stability result of the whole
  consolidation, precisely because nothing about the canonical-
  realization work should have touched C8 or C3 at all.
- Re-derivation input handed to the user for WP1 Turn 4 (thresholds are
  the user's own call, not derived here): worst-phase CS_f/CS_r/
  stability percentiles, exceedance counts at the current thresholds,
  and the inter-lap-agreement table, all over the final post-partition
  56-instance set.

### WP1 arc closeout [2026-07-27]
- Full arc, four turns, one method: per-lap-independent corner/phase
  detection (unchanged throughout, verified by source hash every
  turn) now feeds a canonical realization layer -- one bracket + one
  set of phase boundaries per stable_corner_id (median per boundary
  across laps, reset-guarded), re-realized onto every valid lap
  (`canonical_quiet` for a lap that reached the window without
  independently detecting it, absent only for a lap that never
  reached it at all), with a boundary-partition post-pass (Turn 3) for
  any pair of canonical windows still overlapping more than 10% after
  that -- resolved at the pooled |ay| minimum between the two apex
  positions, non-overlapping by construction, reviewer's call
  (partition, not merge) executed exactly as decided.
- A real implementation bug was caught and fixed before any of this
  was trustworthy (Turn 1): smoothing a short bracket slice instead of
  the full lap first biased every apex speed low via convolution edge
  effects. Caught by comparing before/after numbers against physical
  plausibility, not by assuming a diff looked reasonable -- the
  methodological point (verify against ground truth, not internal
  consistency) is worth keeping for the thesis write-up regardless of
  the specific bug.
- C11 was genuinely MIS-classified before this arc (medium, ~81-85
  km/h) and is genuinely a high-speed corner (152-162 km/h) once its
  canonical window stopped drawing samples from neighbouring C10's
  slow apex -- not a borderline call resolved either way, a real
  correction with a traced mechanism.
- Inter-lap stability agreement collapsed 10-100x for the large
  majority of corners after canonical phasing (e.g. C8 std 124.4->0.6,
  C10 192.0->0.8, C11 209.5->2.2) -- the strongest, cleanest
  confirmation that the "pooled grid makes stability a per-corner
  property" mechanism (2026-07-24 finding, unchanged) behaves exactly
  as that finding predicted once the phase-jitter source it named is
  actually closed.
- TRI-STATE INVARIANCE, the single most reassuring cross-check this
  session: the six-most-negative-stability distribution's corner
  attribution -- C8 (4 of 6) + C3 (2 of 6) -- is IDENTICAL across all
  three states measured (pre-WP1 baseline, post-Turn-1 canonical
  realization, post-Turn-3 boundary partition). Nothing about this
  arc's work should have touched C8 or C3, and by this measure,
  nothing did. A large, multi-turn structural change to how corners
  and phases are realized left the project's most safety-relevant
  classification signal (which corner triggers the destabilising-yaw
  threshold) completely undisturbed -- exactly what "the canonical
  layer only changes realization, never detection or classification
  logic" should look like when checked, not just claimed.

### Threshold re-confirmation after WP1 consolidation [2026-07-27]
- All five classification thresholds (STRONG_CSF/STRONG_CSR/
  MODERATE_CSF/MODERATE_CSR/stab_neg_thresh_Nm_per_deg,
  config/parameters.json) re-confirmed UNCHANGED -- values kept, only
  each `derived_from` string gained a dated append recording the
  argument, per this project's standing convention of never silently
  re-deriving without a paper trail.
- The argument: stability's six-most-negative distribution is now
  -398/+25 (C8-only negative population, tri-state-invariant per the
  closeout entry above) with exceedance counts 4/4/4 unchanged across
  the pre-WP1, post-Turn-1, and post-Turn-3 states alike (stab<-50,
  stab<-100 both hold at their historical counts). The CS-side moderate
  counts (11 CS-branch flags now, see the verdict-distribution
  re-check below) shift, but traceably to REPAIRED realization defects
  (C11's speed misclassification, window-boundary jitter for the
  C9/C10/C11/C12 region) rather than to any change in the underlying
  physical signal or estimator -- a realization-ACCURACY upgrade, not
  drift in what's being measured. Keeping the values means the
  improved realization's marginal flag movements read as signal
  (corners now measured correctly that weren't before), not as
  threshold decay requiring a re-derivation. Same reasoning already
  applied twice before in this file (Fy yaw-term upgrade, steering-
  ratio L4 upgrade) -- this is the third confirmation under the same
  standing argument, now against the largest single change (canonical
  realization) any of the three has been checked against.

### Verdict-distribution re-check after WP1 consolidation [2026-07-27]
- `diagnostics/inspect_b3_verdict_distribution.py` (updated to report
  its own instance count dynamically rather than a stale hardcoded 51,
  since realization now legitimately changes it) re-run over the final
  56-instance canonical set: 0 strong / 15 moderate / 41 normal, 26.8%
  flagged. Branch split: CS=11, stability-only=4, both=0.
- Against the historical pre-Fy-correction baseline (1 strong / 16
  moderate / 34 normal, n=51, ~33% flagged) and the post-Fy-correction
  point (0/14/37, n=51, ~27.5% flagged): 26.8% continues the SAME
  downward trend both prior corrections already established, not a
  reversal or a new regime -- three independent changes (Fy yaw term,
  steering-ratio L4, WP1 canonical realization), one consistent
  direction.
- Against the June driver report ("balanced, mild understeer, no
  instability"): 0 strong (matches "no instability" as the most severe
  category) and a CS-branch-dominated moderate population (11 of 15,
  understeer/oversteer flags) rather than stability-branch (4 of 15) --
  consistent with "mild understeer" being the dominant real pattern,
  not yaw instability. Same validation argument already made after the
  Fy correction (2026-07-24): each independent upgrade moves the tool
  further toward the driver's own characterisation, not away from it.
- Two of the moderate flags are the canonical_quiet instances this
  session's diagnostics already tracked in detail (C9-lap1, worst_CSr
  =-0.721; C12-lap4, worst_CSf=0.046) -- both correctly flagged here as
  real, moderate-severity CS-branch instances, not silently dropped or
  inflated by being quiet realizations. C8 remains 4/4 laps, stability-
  only, exactly as every other check this session found it.

### WP1 open watch items, carried forward [2026-07-27]
- `corner_radius_filtered` overlap for C10: 0.60 (post-partition) vs.
  the original pre-WP1 baseline of 0.67 -- improved from Turn 2's 0.55
  but not fully recovered. Not re-investigated further this session;
  a candidate for another look if C10 is ever load-bearing for a
  recommendation-engine rule.
- C9's inter-lap stability agreement (std 6.7 -> 12.0 Nm/deg
  post-partition) and the C9-lap1 (canonical_quiet) CS_r=-0.721 flag,
  unchanged since first noticed in Turn 2 and confirmed to persist in
  Turn 3 and again in this closeout's verdict-distribution re-check.
  Both trace to the SAME unexamined variable: C9's start boundary was
  never touched by the Turn 3 partition (only its end moved) and was
  never independently re-examined on its own terms the way the C9/C10
  shared boundary was. Flagged as the one genuinely open thread of
  this arc, not resolved here -- a candidate for a future dedicated
  look, not a known defect with a known fix.

### CS credibility diagnostics: kerb audit + filter sensitivity [2026-08-17]
- WP-A item 1 (kerb audit, diagnostics/inspect_cs_kerb_window_audit.py):
  checked whether Module 4b's extreme negative worst-phase CS ratios are
  kerb artifacts. Reconstructed the actual regression window for the 10
  most negative worst-phase CS_ratio_f and CS_ratio_r instances on Dubai
  (56 corners), verifying window-reconstruction parity against
  production's own C_window_f/r (21/21 matched to 1e-6 relative
  tolerance). Result: extreme negatives on both axles are predominantly
  kerb-clean -- the single most negative instance on each axle (front:
  C4 lap 3, -0.552; rear: C9 lap 1, -0.721) sits at 0% kerb fraction
  inside its regression window, 0.6-3.1 s from the nearest kerb flag.
  Where kerb-coincidence does occur (2/10 front, 3/10 rear instances),
  it clusters at the MILD end of each ranked list (CS_ratio closer to
  0, window kerb fraction 9-60%), not at the extremes. Kerb
  contamination of Module 4b's inputs is real and structurally possible
  (the fitting window only gates on the endpoint sample's moving/kerb
  mask, not every sample inside the window body, and the upstream
  Butterworth filtering is zero-phase/acausal, so a kerb transient can
  leak beyond the dilated kerb_mask band) -- but it is a secondary
  effect, not the mechanism producing the extreme negative tail.
- WP-A item 2 (filter sensitivity, diagnostics/
  inspect_cs_filter_sensitivity.py): swept cs_filter_cutoff_hz over
  {2.0, 1.5, 1.0, 0.7} Hz in an in-memory copy of the params dict only
  -- config/parameters.json untouched throughout. Front and rear
  negative tails behave oppositely. FRONT is filter-dependent: p5 of
  the worst-phase distribution crosses from negative to positive
  between 1.5 Hz and 1.0 Hz (-0.007 -> 0.061), and the single worst
  front instance (C4 lap 3, -0.552 at 2 Hz) is negative ONLY at 2 Hz,
  flipping to +0.52/+0.82/+0.57 at 1.5/1.0/0.7 Hz. REAR is filter-
  robust in SIGN and confined almost entirely to one physical corner,
  C9: lap 1 and lap 2 stay negative at all four cutoffs (lap 1:
  -0.721/-1.070/-2.379/-2.437; lap 2: -0.361/-0.445/-2.315/-2.650),
  and lap 4 joins the negative set only at the lowest cutoff tested,
  0.7 Hz (0.326/0.298/0.183/-0.833 across the same four cutoffs). In
  all three C9 rear instances, magnitude GROWS monotonically as cutoff
  drops -- the opposite of the front tail's behaviour.
- MECHANISM BEHIND THE MAGNITUDE GROWTH: UNRESOLVED, stated explicitly
  so it is not silently assumed either way. Heavier low-pass filtering
  attenuates transients, so a real, correctly-measured throwaway/
  beyond-peak event getting LARGER as cutoff drops is not what a
  shrinking-artifact-under-filtering picture would predict -- but it is
  also not proof of a larger real physical event. CS_ratio is a ratio
  of two filtered-signal-derived quantities (windowed-OLS slope C_alpha
  over the linear-reference slope C_linear_ref), and numerator and
  denominator are not guaranteed to respond identically to a filter-
  cutoff change: a denominator shrinking faster than the numerator
  under heavier filtering would produce exactly this monotonic-growth
  signature with no change in the underlying physical event. Not
  checked either way yet. Open question, to be examined visually once
  the tyre-curve scatter plot (WP-A item 3, not yet implemented) can
  show the actual Fy-vs-alpha point cloud at each cutoff instead of the
  single collapsed ratio number.
- Overlap with the WP1 watch items above: the C9-lap1 CS_r=-0.721 flag
  investigated here is the SAME flag first noted in Turn 2, confirmed
  to persist through the Turn 3 partition and the B3 verdict-
  distribution re-check, and already carried forward as an open watch
  item ("C9's inter-lap stability agreement... C9's start boundary was
  never touched by the Turn 3 partition (only its end moved)", above).
  This diagnostic pass adds evidence but does not close that thread:
  kerb-clean and filter-robust-in-sign rules out two candidate artifact
  explanations, but does not distinguish genuine beyond-peak tyre
  physics at C9 from an unexamined realization defect at C9's own start
  boundary -- exit_4, the phase carrying this flag, abuts the Turn 3
  partition's moved end boundary. Both remain open until C9's start
  boundary gets the dedicated look the watch item above already calls
  for.
- Production cs_filter_cutoff_hz stays at 2 Hz (chair-identical,
  config/parameters.json), now on record as a deliberate, evidence-
  backed choice rather than an unexamined default: this pass found no
  cutoff-independent case for lowering it (the front tail needs it
  lowered below 1.5 Hz to read negative at all, and even then flips
  sign per corner; the rear C9 finding is unaffected by cutoff choice
  either way, since it already persists at the production value) and
  it is not being changed here.

### C9 negative-CS decomposition + zero-slip offset finding [2026-08-17]
- WINDOW DECOMPOSITION (diagnostics/inspect_c9_negative_cs.py): the two
  candidate artifact explanations for C9's rear negative-CS windows
  (laps 1, 2, 4, all three found negative at the production 2 Hz
  cutoff) are both refuted by direct inspection. Sign-inconsistency:
  0.0% of samples show sign(alpha_r) != sign(Fy_r) in any of the three
  windows -- alpha_r and Fy_r stay same-sign (both negative) throughout
  every window, a physically coherent single-tyre-curve relation, not
  noisy or contradictory data. Boundary contamination: every window
  ends 71-85 m before C9's own canonical bracket end (out of a ~172 m
  bracket), nowhere near the C9/C10 partition boundary the WP1 watch
  item above flags as unexamined. The windows ARE clean; the negative
  ratio comes from the window's own regression slope going negative
  (Fy_r's magnitude not keeping pace with, or reversing against,
  growing |alpha_r|) -- the "beyond-peak, cloud folds back" signature
  the tyre-curve plot's own docstring names, not a data-quality defect.
  Bonus finding while reconciling this against the tyre-curve plot's
  visual "alpha sweeps through zero while Fy stays negative": that
  zero-crossing is real, but happens 67-92 samples (~1.3-1.8 s) AFTER
  each window's extreme sample, near exit_4's own boundary -- a later,
  distinct part of the same corner pass, not inside the window that
  produces the negative CS_ratio extreme.
- ZERO-SLIP Fy OFFSET, GLOBAL AND DIRECTION-CORRELATED: median Fy_f/
  Fy_r over samples with |slip angle| < 0.2 deg, computed per stable
  corner over each corner's own canonical window, is nonzero at every
  corner with enough near-zero-slip samples to measure (typically
  several thousand N) -- C9 is NOT an outlier (rank 7 of 13 front,
  8 of 14 rear by |median|; several corners, e.g. C1/C3/C13/C14, show
  comparable or larger offsets). Extended the same script with a
  turn-direction correlation check: direction taken as sign(median
  ay_mps2) over each corner's whole canonical window (ay, not yaw rate
  -- this codebase's own established cornering-direction signal, the
  dual-criterion corner detector's |ay| > 0.6g entry gate). Result:
  the offset sign matches the turn-direction sign at EVERY corner with
  a valid median -- 13/13 front, 14/14 rear, no exceptions.
- IMPLICATION: a bias that flips sign in lockstep with turn direction,
  at the instant the kinematic slip-angle estimate reads zero, is not
  random noise or a fixed sensor tare -- it means the KINEMATIC
  (Level 1) sideslip estimate beta itself carries a direction-
  dependent error large enough that "zero slip angle" by that estimate
  does not correspond to a real zero-lateral-force physical state.
  Since Module 4b's linear-reference gate (C_linear_ref, the CS_ratio
  denominator) is defined by exactly this near-zero-slip region, any
  systematic error there propagates into every CS_ratio value computed
  from it. Practical consequence: a CS_ratio < 0 cannot currently be
  safely read as "genuine beyond-peak tyre saturation" until this
  sideslip-estimate error is characterised or corrected -- the sign
  could in principle be doing some of that work by coincidence, but
  that is not the same as the estimator being trustworthy at the
  precision a beyond-peak claim would need.
- WP-A item 5 (the beyond-peak verdict tier proposed at the start of
  the CS credibility bundle) is SHELVED, not abandoned or redesigned
  around this finding. Reopen condition: a validated sideslip estimate
  (beta) that has been checked against this direction-dependent-offset
  failure mode and shown not to carry it, or shown to carry a
  characterised, correctable version of it. Implementing a beyond-peak
  verdict on top of the current Level 1 kinematic beta would be
  building a classification tier on a denominator this finding shows
  is not yet trustworthy at the required precision.
- EXPLICIT LINK: this is the empirical motivation for the sideslip
  methods-comparison framework, Open Board item B (supervisor-
  mandated) -- not a coincidental side-finding but the concrete,
  data-grounded case for why that comparison needs to happen before
  any beyond-peak/saturation claim can be built on top of beta.

### Corner-numbering display bug: wrong field, not wrong indexing [2026-07-26]
- Reported as "C14 shows 13" (and similar off-by-some mismatches for
  other corners): clicking stable-corner grid cell C14 opened a details
  panel reading "Corner 13". Root cause was NOT zero-vs-one-based
  indexing, as first hypothesised -- it was two legitimately different
  fields on the same summary dict being conflated at one display site:
  `stable_corner_id` (the cross-lap cluster identity every other
  surface -- grid cells, corner-map markers, recommendation-engine
  corner chips -- already keys off) vs. `corner_number` (the raw, per-
  lap sequential detection index, which legitimately drifts from the
  cluster id whenever a corner's bracket splits/merges differently lap
  to lap, e.g. C14's own raw corner_number was 13,13,13,12 across its
  four laps). `ui/views/outing_form.py`'s `_build_corner_details`
  header was the only production call site reading the wrong one
  (grepped `corner_number` project-wide to confirm -- every other hit
  is either module internals where it's the correct field, or a
  diagnostics/*.py script using it for its own stated purpose). One-
  line fix: read `stable_corner_id`, formatted identically to the grid
  (`C<n>`). Tier C (UI display only, no analysis-layer change).

### Matrix v2 review round [2026-07-26, project-lead review]
- Provenance grade added between engineer-verbatim and the capped
  grades: `project-lead-reviewed` -- action-eligible unless also marked
  `situational`. All 8 cells that were `project-default` (elicited-
  session defaults, no clean engineer statement) and 3 of the 6
  `mirror-derived` cells (OS-TIN-low/med/high) were confirmed by
  project-lead review and promoted; OS-APX-low/med/high remain
  mirror-derived (see situational note below).
- TWO DECODES: US-BRK-low and INST-BRK-low's original v1 answers were
  ABS-map literals (targets 2 and 4) built from what turned out to be a
  garbled engineer response, defaulted rather than confirmed. Decoded:
  US-BRK-low's real first change is front toe +0.5mm more toe-out; the
  ABS-map idea survives only as a held (never-firing) escalation,
  try-and-error 1-2 positions toward the front-axle-stable variant.
  INST-BRK-low's real first change is the same try-and-error pattern,
  rear-axle-stable variant, no literal target. Both cases: a literal,
  precise-looking value (position 2, position 4) had actually encoded
  low confidence, not high -- precision is not the same as confirmation,
  worth remembering when reading any elicited value that looks exact.
- ABS DIRECTION SEMANTICS, annotated not rewritten (config/
  setup_parameters.json abs_position notes): the "categorical, not one
  monotonic axis" finding (WP2b-1) stays true at the FULL 0-11 range
  (jumping grip-level brackets is still nonsensical) -- what changes is
  that WITHIN one bracket, an ordinal +1/+2 nudge IS a real, project-
  lead-confirmed stability lever (the two try-and-error rules above).
  Both statements coexist; the annotation says so explicitly rather
  than one silently overwriting the other.
- CAMBER STEP CORRECTED: 0.3deg (WP2b-2 elicited-default, always
  flagged as pending review) -> 0.1deg (project-lead review). Typical
  field practice is 0.2deg (two 0.1deg clicks); a tyre-surface/wear-
  pattern inspection precedes any camber change, and a one-sided
  (single-corner) change is sometimes preferred over the symmetric
  FL+FR pair the rules encode -- both now stated in the citing rules'
  rationale, since neither is capturable as a single numeric field.
- SITUATIONAL CLASS (engineering principle, not just a config flag):
  OS-APX-low/med/high are the three apex-oversteer cells where the
  matrix v2 review explicitly declines to name one first lever --
  front ARB +1 OR rear ARB -1 (low speed), diff +1 or clear via toe/
  camber/ARB (medium), springs or rear ARB -1 or camber (high). The
  AXLE-GRIP LOAD-SENSITIVITY PRINCIPLE this elicits: at the apex, which
  axle is actually closer to its own grip limit at that moment decides
  which lever helps, and that isn't determinable from the verdict alone
  (front vs rear CS ratio) the way it is at other phases -- the matrix's
  silence on a single answer here is itself elicited engineering
  knowledge, not a gap to be filled by picking one arbitrarily. Encoded
  as `situational: true` -- permanently advisory regardless of
  provenance/severity/corroboration, alternatives listed verbatim in the
  rationale, engineer decides. US-APX-low (the understeer-side, non-
  situational sibling) gains its own noted alternative (rear ARB +1)
  without losing action-eligibility on its primary lever -- the
  asymmetry (US-side has one clear answer, OS-side doesn't) is the
  interesting part, not an inconsistency.

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
- ~~[ADDED 2026-07-24] Verify page numbers for the estimate_sideslip
  citation (Mitschke/Wallentowitz, Dynamik der Kraftfahrzeuge,
  single-track lateral kinematics) -- currently cited as "p. TBD,
  verify" in the docstring pending access to the primary source.~~
  [RESOLVED 2026-08-19, WP-S4 citation cleanup] The Mitschke/
  Wallentowitz page was never verified and is REPLACED, not sourced:
  estimate_sideslip's anchor is now Rajamani, Vehicle Dynamics and
  Control, 2nd ed., Springer 2012, sec. 2.6 "Dynamic Model in Terms of
  Yaw Rate and Slip Angle" (p. 37) and sec. 2.3 "Bicycle Model" (p. 27)
  -- the same book already anchoring WP-S4's Kalman observer vehicle
  model, both sections user-confirmed visually. modules/stability_
  analysis.py's docstring now carries a pointer line only ("method
  anchor recorded in thesis_notes.md, WP-S4 entry"), per the citation-
  location rule change (thesis_notes.md "WP-S4: Kalman sideslip
  observer" entry, [2026-08-19] RULE CHANGE bullet); no author/title/
  page in code.

- [ADDED 2026-08-19, WP-N1] ~~Verify page numbers for citations added this
  session, none checked against a primary source yet: Rajamani Ch. 13.10
  (Dugoff tyre model, eqs. 13.72-13.76 -- modules/tyre_model.py, page
  TBD verify -- chapter/topic not previously anchored, unlike the two
  below); Rajamani Ch. 14 (Kalman filter treatment, page TBD verify --
  chapter itself already confirmed, WP-S4/PLAN.md ANCHORS, only the page
  is new); Kiencke & Nielsen "Automotive Control Systems" 2nd ed.,
  "Vehicle Body Side Slip Angle Observer" section (exact section number
  TBD verify -- the section's TOPIC is already confirmed, WP-S4/PLAN.md
  ANCHORS, only its numeric reference is new). Ulsoy, Peng, Cakmakci
  "Automotive Control Systems" is a SEPARATE, MORE SEVERE case, not just
  a page/number gap: no part of this source -- book, chapter, or
  section -- has been opened or confirmed by anyone this session or
  before. "Ch. 14" as recorded in the WP-N1 entry above is an unverified
  guess, not a checked citation, and must not be treated as one (or used
  in any written text) until the actual chapter is located and
  confirmed against the primary source.~~ [RESOLVED 2026-08-19, WP-N2
  implementation turn: Ulsoy, Peng, Cakmakci CONFIRMED by two
  independent readings of the primary text -- sec. 14.3 p. 263
  ("Nonlinear Vehicle Model" header, read directly), sec. 14.1 p. 258ff
  (read directly), and Eq. 14.8 confirmed as a term-by-term match for
  this project's EKF force/moment balances (two documented
  simplifications: no roll DOF, pure-lateral Dugoff vs. Ulsoy's
  combined-slip Magic Formula) -- see the corrected WP-N1 entry bullet
  above and the WP-N2 entry below for full detail. STILL OPEN: Rajamani
  Ch. 13.10 and Ch. 14 exact page numbers; Kiencke & Nielsen's exact
  section number -- the last of these likely PERMANENTLY open by normal
  means, since the source PDF's body text does not survive text
  extraction (broken font encoding, systematically substituted glyphs)
  and can only be checked from the printed copy.]

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