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

### Small-decisions sweep [2026-07-26]
- cs fallback reference constants deleted (small-decisions sweep):
  `cs_front_fallback_reference_n_per_rad`/`cs_rear_fallback_reference_
  n_per_rad` removed from config/parameters.json -- defined, commented,
  consumed nowhere, verified 2026-07-26. The no-linear-reference case
  has never occurred on real data; if it ever does, the corner reports
  invalid, more honest than silently filling from an unvalidated
  constant.
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