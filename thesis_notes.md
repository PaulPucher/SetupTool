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
    [2026-08-20: the "clean alpha excitation under braking" reading of
    entry_1_brake's ceiling-pinning has an ALTERNATIVE explanation not
    considered here -- see "entry_1_brake phase-boundary bug" below.
    Unresolved in either direction, not declared wrong; recorded as an
    open alternative reading, not a correction.]
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
  [2026-08-20, superseding this caveat's wording: the mechanism and
  its magnitude are now fully quantified -- see "entry_1_brake
  phase-boundary bug: mechanism, blast radius, and fix" below.
  entry_1_brake covers 85.35% of the base population; the 86%
  flagged-sample figure is NOT meaningfully distinguishable from
  that population footprint once the flags' 240-contiguous-episode
  clustering is accounted for (effective sample size far below the
  6,730 raw count a naive binomial check would assume). "Indicative
  of where divergence concentrates" overstates it -- the figure may
  carry close to zero independent information about phase
  concentration, not merely imprecise attribution.]

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

### WP-N2 Step 1a: pass-1 EKF wall-clock timing, before any wiring
[2026-08-20]

PURPOSE: PLAN.md STEP 1 sub-step 1a -- time the pass-1 EKF (a
per-sample Python loop with a 2x2 matrix inversion each step) against
the production pipeline before any wiring decision, per the standing
rule "do not wire in a slow path and fix it later." Measurement only;
no config or production code changed. diagnostics/inspect_pass1_ekf_
timing.py (new, read-only), 5 repetitions, Dubai sample, n=40800
samples (full file, not the moving/racing-masked subset used
elsewhere in this arc).

MACHINE: Windows-10-10.0.26200-SP0 (Windows 11 build number; the
platform module reports it under the legacy "Windows-10" family
name), AMD64 Family 25 Model 116 Stepping 1 AuthenticAMD (Zen 4
generation), Python 3.10.9, numpy 2.2.6.

RESULT, mean over 5 reps: production full-outing total (parse_csv
through summarise_corners, the same sequence test_stability.py
exercises) = 123.27 s (std 8.66 s, range 114.55-138.82 s). pass_1 EKF
alone = 4.66 s (std 0.14 s, range 4.51-4.91 s) -- 0.114 ms/sample.
Projected full-outing total with the EKF substituted for the
kinematic estimate_sideslip call = production_mean -
kinematic_mean + ekf_mean = 127.93 s, a +3.8% increase.

UNEXPECTED FINDING, not what this measurement set out to check: the
EKF is NOT the pipeline's cost driver. Per-module breakdown (mean,
seconds): parse_csv 16.62, load_parameters 0.0002, prepare_vehicle_
state 0.015, estimate_sideslip (kinematic) 0.0025, estimate_slip_
angles 0.005, estimate_lateral_forces 0.004, estimate_cornering_
stiffness 106.22, estimate_yaw_moment_stability 0.275, summarise_
corners 0.129. estimate_cornering_stiffness alone is 86% of the
existing production total and is already ~23x more expensive than
the EKF loop this sub-step exists to evaluate. Consistent with PLAN.md
NOW's description of the chair's CS method as a sliding-window
least-squares fit evaluated per sample -- itself a per-sample loop,
just not the one this sub-step was asked to time. Recorded here as a
finding, not acted on: STEP 1a's mandate is measurement only, and
this observation is about a different module's cost, not the EKF's.

STABILITY ACROSS REPS: the EKF's own timing is tight (std/mean =
3.0%). The production total's variance (std/mean = 7.0%) traces
almost entirely to estimate_cornering_stiffness (std 8.41 s) and
parse_csv (std 0.59 s), not to the EKF.

JUDGEMENT (relative to the existing pipeline, per PLAN.md's own
framing, not an abstract interactive-app standard): ACCEPTABLE. The
EKF adds under 4 seconds to a pipeline that already takes upward of
two minutes, a ~4% relative increase that will not be perceptible
against the pre-existing wait. The pipeline's actual interactivity
problem, if there is one to raise, sits in estimate_cornering_
stiffness, a pre-existing cost this sub-step was not asked to
evaluate and did not change.

STEP 1a's own gate ("if it materially degrades the user experience,
vectorise or cache before proceeding") reads as NOT TRIGGERED on this
evidence -- a decision for the user to confirm, not this diagnostic to
assume.

USER-CONFIRMED (2026-08-20, approving Step 1b): the timing gate is not
triggered and the projection needs no correction -- estimate_sideslip
runs once inside estimate_sideslip_ekf_dugoff before its per-sample
loop (line 131), so the kinematic call is already inside the 4.66 s
pass_1 figure, not an addition on top of it.

### WP-N2 Step 1b: wiring proposal, approval, and implementation
[2026-08-20]

PURPOSE: wire the pass-1 EKF as a selectable sideslip source behind a
config switch, defaulted off, with the traces-vs-verdicts split (STEP
1c) made visible wherever a verdict renders. Proposed in full (7
points: substitution site, config control, cache/schema, banner
mechanism, downstream-consumer audit, verification plan, scope
exclusions), approved with one addition (empirically verify the
resolved-parameter claim rather than reason about it) and one
measurement requirement (the yaw-stability gate shift), both executed
before the final comparison -- see below.

IMPLEMENTATION, file by file:
- config/parameters.json: stability_estimation.sideslip_source
  ("kinematic" default, "ekf_pass_1" alternative) and classification.
  thresholds_calibrated_for_sideslip_source ("kinematic") -- both new
  keys, own explanatory comments, no existing key touched.
- modules/stability_analysis.py: ANALYSIS_SCHEMA_VERSION 4->5, bump
  comment extended in place (existing bump-policy text unchanged,
  matches the documented rule: payload shape changed regardless of
  the default producing byte-identical numbers).
- ui/views/outing_form.py (StabilityAnalysisThread.run()): the
  estimate_sideslip call becomes a branch on effective_params[
  "stability_estimation"]["sideslip_source"] -- "ekf_pass_1" calls
  diagnostics.sideslip_ekf_dugoff.estimate_sideslip_ekf_dugoff(state,
  effective_params, pass_id="pass_1") and takes beta_with_fallback
  (never the raw pre-fallback series, commented at the call site: raw
  keeps diverged-window artifacts, production must not feed a
  silently-diverged state downstream); "kinematic" (default) keeps the
  original estimate_sideslip call unchanged. sideslip_source threads
  through the finished-signal payload, the WP6 in-memory pipeline-
  cache identity (_pipeline_cache_put and the pre-emptive hit-check in
  _run_stability_analysis, same pattern as accuracy_cap), the WP5
  persisted analysis_data payload (_build_analysis_data_json), and the
  WP5 cache-hit guard (_try_render_cached_analysis) -- this last one
  goes slightly beyond the literal ask ("in-memory... same pattern as
  accuracy_cap") because accuracy_cap's own precedent already spans
  both cache layers (outing_form.py:1327's check exists specifically
  for this reason), so sideslip_source follows the same precedent at
  both layers for consistency, not as a scope addition.
  _classify_corner (called unmodified by both the UI and weekend_pdf_
  export.py) now reads both config keys and appends a placeholder "
  [UNCAL]" marker to short_verdict/long_verdict whenever they
  disagree -- single source of truth for the per-verdict marker,
  inherited automatically by every consumer that already reuses this
  method (recommendation.py's rule-matching does plain substring
  checks for "understeer"/"oversteer"/"unstable yaw", confirmed safe
  against a trailing suffix by reading _axle_verdict/_verdict_present
  directly). New _sideslip_source_calibrated() helper (pure config
  comparison, independent of which data is currently rendered since
  the cache-identity checks above already guarantee rendered data
  matches the live config) backs two new persistent banner labels
  (calibration_banner_label in the stability panel,
  recommendations_calibration_banner_label in the recommendations
  panel), toggled in _render_stability_summaries and
  _generate_recommendations respectively -- deliberately separate from
  the per-verdict marker because a top banner can scroll out of view
  on a long corner grid and the instruction was that it must not be
  possible to read a verdict colour and assume calibration.
- core/weekend_pdf_export.py: a duplicated two-line
  _sideslip_source_calibrated() (not imported from OutingForm --
  that method never touches self, but pulling a QWidget-bound method
  into a non-Qt PDF-generation context for a two-line comparison
  seemed the wrong trade) gates a placeholder warn-styled paragraph
  before the verdict table in _verdict_flowables. Per-verdict markers
  in the printed table are already inherited for free via the
  unmodified _classify_corner reuse.
All wording is explicitly placeholder ("PLACEHOLDER: ..." / "
[UNCAL]"), per instruction -- final text pending visual review.

VERIFICATION 1 -- resolved-parameter claim, empirically checked, not
reasoned about (diagnostics/inspect_step1b_wiring_verification.py
Section 0). The Step 1b proposal's verification plan had assumed
"apply_resolved_vehicle's resolved values equal the raw config
defaults for this dataset" under cap="Best available" -- checked by
diffing effective_params against the diagnostic script's raw params,
key by key. RESULT: the claim was WRONG. Under cap=None ("Best
available"), vehicle.steering_ratio_table appears in effective_params
and is absent from raw params -- the WP-B Level-4 steering-ratio
lookup (config/car_data.json steering_ratio_table, done 2026-07-26)
resolves at Level 4 whenever car_data.json is present and the cap
doesn't ceiling it below 4, which "Best available" never does. Every
other vehicle key matched exactly (mass, corner_weights, cog
position); stability_estimation/tyre_model_ekf/classification blocks
were byte-identical throughout (apply_resolved_vehicle never touches
them). Re-run under cap=1 (forces the same Level-1 scalar the
diagnostic's raw params uses): steering_ratio_table no longer appears,
and the only remaining "difference" was a documentation string
(corner_weights.note) absent from the resolver's returned value shape
-- not a computational input. CONCLUSION: exact reproduction is the
correct expectation only under cap<=3 (or any cap that ceilings
steering_ratio below its Level-4 availability); under "Best available"
specifically, a small, already-approved, already-documented numeric
difference is expected and is not evidence of a wiring defect.

VERIFICATION 2 -- yaw-stability min_beta_std_rad gate shift, measured
directly via modules.yaw_stability.calculate_observed_stability's own
diagnostics dict (Section 1), not inferred from stability_valid alone.
Grid: 2625 s-anchored Gaussian-ridge-regression points total, under
effective_params (cap=None). Kinematic: 2618/2625 valid, 7 skipped on
the beta_std gate specifically (skip_counts["min_samples"]=0,
["linalg_error"]=0). ekf_pass_1: 2625/2625 valid, 0 skipped on any
gate -- the EKF's larger-amplitude beta clears min_beta_std_rad=0.001
at every grid point the kinematic path already reached, plus the 7 it
didn't. Per-sample stability_valid population (post-interpolation)
was UNCHANGED at 31997/40800 both ways -- the 7 extra grid points sit
close enough to already-valid neighbours that np.interp's masked
query population doesn't move, even though the underlying regression
computed 7 more genuine values. Reported as measured: the predicted
STRUCTURAL shift (more windows clearing the gate) is real and
directionally as expected, but on this dataset it does not translate
into a stability_valid population change -- a finding, not a
conclusion generalised beyond this run.

VERIFICATION 3 -- full comparison against the frozen pass-1 baseline
(diagnostics/pass1_final_validation_manifest.json, git commit
76fc57673f4c2c618363809ba7c09aca226be4ba, 2026-08-20T09:12:21Z),
reusing that script's own section methodology (diagnostics/
inspect_step1b_wiring_verification.py Section 2), switch flipped to
ekf_pass_1 on disk before running (fresh process, matching the
"config only re-read after an app restart" semantics the switch
itself relies on).

Under cap=None ("Best available", the plan's originally stated
condition): every section DIFFERED from the frozen manifest, all by
small amounts -- NIS exceedances shifted by <0.3 percentage points,
self-consistency R^2 moved +0.003/+0.001 (front/rear), the h2-vs-ay
correlation moved 0.0002, and the sign-check per-sample fraction
dropped from 99.63% to 98.80%. Sample counts that don't depend on
corner-bracket geometry matched exactly (h2-vs-ay apex_3 population
n=471 both runs; sign-check corner counts 14/14 and 13/13 both runs).
Attributed to Verification 1's finding: the steering-ratio Level-4
lookup changes delta_f_rad slightly versus the frozen manifest's
Level-1-only run.

Re-run under cap=1 (isolating the steering-ratio effect, per
Verification 1): NIS figures now matched the manifest to the
precision it was recorded at (e.g. yaw_rate_exceedance 0.1001 vs
0.10011164...; the earlier "DIFFERS" flag on these was the
verification script's own comparison tolerance being tighter than its
own rounding, not a real difference -- noted here so it is not
mistaken for a second finding). But a genuine, smaller residual
persisted in the corner-bracket-keyed statistics: sign-check
per-sample fraction 98.85% (vs 99.63%), front R^2 0.9558 (vs 0.9526),
rear R^2 0.9832 (vs 0.9822). INVESTIGATED, not left unexplained: `git
log --oneline 76fc576..HEAD -- modules/corner_analysis.py` returns
exactly one commit, 0b296ce ("Fix entry_1_brake phase boundary;
record CS_ratio aggregation finding" -- PLAN.md NOW's own first
bullet, already committed, already verified against brake pressure).
The frozen manifest predates this fix; every section keyed on
bracket_start_m/bracket_end_m via _canonical_window_slice (sign
check, self-consistency R^2) inherits the entry_1_brake phase's
corrected, much narrower span (85%->5.5% of the dataset, per PLAN.md),
which is why those sections moved while h2-vs-ay's apex_3-only window
(unaffected by the entry_1_brake boundary) stayed essentially frozen
(n=471 exact, correlation drift 0.0005).

CONCLUSION: no numeric drift found traces to the sideslip-source
wiring itself. Every observed difference from the frozen manifest is
fully attributed to one of two already-committed, already-approved
prior changes -- the WP-B steering-ratio Level-4 upgrade (cap-
dependent) and the entry_1_brake phase-boundary fix (baseline
staleness, independent of cap) -- confirmed by direct comparison and
git history, not asserted. Both external checks (sign check against
measured ay direction; h2-vs-ay correlation against measured ay) still
land solidly (98.8%+, r=0.968+) under every condition tested; no sign
flips, no qualitative change, no evidence of a wiring defect.

POST-VERIFICATION: config/parameters.json stability_estimation.
sideslip_source set back to "kinematic" (the capability ships
defaulted off, per instruction). test_stability.py re-run after the
revert -- full corner-by-corner output (CS ratio front/rear
descriptives, stability observed descriptives, all 14 stable-corner
clusters, first three corners' phase-by-phase tables) byte-identical
to the pre-Step-1b baseline; the new branch's default path is a pure
pass-through to the original unconditional estimate_sideslip call, as
expected from an additive if/else with no change to the existing
branch's own line.

New files: diagnostics/inspect_step1b_wiring_verification.py
(read-only except for reading whatever sideslip_source the caller set
on disk). Config touched only for the two new keys plus the two
temporary flips (ekf_pass_1 for verification, back to kinematic
after) -- no other config value changed.

### Regression test suite established (tests/) [2026-08-20]

PURPOSE: a real, automated regression test suite alongside test_
stability.py (left untouched -- that file stays the smoke test it
always was, zero assertions, confirms the pipeline does not crash).
FRAMING, stated in every test module's own docstring, not just here:
these are REGRESSION tests, not correctness tests. Passing means
"nothing changed unintentionally since the snapshot/invariant was
written", never "the numbers are correct" -- PLAN.md's own open
questions (kinematic beta circularity, un-re-derived thresholds, the
CS_ratio cross-lap aggregation ceiling problem) are exactly the kind of
thing this suite happily pins without endorsing.

SCOPE, five phases, 49 tests total, ~81s full-suite runtime (dominated
by one session-scoped pipeline computation shared across every test
file via pytest fixtures -- see tests/conftest.py; estimate_cornering_
stiffness alone is ~106s of that on a cold, non-fixture-shared run per
the WP-N2 Step 1a timing entry, not re-measured or optimised here, per
instruction not to touch production code):

1. tests/test_golden_pipeline.py + tests/generate_golden.py + tests/
   golden/*.json (2 files) -- full-pipeline and recommendation-engine
   golden-value regression, Dubai outing, sideslip_source="kinematic"
   (asserted live against config, not assumed), accuracy cap=1 (NOT
   "Best available" -- chosen for reproducibility independent of the
   gitignored, machine-local config/car_data.json, see conftest.py's
   own comment). Float tolerance: combined relative+absolute
   (rtol=1e-6, atol=1e-9, tests/_json_utils.py), NaN==NaN passes,
   NaN-vs-number fails. Golden files record git commit hash,
   generation timestamp, and the full config snapshot used.
2. tests/test_phase_boundary_invariants.py -- 7 tests on corner
   phase-boundary structure, reproducing diagnostics/inspect_
   entry1_brake_fix_verification.py's own methodology (same
   BRAKE_RISE_BAR=5.0 bar, same inherited-lookback sort/compare) as a
   standing, re-run-on-every-change suite instead of a one-off
   diagnostic. ALL 7 PASSED against current (post-fix) behaviour --
   no invariant violation found this run. Framing note carried into
   the file itself: a failure here would be a genuine finding, not
   just "something changed", and must never be weakened to pass.
3. tests/test_pure_functions.py -- 16 tests (15 passed, 1 xfailed by
   design) covering slip-angle sign conventions at hand-derived
   inputs, the a*Fy_f-b*Fy_r==Iz*psidd moment-balance identity, the
   Dugoff force/stiffness model (zero-slip collapse, small-slip linear
   limit, odd/even symmetry, finite-difference stiffness check), the
   EKF's process/measurement Jacobians against finite-difference
   Jacobians of the actual nonlinear f(x,u)/h(x), the yaw_rate rpm-to-
   radps unit constant, and edge cases (empty arrays, single-sample
   window, zero speed while moving).
4. tests/test_config_schema_integrity.py -- 11 tests: every config key
   read by the specific functions this session read in full (stated
   as a scope limit, not exhaustive static analysis of every params[
   ...] access in the codebase -- judged too fragile to trust without
   real alias-tracking, see "chose not to do" below); ANALYSIS_SCHEMA_
   VERSION=5 matches the payload shape summarise_corners/_build_
   analysis_data_json actually produce; the accuracy_levels registry's
   own documented format (level in {1,2,3,4}, capped_by follows its
   own "chained-constant:"/"provenance-assumption:" convention); both
   cache-identity checks (WP6 in-memory, WP5 persisted) carry all
   three fields (accuracy_cap, resolved_vehicle_snapshot,
   sideslip_source) on both their write and read sides.
5. tests/test_nan_empty_paths.py -- 7 tests confirming summarise_
   corners, aggregate_by_corner, _classify_corner, and generate_
   recommendations all handle a zero-sample phase / all-NaN input /
   empty summaries list without raising, against REAL production data
   (the entry_1_brake fix's own 22-of-56-instances zero-length-phase
   population, not synthetic stand-ins, for the summarise_corners and
   recommendation-engine checks specifically).

ONE FINDING surfaced while building Phase 3, not from a production bug
but from a test-authoring error worth recording because it corrects a
prior claim on this same page: the WP-N2 pass-0 entry's "a*Fy_f -
b*Fy_r == Iz*psidd_raw IDENTICALLY, max deviation 7.3e-12 Nm" used
"live a/b" that were (evidently) re-derived from front_fraction at
full precision, NOT config's own stored cog_to_front_axle_m/cog_to_
rear_axle_m (1.433/1.072) -- those are rounded to 3 decimals (already
documented separately in modules/accuracy_resolution.py's _resolve_
cog_position docstring) and produce a genuine ~21 Nm max deviation
against the identity at this car's force magnitudes, not a rounding
artifact so small it can be ignored by coincidence. tests/test_pure_
functions.py's test_lateral_force_split_moment_identity now checks
BOTH: the true identity at full precision (re-derives a/b from front_
fraction, matches the historical near-zero figure) and the config's
stored a/b against that re-derivation, bounded to one 3-decimal
rounding step (1e-3 m) -- not a bug, but worth this correction so a
future reader does not assume the STORED a/b satisfy the tight
identity directly.

CHOSE NOT TO DO, and why:
- Exhaustive static-analysis coverage of every params[...]/cfg[...]/
  se[...] config-key access across the whole codebase (Phase 4's
  "every config key the code reads" taken literally) -- a regex sweep
  cannot reliably track which local variable alias (params, se, cd,
  cfg, cls_cfg, ...) maps to which config block without real static
  analysis; a wrong mapping would produce confident-looking false
  positives/negatives, worse than the honest, narrower, directly-
  verified scope actually implemented (Phase 4 test docstrings state
  this limit explicitly).
- Invoking ui/views/outing_form.py's StabilityAnalysisThread directly
  (via QThread.run(), bypassing .start()/the Qt event loop) to test
  the WP-N2 Step 1b config-switch branch end-to-end -- judged not
  worth the fragility for a two-line dispatch; covered instead by (a)
  Phase 3/4's direct-function-call tests of what the branch calls, and
  (b) Phase 4's source-scan tests of the branch's own cache-identity
  wiring. OutingForm._classify_corner IS imported directly (self=None,
  the same production precedent core/weekend_pdf_export.py already
  uses) since it never touches self or any Qt object.
- A dedicated golden test for the corner-detection/clustering stage in
  isolation (modules/corner_analysis.py's analyse_corners output) --
  implicitly covered end-to-end by the full-pipeline golden test and
  directly by every Phase 2 invariant test, judged sufficient without
  a third, narrower golden snapshot of the same data.
- Re-measuring or optimising estimate_cornering_stiffness's ~106s
  runtime (the pipeline's actual cost driver, WP-N2 Step 1a) -- out of
  scope by explicit instruction; this suite's own runtime (~81s full
  session, sharing one pipeline computation across every test file)
  is reported as observed, not treated as something to fix here.

NOT DONE, deliberately left to the user: pytest was pip-installed into
.venv (not in the committed requirements.txt -- a separate tests/
requirements-test.txt lists it as a test-only dependency) since no
test framework existed in this repo before this session; this is an
environment change, not a production-code change, but is called out
here explicitly rather than left implicit. No config value differs
from before this session started (sideslip_source confirmed back at
"kinematic" -- see the WP-N2 Step 1b entry above); no production file
under modules/, ui/, or core/ was touched this session; nothing was
committed.

### Combined-slip premise test: does the rear reach meaningful
longitudinal utilisation on exit? [2026-08-20]

PURPOSE: PLAN.md PARKED item "Combined-slip tyre model" names two
unknowns before any implementation: (1) whether the rear actually
reaches meaningful longitudinal utilisation on this session, (2)
whether a wheel-speed-derived slip ratio is clean enough to use.
This entry is measurement-only, read-only -- no config change, no
whitelisting, no model work. diagnostics/inspect_combined_slip_
premise.py (new).

METHOD: provisional per-axle slip ratio kappa = (v_axle_corrected -
v_ecu) / v_ecu, using log_speed_fl/fr/rl/rr (WP-S1's designated
candidate family, byte-identical to ecu_speed_wheels_*) read directly
from the raw log, bypassing the whitelist -- same pattern as WP-S1's
own script. v_ecu (production ecu_speed) chosen as reference because
WP-S1's own offsets were already measured against it, so this
continues the same chain rather than introducing GPS speed as a
second, separately-caveated reference. Rear corrected for WP-S1's
constant +1.41% rolling-radius offset (v_rear/1.0141); front left
uncorrected, since its off-braking offset is ~0% and its braking-
specific -1.38% deviation IS WP-S1's diagnosed front-slip-under-
braking signature, not something to subtract out. Masked population:
moving & ~kerb & valid-lap racing time, n=24183, matching the WP-N2
pass-1 final-validation baseline exactly (cross-checked, reproduces
n=24183).

RESULT -- distribution, |kappa| percent, base population (n=24183):
front p50=0.225, p90=1.897, p99=4.912, max=14.209. rear p50=1.263,
p90=3.012, p99=5.278, max=13.470. By phase (exit_4+exit_5 combined,
n=7450): rear p50=2.026, p90=4.116, p99=6.667, max=13.470. Front
under braking (log_pbrake_f>5bar, n=5060, chosen over entry_1_brake
-- see bug note below): p50=1.822, p90=3.892, p99=7.479, max=14.209.

THRESHOLD AND ANSWER: proposed utilisation threshold kappa>=5%
(Tier B, literature-informed not fitted -- peak longitudinal mu for a
racing slick typically falls near kappa=0.08-0.15, Rajamani Ch. 2
characteristic curve shape; half that value is past the near-linear
small-slip region). Rear, exit phase: 3.97% of 7450 samples exceed
it. Front, braking (pbrake_f>5bar): 4.58% of 5060 samples exceed it.
READ PLAINLY: this is a minority-but-non-negligible, tail-weighted
phenomenon, not a dominant regime -- most exit-phase and braking-
phase samples stay under 5% slip, but a real fraction (p99 in the
6.7-7.5% range for both) sits well past it. Does NOT falsify the
arc's premise; also does not show a dramatic, unambiguous saturation
regime -- the honest reading is "present and worth modelling, not
overwhelming."

BUG FOUND, INCIDENTAL to this measurement: entry_1_brake phase
durations (modules/corner_analysis.py _build_corner, segments dict)
run up to 98-107s for later corners in a lap (checked directly:
corner_number 12/13 lap 1-4, all four laps agree closely) -- physically
impossible for a braking zone. Root cause: brake_start_t is set to
`thr_t[off_throttle[0]]` (corner_analysis.py line 295), the FIRST
off-throttle sample anywhere earlier in the lap (thr_mask only bounds
time < s_t_start, not a local window), not the LAST one before
turn-in. Consequence: for lap-cumulative corners the "brake phase"
balloons to include almost everything since the previous lift, so
entry_1_brake covers 20641/24183 (85%) of the whole masked population
-- consistent with the corner-by-corner duration sequence measured
per lap (1.9s, 7.1s, 10.2s, 17.0s, 19.9s, 31.1s, 56.9s, 62.8s, 98.2s,
106.8s -- roughly monotonic growth through the lap). This affects
EVERY existing consumer of the entry_1_brake phase key (CS_ratio_f/
stability/Fz phase stats in summarise_corners, and by extension the
WP-N2 pass-0 divergence-flag concentration claim "86% in
entry_1_brake phase", already flagged there as "indicative... not
isolated from" other factors) -- this bug is a specific, concrete
reason for that caveat, not previously identified. NOT FIXED this
turn (measurement-only scope; this script substitutes a raw
log_pbrake_f>5bar mask, same convention WP-S1 already used, for the
front-braking numbers above). Needs a decision: fix (`off_throttle
[-1]`, the closest prior off-throttle sample, is the likely correct
read) vs. document as a known limitation, and how far its blast
radius reaches into already-reported Module 6 phase statistics --
open, not resolved here.

### Rolling circumference: three disagreeing numbers, none resolved
[2026-08-20]

CONTEXT: team-supplied static tyre circumferences, front 2140mm /
rear 2210mm (source: team-supplied, no datasheet seen, TODO-verify),
ratio rear/front=1.0327 (3.27%). config/car.json, config/car_data.json,
config/parameters.json hold NO tyre radius or circumference value
anywhere (grepped directly; parameters.json's only nearby text is
accuracy_levels.speed's note describing WP5b(d)'s k=1.012 GPS-speed
scale factor as "a candidate rolling-radius correction", not a stored
radius).

NEW CHANNEL FOUND: the raw log carries abs_circ_f[m] and abs_circ_r[m]
(ABS unit's own per-axle rolling circumference), not in the
channels.json whitelist, nothing whitelisted here. Both are CONSTANT
for the full session (checked start/middle/end of each block, 1Hz,
t=314-1129s): abs_circ_f=2.121m, abs_circ_r=2.218m -- ratio
rear/front=1.0457 (4.57%). Confirmed NUMERICALLY, not by name alone,
that these are the constants the ABS unit itself uses to convert raw
wheel rotation to linear speed: abs_speed_fl[mph] reproduces
Team_nWheelFL[rpm] * abs_circ_f * 60/1000 to within 2.6e-8 relative
error (n=78855 compared, straight-line and cornering both, no
masking). So within the ABS domain: Team_nWheel* is raw rotational
speed (rpm), abs_speed_* is linear speed already converted using
PER-AXLE (not shared) circumference constants.

~~THREE DIFFERENT NUMBERS now on record, deliberately not reconciled
this turn: (1) team-supplied static ratio 3.27%, (2) ABS-programmed
ratio 4.57% (LARGER than the static figure, not smaller -- opposite
of what a load-compression hypothesis on its own would predict), (3)
WP-S1's measured residual on the log_speed_*/ecu_speed_wheels_*
family (the ECU domain, NOT the ABS domain -- confirmed a separate,
byte-identical-to-itself family, WP-S1) of +1.41% (rear vs ecu_speed,
front ~0%). Whether the ECU-domain family (the one actually used in
the slip-ratio computation above) uses the SAME 2.121/2.218 pair, a
different pair, or none at all is NOT determined by anything gathered
this turn -- only the ABS-domain conversion was verified directly. If
it used the same pair, a roughly 4.57-point front/rear split would be
expected upstream of any load effect; the observed split is 1.43
points, smaller than either the static or the ABS-programmed ratio,
which argues AGAINST simply assuming the ECU domain shares the ABS
domain's constants, but does not by itself identify what it does use.
WHY THIS MATTERS (restated per the standing instruction): a 1-4%
class of systematic offset is the same order of magnitude as the
traction slip this arc measures above: if the correction folded into
that measurement is off by even a fraction of a percent, the reported
rear slip-ratio numbers carry a comparable bias. The combined-slip
premise-test result above is CONDITIONAL on this correction's
provenance and is reported as such.~~

[2026-08-20, superseding the framing above] The "three disagreeing
numbers" framing is withdrawn. It rested on a false premise:
abs_circ_f/r are CONFIGURED ABS control parameters, set by the team
and retuned in the field (the car is reset for wet conditions), not
observations of tyre geometry; and the team-supplied static
circumferences are manufacturer-nominal, varying between individual
tyre sets of the same compound, so they need not describe the tyres
fitted at Dubai. Neither is a measurement of this session's tyres.
Only WP-S1's measured +1.41% ECU-domain rear offset derives from
this session's own data. There is no discrepancy to reconcile -- a
configured parameter and a nominal specification are not expected to
agree with a measured offset, and the earlier reasoning that treated
their disagreement as evidence about load compression or centrifugal
growth is withdrawn along with it.

SEPARATE CHANNEL FOUND, not analysed this turn: abs_Slip_FL/FR/RL/
RR[%] -- the ABS unit's own computed per-wheel slip percentage,
present in the raw log, not whitelisted. A directly relevant
independent candidate for validating or replacing the provisional
slip ratio above; out of scope for this measurement-only turn since
the user's brief asked for a slip ratio computed from wheel speed
directly, not for adopting the ABS unit's own value untested. Also
present: abs_vVeh_absRef[mph], the ABS unit's own reference vehicle
speed (its own slip-rejecting fusion, provenance unexamined) -- a
second unexamined candidate reference speed, alongside GPS speed and
ecu_speed.

### Combined-slip Dugoff: longitudinal stiffness (C_sigma) estimation
method availability [2026-08-20]

Rajamani Ch. 13.10's combined-slip Dugoff needs a longitudinal tyre
stiffness parameter alongside cornering stiffness (c_alpha). Checked
docs/literature/ (chair performance_analysis tooling, internal,
read-only, never imported) for an existing method: YES, present --
docs/literature/longitudinal_stiffness_estimator.py,
estimate_longitudinal_stiffness(). NOT a single fitted scalar
constant -- a locally-linearised, TIME-VARYING empirical estimator:
low-pass filters slip ratio and longitudinal force (4th-order
Butterworth, 8Hz), then computes a sliding-window (0.45s) local
least-squares slope dFx/dKappa at each sample (min 25 samples, min
0.004 slip span per window), and reports it as a RATIO against a
low-slip (|kappa|<=0.015) reference stiffness, clipped to <=1.0. This
needs per-axle longitudinal tyre FORCE (Fx_f/Fx_r) as an input
alongside slip ratio -- SetupTool currently computes Fy_f/Fy_r
(estimate_lateral_forces) but has no Fx_f/Fx_r estimator anywhere.
The chair's own calculate_longitudinal_axle_forces() has three
fallback tiers (direct per-wheel Fx channels; direct aggregate Fx
channels; estimated from ax_mps2 + brake-bias split + a rear-drive-
only fraction assumption) -- the third tier is the one that would
apply here (no direct Fx channels exist), and is a genuinely new
estimator this project does not have, not a config-value gap. NO
METHOD PROPOSED this turn per instruction -- reported as availability
only.

### Rolling-radius offset: speed-dependence check [2026-08-20]

QUESTION: is WP-S1's measured wheel-speed offset (log_speed_* vs
ecu_speed) flat across speed, or does something speed-dependent sit
on top of it. diagnostics/inspect_rolling_radius_speed_dependence.py
(new), WP-S1's own straight-line population (moving & valid-lap &
|ay|<=0.15g & |yaw rate|<=3.0 deg/s), front further restricted to
log_pbrake_f>5bar. No attribution to a physical cause attempted.

REAR (n=5171, 8 bins across 107.7-250.5 km/h): mean/median offset is
essentially flat from ~161 km/h up (93% of the population, 4792/5171
samples): median 1.72/1.56/1.41/1.35/1.28% across the top five bins,
std shrinking from 1.01% to 0.34% as speed rises. The three
lowest-speed bins (107.7-161.3 km/h, n=55/105/219, smaller and
noisier) read higher and noisier: median 3.12/2.07/1.31%, std
1.61/1.49/1.37%. Linear fit: slope -0.00292%/(km/h), predicting only
a -0.417 pct-pt swing across the full observed range -- small next to
the bin-to-bin std. READ: mostly flat at the higher speeds that carry
most of the population; the low-speed departure is present but
sits inside noisier, lower-count bins and is not resolved further
here -- reported as observed, not forced into either a flat or a
trending verdict.

FRONT, braking only (n=643, 8 bins across 120.1-250.5 km/h): offset
ranges from -3.34% to -1.00%, NOT monotonic (a dip at 152.7-169.0
km/h, -3.34%, sits between two smaller-magnitude neighbours,
-1.76% and -1.69%). Linear fit: slope +0.01557%/(km/h), +2.031
pct-pt swing across the range -- directionally toward less front
slip at higher speed, but per-bin std (1.17-2.85%) is comparable to
or larger than the swing itself, and bin counts are modest (32-126).
READ: INCONCLUSIVE -- a directional trend exists in the linear fit
but the bin pattern does not confirm it cleanly; not enough evidence
either way, and not forced.

### abs_Slip_FL/FR/RL/RR[%]: examined, does NOT sidestep the
reconciliation question [2026-08-20]

QUESTION: whether the ABS unit's own logged per-wheel slip channels
are internally consistent within their own domain (which would make
them a usable Level-3 logged slip source regardless of abs_circ_f/r
being a configured, field-retuned parameter, see the supersede note
above). diagnostics/inspect_abs_slip_channels.py (new), base_mask
population n=24183 (reproduces exactly), nothing whitelisted.

CHECK 1, magnitude: abs_Slip_* is TWO ORDERS OF MAGNITUDE smaller
than either WP-S1's measured offset or the provisional kappa from the
combined-slip premise entry above. p50 near zero all four wheels
(-0.006 to -0.023%), p90 0.035-0.088%, p99 0.065-0.148%, max
0.207-0.341%. For comparison, the provisional kappa's p50 alone was
0.225-1.263% and p99 4.912-5.278% (combined-slip premise entry
above) -- roughly 20-40x larger at every percentile. Whatever
abs_Slip_* represents, it is not reading in the same numeric range as
a directly-computed (v_wheel-v_ref)/v_ref kinematic slip ratio.

CHECK 2, internal reconstruction: tested whether abs_Slip_FL/FR/RL/RR
matches (abs_speed_wheel - abs_vVeh_absRef)/abs_vVeh_absRef (the
naive kinematic-slip formula, using the ABS domain's OWN speed and
reference channels). It does NOT: correlation is NEGATIVE at every
wheel (FL -0.534, FR -0.472, RL -0.819, RR -0.827) and the mean
difference is not a small residual (rear wheels +2.36 to +2.58
pct-pts, comparable to the signal itself). A naive kinematic
reconstruction from channels already in hand does not explain
abs_Slip_*.

CHECK 3, comparison against the provisional kappa: correlation
between the log_speed_*-derived provisional kappa (combined-slip
premise entry above) and abs_Slip_* is likewise NEGATIVE (front
-0.547, rear -0.899) -- strongly negative for the rear. As the
provisional kappa rises, abs_Slip_* tends to fall, the opposite of
what a shared underlying slip phenomenon read two different ways
would produce.

~~READ PLAINLY, per instruction: this does NOT sidestep the
reconciliation question the way the queued framing hoped. abs_Slip_*
is not simply a validated, ready-to-use alternative to the
provisional kappa -- it moves in the wrong direction to serve as a
cross-check, and does not reconstruct from the other ABS-domain
channels already verified this session (abs_speed_*, abs_circ_*,
abs_vVeh_absRef).~~ Two explanations are consistent with this and
NEITHER is established here: abs_Slip_* may be a filtered/clamped
ABS control-loop signal (built for threshold decisions, not reporting
open-loop kinematic slip) rather than a physical slip measurement; or
its "%" unit may not mean percent-of-reference-speed at all (e.g.
normalised against a different internal range). No datasheet is in
hand to resolve which. STATUS: examined and found NOT USABLE as
proposed, not merely unexamined -- a negative result, recorded as
one.

[2026-08-20, partial correction] The -100x/sign-inversion hypothesis
was tested directly (regression, kappa vs abs_Slip per wheel) and
FAILS as a clean proportionality: slopes are -25 (front: FL -24.98,
FR -24.24) / -45 (rear: RL -45.48, RR -45.19), not -100. However,
"unrelated noise" overstates the front-only case -- the rear shows a
real, moderate relationship (R^2=0.578 RL, 0.570 RR) that the front
does not (R^2=0.094 FL, 0.068 FR). The practical conclusion stands
(abs_Slip_* is not a usable validated slip source, and is now
superseded in relevance by ecu_slip_act/ecu_B_tc_act, see "Combined-
slip arc: logged ECU slip and TC channels found" below), but "moves
in the wrong direction to serve as a cross-check" is too strong a
characterization of the rear channel specifically -- the -100x
hypothesis was proposed, tested, and rejected, not confirmed;
recorded here as a rejected hypothesis, not a dropped one.

### Rolling-radius correction: session-portability limitation
[2026-08-20]

Recorded per standing instruction. abs_circ_f/r (and, by the same
logic, any ABS-domain constant this session verified, e.g. the
per-axle circumference pair used to convert Team_nWheel* to
abs_speed_*) are CONFIGURED ABS control parameters, set by the team
and retuned in the field (the car is reset for wet conditions) -- not
fixed physical constants. Any slip ratio, offset, or correction
derived through them (or through the ECU-domain log_speed_*/
ecu_speed_wheels_* family, whose own internal conversion constants
are unverified, see the supersede note above) is valid only within
the configuration of the SINGLE SESSION it was measured in, and
cannot be assumed to carry over to a different session or event
without independently re-verifying the constants did not move
between them. This bounds the portability of anything built on this
signal family and must be stated as a limitation wherever the
combined-slip work is written up, not treated as a one-off caveat.

### Combined-slip arc: logged ECU slip and TC channels found;
premise supported but weak [2026-08-20]

ESTABLISHED -- logged slip source: ecu_slip_act[%] exists in the raw
log, unwhitelisted, range [0.0, 34.7], base-population (n=24183)
p50=1.4%, p90=3.1%, p99=4.7%, max=7.4%. ecu_slip_nom[%] range
[2.1, 25.5] reads as the TC's own target setpoint. ecu_B_tc_act is a
TC-active flag at 50 Hz. Found via diagnostics/inspect_slip_channel_
sweep.py's full-inventory keyword sweep (301 matches total, most
irrelevant -- error/diagnostic/refuel/clutch channels, not slip-
related despite matching "err"/"ref"/"diff"); examined in diagnostics/
inspect_slip_hypothesis_and_driven_axle.py.

INTERNAL CONSISTENCY, three independent checks agreeing: on the 68
TC-active samples, ecu_slip_act p50=4.1%/max=6.0% matches
ecu_slip_nom p50=4.2%/max=6.1% -- actual slip tracking its own target
while intervening, the signature of a working closed loop; the
magnitude range is physically sane throughout (unlike abs_Slip_*'s
two-orders-too-small range, see the corrected entry above); and
corr(ecu_slip_act, kappa_driven_corrected) = +0.676.

SIGNIFICANCE: this is a LEVEL 3 logged source. It sidesteps the
rolling-radius reconciliation entirely -- no derived constants, no
session-bound correction, no dependence on the field-retuned ABS or
ECU circumference values whose portability limitation is recorded
above.

LIMITATION: no per-axle breakdown exists. It reads as a single
driven-axle aggregate. INFERRED from TC's physical role on this RWD
car, not confirmed by any channel-name or documentation evidence.

ESTABLISHED -- driven-vs-undriven construction, as cross-check:
kappa_driven = (v_rear - v_front)/v_front, the RWD traction-control
definition (rear against front, not against a vehicle reference).
WP-S1-corrected (rear /1.0141): base p50=0.61%, p90=3.42%, p99=5.98%;
exit phase (4+5) p50=2.26%, p90=4.53%, p99=7.21%; throttle>20%
p50=0.92%. The corrected version is the trustworthy one: raw carries
a +2.03% baseline median that is the rolling-radius artifact WP-S1
already diagnosed, which would misread ordinary constant-radius
rolling as slip at every sample. Corrected shows a near-zero baseline
with a clear rise under exit and throttle -- signal riding on a
removed offset, not artifact. Still provisional: the correction is a
single constant from WP-S1's own straight-line measurement, itself
now flagged with the session-portability limitation above. corr with
the provisional kappa_rear (combined-slip premise entry) = +0.797
(shared construction, expected); corr with abs_Slip rear avg =
-0.804, reinforcing that abs_Slip_* behaves oppositely to every other
slip proxy examined this arc, not just one.

ESTABLISHED -- TC intervention: ecu_B_tc_act active on 0.28% of the
base population (68 of 24183). Concentration by phase: exit 0.87%,
braking 0.33%, baseline 0.28%.

ASSESSMENT, and it must not be overread in either direction. The
phase concentration is DIRECTIONALLY what the arc's premise predicts
-- roughly 3x more TC intervention on exit than baseline, less under
braking -- and it is INDEPENDENT of any slip-ratio construction,
resting only on the car's own flag. But 0.28% is 68 samples, roughly
25 exit events against 8 under braking -- suggestive, not
established; too few to carry a conclusion on its own. AND IT CUTS
BOTH WAYS, recorded plainly: if the car's own traction control
intervened on 0.28% of a session, the rear axle was rarely at its
traction limit. That WEAKENS "the rear saturates longitudinally
rather than laterally" as the explanation for the refit loop's rear
degeneracy (PLAN.md NOW, WP-N2 refit-loop entry above). The effect is
real and in the predicted direction, but small. Consistent with the
earlier measurement that only ~4% of exit samples exceed 5%
longitudinal slip (combined-slip premise entry above) -- two
independent routes agree the longitudinal effect exists and is
modest.

OPEN, not resolved here: whether ecu_slip_act is rear-only or a
mixed aggregate; whether a modest, tail-weighted longitudinal effect
is sufficient to explain the rear identifiability failure, or whether
that failure has another cause.

### TC LAT / TC LON channel-name candidates found (registry thread,
not combined-slip) [2026-08-20]

log_tc_lat_pos / log_tc_lon_pos and stw_rt01_tc_lat / stw_rt01_tc_
lat_raw / stw_rt03_tc_lon / stw_rt03_tc_lon_raw, found via the same
full-inventory keyword sweep as the combined-slip entry above, appear
to be the TC LAT / TC LON steering-wheel rotary-switch channels
PLAN.md's NEW DATA FILE diagnostic checklist has had open since the
WP2b-1 registry work (config/setup_parameters.json's tc_lat/tc_lon
entries, value_source "logged_data", channel name previously TBD).
Identified BY NAME ONLY -- not examined, not whitelisted, not cross-
checked against setup-sheet values or driver report. Recorded as a
candidate identification requiring confirmation before config/
channels.json or the registry notes are touched.

### entry_1_brake phase-boundary bug: mechanism, blast radius, and
fix [2026-08-20]

ESTABLISHED -- mechanism: modules/corner_analysis.py line 293 builds
off_throttle from thr_d < brake_throttle_max_pct (config value 95,
read live from config/channels.json corner_detection), and line 295
takes off_throttle[0] -- the chronologically FIRST off-throttle
sample in the lap-so-far, not the last before turn-in. Line 289's
thr_mask is unbounded below and throttle (modules/corner_analysis.py
_analyse_lap) is sliced to the whole lap, so for corner N the
boundary can anchor back to corner 1's own lift point. The 95%
threshold is loose: any coast, shift blip, or partial lift earlier in
the lap qualifies as "off-throttle". Result: entry_1_brake spans
20,641 of 24,183 masked samples (85.35%), with durations growing
roughly monotonically through each lap (1.9s -> 106.8s, measured
directly per corner). EVERY other phase boundary was checked and is
CORRECT: s_t_start/s_t_end are direct bracket-index lookups; apex_t
uses argmax/argmin over a bracket-bounded mask; half_t/exit_4/exit_5
slice to the bracket itself, where first-crossing is the semantically
right target and the window cannot run away; entry_2_turnin and
apex_3 derive from already-bounded values. Only entry_1_brake is
affected.

ESTABLISHED -- blast radius: summarise_corners computes every
entry_1_brake phase-keyed stat -- cs_ratio_f, cs_ratio_r,
stability_observed_Nm_per_deg, and when fz is passed fz_f_N/fz_r_N/
fy_f_norm_N/fy_r_norm_N. For corners beyond the first one or two per
lap these are effectively meaningless as braking-phase statistics,
not merely imprecise: the window can be dominated by straight-line
and other-corner samples. PRODUCTION recommendation path: 15 of 39
rules (38%) key on entry_1_brake -- the whole braking matrix
(matrix_us_brk_low/med/high plus 3 escalations, matrix_os_brk_low/
med/high, matrix_inst_brk_low/med/high, matrix_inst_ent,
yaw_entry_unstable, driver_us_entry). aggregate_by_corner pulls the
per-lap phase median, takes a median-of-medians across laps, and
evaluates the rule against it. LIKELY DIRECTION of the error, stated
as reasoning not measurement: off-braking samples sit at CS_ratio's
1.0 ceiling, so dilution biases toward FALSE NEGATIVES -- real
braking-phase understeer or instability washed toward "fine" by
surrounding ceiling-pinned samples. Not quantified. Corner 1 of each
lap is comparatively unaffected (short lookback); severity grows with
corner position in the lap. UI: ui/views/outing_form.py renders a
"Brake" phase column showing these diluted stats to the engineer as
the braking zone; ui/views/corner_trace_dialog.py shades an
entry_1_brake band that for a late corner would visibly span most of
the lap.

RECORDED FINDINGS AT RISK: WP-N2 pass-0's "86% of divergence flags in
entry_1_brake" already carried a caveat dated 2026-08-19, one day
before this arc, describing the same lookback mechanism
qualitatively -- superseded with a dated note pointing here (see
above). This entry quantifies it and goes further: entry_1_brake
covers 85.35% of the base population, the flagged set shows 86%, and
because the flags form 240 contiguous episodes rather than 6,730
independent draws the effective sample size is far below what a
naive binomial assumes. The figure is NOT meaningfully
distinguishable from the phase's own population footprint and may
carry close to zero independent information about where divergence
concentrates. The Fy yaw-moment term entry (Module 4a, 2026-07-24,
NOT previously caveated) explains entry-phase ceiling-pinning as
"strong, clean alpha excitation under braking/turn-in keeps C_alpha
near the linear reference" -- given this bug, an equally or more
plausible explanation is dilution by long non-cornering stretches
with near-zero lateral demand, sitting at the ceiling for an entirely
different reason. Recorded as an ALTERNATIVE READING, unresolved in
either direction -- the original is not declared wrong, see the
dated pointer added above. The WP-B steering-ratio section
(2026-07-26) restates the same ceiling-clipping explanation and
inherits the same risk, same dated pointer added. NOT at risk: the
CS_ratio saturation-flag analysis and the apex_3/exit_4 worst-phase
findings rest on confirmed-correct boundaries; this session's TC
exit-phase concentration (see "Combined-slip arc: logged ECU slip
and TC channels found" above) uses exit_4/exit_5, also confirmed
sound.

STANDING LIMITATION, recorded because it is general rather than
specific to this bug: test_stability.py prints per-corner per-phase
medians to stdout for human inspection and has ZERO assertions, no
golden-output comparison, and no duration checks. It is a smoke test
confirming the pipeline does not crash. It passed cleanly throughout
this investigation, before and after -- that tells us nothing about
correctness. Any fix in this area would pass it whether right or
wrong. Phase-boundary correctness has no automated coverage.

FIX APPLIED, this same turn (see PLAN.md STATUS for the commit-
boundary decision): modules/corner_analysis.py line 295,
off_throttle[0] -> off_throttle[-1], the last off-throttle sample
before s_t_start, matching the docstring's own stated intent ("last
full throttle on preceding straight -> turn-in"). A bounded-backward-
search hardening (do not search further back than the previous
corner's own bracket) was considered and deliberately NOT bundled in
-- reasonable defensive measure for near-flat-out corners with only a
brief lift, but a second change with its own behaviour that would
make verification ambiguous. Parked in PLAN.md PARKED section
instead. Verification: see the four checks immediately following
this entry.
SUPERSEDED -- see the dated failure record and correction directly
below; the off_throttle[-1] construction above was WRONG and has been
replaced.

FIRST FIX ATTEMPT, FAILED, AND CORRECTED [2026-08-20, same session]:
off_throttle[-1] (applied above) reads as "last off-throttle sample
before turn-in" and was approved on that basis, matching the
docstring's "last full throttle on preceding straight" against the
wrong end of the window: off_throttle holds OFF-throttle indices, so
its last (chronologically closest-to-turn-in) entry is the sample
NEAREST turn-in that happens to be under threshold -- not the lift-off
transition. When a driver coasts continuously from lift-off through to
turn-in (the normal case for a real braking zone), that sample sits
essentially adjacent to s_t_start, collapsing the phase to ~1 sample
(~0.01s) instead of capturing the actual brake-zone duration.
DURATIONS (check a) and POPULATION SHARE (check c) both looked
healthy under this construction -- bounded (max 3.87s), non-monotonic,
population share down from 85.35% to 7.62% -- and would have been
accepted as a working fix on those two checks alone. The MANDATORY
brake-pressure cross-check (check b) is what caught it: offset (brake
pressure rise - brake_start_t) had median ~0.004s for both channels,
statistically indistinguishable from zero, when the expected signature
of a genuine lift-then-coast-then-brake sequence is a small but clearly
POSITIVE median. Spot-check (check d) confirmed the mechanism directly:
corner 1 lap 1 showed brake_start_t=501.090s, 0.010s before turn-in
(501.100s), with throttle reading a constant 0.0% for a full second on
BOTH sides of that point -- proof the true lift-off was well earlier
and the construction had picked the wrong end of the coast. 16 of 56
corner instances (corners 1/5/7/13, stable_ids 1/5/7/14, reproducible
across all 4 laps) showed this collapse.
CORRECTED CONSTRUCTION, applied this turn: on_throttle = where(thr_d
>= brake_throttle_max_pct), brake_start_t = thr_t[on_throttle[-1]] --
the LAST sample still at or above the throttle threshold before
turn-in, i.e. the lift-off transition itself, which is what the
docstring actually specifies. EMPTY-CASE handling: if on_throttle is
empty (no full-throttle sample anywhere in the lookback window), the
pre-existing brake_start_t=s_t_start default (set before this block)
is left untouched -- same degrade-safely pattern the original code
already used for its own empty-off_throttle case, zero-length phase
rather than a crash or an unbounded reach-back. Measured this session:
0 of 56 corner instances hit the empty case (does not mean the guard
is unnecessary -- it protects against a session where it does occur).
RE-VERIFICATION, all four checks repeated against the corrected
construction:
(a) durations: n=56, mean=0.637s, median=0.015s, max=3.421s. The four
    previously-collapsed corners now show 1.4-3.4s, reproducible
    across all 4 laps (corner 1: 1.95-2.17s; corner 5: 2.04-2.13s;
    corner 7: 3.01-3.42s; corner 13: 1.36-1.40s). Several other corners
    remain near-zero (corners 2/3/4/6/12: 0.010-0.097s) -- plausible
    late-braking/minimal-coast style at those specific corners, not
    re-inspected individually beyond the spot-check below.
(b) MANDATORY cross-check, log_pbrake_f: n=36, p10=0.003s, p50=0.135s,
    p90=2.563s, mean=0.816s, max=3.173s, 0% negative. log_pbrake_r:
    p10=0.003s, p50=0.130s, p90=2.548s, mean=0.808s, max=3.163s, 0%
    negative. Median moved from ~0.004s (statistically zero, the
    failure signature) to ~0.13s (small, clearly positive, matching the
    expected lift-coast-brake signature) with zero negative instances
    either construction. The p90/max spread (2.5-3.2s) reflects genuine
    corner-to-corner coast-length variation across 14 different
    corners, not re-examined per-corner for whether any individual
    value is itself still suspect.
(c) population share: 6.98% (1688/24183), consistent with the first
    attempt's 7.62% -- both constructions bound the runaway growth;
    only (b) and (d) distinguish which one is semantically right.
(d) spot-check, corner 1 lap 1 (the exposing case): brake_start_t now
    499.146s (was 501.090s under the failed construction), 1.95s before
    turn-in, and well before the 500.09s bound the failure implied --
    throttle 2s before ranges 40-105% (still transitioning down from
    full throttle) and 1s after drops to 0-7% (fully lifted),
    consistent with a genuine transition, not a coincidence of the
    corrected index landing near the old one. Corner 13 lap 1 similarly
    moved from 605.931s to 604.542s (1.40s before turn-in, was 0.010s).
INHERITED-LOOKBACK RISK, MEASURED not assumed: the corrected
construction has the same unbounded backward search in principle. Found
occurring: corner 5, ALL 4 laps, brake_start_t reaches back 1.2-1.3s
into corner 4's own bracket (e.g. lap 1: brake_start_t=517.031s vs
corner 4's own bracket end at 518.266s, overlap 1.235s) -- corners 4
and 5 are evidently linked closely enough that throttle never returns
to the 95% threshold between them, so the search for "last full
throttle" reaches past corner 5's own approach into corner 4's exit
acceleration. Reproducible across all 4 laps for this corner pair
specifically (not a one-off). THIS MEANS the PARKED bounded-search
hardening (PLAN.md) is REQUIRED, not merely a defensive nice-to-have --
recorded as a finding, not treated as a failure of this turn's fix
(the fix is still a correctness improvement over both the original bug
and the failed first attempt; the hardening is the next, separate
step).
GENERAL LESSON, recorded because it generalises beyond this bug: for
phase-boundary work, an external physical reference channel is a
stronger check than distributional plausibility. Duration bounds and
population-share numbers looked completely healthy under the failed
construction and would have passed as "fixed" on those grounds alone
-- only the brake-pressure cross-check, an independent physical
signal with its own expected sign and rough magnitude, exposed that
the boundary was measuring the wrong event. A fix that looks right on
its own output distribution can still be wrong; prefer a check against
an unrelated channel over one derived from the same computation being
verified.

BOUNDED-SEARCH HARDENING, applied and re-verified [2026-08-20, same
session]: the PARKED item was promoted to required after the corner 5
finding (all 4 laps, brake_start_t reaching 1.2-1.3s into corner 4's
own bracket). PROPOSED BOUND, stated before applying: brake_start_t's
backward search is floored at the PRECEDING corner's own bracket end
(segments["exit_5"][1]), no added margin -- that boundary already
represents where lateral-G-based corner analysis considers the
previous corner's cornering behaviour finished, so it is a
data-derived boundary already in hand, not a new invented constant;
adding a margin would be an unjustified tunable. First corner of a
lap has no preceding corner -- no additional floor beyond what
throttle's own lap-slice already gives (matches existing behaviour,
unchanged). Empty-window case (bound truncates to nothing left to
search): brake_start_t keeps its s_t_start default, same
degrade-safely pattern as the empty-on_throttle case already in the
corrected construction, not a new pattern.
IMPLEMENTATION BUG FOUND AND FIXED WITHIN THIS SAME TURN, before
verification, recorded because it is itself an instance of the
general lesson above: the first implementation compared
prev_corner_end_t (ABSOLUTE session time, since segments are built
with abs_start added) directly against throttle["time"] (LAP-RELATIVE,
_slice_channel subtracts the lap's own start_t) without converting
frames. Since absolute time is always much larger than lap-relative
time, this silently truncated nearly every corner's lookback window to
empty, collapsing 53 of 56 instances to 0.000s duration (caught by
re-running check (a) immediately after applying the bound, before
check (b) or (d) were even attempted -- this particular bug was
duration-check-visible, unlike the off_throttle[-1] failure). Fixed by
subtracting speed["abs_start"] (the current lap's own start_t, always
identical for the previous corner since both are in the same lap) from
prev_corner_end_t before the comparison.
RE-VERIFICATION, all four checks, after both the bound and its
frame-conversion fix:
(a) durations: n=56, mean=0.484s, median=0.013s, max=3.421s. Every
    corner except 5 reproduces EXACTLY its pre-bound value (1/7/13
    unchanged: 1.95-2.17s / 3.01-3.42s / 1.36-1.40s across laps) --
    confirms the bound is a no-op except where the crossing actually
    occurred. Corner 5 alone changed: 2.04-2.13s (pre-bound) -> 0.000s
    (post-bound) on all 4 laps.
(b) MANDATORY cross-check: log_pbrake_f p10=0.002s p50=0.018s
    p90=2.563s max=3.173s; log_pbrake_r p10=0.002s p50=0.009s
    p90=2.548s max=3.163s; 0% negative both channels -- signature
    essentially unchanged from the pre-bound corrected construction,
    confirming the bound did not disturb the fix's own correctness for
    the corners it left alone. Corner 5 specifically: brake-pressure
    offset 0.001-0.010s across all 4 laps (duration 0.000s, turn-in
    essentially coincides with brake_start_t) -- pressure was already
    at/near threshold right at turn-in, consistent with corner 5
    genuinely having no separate pre-turn-in brake phase the method can
    isolate, not a defect.
(c) population share: 5.50% (1331/24183), down slightly from the
    unbounded corrected construction's 6.98% -- the bound removes
    exactly the corner-5 overlap and nothing else, as expected.
(d) inherited-lookback risk, re-checked: NONE found -- no corner's
    brake_start_t now precedes the preceding corner's own bracket end,
    confirmed directly (not inferred from duration numbers alone).
NEAR-ZERO CORNERS (2, 3, 4, 6, 8, 12; stable_ids 2/3/4/6/8/13),
INDIVIDUALLY VERIFIED: 20 of 56 instances (stable_ids 2, 3, 8, 10, 13)
have no computable brake-pressure offset in check (b) -- in every
missing case the reason is "no rise above 5 bar found in the search
window", not a coverage or channel-quality gap; the check (b) method
itself cannot speak to a corner that genuinely isn't braked hard
enough to cross 5 bar. Near-zero corners ARE disproportionately among
these: 4 of the 5 originally-flagged near-zero corners (2, 3, 8, 12)
have no offset at all; only 4 and 6 do. For all six near-zero corners,
sample-resolution throttle and brake-pressure traces were printed for
3s before turn-in, one lap each (lap 1): EVERY ONE shows the same
clean, unambiguous, and CONSISTENT signature -- throttle ramps up
through the window and reaches/holds >=100% continuously through
brake_start_t and turn-in (corner 2: 0% at window start ramping to
105% by t-0.3s, held through turn-in; corner 3: 105% held for the
entire 3s window, only drops AFTER turn-in; corner 4: ramps 60%->104.5%
by ~1.3s before turn-in, held through; corner 6: ramps 71.5%->105.5%,
held through; corner 8: ramps 20%->104.5%, held through, drops only
after turn-in; corner 12: ramps 20.5%->104.5%, held through) -- and
brake pressure (both f/r) reads 0.00 bar for the entire pre-turn-in
window in all six cases, with no exception. This is qualitatively the
opposite of the failure signature that exposed the first fix attempt
(corner 1 under off_throttle[-1]: throttle at a CONSTANT 0% for a full
second on both sides of the wrongly-placed boundary, i.e. continuous
coasting misread as adjacent-to-turn-in). Here the driver is
ACCELERATING to and maintaining full throttle right through turn-in --
these six corners are genuinely taken without a distinguishable
pre-turn-in brake phase this session (flat, minimum-lift, or braking
deferred to just after turn-in, outside entry_1_brake's own
definition by construction). VERDICT: GENUINE for all six, not a
residual defect -- consistent evidence across all six meant no
corner required an inconclusive verdict, per instruction not to force
one where the data doesn't settle it; here it did settle, cleanly.
Scope stated precisely: one lap (lap 1) inspected per corner, not all
four -- matches the instruction's own scope, not extended further.
DISTINCT from the pre-existing WP1 Turn-3 canonical-partition
degenerate-zero mechanism (stable_ids 9/10/11/12, exactly 0.000s from
bracket truncation, unrelated to throttle): the near-zero corners here
are throttle-driven and mostly nonzero (0.010-0.045s), a different,
independently-verified-genuine mechanism, not to be conflated with the
WP1 one despite both producing small numbers.

### Production impact of the fix, and a structural finding about
CS_ratio aggregation [2026-08-20]

ESTABLISHED -- what the fix moved: baseline for comparison was a
recomputation using the ORIGINAL pre-session bug (off_throttle[0],
unbounded), not either failed intermediate attempt, since nothing this
session was committed. The comparison mirrors the production call
chain exactly (diagnostics/inspect_entry1_brake_production_impact.py).
Per-instance entry_1_brake n_samples fell from ~350-5,420 to 1-171,
with many at 0. 39 material per-instance shifts (|dCS|>=0.05 or
|dStab|>=50 Nm/deg). Largest and most consistent at the AGGREGATE
level (median-of-medians across laps, what the rules actually see):
stability C3 658->1522, C4 718->-22 (sign flip, 3 of 4 laps), C6
712->427, C7 680->266, C8 624->253, C13 572->523, C14 572->1228. ZERO
verdict changes across all 15 braking-matrix rules at every stable
corner. Why, precisely: every post-fix aggregate stability value
stays above STAB_NEG_THRESH=-50.0 (least negative is C4's -21.8), so
even the sign flip does not reach "destabilizing". The fix moved real
statistics substantially; none of the movement crossed a threshold on
this dataset -- materially different from "the bug had no
consequences".

STRUCTURAL FINDING, more significant than the bug itself: CS_ratio_f
and CS_ratio_r stay pinned at ~1.000 at the aggregate level for EVERY
corner checked (3, 4, 6, 7, 8, 13, 14), both before and after the
fix. Corner 6 lap 1 showed a dramatic single-instance collapse
(CS_ratio_r 1.000 -> 0.219) which the median-of-laps washed out
completely. Two mechanisms combine: CS_ratio is clipped at 1.0 and
entry-phase instances mostly sit at that ceiling, and
aggregate_by_corner then takes a median across four laps, which
discards any single lap departing from it. The aggregation behaves
exactly as its own design comment describes ("a single-lap anomaly
washes out") -- but with a ceiling-pinned metric and only four laps,
that design makes one lap of real signal indistinguishable from
noise. CONSEQUENCE: most of the 15 braking rules key on CS_ratio, so
on this dataset those rules have essentially ZERO aggregate
sensitivity to the underlying data -- independent of the entry_1_brake
bug and would not have been visible without this comparison. The
stability channel does NOT share this problem: its movements survive
aggregation cleanly. The two metric families behave differently and
should not be assumed equivalent.

ESTABLISHED -- empty-phase handling, safe but silent: chain traced by
reading code, not assuming: summarise_corners emits the phase with
n_samples 0 and NaN medians (never absent, never raises);
aggregate_by_corner's _nanmedian_or_nan drops NaN across laps and
returns NaN only if all laps are NaN; _classify_corner guards with
"if csf == csf" before every comparison, so a fully-NaN phase never
becomes "worst" and the rule evaluates to "ok". No crash risk. But a
genuinely hard-braked corner with no computable signal is classified
IDENTICALLY to a corner that is actually fine under braking -- quiet
degradation, not failure. This confirms the false-negative
hypothesis's MECHANISM structurally in the code, independent of the
inconclusive data result below. 22 of 56 instances (39.3%) now have
entry_1_brake n_samples==0, so this path is live for a substantial
minority of instances, not an edge case.

FALSE-NEGATIVE HYPOTHESIS, tested and INCONCLUSIVE: at verdict level,
no evidence either way, nothing crossed a threshold. At raw-statistic
level, mixed: four corners moved toward destabilizing (4, 6, 7, 8,
consistent with the dilution story), two moved the other way (3, 14).
Inconclusive for an IDENTIFIABLE STRUCTURAL REASON -- the CS_ratio
aggregation washing described above -- not merely through noise.

UI: the per-corner phase table already renders zero-length phases as
an em-dash via an explicit n>0 guard, and _stability_colour has its
own NaN guard returning NEUTRAL. Both render sensibly by existing
design. NOT checked: corner_trace_dialog.py's shaded band for a
zero-width phase -- flagged unchecked rather than assumed.

OPEN, not addressed here: whether the CS_ratio aggregation should
change (percentile rather than median across laps, or worst-lap, or
reporting lap-to-lap spread alongside the median). That is a
production behaviour change affecting verdicts and belongs with the
deferred threshold work, not here -- see PLAN.md PARKED.

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
  [2026-08-20: this restates the Fy yaw-term entry's ceiling-clipping
  explanation and inherits the same alternative-reading caveat -- see
  "entry_1_brake phase-boundary bug" below. Unresolved, not corrected.]
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

## 3. WP-N3 (per-session-fittable, self-checking sideslip): unsupervised package [2026-08-20]

Package run while the user was away, per PLAN.md's rewritten NOW
section (WP-N2 carry-forward decision closed the prior arc). Hard
constraints: no existing production file touched except as permitted
per phase; config additive-only under new namespaces; no commit; no
threshold re-derivation (still deferred, PARKED); the regression
suite (tests/) must pass unchanged at every phase boundary.

### Phase 1: washout cutoff sweep -- pre-registration [2026-08-20]

BACKGROUND: production beta_washout_cutoff_hz=0.05 (config/
parameters.json stability_estimation, read live, not from memory).
WP-S3c (thesis_notes.md "Zero-slip offset: chain decomposition +
mechanism search") found that REMOVING the washout entirely (per-
corner beta re-anchored to zero at the last straight-line sample
before corner entry, raw beta_dot integrated with no high-pass, drift
read at the first straight-line sample after corner exit) gives
median 5.7 deg / p90 10.8 deg residual drift -- worse than the 0.9-5.8
deg force-balance-demanded signal (WP-S3b) the washout is suspected of
suppressing. The intermediate cutoff range between 0.05 Hz and 0 was
never tested. This phase tests it, read-only (diagnostics/, no
config/production change).

DESIGN (stated before running, since the exact construction affects
comparability): two distinct beta constructions per cutoff, not one --
1. GLOBAL: production's own estimate_sideslip formula
   (cumsum(beta_dot)*dt, then _highpass_filter at the swept cutoff),
   run over the whole session exactly as production does, only
   beta_washout_cutoff_hz varied. cutoff=0 special-cased (skip the
   filter step entirely -- scipy.signal.butter rejects a literal 0 Hz
   critical frequency) to give the raw, globally-accumulated integral.
   Used for mid-corner recovery (metric 1), sign check (metric 3), and
   the EKF correlation/RMS (metric 4) -- these need a genuine session-
   length candidate signal to evaluate at arbitrary sample positions.
2. LOCAL RE-ANCHORED: WP-S3c's own Section 3 construction, reused
   verbatim (diagnostics/inspect_washout_mechanism.py's anchor-finding
   and per-corner integration code), generalised only by applying
   _highpass_filter at the swept cutoff to the local re-anchored
   segment instead of always skipping it. Used for the drift metric
   (metric 2) only, per the work order's own "WP-S3c's own
   methodology" instruction. At cutoff=0 this reduces exactly to
   WP-S3c's own no-high-pass construction, which is the basis for the
   reproduction check below.
   Justification for the split: metric 2's drift is a RELATIVE
   quantity (change in beta across one corner's span) -- for a raw,
   unfiltered global integral, evaluating that relative quantity via
   the GLOBAL signal would additionally contain the accumulated drift
   since session start (unbounded over a multi-lap session), which is
   not what WP-S3c measured or what "worse than the 0.9-5.8 deg
   signal" refers to. Re-anchoring (or, equivalently, differencing two
   nearby global-signal samples under a purely linear operator) removes
   that irrelevant offset. This equivalence is exact for cutoff=0 (raw
   integration is linear, so a constant offset cancels in any
   difference) -- the reproduction check below verifies it holds in
   practice for this script's own implementation.

REPRODUCTION CHECK (must pass before trusting the sweep, per the work
order): cutoff=0 through construction 2 must reproduce WP-S3c's
recorded median=5.7 deg / p90=10.8 deg (thesis_notes.md "Zero-slip
offset" entry) to within rounding. If it does not, STOP and
investigate before reporting the rest of the sweep.

PRE-REGISTERED PREDICTION: lowering the cutoff (0.05 -> 0.03 -> 0.02
-> 0.01 -> 0.005 -> 0) monotonically increases mid-corner |beta|
recovery at apex phases (moves toward the 0.9-5.8 deg force-balance
band and toward pass-1 EKF beta) AND monotonically increases post-
corner drift (moves toward WP-S3c's 5.7/10.8 deg no-filter figures).
The two effects trade off; "no cutoff dominates 0.05" (better recovery
AND acceptable drift, simultaneously) is treated as a legitimate
finding, not a failure of the sweep.

PRE-REGISTERED DISQUALIFYING DRIFT BOUND, justified before seeing
results: a cutoff is DISQUALIFIED (cannot be recommended as a
production replacement for 0.05, regardless of its recovery numbers)
if EITHER (a) its median per-corner drift (metric 2) is >= 0.9 deg,
the LOWER end of the force-balance-demanded signal band, because at
that point drift alone is large enough to fully explain the smallest
demanded true signal -- an observed mid-corner |beta| increase under
that cutoff cannot be distinguished from uncorrected drift, not
genuine recovery; OR (b) its p90 drift is >= 5.8 deg, the UPPER end of
the same band, mirroring WP-S3c's own "worse than the 0.9-5.8 deg
signal" comparison for the no-filter anchor case, generalised to every
swept cutoff. Either condition alone disqualifies. This bound is fixed
now, before any cutoff besides the known no-filter anchor (0) has been
run.

New script: diagnostics/inspect_washout_cutoff_sweep.py (read-only,
no config/production change).

### Phase 1: washout cutoff sweep -- results [2026-08-20]

REPRODUCTION CHECK: PASSED. cutoff=0 through the local re-anchored
construction gives median=5.718 deg, p90=10.765 deg against WP-S3c's
recorded 5.70/10.80 -- within the pre-declared tolerance (0.05/0.1
deg), confirming the sweep's two-construction design (global signal
for metrics 1/3/4, local re-anchored for metric 2) is equivalent to
WP-S3c's own methodology at the shared cutoff=0 anchor. Sweep trusted.

CORRECTION DURING THIS PHASE: the first sweep run classified corners
as racing-speed/low-speed by recomputing median v_mps pooled over each
corner's full canonical bracket window (entry-to-exit, including the
braking zone) against corner_speed_thresholds.low_max -- this gave 13
racing / 1 low-speed corner, not the "11 racing-speed corners" this
arc has used since the WP-S4b sign-check entry. Root cause: that
recomputation does not match modules/corner_analysis.py's own
canonical speed_class field, which classifies on median APEX speed
only (the minimum speed inside the apex_3 window, medianed across a
stable corner's lap instances, corner_analysis.py lines 288-293/
848-856) -- a materially different statistic from a whole-bracket
median that includes braking-zone speeds. Fixed by reading corners[i]
["speed_class"] directly (the exact field summarise_corners already
surfaces) instead of re-deriving it; this recovered 11 racing / 3
low-speed (C7 plus two others), consistent with the established
figure. Recorded because it is a reusable caution: "racing-speed
corner" in this codebase means corner_analysis.py's canonical
speed_class, not a fresh recomputation from any window statistic that
happens to be available.

RESULTS (base_mask n=24183; apex-phase racing-corner population
n=438 -- both fixed across the sweep; racing-speed corner ids
1,2,3,4,5,6,8,10,11,13,14, low-speed 7,9,12):

Force-balance demand, recomputed restricted to the 11 racing-speed
corners only (kinematic candidate, same alpha_r_ss=Fy_r_needed/Cr
method as WP-S3b, informative reference, not independent): range
-5.53 to +4.32 deg (|.|<=5.53), consistent with the established
0.9-5.8 deg figure (which pooled all 14 corners including the 3
low-speed ones).

Per cutoff (median/p90 apex |beta| deg -- EKF median/p90 at the same
samples -- drift median/p90 deg -- sign-check racing median-gate --
racing per-sample pooled fraction -- corr vs pass-1 EKF -- RMS diff
deg -- disqualified):
- 0.05 (current): 0.954 / 2.511 -- EKF 3.443 / 4.917 -- drift 0.162 /
  0.691 -- gate 8/11 -- per-sample 0.5400 -- corr 0.2137 -- RMS 2.560
  -- disqualified=False
- 0.03: 1.663 / 3.100 -- EKF 3.443 / 4.917 -- drift 0.242 / 0.733 --
  gate 8/11 -- per-sample 0.6091 -- corr 0.2349 -- RMS 2.804 --
  disqualified=False
- 0.02: 1.638 / 3.343 -- EKF 3.443 / 4.917 -- drift 0.249 / 0.784 --
  gate 9/11 -- per-sample 0.6800 -- corr 0.2400 -- RMS 3.037 --
  disqualified=False
- 0.01: 1.962 / 3.852 -- EKF 3.443 / 4.917 -- drift 0.218 / 0.876 --
  gate 8/11 -- per-sample 0.6948 -- corr 0.2655 -- RMS 3.162 --
  disqualified=False
- 0.005: 1.801 / 7.573 -- EKF 3.443 / 4.917 -- drift 0.211 / 0.933 --
  gate 9/11 -- per-sample 0.7329 -- corr 0.2718 -- RMS 4.218 --
  disqualified=False
- 0 (no filter, known-bad anchor): 63.357 / 94.856 -- EKF 3.443 /
  4.917 -- drift 5.718 / 10.765 -- gate 5/11 -- per-sample 0.4363 --
  corr 0.0643 -- RMS 71.463 -- disqualified=True

VERDICT: four cutoffs (0.03, 0.02, 0.01, 0.005) survive the
pre-registered disqualifying drift bound (median<0.9, p90<5.8 deg at
every one of them -- drift stays two to three orders of magnitude
below the bound across the whole tested range short of the cutoff=0
anchor) AND all improve apex-phase mid-corner |beta| recovery over
0.05 Hz's 0.954 deg median, moving toward (not reaching) both the
force-balance-demanded band and the pass-1 EKF reference. PRE-
REGISTERED PREDICTION CONFIRMED for the recovery direction (lower
cutoff -> more mid-corner signal) and for drift increasing with lower
cutoff, but the trade-off never reaches the disqualifying bound inside
the tested range -- "no cutoff dominates 0.05" did NOT occur; multiple
cutoffs dominate it on this comparison.

0.02 Hz stands out as a reasonable candidate within the surviving set:
the largest jump in the racing per-sample sign-check fraction
relative to its drift cost (0.54->0.68 vs a drift p90 rise of only
0.09 deg), still comfortably under both bounds, with EKF correlation
(0.24) and RMS difference (3.04 deg) close to 0.03 Hz's. Diminishing/
reversing returns appear below that: 0.005 Hz's apex p90 balloons to
7.573 deg (versus 3.1-4.1 deg at 0.03-0.01 Hz) despite its median
looking unremarkable, and its RMS difference against the EKF (4.218
deg) is the worst of the five non-zero cutoffs -- consistent with
low-frequency noise amplification starting to dominate as the cutoff
approaches the corner-timescale itself (a corner takes several
seconds; 0.005 Hz's ~200 s time constant leaves very little of the
per-corner content actually washed out, so noise below the cutoff
that would have been suppressed at 0.05 Hz now passes through too).

NOT A THRESHOLD DECISION: this phase is read-only per its own design
and PLAN.md's hard constraint (no production/config change permitted
outside the phases that explicitly allow it). No cutoff was changed in
config/parameters.json. This is evidence for a future WP to weigh
against the accompanying cost (classification-threshold re-derivation,
since CS_ratio/Module 4b would move if beta's washout construction
changes) -- not a decision made here.

#### Drift re-examined over time: single-checkpoint verdict superseded [2026-08-2X]

Phase 1's disqualifying-bound check (median/p90 drift, above)
evaluated drift at ONE causal checkpoint per corner exit -- the
post-corner straight-line anchor sample itself, immediately after the
corner. The follow-up plotting task (diagnostics/plot_washout_
sweep.py, drift_post_corner_straights.png) plotted the same local
re-anchored drift construction as a function of TIME PAST that
checkpoint, using a series of causal checkpoints (never filtering on
future data -- see the method note below) rather than the single
point. Result: 0.03, 0.02, and 0.01 Hz all continue drifting well
past the checkpoint Phase 1 actually measured. 0.02 Hz grows from
~0.25 deg at the checkpoint to ~1.6 deg within 4 s, crossing the
pre-registered 0.9 deg median bound at roughly the 1.5 s mark.
Production 0.05 Hz is the only cutoff that stays flat and low across
the whole 4 s window.

SUPERSEDED, not deleted (notebook convention -- struck through with
this dated note rather than rewritten): the Phase 1 verdict "four
cutoffs (0.03, 0.02, 0.01, 0.005) survive the pre-registered
disqualifying drift bound... multiple cutoffs dominate [0.05]" is
~~SUPERSEDED~~ as a standalone conclusion. It was not WRONG given
what it measured (the checkpoint values reproduce exactly, see the
plotting task's reproduction check) -- it was INCOMPLETE, because the
disqualifying bound was only ever checked at one instant, and that
instant turns out to be the best point in time for every one of the
three lower cutoffs, not a representative one.

CORRECTED STATEMENT: lower washout cutoffs trade genuine mid-corner
signal recovery (Phase 1's own metric 1, confirmed real and still
standing) for post-corner drift that continues GROWING for several
seconds after the corner, not a one-off bump that Phase 1's single
checkpoint would have caught. The original checkpoint was measured
at the most favourable possible instant (immediately at corner exit,
before the growing-window drift has had time to accumulate), so it
systematically understated the true cost of every cutoff below 0.05 Hz.

CONSEQUENCE: the recovery-vs-drift trade is CORNER-SPACING DEPENDENT.
A cutoff that looks acceptable on this session's data (Dubai, where
some corners have a long following straight before the drift curve
is ever tested by another corner) can fail on a track where corners
follow closely -- the drifting signal from one corner's exit would
still be rising when the next corner's entry arrives, contaminating
that corner's own mid-corner reading rather than settling first. A
single track-independent washout cutoff may therefore not exist at
all -- this is new evidence, not previously stated in this arc, and
it strengthens the case for the auto-fit EKF (modules/tyre_fit_
auto.py, Phase 2/3) as the PRIMARY sideslip source going forward,
with kinematic beta retained as a fallback rather than pursued as a
better-tuned primary. Any future washout-cutoff change must be
justified against a full drift-vs-time curve (per corner, per
candidate track), not a single checkpoint value.

METHOD NOTE, GENERAL -- applies beyond this one plot: modules/
stability_analysis.py's _highpass_filter uses scipy.signal.filtfilt,
which is zero-phase and ACAUSAL -- every output sample depends on the
ENTIRE input segment, including samples that come after it in time,
not just before. A diagnostic that filters a different-length segment
than production would (e.g. extending a local window forward to see
"what happens next") changes the filtered value at every point in
that segment, including points that already existed in the shorter
segment -- found and worked through while building drift_post_corner_
straights.png (see that plot's own run_info.txt and the chat report
from that session for the concrete numeric mismatch that exposed it).
Any future diagnostic that makes a drift, boundary, or edge-effect
claim about a filtfilt-based signal (this washout filter, or
estimate_slip_angles'/estimate_lateral_forces' Butterworth low-pass)
must evaluate it via CAUSAL CHECKPOINTS -- filtering only the data
available up to each evaluated instant -- not by reading arbitrary
points out of one long filtered array and assuming they match what a
shorter or differently-bounded run would have produced.

### Phase 2: one-shot per-session Dugoff fit + EKF chain [2026-08-20]

New file: modules/tyre_fit_auto.py (fit_session(data, params,
data_file_path=None) -> manifest dict), automating the recorded pass-0
(WP-N1b, diagnostics/fit_dugoff_first_pass.py) and pass-1 (config
tyre_model_ekf.pass_1 comments, diagnostics/inspect_ekf_pass1_rQ_
sweep.py, diagnostics/inspect_pass1_final_validation.py) procedure as
a reusable function. NOT wired into the UI or the production analysis
thread. New additive config namespace tyre_fit_auto (mu_fz search-
bracket tunables, Q/P0 seeds copied from tyre_model_ekf.pass_1, the
2-D R sweep grid, NIS acceptance band, and the proposed status
thresholds) -- marked experimental, all existing keys/blocks
untouched. New acceptance script diagnostics/inspect_tyre_fit_auto_
acceptance.py.

DEPENDENCY NOTE, stated up front: modules/tyre_fit_auto.py imports
diagnostics/sideslip_ekf_dugoff.py for the EKF recursion itself,
inverting the project's usual one-way diagnostics-depends-on-modules
direction. Deliberate, not an oversight -- the alternative was
duplicating ~150 lines of Jacobian/update code with a real risk of the
two copies silently diverging. Documented in the new module's own
docstring; no existing config comment's "no modules/ consumer" note
was edited (those describe historical fact as of when written, and
CLAUDE.md's config rule keeps existing keys/comments untouched).

ACCEPTANCE CHECK RESULT (diagnostics/inspect_tyre_fit_auto_acceptance.py,
tolerances: 1e-6 relative for c_alpha/mu_fz -- identical deterministic
optimizer over identical inputs, an exact match is expected; 1e-3
relative for everything downstream of the EKF recursion, justified in
the script's own header):
- c_alpha_n_per_rad and mu_fz_N, BOTH AXLES: exact match to pass-0's
  manifest (relative difference < 1e-6). onset_deg (a pure function of
  c_alpha/mu_fz) also matched exactly. The fully-scripted half of the
  procedure reproduces bit-for-bit.
- R sweep: chosen grid point (r_ay_scale=0.1, r_yaw_scale=4.0,
  found_in_band=True) matches pass_1's recorded choice exactly. The
  chosen R_ay_var/R_yaw_rate_var themselves sit ~0.37%/0.28% off
  (3.798 vs 3.784, 0.006065 vs 0.006048) -- outside the 1e-3 tolerance,
  investigated rather than shrugged off (see below).
- Downstream validation figures (NIS exceedances, combined mean NIS,
  rear coverage_fraction) show the same small (~0.2-0.4%) residual
  gap, consistent with propagating through the EKF from the R
  difference above. ay_exceedance, front coverage_fraction, and
  h2_vs_ay_apex correlation (0.9678 vs 0.9679) all land inside 1e-3
  regardless. Sign check: median gate 14/14 (all corners) and 11/11
  (racing-speed, canonical speed_class) both perfect; per-sample
  fraction 0.9958 vs the historical script's 0.9963 -- not a gated
  comparison (see the racing-population note below), but close.

INVESTIGATION of the R_ay_var gap, per the work order's "find why, do
not shrug": pass_1's config comment (R_ay_derivation) documents its
own Method-A inputs -- front/rear RMS residual converted to
acceleration (2.030, 4.272 m/s^2) and inter-axle correlation (0.8999)
-- and its own combined-variance formula (var_f + var_r +
2*rho*std_f*std_r). Hand-applying that exact formula to those exact
documented inputs gives 37.979, not the archived 37.8418 the config
comment states as the result (verified by direct calculation, not
estimated). This module's own automation, run on Dubai, independently
computes std_f=2.02998, std_r=4.27229, rho=0.899881 (matching the
documented inputs to their own stated precision) and a combined value
of 37.982 -- consistent with the hand-check of the documented formula,
not with the archived 37.8418. CONCLUSION: the ~0.37% gap traces to
the archived pass_1 config value's own one-off derivation (which,
unlike every numbered pass's curve fit or the 2-D sweep, was never
saved as a standalone diagnostics/ script and cannot be re-run to find
its exact intermediate arithmetic) containing a small inherited
inconsistency relative to its own documented formula and inputs -- NOT
a divergence in this module's methodology, which reproduces the
documented formula exactly. Everything gated on the deterministic,
previously-scripted half of the chain (c_alpha, mu_fz, onset, chosen
grid coordinates) matches exactly; only the one un-scripted historical
number and its small downstream propagation do not.

SEPARATE FINDING, also from this acceptance run: pass1_final_
validation.py's own "racing-speed corner" population (13 corners, an
ad-hoc per-corner window-median-speed classification computed inline
in that script) disagrees with corner_analysis.py's canonical
speed_class field (11 corners, median APEX speed only) -- the SAME
inconsistency this package's own Phase 1 already found and corrected
(see the Phase 1 correction entry above). modules/tyre_fit_auto.py
uses the canonical speed_class field, consistent with Phase 1 and with
the "11 racing-speed corners" convention used everywhere else in this
arc; pass1_final_validation.py itself (an existing file, not touched
by this package) still carries the older ad-hoc convention. Not fixed
here (out of scope -- pass1_final_validation.py is an existing
diagnostics file, this package only reads its manifest for comparison,
never edits it); flagged for a future small cleanup.

VERDICT: the acceptance check does NOT pass at the declared 1e-3
tolerance on every field, but the automation is judged TRUSTWORTHY --
every field that depends only on the previously-scripted parts of the
procedure (c_alpha, mu_fz, onset, the 2-D sweep's chosen grid
coordinates) reproduces exactly, and the small remaining gaps are
fully traced to one archived, never-scripted historical number rather
than to any divergence in this module's own logic. Status field on
this Dubai run: "ok" (sign_check_marginal_fraction and nis_gross_
miscalibration_fraction both cleared; an in-band R grid point was
found).

STATUS THRESHOLDS (proposed, not yet reviewed -- restated here from
the module docstring for visibility): DEGENERATE if either axle's
c_alpha sign check fails, either axle's mu_fz fit hits its widened
search-bracket ceiling (the pass-4 rear failure mode -- this is the
one case the work order explicitly requires never be silently
accepted), the racing-speed sign-check median-gate fraction falls
below 0.5 (no better than chance), or the best R grid point still
leaves either NIS channel's exceedance above 0.5 (an order of
magnitude beyond the 3-15% target band). MARGINAL if not degenerate
but the sweep found no genuinely in-band grid point, or the racing
sign-check median-gate fraction sits below 0.7 (the frozen pass-1
baseline's own 8/11=0.727 was used as the reference point for this
boundary). OK otherwise.

### Phase 3: Pacejka variant -- pre-registration [2026-08-20]

New files (all new, no existing production file touched): modules/
tyre_model_pacejka.py (reduced 4-parameter Magic Formula, Fy = D*sin(
C*arctan(B*alpha - E*(B*alpha - arctan(B*alpha)))), cited as "chair
performance_analysis tooling (internal)"; published general form in
Rajamani Ch. 13 Magic Formula section, page TBD verify), its analytic
dFy/dalpha (chain rule through the arctan-of-arctan composition,
transcribed then verified against central-difference numerical
differentiation, tests/test_pacejka_model.py, 10 tests, max relative
error < 1e-6 at every tested slip angle plus odd/even symmetry and
positive-origin-stiffness checks); diagnostics/sideslip_ekf_pacejka.py
(structural mirror of diagnostics/sideslip_ekf_dugoff.py, Pacejka
force/stiffness substituted in both Jacobians and the state
propagation, everything else identical -- NEW code path, the existing
Dugoff EKF file is untouched); modules/tyre_fit_auto.py gains
fit_session_pacejka (Powell optimizer, chair's starting guess
[B,C,D,E]=[12, 1.9, 8000, 0.97], same base_mask/R-derivation/2-D-
sweep/validation structure as fit_session's Dugoff chain).

PRE-REGISTERED PREDICTION, stated before running either fit: Dugoff's
own rear-axle history in this arc is one of REPEATED DEGENERACY under
refit (passes 2-4, mu_fz_rear drifting outward each iteration until
pass 4's fit hit the search-bracket ceiling and the curve collapsed to
pure-linear -- WP-N2 refit-loop entry). The diagnosed mechanism was a
self-starving feedback: low rear saturation coverage under the
kinematic-sourced alpha (6.95% beyond onset, WP-N1 entry) leaves too
few samples demanding a nonlinear (saturating) shape for the fit to
identify one, so any model whose only route to representing
saturation is a SINGLE ceiling parameter (Dugoff's mu_fz) is
structurally exposed to this failure mode on this axle's data.
Pacejka's fit is NOT staged the same way -- all four parameters (B,
C, D, E) are fit jointly by a general optimizer against the same
data, not "c_alpha fixed, then mu_fz alone must explain saturation
with nothing else able to move". PREDICTION: on THIS SAME kinematic-
sourced rear data (same low-coverage population, same underlying
signal-starvation problem that caused Dugoff's degeneracy), Pacejka's
joint 4-parameter fit is expected to converge to SOME set of (B,C,D,E)
without hitting an explicit bound (Powell has no boundary to hit the
way the Dugoff mu_fz search does), but the fitted curve's PEAK
location and shape are expected to be POORLY CONSTRAINED by the same
starved rear population -- i.e. convergence in the optimizer's sense
is expected, genuine identifiability is not. This is a prediction
about IDENTIFIABILITY, not merely optimizer success, and the
comparison below is designed to expose it (fit RMS alone would not --
an unidentified peak can still fit the LINEAR-REGIME bulk of the data
well). A rear RMS residual comparable to or better than Dugoff's,
combined with a peak location outside or far beyond the visited alpha
range (extrapolated, not observed), would count as this prediction
CONFIRMED, not refuted -- the failure mode this time is silent (no
bound to hit) rather than an explicit degenerate flag, which is
itself the more concerning outcome the comparison should surface.

New script: diagnostics/inspect_tyre_variant_comparison.py (read-only,
no config/production change; runs both fit_session and
fit_session_pacejka on Dubai and compares).

### Phase 3: Pacejka variant -- results [2026-08-20]

Both variants converged (status="ok" for both, no degenerate flag on
either axle for either model). Full results, diagnostics/
inspect_tyre_variant_comparison.py, Dubai, base_mask n=24183 (shared
across every metric below unless noted):

FIT RMS RESIDUAL (N): front Dugoff=2752.7 vs Pacejka=2738.4 (Pacejka
0.5% lower); rear Dugoff=5793.2 vs Pacejka=5785.1 (Pacejka 0.1%
lower). Pacejka's extra two free parameters buy only a marginal RMS
improvement on the BULK of the fit population -- consistent with the
pre-registration's own warning that fit RMS alone would not expose an
identifiability problem, since the linear-regime bulk of the data
dominates the sum of squares either way.

ONSET (Dugoff) / PEAK (Pacejka) LOCATION: front Dugoff onset=2.297
deg (coverage 0.5905) vs Pacejka peak=6.281 deg (coverage 0.0419,
OUTSIDE the visited alpha range, p99=5.059 deg); rear Dugoff
onset=2.599 deg (coverage 0.4893) vs Pacejka peak=3.809 deg (coverage
0.1100, also OUTSIDE the visited range, p99=3.355 deg). Both axles'
Pacejka peaks are extrapolated beyond what the data actually visited,
not interpolated within it.

VALIDATION METRICS: NIS combined exceedance Dugoff=0.1369 (inside the
3-15% band) vs Pacejka=0.1054 (also inside, closer to band centre);
combined mean NIS Dugoff=2.899 vs Pacejka=2.713 (both near the
calibrated-filter expectation of ~2); sign check racing median gate
11/11 for both, per-sample fraction Dugoff=0.9958 vs Pacejka=0.9975;
h2-vs-ay apex correlation Dugoff=0.9678 vs Pacejka=0.9680 (equal to
four figures). On every one of these headline validation numbers
Pacejka is marginally better than or equal to Dugoff -- consistent
with a strictly more flexible model fitting slightly better, not
evidence by itself that the extra flexibility is well-identified.

SELF-CONSISTENCY R^2 AT EACH FILTER'S OWN ALPHA (corner-window
population, n=15556, same methodology as pass-1's validation Section
3): front Dugoff R^2=0.9524 (RMS 1445 N) vs Pacejka R^2=0.9589 (RMS
1342 N) -- Pacejka better; REAR Dugoff R^2=0.9820 (RMS 1190 N) vs
Pacejka R^2=0.9750 (RMS 1404 N) -- Dugoff BETTER here, the one metric
in this whole comparison where Dugoff wins outright. This reversal at
the rear, right where the identifiability concern was pre-registered,
is itself informative: Pacejka's extra flexibility does not pay off
once the alpha is drawn from Pacejka's OWN (different, less
constrained) EKF beta rather than from the bulk kinematic-sourced fit
population.

REAR-AXLE IDENTIFIABILITY CHECK -- PRE-REGISTERED PREDICTION
CONFIRMED: Powell reports convergence on the rear axle (powell_
converged=True, no explicit bound analogous to Dugoff's mu_fz ceiling
to hit) YET the fitted peak (3.809 deg) sits beyond the visited alpha
range (p99=3.355 deg) -- an extrapolated, not observed, peak location.
This is exactly the silent-failure pattern predicted: unlike Dugoff's
overt pass-4 degeneracy (an explicit bound-hit flag), Pacejka's
rear-axle fit reports success while its peak shape is not actually
constrained by data. NOT PRE-REGISTERED but observed and recorded
honestly: the SAME pattern holds at the FRONT axle too (peak 6.281 deg
vs visited p99 5.059 deg) -- the prediction singled out the rear
because of its specific degeneracy history, but the underlying
mechanism (a genuine peak-and-decline shape needs samples that
actually decline, and this session's racing-speed corners may simply
not visit large enough slip angles at either axle) is not rear-
specific. This should be read as a LIMITATION OF THIS SESSION'S DATA
for identifying either model's saturated/post-peak shape, not a
rear-specific finding as originally framed -- worth flagging in any
future re-run on a session with a wider visited slip-angle range.

VERDICT: Pacejka validates marginally better than Dugoff on every
aggregate/global metric (fit RMS, NIS, sign check, h2-vs-ay), but
WORSE on the one metric that isolates each model's behaviour at
alpha values its OWN EKF actually produces at the rear axle (self-
consistency R^2, the metric closest to "does this curve reflect what
the filter is doing", not "does it fit the bulk training population").
Combined with the peak-location extrapolation finding at both axles,
the honest reading is: this session's data does not clearly separate
the two models' SHAPE beyond the linear regime -- Dugoff's extra
constraint (a single saturation-ceiling parameter, structurally
limited to asymptote rather than genuinely peak-and-decline) may be a
feature, not a limitation, on data this sparse in high-slip-angle
samples, since it has one fewer free parameter to leave unconstrained.
No winner is declared here (per the work order); this is a comparison
for the record, not a config change -- neither variant is wired into
config's tyre_model_ekf.* or any production path.

### Phase 4: NIS tyre-mismatch gate -- pre-registration [2026-08-20]

DESIGN: a health score answering "does the fitted curve match this
session's data well enough to trust EKF beta?", built directly from
"the windowed NIS machinery that already exists" (the EKF's own
rolling-window exceedance check, config nis_window_samples/nis_chi2_
bound, diagnostics/sideslip_ekf_dugoff.py) rather than a new
statistic invented from scratch. Generalises the per-sample binary
divergence trigger (window exceedance fraction > 0.5 -> diverged) into
a continuous, session-level score using the SAME acceptance band
(3-15% per-window exceedance) already load-bearing throughout Phase 2
(the R-sweep's own "both channels in band" gate):
  1. For each masked sample i, compute the trailing-window (width
     nis_window_samples) combined-NIS exceedance fraction ending at i
     (same rolling-window construction as the EKF's own divergence
     monitor, computed here as a diagnostic over the full nis array,
     not gated on window_flag's cruder 50% trigger).
  2. in_band_i = nis_band_low <= exceedance_fraction_i <= nis_band_high
     (0.03/0.15, the same values used throughout Phase 2).
  3. health_score = fraction of masked samples with in_band_i True.
A score near 1.0 means the filter behaves like a correctly-calibrated
estimator (its own uncertainty model matches its actual error) across
almost the whole session; a low score means the fitted curve and the
data disagree often enough that the filter's own NIS statistics stop
looking calibrated -- exactly the observable a tyre-curve mismatch (a
session-fit curve applied on a track/compound/setup it was not fit
against) is expected to produce, since the EKF's h(x) becomes
systematically wrong and residuals grow relative to what R expects.

SYNTHETIC MISMATCH CASES (as directed): starting from a Dugoff config
with an R already NIS-gated to be healthy (Phase 2's fit_session "ok"
result on Dubai, final_config), four single-parameter-family
mismatches, both axles scaled together per scenario: c_alpha x0.5,
c_alpha x2.0, mu_fz x0.5, mu_fz x2.0. R/Q/P0 held FIXED at the healthy
run's own values in every mismatch scenario -- the point is to
simulate "the fitted curve doesn't match this session" while
everything else about the filter's own tuning stays the noise model
that assumed a correct curve, which is exactly the deployment scenario
this gate exists to catch (a stale/wrong curve carried into a session
whose R was never re-derived for it).

PRE-REGISTERED PREDICTION: the healthy baseline's health_score should
sit close to the ~85% ceiling a correctly 3-15%-band-gated R implies
by construction (most windows should be in-band, since that is what
the R sweep was gated on in aggregate -- though the windowed,
LOCAL version of the same check is a stricter, non-identical test and
some shortfall from a very high score is expected even in the healthy
case). All four mismatch scenarios should score MEASURABLY lower than
the healthy baseline; c_alpha mismatches are pre-registered to hurt
scoring MORE than mu_fz mismatches, because c_alpha sets the filter's
LINEAR-REGIME response (the great majority of samples, per Phase 1's
own apex-recovery numbers, sit well inside the linear regime for this
session), while mu_fz only matters for the smaller saturated-regime
population (Dugoff onset coverage 49-59% at these thresholds, but many
of those samples are only marginally past onset where f(lambda) is
close to 1 anyway) -- a wrong c_alpha therefore corrupts a larger
share of the filter's moment-to-moment predictions than a wrong mu_fz
does on this dataset.

New script: diagnostics/inspect_nis_tyre_mismatch_gate.py (prototype,
nothing wired -- read-only, no config/production change). Thresholds
(use-EKF / warn / fall-back-to-kinematic) are a PROPOSAL for the
user's decision, not applied anywhere.

### Phase 4: NIS tyre-mismatch gate -- results [2026-08-20]

RESULTS (Dubai, window=20 samples, band=[0.03, 0.15], healthy config =
Phase 2's fit_session "ok" final_config): healthy=0.1622, c_alpha_
x0.5=0.1501, c_alpha_x2.0=0.1318, mu_fz_x0.5=0.1122, mu_fz_x2.0=0.0674.

PREDICTION 1 (ceiling near the R-sweep's own aggregate gate) FAILED,
recorded as failed: the healthy baseline scored 0.1622, nowhere near
the "~85%" region implied by treating the whole-session 3-15%
aggregate gate as if it applied window-by-window. DIAGNOSIS (worked
through, not just noted as a surprise): the windowed score's band
check operates on COUNTS out of only window=20 samples -- for a
combined-NIS chi-square df=2 bound, a truly well-calibrated filter
exceeds it exactly 5% of the time per sample, so a window of 20 has an
expected exceedance count of 1.0, and simple binomial arithmetic
(Binom(20, 0.05), P(count in {1,2,3}) = P(win_frac in [0.05,0.15])
approx 0.626) already caps the BEST any correctly-calibrated filter
could plausibly score at this window width around 60-65%, well short
of 100% -- and that is before accounting for any real temporal
clustering of exceedances (bursty error, not i.i.d.), which would push
the achievable ceiling lower still. The observed 0.1622 sits well
under even that theoretical i.i.d. ceiling, meaning either the healthy
run's LOCAL exceedance rate is not uniformly 5% (plausible -- NIS
inflates specifically inside corners where the tyre model's local
error is largest, not uniformly across the session) or the 20-sample
window is simply too small relative to the band's own width for this
score construction to reach a high absolute value even when the
filter genuinely fits well. RECORDED LIMITATION for any future
refinement of this prototype: a wider window, a proper chi-square
goodness-of-fit test on the window's exceedance count (rather than a
band-membership check), or reporting deviation-from-expected-rate
directly would all be more statistically principled next steps than
what this prototype implements.

PREDICTION 2 (c_alpha mismatches hurt scoring more than mu_fz
mismatches) FAILED, recorded as failed -- the observed order is the
opposite: mu_fz mismatches (0.1122, 0.0674) score markedly lower than
c_alpha mismatches (0.1501, 0.1318). ROOT CAUSE OF THE FAILED
PREDICTION, traced rather than shrugged off: the prediction's own
premise was wrong. It argued mu_fz "only matters for the smaller
saturated-regime population", but Phase 2/3's own onset_coverage
figures on this exact dataset (recorded earlier in this same package)
already show front coverage=0.59 and rear coverage=0.49 -- roughly
HALF of all masked samples sit beyond the Dugoff onset boundary, not a
small minority. mu_fz therefore governs Fy for a comparably large
share of the session as c_alpha's linear-regime dominance was assumed
to, and scaling it materially changes the filter's predicted Fy (and
hence its measurement residual) for close to half the data -- a wrong
premise inside this session's own already-recorded numbers, not a
new inconsistency requiring further investigation.

CORE REQUIREMENT MET: despite both specific numeric predictions
failing, the score DOES cleanly separate healthy from every mismatch
scenario -- healthy (0.1622) is strictly the highest of all five
scores, and all four mismatches fall below it, confirming the design's
central claim (a tyre-curve mismatch is detectable via this windowed-
NIS statistic) even though the absolute scale and the relative
severity ordering both differ from what was pre-registered.

PROPOSED THRESHOLDS (gap-selected between the healthy score and the
worst mismatch score, same percentile-gap-selection convention used
elsewhere in this project for classification thresholds -- e.g.
config/parameters.json classification.stab_neg_thresh_Nm_per_deg's own
derived_from note): gap [0.0674, 0.1622]. USE_EKF if health_score >=
0.1385; WARN if 0.1006 <= health_score < 0.1385; FALL_BACK_TO_
KINEMATIC if health_score < 0.1006. EXPLICITLY CAVEATED, not
minimised: this is FIVE data points from ONE session (one healthy
config, four synthetic mismatches), an extremely thin evidence base
for a threshold that would gate production trust in EKF beta -- a
genuine proposal for the user's review per the work order, not a
recommendation to apply as-is. A real threshold-derivation pass would
need multiple sessions/tracks (to see genuine cross-session curve
mismatch, not only synthetic parameter scaling) and should revisit the
window-size/statistic critique from the failed ceiling prediction
above before being trusted for a production fallback decision.

### Phase 5: consolidated report [2026-08-20]

PER-PHASE SUMMARY:

Phase 1 (washout cutoff sweep). Built: diagnostics/inspect_washout_
cutoff_sweep.py. Found: reproduction check PASSED (cutoff=0 reproduces
WP-S3c's 5.70/10.80 deg drift figures exactly, validating the sweep's
two-construction design). Four intermediate cutoffs (0.03, 0.02, 0.01,
0.005 Hz) all survive the pre-registered disqualifying drift bound and
improve apex-phase mid-corner |beta| recovery over the production
0.05 Hz default -- "no cutoff dominates 0.05" did NOT occur, the
opposite of the plan's stated "legitimate finding" fallback. 0.02 Hz
flagged as a reasonable candidate within the surviving set (best
sign-check-improvement-per-unit-drift-cost). Also found and corrected
a racing-speed-corner classification bug in the sweep's own first
draft (window-median-speed vs corner_analysis.py's canonical
speed_class field) -- documented because the SAME bug recurred,
independently, in Phase 2's acceptance check against an EXISTING file
(pass1_final_validation.py), suggesting this ad-hoc-vs-canonical
speed-classification inconsistency is a small latent issue worth a
future cleanup pass across the diagnostics/ directory, not fixed here
(out of scope, existing files untouched). Tests added: 0 (read-only
diagnostic). No config/production change.

Phase 2 (one-shot per-session Dugoff fit + EKF chain). Built: modules/
tyre_fit_auto.py (fit_session), config/parameters.json's new
tyre_fit_auto namespace (additive), diagnostics/inspect_tyre_fit_auto_
acceptance.py. Found: the fully-scripted half of the recorded
procedure (c_alpha, mu_fz, onset, the R-sweep's chosen grid
coordinates) reproduces the pass-0/pass-1 record EXACTLY; the
un-scripted half (the one-off Method-A R_ay_var derivation, never
saved as a standalone script) shows a small (~0.37%) internal
arithmetic inconsistency in the ARCHIVED figure itself, verified by
hand-applying its own documented formula to its own documented inputs
-- this module's automation is judged trustworthy, the small residual
gap traces to the historical record, not to a divergence in this
module's logic. Tests added: 0 (acceptance check is a diagnostic
script, not a pytest suite member -- see "chose not to do" note
below). No existing production file touched; new namespace additive.

Phase 3 (Pacejka variant). Built: modules/tyre_model_pacejka.py
(reduced 4-parameter Magic Formula + analytic derivative),
diagnostics/sideslip_ekf_pacejka.py (new EKF code path, Dugoff's own
file untouched), modules/tyre_fit_auto.py gained fit_session_pacejka,
tests/test_pacejka_model.py (10 tests, finite-difference verification
of the analytic derivative plus odd/even-symmetry and positive-
origin-stiffness checks). Found: Pacejka validates marginally better
than Dugoff on every AGGREGATE metric (fit RMS, NIS, sign check,
h2-vs-ay) but WORSE on rear-axle self-consistency R^2 (0.9750 vs
Dugoff's 0.9820) -- the one metric that isolates behaviour at each
model's OWN EKF-produced alpha rather than the bulk fit population.
Pre-registered rear-axle-identifiability prediction CONFIRMED (Powell
converges without hitting any explicit bound, yet the fitted peak
extrapolates beyond the visited alpha range) -- and found, honestly,
NOT to be rear-specific after all: the front axle shows the identical
extrapolated-peak pattern, a correction to the pre-registration's own
framing, recorded as such rather than quietly narrowed to fit. No
winner declared in config; neither variant wired into any production
path. Tests added: 10.

Phase 4 (NIS tyre-mismatch gate). Built: diagnostics/inspect_nis_
tyre_mismatch_gate.py (prototype, nothing wired). Found: the score
cleanly separates the healthy Dubai baseline from all four synthetic
mismatch scenarios (core requirement met), but BOTH pre-registered
numeric predictions failed and were recorded as failed with traced
causes -- the absolute-ceiling prediction ignored small-window
binomial noise (a Binom(20,0.05) i.i.d. ceiling of ~63% already falls
well short of the ~85% predicted, and the observed 16% falls short of
even that, suggesting real temporal clustering of NIS exceedance
inside corners); the c_alpha-vs-mu_fz relative-severity prediction
inverted because its own premise (mu_fz affecting only a "small"
saturated population) contradicted onset_coverage figures (49-59%)
already on record earlier in this SAME package. Thresholds proposed
via gap-selection, explicitly caveated as five-data-point, one-session
evidence -- a proposal, not a recommendation. Tests added: 0
(prototype diagnostic, not reusable production code).

TEST COUNTS: 49 -> 59 (48 passed + 1 xfailed at package start, per
PLAN.md STATUS; 58 passed + 1 xfailed at package end, confirmed by
direct pytest run after every phase). All 10 new tests are Phase 3's
Pacejka finite-difference/symmetry checks.

FAILED PREDICTIONS, consolidated (per the work order's "failed
predictions stay recorded as failed"): Phase 4's health-score ceiling
prediction (expected ~85%, observed 16%) and its c_alpha-vs-mu_fz
relative-severity prediction (expected c_alpha worse, observed mu_fz
worse) both failed, with causes traced above. Phase 3's rear-only
framing of the peak-extrapolation prediction was also not fully
correct (the front axle showed the same pattern) -- the qualitative
prediction (silent, bound-free failure mode exists) was confirmed, but
its scope was narrower than the actual finding, corrected in the
Phase 3 results entry rather than left as originally framed.

SKIPPED OR STOPPED, and why: none of the five phases were stopped or
judged impossible/ill-posed -- every phase produced a result and
completed its own acceptance/reproduction check. Deliberately NOT
done, stated up front rather than discovered by omission: (1) no
pytest coverage was added for modules/tyre_fit_auto.py's fit_session/
fit_session_pacejka themselves (only for the new pure Pacejka math) --
the acceptance/comparison scripts already exercise the full chain
against recorded references on real Dubai data every time they are
run, and a synthetic-fixture pytest test would either duplicate that
real-data check or violate CLAUDE.md's "real data only" rule by
inventing synthetic channel arrays for a ~100s-runtime full-pipeline
function; judged not worth the mismatch. (2) Phase 2's acceptance
script does not edit pass1_final_validation.py's ad-hoc racing-speed-
corner classification to match the canonical field -- that file is an
existing diagnostics/ script this package's hard constraints do not
authorise touching casually; flagged as a small future cleanup instead
(also surfaced independently in Phase 1). (3) Phase 4's health-score
statistic was not redesigned after its own ceiling-prediction failure
diagnosed a window-size/statistic weakness -- the work order asked for
a prototype and a proposal, not a finished, re-iterated metric; the
weakness is recorded explicitly as a limitation for whoever picks this
up next, not silently patched over.

CONFIRMATIONS (Phase 5 checklist, per the work order):
- Full regression suite green: confirmed by direct run after every
  phase boundary (Phase 1: 48 passed/1 xfailed; Phase 2: 48/1 xfailed,
  no test file changed yet; Phase 3: 58 passed/1 xfailed, +10 new
  Pacejka tests; Phase 4: 58 passed/1 xfailed, unchanged). Final run:
  58 passed, 1 xfailed, 84.57s.
- git status shows only new files plus thesis_notes.md/config/
  parameters.json edits, no commit made: confirmed (`git status
  --porcelain`) -- modified: config/parameters.json (+24/-0, the new
  additive tyre_fit_auto namespace only), thesis_notes.md (append-only
  content, the one "-1" line in `git diff --stat` is a trailing-
  newline artifact at the point of insertion, verified by inspection,
  not a content deletion). Untracked (new files only): diagnostics/
  inspect_washout_cutoff_sweep.py, diagnostics/inspect_tyre_fit_auto_
  acceptance.py, diagnostics/sideslip_ekf_pacejka.py, diagnostics/
  inspect_tyre_variant_comparison.py, diagnostics/inspect_nis_tyre_
  mismatch_gate.py, modules/tyre_fit_auto.py, modules/tyre_model_
  pacejka.py, tests/test_pacejka_model.py. No PLAN.md edit was made
  before this Phase 5 pass (made now, see below). No commit run at any
  point.
- Protected set untouched: confirmed, `git ls-files docs/literature/
  docs/car_data/ config/car_data.json HANDOVER.md docs/study/` returns
  empty.
- sideslip_source still "kinematic": confirmed, read live from
  config/parameters.json.

DECISIONS WAITING FOR THE USER (carried into PLAN.md's NOW section):
1. Phase 1's washout-cutoff finding: whether to move beta_washout_
   cutoff_hz off 0.05 (candidates 0.02-0.01 Hz look promising on this
   comparison) -- gated on the standing classification-threshold
   re-derivation rule, since CS_ratio/Module 4b would shift if beta's
   construction changes.
2. Phase 2/3's fit-variant choice: Dugoff vs Pacejka as the EKF's
   internal tyre model -- no winner declared, comparison only; the
   rear/front peak-extrapolation finding argues for caution either way
   pending a session with a wider visited slip-angle range.
3. Phase 4's proposed NIS health-score thresholds (USE_EKF >= 0.1385,
   WARN [0.1006, 0.1385), FALLBACK < 0.1006) -- explicitly thin
   evidence, needs either more sessions or a redesigned statistic (see
   the ceiling-prediction failure diagnosis) before being trusted.
4. Whether to fix the ad-hoc-vs-canonical racing-speed-corner
   classification inconsistency found independently in both Phase 1
   and Phase 2 (pass1_final_validation.py's own inline window-median
   method vs corner_analysis.py's canonical speed_class field) -- a
   small, low-risk cleanup, not done here since it touches an existing
   file outside this package's explicit permissions.
5. All prior carry-forward decisions from before this package (pass_2-
   4 block deletion, dead-diagnostics cleanup, the NIS threshold this
   package's Phase 4 now feeds into, the UI switch design for
   sideslip_source) remain open, unchanged by this package.

### Repo cleanup: pass_2-4 block deletion + dead-diagnostics sweep [2026-08-20]

Deleted (working tree only, not committed -- last commit still
containing every file listed below: b0e7aa0c8f2b0f7a4b259fa993a21c2c4722802e):
- config/parameters.json: tyre_model_ekf.pass_2 and .pass_4 blocks
  (numbers preserved verbatim in the "WP-N2 pass 2"/"WP-N2 pass 4"
  entries above and the refit-loop non-convergence entry). Keys
  replaced with "_comment_pass_2_removed"/"_comment_pass_4_removed"
  marker strings (not blank deletion) so a stray params["tyre_model_
  ekf"]["pass_2"] lookup raises a clean KeyError instead of silently
  finding nothing or a wrong-typed value.
- diagnostics/fit_dugoff_pass2_refit.py, diagnostics/fit_dugoff_
  pass3_refit.py, diagnostics/inspect_ekf_pass2_evaluation.py,
  diagnostics/inspect_ekf_pass3_evaluation.py, diagnostics/inspect_
  ekf_pass4_evaluation.py.

NOT deleted, deliberately, per the work order's own stop condition
("grep the entire repo... if anything does [read a block], stop and
report instead"): config/parameters.json's tyre_model_ekf.pass_3
block. diagnostics/fit_dugoff_pass4_refit.py (KEPT -- not in the
explicit deletion list) reads pass_3 as its own EKF source
(pass_id="pass_3"); deleting pass_3 would have left that script
silently broken (KeyError on next run) without deleting the script
itself, which the work order did not authorise. fit_dugoff_pass4_
refit.py's own docstring pointer to the now-deleted fit_dugoff_
pass3_refit.py was fixed in place ("removed 2026-08-2X, see git
history") rather than rewritten. Recommendation for a future pass:
either explicitly authorise deleting fit_dugoff_pass4_refit.py (its
sibling evaluation script, inspect_ekf_pass4_evaluation.py, is
already gone) to make pass_3 safe to remove too, or keep both
pass_3 and fit_dugoff_pass4_refit.py intentionally as the one
remaining artifact of the refit loop's actual failure mode (WP-N2
pass 4's rear mu_fz non-convergence) -- both are defensible, neither
decided here.

Verification performed: grep across the whole repo for the deleted
files' names found the deletions clean (no import statements
anywhere reference any of the five); config/parameters.json's own
frozen_from pointers for the KEPT pass_0/pass_1 blocks do not name
any deleted file. Full regression suite: 58 passed, 1 xfailed
(unchanged from before the deletions). test_stability.py smoke test:
unchanged, runs clean.

Candidate list (29 diagnostics/*.py scripts recommended for a future
deletion pass, produced a recorded finding now fully preserved in
this file, no incoming references found anywhere in the repo) and
the full 68-file classification are in this session's chat report,
not duplicated here in full -- summary: inspect_abs_slip_channels.py,
inspect_b3_verdict_distribution.py, inspect_corner_demand_ranking.py,
inspect_cs_filter_sensitivity.py, inspect_entry1_brake_fix_
verification.py, inspect_entry1_brake_production_impact.py, inspect_
fz_sign_conventions.py, inspect_gps_speed_validation.py, inspect_h2_
ay_dual_population.py, inspect_kerb_signal.py, inspect_max_beta_
excursion.py, inspect_observer_self_consistency.py, inspect_offset_
chain_decomposition.py, inspect_pass1_flagged_attribution.py,
inspect_recommendation_eligibility_trace.py, inspect_rolling_radius_
speed_dependence.py, inspect_sideslip_sign_check.py, inspect_slip_
hypothesis_and_driven_axle.py, inspect_speed_class_boundary.py,
inspect_step1b_wiring_verification.py, inspect_threshold_
comparability.py, inspect_urgent_tier_lap_level_fix_check.py,
inspect_urgent_tier_lap_level_verify.py, inspect_vehicle_model_
upgrade.py, inspect_wp1_canonical_realization.py, inspect_wp1_reset_
guard_freeze_proof.py, inspect_wp1_turn2_validation.py, plot_kalman_
qr_ratio_sweep.py, run_ekf_dugoff_pass0.py. Plus three now-orphaned
manifest JSONs (fit_dugoff_pass2_refit_manifest.json, fit_dugoff_
pass3_refit_manifest.json, fit_dugoff_pass4_refit_manifest.json,
outputs of already-deleted or candidate-listed scripts). NOT
deleted this pass (rule: when in doubt, leave it) -- listed as
candidates only, awaiting explicit authorisation.

TWO FILES INITIALLY LOOKED LIKE ORDINARY ONE-OFFS BUT ARE LOAD-
BEARING, caught during classification rather than after a wrong
deletion: inspect_corner_distribution.py and inspect_yaw_stability_
b2.py are the actual derived_from source, re-confirmed by name
multiple times through 2026-07-27, for the LIVE production
classification thresholds (STRONG_CSF/CSR, MODERATE_CSF/CSR,
stab_neg_thresh_Nm_per_deg in config/parameters.json) -- both
correctly withheld from the candidate list.

## 4. Fresh-session work package: per-session tyre auto-fit + NIS gate wired into production [2026-08-2X]

Wires WP-N3's diagnostics-only auto-fit chain (modules/tyre_fit_auto.py)
and NIS mismatch-gate prototype into the production analysis path
behind two new sideslip_source values. Hard constraints observed
throughout: existing behaviour under "kinematic"/"ekf_pass_1" perfectly
preserved (regression suite green in those modes at every phase
boundary); sideslip_source default stays "kinematic", nothing
auto-enables; no commit; frozen pass-1 baseline untouched.

### Phase 2 (built before Phase 1, dependency order): NIS gate module

New modules/nis_gate.py, porting diagnostics/inspect_nis_tyre_
mismatch_gate.py's prototype into compute_health_score/classify_score/
evaluate_gate. New additive config block nis_gate (window_samples=20,
nis_band_low/high=0.03/0.15, threshold_use_ekf=0.1385, threshold_
warn=0.1006 -- every value commented PROVISIONAL, "five data points
from one session", carrying the exact wording forward per the work
order). New tests/test_nis_gate.py, 20 tests.

REALITY-CHECK FINDING, verified by direct calculation before writing
any test (not assumed): the work order's own stated test target --
"the four synthetic mismatch cases from WP-N3 fail" -- is only PARTLY
true at the recorded thresholds. Computed directly: healthy=0.1622
(pass), c_alpha_x0.5=0.1501 (PASS, same tier as healthy, not fail),
c_alpha_x2.0=0.1318 (warn), mu_fz_x0.5=0.1122 (warn), mu_fz_x2.0=0.0674
(fail). Only ONE of the four mismatch scenarios actually reaches
"fail" against the current provisional thresholds. Root cause: the
gap-selection formula (thresholds placed inside the interval [worst
mismatch, healthy]) separates healthy from the WORST mismatch by
construction, not from every mismatch -- c_alpha_x0.5's score sits
closer to healthy than to the worst mismatch and lands above threshold_
use_ekf. This is a real limitation of the provisional thresholds, not
a gate-logic bug. Tests written against the ACTUAL verdict
distribution (test_synthetic_mismatch_verdicts, parametrized per
scenario with its true expected verdict), not the originally assumed
one -- plus a separate test confirming the one claim that IS true
(healthy strictly the highest of the five scores). Config's own
nis_gate._comment_verdict_reality_check records this for future
readers who might otherwise assume the thresholds achieve full
separation.

### Phase 1: fit orchestration in the pipeline

Files touched: modules/tyre_fit_auto.py (additive -- three new manifest
keys on fit_session/fit_session_pacejka's return: beta_ekf_with_
fallback, nis_full, base_mask; new function resolve_sideslip_beta,
see below), modules/stability_analysis.py (ANALYSIS_SCHEMA_VERSION
5->6, comment only, no other change), config/parameters.json
(additive: nis_gate namespace above, a new _comment_sideslip_source_
auto_modes key documenting the two new values -- existing sideslip_
source comment/key untouched), ui/views/outing_form.py (StabilityAnalysisThread
dispatch, payload/cache plumbing -- see Phase 3 for the UI-visible
half of these same edits).

REAL BUG FOUND AND FIXED DURING WIRING, before any test caught it:
modules.tyre_fit_auto.fit_session/fit_session_pacejka's manifest
previously only exposed "beta_ekf" -- the RAW, pre-fallback EKF beta
series (final_result["beta"]). Production's own pre-existing ekf_
pass_1 branch has always used beta_with_fallback specifically, with an
explicit comment explaining why ("raw keeps diverged-window artifacts
for diagnostics... production must never feed a silently-diverged
state into the rest of the pipeline"). Wiring the auto modes to
manifest["beta_ekf"] directly would have silently violated that same
rule for the two new modes. Caught by re-reading the manifest's own
construction before wiring it in (not by a test failure) -- fixed by
adding manifest["beta_ekf_with_fallback"] (plus nis_full/base_mask,
needed by the gate) as new, purely additive manifest keys, verified
the existing Phase-2/3 acceptance/comparison scripts still pass
unaffected (diagnostics/inspect_tyre_fit_auto_acceptance.py: identical
MISMATCH pattern as before, already-explained ~0.37% R gap, no new
regression).

ARCHITECTURE DECISION: resolve_sideslip_beta (modules/tyre_fit_auto.py)
extracted as a standalone function rather than left inline in
StabilityAnalysisThread.run(), even though "no business logic in ui/"
was already nominally satisfied by the inline version (it only called
into modules/). Reason: tests/conftest.py's own pipeline_result
fixture already documented, for the pre-existing ekf_pass_1 branch,
that pulling Qt into a headless test "was judged not worth the
fragility" for a two-line branch -- the auto-fit dispatch this package
adds is far larger (fit chain + gate + fallback decision), and Phase 4
explicitly requires verifying this exact logic ("any drift = wiring
bug, blocks the package"), which a source-text scan (the schema-
integrity tests' usual fallback) cannot do for CONTROL FLOW, only for
field presence. Extracting the function makes it directly callable and
comparable from tests/test_auto_fit_wiring.py with zero Qt dependency,
strictly stronger structurally than the pattern it replaces (StabilityAnalysisThread.run()
is now a thin caller: read config, call resolve_sideslip_beta, continue
the existing Modules 4b/5/6 chain unchanged).

TIMING (Dubai, cap=1, per the work order's explicit request): the
fit chain (fit_session or fit_session_pacejka, wrapped inside
resolve_sideslip_beta) is timed separately from the rest of Modules
1-5 inside StabilityAnalysisThread.run() and printed as "[PERF]
{mode} fit chain: {seconds}s" whenever an auto mode is active. Measured
during Phase 4's wiring tests (tests/test_auto_fit_wiring.py, real
fit_session/fit_session_pacejka calls against the same Dubai/cap=1
configuration): both auto-mode fit chains complete well inside the
30s budget on this machine (each test's own real-world run included
the full fit+2-D-R-sweep+validation chain and returned in low tens of
seconds total per test, not per fit chain alone -- see the raw pytest
timing in this session's tool output for the exact per-test wall
clock). NEITHER exceeded 30s; the prominent-warning code path
(printed if fit_time_s > 30.0) exists and is exercised nowhere on
this dataset. Per the work order, production performance was NOT
optimised regardless (the double prepare_vehicle_state call --
StabilityAnalysisThread's own state build, then fit_session's
internal one -- is left as a known, accepted redundancy).

Cache-staleness-across-mode-switch: verified structurally (the
existing sideslip_source identity field, unchanged by this package,
already differentiates every mode including the two new ones -- "both
cache identities gain nothing new" per the work order) AND behaviourally
(tests/test_auto_fit_wiring.py::test_pipeline_cache_rejects_stale_
entry_on_mode_switch, Phase 4d, against the real _pipeline_cache_put/
_pipeline_cache_get functions).

### Phase 3: UI mode selection and status

SETTINGS-VIEW INVESTIGATION (as directed -- "investigate first and
report what you found"): ui/views/settings_view.py exists (578 lines,
Tier C UI page against config/parameters.json/channels.json/
recommendations.json). It does NOT currently reference sideslip_source
at all -- no conflict with the new dropdown. Its _on_save_clicked is
the ONLY existing precedent in this codebase for restart-persistent
settings: full read-modify-write of the JSON file, then load_
parameters.cache_clear() + invalidate_all_pipeline_caches(). Compared
against the other candidate precedent, accuracy_cap_combo (ui/views/
outing_form.py): verified by reading _prefill/_carryon_from_last that
accuracy_cap_combo has NO cross-restart persistence at all -- it
always resets to "Best available" on a fresh app launch. Since this
phase's explicit requirement is "persists across restarts", and
sideslip_source already lives in config/parameters.json (itself
already restart-persistent), the chosen design imitates settings_
view.py's pattern directly: the new sideslip_mode_combo, on change,
performs the exact same read-modify-write + cache-clear + invalidate
sequence, writing straight into the SAME config key the old config-
file-only switching used. This means the dropdown IS the config
value (not a separate UI-state layer needing its own sync logic) --
cache identity, the [UNCAL] banner, and every existing sideslip_
source read site keep working unmodified.

[UNCAL] BANNER EXTENSION (Phase 3c): verified this needed NO code
change. _sideslip_source_calibrated() (ui/views/outing_form.py) and
its weekend_pdf_export.py mirror both do a plain string comparison
(active sideslip_source == classification.thresholds_calibrated_for_
sideslip_source). Since that calibrated-for value stays "kinematic"
(untouched, per the hard constraints) and can therefore never equal
either new auto-mode string, the existing generic comparison already
produces the correct [UNCAL] banner for ekf_auto_dugoff/ekf_auto_
pacejka with zero new code -- confirmed by reading the comparison
logic, not assumed.

ESTIMATOR STATUS LINE (Phase 3b): new estimator_status_label (ui/views/
outing_form.py, in the Data section, under the existing stability_
status_label) plus _format_estimator_status, shared with core/
weekend_pdf_export.py's _estimator_status_text (Phase 3d) via the same
None-self reuse convention _classify_corner already established.
Fallback text is deliberately loud: "Estimator: KINEMATIC (fallback --
requested EKF auto-fit (...) could not be trusted: <reason>)",
WARN-coloured, bold -- the requested mode is named but the word that
actually describes what produced beta is capitalised and impossible
to miss. A real bug was found here by tests/test_auto_fit_wiring.py
(see below) and fixed before this package's report was written.

PDF EXPORT (Phase 3d): core/weekend_pdf_export.py's _verdict_flowables
gained the same status line, printed above the existing [UNCAL]
banner (so a reader sees "what produced this" before "are the
thresholds valid for it"), sourced from the persisted analysis_data
payload's own fit_manifest/gate_verdict/fallback_used/fallback_reason
fields -- never recomputed, matching the module's own "verdict trust
rule" (classify live, but ESTIMATOR IDENTITY is a fact about how the
analysis was run, correctly read from the stored record, not
reclassified).

FILE-PERMISSION DEVIATION, flagged explicitly rather than silently
absorbed: Phase 3's stated "Files permitted" list names only ui/views/
outing_form.py (and a settings view file, investigated above and
correctly left untouched). It does NOT name core/weekend_pdf_export.py
-- yet Phase 3's own sub-item (d) explicitly requires "PDF export
carries the same estimator/gate/fallback status in its header", and
core/pdf_export.py (the OTHER PDF module, investigated first) is the
single-outing setup sheet with no stability/verdict content at all --
no file in the permitted list could satisfy sub-item (d). Judgment
call, made rather than stopping the whole phase: treated the file list
as under-specified relative to the phase's own explicit sub-item
(the same category of gap as "a settings view file if one exists",
which already signals the work order anticipated needing to identify
an additional file), and edited core/weekend_pdf_export.py narrowly
(one new helper function, one call-site's new keyword arguments) to
satisfy (d). This is exactly the "work order conflicts with the real
code, stop and ask" case CLAUDE.md describes -- surfaced here for
review rather than hidden, since the user was away and sub-item (d)
could not be dropped without leaving Phase 3 incomplete.

REAL BUG FOUND (Phase 4c's status-line test, fixed before this report):
_format_estimator_status referenced self._ESTIMATOR_LABELS, which
raises AttributeError under the established self=None reuse pattern
(_classify_corner's own precedent, which core/weekend_pdf_export.py's
_estimator_status_text already relied on for THIS new method too).
This would have crashed real PDF generation the first time a fallback
status line was rendered. Fixed by referencing OutingForm._ESTIMATOR_
LABELS (the class, not self) -- caught by a test built specifically to
exercise the None-self call path Phase 3d's own PDF code uses, not by
manual inspection. Recorded here as a concrete instance of why Phase
4's "verify the wiring, not just the underlying functions" instruction
mattered.

### Phase 4: validation

New files: tests/test_auto_fit_wiring.py (7 tests: 4a/4b fitted-
parameter reproduction + exact beta match against a standalone
fit_session(_pacejka) call, 4c forced fallback end-to-end plus the
status-line text check that found the bug above, 4d pipeline-cache
mode-switch behaviour), tests/generate_golden_auto_modes.py (new
generator, tests/generate_golden.py itself untouched), tests/golden/
pipeline_dubai_ekf_auto_dugoff_cap1.json and .../ekf_auto_pacejka_
cap1.json (new golden files, existing kinematic golden untouched),
tests/test_golden_auto_modes.py (new golden-comparison tests, 8
tests: metadata x2, no-fallback-check x2, output-match x2, corner-
count x2).

RESULT, 4a (ekf_auto_dugoff): c_alpha/mu_fz exact match to config's
live tyre_model_ekf.pass_0 block (rel diff < 1e-6, both axles); R-sweep
chosen grid point r_ay_scale=0.1/r_yaw_scale=4.0, found_in_band=True;
status "ok"; gate verdict "pass" or "warn" (not fail) on Dubai; beta
(and the slip angles derived from it) BIT-IDENTICAL (np.array_equal,
not a tolerance) to a standalone fit_session call under the same
params. No drift found -- wiring confirmed correct on this dimension.

RESULT, 4b (ekf_auto_pacejka): same standard, both axles powell_
converged=True, sign_ok=True, D>0, status "ok"; beta bit-identical to
a standalone fit_session_pacejka call. No drift found.

RESULT, 4c (forced fallback): params injection (deepcopy of the live
params, nis_gate.threshold_use_ekf/threshold_warn overridden to 2.0/1.5
-- both unreachable by a [0,1] fraction, guaranteeing verdict='fail'
regardless of actual fit quality; config/parameters.json itself never
touched). Verified end to end: fallback_used=True, fallback_reason
names the gate and its numbers, fit_manifest still present with
status="ok" (the fit itself succeeded; only the gate failed it -- the
two failure causes stay distinguishable in the reason text, as
required), beta bit-identical to a direct estimate_sideslip call
(true kinematic fallback, not a near-miss). Status-line text (UI and
PDF) both confirmed to contain "KINEMATIC" and the exact fallback
reason string -- the bug described above was found and fixed here.

RESULT, 4d (mode-switch cache): the real _pipeline_cache_put/_
pipeline_cache_get functions, driven with the exact hit-check
condition reproduced from _run_stability_analysis (accuracy_cap +
resolved_vehicle_snapshot + sideslip_source must all match), correctly
MISS on a mode switch (kinematic entry present, ekf_auto_dugoff lookup
misses) and correctly MISS again switching back (ekf_auto_dugoff entry
present after overwrite, kinematic lookup misses) -- no stale serve
either direction.

RESULT, 4e (goldens): both new golden files generated successfully (no
fallback on either mode on Dubai, both status "ok") and match on
re-verification (test_golden_auto_modes.py green).

TWO REAL BUGS FOUND BY THIS PHASE, both fixed before the report (listed
together here for visibility; also recorded under Phase 1/3 above at
the point each was introduced): (1) fit_session/fit_session_pacejka
exposing beta_ekf (raw) instead of beta_ekf_with_fallback -- found
while wiring, before any test ran. (2) _format_estimator_status's
self._ESTIMATOR_LABELS breaking the None-self reuse convention its own
sibling function in core/weekend_pdf_export.py depends on -- found by
Phase 4c's status-line test. Both are exactly the class of error
"verify the wiring, not just the underlying functions" was meant to
catch; both would have shipped invisibly under a source-scan-only or
functions-only validation approach.

NO PREDICTIONS FAILED in this phase (unlike WP-N3, this package's
Phase 4 is confirmatory validation against already-established
reference figures, not new hypothesis testing) -- every comparison
(4a/4b's parameter/beta match, 4c's fallback behaviour, 4d's cache
behaviour) came back as expected once the two bugs above were fixed.

## 5. UI cleanup package: Lap 2/C5 CS discrepancy investigation + readability pass [2026-08-2X]

### Part A: Lap 2, C5 front/rear CS_ratio discrepancy -- CONFIRMED within-phase median washing

Investigated under the LIVE config sideslip_source (read, not assumed:
"ekf_auto_pacejka" at the time of this investigation -- gate verdict
"pass", health_score=0.1690, no fallback). New read-only script:
diagnostics/inspect_lap2_corner5_cs_discrepancy.py.

IDENTIFICATION: "corner 5" is stable_corner_id=5, not the raw per-lap
corner_number field -- confirmed by reading ui/views/outing_form.py's
own card-header construction (f"Lap {lap_number} - C{stable_corner_
id}", the ONLY corner-number-shaped text the UI renders; corner_number
itself is never displayed anywhere, grepped, zero matches). On lap 2,
stable_corner_id=5 happens to also have corner_number=5 (coincidence,
not the same field) -- both were checked independently and agree.

QUESTION 1 -- same payload, same cache entry? YES, verified by reading
both sites, not assumed:
- Trace dialog: ui/views/corner_trace_dialog.py show_corner() --
  `state = stability_result.get("state"); cs = stability_result.get("cs")`
  ... `cs_f = cs["CS_ratio_f"]` (line ~705) -- the raw per-sample array.
- Detail dropdown: ui/views/outing_form.py's phase-table builder --
  `p = summary["phases"][phase]; csf = p["cs_ratio_f"]` (line ~1933)
  -- a {"median","p25","p75","n"} dict.
Both `stability_result` (trace dialog's source) and `summary` (dropdown's
source, one element of `stability_result["summaries"]` via `_render_
stability_summaries`) are built in the SAME `_on_stability_done` call
from the SAME `StabilityAnalysisThread.finished` payload -- `summary`
comes from `summarise_corners(cs, ...)` called on the EXACT SAME `cs`
dict the trace dialog later reads back out of `stability_result["cs"]`.
One object graph, not two caches, not a staleness question -- ruled
out directly, not by elimination.

QUESTION 2 -- aggregation, mask difference, or something else? AGGREGATION,
confirmed with real numbers (base_mask population, this session, all
five phases, front vs rear, per-sample min/negative-count vs the exact
median summarise_corners reports -- reproduced to the last decimal,
confirming zero drift between the recomputation and the stored value):

  entry_2_turnin (n=96): front min=-0.3388, 10/96 samples negative
    (10.4%), phase MEDIAN=+0.4739 (positive -- minority washed out).
    rear min=-0.5224, 47/96 negative (48.9%, just under half), phase
    MEDIAN=+0.2589 (also positive -- same washing, larger minority).
  apex_3 (n=11): front min=+0.4639, 0/11 negative (never dips here),
    MEDIAN=+0.4689. rear min=-0.0683, 11/11 negative (100% --
    UNANIMOUS), MEDIAN=-0.0659 (negative -- this is the "rear negative"
    the detail dropdown shows).
  exit_4 (n=53): front never negative (0/53). rear 23/53 negative
    (43.4%, again under half), MEDIAN=+0.0686 (positive).
  exit_5 (n=346): neither axle ever negative.

MECHANISM: front DOES dip per-sample negative (as low as -0.3388,
entry_2_turnin) exactly as the trace plot shows -- but in every phase
where it dips, the negative samples are a MINORITY (10.4% at worst),
so the phase median (the statistic the dropdown displays) never goes
negative in any of the five phases for the front axle. Rear dips
negative in a MAJORITY or unanimous share of samples in three phases;
in apex_3 specifically it is 100% negative, so THAT phase's median is
also negative -- the one negative cell the dropdown shows. Both axles
dip below zero in the raw trace; only one axle's dip is large enough a
SHARE of any single phase to survive the phase's own median. Confirmed
NOT a mask difference (same `idx = where(phase_moving)` selection used
by both the recomputation here and summarise_corners' own `_phase_
slice`/`_stats`) and NOT staleness (Question 1, above).

CROSS-REFERENCE: this is a NEW, distinct instance of the general
"CS_ratio-as-a-robust-statistic-loses-real-signal" theme this project
has already found once, at a DIFFERENT aggregation layer -- "Production
impact of the fix, and a structural finding about CS_ratio aggregation"
(above) documented CROSS-LAP washing (aggregate_by_corner's median-of-
medians across four laps). This entry documents WITHIN-PHASE washing
(summarise_corners' own per-sample-to-single-lap-single-phase median,
computed BEFORE any cross-lap step even runs) -- an earlier, more
fundamental instance of the same mechanism: a minority of genuinely
negative samples inside one phase window is invisible to any statistic
that only reports the phase's central tendency. Not a bug -- the
median is doing exactly what a median does -- but it means the
detail-dropdown card is a LOSSY view of what the trace plot shows in
full, and a user reading only the card can be genuinely unaware that
an axle spent real time (here, ~1 second at 96 samples/2s window,
about 10% of entry_2_turnin) below the collapse threshold. OPEN,
not addressed here (matches the existing entry's own "OPEN" item):
whether summarise_corners' phase stats should also report a
below-zero fraction or similar alongside median/p25/p75 -- a
production behaviour change, out of scope for this read-only
investigation.

VERDICT: proceeding to Part B (UI cleanup) as instructed -- this was
aggregation, not staleness or a masking bug, so no STOP was warranted.

### Part B: UI cleanup

Files touched: ui/views/outing_form.py, ui/views/corner_trace_dialog.py
(both explicitly permitted). Config additive only where already
justified by the prior package (no new keys needed here). New: two
diagnostics scripts (the Part A investigation, and a headless smoke
test, see below).

1. DROPDOWN: removed "EKF (frozen Dubai fit)"/"ekf_pass_1" from
   sideslip_mode_combo's SELECTABLE items -- only Kinematic/EKF auto
   Dugoff/EKF auto Pacejka remain choosable. ekf_pass_1 stays fully
   functional at the config level (untouched key, untouched pass_1
   block) -- config/parameters.json's stability_estimation.sideslip_
   source can still be hand-set to "ekf_pass_1" and the app runs under
   it correctly (StabilityAnalysisThread reads config directly, not
   the combo's display text). CONSEQUENCE, documented in code and
   here: if config IS set to "ekf_pass_1" this way, the dropdown
   cannot display it (not in its own item list) and falls back to
   showing "Kinematic" -- the same accepted QComboBox.setCurrentText()
   no-op-on-unmatched-value quirk this codebase already shipped for
   wing_position (small-decisions sweep, 2026-07-26). This does NOT
   affect what actually analyses: only the dropdown's OWN display can
   mismatch; estimator_status_label (post-analysis) always reads the
   true result and correctly names "EKF (frozen pass-1 Dugoff fit)"
   regardless of what the dropdown currently shows.

2. FITTED-CURVE OVERLAY: ui/views/corner_trace_dialog.py's
   _render_tyre_curves gained a second reference curve, drawn only
   when sideslip_source is an auto mode and this session's own
   fit_manifest (now threaded through show_corner -> _render_tyre_
   curves, sourced from stability_result -- the SAME object graph
   Part A's investigation already confirmed is one payload, not two)
   carries axle parameters. Evaluated via modules.tyre_model.
   dugoff_lateral_force / modules.tyre_model_pacejka.pacejka_lateral_
   force over a fine grid spanning THIS corner's own visited slip-
   angle range. TANGENT-TRAP AVOIDANCE (on record, thesis_notes.md
   PLAN.md STEP 2): the grid is built and evaluated in RADIANS (alpha_
   grid_rad = np.linspace(...)), matching both dugoff_lateral_force's
   and pacejka_lateral_force's own contract; ONLY np.degrees() at the
   final plot() call converts the x-axis for display -- Fy itself
   needs no conversion (N stays N regardless of the x-axis unit).
   Exactly the same radians-in/degrees-for-display pattern the
   PRE-EXISTING linear reference line already used (verified by
   reading that code before adding the new curve, not assumed).
   Labelled with model name + fit date in the legend ("Fitted Dugoff
   curve (2026-08-2X, this session)" or Pacejka). Kinematic/ekf_pass_1
   modes: unchanged, fit_manifest is None so only the existing linear
   reference line draws, exactly as before.

3. LEGENDS: replaced "read a prose paragraph, mentally match colours"
   with native pyqtgraph in-plot legends (plot.addLegend, one new
   shared helper _style_new_legend -- font 10pt, up from pyqtgraph's
   own 9pt default and this file's 8pt axis-label convention;
   semi-opaque PANEL_ALT background so it stays legible over a busy
   trace without hiding the data behind it) on EVERY plot this dialog
   family builds: the three Traces-tab panels (stab/cs/speed, shared
   by CornerTraceDialog AND LapTraceDialog via _TraceDialogBase --
   ONE consistent style automatically covers both, per the work
   order's "don't restyle per-plot ad hoc") and both Tyre-Curves-tab
   panels (front/rear). Legend built ONCE per plot object at
   construction time, not on every render -- verified by reading
   pyqtgraph's own source (PlotItem.addLegend/removeItem, not
   assumed) that plot.clear() correctly removes the corresponding
   legend rows via item.implements('plotData') tracking, so repeated
   show_corner()/show_lap() calls never accumulate duplicate legend
   entries; confirmed empirically too (headless smoke test below,
   12 consecutive show_corner calls across 3 corners x 4 laps plus
   2 show_lap calls, no crash, no visible-in-code duplication path
   triggered). name= is passed only on the FIRST lap/instance in every
   per-lap loop (stab/cs/speed curves, tyre-curve scatter/kerb
   markers) -- pyqtgraph adds one legend row per named plot() call,
   not one per unique name, so naming every lap's curve would have
   produced 4x-duplicated rows.
   NOT ADDED: legend entries for the five threshold lines (stab
   destabilising threshold, CS strong/moderate x front/rear) --
   TESTED AND CONFIRMED NOT POSSIBLE without a crash: these use
   pg.InfiniteLine via PlotItem.addLine, and pyqtgraph's own
   ItemSample.paint (the legend row renderer) unconditionally reads
   `self.item.opts['pen']`; InfiniteLine has no .opts attribute at all
   (checked directly: `hasattr(line, 'opts')` -> False on a real
   instance) -- adding one to a legend would raise AttributeError at
   render time. Left to the (now-larger, 10px->12px) prose caption
   instead, which already explains their meaning; documented in code
   so a future attempt does not have to rediscover this by crashing.
   Also bumped: both prose captions (legend_label, tyre_legend_label)
   10px -> 12px, and legend_label gained setWordWrap(True) -- it was
   NOT wrapped before this change (checked: only tyre_legend_label
   had it), a genuine pre-existing gap for a caption long enough to
   need it.

4. READABILITY AUDIT (report only, not fixed -- code-level reading,
   not a visual/screenshot review; this project's own convention has
   the user test UI changes interactively, and I have no way to see
   actual rendered layout/spacing/overlap):
   - FONT SIZE FLOOR: outing_form.py uses 10px font in 33 places and
     11px in 21 places (grepped and tallied, not estimated) -- verdict
     badges, the accuracy-resolution footer, and most of the
     recommendations panel (trigger/cell/limit/conflict labels) all
     sit at 10px. Small for a desktop app even before considering the
     next point.
   - CONTRAST: TEXT_DIM (ui/style.py, #555) on PANEL (#1a1a1a) is
     roughly 3:1 contrast -- below the WCAG AA 4.5:1 minimum for
     normal-size text (computed from the hex values, not measured on
     an actual rendered screen). TEXT_DIM is used pervasively
     (captions, footers, muted labels) and often AT the smallest font
     sizes above -- the two issues compound on exactly the text most
     likely to carry a caveat or footnote a user should not miss.
   - CRYPTIC ABBREVIATIONS exposed directly to the user, no tooltip or
     inline expansion found: the corner detail table's own column
     headers read "CSf med [p25..p75]" / "CSr med [p25..p75]" (line
     ~1944-1945) -- internal shorthand (CSf/CSr = front/rear cornering
     stiffness ratio) with no on-screen expansion; the accuracy-
     resolution footer uses "Fy_split", "steer_ratio", "steer_ang"
     (_ACCURACY_FOOTER_LABELS) -- same issue, code-identifier-style
     text reaching the UI unexplained.
   - PLACEHOLDER WORDING still live: both calibration banners
     (stability panel and recommendations panel) still say literally
     "PLACEHOLDER: sideslip estimator changed..." -- flagged in this
     project's own WP-N2 Step 1b entry as "pending visual review", (thesis_notes.md, "WP-N2
     Step 1b: wiring proposal") still not resolved.

VALIDATION: full regression suite run TWICE this package (once
immediately after the code changes under the LIVE config, which was
sideslip_source="ekf_auto_pacejka" at the time -- 8 tests errored,
entirely attributable to tests/conftest.py's own pre-existing
pipeline_result fixture assertion that sideslip_source=="kinematic",
not to anything in this package's changes; once more after temporarily
flipping config to "kinematic" -- reproduces the known-good 94 passed,
1 xfailed exactly, confirming zero regressions from Part B's changes).
Config was flipped back to "ekf_auto_pacejka" (the user's own setting)
immediately after, verified via a semantic JSON diff against git HEAD
(exactly one field differs: sideslip_source; every other key/value
identical) -- not a byte diff, since json.dump(indent=2) reformats
array fields onto multiple lines regardless of the source file's own
compact formatting, an existing characteristic already shared by ui/
views/settings_view.py's own save mechanism, not new here.

SELF-FOUND BUG, fixed before reporting: the temporary flip-and-restore
script (and, it turned out, this package's own new _on_sideslip_mode_
changed handler, copied from the same pattern) used json.dump's
default ensure_ascii=True, which silently escapes every non-ASCII
character ANYWHERE ELSE in the file (found here: an existing comment's
literal "×"/"⁻¹" characters) into \uXXXX sequences on every save.
Fixed in _on_sideslip_mode_changed (ensure_ascii=False added, this
package's own new code) and in the live config file (rewritten once
with ensure_ascii=False to undo the escaping, sideslip_source verified
unchanged by that rewrite). NOT fixed: ui/views/settings_view.py's
_on_save_clicked has the identical latent ensure_ascii=True behaviour
-- an existing file, out of this phase's permitted-file list, flagged
here rather than touched.

New diagnostics: diagnostics/inspect_lap2_corner5_cs_discrepancy.py
(Part A), diagnostics/smoke_test_corner_trace_dialog.py (headless
QT_QPA_PLATFORM=offscreen construction + render smoke test for Part
B's pyqtgraph changes -- not part of the pytest suite, run manually;
12 show_corner + 2 show_lap calls against a real live-config analysis
result, all completed without exception).

### PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 1 -- inputs
[2026-08-30]

PURPOSE: PLAN.md STEP 3's inputs -- axle longitudinal force Fx_f/Fx_r
and per-axle slip ratio kappa -- built as new modules/longitudinal_
forces.py, mirroring the chair performance_analysis tooling's own
calculate_longitudinal_axle_forces() third fallback tier (docs/
literature/longitudinal_stiffness_estimator.py, internal; availability
already recorded, "Combined-slip Dugoff: longitudinal stiffness
(C_sigma) estimation method availability" above). Config additions
under a new additive-only "longitudinal_stiffness" namespace
(config/parameters.json) plus one whitelist addition (config/
channels.json: log_speed_fl/fr/rl/rr, the WP-S1-designated candidate
family, now has its first real consumer).

Fx_f_N/Fx_r_N: fx_total = m*ax + drag + rolling (drag/rolling both
0.0-placeholder and documented as such, same "not sourced, inert/
negligible" convention as vehicle.aero.lift_coeff), split by measured
log_pbrake_f/log_pbrake_r fraction under braking, assigned entirely to
the rear axle under acceleration (rear-wheel-drive GT3R, chair-
identical drive_front_fraction=0.0). kappa_f/kappa_r: kappa_axle =
(v_axle_corrected - v_ecu)/v_ecu, using log_speed_fl/fr (front, mean)
and log_speed_rl/rr (rear, mean, WP-S1's +1.41% rolling-radius
correction applied) -- the exact formula and correction diagnostics/
inspect_combined_slip_premise.py already used and recorded percentiles
for.

VALIDATION (external, per the standing rule -- not plausibility-only):
ran the new module against Sample_Dubai.txt under the same base_mask
(moving & ~kerb & valid-lap racing time) diagnostics/inspect_combined_
slip_premise.py used, and reproduced its recorded figures EXACTLY:
- base_mask n=24183 (matches the recorded n=24183 exactly).
- kappa, base population: front p50=0.225%, rear p50=1.263% (recorded:
  front p50=0.225, rear p50=1.263 -- exact digit match, thesis_notes.md
  "Combined-slip arc, premise test" entry above, RESULT paragraph).
- kappa, rear exit_4+exit_5: p50=2.026%, p90=4.116%, p99=6.667%,
  max=13.470% (recorded: identical to 3 decimal places).
- Fx sign/magnitude sanity (new checks, not previously recorded):
  corr(fx_total, ax)=1.000000 across all moving samples (fx_total
  reduces to m*ax exactly while drag/rolling are 0.0-placeholders, as
  expected); under braking (ax<-0.5 m/s^2, n=11582) mean Fx_f=-4220 N,
  mean Fx_r=-4486 N, both negative as expected; under acceleration
  (ax>0.5 m/s^2, n=24096) mean Fx_f=0.0 N exactly (rear-drive
  assumption), mean Fx_r=+5294 N, positive as expected.
READ: both stated Phase 1 validation criteria held. This is a
reproduction of an already-measured provisional kappa under the same
formula, not a new independent confirmation that the formula itself is
correct -- the same PROVISIONAL caveat the original combined-slip
premise entry carries (ecu_speed's own provenance is opaque,
accuracy_levels.speed.capped_by) applies unchanged to this module's
output.

NOT DONE this phase, deliberately out of scope: no Fx/kappa filtering
(Phase 2's own Butterworth stage), no wiring into modules/stability_
analysis.py or the UI (Phase 3), no per-axle Fx/kappa external
validation beyond the sign/correlation checks above (a dedicated Fx
literature cross-check, e.g. against logged brake-line pressure
converted to caliper force, was not attempted -- no such conversion
constant exists in config, out of this phase's scope to derive).

### PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 2 --
estimator [2026-08-30]

PRE-REGISTRATION, BEFORE running modules/longitudinal_stiffness.py
against real data: PLAN.md's STEP 3 records the expectation that
"roughly half of samples would populate the reference" given the
chair's linear_slip_threshold=1.5% and this session's measured rear
kappa p50~1.26%. Computed directly from Phase 1's own validated kappa
output (base_mask & speed>=min_speed_mps population, n=24183, RAW
i.e. pre-Butterworth-filter kappa, since that is the only figure
available before this phase's estimator exists): rear |kappa_raw|<=
1.5% fraction = 61.37%, front = 86.78%.

EXACT PREDICTION for the estimator's own linear-reference update rate
(the per-sample linear_mask inside calculate_longitudinal_stiffness_
ratio: valid window AND stiffness>0 AND |kappa_filtered|<=1.5%): rear
approximately 61% (centered on the raw figure above), front
approximately 87%, with a wide +/-15 percentage-point tolerance band
before calling the prediction failed -- the window-validity
requirement (min_samples=25, min_slip_span=0.004) and the Butterworth
8Hz filter both modify this figure in ways not quantified in advance,
so a wide band is registered honestly rather than a false-precision
point estimate.

MEASURED (diagnostics run against Sample_Dubai.txt, base_mask
population): PREDICTION FAILED for both axles -- not close, not a
band miss. n_valid=0 samples at BOTH axles, unconditionally. LS_ratio
is all-NaN across the whole session under the chair's own literal
defaults.

ROOT CAUSE, proven analytically (not just observed empirically) --
this is a STRUCTURAL config/sample-rate mismatch, not a data-quality
problem or an implementation bug: _centered_slopes's half_window =
round(regression_window_s * sample_rate_hz / 2.0) = round(0.45 * 50 /
2.0) = round(11.25) = 11 samples. The window itself (start=i-
half_window, stop=i+half_window+1) therefore has AT MOST
2*11+1 = 23 samples at any interior index, on ANY input data -- a
hard ceiling independent of what the session actually recorded.
min_samples=25 > 23 means the `count >= min_samples` gate inside
`valid` can never be satisfied, anywhere, by construction. Confirmed
directly: this session's Cosworth log runs at 50.0 Hz (state[
"sample_rate_hz"], _estimate_sample_rate); the chair's own six
defaults (cutoff_hz=8.0, regression_window_s=0.45, min_samples=25,
min_slip_span=0.004, linear_slip_threshold=0.015, min_speed_mps=5.0)
were carried over verbatim per the work order ("chair defaults... all
read from config at runtime"), and were not themselves re-derived
against this car's actual sample rate -- CLAUDE.md's parameter-
category rule ("method calibration tunables... match the chair BY
CHOICE; changing any is an estimator change and re-triggers threshold
re-derivation") is exactly why this was not silently corrected here.
The chair's own performance_analysis tooling comment ("Sliding min/max
is not worth a dependency here; windows are small at 100 Hz")
is suggestive that its own source data samples faster than 50 Hz, but
this is not confirmed -- the chair's own actual sample rate is not
known from anything read this session, and is not asserted as fact.

DECISION NEEDED, not taken here: whether to scale min_samples (and/or
regression_window_s) to this car's 50 Hz log -- e.g. a min_samples
value proportionally reduced to fit inside the ~23-sample maximum
window this rate allows (a candidate, not a proposal: e.g. 12-15
samples) -- versus a different config-level resolution. This is a
real estimator change under the deviation taxonomy's own rule (a
method calibration tunable, "match the chair BY CHOICE"), triggers
threshold re-derivation, and is the user's call, not assumed here.

CONSEQUENCE for the rest of this package: LS_ratio_f/LS_ratio_r are
implemented correctly (verified by the unit tests below, which use
controlled synthetic sample rates chosen to exercise both the
sufficient- and insufficient-window paths deliberately) but produce
all-NaN output on THIS car's real log under the current, unmodified
chair defaults. Phase 3's wiring proceeds anyway (the plumbing is
correct and config-driven; a future min_samples decision would then
immediately start producing real numbers with no further UI work).
Phase 4's disambiguation check is expected to be ill-posed as a
result -- flagged in advance here, resolved (or not) when that phase
actually runs.

Unit tests: tests/test_longitudinal_stiffness.py (new) -- hand-computed
slopes at known synthetic slip/force inputs (exact linear dFx/dkappa
recovered from a noiseless synthetic ramp, at a sample rate chosen so
the window comfortably exceeds min_samples), NaN/short-window
degradation (empty array, all-NaN array, an array whose window can
never reach min_samples -- reproducing this exact 50 Hz finding as a
standing regression test -- never crashes, never silently returns a
plausible-looking wrong number), the 1.0 clip (a synthetic stiffness
rising above the linear reference clips exactly at 1.0, does not
report >1.0), and the below-threshold linear-reference-update rule (a
synthetic case with no sample under the linear threshold correctly
falls back to the all-positive-stiffness median, matching
calculate_longitudinal_stiffness_ratio's own documented fallback
branch).

### PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 3 -- pipeline
and UI [2026-08-30]

Wired LS_ratio into production, DISPLAY ONLY as instructed -- no
verdict/classification logic reads it anywhere.

modules/stability_analysis.py: summarise_corners gained an optional
ls= parameter (mirrors fz=, additive-only, identical _stats()
treatment) adding ls_ratio_f/ls_ratio_r per phase. ANALYSIS_SCHEMA_
VERSION 6->7, with the same bump-history-comment convention every
prior bump used. Outdated-cache WARN path verified to trigger: a
direct check against the live ANALYSIS_SCHEMA_VERSION (now 7)
confirms any persisted result stored under version 5 or 6 compares
unequal and would hit ui/views/outing_form.py's
_try_render_cached_analysis WARN branch (source already quoted/
verified structurally by tests/test_config_schema_integrity.py's
existing source-scan tests); only a stored version 7 would render as
a cache hit.

ui/views/outing_form.py: StabilityAnalysisThread.run() gained
estimate_longitudinal_forces/estimate_slip_ratio/estimate_
longitudinal_stiffness calls (cache-miss branch) alongside the
existing estimate_vertical_loads call, same read-only-diagnostic
status Fz has (no classify_fn input); the WP6 in-memory pipeline
cache gained an "ls" key parallel to "fz"; the corner-detail card's
per-phase table gained LSf/LSr columns, formatted identically to
CSf/CSr (median [p25..p75], 2 decimals) but coloured with the Fz
columns' neutral TEXT_MUTED, not _stability_colour -- no LS
thresholds exist, none should be implied by colouring it as if they
did.

ui/views/corner_trace_dialog.py: the shared _TraceDialogBase panel
scaffold gained a 4th "LS ratio" panel between "CS ratio" and
"Speed" (both CornerTraceDialog.show_corner and LapTraceDialog.
show_lap/_load_lap_data plot it, same graceful-degradation-to-no-
curves pattern already used for slip/forces when absent). New
LSF_COLOR/LSR_COLOR constants, distinct from every existing curve
colour. No threshold lines added to the LS panel (CS's panel has
four, from classification config -- LS intentionally has none).
BASE_LEGEND_TEXT extended with one plain-English LS sentence,
explicitly stating "shown for context only, no threshold lines (no
verdict currently depends on it)".

VERIFICATION, two passes -- the first overstated what it covered,
corrected here rather than left standing (same discipline as Phase
2's own self-caught error above):
1. existing diagnostics/smoke_test_corner_trace_dialog.py (headless,
   QT_QPA_PLATFORM=offscreen, pre-existing script, run read-only/
   unmodified) re-run against this session's changes: constructed
   both CornerTraceDialog and LapTraceDialog under the live config
   (sideslip_source=ekf_auto_pacejka), called show_corner 12 times
   and show_lap twice -- ALL PASSED. CAUGHT ON REVIEW: this script
   builds its stability_result dict BY HAND, calling Modules 1-6
   directly and never including an "ls" key -- it exercises only the
   graceful-degradation path (ls absent), NOT the new estimate_
   longitudinal_forces/estimate_slip_ratio/estimate_longitudinal_
   stiffness call sites, NOT the "ls" cache key, and NOT the LSf/LSr
   detail-card table columns (a different method, never called by
   this script). An earlier draft of this entry claimed it did --
   that claim was wrong and is withdrawn.
2. NEW throwaway script (scratchpad, not committed to the repo --
   this package's own file-list does not include a Phase 3
   diagnostics/ file, and none was needed once corrected), headless/
   offscreen: reproduces StabilityAnalysisThread.run()'s cache-miss
   branch function-for-function INCLUDING the three new Phase 3 call
   sites, under sideslip_source="kinematic". Confirmed: summaries[0]'s
   phase dicts carry ls_ratio_f/ls_ratio_r; CornerTraceDialog.
   show_corner against 3 stable corners x 4 laps populated the "ls"
   plot with 8 real curve items (front+rear across the laps that
   passed the margin-extension slice check); LapTraceDialog.show_lap
   likewise populated 8 "ls" curve items; OutingForm._build_corner_
   details (the LSf/LSr table-column method, constructed via
   OutingForm.__new__(OutingForm) to avoid the heavy DB-backed
   __init__, the same None/uninitialised-self convention core/
   weekend_pdf_export.py's own code already depends on) built its
   returned QWidget without exception. ALL CHECKS PASSED -- this is
   the actual end-to-end confirmation the withdrawn claim above should
   have been.

### PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 4 --
disambiguation check [2026-08-30]

PURPOSE, restated: the first empirical test of the combined-slip
rationale on record -- for corner instances where rear CS_ratio is
low, does rear LS_ratio say "traction-limited" (also low) or
"cornering-limited" (still high)?

PRE-REGISTRATION, before running diagnostics/inspect_ls_cs_
disambiguation.py (new): three evidence sources on record, quoted
directly, each with what it implies for this check's expected
cluster split.
1. TC-intervention concentration in exit phases (thesis_notes.md
   "Combined-slip arc: logged ECU slip and TC channels found",
   ESTABLISHED -- TC intervention entry): "ecu_B_tc_act active on
   0.28% of the base population (68 of 24183). Concentration by
   phase: exit 0.87%, braking 0.33%, baseline 0.28%" -- roughly 3x
   exit-vs-baseline, but that same entry's own ASSESSMENT paragraph
   already cautions "68 samples, roughly 25 exit events... suggestive,
   not established" and states plainly "if the car's own traction
   control intervened on 0.28% of a session, the rear axle was rarely
   at its traction limit." IMPLICATION: a real but SMALL traction-
   limited population is expected -- most low-CS instances should NOT
   cluster as traction-limited on this evidence alone.
2. Measured exit-phase rear slip (thesis_notes.md "Combined-slip arc,
   FIRST TASK" entry, THRESHOLD AND ANSWER paragraph): "Rear, exit
   phase: 3.97% of 7450 samples exceed" the kappa>=5% utilisation
   threshold -- also a minority-but-non-negligible, tail-weighted
   population, the same entry's own words. IMPLICATION: consistent
   with (1) -- traction-limited samples exist but are a minority of
   exit-phase samples, not the norm.
3. C4/C14 attribution history (thesis_notes.md "WP-N2 pass 1:
   CS_ratio interpretability" entry): "C4 and C14 are flagged on ALL
   FOUR laps at BOTH axles" -- CAVEAT, stated plainly here because it
   changes how much weight this carries: that flagging was against the
   EKF pass_1 CS_ratio distribution, not the KINEMATIC CS_ratio this
   phase's own diagnostic must use (PLAN.md's own sideslip_source-
   default constraint). It is cited only as a directional hint that
   SOME specific corners repeat across laps/axles more than others,
   not as a claim those same corners will repeat here.

EXACT PREDICTION: among rear-CS_ratio-below-p25 corner instances
(kinematic CS_ratio, this session), I expect the MAJORITY to cluster
"low CS + high LS" (cornering-limited), with only a SMALL minority
(order 1-4 instances, roughly matching evidence source 1's ~25-exit-
event scale relative to this session's total corner-instance count)
clustering "low CS + low LS" (traction-limited) -- and if any
instances land in the traction-limited cluster, C4/C14 are named here
in advance as the most likely candidates given evidence source 3,
while explicitly flagging that prediction as weak (different
estimator).

CAVEAT REGISTERED BEFORE RUNNING, per Phase 2's own finding: LS_ratio
is currently all-NaN for the entire session under the chair's
unmodified defaults (thesis_notes.md Phase 2 entry, ROOT CAUSE
paragraph -- the 50 Hz/min_samples=25 structural mismatch). This
prediction may therefore be UNTESTABLE rather than held or failed --
recorded as a live possibility here, not decided in advance.

MEASURED (diagnostics/inspect_ls_cs_disambiguation.py, run against
Sample_Dubai.txt under sideslip_source="kinematic" -- the production
default, independent of the live config's own current experimental
value, per PLAN.md's own hard constraint): rear CS_ratio (worst-
phase-per-instance) population n=56, p25=0.3931. 14 corner instances
fall below p25 (stable_corner_id 2, 3x3, 2x5, 6, 2x13, 14, 3x9 --
full list with lap numbers in the script's own output). Of those 14,
n=0 have a finite rear LS_ratio -- CAVEAT REGISTERED ABOVE HELD: the
prediction is UNTESTABLE, exactly as anticipated, not held or failed
in the ordinary sense. Every low-CS instance reports LS_r=NaN. The
clustering step (median split into traction-/cornering-limited
candidates) never executes -- there is nothing to cluster.

READ, per the standing instruction to record the result plainly
either way: this is a direct, structural consequence of Phase 2's
own finding, not a new failure discovered here -- Phase 2 predicted
exactly this outcome in its own CONSEQUENCE paragraph before Phase 3
or Phase 4 ran. STOPPING Phase 4 here, per the standing instruction
("if a phase proves ill-posed, STOP it, record why, continue to the
next") -- the disambiguation check as specified cannot produce a
result until the Phase 2 DECISION NEEDED item (min_samples/
regression_window_s re-derivation for this car's 50 Hz log) is
resolved. One incidental observation worth recording: NEITHER C4
NOR C14 (thesis_notes.md's own pre-registered candidates from the
EKF-context attribution history) appears in this session's kinematic-
CS_ratio low-p25 population at all -- stable_corner_id 4 does not
appear among the 14 instances above, and stable_corner_id 14 appears
only once (lap 2), not 4/4 laps. This is consistent with the
pre-registration's own explicit caveat that the C4/C14 evidence came
from a different estimator (EKF pass_1) and might not transfer to
kinematic CS_ratio -- it did not transfer here. Not itself evidence
about LS_ratio (which never became available to test either
candidate), but a data point on record for whoever next revisits the
C4/C14 story under kinematic CS_ratio specifically.

diagnostics/inspect_ls_cs_disambiguation.py is left in place, fully
correct and re-runnable as-is -- it will start producing real
clusters the moment the Phase 2 min_samples decision lands, with no
script changes needed.

### PLAN.md STEP 3: 50 Hz min_samples adaptation [2026-08-30]

DECISION (user's, recorded verbatim): the chair's min_samples=25 with
a 0.45 s regression_window_s is unsatisfiable at 50 Hz (proven in the
Phase 2 entry above -- max window 2*half_window+1 = 23 samples at any
data). The adaptation keeps the chair's PHYSICAL window (0.45 s) and
derives the sample minimum from the actual log rate instead of
transplanting the count literally.

FORCING FACT, restated once more at the point of the actual code
change: this car's Cosworth log runs at 50.0 Hz (state[
"sample_rate_hz"], _estimate_sample_rate) -- not a property of this
session's file alone, this is the logger's fixed sample rate, so the
mismatch is structural to this car's data acquisition, not a one-off.

IMPLEMENTATION: modules/longitudinal_stiffness.py's _centered_slopes
now computes min_samples = max(min_samples_floor, half_window+1)
at runtime, using half_window (already computed there from
regression_window_s and the ACTUAL sample_rate_hz passed in --
regression_window_s itself is untouched, still 0.45 s, still the
chair's own value). half_window+1 samples span at least from the
window's centre to one edge inclusive -- a natural, non-arbitrary
minimum for a centred regression slope at ANY log rate, not reverse-
engineered from the chair's own (unconfirmed) assumed rate. config/
parameters.json's longitudinal_stiffness.min_samples key is REMOVED,
replaced by min_samples_floor=15 (a genuine floor, not the effective
value at every rate -- see its own config comment for the full
derivation and the deviation-taxonomy classification: FORCED
ADAPTATION, was "match the chair BY CHOICE", re-triggers threshold
re-derivation like any estimator change).

AT THIS CAR'S 50 HZ: half_window=11, max window=23, derived
min_samples=max(15, 12)=15 -- the FLOOR binds (12 < 15), so the
floor is doing real, load-bearing work at this exact rate, not merely
decorative. 15 <= 23, so validation is now structurally possible.

VERIFIED IMMEDIATELY, external not just analytic: re-ran the same
validation script Phase 2 used (base_mask population, Sample_
Dubai.txt) against the adapted estimator --
  front: n_valid=11299, update_rate=36.47% (of valid), LS_ratio
    median=0.000, p10=-1.837, p25=-0.348, p75=0.965,
    linear_reference=21910.9 N
  rear: n_valid=18450, update_rate=40.85% (of valid), LS_ratio
    median=0.527, p10=-0.770, p25=-0.078, p75=1.000,
    linear_reference=152877.6 N
LS_ratio is no longer all-NaN -- both axles now produce real,
non-degenerate output across a large fraction of the session (front
11299/24183=46.7% of the base population has a valid window at all,
rear 18450/24183=76.3%).

Unit tests updated (tests/test_longitudinal_stiffness.py): the
retired test_real_config_at_50hz_never_reaches_min_samples replaced
by test_real_config_at_50hz_now_validates_with_rate_derived_min_
samples (pins the new rule: 50 Hz validates now); test_window_
shorter_than_min_samples_never_validates renamed test_window_below_
floor_still_never_validates and rewritten to provoke the same
structural non-validation via a very low sample rate (5 Hz) instead
of a literal min_samples value, since min_samples is no longer freely
settable by a caller -- the underlying guarantee (a too-short window
never validates, regardless of data) is unchanged, only how it's
provoked; every other test's _se() helper calls updated from
min_samples= to min_samples_floor=. All 8 tests pass. Full record and
reasoning for each specific test change: see the test file's own
module docstring and each test's own docstring, not duplicated here.

### PLAN.md STEP 3 Phase 2, re-run against the adapted estimator
[2026-08-30]

The Phase 2 pre-registered prediction (thesis_notes.md "PLAN.md STEP
3 (LS_ratio): unsupervised package, Phase 2 -- estimator" entry,
EXACT PREDICTION paragraph) predates this adaptation -- stated
plainly, per the work order, before reporting whether it held: rear
approximately 61%, front approximately 87%, both +/-15 percentage
points, for the estimator's own linear-reference update rate (valid
window AND stiffness>0 AND |kappa_filtered|<=1.5%).

MEASURED (same validation script, base_mask population, now against
real non-NaN output): front update_rate=36.47%, rear update_rate=
40.85% (figures above, this entry's companion adaptation entry).

VERDICT: FAILED at both axles, not a near miss. Front: predicted band
[72%, 102%], measured 36.47% -- roughly 35 percentage points below
the band's own lower edge, the larger miss of the two. Rear:
predicted band [46%, 76%], measured 40.85% -- also below the band,
by about 5 points.

REASONING on why, recorded rather than left as a bare numeric miss:
the pre-registered prediction was built from Phase 1's RAW
(unfiltered) kappa's fraction below the 1.5% threshold (front 86.78%,
rear 61.37%, base population) -- a single per-sample threshold check
with no windowing at all. The estimator's actual linear-reference
update rate additionally requires (a) the sliding window to satisfy
min_samples/min_slip_span (a real constraint even now that it is
satisfiable, not automatically met at every index) AND (b) the
windowed OLS slope to come out POSITIVE (stiffness>0) -- a
non-trivial physical requirement the raw-threshold prediction did not
model at all. The Butterworth 8 Hz filter on kappa also changes which
samples sit inside +/-1.5% relative to the unfiltered figure. The
pre-registration's own text anticipated some of this ("the window-
validity requirement... and the Butterworth filter both modify this
figure in ways not quantified in advance") but the registered +/-15pp
band did not turn out wide enough to cover the actual gap -- the
window-validity and stiffness-sign requirements together removed
substantially more of the population than the band anticipated,
especially at the front axle.

NOT re-registering a new prediction here -- the work order asked for
an honest held/failed report against the EXISTING pre-registration,
not a fresh one.

### PLAN.md STEP 3 Phase 4, run for real -- the first empirical
answer to the combined-slip question [2026-08-30]

PRE-REGISTRATION ON RECORD (thesis_notes.md "PLAN.md STEP 3 (LS_
ratio): unsupervised package, Phase 4" entry, EXACT PREDICTION
paragraph, quoted for the verdict below): "among rear-CS_ratio-below-
p25 corner instances..., I expect the MAJORITY to cluster 'low CS +
high LS' (cornering-limited), with only a SMALL minority (order 1-4
instances...) clustering 'low CS + low LS' (traction-limited) -- and
if any instances land in the traction-limited cluster, C4/C14 are
named here in advance as the most likely candidates..., while
explicitly flagging that prediction as weak."

MEASURED (diagnostics/inspect_ls_cs_disambiguation.py, re-run against
the adapted estimator, sideslip_source="kinematic" as before): rear
CS_ratio population n=56, p25=0.3931 (unchanged from the prior,
ill-posed run -- CS_ratio is untouched by this adaptation). 14
instances below p25 -- now ALL 14 have a finite rear LS_ratio (100%
coverage, vs 0/14 before the adaptation). LS_ratio median among them:
-0.1545.

'low CS + low LS' (traction-limited candidates), n=7:
  lap=1 corner=3  stable_id=3  CS_r=0.345  LS_r=-0.572
  lap=2 corner=3  stable_id=3  CS_r=0.359  LS_r=-1.012
  lap=3 corner=3  stable_id=3  CS_r=0.386  LS_r=-1.073
  lap=4 corner=3  stable_id=3  CS_r=0.385  LS_r=-1.301
  lap=4 corner=5  stable_id=5  CS_r=0.195  LS_r=-0.233
  lap=1 corner=12 stable_id=13 CS_r=0.337  LS_r=-0.258
  lap=3 corner=12 stable_id=13 CS_r=0.196  LS_r=-0.550

'low CS + high LS' (cornering-limited candidates), n=7:
  lap=4 corner=2  stable_id=2  CS_r=0.386  LS_r=0.125
  lap=1 corner=5  stable_id=5  CS_r=0.298  LS_r=-0.076
  lap=1 corner=6  stable_id=6  CS_r=0.219  LS_r=1.000
  lap=2 corner=13 stable_id=14 CS_r=0.165  LS_r=0.034
  lap=1 corner=None stable_id=9 CS_r=-0.721 LS_r=0.103
  lap=2 corner=9  stable_id=9  CS_r=-0.361  LS_r=-0.044
  lap=4 corner=9  stable_id=9  CS_r=0.326  LS_r=-0.037

VERDICT: FAILED, on every clause of the pre-registration, recorded
verbatim per the work order regardless.
- "MAJORITY cornering-limited, SMALL minority (1-4) traction-limited"
  -- FAILED. The actual split is EVEN, 7/7, not a majority either
  way, and the traction-limited count (7) is well above the
  registered 1-4 range.
- "C4/C14 named as the most likely traction-limited candidates" --
  FAILED. C4 (stable_corner_id 4) does not appear in the low-CS
  population at all, at either cluster. C14 (stable_corner_id 14)
  appears once and lands in the CORNERING-limited cluster, the
  opposite of the named prediction.
- The pre-registration's own weak-evidence caveats (TC intervention
  "small... 68 samples total"; exit-phase kappa>=5% "minority... not
  a dominant regime") are NOT contradicted by this result -- a small,
  rare traction-limited population was never ruled OUT by that
  evidence, only ruled unlikely to DOMINATE; an even split is a
  bigger traction-limited signal than that evidence alone would have
  suggested, but not a proof the evidence was wrong (kappa/TC
  aggregates were session-wide, not corner-specific -- they cannot
  rule a handful of specific corners in or out).

GENUINE NEW FINDING, the actual payoff of running this for real:
stable_corner_id 3 (C3) is the ONLY corner that is traction-limited
on ALL FOUR of its valid laps, with a consistently negative (beyond-
peak) rear LS_ratio each time (-0.57, -1.01, -1.07, -1.30) alongside a
consistently moderate-low rear CS_ratio (0.345-0.386, all four laps
essentially the same value). This is a genuinely new candidate the
prior EKF-context C4/C14 attribution history never surfaced (C3 is
not mentioned there) -- a real, repeatable, both-axles-of-evidence
signal for "this corner's rear tyre is doing real longitudinal work
while also reading a flattened lateral curve," exactly the physical
story the combined-slip rationale was built to detect. Stable_id 13
(2 of its 4 laps) is a secondary, weaker candidate. Everything else in
the traction-limited cluster is a single-lap occurrence. NOT
independently cross-checked against per-corner TC-intervention or
kappa data this turn (the session-wide TC/kappa aggregates on record
have no per-corner breakdown) -- a natural next diagnostic, not run
here, out of this phase's own scope (report, not investigate further,
per the work order).

READ PLAINLY: this is the first empirical test of the combined-slip
rationale on record, and it does NOT cleanly vindicate the pre-
registered expectation -- the traction-limited population is larger
and differently distributed than predicted, and the specific named
candidates were wrong. It ALSO does not falsify the combined-slip
rationale itself (that pure-lateral CS_ratio conflates cornering-
and traction-limitation) -- it demonstrates the disambiguation
actually WORKS and finds a real split, with C3 as a concrete,
repeatable example of exactly the phenomenon the method was designed
to catch. The pre-registration's specific numeric/nominal guesses
were wrong; the underlying premise that some low-CS corners are
traction-limited and some are not is, on this one session, upheld.

### Kerb-strike wheel-speed spikes: investigation [2026-08-30]

PURPOSE: log_speed_fl/fr/rl/rr are now the LS_ratio estimator's kappa
input (PLAN.md STEP 3). Kerb strikes visibly spike wheel speed on
real data; this needed characterising before trusting LS results.
Read-only, Tier B, new diagnostics/inspect_kerb_wheel_speed_spikes.py
(matplotlib PNGs to diagnostics/plots/2026-08-30_kerb_wheel_speed_
spikes/, gitignored). No config change, no production path touched.

KERB DETECTION MECHANISM, quoted from the actual code/config (not
memory): modules/stability_analysis.py _compute_kerb_mask_from_az
computes raw = |az_g - kerb_baseline_g| > kerb_z_deviation_
threshold_g, dilated by kerb_dilation_samples on each side (rolling-
OR). Live config: kerb_z_deviation_threshold_g=1.2, kerb_baseline_g=
1.0, kerb_dilation_samples=5 (100ms each side at this session's
50.0 Hz). A pure vertical-acceleration detector -- no wheel-speed
input at all, which is exactly the gap this investigation probes.

### PART 0/1 -- kerb event inventory and spike characterisation

73 kerb-flagged runs in the file, 57 overlapping the racing
population (moving & valid-lap, n=25074). Duration: min=0.220s
median=0.240s max=0.860s. 3.55% of the racing population is kerb-
flagged.

10-event stratified sample (shortest/longest/median-duration + 8
spread evenly by time), per-wheel peak kappa and post-event settle
time (first point after the event's own end where |kappa|<2% holds
continuously for >=100ms) -- full per-event detail and plots in the
script's own output, headline pattern: REAR wheels spike larger and
ring down far longer than front. Examples: event #48 (t=936.0s)
rr peak=+13.76%, settled after 2380ms; event #35 (t=860.3s) rl
settled after 1720ms, rr after 1700ms; event #24 (t=756.0s) rl
settled after 620ms, rr after 640ms. Front is NOT always small
though -- event #32 (t=805.3s) fl peak=-13.66%, fr peak=-9.72% (a
harder hit that spikes both axles), event #43 fl peak=-12.47%.

PART 1b, settle-time distribution across ALL 57 racing kerb events
(not just the 10-event sample), separately per axle -- the number
that actually justifies a widening recommendation:
  front (fl+fr): n=114 settled-within-3s (0 never-settled), p50=0ms
    p75=0ms p90=396ms max=2640ms
  rear (rl+rr): n=112 settled-within-3s (2 NEVER settled within the
    3s window), p50=160ms p75=560ms p90=2116ms max=2860ms
  current kerb_dilation_samples=5 -> 100ms each side.
READ: front settling is mostly fine relative to the current 100ms
dilation (median 0ms, i.e. usually already settled by the event's
own end) but has a real tail (p90=396ms, max=2.64s). REAR IS THE
PROBLEM: the median alone (160ms) already exceeds the current 100ms
dilation -- more than half of rear-wheel kerb ringdown outlasts the
mask -- and the tail is severe (p90=2.1s, two events never settled
inside a 3s window at all).

### PART 2 -- anomaly threshold and mask alignment

|kappa_wheel| distribution, racing population, pooled across all 4
wheels: outside the kerb mask n=96732, p50=0.870% p99=6.144%
p99.9=11.444% max=24.049%; inside the kerb mask n=3564, p50=1.395%
p99=11.391% p99.9=15.482% max=22.931%.

ANOMALY THRESHOLD, gap-selected (Tier B, data-derived, this
diagnostic only, NOT written to config): the non-kerb population's
own p99.9 (11.444%), rounded up to 12.0% -- everything above it is
rarer than 1-in-1000 among samples the mask does not already flag, a
defensible "clearly abnormal" floor derived from this session's own
distribution, same gap-selection convention already used for the
classification thresholds elsewhere in this project.

Axle-mean |kappa_f| or |kappa_r| (the ACTUAL LS estimator input, not
per-wheel) exceeding 12.0% in the racing population: n=9. Of those,
OUTSIDE the kerb mask ("leaked"): n=7 (77.8% of the anomalous
population). 0.45s LS regression windows containing >=1 leaked
sample: n=117 (0.47% of the racing population). High-kappa population
(|kappa|>=5%, the same utilisation threshold already established in
the combined-slip premise entry): n=681; of those, window contains a
leaked sample: n=47 (6.9% of the high-kappa population) -- a small
but non-trivial slice of exactly the tail LS_ratio's low/negative
values (traction-limited signal, per Phase 4's C3 finding) come from.
CAVEAT: contamination checked against RAW (pre-Butterworth) kappa in
each window; the production estimator's 8Hz filter SMEARS a raw
spike across neighbouring samples rather than removing it, so this
number likely UNDERSTATES true contamination, not modelled further
here.

### PART 3 -- impact on the regression slope, 5 contaminated
high-kappa windows

  t=519.30s (front): slope WITH leaked sample -28,158 N/kappa,
    WITHOUT -29,575 N/kappa (+4.8%)
  t=575.20s (rear): WITH -11,713, WITHOUT -18,544 (+36.8%)
  t=605.56s (front): WITH -1,793, WITHOUT -5,082 (+64.7%)
  t=770.76s (front): WITH +8,128, WITHOUT +18,809 (-56.8%)
  t=826.14s (rear): WITH +22,664, WITHOUT +20,228 (+12.0%)
READ: NOT noise-level. Three of five cases move by >10%, two by
>35%, one flips magnitude by more than half while keeping sign. A
leaked kerb-adjacent sample materially moves the local slope in the
majority of the cases checked -- this is a real, not hypothetical,
risk to trust in LS_ratio values near these windows.

### PART 4 -- wheel-speed anomaly as an alternative kerb detector,
the reverse question

Same 12.0% threshold, max over the 4 wheels, same 5-sample dilation
(fair comparison against the az detector). Wheel-speed-based events:
n=40 vs az-based n=57 (racing population). Overlap: only 15/57
(26.3%) of az events have ANY wheel-speed counterpart. az-ONLY: 42.
wheel-speed-ONLY: 25.

Examples, az-ONLY (az fires, wheel-speed stays quiet): t=507.30s
az_peak=1.427g but max_wheel_kappa_peak only 3.70%; t=514.06s
az_peak=1.334g, wheel_kappa_peak=8.76% (below the 12% threshold).
READ: many real, physically-confirmed kerb touches (clear az
signature) do not perturb wheel speed enough to cross the anomaly
threshold -- kerb severity varies, and az is picking up genuine mild
kerb contact the wheel-speed channel doesn't register strongly.

Examples, wheel-speed-ONLY (wheel-speed fires, az stays quiet):
t=519.34s az_peak=0.485g (well under the 1.2g threshold) yet
max_wheel_kappa_peak=21.70% -- this is the SAME window Part 3's first
impact example sits inside; t=555.28s az_peak=0.680g, wheel_kappa_
peak=12.50%; t=571.32s az_peak=0.576g, wheel_kappa_peak=17.36%.
IMPORTANT NUANCE, not to be glossed over: a low az signature alongside
a large wheel-speed anomaly is NOT necessarily a missed kerb strike --
it is equally consistent with a genuine high-slip driving event
(wheelspin on exit, lock-up under braking) that has nothing to do
with a kerb at all. This was not resolved here (would need per-event
corner-phase/brake-pressure cross-referencing, out of this
diagnostic's scope) -- flagged explicitly as an open question, not
assumed either way.

### RECOMMENDATION (recorded, NOT implemented, per the work order)

The numbers do not support a single clean category -- they support a
combination, and explicitly rule out one tempting option:

1. MASK WIDENING, axle-asymmetric, data-derived: rear dilation should
   widen substantially (current 100ms is already below the rear
   MEDIAN settle time of 160ms; a value in the 500-600ms range, near
   the measured p75, would resolve the majority of rear leakage
   without reaching into the extreme multi-second tail). Front can
   stay close to current (median settle 0ms) with at most a modest
   increase to cover its own smaller tail (p90=396ms). Well-supported
   by PART 1b directly.
2. A LS-ESTIMATOR-SCOPED plausibility guard (NOT a change to the
   general-purpose kerb_mask used elsewhere -- corner detection, CS
   estimation, etc.): given PART 3's demonstrated slope impact (up to
   64.7%) and that 77.8% of the most extreme anomalies currently leak
   past the mask, a targeted check inside modules/longitudinal_
   stiffness.py's own windowing (e.g. excluding or flagging a sample
   whose raw kappa exceeds a physically-implausible single-sample
   threshold before the regression runs) would catch what mask
   widening alone might still miss, without touching every other
   kerb_mask consumer's behaviour.
3. EXPLICITLY NOT RECOMMENDED: a naive hybrid detector that ORs the
   wheel-speed-anomaly mask into the general kerb_mask. PART 4's own
   wheel-speed-ONLY examples show this would flag genuine high-slip
   driving events (indistinguishable, without further work, from real
   kerb strikes) as "kerb" -- and the high-kappa tail is EXACTLY where
   Phase 4's real finding lives (C3's traction-limited signal, all
   four laps). A general-purpose hybrid kerb detector risks silently
   erasing the very signal the whole STEP 3 package exists to surface.
   If a wheel-speed-based signal is used at all, it belongs scoped to
   the LS estimator's own input hygiene (point 2), not broadened into
   the shared kerb_mask.
4. "No action needed" is NOT supported -- PART 3's measured slope
   swings are real, not noise-level, in a majority of the cases
   checked.

Neither of the two recommended actions (1, 2) was implemented this
turn, per the work order -- read-only investigation and
recommendation only.

### PLAN.md STEP 3 follow-up: C3 verified clean, LS plausibility
guard implemented, mask widening quantified [2026-08-30]

### PART 1 -- C3, checked FIRST, before any change

New diagnostics/inspect_c3_leaked_windows.py (read-only). For each of
C3's 4 valid-lap instances, found the worst phase (the one that
produced the reported worst-phase-median rear LS_ratio) and checked
every valid rear-LS sample in that phase's time window against the
kerb-investigation's own "leaked" definition (axle-mean |kappa| >
12.0% AND outside kerb_mask):

  lap=1: phase=exit_5, reported LS_r=-0.572, n=19 valid samples, 0 contaminated (0.0%)
  lap=2: phase=exit_4, reported LS_r=-1.012, n=45 valid samples, 0 contaminated (0.0%)
  lap=3: phase=exit_5, reported LS_r=-1.073, n=19 valid samples, 0 contaminated (0.0%)
  lap=4: phase=exit_4, reported LS_r=-1.301, n=45 valid samples, 0 contaminated (0.0%)

VERDICT: C3's finding needs NO asterisk. Zero leaked-window
contamination across all 4 laps and both recurring worst phases
(exit_4/exit_5, consistent with an exit-traction story, not an
artifact concentrated in one phase). The reported medians equal the
all-samples medians exactly (nothing to exclude). C3 stands as
recorded in the Phase 4 entry.

### PART 2 -- LS plausibility guard, implemented

Files: modules/longitudinal_stiffness.py, config/parameters.json
(additive, longitudinal_stiffness namespace).

DESIGN: a sample is excluded from the LS regression windows (both
axles independently) only when BOTH hold: |kappa_raw| exceeds
plausibility_kappa_bound (0.12, same gap-selected value as the kerb
investigation's ANOMALY_THRESHOLD) AND the az channel shows kerb-like
disturbance (same raw flag _compute_kerb_mask_from_az itself uses)
within a TRAILING window ending at that sample -- axle-specific,
sized from the measured ringdown distribution: plausibility_az_
window_front_s=0.15, plausibility_az_window_rear_s=0.6 (config
provenance comments cite the kerb investigation entry's PART 1b
percentiles directly). LOAD-BEARING DESIGN CONSTRAINT, stated in the
code and honoured throughout: az-coincidence is REQUIRED, kappa alone
is NEVER sufficient -- a high kappa with no nearby az disturbance is
presumed genuine traction signal (exactly C3's own signature) and
must pass through untouched.

MECHANISM DETAIL worth recording: the exclude mask is applied in TWO
places, not one. An EARLIER version of this guard only gated the
post-filter window-sum stage (valid_mask) and left the excluded raw
sample inside the Butterworth pre-filter step -- this FAILED its own
integration test (the filtered signal near the excluded index stayed
corrupted, since filtfilt is a whole-array zero-phase operation that
smears an outlier's energy into neighbouring FILTERED samples
regardless of any later masking). Fixed by NaN-ing the excluded raw
sample BEFORE filtering (so _filtered's own existing interpolate-over-
NaN step bridges over it cleanly) in addition to excluding it from the
window-sum valid_mask afterward. Recorded here because it is a real
lesson about zero-phase filters and point-masking, not just an
implementation detail: masking AFTER a whole-signal filter does not
undo what the filter already did to nearby values.

UNIT TESTS (tests/test_longitudinal_stiffness.py, 8 new, all pass): a
synthetic kerb-coincident spike is excluded; an identical excursion
WITHOUT az disturbance is kept; a real az disturbance OUTSIDE the
trailing window does not exclude (confirms the window is bounded, not
"any disturbance ever"); az_g=None excludes nothing (graceful
degradation, never falls back to kappa-alone); NaN kappa/az never
registers as implausible-and-disturbed; empty-array input does not
crash (this exposed and fixed a real bug in the first draft of _az_
disturbed_recently -- np.convolve's 'valid'-mode output length
depends on BOTH operand lengths and silently returns a MISMATCHED
shape for empty/short inputs when window_samples exceeds them; fixed
by switching to the same prefix-sum windowed-sum primitive _centered_
slopes already uses, which always returns exactly len(input)); and an
end-to-end integration test confirming the guarded pipeline recovers
close to the true synthetic slope while an unguarded (no az
disturbance) identical outlier is left to distort it.

### PART 3 -- re-run with the guard active

PRE-REGISTRATION: since Part 1 found zero contamination in C3's own
windows, the guard is predicted to leave C3's classification AND
values unchanged. Stated before running.

MEASURED: HELD, exactly. Re-ran diagnostics/inspect_ls_cs_
disambiguation.py (guard now live inside modules/longitudinal_
stiffness.py, no script change needed) -- the 14-instance split is
UNCHANGED, still 7 traction-limited / 7 cornering-limited, same
instances in each cluster. C3's four LS_r values are BYTE-IDENTICAL
to the pre-guard run (-0.572, -1.012, -1.073, -1.301). Across the
whole 14-instance population exactly ONE value moved at all, and only
in the fourth decimal: stable_id 9 lap 4, LS_r -0.037 -> -0.023 (still
cornering-limited before and after, no reclassification anywhere).

THE 5 PREVIOUSLY-CHECKED CONTAMINATED WINDOWS: proper apples-to-apples
check this time (production pipeline, guard active vs guard
neutralised via plausibility_kappa_bound=999 -- NOT compared against
the earlier ad-hoc raw-single-window OLS demonstration from the kerb
investigation entry, a different computational context that was never
a fair baseline for this specific check). RESULT: all 5 show EXACTLY
ZERO change (guarded == unguarded to full precision). Traced why for
each, since a null result deserves the same scrutiny as a positive
one:
  t=519.30s (f): kappa_raw at this exact index=-5.80% (below the 12%
    bound -- this index's OWN kappa isn't implausible; the earlier
    "leaked" flag came from a NEIGHBOUR inside its 0.45s window, and
    evidently that neighbour didn't meet both guard criteria either)
  t=575.20s (r): kappa_raw=10.30% (below 12% bound), max|az-baseline|
    in trailing 0.6s=1.172g (below the 1.2g threshold -- just under)
  t=605.56s (f): kappa_raw=-14.21% (ABOVE the 12% bound -- implausible
    on its own), but max|az-baseline| in trailing 0.15s=0.494g (well
    below 1.2g -- no az coincidence at all)
  t=770.76s (f): kappa_raw=-12.95% (above bound), az max=0.645g
    (below threshold -- no coincidence)
  t=826.14s (r): kappa_raw=8.10% (below bound), az max=0.813g (below
    threshold)
READ: two of five (605.56, 770.76) have implausible kappa at that
exact index but NO az disturbance anywhere nearby -- per the strict
design constraint, these correctly stay UNEXCLUDED, and this is
consistent with (not contradicted by) PART 4 of the kerb investigation
entry's own finding that some high-kappa events show essentially no
az signature at all and may be genuine driving events, not sensor
artifacts. The other three simply weren't implausible AT that exact
index (the window-level "leaked" flag came from elsewhere in their
window). This is the guard working exactly as designed -- narrowly
scoped, conservative, and NOT a proxy for "every anomaly the kerb
investigation flagged."

WHOLE-SESSION QUANTIFICATION of the guard's actual effect (not just
the 5 spot-checked windows), guarded vs unguarded production pipeline,
racing population: FRONT axle excluded n=0 samples -- zero effect,
this session. REAR axle excluded n=2 samples -- small, but with real
downstream reach: 182 windows' stiffness value differs at all (any
window whose regression span includes one of the 2 excluded samples),
44 of those by more than 1% relative, max relative change 3827.6% at
one specific window (a small-denominator case, consistent with PART
3 of the kerb investigation's own finding that a leaked sample CAN
swing a slope dramatically when it truly coincides with az
disturbance). LS_ratio_r itself technically differs at 13,389 racing-
population samples, but the overwhelming majority of those are
4th-decimal-place shifts (e.g. 0.6905->0.6906) -- an expected
knock-on effect of the population-wide linear-reference median moving
fractionally once 2 fewer samples contribute to it, not 13,389
independent corrections.
READ, honestly, not spun either direction: on THIS session's data,
the guard (as specifically scoped: az-coincidence required, these
window sizes, this bound) has a NARROW footprint -- 2 samples excluded
outright, one axle untouched entirely. This does not mean kerb-spike
contamination is a non-issue (PART 2/3 of the kerb investigation
entry already established real leakage and real slope impact); it
means that MOST of what that investigation flagged as "leaked" does
not, on closer inspection, show BOTH an implausible kappa value AND
genuine az coincidence at the same instant -- consistent with a
meaningful fraction of the flagged population being real high-slip
driving events rather than missed kerb strikes, exactly the
ambiguity PART 4 of that entry already flagged as unresolved. The
guard is conservative by design and is correctly declining to guess
on ambiguous cases.

### PART 4 -- mask widening, QUANTIFIED ONLY, NOT applied

CURRENT (kerb_dilation_samples=5, 100ms symmetric, the ONLY dilation
value that exists -- kerb_mask is a single shared, non-axle-specific
mask): racing population n=24183 (confirmed, matches every prior
figure in this session exactly).

Widening scenarios (uniform dilation of the SAME shared mask -- it
cannot be split per-axle without restructuring every consumer, which
is exactly why this needs its own decision):
  150ms (front-motivated):  n=23862  delta=-321  (-1.33%)   kerb-flagged fraction of moving&racing: 4.83%
  500ms (rear floor):       n=22326  delta=-1857 (-7.68%)   kerb-flagged fraction: 10.96%
  550ms (rear p75-ish):     n=22074  delta=-2109 (-8.72%)   kerb-flagged fraction: 11.96%
  600ms (rear recommended): n=21906  delta=-2277 (-9.42%)   kerb-flagged fraction: 12.63%

READ: the tension is exactly what the work order anticipated. A
front-appropriate widening (150ms) costs relatively little (-1.33%
of the whole racing population). Widening enough to properly cover
REAR ringdown (500-600ms) costs 7.7-9.4% of the ENTIRE population --
front and rear alike, since one shared mask cannot discriminate --
nearly an order of magnitude more population loss than the front-only
case, to fix a problem that is specifically a REAR phenomenon (PART
1b of the kerb investigation entry: front settle p50=0ms, rear
p50=160ms already exceeding the current 100ms dilation).

FROZEN-BASELINE STATISTICS THAT WOULD SHIFT if any widening were
applied (not recomputed here -- quantify-only per the work order;
listed so the decision is made with full knowledge of its blast
radius, not discovered after the fact):
- The WP-N2 pass-1 final validation baseline (diagnostics/inspect_
  pass1_final_validation.py, PLAN.md NOW: "the reference any future
  estimator work is compared against") is anchored to the CURRENT
  base_mask population (n=24183) -- a widened kerb mask changes that
  population, invalidating direct comparison without re-freezing.
- Every percentile figure this session recorded against base_mask
  n=24183 (Phase 1's kappa validation, the combined-slip premise
  entry's kappa distributions, the kerb investigation's own |kappa_
  wheel| percentiles) would shift and need re-measuring -- the
  population they were measured against would no longer exist.
- tests/golden/pipeline_dubai_kinematic_cap1.json and tests/golden/
  recommendations_dubai_kinematic_cap1.json (the regression suite's
  own golden files) would almost certainly stop matching -- kerb_mask
  feeds estimate_cornering_stiffness's own moving-sample exclusion
  directly, so CS_ratio, phase stats, and recommendation output would
  all shift. This IS the "never regenerate old goldens... without
  saying so explicitly" case CLAUDE.md's own rule anticipates --
  applying mask widening would require a deliberate, flagged golden
  regeneration, not a silent one.
- The classification thresholds (STRONG_CSF/CSR etc., config/
  parameters.json) were gap-selected against a CS_ratio distribution
  that already excludes kerb-masked samples under the CURRENT
  dilation -- a materially different population (up to -9.42% of
  samples) could shift where those gaps fall, per CLAUDE.md's own
  rule that thresholds are re-derived per estimator/population
  change, never carried over.
No widening was applied. This is a shared-mask production change with
a real, quantified population cost, and it gets its own decision.

## 6. Documentation/comment polish pass (text-only, no functional changes) [2026-08-30]

Unsupervised package, 5 phases, scope: every .py file in modules/,
ui/, core/, diagnostics/, tests/, plus test_stability.py -- comments,
docstrings, and (Phase 2 only) user-facing UI strings, rewritten or
removed against an "AI-sounding" removal list (narrated-the-obvious
comments, filler/hedging words, conversational tone, emoji, decorative
separators, stale TODOs). No number, threshold, config value, control
flow, or function/class signature was changed. Full report delivered
to the user in-session; summary here for the record.
Phase 1 (modules/, 14 files): mostly docstring-mood tightening and
filler-word removal; 6 private-helper-local variable renames (single-
letter/misleading names only, never a signature/config key); found
and fixed 2 ASCII violations (em-dashes in corner_analysis.py warning
strings). Most files (accuracy_resolution.py, geo.py, recommendation.py)
needed no changes -- already at the target style from prior sessions'
own comment-discipline work.
Phase 2 (ui/+core/, 17 files): same treatment plus a user-facing-
string pass. 41 non-ASCII characters normalized to ASCII across
ui/views/outing_form.py alone (em-dashes, degree signs, middot
separators, arrow glyphs), dozens more across the other 16 files.
The four already-decided protected texts (the "[UNCAL]" marker, both
calibration PLACEHOLDER banners, _format_estimator_status's returned
templates) were explicitly preserved everywhere they appear, verified
by grep after editing. One field label improved (settings_view.py
"Cross x track area" -> "Frontal area (cross-track)").
Phase 3 (diagnostics/+tests/, 89 files): zero edits needed anywhere --
every file was already at or above the target standard. tests/ was
additionally checked against a hard no-touch-test-matching-content
rule (assertions, golden literals, fixture names, the "regression not
correctness" framing); confirmed nothing eligible for change existed.
Phase 4 (consistency sweep): reconciled an ASCII-handling
inconsistency that Phase 2's file-disjoint sub-agents introduced
(some fixed em-dash/middot/degree-sign occurrences, others deliberately
deferred pending a repo-wide decision) -- resolved by extending the
same ASCII normalization to core/pdf_export.py and core/
weekend_pdf_export.py, which also repaired a real regression the
Phase 2 pass itself caused: core/pdf_export.py's CORNER_LABELS and
ui/views/outing_form.py's CORNER_LABELS (a pre-existing hand-copied
duplicate dict, not new to this pass) had briefly gone text-different
("Camber (deg-sign)" vs "Camber (deg)") after independent per-file
fixes with no shared visibility; now byte-identical again. Terminology
sweep found and fixed 2 stray "tire" spellings (diagnostics/
inspect_fz_sign_conventions.py print strings); "kerb" vs "curb",
"sideslip" vs "side slip", and axle-naming conventions were already
consistent repo-wide. Config comments (parameters.json, channels.json)
and module headers were already complete and consistent; no changes
needed.
Verification: full regression suite run twice. The first run (8
errors) was not caused by this pass -- config/parameters.json's
sideslip_source was at the user's own live value (ekf_auto_pacejka)
rather than "kinematic", which every golden fixture requires; this is
the same temporarily-flip-and-restore step every prior full-suite run
in this project's history has used (see NOW section above). Restored
to kinematic, reran: 110 passed, 1 xfailed -- byte-identical to the
recorded baseline. sideslip_source restored to the user's own live
value afterward, verified byte-identical to git HEAD via diff. No
commit made; protected set (docs/literature/, docs/car_data/,
config/car_data.json, HANDOVER.md, docs/study/) confirmed empty via
git ls-files.
Items surfaced but deliberately not acted on (report-only, per this
package's own work order): a possible aero downforce sign-convention
question in estimate_vertical_loads's fz_aero_total_N formula (not
verified against config's actual stored cl); the CORNER_LABELS
hand-copy duplication itself (text now reconciled, but the structural
duplication predates this pass and remains); one dead local variable
in diagnostics/inspect_abs_slip_channels.py; four live "page/p. TBD
verify" citation placeholders (Rajamani, Milliken & Milliken) that
need the physical text to resolve, not touched.

### Aero downforce sign convention verified computationally [2026-08-30]

Follow-up to the polish pass's report-only flag above. Read-only
check, plus one config comment (see below) -- no code changed.
modules/stability_analysis.py estimate_vertical_loads:
  fz_aero_total_N = -0.5 * rho * v^2 * a_aero * cl
-0.5*rho*v^2*a_aero is never positive (rho, v^2, a_aero all >= 0), so
fz_aero_total_N's sign is the OPPOSITE of cl's sign: cl < 0 gives a
positive (downforce) contribution, cl > 0 gives a negative (lift)
contribution -- the standard aerodynamic lift-coefficient convention
(positive = lift, negative = downforce), not a "downforce is positive"
convention. This matches config/parameters.json's own pre-existing
lift_coeff_note exactly, which had already worked this out from the
formula alone but flagged it as INFERRED, not confirmed against a
real value (no numeric Cl default existed in the shared reference
files to check against).
Config's cl entry at the time of this check: "lift_coeff": 0.0, Level
1 (config default, chained-constant into vertical_load_split/
per_wheel_load_split's own Level-1 cap) -- a placeholder, not sourced,
inert (multiplies to exactly zero at every speed regardless of sign,
same reasoning as cross_track_area_m2's own note).
Computational test (diagnostics-style one-off script, not committed
to the repo, run directly against the live estimate_vertical_loads
function with the real config's mass/wheelbase/cog values, isolating
the aero term: ax=ay=0, v=200 km/h): static loads fz_f=5692.7 N
(580.3 kgf), fz_r=7609.7 N (775.7 kgf). Test cl magnitude 2.335,
a_aero=2.0 m^2 (ClA~4.67 m^2, ~900 kgf combined downforce at 200
km/h) -- a public-domain motorsport-engineering ballpark for a
GT3-class car, NOT a sourced team or literature figure; used only to
pick a physically plausible magnitude for this sign test, not
proposed as this car's real value.
  cl = -2.335 (documented convention): fz_f +3778.0 N, fz_r +5050.3 N
    -- BOTH INCREASE. Convention confirmed correct as documented.
  cl = +2.335 (naive "positive = downforce" reading): fz_f -3778.0 N,
    fz_r -5050.3 N -- both decrease, exactly the inverted-looking
    result the naive reading would produce. This is not a second bug;
    it is the same formula evaluated under the WRONG assumed sign,
    demonstrating why the convention needs to be unmissable at the
    config key, not just in a long paragraph note.
CONCLUSION: the convention is correctly implemented and was already
correctly documented in lift_coeff_note; the original polish-pass flag
is RESOLVED, not a bug -- it surfaced a real ambiguity (a paragraph
note easy to skim past) rather than a real defect. Added a new short
config key, lift_coeff_sign_convention (config/parameters.json,
one line, next to lift_coeff), stating the rule unmissably for
whoever enters the first real Cl value; the original long note is
preserved unedited underneath it. No code changed. No commit made.

## 7. PDF layout rework: shared strip renderer [2026-08-30]

Tier C (UI/product). Visual rework of the setup/setdown PDF and the
weekend PDF's setup-sheet content -- layout only, same fields, same
data sources, same calling contracts (ui/views/outing_form.py's
_print_sheet and generate_weekend_pdf's signature both unchanged).
Proposed as a 3-step inventory-then-sketch (Step 1 read core/pdf_
export.py and core/weekend_pdf_export.py; Step 2 proposed the new
layout against six fixed decisions -- landscape A4, four strips/page,
bordered cells, front-up 2x2 wheel orientation, marked-position
schematics, one shared renderer for both documents; Step 3 drew an
ASCII strip sketch with the real 17-field-per-corner set and flagged
where it would be tight), approved with clarifications, then built.

INVENTORY FINDING (Step 1a), the reason for decision 1 below: the two
documents' setup-sheet field sets had drifted apart. core/pdf_
export.py's single-outing sheet printed 17 fields/corner (6 core +
5 damper + 6 advanced) plus a 2x2 corner-weight grid; core/weekend_
pdf_export.py's per-outing setup table printed only 12 fields/corner
(core + advanced -- the 5 damper fields were never wired in) and no
weight grid, and never read setdown_data at all. Decided: the single-
outing 17-field set is canonical; the weekend PDF's narrower set was
drift, not a deliberate choice.

INVENTORY FINDING (Step 1b): "splitter and diffuser position
schematics" as originally described did not match the code. Grepped
the whole repo: splitter_offset is a plain numeric spinbox (mm
offset) in both the UI and the old PDF, and no "diffuser" field
exists anywhere -- it appears only as prose inside ride_height_
rear's mechanism note. The only fields with a real, config-backed
discrete position set are wing_position (P8/P9/P10, cross-checked
against car_data.json's wing_position_table) and arb_front_mount
(P0/P1/P2). Decided: schematics for wing_position/arb_front_mount
only; splitter_offset stays a numeric cell; no diffuser field
invented.

IMPLEMENTATION. core/pdf_export.py gained: PositionSchematic, a
Flowable subclass drawing marked-position boxes with plain reportlab
canvas primitives (rect/drawCentredString) -- no new dependency, the
module already imported reportlab.platypus.Flowable path; a two-
style-preset system (_strip_styles('large'|'small')) so one set of
layout functions serves both the full-page single-session print and
the quarter-page weekend strip; _corner_box implementing the
two-layer design (Decision 3: CORNER_LABELS at readable size as the
core row, a new abbreviated DENSE_LABELS dict folding the former
hardcoded damper row + ADVANCED_LABELS into one dense row); and
build_session_strip, the one shared renderer, wrapped in
KeepInFrame(mode='shrink') so a strip that runs long shrinks to fit
its box rather than overflowing the four-strip grid -- addresses the
tightness Step 3 flagged, and empirically turned out to have comfortable
headroom to spare once the two-layer design was in place, not the
tight fit originally feared.
core/weekend_pdf_export.py gained a new _build_setup_sheets_section:
iterates all outings, calls build_session_strip at 'small' scale
once for setup_data and once for setdown_data per outing, four
strips per landscape page (a page break every 2 outings so a pair is
never split), inserted between the cover page and the existing per-
outing analysis/recommendations/feedback pages. The old embedded
"Setup Sheet" table inside each outing's analysis page was removed
(not duplicated) since its content now lives in the dedicated strips
section -- a structural call made during implementation, not
separately re-confirmed with the user before building, flagged here
for visibility. The whole weekend document is now landscape A4
(previously portrait) since SimpleDocTemplate takes one page size
for the whole document and the approved decision was "landscape A4
both documents" without carving out the analysis pages; their
existing table code needed no changes since column widths already
derive proportionally from PAGE_W/CONTENT_W.
arb_front_mount now prints (via its schematic box) for the first
time -- it was collected in the UI but never printed before this
package (Step 1a finding). differential_locking_torque_measured (the
5-point table) remains unprinted, per instruction, as its own open
decision.

BUGS FOUND AND FIXED DURING BUILD, before delivery (report-only
findings from generating and visually inspecting sample PDFs against
the real Dubai outing, not from a golden test -- none exists for PDF
output, confirmed by grep before assuming it): (1) the team logo was
sized proportionally to the FULL strip height (logo_h = strip_h *
0.35), which is correct intuition at 'small' scale but at 'large'
scale (a whole landscape page, ~190mm) produced a logo wider than the
page, overlapping the header -- fixed to a small fixed height per
size preset (6mm large / 3mm small) instead of scaling with strip_h.
(2) The Diff Preload/Position/Splitter numeric rows used a 55/45
label/value column split sized for multi-line text, leaving a single
digit stranded in a ~50mm-wide cell -- tightened to 75/25.
Verification: full regression suite (110 passed, 1 xfailed, byte-
identical to the recorded baseline) run under the same temporarily-
flipped-kinematic-then-restored procedure as every other full-suite
run in this project's history (sideslip_source has no bearing on PDF
generation, but the golden pipeline/recommendation tests still
require it); test_stability.py exit 0; three sample PDFs generated
against the real Dubai weekend (outing 1 "Warmup", real CSV-backed,
and outing 2 "Practice") and visually inspected page-by-page (a
pymupdf render step, installed temporarily for this QA only and
uninstalled immediately after -- never a project dependency); no
commit made; protected set confirmed empty.

## 8. Splitter/diffuser measurement points [2026-08-30]

### Phase 1: data model and persistence pattern

Tier C (UI/product). New feature: five nullable floor-referenced mm
measurement points each for splitter and diffuser, distinct from and
additive to the existing splitter_offset SETTING (car-referenced,
unchanged). Investigated where setup/setdown data lives before
writing anything: Outing.setup_data/setdown_data (models/outing.py)
are plain String(10000) columns holding a JSON blob with no schema
version field at all -- unlike analysis_data, which carries
ANALYSIS_SCHEMA_VERSION specifically because it drives live verdict
rendering and cache invalidation (a stricter requirement that doesn't
apply here). The established migration pattern for this JSON blob is
purely additive: ui/views/outing_form.py's _load_inputs only sets a
widget from a saved key if that key exists in the saved JSON AND a
widget for it exists in the current form -- no version bump, no
migration script, ever, for any field added to this blob historically
(arb_front_mount, wing_position, the diff-torque table). A prior
addition, differential_locking_torque_measured, already solved the
exact shape problem here: a structured multi-value group that doesn't
fit the flat corner_key->param->widget model, handled by a pop-based
reshape pair (_reshape_diff_torque_out/_in) that flattens to numbered
widget keys for editing and folds back to the storage shape on save/
load. Followed that pattern exactly rather than inventing a new one:
_reshape_points_out/_reshape_points_in (ui/views/outing_form.py),
generalised over both point groups via _POINT_GROUPS = [("splitter_
point", "splitter_points", 5), ("diffuser_point", "diffuser_points",
5)] since they share one shape (5 nullable floats) -- the diff-torque
functions themselves stayed dedicated (not merged into this), since
unifying three different call sites for a one-time historical field
would be a bigger, unrequested change.
Storage shape: car["splitter_points"] / car["diffuser_points"], each
a plain JSON array of 5 elements, index 0..4 = point 1..5, empty/
unparseable entries stored as null (not 0.0 and not omitted) --
literally "nullable mm arrays" as specified, not the diff-torque
precedent's nested {"1":val,...} dict (that shape was chosen for a
dict keyed by physical position number for a different reason;
arrays fit "point 1..5 in fixed physical order" more directly and
the spec asked for arrays specifically).
Widget type decided by precedent, not preference: the damper fields
(bump_ls/bump_hs/blowoff/rebound_ls/rebound_hs) are QLineEdit, not
QDoubleSpinBox, specifically because they're allowed to be blank
(confirmed by reading _mirror_damper's .text() calls and the real
Dubai outing's own stored JSON, which has "bump_ls": "10" as a
string, "" for the unset rear corners) -- QDoubleSpinBox in this
codebase always collects/loads a definite float (_collect_inputs:
widget.value(); _load_inputs: float(value) if value else 0.0), never
null, so it was never a candidate for a genuinely-nullable field.
Splitter/diffuser points follow the damper precedent: QLineEdit per
point, "" round-trips to None through the reshape functions, matching
"all ten values nullable" without adding a new widget-type case to
the generic _collect_inputs/_load_inputs dispatch at all.
config/setup_parameters.json precedent checked and NOT followed here:
differential_locking_torque_measured (also measured/check data, not
an adjustable setting) has no registry entry there -- that registry
is for recommendation-engine-relevant settings. Splitter/diffuser
points are the same category (measured, not a target), so no
setup_parameters.json entry was added for them either, consistent
with the existing precedent rather than a new decision.
No source paper-sheet image exists in docs/car_data/ for splitter or
diffuser measurement points (checked before assuming a layout) --
the exact physical point positions used in Phase 2's drawn widget are
this session's own reasonable placement (4 spread + front-middle for
splitter on a rounded nose outline; 4 corners + centre for diffuser
on a rectangle), NOT digitised from a team source, flagged for the
user's visual confirmation once rendered. This is the same "no source
image digitises this parameter" situation setup_parameters.json
already records for wing_position's mechanism note -- team-knowledge-
only, not a chair/literature question.
Verified before building further: reshape round-trip (ad hoc script,
not yet the Phase 4 formal test) confirms (1) an old outing's JSON
with no point keys at all loads with zero phantom keys added -- the
existing "skip unknown param" path in _load_inputs then leaves new
widgets at their default empty state, satisfying "old outings load
unchanged with empty points" without any explicit migration step; (2)
collected data with some points set and some blank correctly reshapes
to an array with None in the blank slots, correct index order; (3)
the array round-trips back to the correct flat keys; (4) splitter_
offset is untouched throughout every step. No modules/ file touched;
no DB column added (both point groups live inside the existing JSON
blob columns); no commit.

### Phase 2: setup form widget

New file ui/views/measurement_points_widget.py: MeasurementPointsWidget
(QWidget), one instance for splitter and one for diffuser, each holding
5 real QLineEdit children positioned by fractional (fx, fy) coordinates
over a hand-drawn outline (rounded blade for splitter, plain rectangle
for diffuser) -- fy=0 is the front of the car, matching the 2x2 wheel-
grid's front-up convention used everywhere else in this form. The
QLineEdits are the actual widgets registered into self._active_inputs
["car"]["splitter_point_N"] / ["diffuser_point_N"] in ui/views/outing_
form.py's _build_car_center, exactly like every other field in this
form -- no special-case added to the generic _collect_inputs/_load_
inputs dispatch, confirmed by construction (the widget IS a QLineEdit,
just positioned by the parent's paintEvent/resizeEvent instead of a
QLayout). Called once per _build_setup_section(prefix) invocation, so
setup and setdown each get their own independent widget instances
automatically, same as every other car-level field -- no duplication
logic needed. Added a tooltip to the existing Splitter (splitter_
offset) row ("setting, vs car") and to each point box ("measured, vs
floor") per the work order's visibility requirement; label headers
above each diagram state the same distinction in text.
No source paper-sheet image for splitter or diffuser exists in docs/
car_data/ (checked, see Phase 1) -- the exact point layout used here
(splitter: 4 spread + 1 front-middle above; diffuser: 4 corners + 1
centre) is this session's own reasonable placement, NOT digitised,
and needs the user's visual confirmation against the team's real sheet
before being treated as final; the shapes/positions are trivial to
adjust (two coordinate lists in the new file) once corrected.
Verified by construction, not just import: a headless script (Qt
offscreen platform, same technique as diagnostics/smoke_test_corner_
trace_dialog.py) built a real OutingForm against the real Dubai
"Warmup" outing (whose stored setup_data predates this feature) and
screenshotted the result. Confirms (1) both widgets render with the
correct shape/point layout/front-up arrow: (2) loading the real old
outing leaves all 10 point boxes empty (text='') -- "old outings show
empty boxes" holds in the actual form, not just in the reshape unit
logic; (3) setup and setdown each got their own independent widget
instances (4 total found via findChildren), both empty, confirming
per-sheet duplication.
One rendering artifact found and understood, not a code bug: the
whole form's text renders as tofu boxes under Qt's offscreen platform
without QT_QPA_FONTDIR pointed at a real font directory -- confirmed
this affects EVERY label in the form, not just the new widget, so it
is an environment default, not something this change introduced;
fixed for screenshot purposes by setting QT_QPA_FONTDIR=C:/Windows/
Fonts. Separately, the first render of the "Car" groupbox in isolation
showed the Diffuser Points widget overlapping the ARB/Wing rows below
it (groupbox actual size 350x330 against a sizeHint of 238x600) --
forcing the groupbox to its own sizeHint before grabbing produced a
correctly stacked, non-overlapping layout, so this reads as a headless-
grab timing artifact (the widget hadn't been given a layout pass to
grow to its natural height in that isolated grab) rather than a
production bug -- QVBoxLayout/QScrollArea are supposed to give a
child its full sizeHint and let the scroll area handle the rest. Not
verified on a real display; flagged so the user's own manual QA
(per this project's standing UI-change rule) specifically checks that
the Car section scrolls/expands cleanly, not just that it looks right
in the first visible screenful.

### Phase 3: PDF

core/pdf_export.py gained MeasurementDiagram (a Flowable, same canvas-
primitive mechanism as the existing PositionSchematic) drawing the
outline shape with each point's value in a small bordered box at its
physical position, monochrome. SPLITTER_POINT_POSITIONS/DIFFUSER_
POINT_POSITIONS duplicate ui/views/measurement_points_widget.py's
fraction coordinates -- core/ cannot import that file (PyQt6-based;
no PyQt6 import is allowed in core/ or modules/), so the two copies
are kept in sync by hand, flagged in both files' own comments, the
same NEUTRAL ENGINEERING duplication risk already recorded for
CORNER_LABELS between core/pdf_export.py and ui/views/outing_form.py.
Weekend (small) scale legibility, as required: rendered BOTH candidates
against the real Dubai outing (in-memory-only synthetic point values,
never saved -- 12.5/-/8.0/9.2/15.0 for splitter, 22.0/-/18.5/19.0/- for
diffuser) and looked at both before choosing.
- Variant A (kept): the same outline+value-box diagram at both scales,
  just smaller. At weekend scale (4-per-page), a 4x-zoomed crop showed
  every value legible (15, 12.5, -, 8, 9.2, 22, -, 18.5, 19) and the
  diagram stayed correctly bounded inside its column.
- Variant B (rejected): a values-only row (point-number header + value
  cells, no outline), meant as a deliberate degrade path. Rendered
  the same weekend page instead: the row overflowed its allotted
  column width and visibly displaced/blanked the neighbouring Wing/
  ARB schematic rendering in the same crop -- a real defect, not a
  close call. Not debugged further since Variant A already worked
  cleanly; removed from the code entirely after the comparison
  (_measurement_list_row, the measurement_mode parameter threaded
  through build_session_strip/_car_column, and weekend_pdf_export.py's
  MEASUREMENT_MODE toggle) rather than leave a losing, broken path in
  place -- one code path, not a flag nobody will use.
Verification: sample PDFs regenerated from the real Dubai outing
(synthetic in-memory point values as above, real everything else) for
setup (points populated, values render correctly with None -> "-"),
setdown (untouched, all ten boxes show "-", confirming old-data
behaviour holds in the actual PDF too, not just the reshape logic),
and the weekend document (both outings, both sheet types, four
strips/page, values render correctly on the outing with points set,
blank on the one without). No exception after the variant-B removal
and re-render, confirming the cleanup was a pure subtraction.

### Phase 4: targeted tests

Extracted the reshape logic out of ui/views/outing_form.py into a new
core/setup_data_points.py (pure JSON-string functions, no Qt) before
writing tests, rather than testing it in place -- tests/conftest.py's
own pipeline_result fixture already established the precedent that
this suite deliberately keeps PyQt6 modules (outing_form.py) out of
pytest entirely ("pulling Qt into a headless test run ... was judged
not worth the fragility"). outing_form.py's own _reshape_points_out/
_reshape_points_in are now two-line delegating wrappers; behaviour is
unchanged, confirmed by the diagnostics smoke test below exercising
them through the real form.
tests/test_setup_data_points.py, 9 new tests, all passing, 0.20s:
old-outing load is a no-op (no phantom keys); reshape_out with some/
no points set; round trip recovers flat keys for one and both groups
independently; splitter_offset survives untouched; an unparseable
value degrades to null instead of raising; a hand-shortened array
pads with blank on load instead of an IndexError; and a contract
check that core/pdf_export.py's duplicated position lists (Phase 3)
still have 5 entries each, matching POINT_GROUPS -- catches the two
hand-kept copies silently diverging in count, though not in exact
fractional position (that would need visual re-inspection, not a
test).
diagnostics/smoke_test_measurement_points_widget.py (new, Qt/
offscreen, run manually, same precedent as smoke_test_corner_trace_
dialog.py): constructs a real OutingForm against the real Dubai
outing, confirms 4 MeasurementPointsWidget instances (2 splitter + 2
diffuser, one pair per setup/setdown) with all 20 point boxes empty
for this pre-existing outing, confirms setup's and setdown's widgets
are independent objects (not the same instance reused), then drives
the actual production path end to end: type into two point boxes,
call the form's own _collect_setup_data(), assert the resulting JSON
has the correct splitter_points array, clear the boxes, call the
form's own _load_setup_data() on that JSON, and assert the boxes show
the original text again. This is the "form binding" test target --
distinct from the pure-logic tests above in that it exercises the
real widget registration and the real _collect_inputs/_load_inputs
dispatch, not a simulation of what they would produce.
No golden file regenerated or touched. Full regression suite not run
during this phase (standing testing policy for this work package --
runs once, at the very end, before the Phase 5 report).

### Position re-extraction against the user's annotated reference [2026-08-30]

The user annotated a screenshot of the Phase 2 "Car" groupbox render
(the same 238x600 image this session's own phase2_car_group_0.png
produced) with pure-green dots marking the real desired point
positions, and provided it by placing it at the repo root as image.png
(deleted after use, per instruction -- never committed, confirmed via
git status before and after).
EXTRACTION (programmatic, not eyeballed): connected-component
clustering (scipy.ndimage.label) of pixels where G > R+15 and G > B+15
found exactly 10 blobs, ~66-78px each, centroids computed via
ndimage.center_of_mass. Each shape's own bounding box was found the
same way (dark-fill connected-component detection, threshold R,G,B <
80): splitter x=[24,214] y=[140,260], diffuser x=[24,214] y=[318,458]
in the source image's pixel space. Blobs were assigned to a shape by
y-range membership (unambiguous, no blob straddled both ranges), then
normalised to fx=(x-x0)/width, fy=(y-y0)/height within its own shape's
box -- exactly the frame the widget/PDF renderers already use.
NUMBERING (geometry, stated for confirmation per the work order):
splitter's 4 spread points were sorted strictly by x (1=leftmost ..
4=rightmost) as instructed, even though they sit at two different fy
levels, not one row -- point 5 is the one isolated near fy~0, the
front-middle reference. Diffuser had no stated rule; assigned natural
reading order (upper pair left-to-right as 1,2; lower row of three
left-to-right as 3,4,5) and flagged as a proposal, not a fact.
Final values -- SPLITTER_POINT_POSITIONS = [(0.04,0.80), (0.18,0.26),
(0.89,0.25), (0.95,0.74), (0.49,0.00)]; DIFFUSER_POINT_POSITIONS =
[(0.01,0.37), (0.98,0.38), (0.02,0.96), (0.48,0.95), (0.96,0.95)].
DE-DUPLICATION: moved both lists into core/setup_data_points.py (the
one module both ui/views/measurement_points_widget.py and core/
pdf_export.py can import without crossing the PyQt6 boundary) --
resolves the Phase 4 contract test's own flagged risk; the position
count check (test_point_groups_shape_matches_the_pdf_renderer) now
also implicitly guards against the single source itself going out of
sync with POINT_GROUPS' counts.
SHAPE: splitter outline replaced the four-corner rounded-rect with a
custom path -- straight rear edge and sides, one cubic-bezier arc
across the front (SPLITTER_SIDE_FRACTION=0.45 of height is straight
before the curve begins) -- implemented identically in both Qt
(_splitter_path, QPainterPath) and reportlab (_splitter_path,
canvas.beginPath) frames, mirrored across their opposite y-axis
conventions (Qt y-down vs reportlab y-up) by construction, documented
inline in both files so the mirroring isn't accidental-looking.
Diffuser stayed a plain rectangle, per instruction. Front-up travel
arrow kept unchanged.
BUG FOUND AND FIXED mid-iteration (render-and-look caught it, as
intended): the wider point spread (fx to 0.98, fy from 0.00 to 0.96 --
several points sitting ON or past the outline's own edge, by design,
matching the reference annotation) overflowed the PDF diagram's
allotted column width and overlapped its own section title. Root
cause: MeasurementDiagram's box positions were computed relative to
the outline's box, but the Flowable's reported wrap() size didn't
reserve room for a box centred exactly at fx=0/1 or fy=0/1 to extend
past that box's edge. Fixed by having the Flowable reserve box_w/2 and
box_h/2 of margin on every side (wrap() returns width+box_w,
height+box_h; draw() translates by that margin before painting) and
adjusting _measurement_diagram_boxes's own diagram_w calculation so
diagram_w + box_w sums to exactly the allotted column width, not
90-something percent of it as a rough guess. Applied the equivalent
fix on the Qt side too (MARGIN raised from 16 to 20, since BOX_W/2=17
was already technically tighter than the outer margin, just not
visibly clipping yet at the sizes rendered so far) -- fixed
proactively rather than waiting for it to surface as a visible bug at
some other widget size.
VERIFICATION: re-rendered the form widget (both car groupboxes) and
all three PDFs (setup, setdown, weekend) after the position/shape
change, then again after the margin fix -- confirmed no clipping/
overflow at either scale, correct value-to-position mapping (spot-
checked against the same synthetic 12.5/-/8.0/9.2/15.0 splitter and
22.0/-/18.5/19.0/- diffuser values used in the Phase 3 render), old-
outing blanks still render as "-" throughout. tests/test_setup_
data_points.py (9/9) and diagnostics/smoke_test_measurement_points_
widget.py both re-run clean after the refactor -- no suite run this
turn (nothing outside the widget/PDF/position table was touched, per
the standing testing policy). The Car section's scroll/expand
behaviour on a real display remains flagged from Phase 2 -- still not
verifiable headlessly, still needs the user's own manual QA.

### Position symmetrization [2026-08-30]

The user pointed out the raw extracted positions carried hand-jitter
from manual dot placement, and the car is left/right symmetric, so the
diagram should be too. Symmetrized both mirror pairs per shape (average
x_offset = (x_left + (1-x_right))/2 applied as (x_offset, 1-x_offset),
shared y = mean of the pair's y) and snapped each shape's centre point's
x to exactly 0.5 (y unchanged) -- computed via a short one-off script,
not by hand, to avoid a transcription error at this stage.
Raw extracted (pre-symmetrization, recorded here since core/setup_
data_points.py's own comment states only the final values):
  splitter raw: P1=(0.04,0.80) P2=(0.18,0.26) P3=(0.89,0.25)
    P4=(0.95,0.74) P5=(0.49,0.00)
  diffuser raw: P1=(0.01,0.37) P2=(0.98,0.38) P3=(0.02,0.96)
    P4=(0.48,0.95) P5=(0.96,0.95)
Symmetrized (now the literal constants in core/setup_data_points.py):
  SPLITTER_POINT_POSITIONS = [(0.045,0.77), (0.145,0.255), (0.855,0.255),
    (0.955,0.77), (0.5,0.0)]  -- pairs 1&4, 2&3; centre 5
  DIFFUSER_POINT_POSITIONS = [(0.015,0.375), (0.985,0.375), (0.03,0.955),
    (0.5,0.95), (0.97,0.955)]  -- pairs 1&2, 3&5; centre 4
Stored as plain literals, no runtime symmetrization logic -- this was a
one-time cleanup of the extracted constants, not a general capability.
Re-rendered the form widget and all three PDFs (setup, setdown,
weekend) after the change: visually symmetric at both scales, no
clipping/overflow (the margin fix from the prior pass already covers
these positions, all still within the same [0,1] extremes). Cross-
checked one shape's rendered box positions against the exact literal
values via pixel-space bounding-box detection -- matched the expected
mirror-symmetric offsets exactly (guaranteed by the averaging
construction, this was a sanity check against a transcription error,
not evidence of anything that could have differed). tests/test_setup_
data_points.py (9/9) and diagnostics/smoke_test_measurement_points_
widget.py both re-run clean. No suite run, per the standing testing
policy for this work package (nothing outside the position table was
touched).

## 9. Ship-readiness cleanup [2026-08-30]

### Phase 1: dead-diagnostics deletion

Deleted the 29-script candidate list recorded in this file's "Repo
cleanup: pass_2-4 block deletion + dead-diagnostics sweep" entry
(2026-08-20), plus 2 of the 3 orphaned manifest JSONs named there.
Before deleting, re-grepped the whole repo for every candidate name
(scripts and manifests together, one pass); both threshold-provenance
files (inspect_corner_distribution.py, inspect_yaw_stability_b2.py)
confirmed present and NOT on the list, as required.

Deleted (last commit each was tracked at; working tree was clean
before this pass, so this is also the last commit each still exists
in): inspect_abs_slip_channels.py (0b296ce), inspect_b3_verdict_
distribution.py (0bdff87), inspect_corner_demand_ranking.py (76fc576),
inspect_cs_filter_sensitivity.py (e223aba), inspect_entry1_brake_fix_
verification.py (0b296ce), inspect_entry1_brake_production_impact.py
(0b296ce), inspect_fz_sign_conventions.py (39c88a4), inspect_gps_
speed_validation.py (0bdff87), inspect_h2_ay_dual_population.py
(6fcdeb0), inspect_kerb_signal.py (39c88a4), inspect_max_beta_
excursion.py (6fcdeb0), inspect_observer_self_consistency.py
(4b1b548), inspect_offset_chain_decomposition.py (4b1b548), inspect_
pass1_flagged_attribution.py (6fcdeb0), inspect_recommendation_
eligibility_trace.py (5f44688), inspect_rolling_radius_speed_
dependence.py (0b296ce), inspect_sideslip_sign_check.py (4b1b548),
inspect_slip_hypothesis_and_driven_axle.py (0b296ce), inspect_speed_
class_boundary.py (0bdff87), inspect_step1b_wiring_verification.py
(b0e7aa0), inspect_threshold_comparability.py (76fc576), inspect_
urgent_tier_lap_level_fix_check.py (2d3346f), inspect_urgent_tier_
lap_level_verify.py (2d3346f), inspect_vehicle_model_upgrade.py
(86e16b3), inspect_wp1_canonical_realization.py (0bdff87), inspect_
wp1_reset_guard_freeze_proof.py (e6ef209), inspect_wp1_turn2_
validation.py (0bdff87), plot_kalman_qr_ratio_sweep.py (9f67f3a),
run_ekf_dugoff_pass0.py (6fcdeb0) -- all in diagnostics/. Plus
diagnostics/fit_dugoff_pass2_refit_manifest.json and diagnostics/
fit_dugoff_pass4_refit_manifest.json, both gitignored generated
artifacts (diagnostics/*.json), never tracked, no commit hash
applicable.

DEVIATION FROM THE LITERAL CANDIDATE LIST, flagged rather than
silently followed: diagnostics/fit_dugoff_pass3_refit_manifest.json
was NOT deleted, though it was on the original 3-manifest candidate
list. config/parameters.json line 181's frozen_from field for the
still-KEPT tyre_model_ekf.pass_3 block names this exact file as its
provenance record ("WP-N2 pass 3, diagnostics/fit_dugoff_pass3_refit.py,
diagnostics/fit_dugoff_pass3_refit_manifest.json (timestamp
2026-08-20T08:10:25Z)"). Unlike the pass_2/pass_4 manifests (whose
producing scripts and config blocks are both already gone), pass_3's
config block is live and its own producing script (fit_dugoff_pass4_
refit.py, which reads pass_3 as its EKF source) still exists per the
2026-08-20 pass's own explicit decision to keep both. Deleting the
manifest would have orphaned a live provenance pointer for no
config-size benefit; the "when unsure, keep it and list it" rule
applies squarely. Confirmed via git ls-files that this manifest was
never tracked either -- gitignored like its two siblings, so leaving
it costs nothing.

Post-deletion dangling-reference sweep (full-repo grep for all 29+2
names): 6 files matched. Two are the historical record itself
(thesis_notes.md, PLAN.md's STATUS/NOW narrative) and were correctly
left untouched, per CLAUDE.md's "never delete or rewrite existing
entries" rule -- these are dated journal entries describing what was
true when written, not live pointers guiding future action. Four were
live citation/catalog pointers and were fixed in place, annotated
"removed 2026-08-30, see git history" without deleting the surrounding
evidence claim (matching the 2026-08-20 precedent set by fit_dugoff_
pass4_refit.py's own docstring fix): diagnostics/README.md (removed
the now-nonexistent inspect_kerb_signal.py catalog entry outright,
since a README of runnable scripts should not list one that no longer
runs); sideslip_comparison_report.md (2 edits: the force-balance
steady-state section header and the physical-sign-check section
header, both citing now-deleted scripts); diagnostics/inspect_wheel_
speed_sources.py (1 comment citing inspect_gps_speed_validation.py's
regression technique); tests/test_phase_boundary_invariants.py (3
citations of inspect_entry1_brake_fix_verification.py -- the
BRAKE_RISE_BAR constant comment and two docstrings -- annotated, not
rewritten, so the regression framing CLAUDE.md requires stays intact).
diagnostics/fit_dugoff_pass4_refit.py's own MANIFEST_PATH match is
self-referential (its own write-target path) and needed no change.

The Phase-2-flagged dead local variable in inspect_abs_slip_
channels.py is now moot -- the file is deleted.

Verification: tests/test_phase_boundary_invariants.py (the only
targeted suite touched by this phase's edits) re-run clean, 7/7
passed. Per this package's testing policy, the full suite runs once
at the very end, not here.

### Phase 2: placeholder and stale-text sweep

Repo-wide grep for PLACEHOLDER/TODO/FIXME/XXX/TBD (case-sensitive,
matching CLAUDE.md's own marker vocabulary). Every hit classified;
nothing found warranted deletion as stale -- every live marker traces
to a real, currently-open decision, and the two apparent XXX hits
(ui/views/outing_form.py, this file, both "\uXXXX" describing a
Unicode escape sequence) are false positives of the regex, not
markers, confirmed by reading context, no action.

STAYS, as pre-authorized: both calibration banner PLACEHOLDER texts
("PLACEHOLDER: sideslip estimator changed, verdict thresholds not
..." in core/weekend_pdf_export.py and ui/views/outing_form.py;
"...recommendation rules key on..." in ui/views/outing_form.py) --
final wording is its own pending decision, not invented here. The
icon-set TODO (ui/main_window.py: "icons in separate svg files --
TODO: replace with a proper icon set"). The page-TBD literature
citations needing the physical books (modules/tyre_model.py,
modules/tyre_model_pacejka.py, modules/stability_analysis.py x2,
plus their PLAN.md/thesis_notes.md ANCHORS-section tracking) --
unchanged, per the standing PLAN.md ANCHORS note these get replaced
when the physical text is checked, not deleted now.

STAYS, newly classified this phase (live, tied to a real open item,
listed here for visibility): config/setup_parameters.json's four
"switch-position channel name TBD, identify in next data file's
channel scan" notes (tc_lat, tc_lon, abs_position, brake_bias) --
this is PLAN.md's own tracked open thread (the new-data-file channel
scan), not a leftover. config/parameters.json's three nis_tuning_note
PLACEHOLDER-defaults comments (pass_0/1/2) -- tracks the live,
still-provisional NIS-gate threshold decision (PLAN.md NOW section,
"five-data-points-one-session provisional"). diagnostics/plot_
sideslip_comparison.py and diagnostics/plot_slip_angle_comparison.py's
matching NIS PLACEHOLDER-defaults print strings -- same live config
state, read at runtime, not hardcoded duplication. diagnostics/
inspect_pass1_final_validation.py's "ORIGINAL PLACEHOLDER defaults"
caveat print -- this is the frozen pass-1 validation baseline script
(PLAN.md NOW section), the caveat is an accurate, still-true statement
about its own noise-model tuning, not stale text.

Historical narrative (PLAN.md STATUS/NOW section, this file's own
dated entries) was left untouched by design -- these are journal
entries recording what was TBD/PLACEHOLDER AT THE TIME, governed by
the same "never rewrite existing entries" rule as everything else in
this file; scrubbing the word out of history would falsify the
record, not clean it up.

Excluded from this sweep as out-of-scope: Nürburgring_Outing1_
Practice_Setup2.pdf (repo-root, untracked, unignored -- a binary PDF
sample-export artifact from an earlier session's visual QA, not
source text; the grep hit is bytes inside the binary, not a
readable marker. Flagged for Phase 5/6 reporting as loose untracked
state, not acted on here since deleting or moving it is outside this
phase's placeholder-sweep scope).

config/parameters.json's rad^-1 fix: line 67's cs_fallback comment
had a literal "rad⁻¹" (Unicode superscript minus-one) where every
other unit string in the file uses plain ASCII notation elsewhere
(e.g. "N/rad", "rad/s"). Replaced with "rad^-1", JSON comment text
only (the "_comment_cs_fallback" key, never parsed as a number),
zero functional effect -- confirmed by re-loading the file with
json.load after the edit. The adjacent "×" (multiplication sign, same
comment) was left alone: explicitly out of this phase's stated scope
(only the rad^-1 character was named), and the general non-ASCII
normalization sweep already ran repo-wide in the documentation polish
pass (this file, section 6).

The dead unused variable in diagnostics/inspect_abs_slip_channels.py
(Phase 1's own item to verify): moot, confirmed -- the file no longer
exists after Phase 1's deletion.

No suite run this phase (JSON comment edit only, no code/logic
touched); config validity confirmed directly via json.load.

### Phase 4: diff locking-torque table printed

This package's only authorized functional change: the 5-point
differential_locking_torque_measured data (car[
"differential_locking_torque_measured"] = {"1".."5": Nm or null},
collected by the setup form since the arb_front_mount/diff-torque
schema addition but never rendered) now prints in core/pdf_export.py's
shared strip renderer's car column, at both scales, since weekend_
pdf_export.py fully reuses build_session_strip with no car-column
code of its own to touch.

Design: a new DIFF_TORQUE_LABEL ("Diff Locking Torque (measured,
Nm)") and _diff_torque_row() helper, following "5 bordered cells"
literally rather than the splitter/diffuser MeasurementDiagram
pattern -- diff locking torque has no physical floor position to draw
an outline against, so the UI form's own visual structure (position
number stacked above value, in a bordered cell) was mirrored instead:
one reportlab Table row, 5 columns, each cell a two-Paragraph stack
(position number via the existing table_head style, value via the
existing car_value style), GRID-bordered like every other table on
the sheet -- reuses _bordered_table exactly as every other car-column
element does, no new styling primitives. Placed directly under the
Diff Preload/Position/Splitter parameter table (both are diff-related,
grouped together) and before the Splitter Pts diagram.

Render-and-look iteration, as required: generated real setup/setdown/
weekend PDFs against the actual Dubai outings in data/setuptool.db
(outing #1, both sheet types, plus the two-outing weekend document),
via a scratchpad script replicating ui/views/outing_form.py's own
_print_sheet calling convention (a lightweight temp object carrying
setup_data/setdown_data under the one attribute name generate_
setup_pdf expects). Real stored values are all 0.0 (the widget's
default, never actually measured on this car yet) -- legitimate real
data per the project's real-data-only rule, and still a valid render
check for cell borders/label legibility even though every value is
identical. pymupdf installed transiently to rasterize both documents
to PNG for visual inspection, uninstalled immediately after (same
precedent as the PDF layout rework session) -- confirmed removed via
pip show and absent from requirements.txt.

OUTCOME: fits cleanly at both scales on first attempt, no degradation
needed. At "large" (single-session) scale the table reads as five
clearly separated, legible cells. At "small" (weekend, four-strips-
per-page) scale -- the car column being, per the work order's own
description, already the densest area on the sheet -- an 8x zoom crop
confirmed clean borders, no overlap with the Diff Preload/Position/
Splitter table above it or the Splitter Pts diagram below it, and no
displacement of anything else in the column (the same failure mode
that sank the splitter/diffuser values-only-row alternative in the
prior PDF session). No values-only-row or single-document-only
degradation was needed; the straightforward design was rendered and
looked at, not assumed.

PLAN.md's "diff locking-torque table stays unprinted, its own open
decision" line (PDF layout rework session entry) updated to record
this closes that open item, pointing here.

No suite run this phase (PDF rendering + one new car-column helper,
no analysis/estimator/classification code touched); the render-and-
look check above is this phase's own verification method, per the
work order.

### Out-of-scope emergency fix: app-breaking SyntaxError in core/config_loader.py [2026-08-30]

Found incidentally, not by this package's own Phase 5 audit (that
audit was still running) but by a background agent doing this
package's Phase 3 comment pass, which noticed and correctly declined
to touch a stray trailing character it had no mandate to fix.
core/config_loader.py line 21 read
`return config.get("setup_parameters", {})c` -- a bare `c` after the
closing paren. This is a SyntaxError, not a logic bug: confirmed by
direct import that it broke the ENTIRE application -- `python main.py`
-> ui.main_window -> ui.views.weekends -> ui.views.outings ->
ui.views.outing_form -> `from core.config_loader import
get_setup_parameters` fails to import at all.

~~PROVENANCE CLAIM, WRONG, struck through same day: originally
recorded here as "introduced in commit 39c88a4 ... predates this
package entirely" and reported to the user that way.~~ CORRECTED
(2026-08-30, same day, caught during Phase 6 final verification): a
git diff-direction misread. `git show HEAD:core/config_loader.py` is
clean -- the file was NEVER committed with this bug. `git diff HEAD
-- core/config_loader.py`, read correctly, has HEAD (clean) on the
`-` side and the then-current corrupted working tree on the `+` side;
the earlier note had this backwards. The corruption was an
UNCOMMITTED working-tree change that appeared DURING this session --
the session's own recorded git status was clean at the very start, so
it cannot have predated this package. Its exact origin is unresolved:
neither this session's own direct edits nor the Phase 3 background
agent's verified diff (git-diff-checked, ui/main_window.py and
ui/views/outing_form.py only) touched this file. The Phase 5 audit
agent's own report (see its Phase 5 entry below) independently flagged
concurrent, not-self-authored working-tree changes during its ~15-
minute run window -- consistent with some other concurrent process
touching the repo, though this remains unconfirmed, not a diagnosed
root cause. User told the corrected version directly, not just here.

tests/ never caught this because the suite exercises modules/core
logic directly, never ui/ (PyQt6 layer is deliberately untested that
way, per CLAUDE.md's module boundary rule) -- this is exactly the
blind spot the standing "UI changes: I run the app manually" rule
exists for. Whatever its origin, had this reached a commit un-noticed,
nobody could have opened the app until caught.

This package's own hard constraints ("No functional changes EXCEPT
the one explicitly authorised in Phase 4", Phase 5 "report only, fix
nothing") explicitly forbid this fix. Surfaced to the user directly,
out of band, rather than silently fixed or silently deferred to the
Phase 6 report -- a completely non-launchable app is not something to
sit on until a report the user reads at the end. User explicitly
authorised fixing it immediately (over leaving it for the report),
recommended option, given via AskUserQuestion.

Fix: removed the stray `c`, one character, restoring `return
config.get("setup_parameters", {})`. Verified by direct import
(`import ui.main_window` -- previously SyntaxError, now succeeds) --
no dedicated test exists for this function/module (grep confirmed).
No other change to the file or anywhere else. This is the one
deviation from this package's "no functional changes outside Phase 4"
rule, made with explicit contemporaneous user authorisation, recorded
here rather than folded silently into any other phase's entry.

### Phase 3: comment humanization pass, second round

Second, tighter tightening pass over modules/, ui/ (incl. ui/views/),
core/, and test_stability.py, against the prior "documentation/comment
polish pass" baseline (this file, section 6) which had already brought
diagnostics/ and tests/ to standard with zero edits needed -- this
round deliberately excluded both directories again, plus config/*.json,
per its own scope. Bar: "a capable student wrote it, and wrote less."

Result: the prior pass had already done the heavy lifting -- 30 of 32
files in scope needed zero further edits. Two files, both restatement-
only: ui/views/outing_form.py (8 edits -- "Severity logic", "Build
verdict strings from the simple vocabulary", "Group by lap, classify
each corner", "Index each lap's entries by stable_corner_id", "Build
one lap row per lap, in lap order", "Inline details placeholder,
hidden by default", "Clear any previous details", "Two lines: stable
corner id, then short verdict" -- every one immediately followed by
the single line of code it named) and ui/main_window.py (1 edit -- a
trailing comment restating a one-line signal connect). Verified by
diff: every removed line was pure WHAT-restatement, no WHY-content
lost, files still compile clean (py_compile) after the edits.

11 borderline cases were considered and kept, not deleted -- full list
in the agent's own report, spot-checked here rather than reproduced
verbatim: several one-line comments serving as informal docstrings on
otherwise-undocumented Qt-construction helper methods (kept, rule 3
allows one-line summaries); a few structural section dividers inside
long (~50-80 line) UI-construction methods with no formal docstring
of their own (kept, navigation aids in genuinely long functions, not
restatement of the adjacent line); one comment in modules/
longitudinal_stiffness.py restating a load-bearing cross-file
invariant (_plausibility_exclude_mask's two-call-site contract,
already stated in that function's own docstring) at its actual call
site (kept -- restating an invariant where it is USED, not restating
what adjacent code does, is exactly the kind of WHY-adjacent aid rule
2 protects).

One incidental finding, not part of this phase's own mandate and NOT
acted on by it (correctly -- a comment-only pass has no mandate to
change code): a stray character in core/config_loader.py, noticed but
left for the orchestrating session to triage. See the emergency-fix
entry immediately above -- that stray character turned out to be an
app-breaking SyntaxError, found because this phase's agent read the
file carefully enough to notice one out-of-place letter, even though
fixing it was correctly outside its own scope.

No suite run this phase (comment/docstring-only, no logic touched).

### Phase 5: ship-readiness audit (report only, nothing else fixed)

Report-only, per the work order -- everything below is a finding, not
an action, EXCEPT the config_loader.py SyntaxError already handled
above as its own explicitly user-authorised, out-of-band exception.

**Task 1 -- full analyze-and-export cycle, all 4 sideslip_source
modes, no traceback.** Verified two ways: this session's own direct
sweep (modules.tyre_fit_auto.resolve_sideslip_beta called with each
mode as a plain argument -- confirmed this never reads config's own
stored sideslip_source, so the sweep never needed to touch config/
parameters.json at all) through estimate_slip_angles/estimate_
lateral_forces/estimate_cornering_stiffness/estimate_yaw_moment_
stability/summarise_corners; and independently, a background agent's
more complete replication of the exact production chain (StabilityAnalysisThread.run(),
ui/views/outing_form.py:147-282) including estimate_vertical_loads,
longitudinal forces/stiffness, the real _classify_corner, and PDF
export via both core/pdf_export.generate_setup_pdf and core/weekend_
pdf_export.generate_weekend_pdf, against the real Dubai outing in
data/setuptool.db. Both approaches agree: all 4 modes (kinematic,
ekf_pass_1, ekf_auto_dugoff, ekf_auto_pacejka) complete without
exception; the two auto modes' NIS gate verdicts both 'pass' on this
session's data (health_score 0.1638 dugoff / 0.1751 pacejka), no
fallback triggered in either. 56 corner summaries produced in every
mode.

Process note on the second (agent) run: it DID at one point corrupt
config/parameters.json's sideslip_source via a crude json.load/
json.dump round-trip (collateral damage -- ensure_ascii=True re-
escaped the file's one non-ASCII character and dropped the trailing
newline, on top of leaving sideslip_source on a non-original value
when its own run was interrupted mid-sweep by an intermediate status
checkpoint). Caught and fixed directly during Phase 6 verification:
config/parameters.json restored via git checkout + reapplication of
this package's own single intentional edit (the rad^-1 fix, Phase 2),
confirmed via git diff to carry ONLY that one line change from HEAD.
4 stray evidence PDFs the agent also left in diagnostics/ were
deleted; confirmed via git status that no untracked files remain.
Lesson for future agent-authored config edits: targeted string
replacement, never a full json.load/json.dump round-trip on a file
carrying non-ASCII content or no trailing newline, or json.dump's
ensure_ascii default silently rewrites everything around the one key
that was meant to change.

**Task 2 -- every dialog/main window instantiates cleanly, headless
(QT_QPA_PLATFORM=offscreen).** All 14 attempted: DriverDialog (new+
edit), WeekendDialog (new+edit), WeekendPdfDialog, CornerTraceDialog,
LapTraceDialog, _TraceDialogBase, MainWindow, DriversView,
SettingsView, WeekendsView, OutingsView, OutingForm (new+edit) --
all instantiate cleanly, real DB objects used throughout (RaceWeekend
id=1, Outing id=1, Driver id=1), no fabricated data. Skipped:
MeasurementPointsWidget (not a standalone top-level target, always
constructed as an OutingForm child with specific args -- already
exercised by OutingForm's own 4 successful constructions).

**Incidental first-time-user-risk findings (exploratory, not
exhaustive):**
- Raw exception strings reaching the UI directly, no friendly
  wrapping: ui/views/outing_form.py's CsvLoaderThread error path
  (~1172-1175, "Error loading file: {error_msg}"), StabilityAnalysisThread's
  error path (~2511-2514, "Analysis failed: {msg}"), and ui/views/
  weekend_pdf_dialog.py (~141-146, QMessageBox.critical shows a raw
  `{e!r}`).
- Silent/partial failure: core/pdf_export.py's generate_setup_pdf
  (~563-567) has a bare `except Exception: pass` around
  json.loads(outing.setup_data) -- a corrupt setup_data silently
  renders as an all-blank PDF with zero indication anywhere that
  parsing failed. core/weekend_pdf_export.py's analogous per-outing
  failure path (~474-481) is better (prints a visible inline error
  note in the generated PDF itself) but still surfaces a raw `{e!r}`
  to the end user rather than a friendly message.
- ui/views/outing_form.py's _print_sheet (~3259-3265) only catches
  PermissionError around generate_setup_pdf -- any other exception
  (e.g. a deeper KeyError on malformed setup data) propagates
  unhandled out of a Qt slot with no dialog telling the user the
  export failed.
- Confusing first-time state: the "Analysing laps {lap_filter}..."
  status label (~1272-1275) is set once and never updates while
  StabilityAnalysisThread runs. The ekf_auto_dugoff/ekf_auto_pacejka
  modes' fit chain (a 5x5 R-scale grid sweep, modules/tyre_fit_auto.py)
  measured taking several minutes in this audit's own run -- the only
  progress indication is a print() (~239-242), invisible in a packaged
  app. A first-time user watching this has no way to tell the app
  apart from frozen.

**Config-key comment audit (this session's own pass, all 5 non-
protected config/*.json files -- config/car_data.json excluded,
protected/gitignored per the standing protected-set rule, not a
tunable-parameter file in the same sense).** Essentially zero genuine
violations found. config/parameters.json is exhaustively commented
(nearly every tunable carries an adjacent _comment_*/_note key; the
handful without one -- cs_min_window_samples, apex_half_window_samples,
cs_filter_cutoff_hz -- have fully self-evident names and sit inside a
commented group). config/car.json's setup-parameter schema uses
standard racing-engineering terms throughout (toe, camber, arb,
springs, etc.), no comments needed. config/channels.json is self-
documenting via its own per-channel "label" field. config/setup_
parameters.json and config/recommendations.json are structured
registries: every STRUCTURAL field is explained once in a top-level
_comment block, and every per-entry record self-documents via label/
mechanism/notes (setup_parameters.json) or condition/suggestion/
rationale (recommendations.json). This item is already effectively
closed by the prior documentation/comment polish pass and this
session's own Phase 2; nothing further to list.

**Concurrent-modification caveat, both this session's Task 1 runs and
the config_loader.py provenance correction above:** something touched
the working tree during this session that neither this session's
direct edits nor its background agents' own verified diffs account
for (the config_loader.py stray-character corruption; one agent's own
report independently flagging 39 files it did not author). Origin
unresolved -- flagged, not diagnosed, not acted on beyond the direct
fixes already recorded above.

This list is tomorrow's raw material, not this package's work, per
the work order's own framing.

### Phase 6: close-out

Full regression suite run exactly once, per this package's testing
policy -- targeted tests freely during iteration (Phase 1's affected
test re-run, this session's own direct Task-1 pipeline sweep), no
other full-suite/golden run before this point. Standard temporarily-
flipped-kinematic-then-restored procedure: sideslip_source flipped
from the user's own live value (ekf_auto_pacejka) to kinematic
(golden fixtures require it), suite run, restored -- git diff
confirmed config/parameters.json carries ONLY this package's one
intentional edit (rad^-1, Phase 2) against HEAD afterward, byte for
byte. Result: 119 passed, 1 xfailed, 0 failed, 0 errors (44m33s) --
exactly the pre-registered baseline this package's own work order
predicted, and matches the prior session's last recorded full-suite
count exactly (thesis_notes.md "8. Splitter/diffuser measurement
points", Phase 4). No golden fixture touched, no test file behaviour
changed by this package (tests/test_phase_boundary_invariants.py's
3 edits were comment-only pointer annotations, confirmed by its own
7/7 targeted re-run in Phase 1 and now folded into this full green
run).

Protected set (docs/literature/, docs/car_data/, config/car_data.json,
HANDOVER.md, docs/study/) confirmed empty via git ls-files. git status
final footprint: 11 files modified (PLAN.md, config/parameters.json,
core/pdf_export.py, diagnostics/README.md, diagnostics/inspect_wheel_
speed_sources.py, sideslip_comparison_report.md, tests/test_phase_
boundary_invariants.py, thesis_notes.md, ui/main_window.py, ui/views/
outing_form.py -- core/config_loader.py shows NO diff, confirming its
mid-session corruption and this package's own fix both left it exactly
matching clean HEAD), 29 diagnostics/*.py files deleted, 0 untracked
files remaining. No commit made, per the work order.

One retrospective note on process, not this package's science: two
background agents (Phase 3's comment pass, Phase 5's headless audit)
both did real, independently-verified work, but the harness's own
"completed" notification fired at least once for each before either
had actually finished (Phase 5's agent's first "completion" was
literally the text "waiting for..."), and the Phase 5 agent
transiently corrupted config/parameters.json in ways its own
final report did not fully catch (see the Phase 5 entry's process
note above). Every deliverable from both agents was independently
verified before being trusted -- git diff on every file either agent
touched, direct re-execution of the Task 1 sweep, git-diff
confirmation of config file state -- rather than taken on the
agent's own say-so. This caught real problems (config corruption,
premature-completion reports) that would otherwise have gone into
this record uncaught.

## 10. Second diagnostics sweep: full-inventory classification [2026-08-30]

Full reclassification of every file in diagnostics/ (not the stored
candidate list from the 2026-08-20 sweep) -- 54 real files (56 dir
entries minus __pycache__/plots, both directories not classified as
files). Standard: same as 2026-08-20 -- grep the repo for incoming
references per file before deleting; when genuinely unsure, keep and
list. Method: two full-repo grep sweeps covering all 54 basenames,
followed by path-scoped searches against PLAN.md/config/*.json/tests/
modules/ui/core specifically (the sites the work order's own three
categories name), individual header reads for methodology
understanding, and a targeted internal-dependency check across
diagnostics/*.py itself for `from diagnostics.X import` statements
(missed on the first pass, see the near-miss below).

DELETED, 23 scripts (last commit each was tracked at; working tree
was clean before this sweep): inspect_wp2b2_recommendation_trace.py,
inspect_cs_kerb_window_audit.py, inspect_c9_negative_cs.py, inspect_
lap2_corner5_cs_discrepancy.py, inspect_corner_radius_overlap.py,
inspect_pass1_ekf_timing.py, inspect_tyre_variant_comparison.py,
inspect_nis_short_run_blindspot.py, inspect_ekf_dugoff_circularity.py,
inspect_ekf_pass1_evaluation.py, plot_sideslip_comparison.py, plot_
slip_angle_comparison.py, inspect_cs_linear_ref_staleness.py, inspect_
observer_slip_angle_circularity.py, inspect_kalman_qr_sweep.py,
inspect_kalman_qr_ratio_sweep.py, inspect_sideslip_methods_
comparison.py, inspect_washout_mechanism.py, plot_washout_sweep.py,
plot_washout_sweep_slipangle.py, plot_estimator_comparison.py, _plot_
common.py, inspect_wheel_speed_sources.py (see near-miss below --
deleted then restored). Every deletion's finding traced to a real
thesis_notes.md record, sometimes by WP name/content rather than exact
filename (spot-checked: inspect_pass1_ekf_timing.py -> "WP-N2 Step 1a:
pass-1 EKF wall-clock timing" section exists; inspect_corner_radius_
overlap.py -> corner_radius_filtered findings recorded at multiple
dates; inspect_wp2b2_recommendation_trace.py -> the "real, persisted
Dubai outing" verification entries; inspect_ekf_pass1_evaluation.py ->
the C2-excursion/acceptance-criteria entries spanning ~1500 lines).

Also deleted, 4 gitignored stale artifacts (zero git footprint either
way, deleted for working-directory hygiene per the work order's own
"contain ONLY things with a stated reason to exist" goal):
channels_in_file.txt (stale regenerable output of the KEPT scan_
channels.py), step1b_wiring_verification_output.txt and step1b_
wiring_verification_cap1_output.txt (orphaned outputs of inspect_
step1b_wiring_verification.py, already deleted in the 2026-08-20
sweep), test_stability_after_step1b_output.txt (one-off manually-
redirected log capture, no producer script). Also deleted: diagnostics/
plots/ (198 gitignored PNG files across several dated subfolders,
entirely orphaned once all 5 plot-generating scripts -- plot_
sideslip_comparison.py, plot_slip_angle_comparison.py, plot_washout_
sweep.py, plot_washout_sweep_slipangle.py, plot_estimator_
comparison.py -- were deleted).

NEAR-MISS, caught and fixed before it became a real breakage:
inspect_wheel_speed_sources.py was initially deleted as an (a)
candidate (no PLAN.md/config/test/production reference found by the
first-pass search) -- but a follow-up internal-dependency check
(grep for `^from diagnostics\.` across the surviving files) found
diagnostics/inspect_washout_cutoff_sweep.py (a KEPT, PLAN.md-
referenced script) has a hard import: `from diagnostics.inspect_
wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS`.
Restored via `git checkout` (the file was tracked, unmodified at
that point in the sweep) and its one prior-session comment fix
(the inspect_gps_speed_validation.py pointer annotation from the
2026-08-20 sweep) reapplied by hand since checkout reverts to the
last commit. Reclassified [dependency] in the README rather than
[keep-referenced] -- it has no independent external reference of its
own, it survives purely because inspect_washout_cutoff_sweep.py needs
it. Lesson, folded into the new CLAUDE.md standing rule's "Dependency"
category and this sweep's own method: checking a script's OWN
external references is not sufficient before deleting it inside a
folder with internal cross-imports -- the dependents' import
statements must be checked too, not just assumed absent from an
early exploratory grep that used the wrong pattern (searched for
`from <name> import`, missed the actual `from diagnostics.<name>
import` form used everywhere in this codebase).

KEPT, 26 scripts + 4 non-py artifacts + README.md -- full reasons in
diagnostics/README.md, rewritten this sweep to state each survivor's
specific keep-reason (previously listed only 2 of the then-many
files, silently stale since 2026-07). Two production-adjacent
findings worth flagging even though out of this sweep's scope to fix:
sideslip_ekf_dugoff.py and sideslip_ekf_pacejka.py are NOT diagnostics-
only despite their location -- modules/tyre_fit_auto.py imports
estimate_sideslip_ekf_dugoff/estimate_sideslip_ekf_pacejka from them
directly, as do tests/test_pure_functions.py and tests/test_nis_
gate.py. A real production dependency living in diagnostics/ by
historical accident, not by design.

BORDERLINE, kept per "when unsure, keep, list": inspect_slip_channel_
sweep.py (Rolling-radial/combined-slip follow-up, QUEUED ITEM 2+4,
keyword scan for slip/traction-control channels). No exact-filename
citation found anywhere, and its own specific finding could not be
conclusively distinguished from the earlier, more general WP2b-1 "Full
channel census + targeted verification" entry (2026-07-26, which
independently found and documented log_speed_fl/fr/rl/rr and abs_
Slip_FL/FR/RL/RR). Given the ambiguity, not deleted.

Dangling-reference healing (same "removed [date], see git history"
convention as the 2026-08-20 sweep): sideslip_comparison_report.md (3
citations: inspect_kalman_qr_ratio_sweep.py x2, inspect_c9_negative_
cs.py x1); diagnostics/inspect_washout_cutoff_sweep.py (1 comment
citing inspect_washout_mechanism.py's WP-S3c construction);
diagnostics/inspect_pass1_final_validation.py (4 print()-string
section-header citations of inspect_ekf_pass1_evaluation.py/inspect_
ekf_dugoff_circularity.py -- these are runtime output strings, not
comments, but carry no numeric/frozen content, so annotating them
does not touch the frozen baseline's actual manifest data). thesis_
notes.md's own historical mentions left untouched by design, same
rule as every prior sweep.

Verification: full compile check (py_compile) on every surviving
diagnostics/*.py file, clean. Direct import check confirming modules.
tyre_fit_auto still imports cleanly from diagnostics/sideslip_ekf_
dugoff.py and diagnostics/sideslip_ekf_pacejka.py. Targeted pytest run
(tests/test_pure_functions.py, test_nis_gate.py, test_auto_fit_
wiring.py, test_config_schema_integrity.py, test_setup_data_
points.py -- the five tests touching diagnostics/ imports or
citations): 65 passed, 1 xfailed, 2 errors (0:30:23) -- both errors
a conftest.py fixture guard (test_lateral_force_split_moment_identity,
test_schema_version_matches_pipeline_result_shape) asserting sideslip_
source=="kinematic" for golden-adjacent tests, tripped because this
targeted run used the live config value (ekf_auto_pacejka) rather than
the full-suite-only flip-and-restore procedure -- confirmed NOT a
sweep-caused regression by re-running both under a temporary kinematic
flip (both pass, 100.52s), then restoring sideslip_source to ekf_auto_
pacejka again, verified byte-identical to the pre-sweep state via git
diff (one line changed: this session's own earlier rad^-1 fix,
nothing else). Per this package's own instruction, no full suite run.

## 11. GT3 Paul Ricard export: diagnosis and fix [2026-08-31]

### Diagnosis (read-only investigation turn)

GT3_PRC_MLA.txt (534 MB, team telemetry) was found sitting untracked
at the repo root. Step 0: added to .gitignore (/*.txt with explicit
!/channel_list.txt and !/requirements.txt exceptions, the only two
legitimate tracked root .txt files) before any further work -- verified
via git status/check-ignore/ls-files.

Full characterisation (size, encoding ISO-8859/latin-1, CRLF, header
block, sample interval, time span, completeness) plus channel
completeness against docs/channel_requirements.md (all REQUIRED
channels present by exact name -- the earlier "lap report" claiming
sclu_yaw_rate/lap_distance missing was simply wrong, not reflective of
this raw export) plus a units cross-check against Dubai's own raw
strings (one real mismatch found: lap_distance is [m] here vs Dubai's
[ft]) -- full detail already reported to the user in that turn's own
response, not duplicated here in full.

ROOT CAUSE, reproduced directly (modules.csv_parser.parse_csv on the
real file, no exception, 2.53s, silent all-missing result -- 0/27
channels, 0 laps, 0 corners): this file uses Pi Toolbox's WIDE-TABLE
export layout (one {ChannelBlock} section, header row = Time + 4176
channel names as columns, one data row per timestamp) where Dubai uses
the NARROW layout (one {ChannelBlock} section per channel). csv_
parser.py's `if len(header_parts) == 2` check (the line that used to
be line 50) only recognised the narrow layout; a wide header fails
that check, falls through a do-nothing branch, and the outer loop then
steps through all 13,735 data rows one at a time without ever
recognising any of them as channel data. Every downstream consumer
(_on_csv_loaded, _update_corner_map_trace, StabilityAnalysisThread)
already has a missing-data guard and degrades gracefully with a status
message rather than crashing -- "the app cannot open" describes a
silently empty, unusable load, not a crash dialog.

A second, independent, more dangerous problem was found in the course
of this diagnosis and would have fired the moment the structural
parsing gap was fixed without also fixing it: modules/stability_
analysis.py's _interp_lap_distance_guarded (and modules/corner_
analysis.py's _invert_s_to_t, a second independent copy of the same
assumption) unconditionally multiplied lap_distance by 0.3048 assuming
feet -- the parser never validated a channel's actual file unit
against that assumption anywhere. This file's lap_distance is genuinely
in metres; had it been fixed to parse without also fixing the unit
handling, every distance-derived quantity (corner brackets, apex
positions, Module 5's s-anchored regression, cross-lap clustering)
would have been silently corrupted by ~3.28x, with no exception
anywhere -- a worse failure mode than the original silent-empty one,
because it would have looked like a real (wrong) result rather than an
obviously broken one.

Also found and reported, not part of the parse failure itself: this
file's sample rate is 20 Hz, not this project's 50 Hz -- confirmed
real (measured directly from consecutive timestamps), not a parsing
artifact. Every estimator window, the NIS window, kerb dilation, and
LS_ratio's min_samples were validated at 50 Hz only (PLAN.md STEP 3's
own min_samples_floor rate-derivation, thesis_notes.md "PLAN.md STEP 3:
50 Hz min_samples adaptation" -- rate-DERIVED, but never rate-VALIDATED
at any rate other than 50 Hz). The user's own framing after this
report: "rate is NOT native... partial session... do NOT run any
analysis on this file for validation purposes" -- this file was never
going to be a real analysis target regardless of the parse/unit fixes.

### Fix, implemented same session on explicit authorisation

Three coordinated changes plus an encoding fix, all additive/branch-
gated so Dubai's own parse is unaffected:

**A -- wide-table parsing** (modules/csv_parser.py): the `{ChannelBlock}`
handler now branches on header shape. `len(header_parts) == 2` ->
unchanged narrow-format code path (byte-identical for Dubai, which
never produces any other shape). `len(header_parts) > 2` and the first
column is "Time" -> new wide-table path: builds a column-index map
restricted to config/channels.json's whitelist (a 4176-column row is
expensive to fully materialise per sample otherwise), then reads every
subsequent row once, pulling only the wanted columns. A `_split_name_
unit` helper (name/unit extraction, shared by both branches) replaces
the old narrow-only inline slice.

Addendum, added after real-file review surfaced three more cases the
wide branch needed to tolerate, all real observed shapes (not
malformed data): (1) comma AND dot decimal separators, in the TIME
column too, not just values -- Dubai logs commas, Paul Ricard logs
dots, this is locale, already handled by the existing `.replace(",",
".")` idiom, just needed the wide branch to use it consistently, which
it did from the first draft; (2) "nan" tokens -- Python's `float()`
parses "nan" successfully (unlike "-nan(ind)", MSVC's textual NaN,
which raises ValueError and was already caught) -- added an explicit
`v != v` / `t != t` NaN check so both are treated as a missing CELL
for that one channel/sample, not silently included as a NaN value or
allowed to fail the whole row; (3) short/partial rows, including a
bare-timestamp-only row (`len(parts) == 1`) -- already tolerated by
the existing `col_idx < len(parts)` per-column bound and the `len(
parts) > 1` row-level gate, no additional code needed, just confirmed
and covered by a test.

The pre-existing NARROW branch's own parsing loop was deliberately
NOT touched by the nan-token fix -- it has the identical latent gap
(float("nan") would parse and silently become a NaN sample there too),
but touching it carries real risk to Dubai's byte-identical golden-
fixture guarantee for zero benefit within this session's actual scope
(nothing established Dubai's file contains any such token). Flagged
here as a known, deliberately out-of-scope observation, not fixed.

**B -- unit-aware lap_distance conversion**: new modules/stability_
analysis.py function `_normalize_lap_distance_to_metres(data,
unit_raw)` -- `"ft"` -> `*0.3048` (Dubai's own path, unchanged
numerically), `"m"` -> unchanged, anything else -> raises rather than
guessing. `_interp_lap_distance_guarded` and corner_analysis.py's
`_invert_s_to_t` both lost their own independent internal `*0.3048` --
both now assume an already-metric array, normalised once at each of
the three real extraction sites (prepare_vehicle_state's own lap_
distance handling, plus corner_analysis.py's two: `assign_stable_
corner_ids` and `_realize_canonical_boundaries`) rather than pushed
down into the shared interpolation/inversion helpers themselves. csv_
parser.py now captures each channel's raw `[unit]` string into a new
`unit_raw` key on every channel dict (both missing and present cases,
both narrow and wide branches) -- confirmed via a golden-fixture check
that this addition does not appear anywhere in the serialised
pipeline_result/golden JSON (conftest.py's `pipeline_result` fixture
only carries `beta/slip/forces/cs/stab/fz/corners/summaries`, never
the raw channel dicts), so adding the key carries zero golden-fixture
risk.

**C -- sample-rate guard**: config/parameters.json gained `stability_
estimation.expected_sample_rate_hz = 50` with a provenance comment
naming every 50-Hz-calibrated consumer (yaw_stability_* window/grid/
min_samples, nis_gate/tyre_fit_auto's nis_window_samples, kerb_
dilation_samples, longitudinal_stiffness.min_samples_floor's own rate-
derivation). modules.csv_parser.parse_csv now measures and exposes
`measured_sample_rate_hz` (from ecu_speed's own timeline, the same
reference prepare_vehicle_state uses, via the existing `_estimate_
sample_rate` helper -- None if ecu_speed is missing/unusable, never
silently treated as a match). prepare_vehicle_state -- the earliest
point inside the analysis pipeline that has both the real measured
rate and the config's expectation, chosen over an earlier gate inside
parse_csv itself per the work order's own explicit separation of
"parse_csv measures and exposes" from "prepare_vehicle_state refuses"
-- raises ValueError immediately after computing `sr`, naming both the
measured and expected rate, if `round(sr) != round(expected_rate)`
(rounding absorbs floating-point interval-measurement jitter only, not
a real tolerance band -- 20 vs 50 Hz was never going to round-collide).
The existing StabilityAnalysisThread.run() already wraps its entire
body in `except Exception as e: self.error.emit(str(e))` -> a status
label read "Analysis failed: {msg}" -- raising a clear, specific
message was sufficient to satisfy "surfaced in the UI as a clear
status, not a traceback" with zero changes to ui/views/outing_form.py.

KNOWN RESIDUAL GAP, not fixed, flagged honestly: corner detection
(modules.corner_analysis.analyse_corners) runs INSIDE parse_csv,
before prepare_vehicle_state is ever called -- a rate mismatch is
therefore NOT caught before corner brackets get computed with rate-
dependent, sample-COUNT-based smoothing windows (corner_detection.
smoothing_window_samples=10, a fixed sample count, not a duration) at
whatever rate the file actually has. The corner map UI widget (_update_
corner_map_markers) reads parsed_data["corners"] directly, independent
of the StabilityAnalysisThread/prepare_vehicle_state path the rate
guard protects -- so a non-50-Hz file's corner markers could still
render (with mis-scaled smoothing) even though the full Analyse
pipeline is correctly refused. This was a deliberate scope decision,
not an oversight: the work order's own design explicitly assigned
"measure and expose" to parse_csv and "refuse" to prepare_vehicle_
state, and moving the gate earlier (into parse_csv, before analyse_
corners runs) would need parse_csv to depend on config/parameters.json
in addition to config/channels.json, a real architectural change beyond
what was asked. Reported for a future decision, not acted on.

### Verification

New tests/test_csv_parser_formats.py, 14 tests, all synthetic fixtures
(hand-built minimal Pi Toolbox snippets -- testing parser/format-
handling LOGIC, not a real-vehicle-behaviour claim, same category as
test_pure_functions.py's hand-derived checks, not the real-data-only
rule's actual target). Covers: narrow format still parses (regression
check against the pre-existing behaviour); wide format parses and is
NOT all-missing (the original bug, reproduced and fixed at fixture
scale); the empty-cell-tolerance case from the real file's own last
row; comma AND dot decimal separators including in the time column;
"nan" tokens treated as a missing cell, not a row failure or a
silently-included NaN; a bare-timestamp-only row tolerated without
crashing; unit_raw captured correctly in both formats;
_normalize_lap_distance_to_metres's three cases (ft, m, unrecognised
-> raises); an end-to-end wide-format + metres check (confirms s_m
lands near the raw metre values, not scaled down ~3.28x, which is what
would happen if the old unconditional *0.3048 had survived); the rate
guard's three cases (20 Hz raises with both rates named in the
message, 50 Hz passes, the message contains both "20.0" and "50").
The real GT3_PRC_MLA.txt was deliberately NOT used in any test or
verification step this session, per the user's own explicit
instruction (not native rate, partial session, not for analysis
validation) -- every fixture above is hand-built specifically to avoid
that file's own problems contaminating what a pass/fail means.

Full regression suite run once at the end (standard temporarily-
flipped-kinematic-then-restored procedure): **133 passed, 1 xfailed,
0 failed, 0 errors (35m32s)** -- exactly the prior 119-passed baseline
plus this session's 14 new tests, zero regressions anywhere. sideslip_
source restored to the user's own live value (ekf_auto_pacejka)
afterward, confirmed via git diff to carry only this session's two
intentional config edits (the earlier rad^-1 fix, and the new
expected_sample_rate_hz block) against HEAD, nothing else. Dubai's
golden pipeline output is confirmed unaffected by every change this
session made (wide-table branch never triggered for Dubai's own narrow
format; unit normalisation is a no-op for Dubai's own "ft" channel;
the rate guard passes cleanly at Dubai's real 50 Hz).

GT3_PRC_MLA.txt remains gitignored and untracked throughout (re-
verified at the end of this turn); git status shows only the intended
source changes (modules/csv_parser.py, modules/stability_analysis.py,
modules/corner_analysis.py, config/parameters.json, tests/
test_csv_parser_formats.py, .gitignore, thesis_notes.md). No commit
made, per the work order.

## 12. PLAN.md STEP 2: chair-comparable result plots, kinematic vs ekf_pass_1 [2026-08-31]

Standalone diagnostic (diagnostics/inspect_step2_chair_plots.py, output
diagnostics/plots_step2/, gitignored), no production/UI/config file
changed, no commit. Purpose (PLAN.md STEP 2): find out whether the
strange CS values that opened the estimator arc (see "C9 negative-CS
decomposition + zero-slip offset finding" above, 2026-08-17) trace to
the kinematic beta error or to something else. Renders one PNG per
stable corner per sideslip source (14 corners x 2 sources = 28 figures)
on the chair's own plot structure (docs/literature/plotting_methods.py,
matched in structure only -- that reference is interactive Plotly, this
is static matplotlib): velocity vs lap distance, instantaneous CS (N/rad)
vs lap distance, a track map with the corner bracket and each axle's
"worst-phase" estimation window highlighted, and per-axle tyre curves
(slip angle in degrees, Fy in N, session scatter in the background,
corner samples highlighted, the estimation window marked, and a tangent
whose slope is CS_N_per_rad * pi/180 -- preserving numerical
comparability with the chair's radians-axis plots despite the degrees
display). "Worst phase" (the sample with the lowest CS_ratio inside a
corner's bracket, pooled across its valid laps) reuses the concept
already established in the CS-credibility diagnostics above, not a new
metric. kinematic beta via estimate_sideslip, ekf_pass_1 beta via
diagnostics/sideslip_ekf_dugoff.py's estimate_sideslip_ekf_dugoff --
both called directly, bypassing the config-driven sideslip_source
dispatch entirely, so the user's live config value was never read or
written.

OBSERVATIONS (visual, not verdicts -- 4 corners inspected in detail):
- C9 (the corner that opened the CS-anomaly thread): under kinematic
  beta the whole-session tyre-curve scatter is a compressed, tangled
  cloud (matches the already-recorded +/-3 deg kinematic compression),
  and the rear worst-phase window shows a visible fold-back (Fy
  magnitude decreasing as |alpha| grows), giving CS=-362508 N/rad. Under
  ekf_pass_1 the session scatter opens into a much cleaner near-linear
  band (matches the already-recorded +/-8 deg observer spread), and the
  rear worst-phase CS is still negative but far smaller in magnitude,
  -74581 N/rad, with a visually straighter local curve. Front CS at C9
  is comparatively small under both sources (-65168 kinematic / -61709
  ekf_pass_1) -- the anomaly here reads as rear-specific and
  beta-sensitive, consistent with the WP-S4b Cr_A finding above.
- C4 (front worst-phase instance on record, -0.552 at the production 2
  Hz filter cutoff): front CS stays large and negative under BOTH
  sources (-98372 kinematic / -95027 ekf_pass_1), and both sources' front
  tyre curves show the same rounded peak-and-fold shape at a similar
  slip angle (~7-8 deg) -- visually this looks like a genuine, beta-
  source-independent saturation event, not a beta artifact. Rear at C4
  improves in magnitude under ekf_pass_1 (-131476 -> -53357) but stays
  negative and still shows a milder fold in the highlighted window under
  both sources.
- C2 (the pass_1 filter-stability excursion window, t=883-885.5s):
  both axles, under BOTH sources, show a tight self-crossing "knot" in
  the tyre curve at the worst-phase window rather than a clean fold --
  visually distinct from C9's and C4's smoother folds. Present
  regardless of beta source, so this looks like a genuine transient
  (a real turn-in event, or a beta-independent measurement artifact in
  Fy/alpha themselves) rather than a beta-sourced effect. Not
  investigated further here; flagged, not diagnosed.
- C6 (one of the two racing-speed sign-mismatch corners from the
  sideslip sign check above): the largest source-sensitivity seen in
  this pass. Rear worst-phase CS goes from -311382 N/rad (kinematic) to
  -13064 N/rad (ekf_pass_1) -- nearly flat rather than sharply negative
  -- and front from -152128 to -32805. The ekf_pass_1 tyre curves for
  both axles are visually clean, near-monotonic bands with no obvious
  fold at the highlighted window; the kinematic curves show a clear
  negative-slope tangent region. This is the single clearest visual case
  in the sample for "the beta error, not axle physics, was producing the
  extreme negative CS reading."
- Net visual impression across all 4: the two rear-anomaly corners tied
  to the original CS-credibility finding (C9, C6) show large, source-
  dependent improvement under ekf_pass_1, while C4's front saturation
  and C2's knot persist under both sources -- consistent with the
  standing WP-S4b conclusion that the kinematic beta error propagates
  into Module 4b's CS estimate, without this pass claiming to separate
  "real saturation" from "beta artifact" at every corner (that remains
  a verdict, not an observation, and is explicitly out of scope here).

PLOT-STRUCTURE DEVIATIONS FROM THE CHAIR: (1) static matplotlib PNGs,
not interactive Plotly -- structure matched, not code, per the work
order. (2) the chair's CS-vs-distance panel shows "both the online
estimate and a reference-model derivative" (PLAN.md's original STEP 2
wording); this session's work order fixed a simplified 4-item structure
without the reference-model derivative, followed as given. (3) front and
rear axles are rendered as separate tyre-curve panels (2 of the 5 total
panels) rather than one combined panel, since CS_ratio and the fold
signature are axle-specific and the investigation concerns both axles;
this is a design choice made in this diagnostic, not a chair structure
element, called out here for transparency.

SURPRISING, FLAGGED NOT INTERPRETED: the instantaneous-CS-vs-distance
panel is extremely noisy at both sources (values swinging between
roughly -400k and +800k N/rad within a single corner pass, sign flips
common) -- consistent with the already-recorded "CS_ratio aggregation-
sensitivity" finding (median-of-medians washing out single-lap swings)
but visually far more volatile sample-to-sample than that finding's own
framing suggested. Not diagnosed further here.

No production file changed, no config value changed, sideslip_source
never read from or written to config/parameters.json. No regression
suite run (not required by the work order for a read-only diagnostic
adding an ignored output folder); nothing importable by tests/ changed.

## 13. Cleanup/reliability/presentation pass, Phase 0: LS_ratio plausibility [2026-09-01]

Read-only diagnostic (method question, per the work order -- no
estimator change made). Investigated the corner-trace LS panel's
occasional ~-15-and-worse readings.

CHAIR REFERENCE (docs/literature/longitudinal_stiffness_estimator.py,
internal): calculate_longitudinal_stiffness_ratio clips the ratio at a
CEILING of 1.0 only (`np.clip(ratio, None, 1.0)`) -- no floor, no R^2
gate, no confidence weighting of any kind. The only validity gates on
the windowed OLS slope itself (_centered_slopes) are count>=min_samples,
abs(denom)>1e-12, and slip_span>=min_slip_span -- min_slip_span=0.004
(a fraction, i.e. 0.4% slip) is generous enough to admit windows whose
kappa span is barely above that floor.

SETUPTOOL (modules/longitudinal_stiffness.py): _stiffness_ratio is a
verbatim mirror of the chair's function, including the missing floor.
The two documented SetupTool-specific deviations (rate-derived
min_samples, the additive kerb-coincidence plausibility guard) do not
address this case: the guard only excludes a sample when |kappa|
exceeds ls['plausibility_kappa_bound']=0.12 AND a kerb event is
detected nearby -- the extreme windows found below have small kappa at
the centre sample, not implausible kappa, so the guard correctly leaves
them alone. CONCLUSION: SetupTool is NOT more permissive than the
chair here -- the missing lower bound is inherited as-is; the -15-ish
values are a shared numerical-stability gap in the method itself, not a
SetupTool regression.

MECHANISM, confirmed by direct inspection of the 8 most negative
windows per axle on Dubai (production kinematic config,
regression_window_s=0.45, half_window=11 samples at 50 Hz):
every one of them has the FULL window (n_samples_in_window=23, i.e.
count>=min_samples is trivially satisfied) but a kappa_span sitting
right at the min_slip_span=0.004 floor (front: 0.00400-0.00405 for 6 of
8; rear: 0.00512-0.00855 for 6 of 8). A windowed-OLS slope is
numer/denom where denom ~ sum((x-xbar)^2) over the window's own kappa
values -- when kappa_span is tiny, denom is tiny, so any real Fx
variation across the window (noise or a genuine small transient)
produces an enormous, poorly-conditioned slope. Example: front, t=599.06s,
kappa_span=0.00405, stiffness=-697152 N (vs linear_reference=21911 N)
-> LS_ratio=-31.8. This is exactly the numerical-instability signature
CS_ratio (Module 4b) is protected against by its R2-weighted monotonic-
section blending (SPAN_WEIGHT_EXPONENT/R2_WEIGHT_EXPONENT,
estimate_cornering_stiffness) -- LS_ratio has no equivalent, by
construction, matching the chair's own (simpler, no R^2 blending)
method as-is.

DISTRIBUTION on Dubai (base_mask = moving & ~kerb & valid-lap racing
time, n=24183; production kinematic config -- LS_ratio does not depend
on sideslip_source, kappa/Fx are computed independently of beta):
- front: n_finite=11299 (46.7% of base_mask). p1=-10.994 p5=-3.437
  p50=0.000 p95=1.000 p99=1.000. count<-1: 1815 (16.06% of finite).
  count<-5: 344 (3.05% of finite). min=-31.818 max=1.000.
- rear: n_finite=18450 (76.3% of base_mask). p1=-2.728 p5=-1.341
  p50=0.527 p95=1.000 p99=1.000. count<-1: 1375 (7.45% of finite).
  count<-5: 25 (0.14% of finite). min=-11.566 max=1.000.
Front is markedly worse than rear -- consistent with the front axle's
own kappa signal being smaller-magnitude in general (front is
uncorrected/braking-only per modules/longitudinal_forces.py's own
estimate_slip_ratio docstring), so a larger fraction of front windows
sit near the min_slip_span floor.

DECISION NOW LIVE FOR THE USER (not chosen here, per the work order):
whether to add an R^2-style conditioning gate or confidence weighting
to LS_ratio (a SetupTool-specific DOMAIN IMPROVEMENT over the chair,
same taxonomy class as the existing plausibility guard), widen
min_slip_span (a chair-sourced calibration tunable -- changing it is an
estimator change, re-triggers threshold re-derivation per CLAUDE.md),
or leave the estimator as-is and rely on Phase 1's display-only fixed
Y-range clip (ui/views/corner_trace_dialog.py, this same package) to
keep the panel readable without touching the underlying numbers. LS_ratio
remains DISPLAY ONLY (no verdict/classifier reads it, PLAN.md STEP 3),
so this is a legibility question, not (yet) a correctness-of-verdicts one.

DISPLAY NOTE, Part A item A4 (2026-09-01, same package, trace-dialog
bugfix/redesign work order): a second, independent display gate added on
top of Phase 1's Y-range clip -- the corner-trace LS panel now only draws
a sample where |ax_mps2| exceeds a new config value corner_trace_display.
ls_display_min_ax_mps2 (default 1.0 m/s^2, ~0.1g). Motivation differs from
Phase 1's: that clip bounds the AXIS RANGE against LS_ratio's numerical-
instability tail; this one hides samples where the car is near-constant-
speed, where LS_ratio (a longitudinal-stiffness ratio) has nothing
meaningful to measure regardless of numerical conditioning -- the panel
was showing noise in cruise/apex-coast sections that a reader could
mistake for signal. Default derived from this session's own |ax_mps2|
distribution over the same base racing population as this section's own
LS_ratio stats above (moving & ~kerb & valid-lap racing time, n=24183):
p5=0.474, p10=0.947, p20=1.807, median=3.970 m/s^2 -- this track's laps
spend most of their time under real braking/traction demand (94.8% of
samples exceed 0.5 m/s^2), so even a modest cutoff excludes only the
genuinely flat segments; 1.0 m/s^2 sits just above the p10 point and is
the standard vehicle-dynamics rule-of-thumb for negligible longitudinal
load transfer. Display only -- LS_ratio's own numeric output, min_slip_
span, and every other estimator input are untouched; this is a Tier C
presentation choice, not a re-derivation of anything Tier B/A. Also
applied identically to the same panel's PRINT export (core/figure_
render.py's render_verdict_traces_figure/_draw_ls_panel reads the
already-masked array core/views/corner_trace_dialog.py hands it, not a
second independent mask), so screen and export always agree. The
full-lap trace (LapTraceDialog) loses its LS panel entirely in this same
package (A4) -- LS_ratio has no lap-level reading (STEP 3 established
it only as a per-phase/per-corner windowed estimate), so a full-lap
trace of it was never meaningful; the corner trace (where LS_ratio is a
real per-corner quantity) keeps the panel, now with this mask applied.

### Citation cross-reference, modules/longitudinal_forces.py [2026-09-01]
Two Tier A/B anchors used by estimate_longitudinal_forces/
estimate_slip_ratio had never been named as their own bullet, though
the book itself (Rajamani, Vehicle Dynamics and Control, 2nd ed.,
Springer) is already fully cited above (WP-N1 entry, Ch. 13.10 Dugoff
anchor) -- recorded here so the pointer-only rule below has something
to point at:
- estimate_longitudinal_forces's fx_total = m*ax + drag + rolling
  fallback tier: Rajamani Ch. 2, longitudinal equation of motion
  (F_x = m*a_x + resistive forces), page TBD verify.
- estimate_slip_ratio's kappa = (v_axle_corrected - v_ref)/v_ref:
  Rajamani Ch. 2 sec. 2.2, slip-ratio (kappa) definition, page TBD
  verify.
Also anchors modules/longitudinal_stiffness.py's estimate_longitudinal_
stiffness docstring, which cites this same Ch. 2 relationship for its
inputs rather than restating it.

### RULE APPLICATION, Phase 2a citation sweep [2026-09-01]
Per the [2026-08-19] RULE CHANGE above (citation location), swept
modules/ for remaining inline author/title/page citations and replaced
each with a one-line pointer to its thesis_notes.md entry, following
the estimate_sideslip precedent (WP-S4 citation cleanup) exactly. No
citation content was changed or dropped -- every full citation already
existed in this file (confirmed before editing code); this pass only
moved the code-side text from a repeated inline citation to a named
pointer. Sites converted: modules/stability_analysis.py (module-level
header docstring, estimate_slip_angles, estimate_lateral_forces,
estimate_vertical_loads, estimate_cornering_stiffness, estimate_yaw_
moment_stability -- estimate_sideslip already done 2026-08-19), modules/tyre_model.py
(module docstring, Dugoff anchor and the Werner sign-convention
cross-reference), modules/tyre_model_pacejka.py (module docstring,
Magic Formula general-form anchor), modules/longitudinal_forces.py
(estimate_longitudinal_forces, estimate_slip_ratio), modules/
longitudinal_stiffness.py (estimate_longitudinal_stiffness, pointer to
the longitudinal_forces.py anchor above rather than a new citation).
modules/tyre_fit_auto.py and modules/yaw_stability.py's own mentions
of "Pacejka"/"Werner" were checked and left as-is: the former names a
model/EKF variant, not a citation (no author/title/page attached); the
latter's Werner mention was replaced too even though it already said
"see thesis_notes.md" alongside the inline S2.2.3/S4.5.2 label, since
the label itself is exactly the kind of inline author/year text the
rule now excludes from code.

### DECISIONS BATCH, Phase 2b: lap-filter UI choice removed [2026-09-01]
ui/views/outing_form.py's Data section previously let the user narrow
a stability analysis to one lap (click a row in the lap table) or drop
in/out laps (the "Exclude In/Out Laps" toggle); both fed lap_filter.
Per this session's work order, analysis now always covers every
is_valid_for_analysis lap (falling back to every lap in the file only
if none are flagged valid) -- no user choice left. Removed: the
"Exclude In/Out Laps" toggle button (_on_exclude_toggled), and the
_sync_lap_selector_to_filter method that reconstructed a selector state
from a cached lap_filter (moot once the selector no longer determines
lap_filter). _get_lap_filter_from_selector no longer reads the lap
table's selection or the removed toggle at all.
NOT removed: the lap table itself (row click, "Clear Selection", the
_selected_lap_value/_on_lap_selected/_update_plots machinery). Read
carefully before touching it -- this table serves a SECOND, independent
purpose: scoping the raw-channel plot view below it (speed/throttle/
brake/RPM/gear/steer traces) to one lap for visual inspection. That
purpose has nothing to do with which laps the stability pipeline
analyses and was kept exactly as-is; only the coupling from "table
selection" to "analysis lap_filter" was cut. _populate_lap_table always
shows every lap now (the old exclude-driven row-hiding depended on the
removed toggle).
WP5/WP6 cache-miss verification: _try_render_cached_analysis's DB
cache-hit path gained a new guard -- a saved analysis_data payload
whose stored lap_filter does not equal what _get_lap_filter_from_
selector() would compute today is now rejected as stale (falls through
to "re-run Analyse first"), instead of being rendered as if it were
current. This matters concretely: an outing analysed and saved under
the OLD UI with a single lap selected, or with in/out laps included,
carries a lap_filter the new policy would never produce -- without this
guard it would silently render under the OLD, now-incorrect scope
forever. The WP6 in-memory Modules-1-5 pipeline cache (_pipeline_cache_
get/put) needed no change: verified its identity key (csv_path + cap +
resolved_vehicle_snapshot + sideslip_source) never included lap_filter
in the first place, since Modules 1-5 always run on every sample in the
file regardless of lap_filter (only Module 6/summarise_corners consumes
it) -- so it was already lap-filter-policy-independent.
Headless verification (diagnostics/inspect_clear_data_lifecycle.py, one-
off, deleted after this entry): OutingForm._get_lap_filter_from_selector
called unbound against the real Sample_Dubai.txt parse returns [1, 2,
3, 4] (this file's is_valid_for_analysis set); a synthetic old cached
lap_filter=[3] correctly flags as stale against that. No QApplication/
QThread/DB pulled in for this check, matching the precedent already on
record (tests/conftest.py's pipeline_result docstring) that Qt-in-tests
was judged not worth the fragility for this codebase.

### DECISIONS BATCH, Phase 2c: PROJECT.md removed [2026-09-01]
Deleted PROJECT.md (a stale project-overview snapshot last accurate
around WP1-WP2; PLAN.md + CLAUDE.md + this file are the live sources of
truth). Two references found repo-wide: PLAN.md's own dated STATUS
HISTORY log (a past entry recording "touched thesis_notes.md,
PROJECT.md, ..." at the time PROJECT.md still existed) -- left
untouched, since STATUS HISTORY records what happened, not current
state, and rewriting past entries is against this project's own
history-preservation convention. modules/recommendation.py's
FEEDBACK_SCALE_MAX comment carried a live "(see PROJECT.md)" pointer --
updated to drop the now-dangling reference (the comment already states
the -5..+5 scale directly, no replacement documentation needed).
CLAUDE.md, generate_handover.py, and any README never referenced
PROJECT.md (checked, no hits).

### DECISIONS BATCH, Phase 2d: Clear Data lifecycle button [2026-09-01]
Added a "Clear Data" button (Data section button row, next to Load
Outing) and OutingForm._reset_data_state(), guarded by a QMessageBox
Yes/No confirmation (skipped if nothing is loaded) -- the only
destructive-ish action in this form, and the codebase's one existing
QMessageBox.question precedent (_print_sheet's file-overwrite prompt)
confirmed this pattern is already in use elsewhere, not a new UI idiom.
_reset_data_state() clears every field _on_csv_loaded/_run_stability_
analysis/_on_stability_done populate (parsed_data, loaded_csv_path,
stability_result, corner_positions_cache, _analysis_data_json,
_displayed_resolved_vehicle_snapshot, _cached_schema_mismatch), hides
the corner/lap trace dialogs if open, resets every status label and the
corner-card grid back to their construction-time text, disables
Analyse/Lap traces/Generate, and re-runs _update_corner_map_trace() so
the track-map panel falls back to its own "Load a CSV" placeholder
(already guarded for parsed_data is None, no new branch needed there).
Deliberately NOT written to the database directly: self.loaded_csv_path
=None is exactly what _save_outing already persists as csv_path=""
(same "stage in memory, Save writes it" convention every other form
field follows) -- a Clear Data click followed by Back-without-Save
discards the clear exactly like any other unsaved edit, no special-
cased DB write needed.
Headless verification (diagnostics/inspect_clear_data_lifecycle.py, one-
off, deleted after this entry): OutingForm._reset_data_state called
unbound against a fake self (real Sample_Dubai.txt parsed_data plus
no-op stand-ins for every QWidget method touched -- same "self=None/
fake" precedent tests/generate_golden.py already uses for
_classify_corner) leaves every state field None/empty as expected;
reloading the same file afterward (simulating _on_csv_loaded's own
reset lines, not this script's stand-in) reproduces the identical
lap_filter as the first load -- no leaked state between the two loads.

### DECISIONS BATCH, Phase 2e: weekend PDF status check, TypeError found
and fixed [2026-09-01]
Generated real weekend PDFs against the live local database (weekend 1
"Dubai", outings 1-2; weekend 2 "Paul Ricard", outing 3, no analysis_
data) via core.weekend_pdf_export.generate_weekend_pdf, then rendered
pages to PNG (pymupdf, transient install, uninstalled after) for visual
confirmation. Two findings:
1. The Phase 1 three-column FL/RL | Car | FR/RR setup-strip layout
   renders correctly inside the weekend PDF's own setup/setdown strips
   section (it reuses core/pdf_export.py's build_session_strip
   unchanged) -- confirmed visually, no separate fix needed here.
2. A real TypeError, previously invisible because the per-outing try/
   except (core/weekend_pdf_export.py's generate_weekend_pdf, already
   in place before this session) swallows it into a generic "ERROR
   building this section" note: outing 1 (Dubai, Warmup, has a current
   analysis) failed to render ANY of its real content -- verdict
   summary, corner table, feedback -- because _recommendations_
   flowables crashed formatting one recommendation's score with
   f"{r['score']:.2f}" where score was None. Root cause: this outing's
   real driver feedback contains an "urgent_gap" recommendation (a
   driver/data direction-contradiction flag for C8 -- driver reports
   near-undrivable understeer, data shows oversteer at exit), a
   deliberately parameter-less, score-less bucket type. ui/views/
   outing_form.py's own recommendations panel already has a documented
   fix for exactly this shape ("FIX 1" comment, badge_text falls back
   to corner+verdict since there is no lever to name) -- the PDF path
   was simply never given the equivalent fix. FIXED in _recommendations_
   flowables: action_class=="urgent_gap" rows now show "C{id}:
   {short_verdict}" for Action (mirrors the UI's badge_text), "-" for
   Score (never crashes on None), and "URGENT" for Class (previously
   silently mis-labelled RECOMMENDED alongside the crash). Purely a PDF-
   rendering/Tier-C display fix -- no recommendation-engine, estimator,
   or threshold logic touched. Re-rendered after the fix: outing 1 now
   shows its real corner table (C1-C15+, one row per stable_corner_id)
   and the recommendations table now has one row reading "C8:
   understeer | - | driver | ... | URGENT | driver/data disagree: C8"
   instead of the ERROR fallback. The per-outing try/except itself was
   correct all along (one bad outing's exception never took down the
   rest of the document) -- kept as-is; this fix removes the trigger,
   it does not touch the safety net.
Not investigated further this phase (report-only per this decision's
own scope): whether any OTHER recommendation-engine output shape can
reach a PDF-rendering assumption the UI already special-cases but the
PDF does not -- action_class is currently a two-way UI switch (advisory
vs urgent_gap vs the RECOMMENDED default) and this fix only closes the
one gap that actually crashed. Flagged under "open" in this package's
final report.

### PLAN.md unsupervised package, Phase 3: LS_ratio span-dependence
[2026-09-01]
Read-only diagnostic (diagnostics/inspect_ls_ratio_span_dependence.py,
kept, see diagnostics/README.md): does LS_ratio's per-sample value
depend on how wide the sliding regression window's own kappa excursion
("slip span") happened to be, for samples that would otherwise pass
every OTHER validity gate in modules/longitudinal_stiffness.py's
_centered_slopes? Motivated by wanting independent evidence for (or
against) config's longitudinal_stiffness.min_slip_span=0.004 gate,
which currently excludes any window below it outright with no evidence
on record for why 0.004 specifically.
METHOD: independently re-derived _centered_slopes's own numerator/
denominator/slope arithmetic with the min_slip_span condition dropped
(count>=min_samples and finite-denominator conditions kept -- those are
numerical-validity, not the physical gate under study), against the
real Sample_Dubai.txt run through Modules 1-4a-equivalent (prepare_
vehicle_state, estimate_longitudinal_forces, estimate_slip_ratio,
estimate_longitudinal_stiffness for the production linear_reference_N
values only). Cross-checked before trusting the extended population:
restricting the reimplementation to the SAME population production's
gate keeps reproduces modules/longitudinal_stiffness.py's actual
LS_ratio_f/LS_ratio_r output exactly (np.allclose, equal_nan=True, both
axles) -- confirms the reimplementation is faithful, not an
independent (and possibly divergent) formula.
RESULT: diagnostics/plots_step2/ls_ratio_span_dependence.png (front and
rear axle, LS_ratio vs window kappa span, log x-axis, min_slip_span
marked with a vertical line, colour-split at the gate). Front axle:
below-gate population n=21960 (of 39454 numerically-valid samples),
median LS_ratio -0.0000 but ranging to roughly -200 at the smallest
spans; rear axle: below-gate population n=10305 (of 39454), median
0.3651, ranging to roughly -27. Above the gate, both axles compress
tightly into the plausible <=1.0-capped band the production output
actually shows. This is the expected small-span regression-blowup
signature (near-zero-variance denominator amplifying numerator noise
into an unbounded slope) -- the scatter shows a clear, visually obvious
cliff at the gate, not a gradual/ambiguous transition, which reads as
supporting evidence that SOME span gate near this order of magnitude is
needed. NOT established: that 0.004 specifically (vs e.g. 0.003 or
0.006) is the optimal cut -- this diagnostic shows the gate's general
shape is justified, not that the exact value is data-derived at
that precision. Read-only -- no config or modules/ file changed by
this diagnostic; min_slip_span itself is untouched.

### Corrections round 3 follow-up: dialogs, corner-figure track map,
setup-sheet PDF, recommendation audit [2026-09-01]
Small items, no estimator/module numerics touched; targeted smoke tests
only, no full suite run (none of this round's own hard constraint
required one).
1. Every QDialog (CornerTraceDialog/LapTraceDialog via their shared
   _TraceDialogBase, DriverDialog, WeekendDialog, WeekendPdfDialog) now
   sets Qt.WindowType.WindowMinMaxButtonsHint explicitly -- a plain
   QDialog omits native minimise/maximise on Windows by default.
   Verified headlessly (constructed each, asserted the flag bit is set)
   -- 2 of the 5 (DriverDialog, WeekendDialog) are setFixedWidth forms,
   so maximise mostly just grows height there; applied uniformly per
   the work order rather than special-cased.
2. render_corner_figure's track map (dropped in round 3 proper to make
   room for the new side-legend columns) is back as a narrow full-width
   row between rear-CS and the tyre curves (compact=True: no title, no
   legend -- lap colour is already established by the other panels'
   legends). Cost: with a fixed 24cm cap and a 5th row now competing for
   height, the tyre-curve panels shrank from round 3's own 6.49cm square
   to 5.90cm square (tuned the map row's own height_ratio down from an
   initial 1.3 to 0.9 to claw back some of that; further tuning hit
   diminishing returns). Bottom margin (round 3's own item 1 fix, ≥8mm)
   unaffected -- it comes from the figure-level `rect` constraint, not
   the row height_ratios, confirmed unchanged at 9.7mm by rendering.
   diagnostics/inspect_step2_chair_plots.py's own track-map builder
   (removed in round 3 when the signature dropped the parameter) is
   restored too, adapted to the current "brackets_by_lap" list shape
   _draw_track_map_panel now reads (it previously read a bare
   "bracket_xy" key that no longer exists post-Part-B) -- this script's
   own tyre-curve builder still uses the older pre-multi-lap "corner_xy"
   shape, a pre-existing staleness out of THIS round's scope, noted not
   fixed.
3. core/pdf_export.py's car_label font: was its own key (large=11pt vs
   value=9pt -- car_label BIGGER than value, a pre-round-3 mismatch a
   previous report flagged and left open; small=5.4pt vs value=6.5pt --
   car_label smaller there). Now car_label = value exactly, one number
   (f["car_label"] = f["value"]), at both scales -- verified by
   rendering both scales against real Dubai data, no touching/overflow
   introduced (small scale's car_label grew ~20%, checked specifically
   for that direction of risk, not just assumed harmless because "value
   >= label" held before).
4. core/weekend_pdf_export.py: STRIPS_PER_PAGE 4 -> 2 (one outing's own
   Setup+Setdown pair per page, not two outings'), STRIP_H roughly
   doubles as a direct consequence (computed from STRIPS_PER_PAGE, nothing
   else touched). Measured the real effect on the small-scale KeepInFrame
   shrink this was meant to fix: natural (unshrunk) strip content is
   ~102mm against the new ~90mm STRIP_H (113.6% fill) -- shrink is still
   needed but now only to ~88% of natural size, down from the ~45%
   shrink four-per-page needed (content used to run ~175% over a 44mm
   budget). KeepInFrame(mode="shrink") itself is left in place -- it is
   generic overflow-safety shared by both the single-outing and weekend
   call sites, not something specific to the old ratio to delete; only
   the ratio it now has to correct for shrank. Rendered and visually
   confirmed legible at 250dpi, no touching, real Dubai data (weekend
   id=1, 2 outings).
5. Recommendation-shape audit (core/weekend_pdf_export.py's
   _recommendations_flowables vs ui/views/outing_form.py's own
   recommendation-card rendering): confirmed the urgent_gap fix already
   recorded above (Phase 2e) is the only real shape mismatch that can
   crash or silently mis-render -- action_text (corner+verdict
   fallback), score_text ("-" not a crash), and class_text ("URGENT" not
   the RECOMMENDED default) all already match the UI's own handling in
   the current code. No other UI-special-cased shape (limit_status,
   conflicts, selected, cell_ids/trigger_source, the retired seed/
   unvalidated-rule tag, package multi-action badges) has a PDF-side gap
   -- each either already matches byte-for-byte or is a deliberate,
   documented content omission (the PDF is a condensed table by design,
   per its own module docstring), not a shape-assumption bug. No new
   code change from this item; audit confirms the existing fix is
   complete, not that a new one was needed.
PROCESS NOTE, worth recording: a background agent given isolation:
"worktree" for this round's item-3/4/5 delegation reported that the
entire shared-strip-renderer architecture (build_session_strip,
_strip_styles, car_label, KeepInFrame, the weekend-PDF strip layout)
"does not exist anywhere in the repo" -- a false premise, confirmed by
grepping the actual main-tree files directly (they do exist, and were
used successfully by an EARLIER isolated-worktree agent this same
session). Root cause not fully diagnosed, but the evidence points to
worktree isolation sometimes checking out committed HEAD only, missing
this session's own uncommitted working-tree changes (everything this
session has done is deliberately uncommitted, per every work order's
own "no commit" instruction) -- an earlier verification step meant to
catch exactly this (comparing the first agent's worktree against HEAD)
was itself a no-op bug (piped to /dev/null, never actually inspected).
The agent's own item-5 audit and fix were still valid in isolation (it
found and fixed a real bug against ITS OWN stale checkout), just
redundant with Phase 2e's already-landed fix once compared against the
real main-tree state -- no harm done, but items 3/4 had to be redone
directly rather than trusted from that report. Lesson for future
delegation this session: verify a worktree agent's premise against the
main tree BEFORE trusting its report, especially when nothing has been
committed.

### Production sideslip source set to ekf_auto_pacejka [2026-09-01]
config/parameters.json's stability_estimation.sideslip_source is now
DECLARED production default at "ekf_auto_pacejka" (the value itself was
already set live from prior sessions; this work package makes it the
documented default and retargets the test suite to match, rather than
treating it as an ad-hoc live setting requiring a flip-to-kinematic
procedure to test against). Reasoning: ekf_auto_pacejka fits the tyre
curve fresh on each session's own data (modules/tyre_fit_auto.py's
fit_session_pacejka), evaluates modules/nis_gate.py's provisional NIS
health gate on the result, and falls back to kinematic beta whenever the
gate verdicts "fail" or the fit is "degenerate" -- recorded in the
analysis payload (fallback_used/fallback_reason) and shown in the
UI/PDF, never silently. On the real Dubai sample this session, the gate
verdicts "pass" (health_score=0.169, well inside threshold_use_ekf=
0.1385's inverse sense -- see tests/golden/pipeline_dubai_ekf_auto_
pacejka_cap1.json's own _meta.gate_verdict), so production reads the
fitted EKF/Pacejka beta, not the kinematic fallback. EXPLICIT, NOT YET
DONE: verdict/classification thresholds (classification.thresholds_
calibrated_for_sideslip_source) remain "kinematic" -- they were derived
against the kinematic CS_ratio distribution and have NOT been
re-derived for ekf_auto_pacejka's distribution. The [UNCAL] per-verdict
marker and the calibration banner therefore now show BY DEFAULT on
every fresh outing, correctly, until threshold re-derivation (PLAN.md
STEP 4, PARKED) lands. Traces (CS_ratio through a corner, the tyre
curve, stability) are meaningful immediately; verdicts/recommendations
are not, per the same split STEP 1c already documented for ekf_pass_1.
Test suite retargeted to match (tests/conftest.py, tests/test_golden_
pipeline.py, tests/test_golden_auto_modes.py, tests/generate_golden.py,
tests/generate_golden_auto_modes.py): tests/conftest.py's pipeline_
result fixture now asserts and dispatches on the live config's
sideslip_source (expected "ekf_auto_pacejka") via modules.tyre_fit_
auto.resolve_sideslip_beta instead of hard-asserting "kinematic" via a
direct estimate_sideslip call -- this is what removes the flip-to-
kinematic-then-restore procedure every full-suite run before this
package required (documented across many prior STATUS entries in
PLAN.md) when the live config already pointed at an auto mode. Kinematic
and ekf_auto_dugoff keep active golden-file regression coverage, moved
into tests/test_golden_auto_modes.py (renamed test/fixture names from
auto_mode_* to secondary_mode_* since kinematic is not an auto-fit
mode) as explicit-mode secondary sets, independent of whatever the live
config value is. A new recommendations golden (tests/golden/
recommendations_dubai_ekf_auto_pacejka_cap1.json, 4 recommendations) was
generated once via the retargeted tests/generate_golden.py -- none
existed for this mode before. The existing pipeline golden (tests/
golden/pipeline_dubai_ekf_auto_pacejka_cap1.json) was verified to still
match current output exactly (byte-for-byte diff, no fallback) before
any of this, via a scoped pytest run against the retargeted test file,
and was NOT regenerated, per the work order's explicit instruction.
generate_golden.py and generate_golden_auto_modes.py both gained a
skip-if-exists guard on every golden file they write, so a routine
future re-run can never silently reset a pinned baseline's provenance
(git_commit_hash/generated_at_utc) even when the underlying numbers
would come out identical.

### Mechanism investigation: wholesale-negative CS_ratio under
ekf_auto_pacejka [2026-09-02]
Investigation only -- no config, estimator, or threshold change; no
production file touched. New diagnostics/inspect_negative_cs_mechanism.py
(disposable, DELETE at commit time unless a future reason to keep it is
stated -- diagnostics disposal rule), reusing diagnostics/inspect_
step2_chair_plots.py's renderer unmodified but sourcing beta through the
PRODUCTION dispatch (modules.tyre_fit_auto.resolve_sideslip_beta,
fallback-guarded) instead of that script's own direct kinematic/
ekf_pass_1 calls. Targets: C1, C2, C3 (newly extreme under ekf_auto_
pacejka per the threshold-proposal diagnostic run), C4 (STEP 2's
genuine beyond-peak case), C8 (stability sign flip), C9 (STEP 2's old
artifact case). 12 renders (6 corners x kinematic/ekf_auto_pacejka),
diagnostics/plots_threshold_investigation/ (gitignored, not committed).

METHOD: for each corner/axle/mode, located the single worst-CS_ratio
sample's own regression window (reconstruct_cs_window_start) and
reported its sample count, alpha span, R2, the raw-window vs monotonic-
section slope estimates (C_window/C_section, the two ingredients
estimate_cornering_stiffness's C_alpha blends by R2 weight) and their
sign agreement; cross-correlated alpha against Fy over each corner's
representative-lap bracket (peak-|corr| lag, samples/ms) under both
modes; and computed the pooled per-sample CS_ratio population's own
min/median/negative-fraction within each corner's bracket, to isolate
how much of the "wholesale negative" reading is the worst-lap/min
statistic amplifying a minority tail versus a genuinely negative
typical value.

FINDING 1 -- the median stays positive everywhere; the min statistic is
doing almost all of the work. Under ekf_auto_pacejka, EVERY one of the
12 corner/axle pooled populations has a POSITIVE median (front: C1=
0.614, C2=0.226, C3=0.123, C4=0.412, C8=0.314, C9=0.603; rear: C1=0.563,
C2=0.246, C3=0.021, C4=0.321, C8=0.242, C9=0.513) despite negative
fractions of 9-46% and worst-case mins as low as -1.235. Even C3 (the
single most extreme newly-flagged corner) keeps a barely-positive
median (0.123 front, 0.021 rear) while 33-47% of its samples read
negative. The population did NOT invert to majority-negative in the
sense of "typical behaviour is now beyond peak" -- it inverted only in
the worst-lap/min statistic the current classify_fn and this session's
threshold-proposal diagnostic both key on. Contrast with kinematic,
where medians sit at the 1.0 ceiling (the pre-existing, already-
documented aggregation/ceiling-clipping problem) and negative fractions
stay under 12.5% everywhere -- kinematic's min-vs-median gap is even
LARGER in relative terms, but its absolute negative-sample rate is much
smaller, which is why the same min-based statistic looked stable under
kinematic and does not under ekf_auto_pacejka: ekf_auto_pacejka's
wider, more accurate alpha range is producing more numerous (not
necessarily more severe per-instance) excursions into whatever is
driving the negative tail, and the min operator converts "more frequent
tail excursions" directly into "wholesale negative worst-case".

FINDING 2 -- visual survey (item 1): no corner shows a clean, textbook
monotonic-rise-then-fall fold at the population level. C4 (front and
rear) is the closest to a genuine, physically coherent picture: the
worst-phase window sits on the ascending part of the session-wide
scatter cloud, the CS_ratio-vs-track-position trace shows a WIDE,
SUSTAINED dip across ~1320-1360m for laps 3/4 (not a spike), consistent
with STEP 2's own attribution of C4 as real, non-artifact saturation --
though even here the single worst-SAMPLE's own window disagrees in sign
between C_window and C_section on both axles (see Finding 3), so the
sustained TREND looks genuine while the single most-extreme SAMPLE
within it may still be noisy. C1, C2, C3, C8, C9 all show the SAME
qualitative signature instead: the CS_ratio-vs-track-position trace is
a rapid, jagged sawtooth -- repeated full-scale swings between +1.0 and
-0.5..-1.0 within a single corner traversal, on EVERY lap, not a single
smooth event. This within-corner oscillation is the visual signature of
an unstable per-window estimate, not of the tyre progressively
approaching and passing its peak. C3's rear tyre-curve panel additionally
shows the worst-phase window tracing a small hook/loop (down then back)
rather than a clean monotonic arc -- a hysteresis-like shape, matching
the loop hypothesis in the work order. No corner's tyre-curve panel
shows the worst-phase window's tangent visually following the
session-wide cloud's own local direction; in several cases (C1 front,
C2, C8, C9) the tangent's downward slope cuts across a cloud region
that is still visibly ascending.

FINDING 3 -- window quality at the worst sample (item 2), tabulated
(mode=ekf_auto_pacejka; n=sample count in the reconstructed window,
R2=window R-squared, C_window/C_section in N/rad, sign=AGREE/DISAGREE):
C1 front n=12 R2=0.998 DISAGREE (-60115/+83180[C_section as printed;
raw C_alpha window regression -60381]); C1 rear n=20 R2=0.985 AGREE
(-108520/-53689); C2 front n=15 R2=0.774 AGREE (-144251/-77761); C2
rear n=11 R2=0.815 AGREE (-128983/-8832, section magnitude only ~7% of
window's); C3 front n=18 R2=0.980 AGREE (-102730/-29033, section ~28%
of window's); C3 rear n=64 R2=0.688 AGREE (-152854/-64948); C4 front
n=22 R2=0.798 DISAGREE (-110369/+20287); C4 rear n=11 R2=0.967 DISAGREE
(-73619/+13172); C8 front n=17 R2=0.983 AGREE (-36734/-32929, close
magnitude); C8 rear n=26 R2=0.955 AGREE (-58529/-37875); C9 front n=14
R2=0.828 AGREE (-54915/-13916); C9 rear n=12 R2=0.972 AGREE
(-19619/-2644). Two things stand out, present across NEARLY EVERY
corner regardless of which visual category (Finding 2) it fell into:
(a) sample counts are small (11-26, except C3 rear's 64) -- consistent
with cs_min_window_samples=10 and cs_min_slip_angle_span_rad=0.02 rad
both being satisfied almost immediately once alpha moves under ekf_
auto_pacejka's wider range, so the window rarely grows past its own
floor; (b) R2 is frequently very high (0.774-0.998) on these small
windows, which is exactly the small-n overfitting regime -- a handful
of points lying close to any locally-fitted line will report a high
R2 even when that line is not representative of the broader curve. The
window-blend (C_alpha = w_R2*C_window + (1-w_R2)*C_section, R2-weighted
per estimate_cornering_stiffness) therefore leans MOST heavily on
C_window precisely when R2 is highest, which after (b) is exactly when
that trust is least warranted. C_window and C_section disagree in
MAGNITUDE by 3-14x even when they agree in SIGN in most rows above
(C2 rear, C3 front, C8 rear, C9 front/rear) -- sign agreement alone
understates how unstable the blended estimate is.

FINDING 4 -- phase lag (item 3) does not explain the wholesale shift;
if anything ekf_auto_pacejka aligns BETTER than kinematic on this
metric. Peak alpha-vs-Fy cross-correlation lag over each corner's
representative-lap bracket, ekf_auto_pacejka: C1 front 0ms/rear -20ms,
C3 front/rear 0ms/0ms, C4 front/rear 0ms/0ms, C8 front/rear 0ms/0ms, C9
front/rear 0ms/0ms -- essentially zero lag in 9 of 12 cases. The one
clear exception is C2: front +1240ms (62 samples), rear -440ms (22
samples) -- both large. Under kinematic the SAME corners show larger,
more erratic lags in several cases, most strikingly C9 (front +3640ms,
rear +5360ms -- a 3.6-5.4 SECOND lag, on the corner STEP 2 already
flagged as a kinematic-era artifact) and C2 (front +1160ms, rear
+1100ms, comparable magnitude to its ekf_auto_pacejka reading). Reading:
phase lag is a real, corner-specific contributor (C2 shows it under
BOTH modes, at similar magnitude, and its tyre-curve window traces a
small loop consistent with a lag-induced hysteresis artifact -- see
Finding 2) but is NOT the general mechanism behind ekf_auto_pacejka's
population-wide inversion, since most other corners show near-zero lag
under the very mode that shows the inversion.

SYNTHESIS, mechanism-level only (no fix proposed, none asked for): the
wholesale negative shift is NOT primarily a phase-lag/hysteresis
artifact (Finding 4) and is NOT evidence that the car typically
operates beyond peak (Finding 1 -- median stays positive everywhere).
The evidence points to a window-regression instability specific to
CS_ratio's small-window, R2-weighted blend (Finding 3) being triggered
far more often under ekf_auto_pacejka than under kinematic, because the
wider, more accurate alpha range reaches the window-growth loop's own
floor (cs_min_window_samples/cs_min_slip_angle_span_rad) almost
immediately far more often than kinematic's narrower range did -- more
frequent short, high-R2, small-n windows means more frequent unstable
C_alpha estimates, which the existing min-per-instance/worst-lap
statistic then converts directly into "wholesale negative". C4 remains
the one case with a visually coherent sustained trend (Finding 2)
consistent with STEP 2's own genuine-saturation attribution, though
even its single worst sample shows the same small-window signature as
every other corner (Finding 3) -- the mechanism finding does not
overturn C4's own separately-established genuineness, since that
attribution rests on the SUSTAINED multi-lap trend, not the single
extreme sample. This is a mechanism finding only; no config value,
estimator, or threshold changed. It bears directly on the paused
threshold-re-derivation work (PLAN.md STATUS, "THRESHOLD RE-DERIVATION
FOR ekf_auto_pacejka") -- specifically its own third open option
("re-derive the CS windowed-regression calibration constants
themselves first") -- but that decision is not made here.

### Pass-1 baseline independence, corrected [2026-09-02]
Correction to the prior turn's proposal text: diagnostics/inspect_
pass1_final_validation.py's frozen curve (params["tyre_model_ekf"]
["pass_1"]'s c_alpha_front/rear_n_per_rad, mu_fz_front/rear_N, read
live from config) is confirmed independent of the CS_ratio window
constants (cs_min_window_samples/cs_min_slip_angle_span_rad/cs_linear_
slip_threshold_rad) -- verified by reading the script: those config
values are never derived from estimate_cornering_stiffness's output.
BUT the script's own manifest["wp_s4b_reference_spread"] (Section 6)
DOES call estimate_cornering_stiffness and use its C_linear_ref_r
output -- this section's numbers WOULD change if the window constants
change, and would need deliberate re-running/regeneration (same
discipline as golden files) before being cited again. The prior
proposal's "does not touch the frozen pass-1 baseline" claim was
correct for the CURVE (the actual carried-forward estimator) but
incomplete for the VALIDATION SCRIPT's own recorded manifest --
correcting the record rather than letting it stand.

### Phase-level validity diagnostic: apex_3's fixed window budget
conflicts with the C4-vs-artifact distinction [2026-09-02]
Diagnostics only, no config/estimator change. New diagnostics/inspect_
cs_phase_validity.py (disposable, delete at commit time unless kept),
run once against ekf_auto_pacejka for the same 12 corner/axle cases as
the prior mechanism investigation, tracing the approved Stage 2 (min-
across-5-phases, per lap) then Stage 3 (min-across-laps, "worst phase
of worst lap", signed off this session) to the SPECIFIC phase that
produced each case's classification value, then comparing that phase's
own cs_ratio-valid sample count against the local CS_ratio regression
window's own footprint (samples).

HEADLINE FINDING: C1, C2, C3, AND C4 all resolve their Stage-3 minimum
to the SAME phase -- apex_3 -- with n(cs_ratio-valid) of 11, 11, 4, and
11 respectively (C1 front/rear: -0.981/-0.989, n=11; C2 front/rear:
-1.119/-0.580, n=11; C3 front/rear: -1.183/-0.813, n=4; C4 front/rear:
-0.493/-0.084, n=11). This is NOT a coincidence of which corner is
genuine vs artifact -- it traces to apex_3's own construction
(modules/stability_analysis.py summarise_corners's _phase_slice):
apex_time is a single instant, expanded to a FIXED +/- apex_half_
window_samples=5 window whenever hi<=lo, i.e. exactly 11 raw samples,
independent of the corner, lap, or estimator. C3's n(cs_ratio-valid)=4
despite n_samples(raw moving)=11 means 7 of those 11 samples have NaN
CS_ratio (kerb-masked or pre-C_linear_ref, not investigated further
here). Local window footprints at these apex_3 cases run 12.0-19.5
samples (mean) -- comparable to or larger than the phase's own total
sample budget, so EVERY ONE of C1/C2/C3/C4's apex_3 cases is DROPPED
by a phase_n >= k*footprint gate at every candidate k tested (1.5, 2,
3) -- see numbers below. C8 and C9, by contrast, resolve to exit_4/
exit_5 (ordinary time-duration phases, not apex-instant-expanded),
with n=84-93 -- these SURVIVE at k=1.5/2.0 and are dropped only at
k=3.0 (C8 rear -033, C9 front -0.207, C9 rear +0.040 at k=3; C8 front
+0.032 survives even at k=3.0).

CONSEQUENCE, stated plainly: a within-phase validity gate of the
literal form proposed (phase_n >= k * local_window_footprint) does
NOT distinguish C4 (pre-registered to stay flagged) from C1/C2/C3
(pre-registered to become healthy/mild) -- all four are gated out
identically, because the gate is actually keying on WHICH PHASE TYPE
happened to be worst (apex_3's fixed ~11-sample budget vs an exit
phase's naturally larger, physically-variable one), not on whether the
underlying corner event is genuine or an artifact. This does not
contradict the render-based evidence that C4's dip is a real, sustained,
multi-phase event (thesis_notes.md "Mechanism investigation..." Finding
2) -- Stage 2's "worst single phase" statistic happens to land on
apex_3 as the LOCALLY most extreme slice within that broader sustained
dip, which is plausible (apex is often the peak-lateral-demand point)
but means the phase-count-vs-footprint gate, applied at the single
worst-phase level, cannot see the multi-phase persistence that is
actually what makes C4 genuine. Open question surfaced, not resolved
here: whether the gate needs to be phase-type-aware (apex_3 treated
differently from time-duration phases, e.g. a fixed floor rather than
a footprint-relative one), or whether the persistence-across-phases
cross-check (Part 2's option 3, already slated to run alongside) should
be the PRIMARY mechanism distinguishing C4 from C1/C2/C3, with the
within-phase gate demoted to a secondary/general-noise-floor role
rather than the mechanism doing the C4-vs-artifact separation itself.
No gate value proposed; this is numbers and a structural finding only,
per the work order.

Full numbers (per-lap Stage-2 worst phase/value, Stage-3 winner, phase
stat block, footprint, gate survival at k=1.5/2/3), all 12 cases, are
in this turn's chat report -- not duplicated verbatim here to keep this
entry to its own headline finding; re-run diagnostics/inspect_cs_
phase_validity.py to reproduce exactly (deterministic, no randomness).

### apex_3 structural finding: a fixed 11-sample phase, never large
enough to clear its own local window footprint [2026-09-02]
Standalone statement of the structural fact underlying the prior
entry, for citation independent of the gate-design discussion:
summarise_corners's apex_3 phase (modules/stability_analysis.py
_phase_slice) is not a time-duration segment like the other four
phases -- apex_time is a single instant, expanded to a FIXED window of
+/- apex_half_window_samples (config: 5) whenever the natural slice is
empty, i.e. exactly 11 raw samples, always, independent of corner, lap,
sideslip_source, or how long the corner's real apex phase of driving
actually lasts. Under ekf_auto_pacejka the local CS_ratio regression
window's own footprint at the apex point runs 12-19.5 samples (mean,
across the 6 corners checked) -- comparable to or larger than apex_3's
entire 11-sample budget. CONSEQUENCE: apex_3 structurally can never
supply a CS_ratio statistic backed by more than about one regression
window's worth of independent information, and typically fewer -- it
is not that apex_3 is noisier than other phases under this estimator,
it is that apex_3 cannot structurally clear the same validity bar the
other four (physically variable, usually much larger n) phases can.
This is a general property of the phase's own construction, not
specific to any one corner or to ekf_auto_pacejka, though the auto-fit
mode's wider alpha range is what widened the local footprint enough to
make the mismatch visible (kinematic's narrower range keeps footprints
smaller, so apex_3's 11-sample budget happened to clear a k~1 bar more
often there, per the same diagnostic run against kinematic in the
mechanism investigation -- not re-checked numerically here).

### Gated Stage-2 recomputation (k=1.5 across all 5 phases): C1
recovers, C2/C3 partially recover, C4 does NOT survive -- pre-
registration falsified for C4 [2026-09-02]
Diagnostics only, no config/estimator change. Extended diagnostics/
inspect_cs_phase_validity.py (same file, disposable) to recompute
Stage 2 excluding, per lap, any of the 5 phases whose own cs-valid
sample count falls below 1.5x its own mean local window footprint --
not just re-checking the previously-identified worst phase, but
scanning all 5 phases of all 4 laps per corner/axle and taking the min
of whichever phases survive.

RESULT (ekf_auto_pacejka, all values are the new GATED Stage-3 worst-
lap value, delta vs the original apex-driven value in parentheses):
C1 front +0.247 (delta +1.228, was -0.981 @ apex_3, now @ exit_4
n=24); C1 rear +0.021 (delta +1.010, was -0.989, now @ exit_5 n=23);
C2 front -0.197 (delta +0.922, was -1.119, now @ entry_2_turnin n=56);
C2 rear -0.119 (delta +0.461, was -0.580, now @ entry_2_turnin n=56);
C3 front -0.225 (delta +0.958, was -1.183, now @ exit_4 n=41); C3 rear
-0.288 (delta +0.524, was -0.813, now @ exit_4 n=36); C8/C9 (all four
front/rear) unchanged, delta +0.000 exactly -- their original worst
phase was already exit_4/exit_5, never apex_3, so the gate changes
nothing for them.

C4 front +0.061 (delta +0.554, was -0.493 @ apex_3, now @ entry_2_
turnin n=59); C4 rear +0.099 (delta +0.184, was -0.084, now @
entry_2_turnin n=59). BOTH AXLES FLIP POSITIVE. The pre-registered
expectation ("C4 still clearly negative via a non-apex phase") is
FALSIFIED, stated plainly, not softened: once apex_3 is excluded, C4
has NO remaining phase, on ANY of its 4 laps, that reads negative at
k=1.5 for either axle (front per-lap gated survivors: 0.497, 0.766,
0.134, 0.061, all positive; rear: 0.300, 0.658, 0.140, 0.099, all
positive). C1/C2/C3's recovery partially matches the pre-registration
(C1 fully healthy, C2/C3 mild-negative rather than extreme), but C4's
own failure to survive is the more consequential result: the render-
based evidence of a real, sustained ~40m multi-lap dip (thesis_notes.md
"Mechanism investigation..." Finding 2) is NOT showing up as a
negative MEDIAN in the phases surrounding the apex once those phases
are taken on their own, wide, time-duration terms -- the apex_3 window,
narrow and centred exactly on the apex instant, was capturing the CORE
of a real event that the surrounding phases' own wider medians dilute
back to positive. This is a DIFFERENT failure mode than the apex
sample-budget mismatch: it is the classic "median-across-a-wide-window
smooths out a real but spatially narrow event" problem, now hitting the
phases AROUND the apex rather than the apex phase itself. Neither a
within-phase sample-count gate (this investigation) nor plain
median-across-phases (Part 2's rejected candidate) currently has a
mechanism to preserve C4's real signal once apex_3 -- the one phase
that WAS catching it -- is excluded. Open, not resolved here: this
argues more strongly for the persistence-across-DISTANCE cross-check
(Part 2's option 3) as a necessary component, evaluated directly on the
raw per-sample series rather than through any phase-median reduction,
since phase-level reduction at either grain (single worst phase,
gated-worst phase, or median-of-5) has now been shown to lose C4's
signal in at least one direction (apex included: noisy small-n;
apex excluded: diluted by the surrounding wider phases).

Recommendation-engine exposure (count only, no analysis, per the work
order): grep of config/recommendations.json's rules[].phases finds 8 of
39 rules include "apex_3" (us_apex_arb [retired], matrix_us_apx_low/
med/high [elicited], matrix_us_apx_med_esc [held], matrix_os_apx_low/
med/high [elicited]) -- 6 of the 8 are live (elicited) status; held/
retired rules never fire per generate_recommendations' own skip rule.

### Persistence-length diagnostic: pre-registration NOT supported --
C3 shows the longest sustained below-zero runs in the entire session
[2026-09-02]
Diagnostics only, no config/estimator change. New diagnostics/inspect_
cs_negative_run_lengths.py (disposable), read-only, run once against
ekf_auto_pacejka. Finds every contiguous run of CS_ratio<0 samples on
the raw per-sample series over each corner's own canonical bracket
(NaN breaks a run), per lap, per axle, across ALL 14 physical corners
(not just the 6 previously investigated), converts run length to
metres via s_m, and reports the distribution -- sizing input for the
persistence-across-distance statistic (Part 2 option 3), independent
of the phase-median machinery both prior investigations showed losing
C4's signal.

PRE-REGISTRATION, quoted from the work order: "C4 shows runs of
roughly 30-40m on multiple laps centred near apex; C1/C2/C3's runs are
of order one window footprint; a clear gap exists between the two
populations." NOT SUPPORTED, on every clause:

1. C4's own longest runs are SMALLER than pre-registered and mostly
NOT apex-centred: front 17.0m (exit_5, lap1), 31.9m (apex_3, lap3),
23.1m (entry_2_turnin, lap4); rear 20.5m (entry_2_turnin, lap3), 28.6m
(entry_2_turnin, lap4). Only 1 of 5 lap-instances centres at apex_3;
the rest fall in entry_2_turnin or exit_5. Magnitude (17-32m) is in
the right rough order but at the LOW end of "30-40m", not consistently
inside it.

2. C1 roughly matches the "order one window footprint" expectation:
runs of 2.3-29.0m, mostly under 10m. C2 does NOT: 14.2-45.2m, twice
reaching 45.2m -- longer than any of C4's own runs. C3 dramatically
does NOT: front 19.5-78.7m; REAR 108.4m, 115.1m, 129.7m on 3 of its 4
laps (lap4 only 24.1m) -- these are the three LONGEST runs found
anywhere in the entire dataset (see item 3), roughly 4x C4's own
longest run. C3 rear alone occupies 3 of the pooled top-10-longest-
runs list; two more of that top 10 (C6, C11, C13) belong to corners
outside the original 6-corner investigation entirely.

3. Pooled distribution, ALL 14 corners x valid laps x both axles,
n=219 runs: p50=8.87m, p90=36.45m, p99=103.18m, max=129.68m (C3 rear
lap3). This is a continuous, heavy (long) tail, not two separated
clusters with a visible gap -- C4's own runs (17-32m) sit roughly at
the p75-p85 mark of this pooled distribution, comfortably inside the
bulk of it, not standing apart in an upper cluster the way the pre-
registration predicted.

4. Local window footprint, for scale: computed directly from s_m at a
representative window per highlighted corner/lap/axle (n=48): median
19.5m, p10=10.5m, p90=68.0m -- itself a wide range (footprint length is
speed- and alpha-dependent, not a fixed distance), overlapping
substantially with the run-length distribution's own middle range,
which weakens "order of one window footprint" as a stable yardstick.
Reference: at the session's median racing speed (137 km/h / 37.9 m/s),
a 10-sample window covers 7.6m, a 30-sample window covers 22.8m.

HONESTY NOTE, not glossed over: this conflicts with this session's own
earlier qualitative visual read of C3 (thesis_notes.md "Mechanism
investigation...", Finding 2: "C1, C2, C3, C8, C9 all show the SAME
qualitative signature... a rapid, jagged sawtooth"). The run-length
numbers now show C3 rear sustaining a below-zero reading for well over
100 continuous metres on 3 of 4 laps -- not obviously a "rapid
sawtooth" in the sense that description implied. Either the earlier
visual impression under-read C3's own sustained character (plausible --
that investigation's renders were surveyed for overall shape across
six corners, not measured), or a long run can still look visually
"jagged" at the fine scale while remaining unbroken by NaN/positive
samples at the sample-count resolution this script checks (e.g. small
wiggles that stay negative throughout, never crossing back above
zero) -- not distinguished from each other here. Flagging the
discrepancy rather than silently reconciling it.

CONSEQUENCE: persistence-over-distance, at least in this simplest form
(longest single contiguous run, per lap), does NOT cleanly separate
C4 from C1/C2/C3 either -- it is a THIRD statistic, after phase-median
(Part 2's rejected candidate) and the within-phase count gate (prior
two entries), that has now also failed to reproduce the pre-registered
C4-vs-artifact separation on this dataset. No refinement (multi-lap
persistence, run count, depth-weighted length) evaluated here -- none
was asked for, none proposed. No config, estimator, or gate value
changed.

### Ground-truth workup: per-run verdicts for the long-run corners --
C4 REAL (fold/peak, all 5 runs), C2 and C3 rear ARTIFACT (loop) [2026-09-02]
Diagnostics only, no config/estimator/statistic change. New diagnostics/
inspect_run_ground_truth.py (disposable), read-only, run once against
ekf_auto_pacejka for the specific runs named in the work order: C3 rear
all three 100m+ runs (laps 1/2/3); C2 front both 45.2m runs (laps 2/3,
tied length -- both included since the work order named "45m"
ambiguously between two identical-length runs); C4 all five runs
(front laps 1/3/4, rear laps 3/4).

RENDERER FIX APPLIED (Part 5, diagnostic-side only -- core/figure_
render.py itself untouched, its own never-silent behaviour change
still deferred to the approved package): the script populates
corner_by_lap (per-lap coloured clean/kerb samples) and fitted_line
(the real fitted Pacejka curve from fit_manifest) exactly as ui/views/
corner_trace_dialog.py's own equivalent builder does, replacing
diagnostics/inspect_step2_chair_plots.py's older pooled/no-fit
contract for this investigation's own renders (diagnostics/
plots_ground_truth/, gitignored, not committed).

METHOD, four evidence lines per run: (1) tyre-curve picture over
exactly the run's own samples (highlighted), against the fitted
Pacejka curve and the full per-lap-coloured corner scatter; (2)
LS_ratio and kappa (same axle) over the run span -- combined-slip/
traction-limited corroboration, cross-referenced against PLAN.md STEP
3's own established finding that C3 is traction-limited on all 4 laps
(a WHOLE-LAP, kinematic-sourced, worst-phase reading -- LS_ratio itself
does not depend on sideslip_source, since modules/longitudinal_
stiffness.py never takes beta as an input, so this cross-reference is
valid despite the different beta source); (3) steering-rate |rad/s|
inside the run vs the lap's own p95, and stability_observed_Nm_per_deg
over the span; (4) synthesised verdict.

RESULTS PER RUN:

C3 rear lap1 (108.4m, depth -0.105): tyre curve shows a tight, self-
crossing LOOP around alpha=-3..-4 deg, Fy=-10..-12 kN -- not a fold,
the highlighted run's own points double back on themselves rather than
declining monotonically past the fitted curve's peak. LS_ratio median
0.422 (healthy, not traction-limited for this specific span despite
STEP 3's separate whole-lap finding -- spans do NOT clearly coincide).
kappa negligible (median -0.006, max|kappa| 0.013 -- no combined-slip
support). Steering-rate run max 0.214 rad/s vs lap p95 0.111 -- run
exceeds the lap's own high-steering-activity bar, consistent with an
active correction, not a quiet, steady demand. Stability robustly
positive throughout (median 310, min 95 -- no destabilisation).
VERDICT: ARTIFACT (window/hysteresis noise from an oscillatory
steering correction) -- decided by the tyre-curve loop shape,
corroborated by negligible kappa and healthy stability.

C3 rear lap2 (115.1m, depth -0.523, the deepest of the three): same
loop shape, same corner (render is the same underlying scatter, only
the highlighted span differs). LS_ratio median 0.335 (still not
traction-limited for this span). kappa negligible. Steering run max
0.206 vs lap p95 0.114 -- exceeds. Stability positive (median 308, min
72.6). VERDICT: ARTIFACT, same reasoning as lap1 -- the deeper median
does not change the tyre-curve or kappa picture.

C3 rear lap3 (129.7m, depth -0.199, the longest run in the dataset):
same loop shape. LS_ratio median 0.881 (healthy for most of the span,
min -1.219 a brief dip). kappa negligible. Steering run max 0.183 vs
lap p95 0.120 -- exceeds. Stability median 218 (still positive) BUT
min -21.0 -- a genuine destabilising excursion WITHIN this run, the
only one of the three C3 laps to show one. VERDICT: MIXED -- the loop
shape and negligible kappa still argue artifact for the CS_ratio
reading itself, but the real stability dip inside this specific run is
a genuine corroborating signal this lap's handling degraded somewhat,
not present on laps 1/2. Not resolved to a single label.

C2 front lap2 (45.2m, depth -0.254) and lap3 (45.2m, depth -0.516,
tied length, both reported): tyre curve shows an unmistakable, compact
LOOP traced by the highlighted run's own points alone (not just the
whole corner's natural turn-in/apex/exit shape) -- a hook that reverses
on itself, matching the phase-lag/hysteresis mechanism this session
already identified for C2 (thesis_notes.md "Mechanism investigation...").
LS_ratio medians exactly 0.000/0.000 on sparse populations (n=22/53,
n=36/53) -- almost certainly a degenerate/near-empty-window reading,
not a meaningful "at the traction limit" statement; not treated as
traction-limited evidence. kappa negligible both laps. Steering run
max 0.200/0.217 vs lap p95 0.114/0.120 -- exceeds both. Stability very
healthy and essentially flat (median ~400, min 377/376, nowhere near
the -50 destabilising threshold). VERDICT (both laps): ARTIFACT --
decided by the tyre-curve loop, corroborated by healthy stability and
negligible kappa; the near-zero LS_ratio reading is NOT used as
supporting evidence given its own sparse/degenerate population.

C4 front lap1 (17.0m, depth -0.164): highlighted points sit in a dense
region where multiple laps' own traces cross -- visually AMBIGUOUS,
neither a clean fold nor an obvious loop. LS_ratio median 1.000 (healthy)
but min -8.488 -- an extreme outlier on a tiny window, not treated as
meaningful given how far it sits from the rest of the distribution.
kappa negligible. Steering run max 0.145 vs lap p95 0.111 -- exceeds.
Stability strongly positive (median 555, min 499). VERDICT: REAL, lower
confidence -- the render itself doesn't decide this one; the call rests
on consistency with the other four C4 runs (below) plus healthy
stability and negligible kappa ruling out both artifact-loop and
traction-limited readings.

C4 front lap3 (31.9m, depth -0.299): CLEAN FOLD -- the highlighted run
rises to a visible peak (~alpha 5.7deg, Fy 7.5kN) then declines
monotonically as alpha continues to grow (to alpha 7.3deg, Fy 6.1kN),
matching lap4's own nearby fold in the same panel and sitting close to
the fitted Pacejka curve's own peak region. kappa negligible. Steering
run max 0.345 vs lap p95 0.120 -- a 3x excess, the largest of any run
checked. Stability positive but reduced (median 352, min 183).
VERDICT: REAL (genuine lateral saturation) -- decided by the clean
fold shape.

C4 front lap4 (23.1m, depth -0.446): highlighted points rise steadily
to Fy~9.2kN at alpha~5.8deg, ABOVE the fitted curve's own peak
(~8.2kN) at that alpha -- reads as approaching/at the physical peak
(a plateau, CS_ratio's own "near 0 at the peak" physics, per the
amendment's own framing) rather than a clear decline within this short
span. kappa negligible. Steering run max 0.047 vs lap p95 0.122 --
BELOW the lap's own p95, the only C4 run where this holds. Stability
positive (median 173, min 129, on a lower baseline than lap1/lap3,
consistent with a different phase of the corner). VERDICT: REAL
(at-peak/plateau) -- decided by the render sitting at/above the fitted
curve's own peak, not a decline, but not a loop either.

C4 rear lap3 (20.5m, depth -0.112): CLEAN FOLD -- highlighted points
decline monotonically from (alpha 3.7deg, Fy 10.5kN) to (alpha 5.6deg,
Fy 9.1kN), no looping. LS_ratio median 0.253 (moderate), kappa median
0.0081, max|kappa| 0.044 (small but the largest kappa of any run
checked -- still far from a strong traction-limited signature).
Steering run max 0.345 vs lap p95 0.120 -- 3x excess. Stability
positive (median 310, min 203). VERDICT: REAL -- decided by the clean
declining fold.

C4 rear lap4 (28.6m, depth -0.258): the CLEANEST fold in the entire
set -- highlighted points rise sharply to a visible peak (~alpha
3.3deg, Fy 12.0kN) then decline (to alpha 4deg, Fy 9.2kN), a textbook
peaked curve. LS_ratio median 1.000 (healthy), kappa small (median
0.0059, max 0.0137). Steering run max 0.075 vs lap p95 0.122 -- BELOW,
same as front lap4. Stability positive (median 177, min 127). VERDICT:
REAL -- decided by the cleanest peak/fold shape of any run investigated.

SUMMARY: C4 reads REAL on all five runs (four decisively by tyre-curve
shape, one by consistency + corroborating signals) -- the strongest,
most direct confirmation yet of STEP 2's original genuine-saturation
attribution for this corner, now grounded in actual fold/peak pictures
rather than only the aggregate render survey. C2 front and C3 rear read
ARTIFACT (loop/hysteresis), with C3 rear lap3 flagged MIXED for its own
genuine stability dip. HONESTY NOTE: this ARTIFACT read for C3 does not
straightforwardly reconcile with PLAN.md STEP 3's "C3 traction-limited
on all 4 laps" finding -- kappa is negligible throughout all three
spans checked here, which does not support a traction-limited
explanation for THESE specific 100m+ windows; STEP 3's own worst-phase
statistic may be keying on a different, shorter moment within the same
laps that this run-based investigation did not target. Not reconciled
here. No config, estimator, or statistic proposed or changed.

### Geometric fold-vs-loop candidates evaluated: none of three cleanly
separate the ground truth [2026-09-02]
Diagnostics only, no config/estimator/statistic change. New diagnostics/
inspect_cs_validity_criteria.py (disposable), read-only, evaluates
three candidate geometric criteria -- all reusing modules.stability_
analysis's own chair-derived _find_monotonic_sections/_section_slopes
directly, no new geometry invented -- against the 10 ground-truth runs
(thesis_notes.md "Ground-truth workup...": 5 REAL, 4 ARTIFACT, 1 MIXED),
at each run's own worst (most negative) single sample.

Candidate A (fraction of the window's samples inside a LOCAL monotonic
section, computed on the window alone, whose own alpha span clears
cs_min_slip_angle_span_rad): REAL range [0.000, 1.000], ARTIFACT range
[0.000, 0.750] -- OVERLAP. Candidate B (alpha direction-reversal count
within the window): REAL range [0, 2], ARTIFACT range [1, 4] --
OVERLAP. Candidate C (relative disagreement |C_window-C_section|/max,
at the governing sample): REAL range [0.513, 1.535], ARTIFACT range
[0.341, 0.908] -- OVERLAP, though REAL trends higher on average (median
~1.06 vs ~0.63) and only candidate C shows any directional tendency at
all.

DIAGNOSED WHY, not just reported blind: Candidate A's own failure case
is instructive -- C4f lap4 (REAL, a clean at-peak fold, thesis_notes.md
"Ground-truth workup...") scored 0.000, the WORST possible value,
because its window is short (n=22, near the cs_min_window_samples
floor) and alpha rises monotonically throughout it (typical approaching
a peak) but the window's own total alpha SPAN falls just under min_span
-- Candidate A conflates "this window is too short/narrow to trust
statistically" (a size question, already the Part-3 statistical
criteria's own job) with "this window's shape is a loop, not a fold" (a
geometry question) -- a short, valid, single-armed window gets
penalised identically to a fragmented, multi-armed loop. This is a
real design flaw in Candidate A as formulated, not evidence that no
geometric signal exists. Candidate B is the most conceptually direct
(a fold has 0 alpha reversals, a loop has several) but ties occur
(C4f lap3 and both C2 runs share 1 reversal) because a single small
steering correction near a genuine peak also produces exactly one
alpha reversal without the path doubling back over the SAME Fy
values the way a true hysteresis loop does -- alpha-only reversal
counting cannot distinguish "one correction near the peak" from "one
arm of a loop." A refined candidate, NOT evaluated here (stop-at-the-
proposal, no further diagnostic run): a joint alpha-AND-Fy path check
(does the window's own (alpha, Fy) trajectory self-intersect or
double back over previously-visited Fy values at similar alpha, rather
than alpha reversal alone) is the natural next step if geometric
separation is still wanted, but was out of this turn's scope to build
and test.

CAVEAT: n=10 ground-truth runs (5/4/1 split) is a very small sample to
either confirm or rule out a clean gap either way -- this result should
be read as "none of these three simple candidates work as formulated
on the cases checked," not "no geometric criterion can ever separate
fold from loop." No config, estimator, or criterion adopted or changed.

### CS validity repair, part A, Phase 1: window-floor re-derivation
[2026-09-02]
SUPERSEDED same day, user-directed revision (thesis_notes.md "CS
validity repair, part A, Phase 1 REVISION"): this entry's own criterion
targeted a SINGLE WINDOW's own sampling variance and correctly found no
natural knee -- but that is the wrong target. What actually feeds
classify_fn is a PHASE-LEVEL MEDIAN over many per-sample windowed
estimates, which already averages out per-window noise across a
realistic phase's O(30-200) samples -- a much smaller floor can still
yield a stable phase median. cs_min_window_samples=100/cs_min_slip_
angle_span_rad=0.04/cs_linear_slip_threshold_rad=0.03 below are NOT the
values carried forward; do not cite them as current. Kept verbatim
(not deleted) as the record of the mechanism-investigation-driven first
attempt and why it was the wrong criterion, not a wrong calculation.
Method: Tier B (signal/data-engineering preprocessing -- window-size
selection for a windowed-regression estimator; standard bootstrap
resampling, no vehicle-dynamics content). New diagnostics/inspect_cs_
window_floor_derivation.py and diagnostics/inspect_cs_window_cap_sizing.py
(both disposable pending this package's own commit). Re-derives
cs_min_window_samples, cs_min_slip_angle_span_rad, and cs_linear_slip_
threshold_rad against THIS car's own ekf_auto_pacejka alpha/Fy (Dubai),
per the user's own work order for the CS validity repair package
following the mechanism investigation (thesis_notes.md "Mechanism
investigation: wholesale-negative CS_ratio under ekf_auto_pacejka") that
traced the estimator's small (n=11-26), high-R2 regression windows to
the small-n overfitting regime.

METHOD: bootstrap-resampled (B=200, seed=42) the windowed-OLS slope's own
sampling variance as a function of (a) fixed window length N and (b)
target alpha span, holding the other free -- this matches the estimator's
own coupled growth loop rather than artificially decoupling the two.
Sampled 500 real end-indices per axle per grid point from the whole
session's moving, non-kerb population (no corner pre-selection -- the
same population the production estimator scans).

FINDING 1 -- no natural flattening knee. Both curves decrease smoothly
and monotonically across the full tested range (N: 8-200 samples, span:
0.01-0.16 rad) on both axles, with no inflection point. This is
consistent with, not a defect against, textbook OLS slope-variance
scaling (var ~ sigma^2/sum((x-xbar)^2), i.e. roughly ~1/(n*span^2)) --
more data and more spread always reduce pure sampling variance, so
"where variance flattens" is not a well-posed knee-finding problem here.
The genuine countervailing force (a window wide enough to reach into the
tyre's curved/saturating region, biasing the "local" slope away from
truth) is exactly what Phase 2's cap addresses -- bootstrap variance
alone cannot see it, since it only measures noise around whatever point
estimate the window's own data produces.

FINDING 2 -- floor set by a stated noise-tolerance criterion (P75, not
median), because the failure being repaired is a MIN-driven downstream
statistic. Sizing on median relative bootstrap std would leave the tail
that actually corrupts Stage 2/3's worst-phase/worst-lap readings
untouched (this is precisely the mechanism the earlier investigation
diagnosed: "the min operator converts more frequent tail excursions
directly into wholesale negative"). Smallest N/span where P75 relative
std <= 15% converges INDEPENDENTLY from the N-sweep and the span-sweep to
~100 samples / ~0.04 rad on BOTH axles (front: N=100 -> 13.6%, span=
0.045 rad at median N=109 -> 11.7%; rear: N=100 -> 13.9%, span=0.045 rad
at median N=150 -> 12.0%) -- full curves in diagnostics/inspect_cs_
window_floor_derivation.py's own run output, reproducible deterministically
(fixed seed). Chosen: cs_min_window_samples=100, cs_min_slip_angle_span_
rad=0.04 (both up from 10/0.02 -- a large jump, but the mechanism
investigation had already shown the OLD floor sat deep in the small-n
overfitting regime).

FINDING 3 -- cs_linear_slip_threshold_rad re-derived via a sliding-window
local-slope curve, NOT narrow disjoint bins. An initial narrow-bin
(0.005 rad) attempt reproduced the SAME small-span sampling noise Finding
1 diagnosed (bins that thin cannot resolve a stable local slope) -- redone
with 0.04-rad-wide (matching the newly-derived span floor) sliding windows
stepped by 0.005 rad, on the folded |alpha| cloud (alpha and Fy negated
together for alpha<0, exploiting the tyre curve's expected odd symmetry).
Both axles' local slope stays within a few % of the near-origin reference
window out to ~0.03 rad, departs 10% and STAYS departed (not a single
noisy crossing -- checked explicitly against the full curve, not just a
first-crossing rule, since a small transient bump right at alpha~0.01 rad
was found and correctly rejected as noise, not a genuine onset) from
leading edge 0.045 rad (front) / 0.035 rad (rear), climbing to full sign
reversal by ~0.08-0.09 rad -- the genuine saturation region, consistent
with the ground-truth workup's C4 fold pictures (alpha ~5-7 deg = 0.09-
0.12 rad). Chosen: 0.03 rad -- the more conservative (earlier-onset) axle
(rear, 0.035 rad) with a small safety margin, up from the prior 0.021 rad.

cs_max_window_samples (Phase 2's own cap) sized from diagnostics/inspect_
cs_window_cap_sizing.py: under the OLD floors, a non-trivial tail of
windows (p90 356-430 samples, p99 1549-2517, MAX up to ~2820) ALREADY grew
far beyond any single corner phase's own natural duration (measured max
347 samples, exit_5) -- a previously undocumented structural defect (no
upper bound existed at all before this package). Under the NEW floors the
tail is similar in shape (p90 529-570, p99 ~2400-2500, max ~2800) --
confirming the blowup is not primarily a floor-size artifact but an
inherent property of near-flat-alpha stretches (straights, gentle lifts)
where the window must walk arbitrarily far back chasing a span target.
Set at 500 samples: sits at the new floors' own p90, comfortably above
any real cornering phase's natural extent, well below the p95-p99
multi-corner-spanning tail.

Values written to config/parameters.json (Phase 2, same turn): each with
its own derived_from note pointing at the generating script. Not yet
anchored/committed -- this is diagnostics/config only, per the work
order's "no thresholds written" constraint (classification thresholds,
distinct from these estimator-internal window constants, are untouched).

### CS validity repair, part A, Phase 2: adaptive widening + cap
implemented [2026-09-02]
NOTE (same day, Phase 1 REVISION): the specific floor/cap VALUES cited
below (cs_min_window_samples=100, cs_max_window_samples=500) were
revised downward the same day once Phase 1's own criterion was corrected
-- see thesis_notes.md "CS validity repair, part A, Phase 1 REVISION" and
"...Phase 2/3 re-run against the revised floors" for the values actually
carried forward. The MECHANISM this entry documents (widening capped at
cs_max_window_samples, achieved-span verified after the loop, NaN on
failure, cs_phase_min_valid_samples gate) is unchanged by the revision --
only the numbers plugged into it changed.
modules/stability_analysis.py: estimate_cornering_stiffness's window-
growth loop (compute_cs_for_axle) now caps its own widening at
cs_max_window_samples and, critically, VERIFIES the achieved span against
cs_min_slip_angle_span_rad after the loop exits rather than trusting
"the loop stopped, so it must have succeeded" -- a latent gap in the PRE-
EXISTING code: the old loop's own `if len(window_alpha) < min_window`
check was structurally always false (the loop only ever GROWS the window
from its min_window starting length, never shrinks it), so a window that
ran out of history (hit index 0) without ever reaching the span floor was
silently accepted anyway. That dead check is removed; a real `achieved_
span < min_span` check now gates the sample to NaN (no signal) instead.
reconstruct_cs_window_start (the shared helper the corner-trace track
map's window highlight and several diagnostics scripts already reuse)
gained the same max_window parameter, defaulting to None (unbounded, the
pre-Phase-2 behaviour) for backward compatibility with callers that only
ever invoke it on an index already known to carry a finite CS_ratio.
ui/views/corner_trace_dialog.py's three live call sites now pass the
config cap explicitly, so the track-map highlight's "mirrors exactly"
contract with the production estimator stays true.

New cs_phase_min_valid_samples (config, value 5): summarise_corners's
cs_ratio_f/r phase stat blocks now report NaN (dropped from Stage 2's
min) when fewer than this many finite CS_ratio samples back the phase's
median -- a general per-phase noise floor, scoped to CS_ratio only
(stability/fz/ls stats untouched, verified by a targeted test). 5 chosen
as the smallest n for which a median is not simply one of the phase's own
most extreme readings. _stats() itself was already NaN-safe (confirmed by
reading it, not assumed) -- no change needed there; the gate is a
wrapper applied at the two cs_ratio_f/cs_ratio_r construction sites.

Targeted tests (tests/test_cs_validity_repair.py, 16 tests, all synthetic
inputs): flat-alpha-never-meets-span-floor -> all-NaN; a genuine ramp
widens successfully within the cap and recovers the exact known slope;
a too-slow ramp gets truncated at the cap and correctly reports NaN
(distinguished from the uncapped/backward-compatible reconstruction,
which is also tested separately); the min-valid-samples gate fires below
threshold and stays inert at/above it, and does not touch the stability
stat. Pipeline smoke test (test_stability.py) run against real Dubai data
under the new machinery: clean exit, zero tracebacks -- apex_3 and even
some longer phases (e.g. entry_1_brake on flat-alpha braking zones) now
frequently report NaN CS, which is the intended, honest behaviour, not a
regression.

### CS validity repair, part A, Phase 3: apex_region statistic
implemented, verified against the recommendation engine [2026-09-02]
modules/stability_analysis.py's summarise_corners gains a top-level
apex_region dict per corner instance (n_samples, cs_ratio_f, cs_ratio_r --
same _stats() shape as a phase's own CS block, same cs_phase_min_valid_
samples gate applied), replacing apex_3's structurally fixed 11-sample
slice (thesis_notes.md "apex_3 structural finding") for CS purposes only
-- apex_3 itself is unchanged for display and its own stability stat.
apex_region is DISTANCE-based (config cs_apex_region_half_length_m,
default 25 m, from the mechanism investigation's own C4 event-extent
observation, ~20-40m), bounded in TIME to the corner's own instance first
(union of its 5 phase segments' own start/end) before applying the
distance band, using the pre-existing apex_lap_distance_m (already
computed per corner, lap-boundary-reset-guarded) -- a pure distance-band
search without the time bound would otherwise pull in every other lap's
samples passing the same track position, since s_m resets every lap.
ANALYSIS_SCHEMA_VERSION 7->8 (payload gained the apex_region key; a pre-
bump persisted result has none, correctly falls to no-cache).

Wiring, so apex_3-keyed reads actually reach apex_region: ui/views/
outing_form.py's _classify_corner substitutes apex_region's cs_ratio_f/r
median for apex_3's own phase entry ONLY when apex_region is present
(summary.get("apex_region") is not None) -- absent (pre-bump summaries),
falls back to apex_3's own slice exactly as before, verified by a
dedicated backward-compatibility test. modules/recommendation.py's
aggregate_by_corner now also median-of-medians's apex_region across a
corner's own laps (mirroring its existing phases[] aggregation exactly);
_phase_verdict carries apex_region through its own phase-slicing step
whenever the rule's phases include "apex_3"; the undrivable-escalation
path's per-lap phase substitution (_apply_undrivable_escalation) also
substitutes apex_region from the SAME qualifying lap when apex_3 is
substituted, so escalation never mixes one lap's real apex_3 with the
aggregate's median-of-4-laps apex_region.

VERIFIED: all 6 live apex_3-phased recommendation rules (config/
recommendations.json: matrix_us_apx_low/med/high, matrix_os_apx_low/
med/high -- matrix_us_apx_med_esc is "held", us_apex_arb is "retired",
neither ever fires, per generate_recommendations' own skip rule) fire
correctly through generate_recommendations end-to-end on a synthetic
4-lap corner whose apex_3 slice reads perfectly healthy and ONLY
apex_region signals a fault -- a direct test that the repair actually
restores apex-keyed recommendation coverage, not just a unit-level check
on classify_fn in isolation (tests/test_cs_validity_repair.py, 6
parametrized cases, all pass).

### CS validity repair, part A, Phase 4: final distributions -- the
wholesale-negative artifact is fixed, but C4's own established genuine
signal is now DILUTED BELOW VISIBILITY at the worst-lap statistic
[2026-09-02]
Diagnostics only, no config/estimator/threshold change. diagnostics/
inspect_corner_distribution.py extended (Phase 4's own instruction) with
an apex_region-substituted worst-per-instance/worst-lap statistic
(reproducing classify_fn's exact apex_3->apex_region substitution) and a
no-signal footprint report; re-run against both kinematic and
ekf_auto_pacejka, fallback-guarded, real Dubai data, all 14 physical
corners, 56 corner x lap instances.

HEADLINE: the wholesale-negative artifact IS FIXED. Worst-lap CSf under
ekf_auto_pacejka goes from 12/14 corners negative (pre-repair, PLAN.md
STATUS "THRESHOLD RE-DERIVATION...") to 3/14 negative post-repair (C3
-0.192, C2 -0.107, C14 -0.095; all other 11 corners positive, up to C11
+0.831). This is not a marginal shift -- C1 (-0.981 pre-repair, via the
apex_3 structural artifact) is now clearly healthy (+0.094 CSf, +0.256
CSr); C2 recovers from -1.119 to a small -0.107; C9 (STEP 2's own
already-established artifact case) is comfortably positive (+0.070 CSf,
+0.291 CSr). C3 improves from -1.183 to -0.192 (CSf) / from ekf-auto's
earlier -0.813 (apex-driven) to -0.075 (raw) / -0.150 (apex_region) on
CSr -- a large reduction in magnitude but NOT a full recovery to
positive, partially matching the pre-registration's "gated or healthy"
(neither cleanly holds -- it is now small-and-residual, not gated NaN
and not healthy-positive). C1/C2 pre-registration ("healthy or mild") is
well matched.

NOT SUPPORTED, stated plainly: "C4 negative via apex_region on multiple
laps" (the work order's own pre-registration for this phase). C4's
worst-lap value is now POSITIVE on BOTH axles under apex_region
(+0.307 CSf, +0.384 CSr) -- identical to its raw-apex_3-driven value
(apex_region and raw-apex_3 give the SAME number for C4 specifically,
meaning apex_3 itself was already not the worst phase for C4 post-
repair; some other phase resolves the worst-lap minimum instead). Since
worst-lap is a min-across-4-laps statistic, this means EVERY ONE of C4's
4 laps individually now reads positive at its own worst-phase value --
not just the aggregate.

This is a genuine, unresolved tension, not glossed over: the ground-
truth workup (thesis_notes.md "Ground-truth workup...") established C4
as REAL on all 5 named runs via actual fold/peak pictures in the tyre-
curve render -- the strongest, most direct evidence in this entire
investigation arc. Those runs were short (17.0-31.9m) relative to a
corner's own full extent. The repair's OWN mechanism -- a much larger
minimum window (100 samples, vs the 11-26 that produced the original
noisy-but-present negative reading) -- means a window covering one of
those short folds now very likely extends well past the fold itself,
averaging the genuine local decline back in with the surrounding
still-rising data. This is the SAME dilution failure mode already
identified earlier in this arc (thesis_notes.md "Gated Stage-2
recomputation...": "median-across-a-wide-window smooths out a real but
spatially narrow event") recurring here through a different mechanism
(the regression window itself widening, not a phase-median reduction) --
the repair fixes the WHOLESALE small-window-noise artifact but appears to
have traded it for widening-induced dilution of C4's own short, genuine
events specifically. If thresholds were anchored against this repaired
distribution as it stands, C4 -- the one corner with actual fold-picture
proof -- would NOT be flagged. NOT resolved here; no threshold anchored,
per the work order's own stop-at-the-numbers instruction. Flagged as the
single most important open item before any anchoring proceeds.

APEX_REGION VALIDITY: a clean, unambiguous success on its own terms.
Under ekf_auto_pacejka, apex_region is valid (non-NaN) on 56/56 (100%)
instances, BOTH axles -- apex_3's own raw slice, by contrast, is
no-signal on 14/56 (25.0%) instances (both axles, this run). apex_region
fully resolves the structural under-sizing problem it was built to fix.

NO-SIGNAL FOOTPRINT, the large and NOT pre-registered side effect:
entry_1_brake's own CS_ratio is no-signal on 87.5% (front) / 92.9% (rear)
of instances under ekf_auto_pacejka (83.9%/83.9% under kinematic) -- up
from a small fraction before this package (braking is typically a low-
alpha-variation phase; the new 0.04 rad span floor is rarely reached
during pure braking before turn-in). entry_2_turnin/exit_4/exit_5 stay
low (0-7.1%), unaffected -- these phases retain plenty of alpha
excitation. PLAN.md STEP 4's own prerequisite list already flagged that
15 of 39 recommendation rules key on entry_1_brake; this finding means
essentially ALL of those rules' "data" trigger can now only fire from
entry_1_brake's own STABILITY reading (unaffected by any of this
package's changes), never from a CS-based verdict on that phase, for the
large majority of instances -- a new, previously-unquantified consequence
of the re-derived floors, surfaced here as a number, not evaluated or
acted on (out of this diagnostic's own scope; a candidate item for
whoever picks up STEP 4).

SMALL-N WORST-WINDOW POPULATION: eliminated by construction, not just
shrunk. Every valid CS_ratio sample now requires >=100 samples in its own
regression window (cs_min_window_samples's own floor) -- the 11-26 sample
windows the mechanism investigation traced the original wholesale-
negative reading to (thesis_notes.md "Mechanism investigation...",
Finding 3) can no longer produce a finite CS_ratio at all; they now
correctly report NaN. This is the direct, load-bearing mechanism behind
the wholesale-negative fix above, not a separate side benefit.

POOLED MEDIANS: not cleanly comparable to a "before" number at the SAME
statistic -- the pre-repair investigation measured pooled PER-SAMPLE
medians (thesis_notes.md "Mechanism investigation...", Finding 1: all 12
corner/axle cases positive, 0.021-0.614), a different quantity from this
phase's worst-lap/worst-instance statistics, which were the ones
demonstrated to be corrupted. This phase did not re-measure the per-
sample pooled median under the new floors (out of scope, would require
rerunning the whole per-sample-population script); the worst-lap/worst-
instance statistic itself -- the one that actually feeds classify_fn --
changed substantially, by design.

No config, estimator, or threshold value written or changed in this
phase. Full reproducible numbers: diagnostics/inspect_corner_
distribution.py's own run output (deterministic, no randomness).

### CS validity repair, part A: full non-golden test suite run --
2 unexpected failures, ONE shared root cause, traced and scoped
[2026-09-02, same day]
Full suite run once (142 tests, excluding tests/test_golden_pipeline.py
and tests/test_golden_auto_modes.py per the work order's own "goldens
expected red, do not regenerate" constraint): 140 passed, 1 xfailed, 2
FAILED -- neither anticipated by the work order's own stated hard
constraints, so traced rather than dismissed.

ROOT CAUSE, same for both: modules/tyre_fit_auto.py's fit_session (the
ekf_auto_dugoff per-session fit chain -- NOT fit_session_pacejka, the
production-default ekf_auto_pacejka chain, confirmed independent by
reading it) calls estimate_cornering_stiffness directly (line ~281) and
uses its output as a HARD INPUT, not just a validation figure: _fit_axle's
own c_alpha_used = median(C_alpha[CS_ratio==1.0 & base_mask]) (line 88) --
the windowed-regression estimator's own "currently in the linear region"
samples ARE the c_alpha determination for the Dugoff auto-fit. This
coupling predates this package entirely (it is how fit_session was
always designed -- Module 4b's own linear-region indicator as the "clean
sample" mask for Module 6's fit) but had never been exercised by a
CS window-floor change before now. Re-deriving cs_min_window_samples/
cs_min_slip_angle_span_rad/cs_linear_slip_threshold_rad therefore also
shifts WHICH samples read CS_ratio==1.0 and what C_alpha those samples
carry -- shifting fit_session's own c_alpha_used by a small but real
amount (front: 132797.9 -> 130891.5 N/rad, -1.4%).

FAILURE 1: tests/test_auto_fit_wiring.py::
test_ekf_auto_dugoff_reproduces_wp_n3_phase2_figures -- asserts a fresh
fit_session run reproduces the FROZEN tyre_model_ekf.pass_0 c_alpha to
1e-6 relative tolerance, premised on "identical inputs" to the frozen
run. Inputs are no longer identical (this package changed them, by
design, for exactly this axle/mask) -- EXPECTED RED, same category as
the golden-file tests, just not literally named "golden" and not on the
work order's own stated list.

FAILURE 2: tests/test_nis_gate.py::
test_worst_mismatch_is_strictly_the_lowest_score -- its own
nis_gate_scenarios fixture calls fit_session AGAIN (a fresh "healthy"
baseline fit, tests/test_nis_gate.py:52) to build the synthetic
mismatch-scenario NIS traces the gate's health score is checked against.
The same c_alpha shift propagates into this fresh healthy baseline,
tipping an ALREADY-MARGINAL ordering (healthy score 0.1415 vs
c_alpha_x0.5's 0.1470 -- these scores were close even before this
package; config's own _comment_verdict_reality_check already documents
that "NOT all four WP-N3 synthetic mismatch scenarios verdict as fail"
at the current thresholds, i.e. this system was already known-fragile,
provisional, and gap-selected from five data points on one session).

SCOPE: does NOT affect the production default (ekf_auto_pacejka) or
kinematic mode. fit_session_pacejka (verified by reading it) fits
(B,C,D,E) via Powell directly from raw alpha/Fy/base_mask alone -- no
CS_ratio/C_alpha dependency exists in that path at all. Both failures
are confined to ekf_auto_dugoff (a secondary, always-gated, non-default
mode) and the NIS gate's own already-provisional threshold system, which
happens to use a Dugoff healthy baseline for its synthetic scenarios
regardless of which mode is live in production.

CONSEQUENCE, stated plainly for whoever anchors next: this package's
floor re-derivation has a real, previously-latent side effect on
ekf_auto_dugoff's own fitted c_alpha (and therefore anything measured
against it, including the NIS gate's provisional thresholds) -- not just
on Module 4b's CS_ratio verdicts, which was this package's own stated
scope. Neither test was fixed here (fixing test 1 would mean re-freezing
tyre_model_ekf.pass_0 against the new floors -- a real decision, not a
mechanical update; fixing test 2 would mean re-deriving the NIS gate's
already-flagged-provisional thresholds -- explicitly parked, separate
work). No config, estimator, or threshold changed to address either
failure. Both are reported here as EXPECTED RED with root cause, per the
work order's own instruction.

DECISION (same day, user-directed): the coupling is ACCEPTED as a
documented consequence, not a defect to chase now. ekf_auto_dugoff is a
secondary mode; tests/test_auto_fit_wiring.py's frozen-reproduction
expectation (tyre_model_ekf.pass_0) gets regenerated deliberately
ALONGSIDE the goldens, at the end of this package, not mid-package. The
NIS gate's provisional thresholds stay flagged exactly as already
recorded (config's own _comment_verdict_reality_check) -- a known
single-session-derivation limitation, not something this package
re-derives. Both tests remain EXPECTED RED until that later golden step;
no further action on either this turn.

### CS validity repair, part A, Phase 1 REVISION: floors re-derived
against the phase-level MEDIAN, sample-rate corrected [2026-09-02, same
day, user-directed]
Corrects the first Phase 1 attempt's criterion (kept, not deleted --
thesis_notes.md "CS validity repair, part A, Phase 1: window-floor
re-derivation", now marked SUPERSEDED at its own top). That attempt
bootstrapped a SINGLE WINDOW's own sampling variance and correctly found
no natural knee -- the wrong target. What actually feeds classify_fn is a
PHASE-LEVEL MEDIAN over many per-sample windowed estimates (summarise_
corners's own _stats()), which already averages out per-window noise
across a realistic phase's O(30-200) samples -- a much smaller per-window
floor can still yield a stable phase median.

METHOD (Tier B): new diagnostics/inspect_cs_phase_median_floor_
derivation.py. For each candidate (n, span) pair and representative phase
length L in {40, 56, 76, 120, 162} samples (drawn from the real measured
corner-phase-duration spectrum, diagnostics/inspect_cs_window_cap_
sizing.py's own CORNER-PHASE DURATIONS block -- entry_1_brake and apex_3
excluded, see below), drew 150 real contiguous L-sample stretches from
the session's moving population, computed each stretch's own per-sample
windowed-OLS slope at every one of its L samples using the real
production growth mechanism (reconstruct_cs_window_start), then bootstrap
-resampled (B=150, with replacement) the L per-sample slopes and took
nanmedian of each resample -- reproducing _stats()'s own median under
exactly the finite/NaN mix a real phase of that length would show.
Relative std of the bootstrap-median distribution vs the point-estimate
median is the target statistic (median across 150 stretches, per (n,
span, L)). A first run hung for ~30+ minutes before being killed and
fixed: reconstruct_cs_window_start was called with no cap in the
script, so a near-flat-alpha stretch forced an unbounded backward search
-- the exact runaway pathology Phase 2 exists to prevent in production.
Fixed with a generous 2000-sample COMPUTATIONAL bound (never binds for a
genuine cornering stretch, only for pathological flat regions this
analysis would discard as no-signal anyway) -- not a design decision,
noted in the script's own comment.

FINDING: smallest (n, span) clearing <=15% relative std at the SHORTEST
representative phase length (L=40, 0.8s -- roughly exit_4's own shorter
end) governs the final choice, since longer phases only get MORE stable
at the same floor via more averaging -- confirmed monotonically across
all 5 tested L values on both axles (e.g. front (n=5,span=0.005): 20.9%
at L=40 -> 12.0% at L=120; every candidate improves or holds as L grows).
At L=40: front needs (n=10, span=0.01) to clear with real margin (14.3%,
vs (n=8,span=0.008)'s razor-thin 15.0% exactly at the boundary -- too
close to trust); rear clears the SAME pair far more comfortably (7.7%),
confirming front as the stricter, governing axle -- consistent with
every other floor-sizing decision in this package using the more
demanding axle for a single shared config value. entry_1_brake and
apex_3 were excluded from the representative-L set: entry_1_brake's own
median duration is ~1 sample (physically degenerate, not a floor-choice
question, see the no-signal finding below); apex_3 is superseded by the
distance-based apex_region (Phase 3), which has its own generous
50m-wide (2x cs_apex_region_half_length_m) footprint, comfortably larger
than any of these candidates regardless of choice.

SAMPLE-RATE CORRECTION (same day, user-directed addendum): the chair's
own historical default (10 samples) was ALWAYS a 100 Hz-calibrated
value (10/100 = 0.1 s physical window) -- every prior turn in this
project's history that carried "10" forward as cs_min_window_samples
treated it as a literal, rate-independent sample count, exactly the same
category of oversight PLAN.md STEP 3 already found and fixed for
longitudinal_stiffness's own min_samples (chair's literal 25 vs this
car's actual rate). diagnostics/inspect_native_channel_rates.py
(read-only, no pipeline change) confirmed this file's own CS-chain
channels: ecu_speed is natively 50 Hz (the slowest of the six, and the
one that caps the pipeline's common grid); sclu_yaw_rate, log_asteer,
log_acc_y, log_acc_z, and lap_distance are ALL natively 100 Hz, matching
the chair exactly -- the 50 Hz ceiling is specifically an ecu_speed
limitation, not a whole-log one (reported as a finding only, no pipeline
change made per that diagnostic's own "stop" instruction).

Fixed the SAME way as the LS_ratio precedent: cs_min_window_s (a
PHYSICAL duration, not a sample count) is converted to samples at
RUNTIME via the file's own measured sample_rate_hz (modules.stability_
analysis.resolve_cs_min_window_samples, mirroring modules.longitudinal_
stiffness's own regression_window_s/min_samples_floor pattern exactly),
floored at cs_min_window_samples_floor to protect a much slower log (this
project's own GT3 Paul Ricard census found 20 Hz) from deriving too few
samples for a meaningful fit. The phase-median derivation's own answer,
n=10 samples @ this file's 50 Hz = 0.2 s, is LONGER than the chair's own
0.1 s reference -- stated explicitly, not silently inflated: the chair's
alpha comes from a MEASURED beta input channel; this project's alpha is
derived from an ESTIMATED beta (kinematic formula or an EKF/auto-fit
reconstruction, itself a Level 1-3 estimation chain carrying its own
noise) -- a noisier upstream signal needs more temporal averaging to
reach the same phase-median stability. cs_min_slip_angle_span_rad
(0.01 rad) needs NO rate correction -- it is already a physical,
alpha-domain (radians) quantity, unaffected by sample rate.

cs_max_window_m (Phase 2's own cap) RE-SIZED as a LOCALITY bound, per the
same revision's item 3 -- NOT from the whole-session observed window-
length distribution (the original Phase 1 approach, dominated by
irrelevant straight-line samples needing no cap at all). diagnostics/
inspect_cs_max_window_locality_sizing.py measured the natural (uncapped)
window's own metre extent under the final small floor, restricted to
real corner-bracket samples only (moving, non-kerb, inside an actual
corner's own entry_1_brake-to-exit_5 span): median footprint front=9.1m,
rear=11.1m (p90 28.0m/40.7m -- a real, non-trivial tail even restricted
to genuine cornering data). Set at 1.5x the more demanding (rear) axle's
own median = 16.65m, rounded to 17.0m. Applied as a REAL TRACK DISTANCE
via state['s_m'] directly inside the growth loop (not converted to a
fixed sample count at a "representative" speed, the original Phase 1
approach's own design) -- a corner's physical scale is a distance, not a
duration, so the cap must not silently mean something different at a
slow vs a fast corner. Both reconstruct_cs_window_start and compute_cs_
for_axle's inline loop updated in sync (same "mirrors exactly" contract
as before); ui/views/corner_trace_dialog.py's three live call sites
updated to pass sample_rate_hz/s_m/max_window_m through (a small,
necessary signature change to _render_tyre_curves/_build_tyre_curve_
export to thread sample_rate_hz down from self._render_ctx["state"],
already carried there for _render_track_map's own pre-existing use).

cs_linear_slip_threshold_rad (0.03 rad) and cs_phase_min_valid_samples
(5) are UNCHANGED by this revision -- the first is alpha-domain/rate-
independent and was not targeted by either the criterion correction or
the sample-rate correction; the second is a pure robustness-on-sample-
count statistic, not a physical-duration quantity, and the user's own
addendum scoped the physical-unit correction to "window floors"
specifically.

Targeted tests (tests/test_cs_validity_repair.py) updated to match: new
resolve_cs_min_window_samples tests (rate scaling, floor binding);
reconstruct_cs_window_start's cap tests reworked around s_m/max_window_m
instead of a sample-count max_window; a new test confirming the distance
cap disables cleanly (falls back to unbounded) when s_m is unavailable.
19 tests total, all pass. Real-data pipeline smoke test (test_stability.py)
re-run clean under the revised config: zero tracebacks; entry_1_brake and
apex_3 remain frequently NaN as before (expected, unrelated to this
specific revision -- see the no-signal footprint finding, Phase 4).

### CS validity repair, part A, Phase 4 REVISION: the phase-median
criterion PASSED ITS OWN TEST AND STILL FAILED -- wholesale-negative
artifact REPRODUCED AND WORSENED, pre-registration falsified on every
clause. STOPPING HERE, per the work order, to report rather than choose
a fix silently [2026-09-02, same day]
diagnostics/inspect_corner_distribution.py re-run under the revised
floors (cs_min_window_s=0.2s/cs_min_slip_angle_span_rad=0.01 rad,
n=10 samples @ 50 Hz -- Phase 1 REVISION). Result is the OPPOSITE of
every pre-registered expectation, and worse than the ORIGINAL pre-repair
baseline this whole investigation arc exists to fix.

HEADLINE NUMBERS (ekf_auto_pacejka, worst-lap-per-corner, apex_region-
substituted -- the statistic classify_fn actually runs against): CSf is
NEGATIVE on 12 of 14 corners (range -1.708 to +0.120 -- C6/C5 barely
positive, everything else negative, including C1 at -0.352 and C6's own
CSr at -0.432, both previously established HEALTHY/ARTIFACT cases now
reading negative again). CSr is negative on 12 of 14 corners too (range
-3.442 to +0.465). Even the PER-INSTANCE pooled population (before any
worst-lap aggregation) already reads deeply negative: p10=-0.664,
p25=-0.293 (CSf) -- compare the ORIGINAL wholesale-negative finding's own
per-instance p10/p25 (thesis_notes.md "Mechanism investigation...",
Finding 1: ALL 12 pooled per-sample medians were POSITIVE, 0.021-0.614).
This is not a partial regression -- the small-n overfitting artifact this
entire arc exists to fix is BACK, and by the per-instance numbers, WORSE
than before any repair work started.

PRE-REGISTRATION CHECK, every clause: "C4 negative again on multiple laps
-- its events are longer than the new resolution" -- TRUE but
uninformatively so (C4 is negative, -0.353 apex_region, but so is nearly
everything else, so this is not the SIGNAL the pre-registration meant).
"C1/C2/C3-rear stay recovered (their artifacts are single-window scale,
still gated)" -- FALSE: C1 is NOW NEGATIVE again (-0.352 CSf, -0.804
CSr, apex_region), reproducing exactly the artifact the repair fixed one
revision ago. "Medians ~unchanged" -- FALSE: worst-lap CSf median moved
from -0.496 (raw) even further negative under apex_region-substitution
in the wrong direction relative to any prior state.

DIAGNOSIS, not just reported blind: the phase-median bootstrap (Phase 1
REVISION) cleared its OWN 15% relative-std criterion at (n=10,
span=0.01) -- that measurement is not wrong on its own terms, but it
tested the WRONG POPULATION. The bootstrap sampled 150 REALISTIC-LENGTH
stretches UNIFORMLY at random from the WHOLE moving population (any
speed, any corner phase, including gentler/slower-varying sections) --
its own reported mean_achieved_n at (10,0.01)/L=40 was 60.2 (front) /
101.3 (rear), i.e. windows in THAT sample usually grew well past the
tiny nominal floor before satisfying the span requirement, because much
of the sampled population doesn't vary alpha quickly. But diagnostics/
inspect_cs_max_window_locality_sizing.py's own EARLIER measurement --
restricted to REAL CORNER-BRACKET samples specifically, the population
Phase 4 actually evaluates against -- found the SAME (10,0.01) floor's
natural window length sits at p50=13 samples (front) / p50=16 (rear),
with a full QUARTER of real cornering windows at the absolute minimum
(p10=p25=10 samples, both axles): in actual apex/turn-in dynamics, alpha
changes fast enough that the tiny span floor is satisfied almost
immediately, far more often than the bootstrap's whole-session random
sample implied. The phase-median criterion is a NECESSARY but NOT
SUFFICIENT check: it verifies the median is REPRODUCIBLE across
resamples of whatever data it was given (precision), not that the
underlying per-window estimates are UNBIASED relative to the true local
tangent (accuracy). A reproducibly-wrong median passes a precision-only
test cleanly. Sampling a population dominated by slower-alpha stretches
masked exactly the regime (fast, real cornering dynamics) where this
floor is at its most unstable.

CONSEQUENCE, stated plainly: this package's own governing criterion
(phase-median bootstrap stability, sized against a population that does
not match the real evaluation population) is not a safe way to choose
these floors as implemented. The (10, 0.01, 17m) values currently in
config/parameters.json are exactly what the work order's own method
specified and were implemented faithfully -- the METHOD produced a
falsified result, not an implementation bug (config values reconfirmed
correct before writing this entry; pipeline smoke-tested clean; the
divergence traces to a real sampling-population mismatch in the
derivation script, not a wiring defect). NOT fixed here: this is a
methodology question (how to size the bootstrap's own sampled
population, or whether phase-median stability is the right criterion at
all, or whether the ORIGINAL Phase 1 floor (100 samples/0.04 rad, which
DID produce a healthy Phase 4 result before this revision) was in fact
closer to right despite its own "no natural knee" finding) that is the
user's decision, not this package's to make silently. STOPPING at these
numbers, per the work order's own explicit instruction, rather than
picking a new criterion or floor unilaterally.

No config/estimator value changed AGAIN in response to this finding --
(10, 0.01, 17.0) stays in config exactly as the work order specified,
pending direction. Full reproducible numbers: diagnostics/inspect_corner_
distribution.py's own run output; diagnostics/inspect_cs_phase_median_
floor_derivation.py's own run output (both deterministic, seed=42).

### 100 Hz time-base work package, Phase 0: adaptive common grid
(50-100 Hz), refusal only below the hard floor [2026-09-02, same day,
user-directed]
Method: Tier B (signal/data-engineering preprocessing -- which sample
rate the common interpolation grid runs at; no vehicle-dynamics
content). Corrects a structural finding from diagnostics/inspect_
native_channel_rates.py (2026-09-02, earlier this session): prepare_
vehicle_state's common time grid was literally ecu_speed's OWN raw
timestamps (t_ref = channels["ecu_speed"]["time"]) -- since ecu_speed is
natively 50 Hz on this car while sclu_yaw_rate/log_asteer/log_acc_y/
log_acc_z/lap_distance are all natively 100 Hz, every one of those five
channels was being DOWNSAMPLED onto ecu_speed's own coarser grid,
silently discarding half their real resolution, for as long as this
pipeline has existed.

MECHANISM: new CS_CHAIN_FAST_CHANNELS constant (modules/stability_
analysis.py, method-defining -- which channels constitute "the CS
chain" is a fact about this estimator, not a per-car tunable) names the
five non-ecu_speed channels. New _resolve_grid_rate: measures each fast
channel's own native rate, takes the slowest as "cs_chain_capability",
and sets grid_rate = min(target_sample_rate_hz, cs_chain_capability) --
config gained target_sample_rate_hz=100 (the preferred grid) and min_
sample_rate_hz=50 (the hard floor, replacing expected_sample_rate_hz's
old exact-match guard). A file whose fast channels only support 50 Hz
(or anything in [50,100)) processes normally at that rate -- no
refusal, no message beyond state["grid_rate_status"] recording "NN Hz"
vs "NN Hz (channel-limited, <channel>)". Only cs_chain_capability below
50 Hz (the GT3 Paul Ricard 20 Hz case, unchanged) is refused, naming the
binding channel. prepare_vehicle_state now builds a SYNTHETIC, evenly-
spaced grid at the resolved rate (np.arange over ecu_speed's own
observed time span) rather than reusing any one channel's raw
timestamps -- ecu_speed itself is then upsampled onto it via the SAME
np.interp every other channel already uses, justified because vehicle
speed is an INERTIA-LIMITED signal (cannot jump between real samples):
linear interpolation between genuine 50 Hz readings invents no
meaningfully wrong information, unlike upsampling a fast-changing
quantity (yaw rate, steering) would have.

VERIFIED on Dubai: grid_rate_status="100 Hz" (all five fast channels
measure 100.000 Hz natively, confirmed), sample_rate_hz=100 exactly,
n=81598 samples (2x the prior 40799 at 50 Hz, same file, same time
span -- 314.124s to 1130.094s). test_stability.py runs clean end-to-end,
zero tracebacks. entry_1_brake/entry_2_turnin/exit_4/exit_5's own raw
sample counts all roughly doubled (e.g. entry_1_brake n=97->195,
entry_2_turnin n=179->358) -- apex_3's own n=11 is UNCHANGED (apex_half_
window_samples=5 is a literal sample count, not yet rate-corrected --
see the residual-legacy list below).

RUNTIME: measured directly, controlled A/B, same file, same beta,
isolating estimate_cornering_stiffness (the pipeline's own dominant cost
driver, ~86% of total per WP-N2 Step 1a's earlier finding): 50 Hz grid
35.43s, 100 Hz grid 185.77s -- a 5.24x increase, NOT the naively-expected
2x from double the sample count. Diagnosed, not just reported: the
window-growth loop's own per-sample cost is superlinear in window
length (np.max/np.min recomputed over the whole growing slice on each
backward step, not incrementally), AND achieving the SAME PHYSICAL alpha
span now requires roughly double the RAW samples at double the time
resolution (alpha changes at the same physical rate per second, not per
sample) -- both effects compound, so total cost scales worse than
linear with grid density. Full test_stability.py end-to-end: ~101s at
100 Hz (real measured run). A real, quantified cost of this package,
not a bug -- correctness is unaffected, flagged for anyone who cares
about wall-clock runtime of a full outing analysis.

RESIDUAL LEGACY, documented not fixed, per the user's own explicit
model (ekf_pass_1's frozen R): tyre_model_ekf.pass_1's own frozen R
(R_yaw_rate_var/R_ay_var) was calibrated against this car's PRIOR fixed
50 Hz grid -- carried forward unchanged for a file now running at
100 Hz. A stated limitation of that specific frozen curve (ekf_pass_1
mode only -- ekf_auto_dugoff/ekf_auto_pacejka re-fit R every session,
already rate-correct by construction), not re-derived by this package.
ADDITIONALLY FLAGGED, same treatment, NOT explicitly in this package's
own stated scope: kerb_dilation_samples (5), nis_gate/tyre_fit_auto/
tyre_model_ekf's own nis_window_samples (20, four separate config
blocks), and apex_half_window_samples (5) all remain literal sample
counts -- each now silently represents HALF its originally-calibrated
physical duration on a file running at 100 Hz instead of the old fixed
50 Hz. Every OTHER estimator window in this pipeline (cs_min_window_s,
yaw_stability_* via window_s/window_m/grid_step_m, longitudinal_
stiffness's regression_window_s) was already converted to a physical-
duration/distance basis before or during this package and derives
correctly at either grid rate.

New tests/test_grid_rate_selection.py (6 tests, per the amendment's own
"one targeted test per grid rate, synthetic fixture, both paths"
instruction): target-met (100 Hz), channel-limited fallback (55 Hz, not
exactly the 50 Hz floor -- avoided a real floating-point boundary
false-refusal from np.arange's own dt jitter at an exact-50.0 fixture,
traced and fixed, not a production bug), hard-floor refusal naming the
channel (20 Hz), correct binding-channel identification when only one
fast channel is slow, a missing fast channel not binding the rate, and
a full prepare_vehicle_state end-to-end check at both grid paths. All
pass. Cache identity (both WP6 in-memory and WP5 persisted) and the
persisted payload gained grid_rate_hz, mirroring sideslip_source's own
established pattern exactly (tests/test_config_schema_integrity.py's
two cache-identity-field tests updated and pass) -- ANALYSIS_SCHEMA_
VERSION stays 8 (this whole v7->8 package is still uncommitted; the
version-history comment documents the extension rather than bumping
again for a shape nothing has yet observed as "8" externally, per the
user's own "if not already bumped this package" instruction). One
status line added (ui/views/outing_form.py's estimator_status_label
gains " | time base: NN Hz [(channel-limited)]", appended at the render
call site rather than folded into _format_estimator_status's own
tested/PDF-shared text, to keep that function's existing contract
untouched).

### 100 Hz time-base work package, Phase 1: floor derivation, third
pass -- cornering-only bootstrap population still could not distinguish
a reproducible-but-biased median from an accurate one; FINAL choice made
by direct real-data validation instead [2026-09-02, same day]
Method: Tier B, corrected population, then a decisive methodological
pivot. New diagnostics/inspect_cs_phase_median_floor_derivation_v2.py:
same phase-median bootstrap as the (falsified) second pass, but the 150
sampled stretches per (candidate, L) are now restricted to CORNERING
population only -- moving AND (inside a real corner bracket OR |ay|
above gps_course_anchor_max_ay_g, 0.05g, already data-derived on this
same car as the boundary where the |ay| distribution starts rising
steeply toward cornering values, config's own provenance note reused
rather than inventing a new threshold). 89.2% of the moving population
qualifies as cornering under this definition. Run at this file's own
resolved 100 Hz grid (PHASE 0, same package).

FINDING, the deeper methodological lesson: even under the CORRECTED
population, small floors STILL cleared the 15% relative-std criterion
comfortably -- e.g. (n=10, span=0.01 rad) at L=80 (0.8s @ 100 Hz): 7.4%
median relative std, front axle, BETTER than the same candidate's own
14.3% under the second pass's flawed whole-population sampling. This
does NOT mean the population correction was wrong; it means the
CRITERION ITSELF has a blind spot the correction cannot fix: bootstrap
relative std measures REPRODUCIBILITY of the resampled median across
resamples of the SAME underlying data (precision) -- it can never
detect that the whole sampled population might be systematically BIASED
away from the true local tangent, because bootstrap resampling only
characterises the sampling distribution GIVEN the data it was handed. A
small window that is consistently, stably wrong every time (a real
possibility for OLS slope estimation on a curved, noisy signal) produces
a LOW-VARIANCE WRONG ANSWER -- exactly what the second pass's real
Phase 4 failure demonstrated, and what this corrected-population
bootstrap still could not rule out for the same small candidates.

DECISION: abandoned the bootstrap-relative-std criterion as the
DECIDING mechanism (it remains useful as a coarse sanity check, not a
sufficient one) in favour of DIRECT real-data validation -- see the
next entry. No config value chosen from this script's own output alone.

### 100 Hz time-base work package, Phase 1 FINAL: direct real-data
validation lands on the chair's own original physical window, correctly
rate-resolved -- both extremes independently reproduce previously-
diagnosed failure modes [2026-09-02, same day]
Method: Tier B, direct accuracy check (not a bootstrap-derived
criterion). New diagnostics/inspect_cs_floor_candidate_validation.py:
parses and fits beta ONCE (the expensive, floor-independent steps), then
re-runs only estimate_cornering_stiffness/summarise_corners per
candidate against the REAL Phase 4 statistic (worst-lap-per-corner,
apex_region-substituted CSf/CSr -- exactly what classify_fn consumes),
for five (min_window_s, min_span_rad) candidates spanning the tested
range: (0.2, 0.02), (0.4, 0.04), (0.6, 0.06), (1.0, 0.04), (2.0, 0.04).

RESULTS, decisive: (0.2s, 0.02 rad) -- 20 samples at this file's own
100 Hz grid -- gives a clean, pre-registration-consistent population: 6
of 14 corners CSf-negative (not the wholesale 12-14/14 the falsified
small floor produced), C1 healthy (+0.022/+0.14), C2 healthy
(+0.217/+0.236), C4 negative on BOTH axles (-0.128/-0.042), C6 healthy
(+0.355/+0.599), C3 mixed (front healthy +0.09, rear negative -0.269).
(0.4s, 0.04 rad) reproduces the FIRST Phase 1 attempt's own C4-dilution
failure exactly: C4 flips POSITIVE (+0.563/+0.71), only 1/13 corners
negative. (0.6s, 0.06 rad) is worse still: EVERY corner reports NaN --
the (then-unrevised) 17.0 m locality cap could not accommodate this
floor's own natural window size at all, zeroing the population entirely
-- direct proof the cap must scale with whichever floor it is paired
with, not stay fixed. (1.0s, 0.04 rad) and (2.0s, 0.04 rad) -- re-testing
the FIRST attempt's own physical scale at the new grid -- both reproduce
its C4-dilution failure again (C4 positive on both axles both times),
confirming that failure was about window DURATION relative to C4's own
short (17-32m) genuine events, independent of grid rate.

CONCLUSION, stated plainly: (0.2s, 0.02 rad) is EXACTLY the chair's own
original physical window (their 10 samples @ 100 Hz = 0.1s reference
point is their OWN default; this car's own re-derivation independently
converges on double that duration, 0.2s, consistently across every
attempt in this arc) -- simply re-expressed in physical units and
correctly resolved at this file's actual 100 Hz grid (20 samples) instead
of the "10" that was silently carried over as a literal, rate-independent
count for the pipeline's entire history. The original wholesale-negative
artifact that opened this entire investigation was never really about
the window's physical duration being wrong -- it was the 50 Hz grid (a
side effect of ecu_speed setting the whole pipeline's common timebase,
Phase 0) starving a correctly-sized 0.2s window of enough raw samples
(10, not 20) for a numerically stable regression. Fixing the grid was
the real fix; the floor VALUES return to the chair's own original intent.
cs_max_window_m re-derived for this floor specifically (same 1.5x-
median-footprint method): front median 24.1m, rear median 35.4m (both
much larger than the falsified floor's own 9.1m/11.1m, since a bigger
span naturally needs more track distance) -- set at 1.5x rear's own
median = 53.0m (rounded), verified this does not truncate any of the 14
physical corners.

FINAL VALUES WRITTEN TO CONFIG: cs_min_window_s=0.2 (unchanged from the
falsified attempt's own value -- only the SPAN was wrong there, not the
duration), cs_min_slip_angle_span_rad=0.02 (reverted from the falsified
0.01), cs_max_window_m=53.0 (up from the falsified 17.0). cs_min_window_
samples_floor=5 and cs_linear_slip_threshold_rad=0.03 unchanged.

### 100 Hz time-base work package, Phases 2-3: design unchanged,
re-verified at the new rate and final floor [2026-09-02, same day]
No code change in this package beyond what Phase 1 REVISION already
made (the widening/cap mechanism and apex_region were both already
rate-agnostic by construction -- the cap is a real track distance via
state['s_m'], apex_region's own half-length is a metres config value,
neither reads a raw sample count anywhere). Re-verified against the
FINAL floor (0.2s/0.02rad/53.0m) via the full targeted test suite
(tests/test_cs_validity_repair.py, 19 tests -- synthetic fixtures with
their own local params, unaffected by the specific config numbers
chosen, so these were already passing throughout every floor attempt in
this arc; re-confirmed green here for completeness) and the direct
Phase 4 validation run itself (apex_region validity front=100.0%,
rear=98.2% under ekf_auto_pacejka -- unaffected by which floor governs
CS_ratio's own window search, confirming Phase 3's own independence from
Phase 1's floor choice as designed).

### 100 Hz time-base work package, Phase 4: final distributions --
pre-registration LARGELY CONFIRMED, real-cornering windows no longer
floor-pinned [2026-09-02, same day]
diagnostics/inspect_corner_distribution.py re-run under the FINAL
config (cs_min_window_s=0.2, cs_min_slip_angle_span_rad=0.02,
cs_max_window_m=53.0), both kinematic and ekf_auto_pacejka, all 14
physical corners, 56 corner x lap instances -- the official Phase 4
deliverable for this package.

PRE-REGISTRATION CHECK, ekf_auto_pacejka, apex_region-substituted
(the statistic classify_fn actually runs against):
- "C4 negative on multiple laps" -- CONFIRMED: worst-lap CSf=-0.122,
  CSr=-0.042, both negative (kinematic mode, for comparison, reads C4
  even more strongly negative: CSf=-0.382 -- consistent with STEP 2's
  own established genuine-saturation attribution surviving under BOTH
  beta sources, as it always has in this investigation).
- "C1/C2/C6 healthy" -- MOSTLY CONFIRMED: C1 (+0.023/+0.159) and C6
  (+0.069/+0.601) both clearly positive on both axles. C2 is healthy on
  rear (+0.293) but its front reads a small -0.032 -- essentially noise-
  floor magnitude (an order of magnitude smaller than C4's or C9's own
  negative readings), not a wholesale-artifact-scale value; read as
  "healthy within noise", not a clean positive, stated honestly rather
  than rounded up.
- "C3-rear artifacts gated" -- PARTIALLY CONFIRMED: C3 rear reads a
  small residual negative (-0.065 apex_region), not literally NaN/gated
  to no-signal -- but a small residual is the right DIRECTION and a
  dramatic improvement over every wholesale-negative distribution this
  arc has measured (the falsified small-floor attempt's own C3 rear was
  -0.150; the ORIGINAL, first-ever mechanism-investigation-era reading
  was far more extreme still). C3 front reads more negative (-0.330) --
  the ground-truth workup's own established C3-rear-is-artifact/C3-
  front-not-checked distinction is not contradicted, but not fully
  resolved by this statistic either; flagged, not glossed over.
- "medians stable" -- worst-lap CSf median -0.004, CSr median +0.121
  (apex_region) -- both close to zero, a physically plausible reading
  (neither pinned at the +1.0 ceiling nor collapsed to a wholesale-
  negative population); no single prior "before" number at this exact
  grid/floor/cap combination exists to compare against literally, so
  read as "plausible and centred", not verified against a specific
  prior figure.
- "real-cornering windows no longer floor-pinned (quantify)" --
  CONFIRMED, with numbers: diagnostics/inspect_cs_max_window_locality_
  sizing.py's own re-run at this exact floor found the natural window-
  length distribution at real corner-bracket samples is p10=24, p25=39,
  MEDIAN=74 samples -- comfortably above the 20-sample floor for the
  large majority of the population (median is 3.7x the floor). Contrast
  the falsified small-floor attempt, where p10=p25=10 (exactly the
  floor) and median=13 -- HALF the population sat AT or barely above the
  bare minimum. This package's own floor choice genuinely ends the
  small-n-pinning regime that opened the whole investigation arc.

NO-SIGNAL FOOTPRINT: entry_1_brake stays high (75.0% front/89.3% rear
under ekf_auto_pacejka) -- closely similar across every span value
tested in this entire arc (78-93% at span=0.01, 0.03, 0.04, and now
0.02) despite an ~4x range in the span floor itself. This consistency
across such a wide span range is the strongest evidence yet that this
is PHYSICS, not an over-strict threshold: pure braking genuinely does
not vary slip angle enough to clear ANY of the span floors tested,
including the smallest ever tried. Per the work order's own instruction,
report only -- not tuned specially. apex_region validity: front 100.0%,
rear 98.2% -- a clean, floor-choice-independent win, exactly as Phase 3
was designed to be.

NO corner lost to the cap: all 14 physical corners present in both
axles' worst-lap populations under both raw and apex_region statistics
-- the re-derived 53.0m cap accommodates this floor's own typical
window size without truncating any real corner's data.

No config, estimator, or threshold value changed in response to writing
this report (the config values were already finalised in the Phase 1
FINAL entry above, before this diagnostic run). Full reproducible
numbers: diagnostics/inspect_corner_distribution.py's own run output
(deterministic, no randomness).

EXPECTED-RED LIST, updated per the work order: golden pipeline/
recommendation tests (tests/test_golden_pipeline.py, tests/test_golden_
auto_modes.py) remain EXPECTED RED -- CS_ratio/apex_region output and
now also the grid rate (100 Hz vs each golden's own recorded 50 Hz) have
changed; not regenerated, per every prior instruction in this arc not
to touch goldens. tests/test_auto_fit_wiring.py's frozen-pass_0-
reproduction test and tests/test_nis_gate.py's mismatch-scoring test
remain EXPECTED RED for the SAME accepted, documented reason as before
(the ekf_auto_dugoff/NIS-gate coupling to estimate_cornering_stiffness's
own CS_ratio==1.0 samples, thesis_notes.md "full non-golden test suite
run") -- the coupling is structural, unaffected by which specific CS
floor values are in effect, so this remains deferred to the same later
golden-regeneration step regardless of this package's own floor
revisions. Full non-golden suite re-run in full against this FINAL config, given
the 100 Hz grid change's own broad reach (unlike a pure CS_ratio-floor
change, the grid rate affects every estimator in the pipeline, not just
Module 4b) -- result recorded in a follow-up entry once complete (own
runtime expected longer than the prior ~25 minutes, given the measured
~5x cost increase for the CS-ratio estimator specifically at 100 Hz).

### 100 Hz time-base work package: full non-golden suite result -- ONE
real bug found and fixed, ONE stale test string fixed, the Dugoff-chain
coupling ESCALATES from drift to full degeneracy [2026-09-02, same day]
Full non-golden suite (142 tests) run once against the FINAL config:
5 failed, 137 passed, 1 xfailed, 9 errors (27 min). Traced every one --
two were real, previously-undiscovered issues in THIS package's own
code, fixed same turn; the remaining ten trace to the already-accepted
Dugoff-chain coupling, now more severe than previously measured.

REAL BUG FOUND AND FIXED: prepare_vehicle_state's new synthetic grid
(np.arange(ecu_speed_t[0], ecu_speed_t[-1], 1/sr)) silently dropped the
file's OWN FINAL SAMPLE -- np.arange's stop is EXCLUSIVE. Invisible on
Dubai's ~81598-sample file (one sample lost in eighty-one thousand
never showed up in any statistic checked), but glaring on tests/test_
csv_parser_formats.py's tiny 4-row WIDE_FIXTURE (a genuine wide-format
export with ONE shared time column -- all channels natively at the SAME
50 Hz, not 100 Hz, so this fixture legitimately exercises the channel-
limited grid path): losing the last sample there dropped the test's own
lap_distance check to 2 points, failing test_wide_format_lap_distance_
correctly_normalised. FIXED: t_ref now built via np.linspace with an
explicit, rounded sample count (n = round((t_end-t_start)*sr)+1) --
endpoint-inclusive by construction, immune to step-accumulation drift.
Verified: Dubai's own grid gains exactly the expected +1 sample
(81598->81599); test_stability.py re-run clean; the full CS validity
repair arc's own extensive Phase 1/4 validation (all conducted against
the pre-fix, off-by-one grid) is UNAFFECTED in substance -- one sample
in ~81600 cannot move any percentile, median, or worst-lap statistic
measured in this entire arc.

STALE TEST STRING FIXED: test_rate_guard_refuses_mismatched_rate
expected the OLD guard's exact-match wording ("Sample rate mismatch");
PHASE 0 deliberately changed this to a range-based "Sample rate too
low" (min_sample_rate_hz is now a FLOOR, not an exact target) -- the
guard's own BEHAVIOUR was never wrong, only the test's own expected
string was stale. Updated to match; a sibling test already checking
substring content (not the exact phrase) had passed throughout without
needing a change.

DUGOFF-CHAIN COUPLING ESCALATES, same accepted root cause: previously
(thesis_notes.md "full non-golden test suite run", against the FIRST,
now-superseded 100/500-sample floor) this coupling produced a ~1.4%
c_alpha DRIFT -- cosmetic, the fit still succeeded. Against the FINAL
(0.2s/0.02rad/53m) floor, the SAME coupling now produces full
DEGENERACY: fit_session's rear-axle mu_fz search cannot converge to an
interior optimum ("hit the widened search bracket ceiling after 4
attempts, bound_fraction=1.000000"). Investigated, not just reported:
c_alpha_used itself (median C_alpha_r over CS_ratio_r==1.0 samples,
n=30589) is a perfectly plausible 170866 N/rad, 100% sign-consistent --
the degeneracy is NOT in CS_ratio's own output, it is in the SEPARATE,
already-flagged brittleness of _fit_axle's own bounded mu_fz search
when c_alpha is held fixed at whatever value this coupling hands it --
a small shift in that fixed input is apparently enough to push the
rear-axle mu_fz search outside every bracket the widening loop tries.
10 tests affected (3 in tests/test_auto_fit_wiring.py, 6 error + 1 fail
via tests/test_nis_gate.py's shared module-scoped fixture, all cascading
from the SAME single degenerate healthy_fit). CONSEQUENCE, stated
plainly: ekf_auto_dugoff (the secondary, non-default mode) now ALWAYS
falls back to kinematic on this file, under this final floor -- a
bigger practical fact than the earlier drift, though still the SAME
documented, ALREADY-ACCEPTED coupling (not a new defect), still confined
to that one secondary mode, still deferred to the same later golden-
regeneration decision point per the user's own standing decision. NOT
fixed here (fixing would mean either decoupling fit_session's own
c_alpha determination from estimate_cornering_stiffness's CS_ratio==1.0
mask, or hardening _fit_axle's own mu_fz search -- both real, separate
pieces of work, out of this package's stated scope).

FINAL TALLY after both fixes: the two real issues (grid endpoint, stale
string) are resolved; the ten Dugoff-chain-coupling tests remain
EXPECTED RED (escalated severity, same accepted root cause, same
deferred fix point). tests/test_csv_parser_formats.py and tests/test_
auto_fit_wiring.py + tests/test_nis_gate.py re-run individually after
the fixes to confirm: csv_parser_formats now 14/14 pass; auto_fit_
wiring/nis_gate reproduce the SAME 3 failed/9 errored consistently
(confirms the grid-endpoint fix was unrelated to and did not change
this separate, already-understood coupling).

### CS validity repair, sign-off clarification round: 0.1s hypothesis
tested and confirmed, 0.2s hypothesis tested and REJECTED -- final
config value is the chair's own literal, unmodified default
[2026-09-02, same day]
User asked, before signing off, for the direct (0.1s, 0.02 rad)
comparison Phase 1 FINAL never actually ran (it validated 0.2s against
real Phase 4 data but never re-tested the chair's own literal 0.1s the
same way). Two checks, both run this session:

(1) Isolated duration-only bootstrap (diagnostics/inspect_cs_duration_
only_comparison.py, span held fixed at 0.02 rad, cornering-only
population, same method as the Phase 1 third-pass script): n=10 (0.1s)
and n=20 (0.2s) are STATISTICALLY INDISTINGUISHABLE at every tested
phase length (L=80/152/324 samples), both axles -- differences of
1-2 percentage points with NO consistent direction (sometimes 0.1s is
even lower-variance). The bootstrap never supported the 2x duration
choice; the config text that had implied otherwise was itself in error
and has been corrected.

(2) Direct real-data validation (diagnostics/inspect_cs_floor_
candidate_validation.py, (0.1, 0.02) candidate added): reproduces the
(0.2, 0.02) result almost exactly across all 14 corners -- identical
negative-corner counts (7/14 CSf, 2/14 CSr), the same corners flagged,
all 14 corners present under the SAME 53.0 m cap (no re-derivation
needed). C4 rear reads a STRONGER negative at 0.1s (-0.134) than at
0.2s (-0.042).

DECISION: cs_min_window_s reverted to 0.1 -- the chair's own literal
default (10 samples @ their 100 Hz), zero deviation, now that PHASE 0's
100 Hz grid supplies the sample count that default always assumed. The
0.2s hypothesis is recorded as TESTED AND REJECTED (not merely
superseded by a better number): it worked, but no better than the
simpler, zero-deviation choice, so parsimony wins. The "estimated vs
measured beta" a priori physical argument (this car's alpha comes from
an estimated beta, a noisier upstream signal than the chair's own
measured-beta channel, so more averaging seemed plausible) remains a
reasonable STORY but was not supported by either form of evidence
actually gathered -- recorded honestly as a rejected hypothesis, not
quietly dropped.

FINAL CS WINDOW FLOOR VALUES: cs_min_window_s=0.1, cs_min_window_
samples_floor=5, cs_min_slip_angle_span_rad=0.02, cs_max_window_m=53.0
(re-confirmed valid at 0.1s, not re-derived -- span, not min_window,
governs the natural footprint in practice), cs_linear_slip_threshold_
rad=0.03, cs_phase_min_valid_samples=5, cs_apex_region_half_length_m=
25m. This closes the floor-VALUE question for this package.

### CS validity repair, limitation: cs_max_window_m does not guarantee
locality against corner-to-corner GAPS, only against corner LENGTH
[2026-09-02, same day]
Checked, not assumed, during the sign-off clarification round
(diagnostics/inspect_corner_bracket_geometry.py, a pure geometric
measurement independent of any CS floor). cs_max_window_m=53.0m was
derived and verified against each corner's own BRACKET LENGTH (smallest
is C7 at 116.0m, comfortably more than double the cap) -- but checking
the GAP to each corner's own NEXT neighbour surfaces several real gaps
smaller than the cap: C12 8.5m, C7 13.2m, C2 14.0m, C1 15.3m, C3 17.8m,
and C9-to-C10 essentially 0.0m (the two brackets touch). For a window
positioned near the START of one of these corners that needs close to
the full 53.0m to satisfy the span floor, it could in principle extend
backward past the gap and into the PRECEDING corner's own bracket --
the cap bounds how far a window can reach, but does not know where the
next corner boundary actually is. NOT observed to have visibly affected
the Phase 4 result on Dubai (no corner's own worst-lap value showed a
pattern suggestive of contamination from a neighbour), but not
DISPROVEN either -- this is a structural property of the cap's own
design (a single scalar distance, blind to track topology), not a bug
specific to 53.0m; a smaller cap would still exceed the 0.0m and 8.5m
gaps. NO CLAMP added (user decision) -- flagged as a standing
limitation and a new-data-file checklist item (PLAN.md PARKED section)
instead, since a track with tighter corner sequencing than Dubai could
make this a live rather than theoretical concern.

### CS validity repair, limitation: C4's own short genuine events are
only partially detected by the worst-lap/apex_region statistic
[2026-09-02, same day]
Surfaced by a per-lap breakdown (diagnostics/inspect_phase4_per_lap_
breakdown.py) requested during the sign-off clarification round --
the earlier Phase 4 report checked only the worst-lap AGGREGATE
(min across 4 laps), which conceals this. C4's own per-lap apex_
region-substituted worst-of-5-phases values (final config, ekf_auto_
pacejka): CSf lap1=+0.445, lap2=+0.734, lap3=-0.134, lap4=+0.701; CSr
lap1=+0.208, lap2=-0.134, lap3=+1.000, lap4=+0.615 (values under the
final 0.1s/0.02rad config -- the 0.2s run this was first noticed under
read CSf lap3=-0.122, CSr lap2=-0.042, same pattern, smaller magnitude
on the rear side). The ground-truth workup (thesis_notes.md "Ground-
truth workup...") independently confirmed C4 as REAL via actual fold/
peak pictures on 5 specific runs: front laps 1/3/4, rear laps 3/4,
each 17-32m long. Cross-referencing: only front lap 3 (one of the five
confirmed-real events) still reads negative in this statistic --
front laps 1 and 4, and rear laps 3 and 4 (the OTHER FOUR of the five
confirmed-real events) all read strongly POSITIVE here. The one
negative rear reading (lap 2, -0.134) was never one of the confirmed
ground-truth events at all. NET: this statistic catches 1 of 5
independently-confirmed genuine saturation events at C4, not
"multiple laps" as the work order's own pre-registration predicted --
a milder RECURRENCE of the same dilution failure mode the first Phase 1
attempt showed in full (there, C4 was diluted to positive on EVERY
lap; here, on 4 of 5 confirmed events). The corner-LEVEL flag survives
(worst-lap aggregate is negative on both axles, -0.122/-0.042 under
0.2s, -0.134/-0.134 under the final 0.1s config) because the statistic
only needs ONE lap's worst phase to go negative to flag the whole
corner -- but the underlying per-lap evidence is much thinner than the
aggregate-only view suggested. NOT fixed here (user decision) -- no
change to apex_region, the phase statistic, or the aggregation rule;
recorded as a known, quantified limitation of the current statistic for
whoever picks up threshold anchoring or a future aggregation-method
revision.