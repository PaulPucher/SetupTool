# WP-S6: Sideslip Estimator Comparison Report

Location note: the work order specified `docs/sideslip_comparison_report.md`.
`git ls-files docs/` returns zero tracked files — every existing `docs/`
subdirectory (`car_data/`, `literature/`, `study/`) is in the protected,
gitignored set (proprietary/local-only reference material), with no
precedent for tracked general documentation there. Placed in the repo root
instead, alongside `thesis_notes.md` and `PLAN.md`.

Audience: the user and their supervisor, deciding win/no-win on whether the
Kalman sideslip observer replaces the kinematic estimate as the production
sideslip source. This document reports evidence only — no recommendation.

## 1. The problem

Metric 5 of the sideslip-methods-comparison harness found that at every one
of 14 stable corners, the production kinematic sideslip estimate reads
close to zero at the exact samples where lateral force (Fy) is large and
direction-locked to the turn — physically, a tyre generating several
kilonewtons of lateral force at zero slip angle is not sane for a standard
tyre curve. Follow-up work traced this to the kinematic estimator's washout
high-pass filter, which is designed to correct integration drift but, as a
side effect, also strips out genuine steady-state (low-frequency) sideslip
content, leaving the estimator with no low-frequency truth source for what
the vehicle's sideslip actually does mid-corner.

## 2. Candidates compared

- **A — kinematic (production)**: `estimate_sideslip`, `modules/
  stability_analysis.py`. Kinematic integration of `beta_dot = ay/v -
  yaw_rate` with a 0.05 Hz washout high-pass. This is what the pipeline
  currently uses everywhere sideslip is consumed.
- **B — GPS-course (shelved, negative control)**: `estimate_sideslip_gps`,
  validation-only, never called from any pipeline/UI path. Included in the
  harness purely as a known-quantity sanity check (its numbers were already
  on record before this work started, and reproducing them exactly is a
  standing regression gate), not as a live candidate.
- **C — Kalman observer (tuned, ratio 0.3162)**: `estimate_sideslip_kalman`,
  `diagnostics/sideslip_kalman_observer.py`. Diagnostics-only, no
  production wiring anywhere. Discrete-time linear Kalman filter on the
  bicycle-model state-space form (vehicle model: Rajamani, *Vehicle
  Dynamics and Control*, 2nd ed., sec. 2.3/2.6; Kalman recursion: Rajamani
  Ch. 14 as a worked automotive application — full citations in
  `thesis_notes.md`). Q/R process/measurement-noise ratio tuned via a
  7-point logarithmic sweep (`diagnostics/inspect_kalman_qr_ratio_sweep.py`)
  to ratio=0.3162, an interior point chosen to preserve transient response
  rather than the steady-state-optimal extreme (detail in section 3 below).

## 3. Evidence per candidate

### Harness Metric 1 — straight-line near-zero check
What it measures: median and 90th-percentile |sideslip| during
straight-line driving (low lateral acceleration, low yaw rate) — sideslip
should read near zero here regardless of estimator.
- A: median 0.355°, p90 1.210°
- B: median 1.597°, p90 2.246°
- C: median 0.260°, p90 0.542° (best of the three)

### Harness Metric 2 — cross-method correlation and sign agreement
What it measures: Pearson correlation and per-corner-phase sign agreement
between every pair of candidates, over all valid-lap moving samples.
- A vs B: r=-0.2365, sign agreement 127/257=49.4% (mandatory sanity gate,
  reproduces the on-record shelving numbers — PASSED)
- A vs C: r=+0.2284, sign agreement 140/257=54.5%
- B vs C: r=-0.6459, sign agreement 124/257=48.2%
- Reading: C is weakly, barely-above-chance correlated with A overall — not
  surprising, since A is the method under scrutiny, not a target to match
  (see the explicit non-goal noted throughout this arc: agreement with the
  kinematic estimate was never a tuning target).

### Harness Metric 3 — physical-plausibility envelope
What it measures: p1/p99/max |sideslip| against a generous 20° sane bound
(a diagnostic check for gross unit/construction errors, not a real
handling-limit claim).
- A: p1=-2.842°, p99=+2.336°, max=4.291° — within bound
- B: p1=-2.059°, p99=+5.611°, max=7.609° — within bound
- C: p1=-6.653°, p99=+6.764°, max=11.651° — within bound, widest spread of
  the three, consistent with C tracking genuine mid-corner steady-state
  slip rather than staying compressed near zero

### Harness Metric 4 — inter-lap consistency
What it measures: standard deviation, across a corner's ~4 laps, of that
corner's median sideslip — lower means more repeatable lap-to-lap.
- A: median cross-lap std 0.102°, mean 0.111°, max 0.241°
- B: median 0.402°, mean 0.418°, max 0.684°
- C: median 0.167°, mean 0.172°, max 0.351° — between A and B, and each
  physical corner gets a large, repeatably-signed value across its laps
  (not noise)

### Harness Metric 5 — zero-slip Fy offset and direction-match
What it measures: at the samples where each candidate's own alpha reads
near zero, what is the lateral force (Fy) doing — should be near zero for a
sane tyre curve, and its sign (if nonzero) is checked against turn
direction.
- A: global median Fy_f=-3801N, Fy_r=+6197N; direction match front 13/13,
  rear 14/14 (mandatory regression gate, reproduces the on-record
  `inspect_c9_negative_cs.py` report — PASSED). This is the original
  finding described in section 1: large, direction-locked force at
  supposedly-zero slip.
- B: global median Fy_f=+4616N, Fy_r=-9647N; direction match front 7/7,
  rear 7/7 — same signature, different beta source, evidence the offset is
  not specific to the kinematic construction.
- C: global median Fy_f=-126N, Fy_r=-10N; direction match front 0/4, rear
  2/5 — **this metric is close to vacuous for C**, not evidence of a
  problem: C's alpha rarely reads near zero at all during cornering (most
  corners have zero qualifying samples), which is the expected consequence
  of C tracking genuine ongoing slip rather than letting alpha collapse.

### Force-balance steady-state check (not a harness metric — a physics
cross-check, `diagnostics/inspect_offset_chain_decomposition.py` /
`inspect_observer_self_consistency.py`)
What it measures: the slip angle a standard tyre-force-balance model
demands to support the observed lateral acceleration, compared against
what each estimator reports at the same samples.
- At every one of 14 corners, the physics model demands 0.9-5.8° of
  steady-state rear slip while A's estimate reads ~0° at those exact
  samples — a large, systematic, direction-signed gap.
- C's own alpha_r matches this steady-state expectation's sign at all
  14/14 corners, with comparable order-of-magnitude at most corners after
  correcting for a reference-stiffness bias identified in the comparison
  (the originally-observed 2-3x overshoot at three corners traced to an
  inflated reference stiffness computed from the kinematic candidate's own
  under-reading alpha, not to a flaw in the observer — recomputing that
  reference from the observer's own alpha brought the overshoot to within
  a few percent everywhere except one 3-sample corner).

### Physical sign check (`diagnostics/inspect_sideslip_sign_check.py`) —
the arc's only EXTERNAL validation, not a comparison against another
estimate or a self-consistency check
What it measures: standard bicycle-model physics says sideslip signs
opposite the turn direction at racing speed (rear points to the outside);
turn direction taken from the sign of median lateral acceleration per
corner.
- Observer: matches at all 14 corners (11 of 11 racing-speed corners,
  unchanged after tuning — re-confirmed with the tuned observer wired in).
- Kinematic: matches at 8 of 14. Of the 6 mismatches, 3 (C7, C9, C12) are
  this dataset's only low-speed-class corners, where a sign reversal is
  separately expected by the same physics — at those three, neither method
  is demonstrably wrong by this check alone. That leaves 2 genuinely
  unexplained kinematic mismatches at racing speed (C6, 130.6 km/h; C10,
  150.5 km/h) and one near-zero borderline case (C11, +0.022°, too small
  to read as a decisive sign either way).

### Transient-tracking check (`diagnostics/inspect_kalman_qr_ratio_sweep.py`)
What it measures: correlation between the rate of change of sideslip and
the rate of change of lateral acceleration during corner entry/exit phases
— a setting that smooths away genuine transient response should show this
degrading even if steady-state measures improve.
- At the chosen ratio (0.3162): correlation -0.9896, within about 1
  percentage point of the asymptotic ceiling (-0.998) reached by heavier
  (less-smoothed) settings.
- At the zone that looked steady-state-optimal in isolation (ratio
  0.007-0.05): correlation only -0.70 to -0.91 — a real, measurable loss of
  transient responsiveness that motivated choosing the interior ratio
  0.3162 over that zone.

### Saturation-detection critical check (`diagnostics/inspect_observer_
slip_angle_circularity.py`) — the decisive finding
What it measures: whether cornering-stiffness ratios (CS_ratio, the
production signal used to detect tyre saturation) computed from the
observer's slip angles still carry independent information, or have
collapsed into a restatement of lateral force through the observer's own
fixed stiffness assumption.
- The observer's slip angle explains 99.8% (rear, R²=0.9979) and 99.7%
  (front, R²=0.9971) of lateral-force variance as a straight line, with
  best-fit slopes within 11-12% of the fixed stiffness priors used inside
  the observer (rear: fitted 101055 N/rad vs 91343 N/rad prior; front:
  fitted 60070 N/rad vs 68268 N/rad prior) and residual scatter under 6%
  of the force's own spread.
- CS_ratio compresses toward 1 at both axles when computed from observer
  slip angles (corner-sample p5: rear 0.211→0.716, front 0.107→0.587,
  kinematic→observer).
- Under the CURRENT production thresholds, worst-phase-per-corner-instance
  flagged counts fall from 7 strong + 4 moderate (front) and 5 strong + 4
  moderate (rear), out of 56 instances, to **zero at both axles** using
  observer-derived CS_ratio.
- Mechanism: the observer's measurement equation ties lateral acceleration
  to sideslip through the fixed stiffness prior, and lateral acceleration
  is one of only two correcting measurements — so the state is pulled onto
  the assumed linear tyre relationship regardless of what the tyre is
  actually doing. The measured steering angle enters as a control input,
  not a correcting measurement, which is why the front axle was expected
  to retain more independent content than the rear but did not (0.9971 vs
  0.9979 — essentially the same collapse). General statement: a state
  observer built on a linear tyre model cannot detect departure from tyre
  linearity, because saturation does not exist in its model.
- What this does not invalidate: the diagnosis of the kinematic estimate's
  own failures (sections 1 and 3 above) and the observer's validated
  properties tested by the other checks in this section (sign correctness,
  order-of-magnitude steady-state recovery) — those test direction and
  magnitude, not local linearity, so they are unaffected by this finding.
  Full write-up: `thesis_notes.md`, "Observer saturation-detection
  failure: the decisive finding".

## 4. Honest limits

- **No ground truth for sideslip exists anywhere in this log.** Every check
  in this comparison is either cross-candidate agreement, an external
  physics consistency check (the sign check, the force-balance gap), or
  internal self-consistency (the observer's re-derived stiffness against
  its own alpha) — never an independent measurement of the true sideslip
  angle.
- **Self-consistency is not accuracy.** The observer's re-derived
  cornering-stiffness check (recomputing a reference stiffness from the
  observer's own alpha, then checking it predicts the observer's own
  alpha) is close to circular wherever the underlying regression is
  well-conditioned — a wrong-but-internally-coherent estimator would pass
  the same test. It is evidence the numerical implementation behaves as
  designed, not independent proof the observer is correct.
- **The slide/moment-preservation target is unverifiable on this data.**
  No sustained (≥0.2s) window exists anywhere in the Dubai sample where the
  observer's sideslip exceeds 10° simultaneously with lateral acceleration
  exceeding 0.8g — this session simply contains no genuine large-excursion
  event to test against. Whether the tuned observer preserves a real slide
  rather than smoothing it away remains untested; re-check when a session
  with an actual slide/moment arrives.
- **Low-speed corners are ambiguous on the sign check**, not wrong for
  either method: at low speed the physical sign expectation itself
  reverses, so a "mismatch" against the racing-speed expectation at C7,
  C9, or C12 does not indict either estimator on its own.

## 5. What each decision would trigger

**UPDATE (section 7 below): the decision has been made — NO-WIN, on the
strength of the saturation-detection finding in section 3. The WIN scope
below is kept for the record as what WOULD have been required; none of it
follows from the actual outcome. No production wiring, no threshold
re-derivation, no schema change happens as a result of this comparison.**

**A WIN decision** (the observer replaces the kinematic estimate as the
production sideslip source, or is wired in alongside it) would require, as
follow-on work — none of it done in this comparison:
- Production wiring: `estimate_sideslip_kalman` (or a hardened version of
  it) called from the actual pipeline in place of, or alongside,
  `estimate_sideslip`.
- Threshold re-derivation: per the project's standing estimator-input-change
  rule, any downstream classification threshold that consumes sideslip
  (directly or via alpha/CS_ratio) would need re-derivation against the new
  estimator's output distribution before being trusted — this is not
  optional once a beta-source change reaches production, per CLAUDE.md's
  deviation taxonomy.
- A before/after verdict comparison: re-running the classification pipeline
  on the same session(s) under both estimators and reporting how corner
  verdicts move, so the change's real-world effect is visible, not assumed.
- An accuracy-registry update: `config/parameters.json`'s
  `accuracy_levels.sideslip_angle` node would need a new `source`/`note`
  reflecting the observer's provenance (and the Iz/Caf/Car placeholder
  values currently in use, which are reviewer placeholders, not sourced
  figures, and would need flagging or replacing before production use).
- A schema version bump: `ANALYSIS_SCHEMA_VERSION` (whatever consumers key
  their cache/persistence off it) would need bumping, per the project's own
  existing convention for any change to what Module 2-6 output actually
  contains.

**A NO-WIN decision** would instead record: the observer is a validated,
physically-sign-correct diagnostic candidate that is not being promoted to
production at this time, with the reasons (whichever of the honest limits
above were decisive) stated explicitly in `thesis_notes.md`, and the
kinematic estimate remains the production sideslip source with its known
limitation (the washout-suppressed zero-slip offset, sections 1 and 3
above) documented and unresolved.

## 6. No recommendation

This report deliberately stops short of recommending a decision. The
evidence above is presented for the user and their supervisor to weigh —
the win/no-win call, and any follow-on scope from section 5, belongs to
them.

## 7. Decision recorded: NO-WIN [2026-08-20]

The observer is **not adopted into production**. Decisive reasoning: the
saturation-detection critical check (section 3) found that CS_ratio
computed from the observer's slip angles collapses to a near-constant
value at both axles (R²>0.997 against a straight line through the fixed
stiffness prior at both axles, zero flagged instances under current
thresholds versus a real population of 7 strong + 4 moderate front and 5
strong + 4 moderate rear from the production kinematic path). Since
cornering-stiffness-ratio and saturation/tyre-limit detection are a core
part of what this pipeline uses sideslip-derived quantities for, an
estimator that structurally cannot support that use case is not a viable
production replacement, independent of its other validated properties
(sign correctness, steady-state order-of-magnitude recovery).

The kinematic estimate remains the production sideslip source, with its
own known limitation (the washout-suppressed zero-slip offset, sections 1
and 3) documented and unresolved. The observer remains a documented
diagnostic instrument (`diagnostics/sideslip_kalman_observer.py`), useful
for the purpose that motivated building it — locating where and roughly
how large the kinematic estimate's steady-state suppression is — but not
usable as a CS_ratio input as currently constructed. No production wiring,
threshold re-derivation, before/after verdict comparison, accuracy-
registry update, or schema version bump follows from this comparison (see
section 5's update above). Future work that could in principle restore
saturation detection (a nonlinear-tyre observer, or an adaptively
estimated rather than fixed stiffness prior) is named but not pursued —
both carry their own circularity problem (slip angles are needed to fit
the tyre curve that in turn produces slip angles) needing explicit
resolution first. Full write-up: `thesis_notes.md`, "Observer saturation-
detection failure: the decisive finding".
