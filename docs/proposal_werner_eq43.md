# Proposal: completing Werner Eq. 4.3 (yaw-damping term D_psi)

Status: PROPOSAL ONLY, 2026-09-29, branch backlog-g. Nothing implemented.
Tier A (vehicle-dynamics quantity), reviewed before any code is written.
Full verified anchor record: thesis_notes.md "Werner Eq. 4.3 completion:
anchor check".

## a. The anchor

Werner 2021 (MA), sec. 4.5.2 "Korrelation des Giermoments", p.56:

> "Das Giermoment setzt sich zusammen aus einem Teil, der von der
> Rotationsträgheit abhängt und einem Teil, der von der Gierdämpfung
> abhängt." [my translation: the yaw moment consists of an inertia part
> and a yaw-damping part.]
>
> "Die Gierdämpfung ergibt sich aus den effektiven
> Schräglaufsteifigkeiten der Vorder- und Hinterachse sowie den
> Hebelarmen der Achsen zum Schwerpunkt und der
> Fahrzeuggeschwindigkeit." [my translation: the yaw damping follows from
> the effective cornering stiffnesses of front and rear axle, the axles'
> lever arms to the CG, and the vehicle speed.]

    Eq. 4.3   Mz    = Iz * psi_ddot + D_psi * psi_dot
    Eq. 4.4   D_psi = (C_alpha,f,eff * l_f^2 + C_alpha,r,eff * l_r^2) / v

"Effective" is the local tangent slope: it falls in the nonlinear range and
"reaches zero at the lateral-force maximum" (p.56-57). Werner needed Fz only
to evaluate that slope through his Pacejka tyre model (p.56); without
wheel-load sensors he stopped there, and noted that for his car the inertia
part was negligible next to the damping part (p.57).

Second anchor, Milliken RCVD sec. 5.7, Eq. 5.6, p.149:
N = (aC_F - bC_R) beta + (1/V)(a^2 C_F + b^2 C_R) r - aC_F delta. The
r-coefficient is D_psi (Milliken's C carry a negative sign by convention,
so D_psi = -N_r).

What this licenses: computing D_psi from per-axle effective cornering
stiffness, CG lever arms and speed, and forming the damping moment
D_psi * psi_dot next to the inertial moment Iz * psi_ddot. It does NOT
license any particular way of obtaining the effective stiffness. That
part is the project's existing, already-documented adaptation (below).

## b. The construction

Every input already exists in the pipeline; nothing new is measured.

1. C_alpha_f, C_alpha_r [N/rad], per sample, per axle: Module 4b
   (estimate_cornering_stiffness), the local windowed dFy/dalpha on logged
   Fy and alpha. This is the same quantity Werner calls C_alpha,eff (local
   tangent, zero or negative past the peak) and the same adaptation the
   project already documents for CS_ratio: "effective Calpha is estimated
   directly from logged Fy/alpha" instead of from a tyre model.
2. l_f = wheelbase * rear_fraction, l_r = wheelbase * front_fraction,
   from the same static corner-weight split estimate_lateral_forces uses
   (cog_position). This keeps D_psi consistent with the Fy the stiffness
   was fitted on.
3. v = state v_mps (ecu_speed); psi_dot = state yaw_rate_radps
   (sclu_yaw_rate).
4. D_psi = (C_alpha_f * l_f^2 + C_alpha_r * l_r^2) / v  [N m s/rad]
5. Mz_damping = D_psi * psi_dot  [N m];  Mz_inertial = Iz * psi_ddot, the
   existing Module 5 quantity, reused, not recomputed.
6. Also returned: the two axle contributions separately
   (C_alpha_f * l_f^2 / v and C_alpha_r * l_r^2 / v) so a reader can see
   which axle drives the damping and which one is missing.

ROLE OF MEASURED Fz: none in this construction. This corrects the BACKLOG G
close-out note of 2026-09-28 ("measured Fz is better placed as a D_psi
input"), which is true only on Werner's own route. That route, a tyre model
evaluated at the measured (alpha, Fz), would run through this project's
kinematic-seeded fitted Pacejka curve, and the load-normalised
D = mu*Fz fit that stays non-production. It would inherit the documented
beta/tyre-curve identifiability limit. It is NOT proposed. Stated as an
alternative only, so the choice is visible.

DEPENDENCIES THAT DO REMAIN, stated: C_alpha depends on alpha, hence on the
sideslip source. Under production ekf_auto_pacejka, beta comes from an EKF
whose internal model is the fitted curve. D_psi therefore inherits
whatever beta error CS_ratio already carries, no more and no less. It does
not add a new dependency on the tyre model.

## c. Degradation

- Availability follows Module 4b validity, not the damper channels.
  D_psi is NaN wherever C_alpha_f or C_alpha_r is NaN (invalid window,
  not moving, kerb-masked). The per-axle contributions stay available
  individually. Never substitute the linear reference stiffness or a
  config constant into a missing axle, since that would silently mix a
  measured slope with a Level-1 number.
- v below the moving threshold -> NaN, not a division by a small number.
- Dubai's dead RR travel pot and v3's corrected FR gauge do not matter
  here: no Fz enters. The same holds for any damper/wheel-load degradation
  on either session.
- C_alpha < 0 past the peak gives a reduced or negative D_psi. That is
  the physics Werner describes ("reaches zero at the maximum"), reported
  as is, not clipped.

## d. Consumer (minimal)

v1 = one pure function plus one diagnostic, nothing else:
- a pure function in modules/ (e.g. estimate_yaw_damping(state, cs,
  params)), no Qt, not called by the production pipeline;
- diagnostics/inspect_yaw_damping_eq43.py: per corner and phase, the
  magnitude of Mz_damping vs Mz_inertial on both real sessions. It answers
  the project's own limitation #6 ("Iz*psidd-only ... same order of
  magnitude"). Pre-registered expectation, order of magnitude only:
  damping is comparable to or larger than inertia at race speed, as
  Werner found for his car.

Not in v1: no Module 5 wiring, no UI, no analysis payload field, no PDF.
Feeding D_psi * psi_dot into Module 5's regressand (which today is
mz_inertial only; the chair's reference is also inertial-only) would be an
estimator-input change. It would re-trigger the stab_neg_thresh
re-derivation, itself blocked (no negative population under
ekf_auto_pacejka), and would be a DOMAIN IMPROVEMENT deviation from the
chair. That is a separate v2 proposal, gated on the v1 result.

## e. What this does NOT do

No change to CS_ratio, C_alpha, beta, Fy, Module 5 or any threshold. No
config value changes (no new tunable: every input already exists, and
Eq. 4.4 has no free parameter). No payload/schema change, so
ANALYSIS_SCHEMA_VERSION and the goldens are untouched. Because no
estimator input and no classification input changes, no re-derivation
duty is triggered in v1.

## f. Accuracy levels

- C_alpha_f/r: Level 1. It is derived from lateral_force_split (L1,
  capped by yaw_inertia/corner_weights) and from alpha, which is capped by
  sideslip_angle (L1). The registry has no own node for it today.
- l_f, l_r: cog_position, Level 1 by default, Level 2 when a session
  weighing is on record (resolve_accuracy).
- v: speed, Level 1 (provenance-assumption, opaque ECU calibration).
  psi_dot: yaw_rate, Level 3.
- D_psi and Mz_damping: Level 1 by the weakest-link rule. New registry
  node proposed: accuracy_levels.yaw_damping, capped_by chained-constant:
  lateral_force_split, sideslip_angle, cog_position, speed.

## g. Tests

- Hand-derived unit case: C_f = C_r = 100000 N/rad, l_f = 1.1 m,
  l_r = 1.4 m, v = 40 m/s gives D_psi = (100000*1.21 + 100000*1.96)/40 =
  7925 N m s/rad; with psi_dot = 0.5 rad/s, Mz_damping = 3962.5 N m.
- NaN propagation: one axle NaN -> D_psi NaN, that axle's contribution
  NaN, the other axle's contribution finite.
- v at/below the moving threshold -> NaN.
- Sign case: C_f < 0 (past peak) reduces D_psi, no clipping.
- Untouched, asserted by running them unchanged: test_golden_pipeline.py,
  test_golden_auto_modes.py, test_stability.py. The function has no
  pipeline caller, so these cannot move.
