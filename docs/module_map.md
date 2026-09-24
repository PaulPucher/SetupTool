# Module map

One section per file in `modules/` and `core/`, written from the actual
code and docstrings (WP-CLEAN Phase 3, 2026-09-24), not from memory or
from the README/PLAN.md summaries. Each section states: (1) what the
module does, (2) the decisions that shaped it, with their thesis_notes.md
entry names, (3) its method anchor under the project's Tier A/B/C system,
(4) the config it reads. Cross-references to
`docs/thesis_material_index.md` are given where that document already
indexes a module's own story in more depth.

Pipeline order roughly follows how a session actually flows through the
app: parse -> segment corners -> Modules 1-5 (state, sideslip, forces,
loads, CS_ratio, yaw stability) -> longitudinal/damper/tyre-fit
extensions -> accuracy resolution and caching -> recommendation /
decision layer -> PDF/figure export. `core/` modules are listed after
`modules/` since none of them contain vehicle-dynamics logic.

---

## modules/csv_parser.py

**What it does.** Reads a Cosworth Pi Toolbox ASCII export and turns it
into the channel dict every later module consumes. Handles two real
export layouts (NARROW: one `{ChannelBlock}` per channel; WIDE: one
block, header row names every channel, one data row per timestamp),
European decimal notation, latin-1 encoding, and only the channels
`config/channels.json` whitelists. Also splits the session into laps,
merges the session-trailing pit fragment into its preceding lap, and
classifies out/in laps by pit-limiter engagement, additive to the
existing `lap_number==0` positional rule.

**Decisions that shaped it.**
- latin-1 over utf-8+replace: a prior utf-8 decode silently turned every
  degree sign into U+FFFD. thesis_notes.md "Full channel census +
  targeted verification (2622 channels)" and "GT3 Paul Ricard export:
  diagnosis and fix".
- WIDE-layout support: added once a second real export (GT3 Paul Ricard)
  turned out to use a completely different `{ChannelBlock}` shape from
  Dubai's NARROW layout — same entry as above.
- Pit-limiter-based out/in-lap classification, additive to the
  positional rule, handles a pit box before start/finish (found on
  GT3_PRC_MLA-v3.txt): thesis_notes.md "Fz-integration Phase 4:
  pit-limiter-based out/in-lap classification".
- `channel_corrections`: evidence-gated additive-offset decoding fix for
  a specific channel/session (e.g. v3's `log_dms_dam_fr`), guarded by a
  `precondition_mean_range` so a differently-faulted or healthy future
  export of the same channel name is never silently mis-corrected.
  thesis_notes.md "Deepening Phase 2: FR gauge decoding correction,
  SHIPPED".

**Method anchor.** Tier B/C throughout — file-format parsing and
lap-segmentation bookkeeping, no vehicle-dynamics claim of its own.

**Config read.** `config/channels.json` (channel whitelist, ranges,
quality gates, `channel_corrections`, `corner_speed_thresholds`,
`lap_splitting.pit_fragment_max_duration_s`).

---

## modules/corner_analysis.py

**What it does.** Segments each lap into corners and assigns phase
boundaries (Entry 1 Brake / Entry 2 Turn-in / Apex 3 / Exit 4 / Exit 5),
then links the same physical corner across laps into a `stable_corner_id`
and realizes one canonical bracket/geometry per stable corner. The
module's own top-of-file docstring lays out the full 9-step algorithm
(bracket by steering/lateral-G with hysteresis, apex by lateral-G peak
cross-checked with speed minimum, classify by apex speed, merge
short-gap same-direction brackets, cross-lap identity via a two-pass
deterministic split, canonical realization by median-window
re-inversion, representative-lap filtering). Falls back to speed-minima
bracketing / speed-minimum apex / throttle-collapsed Entry-1 when
steering, lateral-G, or throttle channels are unavailable.

**Decisions that shaped it.**
- Two-pass straddler reassignment (`_reassign_straddlers_pass2`): a
  once-per-corner-pair decision against each cluster's confident-member
  canonical window, not a per-lap race — WP1 Turn 1.
- Representative-lap filtering (a lap outside
  `lap_time_representative_factor` of the session's fastest valid lap
  can be realized against every canonical corner but cannot itself seed
  one or shape its geometry): 2026-09-03 corner canonicalisation fix,
  thesis_notes.md "v3 diagnostics, Part B1/B2" — same fragment-lap
  plausibility idea as csv_parser's `valid_lap_max_ratio`, one level up.

**Method anchor.** Tier B (signal/data engineering: segmentation,
clustering, thresholds) — explicitly not presented as a vehicle-dynamics
method. See thesis_notes.md for the primary dual-criterion (steering +
lateral-G) design rationale.

**Config read.** `config/channels.json` (`corner_speed_thresholds`,
steering/lateral-G bracket thresholds, `min_apex_speed_drop_kmh`, the
representative-lap factor).

---

## modules/geo.py

**What it does.** Two tiny shared primitives: an equirectangular
GPS-to-local-metres projection (fine at track scale, no proper geodesic
projection needed) and a GPS origin picker (the first valid sample of
the lat/lon channels). Used wherever a track map or corner position
needs a local x/y frame before a resampled vehicle state exists yet.

**Decisions that shaped it.** None load-bearing beyond the projection
choice itself — the module docstring flags that this origin can differ
by a fraction of a second (and so a small distance) from
`prepare_vehicle_state`'s own GPS origin in stability_analysis.py, since
the two call sites anchor at different moments; both are valid local
origins for the same formula.

**Method anchor.** Tier C/B — a standard small-area map projection, no
literature anchor needed (equirectangular is the textbook choice at this
scale).

**Config read.** None.

---

## modules/stability_analysis.py

**What it does.** The largest and most central module: builds the common
resampled vehicle state (`prepare_vehicle_state`, Module 1), then
Modules 2-5 — kinematic and GPS-course sideslip estimation, slip angles,
axle lateral forces (2-DOF force/moment balance), vertical loads
(static split, or a measured-damper cascade via `modules/wheel_loads.py`
when configured), cornering-stiffness ratio (CS_ratio, Module 4b), and
cross-lap yaw-moment stability (dMz/dbeta, Module 5). Also owns
`summarise_corners`, which turns the raw per-sample arrays into the
per-lap-per-corner-per-phase summary dicts every later consumer
(recommendation engine, decision frame, UI, PDF export) reads.

**Decisions that shaped it.** This file carries the project's single
largest concentration of documented method decisions; the module's own
`ANALYSIS_SCHEMA_VERSION` history (bumped 1 through 8 in the file
header) is itself a decision log of every payload-shape change. Notable
ones:
- CS_ratio's adaptive window widening + distance-capped locality bound
  (`reconstruct_cs_window_start`/`resolve_cs_min_window_samples`): CS
  validity repair, thesis_notes.md "CS validity repair, part A" and its
  Phase 1/2 REVISION entries.
- The 100 Hz common-grid resampling in `prepare_vehicle_state` (ecu_speed
  is natively 50 Hz, five other CS-chain channels are natively 100 Hz —
  downsampling them onto the coarser grid was discarding real
  resolution): thesis_notes.md "100 Hz time-base work package, Phase 0".
- The measured-Fz cascade switch in `estimate_vertical_loads` (Tier B
  consumption switch into `modules/wheel_loads.py`, not a new
  vehicle-dynamics method itself): thesis_notes.md "Fz-integration Phase
  1".
- `estimate_yaw_moment_stability` deliberately does NOT port the chair's
  time-anchored fallback mode when `s_m` is unusable — a differently-
  behaving estimator the s-grid thresholds could not classify
  meaningfully. [domain improvement]

**Method anchor.** Mixed, function by function:
- `estimate_sideslip` (kinematic beta): Tier A, Werner MA method
  (thesis_notes.md "WP-S4" entry); washout integration around it is Tier
  B.
- `estimate_lateral_forces` (Module 4a): Tier A, chair performance_analysis
  tooling (internal), adopted as-is (thesis_notes.md "Fy yaw-moment term
  (Module 4a)").
- `estimate_vertical_loads` (static split): Tier A, chair tooling, adopted
  as-is (thesis_notes.md "WP5b(b) phase 1: chair-parity vertical loads").
- `estimate_cornering_stiffness` (Module 4b, CS_ratio): Tier A, **Werner
  MA method**, adapted (windowed regression from logged Fy/alpha in
  place of Werner's Pacejka-model evaluation) — thesis_notes.md
  "CS_ratio (cornering stiffness ratio) -- Werner MA method".
- `estimate_yaw_moment_stability` (Module 5): target relation is Werner
  (Mz = Iz*psidd + D_psi*psid, thesis_notes.md "Yaw moment stability
  dMz/dbeta"); the estimator construction itself is after the chair
  tooling, not Werner's own construction.

**Config read.** `config/parameters.json` almost in full: `vehicle`,
`stability_estimation` (window floors/spans, cutoff frequencies, kerb
thresholds, apex/phase gating), plus `config/car_data.json` via
`load_car_data()`.

Cross-reference: docs/thesis_material_index.md Ch.2/Ch.3 index this
module's CS_ratio/sideslip story in much more depth than this map
repeats.

---

## modules/yaw_stability.py

**What it does.** Two pure functions consumed only by
`stability_analysis.estimate_yaw_moment_stability`: a centred
rolling-mean yaw-acceleration filter, and the s-anchored
Gaussian-weighted local ridge regression that produces dMz/dbeta at a
grid of track-distance points (pooling samples across laps at the same
`s_m`), then interpolated back onto every sample.

**Decisions that shaped it.** `GAUSS_SIGMA_DIVISOR = 2.5` is kept as a
named code constant, not a config value, per the project's own
method-defining-constant rule (CLAUDE.md) — it fixes the weighting
*shape*, not a per-track calibration. The raw-yaw-rate-only path (no
pre-smoothed-input option) is a stated forced adaptation: the chair's
own function also accepts a pre-smoothed yaw-rate input from a
chair-external filter list outside this reference's scope.

**Method anchor.** Tier A for the target relation (Werner, cited from the
calling module); the estimator construction itself (rolling mean +
s-anchored Gaussian ridge regression) is after the chair
performance_analysis tooling (internal), not part of Werner's method —
thesis_notes.md "Yaw moment stability dMz/dbeta".

**Config read.** None directly — every tunable (window_m, min_samples,
ridge, grid_step_m, min_beta_std_rad) is passed in by the caller from
`config/parameters.json`'s `stability_estimation` block.

---

## modules/longitudinal_forces.py

**What it does.** Axle longitudinal force Fx (fallback tier: no direct
per-wheel/per-axle Fx channel exists) and per-axle kinematic slip ratio
kappa. Fx = m·ax + drag + rolling, braking split by measured front/rear
brake pressure, driving assigned entirely to the rear axle
(rear-wheel-drive GT3R). Also owns the wheel-speed plausibility guard
(`_guarded_wheel_speed_kmh`/`_rolling_plausibility_mask`) that falls
back from `log_speed_*` to `abs_speed_*` when a corner's own wheel-speed
channel goes stuck or disagrees implausibly with its axle mate.

**Decisions that shaped it.**
- Wheel-speed guard's mate-disagreement tie-breaker: an ecu_speed-based
  check that attributes which SIDE of a mate disagreement is at fault,
  after a first version flagged v3's healthy `log_speed_rl` almost as
  often as its genuinely faulty mate `log_speed_rr`. thesis_notes.md
  "Fz-integration Phase 5: wheel-speed plausibility guard".
- Rear rolling-radius correction (`rear_rolling_radius_offset`, +1.41%):
  a measured, throttle-independent offset, not a slip artifact — WP-S1
  wheel-speed source characterization.
- Front axle left uncorrected: its braking-specific deviation from
  ecu_speed IS the front-slip-under-braking signal this ratio exists to
  measure.

**Method anchor.** Tier A for the Fx fallback-tier construction: same as
the chair performance_analysis tooling's own third fallback tier,
adopted as-is (thesis_notes.md "Citation cross-reference, modules/
longitudinal_forces.py"). The wheel-speed guard is Tier B (signal
plausibility engineering).

**Config read.** `config/parameters.json`'s `longitudinal_stiffness`
(drag/rolling coefficients, brake-split fallback fraction, rolling-radius
offset, min_speed_mps) and `wheel_speed_guard` (window_s, std_min_kmh,
ratio_max_deviation) blocks; `vehicle.aero`, `vehicle.mass_kg`.

---

## modules/longitudinal_stiffness.py

**What it does.** LS_ratio (longitudinal stiffness ratio), the
longitudinal analogue of CS_ratio: a Butterworth-filtered sliding-window
local least-squares slope dFx/dkappa, reported against a low-slip linear
reference, on the same 1=linear/0=peak/<0=beyond scale as CS_ratio.
Also carries a kerb-strike plausibility guard that excludes a sample
only when its kappa is implausible AND a vertical-acceleration channel
shows kerb-like disturbance nearby — never on kappa magnitude alone.

**Decisions that shaped it.**
- Two documented deviations from the chair's own dataclass defaults,
  both decided 2026-08-30 (thesis_notes.md): `min_samples` is rate-derived
  from the actual log rate rather than a transplanted literal (the
  chair's literal 25 is structurally unsatisfiable at this car's 50 Hz
  log); and the additive az-coincidence plausibility guard has no chair
  equivalent at all, forced by this car's kerb-spike behaviour.
- The az-coincidence design constraint is explicitly load-bearing: a
  large kappa excursion with NO az disturbance nearby is exactly the
  traction-limited signal this estimator exists to measure (PLAN.md STEP
  3 Phase 4's C3 finding) — excluding on kappa alone would erase it.
- The adaptive-widening/distance-capped window (mirroring CS_ratio's own
  playbook): "Metrology extension Phase 2: LS_ratio validity repair",
  2026-09-19.

**Method anchor.** Tier A for the ratio construction itself (windowed
OLS slope, low-slip reference, clip-at-1.0) — the chair's own estimator,
adopted with the two documented Tier B deviations above. thesis_notes.md,
"Citation cross-reference, modules/longitudinal_forces.py" (inputs) and
the Phase 2 repair entry (window mechanics).

**Config read.** `config/parameters.json`'s `longitudinal_stiffness`
(cutoff_hz, linear_slip_threshold, min_window_s/min_window_samples_floor/
min_slip_span/max_window_m, plausibility_kappa_bound,
plausibility_az_window_front/rear_s) and `stability_estimation`
(kerb_z_deviation_threshold_g, kerb_baseline_g).

---

## modules/wheel_loads.py

**What it does.** Per-wheel vertical load (Fz) from damper/suspension-
travel channels, as a four-term additive decomposition: sprung force at
the wheel (measured pushrod force through the digitised motion-ratio
table — this also carries the elastic share of lateral transfer), ARB
force (a separate load path invisible to the pushrod gauge), unsprung-
mass lateral transfer, and sprung-mass geometric lateral transfer via the
roll centre. Also derives a session-corrected axle-total model (mass and
aero jointly fit from this session's own straight-line damper data) used
only by the missing-corner reconstruction path, and reconstructs a
single invalid corner from its axle-total model plus its real axle mate.

**Decisions that shaped it.**
- The session-corrected axle-total regression: mass and aero
  recovered jointly from ONE fit (`total_fz = static_total + c·v²`) after
  an earlier two-fit version was found to double-count aero between the
  mean term and the v² term. thesis_notes.md "Metrology extension Phase
  1: mass/aero double-counting fix".
- `_axle_total_with_proxy`'s generalisation from a v3-specific hardcoded
  FR-from-FL proxy to a per-axle "real where valid, ratio-proxied from
  the mate otherwise" rule, after the hardcoded version silently NaN'd
  on Dubai (found by looking at a figure with a missing rear-axle trace,
  not from a number). thesis_notes.md "Fz-integration Phase 1".
- Bump-rubber engagement is explicitly NOT modelled — a documented
  underestimate at extreme compression, not a defect (config
  `wheel_loads.bump_rubber_note`).
- Aero front/rear split stays the Level-1 config placeholder even after
  mass/aero correction: straight-line data alone cannot measure how aero
  splits front/rear (no differential signal at near-zero roll).

**Method anchor.** Tier A. Segers, *Analysis Techniques for Racecar Data
Acquisition* (SAE, 2014), ch.9 (pushrod/damper force to wheel load via
motion ratio) and ch.10 (roll-centre geometric vs spring-path elastic
load-transfer split) — page anchors ch.9 p.199, ch.10 pp.221-256,
verified 2026-09-03 against the docs/literature excerpt. The
session-corrected axle-total regression and the missing-corner
reconstruction are Tier B (data engineering built on top of the Tier A
decomposition, not a new physical claim).

**Config read.** `config/parameters.json`'s `wheel_loads` block
(pushrod offsets, dead-channel std thresholds, tyre dynamic radius,
roll-centre heights, unsprung masses, ARB position fallback) and
`config/car_data.json` (`motion_ratio_vs_wheel_travel`, `arb`).

Cross-reference: docs/thesis_material_index.md Ch.4 indexes the whole
damper package (Segers anchor, showcase/ground-truth/aero-gap figures)
in depth.

---

## modules/damper_motion.py

**What it does.** Per-corner, per-transient-phase loading/unloading
classification from the 100 Hz `log_susp_travel_*` channels, feeding the
decision frame's `damper_motion` evidence type. Reuses
`modules/wheel_loads.py`'s own channel-access/dead-channel/unit-
normalisation helpers directly rather than a second copy.

**Decisions that shaped it.**
- Empirically-verified sign convention (decreasing travel = compression
  = "loading"): correlated front-axle travel against braking ax on BOTH
  real sessions (+0.49 Dubai, +0.20 v3), same technique as the existing
  ARB sign-convention check. thesis_notes.md "Damper motion sign-
  convention and threshold derivation".
- The motion-vs-noise rate threshold could not use the originally
  planned apex_3-vs-transient comparison (apex_3 proved too narrow for
  any ≥2-sample rate window on either session) — reused the project's
  existing straight-line mask (|ax|<0.5, |ay|<0.5 g) as the reference
  population instead.

**Method anchor.** Tier B throughout (derivative, threshold, validity
floor) — explicitly never presented as a vehicle-dynamics method of its
own. Method pointer: thesis_notes.md "Frame depth programme..." entries;
docs/segers_bridge_review.md C11-1/C11-2 (Segers ch.11: dampers develop
force only while the shaft has velocity, never at steady-state apex
cornering) motivates WHY this evidence type exists, without this module
itself being a Segers-anchored estimator.

**Config read.** `config/decision_frame.json`'s `damper_motion` block
(`rate_threshold_mm_s`, `min_valid_fraction`) and
`config/parameters.json`'s `wheel_loads.dead_channel_std_max_travel_mm`
(reused, not re-derived).

---

## modules/tyre_model.py

**What it does.** The Dugoff lateral tyre model: `dugoff_lateral_force`
and its analytic derivative `dugoff_lateral_stiffness`, pure-cornering
reduction only (no combined slip, no longitudinal slip term).

**Decisions that shaped it.** Drops the literature's leading minus sign:
the SAE-style textbook form is Fy = -c_alpha·tan(alpha)·f(lambda), but
this codebase's own slip-angle sign convention already produces a
positive Fy-vs-alpha slope (confirmed empirically, both axles,
corr(alpha, Fy_filt) > 0) — so the module uses Fy =
+c_alpha·tan(alpha)·f(lambda) to match, without changing the shape or
either parameter's sign.

**Method anchor.** Tier A, thesis_notes.md "WP-N1: Dugoff tyre model
chosen + first-pass fit, identifiability finding".

**Config read.** None — pure function of its own arguments; callers
supply c_alpha/mu_fz from `config/parameters.json`'s `tyre_model_fit`/
`tyre_model_ekf` blocks.

---

## modules/tyre_model_pacejka.py

**What it does.** The reduced 4-parameter Magic Formula lateral tyre
model (`pacejka_lateral_force`/`pacejka_lateral_stiffness`), a parallel
"new code path" alongside `tyre_model.py`'s Dugoff pair, never editing
that existing production file.

**Decisions that shaped it.** Kept as a fully separate module rather
than a Dugoff-module edit, per PLAN.md's own "new code path" instruction
for this phase (the parent work order's hard constraint did not
authorise editing `tyre_model.py`). Parameter order (B, C, D, E) and
starting guess `(12, 1.9, 8000, 0.97)` match the chair's own tooling
verbatim.

**Method anchor.** Tier A, cited as "chair performance_analysis tooling
(internal)" for the reduced-form construction; thesis_notes.md "Phase 3:
Pacejka variant -- pre-registration".

**Config read.** None — pure function; callers supply B/C/D/E from the
tyre-fit chain's own fit results.

---

## modules/tyre_fit_auto.py

**What it does.** The one-shot per-session tyre-curve fit + EKF
validation chain: fits c_alpha/mu_fz (Dugoff) or B/C/D/E (Pacejka) per
axle from this session's own CS_ratio-gated linear-regime samples, runs
the corresponding EKF sideslip observer
(`modules/sideslip_ekf_dugoff.py`/`sideslip_ekf_pacejka.py`, imported
directly rather than duplicated), and validates the result
(NIS/sign-check/onset-coverage). `resolve_sideslip_beta` is the actual
production dispatcher — despite the module's own header comment calling
the fit machinery "not called from the UI directly", `resolve_sideslip_
beta` at the bottom of the file IS called directly by
`ui/views/outing_form.py`'s `StabilityAnalysisThread` for every
`sideslip_source` including `ekf_auto_dugoff`/`ekf_auto_pacejka`.

**Decisions that shaped it.**
- Imports `modules/sideslip_ekf_dugoff.py`/`sideslip_ekf_pacejka.py`
  rather than duplicating their ~150 lines of Jacobian/update code each.
  Until 2026-09-24 (WP-CLEAN relocation mini-package) those two files
  lived in `diagnostics/`, making this the one place in `modules/` that
  reached into `diagnostics/` — a neutral-engineering dependency
  inversion, not a science decision, and now resolved: both files are
  ordinary `modules/` siblings.
- `_fit_axle`'s bracket-widening + ≥0.95-bound-fraction degenerate check
  reproduces `fit_dugoff_first_pass.py`'s loop exactly — the safeguard
  that must catch a rear-axle degeneracy (mu_fz drifting to the bracket
  ceiling, curve collapsing to pure-linear) rather than silently
  accepting it.
- `resolve_sideslip_beta`'s fallback-to-kinematic behaviour (fit
  degenerate, or NIS gate verdicts "fail") is never silent — the reason
  is always recorded as text in `fallback_reason`.

**Method anchor.** Tier A for the fit procedure lineage (c_alpha/mu_fz
from `fit_dugoff_first_pass.py`'s WP-N1b method, R-noise-model from
`inspect_ekf_pass1_rQ_sweep.py`'s grid, EKF recursion from
`sideslip_ekf_dugoff.py`/`sideslip_ekf_pacejka.py`, validation from
`inspect_pass1_final_validation.py`'s five sections) — see the module's
own header for the full pointer chain into thesis_notes.md. The NIS gate
consumption (`resolve_sideslip_beta`'s fallback logic) is Tier B.

**Config read.** `config/parameters.json`'s `tyre_fit_auto` block (Q/R/P0
seeds, R-sweep grid, NIS band) and `tyre_model_fit`/`tyre_model_ekf`
(mu_fz bound-widening parameters); `config/parameters.json`'s `nis_gate`
via `modules/nis_gate.py`.

---

## modules/sideslip_ekf_dugoff.py

**What it does.** The nonlinear single-track EKF sideslip observer,
Dugoff tyre model, pass 0 — states `[beta, yaw_rate]`, input the front
steering angle, measurements `[yaw_rate, ay]`. Relocated here from
`diagnostics/` on 2026-09-24 (WP-CLEAN relocation mini-package): its own
module docstring had claimed "diagnostics-only, no modules/ consumer",
but `modules/tyre_fit_auto.py` imports `estimate_sideslip_ekf_dugoff`
from it directly at runtime, and so do `tests/test_pure_functions.py`
and `tests/test_nis_gate.py` — a real production dependency living in
the wrong directory, not a diagnostics script that happened to get
reused. No PyQt6 import (verified before the move); no other change
made during relocation beyond the import path and a header correction.

**Decisions that shaped it.** Model equations and EKF treatment: Rajamani
sec. 2.3/2.6 (bicycle model) + Ulsoy/Peng/Cakmakci sec. 14.1/14.3
(sideslip significance), Rajamani Ch. 14 (Kalman application) —
thesis_notes.md "WP-N2: nonlinear Dugoff EKF proposal". Production
callers must use `beta_with_fallback`, never the raw `beta` — the raw
series keeps diverged-window artifacts for diagnostics figures by
design; production must never feed a silently-diverged state downstream.

**Method anchor.** Tier A. thesis_notes.md "WP-N2: nonlinear Dugoff EKF
proposal"; Dugoff force/stiffness terms delegate to `modules/
tyre_model.py` (same anchor as that module).

**Config read.** `config/parameters.json`'s `tyre_model_ekf` blocks
(Q/R/P0 per pass, `beta_hard_bound`, NIS divergence thresholds).

---

## modules/sideslip_ekf_pacejka.py

**What it does.** The same EKF sideslip observer as
`sideslip_ekf_dugoff.py`, with the reduced 4-parameter Magic Formula
(`modules/tyre_model_pacejka.py`) substituted for Dugoff in the state
propagation and both Jacobians — a structural mirror, not an independent
implementation. Same relocation history and same production-dependency
status as `sideslip_ekf_dugoff.py` (moved from `diagnostics/`
2026-09-24; `modules/tyre_fit_auto.py` imports
`estimate_sideslip_ekf_pacejka` from it directly).

**Decisions that shaped it.** Built as a new, fully separate code path
rather than a `sideslip_ekf_dugoff.py` edit, per PLAN.md's own Phase 3
work order ("do not modify the existing Dugoff path") — everything
except the tyre-force/stiffness calls (states, measurements,
discretization, divergence monitor, fallback behaviour) is identical to
the Dugoff filter by construction.

**Method anchor.** Tier A, same EKF/model anchors as
`sideslip_ekf_dugoff.py`; the Pacejka force/stiffness terms delegate to
`modules/tyre_model_pacejka.py` (chair performance_analysis tooling,
internal, thesis_notes.md "Phase 3: Pacejka variant").

**Config read.** Same `tyre_model_ekf` blocks as `sideslip_ekf_dugoff.py`.

---

## modules/nis_gate.py

**What it does.** The NIS (normalised innovation squared) tyre-mismatch
health gate: a rolling windowed-NIS-exceedance-fraction score, classified
pass/warn/fail, answering "does the fitted tyre curve match this
session's data well enough to trust EKF beta?" Ports
`diagnostics/inspect_nis_tyre_mismatch_gate.py`'s WP-N3 prototype into a
reusable module.

**Decisions that shaped it.**
- `resolve_nis_window_samples`: the window is a PHYSICAL duration
  (`nis_window_s`, 0.4s), not a literal sample count — the prior literal
  `window_samples=20` was silently a 50Hz-calibrated value, wrong on any
  other grid rate. Same rate-derivation pattern as
  `stability_analysis.resolve_cs_min_window_samples`.
- Two pre-registered predictions about the score's own behaviour were
  tested and found to have FAILED (recorded, not hidden): the absolute
  ceiling sits far below the naive ~85% expectation (small-window
  binomial noise caps it near 60-65% even for a perfectly calibrated
  filter), and mu_fz mismatches hurt scoring MORE than c_alpha
  mismatches, the opposite of the original prediction.
- NaN health score always classifies "fail", never "pass" — the "never
  silently pass on a degenerate input" rule.

**Method anchor.** Tier B (a reusable statistical health check built on
the EKF's own windowed-NIS machinery, not a new vehicle-dynamics claim).
PROVISIONAL: every threshold in `config/parameters.json`'s `nis_gate`
namespace is commented as such — five data points from one session.
thesis_notes.md "Phase 4: NIS tyre-mismatch gate -- results".

**Config read.** `config/parameters.json`'s `nis_gate` block
(`nis_window_s`, `nis_band_low/high`, `threshold_use_ekf`,
`threshold_warn`).

---

## modules/accuracy_resolution.py

**What it does.** Resolves the static accuracy-level registry
(`config/parameters.json`'s `accuracy_levels`) against per-outing
`setup_data` and an optional global cap, for the three dynamically-wired
leaf nodes (mass, corner_weights, steering_ratio) plus their two pure
cascades (cog_position, steering_angle). Every other registry node stays
at its static declared level regardless of cap. Produces both the
resolved values Modules 1-5 actually consume
(`apply_resolved_vehicle`) and the level map the UI/cache layers report.

**Decisions that shaped it.**
- yaw_inertia/lateral_force_split are explicitly NOT cascaded
  dynamically even though they are chain-limited by mass/corner_weights
  — yaw_inertia's m·a·b estimate carries its own method ceiling of 1
  regardless of how well its inputs are known, so wiring the cascade
  would be a no-op until the ceiling itself changes (a different Iz
  measurement method).
- Mass priority when two Level-2 sources are both available (explicit
  `setup_data.total_weight` vs. derived corner-weight sum): explicit
  wins, never blended — the "highest available wins" standing rule — with
  an independent consistency warning if the two disagree beyond
  `MASS_CORNER_SUM_TOLERANCE`.
- steering_ratio (WP-B) is explicitly stated as a parameterization
  upgrade, not a deviation from any chair position — the 15.7 constant
  was never a chair-adopted method, it is this car's own digitised
  mechanical geometry.

**Method anchor.** Tier C/B — pure per-session data-availability
resolution logic, no vehicle-dynamics claim of its own (the physical
quantities it resolves are governed by the Level 1-4 accuracy system,
not by this module).

**Config read.** `config/parameters.json`'s `accuracy_levels` registry
and `vehicle` block; `config/car_data.json`'s `steering_ratio_table`.

---

## modules/pipeline_sidecar.py

**What it does.** WP-CACHE's gzip-compressed pickle persistence of the
full Modules 1-5 pipeline result (state/cs/stab/fz/ls/slip/forces) per
outing, alongside the existing DB summary cache — closes the gap where
graphs/trace dialogs need the large numpy-array-bearing outputs directly
but the in-memory pipeline cache dies with the process.

**Decisions that shaped it.**
- Pickle over npz: the payload mixes numpy-array-heavy dicts with small
  nested Python structures (corners) that npz cannot hold without
  falling back to `allow_pickle` object arrays anyway — pickle handles
  both uniformly in one file.
- Atomic write (tmp file + `os.replace`): a sidecar exists whole or not
  at all, never truncated/partial.
- `write_sidecar` never raises — a write failure is logged and swallowed,
  since it must never fail the analysis that produced the payload being
  cached.
- Explicit PICKLE SAFETY note: `pickle.load` is used only on files this
  same module wrote itself under `data/analysis_cache/`, never on a
  transferred or foreign file.

**Method anchor.** Tier C (engineering/caching infrastructure, no
science content). thesis_notes.md "WP-CACHE Phase 1: sidecar
implementation + Phase 1e real-data measurement".

**Config read.** None — the 7 identity fields it compares
(`IDENTITY_FIELDS`) mirror the WP5 DB-cache check's own fields, not a
separate config block.

---

## modules/recommendation.py

**What it does.** The original 39-rule recommendation engine: converts
per-lap-per-corner stability summaries, driver feedback, and
`config/recommendations.json`'s rule table into a ranked,
evidence-backed list of setup direction suggestions. As of Frame-Stage-2
(2026-09-04) its own `generate_recommendations()` is no longer surfaced
by any UI — `modules/decision_frame.py`'s frame migrated all 39 rules as
candidate bridges and passed parity on real data — but this module is
UNCHANGED and still the source of truth: its rule table, config, and
every helper `decision_frame.py` imports (`aggregate_by_corner`,
`_phase_verdict`, `load_recommendations_config`, etc.) are what the
frame actually calls.

**Decisions that shaped it.**
- WP2b-2: rules re-sourced from an external engineer decision matrix
  (scenario × speed-class grid, `cell_id`), referencing real
  `config/setup_parameters.json` registry keys instead of the WP2
  placeholder `front_arb`/`rear_arb` labels.
- The undrivable-feedback escalation tier
  (`_apply_undrivable_escalation`): at `|raw feedback| >= 4` the tool
  must never render silent emptiness for that corner — pierces every
  soft cap, synthesizes a "no elicited rule covers this" row, or flags
  an urgent contradiction, per lap-level (not aggregate-only) evidence.
  A turn-2 repair (2026-07-27) was needed because the original
  aggregate-only check could dilute a real per-lap pattern to "normal".
  thesis_notes.md "Undrivable tier: lap-level cell matching".
- ADVISORY vs RECOMMENDED action-class split: unsubstantiated
  moderate-severity data-only matches stay ADVISORY (observation, never
  budget-eligible) — mild understeer is this car's deliberate stable
  baseline, and the elicitation is biased against unnecessary changes
  when the driver disagrees with the data.

**Method anchor.** Tier C/B (rule-matching, scoring, and ranking
plumbing on top of Modules 1-5's Tier A/B outputs — no vehicle-dynamics
claim of its own).

**Config read.** `config/recommendations.json` (the full rule table,
escalation config, feedback-override thresholds) and
`config/setup_parameters.json` (the registry `_current_setup_value`/
`_check_feasible` check against).

---

## modules/decision_frame.py

**What it does.** The current production recommendation UI's engine (the
old `recommendation.py`-driven Recommendations section was removed once
this passed parity). A three-layer frame — evidence (`build_evidence`) →
candidates (`generate_candidates`, sourced from both the migrated
39-rule matrix bridges and newer lever-bridge conditions) → scoring
(`score`, a six-term cost function) — plus a conflict resolver
(`resolve_conflicts`) and the terse-UI rendering helpers
(`render_top_line`/`render_tail_line`/`group_display_rows`). Every
reachable lever always carries a status (proposed / no-trigger /
blocked-at-edge / contradicted / not-assessable) — silent
unreachability is structurally impossible by design (DECISION LAYER
SPEC's own design principle).

**Decisions that shaped it.** This file is governed end to end by the
DECISION LAYER SPEC in PLAN.md (elicited 2026-09-22); nearly every
function has its own named Stage/Phase decision. The largest:
- Full Stage-2 migration (2026-09-04): all 39 `recommendations.json`
  rules re-expressed as candidate bridges
  (`_bridge_candidates_for_matrix_rules`), parity-verified against the
  old engine on both real sessions
  (`diagnostics/inspect_frame_stage2_parity.py`) before the old UI
  section was removed.
- The six-term `score()` cost function (REVISED Phase C, "C1:
  scoring-term fold"): problem weight (severity × phase_importance ×
  confidence) folds phase_importance IN since it qualifies the problem,
  not the lever's fit to it; effect_class is deliberately kept SEPARATE
  from problem weight since the same problem can yield a different
  effect_class from different candidate levers — a category-error
  argument recorded in thesis_notes.md.
- `_apply_eligibility_gate`: heavy correctors (springs, toe, camber) fire
  only on multi-corner same-axle strong patterns or high driver
  feedback, never on a single-corner moderate signal alone.
- Contradiction status (B6): a required `evidence_corroboration`
  condition that actively disagrees (not just absent) produces
  `status=contradicted` in the tail, distinct from a silent structural
  non-generation.
- Feedback-only routing (B2) and several `lever_bridges` condition types
  are fully implemented and tested but currently inert on real data (no
  config entry yet declares a `damper_motion`/`intervention_abs`/
  `intervention_tc` condition) — a stated gap, not a bug.

**Method anchor.** Tier C/B throughout — a scoring/routing/UI-rendering
frame built on top of Modules 1-5's Tier A outputs and
`recommendation.py`'s Tier C rule matching; no vehicle-dynamics claim of
its own. `damper_motion`/`intervention_abs`/`intervention_tc` evidence
types' WHY (not their own classification logic) traces to Segers
(docs/segers_bridge_review.md C-series).

**Config read.** `config/decision_frame.json` in full (eligibility
classes, cost_function weights, lever_bridges, interaction_table,
damper_motion thresholds, contradiction_sources) and
`config/setup_parameters.json` (the lever registry) and
`config/recommendations.json` (via the imported matrix-rule helpers).

Cross-reference: docs/thesis_material_index.md Ch.5 indexes the whole
decision-layer story (figures, open elicitation gaps) in depth.

---

## core/config_loader.py

**What it does.** Two tiny functions: load `config/car.json`, and pull
its `setup_parameters` key. Returns `None`/`{}` on any read/parse
failure rather than raising.

**Decisions/method anchor.** None — trivial config plumbing, Tier C.

**Config read.** `config/car.json`.

---

## core/error_text.py

**What it does.** One shared function, `friendly_error_text`, formatting
any caught exception into one readable line: a `ValueError` with real
text is trusted as already human-readable (this codebase's own
convention for validation-boundary errors); any other exception type is
prefixed with its class name, since those are more often an
unanticipated bug than a deliberately-worded message.

**Decisions that shaped it.** Extracted specifically so both `ui/`
(status labels, QMessageBox text) and `core/` (inline PDF error notes)
call sites share one formatting rule, and so it is directly testable
without pulling PyQt6 into the test suite.

**Method anchor.** Tier C.

**Config read.** None.

---

## core/setup_data_points.py

**What it does.** Splitter/diffuser measurement-point JSON reshape
helpers (`reshape_points_out`/`reshape_points_in`, flat
`splitter_point_1.._5` widget keys ↔ a plain 5-element array), plus the
physical point positions (`SPLITTER_POINT_POSITIONS`/
`DIFFUSER_POINT_POSITIONS`) shared by the measurement-points widget and
the PDF export diagram.

**Decisions that shaped it.** Positions were extracted programmatically
(connected-component clustering of marker pixels on a user-annotated
reference screenshot), then symmetrized across the vertical centreline —
the dots were hand-placed and the car is left/right symmetric, so raw
mirror-pair coordinates carried hand jitter the underlying geometry does
not have. Full extraction/symmetrization record: thesis_notes.md "8.
Splitter/diffuser measurement points, position re-extraction" and
"...symmetrized". Extracted out of `ui/views/outing_form.py` specifically
so it is testable without importing PyQt6.

**Method anchor.** Tier C (UI/product geometry, no vehicle-dynamics
content).

**Config read.** None — operates on the outing's own JSON setup blob
passed in.

---

## core/plot_style.py

**What it does.** Shared plot styling constants (colours, line widths,
marker sizes, PRINT/INTERACTIVE theme dicts, export geometry) for both
the interactive pyqtgraph corner-trace dialog and the static matplotlib
export/diagnostic figures — one source of truth so a curve means the
same colour on screen and in a thesis figure. Deliberately import-free
of PyQt6/pyqtgraph/matplotlib itself, so it is safe to import from
`core/`, `ui/`, and a headless diagnostics script alike.

**Decisions that shaped it.** The Part B redesign (2026-09-01): lap
identity is now carried by COLOUR (`lap_styles`, assigned dynamically to
the checked set in ascending lap order), while axle/quantity identity is
carried by PANEL, not colour — retiring the old per-quantity colour
constants. `LAP_PALETTE` is matplotlib's own "tab10" categorical palette
verbatim, hardcoded (not imported) to keep this module dependency-free.
Several later "corrections batch"/"corrections round 3" entries re-tuned
marker sizes/alphas after visual review — recorded inline, not restated
here.

**Method anchor.** Tier C — pure UI/product styling, explicitly "no
science content: nothing here is a measurement, threshold, or method
choice."

**Config read.** None.

---

## core/figure_render.py

**What it does.** The shared matplotlib figure renderer: pure display
functions (`render_corner_figure`, `render_verdict_traces_figure`,
`render_lap_figure`, `save_png`) that draw arrays handed to them,
computing nothing about corners, brackets, windows, or slopes
themselves. Two callers share every function: the app's own "Export
figure"/"Export verdict traces" buttons, and
`diagnostics/inspect_step2_chair_plots.py`'s batch export — so an app
export and a diagnostics-script export of the same corner are visually
identical.

**Decisions that shaped it.** The Part B redesign and the same-day
corrections batch (2026-09-01): removed the bold/faint "emphasised lap"
line-width distinction entirely (colour is now the only lap-identity
carrier); reworked `render_corner_figure`'s layout from a three-column
squeeze (unreadable tyre-curve panels) into a stacked layout with a
full-width tyre-curve row.

**Method anchor.** Tier C — "no estimation logic," by its own module
docstring.

**Config read.** None — reads `core/plot_style.py`'s constants, not
config files.

---

## core/pdf_export.py

**What it does.** The car setup/setdown sheet PDF export: landscape A4,
monochrome, one shared strip renderer (`build_session_strip`) reused at
two scales — "large" fills a whole page (single-session print), "small"
is reused by `core/weekend_pdf_export.py` to pack strips onto a weekend
summary page.

**Decisions that shaped it.** The shared-strip-renderer design itself is
the headline decision: thesis_notes.md "PDF layout rework: shared strip
renderer" — one renderer for both scales rather than two independently
maintained layouts. Team branding is a config-swappable logo path
(`config/images/team_logo.png`), no code change needed to rebrand.

**Method anchor.** Tier C.

**Config read.** `config/images/team_logo.png` (referenced, not a JSON
config); the outing's own setup-sheet data structure (not a config file).

---

## core/weekend_pdf_export.py

**What it does.** The weekend PDF export: a cover summary page, a
Setup/Setdown sheets section (two strips per page, one outing's own
pair, via `core/pdf_export.py`'s `build_session_strip` at "small"
scale), then one section per outing for what the setup sheets don't
cover — resolved accuracy footer, verdict summary, recommendations,
driver feedback.

**Decisions that shaped it.** The verdict trust rule (Guard-A/B
consistency, stated explicitly in the module docstring): a verdict is
only ever printed for an outing whose cached
`analysis_data.schema_version` matches the current
`ANALYSIS_SCHEMA_VERSION`, and even then it is classified LIVE from
current config against the stored summaries, never read as a cached
value — mirroring exactly how `ui/views/outing_form.py`'s own render
path works. A stale-schema or missing analysis prints "Not analysed
under current version" instead of any verdict section. Follow-up item 4
changed the page layout from four strips (two outings) to two strips
(one outing) per page after a content-density ceiling was flagged as
open in a prior review round.

**Method anchor.** Tier C.

**Config read.** None directly — imports `ANALYSIS_SCHEMA_VERSION` from
`modules/stability_analysis.py` and recommendation helpers from
`modules/recommendation.py`; reads `models.base.Session`/
`models.driver.Driver` for outing/driver data, not config files.
