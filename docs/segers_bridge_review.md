# Segers Deep-Dive: Reasoned Bridge Review

Source: Jorge Segers, *Analysis Techniques for Racecar Data Acquisition*, 2nd ed.
(SAE International, 2014). `docs/literature/` (read-only, not committed).
Published literature — cited normally (author, chapter, page), unlike the
chair's internal tooling (never cited this way; see CLAUDE.md deviation
taxonomy).

Scope: only levers present in the parameter registry (config/setup_parameters.json).
Every candidate below is PROPOSED grade; nothing in this document is
implemented. Verdict axis vocabulary (config/decision_frame.json,
`_comment_performance_axes`): `understeer_tendency`, `oversteer_tendency`,
`yaw_stability` (the only three that currently score); `braking_performance`,
`traction_performance`, `platform_stability` (schema-present, structurally
inert — no `_AXIS_TO_VERDICT` mapping yet).

Reading plan and chapter in/out justification: see session record (not
duplicated here; consolidated into Phase 3 below once all chapters are read).

---

## Chapter 11 — Shock Absorbers (pp.257-286)

Continuation of the PENDING SCHEMA SUPPORT item named in thesis_notes.md
("Literature-bridge work package: coverage check and ride-height platform
bridges" — item 3, damper transient-phase bridges). Cross-checked against
the codebase first (channel-census rule applied to our own code, not just
raw telemetry): zero references to shock velocity, histogram, zero-bin,
skewness, or kurtosis anywhere in modules/ or diagnostics/. The only
consumer of `log_dms_dam_*`/`log_susp_travel_*` today is modules/wheel_loads.py
(Fz reconstruction) — Chapter 11's entire velocity/statistical toolkit is
genuinely new territory, not an overlap.

### C11-1 — Low-speed damping sets transfer RATE, not magnitude (boundary statement)

a. **Statement** (p.260-261): shock-setting changes aimed at corner
entry/exit balance are made in the low-speed range (<25 mm/s); in this
range damping rate governs how quickly weight transfers to/from each
corner during roll/pitch, i.e. how fast each tire is (un)loaded. Stated
boundary (p.261): a low-speed front damping change has NO influence during
steady-state cornering, since there is no shaft velocity there to act on.

b. **Mechanism**: shock force is proportional to shaft velocity (F = C·v).
During a transient the shaft is moving; LS damping changes how fast the
corner reaches its quasi-static load. Final load magnitude is set by
springs/ARBs (ch.9/10, already encoded) — dampers only govern the RATE of
approach to it, and only while shaft velocity is non-zero.

c. **Applicability**: transfers directly — this is a kinematic/rate
statement independent of vehicle class, aero, or drivetrain layout.

d. **Cross-check**: NEW. No interaction_table or lever_bridges entry
references any `damper_*` parameter — confirmed by direct read of
config/decision_frame.json.

e. **VERDICT: INSIGHT-ONLY.** Not itself a directional bridge (no sign, no
verdict pairing) — it is the physics boundary that governs where C11-2
through C11-5 below are allowed to apply (transient phases only, never
apex/steady-state), and is cited from those entries rather than duplicated.

f. n/a.

### C11-2 — Front LS rebound decrease reduces understeer at turn-in (worked example)

a. **Statement** (p.262-263, Table 11.1, Zolder T1 worked example): "in the
case of understeer during initial cornering, front rebound could be
decreased to improve the compliance between the left-front tire and the
track surface." Table 11.1 places LF in rebound during the book's Section 2
(initial cornering + continued braking) — the closest match in our own
phase vocabulary is `entry_2_turnin`.

b. **Mechanism**: at turn-in the unloading (outside-of-roll) front wheel
extends into rebound as roll builds. Excess LS rebound damping resists that
extension, delaying the tire regaining full contact-patch compliance,
reducing the front axle's effective transient grip — read by the driver as
understeer. Reducing LS rebound lets that wheel follow the road faster.

c. **Applicability**: mechanism transfers. Two things differ from the
book's context, both named rather than silently carried over:
  - **Corner-direction dependency**: the book's example is asymmetric by
    construction — LF is the unloading wheel on THIS left-hand corner; on a
    right-hander it would be RF. Our registry keeps `damper_rebound_ls_fl`
    and `_fr` independent, so a correct encoding needs to know WHICH front
    corner is unloading at a given corner/phase — evidence our decision
    frame does not currently produce (see (d)).
  - **Aero, GT3-specific**: this car carries meaningful downforce even at
    turn-in speed (measured, not book-assumed); added front load from aero
    partially offsets the rebound-lag mechanism at high-speed corners, less
    so at low-speed ones. A caveat on magnitude, not on direction — the
    book gives no aero-car magnitude to begin with, so this doesn't
    contradict it, just bounds where the effect is strongest.

d. **Cross-check**: NEW parameter family, no existing bridge. Same
verdict/phase pairing (`understeer`, `entry_2_turnin`) as the already-shipped
`springs_front`/soften bridge, but a genuinely DIFFERENT mechanism (transient
rate vs. steady-state roll stiffness) and a NARROWER phase scope — springs
correctly also fire at `apex_3`/`exit_4`/`exit_5` (steady-state load transfer
is present there), while a damper LS bridge must NOT, per C11-1's own
boundary statement. No contradiction with the springs entry; a
complementary, phase-disjoint mechanism at the same evidence trigger.

e. **VERDICT: ADAPT — encoding target REQUIRES a schema/evidence extension,
named precisely, not silently worked around.** The mechanism and the
`entry_2_turnin`-only phase restriction are adoptable as-is. What blocks
encoding is NOT a missing "phase field" in the generic sense — `lever_bridges`
already carries per-entry `phase_groups` (confirmed against the shipped
springs entries) and that granularity is sufficient. The actual gap is that
`lever_bridges`' `condition` schema is verdict-only (`{verdict, min_severity}`)
and no evidence source characterizes PER-CORNER damper motion state
(bump/rebound direction, by phase) at all — confirmed by the same code
census in the chapter preamble. Encoding this bridge at the axle level
(assume FL/FR move together) would be physically wrong, since only the
unloading corner should trigger it — that would misrepresent an
asymmetric mechanism as a symmetric one, which the honesty rules here
forbid.

f. **Encoding target**: `lever_bridge` entries for `damper_rebound_ls_fl`
and `damper_rebound_ls_fr` (kept independent, matching the registry),
condition = `understeer` + `min_severity`, `phase_groups: [["entry_2_turnin"]]`
only. Blocked on: (1) a new evidence source building per-corner damper
motion-state (bump/rebound direction, LS/HS regime) from
`log_dms_dam_*`/`log_susp_travel_*`, and (2) a `lever_bridges` condition type
that can gate on "this specific corner is unloading" rather than only on
verdict. Both named as requirements here, neither built.

### C11-3 — Apply symmetric L/R deltas even when the diagnosis is asymmetric (annotation)

a. **Statement** (p.263): per-corner shock tables tend to suggest
asymmetric settings, "because often during cornering the wheels on the
opposite side of an axle experience opposite movement... concentrate on the
wheel creating the least amount of grip (usually the wheel on the inside of
the corner) and keep the changes in damping the same on the left- and
right-hand sides."

b. **Mechanism**: a car sees both left- and right-hand corners over a lap;
an L/R-asymmetric setting tuned for one corner direction actively hurts its
mirror-image corner. A symmetric axle-wide delta, sized for the
worse-affected (inside/lower-grip) wheel, is the generalizable
recommendation.

c. **Applicability**: transfers directly — a general race-engineering
caution independent of vehicle class.

d. **Cross-check**: nothing in config/decision_frame.json currently states
this as an explicit rule for ANY lever family. Springs/ARB bridges already
apply symmetric axle-wide deltas in practice, so this caution is already
being followed by convention, just never written down as a rule.

e. **VERDICT: ADOPT, as an ANNOTATION**, not a new interaction_table row or
lever_bridge — it governs how a future per-corner damper bridge (C11-2)
should present its recommended delta (symmetric on the axle) even though
the triggering evidence is single-corner.

f. **Encoding target**: a documentation-level note (e.g. a `symmetry_note`
field or config comment) attached to the damper `lever_bridges` block
whenever C11-2 becomes implementable. Not an independent config row.

### C11-4 — High-speed damping trades contact-patch load variation for average grip

a. **Statement** (p.265, Table 11.2 speed-range breakdown): <5 mm/s =
suspension friction (not meaningfully tunable); 5-25 mm/s = "low-speed",
governs driver-felt transient handling; >25 mm/s = "high-speed", road
input, "should be optimized to minimize tire contact patch load
variation... rarely assessed reliably by a driver."

b. **Mechanism**: at high shaft velocities the input is road-surface-driven
(bumps, kerbs) rather than chassis-attitude-driven. HS damping controls the
rate at which normal load oscillates in response, which affects average
available grip via tyre load-sensitivity (non-linear mu(Fz)), not
cornering balance per se.

c. **Applicability**: transfers directly, and this project has its OWN
empirical reinforcement for why it matters here specifically: the mu-vs-Fz
load-sensitivity finding confirmed on clean v3 front data (e8101ad) is
direct evidence that THIS car's tyres lose real peak grip to load
oscillation — strengthening, not just repeating, the book's general claim.

d. **Cross-check**: maps to `traction_performance` (corner exit) /
`braking_performance` (corner entry) — schema-present but currently inert
axes, same status as the existing `platform_stability` entries. No
contradiction; a legitimate first use of axes that have sat unused since
Deepening Phase 4b.

e. **VERDICT: ADAPT — direction only, no magnitude, stated as such.** The
book itself gives no numeric guidance ("no easy answer... depends on
experience with a specific type of car") — any encoded entry must not
invent a threshold or sign magnitude the source doesn't provide. This also
is not an understeer/oversteer bridge at all (no balance-tendency claim),
only a grip-magnitude one, so it can only ever target the inert axes, not
the three that currently score.

f. **Encoding target**: `interaction_table` entries for
`damper_bump_hs_{fl,fr,rl,rr}` / `damper_rebound_hs_{fl,fr,rl,rr}`, axis
`traction_performance` (exit-phase corners) and `braking_performance`
(entry-phase corners). Sign convention deliberately NOT assigned here —
the book gives "more control = less load variation = more grip" but no
directional magnitude answerable at PROPOSED grade; left as an open
scoring-layer decision for implementation, not silently invented.

### C11-5 — Front LS rebound as an aero-platform (rake) lever, not a balance lever

a. **Statement** (p.273-274): "excessive front rebound damping applied to
jack the car's nose down to improve airflow under the car... the front
downforce pushes the front of the car down, creating a greater rake
angle... A high rebound damping helps keep it there."

b. **Mechanism**: under sustained aero load the front axle is pushed toward
a lower/more-compressed attitude; high REBOUND damping resists the axle
re-extending between load events, holding a compressed (nose-down,
higher-rake) attitude longer than spring/ARB rate alone would — an average
aero-platform effect (front/rear aero balance via rake), not a transient
handling effect.

c. **Applicability**: HIGH confidence transfer, arguably more applicable
here than to the book's median example car — this project already carries
its own documented, currently-unresolved aero-platform limitation (the
measured front/rear aero split and its rake sensitivity, flagged as a
genuine identifiability limit of 3-gauge straight-line data in the
Fz-integration work). A heavy-aero GT3 is exactly the class the book's own
caveat is written for.

d. **Cross-check**: same axis (`platform_stability`) and same physical
target (aero platform / rake) as the ALREADY-SHIPPED `ride_height_front/rear`
and `springs_*` `platform_stability` entries (Deepening Phase 4b /
literature-bridge package) — this extends that same Segers ch.9/10-adjacent
mechanism family to a new lever, not a new physics claim. One real
divergence, flagged rather than silently absorbed: the existing
`platform_stability` entries use sign -1 with the convention "either
direction trades platform margin" (a magnitude-only cost, no preferred
direction — matching springs, where stiffening OR softening both cost
platform margin). This statement is explicitly DIRECTIONAL (more rebound →
more rake; less rebound → less rake, not "either way costs margin") —
encoding it under the existing symmetric-cost convention would
misrepresent what the book actually claims.

e. **VERDICT: ADOPT, with the sign-convention conflict named as an
implementation decision, not resolved here.** The mechanism, lever, and
axis are all adoptable; the directional-vs-magnitude-only convention clash
with the existing `platform_stability` entries needs a decision (extend the
convention to allow signed entries, or encode this one differently) before
it can actually ship.

f. **Encoding target**: `interaction_table` entry, `damper_rebound_ls_front`
(axle-level — rake is a whole-axle geometric quantity, and nothing in this
statement is per-corner, unlike C11-2), direction `increase` → axis
`platform_stability`, DIRECTIONAL sign. Ships only once (e) above is
resolved.

### Future damper-analysis feature list (not setup-behaviour bridges — Tier B/C tooling this chapter's own data would support)

Not lever_bridge/interaction_table candidates; recorded because the user
asked whether our channels could support the book's own analysis
techniques. All of these are currently UNBUILT (confirmed: zero references
anywhere in modules/ or diagnostics/ to shock velocity, histograms,
zero-bin height, skewness, or kurtosis) — `log_dms_dam_*`/`log_susp_travel_*`
are consumed today ONLY by wheel_loads.py's Fz reconstruction, nothing
downstream uses them for damper motion analysis itself.

- **Shock velocity math channel** (11.1, Eq.11.3 — derivative of suspension
  travel): SUPPORTED. `log_susp_travel_{fl,fr,rl,rr}` logged at 100 Hz on
  GT3_PRC_MLA-v3.txt (verified sample rate, Phase 2 validation census,
  thesis_notes.md "Damper package" entry) — sufficient resolution for a
  clean numerical derivative. This is also exactly the missing evidence
  source C11-2 is blocked on.
- **Shock speed histogram, zero-bin height, LS/HS time-share, bump/rebound
  asymmetry** (11.3, 11.4.1-11.4.3.2): SUPPORTED by the velocity channel
  above. Would need Tier B binning/threshold choices — the book's 25 mm/s
  LS/HS cutoff is its OWN worked example, not a universal constant, so this
  car's own cutoff would need own-data derivation, per the project's
  standing rule that classification thresholds are always re-derived from
  this car's own distribution, never carried over from a source.
- **Median/variance/skewness/kurtosis statistics** (11.4.3.3-11.4.3.6):
  SUPPORTED, standard descriptive statistics, Tier B, no new channels
  needed beyond the velocity channel.
- **Shock speed box plot** (11.5): SUPPORTED, same data; visualization
  only (Tier C) once the velocity channel and percentile calc exist.
- **Run charts / lap-by-lap trend tracking** (11.6): SUPPORTED IN
  PRINCIPLE, but GT3_PRC_MLA-v3.txt is currently the ONLY session with
  these channels live at all (Dubai has none — the original damper
  package's own "Dubai untouched" note) — a data-availability gap, not a
  method gap; a trend chart needs multiple comparable sessions.

### Chapter 11 census: 5 candidates (0 ADOPT clean / 2 ADOPT-with-open-decision / 2 ADAPT-blocked-on-schema / 1 INSIGHT-ONLY), 0 REJECT, 0 contradictions with existing bridges.

---

## Chapter 9 — Quantifying Roll Stiffness Distribution (pp.199-220)

This is one of the two chapters already used as the citation anchor for the
shipped `springs_front/rear` `platform_stability`/understeer/oversteer
bridges and the ARB matrix coverage. Re-read in full for anything not yet
harvested. Net result: mostly CONFIRMS existing encoding rather than adding
new candidates, plus two genuine registry gaps worth recording plainly
rather than working around. Empty/confirmatory results are still results.

### C9-1 — Front roll-stiffness decrease relieves understeer, rear increase relieves understeer (worked example, 9.3)

a. **Statement** (p.208-212): worked numeric example — resolving a
steady-state understeer problem by decreasing front roll stiffness while
increasing rear roll stiffness (via spring-rate changes), holding overall
roll gradient constant, shifts the roll-stiffness distribution from 67.4%
front-biased toward 65%/... rearward.

b. **Mechanism**: identical to the mechanism already cited for the shipped
`springs_front`/soften and `springs_rear`/stiffen `understeer_tendency`
entries (sign +1 each) — less front roll stiffness / more rear roll
stiffness shifts relative lateral load transfer rearward, increasing the
rear axle's relative slip-angle demand and reducing the front's, countering
understeer.

c. **Applicability**: transfers directly — no GT3-specific caveat needed,
this is the same book, same chapter, already used as this project's own
anchor.

d. **Cross-check**: OVERLAP, not new. `springs_front`/soften/understeer and
`springs_rear`/stiffen/understeer are both already shipped (`lever_bridges`,
BACKLOG item H, 2026-09-20). This worked example is independent
confirmation that the existing encoding matches the book's own numeric
worked case, not just its qualitative claim — worth recording as
corroboration, not re-encoding.

e. **VERDICT: REJECT (as new) — already ADOPTED.** No action; recorded here
only as a citation-strength note (the existing bridges are now backed by
both the qualitative ch.9/10 claim, from the earlier package, AND this
chapter's own worked numeric example).

f. n/a — targets the existing `lever_bridges` entries, no new row.

### C9-2 — Front ARB full-soft → full-hard shifts roll ratio forward (real-world example, 9.5)

a. **Statement** (p.215-216, Table 9.7, Dodge Viper/Zolder data): moving the
front ARB from softest to hardest position decreased both front and rear
roll gradients (total roll stiffness increased) with the front decreasing
more, increasing the roll ratio λ (rear/front roll angle) from 1.84 to
2.18 — i.e. shifted the roll-stiffness distribution toward the front.

b. **Mechanism**: same roll-stiffness-distribution mechanism as C9-1,
applied to the ARB rather than the spring, consistent with springs and ARB
being parallel roll-stiffness elements (Eq.9.11/9.12, already the basis for
the existing `arb_*` matrix coverage).

c. **Applicability**: transfers directly.

d. **Cross-check**: OVERLAP. `arb_fl/fr/rl/rr` are already matrix-covered in
every direction physics supports — confirmed directly against
config/decision_frame.json's interaction_table (the literature-bridge
coverage-check package's own finding, 2026-09-20) and re-confirmed here by
grep: entries exist for both stiffen and soften directions on
understeer_tendency/oversteer_tendency. No gap to fill.

e. **VERDICT: REJECT (as new) — already ADOPTED.** Same disposition as C9-1:
independent numeric corroboration of already-shipped coverage, not a new
candidate.

f. n/a.

### C9-3 — Tire pressure is a first-order roll-stiffness-distribution contributor (registry gap, not a bridge)

a. **Statement** (p.212-214, Table 9.6 + worked example): tire spring rate
(directly set by tire pressure) is one of the parallel roll-stiffness
elements at each axle; a worked example shows a 10% increase in rear tire
spring rate requires a 25% increase in rear wheel spring rate to restore
the original roll ratio — tire pressure changes measurably shift roll
balance, disproportionately to the pressure change itself.

b. **Mechanism**: tire, spring, and ARB roll stiffnesses are all parallel
spring elements at each axle (Eq.9.11/9.12); changing any one changes the
axle's total roll stiffness and hence the front/rear distribution, exactly
like a spring or ARB change would.

c. **Applicability**: mechanism transfers to any car with pneumatic tyres,
GT3 included — no vehicle-class caveat needed.

d. **Cross-check**: NOT ENCODABLE. Direct read of config/setup_parameters.json
confirms there is no `tire_pressure` (or equivalent) key anywhere in the
registry — the only tire-adjacent levers are `camber_fl/fr/rl/rr` and
`toe_front/rear`. Scope rule 1 (this package) restricts candidates to
registry levers only, so this mechanism has no lever to attach to, not a
contradiction with anything encoded.

e. **VERDICT: INSIGHT-ONLY — registry gap named, not worked around.** This
is a real, book-anchored, first-order lever (Segers' own worked example
shows a 10% tire-pressure-driven spring-rate change roughly matching a 25%
wheel-rate change in leverage) that this project's parameter registry
currently cannot represent at all. Recorded as a gap for the parameter
registry itself, not something this package can propose a bridge for.

f. No encoding target exists. If a `tire_pressure_front/rear` (or per-corner)
lever is ever added to config/setup_parameters.json, this entry (and the
existing `interaction_table`'s roll-stiffness-distribution family) would be
the first candidate to revisit.

### C9-4 — Roll-ratio linearity/trend as a component-health and puncture-detection signal (9.6)

a. **Statement** (p.218-219): a broken rear ARB blade shows up as a
discontinuity in the front-vs-rear roll-angle X-Y relationship (rear roll
angle stays near zero until front roll angle exceeds a threshold, instead
of the usual linear relationship). Separately, a slowly increasing roll
ratio across a 12-lap run correctly identified a front tire losing pressure
before the driver reported a puncture (cross-referenced against Table 9.6's
own direction-of-effect table).

b. **Mechanism**: both are anomaly-detection applications of the same roll
gradient/roll-ratio math channels described earlier in the chapter — a
sudden non-linearity indicates a broken/disconnected component, a slow
drift indicates a slowly-changing physical parameter (tire pressure) that
the roll-ratio metric is sensitive to by construction.

c. **Applicability**: transfers directly as a DIAGNOSTIC technique — not
vehicle-class dependent, though this car's specific component set (ARB
blades, tire pressures) would need its own baseline range to detect
against.

d. **Cross-check**: NOT a setup-change → behaviour-tendency bridge at all —
these are fault/anomaly detections, not deliberate lever adjustments. They
do not fit `interaction_table` (which encodes intentional setup deltas) or
`lever_bridges` (same). No existing code does anything like this — no
roll-angle/roll-ratio math channel exists anywhere in modules/ (confirmed:
zero hits for "roll_ratio"/"roll_gradient"/"roll_angle" in modules/).

e. **VERDICT: INSIGHT-ONLY, out of this package's scope by construction**
(scope rule 2 restricts candidates to lever_bridge/interaction_table/
annotation targets; a fault-detection evidence source is none of those).
Recorded for the future-feature list below rather than dropped silently.

f. No lever_bridges/interaction_table target — would require a new
evidence-source function (roll-ratio anomaly/trend detection), out of
scope for this document.

### C9-5 — Pitch gradient / anti-dive-antisquat (9.7): no adoptable lever

a. **Statement** (p.219-220): a pitch gradient (pitch angle per unit
longitudinal G) can be calculated analogously to roll gradient; its
linearity is affected by anti-squat/anti-dive suspension geometry.

b. **Mechanism**: anti-squat/anti-dive geometry couples longitudinal load
transfer partially through the suspension links rather than purely through
spring/damper compression, changing how much the chassis pitches under
braking/acceleration for a given weight transfer.

c. **Applicability**: mechanism is real and car-class-independent, but see
(d) — this project has no way to act on it.

d. **Cross-check**: NOT ENCODABLE, checked directly rather than assumed.
config/setup_parameters.json's `kinematic_variants` entry (the only
registry item that plausibly covers anti-dive/anti-squat, confirmed present
in config/car_data.json's `kinematic_variants_front` digitised Porsche
homologation data) is explicitly marked `"category": "context"` and
`"recommendation_target": false` — background information for interpreting
other channels, not a session-level tunable this decision frame is allowed
to suggest changing (mechanism: "Homologated suspension-geometry variants
... background context ... not a session-level tunable", config's own
words).

e. **VERDICT: REJECT — no lever exists, confirmed by direct registry read,
not assumed.** The mechanism is real but there is nothing in scope for this
project to recommend changing; a kinematic-variant swap is a build-spec
decision, not a setup adjustment this tool advises on.

f. No encoding target.

### Chapter 9 census: 0 new candidates (2 confirmatory overlaps with existing shipped bridges, 2 registry-gap insight-only, 1 out-of-scope diagnostic insight-only), 0 ADOPT/ADAPT, 0 contradictions.

---

## Chapter 10 — Wheel Loads and Weight Transfer (pp.221-256)

The other existing anchor chapter (modules/wheel_loads.py, damper package
Phases 1-6, 2026-09-03). Sections 10.1-10.3 (lateral/longitudinal weight
transfer, banking/slope) are pure measurement mathematics, already
implemented in modules.stability_analysis.estimate_vertical_loads (the
axle-total weight+longitudinal-transfer+aero-share formula the FR
reconstruction itself cites) — confirmed by direct code reference in
thesis_notes.md's own "ITEM 2 -- FR RECONSTRUCTION" entry, not re-derived
here. Sections 10.4-10.8 yield one genuinely new candidate (chassis
torsion) plus confirmatory/gap findings.

### C10-1 — Bump rubbers as ride-height/platform control on aero cars (confirms existing documented limitation)

a. **Statement** (p.236): "Bump rubbers are used not only to prevent the
shock absorbers from bottoming out but also to control the ride height,
especially in cars where aerodynamics are of vital importance. In this
case the bump rubber is an integral part of the suspension and will have
a significant effect on the wheel loads."

b. **Mechanism**: in aero-heavy cars the bump rubber's progressive spring
rate is deliberately engaged under sustained aero load to set a secondary,
stiffer effective rate near the ride-height limit, controlling platform
attitude rather than only cushioning transient hits.

c. **Applicability**: HIGH — this project's own car carries real, measured
aero load (mu-load-sensitivity finding, e8101ad; 25/75 measured aero
split) — exactly the class of car the book flags as needing this modelled.

d. **Cross-check**: CONFIRMS an already-documented limitation rather than
revealing a new one. config/parameters.json's own `bump_rubber_note`
(wheel_loads estimator) already states: "NEGLECTED (documented limitation,
not modelled)... a wheel deep enough into bump to contact its bump rubber
will read an UNDER-estimated load," and separately notes
config/car_data.json's own `bump_stop_curve` (`cellasto_20mm_40shore`)
exists but is unconsumed, suggesting the same future upgrade path
(`log_susp_travel_*` proximity-to-engagement flagging) the book's own
framing would suggest. This chapter is independent literature confirmation
that the limitation is real and matters most for exactly this car's class
— not a new finding, but it gives the existing documented-limitation entry
a source anchor it didn't have before.

e. **VERDICT: INSIGHT-ONLY — confirms an existing limitation.** Bump-rubber
engagement is not an independent adjustable lever in the registry (it's a
fixed physical part per `bump_stop_curve`, not a `setup_parameters.json`
key), so there is no new bridge to add.

f. No encoding target — attaches to the existing `bump_rubber_note` in
config/parameters.json as a literature confirmation, not a new row.

### C10-2 — Full 4-corner modal reconstruction is possible from travel + wheel-rate asymmetry alone (capability gap vs. shipped reconstruction)

a. **Statement** (p.240-249, Eq.10.26-10.34): all four wheel loads can be
derived purely from the four suspension travel signals decomposed into
heave/pitch/roll/warp modes, combined with each mode's own wheel rate
(from spring/ARB rates and motion ratios) — no direct force/load-cell
measurement required, given known motion ratios, spring rates, and ARB
rates, INCLUDING the front/rear roll and warp rate ASYMMETRY (Eq.10.32-10.34).

b. **Mechanism**: the four travel channels are a complete, invertible basis
for the four independent suspension DOFs; each DOF's force response is
linear in its own modal stiffness, so wheel loads follow from travel +
known rates alone, without needing a working force sensor on any corner.

c. **Applicability**: transfers directly — pure kinematics/statics,
vehicle-class-independent.

d. **Cross-check**: SUBSTANTIAL OVERLAP but MORE CAPABLE than what is
shipped. `modules.wheel_loads.reconstruct_missing_corner` (damper package
Item 2, 2026-09-03) already implements a SIMPLIFIED version — heave+pitch
(axle-total) only, explicitly not modelling the roll/warp split — and
requires one REAL corner per axle to close the equation
(axle_total − real_mate). That was a DELIBERATE choice, recorded at the
time ("no roll/ARB model is needed for the split once a real mate
measurement exists... strictly worse once a real measurement exists on
that axle" — thesis_notes.md, ITEM 2), not an oversight. This chapter shows
the full asymmetry-modelled path (Eq.10.32-10.34) IS tractable — spring
rates, ARB rates, and motion ratios all already exist in
car_data.json/setup_parameters.json — for the specific case the existing
code identifies as its own weakest fallback tier: BOTH corners of an axle
damper-invalid at once, which currently falls straight through to the
plain static-split (Level 1, no real sensor at all), skipping any
modal/kinematic reconstruction entirely.

e. **VERDICT: INSIGHT-ONLY — a real, literature-anchored capability gap
named, not proposed as an implementation.** Not a lever_bridge/
interaction_table candidate (a reconstruction-methodology upgrade, not a
setup→behaviour claim). Flagged for the future-feature list below.

f. No interaction_table/lever_bridges target. Future note for
modules/wheel_loads.py's own reconstruction cascade: a full-modal fallback
using Eq.10.26-10.34 could raise the both-corners-invalid case above plain
static-split, still without needing any working force sensor on that axle.

### C10-3 — Chassis torsional stiffness caps the real-world effectiveness of roll-stiffness-distribution changes (NEW)

a. **Statement** (p.255-256, worked example): "The nonlinear behavior of the
race car tire provides the means of tuning the vehicle's handling balance
by changing the amount of weight transfer on one axle... HOWEVER, the car
handling can be influenced only in this way if the chassis serves as a
platform to feed the involved torques through." Worked example: measured
chassis torsional stiffness (133.4 kg/mm) found comparable in magnitude to
that car's own front roll stiffness — "do not ignore the torsion stiffness
of the chassis" in that case.

b. **Mechanism**: a springs/ARB change intended to shift roll-stiffness
DISTRIBUTION between axles only fully delivers if torque is transmitted
rigidly through the chassis structure; a torsionally soft chassis absorbs
part of the intended shift as chassis twist (the "warp" mode of section
10.5) instead of delivering it to the other axle — silently reducing the
real-world magnitude of any springs/ARB balance change below a
rigid-chassis calculation's prediction.

c. **Applicability**: mechanism transfers, magnitude does not. This
project's GT3R uses a substantially stiffer chassis than the book's own
worked (non-GT3) example car is likely to, so the effect size here is
probably smaller — but not eliminated, and nothing in this project
measures this car's actual chassis torsional stiffness to size it
(confirmed: no `chassis_torsion`/`torsion` key anywhere in
config/car_data.json or config/setup_parameters.json). A caveat that can be
stated but not quantified.

d. **Cross-check**: DIRECTLY touches the ALREADY-SHIPPED `springs_front/rear`
and `arb_fl/fr/rl/rr` bridges (both `interaction_table` and `lever_bridges`),
all of which implicitly assume a rigid chassis — the standard assumption
Chapter 9's own roll-gradient derivation states explicitly ("when comparing
setups within one vehicle, the chassis spring rate can be ignored because
normally it is a parameter... that does not change" — true for
comparability across setups, silent on absolute effectiveness, which is
exactly what this chapter's own section adds). No contradiction with those
bridges' DIRECTION (front-soften-helps-understeer stays correctly signed),
only an unstated magnitude caveat. Also connects directly to the
ALREADY-DOCUMENTED warp/torsion-mode-unobservable limitation in
`modules.wheel_loads.reconstruct_missing_corner` (2026-09-03, "the warp/
torsion mode is unobservable with three sensors and a heave/pitch-only
model") — same physical DOF, two different contexts: there it means we
can't MEASURE the mode, here it means unmeasured chassis compliance can
silently CAP how much a recommended springs/ARB change actually achieves.
Worth cross-referencing the two notes.

e. **VERDICT: ADOPT, as an ANNOTATION on the existing `springs_front/rear`
and `arb_fl/fr/rl/rr` bridges** (magnitude caveat, not a new bridge or a new
lever — chassis torsional stiffness is not adjustable hardware on this car
and is not a registry parameter). No magnitude is encodable: the book gives
none beyond its own non-GT3 worked example, and this project has no
measurement of its own car's torsional stiffness.

f. **Encoding target**: an annotation attached to the shipped
`springs_front/rear` and `arb_*` `lever_bridges`/`interaction_table` entries'
rationale text, e.g.: "magnitude assumes a rigid chassis (standard ch.9/10
assumption); real effect may be smaller if chassis torsional compliance
absorbs part of the intended roll-stiffness redistribution — this car's
torsional stiffness is not measured (config/car_data.json has no
chassis-torsion entry)." Not a new config row; a documentation addition to
existing ones.

### C10-4 — Tire spring rate (10.7): same registry gap as C9-3

a. **Statement** (p.253-254, Eq.10.38): tire spring rate (set by pressure/
construction) works in series with the suspension spring and in parallel
with the ARB, contributing to the true roll-stiffness distribution;
ignoring it (as a pure suspension-potentiometer method does) misses part of
the real distribution.

b-c. Same mechanism and applicability as C9-3.

d. **Cross-check**: same registry gap already named at C9-3 — no
`tire_pressure` (or equivalent) key exists in config/setup_parameters.json.

e. **VERDICT: REJECT — duplicate of C9-3's registry-gap finding.** Not
re-recorded as a separate candidate; both citations (ch.9 Table 9.6 and
ch.10 Eq.10.38) point to the same gap, worth keeping as a two-citation note
under C9-3 rather than two separate entries.

f. See C9-3.

### C10-5 — Track banking/slope corrupts vertical-load estimates on non-flat circuits (not a car lever)

a. **Statement** (p.231-234, Eq.10.16-10.18): on a banked or sloped track,
accelerometer-measured lateral/vertical acceleration is not the same as
true cornering acceleration — measured Glat must be corrected by the
(otherwise unknown) banking/slope angle before use in the weight-transfer
equations, or calculated wheel loads will be systematically biased.

b. **Mechanism**: gravity has a track-surface-relative component on banked/
sloped sections the accelerometer cannot separate from true lateral/
longitudinal G without independent track-geometry data.

c. **Applicability**: matters only on meaningfully banked/sloped circuits;
not evaluated here for either real session used by this project.

d. **Cross-check**: NOT MODELLED anywhere in modules/stability_analysis.py
or modules/wheel_loads.py — confirmed directly (no banking/slope/grade term
in either module; the only "slope" hits in stability_analysis.py are
unrelated tire force-vs-slip-angle slope calculations, checked to rule out
a false read). Not a contradiction, simply unaddressed.

e. **VERDICT: INSIGHT-ONLY — not a car setup lever at all (a TRACK
property), out of this package's scope by definition** (scope rule 1
restricts candidates to registry hardware levers). Recorded as a
data-quality caveat for the vertical-load estimator generally, worth a
one-line limitation note if the team ever races a meaningfully banked
circuit.

f. No encoding target.

### Chapter 10 census: 1 new candidate (C10-3, ADOPT as annotation), 2 confirmatory/overlap findings (C10-1, C10-2 — both feed the future-feature list), 1 duplicate registry-gap (C10-4, folded into C9-3), 1 out-of-scope insight (C10-5, track not car property). 0 REJECT-as-wrong, 0 contradictions.

---

## Chapter 8 — Understanding Tire Performance (pp.169-198)

Channel census performed FIRST, directly against the raw files, per the
standing channel-census rule — this chapter's applicability hinges entirely
on what tire sensors this car actually carries, which cannot be assumed
from the book's own generic TPMS/IR-sensor description. config/channels.json
has zero tire-pressure or tire-temperature entries. Direct grep of the two
real session files finds a real, if partial, picture: Sample_Dubai.txt
carries a full raw TPMS block, including live `tpms_press_fl/fr/rl/rr[bar]`
and `tpms_temp_fl/fr/rl/rr[°C]` channels (alongside a large diagnostic/alarm
sub-block — timeouts, ECU IDs, `alarm_tpms_hard/soft` — that is clearly not
payload data), none of it registered in config/channels.json.
GT3_Creventic_PRC_MLA.txt (the "v3" damper-channel session) carries ZERO
TPMS-related channel names anywhere (grep count 0) — not logged on that
outing at all. Neither file carries any 3-zone (inside/middle/outside)
infrared tread-temperature channel — no `shoulder`/`inner`/`outer`/`middle`
per-wheel pattern found in either file. This session-specific split (TPMS:
Dubai only; IR tread sensing: neither session) governs every candidate
below and is checked directly rather than assumed, per the standing rule
that a work order's own premise about the data is exactly the kind of claim
that needs checking, not carried over from one file to another.

### C8-1 — TPMS pressure/temperature channels exist raw (Dubai only) but are unregistered

a. **Statement** (p.175-179, Eq.8.1/8.2): a TPMS provides per-wheel tire
pressure and internal air temperature, used for puncture detection, cold-
pressure management, and (via the Ideal Gas Law) normalizing hot-pressure
readings logged at different temperatures to a common reference
temperature for fair comparison.

b. **Mechanism**: tire internal air pressure and temperature are physically
coupled (ideal gas law); raw hot-pressure readings taken at different
temperatures are not directly comparable without this correction.

c. **Applicability**: directly relevant where the channels exist — verified
against this project's own raw files rather than assumed from the book's
generic description (see chapter preamble above).

d. **Cross-check**: NOT REGISTERED anywhere in config/channels.json,
confirmed by direct read. The underlying data exists in Sample_Dubai.txt's
raw log (`tpms_press_fl/fr/rl/rr[bar]`, `tpms_temp_fl/fr/rl/rr[°C]`) and is
currently unused by this project. Not present at all on the v3 session —
a genuine session-availability difference, not a project-wide gap.

e. **VERDICT: INSIGHT-ONLY as a lever_bridge candidate** (a sensor channel,
not a setup lever — cannot itself target `interaction_table`/`lever_bridges`),
**but a concrete, low-effort, book-anchored channel-registry gap**, distinct
from the tire-pressure LEVER gap already named at C9-3/C10-4 (there is no
tire-pressure adjustment parameter in the registry either way — TPMS tells
you what pressure IS, not a way to set it, so registering these channels
does not by itself resolve that lever gap).

f. No interaction_table/lever_bridges target. Future-feature note:
registering these 8 channels (Dubai-only) in config/channels.json would be
the prerequisite for C8-2 below.

### C8-2 — Axle-average and L/R tire-temperature-difference balance diagnosis is achievable WITHOUT 3-zone IR sensing (8.5, 8.6)

a. **Statement**: working temperature range (8.5, |lateral G| vs. a SINGLE
per-wheel tire temperature, front/rear) and lateral-load-transfer balance
diagnosis via left/right temperature difference (8.6, Eq.8.4/8.5: ΔT_front
= T_LF − T_RF, ΔT_rear = T_LR − T_RR; a worked example, p.191-192, reads
"understeer when front difference exceeds rear, oversteer when rear
exceeds front") both use only ONE temperature reading per wheel — no
inside/outside/center zone breakdown required, unlike 8.7-8.9 below.

b. **Mechanism**: more lateral load transfer on an axle produces a bigger
inside/outside grip (and therefore heat-generation, per Eq.8.3's Fy²·V²
term) split; the L/R temperature difference is a proxy for that axle's
realized lateral load transfer, and a persistently out-of-range axle
average identifies which axle is running cold or hot relative to its own
grip-optimal window.

c. **Applicability**: mechanism transfers, with one honest fidelity caveat:
this car's TPMS measures internal air temperature, not tread surface
temperature — a thermally damped, lagged proxy for what the book's IR
sensors measure directly (heat generates at the tread first, then conducts
into the carcass/air). This should preserve the qualitative trend (which
side/axle runs hotter) with reduced amplitude and a time lag, not a
different sign — a caveat on sensitivity, not on direction.

d. **Cross-check**: NOT BUILT. No temperature-based evidence source exists
anywhere in modules/ (confirmed: the channels this would consume, per C8-1,
aren't even registered yet, so nothing downstream could exist). Would be a
genuinely NEW evidence source, not overlapping with anything existing —
closest existing evidence is `matrix_verdict`/corner-verdict, built from
yaw/slip-angle classification, not thermal.

e. **VERDICT: INSIGHT-ONLY for this package** — a Tier B evidence-source
proposal, not itself an `interaction_table`/`lever_bridges` row. Would (if
ever built) CORROBORATE the existing `springs_front/rear` and `arb_*`
understeer/oversteer bridges with an independent thermal signal, similar in
spirit to how `ls_threshold` evidence was added to sharpen existing
verdicts rather than replace them. Recorded for the future-feature list,
Dubai-only, gated behind C8-1.

f. No interaction_table/lever_bridges target directly. Future evidence-
source note: a `_build_tire_temp_evidence` function (axle-average
range-membership + L/R difference sign) could feed the SAME
`understeer_tendency`/`oversteer_tendency` axes the matrix/corner-verdict
evidence already scores against.

### C8-3 — Camber and pressure evaluation via 3-zone IR tread temperature: REJECT, no hardware

a. **Statement**: 8.8 (camber, via inside/outside shoulder temperature
spread — directly targets `camber_fl/fr/rl/rr`, a REAL registry lever) and
8.9 (pressure, via center-vs-shoulder-average temperature) both require
three temperature zones per tire.

b. **Mechanism**: camber sets which shoulder carries more load/slip and
therefore heats more; under/over-inflation changes the contact-patch
pressure distribution between center and shoulders, changing which zone
runs hottest.

c. **Applicability**: mechanism transfers directly, and 8.8 specifically
targets a real lever this project has — the most directly actionable
content in this entire chapter, IF the sensor existed.

d. **Cross-check**: NO HARDWARE, confirmed directly. No
inside/outside/center/shoulder-per-wheel channel pattern exists anywhere in
Sample_Dubai.txt's raw header or in config/channels.json. TPMS provides
only ONE bulk temperature value per wheel, not a 3-zone tread breakdown —
cannot support this method even as a degraded proxy, unlike C8-2's L/R
methods above.

e. **VERDICT: REJECT** — no `lever_bridge`/`interaction_table` candidate is
possible without new physical instrumentation (3-zone IR tire temperature
sensors), which is outside this project's scope to propose (a hardware
fitment decision, not a setup or software change).

f. No encoding target. Named because it is the chapter's single most direct
hit on a real registry lever (camber), blocked purely by instrumentation
absence — worth stating plainly for the record rather than silently
omitted: the book gives an exact, book-anchored method for `camber_fl/fr/
rl/rr`, this project simply doesn't have the sensor to feed it.

### C8-4 — Gated combined-acceleration "grip factor" statistics (8.1): future KPI, not a lever bridge

a. **Statement**: overall/cornering/braking/traction/aero "grip factor"
computed by gating the combined-acceleration (traction-circle radius)
channel to grip-limited-only samples, tracked lap-by-lap to monitor tire
degradation, setup changes, or weather.

b. **Mechanism**: combined-acceleration magnitude, restricted to phases
where the car is genuinely grip-limited (excluding power/drag-limited
straight-line segments), is a direct proxy for how much of the tire's
available grip is realized.

c. **Applicability**: transfers directly; gating thresholds are car-specific
by the book's own admission, not universal constants.

d. **Cross-check**: no existing evidence source computes anything like this
(confirmed: no gated-combined-acceleration or "grip factor" statistic
anywhere in modules/).

e. **VERDICT: INSIGHT-ONLY** — a monitoring/KPI tool (Tier C/B), not a
setup-change→behaviour bridge (no lever is being adjusted; it's a
performance-tracking statistic). Recorded for the future-feature list.

f. No encoding target.

### Chapter 8 census: 0 lever_bridge/interaction_table candidates (0 ADOPT/ADAPT), 1 REJECT (C8-3, real lever but no hardware), 3 INSIGHT-ONLY feeding the future-feature list (C8-1, C8-2, C8-4). The chapter's single highest-value hit (camber via tire temperature) is blocked purely by instrumentation this car does not carry, confirmed by direct census rather than assumed.

---

## Chapter 13 — Aerodynamics (pp.321-352)

This is where the book's own material goes into direct dialogue with this
project's own measured findings, as flagged before reading: the 25/75
measured front/rear aero split (config/parameters.json `aero_front_fraction`,
Metrology Phase 3, e8101ad) and its own v²-functional-form fit. The chapter
yields one substantial cross-validation finding (C13-2), one real,
previously-unflagged registry gap (C13-6, wing/splitter have zero decision-
frame coverage of any kind), and several corroboration/limitation notes for
the existing aero work.

### C13-1 — Ride-height sensitivity of aerodynamic balance: a second, independent anchor for the existing ride_height platform_stability entries

a. **Statement** (p.322, opening section; developed throughout 13.4-13.8):
"The longitudinal location of the center of pressure represents the
downforce distribution between the front and rear axles (i.e., the
aerodynamic balance). This balance can be very sensitive to changes in
ride height." The chapter's entire aeromap methodology (13.4-13.8) exists
specifically because CL, CD, and center-of-pressure location are all
ride-height-dependent quantities.

b. **Mechanism**: front and rear ride height set the underside/floor
geometry (splitter and diffuser proximity to the ground, rake angle); this
changes the local airflow and therefore the aerodynamic COEFFICIENTS
themselves (not just how much load transfer a given downforce produces) —
a mechanistically distinct path from the roll-stiffness/rake-holding
mechanism (ch.9/10, ch.11 C11-5) already used to anchor the existing
`ride_height_front/rear` `platform_stability` entries.

c. **Applicability**: transfers directly to any ground-effect-reliant GT
car, this one included.

d. **Cross-check**: CORROBORATES, does not duplicate, the already-shipped
`ride_height_front/rear` `platform_stability` entries (literature-bridge
coverage-check package, 2026-09-20, currently anchored only to ch.9/10's
roll-stiffness/rake mechanism). This chapter adds a SECOND, mechanistically
independent reason the same lever→axis link is real. It does NOT license a
new signed `understeer_tendency`/`oversteer_tendency` entry for ride
height's aero effect specifically — the DIRECTION of any such balance
shift is floor/rake-design-dependent and the book gives no generic sign;
this project has no ride-height-sweep aeromap of its own to determine one
(see C13-2 — no dedicated aero test has been run).

e. **VERDICT: ADOPT as an additional citation** on the existing
`ride_height_front/rear` `platform_stability` entries' rationale text — not
a new row. **REJECT** encoding any new signed understeer/oversteer entry
for ride height's aero effect — no data exists to size or sign it.

f. Encoding target: append to the existing entries' `derived_from`/rationale
text, e.g. "additionally: Segers ch.13 (aero coefficients themselves are
ride-height-dependent, distinct from the ch.9/10 roll-stiffness mechanism)."
Not a new config row.

### C13-2 — Constant-velocity test method for front/rear downforce split: an independent, literature-standard cross-check for the measured 25/75 aero split (star finding)

a. **Statement** (p.339-341, 13.7; worked GT3 example pp.343-348, 13.8):
downforce and its front/rear distribution can be measured by comparing
axle loads (from suspension-force-from-travel, Eq.13.18/13.19, or direct
load cells) at a near-zero "static" reference speed against a constant
higher speed; the DIFFERENCE is pure aerodynamic downforce per axle,
Aero% = LF/(LF+LR) (Eq.13.20). Worked GT3 example: 30.9%-38% front,
depending on the specific run. Explicitly requires a controlled test
protocol: springs/damping as soft as possible, bump rubbers uninvolved,
ARBs preferably disconnected, steady constant speed (to remove inertial
transients), runs repeated in both directions (to cancel wind).

b. **Mechanism**: at a low enough speed, dynamic pressure (∝V², Eq.13.7) is
negligible and measured axle load is pure static weight; at a held higher
speed, the additional load above that baseline is, by construction, purely
aerodynamic (no roll/pitch/longitudinal-transfer contamination if truly
steady-state) — a direct differencing measurement, not a regression fit.

c. **Applicability**: HIGH, and this is exactly the open question this
project has already named. config/parameters.json's own `aero_front_
fraction_note` (0.25, Metrology Phase 3) explicitly states this figure is
"single-car, two-session telemetry evidence only, not a substitute for a
real aero-balance figure" and remains on the "Engineer follow-up questions"
list for confirmation "against real windtunnel/CFD data." This chapter
offers a THIRD path, neither windtunnel/CFD nor the existing opportunistic
regression: a literature-standard, on-track, no-windtunnel-required test
protocol using data channels this project already logs (suspension travel,
spring rates, speed) — the book's own method needs no more instrumentation
than this project already has on v3.

d. **Cross-check**: COMPLEMENTARY, not contradictory, to the existing
method. Our shipped `aero_front_fraction` (0.25) comes from fitting
`residual_N = a + b·v²` per corner on straight-line RACING samples (damper-
derived Fz, after subtracting weight + longitudinal transfer), cross-
session-confirmed (Dubai RL b=0.611 vs v3 RL b=0.604, independently fit).
The book's method is a controlled STEADY-STATE TEST (soft springs, ARB
disconnected, two-direction runs) this project has never run — no such
dedicated aero test appears anywhere in thesis_notes.md. The two methods
measure the same physical quantity through different means: the existing
one is opportunistic (uses racing data already in hand, more assumptions —
v² functional form, no roll contamination — already independently checked,
per the Metrology Phase 3 entry's own "structural check performed before
trusting this at all") while the book's is a dedicated, lower-assumption
protocol (near-zero contamination by design) that has simply never been
executed on this car.

e. **VERDICT: ADOPT as a proposed VALIDATION/TEST-PROTOCOL, not a
lever_bridge or interaction_table candidate** (no setup lever is being
changed; this is a measurement method for an existing Level-1-adjacent
estimate, aimed at the accuracy-level system, not the recommendation
engine). Two forms named, not conflated: (1) a cheap, approximate
retrospective check — scan existing racing telemetry for any segments
close to steady-state at two distinct speeds and apply Eq.13.18-13.20
opportunistically (imperfect: racing setups won't have soft springs/
disconnected ARB, so this only approximates the book's controlled
precondition); (2) the real prize — a dedicated straight-line test session
(coast-down + constant-velocity runs, both directions) that would upgrade
`aero_front_fraction` from its current opportunistic-fit provenance to a
genuine Level 2 (session measurement, book-standard protocol).

f. **Encoding target**: not `interaction_table`/`lever_bridges`. Proposed
as a new line item on PLAN.md's "Engineer follow-up questions — damper/
wheel-load package" list (already the home for this exact open question) —
citing Segers ch.13.7-13.8 as the protocol to run, not proposing to run it
in this document.

### C13-3 — Dynamic pressure ∝ V²: first-principles confirmation of the existing fit's functional form

a. **Statement** (p.326, Eq.13.7): dynamic pressure q = ½ρV² — the physical
basis for every aerodynamic-force equation in the chapter.

b. **Mechanism**: standard fluid-dynamics result, not specific to racing.

c. **Applicability**: universal.

d. **Cross-check**: CONFIRMS, does not extend, an existing project choice.
The v²-residual fit underlying `aero_front_fraction` (`residual_N = a +
b·v²`) is exactly this functional form — previously justified only
empirically (R²=0.56-0.91 per corner, Metrology Phase 3). This chapter
gives that functional-form choice a first-principles anchor it didn't
explicitly cite before.

e. **VERDICT: ADOPT as a citation strengthening an existing method choice**,
not a new candidate.

f. Encoding target: a citation addition to `aero_front_fraction_note`
(config/parameters.json) or its thesis_notes.md source entry — e.g. "the
v² functional form is not an arbitrary curve-fit choice; it is the
first-principles dynamic-pressure relationship (Segers ch.13, Eq.13.7)."

### C13-4 — Air density variation: an unstated cross-session confound

a. **Statement** (p.323-325, Eq.13.3-13.6): aerodynamic force at a given CL·A
scales with air density, itself a function of temperature, pressure, and
humidity — "race car engineers place a high priority on understanding the
impact of changing weather conditions" specifically because comparisons
across sessions with different atmospheric conditions can be misread as
setup or aero differences.

b. **Mechanism**: force = ½ρV²·C·A — for fixed geometry (C·A), measured
force still varies with ρ, which varies session-to-session with weather.

c. **Applicability**: directly relevant to any cross-session comparison of
aero-derived quantities.

d. **Cross-check**: NOT CONTROLLED FOR in the existing cross-session
confirmation (Dubai RL b=0.611 vs v3 RL b=0.604). Not a flaw — the match
being that close DESPITE this uncontrolled variable is, if anything, mild
additional reassurance — but it is a genuinely unstated assumption, not
previously named in the Metrology Phase 3 entry.

e. **VERDICT: INSIGHT-ONLY — a documentation gap, not a re-fit request.**
Worth naming in the existing `aero_front_fraction_note` as an unquantified
source of the fit's residual scatter, not something this package proposes
correcting (would require ambient weather channels this project has not
censused for either session).

f. No interaction_table/lever_bridges target. A documentation addition to
`aero_front_fraction_note`.

### C13-5 — Wind as an unquantified confound in the existing aero fit

a. **Statement** (p.326-327, p.337, p.347-348): headwind/tailwind biases
measured dynamic pressure and therefore any speed-derived aero estimate;
the book's own protocol runs every aero test in BOTH directions specifically
to cancel this out (worked example: a 10 km/h wind inferred and separated
from the true CLA by comparing two opposite-direction runs).

b. **Mechanism**: measured dynamic pressure depends on airspeed relative to
the car, not ground speed; wind adds or subtracts from that relative speed.

c. **Applicability**: directly relevant to any track-session aero estimate,
this project's included.

d. **Cross-check**: NOT CONTROLLED FOR. The existing `aero_front_fraction`
fit uses straight-line samples from real racing sessions — single-direction
by construction (a race track is driven one way), so wind cannot be
cancelled the way the book's dedicated two-direction protocol does. This is
a genuinely new limitation, not previously named alongside the existing
"single-car, two-session telemetry evidence only" caveat.

e. **VERDICT: INSIGHT-ONLY — a limitation named, not correctable from
existing data.** A racing session cannot retroactively gain a return run on
the same straight in the opposite direction; this stays a stated limitation
of the opportunistic method, reinforcing (with a concrete, book-anchored
mechanism) why C13-2's dedicated test protocol would be worth running.

f. No interaction_table/lever_bridges target. A documentation addition to
`aero_front_fraction_note`.

### C13-6 — wing_position and splitter_offset have ZERO decision-frame coverage of any kind (new finding, no existing axis fits cleanly)

a. **Statement** (p.334-337, coast-down worked example): comparing minimum
and maximum rear wing angle via coast-down CDA measurement on the same car
showed an 18% increase in total vehicle drag (CDA 1.075 → 1.268, rolling
resistance corrected) — a directly measured, book-worked, wing-angle-to-
drag relationship.

b. **Mechanism**: increasing wing angle increases both CL and CD (Eq.13.1/
13.2) — more downforce for more drag, the fundamental wing trade-off.

c. **Applicability**: transfers directly (any wing produces this trade-off);
the specific 18% MAGNITUDE is that car's own number, not transferable to
this GT3R.

d. **Cross-check, checked directly rather than assumed**: `wing_position`
and `splitter_offset` are BOTH real registry levers (config/
setup_parameters.json) with ZERO entries anywhere in `interaction_table` and
ZERO entries in `lever_bridges` — confirmed by direct query, not carried
over from the earlier coverage-check package (which named ARB/camber/diff_
position/ride_height/tc_lon/toe_front as matrix-covered but never
mentioned wing_position or splitter_offset at all — this is a genuinely new
gap, not a re-statement of a known one).

e. **VERDICT: INSIGHT-ONLY — real gap, but no encodable target under the
CURRENT axis vocabulary.** The book's own statement here is about DRAG
(top speed / straight-line performance), and none of the six existing axes
(`understeer_tendency`, `oversteer_tendency`, `yaw_stability`,
`braking_performance`, `traction_performance`, `platform_stability`) is a
clean fit — `traction_performance` is about exit-phase contact-patch grip,
not straight-line drag/top-speed trade-off. Forcing this into
`traction_performance` would misrepresent what the book actually measured.
This chapter does NOT state a cornering-balance direction for wing angle at
all (unlike, say, the ch.9/10 roll-stiffness statements) — inventing one
(e.g. "more rear wing counters oversteer") would be exactly the kind of
unanchored claim the Tier A rule forbids, so none is proposed here.

f. No encoding target under the current schema. Named as a genuine gap for
a future package: if a `drag_performance`/`straight_line_performance` axis
is ever added to the vocabulary, this chapter's coast-down method (C13-2's
own protocol, section 13.6) is the anchor to use for a `wing_position`
`interaction_table` entry. Not decided here — a vocabulary extension is
outside this document's scope.

### Chapter 13 census: 0 lever_bridge/interaction_table rows added, 2 citation-strengthening additions to already-shipped entries (C13-1, C13-3), 1 proposed validation/test-protocol for an existing Level-1-adjacent measurement (C13-2 — the chapter's highest-value finding), 2 named-but-uncorrected limitations of that same existing measurement (C13-4 air density, C13-5 wind), 1 genuine new registry-coverage gap named without a clean axis to encode it under (C13-6, wing/splitter). 0 REJECT-as-wrong, 0 contradictions with existing bridges.

---

## Chapter 5 — Braking (pp.101-122)

Channel census performed first: Sample_Dubai.txt carries NO brake line
pressure channels and NO pedal travel channel anywhere (grep for `brk`/
`brake` combined with `press`/`pressure`/`travel`/`pedal`, zero hits,
excluding alarm/diagnostic sub-channels) — the book's PRIMARY brake-balance
method (5.5, brake line pressure ratio) cannot be built on this car's
logged data at all. Two things DO exist and are real, checked directly: per-
corner brake disc temperature (`log_brkdisctemp_fl/fr/rl/rr[°C]`, raw,
UNREGISTERED in config/channels.json) and per-corner wheel speed
(`log_speed_fl/fr/rl/rr[kph]`, ALREADY REGISTERED, plus a raw
`log_speed_*_lock[s]` sub-channel per corner whose exact semantics were not
decoded in this read-only pass). Also checked directly against
config/recommendations.json (the 39-rule matrix base, not just
interaction_table/lever_bridges): the engineer-verbatim braking-phase cells
(US-BRK-low/med/high, OS-BRK-med/high, INST-BRK-low/med/high) use `toe_front`,
`arb_*`, `damper_bump_*`, `wing_position`/`ride_height_*`, and `abs_position`
— never `brake_bias`, despite `brake_bias` being a real, currently
zero-coverage registry lever.

### C5-1 — Brake bias directly sets corner-entry/mid-corner understeer vs. corner-entry oversteer (star finding — real lever, zero coverage, adjacent to engineer-verbatim cells)

a. **Statement** (p.107): "Corner-entry understeer followed by mid-corner
understeer and the combination of low braking Gs can be diagnosed as too
much front brake bias. Too much rear bias leads to corner-entry oversteer
if not anticipated by the driver."

b. **Mechanism**: brake bias sets the front/rear split of brake-line
pressure (Eq.5.3), which sets how close each axle runs to its own peak
braking-friction slip ratio; too much front bias uses up front-tyre grip
capacity under straight-line braking that is then unavailable for turn-in
cornering force (understeer as the driver turns in), while too much rear
bias risks the rear stepping out under trail-braking (oversteer) — a
direct, textbook-standard mechanism, not this car's own invention.

c. **Applicability**: transfers directly — `brake_bias` is a real
registry lever on this GT3R (config/setup_parameters.json), and the
`entry_1_brake`/`entry_2_turnin` phase groups this project's own decision
frame already uses are an exact match for "corner-entry"/"mid-corner
following entry."

d. **Cross-check, the reason this is flagged rather than adopted outright**:
`brake_bias` carries ZERO entries in `interaction_table` and ZERO entries in
`lever_bridges`, confirmed by direct query. But it is NOT simply
"uncovered" the way springs_front/rear were before BACKLOG item H — the
SAME scenario (braking-phase understeer/oversteer/instability, by speed
class) is already served by seven ENGINEER-VERBATIM or project-lead-reviewed
matrix cells (US-BRK-low/med/high, OS-BRK-med/high, INST-BRK-low/med/high),
all of which reach for a DIFFERENT lever each time — front toe, front ARB
softening + rear ARB stiffening, front LS/HS bump damping, wing+rake
package, ABS map position — and NONE of them ever suggests adjusting
`brake_bias` itself. CLAUDE.md's own standing rule: "Contradictions with
engineer-verbatim bridges are flagged, never overridden." This is not a
direct contradiction (no cell recommends the OPPOSITE brake_bias direction)
— it is an ABSENCE across seven independent, real elicited decisions for
exactly this scenario, which is itself worth surfacing rather than silently
filled in: either the omission is deliberate (brake bias is commonly left
alone mid-event for reliability/consistency reasons not recorded anywhere
in this config), or it is a genuine gap the engineer simply hasn't been
asked about yet.

e. **VERDICT: ADAPT, flagged for engineer confirmation before encoding —
not silently ADOPTED.** The book's mechanism and phase mapping are sound
and directly anchored; what is NOT resolved by literature alone is whether
`brake_bias` should join the existing braking-phase toolkit as an additional
PROPOSED-grade option (via the same generic `lever_bridges` mechanism
BACKLOG item H already built) or whether its absence from seven
engineer-verbatim decisions reflects a real-world constraint this project
doesn't currently model.

f. **Encoding target, if confirmed**: `lever_bridges` entries —
`brake_bias` direction `more_front` → condition `{verdict: understeer,
min_severity: moderate}`, `phase_groups: [["entry_1_brake"], ["entry_2_turnin"]]`
(per the book's own "corner-entry followed by mid-corner" phrasing);
`brake_bias` direction `more_rear` → condition `{verdict: oversteer,
min_severity: moderate}`, `phase_groups: [["entry_1_brake"]]` (the book
specifies corner-ENTRY only for the rear-bias case, not mid-corner). Grade
`proposed`, `effect_class` `secondary` (mirroring how springs_front/rear
were seeded relative to the matrix's own primary cells) — pending the
engineer-confirmation question in (d), not shipped in this document.

### C5-2 — Wheel lockup detection via per-corner speed drop: a real, more direct evidence source than the existing CS-ratio proxy

a. **Statement** (p.106-107, 5.4): "the easiest way to detect lock-up is
the speed graphs... When a wheel locks, the speed trace drops nearly
vertically." A positive longitudinal slip ratio during braking indicates a
front wheel locking; negative indicates rear. The wheel most likely to lock
is the least-loaded one (inside of the corner) on the axle carrying the
higher brake bias.

b. **Mechanism**: a locked wheel's rotational speed decouples from true
ground speed; comparing each corner's own wheel speed against the other
three (or against a GPS/non-driven reference) directly exposes this,
independent of any tyre-model assumption.

c. **Applicability**: transfers directly, and the required channels exist
and are ALREADY registered on this car (`log_speed_fl/fr/rl/rr[kph]`,
confirmed in config/channels.json) — no new instrumentation needed, unlike
most of Chapter 8's tire-sensing content.

d. **Cross-check**: NOT BUILT. No lockup-detection logic exists anywhere in
modules/ (confirmed: every "lock" hit across modules/stability_analysis.py
and modules/corner_analysis.py is a false positive — steering-lock,
"gridlock"-style phrasing, unrelated). The EXISTING brake-balance evidence
(`_build_brake_balance_evidence`, `plausibility_checks.brake_balance_
signature`) is a PROXY built from cornering-stiffness-ratio severity
(front-beyond-limit while rear stays healthy during the braking phase) —
useful, and already wired to real candidates, but one step removed from
what actually happens at the tyre. A genuine per-corner speed-drop lockup
detector would be a direct, first-principles complement to that existing
proxy, not a replacement — corroborating (or occasionally contradicting,
which would itself be informative) the CS-ratio-based signal.

e. **VERDICT: INSIGHT-ONLY for this package** — a new Tier B evidence
source, not itself an `interaction_table`/`lever_bridges` row. High-value
for the future-feature list precisely because the channels already exist
and are already registered — the lowest-effort new-evidence proposal in
this entire review.

f. No interaction_table/lever_bridges target directly. Future evidence-
source note: a `_build_lockup_evidence` function (per-corner speed-drop
detection during `entry_1_brake`, direction of asymmetry indicating which
axle carries excess bias) would strengthen the existing brake_balance
plausibility check and directly support C5-1's `brake_bias` bridge (if
confirmed) with real, mechanism-level evidence instead of the CS-ratio
proxy alone. The raw file's own `log_speed_*_lock[s]` sub-channels (present
but unregistered, semantics not decoded in this pass) may already do part
of this work at the logger level — worth checking before building a
from-scratch detector.

### C5-3 — Brake disc temperature and front/rear temperature balance (5.8): channels exist, unregistered, future feature

a. **Statement** (p.116-121, Eq.5.4-5.10): front/rear brake disc
temperature and their ratio (Eq.5.4, directly analogous in form to the
brake PRESSURE balance of Eq.5.3) track mechanical brake balance with a
time lag; temperature build-up/cooling-speed channels (Eq.5.7-5.10) support
pad-compound and cooling-system evaluation.

b. **Mechanism**: braking converts kinetic energy to heat at the
pads/rotor; more brake-line pressure (or more aggressive bias) on an axle
produces more heat there, so the front/rear temperature ratio tracks
(lags) the same mechanical balance the pressure ratio measures directly.

c. **Applicability**: transfers directly — this is a real, if indirect,
substitute for the brake-PRESSURE channels this car does not log (see
chapter preamble), since disc temperature IS logged.

d. **Cross-check**: NOT REGISTERED. `log_brkdisctemp_fl/fr/rl/rr[°C]` exists
raw in Sample_Dubai.txt (confirmed by direct grep) but has zero entries in
config/channels.json. No brake-temperature evidence or analysis exists
anywhere in modules/ (confirmed alongside the lockup check above).

e. **VERDICT: INSIGHT-ONLY** — not itself a lever_bridge (brake temperature
is a measured OUTCOME, not a setup lever), a channel-registration and
future-evidence-source opportunity for the future-feature list, similar in
kind to C8-1/C8-2's TPMS finding.

f. No interaction_table/lever_bridges target. Future-feature note:
registering these 4 channels would be the prerequisite for a
temperature-balance evidence source complementary to (not a replacement
for) C5-2's lockup-based and the existing CS-ratio-based brake-balance
signals.

### C5-4 — Target braking-G ceiling from cornering capability (5.2, Buddy Fey method): insight only, cannot validate without brake pressure

a. **Statement** (p.102-103, Eq.5.1/5.2): peak achievable longitudinal
deceleration is bounded by the car's own cornering capability (~91-95% of
max lateral G, with vehicle-configuration corrections), combining via the
traction-circle relationship (Eq.5.2) to give a speed/lateral-G-dependent
braking-G target; underperforming this target may indicate a brake-balance
or vehicle-configuration problem rather than a driver deficiency.

b. **Mechanism**: total available grip forms a roughly circular limit
(traction circle, ch.7); braking and cornering share the same tyre grip
budget, so the achievable longitudinal deceleration at a given lateral G is
bounded by the same total-grip envelope the car already demonstrates in
pure cornering.

c. **Applicability**: transfers directly — the traction-circle mechanism is
vehicle-class-independent, though the specific correction percentages (front
/rear engine, tyre contact patch shape) are the book's own examples, not
this car's.

d. **Cross-check**: NOT BUILT (no combined-acceleration-vs-target-ceiling
channel exists in modules/), and CANNOT be cross-validated against an
actual under-braking diagnosis without brake pressure data this car does
not log — the method can flag "this car isn't reaching its computed
ceiling" but, without brake_bias/pressure telemetry, cannot distinguish
driver conservatism from a real setup problem, which is exactly the
distinction the book itself says the method is for.

e. **VERDICT: INSIGHT-ONLY** — a diagnostic method (Tier B), not a setup
lever; recorded for the future-feature list with its own real limitation
named (needs brake pressure to be fully useful, which this car lacks).

f. No encoding target.

### Chapter 5 census: 1 new lever_bridge candidate flagged for engineer confirmation rather than silently adopted (C5-1 — brake_bias, real lever, zero coverage, adjacent to but absent from 7 engineer-verbatim cells), 1 high-value low-effort future evidence source using already-registered channels (C5-2, lockup detection), 2 further INSIGHT-ONLY future-feature notes (C5-3 brake temp, C5-4 braking-G ceiling). 0 REJECT, 0 direct contradictions with existing bridges (C5-1 is an absence, not a contradiction, and is named as such).

---

## Chapter 7 — Cornering, targeted read: 7.5-7.7 (pp.156-168)

Read narrowly per the Phase 1 plan: not a lever-family chapter at all, but
the book's own foundation for WHAT understeer/oversteer/yaw-balance
CLASSIFICATION means and how to measure it from telemetry — directly
comparable to this project's own classification method
(modules.stability_analysis, CS_ratio / cornering-stiffness-ratio, Werner
MA-anchored) and yaw-balance method (modules/yaw_stability.py). No setup
lever appears anywhere in 7.5-7.7; every finding here is either a
methodological cross-check or a hardware-availability check, never a
lever_bridge/interaction_table candidate.

### C7-1 — The book's own classical understeer/yaw metrics admit exactly the limitation our CS_ratio method was built to avoid

a. **Statement** (p.163, 7.5; p.166, 7.6): the understeer-angle definition
(Ackermann-deviation, Eq.7.6) is, in the book's own words, "valid only in
the linear operating range of the tires and under steady-state
conditions... If the equations are applied in the nonlinear operating
range of the tires, their mathematical validity is lost." The attitude-
velocity yaw-balance metric (Eq.7.12) carries the same admitted caveat:
"this mathematical channel does not take into account tire slip angles."

b. **Mechanism**: both metrics are KINEMATIC proxies (steering geometry vs.
actual path, or yaw rate vs. a steady-state-cornering theoretical value) —
neither directly measures how saturated the tyres actually are; both are
only exact when the tyres are still operating in their linear
force/slip-angle region, which a race car at the limit routinely is not.

c. **Applicability**: directly relevant as a methodological comparison —
this project's cars are raced at or near the tyre limit by design, exactly
the regime where the book admits these classical metrics lose validity.

d. **Cross-check**: this project's OWN classification method
(modules.stability_analysis, CS_ratio — the ratio of a corner's measured
cornering stiffness to its own linear-region reference stiffness, Werner
MA-anchored per thesis_notes.md "CS_ratio (cornering stiffness ratio)")
measures tyre saturation DIRECTLY (how far into the nonlinear region the
tyre is actually operating), rather than inferring balance from steering
geometry or yaw kinematics. This is not a contradiction of the book — the
book itself flags exactly this gap and even says these metrics remain
"still a very powerful tool... to detect trends" despite it (p.162) — it
is independent, book-sourced corroboration that this project's own
CS_ratio-based approach is methodologically better suited to the regime
this car actually races in.

e. **VERDICT: INSIGHT-ONLY — strengthens confidence in an existing method
choice, not a new candidate.** No lever, no new evidence source proposed;
this is a citation opportunity for the existing CS_ratio method's own
documentation.

f. No encoding target. Documentation note: the existing CS_ratio method
entry (thesis_notes.md) could cite Segers ch.7.5/7.6 as independent
confirmation that classical kinematic understeer/yaw metrics are
explicitly nonlinear-tyre-limited, motivating the saturation-based
alternative this project already uses.

### C7-2 — Attitude velocity (yaw rate vs. theoretical) as a cheap, independent classical cross-check — distinct from the existing yaw_stability module

a. **Statement** (p.163-165, Eq.7.10-7.13): attitude velocity = measured
yaw rate − theoretical steady-state yaw rate (V/R, from speed and lateral
G alone); positive means the car tends to oversteer, negative understeer.
Needs only yaw rate, lateral acceleration, and speed — no steering angle,
no per-axle sensing.

b. **Mechanism**: in a steady-state turn at a given speed and radius, yaw
rate has one theoretically consistent value; any measured excess is the
car rotating faster than pure kinematics would predict (rear stepping
out), any deficit is rotating slower (understeer).

c. **Applicability**: the required channels (yaw rate, lateral G, speed)
are basic IMU/GPS signals this project certainly logs already (used
throughout stability_analysis.py) — no new instrumentation needed.

d. **Cross-check**: this project already has a yaw-balance evidence source
(modules/yaw_stability.py, referenced in modules/stability_analysis.py,
described elsewhere in this project's own deviation taxonomy as using the
chair's own more sophisticated ridge-regression approach on filtered yaw
acceleration — not read in full during this targeted chapter-7 pass, so no
claim is made here about its internal method beyond what is already
on record). Attitude velocity is a MUCH simpler, purely classical
formula — not a replacement, but a candidate cheap, independent SANITY
CHECK a more complex estimator could be cross-validated against, in the
same spirit as C5-2's lockup detector or C13-2's constant-velocity aero
test: an cheap, independent, book-anchored second opinion.

e. **VERDICT: INSIGHT-ONLY** — a potential future cross-validation
evidence source, not a lever_bridge (no setup lever; a classification
method). Recorded for the future-feature list.

f. No interaction_table/lever_bridges target. Future note: a cheap
`attitude_velocity` math channel could serve as an independent
cross-check against modules/yaw_stability.py's own verdict, flagging
disagreements for engineer review rather than replacing either method.

### C7-3 — Front/rear lateral acceleration comparison (7.7): REJECT, no hardware

a. **Statement** (p.166-167): if lateral acceleration can be measured
separately at the front and rear axles, the axle with the higher value has
the higher grip — front > rear indicates oversteer, front < rear indicates
understeer (Eq.7.14/7.15 additionally allow deriving yaw rate from this
pair in the absence of a gyro).

b. **Mechanism**: the axle generating more lateral force per unit mass
(higher local lateral G) is, by definition, closer to its own available
grip limit relative to the other axle.

c. **Applicability**: mechanism transfers directly, IF the sensors existed.

d. **Cross-check**: NO HARDWARE, checked directly against the raw file, not
assumed. No front/rear-mounted lateral accelerometer channel pattern
exists anywhere in Sample_Dubai.txt (grep for lateral-acceleration-like
channel names combined with front/rear qualifiers, excluding wheel-speed/
brake/suspension channels, zero hits) — this car almost certainly logs a
single CG-mounted IMU, like the vast majority of cars this book itself
targets outside single/dual-purpose research rigs.

e. **VERDICT: REJECT** — no lever_bridge/evidence-source candidate possible
without new physical instrumentation (a second, axle-mounted
accelerometer), outside this project's scope to propose.

f. No encoding target.

### Chapter 7 census: 0 lever_bridge/interaction_table candidates (not a lever chapter by design — targeted read confirmed this), 1 REJECT (C7-3, no hardware), 2 INSIGHT-ONLY methodological findings (C7-1 corroborates the existing CS_ratio method choice against the book's own admitted classical-metric limitation; C7-2 names a cheap future cross-check for the existing yaw_stability module). 0 contradictions.

---

## Chapter 4.3 — TCS and Slip Ratios, targeted read (pp.88-91)

Read narrowly per the Phase 1 plan, as the one section of the
straight-line-acceleration chapter touching a registry lever (`tc_lon`).
Genuinely empty of new candidates — recorded honestly rather than padded,
per the work order's own "if a chapter yields nothing, say so" instruction.

**What the section actually covers**: how an engine-controlled traction
control system works mechanically — a nominal-slip lookup table (indexed
by speed, throttle position, and lateral G, not a single constant),
ignition retard as the intervention mechanism, and a regulatory workaround
for series that permit only one wheel-speed sensor (Eq.4.14, calculating
driven-wheel speed from engine RPM and gear ratio) — not applicable here,
since this car already logs all four corner wheel speeds directly
(confirmed in the Chapter 5 census above). The section explicitly notes
"the slip value where maximum traction occurs is not a constant and is
likely to vary even during one race lap" — a caveat about what a single
`tc_lon` "position" setting actually indexes into (a multi-variable ECU
table), not a setup-change → behaviour-tendency claim of the kind every
other chapter in this review yielded.

**Cross-check against the registry**: `tc_lon` already carries 2
`interaction_table` entries, confirmed by direct query — consistent with
the earlier literature-bridge coverage-check package's finding that
`tc_lon` is fully matrix-covered. This section adds mechanism-level
background for why `tc_lon` behaves the way it's already encoded, not a
new direction or magnitude. `tc_lat` (cornering-phase traction control)
carries ZERO `interaction_table` entries — a real gap, but this section is
specifically about STRAIGHT-LINE acceleration slip control and says nothing
about cornering-phase TC; a `tc_lat` gap would need Chapter 7's cornering
content (already read, targeted to 7.5-7.7 only, which does not cover TC
either) or a different source — not manufactured here from unrelated text.

**VERDICT for this section**: no ADOPT/ADAPT/REJECT/INSIGHT candidates
generated — the section is confirmatory background for an already-fully-
covered lever, honestly reported as yielding nothing new rather than
stretched into a candidate.

### Section 4.3 census: 0 candidates of any kind. Empty result, reported as such.

---

# Phase 3 — Consolidation

32 candidates total across 8 chapters/sections (Ch.11, 9, 10, 8, 13, 5, 7
targeted, 4.3 targeted). Reading order was chosen by expected gap size
(dampers, the biggest known gap, first) rather than book order; this
section re-groups everything by lever family and by verdict class for a
reader who wants the destination rather than the journey.

## Summary census (per chapter/section)

- **Ch.11 Shock Absorbers**: 5 candidates — 0 clean ADOPT, 2 ADOPT-with-
  open-decision (C11-3, C11-5), 2 ADAPT-blocked-on-schema (C11-2, C11-4),
  1 INSIGHT-ONLY (C11-1). 0 REJECT.
- **Ch.9 Roll Stiffness Distribution**: 5 candidates — 0 ADOPT/ADAPT,
  3 REJECT (C9-1, C9-2 confirmatory overlaps with already-shipped bridges;
  C9-5 no lever), 2 INSIGHT-ONLY (C9-3 registry gap, C9-4 out-of-scope
  diagnostic).
- **Ch.10 Wheel Loads and Weight Transfer**: 5 candidates — 1 ADOPT
  (C10-3, annotation), 1 REJECT (C10-4, duplicate of C9-3), 3 INSIGHT-ONLY
  (C10-1, C10-2, C10-5).
- **Ch.8 Understanding Tire Performance**: 4 candidates — 0 ADOPT/ADAPT,
  1 REJECT (C8-3, real lever/no hardware), 3 INSIGHT-ONLY (C8-1, C8-2,
  C8-4).
- **Ch.13 Aerodynamics**: 6 candidates — 3 ADOPT (C13-1, C13-2, C13-3,
  all citation/validation-protocol additions, no new interaction_table
  rows), 0 REJECT, 3 INSIGHT-ONLY (C13-4, C13-5, C13-6).
- **Ch.5 Braking**: 4 candidates — 1 ADAPT-flagged-for-confirmation
  (C5-1), 0 REJECT, 3 INSIGHT-ONLY (C5-2, C5-3, C5-4).
- **Ch.7 Cornering (targeted 7.5-7.7)**: 3 candidates — 0 ADOPT/ADAPT,
  1 REJECT (C7-3, no hardware), 2 INSIGHT-ONLY (C7-1, C7-2).
- **§4.3 TCS and Slip Ratios (targeted)**: 0 candidates — empty result.

**Totals**: 6 ADOPT (all citations/annotations/protocol proposals, ZERO new
`interaction_table` rows), 3 ADAPT (2 blocked on schema, 1 pending
engineer confirmation), 6 REJECT, 17 INSIGHT-ONLY. Not one candidate in
this entire review proposes a new signed `interaction_table` row ready to
ship as-is — every genuinely new directional claim (C11-2, C11-4, C11-5,
C5-1) carries a named, unresolved blocker (schema gap, axis-mapping
decision, sign-convention conflict, or engineer confirmation).

## Full candidate list, grouped by lever family

**springs_front / springs_rear** (already fully shipped, BACKLOG item H):
no new candidates; C9-1 and C10-3 add corroboration and a magnitude
caveat (chassis torsion) to the existing entries.

**arb_fl/fr/rl/rr** (already fully shipped, matrix-covered): no new
candidates; C9-2 corroborates, C10-3's chassis-torsion caveat applies here
too.

**damper_bump_ls/hs, damper_rebound_ls/hs (×4 corners), damper_blowoff**
(ZERO prior coverage — the biggest single gap this review addresses):
C11-2 (rebound_ls_fl/fr, understeer, entry_2_turnin — ADAPT, blocked on a
new per-corner motion-state evidence source and condition type), C11-3
(symmetric L/R annotation for any damper bridge), C11-4 (bump_hs/rebound_hs,
traction_performance/braking_performance, direction-only), C11-5
(rebound_ls_front, platform_stability/rake, sign-convention conflict with
existing entries to resolve).

**ride_height_front/rear** (already shipped, platform_stability): C13-1
adds a second, independent anchor (aero-coefficient dependency, distinct
from the roll-stiffness/rake mechanism already cited).

**wing_position / splitter_offset** (ZERO prior coverage, newly
discovered this session): C13-6 — real gap, no clean axis fit under the
current vocabulary; not encoded, named for a future vocabulary extension.

**brake_bias** (ZERO prior coverage, newly discovered this session):
C5-1 — the single most direct, literally-worded book statement in this
entire review, flagged for engineer confirmation rather than adopted,
because it sits adjacent to (not contradicting) seven engineer-verbatim
matrix cells that solve the identical scenario with different levers.

**abs_position**: no new candidates (already covered by two HELD
escalation cells; Ch.5.7's ABS material was descriptive, not a new
directional claim).

**camber_fl/fr/rl/rr**: C8-3 — the chapter's most directly actionable
content (3-zone IR tire-temp camber evaluation), blocked purely by
instrumentation this car does not carry. REJECT, not encodable.

**tc_lon**: confirmed already fully matrix-covered (§4.3 census); no new
candidates. **tc_lat**: a real zero-coverage gap exists but was not
addressed by any chapter read in this package (§4.3 is straight-line-only;
targeted Ch.7 didn't cover TC either) — named, not filled.

**tire_pressure (no such lever exists in the registry at all)**: C9-3/
C10-4 — a first-order roll-stiffness contributor per the book, with no
parameter to attach it to. Registry gap, not a bridge gap.

**kinematic_variants**: C9-5 — explicitly `recommendation_target: false`
in the registry; the book's pitch-gradient/anti-dive content correctly has
no encodable target.

**diff_package / diff_position**: no book chapter read in this package
gave usable content — Chapter 7's targeted read (7.5-7.7) is measurement
methodology, not lever-specific, and no dedicated differential chapter
exists in this book's table of contents at all (noted in the Phase 1
reading plan). **This is itself the finding, as flagged before reading**:
these two levers' bridges rest on engineer elicitation alone, which is
exactly the provenance system's job when literature offers nothing —
Milliken (Race Car Vehicle Dynamics) remains the literature home for
differential effects if a future session takes up that book.

## Contradiction / tension list

**No direct contradictions** with any engineer-verbatim or matrix-derived
bridge were found — every existing signed entry checked against this
book (springs, ARB, ride_height, tc_lon) agreed in direction, several with
independent worked-numeric corroboration (C9-1, C9-2).

Two **tensions**, both named at the point they arose rather than silently
resolved:

1. **C5-1 (brake_bias absence)**: not a sign contradiction, but a real
   adjacency tension — seven engineer-verbatim/project-lead-reviewed
   matrix cells solve braking-phase understeer/oversteer/instability
   without ever touching `brake_bias`, despite the book's mechanism being
   textbook-direct. Requires an engineer decision (deliberate omission vs.
   genuine gap), not a literature-only resolution.

2. **C11-5 (platform_stability sign-convention conflict)**: the existing
   shipped `platform_stability` entries (springs, ride_height) all use a
   magnitude-only "-1, either direction trades margin" convention. The
   book's front-rebound-holds-rake statement is genuinely directional
   (more rebound → more rake, not "either way costs margin"). Encoding it
   under the existing convention would misrepresent the book's own claim;
   resolving this requires a scoring-layer decision (allow directional
   `platform_stability` entries, or represent this one differently) that
   this document deliberately leaves open rather than deciding unilaterally.

## Proposed implementation order (smallest coherent package first)

**Package 1 — pure documentation/citation additions, config+docs only, zero
module logic, zero new evidence, ships with no open questions**: C10-3
(chassis-torsion caveat on the shipped springs/ARB bridges), C13-1 (second
aero anchor on the shipped ride_height entries), C13-3/C13-4/C13-5 (three
additions to `aero_front_fraction_note` — functional-form citation, air-
density confound, wind confound). Lowest possible risk; matches the
"config + docs + tests only" scope the earlier literature-bridge package
already used successfully.

**Package 2 — one engineer decision, then trivial to encode**: C5-1
(`brake_bias` lever_bridges). If confirmed, ships via the SAME generic
`_bridge_candidates_for_levers` mechanism BACKLOG item H already built —
zero new module code, only two new config rows. If declined, the reasoning
itself (why brake bias is deliberately left to the seven existing cells)
becomes a documentation addition worth recording.

**Package 3 — cheapest new evidence source, channels already registered**:
C5-2 (per-corner wheel-speed lockup detection). `log_speed_fl/fr/rl/rr`
already exist in config/channels.json; this is a new evidence-building
function only, no channel registration and no schema change needed.

**Package 4 — channel registration + new evidence source, two small
chapters worth of gap closed together**: C8-1+C8-2 (TPMS pressure/
temperature, Dubai-only — register 8 channels, then build the axle-average/
L-R-difference evidence source) and C5-3 (brake disc temperature —
register 4 channels; a temperature-balance evidence source can follow once
C5-2's lockup evidence establishes the pattern).

**Package 5 — the named schema extension, largest single piece**: C11-2
(damper transient-phase bridges) — requires the new per-corner
motion-state evidence source AND a new `lever_bridges` condition type that
can gate on which specific corner is unloading, not just verdict. This is
the exact item PLAN.md already carries as "PENDING SCHEMA SUPPORT" (item 3,
literature-bridge coverage-check package) — this review supplies the full
worked mechanism and phase-scoping (C11-2), the symmetry annotation to
attach once it ships (C11-3), and the honest statement of what schema work
it actually requires, closing that PLAN.md item's own open question rather
than leaving it as a bare pointer.

**Package 6 — vocabulary/axis decisions, deferred**: C11-4 (damper HS →
traction_performance/braking_performance — direction-only, no magnitude,
needs those two inert axes activated in `_AXIS_TO_VERDICT` before it means
anything in scoring) and C13-6 (wing/splitter → no existing axis fits;
would need a new `drag_performance`/`straight_line_performance` axis).
Both real, both correctly left undecided here — extending the axis
vocabulary is a scoring-architecture decision outside a literature-review
document's remit.

**Not a code package — an on-track test proposal**: C13-2 (constant-
velocity + coast-down aero test, both directions, controlled suspension
prep). The chapter's single highest-value finding, but its deliverable is
a test session, not a commit — proposed as a new line item on PLAN.md's
existing "Engineer follow-up questions — damper/wheel-load package" list,
which already carries the exact open question (`aero_front_fraction`
confirmation) this test would answer with a book-standard protocol instead
of windtunnel/CFD access this project doesn't have.

**Named but not queued**: C11-5's sign-convention conflict (needs a
scoring-layer decision before ANY implementation order applies to it);
C7-2/C8-4 (cheap future cross-check evidence sources, lower priority than
Packages 3-4); the diagnostics-only insights that were explicitly ruled
out of this package's scope (C9-4 roll-ratio anomaly/puncture detection,
C10-1/C10-2 wheel-load-estimator upgrades, C10-5 banking/slope) — real
findings, recorded, but belonging to a different future work package than
"bridge review."

## thesis_notes.md pointer

A short dated entry pointing to this document (not duplicating its
content) is the next step, per the work order's own instruction.
