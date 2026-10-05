# Evaluation protocol (pre-registered)

Status: PRE-REGISTERED, 2026-10-03. Written before any controlled
evaluation data exists. Changes after this date are dated addenda at
the end of this file; the original text is never edited.

## 1. Purpose and framing

The thesis evaluates the SetupTool analysis chain on real GT3R
telemetry by two pairwise comparisons. Framing (supervisor direction,
2026-10 meeting): this is a CHARACTERISATION of the method against
independent references, not a statistical validation. The method's
validity rests on its physics grounding and traceability (Milliken
MMM; Hoffman et al. 2008 front-saturation = controllability loss,
rear = stability loss; Ono 1998 saddle-node; chair
performance_analysis tooling, internal). The argument stands without
driver feedback; agreement with driver statements or team decisions
is corroborating evidence, disagreement is discussion material.
Neither is a pass/fail criterion.

## 2. Comparison 1 - diagnosis layer

Question: does the tool's state description correspond to what the
driver reports?

Tool side: per-corner, per-phase CS_front, CS_rear and the stability
metric as rendered by production at the then-current thresholds
(threshold provenance stated in the results chapter; figures
generated only after the post-Q-CAL threshold re-derivation).

Driver side: sentence-wise feedback, recorded verbatim, attributed to
run and (where the driver names it) corner.

Mapping rule (applied per statement BEFORE inspecting the tool output
for that session):
- Each statement is decomposed into: axle (front/rear/unclear),
  corner (canonical corner ID, or session-general), phase
  (entry/mid/exit, or unspecified), direction (understeer /
  oversteer-instability / unpredictability / other).
- Expected signature, from the method lineage:
  - understeer complaint -> front-axle CS saturation / reduced CS
    margin in the named phase (controllability loss, Hoffman);
  - oversteer or instability complaint -> rear-axle CS saturation
    and/or negative-stability samples in the named phase (stability
    loss, Hoffman; saddle-node, Ono);
  - unpredictability / "nervous" without axle attribution ->
    stability-margin reduction, axle left open;
  - statements outside the tool's domain (kerbs, traffic, engine,
    brakes-as-system) -> OUT OF SCOPE, listed, not scored.
- The mapping of each statement is written down before the
  corresponding tool output is opened. Where feedback was already
  known to the author (retrospective cases), this is stated and the
  case is labelled accordingly (section 5).

Outcome categories per in-scope statement:
- MATCH: expected signature present at the named corner/phase (or,
  for session-general statements, in the session-level read-out).
- PARTIAL: signature on the named axle but wrong phase, or named
  phase but weak/graded signal.
- NO SIGNAL: windows graded insufficient / no coverage at the named
  location. Counted separately; a coverage statement, not a miss.
- CONTRADICTION: opposite signature at the named location.

Counts are reported, but with the expected small N every PARTIAL and
CONTRADICTION is discussed individually (case-study framing).

## 3. Comparison 2 - recommendation layer

Question: does the tool's recommendation correspond to the change the
team actually made?

Tool side: the rendered recommendation list (top entries as displayed
by production) for the outing preceding the change.

Team side: the recorded setup change (parameter, direction; values if
released), made by the team on their own process (data + driver
feedback, independent of the tool unless stated otherwise).

Agreement scale (direction-only; magnitudes are out of scope on the
Level-1 parameter basis):
- A1: same parameter, same direction.
- A2: different parameter, same physical intent. Intent is argued in
  writing from the lineage physics (e.g. both reduce front mechanical
  grip contribution), never asserted.
- A3: disjoint (tool and team address different problems).
- A4: opposing (same mechanism, opposite direction).

Per case record: session/outing, tool top entries, team change,
who-knew-what (whether the team saw tool output before deciding;
contamination direction stated), classification, one-paragraph
rationale.

Supervisor direction (2026-10 meeting, binding on interpretation):
recommendations are NOT required to coincide with team changes; A2-A4
outcomes feed the discussion of what each process weighs, they do not
invalidate the method.

## 4. Shared-input caveat (stated in the discussion)

The team's change is informed by the same driver feedback used in
Comparison 1. The two comparisons therefore share an input and are
correlated evidence, not independent confirmations. This is stated
explicitly in the discussion chapter.

## 5. Admitted cases

- CONTROLLED: the planned documented session(s) (data request in
  progress). Requirements per case: valid laps before the change;
  verbatim, run-attributed driver feedback; documented change
  (parameter + direction minimum); laps after the change where
  available.
- RETROSPECTIVE: cases already in the archive, admitted with weaker
  documentation and labelled as such. Currently identified: the NBR
  A->B change (rake + springs; driver: B "more predictable") and the
  phase-differentiated pro-driver NBR statements (Phase-1 criteria
  (ii) and (iii)). Their documentation gaps (exact change records,
  feedback verbatim) are stated where they bind.

## 6. Non-goals

- No statistical validation claims (N is small by design).
- No magnitude claims for recommendations.
- No lap-time-improvement claims (closed loop, out of scope).
- No ranking of the tool against the team's process.

## 7. Confidentiality

Absolute setup values, car-identifying data and team records go to
the confidential appendix or are presented normalised/relative in the
public text, per the Geheimhaltungsverpflichtung. Decided per figure
and per table at specification time.
