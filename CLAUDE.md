# SetupTool — Claude Code instructions

PyQt6/SQLAlchemy desktop app for Porsche 992 GT3R race engineering.
Bachelor thesis (TUM / Proton Competition). Windows, PowerShell, venv.

## Commands
- Activate venv: `.\.venv\Scripts\Activate.ps1`
- Run app: `python main.py`
- Test pipeline (no UI): `python test_stability.py`
- Sample data: `C:\UNI\Bachelorarbeit\Data\Sample\Sample_Dubai.txt`
  (7 laps: 0=outlap, 1-5=valid, 6=inlap)

## Work plan
PLAN.md in this repo contains the work packages (WP1-WP7). Execute them in
order. Read PLAN.md before starting any work.

## Non-negotiable rules
- One work step at a time. PROPOSE the change and wait for my confirmation
  before editing any file. Explain the science/method before the code.
- Never guess: read the actual file before editing it.
- No PyQt6 imports in modules/ or core/. No business logic in ui/.
- All tunable numbers go in config/*.json, never hardcoded.
- Colour literals only from ui/style.py constants (OK, WARN, BAD, NEUTRAL,
  ACCENT, PANEL, PANEL_ALT, BORDER, TEXT, TEXT_MUTED, TEXT_DIM).
- Short comment blocks. Real data only — no synthetic test data.
- Accuracy levels: 1=config default, 2=session measurement, 3=logged sensor,
  4=lookup table. Every physical quantity states its level.
- After each change, run `python test_stability.py` and show me the output
  before continuing. UI changes: I run the app manually and report back.
- HANDOVER.md and generate_handover.py stay local — never commit them.
- If a work order in PLAN.md is ambiguous or conflicts with the real code,
  STOP and ask me. Do not choose silently.

- After implementing any step, before I test: summarise what changed
  file-by-file, and explain any new function's logic in 3-5 sentences —
  the method, not just the mechanics. This is thesis code; I must be able
  to defend every line.
- When I ask "explain X", walk through the actual code line by line.

End every turn with a "RESULT:" block — 3-6 lines of plain-language summary of findings/changes, separate from the working output.s

- thesis_notes.md in the project root collects thesis material. When a
  session produces a thesis-worthy finding — a method justification, a
  validated result, a discovered limitation, a design principle — append
  it under the matching section with a date. Never delete or rewrite
  existing entries; if superseded, strike through with a dated note.
  When in doubt whether something is thesis-worthy, ask me.

  - At the end of every session and before every commit: update the
  STATUS block in PLAN.md. Commit checkpoints are per work package
  (or per approved sub-issue if a WP spans days) — never leave a WP
  boundary uncommitted, never commit mid-implementation.

## Scientific grounding rule (hard, project-wide)
Three tiers govern every implementation decision:
- Tier A (vehicle dynamics methods): only with a literature anchor
  (Werner 2021, Milliken, standard textbooks). No unanchored
  methods. Docstrings carry the reference.
- Tier B (signal/data engineering: filters, segmentation,
  clustering, exclusion masks, thresholds): standard techniques
  only, parameters config-driven, documented and data-derived,
  explicitly labelled as preprocessing — never presented as
  methodological novelty.
- Tier C (UI/product): free.
When proposing anything technical, state its tier and anchor. If no
anchor exists for a Tier A proposal, STOP and ask — never invent.
Rationale: bachelor thesis defended before doctoral-level examiners;
the author must understand and support every technical decision.
Traceable science beats better results: a defensible method is
always preferred over an unexplainable improvement.
Calibration tunables (thresholds, cutoffs, gates tuned to data or
hardware) go in config; METHOD-DEFINING constants (filter order,
blend exponents, normalisers — values that define what the method
IS, not how it's tuned) stay as named constants in code with a
justifying comment, since moving them to config would misrepresent
them as tunable.

## Deviation taxonomy (chair performance_analysis comparison)
Every place SetupTool's Module 4b/5 estimators differ from the chair's
reference implementation (docs/literature/, read-only, never imported)
carries exactly one of three class labels:

1. FORCED ADAPTATION — the GT3R sensor/data situation leaves no
   alternative (examples: kinematic beta, Level-1 Fy split, the
   raw-yaw-rate path because the chair's pre-smoothed-input filter
   list is outside this reference's scope). Framing: same method,
   different available inputs.
2. DOMAIN IMPROVEMENT — the chair version is correct for its own
   context; our context differs and we improve on it for our use case.
   Always framed as "based on their version, which is not wrong."
3. NEUTRAL ENGINEERING — no science content (channel-alignment
   guards, config key naming, module boundaries).

VEHICLE PARAMETERIZATION IS NOT A DEVIATION. The chair tooling is
vehicle-agnostic; all physical vehicle quantities enter through config
(car.json / car_data.json / setup values). SetupTool parameterizes the
identical algorithms for the Porsche 992 GT3R, whose properties differ
fundamentally from the chair's reference vehicle (different vehicle
class). "Chair-identical" always refers to the algorithm, never to
vehicle numbers. Provenance of vehicle quantities is governed by the
Level 1-4 accuracy system, not by chair comparison.

Parameter categories, for clarity:
- vehicle description (mass, Iz, wheelbase, track widths, ...): differ
  from the chair BY NECESSITY; provenance = accuracy levels.
- method calibration tunables (yaw_stability_* six, cs_* values, ...):
  match the chair BY CHOICE; changing any is an estimator change and
  re-triggers threshold re-derivation.
- classification thresholds: differ from any chair values BY RULE;
  always re-derived from this car's own distribution, never carried
  over from the chair or from a prior estimator's distribution.

## Comment style rule (project-wide)
Comments and docstrings must read like a capable engineering student
wrote them for himself and his examiners:
- Explain WHY (reasoning, units, references, pitfalls), not WHAT the
  next line obviously does. Never restate code in prose.
- Short where things are obvious; full sentences where a decision
  needs justification. No comment is better than a filler comment.
- No boilerplate docstrings on trivial helpers; substantial
  docstrings only where method or contract needs stating.
- No decorative headers, no emoji, no "This function is responsible
  for..." filler, no exhaustive parameter lists that repeat obvious
  names, no marketing adjectives.
- Consistent plain English, ASCII in .py files.
- When editing a file for any reason, bring touched comments up to
  this standard; do not launch mass rewrites without instruction.

## diagnostics/ disposal rule (standing, 2026-08-30)
A diagnostic script is disposable by default. Investigation scripts
are scaffolding, not deliverables. Once a script's finding is
recorded in thesis_notes.md and its work package commits, DELETE the
script in that same commit unless it qualifies as one of:
- **Referenced** — cited by exact filename from outside diagnostics/:
  a docstring/comment pointer in modules/ui/core, a config
  `derived_from`/provenance string, or a PLAN.md entry.
- **Reproduces** — live tooling still needed: a golden-value
  generator, the frozen pass-1 validation baseline, a reusable
  headless Qt smoke test, or (rare, flag it) a script something in
  modules/ actually imports from.
- **Dependency** — imported by another diagnostics script that itself
  qualifies as Referenced or Reproduces.
Do not accumulate a growing pile of "might be useful later" one-offs
between cleanup passes — dispose at commit time, not in a later batch
sweep. diagnostics/README.md must list every surviving script's
specific keep-reason; a script with no stated reason there is a bug
in the README, not license to leave the next one uncommented.