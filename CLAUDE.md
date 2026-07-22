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