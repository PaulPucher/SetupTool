# DECISIONS -- standing rulings, current state only

One line per ruling. Current state only: superseded versions are not listed
here (their history stays in PLAN.md and thesis_notes.md). Maintained alongside
PLAN.md: every ruling that lands in PLAN lands here in the same turn, and a
line that changes is replaced, not appended. Distilled 2026-10-05.

## Scope and product
- Thesis centerpieces are fixed: CS, stability, recommendations; everything ships only insofar as it serves them (2026-10-05).
- Q-SLIM: every module, UI element, export and file justifies itself by a consumer in the shipped minimal flow or an anchor the author defends; unreferenced = removal candidate (2026-10-05).
- Minimal app: load an outing -> laps with in/out marked -> recorded data below; analysis runs with best-available defaults (2026-10-05).
- Removed from the app (pending the UI batch): accuracy-level chooser, tyre-model chooser (the sideslip-estimator selector removed 2026-10-06), lap-selection UI, weekend PDF export (2026-10-05).
- Worst-corner checkbox removed from the feedback table: unwired in the shipped flow (only the legacy rule engine behind the weekend PDF export read it) (2026-10-07).
- Deletion runs in ordered batches (UI first, then files), targeted tests per batch, goldens once at the end, only on a committed tree (2026-10-05).
- AUTHOR-DECIDES items from the Q-SLIM audit are ruled one-word with the first UI batch proposal (2026-10-05).
- Decision frame stays as the recommendations carrier, slimmed per the audit (2026-10-05).
- Thesis explains the tool's RESULTS, not the tool; only what ships and runs enters the thesis; failed variants get one line (2026-10-04).

## Estimator (production, final)
- Sideslip: ekf_auto_pacejka with NIS gate and kinematic fallback; Dugoff config-only, not in the UI (2026-09-30).
- Tyre fit: one-pass per-session, per-axle Pacejka fit on kinematic-seed slip angles and 2-DOF axle forces, 2 Hz conditioning; pooled fits HARM as replacement, refit iteration HARM (no fixed point) (2026-10-05).
- Fit speed window: in-corner [p1, p99] speed per session (tyre_fit_auto.fit_speed_window_pct), all Pacejka modes, forced adaptation of the chair's band function (2026-10-04).
- Steering: Level-4 manufacturer table read wheel-angle vs steering-wheel-angle directly (the table's ratio column is LOCAL); two-wheel mean per Rajamani sec. 2.2 (2026-10-04).
- Production estimator and metric are FINAL; no load-dependent metric change (2026-10-04).
- Stability measure stays chair-conform: inertial regressand, no damping term, no schema bump -- default (c) after the Werner read found only a sign criterion (2026-10-05).
- Werner Eq. 4.3 v2 damping term: considered, not adopted (chair deviation, unmeasured benefit, route abandoned by Werner himself); yaw_damping.py, its test and diagnostic stay as the v1 record (2026-10-05).
- Iz stays the config Level-1 value; a mass-derived Iz is not shipped (2026-10-05).
- Tyre-fit legitimacy anchor: lateral-only axle-level Magic Formula fit per Pacejka secs. 1.2.1, 1.2.2, 4.3.1 (Eqs. 4.49 / 4.E19, combined slip as G_yk extension); large-Fx tails are the known bias (2026-10-05).
- MLA outing 5 stays excluded from Q-MU (frozen E3 rule, one-sample float32 artefact) (2026-10-05).

## Kerb exclusion (one concept)
- Kerb treatment is ONE user-facing concept, kerb exclusion; mask, time dilation and the 0.2 s filter-smear reach are internal workings (2026-10-05).
- Internally: the mask gates estimation (kerb_dilation_s = 0.10 s, time-based); the grade mark grades samples whose window the filtered strike reaches (0.20 s); nothing is deleted (2026-10-03).
- One switch corner_trace_display.kerb_exclusion_display: greyed (default) / excluded / off; app, thesis exports and the tyre-curve scatter agree; kerb-affected = mask or grade mark; near_strike_display key and "near-strike" / "smear" product wording retired, internal names stay (2026-10-07).
- ~~One caption everywhere: "Kerb-affected samples excluded/greyed (n = X, Y %); retained in all computed results."; the 0.2 s detail lives once in the methods text (2026-10-05).~~ [SUPERSEDED 2026-10-07: inaccurate since schema 12 (2026-10-05), when the verdict statistics began to skip kerb-affected samples.]
- One caption everywhere: "kerb-affected samples excluded from verdict statistics, retained in CS arrays", prefixed by the plot mode (Grey / Not drawn / Drawn normally) and followed by the count; the 0.2 s detail lives once in the methods text (2026-10-07).
- Thesis scripts: one flag per figure, full vs kerb-excluded, caption auto-stating it (2026-10-05).
- Kerb-gap display bridge stays on in traces and exports (display only) (2026-09-30).

## Thresholds
- Classification thresholds derive on the CLEAN population (kerb-affected samples excluded), full-population values reported beside; one population for classification and presentation (2026-10-05).
- Threshold derived_from phrase: "kerb-affected samples excluded" (2026-10-05).
- Thresholds derive on the best-available chain, not cap 1 (cap 1 is no user-facing path) (2026-10-05).
- CS thresholds: percentile rule on the clean pool, STRONG = p10, MODERATE = p30 per axle -- front -0.646 / -0.285, rear -0.404 / -0.131; the CS = 0 force-peak line stays display / interpretation only (2026-10-05).
- STRONG means "anomalous force loss beyond the quasi-steady characteristic (kerb-shown majority per audit; mechanism open otherwise)": it lies below the fitted curve's own floor on every pool session and axle (2026-10-05).
- stab_neg_thresh = 0: Werner's sign criterion (open-loop unstable), explicitly not criticality; verdict wording "open-loop unstable (Werner)", severity contextual; the -50 placeholder retired (2026-10-05).
- The classifier reads the same clean population its thresholds derive on: summarise_corners' CS statistics skip kerb-affected samples; the CS arrays keep every sample (schema 12) (2026-10-05).
- cs_margin = 0.11 retained: re-measured with the wheel-angle-column sweep, no proportional flip within +/-1 %; classification robust to input uncertainty at the derived thresholds (2026-10-05).
- Every threshold re-derivation re-runs the curve-floor check (inspect_curve_floor.py) -- the verdict meaning rides with it (2026-10-05).
- UI (with the UI batch): STRONG card text "anomalous force loss beyond the tyre's quasi-steady characteristic"; full sentence with the kerb-shown-majority qualifier in tooltip / caption (2026-10-05).
- Manual threshold override via Settings allowed, recorded config-side, restorable; never rewrites the derived default (2026-09-28).
- Seven-session threshold pool; Le Mans banned from scoring (different tyres), Dubai least-representative weight (2026-10-01).

## Product text and display (app check 2026-10-06)
- No literature names in the product: UI text carries no source names (verdict reads "open-loop unstable"); anchors live in code comments, PRODUCTION.md and the thesis (2026-10-06).
- Verdict card says condition, phase, number, severity -- short; no meta-sentences; robustness qualifiers go to the tooltip or vanish (2026-10-06).
- Shading budget, per panel: filled area only for the one background tint family (verdict context) and the one exclusion grey; everything else (events, envelopes) is lines or strips; no stacked filled bands (2026-10-06).
- The product describes what, not after whom: no source or model names in user-facing text ("fitted tyre curve") (2026-10-06).
- Corner-trace default view: CS front, CS rear, stability; selected lap full-strength over cross-lap p10 / p90 lines of all analysis laps; everything else opt-in (2026-10-06).
- Every mutable input is locked while an analysis runs (feedback table included); a file-identity guard discards a result whose file changed (2026-10-06).
- One lap-selection state in the trace windows: every drawn element follows the checked laps; hiding a lap hides all its marks (2026-10-07; the cross-lap envelopes' population is an open author question against the 2026-10-06 default view).
- An empty per-phase CS cell states why: kerb-affected samples excluded (with count) or too few samples (2026-10-07).
- Per-phase table Apex row shows the apex-region CS value the verdict reads; a fixed-slice value is shown only under its own distinct label, else not at all (2026-10-07).

## Loads and mass
- Measured Level-4 NBR wheel loads stay in the pipeline (voltage-referenced event zero entries, L2 zero); NBR zero entries stay in config (2026-10-05).
- c_session stays as live insurance in the reconstruction cascade (2026-10-05).
- Loads' consumer = ONE thesis-script g-g figure (per-corner ay-ax, phase-coloured, kerb-graded, measured-Fz occupancy), results phase, no app view; fy_norm persists for it (2026-10-05).
- Revisit condition (the only one): if the g-g figure is cut from the thesis, the loads ruling reopens (2026-10-05).
- App-side per-wheel load display surfaces beyond the summaries are UI-batch removal candidates (2026-10-05).
- Q-MASS dead: per-outing mass from the damper loads is not stable within a file (split-half up to 52 kg) (2026-10-05).
- Archive starting mass (fuel) is permanently unknown, about +/-40 kg; fuel load is a future session input; the cumulative ECU burn channel is authoritative for burn (2026-10-02).
- Front-vs-rear Fy/Fz is not a balance metric: with the Level-1 force split it reports load distribution (Q-MU) (2026-10-05).

## Deep CS tail
- Cause attribution counts only on criteria that exceed their base rate in matched random corner windows; the first spike-audit pass's az-alone and fallback-within-0.5 s criteria are withdrawn as evidence (2026-10-05).
- Proven estimator artefacts are lived with (no estimator surgery; the reference-sensor day is the fix); unproven ones are "no specific mechanism shown" (2026-10-05).
- Detector recalibration acts only on specificity-checked kerb-missed events (2026-10-05).
- Kerb detector VALIDATED on NBR JST 29: catches the strikes that drive the deep tail (25 events, all of the deepest); 4 of 58 demonstrable misses -> no recalibration, no new term; C2 1044-1057 m left-wheel pair is a watch item (2026-10-05).
- Deep-tail ledger closed: kerb-shown 25 + 4, car events 2, proven artefacts 3, residual ~50 % front / 36 % rear "incompatible with the session's quasi-steady tyre characteristic; specific mechanism not shown" (2026-10-05).

## Balance readout
- Understeer-gradient line SHIPS: one measured, beta-free session quantity (Rajamani sec. 3.2 Eq. 3.23, Level 2) with its 95 % range, one UI line beside the verdicts; config derived_from names Eq. 3.23 and the quasi-steady population provenance; schema bump "balance key added - measured understeer gradient, Level 2" (2026-10-05).
- K2 is a readout, never a constraint on the fit (2026-10-04).
- Steady-state gradient and per-corner verdicts measure different things (steady-state balance vs local axle saturation per corner and phase); both stand (2026-10-05).

- Event layer SHIPS as display + record only (2026-10-06): CS below the session's 0-30 deg curve floor (promoted curve-floor method, B C D normalisation as is), bridge gap 0.5 s, minimum duration 0.1 s = one CS regression window, labels kerb-affected / recurring (+/-10 m) / single; feeds no verdict and no recommendation; schema 13 (2026-10-06).
- Event thesis consumer scoped to corner and phase (the feedback form's resolution); the feedback form gains no lap tags (team instrument, frozen) (2026-10-06).
## Evidence and method
- Beta investigation closed (2026-10-08): washed-out kinematic seed and washout retained as a documented limitation (Bechtloff identifiability); D3 divergence re-entry (NIS window: keep state, P -> P0; hard bound: beta -> 0, yaw rate -> measured, P -> P0) and D4 tyre-curve plausibility gate implemented; phase 2 (re-derivation, goldens, schema bump) pending.
- Chair lineage of beta: the chair code consumes beta (measured, state-estimated or logged) and has no integration, washout or IMU compensation; SetupTool's integration and washout are SetupTool-authored and need their own anchors; the 2026-10-07 "chair-pattern drift handling" ruling is struck (2026-10-07).
- Provenance classes: (i) chair internal code / CSVs / README, cited only as "chair performance_analysis tooling (internal)", never committed or named; (ii) TUM chair-lineage publications (Werner 2021, Goblirsch et al., IFAC 2019, Betz et al. 2023), cited normally; (iii) general literature (2026-10-07).
- Approved criteria for estimator comparisons (alpha-robust): fit-internal mechanics, NBR A->B rake resolution, the driver's phase-differentiated statements, internal cross-lap coherence; CS physical bounds are a sanity screen only (2026-10-02).
- Production is the candidate, not the standard: "vs production" numbers track change, never quality (2026-10-03).
- Agreement metrics cannot tell self-consistency from truth; only data-anchored metrics arbitrate; relative verdicts valid, absolute calibration only (2026-10-05).
- Position-locked CS dips: protected from silent deletion, not quality anchors, not verdicts, magnitude-incompatible with the fitted model (2026-10-03).
- Design spec: exclude what is physically disturbed, grade what is statistically thin, delete nothing else; coverage loss is not an acceptable price (2026-10-03).
- Sanity screen (B >= 1, 1 < C <= 3, E <= 1) is a fit-shape rule, not physics (2026-10-04).
- Recorded negatives are cited with a scope line (data, conditions, reopen condition) (2026-10-01).
- Comparable evidence -> the simpler candidate ships (2026-09-30).
- Frozen frames: rules written before any number; conflict = record, never re-tune; gates escalate, never cancel (2026-10-02).
- The NBR A->B pair is a measured multi-change outing pair (line and map changes), not a single-change anchor (2026-10-02).
- Every estimator item in BACKLOG-PERMANENT leaves only by author ruling, never by metric (2026-10-03).
- Reference-tuning day (optical sideslip sensor) is an outlook recommendation, not a work item (2026-10-04).

- Channel census DATA-FINAL (Q-CHANNEL-AUDIT, 2026-10-06): no INDEPENDENT lateral-velocity, sideslip or heading measurement exists in any export (24 files, 4411 channels); the optical-sensor slot (corr_*) is pre-wired but dead; residual: VBOX_* / sclu_FA/RA_* present in raw metadata, absent from every export (raw format proprietary), team channel list requested (2026-10-06).
## Process
- One step at a time; proposal and implementation never in one turn; the author runs git (standing).
- Runs launch only on an explicit author EXECUTE / go; ambiguity = ask and wait (2026-10-02).
- Chat is lossy: every ruling lands in PLAN.md (and here) in the same turn (2026-10-01).
- Report-framing conflicts with author positions are surfaced, never resolved in the report's favour (2026-10-03).
- A finding that identifies a production defect carries a PRODUCTION CONSEQUENCE line (2026-10-03).
- Schema bump rule: any estimator-input change outside Settings bumps ANALYSIS_SCHEMA_VERSION with a dated reason (2026-10-03).
- Plot text: plain words, "cornering stiffness ratio" spelled out once, no internal codes; every render titled; plots_INDEX lists every folder (2026-10-05).
- Per-iteration render rule: every scored iteration renders its trace set before any score-based exit (2026-09-30).
- PRODUCTION.md describes settled production only: updated once per closed package (after its app check), never mid-package (2026-10-06).
- Diagnostics are disposable unless Referenced / Reproduces / Dependency, with the reason in diagnostics/README.md (2026-08-30).
- Full test suite only at an author-named boundary; settings tests never run concurrently with another test process (standing).
- Protected set stays untracked: docs/literature, docs/car_data, config/car_data.json, HANDOVER.md, docs/study, generate_handover.py (standing).
- Reports end RESULT + OPEN, plain lists, never tables; questions to the author only under OPEN (2026-10-05).
- Reporting discipline (hard): exactly ONE report per cycle, at the hard stop; no interim messages, status pings or waiting notes; full suite exactly once, immediately before that report, targeted test files only during work; stop before commit (2026-10-07).
- Never state a config value from memory; per-file channel units read from each file's header; Porsche offset convention converted before use (standing).
