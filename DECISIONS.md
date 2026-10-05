# DECISIONS -- standing rulings, current state only

One line per ruling. Current state only: superseded versions are not listed
here (their history stays in PLAN.md and thesis_notes.md). Maintained alongside
PLAN.md: every ruling that lands in PLAN lands here in the same turn, and a
line that changes is replaced, not appended. Distilled 2026-10-05.

## Scope and product
- Thesis centerpieces are fixed: CS, stability, recommendations; everything ships only insofar as it serves them (2026-10-05).
- Q-SLIM: every module, UI element, export and file justifies itself by a consumer in the shipped minimal flow or an anchor the author defends; unreferenced = removal candidate (2026-10-05).
- Minimal app: load an outing -> laps with in/out marked -> recorded data below; analysis runs with best-available defaults (2026-10-05).
- Removed from the app (pending the UI batch): accuracy-level chooser, tyre-model chooser, lap-selection UI, weekend PDF export (2026-10-05).
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
- One switch: greyed (default) / excluded / off; app and thesis exports agree; the near_strike_display key and "smear" wording are retired with the UI batch (2026-10-05).
- One caption everywhere: "Kerb-affected samples excluded/greyed (n = X, Y %); retained in all computed results."; the 0.2 s detail lives once in the methods text (2026-10-05).
- Thesis scripts: one flag per figure, full vs kerb-excluded, caption auto-stating it (2026-10-05).
- Kerb-gap display bridge stays on in traces and exports (display only) (2026-09-30).

## Thresholds
- Classification thresholds derive on the CLEAN population (kerb-affected samples excluded), full-population values reported beside; one population for classification and presentation (2026-10-05).
- Threshold derived_from phrase: "kerb-affected samples excluded" (2026-10-05).
- Thresholds derive on the best-available chain, not cap 1 (cap 1 is no user-facing path) (2026-10-05).
- CS thresholds: percentile rule on the clean pool, STRONG = p10, MODERATE = p30 per axle -- front -0.646 / -0.285, rear -0.404 / -0.131; they encode unusual depth for this car, the CS = 0 force-peak line stays display / interpretation only (2026-10-05).
- stab_neg_thresh = 0: Werner's sign criterion (open-loop unstable), explicitly not criticality; verdict wording "open-loop unstable (Werner)", severity contextual; the -50 placeholder retired (2026-10-05).
- The classifier reads the same clean population its thresholds derive on: summarise_corners' CS statistics skip kerb-affected samples; the CS arrays keep every sample (schema 12) (2026-10-05).
- cs_margin = 0.11 retained: re-measured with the wheel-angle-column sweep, no proportional flip within +/-1 %; classification robust to input uncertainty at the derived thresholds (2026-10-05).
- Manual threshold override via Settings allowed, recorded config-side, restorable; never rewrites the derived default (2026-09-28).
- Seven-session threshold pool; Le Mans banned from scoring (different tyres), Dubai least-representative weight (2026-10-01).

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

## Balance readout
- Understeer-gradient line SHIPS: one measured, beta-free session quantity (Rajamani sec. 3.2 Eq. 3.23, Level 2) with its 95 % range, one UI line beside the verdicts; config derived_from names Eq. 3.23 and the quasi-steady population provenance; schema bump "balance key added - measured understeer gradient, Level 2" (2026-10-05).
- K2 is a readout, never a constraint on the fit (2026-10-04).
- Steady-state gradient and per-corner verdicts measure different things (steady-state balance vs local axle saturation per corner and phase); both stand (2026-10-05).

## Evidence and method
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

## Process
- One step at a time; proposal and implementation never in one turn; the author runs git (standing).
- Runs launch only on an explicit author EXECUTE / go; ambiguity = ask and wait (2026-10-02).
- Chat is lossy: every ruling lands in PLAN.md (and here) in the same turn (2026-10-01).
- Report-framing conflicts with author positions are surfaced, never resolved in the report's favour (2026-10-03).
- A finding that identifies a production defect carries a PRODUCTION CONSEQUENCE line (2026-10-03).
- Schema bump rule: any estimator-input change outside Settings bumps ANALYSIS_SCHEMA_VERSION with a dated reason (2026-10-03).
- Plot text: plain words, "cornering stiffness ratio" spelled out once, no internal codes; every render titled; plots_INDEX lists every folder (2026-10-05).
- Per-iteration render rule: every scored iteration renders its trace set before any score-based exit (2026-09-30).
- Diagnostics are disposable unless Referenced / Reproduces / Dependency, with the reason in diagnostics/README.md (2026-08-30).
- Full test suite only at an author-named boundary; settings tests never run concurrently with another test process (standing).
- Protected set stays untracked: docs/literature, docs/car_data, config/car_data.json, HANDOVER.md, docs/study, generate_handover.py (standing).
- Reports end RESULT + OPEN, plain lists, never tables; questions to the author only under OPEN (2026-10-05).
- Never state a config value from memory; per-file channel units read from each file's header; Porsche offset convention converted before use (standing).
