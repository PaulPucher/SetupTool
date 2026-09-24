# Thesis Material Index

Writing-preparation index for the SetupTool bachelor thesis. Source of truth:
`thesis_notes.md` (read in full, all 20,285 lines / 302 headers, sequentially,
2026-09-23/24), cross-referenced against `PLAN.md`'s DECISION LAYER SPEC and
STATUS blocks. This file indexes — it does not summarize away, invent, or
editorialize. Every entry below is listed by its exact `thesis_notes.md`
heading text. Tags: **[result]** empirical finding, **[decision]** design/method
decision, **[limitation]** documented limitation, **[figure-source]** points to
a reusable diagnostics figure generator.

Coverage note: `thesis_notes.md` contains 302 Markdown headers. All 302 are
represented below (closely-related sub-phase headers of one multi-part work
package are grouped under a single index line where the source file itself
presents them as one continuous narrative — noted inline). One single-sentence
pointer header (`## Frame depth programme recorded in PLAN.md`, no independent
content beyond the pointer itself) is folded into Chapter 5 without its own line.

---

## 1. Method foundations
*(lineage: Milliken MMM / Hoffman / Werner / chair tooling / Segers; the deviation taxonomy)*

- **CS_ratio (cornering stiffness ratio) — Werner MA method** [decision] — production CS_ratio estimator; 2026-07-24 framing correction: adopts Werner's framework as-is, effective-Cα estimation is the necessary adaptation (no validated tyre model exists for the 992 GT3R).
- **Module 4b vs. the chair estimator: method-identical, not implementation-identical** [decision] — line-by-line comparison finds method-defining parameters identical, 8 structural implementation differences; precise framing established: "method-identical," not "source-verified-identical."
- **Fy yaw-moment term (Module 4a)** [decision][result] — 2-DOF force/moment balance replaces static split (Tier A, Milliken RCVD); psidd kept raw/fresh, deliberately not Module 5's filtered signal.
- **Yaw moment stability dMz/dbeta** [decision][result], superseded 2026-07-24 — sign convention corrected to Werner (2021) S2.2.3 (replacing an unsourced "Suzuka convention"); yaw rate excluded from regressors (multicollinearity).
- **Front/rear saturation and saddle-node concept anchors closed** [decision] — Hoffman et al. 2008 (p.136) verified by reviewer for front/rear→controllability/stability pairing; Ono et al. 1998 cited as motivation only, not implemented.
- **Completing Werner Eq. 4.3 — damping term via wheel loads** [decision][limitation] — Werner stopped at Mz measurement (no wheel-load sensors); WP5b is this project's sensor-enabled extension of his prior work.
- **Module 5 chair-basis alignment: s-anchored ridge regression** [decision] — centerpiece deviation-taxonomy entry; replaces time-anchored single-lap OLS with s-anchored, cross-lap-pooled ridge regression; raw-yaw-rate path = FORCED ADAPTATION (confirmed = chair's own last-resort fallback tier, not a simplification).
- **In/out-lap exclusion: two-leg rationale** [decision][result] — outlap leg reclassified FORCED (s-coordinate degeneracy); inlap leg stays DOMAIN IMPROVEMENT (cold-tyre stationarity).
- **Moving-speed mask: domain-improvement classification** [decision] — excludes stationary/pit samples; DOMAIN IMPROVEMENT, no chair-side equivalent.
- **Kerb/jump exclusion** [decision][result][limitation] — az deviation gate; z-down sign convention discovered empirically; DOMAIN IMPROVEMENT.
- **Dual-criterion corner detection** [decision][result] — OR-entry (steer OR ay)/AND-exit replaces a fixed steering threshold, physics-grounded (δ ~ L/R + understeer term).
- **No-steering-channel fallback: Tier B heuristic, not part of the method** [decision][limitation] — speed-minima valley-depth fallback; corrected an earlier "prominence threshold" overclaim.
- **Compound-corner finding** [result][decision] — AND-exit reveals a ~430 m double-apex complex; brackets >300 m flagged `compound_corner`.
- **Cross-lap corner identity via lap-distance clustering** [decision][result], superseded (kept for narrative) — apex-position interpolation + 1D gap clustering.
- **Overlap-fraction join criterion** [decision] — proportional overlap-length criterion replaces a fixed-metre gap tolerance; untested cross-track assumption documented.
- **Connected components + seeded splitting** [decision][result] — replaces a greedy sweep shown by full manual trace to fail; final Dubai structure 15 stable corners (later 14).
- **Moving-speed mask** — see above.
- **No-steering-channel fallback** — see above.
- **Limiter-based inlap reclassification** [decision][result] — Level-3 limiter-channel detection catches a STRUCTURAL misclassification a duration window could not detect in principle; thesis point on the accuracy cascade.
- **Small-decisions sweep** [decision] — CS fallback reference constants deleted (unconsumed at the time, later reintroduced with a named consumer); `s_m` time-anchored fallback deliberately not ported.
- **Deviation taxonomy for chair-comparison** [decision] — restates the CLAUDE.md three-way FORCED ADAPTATION / DOMAIN IMPROVEMENT / NEUTRAL ENGINEERING taxonomy and its rationale.
- **Vehicle parameterization is not a deviation** [decision] — states plainly that 992 GT3R parameterisation is never itself a taxonomy entry; defines the three parameter-category provenance rules.
- **Accuracy-level cascade** [decision] — the four-level (config→session→sensor→lookup) cascade as the project's general answer to shipping with imperfect data.
- **Accuracy-level registry consolidated** [decision][result] — registry extended to 11 nodes; two distinct `capped_by` mechanisms identified with code evidence ("chained-constant" vs "provenance-assumption").
- **Per-session accuracy resolution + global level cap** [decision] — highest-available-wins, never blended; pure vs method-ceilinged cascade distinction; accuracy cap is a viewing choice, not a reference-configuration change.
- **WP-C resolver end-to-end acceptance proof** [result][decision] — cap=1 output exactly equals no-setup-data baseline (float-identical), confirming genuine discarding not relabelling.
- **Steering ratio Level 1 → Level 4 lookup (WP-B)** [decision][result] — 21-row manufacturer table; PARAMETERIZATION NOT DEVIATION; zero verdict flips despite a real, measured distribution shift.
- **Analysis layer vs human layer for corner identity** [decision] — tool detects load events, humans think in named corners; three-layer track-robustness architecture.
- **Transparency over suppression** [decision] — `kerb_fraction` reported not filtered; a missing corner is an empty cell (information), not renumbering.
- **Reference-grid choice reviewed against the chair's fixed-rate resampling** [decision] — `ecu_speed`'s 50 Hz grid reviewed and deliberately retained (measured near-uniform, functionally equivalent to a fixed grid).
- **WP1 consolidation, Turn 1: canonical corner realization** [decision][result] — one canonical bracket+phase-set per stable_corner_id; a real smoothing-edge-effect bug caught by comparing to physical plausibility, not a clean diff.
- **WP1 consolidation, Turn 2: validation findings that motivated Turn 3** [result] — traces the C9/C10 pairing to pass-1's own union-find; inter-lap agreement tightened 10–100x for 13 of 14 corners.
- **WP1 consolidation, Turn 3: canonical boundary resolution** [decision][result] — new geometric post-pass truncates overlapping windows at the pooled |ay| minimum; C11 genuinely re-classified from medium to high speed.
- **WP1 arc closeout** [decision][result] — tri-state invariance (C8+C3 attribution unchanged across three states) is the arc's cleanest cross-check.
- **CS credibility diagnostics: kerb audit + filter sensitivity** (see Ch. 3) — cross-referenced here for its production-default-retention decision (2 Hz cutoff kept, now evidence-backed).
- **Corner-numbering display bug: wrong field, not wrong indexing** [result] — `stable_corner_id` vs `corner_number` conflation at one UI site; a good example of a corrected hypothesis (not 0/1-indexing as first guessed).
- **Aero downforce sign convention verified computationally** [result][decision] — algebraically and computationally confirmed the existing `Cl<0=downforce` convention was already correct and already documented; resolved, not a bug.
- **Citation cross-reference, modules/longitudinal_forces.py** [decision] — names two Rajamani Ch. 2 anchors that existed implicitly, gives them their own bullet.
- **RULE APPLICATION, Phase 2a citation sweep** [decision] — systematic application of the 2026-08-19 citation-location rule across modules/; no citation content changed, only relocated.
- **GT3 Paul Ricard export: diagnosis and fix** (Diagnosis / Fix / Verification) [result][decision] — parser-format bug plus a second, more dangerous latent unit-conversion bug found in the same investigation before it could ever fire; also cross-referenced in Ch. 6.
- **Fz-integration Phase 4: pit-limiter-based out/in-lap classification** [result][decision][limitation] — real, load-bearing lap-classification bug (v3's actual last lap silently contaminated the valid-lap population); premise checked and found false before trusting it.
- **Fz-integration Phase 5: wheel-speed plausibility guard + ABS-domain fallback** [decision][result][limitation] — a real sensor fault diagnosed and guarded, with two real calibration bugs in the guard itself caught before shipping.
- **v3 diagnostics, Part B1: corner-separation investigation, C17-C20 / C4-C5 / C8-C11** [result][limitation] — two clusters confirmed correct; one genuine algorithm-level finding (a seed-lap heuristic picks a single atypical lap's fragmentation as ground truth) — correctly escalated as a code, not config, question.
- **Part C1: majority-vote seed lap -- ATTEMPTED AND REVERTED** [decision][result] — a considered fix violated the Dubai hard constraint; root cause understood (two structurally identical situations need opposite resolutions); reverted in full rather than shipping a speculative fix.
- **Corner canonicalisation fix: representative-lap filtering** [decision][result] — a pre-registration trap caught before coding (a rounded quote would have made the proposed default a no-op); deliberately leaves seed-lap selection untouched, a minimal post-filter instead.
- **Deepening Phase 5: wing-graph check (read-only)** [result][limitation] — corrects a plural-framing premise (only one wing image exists); confirms the aero-split identifiability limit is NOT lifted by this source.
- **Deepening Phase 6: records + closure, package complete** — cross-referenced; the "Sensors break; the cascade handles it" design principle is tied to three concrete realisations within this one package.
- **WP-FD1+2: Frame depth Steps 1-2 — condition schema + damper motion evidence** (parts a–d) [decision][result] — sign-convention verification finds the REAL sign opposite an unverified placeholder, corrected same session (part b); reference-population failure discovered and reported as a real property of the phase-segmentation design, not a script bug (part c).
- **DECISION LAYER SPEC Phase A: registry/config schema + channel-census correction** [decision][result] — the thesis-worthy channel-census correction: a long-standing "no live tyre-pressure channel" config claim was true of every config file but never checked against the raw telemetry — a live channel family is found on both real sessions (the target-value gap itself stays open).
- **DECISION LAYER SPEC B7: three bridges** [decision][limitation] — follows an already-reviewed literature encoding target verbatim; finds and fixes a real sign-mapping bug in the shared generic bridge mechanism while implementing one of the three.
- **Phase C: splitter_offset direction convention resolved, correcting same-session inversion** [decision] — explicit provenance care distinguishing "resolved by elicitation" from "corrected because this session got it wrong" (the latter never happened).
- **Segers deep-dive bridge review: full document, pointer only** [decision] — full graded review of 8 Segers chapters against the registry, 32 candidates, nothing implemented; every REJECT confirmed by direct file census. (`docs/segers_bridge_review.md`, committable.)

**Figure sources:** none purpose-built for Ch. 1 alone (WP1/corner-detection findings are documented in prose with diagnostic-script references, not standalone kept figures).

**Literature anchors, Ch. 1:**
- Werner, F., *Analyse des Fahrverhaltens eines autonomen Rennfahrzeugs...*, MA thesis, Hochschule München, 2021 (supervisors Pfeffer/Hermansdorfer), 122 pp. — **verified**, full citation resolved 2026-07-24.
- Milliken & Milliken, *Race Car Vehicle Dynamics* (RCVD) — 2-DOF planar force/moment balance, load-transfer chapter — **page numbers TO VERIFY** (no PDF-render tool available across multiple sessions; same standing limitation).
- Hoffman, Stein, Louca, Huh (2008), *Int. J. Vehicle Design* Vol. 48 — front/rear saturation ↔ controllability/stability, p.136 §2 — **verified** by reviewer against primary source, 2026-07-26.
- Ono et al. (1998) — saddle-node bifurcation framing — cited via Hoffman et al. only; primary source **not obtained, TO VERIFY** if pursued further.
- Segers, *Analysis Techniques for Racecar Data Acquisition*, SAE International 2014 — ch.9 p.199 (pushrod/damper force → wheel load), ch.10 pp.221-256 (roll-centre split) — **verified** 2026-09-03 against `docs/literature/` excerpts. Extended coverage of chs. 5, 7, 8, 11, 13 and sec. 4.3 — see `docs/segers_bridge_review.md` (2026-09-22), 32 graded candidates, PROPOSED grade throughout.
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012 — see Ch. 2 anchor list.

**Open gaps, Ch. 1:**
- Werner method-delta three-column comparison table (adopted-as-is / deliberately-different-and-why / not-implemented-and-upgrade-path) — planned but never built.
- 992 GT3R official corner-weight/mass provenance for the constants table — still unconfirmed.
- Second-track validation of the 50 m clustering tolerance and the overlap-fraction "genuine same-corner pairs always overlap" assumption — untested beyond Dubai/v3 (same car).
- `cs_max_window_m` does not guard against corner-to-corner track-topology gaps, only corner length — a standing limitation flagged for a tighter-corner-sequence track.
- The C17-C20 (v3) corner-separation finding is left as a documented, unresolved limitation, not a solved item (Part C1 reverted).

---

## 2. Sideslip estimation arc
*(kinematic beta → linear observer rejected → nonlinear Dugoff/Pacejka EKF, refit-loop closure, NIS gate)*

- **GPS-course sideslip (beta_gps, WP5b(c)) attempted and shelved** [result][decision][limitation] — two iterations improve plausibility checks but decision-criteria metrics stay flat (r=-0.24); VERDICT: shelved.
- **Wheel-speed source characterization (WP-S1)** [result] — two independent channel families (not four); rear reads a constant rolling-radius offset, not traction slip; corroborated independently by GPS-speed.
- **Zero-slip offset: chain decomposition + mechanism search (WP-S3b/S3c)** [result][limitation] — reframes the near-zero-alpha-at-high-ay effect as geometric cancellation in the slip-angle construction; sets the EKF's explicit target.
- **WP-S4: Kalman sideslip observer (diagnostics candidate)** [decision][result] — linear KF on the bicycle model; circularity option 2 (fixed Caf/Car prior) chosen specifically to avoid alpha-derived circularity.
- **WP-S4b: observer self-consistency and the Cr_A inflation finding** [result][limitation] — the kinematic alpha's error is shown to propagate into the production CS_ratio estimate itself, not only into beta.
- **Sideslip sign check: physical validation of the observer** [result] — observer matches physical sign expectation at all 14 corners; kinematic is wrong at 2 racing-speed corners with no low-speed excuse.
- **WP-S5b: Kalman observer tuning outcome** [result][decision][figure-source: `diagnostics/plot_kalman_qr_ratio_sweep.py` → `diagnostics/plots/qr_ratio_sweep/`] — KF gain ratio-invariance proven; interior turning points found; a new transient-tracking check catches over-smoothing.
- **GPS-course sideslip as a potential arbiter -- not usable yet** [limitation] — recorded as a potential arbiter but its own shelving record rules it out on current data.
- **Observer saturation-detection failure: the decisive finding** [result][decision], status superseded by the entry below (kept, not struck).
- **Linear observer saturation-detection failure: why the tyre model must be nonlinear** [decision][result] — establishes the requirement for a nonlinear single-track Kalman filter with the tyre curve identified from this car's own data; circularity resolution planned before implementation starts.
- **WP-N1: Dugoff tyre model chosen + first-pass fit, identifiability finding** [decision][result] — Dugoff chosen over an unanchored ad-hoc form; first-pass fit superseded by WP-N1b (errors-in-variables attenuation).
- **WP-N1b: c_alpha refit from Module 4b** [decision][result][limitation] — sources c_alpha from Module 4b's own linear-regime population; circularity status stated explicitly in advance.
- **WP-N2: nonlinear Dugoff EKF, pass 0 (frozen parameters, no refit)** [decision] — analytic Jacobians; Fy-axle dependency identity proven (only two independent measured quantities behind "four" axle forces).
- **WP-N2 pass-0 run: NIS baseline, saturation coverage, and three convergent lines of evidence for slip-angle under-read** [result][limitation] — NIS dominated by MODEL error not tuning; Dugoff's no-peak shape limitation stated.
- **Circularity check: pass-0 EKF vs the rejected linear observer** [result] — partial, front-dominant independence; standing summary: not the linear observer's failure mode, not yet a demonstrated saturation detector.
- **Combined-slip limitation: rear exit-traction and front entry-braking false negatives** [limitation] — friction-circle physics predicts false negatives exactly where an engineer cares most; deliberately not pursued yet.
- **Max-|beta| excursion and the divergence monitor's short-run blind spot** [result][limitation] — diagnosed as most likely numerical, superseded in outcome by pass-1 recalibration; a threshold-provenance process lesson recorded.
- **WP-N2 pass 1: noise-model recalibration, derivation and 2-D sweep** [decision][result] — R redefined as total innovation uncertainty; C2 excursion gate passed with the correct interpretation stated explicitly.
- **Circularity and flag attribution at the calibrated setting** [result] — frozen-curve check is the decisive number (calibration moves slip angles further from, not closer to, the assumed curve).
- **WP-N2 pass 1: CS_ratio interpretability, linear-reference staleness hypothesis DISPROVED, and the WP-S4b reference-spread improvement** [result][decision] — flags NOT YET INTERPRETABLE; staleness hypothesis tested and disproved; threshold re-derivation deliberately deferred.
- **WP-N2 pass 2: EKF-sourced Dugoff refit -- proposal and pre-registered predictions** [decision] — first pass refitting from the EKF's own slip angles; convergence criterion, 4-pass cap, three named failure modes fixed in advance.
- **WP-N2 pass 2: refit results, prediction verdicts, convergence status** [result] — mu_fz prediction FAILED; c_alpha/mu_fz found COUPLED, not independently identified by fit order.
- **WP-N2 pass 3: pre-registered predictions, carrying forward the c_alpha/mu_fz coupling finding** [decision] — adds a ridge check to detect parameter-pair sliding.
- **WP-N2 pass 3: refit results, prediction verdicts, ridge check, convergence status** [result] — front c_alpha/mu_fz both flip sign (oscillation); ridge-sliding explicitly not observed.
- **WP-N2 pass 4: pre-registered predictions, the front-axle discriminating test** [decision] — explicit rule: per-axle convergence does not count; three-way decision rule fixed before pass 4 runs.
- **WP-N2 pass 4: rear mu_fz fit failure (failure mode 3), front oscillation verdict, arc STOPPED** [result][decision][limitation] — rear curve degenerates to pure-linear, structurally the same blind spot as the rejected linear observer; recommendation to stop, not run pass 5.
- **WP-N2 refit loop: NON-CONVERGENCE, rear degeneracy to a pure-linear curve, and the identifiability limit** [decision][result][limitation] — self-starvation mechanism identified; a pre-registered risk that materialised.
- **WP-N2 carry-forward decision: pass 1** [decision][limitation] — pass 1 carried forward by a provenance rule fixed before comparison, not by best outcome; carried-forward limitation stated for the write-up.
- **WP-N2 pass 1: final validation baseline for combined-slip comparison** [result][figure-source: `diagnostics/inspect_pass1_final_validation.py`, `pass1_final_validation_manifest.json`] — consolidates every established pass-1 check into one timestamped baseline.
- **WP-N2 Step 1a: pass-1 EKF wall-clock timing, before any wiring** (see Ch. 6) — EKF itself is cheap; CS estimator is the real cost driver, later addressed by WP-PERF.
- **WP-N2 Step 1b: wiring proposal, approval, and implementation** [decision][result] — EKF wired as a selectable, config-gated, default-off source; three verifications run including an empirically-checked (not assumed) resolved-parameter claim.
- **Combined-slip premise test: does the rear reach meaningful longitudinal utilisation on exit?** [result][limitation] — provisional kappa built; ~4% of exit/braking samples exceed the utilisation threshold; incidental bug found (entry_1_brake phase-boundary, fixed later).
- **Rolling circumference: three disagreeing numbers, none resolved**, PARTIALLY SUPERSEDED same date — original framing withdrawn; team-supplied and ABS-programmed figures are neither a measurement of the tyres actually fitted.
- **Combined-slip Dugoff: longitudinal stiffness (C_sigma) estimation method availability** [result][limitation] — chair HAS an estimator but needs per-axle Fx this project doesn't compute; availability-only.
- **Rolling-radius offset: speed-dependence check** [result] — rear flat at higher speed; front braking-only reads INCONCLUSIVE, honestly reported.
- **abs_Slip_FL/FR/RL/RR[%]: examined, does NOT sidestep the reconciliation question** [result][limitation] — examined and found NOT USABLE; a -100x sign-inversion hypothesis tested and rejected in a later correction.
- **Combined-slip arc: logged ECU slip and TC channels found; premise supported but weak** [result][decision] — ecu_slip_act is a genuine Level 3 logged slip source; TC intervention rate too low (68 samples) to be conclusive, cuts both ways.
- **entry_1_brake phase-boundary bug: mechanism, blast radius, and fix** (also Ch. 6) — see Ch. 6.
- **Pacejka load-normalised (mu) tyre fit -- Fz-integration Phase 2** (also Ch. 4) — see Ch. 4.
- **Observer/Kalman-filter beta estimation reviewed and not adopted** [decision] — early un-considered choice reviewed and deliberately retained (minimal-assumption method); later pursued at length in the WP-S/N series.
- **C9 negative-CS decomposition + zero-slip offset finding** [result][limitation] — major finding: a global, direction-correlated zero-slip Fy offset at every corner; CS_ratio<0 cannot currently be read as genuine beyond-peak saturation; the empirical motivation for the whole sideslip-methods-comparison arc.
- **### 3. WP-N3 (per-session-fittable, self-checking sideslip): unsupervised package** — package header, five phases.
- **Phase 1: washout cutoff sweep -- pre-registration / results** [decision][result] — reproduction check passed; four intermediate cutoffs all survive the disqualifying bound; "no cutoff dominates 0.05" did NOT occur.
- **Drift re-examined over time: single-checkpoint verdict superseded** [result][limitation] — a full drift-vs-time curve shows Phase 1's checkpoint was the MOST FAVOURABLE possible instant; strengthens the case for the auto-fit EKF over a better-tuned kinematic path.
- **Phase 2: one-shot per-session Dugoff fit + EKF chain** [decision][result] — automates pass-0/pass-1; a small archived-value discrepancy investigated and traced to the historical, never-scripted derivation, not to the new module.
- **Phase 3: Pacejka variant -- pre-registration / results** [decision][result] — Pacejka validates marginally better on aggregate metrics but WORSE on rear self-consistency; rear-identifiability prediction confirmed but honestly found NOT rear-specific after all.
- **Phase 4: NIS tyre-mismatch gate -- pre-registration / results** [decision][result][limitation] — health score cleanly separates healthy from all four mismatches; BOTH numeric pre-registered predictions failed, both causes traced.
- **Phase 5: consolidated report** [result] — full package close-out; explicit failed-predictions record.
- **Phase 2 (built before Phase 1, dependency order): NIS gate module** [decision][result] — reality-check finding: only 1 of 4 mismatch scenarios actually reaches "fail" at the provisional thresholds.
- **Phase 1: fit orchestration in the pipeline** [decision][result] — real bug found before wiring (manifest exposed raw not fallback-corrected beta).
- **PLAN.md STEP 3 (LS_ratio): unsupervised package, Phase 1 -- inputs / Phase 2 -- estimator** [result][limitation] — Phase 2's pre-registered prediction FAILED completely; root cause proven analytically (a structural sample-rate mismatch).
- **PLAN.md STEP 3 Phase 4 -- disambiguation check / run for real** [result][limitation] — first empirical combined-slip test; pre-registration falsified on every clause but the disambiguation method itself is validated by a genuine new finding (C3, traction-limited on all four laps).
- **PLAN.md STEP 3: 50 Hz min_samples adaptation / Phase 2 re-run** [decision][result] — rate-derives the sample-count minimum from the actual log rate instead of the chair's literal count; the follow-on prediction still fails, root-caused precisely.
- **Mechanism investigation: wholesale-negative CS_ratio under ekf_auto_pacejka** [result] — four findings; the "wholesale negative" reading is the MIN statistic amplifying a minority tail, not a genuine shift; small-window/high-R² overfitting identified as the mechanism.
- **v3 sawtooth mechanism investigation: corner selection, window stats, floor-fraction and alpha-character comparison vs Dubai** [result][figure-source: `diagnostics/inspect_v3_sawtooth_mechanism.py`, keep-reproduces] — cross-file validation of the small-window overfitting mechanism; one corner explicitly flagged a partial exception.
- **v3 Pacejka refit evaluation: one iteration from ekf_auto_pacejka's own beta -- VIABLE on this data, NOT shipped** [result][limitation] — encouraging on every metric checked but explicitly only shows the second pass is viable, not that a loop converges.
- **Phase 2 extension: Dubai confirmation + refit iterations 2-4, both files** [result] — a different non-convergence pattern (D grows every iteration, no plateau) on both files.
- **Refit-loop conclusion: structural non-convergence confirmed on two sessions, two failure directions -- DECISION** [decision][result][limitation] — major closing entry; identifiability argument given as the structural cause; external-reference census finds NO data on either session that could break it; explicit reopen condition recorded.
- **Fz-integration Phase 2: load-normalised (mu) Pacejka tyre fit -- STOPPED on the pre-registered plausibility gate** [decision][result][limitation] — a real bug caught before any real-session number was produced; 3 of 4 axle/session combinations implausible, correctly stops the package.
- **Fz-integration Phase 2 gate resolution** [decision][result] — the plausibility BAND, not the fit, corrected (consistent with ordinary tyre load sensitivity); a decisive cross-check finds one pair mixed, honestly reported as mixed.
- **v3 rear divergence dig, read-only -- CONCLUSION: LEGITIMATE LOAD EFFECT** [result] — three independent checks all point the same way; a real sign caveat recorded rather than smoothed over.
- **Fz-integration Phase 3: bounded refit loop under mu -- BOTH SESSIONS NON-CONVERGENT** [decision][result] — amended per-axle criteria; strengthens, not reopens, the earlier refit-loop closure even with a real measured load input.
- **Mu-fit re-evaluation with FR live** [result] — re-tests the load-sensitivity finding on cleaned inputs (two confounds since fixed); central finding reproduces essentially number-for-number.
- **NIS gate band decision: divergence-not-quality redesign** [decision] — at n=2 sessions the gate's job is redefined from quality-grading to divergence-only; the resulting wide band explicitly accepted as the cost of n=2 evidence.

**Figure sources, Ch. 2:**
- `diagnostics/plot_kalman_qr_ratio_sweep.py` → `diagnostics/plots/qr_ratio_sweep/` (gitignored) — heavy/light-smoothing lap-trace overlay.
- `diagnostics/inspect_pass1_final_validation.py` + `pass1_final_validation_manifest.json` — the frozen pass-1 combined-slip comparison baseline.
- `diagnostics/inspect_v3_sawtooth_mechanism.py` [keep-reproduces] — cross-file oscillation-mechanism figures.
- `diagnostics/inspect_step2_chair_plots.py` → `diagnostics/plots_step2/` — chair-comparable CS/tyre-curve/track-map figures per corner (see Ch. 3).
- `diagnostics/plot_sideslip_comparison.py`, `plot_slip_angle_comparison.py` — kinematic-vs-observer visual evidence (deleted in the 2026-08-30/08-20 diagnostics sweeps; findings preserved in-text only, scripts no longer present).

**Literature anchors, Ch. 2:**
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012 — Ch. 2 §2.3 bicycle model p.27, §2.6 yaw-rate/slip-angle model p.37 — **verified** (both confirmed visually by the user); Ch. 13.10 Dugoff eqs. 13.72-13.76 — **page TO VERIFY**; Ch. 14 Kalman application — **page TO VERIFY** (chapter itself confirmed).
- Kiencke & Nielsen, *Automotive Control Systems*, 2nd ed., Springer 2005 — "Vehicle Body Side Slip Angle Observer" section — **topic verified visually**, **exact section number likely PERMANENTLY TO VERIFY** (source PDF's body text does not survive extraction; printed-copy-only). Own innovation-testing/NIS-consistency section — **TO VERIFY**, named only as Option C in the (superseded) NIS-gate redesign proposal.
- Ulsoy, Peng, Cakmakci, *Automotive Control Systems* — sec. 14.3 p.263 (nonlinear vehicle model), sec. 14.1 p.258ff (operational significance of sideslip) — **verified by two independent readings**; Eq. 14.8 confirmed as a term-by-term match for this project's EKF balances.
- Dugoff tyre model — Rajamani Ch. 13.10 (see above); sign convention adapted, drops the literature minus sign to match this codebase's established positive Fy-vs-alpha slope.
- Pacejka Magic Formula (reduced 4-parameter) — general form, Rajamani Ch. 13 — **page TO VERIFY**; also cited as "chair performance_analysis tooling (internal)" for the reduced-form construction.

**Open gaps, Ch. 2:**
- The kinematic beta estimate's own zero-slip offset / circularity is never fully resolved — it motivates the entire EKF/refit arc, which itself closes as structurally non-identifiable without an external reference. **This is arguably the project's single most important honest limitation** (see Ch. 7).
- GPS-course beta (shelved) has a documented reopen condition (denser anchors, longer session) that neither real session meets; v3 is shown to have NO usable GPS-course channel at all.
- Combined-slip Dugoff extension (C_sigma / per-axle Fx) has no estimator in this project — availability-only finding, never built.
- The load-normalised (mu) tyre-fit plausibility question is resolved as a real load-sensitivity effect but is explicitly NOT a validated quantitative load-sensitivity measurement (no controlled load sweep).
- Whether a session with genuine external sideslip reference data would resolve the refit-loop's identifiability problem remains the named reopen condition, unmet as of the last session.

---

## 3. Cornering/longitudinal stiffness + thresholds
*(validity repairs, anchoring, worst-lap aggregation, MARGINAL annotation)*

- **CS threshold re-confirmation after Fy yaw term** [decision][result] — thresholds kept unchanged; the N=51 resolution argument established (exceedance counts, not fine percentiles, are trustworthy at this sample size).
- **Ground-truth alignment improved under the corrected Fy model** [result] — the session's only "strong" verdict disappears post-fix, more consistent with the driver report.
- **Two-signal AND-logic for severity classification** [decision][result] — "strong" requires CS collapse AND destabilising yaw together; asymmetric front/rear thresholds (57.2% rear weight).
- **Stability-threshold re-derivation for the chair-basis estimator** [decision][result] — STAB_NEG_THRESH -500→-50 Nm/deg, gap-selected against the new estimator's own distribution, not a re-tightening of the old one.
- **B1 exclusion ablation numbers** [result] — 94.8% vs 73.1% stabilising; explicit caveat against over-reading as cold-tyre evidence.
- **Pooled grid makes stability a per-corner, not per-lap-instance, property** [limitation][decision] — s-anchoring pools cross-lap samples; per-lap stability differences reflect grid-coverage noise, not real driving differences.
- **CS/stability thresholds re-confirmed after steering-ratio L4 upgrade** [decision] — the scale-vs-input-accuracy argument articulated (an input-accuracy upgrade is a different kind of change from an estimator rebuild); one ceiling-tied NaN migration explained precisely.
- **Threshold re-confirmation after WP1 consolidation** [decision] — third confirmation under the same standing argument, now against the largest single change yet tested.
- **Verdict-distribution re-check after WP1 consolidation** [result] — continues the same downward flagged-rate trend three independent corrections in a row.
- **WP1 open watch items, carried forward** [limitation] — the one genuinely open thread of the WP1 arc (C9's own start boundary never independently examined).
- **CS credibility diagnostics: kerb audit + filter sensitivity** [result] — extreme negative CS ratios shown predominantly kerb-clean; filter-cutoff sweep shows front is filter-dependent, rear is filter-robust and corner-specific; production 2 Hz cutoff kept, now evidence-backed.
- **C9 negative-CS decomposition + zero-slip offset finding** (see Ch. 2) — cross-referenced here as the direct trigger for the whole CS-credibility/sideslip-arc.
- **## 3. Validation results (Dubai sample...)** — reference figures section: beta range, alpha_f>alpha_r understeer signature, C_alpha front/rear matching weight split, stability median, Iz=2082 kg·m².
- **## 4. Level 1 limitations register** — see Ch. 7 (core reference list).
- **## 6. Open questions / to verify before writing** — see consolidated anchors/gaps lists.
- **PLAN.md STEP 3 (LS_ratio) Phase 1/2** (see Ch. 2) — cross-referenced.
- **Kerb-strike wheel-speed spikes: investigation** (see Ch. 6) — cross-referenced (LS_ratio input hygiene).
- **PLAN.md STEP 3 follow-up: C3 verified clean, LS plausibility guard implemented, mask widening quantified** [decision][result] — Part 1 checks C3 first and finds it clean; guard's whole-session footprint is narrow, read honestly (not spun as either "fixed" or "no issue").
- **Part A: Lap 2, C5 front/rear CS_ratio discrepancy -- CONFIRMED within-phase median washing** [result] — a NEW, earlier-layer instance of the same "CS_ratio-as-robust-statistic-loses-signal" theme already found once at the cross-lap layer.
- **Mechanism investigation: wholesale-negative CS_ratio under ekf_auto_pacejka** (see Ch. 2) — cross-referenced; the finding that directly opens the CS validity repair arc.
- **Pass-1 baseline independence, corrected** [decision][limitation] — self-correction: the frozen curve is independent of the CS window constants but one validation-script output section is NOT.
- **Phase-level validity diagnostic: apex_3's fixed window budget conflicts with the C4-vs-artifact distinction** [result][limitation] — C1-C4 all resolve to the SAME phase (apex_3) for a structural, not corner-specific, reason.
- **apex_3 structural finding: a fixed 11-sample phase, never large enough to clear its own local window footprint** [limitation] — standalone citable statement of the structural fact.
- **Gated Stage-2 recomputation (k=1.5 across all 5 phases): C1 recovers, C2/C3 partially recover, C4 does NOT survive -- pre-registration falsified for C4** [result] — a genuinely different dilution failure mode (wide surrounding phases smooth out C4's real, narrow event).
- **Persistence-length diagnostic: pre-registration NOT supported -- C3 shows the longest sustained below-zero runs in the entire session** [result] — a third statistic that also fails to separate genuine from artifact corners; an honest discrepancy with an earlier qualitative read flagged, not reconciled.
- **Ground-truth workup: per-run verdicts for the long-run corners -- C4 REAL (fold/peak, all 5 runs), C2 and C3 rear ARTIFACT (loop)** [result] — the strongest evidence in the whole investigation arc; an honest, unresolved tension with a separate LS_ratio-based C3 finding noted, not reconciled.
- **Geometric fold-vs-loop candidates evaluated: none of three cleanly separate the ground truth** [result][limitation] — each candidate's own failure mode diagnosed precisely rather than reported blind.
- **CS validity repair, part A, Phase 1: window-floor re-derivation** — SUPERSEDED same day (kept, not deleted); wrong statistical target (single-window variance, not phase-level median).
- **CS validity repair, part A, Phase 2: adaptive widening + cap implemented** [decision][result] — fixes a latent pre-existing bug in the old widening loop's own "did it succeed" check (structurally always false).
- **CS validity repair, part A, Phase 3: apex_region statistic implemented, verified against the recommendation engine** [decision][result] — distance-based replacement for apex_3's fixed window, verified end-to-end through the real recommendation engine, not just unit-level.
- **CS validity repair, part A, Phase 4: final distributions -- the wholesale-negative artifact is fixed, but C4's own established genuine signal is now DILUTED BELOW VISIBILITY at the worst-lap statistic** [result][limitation] — the single most important open item flagged before any anchoring proceeds.
- **CS validity repair, part A: full non-golden test suite run -- 2 unexpected failures, ONE shared root cause, traced and scoped** [result] — a previously-latent coupling (fit_session's own c_alpha depends on CS_ratio's linear-region flag), accepted as a documented consequence.
- **CS validity repair, part A, Phase 1 REVISION: floors re-derived against the phase-level MEDIAN, sample-rate corrected** [decision][result] — corrects the Phase 1 criterion; the chair's own literal "10" was always a 100 Hz-calibrated value, silently carried as rate-independent.
- **CS validity repair, part A, Phase 4 REVISION: the phase-median criterion PASSED ITS OWN TEST AND STILL FAILED** [result][limitation] — major methodological lesson: bootstrap relative-std measures precision, never accuracy; STOPPED explicitly rather than picking a new criterion unilaterally.
- **100 Hz time-base work package, Phase 0: adaptive common grid (50-100 Hz), refusal only below the hard floor** [decision][result] — a long-standing structural bug fixed (the pipeline's common grid was silently downsampling five 100 Hz channels for the pipeline's entire history); measured 5.24x (not 2x) CS-estimator cost increase.
- **100 Hz time-base work package, Phase 1: floor derivation, third pass** / **Phase 1 FINAL: direct real-data validation** [decision][result] — decisive methodological pivot away from bootstrap criteria to direct validation; converges on the chair's own original 0.1s window, simply mis-resolved at the wrong rate for the pipeline's history.
- **100 Hz time-base work package, Phases 2-3 / Phase 4: final distributions -- pre-registration LARGELY CONFIRMED** [result] — every partial match reported honestly, not rounded up; real-cornering windows quantifiably no longer floor-pinned.
- **100 Hz time-base work package: full non-golden suite result** [result] — one real off-by-one grid bug found and fixed; Dugoff-chain coupling escalates from drift to full degeneracy under the final floor.
- **CS validity repair, sign-off clarification round: 0.1s hypothesis tested and confirmed, 0.2s hypothesis tested and REJECTED -- final config value is the chair's own literal, unmodified default** [decision][result] — 0.2s explicitly recorded as tested-and-rejected (not merely superseded); parsimony wins.
- **CS validity repair, limitation: cs_max_window_m does not guarantee locality against corner-to-corner GAPS, only against corner LENGTH** (see Ch. 1).
- **CS validity repair, limitation: C4's own short genuine events are only partially detected by the worst-lap/apex_region statistic** [limitation] — a per-lap breakdown shows this statistic catches only 1 of 5 independently-confirmed genuine C4 events.
- **Threshold anchoring, Phase 1: confirmation run clean, byte-identical to threshold_anchoring_input.md** [result].
- **Threshold anchoring, Phase 2: four CS_ratio thresholds derived as candidates; stab_neg_thresh BLOCKED -- no negative population exists under ekf_auto_pacejka** [decision][result][limitation] — ships the MARGINAL-annotation-adjacent physical-anchor CS thresholds; stability threshold explicitly STOPPED, no value invented, per CLAUDE.md's own rule.
- **CS validity repair, pooled per-sample median re-measured at the final config, same statistic as the original pre-repair number** [result] — closes the Phase 4 gap; confirms the underlying population was never wholesale-negative.
- **Threshold anchoring + arc closure, Phase 5: golden regeneration, determinism check, full suite -- and a real, expected consequence: the live recommendation set drops to zero** [result] — a genuinely significant coupling: tighter thresholds + worst_lap aggregation now interact with the pre-existing consistency gate to zero the live recommendation set; reported as arguably correct, not a defect.
- **Threshold anchoring + arc closure, Phase 6/7/8** [decision][result] — diagnostics classification, two real test regressions found and fixed, formal arc closure; the run of estimator/threshold entries between "Threshold re-derivation deliberately deferred" and here is the CS/MARGINAL chapter's central arc.
- **Metrology Phase 1: verdict sensitivity map** [result][limitation][figure-source: `diagnostics/plots_metrology/{dubai,v3}_margin_distribution.png`, `_corner_sequence_marginal.png`] — quantifies a real reproducibility bound (~10 corner-phases per session flip under a sub-1% parameter perturbation), concentrated overwhelmingly in already-borderline corners, never in a confident STRONG verdict — directly motivates the MARGINAL annotation.
- **Metrology close-out, Phase 2 implementation: verdict-stability [MARGINAL] annotation, SHIPPED** [decision][result] — minimal-blast-radius design (string-append, no shape change); both relevant frame-evidence types capped, not just the one literally named.
- **Deepening Phase 1: per-outing corner weights -- premise corrections, STOP on verdict flips** [result][limitation] — striking correction: Dubai's own front fraction had been quoted from the config fallback throughout the project, not from the real, already-stored setup weighing; a tiny mass correction produces 4 real verdict flips, root-caused to CS_ratio's window-floor sensitivity.
- **Deepening Phase 4e: LS_ratio threshold groundwork -- diagnostic + proposal, NOT shipped** (see Ch. 2) — cross-referenced; explicit recognition of the SAME wholesale-negative pattern in LS_ratio before it is repaired.
- **Metrology extension Phase 2: LS_ratio validity repair** (see Ch. 2) — cross-referenced; applies the CS_ratio playbook to LS_ratio with fresh, independently-derived numbers.
- **LS-evidence work package: negative-population co-occurrence census** (see Ch. 2) — cross-referenced; resolves the LS_ratio genuine-vs-noise question with a clean phase-type split.
- **LS threshold decision: SHIPPED, phase-scoped** (see Ch. 2/5) — cross-referenced.
- **PLAN.md STEP 2: chair-comparable result plots, kinematic vs ekf_pass_1** [result][figure-source: `diagnostics/inspect_step2_chair_plots.py` → `diagnostics/plots_step2/`, 28 PNGs] — direct visual answer to the C9 question: rear-anomaly corners improve sharply under ekf_pass_1, C4's front saturation and C2's "knot" persist under both beta sources.
- **Cleanup/reliability/presentation pass, Phase 0: LS_ratio plausibility** (see Ch. 2) — cross-referenced (LS_ratio's shared-with-CS numerical-instability gap, confirmed NOT a SetupTool regression).
- **Fz-integration Phase 1 (finish): CS_ratio independence, axle-total proxy bug fix, before/after comparison** [result][decision] — the impossible requested comparison correctly abandoned; proves fz never reaches the classifier chain, both by code read and empirically.
- **Fz-integration final edit before commit: vertical_load_source default flipped to "measured"** [decision] — authorised specifically because verdict independence from Fz was PROVEN, not assumed, before the flip.

**Figure sources, Ch. 3:**
- `diagnostics/inspect_step2_chair_plots.py` → `diagnostics/plots_step2/` — 28 chair-comparable per-corner PNGs (velocity, instantaneous CS, track map, tyre curves).
- `diagnostics/plots_metrology/{dubai,v3}_margin_distribution.png`, `{dubai,v3}_corner_sequence_marginal.png` — verdict-sensitivity/marginal-corner figures.
- `diagnostics/inspect_corner_distribution.py` — the recurring worst-lap/worst-instance distribution script, re-run at every CS validity-repair checkpoint (not a "kept for a figure" script per se but the arc's own load-bearing measurement tool throughout).
- `threshold_anchoring_input.md` (repo root) — the frozen threshold-derivation input record.

**Literature anchors, Ch. 3:** none beyond those already listed in Ch. 1/2 (Werner, Milliken/Hoffman for the classification framework itself; CS_ratio's own window-floor/threshold work is Tier B, calibration-tunable, not literature-anchored by design).

**Open gaps, Ch. 3:**
- `stab_neg_thresh` (stability threshold) remains explicitly UNRESOLVED under `ekf_auto_pacejka` — Module 5 takes beta as a direct argument, unlike the proven-independent CS/Fz chain; no negative population exists to anchor a value against. A separate calibration marker (`stab_thresh_calibrated_for_sideslip_source`) tracks this as distinct from the [UNCAL] question.
- C4's own genuine, independently-confirmed short saturation events are only partially visible at the worst-lap/apex_region statistic (1 of 5 events) — a known, quantified limitation of the current aggregation, explicitly left open.
- The tightened thresholds + worst_lap aggregation now interact with the pre-existing recommendation-engine consistency gate to produce zero live recommendations on both real sessions — flagged as a new open question for the consistency gate's own calibration, not resolved.
- LS_ratio's own threshold is shipped phase-scoped (braking/turn-in trusted, exit-phase confidence-discounted) rather than as a flat verdict tier — the exit-phase evidence gap (near-silent TC activity on both sessions) is the explicit, named reopen condition.

---

## 4. Wheel loads + metrology
*(Segers chain, corner weights, aero split 25/75, mass/aero joint fit, FR gauge recovery)*

- **WP5b(b) phase 1: chair-parity vertical loads (Fz)** [decision][result][limitation] — chair-identical construction; empirical sign-convention checks both matched; placeholder provenance flagged explicitly (cog height, track widths, Cl=0.0).
- **WP5b(d): GPS speed cross-validation (validation only)** [result][decision] — k=1.01211 scale factor, tight and condition-independent; a strong, well-evidenced rolling-radius correction candidate, not yet applied.
- **Rolling circumference: three disagreeing numbers, none resolved** (see Ch. 2) — cross-referenced.
- **Pacejka load-normalised (mu) tyre fit -- Fz-integration Phase 2** [decision] — D=mu·Fz reparameterisation requiring measured per-axle Fz; plausibility band reported not silently enforced.
- **GT3_PRC_MLA-v3 census: per-channel-block layout, 100 Hz dampers** [result][figure-source: `diagnostics/inspect_prc_v3_sample_rates.py`, keep-reproduces] — damper channels present for the first time in this project, the capability gain enabling the whole damper package.
- **Damper package: wheel loads from pushrod/suspension-travel channels, Phases 1-6** [decision][result][limitation] — foundational metrology entry (Segers ch.9/10 anchor); critical data-quality finding (FR gauge corrupted whole-session, caught by the range gate); motion-ratio sign determined analytically; ARB sign convention resolved empirically.
- **Morning follow-up to the damper package: NIS window rate-correction, FR reconstruction, ABS consistency check** [decision][result] — Segers modal decomposition reconstructs a missing corner from its real axle-mate; single-wheel-event limitation illustrated directly in a real figure.
- **Wheel-load showcase, ground-truth check, and closing the reconstruction's aero gap** [decision][result][limitation] — elegant ground-truth design (drop a known-good corner, reconstruct, compare); traces the ~25% axle-total bias to two specific static-model gaps (mass, aero); an 80%+ reduction achieved and honestly reported alongside a NOT-pre-registered side effect (front-axle residual worsens).
- **Session-measured split fractions -- the third and last term of the session-correction set** [decision][result][limitation] — completes the three-term session-correction set; honestly reports a pre-registration failure (the "isolated" test was never actually isolated) and a real, unresolved double-counting interaction between the front mass split and the still-placeholder aero split.
- **Reconstruction refinement chain: closing note** [decision][limitation] — formal closure: the remaining residual is an IDENTIFIABILITY LIMIT (straight-line data cannot supply a 4th independent quantity), not an open bug; two concrete data requirements named to lift it.
- **v3 sawtooth mechanism investigation** (see Ch. 2/3) — cross-referenced.
- **Frame-Stage-2 Phase 0: FR damper-force gauge forensics, re-examined full-session, not the prior 9-point sample** [result][limitation][figure-source: `diagnostics/inspect_v3_fr_gauge_forensics.py`, keep-reproduces] — meticulous re-examination upgrades a 9-point sample verdict to a full-session census; a precise new verdict category introduced ("miscoded, mechanism identified, absolute value not recoverable from this data").
- **Frame-Stage-2 Phase 1: vehicle-speed forensics, ecu_speed cross-plot, synthetic reference, C13 substitution test** (see Ch. 2) — cross-referenced (found a genuine ecu_speed glitch, exonerated speed as the sawtooth mechanism).
- **Deepening Phase 1: per-outing corner weights -- premise corrections, STOP on verdict flips** (see Ch. 3) — cross-referenced.
- **Deepening Phase 2: FR gauge decoding correction, SHIPPED** [decision][result] — pure additive-offset decoding, K derived empirically against the now-real outing weighing; a new, generic, evidence-gated per-channel decoding-correction mechanism shipped; an unplanned, larger axle-asymmetric mass residual surfaced and explicitly not chased further.
- **Deepening Phase 3: ABS channel-mapping decision** [decision][result] — trusted-channel decision generalises across both real sessions in two different specific ways, corroborating the user's own resolution.
- **Metrology Phase 3: rear axle-total residual decomposition** [result][limitation][figure-source: `diagnostics/plots_metrology/{dubai,v3}_residual_vs_speed.png`] — major finding: measures a real 25%/75% front/rear aero split, cross-session-confirmed; roll/geometric transfer structurally ruled out; a smaller residual explicitly left unresolved between two equally-plausible mechanisms.
- **Metrology close-out: aero_front_fraction shipped, mass/aero double-counting found** [decision][result][limitation] — ships the 25% split; the pre-registration EXPLICITLY FAILS on the ground-truth check and is root-caused to a separate, now-quantified double-counting flaw; the more accurate number is kept, the regression reported honestly as a symptom of the other bug.
- **Metrology close-out: measured front/rear aero split, engineer question** [result] — standalone citable 25/75 measurement, explicitly framed as single-car/two-session evidence, posed as an engineer-facing confirm-or-contradict question.
- **Metrology extension Phase 1: mass/aero double-counting fix, ACCEPTANCE CLEARED** [decision][result] — single joint regression replaces two overlapping fits; acceptance table clears both conditions with a large margin.
- **Mu-fit re-evaluation with FR live** (see Ch. 2) — cross-referenced.
- **Segers deep-dive bridge review** (see Ch. 1) — cross-referenced (aero coast-down test protocol proposed as an independent cross-check of the measured 25/75 split).

**Figure sources, Ch. 4:**
- `diagnostics/plots_v3/wheel_load_comparison_C12_lap8.png`, `wheel_load_reconstruction_C12_lap8.png`, `wheel_load_reconstruction_ground_truth_lap8.png`, `wheel_load_showcase_fastest_lap8.png`, `wheel_load_showcase_heavy_braking_zoom.png` — static-vs-damper-vs-reconstructed Fz comparison figures.
- `diagnostics/plots_metrology/v3_residual_vs_speed.png`, `dubai_residual_vs_speed.png` — the aero-split measurement figures.
- `diagnostics/inspect_fz_before_after.py` → `diagnostics/plots_fz_integration/{dubai,v3}/` — per-session static-vs-measured Fz corner traces.
- `diagnostics/inspect_v3_wheel_load_showcase.py`, `inspect_v3_reconstruction_ground_truth.py`, `inspect_dubai_wheel_load_validation.py` — validation/ground-truth scripts underlying the above figures.

**Literature anchors, Ch. 4:**
- Segers, *Analysis Techniques for Racecar Data Acquisition*, SAE 2014 — ch.9 p.199, ch.10 pp.221-256 — **verified** 2026-09-03 (see Ch. 1 full entry). This is the module anchor for `modules/wheel_loads.py` in full.
- Milliken & Milliken RCVD — load-transfer chapter — **page TO VERIFY** (same standing limitation as Ch. 1/2).

**Open gaps, Ch. 4:**
- Aero front/rear split (measured at 25/75) is explicitly single-car, two-session evidence only — posed as an open engineer confirm-or-contradict question, not validated against a windtunnel/CFD figure.
- Pushrod zero-offset calibration procedure and UI entry point — none exists; flagged open.
- Which `kinematic_variants_front/rear` roll-centre variant is actually fitted on either real session — unconfirmed, used as a Level-1 estimate regardless.
- A proper tyre dynamic (loaded) radius measurement to replace the static-circumference-derived unsprung-height estimate — not done.
- The FR gauge's own absolute decoding constant is explicitly stated as NOT recoverable from this session's data alone — a calibration reference or a repaired-gauge session would be needed.
- Wiring `wheel_load_damper`/the session-corrected axle-total model into `modules/accuracy_resolution.py`'s dynamic cascade and any real UI/pipeline consumer — deliberately out of scope throughout, still unwired.
- The remaining, smaller rear axle-total residual (motion-ratio table region vs genuine session-to-session static condition differences) — explicitly stated as not distinguishable with the evidence at hand.
- ecu_speed's own isolated, real, previously-undocumented glitch (Frame-Stage-2 Phase 1) — found but not investigated further, flagged as an open question.

---

## 5. Decision layer
*(the six-stage spec, registry, census findings, elicitation record, generated figures)*

- **Recommendation engine: median-of-medians aggregation + classifier reuse** [decision] — privileges repeatable behaviour; rules reuse the identical classifier the stability grid uses, a structural guarantee against disagreement.
- **Consistency-gate feedback override** [decision][result] — strong unprompted feedback can bypass the multi-lap repeat requirement under two independent floors.
- **Repair turn: feedback encoding unification + severity floor** [decision][result] — canonical feedback encoding audited against all 26 rules; a real direction bug found and fixed in the consistency-gate override.
- **Undrivable tier: lap-level cell matching** [decision][result] — bug found (aggregate-level phase pinning masked a real per-lap pattern); ruling generalises the fix to search across every candidate rule and lap.
- **Data and driver as co-equal evidence sources** [decision] — data/driver/both trigger types treat classifier output and driver feedback symmetrically; conflicts surfaced, not suppressed.
- **Verdict vocabulary** [decision] — deliberately limited to understeer/oversteer/unstable yaw/ok; engineering jargon tested and rejected.
- **Setup-parameter registry design** [decision] — five-concern schema whose separation caught a real direction-semantics error (tc_lat/tc_lon); a sixth axis (`value_source`) added after a real modelling mistake.
- **TC LAT for power-induced understeer - engineer rationale resolved** [decision] — resolved via load-transfer mechanism, not a convention contradiction.
- **WP2b-2: decision-matrix rule engineering** [decision] — 26 elicited rules supersede 7 provisional seeds; escalation-order kept deliberately separate from change_effort; a real feasibility limitation (unfilled-vs-genuine-zero setup fields) accepted rather than fixed.
- **TC LAT / TC LON channel-name candidates found (registry thread, not combined-slip)** [result] — candidate names found by keyword sweep, identified by name only.
- **Matrix v2 review round** [decision] — new provenance grade introduced; two garbled v1 answers decoded, illustrating that a precise-looking elicited value can encode low, not high, confidence; SITUATIONAL CLASS introduced as an engineering principle.
- **Phase 2 (built before Phase 1, dependency order): NIS gate module** (see Ch. 2) — cross-referenced.
- **Phase 3: UI mode selection and status** (see Ch. 6) — cross-referenced.
- **DECISIONS BATCH, Phase 2b: lap-filter UI choice removed** (see Ch. 6) — cross-referenced.
- **Production sideslip source set to ekf_auto_pacejka** [decision] — declares the production default; explicitly states verdict thresholds are NOT yet re-derived for it, so [UNCAL] correctly shows by default on every fresh outing.
- **Decision-matrix frame, Stage 1: evidence/candidate/scoring layers, one worked scenario end to end** [decision][result] — the first substantial decision-layer precursor; every provenance grade computed, not asserted; a real work-order factual claim checked and found false, then correctly re-elicited from the user rather than invented.
- **NIS gate band decision: divergence-not-quality redesign** (see Ch. 2) — cross-referenced.
- **Frame-Stage-2 Phase 2: intervention-channel survey, classification + co-occurrence** [result] — explicit three-way USABLE-NOW/READ-AND-RECORD/UNCLEAR classification; one channel family explicitly flagged UNCLEAR (four competing candidates, genuinely unresolved).
- **Frame-Stage-2 Phase 3: decision-frame Stage 2, full migration** [decision][result] — full migration of the 39-rule engine into the three-layer frame; a genuinely new conflict resolver built; parity verified via a diagnostic-only stress test against real forced old-engine output, zero discrepancies.
- **Deepening Phase 3: ABS channel-mapping decision** (see Ch. 4) — cross-referenced.
- **Deepening Phase 4: decision-frame deepening (a-d)** [decision][result] — parameter windows filled with two real latent-crash risks caught; interaction table grown 9→43 entries, several parameters deliberately NOT extracted with stated reasons; ABS masking bridge and driver-feedback magnitude weighting both wired as confidence discounts, not hard rules.
- **Deepening Phase 4f: end-to-end frame output, before/after both real sessions** [result] — candidate identity proven stable; every score change traced to a specific, explainable mechanism.
- **Metrology close-out, Phase 2 implementation: verdict-stability [MARGINAL] annotation, SHIPPED** (see Ch. 3) — cross-referenced.
- **LS threshold decision: SHIPPED, phase-scoped** [decision][result] — new phase-conditioned "ls_threshold" evidence type, deliberately placed outside the classification block so it can never reach the severity colour; confidence discount reuses existing MIN-cap machinery with its own reopen condition named in provenance.
- **Literature-bridge work package: coverage check and ride-height platform bridges** [decision][result][limitation] — a real premise over-specification resolved by user decision; the real content is an architectural coverage-check finding (interaction_table entries can never generate a candidate on their own — several registered parameters have literally no recommendation path); a real bug caught in the package's own first-draft test.
- **Generic lever-bridge candidate mechanism (BACKLOG item H): shipped** [decision][result] — closes the coverage gap the literature-bridge package found; a sign-flipped acceptance-case contradiction found and corrected against the config's own authoritative sign convention; a real, latent, pre-existing KeyError bug found as a byproduct of exercising a previously-untested path.
- **Frame candidate census: single-lap evidence confirmed on both real sessions** [result][figure-source: `diagnostics/inspect_frame_candidate_census.py`, keep-reproduces] — foundational finding: zero corner_verdict/matrix_verdict evidence items on either real session rest on more than one repeating lap; directly justifies the next package's deliberate design decision.
- **WP-FD1+2: Frame depth Steps 1-2** (see Ch. 1/4) — cross-referenced; condition-schema mechanism ships.
- **DECISION LAYER SPEC Phase A** (see Ch. 1) — cross-referenced.
- **DECISION LAYER SPEC B1 design resolution: lever-status granularity** [decision] — resolves a real spec-text ambiguity by checking which reading the existing architecture already commits to.
- **DECISION LAYER SPEC B2 design resolution: feedback-only routing mechanism** [decision][limitation] — rejects one plausible mechanism on principled grounds; a real, honestly-reported coverage gap found (zero click-class levers currently eligible).
- **DECISION LAYER SPEC B4 breadth design resolution** [decision][limitation] — two candidate readings tried and rejected; surfaces an independent structural finding (the evidence layer is blind to healthy corners by construction).
- **DECISION LAYER SPEC B5/B6 implementation notes** [decision][limitation] — B5 fully spec-derivable, explicitly directional; B6 reuses an existing outcome type; the same "complete but currently inert" gap shape as B2.
- **DECISION LAYER SPEC B7: three bridges** (see Ch. 1) — cross-referenced.
- **WP-DL Phase B close-out: census re-run, new baseline** [result] — every count-change class explicitly classified rather than accepted in aggregate.
- **Phase C: splitter_offset direction convention resolved** (see Ch. 1) — cross-referenced.
- **DECISION LAYER SPEC C1: scoring-term fold, phase_importance vs effect_class category split** [decision] — resolves a real spec/code term-count mismatch by a reasoned category argument (problem property vs lever-fit property), not by convenience.
- **WP-DL Phase C close-out: C2/C3, census STOP investigated and resolved, new baseline 6/21** [decision][result] — a real candidate-count change triggers an automatic STOP; investigated and traced precisely before accepting the new baseline.
- **Phase D: D6 top-line rendering rule resolved** [decision][result] — the UI spec's own illustrative examples don't generalise to over half of real candidates; one genuine registry gap found and distinguished from the broader enum-lever class; a sub-item cleanly and explicitly deferred.
- **Phase D feedback round, ITEM 1: display-layer grouping, max-score, user-elicited** [decision][result] — user-visible duplicate-row bug confirmed reproducible on the other session first; group score is MAX never summed, argued via a precise category distinction against the breadth term.
- **Phase D feedback round, ITEM 2: driver-feedback path diagnosis** [result][limitation] — exemplary three-leg trace of a user "nothing happened" report; one leg's diagnosis corrects a possible mis-read of the spec itself, not a code bug.
- **Item (a) term listing + eligibility gate amendment** [decision][result] — a shortlist-absence finding traced precisely to a phase-mismatch in the eligibility gate, not a scoring-term effect; after the fix, a byte-identical re-run is investigated (not assumed a failure) and explained by a second, independent, physically sound reason.
- **WP-DL Phase E: decision-layer figures generated from live config** [decision][result][figure-source: `diagnostics/generate_decision_layer_figure.py` → `diagnostics/plots_decision_layer/`, keep-reproduces] — figures read only live config/production classifier so they cannot drift; a real stale config note found by the audit itself; a second real bug found one phase later by cross-checking against an earlier, more authoritative notebook entry.
- **WP-DL Phase F close-out: config note fix, thesis_notes completeness check, census stability, full suite** [decision][result] — full documentation completeness audit of the entire multi-week package, one real gap found and fixed same turn.

**Figure sources, Ch. 5:**
- `diagnostics/inspect_frame_candidate_census.py` [keep-reproduces] — candidate-inventory census, re-run as the byte-stability regression bar at every decision-layer phase boundary.
- `diagnostics/generate_decision_layer_figure.py` → `diagnostics/plots_decision_layer/six_stage_flow.png/.pdf`, `lever_coverage_table.png/.pdf` — the governing six-stage-spec flow diagram and the 35-row lever-coverage table, both generated live from config.

**Literature anchors, Ch. 5:** none directly (the decision layer is Tier C/product design plus Tier B config-driven mechanics; its lever-bridge content inherits Segers anchors already listed in Ch. 1/4).

**Open gaps, Ch. 5:**
- **Elicitation item 10** (B2 feedback-only routing): zero click-class levers currently carry a helping-sign interaction_table entry on either tendency axis — the mechanism fires zero candidates on real data until this is filled. Still OPEN as of the last session.
- **Elicitation item 4** (brake_bias channel identity/direction, 4 candidates, 2 scales) — unresolved throughout; brake_bias direction convention itself IS resolved (standard, forward=more front) but the channel mapping is not.
- Splitter_offset per-direction hard/soft edge asymmetry — a known, explicitly-flagged simplification in the B5 window-edge check, not rebuilt.
- B6 contradiction detection and several lever_bridges condition types are fully implemented and tested but currently completely inert on real data — no lever_bridges entry yet declares a `damper_motion`/`intervention_abs`/`intervention_tc` condition.
- The literature-bridge coverage-check package's own larger finding (springs_front/rear candidate-bridge coverage, later closed by BACKLOG item H) — camber_rl/rr carry the identical gap and were explicitly out of that package's scope; still open.
- Display-cutoff score threshold, all six cost_function weights, and most `parameter_windows` spans remain project-lead-elicited PLACEHOLDERS, explicitly not yet compared against real engineer judgement (Stage 2 calibration item).
- Tyre-pressure target window: a real, live pressure channel is now confirmed present on both sessions (Phase A channel-census correction), but no target value exists anywhere in the project — the check-only plausibility mechanism stays silent.

---

## 6. Engineering quality
*(graceful degradation cascades, regression suite, byte-identity perf work, cache design, profiling)*

- **Repo cleanup: pass_2-4 block deletion + dead-diagnostics sweep** [decision][result] — a "stop and report instead" case correctly triggered by the disposal rule's own stop condition; two files caught as load-bearing derived_from citations, correctly withheld from deletion.
- **Sensors break; the cascade handles it** [decision] — flagship cross-project design-principle entry naming a pattern realised independently across three separate mechanisms; later shown to recur a further three times in the Deepening package alone.
- **WP-N2 Step 1a: pass-1 EKF wall-clock timing, before any wiring** [result] — the EKF is cheap; the unexpected finding is that estimate_cornering_stiffness is 86% of total runtime, later addressed by WP-PERF.
- **WP-N2 Step 1b: wiring proposal, approval, and implementation** (see Ch. 2) — cross-referenced.
- **Regression test suite established (tests/)** [decision][result] — first real regression suite, explicitly framed as regression not correctness tests; a real prior-claim correction found while building it (stored vs full-precision cog position).
- **entry_1_brake phase-boundary bug: mechanism, blast radius, and fix** [result][decision][limitation] — major engineering-robustness case study: a first fix attempt FAILS and is caught only by a mandatory external physical cross-check, not distributional plausibility; a second, inherited-lookback risk MEASURED (not assumed) to actually occur, promoting a hardening step to required; general lesson stated for the whole project.
- **Production impact of the fix, and a structural finding about CS_ratio aggregation** [result][limitation] — structural finding more significant than the bug itself (ceiling-clip + 4-lap median discards real single-lap signal as noise); false-negative hypothesis left explicitly inconclusive for a stated structural reason.
- **Documentation/comment polish pass** (4 phases) [decision][result] — Phase 3 needed zero edits, a positive data point on the durability of the comment discipline; Phase 4 reconciled a real coordination risk from file-disjoint parallel sub-agent work.
- **WP-C resolver end-to-end acceptance proof** (see Ch. 1) — cross-referenced.
- **Full channel census + targeted verification (2622 channels)** [result] — corrects a UTF-8 decoding bug; definitively confirms no hidden sideslip sensor exists anywhere in the log.
- **Analysis layer vs human layer for corner identity** (see Ch. 1) — cross-referenced.
- **Transparency over suppression** (see Ch. 1) — cross-referenced.
- **WP1 arc closeout** (see Ch. 1) — cross-referenced.
- **PDF layout rework: shared strip renderer** [decision][result] — two real inventory findings drove the design (drifted field sets between two PDF documents; a mismatched "schematics" premise against the real code); three real layout bugs found by rendering and looking, not by inspection alone.
- **Splitter/diffuser measurement points, Phases 1-4** [decision][result] — followed an existing reshape-pair precedent rather than inventing a new one; a rendered-and-rejected UI variant (values-only row) removed entirely rather than left as an unused path.
- **Position re-extraction against the user's annotated reference** [decision][result] — positions extracted programmatically (pixel clustering), not eyeballed; a real Flowable-sizing overflow bug found and fixed proactively on both renderers.
- **Ship-readiness cleanup, Phases 1-6** [decision][result] — dangling-reference healing distinguishes historical narrative (left untouched) from live pointers (annotated); a background agent's transient config corruption caught and fixed by independent verification, not trusted on say-so.
- **Out-of-scope emergency fix: app-breaking SyntaxError in core/config_loader.py** [result][decision] — an initial wrong provenance claim explicitly corrected same-day; the fix itself is the one deliberate, explicitly-authorised deviation from that package's own "no functional changes" rule.
- **Second diagnostics sweep: full-inventory classification** [decision][result] — a near-miss caught before it became a real breakage (an internal cross-import missed by the first-pass reference check) directly amended the CLAUDE.md disposal rule's own "Dependency" category.
- **GT3 Paul Ricard export: diagnosis and fix** [result][decision][limitation] — a silent-empty parse failure PLUS a second, more dangerous latent unit-conversion bug found in the same investigation before it could ever fire; a known residual gap explicitly reported, not fixed (a scope decision, not an oversight).
- **PLAN.md STEP 3 (LS_ratio) Phase 3: pipeline and UI** [decision][result] — a genuine self-correction: an earlier draft of the same entry overstated what its own verification script actually covered, withdrawn and replaced with a real end-to-end check.
- **Corrections round 3 follow-up: dialogs, corner-figure track map, setup-sheet PDF, recommendation audit** [decision][result] — a background-agent worktree-isolation false report caught by verifying against the main tree first, with an explicit lesson recorded.
- **entry_1_brake phase-boundary bug** — see above.
- **Regression test suite established** — see above.
- **Kerb-strike wheel-speed spikes: investigation** [result][limitation] — rear wheels ring down far longer than the current kerb-mask dilation; explicitly rules out a naive hybrid kerb detector because it would erase the very traction-limited signal the package exists to surface.
- **Threshold anchoring + arc closure, Phase 6/7** (see Ch. 3) — cross-referenced.
- **Frame-Stage-2 Phase 0/4** (see Ch. 4/5) — cross-referenced.
- **Pipeline wall-clock timing, per-stage, both real sessions** [result][figure-source: `diagnostics/inspect_pipeline_wall_times.py`, keep-reproduces] — measurement-only; cost concentrated in exactly three named functions (>97% of total on both sessions); explicitly declines to force reconciliation with an informally-recorded prior figure.
- **Phase D feedback round, ITEM 3: cache-miss diagnosis** [result][limitation] — a suspected package structurally proven NOT the cause via exhaustive diff; three concrete, ranked alternative causes named for the user.
- **Cache-miss reproduction attempt, WP5 identity check** [result][limitation] — escalates from code-read to a real, headless reproduction; could not reproduce a miss; constructs a concrete sequence that WOULD reproduce one, tied to an already-known UI persistence gap.
- **WP-CACHE Phase 1a groundwork: the "empty hull" finding, backfilled** [result][limitation] — a striking, explicit application of the channel-census-rule's SPIRIT to the project's own prior claims about itself: a session-start briefing cited a notebook entry that does not exist; verified true by code, then backfilled.
- **WP-CACHE Phase 1: sidecar implementation + Phase 1e real-data measurement** [decision][result] — atomic writes, explicit pickle trust-boundary documentation, graceful degradation on write failure; real (not estimated) size/timing numbers measured against pre-set gates.
- **WP-CACHE Phase 2: Analyse-button contract + acceptance-condition smoke test, real data** [decision][result] — an unusually transparent five-consecutive-failed-iteration account, every failure explicitly in the test harness, never the code; a real, subtler test-suite breakage (a test that would keep passing while checking the wrong code region) found and fixed.
- **WP-PERF Phase 0: cost-driver localisation for the two window estimators + measured CS Dubai-vs-v3 asymmetry answer** [result] — precise cProfile attribution to a specific, dominant, avoidable cost (full-array numpy reduction re-executed at every widening step); directly answers a previously-flagged asymmetry from measurement.
- **WP-PERF close-out: byte-identical speedup of the two window estimators** [decision][result][figure-source: `diagnostics/capture_wp_perf_reference.py`, `compare_wp_perf_reference.py`, keep-reproduces] — method proven completely unchanged; byte-identity proven (not assumed) via a self-tested comparison tool; ~1.6-1.8x end-to-end, ~2-2.5x on the touched estimators; two independent correctness proofs (golden suite + byte-identity capture) explicitly checked to agree, with a pre-stated reviewer condition that disagreement would have been a mandatory STOP.

**Figure sources, Ch. 6:**
- `diagnostics/inspect_pipeline_wall_times.py`, `diagnostics/capture_wp_perf_reference.py` + `compare_wp_perf_reference.py` [keep-reproduces] — profiling and byte-identity-proof tooling underlying the WP-PERF close-out numbers (before/after wall-clock tables reproduced verbatim in that entry above).

**Literature anchors, Ch. 6:** none (Tier C/product and Tier B engineering practice throughout; no literature claims made).

**Open gaps, Ch. 6:**
- `test_stability.py` has ZERO assertions — a standing, explicitly-flagged limitation of the project's own smoke-test coverage (phase-boundary and numerical correctness have no automated coverage beyond the targeted `tests/` suite built from 2026-08-20 onward).
- The `estimate_cornering_stiffness` Dubai-vs-v3 cost asymmetry is explained (widening-step count, not sample count) but the underlying WHY of Dubai's own alpha-signal character needing more widening steps is not investigated further.
- A concurrent, not-self-authored modification to the working tree during one ship-readiness session (Phase 5) was flagged but never diagnosed — origin remains unresolved.
- The fit chain (EKF run) is explicitly named as the new dominant remaining pipeline cost after WP-PERF, with no optimisation attempted (out of scope for that package).
- WP-PERF itself is on branch `perf-estimators`, not yet merged to `main` as of the last recorded session state — a real, live commit-boundary decision awaiting the user.

---

## 7. Honest limitations
*(kinematic circularity, stab threshold legacy, aero split gate, TC/EB unknowns, single-car/two-session basis)*

**Core reference list — `## 4. Level 1 limitations register`** (thesis_notes.md, unchanged since first written; check current code state before citing any item as still-accurate, several have since been partially superseded by later work — cross-references added):
1. Static weight split for Fy (overstates rear on roll-stiff GT3) — since qualified by the Fy yaw-moment term (Ch. 1) and the session-measured mass/split fractions work (Ch. 4), but the underlying Level-1 static-split limitation itself is not eliminated everywhere.
2. Iz from m·a·b bicycle estimate (~10-20% error; all Mz scales linearly).
3. Steering ratio constant (±25% real variation over travel) — since upgraded to a Level-4 manufacturer lookup on this car (Ch. 1, WP-B), reducing but not eliminating this line's force.
4. beta from kinematic integration + washout (drift-corrected, not measured) — the register's own upgrade path (GPS course) was attempted and shelved (Ch. 2); the EKF/refit arc closes as structurally non-identifiable without an external reference (Ch. 2) — **this remains the project's single most load-bearing open limitation**.
5. Accelerometer assumed at CoG — tested as a candidate mechanism for the zero-slip Fy offset and rejected as the dominant explanation (Ch. 2, WP-S3b).
6. No yaw damping term in Mz (Werner drops it too) — explicitly named as awaiting the damper/wheel-load work (Ch. 1/4), completed for Fz but the D_psi damping term itself is not confirmed wired into Module 5.
7. Closed-loop derivative — driver-in-the-loop affects c_beta.
8. Bicycle model: identical slip per axle.
9. Kerb detection: static threshold (rate-of-change upgrade documented as the path, not built) — a plausibility guard was later added for the LS_ratio-specific case (Ch. 2/6), not a general kerb-mask redesign.
10. ecu_speed is Level 1; log_gps_speed available for cross-validation — a k=1.01211 correction candidate was measured (Ch. 4) but never applied to production.

**Additional standing limitations found across the notebook (Ch. 7-relevant, not in the original register):**
- Kinematic beta's circularity into Module 4b's own production CS_ratio estimate (WP-S4b's Cr_A inflation finding, Ch. 2) — the kinematic slip-angle error is shown to propagate into the cornering-stiffness estimate itself, not only into beta.
- The refit-loop identifiability limit (Ch. 2) — a structural, not merely empirical, argument that the EKF's two measurement channels cannot jointly resolve state and curve parameters without an external reference; no such reference exists in either real session's own data (external-reference census, Ch. 2).
- `stab_neg_thresh` (stability threshold) legacy status under the production `ekf_auto_pacejka` default — explicitly BLOCKED, no value derivable from the current (all-positive) distribution (Ch. 3).
- Aero front/rear split — measured at 25%/75% but gated as single-car, two-session telemetry evidence only, explicitly not a substitute for a real windtunnel/CFD figure (Ch. 4).
- TC/EB channel identity and mapping — largely still IDENTIFICATION EVIDENCE ONLY; TC activity is on record as near-silent on both real sessions (0.28% and effectively 0.0044%), directly limiting what any TC-corroborated evidence source can currently confirm (Ch. 2/5).
- Combined-slip (longitudinal-lateral coupling) — this project's tyre models remain pure-lateral throughout; the friction-circle argument for why this produces specific false negatives (rear exit-traction, front entry-braking) is Tier A-anchored but never implemented (Ch. 2).
- Single-car, two-session evidentiary basis for every cross-session claim in this project (Dubai + GT3_PRC_MLA-v3) — explicitly named in several entries (the aero split, the premise-correction pattern, the mu load-sensitivity finding) as a real scope limit on how far any of these findings generalise.
- C4's own genuine saturation signal is diluted below visibility by the very CS_ratio repair that fixed the wholesale-negative artifact — an explicit, unresolved tension between two correct-on-their-own-terms fixes (Ch. 3).
- Verdict-pipeline non-robustness to sub-1%-level, entirely legitimate accuracy corrections (Ch. 3, Metrology Phase 1) — a real, now-quantified reproducibility bound, concentrated in already-borderline corners.
- The consistency gate's interaction with worst-lap aggregation now zeroes the live recommendation set on both real sessions under the anchored thresholds — reported as arguably correct behaviour, but a real, named open question for future calibration (Ch. 3/5).

**Open questions register (`## 6. Open questions / to verify before writing`, thesis_notes.md) — status as last recorded:**
- Official turn names for the compound-corner complex (GPS coordinates vs track map) — pending.
- Second-track validation of the 50 m clustering tolerance — pending new data.
- Werner MA full citation — **RESOLVED** 2026-07-24.
- "Suzuka convention" — **RESOLVED**, no citation ever existed.
- 992 GT3R official corner-weight/mass provenance — still open.
- Mitschke/Wallentowitz `estimate_sideslip` citation — **RESOLVED** (replaced, not sourced, by Rajamani §2.3/2.6).
- Rajamani Ch. 13.10/Ch. 14 exact pages, Kiencke & Nielsen exact section number — **TO VERIFY**, the last of these likely permanently (source PDF text does not survive extraction).
- Ulsoy, Peng, Cakmakci citation — **RESOLVED** 2026-08-19 (confirmed by two independent readings).
- Werner method-delta three-column comparison table — not yet built.

---

## Cross-reference: DECISION LAYER SPEC (PLAN.md, elicited 2026-09-22) vs. this index

The governing spec's six stages map onto Chapter 5 entries as follows, for quick lookup while writing the decision-layer chapter section-by-section:
- **Stage 1 (lever inventory)** — DECISION LAYER SPEC Phase A (registry/config schema); the 46-key `config/setup_parameters.json` registry itself (Ch. 1, Setup-parameter registry design).
- **Stage 2 (triggers)** — B2 (feedback-only routing, OPEN gap), the 39-rule matrix bridge (Frame-Stage-2 Phase 3).
- **Stage 3 (windows/current state)** — B5 (window edge), WP-FD1+2 (condition schema).
- **Stage 4 (interference)** — the interaction_table mechanism (Deepening Phase 4b, Literature-bridge package, Generic lever-bridge mechanism).
- **Stage 5 (corroboration/contradiction)** — B6 (contradiction), driver-feedback evidence (Deepening Phase 4d), ABS/TC intervention evidence (Frame-Stage-2 Phase 2, Deepening Phase 4c).
- **Stage 6 (cost function/output)** — C1 (scoring-term fold), Phase D (D6 rendering rule, ITEM 1 display grouping).

Elicitation list status (PLAN.md, as last recorded): items 1-9 mostly OPEN/pending engineer input (cost-function weights, tyre-pressure windows, damper step sizes, brake-bias channel identity, TC/EB mapping, rake default, the 14 non-verbatim matrix cells, aero-split expectation check, display-cutoff threshold); item 10 (feedback-router click-class coverage) OPEN; item 11 (splitter sign convention) **RESOLVED** 2026-09-22.

---

*Index built 2026-09-24 from a complete, sequential read of `thesis_notes.md` (20,285 lines, all 302 headers) and the referenced `PLAN.md` sections. Not edited into `thesis_notes.md` itself. Not committed.*
