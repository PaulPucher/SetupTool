# INST margin and band: ORDER INST-1, INST-2 and the INST-2 addendum W (2026-11-07)

Branch inst-margin, schema-33 chain. Read-only: no module or config file changed and nothing was adopted.
Records: thesis_notes 'ORDER INST-1', 'ORDER INST-2', 'ORDER INST-2 addendum (W)'; PLAN.md STATUS; DECISIONS
'INST margin rulings' and 'INST band proposed, pending author adoption'.

## Question

The CS verdict reads its boundary with an uncertainty margin. The open-loop instability verdict (INST,
dMz/dbeta < 0, Werner 2021 sign criterion) had no margin. The orders asked what margin the recorded CS rule
gives for INST, and what that margin does to the final evaluation.

## What was run

- INST-1, chain runs, launched by the work order: diagnostics/inspect_inst_margin.py, run from a scratchpad
  copy of the working repo (real config, copy of the app DB).
  - 14 development sessions x 13 variants = 182 runs. The variants are: baseline, observer Q x 0.3, R_yaw x 10,
    and the ten +/-1 % vehicle-input runs (mass, front fraction, wheelbase, yaw inertia, steering-table wheel
    angle).
  - Plus 10 final-session runs (NBR 1-5, PRC 1-5; outings 17-22, 25-28), with the stored setup sheets.
  - None failed. Corner segmentation was identical in every run.
- INST-2 and the addendum: post-processing of the stored INST-1 runs only, no new chain run. Scripts are in
  scripts/ (inst2_band.py, inst2_w.py; inst2_tiers.py superseded); output is in inst2_band.*, inst2_w.* and
  inst2_tiers.*.
- Every counterfactual rebuilds the classifier and the setup-engine data layer in memory, with the INST boundary
  or severity changed. In all ten final sessions the baseline reproduces the pipeline's counted symptoms exactly.

## INST-1: the recorded CS rule applied unchanged

- The rule (inspect_cs_margin_combined):
  - obs = max |change| under Q x 0.3 or R_yaw x 10
  - veh = largest |change| over the ten +/-1 % vehicle runs
  - combined = sqrt(obs^2 + veh^2)
  - margin = pooled p90 over the 14 development sessions
  - Unit: corner x phase, median over the analysis laps of dMz/dbeta, in N m/deg; 975 units.
- Rulings (review): semantics as CS in the code, i.e. count and vote below -margin, [MARGINAL] tag just beyond,
  nothing in [-margin, 0). The band is the 2nd-most-marginal flip distance rounded up to 1 N m/deg. Analysis
  laps are used for obs and veh.
- Result: m_INST = 511.84 N m/deg, set by Q x 0.3 alone.
  - |change| p50 / p90 / p99 under Q x 0.3: 260.6 / 511.8 / 934.4
  - under R_yaw x 10: 0.8 / 2.8 / 9.3
  - vehicle part: 5.6 / 14.6 / 23.9 (yaw inertia dominant)
- Q x 0.3 is a systematic upward shift of the slope estimate, not noise around zero. The median ratio of
  perturbed to baseline value is 1.44. 16 of the 18 development INST units turn positive under it, which
  matches SENS-1 on the same chain.
- Flip band: no flip at -m_INST, so the rule gives no value. The p90 of veh (14.56) is shown for comparison.
- Analysis laps and all laps give identical numbers: out- and in-laps carry no corner summaries.
- Chain check: the CS margins rebuilt from the same runs are 0.0638 front / 0.0788 rear (adopted 0.063 / 0.077).
- Counterfactual with 512, on the ten final sessions:
  - INST 11 -> 0
  - 7 of 159 verdicts changed, 11 counted symptoms changed, 4 session recommendations changed
  - Every named case is removed.
  - Before the margin, INST never coincided with rear limiting: 0 / 11 units, 0 / 57 cells.
- Decision: not adopted (INST-2).

## INST-2: band from measurement and vehicle-input uncertainty only

- Ruling (author + review): Q is a model setting for dMz/dbeta. It is reported as a sensitivity, like the fit-L
  curve shape E, and not built into the band.
- band_INST = pooled p90 of sqrt(r_yaw^2 + veh^2) = 15.3856 N m/deg, unrounded (p50 5.80, p99 23.89; 975 units,
  14 sessions). veh is larger than r_yaw in 940 of the 975 units.
- None of the 18 development INST units lies in [-band, 0).
- Counterfactual on the ten final sessions (INST counts only below -band, strong):
  - INST 11 -> 10. Only PRC 5 C7 is removed (apex -14.9, 0.5 N m/deg inside the band).
  - 1 of 159 verdicts changed, 1 counted symptom changed.
  - 1 recommendation changed: PRC 5 rear ride height lower (net 2.0) -> front low-speed rebound stiffer
    (net 1.2).
  - These survive: NBR 1 C8 (-69.8 / -75.1), NBR 1 C11 (-140.9 / -173.5), NBR 5 C2 (-65.0),
    NBR 5 C9 (-129.6 / -55.7), PRC 2 C11 (-116.0).
- Status: proposal, pending author adoption.

## INST-2 addendum W: INST votes weight 1 (replaces the tier addendum T)

- Variant W uses band_INST as in INST-2, but INST votes with weight 1 (like a moderate CS symptom) instead of 2.
  INST severity and the verdict text are unchanged; only the session vote weight changes.
- Session recommendations: the default, the band alone and W each give 6 (NBR 1, NBR 4, NBR 5, PRC 1, PRC 2,
  PRC 5).
- W differs from the band alone in 3 sessions, in net support only. The lever and direction stay the same:
  - NBR 1: rear anti-roll bar softer, net 8.0 -> 4.0 (4 INST votes: C8 turn-in and apex, C11 apex and early exit)
  - NBR 5: rear anti-roll bar softer, net 10.0 -> 5.0 (5 INST votes: C2 apex, C5 apex and early exit, C9 turn-in
    and apex)
  - PRC 2: rear low-speed bump stiffer, net 2.0 -> 1.0 (1 INST vote: C11 late exit)
  - Unchanged: NBR 4 front low-speed rebound stiffer (net 2.0); PRC 1 front low-speed rebound stiffer (net 2.0);
    PRC 5 front low-speed rebound stiffer (net 1.2; its INST at C7 is already below the band).
- How many of the six recommendations still come from INST:
  - 3 of 6 (NBR 1, NBR 5, PRC 2), under both the band alone and W.
  - In each of the three, INST supplies all of the support and there is no opposition, so halving the weight
    halves the net support but cannot change the lever.
  - The other 3 (NBR 4, PRC 1, PRC 5) have no INST support.
  - For comparison, the schema-33 default: 4 of 6 come entirely from INST (also PRC 5 rear ride height lower,
    net 2.0).
- Script: scripts/inst2_w.py; output: inst2_w.txt / .json.

## Superseded: tier variant T (author, 2026-11-07, replaced by W)

- T was computed before the replacement arrived and is kept as a record only (scripts/inst2_tiers.py,
  inst2_tiers.*). Development units below -band: 18 in 7 sessions; p10 / p50 / p90 -139.95 / -76.59 / -31.34;
  bootstrap 90 % interval of the p50 [-102.50, -38.66]. With strong = -76.59 and moderate in (-76.59, -15.39):
  NBR 1 net 8.0 -> 6.0, NBR 5 net 10.0 -> 6.0 (same lever), PRC 5 as with the band alone.

## Open for the author

- Adopting band_INST = 15.3856 N m/deg. This needs a new config key and, by the standing rule, a schema bump.
- PRC 5 C7 sits 0.5 N m/deg inside the band. The PRC 5 recommendation depends on it.
- Variant W (INST weight 1) is a comparison only. It halves the net support of the three INST-driven recommendations
  (NBR 1, NBR 5, PRC 2) and changes no lever.
