ORDER SPEED-1 (2026-11-06) results. Records only; thesis_notes 'ORDER SPEED-1' holds the numbers.

part3/  Decomposition on the ten final sessions (NBR 1-5, PRC 1-5; app outings 17-22, 25-28),
        one JSON per session and run, made by scripts/speed1_runner.py on scratchpad copies of
        the working repo (real config, stored setup sheets from a DB copy):
          off     all three signal corrections off (= schema 30 chain; stored_identity block
                  compares with the stored schema-30 payload)
          r1      speed reference only
          r1r2    speed reference + engine braking
          all     all three = new default
          r3post  extra variant: as 'all' but the sensor offsets applied after the corner
                  detection (separates segmentation shifts from the estimator effect)
        part3.txt = scripts/speed1_compare.py over off / r1 / r1r2 / all.
part5/  Re-derivation PROPOSALS on the 14 development sessions with the new default, by the
        recorded scripts run unchanged in a scratchpad copy:
          rederivation_l_speed1_20261106.*  (a) inspect_threshold_rederivation_l.py
          closeout.*                        (b) inspect_closeout_l.py
          nis_window_share.*                (b) inspect_nis_window_share.py
          mu_uncertainty.json               (c) inspect_mu_uncertainty.py, run as
                                            scripts/inspect_mu_uncertainty_scratch_u_recorded.py
                                            (u_wheel_N read from the config as of 864f77c^,
                                            where it was removed; nothing else changed)
          cs_margin_l/, cs_margin_combined/ (d) inspect_cs_margin_l.py and
                                            inspect_cs_margin_combined.py
scripts/r1_signature.py  root-cause check of the R1 detection reference (front vs four-wheel
                         ABS maximum) on the ten final sessions.

speed1b/ (ORDER SPEED-1b, 2026-11-06)
  r3_check.json   item 2: standstill on wheels vs the chain's straights for yaw rate, a_y and
                  steering, v r vs a_y on the straights, heading change per analysis lap
                  (scripts/r3_check.py, R1-only scratchpad copy, R3 off).
  r3stand/        item 3: run (3) with R3 taking yaw rate and a_y from the standstill and the
                  steering from the straights (scratchpad variant of signal_corrections).
part5/mu_uncertainty_repo_1b.json  u_mu from the repo script after u_wheel_N was restored.

speed1c/ (ORDER SPEED-1c, 2026-11-06, amended R3: standstill offsets for yaw rate and a_y,
         near-zero-yaw straights for steering)
  part3/{off,r1r2,all}/  runs (0) all off (stored_identity vs the schema-30 payloads), (2) R1+R2,
                         (3) new default; part3.txt = scripts/summ_1c.py
  part5/                 the four re-derivation proposals, recorded scripts unchanged
                         (rederivation_l_speed1c_20261106.*, closeout.*, nis_window_share.*,
                         mu_uncertainty.json, cs_margin_l/, cs_margin_combined/)
  corner_id_map.json     NBR 1 and PRC 3 stable corner IDs, pre-speed-fix -> SPEED-1c, by
                         bracket overlap (scripts/corner_map.py)
The part3/ and part5/ folders one level up hold the SPEED-1 (straight-line R3) results,
superseded by speed1c/.

speed2a/ (ORDER SPEED-2a, 2026-11-06): sens1_recount_analysis.* = inspect_sens1_recount.py on the adopted chain (schema 33); speed1c/corner_id_map_all10.json = all ten final sessions (scripts/corner_map_all.py); old vs new by scripts/sens_cmp.py.
