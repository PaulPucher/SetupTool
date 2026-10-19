# Q-DATA-QUALITY -- uniform quality screen of every NBR and Le Castellet
# export (PLAN.md queue entry; author order and EXECUTE 2026-10-19; replaces
# Q-PR-DEMAND). Read-only. A file leaves the thesis set only by failing this
# screen, never by its analysis results.
#
# FROZEN FRAME (written before any number exists)
#
# CANDIDATES: every NBR and Le Castellet export found on this machine
#   (Dubai, Le Mans excluded): H:/Data_Bachelorthesis (tests/conftest.py
#   resolver), H:/UNI/Bachelorarbeit/Data/Sample, and -- outside the
#   resolver -- C:/Paul/UNI/Bachelorarbeit/outings (the GT3_PRC_MLA exports
#   of Le Castellet outing 25, v3 among them). Every export is screened on
#   its own, duplicates of one outing included.
# INPUTS: production parse and state (best-available params); axle force and
#   load exactly as fit L builds them (tyre_fit_auto._axle_load_inputs: front
#   in the tyre frame, Goblirsch et al. 2025 Eq. 13 with the Eq. 9 Fx split;
#   measured Fz from the cascade; 2 Hz); wheel loads from the measured
#   cascade (estimate_vertical_loads, vertical_load_source "measured").
# POPULATIONS: verdict population = moving, analysis laps, not kerb mask,
#   not smear zone; straight = verdict population with |a_y| (2 Hz) < 1 m/s^2;
#   census population = moving analysis-lap samples.
# Q1 physical ceiling: share of verdict-population samples with |Fy|/Fz > 2.0
#   per axle (Pacejka 2006 p. 6; Goblirsch et al. 2025 Table I). PASS if
#   < 0.1 % on both axles.
# Q2 load calibration: ratio of the measured four-wheel load sum to m g + c
#   v^2 on straight samples, m = config mass, c FIXED from the pooled NBR
#   straights (least squares of sum - m g on v^2 over every straight sample
#   of the NBR exports that pass Q3, one export per outing -- for outing 29 the
#   NBR_Sturm_Solution copy, the one used in all earlier records). PASS if
#   the median lies in [0.97, 1.03] and p5 >= 0.90.
# Q3 load completeness: damper-sourced share per corner on the census
#   population (the census definition of 2026-10-18). PASS if > 99 % on every
#   corner.
# Q4 analysis laps >= 3.
# A file that cannot be parsed or has no state FAILS with the reason.
# ALSO: FR / FL and RR / RL median load ratio on straights for every file
#   (asked for US-v2 and v3: the FR channel correction was derived on v3).
# OUTCOME: files passing Q1-Q4 form the thesis analysis set; failing files
#   are excluded with their criterion; no passing Le Castellet file -> Le
#   Castellet reported as a data limitation.
# Outputs: diagnostics/output/data_quality/ (screen.log, screen.json).
# Run: python -m diagnostics.inspect_data_quality
#
# [2026-10-19 PICK MODE, author ruling: --pick-o25 chooses ONE export of Le
#   Castellet outing 25 by the rule stated in thesis_notes before the count:
#   most valid required channels (channels.json required + prepare_vehicle_
#   state's list + fit L's inputs), ties -> latest file timestamp. Q2 is
#   withdrawn as an exclusion criterion (declared post-result revision).]

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402

OUT_DIR = "diagnostics/output/data_quality"
C_OUTINGS = "C:/Paul/UNI/Bachelorarbeit/outings"
NBR = "unlabeled/02_NBR_CRV_data"
# (key, track, outing, label, path or ("rel", relative path))
CANDIDATES = (
    ("nbr_o05_mla", "NBR", 5, "NBR outing 5 MLA", ("rel", f"{NBR}/01_SE_01/NBR-73-005-001-MLA-26.07.01.0919.txt")),
    ("nbr_o10_jdr", "NBR", 10, "NBR outing 10 JDR", ("rel", f"{NBR}/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt")),
    ("nbr_o18_jst", "NBR", 18, "NBR outing 18 JST", ("rel", f"{NBR}/01_SE_02/NBR-73-018-057-JST-26.07.01.1421.txt")),
    ("nbr_o19_jst", "NBR", 19, "NBR outing 19 JST", ("rel", f"{NBR}/01_SE_02/NBR-73-019-062-JST-26.07.01.1437.txt")),
    ("nbr_o20_jst", "NBR", 20, "NBR outing 20 JST", ("rel", f"{NBR}/01_SE_02/NBR-73-020-065-JST-26.07.01.1454.txt")),
    ("nbr_o21_jst", "NBR", 21, "NBR outing 21 JST", ("rel", "NBR_Sturm_problem/NBR-73-021-071-JST-26.07.01.1509.txt")),
    ("nbr_o29_jst", "NBR", 29, "NBR outing 29 JST (NBR_Sturm_Solution copy)",
     ("rel", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")),
    ("nbr_o29_jst_dup", "NBR", 29, "NBR outing 29 JST (unlabeled copy, different checksum)",
     ("rel", f"{NBR}/01_SE_02/NBR-73-029-101-JST-26.07.01.1734.txt")),
    ("nbr_o46_jst", "NBR", 46, "NBR outing 46 JST", ("rel", f"{NBR}/01_TE_04/NBR-73-046-162-JST-26.07.02.1411.txt")),
    ("nbr_o53_mla", "NBR", 53, "NBR outing 53 MLA", ("rel", f"{NBR}/01_TE_04/NBR-73-053-207-MLA-26.07.02.1707.txt")),
    ("nbr_o54_cdr", "NBR", 54, "NBR outing 54 CDR", ("rel", f"{NBR}/01_TE_04/NBR-73-054-214-CDR -26.07.02.1727.txt")),
    ("nbr_o59_mla", "NBR", 59, "NBR outing 59 MLA", ("rel", f"{NBR}/New folder/NBR-73-059-232-MLA-26.07.03.1210.txt")),
    ("nbr_o61_jdr", "NBR", 61, "NBR outing 61 JDR", ("rel", f"{NBR}/New folder/NBR-73-061-238-JDR-26.07.03.1229.txt")),
    ("nbr_o63_cdr", "NBR", 63, "NBR outing 63 CDR", ("rel", f"{NBR}/New folder/NBR-73-063-256-CDR-26.07.03.1309.txt")),
    ("nbr_o64_jst", "NBR", 64, "NBR outing 64 JST", ("rel", f"{NBR}/New folder/NBR-73-064-265-JST-26.07.03.1330.txt")),
    ("nbr_qp_cdr", "NBR", 69, "NBR qualifying CDR (outing 69)", ("rel", "NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt")),
    ("nbr_qp_mla", "NBR", 70, "NBR qualifying MLA (outing 70)", ("rel", "NBR_Qualy_MLA/MLA-QP01-NBR-v2.txt")),
    ("pr_o25_us_v1", "PR", 25, "Le Castellet outing 25, MLA-SE07_US-v1", ("rel", "Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt")),
    ("pr_o25_prc", "PR", 25, "Le Castellet outing 25, GT3_PRC_MLA (C:)", ("abs", f"{C_OUTINGS}/GT3_PRC_MLA.txt")),
    ("pr_o25_prc_v2", "PR", 25, "Le Castellet outing 25, GT3_PRC_MLA-v2 (C:)", ("abs", f"{C_OUTINGS}/GT3_PRC_MLA-v2.txt")),
    ("pr_o25_prc_v3", "PR", 25, "Le Castellet outing 25, GT3_PRC_MLA-v3 (C:, the records' v3)",
     ("abs", f"{C_OUTINGS}/GT3_PRC_MLA-v3.txt")),
    ("pr_o32_us_v2", "PR", 32, "Le Castellet outing 32, MLA-SE07_US-v2", ("rel", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt")),
    ("pr_o32_creventic", "PR", 32, "Le Castellet outing 32, GT3_Creventic_PRC_MLA (291 KB export)",
     ("rel", "Sample/GT3_Creventic_PRC_MLA.txt")),
)
POOL_SKIP = ("nbr_o29_jst_dup",)
STRAIGHT_AY = 1.0
Q1_LIMIT, Q1_PASS = 2.0, 0.001
Q2_BAND, Q2_P5 = (0.97, 1.03), 0.90
Q3_PASS = 0.99
Q4_LAPS = 3
CORNERS = ("fl", "fr", "rl", "rr")

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def screen_file(key, label, src, params_base):
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (prepare_vehicle_state, estimate_lateral_forces, estimate_vertical_loads,
                                            load_car_data, _butterworth_lowpass)
    from modules.corner_analysis import reclassify_speed_classes
    import modules.tyre_fit_auto as tfa
    kind, p = src
    path = resolve_data_path(p) if kind == "rel" else p
    out = {"key": key, "label": label, "path": path}
    if not os.path.exists(path):
        out["fail"] = ["file not found"]
        return out, None
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    params = params_base
    state = prepare_vehicle_state(data["channels"], params)
    laps_an = [l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")]
    out["analysis_laps"] = laps_an
    out["damper_quality"] = {c: (data["channels"].get(f"log_dms_dam_{c}") or {}).get("quality") for c in CORNERS}
    out["event_zero_entry"] = data.get("event_zero_entry")
    if state is None:
        out["fail"] = ["prepare_vehicle_state returned None (required channels missing)"]
        return out, None
    t = state["time"]
    an = tfa._analysis_lap_mask(t, data["laps"])
    moving_an = state["moving_mask"] & an
    aff = np.zeros(t.size, dtype=bool)
    for k in ("kerb_mask", "kerb_smear_zone"):
        if state.get(k) is not None:
            aff |= state[k]
    verdict = moving_an & ~aff
    forces = estimate_lateral_forces(state, params)
    li = tfa._axle_load_inputs(state, data, params, forces, verdict)
    if "degenerate_reason" in li:
        out["fail"] = [li["degenerate_reason"]]
        return out, None
    fy = {"front": tfa._front_tyre_frame_force(forces["Fy_f_filt"], li["fx_f_N"], li["delta_f"]),
          "rear": forces["Fy_r_filt"]}
    fz = {"front": li["fz_f_N"], "rear": li["fz_r_N"]}
    q1 = {}
    for a in ("front", "rear"):
        with np.errstate(invalid="ignore", divide="ignore"):
            d = np.abs(fy[a][verdict]) / fz[a][verdict]
        d = d[np.isfinite(d)]
        q1[a] = {"share_above_2": float(np.mean(d > Q1_LIMIT)) if d.size else float("nan"),
                 "max": float(d.max()) if d.size else float("nan"), "n": int(d.size)}
    out["Q1"] = q1
    meas = estimate_vertical_loads(state, forces, params, channels=data["channels"], car_data=load_car_data())
    src_lab = meas["vertical_load_source_per_sample"]
    if src_lab is None:
        out["Q3"] = {c: 0.0 for c in CORNERS}
        out["Q3_note"] = "static path (no damper data or no car_data)"
    else:
        out["Q3"] = {c: float(np.mean(np.asarray(src_lab[c])[moving_an] == "damper")) if moving_an.any() else float("nan")
                     for c in CORNERS}
    ayf = _butterworth_lowpass(state["ay_mps2"], params["stability_estimation"]["cs_filter_cutoff_hz"],
                               state["sample_rate_hz"])
    straight = verdict & (np.abs(ayf) < STRAIGHT_AY)
    tot = meas["fz_fl_N"] + meas["fz_fr_N"] + meas["fz_rl_N"] + meas["fz_rr_N"]
    out["n_straight"] = int(straight.sum())
    out["symmetry_straights"] = {
        "FR/FL": float(np.median(meas["fz_fr_N"][straight] / meas["fz_fl_N"][straight])) if straight.any() else float("nan"),
        "RR/RL": float(np.median(meas["fz_rr_N"][straight] / meas["fz_rl_N"][straight])) if straight.any() else float("nan")}
    out["c_session"] = meas.get("c_session_N_per_mps2")
    return out, {"v": state["v_mps"][straight], "tot": tot[straight]}


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "screen.log"), "w", encoding="utf-8")
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from diagnostics.inspect_fit_lit import keep_awake
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))
    mass = params["vehicle"]["mass_kg"]
    keep_awake(True)
    t0 = time.perf_counter()
    try:
        R, STR = {}, {}
        for key, track, outing, label, src in CANDIDATES:
            try:
                rec, st = screen_file(key, label, src, params)
            except Exception as e:  # a broken export fails the screen, it does not stop it
                rec, st = {"key": key, "label": label, "fail": [f"exception: {type(e).__name__}: {e}"]}, None
            rec["track"], rec["outing"] = track, outing
            R[key], STR[key] = rec, st
            log(f"{label}: screened ({(time.perf_counter() - t0) / 60:.1f} min)")
        # pooled NBR c from the Q3-passing NBR exports, one per outing
        pool = [k for k, r in R.items() if r["track"] == "NBR" and k not in POOL_SKIP and STR[k] is not None
                and all(r["Q3"][c] > Q3_PASS for c in CORNERS)]
        v2 = np.concatenate([STR[k]["v"] ** 2 for k in pool])
        dt = np.concatenate([STR[k]["tot"] - mass * 9.81 for k in pool])
        c_nbr = float(np.sum(v2 * dt) / np.sum(v2 * v2))
        log(f"\npooled NBR c = {c_nbr:.4f} N / (m/s)^2 from {len(pool)} exports, {v2.size} straight samples: {pool}")
        for k, r in R.items():
            fails = list(r.get("fail", []))
            if STR[k] is not None and STR[k]["v"].size:
                ratio = STR[k]["tot"] / (mass * 9.81 + c_nbr * STR[k]["v"] ** 2)
                r["Q2"] = {"median": float(np.median(ratio)), "p5": float(np.percentile(ratio, 5)), "n": int(ratio.size)}
            elif "fail" not in r:
                r["Q2"] = {"median": float("nan"), "p5": float("nan"), "n": 0}
            if "fail" not in r:
                q = {"Q1": all(r["Q1"][a]["share_above_2"] < Q1_PASS for a in ("front", "rear")),
                     "Q2": Q2_BAND[0] <= r["Q2"]["median"] <= Q2_BAND[1] and r["Q2"]["p5"] >= Q2_P5,
                     "Q3": all(r["Q3"][c] > Q3_PASS for c in CORNERS),
                     "Q4": len(r["analysis_laps"]) >= Q4_LAPS}
                r["checks"] = q
                fails += [n for n, ok in q.items() if not ok]
            r["fails"] = fails
            r["pass"] = not fails
        log("\n## PER FILE")
        for k, r in R.items():
            if "checks" not in r:
                log(f"- {r['label']}: FAIL -- {'; '.join(r['fails'])}")
                continue
            log(f"- {r['label']}: {'PASS' if r['pass'] else 'FAIL (' + ', '.join(r['fails']) + ')'}")
            log(f"    Q1 share |Fy|/Fz > 2: front {r['Q1']['front']['share_above_2'] * 100:.3f} % (max "
                f"{r['Q1']['front']['max']:.2f}), rear {r['Q1']['rear']['share_above_2'] * 100:.3f} % (max "
                f"{r['Q1']['rear']['max']:.2f})")
            log(f"    Q2 straight load / (m g + c_NBR v^2): median {r['Q2']['median']:.3f}, p5 {r['Q2']['p5']:.3f} "
                f"(n {r['Q2']['n']}; session c {r.get('c_session')})")
            log(f"    Q3 damper share: " + ", ".join(f"{c} {r['Q3'][c] * 100:.2f} %" for c in CORNERS)
                + f"; damper channel quality {r['damper_quality']}")
            log(f"    Q4 analysis laps {len(r['analysis_laps'])} {r['analysis_laps']}")
            log(f"    symmetry on straights FR/FL {r['symmetry_straights']['FR/FL']:.3f}, RR/RL "
                f"{r['symmetry_straights']['RR/RL']:.3f}")
        passing = [k for k, r in R.items() if r["pass"]]
        pr_pass = [k for k in passing if R[k]["track"] == "PR"]
        log(f"\n## OUTCOME: analysis set ({len(passing)}): {[R[k]['label'] for k in passing]}")
        log("Le Castellet: " + (f"passing {[R[k]['label'] for k in pr_pass]}" if pr_pass
                                else "NO file passes -> reported as a data limitation"))
        with open(os.path.join(OUT_DIR, "screen.json"), "w", encoding="utf-8") as fh:
            json.dump({"c_nbr": c_nbr, "pool": pool, "files": R}, fh, indent=1,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


PICK_REQUIRED = ("ecu_speed", "lap_number", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_x",
                 "log_dms_dam_fl", "log_dms_dam_fr", "log_dms_dam_rl", "log_dms_dam_rr",
                 "log_susp_travel_fl", "log_susp_travel_fr", "log_susp_travel_rl", "log_susp_travel_rr",
                 "log_pbrake_f", "log_pbrake_r")
PICK_CANDIDATES = ("pr_o25_us_v1", "pr_o25_prc_v2", "pr_o25_prc_v3")


def main_pick_o25():
    from modules.csv_parser import parse_csv
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    with open(os.path.join(OUT_DIR, "pick_o25.log"), "w", encoding="utf-8") as fh:
        for key, track, outing, label, src in CANDIDATES:
            if key not in PICK_CANDIDATES:
                continue
            kind, p = src
            path = resolve_data_path(p) if kind == "rel" else p
            data = parse_csv(path)
            q = {c: (data["channels"].get(c) or {}).get("quality") for c in PICK_REQUIRED}
            n_req = sum(v == "valid" for v in q.values())
            n_all = sum(ch.get("quality") == "valid" for ch in data["channels"].values())
            mtime = os.path.getmtime(path)
            rows.append((n_req, mtime, key, label, path, q, n_all))
            line = (f"{label}: valid required {n_req} / {len(PICK_REQUIRED)}; valid of all configured channels "
                    f"{n_all} / {len(data['channels'])}; file timestamp {time.strftime('%Y-%m-%d %H:%M', time.localtime(mtime))}; "
                    f"not valid: {[c for c, v in q.items() if v != 'valid'] or 'none'}")
            print(line, flush=True)
            fh.write(line + chr(10))
        best = max(rows, key=lambda r: (r[0], r[1]))
        line = f"CHOICE (most valid required, ties -> latest timestamp): {best[3]} -- {best[4]}"
        print(line, flush=True)
        fh.write(line + chr(10))
    return 0


if __name__ == "__main__":
    sys.exit(main_pick_o25() if "--pick-o25" in sys.argv[1:] else main())
