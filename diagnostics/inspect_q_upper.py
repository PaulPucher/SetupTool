# Q-UPPER (author STEP 2026-10-19): the fit-L observer with Q at the per-session
# held-over-step upper bound of Q-PHYS, against the divergence trigger.
# Frame written before any number: DECISIONS.md "Q-UPPER frame". Read-only:
# Q, trigger and h are set in memory per run (fit-L keys l_q_beta_var,
# l_q_yaw_rate_var, l_divergence_trigger, l_cd_threshold_h); config untouched.
#
# Derive split (NBR 5, 20, 46, 54, 61, 69, Le Castellet 25). Per session the
# export is parsed once; per point the FULL chain runs (fit L, interim run,
# R sweep, final run), so R is re-derived. Points: change detector h = 400,
# 800, 1600, 3200; hard-bound only (change detector with h = inf, no soft
# reset); NIS-window trigger at Q-UPPER as reference line only.
# Per point and session: hard-bound re-entries (base mask), alarms (resets
# that are not hard-bound, base mask), alarms and time on all steering-defined
# straights, mean / median NIS (base mask), R pick, gates (status, NIS gate,
# sign check, ay / yaw exceedance, D4-L, h2), straight-line |beta| median of
# the observer on the validation (odd) straights.
# Selection (frame): pooled mean time between alarms on all straights >= 10 s;
# re-entries within 10 % of the minimum tie, then fewest alarms; no pick if
# re-entries fall monotonically to the smallest h.
# Output diagnostics/plots/q_upper/ (run.log, results.json).
# Run: python -m diagnostics.inspect_q_upper
# [2026-10-19, observer finalisation: the chain now computes the fit-L Q per
#   session itself and the keys l_q_beta_var / l_q_yaw_rate_var are removed,
#   so the Q override below is inert; the recorded results reproduce at the
#   code state before that change.]

import copy
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "q_upper")
Q_PHYS_DIR = os.path.join(ROOT, "diagnostics", "plots", "q_phys")
DERIVE = ("nbr_o05_mla", "nbr_o20_jst", "nbr_o46_jst", "nbr_o54_cdr", "nbr_o61_jdr", "nbr_qp_cdr", "pr_o25_us_v1")
POINTS = (("h 400", "change_detector", 400.0), ("h 800", "change_detector", 800.0),
          ("h 1600", "change_detector", 1600.0), ("h 3200", "change_detector", 3200.0),
          ("hard-bound only", "change_detector", float("inf")), ("NIS window (reference)", "nis_window", None))
ORDERED = ("h 400", "h 800", "h 1600", "h 3200", "hard-bound only")
MTBA_MIN_S = 10.0
TIE_FRAC = 0.10
WORKERS = 3


def _straights(st, laps):
    from diagnostics.inspect_kinematic_inputs import steering_straights
    t = st["time"]
    valid = [l for l in laps if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in valid:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    segs = sorted(steering_straights(st, valid, lap_id, st["kerb_mask"] | st["kerb_smear_zone"])["segs"])
    all_m, val_m = np.zeros(t.size, dtype=bool), np.zeros(t.size, dtype=bool)
    for k, (a, b) in enumerate(segs):
        all_m[a:b] = True
        if k % 2 == 1:
            val_m[a:b] = True
    return all_m, val_m


def run_session(key):
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    base_p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                                  resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    base_p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
    st = prepare_vehicle_state(data["channels"], base_p)
    sr = st["sample_rate_hz"]
    with open(os.path.join(Q_PHYS_DIR, f"q_phys_{key}.json"), encoding="utf-8") as fh:
        qp = json.load(fh)
    q_beta, q_r = qp["e_beta_dot"]["q_held_step"], qp["e_r_dot"]["q_held_step"]
    straight_all, straight_val = _straights(st, data["laps"])
    out = {"key": key, "label": label, "q_beta": q_beta, "q_r": q_r, "parse_s": time.perf_counter() - t0,
           "straight_all_s": float(straight_all.sum() / sr), "straight_val_s": float(straight_val.sum() / sr),
           "points": {}}
    o_fsp, o_ekf = tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka
    for name, trig, h in POINTS:
        p = copy.deepcopy(base_p)
        cfg = p["tyre_fit_auto"]
        cfg["l_q_beta_var"], cfg["l_q_yaw_rate_var"] = q_beta, q_r
        cfg["l_divergence_trigger"] = trig
        if h is not None:
            cfg["l_cd_threshold_h"] = h
        held = {}

        def fsp(*a, **kw):
            held["raw"] = o_fsp(*a, **kw)
            return held["raw"]

        def ekf(state, params, pass_id=None, **kw):
            r = o_ekf(state, params, pass_id=pass_id, **kw)
            if pass_id == "_auto_final":
                held["final"] = r
            return r

        tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = fsp, ekf
        t1 = time.perf_counter()
        try:
            beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, data, "ekf_auto_pacejka", csv_path=path)
        finally:
            tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = o_fsp, o_ekf
        raw, fin = held.get("raw") or {}, held.get("final")
        rec = {"status": man.get("status"), "reason": man.get("degenerate_reason") or man.get("marginal_reason"),
               "fallback": fb, "fallback_reason": fr,
               "gate": gate and {k: gate[k] for k in ("verdict", "health_score")},
               "r_pick": (man.get("r_sweep") or {}).get("chosen") and
                         {k: man["r_sweep"]["chosen"][k] for k in ("r_ay_scale", "r_yaw_scale")},
               "r_selection": (man.get("r_sweep") or {}).get("selection"),
               "sign_check": (man.get("sign_check") or {}).get("median_gate_racing_fraction"),
               "nis_exceed": man.get("nis"), "h2": (man.get("h2_vs_ay_apex") or {}).get("correlation"),
               "d4": {a: (raw.get("plausibility_gate") or {}).get(a, {}).get("failed") for a in ("front", "rear")},
               "chain_s": time.perf_counter() - t1}
        if fin is not None:
            base = raw["base_mask"]
            bound = np.radians(cfg["beta_hard_bound_deg"])
            alarms = fin["diverged_mask"] & ~fin["hard_bound_mask"]
            nis = fin["nis"][base & np.isfinite(fin["nis"])]
            rec.update({"hard_bound": int(((np.abs(fin["beta"]) > bound) & base).sum()),
                        "alarms": int((alarms & base).sum()), "alarms_straight": int((alarms & straight_all).sum()),
                        "nis_mean": float(nis.mean()), "nis_median": float(np.median(nis)),
                        "beta_val_median_deg": float(np.median(np.abs(np.degrees(fin["beta_with_fallback"][straight_val]))))})
        out["points"][name] = rec
    out["total_s"] = time.perf_counter() - t0
    return out


def main():
    from diagnostics.inspect_fit_lit import keep_awake, _jsonable
    os.makedirs(OUT_DIR, exist_ok=True)
    log_f = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")

    def log(msg=""):
        print(msg, flush=True)
        log_f.write(msg + "\n")
        log_f.flush()

    keep_awake(True)
    t0 = time.perf_counter()
    try:
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            results = list(ex.map(run_session, DERIVE))
        log(f"wall time {time.perf_counter() - t0:.0f} s, {WORKERS} workers")
        for r in results:
            log("=" * 100)
            log(f"{r['label']}: Q-UPPER q_beta {r['q_beta']:.3e}, q_r {r['q_r']:.3e} per step at 50 Hz; straights all "
                f"{r['straight_all_s']:.0f} s, validation {r['straight_val_s']:.0f} s; parse {r['parse_s']:.0f} s, "
                f"session total {r['total_s']:.0f} s")
            for name, x in r["points"].items():
                ex_ = x["nis_exceed"] or {}
                log(f"  {name}: status {x['status']} ({x['reason'] or '-'}), fallback {x['fallback']}"
                    + (f" ({x['fallback_reason']})" if x["fallback"] else "")
                    + f"; NIS gate {x['gate']}; R {x['r_pick']} ({x['r_selection']}); sign check {x['sign_check']}; "
                    f"exceedance yaw {ex_.get('yaw_rate_exceedance', float('nan')):.3f} ay "
                    f"{ex_.get('ay_exceedance', float('nan')):.3f}; h2 {x['h2']}; D4-L failed {x['d4']}")
                if "hard_bound" in x:
                    log(f"     hard-bound re-entries {x['hard_bound']}, alarms {x['alarms']} ({x['alarms_straight']} on "
                        f"straights), NIS mean {x['nis_mean']:.3f} median {x['nis_median']:.3f}, straight-line |beta| "
                        f"median {x['beta_val_median_deg']:.3f} deg; chain {x['chain_s']:.0f} s")
        log("=" * 100)
        t_all = sum(r["straight_all_s"] for r in results)
        summed = {}
        for name, _, _ in POINTS:
            xs = [r["points"][name] for r in results if "hard_bound" in r["points"][name]]
            n_st = sum(x["alarms_straight"] for x in xs)
            summed[name] = {"n": len(xs), "hard_bound": sum(x["hard_bound"] for x in xs),
                            "alarms": sum(x["alarms"] for x in xs), "alarms_straight": n_st,
                            "mtba_s": t_all / n_st if n_st else float("inf"),
                            "fallbacks": sum(bool(r["points"][name]["fallback"]) for r in results),
                            "gate_pass": sum((r["points"][name]["gate"] or {}).get("verdict") == "pass" for r in results),
                            "beta_val_max_deg": max(x["beta_val_median_deg"] for x in xs) if xs else float("nan")}
            s = summed[name]
            log(f"SUM {name} ({s['n']} sessions with an observer run): hard-bound re-entries {s['hard_bound']}, alarms "
                f"{s['alarms']}, {s['alarms_straight']} on {t_all:.0f} s of straights -> mean time between alarms "
                f"{s['mtba_s']:.1f} s; fallbacks {s['fallbacks']}; NIS gate pass {s['gate_pass']} / {len(results)}; "
                f"worst straight-line |beta| median {s['beta_val_max_deg']:.3f} deg")
        cand = [n for n in ORDERED if summed[n]["mtba_s"] >= MTBA_MIN_S]
        hb = [summed[n]["hard_bound"] for n in ORDERED]
        monotone = all(hb[i] < hb[i + 1] for i in range(len(hb) - 1))
        if not cand:
            verdict = "NO PICK: no point meets the straight constraint"
        else:
            m = min(summed[n]["hard_bound"] for n in cand)
            tied = [n for n in cand if summed[n]["hard_bound"] <= m * (1 + TIE_FRAC)]
            pick = min(tied, key=lambda n: summed[n]["alarms"])
            if monotone and tied == ["h 400"]:
                verdict = "NO PICK: re-entries still fall monotonically to the smallest h"
            else:
                verdict = f"SELECTED {pick} (tied within 10 % of minimum {m}: {tied})"
        log(f"constraint met: {cand}; re-entries in order {dict(zip(ORDERED, hb))}; strictly monotone {monotone}")
        log(verdict)
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable({"results": results, "summed": summed, "verdict": verdict}), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
