# Fit-L observer after the finalisation of 2026-10-19 (author EXECUTE): Q per
# session from the fit-L force residual (Q-UPPER), trigger hard bound only,
# R_ay grid continued down to the session's ay noise floor. Full chain on the
# 14-session analysis set with the shipped config; read-only.
# Per session: status / fallback and why, Q used, the ay and yaw-rate noise
# floors, the R_ay pick and where it lands, the R_yaw used (the yaw floor),
# hard-bound re-entries (base mask), NIS mean / median (base mask),
# straight-line |beta| median of the observer on the validation straights
# (odd-indexed steering-defined straights, modules/straights.py), all gate
# values (NIS gate, sign check, yaw / ay exceedance, h2, D4-L per axle).
# Output diagnostics/plots/fit_l_observer_final/ (run.log, results.json).
# Run: python -m diagnostics.run_fit_l_observer_final

import copy
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "fit_l_observer_final")
ANALYSIS = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
            "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
WORKERS = 3


def run_session(key):
    import modules.tyre_fit_auto as tfa
    from modules.straights import steering_straights
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
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
    st = prepare_vehicle_state(data["channels"], p)
    held = {}
    o_fsp, o_ekf = tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka

    def fsp(*a, **kw):
        held["raw"] = o_fsp(*a, **kw)
        return held["raw"]

    def ekf(state, params, pass_id=None, **kw):
        r = o_ekf(state, params, pass_id=pass_id, **kw)
        if pass_id == "_auto_final":
            held["final"] = r
        return r

    tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = fsp, ekf
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, data, "ekf_auto_pacejka", csv_path=path)
    finally:
        tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = o_fsp, o_ekf
    raw, fin = held.get("raw") or {}, held.get("final")
    cfg = p["tyre_fit_auto"]
    sw = man.get("r_sweep") or {}
    ch = sw.get("chosen") or {}
    rec = {"key": key, "label": label, "status": man.get("status"),
           "reason": man.get("degenerate_reason") or man.get("marginal_reason"), "fallback": fb, "fallback_reason": fr,
           "gate": gate and {k: gate[k] for k in ("verdict", "health_score")},
           "q": man.get("observer_q") or raw.get("observer_q"),
           "r_pick": {k: ch.get(k) for k in ("r_ay_scale", "r_yaw_scale", "R_ay_var", "R_yaw_rate_var")},
           "r_selection": sw.get("selection"), "floors": raw.get("noise_floors"),
           "ay_grid": sorted({g["r_ay_scale"] for g in sw.get("grid_points", [])}),
           "sign_check": (man.get("sign_check") or {}).get("median_gate_racing_fraction"),
           "nis_exceed": man.get("nis"), "h2": (man.get("h2_vs_ay_apex") or {}).get("correlation"),
           "d4": {a: {k: (raw.get("plausibility_gate") or {}).get(a, {}).get(k)
                      for k in ("stiffness_norm_per_rad", "alpha_span_deg", "mu", "failed")}
                  for a in ("front", "rear")}}
    if fin is not None:
        base = raw["base_mask"]
        bound = np.radians(cfg["beta_hard_bound_deg"])
        segs = steering_straights(st, data["laps"], cfg)["segs"]
        val = np.zeros(st["time"].size, dtype=bool)
        for k, (a, b) in enumerate(segs):
            if k % 2 == 1:
                val[a:b] = True
        nis = fin["nis"][base & np.isfinite(fin["nis"])]
        rec.update({"hard_bound": int(((np.abs(fin["beta"]) > bound) & base).sum()),
                    "soft_resets": int((fin["diverged_mask"] & ~fin["hard_bound_mask"]).sum()),
                    "nis_mean": float(nis.mean()), "nis_median": float(np.median(nis)),
                    "beta_val_median_deg": float(np.median(np.abs(np.degrees(fin["beta_with_fallback"][val])))),
                    "val_straight_s": float(val.sum() / st["sample_rate_hz"])})
    rec["total_s"] = time.perf_counter() - t0
    return rec


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
            results = list(ex.map(run_session, ANALYSIS))
        log(f"wall time {time.perf_counter() - t0:.0f} s, {WORKERS} workers")
        for r in results:
            log("=" * 100)
            log(f"{r['label']}: status {r['status']} ({r['reason'] or '-'}); fallback {r['fallback']}"
                + (f" ({r['fallback_reason']})" if r["fallback"] else "") + f"; session {r['total_s']:.0f} s")
            q, fls = r["q"] or {}, r["floors"] or {}
            fl, fy = fls.get("ay") or {}, fls.get("yaw_rate") or {}
            if q:
                log(f"  Q: q_beta {q['q_beta_var']:.3e} rad^2, q_r {q['q_yaw_rate_var']:.3e} (rad/s)^2 per step at 50 Hz "
                    f"(SD beta_dot {q['sd_beta_dot']:.4f} rad/s, r_dot {q['sd_r_dot']:.3f} rad/s^2, n {q['n_joint']})")
            if fl:
                log(f"  noise floors on {fl['straight_s']:.0f} s of straights: ay {fl['var']:.4f} (m/s^2)^2 = "
                    f"r_ay_scale {fl['r_ay_scale']:.4g}; yaw rate {fy['var']:.3e} (rad/s)^2 = r_yaw_scale "
                    f"{fy['r_yaw_scale']:.4g}; ay grid {[round(x, 5) for x in r['ay_grid']]}")
            log(f"  R_ay pick: r_ay_scale {r['r_pick']['r_ay_scale']:.4g} = {r['r_pick']['R_ay_var']:.4f} (m/s^2)^2 "
                f"({fl.get('pick_location')}; floor binding {fl.get('floor_binding')}; without the floor "
                f"{fl.get('pick_without_floor')}); R_yaw used {r['r_pick']['R_yaw_rate_var']:.3e} (rad/s)^2 "
                f"(r_yaw_scale {r['r_pick']['r_yaw_scale']:.4g}); selection {r['r_selection']}")
            ex_ = r["nis_exceed"] or {}
            log(f"  gates: NIS gate {r['gate']}; sign check {r['sign_check']}; exceedance yaw "
                f"{ex_.get('yaw_rate_exceedance', float('nan')):.3f} ay {ex_.get('ay_exceedance', float('nan')):.3f}; "
                f"h2 {r['h2']}")
            for a, g in r["d4"].items():
                log(f"  D4-L {a}: K {g['stiffness_norm_per_rad']}, span {g['alpha_span_deg']}, mu_demo {g['mu']}, "
                    f"failed {g['failed']}")
            if "hard_bound" in r:
                log(f"  hard-bound re-entries {r['hard_bound']}, soft resets {r['soft_resets']}, NIS mean "
                    f"{r['nis_mean']:.3f} median {r['nis_median']:.3f}, straight-line |beta| median "
                    f"{r['beta_val_median_deg']:.3f} deg on {r['val_straight_s']:.0f} s")
        log("=" * 100)
        hb = [r["hard_bound"] for r in results if "hard_bound" in r]
        log(f"SUM hard-bound re-entries over {len(hb)} sessions with an observer run: {sum(hb)}; fallbacks "
            f"{sum(bool(r['fallback']) for r in results)}")
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable(results), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
