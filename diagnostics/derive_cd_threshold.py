# h derivation for the change-detector trigger of the fit-L observer (author
# EXECUTE 2026-10-19). Frame written before any number: DECISIONS.md "h
# derivation frame" and thesis_notes "h derivation for the change-detector
# trigger". Read-only: no module or config change, default trigger untouched.
#
# Derive split: NBR outings 5, 20, 46, 54, 61, 69 and Le Castellet 25.
# Per session the shipped fit-L chain (NIS-window trigger) runs once and its
# inputs are cached; per grid point only the final observer re-runs, with the
# change detector (d = l_cd_drift, h from the grid) and R held at the cached
# picks (declared simplification; the acceptance run re-derives R).
# Per point, per session and summed: hard-bound re-entries (base mask), alarm
# count (resets that are not hard-bound), mean time between alarms on the
# validation straights (odd-indexed steering-defined straights,
# inspect_kinematic_inputs.steering_straights, split as in
# check_estimator_hardening.py), NIS-gate value. The shipped NIS-window
# trigger is the reference line (reported, not a criterion).
# Selection: smallest summed hard-bound re-entries among points with a pooled
# mean time between alarms >= 10 s; tie -> fewer alarms. No pick if no point
# qualifies, if the objective is flat across the grid, or if the pooled and
# the every-session reading of the constraint pick different h.
#
# Cache: one pickle per session in diagnostics/output/cd_threshold_cache/
# (gitignored), keyed by a hash of modules/*.py, config/parameters.json,
# inspect_kinematic_inputs.py and the data file's size and mtime; reused by
# inspect_cs_dips.py. Sessions run in 3 spawn-safe worker processes.
# Output diagnostics/plots/cd_threshold/ (run.log, results.json).
# Run: python -m diagnostics.derive_cd_threshold

import copy
import glob
import hashlib
import json
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

CACHE_DIR = os.path.join(ROOT, "diagnostics", "output", "cd_threshold_cache")
OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "cd_threshold")
DERIVE = ("nbr_o05_mla", "nbr_o20_jst", "nbr_o46_jst", "nbr_o54_cdr", "nbr_o61_jdr", "nbr_qp_cdr", "pr_o25_us_v1")
GRID = (100, 200, 400, 800, 1600, 3200)
MTBA_MIN_S = 10.0  # declared Tier B, DECISIONS.md "h derivation frame"
WORKERS = 3
CACHE_FORMAT = "1"


def _source_hash(data_path):
    h = hashlib.sha256(CACHE_FORMAT.encode())
    files = sorted(glob.glob(os.path.join(ROOT, "modules", "*.py")))
    files += [os.path.join(ROOT, "config", "parameters.json"),
              os.path.join(ROOT, "diagnostics", "inspect_kinematic_inputs.py")]
    for f in files:
        with open(f, "rb") as fh:
            h.update(fh.read())
    st = os.stat(data_path)
    h.update(f"{data_path}|{st.st_size}|{st.st_mtime_ns}".encode())
    return h.hexdigest()


def _validation_straights(st, laps):
    from diagnostics.inspect_kinematic_inputs import steering_straights
    t = st["time"]
    valid = [l for l in laps if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in valid:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    segs = sorted(steering_straights(st, valid, lap_id, kerb)["segs"])
    mask = np.zeros(t.size, dtype=bool)
    for a, b in segs[1::2]:
        mask[a:b] = True
    return mask


def build_cache(key):
    """Run the shipped fit-L chain once and keep what the grid and the dip
    census need. Returns the cache dict (also written to disk)."""
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
    st = prepare_vehicle_state(data["channels"], p)
    held = {}
    o_fsp, o_ekf, o_load = tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka, tfa._axle_load_inputs

    def fsp(*a, **kw):
        held["raw"] = o_fsp(*a, **kw)
        return held["raw"]

    def ekf(state, params, pass_id=None, **kw):
        out = o_ekf(state, params, pass_id=pass_id, **kw)
        if pass_id == "_auto_final":
            held["final"], held["final_cfg"] = out, params["tyre_model_ekf_pacejka"]["_auto_final"]
            held["loads"] = {k: state[k] for k in ("fz_axle_f_N", "fz_axle_r_N", "fx_axle_f_N") if k in state}
        return out

    def load(*a, **kw):
        held["load_in"] = o_load(*a, **kw)
        return held["load_in"]

    tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka, tfa._axle_load_inputs = fsp, ekf, load
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, data, "ekf_auto_pacejka", csv_path=path)
    finally:
        tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka, tfa._axle_load_inputs = o_fsp, o_ekf, o_load
    raw = held.get("raw") or {}
    li = held.get("load_in") or {}
    cache = {
        "key": key, "label": label, "track": track, "outing": outing, "path": path, "hash": _source_hash(path),
        "params": p, "state": st, "laps": data["laps"], "corners": data["corners"],
        "beta_shipped": beta, "fallback": fb, "fallback_reason": fr, "status": man.get("status"),
        "gate": gate and {k: gate[k] for k in ("verdict", "health_score")},
        "r_chosen": (man.get("r_sweep") or {}).get("chosen") and
                    {k: man["r_sweep"]["chosen"][k] for k in ("r_ay_scale", "r_yaw_scale")},
        "axles": {a: {k: v for k, v in man["axles"][a].items() if not isinstance(v, np.ndarray)}
                  for a in ("front", "rear")} if "axles" in man else None,
        "base_mask": raw.get("base_mask"), "observer_reset_mask": raw.get("observer_reset_mask"),
        "final": held.get("final"), "final_cfg": held.get("final_cfg"), "loads": held.get("loads"),
        "load_in": {k: li[k] for k in ("fz_f_N", "fz_r_N", "fx_f_N", "fx_r_N", "delta_f") if k in li},
        "val_straights": _validation_straights(st, data["laps"]),
    }
    os.makedirs(CACHE_DIR, exist_ok=True)
    with open(os.path.join(CACHE_DIR, f"{key}.pkl"), "wb") as fh:
        pickle.dump(cache, fh, protocol=pickle.HIGHEST_PROTOCOL)
    return cache


def load_cache(key):
    """Cached session if its hash still matches the sources, else rebuilt.
    Returns (cache, seconds spent building, 0 when reused)."""
    f = os.path.join(CACHE_DIR, f"{key}.pkl")
    if os.path.exists(f):
        with open(f, "rb") as fh:
            c = pickle.load(fh)
        if c.get("hash") == _source_hash(c["path"]):
            return c, 0.0
    t0 = time.perf_counter()
    c = build_cache(key)
    return c, time.perf_counter() - t0


def _metrics(res, c, sr):
    base, val = c["base_mask"], c["val_straights"]
    bound = np.radians(c["params"]["tyre_fit_auto"]["beta_hard_bound_deg"])
    alarms = res["diverged_mask"] & ~res["hard_bound_mask"]
    n_val = int(alarms[val].sum())
    t_val = val.sum() / sr
    from modules.nis_gate import evaluate_gate
    g = evaluate_gate(res["nis"], base, c["params"], sr)
    return {"hard_bound": int(((np.abs(res["beta"]) > bound) & base).sum()),
            "alarms_base": int((alarms & base).sum()), "alarms_all": int(alarms.sum()),
            "alarms_val": n_val, "val_time_s": float(t_val),
            "mtba_val_s": float(t_val / n_val) if n_val else float("inf"),
            "gate": {"verdict": g["verdict"], "health_score": g["health_score"]}}


def run_session(key):
    """Worker: cache, the shipped reference line and the grid for one session."""
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
    c, t_build = load_cache(key)
    out = {"key": key, "label": c["label"], "build_s": t_build, "status": c["status"], "fallback": c["fallback"],
           "fallback_reason": c["fallback_reason"], "r_chosen": c["r_chosen"], "grid": {}}
    if c["final"] is None or c["fallback"]:
        out["note"] = "shipped chain fell back -- no fit-L observer to vary"
        return out
    st, sr = c["state"], c["state"]["sample_rate_hz"]
    ekf_state = dict(st, **c["loads"])
    p = dict(c["params"])
    p["tyre_model_ekf_pacejka"] = {"_ref": dict(c["final_cfg"])}
    t0 = time.perf_counter()
    ref = estimate_sideslip_ekf_pacejka(ekf_state, p, pass_id="_ref")
    out["ref_reproduces_shipped"] = bool(np.array_equal(ref["beta_with_fallback"], c["final"]["beta_with_fallback"]))
    out["ref"] = _metrics(c["final"], c, sr)
    d = c["params"]["tyre_fit_auto"]["l_cd_drift"]
    for h in GRID:
        cfg = dict(c["final_cfg"], divergence_trigger="change_detector", cd_drift=d, cd_threshold=float(h))
        p["tyre_model_ekf_pacejka"] = {"_cd": cfg}
        out["grid"][h] = _metrics(estimate_sideslip_ekf_pacejka(ekf_state, p, pass_id="_cd"), c, sr)
    out["grid_s"] = time.perf_counter() - t0
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
        ok = [r for r in results if "grid" in r and r["grid"]]
        for r in results:
            log("=" * 100)
            log(f"{r['label']}: status {r['status']}, fallback {r['fallback']}, R {r['r_chosen']}, cache build "
                f"{r['build_s']:.0f} s" + (f", grid {r['grid_s']:.0f} s" if "grid_s" in r else ""))
            if "note" in r:
                log(f"  {r['note']} ({r['fallback_reason']})")
                continue
            log(f"  reference rerun reproduces shipped beta: {r['ref_reproduces_shipped']}")
            rows = [("NIS window (shipped, reference)", r["ref"])] + [(f"h {h}", r["grid"][h]) for h in GRID]
            for name, m in rows:
                log(f"  {name}: hard-bound re-entries {m['hard_bound']}, alarms {m['alarms_base']} (base mask), "
                    f"{m['alarms_val']} on validation straights ({m['val_time_s']:.0f} s), mean time between alarms "
                    f"{m['mtba_val_s']:.1f} s, NIS gate {m['gate']['verdict']} {m['gate']['health_score']:.4f}")
        log("=" * 100)
        summed = {}
        for name, getter in [("ref", lambda r: r["ref"])] + [(h, (lambda hh: lambda r: r["grid"][hh])(h)) for h in GRID]:
            ms = [getter(r) for r in ok]
            n_val = sum(m["alarms_val"] for m in ms)
            t_val = sum(m["val_time_s"] for m in ms)
            summed[name] = {"hard_bound": sum(m["hard_bound"] for m in ms), "alarms": sum(m["alarms_base"] for m in ms),
                            "alarms_val": n_val, "val_time_s": t_val,
                            "mtba_pooled_s": t_val / n_val if n_val else float("inf"),
                            "mtba_min_session_s": min(m["mtba_val_s"] for m in ms),
                            "gate_pass": sum(m["gate"]["verdict"] == "pass" for m in ms)}
            s = summed[name]
            log(f"SUM {'NIS window (reference)' if name == 'ref' else 'h ' + str(name)} over {len(ms)} sessions: "
                f"hard-bound re-entries {s['hard_bound']}, alarms {s['alarms']}, pooled mean time between alarms on "
                f"validation straights {s['mtba_pooled_s']:.1f} s (lowest session {s['mtba_min_session_s']:.1f} s), "
                f"NIS gate pass {s['gate_pass']} / {len(ms)}")

        def pick(ok_fn):
            cand = [h for h in GRID if ok_fn(summed[h])]
            if not cand:
                return None, cand
            return min(cand, key=lambda h: (summed[h]["hard_bound"], summed[h]["alarms"])), cand

        pooled, c_pooled = pick(lambda s: s["mtba_pooled_s"] >= MTBA_MIN_S)
        every, c_every = pick(lambda s: s["mtba_min_session_s"] >= MTBA_MIN_S)
        hb = [summed[h]["hard_bound"] for h in GRID]
        log(f"constraint met (pooled): {c_pooled}; (every session): {c_every}")
        if not c_pooled:
            verdict = "NO PICK: no grid point meets the straight-line constraint"
        elif max(hb) == min(hb):
            verdict = "NO PICK: objective flat across the grid"
        elif pooled != every:
            verdict = f"NO PICK: pooled reading picks h {pooled}, every-session reading picks h {every}"
        else:
            verdict = f"SELECTED h = {pooled}"
        log(verdict)
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable({"results": results, "summed": {str(k): v for k, v in summed.items()},
                                 "verdict": verdict}), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
