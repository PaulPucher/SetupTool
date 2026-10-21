# Washout cost on the FINAL chain (author EXECUTE 2026-10-21, read-only).
# Re-run of the Q-PR-ANOMALY H-alpha test (diagnostics/inspect_pr_anomaly.py,
# 2026-10-18, which used the pre-final fit L with C 1.3 and mu / B fitted) on
# the shipped fit L (C 1.0, E -1, mu = demonstrated utilisation, B fitted),
# schema 18, 14 development sessions, both axles. No module or config change.
#
# Fit inputs are captured from tyre_fit_auto._fit_axle_l itself (wrapped in
# memory): kinematic-seed slip angle (2 Hz), Fy (front in the tyre frame,
# Goblirsch Eq. 13; rear body), measured axle Fz, and the final fit
# population mask (base mask x speed window x braking filter), with the
# fitted B and mu. Definitions as in inspect_pr_anomaly.py:
# - unit: corner instance = canonical bracket on an analysis lap; per axle
#   at least 20 fit samples inside it, else dropped;
# - residual r = Fy/Fz - mu f(alpha; B, C 1.0, E -1); instance residual =
#   mean of r x sign(median Fy/Fz) -- positive = more |Fy/Fz| than the curve;
# - corner duration = bracket duration [s]; time since the last straight at
#   the instance midpoint, straight sample = |a_y| (2 Hz) < 1 m/s^2;
# - Spearman rho (scipy.stats.spearmanr) of the instance residual with both.
# Output diagnostics/plots/washout_final/ (cache/, washout_final.txt).
# Run: python -m diagnostics.inspect_washout_final [report]

import copy
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "washout_final")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
MIN_SAMPLES = 20      # as inspect_pr_anomaly.py
STRAIGHT_AY = 1.0     # m/s^2, as inspect_pr_anomaly.py
WORKERS = 3


def compute(key):
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state, _butterworth_lowpass
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.tyre_model_pacejka import pacejka_lateral_force
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    captured = []
    orig = tfa._fit_axle_l

    def wrap(alpha, Fy, Fz, base_mask, *a, **kw):
        out = orig(alpha, Fy, Fz, base_mask, *a, **kw)
        captured.append({"alpha": alpha.copy(), "Fy": np.asarray(Fy).copy(), "Fz": np.asarray(Fz).copy(),
                         "mask": base_mask.copy(), "B": out["B"], "mu": out["mu"]})
        return out

    tfa._fit_axle_l = wrap
    try:
        man = tfa.fit_session_pacejka(data, p, data_file_path=path, load_normalised=False,
                                      tyre_fit_mode="literature_bounded")
    finally:
        tfa._fit_axle_l = orig
    assert len(captured) == 2, f"expected front and rear fit, got {len(captured)}"
    t, s_m = st["time"], st["s_m"]
    ay_f = _butterworth_lowpass(st["ay_mps2"], p["stability_estimation"]["cs_filter_cutoff_hz"],
                                st["sample_rate_hz"])
    straight = np.abs(ay_f) < STRAIGHT_AY
    idx = np.where(straight, np.arange(t.size), -1)
    last = np.maximum.accumulate(idx)
    since = np.where(last >= 0, t - t[np.maximum(last, 0)], np.nan)
    laps = {l["lap_number"]: l for l in data["laps"]}
    rows = []
    for c in data["corners"]:
        lap = laps.get(c.get("lap_number"))
        if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is None:
            continue
        sl = tfa._canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                         c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        mid = (sl.start + sl.stop) // 2
        for axle, cap in zip(("front", "rear"), captured):
            al, fy, fz = cap["alpha"][sl], cap["Fy"][sl], cap["Fz"][sl]
            pop = cap["mask"][sl] & np.isfinite(al) & np.isfinite(fy) & np.isfinite(fz) & (fz > 0)
            if pop.sum() < MIN_SAMPLES:
                continue
            y = fy[pop] / fz[pop]
            f = cap["mu"] * pacejka_lateral_force(al[pop], cap["B"], tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)
            sgn = np.sign(np.median(y)) or 1.0
            rows.append({"axle": axle, "sid": c["stable_corner_id"], "lap": c["lap_number"], "n": int(pop.sum()),
                         "resid": float(np.mean((y - f) * sgn)),
                         "duration_s": float(t[sl.stop - 1] - t[sl.start]), "since_straight_s": float(since[mid])})
    out = {"key": key, "label": label, "status": man.get("status"), "rows": rows,
           "fit": {a: {"B": c["B"], "mu": c["mu"]} for a, c in zip(("front", "rear"), captured)}}
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] status {out['status']}, instances {len(rows)}", flush=True)
    return key


def report():
    from scipy.stats import spearmanr
    L = ["WASHOUT COST ON THE FINAL CHAIN (fit L: C 1.0, E -1, mu = demonstrated utilisation; schema 18)",
         "per session and axle: n corner instances; Spearman rho (p) of the turn-signed instance residual with "
         "corner duration | with time since the last straight; median residual", ""]
    for k in POOL:
        r = pickle.load(open(os.path.join(CACHE, f"{k}.pkl"), "rb"))
        parts = []
        for axle in ("front", "rear"):
            rows = [x for x in r["rows"] if x["axle"] == axle]
            res = np.array([x["resid"] for x in rows])
            dur = np.array([x["duration_s"] for x in rows])
            ss = np.array([x["since_straight_s"] for x in rows])
            ok = np.isfinite(ss)
            r1, p1 = spearmanr(dur, res) if len(rows) >= 10 else (np.nan, np.nan)
            r2, p2 = spearmanr(ss[ok], res[ok]) if ok.sum() >= 10 else (np.nan, np.nan)
            parts.append(f"{axle} n {len(rows)}: duration rho {r1:+.2f} (p {p1:.1e}) | since straight rho {r2:+.2f} "
                         f"(p {p2:.1e}, n {int(ok.sum())}); median residual {np.median(res):+.3f}")
        L.append(f"{r['label']} (status {r['status']}): " + "; ".join(parts))
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "washout_final.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    from diagnostics.inspect_fit_lit import keep_awake
    keep_awake(True)
    try:
        todo = [k for k in POOL if not os.path.exists(os.path.join(CACHE, f"{k}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, todo):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
