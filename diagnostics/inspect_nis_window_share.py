# NIS window-share census on the fit-L default (author EXECUTE 2026-10-22,
# read-only). The session gate scores only the share of base-mask samples
# whose trailing 0.4 s window has a combined-NIS exceedance share inside
# [nis_band_low, nis_band_high]; this script splits the rest into below and
# above the band, per session, to show where the out-of-band mass sits.
# Window statistic identical to modules.nis_gate._score_and_skips (NaN NIS
# leaves numerator and denominator, windows under half covered are skipped
# and leave the denominator). Acceptance: the in-band share equals the gate
# health_score; a mismatch is reported, never fitted away.
# Pool = the 14 development sessions, chain as in inspect_closeout_l.py.
# Output diagnostics/results_nis_window_share_20261022/ (cache/,
# per_session.txt / .json; run.log from the shell).
# Run: python -m diagnostics.inspect_nis_window_share [report]

import copy
import json
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# Development-set file list, the 14 entries of the retired hub
# inspect_data_quality.py (inlined in the 2026-10-22 pruning, unchanged):
# (key, track, outing, label, ("rel", path under the data root)).
_NBR = "unlabeled/02_NBR_CRV_data"
CANDIDATES = (
    ("nbr_o05_mla", "NBR", 5, "NBR outing 5 MLA", ("rel", f"{_NBR}/01_SE_01/NBR-73-005-001-MLA-26.07.01.0919.txt")),
    ("nbr_o10_jdr", "NBR", 10, "NBR outing 10 JDR", ("rel", f"{_NBR}/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt")),
    ("nbr_o20_jst", "NBR", 20, "NBR outing 20 JST", ("rel", f"{_NBR}/01_SE_02/NBR-73-020-065-JST-26.07.01.1454.txt")),
    ("nbr_o29_jst", "NBR", 29, "NBR outing 29 JST (NBR_Sturm_Solution copy)",
     ("rel", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt")),
    ("nbr_o46_jst", "NBR", 46, "NBR outing 46 JST", ("rel", f"{_NBR}/01_TE_04/NBR-73-046-162-JST-26.07.02.1411.txt")),
    ("nbr_o53_mla", "NBR", 53, "NBR outing 53 MLA", ("rel", f"{_NBR}/01_TE_04/NBR-73-053-207-MLA-26.07.02.1707.txt")),
    ("nbr_o54_cdr", "NBR", 54, "NBR outing 54 CDR", ("rel", f"{_NBR}/01_TE_04/NBR-73-054-214-CDR -26.07.02.1727.txt")),
    ("nbr_o59_mla", "NBR", 59, "NBR outing 59 MLA", ("rel", f"{_NBR}/New folder/NBR-73-059-232-MLA-26.07.03.1210.txt")),
    ("nbr_o61_jdr", "NBR", 61, "NBR outing 61 JDR", ("rel", f"{_NBR}/New folder/NBR-73-061-238-JDR-26.07.03.1229.txt")),
    ("nbr_o63_cdr", "NBR", 63, "NBR outing 63 CDR", ("rel", f"{_NBR}/New folder/NBR-73-063-256-CDR-26.07.03.1309.txt")),
    ("nbr_qp_cdr", "NBR", 69, "NBR qualifying CDR (outing 69)", ("rel", "NBR_Qualy_CDr/CDR-QP01-NBR-v1.txt")),
    ("nbr_qp_mla", "NBR", 70, "NBR qualifying MLA (outing 70)", ("rel", "NBR_Qualy_MLA/MLA-QP01-NBR-v2.txt")),
    ("pr_o25_us_v1", "PR", 25, "Le Castellet outing 25, MLA-SE07_US-v1", ("rel", "Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt")),
    ("pr_o32_us_v2", "PR", 32, "Le Castellet outing 32, MLA-SE07_US-v2", ("rel", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt")),
)


def keep_awake(on):
    # multi-hour runs: stop Windows from sleeping (from the retired hub
    # inspect_fit_lit.py, inlined 2026-10-22)
    if os.name == "nt":
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | (0x00000001 if on else 0))

OUT_DIR = os.path.join(ROOT, "diagnostics", "results_nis_window_share_20261022")
CACHE = os.path.join(OUT_DIR, "cache")
POOL = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
        "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
WORKERS = 3


def window_shares(nis_full, mask, window_samples, band_low, band_high):
    # same cumulative-sum window as nis_gate._score_and_skips, kept inline so
    # the three-way split reads the very fractions the gate reads
    from modules.nis_gate import _CHI2_DF2_95, MIN_WINDOW_VALID_FRACTION
    nis = np.asarray(nis_full, dtype=float)
    n = nis.size
    finite = np.isfinite(nis)
    exceed = (finite & (nis > _CHI2_DF2_95)).astype(float)
    cw = np.cumsum(np.insert(exceed, 0, 0.0))
    cv = np.cumsum(np.insert(finite.astype(float), 0, 0.0))
    ends = np.arange(window_samples - 1, n)
    n_exceed = cw[ends + 1] - cw[ends + 1 - window_samples]
    n_valid = cv[ends + 1] - cv[ends + 1 - window_samples]
    covered = n_valid >= MIN_WINDOW_VALID_FRACTION * window_samples
    win_frac = np.full(n, np.nan)
    win_frac[ends[covered]] = n_exceed[covered] / n_valid[covered]
    low_coverage = np.zeros(n, dtype=bool)
    low_coverage[ends[~covered]] = True
    m = np.asarray(mask, dtype=bool)
    valid = m & np.isfinite(win_frac)
    f = win_frac[valid]
    return {
        "n_masked": int(m.sum()), "n_valid": int(valid.sum()),
        "n_skipped_low_coverage": int((m & low_coverage).sum()),
        "n_below": int((f < band_low).sum()),
        "n_in": int(((f >= band_low) & (f <= band_high)).sum()),
        "n_above": int((f > band_high).sum()),
    }


def compute(key):
    import modules.tyre_fit_auto as tfa
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.nis_gate import _score_and_skips, resolve_nis_window_samples
    t0 = time.perf_counter()
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    # _resolve, not the public wrapper: its raw manifest still holds nis_full
    # and base_mask, which the public manifest strips
    _beta, man, gate, fb, fr, raw = tfa._resolve(st, p, data, p["stability_estimation"]["sideslip_source"], path)
    out = {"key": key, "label": label, "status": (man or {}).get("status"), "fallback": fb,
           "fallback_reason": fr, "gate": gate}
    if gate is not None and raw is not None:
        g = p["nis_gate"]
        ws = resolve_nis_window_samples(p, st["sample_rate_hz"])
        out["window_samples"] = ws
        out["shares"] = window_shares(raw["nis_full"], raw["base_mask"], ws, g["nis_band_low"], g["nis_band_high"])
        # independent re-call of the gate's own function, beside the inline split
        out["score_recomputed"] = _score_and_skips(raw["nis_full"], raw["base_mask"], ws,
                                                   g["nis_band_low"], g["nis_band_high"])[0]
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] gate {gate and gate['verdict']} {gate and gate['health_score']}, "
          f"{time.perf_counter() - t0:.0f} s", flush=True)
    return key


def report():
    from modules.stability_analysis import load_parameters
    g = load_parameters()["nis_gate"]
    res = {k: pickle.load(open(os.path.join(CACHE, f"{k}.pkl"), "rb")) for k in POOL}
    L = [f"NIS WINDOW-SHARE CENSUS, fit-L default, base_mask, window {g['nis_window_s']} s, "
         f"band [{g['nis_band_low']}, {g['nis_band_high']}] (inclusive), chi2_2 95 %",
         "share = samples / valid samples (masked, window at least half covered); skipped = masked samples "
         "whose window is under half covered (outside the denominator)", ""]
    tot = {"n_valid": 0, "n_below": 0, "n_in": 0, "n_above": 0, "n_skipped_low_coverage": 0}
    per = {}
    mismatches = []
    for k in POOL:
        r = res[k]
        if "shares" not in r:
            L.append(f"  {r['label']}: no gate (status {r['status']}, fallback {r['fallback_reason']}) -- not counted")
            mismatches.append(k)
            continue
        s = r["shares"]
        nv = s["n_valid"]
        below, inb, above = s["n_below"] / nv, s["n_in"] / nv, s["n_above"] / nv
        hs = r["gate"]["health_score"]
        ok = round(inb, 4) == round(hs, 4) and round(r["score_recomputed"], 4) == round(hs, 4)
        if not ok:
            mismatches.append(k)
        per[k] = {"label": r["label"], "below": below, "in_band": inb, "above": above,
                  "skipped": s["n_skipped_low_coverage"], "n_valid": nv, "n_masked": s["n_masked"],
                  "gate_health_score": hs, "gate_verdict": r["gate"]["verdict"], "acceptance_ok": ok}
        for kk in tot:
            tot[kk] += s[kk]
        L.append(f"  {r['label']}: below {below:.4f}, in band {inb:.4f}, above {above:.4f}; skipped "
                 f"{s['n_skipped_low_coverage']} (valid {nv} of {s['n_masked']} masked); gate health_score "
                 f"{hs:.4f} -> {'MATCH' if ok else 'MISMATCH'}")
    pooled = {k: tot[f"n_{k}"] / tot["n_valid"] for k in ("below", "in", "above")} if tot["n_valid"] else {}
    L += ["", f"POOLED, sample-weighted over all valid masked samples (n {tot['n_valid']}, skipped "
              f"{tot['n_skipped_low_coverage']}): below {pooled['below']:.4f}, in band {pooled['in']:.4f}, "
              f"above {pooled['above']:.4f}"]
    for name in ("below", "in_band", "above"):
        v = np.array([per[k][name] for k in per])
        L.append(f"PER-SESSION {name}: median {np.median(v):.4f}, min {v.min():.4f}, max {v.max():.4f} (n {v.size})")
    L += ["", f"ACCEPTANCE (in-band share == gate health_score to 4 decimals): "
              + ("all 14 match" if not mismatches else f"MISMATCH on {', '.join(mismatches)}")]
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "per_session.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT_DIR, "per_session.json"), "w") as fh:
        json.dump({"per_session": per, "pooled_sample_weighted": pooled, "totals": tot,
                   "mismatches": mismatches, "nis_gate": {k: g[k] for k in ("nis_window_s", "nis_band_low",
                                                                            "nis_band_high")}}, fh, indent=1)
    print(text)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
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
