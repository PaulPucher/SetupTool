# Fit-L fallback check (author STEP 2026-10-19, read-only): NBR outings 59 and
# 70 fail D4-L g2 on the front axle. Per analysis session and axle the D4-L
# inputs and values (B, C, mu_demo, K = B C mu against L_K_BAND, the g1 slip
# span against min_alpha_span_deg, n fit) recomputed with plausibility_gate_l
# on the cached chain inputs, and per session the weather / track-state
# channels read straight from the export (census rule): ambient temperature
# log_tamb, wiper state log_state_wiper, wet-tyre flag pcu_B_tyre_wet,
# gpsa_track_condition (whole file; medians and shares of non-zero samples).
# Caches: derive_cd_threshold's pickles, read directly -- their hash only
# differs by the text-only derived_from edit of 2026-10-19 (Q-UPPER frame).
# Output diagnostics/plots/fallback_59_70/ (run.log, results.json).
# Run: python -m diagnostics.inspect_fallback_59_70

import json
import os
import pickle
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from diagnostics.derive_cd_threshold import CACHE_DIR  # noqa: E402

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "fallback_59_70")
ANALYSIS = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
            "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
CHANNELS = ("log_tamb", "log_state_wiper", "pcu_B_tyre_wet", "gpsa_track_condition")


def read_channels(path, names):
    """Values of the named channel blocks of a Pi Toolbox ASCII export, read
    by streaming; stops once every block has been seen."""
    out, cur, want = {}, None, set(names)
    with open(path, "r", encoding="latin-1") as fh:
        for line in fh:
            if line.startswith("Time\t"):
                base = line.split("\t", 1)[1].strip().split("[")[0]
                cur = base if base in want else None
                if cur:
                    out[cur] = []
                continue
            if cur is None:
                continue
            if not line.strip():
                want.discard(cur)
                cur = None
                if not want:
                    break
                continue
            try:
                out[cur].append(float(line.split("\t")[1]))
            except (IndexError, ValueError):
                pass
    return {k: np.asarray(v) for k, v in out.items()}


def main():
    import modules.tyre_fit_auto as tfa
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles
    os.makedirs(OUT_DIR, exist_ok=True)
    log_f = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")

    def log(msg=""):
        print(msg, flush=True)
        log_f.write(msg + "\n")
        log_f.flush()

    log(f"D4-L: g2 K = B C mu in {tfa.L_K_BAND}, mu_demo ceiling {tfa.L_MU_CEILING}")
    res = {}
    try:
        for key in ANALYSIS:
            with open(os.path.join(CACHE_DIR, f"{key}.pkl"), "rb") as fh:
                c = pickle.load(fh)
            st, p = c["state"], c["params"]
            cfg = p["tyre_fit_auto"]
            window, _ = tfa._speed_window_mask(st, {"corners": c["corners"], "laps": c["laps"]}, cfg)
            fit_mask = c["base_mask"] & window
            slip = estimate_slip_angles(st, estimate_sideslip(st, p), p)
            rec = {"label": c["label"], "fallback": c["fallback"], "fallback_reason": c["fallback_reason"],
                   "laps": [l["lap_number"] for l in c["laps"] if l.get("is_valid_for_analysis")], "axles": {}}
            for axle, key_a in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
                fit = c["axles"][axle]
                g = tfa.plausibility_gate_l(slip[key_a], fit_mask, fit, cfg)
                rec["axles"][axle] = {"B": fit["B"], "C": fit["C"], "mu": fit["mu"], "n_fit": fit["fit_n_samples"],
                                      "K": g["stiffness_norm_per_rad"], "span_deg": g["alpha_span_deg"],
                                      "failed": g["failed"]}
            ch = read_channels(c["path"], CHANNELS)
            rec["weather"] = {k: {"n": int(v.size), "median": float(np.median(v)) if v.size else None,
                                  "min": float(v.min()) if v.size else None, "max": float(v.max()) if v.size else None,
                                  "share_nonzero": float((v != 0).mean()) if v.size else None}
                              for k, v in ch.items()}
            rec["weather_missing"] = [k for k in CHANNELS if k not in ch]
            res[key] = rec
            log("=" * 100)
            log(f"{c['label']}: fallback {c['fallback']}" + (f" ({c['fallback_reason']})" if c["fallback"] else "")
                + f"; analysis laps {rec['laps']}")
            for axle, x in rec["axles"].items():
                log(f"  {axle}: B {x['B']:.3f}, C {x['C']:.1f}, mu_demo {x['mu']:.3f}, K {x['K']:.3f} / rad, g1 span "
                    f"{x['span_deg']:.2f} deg (min {cfg['min_alpha_span_deg']}), n fit {x['n_fit']}, failed {x['failed']}")
            for k, w in rec["weather"].items():
                log(f"  {k}: n {w['n']}, median {w['median']}, min {w['min']}, max {w['max']}, non-zero share "
                    f"{w['share_nonzero']}")
            if rec["weather_missing"]:
                log(f"  channels not in the export: {rec['weather_missing']}")
        from diagnostics.inspect_fit_lit import _jsonable
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable(res), fh, indent=1, default=str)
        return 0
    finally:
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
