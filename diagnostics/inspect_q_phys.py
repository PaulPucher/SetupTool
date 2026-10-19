# Q-PHYS (author order 2026-10-19, step of Q-OBS-DERIVE): physical process
# noise for the fit-L observer from the fit-L model error. Read-only; no module
# or config change. Frozen frame: DECISIONS.md "Observer derivation frozen
# frame" (Q per session from the model error; Villano et al. 2021 Eqs. 28-29 Q
# form, Bechtloff ch. 5 Eq. 5.16 physically meaningful variances).
#
# Per session (the 14-session analysis set; outings 10, 29, 32 are the three
# sessions used so far): the fit-L chain runs up to both axle fits (wrapping
# tyre_fit_auto._fit_axle_l and _axle_load_inputs) and stops before the
# observer. On the joint fit population (both axles' fit masks):
#   force residual per axle, Fy - mu Fz f(alpha) (front in the tyre frame,
#     brought to the body frame with cos(delta), Goblirsch et al. 2025 Eq. 13);
#   e_beta_dot = (dFy_f + dFy_r) / (m Vx), e_r_dot = (a dFy_f - b dFy_r) / Iz,
#     sample-wise, both axles jointly (front/rear correlation kept), Vx floored
#     at moving_speed_min_mps as in the observer;
#   SD, mean, p50 / p95 of |e|, autocorrelation time (integral to the first
#     zero crossing, and the 1/e time) on the masked series;
#   per-step Q under two assumptions, in the config convention (per step at
#     q_reference_rate_hz): held over one step (sigma^2 dt^2) and equivalent
#     white noise of a correlated error (2 sigma^2 tau dt, the zero-frequency
#     spectral density of the error), each as a factor of the current Q.
# The correlation-time floor of the 2 Hz zero-phase CS filter alone (filtered
# white noise) is reported for comparison.
# Output diagnostics/plots/q_phys/ (run.log, q_phys_<key>.json).
# Run: python -m diagnostics.inspect_q_phys

import copy
import json
import os
import sys

import numpy as np
from scipy.signal import fftconvolve

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import keep_awake, _jsonable  # noqa: E402
from diagnostics.inspect_data_quality import CANDIDATES  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from tests.conftest import resolve_data_path  # noqa: E402

OUT_DIR = "diagnostics/plots/q_phys"
ANALYSIS = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
            "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
MAX_LAG_S = 10.0

_LOG = None


class _Stop(Exception):
    pass


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def acf_time(x, m, sr):
    """Integral time (rectangle rule up to the first non-positive lag) and 1/e
    time of the masked, mean-removed series; pairs count only where both
    samples are in the mask."""
    x = np.where(m, x - x[m].mean(), 0.0)
    mf = m.astype(float)
    k = int(MAX_LAG_S * sr)
    num = fftconvolve(x, x[::-1])[len(x) - 1:len(x) - 1 + k]
    den = fftconvolve(mf, mf[::-1])[len(x) - 1:len(x) - 1 + k]
    rho = (num / np.maximum(den, 1)) / (num[0] / den[0])
    k0 = int(np.argmax(rho <= 0)) if np.any(rho <= 0) else k
    k_e = int(np.argmax(rho < np.exp(-1))) if np.any(rho < np.exp(-1)) else k
    return {"tau_int_s": float(rho[:k0].sum() / sr), "tau_1e_s": k_e / sr, "rho_at_0p5s": float(rho[int(0.5 * sr)]),
            "first_zero_s": k0 / sr}


def stats(e, m):
    v = e[m]
    return {"mean": float(v.mean()), "sd": float(v.std()), "p50_abs": float(np.percentile(np.abs(v), 50)),
            "p95_abs": float(np.percentile(np.abs(v), 95)), "n": int(m.sum())}


def main():
    global _LOG
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state, _butterworth_lowpass
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    cand = {c[0]: c for c in CANDIDATES}
    orig_fit, orig_load = tfa._fit_axle_l, tfa._axle_load_inputs
    try:
        base = load_parameters()
        cfg0 = base["tyre_fit_auto"]
        q_cur = {"beta": cfg0["q_beta_var"], "r": cfg0["q_yaw_rate_var"]}
        dt_ref = 1.0 / cfg0["q_reference_rate_hz"]
        log(f"current Q per step at {cfg0['q_reference_rate_hz']} Hz: q_beta_var {q_cur['beta']:.4e} rad^2, "
            f"q_yaw_rate_var {q_cur['r']:.4e} (rad/s)^2")
        floor = None
        for key in ANALYSIS:
            _, track, outing, label, src = cand[key]
            path = resolve_data_path(src[1])
            data = parse_csv(path)
            reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
            p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                                     resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
            p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
            st = prepare_vehicle_state(data["channels"], p)
            sr = st["sample_rate_hz"]
            if floor is None:
                rng = np.random.default_rng(0)
                w = _butterworth_lowpass(rng.standard_normal(200000), p["stability_estimation"]["cs_filter_cutoff_hz"], sr)
                floor = acf_time(w, np.ones(w.size, dtype=bool), sr)
                log(f"2 Hz zero-phase filter floor (filtered white noise, {sr:g} Hz): tau_int {floor['tau_int_s']:.3f} s,"
                    f" tau_1/e {floor['tau_1e_s']:.3f} s")
            cap = {"fits": []}

            def fit_wrap(alpha, Fy, Fz, mask, bin_rad, mu_pct, weighted=True):
                r = orig_fit(alpha, Fy, Fz, mask, bin_rad, mu_pct, weighted=weighted)
                cap["fits"].append(r)
                if len(cap["fits"]) == 2:
                    raise _Stop
                return r

            def load_wrap(*a, **kw):
                cap["load"] = orig_load(*a, **kw)
                return cap["load"]

            tfa._fit_axle_l, tfa._axle_load_inputs = fit_wrap, load_wrap
            try:
                man = tfa.fit_session_pacejka(data, p, data_file_path=path, tyre_fit_mode="literature_bounded")
            except _Stop:
                man = None
            finally:
                tfa._fit_axle_l, tfa._axle_load_inputs = orig_fit, orig_load
            log("=" * 100)
            if man is not None:
                log(f"{label}: fit L did not reach both axle fits ({man.get('degenerate_reason')})")
                continue
            ff, fr = cap["fits"]
            li = cap["load"]
            n = st["v_mps"].size
            res = {}
            for name, fit in (("front", ff), ("rear", fr)):
                full = np.full(n, np.nan)
                full[fit["residual_mask"]] = fit["residuals"]
                res[name] = full
            # front residual is in the tyre frame; body frame adds Fx sin(delta)
            # on both sides of the residual, so only cos(delta) remains
            dfy_f = res["front"] * np.cos(li["delta_f"])
            dfy_r = res["rear"]
            joint = ff["residual_mask"] & fr["residual_mask"]
            vp = p["vehicle"]
            m, a, b, iz = vp["mass_kg"], vp["cog_to_front_axle_m"], vp["cog_to_rear_axle_m"], vp["yaw_inertia_kgm2"]
            vx = np.maximum(st["v_mps"], p["stability_estimation"]["moving_speed_min_mps"])
            e_b = (dfy_f + dfy_r) / (m * vx)
            e_r = (a * dfy_f - b * dfy_r) / iz
            rec = {"label": label, "track": track, "outing": outing, "file": path, "sample_rate_hz": sr,
                   "B": {"front": ff["B"], "rear": fr["B"]}, "mu": {"front": ff["mu"], "rear": fr["mu"]},
                   "n_fit": {"front": ff["fit_n_samples"], "rear": fr["fit_n_samples"]}, "n_joint": int(joint.sum()),
                   "v_p50_mps": float(np.median(vx[joint])), "axles": {}, "filter_floor": floor}
            for name, d, fz in (("front", dfy_f, li["fz_f_N"]), ("rear", dfy_r, li["fz_r_N"])):
                mk = joint & np.isfinite(d)
                rec["axles"][name] = dict(stats(d, mk), norm_sd=float((d / fz)[mk].std()), **acf_time(d, mk, sr))
            rec["corr_front_rear_resid"] = float(np.corrcoef(dfy_f[joint], dfy_r[joint])[0, 1])
            rec["corr_ebeta_er"] = float(np.corrcoef(e_b[joint], e_r[joint])[0, 1])
            for nm, e in (("beta", e_b), ("r", e_r)):
                s = dict(stats(e, joint), **acf_time(e, joint, sr))
                s["q_held_step"] = s["sd"] ** 2 * dt_ref ** 2
                s["q_equiv_white"] = 2 * s["sd"] ** 2 * s["tau_int_s"] * dt_ref
                s["factor_held"] = s["q_held_step"] / q_cur[nm]
                s["factor_equiv_white"] = s["q_equiv_white"] / q_cur[nm]
                rec[f"e_{nm}_dot"] = s
            log(f"{label} (track {track}, outing {outing}): B front {ff['B']:.3f} rear {fr['B']:.3f}, mu_demo front "
                f"{ff['mu']:.3f} rear {fr['mu']:.3f}; joint fit population n {rec['n_joint']} (front {ff['fit_n_samples']},"
                f" rear {fr['fit_n_samples']}); Vx p50 {rec['v_p50_mps']:.1f} m/s")
            for name in ("front", "rear"):
                x = rec["axles"][name]
                log(f"  {name} force residual: SD {x['sd']:.0f} N (normalised {x['norm_sd']:.4f}), mean {x['mean']:.0f} N, "
                    f"tau_int {x['tau_int_s']:.3f} s, tau_1/e {x['tau_1e_s']:.3f} s, rho(0.5 s) {x['rho_at_0p5s']:.3f}")
            log(f"  corr(front, rear residual) {rec['corr_front_rear_resid']:.3f}; corr(e_beta_dot, e_r_dot) "
                f"{rec['corr_ebeta_er']:.3f}")
            for nm, unit in (("beta", "rad/s"), ("r", "rad/s^2")):
                s = rec[f"e_{nm}_dot"]
                log(f"  e_{nm}_dot [{unit}]: SD {s['sd']:.4e}, mean {s['mean']:.3e}, p50|e| {s['p50_abs']:.4e}, "
                    f"p95|e| {s['p95_abs']:.4e}; tau_int {s['tau_int_s']:.3f} s, tau_1/e {s['tau_1e_s']:.3f} s")
                log(f"     Q per step: held-over-step {s['q_held_step']:.3e} (x{s['factor_held']:.3g} current), "
                    f"equivalent white {s['q_equiv_white']:.3e} (x{s['factor_equiv_white']:.3g} current)")
            with open(os.path.join(OUT_DIR, f"q_phys_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1, default=str)
        return 0
    finally:
        tfa._fit_axle_l, tfa._axle_load_inputs = orig_fit, orig_load
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    sys.exit(main())
