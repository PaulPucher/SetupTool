# Q-FIT-LIT final implementation run (author order and EXECUTE 2026-10-19).
# Read-only; mode literature_bounded as now implemented (C 1.0, E -1, mu =
# demonstrated utilisation p99.5, heavy braking out of the fit population,
# D4-L with the mu ceiling) set in memory; observer-reset mark on the CS.
# Information only, no comparison rule.
#
# Per session (NBR-73-010-026-JDR, NBR JST 29, MLA-SE07_US-v2) and axle: B, K,
#   mu_demo, samples removed by the braking filter (n, share of the fit
#   population), in-range curve change against the conditional run without
#   the braking filter (inspect_capacity.demo_fitter on the unfiltered fit
#   population; RMS / max over [p1, p99] of the visited slip, as a share of
#   that run's mu); chain function (status, D4-L, gate, fallback, R pick);
#   hard-bound re-entries (base mask, final run); share of verdict CS samples
#   (finite CS inside canonical brackets on analysis laps, moving, not kerb
#   mask, not the kerb-affected mark) marked observer_reset_affected, and the
#   summarise_corners count n_reset_excluded summed over phases.
# Output diagnostics/plots/fit_lit/final/ (run.log, final_<key>.json).
# Run: python -m diagnostics.inspect_fit_lit_final
#
# [2026-10-19 re-run after the braking-only clarification (author EXECUTE): also
#   reports per axle the share of fit samples above fit_long_util_max of either
#   sign (context) beside the removed braking share, the rear drive utilisation
#   |Fx_r| / Fz_r on drive samples of the fit population (p50 / p95 / p99), the
#   friction-circle factor sqrt(1 - (u / mu_demo_rear)^2) at p50 and p95 (an
#   estimate, Pacejka 2006 p. 5), n fit samples, gate checks with values and
#   the peak note. No CS values or verdict statistics.]

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session, capture, keep_awake, _jsonable  # noqa: E402
from diagnostics.inspect_capacity import demo_fitter  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force  # noqa: E402

OUT_DIR = "diagnostics/plots/fit_lit/final"
ORDER = ("nbr_jdr", "nbr_jst29", "pr_us_v2")
AXLES = ("front", "rear")

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def curve(fit, g):
    return fit["mu"] * pacejka_lateral_force(g, fit["B"], tfa.L_SHAPE_C, 1.0, tfa.L_CURVATURE_E)


def main():
    global _LOG
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners)
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    sessions = {k: (lab, rel) for k, lab, rel in SESSIONS}
    try:
        for key in ORDER:
            label, rel = sessions[key]
            S = load_session(key, label, rel)
            st, p = S["state"], S["params"]
            holder = {}
            with capture(holder, True):
                beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, S["data"], "ekf_auto_pacejka",
                                                                     csv_path=S["path"])
            raw, fin = holder.get("raw") or {}, holder.get("final")
            slip = estimate_slip_angles(st, beta, p)
            forces = estimate_lateral_forces(st, p)
            reset = None if fb else raw.get("observer_reset_mask")
            cs = estimate_cornering_stiffness(slip, forces, st, p, S["data"]["laps"], reset_mask=reset)
            stab = estimate_yaw_moment_stability(st, beta, p, S["data"]["laps"])
            summ = summarise_corners(S["data"]["corners"], cs, stab, st)
            rec = {"label": label, "status": man.get("status"), "reason": man.get("degenerate_reason") or
                   man.get("marginal_reason"), "gate": gate and {k: gate[k] for k in ("verdict", "health_score")},
                   "fallback": fb, "fallback_reason": fr,
                   "R": (man.get("r_sweep") or {}).get("chosen") and
                        {k: man["r_sweep"]["chosen"][k] for k in ("r_ay_scale", "r_yaw_scale")}, "axles": {}}
            bound = np.radians(p["tyre_fit_auto"]["beta_hard_bound_deg"])
            rec["hard_bound_reentries"] = (int(((np.abs(fin["beta"]) > bound) & S["base"]).sum())
                                           if fin is not None else None)
            rec["n_resets"] = int(reset.sum()) if reset is not None else 0
            br = (man.get("axle_load") or {}).get("braking_excluded", {})
            d4 = raw.get("plausibility_gate") or {}
            for a in AXLES:
                fit = man["axles"][a]
                cond = demo_fitter(S["alpha"][a], S["Fy"][a], S["Fz"][a], S["fit_mask"],
                                   p["tyre_fit_auto"]["fit_density_bin_rad"])
                pm = S["fit_mask"] & np.isfinite(S["alpha"][a])
                p1, p99 = np.percentile(S["alpha"][a][pm], [1, 99])
                g = np.linspace(p1, p99, 2000)
                d = np.abs(curve(fit, g) - curve(cond, g)) / cond["mu"]
                rec["axles"][a] = {"B": fit["B"], "K": fit["B"] * fit["C"] * fit["mu"], "mu_demo": fit["mu"],
                                   "mu_source": fit.get("mu_source"), "B_bound": fit["bound_active"]["B"],
                                   "braking_excluded": br.get(a), "d4": d4.get(a),
                                   "vs_no_braking_filter": {"rms": float(np.sqrt(np.mean(d ** 2))), "max": float(d.max()),
                                                            "B_cond": cond["B"], "mu_cond": cond["mu"]}}
            li = S["li"]
            thr = p["tyre_fit_auto"]["fit_long_util_max"]
            for a, fx_k, fz_k in (("front", "fx_f_N", "fz_f_N"), ("rear", "fx_r_N", "fz_r_N")):
                with np.errstate(invalid="ignore", divide="ignore"):
                    u = np.abs(li[fx_k]) / li[fz_k]
                m = S["fit_mask"]
                rec["axles"][a]["above_thr_either_sign_share"] = float((m & (u > thr)).sum() / max(1, m.sum()))
                rec["axles"][a]["n_fit"] = man["axles"][a]["fit_n_samples"]
                if a == "rear":
                    drive = m & (li[fx_k] > 0) & np.isfinite(u)
                    q = {k: float(np.percentile(u[drive], v)) for k, v in (("p50", 50), ("p95", 95), ("p99", 99))}
                    mu_r = man["axles"]["rear"]["mu"]
                    rec["rear_drive_util"] = dict(q, n=int(drive.sum()))
                    rec["friction_circle_factor"] = {k: float(np.sqrt(max(0.0, 1 - (q[k] / mu_r) ** 2)))
                                                     for k in ("p50", "p95")}
            kerb = st.get("kerb_mask")
            for ax in ("f", "r"):
                pop = S["in_br"] & st["moving_mask"] & ~cs[f"near_strike_{ax}"] & np.isfinite(cs[f"CS_ratio_{ax}"])
                if kerb is not None:
                    pop &= ~kerb
                mk = cs.get(f"observer_reset_affected_{ax}")
                n_m = int((mk[pop]).sum()) if mk is not None else 0
                rec[f"reset_affected_{ax}"] = {"n_verdict": int(pop.sum()), "n_marked": n_m,
                                               "share": n_m / max(1, int(pop.sum()))}
                rec[f"n_reset_excluded_phases_{ax}"] = int(sum(
                    ph[f"cs_ratio_{ax}"].get("n_reset_excluded", 0) for s in summ for ph in s["phases"].values()))
            with open(os.path.join(OUT_DIR, f"final_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1, default=str)
            log("=" * 100)
            shipped = "kinematic fallback" if fb else "EKF L"
            log(f"{label}: shipped {shipped}" + (f" ({fr})" if fb else "") + f"; status {rec['status']}"
                f" ({rec['reason'] or '-'}); gate {rec['gate']}; R {rec['R']}")
            for a in AXLES:
                x = rec["axles"][a]
                g4 = x["d4"] or {}
                failed = g4.get("failed") or []
                log(f"  {a}: removed (braking) {x['braking_excluded']['share'] * 100:.2f} % (n {x['braking_excluded']['n']}); "
                    f"above threshold either sign {x['above_thr_either_sign_share'] * 100:.2f} %; n fit {x['n_fit']}")
                log(f"     B {x['B']:.3f} (bound {x['B_bound']}), K {x['K']:.2f} / rad, mu_demo {x['mu_demo']:.3f}; gate: g1 "
                    f"span {g4.get('alpha_span_deg', float('nan')):.2f} deg {'FAIL' if 'g1 excitation' in failed else 'pass'}, "
                    f"K band [6, 30] {'FAIL' if 'g2 normalised stiffness' in failed else 'pass'}, mu ceiling 2.0 "
                    f"{'FAIL' if any('ceiling' in f for f in failed) else 'pass'}; peak note '{g4.get('peak_note', '')}'")
            d = rec["rear_drive_util"]
            log(f"  rear drive utilisation on drive samples (n {d['n']}): p50 {d['p50']:.3f}, p95 {d['p95']:.3f}, "
                f"p99 {d['p99']:.3f}; friction-circle factor at p50 {rec['friction_circle_factor']['p50']:.4f}, "
                f"at p95 {rec['friction_circle_factor']['p95']:.4f} (rear mu_demo {rec['axles']['rear']['mu_demo']:.3f})")
            if not fb:
                for ax in ("f", "r"):
                    r = rec[f"reset_affected_{ax}"]
                    log(f"  observer_reset_affected {ax}: {r['n_marked']} / {r['n_verdict']} = {r['share'] * 100:.2f} %; "
                        f"n_reset_excluded over phases {rec[f'n_reset_excluded_phases_{ax}']}")
            log(f"  hard-bound re-entries {rec['hard_bound_reentries']}; observer resets {rec['n_resets']}")
            with open(os.path.join(OUT_DIR, f"final_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    sys.exit(main())
