# Q-NIS-RESET (author order and EXECUTE 2026-10-19): does a NIS-window
# covariance reset (state kept, P to P0) change the windowed CS slope? Decides
# the scope of the observer-reset mask. Read-only.
#
# Per session (NBR-73-010-026-JDR, NBR JST 29, MLA-SE07_US-v2), L mode as in the
# final run: run A = shipped EKF L (resolve_sideslip_beta); run B = the same
# final observer call (same state, same fit, same R) with
# nis_window_reset=False, hard-bound reset active. Per axle and per verdict CS
# window [start, i) (same population as inspect_fit_lit_final) that run A's
# reset mask marks and that holds no run-A hard-bound reset ("NIS-only"): the
# windowed OLS slope with alpha from A and from B, same Fy, same window
# boundaries (reconstruct_cs_window_start on run A's alpha, checked against
# C_window), metric |CS_A - CS_B| / |CS_A|. Windows where run B hits a hard
# bound are counted as affected and kept out of the metric.
# Declared criterion (DECISIONS.md, before the run): p95 <= 0.10 on every axle
# of every session -> mask scope hard-bound only; otherwise the mask stays.
# [2026-10-19 Q-NIS-CONTROL (author EXECUTE): the same metric on the control
#   population U, verdict windows holding no run-A reset of either kind, beside
#   M (the NIS-only windows above); also in-window mean |beta_A - beta_B| and the
#   alpha_A window span, and for U the share of windows with span below 0.021 rad.
#   Declared rule: U p95 <= 0.10 on every axle -> sensitivity specific to reset
#   windows; otherwise sensitive to observer policy in general.]
# [2026-10-19 --populations (author addendum to Q-NIS-CONTROL): run A only;
#   per session, axle, stable corner and phase, the verdict CS samples on
#   analysis laps summed over laps from summarise_corners: unfiltered (n + kerb +
#   reset excluded), after kerb exclusion (n + reset), after kerb + reset (n),
#   and the lap instances below cs_phase_min_valid_samples at each step.
#   Output populations.log, populations_<key>.json. Report only.]
# Output diagnostics/plots/fit_lit/nis_reset/ (Q-NIS-RESET: run.log,
# nis_reset_<key>.json; Q-NIS-CONTROL: run_control.log, nis_control_<key>.json).
# Run: python -m diagnostics.inspect_nis_reset_effect

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, ORIG, load_session, capture, keep_awake, _jsonable  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402

OUT_DIR = "diagnostics/plots/fit_lit/nis_reset"
ORDER = ("nbr_jdr", "nbr_jst29", "pr_us_v2")
TOL = 0.10  # declared Tier-B tolerance on p95 (DECISIONS.md, 2026-10-19)
# chair estimator linear threshold (internal reference), context share for U only
LINEAR_REF_RAD = 0.021

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    _LOG.write(msg + "\n")
    _LOG.flush()


def ols(a, f):
    am = a - a.mean()
    return float(np.sum(am * (f - f.mean())) / np.sum(am ** 2))


def main():
    global _LOG
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, reconstruct_cs_window_start,
                                            resolve_cs_min_window_samples)
    from modules.nis_gate import evaluate_gate
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run_control.log"), "w", encoding="utf-8")
    keep_awake(True)
    sessions = {k: (lab, rel) for k, lab, rel in SESSIONS}
    verdicts = {"U": [], "M": []}
    try:
        for key in ORDER:
            label, rel = sessions[key]
            S = load_session(key, label, rel)
            st, p = S["state"], S["params"]
            holder = {}
            with capture(holder, True):
                inner = tfa.estimate_sideslip_ekf_pacejka

                def keep_args(state, params, pass_id=None, **kw):
                    if pass_id == "_auto_final":
                        holder["final_args"] = (state, params)
                    return inner(state, params, pass_id=pass_id, **kw)

                tfa.estimate_sideslip_ekf_pacejka = keep_args
                beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, S["data"], "ekf_auto_pacejka",
                                                                     csv_path=S["path"])
            log("=" * 100)
            if fb:
                log(f"{label}: run A fell back to the kinematic beta ({fr}) -- nothing to compare")
                verdicts["U"].append(False)
                verdicts["M"].append(False)
                continue
            raw, fin = holder["raw"], holder["final"]
            assert np.array_equal(beta, fin["beta_with_fallback"])
            ekf_state, ekf_params = holder["final_args"]
            res_b = ORIG["ekf"](ekf_state, ekf_params, pass_id="_auto_final", nis_window_reset=False)
            gate_b = evaluate_gate(res_b["nis"], raw["base_mask"], p, st["sample_rate_hz"])

            bound = np.radians(p["tyre_fit_auto"]["beta_hard_bound_deg"])
            reset_a = raw["observer_reset_mask"]
            hb_a = fin["diverged_mask"] & (np.abs(fin["beta"]) > bound)
            hb_b = res_b["diverged_mask"]
            cum = {k: np.concatenate(([0], np.cumsum(m, dtype=np.int64)))
                   for k, m in (("reset_a", reset_a), ("hb_a", hb_a), ("hb_b", hb_b))}

            forces = estimate_lateral_forces(st, p)
            slip_a = estimate_slip_angles(st, beta, p)
            beta_b = res_b["beta_with_fallback"]
            slip_b = estimate_slip_angles(st, beta_b, p)
            cs = estimate_cornering_stiffness(slip_a, forces, st, p, S["data"]["laps"], reset_mask=reset_a)
            se = p["stability_estimation"]
            min_window = resolve_cs_min_window_samples(p, st["sample_rate_hz"])
            min_span, max_m = se["cs_min_slip_angle_span_rad"], se["cs_max_window_m"]
            kerb = st.get("kerb_mask")

            rec = {"label": label, "gate_a": {k: gate[k] for k in ("verdict", "health_score")},
                   "gate_b": {k: gate_b[k] for k in ("verdict", "health_score")},
                   "hard_bound_reentries_a": int(((np.abs(fin["beta"]) > bound) & S["base"]).sum()),
                   "hard_bound_reentries_b": int(((np.abs(res_b["beta"]) > bound) & S["base"]).sum()),
                   "resets_a": int(reset_a.sum()), "resets_b": int(hb_b.sum()), "axles": {}}
            log(f"{label}: run A gate {rec['gate_a']}, resets {rec['resets_a']}, hard-bound re-entries "
                f"{rec['hard_bound_reentries_a']}; run B (NIS-window reset off) gate {rec['gate_b']}, "
                f"resets {rec['resets_b']}, hard-bound re-entries {rec['hard_bound_reentries_b']}")
            for ax in ("f", "r"):
                alpha_a, alpha_b = slip_a[f"alpha_{ax}_filt"], slip_b[f"alpha_{ax}_filt"]
                fy = forces[f"Fy_{ax}_filt"]
                pop = S["in_br"] & st["moving_mask"] & ~cs[f"near_strike_{ax}"] & np.isfinite(cs[f"CS_ratio_{ax}"])
                if kerb is not None:
                    pop &= ~kerb
                mark = cs[f"observer_reset_affected_{ax}"]
                name = "front" if ax == "f" else "rear"
                rec["axles"][ax] = {}
                for popname, idx in (("U", np.where(pop & ~mark)[0]), ("M", np.where(pop & mark)[0])):
                    rel, dbeta, span, n_hb_a, n_hb_b, max_repro = [], [], [], 0, 0, 0.0
                    for i in idx:
                        start = reconstruct_cs_window_start(alpha_a, i, min_window, min_span, st.get("s_m"), max_m)
                        if cum["hb_a"][i] > cum["hb_a"][start]:
                            n_hb_a += 1
                            continue
                        c_a = ols(alpha_a[start:i], fy[start:i])
                        max_repro = max(max_repro, abs(c_a - cs[f"C_window_{ax}"][i]) / abs(cs[f"C_window_{ax}"][i]))
                        if cum["hb_b"][i] > cum["hb_b"][start]:
                            n_hb_b += 1
                            continue
                        rel.append(abs(c_a - ols(alpha_b[start:i], fy[start:i])) / abs(c_a))
                        dbeta.append(np.degrees(np.mean(np.abs(beta[start:i] - beta_b[start:i]))))
                        span.append(alpha_a[start:i].max() - alpha_a[start:i].min())
                    rel, dbeta, span = np.asarray(rel), np.asarray(dbeta), np.asarray(span)
                    pct = (lambda x, v: float(np.percentile(x, v)) if x.size else np.nan)
                    q = {"rel_p50": pct(rel, 50), "rel_p95": pct(rel, 95), "rel_p99": pct(rel, 99),
                         "dbeta_deg_p50": pct(dbeta, 50), "dbeta_deg_p95": pct(dbeta, 95),
                         "span_deg_p50": pct(np.degrees(span), 50), "span_deg_p5": pct(np.degrees(span), 5),
                         "share_span_below_linear": float((span < LINEAR_REF_RAD).mean()) if span.size else np.nan}
                    ok = bool(rel.size and q["rel_p95"] <= TOL)
                    verdicts[popname].append(ok)
                    rec["axles"][ax][popname] = dict(q, n_windows=int(len(idx)), n_hard_bound_a=n_hb_a,
                                                     n_b_hard_bound=n_hb_b, n_metric=int(rel.size),
                                                     window_repro_max_rel=max_repro, p95_within_tol=ok)
                    log(f"  {name} {popname}: windows {len(idx)} (run-A hard-bound {n_hb_a}, run B hard-bound "
                        f"{n_hb_b}, metric on {rel.size}); rel diff p50 {q['rel_p50']:.4f} p95 {q['rel_p95']:.4f} "
                        f"p99 {q['rel_p99']:.4f}; mean |dbeta| deg p50 {q['dbeta_deg_p50']:.4f} p95 "
                        f"{q['dbeta_deg_p95']:.4f}; alpha span deg p50 {q['span_deg_p50']:.3f} p5 "
                        f"{q['span_deg_p5']:.3f}; share span < 0.021 rad {q['share_span_below_linear']:.4f}; "
                        f"p95 {'<=' if ok else '>'} {TOL}; window repro max rel dev {max_repro:.2e}")
            with open(os.path.join(OUT_DIR, f"nis_control_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1, default=str)
        log("=" * 100)
        log("Q-NIS-RESET criterion (M): " + ("p95 <= 0.10 everywhere -> mask scope hard-bound only"
                                              if all(verdicts["M"]) else "p95 > 0.10 somewhere -> mask stays as ruled"))
        log("Q-NIS-CONTROL rule (U): " + ("(a) p95 <= 0.10 on every axle -> sensitivity specific to reset windows"
                                           if all(verdicts["U"]) else "(b) p95 > 0.10 somewhere -> CS slope sensitive "
                                           "to observer policy in general; report and wait"))
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


def populations():
    global _LOG
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners)
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "populations.log"), "w", encoding="utf-8")
    keep_awake(True)
    sessions = {k: (lab, rel) for k, lab, rel in SESSIONS}
    try:
        for key in ORDER:
            label, rel = sessions[key]
            S = load_session(key, label, rel)
            st, p, laps = S["state"], S["params"], S["data"]["laps"]
            holder = {}
            with capture(holder, True):
                beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, S["data"], "ekf_auto_pacejka",
                                                                     csv_path=S["path"])
            log("=" * 100)
            if fb:
                log(f"{label}: kinematic fallback ({fr}) -- no reset mask")
                continue
            cs = estimate_cornering_stiffness(estimate_slip_angles(st, beta, p), estimate_lateral_forces(st, p),
                                              st, p, laps, reset_mask=holder["raw"]["observer_reset_mask"])
            summ = summarise_corners(S["data"]["corners"], cs, estimate_yaw_moment_stability(st, beta, p, laps), st)
            floor = p["stability_estimation"]["cs_phase_min_valid_samples"]
            ok_laps = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
            rec = {"label": label, "floor": floor, "axles": {}}
            log(f"{label}: verdict CS samples on analysis laps {sorted(ok_laps)}, summed over laps per corner and "
                f"phase; 'below floor' = lap instances with fewer than {floor} samples")
            for ax in ("f", "r"):
                cells = {}
                for s in summ:
                    if s["lap_number"] not in ok_laps or s.get("stable_corner_id") is None:
                        continue
                    for ph, d in s["phases"].items():
                        c = d[f"cs_ratio_{ax}"]
                        n_both, n_k, n_r = c["n"], c["n_kerb_excluded"], c.get("n_reset_excluded", 0)
                        x = cells.setdefault((s["stable_corner_id"], ph), {"raw": 0, "kerb": 0, "both": 0, "inst": 0,
                                                                             "below_raw": 0, "below_kerb": 0,
                                                                             "below_both": 0})
                        x["raw"] += n_both + n_k + n_r
                        x["kerb"] += n_both + n_r
                        x["both"] += n_both
                        x["inst"] += 1
                        x["below_raw"] += int(n_both + n_k + n_r < floor)
                        x["below_kerb"] += int(n_both + n_r < floor)
                        x["below_both"] += int(n_both < floor)
                name = "front" if ax == "f" else "rear"
                tot = {k: sum(x[k] for x in cells.values()) for k in ("raw", "kerb", "both", "inst", "below_raw",
                                                                     "below_kerb", "below_both")}
                log(f"  {name} TOTAL: unfiltered {tot['raw']}; after kerb {tot['kerb']} "
                    f"({tot['kerb'] / max(1, tot['raw']):.1%}); after kerb + reset {tot['both']} "
                    f"({tot['both'] / max(1, tot['raw']):.1%}); lap instances {tot['inst']}, below floor "
                    f"{tot['below_raw']} / {tot['below_kerb']} / {tot['below_both']}")
                for ph in ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"):
                    sub = [x for (cid, q), x in cells.items() if q == ph]
                    r_, k_, b_ = (sum(x[k] for x in sub) for k in ("raw", "kerb", "both"))
                    i_, bb = sum(x["inst"] for x in sub), sum(x["below_both"] for x in sub)
                    log(f"  {name} phase {ph}: unfiltered {r_}, after kerb {k_} ({k_ / max(1, r_):.1%}), after kerb + "
                        f"reset {b_} ({b_ / max(1, r_):.1%}); below floor after both {bb} of {i_} lap instances")
                for (cid, ph), x in sorted(cells.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
                    log(f"    {name} corner {cid} {ph}: unfiltered {x['raw']}, after kerb {x['kerb']} "
                        f"({x['kerb'] / max(1, x['raw']):.1%}), after kerb + reset {x['both']} "
                        f"({x['both'] / max(1, x['raw']):.1%}); lap instances {x['inst']}, below floor "
                        f"{x['below_raw']} / {x['below_kerb']} / {x['below_both']}")
                rec["axles"][ax] = {"total": tot, "cells": {f"{cid}|{ph}": x for (cid, ph), x in cells.items()}}
            with open(os.path.join(OUT_DIR, f"populations_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


if __name__ == "__main__":
    sys.exit(populations() if "--populations" in sys.argv else main())
