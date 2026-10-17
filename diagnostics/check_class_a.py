# Old vs new validation of the Class A estimator corrections (R2 rebuild,
# author work order 2026-10-16). Numbers only; the verdict is the author's.
#
# OLD = the modules at BASE_REV (the tree before R2), loaded from git into
# this process; NEW = the working tree. Same params (best available, as the
# app's default path) and the same three exports.
#   chain       : full production beta chain (resolve_sideslip_beta), old vs new
#   #26 isolated: CS with the NEW beta, old vs new estimate_cornering_stiffness
#   #32 isolated: stability with the NEW beta, old vs new estimate_yaw_moment_stability
#   #12         : apex_3 instances whose apex instant lands exactly on a grid
#                 sample (one-sample phase slice) and their apex stability
# Worst-lap CS per corner = the classifier's read-out: per phase min across
# analysis laps (aggregate_by_corner, worst_lap), apex_3 from apex_region,
# then min over phases, capped at 1.0 like _classify_corner.
# Run: python -m diagnostics.check_class_a

import os
import subprocess
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402

BASE_REV = "279f4ee"
SESSIONS = (
    ("Dubai sample export", "Sample/Sample_Dubai.txt"),
    ("NBR-73-010-026-JDR", "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
)
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


def old_module(path, name):
    src = subprocess.run(["git", "show", f"{BASE_REV}:{path}"], capture_output=True, text=True,
                         check=True, encoding="utf-8").stdout
    mod = types.ModuleType(name)
    mod.__file__ = f"{BASE_REV}:{path}"
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def load_old():
    sa = old_module("modules/stability_analysis.py", "old_stability_analysis")
    ekf = old_module("modules/sideslip_ekf_pacejka.py", "old_sideslip_ekf_pacejka")
    gate = old_module("modules/nis_gate.py", "old_nis_gate")
    tfa = old_module("modules/tyre_fit_auto.py", "old_tyre_fit_auto")
    tfa.estimate_sideslip_ekf_pacejka = ekf.estimate_sideslip_ekf_pacejka
    tfa.evaluate_gate = gate.evaluate_gate
    return sa, tfa


def run_chain(tfa, state, params, data, path):
    """resolve_sideslip_beta with the module's EKF calls logged: final-run
    result, and per sweep call the hard-bound samples (the only samples where
    raw beta and beta_with_fallback differ)."""
    orig = tfa.estimate_sideslip_ekf_pacejka
    log = {"sweep_hb": []}

    def logged(st, prm, pass_id):
        res = orig(st, prm, pass_id)
        bound = np.radians(prm["tyre_model_ekf_pacejka"][pass_id]["beta_hard_bound_deg"])
        hb = res["diverged_mask"] & (np.abs(res["beta"]) > bound)
        if pass_id == "_auto_final":
            log["final"] = res
            log["final_hb"] = hb
        elif pass_id == "_auto_sweep":
            log["sweep_hb"].append(hb)
        return res

    tfa.estimate_sideslip_ekf_pacejka = logged
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka", csv_path=path)
    finally:
        tfa.estimate_sideslip_ekf_pacejka = orig
    return {"beta": beta, "man": man, "gate": gate, "fallback": fb, "fallback_reason": fr, "log": log}


def straights_beta(parsed, st, beta):
    from diagnostics.inspect_kinematic_inputs import steering_straights
    t = st["time"]
    laps = [l for l in parsed["laps"] if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in laps:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    segs = sorted(steering_straights(st, laps, lap_id, kerb)["segs"])
    mids = np.array([(a + b) // 2 for a, b in segs[1::2]])
    x = np.abs(np.degrees(beta[mids]))
    return float(np.median(x)), float(x.max()), int(mids.size)


def summaries(parsed, cs, stab, st):
    from modules.stability_analysis import summarise_corners
    from modules.recommendation import aggregate_by_corner
    valid = {l["lap_number"] for l in parsed["laps"] if l.get("is_valid_for_analysis")}
    summ = summarise_corners(parsed.get("corners", []), cs, stab, st, lap_filter=None)
    return summ, aggregate_by_corner([s for s in summ if s["lap_number"] in valid])


def worst_cs(agg, axle):
    out = {}
    for cid, a in agg.items():
        if cid is None:
            continue
        vals = []
        for ph in PHASES:
            src = a["apex_region"] if ph == "apex_3" else a["phases"][ph]
            v = src[f"cs_ratio_{axle}"]["median"]
            if v == v:
                vals.append(v)
        if vals:
            out[cid] = min(1.0, min(vals))
    return out


def worst_stab(agg):
    out = {}
    for cid, a in agg.items():
        vals = [a["phases"][ph]["stability_observed_Nm_per_deg"]["median"] for ph in PHASES]
        vals = [v for v in vals if v == v]
        if cid is not None and vals:
            out[cid] = min(vals)
    return out


def delta_line(old, new, unit=""):
    common = sorted(set(old) & set(new))
    if not common:
        return "no corner finite in both"
    d = np.array([abs(new[c] - old[c]) for c in common])
    k = common[int(np.argmax(d))]
    return (f"median {np.median(d):.3f}{unit}, max {d.max():.3f}{unit} (C{k} {old[k]:+.3f} -> {new[k]:+.3f}); "
            f"{len(common)} corners, {int((d > 1e-9).sum())} changed")


def apex_on_grid(parsed, st, summ):
    t = st["time"]
    by_key = {(s["lap_number"], s["corner_number"]): s for s in summ}
    valid = {l["lap_number"] for l in parsed["laps"] if l.get("is_valid_for_analysis")}
    n_all = n_valid = hit = hit_nan = 0
    for c in parsed.get("corners", []):
        n_all += 1
        if c["lap_number"] not in valid:
            continue
        n_valid += 1
        s0, s1 = c["segments"]["apex_3"]
        if s1 != s0:
            continue
        lo = int(np.searchsorted(t, s0, side="left"))
        if lo < t.size and t[lo] == s0:
            hit += 1
            s = by_key.get((c["lap_number"], c["corner_number"]))
            v = s["phases"]["apex_3"]["stability_observed_Nm_per_deg"]["median"] if s else float("nan")
            hit_nan += int(v != v)
    return n_all, n_valid, hit, hit_nan


def zero_steps(alpha, moving):
    d = np.diff(alpha)
    return int(((d == 0.0) & moving[1:]).sum())


def main():
    from diagnostics.inspect_tyre_prior_d2 import _params
    from modules.csv_parser import parse_csv
    import modules.tyre_fit_auto as tfa_new
    from modules.stability_analysis import (prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability)
    sa_old, tfa_old = load_old()
    params = _params()
    assert params["stability_estimation"]["sideslip_source"] == "ekf_auto_pacejka"
    print(f"Class A check: OLD = {BASE_REV}, NEW = working tree; params best available (cap None)")
    for label, rel in SESSIONS:
        path = resolve_data_path(rel)
        parsed = parse_csv(path)
        st = prepare_vehicle_state(parsed["channels"], params)
        print("\n" + "=" * 100)
        print(f"{label}: {path} (grid {st['sample_rate_hz']:.0f} Hz)")
        runs = {"old": run_chain(tfa_old, st, params, parsed, path),
                "new": run_chain(tfa_new, st, params, parsed, path)}
        for k in ("old", "new"):
            r = runs[k]
            man, gate, lg = r["man"], r["gate"] or {}, r["log"]
            ch = man["r_sweep"]["chosen"]
            div = lg["final"]["diverged_mask"]
            sw_hb = [int(h.sum()) for h in lg["sweep_hb"]]
            med, worst, n_mid = straights_beta(parsed, st, r["beta"])
            print(f"  - {k}: status '{man['status']}'"
                  + (f" ({man.get('degenerate_reason') or man.get('marginal_reason')})"
                     if man["status"] != "ok" else "")
                  + f"; fallback {r['fallback']}"
                  + (f" ({r['fallback_reason']})" if r["fallback"] else ""))
            print(f"      sign check {man['sign_check']['median_gate_racing']}; R pick ay x{ch['r_ay_scale']:g} / yaw "
                  f"x{ch['r_yaw_scale']:g} ({man['r_sweep'].get('selection')}, f_ay {ch['f_ay']:.3f}, f_yaw "
                  f"{ch['f_yaw']:.3f})")
            print(f"      NIS gate '{gate.get('verdict')}' score {gate.get('health_score', float('nan')):.4f}, window "
                  f"{gate.get('window_samples')} samples, skipped low coverage "
                  f"{gate.get('n_windows_skipped_low_coverage', 'n/a')}")
            print(f"      final-run divergences {int(div.sum())} (hard-bound {int(lg['final_hb'].sum())}); sweep "
                  f"hard-bound samples per call max {max(sw_hb) if sw_hb else 0}")
            print(f"      straights |beta| median {med:.2f} / worst {worst:.2f} deg ({n_mid} validation straights)")
            if k == "new":
                cov = man["onset_coverage"]
                print(f"      coverage front {cov['front']['coverage_fraction']:.4f} / rear "
                      f"{cov['rear']['coverage_fraction']:.4f}; h2 corr {man['h2_vs_ay_apex']['correlation']:.4f}")
                print("      guard trace: " + ", ".join(
                    f"{g['r_ay_scale']:g}/{g['r_yaw_scale']:g} sign {g['sign_fraction']:.3f} "
                    f"{'pass' if g['guard_pass'] else 'FAIL'}" for g in man["r_sweep"]["guard_trace"]))

        beta = runs["new"]["beta"]
        slip = estimate_slip_angles(st, beta, params)
        forces = estimate_lateral_forces(st, params)
        print(f"  - fix 1 residual: exact zero alpha steps on moving samples front "
              f"{zero_steps(slip['alpha_f_filt'], st['moving_mask'])}, rear "
              f"{zero_steps(slip['alpha_r_filt'], st['moving_mask'])} (zero-step rule only matters here)")
        cs_old = sa_old.estimate_cornering_stiffness(slip, forces, st, params)
        cs_new = estimate_cornering_stiffness(slip, forces, st, params)
        stab_old = sa_old.estimate_yaw_moment_stability(st, beta, params, parsed.get("laps", []))
        stab_new = estimate_yaw_moment_stability(st, beta, params, parsed.get("laps", []))
        _, agg_cs_old = summaries(parsed, cs_old, stab_new, st)
        summ_new, agg_new = summaries(parsed, cs_new, stab_new, st)
        _, agg_stab_old = summaries(parsed, cs_new, stab_old, st)
        for axle, name in (("f", "front"), ("r", "rear")):
            print(f"  - #26 isolated, worst-lap CS {name}: "
                  + delta_line(worst_cs(agg_cs_old, axle), worst_cs(agg_new, axle)))
        print("  - #32 isolated, worst-phase stability [Nm/deg]: "
              + delta_line(worst_stab(agg_stab_old), worst_stab(agg_new)))
        so, sn = stab_old["stability_observed_Nm_per_deg"], stab_new["stability_observed_Nm_per_deg"]
        print(f"      finite stability samples old {int(np.isfinite(so).sum())} -> new {int(np.isfinite(sn).sum())}")
        n_all, n_valid, hit, hit_nan = apex_on_grid(parsed, st, summ_new)
        print(f"  - #12 apex-on-grid: {hit} of {n_valid} analysis-lap instances ({n_all} total), apex stability "
              f"NaN on {hit_nan} of them")


if __name__ == "__main__":
    main()
