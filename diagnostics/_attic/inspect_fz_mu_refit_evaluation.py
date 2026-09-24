# Fz-integration Phase 3 (2026-09-03): bounded refit loop under mu,
# DIAGNOSTIC ONLY, no production change regardless of outcome. Mirrors
# diagnostics/inspect_v3_pacejka_refit_evaluation.py's exact iteration
# mechanics (same dependency reasoning: reuses modules.tyre_fit_auto and
# diagnostics/sideslip_ekf_pacejka.py directly, does not reimplement
# either) -- the ONLY change is substituting _fit_axle_pacejka_mu (D=
# mu*Fz, Fz measured, held fixed across iterations since Fz does not
# depend on beta) for _fit_axle_pacejka (free D) at each iteration's
# refit step. R (process/measurement noise) held fixed at pass 1's own
# chosen values throughout, same as the free-D loop.
#
# AMENDED CRITERIA (2026-09-03, user instruction, replacing the closed
# Dugoff/Pacejka loop's absolute-plausibility framing): growth band is
# PER-SESSION/PER-AXLE, mu within +/-15% of that axle's OWN iteration-1
# value -- a growth-detection band, not an absolute plausibility check
# (the absolute [1.2, 2.0] band already served its purpose at Phase 2's
# own gate and is not reapplied here). mu drift and peak-position
# settling are tracked and classified SEPARATELY -- the synthetic
# recovery test (tests/test_tyre_fit_auto_mu.py) already found mu is the
# reliably-identified parameter of the joint (B, C, mu, E) fit; B/C/E
# wander is EXPECTED and is not, on its own, a failure criterion.
#
# CLASSIFICATION per axle (BOUNDED / CREEPING / WANDERING):
#   WANDERING  -- peak_in_visited_range does not settle (flips value
#                 across consecutive iterations at least once after the
#                 first) -- "limit is beta itself" (the original work
#                 order's own phrase): the curve's own peak location does
#                 not converge, independent of what mu is doing.
#   CREEPING   -- peak settles, but mu exits the +/-15%-of-iteration-1
#                 band at any iteration, OR the iteration-to-iteration
#                 |delta mu| is not shrinking (monotonic drift with no
#                 sign of a plateau).
#   BOUNDED    -- peak settles AND mu stays inside the +/-15% band AND
#                 |delta mu| is non-increasing across iterations
#                 (circularity reduced, not just not-yet-diverged).
# WANDERING takes precedence over CREEPING if both apply (peak location
# not settling is the more severe failure, matching the original work
# order's own framing).

import json

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, load_car_data, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_vertical_loads,
)
from modules.tyre_fit_auto import fit_session_pacejka, _fit_axle_pacejka_mu, _base_mask, CHI2_DF1_95, CHI2_DF2_95
from diagnostics.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
from modules.nis_gate import evaluate_gate

MAX_ITERATIONS = 4
MU_GROWTH_BAND_FRACTION = 0.15

FILES = [
    ("DUBAI (rear EXCLUDED from conclusions -- reconstructed-RR proxy, still run/reported)",
     "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("V3 (PRIMARY evidence axle: rear -- strongest load coupling, thesis_notes.md "
     "'v3 rear divergence dig')", "GT3_PRC_MLA-v3.txt"),
]


def _axle_line(fit):
    mu_txt = f" mu={fit['mu']:.4f}" if "mu" in fit else ""
    return (f"B={fit['B']:.4f} C={fit['C']:.4f} D={fit['D']:.1f} E={fit['E']:.4f}{mu_txt}  "
            f"powell_converged={fit['powell_converged']} sign_ok={fit['sign_ok']}  "
            f"peak_alpha_deg={fit['peak_alpha_deg']:.3f} (visited p99={fit['visited_alpha_p99_deg']:.3f} deg, "
            f"{'INSIDE' if fit['peak_in_visited_range'] else 'OUTSIDE -- extrapolated'})  "
            f"n={fit['fit_n_samples']} rms_resid={fit['fit_rms_resid_N']:.1f} N")


def _classify(mu_series, peak_in_range_series):
    """mu_series/peak_in_range_series: lists indexed by iteration (1-based,
    mu_series[0] is iteration 1). Returns (classification, reasons: list[str]).
    """
    reasons = []
    n = len(mu_series)
    if n < 2:
        return "INSUFFICIENT_ITERATIONS", ["fewer than 2 completed iterations -- cannot classify"]

    peak_flips = any(peak_in_range_series[i] != peak_in_range_series[i - 1] for i in range(1, n))
    if peak_flips:
        reasons.append(f"peak_in_visited_range does not settle: {peak_in_range_series}")
        return "WANDERING", reasons

    mu1 = mu_series[0]
    band_lo, band_hi = mu1 * (1.0 - MU_GROWTH_BAND_FRACTION), mu1 * (1.0 + MU_GROWTH_BAND_FRACTION)
    out_of_band = [i + 1 for i, m in enumerate(mu_series) if not (band_lo <= m <= band_hi)]
    if out_of_band:
        reasons.append(f"mu exits +/-15% band [{band_lo:.4f}, {band_hi:.4f}] (mu_1={mu1:.4f}) "
                        f"at iteration(s) {out_of_band}")

    deltas = [abs(mu_series[i] - mu_series[i - 1]) for i in range(1, n)]
    shrinking = all(deltas[i] <= deltas[i - 1] * 1.0001 for i in range(1, len(deltas))) if len(deltas) > 1 else True
    if not shrinking:
        reasons.append(f"|delta mu| not shrinking across iterations: {['%.5f' % d for d in deltas]}")

    if out_of_band or not shrinking:
        if not reasons:
            reasons.append("mu drift did not plateau")
        return "CREEPING", reasons

    reasons.append(f"mu stayed inside +/-15% band throughout and |delta mu| shrank: {['%.5f' % d for d in deltas]}")
    return "BOUNDED", reasons


def run_refit_chain_mu(label, raw_file, max_iterations=MAX_ITERATIONS):
    print("=" * 100)
    print(f"{label}: {raw_file}")
    print("=" * 100)
    data = parse_csv(raw_file)
    params = load_parameters()
    car_data = load_car_data()

    pass1 = fit_session_pacejka(data, params, data_file_path=raw_file, load_normalised=True)
    print(f"PASS 1 (mu-mode first-shot, kinematic-alpha-seeded) status: {pass1['status']}")
    if pass1["status"] == "degenerate":
        print(f"degenerate_reason: {pass1.get('degenerate_reason')}")
        print("Cannot proceed to any refit iteration -- pass 1 itself produced no usable curve.")
        return None

    state = prepare_vehicle_state(data["channels"], params)
    laps = data.get("laps", [])
    base_mask = _base_mask(state, laps)
    sample_rate_hz = state["sample_rate_hz"]

    # Fz (measured) does not depend on beta -- computed ONCE, reused at
    # every iteration (unlike alpha, which is recomputed from each
    # iteration's own beta below).
    forces0 = estimate_lateral_forces(state, params)
    params_measured = dict(params)
    params_measured["stability_estimation"] = dict(params["stability_estimation"])
    params_measured["stability_estimation"]["vertical_load_source"] = "measured"
    fz = estimate_vertical_loads(state, forces0, params_measured, channels=data["channels"], car_data=car_data)
    assert fz["vertical_load_source_used"] == "measured", f"{raw_file}: measured Fz not resolved for the refit loop"

    gate1 = evaluate_gate(pass1["nis_full"], pass1["base_mask"], params, sample_rate_hz)
    fits = {1: {"front": pass1["axles"]["front"], "rear": pass1["axles"]["rear"]}}
    nis = {1: dict(pass1["nis"])}
    gates = {1: gate1}
    print(f"  front: {_axle_line(fits[1]['front'])}")
    print(f"  rear:  {_axle_line(fits[1]['rear'])}")
    print(f"  NIS: yaw_exceedance={nis[1]['yaw_rate_exceedance']:.4f} ay_exceedance={nis[1]['ay_exceedance']:.4f} "
          f"combined_exceedance={nis[1]['combined_exceedance']:.4f}")
    print(f"  NIS-gate health_score={gate1['health_score']:.4f} verdict={gate1['verdict']!r}")

    beta_prev = pass1["beta_ekf_with_fallback"]
    final_cfg_pass1 = pass1["final_config"]
    last_completed = 1

    for it in range(2, max_iterations + 1):
        print()
        print("-" * 100)
        print(f"ITERATION {it} -- refit B/C/mu/E from iteration {it - 1}'s own beta-derived alpha "
              f"(Fz unchanged -- does not depend on beta)")
        print("-" * 100)
        slip_prev = estimate_slip_angles(state, beta_prev, params)
        forces = estimate_lateral_forces(state, params)
        front_it = _fit_axle_pacejka_mu(slip_prev["alpha_f_filt"], forces["Fy_f_filt"], fz["fz_f_N"], base_mask)
        rear_it = _fit_axle_pacejka_mu(slip_prev["alpha_r_filt"], forces["Fy_r_filt"], fz["fz_r_N"], base_mask)

        if not (front_it["sign_ok"] and front_it["powell_converged"] and
                rear_it["sign_ok"] and rear_it["powell_converged"]):
            print(f"  front: {_axle_line(front_it)}")
            print(f"  rear:  {_axle_line(rear_it)}")
            print(f"  ITERATION {it} DEGENERATE -- Powell did not converge or mu<=0 on at least one axle. "
                  f"STOPPING this file's chain here.")
            break

        cfg_it = dict(final_cfg_pass1)
        cfg_it.update({
            "b_front": front_it["B"], "c_front": front_it["C"], "d_front": front_it["D"], "e_front": front_it["E"],
            "b_rear": rear_it["B"], "c_rear": rear_it["C"], "d_rear": rear_it["D"], "e_rear": rear_it["E"],
        })
        params_it = dict(params)
        pass_id = f"_auto_refit_mu_iter{it}"
        params_it["tyre_model_ekf_pacejka"] = {pass_id: cfg_it}
        result_it = estimate_sideslip_ekf_pacejka(state, params_it, pass_id=pass_id)

        nis_full_it = result_it["nis"]
        gate_it = evaluate_gate(nis_full_it, base_mask, params, sample_rate_hz)
        innovation_it = result_it["innovation"][base_mask]
        S_diag_it = result_it["S_diag"][base_mask]
        f_yaw_it = float((innovation_it[:, 0] ** 2 / S_diag_it[:, 0] > CHI2_DF1_95).mean())
        f_ay_it = float((innovation_it[:, 1] ** 2 / S_diag_it[:, 1] > CHI2_DF1_95).mean())
        f_comb_it = float((nis_full_it[base_mask] > CHI2_DF2_95).mean())

        fits[it] = {"front": front_it, "rear": rear_it}
        nis[it] = {"yaw_rate_exceedance": f_yaw_it, "ay_exceedance": f_ay_it, "combined_exceedance": f_comb_it}
        gates[it] = gate_it

        print(f"  front: {_axle_line(front_it)}")
        print(f"  rear:  {_axle_line(rear_it)}")
        print(f"  NIS: yaw_exceedance={f_yaw_it:.4f} ay_exceedance={f_ay_it:.4f} combined_exceedance={f_comb_it:.4f}")
        print(f"  NIS-gate health_score={gate_it['health_score']:.4f} verdict={gate_it['verdict']!r}")

        beta_prev = result_it["beta_with_fallback"]
        last_completed = it

    print()
    print(f"SUMMARY -- {label}, iterations 1..{last_completed} of {max_iterations} requested")
    print(f"{'iter':>4}  {'mu_f':>8} {'mu_r':>8}  {'D_f':>9} {'D_r':>9}  "
          f"{'front_peak_in':>13} {'rear_peak_in':>12}  {'NIS_comb':>9} {'health':>8} {'verdict':>7}")
    for i in sorted(fits):
        f, r = fits[i]["front"], fits[i]["rear"]
        g = gates[i]
        print(f"{i:>4}  {f['mu']:>8.4f} {r['mu']:>8.4f}  {f['D']:>9.1f} {r['D']:>9.1f}  "
              f"{str(f['peak_in_visited_range']):>13} {str(r['peak_in_visited_range']):>12}  "
              f"{nis[i]['combined_exceedance']:>9.4f} {g['health_score']:>8.4f} {g['verdict']:>7}")
    if last_completed < max_iterations:
        print(f"Chain stopped early at iteration {last_completed + 1} (degenerate) -- "
              f"iterations {last_completed + 1}..{max_iterations} not reached.")

    classification = {}
    for axle in ("front", "rear"):
        mu_series = [fits[i][axle]["mu"] for i in sorted(fits)]
        peak_series = [fits[i][axle]["peak_in_visited_range"] for i in sorted(fits)]
        verdict, reasons = _classify(mu_series, peak_series)
        classification[axle] = {"verdict": verdict, "reasons": reasons, "mu_series": mu_series}
        print(f"  CLASSIFICATION [{axle}]: {verdict}")
        for r in reasons:
            print(f"    - {r}")

    return {"fits": fits, "nis": nis, "gates": gates, "last_completed": last_completed,
            "classification": classification}


def main():
    results = {}
    for label, raw_file in FILES:
        results[label] = run_refit_chain_mu(label, raw_file)
        print()

    out_json = "diagnostics/fz_mu_refit_evaluation_results.json"

    def _strip(d):
        out = {}
        for k, v in d.items():
            if k in ("fits",):
                out[k] = {it: {ax: {kk: vv for kk, vv in ax_fit.items()
                                     if kk not in ("residuals", "residual_mask")}
                                for ax, ax_fit in per_it.items()}
                          for it, per_it in v.items()}
            else:
                out[k] = v
        return out

    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({label: (_strip(r) if r is not None else None) for label, r in results.items()},
                   f, indent=2, default=str)
    print(f"saved {out_json}")
    return results


if __name__ == "__main__":
    main()
