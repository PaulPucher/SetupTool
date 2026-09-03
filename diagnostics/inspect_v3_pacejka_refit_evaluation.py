# PLAN.md work order, Phase 2 (2026-09-03): refit evaluation, READ-ONLY.
# Does NOT ship anything -- report only, per the work order's own
# pre-registration. No config/production file touched.
#
# Continues the historical Dugoff pass-2/3/4 refit loop (thesis_notes.md,
# "refit loop... STOPPED as non-converging on pre-registered criteria",
# PLAN.md STATUS) for the CURRENT production tyre model (Pacejka,
# ekf_auto_pacejka), reusing modules.tyre_fit_auto's exact machinery
# (fit_session_pacejka, _fit_axle_pacejka, _base_mask) and diagnostics/
# sideslip_ekf_pacejka.py's EKF recursion directly, rather than
# reimplementing either -- same dependency reasoning as modules/
# tyre_fit_auto.py's own header note.
#
# Method, per iteration: pass 1 is production's own ekf_auto_pacejka
# first-shot fit (curve fitted from KINEMATIC alpha, then run through the
# EKF to get beta_ekf -- fit_session_pacejka's documented procedure).
# Iteration N (N=1..MAX_ITERATIONS) re-fits B/C/D/E from iteration N-1's
# OWN beta-derived alpha (pass 1's for N=1) -- breaking the kinematic
# dependency one more step each time, exactly the refit loop's original
# purpose -- and re-runs the EKF with the refit curve, holding R (process/
# measurement noise) fixed at pass 1's own chosen values throughout (only
# the tyre curve is refit at each step, matching the original work
# order's scope; a changing R at every step would confound curve quality
# with re-tuned filter noise).
#
# Extension (2026-09-03, same day, user instruction): run this same
# chain to 4 iterations (matching the historical pass-2/3/4 depth) on
# BOTH v3 and Dubai -- Dubai as a CONFIRMATION run, since Dubai is the
# session the historical Dugoff loop originally degenerated on. Report
# numbers only; no viability judgement is made here (see thesis_notes.md
# for the single-iteration v3 finding this extends).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
)
from modules.tyre_fit_auto import fit_session_pacejka, _fit_axle_pacejka, _base_mask, CHI2_DF1_95, CHI2_DF2_95
from diagnostics.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka
from modules.nis_gate import evaluate_gate

MAX_ITERATIONS = 4

FILES = [
    ("DUBAI (confirmation run)", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("V3", "GT3_PRC_MLA-v3.txt"),
]


def _axle_line(fit):
    return (f"B={fit['B']:.4f} C={fit['C']:.4f} D={fit['D']:.1f} E={fit['E']:.4f}  "
            f"powell_converged={fit['powell_converged']} sign_ok(D>0)={fit['sign_ok']}  "
            f"peak_alpha_deg={fit['peak_alpha_deg']:.3f} (visited p99={fit['visited_alpha_p99_deg']:.3f} deg, "
            f"{'INSIDE' if fit['peak_in_visited_range'] else 'OUTSIDE -- extrapolated'})  "
            f"n={fit['fit_n_samples']} rms_resid={fit['fit_rms_resid_N']:.1f} N")


def run_refit_chain(label, raw_file, max_iterations=MAX_ITERATIONS):
    print("=" * 100)
    print(f"{label}: {raw_file}")
    print("=" * 100)
    data = parse_csv(raw_file)
    params = load_parameters()

    pass1 = fit_session_pacejka(data, params, data_file_path=raw_file)
    print(f"PASS 1 (production first-shot, kinematic-alpha-seeded) status: {pass1['status']}")
    if pass1["status"] == "degenerate":
        print(f"degenerate_reason: {pass1.get('degenerate_reason')}")
        print("Cannot proceed to any refit iteration -- pass 1 itself produced no usable curve.")
        return None

    state = prepare_vehicle_state(data["channels"], params)
    laps = data.get("laps", [])
    base_mask = _base_mask(state, laps)
    sample_rate_hz = state["sample_rate_hz"]

    gate1 = evaluate_gate(pass1["nis_full"], pass1["base_mask"], params, sample_rate_hz)
    fits = {1: {"front": pass1["axles"]["front"], "rear": pass1["axles"]["rear"]}}
    nis = {1: dict(pass1["nis"])}
    gates = {1: gate1}
    print(f"  front: {_axle_line(fits[1]['front'])}")
    print(f"  rear:  {_axle_line(fits[1]['rear'])}")
    print(f"  NIS: yaw_exceedance={nis[1]['yaw_rate_exceedance']:.4f} ay_exceedance={nis[1]['ay_exceedance']:.4f} "
          f"combined_exceedance={nis[1]['combined_exceedance']:.4f}")
    print(f"  NIS-gate health_score={gate1['health_score']:.4f} verdict={gate1['verdict']!r} "
          f"(threshold_use_ekf={gate1['threshold_use_ekf']}, threshold_warn={gate1['threshold_warn']})")

    beta_prev = pass1["beta_ekf_with_fallback"]
    final_cfg_pass1 = pass1["final_config"]
    last_completed = 1

    for it in range(2, max_iterations + 1):
        print()
        print("-" * 100)
        print(f"ITERATION {it} -- refit B/C/D/E from iteration {it - 1}'s own beta-derived alpha")
        print("-" * 100)
        slip_prev = estimate_slip_angles(state, beta_prev, params)
        forces = estimate_lateral_forces(state, params)
        front_it = _fit_axle_pacejka(slip_prev["alpha_f_filt"], forces["Fy_f_filt"], base_mask)
        rear_it = _fit_axle_pacejka(slip_prev["alpha_r_filt"], forces["Fy_r_filt"], base_mask)

        if not (front_it["sign_ok"] and front_it["powell_converged"] and
                rear_it["sign_ok"] and rear_it["powell_converged"]):
            print(f"  front: {_axle_line(front_it)}")
            print(f"  rear:  {_axle_line(rear_it)}")
            print(f"  ITERATION {it} DEGENERATE -- Powell did not converge or D<=0 on at least one axle. "
                  f"This REPRODUCES the historical refit loop's failure signature. STOPPING this file's chain here.")
            break

        cfg_it = dict(final_cfg_pass1)
        cfg_it.update({
            "b_front": front_it["B"], "c_front": front_it["C"], "d_front": front_it["D"], "e_front": front_it["E"],
            "b_rear": rear_it["B"], "c_rear": rear_it["C"], "d_rear": rear_it["D"], "e_rear": rear_it["E"],
        })
        params_it = dict(params)
        pass_id = f"_auto_refit_iter{it}"
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
    print(f"{'iter':>4}  {'B_f':>8} {'C_f':>7} {'D_f':>9} {'E_f':>9}  {'B_r':>8} {'C_r':>7} {'D_r':>9} {'E_r':>9}  "
          f"{'rear_peak_deg':>13} {'rear_in_range':>13}  {'NIS_comb':>9} {'health':>8} {'verdict':>7}")
    for i in sorted(fits):
        f, r = fits[i]["front"], fits[i]["rear"]
        g = gates[i]
        print(f"{i:>4}  {f['B']:>8.4f} {f['C']:>7.4f} {f['D']:>9.1f} {f['E']:>9.4f}  "
              f"{r['B']:>8.4f} {r['C']:>7.4f} {r['D']:>9.1f} {r['E']:>9.4f}  "
              f"{r['peak_alpha_deg']:>13.3f} {str(r['peak_in_visited_range']):>13}  "
              f"{nis[i]['combined_exceedance']:>9.4f} {g['health_score']:>8.4f} {g['verdict']:>7}")
    if last_completed < max_iterations:
        print(f"Chain stopped early at iteration {last_completed + 1} (degenerate) -- "
              f"iterations {last_completed + 1}..{max_iterations} not reached.")
    return {"fits": fits, "nis": nis, "gates": gates, "last_completed": last_completed}


def main():
    results = {}
    for label, raw_file in FILES:
        results[label] = run_refit_chain(label, raw_file)
        print()
    return results


if __name__ == "__main__":
    main()
