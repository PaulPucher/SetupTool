# Fz-integration Phase 2: free-D vs load-normalised (mu) Pacejka fit,
# both real sessions. Read-only, no config/production changes (config
# tyre_fit_auto.load_normalised_fit_enabled stays False throughout --
# this script calls fit_session_pacejka directly with an explicit
# load_normalised= argument, never flips the live config).
#
# Reports, per session per axle: B/C/D/E (or mu), fit_rms_resid_N,
# mu plausibility (config tyre_fit_auto.mu_plausibility_band_low/high),
# and the resulting EKF's NIS exceedance/health numbers -- the same
# comparison method's own thesis_notes.md entry ("Pacejka load-
# normalised (mu) tyre fit") calls for.

import json

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.tyre_fit_auto import fit_session_pacejka
from modules.nis_gate import evaluate_gate

SESSIONS = (
    ("dubai", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("v3", "GT3_PRC_MLA-v3.txt"),
)


def _print_axle(label, fit):
    if fit["fit_n_samples"] == 0:
        print(f"    {label}: EMPTY fit population -- degenerate")
        return
    mu_txt = f" mu={fit['mu']:.4f}" if "mu" in fit else ""
    print(f"    {label}: B={fit['B']:.3f} C={fit['C']:.3f} D={fit['D']:.1f} E={fit['E']:.3f}{mu_txt} "
          f"n={fit['fit_n_samples']} rms_resid={fit['fit_rms_resid_N']:.1f}N "
          f"powell_converged={fit['powell_converged']} sign_ok={fit['sign_ok']} "
          f"peak={fit['peak_alpha_deg']:.2f}deg in_range={fit['peak_in_visited_range']}")
    if "mean_axle_fz_N" in fit:
        print(f"      mean_axle_fz_N={fit['mean_axle_fz_N']:.1f}")


def run_one(session_label, raw_file, mode_label, load_normalised, results):
    print(f"\n--- {session_label} / {mode_label} (load_normalised={load_normalised}) ---")
    data = parse_csv(raw_file)
    params = load_parameters()
    manifest = fit_session_pacejka(data, params, data_file_path=raw_file, load_normalised=load_normalised)

    if manifest.get("status") == "degenerate":
        print(f"    DEGENERATE: {manifest.get('degenerate_reason')}")
        results[(session_label, mode_label)] = manifest
        return

    _print_axle("front", manifest["axles"]["front"])
    _print_axle("rear", manifest["axles"]["rear"])
    print(f"    status={manifest['status']}")
    print(f"    nis: yaw_exceedance={manifest['nis']['yaw_rate_exceedance']:.4f} "
          f"ay_exceedance={manifest['nis']['ay_exceedance']:.4f} "
          f"combined_exceedance={manifest['nis']['combined_exceedance']:.4f} "
          f"combined_mean_nis={manifest['nis']['combined_mean_nis']:.4f}")
    print(f"    sign_check: {manifest['sign_check']}")

    state = prepare_vehicle_state(data["channels"], params)
    gate_verdict = evaluate_gate(manifest["nis_full"], manifest["base_mask"], params, state["sample_rate_hz"])
    print(f"    nis_gate: verdict={gate_verdict['verdict']} health_score={gate_verdict['health_score']:.4f}")

    if load_normalised:
        mp = manifest["mu_plausibility"]
        print(f"    mu_plausibility: front={mp['mu_front']:.4f} (plausible={mp['front_plausible']}), "
              f"rear={mp['mu_rear']:.4f} (plausible={mp['rear_plausible']}), "
              f"band=[{mp['band_low']}, {mp['band_high']}]")
        if not mp["front_plausible"] or not mp["rear_plausible"]:
            print("    *** mu OUTSIDE plausibility band -- STOP condition per the work order ***")

    results[(session_label, mode_label)] = {
        "status": manifest["status"],
        "axles": manifest["axles"],
        "nis": manifest["nis"],
        "mu_plausibility": manifest.get("mu_plausibility"),
        "gate_verdict": {"verdict": gate_verdict["verdict"], "health_score": gate_verdict["health_score"]},
    }


def main():
    results = {}
    for session_label, raw_file in SESSIONS:
        run_one(session_label, raw_file, "free-D", False, results)
        run_one(session_label, raw_file, "mu", True, results)

    out_path = "diagnostics/fz_mu_tyre_fit_results.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({f"{k[0]}/{k[1]}": v for k, v in results.items()}, f, indent=2, default=str)
    print(f"\nsaved {out_path}")


if __name__ == "__main__":
    main()
