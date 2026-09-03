# v3 work package: investigation into why the v3 Pacejka fit lands in the
# NIS gate's "warn" tier. Read-only -- no gate/threshold/config changes.
# Runs modules.tyre_fit_auto.fit_session_pacejka directly (same function
# resolve_sideslip_beta calls) against both v3 and Dubai for an apples-to-
# apples comparison, using the SAME production code, no reimplementation.
# Disposable per CLAUDE.md's diagnostics/ rule.

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.tyre_fit_auto import fit_session_pacejka, _base_mask
from modules.tyre_model_pacejka import pacejka_lateral_force
from modules.nis_gate import evaluate_gate

OUT_DIR = "diagnostics/plots_v3"


def audit_session(label, raw_file, params):
    print(f"\n{'=' * 70}\n{label}\n{'=' * 70}")
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    assert state is not None
    print(f"sample_rate_hz = {state['sample_rate_hz']:.2f}")

    manifest = fit_session_pacejka(data, params, data_file_path=raw_file)
    print(f"fit status: {manifest['status']}")
    if manifest["status"] == "degenerate":
        print(f"degenerate_reason: {manifest['degenerate_reason']}")
        return None

    for axle in ("front", "rear"):
        f = manifest["axles"][axle]
        print(f"\n[{axle}] B={f['B']:.4f} C={f['C']:.4f} D={f['D']:.1f} E={f['E']:.4f}")
        print(f"  powell_converged={f['powell_converged']} sign_ok={f['sign_ok']} "
              f"fit_n_samples={f['fit_n_samples']} fit_rms_resid_N={f['fit_rms_resid_N']:.1f}")
        print(f"  peak_alpha_deg={f['peak_alpha_deg']:.2f} "
              f"peak_in_visited_range={f['peak_in_visited_range']} "
              f"visited_alpha_p99_deg={f['visited_alpha_p99_deg']:.2f}")

    print(f"\nsign_check: {manifest['sign_check']}")
    print(f"onset_coverage: {manifest.get('onset_coverage')}")
    print(f"h2_vs_ay_apex: {manifest.get('h2_vs_ay_apex')}")
    print(f"r_derivation: {manifest['r_derivation']}")

    # --- NIS gate, as actually evaluated in production -----------------
    # window_samples is now rate-derived (morning follow-up, NIS gate
    # window rate-correction, config nis_gate.nis_window_s) -- this
    # experiment block, which used to compute the rate-corrected window
    # by hand for comparison against the old literal, is retired: the
    # live gate now IS the rate-corrected one, so evaluate_gate's own
    # returned window_samples is already the corrected value.
    nis_gate_verdict = evaluate_gate(manifest["nis_full"], manifest["base_mask"], params, state["sample_rate_hz"])
    print(f"\nNIS gate (live config, rate-corrected window_samples="
          f"{nis_gate_verdict['window_samples']} at {state['sample_rate_hz']:.1f} Hz): {nis_gate_verdict}")

    # --- Input population audit -----------------------------------------
    base_mask = manifest["base_mask"]
    from modules.stability_analysis import estimate_sideslip, estimate_slip_angles, estimate_lateral_forces
    beta_kin = estimate_sideslip(state, params)
    slip_kin = estimate_slip_angles(state, beta_kin, params)
    forces = estimate_lateral_forces(state, params)
    alpha_f = slip_kin["alpha_f_filt"][base_mask]
    alpha_r = slip_kin["alpha_r_filt"][base_mask]
    print(f"\n[population] base_mask population n={int(np.sum(base_mask))} / {len(base_mask)} total samples")
    print(f"[population] front alpha range: [{np.degrees(np.nanmin(alpha_f)):.2f}, "
          f"{np.degrees(np.nanmax(alpha_f)):.2f}] deg, "
          f"cornering fraction (|alpha_f|>1deg): {float(np.mean(np.abs(np.degrees(alpha_f)) > 1.0)):.3f}")
    print(f"[population] rear alpha range: [{np.degrees(np.nanmin(alpha_r)):.2f}, "
          f"{np.degrees(np.nanmax(alpha_r)):.2f}] deg, "
          f"cornering fraction (|alpha_r|>1deg): {float(np.mean(np.abs(np.degrees(alpha_r)) > 1.0)):.3f}")

    return {
        "manifest": manifest, "state": state, "slip_kin": slip_kin, "forces": forces,
        "base_mask": base_mask, "nis_gate_verdict": nis_gate_verdict,
    }


def plot_tyre_cloud_with_fit(label, result, out_dir):
    manifest = result["manifest"]
    base_mask = result["base_mask"]
    slip_kin = result["slip_kin"]
    forces = result["forces"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, axle, alpha_key, fy_key in (
        (axes[0], "front", "alpha_f_filt", "Fy_f_filt"),
        (axes[1], "rear", "alpha_r_filt", "Fy_r_filt"),
    ):
        alpha = slip_kin[alpha_key][base_mask]
        Fy = forces[fy_key][base_mask]
        ax.scatter(np.degrees(alpha), Fy / 1000.0, s=2, alpha=0.15, color="grey", label="session cloud")

        f = manifest["axles"][axle]
        alpha_grid_deg = np.linspace(-15, 15, 400)
        fy_pred = pacejka_lateral_force(np.radians(alpha_grid_deg), f["B"], f["C"], f["D"], f["E"])
        ax.plot(alpha_grid_deg, fy_pred / 1000.0, color="red", linewidth=1.5,
                label=f"fit (B={f['B']:.2f} C={f['C']:.2f} D={f['D']:.0f} E={f['E']:.2f})")

        ax.set_title(f"{axle} ({label})")
        ax.set_xlabel("Slip angle (deg)")
        ax.set_ylabel("Fy (kN)")
        ax.axhline(0, color="black", linewidth=0.5)
        ax.axvline(0, color="black", linewidth=0.5)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

    fig.suptitle(f"Session tyre cloud vs attempted Pacejka fit -- {label}")
    fig.tight_layout()
    out_path = os.path.join(out_dir, f"tyre_cloud_fit_{label.replace(' ', '_')}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    params = load_parameters()

    dubai_result = audit_session("Dubai (reference)", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt", params)
    v3_result = audit_session("GT3_PRC_MLA-v3", "GT3_PRC_MLA-v3.txt", params)

    if v3_result:
        path = plot_tyre_cloud_with_fit("v3", v3_result, OUT_DIR)
        print(f"\nwrote {path}")
    if dubai_result:
        path = plot_tyre_cloud_with_fit("Dubai", dubai_result, OUT_DIR)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
