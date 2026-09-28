# Gated-kinematic diagnostic, step 1 of 4: compute and cache.
# Per session: every beta source, and for the CS-relevant sources the full
# production chain (slip -> Fy -> CS -> yaw stability -> summarise_corners),
# called function-for-function as tests/conftest.py's pipeline_result does.
# Output: diagnostics/plots_gated_kinematic/_cache/<session>.pkl, read by
# the three analysis scripts. Read-only: no production file, no config.
#
# Run from the repo root:  python -m diagnostics.inspect_gated_kinematic_run [dubai|v3]

import sys
import time

import numpy as np

from modules.stability_analysis import (
    estimate_sideslip, estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from diagnostics.gated_kinematic_common import (
    SESSIONS, GATE_FACTORS, load_session, racing_mask, otsu_threshold,
    gated_beta, causal_washout_beta, save_cache,
)

CAUSAL_WASHOUT_HZ = (0.05, 0.02)


def run(name):
    t0 = time.time()
    data, params, state = load_session(name)
    laps = data.get("laps", [])
    bg = racing_mask(state, laps)
    abs_ay = np.abs(state["ay_mps2"][bg])
    thr_otsu = otsu_threshold(abs_ay)
    print(f"[{name}] sr={state['sample_rate_hz']:.1f} Hz, racing samples={int(bg.sum())}, "
          f"Otsu |ay| threshold={thr_otsu:.3f} m/s^2 "
          f"(percentile of racing |ay|: {100.0 * np.mean(abs_ay < thr_otsu):.1f})")

    betas, gates = {}, {}
    betas["kinematic_prod"] = estimate_sideslip(state, params)
    ekf_beta, manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, "ekf_auto_pacejka", csv_path=SESSIONS[name])
    betas["ekf_auto_pacejka"] = ekf_beta
    print(f"[{name}] ekf_auto_pacejka: gate={gate_verdict}, fallback_used={fallback_used} {fallback_reason or ''}")
    for f in GATE_FACTORS:
        key = f"gated_x{f:g}"
        b, g, rid = gated_beta(state, f * thr_otsu)
        betas[key] = b
        gates[key] = {"threshold": f * thr_otsu, "gate": g, "run_id": rid}
    for hz in CAUSAL_WASHOUT_HZ:
        betas[f"causal_washout_{hz:g}hz"] = causal_washout_beta(state, hz)

    forces = estimate_lateral_forces(state, params)
    cs_sources = ["kinematic_prod", "ekf_auto_pacejka"] + [f"gated_x{f:g}" for f in GATE_FACTORS]
    chains = {}
    for src in cs_sources:
        ts = time.time()
        slip = estimate_slip_angles(state, betas[src], params)
        cs = estimate_cornering_stiffness(slip, forces, state, params)
        stab = estimate_yaw_moment_stability(state, betas[src], params, laps)
        summaries = summarise_corners(data.get("corners", []), cs, stab, state, fz=None, lap_filter=None)
        chains[src] = {
            "alpha_f": slip["alpha_f_filt"], "alpha_r": slip["alpha_r_filt"],
            "CS_ratio_f": cs["CS_ratio_f"], "CS_ratio_r": cs["CS_ratio_r"],
            "C_alpha_f": cs["C_alpha_f"], "C_alpha_r": cs["C_alpha_r"],
            "summaries": summaries,
        }
        print(f"[{name}] chain {src}: {time.time() - ts:.0f}s")

    keep = ("time", "s_m", "v_mps", "ay_mps2", "yaw_rate_radps", "moving_mask", "kerb_mask", "sample_rate_hz")
    save_cache(name, {
        "session": name,
        "state": {k: state.get(k) for k in keep},
        "laps": laps, "corners": data.get("corners", []),
        "racing_mask": bg, "thr_otsu": thr_otsu,
        "betas": betas, "gates": gates, "chains": chains,
        "Fy_f": forces["Fy_f_filt"], "Fy_r": forces["Fy_r_filt"],
        "ekf": {"gate_verdict": gate_verdict, "fallback_used": fallback_used,
                "fallback_reason": fallback_reason},
    })
    print(f"[{name}] cached, total {time.time() - t0:.0f}s")


if __name__ == "__main__":
    for n in (sys.argv[1:] or list(SESSIONS)):
        run(n)
