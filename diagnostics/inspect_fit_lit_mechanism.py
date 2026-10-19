# Q-FIT-LIT verification, POST-HOC mechanism check of the exit (2026-10-18;
# the frozen exit of inspect_fit_lit.py asks for "the failing check and its
# mechanism"). Fit L failed on every session through the NIS gross
# miscalibration trigger (ay exceedance > nis_gross_miscalibration_fraction).
# No new fit variant: the observer is re-run with L's own fitted curves and
# L's final R, removing one ingredient of the L force model at a time, and
# production's own final run is recomputed as the reference.
#   M1 L as run: D = mu Fz(t), front in the tyre frame with Fx_f(t)
#   M2 as M1 with Fx_f = 0 (the Fx sin(delta) term removed)
#   M3 as M2 with Fz(t) = Fz_ref (load variation removed; cos(delta) kept)
#   M4 constant-D path, D = mu Fz_ref (the plain curve, no frame term)
#   M5 L curve (as M4) with production's final R instead of L's
#   P  production fit and its own final R (reference)
# Statistic: share of base-mask samples with ay NIS > chi2(1) 95 % -- the
# quantity the gross trigger reads -- plus the yaw share and the RMS of the
# ay innovation. Run: python -m diagnostics.inspect_fit_lit_mechanism

import copy
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_fit_lit import SESSIONS, load_session  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka  # noqa: E402

OUT = "diagnostics/output/fit_lit/mechanism.log"


def stats(S, params, cfg, state):
    p = dict(params)
    p["tyre_model_ekf_pacejka"] = {"_m": cfg}
    r = estimate_sideslip_ekf_pacejka(state, p, pass_id="_m")
    inn, sd = r["innovation"][S["base"]], r["S_diag"][S["base"]]
    f_ay = float((inn[:, 1] ** 2 / sd[:, 1] > tfa.CHI2_DF1_95).mean())
    f_yaw = float((inn[:, 0] ** 2 / sd[:, 0] > tfa.CHI2_DF1_95).mean())
    return f_ay, f_yaw, float(np.sqrt(np.mean(inn[:, 1] ** 2)))


def main():
    lines = []

    def log(m):
        print(m, flush=True)
        lines.append(m)

    for key, label, rel in SESSIONS:
        S = load_session(key, label, rel)
        params_l = S["params"]
        params_p = copy.deepcopy(params_l)
        params_p["tyre_fit_auto"]["tyre_fit_mode"] = "per_session"
        man_l = tfa.fit_session_pacejka(S["data"], params_l, tyre_fit_mode="literature_bounded")
        man_p = tfa.fit_session_pacejka(S["data"], params_p, tyre_fit_mode="per_session")
        cl, cp = man_l["final_config"], man_p["final_config"]
        li = S["li"]
        st_full = dict(S["state"], fz_axle_f_N=li["fz_f_N"], fz_axle_r_N=li["fz_r_N"], fx_axle_f_N=li["fx_f_N"])
        zero = np.zeros_like(li["fx_f_N"])
        st_nofx = dict(st_full, fx_axle_f_N=zero)
        fzf, fzr = man_l["axles"]["front"]["fz_ref_N"], man_l["axles"]["rear"]["fz_ref_N"]
        st_const = dict(st_nofx, fz_axle_f_N=np.full_like(zero, fzf), fz_axle_r_N=np.full_like(zero, fzr))
        c4 = dict(cl)
        c4.pop("d_is_mu")
        c4["d_front"], c4["d_rear"] = cl["d_front"] * fzf, cl["d_rear"] * fzr
        c5 = dict(c4, R_ay_var=cp["R_ay_var"], R_yaw_rate_var=cp["R_yaw_rate_var"])
        log("=" * 90)
        log(f"{label}: L status {man_l['status']} (manifest ay exceedance {man_l['nis']['ay_exceedance']:.3f}); "
            f"P status {man_p['status']} (manifest ay exceedance {man_p['nis']['ay_exceedance']:.3f})")
        log(f"  R_ay_var L {cl['R_ay_var']:.4f} / P {cp['R_ay_var']:.4f}; R_yaw L {cl['R_yaw_rate_var']:.3e} / "
            f"P {cp['R_yaw_rate_var']:.3e}; R pick L {man_l['r_sweep']['selection']}, P {man_p['r_sweep']['selection']}")
        for name, cfg, st in (("M1 L as run", cl, st_full), ("M2 no Fx term", cl, st_nofx),
                              ("M3 no Fx, constant Fz", cl, st_const), ("M4 plain L curve, constant D", c4, S["state"]),
                              ("M5 plain L curve, production R", c5, S["state"]), ("P production", cp, S["state"])):
            f_ay, f_yaw, rms = stats(S, params_l, cfg, st)
            log(f"  {name}: ay exceedance {f_ay:.3f}, yaw exceedance {f_yaw:.3f}, ay innovation RMS {rms:.3f} m/s^2")
    with open(OUT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
