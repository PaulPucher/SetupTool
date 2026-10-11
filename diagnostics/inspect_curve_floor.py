# Curve floor vs classification thresholds (PLAN.md STATUS 2026-10-05, author
# EXECUTE; read-only, no config write).
#
# RULES, frozen 2026-10-05 BEFORE any number.
# SESSIONS: the seven-session threshold pool (dubai, v3, pr_us_v2,
#   nbr_jst_problem, nbr_jst_solution, nbr_qp_cdr, nbr_qp_mla).
# CURVES: the production Pacejka curve per session and axle at best available
#   -- six from the 2026-10-05 production fit caches (d2 fit manifest), Dubai
#   fitted fresh once (fit_session_pacejka, configured modes).
# FLOOR: min over slip angle of (dFy/dalpha) / (B C D) -- the curve's own
#   analogue of the cornering stiffness ratio (1 at the origin, 0 at the peak,
#   negative past it) -- on the VISITED range 0 .. p99 |alpha| of the session's
#   production fit population (manifest visited_alpha_p99_deg). Beside it: the
#   floor over 0 .. 30 deg (Q-CHAIR-AUDIT convention) and whether the peak lies
#   inside the visited range.
# COMPARISON per axle against STRONG / MODERATE (live config): per session and
#   pooled (pool floor = the deepest session floor). Pre-stated outcomes
#   (author): thresholds ABOVE the floor -> "unusually deep for this car";
#   STRONG BELOW the floor -> "anomalous force loss beyond the quasi-steady
#   characteristic (kerb-shown majority per audit; mechanism open otherwise)";
#   mixed -> per axle. Caveat stated with the result: thresholds act on corner
#   values (phase medians, worst lap, worst phase), the floor is pointwise.
#
# Output: diagnostics/plots/curve_floor/report.txt, floors.json

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "curve_floor")
POOL = ("dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla")
NAMES = {"dubai": "Dubai sample", "v3": "Paul Ricard, MLA outing 25", "pr_us_v2": "Paul Ricard, MLA outing 32",
         "nbr_jst_problem": "Nurburgring, JST outing 21", "nbr_jst_solution": "Nurburgring, JST outing 29",
         "nbr_qp_cdr": "Nurburgring qualifying, CDR", "nbr_qp_mla": "Nurburgring qualifying, MLA"}
AX = {"front": "f", "rear": "r"}


def _axles(key):
    if key != "dubai":
        return d2.load_fit(key)["prod"]["manifest"]["axles"]
    from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
    from modules.csv_parser import parse_csv
    from modules.tyre_fit_auto import fit_session_pacejka
    params = d2._params()
    tf = params["tyre_fit_auto"]
    man = fit_session_pacejka(parse_csv(SESSIONS[key]), params, data_file_path=SESSIONS[key],
                              load_normalised=tf.get("load_normalised_fit_enabled", False),
                              tyre_fit_mode=tf.get("tyre_fit_mode", "per_session"))
    return man["axles"]


def _floor(ax, hi_deg):
    # promoted 2026-10-06 to modules/limit_events.curve_floor (same grid, same
    # normalisation); this check now reads the production function
    from modules.limit_events import curve_floor
    return curve_floor(ax, hi_deg)


def main():
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    th = {"f": (cl["STRONG_CSF"]["value"], cl["MODERATE_CSF"]["value"]),
          "r": (cl["STRONG_CSR"]["value"], cl["MODERATE_CSR"]["value"])}
    out = {}
    L = ["CURVE FLOOR vs THRESHOLDS (production curves, best available; floor = min dFy/dalpha / BCD on 0 .. p99 "
         "|alpha| of the fit population)",
         f"thresholds: front STRONG {th['f'][0]} / MODERATE {th['f'][1]}; rear STRONG {th['r'][0]} / "
         f"MODERATE {th['r'][1]}"]
    for k in POOL:
        axes = _axles(k)
        out[k] = {}
        for axn, a in AX.items():
            ax = axes[axn]
            p99 = float(ax["visited_alpha_p99_deg"])
            fv, at = _floor(ax, p99)
            f30, at30 = _floor(ax, 30.0)
            out[k][a] = {"floor_visited": fv, "at_deg": at, "p99_deg": p99, "floor_30": f30, "at30_deg": at30,
                         "peak_deg": ax.get("peak_alpha_deg"), "peak_in_visited": ax.get("peak_in_visited_range"),
                         "B": ax["B"], "C": ax["C"], "D": ax["D"], "E": ax["E"]}
    for axn, a in AX.items():
        S, M = th[a]
        L.append(f"\n{axn.upper()} AXLE:")
        below = 0
        for k in POOL:
            r = out[k][a]
            pos = ("STRONG below floor" if S < r["floor_visited"] else
                   "STRONG above floor" + (", MODERATE below floor" if M < r["floor_visited"] else ""))
            below += S < r["floor_visited"]
            L.append(f"  {NAMES[k]}: floor {r['floor_visited']:+.3f} at {r['at_deg']:.1f} deg (visited p99 "
                     f"{r['p99_deg']:.1f} deg; peak {r['peak_deg']:.1f} deg, in range {r['peak_in_visited']}); "
                     f"0-30 deg floor {r['floor_30']:+.3f} -> {pos}")
        pool_floor = min(out[k][a]["floor_visited"] for k in POOL)
        case = ("STRONG BELOW the floor on every session" if below == len(POOL) else
                "thresholds ABOVE the floor on every session" if below == 0 else
                f"MIXED ({below} of {len(POOL)} sessions with STRONG below their floor)")
        L.append(f"  POOLED: deepest session floor {pool_floor:+.3f}; STRONG {S} is "
                 f"{'below' if S < pool_floor else 'above'} it; MODERATE {M} is "
                 f"{'below' if M < pool_floor else 'above'} it. Per session: {case}.")
        out[f"pool_{a}"] = {"pool_floor": pool_floor, "sessions_strong_below": below, "case": case}
    L.append("\nCAVEAT: thresholds act on corner values (per-phase medians, worst lap, worst phase); the floor is a "
             "pointwise property of the fitted curve.")
    txt = "\n".join(L)
    print(txt)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    with open(os.path.join(OUT_DIR, "floors.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)


if __name__ == "__main__":
    main()
