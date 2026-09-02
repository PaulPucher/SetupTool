# Diagnostic, read-only, PROPOSAL input: evaluates three candidate
# geometric fold-vs-loop separators -- all reusing modules.stability_
# analysis's own chair-derived monotonic-section machinery
# (_find_monotonic_sections/_section_slopes), per the work order's
# "check the chair's own machinery first" instruction -- against the
# 10 ground-truth runs already verdicted by hand (thesis_notes.md
# "Ground-truth workup..."): 5 REAL (fold/peak), 4 ARTIFACT (loop), 1
# MIXED. No config or estimator change; no criterion is adopted here,
# only evaluated.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    reconstruct_cs_window_start, _find_monotonic_sections, _section_slopes,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from diagnostics.inspect_step2_chair_plots import _canonical_window_slice

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"

# (stable_corner_id, axle, lap_number, verdict) -- ground truth from
# thesis_notes.md "Ground-truth workup: per-run verdicts...".
GROUND_TRUTH = [
    (3, "r", 1, "ARTIFACT"), (3, "r", 2, "ARTIFACT"), (3, "r", 3, "MIXED"),
    (2, "f", 2, "ARTIFACT"), (2, "f", 3, "ARTIFACT"),
    (4, "f", 1, "REAL"), (4, "f", 3, "REAL"), (4, "f", 4, "REAL"),
    (4, "r", 3, "REAL"), (4, "r", 4, "REAL"),
]


def _negative_runs(values, s_m_local, global_offset):
    runs = []
    n = len(values)
    i = 0
    while i < n:
        if values[i] == values[i] and values[i] < 0:
            j = i
            while j + 1 < n and values[j + 1] == values[j + 1] and values[j + 1] < 0:
                j += 1
            runs.append({"start_global": i + global_offset, "end_global": j + global_offset})
            i = j + 1
        else:
            i += 1
    return runs


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    t = state["time"]
    s_m = state["s_m"]

    beta, _fm, _gv, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, MODE, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(f"{MODE} fell back to kinematic ({fallback_reason}) -- refusing to run")

    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)

    se = params["stability_estimation"]
    min_window = se["cs_min_window_samples"]
    min_span = se["cs_min_slip_angle_span_rad"]

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    corners_by_id = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_id.setdefault(sid, []).append(c)

    alpha_arrs = {"f": slip["alpha_f_filt"], "r": slip["alpha_r_filt"]}
    cs_ratio_arrs = {"f": cs["CS_ratio_f"], "r": cs["CS_ratio_r"]}
    c_window_arrs = {"f": cs["C_window_f"], "r": cs["C_window_r"]}
    c_section_arrs = {"f": cs["C_section_f"], "r": cs["C_section_r"]}

    print(f"{'run':<22} {'verdict':<9} {'idx':>7} {'A frac_qualifying':>18} "
          f"{'B reversals':>12} {'C |Cw-Cs|/max':>15}")

    results = []
    for cid, axle_key, lap_number, verdict in GROUND_TRUTH:
        c = next(cc for cc in corners_by_id[cid] if cc["lap_number"] == lap_number)
        lap = laps_by_number[lap_number]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        cs_ratio_arr = cs_ratio_arrs[axle_key]
        runs = _negative_runs(cs_ratio_arr[sl], s_m[sl], sl.start)
        run = max(runs, key=lambda r: r["end_global"] - r["start_global"])
        rsl = slice(run["start_global"], run["end_global"] + 1)

        # Worst (most negative) single sample within the run -- the
        # governing sample, same precedent as every prior investigation.
        seg = cs_ratio_arr[rsl]
        worst_local = int(np.nanargmin(np.where(np.isfinite(seg), seg, np.inf)))
        idx = run["start_global"] + worst_local

        alpha_arr = alpha_arrs[axle_key]
        window_start = reconstruct_cs_window_start(alpha_arr, idx, min_window, min_span)
        window_alpha = alpha_arr[window_start:idx]
        window_Fy = forces["Fy_f_filt" if axle_key == "f" else "Fy_r_filt"][window_start:idx]
        n_window = len(window_alpha)

        # Candidate A: fraction of the window's samples that belong to a
        # LOCAL monotonic section (computed on the window alone, not the
        # whole session) whose own alpha span clears cs_min_slip_angle_
        # span_rad -- the SAME span-qualification C_section already
        # applies via _smooth_weight before trusting a section.
        sections, section_id = _find_monotonic_sections(window_alpha)
        sec_slopes, sec_spans = _section_slopes(window_alpha, window_Fy, sections)
        qualifying = {k for k, sp in enumerate(sec_spans) if sp >= min_span}
        frac_qualifying = float(np.mean([section_id[i] in qualifying for i in range(n_window)])) \
            if n_window else float("nan")

        # Candidate B: alpha direction-reversal count within the window
        # (sign changes in diff(alpha), zeros carried forward -- same
        # convention _find_monotonic_sections itself uses).
        d = np.diff(window_alpha)
        sign = np.sign(d)
        for k in range(1, len(sign)):
            if sign[k] == 0:
                sign[k] = sign[k - 1]
        reversals = int(np.sum((sign[1:] != sign[:-1]) & (sign[1:] != 0) & (sign[:-1] != 0)))

        # Candidate C: relative disagreement between the window's own raw
        # regression slope and the monotonic-section-blended slope, at
        # the governing sample -- already-computed production arrays.
        c_w = c_window_arrs[axle_key][idx]
        c_s = c_section_arrs[axle_key][idx]
        if np.isfinite(c_w) and np.isfinite(c_s) and max(abs(c_w), abs(c_s)) > 1e-6:
            rel_disagreement = abs(c_w - c_s) / max(abs(c_w), abs(c_s))
        else:
            rel_disagreement = float("nan")

        label = f"C{cid}{axle_key} lap{lap_number}"
        print(f"{label:<22} {verdict:<9} {idx:>7} {frac_qualifying:>18.3f} "
              f"{reversals:>12d} {rel_disagreement:>15.3f}")
        results.append({
            "label": label, "verdict": verdict, "n_window": n_window,
            "frac_qualifying": frac_qualifying, "reversals": reversals,
            "rel_disagreement": rel_disagreement,
        })

    print(f"\nn_window (regression window sample count) at the governing sample, for reference:")
    for r in results:
        print(f"  {r['label']:<22} n_window={r['n_window']}")

    print("\nSeparation check -- REAL vs ARTIFACT (MIXED excluded) per candidate:")
    real = [r for r in results if r["verdict"] == "REAL"]
    art = [r for r in results if r["verdict"] == "ARTIFACT"]
    for key in ("frac_qualifying", "reversals", "rel_disagreement"):
        real_vals = [r[key] for r in real if r[key] == r[key]]
        art_vals = [r[key] for r in art if r[key] == r[key]]
        print(f"  {key}: REAL range=[{min(real_vals):.3f}, {max(real_vals):.3f}], "
              f"ARTIFACT range=[{min(art_vals):.3f}, {max(art_vals):.3f}], "
              f"{'CLEAN GAP' if (min(real_vals) > max(art_vals) or max(real_vals) < min(art_vals)) else 'OVERLAP'}")


if __name__ == "__main__":
    main()
