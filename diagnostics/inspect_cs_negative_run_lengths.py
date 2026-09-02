# Diagnostic, read-only, PROPOSAL Part 2 input: sizes the persistence
# statistic (Part 2 option 3) directly on the raw per-sample CS_ratio
# series, independent of the phase-median machinery the prior two
# investigations showed losing C4's signal in both directions (apex
# included: noisy small-n; apex excluded: diluted by wide surrounding
# phases -- thesis_notes.md "Gated Stage-2 recomputation..."). Finds
# every contiguous run of CS_ratio < 0 samples over each corner's own
# canonical bracket, per lap, per axle, across all 14 physical corners,
# converts run length to metres via s_m, and reports the distribution.
# No config or estimator change.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from diagnostics.inspect_step2_chair_plots import _canonical_window_slice

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODE = "ekf_auto_pacejka"
HIGHLIGHT_IDS = (1, 2, 3, 4, 8, 9)
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _phase_slices(t, segments, apex_half_window_samples):
    out = {}
    for phase in PHASE_KEYS:
        start_t, end_t = segments[phase]
        if end_t < start_t:
            out[phase] = slice(0, 0)
            continue
        lo = int(np.searchsorted(t, start_t, side="left"))
        hi = int(np.searchsorted(t, end_t, side="right"))
        if phase == "apex_3" and hi <= lo:
            centre = lo
            lo = max(0, centre - apex_half_window_samples)
            hi = min(len(t), centre + apex_half_window_samples + 1)
        out[phase] = slice(lo, hi)
    return out


def _phase_of_index(idx, phase_slices):
    for phase, sl in phase_slices.items():
        if sl.start <= idx < sl.stop:
            return phase
    return "none/between-phases"


def _negative_runs(values, s_m_local, global_offset):
    # values, s_m_local: arrays over the bracket slice (local indexing).
    # Returns list of dicts with local start/end (inclusive), global
    # centre index, length_samples, length_m, depth (median in run).
    runs = []
    n = len(values)
    i = 0
    while i < n:
        if values[i] == values[i] and values[i] < 0:
            j = i
            while j + 1 < n and values[j + 1] == values[j + 1] and values[j + 1] < 0:
                j += 1
            length_samples = j - i + 1
            length_m = float(s_m_local[j] - s_m_local[i])
            depth = float(np.median(values[i:j + 1]))
            centre_local = (i + j) // 2
            runs.append({
                "start_local": i, "end_local": j, "length_samples": length_samples,
                "length_m": length_m, "depth": depth,
                "centre_global": centre_local + global_offset,
            })
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
    v_mps = state["v_mps"]
    sample_rate_hz = state["sample_rate_hz"]

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
    apex_half_window_samples = se["apex_half_window_samples"]

    laps = data.get("laps", [])
    laps_by_number = {l["lap_number"]: l for l in laps}
    valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}

    corners_by_id = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_id.setdefault(sid, []).append(c)
    stable_ids = sorted(corners_by_id)

    alpha_arrs = {"f": slip["alpha_f_filt"], "r": slip["alpha_r_filt"]}
    cs_ratio_arrs = {"f": cs["CS_ratio_f"], "r": cs["CS_ratio_r"]}

    all_runs = []  # pooled across all 14 corners x 4 laps x both axles, for item 3
    per_corner_runs = {cid: {"f": [], "r": []} for cid in stable_ids}  # longest-run tracking

    for cid in stable_ids:
        for c in corners_by_id[cid]:
            lap_number = c["lap_number"]
            if lap_number not in valid_lap_numbers:
                continue
            lap = laps_by_number[lap_number]
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                          c["bracket_start_m"], c["bracket_end_m"])
            if sl.stop <= sl.start:
                continue
            phase_slices = _phase_slices(t, c["segments"], apex_half_window_samples)

            for axle_key in ("f", "r"):
                values = cs_ratio_arrs[axle_key][sl]
                s_m_local = s_m[sl]
                runs = _negative_runs(values, s_m_local, sl.start)
                for run in runs:
                    run["lap_number"] = lap_number
                    run["phase"] = _phase_of_index(run["centre_global"], phase_slices)
                    all_runs.append({"cid": cid, "axle": axle_key, **run})
                per_corner_runs[cid][axle_key].extend(
                    {"lap_number": lap_number, "phase": _phase_of_index(r["centre_global"], phase_slices), **r}
                    for r in runs
                )

    # --- items 1 & 2: longest run per lap, for the highlighted corners ---
    print(f"{'=' * 90}\nITEMS 1 & 2 -- longest below-zero run per lap, highlighted corners "
          f"({MODE})\n{'=' * 90}")
    for cid in HIGHLIGHT_IDS:
        for axle_label, axle_key in (("front", "f"), ("rear", "r")):
            runs = per_corner_runs.get(cid, {}).get(axle_key, [])
            print(f"\n-- C{cid} {axle_label} --")
            if not runs:
                print("  no below-zero runs found on any valid lap")
                continue
            by_lap = {}
            for r in runs:
                ln = r["lap_number"]
                if ln not in by_lap or r["length_m"] > by_lap[ln]["length_m"]:
                    by_lap[ln] = r
            for ln in sorted(by_lap):
                r = by_lap[ln]
                print(f"  lap {ln}: longest run = {r['length_m']:.1f} m "
                      f"({r['length_samples']} samples), depth(median)={r['depth']:.3f}, "
                      f"phase@centre={r['phase']}")

    # --- item 3: pooled distribution across all 14 corners x 4 laps ---
    print(f"\n{'=' * 90}\nITEM 3 -- pooled below-zero run-length distribution, "
          f"ALL {len(stable_ids)} corners x valid laps x both axles ({MODE})\n{'=' * 90}")
    lengths_m = np.array([r["length_m"] for r in all_runs])
    print(f"  n_runs={len(lengths_m)}")
    if lengths_m.size:
        for p in (50, 90, 99):
            print(f"  p{p} = {np.percentile(lengths_m, p):.2f} m")
        print(f"  max = {lengths_m.max():.2f} m")
        top = sorted(all_runs, key=lambda r: -r["length_m"])[:10]
        print("  top 10 longest runs overall:")
        for r in top:
            print(f"    C{r['cid']} {r['axle']} lap{r['lap_number']}: {r['length_m']:.1f} m, "
                  f"depth={r['depth']:.3f}, phase@centre={r['phase']}")

    # --- item 4: local window footprint in metres, for scale ---
    print(f"\n{'=' * 90}\nITEM 4 -- local CS_ratio window footprint in metres, for scale "
          f"({MODE})\n{'=' * 90}")
    footprint_m = []
    moving = state["moving_mask"]
    for cid in HIGHLIGHT_IDS:
        for c in corners_by_id[cid]:
            if c["lap_number"] not in valid_lap_numbers:
                continue
            lap = laps_by_number[c["lap_number"]]
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                          c["bracket_start_m"], c["bracket_end_m"])
            if sl.stop <= sl.start:
                continue
            idx_all = np.where(moving[sl])[0] + sl.start
            for axle_key in ("f", "r"):
                alpha_arr = alpha_arrs[axle_key]
                cs_vals = cs_ratio_arrs[axle_key][idx_all]
                valid_idx = idx_all[np.isfinite(cs_vals)]
                if valid_idx.size == 0:
                    continue
                sample_idx = valid_idx[len(valid_idx) // 2]
                start = reconstruct_cs_window_start(alpha_arr, int(sample_idx), min_window, min_span)
                footprint_m.append(float(s_m[sample_idx] - s_m[start]))
    footprint_m = np.array(footprint_m)
    racing_v = v_mps[state["moving_mask"]]
    if footprint_m.size:
        print(f"  footprint-in-metres samples (n={footprint_m.size}, one representative window per "
              f"highlighted corner/lap/axle): median={np.median(footprint_m):.1f} m, "
              f"p10={np.percentile(footprint_m, 10):.1f} m, p90={np.percentile(footprint_m, 90):.1f} m")
    if racing_v.size:
        v_med = float(np.median(racing_v))
        print(f"  reference racing speed (median of moving samples): {v_med:.1f} m/s "
              f"({v_med * 3.6:.0f} km/h)")
        for n_samples_ref in (10, 15, 20, 30):
            print(f"    at {v_med * 3.6:.0f} km/h: a {n_samples_ref}-sample window covers "
                  f"{n_samples_ref / sample_rate_hz * v_med:.1f} m")


if __name__ == "__main__":
    main()
