# v3 sawtooth investigation, Phase 7 (2026-09-02): GT3_PRC_MLA-v3 shows its
# own version of the Dubai "sawtooth" artifact (thesis_notes.md, "Mechanism
# investigation: wholesale-negative CS_ratio under ekf_auto_pacejka",
# FINDING 2) in at least stable_corner_id 13. This script gathers evidence
# only -- window-level stats, floor-fraction comparison against Dubai,
# alpha signal character, and a fold-vs-loop self-crossing check -- to
# decide whether the SAME mechanism (small-window regression instability at
# cs_min_window_samples/cs_min_slip_angle_span_rad) explains v3's sawtooth,
# or whether v3's own corner/alpha character differs enough to matter.
#
# Read-only. No config, estimator, or threshold change. sideslip_source is
# overridden only in a local copy.deepcopy'd params dict (never written to
# config/parameters.json). Reuses production functions exactly:
# modules.stability_analysis.{resolve_cs_min_window_samples,
# reconstruct_cs_window_start, estimate_cornering_stiffness, ...} and
# modules.tyre_fit_auto.resolve_sideslip_beta -- no window-growth or
# beta-selection logic is reimplemented here. Also reuses diagnostics/
# inspect_step2_chair_plots.py's _valid_lap_instances/_canonical_window_
# slice (corner-bracket geometry, not estimator logic) rather than a
# second copy.
#
# Tier B (signal/data engineering: window reconstruction, floor-fraction
# bookkeeping, sign-change/self-crossing metrics) -- standard techniques,
# no methodological novelty claimed. Evidence gathering only; no fix
# proposed or made, per the work order's explicit instruction.
#
# [keep-reproduces]: this is a live, open investigation thread (v3's
# sawtooth cause is unresolved) that will need re-running once the paused
# threshold re-derivation work (PLAN.md STATUS) reopens. See thesis_notes.md
# "v3 sawtooth mechanism investigation" entry for the recorded findings.

import copy

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness, resolve_cs_min_window_samples,
    reconstruct_cs_window_start,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from diagnostics.inspect_step2_chair_plots import _valid_lap_instances, _canonical_window_slice

V3_FILE = "GT3_PRC_MLA-v3.txt"
DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

DUBAI_MECHANISM_CORNERS = (1, 2, 3, 4, 8, 9)  # thesis_notes.md Finding 3 set


# --- pipeline plumbing ----------------------------------------------------

def _params_with_source(base_params, source):
    p = copy.deepcopy(base_params)
    p["stability_estimation"]["sideslip_source"] = source
    return p


def run_pipeline(raw_file, params):
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        raise RuntimeError(f"{raw_file}: prepare_vehicle_state returned None")
    source = params["stability_estimation"]["sideslip_source"]
    beta, _fit_manifest, _gate, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, source, csv_path=raw_file)
    if fallback_used:
        print(f"  ** NOTE: {raw_file} sideslip_source={source!r} fell back to kinematic: {fallback_reason}")
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    corners_by_stable_id = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
    return {
        "data": data, "state": state, "slip": slip, "forces": forces, "cs": cs,
        "corners_by_stable_id": corners_by_stable_id, "laps_by_number": laps_by_number,
    }


def _sign_change_rate(arr):
    finite = np.isfinite(arr)
    vals = arr[finite]
    if vals.size < 3:
        return float("nan"), int(vals.size)
    signs = np.sign(vals)
    changes = int(np.sum(signs[1:] != signs[:-1]))
    return changes / (vals.size - 1), int(vals.size)


# --- Step 1: corner selection ---------------------------------------------

def select_corners(pipe, forced_id):
    """Oscillation metric: sign-change rate of CS_ratio across consecutive
    finite-valued samples, per axle per valid lap instance, within each
    corner's own canonical bracket slice (same slice geometry the
    figure-export pipeline uses). Corner score = MAX rate over its
    axle/instance combos (the worst-oscillating combo), since that is what
    a visual "sawtooth" survey would flag first. Also records WHICH
    axle/instance combo produced that max, for reuse in Finding 1 so the
    same physical stretch is compared across both beta sources.
    """
    state, cs = pipe["state"], pipe["cs"]
    t, s_m = state["time"], state["s_m"]
    corners_by_stable_id, laps_by_number = pipe["corners_by_stable_id"], pipe["laps_by_number"]

    scored = []
    for cid, insts_all in corners_by_stable_id.items():
        instances = _valid_lap_instances(insts_all, laps_by_number)
        if not instances:
            continue
        best = None  # (rate, axle, lap_number, slice, n)
        for axle, key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
            arr = cs[key]
            for c in instances:
                lap = laps_by_number[c["lap_number"]]
                sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                              c["bracket_start_m"], c["bracket_end_m"])
                if sl.stop <= sl.start:
                    continue
                rate, n = _sign_change_rate(arr[sl])
                if n < 3 or not np.isfinite(rate):
                    continue
                if best is None or rate > best[0]:
                    best = (rate, axle, c["lap_number"], sl, n)
        if best is not None:
            scored.append((cid, insts_all[0].get("speed_class"), best))

    scored.sort(key=lambda row: row[2][0], reverse=True)
    print("\n=== Step 1: v3 corner oscillation ranking (ekf_auto_pacejka, CS_ratio sign-change rate) ===")
    for cid, speed_class, (rate, axle, lap_number, sl, n) in scored:
        flag = "  <== forced (C13)" if cid == forced_id else ""
        print(f"  C{cid:>2} ({speed_class:<6}) worst={axle:<5} lap={lap_number} "
              f"n={n:>4} sign_change_rate={rate:.3f}{flag}")

    others = [row for row in scored if row[0] != forced_id]
    chosen_ids = [forced_id] + [row[0] for row in others[:2]]
    print(f"\nSelected corners: {chosen_ids} (C{forced_id} forced per work order; "
          f"other two = top sign_change_rate excluding C{forced_id})")
    by_id = {cid: (speed_class, best) for cid, speed_class, best in scored}
    return chosen_ids, by_id


# --- Finding 1: window-level stats at the worst-oscillating instance ------

def window_stats_for_slice(alpha_arr, cs_ratio_arr, r2_arr, sl, min_window, min_span, s_m, max_window_m):
    idxs = np.arange(sl.start, sl.stop)
    finite = np.isfinite(cs_ratio_arr[sl])
    idxs = idxs[finite]
    if idxs.size == 0:
        return None
    ns, spans, r2s = [], [], []
    for i in idxs:
        start = reconstruct_cs_window_start(alpha_arr, int(i), min_window, min_span, s_m=s_m, max_window_m=max_window_m)
        n = int(i) - start
        if n <= 0:
            continue
        window_alpha = alpha_arr[start:int(i)]
        ns.append(n)
        spans.append(float(np.max(window_alpha) - np.min(window_alpha)))
        r2s.append(float(r2_arr[int(i)]))
    if not ns:
        return None
    ns_arr, spans_arr, r2_arr_ = np.array(ns), np.array(spans), np.array(r2s)
    sign_rate, n_finite = _sign_change_rate(cs_ratio_arr[sl])
    worst_local = int(np.nanargmin(np.where(finite, cs_ratio_arr[sl], np.inf)))
    worst_i = sl.start + worst_local
    worst_start = reconstruct_cs_window_start(alpha_arr, worst_i, min_window, min_span, s_m=s_m, max_window_m=max_window_m)
    return {
        "n_windows": int(ns_arr.size),
        "worst_sample": {
            "index": int(worst_i), "cs_ratio": float(cs_ratio_arr[worst_i]),
            "n": int(worst_i - worst_start), "span": float(np.max(alpha_arr[worst_start:worst_i]) - np.min(alpha_arr[worst_start:worst_i])),
            "r2": float(r2_arr[worst_i]),
        },
        "n_median": float(np.median(ns_arr)), "n_min": int(np.min(ns_arr)), "n_max": int(np.max(ns_arr)),
        "span_median": float(np.median(spans_arr)),
        "r2_median": float(np.median(r2_arr_)),
        "sign_change_rate": sign_rate, "n_finite_samples": n_finite,
    }


def finding1(chosen, v3_ekf_pipe, v3_kin_pipe, selection_by_id, min_span, max_window_m):
    print("\n=== Finding 1: window-level stats at the worst-oscillating instance ===")
    results = {}
    for cid in chosen:
        speed_class, (rate, axle, lap_number, sl, n) = selection_by_id[cid]
        key = "CS_ratio_f" if axle == "front" else "CS_ratio_r"
        alpha_key = "alpha_f_filt" if axle == "front" else "alpha_r_filt"
        r2_key = "R2_f" if axle == "front" else "R2_r"
        print(f"\n  C{cid} ({speed_class}), worst axle={axle}, lap={lap_number}, slice=[{sl.start}:{sl.stop}] (n_samples={sl.stop - sl.start})")
        for label, pipe in (("ekf_auto_pacejka", v3_ekf_pipe), ("kinematic", v3_kin_pipe)):
            state, slip, cs = pipe["state"], pipe["slip"], pipe["cs"]
            min_window = resolve_cs_min_window_samples(pipe["params"], state["sample_rate_hz"])
            stats = window_stats_for_slice(slip[alpha_key], cs[key], cs[r2_key], sl, min_window, min_span,
                                            state.get("s_m"), max_window_m)
            if stats is None:
                print(f"    [{label}] no finite CS_ratio samples in this slice")
                continue
            ws = stats["worst_sample"]
            stability = "UNSTABLE (flips >20% of steps)" if stats["sign_change_rate"] > 0.2 else \
                        ("borderline" if stats["sign_change_rate"] > 0.05 else "stable")
            print(f"    [{label}] n_windows={stats['n_windows']} min_window_floor={min_window} "
                  f"n_median={stats['n_median']:.0f} (min {stats['n_min']}, max {stats['n_max']}) "
                  f"span_median={stats['span_median']:.4f} rad r2_median={stats['r2_median']:.3f} "
                  f"| worst sample: n={ws['n']} span={ws['span']:.4f} rad R2={ws['r2']:.3f} CS_ratio={ws['cs_ratio']:.3f} "
                  f"| adjacent-window sign-change rate={stats['sign_change_rate']:.3f} ({stability})")
            results[(cid, label)] = stats
    return results


# --- Finding 2: fraction of windows at validity floors --------------------

def floor_fraction_for_corner(pipe, cid, axle_key, alpha_key, min_window, min_span, max_window_m, floor_tol_samples=2):
    state, slip, cs = pipe["state"], pipe["slip"], pipe["cs"]
    t, s_m = state["time"], state["s_m"]
    insts_all = pipe["corners_by_stable_id"].get(cid, [])
    instances = _valid_lap_instances(insts_all, pipe["laps_by_number"])
    ns = []
    for c in instances:
        lap = pipe["laps_by_number"][c["lap_number"]]
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        cs_ratio = cs[axle_key][sl]
        alpha_arr = slip[alpha_key]
        idxs = np.arange(sl.start, sl.stop)[np.isfinite(cs_ratio)]
        for i in idxs:
            start = reconstruct_cs_window_start(alpha_arr, int(i), min_window, min_span, s_m=s_m, max_window_m=max_window_m)
            ns.append(int(i) - start)
    if not ns:
        return None
    ns_arr = np.array(ns)
    at_floor = float(np.mean(ns_arr <= min_window))
    near_floor = float(np.mean(ns_arr <= min_window + floor_tol_samples))
    return {"n_samples": int(ns_arr.size), "n_median": float(np.median(ns_arr)),
            "n_p10": float(np.percentile(ns_arr, 10)), "n_p90": float(np.percentile(ns_arr, 90)),
            "frac_at_floor": at_floor, "frac_near_floor": near_floor}


def finding2(chosen, v3_ekf_pipe, dubai_ekf_pipe, min_span_v3, max_window_m_v3, min_span_dubai, max_window_m_dubai):
    print("\n=== Finding 2: fraction of windows at/near the validity floors, v3 vs Dubai (both ekf_auto_pacejka) ===")
    v3_state = v3_ekf_pipe["state"]
    dubai_state = dubai_ekf_pipe["state"]
    min_window_v3 = resolve_cs_min_window_samples(v3_ekf_pipe["params"], v3_state["sample_rate_hz"])
    min_window_dubai = resolve_cs_min_window_samples(dubai_ekf_pipe["params"], dubai_state["sample_rate_hz"])
    print(f"  v3 sample_rate_hz={v3_state['sample_rate_hz']:.2f} -> min_window={min_window_v3} samples, min_span={min_span_v3} rad")
    print(f"  Dubai sample_rate_hz={dubai_state['sample_rate_hz']:.2f} -> min_window={min_window_dubai} samples, min_span={min_span_dubai} rad")

    print("\n  -- v3 (chosen 3 corners) --")
    for cid in chosen:
        for axle, cs_key, alpha_key in (("front", "CS_ratio_f", "alpha_f_filt"), ("rear", "CS_ratio_r", "alpha_r_filt")):
            r = floor_fraction_for_corner(v3_ekf_pipe, cid, cs_key, alpha_key, min_window_v3, min_span_v3, max_window_m_v3)
            if r is None:
                print(f"    C{cid} {axle}: no finite CS_ratio samples")
                continue
            print(f"    C{cid} {axle}: n={r['n_samples']} n_median={r['n_median']:.0f} (p10={r['n_p10']:.0f}, p90={r['n_p90']:.0f}) "
                  f"frac_at_floor(n<={min_window_v3})={r['frac_at_floor']:.3f} frac_near_floor(n<={min_window_v3+2})={r['frac_near_floor']:.3f}")

    print("\n  -- Dubai (Finding-3 mechanism-investigation corner set) --")
    for cid in DUBAI_MECHANISM_CORNERS:
        for axle, cs_key, alpha_key in (("front", "CS_ratio_f", "alpha_f_filt"), ("rear", "CS_ratio_r", "alpha_r_filt")):
            r = floor_fraction_for_corner(dubai_ekf_pipe, cid, cs_key, alpha_key, min_window_dubai, min_span_dubai, max_window_m_dubai)
            if r is None:
                print(f"    C{cid} {axle}: no finite CS_ratio samples")
                continue
            print(f"    C{cid} {axle}: n={r['n_samples']} n_median={r['n_median']:.0f} (p10={r['n_p10']:.0f}, p90={r['n_p90']:.0f}) "
                  f"frac_at_floor(n<={min_window_dubai})={r['frac_at_floor']:.3f} frac_near_floor(n<={min_window_dubai+2})={r['frac_near_floor']:.3f}")


# --- Finding 3: alpha signal character, matched speed class ---------------

def alpha_character_for_corner(pipe, cid, alpha_key, straight_alpha_thresh_rad=0.01):
    state, slip = pipe["state"], pipe["slip"]
    t = state["time"]
    dt = float(np.median(np.diff(t)))
    alpha = slip[alpha_key]
    dalpha_dt = np.gradient(alpha, t)

    insts_all = pipe["corners_by_stable_id"].get(cid, [])
    instances = _valid_lap_instances(insts_all, pipe["laps_by_number"])
    corner_rates = []
    for c in instances:
        lap = pipe["laps_by_number"][c["lap_number"]]
        sl = _canonical_window_slice(t, state["s_m"], lap["start_time"], lap["end_time"],
                                      c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        seg = dalpha_dt[sl]
        finite = np.isfinite(seg)
        if finite.any():
            corner_rates.append(np.abs(seg[finite]))
    corner_rates = np.concatenate(corner_rates) if corner_rates else np.array([])

    # Straight-line jitter: samples outside any corner bracket, with small
    # |alpha| (near-zero steering/slip), moving, not on a kerb -- a "clean"
    # stretch by construction rather than hand-picked.
    moving = state["moving_mask"].copy()
    kerb_mask = state.get("kerb_mask")
    if kerb_mask is not None:
        moving = moving & ~kerb_mask
    in_any_corner = np.zeros_like(moving, dtype=bool)
    for insts in pipe["corners_by_stable_id"].values():
        for c in insts:
            lap = pipe["laps_by_number"].get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, state["s_m"], lap["start_time"], lap["end_time"],
                                          c["bracket_start_m"], c["bracket_end_m"])
            in_any_corner[sl] = True
    straight_mask = moving & ~in_any_corner & (np.abs(alpha) < straight_alpha_thresh_rad)
    jitter = np.diff(alpha[straight_mask])
    jitter = jitter[np.isfinite(jitter)]

    return {
        "dt_s": dt,
        "n_corner_rate_samples": int(corner_rates.size),
        "slip_rate_median_rad_s": float(np.median(corner_rates)) if corner_rates.size else float("nan"),
        "slip_rate_p90_rad_s": float(np.percentile(corner_rates, 90)) if corner_rates.size else float("nan"),
        "n_straight_samples": int(straight_mask.sum()),
        "straight_jitter_std_rad": float(np.std(jitter)) if jitter.size else float("nan"),
    }


def finding3(v3_ekf_pipe, dubai_ekf_pipe, v3_corner_id, dubai_corner_id):
    print(f"\n=== Finding 3: alpha signal character, v3 C{v3_corner_id} vs Dubai C{dubai_corner_id} "
          f"(both 'medium' speed_class, both ekf_auto_pacejka, front axle) ===")
    r_v3 = alpha_character_for_corner(v3_ekf_pipe, v3_corner_id, "alpha_f_filt")
    r_dubai = alpha_character_for_corner(dubai_ekf_pipe, dubai_corner_id, "alpha_f_filt")
    for label, r in (("v3", r_v3), ("Dubai", r_dubai)):
        print(f"  {label}: dt={r['dt_s']*1000:.1f} ms  corner |dalpha/dt| median={np.degrees(r['slip_rate_median_rad_s']):.2f} deg/s "
              f"(p90={np.degrees(r['slip_rate_p90_rad_s']):.2f} deg/s, n={r['n_corner_rate_samples']})  "
              f"straight-line alpha jitter std={np.degrees(r['straight_jitter_std_rad']):.4f} deg/sample (n={r['n_straight_samples']})")
    return r_v3, r_dubai


# --- Finding 4: fold-vs-loop self-crossing check ---------------------------

def _segments_intersect(p1, p2, p3, p4):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = ccw(p3, p4, p1), ccw(p3, p4, p2)
    d3, d4 = ccw(p1, p2, p3), ccw(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def self_crossing_count(xs, ys):
    """Counts self-intersections of the time-ordered polyline (alpha, Fy)
    traces out. A clean monotonic 'fold' has zero self-crossings; a
    hysteresis-like 'loop'/hook crosses its own earlier path at least once.
    Adjacent segments are skipped (they always share an endpoint, not a
    genuine crossing). O(n^2), fine for single-window point counts (<100).
    """
    pts = list(zip(xs, ys))
    n = len(pts)
    count = 0
    for i in range(n - 1):
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue  # shared-endpoint adjacency at the wrap, not a crossing
            if _segments_intersect(pts[i], pts[i + 1], pts[j], pts[j + 1]):
                count += 1
    return count


def finding4(chosen_two, v3_ekf_pipe, selection_by_id, min_span, max_window_m):
    print("\n=== Finding 4: fold-vs-loop self-crossing check, worst-phase window, alpha-vs-Fy ===")
    for cid in chosen_two:
        speed_class, (rate, axle, lap_number, sl, n) = selection_by_id[cid]
        alpha_key = "alpha_f_filt" if axle == "front" else "alpha_r_filt"
        fy_key = "Fy_f_filt" if axle == "front" else "Fy_r_filt"
        cs_key = "CS_ratio_f" if axle == "front" else "CS_ratio_r"
        state, slip, forces, cs = v3_ekf_pipe["state"], v3_ekf_pipe["slip"], v3_ekf_pipe["forces"], v3_ekf_pipe["cs"]
        min_window = resolve_cs_min_window_samples(v3_ekf_pipe["params"], state["sample_rate_hz"])
        cs_ratio_seg = cs[cs_key][sl]
        finite = np.isfinite(cs_ratio_seg)
        if not finite.any():
            print(f"  C{cid}: no finite CS_ratio in worst slice -- skipped")
            continue
        worst_local = int(np.nanargmin(np.where(finite, cs_ratio_seg, np.inf)))
        worst_i = sl.start + worst_local
        start = reconstruct_cs_window_start(slip[alpha_key], worst_i, min_window, min_span, s_m=state.get("s_m"), max_window_m=max_window_m)
        window_sl = slice(start, worst_i)
        alpha_win = slip[alpha_key][window_sl]
        fy_win = forces[fy_key][window_sl]
        crossings = self_crossing_count(alpha_win, fy_win)
        shape = "LOOP/hook (self-crossing)" if crossings > 0 else "FOLD (clean, no self-crossing)"
        print(f"  C{cid} ({speed_class}) worst axle={axle} window=[{start}:{worst_i}] n={window_sl.stop - window_sl.start} "
              f"-> {crossings} self-crossing(s) -> {shape}")


# --- main -------------------------------------------------------------------

def main():
    base_params = load_parameters()
    se = base_params["stability_estimation"]

    print("--- loading v3 (ekf_auto_pacejka, production default) ---")
    v3_ekf_params = _params_with_source(base_params, "ekf_auto_pacejka")
    v3_ekf_pipe = run_pipeline(V3_FILE, v3_ekf_params)
    v3_ekf_pipe["params"] = v3_ekf_params

    chosen, selection_by_id = select_corners(v3_ekf_pipe, forced_id=13)

    print("\n--- loading v3 (kinematic, for Finding 1 beta-source comparison) ---")
    v3_kin_params = _params_with_source(base_params, "kinematic")
    v3_kin_pipe = run_pipeline(V3_FILE, v3_kin_params)
    v3_kin_pipe["params"] = v3_kin_params

    max_window_m = se["cs_max_window_m"]
    min_span = se["cs_min_slip_angle_span_rad"]
    finding1(chosen, v3_ekf_pipe, v3_kin_pipe, selection_by_id, min_span, max_window_m)

    print("\n--- loading Dubai (ekf_auto_pacejka, production default) ---")
    dubai_ekf_params = _params_with_source(base_params, "ekf_auto_pacejka")
    dubai_ekf_pipe = run_pipeline(DUBAI_FILE, dubai_ekf_params)
    dubai_ekf_pipe["params"] = dubai_ekf_params

    finding2(chosen, v3_ekf_pipe, dubai_ekf_pipe, min_span, max_window_m, min_span, max_window_m)

    medium_v3 = next((cid for cid in chosen if selection_by_id[cid][0] == "medium"), chosen[0])
    finding3(v3_ekf_pipe, dubai_ekf_pipe, medium_v3, dubai_corner_id=1)

    finding4(chosen[:2], v3_ekf_pipe, selection_by_id, min_span, max_window_m)

    print("\nDONE. See thesis_notes.md for the recorded conclusion.")


if __name__ == "__main__":
    main()
