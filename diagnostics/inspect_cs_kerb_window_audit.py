# WP-A item 1: kerb audit for Module 4b's extreme negative worst-phase
# CS ratios. Read-only diagnostic, Tier B -- no edits to modules/ or
# config/. Determines whether extreme negative CS_ratio_f/r instances are
# kerb-coincident (kerb flag inside the actual regression window), leakage-
# suspect (window mask-clean but close in time to a kerb strike -- filtfilt
# is zero-phase/acausal and can smear a kerb transient beyond the dilated
# kerb_mask band), or clean. No threshold, config, or code decision is made
# here.
#
# estimate_cornering_stiffness's window-construction loop
# (modules/stability_analysis.py, compute_cs_for_axle, ~lines 707-721) is a
# closure and not importable standalone, so it is minimally re-implemented
# below (_reconstruct_window, a line-for-line mirror). Parity with the real
# production window is not assumed -- it is checked per instance by
# recomputing the OLS slope over the reconstructed window and comparing it
# against estimate_cornering_stiffness's own returned C_window_f/r at that
# sample. A different window would generically produce a different slope on
# real, noisy measured data, so a slope match is strong evidence the window
# boundary is identical to production's. summarise_corners's private
# _phase_slice (~lines 919-929) is mirrored the same way and cross-checked
# by comparing the recomputed phase median against the value
# summarise_corners itself reports.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)

DATA_PATH = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
N_REPORT = 10
C9_WATCH_LAP = 1
C9_WATCH_STABLE_ID = 9
C9_WATCH_TARGET_CSR = -0.721
# Illustrative-only proximity bound for the plain-language summary grouping,
# not a proposed detection threshold: ~1/cutoff_hz at the current 2 Hz
# cs_filter_cutoff_hz, one order-of-magnitude proxy for filtfilt settling.
LEAKAGE_SUSPECT_SECONDS = 0.5


def _phase_slice(t, start_t, end_t, is_apex, apex_half_window_samples):
    # Mirrors modules/stability_analysis.py summarise_corners._phase_slice.
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if is_apex and hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window_samples)
        hi = min(len(t), centre + apex_half_window_samples + 1)
    return slice(lo, hi)


def _reconstruct_window(alpha, i, min_window, min_span):
    # Mirrors modules/stability_analysis.py estimate_cornering_stiffness.
    # compute_cs_for_axle's window-growth loop.
    start = i - min_window
    while start > 0:
        span = np.max(alpha[start:i]) - np.min(alpha[start:i])
        if span >= min_span:
            break
        start -= 1
    return start


def _distance_to_kerb_samples(kerb_mask):
    n = len(kerb_mask)
    dist = np.full(n, np.inf)
    if not kerb_mask.any():
        return dist
    last = -n - 1
    for idx in range(n):
        if kerb_mask[idx]:
            last = idx
        dist[idx] = idx - last
    last = 2 * n
    for idx in range(n - 1, -1, -1):
        if kerb_mask[idx]:
            last = idx
        dist[idx] = min(dist[idx], last - idx)
    return dist


def worst_phase_instances(summaries, stat_key):
    out = []
    for s in summaries:
        worst_val = None
        worst_phase = None
        for phase in PHASE_KEYS:
            v = s["phases"][phase][stat_key]["median"]
            if v == v and (worst_val is None or v < worst_val):
                worst_val = v
                worst_phase = phase
        if worst_val is not None:
            out.append({
                "lap": s["lap_number"], "corner": s["corner_number"],
                "stable_corner_id": s.get("stable_corner_id"),
                "phase": worst_phase, "phase_median": worst_val, "summary": s,
                "corner_obj_key": (s["lap_number"], s["corner_number"], s.get("stable_corner_id")),
            })
    out.sort(key=lambda r: r["phase_median"])
    return out


def build_instance_report(r, axle_data, corner_by_id, t, moving, kerb_mask,
                           dist_to_kerb, sr, min_window, min_span,
                           apex_half_window_samples):
    phase = r["phase"]
    c = corner_by_id.get(r["corner_obj_key"])
    if c is None:
        return {"error": "corner not found in corners list"}

    seg_start, seg_end = c["segments"][phase]
    sl = _phase_slice(t, seg_start, seg_end, phase == "apex_3", apex_half_window_samples)
    if sl.stop <= sl.start:
        return {"error": "empty phase slice"}

    cs_ratio_arr = axle_data["cs_ratio"]
    phase_idx = np.arange(sl.start, sl.stop)
    phase_moving = moving[sl]
    phase_vals = cs_ratio_arr[sl]
    phase_valid = phase_moving & ~np.isnan(phase_vals)
    if not phase_valid.any():
        return {"error": "no valid CS_ratio samples in phase"}

    local_vals = phase_vals[phase_valid]
    local_idx = phase_idx[phase_valid]

    recomputed_median = float(np.median(local_vals))
    reported_median = r["phase_median"]
    median_check_ok = abs(recomputed_median - reported_median) < 1e-9

    extreme_pos = int(np.argmin(local_vals))
    i_extreme = int(local_idx[extreme_pos])
    extreme_val = float(local_vals[extreme_pos])

    alpha_arr = axle_data["alpha"]
    Fy_arr = axle_data["Fy"]
    start = _reconstruct_window(alpha_arr, i_extreme, min_window, min_span)
    if i_extreme - start < 2:
        return {"error": "degenerate reconstructed window (< 2 samples)"}

    window_kerb_fraction = float(kerb_mask[start:i_extreme].mean())
    nearest_dist_samples = float(dist_to_kerb[start:i_extreme].min())
    nearest_dist_seconds = nearest_dist_samples / sr if np.isfinite(nearest_dist_samples) else float("inf")

    wa = alpha_arr[start:i_extreme]
    wf = Fy_arr[start:i_extreme]
    a_mean = np.mean(wa)
    f_mean = np.mean(wf)
    denom = np.sum((wa - a_mean) ** 2)
    if denom > 1e-10:
        recomputed_slope = float(np.sum((wa - a_mean) * (wf - f_mean)) / denom)
    else:
        recomputed_slope = float("nan")
    production_slope = float(axle_data["c_window"][i_extreme])
    if denom > 1e-10 and production_slope == production_slope:
        slope_parity_ok = abs(recomputed_slope - production_slope) < max(1.0, abs(production_slope)) * 1e-6
    else:
        slope_parity_ok = False

    return {
        "lap": r["lap"], "corner": r["corner"], "stable_corner_id": r["stable_corner_id"],
        "phase": phase, "axle": axle_data["name"],
        "cs_ratio_phase_median": reported_median, "median_check_ok": median_check_ok,
        "cs_ratio_extreme_sample": extreme_val, "extreme_sample_idx": i_extreme,
        "window_start_idx": start, "window_end_idx": i_extreme,
        "window_n_samples": i_extreme - start,
        "window_kerb_fraction": window_kerb_fraction,
        "nearest_kerb_distance_samples": nearest_dist_samples,
        "nearest_kerb_distance_seconds": nearest_dist_seconds,
        "slope_parity_ok": slope_parity_ok,
        "recomputed_slope": recomputed_slope, "production_slope": production_slope,
    }


def print_instance(rank, rep, session_kerb_fraction):
    if "error" in rep:
        print(f"{rank}. SKIPPED -- {rep['error']}")
        print()
        return
    print(f"{rank}. lap={rep['lap']}  corner={rep['corner']} (C{rep['stable_corner_id']})  "
          f"phase={rep['phase']}  axle={rep['axle']}")
    print(f"   phase median CS_ratio (ranking value): {rep['cs_ratio_phase_median']:.3f}"
          f"  [recomputed-median check: {'ok' if rep['median_check_ok'] else 'MISMATCH'}]")
    print(f"   most negative single sample in phase (drives the window): "
          f"{rep['cs_ratio_extreme_sample']:.3f}  at sample idx {rep['extreme_sample_idx']}")
    print(f"   reconstructed window: samples [{rep['window_start_idx']}, {rep['window_end_idx']})"
          f"  n={rep['window_n_samples']}")
    print(f"   window-body kerb fraction: {rep['window_kerb_fraction']*100:.1f}%"
          f"   (session average, moving samples: {session_kerb_fraction*100:.1f}%)")
    if np.isfinite(rep['nearest_kerb_distance_seconds']):
        print(f"   nearest kerb-flagged sample to window body: "
              f"{rep['nearest_kerb_distance_samples']:.0f} samples "
              f"/ {rep['nearest_kerb_distance_seconds']:.3f} s")
    else:
        print(f"   nearest kerb-flagged sample to window body: none in session")
    print(f"   window-reconstruction parity check: {'OK' if rep['slope_parity_ok'] else 'MISMATCH'}"
          f"  (recomputed slope={rep['recomputed_slope']:.0f} N/rad, "
          f"production C_window={rep['production_slope']:.0f} N/rad)")
    print()


def classify(rep):
    if "error" in rep:
        return "skipped"
    if rep["window_kerb_fraction"] > 0.0:
        return "kerb-coincident"
    if rep["nearest_kerb_distance_seconds"] < LEAKAGE_SUSPECT_SECONDS:
        return "leakage-suspect"
    return "clean"


def summarise_axle(name, reports):
    counts = {"kerb-coincident": 0, "leakage-suspect": 0, "clean": 0, "skipped": 0}
    for rep in reports:
        counts[classify(rep)] += 1
    total = len(reports)
    print(f"{name.upper()} summary ({total} instances reviewed): "
          f"{counts['kerb-coincident']} kerb-coincident, "
          f"{counts['leakage-suspect']} leakage-suspect (mask-clean but within "
          f"{LEAKAGE_SUSPECT_SECONDS:.1f}s of a kerb flag), "
          f"{counts['clean']} clean, {counts['skipped']} skipped.")
    print()


data = parse_csv(DATA_PATH)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
if not state:
    print("State preparation failed - check required channels")
    raise SystemExit

beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
corners = data.get("corners", [])
summaries = summarise_corners(corners, cs, stab, state)

t = state["time"]
sr = state["sample_rate_hz"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
if kerb_mask is None:
    print("state['kerb_mask'] is None (az channel missing or filter rejected it) "
          "-- kerb audit cannot run.")
    raise SystemExit
moving = moving_raw & ~kerb_mask

se = params["stability_estimation"]
min_span = se["cs_min_slip_angle_span_rad"]
min_window = se["cs_min_window_samples"]
apex_half_window_samples = se["apex_half_window_samples"]

session_kerb_fraction = float(kerb_mask[moving_raw].sum() / moving_raw.sum())
dist_to_kerb = _distance_to_kerb_samples(kerb_mask)

corner_by_id = {}
for c in corners:
    # keyed on (lap, corner_number, stable_corner_id): corner_number alone is
    # not unique -- multiple corners on the same lap can carry
    # corner_number=None (canonical/boundary-resolved corners), so
    # (lap, corner_number) collides. Verified: (lap, stable_corner_id) IS
    # unique across this session's corners list.
    corner_by_id[(c["lap_number"], c["corner_number"], c.get("stable_corner_id"))] = c

axles = {
    "front": {"name": "front", "alpha": slip["alpha_f_filt"], "Fy": forces["Fy_f_filt"],
              "cs_ratio": cs["CS_ratio_f"], "c_window": cs["C_window_f"], "stat_key": "cs_ratio_f"},
    "rear": {"name": "rear", "alpha": slip["alpha_r_filt"], "Fy": forces["Fy_r_filt"],
             "cs_ratio": cs["CS_ratio_r"], "c_window": cs["C_window_r"], "stat_key": "cs_ratio_r"},
}

print("=" * 78)
print(f"CS kerb-window audit -- {DATA_PATH}")
print(f"Samples: {len(t)} @ {sr:.1f} Hz.  Moving: {moving_raw.sum()}.  "
      f"cs_min_window_samples={min_window}  cs_min_slip_angle_span_rad={min_span}")
print(f"Session-average kerb fraction (moving samples): {session_kerb_fraction*100:.2f}%")
print("=" * 78)
print()

all_reports = {"front": [], "rear": []}

for axle_name, axle_data in axles.items():
    worst = worst_phase_instances(summaries, axle_data["stat_key"])
    top_n = worst[:N_REPORT]

    print("-" * 78)
    print(f"{axle_name.upper()} AXLE -- {N_REPORT} most negative worst-phase "
          f"CS_ratio_{axle_data['stat_key'][-1]} instances")
    print("-" * 78)
    for rank, r in enumerate(top_n, 1):
        rep = build_instance_report(r, axle_data, corner_by_id, t, moving, kerb_mask,
                                     dist_to_kerb, sr, min_window, min_span,
                                     apex_half_window_samples)
        print_instance(rank, rep, session_kerb_fraction)
        all_reports[axle_name].append(rep)

# --- C9 lap-1 watch item (rear axle) ---
print("-" * 78)
print(f"WATCH ITEM -- C{C9_WATCH_STABLE_ID} lap {C9_WATCH_LAP}, rear axle "
      f"(expected CS_ratio_r around {C9_WATCH_TARGET_CSR})")
print("-" * 78)
rear_worst = worst_phase_instances(summaries, "cs_ratio_r")
c9_candidates = [r for r in rear_worst
                 if r["stable_corner_id"] == C9_WATCH_STABLE_ID and r["lap"] == C9_WATCH_LAP]
if not c9_candidates:
    print(f"No corner-lap match found for stable_corner_id={C9_WATCH_STABLE_ID}, "
          f"lap={C9_WATCH_LAP} with a valid rear worst-phase value.")
    print()
else:
    r = c9_candidates[0]
    print(f"Found worst-phase CS_ratio_r = {r['phase_median']:.3f} at phase={r['phase']} "
          f"(target was {C9_WATCH_TARGET_CSR}).")
    rep = build_instance_report(r, axles["rear"], corner_by_id, t, moving, kerb_mask,
                                 dist_to_kerb, sr, min_window, min_span,
                                 apex_half_window_samples)
    print_instance("C9", rep, session_kerb_fraction)
    if rep.get("error") is None:
        all_reports["rear"].append(rep)

# --- window-reconstruction verification ---
verified = [rep for reps in all_reports.values() for rep in reps
            if "error" not in rep]
n_ok = sum(1 for rep in verified if rep["slope_parity_ok"])
print("=" * 78)
print("WINDOW-RECONSTRUCTION VERIFICATION")
print("=" * 78)
print(f"Slope parity check (reconstructed window slope vs production C_window_f/r "
      f"at the same sample index): {n_ok}/{len(verified)} instances matched within "
      f"1e-6 relative tolerance.")
if n_ok < len(verified):
    print("Mismatched instances:")
    for rep in verified:
        if not rep["slope_parity_ok"]:
            print(f"  lap={rep['lap']} corner={rep['corner']} phase={rep['phase']} "
                  f"axle={rep['axle']}: recomputed={rep['recomputed_slope']:.0f}, "
                  f"production={rep['production_slope']:.0f}")
print()

# --- plain-language summaries ---
print("=" * 78)
print("PLAIN-LANGUAGE SUMMARY (facts only -- no threshold/code decision)")
print("=" * 78)
summarise_axle("front", all_reports["front"])
summarise_axle("rear", all_reports["rear"])
