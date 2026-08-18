# WP: C9 negative-CS decomposition. Read-only diagnostic, Tier B -- no
# edits to modules/ or config/. Follow-up to the CS credibility bundle
# (items 1/2) and the tyre-curve plot (item 3), which visually showed C9's
# rear slip angle sweeping through zero to positive while Fy_r stays
# strongly negative, plus a nonzero-Fy-at-zero-slip offset on both axles.
# Two separate questions, kept separate throughout:
#   1. Are C9's negative-CS_ratio_r windows driven by sign-inconsistent
#      samples (alpha and Fy disagreeing in sign -- not a physically
#      sane single-tyre-curve relation) and do they sit near the C9/C10
#      canonical-bracket partition boundary?
#   2. Is the zero-slip Fy offset a global signal-chain artifact (present
#      everywhere) or concentrated in specific corners (C9 in particular)?
#
# Methodology note: for each lap instance in question, the "negative-CS
# instance" is the single most negative per-sample CS_ratio_r within that
# lap's C9 canonical bracket (bracket_start_m/end_m, no approach/coast-out
# margin), and its regression window is reconstructed the same way as
# WP-A item 1's kerb audit (diagnostics/inspect_cs_kerb_window_audit.py) --
# a minimal, line-for-line mirror of estimate_cornering_stiffness's window-
# growth loop (modules/stability_analysis.py, compute_cs_for_axle), since
# that loop is a closure and not importable standalone. Not re-verified
# against production C_window here (already verified in item 1 on this
# same production code); reused as established.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)

DATA_PATH = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
C9_STABLE_ID = 9
C9_LAPS_TO_CHECK = [1, 2, 4]
NEAR_ZERO_SLIP_DEG = 0.2


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Minimal reimplementation of ui/views/corner_trace_dialog.py's
    # _extend_slice_with_margin with margin_before_m=margin_after_m=0.0 --
    # the corner's own canonical bracket, clamped to this lap's own s_m
    # extent, no approach/coast-out context.
    #
    # lap_s_lo/lap_s_hi MUST be computed via min()/max(), not lap_s's first/
    # last FINITE index -- verified against real data (laps 2 and 4 here)
    # that searchsorted(t, lap_end_t, side="right") can include one sample
    # already reset for the NEXT lap (s collapses to ~0.5 m) as the final
    # element of [lo:hi). Using lap_s[-1] as "this lap's max s" then clamps
    # target_end_s down to that ~0.5 m value, producing a false-empty slice
    # for any bracket that isn't itself near s=0 -- this is exactly the
    # failure mode ui/views/corner_trace_dialog.py's _lap_slice trims
    # against with its own reset-guard; min()/max() sidesteps it without
    # reproducing that trim, since the one out-of-order trailing sample
    # doesn't corrupt a plain min/max the way it would an ordering
    # assumption. searchsorted itself still works correctly against the
    # (mostly sorted, one trailing outlier) array for any target inside the
    # sorted prefix -- confirmed against the same real laps.
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    lap_s_lo = float(np.min(lap_s[finite]))
    lap_s_hi = float(np.max(lap_s[finite]))
    target_start_s = max(lap_s_lo, bracket_start_m)
    target_end_s = min(lap_s_hi, bracket_end_m)
    start_local = int(np.searchsorted(lap_s, target_start_s, side="left"))
    end_local = int(np.searchsorted(lap_s, target_end_s, side="right"))
    return slice(lo + start_local, lo + end_local)


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
s_m = state.get("s_m")
sr = state["sample_rate_hz"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

se = params["stability_estimation"]
min_span = se["cs_min_slip_angle_span_rad"]
min_window = se["cs_min_window_samples"]

laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
corner_by_lap_id = {(c["lap_number"], c.get("stable_corner_id")): c for c in corners}
summary_by_lap_id = {(s["lap_number"], s["stable_corner_id"]): s for s in summaries}

alpha_r = slip["alpha_r_filt"]
Fy_r = forces["Fy_r_filt"]
cs_ratio_r = cs["CS_ratio_r"]


# --- Requirement 1: per-lap C9 rear negative-CS window decomposition ---

print("=" * 78)
print(f"REQUIREMENT 1 -- C9 (stable_corner_id={C9_STABLE_ID}) rear negative-CS "
      f"window decomposition, laps {C9_LAPS_TO_CHECK}")
print("=" * 78)
print()

c9_summary_any = next((s for s in summaries if s["stable_corner_id"] == C9_STABLE_ID), None)
c9_bracket_start = c9_summary_any.get("bracket_start_m") if c9_summary_any else None
c9_bracket_end = c9_summary_any.get("bracket_end_m") if c9_summary_any else None
c10_summary_any = next((s for s in summaries if s["stable_corner_id"] == 10), None)
c10_bracket_start = c10_summary_any.get("bracket_start_m") if c10_summary_any else None

print(f"C9 canonical bracket: [{c9_bracket_start:.1f}, {c9_bracket_end:.1f}] m")
if c10_bracket_start is not None:
    print(f"C10 canonical bracket start: {c10_bracket_start:.1f} m "
          f"(gap to C9 bracket end: {c10_bracket_start - c9_bracket_end:+.1f} m)")
print()

req1_results = []  # per-lap dicts, for the aggregate at the end

for lap_no in C9_LAPS_TO_CHECK:
    lap = laps_by_number.get(lap_no)
    corner = corner_by_lap_id.get((lap_no, C9_STABLE_ID))
    if lap is None or corner is None or c9_bracket_start is None:
        print(f"lap {lap_no}: no C9 instance found -- skipped.")
        print()
        continue

    sl = _canonical_window_slice(
        t, s_m, lap["start_time"], lap["end_time"], c9_bracket_start, c9_bracket_end,
    )
    if sl.stop <= sl.start:
        print(f"lap {lap_no}: empty canonical-window slice -- skipped.")
        print()
        continue

    ratios = cs_ratio_r[sl]
    moving_slice = moving[sl]
    valid = moving_slice & ~np.isnan(ratios)
    if not valid.any():
        print(f"lap {lap_no}: no valid CS_ratio_r samples in the canonical window.")
        print()
        continue

    local_idx = np.arange(sl.start, sl.stop)[valid]
    local_vals = ratios[valid]
    worst_pos = int(np.argmin(local_vals))
    i_worst = int(local_idx[worst_pos])
    worst_val = float(local_vals[worst_pos])

    if worst_val >= 0:
        print(f"lap {lap_no}: NOT APPLICABLE -- no negative CS_ratio_r sample anywhere "
              f"in C9's canonical window at production (2 Hz) cutoff "
              f"(most negative sample: {worst_val:.3f}).")
        print()
        continue

    start = _reconstruct_window(alpha_r, i_worst, min_window, min_span)
    if i_worst - start < 2:
        print(f"lap {lap_no}: degenerate window at the most negative sample -- skipped.")
        print()
        continue

    w_alpha = alpha_r[start:i_worst]
    w_Fy = Fy_r[start:i_worst]
    sign_mismatch = np.sign(w_alpha) != np.sign(w_Fy)
    mismatch_fraction = float(sign_mismatch.mean())

    window_start_s = float(s_m[start])
    window_end_s = float(s_m[i_worst - 1])

    seg_start_t, seg_end_t = corner["segments"]["exit_4"]
    if seg_end_t > seg_start_t:
        exit4_start_s = float(np.interp(seg_start_t, t, s_m))
        exit4_end_s = float(np.interp(seg_end_t, t, s_m))
    else:
        exit4_start_s, exit4_end_s = None, None

    print(f"lap {lap_no}: most negative CS_ratio_r = {worst_val:.3f} "
          f"at sample idx {i_worst}")
    print(f"  regression window: samples [{start}, {i_worst})  n={i_worst - start}  "
          f"s=[{window_start_s:.1f}, {window_end_s:.1f}] m")
    print(f"  sign(alpha_r) != sign(Fy_r) fraction in window: {mismatch_fraction*100:.1f}%")
    print(f"  alpha_r range in window: [{np.degrees(w_alpha.min()):.3f}, "
          f"{np.degrees(w_alpha.max()):.3f}] deg")
    print(f"  Fy_r range in window: [{w_Fy.min():.0f}, {w_Fy.max():.0f}] N")
    print(f"  C9 canonical bracket: [{c9_bracket_start:.1f}, {c9_bracket_end:.1f}] m -- "
          f"window end vs bracket end: {window_end_s - c9_bracket_end:+.1f} m")
    if exit4_start_s is not None:
        print(f"  exit_4 phase bounds (this lap): [{exit4_start_s:.1f}, {exit4_end_s:.1f}] m -- "
              f"window end vs exit_4 end: {window_end_s - exit4_end_s:+.1f} m")
    else:
        print(f"  exit_4 phase bounds (this lap): degenerate (zero-length)")
    if c10_bracket_start is not None:
        print(f"  window end vs C10 bracket start: {window_end_s - c10_bracket_start:+.1f} m")
    print()

    req1_results.append({
        "lap": lap_no,
        "mismatch_fraction": mismatch_fraction,
        "window_end_vs_bracket_end_m": window_end_s - c9_bracket_end,
        "window_end_vs_exit4_end_m": (window_end_s - exit4_end_s) if exit4_start_s is not None else None,
    })


# --- Requirement 2: zero-slip Fy offset across all 14 stable corners ---

print("=" * 78)
print("REQUIREMENT 2 -- zero-slip (|alpha| < 0.2 deg) Fy offset, all stable corners")
print("=" * 78)
print()

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)
alpha_f = slip["alpha_f_filt"]
Fy_f = forces["Fy_f_filt"]

stable_ids = sorted({s["stable_corner_id"] for s in summaries if s["stable_corner_id"] is not None})

pooled_f_all = []
pooled_r_all = []
medians_by_corner = {}  # cid -> (median_f, median_r)

for cid in stable_ids:
    corner_summaries = [s for s in summaries if s["stable_corner_id"] == cid]
    bracket_start = corner_summaries[0].get("bracket_start_m")
    bracket_end = corner_summaries[0].get("bracket_end_m")
    if bracket_start is None or bracket_end is None:
        print(f"C{cid}: no canonical bracket recorded -- skipped.")
        continue

    pooled_f = []
    pooled_r = []
    pooled_ay = []
    for s in corner_summaries:
        lap = laps_by_number.get(s["lap_number"])
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        sl = _canonical_window_slice(
            t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end,
        )
        if sl.stop <= sl.start:
            continue
        m = moving[sl]

        af = alpha_f[sl]
        valid_f = m & np.isfinite(af) & (np.abs(af) < near_zero_rad)
        if valid_f.any():
            pooled_f.append(Fy_f[sl][valid_f])

        ar = alpha_r[sl]
        valid_r = m & np.isfinite(ar) & (np.abs(ar) < near_zero_rad)
        if valid_r.any():
            pooled_r.append(Fy_r[sl][valid_r])

        # Turn-direction check (extension): ay over the WHOLE canonical
        # window (not the near-zero-slip subset -- direction is a property
        # of the corner as a whole, not of its straightest instant).
        ay_win = state["ay_mps2"][sl]
        valid_ay = m & np.isfinite(ay_win)
        if valid_ay.any():
            pooled_ay.append(ay_win[valid_ay])

    n_f = sum(len(a) for a in pooled_f)
    n_r = sum(len(a) for a in pooled_r)
    med_f = float(np.median(np.concatenate(pooled_f))) if pooled_f else float("nan")
    med_r = float(np.median(np.concatenate(pooled_r))) if pooled_r else float("nan")
    med_ay = float(np.median(np.concatenate(pooled_ay))) if pooled_ay else float("nan")
    if pooled_f:
        pooled_f_all.append(np.concatenate(pooled_f))
    if pooled_r:
        pooled_r_all.append(np.concatenate(pooled_r))

    print(f"C{cid}: n_near_zero_f={n_f:4d}  median Fy_f={med_f:8.0f} N   "
          f"n_near_zero_r={n_r:4d}  median Fy_r={med_r:8.0f} N")
    medians_by_corner[cid] = (med_f, med_r, med_ay)

print()
if pooled_f_all:
    global_med_f = float(np.median(np.concatenate(pooled_f_all)))
    print(f"GLOBAL median Fy_f over all corners' near-zero-slip samples: {global_med_f:.0f} N")
if pooled_r_all:
    global_med_r = float(np.median(np.concatenate(pooled_r_all)))
    print(f"GLOBAL median Fy_r over all corners' near-zero-slip samples: {global_med_r:.0f} N")

# Rank C9 by |median Fy| among all corners with a valid median -- descriptive
# only, not a threshold: shows whether C9 stands out or sits mid-pack.
print()
for axle, idx in (("front", 0), ("rear", 1)):
    ranked = sorted(
        ((cid, abs(m[idx])) for cid, m in medians_by_corner.items() if m[idx] == m[idx]),
        key=lambda pair: pair[1], reverse=True,
    )
    rank = next((i + 1 for i, (cid, _v) in enumerate(ranked) if cid == C9_STABLE_ID), None)
    if rank is not None:
        print(f"C9 rank by |median Fy_{axle[0]}| among {len(ranked)} corners with a "
              f"valid median: {rank} of {len(ranked)} (1 = largest magnitude)")

print()
print("=" * 78)
print("AGGREGATE ACROSS THE C9 REAR NEGATIVE-CS INSTANCES ABOVE")
print("=" * 78)
if req1_results:
    max_mismatch = max(r["mismatch_fraction"] for r in req1_results)
    any_mismatch = any(r["mismatch_fraction"] > 0 for r in req1_results)
    print(f"Instances found negative: {len(req1_results)} of {len(C9_LAPS_TO_CHECK)} laps checked "
          f"({[r['lap'] for r in req1_results]}).")
    print(f"Max sign(alpha_r)!=sign(Fy_r) fraction across those windows: {max_mismatch*100:.1f}%  "
          f"(any window with a nonzero mismatch fraction: {any_mismatch})")
    for r in req1_results:
        near_bracket = abs(r["window_end_vs_bracket_end_m"]) < 10.0
        print(f"  lap {r['lap']}: window end is "
              f"{'within 10 m of' if near_bracket else 'more than 10 m from'} "
              f"C9's own bracket end ({r['window_end_vs_bracket_end_m']:+.1f} m)")
else:
    print("No C9 rear negative-CS instances found among the checked laps.")


# --- Extension: does the zero-slip Fy offset sign track turn direction? ---
# Direction signal used: median ay_mps2 (lateral acceleration) over each
# corner's whole canonical window (all moving samples, not just the near-
# zero-slip subset above) -- ay is this codebase's own established
# "is cornering / which way" signal (dual-criterion corner detection's
# |ay| > 0.6g entry gate, thesis_notes.md), not a new convention invented
# for this check. Not cross-checked against yaw rate here.

print()
print("=" * 78)
print("EXTENSION -- zero-slip Fy offset sign vs turn direction (median ay_mps2)")
print("=" * 78)
print()

n_match_f = n_total_f = 0
n_match_r = n_total_r = 0

for cid in stable_ids:
    if cid not in medians_by_corner:
        continue
    med_f, med_r, med_ay = medians_by_corner[cid]
    if med_ay != med_ay:
        print(f"C{cid}: no valid ay samples in canonical window -- skipped.")
        continue
    # Labelled by ay sign only ("ay+"/"ay-"), not "left"/"right" -- this
    # script does not verify which physical turn direction ay's positive
    # sign corresponds to in this channel's convention, only that opposite
    # signs mean opposite turn directions.
    direction = "ay+" if med_ay > 0 else "ay-" if med_ay < 0 else "zero"
    direction_sign = np.sign(med_ay)

    match_f = "n/a"
    if med_f == med_f:
        match_f = "yes" if np.sign(med_f) == direction_sign else "no"
        n_total_f += 1
        n_match_f += (match_f == "yes")

    match_r = "n/a"
    if med_r == med_r:
        match_r = "yes" if np.sign(med_r) == direction_sign else "no"
        n_total_r += 1
        n_match_r += (match_r == "yes")

    print(f"C{cid}: direction={direction:>5} (median ay={med_ay:+.2f} m/s^2)  "
          f"Fy_f median={med_f:8.0f} N (matches: {match_f})  "
          f"Fy_r median={med_r:8.0f} N (matches: {match_r})")

print()
if n_total_f and n_total_r:
    if n_match_f == n_total_f and n_match_r == n_total_r:
        conclusion = "YES -- offset sign matches turn direction at every corner with a valid median, both axles."
    elif n_match_f == 0 and n_match_r == 0:
        conclusion = "NO -- offset sign matches turn direction at zero corners, both axles."
    else:
        conclusion = "MIXED -- offset sign matches turn direction at some corners/axles but not others."
    print(f"Front: {n_match_f}/{n_total_f} corners match.  Rear: {n_match_r}/{n_total_r} corners match.")
    print(f"CONCLUSION: {conclusion}")
else:
    print("Not enough corners with both a direction and an offset median to conclude.")
