# Tests the reference-starvation hypothesis for pass_1's noisy CS_ratio:
# estimate_cornering_stiffness (modules/stability_analysis.py) only
# refreshes C_linear_ref when an entire regression window sits inside
# cs_linear_slip_threshold_rad (window_max_abs_alpha < linear_thresh).
# If pass_1's larger |alpha| rarely satisfies that, C_linear_ref goes
# stale -- held from whichever update last qualified -- and CS_ratio is
# divided by an increasingly unrepresentative reference. Read-only, no
# config/production change.
#
# Update detection is inferred from the ALREADY-RETURNED C_linear_ref_arr
# (the held value in effect at each sample) via value-change detection,
# not by re-deriving the window condition separately -- two independent
# genuine regression slopes over shifting windows are not bit-identical
# in practice, so a change in the held value is an exact proxy for "an
# update happened here," without duplicating estimate_cornering_
# stiffness's internal windowing logic in a second, driftable copy.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, summarise_corners,
)
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
NEAR_ZERO_SLIP_DEG = 0.2  # matches WP-S4b's own near-zero-alpha_r(A) comparison population
FOCUS_CORNERS = (4, 14)

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
laps = data.get("laps", [])
corners = data.get("corners", [])

t = state["time"]
sr = state["sample_rate_hz"]
moving_raw = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving = moving_raw & ~kerb_mask if kerb_mask is not None else moving_raw

valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t >= s) & (t <= e)
base_mask = moving & racing_mask

se = params["stability_estimation"]
linear_thresh_rad = se["cs_linear_slip_threshold_rad"]
print(f"cs_linear_slip_threshold_rad (config, read directly) = {linear_thresh_rad} rad "
      f"({np.degrees(linear_thresh_rad):.3f} deg)")
print(f"sample_rate_hz = {sr}")
print()

beta_a = estimate_sideslip(state, params)
slip_a = estimate_slip_angles(state, beta_a, params)
ekf_result = estimate_sideslip_ekf_dugoff(state, params, pass_id="pass_1")
beta_c = ekf_result["beta"]
slip_c = estimate_slip_angles(state, beta_c, params)
forces = estimate_lateral_forces(state, params)
cs_a = estimate_cornering_stiffness(slip_a, forces, state, params)
cs_c = estimate_cornering_stiffness(slip_c, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta_a, params, laps)

alpha_r_a = slip_a["alpha_r_filt"]


def update_events(Clr_arr):
    """Sample indices where the held C_linear_ref value changed."""
    n = len(Clr_arr)
    events = []
    last_val = np.nan
    for i in range(n):
        v = Clr_arr[i]
        if np.isnan(v):
            continue
        if np.isnan(last_val) or v != last_val:
            events.append(i)
        last_val = v
    return np.array(events, dtype=int)


def last_update_idx_array(n, events):
    out = np.full(n, -1, dtype=int)
    ptr = 0
    last = -1
    for i in range(n):
        if ptr < len(events) and events[ptr] == i:
            last = i
            ptr += 1
        out[i] = last
    return out


paths = {"kinematic": cs_a, "pass_1": cs_c}
axles = {"front": "C_linear_ref_f", "rear": "C_linear_ref_r"}

events_cache = {}
last_idx_cache = {}
for path_name, cs in paths.items():
    for axle_name, key in axles.items():
        ev = update_events(cs[key])
        events_cache[(path_name, axle_name)] = ev
        last_idx_cache[(path_name, axle_name)] = last_update_idx_array(len(cs[key]), ev)

# --- Section 1+2: update rate and inter-update gap distribution -----------

print("=" * 100)
print("SECTION 1+2 -- update rate and inter-update gap distribution")
print("=" * 100)
n_masked = int(base_mask.sum())
print(f"masked population (moving, kerb-excluded, valid-lap racing time) n={n_masked}")
print()

for axle_name in axles:
    for path_name in paths:
        Clr = paths[path_name][axles[axle_name]]
        ev = events_cache[(path_name, axle_name)]
        updated_mask = np.zeros(len(Clr), dtype=bool)
        updated_mask[ev] = True
        n_updates_masked = int((updated_mask & base_mask).sum())
        frac = n_updates_masked / n_masked if n_masked else float("nan")
        if len(ev) >= 2:
            gaps = np.diff(ev)
            p50, p90 = np.percentile(gaps, [50, 90])
            gmax = np.max(gaps)
        else:
            p50 = p90 = gmax = float("nan")
        print(f"  {axle_name:5s} / {path_name:9s}: updates in masked pop={n_updates_masked:6d} "
              f"({frac*100:5.2f}%)   total update events (whole session)={len(ev)}")
        print(f"           gap between updates: p50={p50:8.1f} smp ({p50/sr:6.2f} s)   "
              f"p90={p90:8.1f} smp ({p90/sr:6.2f} s)   max={gmax:8.1f} smp ({gmax/sr:6.2f} s)")
    print()

# --- Section 3: value distribution -----------------------------------------

print("=" * 100)
print("SECTION 3 -- C_linear_ref value distribution")
print("=" * 100)
print("(a) global: held value at every masked sample (this is CS_ratio's actual")
print("    denominator over time, repeats included where the reference is stale)")
print()

for axle_name in axles:
    for path_name in paths:
        Clr = paths[path_name][axles[axle_name]]
        vals = Clr[base_mask]
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            print(f"  {axle_name:5s} / {path_name:9s}: no finite samples in masked population")
            continue
        p5, p25, p50, p75, p95 = np.percentile(vals, [5, 25, 50, 75, 95])
        print(f"  {axle_name:5s} / {path_name:9s}: n={len(vals):6d}  "
              f"p5={p5:9.0f}  p25={p25:9.0f}  p50={p50:9.0f}  p75={p75:9.0f}  p95={p95:9.0f}  "
              f"(p95/p5 ratio={p95/p5 if p5 else float('nan'):.2f})")
print()

print("(b) per-corner median of the held value, full canonical window, all valid laps")
print("    pooled -- generalises WP-S4b's 'corner-to-corner spread' framing to the full")
print("    sample population (not restricted to near-zero-alpha), both axles.")
print()

laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in corners:
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
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


s_m = state.get("s_m")

for axle_name in axles:
    for path_name in paths:
        Clr = paths[path_name][axles[axle_name]]
        medians = []
        for cid in stable_ids:
            instances = corners_by_stable_id[cid]
            bracket_start = instances[0].get("bracket_start_m")
            bracket_end = instances[0].get("bracket_end_m")
            if bracket_start is None or bracket_end is None:
                continue
            pooled = []
            for c in instances:
                lap = laps_by_number.get(c["lap_number"])
                if lap is None or not lap.get("is_valid_for_analysis"):
                    continue
                sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
                if sl.stop <= sl.start:
                    continue
                m = base_mask[sl]
                if not m.any():
                    continue
                pooled.append(Clr[sl][m])
            if not pooled:
                continue
            vals = np.concatenate(pooled)
            vals = vals[np.isfinite(vals)]
            if len(vals) == 0:
                continue
            medians.append((cid, float(np.median(vals))))
        if not medians:
            print(f"  {axle_name:5s} / {path_name:9s}: no corner medians computable")
            continue
        vals_only = [v for _, v in medians]
        lo_cid, lo_v = min(medians, key=lambda x: x[1])
        hi_cid, hi_v = max(medians, key=lambda x: x[1])
        print(f"  {axle_name:5s} / {path_name:9s}: {len(medians)} corners, "
              f"range {lo_v:.0f} (C{lo_cid}) - {hi_v:.0f} (C{hi_cid}) N/rad  "
              f"ratio max/min={hi_v/lo_v if lo_v else float('nan'):.2f}")
print()

print("(c) WP-S4b-comparable check, rear axle only, EXACT same near-zero-alpha_r(A)")
print(f"    sample selection (|alpha_r kinematic| < {NEAR_ZERO_SLIP_DEG} deg) WP-S4b used for its")
print("    79k-337k N/rad kinematic finding -- same samples, both paths' C_linear_ref_r read")
print("    at them, so only the reference construction differs, not the sample population.")
print()

near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)
for path_name in paths:
    Clr_r = paths[path_name]["C_linear_ref_r"]
    medians = []
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled = []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = base_mask[sl] & np.isfinite(alpha_r_a[sl]) & (np.abs(alpha_r_a[sl]) < near_zero_rad)
            if not m.any():
                continue
            pooled.append(Clr_r[sl][m])
        if not pooled:
            continue
        vals = np.concatenate(pooled)
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            continue
        medians.append((cid, float(np.median(vals)), len(vals)))
    if not medians:
        print(f"  {path_name:9s}: no corner has a near-zero-alpha_r(A) sample with a finite C_linear_ref_r")
        continue
    vals_only = [v for _, v, _ in medians]
    lo_cid, lo_v, _ = min(medians, key=lambda x: x[1])
    hi_cid, hi_v, _ = max(medians, key=lambda x: x[1])
    n_corners_covered = len(medians)
    print(f"  {path_name:9s}: {n_corners_covered}/{len(stable_ids)} corners have >=1 qualifying sample; "
          f"range {lo_v:.0f} (C{lo_cid}) - {hi_v:.0f} (C{hi_cid}) N/rad  "
          f"ratio max/min={hi_v/lo_v if lo_v else float('nan'):.2f}")
    for cid, v, n in sorted(medians):
        print(f"      C{cid}: median={v:8.0f} N/rad  n={n}")
print()

# --- Section 4: C4/C14 worst-phase-sample detail ---------------------------

print("=" * 100)
print("SECTION 4 -- C4/C14 worst-phase sample: C_linear_ref, staleness, C_alpha")
print("=" * 100)
print("Worst phase/sample identified from pass_1 (matches the flagged instances); both")
print("paths' C_linear_ref/C_alpha are then read at that SAME sample index.")
print()

apex_half_window_samples = se["apex_half_window_samples"]
phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _phase_slice(start_t, end_t, is_apex=False):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if is_apex and hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window_samples)
        hi = min(len(t), centre + apex_half_window_samples + 1)
    return slice(lo, hi)


summaries_c = summarise_corners(corners, cs_c, stab, state)
summaries_by_key = {(s["stable_corner_id"], s["lap_number"]): s for s in summaries_c}
corners_by_key = {(c.get("stable_corner_id"), c["lap_number"]): c for c in corners}

min_window = se["cs_min_window_samples"]
min_span = se["cs_min_slip_angle_span_rad"]


def _window_span_at(alpha, i):
    """Reproduces estimate_cornering_stiffness's own window-growth loop
    (modules/stability_analysis.py lines ~714-724) to report the |alpha|
    span and sample count of the regression window used AT sample i,
    without re-deriving CS_ratio itself."""
    start = i - min_window
    while start > 0:
        span = np.max(alpha[start:i]) - np.min(alpha[start:i])
        if span >= min_span:
            break
        start -= 1
    window_alpha = alpha[start:i]
    if len(window_alpha) < min_window:
        return float("nan"), len(window_alpha)
    return float(np.max(window_alpha) - np.min(window_alpha)), len(window_alpha)


slip_by_path = {"kinematic": slip_a, "pass_1": slip_c}

for cid in FOCUS_CORNERS:
    for axle_name, summary_key, CS_key, Clr_key, Ca_key, R2_key, alpha_key in (
        ("front", "cs_ratio_f", "CS_ratio_f", "C_linear_ref_f", "C_alpha_f", "R2_f", "alpha_f_filt"),
        ("rear", "cs_ratio_r", "CS_ratio_r", "C_linear_ref_r", "C_alpha_r", "R2_r", "alpha_r_filt"),
    ):
        print(f"--- C{cid} {axle_name} ---")
        for lap in (1, 2, 3, 4):
            s = summaries_by_key.get((cid, lap))
            c = corners_by_key.get((cid, lap))
            if s is None or c is None:
                print(f"  lap{lap}: no data")
                continue
            phase_meds = [(ph, s["phases"][ph][summary_key]["median"]) for ph in phase_keys
                          if s["phases"][ph][summary_key]["median"] == s["phases"][ph][summary_key]["median"]]
            if not phase_meds:
                print(f"  lap{lap}: no valid phase medians")
                continue
            worst_phase, worst_med = min(phase_meds, key=lambda pv: pv[1])

            start_t, end_t = c["segments"][worst_phase]
            sl = _phase_slice(start_t, end_t, is_apex=(worst_phase == "apex_3"))
            phase_moving = moving[sl]
            idx = np.where(phase_moving)[0] + sl.start
            if len(idx) == 0:
                print(f"  lap{lap}: worst_phase={worst_phase} but no moving samples in window")
                continue
            cs_vals_pass1 = cs_c[CS_key][idx]
            finite = np.isfinite(cs_vals_pass1)
            if not finite.any():
                print(f"  lap{lap}: worst_phase={worst_phase} but no finite pass_1 CS_ratio samples")
                continue
            idx_finite = idx[finite]
            worst_i = idx_finite[np.argmin(cs_vals_pass1[finite])]

            for path_name, cs in paths.items():
                Clr_val = cs[Clr_key][worst_i]
                Ca_val = cs[Ca_key][worst_i]
                cs_val = cs[CS_key][worst_i]
                R2_val = cs[R2_key][worst_i]
                last_idx = last_idx_cache[(path_name, axle_name)][worst_i]
                staleness = (worst_i - last_idx) if last_idx >= 0 else None
                staleness_s = staleness / sr if staleness is not None else float("nan")
                alpha_arr = slip_by_path[path_name][alpha_key]
                span_rad, n_win = _window_span_at(alpha_arr, worst_i)
                print(f"  lap{lap}: worst_phase={worst_phase:15s} t={t[worst_i]:9.3f}s  "
                      f"[{path_name:9s}] CS_ratio={cs_val:+7.3f}  C_linear_ref={Clr_val:9.0f}  "
                      f"C_alpha={Ca_val:9.0f}  stale={staleness}smp ({staleness_s:.2f}s)  "
                      f"window|alpha|span={np.degrees(span_rad):5.3f}deg (n={n_win})  R2={R2_val:.3f}")
        print()
