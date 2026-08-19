# WP-S2 (Open Board item B, sideslip methods comparison): comparison
# harness skeleton. Read-only, Tier B (comparison/QA machinery -- no new
# vehicle-dynamics claim). Nothing here changes production code, calls
# any pipeline/UI path, or writes config. Establishes the harness the
# observer candidate (WP-S4, gated on its own literature anchors) will
# plug into as a third registry entry -- no rewrite of this file expected
# when that happens, only a new CANDIDATES line.
#
# CANDIDATES THIS ROUND:
#   A: kinematic beta (estimate_sideslip) -- production baseline.
#   B: GPS-course beta (estimate_sideslip_gps) -- included ONLY as a
#      documented NEGATIVE CONTROL. This line is SHELVED (thesis_notes.md
#      "GPS-course sideslip (beta_gps, WP5b(c)) attempted and shelved",
#      2026-07-26): both construction iterations left correlation with
#      kinematic beta weak (r=-0.24) and per-corner sign agreement barely
#      above chance (52.2% as recorded in that entry). Its presence here
#      is to exercise the harness against known numbers (Metric 2's
#      sanity gate below), not because it is a live candidate. Reopen
#      condition unchanged from that write-up: a denser/multi-session
#      GPS anchor set, not attempted here.
#      NOTE: the sign-agreement figure was re-baselined 133/255=52.2% ->
#      127/257=49.4% (r unchanged) after commit 0bdff87's WP1 canonical
#      corner realization moved corner-phase boundaries; see the Metric 2
#      gate below and thesis_notes.md's dated supersession note. Verdict
#      (shelved, both chance-level) unaffected.
#   C: linear Kalman sideslip observer (WP-S4, diagnostics/sideslip_
#      kalman_observer.py, estimate_sideslip_kalman) -- diagnostics-only,
#      no production wiring. Model/method anchors in thesis_notes.md,
#      WP-S4 entry.
#
# Metrics (per candidate unless noted):
#   1. Straight-line near-zero check: median/p90 |beta|.
#   2. Cross-method correlation r + per-corner-phase sign agreement %,
#      all registry pairs (this round: just A vs B). Sanity-gated against
#      the on-record shelving numbers.
#   3. Physical-plausibility envelope: p1/p99, max |beta|, flagged
#      against a generous sane bound.
#   4. Inter-lap consistency: per stable corner, cross-lap std of
#      median beta inside the canonical bracket window.
#   5. (WP-S3) Zero-slip Fy offset + direction-match, generalised from
#      diagnostics/inspect_c9_negative_cs.py's REQUIREMENT 2 + EXTENSION
#      sections: per stable corner, median Fy_f/Fy_r over samples with
#      |alpha| < NEAR_ZERO_SLIP_DEG inside the canonical window, plus
#      turn direction (sign of median ay over the window) and
#      offset-sign-vs-direction match count. Fy_f/Fy_r (estimate_lateral_
#      forces) do not depend on beta; only which samples count as
#      near-zero-slip does, via estimate_slip_angles(state, beta, params)
#      with the candidate beta substituted. Regression-gated against
#      inspect_c9_negative_cs.py's own on-record report for the
#      production kinematic beta before any candidate comparison runs.
#      Factored inline here rather than into a shared diagnostics/
#      helper: a second consumer alone doesn't justify extraction, and
#      keeping inspect_c9_negative_cs.py untouched preserves it as this
#      metric's independent regression reference.
#
# Out of scope this WP: any observer code, any whitelist or config
# change.

import itertools
import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip, estimate_sideslip_gps,
    estimate_slip_angles, estimate_lateral_forces,
)
from diagnostics.inspect_wheel_speed_sources import AY_STRAIGHT_MAX_G, YAW_STRAIGHT_MAX_DEGPS
from diagnostics.sideslip_kalman_observer import estimate_sideslip_kalman

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

# Generous sanity envelope (Section 3) -- NOT a physical claim about this
# car's real handling limit. A GT3 car sustaining |beta| above this would
# already be well into a spin; set high enough that only a genuine
# construction/unit bug (e.g. a missed deg<->rad conversion) would trip
# it, not hard-but-controlled driving.
BETA_SANE_BOUND_DEG = 20.0

# Metric 5 -- same value as inspect_c9_negative_cs.py's NEAR_ZERO_SLIP_DEG,
# required for the regression check below to be meaningful.
NEAR_ZERO_SLIP_DEG = 0.2

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
se = params["stability_estimation"]

t_ref = state["time"]
sr = state["sample_rate_hz"]
moving = state["moving_mask"]
v_kmh = state["v_mps"] * 3.6

laps = data.get("laps", [])
valid_windows = [(l["start_time"], l["end_time"]) for l in laps if l.get("is_valid_for_analysis")]
racing_mask = np.zeros_like(t_ref, dtype=bool)
for s, e in valid_windows:
    racing_mask |= (t_ref >= s) & (t_ref <= e)

# --- Candidate registry -----------------------------------------------
# Each entry: name -> zero-arg callable returning a beta array (rad) on
# the common t_ref timebase.
CANDIDATES = {
    "A_kinematic": lambda: estimate_sideslip(state, params),
    "B_gps_course": lambda: estimate_sideslip_gps(state, data["channels"], params),
    "C_kalman_observer": lambda: estimate_sideslip_kalman(state, params),
}

betas = {name: fn() for name, fn in CANDIDATES.items()}
valid_masks = {name: moving & racing_mask & np.isfinite(b) for name, b in betas.items()}

print("=" * 78)
print("HEADER -- candidate registry")
print("=" * 78)
print("A_kinematic  : production baseline (modules.stability_analysis.estimate_sideslip)")
print("B_gps_course : NEGATIVE CONTROL ONLY -- SHELVED (thesis_notes.md 2026-07-26),")
print("               r=-0.24 / 52.2% sign agreement on both construction iterations;")
print("               included here to exercise the harness against known numbers,")
print("               not as a live candidate. Not called from any pipeline/UI path.")
print("C_kalman_observer : WP-S4 diagnostics-only candidate (diagnostics/sideslip_")
print("               kalman_observer.py), linear KF on the bicycle-model state-space")
print("               form, fixed Caf/Car prior. Not called from any pipeline/UI path.")
for name in CANDIDATES:
    print(f"  {name:14s} n_valid(moving & racing_mask & finite) = {valid_masks[name].sum()}")

print()
print("=" * 78)
print("METRIC 1 -- Straight-line near-zero check")
print("=" * 78)
ay_g = state["ay_mps2"] / 9.81
yaw_rate_degps = state["yaw_rate_radps"] * 180.0 / np.pi
print(f"Straight-line gate (reused from diagnostics/inspect_wheel_speed_sources.py): "
      f"moving & valid-lap & |ay|<={AY_STRAIGHT_MAX_G}g & |yaw rate|<={YAW_STRAIGHT_MAX_DEGPS} deg/s.")
print("Reused rather than each estimator's own internal anchor gate (e.g. beta_gps's")
print("smoothed gps_course_anchor_max_ay_g) so the harness stays candidate-agnostic --")
print("required for a third registry entry to slot in without a metric rewrite.")
straight_base = moving & racing_mask & (np.abs(ay_g) <= AY_STRAIGHT_MAX_G) & (np.abs(yaw_rate_degps) <= YAW_STRAIGHT_MAX_DEGPS)
for name, b in betas.items():
    m = straight_base & valid_masks[name]
    b_deg = np.degrees(np.abs(b[m]))
    print(f"  {name:14s} n={m.sum():6d}  median|beta|={np.median(b_deg):.3f} deg  p90|beta|={np.percentile(b_deg, 90):.3f} deg")

print()
print("=" * 78)
print("METRIC 2 -- Cross-method correlation + per-corner-phase sign agreement")
print("=" * 78)
phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
apex_half_window = se["apex_half_window_samples"]


def _phase_slice(start_t, end_t, is_apex=False):
    if end_t < start_t:
        return slice(0, 0)
    lo = int(np.searchsorted(t_ref, start_t, side="left"))
    hi = int(np.searchsorted(t_ref, end_t, side="right"))
    if is_apex and hi <= lo:
        lo = max(0, lo - apex_half_window)
        hi = min(len(t_ref), lo + 2 * apex_half_window + 1)
    return slice(lo, hi)


pair_results = {}
for name_x, name_y in itertools.combinations(CANDIDATES, 2):
    bx_deg, by_deg = np.degrees(betas[name_x]), np.degrees(betas[name_y])
    core_mask = valid_masks[name_x] & valid_masks[name_y]

    r = np.corrcoef(bx_deg[core_mask], by_deg[core_mask])[0, 1]

    sign_matches, sign_total = 0, 0
    for c in data.get("corners", []):
        lap = next((l for l in laps if l["lap_number"] == c["lap_number"]), None)
        if lap is None or not lap.get("is_valid_for_analysis"):
            continue
        for phase in phase_keys:
            s_t, e_t = c["segments"][phase]
            sl = _phase_slice(s_t, e_t, is_apex=(phase == "apex_3"))
            m = core_mask[sl]
            if m.sum() == 0:
                continue
            x_med = np.median(bx_deg[sl][m])
            y_med = np.median(by_deg[sl][m])
            sign_total += 1
            if np.sign(x_med) == np.sign(y_med):
                sign_matches += 1

    pair_results[(name_x, name_y)] = (r, sign_matches, sign_total)
    print(f"  {name_x} vs {name_y}: n={core_mask.sum()}  r={r:+.4f}  "
          f"sign agreement {sign_matches}/{sign_total} = {sign_matches/sign_total*100:.1f}%")

print()
print("Sanity gate (this pair must reproduce the on-record shelving numbers, RE-BASELINED")
print("thesis_notes.md 2026-07-26 entry: r=-0.24 unchanged, sign agreement 127/257=49.4%):")
print("Original 133/255=52.2% was frozen before commit 0bdff87 ('lap segmentation big")
print("improve') landed WP1's canonical corner realization, which moved corner-phase")
print("boundaries; re-running the unchanged original inspect_beta_gps_validation.py")
print("now also prints 127/257 -- the harness is correct, the old baseline was stale.")
gate_key = ("A_kinematic", "B_gps_course")
if gate_key in pair_results:
    r_gate, sm_gate, st_gate = pair_results[gate_key]
    r_ok = abs(r_gate - (-0.24)) < 0.005
    sign_pct = sm_gate / st_gate * 100
    sign_ok = abs(sign_pct - 49.4) < 0.5 and sm_gate == 127 and st_gate == 257
    print(f"  r={r_gate:+.4f} (target -0.24): {'PASS' if r_ok else 'FAIL'}")
    print(f"  sign agreement {sm_gate}/{st_gate}={sign_pct:.1f}% (target 127/257=49.4%): {'PASS' if sign_ok else 'FAIL'}")
    if not (r_ok and sign_ok):
        print("  *** SANITY GATE FAILED -- harness is wired differently from the on-record run.")
        print("  *** Stopping before proceeding further; see the discrepancy above.")
        raise SystemExit(1)
    print("  SANITY GATE PASSED -- harness reproduces the on-record numbers.")
else:
    print("  A_kinematic/B_gps_course pair not found in registry -- gate skipped.")

print()
print("=" * 78)
print("METRIC 3 -- Physical-plausibility envelope")
print("=" * 78)
print(f"Sane bound (generous, diagnostic-only, not a physical claim): |beta| <= {BETA_SANE_BOUND_DEG} deg")
for name, b in betas.items():
    m = valid_masks[name]
    b_deg = np.degrees(b[m])
    p1, p99 = np.percentile(b_deg, [1, 99])
    max_abs = np.max(np.abs(b_deg))
    flag = "FLAGGED (exceeds sane bound)" if max_abs > BETA_SANE_BOUND_DEG else "within bound"
    print(f"  {name:14s} n={m.sum():6d}  p1={p1:+.3f} deg  p99={p99:+.3f} deg  max|beta|={max_abs:.3f} deg  -> {flag}")

print()
print("=" * 78)
print("METRIC 4 -- Inter-lap consistency (cross-lap std of median beta per stable corner)")
print("=" * 78)
by_stable_id = {}
for c in data.get("corners", []):
    lap = next((l for l in laps if l["lap_number"] == c["lap_number"]), None)
    if lap is None or not lap.get("is_valid_for_analysis"):
        continue
    sid = c.get("stable_corner_id")
    if sid is None:
        continue
    by_stable_id.setdefault(sid, []).append(c)

for name, b in betas.items():
    b_deg = np.degrees(b)
    m_valid = valid_masks[name]
    print(f"--- {name} ---")
    stds = []
    for sid, instances in sorted(by_stable_id.items()):
        if len(instances) < 2:
            continue
        lap_medians = []
        for c in instances:
            s_t = c["segments"]["entry_1_brake"][0]
            e_t = c["segments"]["exit_5"][1]
            sl = _phase_slice(s_t, e_t)
            m = m_valid[sl]
            if m.sum() == 0:
                continue
            lap_medians.append(np.median(b_deg[sl][m]))
        if len(lap_medians) < 2:
            continue
        std_here = float(np.std(lap_medians))
        stds.append(std_here)
        print(f"  stable_corner_id={sid}  n_laps={len(lap_medians)}  "
              f"lap medians={['%+.3f' % v for v in lap_medians]}  cross-lap std={std_here:.3f} deg")
    if stds:
        print(f"  Summary: {len(stds)} stable corners with >=2 laps -- "
              f"median cross-lap std={np.median(stds):.3f} deg, mean={np.mean(stds):.3f} deg, max={np.max(stds):.3f} deg")
    else:
        print("  No stable corner had >=2 valid-lap instances with samples.")

print()
print("=" * 78)
print("METRIC 5 -- Zero-slip Fy offset + direction-match (WP-S3)")
print("=" * 78)
print("Fy_f/Fy_r (estimate_lateral_forces) do not depend on beta -- computed once,")
print("shared across candidates. Only which samples count as near-zero-slip depends")
print("on the candidate beta, via estimate_slip_angles(state, beta, params).")

s_m = state.get("s_m")
kerb_mask = state.get("kerb_mask")
# Matches inspect_c9_negative_cs.py's own moving mask exactly (kerb-excluded) --
# distinct from the top-level `moving` used by Metrics 1-4, which does not
# exclude kerb samples. Required for the regression check below to hold.
moving_no_kerb = moving & ~kerb_mask if kerb_mask is not None else moving
laps_by_number = {l["lap_number"]: l for l in laps}
corners_by_stable_id = {}
for c in data.get("corners", []):
    sid = c.get("stable_corner_id")
    if sid is not None:
        corners_by_stable_id.setdefault(sid, []).append(c)
stable_ids = sorted(corners_by_stable_id)
forces = estimate_lateral_forces(state, params)


def _canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Identical to inspect_c9_negative_cs.py's own helper of the same name:
    # a minimal reimplementation of ui/views/corner_trace_dialog.py's
    # _extend_slice_with_margin with zero margin. min()/max() for the lap's
    # own s extent, not first/last-finite-index -- a lap-boundary reset
    # sample can trail into [lo:hi) with s collapsed to ~0, verified there
    # against real laps 2 and 4.
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


def _zero_slip_fy_offset(beta):
    slip = estimate_slip_angles(state, beta, params)
    alpha_f, alpha_r = slip["alpha_f_filt"], slip["alpha_r_filt"]
    Fy_f, Fy_r = forces["Fy_f_filt"], forces["Fy_r_filt"]
    near_zero_rad = np.radians(NEAR_ZERO_SLIP_DEG)

    per_corner = {}
    pooled_f_all, pooled_r_all = [], []
    for cid in stable_ids:
        instances = corners_by_stable_id[cid]
        bracket_start = instances[0].get("bracket_start_m")
        bracket_end = instances[0].get("bracket_end_m")
        if bracket_start is None or bracket_end is None:
            continue
        pooled_f, pooled_r, pooled_ay = [], [], []
        for c in instances:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t_ref, s_m, lap["start_time"], lap["end_time"], bracket_start, bracket_end)
            if sl.stop <= sl.start:
                continue
            m = moving_no_kerb[sl]

            af = alpha_f[sl]
            valid_f = m & np.isfinite(af) & (np.abs(af) < near_zero_rad)
            if valid_f.any():
                pooled_f.append(Fy_f[sl][valid_f])

            ar = alpha_r[sl]
            valid_r = m & np.isfinite(ar) & (np.abs(ar) < near_zero_rad)
            if valid_r.any():
                pooled_r.append(Fy_r[sl][valid_r])

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
        per_corner[cid] = {"n_f": n_f, "med_f": med_f, "n_r": n_r, "med_r": med_r, "med_ay": med_ay}

    global_med_f = float(np.median(np.concatenate(pooled_f_all))) if pooled_f_all else float("nan")
    global_med_r = float(np.median(np.concatenate(pooled_r_all))) if pooled_r_all else float("nan")

    n_match_f = n_total_f = n_match_r = n_total_r = 0
    for d in per_corner.values():
        med_ay = d["med_ay"]
        if med_ay != med_ay:
            continue
        dir_sign = np.sign(med_ay)
        if d["med_f"] == d["med_f"]:
            n_total_f += 1
            n_match_f += int(np.sign(d["med_f"]) == dir_sign)
        if d["med_r"] == d["med_r"]:
            n_total_r += 1
            n_match_r += int(np.sign(d["med_r"]) == dir_sign)

    return {
        "per_corner": per_corner,
        "global_med_f": global_med_f,
        "global_med_r": global_med_r,
        "match_f": (n_match_f, n_total_f),
        "match_r": (n_match_r, n_total_r),
    }


def _print_metric5(name, result):
    print()
    print(f"--- {name} ---")
    for cid in stable_ids:
        d = result["per_corner"].get(cid)
        if d is None:
            print(f"  C{cid}: no canonical bracket recorded -- skipped.")
            continue
        print(f"  C{cid}: n_near_zero_f={d['n_f']:4d}  median Fy_f={d['med_f']:8.0f} N   "
              f"n_near_zero_r={d['n_r']:4d}  median Fy_r={d['med_r']:8.0f} N")
    print(f"  GLOBAL median Fy_f={result['global_med_f']:.0f} N   "
          f"GLOBAL median Fy_r={result['global_med_r']:.0f} N")
    mf, tf = result["match_f"]
    mr, tr = result["match_r"]
    print(f"  Direction match: front {mf}/{tf}   rear {mr}/{tr}")


metric5_results = {"A_kinematic": _zero_slip_fy_offset(betas["A_kinematic"])}
_print_metric5("A_kinematic", metric5_results["A_kinematic"])

# --- Regression check: production kinematic beta must reproduce the
# on-record inspect_c9_negative_cs.py REQUIREMENT 2 + EXTENSION report
# (captured from that script's own output on this same sample file,
# 2026-08-18) before any other candidate below is trusted. ---
METRIC5_REGRESSION_REF = {
    1:  {"n_f": 26,  "med_f": -6948, "n_r": 24,  "med_r": -11038},
    2:  {"n_f": 14,  "med_f": 3662,  "n_r": 32,  "med_r": 7249},
    3:  {"n_f": 16,  "med_f": -5974, "n_r": 58,  "med_r": -10469},
    4:  {"n_f": 6,   "med_f": 1815,  "n_r": 3,   "med_r": 3255},
    5:  {"n_f": 64,  "med_f": -5851, "n_r": 82,  "med_r": -8729},
    6:  {"n_f": 108, "med_f": 5611,  "n_r": 541, "med_r": 9557},
    7:  {"n_f": 87,  "med_f": -5011, "n_r": 70,  "med_r": -7912},
    8:  {"n_f": 71,  "med_f": 4975,  "n_r": 94,  "med_r": 8566},
    9:  {"n_f": 75,  "med_f": -5120, "n_r": 132, "med_r": -8592},
    10: {"n_f": 129, "med_f": -2623, "n_r": 210, "med_r": -3860},
    11: {"n_f": 0,   "med_f": float("nan"), "n_r": 463, "med_r": 7210},
    12: {"n_f": 16,  "med_f": 4133,  "n_r": 91,  "med_r": 9286},
    13: {"n_f": 12,  "med_f": -5742, "n_r": 68,  "med_r": -10289},
    14: {"n_f": 81,  "med_f": -5556, "n_r": 97,  "med_r": -8983},
}
METRIC5_REGRESSION_GLOBAL = {"med_f": -3801, "med_r": 6197}
METRIC5_REGRESSION_MATCH = {"f": (13, 13), "r": (14, 14)}

print()
print("Regression check (production kinematic beta must reproduce the on-record")
print("inspect_c9_negative_cs.py REQUIREMENT 2 + EXTENSION report, within rounding):")
ref = metric5_results["A_kinematic"]
mismatches = []
for cid, expected in METRIC5_REGRESSION_REF.items():
    got = ref["per_corner"].get(cid)
    if got is None:
        mismatches.append(f"C{cid}: missing from harness output")
        continue
    if got["n_f"] != expected["n_f"]:
        mismatches.append(f"C{cid} n_near_zero_f: got {got['n_f']}, expected {expected['n_f']}")
    if got["n_r"] != expected["n_r"]:
        mismatches.append(f"C{cid} n_near_zero_r: got {got['n_r']}, expected {expected['n_r']}")
    for key in ("med_f", "med_r"):
        g, e = got[key], expected[key]
        g_nan, e_nan = (g != g), (e != e)
        if g_nan != e_nan:
            mismatches.append(f"C{cid} {key}: got {g}, expected {e}")
        elif not g_nan and abs(g - e) > 0.5:
            mismatches.append(f"C{cid} {key}: got {g:.0f}, expected {e:.0f}")
if abs(ref["global_med_f"] - METRIC5_REGRESSION_GLOBAL["med_f"]) > 0.5:
    mismatches.append(f"global med_f: got {ref['global_med_f']:.0f}, expected {METRIC5_REGRESSION_GLOBAL['med_f']}")
if abs(ref["global_med_r"] - METRIC5_REGRESSION_GLOBAL["med_r"]) > 0.5:
    mismatches.append(f"global med_r: got {ref['global_med_r']:.0f}, expected {METRIC5_REGRESSION_GLOBAL['med_r']}")
if ref["match_f"] != METRIC5_REGRESSION_MATCH["f"]:
    mismatches.append(f"direction match front: got {ref['match_f']}, expected {METRIC5_REGRESSION_MATCH['f']}")
if ref["match_r"] != METRIC5_REGRESSION_MATCH["r"]:
    mismatches.append(f"direction match rear: got {ref['match_r']}, expected {METRIC5_REGRESSION_MATCH['r']}")

if mismatches:
    print("  *** REGRESSION CHECK FAILED -- harness does not reproduce the on-record report:")
    for m in mismatches:
        print(f"  ***   {m}")
    print("  *** Stopping before proceeding further; see the discrepancy above.")
    raise SystemExit(1)
print("  REGRESSION CHECK PASSED -- harness reproduces inspect_c9_negative_cs.py's on-record report.")

# --- Remaining candidates, only reached once the anchor above is trusted. ---
for name, b in betas.items():
    if name == "A_kinematic":
        continue
    metric5_results[name] = _zero_slip_fy_offset(b)
    _print_metric5(name, metric5_results[name])
