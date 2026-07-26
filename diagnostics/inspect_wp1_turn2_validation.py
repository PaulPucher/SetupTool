# WP1 consolidation, Turn 2: validation + re-derivation inputs. Read-only.
# Covers items 0 (canonical overlap matrix), 2 (B3-style pre/post
# distribution), 3 (canonical_quiet instance inspection). Item 1 (corner_
# radius_filtered overlap) is a separate script, inspect_corner_radius_
# overlap.py, re-run and extended to a full table alongside this one.
#
# No thresholds changed, no core module code touched -- this only reads
# modules/corner_analysis.py + modules/stability_analysis.py as they stand.

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners, _interp_lap_distance_guarded,
)
import modules.corner_analysis as ca
from ui.views.outing_form import OutingForm

classify_fn = lambda s: OutingForm._classify_corner(None, s)
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]

SRC = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
data = parse_csv(SRC)
channels = data["channels"]
laps = data["laps"]
config = ca._load_config()
cd = config["corner_detection"]
speed_thresholds = config["corner_speed_thresholds"]

# classification thresholds (current, unchanged)
params = load_parameters()
cls_cfg = params["classification"]
STRONG_CSF = cls_cfg["STRONG_CSF"]["value"]
STRONG_CSR = cls_cfg["STRONG_CSR"]["value"]
MODERATE_CSF = cls_cfg["MODERATE_CSF"]["value"]
MODERATE_CSR = cls_cfg["MODERATE_CSR"]["value"]
STAB_NEG_THRESH = cls_cfg["stab_neg_thresh_Nm_per_deg"]["value"]

# --- BEFORE: pass-1 + pass-2 corners, no canonical realization ----------
pre_corners = []
for lap in laps:
    if not lap.get("is_valid_for_analysis", False):
        continue
    pre_corners.extend(ca._analyse_lap(lap, channels, cd, speed_thresholds))
ca.assign_stable_corner_ids(pre_corners, channels)

# --- Modules 1-5 (shared, unaffected by corner realization) --------------
state = prepare_vehicle_state(channels, params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, laps)

summaries_before = summarise_corners(pre_corners, cs, stab, state, lap_filter=None)
summaries_after = summarise_corners(data["corners"], cs, stab, state, lap_filter=None)

print(f"BEFORE: {len(summaries_before)} corner-lap instances (pass-1+pass-2, no canonical realization)")
print(f"AFTER:  {len(summaries_after)} corner-lap instances (canonical realization)\n")


def worst_phase_values(summary):
    csf_vals, csr_vals, stab_vals = [], [], []
    for phase in PHASE_KEYS:
        p = summary["phases"].get(phase)
        if p is None:
            continue
        csf, csr, sob = p["cs_ratio_f"]["median"], p["cs_ratio_r"]["median"], p["stability_observed_Nm_per_deg"]["median"]
        if csf == csf:
            csf_vals.append(csf)
        if csr == csr:
            csr_vals.append(csr)
        if sob == sob:
            stab_vals.append(sob)
    worst_csf = min(csf_vals) if csf_vals else float("nan")
    worst_csr = min(csr_vals) if csr_vals else float("nan")
    worst_stab = min(stab_vals) if stab_vals else float("nan")
    return worst_csf, worst_csr, worst_stab


# =========================================================================
# ITEM 0: canonical overlap matrix
# =========================================================================
print("=" * 70)
print("ITEM 0: CANONICAL OVERLAP MATRIX")
print("=" * 70)

by_id_after = {}
for s in summaries_after:
    by_id_after.setdefault(s["stable_corner_id"], []).append(s)

canon_bounds = {}
for cid, members in by_id_after.items():
    m0 = members[0]
    # bracket_start_m/end_m live on the raw corner dicts, not the summary --
    # pull them straight from data["corners"] for this id (uniform per id).
    raw = next(c for c in data["corners"] if c["stable_corner_id"] == cid)
    canon_bounds[cid] = (raw["bracket_start_m"], raw["bracket_end_m"])

print("\nCanonical bounds per stable corner:")
for cid in sorted(canon_bounds):
    start_m, end_m = canon_bounds[cid]
    print(f"  C{cid}: [{start_m:8.1f}, {end_m:8.1f}] m  (length {end_m - start_m:6.1f} m)")

print("\nPairwise overlap fraction (of the smaller window), pairs > 0.10:")
ids = sorted(canon_bounds)
any_pair = False
for i, a in enumerate(ids):
    a_start, a_end = canon_bounds[a]
    for b in ids[i + 1:]:
        b_start, b_end = canon_bounds[b]
        ov = min(a_end, b_end) - max(a_start, b_start)
        if ov <= 0:
            continue
        frac = ov / min(a_end - a_start, b_end - b_start)
        if frac > 0.10:
            any_pair = True
            print(f"  C{a} <-> C{b}: overlap {ov:.1f} m, fraction {frac:.3f}")
if not any_pair:
    print("  none")

# --- C9/C10/C11 deep dive -----------------------------------------------
print("\n--- C9/C10/C11 deep dive ---")
FOCUS = [9, 10, 11]
for cid in FOCUS:
    s, e = canon_bounds[cid]
    print(f"C{cid}: canonical bounds [{s:.1f}, {e:.1f}] m, length {e - s:.1f} m")

print("\nMutual overlaps among C9/C10/C11:")
for i, a in enumerate(FOCUS):
    a_start, a_end = canon_bounds[a]
    for b in FOCUS[i + 1:]:
        b_start, b_end = canon_bounds[b]
        ov = min(a_end, b_end) - max(a_start, b_start)
        frac = ov / min(a_end - a_start, b_end - b_start) if ov > 0 else 0.0
        print(f"  C{a} <-> C{b}: overlap {max(ov,0):.1f} m, fraction {frac:.3f}")

ld_time, ld_data = channels["lap_distance"]["time"], channels["lap_distance"]["data"]
lat_g_ch = channels.get("log_acc_y")

print("\nPer-lap true local apex position (peak |lat_g| within the canonical "
      "window, independent of the canonical median apex_s) + verdict + apex speed:")
for cid in FOCUS:
    print(f"\n  C{cid} (canonical bounds [{canon_bounds[cid][0]:.1f}, {canon_bounds[cid][1]:.1f}] m):")
    members = sorted(by_id_after[cid], key=lambda s: s["lap_number"])
    for s in members:
        # recover this instance's own bracket time window from the raw corner dict
        raw = next(c for c in data["corners"]
                   if c["stable_corner_id"] == cid and c["lap_number"] == s["lap_number"])
        w_start_t = raw["segments"]["entry_2_turnin"][0]
        w_end_t = raw["segments"]["exit_5"][1]
        g_mask = (lat_g_ch["time"] >= w_start_t) & (lat_g_ch["time"] <= w_end_t)
        if g_mask.any():
            g_t = lat_g_ch["time"][g_mask]
            g_d = np.abs(ca._smooth(lat_g_ch["data"][g_mask], cd["smoothing_window_samples"]))
            true_apex_t = float(g_t[np.argmax(g_d)])
            true_apex_s = float(_interp_lap_distance_guarded(true_apex_t, ld_time, ld_data))
        else:
            true_apex_s = float("nan")
        sev, short, _long, _colour = classify_fn(s)
        quiet = "canonical_quiet" in raw["warnings"]
        print(f"    lap={s['lap_number']}  true_local_apex_s={true_apex_s:8.1f}m  "
              f"apex_v={s['apex_speed']:6.1f} km/h  verdict={sev}:{short}"
              f"{'  [canonical_quiet]' if quiet else ''}")

# --- sample-sharing: C10 vs C9 ------------------------------------------
print("\nSample-sharing (time-domain), C10 vs C9, per lap:")
t_ref = state["time"]
for lap_number in sorted({s["lap_number"] for s in by_id_after[10]}):
    r10 = next(c for c in data["corners"] if c["stable_corner_id"] == 10 and c["lap_number"] == lap_number)
    r9 = next((c for c in data["corners"] if c["stable_corner_id"] == 9 and c["lap_number"] == lap_number), None)
    t10_start, t10_end = r10["segments"]["entry_1_brake"][0], r10["segments"]["exit_5"][1]
    mask10 = (t_ref >= t10_start) & (t_ref <= t10_end)
    n10 = int(mask10.sum())
    if r9 is None or n10 == 0:
        print(f"  lap={lap_number}: C10 n_samples={n10}, no C9 instance this lap")
        continue
    t9_start, t9_end = r9["segments"]["entry_1_brake"][0], r9["segments"]["exit_5"][1]
    mask9 = (t_ref >= t9_start) & (t_ref <= t9_end)
    shared = int((mask10 & mask9).sum())
    print(f"  lap={lap_number}: C10 n_samples={n10}, shared with C9 window={shared} "
          f"({100 * shared / n10:.1f}%)")

# =========================================================================
# ITEM 2: B3-style pre/post distribution diagnostic
# =========================================================================
print("\n" + "=" * 70)
print("ITEM 2: B3-STYLE PRE/POST DISTRIBUTION DIAGNOSTIC")
print("=" * 70)

PCTS = [5, 10, 25, 50, 75, 90, 95]


def report_distribution(label, summaries):
    csf_all, csr_all, stab_all = [], [], []
    per_instance = []
    for s in summaries:
        wcsf, wcsr, wstab = worst_phase_values(s)
        csf_all.append(wcsf)
        csr_all.append(wcsr)
        stab_all.append(wstab)
        per_instance.append((s["stable_corner_id"], s["lap_number"], wcsf, wcsr, wstab))
    csf_a, csr_a, stab_a = np.array(csf_all), np.array(csr_all), np.array(stab_all)

    print(f"\n--- {label} (n={len(summaries)} instances) ---")
    print("Percentiles (worst-phase-per-instance):")
    print(f"  {'pct':>5} {'CS_f':>8} {'CS_r':>8} {'stab':>10}")
    for p in PCTS:
        print(f"  {p:>4}% {np.nanpercentile(csf_a, p):8.3f} {np.nanpercentile(csr_a, p):8.3f} "
              f"{np.nanpercentile(stab_a, p):10.1f}")

    print("Exceedance counts (of instances whose worst phase crosses):")
    print(f"  CS_f < {STRONG_CSF}  (strong):   {int(np.nansum(csf_a < STRONG_CSF))}")
    print(f"  CS_f < {MODERATE_CSF} (moderate): {int(np.nansum(csf_a < MODERATE_CSF))}")
    print(f"  CS_r < {STRONG_CSR}  (strong):   {int(np.nansum(csr_a < STRONG_CSR))}")
    print(f"  CS_r < {MODERATE_CSR} (moderate): {int(np.nansum(csr_a < MODERATE_CSR))}")
    print(f"  stab < 0:     {int(np.nansum(stab_a < 0))}")
    print(f"  stab < {STAB_NEG_THRESH} (threshold): {int(np.nansum(stab_a < STAB_NEG_THRESH))}")
    print(f"  stab < -100:  {int(np.nansum(stab_a < -100))}")

    ranked = sorted(per_instance, key=lambda x: x[4])
    print("Six most negative worst-phase stability values:")
    for cid, lap_n, wcsf, wcsr, wstab in ranked[:6]:
        print(f"    C{cid} lap={lap_n}: stab={wstab:8.1f} Nm/deg  (CS_f={wcsf:.2f} CS_r={wcsr:.2f})")
    if len(ranked) > 6:
        gap = ranked[6][4] - ranked[5][4]
        print(f"  gap between 6th and 7th most negative: {gap:.1f} Nm/deg "
              f"({'clear separation' if gap > 50 else 'no clear gap -- blends into the rest'})")
    ids_in_six = [x[0] for x in ranked[:6]]
    from collections import Counter
    print(f"  stable_corner_id distribution among the six: {dict(Counter(ids_in_six))}")

    return per_instance


pre_instances = report_distribution("BEFORE (pass-1+pass-2, no canonical realization)", summaries_before)
post_instances = report_distribution("AFTER (canonical realization)", summaries_after)

print("\n--- Inter-lap stability agreement per corner, pre vs post (actual numbers) ---")
print(f"  {'corner':>8} {'pre_std':>10} {'pre_range':>12} {'post_std':>10} {'post_range':>12}")


def per_corner_stats(per_instance):
    by_cid = {}
    for cid, lap_n, wcsf, wcsr, wstab in per_instance:
        by_cid.setdefault(cid, []).append(wstab)
    out = {}
    for cid, vals in by_cid.items():
        arr = np.array(vals)
        out[cid] = (float(np.nanstd(arr)), float(np.nanmax(arr) - np.nanmin(arr)), len(arr))
    return out


pre_stats = per_corner_stats(pre_instances)
post_stats = per_corner_stats(post_instances)
tightened, widened = 0, 0
for cid in sorted(set(pre_stats) | set(post_stats)):
    pre = pre_stats.get(cid)
    post = post_stats.get(cid)
    pre_std = f"{pre[0]:.1f}" if pre else "n/a"
    pre_rng = f"{pre[1]:.1f}" if pre else "n/a"
    post_std = f"{post[0]:.1f}" if post else "n/a"
    post_rng = f"{post[1]:.1f}" if post else "n/a"
    print(f"  {'C' + str(cid):>8} {pre_std:>10} {pre_rng:>12} {post_std:>10} {post_rng:>12}")
    if pre and post:
        if post[0] < pre[0]:
            tightened += 1
        elif post[0] > pre[0]:
            widened += 1
print(f"\n  corners where post-canonical std is smaller: {tightened}; "
      f"larger: {widened}; unchanged/n-a: {len(set(pre_stats)|set(post_stats)) - tightened - widened}")

# =========================================================================
# ITEM 3: canonical_quiet instance inspection
# =========================================================================
print("\n" + "=" * 70)
print("ITEM 3: CANONICAL_QUIET INSTANCE INSPECTION")
print("=" * 70)

quiet_raw = [c for c in data["corners"] if "canonical_quiet" in c["warnings"]]
print(f"\n{len(quiet_raw)} canonical_quiet instances total:\n")
for raw in sorted(quiet_raw, key=lambda c: (c["stable_corner_id"], c["lap_number"])):
    cid, lap_n = raw["stable_corner_id"], raw["lap_number"]
    s = next(x for x in summaries_after if x["stable_corner_id"] == cid and x["lap_number"] == lap_n)
    wcsf, wcsr, wstab = worst_phase_values(s)
    sev, short, _long, _colour = classify_fn(s)
    # where does this instance's worst-phase value rank against its own corner's other (non-quiet) laps?
    siblings = [x for x in summaries_after if x["stable_corner_id"] == cid and x["lap_number"] != lap_n]
    sib_stabs = [worst_phase_values(x)[2] for x in siblings]
    sib_csfs = [worst_phase_values(x)[0] for x in siblings]
    sib_csrs = [worst_phase_values(x)[1] for x in siblings]
    print(f"  C{cid} lap={lap_n}: apex_v={raw['apex_speed']:.1f} km/h  "
          f"worst CS_f={wcsf:.3f} CS_r={wcsr:.3f} stab={wstab:.1f}  verdict={sev}:{short}")
    print(f"      other (non-quiet) laps of C{cid}: CS_f={[round(v,3) for v in sib_csfs]}  "
          f"CS_r={[round(v,3) for v in sib_csrs]}  stab={[round(v,1) for v in sib_stabs]}")
