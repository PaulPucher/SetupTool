# Inspects the distribution of CS_ratio and stability worst-phase values,
# per instance and per worst-lap-aggregated corner, under both kinematic
# and ekf_auto_pacejka -- feeds the classification-threshold re-derivation
# work order (thesis_notes.md "Threshold re-derivation deliberately
# deferred", PLAN.md STEP 4). Retains per-instance corner identity
# (stable_corner_id, lap_number) so the physical-anchoring amendment's
# noise-margin sizing can be checked against STEP 2's own findings (C4 =
# genuinely beyond peak; C6/C9 = beta artifacts, largely resolved under a
# corrected beta) and so the worst-lap-per-corner population -- what
# classify_fn will actually be evaluated against once worst-lap
# aggregation ships alongside the re-derived thresholds -- can be
# computed directly (min across a corner's own laps), without needing
# modules.recommendation.aggregate_by_corner's own median-of-medians path.
#
# Refuses to derive against a silently-kinematic run: if ekf_auto_pacejka
# falls back, this raises rather than printing a distribution that looks
# like the new estimator's but is not (same guard as tests/generate_
# golden.py and tests/generate_golden_auto_modes.py).

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state,
    estimate_slip_angles, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
MODES = ["kinematic", "ekf_auto_pacejka"]

# STEP 2's own physical attribution (thesis_notes.md "PLAN.md STEP 2:
# chair-comparable result plots, kinematic vs ekf_pass_1"): C4 front
# saturation is genuine (large and negative under both beta sources, same
# fold shape both times); C6/C9 rear extremes are largely beta artifacts,
# improving sharply under a corrected (non-kinematic) beta source. C8 is
# the corner the existing gap-selected stab_neg_thresh_Nm_per_deg (-50.0,
# 2026-07-24) isolates, consistently destabilising across all 4 laps.
CS_ATTRIBUTION_IDS = (4, 6, 9)
STAB_ATTRIBUTION_ID = 8


def _worst_per_instance(summaries):
    rows = []
    for s in summaries:
        csfs, csrs, stabs = [], [], []
        for _phase, p in s["phases"].items():
            cs_f = p["cs_ratio_f"]["median"]
            cs_r = p["cs_ratio_r"]["median"]
            sb = p["stability_observed_Nm_per_deg"]["median"]
            if cs_f == cs_f:
                csfs.append(cs_f)
            if cs_r == cs_r:
                csrs.append(cs_r)
            if sb == sb:
                stabs.append(sb)
        rows.append({
            "stable_corner_id": s.get("stable_corner_id"),
            "lap_number": s.get("lap_number"),
            "worst_csf": min(csfs) if csfs else float("nan"),
            "worst_csr": min(csrs) if csrs else float("nan"),
            "worst_stab": min(stabs) if stabs else float("nan"),
        })
    return rows


def _worst_per_instance_apex_region(summaries):
    # CS validity repair part A, Phase 4: reproduces _classify_corner's
    # OWN worst-of-5-phases logic exactly -- apex_3's CS reads come from
    # apex_region (summary["apex_region"]), every other phase from its own
    # raw slice, same as ui/views/outing_form.py. This is the statistic
    # that now actually drives verdicts/recommendations; _worst_per_
    # instance above (unmodified) is kept for direct before/after
    # comparison against the pre-repair distribution.
    rows = []
    for s in summaries:
        apex_region = s.get("apex_region")
        csfs, csrs, stabs = [], [], []
        for phase, p in s["phases"].items():
            if phase == "apex_3" and apex_region is not None:
                cs_f = apex_region["cs_ratio_f"]["median"]
                cs_r = apex_region["cs_ratio_r"]["median"]
            else:
                cs_f = p["cs_ratio_f"]["median"]
                cs_r = p["cs_ratio_r"]["median"]
            sb = p["stability_observed_Nm_per_deg"]["median"]
            if cs_f == cs_f:
                csfs.append(cs_f)
            if cs_r == cs_r:
                csrs.append(cs_r)
            if sb == sb:
                stabs.append(sb)
        rows.append({
            "stable_corner_id": s.get("stable_corner_id"),
            "lap_number": s.get("lap_number"),
            "worst_csf": min(csfs) if csfs else float("nan"),
            "worst_csr": min(csrs) if csrs else float("nan"),
            "worst_stab": min(stabs) if stabs else float("nan"),
        })
    return rows


PHASE_KEYS_FOR_FOOTPRINT = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def _no_signal_footprint(summaries):
    # Phase 4: how many phase-level CS stats went no-signal (NaN median),
    # per phase, plus the apex_region validity rate -- the repair's own
    # expected side effect (more NaN, fewer noisy small-n fits) needs to
    # be quantified, not just asserted.
    per_phase = {p: {"n_total": 0, "n_nan_f": 0, "n_nan_r": 0} for p in PHASE_KEYS_FOR_FOOTPRINT}
    apex_region_total = 0
    apex_region_valid_f = 0
    apex_region_valid_r = 0
    for s in summaries:
        for phase in PHASE_KEYS_FOR_FOOTPRINT:
            p = s["phases"].get(phase)
            if p is None:
                continue
            per_phase[phase]["n_total"] += 1
            if p["cs_ratio_f"]["median"] != p["cs_ratio_f"]["median"]:
                per_phase[phase]["n_nan_f"] += 1
            if p["cs_ratio_r"]["median"] != p["cs_ratio_r"]["median"]:
                per_phase[phase]["n_nan_r"] += 1
        ar = s.get("apex_region")
        if ar is not None:
            apex_region_total += 1
            if ar["cs_ratio_f"]["median"] == ar["cs_ratio_f"]["median"]:
                apex_region_valid_f += 1
            if ar["cs_ratio_r"]["median"] == ar["cs_ratio_r"]["median"]:
                apex_region_valid_r += 1
    return per_phase, apex_region_total, apex_region_valid_f, apex_region_valid_r


def _worst_lap_per_corner(rows, key):
    # Min across a corner's own laps -- the population worst-lap
    # aggregation (approved rider 2) will actually evaluate classify_fn
    # against, once it ships alongside the re-derived thresholds.
    by_id = {}
    for r in rows:
        cid = r["stable_corner_id"]
        val = r[key]
        if cid is None or val != val:
            continue
        by_id.setdefault(cid, []).append(val)
    return {cid: min(vals) for cid, vals in by_id.items()}


def _percentiles(values):
    arr = np.asarray(values, dtype=float)
    arr = arr[~np.isnan(arr)]
    return {
        "n": len(arr), "min": float(arr.min()), "p10": float(np.percentile(arr, 10)),
        "p25": float(np.percentile(arr, 25)), "p50": float(np.percentile(arr, 50)),
        "p75": float(np.percentile(arr, 75)), "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def _print_percentiles(label, stats, fmt="{:.3f}"):
    print(f"{label} (n={stats['n']}):")
    for k in ("min", "p10", "p25", "p50", "p75", "p90", "max"):
        print(f"  {k:>4} = {fmt.format(stats[k])}")


def _largest_gaps(values, top_n=3):
    # Sanity cross-check only (amendment item 4) -- reported, never used
    # to place the anchor.
    arr = np.sort(np.asarray(values, dtype=float))
    arr = arr[~np.isnan(arr)]
    if len(arr) < 2:
        return []
    gaps = np.diff(arr)
    idx = np.argsort(gaps)[::-1][:top_n]
    return [(float(arr[i]), float(arr[i + 1]), float(gaps[i])) for i in sorted(idx)]


def _noise_margin(by_id, exclude_ids, fmt="{:.3f}"):
    # Most extreme (lowest) worst-lap value among corners NOT in the
    # excluded (genuinely-beyond-peak / genuinely-destabilising) set --
    # how far estimator noise alone pushes a corner that should read near
    # neutral. The physical anchor (0) minus this margin is the candidate
    # STRONG/negative-stability threshold; the data's role is sizing the
    # margin only, per the amendment.
    pool = {cid: v for cid, v in by_id.items() if cid not in exclude_ids}
    if not pool:
        return None
    worst_cid = min(pool, key=pool.get)
    return worst_cid, pool[worst_cid]


def run_mode(mode, data, params, state):
    beta, _fit_manifest, _gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, mode, csv_path=RAW_FILE
    )
    if fallback_used:
        raise SystemExit(
            f"{mode} fell back to kinematic on this run ({fallback_reason}) -- refusing to derive "
            "thresholds against a silently-kinematic distribution"
        )
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    return summarise_corners(data["corners"], cs, stab, state)


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)

    for mode in MODES:
        print(f"\n{'=' * 78}\nMODE: {mode}\n{'=' * 78}")
        summaries = run_mode(mode, data, params, state)
        rows = _worst_per_instance(summaries)

        csf_inst = [r["worst_csf"] for r in rows]
        csr_inst = [r["worst_csr"] for r in rows]
        stab_inst = [r["worst_stab"] for r in rows]

        print(f"\n-- per-instance (n={len(rows)} corner x lap instances, historical derivation population) --")
        _print_percentiles("worst CSf per instance", _percentiles(csf_inst))
        _print_percentiles("worst CSr per instance", _percentiles(csr_inst))
        _print_percentiles("worst stability per instance", _percentiles(stab_inst), fmt="{:.1f}")

        wl_csf = _worst_lap_per_corner(rows, "worst_csf")
        wl_csr = _worst_lap_per_corner(rows, "worst_csr")
        wl_stab = _worst_lap_per_corner(rows, "worst_stab")

        print(f"\n-- worst-lap-per-corner aggregate (n={len(wl_csf)} physical corners; the population "
              "classify_fn will actually run against once worst-lap aggregation ships, rider 2) --")
        _print_percentiles("worst-lap CSf", _percentiles(list(wl_csf.values())))
        _print_percentiles("worst-lap CSr", _percentiles(list(wl_csr.values())))
        _print_percentiles("worst-lap stability", _percentiles(list(wl_stab.values())), fmt="{:.1f}")

        print("\n-- per-corner worst-lap values, sorted --")
        print("  CSf:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csf.items(), key=lambda kv: kv[1])])
        print("  CSr:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csr.items(), key=lambda kv: kv[1])])
        print("  stab:", [(f"C{cid}", round(v, 1)) for cid, v in sorted(wl_stab.items(), key=lambda kv: kv[1])])

        print("\n-- largest sorted-distribution gaps (sanity cross-check only, amendment item 4) --")
        print("  CSf gaps:", _largest_gaps(list(wl_csf.values())))
        print("  CSr gaps:", _largest_gaps(list(wl_csr.values())))
        print("  stab gaps:", _largest_gaps(list(wl_stab.values())))

        print("\n-- flagged physical-attribution corners --")
        for cid in CS_ATTRIBUTION_IDS:
            print(f"  C{cid}: worst-lap CSf={wl_csf.get(cid)}, CSr={wl_csr.get(cid)}")
        print(f"  C{STAB_ATTRIBUTION_ID}: worst-lap stab={wl_stab.get(STAB_ATTRIBUTION_ID)}")

        print("\n-- noise margin (worst-lap value among corners EXCLUDING the attributed genuine case) --")
        m = _noise_margin(wl_csf, {4})
        print(f"  CSf noise floor (excl. C4): C{m[0]}={m[1]:.3f}" if m else "  CSf: insufficient data")
        m = _noise_margin(wl_csr, {6, 9})
        print(f"  CSr noise floor (excl. C6/C9): C{m[0]}={m[1]:.3f}" if m else "  CSr: insufficient data")
        m = _noise_margin(wl_stab, {STAB_ATTRIBUTION_ID})
        print(f"  stab noise floor (excl. C8): C{m[0]}={m[1]:.1f}" if m else "  stab: insufficient data")

        print("\n-- candidate MODERATE band: percentiles of the POSITIVE-only worst-lap population --")
        pos_csf = [v for v in wl_csf.values() if v > 0]
        pos_csr = [v for v in wl_csr.values() if v > 0]
        if pos_csf:
            _print_percentiles("positive worst-lap CSf", _percentiles(pos_csf))
        if pos_csr:
            _print_percentiles("positive worst-lap CSr", _percentiles(pos_csr))

        # --- CS validity repair part A, Phase 4: apex_region-substituted
        # statistic (what classify_fn/generate_recommendations actually
        # run against now), plus the no-signal footprint. ---
        rows_ar = _worst_per_instance_apex_region(summaries)
        wl_csf_ar = _worst_lap_per_corner(rows_ar, "worst_csf")
        wl_csr_ar = _worst_lap_per_corner(rows_ar, "worst_csr")

        print(f"\n{'-' * 78}\nPHASE 4 (apex_region-substituted, matches classify_fn exactly)\n{'-' * 78}")
        print(f"\n-- worst-lap-per-corner aggregate, apex_region-substituted (n={len(wl_csf_ar)}) --")
        _print_percentiles("worst-lap CSf (apex_region)", _percentiles(list(wl_csf_ar.values())))
        _print_percentiles("worst-lap CSr (apex_region)", _percentiles(list(wl_csr_ar.values())))
        print("  CSf:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csf_ar.items(), key=lambda kv: kv[1])])
        print("  CSr:", [(f"C{cid}", round(v, 3)) for cid, v in sorted(wl_csr_ar.items(), key=lambda kv: kv[1])])

        print("\n-- pre-registration check: raw-apex_3-driven vs apex_region-substituted, per flagged corner --")
        for cid in (1, 2, 3, 4):
            print(f"  C{cid}: raw CSf={wl_csf.get(cid)}, CSr={wl_csr.get(cid)}  |  "
                  f"apex_region CSf={wl_csf_ar.get(cid)}, CSr={wl_csr_ar.get(cid)}")

        per_phase, ar_total, ar_valid_f, ar_valid_r = _no_signal_footprint(summaries)
        print("\n-- no-signal footprint (phase-level cs_ratio NaN fraction) --")
        for phase, counts in per_phase.items():
            n = counts["n_total"]
            if n == 0:
                continue
            print(f"  {phase:>16}: n={n:4d}  no-signal front={counts['n_nan_f']}/{n} "
                  f"({counts['n_nan_f'] / n:.1%})  no-signal rear={counts['n_nan_r']}/{n} "
                  f"({counts['n_nan_r'] / n:.1%})")
        if ar_total:
            print(f"  {'apex_region':>16}: n={ar_total:4d}  valid front={ar_valid_f}/{ar_total} "
                  f"({ar_valid_f / ar_total:.1%})  valid rear={ar_valid_r}/{ar_total} "
                  f"({ar_valid_r / ar_total:.1%})")


if __name__ == "__main__":
    main()
