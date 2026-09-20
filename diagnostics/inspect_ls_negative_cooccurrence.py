# LS-evidence work package: negative-population co-occurrence diagnostic.
# Read-only science -- no config/production change, no threshold shipped.
# QUESTION UNDER TEST: are the surviving negative LS_ratio worst-phase
# values (post Metrology-extension-Phase-2 repair) genuine beyond-peak
# longitudinal operation (normal at threshold braking / traction-limited
# exits) or residual estimation noise? Genuine moments should co-occur
# with corroborating signals (ABS/TC activity, high longitudinal demand,
# repeatability across laps); noise should not.
#
# COST CAP (hard, per the work order): no EKF/fit re-run, no CS/stability
# computation at all -- neither is used by this diagnostic, and both are
# expensive (CS's own windowed-regression loop, ~370s/session; the EKF
# fit, ~230s/session). summarise_corners requires cs/stab arguments
# structurally (its own signature), but never RETURNS cs_ratio/stability
# values this script reads -- so a dummy, structurally-valid all-NaN
# array of the right length is passed instead of computing them for
# real. This is the ONE deliberate scope-appropriate shortcut in this
# script, stated here rather than silently taken: it changes nothing
# about what this diagnostic measures (LS_ratio, brake/throttle/ABS/TC/
# ax), only avoids computing two real quantities never read. The
# genuinely unavoidable, real analysis cost is estimate_longitudinal_
# stiffness itself (the repaired, per-sample adaptive-widening
# estimator) -- run ONCE per session, per the work order's own cap.

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, summarise_corners
from modules.longitudinal_forces import estimate_longitudinal_forces, estimate_slip_ratio
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness
from modules.decision_frame import aggregate_ls_by_corner
from modules.recommendation import _group_by_corner, PHASE_KEYS

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"

# Reused EXACTLY from thesis_notes.md "Frame-Stage-2 Phase 2:
# intervention-channel survey" (itself reusing the even earlier "ABS
# consistency check" convention) -- so this package's own numbers are
# directly comparable to the already-recorded ones, not a second,
# incompatible definition.
HARD_BRAKING_BAR = 120.0
TRACTION_THROTTLE_PCT = 80.0
TRACTION_AX_MPS2 = 1.0

PLOTS_DIR = "diagnostics/plots_ls_evidence"


def _interp_channel(channels, name, t_ref):
    ch = channels.get(name)
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None:
        return None
    return np.interp(t_ref, ch["time"], ch["data"])


def _phase_window_idx(corner_instance, phase, t_ref):
    seg = corner_instance["segments"].get(phase)
    if seg is None:
        return None
    start_t, end_t = seg
    if end_t < start_t:
        # apex_3's own point convention -- +/-0.2s around apex_time, same
        # order of magnitude as apex_half_window_samples elsewhere.
        apex_t = corner_instance["apex_time"]
        lo = int(np.searchsorted(t_ref, apex_t - 0.2, side="left"))
        hi = int(np.searchsorted(t_ref, apex_t + 0.2, side="right"))
    else:
        lo = int(np.searchsorted(t_ref, start_t, side="left"))
        hi = int(np.searchsorted(t_ref, end_t, side="right"))
    if hi <= lo:
        return None
    return slice(lo, hi)


def _phase_type(phase):
    if phase == "entry_1_brake":
        return "braking"
    if phase in ("exit_4", "exit_5"):
        return "exit"
    return "other"


def run_session(label, raw_file):
    print(f"\n{'#'*100}\n{label}\n{'#'*100}", flush=True)
    params = load_parameters()
    data = parse_csv(raw_file)
    state = prepare_vehicle_state(data["channels"], params)
    t_ref = state["time"]
    n = len(t_ref)

    long_forces = estimate_longitudinal_forces(state, data["channels"], params)
    slip = estimate_slip_ratio(state, data["channels"], params)
    print("computing LS_ratio (the one real, expensive step this session)...", flush=True)
    ls = estimate_longitudinal_stiffness(long_forces, slip, state, params)
    print("done.", flush=True)

    # Dummy cs/stab -- see module docstring. summarise_corners never
    # gates ls_ratio_f/r on these, confirmed by reading the function.
    cs_dummy = {"CS_ratio_f": np.full(n, np.nan), "CS_ratio_r": np.full(n, np.nan)}
    stab_dummy = {"stability_observed_Nm_per_deg": np.full(n, np.nan), "stability_valid": np.zeros(n, dtype=bool)}

    summaries = summarise_corners(data["corners"], cs_dummy, stab_dummy, state, ls=ls, lap_filter=None)
    ls_by_corner = aggregate_ls_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)

    brake_f = _interp_channel(data["channels"], "log_pbrake_f", t_ref)
    brake_r = _interp_channel(data["channels"], "log_pbrake_r", t_ref)
    throttle = _interp_channel(data["channels"], "ecu_aps", t_ref)
    abs_active = _interp_channel(data["channels"], "abs_active", t_ref)
    tc_active = _interp_channel(data["channels"], "ecu_B_tc_act", t_ref)
    ax = state["ax_mps2"]
    print(f"channels available: brake_f={brake_f is not None} brake_r={brake_r is not None} "
          f"throttle={throttle is not None} abs_active={abs_active is not None} "
          f"tc_active={tc_active is not None}", flush=True)

    corners_by_id = {}
    for c in data["corners"]:
        corners_by_id.setdefault(c["stable_corner_id"], []).append(c)

    instances = []
    for cid, phases in ls_by_corner.items():
        for phase in PHASE_KEYS:
            p = phases.get(phase, {})
            for axle, key, n_key in (("front", "ls_ratio_f", "n_contributing_laps_f"),
                                       ("rear", "ls_ratio_r", "n_contributing_laps_r")):
                val = p.get(key)
                if val is None or val != val or val >= 0:
                    continue

                laps = by_corner_laps.get(cid, [])
                lap_vals = []
                for lap_summary in laps:
                    lp = lap_summary["phases"].get(phase, {})
                    lv_entry = lp.get(key)
                    lv = lv_entry.get("median") if isinstance(lv_entry, dict) else None
                    lap_vals.append((lap_summary["lap_number"], lv))
                finite_lap_vals = [(ln, lv) for ln, lv in lap_vals if lv is not None and lv == lv]
                if not finite_lap_vals:
                    continue
                worst_lap_number, worst_val = min(finite_lap_vals, key=lambda x: x[1])
                negative_laps = [ln for ln, lv in finite_lap_vals if lv < 0]
                repeat_frac = len(negative_laps) / len(finite_lap_vals)

                corner_instance = next(
                    (c for c in corners_by_id.get(cid, []) if c["lap_number"] == worst_lap_number), None)
                if corner_instance is None:
                    continue
                sl = _phase_window_idx(corner_instance, phase, t_ref)
                if sl is None:
                    continue

                bf = float(np.nanmean(brake_f[sl])) if brake_f is not None else float("nan")
                br = float(np.nanmean(brake_r[sl])) if brake_r is not None else float("nan")
                combined_brake = bf + br if (bf == bf and br == br) else float("nan")
                thr = float(np.nanmean(throttle[sl])) if throttle is not None else float("nan")
                abs_duty = float(np.nanmean(abs_active[sl] > 0.5)) if abs_active is not None else float("nan")
                tc_duty = float(np.nanmean(tc_active[sl] > 0.5)) if tc_active is not None else float("nan")
                ax_mean_abs = float(np.nanmean(np.abs(ax[sl])))
                ax_max_abs = float(np.nanmax(np.abs(ax[sl])))

                abs_fires = abs_duty == abs_duty and abs_duty > 0.02
                tc_fires = tc_duty == tc_duty and tc_duty > 0.02
                high_demand = ax_max_abs >= TRACTION_AX_MPS2  # same reference line as the display mask
                repeatable = repeat_frac >= 0.5
                one_off = not repeatable

                # Literal reading of the work order's own classification text:
                # CORROBORATED = (b or c fires) OR (high demand AND repeatable);
                # UNCORROBORATED = (low demand OR one-off) AND no system activity;
                # MIXED = neither of the above. A first draft of this script
                # required low demand for UNCORROBORATED too (an AND that never
                # fires in this dataset, since every negative instance sits
                # above the 1.0 m/s^2 reference line) -- caught by hand-
                # verifying the printed census against the work order's own
                # text before writing up the verdict, corrected here.
                if abs_fires or tc_fires or (high_demand and repeatable):
                    verdict = "CORROBORATED"
                elif (not abs_fires) and (not tc_fires) and ((not high_demand) or one_off):
                    verdict = "UNCORROBORATED"
                else:
                    verdict = "MIXED"

                instances.append({
                    "session": label, "corner": cid, "phase": phase, "phase_type": _phase_type(phase),
                    "axle": axle, "ls_value": val, "worst_lap": worst_lap_number,
                    "n_laps_contributing": len(finite_lap_vals), "n_laps_negative": len(negative_laps),
                    "repeat_frac": repeat_frac,
                    "combined_brake_bar": combined_brake, "throttle_pct": thr,
                    "abs_duty": abs_duty, "tc_duty": tc_duty,
                    "ax_mean_abs": ax_mean_abs, "ax_max_abs": ax_max_abs,
                    "high_demand": high_demand, "abs_fires": abs_fires, "tc_fires": tc_fires,
                    "verdict": verdict,
                })

    return instances


def print_census(instances):
    print(f"\n{'='*100}\nPER-INSTANCE LIST ({len(instances)} negative worst-phase instances)\n{'='*100}")
    for e in sorted(instances, key=lambda e: (e["session"], e["corner"], e["phase"], e["axle"])):
        print(f"{e['session']} C{e['corner']} {e['phase']:14s} {e['axle']:5s} LS={e['ls_value']:+.3f} "
              f"lap={e['worst_lap']} repeat={e['n_laps_negative']}/{e['n_laps_contributing']} "
              f"brake={e['combined_brake_bar']:.1f}bar throttle={e['throttle_pct']:.1f}% "
              f"abs_duty={e['abs_duty']:.2f} tc_duty={e['tc_duty']:.2f} "
              f"|ax|max={e['ax_max_abs']:.2f} -> {e['verdict']}")

    print(f"\n{'='*100}\nCENSUS\n{'='*100}")
    for session in sorted(set(e["session"] for e in instances)):
        sess_inst = [e for e in instances if e["session"] == session]
        print(f"\n{session}: n={len(sess_inst)}")
        for v in ("CORROBORATED", "MIXED", "UNCORROBORATED"):
            n = sum(1 for e in sess_inst if e["verdict"] == v)
            print(f"  {v}: {n}/{len(sess_inst)} ({n/len(sess_inst)*100:.0f}%)" if sess_inst else f"  {v}: 0")
        for pt in ("braking", "exit", "other"):
            n = sum(1 for e in sess_inst if e["phase_type"] == pt)
            if n:
                cor = sum(1 for e in sess_inst if e["phase_type"] == pt and e["verdict"] == "CORROBORATED")
                print(f"  phase_type={pt}: n={n}, corroborated={cor}/{n}")

    total = len(instances)
    if total:
        for v in ("CORROBORATED", "MIXED", "UNCORROBORATED"):
            n = sum(1 for e in instances if e["verdict"] == v)
            print(f"\nPOOLED {v}: {n}/{total} ({n/total*100:.0f}%)")


def plot_census(instances, out_path):
    import os
    os.makedirs(PLOTS_DIR, exist_ok=True)
    sessions = sorted(set(e["session"] for e in instances))
    verdicts = ("CORROBORATED", "MIXED", "UNCORROBORATED")
    colors = {"CORROBORATED": "tab:green", "MIXED": "tab:orange", "UNCORROBORATED": "tab:red"}

    fig, ax = plt.subplots(figsize=(7, 4.5))
    width = 0.25
    x = np.arange(len(sessions))
    for i, v in enumerate(verdicts):
        counts = [sum(1 for e in instances if e["session"] == s and e["verdict"] == v) for s in sessions]
        ax.bar(x + (i - 1) * width, counts, width, label=v, color=colors[v])
    ax.set_xticks(x)
    ax.set_xticklabels(sessions)
    ax.set_ylabel("count of negative worst-phase instances")
    ax.set_title("LS negative-population co-occurrence census")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\nfigure saved: {out_path}")


def main():
    all_instances = []
    all_instances += run_session("Dubai", DUBAI_FILE)
    all_instances += run_session("v3", V3_FILE)
    print_census(all_instances)
    plot_census(all_instances, f"{PLOTS_DIR}/ls_negative_cooccurrence_census.png")


if __name__ == "__main__":
    main()
