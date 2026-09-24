# Part B1 (NIS gate band decision + v3 diagnostics work order, 2026-09-03):
# corner-separation evidence for three v3 clusters (C17-C20, C4/C5,
# C8/C9/C10/C11). Read-only -- no config/production changes. For one
# representative lap, reproduces modules.corner_analysis._bracket_corners_
# by_steering's own entering/exiting/merge logic with instrumentation
# (which criterion -- steering or ay -- triggered each transition, and the
# raw pre-merge bracket time-gaps vs bracket_merge_gap_s), then plots
# steering, ay, and the detector's activation trace against lap_distance
# for each cluster. Disposable per CLAUDE.md's diagnostics/ rule.

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from modules.csv_parser import parse_csv
from modules.corner_analysis import _load_config, _slice_channel, _smooth, _bracket_corners_by_steering

RAW_FILE = "GT3_PRC_MLA-v3.txt"
OUT_DIR = "diagnostics/plots_v3"

# From diagnostics/inspect_v3_corner_census.py's own output (canonical
# bracket_start_m/end_m, median across the 4 valid laps) -- distance
# windows widened by a fixed margin to show approach/exit context.
CLUSTERS = {
    "C17-C20": (5319.2 - 60.0, 5485.4 + 60.0),
    "C4-C5": (1138.8 - 60.0, 1413.3 + 60.0),
    "C8-C11": (2829.0 - 60.0, 3224.4 + 60.0),
}


def _pick_representative_lap(data):
    laps = [l for l in data["laps"] if l.get("is_valid_for_analysis")]
    # Fastest valid lap -- same selection rationale as the wheel-load
    # showcase script (cleanest single-lap trace, least likely to carry a
    # lap-specific anomaly).
    return min(laps, key=lambda l: l["end_time"] - l["start_time"])


def _raw_brackets_with_criterion(steering, cd, lat_g):
    """Reproduces _bracket_corners_by_steering's own entering/exiting
    per-sample logic (duplicated here, not imported, purely to attach an
    ENTRY_CRITERION label per bracket for this diagnostic's own reporting
    -- the production function itself has no such label since it doesn't
    need one). Returns the pre-merge bracket list, matching the production
    function's own output before its merge step."""
    sw = cd["smoothing_window_samples"]
    entry_th = cd["steering_entry_threshold_deg"]
    exit_th = cd["steering_exit_threshold_deg"]
    min_dur = cd["min_corner_duration_s"]
    ay_entry_th = cd["ay_entry_threshold_g"]
    ay_exit_th = cd["ay_exit_threshold_g"]

    smoothed = _smooth(steering["data"], sw)
    abs_steer = np.abs(smoothed)
    t = steering["time"]
    ay_on_steer_grid = np.interp(t, lat_g["time"], lat_g["data"])
    ay_abs = np.abs(_smooth(ay_on_steer_grid, sw))

    brackets = []
    in_corner = False
    b_start = 0
    entry_crit = None
    for i in range(len(abs_steer)):
        steer_entering = abs_steer[i] > entry_th
        ay_entering = ay_abs[i] > ay_entry_th
        entering = steer_entering or ay_entering
        exiting = (abs_steer[i] < exit_th) and (ay_abs[i] < ay_exit_th)
        if not in_corner and entering:
            in_corner = True
            b_start = i
            entry_crit = "steering" if steer_entering else "ay"
        elif in_corner and exiting:
            in_corner = False
            if t[i] - t[b_start] >= min_dur:
                brackets.append((b_start, i, entry_crit))
    if in_corner and t[-1] - t[b_start] >= min_dur:
        brackets.append((b_start, len(abs_steer) - 1, entry_crit))
    return brackets, smoothed, ay_abs


def audit_cluster(label, lo_m, hi_m, steering, lat_g, ld_time, ld_data, cd, merge_gap):
    t = steering["time"]
    s_m_on_steer_grid = np.interp(t, ld_time, ld_data)
    brackets, smoothed, ay_abs = _raw_brackets_with_criterion(steering, cd, lat_g)

    in_range = [b for b in brackets if lo_m <= s_m_on_steer_grid[b[0]] <= hi_m]
    print(f"\n{'=' * 70}\n{label} ({lo_m:.1f}-{hi_m:.1f} m), raw pre-merge brackets\n{'=' * 70}")
    print(f"{len(in_range)} raw brackets in range (min_corner_duration_s={cd['min_corner_duration_s']} "
          f"already applied, lateral_g_apex_threshold NOT yet applied -- some may still be rejected "
          f"downstream by _build_corner)")
    for i, (b_start, b_end, crit) in enumerate(in_range):
        dur = t[b_end] - t[b_start]
        direction = "left" if np.mean(smoothed[b_start:b_end + 1]) > 0 else "right"
        s_start = s_m_on_steer_grid[b_start]
        s_end = s_m_on_steer_grid[b_end]
        print(f"  bracket {i}: s=[{s_start:.1f}, {s_end:.1f}] m, t=[{t[b_start]:.2f}, {t[b_end]:.2f}] s, "
              f"dur={dur:.2f} s, entry_criterion={crit}, direction={direction}")
        if i + 1 < len(in_range):
            next_b = in_range[i + 1]
            gap_t = t[next_b[0]] - t[b_end]
            gap_m = s_m_on_steer_grid[next_b[0]] - s_end
            next_dir = "left" if np.mean(smoothed[next_b[0]:next_b[1] + 1]) > 0 else "right"
            same_dir = direction == next_dir
            would_merge = gap_t < merge_gap and same_dir
            print(f"    -> gap to next: {gap_t:.3f} s ({gap_m:.1f} m), same_dir={same_dir} "
                  f"(next={next_dir}), merge_gap={merge_gap} s -> "
                  f"{'WOULD MERGE' if would_merge else 'stays separate'} "
                  f"({'gap >= merge_gap' if gap_t >= merge_gap else 'opposite direction'} "
                  f"if not merging)")

    return in_range, smoothed, ay_abs


def plot_cluster(label, lo_m, hi_m, t, s_m, steering_smoothed, ay_abs, brackets, cd, out_dir):
    mask = (s_m >= lo_m) & (s_m <= hi_m)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(s_m[mask], steering_smoothed[mask], color="tab:blue")
    axes[0].axhline(cd["steering_entry_threshold_deg"], color="grey", linestyle="--", linewidth=0.8)
    axes[0].axhline(-cd["steering_entry_threshold_deg"], color="grey", linestyle="--", linewidth=0.8)
    axes[0].axhline(cd["steering_exit_threshold_deg"], color="lightgrey", linestyle=":", linewidth=0.8)
    axes[0].axhline(-cd["steering_exit_threshold_deg"], color="lightgrey", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("steering (deg)")

    axes[1].plot(s_m[mask], ay_abs[mask], color="tab:orange")
    axes[1].axhline(cd["ay_entry_threshold_g"], color="grey", linestyle="--", linewidth=0.8)
    axes[1].axhline(cd["ay_exit_threshold_g"], color="lightgrey", linestyle=":", linewidth=0.8)
    axes[1].set_ylabel("|ay| smoothed (g)")

    activation = np.zeros(mask.sum())
    s_sub = s_m[mask]
    for b_start, b_end, _crit in brackets:
        b_lo, b_hi = s_m[b_start], s_m[b_end]
        activation[(s_sub >= b_lo) & (s_sub <= b_hi)] = 1.0
    axes[2].fill_between(s_sub, activation, step="mid", color="tab:green", alpha=0.5)
    axes[2].set_ylabel("detector active")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_xlabel("lap_distance (m)")

    fig.suptitle(f"Corner separation evidence -- {label} (raw pre-merge brackets)")
    fig.tight_layout()
    out_path = os.path.join(out_dir, f"corner_separation_{label.replace('/', '_')}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    data = parse_csv(RAW_FILE)
    lap = _pick_representative_lap(data)
    print(f"representative lap: lap_number={lap['lap_number']}, "
          f"duration={lap['end_time'] - lap['start_time']:.2f} s")

    channels = data["channels"]
    config = _load_config()
    cd = config["corner_detection"]
    merge_gap = cd["bracket_merge_gap_s"]

    steering = _slice_channel(channels.get("log_asteer"), lap["start_time"], lap["end_time"])
    lat_g = _slice_channel(channels.get("log_acc_y"), lap["start_time"], lap["end_time"])
    ld_ch = channels.get("lap_distance")
    ld_time = ld_ch["time"] - lap["start_time"]
    ld_data = ld_ch["data"]

    t = steering["time"]
    s_m = np.interp(t, ld_time, ld_data)

    for label, (lo_m, hi_m) in CLUSTERS.items():
        in_range, smoothed, ay_abs = audit_cluster(
            label, lo_m, hi_m, steering, lat_g, ld_time, ld_data, cd, merge_gap
        )
        path = plot_cluster(label, lo_m, hi_m, t, s_m, smoothed, ay_abs, in_range, cd, OUT_DIR)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
