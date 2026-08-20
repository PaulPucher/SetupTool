# Shared helpers for diagnostics/plot_*.py scripts. Factored out as a
# deliberate exception to this project's usual "duplicate a small
# pattern rather than extract for a third consumer" convention
# (see plot_sideslip_comparison.py / plot_slip_angle_comparison.py
# headers) -- this module's three new plot scripts bring the total to
# five consumers of the same git-info/canonical-window-slice/corner-
# shading code, at which point duplication cost outweighs abstraction
# cost. Diagnostics-only, no Qt, pure read helpers -- no production
# import beyond what a caller passes in.

import os
import subprocess

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def git_commit_info():
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True).strip())
        return f"{commit} ({'dirty -- uncommitted changes present' if dirty else 'clean'})"
    except Exception as exc:
        return f"unavailable ({exc})"


def canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    """Identical to the helper of the same name used throughout the
    sideslip-methods-comparison arc (inspect_c9_negative_cs.py /
    Metric 5 / WP-S3b/c/S4b / this package's Phase 1-4 scripts)."""
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo or s_m is None:
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


def shade_corners_by_distance(ax, corners, lap_number):
    """axvspan each corner's canonical bracket (bracket_start_m to
    bracket_end_m) on a distance-axis plot for one lap, labelled by
    stable_corner_id."""
    for c in corners:
        if c["lap_number"] != lap_number:
            continue
        bs, be = c.get("bracket_start_m"), c.get("bracket_end_m")
        if bs is None or be is None or be <= bs:
            continue
        ax.axvspan(bs, be, color="gray", alpha=0.15, lw=0)
        sid = c.get("stable_corner_id")
        if sid is not None:
            ymax = ax.get_ylim()[1]
            ax.text((bs + be) / 2, ymax * 0.90, f"C{sid}", fontsize=6, ha="center", color="dimgray")


def pick_representative_corners(corners, racing_ids):
    """Slow/medium/fast racing-speed corners, by median apex_speed
    (km/h) across that stable corner's own instances -- slow=min,
    fast=max, medium=the racing corner whose own median apex speed
    sits closest to the midpoint between slow and fast (not
    necessarily the rank-median corner; a genuinely representative
    'middle of the range' pick given the population may be skewed).
    Returns ({"slow": cid, "medium": cid, "fast": cid}, {cid: median_speed}).
    """
    by_id = {}
    for c in corners:
        sid = c.get("stable_corner_id")
        if sid in racing_ids and c.get("apex_speed") is not None:
            by_id.setdefault(sid, []).append(c["apex_speed"])
    medians = {sid: float(np.median(v)) for sid, v in by_id.items()}
    ranked = sorted(medians.items(), key=lambda kv: kv[1])
    slow_cid, slow_v = ranked[0]
    fast_cid, fast_v = ranked[-1]
    mid_target = (slow_v + fast_v) / 2.0
    medium_cid, medium_v = min(ranked, key=lambda kv: abs(kv[1] - mid_target))
    return {"slow": slow_cid, "medium": medium_cid, "fast": fast_cid}, medians


def find_anchor_before(straight_mask, lap_lo, window_start):
    idx = window_start - 1
    while idx >= lap_lo:
        if straight_mask[idx]:
            return idx
        idx -= 1
    return None


def find_anchor_after(straight_mask, window_end, lap_hi):
    idx = window_end
    while idx < lap_hi:
        if straight_mask[idx]:
            return idx
        idx += 1
    return None
