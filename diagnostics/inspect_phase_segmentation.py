# Phase segmentation against the chair basis (author EXECUTE 2026-10-21,
# Q-UTIL addition, read-only). No module or config change.
#
# 8. Throttle census per session: channel ecu_aps (the one corner_analysis
#    reads), unit, range, p99, and per analysis-lap corner instance whether
#    the throttle reaches corner_detection.brake_throttle_max_pct between
#    the previous corner's end and turn-in (the lift-off lookback window).
# 9. Chair rCosPhi phases (chair performance_analysis tooling (internal),
#    corner_characterisation._create_corner_phase_channel and
#    _find_deceleration_points, thresholds unchanged) for our corner
#    instances, against our five phases.
#    rCosPhi = ax / hypot(ax, ay) on the common grid, centred rolling mean of
#    round(0.15 s x rate) samples, min_periods 1 (chair filter_config).
#    Declared simplifications of the chair's corner extent (its full
#    characteristic-point graph needs its curvature channel): start = the
#    last chair deceleration point after the previous corner's apex and
#    before our apex, else our turn-in (counted); end = our corner end.
# Output diagnostics/plots/phase_segmentation/ (phase_segmentation.txt).
# Run: python -m diagnostics.inspect_phase_segmentation

import copy
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "phase_segmentation")
CENSUS = ("nbr_o29_jst", "nbr_o54_cdr", "nbr_o10_jdr", "pr_o25_us_v1")
CHAIR_RUN = ("nbr_o29_jst", "nbr_o10_jdr")
CHAIR_SMOOTH_S = 0.15                               # chair filter_config r_cos_phi
CHAIR_ENTRY, CHAIR_MID, CHAIR_EXIT = -0.97, -0.3, 0.1   # chair defaults


def chair_phase_starts(rcp):
    # chair _create_corner_phase_channel: each boundary = the LAST sample below
    # its threshold, searched from the previous boundary on (0 if none)
    def last_below(seq, val, start):
        hits = np.where(seq[start:] < val)[0]
        return int(hits[-1]) if len(hits) else 0
    e = last_below(rcp, CHAIR_ENTRY, 0)
    m = last_below(rcp, CHAIR_MID, e) + e
    x = last_below(rcp, CHAIR_EXIT, m) + m
    return e, m, x


def chair_decel_points(rcp, s_m, min_distance=50):
    # chair _find_deceleration_points, verbatim logic on one lap
    is_at = np.gradient(np.sign(rcp + 0.15)) != 0
    neg = pd.Series(np.gradient(rcp))[::-1].rolling(20, min_periods=1).quantile(0.75)[::-1].to_numpy() < 0
    pot = np.where(is_at & neg)[0]
    if not len(pot):
        return []
    groups, g = [], [pot[0]]
    for a, b in zip(pot[:-1], pot[1:]):
        if s_m[b] - s_m[a] < min_distance:
            g.append(b)
        else:
            groups.append(g)
            g = [b]
    groups.append(g)
    return [gr[-1] for gr in groups]


def load(key):
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    data = parse_csv(resolve_data_path(src[1]))
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    return label, data, st, load_channels_config()["corner_detection"]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    L = ["PHASE SEGMENTATION against the chair basis (read-only)", ""]
    for key in CENSUS:
        label, data, st, cd = load(key)
        thr_max = cd["brake_throttle_max_pct"]
        ch = data["channels"].get("ecu_aps")
        L.append("=" * 100)
        L.append(f"{label}")
        if ch is None:
            L.append("8. ecu_aps: ABSENT")
            continue
        thr_t, thr = np.asarray(ch["time"], float), np.asarray(ch["data"], float)
        rate = 1.0 / np.median(np.diff(thr_t))
        L.append(f"8. ecu_aps: unit {ch.get('unit_raw', ch.get('unit'))!r}, native rate {rate:.0f} Hz, range "
                 f"{np.nanmin(thr):.1f}..{np.nanmax(thr):.1f}, p50 {np.nanpercentile(thr, 50):.1f}, p99 "
                 f"{np.nanpercentile(thr, 99):.1f}, share >= {thr_max} %: {np.mean(thr >= thr_max) * 100:.1f} %; "
                 f"quality {ch.get('quality')}")
        valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
        corners = sorted((c for c in data["corners"] if c.get("lap_number") in valid),
                         key=lambda c: c["segments"]["entry_2_turnin"][0])
        reach, n, maxes, zero = 0, 0, [], 0
        prev_end = {}
        for c in corners:
            ln = c["lap_number"]
            ti = c["segments"]["entry_2_turnin"][0]
            lo = prev_end.get(ln, -np.inf)
            w = (thr_t >= lo) & (thr_t < ti)
            prev_end[ln] = c["segments"]["exit_5"][1]
            if not w.any():
                continue
            n += 1
            mx = float(np.nanmax(thr[w]))
            maxes.append(mx)
            reach += mx >= thr_max
            zero += (c["segments"]["entry_1_brake"][1] - c["segments"]["entry_1_brake"][0]) <= 0
        L.append(f"   lookback window (previous corner end -> turn-in), analysis laps: {n} instances; throttle reaches "
                 f"{thr_max} on {reach} ({reach / max(1, n) * 100:.0f} %); window max p10 / p50 / p90 "
                 f"{np.percentile(maxes, 10):.1f} / {np.percentile(maxes, 50):.1f} / {np.percentile(maxes, 90):.1f}; "
                 f"zero-length entry_1_brake {zero}")
        if key in CHAIR_RUN:
            L.extend(chair_compare(label, data, st))
    text = "\n".join(L)
    with open(os.path.join(OUT_DIR, "phase_segmentation.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


def chair_compare(label, data, st):
    t, s_m, sr = st["time"], st["s_m"], st["sample_rate_hz"]
    ax, ay = st["ax_mps2"], st["ay_mps2"]
    with np.errstate(invalid="ignore", divide="ignore"):
        rcp_raw = ax / np.hypot(ax, ay)
    win = int(round(CHAIR_SMOOTH_S * sr))
    rcp = pd.Series(rcp_raw).rolling(window=win, center=True, min_periods=1).mean().to_numpy()
    valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
    laps = {l["lap_number"]: l for l in data["laps"]}
    out = [f"9. chair rCosPhi phases vs ours, {label} (median over analysis laps, seconds; boundaries in lap "
           "distance m)"]
    out.append("   ours = brake / turn-in / apex region / exit_4 / exit_5; chair = Braking / Entry / Mid / Exit; "
               "boundary shifts chair minus ours: start vs lift-off, Entry start vs turn-in, Mid start vs apex, "
               "Exit start vs apex")
    rows = {}
    no_decel = 0
    for ln in sorted(valid):
        lap = laps[ln]
        on = np.flatnonzero((t >= lap["start_time"]) & (t <= lap["end_time"]))
        if not on.size:
            continue
        dp = [on[i] for i in chair_decel_points(rcp[on], s_m[on])]
        cs = sorted((c for c in data["corners"] if c.get("lap_number") == ln and c.get("stable_corner_id") is not None),
                    key=lambda c: c["apex_time"])
        prev_apex_i = on[0]
        for c in cs:
            seg = c["segments"]
            i_ap = int(np.searchsorted(t, c["apex_time"]))
            i_ti = int(np.searchsorted(t, seg["entry_2_turnin"][0]))
            i_end = int(np.searchsorted(t, seg["exit_5"][1]))
            cand = [i for i in dp if prev_apex_i < i < i_ap]
            if cand:
                i0 = cand[-1]
            else:
                i0 = i_ti
                no_decel += 1
            prev_apex_i = i_ap
            if i_end - i0 < 3:
                continue
            e, m, x = (i0 + k for k in chair_phase_starts(rcp[i0:i_end + 1]))
            T = lambda a, b: float(t[b] - t[a]) if b > a else 0.0
            i_bs = int(np.searchsorted(t, seg["entry_1_brake"][0]))
            i_half = int(np.searchsorted(t, seg["exit_4"][1]))
            apex_s = c.get("apex_lap_distance_m")
            ar = (np.abs(s_m[i_ti:i_end + 1] - apex_s) <= 25.0).sum() / sr if apex_s == apex_s else np.nan
            r = {"ours": (T(i_bs, i_ti), T(i_ti, i_ap), ar, T(i_ap, i_half), T(i_half, i_end)),
                 "chair": (T(i0, e), T(e, m), T(m, x), T(x, i_end)),
                 "shift": (s_m[i0] - s_m[i_bs], s_m[e] - s_m[i_ti], s_m[m] - s_m[i_ap], s_m[x] - s_m[i_ap])}
            rows.setdefault(int(c["stable_corner_id"]), []).append(r)
    for cid in sorted(rows):
        rr = rows[cid]
        med = lambda k, j: float(np.nanmedian([r[k][j] for r in rr]))
        o = [med("ours", j) for j in range(5)]
        ch = [med("chair", j) for j in range(4)]
        sh = [med("shift", j) for j in range(4)]
        big = int(np.argmax(np.abs(sh)))
        names = ("start", "Entry", "Mid", "Exit")
        out.append(f"   C{cid:<3d} ours {o[0]:.2f} / {o[1]:.2f} / {o[2]:.2f} / {o[3]:.2f} / {o[4]:.2f} s | chair "
                   f"{ch[0]:.2f} / {ch[1]:.2f} / {ch[2]:.2f} / {ch[3]:.2f} s | shift m {sh[0]:+.0f} / {sh[1]:+.0f} / "
                   f"{sh[2]:+.0f} / {sh[3]:+.0f}, largest at {names[big]} (n laps {len(rr)})")
    allr = [r for rr in rows.values() for r in rr]
    out.append(f"   pooled medians: ours brake {np.median([r['ours'][0] for r in allr]):.2f} s, chair Braking "
               f"{np.median([r['chair'][0] for r in allr]):.2f} s; chair Braking zero-length on "
               f"{sum(r['chair'][0] == 0 for r in allr)} of {len(allr)}; ours brake zero-length on "
               f"{sum(r['ours'][0] == 0 for r in allr)}; corners without a chair deceleration point (start = turn-in) "
               f"{no_decel}")
    return out


if __name__ == "__main__":
    main()
