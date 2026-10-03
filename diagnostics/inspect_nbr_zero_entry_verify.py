# Q-CAL-2 acceptance -- NBR wheel loads through the production parser with the
# event zero entries (config/channels.json event_zero_entries). Author EXECUTE
# 2026-10-03; read-only, no production or config change. The reference
# diagnostics/inspect_recover_rr.py stays frozen; this script re-derives its
# numbers on the PRODUCTION path.
#
# RULES, frozen before the run.
# FILES: the 16 NBR files of Q-RECOVER-2 (recover.json files_in_outing_order);
#   Paul Ricard v3 and pr_us_v2 as the voltage reference.
# A. ENTRY: per NBR file, parse_csv must report event NBR_221 with all four
#   damper channels "applied" and quality "valid"; damper share = fraction of
#   samples whose production wheel-load source is "damper", per corner, on all
#   samples and on the base mask.
# B. OFFSET REPRODUCTION (frozen acceptance): per file and corner, the gauge-
#   vs-raw-voltage map is re-fitted on the production-corrected channel with
#   the inventory's construction (Input 21 / 29 / 31 / 39 Voltage -> FL / FR /
#   RL / RR, common span of the four gauges, 10 Hz grid, linear interpolation,
#   least-squares line; voltage read with inspect_v4_windows._scan). Paul
#   Ricard reference offset per corner = mean over v3 and pr_us_v2 of the same
#   fit on their production channels (PR FR carries the production channel
#   correction automatically). Implied zero per file = zero_N + (corrected
#   NBR offset - PR offset). PASS per corner iff |median over the 16 files -
#   zero_N| <= that corner's file_spread_sd_N (44.5 / 45.7 / 95.4 / 76.0
#   gauge-N). |r| of each fit reported; a fit below 0.95 is flagged.
# C. V1-V4 on the production loads (Q-SHARED-OFFSET definitions, unchanged):
#   base = production _base_mask & ~kerb_mask (today's +/-100 ms mask; the Q2
#   cache used +/-50 ms -- stated). V1: p0.1 of each corner's Fz on base >=
#   -200 N. V2: corr(a_y [g], Fz) on base, sign left < 0 / right > 0 and |r|
#   >= 0.85. V3: total straight-line load (base, |a_x|, |a_y| < 0.1 g, v > 60
#   km/h) rises with v^2 (OLS slope b > 0). V4: |C1 - zero_N| x MR_straight <=
#   700 N per file and corner with a standing window (C1 from Q2, nbr_qp_mla
#   window 2 only -- inspect_recover_rr._c1).
# EXIT: ACCEPTED iff A passes on all 16 files, B on all four corners and
#   V1-V4 on all 16 files and corners; else the failing items are named.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_nbr_zero_entry_verify

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_shared_offset as so
from diagnostics import inspect_recover_rr as rr
from diagnostics import inspect_v4_windows as v4w

OUT_DIR = "diagnostics/plots/nbr_zero_entry_verify"
CORNERS = ("fl", "fr", "rl", "rr")
PR_KEYS = ("v3", "pr_us_v2")
G = 9.81
STR_G, STR_V_KPH = 0.1, 60.0
V1_FLOOR_N, V2_CORR, BAR_N = -200.0, 0.85, 700.0
SIGN = {"fl": -1, "fr": 1, "rl": -1, "rr": 1}


def _log(m):
    print(m, flush=True)


def _path(key):
    p = d2.sessions()[key]["path"]
    if not os.path.isabs(p) and not os.path.exists(p):
        p = os.path.join(d2.DATA_ROOT, p)
    return p


def map_offsets(key, channels):
    """Inventory construction on the production-parsed gauges."""
    accs, _units, _layout = v4w._scan(_path(key))
    gauge = {}
    for c in CORNERS:
        ch = channels[f"log_dms_dam_{c}"]
        ok = np.isfinite(ch["data"]) & np.isfinite(ch["time"])
        gauge[c] = (ch["time"][ok], ch["data"][ok].astype(float))
    lo = max(g[0][0] for g in gauge.values())
    hi = min(g[0][-1] for g in gauge.values())
    grid = np.arange(lo, hi, 1.0 / v4w.GRID_HZ)
    out = {}
    for c in CORNERS:
        a = accs[rr.VOLT[c]]
        tx, vx = np.concatenate(a.tk), np.concatenate(a.vk).astype(float)
        g = grid[(grid >= tx[0]) & (grid <= tx[-1])]
        x = np.interp(g, tx, vx)
        y = np.interp(g, *gauge[c])
        gain, off = np.polyfit(x, y, 1)
        out[c] = {"offset": float(off), "gain": float(gain), "r": float(np.corrcoef(x, y)[0, 1])}
    return out


def production(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (estimate_lateral_forces, estimate_vertical_loads, load_car_data,
                                            prepare_vehicle_state)
    from modules.tyre_fit_auto import _base_mask
    params = d2._params()
    data = parse_csv(_path(key))
    st = prepare_vehicle_state(data["channels"], params)
    fz = estimate_vertical_loads(st, estimate_lateral_forces(st, params), params,
                                 channels=data["channels"], car_data=load_car_data())
    base = _base_mask(st, data["laps"]) & ~st["kerb_mask"]
    return data, st, fz, base


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    with open("diagnostics/plots/recover_rr/recover.json") as fh:
        files = json.load(fh)["files_in_outing_order"]
    with open("config/channels.json") as fh:
        entry = json.load(fh)["event_zero_entries"]["NBR_221"]["channels"]
    zero = {c: entry[f"log_dms_dam_{c}"]["zero_N"] for c in CORNERS}
    sd = {c: entry[f"log_dms_dam_{c}"]["file_spread_sd_N"] for c in CORNERS}
    lines = ["Q-CAL-2 ACCEPTANCE: NBR event zero entries on the production path"]

    pr = {}
    for key in PR_KEYS:
        data, _st, _fz, _b = production(key)
        pr[key] = map_offsets(key, data["channels"])
        lines.append(f"[{key}] PR reference map: " + ", ".join(
            f"{c} offset {pr[key][c]['offset']:.1f} r {pr[key][c]['r']:.3f}" for c in CORNERS))
        _log(f"[{key}] PR map done")
    o_pr = {c: float(np.mean([pr[k][c]["offset"] for k in PR_KEYS])) for c in CORNERS}

    c1 = rr._c1(files)
    implied = {c: [] for c in CORNERS}
    fails = []
    rows = {}
    for key in files:
        data, st, fz, base = production(key)
        ez = data["event_zero_entry"]
        a_ok = ez["event"] == "NBR_221" and all(
            ez["channels"].get(f"log_dms_dam_{c}") == "applied" and data["channels"][f"log_dms_dam_{c}"]["quality"] == "valid"
            for c in CORNERS)
        src = fz["vertical_load_source_per_sample"]
        share = {c: float(np.mean(np.asarray(src[c]) == "damper")) for c in CORNERS}
        share_b = {c: float(np.mean(np.asarray(src[c])[base] == "damper")) for c in CORNERS}
        maps = map_offsets(key, data["channels"])
        ay_g = st["ay_mps2"] / G
        ax_g = st["ax_mps2"] / G
        v_kph = st["v_mps"] * 3.6
        row = {"A": a_ok, "share": share, "share_base": share_b, "map": maps, "V1": {}, "V2": {}, "V4": {}}
        for c in CORNERS:
            implied[c].append(zero[c] + maps[c]["offset"] - o_pr[c])
            f = fz[f"fz_{c}_N"]
            row["V1"][c] = float(np.nanpercentile(f[base], 0.1))
            m = base & np.isfinite(f)
            row["V2"][c] = float(np.corrcoef(ay_g[m], f[m])[0, 1])
            k1 = c1[key][c]
            mr_s = so.q2(key)["choice"][c]["mr_straight"]
            row["V4"][c] = float(abs(k1 - zero[c]) * mr_s) if np.isfinite(k1) else float("nan")
        tot = fz["fz_fl_N"] + fz["fz_fr_N"] + fz["fz_rl_N"] + fz["fz_rr_N"]
        S = base & (np.abs(ax_g) < STR_G) & (np.abs(ay_g) < STR_G) & (v_kph > STR_V_KPH) & np.isfinite(tot)
        A = np.vstack([np.ones(S.sum()), st["v_mps"][S] ** 2]).T
        (_a, b), *_ = np.linalg.lstsq(A, tot[S], rcond=None)
        row["V3_b"] = float(b)
        row["V1_ok"] = all(row["V1"][c] >= V1_FLOOR_N for c in CORNERS)
        row["V2_ok"] = all(np.sign(row["V2"][c]) == SIGN[c] and abs(row["V2"][c]) >= V2_CORR for c in CORNERS)
        row["V3_ok"] = row["V3_b"] > 0
        row["V4_ok"] = all(row["V4"][c] <= BAR_N for c in CORNERS if np.isfinite(row["V4"][c]))
        rows[key] = row
        for item in ("A", "V1_ok", "V2_ok", "V3_ok", "V4_ok"):
            if not row[item]:
                fails.append(f"{key} {item}")
        lines.append(
            f"\n[{key}] entry {'PASS' if a_ok else 'FAIL'} ({ez['event']}); damper share all / base: "
            + ", ".join(f"{c} {share[c]:.3f}/{share_b[c]:.3f}" for c in CORNERS)
            + f"\n  map r: " + ", ".join(f"{c} {maps[c]['r']:.3f}" for c in CORNERS)
            + f"; implied zero - entry: " + ", ".join(f"{c} {implied[c][-1] - zero[c]:+.1f}" for c in CORNERS)
            + f"\n  V1 p0.1 N: " + ", ".join(f"{c} {row['V1'][c]:.0f}" for c in CORNERS) + f" -> {row['V1_ok']}"
            + f"\n  V2 r: " + ", ".join(f"{c} {row['V2'][c]:+.3f}" for c in CORNERS) + f" -> {row['V2_ok']}"
            + f"\n  V3 b {row['V3_b']:.3f} N/(m/s)^2 -> {row['V3_ok']}"
            + f"\n  V4 err N: " + ", ".join(f"{c} {row['V4'][c]:.0f}" for c in CORNERS) + f" -> {row['V4_ok']}")
        _log(f"[{key}] done")

    lines.append("\nOFFSET REPRODUCTION (median over 16 files of the implied zero vs the config entry)")
    b_ok = True
    for c in CORNERS:
        med = float(np.median(implied[c]))
        ok = abs(med - zero[c]) <= sd[c]
        b_ok &= ok
        weak = [k for k in files if abs(rows[k]["map"][c]["r"]) < v4w.PAIR_R]
        lines.append(f"  {c}: median {med:.1f} vs entry {zero[c]:.1f} -> diff {med - zero[c]:+.1f} gauge-N, tolerance "
                     f"{sd[c]:.1f} -> {'PASS' if ok else 'FAIL'}; file SD {np.std(implied[c], ddof=1):.1f}; "
                     f"maps with |r| < {v4w.PAIR_R}: {weak or 'none'}")
        if not ok:
            fails.append(f"offset reproduction {c}")
    lines.append("\nDAMPER SHARE per corner, min over files (all samples / base): " + ", ".join(
        f"{c} {min(rows[k]['share'][c] for k in files):.3f}/{min(rows[k]['share_base'][c] for k in files):.3f}"
        for c in CORNERS))
    lines.append(f"\nEXIT: {'ACCEPTED' if not fails else 'NOT ACCEPTED -- ' + '; '.join(fails)}")
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "verify.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "verify.json"), "w") as fh:
        json.dump({"o_pr": o_pr, "implied": implied, "rows": rows, "fails": fails}, fh, indent=1, default=float)
    _log(text)


if __name__ == "__main__":
    main()
