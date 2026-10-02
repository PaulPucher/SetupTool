# NBR A -> B anchor examination (PLAN.md PHASE 1 PLAN Q3, author ruling 3 as
# REFRAMED 2026-10-02: "what does A -> B actually look like in the data given
# the corrected feedback"). Read-only. A = outing 21 (nbr_jst_problem), B =
# outing 29 (nbr_jst_solution). Corrected premises (thesis_notes "AUTHOR
# CORRECTIONS 2026-10-02"): the change was RAKE + SPRINGS, no damper clicks;
# B's feedback was "more predictable, safer to drive", not "less understeer".
#
# RULES, fixed before the run. CS = production (per_session fit, the cached
# (d2) chain); the variant question is closed. A has 2 analysis laps, B 6.
# (a) CHANGE CENSUS, logged data only (setup sheets are the author's):
#   every channel block of both raw files, stats over the whole file;
#   SETUP-TYPE candidate = <= 5 distinct values in both files with a different
#   median, or constant within each file at different values. Named groups
#   reported regardless: ride heights per corner and rake in matched speed
#   bands (100-150 / 150-200 / 200-260 kph), tyre pressures (moving median),
#   any channel with "temp" in its name (moving median), fuel used over the
#   file (cumulative channel, Q2b rule 3). Units from each file's own header.
# (b) CORNER TRACES C1 / C4 / C10 (A ids; B partner by bracket overlap > 50 %,
#   as in Q4 (ii)): per lap vs lap distance over the bracket +/- 50 m --
#   speed, a_y, steering-wheel angle, yaw rate, throttle / brake, lateral line
#   offset, |alpha_f| / front peak alpha (session's own production curve),
#   CS_f, CS_r. Lateral offset [impl]: reference line = per-1 m mean of the
#   GPS position over all analysis laps of A and B in the window; offset =
#   signed perpendicular distance, positive left of travel; GPS latency
#   (~0.3 s) shifts both sessions alike. Per-corner numbers per lap: min
#   speed and peak |a_y| in the bracket; apex-region medians of |alpha_f| /
#   peak, lateral offset, |steer|. A vs B resolved iff |dmean| > 2 sqrt(SE_A^2
#   + SE_B^2), SE = jackknife over laps. Descriptive read per corner:
#   OPERATING-POINT SHIFT iff (min speed or peak |a_y| resolved higher in B)
#   and |alpha_f| / peak resolved higher in B; LINE CHANGE iff apex offset
#   resolved; else UNRESOLVED (both labels may apply).
# (c) CONSISTENCY / PHASE FLIPPING over all matched corners (the "predictable"
#   signature):
#   c1 lap-to-lap coherence: per matched corner x phase x axle, mean absolute
#      pairwise difference between laps of the per-lap phase medians
#      (production gate cs_phase_min_valid_samples); defined for 2 laps, so A
#      and B compare without an n bias. Share of cells with B < A; CALMER iff
#      share > 0.5 and one-sided sign test p < 0.05 (ties dropped).
#   c2 phase flipping: per corner-lap, sign changes of the gap CS_r - CS_f
#      across the five phases in order (NaN phases skipped); per matched
#      corner, mean over laps; LESS FLIPPING iff B < A on a share > 0.5 of
#      corners with sign test p < 0.05.
#   c3 (supporting) within-corner gap sign changes per 100 m of the 0.3 s
#      moving-mean gap trace inside the bracket; same paired sign test.
#   c4 (supporting, estimator-free) steering corrections: zero crossings per
#      100 m of the 0.2 s moving-mean steering-wheel rate inside the bracket;
#      same paired sign test.
#   READ: "B shows the calmer signature" iff c1 AND c2 pass; PARTIAL if one;
#   NOT SHOWN if neither. c3 / c4 support, never decide. No criterion is
#   rewritten from this; it characterises the anchor for the record.
# Outputs diagnostics/plots/rake_ab_anchor/. Modes: census | corners | coherence | all

import json
import math
import os
import sys
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks, _strip_unit
from diagnostics.trace_render import INK, BAND, REF

OUT = "diagnostics/plots/rake_ab_anchor"
A_KEY, B_KEY = "nbr_jst_problem", "nbr_jst_solution"
CORNERS = (1, 4, 10)
SPEED_BANDS = ((100, 150), (150, 200), (200, 260))
SPEED_KPH = {"kph": 1.0, "km/h": 1.0, "mph": 1.609344, "m/s": 3.6}
LEN_M = {"mm": 1.0, "m": 1000.0, "ft": 304.8, "in": 25.4}
G = 9.81


def _unit(h):
    return h[h.find("[") + 1:h.find("]")] if "[" in h else ""


def _save(name, obj):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=float)


def _mavg(x, n):
    return np.convolve(np.nan_to_num(x), np.ones(n) / n, mode="same")


def _sign_test(b_less, b_more):
    """One-sided p for 'B < A' from paired signs (ties already dropped)."""
    n = b_less + b_more
    if n == 0:
        return float("nan")
    return float(sum(math.comb(n, k) for k in range(b_less, n + 1)) / 2 ** n)


# ---------------------------------------------------------------- (a) census

def _block_stats(path):
    """One streaming pass, one block in memory at a time."""
    out = {}
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.startswith("{ChannelBlock}"):
                header = f.readline().rstrip("\n").split("\t")
                vs = []
                line = f.readline()
                while line and not line.startswith("{"):
                    parts = line.replace(",", ".").split("\t")
                    if len(parts) == 2:
                        try:
                            vs.append(float(parts[1]))
                        except ValueError:
                            pass
                    line = f.readline()
                if len(header) == 2 and vs:
                    v = np.array(vs)
                    u = np.unique(v)
                    out.setdefault(_strip_unit(header[1]), {
                        "unit": _unit(header[1]), "n": int(v.size), "distinct": int(u.size),
                        "median": float(np.median(v)), "min": float(v.min()), "max": float(v.max()),
                        "values": [float(x) for x in u[:5]] if u.size <= 5 else None})
                continue
            line = f.readline()
    return out


def census():
    paths = {k: d2.sessions()[k]["path"] for k in (A_KEY, B_KEY)}
    st = {k: _block_stats(p) for k, p in paths.items()}
    common = sorted(set(st[A_KEY]) & set(st[B_KEY]))
    setup = []
    for n in common:
        a, b = st[A_KEY][n], st[B_KEY][n]
        if a["distinct"] <= 5 and b["distinct"] <= 5 and a["median"] != b["median"]:
            setup.append({"channel": n, "unit_A": a["unit"], "unit_B": b["unit"],
                          "values_A": a["values"], "values_B": b["values"],
                          "median_A": a["median"], "median_B": b["median"]})
    out = {"n_channels": {k: len(v) for k, v in st.items()}, "common": len(common),
           "only_A": sorted(set(st[A_KEY]) - set(st[B_KEY])), "only_B": sorted(set(st[B_KEY]) - set(st[A_KEY])),
           "setup_type_changed": setup, "named": {}}
    temps = [n for n in common if "temp" in n.lower()]
    named = ["ecu_speed", "log_rideheight_fl", "log_rideheight_fr", "log_rideheight_rl", "log_rideheight_rr",
             "log_rideheight_rake", "tpms_press_fl", "tpms_press_fr", "tpms_press_rl", "tpms_press_rr",
             "ecu_fuelcons_kg"] + temps
    for k, p in paths.items():
        B = read_raw_blocks(p, named)
        ts, vs, hs = B["ecu_speed"]
        spd = vs * SPEED_KPH.get(_unit(hs), 1.0)
        rec = {}
        for n in named[1:]:
            if n not in B:
                continue
            t, v, h = B[n]
            sp = np.interp(t, ts, spd)
            u = _unit(h)
            if n.startswith("log_rideheight"):
                vv = v * LEN_M.get(u, 1.0)
                rec[n + " [mm]"] = {f"{lo}-{hi}": float(np.median(vv[(sp >= lo) & (sp < hi)]))
                                    if np.any((sp >= lo) & (sp < hi)) else None for lo, hi in SPEED_BANDS}
            elif n == "ecu_fuelcons_kg":
                rec[n] = {"start": float(v[0]), "end": float(v[-1]), "burn": float(v[-1] - v[0]), "unit": u}
            else:
                mv = sp > 30
                rec[n] = {"moving_median": float(np.median(v[mv])) if mv.any() else None, "unit": u}
        out["named"][k] = rec
    _save("census.json", out)
    print(f"[census] channels A {out['n_channels'][A_KEY]} B {out['n_channels'][B_KEY]}, common {out['common']}; "
          f"setup-type changed {len(setup)}: " + ", ".join(s["channel"] for s in setup), flush=True)
    for n in out["named"][A_KEY]:
        print(f"  {n}: A {out['named'][A_KEY][n]} | B {out['named'][B_KEY].get(n)}", flush=True)


# ---------------------------------------------------------------- shared context

def _ctx(key):
    from diagnostics.inspect_phase1_q3 import _pims
    params, data, state = d2._state(key)
    F = d2.load_fit(key)
    laps = [l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")]
    return {"params": params, "data": data, "state": state, "F": F, "laps": laps,
            "pims": _pims(key, params, data, state),
            "cs_f": np.asarray(F["prod"]["down"]["cs_f"], float), "cs_r": np.asarray(F["prod"]["down"]["cs_r"], float),
            "peak_f": float(F["prod"]["manifest"]["axles"]["front"]["peak_alpha_deg"])}


def _pairs(cA, cB):
    from diagnostics.inspect_cs_value_lens import _brackets
    ba, bb = _brackets(cA["data"]), _brackets(cB["data"])
    out = {}
    for ca, (a0, a1) in sorted(ba.items()):
        for cb, (b0, b1) in bb.items():
            if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0):
                out[ca] = (cb, (a0, a1), (b0, b1))
                break
    return out


def _lap_mask(c, lap, s_lo, s_hi):
    l = next(x for x in c["data"]["laps"] if x["lap_number"] == lap)
    t, s = c["state"]["time"], c["state"]["s_m"]
    return (t >= l["start_time"]) & (t <= l["end_time"]) & (s >= s_lo) & (s <= s_hi)


def _jk_mean(v):
    v = np.array([x for x in v if np.isfinite(x)])
    n = v.size
    if n < 2:
        return (float(v.mean()) if n else float("nan")), float("nan")
    loo = np.array([np.delete(v, i).mean() for i in range(n)])
    return float(v.mean()), float(np.sqrt((n - 1) / n * np.sum((loo - loo.mean()) ** 2)))


# ---------------------------------------------------------------- (b) corners

def _local_xy(st, origin):
    # one origin for both sessions: each file carries its own gps_origin, and
    # projecting around different origins puts A and B in shifted frames
    lat0, lon0 = np.radians(origin[0]), np.radians(origin[1])
    lat, lon = np.radians(st["gps_lat"]), np.radians(st["gps_lon"])
    return (lon - lon0) * np.cos(lat0) * 6371000.0, (lat - lat0) * 6371000.0


def corners():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cA, cB = _ctx(A_KEY), _ctx(B_KEY)
    pairs = _pairs(cA, cB)
    origin = (cA["state"]["gps_origin_lat"], cA["state"]["gps_origin_lon"])
    xy = {k: _local_xy(c["state"], origin) for k, c in ((A_KEY, cA), (B_KEY, cB))}
    out = {}
    for cid in CORNERS:
        if cid not in pairs:
            out[cid] = {"error": "no B partner"}
            continue
        cb, (a0, a1), (b0, b1) = pairs[cid]
        lo, hi = min(a0, b0) - 50.0, max(a1, b1) + 50.0
        grid = np.arange(lo, hi, 1.0)
        runs = []
        for key, c, cc in ((A_KEY, cA, cid), (B_KEY, cB, cb)):
            st = c["state"]
            for lap in c["laps"]:
                m = _lap_mask(c, lap, lo, hi)
                if m.sum() < 50:
                    continue
                s = st["s_m"][m]
                o = np.argsort(s)
                apex = [cr["apex_lap_distance_m"] for cr in c["data"]["corners"]
                        if cr["stable_corner_id"] == cc and cr["lap_number"] == lap]
                runs.append({"key": key, "lap": lap, "s": s[o], "idx": np.where(m)[0][o],
                             "x": xy[key][0][m][o], "y": xy[key][1][m][o], "apex": apex[0] if apex else float("nan"),
                             "c": c})
        rx = np.nanmean([np.interp(grid, r["s"], r["x"]) for r in runs], axis=0)
        ry = np.nanmean([np.interp(grid, r["s"], r["y"]) for r in runs], axis=0)
        hx, hy = np.gradient(rx), np.gradient(ry)
        hn = np.hypot(hx, hy)
        hx, hy = hx / hn, hy / hn
        per = {A_KEY: {}, B_KEY: {}}
        fig, axs = plt.subplots(9, 1, figsize=(12, 20), sharex=True, constrained_layout=True)
        for r in runs:
            c, st, idx = r["c"], r["c"]["state"], r["idx"]
            gx, gy = np.interp(r["s"], grid, rx), np.interp(r["s"], grid, ry)
            ex, ey = np.interp(r["s"], grid, hx), np.interp(r["s"], grid, hy)
            off = ex * (r["y"] - gy) - ey * (r["x"] - gx)
            v = st["v_mps"][idx] * 3.6
            ay = st["ay_mps2"][idx] / G
            sw = np.degrees(st["steer_sw_rad"][idx])
            yr = np.degrees(st["yaw_rate_radps"][idx])
            af = np.abs(np.degrees(c["F"]["alpha_f"][idx])) / c["peak_f"]
            csf, csr = c["cs_f"][idx], c["cs_r"][idx]
            ls = "--" if r["key"] == A_KEY else "-"
            col = INK if r["key"] == A_KEY else REF
            # pedal channels can be absent from the state on some files
            thr = st["throttle_pct"][idx] if st.get("throttle_pct") is not None else None
            brk = st["brake_f_bar"][idx] if st.get("brake_f_bar") is not None else None
            for ax, y in zip(axs, (v, ay, sw, yr, thr, brk, off, af, None)):
                if y is not None:
                    ax.plot(r["s"], y, ls, color=col, lw=0.8)
            axs[8].plot(r["s"], csf, ls, color=col, lw=0.8)
            axs[8].plot(r["s"], csr, ls, color=col, lw=0.4, alpha=0.6)
            inb = (r["s"] >= (a0 if r["key"] == A_KEY else b0)) & (r["s"] <= (a1 if r["key"] == A_KEY else b1))
            ap = np.abs(r["s"] - r["apex"]) <= c["params"]["stability_estimation"]["cs_apex_region_half_length_m"]
            per[r["key"]][r["lap"]] = {
                "min_speed_kph": float(np.min(v[inb])) if inb.any() else float("nan"),
                "peak_abs_ay_g": float(np.max(np.abs(ay[inb]))) if inb.any() else float("nan"),
                "apex_alpha_f_over_peak": float(np.nanmedian(af[ap])) if ap.any() else float("nan"),
                "apex_offset_m": float(np.median(off[ap])) if ap.any() else float("nan"),
                "apex_abs_steer_deg": float(np.median(np.abs(sw[ap]))) if ap.any() else float("nan")}
        for ax, lab in zip(axs, ("speed [kph]", "a_y [g]", "steer sw [deg]", "yaw rate [deg/s]", "throttle [%]",
                                 "brake front [bar]", "line offset [m]\n(+ left)", "|alpha_f| / peak", "CS f (thick)\nCS r (thin)")):
            ax.set_ylabel(lab, color=INK, fontsize=8)
            ax.axvspan(a0, a1, color=BAND, lw=0, zorder=0)
        axs[7].axhline(1.0, color=REF, lw=0.6)
        axs[8].axhline(0.0, color=REF, lw=0.6)
        axs[-1].set_xlabel("lap distance [m]", color=INK)
        fig.suptitle(f"NBR A (dashed, outing 21, {len(per[A_KEY])} laps) vs B (solid grey, outing 29, "
                     f"{len(per[B_KEY])} laps): C{cid} (B C{cb}); band = A bracket; production CS", color=INK, fontsize=10)
        path = os.path.join(OUT, f"corner_C{cid}.png")
        os.makedirs(OUT, exist_ok=True)
        fig.savefig(path, dpi=90)
        plt.close(fig)
        stats = {}
        for q in ("min_speed_kph", "peak_abs_ay_g", "apex_alpha_f_over_peak", "apex_offset_m", "apex_abs_steer_deg"):
            ma, sa = _jk_mean([d[q] for d in per[A_KEY].values()])
            mb, sb = _jk_mean([d[q] for d in per[B_KEY].values()])
            res = bool(np.isfinite(sa) and np.isfinite(sb) and abs(mb - ma) > 2 * np.sqrt(sa ** 2 + sb ** 2))
            stats[q] = {"A": ma, "se_A": sa, "B": mb, "se_B": sb, "resolved": res}
        up = lambda q: stats[q]["resolved"] and stats[q]["B"] > stats[q]["A"]
        labels = []
        if (up("min_speed_kph") or up("peak_abs_ay_g")) and up("apex_alpha_f_over_peak"):
            labels.append("OPERATING-POINT SHIFT")
        if stats["apex_offset_m"]["resolved"]:
            labels.append("LINE CHANGE")
        out[cid] = {"B_partner": cb, "per_lap": {k: {str(l): d for l, d in v.items()} for k, v in per.items()},
                    "stats": stats, "read": labels or ["UNRESOLVED"], "render": path}
        print(f"[corner C{cid}] read {out[cid]['read']}; " + "; ".join(
            f"{q} A {s['A']:.3f}+/-{s['se_A']:.3f} B {s['B']:.3f}+/-{s['se_B']:.3f}{' R' if s['resolved'] else ''}"
            for q, s in stats.items()), flush=True)
    _save("corners.json", out)


# ---------------------------------------------------------------- (c) coherence

def _phase_medians(c, cid, lap, gate):
    from diagnostics.inspect_cs_readout_evidence import PHASES
    ph = c["pims"].get((cid, lap))
    if ph is None:
        return None
    out = {}
    for p in PHASES:
        i = ph[p]
        f, r = c["cs_f"][i], c["cs_r"][i]
        f, r = f[np.isfinite(f)], r[np.isfinite(r)]
        out[p] = (float(np.median(f)) if f.size >= gate else float("nan"),
                  float(np.median(r)) if r.size >= gate else float("nan"))
    return out


def _pairwise(vals):
    v = [x for x in vals if np.isfinite(x)]
    if len(v) < 2:
        return float("nan")
    return float(np.mean([abs(a - b) for i, a in enumerate(v) for b in v[i + 1:]]))


def _flips(seq):
    s = [np.sign(x) for x in seq if np.isfinite(x) and x != 0]
    return sum(1 for a, b in zip(s, s[1:]) if a != b)


def _crossings_per_100m(y, s):
    sg = np.sign(y)
    sg = sg[sg != 0]
    n = int(np.sum(sg[1:] != sg[:-1])) if sg.size > 1 else 0
    span = float(s.max() - s.min()) if s.size > 1 else 0.0
    return n / span * 100.0 if span > 10 else float("nan")


def coherence():
    from diagnostics.inspect_cs_readout_evidence import PHASES
    cA, cB = _ctx(A_KEY), _ctx(B_KEY)
    gate = cA["params"]["stability_estimation"]["cs_phase_min_valid_samples"]
    pairs = _pairs(cA, cB)
    c1 = {"b_less": 0, "b_more": 0, "cells": []}
    per_corner = {}
    for ca, (cb, (a0, a1), (b0, b1)) in pairs.items():
        pm = {k: {lap: _phase_medians(c, cc, lap, gate) for lap in c["laps"]}
              for k, c, cc in ((A_KEY, cA, ca), (B_KEY, cB, cb))}
        for p_i, ph in enumerate(PHASES):
            for ax_i, ax in enumerate(("f", "r")):
                da = _pairwise([d[ph][ax_i] for d in pm[A_KEY].values() if d])
                db = _pairwise([d[ph][ax_i] for d in pm[B_KEY].values() if d])
                if np.isfinite(da) and np.isfinite(db) and da != db:
                    c1["b_less" if db < da else "b_more"] += 1
                    c1["cells"].append((ca, ph, ax, da, db))
        rec = {}
        for k, c, cc, (s0, s1) in ((A_KEY, cA, ca, (a0, a1)), (B_KEY, cB, cb, (b0, b1))):
            fl, gx, sx = [], [], []
            for lap, d in pm[k].items():
                if d:
                    fl.append(_flips([d[ph][1] - d[ph][0] for ph in PHASES]))
                m = _lap_mask(c, lap, s0, s1)
                if m.sum() < 50:
                    continue
                s = c["state"]["s_m"][m]
                gap = c["cs_r"][m] - c["cs_f"][m]
                ok = np.isfinite(gap)
                if ok.sum() > 50:
                    gx.append(_crossings_per_100m(_mavg(gap[ok], 30), s[ok]))
                sw = np.degrees(c["state"]["steer_sw_rad"][m])
                rate = _mavg(np.gradient(sw, c["state"]["time"][m]), 20)
                sx.append(_crossings_per_100m(rate, s))
            rec[k] = {"flips_mean": float(np.mean(fl)) if fl else float("nan"),
                      "gap_crossings_per_100m": float(np.nanmean(gx)) if gx else float("nan"),
                      "steer_corrections_per_100m": float(np.nanmean(sx)) if sx else float("nan")}
        per_corner[ca] = rec
    res = {"c1": {"b_less": c1["b_less"], "b_more": c1["b_more"],
                  "share_b_less": c1["b_less"] / max(c1["b_less"] + c1["b_more"], 1),
                  "p_one_sided": _sign_test(c1["b_less"], c1["b_more"]),
                  "mean_A": float(np.mean([x[3] for x in c1["cells"]])) if c1["cells"] else float("nan"),
                  "mean_B": float(np.mean([x[4] for x in c1["cells"]])) if c1["cells"] else float("nan")}}
    for name, q in (("c2", "flips_mean"), ("c3", "gap_crossings_per_100m"), ("c4", "steer_corrections_per_100m")):
        less = more = 0
        for rec in per_corner.values():
            a, b = rec[A_KEY][q], rec[B_KEY][q]
            if np.isfinite(a) and np.isfinite(b) and a != b:
                less += b < a
                more += b > a
        res[name] = {"b_less": less, "b_more": more, "share_b_less": less / max(less + more, 1),
                     "p_one_sided": _sign_test(less, more),
                     "mean_A": float(np.nanmean([r[A_KEY][q] for r in per_corner.values()])),
                     "mean_B": float(np.nanmean([r[B_KEY][q] for r in per_corner.values()]))}
    passed = {k: bool(res[k]["share_b_less"] > 0.5 and res[k]["p_one_sided"] < 0.05) for k in ("c1", "c2", "c3", "c4")}
    res["passed"] = passed
    res["read"] = ("B SHOWS THE CALMER SIGNATURE" if passed["c1"] and passed["c2"]
                   else "PARTIAL" if passed["c1"] or passed["c2"] else "NOT SHOWN")
    res["per_corner"] = {str(k): v for k, v in per_corner.items()}
    res["n_matched_corners"] = len(pairs)
    _save("coherence.json", res)
    for k in ("c1", "c2", "c3", "c4"):
        r = res[k]
        print(f"[coherence {k}] A {r['mean_A']:.3f} B {r['mean_B']:.3f}; B<A {r['b_less']} B>A {r['b_more']} "
              f"share {r['share_b_less']:.2f} p {r['p_one_sided']:.4f} pass {passed[k]}", flush=True)
    print(f"[coherence] READ: {res['read']} over {len(pairs)} matched corners", flush=True)
    _render_coherence(cA, cB, pairs, per_corner, c1)


def _render_coherence(cA, cB, pairs, per_corner, c1):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 4, figsize=(16, 4.2), constrained_layout=True)
    da = np.array([x[3] for x in c1["cells"]])
    db = np.array([x[4] for x in c1["cells"]])
    if da.size:
        m = max(da.max(), db.max())
        axs[0].scatter(da, db, s=8, color=INK)
        axs[0].plot([0, m], [0, m], color=REF, lw=0.8)
    axs[0].set_xlabel("A lap-to-lap |dCS| (pairwise mean)", color=INK)
    axs[0].set_ylabel("B", color=INK)
    axs[0].set_title("c1 coherence per corner x phase x axle", color=INK, fontsize=9)
    for ax, q, t in zip(axs[1:], ("flips_mean", "gap_crossings_per_100m", "steer_corrections_per_100m"),
                        ("c2 phase flips per corner-lap", "c3 gap sign changes /100 m", "c4 steering corrections /100 m")):
        ks = sorted(per_corner)
        a = [per_corner[k][A_KEY][q] for k in ks]
        b = [per_corner[k][B_KEY][q] for k in ks]
        x = np.arange(len(ks))
        ax.plot(x, a, "o--", color=INK, ms=4, lw=0.8, label="A outing 21")
        ax.plot(x, b, "s-", color=REF, ms=4, lw=0.8, label="B outing 29")
        ax.set_xticks(x)
        ax.set_xticklabels([f"C{k}" for k in ks], fontsize=7)
        ax.set_title(t, color=INK, fontsize=9)
        ax.legend(frameon=False, fontsize=7)
    fig.suptitle("NBR A vs B consistency / phase-flipping (production CS; c4 estimator-free)", color=INK, fontsize=10)
    fig.savefig(os.path.join(OUT, "coherence_ab.png"), dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    mode = sys.argv[1]
    steps = {"census": census, "corners": corners, "coherence": coherence}
    for name in (("census", "corners", "coherence") if mode == "all" else (mode,)):
        try:
            steps[name]()
        except Exception:
            print(f"[{name}] STEP ERROR\n{traceback.format_exc()}", flush=True)
