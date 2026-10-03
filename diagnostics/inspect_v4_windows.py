# Q-V4-WINDOWS + Q-CHANNEL-INVENTORY (PLAN.md PHASE 1 PLAN; both READ-ONLY, no
# refit, no production or config change; received 2026-10-03 with the
# Q-SHARED-OFFSET rulings; run only on the author's EXECUTE).
#
# RULES, frozen 2026-10-03 BEFORE any compute.
#
# PART W -- V4 WINDOW DIAGNOSTIC.
# QUESTION: the frozen Q-SHARED-OFFSET exit stands (PARTIAL, FR). FL failed V4
#   only on nbr_jst_solution (901 N), RL only on nbr_qp_mla (811 N). V4 = |C1 -
#   K_shared| x MR_s, C1 = Q2's offset from the standing on-wheel windows. The
#   standing-vs-rolling disagreement is itself a MEASUREMENT (author reading,
#   2026-10-03), not presumed check-noise: this part decides CONTAMINATED
#   WINDOWS vs REAL GAP per file; both exits pre-registered, neither favoured.
# CASES: (nbr_jst_solution, FL), (nbr_qp_mla, RL). The other three corners in
#   the same windows are reported as context (they pass V4 on these files).
# DATA: Q2 cache (fz0, MR, v, t, s_m, laps, W), the Q-SHARED-OFFSET burn trace
#   and fit.json (K_shared and the per-file rolling offset K_f, constant
#   variant), the harness state (kerb mask, moving mask, a_x) and the raw
#   channels read fresh: log_susp_travel_*, log_rideheight_*, log_pbrake_f/r,
#   ecu_B_speedlimit_en, ecu_nmot, ecu_gear, log_gps_lat/lon.
# WINDOWS rebuilt exactly as Q2 (v < 3 km/h for >= 1 s; on wheels = all four
#   travels inside the p0.5-p99.5 band of 5-80 km/h moving driving; >= 50 on-
#   wheel samples per window). REPRODUCTION CHECK: window starts / durations
#   and per-corner C1 must match partB_<key>.json to 1 ms / 1 gauge-N, else
#   STOP and report (no evidence is read off a window that is not Q2's).
# EVIDENCE per window (per sample, implied offset k = (fz0 - target) / MR,
#   gauge units; wheel N = gauge x MR_s):
#   E1 extent: start, duration, on-wheel samples used, time from file start /
#      to file end, lap context (before first lap / inside lap n with out- /
#      in-lap flag / after last lap), lap distance, GPS position.
#   E2 jack: share of window samples on wheels; travel and ride height per
#      corner vs the low-speed band and the rolling straight-line median.
#   E3 transient / settling: max speed and max |a_x| in the window; change of
#      k between the first and second half of the on-wheel samples and its
#      linear slope (wheel N); k over +/-20 s around the window (render).
#   E4 pit / procedure context: pit-limiter share, engine speed, gear, front /
#      rear brake pressure in the window.
#   E5 kerb / uneven ground: kerb-mask share in the window, and the kerb-mask
#      share at the same lap distance (+/-10 m) on the analysis laps.
#   E6 load pattern (static equilibrium -- the four wheel loads of a car at
#      rest sum to its weight, standard statics): per corner the window
#      offset error e_c = (k_window - K_f) x MR_s, and their sum over the four
#      corners. A sum near 0 with corner errors of opposite sign = load
#      REDISTRIBUTION between corners (stance, ground, locked suspension); a
#      corner error with no counterpart = a level change of that gauge.
#   E7 consistency: between-window difference of k for the failing corner
#      (wheel N) and which window(s) exceed 700 N against K_shared.
# EXITS per file (frozen; the author rules on the evidence):
#   CONTAMINATED WINDOWS iff every window that exceeds 700 N for the failing
#     corner shows at least one of: E2 on-wheel share < 0.95; E3 half-to-half
#     change > 350 N (half the bar) or |a_x| > 0.1 g; E5 window or same-
#     distance kerb share >= 0.5; AND the windows that do not show any of
#     these stay within 700 N.
#   REAL GAP iff every exceeding window is clean on E2, E3 and E5, and the
#     clean windows agree with each other within 350 N for the failing corner
#     (the gap is reproducible at rest, not one odd stop).
#   Anything else: EVIDENCE SPLIT, items named; the author rules.
#   Either way the bar never moves; ONLY an author ruling "windows invalid"
#   upgrades FL / RL (Q-SHARED-OFFSET ruling 1). E4 and E6 are context: they
#   characterise a real gap (procedure, redistribution vs level), they do not
#   decide the exit.
# RENDERS: one figure per case, one column per window, +/-20 s context: speed
#   with pit limiter; offset error per corner (wheel N, vs the per-file rolling
#   value, +/-700 N band); travel per corner; brake pressure. Titles in words.
#
# PART I -- CHANNEL INVENTORY (all 20 usable files).
# Per file, every channel block (narrow layout: one channel per block; wide
#   layout: Time + all channels as columns, read with pandas in chunks --
#   the two layouts modules/csv_parser.py handles): name, unit (header
#   bracket, as written), samples, rate = (n - 1) / (t_last - t_first),
#   min / max / std, liveness: EMPTY (no samples), STUCK (one distinct value
#   or modal value >= 99 % of samples), else LIVE. Distinct-value counting is
#   capped at 5000 keys (beyond that the channel is LIVE by construction).
# CONSUMED = named in config/channels.json (the parser loads exactly those);
#   ecu_fuelcons_kg is noted as diagnostics-only.
# RELEVANCE (rule-based, frozen): LOAD-ADJACENT if the name matches
#   dam|dms|load|force|strain|push|rod|spring|susp|trav|ride|fz|gauge|sg_|_sg|
#   adc|ain|volt|_v$|raw|offs|zero|cal|tare|bridge|shunt (case-insensitive) or
#   the unit is one of V, mV, N, kN, daN, kg, mm, um, Nm, bit, counts, digit;
#   else OTHER. RAW-vs-PROCESSED PAIR TEST for every LIVE load-adjacent
#   channel except the four consumed gauges: Pearson r with each
#   log_dms_dam_<c> on a common 10 Hz grid over their shared time span; |r|
#   >= 0.95 = LINEAR COMPANION (gain and offset of the least-squares map to
#   the gauge reported), else best |r| and corner reported.
# FAMILY = files with an identical channel-name set; sets with Jaccard overlap
#   >= 0.95 merge into one family with per-file extras / missing listed.
# OUTPUTS diagnostics/plots/v4_windows/: windows.json, window_<key>.png,
#   inventory/inv_<key>.json, inventory/families.json, inventory/
#   families.txt (plain list per family: channel, unit, rate, consumed,
#   relevance), run logs.
#
# Usage: python -m diagnostics.inspect_v4_windows windows | inventory <key> |
#        families

import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_shared_offset as so
from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/v4_windows"
INV_DIR = os.path.join(OUT_DIR, "inventory")
CORNERS = so.CORNERS
WORDS = so.WORDS
G = 9.81
CASES = (("nbr_jst_solution", "fl"), ("nbr_qp_mla", "rl"))
STILL_KPH, STILL_MIN_S, MIN_ON = 3.0, 1.0, 50
BAR_N, HALF_N, AX_G, KERB_SHARE, ONW_MIN = 700.0, 350.0, 0.1, 0.5, 0.95
CTX_S, KERB_DS_M = 20.0, 10.0
RAW_EXTRA = ("ecu_B_speedlimit_en", "ecu_nmot", "ecu_gear", "log_pbrake_f", "log_pbrake_r",
             "log_gps_lat", "log_gps_lon") + tuple(f"log_rideheight_{c}" for c in CORNERS)
REL_RE = re.compile(r"dam|dms|load|force|strain|push|rod|spring|susp|trav|ride|fz|gauge|sg_|_sg|adc|ain|"
                    r"volt|_v$|raw|offs|zero|cal|tare|bridge|shunt", re.I)
REL_UNITS = {"v", "mv", "n", "kn", "dan", "kg", "mm", "um", "nm", "bit", "counts", "digit"}
DISTINCT_CAP, PAIR_R, GRID_HZ, JACCARD = 5000, 0.95, 10.0, 0.95
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2
CORNER_COL = {"fl": "#2563eb", "fr": "#d97706", "rl": "#059669", "rr": "#dc2626"}


def _log(m):
    print(m, flush=True)


def _session_words(key):
    s = d2.sessions()[key]
    return f"{s['track']} outing {s['outing']} (driver {s['driver']})"


# ---------------------------------------------------------------- part W

def _runs(mask, t, min_s):
    d = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    return [(a, b) for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]) if t[b - 1] - t[a] >= min_s]


def _lap_context(t0, laps):
    if not laps or t0 < laps[0]["start_time"]:
        return "before first lap"
    for l in laps:
        if l["start_time"] <= t0 <= l["end_time"]:
            tag = " (out-lap)" if l.get("is_outlap") else " (in-lap)" if l.get("is_inlap") else ""
            return f"inside lap {l['lap_number']}{tag}"
    return "after last lap" if t0 > laps[-1]["end_time"] else "between laps"


def _case(key, corner, fit):
    from modules.wheel_loads import _normalize_travel_to_mm
    from diagnostics.inspect_imu_offsets import read_raw_blocks
    params, data, state = d2._state(key)
    C = so.load(key)
    t = C["t"].astype(float)
    if state["time"].size != t.size or np.max(np.abs(state["time"] - t)) > 1e-6:
        raise SystemExit(f"STOP {key}: Q2 cache time grid differs from the harness state")
    ch, laps = data["channels"], data.get("laps", [])
    v = C["v"].astype(float)
    ax_g = state["ax_mps2"] / G
    trav = {c: _normalize_travel_to_mm(np.interp(t, np.asarray(ch[f"log_susp_travel_{c}"]["time"], float),
                                                 np.asarray(ch[f"log_susp_travel_{c}"]["data"], float)),
                                       ch[f"log_susp_travel_{c}"]["unit_raw"]) for c in CORNERS}
    low = state["moving_mask"] & (v > 5.0) & (v < 80.0)
    band = {c: np.percentile(trav[c][low], [0.5, 99.5]) for c in CORNERS}
    on = np.all([(trav[c] >= band[c][0]) & (trav[c] <= band[c][1]) for c in CORNERS], axis=0)
    runs = _runs(v < STILL_KPH, t, STILL_MIN_S)
    k = {c: (C["fz0"][c].astype(float) - C["target"][c]) / C["mr"][c].astype(float) for c in CORNERS}

    # reproduction check against Q2's own record
    q2 = so.q2(key)
    if len(runs) != len(q2["stationary_windows"]):
        raise SystemExit(f"STOP {key}: {len(runs)} windows vs Q2 {len(q2['stationary_windows'])}")
    for (a, b), w in zip(runs, q2["stationary_windows"]):
        if abs(t[a] - w["t0"]) > 1e-3 or abs((t[b - 1] - t[a]) - w["dur_s"]) > 1e-3:
            raise SystemExit(f"STOP {key}: window at {t[a]:.3f} does not match Q2 {w}")
    for c in CORNERS:
        ks = [float(np.mean(k[c][a:b][on[a:b] & np.isfinite(k[c][a:b])])) for a, b in runs
              if np.sum(on[a:b] & np.isfinite(k[c][a:b])) >= MIN_ON]
        if abs(np.mean(ks) - q2["C1"][c]["K"]) > 1.0:
            raise SystemExit(f"STOP {key} {c}: C1 {np.mean(ks):.1f} vs Q2 {q2['C1'][c]['K']:.1f}")

    raw = read_raw_blocks(d2.sessions()[key]["path"], list(RAW_EXTRA))
    rx = {n: np.interp(t, raw[n][0], raw[n][1]) for n in raw}
    kf = {c: fit["corners"][c]["constant"]["per_file_K"][key] for c in CORNERS}
    ksh = {c: fit["corners"][c]["constant"]["K0"] for c in CORNERS}
    mrs = {c: fit["corners"][c]["mr_straight"] for c in CORNERS}
    an = [l for l in laps if l.get("is_valid_for_analysis")]
    lap_m = np.zeros(t.size, bool)
    for l in an:
        lap_m |= (t >= l["start_time"]) & (t <= l["end_time"])
    S = C["S"]
    rolling = {c: float(np.median(trav[c][S])) for c in CORNERS}
    rolling_rh = {c: float(np.median(rx[f"log_rideheight_{c}"][S])) for c in CORNERS if f"log_rideheight_{c}" in rx}

    wins = []
    for a, b in runs:
        sl = slice(a, b)
        use = on[sl] & np.all([np.isfinite(k[c][sl]) for c in CORNERS], axis=0)
        idx = np.arange(a, b)[use]
        half = idx.size // 2
        s0 = float(np.nanmedian(C["s_m"][sl]))
        near = lap_m & (np.abs(C["s_m"] - s0) <= KERB_DS_M) & (v > STILL_KPH)
        w = {"t0": float(t[a]), "dur_s": float(t[b - 1] - t[a]), "n_on_wheels": int(idx.size),
             "from_file_start_s": float(t[a] - t[0]), "to_file_end_s": float(t[-1] - t[b - 1]),
             "lap_context": _lap_context(t[a], laps), "lap_distance_m": s0,
             "gps": [float(np.nanmedian(rx[n][sl])) if n in rx else None for n in ("log_gps_lat", "log_gps_lon")],
             "E2_on_wheel_share": float(np.mean(on[sl])),
             "E2_travel_mm": {c: {"window": float(np.median(trav[c][sl])), "band": [float(x) for x in band[c]],
                                  "rolling_straight": rolling[c]} for c in CORNERS},
             "E2_rideheight_mm": {c: {"window": float(np.median(rx[f"log_rideheight_{c}"][sl])),
                                      "rolling_straight": rolling_rh[c]} for c in rolling_rh},
             "E3_v_max_kph": float(np.max(v[sl])), "E3_ax_max_g": float(np.max(np.abs(ax_g[sl]))),
             "E4": {"limiter_share": float(np.mean(rx["ecu_B_speedlimit_en"][sl] > 0.5)) if "ecu_B_speedlimit_en" in rx else None,
                    "nmot_median": float(np.median(rx["ecu_nmot"][sl])) if "ecu_nmot" in rx else None,
                    "gear_median": float(np.median(rx["ecu_gear"][sl])) if "ecu_gear" in rx else None,
                    "pbrake_f_median": float(np.median(rx["log_pbrake_f"][sl])) if "log_pbrake_f" in rx else None,
                    "pbrake_r_median": float(np.median(rx["log_pbrake_r"][sl])) if "log_pbrake_r" in rx else None},
             "E5_kerb_share_window": float(np.mean(state["kerb_mask"][sl])),
             "E5_kerb_share_same_distance": float(np.mean(state["kerb_mask"][near])) if near.any() else None,
             "corners": {}}
        for c in CORNERS:
            kw = float(np.mean(k[c][idx])) if idx.size >= MIN_ON else float("nan")
            h1, h2 = float(np.mean(k[c][idx[:half]])), float(np.mean(k[c][idx[half:]]))
            slope = float(np.polyfit(t[idx], k[c][idx], 1)[0]) if idx.size >= MIN_ON else float("nan")
            w["corners"][c] = {"k_window": kw,
                               "E6_err_vs_rolling_file_N": (kw - kf[c]) * mrs[c],
                               "V4_err_vs_shared_N": abs(kw - ksh[c]) * mrs[c],
                               "E3_half_change_N": (h2 - h1) * mrs[c],
                               "E3_slope_N_per_s": slope * mrs[c]}
        w["E6_sum_err_N"] = float(sum(w["corners"][c]["E6_err_vs_rolling_file_N"] for c in CORNERS))
        wins.append(w)

    # exit, frozen rule
    def dirty(w):
        e5 = max(w["E5_kerb_share_window"], w["E5_kerb_share_same_distance"] or 0.0)
        return [n for n, hit in (("E2 on-wheel share", w["E2_on_wheel_share"] < ONW_MIN),
                                 ("E3 half change", abs(w["corners"][corner]["E3_half_change_N"]) > HALF_N),
                                 ("E3 a_x", w["E3_ax_max_g"] > AX_G),
                                 ("E5 kerb", e5 >= KERB_SHARE)) if hit]
    exceed = [w for w in wins if w["corners"][corner]["V4_err_vs_shared_N"] > BAR_N]
    clean = [w for w in wins if not dirty(w)]
    others_ok = all(w["corners"][corner]["V4_err_vs_shared_N"] <= BAR_N for w in clean)
    kc = [w["corners"][corner]["k_window"] * mrs[corner] for w in clean]
    agree = (max(kc) - min(kc) <= HALF_N) if kc else False
    if exceed and all(dirty(w) for w in exceed) and others_ok:
        verdict = "CONTAMINATED WINDOWS"
    elif exceed and all(not dirty(w) for w in exceed) and agree:
        verdict = "REAL GAP"
    else:
        verdict = "EVIDENCE SPLIT"
    rec = {"key": key, "corner": corner, "session": _session_words(key), "K_shared": ksh, "K_file_rolling": kf,
           "mr_straight": mrs, "windows": wins, "dirty_items": [dirty(w) for w in wins],
           "n_exceeding": len(exceed), "clean_windows_agree": agree, "exit": verdict}
    _render_case(key, corner, rec, t, v, rx, k, trav, kf, mrs, runs)
    return rec


def _render_case(key, corner, rec, t, v, rx, k, trav, kf, mrs, runs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(runs)
    fig, ax = plt.subplots(4, n, figsize=(6.5 * n, 11), sharey="row", squeeze=False)
    for j, (a, b) in enumerate(runs):
        m = (t >= t[a] - CTX_S) & (t <= t[b - 1] + CTX_S)
        tt = t[m] - t[a]
        for r in range(4):
            ax[r, j].axvspan(0, t[b - 1] - t[a], color=BAND, zorder=0)
        ax[0, j].plot(tt, v[m], color=INK, lw=1)
        if "ecu_B_speedlimit_en" in rx:
            ax0b = ax[0, j].twinx()
            ax0b.plot(tt, rx["ecu_B_speedlimit_en"][m], color=ACC2, lw=1)
            ax0b.set_ylim(-0.1, 1.5)
            ax0b.set_yticks([0, 1])
            ax0b.set_ylabel("pit limiter (0/1)", color=ACC2)
        for c in CORNERS:
            lw = 2.0 if c == corner else 0.9
            ax[1, j].plot(tt, (k[c][m] - kf[c]) * mrs[c], color=CORNER_COL[c], lw=lw, label=WORDS[c])
            ax[2, j].plot(tt, trav[c][m], color=CORNER_COL[c], lw=lw, label=WORDS[c])
        ax[1, j].axhspan(-BAR_N, BAR_N, color=REF, alpha=0.12, zorder=0)
        ax[1, j].axhline(0, color=REF, lw=0.8)
        for nm, col in (("log_pbrake_f", INK), ("log_pbrake_r", ACC)):
            if nm in rx:
                ax[3, j].plot(tt, rx[nm][m], color=col, lw=1, label=nm.replace("log_pbrake_", "brake pressure "))
        w = rec["windows"][j]
        ax[0, j].set_title(f"standing window {j + 1}: {w['dur_s']:.1f} s, {w['lap_context']}\n"
                           f"on wheels {100 * w['E2_on_wheel_share']:.0f} %", fontsize=10)
        ax[3, j].set_xlabel("time from window start (s)")
    ax[0, 0].set_ylabel("speed (km/h)")
    ax[1, 0].set_ylabel("offset error vs this file's\nrolling value, wheel load (N)")
    ax[2, 0].set_ylabel("suspension travel (mm)")
    ax[3, 0].set_ylabel("brake pressure (bar)")
    ax[1, 0].legend(fontsize=8, loc="upper left")
    ax[3, 0].legend(fontsize=8, loc="upper left")
    fig.suptitle(f"{rec['session']}: standing windows behind the failed {WORDS[corner]} static check\n"
                 f"gauge offset at rest vs the rolling-derived value (band = 700 N bar); "
                 f"pre-registered read: {rec['exit']}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT_DIR, f"window_{key}.png"), dpi=110)
    plt.close(fig)


def windows():
    os.makedirs(OUT_DIR, exist_ok=True)
    fit = json.load(open(os.path.join(so.OUT_DIR, "fit.json")))
    out = {}
    for key, corner in CASES:
        out[key] = _case(key, corner, fit)
        _log(f"[{key} {corner}] exit {out[key]['exit']}; dirty {out[key]['dirty_items']}")
    with open(os.path.join(OUT_DIR, "windows.json"), "w") as fh:
        json.dump(out, fh, indent=1)


# ---------------------------------------------------------------- part I

class _Acc:
    """Streaming per-channel statistics; values never held in full."""

    def __init__(self, keep):
        self.n, self.t0, self.t1 = 0, None, None
        self.mn, self.mx, self.s, self.ss = np.inf, -np.inf, 0.0, 0.0
        self.counts, self.capped = {}, False
        self.keep, self.tk, self.vk = keep, [], []

    def add(self, t, v):
        ok = np.isfinite(v) & np.isfinite(t)
        t, v = t[ok], v[ok]
        if not v.size:
            return
        self.n += v.size
        self.t0 = t[0] if self.t0 is None else self.t0
        self.t1 = t[-1]
        self.mn, self.mx = min(self.mn, v.min()), max(self.mx, v.max())
        self.s += float(v.sum())
        self.ss += float(np.dot(v, v))
        u, c = np.unique(v, return_counts=True)
        for val, cnt in zip(u.tolist(), c.tolist()):
            if val in self.counts:
                self.counts[val] += cnt
            elif len(self.counts) < DISTINCT_CAP:
                self.counts[val] = cnt
            else:
                self.capped = True
        if self.keep:
            self.tk.append(t.astype(float))
            self.vk.append(v.astype(np.float32))

    def stats(self):
        if not self.n:
            return {"n": 0, "liveness": "EMPTY"}
        modal = max(self.counts.values()) / self.n
        distinct = len(self.counts)
        live = "LIVE" if self.capped or (distinct > 1 and modal < 0.99) else "STUCK"
        mean = self.s / self.n
        return {"n": self.n, "rate_hz": (self.n - 1) / (self.t1 - self.t0) if self.t1 > self.t0 else None,
                "t_first": float(self.t0), "t_last": float(self.t1), "min": float(self.mn), "max": float(self.mx),
                "std": float(np.sqrt(max(0.0, self.ss / self.n - mean ** 2))),
                "distinct": f">{DISTINCT_CAP}" if self.capped else distinct, "modal_share": modal, "liveness": live}


def _split(raw):
    raw = raw.strip()
    if "[" in raw and raw.endswith("]"):
        return raw[:raw.index("[")].strip(), raw[raw.index("[") + 1:-1].strip()
    return raw, None


def _relevant(name, unit):
    return bool(REL_RE.search(name)) or (unit or "").strip().lower() in REL_UNITS


def _scan(path):
    import pandas as pd
    accs, units, layout = {}, {}, set()
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if not line.startswith("{ChannelBlock}"):
                line = f.readline()
                continue
            hdr = f.readline().rstrip("\r\n").split("\t")
            if len(hdr) == 2:
                layout.add("narrow")
                name, unit = _split(hdr[1])
                acc = accs.setdefault(name, _Acc(_relevant(name, unit) or name.startswith("log_dms_dam_")))
                units[name] = unit
                ts, vs = [], []
                line = f.readline()
                while line and not line.startswith("{"):
                    p = line.replace(",", ".").split("\t")
                    if len(p) == 2:
                        # parse both before appending: a bad value must not
                        # leave its time stamp behind (arrays drift apart)
                        try:
                            tv, vv = float(p[0]), float(p[1])
                        except ValueError:
                            tv = None
                        if tv is not None:
                            ts.append(tv); vs.append(vv)
                    if len(ts) >= 200000:
                        acc.add(np.array(ts), np.array(vs)); ts, vs = [], []
                    line = f.readline()
                acc.add(np.array(ts), np.array(vs))
                continue
            if len(hdr) > 2 and hdr[0].strip() == "Time":
                layout.add("wide")
                cols = [_split(h) for h in hdr[1:]]
                for name, unit in cols:
                    accs.setdefault(name, _Acc(_relevant(name, unit) or name.startswith("log_dms_dam_")))
                    units[name] = unit
                # collect the block's rows, then let pandas parse them in chunks
                buf = []

                def flush():
                    if not buf:
                        return
                    from io import StringIO
                    df = pd.read_csv(StringIO("".join(buf)), sep="\t", header=None, decimal=",",
                                     na_values=["-nan(ind)", "nan", "-nan", "inf", "-inf"], dtype=float,
                                     on_bad_lines="skip", engine="c")
                    tcol = df.iloc[:, 0].to_numpy()
                    for j, (name, _) in enumerate(cols, start=1):
                        if j < df.shape[1]:
                            accs[name].add(tcol, df.iloc[:, j].to_numpy())
                    buf.clear()
                line = f.readline()
                while line and not line.startswith("{"):
                    buf.append(line)
                    if len(buf) >= 20000:
                        flush()
                    line = f.readline()
                flush()
                continue
            line = f.readline()
    return accs, units, layout


def inventory(key):
    os.makedirs(INV_DIR, exist_ok=True)
    consumed = set(json.load(open("config/channels.json"))["channels"].keys())
    accs, units, layout = _scan(d2.sessions()[key]["path"])
    grid = None
    gauge = {}
    for c in CORNERS:
        a = accs.get(f"log_dms_dam_{c}")
        if a is not None and a.tk:
            gauge[c] = (np.concatenate(a.tk), np.concatenate(a.vk).astype(float))
    if gauge:
        lo = max(g[0][0] for g in gauge.values())
        hi = min(g[0][-1] for g in gauge.values())
        grid = np.arange(lo, hi, 1.0 / GRID_HZ) if hi > lo else None
    out = {}
    for name, a in accs.items():
        st = a.stats()
        rel = "CONSUMED GAUGE" if name.startswith("log_dms_dam_") else (
            "LOAD-ADJACENT" if _relevant(name, units.get(name)) else "OTHER")
        rec = {"unit": units.get(name), **st, "consumed": name in consumed,
               "diagnostics_only": name == "ecu_fuelcons_kg", "relevance": rel}
        if rel == "LOAD-ADJACENT" and st["liveness"] == "LIVE" and grid is not None and a.tk:
            tx, vx = np.concatenate(a.tk), np.concatenate(a.vk).astype(float)
            g = grid[(grid >= tx[0]) & (grid <= tx[-1])]
            best = (0.0, None, None, None)
            if g.size > 100:
                x = np.interp(g, tx, vx)
                for c, (tg, vg) in gauge.items():
                    y = np.interp(g, tg, vg)
                    if np.std(x) > 0 and np.std(y) > 0:
                        r = float(np.corrcoef(x, y)[0, 1])
                        if abs(r) > abs(best[0]):
                            gain, off = np.polyfit(x, y, 1)
                            best = (r, c, float(gain), float(off))
            rec["pair_test"] = {"best_r": best[0], "corner": best[1],
                                "linear_companion": abs(best[0]) >= PAIR_R,
                                "gauge_per_unit": best[2], "offset_gauge": best[3]}
        out[name] = rec
    res = {"key": key, "layout": sorted(layout), "n_channels": len(out), "channels": out}
    with open(os.path.join(INV_DIR, f"inv_{key}.json"), "w") as fh:
        json.dump(res, fh, indent=1)
    la = [n for n, r in out.items() if r["relevance"] == "LOAD-ADJACENT"]
    comp = [n for n, r in out.items() if r.get("pair_test", {}).get("linear_companion")]
    _log(f"[{key}] {sorted(layout)} {len(out)} channels; consumed present "
         f"{sum(r['consumed'] for r in out.values())}; load-adjacent {len(la)}; linear companions {comp}")


def families():
    keys = sorted(d2.usable_keys())
    inv = {k: json.load(open(os.path.join(INV_DIR, f"inv_{k}.json"))) for k in keys}
    sets = {k: frozenset(inv[k]["channels"]) for k in keys}
    fams = []
    for k in keys:
        for f in fams:
            ref = sets[f[0]]
            if len(sets[k] & ref) / len(sets[k] | ref) >= JACCARD:
                f.append(k)
                break
        else:
            fams.append([k])
    summary, lines = [], []
    for i, f in enumerate(fams, start=1):
        union = sorted(set().union(*(sets[k] for k in f)))
        core = set.intersection(*(set(sets[k]) for k in f))
        fam = {"family": i, "files": f, "layout": sorted(set().union(*(inv[k]["layout"] for k in f))),
               "n_union": len(union), "n_core": len(core),
               "per_file_extra": {k: sorted(sets[k] - core) for k in f}, "channels": {}}
        lines.append(f"FAMILY {i}: {', '.join(f)} -- layout {'/'.join(fam['layout'])}; "
                     f"{len(core)} channels in every file, {len(union)} in any")
        for name in union:
            rs = [inv[k]["channels"][name] for k in f if name in inv[k]["channels"]]
            unitset = sorted({str(r["unit"]) for r in rs})
            rates = [r["rate_hz"] for r in rs if r.get("rate_hz")]
            live = {s: sum(r["liveness"] == s for r in rs) for s in ("LIVE", "STUCK", "EMPTY")}
            pt = [r["pair_test"] for r in rs if "pair_test" in r]
            ch = {"unit": unitset, "rate_hz": [min(rates), max(rates)] if rates else None, "liveness": live,
                  "consumed": rs[0]["consumed"], "relevance": rs[0]["relevance"], "in_files": len(rs),
                  "pair_best_abs_r": max((abs(p["best_r"]) for p in pt), default=None),
                  "linear_companion_files": sum(p["linear_companion"] for p in pt)}
            fam["channels"][name] = ch
            rate = f"{ch['rate_hz'][0]:.0f}-{ch['rate_hz'][1]:.0f} Hz" if rates else "no rate"
            extra = ""
            if ch["pair_best_abs_r"] is not None:
                extra = f"; pair |r| max {ch['pair_best_abs_r']:.3f}, companion in {ch['linear_companion_files']} file(s)"
            lines.append(f"- {name}, unit {'/'.join(unitset)}, {rate}, consumed {'yes' if ch['consumed'] else 'no'}, "
                         f"{ch['relevance'].lower()}, live {live['LIVE']} / stuck {live['STUCK']} / empty "
                         f"{live['EMPTY']} of {len(rs)}{extra}")
        lines.append("")
        summary.append(fam)
    with open(os.path.join(INV_DIR, "families.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    with open(os.path.join(INV_DIR, "families.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    for f in summary:
        _log(f"FAMILY {f['family']}: {f['files']} {f['layout']} core {f['n_core']} union {f['n_union']}")


if __name__ == "__main__":
    m = sys.argv[1]
    if m == "windows":
        windows()
    elif m == "inventory":
        inventory(sys.argv[2])
    elif m == "families":
        families()
