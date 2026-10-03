# Q-WHEELLOAD -- per-corner wheel-load renders + damper-gauge decode check
# (PLAN.md PHASE 1 PLAN Q-WHEELLOAD incl. amendment 2; read-only; EXECUTE
# received 2026-10-03 after the Q-PAIRING report).
#
# RULES, frozen 2026-10-03 BEFORE any run. NO new verdict: Q2 ("NBR damper
# offset not recoverable per outing") stands unless Part 0 or the renders force
# it open; the author rules. Corners are NEVER summed to axles. Titles in
# plain words. All 20 usable files (16 NBR serial 221; PR US-v2 and v3, serial
# 221; Dubai and Le Mans, serial 147).
# STANDING WARNINGS honoured: per-file channel units (every ride-height channel
# converted by its own header unit); the Porsche offset convention (offsets
# specified inversely, unusual units) applies to TEAM-SUPPLIED offset values.
#
# PART 0 -- DECODE CHECK (runs first, before any offset statistic)
#   (a) header unit strings of log_dms_dam_* per file, verbatim (the bracket
#       text the parser stored).
#   (b) per file and corner: raw gauge mean and dynamic std on the in-motion,
#       off-kerb analysis-lap samples; the stationary offset per file from Q2
#       (partB_<key>.json choice K, wheel-load chain fz = fz(0) - K MR).
#       Candidates:
#       D1 SCALE (raw / 10, daN logged as N): a pure unit factor rescales the
#          DYNAMICS too. Test: NBR dynamic std vs the healthy serial-221 files
#          (PR US-v2, v3) per corner. The x10 / x0.1 factor is SUPPORTED only if
#          the NBR / PR std ratio lies within [5, 20] or [0.05, 0.2]; a ratio in
#          [0.5, 2] REJECTS it (same scale, additive offset only).
#       D2 SHARED CONSTANT (raw - C per corner, one C for all 16 NBR files):
#          residual spread across files = the spread of the per-file offsets K.
#          COLLAPSE iff max - min of K across the 16 files <= 2 x the median
#          |burn-corrected within-run drift| of that corner (Q2 Part B) -- i.e.
#          within the drift level -- on all four corners.
#       D3 PORSCHE INVERSE: (i) a sign-inverted LOGGED gauge (raw falls with
#          load): tested by the load-transfer sign -- corr(a_y, raw) must be
#          negative on left and positive on right corners for a load-positive
#          gauge; an inverted gauge shows the opposite on all four. (ii) the
#          inverse convention for a TEAM-SUPPLIED offset value: no such value
#          exists in the data -- not testable, recorded.
#       Any candidate that COLLAPSES -> REPORT LOUDLY, Q2 reopens on decode
#       grounds for the author's ruling.
#   (c) record: the dead NBR FL ride-height laser was blocked by a broken
#       carbon piece (author); the team question is struck (already recorded).
# PART 1 -- RENDERS per file, 4 panels FL / FR / RL / RR: wheel load from the
#   production damper chain (modules.wheel_loads) vs lap distance, every
#   analysis lap, config static corner weight dashed (Level 1, event unstated),
#   kerb samples blanked and bridged dashed (core.trace_display.kerb_gap_bridge).
#   Healthy files: the chain as configured (label by the chain's own tier).
#   NBR files: fz(0) - K MR with the Q2 chosen K per corner, labelled L4-rel.
# PART 2 -- FUEL-DRIFT OVERLAY: per file and corner, the median wheel load on
#   straight samples (|a_x|, |a_y| < 0.1 g, off-kerb, analysis laps) in the
#   most populated 20 km/h speed bin above 60 km/h, per analysis lap, against
#   the lap-mean cumulative burn (ecu_fuelcons_kg, the authoritative
#   cumulative channel); OLS slope (N per kg) across laps (>= 3 laps). Expected
#   from fuel mass alone: -g / 4 = -2.45 N/kg per corner (equal split -- tank
#   position NOT stated in car_data, labelled). DRIFT TRACKS BURN iff the slope
#   is negative and within a factor 2 of -2.45 N/kg on >= 3 of 4 corners.
#   If it tracks: Q2's drift bar gets a dated re-read note.
# PART 3 -- SETUP-REFERENCE TEST (author hypothesis: gauges are setup-referenced,
#   offsets = setup history): across the 16 NBR files, Spearman correlation of
#   each corner's offset K with (i) that corner's ride height at matched speed
#   (straight samples, the Part 2 speed bin, median; FL excluded -- laser dead
#   on NBR) and (ii) the outing order (time trend). SUPPORTED iff |rho(K, ride
#   height)| >= 0.6 on >= 2 of the 3 usable corners AND exceeds |rho(K, order)|
#   there. Rake pair A / B (nbr_jst_problem -> nbr_jst_solution): change of K
#   per corner beside the change of ride height. The Porsche convention is
#   re-read in this frame in the record (no computation).
# PART 4 -- CROSS-FILE PER CORNER: K per corner across the 16 NBR files in
#   outing order with the Q2 uncertainty u as error bars -- the engineer-
#   conversation question ("does the same corner's offset jump file to file")
#   answered visually.
# Outputs diagnostics/plots/wheelload/.
#
# Usage: python -m diagnostics.inspect_wheelload part0 | file <key> | summary

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/wheelload"
Q2_DIR = "diagnostics/plots/damper_offset_q2"
CORNERS = ("fl", "fr", "rl", "rr")
G = 9.81
HEALTHY_221 = ("pr_us_v2", "v3")
STR_G, STR_V_KPH, BIN_KPH = 0.10, 60.0, 20.0
EXPECTED_N_PER_KG = -G / 4.0
INK, BAND, REF, ACC, ACC2 = lp.INK, lp.BAND, lp.REF, lp.ACC, lp.ACC2
CORNER_WORDS = {"fl": "front left", "fr": "front right", "rl": "rear left", "rr": "rear right"}


def _dump(name, obj):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w") as fh:
        json.dump(obj, fh, indent=1, default=lp._jsonable)


def _log(m):
    print(m, flush=True)


def nbr_keys():
    return [k for k in d2.usable_keys() if k.startswith("nbr_")]


def title_of(key):
    s = d2.sessions()[key]
    names = {"NBR": "Nurburgring", "PRC": "Paul Ricard", "Dubai": "Dubai", "LMN": "Le Mans"}
    return f"{names.get(s['track'], s['track'])}, driver {s['driver']}, outing {s['outing']}"


def q2(key):
    return json.load(open(os.path.join(Q2_DIR, f"partB_{key}.json")))


def unit_factor_mm(unit):
    return {"mm": 1.0, "m": 1000.0, "ft": 304.8, "in": 25.4}.get((unit or "").strip(), float("nan"))


class F:
    """One file: state, base mask, raw gauges, chain loads."""

    def __init__(self, key):
        from modules.wheel_loads import estimate_wheel_loads_from_dampers
        from modules.stability_analysis import load_car_data
        from modules.tyre_fit_auto import _base_mask
        self.key = key
        self.params, self.data, self.state = d2._state(key)
        st, ch = self.state, self.data["channels"]
        self.t = st["time"]
        self.kerb = st["kerb_mask"] if st.get("kerb_mask") is not None else np.zeros(self.t.size, dtype=bool)
        self.base = _base_mask(st, self.data.get("laps", [])) & ~self.kerb
        self.v = st["v_mps"] * 3.6
        self.ax_g, self.ay_g = st["ax_mps2"] / G, st["ay_mps2"] / G
        self.raw = {c: np.interp(self.t, np.asarray(ch[f"log_dms_dam_{c}"]["time"], float),
                                 np.asarray(ch[f"log_dms_dam_{c}"]["data"], float)) for c in CORNERS}
        self.units = {c: ch[f"log_dms_dam_{c}"].get("unit_raw") for c in CORNERS}
        cw = self.params["vehicle"]["corner_weights"]
        self.W = {"fl": cw["FL_kg"] * G, "fr": cw["FR_kg"] * G, "rl": cw["RL_kg"] * G, "rr": cw["RR_kg"] * G}
        ch2 = dict(ch)
        nbr = key.startswith("nbr_")
        if nbr:  # force the gauges through the chain at K = 0 (as Q2)
            for c in CORNERS:
                ch2[f"log_dms_dam_{c}"] = dict(ch[f"log_dms_dam_{c}"], quality="valid")
        wl = estimate_wheel_loads_from_dampers(st, ch2, self.params, load_car_data())
        self.mr = {c: wl[c]["motion_ratio"] for c in CORNERS}
        self.tier = {c: f"valid share {float(np.mean(wl[c]['valid'])):.2f}, dead {bool(np.any(wl[c]['dead_channel']))}"
                     for c in CORNERS}
        if nbr:
            Q = q2(key)
            self.K = {c: Q["choice"][c]["K"] for c in CORNERS}
            self.fz = {c: wl[c]["fz_N"] - self.K[c] * self.mr[c] for c in CORNERS}
            self.label = "L4-rel (Q2 offset per corner)"
        else:
            # healthy files: only what the chain itself marks valid (a dead
            # corner, e.g. Dubai RR, stays empty rather than drawn from junk)
            self.K = None
            self.fz = {c: np.where(wl[c]["valid"], wl[c]["fz_N"], np.nan) for c in CORNERS}
            self.label = "damper chain as configured (valid samples only)"
        self.laps = [l for l in self.data.get("laps", []) if l.get("is_valid_for_analysis")]
        self.s_m = st["s_m"]
        self.straight = self.base & (np.abs(self.ax_g) < STR_G) & (np.abs(self.ay_g) < STR_G) & (self.v > STR_V_KPH)
        edges = np.arange(STR_V_KPH, 320, BIN_KPH)
        cnt = [int((self.straight & (self.v >= e) & (self.v < e + BIN_KPH)).sum()) for e in edges]
        self.bin = (float(edges[int(np.argmax(cnt))]), float(edges[int(np.argmax(cnt))] + BIN_KPH))
        self.sb = self.straight & (self.v >= self.bin[0]) & (self.v < self.bin[1])

    def ride_height_mm(self, c):
        ch = self.data["channels"].get(f"log_rideheight_{c}")
        if ch is None:
            return float("nan"), None
        u = ch.get("unit_raw")
        x = np.interp(self.t, np.asarray(ch["time"], float), np.asarray(ch["data"], float)) * unit_factor_mm(u)
        return float(np.median(x[self.sb])) if self.sb.any() else float("nan"), u


# ---------------------------------------------------------------- Part 0

def part0():
    out = {"files": {}, "healthy": {}}
    for key in d2.usable_keys():
        f = F(key)
        mv = f.base
        rec = {"units_verbatim": f.units,
               "raw_mean_N": {c: float(np.mean(f.raw[c][mv])) for c in CORNERS},
               "raw_dyn_std_N": {c: float(np.std(f.raw[c][mv])) for c in CORNERS},
               "corr_ay_raw": {c: float(np.corrcoef(f.ay_g[mv], f.raw[c][mv])[0, 1]) for c in CORNERS}}
        if key.startswith("nbr_"):
            Q = q2(key)
            rec["K"] = f.K
            rec["u_wheel_N"] = Q["u_wheel_N"]
            rec["drift_burn_corrected_N"] = {c: Q["drift"][c]["burn_corrected_N"] for c in CORNERS}
        out["files"][key] = rec
        _log(f"[{key}] units {f.units}; raw mean {[round(rec['raw_mean_N'][c]) for c in CORNERS]}; dyn std "
             f"{[round(rec['raw_dyn_std_N'][c]) for c in CORNERS]}; corr(a_y, raw) "
             f"{[round(rec['corr_ay_raw'][c], 2) for c in CORNERS]}")
    F_ = out["files"]
    nbr = [k for k in F_ if k.startswith("nbr_")]
    # D1 scale
    ref_std = {c: float(np.mean([F_[k]["raw_dyn_std_N"][c] for k in HEALTHY_221])) for c in CORNERS}
    ratio = {c: float(np.median([F_[k]["raw_dyn_std_N"][c] for k in nbr]) / ref_std[c]) for c in CORNERS}
    d1 = {"healthy_221_dyn_std_N": ref_std, "nbr_over_healthy_std_ratio": ratio,
          "supported": bool(all((5 <= r <= 20) or (0.05 <= r <= 0.2) for r in ratio.values())),
          "rejected_same_scale": bool(all(0.5 <= r <= 2 for r in ratio.values()))}
    # D2 shared constant
    d2r = {}
    for c in CORNERS:
        Ks = np.array([F_[k]["K"][c] for k in nbr])
        dr = np.median([abs(F_[k]["drift_burn_corrected_N"][c]) for k in nbr])
        d2r[c] = {"K_min": float(Ks.min()), "K_max": float(Ks.max()), "K_range": float(Ks.max() - Ks.min()),
                  "K_sd": float(Ks.std(ddof=1)), "median_abs_drift": float(dr),
                  "collapse": bool(Ks.max() - Ks.min() <= 2 * dr)}
    d2_ok = all(x["collapse"] for x in d2r.values())
    # D3 sign
    exp_sign = {"fl": -1, "fr": 1, "rl": -1, "rr": 1}
    sign_ok = {k: {c: bool(np.sign(F_[k]["corr_ay_raw"][c]) == exp_sign[c]) for c in CORNERS} for k in F_}
    n_inverted_all4 = sum(all(not v for v in s.values()) for s in sign_ok.values())
    out["D1_scale"] = d1
    out["D2_shared_constant"] = {"per_corner": d2r, "collapse_all_corners": d2_ok}
    out["D3_inverse"] = {"load_positive_sign_per_file": sign_ok, "files_inverted_on_all_four": n_inverted_all4,
                         "team_supplied_offset": "no team-supplied offset value exists in the data -- not testable"}
    out["any_collapse"] = bool(d1["supported"] or d2_ok or n_inverted_all4 > 0)
    _log("PART 0 -- D1 " + json.dumps(d1) + "\nD2 " + json.dumps(d2r) + f"\nD3 files inverted on all four: {n_inverted_all4}"
         f"; ANY COLLAPSE: {out['any_collapse']}")
    _dump("part0.json", out)


# ---------------------------------------------------------------- per file

def file_run(key):
    from core.trace_display import kerb_gap_bridge
    from diagnostics.inspect_imu_offsets import read_raw_blocks
    f = F(key)
    plt = lp._plt()
    # Part 1 render
    lap_len = max(float(np.nanmax(f.s_m[(f.t >= l["start_time"]) & (f.t <= l["end_time"])])) for l in f.laps) if f.laps else 1
    fig, axes = plt.subplots(4, 1, figsize=(max(14, lap_len / 400), 12), sharex=True, constrained_layout=True)
    for axp, c in zip(axes, CORNERS):
        y = np.where(f.kerb, np.nan, f.fz[c])
        for l in f.laps:
            m = (f.t >= l["start_time"]) & (f.t <= l["end_time"])
            axp.plot(f.s_m[m], y[m] / 1000, color=INK, lw=0.5, alpha=0.6)
            axp.plot(f.s_m[m], kerb_gap_bridge(f.s_m[m], y[m] / 1000, f.kerb[m]), color=INK, lw=0.5, alpha=0.3, ls="--")
        axp.axhline(f.W[c] / 1000, color=ACC2, lw=1.2, ls="--", label="config static corner weight (Level 1)")
        axp.set_ylabel(f"{CORNER_WORDS[c]} wheel load (kN)", color=INK)
        axp.legend(frameon=False, fontsize=7, loc="upper right")
        lp._style(axp)
    axes[-1].set_xlabel("lap distance (m)", color=INK)
    note = (f"NBR: damper gauge offset per corner from Q2 removed -- level label L4-rel" if f.K else
            "damper chain as configured")
    fig.suptitle(lp._wrap(f"{title_of(key)} -- wheel load per corner from the damper load gauges, every analysis lap "
                          f"(n={len(f.laps)}), corners shown separately; {note}; kerb strikes blanked and bridged dashed",
                          fig), fontsize=10, color=INK)
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(os.path.join(OUT_DIR, f"loads_{key}.png"), dpi=100)
    plt.close(fig)
    # Part 2 fuel drift
    fb = read_raw_blocks(d2.sessions()[key]["path"], ["ecu_fuelcons_kg"]).get("ecu_fuelcons_kg")
    rec = {"key": key, "label": f.label, "speed_bin_kph": f.bin, "tier": {c: str(f.tier[c]) for c in CORNERS}}
    if fb is None:
        rec["fuel"] = "ecu_fuelcons_kg absent"
    else:
        burn = np.interp(f.t, fb[0], fb[1]) - np.interp(f.t[0], fb[0], fb[1])
        rows = []
        for l in f.laps:
            m = f.sb & (f.t >= l["start_time"]) & (f.t <= l["end_time"])
            if m.sum() < 20:
                continue
            rows.append({"lap": l["lap_number"], "burn_kg": float(np.mean(burn[m])),
                         **{c: float(np.median(f.fz[c][m])) for c in CORNERS}})
        rec["laps"] = rows
        slopes = {}
        if len(rows) >= 3:
            bx = np.array([r["burn_kg"] for r in rows])
            for c in CORNERS:
                yc = np.array([r[c] for r in rows])
                slopes[c] = float(np.polyfit(bx, yc, 1)[0])
        rec["slope_N_per_kg"] = slopes
        rec["tracks_burn"] = (bool(sum(s < 0 and abs(s) <= 2 * abs(EXPECTED_N_PER_KG) and abs(s) >= abs(EXPECTED_N_PER_KG) / 2
                                       for s in slopes.values()) >= 3) if slopes else None)
        fig, axes = plt.subplots(1, 4, figsize=(15, 4), constrained_layout=True)
        for axp, c in zip(axes, CORNERS):
            if rows:
                bx = np.array([r["burn_kg"] for r in rows])
                yc = np.array([r[c] for r in rows])
                axp.plot(bx, yc - yc[0], "o-", color=INK, label="measured, per lap")
                axp.plot(bx, (bx - bx[0]) * EXPECTED_N_PER_KG, "--", color=ACC2,
                         label="expected from fuel mass leaving (equal split)")
            axp.set_xlabel("fuel burned (kg)", color=INK)
            axp.set_ylabel("change of straight-line wheel load (N)", color=INK)
            axp.set_title(CORNER_WORDS[c], fontsize=9, loc="left")
            axp.legend(frameon=False, fontsize=7)
            lp._style(axp)
        fig.suptitle(lp._wrap(f"{title_of(key)} -- within-run drift of the straight-line wheel load per corner "
                              f"({f.bin[0]:.0f}-{f.bin[1]:.0f} km/h) vs the drift fuel burn alone would cause", fig),
                     fontsize=10, color=INK)
        fig.savefig(os.path.join(OUT_DIR, f"fueldrift_{key}.png"), dpi=100)
        plt.close(fig)
    rec["ride_height_mm"] = {c: f.ride_height_mm(c)[0] for c in CORNERS}
    rec["ride_height_unit"] = {c: f.ride_height_mm(c)[1] for c in CORNERS}
    if f.K:
        rec["K"] = f.K
    _log(f"[{key}] {f.label}; bin {f.bin}; slopes {rec.get('slope_N_per_kg')}; tracks burn {rec.get('tracks_burn')}; "
         f"ride heights mm {rec['ride_height_mm']} units {rec['ride_height_unit']}")
    _dump(f"file_{key}.json", rec)


# ---------------------------------------------------------------- summary (Parts 3, 4)

def summary():
    from scipy.stats import spearmanr
    keys = nbr_keys()
    R = {k: json.load(open(os.path.join(OUT_DIR, f"file_{k}.json"))) for k in keys}
    outing = {k: int(d2.sessions()[k]["outing"]) for k in keys}
    order = sorted(keys, key=lambda k: outing[k])
    p3 = {}
    for c in ("fr", "rl", "rr"):
        K = [R[k]["K"][c] for k in order]
        rh = [R[k]["ride_height_mm"][c] for k in order]
        ok = [i for i in range(len(order)) if np.isfinite(rh[i])]
        r_rh = spearmanr([K[i] for i in ok], [rh[i] for i in ok]).correlation if len(ok) >= 5 else float("nan")
        r_or = spearmanr(K, [outing[k] for k in order]).correlation
        p3[c] = {"rho_K_rideheight": float(r_rh), "rho_K_outing": float(r_or), "n": len(ok)}
    supported = sum(abs(x["rho_K_rideheight"]) >= 0.6 and abs(x["rho_K_rideheight"]) > abs(x["rho_K_outing"])
                    for x in p3.values()) >= 2
    a, b = R["nbr_jst_problem"], R["nbr_jst_solution"]
    ab = {c: {"dK": b["K"][c] - a["K"][c], "d_ride_height_mm": b["ride_height_mm"][c] - a["ride_height_mm"][c]}
          for c in CORNERS}
    out = {"part3": {"per_corner": p3, "supported": bool(supported), "rake_pair": ab},
           "part2": {k: {"slope_N_per_kg": json.load(open(os.path.join(OUT_DIR, f"file_{k}.json"))).get("slope_N_per_kg"),
                         "tracks_burn": json.load(open(os.path.join(OUT_DIR, f"file_{k}.json"))).get("tracks_burn")}
                     for k in d2.usable_keys()}}
    _log("PART 3 " + json.dumps(out["part3"], default=float))
    # Part 4 render
    plt = lp._plt()
    q = {k: json.load(open(os.path.join(Q2_DIR, f"partB_{k}.json"))) for k in keys}
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, constrained_layout=True)
    for axp, c in zip(axes.ravel(), CORNERS):
        x = np.arange(len(order))
        K = np.array([R[k]["K"][c] for k in order]) / 1000
        u = np.array([q[k]["u_wheel_N"][c] / q[k]["choice"][c]["mr_straight"] for k in order]) / 1000
        axp.errorbar(x, K, yerr=u, fmt="o", color=INK, capsize=3)
        axp.set_xticks(x)
        axp.set_xticklabels([f"{outing[k]}" for k in order], fontsize=7)
        axp.set_title(f"{CORNER_WORDS[c]}: gauge offset per file (spread {K.max() - K.min():.1f} kN)", fontsize=9, loc="left")
        axp.set_ylabel("gauge offset (kN, gauge units)", color=INK)
        lp._style(axp)
    for axp in axes[1]:
        axp.set_xlabel("outing number (chronological)", color=INK)
    fig.suptitle(lp._wrap("Nurburgring, 16 outings -- the damper-gauge offset of each corner file by file, in outing order "
                          "(Q2 estimate, error bar = Q2 uncertainty in gauge units)", fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "offsets_crossfile.png"), dpi=100)
    plt.close(fig)
    # Part 3 render
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    for axp, c in zip(axes, ("fr", "rl", "rr")):
        rh = np.array([R[k]["ride_height_mm"][c] for k in order])
        K = np.array([R[k]["K"][c] for k in order]) / 1000
        axp.plot(rh, K, "o", color=INK)
        for k, xx, yy in zip(order, rh, K):
            axp.annotate(str(outing[k]), (xx, yy), fontsize=6, color=REF)
        axp.set_xlabel(f"{CORNER_WORDS[c]} ride height at matched speed (mm)", color=INK)
        axp.set_ylabel("gauge offset (kN, gauge units)", color=INK)
        axp.set_title(f"rho {p3[c]['rho_K_rideheight']:+.2f} (vs outing order {p3[c]['rho_K_outing']:+.2f})",
                      fontsize=9, loc="left")
        lp._style(axp)
    fig.suptitle(lp._wrap("Nurburgring, 16 outings -- is the damper-gauge offset setup history? Offset per corner vs "
                          "that corner's ride height (front-left laser dead on these files); labels = outing number",
                          fig), fontsize=10, color=INK)
    fig.savefig(os.path.join(OUT_DIR, "offsets_vs_rideheight.png"), dpi=100)
    plt.close(fig)
    _dump("summary.json", out)


if __name__ == "__main__":
    m = sys.argv[1]
    if m == "part0":
        part0()
    elif m == "file":
        file_run(sys.argv[2])
    elif m == "summary":
        summary()
