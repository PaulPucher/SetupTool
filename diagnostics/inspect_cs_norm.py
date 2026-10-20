# CS on load-normalised axle force, first run (author EXECUTE 2026-10-20).
# Fit L with the shipped config (Q-UPPER, hard bound only, measured R floors,
# g2 at source precision) on the 14-session analysis set; read-only. No
# verdict statistics (thresholds not yet re-derived).
#
# Per session: status, g2 zone and K per axle. Per session and axle: the
# linear-reference candidates of the normalised estimate (p5 / p50 / p95 per
# rad) and the share the 6-30 band excludes; the CS ratio distribution
# (shares < 0, < -0.2, < -0.53, < -1, < -3) for Fy / Fz next to raw Fy on the
# same samples (verdict population: canonical brackets on analysis laps, base
# mask, moving, not kerb, not kerb-affected, not observer-reset-affected,
# finite in both). Plausibility (author criterion): per analysis lap the
# number of normalised samples below -0.53, and the corner / phase cells
# holding them -- cells on >= half the laps listed, cells on one lap counted.
# Figure: NBR 29, the analysis lap of median lap time, CS ratio raw vs Fy / Fz
# over lap distance, front and rear, corner phases shaded, -0.53 floor drawn.
# Output diagnostics/plots/cs_norm/ (run.log, results.json, PNG).
# Run: python -m diagnostics.inspect_cs_norm

import copy
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "cs_norm")
ANALYSIS = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
            "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
FIG_SESSION = "nbr_o29_jst"
WORKERS = 3
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
PHASE_WORDS = {"entry_1_brake": "entry, braking", "entry_2_turnin": "entry, turn-in", "apex_3": "apex",
               "exit_4": "exit, first part", "exit_5": "exit, second part"}
FLOOR = -0.53   # pure-lateral Magic Formula at C = 3.0 (ruling 2026-10-19, a label)
THRESHOLDS = (0.0, -0.2, FLOOR, -1.0, -3.0)


def run_session(key):
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    p["tyre_fit_auto"]["tyre_fit_mode"] = "literature_bounded"
    st = prepare_vehicle_state(data["channels"], p)
    held = {}
    o_fsp = tfa.fit_session_pacejka

    def fsp(*a, **kw):
        held["raw"] = o_fsp(*a, **kw)
        return held["raw"]

    tfa.fit_session_pacejka = fsp
    try:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(st, p, data, "ekf_auto_pacejka", csv_path=path)
    finally:
        tfa.fit_session_pacejka = o_fsp
    raw = held["raw"]
    rec = {"key": key, "label": label, "status": man.get("status"),
           "reason": man.get("degenerate_reason") or man.get("marginal_reason"), "fallback": fb,
           "fallback_reason": fr, "gate": gate and {k: gate[k] for k in ("verdict", "health_score")},
           "g2": {a: {k: raw["plausibility_gate"][a][k] for k in ("stiffness_norm_per_rad", "k_zone", "failed")}
                  for a in ("front", "rear")}, "axles": {}}
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    reset = None if fb else raw.get("observer_reset_mask")
    laps = data["laps"]
    cs_raw = estimate_cornering_stiffness(slip, forces, st, p, laps, reset_mask=reset)
    cs_n = estimate_cornering_stiffness(slip, forces, st, p, laps, reset_mask=reset,
                                        axle_loads=raw["cs_axle_loads"], ref_band=tfa.L_K_BAND)
    t = st["time"]
    lap_of = {l["lap_number"]: l for l in laps}
    an_laps = [l["lap_number"] for l in laps if l.get("is_valid_for_analysis")]
    in_br = np.zeros(t.size, dtype=bool)
    cell_masks = []   # (stable id, phase, lap, mask)
    for c in data["corners"]:
        lap = lap_of.get(c.get("lap_number"))
        if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            sl = tfa._canonical_window_slice(t, st["s_m"], lap["start_time"], lap["end_time"],
                                             c["bracket_start_m"], c["bracket_end_m"])
            in_br[sl] = True
        for ph in PHASES:
            a, b = c["segments"][ph]
            if b >= a:
                cell_masks.append((str(c["stable_corner_id"]), ph, c["lap_number"], (t >= a) & (t <= b)))
    kerb = st.get("kerb_mask")
    base = raw["base_mask"]
    for ax in ("f", "r"):
        rc, n_c, n_x = cs_n[f"ref_candidates_{ax}"], cs_n[f"n_ref_candidates_{ax}"], cs_n[f"n_ref_band_excluded_{ax}"]
        pop = in_br & base & st["moving_mask"] & np.isfinite(cs_raw[f"CS_ratio_{ax}"]) & np.isfinite(cs_n[f"CS_ratio_{ax}"])
        for cs in (cs_raw, cs_n):
            pop &= ~cs[f"near_strike_{ax}"]
            if reset is not None:
                pop &= ~cs[f"observer_reset_affected_{ax}"]
        if kerb is not None:
            pop &= ~kerb
        r_raw, r_n = cs_raw[f"CS_ratio_{ax}"][pop], cs_n[f"CS_ratio_{ax}"][pop]
        below = pop & (cs_n[f"CS_ratio_{ax}"] < FLOOR)
        per_lap, cells = {}, {}
        lap_id = np.full(t.size, -1)
        for ln in an_laps:
            lap_id[(t >= lap_of[ln]["start_time"]) & (t <= lap_of[ln]["end_time"])] = ln
            per_lap[ln] = int((below & (lap_id == ln)).sum())
        in_cell = np.zeros(t.size, dtype=bool)
        for cid, ph, ln, m in cell_masks:
            in_cell |= m
            if (below & m).any():
                cells.setdefault((cid, ph), set()).add(ln)
        half = math.ceil(len(an_laps) / 2)
        rec["axles"][ax] = {
            "ref": {"n_candidates": n_c, "n_excluded": n_x, "excluded_share": n_x / n_c if n_c else float("nan"),
                    "p5": float(np.percentile(rc, 5)) if rc.size else None,
                    "p50": float(np.percentile(rc, 50)) if rc.size else None,
                    "p95": float(np.percentile(rc, 95)) if rc.size else None},
            "n_pop": int(pop.sum()),
            "shares_norm": {str(x): float((r_n < x).mean()) for x in THRESHOLDS},
            "shares_raw": {str(x): float((r_raw < x).mean()) for x in THRESHOLDS},
            "below_floor_per_lap": per_lap, "below_floor_outside_phases": int((below & ~in_cell).sum()),
            "n_laps": len(an_laps),
            "cells_repeated": sorted([(cid, ph, len(v)) for (cid, ph), v in cells.items() if len(v) >= half],
                                     key=lambda x: (-x[2], x[0], x[1])),
            "n_cells_once": sum(1 for v in cells.values() if len(v) == 1), "n_cells_total": len(cells)}
    if key == FIG_SESSION:
        times = {ln: lap_of[ln]["end_time"] - lap_of[ln]["start_time"] for ln in an_laps}
        ln = sorted(an_laps, key=lambda k: times[k])[len(an_laps) // 2]
        m = (t >= lap_of[ln]["start_time"]) & (t <= lap_of[ln]["end_time"])
        rec["figure"] = {"lap": ln, "lap_time_s": times[ln], "s_m": st["s_m"][m].tolist(),
                         "raw_f": cs_raw["CS_ratio_f"][m].tolist(), "norm_f": cs_n["CS_ratio_f"][m].tolist(),
                         "raw_r": cs_raw["CS_ratio_r"][m].tolist(), "norm_r": cs_n["CS_ratio_r"][m].tolist(),
                         "phases": [(ph, float(np.interp(c["segments"][ph][0], t, st["s_m"])),
                                     float(np.interp(c["segments"][ph][1], t, st["s_m"])))
                                    for c in data["corners"] if c.get("lap_number") == ln
                                    for ph in PHASES if c["segments"][ph][1] >= c["segments"][ph][0]]}
    return rec


def draw(fig_rec, label):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    blue, orange, ink, muted = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e"
    greys = {"entry_1_brake": "#d9d9d6", "entry_2_turnin": "#e6e6e3", "apex_3": "#c9c9c5",
             "exit_4": "#ededeb", "exit_5": "#f3f3f1"}
    s = np.asarray(fig_rec["s_m"])
    fig, axes = plt.subplots(2, 1, figsize=(15, 8.5), sharex=True, facecolor="#fcfcfb")
    lo = -3.0
    for axp, (name, kr, kn) in zip(axes, (("Front axle", "raw_f", "norm_f"), ("Rear axle", "raw_r", "norm_r"))):
        axp.set_facecolor("#fcfcfb")
        for ph, a, b in fig_rec["phases"]:
            axp.axvspan(a, b, color=greys[ph], lw=0, zorder=0)
        r, n = np.asarray(fig_rec[kr], dtype=float), np.asarray(fig_rec[kn], dtype=float)
        axp.plot(s, np.clip(r, lo, None), color=blue, lw=1.2, label="raw axle force Fy (current estimate)", zorder=2)
        axp.plot(s, np.clip(n, lo, None), color=orange, lw=1.2, label="load-normalised force Fy / Fz", zorder=3)
        axp.axhline(FLOOR, color=muted, lw=1.0, ls="--", zorder=1)
        axp.text(s[np.isfinite(s)].min() + 20, FLOOR - 0.22, "physics floor -0.53 (pure-lateral Magic Formula, C = 3.0)",
                 color=muted, fontsize=8)
        n_off = int(np.sum(r < lo)), int(np.sum(n < lo))
        axp.set_ylim(lo - 0.1, 1.1)
        axp.set_ylabel("cornering stiffness ratio (-)", color=ink)
        axp.set_title(f"{name} (values below -3 drawn at -3: raw {n_off[0]}, normalised {n_off[1]} samples)",
                      color=ink, fontsize=10, loc="left")
        axp.grid(axis="y", color="#e4e4e0", lw=0.6)
        for sp in ("top", "right"):
            axp.spines[sp].set_visible(False)
    axes[1].set_xlabel("lap distance (m)", color=ink)
    handles = axes[0].get_legend_handles_labels()[0] + [Patch(color=greys[p], label=f"corner phase: {PHASE_WORDS[p]}")
                                                        for p in PHASES]
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=8, frameon=False)
    fig.suptitle(f"{label}, analysis lap {fig_rec['lap']} (median lap time, {fig_rec['lap_time_s']:.1f} s): "
                 "cornering stiffness ratio from raw axle force vs load-normalised axle force", color=ink, fontsize=11)
    fig.tight_layout(rect=(0, 0.07, 1, 0.96))
    out = os.path.join(OUT_DIR, f"nbr29_lap{fig_rec['lap']}_cs_ratio_raw_vs_normalised.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main():
    from diagnostics.inspect_fit_lit import keep_awake, _jsonable
    os.makedirs(OUT_DIR, exist_ok=True)
    log_f = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")

    def log(msg=""):
        print(msg, flush=True)
        log_f.write(msg + "\n")
        log_f.flush()

    keep_awake(True)
    t0 = time.perf_counter()
    try:
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            results = list(ex.map(run_session, ANALYSIS))
        log(f"wall time {time.perf_counter() - t0:.0f} s, {WORKERS} workers")
        for r in results:
            log("=" * 100)
            log(f"{r['label']}: status {r['status']} ({r['reason'] or '-'}); fallback {r['fallback']}; NIS gate "
                f"{r['gate']}; g2 front K {r['g2']['front']['stiffness_norm_per_rad']:.2f} ({r['g2']['front']['k_zone']}), "
                f"rear K {r['g2']['rear']['stiffness_norm_per_rad']:.2f} ({r['g2']['rear']['k_zone']})")
            for ax in ("f", "r"):
                x = r["axles"][ax]
                rf = x["ref"]
                log(f"  {'front' if ax == 'f' else 'rear'}: reference candidates {rf['n_candidates']}, p5 / p50 / p95 "
                    f"{rf['p5']:.2f} / {rf['p50']:.2f} / {rf['p95']:.2f} per rad, band 6-30 excludes "
                    f"{rf['n_excluded']} ({rf['excluded_share'] * 100:.1f} %)")
                log(f"     ratio shares on {x['n_pop']} samples, Fy / Fz vs raw Fy: " + "; ".join(
                    f"< {k}: {x['shares_norm'][k] * 100:.2f} % vs {x['shares_raw'][k] * 100:.2f} %" for k in x["shares_norm"]))
                log(f"     below -0.53 per analysis lap: {x['below_floor_per_lap']} (outside phases "
                    f"{x['below_floor_outside_phases']}); cells on >= half of {x['n_laps']} laps: {x['cells_repeated']}; "
                    f"cells on one lap only {x['n_cells_once']} of {x['n_cells_total']}")
        fig_rec = next((r["figure"] for r in results if "figure" in r), None)
        if fig_rec:
            log(f"figure: {draw(fig_rec, next(r['label'] for r in results if 'figure' in r))}")
        mx = max(r["axles"][ax]["ref"]["excluded_share"] for r in results for ax in ("f", "r"))
        log(f"largest band exclusion share: {mx * 100:.1f} % -> " + ("ABOVE 10 %: report and wait" if mx > 0.10 else
                                                                    "within 10 % on every session and axle"))
        for r in results:
            r.pop("figure", None)
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable(results), fh, indent=1, default=str)
        return 0
    finally:
        keep_awake(False)
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
