# Q-DIPS (author order 2026-10-19): where do deep negative cornering
# stiffness ratios come from? Report only; no module, config or record change.
# Shipped fit-L config (NIS-window trigger, hard-bound-only reset mask), the
# 14-session analysis set; session caches shared with derive_cd_threshold.py.
#
# D1 physics floor: pure-lateral Magic Formula, the steepest post-peak slope
#   divided by the initial slope B C D, minimised over E in [-1, 1] for C in
#   [1.0, 3.0]. Independent of B and D (slope in B x, normalised by C).
# D2 dip census per session and axle on the verdict CS samples (canonical
#   brackets on analysis laps, base mask, moving, not kerb, not kerb-affected,
#   not observer-reset-affected): shares below 0 / -0.2 / -1 / -3; for ratio
#   < -1 the cause flags (overlapping): corner phase, |alpha| level at the
#   sample, alpha reversing direction inside the window (more than one
#   monotonic section), axle Fz range in the window > 10 % of its mean,
#   |Fx| / Fz > 0.3 and braking at the sample, a hard-bound reset within 1 s.
#   The cuts are reporting cuts, not rules.
# D3 counterfactual on the same windows and slip angle (window start from
#   reconstruct_cs_window_start, reference taken at the samples where the
#   shipped reference was set): (i) current blend; (ii) chair combination
#   rule (R^2 weight unclipped, no value when no section is valid); (iii)
#   Fy / Fz; (iv) both. Variant (i) must reproduce the module ratio.
# Output diagnostics/plots/cs_dips/ (run.log, dips_<key>.json).
# Run: python -m diagnostics.inspect_cs_dips

import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from diagnostics.derive_cd_threshold import load_cache  # noqa: E402

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "cs_dips")
ANALYSIS = ("nbr_o05_mla", "nbr_o10_jdr", "nbr_o20_jst", "nbr_o29_jst", "nbr_o46_jst", "nbr_o53_mla", "nbr_o54_cdr",
            "nbr_o59_mla", "nbr_o61_jdr", "nbr_o63_cdr", "nbr_qp_cdr", "nbr_qp_mla", "pr_o25_us_v1", "pr_o32_us_v2")
WORKERS = 3
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
FZ_RANGE_CUT = 0.10
UTIL_CUT = 0.3
HB_NEAR_S = 1.0
ALPHA_BINS_RAD = (0.03, 0.07)


def physics_floor():
    phi = np.concatenate(([0.0], np.logspace(-3, 4, 40000)))
    out = {}
    for c in np.round(np.arange(1.0, 3.0001, 0.1), 2):
        best = (np.inf, None)
        for e in np.round(np.arange(-1.0, 1.0001, 0.01), 2):
            th = phi - e * (phi - np.arctan(phi))
            dth = (1 - e) + e / (1 + phi ** 2)
            s = np.cos(c * np.arctan(th)) * dth / (1 + th ** 2)
            if s.min() < best[0]:
                best = (float(s.min()), float(e))
        out[float(c)] = {"min_ratio": best[0], "at_E": best[1]}
    return out


def _brackets(c):
    import modules.tyre_fit_auto as tfa
    st, t = c["state"], c["state"]["time"]
    laps = {l["lap_number"]: l for l in c["laps"]}
    in_br = np.zeros(t.size, dtype=bool)
    for k in c["corners"]:
        lap = laps.get(k.get("lap_number"))
        if k.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if k.get("bracket_start_m") is None or k.get("bracket_end_m") is None:
            continue
        sl = tfa._canonical_window_slice(t, st["s_m"], lap["start_time"], lap["end_time"],
                                         k["bracket_start_m"], k["bracket_end_m"])
        in_br[sl] = True
    return in_br


def _phase_of(c):
    t = c["state"]["time"]
    ph = np.full(t.size, "outside phases", dtype=object)
    valid = {l["lap_number"] for l in c["laps"] if l.get("is_valid_for_analysis")}
    for k in c["corners"]:
        if k.get("lap_number") not in valid:
            continue
        for name in PHASES:
            a, b = k["segments"][name]
            if b >= a:
                ph[(t >= a) & (t <= b)] = name
    return ph


def _blend(c_w, r2, c_s, chair):
    if chair:
        # chair: R^2 weight unclipped (order 1 gives w = R^2), no value
        # without a valid section estimate
        return np.nan if np.isnan(c_s) else r2 * c_w + (1 - r2) * c_s
    from modules.stability_analysis import _smooth_weight, R2_WEIGHT_EXPONENT
    if np.isnan(c_s):
        return c_w
    w = _smooth_weight(r2, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
    return w * c_w + (1 - w) * c_s


def _window_terms(alpha, y, start, i, sections, section_id, sec_slopes, sec_spans, min_span):
    from modules.stability_analysis import _section_slope_and_span, _smooth_weight, SPAN_WEIGHT_EXPONENT
    wa, wy = alpha[start:i], y[start:i]
    am, ym = wa.mean(), wy.mean()
    den = np.sum((wa - am) ** 2)
    c_w = np.sum((wa - am) * (wy - ym)) / den
    fit = c_w * wa + (ym - c_w * am)
    ss_tot = np.sum((wy - ym) ** 2)
    r2 = 1.0 - np.sum((wy - fit) ** 2) / ss_tot if ss_tot > 1e-10 else 0.0
    ws, ss = [], []
    for k in range(section_id[start], section_id[i - 1] + 1):
        s, e = sections[k]
        lo, hi = max(s, start), min(e, i)
        if lo == s and hi == e:
            sl, sp = sec_slopes[k], sec_spans[k]
        else:
            sl, sp = _section_slope_and_span(alpha[lo:hi], y[lo:hi])
        if np.isnan(sl):
            continue
        w = _smooth_weight(sp, 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
        if w <= 0:
            continue
        ws.append(w)
        ss.append(sl)
    c_s = float(np.sum(np.array(ws) * np.array(ss)) / np.sum(ws)) if ws else np.nan
    return c_w, r2, c_s


def run_session(key):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, reconstruct_cs_window_start,
                                            resolve_cs_min_window_samples, _find_monotonic_sections, _section_slopes)
    c, t_build = load_cache(key)
    out = {"key": key, "label": c["label"], "build_s": t_build, "fallback": c["fallback"], "axles": {}}
    if c["fallback"]:
        out["note"] = f"shipped chain fell back ({c['fallback_reason']}) -- kinematic beta, no reset mask"
    st, p, sr = c["state"], c["params"], c["state"]["sample_rate_hz"]
    se = p["stability_estimation"]
    slip = estimate_slip_angles(st, c["beta_shipped"], p)
    forces = estimate_lateral_forces(st, p)
    reset = None if c["fallback"] else c["observer_reset_mask"]
    cs = estimate_cornering_stiffness(slip, forces, st, p, c["laps"], reset_mask=reset)
    in_br, phase = _brackets(c), _phase_of(c)
    if c["base_mask"] is None:
        import modules.tyre_fit_auto as tfa
        c["base_mask"] = tfa._base_mask(st, c["laps"])
    kerb = st.get("kerb_mask")
    min_window = resolve_cs_min_window_samples(p, sr)
    min_span, max_m = se["cs_min_slip_angle_span_rad"], se["cs_max_window_m"]
    hb_t = st["time"][reset] if reset is not None else np.array([])
    li = c["load_in"]
    for ax, fz, fx in (("f", li["fz_f_N"], li["fx_f_N"]), ("r", li["fz_r_N"], li["fx_r_N"])):
        alpha, fy = slip[f"alpha_{ax}_filt"], forces[f"Fy_{ax}_filt"]
        ratio = cs[f"CS_ratio_{ax}"]
        pop = in_br & c["base_mask"] & st["moving_mask"] & ~cs[f"near_strike_{ax}"] & np.isfinite(ratio)
        if kerb is not None:
            pop &= ~kerb
        if reset is not None:
            pop &= ~cs[f"observer_reset_affected_{ax}"]
        r = ratio[pop]
        rec = {"n_verdict": int(pop.sum())}
        for thr in (0, -0.2, -1, -3):
            rec[f"share_lt_{thr}"] = float((r < thr).mean()) if r.size else np.nan
        sections, section_id = _find_monotonic_sections(alpha)
        ys = {"fy": fy, "norm": fy / fz}
        sec = {k: _section_slopes(alpha, y, sections) for k, y in ys.items()}
        # reference set events of the shipped run: where the reference in
        # effect changes value. The module leaves NaN on samples it skips, so
        # forward-fill first -- a value reappearing after a gap is not a new
        # reference
        lref = cs[f"C_linear_ref_{ax}"]
        fin_idx = np.where(np.isfinite(lref), np.arange(lref.size), -1)
        ff = np.maximum.accumulate(fin_idx)
        lref_ff = np.where(ff >= 0, lref[np.maximum(ff, 0)], np.nan)
        prev = np.concatenate(([np.nan], lref_ff[:-1]))
        set_idx = np.flatnonzero(np.isfinite(lref) & (np.isnan(prev) | (lref != prev)))
        deep = np.flatnonzero(pop & (ratio < -1))
        need = np.union1d(np.flatnonzero(pop), set_idx)
        var = {v: {} for v in ("i", "ii", "iii", "iv")}
        starts = {}
        for i in need:
            s0 = reconstruct_cs_window_start(alpha, i, min_window, min_span, st.get("s_m"), max_m)
            starts[i] = s0
            for yk, chair, name in (("fy", False, "i"), ("fy", True, "ii"), ("norm", False, "iii"), ("norm", True, "iv")):
                c_w, r2, c_s = _window_terms(alpha, ys[yk], s0, i, sections, section_id, *sec[yk], min_span)
                var[name][i] = _blend(c_w, r2, c_s, chair)
        # reference per verdict sample = the variant's value at the last set event
        pos = np.searchsorted(set_idx, np.flatnonzero(pop), side="right") - 1
        counts, repro = {}, None
        for name in var:
            vals = []
            for i, j in zip(np.flatnonzero(pop), pos):
                ref = var[name].get(set_idx[j]) if j >= 0 else np.nan
                x = var[name][i]
                vals.append(min(x / ref, 1.0) if (ref == ref and ref > 0 and x == x) else np.nan)
            vals = np.array(vals)
            if name == "i":
                repro = float(np.nanmax(np.abs(vals - ratio[pop]))) if vals.size else np.nan
            fin = np.isfinite(vals)
            counts[name] = {"n_finite": int(fin.sum()), "lt_-1": int((vals[fin] < -1).sum()),
                            "lt_-0.2": int((vals[fin] < -0.2).sum())}
        rec["counterfactual"] = counts
        rec["variant_i_max_abs_dev_from_module"] = repro
        # cause flags on ratio < -1
        flags = {"alpha_reversal": 0, "fz_range_gt_10pct": 0, "util_gt_0p3": 0, "braking": 0,
                 "hard_bound_within_1s": 0}
        ph_count = {}
        a_bins = {"lt_0.03": 0, "0.03_0.07": 0, "gt_0.07": 0}
        hb_dist = []
        for i in deep:
            s0 = starts.get(i) or reconstruct_cs_window_start(alpha, i, min_window, min_span, st.get("s_m"), max_m)
            ph_count[phase[i]] = ph_count.get(phase[i], 0) + 1
            aa = abs(alpha[i])
            a_bins["lt_0.03" if aa < ALPHA_BINS_RAD[0] else ("0.03_0.07" if aa < ALPHA_BINS_RAD[1] else "gt_0.07")] += 1
            flags["alpha_reversal"] += int(section_id[s0] != section_id[i - 1])
            wz = fz[s0:i]
            flags["fz_range_gt_10pct"] += int((wz.max() - wz.min()) / wz.mean() > FZ_RANGE_CUT)
            u = abs(fx[i]) / fz[i]
            flags["util_gt_0p3"] += int(u > UTIL_CUT)
            flags["braking"] += int(fx[i] < 0)
            if hb_t.size:
                dmin = float(np.min(np.abs(hb_t - st["time"][i])))
                hb_dist.append(dmin)
                flags["hard_bound_within_1s"] += int(dmin <= HB_NEAR_S)
        nd = max(1, len(deep))
        rec["deep"] = {"n": int(len(deep)), "phase_share": {k: v / nd for k, v in ph_count.items()},
                       "alpha_bin_share": {k: v / nd for k, v in a_bins.items()},
                       "flag_share": {k: v / nd for k, v in flags.items()},
                       "hb_dist_p50_s": float(np.median(hb_dist)) if hb_dist else np.nan}
        out["axles"][ax] = rec
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
        floor = physics_floor()
        log("D1 Magic Formula post-peak slope floor (steepest slope / initial slope, minimised over E in [-1, 1]):")
        for cc, v in floor.items():
            log(f"  C {cc:.1f}: {v['min_ratio']:.4f} at E {v['at_E']:+.2f}")
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            results = list(ex.map(run_session, ANALYSIS))
        log(f"wall time {time.perf_counter() - t0:.0f} s, {WORKERS} workers")
        for r in results:
            log("=" * 100)
            log(f"{r['label']}: cache build {r['build_s']:.0f} s" + (f"; {r['note']}" if "note" in r else ""))
            for ax in ("f", "r"):
                x = r["axles"][ax]
                d = x["deep"]
                log(f"  {'front' if ax == 'f' else 'rear'}: verdict n {x['n_verdict']}; share < 0 {x['share_lt_0']:.4f}, "
                    f"< -0.2 {x['share_lt_-0.2']:.4f}, < -1 {x['share_lt_-1']:.4f}, < -3 {x['share_lt_-3']:.4f}")
                log(f"     ratio < -1 (n {d['n']}): phases {json.dumps({k: round(v, 3) for k, v in d['phase_share'].items()})}; "
                    f"|alpha| {json.dumps({k: round(v, 3) for k, v in d['alpha_bin_share'].items()})}; flags "
                    f"{json.dumps({k: round(v, 3) for k, v in d['flag_share'].items()})}; nearest hard-bound reset p50 "
                    f"{d['hb_dist_p50_s']:.1f} s")
                cf = x["counterfactual"]
                log(f"     counterfactual (n finite / < -1 / < -0.2): " + "; ".join(
                    f"({k}) {v['n_finite']} / {v['lt_-1']} / {v['lt_-0.2']}" for k, v in cf.items())
                    + f"; variant (i) vs module max abs dev {x['variant_i_max_abs_dev_from_module']:.2e}")
        with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable({"floor": {str(k): v for k, v in floor.items()}, "results": results}), fh, indent=1,
                      default=str)
        return 0
    finally:
        keep_awake(False)
        log_f.close()


if __name__ == "__main__":
    sys.exit(main())
