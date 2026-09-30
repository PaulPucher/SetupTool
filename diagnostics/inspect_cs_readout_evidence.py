# DIAGNOSTIC, read-only, [keep-reproduces]. Evidence and prototype for the
# CS read-out decision: slip-vs-force lag check (Part 1), fold/loop at run
# scale (Part 2), sawtooth window-start repair measured in isolation
# (Part 3), pooled read-out prototype scored against production (Part 4).
#
# Builds on the caches of inspect_new_laps_cs_sweep.py (production chain,
# ekf_auto_pacejka) and the ay/Fy supplement of inspect_cs_value_lens.py.
# Nothing in modules/ or config/ changes; prototypes run in memory.
# Tier B (signal processing, read-out statistics) throughout.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_cs_readout_evidence supp2 <key>
#   python -m diagnostics.inspect_cs_readout_evidence lag
#
# DEFINITIONS (fixed before the runs):
# Part 1 lag. Signals: filtered axle slip angle and filtered axle Fy (both
#   through the production 2 Hz zero-phase Butterworth), moving and
#   non-kerb samples of one traversal (canonical bracket, one analysis
#   lap). Lag k* = argmax over k in [-50, +50] samples of the Pearson
#   correlation of alpha(t) with Fy(t + k). k* > 0: force follows slip
#   (Fy lags alpha); k* < 0: slip follows force. Reported in samples, ms
#   (10 ms grid) and metres (k* x mean speed x 10 ms). Phase split: entry
#   (entry_1 start .. entry_2 end), apex (+/- cs_apex_region_half_length_m
#   around the apex), exit (exit_4 start .. exit_5 end); segments with
#   fewer than 40 samples are skipped, search +/- 30 samples there.
# Part 1 consequence. Per traversal and axle: a synthetic LINEAR tyre
#   driven by the real slip trace, Fy_syn(t) = C * alpha(t - k*), is run
#   through the production window rule (reconstruct_cs_window_start, real
#   floors and 53 m cap) and a plain OLS slope; CS_syn = slope / C. Any
#   negative CS_syn is manufactured by the lag alone (the tyre has no
#   peak). "Explained" negative sample = production CS < 0 and CS_syn < 0
#   at the same sample.
# Controls: per session the 3 corners with the highest median front CS
#   over analysis laps among corners whose samples sit at the 1.0 clip
#   less than half the time.
# Part 2 run-scale fold/loop. Run = lap-distance span [s_w - 60 m,
#   s_w + 40 m] around the traversal's minimum-CS sample s_w (the worst
#   window lies behind s_w; 100 m covers it and its neighbours), same lap,
#   moving samples. Criteria made explicit from the 2026-09-02 ground-truth
#   reads: LOOP = the (alpha, Fy) path crosses itself (self-crossing count
#   >= 1); FOLD = no crossing and, after the |Fy| maximum, |Fy| falls by
#   >= 5 % of that maximum while |alpha| grows by >= the span floor
#   (0.02 rad, the method's own resolution); AT-PEAK = no crossing, interior
#   |Fy| maximum, no such post-peak arm; RISING = |Fy| maximum at the run
#   end. Lag correction: Fy shifted by the traversal's Part 1 lag (Fy(t+k)
#   aligned to alpha(t)) and re-read. Final label = corrected read; the
#   instance is LAG-ARTIFACT when the uncorrected read was fold or loop and
#   the corrected read is neither.
# Part 3 repair rule. Production takes, at every sample i, the SHORTEST
#   window ending at i that meets the span floor, so the start is pinned on
#   the old alpha extreme that sets the span and leaps forward in one step
#   when a nearer sample takes over. Repaired start:
#     start_rep(i) = max(cap limit, min(start_prod(i), start_rep(i-1) + 2))
#   i.e. the start may advance at most 2 samples per step, so the window
#   shrinks by at most one sample per step and grows as before. (At most 1
#   would freeze the length: end and start then both move 1 per step and
#   the window could never shrink -- it would ratchet to its longest
#   length.) Because start_rep <= start_prod, the repaired window always
#   CONTAINS the production window, so its alpha span is at least as large
#   and a sample qualifies exactly when it qualifies in production; the
#   53 m cap still bounds it. History resets wherever the previous sample
#   produced no window. The regression, section blend, linear reference
#   and CS_ratio are a line-for-line copy of compute_cs_for_axle; the
#   unrepaired path is recomputed in the same pass and checked against the
#   cached production CS (parity).
# Part 3 noise floor for "negligible": the shift of per-lap phase medians
#   between the two exports of the same outing (v3 vs US-v1, 27 ms
#   re-gridding, zero information content). Repair = cosmetic when
#   qualifying membership is identical and its median and p90 |shift| of
#   phase medians do not exceed that floor; otherwise an estimator change.
# Part 4 read-outs, per corner and phase (apex_3 from apex_region), over a
#   set of analysis laps L. Per-lap values = production phase medians
#   (summarise_corners slicing and n >= cs_phase_min_valid_samples gate):
#   PROD   = min over laps (production worst_lap);
#   REPEAT = second-smallest per-lap value (the worst value reached on >= 2
#            laps; NaN with fewer than 2 finite laps);
#   POOLED = median of all finite samples of that phase pooled over L
#            (primary); POOLED25 = their 25th percentile (conservative).
#   Stability medians stay the production ones. Verdicts through the
#   production classifier with the CURRENT thresholds (derived for PROD's
#   distribution -- re-derivation is exactly the cost criterion d measures).
#   Uncertainty = leave-one-lap-out jackknife standard error of the read-out.
#   RESOLVED (criterion a) = |value_B - value_A| > 2 x sqrt(SE_A^2 + SE_B^2),
#   for the apex front value and the apex gap (r - f) separately.
#   Call stability (criterion c) = split-half agreement: odd vs even analysis
#   laps, sessions with >= 4 laps, share of corners with the same call
#   (understeer / oversteer / none) in both halves; applied to all read-outs
#   so they compare on one metric (PROD also keeps its lap-pair rate).
#   Lap-count bias (f) = PR US-v2, share of corners beyond the current STRONG
#   thresholds (corner worst phase), all 9 laps vs mean over 3-lap windows.
# Part 5 side-by-side render (2026-09-30, decision input for the unparking
#   ruling). Standard trace set (trace_render) from the saw_<key>.npz arrays,
#   production vs repaired, same axes. Jump = the value-lens definition:
#   adjacent in-bracket moving samples (term_idx) whose footprint
#   s[i-1] - s[start] shrinks by more than 2 m; marked at the later sample.
#   Spike, per analysis lap and axle, on in-bracket moving samples, within
#   index-contiguous finite runs: drop(i) = max CS over the samples j <= i
#   with s[i] - s[j] <= 5 m, minus CS(i) (a "within metres" fall). Spikes are
#   the local maxima of drop, one per 20 m (greedy, deepest first); the five
#   deepest per session over both axles are listed. Repaired depth = the same
#   drop on the repaired array, maximised over samples within +/- 5 m of the
#   production spike (same lap, axle). Jump-coincident = a production jump
#   sample lies inside the spike's drop span [j*, i].
#   Spike and window lengths are display-selection choices of this
#   diagnostic, not estimator parameters.

import os
import pickle
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import SESSIONS, load_all
from diagnostics.inspect_cs_value_lens import LABELS, _load_supp, _valid_idx, _by_corner

OUT_DIR = "diagnostics/plots/cs_readout_evidence"
KIN_KEYS = ["dubai", "nbr_jst_solution", "nbr_jst_problem"]
ALL_KEYS = ["dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "lmn_mla", "nbr_qp_cdr", "nbr_qp_mla"]


def supp2(key):
    """Corner phase segments (the sweep cache kept brackets only) and, for
    the lag sessions, slip angles under the kinematic beta."""
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_sideslip,
                                            estimate_slip_angles)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))
    data = parse_csv(SESSIONS[key])
    corners = [{k: c.get(k) for k in ("lap_number", "stable_corner_id", "segments", "apex_time",
                                       "apex_lap_distance_m", "bracket_start_m", "bracket_end_m", "speed_class")}
               for c in data.get("corners", [])]
    out = {"corners": corners}
    if key in KIN_KEYS:
        state = prepare_vehicle_state(data["channels"], params)
        slip = estimate_slip_angles(state, estimate_sideslip(state, params), params)
        out["alpha_f_kin"] = slip["alpha_f_filt"].astype(np.float32)
        out["alpha_r_kin"] = slip["alpha_r_filt"].astype(np.float32)
        out["time"] = state["time"]
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"supp2_{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"{key}: {len(corners)} corners saved" + (", kinematic alpha saved" if key in KIN_KEYS else ""))


def load_supp2(key):
    with open(os.path.join(OUT_DIR, f"supp2_{key}.pkl"), "rb") as fh:
        return pickle.load(fh)


def xcorr_lag(a, f, max_lag):
    """Lag (samples) at the peak of corr(alpha(t), Fy(t+k))."""
    n = a.size
    if n < 3 * max_lag // 2 or np.std(a) == 0 or np.std(f) == 0:
        return None, None
    best_k, best_c = None, -np.inf
    for k in range(-max_lag, max_lag + 1):
        if k >= 0:
            x, y = a[:n - k], f[k:]
        else:
            x, y = a[-k:], f[:n + k]
        if x.size < 20 or np.std(x) == 0 or np.std(y) == 0:
            continue
        c = np.corrcoef(x, y)[0, 1]
        if c > best_c:
            best_k, best_c = k, c
    return best_k, best_c


def _contiguous(idx):
    # the longest contiguous block of valid indices: correlation needs an
    # unbroken time series
    if idx.size == 0:
        return idx
    breaks = np.where(np.diff(idx) != 1)[0]
    starts = np.concatenate(([0], breaks + 1))
    stops = np.concatenate((breaks + 1, [idx.size]))
    j = int(np.argmax(stops - starts))
    return idx[starts[j]:stops[j]]


def synthetic_cs(alpha, s_m, lo, hi, k, min_window, min_span, max_window_m):
    """CS a linear tyre would read if its force lagged the real slip trace
    by k samples. Returns an array over [lo, hi) (NaN where no window)."""
    from modules.stability_analysis import reconstruct_cs_window_start
    fy = np.full(alpha.size, np.nan)
    # linear tyre, C = 1: the ratio is scale-free
    src = np.arange(alpha.size) - k
    ok = (src >= 0) & (src < alpha.size)
    fy[ok] = alpha[src[ok]]
    out = np.full(hi - lo, np.nan)
    for i in range(max(lo, min_window), hi):
        st = reconstruct_cs_window_start(alpha, i, min_window, min_span, s_m=s_m, max_window_m=max_window_m)
        wa, wf = alpha[st:i], fy[st:i]
        if wa.size < 3 or np.max(wa) - np.min(wa) < min_span or not np.all(np.isfinite(wf)):
            continue
        da = wa - wa.mean()
        out[i - lo] = np.sum(da * (wf - wf.mean())) / np.sum(da * da)
    return out


def _controls(r):
    csf = r["cs"]["A_baseline"][0].astype(float)
    rows = []
    for cid, trs in _by_corner(r).items():
        v = np.concatenate([csf[_valid_idx(r, t)] for t in trs])
        v = v[np.isfinite(v)]
        if v.size < 50 or np.mean(v >= 1.0) >= 0.5:
            continue
        rows.append((float(np.median(v)), cid))
    return [cid for _, cid in sorted(rows, reverse=True)[:3]]


def lag():
    from modules.stability_analysis import load_parameters
    se = load_parameters()["stability_estimation"]
    half = se["cs_apex_region_half_length_m"]
    res = load_all()
    dt = 0.01
    report = {}
    # ---- 1-3: lag per corner/lap/phase, both beta sources, focus + controls
    for key in ("dubai", "nbr_jst_solution"):
        r = res[key]
        supp = _load_supp(key, r)
        s2 = load_supp2(key)
        t = r["time"]
        v = supp["v"]
        focus = 4 if key == "dubai" else 1
        ctrl = _controls(r)
        print(f"\n## {LABELS[key]}: focus C{focus}, controls {['C%d' % c for c in ctrl]}")
        seg_by = {(c["stable_corner_id"], c["lap_number"]): c for c in s2["corners"] if c["stable_corner_id"] is not None}
        for cid in [focus] + ctrl:
            for tr in sorted(_by_corner(r)[cid], key=lambda x: x["lap"]):
                for axle in ("f", "r"):
                    fy = supp["fy_" + axle]
                    line = []
                    for src, alpha in (("ekf", r["alpha_" + axle].astype(float)),
                                       ("kin", s2["alpha_%s_kin" % axle].astype(float))):
                        idx = _contiguous(_valid_idx(r, tr))
                        k, c = xcorr_lag(alpha[idx], fy[idx], 50)
                        if k is None:
                            line.append(f"{src} n/a")
                            continue
                        vm = float(np.mean(v[idx]))
                        line.append(f"{src} {k:+d} smp ({k * 10:+d} ms, {k * vm * dt:+.1f} m, r={c:.2f})")
                        if cid == focus and src in ("ekf", "kin"):
                            corner = seg_by.get((cid, tr["lap"]))
                            if corner is not None:
                                seg = corner["segments"]
                                phases = {"entry": (seg["entry_1_brake"][0], seg["entry_2_turnin"][1]),
                                          "exit": (seg["exit_4"][0], seg["exit_5"][1])}
                                parts = []
                                for ph, (t0, t1) in phases.items():
                                    pi = np.arange(np.searchsorted(t, t0), np.searchsorted(t, t1, side="right"))
                                    pi = _contiguous(pi[r["moving"][pi]])
                                    kk, cc = xcorr_lag(alpha[pi], fy[pi], 30) if pi.size >= 40 else (None, None)
                                    parts.append(f"{ph} {'n/a' if kk is None else f'{kk:+d} smp ({kk * 10:+d} ms, n={pi.size})'}")
                                apex_s = corner["apex_lap_distance_m"]
                                pi = np.arange(tr["lo"], tr["hi"])
                                pi = _contiguous(pi[r["moving"][pi] & (np.abs(r["s_m"][pi] - apex_s) <= half)])
                                kk, cc = xcorr_lag(alpha[pi], fy[pi], 30) if pi.size >= 40 else (None, None)
                                parts.append(f"apex {'n/a' if kk is None else f'{kk:+d} smp ({kk * 10:+d} ms, n={pi.size})'}")
                                line[-1] += " [" + ", ".join(parts) + "]"
                    print(f"  C{cid} lap {tr['lap']} {axle}: " + "; ".join(line))

    # ---- 4: consequence, all traversals, all sessions
    print("\n## lag-only synthetic CS vs production negatives (ekf chain)")
    mw_span, cap = se["cs_min_slip_angle_span_rad"], se["cs_max_window_m"]
    for key in ALL_KEYS:
        r = res[key]
        supp = _load_supp(key, r)
        lags, neg_total, neg_expl, syn_neg_total = [], 0, 0, 0
        per_focus = []
        for axle, ai in (("f", 0), ("r", 1)):
            alpha = r["alpha_" + axle].astype(float)
            fy = supp["fy_" + axle]
            cs = r["cs"]["A_baseline"][ai].astype(float)
            for tr in r["traversals"]:
                idx = _contiguous(_valid_idx(r, tr))
                k, c = xcorr_lag(alpha[idx], fy[idx], 50)
                if k is None:
                    continue
                lags.append(k)
                syn = synthetic_cs(alpha, r["s_m"], tr["lo"], tr["hi"], k, r["min_window"], mw_span, cap)
                prod = cs[tr["lo"]:tr["hi"]]
                mv = r["moving"][tr["lo"]:tr["hi"]]
                pn = mv & np.isfinite(prod) & (prod < 0)
                sn = mv & np.isfinite(syn) & (syn < 0)
                neg_total += int(pn.sum()); neg_expl += int((pn & sn).sum()); syn_neg_total += int(sn.sum())
                focus = (key == "dubai" and tr["cid"] == 4) or (key == "nbr_jst_solution" and tr["cid"] == 1)
                if focus and axle == "f":
                    per_focus.append(f"C{tr['cid']} lap {tr['lap']} f: lag {k:+d}, prod min {np.nanmin(np.where(mv, prod, np.nan)):+.2f} "
                                     f"(neg {int(pn.sum())}), lag-only min {np.nanmin(syn) if np.isfinite(syn).any() else float('nan'):+.2f} "
                                     f"(neg {int(sn.sum())}), overlap {int((pn & sn).sum())}")
        la = np.array(lags)
        print(f"  {LABELS[key]}: lag over {la.size} traversal-axles median {np.median(la):+.0f} smp, p10/p90 "
              f"{np.percentile(la, 10):+.0f}/{np.percentile(la, 90):+.0f}, share |lag| >= 5 smp {np.mean(np.abs(la) >= 5):.2f}; "
              f"production negative samples {neg_total}, of which lag-only synthetic also negative {neg_expl} "
              f"({100 * neg_expl / max(neg_total, 1):.1f} %); lag-only negatives in total {syn_neg_total}")
        for p in per_focus:
            print("    " + p)


def _read_path(a, f, span_floor):
    from diagnostics.inspect_v3_sawtooth_mechanism import self_crossing_count
    # left and right turns alike: magnitudes, sign of the corner removed
    sgn = np.sign(np.median(f)) or 1.0
    a, f = a * sgn, f * sgn
    cross = self_crossing_count(a, f)
    j = int(np.argmax(f))
    after_a, after_f = a[j:], f[j:]
    grown = after_a >= a[j] + span_floor
    drop = float(f[j] - np.min(after_f[grown])) if grown.any() else 0.0
    if cross >= 1:
        label = "loop"
    elif drop >= 0.05 * f[j]:
        label = "fold"
    elif j < a.size - 1:
        label = "at-peak"
    else:
        label = "rising"
    # POST-HOC amendment (after the first run): a peaked curve can be
    # traversed backwards (recovering from beyond the peak: slip falls
    # while force rises). Time-agnostic fold: some sample at |alpha| >=
    # peak + span floor carries <= 95 % of the peak force.
    beyond = a >= a[j] + span_floor
    fold_any = bool(beyond.any() and np.min(f[beyond]) <= 0.95 * f[j])
    label_ta = "loop" if cross >= 1 else ("fold" if fold_any else label)
    return {"label": label, "label_ta": label_ta, "cross": cross, "peak_fy": float(f[j]), "peak_a": float(a[j]),
            "drop": drop, "a_max_after": float(np.max(after_a))}


def foldrun():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.stability_analysis import load_parameters
    se = load_parameters()["stability_estimation"]
    floor = se["cs_min_slip_angle_span_rad"]
    res = load_all()
    picks = [("dubai", 4, None), ("nbr_jst_solution", 1, 107), ("nbr_jst_solution", 1, 103)]
    # specificity control (added after the first 100 m read labelled almost
    # everything a loop): the same test on the Part 1 control corners,
    # healthy strongly-positive CS -- if they loop too, a loop at this
    # scale carries no information
    n_focus = None
    ctrl_picks = [(key, cid, None) for key in ("dubai", "nbr_jst_solution") for cid in _controls(res[key])]
    rows = []
    for key, cid, only_lap in picks + ctrl_picks:
        if n_focus is None and (key, cid, only_lap) == ctrl_picks[0]:
            n_focus = len(rows)
        r = res[key]
        supp = _load_supp(key, r)
        cs = r["cs"]["A_baseline"][0].astype(float)
        alpha = r["alpha_f"].astype(float)
        fy = supp["fy_f"]
        for tr in sorted(_by_corner(r)[cid], key=lambda x: x["lap"]):
            if only_lap is not None and tr["lap"] != only_lap:
                continue
            vi = _valid_idx(r, tr)
            vi = vi[np.isfinite(cs[vi])]
            if vi.size == 0:
                continue
            w = int(vi[np.argmin(cs[vi])])
            s = r["s_m"]
            # same lap: stay inside the traversal's own lap time span
            lap = next(l for l in r["laps"] if l["lap_number"] == tr["lap"])
            t = r["time"]
            lo, hi = np.searchsorted(t, lap["start_time"]), np.searchsorted(t, lap["end_time"], side="right")
            span = np.arange(lo, hi)
            # all samples, kerbs included: alpha and Fy exist there, only
            # CS is withheld -- masking them broke the runs into pieces
            span = span[(s[span] >= s[w] - 60) & (s[span] <= s[w] + 40)]
            span = _contiguous(span)
            k, _c = xcorr_lag(alpha[_contiguous(_valid_idx(r, tr))], fy[_contiguous(_valid_idx(r, tr))], 50)
            raw = _read_path(alpha[span], fy[span], floor)
            src = span + (k or 0)
            ok = (src >= 0) & (src < fy.size)
            cor = _read_path(alpha[span][ok], fy[src[ok]], floor)
            def _final(key_):
                if cor[key_] in ("fold", "loop"):
                    return cor[key_]
                if raw[key_] in ("fold", "loop"):
                    return "lag-artifact"
                return cor[key_]
            final = _final("label_ta")
            final_prereg = _final("label")
            rows.append({"key": key, "cid": cid, "lap": tr["lap"], "cs": float(cs[w]), "k": k, "m": float(s[span[-1]] - s[span[0]]),
                         "raw": raw, "cor": cor, "final": final, "final_prereg": final_prereg,
                         "kerb_share": float(np.mean(~r["moving"][span])), "a": alpha[span], "f": fy[span], "fc": fy[src[ok]], "ok": ok})
    from collections import Counter
    ctrl_rows = rows[n_focus:]
    print("CONTROLS (same test, healthy corners): " + ", ".join(
        f"{k} {v}" for k, v in Counter(x["final"] for x in ctrl_rows).items())
          + f" over {len(ctrl_rows)} instances; crossings median {np.median([x['cor']['cross'] for x in ctrl_rows]):.0f}")
    rows = rows[:n_focus]
    for x in rows:
        rw, co = x["raw"], x["cor"]
        print(f"  {LABELS[x['key']]} C{x['cid']} lap {x['lap']} front: min CS {x['cs']:+.2f}, run {x['m']:.0f} m, lag {x['k']:+d} smp | "
              f"uncorrected {rw['label']} (crossings {rw['cross']}, peak |Fy| {rw['peak_fy'] / 1000:.2f} kN at {np.degrees(rw['peak_a']):.2f} deg, "
              f"post-peak drop {rw['drop'] / 1000:.2f} kN) | lag-corrected {co['label']} (crossings {co['cross']}, "
              f"drop {co['drop'] / 1000:.2f} kN) -> pre-registered {x['final_prereg'].upper()}; "
              f"time-agnostic uncorrected {rw['label_ta']}, corrected {co['label_ta']} -> {x['final'].upper()}; "
              f"kerb/masked share of run {x['kerb_share']:.2f}")
    ink, muted = "#1f2937", "#9ca3af"
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for ax, x in zip(axes.ravel(), rows):
        sg = np.sign(np.median(x["f"])) or 1.0
        ax.plot(np.degrees(x["a"] * sg), x["f"] * sg / 1000, color=ink, lw=1.1, label="as logged")
        ax.plot(np.degrees(x["a"][x["ok"]] * sg), x["fc"] * sg / 1000, color=muted, lw=1.1, ls="--",
                label=f"Fy shifted by lag {x['k']:+d} smp")
        ax.plot(np.degrees(x["a"][0] * sg), x["f"][0] * sg / 1000, "o", mfc="white", mec=ink, ms=6)
        ax.plot(np.degrees(x["a"][-1] * sg), x["f"][-1] * sg / 1000, "o", mfc=ink, mec=ink, ms=6)
        short = "Dubai" if x["key"] == "dubai" else "NBR B"
        ax.set_title(f"{short} C{x['cid']} lap {x['lap']} front, min CS {x['cs']:+.2f}: {x['raw']['label']} -> {x['final']}",
                     fontsize=9, color=ink)
        ax.set_xlabel("|alpha| [deg]", fontsize=8); ax.set_ylabel("|Fy| front axle [kN]", fontsize=8)
        ax.legend(frameon=False, fontsize=7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    for ax in axes.ravel()[len(rows):]:
        ax.axis("off")
    fig.suptitle("100 m runs around the minimum-CS sample (open = run start, filled = end)", color=ink, fontsize=10)
    p = os.path.join(OUT_DIR, "fold_run_scale.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"figure: {p}")


PHASES = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]


def phase_index_map(r, s2, se):
    """{(cid, lap): {phase: idx}} with production summarise_corners slicing;
    apex_3 carries the apex_region indices (what the classifier reads)."""
    t, s_m = r["time"], r["s_m"]
    moving = r["moving"]
    half_w = se["apex_half_window_samples"]
    half_m = se["cs_apex_region_half_length_m"]
    valid_laps = {l["lap_number"] for l in r["laps"] if l["is_valid_for_analysis"]}
    out = {}
    for c in s2["corners"]:
        if c["stable_corner_id"] is None or c["lap_number"] not in valid_laps:
            continue
        m = {}
        for ph in PHASES:
            st, en = c["segments"][ph]
            if en < st:
                m[ph] = np.array([], dtype=int)
                continue
            lo, hi = int(np.searchsorted(t, st, "left")), int(np.searchsorted(t, en, "right"))
            if ph == "apex_3" and hi <= lo:
                lo, hi = max(0, lo - half_w), min(t.size, lo + half_w + 1)
            m[ph] = np.arange(lo, hi)[moving[lo:hi]]
        segs = [v for v in c["segments"].values() if v[1] >= v[0]]
        lo = int(np.searchsorted(t, min(v[0] for v in segs), "left"))
        hi = int(np.searchsorted(t, max(v[1] for v in segs), "right"))
        a = c["apex_lap_distance_m"]
        m["apex_3"] = np.arange(lo, hi)[moving[lo:hi] & (np.abs(s_m[lo:hi] - a) <= half_m)] if a == a else np.array([], int)
        out[(c["stable_corner_id"], c["lap_number"])] = m
    return out


def _gated_median(v, gate):
    v = v[np.isfinite(v)]
    return float(np.median(v)) if v.size >= gate else float("nan")


def _roughness(cs, r, idx_by_tr):
    steps, acfs = [], []
    from diagnostics.inspect_cs_value_lens import _acf_length_m, _runs
    for idx in idx_by_tr:
        y = cs[idx]
        ok = np.isfinite(y)
        adj = ok[1:] & ok[:-1] & (np.diff(idx) == 1)
        steps.extend(np.abs(np.diff(y))[adj].tolist())
        for a, b in _runs(ok & np.concatenate(([True], np.diff(idx) == 1))):
            if b - a >= 50:
                L = _acf_length_m(y[a:b], r["s_m"][idx[a:b]])
                if L is not None:
                    acfs.append(L)
    return np.array(steps), np.array(acfs)


def parity():
    """phase_index_map must reproduce the production per-lap phase medians
    (cached summaries) before any pooled statistic is built on it."""
    from modules.stability_analysis import load_parameters
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    res = load_all()
    for key in ALL_KEYS:
        r = res[key]
        pim = phase_index_map(r, load_supp2(key), se)
        bad, n, missing = 0, 0, 0
        for s in r["summaries"]["A_baseline"]:
            m = pim.get((s["stable_corner_id"], s["lap_number"]))
            if m is None:
                missing += 1
                continue
            for ph in PHASES:
                for ai, ax in ((0, "cs_ratio_f"), (1, "cs_ratio_r")):
                    prod = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                    mine = _gated_median(r["cs"]["A_baseline"][ai].astype(float)[m[ph]], gate)
                    n += 1
                    if not ((prod != prod and mine != mine) or abs(prod - mine) < 1e-5):
                        bad += 1
        print(f"{key}: {n} phase medians compared, {bad} differ (> 1e-5), {missing} summaries unmatched")


READOUTS = ["PROD", "REPEAT", "POOLED", "POOLED25"]


def _readout(name, per_lap, pooled, gate):
    v = sorted(x for x in per_lap if x == x)
    if name == "PROD":
        return v[0] if v else float("nan")
    if name == "REPEAT":
        return v[1] if len(v) >= 2 else float("nan")
    p = pooled[np.isfinite(pooled)]
    if p.size < gate:
        return float("nan")
    return float(np.median(p)) if name == "POOLED" else float(np.percentile(p, 25))


def corner_readouts(r, pim, laps, gate):
    """{cid: {phase: {axle: {readout: value}}}} over the lap subset."""
    cs = {"f": r["cs"]["A_baseline"][0].astype(float), "r": r["cs"]["A_baseline"][1].astype(float)}
    out = {}
    cids = sorted({cid for (cid, lap) in pim if lap in laps})
    for cid in cids:
        out[cid] = {}
        for ph in PHASES:
            out[cid][ph] = {}
            for ax in ("f", "r"):
                idxs = [pim[(cid, l)][ph] for l in laps if (cid, l) in pim]
                per_lap = [_gated_median(cs[ax][i], gate) for i in idxs]
                pooled = np.concatenate([cs[ax][i] for i in idxs]) if idxs else np.array([])
                out[cid][ph][ax] = {n: _readout(n, per_lap, pooled, gate) for n in READOUTS}
    return out


def aggregated_for(r, laps, ro, name):
    """Production aggregate (stability etc.) with the CS slots replaced by
    one read-out -- the shape build_evidence and the classifier consume."""
    import copy
    from modules.recommendation import aggregate_by_corner
    agg = aggregate_by_corner([s for s in r["summaries"]["A_baseline"] if s["lap_number"] in laps])
    agg = copy.deepcopy(agg)
    for cid, a in agg.items():
        if cid not in ro:
            continue
        for ph in PHASES:
            a["phases"][ph]["cs_ratio_f"]["median"] = ro[cid][ph]["f"][name]
            a["phases"][ph]["cs_ratio_r"]["median"] = ro[cid][ph]["r"][name]
        a["apex_region"] = {"cs_ratio_f": {"median": ro[cid]["apex_3"]["f"][name]},
                            "cs_ratio_r": {"median": ro[cid]["apex_3"]["r"][name]}}
    return agg


def _calls(agg):
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    out = {}
    for cid, a in agg.items():
        sev, short, _l, _c = classify_fn(a)
        axle = "US" if "understeer" in short else "OS" if "oversteer" in short else "-"
        out[cid] = (axle, sev, short)
    return out


def _worst(ro_c, ax, name):
    v = [ro_c[ph][ax][name] for ph in PHASES]
    v = [x for x in v if x == x]
    return min(v) if v else float("nan")


def readout():
    from collections import Counter
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_cs_value_lens import _brackets
    P = load_parameters()
    se = P["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    S_F, S_R = P["classification"]["STRONG_CSF"]["value"], P["classification"]["STRONG_CSR"]["value"]
    res = load_all()
    full, pims = {}, {}
    for key in ALL_KEYS:
        r = res[key]
        pims[key] = phase_index_map(r, load_supp2(key), se)
        laps = sorted({l for (_c, l) in pims[key]})
        full[key] = (laps, corner_readouts(r, pims[key], laps, gate))

    # sanity: PROD read-out verdicts = production aggregate verdicts
    for key in ALL_KEYS:
        from modules.recommendation import aggregate_by_corner
        r = res[key]
        laps, ro = full[key]
        prod_calls = _calls(aggregate_by_corner(r["summaries"]["A_baseline"]))
        mine = _calls(aggregated_for(r, laps, ro, "PROD"))
        diff = [c for c in prod_calls if prod_calls[c][2] != mine.get(c, (None, None, None))[2]]
        print(f"sanity {key}: PROD read-out verdicts identical to production on {len(prod_calls) - len(diff)}/{len(prod_calls)} corners")

    # a. resolvability, NBR A vs B
    print("\n[a] RESOLVABILITY, NBR A (outing 21) vs B (outing 29), apex region, 2-sigma jackknife rule")
    A, B = res["nbr_jst_problem"], res["nbr_jst_solution"]
    ba, bb = _brackets(A), _brackets(B)
    pairs = []
    for ca, (a0, a1) in sorted(ba.items()):
        for cb, (b0, b1) in bb.items():
            if min(a1, b1) - max(a0, b0) > 0.5 * min(a1 - a0, b1 - b0):
                pairs.append((ca, cb)); break

    def val_and_se(key, cid, name, what):
        laps, _ = full[key]
        r = res[key]

        def stat(lset):
            ro = corner_readouts(r, {k: v for k, v in pims[key].items() if k[0] == cid}, lset, gate)
            if cid not in ro:
                return float("nan")
            f, rr = ro[cid]["apex_3"]["f"][name], ro[cid]["apex_3"]["r"][name]
            return f if what == "f" else rr - f
        theta = stat(laps)
        loo = np.array([stat([l for l in laps if l != x]) for x in laps])
        loo = loo[np.isfinite(loo)]
        n = loo.size
        se_ = float(np.sqrt((n - 1) / n * np.sum((loo - loo.mean()) ** 2))) if n >= 2 else float("nan")
        return theta, se_

    res_a = {}
    for name in READOUTS:
        cnt = {"f": 0, "gap": 0}
        tot = {"f": 0, "gap": 0}
        detail = []
        for ca, cb in pairs:
            for what in ("f", "gap"):
                ta, sa = val_and_se("nbr_jst_problem", ca, name, what)
                tb, sb = val_and_se("nbr_jst_solution", cb, name, what)
                if not all(np.isfinite([ta, tb, sa, sb])):
                    continue
                tot[what] += 1
                if abs(tb - ta) > 2 * np.sqrt(sa ** 2 + sb ** 2):
                    cnt[what] += 1
                    detail.append(f"C{ca} {what} {ta:+.2f}->{tb:+.2f}")
        res_a[name] = (cnt, tot)
        print(f"  {name:<9} front resolved {cnt['f']}/{tot['f']}, gap resolved {cnt['gap']}/{tot['gap']}: {', '.join(detail) or 'none'}")

    # b. hard survival
    print("\n[b] HARD SURVIVAL, Dubai (C4 status UNCONFIRMED after Part 2 -- reported, not disqualifying; C3 LS disqualifying)")
    r = res["dubai"]
    laps, ro = full["dubai"]
    ls_neg = sum(1 for s in r["summaries"]["A_baseline"] if s["stable_corner_id"] == 3 and
                 (s["phases"]["exit_5"].get("ls_ratio_r", {}).get("median", float("nan")) or 0) < 0)
    for name in READOUTS:
        wf = {cid: _worst(ro[cid], "f", name) for cid in ro}
        order = sorted([c for c in wf if wf[c] == wf[c]], key=lambda c: wf[c])
        calls = _calls(aggregated_for(r, laps, ro, name))
        print(f"  {name:<9} C4 worst-phase front {wf[4]:+.3f} (rank {order.index(4) + 1} of {len(order)} most saturated, "
              f"{'below' if wf[4] < S_F else 'above'} current STRONG_CSF); C4 call '{calls[4][2]}'; "
              f"C3 front {wf[3]:+.3f}; C3 LS_r exit_5 negative laps {ls_neg}/4 (LS untouched by any CS read-out)")

    # c. call stability: split-half + production lap-pair legacy
    print("\n[c] CALL STABILITY, split-half (odd vs even analysis laps), share of corners with the same call")
    for key in ALL_KEYS:
        laps, _ = full[key]
        if len(laps) < 4:
            continue
        r = res[key]
        h1, h2 = laps[0::2], laps[1::2]
        ro1 = corner_readouts(r, pims[key], h1, gate)
        ro2 = corner_readouts(r, pims[key], h2, gate)
        parts = []
        for name in READOUTS:
            c1 = _calls(aggregated_for(r, h1, ro1, name))
            c2 = _calls(aggregated_for(r, h2, ro2, name))
            common = [c for c in c1 if c in c2]
            same = sum(1 for c in common if c1[c][0] == c2[c][0])
            flips = sum(1 for c in common if {c1[c][0], c2[c][0]} == {"US", "OS"})
            parts.append(f"{name} {same}/{len(common)} (US<->OS flips {flips})")
        print(f"  {LABELS[key]}: " + "; ".join(parts))

    # d/e. verdict distributions
    print("\n[d/e] VERDICT DISTRIBUTION per read-out (current thresholds), shift vs PROD")
    for key in ALL_KEYS:
        r = res[key]
        laps, ro = full[key]
        base = None
        parts = []
        for name in READOUTS:
            calls = _calls(aggregated_for(r, laps, ro, name))
            cnt = Counter(c[0] for c in calls.values())
            sev = Counter(c[1] for c in calls.values())
            if base is None:
                base = cnt
            parts.append(f"{name} US {cnt.get('US', 0)} OS {cnt.get('OS', 0)} ok {cnt.get('-', 0)} "
                         f"(strong {sev.get('strong', 0)}; changed calls vs PROD "
                         f"{sum(1 for c in calls if calls[c][0] != _calls(aggregated_for(r, laps, ro, 'PROD'))[c][0])})")
        print(f"  {LABELS[key]}: " + "; ".join(parts))

    # f. lap-count bias, PR US-v2
    print("\n[f] LAP-COUNT BIAS, PR US-v2: share of corners beyond current STRONG (front/rear), all laps vs 3-lap windows")
    r = res["pr_us_v2"]
    laps, ro = full["pr_us_v2"]
    for name in READOUTS:
        def share(ro_):
            vf = [_worst(ro_[c], "f", name) for c in ro_]; vr = [_worst(ro_[c], "r", name) for c in ro_]
            vf = [v for v in vf if v == v]; vr = [v for v in vr if v == v]
            return 100 * np.mean(np.array(vf) < S_F), 100 * np.mean(np.array(vr) < S_R)
        fa, ra = share(ro)
        win = [share(corner_readouts(r, pims["pr_us_v2"], laps[k:k + 3], gate)) for k in range(len(laps) - 2)]
        print(f"  {name:<9} all 9 laps f {fa:.0f} % r {ra:.0f} %; 3-lap windows mean f {np.mean([w[0] for w in win]):.0f} % "
              f"r {np.mean([w[1] for w in win]):.0f} % (ratio all/3-lap f {fa / max(np.mean([w[0] for w in win]), 1e-9):.2f})")

    with open(os.path.join(OUT_DIR, "readout_full.pkl"), "wb") as fh:
        pickle.dump({k: v for k, v in full.items()}, fh)


def census():
    """4.4: data-only candidate census, production vs pooled read-out fed
    into the frame in memory; TC score decomposed into its six terms."""
    import modules.decision_frame as df
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    from modules.recommendation import load_setup_parameters_registry, _group_by_corner
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    res = load_all()
    cfg = df.load_decision_frame_config()
    registry = load_setup_parameters_registry()
    original = df.aggregate_by_corner
    for key in ALL_KEYS:
        r = res[key]
        pim = phase_index_map(r, load_supp2(key), se)
        laps = sorted({l for (_c, l) in pim})
        ro = corner_readouts(r, pim, laps, gate)
        sm = r["summaries"]["A_baseline"]
        print(f"\n## {LABELS[key]}")
        for name in ("PROD", "POOLED"):
            df.aggregate_by_corner = original if name == "PROD" else (lambda _s, _r=r, _l=laps, _ro=ro: aggregated_for(_r, _l, _ro, "POOLED"))
            try:
                ev = df.build_evidence(sm, df.aggregate_ls_by_corner(sm), cfg, classify_fn, corners=None, state=None, channels=None)
                cands = df.generate_candidates(ev, registry, cfg, setup_data=None,
                                               assessed_corner_ids=set(_group_by_corner(sm).keys()), driving_level=None)
                split = df.generate_display_split(cands, ev, None, cfg, registry)
                sl = split["shortlist"]
                df.resolve_conflicts(sl)
                rows = df.group_display_rows(sl, registry)
            finally:
                df.aggregate_by_corner = original
            def lab(c):
                return "; ".join(f"{a.get('parameter')} {a.get('direction') or a.get('target')}" for a in c.get("actions", []))
            top = rows[:3]
            tc_ranks = [i + 1 for i, c in enumerate(rows) if any((a.get("parameter") or "").startswith("tc_") for a in c.get("actions", []))]
            dirs = {(a.get("parameter"), a.get("direction")) for c in top for a in c.get("actions", []) if (a.get("parameter") or "").startswith("tc_")}
            both = [p for p in ("tc_lon", "tc_lat") if (p, "increase") in dirs and (p, "decrease") in dirs]
            print(f"  {name}: {len(cands)} candidates, {len(rows)} proposed rows; TC ranks {tc_ranks}; both directions in top 3: {both or 'none'}")
            for i, c in enumerate(top, 1):
                members = c.get("group_members") or [c]
                n_ev = sum(len(m.get("evidence_refs", [])) for m in members)
                corners = sorted({m.get("corner") for m in members if m.get("corner") is not None})
                comp = c.get("score_components", {})
                print(f"    #{i} {lab(c)} score {c.get('score'):.3f} = " + " + ".join(f"{k} {v:.3f}" for k, v in comp.items())
                      + f"; corners {corners}, evidence items {n_ev}")


def sawreport():
    from modules.stability_analysis import load_parameters
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    res = load_all()
    # noise floor: the same outing exported twice (v3 vs US-v1)
    va, vb = res["v3"]["summaries"]["A_baseline"], res["pr_us_v1"]["summaries"]["A_baseline"]
    ib = {(s["stable_corner_id"], s["lap_number"]): s for s in vb}
    noise = []
    for s in va:
        o = ib.get((s["stable_corner_id"], s["lap_number"]))
        if o is None:
            continue
        for ph in PHASES:
            for ax in ("cs_ratio_f", "cs_ratio_r"):
                x = (s["apex_region"] if ph == "apex_3" else s["phases"][ph])[ax]["median"]
                y = (o["apex_region"] if ph == "apex_3" else o["phases"][ph])[ax]["median"]
                if x == x and y == y:
                    noise.append(abs(x - y))
    noise = np.array(noise)
    print(f"noise floor (v3 vs US-v1 re-export, per-lap phase medians, n={noise.size}): median |shift| "
          f"{np.median(noise):.4f}, p90 {np.percentile(noise, 90):.4f}")
    all_shift = []
    for key in ALL_KEYS:
        r = res[key]
        saw = np.load(os.path.join(OUT_DIR, f"saw_{key}.npz"))
        s2 = load_supp2(key)
        pim = phase_index_map(r, s2, se)
        idx_by_tr = [_valid_idx(r, tr) for tr in r["traversals"]]
        parts, shifts = [], []
        for axle, ai in (("f", 0), ("r", 1)):
            prod, rep = saw[f"cs_prod_{axle}"].astype(float), saw[f"cs_rep_{axle}"].astype(float)
            same_mask = np.array_equal(np.isfinite(prod), np.isfinite(rep))
            sp, ap = _roughness(prod, r, idx_by_tr)
            sr, ar = _roughness(rep, r, idx_by_tr)
            # footprint and jump attribution on corner samples
            ti = r["term_idx"]
            s = r["s_m"]
            jm = {}
            for v, cs in (("prod", prod), ("rep", rep)):
                st = saw[f"start_{v}_{axle}"][ti]
                fp = np.where(st >= 0, s[ti - 1] - s[np.maximum(st, 0)], np.nan)
                d_cs, d_fp = np.abs(np.diff(cs[ti])), np.diff(fp)
                ok = (np.diff(ti) == 1) & np.isfinite(d_cs) & np.isfinite(d_fp)
                jump = ok & (d_fp < -2.0)
                jm[v] = (jump.sum() / max(ok.sum(), 1), d_cs[jump].sum() / max(d_cs[ok].sum(), 1e-12),
                         float(np.nanmedian(fp[np.isfinite(cs[ti])])))
            # phase-median shift, same pass production vs repaired
            for (cid, lap), m in pim.items():
                for ph in PHASES:
                    a_, b_ = _gated_median(prod[m[ph]], gate), _gated_median(rep[m[ph]], gate)
                    if a_ == a_ and b_ == b_:
                        shifts.append(abs(a_ - b_))
            parts.append(f"{axle}: step mean {sp.mean():.4f} -> {sr.mean():.4f} (p95 {np.percentile(sp, 95):.3f} -> "
                         f"{np.percentile(sr, 95):.3f}); ACF length {np.median(ap):.1f} -> {np.median(ar):.1f} m; "
                         f"window {jm['prod'][2]:.1f} -> {jm['rep'][2]:.1f} m; jumps {jm['prod'][0]:.3f}/{jm['prod'][1]:.2f} -> "
                         f"{jm['rep'][0]:.3f}/{jm['rep'][1]:.2f} (share samples/|dCS|); qualifying set identical {same_mask}")
        shifts = np.array(shifts)
        all_shift.extend(shifts.tolist())
        print(f"\n## {LABELS[key]}")
        for p in parts:
            print("  " + p)
        print(f"  per-lap phase-median shift repaired vs production: median {np.median(shifts):.4f}, p90 "
              f"{np.percentile(shifts, 90):.4f}, max {shifts.max():.3f} (n={shifts.size})")
    all_shift = np.array(all_shift)
    print(f"\nALL sessions: phase-median shift median {np.median(all_shift):.4f}, p90 {np.percentile(all_shift, 90):.4f} "
          f"vs noise floor {np.median(noise):.4f}/{np.percentile(noise, 90):.4f}")


def _cs_axle_dual(alpha, Fy, moving, s_m, params, min_window):
    """Copy of stability_analysis.compute_cs_for_axle, run twice in one
    pass: production start and repaired start (header, Part 3)."""
    from modules.stability_analysis import (_find_monotonic_sections, _section_slopes, _smooth_weight,
                                            SPAN_WEIGHT_EXPONENT, R2_WEIGHT_EXPONENT)
    se = params["stability_estimation"]
    min_span = se["cs_min_slip_angle_span_rad"]
    linear_thresh = se["cs_linear_slip_threshold_rad"]
    max_window_m = se["cs_max_window_m"]
    n = len(alpha)
    sections, section_id = _find_monotonic_sections(alpha)
    sec_slopes, sec_spans = _section_slopes(alpha, Fy, sections)
    out = {}
    state = {v: {"ref": np.nan, "cs": np.full(n, np.nan), "start": np.full(n, -1, dtype=np.int64)}
             for v in ("prod", "rep")}
    prev_rep = None
    for i in range(min_window, n):
        if not moving[i]:
            prev_rep = None
            continue
        start = i - min_window
        s_i = s_m[i - 1]
        if not np.isfinite(s_i):
            s_i = None
        if start > 0:
            window_max = np.max(alpha[start:i])
            window_min = np.min(alpha[start:i])
        while start > 0:
            if window_max - window_min >= min_span:
                break
            if s_i is not None:
                s_start = s_m[start]
                if not np.isfinite(s_start) or s_start > s_i or (s_i - s_start) >= max_window_m:
                    break
                pass
            start -= 1
            window_max = np.maximum(window_max, alpha[start])
            window_min = np.minimum(window_min, alpha[start])
        w = alpha[start:i]
        if np.max(w) - np.min(w) < min_span:
            prev_rep = None
            continue
        rep = start if prev_rep is None else min(start, prev_rep + 2)
        # never beyond the cap: advance until the cap holds again
        while s_i is not None and rep < start and (not np.isfinite(s_m[rep]) or s_i - s_m[rep] > max_window_m):
            rep += 1
        prev_rep = rep
        for variant, st in (("prod", start), ("rep", rep)):
            S = state[variant]
            wa, wf = alpha[st:i], Fy[st:i]
            a_mean, f_mean = np.mean(wa), np.mean(wf)
            denom = np.sum((wa - a_mean) ** 2)
            if denom < 1e-10:
                continue
            C_w = np.sum((wa - a_mean) * (wf - f_mean)) / denom
            f_hat = C_w * wa + (f_mean - C_w * a_mean)
            ss_res = np.sum((wf - f_hat) ** 2)
            ss_tot = np.sum((wf - f_mean) ** 2)
            R2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0
            weights, slopes = [], []
            for k in np.unique(section_id[st:i]):
                if np.isnan(sec_slopes[k]):
                    continue
                w_k = _smooth_weight(sec_spans[k], 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
                if w_k <= 0:
                    continue
                weights.append(w_k); slopes.append(sec_slopes[k])
            if weights:
                C_s = float(np.sum(np.array(weights) * np.array(slopes)) / np.sum(weights))
                w_r2 = _smooth_weight(R2, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
                C_a = w_r2 * C_w + (1.0 - w_r2) * C_s
            else:
                C_a = C_w
            if np.max(np.abs(wa)) < linear_thresh:
                S["ref"] = C_a
            if not np.isnan(S["ref"]) and S["ref"] > 0:
                S["cs"][i] = min(C_a / S["ref"], 1.0)
            S["start"][i] = st
    return state


def sawtooth(key):
    from modules.stability_analysis import load_parameters, resolve_cs_min_window_samples
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))
    res = load_all()
    r = res[key]
    supp = _load_supp(key, r)
    mw = resolve_cs_min_window_samples(params, 100.0) if r["min_window"] is None else r["min_window"]
    out = {}
    for axle, ai in (("f", 0), ("r", 1)):
        st = _cs_axle_dual(r["alpha_" + axle].astype(float), supp["fy_" + axle], r["moving"], r["s_m"], params, mw)
        prod_cached = r["cs"]["A_baseline"][ai].astype(float)
        # cached alpha is float32, production ran float64: report the size
        # of any mismatch rather than demand bit identity
        fa, fb = np.isfinite(prod_cached), np.isfinite(st["prod"]["cs"])
        both = fa & fb
        d = np.abs(prod_cached[both] - st["prod"]["cs"][both])
        print(f"[{key}] axle {axle}: parity vs cached production -- finite in one only: {int((fa ^ fb).sum())} of "
              f"{int(fa.sum())}; |diff| median {np.median(d):.2e}, p99 {np.percentile(d, 99):.2e}, max {d.max():.2e}; "
              f"repaired vs same-pass production finite masks identical: "
              f"{np.array_equal(fb, np.isfinite(st['rep']['cs']))}")
        for v in ("prod", "rep"):
            out[f"cs_{v}_{axle}"] = st[v]["cs"].astype(np.float32)
            out[f"start_{v}_{axle}"] = st[v]["start"]
    np.savez_compressed(os.path.join(OUT_DIR, f"saw_{key}.npz"), **out)
    print(f"[{key}] sawtooth arrays saved")


SPIKE_SPAN_M, SPIKE_SEP_M, JUMP_FP_DROP_M = 5.0, 20.0, 2.0


def _jump_samples(start, s, ti):
    fp = np.where(start[ti] >= 0, s[ti - 1] - s[np.maximum(start[ti], 0)], np.nan)
    d_fp = np.diff(fp)
    ok = (np.diff(ti) == 1) & np.isfinite(d_fp)
    return ti[1:][ok & (d_fp < -JUMP_FP_DROP_M)]


def _drops(cs, s, idx):
    """drop(i) and the reference sample j* for each i of idx (Part 5)."""
    drop = np.full(idx.size, np.nan)
    ref = np.full(idx.size, -1, dtype=np.int64)
    run_start = 0
    for k in range(idx.size):
        if not np.isfinite(cs[idx[k]]):
            run_start = k + 1
            continue
        if k > 0 and idx[k] - idx[k - 1] != 1:
            run_start = k
        j = k
        while j - 1 >= run_start and s[idx[k]] - s[idx[j - 1]] <= SPIKE_SPAN_M:
            j -= 1
        seg = cs[idx[j:k + 1]]
        jm = j + int(np.argmax(seg))
        drop[k] = seg.max() - cs[idx[k]]
        ref[k] = idx[jm]
    return drop, ref


def sawrender(key):
    from diagnostics.trace_render import render_cs_traces
    res = load_all()
    r = res[key]
    saw = np.load(os.path.join(OUT_DIR, f"saw_{key}.npz"))
    t, s, ti = r["time"], r["s_m"], r["term_idx"]
    an = [l for l in r["laps"] if l["is_valid_for_analysis"]]
    spikes, jumps = [], {}
    for axle in ("f", "r"):
        prod, rep = saw[f"cs_prod_{axle}"].astype(float), saw[f"cs_rep_{axle}"].astype(float)
        jumps[axle] = _jump_samples(saw[f"start_prod_{axle}"], s, ti)
        jset = set(jumps[axle].tolist())
        for l in an:
            idx = ti[(t[ti] >= l["start_time"]) & (t[ti] <= l["end_time"])]
            if idx.size == 0:
                continue
            d_p, ref_p = _drops(prod, s, idx)
            d_r, _ = _drops(rep, s, idx)
            order = np.argsort(np.where(np.isfinite(d_p), -d_p, np.inf))
            taken = []
            for k in order:
                if not np.isfinite(d_p[k]) or d_p[k] <= 0:
                    break
                if any(abs(s[idx[k]] - s[idx[q]]) < SPIKE_SEP_M for q in taken):
                    continue
                taken.append(k)
                near = np.abs(s[idx] - s[idx[k]]) <= SPIKE_SPAN_M
                rep_drop = float(np.nanmax(d_r[near])) if np.isfinite(d_r[near]).any() else float("nan")
                rep_min = float(np.nanmin(rep[idx[near]])) if np.isfinite(rep[idx[near]]).any() else float("nan")
                j0, i0 = int(ref_p[k]), int(idx[k])
                coinc = any(j0 <= q <= i0 for q in jset)
                spikes.append({"axle": axle, "lap": l["lap_number"], "s": float(s[i0]), "i": i0, "j": j0,
                               "drop_p": float(d_p[k]), "val_p": float(prod[i0]), "top_p": float(prod[j0]),
                               "drop_r": rep_drop, "min_r": rep_min, "span_m": float(s[i0] - s[j0]), "jump": coinc})
    spikes.sort(key=lambda x: -x["drop_p"])
    top = spikes[:5]
    lab = LABELS[key]
    for variant in ("prod", "rep"):
        markers = None
        if variant == "prod":
            markers = {}
            for axle in ("f", "r"):
                cs = saw[f"cs_prod_{axle}"].astype(float)
                jm = jumps[axle]
                markers[axle] = {"s": s[jm], "y": cs[jm], "color": "#dc2626",
                                 "label": f"window-start jump (footprint drop > {JUMP_FP_DROP_M:.0f} m), n={jm.size}",
                                 "notes": [(x["s"], x["val_p"], str(n)) for n, x in enumerate(top, 1) if x["axle"] == axle]}
        p = os.path.join(OUT_DIR, f"sawrender_{key}_{variant}.png")
        clipped = render_cs_traces(p, t, s, saw[f"cs_{variant}_f"].astype(float), saw[f"cs_{variant}_r"].astype(float),
                                   r["laps"], r["corners"],
                                   f"{lab} -- {'PRODUCTION' if variant == 'prod' else 'SAWTOOTH REPAIR ACTIVE (in memory)'}",
                                   markers=markers)
        print(f"figure: {p} (samples below y floor: front {clipped['f']}, rear {clipped['r']})")
    print(f"\n## {lab}: jumps marked front {jumps['f'].size}, rear {jumps['r'].size}; spikes found {len(spikes)}")
    print(f"five deepest per-lap spikes (drop within {SPIKE_SPAN_M:.0f} m, production vs repair at +/- {SPIKE_SPAN_M:.0f} m):")
    for n, x in enumerate(top, 1):
        print(f"  {n}. {'front' if x['axle'] == 'f' else 'rear'} lap {x['lap']} at {x['s']:.0f} m: production drop "
              f"{x['drop_p']:.2f} ({x['top_p']:+.2f} -> {x['val_p']:+.2f} over {x['span_m']:.1f} m); repair drop "
              f"{x['drop_r']:.2f} (min {x['min_r']:+.2f}); jump-coincident {'yes' if x['jump'] else 'no'}")
    share = np.mean([x["jump"] for x in spikes]) if spikes else float("nan")
    deep = [x for x in spikes if x["drop_p"] >= 1.0]
    print(f"  context: jump-coincident share over all {len(spikes)} spikes {share:.2f}; over the {len(deep)} spikes "
          f"with drop >= 1.0: {np.mean([x['jump'] for x in deep]) if deep else float('nan'):.2f}")
    # the author's flagged points: per lap, the lowest front sample within
    # +/- 60 m, with the same drop / repair / jump test applied at it
    prod, rep = saw["cs_prod_f"].astype(float), saw["cs_rep_f"].astype(float)
    jset = set(jumps["f"].tolist())
    for s0 in {"nbr_jst_solution": (620.0,), "pr_us_v2": (1300.0,)}.get(key, ()):
        print(f"  flagged ~{s0:.0f} m front, per analysis lap (lowest sample within +/- 60 m):")
        for l in an:
            idx = ti[(t[ti] >= l["start_time"]) & (t[ti] <= l["end_time"])]
            if idx.size == 0:
                continue
            d_p, ref_p = _drops(prod, s, idx)
            d_r, _ = _drops(rep, s, idx)
            win = (np.abs(s[idx] - s0) <= 60) & np.isfinite(prod[idx])
            if not win.any():
                continue
            k = int(np.flatnonzero(win)[np.argmin(prod[idx][win])])
            near = np.abs(s[idx] - s[idx[k]]) <= SPIKE_SPAN_M
            j0, i0 = int(ref_p[k]), int(idx[k])
            print(f"    lap {l['lap_number']} at {s[i0]:.0f} m: production {prod[j0]:+.2f} -> {prod[i0]:+.2f} "
                  f"(drop {d_p[k]:.2f} over {s[i0] - s[j0]:.1f} m); repair min {np.nanmin(rep[idx[near]]):+.2f}, "
                  f"drop {np.nanmax(d_r[near]):.2f}; jump-coincident {'yes' if any(j0 <= q <= i0 for q in jset) else 'no'}")


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "supp2":
        supp2(sys.argv[2])
    elif mode == "sawtooth":
        sawtooth(sys.argv[2])
    elif mode == "sawrender":
        sawrender(sys.argv[2])
    else:
        {"lag": lag, "foldrun": foldrun, "sawreport": sawreport, "parity": parity,
         "readout": readout, "census": census}[mode]()
