# DIAGNOSTIC, read-only. Kerb-detection variants (work order 2026-10-01,
# follow-up to the PARKED open limitation "CS minima cluster at kerb-mask
# edges"). Does a kerb mask that covers the FULL strike disturbance make the
# CS_ratio minima leave the mask edges? Nothing in modules/, core/ or config/
# changes; any adoption is a later calibration change at an estimator-input
# boundary. Tier B throughout (event detection by residual-from-trend
# thresholding, mask dilation, masking inside a regression window); no
# literature anchor claimed, none needed for preprocessing.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_kerb_mask_variants compute <key>   (parse, slim cache)
#   python -m diagnostics.inspect_kerb_mask_variants part1 tier1|all
#   python -m diagnostics.inspect_kerb_mask_variants masks <key>     (variant masks + CS re-estimation)
#   python -m diagnostics.inspect_kerb_mask_variants score tier1|all
# Inputs: the raw file via prepare_vehicle_state (az_g, ay, yaw rate, raw
# moving mask, production kerb mask), time base asserted identical to the
# new_laps cache; value-lens supp (production Fy_f/Fy_r); (g) step-0 cache
# (BASE = production rule, K = production mask with in-window exclusion).
# Outputs in diagnostics/plots/kerb_mask_variants/ (gitignored).
#
# RULES -- PART 1, disturbance envelope (fixed before the first run)
# - Event: a contiguous run of the production kerb mask (|az - 1 g| > 1.2 g,
#   dilated +/-5 samples) with at least one sample on an analysis lap and
#   moving. Runs are not merged.
# - Channels: az [g], ay [m/s^2], yaw rate [deg/s], raw on the state grid
#   (what the CS chain consumes before its own filtering). Header units
#   censused from the file and printed by `compute`.
# - Local trend: cubic in time, fitted on the context CTX_S = 1.0 s before
#   the run onset and after its end, excluding every kerb-mask sample in the
#   context (neighbouring runs too). Three passes of iterative clipping: drop
#   |res| > K_SIGMA * sigma and refit. sigma = 1.4826 * MAD of the retained
#   residuals = the local noise. Cubic, not lower: over ~2.5 s the corner's
#   own entry/exit shape is not quadratic and a misfit would read as
#   disturbance.
# - Disturbance = |residual| > K_SIGMA * sigma, K_SIGMA = 3.
# - Extension after the mask: walk forward from the last masked sample; the
#   envelope continues while the next exceedance lies within GAP_S = 0.10 s
#   of the previous one (bridges the zero crossings of an oscillating
#   transient, ~5 Hz half-period). Extension = last connected exceedance -
#   mask end, in samples and metres; 0 if none starts within GAP_S. Mirror
#   rule before the onset. Reaching the context edge -> censored (counted,
#   reported).
# - Null control (stated so extension is read against what the detector
#   finds with no strike): one pseudo-event per event, same run length,
#   placed at a random position inside a corner bracket of an analysis lap,
#   >= 2 s from any kerb-mask sample, seed SEED; same detector.
# - Unloading (az): per event, the raw threshold trip (first raw sample,
#   before dilation) and its side (above 2.2 g / below -0.2 g); a DOWNWARD
#   pre-mask az excursion = connected exceedances with residual < -K_SIGMA *
#   sigma ending at or after the dilated onset, walked back with GAP_S; its
#   duration to the raw trip; the az minimum in that span.
# - Reported per session: pre/post extension p25/p50/p75/p90 and censored
#   count per channel and for the ay/yaw union, events vs control; the NBR
#   C1 site (600-640 m) and the 4480-4540 m site individually per lap.
#
# RULES -- PART 2, variant masks (constants fixed from Part 1 before scoring)
#  A  production mask.
#  B  asymmetric dilation of A: PRE_MARGIN before each run, POST_MARGIN
#     after it; margins = pooled Tier-1 p75 of the per-event ay/yaw union
#     extension (pre and post separately), converted to samples at the grid
#     rate. Values written below after Part 1, before Part 3.
#     [Set 2026-10-01 from the corrected Part 1 run: pooled Tier-1 union p75
#     pre 9, post 8 samples at 100 Hz -> 0.09 s / 0.08 s, on top of the
#     production +/-5 sample dilation. p75 rather than p90: the null
#     control's own p90 (8-9 samples) shows detector false positives at that
#     level, and the order names p75.]
#  C  unloading-aware: each run's onset moved back to the start of its
#     downward az excursion (Part 1 detector), end unchanged; no excursion ->
#     run unchanged.
#  D  disturbance-based: each run widened to its own measured ay/yaw union
#     envelope (Part 1 detector), never narrower than A.
#  Detector-based masks (C, D) are built on every run of the session, not
#  only on analysis laps, so the CS loop sees one consistent mask.
#  Simplicity order B < C < D (B: one constant pair; C: one channel's
#  per-event detector; D: two channels' per-event detector).
#
# RULES -- PART 3, scoring (fixed before scoring)
# - Re-estimation: (g) step-0 machinery K (inspect_cs_kerb_window.cs_triple)
#   with the variant's mask as the kerb mask: kerb samples excluded inside
#   windows and as endpoints (moving = raw moving & ~mask). A under K is the
#   (g) K array, reused from the step-0 cache (parity printed).
# - Reference = production BASE (endpoint-only exclusion, production mask),
#   as in (g).
# - Edges for "near" are the VARIANT'S OWN mask edges. Reason: a widened mask
#   turns the samples around the production edges into NaN, so production-edge
#   enrichment would collapse by construction; only own-edge enrichment can
#   separate "disturbance now covered" (collapses) from "apex minimum pushed
#   to the new boundary" (persists). Production-edge figures are printed as
#   informative only.
# - Leg 1: the pooled Tier-1 excess over chance (enrichment - 1) at least
#   halves vs BASE on BOTH axles ((g) close-out ruling 3, unchanged).
# - Leg 2: <= 1 new deep-tail minimum per session and axle (below the BASE
#   session p10 while its BASE minimum was above it) more than 3 m from the
#   variant's own edges; plus trace inspection.
# - Leg 3: battery elements intact: US-v2 C13 and JST 29 C9 worst-phase
#   r - f gaps stay > 0 (Tier 1); Dubai C3 LS and NBR A->B in Tier 2.
# - Data cost: masked share of moving analysis-lap samples per variant;
#   finite-CS coverage of corner-traversal samples; changed corner calls.
# - C1 (600-640 m) and the 4480-4540 m site: per-lap front/rear minima per
#   variant. Traces: BASE vs the best variant, standard trace set.
# - Tier 2 (all eight) only for a variant passing all Tier-1 legs.

# RULES -- PART 5, window reach (author ruling 2026-10-01; fixed before the run)
# Question: does the CS window span a masked kerb gap and fit pre-strike
# against post-strike samples as one population?
# - Measurement (`reach tier1`): at every per-corner-lap CS minimum (BASE =
#   production window rule, reconstruct_cs_window_start; A = the K growth
#   rule), the window [start, i) is read against the production mask:
#   masked share = masked samples / window samples; STRADDLE = the window
#   holds >= 1 unmasked sample before its first masked sample AND >= 1 after
#   its last one (one regression over both sides of a strike); post count =
#   unmasked samples after the last masked one. Reported for the near-edge
#   minima (own production edges, 3 m) against all minima as base rate, and
#   split by side (before an onset / after an end, matched within the lap).
# - Decisive variant T (`trunc <key>`, scored by `score_t tier1`): the
#   window may not contain a masked sample. Growth as production (start =
#   i - min_window, back until the alpha span clears the minimum or the
#   cs_max_window_m cap), but a masked sample at or entering the window ends
#   growth with NO estimate. For a left-sided window, truncating at the gap
#   and restarting after it are the same rule: the window can only begin
#   after the last masked sample before its endpoint. Same min_window, span
#   and cap; section slopes over unmasked samples (as K); linear reference
#   carried as in production. This is the (g) alternative considered and not
#   proposed. Self-check: T with an all-false mask equals BASE exactly on
#   20 000 JST 29 samples. Mask = production A.
# - Exit legs: as Part 3 (own-edge enrichment halves on both axles; <= 1 new
#   deep minimum per session/axle; battery gaps > 0), traces inspected.
#   Named: C1 (600-640 m), QP MLA ~2800 m (2780-2820 m, the dip B and D
#   introduced), 4500 m.

import gzip
import os
import pickle
import sys

import numpy as np

from diagnostics import inspect_cs_kerb_edge as ke

KEYS = ke.KEYS
TIER1 = ["nbr_jst_solution", "pr_us_v2", "nbr_qp_mla"]
LABEL = ke.LABEL
OUT_DIR = "diagnostics/plots/kerb_mask_variants"
CTX_S = 1.0
K_SIGMA = 3.0
GAP_S = 0.10
CLIP_PASSES = 3
POLY_DEG = 3
CONTROL_CLEAR_S = 2.0
SEED = 20261001
C1_SITE = (600.0, 640.0)
SITE_4500 = (4480.0, 4540.0)
SITE_2800 = (2780.0, 2820.0)
CH = ("az", "ay", "yaw")
# PART 2 B margins (seconds) -- set from Part 1 pooled Tier-1 p75, before Part 3
PRE_MARGIN_S = 0.09
POST_MARGIN_S = 0.08
VARIANTS = ["BASE", "A", "B", "C", "D"]


def _cache_path(key):
    return os.path.join(OUT_DIR, "cache", f"{key}.pkl.gz")


def compute(key):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
    from diagnostics.gated_kinematic_common import FIXED_CAP
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    data = parse_csv(SESSIONS[key])
    ch = data["channels"]
    for n in ("log_acc_z", "log_acc_y", "sclu_yaw_rate"):
        c = ch.get(n)
        rate = 1.0 / np.median(np.diff(c["time"])) if c is not None and c.get("time") is not None else float("nan")
        print(f"[{key}] census {n}: header unit {c.get('unit_raw') if c else None!r}, config unit "
              f"{c.get('unit') if c else None!r}, quality {c.get('quality') if c else None}, native {rate:.1f} Hz")
    st = prepare_vehicle_state(ch, params)
    with gzip.open(os.path.join("diagnostics/plots/new_laps/cache", f"{key}.pkl.gz"), "rb") as fh:
        nl = pickle.load(fh)
    assert nl["time"].tobytes() == st["time"].tobytes(), key
    assert np.array_equal(nl["moving"], st["moving_mask"] & ~st["kerb_mask"]), key
    se = params["stability_estimation"]
    raw_trip = np.abs(st["az_g"] - se["kerb_baseline_g"]) > se["kerb_z_deviation_threshold_g"]
    out = {"time": st["time"], "s_m": st["s_m"], "sr": float(st["sample_rate_hz"]),
           "az": np.asarray(st["az_g"], float), "ay": np.asarray(st["ay_mps2"], float),
           "yaw": np.degrees(np.asarray(st["yaw_rate_radps"], float)),
           "moving_raw": st["moving_mask"], "kerb": st["kerb_mask"], "raw_trip": raw_trip,
           "laps": nl["laps"], "corners": nl["corners"], "traversals": nl["traversals"]}
    print(f"[{key}] grid {out['sr']:.0f} Hz, {len(out['time'])} samples, kerb share {out['kerb'].mean():.4f}")
    os.makedirs(os.path.join(OUT_DIR, "cache"), exist_ok=True)
    with gzip.open(_cache_path(key), "wb") as fh:
        pickle.dump(out, fh)


def _load(key):
    with gzip.open(_cache_path(key), "rb") as fh:
        return pickle.load(fh)


def _runs(mask):
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def _analysis_idx(c):
    t = c["time"]
    m = np.zeros(len(t), bool)
    for l in c["laps"]:
        if l.get("is_valid_for_analysis"):
            m |= (t >= l["start_time"]) & (t <= l["end_time"])
    return m


def _trend_resid(x, t, idx, fit_ok):
    """Cubic-in-time trend over idx with iterative clipping; returns residuals
    over idx and sigma (robust)."""
    tt = t[idx] - t[idx].mean()
    use = fit_ok.copy()
    if use.sum() < 4 * (POLY_DEG + 1):
        return None, None
    for _ in range(CLIP_PASSES):
        p = np.polyfit(tt[use], x[idx][use], POLY_DEG)
        r = x[idx] - np.polyval(p, tt)
        sig = 1.4826 * np.median(np.abs(r[use] - np.median(r[use])))
        if sig <= 0:
            return None, None
        keep = fit_ok & (np.abs(r) <= K_SIGMA * sig)
        if keep.sum() < 4 * (POLY_DEG + 1) or np.array_equal(keep, use):
            break
        use = keep
    return r, sig


def _walk(exc, start, step, gap_n, stop):
    """Last connected exceedance index walking from `start` in direction
    `step` (exclusive of start), bridging gaps up to gap_n; None if none.
    Returns (index or None, censored)."""
    last, j, since = None, start + step, 0
    while (j <= stop if step > 0 else j >= stop):
        if exc[j]:
            last, since = j, 0
        else:
            since += 1
            if since > gap_n:
                return last, False
        j += step
    return last, True


def event_envelope(c, a, b, kerb_any):
    """Disturbance extension around mask run [a, b] for each channel. kerb_any:
    mask whose samples are excluded from the trend fit."""
    t, s, sr = c["time"], c["s_m"], c["sr"]
    n_ctx, gap_n = int(round(CTX_S * sr)), int(round(GAP_S * sr))
    lo, hi = max(0, a - n_ctx), min(len(t) - 1, b + n_ctx)
    idx = np.arange(lo, hi + 1)
    fit_ok = ~kerb_any[idx] & np.isfinite(c["az"][idx])
    ia, ib = a - lo, b - lo
    # correction after the first Part-1 run: the walk stops at a neighbouring
    # mask run (else an excursion is credited with the neighbour's strike)
    other = kerb_any[idx].copy()
    other[ia:ib + 1] = False
    before, after = np.flatnonzero(other[:ia]), np.flatnonzero(other[ib + 1:])
    stop_lo = before[-1] + 1 if before.size else 0
    stop_hi = ib + after[0] if after.size else len(idx) - 1
    res = {}
    for name in CH:
        r, sig = _trend_resid(c[name], t, idx, fit_ok)
        if r is None:
            res[name] = None
            continue
        exc = np.abs(r) > K_SIGMA * sig
        jp, cens_p = _walk(exc, ia, -1, gap_n, stop_lo)
        jq, cens_q = _walk(exc, ib, +1, gap_n, stop_hi)
        e = {"sig": sig,
             "pre_n": (ia - jp) if jp is not None else 0, "post_n": (jq - ib) if jq is not None else 0,
             "pre_cens": cens_p and jp is not None and stop_lo == 0,
             "post_cens": cens_q and jq is not None and stop_hi == len(idx) - 1,
             "pre_nb": cens_p and jp is not None and stop_lo > 0,
             "post_nb": cens_q and jq is not None and stop_hi < len(idx) - 1,
             "pre_i": lo + jp if jp is not None else a, "post_i": lo + jq if jq is not None else b}
        e["pre_m"] = float(s[a] - s[e["pre_i"]]) if np.isfinite(s[a]) and np.isfinite(s[e["pre_i"]]) else np.nan
        e["post_m"] = float(s[e["post_i"]] - s[b]) if np.isfinite(s[b]) and np.isfinite(s[e["post_i"]]) else np.nan
        if name == "az":
            # downward excursion before the onset (unloading)
            down = r < -K_SIGMA * sig
            jd, _ = _walk(down, ia, -1, gap_n, stop_lo)
            e["down_pre_n"] = (ia - jd) if jd is not None else 0
            e["down_i"] = lo + jd if jd is not None else a
        res[name] = e
    return res


def _union(ev, side):
    vals = [ev[n][f"{side}_n"] for n in ("ay", "yaw") if ev.get(n) is not None]
    return max(vals) if vals else None


def _events(c):
    an = _analysis_idx(c)
    mv = c["moving_raw"]
    return [(a, b) for a, b in _runs(c["kerb"]) if (an[a:b + 1] & mv[a:b + 1]).any()]


def _controls(c, evs, rng):
    sr = c["sr"]
    clear = int(round(CONTROL_CLEAR_S * sr))
    near = np.convolve(c["kerb"].astype(float), np.ones(2 * clear + 1), "same") > 0
    an = _analysis_idx(c)
    cand = np.zeros(len(c["time"]), bool)
    for tr in c["traversals"]:
        cand[tr["lo"]:tr["hi"]] = True
    cand &= an & c["moving_raw"] & ~near
    pool = np.flatnonzero(cand)
    out = []
    for a, b in evs:
        L = b - a
        for _ in range(50):
            p = int(rng.choice(pool))
            if p + L < len(cand) and cand[p:p + L + 1].all():
                out.append((p, p + L))
                break
    return out


def _q(x):
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    if x.size == 0:
        return "n/a"
    return "/".join(f"{np.percentile(x, q):.0f}" for q in (25, 50, 75, 90)) + f" (n {x.size})"


def _qf(x, fmt="{:.1f}"):
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    if x.size == 0:
        return "n/a"
    return "/".join(fmt.format(np.percentile(x, q)) for q in (25, 50, 75, 90))


def part1(tier):
    keys = TIER1 if tier == "tier1" else KEYS
    rng = np.random.default_rng(SEED)
    pooled = {"pre": [], "post": [], "pre_ctl": [], "post_ctl": []}
    store = {}
    for key in keys:
        c = _load(key)
        sr, s = c["sr"], c["s_m"]
        evs = _events(c)
        E = [event_envelope(c, a, b, c["kerb"]) for a, b in evs]
        ctl = _controls(c, evs, rng)
        C = [event_envelope(c, a, b, c["kerb"]) for a, b in ctl]
        print(f"\n== {LABEL[key]}: {len(evs)} kerb events on analysis laps, {len(ctl)} controls, grid {sr:.0f} Hz "
              f"(1 sample = {1000 / sr:.0f} ms)")
        print("  extension in samples, p25/p50/p75/p90; [m] in metres")
        for name in CH:
            ev = [e[name] for e in E if e[name] is not None]
            cv = [e[name] for e in C if e[name] is not None]
            for side in ("pre", "post"):
                print(f"  {name:3s} {side:4s}: events {_q([e[f'{side}_n'] for e in ev])}  "
                      f"[m {_qf([e[f'{side}_m'] for e in ev])}]  censored {sum(e[f'{side}_cens'] for e in ev)}, at neighbour run {sum(e[f'{side}_nb'] for e in ev)}; "
                      f"control {_q([e[f'{side}_n'] for e in cv])}; share > 0: events "
                      f"{np.mean([e[f'{side}_n'] > 0 for e in ev]):.2f}, control {np.mean([e[f'{side}_n'] > 0 for e in cv]):.2f}")
        for side in ("pre", "post"):
            u = [_union(e, side) for e in E]
            uc = [_union(e, side) for e in C]
            um = []
            for e in E:
                best = max((n for n in ("ay", "yaw") if e[n] is not None), key=lambda n: e[n][f"{side}_n"], default=None)
                um.append(e[best][f"{side}_m"] if best else None)
            print(f"  ay/yaw union {side}: events {_q(u)} [m {_qf(um)}]; control {_q(uc)}")
            pooled[side] += [v for v in u if v is not None]
            pooled[f"{side}_ctl"] += [v for v in uc if v is not None]
        # unloading
        az_ev = [(e["az"], a, b) for e, (a, b) in zip(E, evs) if e["az"] is not None]
        trip_lo = trip_hi = 0
        down_n, down_to_trip, az_min, down_lo = [], [], [], 0
        for e, a, b in az_ev:
            tr = a + int(np.argmax(c["raw_trip"][a:b + 1]))
            if c["az"][tr] < 1.0:
                trip_lo += 1
            else:
                trip_hi += 1
            down_n.append(e["down_pre_n"])
            if e["down_pre_n"] > 0:
                down_to_trip.append(tr - e["down_i"])
                # minimum strictly before the trip: the trip sample itself is
                # < -0.2 g by definition when the trip comes from below
                az_min.append(float(np.min(c["az"][e["down_i"]:tr])) if tr > e["down_i"] else np.nan)
                down_lo += int(c["az"][tr] < 1.0)
        dn = np.array(down_n)
        print(f"  unloading: raw trip from below (az < -0.2 g) {trip_lo}, from above (> 2.2 g) {trip_hi}; downward az "
              f"excursion reaching back before the dilated onset on {int((dn > 0).sum())}/{dn.size} events "
              f"({np.mean(dn > 0):.2f}); its length before the onset (samples) {_q(dn[dn > 0])}; "
              f"start-to-raw-trip (samples) {_q(down_to_trip)}; az minimum before the trip [g] {_qf(az_min, '{:.2f}')}; "
              f"of these, trip from below {down_lo}/{int((dn > 0).sum())}")
        # named sites
        for site, (s0, s1) in (("C1", C1_SITE), ("4500 m", SITE_4500)):
            rows = []
            for e, (a, b) in zip(E, evs):
                if not (np.isfinite(s[a]) and s0 <= s[a] <= s1):
                    continue
                lap = next((l["lap_number"] for l in c["laps"] if l["start_time"] <= c["time"][a] <= l["end_time"]), None)
                tr = a + int(np.argmax(c["raw_trip"][a:b + 1]))
                rows.append(f"lap {lap} mask {s[a]:.0f}-{s[b]:.0f} m: " + ", ".join(
                    f"{n} -{e[n]['pre_n']}/+{e[n]['post_n']} (-{e[n]['pre_m']:.1f}/+{e[n]['post_m']:.1f} m)"
                    for n in CH if e[n] is not None) +
                    f"; az down {e['az']['down_pre_n'] if e['az'] else 'n/a'} smp, trip {'below' if c['az'][tr] < 1 else 'above'}")
            if rows:
                print(f"  site {site} ({s0:.0f}-{s1:.0f} m):")
                for r in rows:
                    print("    " + r)
        store[key] = {"evs": evs, "E": E}
    print(f"\n== POOLED {', '.join(LABEL[k] for k in keys)}: ay/yaw union extension (samples) "
          f"pre events {_q(pooled['pre'])}, control {_q(pooled['pre_ctl'])}; post events {_q(pooled['post'])}, "
          f"control {_q(pooled['post_ctl'])}")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"part1_{tier}.pkl"), "wb") as fh:
        pickle.dump({"pooled": pooled}, fh)


def variant_masks(c):
    """A (production), B (dilation), C (unloading onset), D (ay/yaw envelope)."""
    sr, kerb = c["sr"], c["kerb"]
    n = len(kerb)
    pre, post = int(round(PRE_MARGIN_S * sr)), int(round(POST_MARGIN_S * sr))
    M = {v: kerb.copy() for v in ("A", "B", "C", "D")}
    for a, b in _runs(kerb):
        M["B"][max(0, a - pre):min(n, b + post + 1)] = True
        e = event_envelope(c, a, b, kerb)
        if e["az"] is not None and e["az"]["down_pre_n"] > 0:
            M["C"][e["az"]["down_i"]:a] = True
        lo = min([e[k]["pre_i"] for k in ("ay", "yaw") if e[k] is not None], default=a)
        hi = max([e[k]["post_i"] for k in ("ay", "yaw") if e[k] is not None], default=b)
        M["D"][lo:hi + 1] = True
    return M


def masks(key):
    """Variant masks and the K re-estimation of both axles under each."""
    import time
    from diagnostics import inspect_cs_kerb_window as kw
    c = _load(key)
    nl, _k, z = kw._inputs(key)
    assert np.array_equal(_k, c["kerb"]), key
    params = kw._params()
    M = variant_masks(c)
    an = _analysis_idx(c) & c["moving_raw"]
    out = {f"mask_{v}": m for v, m in M.items()}
    for v, m in M.items():
        print(f"[{key}] mask {v}: share of moving analysis-lap samples {m[an].mean():.4f}, runs {len(_runs(m))}")
    s_m = nl["s_m"].astype(float)
    for v in ("B", "C", "D"):
        for ax in "fr":
            t0 = time.perf_counter()
            r = kw.cs_triple(nl[f"alpha_{ax}"].astype(float), z[f"fy_{ax}"], c["moving_raw"] & ~M[v], M[v], s_m,
                             params, nl["min_window"], variants=["K"])
            out[f"{v}_{ax}"] = r["K"].astype(np.float64)
            print(f"[{key}] {v} {ax}: {time.perf_counter() - t0:.0f}s, finite {np.isfinite(r['K']).mean():.4f}")
    np.savez_compressed(os.path.join(OUT_DIR, "cache", f"cs_{key}.npz"), **out)


def _load_variants(key):
    # A under K is the (g) step-0 K array: same mask, same machinery
    from diagnostics import inspect_cs_kerb_window as kw
    g = kw._load_var(key)
    z = np.load(os.path.join(OUT_DIR, "cache", f"cs_{key}.npz"))
    cs = {"BASE": g["BASE"], "A": g["K"]}
    for v in ("B", "C", "D"):
        cs[v] = (z[f"{v}_f"], z[f"{v}_r"])
    mk = {"BASE": z["mask_A"], "A": z["mask_A"], "B": z["mask_B"], "C": z["mask_C"], "D": z["mask_D"]}
    pt = os.path.join(OUT_DIR, "cache", f"cs_T_{key}.npz")
    if os.path.exists(pt):
        zt = np.load(pt)
        cs["T"], mk["T"] = (zt["T_f"], zt["T_r"]), z["mask_A"]
    return cs, mk


def _site_minima(c, cs, s0, s1):
    t, s = c["time"], c["s_m"]
    out = {}
    for l in c["laps"]:
        if not l.get("is_valid_for_analysis"):
            continue
        m = (t >= l["start_time"]) & (t <= l["end_time"]) & (s >= s0) & (s <= s1)
        out[l["lap_number"]] = [float(np.nanmin(a[m])) if np.isfinite(a[m]).any() else np.nan for a in cs]
    return out


def score(tier, vs=None):
    vs = vs or VARIANTS
    from collections import Counter
    from diagnostics import inspect_cs_kerb_window as kw
    from modules.recommendation import aggregate_by_corner
    from modules.stability_analysis import load_parameters
    from diagnostics.inspect_cs_readout_evidence import phase_index_map, load_supp2, corner_readouts, _calls, _worst
    se = load_parameters()["stability_estimation"]
    gate = se["cs_phase_min_valid_samples"]
    keys = TIER1 if tier == "tier1" else KEYS
    rng = np.random.default_rng(ke.SEED)
    D = {}
    for k in keys:
        nl, _kerb, _z = kw._inputs(k)
        c = _load(k)
        cs, mk = _load_variants(k)
        pim = phase_index_map(nl, load_supp2(k), se)
        summ = {v: kw._variant_summaries(nl, pim, cs[v], gate) for v in vs}
        laps = sorted({l for (_c, l) in pim})
        ro = {v: corner_readouts(kw._r_with(nl, cs[v]), pim, laps, gate) for v in vs}
        own = {v: ke.corner_laps(k, cs=cs[v], kerb=mk[v]) for v in vs}
        prod = {v: ke.corner_laps(k, cs=cs[v]) for v in vs}
        D[k] = {"nl": nl, "c": c, "cs": cs, "mk": mk, "summ": summ, "ro": ro, "own": own, "prod": prod}

    print("=== DATA COST: masked share of moving analysis-lap samples; finite-CS coverage of corner samples; calls")
    for k in keys:
        d = D[k]
        an = _analysis_idx(d["c"]) & d["c"]["moving_raw"]
        ti = np.unique(np.concatenate([np.arange(t["lo"], t["hi"]) for t in d["nl"]["traversals"]]))
        c0 = _calls(aggregate_by_corner(d["summ"]["BASE"]))
        for v in vs:
            cov = [float(np.mean(np.isfinite(d["cs"][v][ai][ti]))) for ai in (0, 1)]
            cv = _calls(aggregate_by_corner(d["summ"][v]))
            cnt = Counter(x[0] for x in cv.values())
            changed = [f"C{cc} {c0[cc][0]}->{cv[cc][0]}" for cc in sorted(cv) if cc in c0 and cv[cc][0] != c0[cc][0]]
            print(f"  {LABEL[k]} {v}: masked {d['mk'][v][an].mean():.4f}; coverage f/r {cov[0]:.3f}/{cov[1]:.3f}; "
                  f"calls US {cnt.get('US', 0)} OS {cnt.get('OS', 0)} ok {cnt.get('-', 0)}; changed vs BASE "
                  f"{len(changed)} {', '.join(changed)}")

    print("\n=== LEG 1 -- near-edge enrichment, own-mask edges (production edges informative), pooled "
          + ", ".join(LABEL[k] for k in keys))
    leg1 = {}
    for v in vs:
        st = {}
        for src in ("own", "prod"):
            pooled = {"f": [], "r": []}
            for k in keys:
                for ax in "fr":
                    pooled[ax] += D[k][src][v][ax]
            st[src] = {ax: ke._stats(pooled[ax], rng) for ax in "fr"}
        leg1[v] = st
        o = st["own"]
        print(f"  {v}: own " + "; ".join(f"{ax} {o[ax]['obs']}/{o[ax]['n']} ({o[ax]['share']:.3f}) vs "
                                       f"{o[ax]['chance']:.3f} -> {o[ax]['enrich']:.2f} (p {o[ax]['p']:.4f})"
                                       for ax in "fr")
              + " | prod-edges " + ", ".join(f"{ax} {st['prod'][ax]['enrich']:.2f}" for ax in "fr"))
        for k in keys:
            per = [ke._stats(D[k]["own"][v][ax], rng) for ax in "fr"]
            print(f"      {LABEL[k]}: f {per[0]['share']:.3f} vs {per[0]['chance']:.3f} ({per[0]['enrich']:.2f}), "
                  f"r {per[1]['share']:.3f} vs {per[1]['chance']:.3f} ({per[1]['enrich']:.2f})")
    b = leg1["BASE"]["own"]
    for v in vs[1:]:
        ok = all((leg1[v]["own"][ax]["enrich"] - 1) <= 0.5 * (b[ax]["enrich"] - 1) for ax in "fr")
        leg1[v]["pass"] = ok
        print(f"  LEG 1 {v}: excess f {b['f']['enrich'] - 1:.2f} -> {leg1[v]['own']['f']['enrich'] - 1:.2f}, "
              f"r {b['r']['enrich'] - 1:.2f} -> {leg1[v]['own']['r']['enrich'] - 1:.2f} -> {'PASS' if ok else 'FAIL'}")

    print("\n=== LEG 2 -- new deep minima > 3 m from the variant's own edges (pass <= 1 per session/axle)")
    leg2 = {v: True for v in vs[1:]}
    for k in keys:
        base = D[k]["own"]["BASE"]
        for v in vs[1:]:
            rows = D[k]["own"][v]
            parts = []
            for ax in "fr":
                bmin = {(r["cid"], r["lap"]): r for r in base[ax]}
                bar = float(np.percentile([r["min"] for r in base[ax]], ke.TAIL_PCT))
                new = [r for r in rows[ax] if (r["cid"], r["lap"]) in bmin and r["min"] < bar
                       and bmin[(r["cid"], r["lap"])]["min"] >= bar and not r["near"]]
                if len(new) > 1:
                    leg2[v] = False
                parts.append(f"{ax} new {len(new)}" + (" [" + "; ".join(
                    f"C{r['cid']} lap {r['lap']} {r['min']:.2f} @ {r['s']:.0f} m" for r in new) + "]" if new else ""))
            print(f"  {LABEL[k]} {v}: " + "; ".join(parts))
    print("  LEG 2 counts: " + ", ".join(f"{v} {'PASS' if ok else 'FAIL'}" for v, ok in leg2.items()))

    print("\n=== LEG 3 -- battery gaps (worst-phase r - f), must stay > 0")
    leg3 = {v: True for v in vs[1:]}
    for key, cid in (("pr_us_v2", 13), ("nbr_jst_solution", 9)):
        if key not in D:
            continue
        parts = []
        for v in vs:
            ro = D[key]["ro"][v][cid]
            g = _worst(ro, "r", "PROD") - _worst(ro, "f", "PROD")
            if v != "BASE" and not g > 0:
                leg3[v] = False
            parts.append(f"{v} {g:+.3f}")
        print(f"  {LABEL[key]} C{cid}: " + ", ".join(parts))
    print("  LEG 3: " + ", ".join(f"{v} {'PASS' if ok else 'FAIL'}" for v, ok in leg3.items()))

    print("\n=== NAMED SITES: per-lap front/rear minima, " + " | ".join(vs))
    for k in keys:
        if not k.startswith("nbr"):
            continue
        for site, (s0, s1) in (("C1", C1_SITE), ("2800 m", SITE_2800), ("4500 m", SITE_4500)):
            per = {v: _site_minima(D[k]["c"], D[k]["cs"][v], s0, s1) for v in vs}
            print(f"  {LABEL[k]} {site}:")
            for lap in per["BASE"]:
                print(f"    lap {lap}: " + " | ".join(f"{per[v][lap][0]:+.2f}/{per[v][lap][1]:+.2f}" for v in vs))
    render(D, keys, vs)
    with open(os.path.join(OUT_DIR, f"score_{tier}_{''.join(vs)}.pkl"), "wb") as fh:
        pickle.dump({"leg1": leg1, "leg2": leg2, "leg3": leg3}, fh)


def render(D, keys, vs):
    from PIL import Image
    from diagnostics.trace_render import render_cs_traces
    for k in keys:
        nl = D[k]["nl"]
        paths = []
        for v in vs:
            p = os.path.join(OUT_DIR, f"traces_{k}_{v}.png")
            render_cs_traces(p, nl["time"], nl["s_m"], D[k]["cs"][v][0], D[k]["cs"][v][1], nl["laps"], nl["corners"],
                             f"{LABEL[k]} {v}")
            paths.append(p)
        ims = [Image.open(p) for p in paths]
        w = max(i.size[0] for i in ims)
        comp = Image.new("RGB", (w, sum(i.size[1] for i in ims)), "white")
        y = 0
        for im in ims:
            comp.paste(im, (0, y))
            y += im.size[1]
        comp.save(os.path.join(OUT_DIR, f"traces_{k}_stack.png"))


def cs_trunc(alpha, Fy, moving, kerb, s_m, params, min_window):
    """Variant T (Part 5 header): production growth, but a window may not hold
    a masked sample -- reaching one ends growth with no estimate. Estimation
    as compute_cs_for_axle (copied after inspect_cs_kerb_window.cs_triple)."""
    from modules.stability_analysis import (_find_monotonic_sections, _smooth_weight,
                                            SPAN_WEIGHT_EXPONENT, R2_WEIGHT_EXPONENT)
    se = params["stability_estimation"]
    min_span, linear_thresh, max_window_m = (se["cs_min_slip_angle_span_rad"], se["cs_linear_slip_threshold_rad"],
                                             se["cs_max_window_m"])
    n = len(alpha)
    valid = ~kerb
    sections, section_id = _find_monotonic_sections(alpha)
    slopes, spans = np.full(len(sections), np.nan), np.zeros(len(sections))
    for k, (a0, b0) in enumerate(sections):
        m = valid[a0:b0]
        if m.sum() < 2:
            continue
        a, f = alpha[a0:b0][m], Fy[a0:b0][m]
        den = np.sum((a - a.mean()) ** 2)
        if den < 1e-10:
            continue
        slopes[k] = np.sum((a - a.mean()) * (f - f.mean())) / den
        spans[k] = a.max() - a.min()
    # last masked index at or before j: a window [st, i) is clean iff st > it
    last_k = np.where(kerb, np.arange(n), -1)
    last_k = np.maximum.accumulate(last_k)
    ref = np.nan
    cs = np.full(n, np.nan)
    for i in range(min_window, n):
        if not moving[i]:
            continue
        floor = last_k[i - 1] + 1
        st = i - min_window
        if st < floor:
            continue
        s_i = s_m[i - 1] if np.isfinite(s_m[i - 1]) else None
        wmax, wmin = np.max(alpha[st:i]), np.min(alpha[st:i])
        while st > 0 and wmax - wmin < min_span:
            if s_i is not None:
                s0 = s_m[st]
                if not np.isfinite(s0) or s0 > s_i or (s_i - s0) >= max_window_m:
                    break
            if st - 1 < floor:
                break
            st -= 1
            wmax, wmin = max(wmax, alpha[st]), min(wmin, alpha[st])
        if wmax - wmin < min_span:
            continue
        wa, wf, ids = alpha[st:i], Fy[st:i], section_id[st:i]
        a_mean, f_mean = np.mean(wa), np.mean(wf)
        denom = np.sum((wa - a_mean) ** 2)
        if denom < 1e-10:
            continue
        C_w = np.sum((wa - a_mean) * (wf - f_mean)) / denom
        f_hat = C_w * wa + (f_mean - C_w * a_mean)
        ss_res, ss_tot = np.sum((wf - f_hat) ** 2), np.sum((wf - f_mean) ** 2)
        R2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-10 else 0.0
        weights, sl = [], []
        for k in np.unique(ids):
            if np.isnan(slopes[k]):
                continue
            w_k = _smooth_weight(spans[k], 0.0, min_span, order=SPAN_WEIGHT_EXPONENT)
            if w_k > 0:
                weights.append(w_k)
                sl.append(slopes[k])
        if weights:
            C_s = float(np.sum(np.array(weights) * np.array(sl)) / np.sum(weights))
            w_r2 = _smooth_weight(R2, 0.0, 1.0, order=R2_WEIGHT_EXPONENT)
            C_a = w_r2 * C_w + (1.0 - w_r2) * C_s
        else:
            C_a = C_w
        if np.max(np.abs(wa)) < linear_thresh:
            ref = C_a
        if not np.isnan(ref) and ref > 0:
            cs[i] = min(C_a / ref, 1.0)
    return cs


def trunc(key):
    import time
    from diagnostics import inspect_cs_kerb_window as kw
    c = _load(key)
    nl, kerb, z = kw._inputs(key)
    params = kw._params()
    s_m = nl["s_m"].astype(float)
    if key == "nbr_jst_solution":
        n0 = kw.SELF_CHECK_N
        a = nl["alpha_f"][:n0].astype(float)
        t0 = cs_trunc(a, z["fy_f"][:n0], nl["moving"][:n0], np.zeros(n0, bool), s_m[:n0], params, nl["min_window"])
        b0 = kw.cs_triple(a, z["fy_f"][:n0], nl["moving"][:n0], np.zeros(n0, bool), s_m[:n0], params, nl["min_window"],
                          variants=["BASE"])["BASE"]
        same = np.array_equal(np.isfinite(t0), np.isfinite(b0)) and np.array_equal(t0[np.isfinite(t0)], b0[np.isfinite(b0)])
        print(f"[{key}] self-check T(no mask) == BASE on {n0} samples: {same}")
    out = {}
    for ax in "fr":
        t1 = time.perf_counter()
        # endpoint gate as production: moving & ~mask (nl["moving"])
        out[f"T_{ax}"] = cs_trunc(nl[f"alpha_{ax}"].astype(float), z[f"fy_{ax}"], nl["moving"], kerb, s_m, params,
                                  nl["min_window"])
        print(f"[{key}] T {ax}: {time.perf_counter() - t1:.0f}s, finite {np.isfinite(out[f'T_{ax}']).mean():.4f}")
    np.savez_compressed(os.path.join(OUT_DIR, "cache", f"cs_T_{key}.npz"), **out)


def _k_start(alpha, i, min_window, min_span, kerb, s_m, max_window_m):
    # K growth rule of inspect_cs_kerb_window.cs_triple, start only
    valid = ~kerb
    st = i - min_window
    m = valid[st:i]
    nv = int(m.sum())
    vmax = np.max(alpha[st:i][m]) if nv else -np.inf
    vmin = np.min(alpha[st:i][m]) if nv else np.inf
    s_i = s_m[i - 1] if np.isfinite(s_m[i - 1]) else None
    while st > 0:
        if nv >= min_window and vmax - vmin >= min_span:
            break
        if s_i is not None:
            s0 = s_m[st]
            if not np.isfinite(s0) or s0 > s_i or (s_i - s0) >= max_window_m:
                break
        st -= 1
        if valid[st]:
            nv += 1
            vmax, vmin = max(vmax, alpha[st]), min(vmin, alpha[st])
    return st


def reach(tier):
    from diagnostics import inspect_cs_kerb_window as kw
    from modules.stability_analysis import load_parameters, reconstruct_cs_window_start
    se = load_parameters()["stability_estimation"]
    keys = TIER1 if tier == "tier1" else KEYS
    for v in ("BASE", "A"):
        pooled = {}
        for k in keys:
            nl, kerb, _z = kw._inputs(k)
            cs, _mk = _load_variants(k)
            s, t = nl["s_m"].astype(float), nl["time"]
            runs = _runs(kerb)
            for ax in "fr":
                alpha = nl[f"alpha_{ax}"].astype(float)
                rows = ke.corner_laps(k, cs=cs[v])[ax]
                arr = cs[v][0 if ax == "f" else 1]
                for r in rows:
                    # recover the sample index of the minimum inside its traversal
                    tr = next(x for x in nl["traversals"] if x["cid"] == r["cid"] and x["lap"] == r["lap"])
                    seg = np.arange(tr["lo"], tr["hi"])
                    ok = seg[np.isfinite(arr[seg]) & np.isfinite(s[seg])]
                    i = int(ok[np.argmin(arr[ok])])
                    if v == "BASE":
                        st = reconstruct_cs_window_start(alpha, i, nl["min_window"], se["cs_min_slip_angle_span_rad"],
                                                         s_m=s, max_window_m=se["cs_max_window_m"])
                    else:
                        st = _k_start(alpha, i, nl["min_window"], se["cs_min_slip_angle_span_rad"], kerb, s,
                                      se["cs_max_window_m"])
                    w = kerb[st:i]
                    km = np.flatnonzero(w)
                    straddle = km.size > 0 and km[0] > 0 and km[-1] < w.size - 1
                    post = int(w.size - 1 - km[-1]) if km.size else 0
                    side = None
                    if r["near"]:
                        # nearest onset/end within +/- 1 s of the minimum, same lap by time
                        cand = [(abs(t[a] - t[i]), "pre-onset" if i < a else "in")
                                for a, b in runs if abs(t[a] - t[i]) < 1.0] + \
                               [(abs(t[b] - t[i]), "post-end" if i > b else "in") for a, b in runs if abs(t[b] - t[i]) < 1.0]
                        side = min(cand)[1] if cand else "far"
                    grp = (ax, "near" if r["near"] else "other")
                    pooled.setdefault(grp, []).append((w.mean() if w.size else 0.0, straddle, post, w.size, side))
        print(f"\n=== {v}: CS window at the per-corner-lap minima, pooled {', '.join(LABEL[k] for k in keys)}")
        for ax in "fr":
            for g in ("near", "other"):
                L = pooled.get((ax, g), [])
                if not L:
                    continue
                sh = np.array([x[0] for x in L]); sd = np.array([x[1] for x in L])
                po = np.array([x[2] for x in L]); n = np.array([x[3] for x in L])
                print(f"  {ax} {g:5s} (n {len(L)}): window holds masked samples {np.mean(sh > 0):.2f}; masked share "
                      f"p50/p90 {np.median(sh):.3f}/{np.percentile(sh, 90):.3f}; STRADDLE {sd.mean():.2f} ({int(sd.sum())}); "
                      f"window length p50 {np.median(n):.0f} smp; post-gap unmasked samples in straddling windows p50 "
                      f"{np.median(po[sd]) if sd.any() else float('nan'):.0f}")
                if g == "near":
                    for sname in ("pre-onset", "post-end", "in", "far"):
                        sub = [x for x in L if x[4] == sname]
                        if sub:
                            print(f"      {sname}: n {len(sub)}, straddle {np.mean([x[1] for x in sub]):.2f}, holds masked "
                                  f"{np.mean([x[0] > 0 for x in sub]):.2f}")


def score_t(tier):
    score(tier, ["BASE", "A", "T"])


if __name__ == "__main__":
    globals()[sys.argv[1]](*sys.argv[2:])
