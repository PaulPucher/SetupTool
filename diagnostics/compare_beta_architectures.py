# Beta architecture comparison A / B / B2 / C (author work order 2026-10-07).
# OFFLINE DIAGNOSTIC: nothing here enters production; numbers only, the
# verdict is the author's.
#
# Candidates, all on the arrays the kinematic integrator reads
# (prepare_vehicle_state; d beta/dt = ay / vx - r, PRODUCTION.md sec. 3):
#   A  production kinematic path as-is: stability_analysis.estimate_sideslip
#      (whole-file integration + 0.05 Hz zero-phase washout), called, not
#      re-implemented.
#   B  per-session yaw-rate and ay offsets removed (CALIBRATION straights
#      only), whole-file integration, no washout.
#   B2 B plus a much weaker washout (production filter function, 0.005 Hz).
#   C  B's correction, beta reset to 0 on every CALIBRATION straight and
#      whenever the car is not moving, piecewise integration, no washout.
#
# Fairness (hard rules of the order): B, C never pass the production washout
# before a metric; no metric uses a production beta or a production-tuned
# threshold as reference or gate -- A competes under the same metrics. The
# steering-defined straights (inspect_kinematic_inputs.steering_straights)
# are split alternately in time: even-numbered segments CALIBRATION (offsets,
# and C's anchors), odd-numbered VALIDATION (residual metrics). C is never
# reset on a validation straight, so its residual there is a real error.
#
# Physics anchor of the residual metric: on a straight driven with steady
# steering the true sideslip is ~0, so |beta| at a validation straight's
# midpoint is the accumulated error of that architecture.
#
# ROUND 2 (author work order 2026-10-07, --round 2, the default): gain
# correction and the app's estimator. Correction form FIXED by the order:
#     beta_dot_corr = ay / v - c1 * r - c0
# with c1, c0 from the M1 fit (y = ay / vx against x = r) on the CALIBRATION
# half of the quasi-steady cornering segments of the same export (segments
# alternate in time, even = calibration). DECLARED ASSUMPTION: the M1
# mismatch is treated as measurement error; transient response is scaled by
# c1 as a side effect. Candidates:
#   A   production kinematic path as-is (baseline)
#   B2  round 1, offsets only + 0.005 Hz washout
#   D   gain + offset correction, whole-file integration, no washout
#   D2  D + 0.005 Hz washout
#   CG  gain + offset correction, reset on calibration straights, no washout
#   E   the beta the app uses: tyre_fit_auto.resolve_sideslip_beta, first
#       return value (sideslip_ekf_pacejka beta_ekf_with_fallback unless the
#       session falls back to the kinematic path) -- a candidate, never a
#       reference.
# B and C of round 1 are settled and not rerun.
#
# Run: python -m diagnostics.compare_beta_architectures [--png] [--round 1|2]

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_kinematic_inputs import (  # noqa: E402
    OUT_DIR, SUSTAINED_AY_MPS2, load, quasi_steady_segments, robust_sigma, runs, steering_straights, track_label,
)

EXPORTS = (
    "D:/Data_Bachelorthesis/unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt",
    "D:/Data_Bachelorthesis/paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt",
)
B2_CUTOFF_HZ = 0.005        # "much weaker" washout, a decade below production
N_CORNERS = 5
EDGE_S = 1.0                # phase-cleanliness window before entry / after exit
PURITY_FACTOR = 3.0         # straight-purity flag: |segment yaw mean| > this x robust spread
CANDIDATES = ("A", "B", "B2", "C")
LABELS = {"A": "A production (whole file + 0.05 Hz washout)", "B": "B offsets removed, no washout",
          "B2": "B2 offsets removed + 0.005 Hz washout", "C": "C offsets removed, reset on straights"}


def integrate(rate, reset):
    """Cumulative sum of rate * dt that restarts at 0 wherever reset is True."""
    out = np.empty_like(rate)
    acc = 0.0
    for i in range(rate.size):
        acc = 0.0 if reset[i] else acc + rate[i]
        out[i] = acc
    return out


def offsets(segs, r, ay, sr):
    w = np.array([(b - a) / sr for a, b in segs])
    rm = np.array([np.mean(r[a:b]) for a, b in segs])
    am = np.array([np.mean(ay[a:b]) for a, b in segs])
    return float(np.average(rm, weights=w)), float(np.average(am, weights=w)), rm


def run_export(path, png):
    from modules.stability_analysis import (load_parameters, estimate_sideslip, _highpass_filter,
                                            estimate_slip_angles)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import _base_mask, _speed_window_mask

    parsed, st = load(path)
    raw = load_parameters()
    ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))
    t, sr, v = st["time"], st["sample_rate_hz"], st["v_mps"]
    r = st["yaw_rate_radps"]
    ay = st["ay_mps2"]
    moving = st["moving_mask"]
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    laps = [l for l in parsed["laps"] if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in laps:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    sd = steering_straights(st, laps, lap_id, kerb)
    segs = sorted(sd["segs"])
    cal = segs[0::2]
    val = segs[1::2]
    name = os.path.basename(path)
    print("\n" + "=" * 100)
    print(f"export: {path} ({track_label(path)})")
    print(f"straights (steering-defined, centre {sd['centre']:+.1f} deg, a_thr {sd['a_thr']:.1f} deg, r_thr "
          f"{sd['r_thr']:.1f} deg/s, d_min {sd['d_min']:g} s, v_min {sd['v_min']:.0f} m/s): {len(segs)} segments; "
          f"split alternately in time -> CALIBRATION {len(cal)} (even), VALIDATION {len(val)} (odd)")

    # ---- metric 6: straight purity on the calibration straights
    r_off, ay_off, rm = offsets(cal, r, ay, sr)
    rm_deg = np.degrees(rm)
    spread = robust_sigma(rm_deg)
    flagged = np.abs(rm_deg) > PURITY_FACTOR * spread
    print("\nMETRIC 6 -- straight purity (calibration straights)")
    print(f"  - per-segment yaw-rate means [deg/s]: p5 {np.percentile(rm_deg, 5):+.3f}, p25 "
          f"{np.percentile(rm_deg, 25):+.3f}, p50 {np.median(rm_deg):+.3f}, p75 {np.percentile(rm_deg, 75):+.3f}, "
          f"p95 {np.percentile(rm_deg, 95):+.3f}; robust spread {spread:.3f}")
    print(f"  - flagged (|mean| > {PURITY_FACTOR:g} x spread = {PURITY_FACTOR * spread:.3f} deg/s, probable shallow "
          f"curves): {int(flagged.sum())} of {len(cal)}"
          + ("; " + ", ".join(f"{m:+.2f}" for m in rm_deg[flagged]) if flagged.any() else ""))
    print(f"  - calibration offsets, all segments (used by B / B2 / C): yaw {np.degrees(r_off):+.3f} deg/s, "
          f"ay {ay_off:+.3f} m/s^2")
    if flagged.any() and (~flagged).sum() > 0:
        r2, a2, _ = offsets([s for s, f in zip(cal, flagged) if not f], r, ay, sr)
        print(f"  - calibration offsets, flagged removed (sensitivity only): yaw {np.degrees(r2):+.3f} deg/s, "
              f"ay {a2:+.3f} m/s^2")
    else:
        r2, a2 = r_off, ay_off
        print("  - calibration offsets, flagged removed: identical (nothing flagged)")

    # ---- candidates
    dt = 1.0 / sr
    vs = np.where(moving, v, 1.0)
    rate_raw = np.where(moving, ay / vs - r, 0.0) * dt
    rate_corr = np.where(moving, (ay - ay_off) / vs - (r - r_off), 0.0) * dt
    beta = {"A": estimate_sideslip(st, ep)}
    b_whole = np.where(moving, np.cumsum(rate_corr), 0.0)
    beta["B"] = b_whole
    beta["B2"] = np.where(moving, _highpass_filter(np.cumsum(rate_corr), B2_CUTOFF_HZ, sr), 0.0)
    anchor = ~moving.copy()
    for a, b in cal:
        anchor[a:b] = True
    beta["C"] = integrate(rate_corr, anchor)
    beta_deg = {k: np.degrees(b) for k, b in beta.items()}
    _ = rate_raw

    # ---- metric 1 / 2: validation straights
    mids = np.array([(a + b) // 2 for a, b in val])
    print("\nMETRIC 1 -- |beta| at the midpoint of each VALIDATION straight [deg] "
          f"({len(val)} straights)")
    slopes = {}
    for k in CANDIDATES:
        x = np.abs(beta_deg[k][mids])
        tt = (t[mids] - t[0]) / 60.0
        slope, _icpt = np.polyfit(tt, x, 1) if x.size > 1 else (np.nan, np.nan)
        slopes[k] = slope
        print(f"  - {LABELS[k]}: median {np.median(x):.2f}, p95 {np.percentile(x, 95):.2f}, worst {x.max():.2f}")
    print("\nMETRIC 2 -- drift growth: slope of |beta| at validation straights vs time since file start")
    for k in CANDIDATES:
        print(f"  - {LABELS[k]}: {slopes[k] * 10:+.3f} deg per 10 min")
    if r2 != r_off or a2 != ay_off:
        rc2 = np.where(moving, (ay - a2) / vs - (r - r2), 0.0) * dt
        bb = np.degrees(np.where(moving, np.cumsum(rc2), 0.0))
        cc = np.degrees(integrate(rc2, anchor))
        print(f"  - sensitivity, flagged-removed offsets: validation median |beta| B {np.median(np.abs(bb[mids])):.2f}, "
              f"C {np.median(np.abs(cc[mids])):.2f} deg")

    # ---- metric 3 / 4: five longest sustained-lateral corners
    cands = []
    for c in parsed["corners"]:
        if c["lap_number"] not in {l["lap_number"] for l in laps}:
            continue
        spans = [sp for sp in c["segments"].values() if sp[1] >= sp[0]]
        if not spans:
            continue
        i0 = int(np.searchsorted(t, min(sp[0] for sp in spans)))
        i1 = int(np.searchsorted(t, max(sp[1] for sp in spans)))
        rr = runs(np.abs(ay[i0:i1]) >= SUSTAINED_AY_MPS2)
        if rr:
            a_, b_ = max(rr, key=lambda p: p[1] - p[0])
            cands.append((b_ - a_, c, i0, i1, i0 + a_, i0 + b_))
    cands.sort(key=lambda z: -z[0])
    top = cands[:N_CORNERS]
    print(f"\nMETRIC 3 -- in-corner |beta| on the {N_CORNERS} longest sustained-lateral corners "
          f"(longest run of |ay| >= {SUSTAINED_AY_MPS2:g} m/s^2 per corner instance) [deg, peak / mean]")
    e = int(EDGE_S * sr)
    for n_, c, i0, i1, a, b in top:
        print(f"  - corner C{c.get('stable_corner_id')} lap {c['lap_number']}: run {n_ / sr:.2f} s, mean vx "
              f"{np.mean(v[a:b]):.1f} m/s, mean ay {np.mean(ay[a:b]):+.1f} m/s^2")
        print("      " + "; ".join(f"{k} {np.max(np.abs(beta_deg[k][a:b])):.2f} / {np.mean(np.abs(beta_deg[k][a:b])):.2f}"
                                   for k in CANDIDATES))
    print(f"\nMETRIC 4 -- phase cleanliness: beta in the {EDGE_S:g} s BEFORE corner entry and AFTER corner exit "
          "[deg, mean (signed) / max |.|]")
    for n_, c, i0, i1, a, b in top:
        pre, post = slice(max(0, i0 - e), i0), slice(i1, min(t.size, i1 + e))
        print(f"  - corner C{c.get('stable_corner_id')} lap {c['lap_number']} (in-corner mean beta sign: "
              + ", ".join(f"{k} {np.sign(np.mean(beta_deg[k][a:b])):+.0f}" for k in CANDIDATES) + ")")
        print("      before: " + "; ".join(f"{k} {np.mean(beta_deg[k][pre]):+.2f} / {np.max(np.abs(beta_deg[k][pre])):.2f}"
                                           for k in CANDIDATES))
        print("      after:  " + "; ".join(f"{k} {np.mean(beta_deg[k][post]):+.2f} / {np.max(np.abs(beta_deg[k][post])):.2f}"
                                           for k in CANDIDATES))

    # ---- metric 5: slip-angle range handed to the tyre fit (population as the fit builds it, no fit run)
    fit_mask = _base_mask(st, parsed["laps"]) & _speed_window_mask(st, parsed, ep["tyre_fit_auto"])[0]
    print(f"\nMETRIC 5 -- slip angles each candidate would hand to the tyre fit (fit population: analysis laps, "
          f"moving, off the kerb mask, fit speed window; {int(fit_mask.sum())} samples) [deg]")
    for k in CANDIDATES:
        sl = estimate_slip_angles(st, beta[k], ep)
        parts = []
        for ax_, key in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
            al = np.degrees(sl[key][fit_mask])
            al = al[np.isfinite(al)]
            q = np.percentile(al, [1, 5, 50, 95, 99])
            parts.append(f"{ax_} p1 {q[0]:+.2f}, p5 {q[1]:+.2f}, p50 {q[2]:+.2f}, p95 {q[3]:+.2f}, p99 {q[4]:+.2f}, "
                         f"span p1..p99 {q[4] - q[0]:.2f}")
        print(f"  - {LABELS[k]}:\n      " + "\n      ".join(parts))

    if png:
        plots(name, path, t, beta_deg, top, mids, sr)


def plots(name, path, t, beta_deg, top, mids, sr):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    stem = name.replace(".txt", "")
    title = f"{track_label(path)}, export {stem}"
    colours = {"A": "#d62728", "B": "#1f77b4", "B2": "#2ca02c", "C": "#9467bd"}
    e = int(EDGE_S * sr)

    fig, axs = plt.subplots(len(top), 1, figsize=(9, 2.3 * len(top)))
    for ax, (n_, c, i0, i1, a, b) in zip(np.atleast_1d(axs), top):
        sl = slice(max(0, i0 - e), min(t.size, i1 + e))
        tt = t[sl] - t[i0]
        for k in CANDIDATES:
            ax.plot(tt, beta_deg[k][sl], color=colours[k], lw=1, label=LABELS[k])
        ax.axvspan(t[a] - t[i0], t[b - 1] - t[i0], color="#dddddd", zorder=0)
        ax.axvline(0, color="black", lw=0.6, ls=":")
        ax.axvline(t[i1 - 1] - t[i0], color="black", lw=0.6, ls=":")
        ax.axhline(0, color="black", lw=0.5)
        ax.set_ylabel(f"C{c.get('stable_corner_id')} lap {c['lap_number']}\nsideslip [deg]", fontsize=8)
    np.atleast_1d(axs)[0].legend(fontsize=7, loc="upper right")
    np.atleast_1d(axs)[-1].set_xlabel("time from corner entry [s] (dotted: entry / exit; grey: sustained lateral run)")
    np.atleast_1d(axs)[0].set_title(f"{title}\nsideslip of four architectures on the five longest sustained corners",
                                    fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch_corners_{stem}.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.boxplot([np.abs(beta_deg[k][mids]) for k in CANDIDATES], labels=[k for k in CANDIDATES], whis=(5, 95))
    ax.set_ylabel("|sideslip| at validation-straight midpoints [deg]")
    ax.set_title(f"{title}\nresidual sideslip on held-out straights (true value ~0)", fontsize=10)
    ax.set_xlabel("; ".join(f"{k} = {LABELS[k].split(' ', 1)[1]}" for k in CANDIDATES), fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch_validation_{stem}.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    tt = (t[mids] - t[0]) / 60.0
    for k in CANDIDATES:
        y = np.abs(beta_deg[k][mids])
        ax.scatter(tt, y, s=10, color=colours[k], label=LABELS[k])
        if y.size > 1:
            p = np.polyfit(tt, y, 1)
            ax.plot(tt, np.polyval(p, tt), color=colours[k], lw=1)
    ax.set_xlabel("time since file start [min]")
    ax.set_ylabel("|sideslip| at validation straights [deg]")
    ax.set_title(f"{title}\nresidual sideslip on held-out straights over the session, with linear trend", fontsize=10)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch_drift_{stem}.png"), dpi=120)
    plt.close(fig)


R2_CANDIDATES = ("A", "B2", "D", "D2", "CG", "E")
R2_LABELS = {"A": "A production kinematic (whole file + 0.05 Hz washout)",
             "B2": "B2 offsets only + 0.005 Hz washout",
             "D": "D gain + offset corrected, no washout",
             "D2": "D2 gain + offset corrected + 0.005 Hz washout",
             "CG": "CG gain + offset corrected, reset on straights",
             "E": "E the app's estimator beta"}
R2_COLOURS = {"A": "#d62728", "B2": "#2ca02c", "D": "#1f77b4", "D2": "#17becf", "CG": "#9467bd", "E": "#ff7f0e"}
RESET_BINS_S = ((0.0, 5.0), (5.0, 15.0), (15.0, np.inf))


def five_corners(parsed, t, ay, laps):
    lapset = {l["lap_number"] for l in laps}
    cands = []
    for c in parsed["corners"]:
        if c["lap_number"] not in lapset:
            continue
        spans = [sp for sp in c["segments"].values() if sp[1] >= sp[0]]
        if not spans:
            continue
        i0 = int(np.searchsorted(t, min(sp[0] for sp in spans)))
        i1 = int(np.searchsorted(t, max(sp[1] for sp in spans)))
        rr = runs(np.abs(ay[i0:i1]) >= SUSTAINED_AY_MPS2)
        if rr:
            a_, b_ = max(rr, key=lambda p: p[1] - p[0])
            cands.append((b_ - a_, c, i0, i1, i0 + a_, i0 + b_))
    cands.sort(key=lambda z: -z[0])
    return cands[:N_CORNERS]


def run_export_r2(path, png):
    from modules.stability_analysis import (load_parameters, estimate_sideslip, _highpass_filter,
                                            estimate_slip_angles)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import _base_mask, _speed_window_mask, resolve_sideslip_beta

    parsed, st = load(path)
    raw = load_parameters()
    resolved = resolve_accuracy(raw, None, None)
    ep = apply_resolved_vehicle(raw, resolved)
    t, sr, v = st["time"], st["sample_rate_hz"], st["v_mps"]
    r = st["yaw_rate_radps"]
    ay = st["ay_mps2"]
    moving = st["moving_mask"]
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    laps = [l for l in parsed["laps"] if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in laps:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    sd = steering_straights(st, laps, lap_id, kerb)
    segs = sorted(sd["segs"])
    cal, val = segs[0::2], segs[1::2]
    qs = quasi_steady_segments(st, lap_id, kerb, sd)
    qsegs = sorted(qs["segs"])
    qcal = qsegs[0::2]
    print("\n" + "=" * 100)
    print(f"export: {path} ({track_label(path)})")
    print(f"straights (steering-defined, centre {sd['centre']:+.1f} deg): {len(segs)} -> CALIBRATION {len(cal)} (even), "
          f"VALIDATION {len(val)} (odd); quasi-steady cornering segments {len(qsegs)} -> CALIBRATION {len(qcal)} (even)")

    # ---- metric 6 (carried): straight purity
    r_off, ay_off, rm = offsets(cal, r, ay, sr)
    rm_deg = np.degrees(rm)
    spread = robust_sigma(rm_deg)
    flagged = np.abs(rm_deg) > PURITY_FACTOR * spread
    print("\nMETRIC 6 -- straight purity (calibration straights)")
    print(f"  - per-segment yaw-rate means [deg/s]: p5 {np.percentile(rm_deg, 5):+.3f}, p50 {np.median(rm_deg):+.3f}, "
          f"p95 {np.percentile(rm_deg, 95):+.3f}; robust spread {spread:.3f}; flagged {int(flagged.sum())} of {len(cal)}")
    print(f"  - straight offsets, all segments (used by B2): yaw {np.degrees(r_off):+.3f} deg/s, ay {ay_off:+.3f} m/s^2")
    if flagged.any() and (~flagged).sum() > 0:
        r2_, a2_, _ = offsets([sg for sg, f in zip(cal, flagged) if not f], r, ay, sr)
        print(f"  - straight offsets, flagged removed (sensitivity only): yaw {np.degrees(r2_):+.3f} deg/s, "
              f"ay {a2_:+.3f} m/s^2")

    # ---- correction coefficients: M1 on calibration cornering
    idx = np.concatenate([np.arange(a, b) for a, b in qcal])
    xq = np.degrees(r[idx])
    yq = np.degrees(ay[idx] / v[idx])
    c1, c0 = np.polyfit(xq, yq, 1)
    print(f"\nCORRECTION (M1 on {len(qcal)} calibration cornering segments, {idx.size} samples): c1 = {c1:.4f}, "
          f"c0 = {c0:+.4f} deg/s -> beta_dot_corr = ay / v - {c1:.4f} r - ({c0:+.4f} deg/s)")

    dt = 1.0 / sr
    vs = np.where(moving, v, 1.0)
    rate_b2 = np.where(moving, (ay - ay_off) / vs - (r - r_off), 0.0) * dt
    rate_d = np.where(moving, ay / vs - c1 * r - np.radians(c0), 0.0) * dt
    anchor = ~moving.copy()
    for a, b in cal:
        anchor[a:b] = True
    beta = {"A": estimate_sideslip(st, ep)}
    beta["B2"] = np.where(moving, _highpass_filter(np.cumsum(rate_b2), B2_CUTOFF_HZ, sr), 0.0)
    beta["D"] = np.where(moving, np.cumsum(rate_d), 0.0)
    beta["D2"] = np.where(moving, _highpass_filter(np.cumsum(rate_d), B2_CUTOFF_HZ, sr), 0.0)
    beta["CG"] = integrate(rate_d, anchor)
    src = ep["stability_estimation"]["sideslip_source"]
    beta["E"], _m, _g, fb, fb_reason = resolve_sideslip_beta(st, ep, parsed, src, csv_path=path)
    print(f"  - E read from modules/tyre_fit_auto.resolve_sideslip_beta (sideslip_source {src}), first return value = "
          + (f"KINEMATIC FALLBACK ({fb_reason})" if fb else
             "fit manifest beta_ekf_with_fallback of modules/sideslip_ekf_pacejka (no session fallback)"))
    bd = {k: np.degrees(b) for k, b in beta.items()}

    # ---- metric 1 / 2
    mids = np.array([(a + b) // 2 for a, b in val])
    last_reset = np.maximum.accumulate(np.where(anchor, np.arange(t.size), -1))
    since = np.array([(t[m] - t[last_reset[m]]) if last_reset[m] >= 0 else np.inf for m in mids])
    print(f"\nMETRIC 1 -- |beta| at VALIDATION-straight midpoints [deg] ({len(val)} straights)")
    slopes = {}
    for k in R2_CANDIDATES:
        x = np.abs(bd[k][mids])
        slopes[k] = np.polyfit((t[mids] - t[0]) / 60.0, x, 1)[0]
        print(f"  - {R2_LABELS[k]}: median {np.median(x):.2f}, p95 {np.percentile(x, 95):.2f}, worst {x.max():.2f}")
    print("  - CG by time since its last reset (calibration straight or standstill):")
    for lo, hi in RESET_BINS_S:
        m = (since >= lo) & (since < hi)
        x = np.abs(bd["CG"][mids[m]])
        txt = (f"median {np.median(x):.2f}, p95 {np.percentile(x, 95):.2f}, worst {x.max():.2f}" if m.any()
               else "no straight")
        print(f"      {lo:g}-{hi:g} s: {int(m.sum())} straights, {txt}")
    print("\nMETRIC 2 -- drift slope of |beta| at validation straights vs time since file start [deg per 10 min]")
    for k in R2_CANDIDATES:
        print(f"  - {R2_LABELS[k]}: {slopes[k] * 10:+.3f}")

    # ---- metric 3 / 4
    top = five_corners(parsed, t, ay, laps)
    lr = ep["vehicle"]["cog_to_rear_axle_m"]
    lvl = resolved["levels"]["cog_position"]
    print(f"\nMETRIC 3 -- signed beta on the {N_CORNERS} longest sustained-lateral corners (same selection as round 1) "
          f"[deg, value at peak |beta| / mean]")
    print(f"  - context line: l_r r / v (mean over the run), l_r = {lr:.3f} m = vehicle.cog_to_rear_axle_m after "
          f"accuracy resolution, cog_position Level {lvl} (no setup sheet -> config default). Beta sign "
          f"convention (estimate_slip_angles, single-track, yaw rate counter-clockwise positive): v_y = v sin(beta), "
          f"alpha_r = -atan((v_y - l_r r) / v_x), so in steady cornering beta = l_r r / v - alpha_r; positive beta "
          f"= velocity vector left of the car's x axis.")
    e = int(EDGE_S * sr)
    for n_, c, i0, i1, a, b in top:
        kin = np.degrees(np.mean(lr * r[a:b] / v[a:b]))
        print(f"  - corner C{c.get('stable_corner_id')} lap {c['lap_number']}: run {n_ / sr:.2f} s, mean vx "
              f"{np.mean(v[a:b]):.1f} m/s, mean ay {np.mean(ay[a:b]):+.1f} m/s^2, l_r r / v {kin:+.2f} deg")
        parts = []
        for k in R2_CANDIDATES:
            seg = bd[k][a:b]
            parts.append(f"{k} {seg[np.argmax(np.abs(seg))]:+.2f} / {np.mean(seg):+.2f}")
        print("      " + "; ".join(parts))
    print(f"\nMETRIC 4 -- beta in the {EDGE_S:g} s BEFORE entry / AFTER exit [deg, signed mean / max |.|]")
    for n_, c, i0, i1, a, b in top:
        pre, post = slice(max(0, i0 - e), i0), slice(i1, min(t.size, i1 + e))
        print(f"  - corner C{c.get('stable_corner_id')} lap {c['lap_number']}")
        print("      before: " + "; ".join(f"{k} {np.mean(bd[k][pre]):+.2f} / {np.max(np.abs(bd[k][pre])):.2f}"
                                           for k in R2_CANDIDATES))
        print("      after:  " + "; ".join(f"{k} {np.mean(bd[k][post]):+.2f} / {np.max(np.abs(bd[k][post])):.2f}"
                                           for k in R2_CANDIDATES))

    # ---- metric 5
    fit_mask = _base_mask(st, parsed["laps"]) & _speed_window_mask(st, parsed, ep["tyre_fit_auto"])[0]
    print(f"\nMETRIC 5 -- slip angles handed to the tyre fit ({int(fit_mask.sum())} fit-population samples, no fit run) [deg]")
    for k in R2_CANDIDATES:
        sl = estimate_slip_angles(st, beta[k], ep)
        parts = []
        for ax_, key in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
            al = np.degrees(sl[key][fit_mask])
            al = al[np.isfinite(al)]
            q = np.percentile(al, [1, 5, 50, 95, 99])
            parts.append(f"{ax_} p1 {q[0]:+.2f}, p5 {q[1]:+.2f}, p50 {q[2]:+.2f}, p95 {q[3]:+.2f}, p99 {q[4]:+.2f}, "
                         f"span p1..p99 {q[4] - q[0]:.2f}")
        print(f"  - {R2_LABELS[k]}:\n      " + "\n      ".join(parts))

    if png:
        plots_r2(path, t, bd, top, mids, since, sr)


def plots_r2(path, t, bd, top, mids, since, sr):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    stem = os.path.basename(path).replace(".txt", "")
    title = f"{track_label(path)}, export {stem}"
    e = int(EDGE_S * sr)

    fig, axs = plt.subplots(len(top), 1, figsize=(9, 2.4 * len(top)))
    axs = np.atleast_1d(axs)
    for ax, (n_, c, i0, i1, a, b) in zip(axs, top):
        sl = slice(max(0, i0 - e), min(t.size, i1 + e))
        tt = t[sl] - t[i0]
        for k in R2_CANDIDATES:
            ax.plot(tt, bd[k][sl], color=R2_COLOURS[k], lw=1, label=R2_LABELS[k])
        ax.axvspan(t[a] - t[i0], t[b - 1] - t[i0], color="#dddddd", zorder=0)
        ax.axvline(0, color="black", lw=0.6, ls=":")
        ax.axvline(t[i1 - 1] - t[i0], color="black", lw=0.6, ls=":")
        ax.axhline(0, color="black", lw=0.5)
        ax.set_ylabel(f"C{c.get('stable_corner_id')} lap {c['lap_number']}\nsideslip [deg]", fontsize=8)
    axs[0].legend(fontsize=6, loc="upper right")
    axs[-1].set_xlabel("time from corner entry [s] (dotted: entry / exit; grey: sustained lateral run)")
    axs[0].set_title(f"{title}\nsideslip with gain correction: six architectures on the five longest sustained corners",
                     fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch2_corners_{stem}.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.scatter(since, np.abs(bd["CG"][mids]), s=14, color=R2_COLOURS["CG"], label=R2_LABELS["CG"])
    for lo, _hi in RESET_BINS_S[1:]:
        ax.axvline(lo, color="#999999", lw=0.8, ls=":")
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("time since the last reset (calibration straight or standstill) [s]")
    ax.set_ylabel("|sideslip| at validation straights [deg]")
    ax.set_title(f"{title}\nanchored, gain-corrected sideslip on held-out straights vs time since reset", fontsize=10)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch2_since_reset_{stem}.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    tt = (t[mids] - t[0]) / 60.0
    for k in R2_CANDIDATES:
        y = np.abs(bd[k][mids])
        ax.scatter(tt, y, s=10, color=R2_COLOURS[k], label=R2_LABELS[k])
        p = np.polyfit(tt, y, 1)
        ax.plot(tt, np.polyval(p, tt), color=R2_COLOURS[k], lw=1)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_xlabel("time since file start [min]")
    ax.set_ylabel("|sideslip| at validation straights [deg]")
    ax.set_title(f"{title}\nresidual sideslip on held-out straights over the session (gain-corrected round)",
                 fontsize=10)
    ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"beta_arch2_drift_{stem}.png"), dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--round", type=int, choices=(1, 2), default=2)
    args = ap.parse_args()
    if args.round == 1:
        print("BETA ARCHITECTURE COMPARISON A / B / B2 / C -- offline, numbers only")
        for path in EXPORTS:
            run_export(path, args.png)
    else:
        print("BETA ARCHITECTURE COMPARISON, ROUND 2: A / B2 / D / D2 / CG / E -- offline, numbers only")
        for path in EXPORTS:
            run_export_r2(path, args.png)


if __name__ == "__main__":
    main()
