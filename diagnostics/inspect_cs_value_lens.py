# DIAGNOSTIC, read-only, [keep-reproduces]. Value-level look at CS_ratio:
# NBR outing mapping (Part 0), full-lap traces (Part 1), and -- after the
# author confirms the mapping -- smoothness/reproduction, utilisation
# bands, the NBR A vs B comparison and the C4 fold/loop recheck.
#
# Reads the per-session caches written by inspect_new_laps_cs_sweep.py
# (production chain, ekf_auto_pacejka); nothing in modules/ or config/ is
# touched. VALUE LEVEL ONLY: no verdicts, thresholds or classification in
# any output. Tier B throughout (plots and descriptive statistics).
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_cs_value_lens mapping | traces
#   python -m diagnostics.inspect_cs_value_lens supplement <key>   (ay, axle Fy)
#   python -m diagnostics.inspect_cs_value_lens smooth | util | ab | foldloop
#
# DEFINITIONS (fixed before the Part 2-5 runs):
# - Traversal = one analysis-lap instance of a stable corner, canonical
#   bracket slice; samples used = moving, non-kerb, finite CS_ratio.
# - Roughness: mean |CS(i+1) - CS(i)| over adjacent valid samples (10 ms).
#   Autocorrelation length: on every contiguous valid run >= 50 samples
#   with nonzero spread, the first lag where the autocorrelation of the
#   de-meaned run drops below 1/e, in metres (lag x median ds of the run);
#   runs that never drop within half their length are censored and
#   counted separately. Compared with the median CS window footprint (m)
#   of the same traversal samples.
# - Reproduction: each traversal resampled onto a 1 m grid over the
#   canonical bracket (grid points > 2 m from any valid sample left
#   empty); Pearson r and RMS difference for every lap pair with >= 30
#   common points; per corner the median over pairs.
# - Cornering demand: |ay| >= channels.json corner_detection
#   ay_entry_threshold_g (0.6 g, the corner detector's own entry value),
#   inside brackets, moving, non-kerb. Bands: > 0.5, 0.3-0.5, 0-0.3, < 0.
#   Axle gap = CS_r - CS_f per gated sample (positive = front further past
#   its linear stiffness than rear); per corner: pooled median and per-lap
#   medians.
# - A vs B: corners matched by canonical-bracket overlap (> 50 % of the
#   shorter bracket); per-lap apex_region medians from summarise_corners.
# - Fold/loop: self-crossing count of the time-ordered (alpha, Fy) path of
#   the regression window behind a traversal's minimum-CS sample (the
#   v3 sawtooth script's test); 0 crossings = fold, > 0 = loop.

import os
import sys

import numpy as np

from diagnostics.inspect_new_laps_cs_sweep import SESSIONS, load_all, _strip_unit

OUT_DIR = "diagnostics/plots/cs_value_lens"
NBR_KEYS = ["nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla"]
# Part 1 sessions; all four NBR files are plotted so the figures exist
# whichever mapping the author confirms
TRACE_KEYS = ["dubai", "v3", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla",
              "pr_us_v2", "lmn_mla"]
MAPPING_CHANNELS = ["log_rideheight_rake", "log_rideheight_rake_mc", "l_rake", "l_rake_mc",
                    "log_rideheight_fr", "log_rideheight_rl", "log_rideheight_rr", "ecu_speed"]


def _read_blocks(path, wanted):
    """Streams the raw file and returns {name: (t, v, unit)} for the wanted
    blocks only -- the files are up to 2.7 GB, never held in memory whole."""
    out = {}
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.strip() == "{ChannelBlock}":
                header = f.readline().rstrip("\n").split("\t")
                name = _strip_unit(header[1]) if len(header) == 2 else None
                if name in wanted:
                    ts, vs = [], []
                    line = f.readline()
                    while line and not line.startswith("{"):
                        parts = line.replace(",", ".").split("\t")
                        if len(parts) == 2:
                            try:
                                ts.append(float(parts[0])); vs.append(float(parts[1]))
                            except ValueError:
                                pass
                        line = f.readline()
                    out[name] = (np.array(ts), np.array(vs), header[1])
                    continue
            line = f.readline()
    return out


def _header(path):
    meta = {}
    with open(path, "r", encoding="latin-1") as f:
        for _ in range(20):
            parts = f.readline().strip().split("\t")
            if len(parts) == 2:
                meta[parts[0]] = parts[1]
    return meta


def mapping():
    res = load_all()
    for key in NBR_KEYS:
        path = SESSIONS[key]
        meta = _header(path)
        r = res[key]
        an = [l["lap_number"] for l in r["laps"] if l["is_valid_for_analysis"]]
        times = [l["lap_time"] for l in r["laps"] if l["is_valid_for_analysis"]]
        print(f"\n## {key}: {os.path.basename(path)} ({os.path.getsize(path) / 1e9:.2f} GB)")
        print(f"  header: car {meta.get('CarName')} driver {meta.get('DriverName')} session {meta.get('SessionNumber')} "
              f"outing {meta.get('OutingNumber')} first lap {meta.get('FirstLapNumber')}")
        print(f"  laps in file {len(r['laps'])}: {[l['lap_number'] for l in r['laps']]}; analysis laps {an}; "
              f"lap times {[round(t, 2) if t else None for t in times]}")
        blocks = _read_blocks(path, set(MAPPING_CHANNELS))
        spd = blocks.get("ecu_speed")
        for name in MAPPING_CHANNELS[:-1]:
            if name not in blocks:
                continue
            t, v, unit = blocks[name]
            if v.size == 0:
                continue
            line = f"  {name} [{unit}] n={v.size}: median {np.median(v):.4g}"
            if spd is not None and spd[0].size:
                # rake depends on aero load: compare within speed bands
                vs = np.interp(t, spd[0], spd[1])
                for lo, hi in ((100, 150), (150, 200), (200, 260)):
                    m = (vs >= lo) & (vs < hi)
                    if m.sum() > 50:
                        line += f"; {lo}-{hi} kph median {np.median(v[m]):.4g} (n={m.sum()})"
            print(line)


def traces():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.stability_analysis import load_parameters
    # fixed display range = the app's own corner-trace CS panel range (config)
    y_lo, y_hi = None, None
    params = load_parameters()
    for sect in params.values():
        if isinstance(sect, dict) and "cs_ls_panel_y_min" in sect:
            y_lo, y_hi = sect["cs_ls_panel_y_min"], sect["cs_ls_panel_y_max"]
    res = load_all()
    os.makedirs(OUT_DIR, exist_ok=True)
    ink, band, ref = "#1f2937", "#e5e7eb", "#6b7280"
    for key in TRACE_KEYS:
        r = res[key]
        t, s = r["time"], r["s_m"]
        csf, csr = (a.astype(float) for a in r["cs"]["A_baseline"])
        an = [l for l in r["laps"] if l["is_valid_for_analysis"]]
        lap_len = np.nanmax(s[(t >= an[0]["start_time"]) & (t <= an[0]["end_time"])]) if an else 1.0
        fig, axes = plt.subplots(2, 1, figsize=(max(14, lap_len / 400), 7), sharex=True, constrained_layout=True)
        brackets = sorted({(c["bracket_start_m"], c["bracket_end_m"]) for c in r["corners"]
                           if c["bracket_start_m"] is not None and c["lap_number"] in {l["lap_number"] for l in an}})
        clipped = {"f": 0, "r": 0}
        for ax, arr, lab, ak in ((axes[0], csf, "front CS_ratio", "f"), (axes[1], csr, "rear CS_ratio", "r")):
            for b0, b1 in brackets:
                ax.axvspan(b0, b1, color=band, lw=0, zorder=0)
            ax.axhline(0.0, color=ref, lw=0.8, zorder=1)
            ax.axhline(1.0, color=ref, lw=0.8, ls="--", zorder=1)
            for l in an:
                m = (t >= l["start_time"]) & (t <= l["end_time"])
                y = arr[m]
                clipped[ak] += int(np.sum(np.isfinite(y) & (y < y_lo)))
                ax.plot(s[m], y, color=ink, lw=0.5, alpha=0.6, zorder=2)
            ax.set_ylim(y_lo, y_hi)
            ax.set_ylabel(lab, color=ink)
            ax.grid(axis="y", color=band, lw=0.6)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        axes[1].set_xlabel("lap distance [m]", color=ink)
        fig.suptitle(f"{key}: CS_ratio per analysis lap (n={len(an)}), grey bands = corner brackets, "
                     f"0 = force peak, 1 = linear; y fixed {y_lo}..{y_hi}", color=ink, fontsize=10)
        p = os.path.join(OUT_DIR, f"trace_{key}.png")
        fig.savefig(p, dpi=110)
        plt.close(fig)
        print(f"{key}: {p}; laps {len(an)}; lap length {lap_len:.0f} m; samples below {y_lo}: front {clipped['f']}, rear {clipped['r']}")


LABELS = {
    "dubai": "Dubai", "v3": "Paul Ricard v3 (session 7)", "pr_us_v2": "Paul Ricard US-v2 (session 10)",
    "nbr_jst_problem": "NBR A (JST outing 21)", "nbr_jst_solution": "NBR B (JST outing 29)",
    "lmn_mla": "Le Mans (context, 1 lap)",
    "nbr_qp_cdr": "NBR qualifying CDR (session 5 outing 69, other driver; context)",
    "nbr_qp_mla": "NBR qualifying MLA (session 5 outing 70, other driver; context)",
}
LENS_KEYS = ["dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "lmn_mla", "nbr_qp_cdr", "nbr_qp_mla"]
AXLES = (("f", 0), ("r", 1))


def _valid_idx(r, tr):
    idx = np.arange(tr["lo"], tr["hi"])
    return idx[r["moving"][idx]]


def _by_corner(r):
    out = {}
    for tr in r["traversals"]:
        out.setdefault(tr["cid"], []).append(tr)
    return out


def _acf_length_m(y, s):
    """First lag where the autocorrelation drops below 1/e, in metres;
    None when it does not within half the run (censored)."""
    d = y - y.mean()
    var = np.dot(d, d)
    if var <= 1e-12:
        return None
    n = d.size
    ds = float(np.median(np.diff(s)))
    for k in range(1, n // 2):
        if np.dot(d[:-k], d[k:]) / var < np.exp(-1):
            return k * ds
    return None


def _runs(mask):
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def _resample(r, tr, arr, grid):
    idx = _valid_idx(r, tr)
    y = arr[idx]
    ok = np.isfinite(y)
    s, y = r["s_m"][idx][ok], y[ok]
    if s.size < 5:
        return np.full(grid.size, np.nan)
    order = np.argsort(s)
    s, y = s[order], y[order]
    out = np.interp(grid, s, y)
    pos = np.clip(np.searchsorted(s, grid), 1, s.size - 1)
    near = np.minimum(np.abs(grid - s[pos - 1]), np.abs(grid - s[pos]))
    out[(near > 2.0) | (grid < s[0]) | (grid > s[-1])] = np.nan
    return out


def _footprint_lookup(r, axle):
    tm = r["term"][axle]
    return dict(zip(r["term_idx"].tolist(), tm["footprint_m"].tolist()))


def smooth():
    """Part 2: roughness, autocorrelation length vs window, reproduction,
    and the worked figure."""
    res = load_all()
    repro_all = {}
    for key in LENS_KEYS:
        r = res[key]
        print(f"\n## {LABELS[key]}")
        byc = _by_corner(r)
        repro_all[key] = {}
        for axle, ai in AXLES:
            arr = r["cs"]["A_baseline"][ai].astype(float)
            fp = _footprint_lookup(r, axle)
            rows = []
            for cid, trs in sorted(byc.items()):
                steps, acfs, cens, fps, clip = [], [], 0, [], []
                for tr in trs:
                    idx = _valid_idx(r, tr)
                    y = arr[idx]
                    ok = np.isfinite(y)
                    adj = ok[1:] & ok[:-1] & (np.diff(idx) == 1)
                    steps.extend(np.abs(np.diff(y))[adj].tolist())
                    clip.extend((y[ok] >= 1.0).tolist())
                    fps.extend([fp[i] for i in idx[ok] if i in fp and np.isfinite(fp[i])])
                    for a, b in _runs(ok & np.concatenate(([True], np.diff(idx) == 1))):
                        if b - a >= 50:
                            L = _acf_length_m(y[a:b], r["s_m"][idx[a:b]])
                            if L is None:
                                cens += 1
                            else:
                                acfs.append(L)
                grid = np.arange(trs[0]["bracket"][0], trs[0]["bracket"][1], 1.0)
                curves = [_resample(r, tr, arr, grid) for tr in trs]
                rs, rms = [], []
                for i in range(len(curves)):
                    for j in range(i + 1, len(curves)):
                        m = np.isfinite(curves[i]) & np.isfinite(curves[j])
                        if m.sum() >= 30 and np.std(curves[i][m]) > 0 and np.std(curves[j][m]) > 0:
                            rs.append(float(np.corrcoef(curves[i][m], curves[j][m])[0, 1]))
                            rms.append(float(np.sqrt(np.mean((curves[i][m] - curves[j][m]) ** 2))))
                rows.append({"cid": cid, "step": np.mean(steps) if steps else np.nan,
                             "acf": np.median(acfs) if acfs else np.nan, "n_acf": len(acfs), "cens": cens,
                             "fp": np.median(fps) if fps else np.nan, "clip": np.mean(clip) if clip else np.nan,
                             "r": np.median(rs) if rs else np.nan, "rms": np.median(rms) if rms else np.nan,
                             "pairs": len(rs), "br_len": grid.size})
            repro_all[key][axle] = rows
            st = np.array([x["step"] for x in rows]); ac = np.array([x["acf"] for x in rows])
            fpa = np.array([x["fp"] for x in rows]); rr = np.array([x["r"] for x in rows])
            ratio = ac / fpa
            print(f"  axle {axle}: median over corners -- step {np.nanmedian(st):.4f}/sample, ACF length "
                  f"{np.nanmedian(ac):.1f} m, window footprint {np.nanmedian(fpa):.1f} m, ACF/window "
                  f"{np.nanmedian(ratio):.2f} (corners with ACF < window: {int(np.nansum(ratio < 1))}/"
                  f"{int(np.sum(np.isfinite(ratio)))}), bracket length {np.median([x['br_len'] for x in rows]):.0f} m, "
                  f"censored runs {sum(x['cens'] for x in rows)} of {sum(x['cens'] + x['n_acf'] for x in rows)}, "
                  f"share at 1.0 clip {np.nanmean([x['clip'] for x in rows]):.2f}")
            if np.isfinite(rr).any():
                srt = sorted([x for x in rows if np.isfinite(x["r"])], key=lambda x: -x["r"])
                print(f"    lap-to-lap median pairwise r: session median {np.nanmedian(rr):.2f}; highest "
                      + ", ".join(f"C{x['cid']} {x['r']:.2f} (rms {x['rms']:.2f}, {x['pairs']} pairs)" for x in srt[:3])
                      + "; lowest " + ", ".join(f"C{x['cid']} {x['r']:.2f} (rms {x['rms']:.2f})" for x in srt[-3:]))
                print("    all corners r: " + ", ".join(f"C{x['cid']}={x['r']:.2f}" for x in rows if np.isfinite(x['r'])))
            else:
                print("    lap-to-lap reproduction: not assessable (fewer than 2 analysis laps)")
    _worked_figure(res, repro_all)
    _jump_attribution(res)


def _jump_attribution(res):
    """Share of the total |dCS| that falls on window-start jumps: adjacent
    samples where the regression footprint shrinks by more than 2 m, i.e.
    the start leapt forward because an old alpha extreme left the window."""
    print("\n## window-start jump attribution (|dCS| mass on footprint drops > 2 m)")
    for key in LENS_KEYS:
        r = res[key]
        idx = r["term_idx"]
        adj = np.diff(idx) == 1
        parts = []
        for axle, ai in AXLES:
            cs = r["cs"]["A_baseline"][ai].astype(float)[idx]
            fp = r["term"][axle]["footprint_m"]
            d_cs = np.abs(np.diff(cs))
            d_fp = np.diff(fp)
            ok = adj & np.isfinite(d_cs) & np.isfinite(d_fp)
            jump = ok & (d_fp < -2.0)
            share_samples = jump.sum() / max(ok.sum(), 1)
            share_mass = d_cs[jump].sum() / max(d_cs[ok].sum(), 1e-12)
            parts.append(f"{axle}: jump samples {share_samples:.3f} carry {share_mass:.2f} of |dCS|")
        print(f"  {LABELS[key]}: " + "; ".join(parts))


def _worked_figure(res, repro_all):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.stability_analysis import load_parameters
    cap = load_parameters()["stability_estimation"]["cs_max_window_m"]
    r = res["dubai"]
    rows = [x for x in repro_all["dubai"]["f"] if np.isfinite(x["r"])]
    best = max(rows, key=lambda x: x["r"])["cid"]
    worst = min(rows, key=lambda x: x["r"])["cid"]
    byc = _by_corner(r)
    arr = r["cs"]["A_baseline"][0].astype(float)
    fp = _footprint_lookup(r, "f")
    ink, ref = "#1f2937", "#6b7280"
    styles = ["-", "--", ":", "-."]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex="col", constrained_layout=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    for col, cid in enumerate((best, worst)):
        rr = next(x for x in repro_all["dubai"]["f"] if x["cid"] == cid)["r"]
        for k, tr in enumerate(sorted(byc[cid], key=lambda t: t["lap"])):
            idx = _valid_idx(r, tr)
            s = r["s_m"][idx]
            # laps told apart by line style, not colour
            axes[0, col].plot(s, arr[idx], color=ink, lw=0.9, ls=styles[k % 4], label=f"lap {tr['lap']}")
            w = np.array([fp.get(i, np.nan) for i in idx])
            axes[1, col].plot(s, w, color=ink, lw=0.9, ls=styles[k % 4])
        axes[0, col].axhline(0, color=ref, lw=0.8); axes[0, col].axhline(1, color=ref, lw=0.8, ls="--")
        axes[1, col].axhline(cap, color=ref, lw=0.8, ls="--")
        axes[1, col].text(axes[1, col].get_xlim()[0], cap, f" cs_max_window_m {cap:.0f} m", va="bottom", fontsize=8, color=ref)
        axes[0, col].set_title(f"Dubai C{cid} front, all analysis laps (median pairwise r = {rr:.2f})", color=ink, fontsize=10)
        axes[0, col].set_ylabel("CS_ratio front", color=ink)
        axes[1, col].set_ylabel("regression window [m]", color=ink)
        axes[1, col].set_xlabel("lap distance [m]", color=ink)
        axes[0, col].legend(frameon=False, fontsize=8)
        for ax in axes[:, col]:
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
    p = os.path.join(OUT_DIR, "worked_corner_dubai.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"\nworked figure: {p} (C{best} highest r, C{worst} lowest r, Dubai front)")


def _load_supp(key, r):
    z = np.load(os.path.join(OUT_DIR, f"supp_{key}.npz"))
    if z["time"].size != r["time"].size or not np.allclose(z["time"][[0, -1]], r["time"][[0, -1]]):
        raise RuntimeError(f"{key}: supplement grid differs from the sweep cache")
    return z


def util():
    """Part 3, exploratory: demand-gated band shares and the axle gap."""
    import json
    ay_gate = json.load(open("config/channels.json", encoding="utf-8"))["corner_detection"]["ay_entry_threshold_g"]
    res = load_all()
    bands = ((">0.5", lambda x: x > 0.5), ("0.3-0.5", lambda x: (x >= 0.3) & (x <= 0.5)),
             ("0-0.3", lambda x: (x >= 0) & (x < 0.3)), ("<0", lambda x: x < 0))
    print(f"cornering-demand gate |ay| >= {ay_gate} g (channels.json corner_detection.ay_entry_threshold_g)")
    out = {}
    for key in LENS_KEYS:
        r = res[key]
        z = _load_supp(key, r)
        gate_all = r["moving"] & (np.abs(z["ay"]) / 9.81 >= ay_gate)
        csf, csr = (a.astype(float) for a in r["cs"]["A_baseline"])
        print(f"\n## {LABELS[key]}")
        rows = []
        for cid, trs in sorted(_by_corner(r).items()):
            idx = np.concatenate([np.arange(t["lo"], t["hi"]) for t in trs])
            g = idx[gate_all[idx]]
            prof = {}
            for axle, arr in (("f", csf), ("r", csr)):
                v = arr[g]; v = v[np.isfinite(v)]
                prof[axle] = [float(np.mean(fn(v))) if v.size else np.nan for _, fn in bands]
            both = g[np.isfinite(csf[g]) & np.isfinite(csr[g])]
            gap = csr[both] - csf[both]
            lap_gaps = []
            for t in trs:
                gi = np.arange(t["lo"], t["hi"])
                gi = gi[gate_all[gi] & np.isfinite(csf[gi]) & np.isfinite(csr[gi])]
                if gi.size >= 5:
                    lap_gaps.append(float(np.median(csr[gi] - csf[gi])))
            rows.append({"cid": cid, "n": int(g.size), "prof": prof, "gap": float(np.median(gap)) if gap.size else np.nan,
                         "lap_gaps": lap_gaps})
        out[key] = rows
        pooled = {a: np.nanmean([x["prof"][a] for x in rows], axis=0) for a in ("f", "r")}
        print("  session mean band shares front " + " / ".join(f"{b[0]} {v:.2f}" for b, v in zip(bands, pooled["f"]))
              + "; rear " + " / ".join(f"{b[0]} {v:.2f}" for b, v in zip(bands, pooled["r"])))
        print("  per corner (n gated, front bands | rear bands | pooled median gap r-f | per-lap median gaps):")
        for x in rows:
            print(f"    C{x['cid']:>2} n={x['n']:>5} f " + "/".join(f"{v:.2f}" for v in x["prof"]["f"])
                  + " | r " + "/".join(f"{v:.2f}" for v in x["prof"]["r"])
                  + f" | gap {x['gap']:+.3f} | laps " + ", ".join(f"{v:+.2f}" for v in x["lap_gaps"]))
        top = sorted([x for x in rows if np.isfinite(x["gap"]) and len(x["lap_gaps"]) >= 1],
                     key=lambda x: -x["gap"])[:3]
        print("  largest front-lower gaps: " + "; ".join(
            f"C{x['cid']} {x['gap']:+.3f}, same sign on {sum(1 for v in x['lap_gaps'] if v > 0)}/{len(x['lap_gaps'])} laps"
            for x in top))
    import pickle
    with open(os.path.join(OUT_DIR, "util.pkl"), "wb") as fh:
        pickle.dump(out, fh)


def _brackets(r):
    br = {}
    for c in r["corners"]:
        if c["stable_corner_id"] is not None and c["bracket_start_m"] is not None:
            br.setdefault(c["stable_corner_id"], (c["bracket_start_m"], c["bracket_end_m"]))
    return br


def ab():
    """Part 4: NBR A (outing 21) vs B (outing 29), apex_region medians per lap."""
    import pickle
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = load_all()
    A, B = res["nbr_jst_problem"], res["nbr_jst_solution"]
    ba, bb = _brackets(A), _brackets(B)
    pairs = []
    for ca, (a0, a1) in sorted(ba.items()):
        best = None
        for cb, (b0, b1) in bb.items():
            ov = min(a1, b1) - max(a0, b0)
            if ov > 0.5 * min(a1 - a0, b1 - b0) and (best is None or ov > best[1]):
                best = (cb, ov)
        if best:
            pairs.append((ca, best[0]))
    print(f"matched corners {len(pairs)} of A {len(ba)} / B {len(bb)}: " + ", ".join(f"A C{a}=B C{b}" for a, b in pairs))

    def per_lap(r, cid):
        vals = {"f": [], "r": []}
        for s in r["summaries"]["A_baseline"]:
            if s.get("stable_corner_id") == cid and s.get("apex_region"):
                vals["f"].append(s["apex_region"]["cs_ratio_f"]["median"])
                vals["r"].append(s["apex_region"]["cs_ratio_r"]["median"])
        return vals

    util_rows = pickle.load(open(os.path.join(OUT_DIR, "util.pkl"), "rb"))
    ug = {k: {x["cid"]: x for x in util_rows[k]} for k in ("nbr_jst_problem", "nbr_jst_solution")}
    rows = []
    for ca, cb in pairs:
        va, vb = per_lap(A, ca), per_lap(B, cb)
        med = lambda v: float(np.nanmedian(v)) if np.isfinite(v).any() else np.nan
        fa, fb = med(np.array(va["f"])), med(np.array(vb["f"]))
        ra, rb = med(np.array(va["r"])), med(np.array(vb["r"]))
        rows.append({"ca": ca, "cb": cb, "va": va, "vb": vb, "fa": fa, "fb": fb, "ga": ra - fa, "gb": rb - fb,
                     "iqr_fb": float(np.nanpercentile(vb["f"], 75) - np.nanpercentile(vb["f"], 25)) if np.isfinite(vb["f"]).sum() >= 2 else np.nan,
                     "gate_ga": ug["nbr_jst_problem"].get(ca, {}).get("gap", np.nan),
                     "gate_gb": ug["nbr_jst_solution"].get(cb, {}).get("gap", np.nan)})
        x = rows[-1]
        fmt = lambda v: "[" + ", ".join("nan" if not np.isfinite(t) else f"{t:+.2f}" for t in v) + "]"
        print(f"  A C{ca} / B C{cb}: front A {fmt(va['f'])} B {fmt(vb['f'])} (medians {x['fa']:+.2f} -> {x['fb']:+.2f}, B IQR {x['iqr_fb']:.2f}); "
              f"rear A {fmt(va['r'])} B {fmt(vb['r'])}; apex gap r-f {x['ga']:+.2f} -> {x['gb']:+.2f}; "
              f"demand-gated gap {x['gate_ga']:+.3f} -> {x['gate_gb']:+.3f}")
    ok = [x for x in rows if np.isfinite(x["fa"]) and np.isfinite(x["fb"])]
    okg = [x for x in rows if np.isfinite(x["ga"]) and np.isfinite(x["gb"])]
    okd = [x for x in rows if np.isfinite(x["gate_ga"]) and np.isfinite(x["gate_gb"])]
    print(f"\nfront apex median higher in B: {sum(1 for x in ok if x['fb'] > x['fa'])}/{len(ok)} corners; "
          f"median over corners A {np.median([x['fa'] for x in ok]):+.3f} B {np.median([x['fb'] for x in ok]):+.3f}")
    print(f"apex gap r-f smaller in B: {sum(1 for x in okg if x['gb'] < x['ga'])}/{len(okg)}; "
          f"median A {np.median([x['ga'] for x in okg]):+.3f} B {np.median([x['gb'] for x in okg]):+.3f}")
    print(f"demand-gated gap smaller in B: {sum(1 for x in okd if x['gate_gb'] < x['gate_ga'])}/{len(okd)}; "
          f"median A {np.median([x['gate_ga'] for x in okd]):+.3f} B {np.median([x['gate_gb'] for x in okd]):+.3f}")
    big = [x for x in ok if np.isfinite(x["iqr_fb"]) and abs(x["fb"] - x["fa"]) > x["iqr_fb"]]
    print(f"corners where |front change| exceeds B's own lap-to-lap IQR: {len(big)}/{len(ok)} "
          + ", ".join(f"C{x['ca']} ({x['fa']:+.2f}->{x['fb']:+.2f})" for x in big))

    ink = "#1f2937"
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    xs = np.arange(len(rows))
    for k, x in enumerate(rows):
        for ax, key in ((axes[0], "f"), (axes[1], "gap")):
            if key == "f":
                ya, yb = x["va"]["f"], x["vb"]["f"]
            else:
                ya = [rr - ff for rr, ff in zip(x["va"]["r"], x["va"]["f"])]
                yb = [rr - ff for rr, ff in zip(x["vb"]["r"], x["vb"]["f"])]
            # A open circles left of centre, B filled right: shape + position, not colour
            ax.plot(np.full(len(ya), k - 0.15), ya, "o", mfc="white", mec=ink, ms=6,
                    label="A (outing 21), per lap" if k == 0 else None)
            ax.plot(np.full(len(yb), k + 0.15), yb, "o", mfc=ink, mec=ink, ms=4,
                    label="B (outing 29), per lap" if k == 0 else None)
    for ax, lab in ((axes[0], "apex_region CS_ratio front"), (axes[1], "apex gap CS_r - CS_f")):
        ax.axhline(0, color="#6b7280", lw=0.8)
        ax.set_ylabel(lab, color=ink)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[0].axhline(1, color="#6b7280", lw=0.8, ls="--")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].set_xticks(xs, [f"C{x['ca']}" for x in rows])
    axes[1].set_xlabel("matched corner (A numbering)", color=ink)
    fig.suptitle("NBR A vs B, per-lap apex-region values (value level, no thresholds)", color=ink, fontsize=10)
    p = os.path.join(OUT_DIR, "nbr_a_vs_b.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"figure: {p}")


def foldloop():
    """Part 5: (alpha, Fy) path of the window behind the minimum-CS sample."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.stability_analysis import load_parameters, reconstruct_cs_window_start
    from diagnostics.inspect_v3_sawtooth_mechanism import self_crossing_count
    se = load_parameters()["stability_estimation"]
    res = load_all()

    def instance(key, tr, axle):
        r = res[key]
        ai = 0 if axle == "f" else 1
        cs = r["cs"]["A_baseline"][ai].astype(float)
        idx = _valid_idx(r, tr)
        idx = idx[np.isfinite(cs[idx])]
        if idx.size == 0:
            return None
        i = int(idx[np.argmin(cs[idx])])
        alpha = r["alpha_" + axle].astype(float)
        st = reconstruct_cs_window_start(alpha, i, r["min_window"], se["cs_min_slip_angle_span_rad"],
                                         s_m=r["s_m"], max_window_m=se["cs_max_window_m"])
        fy = _load_supp(key, r)["fy_" + axle]
        a, f = alpha[st:i], fy[st:i]
        # a path whose alpha never reverses cannot self-cross: count
        # reversals so a "fold" can be told from a test without power
        da = np.sign(np.diff(a))
        da = da[da != 0]
        return {"key": key, "cid": tr["cid"], "lap": tr["lap"], "axle": axle, "cs": float(cs[i]),
                "n": i - st, "span_deg": float(np.degrees(a.max() - a.min())),
                "reversals": int(np.sum(da[1:] != da[:-1])),
                "cross": self_crossing_count(a, f), "a": a, "f": f}

    insts = []
    dub = res["dubai"]
    for tr in sorted([t for t in dub["traversals"] if t["cid"] == 4], key=lambda t: t["lap"]):
        for axle in ("f", "r"):
            x = instance("dubai", tr, axle)
            if x:
                insts.append(x)
    cand = []
    for key in ("nbr_jst_problem", "nbr_jst_solution"):
        for tr in res[key]["traversals"]:
            for axle in ("f", "r"):
                x = instance(key, tr, axle)
                if x:
                    cand.append(x)
    cand.sort(key=lambda x: x["cs"])
    seen = set()
    for x in cand:
        k = (x["key"], x["cid"], x["lap"])
        if k not in seen:
            seen.add(k); insts.append(x)
        if len(seen) == 2:
            break
    for x in insts:
        print(f"  {LABELS[x['key']]} C{x['cid']} lap {x['lap']} {x['axle']}: min CS {x['cs']:+.3f}, window n={x['n']} "
              f"samples, alpha span {x['span_deg']:.2f} deg, alpha reversals {x['reversals']}, self-crossings {x['cross']} -> "
              f"{'fold' if x['cross'] == 0 else 'loop'}; |alpha| {abs(np.degrees(x['a'][0])):.2f} -> "
              f"{abs(np.degrees(x['a'][-1])):.2f} deg, |Fy| {abs(x['f'][0]) / 1000:.2f} -> {abs(x['f'][-1]) / 1000:.2f} kN")
    ink = "#1f2937"
    ncol = 4
    nrow = int(np.ceil(len(insts) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(14, 3.3 * nrow), constrained_layout=True)
    for ax, x in zip(axes.ravel(), insts):
        ax.plot(np.degrees(x["a"]), x["f"] / 1000, color=ink, lw=1.0)
        ax.plot(np.degrees(x["a"][0]), x["f"][0] / 1000, "o", mfc="white", mec=ink, ms=6)
        ax.plot(np.degrees(x["a"][-1]), x["f"][-1] / 1000, "o", mfc=ink, mec=ink, ms=6)
        short = {"dubai": "Dubai", "nbr_jst_problem": "NBR A", "nbr_jst_solution": "NBR B"}[x["key"]]
        ax.set_title(f"{short} C{x['cid']} lap {x['lap']} {x['axle']}: CS {x['cs']:+.2f}, crossings {x['cross']}",
                     fontsize=9, color=ink)
        ax.set_xlabel("alpha [deg]", fontsize=8); ax.set_ylabel("Fy axle [kN]", fontsize=8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    for ax in axes.ravel()[len(insts):]:
        ax.axis("off")
    fig.suptitle("Window path behind each minimum-CS sample (open = window start, filled = end)", color=ink, fontsize=10)
    p = os.path.join(OUT_DIR, "fold_loop.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"figure: {p}")


def supplement(key):
    """ay and filtered axle Fy on the same grid as the sweep cache. Neither
    depends on beta, so the slow EKF step is not repeated; the grid length
    is checked against the cache."""
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_lateral_forces
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=1))
    data = parse_csv(SESSIONS[key])
    state = prepare_vehicle_state(data["channels"], params)
    forces = estimate_lateral_forces(state, params)
    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez_compressed(os.path.join(OUT_DIR, f"supp_{key}.npz"), time=state["time"], ay=state["ay_mps2"],
                        fy_f=forces["Fy_f_filt"], fy_r=forces["Fy_r_filt"], v=state["v_mps"])
    print(f"{key}: supplement saved, n={state['time'].size}")


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "supplement":
        supplement(sys.argv[2])
    else:
        {"mapping": mapping, "traces": traces, "smooth": smooth, "util": util,
         "ab": ab, "foldloop": foldloop}[mode]()
