# DIAGNOSTIC, read-only, [keep-reproduces]. PLAN.md EXPERIMENT PLAN (f):
# event coincidence. Do large single-lap deviations show up in independent
# channels on the same corner-lap, more often than chance? Nothing in
# modules/, core/ or config/ changes. Tier B throughout (relative
# percentile bars, permutation test -- standard techniques).
#
# Usage (repo root, venv active; reads existing caches only):
#   python -m diagnostics.inspect_event_coincidence report
# Caches: diagnostics/plots/new_laps/cache (production CS_ratio arrays and
# corner summaries, EKF Pacejka path, gate pass, no fallback) and
# diagnostics/plots/yaw_pooling/cache (yaw rate, steering); time and s_m
# arrays checked identical between the two per session. Outputs in
# diagnostics/plots/event_coincidence/ (gitignored).
#
# RULES (approved proposal, author rulings 2026-09-30; fixed before any run)
# - Unit: corner-lap (stable corner, analysis lap). A corner-lap where a
#   channel has no value is counted and excluded from that channel only.
# - YAW events: core/yaw_pooling via inspect_yaw_pooling.analyse, the
#   pre-registered (c) constants (4 Hz, 0.5 Hz trend, 1 m grid, 2 m gap,
#   p10-p90, >= 3 laps, margin 3 sigma, 5-point run). EXCESS and DEFICIT
#   are separate events; onset position recorded.
# - CS events, front and rear separately: per corner-lap the worst-phase
#   CS_ratio = min over phases of the phase median, apex phase from the
#   apex region -- the classifier's read-out (_classify_corner). Event if
#   below the session's own p10 of that statistic (relative bar; the
#   classification thresholds are NOT used). Sample minimum over the
#   canonical bracket and its position recorded as attributes.
# - STABILITY events: per corner-lap the worst-phase median of
#   stability_observed (the classifier's argmin); event if below the
#   session p10.
# - STEERING (attribute only, never a filter): the existing counteraction
#   test of inspect_yaw_pooling -- unwinding rate above T = 3 x robust std
#   of steering rate on straights within 150-300 ms of the flag onset, with
#   the control share at the same position on the other laps.
# - Chance baseline: each coincidence count with its expectation under
#   independence and a one-sided permutation p-value. Null: lap labels
#   shuffled WITHIN each corner (keeps each corner's own event propensity,
#   breaks only lap identity), N_SHUFFLE = 1000, fixed SEED.
# - Channels for the pass criterion: YAW-EXC, CS-F, CS-R, STAB (six pairs).
#   YAW-DEF pairs are reported descriptively, not part of the criterion.
# - Coincidence matrix: Y = yaw excess, C = CS event on either axle,
#   S = stability event, on corner-laps where all three are assessable.
# - ENTRY: yaw margins reproduce (c) Part 1 (1.6715 / 1.7397 / 1.9081 /
#   1.5221 deg/s); caches aligned; the CS/stability read-out reproduces the
#   classifier's own verdict text (value to 2 dp, phase, axle) on JST 29.
# - PASS: a pair with p < 0.05 on at least 2 of the 3 screening sessions.
#   Exit A (a pair passes): coincident corner-laps = candidate real-event
#   list, feeds the parked (c)-evidence question. Exit B (none): channels
#   independent at corner-lap level; CS-alone = artifact-suspect list.
#   Either way the carried questions are answered per corner-lap.
# Sessions: screening JST 29, US-v2, MLA; JST 21 for C1 (2 analysis laps
# -> no yaw envelope: CS + stability only, author ruling); Dubai C3/C4 as a
# two-corner check only.

import gzip
import os
import pickle
import sys

import numpy as np

from diagnostics import inspect_yaw_pooling as yp

SCREEN = ["nbr_jst_solution", "pr_us_v2", "nbr_qp_mla"]
LABEL = {"nbr_jst_solution": "JST 29", "pr_us_v2": "US-v2", "nbr_qp_mla": "MLA", "nbr_jst_problem": "JST 21",
         "dubai": "Dubai"}
YAW_KEYS = {"nbr_jst_solution", "pr_us_v2", "nbr_qp_mla", "dubai"}
OUT_DIR = "diagnostics/plots/event_coincidence"
EVENT_PCT = 10
N_SHUFFLE = 1000
SEED = 20260930
P_PASS = 0.05
PHASES = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
EXPECTED_MARGIN = {"nbr_jst_solution": 1.6715, "pr_us_v2": 1.7397, "nbr_qp_mla": 1.9081, "dubai": 1.5221}
PAIRS = [("YAW-EXC", "CS-F"), ("YAW-EXC", "CS-R"), ("YAW-EXC", "STAB"), ("CS-F", "CS-R"), ("CS-F", "STAB"),
         ("CS-R", "STAB")]
DESC_PAIRS = [("YAW-DEF", "CS-F"), ("YAW-DEF", "CS-R"), ("YAW-DEF", "STAB")]
# carried questions (thesis_notes "Sawtooth repair side-by-side render",
# "(c) Part 1", "Rulings on (b) and (c)")
SPIKES = {"nbr_jst_solution": [(1, 107)], "pr_us_v2": [(4, 51)]}
NBR_C1 = {"nbr_jst_solution": [(1, 103), (1, 105), (1, 107)], "nbr_jst_problem": [(1, 72), (1, 73)]}
DUBAI_CORNERS = [3, 4]


def _load_nl(key):
    with gzip.open(os.path.join("diagnostics/plots/new_laps/cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def worst_phase(summary):
    """Classifier read-out (ui/views/outing_form._classify_corner): per phase
    median, apex phase from the apex region for CS; min over finite phases.
    None where no phase is finite (the classifier would fall back to 1.0 /
    1e9 -- here the corner-lap is simply not assessable)."""
    apex = summary.get("apex_region")
    out = {"f": (None, None), "r": (None, None), "stab": (None, None)}
    for phase, p in summary["phases"].items():
        if phase == "apex_3" and apex is not None:
            vals = {"f": apex["cs_ratio_f"]["median"], "r": apex["cs_ratio_r"]["median"]}
        else:
            vals = {"f": p["cs_ratio_f"]["median"], "r": p["cs_ratio_r"]["median"]}
        vals["stab"] = p["stability_observed_Nm_per_deg"]["median"]
        for k, v in vals.items():
            if v == v and (out[k][0] is None or v < out[k][0]):
                out[k] = (float(v), phase)
    return out


def session_channels(key):
    """Per corner-lap channel values and events for one session."""
    nl = _load_nl(key)
    valid = {l["lap_number"] for l in nl["laps"] if l.get("is_valid_for_analysis")}
    cs_f, cs_r = nl["cs"]["A_baseline"]
    s_m = nl["s_m"]
    trav = {(tr["cid"], tr["lap"]): tr for tr in nl["traversals"]}
    rows = {}
    for sm in nl["summaries"]["A_baseline"]:
        cl = (sm["stable_corner_id"], sm["lap_number"])
        if sm["lap_number"] not in valid or sm["stable_corner_id"] is None:
            continue
        w = worst_phase(sm)
        row = {"cs_f": w["f"][0], "cs_f_phase": w["f"][1], "cs_r": w["r"][0], "cs_r_phase": w["r"][1],
               "stab": w["stab"][0], "stab_phase": w["stab"][1], "summary": sm,
               "bracket": (sm.get("bracket_start_m"), sm.get("bracket_end_m"))}
        tr = trav.get(cl)
        for ax, arr in (("f", cs_f), ("r", cs_r)):
            row[f"min_{ax}"], row[f"min_{ax}_s"] = None, None
            if tr is not None:
                seg = arr[tr["lo"]:tr["hi"]]
                if np.isfinite(seg).any():
                    i = int(np.nanargmin(seg))
                    row[f"min_{ax}"], row[f"min_{ax}_s"] = float(seg[i]), float(s_m[tr["lo"] + i])
        rows[cl] = row
    bars = {}
    for ch, col in (("CS-F", "cs_f"), ("CS-R", "cs_r"), ("STAB", "stab")):
        vals = np.array([r[col] for r in rows.values() if r[col] is not None])
        bars[ch] = float(np.percentile(vals, EVENT_PCT))
        for r in rows.values():
            r[ch] = None if r[col] is None else bool(r[col] < bars[ch])
    yaw = None
    if key in YAW_KEYS:
        res, _d, _r4, _srate = yp.analyse(key)
        yaw = res
        pooled = {(sid, lap) for sid, c in res["corners"].items() for lap in c["laps"]}
        for cl, r in rows.items():
            fl = [f for f in res["corners"].get(cl[0], {}).get("flags", []) if f["lap"] == cl[1]]
            r["yaw_flags"] = fl
            for kind, ch in (("EXCESS", "YAW-EXC"), ("DEFICIT", "YAW-DEF")):
                r[ch] = (any(f["kind"] == kind for f in fl)) if cl in pooled else None
    else:
        for r in rows.values():
            r["yaw_flags"], r["YAW-EXC"], r["YAW-DEF"] = [], None, None
    return {"key": key, "rows": rows, "bars": bars, "yaw": yaw, "nl": nl}


def entry_check_classifier(sess):
    """The read-out must reproduce the classifier's own verdict text: where
    it names a CS primary, value (2 dp), phase and axle must match; where it
    reports unstable yaw, the stability minimum must be below the threshold."""
    from diagnostics.inspect_frame_stage2_parity import classify_fn
    from modules.decision_frame import PHASE_WORD_LONG
    from modules.stability_analysis import load_parameters
    cls = load_parameters()["classification"]
    th = {k: cls[k]["value"] for k in ("STRONG_CSF", "MODERATE_CSF", "STRONG_CSR", "MODERATE_CSR",
                                       "stab_neg_thresh_Nm_per_deg")}
    n_primary = n_ok = n_stab = 0
    for cl, r in sess["rows"].items():
        _sev, _short, long_text, _c = classify_fn(r["summary"])
        f, rr = r["cs_f"], r["cs_r"]
        f_act = f is not None and f < th["MODERATE_CSF"]
        r_act = rr is not None and rr < th["MODERATE_CSR"]
        if r_act and rr < th["STRONG_CSR"]:
            want = ("CSr", rr, r["cs_r_phase"])
        elif f_act and f < th["STRONG_CSF"]:
            want = ("CSf", f, r["cs_f_phase"])
        elif r_act:
            want = ("CSr", rr, r["cs_r_phase"])
        elif f_act:
            want = ("CSf", f, r["cs_f_phase"])
        else:
            want = None
        if want is not None:
            n_primary += 1
            token = f"{PHASE_WORD_LONG[want[2]]} ({want[0]} {want[1]:.2f})"
            n_ok += token in long_text
        else:
            assert "CSf" not in long_text and "CSr" not in long_text, (cl, long_text)
        if r["stab"] is not None and r["stab"] < th["stab_neg_thresh_Nm_per_deg"]:
            n_stab += 1
    return n_primary, n_ok, n_stab, len(sess["rows"])


def _perm_test(rows, a, b, rng, c=None):
    """Coincidence count of channels a & b (& c) on corner-laps where all are
    assessable; expectation under independence; one-sided p from shuffling
    lap labels within each corner (b, and c, shuffled independently)."""
    cls = [cl for cl, r in rows.items() if r[a] is not None and r[b] is not None
           and (c is None or r[c] is not None)]
    if not cls:
        return None
    A = np.array([rows[cl][a] for cl in cls])
    B = np.array([rows[cl][b] for cl in cls])
    C = np.array([rows[cl][c] for cl in cls]) if c else np.ones(len(cls), bool)
    obs = int(np.sum(A & B & C))
    expected = len(cls) * A.mean() * B.mean() * (C.mean() if c else 1.0)
    groups = {}
    for i, cl in enumerate(cls):
        groups.setdefault(cl[0], []).append(i)
    idx_groups = [np.array(g) for g in groups.values()]
    ge = 0
    for _ in range(N_SHUFFLE):
        Bp, Cp = B.copy(), C.copy()
        for g in idx_groups:
            Bp[g] = B[rng.permutation(g)]
            if c:
                Cp[g] = C[rng.permutation(g)]
        ge += int(np.sum(A & Bp & Cp)) >= obs
    return {"n": len(cls), "obs": obs, "exp": float(expected), "p": (1 + ge) / (1 + N_SHUFFLE),
            "nA": int(A.sum()), "nB": int(B.sum())}


def _fmt_cl(cl, r):
    parts = [f"C{cl[0]} lap {cl[1]}"]
    for ch in ("YAW-EXC", "YAW-DEF", "CS-F", "CS-R", "STAB"):
        if r.get(ch):
            parts.append(ch)
    return " ".join(parts)


def _attrs(r):
    out = []
    if r["cs_f"] is not None:
        out.append(f"CSf worst {r['cs_f']:.2f}@{r['cs_f_phase']}")
    if r["cs_r"] is not None:
        out.append(f"CSr worst {r['cs_r']:.2f}@{r['cs_r_phase']}")
    if r["min_f"] is not None:
        out.append(f"min f {r['min_f']:.2f}@{r['min_f_s']:.0f} m")
    if r["min_r"] is not None:
        out.append(f"min r {r['min_r']:.2f}@{r['min_r_s']:.0f} m")
    if r["stab"] is not None:
        out.append(f"stab worst {r['stab']:.0f}@{r['stab_phase']}")
    for f in r["yaw_flags"]:
        cor = {True: "reaction", False: "no reaction", None: "reaction n/a"}[f["corroborated"]]
        out.append(f"{f['kind']} s {f['s_on']:.0f} m {f['len_m']} m +{f['peak']:.1f} deg/s ({cor})")
    return "; ".join(out)


def figure(sess, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res, rows, key = sess["yaw"], sess["rows"], sess["key"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 7), sharex=True, gridspec_kw={"height_ratios": [2.4, 1]})
    ink, thin, band = "#1f2937", "#9ca3af", "#e5e7eb"
    for sid, c in res["corners"].items():
        g = c["grid"]
        for y in c["Y"]:
            ax1.plot(g, y, color=thin, linewidth=0.5)
        ax1.fill_between(g, c["p10"], c["p90"], color=band, linewidth=0)
        ax1.plot(g, c["med"], color=ink, linewidth=1.6)
        for f in c["flags"]:
            j = c["laps"].index(f["lap"])
            a = int(round((f["s_on"] - g[0]) / yp.GRID_M))
            seg = slice(a, a + f["len_m"])
            ax1.plot(g[seg], c["Y"][j][seg], color="#b91c1c" if f["kind"] == "EXCESS" else "#2563eb", linewidth=1.8)
        ax1.text(g[len(g) // 2], 72, f"C{sid}", ha="center", fontsize=7, color=ink)
    ax1.set_ylim(-80, 80)
    ax1.set_ylabel("yaw rate, 4 Hz (deg/s)", fontsize=8)
    # event rows: CS at the corner-lap's sample-minimum position, stability
    # at the bracket centre (summaries carry no phase positions)
    rowsy = {"YAW": 2.5, "CS-F": 1.5, "CS-R": 1.0, "STAB": 0.0}
    for cl, r in rows.items():
        mid = np.mean(r["bracket"]) if None not in r["bracket"] else None
        for f in r["yaw_flags"]:
            ax2.plot(f["s_on"], rowsy["YAW"], "^" if f["kind"] == "EXCESS" else "v",
                     color="#b91c1c" if f["kind"] == "EXCESS" else "#2563eb", ms=5)
        for ch, s_key in (("CS-F", "min_f_s"), ("CS-R", "min_r_s")):
            if r[ch] and r[s_key] is not None:
                ax2.plot(r[s_key], rowsy[ch], "o", color="#7c3aed", ms=5)
                ax2.annotate(str(cl[1]), (r[s_key], rowsy[ch]), xytext=(0, 5), textcoords="offset points",
                             fontsize=6, ha="center")
        if r["STAB"] and mid is not None:
            ax2.plot(mid, rowsy["STAB"], "s", color="#b45309", ms=5)
            ax2.annotate(str(cl[1]), (mid, rowsy["STAB"]), xytext=(0, 5), textcoords="offset points",
                         fontsize=6, ha="center")
    ax2.set_yticks(list(rowsy.values()), [f"{k} event" for k in rowsy], fontsize=7)
    ax2.set_ylim(-0.7, 3.2)
    ax2.set_xlabel("lap distance (m)", fontsize=8)
    ax1.set_title(f"{LABEL[key]}: yaw per analysis lap (thin), pooled median, p10-p90 envelope, flags beyond "
                  f"+{res['margin']:.2f} deg/s (excess red, deficit blue); lower panel: all three channels' events "
                  f"(CS at the sample minimum, stability at the bracket centre; numbers = lap)", fontsize=8)
    for ax in (ax1, ax2):
        ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def report():
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = np.random.default_rng(SEED)
    sessions = {k: session_channels(k) for k in SCREEN + ["nbr_jst_problem", "dubai"]}
    summary = {}

    print("=== ENTRY")
    for k in YAW_KEYS:
        m = sessions[k]["yaw"]["margin"]
        print(f"  {LABEL[k]} yaw margin {m:.4f} deg/s (recorded {EXPECTED_MARGIN[k]:.4f}) "
              f"{'OK' if abs(m - EXPECTED_MARGIN[k]) < 5e-5 else 'MISMATCH'}")
    n_primary, n_ok, n_stab, n = entry_check_classifier(sessions["nbr_jst_solution"])
    print(f"  JST 29 classifier reproduction: CS primary named on {n_primary} corner-laps, value/phase/axle "
          f"matched {n_ok}/{n_primary}; no-primary corner-laps carry no CS token; stab < threshold on {n_stab}; "
          f"{n} corner-laps")

    print("\n=== PART 1 - events per channel")
    for k, s in sessions.items():
        rows = s["rows"]
        line = [f"{LABEL[k]}: {len(rows)} corner-laps"]
        for ch in ("YAW-EXC", "YAW-DEF", "CS-F", "CS-R", "STAB"):
            ass = [r[ch] for r in rows.values() if r[ch] is not None]
            line.append(f"{ch} {sum(ass)}/{len(ass)}" if ass else f"{ch} n/a")
        print("  " + "; ".join(line) + f" | p10 bars CS-F {s['bars']['CS-F']:.3f}, CS-R {s['bars']['CS-R']:.3f}, "
              f"STAB {s['bars']['STAB']:.0f} Nm/deg" + (f" | yaw margin {s['yaw']['margin']:.2f} deg/s" if s["yaw"] else ""))
        summary[k] = {"n": len(rows), "bars": s["bars"]}

    print("\n=== PART 2 - coincidence (screening sessions)")
    passed = {}
    for k in SCREEN:
        rows = sessions[k]["rows"]
        full = {cl: r for cl, r in rows.items() if all(r[ch] is not None for ch in ("YAW-EXC", "CS-F", "CS-R", "STAB"))}
        Y = {cl for cl, r in full.items() if r["YAW-EXC"]}
        C = {cl for cl, r in full.items() if r["CS-F"] or r["CS-R"]}
        S = {cl for cl, r in full.items() if r["STAB"]}
        print(f"  {LABEL[k]} ({len(full)} corner-laps with all channels): Y only {len(Y - C - S)}, C only "
              f"{len(C - Y - S)}, S only {len(S - Y - C)}, Y&C only {len((Y & C) - S)}, Y&S only {len((Y & S) - C)}, "
              f"C&S only {len((C & S) - Y)}, all three {len(Y & C & S)}, none {len(set(full) - Y - C - S)}")
        for cl in sorted(Y & C & S):
            print(f"    ALL THREE: {_fmt_cl(cl, rows[cl])} -- {_attrs(rows[cl])}")
        for cl in sorted(C - Y):
            print(f"    CS without yaw excess: {_fmt_cl(cl, rows[cl])} -- {_attrs(rows[cl])}")
        for cl in sorted(Y - C):
            print(f"    yaw excess without CS: {_fmt_cl(cl, rows[cl])} -- {_attrs(rows[cl])}")
        for a, b in PAIRS + DESC_PAIRS:
            t = _perm_test(rows, a, b, rng)
            tag = "" if (a, b) in PAIRS else " [descriptive]"
            print(f"    pair {a} x {b}{tag}: obs {t['obs']} vs exp {t['exp']:.2f} (n {t['n']}, {t['nA']}/{t['nB']} "
                  f"events), enrichment {t['obs'] / t['exp'] if t['exp'] else float('nan'):.2f}, p {t['p']:.3f}")
            if (a, b) in PAIRS:
                passed.setdefault((a, b), []).append(t["p"] < P_PASS)
        t3 = _perm_test(rows, "YAW-EXC", "CS-F", rng, c="STAB")
        t3r = _perm_test(rows, "YAW-EXC", "CS-R", rng, c="STAB")
        print(f"    triple YAW-EXC x CS-F x STAB: obs {t3['obs']} vs exp {t3['exp']:.2f}, p {t3['p']:.3f}; "
              f"YAW-EXC x CS-R x STAB: obs {t3r['obs']} vs exp {t3r['exp']:.2f}, p {t3r['p']:.3f}")
    print("  PASS CRITERION (p < 0.05 on >= 2 of 3 screening sessions):")
    for pair, flags in passed.items():
        print(f"    {pair[0]} x {pair[1]}: {sum(flags)}/3 -> {'PASS' if sum(flags) >= 2 else 'no'}")
    summary["passed"] = {f"{a} x {b}": sum(f) for (a, b), f in passed.items()}

    print("\n=== PART 3 - steering reaction per yaw event (attribute)")
    for k in SCREEN:
        fl = [f for r in sessions[k]["rows"].values() for f in r["yaw_flags"]]
        for kind in ("EXCESS", "DEFICIT"):
            sub = [f for f in fl if f["kind"] == kind]
            cor = [f["corroborated"] for f in sub if f["corroborated"] is not None]
            ctrl = [f["control_share"] for f in sub if f["control_share"] is not None]
            print(f"  {LABEL[k]} {kind}: {len(sub)} flags, reaction {sum(cor)}/{len(cor)} "
                  f"({np.mean(cor) if cor else float('nan'):.2f}); same-position control share "
                  f"{np.mean(ctrl) if ctrl else float('nan'):.2f}")
        # reaction split by whether the yaw-excess corner-lap also carries a CS or stability event
        exc = [(r, f) for r in sessions[k]["rows"].values() for f in r["yaw_flags"] if f["kind"] == "EXCESS"
               and f["corroborated"] is not None]
        co = [f["corroborated"] for r, f in exc if r["CS-F"] or r["CS-R"] or r["STAB"]]
        alone = [f["corroborated"] for r, f in exc if not (r["CS-F"] or r["CS-R"] or r["STAB"])]
        print(f"    EXCESS with a CS/stab event on the corner-lap: reaction {sum(co)}/{len(co)}; "
              f"yaw-alone: {sum(alone)}/{len(alone)}")

    print("\n=== PART 4 - carried questions")
    for k, cls in SPIKES.items():
        for cl in cls:
            r = sessions[k]["rows"].get(cl)
            print(f"  Q1 spike {LABEL[k]} {_fmt_cl(cl, r)} -- {_attrs(r)}")
    for k, cls in NBR_C1.items():
        for cl in cls:
            r = sessions[k]["rows"].get(cl)
            print(f"  Q2 {LABEL[k]} {_fmt_cl(cl, r) if r else f'C{cl[0]} lap {cl[1]} missing'} -- "
                  f"{_attrs(r) if r else ''}")
        c1 = [(cl, r) for cl, r in sessions[k]["rows"].items() if cl[0] == 1]
        print(f"    {LABEL[k]} C1, all analysis laps: " + "; ".join(_fmt_cl(cl, r) for cl, r in sorted(c1)))
    for cid in DUBAI_CORNERS:
        for cl, r in sorted(sessions["dubai"]["rows"].items()):
            if cl[0] == cid:
                print(f"  Q3 Dubai {_fmt_cl(cl, r)} -- {_attrs(r)}")

    print("\n=== PART 5 - read-out per session")
    for k in SCREEN:
        rows = sessions[k]["rows"]
        agree, cs_alone, yaw_alone = {}, {}, {}
        for cl, r in rows.items():
            n_ch = sum(bool(x) for x in (r["YAW-EXC"] or r["YAW-DEF"], r["CS-F"] or r["CS-R"], r["STAB"]))
            cs = r["CS-F"] or r["CS-R"]
            yaw = r["YAW-EXC"] or r["YAW-DEF"]
            if n_ch >= 2:
                agree.setdefault(cl[0], []).append(cl[1])
            elif cs and not yaw and not r["STAB"]:
                cs_alone.setdefault(cl[0], []).append(cl[1])
            elif yaw and not cs and not r["STAB"]:
                yaw_alone.setdefault(cl[0], []).append(cl[1])
        f = lambda d: ", ".join(f"C{c} ({'/'.join(map(str, sorted(l)))})" for c, l in sorted(d.items())) or "none"
        print(f"  {LABEL[k]}: channels agree (>= 2 of yaw-any/CS/stab): {f(agree)}")
        print(f"    CS fires alone: {f(cs_alone)}")
        print(f"    yaw fires alone: {f(yaw_alone)}")
        figure(sessions[k], os.path.join(OUT_DIR, f"event_coincidence_{k}.png"))
    with open(os.path.join(OUT_DIR, "summary.pkl"), "wb") as fh:
        pickle.dump(summary, fh)


if __name__ == "__main__":
    {"report": report}[sys.argv[1]]()
