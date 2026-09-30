# DIAGNOSTIC, read-only. Author ruling 2026-09-30 (C1 follow-up, OPEN 2):
# how general is the NBR C1 mechanism? Share of per-corner-lap CS_ratio
# minima that sit near a kerb-mask edge, against the chance rate, across
# the eight census sessions. Sizes the proposal "kerb samples excluded
# inside CS windows" (thesis_notes "NBR C1 fixed-position CS dip: kerb
# strike at the apex"). Nothing in modules/, core/ or config/ changes.
#
# Usage (repo root, venv active; caches only):
#   python -m diagnostics.inspect_cs_kerb_edge
# Caches: diagnostics/plots/new_laps/cache (production CS_ratio f/r,
# canonical corner traversals; EKF Pacejka path) and
# diagnostics/plots/imu_offsets/cache (state with the production kerb mask,
# |az - 1 g| > 1.2 g dilated +/-5 samples); time bases asserted identical.
# Figure in diagnostics/plots/cs_kerb_edge/ (gitignored).
#
# RULES (fixed before the run)
# - Sessions: the eight census sessions (Dubai, v3, US-v2, JST 21, JST 29,
#   LMN MLA, CDR, QP MLA); analysis laps only.
# - Unit: corner-lap = one canonical bracket traversal. Its CS minimum, per
#   axle, is the sample minimum of CS_ratio over the bracket (the level at
#   which the C1 dips live). Deep tail: minima below the session's p10 of
#   those minima, per axle.
# - Kerb-mask edge: every sample where the mask switches state, on the same
#   lap (edges just outside the bracket count). Near = |s_min - s_edge| <=
#   NEAR_M = 3 m (the C1 evidence: dips 0-2 m from the mask edges).
# - Chance rate: per corner-lap, the share of its CS-eligible samples
#   (finite CS_ratio -- the only places a minimum can fall; kerb interiors
#   are NaN) lying within NEAR_M of an edge; a group's expected count is
#   the sum of those shares. p = Monte Carlo over independent Bernoulli
#   draws with the per-corner-lap shares (N_MC = 20000, fixed seed),
#   one-sided (observed >= drawn).
# - Reported: per session and pooled, front and rear, all minima and deep
#   tail: near count / n, observed share, chance share, enrichment, p.
# - Fix coverage: for each near-edge minimum, the CS window at that sample
#   (reconstruct_cs_window_start, the live loop's start) and whether it
#   holds kerb-flagged samples -- the only case that excluding kerb samples
#   inside windows can change (C1 family A; family B sits before the mask).

import gzip
import os
import pickle

import numpy as np

KEYS = ["dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "lmn_mla", "nbr_qp_cdr", "nbr_qp_mla"]
LABEL = {"dubai": "Dubai", "v3": "v3", "pr_us_v2": "US-v2", "nbr_jst_problem": "JST 21", "nbr_jst_solution": "JST 29",
         "lmn_mla": "LMN MLA", "nbr_qp_cdr": "CDR", "nbr_qp_mla": "QP MLA"}
NEAR_M = 3.0
TAIL_PCT = 10
N_MC = 20000
SEED = 20260930
OUT_DIR = "diagnostics/plots/cs_kerb_edge"


def _load(d, key):
    with gzip.open(os.path.join("diagnostics/plots", d, "cache", f"{key}.pkl.gz"), "rb") as fh:
        return pickle.load(fh)


def corner_laps(key, cs=None):
    """cs: optional (front, rear) arrays replacing the cached production CS
    (the (g) step-0 variants); window-kerb counts always use the production
    window rule."""
    nl = _load("new_laps", key)
    st = _load("imu_offsets", key)
    st = st.get("state", st)
    assert st["time"].tobytes() == nl["time"].tobytes(), key
    t, s, kerb = nl["time"], nl["s_m"], np.asarray(st["kerb_mask"], bool)
    laps = {l["lap_number"]: l for l in nl["laps"] if l.get("is_valid_for_analysis")}
    edges_by_lap = {}
    for ln, l in laps.items():
        idx = np.flatnonzero((t >= l["start_time"]) & (t <= l["end_time"]))
        e = idx[1:][np.diff(kerb[idx].astype(np.int8)) != 0]
        es = s[e]
        edges_by_lap[ln] = es[np.isfinite(es)]
    from modules.stability_analysis import load_parameters, reconstruct_cs_window_start
    se = load_parameters()["stability_estimation"]
    out = {"f": [], "r": []}
    for ax, arr in zip(("f", "r"), cs if cs is not None else nl["cs"]["A_baseline"]):
        alpha = nl["alpha_f"] if ax == "f" else nl["alpha_r"]
        for tr in nl["traversals"]:
            if tr["lap"] not in laps:
                continue
            seg = np.arange(tr["lo"], tr["hi"])
            ok = seg[np.isfinite(arr[seg]) & np.isfinite(s[seg])]
            if ok.size == 0:
                continue
            i = ok[np.argmin(arr[ok])]
            es = edges_by_lap[tr["lap"]]
            if es.size:
                d_min = float(np.min(np.abs(es - s[i])))
                near_share = float(np.mean(np.min(np.abs(s[ok][:, None] - es[None, :]), axis=1) <= NEAR_M))
            else:
                d_min, near_share = float("inf"), 0.0
            start = reconstruct_cs_window_start(alpha, int(i), nl["min_window"], se["cs_min_slip_angle_span_rad"],
                                                s_m=s, max_window_m=se["cs_max_window_m"])
            out[ax].append({"cid": tr["cid"], "lap": tr["lap"], "min": float(arr[i]), "s": float(s[i]),
                            "dist": d_min, "near": d_min <= NEAR_M, "p_chance": near_share,
                            "win_kerb": int(kerb[start:int(i) + 1].sum())})
    return out


def _stats(rows, rng):
    if not rows:
        return None
    near = np.array([r["near"] for r in rows])
    p = np.array([r["p_chance"] for r in rows])
    obs, exp = int(near.sum()), float(p.sum())
    draws = (rng.random((N_MC, p.size)) < p).sum(axis=1)
    kw = sum(1 for r in rows if r["near"] and r["win_kerb"] > 0)
    kw_all = sum(1 for r in rows if r["win_kerb"] > 0)
    return {"n": len(rows), "obs": obs, "share": obs / len(rows), "chance": exp / len(rows),
            "enrich": obs / exp if exp > 0 else float("nan"), "p": (1 + int(np.sum(draws >= obs))) / (1 + N_MC),
            "near_kerb_window": kw, "kerb_window_any": kw_all}


def _line(tag, st):
    if st is None:
        return f"{tag}: none"
    return (f"{tag}: near {st['obs']}/{st['n']} ({st['share']:.2f}) vs chance {st['chance']:.2f}, "
            f"enrichment {st['enrich']:.2f}, p {st['p']:.4f}; window holds kerb samples: {st['near_kerb_window']} of "
            f"the near-edge, {st['kerb_window_any']} of all {st['n']}")


def main():
    rng = np.random.default_rng(SEED)
    os.makedirs(OUT_DIR, exist_ok=True)
    pooled = {(ax, g): [] for ax in "fr" for g in ("all", "tail")}
    table = {}
    for key in KEYS:
        cl = corner_laps(key)
        print(f"\n== {LABEL[key]}")
        for ax in "fr":
            rows = cl[ax]
            bar = float(np.percentile([r["min"] for r in rows], TAIL_PCT))
            tail = [r for r in rows if r["min"] < bar]
            a, tl = _stats(rows, rng), _stats(tail, rng)
            table[(key, ax)] = (a, tl)
            pooled[(ax, "all")] += rows
            pooled[(ax, "tail")] += tail
            axn = "front" if ax == "f" else "rear"
            print("  " + _line(f"{axn} all minima", a))
            print("  " + _line(f"{axn} deep tail (< p10 {bar:.2f})", tl))
            near_tail = sorted((r for r in tail if r["near"]), key=lambda r: r["min"])
            if near_tail:
                print("    deep-tail minima near an edge: " + "; ".join(
                    f"C{r['cid']} lap {r['lap']} {r['min']:.2f} @ {r['s']:.0f} m ({r['dist']:.1f} m)" for r in near_tail))
    print("\n== POOLED, eight sessions")
    for ax in "fr":
        axn = "front" if ax == "f" else "rear"
        for g in ("all", "tail"):
            st = _stats(pooled[(ax, g)], rng)
            table[("pooled", ax, g)] = st
            print("  " + _line(f"{axn} {'all minima' if g == 'all' else 'deep tail'}", st))
    figure(table, os.path.join(OUT_DIR, "cs_kerb_edge.png"))


def figure(table, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = [LABEL[k] for k in KEYS] + ["pooled"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharey=True)
    for col, ax_key in enumerate("fr"):
        for row, g in enumerate(("all", "tail")):
            a = axes[row][col]
            obs, ch = [], []
            for k in KEYS:
                st = table[(k, ax_key)][0 if g == "all" else 1]
                obs.append(st["share"] if st else np.nan)
                ch.append(st["chance"] if st else np.nan)
            st = table[("pooled", ax_key, g)]
            obs.append(st["share"]); ch.append(st["chance"])
            x = np.arange(len(labels))
            a.bar(x - 0.2, obs, 0.4, color="#1f2937", label="observed share near a kerb-mask edge")
            a.bar(x + 0.2, ch, 0.4, color="#9ca3af", label="chance share")
            a.set_xticks(x, labels, rotation=30, fontsize=7)
            a.set_title(f"{'front' if ax_key == 'f' else 'rear'} CS_ratio, "
                        f"{'all corner-lap minima' if g == 'all' else 'deep tail (< session p10)'}", fontsize=9)
            a.tick_params(labelsize=7)
    axes[0][0].legend(fontsize=7)
    fig.suptitle(f"Share of CS_ratio minima within {NEAR_M:g} m of a kerb-mask edge vs chance, eight sessions", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
