# STEP 2 investigation (author work order 2026-10-04): where does our speed
# distribution sit relative to the chair's four-band structure, how good is
# the data per speed region, and which speed window fits our data -- the
# per-session [p1, p99] (Q-FIT-DECISIVE SW) or a band-informed window.
# Read-only, report only; the window choice is the author's. Tier B
# (selection on the fit population), chair-convergent in concept.
#
# CHAIR STRUCTURE (after the chair performance_analysis tooling (internal),
#   tyre-fit script): reference speeds linspace(18, 44, 4) m/s, weight
#   w = clip(1 - k (v - v_ref)^2, 0, 1), k = 1 / (spacing (1 + 0.05))^2,
#   samples with w > 0.05 enter a band -> band reach |v - v_ref| <
#   spacing * 1.05 * sqrt(0.95); union 9.13-52.87 m/s.
# REGIONS (nearest-centre zones of the chair bands, plus outside):
#   below the union / slow / mid-slow / mid-high / high / above the union.
# CANDIDATE WINDOWS (all on our data, per session):
#   SW  -- [p1, p99] of speed inside corner instances on analysis laps,
#          moving (identical to inspect_fit_decisive SW).
#   CU  -- the chair union transplanted unchanged (9.13-52.87 m/s); shown as
#          the reference only: vehicle parameterisation says window values
#          come from our car, not theirs.
#   BI  -- band-informed, parameterised to our car: the chair's construction
#          (four equally spaced centres, same overlap and weight cut) with the
#          outer centres placed at our in-corner speed p5 and p95, so the
#          band structure covers our corner-speed range the way the chair's
#          covers theirs. [Construction choice, labelled; the chair's own
#          centre placement rule is not in the reference code.]
# DATA QUALITY per region and per window: sample count / share of the fit
#   population (production base mask), kinematic-seed |alpha| p50 / p95 per
#   axle (the fit's input, corrected-chain cache), share of near-zero alpha
#   (|alpha_f| < 0.5 deg -- straight-line samples that carry no curve shape),
#   kerb density (kerb share of moving analysis-lap samples, base excludes
#   kerbs), in-corner share.
# Output: diagnostics/plots/speed_bands/

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp

KEYS = ("pr_us_v2", "v3", "nbr_jst_solution")
TITLES = {"pr_us_v2": "Paul Ricard, MLA outing 32", "v3": "Paul Ricard, MLA outing 25",
          "nbr_jst_solution": "Nurburgring, JST outing 29"}
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "speed_bands")

# chair constants (method-defining in the chair code)
CHAIR_REF = np.linspace(18.0, 44.0, 4)
OVERLAP = 0.05
W_CUT = 0.05
SW_PCT = (1, 99)          # as inspect_fit_decisive
BI_PCT = (5, 95)          # outer band centres, labelled construction choice
NEAR_ZERO_DEG = 0.5


def band_reach(spacing):
    return spacing * (1 + OVERLAP) * np.sqrt(1 - W_CUT)


def chair_union():
    r = band_reach(np.mean(np.diff(CHAIR_REF)))
    return CHAIR_REF[0] - r, CHAIR_REF[-1] + r


def bi_window(v_inc):
    lo, hi = np.percentile(v_inc, BI_PCT)
    refs = np.linspace(lo, hi, 4)
    r = band_reach(np.mean(np.diff(refs)))
    return refs, (max(refs[0] - r, 0.0), refs[-1] + r)


def region_edges():
    u_lo, u_hi = chair_union()
    mids = 0.5 * (CHAIR_REF[1:] + CHAIR_REF[:-1])
    return [0.0, u_lo, *mids, u_hi, np.inf]


REGION_NAMES = ["below chair window", "chair slow band", "chair mid-slow band", "chair mid-high band",
                "chair high band", "above chair window"]


def stats(sel, ctx, af, ar, kerb_pop, inc):
    n = int(sel.sum())
    if n == 0:
        return {"n": 0}
    return {"n": n,
            "alpha_f_p50": float(np.percentile(af[sel], 50)), "alpha_f_p95": float(np.percentile(af[sel], 95)),
            "alpha_r_p50": float(np.percentile(ar[sel], 50)), "alpha_r_p95": float(np.percentile(ar[sel], 95)),
            "near_zero": float(np.mean(af[sel] < NEAR_ZERO_DEG)),
            "in_corner": float(np.mean(inc[sel]))}


def session(key):
    ctx = lp.Ctx(key)
    st = ctx.state
    v = st["v_mps"]
    on_lap = ctx.lap_mask({l["lap_number"] for l in ctx.analysis_laps()})
    inc = ctx.in_br & on_lap
    base = ctx.base
    cache = d2.load_fit(key)
    af = np.abs(np.degrees(np.asarray(cache["alpha_f"], float)))
    ar = np.abs(np.degrees(np.asarray(cache["alpha_r"], float)))
    assert np.array_equal(cache["base_mask"], base)
    kerb = st["kerb_mask"]
    kerb_pop = st["moving_mask"] & on_lap
    v_inc = v[inc & st["moving_mask"]]

    sw = tuple(np.percentile(v_inc, SW_PCT))
    cu = chair_union()
    bi_refs, bi = bi_window(v_inc)
    out = {"key": key, "n_base": int(base.sum()),
           "v_base_pct": {p: float(np.percentile(v[base], p)) for p in (1, 5, 25, 50, 75, 95, 99)},
           "v_incorner_pct": {p: float(np.percentile(v_inc, p)) for p in (1, 5, 25, 50, 75, 95, 99)},
           "windows": {"SW": sw, "CU": cu, "BI": bi}, "bi_refs": bi_refs.tolist(),
           "regions": [], "window_stats": {}}
    edges = region_edges()
    for name, lo, hi in zip(REGION_NAMES, edges[:-1], edges[1:]):
        sel = base & (v >= lo) & (v < hi)
        kp = kerb_pop & (v >= lo) & (v < hi)
        r = stats(sel, ctx, af, ar, kerb_pop, inc)
        r.update({"name": name, "lo": lo, "hi": hi, "share": r["n"] / base.sum(),
                  "kerb_density": float(np.mean(kerb[kp])) if kp.any() else float("nan")})
        out["regions"].append(r)
    informative = base & (af >= 1.0)
    for wname, (lo, hi) in out["windows"].items():
        keep = base & (v >= lo) & (v <= hi)
        drop = base & ~keep
        out["window_stats"][wname] = {
            "lo_mps": lo, "hi_mps": hi, "retained": float(keep.sum() / base.sum()),
            "retained_informative": float((keep & informative).sum() / informative.sum()),
            "dropped_near_zero": float(np.mean(af[drop] < NEAR_ZERO_DEG)) if drop.any() else float("nan"),
            "dropped_below": float((base & (v < lo)).sum() / base.sum()),
            "dropped_above": float((base & (v > hi)).sum() / base.sum()),
            "dropped_incorner": float((drop & inc).sum() / max(1, (base & inc).sum())),
        }
    out["_plot"] = {"v_base": v[base], "v_inc": v[base & inc], "af": af[base], "ar": ar[base]}
    return out


def report(res):
    L = [f"Chair band centres {', '.join(f'{x:.1f}' for x in CHAIR_REF)} m/s; band union "
         f"{chair_union()[0]:.2f}-{chair_union()[1]:.2f} m/s ({chair_union()[0] * 3.6:.0f}-{chair_union()[1] * 3.6:.0f} km/h)"]
    for k in KEYS:
        r = res[k]
        L.append(f"== {TITLES[k]} ({k}), fit population {r['n_base']} samples")
        L.append("  speed percentiles fit population p1/p5/p25/p50/p75/p95/p99 [m/s]: "
                 + "/".join(f"{x:.1f}" for x in r["v_base_pct"].values()))
        L.append("  speed percentiles in corners  p1/p5/p25/p50/p75/p95/p99 [m/s]: "
                 + "/".join(f"{x:.1f}" for x in r["v_incorner_pct"].values()))
        L.append(f"  band-informed centres (our p5..p95 in corners): {', '.join(f'{x:.1f}' for x in r['bi_refs'])} m/s")
        for g in r["regions"]:
            if g["n"] == 0:
                L.append(f"  {g['name']} [{g['lo']:.1f}, {g['hi']:.1f}) m/s: no samples")
                continue
            L.append(f"  {g['name']} [{g['lo']:.1f}, {g['hi']:.1f}) m/s: {g['n']} samples ({g['share'] * 100:.1f} %), "
                     f"|alpha_f| p50/p95 {g['alpha_f_p50']:.2f}/{g['alpha_f_p95']:.2f} deg, |alpha_r| "
                     f"{g['alpha_r_p50']:.2f}/{g['alpha_r_p95']:.2f} deg, near-zero alpha {g['near_zero'] * 100:.0f} %, "
                     f"in corner {g['in_corner'] * 100:.0f} %, kerb density {g['kerb_density'] * 100:.1f} %")
        for w, s in r["window_stats"].items():
            L.append(f"  window {w} {s['lo_mps']:.1f}-{s['hi_mps']:.1f} m/s ({s['lo_mps'] * 3.6:.0f}-{s['hi_mps'] * 3.6:.0f} km/h): "
                     f"retains {s['retained'] * 100:.1f} % (informative |alpha_f| >= 1 deg: {s['retained_informative'] * 100:.1f} %); "
                     f"drops {s['dropped_below'] * 100:.1f} % below / {s['dropped_above'] * 100:.1f} % above, of which "
                     f"near-zero alpha {s['dropped_near_zero'] * 100:.0f} %; drops {s['dropped_incorner'] * 100:.1f} % of in-corner fit samples")
    txt = "\n".join(L)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(txt + "\n")
    print(txt)


# default validated palette slots; line style / hatching as second cue
C_BASE, C_INC = "#2a78d6", "#eb6834"
C_WIN = {"SW": "#1baf7a", "CU": "#888888", "BI": "#e87ba4"}
W_LABEL = {"SW": "our p1-p99 in corners", "CU": "chair band union (their car)",
           "BI": "band-informed, fitted to our corner speeds"}


def render(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(KEYS), 2, figsize=(14, 3.6 * len(KEYS)), squeeze=False)
    bins = np.arange(0, 80, 2.0)
    centres = 0.5 * (bins[1:] + bins[:-1])
    for i, k in enumerate(KEYS):
        r, p = res[k], res[k]["_plot"]
        ax = axs[i][0]
        ax.hist(p["v_base"], bins=bins, color=C_BASE, alpha=0.55, label="fit population")
        ax.hist(p["v_inc"], bins=bins, histtype="step", lw=2, color=C_INC, label="of which in corners")
        for j, (w, (lo, hi)) in enumerate(r["windows"].items()):
            y = 1.0 - 0.06 * (j + 1)
            ax.plot([lo, hi], [y, y], transform=ax.get_xaxis_transform(), color=C_WIN[w], lw=4,
                    solid_capstyle="butt", label=f"window: {W_LABEL[w]}")
        for c in CHAIR_REF:
            ax.axvline(c, color="#888888", ls=":", lw=1)
        ax.set_xlabel("speed [m/s]  (dotted: chair band centres 18 / 26.7 / 35.3 / 44)")
        ax.set_ylabel("samples")
        ax.set_title(f"{TITLES[k]}: speed distribution and candidate windows", fontsize=10)
        ax.grid(alpha=0.2)
        ax = axs[i][1]
        idx = np.digitize(p["v_base"], bins) - 1
        for arr, col, ls, name in ((p["af"], C_BASE, "-", "front"), (p["ar"], C_INC, "--", "rear")):
            p50 = [np.percentile(arr[idx == b], 50) if np.sum(idx == b) >= 200 else np.nan for b in range(len(centres))]
            p95 = [np.percentile(arr[idx == b], 95) if np.sum(idx == b) >= 200 else np.nan for b in range(len(centres))]
            ax.plot(centres, p50, color=col, ls=ls, lw=2, label=f"{name} median")
            ax.plot(centres, p95, color=col, ls=ls, lw=1, label=f"{name} 95th percentile")
        for c in CHAIR_REF:
            ax.axvline(c, color="#888888", ls=":", lw=1)
        ax.set_xlabel("speed [m/s]  (bins with >= 200 samples)")
        ax.set_ylabel("|seed slip angle| [deg]")
        ax.set_title(f"{TITLES[k]}: slip-angle magnitude per speed bin (fit input)", fontsize=10)
        ax.grid(alpha=0.2)
    # one legend per column, below the panels -- inside it covered the window bars
    for col, x in ((0, 0.27), (1, 0.75)):
        h, l = axs[0][col].get_legend_handles_labels()
        fig.legend(h, l, loc="lower center", bbox_to_anchor=(x, 0.0), ncol=2, fontsize=8, frameon=False)
    fig.suptitle("Speed windows for the tyre fit: our speed distribution against the chair's four speed bands",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(os.path.join(OUT_DIR, "speed_bands.png"), dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    res = {k: session(k) for k in KEYS}
    report(res)
    render(res)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump({k: {kk: vv for kk, vv in r.items() if kk != "_plot"} for k, r in res.items()}, fh, indent=1,
                  default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))
