# Understeer-gradient line (formerly Q-K2; PLAN.md STATUS 2026-10-05, author
# conversions + rulings, EXECUTE steps A and B; read-only).
#
# RULES, frozen 2026-10-05 BEFORE any run.
# STEP A: one measured understeer gradient per session, recomputed on the
#   CORRECTED chain (steering table lookup, schema 10) -- the recorded Q-PUSH
#   values (+1.27-1.41 deg/g, 2026-10-02) predate the steering fix. Method
#   unchanged: inspect_last_push.measure_k, Rajamani sec. 3.2 Eq. 3.23, beta-
#   free OLS of delta_f - offset - L r / v on a_y over the quasi-steady
#   population (2 Hz zero-phase), steering zero offset = median road-wheel
#   angle on straights, 95 % CI by corner-traversal cluster bootstrap (B =
#   200, fixed seed). Sessions: the 16 NBR files + the three Tier-1 anchors
#   (pr_us_v2, v3, nbr_jst_solution -- the last is also an NBR file). State
#   from lp.Ctx: cached parsed data + best-available parameters, i.e. the
#   production steering conversion; nothing here reads the damper loads.
#   Reported beside each anchor: its recorded pre-fix value.
# STEP B: one static mock (no app code): the JST 29 verdict card as the app
#   classifier words it (production summaries from the cached beta, entry-
#   checked as in inspect_mu_trace E3), with ONE added session line carrying
#   the measured gradient, its CI and a plain-word reading. Colours from
#   ui/style constants. Rendered for the author's ship / kill ruling.
#
# Modes: a | report | mock     Output: diagnostics/plots/k2_line/

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_last_push as lp
from diagnostics.inspect_pool_fits import NBR16

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "k2_line")
ANCHORS = ("pr_us_v2", "v3", "nbr_jst_solution")
PRE_FIX = {"pr_us_v2": 1.41, "v3": 1.36, "nbr_jst_solution": 1.27}  # Q-PUSH, thesis_notes 2026-10-02
MOCK_KEY = "nbr_jst_solution"


def _log(msg):
    os.makedirs(OUT_DIR, exist_ok=True)
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, "run_log.txt"), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def step_a(keys):
    os.makedirs(OUT_DIR, exist_ok=True)
    for k in keys or (("pr_us_v2", "v3") + NBR16):
        t0 = time.perf_counter()
        try:
            ctx = lp.Ctx(k)
            cen = lp.steering_census(ctx)
            k2, _ = lp.measure_k(ctx, ctx.state, cen["zero_offset_road_deg"] / lp.RAD2DEG)
            rec = {"key": k, "title": ctx.name, "K2": k2, "census": cen, "pre_fix_deg_per_g": PRE_FIX.get(k)}
            with open(os.path.join(OUT_DIR, f"k2_{k}.json"), "w") as fh:
                json.dump(rec, fh, indent=1, default=float)
            _log(f"[{k}] K2 {k2['K_deg_per_g']:+.3f} deg/g CI [{k2['ci95_deg_per_g'][0]:+.3f}, "
                 f"{k2['ci95_deg_per_g'][1]:+.3f}], traversals {k2['n_traversals']}, n {k2['n_samples']}"
                 + (f", pre-fix {PRE_FIX[k]:+.2f}" if k in PRE_FIX else "") + f", {time.perf_counter() - t0:.0f} s")
        except Exception as e:  # one file failing never stops the run
            _log(f"[{k}] EXCEPTION {type(e).__name__}: {e}")


def _load(k):
    p = os.path.join(OUT_DIR, f"k2_{k}.json")
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def report():
    L = []
    rows = [r for r in (_load(k) for k in ("pr_us_v2", "v3") + NBR16) if r]
    for r in rows:
        k2 = r["K2"]
        L.append(f"{r['title']}: {k2['K_deg_per_g']:+.2f} deg/g [{k2['ci95_deg_per_g'][0]:+.2f}, "
                 f"{k2['ci95_deg_per_g'][1]:+.2f}] ({k2['n_traversals']} traversals)"
                 + (f"; pre-fix {r['pre_fix_deg_per_g']:+.2f}" if r["pre_fix_deg_per_g"] is not None else ""))
    nbr = np.array([r["K2"]["K_deg_per_g"] for r in rows if r["key"] in NBR16])
    if nbr.size:
        L.append(f"NBR ({nbr.size}): median {np.median(nbr):+.2f}, range {nbr.min():+.2f} to {nbr.max():+.2f} deg/g, "
                 f"SD {np.std(nbr, ddof=1):.2f}; sign positive (understeer) on {int(np.sum(nbr > 0))} / {nbr.size}")
    txt = "\n".join(L)
    print(txt)
    with open(os.path.join(OUT_DIR, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")


def _verdict_rows(k):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners)
    from modules.recommendation import aggregate_by_corner
    from ui.views.outing_form import OutingForm
    ctx = lp.Ctx(k)
    st, prm = ctx.state, ctx.params
    beta = np.asarray(ctx.cache["prod"]["beta"], float)
    cs = estimate_cornering_stiffness(estimate_slip_angles(st, beta, prm), estimate_lateral_forces(st, prm), st, prm)
    for a in "fr":
        x, y = np.asarray(cs[f"CS_ratio_{a}"], float), np.asarray(ctx.cache["prod"]["down"][f"cs_{a}"], float)
        fin = np.isfinite(x) & np.isfinite(y)
        if not (np.array_equal(np.isfinite(x), np.isfinite(y)) and np.max(np.abs(x[fin] - y[fin])) <= 1e-5):
            raise SystemExit(f"STOP {k}: recomputed CS does not reproduce the cache")
    stab = estimate_yaw_moment_stability(st, beta, prm, ctx.data["laps"])
    laps = [l["lap_number"] for l in ctx.analysis_laps()]
    agg = aggregate_by_corner(summarise_corners(ctx.data["corners"], cs, stab, st, lap_filter=laps))
    rows = []
    for g in sorted(agg.values(), key=lambda g: g["stable_corner_id"]):
        sev, short, _long, colour = OutingForm._classify_corner(None, g)
        rows.append((g["stable_corner_id"], short, colour))
    return ctx.name, laps, rows


def mock():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ui.style import PANEL, PANEL_ALT, BORDER, TEXT, TEXT_MUTED, ACCENT
    r = _load(MOCK_KEY)
    k2 = r["K2"]
    name, laps, rows = _verdict_rows(MOCK_KEY)
    kv, (lo, hi) = k2["K_deg_per_g"], k2["ci95_deg_per_g"]
    word = "understeer" if lo > 0 else ("oversteer" if hi < 0 else "not resolved from neutral")
    reading = {"understeer": "the car understeers", "oversteer": "the car oversteers"}.get(
        word, "balance not resolved from neutral")
    line = (f"Measured balance in steady cornering: understeer gradient {kv:+.2f} deg/g "
            f"(95 % range {lo:+.2f} to {hi:+.2f}) -- {reading}.\n"
            "Positive = more steering needed per g of lateral acceleration; measured from steering, "
            "independent of the per-corner verdicts below.")
    n = len(rows)
    fig = plt.figure(figsize=(11, 1.6 + 0.32 * n), facecolor=PANEL)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, n + 4.2)
    y = n + 3.5
    ax.text(0.02, y, f"Stability Analysis -- {name} (analysis laps {laps[0]}-{laps[-1]})", color=TEXT,
            fontsize=12, va="center", weight="bold")
    y -= 1.0
    ax.add_patch(plt.Rectangle((0.015, y - 0.7), 0.97, 1.4, facecolor=PANEL_ALT, edgecolor=ACCENT, lw=1.2))
    ax.text(0.03, y, line, color=ACCENT, fontsize=9.5, va="center")
    y -= 1.25
    ax.text(0.03, y, "Per corner (worst lap, from the cornering stiffness ratio):", color=TEXT_MUTED, fontsize=9,
            va="center")
    for sid, short, colour in rows:
        y -= 1.0
        ax.add_patch(plt.Rectangle((0.03, y - 0.36), 0.012, 0.72, facecolor=colour, edgecolor=BORDER))
        ax.text(0.05, y, f"C{sid}", color=TEXT, fontsize=9, va="center")
        ax.text(0.10, y, short, color=TEXT, fontsize=9, va="center")
    ax.text(0.98, 0.25, "MOCK for the author's ship / kill ruling -- not app code", color=TEXT_MUTED, fontsize=7.5,
            ha="right", va="center")
    fig.savefig(os.path.join(OUT_DIR, f"mock_verdict_card_{MOCK_KEY}.png"), dpi=130, facecolor=PANEL)
    plt.close(fig)
    _log(f"[mock] rendered, {n} corners, line: {line}")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "a"
    {"a": lambda: step_a(sys.argv[2:]), "report": report, "mock": mock}[mode]()
