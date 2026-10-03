# How did Q-CAL item 1 (options 4 + 0, shipped 2026-10-03) move CS?
# Read-only: nothing in modules/, core/ or config/ changes. Author EXECUTE
# 2026-10-03 with rulings: (1) cached beta, isolated mask effect -- the
# end-to-end movement comes with the later Q-CAL levers; (2) site value =
# median over laps as dot pairs; the per-lap list covers lap level.
#
# RULES, frozen before the run.
# SESSIONS: pr_us_v2, v3, nbr_jst_solution (Tier 1) and dubai.
# OLD STATE: today's code with kerb_dilation_s forced to 0.05 s (= the old 5
#   samples at 100 Hz), cached production beta. ENTRY: must reproduce the
#   cached production CS (fit cache down.cs_f / cs_r): identical finite set,
#   max |diff| <= 1e-6, else the session is dropped with the reason.
# NEW STATE: current production code and config, same cached beta -- isolates
#   the mask's effect on CS; the tyre re-fit / new EKF beta that option 0 also
#   causes in a full production run is NOT included.
# a. Population: finite CS on analysis laps; in-corner subset (corner
#   instances) reported alongside. Histogram overlay old vs new per axle,
#   bins 0.05 on [-3, 1.05], values below -3 collected in the leftmost bin,
#   log count axis. Plain list: p1 / p5 / p10 / p50, share CS < 0 and
#   CS < -1, n -- old -> new.
# b. Position-locked sites (Q-PAIRING sites_<key>.json, Tier 1): per site and
#   lap the dip = the lap's CS minimum within +/-10 m inside a corner
#   instance (Q-PAIRING's rule), old and new; site value = median over laps
#   (laps without a finite new CS are left out of the new median and
#   counted). Marked = near_strike at the new dip on >= half the laps.
# c. Trace: the largest single site-lap dip change of the verify run, US-v2
#   front 2893 m lap 52; CS over distance +/-60 m, old vs new; old mask,
#   new-only mask and the smear zone shaded.
#
# Usage (repo root, venv active): python -m diagnostics.inspect_kerb_cs_movement

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from diagnostics import inspect_last_push as lp

OUT_DIR = "diagnostics/plots/kerb_cs_movement"
KEYS = ("pr_us_v2", "v3", "nbr_jst_solution", "dubai")
TIER1 = ("pr_us_v2", "v3", "nbr_jst_solution")
OLD_DILATION_S = 0.05
ENTRY_TOL = 1e-6
SITE_TOL_M = 10.0
BINS = np.arange(-3.0, 1.05 + 1e-9, 0.05)
EXHIBIT = ("pr_us_v2", "f", 2893, 52)
EXHIBIT_SPAN_M = 60.0
AXNAME = {"f": "front", "r": "rear"}


def _log(m):
    print(m, flush=True)


def session(key):
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, prepare_vehicle_state)
    from diagnostics.gated_kinematic_common import valid_corner_instances
    ctx = lp.Ctx(key)
    p_new = ctx.params
    p_old = json.loads(json.dumps(p_new))
    p_old["stability_estimation"]["kerb_dilation_s"] = OLD_DILATION_S
    beta = np.asarray(ctx.cache["prod"]["beta"], dtype=float)
    res = {"key": key, "name": ctx.name}
    for tag, prm in (("old", p_old), ("new", p_new)):
        st = prepare_vehicle_state(ctx.data["channels"], prm)
        slip = estimate_slip_angles(st, beta, prm)
        forces = estimate_lateral_forces(st, prm)
        res[tag] = (st, estimate_cornering_stiffness(slip, forces, st, prm))
    entry = {}
    for ax in ("f", "r"):
        cached = np.asarray(ctx.cache["prod"]["down"][f"cs_{ax}"], dtype=float)
        old = res["old"][1][f"CS_ratio_{ax}"]
        same = bool(np.array_equal(np.isfinite(old), np.isfinite(cached)))
        fin = np.isfinite(old) & np.isfinite(cached)
        md = float(np.max(np.abs(old[fin] - cached[fin]))) if fin.any() else 0.0
        entry[ax] = (same and md <= ENTRY_TOL, same, md)
    res["entry"] = entry
    st = res["new"][0]
    t, s = st["time"], st["s_m"]
    on_lap = np.zeros(t.size, dtype=bool)
    laps = {}
    for l in ctx.data["laps"]:
        if l.get("is_valid_for_analysis"):
            m = (t >= l["start_time"]) & (t <= l["end_time"])
            laps[l["lap_number"]] = m
            on_lap |= m
    in_br = np.zeros(t.size, dtype=bool)
    for _sid, _lap, sl in valid_corner_instances({"laps": ctx.data["laps"], "corners": ctx.data["corners"]}, t, s):
        in_br[sl] = True
    res.update(t=t, s=s, on_lap=on_lap, in_br=in_br, laps=laps)
    return res


def pct_line(x):
    if x.size == 0:
        return "n 0"
    p = np.percentile(x, [1, 5, 10, 50])
    return (f"p1 {p[0]:.3f} p5 {p[1]:.3f} p10 {p[2]:.3f} p50 {p[3]:.3f}; CS<0 {np.mean(x < 0) * 100:.2f} %, "
            f"CS<-1 {np.mean(x < -1) * 100:.2f} %; n {x.size}")


def part_a(res, lines):
    plt = lp._plt()
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    for k, ax in enumerate(("f", "r")):
        A = axs[k]
        for tag, col, lab in (("old", lp.REF, "before (kerb mask +/-50 ms)"),
                              ("new", lp.ACC, "after (kerb mask +/-100 ms)")):
            cs = res[tag][1][f"CS_ratio_{ax}"]
            x = cs[np.isfinite(cs) & res["on_lap"]]
            A.hist(np.clip(x, BINS[0] + 1e-9, BINS[-1] - 1e-9), bins=BINS, histtype="step", lw=1.5,
                   color=col, label=lab)
        A.set_yscale("log")
        A.axvline(0.0, color=lp.INK, lw=0.7, ls=":")
        A.set_xlabel("cornering stiffness ratio CS [-] (values below -3 in the leftmost bin)")
        A.set_ylabel("samples (log scale)")
        A.set_title(f"{AXNAME[ax]} axle", fontsize=10)
        lp._style(A)
        for pop, sel in (("analysis laps", res["on_lap"]), ("in-corner", res["on_lap"] & res["in_br"])):
            o = res["old"][1][f"CS_ratio_{ax}"]
            n = res["new"][1][f"CS_ratio_{ax}"]
            lines.append(f"  {AXNAME[ax]}, {pop}:")
            lines.append(f"    before: {pct_line(o[np.isfinite(o) & sel])}")
            lines.append(f"    after:  {pct_line(n[np.isfinite(n) & sel])}")
    axs[0].legend(fontsize=8)
    fig.suptitle(lp._wrap(f"{res['name']}: cornering stiffness ratio on analysis laps, before vs after the "
                          f"kerb-mask unit repair (+/-50 ms -> +/-100 ms), same sideslip input", fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    path = os.path.join(OUT_DIR, f"cs_distribution_{res['key']}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def site_rows(res):
    with open(f"diagnostics/plots/pairing/sites_{res['key']}.json") as fh:
        sites = json.load(fh)
    s = res["s"]
    rows = []
    for ax in ("f", "r"):
        old = res["old"][1][f"CS_ratio_{ax}"]
        new = res["new"][1][f"CS_ratio_{ax}"]
        mark = res["new"][1][f"near_strike_{ax}"]
        for site in sites["axles"][ax]["sites"]:
            laps = []
            for row in site["laps"]:
                near = res["laps"][row["lap"]] & (np.abs(s - site["pos_m"]) <= SITE_TOL_M) & res["in_br"]
                ko = near & np.isfinite(old)
                jo = int(np.flatnonzero(ko)[np.argmin(old[ko])])
                kn = near & np.isfinite(new)
                if kn.any():
                    jn = int(np.flatnonzero(kn)[np.argmin(new[kn])])
                    laps.append((row["lap"], float(old[jo]), float(new[jn]), bool(mark[jn])))
                else:
                    laps.append((row["lap"], float(old[jo]), None, False))
            n_mark = sum(x[3] for x in laps)
            new_vals = [x[2] for x in laps if x[2] is not None]
            rows.append({"key": res["key"], "name": res["name"], "ax": ax, "pos": site["pos_m"],
                         "verdict": site["verdict"], "b": "B" in site["flags"],
                         "old_med": float(np.median([x[1] for x in laps])),
                         "new_med": float(np.median(new_vals)) if new_vals else float("nan"),
                         "lost": len(laps) - len(new_vals), "marked": n_mark >= len(laps) / 2.0,
                         "n_mark": n_mark, "laps": laps})
    return rows


def part_b(rows, lines):
    plt = lp._plt()
    fig, A = plt.subplots(figsize=(14, 6))
    short = {"pr_us_v2": "Paul Ricard US-v2", "v3": "Paul Ricard v3", "nbr_jst_solution": "Nurburgring JST 29"}
    for x, r in enumerate(rows):
        A.plot([x, x], [r["old_med"], r["new_med"]], color=lp.REF, lw=1.0)
        A.scatter([x], [r["old_med"]], s=36, facecolors="none", edgecolors=lp.INK, zorder=3)
        A.scatter([x], [r["new_med"]], s=40, color=lp.ACC2 if r["marked"] else lp.ACC,
                  marker="D" if r["marked"] else "o", zorder=4)
    A.scatter([], [], s=36, facecolors="none", edgecolors=lp.INK, label="before (median dip over laps)")
    A.scatter([], [], s=40, color=lp.ACC, marker="o", label="after, no near-strike mark")
    A.scatter([], [], s=40, color=lp.ACC2, marker="D", label="after, carries the near-strike mark")
    A.axhline(0.0, color=lp.INK, lw=0.7, ls=":")
    A.set_xticks(range(len(rows)))
    A.set_xticklabels([f"{short[r['key']]}\n{AXNAME[r['ax']]} {r['pos']:.0f} m" for r in rows],
                      rotation=60, ha="right", fontsize=7)
    A.set_ylabel("dip CS at the site [-]")
    A.legend(fontsize=8)
    lp._style(A)
    fig.suptitle(lp._wrap("The 22 position-locked CS dip sites: median dip before vs after the kerb-mask unit "
                          "repair (+/-50 ms -> +/-100 ms), same sideslip input; diamonds = sites carrying the "
                          "near-strike grade mark", fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    path = os.path.join(OUT_DIR, "site_dips_before_after.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    changed = 0
    for r in rows:
        lines.append(f"  {r['name']} {AXNAME[r['ax']]} {r['pos']:.0f} m ({r['verdict']}): median dip "
                     f"{r['old_med']:.3f} -> {r['new_med']:.3f}; laps lost {r['lost']}; near-strike on "
                     f"{r['n_mark']}/{len(r['laps'])} -> {'MARKED' if r['marked'] else 'unmarked'}")
        for lap, o, n, _m in r["laps"]:
            if n is None or abs(n - o) > 1e-6:
                changed += 1
                lines.append(f"      lap {lap}: {o:.3f} -> " + ("no finite CS within +/-10 m" if n is None else f"{n:.3f}"))
    lines.append(f"  changed site-laps: {changed} (verify run: 27)")
    return path


def part_c(res):
    key, ax, pos, lap = EXHIBIT
    plt = lp._plt()
    s = res["s"]
    sel = res["laps"][lap] & (np.abs(s - pos) <= EXHIBIT_SPAN_M)
    idx = np.flatnonzero(sel)
    idx = idx[np.argsort(s[idx])]
    x = s[idx]
    st_o, cs_o = res["old"]
    st_n, cs_n = res["new"]
    fig, A = plt.subplots(figsize=(12, 5))

    def shade(m, color, alpha, label, hatch=None):
        first = True
        for a, b in _runs(m[idx]):
            A.axvspan(x[a], x[b], color=color, alpha=alpha, lw=0, hatch=hatch, label=label if first else None)
            first = False

    shade(st_o["kerb_mask"], lp.REF, 0.45, "kerb mask before (+/-50 ms)")
    shade(st_n["kerb_mask"] & ~st_o["kerb_mask"], lp.ACC2, 0.35, "added by the +/-100 ms mask")
    shade(st_n["kerb_smear_zone"], lp.HL, 0.5, "smear zone (0.20 s, grade mark)")
    A.plot(x, cs_o[f"CS_ratio_{ax}"][idx], color=lp.INK, lw=1.4, marker=".", ms=3, label="CS before")
    A.plot(x, cs_n[f"CS_ratio_{ax}"][idx], color=lp.ACC, lw=1.4, marker=".", ms=3, label="CS after")
    A.axvline(pos, color=lp.INK, lw=0.6, ls="--")
    A.axhline(0.0, color=lp.INK, lw=0.6, ls=":")
    A.set_xlabel("lap distance [m]")
    A.set_ylabel(f"{AXNAME[ax]} cornering stiffness ratio CS [-]")
    A.legend(fontsize=8, loc="lower left")
    lp._style(A)
    fig.suptitle(lp._wrap(f"{res['name']}, lap {lap}, {AXNAME[ax]} axle: cornering stiffness ratio around the "
                          f"position-locked dip at {pos} m, before vs after the kerb-mask unit repair (largest "
                          f"single dip change of the verify run, -0.700 -> +0.218)", fig), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    path = os.path.join(OUT_DIR, f"trace_{key}_{AXNAME[ax]}_{pos}m_lap{lap}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def _runs(m):
    mm = np.concatenate(([False], m, [False])).astype(np.int8)
    d = np.diff(mm)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    lines = ["Q-CAL item 1: CS movement, before (+/-50 ms mask) vs after (+/-100 ms), cached production beta"]
    plots = []
    rows = []
    for key in KEYS:
        res = session(key)
        ok = all(v[0] for v in res["entry"].values())
        lines.append(f"\n[{key}] {res['name']}: entry " + ", ".join(
            f"{AXNAME[a]} same set {v[1]} max diff {v[2]:.2e}" for a, v in res["entry"].items())
            + (" -> PASS" if ok else " -> FAIL, session dropped"))
        if not ok:
            continue
        lines.append(f"  mask samples {int(res['old'][0]['kerb_mask'].sum())} -> {int(res['new'][0]['kerb_mask'].sum())}")
        plots.append(part_a(res, lines))
        if key in TIER1:
            rows.extend(site_rows(res))
        if key == EXHIBIT[0]:
            plots.append(part_c(res))
        _log(f"[{key}] done")
    order = {k: i for i, k in enumerate(TIER1)}
    rows.sort(key=lambda r: (order[r["key"]], r["ax"], r["pos"]))
    lines.append("\nPOSITION-LOCKED SITES")
    plots.append(part_b(rows, lines))
    lines.append("\nPLOTS\n" + "\n".join(plots))
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    _log(text)


if __name__ == "__main__":
    main()
