# Gated-kinematic diagnostic, step 2 of 4: cornering-stiffness comparison.
# Metrics 1 and 2 of the work order: per corner, the verdict-relevant worst
# CS_ratio (min over lap x phase of phase medians, apex_3 read through
# apex_region -- the population _classify_corner sees under worst_lap
# aggregation) and the STEP 2 chair-plot quantity (C_alpha at the most
# saturated sample inside the canonical window, pooled over valid laps),
# for kinematic_prod / ekf_auto_pacejka / gated x3. Reads the cache from
# inspect_gated_kinematic_run. Read-only.
#
# Run from the repo root:  python -m diagnostics.inspect_gated_kinematic_cs

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.stability_analysis import load_parameters
from diagnostics.gated_kinematic_common import (
    SESSIONS, OUT_DIR, load_cache, valid_corner_instances,
)
from diagnostics.gated_kinematic_style import SERIES, TEXT, style_axes

CS_SOURCES = ("kinematic_prod", "ekf_auto_pacejka", "gated_x1", "gated_x0.75", "gated_x1.5")
NON_APEX = ("entry_1_brake", "entry_2_turnin", "exit_4", "exit_5")
Y_LO = -5.0  # plot clip only; values below are drawn at the edge and labelled


def _median(stat):
    v = (stat or {}).get("median")
    return float(v) if v is not None and np.isfinite(v) else np.nan


def worst_cs_ratio(summaries, axle):
    """{sid: (value, lap, phase)} -- min over (lap, phase) of phase medians."""
    key = f"cs_ratio_{axle}"
    out = {}
    for s in summaries:
        sid = s.get("stable_corner_id")
        if sid is None:
            continue
        cands = [(_median(s["phases"].get(p, {}).get(key)), p) for p in NON_APEX]
        if s.get("apex_region"):
            cands.append((_median(s["apex_region"].get(key)), "apex_region"))
        for val, phase in cands:
            if np.isfinite(val) and (sid not in out or val < out[sid][0]):
                out[sid] = (val, s["lap_number"], phase)
    return out


def worst_sample_c_alpha(chain, axle, instances):
    """STEP 2 quantity: C_alpha at the lowest-CS_ratio sample per corner."""
    cs_ratio = chain[f"CS_ratio_{axle}"]
    c_alpha = chain[f"C_alpha_{axle}"]
    out = {}
    for sid, lap, sl in instances:
        seg = cs_ratio[sl]
        if not np.isfinite(seg).any():
            continue
        i = sl.start + int(np.nanargmin(np.where(np.isfinite(seg), seg, np.inf)))
        if sid not in out or cs_ratio[i] < out[sid][0]:
            out[sid] = (float(cs_ratio[i]), float(c_alpha[i]), lap)
    return out


def analyse(name, cls):
    cache = load_cache(name)
    st = cache["state"]
    inst = valid_corner_instances({"laps": cache["laps"], "corners": cache["corners"]},
                                  st["time"], st["s_m"])
    res = {"session": name, "ekf": cache["ekf"], "thr_otsu": cache["thr_otsu"], "sources": {}}
    for src in CS_SOURCES:
        ch = cache["chains"][src]
        r = {}
        for axle in ("f", "r"):
            w = worst_cs_ratio(ch["summaries"], axle)
            ws = worst_sample_c_alpha(ch, axle, inst)
            r[axle] = {str(k): {"worst_cs_ratio": v[0], "lap": v[1], "phase": v[2],
                                "step2_cs_ratio": ws.get(k, (np.nan,))[0],
                                "step2_c_alpha": ws.get(k, (np.nan, np.nan))[1]}
                       for k, v in sorted(w.items())}
        strong = {"f": cls["STRONG_CSF"]["value"], "r": cls["STRONG_CSR"]["value"]}
        r["n_negative"] = {a: sum(1 for d in r[a].values() if d["worst_cs_ratio"] < 0) for a in ("f", "r")}
        r["n_strong"] = {a: sum(1 for d in r[a].values() if d["worst_cs_ratio"] < strong[a]) for a in ("f", "r")}
        r["n_corners"] = {a: len(r[a]) for a in ("f", "r")}
        res["sources"][src] = r
    return res


def print_report(res, cls):
    name = res["session"]
    print(f"\n===== {name}  (Otsu |ay| threshold {res['thr_otsu']:.3f} m/s^2; EKF {res['ekf']})")
    print(f"STRONG_CSF={cls['STRONG_CSF']['value']}, STRONG_CSR={cls['STRONG_CSR']['value']} (read-only)")
    for src in CS_SOURCES:
        r = res["sources"][src]
        print(f"  {src:18s} negative worst CS_ratio: front {r['n_negative']['f']}/{r['n_corners']['f']}, "
              f"rear {r['n_negative']['r']}/{r['n_corners']['r']}; below STRONG: front {r['n_strong']['f']}, rear {r['n_strong']['r']}")
    for src in CS_SOURCES:
        r = res["sources"][src]
        cnt = {}
        for a in ("f", "r"):
            for d in r[a].values():
                if d["worst_cs_ratio"] < 0:
                    cnt[d["phase"]] = cnt.get(d["phase"], 0) + 1
        pinned = sum(1 for a in ("f", "r") for d in r[a].values() if d["worst_cs_ratio"] >= 0.999)
        print(f"  {src:18s} phase of negative worst values: {dict(sorted(cnt.items()))}; corners pinned at +1.0: {pinned}")
    sids = sorted({int(k) for src in CS_SOURCES for a in ("f", "r") for k in res["sources"][src][a]})
    for axle, label in (("f", "FRONT"), ("r", "REAR")):
        print(f"  -- {label}: worst CS_ratio [lap/phase] | STEP2 C_alpha at worst sample (N/rad)")
        for sid in sids:
            parts = []
            for src in CS_SOURCES:
                d = res["sources"][src][axle].get(str(sid))
                if d is None:
                    parts.append(f"{src}=n/a")
                else:
                    parts.append(f"{src}={d['worst_cs_ratio']:+.3f}[L{d['lap']}/{d['phase']}]|{d['step2_c_alpha']:+.0f}")
            print(f"    C{sid}: " + "; ".join(parts))


def plot(res, cls):
    sids = sorted({int(k) for src in CS_SOURCES for a in ("f", "r") for k in res["sources"][src][a]})
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    offsets = np.linspace(-0.28, 0.28, len(CS_SOURCES))
    for ax, axle, strong_key in ((axes[0], "f", "STRONG_CSF"), (axes[1], "r", "STRONG_CSR")):
        for off, src in zip(offsets, CS_SOURCES):
            ys = np.array([res["sources"][src][axle].get(str(s), {}).get("worst_cs_ratio", np.nan) for s in sids])
            st = SERIES[src]
            xs = np.arange(len(sids)) + off
            ax.plot(xs, np.clip(ys, Y_LO, None), linestyle="none", marker=st["marker"],
                    markersize=6, color=st["color"], label=st["label"])
            for x, y in zip(xs, ys):
                if np.isfinite(y) and y < Y_LO:
                    ax.annotate(f"{y:.1f}", (x, Y_LO), xytext=(0, 7), textcoords="offset points",
                                ha="center", fontsize=6, color=TEXT)
        ax.axhline(0.0, color=TEXT, linewidth=0.8)
        ax.axhline(cls[strong_key]["value"], color=TEXT, linewidth=0.8, linestyle=":")
        ax.text(len(sids) - 0.5, cls[strong_key]["value"], f" {strong_key}", va="center", fontsize=7, color=TEXT)
        ax.set_ylim(Y_LO - 0.3, 1.3)
        ax.set_ylabel(f"worst CS_ratio, {'front' if axle == 'f' else 'rear'} axle\n(below {Y_LO:g}: at edge, value shown)", fontsize=8)
        style_axes(ax)
    axes[1].set_xticks(np.arange(len(sids)))
    axes[1].set_xticklabels([f"C{s}" for s in sids], fontsize=8)
    axes[0].set_title(f"{res['session']}: worst-lap worst-phase CS_ratio per corner by beta source", fontsize=10, color=TEXT)
    axes[0].legend(fontsize=7, loc="lower left", ncol=2, frameon=False)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, f"cs_worst_by_source_{res['session']}.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main():
    cls = load_parameters()["classification"]
    all_res = {}
    for name in SESSIONS:
        try:
            res = analyse(name, cls)
        except FileNotFoundError:
            print(f"[{name}] no cache -- run inspect_gated_kinematic_run first")
            continue
        print_report(res, cls)
        print(f"  figure: {plot(res, cls)}")
        all_res[name] = res
    with open(os.path.join(OUT_DIR, "cs_results.json"), "w") as f:
        json.dump(all_res, f, indent=1, default=float)


if __name__ == "__main__":
    main()
