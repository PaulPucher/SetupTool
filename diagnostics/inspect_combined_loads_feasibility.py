# COMBINED-LOADS FEASIBILITY, part (c): is a load dependence of CS visible in
# our data at all? (author EXECUTE 2026-10-04, morning ruling 5; parts (a) /
# (b) closed -- the Level-1 Fy split is axle-level, CS sees axle sums.)
# Read-only: nothing in modules/, core/ or config/ changes.
#
# FROZEN RULE: this step produces NO VERDICT -- facts and plots only; the
# direction is the author's ruling. No fit, no model: binned medians only.
#
# RULES, frozen 2026-10-04 BEFORE any number.
# FILES: the 16 NBR files (Q-RECOVER-2 recover.json files_in_outing_order),
#   measured wheel loads available since Q-CAL-2 (event zero entries).
# CHAIN: production as the app (ui/views/outing_form.py non-cache branch):
#   fresh parse_csv, accuracy cap 1, resolve_sideslip_beta at the configured
#   sideslip_source, slip, forces, CS, estimate_vertical_loads with channels
#   and car_data (vertical_load_source = measured -> damper cascade).
# SAMPLES: finite CS on analysis laps inside corner instances
#   (inspect_imu_offsets.populations, the corner-instance population of the
#   calibration passes), per axle. Axle Fz = the production axle total (sum of
#   the cascade wheels); only samples where BOTH wheels of the axle are on the
#   damper tier enter (L4 with an L2 zero, "L4-rel"); the excluded share is
#   reported.
# BANDWIDTH (Tier B preprocessing): axle Fz low-passed with the production CS
#   filter (cs_filter_cutoff_hz, zero-phase Butterworth) before binning, so
#   both quantities share the CS chain's bandwidth.
# BINS: 1 kN wide on axle Fz, edges on whole kN. A bin's median is plotted
#   only with >= 500 samples from >= 3 files; every bin's counts are listed
#   (samples, files, corner instances). Spread shown: per-bin p25-p75 of CS
#   and the per-file bin medians (between-file scatter = the visible noise
#   floor). Noise reference reported: the median over bins of the IQR of the
#   per-file medians.
# STATED CONFOUND (no correction attempted): inside a corner the axle total
#   barely sees lateral transfer -- it varies with speed (aero) and with
#   longitudinal transfer (brake / throttle phase), so an axle-Fz bin is also
#   a speed / phase bin.
# OUTPUTS diagnostics/plots/combined_loads_feasibility/: cs_vs_fz_front_axle.png,
#   cs_vs_fz_rear_axle.png, summary.txt, results.json, cache/<key>.npz,
#   run_log.txt.
#
# Usage (repo root, venv active):
#   python -m diagnostics.inspect_combined_loads_feasibility compute <key>
#   python -m diagnostics.inspect_combined_loads_feasibility report

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

OUT_DIR = "diagnostics/plots/combined_loads_feasibility"
CACHE = os.path.join(OUT_DIR, "cache")
BIN_KN = 1.0
MIN_SAMPLES, MIN_FILES = 500, 3
AXNAME = {"f": "front", "r": "rear"}
WHEELS = {"f": ("fl", "fr"), "r": ("rl", "rr")}


def files():
    with open("diagnostics/plots/recover_rr/recover.json") as fh:
        return json.load(fh)["files_in_outing_order"]


def compute(key):
    from diagnostics import inspect_imu_offsets as imu
    from diagnostics.inspect_nbr_zero_entry_verify import _path
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (load_parameters, load_car_data, prepare_vehicle_state,
                                            estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_vertical_loads,
                                            _butterworth_lowpass)
    from modules.tyre_fit_auto import resolve_sideslip_beta
    path = _path(key)
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
    data = parse_csv(path)
    st = prepare_vehicle_state(data["channels"], params)
    src = params["stability_estimation"]["sideslip_source"]
    beta, _man, gate, fb, _fr = resolve_sideslip_beta(st, params, data, src, csv_path=path)
    slip = estimate_slip_angles(st, beta, params)
    forces = estimate_lateral_forces(st, params)
    cs = estimate_cornering_stiffness(slip, forces, st, params)
    fz = estimate_vertical_loads(st, forces, params, channels=data["channels"], car_data=load_car_data())
    laps = [l for l in data["laps"] if l.get("is_valid_for_analysis")]
    t = st["time"]
    on_lap = np.zeros(t.size, dtype=bool)
    for l in laps:
        on_lap |= (t >= l["start_time"]) & (t <= l["end_time"])
    _r, _thr, _s0, _s1, inst = imu.populations(st, data["laps"], data["corners"])
    in_corner = np.zeros(t.size, dtype=bool)
    inst_id = np.full(t.size, -1, dtype=np.int64)
    for i, (_sid, _lap, sl) in enumerate(inst):
        in_corner[sl] = True
        inst_id[sl] = i
    fc, sr = params["stability_estimation"]["cs_filter_cutoff_hz"], st["sample_rate_hz"]
    srcs = fz["vertical_load_source_per_sample"] or {}
    out = {"gate": np.array(str((gate or {}).get("verdict"))), "fallback": np.array(bool(fb)),
           "load_source": np.array(str(fz["vertical_load_source_used"]))}
    for ax in "fr":
        c = np.asarray(cs[f"CS_ratio_{ax}"], float)
        f = _butterworth_lowpass(np.asarray(fz[f"fz_{ax}_N"], float), fc, sr)
        damper = np.ones(t.size, dtype=bool)
        for w in WHEELS[ax]:
            damper &= (np.asarray(srcs[w]) == "damper") if w in srcs else False
        base = np.isfinite(c) & on_lap & in_corner & np.isfinite(f)
        keep = base & damper
        out[f"cs_{ax}"] = c[keep]
        out[f"fz_{ax}"] = f[keep]
        out[f"inst_{ax}"] = inst_id[keep]
        out[f"n_base_{ax}"] = np.array(int(base.sum()))
    os.makedirs(CACHE, exist_ok=True)
    np.savez_compressed(os.path.join(CACHE, f"{key}.npz"), **out)
    print(f"[{key}] gate {out['gate']} fallback {fb} load source {out['load_source']}; "
          + ", ".join(f"{ax}: {out[f'cs_{ax}'].size} of {int(out[f'n_base_{ax}'])} samples damper-tier"
                      for ax in "fr"), flush=True)


def report():
    from diagnostics import inspect_last_push as lp
    plt = lp._plt()
    keys = [k for k in files() if os.path.exists(os.path.join(CACHE, f"{k}.npz"))]
    data = {k: dict(np.load(os.path.join(CACHE, f"{k}.npz"))) for k in keys}
    lines = ["COMBINED-LOADS FEASIBILITY (c): median in-corner CS per axle-Fz bin, 16 NBR files -- NO VERDICT",
             f"files computed {len(keys)} of 16; missing {[k for k in files() if k not in keys]}"]
    for k in keys:
        d = data[k]
        lines.append(f"  {k}: gate {d['gate']}, fallback {d['fallback']}, load source {d['load_source']}; "
                     + ", ".join(f"{AXNAME[ax]} {d[f'cs_{ax}'].size} / {int(d[f'n_base_{ax}'])} damper-tier"
                                 for ax in "fr"))
    res = {}
    for ax in "fr":
        cs = np.concatenate([data[k][f"cs_{ax}"] for k in keys])
        fz = np.concatenate([data[k][f"fz_{ax}"] for k in keys]) / 1000.0
        fid = np.concatenate([np.full(data[k][f"cs_{ax}"].size, i) for i, k in enumerate(keys)])
        inst = np.concatenate([data[k][f"inst_{ax}"] + 100000 * i for i, k in enumerate(keys)])
        edges = np.arange(np.floor(fz.min()), np.ceil(fz.max()) + BIN_KN, BIN_KN)
        rows = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (fz >= lo) & (fz < hi)
            n, nf = int(m.sum()), int(np.unique(fid[m]).size)
            row = {"lo_kN": float(lo), "hi_kN": float(hi), "n": n, "n_files": nf,
                   "n_instances": int(np.unique(inst[m]).size)}
            if n:
                q = np.percentile(cs[m], [25, 50, 75])
                pf = [float(np.median(cs[m & (fid == i)])) for i in np.unique(fid[m]) if (m & (fid == i)).sum() >= 50]
                row.update(p25=float(q[0]), median=float(q[1]), p75=float(q[2]), per_file_medians=pf,
                           plotted=bool(n >= MIN_SAMPLES and nf >= MIN_FILES))
            rows.append(row)
        pl = [r for r in rows if r.get("plotted")]
        file_iqr = [float(np.subtract(*np.percentile(r["per_file_medians"], [75, 25])))
                    for r in pl if len(r["per_file_medians"]) >= 3]
        res[ax] = {"rows": rows, "n_total": int(cs.size),
                   "noise_ref_median_per_file_iqr": float(np.median(file_iqr)) if file_iqr else float("nan")}
        lines.append(f"\n{AXNAME[ax].upper()} AXLE: {cs.size} samples, axle Fz {fz.min():.1f}-{fz.max():.1f} kN "
                     f"(2 Hz); plotted bins need >= {MIN_SAMPLES} samples from >= {MIN_FILES} files")
        for r in rows:
            if not r["n"]:
                continue
            tag = "" if r.get("plotted") else "  (not plotted)"
            lines.append(f"  {r['lo_kN']:.0f}-{r['hi_kN']:.0f} kN: n {r['n']}, files {r['n_files']}, corner instances "
                         f"{r['n_instances']}; CS median {r['median']:+.3f} (p25 {r['p25']:+.3f}, p75 {r['p75']:+.3f}); "
                         f"per-file medians {min(r['per_file_medians'], default=float('nan')):+.3f} to "
                         f"{max(r['per_file_medians'], default=float('nan')):+.3f}{tag}")
        if pl:
            meds = [r["median"] for r in pl]
            lines.append(f"  plotted-bin medians span {min(meds):+.3f} to {max(meds):+.3f} (range "
                         f"{max(meds) - min(meds):.3f}); noise reference (median over bins of the per-file-median "
                         f"IQR) {res[ax]['noise_ref_median_per_file_iqr']:.3f}")
        fig, A = plt.subplots(figsize=(11, 5.5))
        for r in pl:
            x = 0.5 * (r["lo_kN"] + r["hi_kN"])
            A.plot([x, x], [r["p25"], r["p75"]], color=lp.REF, lw=6, alpha=0.5, solid_capstyle="butt")
            A.scatter(np.full(len(r["per_file_medians"]), x) + np.linspace(-0.25, 0.25, len(r["per_file_medians"])),
                      r["per_file_medians"], s=10, color=lp.ACC2, alpha=0.7)
        A.plot([0.5 * (r["lo_kN"] + r["hi_kN"]) for r in pl], [r["median"] for r in pl], "o-", color=lp.INK, lw=1.5)
        for r in pl:
            A.annotate(f"n={r['n']}\n{r['n_files']} files", (0.5 * (r["lo_kN"] + r["hi_kN"]), r["p75"]),
                       textcoords="offset points", xytext=(0, 6), ha="center", fontsize=6.5)
        A.plot([], [], "o-", color=lp.INK, label="pooled median of the bin")
        A.plot([], [], color=lp.REF, lw=6, alpha=0.5, label="25th-75th percentile of the bin")
        A.scatter([], [], s=10, color=lp.ACC2, label="median of one file in the bin (>= 50 samples)")
        A.axhline(0, color=lp.REF, lw=0.8)
        A.set_xlabel(f"{AXNAME[ax]} axle vertical load, measured (damper gauges, 2 Hz) [kN]")
        A.set_ylabel(f"{AXNAME[ax]} cornering-stiffness ratio CS")
        A.legend(fontsize=7, loc="lower left")
        lp._style(A)
        fig.suptitle(lp._wrap(f"Nurburgring, 16 outings: in-corner {AXNAME[ax]}-axle cornering-stiffness ratio "
                              f"binned by measured {AXNAME[ax]}-axle load (1 kN bins, medians only, no fit)", fig),
                     fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.92))
        fig.savefig(os.path.join(OUT_DIR, f"cs_vs_fz_{AXNAME[ax]}_axle.png"), dpi=130)
        plt.close(fig)
    text = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "summary.txt"), "w") as fh:
        fh.write(text)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(res, fh, indent=1)
    print(text)


if __name__ == "__main__":
    if sys.argv[1] == "compute":
        compute(sys.argv[2])
    else:
        report()
