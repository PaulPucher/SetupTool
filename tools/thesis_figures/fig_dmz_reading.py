# fig:dmz_reading -- how dMz/dbeta is read: a partial-regression plot for
# one stable and one unstable corner phase of one session. The yaw-moment
# regression (yaw_stability.calculate_observed_stability) is a local
# weighted ridge regression over lap distance, pooled across all analysis
# laps: at each grid point, Mz = Iz * yaw acceleration (0.15 s centred mean)
# on beta, delta_f, v, a_x, a_z (standardised, Gaussian weights in s, ridge
# penalty). The plot takes the grid point of the phase whose value is
# closest to the stored phase value, removes the fitted contributions of
# every regressor except beta from the centred Mz (partial residual) and
# draws it over beta with the ridge slope through the weighted means.
# Samples within one Gaussian sigma of the grid point dark, the rest of the
# window light. The grid-point math is the module's, repeated here; build
# checks it against the chain's per-sample profile on the phase.
# Read-only full re-run (common.run_chain).
# Run: python -m tools.thesis_figures.fig_dmz_reading --outing 17 --unstable 11:exit_4
#      [--stable C:phase] [--tag r2] [--out DIR] [--render-only]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_dmz_reading"
PANEL_WIDTH = "0.4\\textwidth"
PANEL_HEIGHT = "0.34\\textwidth"
REPRO_TOL = 1e-6   # N m/deg, module profile vs the repeated grid-point math


def _inputs(session, res):
    """The regression's samples exactly as estimate_yaw_moment_stability
    hands them over: valid mask, predictors in the module's column order,
    Mz from the filtered yaw acceleration."""
    import pandas as pd
    from modules.stability_analysis import _build_non_analysis_lap_mask
    from modules.yaw_stability import _PREDICTOR_COLUMNS, calculate_filtered_yaw_acceleration
    st, p = res["state"], session["params"]
    se = p["stability_estimation"]
    t = st["time"]
    valid = st["moving_mask"] & ~st["kerb_mask"] & ~_build_non_analysis_lap_mask(t, session["parsed"]["laps"])
    mz = p["vehicle"]["yaw_inertia_kgm2"] * calculate_filtered_yaw_acceleration(
        st["yaw_rate_radps"], t, st["sample_rate_hz"], se["yaw_stability_accel_window_s"])
    arrays = {"beta_rad": np.asarray(res["beta"], float), "delta_f_rad": st["delta_f_rad"], "v_mps": st["v_mps"],
              "ax_mps2": st["ax_mps2"], "az_mps2": st["az_g"] * 9.81}
    cols = [c for c in _PREDICTOR_COLUMNS if c in arrays]
    work = pd.DataFrame({"s_m": st["s_m"], **arrays, "mz": mz, "t": t})
    work.loc[~valid, :] = np.nan
    work = work.replace([np.inf, -np.inf], np.nan).dropna().sort_values("s_m")
    return work, cols, se


def _grid_point(work, cols, se, center):
    """One grid point of calculate_observed_stability; returns the slope in
    N m/deg and the pieces the partial plot needs (None when skipped)."""
    from modules.yaw_stability import GAUSS_SIGMA_DIVISOR
    s = work["s_m"].to_numpy()
    lo, hi = np.searchsorted(s, center - se["yaw_stability_window_m"], "left"), \
        np.searchsorted(s, center + se["yaw_stability_window_m"], "right")
    if hi - lo < se["yaw_stability_min_samples"]:
        return None
    x = work[cols].to_numpy()[lo:hi]
    y = work["mz"].to_numpy()[lo:hi]
    sigma = max(se["yaw_stability_window_m"] / GAUSS_SIGMA_DIVISOR, 1e-9)
    d = np.abs(s[lo:hi] - center)
    w = np.exp(-0.5 * (d / sigma) ** 2)
    xm = (x * w[:, None]).sum(0) / w.sum()
    ym = float((y * w).sum() / w.sum())
    xc, yc = x - xm, y - ym
    scale = np.sqrt(np.maximum((w[:, None] * xc ** 2).sum(0) / w.sum(), 1e-12))
    if scale[0] < se["yaw_stability_min_beta_std_rad"]:
        return None
    xs = xc / scale
    a = xs * np.sqrt(w)[:, None]
    coef = np.linalg.solve(a.T @ a + se["yaw_stability_ridge"] * np.eye(xs.shape[1]), a.T @ (yc * np.sqrt(w)))
    slope_deg = coef[0] / scale[0] * np.pi / 180.0
    partial = yc - xs[:, 1:] @ coef[1:]
    return {"slope": float(slope_deg), "beta_deg": np.degrees(x[:, 0]), "beta_mean_deg": float(np.degrees(xm[0])),
            "partial": partial, "inner": d <= sigma, "t": work["t"].to_numpy()[lo:hi], "sigma": sigma,
            "coef": coef, "scale": scale}


def phase_panel(session, res, work, cols, se, agg, cid, phase):
    from modules.stability_analysis import apex_instant_slice
    st = res["state"]
    t, s_all = st["time"], st["s_m"]
    stored = agg[cid]["phases"][phase]["stability_observed_Nm_per_deg"]["median"]
    # the phase's lap-distance range: canonical, the same on every lap
    inst = [c for c in res["corners"] if c.get("stable_corner_id") == cid and c["lap_number"] in session["ident"]["lap_filter"]]
    half = session["params"]["stability_estimation"]["apex_half_window_samples"]

    def phase_idx(c):
        # the samples summarise_corners reads: the apex instant as its fixed slice
        a, b = c["segments"][phase]
        if phase == "apex_3":
            sl = apex_instant_slice(t, a, half)
            return np.arange(sl.start, sl.stop)
        return np.arange(np.searchsorted(t, a), np.searchsorted(t, b, "right"))
    i0 = phase_idx(inst[0])
    s0, s1 = float(s_all[i0[0]]), float(s_all[i0[-1]])
    s_work = work["s_m"].to_numpy()
    step = se["yaw_stability_grid_step_m"]
    grid = np.arange(s_work.min(), s_work.max() + step, step)
    near = grid[(grid >= s0 - 2 * step) & (grid <= s1 + 2 * step)]
    vals = {g: _grid_point(work, cols, se, g) for g in near}
    gv = np.array([v["slope"] if v else np.nan for v in vals.values()])
    # reproduction: the module interpolates its grid values onto the samples
    prof = np.asarray(res["stab"]["stability_observed_Nm_per_deg"], float)
    worst = 0.0
    for c in inst:
        idx = phase_idx(c)
        idx = idx[np.isfinite(prof[idx]) & np.isfinite(s_all[idx])]
        if idx.size:
            ok = np.isfinite(gv)
            mine = np.interp(s_all[idx], near[ok], gv[ok])
            worst = max(worst, float(np.max(np.abs(mine - prof[idx]))))
    if worst > REPRO_TOL:
        raise common.Refused(f"C{cid} {phase}: repeated grid math differs from the module profile by {worst:.3g} N m/deg")
    # a phase shorter than one grid step holds no grid point: take the nearest
    inside = [g for g in near if s0 <= g <= s1 and vals[g]] or         [min((g for g in near if vals[g]), key=lambda g: abs(g - 0.5 * (s0 + s1)))]
    g = min(inside, key=lambda g: abs(vals[g]["slope"] - stored))
    v = vals[g]
    laps = {l["lap_number"]: int(((v["t"] >= l["start_time"]) & (v["t"] <= l["end_time"])).sum())
            for l in session["parsed"]["laps"]}
    per_lap = {s["lap_number"]: s["phases"][phase]["stability_observed_Nm_per_deg"]["median"]
               for s in res["summaries"] if s["stable_corner_id"] == cid}
    return {"cid": cid, "phase": phase, "stored": stored, "grid_s": float(g), "slope": v["slope"],
            "phase_s": (s0, s1), "repro_max_diff": worst, "laps_in_window": {k: n for k, n in laps.items() if n},
            "per_lap_medians": per_lap, "n_window": int(v["partial"].size), "v": v}


def build(outing, unstable, stable=None, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.corner_aggregation import PHASE_KEYS, aggregate_by_corner
    session = common.load_session(outing)
    res = common.run_chain(session)
    marks = common.gate(outing, res["fit_manifest"], res["fallback_used"], allow_status, allow_excluded)
    agg = aggregate_by_corner(res["summaries"])
    work, cols, se = _inputs(session, res)
    if stable is None:
        # rule: the corner phase whose cross-lap median is closest to the
        # session median of all corner-phase values
        vals = [(a["phases"][ph]["stability_observed_Nm_per_deg"]["median"], cid, ph)
                for cid, a in agg.items() for ph in PHASE_KEYS]
        vals = [v for v in vals if v[0] == v[0]]
        med = float(np.median([v[0] for v in vals]))
        stable = min(vals, key=lambda v: abs(v[0] - med))[1:]
    panels = {"stable": phase_panel(session, res, work, cols, se, agg, *stable),
              "unstable": phase_panel(session, res, work, cols, se, agg, *unstable)}
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    data_dir, _plots, _prefix = common.out_dirs(out)
    extra = {f"{k}_phase": (f"C{p['cid']} {common.PHASE_LABELS[p['phase']]}; stored {p['stored']:.3f}; grid point "
                            f"s {p['grid_s']:.1f} m slope {p['slope']:.3f}; beta mean {p['v']['beta_mean_deg']:.4f} deg; "
                            f"window {p['n_window']} samples")
             for k, p in panels.items()}
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label,
                    regressors=" + ".join(cols) + " (standardised; ridge stability_estimation.yaw_stability_ridge "
                                                  f"{se['yaw_stability_ridge']:g})",
                    window=f"+/- {se['yaw_stability_window_m']:g} m (yaw_stability_window_m), Gaussian sigma = window / "
                           f"2.5, grid {se['yaw_stability_grid_step_m']:g} m, all analysis laps pooled",
                    response="Mz = Iz x yaw acceleration (centred mean over yaw_stability_accel_window_s)", **extra)
    for k, p in panels.items():
        v = p["v"]
        for part, mask in (("inner", v["inner"]), ("outer", ~v["inner"])):
            idx = np.flatnonzero(mask)
            keep = idx[common.thin_stratified(v["beta_deg"][idx], v["partial"][idx])]
            common.write_csv(f"{data_dir}/{SCRIPT}_{k}_{part}.csv",
                             {"beta_deg": v["beta_deg"][keep], "mz_partial_Nm": v["partial"][keep]},
                             dict(m, panel=k, part=part, n_total=int(idx.size), n_written=int(keep.size)))
    render(out)
    return {k: {kk: vv for kk, vv in p.items() if kk != "v"} for k, p in panels.items()}


def render(out=None):
    import re
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, _c = common.read_csv(f"{data_dir}/{SCRIPT}_stable_inner.csv")
    t = ["% generated by tools/thesis_figures/fig_dmz_reading.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    info = {}
    for k in ("stable", "unstable"):
        g = re.match(r"(C\d+) (.+?); stored (\S+); grid point s \S+ m slope (\S+); beta mean (\S+) deg",
                     meta[f"{k}_phase"])
        info[k] = (g.group(1), g.group(2), float(g.group(3)), float(g.group(4)), float(g.group(5)))
        for part in ("inner", "outer"):
            t.append(common.table_read(prefix, f"{SCRIPT}_{k}_{part}.csv", f"tbl{k.capitalize()}{part.capitalize()}"))
    t.append("\\begin{groupplot}[group style={group size=2 by 1, horizontal sep=1.9cm},\n"
             f"  thesis axis, scale only axis, width={PANEL_WIDTH}, height={PANEL_HEIGHT},\n"
             "  xlabel={$\\beta$ in deg}, ylabel={$M_z$ partial in N\\,m},\n"
             "  scaled y ticks=base 10:-3, ytick scale label code/.code={$\\cdot 10^{3}$}]\n")
    for n, k in enumerate(("stable", "unstable")):
        cid, phase, stored, slope, bm = info[k]
        _m, inner = common.read_csv(f"{data_dir}/{SCRIPT}_{k}_inner.csv")
        _m, outer = common.read_csv(f"{data_dir}/{SCRIPT}_{k}_outer.csv")
        b = np.array([common.num(x) for x in inner["beta_deg"] + outer["beta_deg"]])
        b0, b1 = float(np.nanmin(b)), float(np.nanmax(b))
        opts = f"title={{{cid}, {phase}}}"
        if n == 1:
            opts += (", legend to name=legDmz, legend style={cells={anchor=west}, legend columns=3, "
                     "/tikz/every even column/.append style={column sep=0.6em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        t.append(f"\\addplot[only marks, mark=*, mark size=0.5pt, auxGray!45, forget plot] "
                 f"table[x=beta_deg, y=mz_partial_Nm]{{\\tbl{k.capitalize()}Outer}};\n")
        t.append(f"\\addplot[only marks, mark=*, mark size=0.6pt, auxGray!100!black, forget plot] "
                 f"table[x=beta_deg, y=mz_partial_Nm]{{\\tbl{k.capitalize()}Inner}};\n")
        t.append(f"\\addplot[black, very thick, forget plot] coordinates {{({b0:.4f},{slope * (b0 - bm):.2f}) "
                 f"({b1:.4f},{slope * (b1 - bm):.2f})}};\n")
        t.append(f"\\node[anchor=north west, align=left, font=\\small, fill=white, inner sep=2pt] at "
                 f"(rel axis cs:0.03,0.97) {{$\\mathrm{{d}}M_z/\\mathrm{{d}}\\beta = {slope:.0f}$ N\\,m/deg}};\n")
        if n == 1:
            t.append("\\addlegendimage{only marks, mark=*, mark size=1.2pt, auxGray!100!black}"
                     "\\addlegendentry{within one $\\sigma$ of the grid point}\n"
                     "\\addlegendimage{only marks, mark=*, mark size=1.2pt, auxGray!45}\\addlegendentry{rest of the window}\n"
                     "\\addlegendimage{black, very thick}\\addlegendentry{ridge slope}\n")
    t.append("\\end{groupplot}\n")
    t.append("\\node[anchor=north, below=2pt] at ($(group c1r1.outer south)!0.5!(group c2r1.outer south)$) "
             "{\\pgfplotslegendfromname{legDmz}};\n\\end{tikzpicture}\n")
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", "".join(t))


def _pick(text):
    cid, phase = text.split(":")
    from tools.thesis_figures.common import PHASE_LABELS
    if phase not in PHASE_LABELS:
        raise argparse.ArgumentTypeError(f"phase {phase!r}: one of {', '.join(PHASE_LABELS)}")
    return int(cid), phase


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--unstable", type=_pick, help="corner:phase_key")
    ap.add_argument("--stable", type=_pick, help="corner:phase_key; default: closest to the session median")
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if None in (a.outing, a.unstable):
        ap.error("--outing and --unstable are required unless --render-only")
    print(build(a.outing, a.unstable, a.stable, a.tag, a.allow_status, a.allow_excluded, a.out))


if __name__ == "__main__":
    main()
