# fig:example_fit -- the session's fitted axle curves, front | rear: F_y / F_z
# over the slip angle with the fit population, the samples the braking
# filter removed, the fitted Magic Formula (C = 1, E = -1, D = mu), the
# origin slope K alpha and the demonstrated mu. Read-only partial re-run of
# fit L (common.capture_fit); one outing.
# Run: python -m tools.thesis_figures.fig_example_fit --outing 12 [--tag r1]
#      [--allow-status] [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "fig_example_fit"
CURVE_POINTS = 121   # per curve; keeps the figure near 3700 plotted points


def build(outing, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.tyre_model_pacejka import pacejka_lateral_force
    session = common.load_session(outing)
    got = common.capture_fit(session)
    info = got["load_in"]["info"]
    fits = dict(zip(("front", "rear"), got["fits"]))
    # the production D4-L gate on the same inputs; the partial run stops
    # before the sign check and the NIS sweep, so only a failed gate (or the
    # damper-path check) can mark the session here -- the stored payload,
    # where valid, carries the full status
    from modules import tyre_fit_auto as tfa
    cfg = session["params"]["tyre_fit_auto"]
    rows, status = {}, "ok"
    for axle, g in fits.items():
        pg = tfa.plausibility_gate_l(g["alpha"], got["fit_mask"], g["fit"], cfg)
        if pg["failed"]:
            status = "degenerate"
        rows[axle] = {"K": pg["stiffness_norm_per_rad"], "k_zone": pg["k_zone"], "mu_zone": pg["mu_zone"]}
    marks = common.gate(outing, {"axle_load": info, "status": status}, False, allow_status, allow_excluded)
    data_dir, plots_dir, prefix = common.out_dirs(out)
    # stable names: the chapter's \input never changes, the session goes
    # into the CSV header only
    base = SCRIPT
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    m = common.meta(SCRIPT, [outing], tag, marks, session_label=label, gate_status_partial=status,
                    note="partial fit-L re-run, stopped before the observer; front F_y in the tyre frame as fitted")

    params_cols = {k: [] for k in ("axle", "B", "C", "E", "mu", "K_per_rad", "fz_ref_N", "n_fit",
                                   "n_braking_removed", "k_zone", "mu_zone")}
    a_all = np.concatenate([np.degrees(g["alpha"][g["axle_mask"] & np.isfinite(g["alpha"])]) for g in fits.values()])
    a_grid_deg = np.linspace(np.nanpercentile(a_all, 0.5), np.nanpercentile(a_all, 99.5), CURVE_POINTS)
    curves = {"alpha_deg": a_grid_deg}
    for axle, g in fits.items():
        f, a, y = g["fit"], g["alpha"], g["Fy"] / g["Fz"]
        fit_pop = f["residual_mask"]                  # what _fit_axle_l actually fitted
        braking = got["fit_mask"] & ~g["axle_mask"] & np.isfinite(a) & np.isfinite(y)
        for name, mask in (("fit", fit_pop), ("braking", braking)):
            idx = np.flatnonzero(mask)
            keep = idx[common.thin_stratified(a[idx], y[idx])]
            common.write_csv(f"{data_dir}/{base}_{axle}_{name}.csv",
                             {"alpha_deg": np.degrees(a[keep]), "fy_fz": y[keep]},
                             dict(m, axle=axle, population=name, n_total=int(mask.sum()), n_written=int(keep.size)))
        a_rad = np.radians(a_grid_deg)               # K and the curve take alpha in rad
        curves[f"mf_{axle}_fy_fz"] = pacejka_lateral_force(a_rad, f["B"], f["C"], f["mu"], f["E"])
        line = rows[axle]["K"] * a_rad
        curves[f"lin_{axle}_fy_fz"] = np.where(np.abs(line) <= f["mu"], line, np.nan)
        for k, v in (("axle", axle), ("B", f["B"]), ("C", f["C"]), ("E", f["E"]), ("mu", f["mu"]),
                     ("K_per_rad", rows[axle]["K"]), ("fz_ref_N", f["fz_ref_N"]), ("n_fit", int(fit_pop.sum())),
                     ("n_braking_removed", int(braking.sum())), ("k_zone", rows[axle]["k_zone"]),
                     ("mu_zone", rows[axle]["mu_zone"])):
            params_cols[k].append(v)
    common.write_csv(f"{data_dir}/{base}_curves.csv", curves, m)
    common.write_csv(f"{data_dir}/{base}_params.csv", params_cols, m)

    tex = _tex(base, prefix, rows, fits, a_grid_deg, marks)
    common.write_tex(f"{plots_dir}/{base}.tex", tex)
    return {"fits": {ax: {"B": g["fit"]["B"], "mu": g["fit"]["mu"], "K": rows[ax]["K"],
                          "fit_n_samples": g["fit"]["fit_n_samples"]} for ax, g in fits.items()},
            "braking_excluded_n": {ax: int((got["fit_mask"] & ~g["axle_mask"]).sum()) for ax, g in fits.items()},
            "status_partial": status, "marks": marks, "files": base}


def _tex(base, prefix, rows, fits, a_grid_deg, marks):
    # one shared legend below the right panel; its front entries are drawn
    # there with \addlegendimage so both axles are explained once
    sym = {"front": "f", "rear": "r"}
    mark = {"front": "o", "rear": "square"}
    colour = {"front": "axleFront", "rear": "axleRear"}
    style = {"front": "axleFront, solid", "rear": "axleRear, dashed"}
    note = f" ({'; '.join(marks)})" if marks else ""
    t = ["% generated by tools/thesis_figures/fig_example_fit.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    for axle in ("front", "rear"):
        for name in ("fit", "braking"):
            t.append(common.table_read(prefix, f"{base}_{axle}_{name}.csv", f"tbl{axle.capitalize()}{name.capitalize()}"))
    t.append(common.table_read(prefix, f"{base}_curves.csv", "tblCurves"))
    t.append("\\begin{groupplot}[group style={group size=2 by 1, horizontal sep=1.3cm},\n"
             "  thesis axis, width=0.5\\textwidth, height=0.42\\textwidth, unbounded coords=jump,\n"
             f"  xmin={a_grid_deg[0]:.2f}, xmax={a_grid_deg[-1]:.2f},\n"
             "  xlabel={$\\alpha$ in deg}, ylabel={$F_y / F_z$}]\n")
    for axle in ("front", "rear"):
        f = fits[axle]["fit"]
        cap = axle.capitalize()
        opts = f"title={{{axle} axle{note}}}"
        if axle == "rear":
            opts += (", legend style={font=\\small, cells={anchor=west}, at={(1,-0.22)}, anchor=north east, "
                     "legend columns=3}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        if axle == "rear":
            # legend images for the front entries, drawn nowhere
            t.append("\\addlegendimage{only marks, mark=*, mark size=0.6pt, color=auxGray}\\addlegendentry{fit samples}\n")
            for ax in ("front", "rear"):
                i, ff = sym[ax], fits[ax]["fit"]
                t.append(f"\\addlegendimage{{only marks, mark={mark[ax]}, mark size=1.2pt, color={colour[ax]}!45}}"
                         f"\\addlegendentry{{braking filter, {ax}}}\n")
                t.append(f"\\addlegendimage{{{style[ax]}, thick}}\\addlegendentry{{fitted curve, {ax}}}\n")
                t.append(f"\\addlegendimage{{black, dotted, thick}}"
                         f"\\addlegendentry{{$K_\\mathrm{{{i}}} = {rows[ax]['K']:.1f}$\\,1/rad}}\n")
                t.append(f"\\addlegendimage{{auxGray, dashdotted}}\\addlegendentry{{$\\mu_\\mathrm{{{i}}} = {ff['mu']:.3f}$}}\n")
        # braking samples first and small, so the fit population stays readable
        t.append(f"\\addplot[only marks, mark={mark[axle]}, mark size=0.4pt, color={colour[axle]}!45, forget plot] "
                 f"table[x=alpha_deg, y=fy_fz]{{\\tbl{cap}Braking}};\n")
        t.append(f"\\addplot[only marks, mark=*, mark size=0.25pt, color=auxGray, forget plot] "
                 f"table[x=alpha_deg, y=fy_fz]{{\\tbl{cap}Fit}};\n")
        t.append(f"\\addplot[{style[axle]}, thick, forget plot] table[x=alpha_deg, y=mf_{axle}_fy_fz]{{\\tblCurves}};\n")
        t.append(f"\\addplot[black, dotted, thick, forget plot] table[x=alpha_deg, y=lin_{axle}_fy_fz]{{\\tblCurves}};\n")
        for sgn in (1, -1):
            t.append(f"\\addplot[auxGray, dashdotted, forget plot] coordinates "
                     f"{{({a_grid_deg[0]:.2f},{sgn * f['mu']:.4f}) ({a_grid_deg[-1]:.2f},{sgn * f['mu']:.4f})}};\n")
    t.append("\\end{groupplot}\n\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int, required=True)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    res = build(a.outing, a.tag, a.allow_status, a.allow_excluded, a.out)
    print(res)


if __name__ == "__main__":
    main()
