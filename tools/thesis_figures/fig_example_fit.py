# fig:example_fit -- the session's fitted axle curves, front | rear: F_y / F_z
# over the slip angle with the fit population, the samples the braking
# filter removed, the fitted Magic Formula (C = 1, E = -1, D = mu), the
# origin slope K alpha and the demonstrated mu. Read-only partial re-run of
# fit L (common.capture_fit); one outing. Deliberate exception to the
# rear = dashed rule of the other figures: here each axle has its own panel,
# so both fitted curves are solid (axle colours unchanged).
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
    # the production degenerate rules on the same fits: Powell converged and
    # D > 0 on both axles (as fit_session_pacejka), then the fit-L
    # plausibility gate. The partial run stops before the NIS sweep, so the
    # stored payload, where valid, carries the full status
    from modules import tyre_fit_auto as tfa
    cfg = session["params"]["tyre_fit_auto"]
    rows, status = {}, "ok"
    for axle, g in fits.items():
        pg = tfa.plausibility_gate_l(g["alpha"], got["fit_mask"], g["fit"], cfg)
        if pg["failed"] or not (g["fit"]["sign_ok"] and g["fit"]["powell_converged"]):
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
                    note="partial fit-L re-run, stopped before the observer; front F_y in the tire frame as fitted")

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

    render(out)
    return {"fits": {ax: {"B": g["fit"]["B"], "mu": g["fit"]["mu"], "K": rows[ax]["K"],
                          "fit_n_samples": g["fit"]["fit_n_samples"]} for ax, g in fits.items()},
            "braking_excluded_n": {ax: int((got["fit_mask"] & ~g["axle_mask"]).sum()) for ax, g in fits.items()},
            "status_partial": status, "marks": marks, "files": base}


def render(out=None):
    """The .tex from the stored CSVs only (no session is loaded): mu per
    axle from the params CSV, the alpha range from the curve grid."""
    data_dir, plots_dir, prefix = common.out_dirs(out)
    meta, params = common.read_csv(f"{data_dir}/{SCRIPT}_params.csv")
    _m, curves = common.read_csv(f"{data_dir}/{SCRIPT}_curves.csv")
    mu = {ax: common.num(m) for ax, m in zip(params["axle"], params["mu"])}
    a = [common.num(x) for x in curves["alpha_deg"]]
    marks = [m for m in (meta.get("marks") or "").split("; ") if m]
    common.write_tex(f"{plots_dir}/{SCRIPT}.tex", _tex(SCRIPT, prefix, mu, a[0], a[-1], marks))


# front = axle colour, circles; rear = axle colour, squares; both curves
# solid (one axle per panel, see the header). Samples are drawn light so
# the fitted curve, drawn last, stands out; braking-filtered samples
# lightest, hollow, behind everything.
MARK = {"front": "*", "rear": "square*"}
HOLLOW = {"front": "o", "rear": "square"}
COLOUR = {"front": "axleFront", "rear": "axleRear"}
CURVE = {"front": "axleFront, solid", "rear": "axleRear, solid"}
FIT_TINT = 55            # % of the axle colour for the fit samples
BRAKING_STYLE = "auxGray!40"


def _tex(base, prefix, mu, a_lo, a_hi, marks):
    # one neutral legend in two rows below both panels (legend to name);
    # the panel titles name the axle, the caption carries K and mu
    note = f" ({'; '.join(marks)})" if marks else ""
    t = ["% generated by tools/thesis_figures/fig_example_fit.py -- do not edit; re-run the script\n",
         "\\begin{tikzpicture}\n"]
    for axle in ("front", "rear"):
        for name in ("fit", "braking"):
            t.append(common.table_read(prefix, f"{base}_{axle}_{name}.csv", f"tbl{axle.capitalize()}{name.capitalize()}"))
    t.append(common.table_read(prefix, f"{base}_curves.csv", "tblCurves"))
    t.append("\\begin{groupplot}[group style={group size=2 by 1, horizontal sep=1.3cm},\n"
             "  thesis axis, width=0.5\\textwidth, height=0.42\\textwidth, unbounded coords=jump,\n"
             f"  xmin={a_lo:.2f}, xmax={a_hi:.2f},\n"
             "  xlabel={$\\alpha$ in deg}, ylabel={$F_y / F_z$}]\n")
    for axle in ("front", "rear"):
        cap = axle.capitalize()
        opts = f"title={{{axle} axle{note}}}"
        if axle == "rear":
            opts += (", legend to name=legExampleFit, legend style={cells={anchor=west}, legend columns=3, "
                     "/tikz/every even column/.append style={column sep=0.6em}}")
        t.append(f"\\nextgroupplot[{opts}]\n")
        if axle == "rear":
            t.append("\\addlegendimage{only marks, mark=*, mark size=1.2pt, black}\\addlegendentry{fit samples}\n"
                     f"\\addlegendimage{{only marks, mark=o, mark size=1.2pt, {BRAKING_STYLE}}}"
                     "\\addlegendentry{removed by braking filter}\n"
                     "\\addlegendimage{black, solid, very thick}\\addlegendentry{fitted curve}\n"
                     "\\addlegendimage{black, dotted, thick}\\addlegendentry{linear slope $K\\,\\alpha$}\n"
                     "\\addlegendimage{auxGray, dashdotted}\\addlegendentry{demonstrated $\\mu$}\n")
        # draw order: braking samples, fit samples, reference lines, fitted curve on top
        t.append(f"\\addplot[only marks, mark={HOLLOW[axle]}, mark size=0.4pt, color={BRAKING_STYLE}, forget plot] "
                 f"table[x=alpha_deg, y=fy_fz]{{\\tbl{cap}Braking}};\n")
        t.append(f"\\addplot[only marks, mark={MARK[axle]}, mark size=0.2pt, color={COLOUR[axle]}!{FIT_TINT}, "
                 f"forget plot] table[x=alpha_deg, y=fy_fz]{{\\tbl{cap}Fit}};\n")
        for sgn in (1, -1):
            t.append(f"\\addplot[auxGray, dashdotted, forget plot] coordinates "
                     f"{{({a_lo:.2f},{sgn * mu[axle]:.4f}) ({a_hi:.2f},{sgn * mu[axle]:.4f})}};\n")
        t.append(f"\\addplot[black, dotted, thick, forget plot] table[x=alpha_deg, y=lin_{axle}_fy_fz]{{\\tblCurves}};\n")
        t.append(f"\\addplot[{CURVE[axle]}, very thick, forget plot] table[x=alpha_deg, y=mf_{axle}_fy_fz]{{\\tblCurves}};\n")
    t.append("\\end{groupplot}\n")
    # legend centred below both panels, under their x labels
    t.append("\\path (group c1r1.outer south west) -- (group c2r1.outer south east) "
             "node[midway, below=2pt] {\\pgfplotslegendfromname{legExampleFit}};\n")
    t.append("\\end{tikzpicture}\n")
    return "".join(t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSVs only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if a.outing is None:
        ap.error("--outing is required unless --render-only")
    res = build(a.outing, a.tag, a.allow_status, a.allow_excluded, a.out)
    print(res)


if __name__ == "__main__":
    main()
