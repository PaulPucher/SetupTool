# Q-UTIL (author EXECUTE 2026-10-21, read-only): are utilisation measures a
# usable companion or alternative to the CS ratio? Shipped default chain
# (fit L, schema 17), nothing changed in modules or config.
#
# Definitions after the chair performance_analysis tooling (internal),
# mue_effective and lateral_slip_efficiency_norm; adaptation: measured axle
# Fz, fit-L mu_demo and K, final-chain beta. Inputs are the CS path's own
# 2 Hz zero-phase signals: Fy_*_filt (body frame, as the CS regressand),
# the fit-L axle Fz and Eq. 9 axle Fx, alpha_*_filt from the shipped beta.
#   u      = (|Fy| / Fz) / mu_demo                lateral utilisation
#   u_comb = sqrt(Fy^2 + Fx^2) / Fz / mu_demo     combined utilisation
#   s      = (|Fy| / Fz) / (|alpha| K), |alpha| >= 0.01 rad (chair limit),
#            K = B C mu of the axle's fit L       secant ratio (1 = linear)
#   mu_eff = m sqrt(ax^2 + ay^2) / (Fz_f + Fz_r)  chair definition
# Caveat: the front mu_demo and K were fitted on the tyre-frame front force
# (Goblirsch Eq. 13); u_f and s_f use the body-frame Fy_f the CS reads.
#
# Population, as for CS: moving, analysis laps, inside a phase of a canonical
# corner (apex_3 = the apex region the verdict reads). CS counts where it
# is verdict-read (finite, not kerb-affected, not observer-reset-affected);
# u counts outside the kerb mask and smear zone (the sample-wise analogue of
# kerb-affected, u has no window); s additionally needs |alpha| >= 0.01.
# Output diagnostics/plots/utilisation/ (cache/, utilisation.txt, PNG).
# Run: python -m diagnostics.inspect_utilisation [report]

import copy
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT_DIR = os.path.join(ROOT, "diagnostics", "plots", "utilisation")
CACHE = os.path.join(OUT_DIR, "cache")
SESSIONS = ("nbr_o29_jst", "nbr_o54_cdr", "nbr_o10_jdr", "pr_o25_us_v1")
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
ALPHA_MIN = 0.01   # chair lateral_slip_efficiency: |alpha| >= 0.01 rad
NBR54_CORNER, NBR54_PHASE, NBR54_LAPS = 3, "entry_2_turnin", range(215, 221)
WORKERS = 3


def compute(key):
    import modules.tyre_fit_auto as tfa
    from diagnostics.inspect_data_quality import CANDIDATES
    from tests.conftest import resolve_data_path
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, summarise_corners,
                                            _butterworth_lowpass)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.limit_events import detect_floor_events
    _, track, outing, label, src = {c[0]: c for c in CANDIDATES}[key]
    path = resolve_data_path(src[1])
    data = parse_csv(path)
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    p = copy.deepcopy(apply_resolved_vehicle(load_parameters(),
                                             resolve_accuracy(load_parameters(), setup_data=None, cap=None)))
    st = prepare_vehicle_state(data["channels"], p)
    beta, man, gate, fb, fr, cs_in = tfa.resolve_sideslip_beta_and_cs_inputs(
        st, p, data, p["stability_estimation"]["sideslip_source"], csv_path=path)
    laps = data.get("laps", [])
    slip = estimate_slip_angles(st, beta, p)
    forces = estimate_lateral_forces(st, p)
    cs = estimate_cornering_stiffness(slip, forces, st, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(st, beta, p, laps)
    summ = summarise_corners(data.get("corners", []), cs, stab, st, lap_filter=None)
    loads, fx = cs_in["cs_kwargs"]["axle_loads"], cs_in["axle_fx"]
    events = detect_floor_events(cs, st, data.get("corners", []), laps, p, slip,
                                 axle_loads=loads, axle_fx=fx)["events"]
    sr = st["sample_rate_hz"]
    cut = p["stability_estimation"]["cs_filter_cutoff_hz"]
    ax_f = _butterworth_lowpass(st["ax_mps2"], cut, sr)
    ay_f = _butterworth_lowpass(st["ay_mps2"], cut, sr)
    out = {"key": key, "label": label, "status": man.get("status"), "fallback": fb,
           "mass_kg": p["vehicle"]["mass_kg"], "fit": {}, "laps": laps, "corners": data.get("corners", []), "summaries": summ,
           "events": events, "classification": p["classification"]}
    for ax, name in (("f", "front"), ("r", "rear")):
        a = man["axles"][name]
        out["fit"][ax] = {"mu": a["mu"], "B": a["B"], "C": a["C"], "K": a["B"] * a["C"] * a["mu"]}
    f32 = lambda x: np.asarray(x, dtype=np.float64)
    out["arr"] = {
        "t": f32(st["time"]), "s_m": f32(st["s_m"]), "moving": np.asarray(st["moving_mask"], bool),
        "kerb": np.asarray(st["kerb_mask"], bool), "smear": np.asarray(st["kerb_smear_zone"], bool),
        "Fy_f": f32(forces["Fy_f_filt"]), "Fy_r": f32(forces["Fy_r_filt"]),
        "Fz_f": f32(loads["fz_f_N"]), "Fz_r": f32(loads["fz_r_N"]),
        "Fx_f": f32(fx["fx_f_N"]), "Fx_r": f32(fx["fx_r_N"]),
        "alpha_f": f32(slip["alpha_f_filt"]), "alpha_r": f32(slip["alpha_r_filt"]),
        "ax": f32(ax_f), "ay": f32(ay_f),
        "CS_f": f32(cs["CS_ratio_f"]), "CS_r": f32(cs["CS_ratio_r"]),
        "ns_f": np.asarray(cs["near_strike_f"], bool), "ns_r": np.asarray(cs["near_strike_r"], bool),
        "ra_f": np.asarray(cs.get("observer_reset_affected_f", np.zeros(st["time"].size, bool)), bool),
        "ra_r": np.asarray(cs.get("observer_reset_affected_r", np.zeros(st["time"].size, bool)), bool),
    }
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] status {out['status']}, fallback {fb}", flush=True)
    return key


def derive(r):
    """Per-sample u, u_comb, s, mu_eff and the masks of the CS population."""
    A = r["arr"]
    d = {}
    with np.errstate(invalid="ignore", divide="ignore"):
        for ax in "fr":
            fit = r["fit"][ax]
            fy, fz, fx, al = A[f"Fy_{ax}"], A[f"Fz_{ax}"], A[f"Fx_{ax}"], A[f"alpha_{ax}"]
            ok = np.isfinite(fz) & (fz > 0)
            d[f"u_{ax}"] = np.where(ok, np.abs(fy) / fz / fit["mu"], np.nan)
            d[f"uc_{ax}"] = np.where(ok, np.hypot(fy, fx) / fz / fit["mu"], np.nan)
            d[f"s_{ax}"] = np.where(ok & (np.abs(al) >= ALPHA_MIN), np.abs(fy) / fz / (np.abs(al) * fit["K"]), np.nan)
            # chair keeps the sign (Fy / alpha); opposite sign = force against slip
            d[f"sgn_{ax}"] = np.sign(fy) != np.sign(al)
        d["mu_eff"] = r["mass_kg"] * np.hypot(A["ax"], A["ay"]) / (A["Fz_f"] + A["Fz_r"])
    t = A["t"]
    valid = {l["lap_number"] for l in r["laps"] if l.get("is_valid_for_analysis")}
    lap_t = {l["lap_number"]: (l["start_time"], l["end_time"]) for l in r["laps"]}
    phase_mask = {ph: np.zeros(t.size, bool) for ph in PHASES}
    inst = []   # (cid, lap, {phase: index array})
    for c in r["corners"]:
        cid, ln = c.get("stable_corner_id"), c.get("lap_number")
        if cid is None or ln not in valid:
            continue
        idx = {}
        for ph in PHASES:
            if ph == "apex_3":
                apex_s = c.get("apex_lap_distance_m")
                segs = [sg for sg in c["segments"].values() if sg[1] >= sg[0]]
                if apex_s is None or apex_s != apex_s or not segs:
                    idx[ph] = np.array([], int)
                    continue
                lo = int(np.searchsorted(t, min(sg[0] for sg in segs), side="left"))
                hi = int(np.searchsorted(t, max(sg[1] for sg in segs), side="right"))
                w = np.abs(A["s_m"][lo:hi] - apex_s) <= 25.0
                idx[ph] = np.flatnonzero(w) + lo
            else:
                a, b = c["segments"][ph]
                idx[ph] = np.flatnonzero((t >= a) & (t <= b)) if b >= a else np.array([], int)
            phase_mask[ph][idx[ph]] = True
        inst.append((int(cid), int(ln), idx))
    base = A["moving"].copy()
    clean_u = base & ~A["kerb"] & ~A["smear"]
    m = {"base": base}
    for ax in "fr":
        m[f"cs_{ax}"] = base & np.isfinite(A[f"CS_{ax}"]) & ~A[f"ns_{ax}"] & ~A[f"ra_{ax}"]
        m[f"u_{ax}"] = clean_u & np.isfinite(d[f"u_{ax}"])
        m[f"s_{ax}"] = clean_u & np.isfinite(d[f"s_{ax}"])
    return d, m, phase_mask, inst, valid, lap_t


def _pct(x, q):
    return float(np.percentile(x, q)) if x.size else float("nan")


def _spearman(a, b):
    from scipy.stats import spearmanr
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 10:
        return float("nan"), int(ok.sum())
    return float(spearmanr(a[ok], b[ok])[0]), int(ok.sum())


def _verdict(agg, cls):
    # mirror of ui/views/outing_form.py _classify_corner (axle tiers, worst
    # phase, apex_3 from the apex region; precedence rear STRONG, front
    # STRONG, rear MODERATE, front MODERATE)
    wf, wr = 1.0, 1.0
    for ph in agg["phases"]:
        src = agg["apex_region"] if ph == "apex_3" else agg["phases"][ph]
        f, r = src["cs_ratio_f"]["median"], src["cs_ratio_r"]["median"]
        if f == f and f < wf:
            wf = f
        if r == r and r < wr:
            wr = r
    sf, sr_ = cls["STRONG_CSF"]["value"], cls["STRONG_CSR"]["value"]
    mf, mr = cls["MODERATE_CSF"]["value"], cls["MODERATE_CSR"]["value"]
    if wr < sr_:
        return "oversteer"
    if wf < sf:
        return "understeer"
    if wr < mr:
        return "oversteer"
    if wf < mf:
        return "understeer"
    return "none"


def report():
    from modules.recommendation import aggregate_by_corner
    res = {k: pickle.load(open(os.path.join(CACHE, f"{k}.pkl"), "rb")) for k in SESSIONS}
    L = ["Q-UTIL -- utilisation and secant ratio next to the CS ratio (fit-L default, schema 17)", ""]
    for k in SESSIONS:
        r = res[k]
        d, m, pm, inst, valid, lap_t = derive(r)
        A = r["arr"]
        ff, fr_ = r["fit"]["f"], r["fit"]["r"]
        L.append("=" * 100)
        L.append(f"{r['label']}: status {r['status']}, fallback {r['fallback']}; mu_demo front {ff['mu']:.3f} / rear "
                 f"{fr_['mu']:.3f}, K front {ff['K']:.2f} / rear {fr_['K']:.2f} per rad")
        # 1 coverage per phase
        L.append("1. coverage (share of moving analysis-lap phase samples): CS verdict-read / u / s, front | rear")
        for ph in PHASES:
            b = m["base"] & pm[ph]
            n = max(1, b.sum())
            parts = [f"{(m[f'cs_{ax}'] & b).sum() / n * 100:.0f} / {(m[f'u_{ax}'] & b).sum() / n * 100:.0f} / "
                     f"{(m[f's_{ax}'] & b).sum() / n * 100:.0f} %" for ax in "fr"]
            L.append(f"   {ph:15s} n {b.sum():6d}: front {parts[0]} | rear {parts[1]}")
        # 2 apex per corner
        L.append("2. apex region per corner: median over laps of the lap medians u_f / u_r, s_f / s_r; leader "
                 "(higher u = closer to its limit; lower s = further below linear); laps with the same leader")
        apex = {}
        for cid, ln, idx in inst:
            ix = idx["apex_3"]
            rec = {}
            for ax in "fr":
                iu = ix[m[f"u_{ax}"][ix]]
                is_ = ix[m[f"s_{ax}"][ix]]
                rec[f"u_{ax}"] = float(np.median(d[f"u_{ax}"][iu])) if iu.size else np.nan
                rec[f"s_{ax}"] = float(np.median(d[f"s_{ax}"][is_])) if is_.size else np.nan
            apex.setdefault(cid, []).append(rec)
        u_lead = {}
        for cid in sorted(apex):
            recs = apex[cid]
            med = {q: float(np.nanmedian([x[q] for x in recs])) if any(x[q] == x[q] for x in recs) else np.nan
                   for q in ("u_f", "u_r", "s_f", "s_r")}
            lu = "front" if med["u_f"] > med["u_r"] else "rear" if med["u_r"] > med["u_f"] else "-"
            ls = "front" if med["s_f"] < med["s_r"] else "rear" if med["s_r"] < med["s_f"] else "-"
            nu = sum(1 for x in recs if (x["u_f"] > x["u_r"]) == (lu == "front") and x["u_f"] == x["u_f"]
                     and x["u_r"] == x["u_r"])
            ns = sum(1 for x in recs if (x["s_f"] < x["s_r"]) == (ls == "front") and x["s_f"] == x["s_f"]
                     and x["s_r"] == x["s_r"])
            u_lead[cid] = lu
            L.append(f"   C{cid:<3d} u {med['u_f']:.2f} / {med['u_r']:.2f} -> {lu:5s} ({nu}/{len(recs)} laps);  "
                     f"s {med['s_f']:.2f} / {med['s_r']:.2f} -> {ls:5s} ({ns}/{len(recs)} laps)")
        lead_u = [v for v in u_lead.values() if v != "-"]
        L.append(f"   u leader count: front {lead_u.count('front')}, rear {lead_u.count('rear')} of {len(u_lead)} corners")
        # 3 distributions
        L.append("3. distribution over the clean phase population: p50 / p95 / p99, share > 1")
        allph = np.zeros(A["t"].size, bool)
        for ph in PHASES:
            allph |= pm[ph]
        for ax, name in (("f", "front"), ("r", "rear")):
            for q, lab in (("u", "u"), ("uc", "u_comb")):
                x = d[f"{q}_{ax}"][m[f"u_{ax}"] & allph]
                L.append(f"   {name:5s} {lab:6s}: {_pct(x, 50):.3f} / {_pct(x, 95):.3f} / {_pct(x, 99):.3f}, "
                         f"> 1: {np.mean(x > 1) * 100:.2f} % (n {x.size})")
        me = d["mu_eff"][m["base"] & ~A["kerb"] & ~A["smear"] & allph]
        me = me[np.isfinite(me)]
        L.append(f"   mu_eff (vehicle): {_pct(me, 50):.3f} / {_pct(me, 95):.3f} / {_pct(me, 99):.3f}; "
                 f"against mu_demo front {ff['mu']:.3f} / rear {fr_['mu']:.3f}")
        for ax, name in (("f", "front"), ("r", "rear")):
            sel = m[f"s_{ax}"] & allph
            L.append(f"   {name} s: p50 {_pct(d[f's_{ax}'][sel], 50):.3f}, Fy against alpha in sign on "
                     f"{np.mean(d[f'sgn_{ax}'][sel]) * 100:.1f} % of s samples (|.| hides it)")
        # 4 correlation
        L.append("4. Spearman rank correlation where CS is verdict-read: samples (autocorrelated, n inflated) | "
                 "corner-lap-phase medians")
        for ax, name in (("f", "front"), ("r", "rear")):
            sel = m[f"cs_{ax}"] & m[f"u_{ax}"] & allph
            rs, ns = _spearman(A[f"CS_{ax}"][sel], d[f"s_{ax}"][sel])
            ru, nu = _spearman(A[f"CS_{ax}"][sel], d[f"u_{ax}"][sel])
            cm, sm_, um = [], [], []
            for cid, ln, idx in inst:
                for ph in PHASES:
                    ix = idx[ph]
                    a = ix[(m[f"cs_{ax}"] & m[f"u_{ax}"])[ix]]
                    if a.size >= 5:
                        cm.append(np.median(A[f"CS_{ax}"][a]))
                        um.append(np.median(d[f"u_{ax}"][a]))
                        sv = d[f"s_{ax}"][a]
                        sm_.append(np.nanmedian(sv) if np.isfinite(sv).any() else np.nan)
            cm, sm_, um = np.array(cm), np.array(sm_), np.array(um)
            rs2, ns2 = _spearman(cm, sm_)
            ru2, nu2 = _spearman(cm, um)
            L.append(f"   {name}: CS vs s {rs:+.2f} (n {ns}) | {rs2:+.2f} (n {ns2});  CS vs u {ru:+.2f} (n {nu}) | "
                     f"{ru2:+.2f} (n {nu2})")
        # 5 NBR 54 C3
        if k == "nbr_o54_cdr":
            L.append(f"5. corner {NBR54_CORNER} turn-in, front, laps {NBR54_LAPS.start}-{NBR54_LAPS.stop - 1}: "
                     "phase median / max u, u_comb; phase min s; at the floor events: u, u_comb, s, CS min")
            ev = [e for e in r["events"] if e["corner"] == NBR54_CORNER and e["axle"] == "front"]
            t = A["t"]
            for cid, ln, idx in inst:
                if cid != NBR54_CORNER or ln not in NBR54_LAPS:
                    continue
                ix = idx[NBR54_PHASE]
                iu = ix[m["u_f"][ix]]
                is_ = ix[m["s_f"][ix]]
                line = (f"   lap {ln}: u {np.median(d['u_f'][iu]):.2f} / {d['u_f'][iu].max():.2f}, u_comb "
                        f"{np.median(d['uc_f'][iu]):.2f} / {d['uc_f'][iu].max():.2f}, s min "
                        f"{(d['s_f'][is_].min() if is_.size else np.nan):.2f}")
                L.append(line)
                for e in ev:
                    if e["lap"] != ln:
                        continue
                    sel = (t >= e["t_start"]) & (t <= e["t_end"])
                    sv = d["s_f"][sel]
                    L.append(f"      event {e['s_start_m']:.0f}-{e['s_end_m']:.0f} m ({e['phase']}), CS min "
                             f"{e['cs_min']:.2f}: u {np.nanmedian(d['u_f'][sel]):.2f}, u_comb "
                             f"{np.nanmedian(d['uc_f'][sel]):.2f}, s "
                             f"{(np.nanmedian(sv) if np.isfinite(sv).any() else np.nan):.2f}, |Fx|/Fz max "
                             f"{e['fx_util_max']:.2f}, kerb {e['kerb_affected']}")
        # 6 balance agreement
        summ = [s for s in r["summaries"] if s["lap_number"] in valid]
        agg = aggregate_by_corner(summ)
        cnt = {"agree": 0, "disagree": 0, "no_verdict": 0}
        detail = []
        for cid, a in agg.items():
            v = _verdict(a, r["classification"])
            lu = u_lead.get(int(cid))
            if lu in (None, "-"):
                continue
            if v == "none":
                cnt["no_verdict"] += 1
                continue
            ok = (v == "understeer") == (lu == "front")
            cnt["agree" if ok else "disagree"] += 1
            detail.append(f"C{cid} {v}/{lu}")
        L.append(f"6. apex u leader vs CS verdict (aggregate): agree {cnt['agree']}, disagree {cnt['disagree']}, "
                 f"corners without a CS verdict {cnt['no_verdict']}; " + ", ".join(detail))
        if k == "nbr_o29_jst":
            L.append(f"7. figure: {draw(r, d, m, pm, valid, lap_t)}")
    text = "\n".join(L)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "utilisation.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(text)


def draw(r, d, m, pm, valid, lap_t):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    A = r["arr"]
    t = A["t"]
    # one flying lap: the fastest analysis lap
    ln = min(valid, key=lambda n: lap_t[n][1] - lap_t[n][0])
    on = (t >= lap_t[ln][0]) & (t <= lap_t[ln][1])
    s = A["s_m"][on]
    ink, muted, grid = "#0b0b0b", "#52514e", "#e4e4e0"
    col = {"f": "#2a78d6", "r": "#eb6834"}
    shade = {"entry_1_brake": "#ececea", "entry_2_turnin": "#e2e2df", "apex_3": "#d6d6d2",
             "exit_4": "#e2e2df", "exit_5": "#ececea"}
    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True, facecolor="#fcfcfb")
    rows = (("CS", "cornering stiffness ratio (-)", (-0.6, 1.1)),
            ("s", "secant ratio s = (|Fy|/Fz) / (|alpha| K) (-)", (0.0, 1.4)),
            ("u", "lateral utilisation u = (|Fy|/Fz) / mu_demo (-)", (0.0, 1.2)))
    for axp, (q, ylab, ylim) in zip(axes, rows):
        axp.set_facecolor("#fcfcfb")
        for ph in PHASES:
            mk = pm[ph][on]
            for a, b in _runs(mk):
                axp.axvspan(s[a], s[b], color=shade[ph], lw=0, zorder=0)
        for ax, name in (("f", "front"), ("r", "rear")):
            if q == "CS":
                y = np.where(m[f"cs_{ax}"], A[f"CS_{ax}"], np.nan)[on]
            else:
                y = np.where(m[f"{q}_{ax}"], d[f"{q}_{ax}"], np.nan)[on]
            axp.plot(s, y, color=col[ax], lw=1.2, label=name, zorder=2)
        if q == "CS":
            axp.axhline(0.0, color=muted, lw=0.8, ls=":")
        else:
            axp.axhline(1.0, color=muted, lw=0.8, ls=":")
        axp.set_ylim(*ylim)
        axp.set_ylabel(ylab, color=ink, fontsize=9)
        axp.grid(axis="y", color=grid, lw=0.6)
        for sp in ("top", "right"):
            axp.spines[sp].set_visible(False)
    axes[0].legend(loc="lower left", ncol=2, fontsize=9, frameon=False)
    axes[-1].set_xlabel("lap distance (m); shading = corner phases (darkest = apex region)", color=ink)
    fig.suptitle(f"{r['label']}, lap {ln} (fastest analysis lap): cornering stiffness ratio, secant ratio and lateral "
                 "utilisation, front and rear axle; kerb-affected samples not drawn", color=ink, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(OUT_DIR, f"nbr29_lap{ln}_cs_secant_utilisation_front_rear.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def _runs(mask):
    mask = np.asarray(mask, bool)
    if not mask.any():
        return []
    e = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1) - 1))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report()
        return
    from diagnostics.inspect_fit_lit import keep_awake
    keep_awake(True)
    try:
        todo = [k for k in SESSIONS if not os.path.exists(os.path.join(CACHE, f"{k}.pkl"))]
        with ProcessPoolExecutor(max_workers=WORKERS) as ex:
            for _ in ex.map(compute, todo):
                pass
    finally:
        keep_awake(False)
    report()


if __name__ == "__main__":
    main()
