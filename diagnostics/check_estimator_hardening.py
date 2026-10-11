# Offline validation of the estimator hardening, phase 1 (author work order
# 2026-10-07, PRODUCTION PACKAGE D3 + D4). Numbers only; the verdict is the
# author's.
#
# D3: sideslip_ekf_pacejka re-enters after a divergence without the kinematic
#     beta: NIS-window divergence keeps the state and resets P to P0; hard-bound
#     divergence resets beta to 0, yaw rate to the measurement, P to P0 (D3
#     amendment 2026-10-07). Before, both wrote the kinematic beta into the
#     state and the output. A hard-bound divergence is a diverged sample whose
#     raw updated beta exceeds the bound -- read the same way from old and new
#     output. OLD = the HEAD version of the module, copied
#     to diagnostics/output/estimator_hardening/sideslip_ekf_pacejka_old.py
#     and swapped in for the fit's EKF calls inside this process only.
# D4: tyre_fit_auto.plausibility_gate, dry-run on every round-3 fit (seeds A,
#     D2, B2; the fitted curve depends on the seed only, so routes (a) and (b)
#     share one fit). The fits are recomputed with the production axle fit on
#     the round-3 seeds (built as in seed_sensitivity_fit.run_export).
#
# Same two exports, straights, corners and splits as rounds 2 and 3.
#
# --step p2s1 (phase 2 step 1, author work order 2026-10-08): Dubai under the
# new re-entry (per-corner divergence counts and phase means, the golden's
# corner 0 walkthrough) and the acceptance-threshold facts (sign check, R
# sweep band) on Dubai, NBR and PR, old vs new. Dubai runs at accuracy cap 1
# exactly as tests/conftest.py builds the golden; NBR and PR as rounds 2-3.
# --step p2s1b (2026-10-11): Paul Ricard with the R sweep pinned to one grid
# point, old and new estimator, to separate the R choice from the re-entry.
# --step p2s2grid (2026-10-11): full 25-point R grid, new estimator, Dubai /
# NBR / PR, NIS exceedance pair and sign check per grid point (band
# re-derivation input).
# --step p2s2verify (2026-10-11): the redesigned R selection, trace per export.
# --step p2s2c (2026-10-11): stability surge mechanism -- three worked corners
# (Dubai R-sensitive, Dubai negative at both picks, NBR JST 21) and the count of
# re-entries inside the worst phase of every newly negative pool corner.
# --step p2s2robust --key <session> / p2s2robust_report (2026-10-11): negative
# worst-phase stability at the ruled pick vs at the fallback R 0.1/16 per pool
# session (R-robust / R-sensitive split) and the acceptance metrics at the pick.
# Run: python -m diagnostics.check_estimator_hardening [--png] [--step p1|p2s1|p2s1b|p2s2grid|p2s2verify|p2s2c|
#      p2s2robust|p2s2robust_report] [--key <session>]

import argparse
import contextlib
import importlib.util
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_kinematic_inputs import (  # noqa: E402
    OUT_DIR, load, quasi_steady_segments, steering_straights, track_label,
)
from diagnostics.compare_beta_architectures import (  # noqa: E402
    B2_CUTOFF_HZ, EDGE_S, EXPORTS, five_corners, offsets,
)

OLD_EKF = os.path.join("diagnostics", "output", "estimator_hardening", "sideslip_ekf_pacejka_old.py")


def old_ekf_function():
    spec = importlib.util.spec_from_file_location("sideslip_ekf_pacejka_old", OLD_EKF)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.estimate_sideslip_ekf_pacejka


MID_CORNER_AY_MPS2 = 5.0   # amendment metric 2: a hard-bound reset at |ay| >= this is a mid-corner zero reset


@contextlib.contextmanager
def ekf_variant(fn, diverged_log):
    """Run the fit's EKF calls through fn; per pass_id log the diverged
    samples split into hard-bound and NIS-window, and the hard-bound ones at
    |ay| >= MID_CORNER_AY_MPS2."""
    import modules.tyre_fit_auto as tfa
    orig = tfa.estimate_sideslip_ekf_pacejka

    def logged(state, params, pass_id):
        res = fn(state, params, pass_id)
        bound = np.radians(params["tyre_model_ekf_pacejka"][pass_id]["beta_hard_bound_deg"])
        div = res["diverged_mask"]
        hb = div & (np.abs(res["beta"]) > bound)
        mid = hb & (np.abs(state["ay_mps2"]) >= MID_CORNER_AY_MPS2)
        diverged_log.setdefault(pass_id, []).append((int(div.sum()), int(hb.sum()), int((div & ~hb).sum()),
                                                      int(mid.sum())))
        if pass_id == "_auto_final":
            diverged_log["final_masks"] = {"hard_bound": hb, "nis_window": div & ~hb,
                                           "beta_raw": res["beta"], "E": res["beta_with_fallback"]}
        return res

    tfa.estimate_sideslip_ekf_pacejka = logged
    try:
        yield
    finally:
        tfa.estimate_sideslip_ekf_pacejka = orig


def run_export(path, png):
    from modules.stability_analysis import (load_parameters, estimate_sideslip, _highpass_filter,
                                            estimate_slip_angles, estimate_lateral_forces)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import (_base_mask, _speed_window_mask, _fit_axle_pacejka, fit_session_pacejka,
                                       plausibility_gate)
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka as new_ekf
    from modules.nis_gate import evaluate_gate

    parsed, st = load(path)
    raw = load_parameters()
    ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))
    cfg = ep["tyre_fit_auto"]
    t, sr, v = st["time"], st["sample_rate_hz"], st["v_mps"]
    r, ay, moving = st["yaw_rate_radps"], st["ay_mps2"], st["moving_mask"]
    kerb = st["kerb_mask"] | st["kerb_smear_zone"]
    laps = [l for l in parsed["laps"] if l.get("is_valid_for_analysis")]
    lap_id = np.full(t.size, -1)
    for l in laps:
        lap_id[(t >= l["start_time"]) & (t <= l["end_time"])] = l["lap_number"]
    sd = steering_straights(st, laps, lap_id, kerb)
    segs = sorted(sd["segs"])
    cal, val = segs[0::2], segs[1::2]
    qcal = sorted(quasi_steady_segments(st, lap_id, kerb, sd)["segs"])[0::2]
    stem = os.path.basename(path).replace(".txt", "")
    print("\n" + "=" * 100)
    print(f"export: {path} ({track_label(path)})")

    # ---- D4 dry run on the round-3 fits
    r_off, ay_off, _ = offsets(cal, r, ay, sr)
    idx = np.concatenate([np.arange(a, b) for a, b in qcal])
    c1, c0 = np.polyfit(np.degrees(r[idx]), np.degrees(ay[idx] / v[idx]), 1)
    dt = 1.0 / sr
    vs = np.where(moving, v, 1.0)
    rate_b2 = np.where(moving, (ay - ay_off) / vs - (r - r_off), 0.0) * dt
    rate_d = np.where(moving, ay / vs - c1 * r - np.radians(c0), 0.0) * dt
    seeds = {"A": estimate_sideslip(st, ep),
             "D2": np.where(moving, _highpass_filter(np.cumsum(rate_d), B2_CUTOFF_HZ, sr), 0.0),
             "B2": np.where(moving, _highpass_filter(np.cumsum(rate_b2), B2_CUTOFF_HZ, sr), 0.0)}
    fit_mask = _base_mask(st, parsed["laps"]) & _speed_window_mask(st, parsed, cfg)[0]
    forces = estimate_lateral_forces(st, ep)
    print(f"\nD4 GATE DRY RUN (bands: span >= {cfg['min_alpha_span_deg']:g} deg; B*C*D in "
          f"[{cfg['stiffness_min_npr']:g}, {cfg['stiffness_max_npr']:g}] N/rad; peak in "
          f"[{cfg['peak_min_deg']:g}, {cfg['peak_max_deg']:g}] deg); fit recomputed per seed")
    verdict = {}
    for seed, beta in seeds.items():
        slip = estimate_slip_angles(st, beta, ep)
        parts, rejected = [], False
        for name, akey, fkey in (("front", "alpha_f_filt", "Fy_f_filt"), ("rear", "alpha_r_filt", "Fy_r_filt")):
            fit = _fit_axle_pacejka(slip[akey], forces[fkey], fit_mask)
            g = plausibility_gate(slip[akey], fit_mask, fit, cfg)
            rejected |= bool(g["failed"])
            parts.append(f"{name} B {fit['B']:.3f} C {fit['C']:.4f} D {fit['D']:.0f} E {fit['E']:+.4f} -> span "
                         f"{g['alpha_span_deg']:.2f} deg, B*C*D {g['stiffness_npr']:.0f} N/rad, peak "
                         f"{g['peak_deg']:.2f} deg, failed: {', '.join(g['failed']) or 'none'}")
        verdict[seed] = rejected
        print(f"  - seed {seed}: {'REJECTED' if rejected else 'passes'}\n      " + "\n      ".join(parts))
    if verdict["A"]:
        print("\nSTOP: seed A fails its own gate -- old vs new comparison not run.")
        return

    # ---- D3: old vs new estimator, seed A
    lr = ep["vehicle"]["cog_to_rear_axle_m"]
    bound = cfg["beta_hard_bound_deg"]
    mids = np.array([(a + b) // 2 for a, b in val])
    top = five_corners(parsed, t, ay, laps)
    runs = {}
    for label, fn in (("old", old_ekf_function()), ("new", new_ekf)):
        log = {}
        with ekf_variant(fn, log):
            man = fit_session_pacejka(parsed, ep, data_file_path=path,
                                      load_normalised=cfg.get("load_normalised_fit_enabled", False),
                                      tyre_fit_mode=cfg.get("tyre_fit_mode", "per_session"))
        gate = evaluate_gate(man["nis_full"], man["base_mask"], ep, sr)
        runs[label] = {"man": man, "div": log, "gate": gate, "E": man["beta_ekf_with_fallback"]}

    print("\nD3 OLD vs NEW (seed A, production fit)")
    for label in ("old", "new"):
        o = runs[label]
        man, ch = o["man"], o["man"]["r_sweep"]["chosen"]
        ed = np.degrees(o["E"])
        sw = [d[0] for d in o["div"].get("_auto_sweep", [])]
        it, fi = o["div"]["_auto_interim"][0], o["div"]["_auto_final"][0]
        print(f"  - {label}: fit status '{man['status']}'"
              + (f" (reason: {man.get('degenerate_reason')})" if man["status"] == "degenerate" else "")
              + f", NIS gate '{o['gate']['verdict']}' (health_score {o['gate'].get('health_score'):.4f})")
        print(f"      diverged samples: interim {it[0]}; sweep sum {sum(sw)} over {len(sw)} calls (min {min(sw)}, "
              f"max {max(sw)}); final {fi[0]}")
        print(f"      split hard-bound / NIS-window: interim {it[1]} / {it[2]}; sweep sum "
              f"{sum(d[1] for d in o['div']['_auto_sweep'])} / {sum(d[2] for d in o['div']['_auto_sweep'])}; "
              f"final {fi[1]} / {fi[2]}")
        print(f"      hard-bound re-entries at |ay| >= {MID_CORNER_AY_MPS2:g} m/s^2: interim {it[3]}; sweep sum "
              f"{sum(d[3] for d in o['div']['_auto_sweep'])}; final {fi[3]}")
        print(f"      R chosen: r_ay_scale {ch['r_ay_scale']:g} (R_ay {ch['R_ay_var']:.4g}), r_yaw_scale "
              f"{ch['r_yaw_scale']:g} (R_yaw {ch['R_yaw_rate_var']:.4g}); in band {man['r_sweep']['found_in_band']} "
              f"(f_yaw {ch['f_yaw']:.3f}, f_ay {ch['f_ay']:.3f})")
        print(f"      moving samples with |E| > hard bound {bound:g} deg: {int((np.abs(ed[moving]) > bound).sum())} "
              f"of {int(moving.sum())}; max |E| {np.max(np.abs(ed[moving])):.2f} deg")
    print("  - five reference corners, E [deg, value at peak |beta| / signed mean], old -> new:")
    for n_, c, i0, i1, a, b in top:
        kin = np.degrees(np.mean(lr * r[a:b] / v[a:b]))
        txt = []
        for label in ("old", "new"):
            seg = np.degrees(runs[label]["E"][a:b])
            txt.append(f"{seg[np.argmax(np.abs(seg))]:+.2f} / {np.mean(seg):+.2f}")
        print(f"      C{c.get('stable_corner_id')} lap {c['lap_number']}: {txt[0]} -> {txt[1]}  (l_r r / v {kin:+.2f})")
    print("  - validation straights |E| [deg, median / p95 / worst], old -> new: " + " -> ".join(
        f"{np.median(x):.2f} / {np.percentile(x, 95):.2f} / {x.max():.2f}"
        for x in (np.abs(np.degrees(runs[k]["E"][mids])) for k in ("old", "new"))))
    pg = runs["new"]["man"].get("plausibility_gate", {})
    print("  - new run's own plausibility gate: " + "; ".join(
        f"{k} failed {', '.join(g['failed']) or 'none'}" for k, g in pg.items()))
    a_ = runs["old"]["man"]["axles"]
    b_ = runs["new"]["man"]["axles"]
    same = all(abs(a_[k][p] - b_[k][p]) < 1e-9 for k in ("front", "rear") for p in "BCDE")
    print(f"  - fitted curves identical old vs new (expected: the axle fit precedes every EKF call): {same}")

    if png:
        plot(path, stem, t, sr, runs, top[0])


def plot(path, stem, t, sr, runs, corner):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    n_, c, i0, i1, a, b = corner
    e = int(EDGE_S * sr)
    sl = slice(max(0, i0 - e), min(t.size, i1 + e))
    tt = t[sl] - t[i0]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(tt, np.degrees(runs["old"]["E"][sl]), color="#d62728", lw=1.0,
            label="before: divergence resets write the kinematic sideslip into the estimator")
    ax.plot(tt, np.degrees(runs["new"]["E"][sl]), color="#1f77b4", lw=1.2,
            label="after: state kept with reset covariance, or reset to zero beyond the bound")
    ax.axvspan(t[a] - t[i0], t[b - 1] - t[i0], color="#dddddd", zorder=0)
    ax.axvline(0, color="black", lw=0.6, ls=":")
    ax.axvline(t[i1 - 1] - t[i0], color="black", lw=0.6, ls=":")
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("time from corner entry [s] (dotted: entry / exit; grey: sustained lateral run)")
    ax.set_ylabel("estimator sideslip [deg]")
    ax.set_title(f"{track_label(path)}, export {stem}\napp estimator sideslip on corner C{c.get('stable_corner_id')} "
                 f"lap {c['lap_number']}, before and after the divergence re-entry change", fontsize=10)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"hardening_corner_{stem}.png"), dpi=120)
    plt.close(fig)


DUBAI = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
P2_OUT_DIR = os.path.join("diagnostics", "output", "estimator_hardening")


def p2s1_export(path, png):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness,
                                            estimate_yaw_moment_stability, estimate_vertical_loads,
                                            summarise_corners)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import resolve_sideslip_beta
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka as new_ekf

    dubai = path == DUBAI
    if dubai:
        parsed = parse_csv(path)
        raw = load_parameters()
        ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
        st = prepare_vehicle_state(parsed["channels"], ep)
    else:
        parsed, st = load(path)
        raw = load_parameters()
        ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))
    cfg = ep["tyre_fit_auto"]
    t, ay, moving = st["time"], st["ay_mps2"], st["moving_mask"]
    n_moving = int(moving.sum())
    label = "Dubai (sample export, different car)" if dubai else track_label(path)
    print("\n" + "=" * 100)
    print(f"export: {path} ({label}); moving samples {n_moving}")

    runs = {}
    for name, fn in (("old", old_ekf_function()), ("new", new_ekf)):
        log = {}
        with ekf_variant(fn, log):
            beta, man, gate, fb, fb_reason = resolve_sideslip_beta(st, ep, parsed, "ekf_auto_pacejka", csv_path=path)
        runs[name] = {"beta": beta, "man": man, "gate": gate, "fb": fb, "fb_reason": fb_reason, "log": log}

    print("\nACCEPTANCE FACTS (old -> new)")
    for name in ("old", "new"):
        o = runs[name]
        man, sc, rs = o["man"], o["man"]["sign_check"], o["man"]["r_sweep"]
        ch = rs["chosen"]
        n_in = sum(1 for g in rs["grid_points"] if g.get("both_in_band", g.get("ay_in_band")))
        print(f"  - {name}: fit status '{man['status']}'" + (f" ({man.get('degenerate_reason')})"
              if man["status"] == "degenerate" else "") + f"; NIS gate '{o['gate']['verdict'] if o['gate'] else '-'}'"
              + f"; production beta = {'KINEMATIC FALLBACK' if o['fb'] else 'estimator'}")
        print(f"      sign check: median_gate_racing {sc['median_gate_racing']} = {sc['median_gate_racing_fraction']:.3f} "
              f"(degenerate below {cfg['sign_check_degenerate_fraction']:g}, marginal below "
              f"{cfg['sign_check_marginal_fraction']:g}); median_gate_all {sc['median_gate_all']}; "
              f"per_sample_fraction_racing {sc['per_sample_fraction_racing']:.3f}")
        print(f"      R sweep: chosen r_ay_scale {ch['r_ay_scale']:g}, r_yaw_scale {ch['r_yaw_scale']:g} -> f_yaw "
              f"{ch['f_yaw']:.3f}, f_ay {ch['f_ay']:.3f}; in band {rs['found_in_band']}; grid points in band "
              f"{n_in} of {len(rs['grid_points'])}; final NIS exceedance yaw {man['nis']['yaw_rate_exceedance']:.3f}, "
              f"ay {man['nis']['ay_exceedance']:.3f}")
        fi = o["log"]["_auto_final"][0]
        print(f"      final run: diverged {fi[0]} (hard-bound {fi[1]}, NIS-window {fi[2]}); hard-bound at |ay| >= "
              f"{MID_CORNER_AY_MPS2:g} m/s^2: {fi[3]} -> {1000.0 * fi[3] / n_moving:.3f} per 1000 moving samples "
              f"(all hard-bound {1000.0 * fi[1] / n_moving:.3f} per 1000)")

    if not dubai:
        p2s1_track_corners(path, parsed, st, ep, runs, png)
        return

    # ---- Dubai per corner (final EKF run masks; E = the beta resolve_sideslip_beta returns)
    print("\nDUBAI PER CORNER, final EKF run (old -> new); E mean per phase [deg]")
    rows = []
    for c in parsed["corners"]:
        spans = [sp for sp in c["segments"].values() if sp[1] >= sp[0]]
        if not spans:
            continue
        i0 = int(np.searchsorted(t, min(sp[0] for sp in spans)))
        i1 = int(np.searchsorted(t, max(sp[1] for sp in spans), side="right"))
        cnt = {k: (int(runs[k]["log"]["final_masks"]["hard_bound"][i0:i1].sum()),
                   int(runs[k]["log"]["final_masks"]["nis_window"][i0:i1].sum())) for k in ("old", "new")}
        hb_idx = i0 + np.where(runs["new"]["log"]["final_masks"]["hard_bound"][i0:i1])[0]
        means = []
        for ph in PHASES:
            sp = c["segments"].get(ph)
            if not sp or sp[1] < sp[0]:
                continue
            a = int(np.searchsorted(t, sp[0]))
            b = int(np.searchsorted(t, sp[1], side="right"))
            if b <= a:
                continue
            means.append(f"{ph} " + " -> ".join(f"{np.degrees(np.mean(runs[k]['beta'][a:b])):+.2f}"
                                                 for k in ("old", "new")))
        rows.append((c, i0, i1, cnt, hb_idx, means))
        affected = any(v != (0, 0) for v in cnt.values())
        print(f"  - lap {c['lap_number']} corner {c.get('corner_number')} (C{c.get('stable_corner_id')}): "
              f"hard-bound {cnt['old'][0]} -> {cnt['new'][0]}, NIS-window {cnt['old'][1]} -> {cnt['new'][1]}"
              + (f"; |ay| at new hard-bound resets [m/s^2]: "
                 + ", ".join(f"{abs(ay[j]):.1f}" for j in hb_idx) if hb_idx.size else "")
              + ("" if affected else " (no divergence)"))
        if affected:
            print("      " + "; ".join(means))

    # ---- golden reproduction and corner 0
    summ = {}
    for k in ("old", "new"):
        beta = runs[k]["beta"]
        slip = estimate_slip_angles(st, beta, ep)
        forces = estimate_lateral_forces(st, ep)
        cs = estimate_cornering_stiffness(slip, forces, st, ep)
        stab = estimate_yaw_moment_stability(st, beta, ep, parsed.get("laps", []))
        fz = estimate_vertical_loads(st, forces, ep)
        summ[k] = summarise_corners(parsed.get("corners", []), cs, stab, st, fz=fz, lap_filter=None)
    s0 = {k: summ[k][0] for k in ("old", "new")}
    print(f"\nCORNER 0 of the golden: lap {s0['old']['lap_number']} corner {s0['old']['corner_number']} "
          f"(C{s0['old']['stable_corner_id']}), apex time {s0['old']['apex_time']:.2f} s")
    for ph in PHASES:
        po, pn = s0["old"]["phases"].get(ph), s0["new"]["phases"].get(ph)
        if not po or not pn:
            continue
        parts = []
        for key in ("cs_ratio_f", "cs_ratio_r", "stability_observed_Nm_per_deg"):
            if key in po and key in pn and po[key].get("median") is not None:
                parts.append(f"{key} median {po[key]['median']:+.3f} -> {pn[key]['median']:+.3f} (n {pn[key].get('n')})")
        print(f"  - {ph}: " + "; ".join(parts))
    c0 = [r for r in rows if r[0]["lap_number"] == s0["old"]["lap_number"]
          and r[0].get("corner_number") == s0["old"]["corner_number"]][0]
    c, i0, i1, cnt, hb_idx, means = c0
    nis_idx = i0 + np.where(runs["new"]["log"]["final_masks"]["nis_window"][i0:i1])[0]
    t_entry = t[i0]
    print(f"  - corner span {t[i0]:.2f}..{t[i1 - 1]:.2f} s; phases: " + ", ".join(
        f"{ph} {c['segments'][ph][0] - t_entry:+.2f}..{c['segments'][ph][1] - t_entry:+.2f} s"
        for ph in PHASES if ph in c["segments"] and c["segments"][ph][1] >= c["segments"][ph][0]))
    print(f"  - new hard-bound resets (time from corner entry, |ay|): " + (", ".join(
        f"{t[j] - t_entry:+.2f} s at {abs(ay[j]):.1f} m/s^2" for j in hb_idx) or "none"))
    print(f"  - new NIS-window divergences: {nis_idx.size} samples" + (
        f", first {t[nis_idx[0]] - t_entry:+.2f} s, last {t[nis_idx[-1]] - t_entry:+.2f} s" if nis_idx.size else ""))
    old_hb = i0 + np.where(runs["old"]["log"]["final_masks"]["hard_bound"][i0:i1])[0]
    old_nis = i0 + np.where(runs["old"]["log"]["final_masks"]["nis_window"][i0:i1])[0]
    print(f"  - old: hard-bound {old_hb.size}, NIS-window {old_nis.size} samples" + (
        f" (first {t[old_nis[0]] - t_entry:+.2f} s, last {t[old_nis[-1]] - t_entry:+.2f} s)" if old_nis.size else ""))
    sp = c["segments"]["apex_3"]
    a = int(np.searchsorted(t, sp[0]))
    b = int(np.searchsorted(t, sp[1], side="right"))
    if b <= a:
        # apex is one instant: same +/- apex_half_window_samples as summarise_corners
        half = ep["stability_estimation"]["apex_half_window_samples"]
        a, b = max(0, a - half), min(t.size, a + half + 1)
    print(f"  - apex_3 window {t[a] - t_entry:+.2f}..{t[b - 1] - t_entry:+.2f} s, |ay| "
          f"{np.min(np.abs(ay[a:b])):.1f}..{np.max(np.abs(ay[a:b])):.1f} m/s^2; NIS-window divergences inside: "
          + ", ".join(f"{k} {int(runs[k]['log']['final_masks']['nis_window'][a:b].sum())}" for k in ("old", "new")))
    for k in ("old", "new"):
        e = np.degrees(runs[k]["beta"][a:b])
        print(f"  - E through apex_3 ({b - a} samples), {k}: " + ", ".join(f"{x:+.2f}" for x in e))

    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(P2_OUT_DIR, exist_ok=True)
        e = int(EDGE_S * st["sample_rate_hz"])
        sl = slice(max(0, i0 - e), min(t.size, i1 + e))
        tt = t[sl] - t_entry
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
        ax1.plot(tt, np.degrees(runs["old"]["beta"][sl]), color="#d62728", lw=1.0,
                 label="before: divergence resets write the kinematic sideslip into the estimator")
        ax1.plot(tt, np.degrees(runs["new"]["beta"][sl]), color="#1f77b4", lw=1.2,
                 label="after: state kept with reset covariance, or reset to zero beyond the bound")
        for j in hb_idx:
            ax1.axvline(t[j] - t_entry, color="#1f77b4", lw=0.8, ls="--")
        if nis_idx.size:
            ax1.plot(t[nis_idx] - t_entry, np.full(nis_idx.size, ax1.get_ylim()[0]), "|", color="#555555",
                     label="after: covariance reset on measurement disagreement")
        ap = c["segments"]["apex_3"]
        for axx in (ax1, ax2):
            axx.axvspan(ap[0] - t_entry, ap[1] - t_entry, color="#dddddd", zorder=0)
        ax1.axhline(0, color="black", lw=0.5)
        ax1.set_ylabel("estimator sideslip [deg]")
        ax1.legend(fontsize=7)
        ax2.plot(tt, ay[sl], color="black", lw=0.8)
        ax2.set_ylabel("lateral acceleration [m/s^2]")
        ax2.set_xlabel("time from corner entry [s] (grey: apex phase; dashed: reset to zero beyond the bound)")
        ax1.set_title(f"Dubai sample session, lap {c['lap_number']} corner {c.get('corner_number')}\n"
                      "app estimator sideslip before and after the divergence re-entry change", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(P2_OUT_DIR, "dubai_corner0_reentry.png"), dpi=120)
        plt.close(fig)


def p2s1_track_corners(path, parsed, st, ep, runs, png):
    """NBR / PR completion (author order 2026-10-11): resets per corner, phase
    means per stable corner, and the sign check per corner -- the production
    _sign_check re-done per corner on the final run's raw beta and base mask,
    as fit_session_pacejka calls it. E = the final run's beta_with_fallback
    (estimator output even where the session would fall back)."""
    from modules.tyre_fit_auto import _base_mask, _canonical_window_slice
    t, ay = st["time"], st["ay_mps2"]
    laps_by_number = {l["lap_number"]: l for l in parsed["laps"]}
    valid = {n for n, l in laps_by_number.items() if l.get("is_valid_for_analysis")}
    fm = {k: runs[k]["log"]["final_masks"] for k in ("old", "new")}

    # ---- resets per corner instance, phase means per stable corner
    print(f"\nPER CORNER, final EKF run, analysis laps only (old -> new)")
    agg = {}
    loc_lines = []
    for c in parsed["corners"]:
        if c["lap_number"] not in valid:
            continue
        spans = [sp for sp in c["segments"].values() if sp[1] >= sp[0]]
        if not spans:
            continue
        i0 = int(np.searchsorted(t, min(sp[0] for sp in spans)))
        i1 = int(np.searchsorted(t, max(sp[1] for sp in spans), side="right"))
        sid = c.get("stable_corner_id")
        g = agg.setdefault(sid, {"n": 0, "hb": [0, 0], "nis": [0, 0], "ph": {}})
        g["n"] += 1
        for j, k in enumerate(("old", "new")):
            g["hb"][j] += int(fm[k]["hard_bound"][i0:i1].sum())
            g["nis"][j] += int(fm[k]["nis_window"][i0:i1].sum())
        for k in ("old", "new"):
            idx = i0 + np.where(fm[k]["hard_bound"][i0:i1])[0]
            if idx.size:
                loc_lines.append(f"      {k}: lap {c['lap_number']} C{sid}: " + ", ".join(
                    f"{t[j] - t[i0]:+.2f} s at |ay| {abs(ay[j]):.1f}" for j in idx))
        for ph in PHASES:
            sp = c["segments"].get(ph)
            if not sp or sp[1] < sp[0]:
                continue
            a = int(np.searchsorted(t, sp[0]))
            b = int(np.searchsorted(t, sp[1], side="right"))
            if b <= a:
                continue
            g["ph"].setdefault(ph, ([], []))
            for j, k in enumerate(("old", "new")):
                g["ph"][ph][j].append(np.degrees(np.mean(fm[k]["E"][a:b])))
    print("  - hard-bound reset locations (time from corner entry):")
    print("\n".join(loc_lines) if loc_lines else "      none")
    print("  - per stable corner (instances; hard-bound old -> new; NIS-window old -> new; E phase means averaged "
          "over instances, old -> new [deg]):")
    for sid in sorted(agg, key=lambda x: (x is None, x)):
        g = agg[sid]
        means = "; ".join(f"{ph} {np.mean(v[0]):+.2f} -> {np.mean(v[1]):+.2f}" for ph, v in g["ph"].items())
        print(f"      C{sid} (n {g['n']}): hard-bound {g['hb'][0]} -> {g['hb'][1]}; NIS-window {g['nis'][0]} -> "
              f"{g['nis'][1]}; {means}")

    # ---- sign check per corner
    base = _base_mask(st, parsed["laps"])
    by_sid = {}
    for c in parsed.get("corners", []):
        if c.get("stable_corner_id") is not None:
            by_sid.setdefault(c["stable_corner_id"], []).append(c)
    racing = {sid for sid, inst in by_sid.items() if inst[0].get("speed_class") != "low"}
    print("\nSIGN CHECK PER CORNER (racing corners count; pass = median beta sign opposite to median ay sign)")
    tally = {"old": [0, 0], "new": [0, 0]}
    for sid in sorted(by_sid):
        inst = by_sid[sid]
        bs, be = inst[0].get("bracket_start_m"), inst[0].get("bracket_end_m")
        if bs is None or be is None:
            continue
        ays, bets = [], {"old": [], "new": []}
        for c in inst:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, st.get("s_m"), lap["start_time"], lap["end_time"], bs, be)
            if sl.stop <= sl.start:
                continue
            m = base[sl]
            if not m.any():
                continue
            ays.append(ay[sl][m])
            for k in ("old", "new"):
                bets[k].append(np.degrees(fm[k]["beta_raw"][sl][m]))
        if not ays:
            continue
        med_ay = float(np.median(np.concatenate(ays)))
        res = {}
        for k in ("old", "new"):
            mb = float(np.median(np.concatenate(bets[k])))
            res[k] = (mb, np.sign(mb) == -np.sign(med_ay))
            if sid in racing:
                tally[k][0] += int(res[k][1])
                tally[k][1] += 1
        flip = " FLIPS" if res["old"][1] != res["new"][1] else ""
        print(f"  - C{sid} ({'racing' if sid in racing else 'low speed, not counted'}): median ay {med_ay:+.2f} m/s^2; "
              f"median beta {res['old'][0]:+.2f} -> {res['new'][0]:+.2f} deg; pass {res['old'][1]} -> "
              f"{res['new'][1]}{flip}")
    for k in ("old", "new"):
        n_, d_ = tally[k]
        print(f"  - recomputed {k}: {n_}/{d_} = {n_ / d_ if d_ else float('nan'):.3f}; manifest "
              f"{runs[k]['man']['sign_check']['median_gate_racing']}")

    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(P2_OUT_DIR, exist_ok=True)
        stem = os.path.basename(path).replace(".txt", "")
        sids = [sid for sid in sorted(by_sid) if sid in racing]
        x = np.arange(len(sids))
        fig, ax = plt.subplots(figsize=(10, 4.5))
        for off, k, col, lab in ((-0.2, "old", "#d62728", "before the re-entry change"),
                                 (0.2, "new", "#1f77b4", "after the re-entry change")):
            vals = []
            for sid in sids:
                inst = by_sid[sid]
                bs, be = inst[0].get("bracket_start_m"), inst[0].get("bracket_end_m")
                pool = []
                for c in inst:
                    lap = laps_by_number.get(c["lap_number"])
                    if lap is None or not lap.get("is_valid_for_analysis") or bs is None or be is None:
                        continue
                    sl = _canonical_window_slice(t, st.get("s_m"), lap["start_time"], lap["end_time"], bs, be)
                    if sl.stop > sl.start and base[sl].any():
                        pool.append(np.degrees(fm[k]["beta_raw"][sl][base[sl]]))
                vals.append(float(np.median(np.concatenate(pool))) if pool else np.nan)
            ax.bar(x + off, vals, width=0.4, color=col, label=lab)
        ax.axhline(0, color="black", lw=0.6)
        ax.set_xticks(x)
        ax.set_xticklabels([f"C{sid}" for sid in sids], fontsize=8)
        ax.set_ylabel("median estimator sideslip in the corner [deg]")
        ax.set_title(f"{track_label(path)}, export {stem}\nmedian estimator sideslip per corner, before and after "
                     "the divergence re-entry change (sign check input)", fontsize=10)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(P2_OUT_DIR, f"sign_check_corners_{stem}.png"), dpi=120)
        plt.close(fig)


PINNED_RUNS = (("old", (0.1, 16.0)), ("new", (1.0, 1.0)), ("new", (0.1, 16.0)), ("old", (1.0, 1.0)))
FLIPPED_PR = (1, 3, 7, 8, 10, 11, 12, 14, 15, 16)   # flipped under new@1/1 in the 2026-10-11 completion run


def sign_check_per_corner(parsed, st, beta_raw):
    """Production _sign_check done per corner: {stable id: (racing, median ay,
    median beta deg, pass)} on the base mask, analysis laps only."""
    from modules.tyre_fit_auto import _base_mask, _canonical_window_slice
    t, ay = st["time"], st["ay_mps2"]
    base = _base_mask(st, parsed["laps"])
    laps_by_number = {l["lap_number"]: l for l in parsed["laps"]}
    by_sid = {}
    for c in parsed.get("corners", []):
        if c.get("stable_corner_id") is not None:
            by_sid.setdefault(c["stable_corner_id"], []).append(c)
    out = {}
    for sid in sorted(by_sid):
        inst = by_sid[sid]
        bs, be = inst[0].get("bracket_start_m"), inst[0].get("bracket_end_m")
        if bs is None or be is None:
            continue
        ays, bets = [], []
        for c in inst:
            lap = laps_by_number.get(c["lap_number"])
            if lap is None or not lap.get("is_valid_for_analysis"):
                continue
            sl = _canonical_window_slice(t, st.get("s_m"), lap["start_time"], lap["end_time"], bs, be)
            if sl.stop <= sl.start or not base[sl].any():
                continue
            ays.append(ay[sl][base[sl]])
            bets.append(np.degrees(beta_raw[sl][base[sl]]))
        if not ays:
            continue
        med_ay = float(np.median(np.concatenate(ays)))
        med_b = float(np.median(np.concatenate(bets)))
        out[sid] = (inst[0].get("speed_class") != "low", med_ay, med_b, bool(np.sign(med_b) == -np.sign(med_ay)))
    return out


def p2s1b_pinned(png):
    """Phase 2 step 1b (author order 2026-10-11): Paul Ricard with the R sweep
    pinned to one grid point, to separate the R choice from the re-entry
    change. Pinning = the sweep scale lists reduced to that one point in a
    copy of the params (this process only); interim R derivation unchanged,
    so a pinned run at the point the sweep chose reproduces that run."""
    import copy
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import fit_session_pacejka
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka as new_ekf

    path = EXPORTS[1]
    parsed, st = load(path)
    raw = load_parameters()
    ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))
    t = st["time"]
    fns = {"old": old_ekf_function(), "new": new_ekf}
    print("\n" + "=" * 100)
    print(f"export: {path} ({track_label(path)}); R pinned per run (r_ay_scale / r_yaw_scale)")
    res = {}
    for est, (sa, sy) in PINNED_RUNS:
        key = f"{est}@{sa:g}/{sy:g}"
        p2 = copy.deepcopy(ep)
        p2["tyre_fit_auto"]["r_sweep_ay_scales"] = [sa]
        p2["tyre_fit_auto"]["r_sweep_yaw_scales"] = [sy]
        log = {}
        with ekf_variant(fns[est], log):
            man = fit_session_pacejka(parsed, p2, data_file_path=path,
                                      load_normalised=p2["tyre_fit_auto"].get("load_normalised_fit_enabled", False),
                                      tyre_fit_mode=p2["tyre_fit_auto"].get("tyre_fit_mode", "per_session"))
        sc = sign_check_per_corner(parsed, st, log["final_masks"]["beta_raw"])
        res[key] = {"man": man, "log": log, "sc": sc}
        fi = log["_auto_final"][0]
        racing = [v for v in sc.values() if v[0]]
        print(f"\n--- {key}: fit status '{man['status']}'" + (f" ({man.get('degenerate_reason')})"
              if man["status"] == "degenerate" else ""))
        print(f"  - sign check: manifest {man['sign_check']['median_gate_racing']} = "
              f"{man['sign_check']['median_gate_racing_fraction']:.3f}; recomputed "
              f"{sum(v[3] for v in racing)}/{len(racing)}")
        print(f"  - NIS exceedance f_yaw {man['nis']['yaw_rate_exceedance']:.3f}, f_ay {man['nis']['ay_exceedance']:.3f}; "
              f"R_ay {man['final_config']['R_ay_var']:.4g}, R_yaw {man['final_config']['R_yaw_rate_var']:.4g}")
        print(f"  - final-run divergences: {fi[0]} (hard-bound {fi[1]}, NIS-window {fi[2]}; hard-bound at |ay| >= "
              f"{MID_CORNER_AY_MPS2:g} m/s^2: {fi[3]})")
        print("  - per corner (median ay; median beta; pass):")
        for sid, (rc, m_ay, mb, ok) in sc.items():
            print(f"      C{sid} ({'racing' if rc else 'low speed, not counted'}): {m_ay:+.2f} m/s^2; {mb:+.2f} deg; "
                  f"{'pass' if ok else 'FAIL'}")

    # phase means on the corners that flipped under new@1/1
    print("\nPHASE MEANS of E on the corners that flipped under new@1/1 (analysis laps, averaged over instances) "
          "[deg]: old@0.1/16 | new@1/1 | new@0.1/16")
    valid = {l["lap_number"] for l in parsed["laps"] if l.get("is_valid_for_analysis")}
    keys = ("old@0.1/16", "new@1/1", "new@0.1/16")
    for sid in FLIPPED_PR:
        acc = {}
        for c in parsed["corners"]:
            if c.get("stable_corner_id") != sid or c["lap_number"] not in valid:
                continue
            for ph in PHASES:
                sp = c["segments"].get(ph)
                if not sp or sp[1] < sp[0]:
                    continue
                a = int(np.searchsorted(t, sp[0]))
                b = int(np.searchsorted(t, sp[1], side="right"))
                if b <= a:
                    continue
                acc.setdefault(ph, {k: [] for k in keys})
                for k in keys:
                    acc[ph][k].append(np.degrees(np.mean(res[k]["log"]["final_masks"]["E"][a:b])))
        print(f"  - C{sid}: " + "; ".join(f"{ph} " + " | ".join(f"{np.mean(v[k]):+.2f}" for k in keys)
                                          for ph, v in acc.items()))

    print("\nCROSS-CHECK old@1/1 vs old@0.1/16, per racing corner:")
    for sid, v in res["old@1/1"]["sc"].items():
        if v[0]:
            print(f"  - C{sid}: old@0.1/16 {'pass' if res['old@0.1/16']['sc'][sid][3] else 'FAIL'} -> old@1/1 "
                  f"{'pass' if v[3] else 'FAIL'}")
    print("\nFRACTIONS (racing corners):")
    for k in ("new@1/1", "new@0.1/16", "old@0.1/16", "old@1/1"):
        racing = [v for v in res[k]["sc"].values() if v[0]]
        n_ = sum(v[3] for v in racing)
        print(f"  - {k}: {n_}/{len(racing)} = {n_ / len(racing):.3f}")

    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(P2_OUT_DIR, exist_ok=True)
        sids = [sid for sid, v in res["old@0.1/16"]["sc"].items() if v[0]]
        x = np.arange(len(sids))
        fig, ax = plt.subplots(figsize=(11, 4.8))
        series = (("old@0.1/16", "#d62728", "before the re-entry change, sweep choice (yaw x16, ay x0.1)"),
                  ("new@1/1", "#1f77b4", "after the re-entry change, sweep choice (yaw x1, ay x1)"),
                  ("new@0.1/16", "#2ca02c", "after the re-entry change, noise pinned to the old choice"))
        for i, (k, col, lab) in enumerate(series):
            ax.bar(x + (i - 1) * 0.27, [res[k]["sc"][sid][2] for sid in sids], width=0.27, color=col, label=lab)
        ax.axhline(0, color="black", lw=0.6)
        ax.set_xticks(x)
        ax.set_xticklabels([f"C{sid}\n(ay {'+' if res['old@0.1/16']['sc'][sid][1] > 0 else '-'})" for sid in sids],
                           fontsize=7)
        ax.set_ylabel("median estimator sideslip in the corner [deg]")
        ax.set_title(f"{track_label(path)}, export MLA-SE07_US-v2\nmedian estimator sideslip per corner: re-entry "
                     "change with the swept and with the pinned measurement noise", fontsize=10)
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(P2_OUT_DIR, "pr_pinned_noise_sign_check.png"), dpi=120)
        plt.close(fig)


def load_any(path):
    """Dubai at accuracy cap 1 (as tests/conftest.py), NBR / PR as rounds 2-3."""
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    raw = load_parameters()
    if path == DUBAI:
        parsed = parse_csv(path)
        ep = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=1))
        return parsed, prepare_vehicle_state(parsed["channels"], ep), ep
    parsed, st = load(path)
    return parsed, st, apply_resolved_vehicle(raw, resolve_accuracy(raw, None, None))


def p2s2_grid():
    """Phase 2 step 2 part 1 (author order 2026-10-11): the full 25-point R
    grid under the NEW estimator on Dubai, NBR and PR -- per grid point the NIS
    exceedance pair (f_yaw, f_ay) exactly as the sweep computes it, plus the
    sign check of that grid point's own EKF run (racing corners), so the band
    can be judged on physical sanity, not only on NIS."""
    import modules.tyre_fit_auto as tfa
    from modules.tyre_fit_auto import fit_session_pacejka
    for path in (DUBAI,) + EXPORTS:
        parsed, st, ep = load_any(path)
        cfg = ep["tyre_fit_auto"]
        sweep_betas = []
        orig = tfa.estimate_sideslip_ekf_pacejka

        def logged(state, params, pass_id):
            res = orig(state, params, pass_id)
            if pass_id == "_auto_sweep":
                sweep_betas.append(res["beta"])
            return res

        tfa.estimate_sideslip_ekf_pacejka = logged
        try:
            man = fit_session_pacejka(parsed, ep, data_file_path=path,
                                      load_normalised=cfg.get("load_normalised_fit_enabled", False),
                                      tyre_fit_mode=cfg.get("tyre_fit_mode", "per_session"))
        finally:
            tfa.estimate_sideslip_ekf_pacejka = orig
        grid = man["r_sweep"]["grid_points"]
        label = "Dubai (sample export, different car)" if path == DUBAI else track_label(path)
        print("\n" + "=" * 100)
        print(f"export: {path} ({label}); status '{man['status']}'; grid points {len(grid)}, sweep EKF runs "
              f"{len(sweep_betas)}; band [{cfg['nis_band_low']:g}, {cfg['nis_band_high']:g}]")
        print("  r_ay_scale / r_yaw_scale: f_yaw, f_ay; sign check racing pass/total (fraction)")
        for g, b in zip(grid, sweep_betas):
            sc = sign_check_per_corner(parsed, st, b)
            racing = [v for v in sc.values() if v[0]]
            n_ = sum(v[3] for v in racing)
            print(f"  - {g['r_ay_scale']:g} / {g['r_yaw_scale']:g}: f_yaw {g['f_yaw']:.3f}, f_ay {g['f_ay']:.3f}; "
                  f"sign {n_}/{len(racing)} ({n_ / len(racing):.3f})" + ("  [in band]" if g.get("both_in_band", g.get("ay_in_band")) else ""))


def p2s2_verify():
    """Phase 2 step 2, Part 1 amendment verify: the production sweep with the
    ay-only selection, sign-check guard and default-R fallback -- selection
    trace, final pick and status per export."""
    from modules.tyre_fit_auto import fit_session_pacejka
    for path in (DUBAI,) + EXPORTS:
        parsed, st, ep = load_any(path)
        cfg = ep["tyre_fit_auto"]
        man = fit_session_pacejka(parsed, ep, data_file_path=path,
                                  load_normalised=cfg.get("load_normalised_fit_enabled", False),
                                  tyre_fit_mode=cfg.get("tyre_fit_mode", "per_session"))
        rs = man["r_sweep"]
        ch = rs["chosen"]
        label = "Dubai (sample export, different car)" if path == DUBAI else track_label(path)
        print("=" * 100)
        print(f"export: {path} ({label})")
        cands = [g for g in rs["grid_points"] if g["ay_in_band"]]
        print(f"  - ay-band candidates [{cfg['r_sweep_ay_band_low']:g}, {cfg['r_sweep_ay_band_high']:g}]: " + ", ".join(
            f"{g['r_ay_scale']:g}/{g['r_yaw_scale']:g} (f_ay {g['f_ay']:.3f}, sign {g['sign_fraction']:.3f})"
            for g in cands))
        print("  - guard trace (nearest the centre first): " + ", ".join(
            f"{g['r_ay_scale']:g}/{g['r_yaw_scale']:g} sign {g['sign_fraction']:.3f} "
            f"{'pass' if g['guard_pass'] else 'FAIL'}" for g in rs["guard_trace"]))
        print(f"  - selection '{rs['selection']}'; pick {ch['r_ay_scale']:g}/{ch['r_yaw_scale']:g} (f_ay {ch['f_ay']:.3f}, "
              f"f_yaw {ch['f_yaw']:.3f}, grid sign {ch['sign_fraction']:.3f})")
        sc = man["sign_check"]
        print(f"  - final run: sign check {sc['median_gate_racing']} = {sc['median_gate_racing_fraction']:.3f}; NIS "
              f"exceedance yaw {man['nis']['yaw_rate_exceedance']:.3f}, ay {man['nis']['ay_exceedance']:.3f}; status "
              f"'{man['status']}'" + (f" ({man.get('degenerate_reason')})" if man["status"] == "degenerate" else "")
              + (f" ({man.get('marginal_reason')})" if man["status"] == "marginal" else ""))


POOL_KEYS = ("dubai", "v3", "pr_us_v2", "nbr_jst_problem", "nbr_jst_solution", "nbr_qp_cdr", "nbr_qp_mla")
STAB_PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


def _session_chain(key, fn=None, pin=None):
    """The threshold diagnostic's chain (inspect_threshold_rederivation
    compute_clean, fresh): best-available params, resolve_sideslip_beta, slip,
    forces, CS, stability with kerb-smear stability samples NaN, summaries,
    aggregate_by_corner over analysis laps. fn = EKF function (None = current
    module), pin = (ay_scale, yaw_scale) to pin the R sweep. Returns a dict with
    the final-run masks, beta, stability inputs and per-corner worst phase."""
    import copy
    from diagnostics import inspect_tyre_prior_d2 as d2
    from diagnostics.inspect_new_laps_cs_sweep import SESSIONS
    from modules.csv_parser import parse_csv
    from modules.recommendation import aggregate_by_corner
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners, prepare_vehicle_state)
    from modules.tyre_fit_auto import resolve_sideslip_beta
    from modules.sideslip_ekf_pacejka import estimate_sideslip_ekf_pacejka as new_ekf
    params = d2._params()
    if pin is not None:
        params = copy.deepcopy(params)
        params["tyre_fit_auto"]["r_sweep_ay_scales"] = [pin[0]]
        params["tyre_fit_auto"]["r_sweep_yaw_scales"] = [pin[1]]
    data = parse_csv(SESSIONS[key] if key in SESSIONS else {"nbr73": EXPORTS[0]}[key])
    state = prepare_vehicle_state(data["channels"], params)
    log = {}
    with ekf_variant(fn or new_ekf, log):
        beta, man, gate, fb, fr = resolve_sideslip_beta(state, params, data,
                                                        params["stability_estimation"]["sideslip_source"])
    gate_info = {"verdict": gate.get("verdict"), "health_score": gate.get("health_score")} if gate else None
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(estimate_slip_angles(state, beta, params), forces, state, params)
    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
    so = np.array(stab["stability_observed_Nm_per_deg"], float)
    so[np.asarray(state["kerb_smear_zone"], bool)] = np.nan
    stab_c = dict(stab)
    stab_c["stability_observed_Nm_per_deg"] = so
    valid = {l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")}
    summ = summarise_corners(data.get("corners", []), cs, stab_c, state, lap_filter=None)
    agg = aggregate_by_corner([x for x in summ if x["lap_number"] in valid])
    corners = {}
    for cid, a in agg.items():
        if cid is None:
            continue
        vals = {ph: a["phases"][ph]["stability_observed_Nm_per_deg"]["median"] for ph in STAB_PHASES}
        fin = {ph: v for ph, v in vals.items() if v == v}
        if fin:
            worst = min(fin, key=fin.get)
            corners[cid] = {"stab": fin[worst], "phase": worst, "phases": vals}
    return {"key": key, "params": params, "data": data, "state": state, "beta": beta, "man": man,
            "fallback": fb, "log": log, "stab": stab, "corners": corners, "valid_laps": valid, "gate": gate_info}


def _phase_slices(run, cid, phase):
    """Time slices of one stable corner's phase on the analysis laps (apex_3 =
    one instant +/- apex_half_window_samples, as summarise_corners)."""
    t = run["state"]["time"]
    half = run["params"]["stability_estimation"]["apex_half_window_samples"]
    out = []
    for c in run["data"]["corners"]:
        if c.get("stable_corner_id") != cid or c["lap_number"] not in run["valid_laps"]:
            continue
        sp = c["segments"].get(phase)
        if not sp or sp[1] < sp[0]:
            continue
        a = int(np.searchsorted(t, sp[0], side="left"))
        b = int(np.searchsorted(t, sp[1], side="right"))
        if b <= a:
            a, b = max(0, a - half), min(t.size, a + half + 1)
        out.append((c["lap_number"], slice(a, b)))
    return out


def _local_regression(run, beta, s_centre):
    """calculate_observed_stability's regression at one grid centre, unpacked:
    regressand mz_inertial_Nm, predictors beta_rad, delta_f_rad, v_mps,
    ax_mps2, az_mps2 (when az exists), Gaussian weights in s (sigma =
    window / 2.5), centred and scaled predictors, ridge solve. Adds an
    approximate coefficient standard error (sandwich form, weighted residual
    variance) -- reporting only."""
    from modules.yaw_stability import GAUSS_SIGMA_DIVISOR, _PREDICTOR_COLUMNS
    st, se = run["state"], run["params"]["stability_estimation"]
    from modules.stability_analysis import _build_inout_lap_mask
    keep = st["moving_mask"].copy()
    if st.get("kerb_mask") is not None:
        keep &= ~st["kerb_mask"]
    keep &= ~_build_inout_lap_mask(st["time"], run["data"].get("laps"))
    cols = {"beta_rad": np.asarray(beta, float), "delta_f_rad": st["delta_f_rad"], "v_mps": st["v_mps"],
            "ax_mps2": st["ax_mps2"]}
    if st.get("az_g") is not None:
        cols["az_mps2"] = st["az_g"] * 9.81
    preds = [c for c in _PREDICTOR_COLUMNS if c in cols]
    y = run["stab"]["mz_inertial_Nm"]
    sm = st["s_m"]
    w_m = se["yaw_stability_window_m"]
    m = keep & np.isfinite(sm) & np.isfinite(y) & np.all([np.isfinite(cols[c]) for c in preds], axis=0)
    m &= np.abs(sm - s_centre) <= w_m
    X = np.column_stack([cols[c][m] for c in preds])
    Y = y[m]
    w = np.exp(-0.5 * (np.abs(sm[m] - s_centre) / (w_m / GAUSS_SIGMA_DIVISOR)) ** 2)
    W = w.sum()
    xm = (X * w[:, None]).sum(0) / W
    ym = float((Y * w).sum() / W)
    xc, yc = X - xm, Y - ym
    sd = np.sqrt(np.maximum((w[:, None] * xc ** 2).sum(0) / W, 1e-12))
    xs = xc / sd
    A = xs * np.sqrt(w)[:, None]
    bvec = yc * np.sqrt(w)
    lam = se["yaw_stability_ridge"]
    M = np.linalg.inv(A.T @ A + lam * np.eye(len(preds)))
    coef = M @ (A.T @ bvec)
    res = bvec - A @ coef
    s2 = float(res @ res) / max(1.0, W - len(preds))
    cov = s2 * M @ (A.T @ A) @ M
    slope = coef[0] / sd[0] * np.pi / 180.0
    se_slope = np.sqrt(cov[0, 0]) / sd[0] * np.pi / 180.0
    return {"n": int(m.sum()), "preds": preds, "mean": dict(zip(preds, xm)), "sd": dict(zip(preds, sd)),
            "y_mean": ym, "slope": slope, "se": se_slope,
            "std_coef": dict(zip(preds, coef)), "mask": m}


def _reset_stats(run, slices):
    """NIS-window and hard-bound re-entries of the final run inside the
    slices, and the beta jump across reset samples vs elsewhere."""
    fm = run["log"]["final_masks"]
    E = fm["E"]
    t = run["state"]["time"]
    nis_t, jumps_r, jumps_o = [], [], []
    hb = 0
    for lap, sl in slices:
        idx = np.arange(sl.start, sl.stop)
        nis_i = idx[fm["nis_window"][sl]]
        hb += int(fm["hard_bound"][sl].sum())
        t0 = t[sl.start]
        nis_t += [(lap, float(t[i] - t0)) for i in nis_i]
        d = np.abs(np.diff(np.degrees(E[max(0, sl.start - 1):sl.stop])))
        flag = fm["nis_window"][sl]
        jumps_r += list(d[flag])
        jumps_o += list(d[~flag])
    return {"nis": nis_t, "hb": hb,
            "jump_reset_median": float(np.median(jumps_r)) if jumps_r else float("nan"),
            "jump_reset_max": float(np.max(jumps_r)) if jumps_r else float("nan"),
            "jump_other_median": float(np.median(jumps_o)) if jumps_o else float("nan"),
            "jump_other_p99": float(np.percentile(jumps_o, 99)) if jumps_o else float("nan")}


def _worked_corner(label, runs, cid, png_name):
    """runs: list of (name, run). The worst phase is taken from the first run
    (the new estimator at the ruled pick)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    first = runs[0][1]
    phase = first["corners"][cid]["phase"]
    slices = _phase_slices(first, cid, phase)
    sm = first["state"]["s_m"]
    s_lo = float(np.median([np.nanmin(sm[sl]) for _, sl in slices]))
    s_hi = float(np.median([np.nanmax(sm[sl]) for _, sl in slices]))
    s_c = 0.5 * (s_lo + s_hi)
    print(f"\n--- {label}: C{cid}, worst phase (first run) {phase}, s {s_lo:.0f}..{s_hi:.0f} m, regression "
          f"centre s {s_c:.0f} m, {len(slices)} analysis-lap instances")
    for name, run in runs:
        c = run["corners"].get(cid, {})
        print(f"  - {name}: worst-phase value {c.get('stab', float('nan')):+.0f} Nm/deg ({c.get('phase')}); "
              f"{phase} median {c.get('phases', {}).get(phase, float('nan')):+.0f}")
    print("  1. regression inputs over the window s +/- window around the centre (weighted mean / weighted std):")
    regs = {}
    for name, run in runs:
        r = _local_regression(run, run["beta"], s_c)
        regs[name] = r
        print(f"     {name}: n {r['n']}; " + "; ".join(
            f"{c} {r['mean'][c]:+.4g} / {r['sd'][c]:.4g}" for c in r["preds"])
              + f"; mz_inertial_Nm mean {r['y_mean']:+.0f}")
    print("  2. beta through the phase and re-entries (final run):")
    for name, run in runs:
        rs = _reset_stats(run, _phase_slices(run, cid, phase))
        e = np.concatenate([np.degrees(run["log"]["final_masks"]["E"][sl]) for _, sl in slices])
        print(f"     {name}: beta in phase mean {np.mean(e):+.2f}, p5..p95 {np.percentile(e, 5):+.2f}..{np.percentile(e, 95):+.2f} "
              f"deg; NIS-window resets {len(rs['nis'])}, hard-bound {rs['hb']}"
              + (" at " + ", ".join(f"lap {l} +{tt:.2f} s" for l, tt in rs["nis"][:12])
                 + (" ..." if len(rs["nis"]) > 12 else "") if rs["nis"] else "")
              + f"; |dbeta| per sample at resets median {rs['jump_reset_median']:.3f} / max {rs['jump_reset_max']:.3f}, "
              f"elsewhere median {rs['jump_other_median']:.3f} / p99 {rs['jump_other_p99']:.3f} deg")
    print("  3. regression result at the centre (beta slope = stability [Nm/deg], approx. standard error, "
          "intercept = weighted mean mz):")
    for name, r in regs.items():
        print(f"     {name}: slope {r['slope']:+.0f} +/- {r['se']:.0f} Nm/deg, intercept {r['y_mean']:+.0f} Nm, n {r['n']}; "
              f"standardised coefficients " + ", ".join(f"{c} {v:+.1f}" for c, v in r["std_coef"].items()))
    os.makedirs(P2_OUT_DIR, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 4.8))
    colours = ("#1f77b4", "#2ca02c", "#d62728")
    for (name, run), col in zip(runs, colours):
        fm = run["log"]["final_masks"]
        for k, (lap, sl) in enumerate(_phase_slices(run, cid, phase)):
            ss = run["state"]["s_m"]
            win = slice(max(0, sl.start - 150), min(ss.size, sl.stop + 150))
            ax.plot(ss[win], np.degrees(fm["E"][win]), color=col, lw=0.7, alpha=0.7, label=name if k == 0 else None)
            ri = np.arange(win.start, win.stop)[fm["nis_window"][win]]
            ax.plot(ss[ri], np.degrees(fm["E"][ri]), "x", color=col, ms=4)
    ax.axvspan(s_lo, s_hi, color="#dddddd", zorder=0)
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("lap distance [m] (grey: worst phase; x: covariance reset on measurement disagreement)")
    ax.set_ylabel("estimator sideslip [deg]")
    ax.set_title(f"{label}, corner C{cid}\nestimator sideslip through the phase with the lowest observed "
                 "stability, all analysis laps", fontsize=10)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(P2_OUT_DIR, png_name), dpi=120)
    plt.close(fig)


def p2s2c_stability():
    """Phase 2 step 2c (author order 2026-10-11): why worst-phase observed
    stability turns negative under the corrected estimator -- three worked
    corners and the population count of re-entries inside newly negative
    worst phases."""
    import pickle
    old_fn = old_ekf_function()
    # worked corners: Dubai at the ruled pick (0.03/1), at 0.1/16, old; JST 21 new, old
    dub_new = _session_chain("dubai")
    dub_016 = _session_chain("dubai", pin=(0.1, 16.0))
    dub_old = _session_chain("dubai", fn=old_fn)
    print("Dubai picks: new " + f"{dub_new['man']['r_sweep']['chosen']['r_ay_scale']:g}/"
          f"{dub_new['man']['r_sweep']['chosen']['r_yaw_scale']:g}, old "
          f"{dub_old['man']['r_sweep']['chosen']['r_ay_scale']:g}/{dub_old['man']['r_sweep']['chosen']['r_yaw_scale']:g}")
    print("Dubai per corner worst-phase stability [Nm/deg]: new@pick | new@0.1/16 | old")
    for cid in sorted(dub_new["corners"]):
        vals = [r["corners"].get(cid, {}).get("stab", float("nan")) for r in (dub_new, dub_016, dub_old)]
        print(f"  - C{cid}: " + " | ".join(f"{v:+.0f}" for v in vals)
              + f"  ({dub_new['corners'][cid]['phase']})")
    r_sens = [c for c in dub_new["corners"] if dub_new["corners"][c]["stab"] < 0
              and dub_016["corners"].get(c, {}).get("stab", -1) >= 0]
    both = [c for c in dub_new["corners"] if dub_new["corners"][c]["stab"] < 0
            and dub_016["corners"].get(c, {}).get("stab", 1) < 0]
    runs3 = lambda: [("new estimator at the ruled pick", dub_new), ("new estimator, noise pinned 0.1/16", dub_016),
                     ("old estimator", dub_old)]
    if r_sens:
        cid = min(r_sens, key=lambda c: dub_new["corners"][c]["stab"])
        _worked_corner("Dubai sample, R-sensitive case", runs3(), cid, f"stability_dubai_C{cid}_r_sensitive.png")
    if both:
        cid = min(both, key=lambda c: dub_new["corners"][c]["stab"])
        _worked_corner("Dubai sample, negative at both picks", runs3(), cid, f"stability_dubai_C{cid}_both.png")
    jst_new = _session_chain("nbr_jst_problem")
    jst_old = _session_chain("nbr_jst_problem", fn=old_fn)
    newly = [c for c in jst_new["corners"] if jst_new["corners"][c]["stab"] < 0
             and jst_old["corners"].get(c, {}).get("stab", -1) >= 0]
    print(f"\nNBR JST 21 newly negative corners (fresh old vs new): {', '.join(f'C{c}' for c in sorted(newly))}")
    if newly:
        cid = min(newly, key=lambda c: jst_new["corners"][c]["stab"])
        _worked_corner("Nurburgring JST outing 21", [("new estimator at the ruled pick", jst_new),
                                                    ("old estimator", jst_old)], cid,
                       f"stability_nbr_jst21_C{cid}.png")

    # population: newly negative corners (2026-10-05 cache old, fresh new) and resets in their worst phase
    print("\nPOPULATION (seven-session pool): newly negative = old cache (2026-10-05) >= 0, new fresh < 0")
    cnt = {"ge5": 0, "zero": 0, "1to4": 0, "n": 0}
    done = {"dubai": dub_new, "nbr_jst_problem": jst_new}
    for key in POOL_KEYS:
        run = done.get(key) or _session_chain(key)
        with open(os.path.join("diagnostics", "plots", "threshold_rederivation", "cache_clean", f"{key}.pkl"), "rb") as fh:
            old = pickle.load(fh)["clean"]
        for cid, c in run["corners"].items():
            o = old.get(cid, {}).get("stab", float("nan"))
            if c["stab"] < 0 and o == o and o >= 0:
                n_res = len(_reset_stats(run, _phase_slices(run, cid, c["phase"]))["nis"])
                cnt["n"] += 1
                cnt["ge5" if n_res >= 5 else ("zero" if n_res == 0 else "1to4")] += 1
                print(f"  - {key} C{cid} ({c['phase']}): {c['stab']:+.0f} Nm/deg, NIS-window resets in phase {n_res}")
    print(f"  TOTAL newly negative {cnt['n']}: >= 5 resets {cnt['ge5']}, 1-4 resets {cnt['1to4']}, none {cnt['zero']}")


ROBUST_OUT = os.path.join(P2_OUT_DIR, "robustness")


def p2s2_robust(key):
    """Phase 2 step 2 Part 2 ride-along (author ruling 2026-10-11): one pool
    session at the ruled pick (production) and with R pinned to the fallback
    0.1/16 -- worst-phase stability per corner for the R-robust / R-sensitive
    split, plus the acceptance metrics (sign-check fraction, NIS exceedances,
    NIS-gate health score) for Part 3. Writes one pickle per session."""
    import pickle
    out = {"key": key}
    for tag, pin in (("pick", None), ("pin_0.1_16", (0.1, 16.0))):
        run = _session_chain(key, pin=pin)
        man = run["man"]
        out[tag] = {"corners": {c: v["stab"] for c, v in run["corners"].items()},
                    "phases": {c: v["phase"] for c, v in run["corners"].items()},
                    "status": man.get("status"), "reason": man.get("degenerate_reason") or man.get("marginal_reason"),
                    "fallback": run["fallback"],
                    "pick": (man["r_sweep"]["chosen"]["r_ay_scale"], man["r_sweep"]["chosen"]["r_yaw_scale"])
                    if "r_sweep" in man else None,
                    "sign": man.get("sign_check", {}).get("median_gate_racing_fraction"),
                    "sign_txt": man.get("sign_check", {}).get("median_gate_racing"),
                    "nis": man.get("nis"), "gate": run.get("gate")}
    os.makedirs(ROBUST_OUT, exist_ok=True)
    with open(os.path.join(ROBUST_OUT, f"{key}.pkl"), "wb") as fh:
        pickle.dump(out, fh)
    print(f"[{key}] done", flush=True)


def p2s2_robust_report():
    import pickle
    tot = {"both": 0, "pick_only": 0, "neg_pick": 0}
    print("R-ROBUSTNESS of negative worst-phase stability (ruled pick vs fallback R 0.1/16), seven-session pool")
    for key in POOL_KEYS:
        with open(os.path.join(ROBUST_OUT, f"{key}.pkl"), "rb") as fh:
            r = pickle.load(fh)
        pk, pn = r["pick"], r["pin_0.1_16"]
        neg = [c for c, v in pk["corners"].items() if v < 0]
        both = [c for c in neg if pn["corners"].get(c, 1.0) < 0]
        only = [c for c in neg if c not in both]
        tot["both"] += len(both)
        tot["pick_only"] += len(only)
        tot["neg_pick"] += len(neg)
        print(f"  - {key}: pick {pk['pick']}, status '{pk['status']}', sign {pk['sign_txt']}; negative at pick {len(neg)} "
              f"of {len(pk['corners'])}: R-robust {len(both)} ({', '.join(f'C{c}' for c in sorted(both)) or '-'}), "
              f"R-sensitive {len(only)} ({', '.join(f'C{c}' for c in sorted(only)) or '-'}); negative at 0.1/16 "
              f"{sum(1 for v in pn['corners'].values() if v < 0)}")
        g = pk["gate"] or {}
        print(f"      acceptance at the pick: sign fraction {pk['sign']}; NIS exceedance yaw "
              f"{(pk['nis'] or {}).get('yaw_rate_exceedance', float('nan')):.3f}, ay "
              f"{(pk['nis'] or {}).get('ay_exceedance', float('nan')):.3f}; NIS gate {g.get('verdict')} "
              f"(health_score {g.get('health_score')})")
    print(f"  TOTAL negative at the ruled picks {tot['neg_pick']}: R-robust {tot['both']}, R-sensitive {tot['pick_only']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--step", choices=("p1", "p2s1", "p2s1b", "p2s2grid", "p2s2verify", "p2s2c", "p2s2robust", "p2s2robust_report"), default="p1")
    ap.add_argument("--skip-dubai", action="store_true", help="p2s1 only: NBR and PR (Dubai done 2026-10-11)")
    ap.add_argument("--key", help="p2s2robust: one pool session key")
    args = ap.parse_args()
    if args.step == "p2s2robust":
        p2s2_robust(args.key)
        return
    if args.step == "p2s2robust_report":
        p2s2_robust_report()
        return
    if args.step == "p2s2c":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 2c -- stability surge mechanism, numbers only")
        p2s2c_stability()
        return
    if args.step == "p2s2verify":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 2 PART 1 -- R selection verify, numbers only")
        p2s2_verify()
        return
    if args.step == "p2s2grid":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 2 PART 1 -- full R grid, new estimator, numbers only")
        p2s2_grid()
        return
    if args.step == "p2s1b":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 1b -- Paul Ricard with pinned measurement noise, numbers only")
        p2s1b_pinned(args.png)
        return
    if args.step == "p2s1":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 1 -- Dubai under the new re-entry + acceptance facts, numbers only")
        for path in (() if args.skip_dubai else (DUBAI,)) + EXPORTS:
            p2s1_export(path, args.png)
        return
    print("ESTIMATOR HARDENING D3 + D4, PHASE 1 -- offline validation, numbers only")
    for path in EXPORTS:
        run_export(path, args.png)


if __name__ == "__main__":
    main()
