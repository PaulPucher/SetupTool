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
# Run: python -m diagnostics.check_estimator_hardening [--png] [--step p1|p2s1]

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
            diverged_log["final_masks"] = {"hard_bound": hb, "nis_window": div & ~hb}
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
        n_in = sum(1 for g in rs["grid_points"] if g["both_in_band"])
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--step", choices=("p1", "p2s1"), default="p1")
    args = ap.parse_args()
    if args.step == "p2s1":
        print("ESTIMATOR HARDENING, PHASE 2 STEP 1 -- Dubai under the new re-entry + acceptance facts, numbers only")
        for path in (DUBAI,) + EXPORTS:
            p2s1_export(path, args.png)
        return
    print("ESTIMATOR HARDENING D3 + D4, PHASE 1 -- offline validation, numbers only")
    for path in EXPORTS:
        run_export(path, args.png)


if __name__ == "__main__":
    main()
