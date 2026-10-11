# Seed sensitivity of the session Pacejka fit and of the app's estimator E
# (author work order 2026-10-07, EXPERIMENT ROUND 3 + amendment).
# OFFLINE DIAGNOSTIC: nothing here enters production; numbers only, the
# verdict is the author's.
#
# Question: the fit is seeded by the washed-out kinematic beta (A); does the
# seed matter downstream? Seeds, built exactly as round 2
# (compare_beta_architectures.run_export_r2):
#   A   production kinematic beta (baseline, identical under both routes)
#   D2  gain + offset corrected integration + 0.005 Hz washout
#   B2  offsets-only corrected integration + 0.005 Hz washout
#
# The kinematic beta is read at two points of the fit path:
#   (1) tyre_fit_auto.fit_session_pacejka, `beta_kin = estimate_sideslip(...)`
#       -> slip angles -> axle fit data;
#   (2) sideslip_ekf_pacejka.estimate_sideslip_ekf_pacejka, `beta_kinematic =
#       estimate_sideslip(...)` -> fallback output AND filter re-initialisation
#       on every diverged sample, in all 11 EKF calls of the fit.
# Route (a) swaps the module-level name estimate_sideslip in tyre_fit_auto only
# (its only call inside fit_session_pacejka is point 1); route (b) also swaps it
# in sideslip_ekf_pacejka. Both swaps live in this process only and are undone
# after each run. fit_session_pacejka writes no file (manifest in memory; its
# only side effect is `git rev-parse HEAD`), so nothing needs redirecting.
# Read point (2) exists only in the pre-2026-10-07 EKF (divergence re-entry
# now widens P instead of writing the kinematic beta, D3); against the
# current EKF route (b) equals route (a).
#
# E is the fit manifest's beta_ekf_with_fallback, and the NIS gate is
# evaluated exactly as resolve_sideslip_beta does. A degenerate fit or a
# gate "fail" marks the run "would not have shipped"; E's metrics are still
# computed from E, never from the seed.
#
# Run: python -m diagnostics.seed_sensitivity_fit [--png]

import argparse
import contextlib
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

RUNS = (("A", "-"), ("D2", "a"), ("D2", "b"), ("B2", "a"), ("B2", "b"))
SEED_LABELS = {"A": "A production kinematic", "D2": "D2 gain + offset corrected + 0.005 Hz washout",
               "B2": "B2 offsets only + 0.005 Hz washout"}
ROUTE_LABELS = {"-": "baseline", "a": "route (a) fit data only", "b": "route (b) fit data and EKF fallback / reset"}
COLOURS = {("A", "-"): "#d62728", ("D2", "a"): "#17becf", ("D2", "b"): "#1f77b4",
           ("B2", "a"): "#98df8a", ("B2", "b"): "#2ca02c"}


def run_name(seed, route):
    return SEED_LABELS[seed] + ("" if route == "-" else f", {ROUTE_LABELS[route]}")


@contextlib.contextmanager
def substituted(seed_beta, route, diverged_log):
    """Swap the kinematic-beta reads for the seed (route a / b) and record the
    diverged-sample count of every EKF call the fit makes, by pass_id."""
    import modules.tyre_fit_auto as tfa
    import modules.sideslip_ekf_pacejka as ekf
    orig_tfa, orig_run = tfa.estimate_sideslip, tfa.estimate_sideslip_ekf_pacejka
    orig_ekf = getattr(ekf, "estimate_sideslip", None)

    def seed_fn(state, params):
        return seed_beta.copy()

    def logged_run(state, params, pass_id):
        res = orig_run(state, params, pass_id)
        diverged_log.setdefault(pass_id, []).append(int(res["diverged_mask"].sum()))
        return res

    if seed_beta is not None:
        tfa.estimate_sideslip = seed_fn
        if route == "b" and orig_ekf is not None:
            ekf.estimate_sideslip = seed_fn
    tfa.estimate_sideslip_ekf_pacejka = logged_run
    try:
        yield
    finally:
        tfa.estimate_sideslip = orig_tfa
        if orig_ekf is not None:
            ekf.estimate_sideslip = orig_ekf
        tfa.estimate_sideslip_ekf_pacejka = orig_run


def linear_stiffness(ax):
    # Magic Formula slope at zero slip: dFy/dalpha(0) = B * C * D [N/rad]
    return ax["B"] * ax["C"] * ax["D"]


def run_export(path, png):
    from modules.stability_analysis import load_parameters, estimate_sideslip, _highpass_filter, estimate_slip_angles
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.tyre_fit_auto import _base_mask, _speed_window_mask, fit_session_pacejka
    from modules.tyre_model_pacejka import pacejka_lateral_stiffness
    from modules.nis_gate import evaluate_gate

    parsed, st = load(path)
    raw = load_parameters()
    resolved = resolve_accuracy(raw, None, None)
    ep = apply_resolved_vehicle(raw, resolved)
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
    print(f"straights {len(segs)} -> CALIBRATION {len(cal)}, VALIDATION {len(val)}; "
          f"calibration quasi-steady cornering segments {len(qcal)} (splits as round 2)")

    # ---- seeds, as round 2
    r_off, ay_off, _ = offsets(cal, r, ay, sr)
    idx = np.concatenate([np.arange(a, b) for a, b in qcal])
    c1, c0 = np.polyfit(np.degrees(r[idx]), np.degrees(ay[idx] / v[idx]), 1)
    dt = 1.0 / sr
    vs = np.where(moving, v, 1.0)
    rate_b2 = np.where(moving, (ay - ay_off) / vs - (r - r_off), 0.0) * dt
    rate_d = np.where(moving, ay / vs - c1 * r - np.radians(c0), 0.0) * dt
    seeds = {"A": None,
             "D2": np.where(moving, _highpass_filter(np.cumsum(rate_d), B2_CUTOFF_HZ, sr), 0.0),
             "B2": np.where(moving, _highpass_filter(np.cumsum(rate_b2), B2_CUTOFF_HZ, sr), 0.0)}
    print(f"seed corrections: D2 c1 = {c1:.4f}, c0 = {c0:+.3f} deg/s; B2 yaw offset {np.degrees(r_off):+.3f} deg/s, "
          f"ay offset {ay_off:+.3f} m/s^2")

    load_norm = ep["tyre_fit_auto"].get("load_normalised_fit_enabled", False)
    fit_mode = ep["tyre_fit_auto"].get("tyre_fit_mode", "per_session")
    fit_mask = _base_mask(st, parsed["laps"]) & _speed_window_mask(st, parsed, ep["tyre_fit_auto"])[0]
    mids = np.array([(a + b) // 2 for a, b in val])
    top = five_corners(parsed, t, ay, laps)
    lr = ep["vehicle"]["cog_to_rear_axle_m"]

    res = {}
    for seed, route in RUNS:
        log = {}
        with substituted(seeds[seed], route, log):
            man = fit_session_pacejka(parsed, ep, data_file_path=path, load_normalised=load_norm,
                                      tyre_fit_mode=fit_mode)
        out = {"man": man, "div": log}
        if man.get("status") == "degenerate" and "beta_ekf_with_fallback" not in man:
            out["shipped"] = False
            out["outcome"] = f"fit degenerate before the EKF: {man.get('degenerate_reason')}"
            res[(seed, route)] = out
            continue
        gate = evaluate_gate(man["nis_full"], man["base_mask"], ep, sr)
        out["gate"] = gate
        out["shipped"] = man["status"] != "degenerate" and gate["verdict"] != "fail"
        out["outcome"] = (f"fit status '{man['status']}'" + (f" ({man.get('degenerate_reason')})"
                          if man["status"] == "degenerate" and man.get("degenerate_reason") else "")
                          + f"; NIS gate verdict '{gate['verdict']}' (health_score {gate.get('health_score')!r}, "
                          f"threshold_warn {gate.get('threshold_warn')})")
        out["E"] = man["beta_ekf_with_fallback"]
        res[(seed, route)] = out

    for seed, route in RUNS:
        o = res[(seed, route)]
        man = o["man"]
        print(f"\n--- RUN: {run_name(seed, route)}")
        print(f"  - resolve outcome: {o['outcome']} -> " + ("would have shipped" if o["shipped"]
                                                             else "WOULD NOT HAVE SHIPPED"))
        print("  - diverged samples per EKF call group: "
              + "; ".join(f"{g} {', '.join(str(x) for x in o['div'].get(p, [])) or 'not run'}"
                          + (f" (sum {sum(o['div'][p])})" if len(o["div"].get(p, [])) > 1 else "")
                          for g, p in (("interim", "_auto_interim"), ("sweep", "_auto_sweep"),
                                       ("final", "_auto_final"))))
        ax = man.get("axles", {})
        for name in ("front", "rear"):
            a_ = ax.get(name, {})
            if not a_:
                continue
            k = linear_stiffness(a_)
            k_chk = float(pacejka_lateral_stiffness(0.0, a_["B"], a_["C"], a_["D"], a_["E"]))
            o.setdefault("K", {})[name] = k
            print(f"  - fit {name}: B {a_['B']:.3f}, C {a_['C']:.4f}, D {a_['D']:.0f} N, E {a_['E']:+.4f}; Powell "
                  f"converged {a_['powell_converged']}, n {a_['fit_n_samples']}, rms resid {a_['fit_rms_resid_N']:.0f} N, "
                  f"peak alpha {a_['peak_alpha_deg']:.2f} deg (in visited range {a_['peak_in_visited_range']}); "
                  f"implied linear stiffness B*C*D = {k:.0f} N/rad = {k * np.pi / 180:.0f} N/deg "
                  f"(analytic dFy/dalpha(0) check {k_chk:.0f})")
        if "r_sweep" in man:
            ch = man["r_sweep"]["chosen"]
            print(f"  - R sweep chose r_ay_scale {ch['r_ay_scale']:g} (R_ay {ch['R_ay_var']:.4g}), r_yaw_scale "
                  f"{ch['r_yaw_scale']:g} (R_yaw {ch['R_yaw_rate_var']:.4g}); in band "
                  f"{man['r_sweep']['found_in_band']} (f_yaw {ch['f_yaw']:.3f}, f_ay {ch['f_ay']:.3f})")
            sc = man["sign_check"]
            print(f"  - self-checks: sign check median gate racing fraction {sc['median_gate_racing_fraction']:.3f}; "
                  f"NIS exceedance yaw {man['nis']['yaw_rate_exceedance']:.3f}, ay {man['nis']['ay_exceedance']:.3f}; "
                  f"H2 vs ay at apex r = {man['h2_vs_ay_apex']['correlation']:.3f} (n {man['h2_vs_ay_apex']['n']})")
        if "E" not in o:
            continue
        ed = np.degrees(o["E"])
        tag = "" if o["shipped"] else " [would not have shipped]"
        print(f"  - E on the five reference corners [deg, value at peak |beta| / signed mean]{tag}:")
        for n_, c, i0, i1, a, b in top:
            seg = ed[a:b]
            kin = np.degrees(np.mean(lr * r[a:b] / v[a:b]))
            print(f"      C{c.get('stable_corner_id')} lap {c['lap_number']}: {seg[np.argmax(np.abs(seg))]:+.2f} / "
                  f"{np.mean(seg):+.2f}  (l_r r / v {kin:+.2f})")
        x = np.abs(ed[mids])
        print(f"  - E on validation straights |beta| [deg]{tag}: median {np.median(x):.2f}, p95 "
              f"{np.percentile(x, 95):.2f}, worst {x.max():.2f}")
        sl = estimate_slip_angles(st, o["E"], ep)
        parts = []
        for name, key in (("front", "alpha_f_filt"), ("rear", "alpha_r_filt")):
            al = np.degrees(sl[key][fit_mask])
            al = al[np.isfinite(al)]
            q = np.percentile(al, [1, 99])
            parts.append(f"{name} p1 {q[0]:+.2f} .. p99 {q[1]:+.2f} (span {q[1] - q[0]:.2f})")
        print(f"  - slip angles from E on the fit population ({int(fit_mask.sum())} samples) [deg]{tag}: "
              + "; ".join(parts))

    # ---- sensitivity summaries (relative to the shared baseline A)
    print(f"\nSENSITIVITY SUMMARY ({track_label(path)}, {stem}): largest |K_seed - K_A| / K_A of the implied linear "
          "axle stiffness")
    kA = res[("A", "-")].get("K", {})
    for route in ("a", "b"):
        parts = []
        for name in ("front", "rear"):
            ch = [(abs(res[(s, route)]["K"][name] - kA[name]) / kA[name], s) for s in ("D2", "B2")
                  if name in res[(s, route)].get("K", {}) and name in kA]
            if ch:
                m, s = max(ch)
                parts.append(f"{name} {100 * m:.1f} % (seed {s})")
        print(f"  - across seeds within {ROUTE_LABELS[route]}: " + ", ".join(parts))
    parts = []
    for name in ("front", "rear"):
        ch = [(abs(res[(s, "b")]["K"][name] - res[(s, "a")]["K"][name]) / res[(s, "a")]["K"][name], s)
              for s in ("D2", "B2") if name in res[(s, "a")].get("K", {}) and name in res[(s, "b")].get("K", {})]
        if ch:
            m, s = max(ch)
            parts.append(f"{name} {100 * m:.1f} % (seed {s})")
    print("  - route (a) vs route (b), same seed, largest |K_b - K_a| / K_a: " + ", ".join(parts))

    if png:
        plots(path, stem, t, sr, res, top)


def plots(path, stem, t, sr, res, top):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from modules.tyre_model_pacejka import pacejka_lateral_force
    os.makedirs(OUT_DIR, exist_ok=True)
    title = f"{track_label(path)}, export {stem}"
    e = int(EDGE_S * sr)

    n_, c, i0, i1, a, b = top[0]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    sl = slice(max(0, i0 - e), min(t.size, i1 + e))
    tt = t[sl] - t[i0]
    for key, o in res.items():
        if "E" in o:
            lab = run_name(*key) + ("" if o["shipped"] else " (would not have shipped)")
            ax.plot(tt, np.degrees(o["E"][sl]), color=COLOURS[key], lw=1.1, label=lab,
                    ls="--" if key[1] == "a" else "-")
    ax.axvspan(t[a] - t[i0], t[b - 1] - t[i0], color="#dddddd", zorder=0)
    ax.axvline(0, color="black", lw=0.6, ls=":")
    ax.axvline(t[i1 - 1] - t[i0], color="black", lw=0.6, ls=":")
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("time from corner entry [s] (dotted: entry / exit; grey: sustained lateral run)")
    ax.set_ylabel("estimator sideslip [deg]")
    ax.set_title(f"{title}\napp estimator sideslip on corner C{c.get('stable_corner_id')} lap {c['lap_number']} "
                 "under three tyre-fit seeds", fontsize=10)
    ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"seed_sens_corner_{stem}.png"), dpi=120)
    plt.close(fig)

    for name in ("front", "rear"):
        fig, ax = plt.subplots(figsize=(7, 4.5))
        p99 = max(o["man"]["axles"][name]["visited_alpha_p99_deg"] for o in res.values() if o["man"].get("axles"))
        g = np.linspace(-max(p99, 1.0) * 1.3, max(p99, 1.0) * 1.3, 400)
        for key, o in res.items():
            a_ = o["man"].get("axles", {}).get(name)
            if a_:
                ax.plot(g, pacejka_lateral_force(np.radians(g), a_["B"], a_["C"], a_["D"], a_["E"]) / 1000.0,
                        color=COLOURS[key], lw=1.1, ls="--" if key[1] == "a" else "-", label=run_name(*key))
        ax.axvline(p99, color="#999999", lw=0.6, ls=":")
        ax.axvline(-p99, color="#999999", lw=0.6, ls=":")
        ax.axhline(0, color="black", lw=0.5)
        ax.set_xlabel(f"{name} slip angle [deg] (dotted: largest 99th-percentile slip visited by any fit)")
        ax.set_ylabel(f"{name} axle lateral force [kN]")
        ax.set_title(f"{title}\nfitted {name}-axle tyre curve under three seeds and two substitution routes",
                     fontsize=10)
        ax.legend(fontsize=6)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"seed_sens_curve_{name}_{stem}.png"), dpi=120)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    print("SEED SENSITIVITY OF THE TYRE FIT AND THE APP ESTIMATOR (ROUND 3) -- offline, numbers only")
    for path in EXPORTS:
        run_export(path, args.png)


if __name__ == "__main__":
    main()
