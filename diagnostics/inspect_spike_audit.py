# Q-SPIKE-AUDIT -- per-event accounting of every deep CS dip in one session
# (PLAN.md STATUS 2026-10-05, author work order + EXECUTE; read-only).
#
# RULES, frozen 2026-10-05 BEFORE any run (conflict = record, never re-tune).
# SESSION: Nurburgring JST outing 29 (nbr_jst_solution), both axles, best
#   available. State and masks from lp.Ctx (cached parse; time grid identical
#   to a fresh parse, Q-MU E2); raw forensic channels (az, damper travel,
#   damper loads, throttle) from a FRESH parse (NBR zero entries applied).
# PRODUCTION CHAIN run fresh once: tyre_fit_auto.fit_session_pacejka at the
#   configured modes; ENTRY E1 the fit's beta_ekf_with_fallback reproduces the
#   cached production beta (max |diff| <= 1e-5); E2 the CS recomputed from it
#   reproduces the cached CS (same finite sets, <= 1e-5). Fallback / reset
#   mask = moving samples where the EKF's raw beta differs from
#   beta_with_fallback -- in sideslip_ekf_pacejka a diverged sample both falls
#   back to the kinematic beta and RESETS the filter state, so one mask covers
#   "fallback switch" and "EKF reset".
# POPULATION: in-corner samples (corner brackets) on analysis laps, production
#   CS arrays (full -- kerb-affected samples included so check a can name
#   them).
# 1 EVENTS: per axle, stretches of CS < -0.5, gaps < 0.5 s bridged (time),
#   within one corner instance. Per event: id, lap, corner, lap-distance span,
#   minimum CS, duration. EVIDENCE WINDOW = from the earliest production
#   regression-window start (reconstruct_cs_window_start) of any event sample
#   to the event end -- what the estimator actually saw.
#   RECURS = an event of the same axle at the same corner on another analysis
#   lap whose minimum lies within +/-10 m lap distance (Q-PAIRING radius);
#   reported k of n laps.
# 2 CHECKS in order, first match decides:
#   a KERB DETECTED: the evidence window overlaps the kerb mask or its 0.20 s
#     zone (production kerb_smear_zone / near-strike mark) -- already excluded
#     from judgement.
#   b KERB MISSED (detector fires on |az - 1.0 g| > 1.2 g): in the window, a
#     ONE-SIDED burst -- damper velocity |v - LP2(v)| or wheel load
#     |Fz - LP2(Fz)| above its own in-corner p95 on the wheels of one side
#     only -- or an az deviation |az - 1 g| above its in-corner p95 (below the
#     1.2 g firing value) with no two-sided burst. Peaks vs p95 vs 1.2 g reported.
#   c ESTIMATOR ARTEFACT: a fallback / reset transition within +/-0.5 s of the
#     event, or degenerate window support at the event minimum: production
#     window slip span < 3 x the straight-line slip noise floor F_a(n)
#     (Q-CHAIR-AUDIT V-GRADE, inspect_chair_audit.span_floor); n and span
#     reported.
#   d FORCE-PATH EVENT WITHOUT KERB SIGNATURE: brake-pressure or throttle rate
#     peak above its in-corner p95, or a TWO-SIDED damper / load burst (bump).
#     Banking: not evaluable (no bank channel; the G1 map is longitudinal).
#   e UNEXPLAINED: none of the above -- named and sized, no invented story.
#   p95 references: in-corner analysis-lap samples outside the kerb mask and
#   its 0.20 s zone. LP2 = production 2 Hz zero-phase Butterworth.
# 3 SUMMARY: events per class per axle; each class's share of the CS < -0.5
#   samples; unexplained events with depths; majority check for (e);
#   cross-reference to the JST 29 Q-PAIRING sites (spike json, known_real,
#   +/-10 m).
# 4 RENDERS (plain words): one overview per axle -- CS over lap distance, all
#   analysis laps, every event marked with its verdict letter, kerb-affected
#   samples greyed; the 5 deepest events zoomed (CS, slip angle, axle Fy, az,
#   damper velocity per wheel, kerb-mask spans).
#
# Modes: run | plots | repass (specificity re-pass, rules at its section)     Output: diagnostics/plots/spike_audit/

import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp
from diagnostics.inspect_chair_audit import span_floor, floor_at, GRADE_K
from diagnostics.gated_kinematic_common import valid_corner_instances

KEY = "nbr_jst_solution"
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "spike_audit")
DEEP = -0.5
BRIDGE_S = 0.5
NEAR_S = 0.5
SITE_M = 10.0
DETECTOR_G = 1.2
AX = {"f": "front", "r": "rear"}
SIDES = {"left": ("fl", "rl"), "right": ("fr", "rr")}
WORDS = {"a": "kerb detected", "b": "kerb missed", "c": "estimator artefact", "d": "car-on-track event",
         "e": "unexplained"}


def _log(msg):
    os.makedirs(OUT_DIR, exist_ok=True)
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, "run_log.txt"), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def _runs(mask):
    e = np.diff(np.r_[0, mask.astype(np.int8), 0])
    return list(zip(np.where(e == 1)[0], np.where(e == -1)[0]))


def build():
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, _butterworth_lowpass,
                                            resolve_cs_min_window_samples, load_car_data)
    from modules.tyre_fit_auto import fit_session_pacejka
    from modules.wheel_loads import (estimate_wheel_loads_from_dampers, TRAVEL_CHANNEL, _interp_channel,
                                     _normalize_travel_to_mm)
    ctx = lp.Ctx(KEY)
    st, prm = ctx.state, ctx.params
    t, sr = st["time"], st["sample_rate_hz"]
    tf = prm["tyre_fit_auto"]
    man = fit_session_pacejka(ctx.data, prm, data_file_path=d2.sessions()[KEY]["path"],
                              load_normalised=tf.get("load_normalised_fit_enabled", False),
                              tyre_fit_mode=tf.get("tyre_fit_mode", "per_session"))
    beta = np.asarray(man["beta_ekf_with_fallback"], float)
    cached = np.asarray(ctx.cache["prod"]["beta"], float)
    d_beta = float(np.nanmax(np.abs(beta - cached)))
    if d_beta > 1e-5:
        raise SystemExit(f"STOP E1: production beta differs from the cache by {d_beta:.2e}")
    moving = st["moving_mask"]
    fallback = moving & (np.asarray(man["beta_ekf"], float) != beta)
    slip = estimate_slip_angles(st, beta, prm)
    forces = estimate_lateral_forces(st, prm)
    cs = estimate_cornering_stiffness(slip, forces, st, prm)
    for a in "fr":
        x, y = np.asarray(cs[f"CS_ratio_{a}"], float), np.asarray(ctx.cache["prod"]["down"][f"cs_{a}"], float)
        fin = np.isfinite(x) & np.isfinite(y)
        if not (np.array_equal(np.isfinite(x), np.isfinite(y)) and np.max(np.abs(x[fin] - y[fin])) <= 1e-5):
            raise SystemExit(f"STOP E2: CS {AX[a]} does not reproduce the cache")
    fresh = parse_csv(d2.sessions()[KEY]["path"])
    fc = prm["stability_estimation"]["cs_filter_cutoff_hz"]

    def hp(x):
        return np.abs(x - _butterworth_lowpass(x, fc, sr))
    vel = {}
    for c, ch in TRAVEL_CHANNEL.items():
        tr = _interp_channel(fresh["channels"], ch, t)
        if tr is None:
            continue
        tr = _normalize_travel_to_mm(tr, fresh["channels"][ch].get("unit_raw"))
        vel[c] = np.gradient(np.nan_to_num(tr), t)
    dmp = estimate_wheel_loads_from_dampers(st, fresh["channels"], prm, load_car_data())
    load = {c: np.nan_to_num(np.asarray(dmp[c]["fz_N"], float)) for c in ("fl", "fr", "rl", "rr")}
    thr = _interp_channel(fresh["channels"], "ecu_aps", t)
    brake = st.get("brake_f_bar")
    az = _interp_channel(fresh["channels"], "log_acc_z", t)
    az_dev = np.abs(np.asarray(az, float) - 1.0)
    kerb = np.asarray(st["kerb_mask"], bool)
    zone = np.asarray(st["kerb_smear_zone"], bool)
    inst = valid_corner_instances({"laps": ctx.data["laps"], "corners": ctx.data["corners"]}, t, st["s_m"])
    incorner = np.zeros(t.size, bool)
    cid = np.full(t.size, -1)
    lapn = np.full(t.size, -1)
    for sid, lap, sl in inst:
        incorner[sl] = True
        cid[sl] = sid
        lapn[sl] = lap
    ref = incorner & ~kerb & ~zone
    sig = {"az_dev": az_dev, "vel": {c: hp(v) for c, v in vel.items()}, "load": {c: hp(v) for c, v in load.items()},
           "brake_rate": np.abs(np.gradient(np.nan_to_num(brake), t)) if brake is not None else None,
           "thr_rate": np.abs(np.gradient(np.nan_to_num(thr), t)) if thr is not None else None}
    p95 = {"az_dev": float(np.percentile(az_dev[ref], 95)),
           "vel": {c: float(np.percentile(v[ref], 95)) for c, v in sig["vel"].items()},
           "load": {c: float(np.percentile(v[ref], 95)) for c, v in sig["load"].items()},
           "brake_rate": float(np.percentile(sig["brake_rate"][ref], 95)) if brake is not None else None,
           "thr_rate": float(np.percentile(sig["thr_rate"][ref], 95)) if thr is not None else None}
    straight = ctx.base & ctx.s0
    floors = {a: span_floor(np.asarray(slip[f"alpha_{a}_filt"], float), straight) for a in "fr"}
    return dict(ctx=ctx, st=st, prm=prm, t=t, sr=sr, cs=cs, slip=slip, forces=forces, fallback=fallback,
                kerb=kerb, zone=zone, incorner=incorner, cid=cid, lapn=lapn, sig=sig, p95=p95, floors=floors,
                vel=vel, min_window=resolve_cs_min_window_samples(prm, sr), d_beta=d_beta,
                n_laps=len(ctx.analysis_laps()))


def events(B, a):
    from modules.stability_analysis import reconstruct_cs_window_start
    t, st, prm = B["t"], B["st"], B["prm"]
    c = np.asarray(B["cs"][f"CS_ratio_{a}"], float)
    deep = B["incorner"] & np.isfinite(c) & (c < DEEP)
    out = []
    for lo, hi in _runs(deep):
        if out and out[-1]["cid"] == B["cid"][lo] and out[-1]["lap"] == B["lapn"][lo] \
                and t[lo] - t[out[-1]["hi"] - 1] < BRIDGE_S:
            out[-1]["hi"] = int(hi)
            continue
        out.append({"lo": int(lo), "hi": int(hi), "cid": int(B["cid"][lo]), "lap": int(B["lapn"][lo])})
    se = prm["stability_estimation"]
    alpha = np.asarray(B["slip"][f"alpha_{a}_filt"], float)

    def wstart(i):
        return reconstruct_cs_window_start(alpha, int(i), B["min_window"], se["cs_min_slip_angle_span_rad"],
                                           st["s_m"], se["cs_max_window_m"])
    for k, e in enumerate(out):
        idx = np.arange(e["lo"], e["hi"])
        idx_d = idx[deep[idx]]
        imin = int(idx_d[np.argmin(c[idx_d])])
        s0w = wstart(imin)
        e.update(id=f"{a.upper()}{k + 1:02d}", axle=a, w0=int(min(wstart(i) for i in idx_d)), imin=imin,
                 min_cs=float(c[imin]), s_lo=float(st["s_m"][e["lo"]]), s_hi=float(st["s_m"][e["hi"] - 1]),
                 s_min=float(st["s_m"][imin]), dur_s=float(t[e["hi"] - 1] - t[e["lo"]]), n_deep=int(idx_d.size),
                 win_n=int(imin - s0w), win_span=float(np.ptp(alpha[s0w:imin])) if imin > s0w else 0.0)
    return out


def classify(B, e):
    t, sig, p95 = B["t"], B["sig"], B["p95"]
    w = slice(e["w0"], e["hi"])
    ev = {}
    if (B["kerb"][w] | B["zone"][w]).any():
        ev["kerb_samples_in_window"] = int(B["kerb"][w].sum())
        ev["zone_samples_in_window"] = int(B["zone"][w].sum())
        return "a", ev
    side_burst = {}
    for side, wheels in SIDES.items():
        hits = []
        for c in wheels:
            for name in ("vel", "load"):
                if c in sig[name] and sig[name][c][w].max() > p95[name][c]:
                    hits.append(f"{c} {name} {sig[name][c][w].max():.0f} > p95 {p95[name][c]:.0f}")
        side_burst[side] = hits
    one_sided = bool(side_burst["left"]) != bool(side_burst["right"])
    two_sided = bool(side_burst["left"]) and bool(side_burst["right"])
    az_peak = float(sig["az_dev"][w].max())
    ev.update(az_dev_peak_g=az_peak, az_p95_g=p95["az_dev"], bursts=side_burst)
    if one_sided or (az_peak > p95["az_dev"] and not two_sided):
        ev["note"] = f"az dev {az_peak:.2f} g (p95 {p95['az_dev']:.2f}, detector {DETECTOR_G})"
        return "b", ev
    near = (t >= t[e["lo"]] - NEAR_S) & (t <= t[e["hi"] - 1] + NEAR_S)
    fb = B["fallback"][near]
    trans = int(np.sum(np.diff(fb.astype(np.int8)) != 0)) + int(fb[0]) if fb.size else 0
    floor = float(floor_at(B["floors"][e["axle"]], max(e["win_n"], 10)))
    ev.update(fallback_transitions=trans, fallback_samples=int(fb.sum()), win_n=e["win_n"],
              win_span_deg=float(np.degrees(e["win_span"])), floor_deg=float(np.degrees(floor)))
    if fb.any() or e["win_span"] < GRADE_K * floor:
        return "c", ev
    force = []
    for name in ("brake_rate", "thr_rate"):
        if sig[name] is not None and sig[name][w].max() > p95[name]:
            force.append(f"{name} {sig[name][w].max():.1f} > p95 {p95[name]:.1f}")
    if two_sided:
        force.append("two-sided burst (bump): " + "; ".join(side_burst["left"] + side_burst["right"]))
    ev["force"] = force
    if force:
        return "d", ev
    return "e", ev


def _line(e):
    ev = e["evidence"]
    v = e["verdict"]
    if v == "a":
        return f"kerb mask {ev['kerb_samples_in_window']} / zone {ev['zone_samples_in_window']} samples in the window"
    if v == "b":
        return f"{ev['note']}; bursts left {ev['bursts']['left'] or '-'}, right {ev['bursts']['right'] or '-'}"
    if v == "c":
        return (f"fallback samples +-0.5 s {ev['fallback_samples']} ({ev['fallback_transitions']} transitions); "
                f"window n {ev['win_n']}, slip span {ev['win_span_deg']:.3f} deg vs 3 x floor {3 * ev['floor_deg']:.3f}")
    if v == "d":
        return "; ".join(ev["force"])
    return (f"no kerb, no signature (az {ev['az_dev_peak_g']:.2f} g vs p95 {ev['az_p95_g']:.2f}), no fallback, "
            f"window n {ev['win_n']} span {ev['win_span_deg']:.3f} deg >= 3 x floor {3 * ev['floor_deg']:.3f}, "
            "no force-path peak")


def run():
    t0 = time.perf_counter()
    B = build()
    _log(f"entry OK: beta max diff {B['d_beta']:.2e}; CS reproduced; in-corner fallback samples "
         f"{int((B['fallback'] & B['incorner']).sum())}; p95 {json.dumps(B['p95'], default=float)}")
    with open(os.path.join(os.path.dirname(OUT_DIR), "stiffness_cs", f"spike_{KEY}.json")) as fh:
        prod = json.load(fh)["variants"]["production"]
    sites = {a: [d["pos_m"] for d in prod[a]["dips"] if d.get("known_real")] for a in "fr"}
    allev = {}
    for a in "fr":
        evs = events(B, a)
        for e in evs:
            e["verdict"], e["evidence"] = classify(B, e)
        for e in evs:
            same = {x["lap"] for x in evs if x["cid"] == e["cid"] and x["lap"] != e["lap"]
                    and abs(x["s_min"] - e["s_min"]) <= SITE_M}
            e["recurs_laps"] = len(same) + 1
            e["site"] = next((p for p in sites[a] if e["s_lo"] - SITE_M <= p <= e["s_hi"] + SITE_M), None)
        allev[a] = evs
    L = [f"Q-SPIKE-AUDIT {lp.title_of(KEY)}: {B['n_laps']} analysis laps, events = CS < {DEEP} in corners, "
         f"gaps < {BRIDGE_S} s bridged"]
    out = {"key": KEY, "events": {}, "summary": {}}
    for a in "fr":
        evs = allev[a]
        n_deep = sum(e["n_deep"] for e in evs)
        L.append(f"\n{AX[a].upper()} AXLE: {len(evs)} events, {n_deep} deep samples")
        for e in evs:
            L.append(f"  {e['id']} lap {e['lap']} C{e['cid']} {e['s_lo']:.0f}-{e['s_hi']:.0f} m, min {e['min_cs']:+.2f}, "
                     f"{e['dur_s']:.2f} s, recurs {e['recurs_laps']}/{B['n_laps']} laps"
                     + (f", Q-PAIRING site {e['site']:.0f} m" if e["site"] is not None else "")
                     + f" -> ({e['verdict']}) {WORDS[e['verdict']]}: {_line(e)}")
        summ = {}
        for v in "abcde":
            sel = [e for e in evs if e["verdict"] == v]
            summ[v] = {"events": len(sel), "deep_samples": sum(e["n_deep"] for e in sel),
                       "share": sum(e["n_deep"] for e in sel) / max(1, n_deep),
                       "depths": sorted(round(e["min_cs"], 2) for e in sel)}
        L.append("  SUMMARY: " + "; ".join(f"({v}) {WORDS[v]} {summ[v]['events']} events, "
                                            f"{summ[v]['share'] * 100:.0f} % of deep samples" for v in "abcde"))
        ue = summ["e"]
        L.append(f"  UNEXPLAINED: {ue['events']} events, depths {ue['depths']}"
                 + ("  *** MAJORITY OF DEEP EVENTS -- a finding about the tail itself ***"
                    if ue["events"] > len(evs) / 2 else ""))
        hit = sorted({round(e["site"]) for e in evs if e["site"] is not None})
        L.append(f"  Q-PAIRING sites ({len(sites[a])}): with an event {hit}; without "
                 f"{sorted(round(p) for p in sites[a] if round(p) not in hit)}")
        out["summary"][a] = summ
        out["events"][a] = evs
    txt = "\n".join(L)
    print(txt)
    with open(os.path.join(OUT_DIR, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    with open(os.path.join(OUT_DIR, "events.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)
    np.savez_compressed(os.path.join(OUT_DIR, "arrays.npz"), t=B["t"], s_m=B["st"]["s_m"],
                        cs_f=B["cs"]["CS_ratio_f"], cs_r=B["cs"]["CS_ratio_r"],
                        alpha_f=B["slip"]["alpha_f_filt"], alpha_r=B["slip"]["alpha_r_filt"],
                        fy_f=B["forces"]["Fy_f_filt"], fy_r=B["forces"]["Fy_r_filt"], az_dev=B["sig"]["az_dev"],
                        kerb=B["kerb"], zone=B["zone"], incorner=B["incorner"], lapn=B["lapn"],
                        **{f"vel_{c}": v for c, v in B["vel"].items()})
    _log(f"done, {time.perf_counter() - t0:.0f} s")


def plots():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from core.plot_style import NEAR_STRIKE_COLOR, LAP_PALETTE, THRESHOLD_GREY
    from ui.style import BAD
    z = np.load(os.path.join(OUT_DIR, "arrays.npz"))
    with open(os.path.join(OUT_DIR, "events.json")) as fh:
        E = json.load(fh)
    title = lp.title_of(KEY)
    legend_words = "; ".join(f"{k} = {v}" for k, v in WORDS.items())
    aff = z["kerb"] | z["zone"]
    for a in "fr":
        cs = z[f"cs_{a}"]
        fig, ax = plt.subplots(figsize=(16, 6))
        laps = sorted({int(x) for x in z["lapn"][z["incorner"]]})
        n_aff = n_fin = 0
        for j, ln in enumerate(laps):
            y = np.where((z["lapn"] == ln) & z["incorner"], cs, np.nan)
            n_fin += int(np.isfinite(y).sum())
            n_aff += int((np.isfinite(y) & aff).sum())
            ax.plot(z["s_m"], np.where(aff, np.nan, y), color=LAP_PALETTE[j % len(LAP_PALETTE)], lw=0.9,
                    label=f"lap {ln}")
            ax.plot(z["s_m"], np.where(aff, y, np.nan), color=NEAR_STRIKE_COLOR, lw=0.9)
        for e in E["events"][a]:
            ax.annotate(e["verdict"], (e["s_min"], e["min_cs"]), color=BAD, fontsize=10, weight="bold",
                        ha="center", va="top")
        ax.axhline(DEEP, color=THRESHOLD_GREY, lw=0.8, ls="--", label="deep-dip line -0.5")
        ax.axhline(0, color=THRESHOLD_GREY, lw=0.6)
        ax.set_ylim(-3, 1.1)
        ax.set_xlabel("lap distance [m]")
        ax.set_ylabel(f"cornering stiffness ratio, {AX[a]}")
        ax.grid(alpha=0.25)
        ax.legend(loc="lower right", fontsize=7, ncol=2)
        fig.suptitle(f"{title}: {AX[a]} cornering stiffness ratio in corners, all analysis laps\n"
                     f"every dip below -0.5 marked with its cause letter ({legend_words})", fontsize=10)
        fig.tight_layout(rect=(0, 0.04, 1, 1))
        fig.text(0.5, 0.01, f"Kerb-affected samples greyed (n = {n_aff}, {100 * n_aff / max(1, n_fin):.1f} %); "
                 "retained in all computed results.", ha="center", fontsize=9)
        fig.savefig(os.path.join(OUT_DIR, f"overview_{a}.png"), dpi=120)
        plt.close(fig)
    deepest = sorted(E["events"]["f"] + E["events"]["r"], key=lambda e: e["min_cs"])[:5]
    for rank, e in enumerate(deepest, 1):
        a = e["axle"]
        lo, hi = max(0, e["w0"] - 100), min(z["t"].size, e["hi"] + 100)
        x = z["s_m"][lo:hi]
        fig, axs = plt.subplots(6, 1, figsize=(11, 12), sharex=True)
        panels = [(z[f"cs_{a}"], "cornering stiffness ratio"), (np.degrees(z[f"alpha_{a}"]), "slip angle [deg]"),
                  (z[f"fy_{a}"] / 1000, "axle lateral force [kN]"), (z["az_dev"], "|vertical acc. - 1 g| [g]")]
        for i, (y, lab) in enumerate(panels):
            axs[i].plot(x, y[lo:hi], color=LAP_PALETTE[0], lw=1.1)
            axs[i].set_ylabel(lab, fontsize=8)
        axs[3].axhline(DETECTOR_G, color=THRESHOLD_GREY, ls="--", lw=0.8, label="kerb detector fires above")
        axs[3].legend(fontsize=7)
        for j, c in enumerate(("fl", "fr", "rl", "rr")):
            if f"vel_{c}" in z:
                axs[4].plot(x, z[f"vel_{c}"][lo:hi], lw=0.9, color=LAP_PALETTE[j % len(LAP_PALETTE)],
                            label=c.upper())
        axs[4].set_ylabel("damper velocity [mm/s]", fontsize=8)
        axs[4].legend(fontsize=7, ncol=4)
        axs[5].fill_between(x, 0, z["kerb"][lo:hi].astype(float), color=NEAR_STRIKE_COLOR, alpha=0.5,
                            label="kerb mask")
        axs[5].fill_between(x, 0, z["zone"][lo:hi].astype(float) * 0.5, color=NEAR_STRIKE_COLOR, alpha=0.25,
                            label="0.2 s kerb zone")
        axs[5].set_ylabel("kerb mask", fontsize=8)
        axs[5].legend(fontsize=7)
        for axx in axs:
            axx.axvspan(z["s_m"][e["w0"]], z["s_m"][e["hi"] - 1], color=THRESHOLD_GREY, alpha=0.12)
            axx.grid(alpha=0.25)
        axs[-1].set_xlabel("lap distance [m]")
        fig.suptitle(f"{title}: dip {e['id']} ({AX[a]}, lap {e['lap']}, corner {e['cid']}, minimum {e['min_cs']:+.2f}) "
                     f"-- cause ({e['verdict']}) {WORDS[e['verdict']]}; shaded = what the estimator saw", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, f"deep_{rank}_{e['id']}.png"), dpi=110)
        plt.close(fig)


# ---------------------------------------------------------------- re-pass
# SPECIFICITY RE-PASS (author EXECUTE 2026-10-05; frozen before classifying).
# Same 58 events and evidence windows (events.json). Per event, N_MATCH random
# windows of the SAME length lying wholly in clean in-corner analysis-lap
# samples (outside the kerb mask and its 0.20 s zone), fixed seed. For every
# statistic, the event's threshold = the p95 of that statistic over its
# matched windows, so a signature counts only where fewer than 5 % of ordinary
# windows of that length show it. Order, first match decides:
#   a KERB DETECTED: unchanged (mask / zone overlap is a definition).
#   b KERB MISSED: a ONE-SIDED burst -- max damper-velocity or wheel-load high
#     band of a wheel above its matched p95 on the wheels of one side only; az
#     (max |az - 1 g| above its matched p95) counts only as a WITNESS reported
#     beside such a burst, never alone.
#   c ESTIMATOR ARTEFACT: the count of fallback / reset samples INSIDE the
#     event's regression window [window start, event end) above its matched
#     p95; or V-GRADE degenerate support (unchanged; its base rate on clean
#     in-corner CS samples reported).
#   d CAR-ON-TRACK EVENT: brake-rate max above its matched p95, or a two-sided
#     burst (both sides above their matched p95).
#   r RESIDUAL: no specific mechanism shown (replaces e).
# Output: old vs new label per event, class shares, per surviving (b) event the
# discriminating channels and their level vs the matched p95, and the measured
# base rate of each final criterion (the share of matched windows that would
# themselves be labelled b / c / d under the same thresholds).
# TIGHTENED (b), ONE FINAL ITERATION (author EXECUTE 2026-10-05, frozen before
# re-labelling; mode repass2): the (b) per-channel cut is the EMPIRICAL UNION-5 %
# LEVEL -- per event, one common percentile q of the matched-window maxima for all
# channel-wheel tests, the smallest q on a 0.05-step grid from 95 at which the
# share of matched windows passing the one-sided union is <= 5 %. Same seed, so
# the same matched windows as the re-pass. (a), (c), (d) cuts unchanged (the (d)
# two-sided test stays at the matched p95). Every event re-runs the cascade;
# moves against the re-pass labels are listed. Pre-registered: survivors = the
# detector test set; none = "no kerb-missed event is demonstrable at 5 % on this
# file". No further tightening; the labels freeze.
N_MATCH = 2000
SEED = 20261005
WORDS_RE = {"a": "kerb detected", "b": "kerb missed", "c": "estimator artefact", "d": "car-on-track event",
            "r": "no specific mechanism shown"}


def _rolling(x, n, how):
    import pandas as pd
    s = pd.Series(np.asarray(x, float))
    r = s.rolling(n).max() if how == "max" else s.rolling(n).sum()
    return r.to_numpy()


def repass(tighten=False):
    t0 = time.perf_counter()
    prev = {}
    if tighten:
        with open(os.path.join(OUT_DIR, "repass.json")) as fh:
            prev = {r["id"]: r["new"] for r in json.load(fh)["rows"]}
    B = build()
    with open(os.path.join(OUT_DIR, "events.json")) as fh:
        E = json.load(fh)
    sig, t = B["sig"], B["t"]
    clean = B["incorner"] & ~B["kerb"] & ~B["zone"]
    fb = B["fallback"].astype(float)
    rng = np.random.default_rng(SEED)
    # V-GRADE base rate on clean in-corner finite CS samples, per axle
    from modules.stability_analysis import reconstruct_cs_window_start
    se = B["prm"]["stability_estimation"]
    vg_rate = {}
    for a in "fr":
        c = np.asarray(B["cs"][f"CS_ratio_{a}"], float)
        alpha = np.asarray(B["slip"][f"alpha_{a}_filt"], float)
        idx = np.where(clean & np.isfinite(c))[0]
        idx = rng.choice(idx, size=min(3000, idx.size), replace=False)
        thin = 0
        for i in idx:
            s0 = reconstruct_cs_window_start(alpha, int(i), B["min_window"], se["cs_min_slip_angle_span_rad"],
                                             B["st"]["s_m"], se["cs_max_window_m"])
            n, span = int(i - s0), float(np.ptp(alpha[s0:i])) if i > s0 else 0.0
            thin += span < GRADE_K * float(floor_at(B["floors"][a], max(n, 10)))
        vg_rate[a] = thin / idx.size
    chans = [(name, c) for name in ("vel", "load") for c in sorted(sig[name])]
    rows, rates = [], {"b": [], "c": [], "d": []}
    for a in "fr":
        for e in E["events"][a]:
            L = int(e["hi"] - e["w0"])
            ok = _rolling((~clean).astype(float), L, "sum") == 0
            ends = np.where(ok)[0]
            ends = ends[ends >= L - 1]
            pick = rng.choice(ends, size=min(N_MATCH, ends.size), replace=False)
            w = slice(e["w0"], e["hi"])
            stat, ref = {}, {}
            for name, c in chans:
                r = _rolling(sig[name][c], L, "max")
                stat[(name, c)] = float(sig[name][c][w].max())
                ref[(name, c)] = r[pick]
            for name in ("az_dev", "brake_rate"):
                if sig[name] is None:
                    continue
                stat[name] = float(sig[name][w].max())
                ref[name] = _rolling(sig[name], L, "max")[pick]
            stat["fb"] = float(fb[w].sum())
            ref["fb"] = _rolling(fb, L, "sum")[pick]
            p95 = {k: float(np.percentile(v, 95)) for k, v in ref.items()}
            thr_b, q_b = {k: p95[k] for k in chans}, 95.0
            if tighten:
                for q in np.arange(95.0, 99.96, 0.05):
                    thr = {k: float(np.percentile(ref[k], q)) for k in chans}
                    hit = {sd: np.zeros(pick.size, bool) for sd in SIDES}
                    for sd, wh in SIDES.items():
                        for (n, c) in chans:
                            if c in wh:
                                hit[sd] |= ref[(n, c)] > thr[(n, c)]
                    if np.mean(hit["left"] != hit["right"]) <= 0.05:
                        thr_b, q_b = thr, float(q)
                        break

            def label(get, has_kerb, vgrade=False):
                if has_kerb:
                    return "a", {}
                side = {sd: [(n, c) for (n, c) in chans if c in wh and get((n, c)) > thr_b[(n, c)]]
                        for sd, wh in SIDES.items()}
                side95 = {sd: [(n, c) for (n, c) in chans if c in wh and get((n, c)) > p95[(n, c)]]
                          for sd, wh in SIDES.items()}
                one = bool(side["left"]) != bool(side["right"])
                two = bool(side95["left"]) and bool(side95["right"])
                if one:
                    return "b", side
                if get("fb") > p95["fb"] or vgrade:
                    return "c", side
                if ("brake_rate" in p95 and get("brake_rate") > p95["brake_rate"]) or two:
                    return "d", side
                return "r", side
            kerb_e = bool((B["kerb"][w] | B["zone"][w]).any())
            floor = float(floor_at(B["floors"][a], max(e["win_n"], 10)))
            vg_e = e["win_span"] < GRADE_K * floor
            new, side = label(lambda k: stat[k], kerb_e, vg_e)
            vg = new == "c" and vg_e and not stat["fb"] > p95["fb"]
            for key in ("b", "c", "d"):
                hits = 0
                for j in range(pick.size):
                    lab, _ = label(lambda k: ref[k][j], False)
                    hits += lab == key
                rates[key].append(hits / pick.size)
            disc = []
            if new == "b":
                for sd, lst in side.items():
                    for (n, c) in lst:
                        disc.append(f"{c.upper()} {'damper velocity' if n == 'vel' else 'wheel-load transient'} "
                                    f"{stat[(n, c)]:.0f} vs matched p{q_b:.2f} {thr_b[(n, c)]:.0f} "
                                    f"(x{stat[(n, c)] / max(thr_b[(n, c)], 1e-9):.2f})")
                if stat["az_dev"] > p95["az_dev"]:
                    disc.append(f"az witness {stat['az_dev']:.2f} g vs matched p95 {p95['az_dev']:.2f}")
            rows.append({"id": e["id"], "axle": a, "lap": e["lap"], "cid": e["cid"], "s_min": e["s_min"],
                         "min_cs": e["min_cs"], "n_deep": e["n_deep"],
                         "old": prev.get(e["id"], e["verdict"]) if tighten else e["verdict"], "new": new,
                         "q_b": q_b,
                         "vgrade": vg, "fb_in_window": stat["fb"], "fb_p95": p95["fb"], "disc": disc,
                         "site": e.get("site"), "L": L, "n_match": int(pick.size)})
    L_ = [f"Q-SPIKE-AUDIT RE-PASS{' (b TIGHTENED, union-5 %)' if tighten else ''} {lp.title_of(KEY)}: "
          f"specificity-checked criteria, {N_MATCH} matched clean corner windows per event, cut = matched p95"
          + ("; (b) per-channel cut = empirical union-5 % level, per-event q from "
             f"{min(r['q_b'] for r in rows):.2f} to {max(r['q_b'] for r in rows):.2f}" if tighten else "")]
    for a in "fr":
        rs = [r for r in rows if r["axle"] == a]
        nd = sum(r["n_deep"] for r in rs)
        L_.append(f"\n{AX[a].upper()} AXLE ({len(rs)} events):")
        for r in rs:
            extra = ""
            if r["new"] == "b":
                extra = " | " + "; ".join(r["disc"])
            elif r["new"] == "c":
                extra = (" | V-GRADE degenerate support" if r["vgrade"] else
                         f" | {r['fb_in_window']:.0f} reset samples in the window vs matched p95 {r['fb_p95']:.0f}")
            L_.append(f"  {r['id']} lap {r['lap']} C{r['cid']} {r['s_min']:.0f} m, min {r['min_cs']:+.2f}: "
                      f"({r['old']}) -> ({r['new']}) {WORDS_RE[r['new']]}{extra}"
                      + (f" [site {r['site']:.0f} m]" if r["site"] is not None else ""))
        L_.append("  CLASSES: " + "; ".join(
            f"({k}) {WORDS_RE[k]} {sum(r['new'] == k for r in rs)} events, "
            f"{100 * sum(r['n_deep'] for r in rs if r['new'] == k) / max(1, nd):.0f} % of deep samples" for k in "abcdr"))
    L_.append("\nBASE RATES of the final criteria (share of matched clean windows that would themselves be labelled, "
              "mean over events): " + "; ".join(f"{k} {100 * np.mean(v):.1f} %" for k, v in rates.items()))
    L_.append("V-GRADE base rate on clean in-corner CS samples: " + "; ".join(
        f"{AX[a]} {100 * v:.1f} %" for a, v in vg_rate.items()))
    txt = "\n".join(L_)
    print(txt)
    tag = "repass2" if tighten else "repass"
    with open(os.path.join(OUT_DIR, f"{tag}_report.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    with open(os.path.join(OUT_DIR, f"{tag}.json"), "w") as fh:
        json.dump({"rows": rows, "base_rates": {k: float(np.mean(v)) for k, v in rates.items()},
                   "vgrade_base_rate": vg_rate}, fh, indent=1, default=float)
    _log(f"re-pass done, {time.perf_counter() - t0:.0f} s")

if __name__ == "__main__":
    {"run": run, "plots": plots, "repass": repass, "repass2": lambda: repass(tighten=True)}[sys.argv[1] if len(sys.argv) > 1 else "run"]()
