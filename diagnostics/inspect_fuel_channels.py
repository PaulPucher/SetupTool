# Q2b fuel-channel content check (PLAN.md PHASE 1 PLAN Q2b; read-only).
#
# Question: do the logged fuel channels give a measured within-outing mass
# trace, and does anything in the files carry the per-outing starting load?
# Premise (author, 2026-10-02): total fuel level is NOT logged;
# log_fuellevel_catch is the ~10 L catch tank; the level-named channels are
# expected to be catch-tank / sensor plumbing -- verified as such, not mined
# for a level. vol_fuel_rem_telem (5 files) is inspected as a possible ECU
# "remaining fuel" = assumed start minus consumption.
#
# Checks, fixed before the run:
#   used families (team / ecu / flow meter): consumed mass per file, summed
#     over reset-free segments (segment max - min); litres -> kg with the
#     file's own log_c_fueldensity; pairwise agreement; lap-sum plausibility
#     (sum of Team_Fuel_Used_per_Lap step values vs the team total).
#   level-named channels: share of moving samples within 2 % of the channel
#     range from its maximum (catch-tank "stays full"); Spearman against
#     cumulative used; change across stationary plateaus (ecu_speed < 3 kph
#     for >= 20 s); slosh = std of (value - 5 s rolling median) at |a_y| > 1 g
#     vs straights (|a_y| < 0.15 g, speed > 100 kph).
#   vol_fuel_rem_telem: start value per outing (first moving sample) and the
#     slope of remaining vs used (a computed "start - used" reads -1 in
#     litres).
# Outputs diagnostics/plots/fuel_q2b/ (q2b.txt, q2b.json, <key>.png).

import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks

OUT_DIR = "diagnostics/plots/fuel_q2b"
USED_L = ("Team_Fuel_Used", "ecu_fuelcons")
USED_KG = ("Team_Fuel_Used_Stint", "ecu_fuelcons_kg", "ffm_mass_total", "ffm_mass_total.mass", "DTM_ffm_mass_total")
LAP_KG = "Team_Fuel_Used_per_Lap"
LEVEL = ("lvlfuel_lvl_raw", "log_fuellevel_catch", "vol_fuel_rem_telem", "lvlfuel_lvl Voltage")
AUX = ("log_c_fueldensity", "ecu_speed", "log_acc_y")
SPEED_TO_KPH = {"kph": 1.0, "km/h": 1.0, "mph": 1.609344, "m/s": 3.6}
G_TO = {"G": 1.0, "g": 1.0, "m/s/s": 1 / 9.81, "m/s^2": 1 / 9.81}
VOLT_TO_V = {"V": 1.0, "mV": 1e-3}


def _unit(h):
    m = re.search(r"\[(.*)\]$", h)
    return m.group(1) if m else ""


def _consumed(v):
    """Sum of (max - min) over segments split at resets (drops > 20 % of range)."""
    if v.size < 2:
        return float("nan"), 0
    rng = np.nanmax(v) - np.nanmin(v)
    cut = np.where(np.diff(v) < -0.2 * rng)[0] if rng > 0 else np.array([], int)
    total, start = 0.0, 0
    for c in list(cut) + [v.size - 1]:
        seg = v[start:c + 1]
        if seg.size:
            total += float(np.nanmax(seg) - np.nanmin(seg))
        start = c + 1
    return total, int(cut.size)


def _rolling_median(x, n):
    from scipy.ndimage import median_filter
    return median_filter(x, size=max(3, n), mode="nearest")


def check(key):
    path = d2.sessions()[key]["path"]
    B = read_raw_blocks(path, list(USED_L + USED_KG + (LAP_KG,) + LEVEL + AUX))
    rec = {"key": key, "present": sorted(B), "units": {n: _unit(h) for n, (_, _, h) in B.items()}}
    dens = float(np.median(B["log_c_fueldensity"][1])) if "log_c_fueldensity" in B else float("nan")
    rec["fuel_density"] = dens
    ts, vs, hs = B["ecu_speed"]
    t = ts
    spd = vs * SPEED_TO_KPH.get(_unit(hs), 1.0)
    moving = spd > 3.0
    ay = np.interp(t, B["log_acc_y"][0], B["log_acc_y"][1]) * G_TO.get(_unit(B["log_acc_y"][2]), 1.0)

    used = {}
    for n in USED_L + USED_KG:
        if n in B:
            tt, vv, h = B[n]
            c, resets = _consumed(vv)
            kg = c * dens if _unit(h) in ("ltr", "l", "L") else c
            used[n] = {"unit": _unit(h), "rate_hz": float(1 / np.median(np.diff(tt))), "first": float(vv[0]),
                       "last": float(vv[-1]), "max": float(np.max(vv)), "resets": resets,
                       "consumed_raw": c, "consumed_kg": kg,
                       "nondecreasing_frac": float(np.mean(np.diff(vv) >= 0))}
    rec["used"] = used
    if LAP_KG in B:
        lv = B[LAP_KG][1]
        steps = lv[1:][np.diff(lv) != 0]
        rec["lap_kg"] = {"n_steps": int(steps.size), "sum_steps_kg": float(np.sum(steps[steps > 0])),
                         "median_step_kg": float(np.median(steps[steps > 0])) if np.any(steps > 0) else float("nan")}
    kgs = {n: u["consumed_kg"] for n, u in used.items() if np.isfinite(u["consumed_kg"]) and u["consumed_kg"] > 0}
    rec["agreement_vs_team_stint"] = ({n: v / kgs["Team_Fuel_Used_Stint"] for n, v in kgs.items()}
                                      if "Team_Fuel_Used_Stint" in kgs else None)

    # first used-channel that actually moves; the flow meter reads 0 on every file
    ref_name = next((n for n in ("ffm_mass_total", "ffm_mass_total.mass", "ecu_fuelcons_kg", "Team_Fuel_Used_Stint")
                     if n in B and used.get(n, {}).get("consumed_kg", 0) > 0), None)
    cum = np.interp(t, B[ref_name][0], B[ref_name][1]) if ref_name else None
    # stationary plateaus
    runs, i = [], 0
    still = ~moving
    while i < still.size:
        if still[i]:
            j = i
            while j < still.size and still[j]:
                j += 1
            if t[j - 1] - t[i] >= 20.0:
                runs.append((i, j - 1))
            i = j
        else:
            i += 1
    from scipy.stats import spearmanr
    level = {}
    for n in LEVEL:
        if n not in B:
            continue
        tt, vv, h = B[n]
        u = _unit(h)
        v = np.interp(t, tt, vv) * (VOLT_TO_V.get(u, 1.0) if "Voltage" in n else 1.0)
        vm = v[moving]
        rng = float(np.nanmax(vm) - np.nanmin(vm)) if vm.size else float("nan")
        near_full = float(np.mean(vm >= np.nanmax(vm) - 0.02 * rng)) if rng > 0 else 1.0
        rho = float(spearmanr(v[moving], cum[moving]).correlation) if cum is not None and rng > 0 else float("nan")
        sr = 1 / np.median(np.diff(t))
        resid = v - _rolling_median(v, int(5 * sr))
        corner = moving & (np.abs(ay) > 1.0)
        straight = moving & (np.abs(ay) < 0.15) & (spd > 100)
        plate = [float(v[b] - v[a]) for a, b in runs]
        first_mov = int(np.argmax(moving)) if moving.any() else 0
        level[n] = {"unit": u, "rate_hz": float(1 / np.median(np.diff(tt))), "distinct": int(np.unique(vv).size),
                    "min": float(np.nanmin(vv)), "max": float(np.nanmax(vv)), "start_moving": float(v[first_mov]),
                    "end": float(v[-1]), "range_moving": rng, "near_full_frac": near_full,
                    "spearman_vs_cum_used": rho, "plateau_changes": plate,
                    "slosh_std_corner": float(np.std(resid[corner])) if corner.any() else float("nan"),
                    "slosh_std_straight": float(np.std(resid[straight])) if straight.any() else float("nan")}
        if n == "vol_fuel_rem_telem" and cum is not None:
            used_l = cum / dens if (ref_name and _unit(B[ref_name][2]) == "kg" and dens > 0) else cum
            mm = moving & np.isfinite(v) & np.isfinite(used_l)
            if mm.sum() > 100:
                A = np.vstack([used_l[mm], np.ones(mm.sum())]).T
                (s, c0), *_ = np.linalg.lstsq(A, v[mm], rcond=None)
                level[n]["slope_vs_used_l"] = float(s)
                level[n]["intercept_l"] = float(c0)
                level[n]["fit_rms_l"] = float(np.sqrt(np.mean((v[mm] - (s * used_l[mm] + c0)) ** 2)))
    rec["level"] = level
    rec["reference_used_channel"] = ref_name
    rec["plateaus"] = len(runs)
    _render(key, t, spd, B, dens, os.path.join(OUT_DIR, f"{key}.png"))
    return rec


def _render(key, t, spd, B, dens, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from diagnostics.trace_render import INK, BAND
    fig, ax = plt.subplots(3, 1, figsize=(14, 8), sharex=True, constrained_layout=True)
    for n in USED_L + USED_KG:
        if n in B:
            tt, vv, h = B[n]
            ax[0].plot(tt, vv * (dens if _unit(h) in ("ltr", "l", "L") else 1.0), lw=0.8, label=f"{n} [kg]")
    ax[0].set_ylabel("fuel used [kg]", color=INK)
    ax[0].legend(fontsize=7, frameon=False, loc="upper left")
    for n in LEVEL:
        if n in B:
            tt, vv, h = B[n]
            r = np.nanmax(vv) - np.nanmin(vv)
            ax[1].plot(tt, (vv - np.nanmin(vv)) / r if r > 0 else vv * 0, lw=0.8, label=f"{n} [{_unit(h)}] "
                       f"{np.nanmin(vv):.3g}..{np.nanmax(vv):.3g}")
    ax[1].set_ylabel("level-named (min-max scaled)", color=INK)
    ax[1].legend(fontsize=7, frameon=False, loc="lower left")
    ax[2].plot(t, spd, color=INK, lw=0.5)
    ax[2].set_ylabel("ecu_speed [kph]", color=INK)
    ax[2].set_xlabel("time [s]", color=INK)
    for a in ax:
        a.grid(axis="y", color=BAND, lw=0.6)
    fig.suptitle(f"Q2b fuel channels -- {key}", color=INK, fontsize=10)
    fig.savefig(out, dpi=100)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    keys = d2.usable_keys()
    out = {}
    for k in keys:
        out[k] = check(k)
        print(f"[{k}] done", flush=True)
    with open(os.path.join(OUT_DIR, "q2b.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    lines = []
    for k, r in out.items():
        u = {n: round(v["consumed_kg"], 2) for n, v in r["used"].items()}
        lines.append(f"{k}: density {r['fuel_density']}; consumed kg {u}; lap {r.get('lap_kg')}; "
                     f"agreement {({n: round(x, 3) for n, x in (r['agreement_vs_team_stint'] or {}).items()})}; "
                     f"plateaus {r['plateaus']}")
        for n, L in r["level"].items():
            lines.append(f"   {n} [{L['unit']}] {L['min']:.4g}..{L['max']:.4g} start {L['start_moving']:.4g} end "
                         f"{L['end']:.4g} near_full {L['near_full_frac']:.2f} rho_used {L['spearman_vs_cum_used']:.2f} "
                         f"slosh c/s {L['slosh_std_corner']:.3g}/{L['slosh_std_straight']:.3g} "
                         f"plateau d {[round(x, 3) for x in L['plateau_changes']]}"
                         + (f" slope {L.get('slope_vs_used_l', float('nan')):.3f} icpt {L.get('intercept_l', float('nan')):.2f}"
                            f" rms {L.get('fit_rms_l', float('nan')):.2f}" if n == "vol_fuel_rem_telem" else ""))
    with open(os.path.join(OUT_DIR, "q2b.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
