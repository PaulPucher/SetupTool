# Q-MASS -- measured per-outing mass from the damper loads (PLAN.md STATUS
# 2026-10-05, author conversions + rulings; read-only).
#
# STEP 1 RULES, frozen 2026-10-05 BEFORE any run (author EXECUTE; pass rule
# author-confirmed). Step 2 is written only if step 1 passes.
# SESSIONS: the 16 NBR files (inspect_pool_fits.NBR16) + the Paul Ricard
#   anchors v3 and pr_us_v2. Each file parsed FRESH with the current
#   parse_csv (NBR zero entries applied), parameters best available
#   (d2._params), state from the fresh channels -- the app's own path.
# ESTIMATE: production wheel_loads.estimate_session_corrected_axle_totals:
#   one least-squares fit total Fz = m g + c v^2 on moving samples with
#   |ay| < 1.5 m/s^2; intercept / g = mass_kg_session (Level 2, damper
#   loads). Nothing re-implemented.
# SPLIT HALF: the same production fit on each half of the file's fit
#   population, split at its median time (the other half's samples get v = 0,
#   which removes them from the moving set; the fit reads nothing else from
#   v outside the population). delta = first half - second half.
# FUEL (interpretation beside the rule, never inside it): the ECU cumulative
#   burn ecu_fuelcons_kg (authoritative per the Q2b rulings), resets unwrapped;
#   expected delta from burn = mean burn over the second half's fit samples -
#   the first half's. Starting fuel is unknown on every archive file (Q2b
#   ruling), so the spread across files carries up to ~+/-40 kg of real fuel
#   difference; reported, not subtracted.
# PASS RULE (author-confirmed): the recorded load uncertainty is u_V = 140-389
#   N per corner (Q-RECOVER-2), i.e. ~15-40 kg. PASS iff (a) the standard
#   deviation of mass_kg_session over the 16 NBR files <= 40 kg AND (b)
#   |delta| <= 40 kg on every file (all 18). Each also reported against the
#   strict 15 kg end. Fail on either -> Q-MASS dies, step 2 does not run.
#
# Modes: step1 [keys...] | report1     Output: diagnostics/plots/mass/

import copy
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics import inspect_last_push as lp
from diagnostics.inspect_imu_offsets import read_raw_blocks
from diagnostics.inspect_pool_fits import NBR16

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots", "mass")
ANCHORS = ("v3", "pr_us_v2")
U_HI_KG, U_LO_KG = 40.0, 15.0
G = 9.81


def _log(msg):
    os.makedirs(OUT_DIR, exist_ok=True)
    print(msg, flush=True)
    with open(os.path.join(OUT_DIR, "run_log.txt"), "a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def _burn_unwrapped(v):
    # cumulative counter with resets: add the pre-reset value back
    v = np.asarray(v, float)
    rng = np.nanmax(v) - np.nanmin(v)
    out, offset = v.copy(), 0.0
    for i in range(1, v.size):
        if rng > 0 and v[i] - v[i - 1] < -0.2 * rng:
            offset += v[i - 1]
        out[i] = v[i] + offset
    return out - out[0]


def _fit(state, damper, params):
    from modules.wheel_loads import estimate_session_corrected_axle_totals
    r = estimate_session_corrected_axle_totals(state, damper, params)
    return r["mass_kg_session"], r["c_session_N_per_mps2"], r["front_mass_fraction"]


def step1_one(k):
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import prepare_vehicle_state, load_car_data
    from modules.wheel_loads import estimate_wheel_loads_from_dampers
    t0 = time.perf_counter()
    path = d2.sessions()[k]["path"]
    data = parse_csv(path)
    prm = d2._params()
    st = prepare_vehicle_state(data["channels"], prm)
    damper = estimate_wheel_loads_from_dampers(st, data["channels"], prm, load_car_data())
    m, c, ff = _fit(st, damper, prm)
    v, ay = st["v_mps"], st["ay_mps2"]
    pop = (v >= prm["stability_estimation"]["moving_speed_min_mps"]) & (np.abs(ay) < 1.5)
    t = st["time"]
    t_mid = float(np.median(t[pop]))
    halves = {}
    for name, keep in (("first", t <= t_mid), ("second", t > t_mid)):
        sh = copy.copy(st)
        sh["v_mps"] = np.where(keep, v, 0.0)
        halves[name] = _fit(sh, damper, prm)[0]
    B = read_raw_blocks(path, ["ecu_fuelcons_kg"])
    burn = None
    if "ecu_fuelcons_kg" in B:
        tb, vb, _h = B["ecu_fuelcons_kg"]
        cum = np.interp(t, tb, _burn_unwrapped(vb))
        burn = {"total_kg": float(cum[-1]),
                "expected_delta_kg": float(np.mean(cum[pop & (t > t_mid)]) - np.mean(cum[pop & (t <= t_mid)])),
                "mean_burn_over_fit_kg": float(np.mean(cum[pop]))}
    ez = data.get("event_zero_entry") or {}
    out = {"key": k, "title": lp.title_of(k), "mass_kg": m, "c_session": c, "front_mass_fraction": ff,
           "config_mass_kg": prm["vehicle"]["mass_kg"], "n_fit": int(pop.sum()),
           "half_first_kg": halves["first"], "half_second_kg": halves["second"],
           "delta_kg": halves["first"] - halves["second"], "burn": burn,
           "zero_entry": ez.get("event"), "damper_valid_share": {
               cc: float(np.mean(np.asarray(damper[cc]["valid"])[pop])) for cc in ("fl", "fr", "rl", "rr")}}
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"step1_{k}.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=float)
    _log(f"[{k}] mass {m:.1f} kg (config {out['config_mass_kg']:.0f}), halves {halves['first']:.1f} / "
         f"{halves['second']:.1f} (delta {out['delta_kg']:+.1f}), burn {burn['total_kg'] if burn else float('nan'):.1f} kg, "
         f"{time.perf_counter() - t0:.0f} s")


def step1(keys):
    for k in keys or (NBR16 + ANCHORS):
        try:
            step1_one(k)
        except Exception as e:  # one file failing never stops the run
            _log(f"[{k}] EXCEPTION {type(e).__name__}: {e}")


def report1():
    R = {}
    for k in NBR16 + ANCHORS:
        p = os.path.join(OUT_DIR, f"step1_{k}.json")
        if os.path.exists(p):
            with open(p) as fh:
                R[k] = json.load(fh)
    L = [f"files with results: {len(R)} / {len(NBR16) + len(ANCHORS)}"]
    for k, r in R.items():
        b = r["burn"] or {}
        fc = r["delta_kg"] - b.get("expected_delta_kg", float("nan"))
        L.append(f"{r['title']}: mass {r['mass_kg']:.1f} kg (config {r['config_mass_kg']:.0f}), front share "
                 f"{r['front_mass_fraction']:.3f}, c {r['c_session']:.3f}; halves {r['half_first_kg']:.1f} / "
                 f"{r['half_second_kg']:.1f}, delta {r['delta_kg']:+.1f} kg; burn total {b.get('total_kg', float('nan')):.1f} kg,"
                 f" expected delta from burn {b.get('expected_delta_kg', float('nan')):+.1f}, burn-corrected delta {fc:+.1f};"
                 f" damper valid {'/'.join(f'{x * 100:.0f}' for x in r['damper_valid_share'].values())} %")
    nbr = [R[k] for k in NBR16 if k in R]
    m = np.array([r["mass_kg"] for r in nbr])
    sd = float(np.std(m, ddof=1)) if m.size > 1 else float("nan")
    d = np.array([abs(r["delta_kg"]) for r in R.values()])
    a, b_ = sd <= U_HI_KG, bool(np.all(d <= U_HI_KG))
    L.append(f"NBR mass: mean {np.mean(m):.1f}, SD {sd:.1f}, range {np.min(m):.1f}-{np.max(m):.1f} kg (n {m.size})")
    L.append(f"(a) SD <= {U_HI_KG:g} kg: {'PASS' if a else 'FAIL'} (strict {U_LO_KG:g} kg: {'within' if sd <= U_LO_KG else 'above'})")
    L.append(f"(b) max |delta| {np.max(d):.1f} kg <= {U_HI_KG:g}: {'PASS' if b_ else 'FAIL'} "
             f"({int(np.sum(d <= U_LO_KG))} / {d.size} files within the strict {U_LO_KG:g} kg)")
    L.append(f"STEP 1 VERDICT: {'PASS' if a and b_ else 'FAIL'}")
    txt = "\n".join(L)
    print(txt)
    with open(os.path.join(OUT_DIR, "report_step1.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "step1"
    {"step1": lambda: step1(sys.argv[2:]), "report1": report1}[mode]()
