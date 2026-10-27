# Derivation of the Paul Ricard event zero entry for logger software 24996
# (config/channels.json event_zero_entries.PRC_221_24996; review ruling
# 2026-10-27). The 24996 Paul Ricard exports read the damper gauges about
# 5 kN low at standstill (sign and scale correct); the 25016 exports of the
# same car and track read correctly and serve as the reference.
#
# Per session and corner: offset = median damper force over
# standstill-on-wheels samples - static corner load (W g / MR, gauge N).
# Standstill-on-wheels: v < 1 km/h and travel within +-5 mm of the session's
# pit-lane rolling median on all four corners (ride height is not usable:
# FL reads a constant -101 mm on every 24996 file); at least 1.0 s.
# Entry = median over the source sessions; u_gauge = sqrt(e_between^2 +
# e_ref^2), e_between = half the source spread, e_ref = RMS of the same
# method's offsets on the healthy 25016 files (their zero is right, so their
# offset is the method's own error, mostly the corner-weight guideline).
# The healthy files also give the validation envelope of the offset-free
# checks (wheel_loads.load_validation_quantities).
#
# Run: python -m diagnostics.inspect_prc24996_zero
# Output: diagnostics/output/prc24996_zero.json

import json
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy  # noqa: E402
from modules.csv_parser import load_channels_config, parse_csv  # noqa: E402
from modules.stability_analysis import load_car_data, load_parameters, prepare_vehicle_state  # noqa: E402
from modules.wheel_loads import (  # noqa: E402
    CORNER_AXLE, _motion_ratio, _normalize_travel_to_mm, estimate_wheel_loads_from_dampers,
    load_validation_quantities,
)

# (label, file, DB outing id for the setup sheet); evaluation files sit
# outside the test data roots, so the paths are absolute
SOURCES = [
    ("PRC1 (outing 41)", "H:/dani_try 100/PRC1.txt", 22),
    ("PRC4 (outing 43)", "H:/dani_try 100/PRC4.txt", 25),
]
HEALTHY = [
    ("PRC5 (outing 27)", "H:/dani_try 100/PRC5.txt", 26),
    ("PRC6 (outing 32)", "H:/dani_try 100/PRC6.txt", 27),
    ("PRC7 (outing 33)", "H:/dani_try 100/PRC7.txt", 28),
    ("LC 25", "H:/Data_Bachelorthesis/Paul_Ricard_MLA_US/MLA-SE07_US-v1.txt", 13),
]
CORNERS = ("fl", "fr", "rl", "rr")
STANDSTILL_KMH = 1.0
ON_WHEELS_TRAVEL_MM = 5.0
MIN_STANDSTILL_S = 1.0
PIT_KMH = (5.0, 70.0)
DB = "data/setuptool.db"
OUT = "diagnostics/output/prc24996_zero.json"


def session_params(outing_id):
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    row = con.execute("SELECT setup_data FROM outings WHERE id = ?", (outing_id,)).fetchone()
    con.close()
    setup = json.loads(row[0]) if row and row[0] else None
    params = load_parameters()
    return apply_resolved_vehicle(params, resolve_accuracy(params, setup))


def analyse(path, outing_id):
    p = session_params(outing_id)
    car = load_car_data()
    parsed = parse_csv(path)
    ch = parsed["channels"]
    state = prepare_vehicle_state(ch, p)
    t, v_kmh = state["time"], state["v_mps"] * 3.6
    sr = state["sample_rate_hz"]
    # undo any applied event entry: the derivation needs the file's own zero
    entries = load_channels_config().get("event_zero_entries", {})
    applied = parsed["event_zero_entry"]
    zero_back = {}
    for c in CORNERS:
        name = f"log_dms_dam_{c}"
        zero_back[c] = (entries[applied["event"]]["channels"][name]["zero_N"]
                        if applied["channels"].get(name) == "applied" else 0.0)
    force = {c: np.interp(t, ch[f"log_dms_dam_{c}"]["time"], ch[f"log_dms_dam_{c}"]["data"]) + zero_back[c]
             for c in CORNERS}
    travel = {c: _normalize_travel_to_mm(np.interp(t, ch[f"log_susp_travel_{c}"]["time"],
                                                   ch[f"log_susp_travel_{c}"]["data"]),
                                         ch[f"log_susp_travel_{c}"]["unit_raw"]) for c in CORNERS}
    lim = np.interp(t, ch["ecu_B_speedlimit_en"]["time"], ch["ecu_B_speedlimit_en"]["data"])
    pit = (lim > 0.5) & (v_kmh > PIT_KMH[0]) & (v_kmh < PIT_KMH[1])
    on_wheels = (v_kmh < STANDSTILL_KMH) & np.all(
        [np.abs(travel[c] - np.median(travel[c][pit])) < ON_WHEELS_TRAVEL_MM for c in CORNERS], axis=0)
    dur = on_wheels.sum() / sr
    if dur < MIN_STANDSTILL_S:
        return {"valid": False, "standstill_on_wheels_s": dur}
    cw = p["vehicle"]["corner_weights"]
    w = {"fl": cw["FL_kg"], "fr": cw["FR_kg"], "rl": cw["RL_kg"], "rr": cw["RR_kg"]}
    out = {"valid": True, "standstill_on_wheels_s": dur, "corner_weights_kg": w, "metadata": {
        k: parsed["metadata"].get(k) for k in ("TrackName", "OutingNumber", "SessionNumber")}, "corners": {}}
    for c in CORNERS:
        mr = float(np.median(_motion_ratio(travel[c][on_wheels], CORNER_AXLE[c], car)))
        reading = float(np.median(force[c][on_wheels]))
        static = w[c] * 9.81 / mr
        out["corners"][c] = {"reading_N": reading, "static_gauge_N": static, "mr": mr, "offset_gauge_N": reading - static}
    dr = estimate_wheel_loads_from_dampers(state, ch, p, car)
    out["_state"], out["_dr"], out["_ch"], out["_p"], out["_car"] = state, dr, ch, p, car
    return out


def quantities(res, zero=None):
    # zero: per-corner gauge offset to subtract first (trial on the sources)
    ch, p = res["_ch"], res["_p"]
    dr = res["_dr"]
    if zero is not None:
        ch = dict(ch)
        for c in CORNERS:
            name = f"log_dms_dam_{c}"
            ch[name] = dict(ch[name], data=ch[name]["data"] - zero[c], quality="valid")
        dr = estimate_wheel_loads_from_dampers(res["_state"], ch, p, res["_car"])
    return load_validation_quantities(res["_state"], {c: dr[c]["fz_N"] for c in CORNERS}, p)


def main():
    src = {lab: analyse(path, oid) for lab, path, oid in SOURCES}
    hea = {lab: analyse(path, oid) for lab, path, oid in HEALTHY}
    for lab, r in list(src.items()) + list(hea.items()):
        if not r["valid"]:
            sys.exit(f"{lab}: no valid standstill on wheels ({r['standstill_on_wheels_s']:.1f} s)")
    entry = {}
    for c in CORNERS:
        offs = [r["corners"][c]["offset_gauge_N"] for r in src.values()]
        zero = float(np.median(offs))
        e_between = (max(offs) - min(offs)) / 2.0
        e_ref = float(np.sqrt(np.mean([r["corners"][c]["offset_gauge_N"] ** 2 for r in hea.values()])))
        u_gauge = float(np.hypot(e_between, e_ref))
        mr = float(np.median([r["corners"][c]["mr"] for r in src.values()]))
        entry[c] = {"zero_N": zero, "e_between_N": e_between, "e_ref_N": e_ref, "u_gauge_N": u_gauge,
                    "mr_standstill": mr, "u_wheel_N": u_gauge * mr}
    print("ENTRY PRC_221_24996 (gauge N; wheel = gauge x MR at standstill):")
    for c in CORNERS:
        e = entry[c]
        print(f"  {c}: zero_N {e['zero_N']:9.1f}  e_between {e['e_between_N']:6.1f}  e_ref {e['e_ref_N']:6.1f}  "
              f"u_gauge {e['u_gauge_N']:6.1f}  u_wheel {e['u_wheel_N']:6.1f} (MR {e['mr_standstill']:.3f})")
    print("RESIDUALS (session offset - entry; healthy: offset itself, entry not applicable):")
    resid = {}
    for lab, r in src.items():
        resid[lab] = {c: r["corners"][c]["offset_gauge_N"] - entry[c]["zero_N"] for c in CORNERS}
        print(f"  source  {lab:18s} " + "  ".join(f"{c} {resid[lab][c]:7.1f}" for c in CORNERS)
              + f"  ({r['standstill_on_wheels_s']:.1f} s on wheels)")
    for lab, r in hea.items():
        resid[lab] = {c: r["corners"][c]["offset_gauge_N"] for c in CORNERS}
        print(f"  healthy {lab:18s} " + "  ".join(f"{c} {resid[lab][c]:7.1f}" for c in CORNERS)
              + f"  ({r['standstill_on_wheels_s']:.1f} s on wheels)")
    q_healthy = {lab: quantities(r) for lab, r in hea.items()}
    keys = ("v2_slope_total", "llt_front_N_per_g", "llt_rear_N_per_g", "llt_sum_N_per_g")
    envelope = {k: [min(q[k] for q in q_healthy.values()), max(q[k] for q in q_healthy.values())] for k in keys}
    print("VALIDATION QUANTITIES (healthy as logged; sources after the entry):")
    q_src = {lab: quantities(r, {c: entry[c]["zero_N"] for c in CORNERS}) for lab, r in src.items()}
    for lab, q in list(q_healthy.items()) + list(q_src.items()):
        print(f"  {lab:18s} v2 front {q['v2_slope_front']:.3f} rear {q['v2_slope_rear']:.3f} total "
              f"{q['v2_slope_total']:.3f} (aero front share {q['aero_front_share']:.3f}) | llt front "
              f"{q['llt_front_N_per_g']:.0f} rear {q['llt_rear_N_per_g']:.0f} sum {q['llt_sum_N_per_g']:.0f} N/g")
    print("ENVELOPE (healthy min-max):", {k: [round(a, 4) for a in v] for k, v in envelope.items()})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump({"entry": entry, "residuals_gauge_N": resid, "envelope": envelope,
                   "sources": {lab: {k: v for k, v in r.items() if not k.startswith("_")} for lab, r in src.items()},
                   "healthy": {lab: {k: v for k, v in r.items() if not k.startswith("_")} for lab, r in hea.items()},
                   "quantities": {**q_healthy, **q_src}}, f, indent=1)
    print("written", OUT)


if __name__ == "__main__":
    main()