# Lap census for the double-outlap defect (PLAN.md STATUS 2026-10-06, author
# EXECUTE; read-only). For every unique Pi Toolbox export on disk: the
# production lap classification (modules/csv_parser._split_laps, unchanged)
# beside the raw facts per lap -- duration, pit limiter at start and end,
# lap-distance rise, speed at start and end -- so a pit-exit fragment
# classified as a lap is visible as such. Channel quality is gated exactly as
# parse_csv does (channels.json ranges and channel_quality_gates).
#
# Output: diagnostics/lap_census/lap_census.txt

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics.inspect_channel_census import read_channels  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lap_census")
WANTED = ["lap_number", "lap_time", "lap_distance", "ecu_B_speedlimit_en", "ecu_speed"]


def _quality(name, data, cfg):
    if len(data) == 0:
        return "failed"
    lo, hi = cfg["channels"][name]["range"]
    ratio = float(np.mean((data >= lo) & (data <= hi)))
    g = cfg["channel_quality_gates"]
    return "failed" if ratio < g["failed_below"] else "partial" if ratio < g["partial_below"] else "valid"


def census(path):
    from modules.csv_parser import _split_laps, load_channels_config
    cfg = load_channels_config()
    raw = read_channels(path, WANTED)
    ch = {}
    for n in WANTED:
        if n in raw:
            t, d = raw[n]
            ch[n] = {"time": t, "data": d, "quality": _quality(n, d, cfg)}
        else:
            ch[n] = {"time": None, "data": None, "quality": "missing"}
    laps = _split_laps(ch, cfg)
    lim = ch["ecu_B_speedlimit_en"]
    spd = ch["ecu_speed"]
    ld = ch["lap_distance"]

    def at(c, t):
        if c["time"] is None:
            return None
        i = min(np.searchsorted(c["time"], t), len(c["data"]) - 1)
        return float(c["data"][i])

    rows = []
    for l in laps:
        rise = None
        if ld["time"] is not None:
            m = (ld["time"] >= l["start_time"]) & (ld["time"] <= l["end_time"])
            if m.any():
                rise = float(ld["data"][m].max() - ld["data"][m][0])
        rows.append({
            "lap": l["lap_number"], "dur_s": round(l["lap_time"], 2), "precise_s": l.get("lap_time_precise"),
            "out": l["is_outlap"], "in": l["is_inlap"], "valid": l["is_valid_for_analysis"],
            "lim_start": at(lim, l["start_time"]), "lim_end": at(lim, l["end_time"]),
            "v_start_kmh": at(spd, l["start_time"]), "v_end_kmh": at(spd, l["end_time"]),
            "dist_rise": rise, "warnings": l["warnings"],
        })
    return {"path": path, "quality": {n: ch[n]["quality"] for n in WANTED}, "laps": rows}


def main(workers=6):
    data = json.load(open(os.path.join(os.path.dirname(OUT_DIR), "channel_census", "census_exports.json"),
                          encoding="utf-8"))
    paths = [f["paths"][0] for f in data]
    with Pool(workers) as pool:
        res = sorted(pool.map(census, paths), key=lambda r: r["path"])
    L = ["LAP CENSUS -- production lap classification beside raw lap facts, every unique export "
         "(Q double-outlap defect, 2026-10-06)", "lim = pit limiter (ecu_B_speedlimit_en) at lap start / end; "
         "dist_rise in the file's lap_distance unit", ""]
    n_double = 0
    for r in res:
        outs = [x["lap"] for x in r["laps"] if x["out"]]
        flag = "  <== MORE THAN ONE OUT LAP" if len(outs) > 1 else ""
        n_double += len(outs) > 1
        L.append(f"== {r['path']}{flag}")
        L.append(f"   channel quality {r['quality']}; out laps {outs}; analysis laps "
                 f"{[x['lap'] for x in r['laps'] if x['valid']]}")
        for x in r["laps"]:
            L.append(f"   lap {x['lap']:>4}: {x['dur_s']:>8.2f} s  out {int(x['out'])} in {int(x['in'])} "
                     f"valid {int(x['valid'])}  lim {x['lim_start']}/{x['lim_end']}  v {x['v_start_kmh']}/"
                     f"{x['v_end_kmh']} km/h  dist_rise {x['dist_rise']}  {x['warnings'] or ''}")
        L.append("")
    L.insert(2, f"files with more than one out lap: {n_double} of {len(res)}")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "lap_census.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L), flush=True)


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 6)
