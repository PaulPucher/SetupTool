# Minimal reader for thesis scripts: one analysis sidecar (data/analysis_cache,
# written by the app after an analysis) -> the arrays a thesis figure needs.
# Reads only; no Qt, no recomputation. Kerb-affected follows the one
# kerb-exclusion concept: inside the kerb mask or carrying the estimator's
# grade mark (internal name near_strike), per axle -- the population the
# verdict statistics skip and the plots grey.

import gzip
import pickle

import numpy as np


def read_sidecar(path, laps=None):
    """Arrays of one sidecar file. laps = the parsed lap table (parse_csv's
    "laps"); None parses the identity's csv_path, so the per-sample lap ids use
    the same lap bounds the app analysed with. Lap id -1 = outside every lap."""
    with gzip.open(path, "rb") as f:
        identity = pickle.load(f)
        payload = pickle.load(f)
    state, cs = payload["state"], payload["cs"]
    if laps is None:
        from modules.csv_parser import parse_csv
        laps = parse_csv(identity["csv_path"])["laps"]

    t = np.asarray(state["time"], dtype=float)
    lap_id = np.full(t.shape, -1, dtype=int)
    for lap in laps:
        lap_id[(t >= lap["start_time"]) & (t <= lap["end_time"])] = lap["lap_number"]

    kerb = np.asarray(state["kerb_mask"], dtype=bool)
    out = {
        "schema_version": identity["schema_version"],
        "csv_path": identity["csv_path"],
        "analysis_laps": list(identity["lap_filter"]),
        "time_s": t,
        "distance_m": np.asarray(state["s_m"], dtype=float),
        "lap": lap_id,
        "kerb_mask": kerb,
        "corners": payload["corners"],
    }
    for ax in ("f", "r"):
        out[f"cs_{ax}"] = np.asarray(cs[f"CS_ratio_{ax}"], dtype=float)
        out[f"kerb_affected_{ax}"] = kerb | np.asarray(cs[f"near_strike_{ax}"], dtype=bool)
    return out
