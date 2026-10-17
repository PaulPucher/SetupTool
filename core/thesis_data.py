# Minimal reader for thesis scripts: one analysis sidecar (data/analysis_cache,
# written by the app after an analysis) -> the arrays a thesis figure needs.
# Reads only; no Qt, no recomputation. Kerb-affected follows the one
# kerb-exclusion concept: inside the kerb mask or carrying the estimator's
# grade mark (internal name near_strike), per axle -- the population the
# verdict statistics skip and the plots grey.

import gzip
import pickle

import numpy as np


class StaleSidecarError(ValueError):
    """The sidecar was written under another schema or analysis identity."""


def _check_identity(identity, path, expected_identity):
    # R24: a thesis figure must never be drawn from a stale sidecar. Always
    # checked: schema and format version, and the identity fields that follow
    # from the current config alone (sideslip source, Settings estimator
    # inputs). Vehicle snapshot, cap, grid rate and lap filter depend on the
    # outing -> checked only when the caller passes expected_identity.
    from modules import pipeline_sidecar
    from modules.accuracy_resolution import resolve_estimator_settings
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION, load_parameters
    params = load_parameters()
    current = {
        "schema_version": ANALYSIS_SCHEMA_VERSION,
        "sidecar_format_version": pipeline_sidecar.SIDECAR_FORMAT_VERSION,
        "sideslip_source": params["stability_estimation"].get("sideslip_source", "kinematic"),
        "estimator_settings": resolve_estimator_settings(params),
    }
    for field, value in current.items():
        if identity.get(field) != value:
            raise StaleSidecarError(f"{path}: stale sidecar, {field} is {identity.get(field)!r}, current "
                                    f"{value!r} -- re-run Analyse in the app before using it")
    if expected_identity is not None:
        mismatch = pipeline_sidecar._first_mismatch(identity, expected_identity)
        if mismatch is not None:
            raise StaleSidecarError(f"{path}: sidecar identity differs from the expected one at {mismatch} "
                                    "-- re-run Analyse in the app before using it")


def read_sidecar(path, laps=None, expected_identity=None):
    """Arrays of one sidecar file. laps = the parsed lap table (parse_csv's
    "laps"); None parses the identity's csv_path, so the per-sample lap ids use
    the same lap bounds the app analysed with. Lap id -1 = outside every lap.
    Raises StaleSidecarError before reading the payload if the sidecar does
    not match the current analysis (see _check_identity)."""
    with gzip.open(path, "rb") as f:
        identity = pickle.load(f)
        _check_identity(identity, path, expected_identity)
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
