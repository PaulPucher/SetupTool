# WP-CACHE Phase 1: sidecar persistence of the full Modules-1-5 pipeline
# result (state/cs/stab/fz/ls/slip/forces/corners) alongside the existing
# DB summary cache. The DB (outing_form.py _build_analysis_data_json)
# stores only summaries + identity/status metadata -- graphs/trace dialogs
# need the large numpy-array-bearing Modules-1-5 outputs directly
# (thesis_notes.md "WP-CACHE Phase 1a groundwork: the empty hull
# finding"), which die with the process (WP6's in-memory _pipeline_cache).
# This sidecar closes that gap without touching the DB cache, which stays
# untouched and authoritative for what it already does.
#
# Format: single gzip-compressed pickle per outing, two objects written
# sequentially into the same stream -- a small identity header (cheap to
# read and compare before touching the large payload) then the payload
# itself. Pickle over npz: the payload mixes numpy-array-heavy dicts
# (state/cs/stab/fz/ls/slip/forces) with small nested Python structures
# (corners) that npz cannot hold without falling back to allow_pickle
# object arrays anyway -- pickle handles both uniformly, in one file, with
# no cross-file consistency risk. gzip gives a real, simple size lever.
#
# PICKLE SAFETY: pickle.load is used here only on files this same module
# wrote itself, under data/analysis_cache/, never transferred or received
# from anywhere else -- loading a foreign/untrusted sidecar is out of
# contract and never attempted.

import gzip
import os
import pickle
import traceback

SIDECAR_FORMAT_VERSION = 1
SIDECAR_DIR = os.path.join("data", "analysis_cache")

# gzip's own default level -- a reasonable size/speed midpoint, not tuned
# against a real measurement yet. Revisit if Phase 1e's real-session size
# check comes back too large or too slow.
SIDECAR_GZIP_COMPRESSLEVEL = 6

# The 7 WP5 DB-cache identity fields (ui/views/outing_form.py
# _try_render_cached_analysis) plus this format's own version tag --
# checked, in this order, before the payload is ever unpickled.
IDENTITY_FIELDS = (
    "schema_version", "csv_path", "accuracy_cap", "resolved_vehicle_snapshot",
    "sideslip_source", "grid_rate_hz", "lap_filter", "sidecar_format_version",
)


def _sidecar_path(outing_id):
    return os.path.join(SIDECAR_DIR, f"outing_{outing_id}.pkl.gz")


def build_identity(schema_version, csv_path, accuracy_cap, resolved_vehicle_snapshot,
                    sideslip_source, grid_rate_hz, lap_filter):
    """Same 7 fields and same values the WP5 DB-cache check compares --
    csv_path must already be normalised (_norm_path) and lap_filter is
    sorted here so caller-side ordering never causes a false mismatch."""
    return {
        "schema_version": schema_version,
        "csv_path": csv_path,
        "accuracy_cap": accuracy_cap,
        "resolved_vehicle_snapshot": resolved_vehicle_snapshot,
        "sideslip_source": sideslip_source,
        "grid_rate_hz": grid_rate_hz,
        "lap_filter": sorted(lap_filter or []),
        "sidecar_format_version": SIDECAR_FORMAT_VERSION,
    }


def _first_mismatch(stored_identity, expected_identity):
    for field in IDENTITY_FIELDS:
        if stored_identity.get(field) != expected_identity.get(field):
            return field
    return None


def write_sidecar(outing_id, identity, payload):
    """Atomic write (tmp file + os.replace) -- a sidecar exists whole or
    not at all, never truncated/partial. Never raises: a write failure is
    logged and swallowed, per Phase 1c -- it must never fail the analysis
    that produced the payload being cached."""
    try:
        os.makedirs(SIDECAR_DIR, exist_ok=True)
        final_path = _sidecar_path(outing_id)
        tmp_path = final_path + ".tmp"
        with gzip.open(tmp_path, "wb", compresslevel=SIDECAR_GZIP_COMPRESSLEVEL) as f:
            pickle.dump(identity, f, protocol=pickle.HIGHEST_PROTOCOL)
            pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp_path, final_path)
        return True
    except Exception:
        print(traceback.format_exc())
        return False


def load_sidecar(outing_id, expected_identity):
    """Returns (payload, None) on an identity match, or (None, reason) on
    any failure/mismatch: no file, unreadable/corrupt (truncated stream,
    bad gzip header, unpickling error), or the name of the first
    identity/version field that disagrees. Never raises -- every failure
    mode is the honest-cascade 'treat as absent, fall back' case."""
    path = _sidecar_path(outing_id)
    if not os.path.exists(path):
        return None, "no sidecar file"
    try:
        with gzip.open(path, "rb") as f:
            stored_identity = pickle.load(f)
            mismatch = _first_mismatch(stored_identity, expected_identity)
            if mismatch is not None:
                return None, mismatch
            payload = pickle.load(f)
        return payload, None
    except Exception:
        print(traceback.format_exc())
        return None, "unreadable/corrupt sidecar"
