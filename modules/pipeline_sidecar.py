# On-disk cache of the full pipeline result (state/cs/stab/fz/ls/slip/
# forces/corners), next to the DB summary cache. The DB only holds
# summaries; graphs and trace dialogs need the arrays.
#
# One gzip pickle per outing: small identity header first, then payload.
# Pickle over npz -- corners are nested Python objects.
# Only ever loads files this module wrote itself (data/analysis_cache/).

import gzip
import os
import pickle
import traceback

SIDECAR_FORMAT_VERSION = 1
SIDECAR_DIR = os.path.join("data", "analysis_cache")

SIDECAR_GZIP_COMPRESSLEVEL = 6  # gzip default, untuned

# same 7 fields as the DB-cache check (outing_form._try_render_cached_analysis)
# + format version; compared in order before the payload is unpickled
IDENTITY_FIELDS = (
    "schema_version", "csv_path", "accuracy_cap", "resolved_vehicle_snapshot",
    "sideslip_source", "grid_rate_hz", "lap_filter", "sidecar_format_version",
)


def _sidecar_path(outing_id):
    return os.path.join(SIDECAR_DIR, f"outing_{outing_id}.pkl.gz")


def build_identity(schema_version, csv_path, accuracy_cap, resolved_vehicle_snapshot,
                    sideslip_source, grid_rate_hz, lap_filter):
    """csv_path must already be normalised (_norm_path). lap_filter sorted
    here -> caller ordering can't cause a false mismatch."""
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
    """Atomic (tmp + os.replace). Never raises -- a failed cache write must
    not fail the analysis."""
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
    """(payload, None) on identity match, else (None, reason) -- reason is
    missing file, corrupt file, or the first mismatching field. Never raises;
    any failure = treat as absent."""
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
