# Storage paths and the on-disk analysis cache.
#
# Data roots: an outing stores the absolute path of its telemetry export, but
# each machine keeps the exports under its own data root
# (config/data_roots.json). A saved path under one root is looked up under the
# others, and the analysis caches compare the path relative to its root, so a
# file reopened on another machine still hits the cache.
#
# On-disk cache of the full pipeline result (state, cs, stab, fz, slip,
# forces, corners, events, setup layers), next to the DB summary cache. The DB only holds
# summaries; graphs and trace dialogs need the arrays.
#
# One gzip pickle per outing: small identity header first, then payload.
# Pickle over npz -- corners are nested Python objects.
# Only ever loads files this module wrote itself (data/analysis_cache/).

import gzip
import json
import os
import pickle
import traceback

# Every config, asset and data path is anchored here, so the app behaves the
# same whatever working directory it was started from.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def repo_path(*parts):
    return os.path.join(REPO_ROOT, *parts)


DATA_ROOTS_CONFIG_PATH = repo_path("config", "data_roots.json")


def load_roots():
    # a missing file, null roots or a malformed entry means "no roots", never
    # an exception: roots only help to reopen a path saved on another machine
    try:
        with open(DATA_ROOTS_CONFIG_PATH, "r", encoding="utf-8") as f:
            roots = json.load(f).get("roots")
    except (OSError, ValueError, AttributeError):
        return []
    if not isinstance(roots, list):
        return []
    return [r for r in roots if isinstance(r, str) and r]


def _norm(path):
    return os.path.normcase(os.path.normpath(path))


def split_root(path, roots=None):
    """(root, relative path) for a path under one of the roots, else None.
    The longest matching root wins, so nested roots resolve to the inner one."""
    if not path:
        return None
    p = _norm(path)
    best = None
    for root in roots if roots is not None else load_roots():
        r = _norm(root)
        if p.startswith(r + os.sep) and (best is None or len(r) > len(_norm(best[0]))):
            best = (root, os.path.relpath(p, r))
    return best


def resolve_saved_path(path, roots=None):
    """The saved path if it exists, else the same relative path under another
    root that exists, else None."""
    if not path:
        return None
    if os.path.exists(path):
        return path
    roots = roots if roots is not None else load_roots()
    hit = split_root(path, roots)
    if hit is None:
        return None
    for root in roots:
        candidate = os.path.join(root, hit[1])
        if os.path.exists(candidate):
            return os.path.normpath(candidate)
    return None


def path_key(path, roots=None):
    """Machine-independent cache key: relative to its data root when it has
    one, else the normalised absolute path."""
    if not path:
        return path
    hit = split_root(path, roots)
    return "<data>" + os.sep + _norm(hit[1]) if hit else _norm(path)


SIDECAR_FORMAT_VERSION = 1
SIDECAR_DIR = repo_path("data", "analysis_cache")

SIDECAR_GZIP_COMPRESSLEVEL = 6  # gzip default, untuned

# same 8 fields as the DB-cache check (outing_form._try_render_cached_analysis)
# + format version; compared in order before the payload is unpickled.
# estimator_settings: the Settings estimator inputs, from
# accuracy_resolution.resolve_estimator_settings. accuracy_cap is always None
# since every analysis runs at the best available level; it stays in the
# identity so existing sidecars keep matching.
IDENTITY_FIELDS = (
    "schema_version", "csv_path", "accuracy_cap", "resolved_vehicle_snapshot",
    "sideslip_source", "grid_rate_hz", "lap_filter", "estimator_settings", "sidecar_format_version",
)


def _sidecar_path(outing_id):
    return os.path.join(SIDECAR_DIR, f"outing_{outing_id}.pkl.gz")


def build_identity(schema_version, csv_path, accuracy_cap, resolved_vehicle_snapshot,
                    sideslip_source, grid_rate_hz, lap_filter, estimator_settings=None):
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
        "estimator_settings": estimator_settings,
        "sidecar_format_version": SIDECAR_FORMAT_VERSION,
    }


def _first_mismatch(stored_identity, expected_identity):
    for field in IDENTITY_FIELDS:
        stored, expected = stored_identity.get(field), expected_identity.get(field)
        if field == "csv_path":
            # same export under another machine's data root = same input
            stored, expected = path_key(stored), path_key(expected)
        if stored != expected:
            # name the input itself, so the miss reason says which setting moved
            if field == "estimator_settings" and isinstance(stored, dict) and isinstance(expected, dict):
                for key in sorted(set(stored) | set(expected)):
                    if stored.get(key) != expected.get(key):
                        return f"{field}.{key}"
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


def delete_sidecar(outing_id):
    """With its outing. SQLite may hand a deleted outing's id to the next
    new one, which would then find a stranger's sidecar. Never raises."""
    try:
        os.remove(_sidecar_path(outing_id))
    except FileNotFoundError:
        pass
    except Exception:
        print(traceback.format_exc())


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
