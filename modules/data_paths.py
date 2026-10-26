"""Machine-independent data paths. An outing stores the absolute path of its
telemetry export, but each machine keeps the exports under its own data root
(config/data_roots.json). A saved path under one root is looked up under the
others, and the analysis caches compare the path relative to its root, so a
file reopened on another machine still hits the cache.
"""

import json
import os

DATA_ROOTS_CONFIG_PATH = "config/data_roots.json"


def load_roots():
    try:
        with open(DATA_ROOTS_CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)["roots"]
    except (OSError, ValueError, KeyError):
        return []


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
