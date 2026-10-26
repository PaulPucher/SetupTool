"""Saved data-file paths across machines.

An outing stores the absolute path of its telemetry export. The laptop
keeps the exports under D:/Data_Bachelorthesis, the temporary machine under
H:/Data_Bachelorthesis, so a path saved on one machine does not exist on
the other: the outing could not reload its file, and once the file was
picked again by hand the analysis caches missed because the path string
differed. Both lookups now go through the configured data roots
(config/data_roots.json): a saved path under one root is found under the
others, and the caches compare the path relative to its root.
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
