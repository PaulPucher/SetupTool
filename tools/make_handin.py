# Builds the hand-in folder from the working repo (development tool, not
# shipped). Copies the whitelist below; the config files get the generic
# values of handin_template/config/*.overlay.json, car_data.json and
# data_roots.json come from the template folder. Then the comment and prose
# edits of handin_template/handin_edits.patch are applied, so the working
# repo keeps its own comments and citations. Tests, diagnostics, tools, docs,
# images and data/ are not copied.
#
#   python tools/make_handin.py [target folder]
#   python tools/make_handin.py --plain <folder>   (export without the patch)

import copy
import json
import os
import shutil
import stat
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = os.path.join(ROOT, "handin_template")
PATCH = os.path.join(TEMPLATE, "handin_edits.patch")
DEFAULT_TARGET = os.path.join(os.path.dirname(ROOT), "Setuptool_handin")

FILES = ("main.py", "requirements.txt")
PACKAGES = ("models", "modules", "core", "ui")
CONFIG_FROM_REPO = ("channels.json", "parameters.json", "mechanisms.json", "setup_parameters.json",
                    "setup_windows.json")
# template file -> shipped name; the template keeps its own name so the repo's
# ignore rule for config/car_data.json cannot hide it
CONFIG_FROM_TEMPLATE = {"car_data.template.json": "car_data.json", "data_roots.json": "data_roots.json"}
REPLACE = "__replace__"


def merge(base, overlay):
    """Overlay values win; an overlay object carrying "__replace__" replaces
    the base object instead of being merged into it."""
    if isinstance(overlay, dict) and overlay.get(REPLACE):
        return {k: v for k, v in overlay.items() if k != REPLACE}
    if isinstance(base, dict) and isinstance(overlay, dict):
        out = copy.deepcopy(base)
        for k, v in overlay.items():
            if k == "_comment" and k in base:
                continue  # the overlay's own comment documents the overlay file
            out[k] = merge(base.get(k), v)
        return out
    return copy.deepcopy(overlay)


def same_leaves(real, template, path=()):
    # overlay leaves still equal to the working value, for the review
    if isinstance(template, dict):
        for k, v in template.items():
            if k in (REPLACE, "_comment"):
                continue
            yield from same_leaves(real.get(k) if isinstance(real, dict) else None, v, path + (k,))
    elif real == template and not isinstance(template, str):
        yield ".".join(path), template


def _writable_retry(func, path, _exc):
    # copytree copies the read-only attribute of the repo folders, and Windows
    # refuses to remove a read-only folder: clear the attribute and retry
    os.chmod(path, stat.S_IWRITE)
    func(path)


def _clear(target):
    # empties the target but keeps the folder itself (it may be open elsewhere)
    for name in os.listdir(target):
        p = os.path.join(target, name)
        if os.path.isdir(p):
            shutil.rmtree(p, onexc=_writable_retry)
        else:
            os.chmod(p, stat.S_IWRITE)
            os.remove(p)


def _remove_tree(path):
    shutil.rmtree(path, onexc=_writable_retry)


def export(target):
    os.makedirs(target, exist_ok=True)
    for f in FILES:
        shutil.copy2(os.path.join(ROOT, f), os.path.join(target, f))
    shutil.copy2(os.path.join(TEMPLATE, ".gitignore"), os.path.join(target, ".gitignore"))
    ignore = shutil.ignore_patterns("__pycache__", "*.pyc")
    for p in PACKAGES:
        shutil.copytree(os.path.join(ROOT, p), os.path.join(target, p), ignore=ignore, dirs_exist_ok=True)
    shutil.copy2(os.path.join(TEMPLATE, "README.md"), os.path.join(target, "README.md"))

    cfg_out = os.path.join(target, "config")
    os.makedirs(cfg_out, exist_ok=True)
    for name in CONFIG_FROM_REPO:
        with open(os.path.join(ROOT, "config", name), encoding="utf-8") as f:
            data = json.load(f)
        overlay_path = os.path.join(TEMPLATE, "config", name.replace(".json", ".overlay.json"))
        if os.path.exists(overlay_path):
            with open(overlay_path, encoding="utf-8") as f:
                overlay = json.load(f)
            for path, value in same_leaves(data, overlay):
                print(f"  note: {name} {path} = {value!r} equals the working value")
            data = merge(data, overlay)
        with open(os.path.join(cfg_out, name), "w", encoding="utf-8", newline="\n") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
    for src, dst in CONFIG_FROM_TEMPLATE.items():
        shutil.copy2(os.path.join(TEMPLATE, "config", src), os.path.join(cfg_out, dst))


def _apply_patch(folder):
    # git apply works outside a repository like patch(1); --check first so a
    # stale patch fails before anything is written. The patch keeps each
    # file's own line endings, so autocrlf must not convert them.
    for extra in (["--check"], []):
        r = subprocess.run(["git", "-c", "core.autocrlf=false", "apply", "--whitespace=nowarn", *extra, PATCH],
                           cwd=folder, capture_output=True, text=True)
        if r.returncode:
            return r.stderr.strip() or r.stdout.strip()
    return None


def main(target):
    # build next to the target and swap at the end, so a failed patch leaves
    # the previous hand-in folder untouched and no half-built one behind
    staging = target.rstrip("\\/") + ".building"
    if os.path.exists(staging):
        _remove_tree(staging)
    export(staging)
    err = _apply_patch(staging)
    if err:
        _remove_tree(staging)
        sys.exit(f"error: {os.path.basename(PATCH)} does not apply to the current export; "
                 f"nothing was written.\n{err}")
    if os.path.exists(target):
        _clear(target)
    shutil.copytree(staging, target, dirs_exist_ok=True)
    _remove_tree(staging)
    n = sum(len(fs) for _d, _s, fs in os.walk(target))
    print(f"wrote {target} ({n} files)")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--plain":
        export(sys.argv[2])
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TARGET)
