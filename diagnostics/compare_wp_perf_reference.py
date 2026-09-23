# WP-PERF Phase 0b: byte-identity comparison helper, used by every later
# phase's proof step (1b/2b/3a) against the frozen reference captured by
# capture_wp_perf_reference.py. `[keep-reproduces]`.
#
# "Byte-identical including NaN positions" is checked two ways per key,
# both must hold: (1) np.array_equal(..., equal_nan=True) -- the
# numerically-meaningful check, NaN-position-aware; (2) a.tobytes() ==
# b.tobytes() -- literal byte identity, catching the (numpy-unlikely but
# not impossible) case of a differently-encoded NaN payload that (1)
# would silently accept. Shape/dtype mismatches are reported as failures,
# never coerced.

import sys

import numpy as np


def compare_npz(path_a, path_b):
    """Returns (ok: bool, mismatches: list[str])."""
    a = np.load(path_a)
    b = np.load(path_b)
    keys_a, keys_b = set(a.files), set(b.files)
    mismatches = []
    if keys_a != keys_b:
        mismatches.append(f"key sets differ: only in A={keys_a - keys_b}, only in B={keys_b - keys_a}")
        return False, mismatches

    for key in sorted(keys_a):
        va, vb = a[key], b[key]
        if va.shape != vb.shape:
            mismatches.append(f"{key}: shape differs {va.shape} vs {vb.shape}")
            continue
        if va.dtype != vb.dtype:
            mismatches.append(f"{key}: dtype differs {va.dtype} vs {vb.dtype}")
            continue
        numerically_equal = np.array_equal(va, vb, equal_nan=True) if np.issubdtype(va.dtype, np.floating) \
            else np.array_equal(va, vb)
        bytes_equal = va.tobytes() == vb.tobytes()
        if not (numerically_equal and bytes_equal):
            n_diff = int(np.sum(~np.isclose(va, vb, equal_nan=True))) if np.issubdtype(va.dtype, np.floating) \
                else int(np.sum(va != vb))
            first_idx = None
            diff_mask = ~(np.isnan(va) & np.isnan(vb)) & (va != vb) if np.issubdtype(va.dtype, np.floating) \
                else (va != vb)
            idxs = np.flatnonzero(diff_mask)
            if len(idxs):
                first_idx = int(idxs[0])
            mismatches.append(
                f"{key}: numerically_equal={numerically_equal} bytes_equal={bytes_equal} "
                f"n_diff={n_diff} first_idx={first_idx}"
            )
    return len(mismatches) == 0, mismatches


def _self_test():
    """Prove the helper actually discriminates before trusting it for the
    package's own hard bar: (1) a file must compare equal to itself,
    including its own NaN entries; (2) a single flipped value must be
    caught, at the exact key/index it was flipped at."""
    import os
    import tempfile

    rng = np.random.default_rng(0)
    arr1 = rng.normal(size=50)
    arr1[3] = np.nan
    arr1[17] = np.nan
    arr2 = np.array([1, 2, 3], dtype=np.int64)

    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "ref.npz")
        np.savez(path, x__a=arr1, x__b=arr2)

        ok, mismatches = compare_npz(path, path)
        assert ok, f"self-comparison must pass: {mismatches}"

        corrupted = os.path.join(d, "ref_corrupted.npz")
        arr1_bad = arr1.copy()
        arr1_bad[10] += 1e-9
        np.savez(corrupted, x__a=arr1_bad, x__b=arr2)
        ok, mismatches = compare_npz(path, corrupted)
        assert not ok, "corrupted copy must be caught, was reported as identical"
        assert any("x__a" in m and "first_idx=10" in m for m in mismatches), \
            f"corruption not localised to the right key/index: {mismatches}"

        print("compare_wp_perf_reference self-test: PASS (identity accepted, single-value corruption caught at index 10)")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        _self_test()
    elif len(sys.argv) == 3:
        ok, mismatches = compare_npz(sys.argv[1], sys.argv[2])
        if ok:
            print(f"BYTE-IDENTICAL: {sys.argv[1]} == {sys.argv[2]}")
        else:
            print(f"MISMATCH: {sys.argv[1]} vs {sys.argv[2]}")
            for m in mismatches:
                print(f"  {m}")
            sys.exit(1)
    else:
        print("usage: compare_wp_perf_reference.py [path_a.npz path_b.npz]  (no args = self-test)")
        sys.exit(2)
