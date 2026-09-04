# Fz-integration Phase 5: before/after report. Read-only, no config/
# production changes beyond what's already implemented. Compares the
# CURRENT (guarded) modules.longitudinal_forces.estimate_slip_ratio /
# LS_ratio against the PRE-Phase-5 behaviour (loaded directly from git
# HEAD, same technique as diagnostics/inspect_v3_pit_limiter_lap_census.py's
# own corner-canonicalisation before/after) on both real sessions.
# Reports: per-corner wheel_speed_source share (log_speed / abs_speed_
# fallback / nan_no_fallback), and LS_ratio no-signal fraction on the
# rear before vs after, per the work order's own explicit ask.

import subprocess
import sys
import tempfile
import importlib.util

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.longitudinal_forces import estimate_slip_ratio as estimate_slip_ratio_after
from modules.longitudinal_stiffness import estimate_longitudinal_stiffness

SESSIONS = (
    ("dubai", "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"),
    ("v3", "GT3_PRC_MLA-v3.txt"),
)


def _load_before_module():
    # Loaded from a system temp file, not diagnostics/ -- this is a
    # throwaway snapshot of git HEAD's own module, regenerated every run,
    # never a real deliverable to leave lying around in a tracked
    # directory (same reasoning as the project's own scratchpad-directory
    # convention for temp files).
    old_src = subprocess.run(["git", "show", "HEAD:modules/longitudinal_forces.py"],
                              capture_output=True, text=True, check=True).stdout
    fd, path = tempfile.mkstemp(suffix=".py", prefix="longitudinal_forces_before_")
    with open(fd, "w", encoding="utf-8") as f:
        f.write(old_src)
    spec = importlib.util.spec_from_file_location("longitudinal_forces_before", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["longitudinal_forces_before"] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    before_mod = _load_before_module()

    for label, raw_file in SESSIONS:
        print(f"\n{'='*80}\n{label}: {raw_file}\n{'='*80}")
        data = parse_csv(raw_file)
        params = load_parameters()
        state = prepare_vehicle_state(data["channels"], params)
        if state is None:
            print("prepare_vehicle_state returned None")
            continue

        slip_before = before_mod.estimate_slip_ratio(state, data["channels"], params)
        slip_after = estimate_slip_ratio_after(state, data["channels"], params)

        kf_b, kr_b = slip_before["kappa_f"], slip_before["kappa_r"]
        kf_a, kr_a = slip_after["kappa_f"], slip_after["kappa_r"]

        n_diff_f = int(np.sum(~np.isclose(kf_b, kf_a, equal_nan=True)))
        n_diff_r = int(np.sum(~np.isclose(kr_b, kr_a, equal_nan=True)))
        print(f"kappa_f samples changed by the guard: {n_diff_f} / {len(kf_b)} ({n_diff_f/len(kf_b)*100:.2f}%)")
        print(f"kappa_r samples changed by the guard: {n_diff_r} / {len(kr_b)} ({n_diff_r/len(kr_b)*100:.2f}%)")

        source = slip_after.get("wheel_speed_source", {})
        for corner in ("fl", "fr", "rl", "rr"):
            src = source.get(corner)
            if src is None:
                print(f"  {corner}: no source array (channel unavailable)")
                continue
            vals, counts = np.unique(src, return_counts=True)
            frac = {v: c / len(src) for v, c in zip(vals, counts)}
            print(f"  {corner} wheel_speed_source: " +
                  ", ".join(f"{v}={frac[v]*100:.2f}%" for v in vals))

        # LS_ratio no-signal fraction, rear, before vs after. fx does not
        # depend on wheel speed at all (braking/driving split from ax and
        # brake pressure) -- the SAME fx feeds both LS_ratio computations,
        # only slip (kappa) differs between before/after.
        from modules.longitudinal_forces import estimate_longitudinal_forces
        fx = estimate_longitudinal_forces(state, data["channels"], params)

        ls_after = estimate_longitudinal_stiffness(fx, slip_after, state, params)
        ls_before = estimate_longitudinal_stiffness(fx, slip_before, state, params)

        for tag, ls in (("BEFORE", ls_before), ("AFTER", ls_after)):
            r = ls["LS_ratio_r"]
            n_valid = int(np.isfinite(r).sum())
            n_total = len(r)
            print(f"  LS_ratio_r {tag}: n_valid={n_valid}/{n_total} "
                  f"({n_valid/n_total*100:.2f}% signal, {(1-n_valid/n_total)*100:.2f}% no-signal)")


if __name__ == "__main__":
    main()
