# Q-FIT-LIT Step 1 acceptance: the inactive fit L leaves the production path
# bit-identical (author order 2026-10-18). OLD = tyre_fit_auto and
# sideslip_ekf_pacejka at BASE_REV, loaded from git into this process; NEW =
# the working tree. Same params (best available, the app's default path),
# same three exports, tyre_fit_mode per_session on both sides (the old path,
# selected explicitly since fit L became the default on 2026-10-20).
# Compared with tolerance 0 (NaN == NaN): fit manifest (timestamp and git
# hash skipped), R pick, sign check, gate verdict and score, fallback,
# shipped beta, every CS array.
# Run: python -m diagnostics.check_fit_lit_inactive

import os
import subprocess
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402
from diagnostics.check_class_b import diff  # noqa: E402

BASE_REV = "fd40c78"
SESSIONS = (
    ("NBR-73-010-026-JDR", "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
    ("NBR JST outing 29", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"),
)
MANIFEST_SKIP = ("timestamp", "git_commit_hash")


def old_module(path, name):
    src = subprocess.run(["git", "show", f"{BASE_REV}:{path}"], capture_output=True, text=True,
                         check=True, encoding="utf-8").stdout
    mod = types.ModuleType(name)
    mod.__file__ = f"{BASE_REV}:{path}"
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def main():
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_slip_angles,
                                            estimate_lateral_forces, estimate_cornering_stiffness)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    import modules.tyre_fit_auto as tfa_new

    tfa_old = old_module("modules/tyre_fit_auto.py", "old_tyre_fit_auto")
    tfa_old.estimate_sideslip_ekf_pacejka = old_module("modules/sideslip_ekf_pacejka.py",
                                                       "old_sideslip_ekf_pacejka").estimate_sideslip_ekf_pacejka
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))
    assert params["stability_estimation"]["sideslip_source"] == "ekf_auto_pacejka"
    # fit L is the config default since 2026-10-20 (schema 16): the old path
    # is selected explicitly, and this check now shows it stays bit-identical
    params["tyre_fit_auto"]["tyre_fit_mode"] = "per_session"
    # and its own NIS gate (config carries the fit-L values since 2026-10-20)
    params["nis_gate"].update({"threshold_use_ekf": 0.023, "threshold_warn": 0.0025})
    print(f"Q-FIT-LIT inactive check: OLD = {BASE_REV}, NEW = working tree", flush=True)
    all_ok = True
    for label, rel in SESSIONS:
        path = resolve_data_path(rel)
        data = parse_csv(path)
        reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
        state = prepare_vehicle_state(data["channels"], params)
        out = {}
        for side, tfa in (("old", tfa_old), ("new", tfa_new)):
            beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka", csv_path=path)
            slip = estimate_slip_angles(state, beta, params)
            cs = estimate_cornering_stiffness(slip, estimate_lateral_forces(state, params), state, params, data["laps"])
            out[side] = {"man": {k: v for k, v in man.items() if k not in MANIFEST_SKIP}, "gate": gate,
                         "fallback": (fb, fr), "beta": beta, "cs": cs}
        print(f"{label}:", flush=True)
        for key in ("man", "gate", "fallback", "beta", "cs"):
            d = diff(out["old"][key], out["new"][key], key)
            all_ok &= not d
            print(f"  {key}: {'BIT-IDENTICAL' if not d else 'DIFFERS: ' + '; '.join(d)}", flush=True)
        m = out["new"]["man"]
        print(f"  status {m['status']}, R pick {m['r_sweep']['chosen']['r_ay_scale']:g}/"
              f"{m['r_sweep']['chosen']['r_yaw_scale']:g}, gate {out['new']['gate']['verdict']} "
              f"{out['new']['gate']['health_score']:.6f}, BCD front "
              f"{m['axles']['front']['B'] * m['axles']['front']['C'] * m['axles']['front']['D']:.1f}", flush=True)
    print("ALL BIT-IDENTICAL" if all_ok else "MISMATCH", flush=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
