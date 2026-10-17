# Old vs new validation of the Class B rebuild (R3, author work order
# 2026-10-17). Numbers only; the verdict is the author's.
#
# OLD = the modules at BASE_REV (the tree before R3), loaded from git into
# this process (the deleted Dugoff modules too, so the old tyre_fit_auto
# imports); NEW = the working tree. Same params (best available, the app's
# default path) and the same three exports.
#   parse      : old vs new parse_csv -- channels, quality, laps, warnings,
#                corners (fixes #1 #2 #3 #5 #6), must be bit-identical
#   chain      : old vs new resolve_sideslip_beta -- fit manifest, R pick,
#                sign check, gate, beta (fixes #23, 7, Dugoff removal); the
#                manifest's h2 check is reported apart (it reads the apex
#                window, #12)
#   stability  : old vs new estimate_yaw_moment_stability on each side's beta
#   Fix 0      : CS with the NEW beta, old vs new estimate_cornering_stiffness
#   #12        : summarise_corners old vs new on the same CS/stability, and
#                the TPMS apex windows
# Worst CS per lap = per corner instance, min over phases of the phase
# median (apex_3 read from apex_region, as the classifier does), capped at
# 1.0 -- the per-lap form of check_class_a's worst-lap read-out.
# Run: python -m diagnostics.check_class_b

import os
import subprocess
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402

BASE_REV = "85b9293"
SESSIONS = (
    ("Dubai sample export", "Sample/Sample_Dubai.txt"),
    ("NBR-73-010-026-JDR", "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
)
PHASES = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")
PR_CORNER, PR_LAPS = 16, (50, 51, 56)
MANIFEST_SKIP = ("timestamp", "git_commit_hash", "h2_vs_ay_apex", "data_file")


def old_module(path, name):
    src = subprocess.run(["git", "show", f"{BASE_REV}:{path}"], capture_output=True, text=True,
                         check=True, encoding="utf-8").stdout
    mod = types.ModuleType(name)
    mod.__file__ = f"{BASE_REV}:{path}"
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def load_old():
    sa = old_module("modules/stability_analysis.py", "old_stability_analysis")
    parser = old_module("modules/csv_parser.py", "old_csv_parser")
    # the old tyre_fit_auto imports the deleted Dugoff modules by name;
    # registered only for its exec, then removed again
    injected = {}
    try:
        injected["modules.tyre_model"] = old_module("modules/tyre_model.py", "modules.tyre_model")
        sys.modules["modules.tyre_model"] = injected["modules.tyre_model"]
        injected["modules.sideslip_ekf_dugoff"] = old_module("modules/sideslip_ekf_dugoff.py",
                                                             "modules.sideslip_ekf_dugoff")
        sys.modules["modules.sideslip_ekf_dugoff"] = injected["modules.sideslip_ekf_dugoff"]
        tfa = old_module("modules/tyre_fit_auto.py", "old_tyre_fit_auto")
    finally:
        for k in injected:
            sys.modules.pop(k, None)
    tfa.estimate_sideslip_ekf_pacejka = old_module("modules/sideslip_ekf_pacejka.py",
                                                   "old_sideslip_ekf_pacejka").estimate_sideslip_ekf_pacejka
    tfa.evaluate_gate = old_module("modules/nis_gate.py", "old_nis_gate").evaluate_gate
    df = old_module("modules/decision_frame.py", "old_decision_frame")
    return sa, parser, tfa, df


def params_best_available():
    from modules.stability_analysis import load_parameters
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    return apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))


def diff(a, b, path="", out=None, limit=8):
    """First differences between two nested results (dict/list/array/float),
    NaN == NaN. Returns a list of paths."""
    out = [] if out is None else out
    if len(out) >= limit:
        return out
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in a or k not in b:
                out.append(f"{path}.{k} (only in {'new' if k in b else 'old'})")
            else:
                diff(a[k], b[k], f"{path}.{k}", out, limit)
    elif isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            out.append(f"{path} (length {len(a)} vs {len(b)})")
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                diff(x, y, f"{path}[{i}]", out, limit)
    elif isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        if not (isinstance(a, np.ndarray) and isinstance(b, np.ndarray) and a.shape == b.shape
                and np.array_equal(a, b, equal_nan=a.dtype.kind == "f")):
            out.append(path)
    elif isinstance(a, float) and isinstance(b, float):
        if not (a == b or (a != a and b != b)):
            out.append(f"{path} ({a!r} vs {b!r})")
    elif a != b:
        out.append(f"{path} ({a!r} vs {b!r})")
    return out


def verdict(d):
    return "BIT-IDENTICAL" if not d else "DIFFERS: " + "; ".join(d)


def run_chain(tfa, state, params, data, path):
    beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(state, params, data, "ekf_auto_pacejka", csv_path=path)
    return {"beta": beta, "man": man, "gate": gate, "fallback": fb, "fallback_reason": fr}


def worst_cs_per_instance(summ, axle):
    out = {}
    for s in summ:
        vals = []
        for ph in PHASES:
            src = s["apex_region"] if ph == "apex_3" else s["phases"][ph]
            v = src[f"cs_ratio_{axle}"]["median"]
            if v == v:
                vals.append(v)
        if vals:
            out[(s["stable_corner_id"], s["lap_number"])] = min(1.0, min(vals))
    return out


def apex_cells(summ_old, summ_new, valid):
    """apex_3 stat medians NaN before and finite after (#12), plus every
    other cell that moved (must be none)."""
    filled = filled_valid = changed_inst = other = 0
    for so, sn in zip(summ_old, summ_new):
        po, pn = so["phases"]["apex_3"], sn["phases"]["apex_3"]
        hit = False
        for k, vo in po.items():
            if isinstance(vo, dict) and "median" in vo:
                a, b = vo["median"], pn[k]["median"]
                if a != a and b == b:
                    filled += 1
                    filled_valid += int(so["lap_number"] in valid)
                    hit = True
        changed_inst += int(hit or bool(diff(po, pn)))
        for ph in PHASES:
            if ph != "apex_3":
                other += int(bool(diff(so["phases"][ph], sn["phases"][ph])))
        other += int(bool(diff(so["apex_region"], sn["apex_region"])))
    return filled, filled_valid, changed_inst, other


def stats_line(x):
    if x.size == 0:
        return "none"
    return f"median {np.median(x):.3f} / p95 {np.percentile(x, 95):.3f} / max {x.max():.3f} (n {x.size})"


def main():
    from modules.csv_parser import parse_csv
    import modules.tyre_fit_auto as tfa_new
    import modules.decision_frame as df_new
    from modules.stability_analysis import (prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness, estimate_yaw_moment_stability,
                                            summarise_corners)
    sa_old, parser_old, tfa_old, df_old = load_old()
    params = params_best_available()
    assert params["stability_estimation"]["sideslip_source"] == "ekf_auto_pacejka"
    print(f"Class B check: OLD = {BASE_REV}, NEW = working tree; params best available (cap None)")
    for label, rel in SESSIONS:
        path = resolve_data_path(rel)
        print("\n" + "=" * 100)
        p_old, p_new = parser_old.parse_csv(path), parse_csv(path)
        print(f"{label}: {path}")
        print(f"  - parse (#1 #2 #3 #5 #6): {verdict(diff(p_old, p_new, 'parse'))}")
        print(f"      laps {len(p_new['laps'])}, analysis laps "
              f"{sum(1 for l in p_new['laps'] if l.get('is_valid_for_analysis'))}, lap warnings "
              f"{sum(len(l['warnings']) for l in p_new['laps'])}, corner instances {len(p_new['corners'])}")
        st_old = prepare_vehicle_state(p_old["channels"], params)
        st = prepare_vehicle_state(p_new["channels"], params)
        print(f"  - state: {verdict(diff(st_old, st, 'state'))} (grid {st['sample_rate_hz']:.0f} Hz)")

        r_old = run_chain(tfa_old, st_old, params, p_old, path)
        r_new = run_chain(tfa_new, st, params, p_new, path)
        m_old = {k: v for k, v in r_old["man"].items() if k not in MANIFEST_SKIP}
        m_new = {k: v for k, v in r_new["man"].items() if k not in MANIFEST_SKIP}
        ch = r_new["man"]["r_sweep"]["chosen"]
        print(f"  - fit manifest (status, axles, R sweep/pick, sign check, coverage): {verdict(diff(m_old, m_new, 'man'))}")
        print(f"      status '{r_new['man']['status']}', R pick ay x{ch['r_ay_scale']:g} / yaw x{ch['r_yaw_scale']:g} "
              f"({r_new['man']['r_sweep']['selection']}), sign check {r_new['man']['sign_check']['median_gate_racing']}")
        print(f"  - gate: {verdict(diff(r_old['gate'], r_new['gate'], 'gate'))} "
              f"('{(r_new['gate'] or {}).get('verdict')}')")
        print(f"  - beta: {verdict(diff(r_old['beta'], r_new['beta'], 'beta'))}; fallback "
              f"{r_old['fallback']} -> {r_new['fallback']}")
        h_old, h_new = r_old["man"]["h2_vs_ay_apex"], r_new["man"]["h2_vs_ay_apex"]
        print(f"  - h2 apex check (reads the #12 window): n {h_old['n']} -> {h_new['n']}, corr "
              f"{h_old['correlation']:.6f} -> {h_new['correlation']:.6f}")

        laps = p_new.get("laps", [])
        valid = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
        stab_old = sa_old.estimate_yaw_moment_stability(st_old, r_old["beta"], params, p_old.get("laps", []))
        stab = estimate_yaw_moment_stability(st, r_new["beta"], params, laps)
        print(f"  - stability: {verdict(diff(stab_old, stab, 'stab'))}")

        beta = r_new["beta"]
        slip = estimate_slip_angles(st, beta, params)
        forces = estimate_lateral_forces(st, params)
        cs_old = sa_old.estimate_cornering_stiffness(slip, forces, st, params)
        cs_new = estimate_cornering_stiffness(slip, forces, st, params, laps)
        for k in ("C_alpha", "C_window", "C_section", "R2"):
            for ax in "fr":
                assert not diff(cs_old[f"{k}_{ax}"], cs_new[f"{k}_{ax}"]), f"Fix 0 moved {k}_{ax}"
        for ax in "fr":
            # near_strike is only set where CS is finite, so compare it there
            both = np.isfinite(cs_old[f"CS_ratio_{ax}"]) & np.isfinite(cs_new[f"CS_ratio_{ax}"])
            assert np.array_equal(cs_old[f"near_strike_{ax}"][both], cs_new[f"near_strike_{ax}"][both])
        print("  - Fix 0: stiffness arrays (C_alpha, C_window, C_section, R2) unchanged; near_strike unchanged "
              "where CS is finite on both sides")
        for ax, name in (("f", "front"), ("r", "rear")):
            o, n = cs_old[f"CS_ratio_{ax}"], cs_new[f"CS_ratio_{ax}"]
            both = np.isfinite(o) & np.isfinite(n)
            d = np.abs(n[both] - o[both])
            print(f"      {name}: |delta CS| over changed samples {stats_line(d[d > 0])}; over all both-finite "
                  f"{stats_line(d)}")
            print(f"      {name}: finite -> NaN {int((np.isfinite(o) & ~np.isfinite(n)).sum())}, NaN -> finite "
                  f"{int((~np.isfinite(o) & np.isfinite(n)).sum())}; no-reference samples total "
                  f"{cs_new[f'n_no_reference_{ax}']} / on analysis laps {cs_new[f'n_no_reference_analysis_{ax}']}")

        corners = p_new.get("corners", [])
        summ_cs_old = summarise_corners(corners, cs_old, stab, st, lap_filter=None)
        summ_new = summarise_corners(corners, cs_new, stab, st, lap_filter=None)
        for ax, name in (("f", "front"), ("r", "rear")):
            wo, wn = worst_cs_per_instance(summ_cs_old, ax), worst_cs_per_instance(summ_new, ax)
            common = [k for k in sorted(set(wo) & set(wn), key=str) if k[1] in valid]
            moved = [k for k in common if abs(wn[k] - wo[k]) > 1e-12]
            print(f"      worst CS {name} per instance (analysis laps): {len(moved)} of {len(common)} moved"
                  + (f"; largest {max(moved, key=lambda k: abs(wn[k] - wo[k]))}" if moved else ""))
            if label.startswith("Paul Ricard") and ax == "f":
                for lap in PR_LAPS:
                    k = (PR_CORNER, lap)
                    print(f"      PR C{PR_CORNER} lap {lap} worst CS front: "
                          f"{wo.get(k, float('nan')):+.3f} -> {wn.get(k, float('nan')):+.3f}")

        summ_old12 = sa_old.summarise_corners(corners, cs_new, stab, st, lap_filter=None)
        filled, filled_valid, changed_inst, other = apex_cells(summ_old12, summ_new, valid)
        print(f"  - #12 apex window: NaN apex_3 cells filled {filled} (on analysis laps {filled_valid}), instances "
              f"changed {changed_inst}; other phases / apex_region moved: {other}")
        tp_moved = 0
        for w in ("fl", "fr", "rl", "rr"):
            a = df_old._tpms_apex_samples(p_new["channels"], corners, st, w)
            b = df_new._tpms_apex_samples(p_new["channels"], corners, st, w)
            if a is None or b is None:
                continue
            for cid in a:
                tp_moved += sum(int(len(x[1]) != len(y[1])) for x, y in zip(a[cid], b[cid]))
        print(f"      TPMS apex windows resized: {tp_moved} (corner-lap-wheel)")


if __name__ == "__main__":
    main()
