# Q-FIT-SIMPLE -- tyre-fit simplification variants through the whole chain
# (PLAN.md queue entry Q-FIT-SIMPLE; author order and EXECUTE 2026-10-18).
# Read-only: nothing in modules/ or config/ changes. Each variant replaces
# tyre_fit_auto._fit_axle_pacejka for one chain run and is restored in
# `finally`; everything after the fit (R derivation and sweep, sign check,
# D4 plausibility gate, NIS gate, kinematic fallback, CS) is production code.
#
# FROZEN FRAME (written before any number exists; author rulings 2026-10-18)
#
# VARIANTS (per session, per axle; fit population, optimiser and chain as
# production unless stated). Starts: every fixed-parameter variant starts
# B, D at 12 / 8000 (production start), so only the parametrisation
# changes; P13 alone changes the start.
#   P      production: B, C, D, E free, unbounded, start (12, 1.9, 8000, 0.97),
#          unweighted SSE, Powell
#   P13    B, C, D, E free, unbounded, start (10, 1.3, max|Fy| of the axle's
#          fit population, 0.97) -- the chair tyre-fit tool's START only (its
#          procedure differs: least_squares, folded, speed-banded weights)
#   E0     E fixed 0; C free, bounded [1, 3]; B, D free; start (12, 1.9, 8000)
#   E0C19  E fixed 0, C fixed 1.9; B, D free; start (12, 8000)
#   E0C13  E fixed 0, C fixed 1.3; B, D free; start (12, 8000)
#   E97C13 E fixed 0.97, C fixed 1.3; B, D free; start (12, 8000)
#   Pw     P with inverse-density weights
#   E0w    E0 with inverse-density weights
# Tier A choices: Goblirsch et al. 2025 (parameter correlation; fix E without
# sliding data; C ~1.3 lateral), Rajamani sec. 13.9 (E changes neither origin
# slope nor peak value), Pacejka 2006 sec. 4.3 (D peak, BCD origin slope),
# chair performance_analysis tooling (internal) for both starts.
# INVERSE-DENSITY WEIGHTS (Tier B preprocessing, motivated by Goblirsch et
# al.'s remark on an even data distribution, not their method): signed alpha
# binned in 0.01 rad bins over the fit population, weight = 1 / (samples in
# the sample's bin), weighted SSE. Deletes nothing. RISK: sparse corrupt
# samples are upweighted as well -- the near-zero-slip band is not removed,
# only its influence on the fit changes. The residuals handed to the R
# derivation stay unweighted (production definition of R).
#
# ENTRY CHECK (before any variant; any mismatch -> stop, no variant runs):
#   E1 P through this harness (the script's own fitter, all wrappers active)
#      vs unswapped production in the same process, BIT-IDENTICAL (tolerance
#      0, NaN == NaN): fit parameters, R pick, sign check, gate verdict and
#      health score, beta_with_fallback (the shipped beta), every CS array.
#   E2 secondary, vs diagnostics/output/r5/stepB.log "[new]" lines where the
#      session is present: R pick, sign check, B*C*D per axle (printed to
#      1 N/rad -> |diff| <= 0.5), fit samples (exact), gate score (4 dp).
#
# METRICS
#   Fit, per axle: B, C, D, E, BCD, peak alpha (production grid), p99 |alpha|
#     visited, peak covered (peak <= p99), Powell converged, rms residual
#     overall and per |alpha| bin 0-1 / 1-2 / 2-4 / > 4 deg, mean residual
#     left vs right (signed alpha < 0 / > 0), mean residual per corner speed
#     class (samples inside canonical corner brackets on analysis laps), D4
#     result, fit status and reason. Residual = Fy_filt - curve on the fit
#     population (kinematic alpha), the population the fit itself saw.
#   M-curve (vs P unless stated): |Fy_variant - Fy_P| / D_P on a 2000-point
#     grid; RMS and max over (i) the visited range, signed alpha [p1, p99]
#     of the fit population, and the |alpha| regions (ii) 0..p95, (iii)
#     p95..p99, (iv) p99..15 deg (peak_max_deg of the D4 gate, read from
#     config). Regions per the (d2) addendum split.
#   M-rel: |BCD_v / BCD_P - 1| and |D_v / D_P - 1|.
#   BAR 5.5 % of D (the (d2) V1 materiality bar, a LOWER bound); applied to
#     M-rel as an EXTENSION of that bar. No lap bootstrap this cycle: every
#     "within the bar" is a POINT ESTIMATE.
#   M-peak: width in deg of the positive-side alpha span where the curve is
#     >= 0.95 x its maximum on the production peak grid (0.01..89.9 deg);
#     NaN when the maximum sits on the grid edge (no peak).
#   EKF / gate: R selection (scales, selection), sign-check fraction, NIS
#     gate health score and verdict, fallback used and reason, hard-bound
#     re-entries (|raw beta| > beta_hard_bound_deg on the base mask, final
#     run), NIS-window divergences (diverged and not bound, base mask),
#     |shipped beta| median / p95 on straights (base mask outside every
#     canonical corner bracket).
#   CS, VERDICT POPULATION per axle: samples inside canonical corner
#     brackets on analysis laps, moving, not in the kerb mask, not carrying
#     the axle's kerb-affected mark (near_strike = kerb mask or smear zone in
#     the regression window, as summarise_corners skips). Finite-CS share,
#     n_no_reference on analysis laps, reference-set events (changes of the
#     linear reference over the whole array), reference median and p5 /
#     median over the population, CS p1 / p5 / p10 / p50, share CS < -1 and
#     < -2 of finite values, and pooled over both axles the share < -2 and
#     the finite share. CROSS-LAP RESIDUAL: the Q-STIFF-CS definition, reused
#     unchanged (inspect_stiffness_cs.grids + decompose: 100-point grid per
#     stable corner, residual = mean across-lap variance), on the clean
#     arrays (kerb-affected samples NaN).
#
# ADMISSIBLE (per variant): on every session and both axles -- fit not
#   degenerate, D4 pass, NIS gate not "fail", no fallback.
# BEATS P (proxy for "CS in an analysable range", NOT a truth claim; numbers
#   vs production track change, never quality): on >= 2 of 3 sessions ALL of
#   (a) pooled share CS < -2 (both axles, verdict population) lower than P,
#   (b) cross-lap residual not worse than P by > 5 % on either axle,
#   (c) pooled finite-CS share not lower than P by > 2 percentage points.
#   Stated for every variant; it counts only for an admissible variant. A
#   session where P is not admissible stays in; a variant that makes it
#   admissible is reported separately as a point in its favour.
#   [2026-10-18, author ruling Q-FIT-LIT: BEATS P WITHDRAWN as a decision
#   basis -- it compared CS proxies against production. Kept as run record.]
#
# PREDICTIONS (scored automatically; each claim per unit = session-axle or
#   session: CONFIRMED if it holds on every unit, REFUTED if it holds on
#   none, MIXED otherwise; a prediction is CONFIRMED / REFUTED if all its
#   claims are, else MIXED):
#   P13: (1) |C - 1.3| <= 0.1 (C stays near its start); (2) visited-range
#        M-curve RMS vs P < bar. Decides: C may be fixed by choice without
#        changing the fit where data exist.
#   E0:  (1) M-rel BCD and D < bar; (2) differences only beyond the peak:
#        M-curve RMS 0..p95 < bar and RMS p99..15 deg >= RMS 0..p95;
#        (3) hard-bound re-entries lower than P (per session). Decides: fix E.
#   E0C19 vs E0C13: (1) visited-range M-curve RMS between them < bar;
#        (2) RMS p99..15 deg between them > bar; (3) E0C13 fewer hard-bound
#        re-entries than E0C19 (per session); (4) E0C13 lower pooled share
#        CS < -2 than E0C19 (per session). Decides: the C value.
#   E97C13: (1) visited-range M-curve RMS vs E0C13 < bar; (2) M-peak wider
#        than E0C13. Decides: E value (0 vs 0.97).
#   Pw / E0w: (1) M-rel BCD vs the unweighted twin (P / E0) > bar; direction
#        NOT predicted (reported only). Decides: whether inverse-density
#        weighting enters the fit.
#
# RENDERS (render title rule; plain words, no codes in titles or legends):
#   fit_<key>_<axle>.png    2D histogram alpha vs Fy with all variant curves
#   csdist_<key>.png        CS distribution per variant and axle (ECDF)
#   traces_<key>_<code>.png standard trace set per variant (per-iteration
#                           render rule; diagnostics/trace_render.py)
# Outputs: diagnostics/plots/fit_simplification/ (JSON per session, run.log).
# Run: python -m diagnostics.inspect_fit_simplification

import ast
import contextlib
import ctypes
import json
import os
import re
import sys
import time

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.conftest import resolve_data_path  # noqa: E402
import modules.tyre_fit_auto as tfa  # noqa: E402
from modules.tyre_model_pacejka import pacejka_lateral_force, pacejka_lateral_stiffness  # noqa: E402

OUT_DIR = "diagnostics/plots/fit_simplification"
SESSIONS = (
    ("nbr_jdr", "NBR-73-010-026-JDR",
     "unlabeled/02_NBR_CRV_data/01_SE_01/NBR-73-010-026-JDR-26.07.01.1029.txt"),
    ("pr_us_v2", "Paul Ricard MLA-SE07_US-v2", "paul_Ricard_MLA_solved/MLA-SE07_US-v2.txt"),
    ("nbr_jst29", "NBR JST outing 29", "NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt"),
)
STEPB_LOG = "diagnostics/output/r5/stepB.log"
STEPB_NAMES = {"nbr_jdr": "NBR-73-010-026-JDR", "pr_us_v2": "PR MLA-SE07_US-v2"}

# free: names optimised; fixed: held values; start: per free name; bounds per free name
VARIANTS = {
    "P": dict(free="BCDE", fixed={}, start=(12.0, 1.9, 8000.0, 0.97), bounds=None, weighted=False,
              words="production fit (all four parameters free)"),
    "P13": dict(free="BCDE", fixed={}, start="chair", bounds=None, weighted=False,
                words="all four free, chair tyre-fit start"),
    "E0": dict(free="BCD", fixed={"E": 0.0}, start=(12.0, 1.9, 8000.0),
               bounds=[(None, None), (1.0, 3.0), (None, None)], weighted=False,
               words="curvature factor fixed at 0, shape factor free in 1-3"),
    "E0C19": dict(free="BD", fixed={"C": 1.9, "E": 0.0}, start=(12.0, 8000.0), bounds=None, weighted=False,
                  words="curvature factor 0, shape factor fixed at 1.9"),
    "E0C13": dict(free="BD", fixed={"C": 1.3, "E": 0.0}, start=(12.0, 8000.0), bounds=None, weighted=False,
                  words="curvature factor 0, shape factor fixed at 1.3"),
    "E97C13": dict(free="BD", fixed={"C": 1.3, "E": 0.97}, start=(12.0, 8000.0), bounds=None, weighted=False,
                   words="curvature factor 0.97, shape factor fixed at 1.3"),
    "Pw": dict(free="BCDE", fixed={}, start=(12.0, 1.9, 8000.0, 0.97), bounds=None, weighted=True,
               words="production fit with even slip-angle weighting"),
    "E0w": dict(free="BCD", fixed={"E": 0.0}, start=(12.0, 1.9, 8000.0),
                bounds=[(None, None), (1.0, 3.0), (None, None)], weighted=True,
                words="curvature factor 0, shape factor free, even slip-angle weighting"),
}
# plot identity: production in ink, the rest in the fixed categorical order;
# weighted twins dashed as a second encoding
INK = "#1f2937"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6"]
STYLE = {"P": (INK, "-")}
for _code, _col in zip([c for c in VARIANTS if c != "P"], SERIES):
    STYLE[_code] = (_col, "--" if VARIANTS[_code]["weighted"] else "-")

BAR = 0.055
DENSITY_BIN_RAD = 0.01
RESID_BINS_DEG = ((0, 1), (1, 2), (2, 4), (4, np.inf))
PEAK_GRID_DEG = np.linspace(0.01, 89.9, 4000)  # production's peak grid

_LOG = None


def log(msg=""):
    print(msg, flush=True)
    if _LOG is not None:
        _LOG.write(msg + "\n")
        _LOG.flush()


def keep_awake(on):
    # process-scoped "system required" flag; the machine may not sleep while
    # the run is alive (no global power setting is touched)
    if os.name == "nt":
        ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | (ES_SYSTEM_REQUIRED if on else 0))


# ------------------------------------------------------------------ fitter

def make_fitter(code):
    """Same signature and return dict as tyre_fit_auto._fit_axle_pacejka;
    population, residuals and peak search copied from it. For P the
    objective and the minimize call are the production ones, so E1 checks
    this reimplementation as well as the harness."""
    spec = VARIANTS[code]

    def fitter(alpha, Fy, base_mask):
        m2 = base_mask & np.isfinite(alpha) & np.isfinite(Fy)
        a2, f2 = alpha[m2], Fy[m2]
        if len(a2) == 0:
            return {
                "B": float("nan"), "C": float("nan"), "D": float("nan"), "E": float("nan"),
                "powell_converged": False,
                "fit_n_samples": 0, "fit_rms_resid_N": float("nan"),
                "peak_alpha_deg": float("nan"), "peak_in_visited_range": False,
                "visited_alpha_p99_deg": float("nan"),
                "sign_ok": False,
                "residuals": np.array([]), "residual_mask": m2,
            }
        if spec["weighted"]:
            k = np.floor(a2 / DENSITY_BIN_RAD).astype(np.int64)
            _, inv, cnt = np.unique(k, return_inverse=True, return_counts=True)
            w = 1.0 / cnt[inv]
        else:
            w = None
        free, fixed = spec["free"], spec["fixed"]

        def full(p):
            d = dict(fixed)
            d.update(zip(free, p))
            return d["B"], d["C"], d["D"], d["E"]

        if w is None and free == "BCDE":
            def _sse(p):
                B, C, D, E = p
                pred = pacejka_lateral_force(a2, B, C, D, E)
                return float(np.sum((pred - f2) ** 2))
        else:
            def _sse(p):
                pred = pacejka_lateral_force(a2, *full(p))
                r2 = (pred - f2) ** 2
                return float(np.sum(r2 if w is None else w * r2))

        start = spec["start"]
        if start == "chair":
            start = (10.0, 1.3, float(np.max(np.abs(f2))), 0.97)
        if spec["bounds"] is None:
            res = minimize(_sse, start, method="Powell")
        else:
            res = minimize(_sse, start, method="Powell", bounds=spec["bounds"])
        B, C, D, E = (float(x) for x in full(res.x))
        pred = pacejka_lateral_force(a2, B, C, D, E)
        resid = f2 - pred
        rms = float(np.sqrt(np.mean(resid ** 2)))

        grid_deg = np.linspace(0.01, 89.9, 4000)
        grid_rad = np.radians(grid_deg)
        stiffness_grid = pacejka_lateral_stiffness(grid_rad, B, C, D, E)
        sign_changes = np.where(np.diff(np.sign(stiffness_grid)) < 0)[0]
        if len(sign_changes) > 0:
            i = sign_changes[0]
            peak_deg = float(grid_deg[i])
            peak_in_visited_range = peak_deg <= float(np.percentile(np.abs(np.degrees(a2)), 99))
        else:
            peak_deg = float("nan")
            peak_in_visited_range = False

        return {
            "B": B, "C": C, "D": D, "E": E,
            "powell_converged": bool(res.success),
            "fit_n_samples": int(len(a2)), "fit_rms_resid_N": rms,
            "peak_alpha_deg": peak_deg, "peak_in_visited_range": peak_in_visited_range,
            "visited_alpha_p99_deg": float(np.percentile(np.abs(np.degrees(a2)), 99)),
            "sign_ok": D > 0,
            "residuals": resid, "residual_mask": m2,
        }

    return fitter


# ------------------------------------------------------------------ harness

ORIG = {"fit": tfa._fit_axle_pacejka, "fsp": tfa.fit_session_pacejka,
        "ekf": tfa.estimate_sideslip_ekf_pacejka}


@contextlib.contextmanager
def swap(fitter, holder):
    """Variant fitter in, raw manifest and final EKF run captured; all three
    names restored whatever happens."""
    def fsp(*a, **kw):
        man = ORIG["fsp"](*a, **kw)
        holder["raw"] = man
        return man

    def ekf(state, params, pass_id=None, **kw):
        out = ORIG["ekf"](state, params, pass_id=pass_id, **kw)
        if pass_id == "_auto_final":
            holder["final"] = out
        return out

    tfa._fit_axle_pacejka, tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = fitter, fsp, ekf
    try:
        yield
    finally:
        tfa._fit_axle_pacejka, tfa.fit_session_pacejka, tfa.estimate_sideslip_ekf_pacejka = (
            ORIG["fit"], ORIG["fsp"], ORIG["ekf"])
    assert tfa._fit_axle_pacejka is ORIG["fit"] and tfa.fit_session_pacejka is ORIG["fsp"]
    assert tfa.estimate_sideslip_ekf_pacejka is ORIG["ekf"]


def chain(S, code=None):
    """One production chain run; code None = unswapped production."""
    from modules.stability_analysis import (estimate_slip_angles, estimate_lateral_forces,
                                            estimate_cornering_stiffness)
    holder = {}
    t0 = time.perf_counter()
    if code is None:
        beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(S["state"], S["params"], S["data"],
                                                             "ekf_auto_pacejka", csv_path=S["path"])
    else:
        with swap(make_fitter(code), holder):
            beta, man, gate, fb, fr = tfa.resolve_sideslip_beta(S["state"], S["params"], S["data"],
                                                                 "ekf_auto_pacejka", csv_path=S["path"])
    t1 = time.perf_counter()
    slip = estimate_slip_angles(S["state"], beta, S["params"])
    forces = estimate_lateral_forces(S["state"], S["params"])
    cs = estimate_cornering_stiffness(slip, forces, S["state"], S["params"], S["data"]["laps"])
    t2 = time.perf_counter()
    return {"beta": beta, "man": man, "gate": gate, "fallback": fb, "fallback_reason": fr, "cs": cs,
            "raw": holder.get("raw"), "final": holder.get("final"), "t_fit": t1 - t0, "t_cs": t2 - t1}


# ------------------------------------------------------------------ comparison

def diff(a, b, path="", out=None, limit=10):
    """First differences between two nested results, NaN == NaN (from
    check_class_b.diff)."""
    out = [] if out is None else out
    if len(out) >= limit:
        return out
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in a or k not in b:
                out.append(f"{path}.{k} (only in one)")
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


def entry_check(S, prod, harn):
    m0, m1 = prod["man"], harn["man"]
    items = {
        "fit parameters": ({a: {k: m0["axles"][a][k] for k in "BCDE"} for a in ("front", "rear")},
                           {a: {k: m1["axles"][a][k] for k in "BCDE"} for a in ("front", "rear")}),
        "fit axles (all manifest fields)": (m0["axles"], m1["axles"]),
        "status": (m0.get("status"), m1.get("status")),
        "R pick": (m0.get("r_sweep", {}).get("chosen"), m1.get("r_sweep", {}).get("chosen")),
        "sign check": (m0.get("sign_check"), m1.get("sign_check")),
        "gate verdict/score": (prod["gate"], harn["gate"]),
        "fallback": ((prod["fallback"], prod["fallback_reason"]), (harn["fallback"], harn["fallback_reason"])),
        "beta_with_fallback (shipped beta)": (prod["beta"], harn["beta"]),
        "CS arrays (every key)": (prod["cs"], harn["cs"]),
    }
    ok = True
    for name, (a, b) in items.items():
        d = diff(a, b, name)
        ok &= not d
        log(f"    E1 {name}: {'BIT-IDENTICAL' if not d else 'DIFFERS: ' + '; '.join(d)}")
    return ok


def entry_check_stepb(S, harn):
    name = STEPB_NAMES.get(S["key"])
    if name is None or not os.path.exists(STEPB_LOG):
        log("    E2 not applicable (session not in stepB.log)")
        return True
    lines = open(STEPB_LOG, encoding="utf-8").read().splitlines()
    i = next(k for k, l in enumerate(lines) if l.startswith(name + ":"))
    new = next(l for l in lines[i:] if "[new] fit status" in l)
    mm = re.search(r"R pick ([\d.]+)/([\d.]+), sign check (\{.*?\}), B\*C\*D front (\d+) rear (\d+), "
                   r"fit samples front (\d+) rear (\d+), gate pass score ([\d.]+)", new)
    m = harn["man"]
    ch = m["r_sweep"]["chosen"]
    bcd = {a: m["axles"][a]["B"] * m["axles"][a]["C"] * m["axles"][a]["D"] for a in ("front", "rear")}
    sc = m["sign_check"]
    sc_rec = ast.literal_eval(mm.group(3))  # a dict literal printed by the R5 script
    checks = [
        ("R pick", (float(mm.group(1)), float(mm.group(2))) == (ch["r_ay_scale"], ch["r_yaw_scale"])),
        ("sign check", all(sc[k] == v for k, v in sc_rec.items())),
        ("B*C*D front", abs(bcd["front"] - int(mm.group(4))) <= 0.5),
        ("B*C*D rear", abs(bcd["rear"] - int(mm.group(5))) <= 0.5),
        ("fit samples", (m["axles"]["front"]["fit_n_samples"], m["axles"]["rear"]["fit_n_samples"])
         == (int(mm.group(6)), int(mm.group(7)))),
        ("gate score", f"{harn['gate']['health_score']:.4f}" == mm.group(8)),
    ]
    for k, v in checks:
        log(f"    E2 {k}: {'MATCH' if v else 'MISMATCH'}")
    log(f"       recorded: {new.strip()}")
    log(f"       now: R {ch['r_ay_scale']:g}/{ch['r_yaw_scale']:g}, BCD front {bcd['front']:.1f} rear "
        f"{bcd['rear']:.1f}, gate score {harn['gate']['health_score']:.6f}")
    return all(v for _, v in checks)


# ------------------------------------------------------------------ session

def load_session(key, label, rel):
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.stability_analysis import (load_parameters, prepare_vehicle_state, estimate_sideslip,
                                            estimate_slip_angles, estimate_lateral_forces)
    from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    path = resolve_data_path(rel)
    data = parse_csv(path)
    params = apply_resolved_vehicle(load_parameters(), resolve_accuracy(load_parameters(), setup_data=None, cap=None))
    assert params["stability_estimation"]["sideslip_source"] == "ekf_auto_pacejka"
    # the app reclassifies speed classes from the Settings boundaries before the fit
    reclassify_speed_classes(data.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    state = prepare_vehicle_state(data["channels"], params)
    S = {"key": key, "label": label, "path": path, "data": data, "params": params, "state": state}
    cfg = params["tyre_fit_auto"]
    base = tfa._base_mask(state, data["laps"])
    window, _ = tfa._speed_window_mask(state, data, cfg)
    S["base"], S["fit_mask"] = base, base & window
    slip_kin = estimate_slip_angles(state, estimate_sideslip(state, params), params)
    forces = estimate_lateral_forces(state, params)
    S["alpha"] = {"front": slip_kin["alpha_f_filt"], "rear": slip_kin["alpha_r_filt"]}
    S["Fy"] = {"front": forces["Fy_f_filt"], "rear": forces["Fy_r_filt"]}
    t, s_m = state["time"], state["s_m"]
    laps = {l["lap_number"]: l for l in data["laps"]}
    inst, in_br = [], np.zeros(t.size, dtype=bool)
    speed_class = np.full(t.size, "", dtype=object)
    for c in data["corners"]:
        lap = laps.get(c.get("lap_number"))
        if c.get("stable_corner_id") is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
            continue
        sl = tfa._canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                         c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            inst.append((c["stable_corner_id"], c["lap_number"], sl))
            in_br[sl] = True
            speed_class[sl] = str(c.get("speed_class"))
    inst.sort(key=lambda x: x[2].start)
    S["inst"], S["in_br"], S["speed_class"] = inst, in_br, speed_class
    moving = state["moving_mask"]
    kerb = state.get("kerb_mask")
    S["verdict_base"] = in_br & moving & (~kerb if kerb is not None else True)
    S["straight"] = base & ~in_br
    return S


class _Ctx:
    # the attributes inspect_stiffness_cs.grids reads
    def __init__(self, S):
        self.state, self.data, self.inst = S["state"], S["data"], S["inst"]


# ------------------------------------------------------------------ metrics

def curve(code_fit, a_rad):
    return pacejka_lateral_force(a_rad, code_fit["B"], code_fit["C"], code_fit["D"], code_fit["E"])


def m_curve(fit_v, fit_ref, S, axle, D_ref):
    a = np.abs(np.degrees(S["alpha"][axle][S["fit_mask"] & np.isfinite(S["alpha"][axle])
                                          & np.isfinite(S["Fy"][axle])]))
    sa = np.degrees(S["alpha"][axle][S["fit_mask"] & np.isfinite(S["alpha"][axle]) & np.isfinite(S["Fy"][axle])])
    p1, p99s = np.percentile(sa, [1, 99])
    q95, q99 = np.percentile(a, [95, 99])
    hi = S["params"]["tyre_fit_auto"]["peak_max_deg"]
    out = {}
    for name, lo_, hi_ in (("visited", p1, p99s), ("0-p95", 0.0, q95), ("p95-p99", q95, q99), ("p99-15deg", q99, hi)):
        if hi_ <= lo_:
            out[name] = {"rms": float("nan"), "max": float("nan")}
            continue
        g = np.radians(np.linspace(lo_, hi_, 2000))
        d = np.abs(curve(fit_v, g) - curve(fit_ref, g)) / D_ref
        out[name] = {"rms": float(np.sqrt(np.mean(d ** 2))), "max": float(np.max(d))}
    out["regions_deg"] = {"p1": float(p1), "p99_signed": float(p99s), "p95_abs": float(q95), "p99_abs": float(q99),
                          "outer": float(hi)}
    return out


def m_peak(fit):
    y = curve(fit, np.radians(PEAK_GRID_DEG))
    i = int(np.argmax(y))
    if not 0 < i < PEAK_GRID_DEG.size - 1:
        return float("nan")
    on = PEAK_GRID_DEG[y >= 0.95 * y[i]]
    return float(on.max() - on.min())


def fit_metrics(S, run, axle):
    m = run["man"]["axles"][axle]
    fit = {k: m[k] for k in "BCDE"}
    al, fy = S["alpha"][axle], S["Fy"][axle]
    pop = S["fit_mask"] & np.isfinite(al) & np.isfinite(fy)
    a, r = al[pop], fy[pop] - curve(fit, al[pop])
    ad = np.abs(np.degrees(a))
    rms_bins = {f"{lo}-{hi}": (float(np.sqrt(np.mean(r[(ad >= lo) & (ad < hi)] ** 2)))
                               if ((ad >= lo) & (ad < hi)).any() else float("nan")) for lo, hi in RESID_BINS_DEG}
    sc = S["speed_class"][pop]
    by_class = {k: float(np.mean(r[sc == k])) for k in sorted(set(sc) - {""})}
    raw = run["raw"] or {}
    d4 = (raw.get("plausibility_gate") or {}).get(axle)
    d4_src = "manifest"
    if d4 is None:
        d4 = tfa.plausibility_gate(al, S["fit_mask"], fit, S["params"]["tyre_fit_auto"])
        d4_src = "computed by the script (fit returned before D4)"
    return {
        **fit, "BCD": fit["B"] * fit["C"] * fit["D"], "peak_alpha_deg": m["peak_alpha_deg"],
        "p99_deg": m["visited_alpha_p99_deg"], "peak_covered": bool(m["peak_in_visited_range"]),
        "powell_converged": m["powell_converged"], "n": m["fit_n_samples"],
        "rms_N": float(np.sqrt(np.mean(r ** 2))), "rms_manifest_N": m["fit_rms_resid_N"], "rms_bins_N": rms_bins,
        "mean_resid_left_N": float(np.mean(r[a < 0])), "mean_resid_right_N": float(np.mean(r[a > 0])),
        "mean_resid_speed_class_N": by_class,
        "d4": {k: d4[k] for k in ("alpha_span_deg", "stiffness_npr", "peak_deg", "failed")}, "d4_source": d4_src,
        "m_peak_deg": m_peak(fit),
    }


def ekf_metrics(S, run):
    man, gate, fin = run["man"], run["gate"], run["final"]
    base = S["base"]
    out = {"status": man.get("status"), "degenerate_reason": man.get("degenerate_reason"),
           "marginal_reason": man.get("marginal_reason"), "fallback": run["fallback"],
           "fallback_reason": run["fallback_reason"]}
    rs = man.get("r_sweep")
    if rs:
        out["R"] = {"r_ay_scale": rs["chosen"]["r_ay_scale"], "r_yaw_scale": rs["chosen"]["r_yaw_scale"],
                    "selection": rs["selection"]}
    out["sign_fraction"] = (man.get("sign_check") or {}).get("median_gate_racing_fraction")
    out["gate"] = None if gate is None else {"verdict": gate["verdict"], "health_score": gate["health_score"]}
    if fin is not None:
        bound = np.radians(S["params"]["tyre_fit_auto"]["beta_hard_bound_deg"])
        bflag = np.abs(fin["beta"]) > bound
        out["hard_bound_reentries"] = int((bflag & base).sum())
        out["nis_window_divergences"] = int((fin["diverged_mask"] & ~bflag & base).sum())
    else:
        out["hard_bound_reentries"] = out["nis_window_divergences"] = None
    b = np.abs(np.degrees(run["beta"][S["straight"]]))
    b = b[np.isfinite(b)]
    out["beta_straight_abs_deg"] = {"median": float(np.median(b)), "p95": float(np.percentile(b, 95))}
    return out


def clean_cs(run, ax):
    cs = run["cs"]
    return np.where(cs[f"near_strike_{ax}"], np.nan, cs[f"CS_ratio_{ax}"])


def cs_metrics(S, run):
    from diagnostics.inspect_stiffness_cs import grids, decompose
    cs = run["cs"]
    out, pooled_fin, pooled_n, pooled_m2 = {}, 0, 0, 0
    for ax in ("f", "r"):
        pop = S["verdict_base"] & ~cs[f"near_strike_{ax}"]
        v = cs[f"CS_ratio_{ax}"][pop]
        fin = v[np.isfinite(v)]
        ref = cs[f"C_linear_ref_{ax}"]
        rf = ref[np.isfinite(ref)]
        n_set = int(1 + np.count_nonzero(np.diff(rf))) if rf.size else 0
        rp = ref[pop]
        rp = rp[np.isfinite(rp)]
        ref_med = float(np.median(rp)) if rp.size else float("nan")
        dec = decompose(grids(_Ctx(S), clean_cs(run, ax)))
        out[ax] = {
            "n_pop": int(pop.sum()), "finite_share": float(fin.size / max(1, v.size)),
            "n_no_reference_analysis": cs[f"n_no_reference_analysis_{ax}"], "reference_set_events": n_set,
            "reference_median_npr": ref_med,
            "reference_p5_over_median": float(np.percentile(rp, 5) / ref_med) if rp.size else float("nan"),
            "p1": float(np.percentile(fin, 1)), "p5": float(np.percentile(fin, 5)),
            "p10": float(np.percentile(fin, 10)), "p50": float(np.percentile(fin, 50)),
            "share_lt_m1": float(np.mean(fin < -1)), "share_lt_m2": float(np.mean(fin < -2)),
            "crosslap_residual_var": dec["residual_var"], "crosslap_signal_share": dec["signal_share"],
            "grid_coverage": dec["coverage"],
        }
        pooled_fin += fin.size
        pooled_n += v.size
        pooled_m2 += int(np.sum(fin < -2))
    out["pooled_share_lt_m2"] = pooled_m2 / max(1, pooled_fin)
    out["pooled_finite_share"] = pooled_fin / max(1, pooled_n)
    return out


def admissible(rec):
    reasons = []
    e = rec["ekf"]
    if e["status"] == "degenerate":
        reasons.append(f"fit degenerate ({e['degenerate_reason']})")
    for axle in ("front", "rear"):
        if rec["fit"][axle]["d4"]["failed"]:
            reasons.append(f"D4 {axle}: {', '.join(rec['fit'][axle]['d4']['failed'])}")
    if e["gate"] is not None and e["gate"]["verdict"] == "fail":
        reasons.append("NIS gate fail")
    if e["fallback"]:
        reasons.append("fallback used")
    return not reasons, reasons


# ------------------------------------------------------------------ scoring

def score_claim(values):
    vals = [bool(v) for v in values if v is not None]
    if not vals:
        return "NOT SCORABLE"
    return "CONFIRMED" if all(vals) else ("REFUTED" if not any(vals) else "MIXED")


def combine(scores):
    if all(s == "CONFIRMED" for s in scores):
        return "CONFIRMED"
    if all(s == "REFUTED" for s in scores):
        return "REFUTED"
    return "MIXED"


def predictions(R):
    keys = [k for k in R if R[k] is not None]
    sa = [(k, a) for k in keys for a in ("front", "rear")]
    F = lambda k, c, a: R[k]["variants"][c]["fit"][a]  # noqa: E731
    out = {}

    def pair_curve(k, c1, c2, a, region):
        return R[k]["pairs"][f"{c1}|{c2}"][a][region]["rms"]

    c1 = score_claim([abs(F(k, "P13", a)["C"] - 1.3) <= 0.1 for k, a in sa])
    c2 = score_claim([R[k]["variants"]["P13"]["m_curve"][a]["visited"]["rms"] < BAR for k, a in sa])
    out["P13"] = {"claims": {"C near 1.3": c1, "visited curve within bar": c2}, "verdict": combine([c1, c2])}

    rel = lambda k, c, a: R[k]["variants"][c]["m_rel"][a]  # noqa: E731
    mc = lambda k, c, a, reg: R[k]["variants"][c]["m_curve"][a][reg]["rms"]  # noqa: E731
    ek = lambda k, c: R[k]["variants"][c]["ekf"]["hard_bound_reentries"]  # noqa: E731
    lt = lambda x, y: None if x is None or y is None else x < y  # noqa: E731
    c1 = score_claim([rel(k, "E0", a)["BCD"] < BAR and rel(k, "E0", a)["D"] < BAR for k, a in sa])
    c2 = score_claim([mc(k, "E0", a, "0-p95") < BAR and mc(k, "E0", a, "p99-15deg") >= mc(k, "E0", a, "0-p95")
                      for k, a in sa])
    c3 = score_claim([lt(ek(k, "E0"), ek(k, "P")) for k in keys])
    out["E0"] = {"claims": {"BCD and D within bar": c1, "differences only beyond the peak": c2,
                            "fewer hard-bound re-entries": c3}, "verdict": combine([c1, c2, c3])}

    c1 = score_claim([pair_curve(k, "E0C13", "E0C19", a, "visited") < BAR for k, a in sa])
    c2 = score_claim([pair_curve(k, "E0C13", "E0C19", a, "p99-15deg") > BAR for k, a in sa])
    c3 = score_claim([lt(ek(k, "E0C13"), ek(k, "E0C19")) for k in keys])
    c4 = score_claim([R[k]["variants"]["E0C13"]["cs"]["pooled_share_lt_m2"]
                      < R[k]["variants"]["E0C19"]["cs"]["pooled_share_lt_m2"] for k in keys])
    out["E0C19 vs E0C13"] = {"claims": {"identical inside visited range": c1, "different beyond p99": c2,
                                        "C 1.3 fewer re-entries": c3, "C 1.3 lower share CS < -2": c4},
                             "verdict": combine([c1, c2, c3, c4])}

    c1 = score_claim([pair_curve(k, "E97C13", "E0C13", a, "visited") < BAR for k, a in sa])
    c2 = score_claim([None if np.isnan(F(k, "E97C13", a)["m_peak_deg"]) or np.isnan(F(k, "E0C13", a)["m_peak_deg"])
                      else F(k, "E97C13", a)["m_peak_deg"] > F(k, "E0C13", a)["m_peak_deg"] for k, a in sa])
    out["E97C13"] = {"claims": {"as E0C13 inside visited range": c1, "flatter peak region": c2},
                     "verdict": combine([c1, c2])}

    for wc, twin in (("Pw", "P"), ("E0w", "E0")):
        bcd = lambda k, c, a: F(k, c, a)["BCD"]  # noqa: E731
        c1 = score_claim([abs(bcd(k, wc, a) / bcd(k, twin, a) - 1) > BAR for k, a in sa])
        out[wc] = {"claims": {f"BCD differs from {twin} beyond bar": c1}, "verdict": combine([c1]),
                   "direction (reported, not predicted)": {f"{k} {a}": bcd(k, wc, a) / bcd(k, twin, a) - 1
                                                           for k, a in sa}}
    return out


def beats_p(R):
    keys = [k for k in R if R[k] is not None]
    out = {}
    for c in VARIANTS:
        if c == "P":
            continue
        per = {}
        for k in keys:
            v, p = R[k]["variants"][c]["cs"], R[k]["variants"]["P"]["cs"]
            a = v["pooled_share_lt_m2"] < p["pooled_share_lt_m2"]
            b = all(v[x]["crosslap_residual_var"] <= 1.05 * p[x]["crosslap_residual_var"] for x in ("f", "r"))
            cc = v["pooled_finite_share"] >= p["pooled_finite_share"] - 0.02
            per[k] = {"a_share_lt_m2_lower": a, "b_residual_not_worse_5pct": b, "c_finite_share_ok": cc,
                      "all": a and b and cc}
        n = sum(x["all"] for x in per.values())
        adm = all(R[k]["variants"][c]["admissible"] for k in keys)
        out[c] = {"per_session": per, "sessions_met": n, "beats_p_rule": n >= 2, "admissible": adm,
                  "beats_p_counts": n >= 2 and adm,
                  "makes_admissible": [k for k in keys if R[k]["variants"][c]["admissible"]
                                       and not R[k]["variants"]["P"]["admissible"]]}
    return out


# ------------------------------------------------------------------ renders

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def render_fit(S, fits, axle, path):
    plt = _plt()
    al, fy = S["alpha"][axle], S["Fy"][axle]
    pop = S["fit_mask"] & np.isfinite(al) & np.isfinite(fy)
    a, f = np.degrees(al[pop]), fy[pop] / 1000.0
    p99 = np.percentile(np.abs(a), 99)
    lim = max(15.0, 1.2 * p99)
    from matplotlib.colors import LogNorm
    f_lim = 1.25 * float(np.percentile(np.abs(f), 99.9))
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.hist2d(a, f, bins=[np.linspace(-lim, lim, 240), np.linspace(-f_lim, f_lim, 160)], cmap="Greys", cmin=1,
              norm=LogNorm())
    g = np.radians(np.linspace(-lim, lim, 1500))
    for code, fit in fits.items():
        col, ls = STYLE[code]
        ax.plot(np.degrees(g), curve(fit, g) / 1000.0, color=col, ls=ls, lw=2.4 if code == "P" else 1.6,
                label=VARIANTS[code]["words"])
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-f_lim, f_lim)
    for x in (-p99, p99):
        ax.axvline(x, color="#6b7280", lw=0.8, ls=":")
    ax.text(p99, ax.get_ylim()[1], " 99 % of samples inside", va="top", fontsize=8, color="#6b7280")
    ax.set_xlabel("axle slip angle from the simple geometric estimate [deg]")
    ax.set_ylabel(f"{axle} axle lateral force [kN]")
    ax.set_title(f"{S['label']}, {axle} axle: fitted tyre curves, simplification variants\n"
                 f"grey: sample density of the fit population (log scale)")
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_csdist(S, runs, path):
    plt = _plt()
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), sharey=True)
    for ax, x, name in ((axs[0], "f", "front"), (axs[1], "r", "rear")):
        for code, run in runs.items():
            cs = run["cs"]
            pop = S["verdict_base"] & ~cs[f"near_strike_{x}"]
            v = cs[f"CS_ratio_{x}"][pop]
            v = np.sort(v[np.isfinite(v)])
            col, ls = STYLE[code]
            ax.plot(v, np.arange(1, v.size + 1) / v.size, color=col, ls=ls, lw=2.2 if code == "P" else 1.4,
                    label=VARIANTS[code]["words"])
        ax.axvline(-2, color="#6b7280", lw=0.8, ls=":")
        ax.axvline(-1, color="#6b7280", lw=0.8, ls=":")
        ax.set_xlim(-3.5, 1.05)
        ax.set_xlabel(f"{name} cornering stiffness ratio")
        ax.grid(alpha=0.25)
        ax.set_title(f"{name} axle")
    axs[0].set_ylabel("share of samples at or below")
    axs[1].legend(fontsize=8, loc="upper left")
    fig.suptitle(f"{S['label']}: cornering stiffness ratio distribution per tyre-fit variant\n"
                 f"corner samples on analysis laps, kerb-affected samples excluded")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render_traces(S, run, code, path):
    from diagnostics.trace_render import render_cs_traces
    cs = run["cs"]
    render_cs_traces(path, S["state"]["time"], S["state"]["s_m"], cs["CS_ratio_f"], cs["CS_ratio_r"],
                     S["data"]["laps"], S["data"]["corners"],
                     f"{S['label']}: cornering stiffness ratio front and rear, tyre fit: {VARIANTS[code]['words']}",
                     near_strike={"f": cs["near_strike_f"], "r": cs["near_strike_r"]})


# ------------------------------------------------------------------ main

def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if o != o else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def main():
    global _LOG
    os.makedirs(OUT_DIR, exist_ok=True)
    _LOG = open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8")
    keep_awake(True)
    t_start = time.perf_counter()
    try:
        R = {}
        sessions = {}
        # entry check on every session first; any mismatch stops before variants
        for key, label, rel in SESSIONS:
            log("=" * 100)
            log(f"{label} ({key})")
            S = load_session(key, label, rel)
            sessions[key] = S
            log(f"  {S['path']}; analysis laps "
                f"{[l['lap_number'] for l in S['data']['laps'] if l.get('is_valid_for_analysis')]}; "
                f"corner instances {len(S['inst'])}")
            prod = chain(S, None)
            log(f"  production run: fit chain {prod['t_fit']:.0f} s, CS {prod['t_cs']:.0f} s")
            harn = chain(S, "P")
            log(f"  harness run P: fit chain {harn['t_fit']:.0f} s, CS {harn['t_cs']:.0f} s")
            ok = entry_check(S, prod, harn) and entry_check_stepb(S, harn)
            if not ok:
                log("ENTRY CHECK FAILED -- stop, no variant runs.")
                return 1
            S["runs"] = {"P": harn}
            del prod
        log("\nENTRY CHECK PASSED on all sessions.\n")

        for key, S in sessions.items():
            log("=" * 100)
            log(f"{S['label']}: variants")
            for code in VARIANTS:
                if code != "P":
                    S["runs"][code] = chain(S, code)
                    log(f"  {code}: fit chain {S['runs'][code]['t_fit']:.0f} s, CS {S['runs'][code]['t_cs']:.0f} s, "
                        f"status {S['runs'][code]['man'].get('status')}, fallback {S['runs'][code]['fallback']}")
            rec = {"label": S["label"], "path": S["path"], "variants": {}, "pairs": {}}
            fits = {c: {a: {k: r["man"]["axles"][a][k] for k in "BCDE"} for a in ("front", "rear")}
                    for c, r in S["runs"].items()}
            for code, run in S["runs"].items():
                v = {"fit": {a: fit_metrics(S, run, a) for a in ("front", "rear")}, "ekf": ekf_metrics(S, run),
                     "cs": cs_metrics(S, run)}
                v["m_curve"] = {a: m_curve(fits[code][a], fits["P"][a], S, a, fits["P"][a]["D"])
                                for a in ("front", "rear")}
                v["m_rel"] = {a: {"BCD": abs(v["fit"][a]["BCD"] / (fits["P"][a]["B"] * fits["P"][a]["C"]
                                                                    * fits["P"][a]["D"]) - 1),
                                  "D": abs(fits[code][a]["D"] / fits["P"][a]["D"] - 1)} for a in ("front", "rear")}
                v["admissible"], v["inadmissible_reasons"] = admissible(v)
                rec["variants"][code] = v
            for c1, c2 in (("E0C13", "E0C19"), ("E97C13", "E0C13")):
                rec["pairs"][f"{c1}|{c2}"] = {a: m_curve(fits[c1][a], fits[c2][a], S, a, fits[c2][a]["D"])
                                              for a in ("front", "rear")}
            R[key] = rec
            for a in ("front", "rear"):
                render_fit(S, {c: fits[c][a] for c in fits}, a, os.path.join(OUT_DIR, f"fit_{key}_{a}.png"))
            render_csdist(S, S["runs"], os.path.join(OUT_DIR, f"csdist_{key}.png"))
            for code, run in S["runs"].items():
                render_traces(S, run, code, os.path.join(OUT_DIR, f"traces_{key}_{code}.png"))
            with open(os.path.join(OUT_DIR, f"fit_simplification_{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(_jsonable(rec), fh, indent=1)
            S["runs"] = None  # free the arrays before the next session
            log(f"  {S['label']} done ({(time.perf_counter() - t_start) / 60:.1f} min elapsed)")

        summary = {"beats_p": beats_p(R), "predictions": predictions(R)}
        with open(os.path.join(OUT_DIR, "summary.json"), "w", encoding="utf-8") as fh:
            json.dump(_jsonable(summary), fh, indent=1)
        report(R, summary)
        log(f"\nTOTAL {(time.perf_counter() - t_start) / 60:.1f} min")
        return 0
    finally:
        keep_awake(False)
        _LOG.close()


def report(R, summary):
    f3 = lambda x: "nan" if x is None or x != x else f"{x:.3f}"  # noqa: E731
    log("\n" + "=" * 100 + "\nREPORT")
    for key, rec in R.items():
        log(f"\n## {rec['label']}")
        for code, v in rec["variants"].items():
            log(f"- {code} ({VARIANTS[code]['words']}): admissible {'yes' if v['admissible'] else 'NO'}"
                + ("" if v["admissible"] else f" -- {'; '.join(v['inadmissible_reasons'])}"))
            for a in ("front", "rear"):
                f, mc, mr = v["fit"][a], v["m_curve"][a], v["m_rel"][a]
                log(f"    {a}: B {f['B']:.4g} C {f['C']:.4g} D {f['D']:.6g} E {f['E']:.4g} BCD {f['BCD']:.6g} "
                    f"peak {f3(f['peak_alpha_deg'])} p99 {f3(f['p99_deg'])} covered {f['peak_covered']} "
                    f"conv {f['powell_converged']} rms {f['rms_N']:.0f} N bins "
                    f"{ {k: round(x) if x == x else None for k, x in f['rms_bins_N'].items()} } "
                    f"L/R {f['mean_resid_left_N']:.0f}/{f['mean_resid_right_N']:.0f} N "
                    f"by class { {k: round(x) for k, x in f['mean_resid_speed_class_N'].items()} } "
                    f"D4 {f['d4']['failed'] or 'pass'} (BCD {f['d4']['stiffness_npr']:.0f}, peak "
                    f"{f3(f['d4']['peak_deg'])}, span {f3(f['d4']['alpha_span_deg'])}) M-peak {f3(f['m_peak_deg'])}")
                log(f"       M-curve vs P rms/max: visited {f3(mc['visited']['rms'])}/{f3(mc['visited']['max'])}, "
                    f"0-p95 {f3(mc['0-p95']['rms'])}, p95-p99 {f3(mc['p95-p99']['rms'])}, p99-15deg "
                    f"{f3(mc['p99-15deg']['rms'])}; M-rel BCD {f3(mr['BCD'])} D {f3(mr['D'])}")
            e = v["ekf"]
            log(f"    EKF: status {e['status']} {e.get('marginal_reason') or e.get('degenerate_reason') or ''}; "
                f"R {e.get('R')}; sign {f3(e['sign_fraction'])}; gate {e['gate']}; fallback {e['fallback']}; "
                f"hard-bound re-entries {e['hard_bound_reentries']}; NIS-window divergences "
                f"{e['nis_window_divergences']}; straight |beta| median/p95 {f3(e['beta_straight_abs_deg']['median'])}/"
                f"{f3(e['beta_straight_abs_deg']['p95'])} deg")
            c = v["cs"]
            for x in ("f", "r"):
                q = c[x]
                log(f"    CS {x}: finite {f3(q['finite_share'])} (n {q['n_pop']}), no-ref {q['n_no_reference_analysis']}, "
                    f"ref events {q['reference_set_events']}, ref median {q['reference_median_npr']:.0f} p5/med "
                    f"{f3(q['reference_p5_over_median'])}, p1/p5/p10/p50 {f3(q['p1'])}/{f3(q['p5'])}/{f3(q['p10'])}/"
                    f"{f3(q['p50'])}, <-1 {f3(q['share_lt_m1'])}, <-2 {f3(q['share_lt_m2'])}, cross-lap residual "
                    f"{f3(q['crosslap_residual_var'])} (signal share {f3(q['crosslap_signal_share'])})")
            log(f"    CS pooled: share < -2 {c['pooled_share_lt_m2']:.4f}, finite share {c['pooled_finite_share']:.4f}")
    log("\n## BEATS P")
    for c, b in summary["beats_p"].items():
        log(f"- {c}: sessions met {b['sessions_met']}/3 -> rule {'MET' if b['beats_p_rule'] else 'not met'}; "
            f"admissible {b['admissible']}; counts {b['beats_p_counts']}; makes P-inadmissible session admissible "
            f"{b['makes_admissible'] or 'none'}; {b['per_session']}")
    log("\n## PREDICTIONS")
    for c, p in summary["predictions"].items():
        log(f"- {c}: {p['verdict']} -- {p['claims']}"
            + (f"; direction {p['direction (reported, not predicted)']}" if "direction (reported, not predicted)" in p
               else ""))


if __name__ == "__main__":
    sys.exit(main())
