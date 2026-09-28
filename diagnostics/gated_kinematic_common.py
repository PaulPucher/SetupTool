# Shared helpers for the gated-kinematic sideslip diagnostic (read-only).
#
# Question under test: does integrating the kinematic identity
# ay = v*(beta_dot + psi_dot) ONLY while cornering (|ay| above a gate
# threshold, integrator reset to zero outside) remove the washout
# artefacts of the production 0.05 Hz continuous washout, and how does it
# compare with ekf_auto_pacejka? Evidence for the blocked washout-cutoff
# decision; nothing here is wired to production and no config is written.
#
# Tier B preprocessing variant of an existing Tier A identity (Rajamani
# ch. 2, same anchor as modules.stability_analysis.estimate_sideslip) --
# the gate itself is a standard segmentation mask, its threshold derived
# from the session's own |ay| distribution (Otsu), never hardcoded.

import os
import pickle

import numpy as np
from scipy.signal import butter, lfilter

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, BUTTERWORTH_ORDER,
)
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle

SESSIONS = {
    "dubai": "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt",
    "v3": "C:/UNI/Bachelorarbeit/Setuptool_local/GT3_PRC_MLA-v3.txt",
}
OUT_DIR = "diagnostics/plots_gated_kinematic"
CACHE_DIR = os.path.join(OUT_DIR, "_cache")

# Gate-threshold sensitivity grid: multiples of the Otsu value.
GATE_FACTORS = (0.75, 1.0, 1.5)

# cap=1 (config defaults) for the same reason the goldens use it: results
# must not depend on machine-local car_data.json or DB setup sheets.
FIXED_CAP = 1


def load_session(name):
    data = parse_csv(SESSIONS[name])
    raw = load_parameters()
    params = apply_resolved_vehicle(raw, resolve_accuracy(raw, setup_data=None, cap=FIXED_CAP))
    state = prepare_vehicle_state(data["channels"], params)
    return data, params, state


def racing_mask(state, laps):
    """Moving, off-kerb samples inside analysis-valid laps."""
    t = state["time"]
    m = state["moving_mask"].copy()
    kerb = state.get("kerb_mask")
    if kerb is not None:
        m &= ~kerb
    in_lap = np.zeros_like(t, dtype=bool)
    for lap in laps:
        if lap.get("is_valid_for_analysis"):
            in_lap |= (t >= lap["start_time"]) & (t <= lap["end_time"])
    return m & in_lap


def otsu_threshold(x, nbins=256):
    """Otsu's between-class-variance maximiser on a 1-D sample. |ay| is
    bimodal (straights near 0, corners well above), so the valley the
    method finds is the data's own straight/corner boundary."""
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    hist, edges = np.histogram(x, bins=nbins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    w = hist.astype(float)
    w0 = np.cumsum(w)
    w1 = w0[-1] - w0
    m0 = np.cumsum(w * centers)
    mu0 = np.divide(m0, w0, out=np.zeros_like(m0), where=w0 > 0)
    mu1 = np.divide(m0[-1] - m0, w1, out=np.zeros_like(m0), where=w1 > 0)
    between = w0 * w1 * (mu0 - mu1) ** 2
    return float(centers[int(np.argmax(between))])


def beta_dot(state):
    v = state["v_mps"]
    moving = state["moving_mask"]
    v_safe = np.where(moving, v, 1.0)
    return np.where(moving, state["ay_mps2"] / v_safe - state["yaw_rate_radps"], 0.0)


def gated_beta(state, threshold):
    """Causal gated integration. Inside a gate run (|ay| > threshold while
    moving) beta is the integral of beta_dot from the run's first sample;
    outside, beta is held at 0 (straight-line sideslip taken as zero). Each
    new run restarts from 0, so drift cannot carry from one corner into the
    next -- but drift accumulated inside a run is kept, uncorrected.
    Returns (beta, gate, run_id) with run_id = -1 outside the gate."""
    dt = 1.0 / state["sample_rate_hz"]
    gate = state["moving_mask"] & (np.abs(state["ay_mps2"]) > threshold)
    inc = np.where(gate, beta_dot(state) * dt, 0.0)
    starts = gate & ~np.concatenate(([False], gate[:-1]))
    run_id = np.cumsum(starts) - 1
    run_id = np.where(gate, run_id, -1)
    cs = np.cumsum(inc)
    # subtract the cumulative value just before each run's first sample
    base_at_start = cs[starts] - inc[starts]
    beta = np.zeros_like(cs)
    beta[gate] = cs[gate] - base_at_start[run_id[gate]]
    return beta, gate, run_id


def causal_washout_beta(state, cutoff_hz):
    """Causal (lfilter) counterpart of the production filtfilt washout, same
    Butterworth order. Production filtfilt is zero-phase/acausal: its value
    after a corner exit already knows the future, so drift claims are only
    made on this causal version."""
    sr = state["sample_rate_hz"]
    b, a = butter(BUTTERWORTH_ORDER, cutoff_hz / (0.5 * sr), btype="high")
    raw = np.cumsum(beta_dot(state)) / sr
    return np.where(state["moving_mask"], lfilter(b, a, raw), 0.0)


def canonical_window_slice(t, s_m, lap_start_t, lap_end_t, bracket_start_m, bracket_end_m):
    # Same lap-time -> lap-distance intersection as
    # diagnostics/inspect_step2_chair_plots.py.
    lo = int(np.searchsorted(t, lap_start_t, side="left"))
    hi = int(np.searchsorted(t, lap_end_t, side="right"))
    if hi <= lo:
        return slice(0, 0)
    lap_s = s_m[lo:hi]
    finite = np.isfinite(lap_s)
    if not finite.any():
        return slice(0, 0)
    start_s = max(float(np.min(lap_s[finite])), bracket_start_m)
    end_s = min(float(np.max(lap_s[finite])), bracket_end_m)
    a = int(np.searchsorted(lap_s, start_s, side="left"))
    b = int(np.searchsorted(lap_s, end_s, side="right"))
    return slice(lo + a, lo + b)


def valid_corner_instances(data, t, s_m):
    """[(stable_id, lap_number, slice)] for every corner instance on an
    analysis-valid lap, time-ordered."""
    laps = {l["lap_number"]: l for l in data.get("laps", [])}
    out = []
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        lap = laps.get(c.get("lap_number"))
        if sid is None or lap is None or not lap.get("is_valid_for_analysis"):
            continue
        if c.get("bracket_start_m") is None or c.get("bracket_end_m") is None:
            continue
        sl = canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                    c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop > sl.start:
            out.append((sid, c["lap_number"], sl))
    out.sort(key=lambda x: x[2].start)
    return out


def cache_path(name):
    return os.path.join(CACHE_DIR, f"{name}.pkl")


def save_cache(name, obj):
    os.makedirs(CACHE_DIR, exist_ok=True)
    with open(cache_path(name), "wb") as f:
        pickle.dump(obj, f)


def load_cache(name):
    with open(cache_path(name), "rb") as f:
        return pickle.load(f)
