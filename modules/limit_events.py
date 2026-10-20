# Limit events: single in-corner moments where an axle's cornering stiffness
# ratio (CS) falls below the floor of the session's own fitted tyre curve --
# force loss the quasi-steady characteristic cannot produce anywhere. A third
# output class beside the corner verdicts (tendencies) and the recommendations;
# display and record only, never a verdict or recommendation input.
#
# Lineage: flagging handling moments in logged traces after Segers (secs.
# 7.4-7.6; sec. 5.4 for a discrete-event precedent); the event LINE is the
# session's fitted Magic Formula curve (Pacejka secs. 1.2.2 / 4.3.1);
# interpretation per Hoffman / Ono / Werner, limit-regime vocabulary per
# Milliken sec. 8.5. The event channel itself -- CS against the curve floor --
# is SetupTool's construction (Tier B segmentation on a Tier A-derived line).
#
# The floor is the curve's own analogue of CS: min over slip angle of
# (dFy/dalpha) / (B C D) -- 1 at the origin, 0 at the peak, negative past it --
# promoted unchanged from the curve-floor check (inspect_curve_floor).
# Two caveats, both deliberate (author rulings 2026-10-06):
# - The floor is taken over 0 .. 30 deg, beyond the visited slip range: the
#   claim is "beyond what this characteristic produces ANYWHERE". The value
#   is the extrapolated curve's own, not evidence the car went there.
# - Scale mismatch: CS divides by the running linear reference from data,
#   the floor by the curve's origin slope B C D. On the recorded sessions the
#   floor expressed on the CS scale sits deeper, so this line is the
#   conservative one -- it errs toward fewer events. No second normalisation.
#
# Fit L (the default since 2026-10-20): the fitted curve is monotonic (C = 1),
# so its floor is a small positive slope and every falling stretch would be an
# event. Under fit L the layer IS the floor events (author ruling 2026-10-20,
# one concept, one name): line = PURE_LATERAL_FLOOR, contiguous runs, measured
# attributes. The curve-floor layer above stays for the old per_session path.

import numpy as np

from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples
from modules.tyre_fit_auto import _canonical_window_slice
from modules.tyre_model_pacejka import pacejka_lateral_stiffness

# method-defining, not tunables: grid resolution of the floor search (as in
# the curve-floor check) and the slip range that "anywhere" means
FLOOR_GRID_N = 4000
EVENT_FLOOR_MAX_DEG = 30.0

# Pure-lateral floor of the CS ratio (author ruling 2026-10-19): the steepest
# post-peak slope of the Magic Formula over its initial slope, at C = 3.0
# (Goblirsch et al. 2025 Table I upper bound) with E in [-1, 1] (Pacejka
# 2006 ch. 4). Independent of B and D. Lower than any pure-slip curve can go,
# so a ratio below it is "not explainable by pure lateral saturation". A
# label, never a deletion or a verdict input. Method-defining, not a tunable.
PURE_LATERAL_FLOOR = -0.53

AXLES = {"f": "front", "r": "rear"}
LINE_CURVE = "fitted-curve floor"
LINE_PURE = "pure-lateral floor"
PHASE_ORDER = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


def curve_floor(axle_params, hi_deg):
    """(floor, slip angle of the floor in deg) of one axle's Magic Formula
    curve over 0 .. hi_deg, or (nan, nan) for an unusable parameter set."""
    B, C, D, E = (float(axle_params.get(k, np.nan)) for k in "BCDE")
    if not all(np.isfinite([B, C, D, E])) or B * C * D == 0:
        return float("nan"), float("nan")
    g = np.radians(np.linspace(0.0, hi_deg, FLOOR_GRID_N))
    k = pacejka_lateral_stiffness(g, B, C, D, E) / (B * C * D)
    i = int(np.argmin(k))
    return float(k[i]), float(np.degrees(g[i]))


def _event_line(fit_manifest, gate_verdict=None):
    """{"f": floor, "r": floor} or a reason string. A degenerate fit, or one
    whose NIS gate failed, is a curve production itself refuses to use (the
    session ships kinematic beta) -> no line from it (R14)."""
    if not fit_manifest or not fit_manifest.get("axles"):
        return "no event line: no tyre fit in this run (sideslip source without a per-session fit)"
    if fit_manifest.get("status") == "degenerate":
        return "no event line: the session's tyre fit is degenerate"
    if gate_verdict is not None and gate_verdict.get("verdict") == "fail":
        return "no event line: the session's NIS gate failed (kinematic sideslip shipped)"
    floors = {}
    for ax, name in AXLES.items():
        fl, _deg = curve_floor(fit_manifest["axles"].get(name, {}), EVENT_FLOOR_MAX_DEG)
        if not np.isfinite(fl):
            return f"no event line: {name} curve parameters unusable"
        floors[ax] = fl
    return floors


def _phase_at(segments, t_query):
    # apex_3 is one instant -> it wins only when the query sits on it
    for p in PHASE_ORDER:
        lo, hi = segments[p]
        if hi >= lo and lo <= t_query <= hi:
            return p
    # outside every segment (bridged into a margin) -> nearest boundary
    return min(PHASE_ORDER, key=lambda p: min(abs(t_query - segments[p][0]), abs(t_query - segments[p][1])))


def _phases_spanned(segments, t0, t1):
    return [p for p in PHASE_ORDER
            if segments[p][1] >= segments[p][0] and segments[p][0] <= t1 and segments[p][1] >= t0]


def _runs(mask):
    d = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


def detect_events(cs, state, corners, laps, fit_manifest, params, slip=None, gate_verdict=None,
                  axle_loads=None, axle_fx=None):
    """Event layer of one session. Returns a JSON-safe dict:
    {"status", "line", "floors", "events": [...], "n_instances_by_corner"}.

    Fit L (fit_manifest tyre_fit_mode literature_bounded): the floor events of
    detect_floor_events, line PURE_LATERAL_FLOOR; axle_loads / axle_fx feed
    their attributes. Old path: limit events against the fitted-curve floor.

    Population: in-corner samples (canonical brackets) on analysis laps, the
    FULL CS arrays -- kerb-affected samples are kept and labelled, so nothing
    is hidden (the classifier's clean population is a separate concern).
    Old path: stretches below the line merge across gaps shorter than
    bridge_gap_s, never across corner instances.
    """
    cfg = params["limit_events"]
    out = {"status": "computed", "floors": None, "events": [], "n_instances_by_corner": {}}
    if not cfg.get("enabled", True):
        out["status"] = "disabled in config"
        return out
    if fit_manifest and fit_manifest.get("tyre_fit_mode") == "literature_bounded":
        fe = detect_floor_events(cs, state, corners, laps, params, slip, axle_loads=axle_loads, axle_fx=axle_fx)
        # the -0.53 line is model-independent, so a degenerate fit or a failed
        # gate does not withdraw it; which beta the CS came from is stated
        fell_back = (fit_manifest.get("status") == "degenerate"
                     or (gate_verdict is not None and gate_verdict.get("verdict") == "fail"))
        if fell_back:
            fe["beta_note"] = "kinematic sideslip (fit L fell back)"
        label_recurrence(fe["events"], cfg["recurrence_radius_m"])
        return fe
    # old path: record unchanged (no "line" key); its absence means the curve floor
    line = _event_line(fit_manifest, gate_verdict)
    if isinstance(line, str):
        out["status"] = line
        return out
    out["floors"] = {AXLES[a]: v for a, v in line.items()}
    t, s_m = state["time"], state.get("s_m")
    if s_m is None:
        out["status"] = "no events: lap distance unusable"
        return out

    sr = state["sample_rate_hz"]
    bridge = cfg["bridge_gap_s"]
    min_dur = cfg.get("min_duration_s")
    kerb = state.get("kerb_mask")
    se = params["stability_estimation"]
    min_window = resolve_cs_min_window_samples(params, sr)

    laps_by = {l["lap_number"]: l for l in laps if l.get("is_valid_for_analysis")}
    events = []
    for c in corners:
        cid = c.get("stable_corner_id")
        lap = laps_by.get(c.get("lap_number"))
        if cid is None or lap is None or c.get("bracket_start_m") is None:
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                     c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        out["n_instances_by_corner"][str(cid)] = out["n_instances_by_corner"].get(str(cid), 0) + 1
        for ax in AXLES:
            seg = cs[f"CS_ratio_{ax}"][sl]
            below = np.isfinite(seg) & (seg < line[ax])
            runs = _runs(below)
            merged = []
            for a, b in runs:
                if merged and t[sl.start + a] - t[sl.start + merged[-1][1]] < bridge:
                    merged[-1] = (merged[-1][0], b)
                else:
                    merged.append((a, b))
            ns = cs.get(f"near_strike_{ax}")
            for a, b in merged:
                i0, i1 = sl.start + a, sl.start + b
                rng = np.arange(i0, i1 + 1)
                hit = rng[below[a:b + 1]]
                i_min = int(hit[np.argmin(cs[f"CS_ratio_{ax}"][hit])])
                duration = float(t[i1] - t[i0] + 1.0 / sr)
                # half-sample tolerance: an event of exactly min_dur (one CS
                # window) must not drop on float rounding of the grid times
                if min_dur is not None and duration < min_dur - 0.5 / sr:
                    continue
                kerb_hit = bool((ns is not None and ns[hit].any())
                                or (kerb is not None and kerb[i0:i1 + 1].any()))
                ev = {
                    "lap": int(c["lap_number"]), "corner": int(cid), "axle": AXLES[ax],
                    "phase": _phase_at(c["segments"], float(t[i_min])),
                    "phases_spanned": _phases_spanned(c["segments"], float(t[i0]), float(t[i1])),
                    "t_start": float(t[i0]), "t_end": float(t[i1]), "duration_s": duration,
                    "s_start_m": float(s_m[i0]), "s_end_m": float(s_m[i1]), "s_min_m": float(s_m[i_min]),
                    "cs_min": float(cs[f"CS_ratio_{ax}"][i_min]), "floor": float(line[ax]),
                    "kerb_affected": kerb_hit,
                }
                if slip is not None:
                    # what the estimator saw: start of the regression window
                    # [start, i0) behind the first event sample
                    w0 = reconstruct_cs_window_start(slip[f"alpha_{ax}_filt"], i0, min_window,
                                                     se["cs_min_slip_angle_span_rad"], s_m, se["cs_max_window_m"])
                    ev["window_start_t"] = float(t[w0])
                events.append(ev)
    label_recurrence(events, cfg["recurrence_radius_m"])
    events.sort(key=lambda e: (e["lap"], e["t_start"]))
    out["events"] = events
    return out


def detect_floor_events(cs, state, corners, laps, params, slip, axle_loads=None, axle_fx=None):
    """CS ratio below PURE_LATERAL_FLOOR, as events with measured attributes
    (author ruling 2026-10-20). JSON-safe {"status", "floor", "label",
    "events": [...]}.

    An event is one contiguous run of finite ratio < floor inside a corner
    instance (canonical bracket, analysis laps), per axle -- no bridging: a
    neighbouring CS sample shares most of its regression window, so a slope
    held over a flat-alpha stretch is one observation, not many. Full CS
    arrays: kerb-affected and observer-reset-affected runs are kept and
    flagged. Attributes only; nothing is classified or removed.

    axle_loads / axle_fx: the fit-L axle Fz and Eq. 9 axle Fx (2 Hz
    zero-phase), for the longitudinal utilisation and the load change inside
    the regression windows; without them those attributes are None.
    """
    out = {"status": "computed", "line": LINE_PURE, "floor": PURE_LATERAL_FLOOR,
           "floors": {name: PURE_LATERAL_FLOOR for name in AXLES.values()},
           "label": "not explainable by pure lateral saturation", "events": [], "n_instances_by_corner": {}}
    t, s_m = state["time"], state.get("s_m")
    if s_m is None:
        out["status"] = "no events: lap distance unusable"
        return out
    sr = state["sample_rate_hz"]
    se = params["stability_estimation"]
    min_window = resolve_cs_min_window_samples(params, sr)
    kerb = state.get("kerb_mask")
    kerb_any = kerb if kerb is not None else np.zeros(t.size, dtype=bool)
    if state.get("kerb_smear_zone") is not None:
        kerb_any = kerb_any | state["kerb_smear_zone"]

    laps_by = {l["lap_number"]: l for l in laps if l.get("is_valid_for_analysis")}
    events = []
    for c in corners:
        cid = c.get("stable_corner_id")
        lap = laps_by.get(c.get("lap_number"))
        if cid is None or lap is None or c.get("bracket_start_m") is None:
            continue
        sl = _canonical_window_slice(t, s_m, lap["start_time"], lap["end_time"],
                                     c["bracket_start_m"], c["bracket_end_m"])
        if sl.stop <= sl.start:
            continue
        out["n_instances_by_corner"][str(cid)] = out["n_instances_by_corner"].get(str(cid), 0) + 1
        on_lap = (t >= lap["start_time"]) & (t <= lap["end_time"])
        s_kerb = s_m[on_lap & kerb_any]
        s_kerb = s_kerb[np.isfinite(s_kerb)]
        for ax in AXLES:
            ratio = cs[f"CS_ratio_{ax}"]
            seg = ratio[sl]
            for a, b in _runs(np.isfinite(seg) & (seg < PURE_LATERAL_FLOOR)):
                i0, i1 = sl.start + a, sl.start + b
                rng = np.arange(i0, i1 + 1)
                i_min = int(rng[np.argmin(ratio[rng])])
                ns, ra = cs.get(f"near_strike_{ax}"), cs.get(f"observer_reset_affected_{ax}")
                s_ev = s_m[rng]
                if s_kerb.size:
                    inside = (s_kerb >= np.nanmin(s_ev)) & (s_kerb <= np.nanmax(s_ev))
                    d_kerb = 0.0 if inside.any() else float(np.min(np.abs(s_kerb[:, None] - s_ev[None, :])))
                else:
                    d_kerb = None   # no kerb sample on this lap
                ev = {
                    "lap": int(c["lap_number"]), "corner": int(cid), "axle": AXLES[ax],
                    "phase": _phase_at(c["segments"], float(t[i_min])),
                    "phases_spanned": _phases_spanned(c["segments"], float(t[i0]), float(t[i1])),
                    "t_start": float(t[i0]), "t_end": float(t[i1]),
                    "duration_s": float(t[i1] - t[i0] + 1.0 / sr), "n_samples": int(rng.size),
                    "s_start_m": float(s_m[i0]), "s_end_m": float(s_m[i1]), "s_min_m": float(s_m[i_min]),
                    "cs_min": float(ratio[i_min]),
                    "kerb_distance_m": d_kerb,
                    "kerb_affected": bool((ns is not None and ns[rng].any())
                                          or (kerb is not None and kerb[rng].any())),
                    "observer_reset_affected": bool(ra is not None and ra[rng].any()),
                    "floor": PURE_LATERAL_FLOOR,
                    "fx_util_max": None, "braking_share": None, "braking": None,
                    "fz_window_range_median": None, "fz_window_range_max": None,
                }
                if axle_loads is not None:
                    fz = axle_loads[f"fz_{ax}_N"]
                    if axle_fx is not None:
                        fx = axle_fx[f"fx_{ax}_N"][rng]
                        ev["fx_util_max"] = float(np.max(np.abs(fx) / fz[rng]))
                        ev["braking_share"] = float(np.mean(fx < 0.0))
                        # flag: braking (Fx < 0) on every event sample
                        ev["braking"] = bool(np.all(fx < 0.0))
                    # Fz range over mean inside each event sample's regression
                    # window [start, i) -- how much the load moved while the
                    # slope was being taken
                    rngs = []
                    for i in (rng if slip is not None else ()):
                        w0 = reconstruct_cs_window_start(slip[f"alpha_{ax}_filt"], int(i), min_window,
                                                         se["cs_min_slip_angle_span_rad"], s_m, se["cs_max_window_m"])
                        w = fz[w0:i]
                        if w.size:
                            rngs.append(float((w.max() - w.min()) / w.mean()))
                    if rngs:
                        ev["fz_window_range_median"] = float(np.median(rngs))
                        ev["fz_window_range_max"] = float(np.max(rngs))
                if slip is not None:
                    # start of the regression window behind the first event
                    # sample, as for the curve-floor events
                    w0 = reconstruct_cs_window_start(slip[f"alpha_{ax}_filt"], i0, min_window,
                                                     se["cs_min_slip_angle_span_rad"], s_m, se["cs_max_window_m"])
                    ev["window_start_t"] = float(t[w0])
                events.append(ev)
    events.sort(key=lambda e: (e["lap"], e["t_start"], e["axle"]))
    out["events"] = events
    return out


def label_recurrence(events, radius_m):
    """Recurring = an event of the same axle at the same corner on another
    analysis lap with its minimum within radius_m lap distance (the audit's
    position-locked rule); else single occurrence. In place."""
    for e in events:
        laps = {o["lap"] for o in events
                if o["axle"] == e["axle"] and o["corner"] == e["corner"]
                and abs(o["s_min_m"] - e["s_min_m"]) <= radius_m}
        e["recurring_laps"] = sorted(laps)
        e["recurrence"] = "recurring" if len(laps) >= 2 else "single"


def aggregate_by_corner_phase(events):
    """The thesis consumer's resolution (the driver-feedback form's): one
    row per corner, phase and axle -- events, laps, kerb-affected count."""
    rows = {}
    for e in events:
        key = (e["corner"], e["phase"], e["axle"])
        r = rows.setdefault(key, {"corner": e["corner"], "phase": e["phase"], "axle": e["axle"],
                                  "n_events": 0, "laps": set(), "n_kerb_affected": 0})
        r["n_events"] += 1
        r["laps"].add(e["lap"])
        r["n_kerb_affected"] += int(e["kerb_affected"])
    out = []
    for key in sorted(rows, key=lambda k: (k[0], PHASE_ORDER.index(k[1]), k[2])):
        r = rows[key]
        out.append({**r, "laps": sorted(r["laps"])})
    return out


def _phase_word(phase):
    from modules.decision_frame import PHASE_WORD_LONG
    return PHASE_WORD_LONG.get(phase, phase)


def _what(line):
    return ("force loss not explainable by pure lateral saturation" if line == LINE_PURE
            else "force loss beyond the tyre characteristic")


def layer_title(events):
    """Session-list heading of the event layer, by its line."""
    line = (events or {}).get("line")
    return f"{'Floor events' if line == LINE_PURE else 'Limit events'} ({_what(line)})"


def legend_text(events):
    """Trace-dialog legend for the event strip, by the layer's line."""
    if (events or {}).get("line") == LINE_PURE:
        return ("Thick strip along the bottom of a cornering-stiffness-ratio panel: floor event -- the ratio "
                f"fell below {PURE_LATERAL_FLOOR}, lower than any pure-slip tyre curve can go (force loss not "
                "explainable by pure lateral saturation); display only, not used by any verdict.")
    return ("Thick strip along the bottom of a cornering-stiffness-ratio panel: limit event -- the ratio "
            "fell below what this session's fitted tyre curve can produce anywhere (force loss beyond the "
            "tyre characteristic); display only, not used by any verdict.")


def describe_event(e, n_laps_at_corner=None):
    """One card line, plain words."""
    rec = ("single occurrence" if e["recurrence"] == "single" else
           f"recurring here on {len(e['recurring_laps'])}"
           + (f" of {n_laps_at_corner}" if n_laps_at_corner else "") + " laps")
    line = LINE_PURE if e.get("floor") == PURE_LATERAL_FLOOR else LINE_CURVE
    return (f"Lap {e['lap']}, C{e['corner']} {_phase_word(e['phase'])} ({e['axle']} axle): {_what(line)}, "
            f"{e['duration_s']:.1f} s, "
            f"{'kerb-affected' if e['kerb_affected'] else 'not kerb-affected'}, {rec}")


def describe_group(r):
    """One session-list line per corner, phase and axle."""
    laps = ", ".join(str(l) for l in r["laps"])
    kerb = f", {r['n_kerb_affected']} kerb-affected" if r["n_kerb_affected"] else ""
    return (f"C{r['corner']} {_phase_word(r['phase'])} ({r['axle']} axle): {r['n_events']} "
            f"event{'s' if r['n_events'] != 1 else ''} across lap{'s' if len(r['laps']) != 1 else ''} {laps}{kerb}")
