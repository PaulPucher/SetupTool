# Floor events: in-corner runs where an axle's cornering stiffness ratio (CS)
# falls below PURE_LATERAL_FLOOR -- lower than any pure-slip tyre curve can
# go. A third output class beside the corner verdicts (tendencies) and the
# recommendations; display and record only, never a verdict or
# recommendation input (author ruling 2026-10-20, one concept, one name).
#
# Lineage: flagging handling moments in logged traces after Segers (secs.
# 7.4-7.6; sec. 5.4 for a discrete-event precedent); interpretation per
# Hoffman / Ono / Werner, limit-regime vocabulary per Milliken sec. 8.5. The
# event channel itself -- CS against a fixed floor -- is SetupTool's
# construction (Tier B segmentation on a Tier A-derived line). The earlier
# fitted-curve-floor layer of the per_session path was removed in the
# 2026-10-22 pruning (tag pre-prune-2026-10-22).

import numpy as np

from modules.stability_analysis import reconstruct_cs_window_start, resolve_cs_min_window_samples
from modules.tyre_fit_auto import _canonical_window_slice

# Pure-lateral floor of the CS ratio (author ruling 2026-10-19): the steepest
# post-peak slope of the Magic Formula over its initial slope, at C = 3.0
# (Goblirsch et al. 2025 Table I upper bound) with E in [-1, 1] (Pacejka
# 2006 ch. 4). Independent of B and D. Lower than any pure-slip curve can go,
# so a ratio below it is "not explainable by pure lateral saturation". A
# label, never a deletion or a verdict input. Method-defining, not a tunable.
PURE_LATERAL_FLOOR = -0.53

AXLES = {"f": "front", "r": "rear"}
LINE_PURE = "pure-lateral floor"
PHASE_ORDER = ("entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5")


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
    """Event layer of one session: the floor events of detect_floor_events,
    line PURE_LATERAL_FLOOR; axle_loads / axle_fx feed their attributes.
    Returns a JSON-safe dict {"status", "line", "floors", "events": [...],
    "n_instances_by_corner"}. Without a tyre fit (kinematic sideslip source)
    there is no event layer.

    Population: in-corner samples (canonical brackets) on analysis laps, the
    FULL CS arrays -- kerb-affected samples are kept and labelled, so nothing
    is hidden (the classifier's clean population is a separate concern).
    """
    cfg = params["limit_events"]
    out = {"status": "computed", "floors": None, "events": [], "n_instances_by_corner": {}}
    if not cfg.get("enabled", True):
        out["status"] = "disabled in config"
        return out
    if not fit_manifest:
        out["status"] = "no events: no tyre fit in this run (kinematic sideslip source)"
        return out
    fe = detect_floor_events(cs, state, corners, laps, params, slip, axle_loads=axle_loads, axle_fx=axle_fx)
    # the -0.53 line is model-independent, so a degenerate fit or a failed
    # gate does not withdraw it; the note says which beta the CS came from
    # and why
    if fit_manifest.get("status") == "degenerate":
        fe["beta_note"] = "kinematic sideslip (tyre fit degenerate)"
    elif gate_verdict is not None and gate_verdict.get("verdict") == "fail":
        fe["beta_note"] = "kinematic sideslip (NIS gate failed)"
    label_recurrence(fe["events"], cfg["recurrence_radius_m"])
    return fe


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
    from modules.corner_aggregation import PHASE_WORD_LONG
    return PHASE_WORD_LONG.get(phase, phase)


WHAT = "force loss not explainable by pure lateral saturation"


def layer_title(events):
    """Session-list heading of the event layer."""
    return f"Floor events ({WHAT})"


def legend_text(events):
    """Trace-dialog legend for the event strip."""
    return ("Thick strip along the bottom of a cornering-stiffness-ratio panel: floor event -- the ratio "
            f"fell below {PURE_LATERAL_FLOOR}, lower than any pure-slip tyre curve can go ({WHAT}); "
            "display only, not used by any verdict.")


def describe_event(e, n_laps_at_corner=None):
    """One card line, plain words."""
    rec = ("single occurrence" if e["recurrence"] == "single" else
           f"recurring here on {len(e['recurring_laps'])}"
           + (f" of {n_laps_at_corner}" if n_laps_at_corner else "") + " laps")
    return (f"Lap {e['lap']}, C{e['corner']} {_phase_word(e['phase'])} ({e['axle']} axle): {WHAT}, "
            f"{e['duration_s']:.1f} s, "
            f"{'kerb-affected' if e['kerb_affected'] else 'not kerb-affected'}, {rec}")


def describe_group(r):
    """One session-list line per corner, phase and axle."""
    laps = ", ".join(str(l) for l in r["laps"])
    kerb = f", {r['n_kerb_affected']} kerb-affected" if r["n_kerb_affected"] else ""
    return (f"C{r['corner']} {_phase_word(r['phase'])} ({r['axle']} axle): {r['n_events']} "
            f"event{'s' if r['n_events'] != 1 else ''} across lap{'s' if len(r['laps']) != 1 else ''} {laps}{kerb}")
