# Labelled development set export (work order DEV-1, author EXECUTE
# 2026-10-24). Read-only: the DB is opened read-only, nothing is written to
# the DB, the sidecars or config. For every outing of the weekends
# "Labeled PRC" / "Labeled NBR" the chain is taken exactly as the app takes
# it (ui/views/outing_form.py): the DB cache and the sidecar are used when
# their identity matches the current config and the outing's stored setup
# sheet; otherwise the chain runs as StabilityAnalysisThread runs it, and
# that result is exported but not persisted.
# Output: diagnostics/results_labeled_dev_20261024/ (or the folder name given
# as the first argument) -- one JSON per outing, index.json, setup_deltas.txt.
# Run: python -m diagnostics.export_labeled_dev [folder name]

import datetime
import json
import math
import os
import sqlite3
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)   # config and DB paths in modules/ are relative to the repo
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

OUT = os.path.join(ROOT, "diagnostics", sys.argv[1] if len(sys.argv) > 1 else "results_labeled_dev_20261024")
DB = os.path.join(ROOT, "data", "setuptool.db")
WEEKENDS = ("Labeled PRC", "Labeled NBR")
# TC position pair, three domains (config/channels.json ecu_B_tc_act note,
# 4g ruling); not whitelisted, so read straight from the raw export
TC_POSITION_CHANNELS = ("ecu_TC_int_pos", "ecu_TC_tim_pos", "log_tc_int_pos", "log_tc_tim_pos",
                        "stw_rt03_tc_int", "stw_rt01_tc_tim")

UNITS = {
    "cs_ratio_f / cs_ratio_r": "cornering stiffness ratio, dimensionless (local dFy/dalpha over the "
                               "session reference slope; < 0 = past the peak of the Fy / Fz curve)",
    "stability_observed_Nm_per_deg": "dMz/dbeta, N m / deg (negative = open-loop unstable yaw)",
    "apex_speed": "km/h (ecu_speed unit)", "apex_lateral_g": "g",
    "lap_time / lap_time_precise / start_time / end_time / t_*": "s (logger time)",
    "s_*_m / bracket_*_m / distance": "m", "fz_*_N": "N", "fy_*_norm_N": "Fy / Fz, dimensionless",
    "stiffness_norm_per_rad (K)": "1 / rad, K = B C mu (tyre_fit_auto.plausibility_gate_l)",
    "mu": "dimensionless (demonstrated utilisation)", "health_score": "fraction of NIS windows in band",
    "shares": "fraction 0..1", "NaN": "written as null",
}


def jsafe(o):
    # strict JSON: NaN / inf -> null, numpy scalars and arrays -> Python
    if isinstance(o, dict):
        return {str(k): jsafe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple, set)):
        return [jsafe(v) for v in o]
    if isinstance(o, np.ndarray):
        return jsafe(o.tolist())
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, (float, np.floating)):
        return float(o) if math.isfinite(float(o)) else None
    if isinstance(o, (datetime.date, datetime.datetime)):
        return o.isoformat()
    return o


def db_outings():
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    rows = con.execute(
        "SELECT o.*, w.track AS weekend, w.series AS series, w.date AS weekend_date, d.name AS driver_name, "
        "d.driving_level AS driver_level FROM outings o JOIN race_weekends w ON o.race_weekend_id = w.id "
        "LEFT JOIN drivers d ON o.driver_id = d.id WHERE w.track IN (?, ?) ORDER BY w.track, o.number",
        WEEKENDS).fetchall()
    con.close()
    return [dict(r) for r in rows]


def _loads(s):
    try:
        return json.loads(s) if s else None
    except (json.JSONDecodeError, TypeError):
        return None


def raw_pass(path):
    """One streaming pass over the raw export: every channel name with its
    raw header and sample count, plus the data of TC_POSITION_CHANNELS and
    nothing else. Handles both {ChannelBlock} layouts like csv_parser."""
    from modules.csv_parser import _split_name_unit
    names, data = {}, {c: ([], []) for c in TC_POSITION_CHANNELS}
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        mode, cols, cur = None, {}, None
        for line in f:
            s = line.strip()
            if s.startswith("{"):
                mode = "header" if s == "{ChannelBlock}" else None
                continue
            if mode == "header":
                parts = s.split("\t")
                if len(parts) == 2:
                    name, unit = _split_name_unit(parts[1])
                    names[name] = {"raw_header": parts[1], "n_samples": 0, "unit_raw": unit}
                    mode, cur = "narrow", name
                elif len(parts) > 2 and parts[0].strip() == "Time":
                    cols = {}
                    for i, raw in enumerate(parts[1:], start=1):
                        name, unit = _split_name_unit(raw)
                        names[name] = {"raw_header": raw, "n_samples": 0, "unit_raw": unit}
                        if name in data:
                            cols[i] = name
                    mode = "wide"
                continue
            if not s:
                continue
            if mode == "narrow":
                names[cur]["n_samples"] += 1
                if cur in data:
                    p = s.split("\t")
                    try:
                        data[cur][0].append(float(p[0].replace(",", ".")))
                        data[cur][1].append(float(p[1].replace(",", ".")))
                    except (ValueError, IndexError):
                        pass
            elif mode == "wide" and cols:
                p = s.split("\t")
                for i, name in cols.items():
                    if i < len(p) and p[i].strip():
                        try:
                            data[name][0].append(float(p[0].replace(",", ".")))
                            data[name][1].append(float(p[i].replace(",", ".")))
                        except ValueError:
                            pass
    return names, {k: (np.asarray(t), np.asarray(v)) for k, (t, v) in data.items()}


def _mode(values):
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    if not v.size:
        return None
    vals, counts = np.unique(np.round(v).astype(int), return_counts=True)
    return {"mode": int(vals[np.argmax(counts)]),
            "distinct": {str(int(a)): int(b) for a, b in zip(vals, counts)}}


def _on_laps(t, laps, lap_numbers):
    m = np.zeros(t.shape, bool)
    for l in laps:
        if l["lap_number"] in lap_numbers:
            m |= (t >= l["start_time"]) & (t <= l["end_time"])
    return m


def cockpit_values(channels, laps, analysis_laps, tc_raw):
    from modules.setup_state import logged_setup_values
    out = {"logged_setup_values (modules.setup_state, as the engine reads them)":
           logged_setup_values(channels, laps, set(analysis_laps))}
    abs_ch = channels.get("log_abs_pos")
    if abs_ch is not None and abs_ch.get("data") is not None and abs_ch.get("quality") not in ("missing", "failed"):
        t = np.asarray(abs_ch["time"])
        out["log_abs_pos"] = {"session": _mode(abs_ch["data"]),
                              "analysis_laps": _mode(np.asarray(abs_ch["data"])[_on_laps(t, laps, analysis_laps)])}
    else:
        out["log_abs_pos"] = {"status": (abs_ch or {}).get("quality", "absent")}
    tc = {}
    for name, (t, v) in tc_raw.items():
        tc[name] = ({"session": _mode(v), "analysis_laps": _mode(v[_on_laps(t, laps, analysis_laps)])}
                    if v.size else {"status": "absent from the raw export"})
    out["tc_position_modes (raw export, LAT / LON mapping undocumented)"] = tc
    act = channels.get("ecu_B_tc_act")
    if act is not None and act.get("data") is not None and act.get("quality") not in ("missing", "failed"):
        t, d = np.asarray(act["time"]), np.asarray(act["data"], float) > 0.5
        m = _on_laps(t, laps, analysis_laps)
        out["ecu_B_tc_act"] = {"definition": "share of the channel's own samples with flag > 0.5",
                               "share_session": float(d.mean()) if d.size else None,
                               "share_analysis_laps": float(d[m].mean()) if m.any() else None,
                               "n_samples_analysis_laps": int(m.sum())}
    else:
        out["ecu_B_tc_act"] = {"status": (act or {}).get("quality", "absent")}
    return out


def db_cache_check(outing, loaded_path, parsed, ident):
    """The guards of OutingForm._try_render_cached_analysis, in order;
    (payload, None) on a hit, else (None, reason)."""
    from modules.data_paths import path_key
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
    cached = _loads(outing["analysis_data"])
    if cached is None:
        return None, "no analysis_data in the DB"
    checks = [
        ("schema_version", cached.get("schema_version") == ANALYSIS_SCHEMA_VERSION),
        ("csv_path", path_key(cached.get("csv_path")) == path_key(loaded_path)),
        ("accuracy_cap", cached.get("accuracy_cap") == ident["cap"]),
        ("resolved_vehicle_snapshot", cached.get("resolved_vehicle_snapshot") == ident["resolved"]["values"]),
        ("estimator_settings", cached.get("estimator_settings") == ident["resolved"]["estimator_settings"]),
        ("sideslip_source", cached.get("sideslip_source") == ident["sideslip_source"]),
        ("grid_rate_hz", cached.get("grid_rate_hz") == ident["grid_rate_hz"]),
        ("summaries", bool(cached.get("summaries"))),
        ("lap_filter", sorted(cached.get("lap_filter") or []) == sorted(ident["lap_filter"] or [])),
    ]
    for name, ok in checks:
        if not ok:
            return None, f"DB cache miss at {name}" + (
                f" (stored {cached.get('schema_version')}, current {ANALYSIS_SCHEMA_VERSION})"
                if name == "schema_version" else "")
    return cached, None


def run_chain(parsed, ident, loaded_path):
    """StabilityAnalysisThread.run, full-pipeline branch. The fit-L
    plausibility gate result is captured by wrapping the module function
    (resolve_sideslip_beta strips it from the manifest)."""
    from modules import tyre_fit_auto
    from modules.stability_analysis import (
        load_parameters, load_car_data, prepare_vehicle_state, estimate_slip_angles, estimate_lateral_forces,
        estimate_cornering_stiffness, estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners)
    from modules.accuracy_resolution import apply_resolved_vehicle
    from modules.corner_analysis import reclassify_speed_classes
    from modules.csv_parser import load_channels_config
    from modules.limit_events import detect_events
    p = apply_resolved_vehicle(load_parameters(), ident["resolved"])
    state = prepare_vehicle_state(parsed["channels"], p)
    if state is None:
        return None, "Required channels missing or failed (prepare_vehicle_state returned None)", None
    reclassify_speed_classes(parsed.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    captured = {}
    orig = tyre_fit_auto.plausibility_gate_l

    def wrapped(alpha, fit_mask, axle_fit, cfg):
        g = orig(alpha, fit_mask, axle_fit, cfg)
        captured["front" if "front" not in captured else "rear"] = g
        return g
    tyre_fit_auto.plausibility_gate_l = wrapped
    try:
        beta, man, gate, fb_used, fb_reason, cs_in = tyre_fit_auto.resolve_sideslip_beta_and_cs_inputs(
            state, p, parsed, ident["sideslip_source"], csv_path=loaded_path)
    finally:
        tyre_fit_auto.plausibility_gate_l = orig
    laps = parsed.get("laps", [])
    slip = estimate_slip_angles(state, beta, p)
    forces = estimate_lateral_forces(state, p)
    cs = estimate_cornering_stiffness(slip, forces, state, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(state, beta, p, laps)
    fz = estimate_vertical_loads(state, forces, p, channels=parsed["channels"], car_data=load_car_data())
    corners = parsed.get("corners", [])
    events = detect_events(cs, state, corners, laps, man, p, slip=slip, gate_verdict=gate,
                           axle_loads=cs_in["cs_kwargs"].get("axle_loads"), axle_fx=cs_in["axle_fx"])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=ident["lap_filter"])
    res = {"summaries": summaries, "fit_manifest": man, "gate_verdict": gate, "fallback_used": fb_used,
           "fallback_reason": fb_reason, "events": events, "corners": corners, "state": state}
    return res, None, captured


def d4l_block(man, captured):
    from modules.stability_analysis import load_parameters
    from modules.tyre_fit_auto import L_K_BAND, L_K_EDGE, L_MU_CEILING, MU_EDGE_CONFIG_KEY
    mu_edge = L_MU_CEILING * (1.0 + load_parameters()["tyre_fit_auto"][MU_EDGE_CONFIG_KEY])
    out = {"k_band_per_rad": list(L_K_BAND), "k_edge_per_rad": list(L_K_EDGE), "mu_ceiling": L_MU_CEILING,
           "mu_edge": mu_edge,
           "note": "K in (30, 35] = edge zone, status marginal by design (ruling 2026-10-20); K > 35 fails g2"}
    if captured:
        out["source"] = "plausibility_gate_l result captured in this run"
        out["axles"] = captured
        return out
    out["source"] = ("derived from the persisted manifest: K = B C mu with the gate's own formula; the "
                     "g1 alpha span is not persisted (stripped by resolve_sideslip_beta)")
    axles = (man or {}).get("axles") or {}
    zones = (man or {}).get("k_band_zone") or {}
    mu_zones = (man or {}).get("mu_ceiling_zone") or {}
    out["axles"] = {}
    for ax in ("front", "rear"):
        a = axles.get(ax)
        if not a:
            continue
        K = a["B"] * a["C"] * a["mu"]
        out["axles"][ax] = {"stiffness_norm_per_rad": K, "k_zone": zones.get(ax), "mu": a["mu"],
                            "mu_source": a.get("mu_source"), "B": a["B"], "C": a["C"],
                            "marginal_by_design": bool(L_K_BAND[1] < K <= L_K_EDGE[1]),
                            "mu_zone": mu_zones.get(ax), "mu_above_ceiling": bool(a["mu"] > L_MU_CEILING)}
    return out


def per_cell(summaries, events):
    from modules.classifier import classify_corner, cs_value_state, phase_symptoms, verdict_cs_stat
    from modules.corner_aggregation import PHASE_KEYS
    ev = (events or {}).get("events") or []
    rows = []
    for s in summaries:
        sev, short, long_, state = classify_corner(s)
        cid, lap = s.get("stable_corner_id"), s["lap_number"]
        ps = phase_symptoms(s)
        phases = {}
        for ph in PHASE_KEYS:
            p = s["phases"].get(ph)
            if p is None:
                phases[ph] = None
                continue
            csf, csr = verdict_cs_stat(s, ph, "f"), verdict_cs_stat(s, ph, "r")
            phases[ph] = {
                "cs_ratio_f (verdict statistic)": csf, "cs_ratio_r (verdict statistic)": csr,
                "cs_state_f": cs_value_state(csf["median"], "f"), "cs_state_r": cs_value_state(csr["median"], "r"),
                "stability_observed_Nm_per_deg": p["stability_observed_Nm_per_deg"],
                "n_samples": p.get("n_samples"), "kerb_fraction": p.get("kerb_fraction"),
                "kerb_flag": bool((p.get("kerb_fraction") or 0) > 0
                                  or csf.get("n_kerb_excluded", 0) or csr.get("n_kerb_excluded", 0)),
                "symptoms": [x for x in ps["symptoms"] if x["phase"] == ph],
                "info": [x for x in ps.get("info", []) if x.get("phase") == ph],
                "floor_events": [e for e in ev if e["lap"] == lap and e["corner"] == cid
                                 and ph in (e.get("phases_spanned") or [e["phase"]])],
            }
        rows.append({"lap_number": lap, "stable_corner_id": cid, "corner_number": s.get("corner_number"),
                     "speed_class": s.get("speed_class"), "apex_speed": s.get("apex_speed"),
                     "apex_lateral_g": s.get("apex_lateral_g"), "method": s.get("method"),
                     "warnings": s.get("warnings"),
                     "verdict": {"severity": sev, "short": short, "long": long_, "state": state},
                     "phases": phases})
    return rows


def per_corner(summaries):
    from modules.classifier import classify_corner, phase_symptoms
    from modules.corner_aggregation import aggregate_by_corner
    from modules.setup_engine import counted_symptoms
    from modules.stability_analysis import load_parameters
    counted = counted_symptoms(summaries, load_parameters()["setup_engine"])
    out = []
    for cid, agg in sorted(aggregate_by_corner(summaries).items(), key=lambda kv: kv[0]):
        sev, short, long_, state = classify_corner(agg)
        c = counted.get(cid, {})
        out.append({"stable_corner_id": cid, "canonical_speed_class": agg.get("speed_class"),
                    "n_laps": c.get("n_laps"),
                    "aggregate_verdict": {"severity": sev, "short": short, "long": long_, "state": state},
                    "aggregate_symptoms": phase_symptoms(agg)["symptoms"],
                    "counted_symptoms (D5 majority rule)": c.get("symptoms", [])})
    return out


def engine_text(layers):
    from modules.setup_engine import corner_summary, engineer_lines, session_summary
    out = {}
    for name in ("data_layer", "engineer_layer"):
        lay = layers[name]
        out[name] = {"session_summary": session_summary(lay["session_recommendation"],
                                                        engineer=name == "engineer_layer"),
                     "corner_summaries": [corner_summary(c, engineer=name == "engineer_layer")
                                          for c in lay["corners"]]}
    out["engineer_lines"] = engineer_lines(layers["data_layer"], layers["engineer_layer"])
    return out


def channel_status(channels):
    return {k: v.get("quality") for k, v in sorted(channels.items())
            if v is None or v.get("quality") in ("missing", "failed") or v.get("time") is None}


def failure_block(parsed, raw_names, chain_error):
    from diagnostics.generate_channel_requirements import CHANNEL_NOTES
    required = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_x"]  # prepare_vehicle_state
    bad = channel_status(parsed["channels"])
    return {
        "failed_step": "prepare_vehicle_state (Module 1 vehicle state)" if chain_error else None,
        "app_message": chain_error,
        "required_channels_absent": [c for c in required if c in bad],
        "blocked_steps": ["vehicle state", "fit L tyre curve + D4-L gate", "EKF observer + NIS gate",
                          "slip angles / lateral forces", "CS estimate", "yaw-moment stability",
                          "vertical loads", "corner summaries + verdicts", "floor events",
                          "setup engine (data + engineer layer)"] if chain_error else [],
        "steps_completed": {"parse": True, "laps": len(parsed.get("laps", [])),
                            "corners_detected": len(parsed.get("corners", []))},
        "whitelisted_channels_missing_or_failed": {
            c: {"quality": q, "role": (CHANNEL_NOTES.get(c) or (None, "no note in "
                                       "generate_channel_requirements.CHANNEL_NOTES"))[0],
                "consumer_note": (CHANNEL_NOTES.get(c) or (None, None))[1]} for c, q in bad.items()},
        "raw_export_channel_count": len(raw_names),
        "raw_export_names_containing_yaw": sorted(n for n in raw_names if "yaw" in n.lower()),
    }


def export_one(o):
    from modules.accuracy_resolution import resolve_accuracy
    from modules.csv_parser import parse_csv, load_channels_config
    from modules.corner_analysis import reclassify_speed_classes
    from modules.data_paths import path_key, resolve_saved_path, split_root
    from modules.pipeline_sidecar import build_identity, load_sidecar
    from modules.setup_engine import run as engine_run
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION, load_parameters, _resolve_grid_rate

    setup_data, feedback = _loads(o["setup_data"]), _loads(o["feedback_data"])
    root_rel = split_root(o["csv_path"])
    rec = {"_about": "DEV-1 export, one outing; key names as in code; units in _units",
           "_units": UNITS, "_generated_at": datetime.datetime.now().isoformat(),
           "schema_version": ANALYSIS_SCHEMA_VERSION,
           "outing": {"outing_id": o["id"], "weekend": o["weekend"], "series": o["series"],
                      "weekend_date": o["weekend_date"], "name": o["name"], "number": o["number"],
                      "date_time": o["date_time"], "session_type": o["session_type"],
                      "driver": o["driver_name"], "driver_level (stored, unused by scoring)": o["driver_level"],
                      "file_root_relative": root_rel[1].replace("\\", "/") if root_rel else None,
                      "file_root": root_rel[0] if root_rel else None,
                      "environment": {k: o[k] for k in ("air_temp", "track_temp", "track_condition", "fuel_level",
                                                        "tyre_age", "tyre_type", "tyre_name")}},
           "setup_sheet (as stored)": setup_data, "setdown_sheet (as stored)": _loads(o["setdown_data"]),
           "comment (verbatim)": o["comments"], "driver_feedback (as stored)": feedback}
    path = resolve_saved_path(o["csv_path"])
    if path is None:
        rec["status"] = "file not found"
        return rec
    parsed = parse_csv(path)
    reclassify_speed_classes(parsed.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    laps = parsed["laps"]
    rec["outing"]["laps"] = [{k: l.get(k) for k in ("lap_number", "lap_time", "lap_time_precise", "start_time",
                                                     "end_time", "is_valid_for_analysis", "is_outlap", "is_inlap",
                                                     "is_fastest")} for l in laps]

    params = load_parameters()
    valid = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis"))
    ident = {"cap": None, "resolved": resolve_accuracy(params, setup_data, None),
             "sideslip_source": params["stability_estimation"].get("sideslip_source", "kinematic"),
             "grid_rate_hz": _resolve_grid_rate(parsed["channels"], params)[0],
             "lap_filter": valid or sorted(l["lap_number"] for l in laps)}
    rec["outing"]["analysis_laps (lap_filter)"] = ident["lap_filter"]

    cached, db_miss = db_cache_check(o, path, parsed, ident)
    sidecar, sc_miss = None, None
    if cached is not None:
        sidecar, sc_miss = load_sidecar(o["id"], build_identity(
            schema_version=ANALYSIS_SCHEMA_VERSION, csv_path=os.path.normcase(os.path.normpath(path)),
            accuracy_cap=None, resolved_vehicle_snapshot=ident["resolved"]["values"],
            sideslip_source=ident["sideslip_source"], grid_rate_hz=ident["grid_rate_hz"],
            lap_filter=ident["lap_filter"], estimator_settings=ident["resolved"]["estimator_settings"]))
    captured, chain_error = None, None
    if cached is not None and sidecar is not None:
        res = {"summaries": cached["summaries"], "fit_manifest": cached["fit_manifest"],
               "gate_verdict": cached["gate_verdict"], "fallback_used": cached["fallback_used"],
               "fallback_reason": cached["fallback_reason"], "events": cached["events"],
               "corners": sidecar["corners"], "state": sidecar["state"]}
        source = "app cache (DB payload + sidecar, identity valid)"
    else:
        res, chain_error, captured = run_chain(parsed, ident, path)
        source = ("full chain run in this export (not persisted); cache: "
                  + (db_miss or f"sidecar miss at {sc_miss}"))
    rec["result_source"] = source

    tc_names, tc_raw = raw_pass(path)
    rec["cockpit_logged"] = cockpit_values(parsed["channels"], laps, ident["lap_filter"], tc_raw)
    bad = channel_status(parsed["channels"])

    if res is None:
        rec["status"] = "chain failed"
        rec["fit_manifest"] = {"status": None, "missing_or_failed_channels": bad}
        rec["failure_diagnosis"] = failure_block(parsed, tc_names, chain_error)
        return rec

    man, gate = res["fit_manifest"] or {}, res["gate_verdict"] or {}
    rec["fit_manifest"] = {
        "status": man.get("status"), "marginal_reason": man.get("marginal_reason"),
        "degenerate_reason": man.get("degenerate_reason"),
        "fallback_used": res["fallback_used"], "fallback_reason": res["fallback_reason"],
        "gate_d4l": d4l_block(man, captured),
        "gate_nis": gate,
        "sign_check": man.get("sign_check"), "nis": man.get("nis"),
        "speed_window": man.get("speed_window"), "laps_used": man.get("laps_used"),
        "axles": man.get("axles"),
        "axle_load_lowest_level": (man.get("axle_load") or {}).get("lowest_level"),
        "missing_or_failed_channels": bad,
        "full_manifest": man,
    }
    rec["per_lap_corner_phase"] = per_cell(res["summaries"], res["events"])
    rec["per_corner"] = per_corner(res["summaries"])
    rec["floor_events"] = {k: v for k, v in (res["events"] or {}).items() if k != "events"}
    rec["floor_events"]["n_events"] = len((res["events"] or {}).get("events") or [])

    computed = engine_run(res["summaries"], setup_data=setup_data, feedback_data=feedback,
                          corners=res["corners"], state=res["state"], channels=parsed["channels"], laps=laps)
    stored = (cached or {}).get("setup_engine") if sidecar is not None else None
    layers = stored or computed
    rec["engine"] = {
        "source": ("stored with the cached analysis (what the app renders)" if stored
                   else "computed in this export from the stored setup sheet and feedback"),
        "recomputed_equals_stored": (jsafe(computed) == jsafe(stored)) if stored else None,
        "data_layer": layers["data_layer"], "engineer_layer": layers["engineer_layer"],
        "cell_shares (data_layer.session.cells)": layers["data_layer"]["session"]["cells"],
        "display_text": engine_text(layers),
    }
    rec["status"] = man.get("status") or "kinematic"
    if rec["status"] != "ok" or res["fallback_used"]:
        rec["failure_diagnosis"] = {"fit_status": man.get("status"),
                                    "marginal_reason": man.get("marginal_reason"),
                                    "degenerate_reason": man.get("degenerate_reason"),
                                    "fallback_reason": res["fallback_reason"],
                                    "gate_d4l": rec["fit_manifest"]["gate_d4l"], "gate_nis": gate,
                                    "missing_or_failed_channels": bad}
    return rec


def summary_line(r):
    o = r["outing"]
    head = f"{o['weekend']} #{o['number']} {o['name']} ({o['driver']}, outing {o['outing_id']})"
    if r.get("status") in ("file not found", "chain failed"):
        fd = r.get("failure_diagnosis", {})
        return f"{head}: {r['status']}; required absent {fd.get('required_channels_absent')}"
    fm = r["fit_manifest"]
    d4 = fm["gate_d4l"]["axles"]
    k = ", ".join(f"{ax} K {v['stiffness_norm_per_rad']:.2f} {v['k_zone']} mu {v['mu']:.3f} {v.get('mu_zone')}"
                  for ax, v in d4.items())
    g = fm["gate_nis"] or {}
    laps = r["outing"]["laps"]
    sym = "; ".join(f"C{c['stable_corner_id']} " + ", ".join(
        f"{s['symptom']}@{s['phase']} {s['laps']}/{c['n_laps']}" for s in c["counted_symptoms (D5 majority rule)"])
        for c in r["per_corner"] if c["counted_symptoms (D5 majority rule)"]) or "none"
    sess = r["engine"]["display_text"]["data_layer"]["session_summary"][0]
    return (f"{head}: status {r['status']}; D4-L {k}; NIS {g.get('verdict')} health "
            f"{g.get('health_score')}; laps {len(r['outing']['analysis_laps (lap_filter)'])} analysis of "
            f"{len(laps)}; counted symptoms {sym}; session recommendation (data) {sess}")


def _flat(d, pre=""):
    out = {}
    for k, v in (d or {}).items():
        key = f"{pre}{k}"
        if isinstance(v, dict):
            out.update(_flat(v, key + "."))
        else:
            out[key] = v
    return out


def setup_deltas(outs):
    L = ["SETUP-SHEET DELTAS, consecutive outings per weekend (by outing number), old -> new", ""]
    for wk in WEEKENDS:
        seq = sorted((o for o in outs if o["weekend"] == wk), key=lambda o: o["number"])
        L.append(wk)
        for a, b in zip(seq, seq[1:]):
            fa, fb = _flat(_loads(a["setup_data"])), _flat(_loads(b["setup_data"]))
            diffs = [f"    {k}: {fa.get(k)!r} -> {fb.get(k)!r}" for k in sorted(set(fa) | set(fb))
                     if fa.get(k) != fb.get(k)]
            L.append(f"  #{a['number']} {a['name']} -> #{b['number']} {b['name']}: {len(diffs)} field(s) changed")
            L += diffs
        L.append("")
    return "\n".join(L)


def main():
    os.makedirs(OUT, exist_ok=True)
    outs = db_outings()
    index = {"_about": "DEV-1 labelled development set, index", "_generated_at": datetime.datetime.now().isoformat(),
             "weekends": list(WEEKENDS), "outings": []}
    for o in outs:
        print(f"[{o['weekend']} #{o['number']}] {o['name']} ...", flush=True)
        r = jsafe(export_one(o))
        fn = f"{o['weekend'].replace(' ', '_')}_{o['number']:02d}_outing{o['id']}.json"
        with open(os.path.join(OUT, fn), "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, ensure_ascii=True, allow_nan=False)
        line = summary_line(r)
        print("  " + line, flush=True)
        index["outings"].append({"file": fn, "outing_id": o["id"], "weekend": o["weekend"], "number": o["number"],
                                 "name": o["name"], "status": r.get("status"), "result_source": r.get("result_source"),
                                 "summary": line})
    with open(os.path.join(OUT, "index.json"), "w", encoding="utf-8") as f:
        json.dump(jsafe(index), f, indent=1, ensure_ascii=True, allow_nan=False)
    txt = setup_deltas(outs)
    with open(os.path.join(OUT, "setup_deltas.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
