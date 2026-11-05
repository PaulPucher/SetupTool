# The analysis chain without Qt: the sequence the app's analysis thread
# runs, from a parsed session to corner summaries and floor events, and the
# setup recommendation on top of the summaries. The UI calls these
# functions, tests/generate_golden pins them, and
#     python -m modules.pipeline <session file> [-o out.json]
# runs them on one exported session and writes a JSON summary.

import argparse
import json
import math
import os
import sys

from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
from modules.corner_analysis import reclassify_speed_classes
from modules.csv_parser import load_channels_config, parse_csv
from modules.limit_events import detect_events
from modules.stability_analysis import (
    REQUIRED_STATE_CHANNELS, estimate_cornering_stiffness, estimate_lateral_forces, estimate_slip_angles,
    estimate_vertical_loads, estimate_yaw_moment_stability, load_car_data, load_parameters,
    prepare_vehicle_state, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs


class MissingChannelsError(ValueError):
    pass


def analysis_laps(parsed_data):
    # every valid lap, or every lap if none is valid
    all_laps = sorted({l["lap_number"] for l in parsed_data.get("laps", [])})
    valid_laps = sorted(l["lap_number"] for l in parsed_data.get("laps", [])
                        if l.get("is_valid_for_analysis", False))
    return valid_laps if valid_laps else all_laps


def run_analysis(parsed_data, resolved_accuracy, csv_path=None):
    """The full chain on one parsed session, without the corner summaries
    (they depend on the lap filter and are re-run alone when only that
    changes). Raises MissingChannelsError when the vehicle state cannot be
    built."""
    params = load_parameters()
    # resolved (cap-clipped) vehicle values in a deep-copied params dict
    effective_params = apply_resolved_vehicle(params, resolved_accuracy)
    state = prepare_vehicle_state(parsed_data["channels"], effective_params)
    if state is None:
        raise MissingChannelsError("Required channels missing or failed")
    sideslip_source = effective_params["stability_estimation"].get(
        "sideslip_source", "kinematic"
    )
    # speed classes from the current Settings boundaries, before the
    # fit's sign check reads them
    reclassify_speed_classes(parsed_data.get("corners", []),
                             load_channels_config()["corner_speed_thresholds"])
    # cs_inputs: fit L's axle loads, reference band and reset mask
    # for the CS estimate; empty on the kinematic path
    (beta, fit_manifest, gate_verdict, fallback_used, fallback_reason,
     cs_inputs) = resolve_sideslip_beta_and_cs_inputs(
        state, effective_params, parsed_data, sideslip_source, csv_path=csv_path
    )
    slip = estimate_slip_angles(state, beta, effective_params)
    forces = estimate_lateral_forces(state, effective_params)
    cs = estimate_cornering_stiffness(slip, forces, state, effective_params, parsed_data.get("laps", []),
                                      **cs_inputs["cs_kwargs"])
    cs_axle_loads = cs_inputs["cs_kwargs"].get("axle_loads")
    stab = estimate_yaw_moment_stability(state, beta, effective_params, parsed_data.get("laps", []))
    # Fz / fy_norm: diagnostic only. channels/car_data only used when
    # vertical_load_source is "measured".
    fz = estimate_vertical_loads(state, forces, effective_params,
                                 channels=parsed_data["channels"], car_data=load_car_data())
    corners = parsed_data.get("corners", [])
    # event layer: display + record only, read after the verdict
    # chain; floor events (-0.53, measured attributes), none on
    # the kinematic path
    events = detect_events(cs, state, corners, parsed_data.get("laps", []),
                           fit_manifest, effective_params, slip=slip,
                           gate_verdict=gate_verdict,
                           axle_loads=cs_inputs["cs_kwargs"].get("axle_loads"),
                           axle_fx=cs_inputs["axle_fx"])
    return {
        "state": state, "cs": cs, "stab": stab, "fz": fz, "slip": slip, "forces": forces,
        "cs_axle_loads": cs_axle_loads, "corners": corners, "sideslip_source": sideslip_source,
        "fit_manifest": fit_manifest, "gate_verdict": gate_verdict,
        "fallback_used": fallback_used, "fallback_reason": fallback_reason, "events": events,
    }


def recommend(summaries, setup_data=None, feedback_data=None, corners=None, state=None, channels=None,
              laps=None):
    """Both setup-engine layers for one analysis: the data layer (session
    recommendation and per-corner options) and the engineer layer on top."""
    from modules.setup_engine import run
    return run(summaries, setup_data=setup_data, feedback_data=feedback_data,
               corners=corners, state=state, channels=channels, laps=laps)


# --- headless entry point ------------------------------------------------

def _finite(x):
    return x if isinstance(x, (int, float)) and math.isfinite(x) else None


def _session_summary(path, chain, summaries, layers):
    from modules.classifier import aggregate_by_corner, classify_corner
    man = chain["fit_manifest"] or {}
    gate = chain["gate_verdict"] or {}
    axles = {}
    for name, fit in (man.get("axles") or {}).items():
        b, c, mu = (_finite(fit.get(p)) for p in ("B", "C", "mu"))
        # normalised origin slope of the fitted curve, K = B C mu (fit L)
        k = b * c * mu if None not in (b, c, mu) else None
        axles[name] = {"K_per_rad": k, "mu": mu,
                       "k_zone": (man.get("k_band_zone") or {}).get(name),
                       "mu_zone": (man.get("mu_ceiling_zone") or {}).get(name)}
    verdicts = []
    for cid, agg in sorted(aggregate_by_corner(summaries).items(), key=lambda kv: (kv[0] is None, kv[0])):
        severity, short, _long, _state = classify_corner(agg)
        verdicts.append({"corner": cid, "speed_class": agg.get("speed_class"), "n_laps": agg.get("n_laps"),
                         "severity": severity, "verdict": short})
    data = layers["data_layer"]
    counted = [{"corner": c["corner"], "symptom": s["symptom"], "phase": s["phase"], "laps": s["laps"]}
               for c in data["corners"] for s in c["symptoms"]]
    vote = data.get("session_recommendation") or {}
    rec, tie = vote.get("recommended"), vote.get("equivalent_options") or []
    if rec:
        session_rec = {"levers": rec["levers"], "direction": rec["direction"], "net": rec["net"]}
    elif tie:
        # an exact tie of the rank key has no single winner; without a setup
        # sheet the headroom term is 0 for every option and cannot break it
        session_rec = {"equivalent_options": [{"levers": o["levers"], "direction": o["direction"]} for o in tie],
                       "net": tie[0]["net"]}
    else:
        session_rec = None
    return {
        "session_file": os.path.basename(path),
        "sideslip_source": chain["sideslip_source"],
        "fit_status": man.get("status"),
        "fallback_used": chain["fallback_used"],
        "fallback_reason": chain["fallback_reason"],
        "vertical_load_source": (man.get("axle_load") or {}).get("vertical_load_source_used"),
        "axles": axles,
        "health_score": _finite(gate.get("health_score")),
        "gate_verdict": gate.get("verdict"),
        "analysis_laps": sorted({s["lap_number"] for s in summaries}),
        "corner_verdicts": verdicts,
        "counted_symptoms": counted,
        "session_recommendation": session_rec,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m modules.pipeline",
                                 description="Analyse one exported session and write a JSON summary.")
    ap.add_argument("session_file")
    ap.add_argument("-o", "--output", help="JSON file to write (default: print to stdout)")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.session_file):
        print(f"error: no such file: {args.session_file}", file=sys.stderr)
        return 2
    try:
        data = parse_csv(args.session_file)
    except ValueError as e:
        print(f"error: the file could not be read as a session export: {e}", file=sys.stderr)
        return 2
    missing = [ch for ch in REQUIRED_STATE_CHANNELS
               if (data["channels"].get(ch) or {}).get("quality", "missing") in ("missing", "failed")]
    if missing:
        print("error: mandatory channel(s) missing or failed in this file: " + ", ".join(missing),
              file=sys.stderr)
        return 3
    # no setup sheet: vehicle values at their best available level from config
    params = load_parameters()
    resolved = resolve_accuracy(params, setup_data=None, cap=None)
    chain = run_analysis(data, resolved, csv_path=args.session_file)
    summaries = summarise_corners(chain["corners"], chain["cs"], chain["stab"], chain["state"],
                                  fz=chain["fz"], lap_filter=analysis_laps(data))
    layers = recommend(summaries, corners=chain["corners"], state=chain["state"],
                       channels=data.get("channels"), laps=data.get("laps"))
    out = _session_summary(args.session_file, chain, summaries, layers)
    text = json.dumps(out, indent=1, default=float)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
