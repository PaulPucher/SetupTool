# Frame-Stage-2 work package, Phase 1: vehicle-speed forensics on
# GT3_PRC_MLA-v3.txt (primary) with Sample_Dubai.txt as a control. Read-
# only except for Part (c)'s own local, deepcopy'd channel substitution
# (never written to config or the raw file). No config/production change.
#
# ecu_speed is state["v_mps"]'s sole source (modules.stability_analysis.
# prepare_vehicle_state) and the pipeline's own primary time-grid anchor
# -- described in the work order as "a car-side computed math channel",
# a forensic target for possible RR-wheel-speed contamination (v3's own
# log_speed_rr carries a documented spurious 300+ kph spike/dropout fault,
# Fz-integration Phase 5).
#
# (a) cross-plot ecu_speed against the four log_speed_* wheels and every
#     real GPS/radar speed reference found in the raw file. The work
#     order's own stated premise ("census said gps_speed absent") is
#     checked directly, not trusted, per CLAUDE.md's channel-census rule
#     -- diagnostics/inspect_v3_speed_channel_survey.py (this same phase)
#     already found this premise FALSE: log_gps_speed[kph], two NMEA-
#     derived GPS speeds, and an independent RADAR ego-speed (MRR_
#     EgoVehSpeed[km/h]) all exist and are populated.
# (b) synthesize a reference speed from the plausibility-guarded left-side
#     + healthy wheels (excludes RR, the known-faulty corner), cross-
#     checked against short-span integrated ax.
# (c) re-run C13's CS chain (ekf_auto_pacejka, the production default)
#     with ecu_speed replaced by the synthetic reference, all else
#     identical (same corners/laps geometry, same config), and compare
#     the sign-change-rate/CS_ratio trace at the same worst-oscillating
#     window already identified for the unmodified pipeline.

import copy
import io
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLOT_DIR = "diagnostics/plots_v3"

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_slip_angles,
    estimate_lateral_forces, estimate_cornering_stiffness,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.longitudinal_forces import _guarded_wheel_speed_kmh
from diagnostics.inspect_step2_chair_plots import _valid_lap_instances, _canonical_window_slice

V3_FILE = "GT3_PRC_MLA-v3.txt"
DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
ENCODING = "latin-1"

RAW_SPEED_TARGETS = [
    "log_gps_speed[kph]",
    "NMEA RX VTG Speed Ground[kph]",
    "MRR_EgoVehSpeed[km/h]",
]


def _sign_change_rate(arr):
    finite = np.isfinite(arr)
    vals = arr[finite]
    if vals.size < 3:
        return float("nan"), int(vals.size)
    signs = np.sign(vals)
    changes = int(np.sum(signs[1:] != signs[:-1]))
    return changes / (vals.size - 1), int(vals.size)


def _read_raw_speed_blocks(raw_file):
    blocks = {name: ([], []) for name in RAW_SPEED_TARGETS}
    current = None
    with io.open(raw_file, encoding=ENCODING) as f:
        for line in f:
            if line.startswith("Time\t"):
                name = line.strip().split("\t", 1)[1]
                current = name if name in RAW_SPEED_TARGETS else None
                continue
            if current is None:
                continue
            parts = line.strip().split("\t")
            if len(parts) != 2:
                continue
            try:
                t = float(parts[0]); v = float(parts[1])
            except ValueError:
                continue
            blocks[current][0].append(t)
            blocks[current][1].append(v)
    return {name: (np.array(t), np.array(v)) for name, (t, v) in blocks.items()}


def part_a(state, channels, raw_file):
    print(f"\n=== Part (a): ecu_speed cross-plot, {raw_file} ===")
    t_ref = state["time"]
    v_kmh = state["v_mps"] * 3.6
    moving = state["moving_mask"]

    wheel_kmh = {}
    for corner in ("fl", "fr", "rl", "rr"):
        ch = channels.get(f"log_speed_{corner}")
        if ch is None or ch.get("time") is None:
            continue
        wheel_kmh[corner] = np.interp(t_ref, ch["time"], ch["data"])

    print("-- ecu_speed vs log_speed_* (deviation stats, moving samples only) --")
    for corner, wk in wheel_kmh.items():
        dev = v_kmh - wk
        m = moving
        print(f"  ecu_speed - log_speed_{corner}: mean={dev[m].mean():+.3f} std={dev[m].std():.3f} "
              f"max_abs={np.max(np.abs(dev[m])):.3f} kph  (p99_abs={np.percentile(np.abs(dev[m]), 99):.3f})")

    raw_blocks = _read_raw_speed_blocks(raw_file)
    print("\n-- ecu_speed vs GPS/radar references (real, populated channels found this phase) --")
    for name, (t_raw, v_raw) in raw_blocks.items():
        if len(v_raw) == 0:
            print(f"  {name}: ABSENT in this file")
            continue
        ref_on_ref = np.interp(t_ref, t_raw, v_raw)
        dev = v_kmh - ref_on_ref
        m = moving & (ref_on_ref > 1.0)  # exclude the reference's own standstill/startup zeros
        print(f"  ecu_speed - {name}: mean={dev[m].mean():+.3f} std={dev[m].std():.3f} "
              f"max_abs={np.max(np.abs(dev[m])):.3f} kph, ecu max={v_kmh.max():.1f} vs ref max={ref_on_ref.max():.1f}")

    # RR fault-window overlap: reuse the production guard directly (no
    # reimplementation) to find exactly which samples it flags as RR-
    # invalid, then check whether ecu_speed's own deviation from the
    # healthy median is elevated specifically there.
    rr_guarded, rr_source = _guarded_wheel_speed_kmh(channels, "rr", t_ref, moving,
                                                       state["sample_rate_hz"], load_parameters())
    rr_flagged = (rr_source == "abs_speed_fallback") if rr_source is not None else np.zeros_like(moving)
    healthy_median = np.median(np.array([wheel_kmh[c] for c in ("fl", "fr", "rl") if c in wheel_kmh]), axis=0)
    ecu_dev_from_healthy = np.abs(v_kmh - healthy_median)
    print(f"\n-- ecu_speed deviation from median(FL,FR,RL), inside vs outside RR's own flagged fault windows --")
    if rr_flagged.any():
        print(f"  RR flagged fraction (moving samples): {rr_flagged[moving].mean()*100:.2f}%")
        print(f"  ecu_speed |dev| median, RR-flagged samples:     {np.median(ecu_dev_from_healthy[moving & rr_flagged]):.3f} kph")
        print(f"  ecu_speed |dev| median, RR-NOT-flagged samples: {np.median(ecu_dev_from_healthy[moving & ~rr_flagged]):.3f} kph")
    else:
        print("  RR guard flagged nothing on this file (or source unavailable)")
    # Figure: ecu_speed vs the healthy wheel median and the GPS/radar
    # references, one representative lap window (lap 8, the fastest lap
    # per the pit-limiter-classification work -- clean racing conditions).
    os.makedirs(PLOT_DIR, exist_ok=True)
    lap8_mask = (t_ref >= 903.6) & (t_ref <= 1028.6)  # approx lap 8 span, trimmed to a plot-friendly window
    if raw_file == V3_FILE and lap8_mask.sum() > 10:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(t_ref[lap8_mask], v_kmh[lap8_mask], label="ecu_speed", color="black", lw=1.2)
        ax.plot(t_ref[lap8_mask], healthy_median[lap8_mask], label="median(FL,FR,RL)", color="tab:blue", lw=1.0, alpha=0.8)
        for name, (t_raw, v_raw) in raw_blocks.items():
            if len(v_raw) == 0:
                continue
            ref_on_ref = np.interp(t_ref, t_raw, v_raw)
            ax.plot(t_ref[lap8_mask], ref_on_ref[lap8_mask], label=name, lw=0.9, alpha=0.7)
        ax.set_xlabel("time [s]"); ax.set_ylabel("speed [kph]")
        ax.set_title("v3 lap 8: ecu_speed vs wheel-median and GPS/radar references")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(PLOT_DIR, "ecu_speed_vs_references_lap8.png"), dpi=130)
        plt.close(fig)
        print(f"\n  figure saved: {PLOT_DIR}/ecu_speed_vs_references_lap8.png")

    return wheel_kmh, healthy_median


def part_b(state, channels, params):
    print("\n=== Part (b): synthetic reference speed (median of guarded FL/FR/RL, RR excluded) ===")
    t_ref = state["time"]
    moving = state["moving_mask"]
    sr = state["sample_rate_hz"]
    guarded = {}
    for corner in ("fl", "fr", "rl"):
        kmh, source = _guarded_wheel_speed_kmh(channels, corner, t_ref, moving, sr, params)
        guarded[corner] = kmh
        n_fallback = int(np.sum(source == "abs_speed_fallback")) if source is not None else 0
        print(f"  {corner}: guard fallback fired on {n_fallback}/{len(t_ref)} samples "
              f"({n_fallback/len(t_ref)*100:.2f}%)")
    synth_kmh = np.nanmedian(np.array([guarded[c] for c in ("fl", "fr", "rl")]), axis=0)
    synth_mps = synth_kmh / 3.6

    ecu_mps = state["v_mps"]
    dev = ecu_mps - synth_mps
    print(f"  ecu_speed - synthetic: mean={dev[moving].mean()*3.6:+.3f} std={dev[moving].std()*3.6:.3f} kph "
          f"(moving samples, n={moving.sum()})")

    print("\n  -- cross-check vs integrated ax over short spans --")
    ax = state["ax_mps2"]
    t = state["time"]
    dt = float(np.median(np.diff(t)))
    span_s = 2.0
    span_n = max(1, int(round(span_s / dt)))
    n_spans = len(t) // span_n
    residuals = []
    for i in range(n_spans):
        sl = slice(i * span_n, (i + 1) * span_n)
        if not moving[sl].all():
            continue
        dv_ax = np.trapz(ax[sl], t[sl])
        dv_synth = synth_mps[sl][-1] - synth_mps[sl][0]
        if np.isfinite(dv_ax) and np.isfinite(dv_synth):
            residuals.append(dv_ax - dv_synth)
    residuals = np.array(residuals)
    if residuals.size:
        print(f"  {span_s}s spans, n={residuals.size}: residual (integrated ax - synth speed delta) "
              f"mean={residuals.mean():+.4f} std={residuals.std():.4f} m/s "
              f"(p90_abs={np.percentile(np.abs(residuals), 90):.4f} m/s)")
    else:
        print("  no fully-moving spans found for the ax cross-check")
    return synth_kmh


def _params_with_source(base_params, source):
    p = copy.deepcopy(base_params)
    p["stability_estimation"]["sideslip_source"] = source
    return p


def run_pipeline_with_channels(channels, raw_data_template, params, csv_path):
    data = dict(raw_data_template)
    data["channels"] = channels
    state = prepare_vehicle_state(channels, params)
    if state is None:
        raise RuntimeError("prepare_vehicle_state returned None")
    source = params["stability_estimation"]["sideslip_source"]
    beta, _fm, _gate, fallback_used, fallback_reason = resolve_sideslip_beta(
        state, params, data, source, csv_path=csv_path)
    if fallback_used:
        print(f"  ** NOTE: sideslip_source={source!r} fell back to kinematic: {fallback_reason}")
    slip = estimate_slip_angles(state, beta, params)
    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)
    return {"state": state, "slip": slip, "forces": forces, "cs": cs}


def part_c(data, base_params, synth_kmh_on_original_grid, original_state, forced_cid=13):
    print(f"\n=== Part (c): C{forced_cid} CS chain, synthetic speed substituted, all else identical ===")
    params = _params_with_source(base_params, "ekf_auto_pacejka")

    print("-- baseline (real ecu_speed) --")
    baseline = run_pipeline_with_channels(data["channels"], data, params, csv_path=V3_FILE)

    corners_by_stable_id = {}
    for c in data.get("corners", []):
        sid = c.get("stable_corner_id")
        if sid is not None and c.get("bracket_start_m") is not None and c.get("bracket_end_m") is not None:
            corners_by_stable_id.setdefault(sid, []).append(c)
    laps_by_number = {l["lap_number"]: l for l in data.get("laps", [])}
    insts_all = corners_by_stable_id.get(forced_cid, [])
    instances = _valid_lap_instances(insts_all, laps_by_number)
    if not instances:
        print(f"  C{forced_cid}: no valid lap instances -- aborting Part (c)")
        return

    t0, s_m0 = original_state["time"], original_state["s_m"]
    best = None
    for axle, key in (("front", "CS_ratio_f"), ("rear", "CS_ratio_r")):
        arr = baseline["cs"][key]
        for c in instances:
            lap = laps_by_number[c["lap_number"]]
            sl = _canonical_window_slice(t0, s_m0, lap["start_time"], lap["end_time"],
                                          c["bracket_start_m"], c["bracket_end_m"])
            if sl.stop <= sl.start:
                continue
            rate, n = _sign_change_rate(arr[sl])
            if n < 3 or not np.isfinite(rate):
                continue
            if best is None or rate > best[0]:
                best = (rate, axle, c["lap_number"], sl, n)
    if best is None:
        print(f"  C{forced_cid}: no scorable instance -- aborting Part (c)")
        return
    rate0, axle, lap_number, sl, n = best
    cs_key = "CS_ratio_f" if axle == "front" else "CS_ratio_r"
    print(f"  worst instance: axle={axle} lap={lap_number} n={n} baseline sign_change_rate={rate0:.3f}")

    # Build the ecu_speed-substituted channel set: replace ecu_speed's own
    # DATA (not time -- prepare_vehicle_state's grid uses the time array's
    # endpoints only) with the synthetic reference, interpolated back onto
    # ecu_speed's own native timestamps. Everything else (corners, laps,
    # every other channel) is untouched.
    ecu_ch = data["channels"]["ecu_speed"]
    synth_on_native = np.interp(ecu_ch["time"], original_state["time"], synth_kmh_on_original_grid)
    modified_channels = copy.deepcopy(data["channels"])
    modified_channels["ecu_speed"] = dict(ecu_ch)
    modified_channels["ecu_speed"]["data"] = synth_on_native

    print("-- substituted (synthetic speed in place of ecu_speed) --")
    substituted = run_pipeline_with_channels(modified_channels, data, params, csv_path=None)

    t1, s_m1 = substituted["state"]["time"], substituted["state"]["s_m"]
    lap = laps_by_number[lap_number]
    sl1 = _canonical_window_slice(t1, s_m1, lap["start_time"], lap["end_time"],
                                   [c for c in instances if c["lap_number"] == lap_number][0]["bracket_start_m"],
                                   [c for c in instances if c["lap_number"] == lap_number][0]["bracket_end_m"])
    rate1, n1 = _sign_change_rate(substituted["cs"][cs_key][sl1])
    print(f"  substituted sign_change_rate={rate1:.3f} (n={n1}) vs baseline {rate0:.3f} (n={n})")
    verdict = "SOFTENS" if rate1 < rate0 - 0.02 else ("WORSENS" if rate1 > rate0 + 0.02 else "NO MATERIAL CHANGE")
    print(f"  PRE-REGISTERED VERDICT: {verdict}")

    baseline_vals = baseline["cs"][cs_key][sl]
    substituted_vals = substituted["cs"][cs_key][sl1]
    print(f"  baseline CS_ratio finite n={np.isfinite(baseline_vals).sum()} "
          f"median={np.nanmedian(baseline_vals):.3f} min={np.nanmin(baseline_vals):.3f} max={np.nanmax(baseline_vals):.3f}")
    print(f"  substituted CS_ratio finite n={np.isfinite(substituted_vals).sum()} "
          f"median={np.nanmedian(substituted_vals):.3f} min={np.nanmin(substituted_vals):.3f} max={np.nanmax(substituted_vals):.3f}")

    os.makedirs(PLOT_DIR, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(np.arange(len(baseline_vals)), baseline_vals, label=f"baseline (real ecu_speed), rate={rate0:.3f}",
            color="black", lw=1.0)
    ax.plot(np.arange(len(substituted_vals)), substituted_vals,
            label=f"substituted (synthetic speed), rate={rate1:.3f}", color="tab:red", lw=1.0, alpha=0.8)
    ax.set_xlabel("sample index within window"); ax.set_ylabel("CS_ratio")
    ax.set_title(f"C{forced_cid} ({axle}, lap {lap_number}): CS_ratio, real vs synthetic ecu_speed -- {verdict}")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOT_DIR, f"ecu_speed_substitution_C{forced_cid}_{axle}.png"), dpi=130)
    plt.close(fig)
    print(f"  figure saved: {PLOT_DIR}/ecu_speed_substitution_C{forced_cid}_{axle}.png")

    return {
        "forced_cid": forced_cid, "axle": axle, "lap_number": lap_number,
        "rate0": rate0, "rate1": rate1, "verdict": verdict,
        "t0": t0[sl], "baseline_vals": baseline_vals,
        "t1": t1[sl1], "substituted_vals": substituted_vals,
    }


def main():
    params = load_parameters()
    data = parse_csv(V3_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        print("prepare_vehicle_state returned None for v3 -- aborting")
        return

    wheel_kmh, healthy_median = part_a(state, data["channels"], V3_FILE)
    synth_kmh = part_b(state, data["channels"], params)

    print("\n\n########## CONTROL: Sample_Dubai.txt ##########")
    dubai_data = parse_csv(DUBAI_FILE)
    dubai_state = prepare_vehicle_state(dubai_data["channels"], params)
    if dubai_state is not None:
        part_a(dubai_state, dubai_data["channels"], DUBAI_FILE)
        part_b(dubai_state, dubai_data["channels"], params)
    else:
        print("prepare_vehicle_state returned None for Dubai -- control skipped")

    result = part_c(data, params, synth_kmh, state, forced_cid=13)
    return result


if __name__ == "__main__":
    main()
