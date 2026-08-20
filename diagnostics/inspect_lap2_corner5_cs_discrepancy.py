# PART A investigation (UI cleanup package): does the trace dialog's
# per-sample CS_ratio_f/r and the corner-detail dropdown's phase
# median/p25/p75 read the SAME payload, and if they disagree at Lap 2
# Corner 5, is it aggregation (within-phase median washing), a mask
# difference, or staleness? Read-only, no config/production change.
# Runs under the LIVE config sideslip_source (read, not assumed) --
# same estimator the user was looking at when they found this.

import json

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_lateral_forces,
    estimate_cornering_stiffness, estimate_yaw_moment_stability,
    estimate_vertical_loads, summarise_corners,
)
from modules.tyre_fit_auto import resolve_sideslip_beta
from modules.stability_analysis import estimate_slip_angles

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

params = load_parameters()
live_sideslip_source = params["stability_estimation"]["sideslip_source"]
print(f"live config sideslip_source = {live_sideslip_source!r}")

data = parse_csv(RAW_FILE)
state = prepare_vehicle_state(data["channels"], params)

beta, fit_manifest, gate_verdict, fallback_used, fallback_reason = resolve_sideslip_beta(
    state, params, data, live_sideslip_source, csv_path=RAW_FILE
)
print(f"fallback_used={fallback_used}  fallback_reason={fallback_reason}")
if gate_verdict:
    print(f"gate_verdict={gate_verdict['verdict']}  health_score={gate_verdict['health_score']}")

slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
fz = estimate_vertical_loads(state, forces, params)
corners = data.get("corners", [])

# --- 1. identify "Lap 2, corner 5" -----------------------------------------
# Checked ui/views/outing_form.py directly: the card header is built as
# f"Lap {lap_number} - C{stable_corner_id}" (line ~1875) -- the UI does
# NOT display corner_number anywhere (grepped, zero matches). "Corner 5"
# in the user's report is therefore stable_corner_id=5, not corner_number.
lap2_corners = [c for c in corners if c["lap_number"] == 2]
print(f"\nLap 2 has {len(lap2_corners)} corners. corner_number -> stable_corner_id:")
for c in sorted(lap2_corners, key=lambda c: c["apex_time"]):
    print(f"  corner_number={c['corner_number']!r}  stable_corner_id={c.get('stable_corner_id')}  "
          f"apex_time={c['apex_time']:.2f}")

target = next((c for c in lap2_corners if c.get("stable_corner_id") == 5), None)
if target is None:
    raise SystemExit("no stable_corner_id=5 found on lap 2 -- check the UI's own numbering before trusting this script")
print(f"\nTarget: lap 2, stable_corner_id=5 (displayed as 'C5'), corner_number={target.get('corner_number')!r}")

summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=None)
summary = next(s for s in summaries if s["lap_number"] == 2 and s["stable_corner_id"] == 5)

# --- 2. per-sample values vs phase median, side by side --------------------

t = state["time"]
moving = state["moving_mask"]
kerb_mask = state.get("kerb_mask")
moving_no_kerb = moving & ~kerb_mask if kerb_mask is not None else moving

phase_keys = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
apex_half_window = params["stability_estimation"]["apex_half_window_samples"]

print("\n" + "=" * 100)
print("PER-SAMPLE CS_ratio_f/r vs the phase median the detail dropdown shows")
print("=" * 100)

any_front_negative_sample = False
for phase in phase_keys:
    start_t, end_t = target["segments"][phase]
    if end_t < start_t:
        print(f"  {phase}: zero-length, skipped")
        continue
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if phase == "apex_3" and hi <= lo:
        centre = lo
        lo = max(0, centre - apex_half_window)
        hi = min(len(t), centre + apex_half_window + 1)
    sl = slice(lo, hi)

    phase_moving = moving[sl]
    idx = np.where(phase_moving)[0] + lo
    csf_all = cs["CS_ratio_f"][idx]
    csr_all = cs["CS_ratio_r"][idx]
    csf_valid = csf_all[np.isfinite(csf_all)]
    csr_valid = csr_all[np.isfinite(csr_all)]

    p = summary["phases"][phase]
    csf_median_reported = p["cs_ratio_f"]["median"]
    csr_median_reported = p["cs_ratio_r"]["median"]

    n_front_neg = int((csf_valid < 0).sum())
    n_rear_neg = int((csr_valid < 0).sum())
    if n_front_neg > 0:
        any_front_negative_sample = True

    print(f"\n  --- {phase} ---  n_samples(moving, unmasked-by-kerb)={len(idx)}  n_valid_cs_f={len(csf_valid)}")
    print(f"    front: per-sample min={np.min(csf_valid) if len(csf_valid) else float('nan'):.4f}  "
          f"n_negative={n_front_neg}/{len(csf_valid)}  "
          f"MEDIAN(recomputed here)={np.median(csf_valid) if len(csf_valid) else float('nan'):.4f}  "
          f"MEDIAN(from summarise_corners)={csf_median_reported:.4f}")
    print(f"    rear:  per-sample min={np.min(csr_valid) if len(csr_valid) else float('nan'):.4f}  "
          f"n_negative={n_rear_neg}/{len(csr_valid)}  "
          f"MEDIAN(recomputed here)={np.median(csr_valid) if len(csr_valid) else float('nan'):.4f}  "
          f"MEDIAN(from summarise_corners)={csr_median_reported:.4f}")
    if n_front_neg > 0:
        neg_idx_local = np.where(csf_valid < 0)[0]
        print(f"    front negative samples (of {len(csf_valid)} in this phase): "
              f"{n_front_neg} ({100*n_front_neg/len(csf_valid):.1f}%) -- "
              f"values: {np.round(csf_valid[neg_idx_local][:10], 4).tolist()}"
              f"{' ...' if n_front_neg > 10 else ''}")

print("\n" + "=" * 100)
print("SECTION 3: are the two read sites the SAME payload / cache entry?")
print("=" * 100)
print("trace dialog: ui/views/corner_trace_dialog.py show_corner() reads")
print("  cs = stability_result.get('cs')  ->  cs['CS_ratio_f']/['CS_ratio_r']  (raw per-sample arrays)")
print("detail dropdown: ui/views/outing_form.py's card builder reads")
print("  p = summary['phases'][phase]; csf = p['cs_ratio_f']  ->  {'median','p25','p75','n'}")
print("Both summary (dropdown) and stability_result['cs'] (trace) are built in the SAME")
print("_on_stability_done call from the SAME StabilityAnalysisThread.finished payload --")
print("summary comes from summarise_corners(cs, ...) called on the exact same cs dict the")
print("trace dialog later reads out of stability_result. Same object graph, not a second cache.")

print(f"\nany front per-sample CS_ratio_f < 0 anywhere in this corner's phases: {any_front_negative_sample}")
