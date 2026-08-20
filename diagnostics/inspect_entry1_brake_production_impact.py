# entry_1_brake fix: production-output impact measurement. Read-only,
# no code change, no config change, no commit.
#
# BASELINE CHOICE: compares the CURRENT (fixed) code's output against
# a recomputed "before" using the ORIGINAL pre-session bug
# (off_throttle[0], unbounded backward search) -- not the two failed
# intermediate attempts. This is the correct baseline because nothing
# in this session's entry_1_brake work has been committed; whatever is
# actually persisted in the local DB (if anything) predates this
# session entirely and reflects the original bug, not an intermediate
# state nobody but this session's own diagnostics ever saw. "Before" is
# reconstructed here (not by reverting the file) using the exact
# original formula, applied only to entry_1_brake -- every other phase
# boundary is already confirmed correct (prior turn) and left
# untouched, so isolating the comparison to entry_1_brake alone is
# both correct and sufficient.
#
# Mirrors the real production call sequence exactly (ui/views/
# outing_form.py's analysis thread, confirmed by direct read):
# estimate_sideslip -> estimate_slip_angles -> estimate_lateral_forces
# -> estimate_cornering_stiffness -> estimate_yaw_moment_stability ->
# estimate_vertical_loads -> summarise_corners(..., fz=fz). Uses plain
# load_parameters() (Level 1 params, no accuracy-cap resolution) for
# both before/after -- identical for both sides, so the comparison
# isolates the phase-boundary fix's effect only.

import copy

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters, prepare_vehicle_state, estimate_sideslip,
    estimate_slip_angles, estimate_lateral_forces, estimate_cornering_stiffness,
    estimate_yaw_moment_stability, estimate_vertical_loads, summarise_corners,
)
from modules.recommendation import aggregate_by_corner, PHASE_KEYS
from ui.views.outing_form import OutingForm

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
BRAKE_THROTTLE_MAX_PCT = 95  # config/channels.json corner_detection, read live below


def read_raw_channels(file_path, wanted_names):
    wanted = set(wanted_names)
    out = {}
    with open(file_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    i, n = 0, len(lines)
    while i < n:
        if lines[i].strip() == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    raw_name = header_parts[1].strip()
                    channel_name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
                    i += 1
                    if channel_name in wanted:
                        times, values = [], []
                        while i < n and not lines[i].strip().startswith("{"):
                            raw_line = lines[i].strip()
                            if raw_line:
                                parts = raw_line.split("\t")
                                if len(parts) == 2:
                                    try:
                                        tt = float(parts[0].replace(",", "."))
                                        vv = float(parts[1].replace(",", "."))
                                        times.append(tt)
                                        values.append(vv)
                                    except ValueError:
                                        pass
                            i += 1
                        out[channel_name] = {"time": np.array(times), "data": np.array(values)}
                        continue
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                        continue
        i += 1
    return out


data = parse_csv(RAW_FILE)
params = load_parameters()
config_channels = __import__("json").load(open("config/channels.json", encoding="utf-8"))
brake_throttle_max_pct = config_channels["corner_detection"]["brake_throttle_max_pct"]
print(f"config brake_throttle_max_pct={brake_throttle_max_pct} (used below)")

state = prepare_vehicle_state(data["channels"], params)
beta = estimate_sideslip(state, params)
slip = estimate_slip_angles(state, beta, params)
forces = estimate_lateral_forces(state, params)
cs = estimate_cornering_stiffness(slip, forces, state, params)
stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))
fz = estimate_vertical_loads(state, forces, params)

corners_after = data.get("corners", [])
laps = data.get("laps", [])
laps_by_number = {l["lap_number"]: l for l in laps}

raw_throttle = read_raw_channels(RAW_FILE, ["ecu_aps"])["ecu_aps"]

corners_before = []
for c in corners_after:
    c2 = copy.deepcopy(c)
    lap = laps_by_number[c["lap_number"]]
    s_t_start_abs = c["segments"]["entry_2_turnin"][0]
    thr_mask = ((raw_throttle["time"] >= lap["start_time"])
                & (raw_throttle["time"] < s_t_start_abs))
    brake_start_t_abs = s_t_start_abs
    if thr_mask.any():
        thr_t = raw_throttle["time"][thr_mask]
        thr_d = raw_throttle["data"][thr_mask]
        off_throttle = np.where(thr_d < brake_throttle_max_pct)[0]
        if len(off_throttle) > 0:
            brake_start_t_abs = float(thr_t[off_throttle[0]])  # ORIGINAL bug: [0], not [-1]
    c2["segments"] = dict(c["segments"])
    c2["segments"]["entry_1_brake"] = (brake_start_t_abs, s_t_start_abs)
    corners_before.append(c2)

summaries_after = summarise_corners(corners_after, cs, stab, state, fz=fz)
summaries_before = summarise_corners(corners_before, cs, stab, state, fz=fz)

print("=" * 78)
print("ITEM 1 -- per-corner-instance entry_1_brake phase stats, before vs after")
print("=" * 78)
by_key_after = {(s["lap_number"], s["corner_number"], s["stable_corner_id"]): s for s in summaries_after}
by_key_before = {(s["lap_number"], s["corner_number"], s["stable_corner_id"]): s for s in summaries_before}

MATERIAL_THRESHOLD = 0.05  # CS_ratio units, Tier B reporting threshold for this comparison only
material_shifts = []
for key in sorted(by_key_after, key=lambda k: (k[2] if k[2] is not None else -1, k[0])):
    sa = by_key_after[key]["phases"]["entry_1_brake"]
    sb = by_key_before[key]["phases"]["entry_1_brake"]
    csf_a, csf_b = sa["cs_ratio_f"]["median"], sb["cs_ratio_f"]["median"]
    csr_a, csr_b = sa["cs_ratio_r"]["median"], sb["cs_ratio_r"]["median"]
    stb_a, stb_b = sa["stability_observed_Nm_per_deg"]["median"], sb["stability_observed_Nm_per_deg"]["median"]
    lap, cn, cid = key
    print(f"  lap={lap} corner_number={cn} stable_id={cid}  n_after={sa['n_samples']} n_before={sb['n_samples']}  "
          f"CSf: {csf_b if csf_b==csf_b else float('nan'):.3f}->{csf_a if csf_a==csf_a else float('nan'):.3f}  "
          f"CSr: {csr_b if csr_b==csr_b else float('nan'):.3f}->{csr_a if csr_a==csr_a else float('nan'):.3f}  "
          f"Stab: {stb_b if stb_b==stb_b else float('nan'):.0f}->{stb_a if stb_a==stb_a else float('nan'):.0f}")
    for label, a, b in [("CSf", csf_a, csf_b), ("CSr", csr_a, csr_b)]:
        if a == a and b == b and abs(a - b) >= MATERIAL_THRESHOLD:
            material_shifts.append((key, label, b, a, a - b))
    if stb_a == stb_a and stb_b == stb_b and abs(stb_a - stb_b) >= 50:
        material_shifts.append((key, "Stab", stb_b, stb_a, stb_a - stb_b))

print()
print(f"  MATERIAL SHIFTS (|CS delta|>={MATERIAL_THRESHOLD} or |Stab delta|>=50 Nm/deg): {len(material_shifts)}")
for key, label, b, a, d in material_shifts:
    print(f"    lap={key[0]} corner_number={key[1]} stable_id={key[2]}  {label}: {b:.3f} -> {a:.3f}  (delta={d:+.3f})")
print()

print("=" * 78)
print("ITEM 2 -- braking-matrix recommendation verdicts, before vs after")
print("=" * 78)
agg_after = aggregate_by_corner(summaries_after)
agg_before = aggregate_by_corner(summaries_before)

BRAKING_RULE_PHASES = {
    "yaw_entry_unstable": ["entry_1_brake", "entry_2_turnin"],
    "driver_us_entry": ["entry_1_brake", "entry_2_turnin"],
    "matrix_us_brk_low": ["entry_1_brake"],
    "matrix_us_brk_low_esc": ["entry_1_brake"],
    "matrix_us_brk_med": ["entry_1_brake"],
    "matrix_us_brk_med_esc": ["entry_1_brake"],
    "matrix_us_brk_high": ["entry_1_brake"],
    "matrix_us_brk_high_esc": ["entry_1_brake"],
    "matrix_os_brk_low": ["entry_1_brake"],
    "matrix_os_brk_med": ["entry_1_brake"],
    "matrix_os_brk_high": ["entry_1_brake"],
    "matrix_inst_brk_low": ["entry_1_brake"],
    "matrix_inst_brk_med": ["entry_1_brake"],
    "matrix_inst_brk_high": ["entry_1_brake"],
    "matrix_inst_ent": ["entry_1_brake"],
}


def phase_verdict(aggregated_corner, phases):
    sliced = {p: aggregated_corner["phases"][p] for p in phases if p in aggregated_corner["phases"]}
    severity, short, _long, _colour = OutingForm._classify_corner(None, {"phases": sliced})
    return severity, short


changes = []
for cid in sorted(agg_after):
    for rule_id, phases in BRAKING_RULE_PHASES.items():
        sev_a, short_a = phase_verdict(agg_after[cid], phases)
        sev_b, short_b = phase_verdict(agg_before[cid], phases)
        if (sev_a, short_a) != (sev_b, short_b):
            changes.append((cid, rule_id, sev_b, short_b, sev_a, short_a))

if not changes:
    print("  NO verdict changed for ANY of the 15 braking-scoped rule-phase combinations, "
          "any stable corner. Reported plainly per instruction.")
else:
    print(f"  {len(changes)} rule x corner combinations changed verdict:")
    for cid, rule_id, sev_b, short_b, sev_a, short_a in changes:
        direction = ("MORE severe" if (short_b == "ok" and short_a != "ok") else
                     "LESS severe (toward ok)" if (short_a == "ok" and short_b != "ok") else
                     "changed")
        print(f"    C{cid}  {rule_id}:  before=({sev_b}, '{short_b}')  after=({sev_a}, '{short_a}')  [{direction}]")

print()
print("  False-negative hypothesis check: count directions among changes")
n_more_severe = sum(1 for c in changes if c[3] == "ok" and c[5] != "ok")
n_less_severe = sum(1 for c in changes if c[5] == "ok" and c[3] != "ok")
n_other = len(changes) - n_more_severe - n_less_severe
print(f"    before=ok -> after=flagged (fix REVEALS a problem, supports false-negative hypothesis): {n_more_severe}")
print(f"    before=flagged -> after=ok (fix REMOVES a flag): {n_less_severe}")
print(f"    other severity/verdict-string change (same ok/flagged status): {n_other}")

print()
print("=" * 78)
print("ITEM 3 -- empty-phase count (entry_1_brake, n_samples==0), AFTER (current code)")
print("=" * 78)
n_empty_after = sum(1 for s in summaries_after if s["phases"]["entry_1_brake"]["n_samples"] == 0)
n_total = len(summaries_after)
print(f"  {n_empty_after} / {n_total} corner instances have entry_1_brake n_samples==0 "
      f"({100*n_empty_after/n_total:.1f}%)")
n_empty_before = sum(1 for s in summaries_before if s["phases"]["entry_1_brake"]["n_samples"] == 0)
print(f"  (for reference, BEFORE the fix: {n_empty_before} / {n_total} were empty)")
