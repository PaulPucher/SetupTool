# WP-ELICIT HANDOFF item 5 (2026-09-25), Phase D brake-bias channel
# identity, read-only diagnostic. Recomputes percent-front brake bias
# from raw front/rear brake pressure during real braking events, both
# real sessions, and correlates it against every candidate channel named
# in the Frame-Stage-2 Phase 2 record (thesis_notes.md "Frame-Stage-2
# Phase 2: intervention-channel survey", 2026-09-04/05) -- names read
# directly from that record (and from its own source script,
# diagnostics/_attic/inspect_v3_intervention_channel_classification.py),
# never from memory. Wires NOTHING -- no config/production change from
# this script itself; a clean correlation is reported for the reviewer
# to act on elsewhere (this WP-ELICIT package's own item 5 handling).

import numpy as np

from modules.csv_parser import parse_csv, _split_name_unit
from modules.stability_analysis import load_parameters, prepare_vehicle_state

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "C:/UNI/Bachelorarbeit/outings/GT3_PRC_MLA-v3.txt"

# Exact names from the Frame-Stage-2 Phase 2 record: Math_Brake_Bias_Hold
# (that record's own READ-AND-RECORD category) plus the four abs_brk_
# bal_* variants (that record's own UNCLEAR category -- four competing,
# differently-scaled candidates for the same physical quantity).
CANDIDATE_CHANNELS = [
    "Math_Brake_Bias_Hold",
    "abs_brk_bal_prop", "abs_brk_bal_prop_ad",
    "abs_brk_bal_at50", "abs_brk_bal_at50_adv",
]

# Same "hard braking" convention as the already-recorded ABS consistency
# check and Frame-Stage-2 Phase 2 (thesis_notes.md): combined front+rear
# brake pressure > 120 bar, reused exactly rather than a second,
# incompatible braking-event definition.
BRAKE_PRESSURE_SUM_THRESHOLD_BAR = 120.0


def _read_raw_channels_bypassing_whitelist(file_path, wanted_channels):
    # modules.csv_parser.parse_csv only extracts channels listed in
    # config/channels.json -- none of CANDIDATE_CHANNELS are in that file
    # (verified directly, zero matches), so the production parser cannot
    # see them. Identical bypass logic to diagnostics/inspect_tpms_
    # pressure_cornering_phase.py's own helper (handles both the NARROW
    # {ChannelBlock} single-channel format and the WIDE multi-column
    # Time-header format either session may use) -- reused verbatim
    # rather than a second, independently-maintained copy of the same
    # parsing logic.
    with open(file_path, "r", encoding="latin-1") as f:
        lines = f.readlines()
    i, n = 0, len(lines)
    raw_channels = {}
    while i < n:
        line = lines[i].strip()
        if line == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    channel_name, unit_raw = _split_name_unit(header_parts[1])
                    i += 1
                    if channel_name in wanted_channels:
                        times, values = [], []
                        while i < n and not lines[i].strip().startswith("{"):
                            parts = lines[i].strip().split("\t")
                            if len(parts) == 2:
                                try:
                                    t = float(parts[0].replace(",", "."))
                                    v = float(parts[1].replace(",", "."))
                                    times.append(t)
                                    values.append(v)
                                except ValueError:
                                    pass
                            i += 1
                        raw_channels[channel_name] = {
                            "time": np.array(times), "data": np.array(values), "unit_raw": unit_raw,
                        }
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                elif len(header_parts) > 2 and header_parts[0].strip() == "Time":
                    wanted_cols = {}
                    for col_idx, raw in enumerate(header_parts[1:], start=1):
                        name, unit_raw = _split_name_unit(raw)
                        if name in wanted_channels:
                            wanted_cols[col_idx] = (name, unit_raw)
                    col_times = {idx: [] for idx in wanted_cols}
                    col_values = {idx: [] for idx in wanted_cols}
                    i += 1
                    while i < n and not lines[i].strip().startswith("{"):
                        parts = lines[i].rstrip("\r\n").split("\t")
                        if len(parts) > 1:
                            try:
                                t = float(parts[0].replace(",", "."))
                            except ValueError:
                                i += 1
                                continue
                            if t == t:
                                for col_idx in wanted_cols:
                                    if col_idx < len(parts) and parts[col_idx] != "":
                                        try:
                                            v = float(parts[col_idx].replace(",", "."))
                                        except ValueError:
                                            continue
                                        if v == v:
                                            col_times[col_idx].append(t)
                                            col_values[col_idx].append(v)
                        i += 1
                    for col_idx, (name, unit_raw) in wanted_cols.items():
                        raw_channels[name] = {
                            "time": np.array(col_times[col_idx]),
                            "data": np.array(col_values[col_idx]),
                            "unit_raw": unit_raw,
                        }
                else:
                    i += 1
        else:
            i += 1
    return raw_channels


def run(label, raw_file):
    print(f"\n=== {label} ({raw_file}) ===")
    params = load_parameters()
    data = parse_csv(raw_file)
    channels = data["channels"]
    state = prepare_vehicle_state(channels, params)
    if state is None:
        print("prepare_vehicle_state returned None -- cannot build a time reference, aborting this session")
        return
    t_ref = state["time"]
    moving = state["moving_mask"]

    if "log_pbrake_f" not in channels or "log_pbrake_r" not in channels:
        print("log_pbrake_f/log_pbrake_r missing from this session -- cannot recompute percent-front, aborting")
        return
    brake_f = np.interp(t_ref, channels["log_pbrake_f"]["time"], channels["log_pbrake_f"]["data"])
    brake_r = np.interp(t_ref, channels["log_pbrake_r"]["time"], channels["log_pbrake_r"]["data"])
    hard_braking = moving & ((brake_f + brake_r) > BRAKE_PRESSURE_SUM_THRESHOLD_BAR)
    print(f"hard_braking (moving & pbrake_f+pbrake_r>{BRAKE_PRESSURE_SUM_THRESHOLD_BAR} bar): "
          f"n={hard_braking.sum()} ({hard_braking.mean()*100:.2f}% of samples)")
    if not hard_braking.any():
        print("no real braking events found in this session -- cannot recompute percent-front, aborting")
        return

    percent_front = 100.0 * brake_f / (brake_f + brake_r)

    raw = _read_raw_channels_bypassing_whitelist(raw_file, set(CANDIDATE_CHANNELS))
    present = [c for c in CANDIDATE_CHANNELS if c in raw]
    missing = [c for c in CANDIDATE_CHANNELS if c not in raw]
    print(f"candidate channels found in raw file (bypassing config/channels.json's whitelist): {present}")
    if missing:
        print(f"candidate channels MISSING in raw file: {missing}")

    pf_hard = percent_front[hard_braking]
    t_hard = t_ref[hard_braking]
    print(f"recomputed percent-front during hard_braking: n={len(pf_hard)} "
          f"min={pf_hard.min():.3f} median={np.median(pf_hard):.3f} max={pf_hard.max():.3f} %")

    for name in present:
        ch = raw[name]
        ch_t, ch_v = ch["time"], ch["data"]
        if len(ch_t) == 0:
            print(f"{name}: no samples")
            continue
        # Restrict to the candidate channel's OWN real time coverage --
        # interpolating t_ref samples outside [ch_t.min(), ch_t.max()]
        # would hold the nearest edge value artificially flat and bias
        # the correlation, not a real reading.
        in_range = (t_hard >= ch_t.min()) & (t_hard <= ch_t.max())
        n_in_range = int(in_range.sum())
        if n_in_range < 2:
            print(f"{name}: unit_raw={ch['unit_raw']!r} n_samples={len(ch_v)} "
                  f"range=[{ch_v.min():.3f},{ch_v.max():.3f}] -- fewer than 2 overlapping "
                  f"hard_braking samples, correlation not computable")
            continue
        on_ref = np.interp(t_hard[in_range], ch_t, ch_v)
        pf_overlap = pf_hard[in_range]
        if np.std(on_ref) == 0:
            print(f"{name}: unit_raw={ch['unit_raw']!r} n_samples={len(ch_v)} "
                  f"range=[{ch_v.min():.3f},{ch_v.max():.3f}] -- CONSTANT over the overlapping "
                  f"hard_braking window (n={n_in_range}), correlation undefined")
            continue
        corr = float(np.corrcoef(pf_overlap, on_ref)[0, 1])
        print(f"{name}: unit_raw={ch['unit_raw']!r} n_samples={len(ch_v)} "
              f"range=[{ch_v.min():.3f},{ch_v.max():.3f}] overlap_n={n_in_range} "
              f"corr_vs_recomputed_percent_front={corr:+.4f}")


if __name__ == "__main__":
    run("Dubai", DUBAI_FILE)
    run("v3", V3_FILE)
