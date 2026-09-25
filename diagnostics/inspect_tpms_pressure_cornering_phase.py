# WP-ELICIT Phase B2 (2026-09-24), channel-census verification, read-only.
# Before activating config/decision_frame.json's tyre_pressure_target
# check against the elicited bands (front 1.85-1.95, rear 1.80-1.90 bar),
# confirm tpms_press_fl/fr/rl/rr actually read plausible hot pressure in
# bar during CORNERING-phase samples specifically (the check only ever
# fires there -- straight-line pressure drop is physics, never a flag,
# per DECISION LAYER SPEC Stage 1). The 2026-09-22 channel census
# (config/decision_frame.json plausibility_checks.tyre_pressure_window)
# found these channels present/populated whole-session; that is not the
# same claim as "cornering-phase values sit near 1.85-1.95/1.80-1.90
# bar" -- the channel census rule requires checking the actual claim
# being relied on, not a related prior one.

import numpy as np

from modules.csv_parser import parse_csv, _split_name_unit
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.accuracy_resolution import resolve_accuracy, apply_resolved_vehicle

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "C:/UNI/Bachelorarbeit/outings/GT3_PRC_MLA-v3.txt"

CORNERING_PHASES = ["entry_2_turnin", "apex_3", "exit_4", "exit_5"]
TPMS_CHANNELS = ["tpms_press_fl", "tpms_press_fr", "tpms_press_rl", "tpms_press_rr"]
SANITY_LO, SANITY_HI = 0.5, 5.0  # bar -- excludes known startup/dropout glitches


def _read_raw_channels_bypassing_whitelist(file_path, wanted_channels):
    # modules.csv_parser.parse_csv only extracts channels listed in
    # config/channels.json (line 2 of that module: "Reads only channels
    # defined in config/channels.json") -- tpms_press_* are NOT in that
    # file (verified directly, zero matches), so the production parser
    # silently drops them even though they are present in the raw export
    # (docs/channel_requirements.md). This is a diagnostic-only bypass
    # reusing the exact same NARROW/WIDE block parsing logic, with an
    # explicit wanted_channels set instead of config's, so this script
    # can see them without touching config/channels.json or parse_csv
    # itself.
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
    resolved = resolve_accuracy(params, setup_data=None, cap=1)
    effective_params = apply_resolved_vehicle(params, resolved)
    data = parse_csv(raw_file)
    channels = data["channels"]

    tpms_channels = _read_raw_channels_bypassing_whitelist(raw_file, set(TPMS_CHANNELS))
    present = [k for k in TPMS_CHANNELS if k in tpms_channels]
    print(f"TPMS channels found in raw file (bypassing config/channels.json's whitelist): {present}")
    missing = [k for k in TPMS_CHANNELS if k not in tpms_channels]
    if missing:
        print(f"TPMS channels MISSING even in raw file: {missing}")
    for k in present:
        print(f"  {k}: unit_raw={tpms_channels[k]['unit_raw']!r}, n_samples={len(tpms_channels[k]['data'])}")

    state = prepare_vehicle_state(channels, effective_params)
    if state is None:
        print("prepare_vehicle_state returned None -- cannot segment phases, aborting this session")
        return

    corners = data.get("corners", [])
    print(f"corner instances: {len(corners)}")

    for ch_key in present:
        ch = tpms_channels[ch_key]
        ch_time, ch_data = ch["time"], ch["data"]
        if ch_time is None or ch_data is None or len(ch_time) == 0:
            print(f"{ch_key}: no data")
            continue

        collected = []
        for c in corners:
            segments = c.get("segments", {})
            for phase in CORNERING_PHASES:
                if phase not in segments:
                    continue
                start_t, end_t = segments[phase]
                if end_t < start_t:
                    continue
                lo = int(np.searchsorted(ch_time, start_t, side="left"))
                hi = int(np.searchsorted(ch_time, end_t, side="right"))
                if hi > lo:
                    collected.append(ch_data[lo:hi])

        if not collected:
            print(f"{ch_key}: no cornering-phase samples collected")
            continue

        vals = np.concatenate(collected).astype(float)
        finite = vals[np.isfinite(vals)]
        sane = finite[(finite >= SANITY_LO) & (finite <= SANITY_HI)]
        excluded = len(finite) - len(sane)

        print(f"{ch_key}: raw_cornering_samples={len(vals)} finite={len(finite)} "
              f"sane_in_[{SANITY_LO},{SANITY_HI}]bar={len(sane)} excluded_as_glitch={excluded}")
        if len(sane) > 0:
            print(f"  min={sane.min():.3f} p10={np.percentile(sane, 10):.3f} "
                  f"median={np.median(sane):.3f} p90={np.percentile(sane, 90):.3f} "
                  f"max={sane.max():.3f} bar")


if __name__ == "__main__":
    run("Dubai", DUBAI_FILE)
    run("v3", V3_FILE)
