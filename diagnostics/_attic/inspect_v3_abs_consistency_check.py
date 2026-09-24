# Morning follow-up, Item 3: ABS consistency check, GT3_PRC_MLA-v3.txt.
# Read-only, single streaming pass over the raw file (same per-channel-
# block technique as inspect_v3_tc_eb_abs_channels.py) -- NO channels.json
# change (abs_switch_pos/log_abs_pos/log_rt_abs_pos/abs_active are not,
# and remain not, whitelisted; brake pressure is read from whichever of
# log_pbrake_f/r or abs_pbrake_f/r the file actually carries, streamed
# the same way, not through the production parser). NO mapping
# conclusion drawn -- this answers three plain engineer questions only:
# (1) do the three position-switch candidates ever change value, (2) do
# any changes co-occur, (3) does abs_active fire inside hard-braking
# zones (checked against brake pressure, not asserted to mean anything
# about the switch identity).

import numpy as np

RAW_FILE = "GT3_PRC_MLA-v3.txt"
TARGETS = {
    "abs_switch_pos", "log_abs_pos", "log_rt_abs_pos", "abs_active",
    "log_pbrake_f", "log_pbrake_r", "abs_pbrake_f", "abs_pbrake_r",
}

# "Hard braking" threshold: combined front+rear brake pressure above this
# is treated as a hard-braking sample. Plain, round, stated openly -- not
# a config value, not a percentile fit; chosen as roughly two-thirds of
# this file's own Phase 4-observed peak combined pressure (abs_pbrake_f
# peak 87.99 bar + abs_pbrake_r peak 111.2 bar =~ 199 bar combined peak),
# so "hard braking" means meaningfully committed braking, not any brake
# pressure at all.
HARD_BRAKE_COMBINED_BAR = 120.0


def _stream_channels(path, wanted):
    out = {}
    with open(path, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.strip() == "{ChannelBlock}":
                header = f.readline()
                parts = header.rstrip("\n").split("\t")
                raw_name = parts[1] if len(parts) > 1 else ""
                channel = raw_name.split("[")[0]
                if channel in wanted:
                    times, values = [], []
                    row = f.readline()
                    while row and row != "\n":
                        cols = row.rstrip("\n").split("\t")
                        try:
                            t = float(cols[0])
                            v = float(cols[1])
                        except (ValueError, IndexError):
                            row = f.readline()
                            continue
                        times.append(t)
                        values.append(v)
                        row = f.readline()
                    out[channel] = (np.array(times), np.array(values))
                else:
                    row = f.readline()
                    while row and row != "\n":
                        row = f.readline()
            line = f.readline()
    return out


def _report_switch_channel(name, times, values):
    distinct = np.unique(values)
    if len(distinct) == 1:
        print(f"  {name}: NO CHANGE all session (constant value {distinct[0]:g}, n={len(values)})")
        return None
    change_idx = np.flatnonzero(np.diff(values) != 0)
    change_times = times[change_idx + 1]
    change_to = values[change_idx + 1]
    print(f"  {name}: CHANGES {len(change_idx)} time(s) -- values {distinct.tolist()}")
    for ct, cv in zip(change_times, change_to):
        print(f"    -> changes to {cv:g} at t={ct:.2f}s")
    return change_times


def main():
    found = _stream_channels(RAW_FILE, TARGETS)
    print(f"channels found: {sorted(found.keys())}\n")

    print("=== (1) does each position-switch candidate ever change value? ===")
    change_events = {}
    for name in ("abs_switch_pos", "log_abs_pos", "log_rt_abs_pos"):
        if name not in found:
            print(f"  {name}: NOT FOUND in this file")
            continue
        times, values = found[name]
        change_events[name] = _report_switch_channel(name, times, values)

    print("\n=== (2) do any changes co-occur? ===")
    any_changes = {k: v for k, v in change_events.items() if v is not None and len(v) > 0}
    if not any_changes:
        print("  N/A -- none of the three channels change value at all this session, "
              "so there is nothing that could co-occur.")
    else:
        names = list(any_changes.keys())
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                ta, tb = any_changes[a], any_changes[b]
                for t1 in ta:
                    nearest = tb[np.argmin(np.abs(tb - t1))]
                    coincident = abs(nearest - t1) < 1.0
                    print(f"  {a} change at t={t1:.2f}s vs nearest {b} change at t={nearest:.2f}s "
                          f"-> {'CO-OCCURS (within 1s)' if coincident else 'does not co-occur'}")

    print(f"\n=== (3) does abs_active fire inside hard-braking zones (combined "
          f"brake pressure > {HARD_BRAKE_COMBINED_BAR:.0f} bar)? ===")
    if "abs_active" not in found:
        print("  abs_active NOT FOUND -- cannot check")
        return

    brake_f_name = "abs_pbrake_f" if "abs_pbrake_f" in found else ("log_pbrake_f" if "log_pbrake_f" in found else None)
    brake_r_name = "abs_pbrake_r" if "abs_pbrake_r" in found else ("log_pbrake_r" if "log_pbrake_r" in found else None)
    if brake_f_name is None or brake_r_name is None:
        print("  no brake pressure channel found -- cannot check")
        return
    print(f"  using {brake_f_name}/{brake_r_name} for brake pressure")

    t_active, v_active = found["abs_active"]
    t_bf, v_bf = found[brake_f_name]
    t_br, v_br = found[brake_r_name]

    # Common grid: abs_active's own time base (100Hz, finest of the three
    # per Phase 4's census), interpolate brake pressure onto it.
    bf_on_active = np.interp(t_active, t_bf, v_bf)
    br_on_active = np.interp(t_active, t_br, v_br)
    combined_bar = bf_on_active + br_on_active
    hard_brake = combined_bar > HARD_BRAKE_COMBINED_BAR
    active = v_active > 0.5

    n_hard = int(hard_brake.sum())
    n_active = int(active.sum())
    n_both = int((hard_brake & active).sum())
    print(f"  hard-braking samples: {n_hard} ({n_hard/len(t_active)*100:.2f}% of session)")
    print(f"  abs_active=1 samples: {n_active} ({n_active/len(t_active)*100:.2f}% of session)")
    if n_hard > 0:
        print(f"  fraction of hard-braking samples with abs_active=1: {n_both/n_hard*100:.2f}%")
    if n_active > 0:
        print(f"  fraction of abs_active=1 samples that are hard-braking: {n_both/n_active*100:.2f}%")
    if n_active == 0:
        print("  abs_active never reads 1 this session -- it never fires, in or out of hard braking.")


if __name__ == "__main__":
    main()
