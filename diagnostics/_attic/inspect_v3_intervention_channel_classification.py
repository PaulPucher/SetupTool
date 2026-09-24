# Frame-Stage-2 work package, Phase 2: intervention-channel survey on
# GT3_PRC_MLA-v3.txt, read-only. Extends the damper package's own Phase 4
# channel-identification survey (thesis_notes.md "Damper package...
# Phase 4 channel survey") with (a) explicit boolean/level/continuous
# classification and (b) co-occurrence with braking/traction events, per
# this phase's own work order -- Phase 4 established candidate NAMES,
# this phase decides whether each is USABLE-NOW (self-identifying
# activity), READ-AND-RECORD (a level pending engineer confirmation), or
# UNCLEAR. Wires NOTHING -- no config/production change.

import io

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "GT3_PRC_MLA-v3.txt"
ENCODING = "latin-1"

CANDIDATES = [
    "ecu_B_tc_act", "ecu_TC_int_pos", "log_tc_int_pos",
    "ecu_EB_int_pos", "ecu_EB_tim_pos",
    "abs_active", "abs_switch_pos", "log_abs_pos", "log_rt_abs_pos",
    "Math_Brake_Bias_Hold",
    "abs_brk_bal_prop", "abs_brk_bal_prop_ad", "abs_brk_bal_at50", "abs_brk_bal_at50_adv",
]


def read_blocks():
    blocks = {name: ([], []) for name in CANDIDATES}
    current = None
    with io.open(RAW_FILE, encoding=ENCODING) as f:
        for line in f:
            if line.startswith("Time\t"):
                raw_name = line.strip().split("\t", 1)[1]
                base = raw_name.split("[")[0]
                current = base if base in CANDIDATES else None
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


def classify(name, t, v, t_ref, moving):
    if len(v) == 0:
        print(f"{name}: ABSENT")
        return None
    n_distinct = len(np.unique(v))
    rate = len(v) / (t[-1] - t[0]) if t[-1] > t[0] else float("nan")
    on_ref = np.interp(t_ref, t, v)
    kind = "BOOLEAN" if n_distinct <= 2 else ("LEVEL" if n_distinct <= 20 else "CONTINUOUS")
    print(f"{name}: n={len(v)} rate~{rate:.1f}Hz n_distinct={n_distinct} kind={kind} "
          f"range=[{v.min():.3f},{v.max():.3f}] changes_during_session={n_distinct > 1}")
    return on_ref, kind, n_distinct


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    state = prepare_vehicle_state(data["channels"], params)
    t_ref = state["time"]
    moving = state["moving_mask"]
    brake_f = np.interp(t_ref, data["channels"]["log_pbrake_f"]["time"], data["channels"]["log_pbrake_f"]["data"])
    brake_r = np.interp(t_ref, data["channels"]["log_pbrake_r"]["time"], data["channels"]["log_pbrake_r"]["data"])
    throttle = np.interp(t_ref, data["channels"]["ecu_aps"]["time"], data["channels"]["ecu_aps"]["data"])
    ax = state["ax_mps2"]

    # Same "hard braking" convention as the already-recorded ABS consistency
    # check (thesis_notes.md "Morning follow-up... ABS consistency check"):
    # combined front+rear brake pressure > 120 bar -- reused exactly so this
    # phase's own numbers are directly comparable, not a second incompatible
    # definition.
    hard_braking = moving & (brake_f + brake_r > 120.0)
    traction_event = moving & (throttle > 80.0) & (ax > 1.0)  # high throttle, genuinely accelerating

    print(f"reference masks: hard_braking n={hard_braking.sum()} ({hard_braking.mean()*100:.2f}% of samples), "
          f"traction_event n={traction_event.sum()} ({traction_event.mean()*100:.2f}% of samples)\n")

    blocks = read_blocks()
    results = {}
    for name in CANDIDATES:
        t, v = blocks[name]
        r = classify(name, t, v, t_ref, moving)
        if r is not None:
            results[name] = r

    print(f"\n(hard_braking mask recomputed with combined front+rear brake pressure > 120 bar, "
          f"same convention as the already-recorded ABS consistency check: n={hard_braking.sum()} "
          f"({hard_braking.mean()*100:.2f}% of samples))")

    print("\n--- BOOLEAN channels: clean activity co-occurrence ---")
    for name, (on_ref, kind, n_distinct) in results.items():
        if kind != "BOOLEAN" or n_distinct < 2:
            continue
        active = on_ref > 0.5
        n_active = int(active.sum())
        print(f"{name}: n_active_samples={n_active} ({active.mean()*100:.4f}% of all samples)")
        if n_active == 0:
            continue
        frac_active_during_braking = active[hard_braking].mean() if hard_braking.any() else float("nan")
        frac_braking_during_active = hard_braking[active].mean() if active.any() else float("nan")
        frac_active_during_traction = active[traction_event].mean() if traction_event.any() else float("nan")
        frac_traction_during_active = traction_event[active].mean() if active.any() else float("nan")
        print(f"  active-during-braking={frac_active_during_braking*100:.2f}% braking-during-active={frac_braking_during_active*100:.2f}% | "
              f"active-during-traction={frac_active_during_traction*100:.2f}% traction-during-active={frac_traction_during_active*100:.2f}%")

    print("\n--- LEVEL/CONTINUOUS channels: value distribution during braking/traction vs neither ---")
    for name, (on_ref, kind, n_distinct) in results.items():
        if kind not in ("LEVEL", "CONTINUOUS") or n_distinct <= 1:
            continue
        neither = moving & ~hard_braking & ~traction_event
        print(f"{name}: mean during hard_braking={np.mean(on_ref[hard_braking]):.3f} (n={hard_braking.sum()}) | "
              f"mean during traction_event={np.mean(on_ref[traction_event]):.3f} (n={traction_event.sum()}) | "
              f"mean during neither={np.mean(on_ref[neither]):.3f} (n={neither.sum()})")


if __name__ == "__main__":
    main()
