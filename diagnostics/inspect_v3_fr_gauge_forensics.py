# Frame-Stage-2 work package, Phase 0: FR damper-force gauge forensics on
# GT3_PRC_MLA-v3.txt. Read-only, no config/production change. Re-examines
# the "constant -15.95 million N" verdict (damper package, thesis_notes.md
# "Damper package: wheel loads from pushrod/suspension-travel channels")
# with the Dubai-lesson treatment: full-session census, not a 9-point
# sample, plus a check for a healthy alternate encoding of the same
# physical channel before accepting DEAD as final.
#
# The file is per-channel BLOCK layout (thesis_notes.md "GT3_PRC_MLA-v3
# census"): each channel is one contiguous run of Time/Value rows behind
# its own "Time\t<name>[<unit>]" header line, not a shared time grid. This
# script streams the file once rather than using modules.csv_parser (which
# loads the whole 1.29GB file into memory) since it also needs the
# log_dms_dam_*_dash[kgf] channels, which are not in config/channels.json's
# whitelist at all (found only by a raw header grep, see thesis_notes.md
# this same phase).
#
# (a) per-window std of the raw fr[N] channel across the whole session.
# (b) decoding hypotheses: /1e3, /1e6, offset removal, correlation with FL.
# (c) fr[N] vs the other three axles' unit_raw, and the _dash[kgf] siblings.

import io
import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "GT3_PRC_MLA-v3.txt"
ENCODING = "latin-1"

TARGET_HEADERS = {
    "log_dms_dam_fl[N]", "log_dms_dam_fr[N]", "log_dms_dam_rl[N]", "log_dms_dam_rr[N]",
    "log_dms_dam_fl_dash[kgf]", "log_dms_dam_fr_dash[kgf]",
    "log_dms_dam_rl_dash[kgf]", "log_dms_dam_rr_dash[kgf]",
}


def _read_blocks():
    """One streaming pass: returns {header_name: (time_list, value_list)}."""
    blocks = {name: ([], []) for name in TARGET_HEADERS}
    current = None
    with io.open(RAW_FILE, encoding=ENCODING) as f:
        for line in f:
            if line.startswith("Time\t"):
                name = line.strip().split("\t", 1)[1]
                current = name if name in TARGET_HEADERS else None
                continue
            if current is None:
                continue
            parts = line.strip().split("\t")
            if len(parts) != 2:
                continue
            try:
                t = float(parts[0])
                v = float(parts[1])
            except ValueError:
                continue
            blocks[current][0].append(t)
            blocks[current][1].append(v)
    return {name: (np.array(t), np.array(v)) for name, (t, v) in blocks.items()}


def main():
    print(f"streaming {RAW_FILE} once for {len(TARGET_HEADERS)} target channel blocks...")
    blocks = _read_blocks()

    for name in sorted(TARGET_HEADERS):
        t, v = blocks[name]
        print(f"{name}: n={len(v)}", end="")
        if len(v):
            print(f", span=[{t[0]:.1f},{t[-1]:.1f}]s, mean={v.mean():.6g}, std={v.std():.6g}, "
                  f"min={v.min():.6g}, max={v.max():.6g}")
        else:
            print(" (empty)")

    t_fr, v_fr = blocks["log_dms_dam_fr[N]"]
    t_fl, v_fl = blocks["log_dms_dam_fl[N]"]

    print("\n--- (a) per-window std of raw fr[N] across the whole session ---")
    if len(v_fr) == 0:
        print("fr[N] block empty -- cannot proceed")
        return
    n_windows = 20
    edges = np.linspace(t_fr[0], t_fr[-1], n_windows + 1)
    for i in range(n_windows):
        mask = (t_fr >= edges[i]) & (t_fr < edges[i + 1])
        if mask.sum() == 0:
            print(f"  window {i:2d} [{edges[i]:7.1f},{edges[i+1]:7.1f})s: n=0")
            continue
        seg = v_fr[mask]
        print(f"  window {i:2d} [{edges[i]:7.1f},{edges[i+1]:7.1f})s: n={mask.sum():5d} "
              f"mean={seg.mean():14.2f} std={seg.std():10.4f} min={seg.min():14.2f} max={seg.max():14.2f}")

    print("\n--- (b) decoding hypotheses ---")
    raw_mean = v_fr.mean()
    print(f"raw mean: {raw_mean:.6f}")
    for label, decoded in [
        ("raw / 1e3", v_fr / 1e3),
        ("raw / 1e6", v_fr / 1e6),
        ("raw + 2^31 (uint32 wrap hypothesis)", v_fr + 2**31),
        ("raw - (-2^31) i.e. raw + 2^31 (dup check)", v_fr - (-(2**31))),
        ("raw mod 2^16", np.mod(v_fr, 2**16)),
        ("raw / 2^16", v_fr / 2**16),
    ]:
        print(f"  {label}: mean={decoded.mean():.6g} std={decoded.std():.6g} "
              f"min={decoded.min():.6g} max={decoded.max():.6g}")

    # correlate FR against FL only where sample counts and cadence allow a
    # like-for-like comparison (same block size expected, 100Hz per the v3
    # census) -- if the arrays are equal length, compare directly by index
    # position (both blocks share the same nominal 100Hz grid per channel,
    # not a common shared time array in this file's own layout).
    print("\n--- correlation of each decoding against FL[N] (load-transfer check) ---")
    if len(v_fr) == len(v_fl):
        for label, decoded in [
            ("raw", v_fr),
            ("raw / 1e3", v_fr / 1e3),
            ("raw / 1e6", v_fr / 1e6),
        ]:
            if decoded.std() == 0:
                print(f"  {label}: zero variance, correlation undefined")
                continue
            corr = np.corrcoef(decoded, v_fl)[0, 1]
            print(f"  {label} vs FL[N]: corr={corr:.4f}")
    else:
        print(f"  length mismatch fr[N] n={len(v_fr)} vs fl[N] n={len(v_fl)} -- index-aligned "
              f"correlation not meaningful, skipped")

    print("\n--- (c) fr_dash[kgf] as an alternate encoding of the same physical channel ---")
    t_frd, v_frd = blocks["log_dms_dam_fr_dash[kgf]"]
    if len(v_frd) == 0:
        print("fr_dash[kgf] block empty")
    else:
        print(f"fr_dash[kgf]: n={len(v_frd)} mean={v_frd.mean():.4f} std={v_frd.std():.4f} "
              f"min={v_frd.min():.4f} max={v_frd.max():.4f}")
        # kgf -> N conversion for sanity (1 kgf = 9.80665 N)
        v_frd_N = v_frd * 9.80665
        print(f"fr_dash converted to N (*9.80665): mean={v_frd_N.mean():.2f} std={v_frd_N.std():.2f} "
              f"min={v_frd_N.min():.2f} max={v_frd_N.max():.2f}")
        for name in ["log_dms_dam_fl_dash[kgf]", "log_dms_dam_rl_dash[kgf]", "log_dms_dam_rr_dash[kgf]"]:
            t_o, v_o = blocks[name]
            if len(v_o):
                print(f"{name}: n={len(v_o)} mean={v_o.mean():.4f} std={v_o.std():.4f} "
                      f"min={v_o.min():.4f} max={v_o.max():.4f}")

    print("\n--- (b cont.) time-aligned correlation vs ay/ax on ecu_speed's own grid ---")
    print("(Pearson r is invariant to any additive offset and any positive linear")
    print(" scale, so raw/1e3/1e6/+offset all give the identical r below -- only")
    print(" the DECODING that fixes the offset/scale can be judged by whether the")
    print(" resulting magnitude is physically plausible, not by correlation alone.)")
    data = parse_csv(RAW_FILE)
    params = load_parameters()
    state = prepare_vehicle_state(data["channels"], params)
    if state is None:
        print("prepare_vehicle_state returned None -- skipped")
    else:
        t_ref = state["time"]
        ax = state["ax_mps2"]
        ay = state["ay_mps2"]
        fr_on_ref = np.interp(t_ref, t_fr, v_fr)
        fl_on_ref = np.interp(t_ref, t_fl, v_fl)
        for label, sig in [("fr[N] raw (interp to ref grid)", fr_on_ref),
                            ("fl[N] raw (interp to ref grid)", fl_on_ref)]:
            print(f"{label}: corr(ay)={np.corrcoef(sig, ay)[0,1]:.4f} "
                  f"corr(ax)={np.corrcoef(sig, ax)[0,1]:.4f} corr(FL)={np.corrcoef(sig, fl_on_ref)[0,1]:.4f}")
        # for reference, the module's own convention on a healthy corner (damper
        # package Phase 2 real numbers): corr(ay, Fz_fr) = +0.966, corr(ay, Fz_fl) = -0.893
        print("reference (damper package Phase 2, healthy channels, static+damper Fz "
              "not raw pushrod): corr(ay,Fz_fr)=+0.966, corr(ay,Fz_fl)=-0.893")


if __name__ == "__main__":
    main()
