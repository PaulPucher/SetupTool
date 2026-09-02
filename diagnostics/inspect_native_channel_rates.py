# DIAGNOSTIC, read-only, stop at numbers -- no pipeline change.
# For every whitelisted channel (config/channels.json), reports the
# NATIVE sample rate (1/median(diff(time))) of that channel's own time
# vector, as parsed BEFORE any resampling onto a common grid. Groups the
# CS-chain channels (ecu_speed, sclu_yaw_rate, log_asteer, log_acc_y,
# log_acc_z, lap_distance) separately from every other whitelisted
# channel, and states the slowest of the CS-chain group as the maximum
# common rate that chain actually supports.

import json
import numpy as np

from modules.csv_parser import parse_csv

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
CS_CHAIN_CHANNELS = ["ecu_speed", "sclu_yaw_rate", "log_asteer", "log_acc_y", "log_acc_z", "lap_distance"]


def native_rate_hz(channel):
    t = channel.get("time")
    if t is None or len(t) < 2:
        return float("nan"), 0
    dt = np.diff(np.asarray(t, dtype=float))
    dt = dt[dt > 0]
    if len(dt) == 0:
        return float("nan"), len(t)
    return float(1.0 / np.median(dt)), len(t)


def main():
    channels_cfg = json.load(open("config/channels.json", encoding="utf-8"))["channels"]
    whitelist = list(channels_cfg.keys())

    data = parse_csv(RAW_FILE)
    ch = data["channels"]

    rows = []
    for name in whitelist:
        c = ch.get(name)
        if c is None or not isinstance(c, dict) or "time" not in c:
            rows.append((name, None, None, "not present in parsed data"))
            continue
        rate, n = native_rate_hz(c)
        rows.append((name, rate, n, c.get("quality")))

    by_name = {r[0]: r for r in rows}

    print(f"\n{'=' * 78}\n(a) CS-CHAIN CHANNELS\n{'=' * 78}")
    cs_rates = []
    for name in CS_CHAIN_CHANNELS:
        r = by_name.get(name)
        if r is None:
            print(f"  {name:>20}: WHITELIST ENTRY NOT FOUND")
            continue
        _n, rate, n_samples, quality = r
        if rate is None or rate != rate:
            print(f"  {name:>20}: no native rate available (quality={quality}, n_samples={n_samples})")
            continue
        cs_rates.append((name, rate))
        print(f"  {name:>20}: {rate:8.3f} Hz  (n_samples={n_samples}, quality={quality})")

    print(f"\n{'=' * 78}\n(b) EVERYTHING ELSE (whitelisted)\n{'=' * 78}")
    for name, rate, n_samples, quality in rows:
        if name in CS_CHAIN_CHANNELS:
            continue
        if rate is None or rate != rate:
            print(f"  {name:>28}: no native rate available (quality={quality})")
            continue
        print(f"  {name:>28}: {rate:8.3f} Hz  (n_samples={n_samples}, quality={quality})")

    print(f"\n{'=' * 78}")
    if cs_rates:
        slowest_name, slowest_rate = min(cs_rates, key=lambda x: x[1])
        print(f"CONCLUSION: slowest CS-chain channel is {slowest_name} at {slowest_rate:.3f} Hz -- "
              f"this is the maximum common rate the CS chain actually supports.")
    else:
        print("CONCLUSION: no CS-chain channel had a usable native rate -- cannot state a common rate.")


if __name__ == "__main__":
    main()
