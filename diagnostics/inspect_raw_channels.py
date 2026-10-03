# Raw-channel case file, fresh signal check (PLAN.md PHASE 1 PLAN Q-RAW;
# read-only, Tier B signal description, decides nothing). Question for the
# team note: do the logged inertial channels look like raw sensor outputs or
# like processed bus signals?
#
# RULES, frozen 2026-10-02 before the run.
# Sessions: pr_us_v2, v3, nbr_jst_solution (serial 221) and dubai (serial
#   147, logger contrast). Native samples read from the raw export (header
#   unit kept, nothing converted except where stated).
# Channels: sclu_yaw_rate, sclu_roll_rate, sclu_pitch_rate, sclu_acc_x/y/z,
#   log_acc_x/y/z, ADR2_acc_x/y/z, MRR_EgoYawRate, plus the bus-frame
#   companions sclu_msg_cnt_tx1, sclu_yaw_rate_stat, sclu_acc_y_stat.
# Window: the analysis laps of the (d2) data cache (racing time only).
# Per channel:
#   rate      = 1 / median sample interval.
#   step      = modal non-zero difference between sorted distinct values
#               (quantisation step); distinct-value count.
#   hold      = share of consecutive equal samples; median run length of
#               repeats -> effective update rate = rate / mean run length
#               (a value refreshed slower than it is logged repeats).
#   limits    = min, max; share of samples within one step of either
#               (saturation / clipping plateau).
#   spectrum  = Welch PSD (nperseg 4 s) of the detrended racing signal;
#               level 2-8 Hz (median), level in the top 20 % below Nyquist,
#               drop between them (dB). A raw MEMS output keeps a noise floor
#               up to Nyquist; a steep drop well below Nyquist indicates
#               on-board low-pass filtering.
#   lag       = cross-correlation lag of each lateral accelerometer vs
#               log_acc_y and of MRR_EgoYawRate vs sclu_yaw_rate (common
#               100 Hz grid, +/-1 s).
#   counter   = sclu_msg_cnt_tx1: step distribution and wrap value (frame
#               counter of the sensor message).
# Outputs diagnostics/plots/raw_channels/: raw_<key>.json, psd_<key>.png,
#   raw_channels.txt.

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diagnostics import inspect_tyre_prior_d2 as d2
from diagnostics.inspect_imu_offsets import read_raw_blocks

OUT_DIR = "diagnostics/plots/raw_channels"
KEYS = ("pr_us_v2", "v3", "nbr_jst_solution", "dubai")
SIG = ("sclu_yaw_rate", "sclu_roll_rate", "sclu_pitch_rate", "sclu_acc_x", "sclu_acc_y", "sclu_acc_z",
       "log_acc_x", "log_acc_y", "log_acc_z", "ADR2_acc_x", "ADR2_acc_y", "ADR2_acc_z", "MRR_EgoYawRate")
AUX = ("sclu_msg_cnt_tx1", "sclu_yaw_rate_stat", "sclu_acc_y_stat")
INK, BAND = "#1f2937", "#e5e7eb"
FAMILY_COL = {"sclu": "#2563eb", "log": "#1f2937", "ADR2": "#d97706", "MRR": "#9ca3af"}


def _window(t, laps):
    m = np.zeros(t.size, dtype=bool)
    for l in laps:
        if l.get("is_valid_for_analysis"):
            m |= (t >= l["start_time"]) & (t <= l["end_time"])
    return m


def describe(t, v):
    from scipy.signal import welch, detrend
    rate = float(1.0 / np.median(np.diff(t)))
    u = np.unique(v)
    du = np.diff(u)
    du = du[du > 0]
    step = float(np.median(du[du <= np.percentile(du, 10)])) if du.size else float("nan")
    eq = v[1:] == v[:-1]
    runs, n = [], 1
    for e in eq:
        if e:
            n += 1
        else:
            runs.append(n)
            n = 1
    runs.append(n)
    runs = np.array(runs)
    lo, hi = float(v.min()), float(v.max())
    tol = step if np.isfinite(step) else 0.0
    sat = float(np.mean((v <= lo + tol) | (v >= hi - tol)))
    out = {"rate_hz": rate, "n": int(v.size), "distinct": int(u.size), "step": step,
           "hold_share": float(np.mean(eq)), "run_mean": float(runs.mean()), "run_p90": float(np.percentile(runs, 90)),
           "effective_update_hz": rate / float(runs.mean()), "min": lo, "max": hi, "share_at_limits": sat}
    if v.size > 8 * rate and np.std(v) > 0:
        f, p = welch(detrend(v), fs=rate, nperseg=int(4 * rate))
        db = 10 * np.log10(p + 1e-30)
        mid = (f >= 2) & (f <= 8)
        top = f >= 0.8 * (rate / 2)
        out.update({"psd_mid_db": float(np.median(db[mid])), "psd_top_db": float(np.median(db[top])),
                    "psd_drop_db": float(np.median(db[mid]) - np.median(db[top]))})
        out["_psd"] = (f, db)
    return out


def _lag(t_ref, a, b, rate=100.0):
    g = np.arange(t_ref[0], t_ref[1], 1.0 / rate)
    x = np.interp(g, *a)
    y = np.interp(g, *b)
    x, y = x - x.mean(), y - y.mean()
    k = int(rate)
    best = max(range(-k, k + 1), key=lambda s: float(np.dot(x[max(0, s):x.size + min(0, s)],
                                                           y[max(0, -s):y.size - max(0, s)])))
    return best / rate


def run(key):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data = d2.load_data(key)
    path = d2.sessions()[key]["path"]
    raw = read_raw_blocks(path, SIG + AUX)
    res = {"key": key, "serial": d2.sessions()[key]["serial"], "channels": {}, "absent": [n for n in SIG + AUX if n not in raw]}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for name in SIG + AUX:
        if name not in raw:
            continue
        t, v, hdr = raw[name]
        w = _window(t, data["laps"])
        if w.sum() < 100:
            res["channels"][name] = {"header": hdr, "status": "no racing samples"}
            continue
        d = describe(t[w], v[w])
        d["header"] = hdr
        psd = d.pop("_psd", None)
        if name == "sclu_msg_cnt_tx1":
            st = np.diff(v[w])
            vals, cnt = np.unique(st, return_counts=True)
            o = np.argsort(cnt)[::-1][:5]
            d["counter_steps"] = {str(vals[i]): int(cnt[i]) for i in o}
        res["channels"][name] = d
        if psd is not None and name in SIG:
            fam = name.split("_")[0]
            ax = axes[0] if ("yaw" in name or "roll" in name or "pitch" in name) else axes[1]
            ax.plot(psd[0], psd[1], lw=1.0, color=FAMILY_COL.get(fam, INK), alpha=0.85,
                    ls="-" if name.endswith(("_y", "yaw_rate", "YawRate")) else ":", label=name)
    tw = (data["laps"][0]["start_time"], data["laps"][-1]["end_time"])
    lags = {}
    if "log_acc_y" in raw:
        for n in ("sclu_acc_y", "ADR2_acc_y"):
            if n in raw:
                lags[f"{n} vs log_acc_y"] = _lag(tw, raw["log_acc_y"][:2], raw[n][:2])
    if "MRR_EgoYawRate" in raw and "sclu_yaw_rate" in raw:
        lags["MRR_EgoYawRate vs sclu_yaw_rate"] = _lag(tw, raw["sclu_yaw_rate"][:2], raw["MRR_EgoYawRate"][:2])
    res["lags_s"] = lags
    for ax, ttl in zip(axes, ("rate channels", "accelerometers")):
        ax.set_xscale("log")
        ax.set_xlabel("frequency (Hz)", color=INK)
        ax.set_ylabel("PSD (dB, header units)", color=INK)
        ax.set_title(f"{key} (serial {res['serial']}): {ttl}, racing samples, Welch 4 s", fontsize=9, loc="left")
        ax.grid(color=BAND, lw=0.6)
        ax.legend(frameon=False, fontsize=7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(os.path.join(OUT_DIR, f"psd_{key}.png"), dpi=110)
    plt.close(fig)
    with open(os.path.join(OUT_DIR, f"raw_{key}.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=float)
    print(f"[{key}] serial {res['serial']}; absent {res['absent']}; lags {lags}")
    for n, d in res["channels"].items():
        if "rate_hz" not in d:
            print(f"  {n}: {d}")
            continue
        print(f"  {n:20s} {d['header']:28s} rate {d['rate_hz']:6.1f} Hz, step {d['step']:.6g}, distinct {d['distinct']}, "
              f"hold {d['hold_share']:.3f} (eff. {d['effective_update_hz']:.1f} Hz), range [{d['min']:.4g}, {d['max']:.4g}] "
              f"at-limit {d['share_at_limits']:.4f}"
              + (f", PSD 2-8 Hz {d['psd_mid_db']:.1f} dB, top {d['psd_top_db']:.1f} dB, drop {d['psd_drop_db']:.1f} dB"
                 if "psd_drop_db" in d else "")
              + (f", counter steps {d['counter_steps']}" if "counter_steps" in d else ""), flush=True)


if __name__ == "__main__":
    for k in (sys.argv[1:] or KEYS):
        run(k)
