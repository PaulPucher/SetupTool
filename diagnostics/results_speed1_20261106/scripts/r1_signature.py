# Root-cause check of R1 with the ruling's detector (larger FRONT ABS speed):
# flagged samples (native 50 Hz, moving, limiter off) without the signature
# |ecu_speed_rr - abs_speed_rr| > 0.5 m/s at the same sample / +-1 sample.
import sys, json, copy
sys.path.insert(0, r"C:\Paul\UNI\Bachelorarbeit\Setuptool_local")
import numpy as np
from scipy.ndimage import maximum_filter1d
import modules.csv_parser as cp
from modules.stability_analysis import load_parameters
orig = cp.load_channels_config
def extra():
    c = orig()
    for n in ("ecu_speed_rr", "abs_speed_rr", "abs_speed_rl"):
        c["channels"][n] = {"label": n, "unit": "km/h", "range": [0, 400]}
    return c
cp.load_channels_config = extra
p = load_parameters()
for k in ("speed_reference", "sensor_offset_removal"):
    p["signal_corrections"][k]["enabled"] = False
def run(key):
    d = cp.parse_csv(rf"H:\dani_try 100\{key}.txt")
    ch = d["channels"]
    t = ch["ecu_speed"]["time"]; v = ch["ecu_speed"]["data"] / 3.6
    def mps(n):
        c = ch[n]; f = 1.609344 if cp._canonical_unit(c["unit_raw"]) == "mph" else 1.0
        return np.interp(t, c["time"], c["data"] * f / 3.6)
    fl, fr, rl, rr = (mps(f"abs_speed_{w}") for w in ("fl", "fr", "rl", "rr"))
    err = np.interp(t, ch["ecu_speed_rr"]["time"], ch["ecu_speed_rr"]["data"] / 3.6)
    lim = np.interp(t, ch["ecu_B_speedlimit_en"]["time"], ch["ecu_B_speedlimit_en"]["data"])
    moving = (v > 5) & (lim < 0.5)
    sig0 = np.abs(err - rr) > 0.5
    sig1 = sig0 | np.r_[sig0[1:], False] | np.r_[False, sig0[:-1]]
    out = {"key": key}
    for name, ref in (("front_max", np.maximum(fl, fr)), ("four_max", np.max([fl, fr, rl, rr], axis=0))):
        for m in (1.0, 3.0):
            f = (v > maximum_filter1d(ref, 11, mode="nearest") + m) & moving
            out[f"{name}_{m}"] = {"flagged": int(f.sum()), "no_sig_same": int((f & ~sig0).sum()),
                                  "no_sig_pm1": int((f & ~sig1).sum())}
    return out
if __name__ == "__main__":
    from concurrent.futures import ProcessPoolExecutor
    keys = [f"NBR{i}" for i in range(1, 6)] + [f"PRC{i}" for i in range(1, 6)]
    with ProcessPoolExecutor(5) as ex:
        res = list(ex.map(run, keys))
    for r in res: print(json.dumps(r))
