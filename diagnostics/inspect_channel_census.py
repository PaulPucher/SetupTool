# Q-CHANNEL-AUDIT (PLAN.md STATUS 2026-10-06, author work order, EXECUTE
# granted; read-only). Full channel census of every Pi Toolbox text export on
# disk -- no name filtering, every channel of every file -- plus the channel
# names found in the raw Pi .pds files (names only: their sample layout is
# proprietary).
#
# Fingerprint per file and channel (one {ChannelBlock} = Time + N columns;
# N = 1 in narrow exports): unit string verbatim from the header; n = present
# cells (non-empty); rate = (n - 1) / (t_last - t_first) [Hz]; min / max over
# finite values; frac_repeat = share of present samples equal to the previous
# present sample (stuck / held values); frac_zero; frac_nan = share of present
# cells that are explicit NaN tokens ("nan", "-nan(ind)"). constant = finite
# min == max.
# Byte-identical files (size + md5 of first and last 16 MB) are fingerprinted
# once and listed with every path.
#
# Modes: exports (census of the text exports) | pds (name census of .pds)
# Output: diagnostics/channel_census/

import hashlib
import json
import os
import re
import sys
import time
from multiprocessing import Pool

import numpy as np
import pandas as pd

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "channel_census")
ROOTS = ["C:/UNI", "C:/Users/puche/Downloads", "C:/Users/puche/OneDrive", "C:/Users/puche/Desktop",
         "C:/Users/puche/Documents", "D:/"]
SKIP_DIRS = (".venv", "node_modules", "scratchpad", "$RECYCLE.BIN", "System Volume Information")
NAN_TOKENS = {"nan", "-nan", "nan(ind)", "-nan(ind)", "inf", "-inf", "1.#INF", "-1.#IND", "1.#QNAN"}
CHUNK_ROWS = 4000
HASH_BYTES = 16 * 1024 * 1024


def _walk(ext):
    out = []
    for root in ROOTS:
        for dp, dns, fns in os.walk(root):
            dns[:] = [d for d in dns if d not in SKIP_DIRS]
            for fn in fns:
                if fn.lower().endswith(ext):
                    p = os.path.join(dp, fn)
                    try:
                        if os.path.getsize(p) > 500_000:
                            out.append(p)
                    except OSError:
                        pass
    return sorted(out)


def _is_pi_export(path):
    with open(path, "rb") as fh:
        head = fh.read(4_000_000)
    return b"{ChannelBlock}" in head or b"{OutingInformation}" in head


def _fingerprint_id(path):
    size = os.path.getsize(path)
    h = hashlib.md5()
    with open(path, "rb") as fh:
        h.update(fh.read(HASH_BYTES))
        fh.seek(max(0, size - HASH_BYTES))
        h.update(fh.read(HASH_BYTES))
    return f"{size}-{h.hexdigest()[:12]}"


def _split_name_unit(raw):
    raw = raw.strip()
    if "[" in raw and raw.endswith("]"):
        return raw[:raw.index("[")].strip(), raw[raw.index("[") + 1:-1]
    return raw, None


def _index_blocks(path):
    """Metadata dict and [(header_offset, n_rows)] per {ChannelBlock}."""
    meta, blocks = {}, []
    with open(path, "rb") as fh:
        section, cur, n_rows = None, None, 0
        while True:
            off = fh.tell()
            line = fh.readline()
            if not line:
                break
            s = line.strip()
            if s.startswith(b"{"):
                if cur is not None:
                    blocks.append((cur, n_rows))
                    cur = None
                section = s
                if s == b"{ChannelBlock}":
                    cur, n_rows = fh.tell(), -1  # header line follows, not a row
                continue
            if section == b"{OutingInformation}":
                parts = line.decode("latin-1").strip().split("\t")
                if len(parts) == 2:
                    meta[parts[0].strip()] = parts[1].strip()
            elif cur is not None and s:
                n_rows += 1
        if cur is not None:
            blocks.append((cur, n_rows))
    return meta, blocks


class _Acc:
    __slots__ = ("unit", "n", "n_nan", "n_zero", "n_rep", "vmin", "vmax", "t0", "t1", "last")

    def __init__(self, unit):
        self.unit, self.n, self.n_nan, self.n_zero, self.n_rep = unit, 0, 0, 0, 0
        self.vmin, self.vmax, self.t0, self.t1, self.last = np.inf, -np.inf, None, None, None

    def add(self, t, raw):
        present = raw.notna().to_numpy()
        if not present.any():
            return
        vals = raw[present]
        tt = t[present]
        if vals.dtype == object:
            low = vals.astype(str).str.strip().str.lower()
            self.n_nan += int(low.isin(NAN_TOKENS).sum())
            num = pd.to_numeric(vals.astype(str).str.replace(",", ".", regex=False), errors="coerce").to_numpy(float)
        else:
            num = vals.to_numpy(float)
            self.n_nan += int(np.isnan(num).sum())
        self.n += len(num)
        fin = np.isfinite(num)
        if fin.any():
            self.vmin = min(self.vmin, float(num[fin].min()))
            self.vmax = max(self.vmax, float(num[fin].max()))
            self.n_zero += int((num[fin] == 0).sum())
        prev = np.concatenate(([self.last if self.last is not None else np.nan], num[:-1]))
        self.n_rep += int(np.sum(num == prev))
        self.last = num[-1]
        tt = tt.to_numpy(float)
        if self.t0 is None:
            self.t0 = float(tt[0])
        self.t1 = float(tt[-1])

    def row(self):
        rate = (self.n - 1) / (self.t1 - self.t0) if (self.n > 1 and self.t1 and self.t1 > self.t0) else float("nan")
        fin = np.isfinite(self.vmin)
        return {"unit": self.unit, "n": self.n, "rate_hz": rate,
                "min": self.vmin if fin else float("nan"), "max": self.vmax if fin else float("nan"),
                "frac_repeat": self.n_rep / max(1, self.n - 1), "frac_zero": self.n_zero / max(1, self.n),
                "frac_nan": self.n_nan / max(1, self.n), "constant": bool(fin and self.vmin == self.vmax)}


def census_file(path):
    t_start = time.time()
    meta, blocks = _index_blocks(path)
    acc = {}
    with open(path, "r", encoding="latin-1", newline="") as fh:
        for off, n_rows in blocks:
            fh.seek(off)
            header = fh.readline().rstrip("\r\n").split("\t")
            names = [_split_name_unit(h) for h in header]
            cols = [n for n, _u in names]
            if n_rows <= 0:
                for name, unit in names[1:]:
                    acc.setdefault(name, _Acc(unit))
                continue
            reader = pd.read_csv(fh, sep="\t", header=None, names=cols, nrows=n_rows, chunksize=CHUNK_ROWS,
                                 decimal=",", keep_default_na=False, na_values=[""], dtype=None,
                                 low_memory=False, engine="c", index_col=False)
            for chunk in reader:
                t = pd.to_numeric(chunk.iloc[:, 0], errors="coerce")
                ok = t.notna()
                chunk, t = chunk[ok], t[ok]
                for j, (name, unit) in enumerate(names[1:], start=1):
                    a = acc.get(name)
                    if a is None:
                        a = acc[name] = _Acc(unit)
                    a.add(t, chunk.iloc[:, j])
    return {"path": path, "meta": meta, "n_blocks": len(blocks), "seconds": round(time.time() - t_start, 1),
            "channels": {k: v.row() for k, v in acc.items()}}


def run_exports(workers):
    os.makedirs(OUT_DIR, exist_ok=True)
    cands = _walk(".txt")
    exports = [p for p in cands if _is_pi_export(p)]
    groups = {}
    for p in exports:
        groups.setdefault(_fingerprint_id(p), []).append(p)
    todo = [ps[0] for ps in groups.values()]
    print(f"{len(exports)} exports, {len(todo)} unique", flush=True)
    results = {}
    with Pool(workers) as pool:
        for res in pool.imap_unordered(census_file, todo):
            results[res["path"]] = res
            print(f"done {res['path']} ({len(res['channels'])} ch, {res['seconds']} s)", flush=True)
    payload = []
    for fid, paths in sorted(groups.items(), key=lambda kv: kv[1][0]):
        r = results[paths[0]]
        payload.append({"file_id": fid, "paths": paths, **r})
    with open(os.path.join(OUT_DIR, "census_exports.json"), "w", encoding="utf-8") as fh:
        json.dump(payload, fh, default=float)
    print("written census_exports.json", flush=True)


_IDENT = re.compile(r"^[A-Za-z][A-Za-z0-9]*_[A-Za-z0-9_]{2,60}$")


def pds_names(path):
    with open(path, "rb") as fh:
        b = fh.read()
    names = set()
    for m in re.finditer(rb"(?:[\x20-\x7e]\x00){4,}", b):
        for tok in re.split(r"[^A-Za-z0-9_]+", m.group().decode("utf-16-le")):
            if _IDENT.match(tok):
                names.add(tok)
    for m in re.finditer(rb"[\x20-\x7e]{6,}", b):
        for tok in re.split(r"[^A-Za-z0-9_]+", m.group().decode("ascii")):
            if _IDENT.match(tok):
                names.add(tok)
    return path, sorted(names)


def run_pds(workers):
    os.makedirs(OUT_DIR, exist_ok=True)
    files = _walk(".pds")
    out = {}
    with Pool(workers) as pool:
        for path, names in pool.imap_unordered(pds_names, files):
            out[path] = names
            print(f"pds {path}: {len(names)} identifiers", flush=True)
    with open(os.path.join(OUT_DIR, "census_pds_names.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh)
    print("written census_pds_names.json", flush=True)


# Candidate sweep, frozen before reading the merged list (work order step 2):
# names matching NAME_RX, or units in UNIT_SET on a channel not mapped in
# channels.json. Units compared case-insensitively after stripping.
NAME_RX = re.compile(
    r"(^|_)(vy|v_y|vx|v_x|beta|sideslip|slip|drift|course|heading|hdg|track_?angle|attitude|yaw_?ang"
    r"|vel|velo|velocity|speed_?lat|lat_?speed|ground_?speed|gnd|sog|cog|vn|ve|vd|ned|enu|north|east"
    r"|ins|imu|ahrs|kalman|kf|ekf|observer|est|estim|estimated|fusion|correvit|kistler|optic|datron|vbox|rtk)"
    r"(_|$|[0-9])|lat.?vel|vel.?lat|sideslip|slipang|swimm|schwimm|gierw",
    re.IGNORECASE)
UNIT_SET = {"m/s", "mps", "km/h", "kph", "kmh", "mph", "deg", "°", "rad", "°/s", "deg/s", "rad/s"}


def _mapped_names():
    with open(os.path.join(os.path.dirname(OUT_DIR), "..", "config", "channels.json"), encoding="utf-8") as fh:
        return set(json.load(fh)["channels"])


def _fmt(x, nd=4):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "nan"
    return f"{x:.{nd}g}"


def run_report(_workers=None):
    data = json.load(open(os.path.join(OUT_DIR, "census_exports.json"), encoding="utf-8"))
    mapped = _mapped_names()
    files = []
    for i, f in enumerate(data, start=1):
        m = f["meta"]
        files.append({"id": f"F{i:02d}", "paths": f["paths"], "n": len(f["channels"]), "track": m.get("TrackName"),
                      "serial": (re.search(r"Serial Number:\s*(\d+)", m.get("SystemDetails", "")) or [None, None])[1],
                      "date": m.get("Date") or m.get("OutingDate"), "chans": f["channels"]})
    merged = {}
    for f in files:
        for name, r in f["chans"].items():
            merged.setdefault(name, []).append((f["id"], r))
    L = ["CHANNEL CENSUS -- every channel of every Pi Toolbox text export on disk (Q-CHANNEL-AUDIT, 2026-10-06)",
         "Fingerprint definitions: see diagnostics/inspect_channel_census.py header. rate = mean rate over the "
         "channel's own present samples. 'live' = not constant in that file.", "", "FILES"]
    for f in files:
        L.append(f"{f['id']}  track {f['track']}  logger serial {f['serial']}  {f['n']} channels")
        for p in f["paths"]:
            L.append(f"      {p}")
    L += ["", f"MERGED CHANNEL LIST ({len(merged)} distinct names over {len(files)} unique files; "
          f"{sum(f['n'] for f in files)} file-channel entries). Columns: name | units seen | files (live/present) "
          "| rate Hz min-max | value min .. max | frac_repeat max | frac_zero max | frac_nan max | mapped", ""]
    for name in sorted(merged, key=str.lower):
        rows = merged[name]
        rs = [r for _fid, r in rows]
        units = sorted({str(r["unit"]) for r in rs})
        live = [fid for fid, r in rows if not r["constant"] and r["n"] > 1]
        rates = [r["rate_hz"] for r in rs if np.isfinite(r["rate_hz"])]
        mins = [r["min"] for r in rs if np.isfinite(r["min"])]
        maxs = [r["max"] for r in rs if np.isfinite(r["max"])]
        L.append(" | ".join([
            name, ",".join(units), f"{len(live)}/{len(rows)} [{' '.join(fid for fid, _ in rows)}]",
            f"{_fmt(min(rates)) if rates else 'nan'}-{_fmt(max(rates)) if rates else 'nan'}",
            f"{_fmt(min(mins)) if mins else 'nan'} .. {_fmt(max(maxs)) if maxs else 'nan'}",
            _fmt(max(r["frac_repeat"] for r in rs), 3), _fmt(max(r["frac_zero"] for r in rs), 3),
            _fmt(max(r["frac_nan"] for r in rs), 3), "mapped" if name in mapped else "-"]))
    # candidate sweep
    cands = []
    for name, rows in merged.items():
        units = {str(r["unit"]).strip().lower() for _f, r in rows}
        hit_name = bool(NAME_RX.search(name))
        hit_unit = bool(units & UNIT_SET) and name not in mapped
        if hit_name or hit_unit:
            cands.append((name, hit_name, hit_unit, rows))
    L += ["", f"CANDIDATE SWEEP (name rule or unmapped speed/angle unit): {len(cands)} channels", ""]
    for name, hn, hu, rows in sorted(cands, key=lambda c: c[0].lower()):
        live = [(fid, r) for fid, r in rows if not r["constant"] and r["n"] > 1]
        r0 = (live or rows)[0][1]
        L.append(f"{name} | by {'name' if hn else ''}{'+' if hn and hu else ''}{'unit' if hu else ''} | units "
                 f"{','.join(sorted({str(r['unit']) for _f, r in rows}))} | live in {len(live)}/{len(rows)} files | "
                 f"e.g. rate {_fmt(r0['rate_hz'])} Hz, {_fmt(r0['min'])} .. {_fmt(r0['max'])}, repeat "
                 f"{_fmt(r0['frac_repeat'], 3)}, zero {_fmt(r0['frac_zero'], 3)}, nan {_fmt(r0['frac_nan'], 3)}"
                 f"{' | mapped' if name in mapped else ''}")
    # unmapped per prefix family
    fam = {}
    for name in merged:
        if name not in mapped:
            fam.setdefault(name.split("_")[0] if "_" in name else "(no prefix)", []).append(name)
    L += ["", "UNMAPPED CHANNELS PER PREFIX FAMILY (prefix = text before the first underscore)", ""]
    for p in sorted(fam, key=lambda k: (-len(fam[k]), k.lower())):
        L.append(f"[{p}] {len(fam[p])}: " + ", ".join(sorted(fam[p], key=str.lower)))
    with open(os.path.join(OUT_DIR, "census_table.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    json.dump([c[0] for c in cands], open(os.path.join(OUT_DIR, "candidates.json"), "w"))
    print(f"report: {len(files)} files, {len(merged)} names, {len(cands)} candidates", flush=True)


# Content test (work order step 3), frozen before reading candidate values.
# Per live candidate the question is whether it carries information beyond
# the channels production already uses; tests are named per channel below.
CONTENT_FILES = {
    "NBR JST outing 29": "D:/Data_Bachelorthesis/NBR_Sturm_Solution/NBR-73-029-101-JST-26.07.01.1734.txt",
    "Paul Ricard MLA outing 25 (v3)": "GT3_PRC_MLA-v3.txt",
}
CONTENT_CANDIDATES = ["log_a_car", "a_yaw_velocity", "a_yaw_theo", "a_us_car", "sta_angle", "abs_ay1_int.acceleration",
                      "MRR_EgoYawRate", "abs_vVeh_absRef"]
CONTENT_REFS = ["sclu_yaw_rate", "log_acc_y", "ecu_speed", "log_gps_course", "log_asteer"]


def read_channels(path, wanted):
    """{name: (t, values)} for the wanted channels, any block layout."""
    wanted = set(wanted)
    _meta, blocks = _index_blocks(path)
    out = {}
    with open(path, "r", encoding="latin-1", newline="") as fh:
        for off, n_rows in blocks:
            fh.seek(off)
            header = fh.readline().rstrip("\r\n").split("\t")
            names = [_split_name_unit(h)[0] for h in header]
            idx = [i for i, n in enumerate(names) if n in wanted]
            if not idx or n_rows <= 0:
                continue
            df = pd.read_csv(fh, sep="\t", header=None, nrows=n_rows, usecols=[0] + idx, decimal=",",
                             keep_default_na=False, na_values=[""], low_memory=False, index_col=False)
            t = pd.to_numeric(df.iloc[:, 0], errors="coerce").to_numpy(float)
            for k, i in enumerate(idx, start=1):
                v = pd.to_numeric(df.iloc[:, k], errors="coerce").to_numpy(float)
                ok = np.isfinite(t) & np.isfinite(v)
                out[names[i]] = (t[ok], v[ok])
    return out


def _corr_gain(x, y):
    ok = np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 10:
        return float("nan"), float("nan")
    x, y = x[ok], y[ok]
    r = float(np.corrcoef(x, y)[0, 1])
    g = float(np.dot(x - x.mean(), y - y.mean()) / np.dot(x - x.mean(), x - x.mean()))
    return r, g


def run_content(_workers=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from modules.csv_parser import parse_csv
    from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_sideslip
    params = load_parameters()
    L = ["CONTENT TEST of the live candidates (Q-CHANNEL-AUDIT step 3)", ""]
    for label, path in CONTENT_FILES.items():
        ch = read_channels(path, CONTENT_CANDIDATES + CONTENT_REFS)
        parsed = parse_csv(path)
        st = prepare_vehicle_state(parsed["channels"], params)
        beta = estimate_sideslip(st, params)
        t = st["time"]
        mv = st["moving_mask"]
        on = lambda name: np.interp(t, *ch[name]) if name in ch else None
        r_dps = np.degrees(st["yaw_rate_radps"])
        ay = st["ay_mps2"]
        v = st["v_mps"]
        ayv_dps = np.degrees(np.where(mv, ay / np.maximum(v, 1.0), np.nan))
        corner = mv & (np.abs(ay) > 0.6 * 9.81)
        L.append(f"== {label}: present {sorted(k for k in ch if k in CONTENT_CANDIDATES)}")
        res = {}
        if "log_a_car" in ch:
            ta, a = ch["log_a_car"]
            au = np.degrees(np.unwrap(np.radians(a)))
            rate = np.interp(t, ta[1:], np.diff(au) / np.diff(ta))
            rr, gg = _corr_gain(r_dps[mv], rate[mv])
            res["a_car"] = (rr, gg)
            L.append(f"log_a_car: d/dt vs gyro yaw rate (moving): r {rr:.4f}, gain {gg:.4f}; native rate "
                     f"{(len(ta) - 1) / (ta[-1] - ta[0]):.1f} Hz")
            # residual of the derivative against the gyro: pure integration -> ~0
            resid = rate - gg * r_dps
            L.append(f"   d/dt minus gain x yaw rate: sd {np.nanstd(resid[mv]):.3f} deg/s (yaw-rate sd "
                     f"{np.nanstd(r_dps[mv]):.2f})")
            if "log_gps_course" in ch:
                c = np.interp(t, *ch["log_gps_course"])
                a_i = np.interp(t, ta, au)
                for sgn in (+1, -1):
                    d = np.degrees(np.angle(np.exp(1j * np.radians(c - sgn * a_i))))
                    du = np.degrees(np.unwrap(np.radians(d)))
                    L.append(f"   course - ({'+' if sgn > 0 else '-'}a_car): range over session {np.nanmin(du[mv]):.1f}"
                             f" .. {np.nanmax(du[mv]):.1f} deg, sd {np.nanstd(du[mv]):.2f}; straight sd "
                             f"{np.nanstd(du[mv & (np.abs(ay) < 0.5)]):.2f}")
                    hp = du - pd.Series(du).rolling(int(20 * st['sample_rate_hz']), center=True,
                                                    min_periods=10).median().to_numpy()
                    rb, gb = _corr_gain(np.degrees(beta[corner]), hp[corner])
                    L.append(f"      high-passed (20 s median removed) vs kinematic beta in corners: r {rb:.3f},"
                             f" gain {gb:.3f}")
        for name, ref, refname in (("a_yaw_velocity", r_dps, "gyro yaw rate"), ("a_yaw_theo", ayv_dps, "a_y / v"),
                                   ("MRR_EgoYawRate", r_dps, "gyro yaw rate"),
                                   ("abs_ay1_int.acceleration", ay / 9.81, "log_acc_y"),
                                   ("abs_vVeh_absRef", v * 3.6, "ecu_speed"),
                                   ("sta_angle", np.interp(t, *ch["log_asteer"]) if "log_asteer" in ch else None,
                                    "log_asteer")):
            x = on(name)
            if x is None or ref is None:
                continue
            for s in (1, -1):
                rr, gg = _corr_gain(ref[mv], s * x[mv])
                if rr > 0:
                    L.append(f"{name} vs {refname} (moving): r {rr:.4f}, gain {gg:.4f}{' (sign flipped)' if s < 0 else ''}")
                    break
            else:
                L.append(f"{name} vs {refname}: no positive correlation either sign")
        if "a_us_car" in ch:
            x = on("a_us_car")
            d_us = np.degrees(np.abs(st["delta_f_rad"]) - (vp := params["vehicle"])["wheelbase_m"] *
                              np.abs(st["yaw_rate_radps"]) / np.maximum(v, 1.0))
            rr, gg = _corr_gain(d_us[corner], x[corner])
            L.append(f"a_us_car vs |delta_f| - L |r| / v (corners): r {rr:.3f}, gain {gg:.3f}")
        if "a_yaw_velocity" in ch and "a_yaw_theo" in ch:
            att = on("a_yaw_velocity") - on("a_yaw_theo")
            kin = np.degrees(-np.gradient(beta, t))
            for s in (1, -1):
                rr, gg = _corr_gain(kin[corner], s * att[corner])
                L.append(f"a_yaw_velocity - a_yaw_theo vs -(d beta_kin / dt) (corners, sign {s:+d}): r {rr:.3f}, gain {gg:.3f}")
        # one corner-dense lap: the analysis lap with most |a_y| > 0.6 g time
        laps = [l for l in parsed["laps"] if l["is_valid_for_analysis"]]
        best = max(laps, key=lambda l: np.sum(corner[(t >= l["start_time"]) & (t <= l["end_time"])]))
        w = (t >= best["start_time"]) & (t <= best["end_time"])
        s_m = st["s_m"]
        panels = [("Sideslip from the kinematic integration [deg]", np.degrees(beta)),
                  ("Lateral acceleration [g]", ay / 9.81)]
        if "log_a_car" in ch:
            ta, a = ch["log_a_car"]
            au = np.degrees(np.unwrap(np.radians(a)))
            panels.append(("Rate of the logged angle log_a_car vs gyro yaw rate [deg/s]",
                           (np.interp(t, ta[1:], np.diff(au) / np.diff(ta)), r_dps)))
        if "a_yaw_velocity" in ch and "a_yaw_theo" in ch:
            panels.append(("Team math channels: yaw velocity minus theoretical yaw [deg/s] vs minus kinematic "
                           "sideslip rate", (on("a_yaw_velocity") - on("a_yaw_theo"),
                                             np.degrees(-np.gradient(beta, t)))))
        fig, axs = plt.subplots(len(panels), 1, figsize=(12, 2.4 * len(panels)), sharex=True)
        for axx, (title, y) in zip(axs, panels):
            if isinstance(y, tuple):
                axx.plot(s_m[w], y[0][w], lw=0.8, label="candidate channel")
                axx.plot(s_m[w], y[1][w], lw=0.8, label="production reference")
                axx.legend(fontsize=7)
            else:
                axx.plot(s_m[w], y[w], lw=0.8)
            axx.set_title(title, fontsize=9)
        axs[-1].set_xlabel("Lap distance [m]")
        fig.suptitle(f"{label}, lap {best['lap_number']}: candidate channels against the kinematic sideslip "
                     f"and lateral acceleration", fontsize=10)
        fig.tight_layout()
        fn = os.path.join(OUT_DIR, f"content_{'nbr_jst29' if 'NBR' in label else 'pr_v3'}.png")
        fig.savefig(fn, dpi=110)
        plt.close(fig)
        L.append(f"   plot: {fn}")
        L.append("")
    with open(os.path.join(OUT_DIR, "content_test.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L), flush=True)


if __name__ == "__main__":
    mode = sys.argv[1]
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    {"exports": run_exports, "pds": run_pds, "report": run_report, "content": run_content}[mode](workers)
