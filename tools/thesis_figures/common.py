# Shared helper of the thesis figure scripts in tools/thesis_figures/.
# Read-only on everything the app owns: the DB
# is opened read-only, no cache or sidecar is written, config is only read.
# Sessions are app outing IDs; the analysis is taken either from the stored,
# identity-valid payload (the app's own cache guards) or from a read-only
# re-run of the app's chain, function for function.

import datetime
import hashlib
import json
import os
import re
import sqlite3
import sys
import unicodedata

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
os.chdir(ROOT)   # config and DB paths in modules/ are relative to the repo
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

FIG_CONFIG = os.path.join(ROOT, "config", "thesis_figures.json")
DB = os.path.join(ROOT, "data", "setuptool.db")
# files whose content decides the analysis result; hashed into every CSV
# header (car_data.json is machine-local and protected: only its hash leaves)
HASHED_CONFIG = ("config/parameters.json", "config/channels.json", "config/car_data.json")
# local, git-ignored: weekend -> short code (author-maintained) and the
# label maps that tie neutral labels to outings and drivers
LOCAL_DIR = os.path.join(ROOT, "tools", "thesis_figures", "local")
LABELS_FILE = os.path.join(LOCAL_DIR, "labels.json")

# the thesis symbol (ch. 3, eq:cs_ratio); every script takes it from here
CS_AXIS = "$\\mathrm{CS}$"
CS_F = "$\\mathrm{CS}_\\mathrm{f}$"
CS_R = "$\\mathrm{CS}_\\mathrm{r}$"

# the thesis's phase words (ch. 3); the app's own words (corner_aggregation
# PHASE_WORD_*) differ on purpose and stay as they are
PHASE_LABELS = {
    "entry_1_brake": "braking",
    "entry_2_turnin": "turn in",
    "apex_3": "apex",
    "exit_4": "early exit",
    "exit_5": "exit completion",
}

TRACE_BIN_M = 2.0          # lap-distance grid of the trace figures
SCATTER_CAP = 800          # points per scatter series; pdflatex memory limit
SCATTER_ALPHA_BINS = 60    # strata over the slip angle
THINNING_SEED = 0


class Refused(SystemExit):
    """A session the figure must not use; the message names outing and reason."""


def out_dirs(override=None):
    if override:
        base = override
    else:
        with open(FIG_CONFIG, "r", encoding="utf-8") as f:
            base = json.load(f)["thesis_figures_dir"]
    data, plots = os.path.join(base, "data"), os.path.join(base, "plots")
    os.makedirs(data, exist_ok=True)
    os.makedirs(plots, exist_ok=True)
    # the .tex files are input from the thesis root, which holds this folder
    return data, plots, os.path.basename(os.path.normpath(base)) + "/data/"


def config_hash():
    h = hashlib.sha256()
    for rel in HASHED_CONFIG:
        p = os.path.join(ROOT, rel)
        h.update(rel.encode())
        h.update(open(p, "rb").read() if os.path.exists(p) else b"<absent>")
    return h.hexdigest()[:16]


def write_csv(path, columns, meta):
    """columns: {name: 1-D array or list}; numbers as %.6g, NaN as nan
    (pgfplots jumps there with unbounded coords=jump). Fields are quoted
    only where needed (a comma in a reason text), so a file without one is
    written exactly as a plain join would. The text passes the name guard
    before anything is written."""
    import csv
    import io
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
    names = list(columns)
    n = len(columns[names[0]])
    head = {"schema_version": ANALYSIS_SCHEMA_VERSION, **meta,
            "date": datetime.datetime.now().isoformat(timespec="seconds"), "config_hash": config_hash()}
    buf = io.StringIO()
    buf.write("".join(f"# {k}: {v}\n" for k, v in head.items()))
    writer = csv.writer(buf, lineterminator="\n")
    writer.writerow(names)
    for i in range(n):
        row = []
        for c in names:
            v = columns[c][i]
            if isinstance(v, (float, np.floating)):
                row.append("nan" if not np.isfinite(v) else f"{v:.6g}")
            else:
                row.append(str(v))
        writer.writerow(row)
    _write_guarded(path, buf.getvalue())


def write_tex(path, body):
    _write_guarded(path, body)


def _write_guarded(path, text):
    guard(text, os.path.basename(path))
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _ascii_variants(s):
    # a name may reach a file in any spelling: as stored, without accents,
    # with the German umlaut transcription
    base = unicodedata.normalize("NFKD", s)
    plain = "".join(ch for ch in base if not unicodedata.combining(ch))
    umlaut = s.translate(str.maketrans({"\u00e4": "ae", "\u00f6": "oe", "\u00fc": "ue", "\u00c4": "Ae",
                                        "\u00d6": "Oe", "\u00dc": "Ue", "\u00df": "ss"}))
    return {s, plain, umlaut}


def load_labels():
    if not os.path.exists(LABELS_FILE):
        raise Refused(f"no {os.path.relpath(LABELS_FILE, ROOT)} -- create it: {{\"<weekend name>\": \"<code>\"}}")
    with open(LABELS_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


_GUARD = None


def guard_tokens():
    """Every driver name (full, first, surname, ASCII spellings), every
    outing name as a whole string, any 4+ letter word of an outing name that
    starts a driver's name (nicknames), and the author's extra tokens
    (labels.json "_guard_tokens", e.g. driver codes in the file names)."""
    global _GUARD
    if _GUARD is not None:
        return _GUARD
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    drivers = [r[0] for r in con.execute("SELECT name FROM drivers") if r[0]]
    outings = [r[0].strip() for r in con.execute("SELECT name FROM outings") if r[0] and r[0].strip()]
    con.close()
    tok, parts = set(), set()
    for d in drivers:
        for v in _ascii_variants(d.strip()):
            tok.add(v)
            for w in re.findall(r"\w+", v):
                if len(w) >= 3:
                    tok.add(w)
                    parts.add(w.lower())
    for o in outings:
        tok |= _ascii_variants(o)
        for w in re.findall(r"\w+", o):
            if len(w) >= 4 and any(p.startswith(w.lower()) for p in parts):
                tok |= _ascii_variants(w)
    tok |= set(load_labels().get("_guard_tokens", []))
    _GUARD = sorted(tok, key=len, reverse=True)
    return _GUARD


def guard_hits(text):
    return [t for t in guard_tokens()
            if re.search(r"(?<![A-Za-z0-9])" + re.escape(t) + r"(?![A-Za-z0-9])", text, re.IGNORECASE)]


def guard(text, fname):
    # anonymisation: no driver or outing name in any
    # written file; the file is named, the matched name is not repeated
    if guard_hits(text):
        raise Refused(f"name guard: {fname} would contain a driver or outing name -- not written")


def session_label(row):
    """'<code> <n>': the weekend's code from local/labels.json and the DB
    outing number within the weekend; independent of the selected set."""
    labels = load_labels()
    code = labels.get(row["weekend"])
    if not code or str(row["weekend"]).startswith("_"):
        raise Refused(f"weekend {row['weekend']!r} has no code in local/labels.json")
    guard(code, "local/labels.json code")
    return f"{code} {row['number']}"


def driver_codes():
    """{driver_id: 'Dn'} by first appearance in DB order (weekend, outing number)."""
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    order = [r[0] for r in con.execute("SELECT o.driver_id FROM outings o JOIN race_weekends w ON o.race_weekend_id "
                                       "= w.id ORDER BY w.id, o.number, o.id") if r[0] is not None]
    con.close()
    codes = {}
    for d in order:
        codes.setdefault(d, f"D{len(codes) + 1}")
    return codes


def write_label_map(tag, outing_ids):
    """The only place neutral labels meet names: local, git-ignored."""
    os.makedirs(LOCAL_DIR, exist_ok=True)
    codes = driver_codes()
    lines = [f"# label map, run tag {tag}, {datetime.datetime.now().isoformat(timespec='seconds')} -- local only"]
    for o in outing_ids:
        row = db_outing(o)
        lines.append(f"outing {o} | {row['weekend']} | {row['name']} -> {session_label(row)} | "
                     f"driver {row['driver_name']} -> {codes.get(row['driver_id'], '-')}")
    with open(os.path.join(LOCAL_DIR, f"label_map_{tag}.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def table_read(prefix, fname, macro):
    # comment chars={\#}: the bare # of the spec breaks inside a TeX argument
    return f"\\pgfplotstableread[col sep=comma, comment chars={{\\#}}]{{{prefix}{fname}}}{{\\{macro}}}\n"


def db_outing(outing_id):
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    row = con.execute(
        "SELECT o.*, w.track AS weekend, d.name AS driver_name FROM outings o "
        "JOIN race_weekends w ON o.race_weekend_id = w.id LEFT JOIN drivers d ON o.driver_id = d.id "
        "WHERE o.id = ?", (outing_id,)).fetchone()
    con.close()
    if row is None:
        raise Refused(f"outing {outing_id}: not in the app DB")
    return dict(row)


def _loads(s):
    try:
        return json.loads(s) if s else None
    except (TypeError, ValueError):
        return None


def load_session(outing_id, allow_no_grid=False):
    """Outing row, resolved file, parsed data (speed classes from the current
    boundaries, as the app does on load) and the app's analysis identity.
    A file the grid-rate check refuses (no usable fast channel, or below the
    rate floor) is refused here naming the outing; allow_no_grid keeps the
    session with grid_rate_hz None and the message in ident["grid_error"],
    for a table that lists it as not analysable."""
    from modules.accuracy_resolution import apply_resolved_vehicle, resolve_accuracy
    from modules.corner_analysis import reclassify_speed_classes
    from modules.csv_parser import load_channels_config, parse_csv
    from modules.data_paths import resolve_saved_path
    from modules.stability_analysis import _resolve_grid_rate, load_parameters
    row = db_outing(outing_id)
    path = resolve_saved_path(row["csv_path"])
    if path is None:
        raise Refused(f"outing {outing_id}: data file not found under the data roots ({row['csv_path']!r})")
    parsed = parse_csv(path)
    reclassify_speed_classes(parsed.get("corners", []), load_channels_config()["corner_speed_thresholds"])
    raw = load_parameters()
    setup = _loads(row["setup_data"])
    resolved = resolve_accuracy(raw, setup, None)   # best available, as the app
    laps = parsed["laps"]
    valid = sorted(l["lap_number"] for l in laps if l.get("is_valid_for_analysis"))
    try:
        grid_rate_hz, grid_error = _resolve_grid_rate(parsed["channels"], raw)[0], None
    except ValueError as e:
        if not allow_no_grid:
            raise Refused(f"outing {outing_id}: {e}")
        grid_rate_hz, grid_error = None, str(e)
    ident = {"cap": None, "resolved": resolved,
             "sideslip_source": raw["stability_estimation"].get("sideslip_source", "kinematic"),
             "grid_rate_hz": grid_rate_hz, "grid_error": grid_error,
             "lap_filter": valid or sorted(l["lap_number"] for l in laps)}
    return {"row": row, "path": path, "parsed": parsed, "params": apply_resolved_vehicle(raw, resolved),
            "ident": ident}


def stored_payload(session):
    """(payload, None) when the DB payload passes the app's cache guards
    (OutingForm._try_render_cached_analysis, in order), else (None, reason)."""
    from modules.data_paths import path_key
    from modules.stability_analysis import ANALYSIS_SCHEMA_VERSION
    row, ident = session["row"], session["ident"]
    cached = _loads(row["analysis_data"])
    if cached is None:
        return None, "no stored analysis"
    checks = [
        ("schema_version", cached.get("schema_version") == ANALYSIS_SCHEMA_VERSION,
         f"stored schema {cached.get('schema_version')}, current {ANALYSIS_SCHEMA_VERSION}"),
        ("csv_path", path_key(cached.get("csv_path")) == path_key(session["path"]), "another data file"),
        ("accuracy_cap", cached.get("accuracy_cap") == ident["cap"], "accuracy cap"),
        ("resolved_vehicle_snapshot", cached.get("resolved_vehicle_snapshot") == ident["resolved"]["values"],
         "vehicle values changed"),
        ("estimator_settings", cached.get("estimator_settings") == ident["resolved"]["estimator_settings"],
         "estimator settings changed"),
        ("sideslip_source", cached.get("sideslip_source") == ident["sideslip_source"], "sideslip source"),
        ("grid_rate_hz", cached.get("grid_rate_hz") == ident["grid_rate_hz"], "grid rate"),
        ("summaries", bool(cached.get("summaries")), "no summaries"),
        ("lap_filter", sorted(cached.get("lap_filter") or []) == sorted(ident["lap_filter"] or []), "lap filter"),
    ]
    for name, ok, why in checks:
        if not ok:
            return None, f"stale stored analysis ({name}: {why}) -- re-analyse the outing in the app"
    return cached, None


def exclusion_reason(fit_manifest):
    # a rejected damper path (static split, Fz Level 1) keeps the session out
    # of the thesis evaluation
    plaus = ((fit_manifest or {}).get("axle_load") or {}).get("damper_plausibility")
    if plaus and not plaus.get("passed", True):
        return f"excluded from the thesis evaluation: {plaus.get('reason')}"
    return None


def gate(outing_id, fit_manifest, fallback_used, allow_status, allow_excluded):
    """Refuse an excluded, degenerate or fallback session unless allowed;
    else return the marks for the CSV header and the legend (empty for a
    clean session)."""
    marks = []
    excl = exclusion_reason(fit_manifest)
    if excl:
        if not allow_excluded:
            raise Refused(f"outing {outing_id}: {excl} (--allow-excluded to plot it marked)")
        marks.append("excluded from the evaluation")
    status = (fit_manifest or {}).get("status")
    if fallback_used or status == "degenerate":
        if not allow_status:
            raise Refused(f"outing {outing_id}: fit status {status!r}, kinematic fallback {fallback_used} "
                          "(--allow-status to plot it marked)")
        marks.append(f"fit {status}, kinematic fallback" if fallback_used else f"fit {status}")
    return marks


def run_chain(session):
    """StabilityAnalysisThread.run, full-pipeline branch, function for
    function (floor events left out: display only, no input to anything
    plotted). Nothing is persisted."""
    from modules.stability_analysis import (
        estimate_cornering_stiffness, estimate_lateral_forces, estimate_sideslip, estimate_slip_angles,
        estimate_vertical_loads, estimate_yaw_moment_stability, load_car_data, prepare_vehicle_state,
        summarise_corners)
    from modules.tyre_fit_auto import resolve_sideslip_beta_and_cs_inputs
    parsed, p, ident = session["parsed"], session["params"], session["ident"]
    state = prepare_vehicle_state(parsed["channels"], p)
    if state is None:
        raise Refused(f"outing {session['row']['id']}: required channels missing or failed")
    beta, man, gate_v, fb_used, fb_reason, cs_in = resolve_sideslip_beta_and_cs_inputs(
        state, p, parsed, ident["sideslip_source"], csv_path=session["path"])
    laps = parsed.get("laps", [])
    slip = estimate_slip_angles(state, beta, p)
    forces = estimate_lateral_forces(state, p)
    cs = estimate_cornering_stiffness(slip, forces, state, p, laps, **cs_in["cs_kwargs"])
    stab = estimate_yaw_moment_stability(state, beta, p, laps)
    fz = estimate_vertical_loads(state, forces, p, channels=parsed["channels"], car_data=load_car_data())
    corners = parsed.get("corners", [])
    summaries = summarise_corners(corners, cs, stab, state, fz=fz, lap_filter=ident["lap_filter"])
    return {"state": state, "beta": beta, "beta_kin": estimate_sideslip(state, p), "slip": slip,
            "forces": forces, "cs": cs, "stab": stab, "summaries": summaries, "corners": corners,
            "fit_manifest": man, "gate_verdict": gate_v, "fallback_used": fb_used, "fallback_reason": fb_reason,
            # hard-bound re-entries of the shipped observer; None on kinematic beta
            "reset_mask": cs_in["cs_kwargs"].get("reset_mask")}


class _StopFit(Exception):
    pass


def capture_fit(session):
    """Fit L up to the two axle fits, with the inputs the production code
    hands them; stops before the observer. Wraps module functions for this
    call only: the fit mask (_base_mask & _speed_window_mask), each axle's
    population after the braking filter and its fit (_fit_axle_l), and the
    load inputs (_axle_load_inputs, for the damper-path check)."""
    from modules import tyre_fit_auto as tfa
    got = {"fits": []}
    orig = {k: getattr(tfa, k) for k in ("_base_mask", "_speed_window_mask", "_axle_load_inputs", "_fit_axle_l")}

    def base_mask(*a, **k):
        got["base"] = orig["_base_mask"](*a, **k)
        return got["base"]

    def speed_window(*a, **k):
        r = orig["_speed_window_mask"](*a, **k)
        got["window"] = r[0]
        return r

    def load_inputs(*a, **k):
        got["load_in"] = orig["_axle_load_inputs"](*a, **k)
        return got["load_in"]

    def fit_axle(alpha, Fy, Fz, base_mask_, bin_rad, mu_pct, weighted=True):
        r = orig["_fit_axle_l"](alpha, Fy, Fz, base_mask_, bin_rad, mu_pct, weighted)
        got["fits"].append({"alpha": alpha, "Fy": Fy, "Fz": Fz, "axle_mask": base_mask_, "fit": r})
        if len(got["fits"]) == 2:
            raise _StopFit
        return r
    tfa._base_mask, tfa._speed_window_mask = base_mask, speed_window
    tfa._axle_load_inputs, tfa._fit_axle_l = load_inputs, fit_axle
    try:
        tfa.fit_session_pacejka(session["parsed"], session["params"], data_file_path=session["path"])
    except _StopFit:
        pass
    finally:
        for k, v in orig.items():
            setattr(tfa, k, v)
    if len(got["fits"]) != 2:
        raise Refused(f"outing {session['row']['id']}: fit L stopped before the axle fits "
                      "(degenerate input, see the app's fit record)")
    got["fit_mask"] = got["base"] & got["window"]
    return got


def lines():
    from modules.stability_analysis import load_parameters
    from modules.tyre_fit_auto import L_K_BAND, L_K_EDGE, L_MU_CEILING, MU_EDGE_CONFIG_KEY
    from modules.classifier import cs_decision_lines
    p = load_parameters()
    return {
        # CS levels straight from the classifier, as the app's trace views
        **cs_decision_lines(),
        "k_band": tuple(L_K_BAND), "k_edge": tuple(L_K_EDGE),
        "mu_ceiling": L_MU_CEILING,
        # the gate's own edge: relative uncertainty on the ceiling
        "mu_edge": L_MU_CEILING * (1.0 + p["tyre_fit_auto"][MU_EDGE_CONFIG_KEY]),
        "cs_filter_cutoff_hz": p["stability_estimation"]["cs_filter_cutoff_hz"],
    }


def bin_trace(s, x, excluded, lo, hi, step=TRACE_BIN_M):
    """Bin means on a lap-distance grid; a bin holding any excluded sample is
    nan, so no value is carried across an exclusion. NaN samples that are
    merely missing (no CS value) are skipped; an all-missing bin is nan."""
    edges = np.arange(lo, hi + step, step)
    centres = 0.5 * (edges[:-1] + edges[1:])
    idx = np.digitize(s, edges) - 1
    out = np.full(len(centres), np.nan)
    for b in range(len(centres)):
        m = idx == b
        if not m.any() or excluded[m].any():
            continue
        v = x[m][np.isfinite(x[m])]
        if v.size:
            out[b] = float(v.mean())
    return centres, out


def thin_stratified(alpha, y, cap=SCATTER_CAP, nbins=SCATTER_ALPHA_BINS, seed=THINNING_SEED):
    """Indices of at most cap points, stratified over alpha: equal-width
    strata, sparse strata kept whole (the high-slip tail), the rest drawn
    at random (fixed seed) with the unused quota passed on."""
    ok = np.flatnonzero(np.isfinite(alpha) & np.isfinite(y))
    if ok.size <= cap:
        return ok
    edges = np.linspace(alpha[ok].min(), alpha[ok].max(), nbins + 1)
    strata = [ok[(np.digitize(alpha[ok], edges[1:-1]) == b)] for b in range(nbins)]
    rng = np.random.default_rng(seed)
    keep, left = [], cap
    for k, st in enumerate(sorted(strata, key=len)):
        quota = left // (nbins - k)
        take = st if st.size <= quota else rng.choice(st, quota, replace=False)
        keep.append(take)
        left -= take.size
    return np.sort(np.concatenate(keep))


def meta(script, outing_ids, tag, marks=None, **extra):
    m = {"script": script, "run_tag": tag, "outing_ids": ",".join(str(o) for o in outing_ids)}
    if marks:
        m["marks"] = "; ".join(marks)
    m.update(extra)
    return m


def tex_escape(s):
    s = str(s)
    for a, b in (("\\", r"\textbackslash{}"), ("&", r"\&"), ("%", r"\%"), ("$", r"\$"), ("#", r"\#"),
                 ("_", r"\_"), ("{", r"\{"), ("}", r"\}"), ("~", r"\textasciitilde{}"), ("^", r"\^{}")):
        s = s.replace(a, b)
    return s


def write_table(path, script, header, rows, colspec, tabcolsep=None):
    """The tabularx environment with a booktabs body; the chapter holds the
    table float, caption and label. The environment has to be in the file:
    \\input is not expandable in current LaTeX, so a body \\input inside the
    chapter's tabularx breaks at the first \\toprule (Misplaced \\noalign).
    header / rows are already TeX (escape plain text with tex_escape)."""
    t = [f"% generated by tools/thesis_figures/{script}.py -- do not edit; re-run the script"]
    # a narrower column gap stays local to this table (group)
    t += [f"{{\\setlength{{\\tabcolsep}}{{{tabcolsep}}}%"] if tabcolsep else []
    t += [f"\\begin{{tabularx}}{{\\textwidth}}{{{colspec}}}",
          r"\toprule", " & ".join(header) + r" \\", r"\midrule"]
    t += [" & ".join(r) + r" \\" for r in rows]
    t += [r"\bottomrule", r"\end{tabularx}"]
    t += ["}"] if tabcolsep else []
    write_tex(path, "\n".join(t) + "\n")


# marginal_reason fragments as tyre_fit_auto writes them -> thesis words;
# "r_sweep_" covers both selection codes (no setting in band / guard failed),
# after which the default R is used
_MARGINAL_WORDS = (("edge of the literature range", "K at the band edge"),
                   ("above the ceiling 2.0 within its uncertainty", "mu in the edge zone"),
                   ("sign check", "sign check"), ("yaw_mismatch", "yaw mismatch"), ("r_sweep_", "default R"))


def estimation_path(fit_manifest, fallback_used, fallback_reason):
    """(path, reason) in plain words, as the thesis states it: excluded
    (damper loads rejected) before kinematic fallback before observer. The
    reason only explains a path other than the observer; the fit's own
    status is a separate statement (fit_status), since an observer session
    can run on a marginal fit."""
    man = fit_manifest or {}
    excl = ((man.get("axle_load") or {}).get("damper_plausibility") or {})
    if excl and not excl.get("passed", True):
        return "excluded", f"damper loads rejected, straight-line total {excl['ratio']:.2f} x static weight"
    if fallback_used:
        r = fallback_reason or ""
        why = ("observer self-check failed" if r.startswith("NIS gate")
               else "tyre-curve fit not plausible" if "plausibility gate" in r
               else "tyre-curve fit degenerate")
        return "kinematic fallback", why
    return "observer", ""


def fit_status(fit_manifest):
    """(status, detail) of the tyre-curve fit: ok / marginal / degenerate,
    "-" without a fit (kinematic source); detail names the marginal reasons."""
    man = fit_manifest or {}
    status = man.get("status") or "-"
    if status != "marginal":
        return status, ""
    mr = man.get("marginal_reason") or ""
    words = [w for k, w in _MARGINAL_WORDS if k in mr] or ["reason not mapped"]
    return status, ", ".join(dict.fromkeys(words))


_SYMPTOM_CODE = {"US": "U", "OS": "O", "INST": "I"}


def verdict(agg):
    """The classifier's corner verdict restated with the thesis phase words:
    classify_corner decides (severity, state, the MARGINAL mark); its own
    primary rule (rear before front, each at its worst phase, plus the worst
    open-loop-unstable phase) supplies the phase keys, because its short
    text writes both exit phases as "exit". Returns (text, code, state)."""
    from modules.classifier import SYMPTOM_WORD, _worst, classify_corner, phase_symptoms
    _sev, short, _long, state = classify_corner(agg)
    syms = phase_symptoms(agg)["symptoms"]
    primary = _worst(syms, "OS") or _worst(syms, "US")
    inst = _worst(syms, "INST")
    parts, code = [], ""
    if primary is not None:
        marginal = "[MARGINAL]" in short.split("|")[0]
        parts.append(f"{SYMPTOM_WORD[primary['symptom']]} ({primary['severity']}) at "
                     f"{PHASE_LABELS[primary['phase']]}" + (", marginal" if marginal else ""))
        code += _SYMPTOM_CODE[primary["symptom"]] + ("*" if marginal else "")
    if inst is not None:
        parts.append(f"open-loop unstable at {PHASE_LABELS[inst['phase']]}")
        code += "I"
    if not parts:
        parts, code = (["no CS data"], "n") if state == "no_data" else (["ok"], "-")
    return "; ".join(parts), code, state


def corner_brackets(summaries):
    """{stable_corner_id: (start_m, end_m)}: median bracket over the laps."""
    by = {}
    for s in summaries:
        by.setdefault(s["stable_corner_id"], []).append((s["bracket_start_m"], s["bracket_end_m"]))
    return {cid: (float(np.median([a for a, _ in v])), float(np.median([b for _, b in v]))) for cid, v in by.items()}


MATCH_MIN_OVERLAP = 0.5   # of the shorter bracket


def match_corners(ref, other):
    """One-to-one match of another session's corners to the reference by
    lap distance (stable_corner_id is per session): pairs whose brackets
    overlap by at least MATCH_MIN_OVERLAP of the shorter one, greedy by
    overlap. Returns {other_cid: (ref_cid, overlap)}."""
    pairs = []
    for rc, (r0, r1) in ref.items():
        for oc, (o0, o1) in other.items():
            ov = max(0.0, min(r1, o1) - max(r0, o0)) / max(1e-9, min(r1 - r0, o1 - o0))
            if ov >= MATCH_MIN_OVERLAP:
                pairs.append((ov, rc, oc))
    out, used = {}, set()
    for ov, rc, oc in sorted(pairs, reverse=True):
        if oc not in out and rc not in used:
            out[oc] = (rc, ov)
            used.add(rc)
    return out


def fastest_lap(laps, analysis_laps):
    from modules.csv_parser import _effective_lap_time
    cand = [l for l in laps if l["lap_number"] in set(analysis_laps)]
    return min(cand, key=_effective_lap_time)


def count_rising(mask):
    if mask is None:
        return None
    m = np.asarray(mask, bool).astype(int)
    return int(m[0] + np.sum(np.diff(m) == 1))
