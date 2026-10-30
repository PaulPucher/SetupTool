# tab:example_verdicts -- one session, one row per corner: corner ID (of
# that session), canonical speed class, corner speed v_min (median over the analysis
# laps of the stored apex_speed: corner_analysis takes the minimum of the
# smoothed ecu_speed from turn-in to corner end, in km/h -- the corner
# minimum, not the speed at the apex instant), worst CS_f and CS_r with their phase
# (worst lap per phase as the classifier aggregates, then the minimum over
# the phases), dMz/dbeta (cross-lap median per phase, the minimum over the
# phases -- the value the open-loop-unstable rule reads) with its phase, and
# the verdict as the classifier states it, with the thesis phase words.
# Stored, identity-valid payload only.
# --appendix: one table per session (tab_verdicts_<label>), each with a line
# below it: the session recommendation of the data layer as lever and
# direction with its net support, else "none", and the alternatives; no
# setup values.
# Run: python -m tools.thesis_figures.tab_example_verdicts --outing 12 [--tag r2] [--allow-status]
#      [--allow-excluded] [--out DIR]
#      python -m tools.thesis_figures.tab_example_verdicts --appendix 17,18,19 [--tag final]
#      ... | --render-only [--appendix-labels "NBR 1,PRC 3"]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "tab_example_verdicts"
APPENDIX = "tab_verdicts"


def appendix_name(label):
    # "PRC 3" -> "tab_verdicts_prc3"
    return f"{APPENDIX}_{label.replace(' ', '').lower()}"


def _worst(agg, getter):
    from modules.corner_aggregation import PHASE_KEYS
    best = (float("nan"), "-")
    for ph in PHASE_KEYS:
        v = getter(agg, ph)
        if v == v and (best[0] != best[0] or v < best[0]):
            best = (float(v), common.PHASE_LABELS[ph])
    return best


# the app's short lever words in the thesis wording (author, 2026-10-30)
_THESIS_LEVER = (("ARB", "anti-roll bar"), ("LS ", "low-speed "), ("TC ", "traction control "))
_THESIS_LEVER_NAME = {"diff": "differential locking", "wing": "rear wing"}   # diff_position: "more locking torque" (setup_parameters.json)
_THESIS_DIRECTION = {"increase": "higher", "decrease": "lower"}


def _option(o):
    # lever and direction only; lever_text would add the step size
    from modules.setup_engine import _DIRECTION_SHORT, lever_name
    name = lever_name(o["levers"])
    name = _THESIS_LEVER_NAME.get(name, name)
    for a, b in _THESIS_LEVER:
        name = name.replace(a, b)
    d = o["direction"]
    return f"{name} {_THESIS_DIRECTION.get(d) or _DIRECTION_SHORT.get(d, d)}"


def recommendation_line(rec):
    """Session recommendation of the data layer in words. A tie or an
    advisory-only result is not a recommendation and reads "none"."""
    o = rec["recommended"]
    if o:
        head = f"{_option(o)} (net support {o['net']:.2f})"
    elif rec["equivalent_options"]:
        head = ("none (tie: " + " / ".join(_option(x) for x in rec["equivalent_options"])
                + f", net support {rec['equivalent_options'][0]['net']:.2f} each)")
    else:
        head = "none"
    alts = ", ".join(_option(x) for x in rec["alternatives"]) or "none"
    return head, alts


def build(outing, tag="dev", allow_status=False, allow_excluded=False, out=None, name=SCRIPT, label_map=True):
    from modules.classifier import verdict_cs_stat
    from modules.corner_aggregation import aggregate_by_corner
    session = common.load_session(outing)
    payload, why = common.stored_payload(session)
    if payload is None:
        raise common.Refused(f"outing {outing}: {why}")
    marks = common.gate(outing, payload["fit_manifest"], payload["fallback_used"], allow_status, allow_excluded)
    label = common.session_label(session["row"])
    if label_map:
        common.write_label_map(tag, [outing])
    summaries = payload["summaries"]
    cols = {k: [] for k in ("corner_id", "speed_class", "apex_speed_kmh", "cs_f_worst", "cs_f_phase", "cs_r_worst",
                            "cs_r_phase", "dmz_dbeta_Nm_per_deg", "dmz_phase", "verdict", "verdict_code")}
    for cid, agg in sorted(aggregate_by_corner(summaries).items(), key=lambda kv: kv[0]):
        laps = [s for s in summaries if s["stable_corner_id"] == cid]
        csf = _worst(agg, lambda a, ph: verdict_cs_stat(a, ph, "f")["median"])
        csr = _worst(agg, lambda a, ph: verdict_cs_stat(a, ph, "r")["median"])
        dmz = _worst(agg, lambda a, ph: a["phases"][ph]["stability_observed_Nm_per_deg"]["median"])
        text, code, _state = common.verdict(agg)
        for k, v in (("corner_id", cid), ("speed_class", agg.get("speed_class")),
                     ("apex_speed_kmh", float(np.median([s["apex_speed"] for s in laps]))),
                     ("cs_f_worst", csf[0]), ("cs_f_phase", csf[1]), ("cs_r_worst", csr[0]), ("cs_r_phase", csr[1]),
                     ("dmz_dbeta_Nm_per_deg", dmz[0]), ("dmz_phase", dmz[1]), ("verdict", text.replace(",", ";")),
                     ("verdict_code", code)):
            cols[k].append(v)
    extra = {}
    if name != SCRIPT:
        rec, alts = recommendation_line(payload["setup_engine"]["data_layer"]["session_recommendation"])
        extra = {"session_recommendation": rec, "alternatives": alts}
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{name}.csv", cols,
                     common.meta(SCRIPT, [outing], tag, marks, session_label=label, **extra,
                                 apex_speed="corner speed v_min in km/h: median over the analysis laps of the "
                                            "stored apex_speed (minimum smoothed ecu_speed from turn-in to corner "
                                            "end; the column name is the stored field's)",
                                 dmz="cross-lap median per phase; minimum over the phases"))
    render(out, name)
    return cols


def build_appendix(outings, tag="dev", allow_status=False, allow_excluded=False, out=None):
    common.write_label_map(tag, outings)
    for o in outings:
        label = common.session_label(common.load_session(o)["row"])
        build(o, tag, allow_status, allow_excluded, out, appendix_name(label), label_map=False)


_SYM = {"understeer": "US", "oversteer": "OS", "open-loop unstable": "INST"}
_SEV = {"moderate": "moderate", "strong": "strong"}


def short_verdict(text):
    """'understeer (moderate) at early exit; marginal' -> 'US moderate (E1)*'.
    The CSV keeps the full words; the table abbreviates them."""
    if text in ("ok", "-"):
        return "--"
    if text == "no CS data":
        return "no data"
    out = []
    for part in text.split("; "):
        if part == "marginal":
            out[-1] += "*"
            continue
        head, _at, phase = part.partition(" at ")
        word, _sp, sev = head.partition(" (")
        sev = _SEV.get(sev.rstrip(")"), "")
        out.append(f"{_SYM[word]}{' ' + sev if sev else ''} ({common.PHASE_ABBR[phase]})")
    return ", ".join(out)


def _fmt(v, fmt):
    # a value that rounds to zero prints without a sign (-0.00 -> 0.00)
    s = format(v, fmt)
    return format(0.0, fmt) if float(s) == 0 else s


def _tex_num(s):
    # only the sign in math mode: a real minus, not a hyphen, while the digits
    # stay in the text font like the positive values (author 2026-10-30)
    return f"$-${s[1:]}" if s.startswith("-") else s


def render(out=None, name=SCRIPT):
    """The .tex from the stored CSV only (no session is loaded): one line
    per corner, value with its phase abbreviation (B, T, A, E1, E2)."""
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    meta, c = common.read_csv(f"{data_dir}/{name}.csv")

    def cell(v, phase, fmt):
        v = common.num(v)
        return f"{_tex_num(_fmt(v, fmt))} ({common.PHASE_ABBR.get(phase, phase)})" if v == v else "--"
    body = []
    for i, cid in enumerate(c["corner_id"]):
        v = common.num(c["apex_speed_kmh"][i])
        # one decimal: a corner just below the 80 km/h class limit (class L)
        # would otherwise print as 80 (author 2026-10-30)
        body.append([cid, (c["speed_class"][i] or "-")[:1].upper(), f"{v:.1f}" if v == v else "--",
                     cell(c["cs_f_worst"][i], c["cs_f_phase"][i], ".2f"),
                     cell(c["cs_r_worst"][i], c["cs_r_phase"][i], ".2f"),
                     # stored unit N m/deg (stability_observed_Nm_per_deg), not converted
                     cell(c["dmz_dbeta_Nm_per_deg"][i], c["dmz_phase"][i], ".0f"),
                     common.tex_escape(short_verdict(c["verdict"][i]))])
    header = ["C", "Class", "$v_\\mathrm{min}$ in km/h", f"{common.CS_F} (phase)", f"{common.CS_R} (phase)",
              "$\\mathrm{d}M_z/\\mathrm{d}\\beta$ in N\\,m/deg (phase)", "Verdict"]
    note = None
    if "session_recommendation" in meta:
        note = ("\\parbox{\\linewidth}{\\small Session recommendation: "
                f"{common.tex_escape(meta['session_recommendation'])}. "
                f"Alternatives: {common.tex_escape(meta['alternatives'])}.}}")
    common.write_table(f"{plots_dir}/{name}.tex", SCRIPT, header, body, "r c r r r r l",
                       env="tabular", small=True, space_every=3, footnote=note)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--appendix", help="comma-separated outing IDs: one table per session")
    ap.add_argument("--appendix-labels", help="render-only: comma-separated session labels of the appendix tables")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSV only")
    a = ap.parse_args(argv)
    if a.render_only:
        if a.appendix_labels:
            for lab in a.appendix_labels.split(","):
                render(a.out, appendix_name(lab.strip()))
        else:
            render(a.out)
        return
    if a.appendix:
        build_appendix([int(x) for x in a.appendix.split(",")], a.tag, a.allow_status, a.allow_excluded, a.out)
        return
    if a.outing is None:
        ap.error("--outing is required unless --render-only or --appendix")
    build(a.outing, a.tag, a.allow_status, a.allow_excluded, a.out)


if __name__ == "__main__":
    main()
