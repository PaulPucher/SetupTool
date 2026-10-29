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
# Run: python -m tools.thesis_figures.tab_example_verdicts --outing 12 [--tag r2] [--allow-status]
#      [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "tab_example_verdicts"


def _worst(agg, getter):
    from modules.corner_aggregation import PHASE_KEYS
    best = (float("nan"), "-")
    for ph in PHASE_KEYS:
        v = getter(agg, ph)
        if v == v and (best[0] != best[0] or v < best[0]):
            best = (float(v), common.PHASE_LABELS[ph])
    return best


def build(outing, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.classifier import verdict_cs_stat
    from modules.corner_aggregation import aggregate_by_corner
    session = common.load_session(outing)
    payload, why = common.stored_payload(session)
    if payload is None:
        raise common.Refused(f"outing {outing}: {why}")
    marks = common.gate(outing, payload["fit_manifest"], payload["fallback_used"], allow_status, allow_excluded)
    label = common.session_label(session["row"])
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
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, [outing], tag, marks, session_label=label,
                                 apex_speed="corner speed v_min in km/h: median over the analysis laps of the "
                                            "stored apex_speed (minimum smoothed ecu_speed from turn-in to corner "
                                            "end; the column name is the stored field's)",
                                 dmz="cross-lap median per phase; minimum over the phases"))
    render(out)
    return cols


_SYM = {"understeer": "US", "oversteer": "OS", "open-loop unstable": "INST"}
_SEV = {"moderate": "mod.", "strong": "strong"}


def short_verdict(text):
    """'understeer (moderate) at early exit; marginal' -> 'US mod. (E1)*'.
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


def render(out=None):
    """The .tex from the stored CSV only (no session is loaded): one line
    per corner, value with its phase abbreviation (B, T, A, E1, E2)."""
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    _meta, c = common.read_csv(f"{data_dir}/{SCRIPT}.csv")

    def cell(v, phase, fmt):
        v = common.num(v)
        return f"{format(v, fmt)} ({common.PHASE_ABBR.get(phase, phase)})" if v == v else "--"
    body = []
    for i, cid in enumerate(c["corner_id"]):
        v = common.num(c["apex_speed_kmh"][i])
        body.append([cid, (c["speed_class"][i] or "-")[:1].upper(), f"{v:.0f}" if v == v else "--",
                     cell(c["cs_f_worst"][i], c["cs_f_phase"][i], ".2f"),
                     cell(c["cs_r_worst"][i], c["cs_r_phase"][i], ".2f"),
                     # stored unit N m/deg (stability_observed_Nm_per_deg), not converted
                     cell(c["dmz_dbeta_Nm_per_deg"][i], c["dmz_phase"][i], ".0f"),
                     common.tex_escape(short_verdict(c["verdict"][i]))])
    header = ["C", "Class", "$v_\\mathrm{min}$ in km/h", f"{common.CS_F} (phase)", f"{common.CS_R} (phase)",
              "$\\mathrm{d}M_z/\\mathrm{d}\\beta$ in N\\,m/deg (phase)", "Verdict"]
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, header, body, "r c r r r r l",
                       env="tabular", small=True, space_every=3)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--render-only", action="store_true", help="write the .tex from the existing CSV only")
    a = ap.parse_args(argv)
    if a.render_only:
        render(a.out)
        return
    if a.outing is None:
        ap.error("--outing is required unless --render-only")
    build(a.outing, a.tag, a.allow_status, a.allow_excluded, a.out)


if __name__ == "__main__":
    main()
